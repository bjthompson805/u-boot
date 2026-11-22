// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2021 Mark Kettenis <kettenis@openbsd.org>
 */

#include <dm.h>
#include <mailbox.h>
#include <mapmem.h>
#include "nvme.h"
#include <reset.h>

#include <cpu_func.h>
#include <asm/io.h>
#include <asm/arch/rtkit.h>
#include <asm/arch/sart.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>

/* ASC registers */
#define REG_CPU_CTRL		0x0044
#define  REG_CPU_CTRL_RUN	BIT(4)

/* Apple NVMe registers */
#define ANS_IOSQ_BASE		0x01200
#define ANS_IOCQ_BASE		0x01208
#define ANS_MAX_PEND_CMDS_CTRL	0x01210
#define  ANS_MAX_QUEUE_DEPTH	64
#define ANS_BOOT_STATUS		0x01300
#define  ANS_BOOT_STATUS_OK	0xde71ce55
#define ANS_MODESEL		0x01304
#define ANS_UNKNOWN_CTRL	0x24008
#define  ANS_PRP_NULL_CHECK	(1 << 11)
#define ANS_LINEAR_SQ_CTRL	0x24908
#define  ANS_LINEAR_SQ_CTRL_EN	(1 << 0)
#define ANS_ASQ_DB		0x2490c
#define ANS_IOSQ_DB		0x24910
#define ANS_NVMMU_NUM		0x28100
#define ANS_NVMMU_BASE_ASQ	0x28108
#define ANS_NVMMU_BASE_IOSQ	0x28110
#define ANS_NVMMU_TCB_INVAL	0x28118
#define ANS_NVMMU_TCB_STAT	0x28120

#define ANS_NVMMU_TCB_SIZE	0x4000
#define ANS_NVMMU_TCB_PITCH	0x80

/*
 * The Apple NVMe controller includes an IOMMU known as NVMMU.  The
 * NVMMU is programmed through an array of TCBs. These TCBs are paired
 * with the corresponding slot in the submission queues and need to be
 * configured with the command details before a command is allowed to
 * execute. This is necessary even for commands that don't do DMA.
 */
struct ans_nvmmu_tcb {
	u8 opcode;
	u8 flags;
	u8 slot;
	u8 pad0;
	u32 prpl_len;
	u8 pad1[16];
	u64 prp1;
	u64 prp2;
};

#define ANS_NVMMU_TCB_FROM_DEV	BIT(0)
#define ANS_NVMMU_TCB_TO_DEV	BIT(1)

enum apple_nvme_type {
	APPLE_NVME_LEGACY,
	APPLE_NVME_SPLIT_NVMMU,
};

struct apple_nvme_priv {
	struct nvme_dev ndev;
	void *base;		/* NVMe registers */
	void *nvmmu;		/* Linear SQ and TCB registers */
	bool split_nvmmu;
	void *asc;		/* ASC registers */
	struct reset_ctl_bulk resets; /* ASC reset */
	struct mbox_chan chan;
	struct apple_sart *sart;
	struct apple_rtkit *rtk;
	struct ans_nvmmu_tcb *tcbs[NVME_Q_NUM]; /* Submission queue TCBs */
	u32 __iomem *q_db[NVME_Q_NUM]; /* Submission queue doorbell */
};

static int apple_nvme_setup_queue(struct nvme_queue *nvmeq)
{
	struct apple_nvme_priv *priv =
		container_of(nvmeq->dev, struct apple_nvme_priv, ndev);

	switch (nvmeq->qid) {
	case NVME_ADMIN_Q:
	case NVME_IO_Q:
		break;
	default:
		return -EINVAL;
	}

	priv->tcbs[nvmeq->qid] = (void *)memalign(priv->split_nvmmu ? SZ_16K : SZ_4K,
					  ANS_NVMMU_TCB_SIZE);
	if (!priv->tcbs[nvmeq->qid])
		return -ENOMEM;

	memset((void *)priv->tcbs[nvmeq->qid], 0, ANS_NVMMU_TCB_SIZE);

	switch (nvmeq->qid) {
	case NVME_ADMIN_Q:
		priv->q_db[nvmeq->qid] =
			priv->nvmmu + ANS_ASQ_DB;
		nvme_writeq((ulong)priv->tcbs[nvmeq->qid],
			    priv->nvmmu + ANS_NVMMU_BASE_ASQ);
		break;
	case NVME_IO_Q:
		priv->q_db[nvmeq->qid] =
			priv->nvmmu + ANS_IOSQ_DB;
		nvme_writeq((ulong)priv->tcbs[nvmeq->qid],
			    priv->nvmmu + ANS_NVMMU_BASE_IOSQ);
		break;
	}

	return 0;
}

static void apple_nvme_submit_cmd(struct nvme_queue *nvmeq,
				  struct nvme_command *cmd)
{
	struct apple_nvme_priv *priv =
		container_of(nvmeq->dev, struct apple_nvme_priv, ndev);
	struct ans_nvmmu_tcb *tcb;
	u16 tail = nvmeq->sq_tail;

	tcb = ((void *)priv->tcbs[nvmeq->qid]) + tail * ANS_NVMMU_TCB_PITCH;
	memset(tcb, 0, sizeof(*tcb));
	if (!priv->split_nvmmu) {
		tcb->opcode = cmd->common.opcode;
		tcb->flags = ANS_NVMMU_TCB_FROM_DEV | ANS_NVMMU_TCB_TO_DEV;
	} else if (cmd->common.prp1) {
		tcb->flags = cmd->common.opcode & 1 ?
			ANS_NVMMU_TCB_TO_DEV : ANS_NVMMU_TCB_FROM_DEV;
	}
	tcb->slot = tail;
	tcb->prpl_len = cmd->rw.length;
	tcb->prp1 = cmd->common.prp1;
	tcb->prp2 = cmd->common.prp2;

	flush_dcache_range((ulong)tcb, (ulong)tcb + ANS_NVMMU_TCB_PITCH);
	if (priv->split_nvmmu)
		apple_rtkit_poll(priv->rtk, 0);
	writel(tail, priv->q_db[nvmeq->qid]);
}

static void apple_nvme_complete_cmd(struct nvme_queue *nvmeq,
				    struct nvme_command *cmd)
{
	struct apple_nvme_priv *priv =
		container_of(nvmeq->dev, struct apple_nvme_priv, ndev);
	struct ans_nvmmu_tcb *tcb;
	u16 tail = nvmeq->sq_tail;

	tcb = ((void *)priv->tcbs[nvmeq->qid]) + tail * ANS_NVMMU_TCB_PITCH;
	memset(tcb, 0, sizeof(*tcb));
	writel(tail, priv->nvmmu + ANS_NVMMU_TCB_INVAL);
	readl(priv->nvmmu + ANS_NVMMU_TCB_STAT);

	if (++tail == nvmeq->q_depth)
		tail = 0;
	nvmeq->sq_tail = tail;
}

static int nvme_shmem_setup(void *cookie, struct apple_rtkit_buffer *buf)
{
	struct apple_nvme_priv *priv = (struct apple_nvme_priv *)cookie;

	if (!buf || buf->dva || !buf->size)
		return -1;

	buf->buffer = memalign(SZ_16K, ALIGN(buf->size, SZ_16K));
	if (!buf->buffer)
		return -ENOMEM;

	if (!sart_add_allowed_region(priv->sart, buf->buffer, buf->size)) {
		free(buf->buffer);
		buf->buffer = NULL;
		buf->size = 0;
		return -1;
	}

	buf->dva = (u64)buf->buffer;

	return 0;
}

static void nvme_shmem_destroy(void *cookie, struct apple_rtkit_buffer *buf)
{
	struct apple_nvme_priv *priv = (struct apple_nvme_priv *)cookie;

	if (!buf)
		return;

	if (buf->buffer) {
		sart_remove_allowed_region(priv->sart, buf->buffer, buf->size);
		free(buf->buffer);
		buf->buffer = NULL;
		buf->size = 0;
		buf->dva = 0;
	}
}

static int apple_nvme_probe(struct udevice *dev)
{
	struct apple_nvme_priv *priv = dev_get_priv(dev);
	fdt_addr_t addr;
	ofnode of_sart;
	u32 ctrl, stat, phandle, queue_limit;
	int ret;

	priv->base = dev_read_addr_ptr(dev);
	if (!priv->base)
		return -EINVAL;

	addr = dev_read_addr_index(dev, 1);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;
	priv->asc = map_sysmem(addr, 0);

	priv->split_nvmmu = dev_get_driver_data(dev) == APPLE_NVME_SPLIT_NVMMU;
	priv->nvmmu = priv->base;
	if (priv->split_nvmmu) {
		addr = dev_read_addr_name(dev, "nvmmu");
		if (addr == FDT_ADDR_T_NONE)
			return -EINVAL;
		priv->nvmmu = map_sysmem(addr, 0);
	}

	ret = reset_get_bulk(dev, &priv->resets);
	if (ret < 0)
		return ret;

	ret = mbox_get_by_index(dev, 0, &priv->chan);
	if (ret < 0)
		return ret;

	ret = dev_read_u32(dev, "apple,sart", &phandle);
	if (ret < 0)
		return ret;

	of_sart = ofnode_get_by_phandle(phandle);
	priv->sart = sart_init(of_sart);
	if (!priv->sart)
		return -EINVAL;

	ctrl = readl(priv->asc + REG_CPU_CTRL);
	writel(ctrl | REG_CPU_CTRL_RUN, priv->asc + REG_CPU_CTRL);

	priv->rtk = apple_rtkit_init(&priv->chan, priv, nvme_shmem_setup, nvme_shmem_destroy);
	if (!priv->rtk)
		return -ENOMEM;

	ret = apple_rtkit_boot_state(priv->rtk, priv->split_nvmmu ?
				     APPLE_RTKIT_PWR_STATE_INIT :
				     APPLE_RTKIT_PWR_STATE_ON);
	printf("apple_nvme: RTKit boot status %d\n", ret);
	if (ret < 0) {
		printf("%s: NVMe apple_rtkit_boot returned: %d\n", __func__, ret);
		return ret;
	}

	if (priv->split_nvmmu) {
		ret = apple_rtkit_set_ap_power(priv->rtk, APPLE_RTKIT_PWR_STATE_ON);
		printf("apple_nvme: AP power status %d\n", ret);
		if (ret)
			return ret;
	}

	ret = readl_poll_sleep_timeout(priv->base + ANS_BOOT_STATUS, stat,
				       (stat == ANS_BOOT_STATUS_OK), 100,
				       500000);
	if (ret < 0) {
		printf("%s: NVMe firmware didn't boot\n", __func__);
		return -ETIMEDOUT;
	}

	queue_limit = ANS_MAX_QUEUE_DEPTH - priv->split_nvmmu;

	if (!priv->split_nvmmu)
		writel(ANS_LINEAR_SQ_CTRL_EN, priv->nvmmu + ANS_LINEAR_SQ_CTRL);
	writel((queue_limit << 16) | queue_limit,
	       priv->base + ANS_MAX_PEND_CMDS_CTRL);

	if (!priv->split_nvmmu)
		clrbits_le32(priv->base + ANS_UNKNOWN_CTRL, ANS_PRP_NULL_CHECK);

	strcpy(priv->ndev.vendor, "Apple");

	writel(priv->split_nvmmu ? ANS_MAX_QUEUE_DEPTH - 1 :
	       (ANS_NVMMU_TCB_SIZE / ANS_NVMMU_TCB_PITCH) - 1,
	       priv->nvmmu + ANS_NVMMU_NUM);
	if (!priv->split_nvmmu)
		writel(0, priv->base + ANS_MODESEL);

	if (priv->split_nvmmu) {
		priv->ndev.fixed_io_queue = true;
		priv->ndev.sq_slot_is_cid = true;
		priv->ndev.queue_depth_override = ANS_MAX_QUEUE_DEPTH;
		priv->ndev.queue_alignment = SZ_16K;
	}
	priv->ndev.bar = priv->base;
	ret = nvme_init(dev);
	if (ret && priv->split_nvmmu) {
		printf("apple_nvme: ASQ doorbell %#x LSQ %#x NVMMU entries %#x TCB base %#llx\n",
		       readl(priv->nvmmu + ANS_ASQ_DB),
		       readl(priv->nvmmu + ANS_LINEAR_SQ_CTRL),
		       readl(priv->nvmmu + ANS_NVMMU_NUM),
		       nvme_readq(priv->nvmmu + ANS_NVMMU_BASE_ASQ));
		printf("apple_nvme: RTKit poll status %d\n",
		       apple_rtkit_poll(priv->rtk, 0));
	}
	return ret;
}

static int apple_nvme_remove(struct udevice *dev)
{
	struct apple_nvme_priv *priv = dev_get_priv(dev);
	u32 ctrl;

	nvme_shutdown(dev);

	apple_rtkit_shutdown(priv->rtk, APPLE_RTKIT_PWR_STATE_SLEEP);

	ctrl = readl(priv->asc + REG_CPU_CTRL);
	writel(ctrl & ~REG_CPU_CTRL_RUN, priv->asc + REG_CPU_CTRL);

	apple_rtkit_free(priv->rtk);
	priv->rtk = NULL;

	sart_free(priv->sart);
	priv->sart = NULL;

	reset_assert_bulk(&priv->resets);
	/* Match the reset pulse used by m1n1 on the split-NVMMU controller. */
	if (priv->split_nvmmu)
		udelay(10);
	reset_deassert_bulk(&priv->resets);

	return 0;
}

static void apple_nvme_io_queue_created(struct nvme_queue *nvmeq)
{
	struct apple_nvme_priv *priv =
		container_of(nvmeq->dev, struct apple_nvme_priv, ndev);

	/* M5 firmware needs the queue addresses in the ANS vendor registers. */
	if (priv->split_nvmmu) {
		nvme_writeq((ulong)nvmeq->cqes, priv->base + ANS_IOCQ_BASE);
		nvme_writeq((ulong)nvmeq->sq_cmds, priv->base + ANS_IOSQ_BASE);
	}
}

static const struct nvme_ops apple_nvme_ops = {
	.setup_queue = apple_nvme_setup_queue,
	.io_queue_created = apple_nvme_io_queue_created,
	.submit_cmd = apple_nvme_submit_cmd,
	.complete_cmd = apple_nvme_complete_cmd,
};

static const struct udevice_id apple_nvme_ids[] = {
	{ .compatible = "apple,t8142-nvme-ans2", .data = APPLE_NVME_SPLIT_NVMMU },
	{ .compatible = "apple,t8140-nvme-ans2", .data = APPLE_NVME_SPLIT_NVMMU },
	{ .compatible = "apple,t8103-nvme-ans2" },
	{ .compatible = "apple,nvme-ans2" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(apple_nvme) = {
	.name = "apple_nvme",
	.id = UCLASS_NVME,
	.of_match = apple_nvme_ids,
	.priv_auto = sizeof(struct apple_nvme_priv),
	.probe = apple_nvme_probe,
	.remove = apple_nvme_remove,
	.ops = &apple_nvme_ops,
	.flags = DM_FLAG_OS_PREPARE,
};
