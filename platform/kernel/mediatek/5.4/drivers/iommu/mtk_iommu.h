/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2016 MediaTek Inc.
 * Author: Honghui Zhang <honghui.zhang@mediatek.com>
 */

#ifndef _MTK_IOMMU_H_
#define _MTK_IOMMU_H_

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/io-pgtable.h>
#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <soc/mediatek/smi.h>
#include <dt-bindings/memory/mtk-smi-larb-port.h>

struct mtk_iommu_suspend_reg {
	u32				misc_ctrl;
	u32				dcm_dis;
	u32				ctrl_reg;
	u32				int_control0;
	u32				int_main_control;
	u32				ivrp_paddr;
	u32				vld_pa_rng;
	u32				wr_len;
};

enum mtk_iommu_plat {
	M4U_MT2701,
	M4U_MT2712,
	M4U_MT6779,
	M4U_MT6873,
	M4U_MT8173,
	M4U_MT8183,
	M4U_MT8519,
};

enum mtk_iommu_type {
	MTK_IOMMU_MM = 0,
	MTK_IOMMU_APU,
	MTK_IOMMU_INFRA,
};

struct mtk_iommu_iova_region;
struct mtk_iommu_resv_iova_region;

struct mtk_iommu_plat_data {
	enum mtk_iommu_plat m4u_plat;
	bool                has_4gb_mode;

	/* HW will use the EMI clock if there isn't the "bclk". */
	bool                has_bclk;
	bool		    has_misc_ctrl;
	bool		    has_sub_comm;
	bool                has_vld_pa_rng;
	bool                has_wr_len;
	bool                reset_axi;
	bool		    iova_34_en;
	bool		    is_apu;
	u32                 inv_sel_reg;
	unsigned char       larbid_remap[8][4];

	const unsigned int  iova_region_cnt;
	const struct mtk_iommu_iova_region	*iova_region;
	enum mtk_iommu_type type;
	struct list_head    *hw_list;
};

struct mtk_iommu_domain;

struct mtk_iommu_data {
	void __iomem			*base;
	int				irq;
	struct device			*dev;
	struct clk			*bclk;
	phys_addr_t			protect_base; /* protect memory base */
	struct mtk_iommu_suspend_reg	reg;
	struct mtk_iommu_domain		*m4u_dom;
	struct iommu_group		*m4u_group[MTK_M4U_DOM_NR_MAX];
	bool                            enable_4GB;
	spinlock_t			tlb_lock; /* lock for tlb range flush */

	struct iommu_device		iommu;
	const struct mtk_iommu_plat_data *plat_data;
	struct device			*smicomm_dev;

	unsigned int			cur_domid;
	struct list_head		list;
	struct mtk_smi_larb_iommu	larb_imu[MTK_LARB_NR_MAX];
};

static inline int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static inline void release_of(struct device *dev, void *data)
{
	of_node_put(data);
}

static inline int mtk_iommu_bind(struct device *dev)
{
	struct mtk_iommu_data *data = dev_get_drvdata(dev);

	return component_bind_all(dev, &data->larb_imu);
}

static inline void mtk_iommu_unbind(struct device *dev)
{
	struct mtk_iommu_data *data = dev_get_drvdata(dev);

	component_unbind_all(dev, &data->larb_imu);
}

#endif
