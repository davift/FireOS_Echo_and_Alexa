/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MT_EMI_API_H__
#define __MT_EMI_API_H__

/* macro for MPU */
#define EMI_MPU_DOMAIN_NUM	8
#define EMI_MPU_REGION_NUM	16
#define EMI_MPU_ALIGN_BITS	16

/* general macro */
#define MAX_CH		1
#define MAX_RK		1
#define DRAM_OFFSET	0x40000000

unsigned int get_dram_type(void);
unsigned int get_ch_num(void);
unsigned int get_rk_num(void);
unsigned int get_rank_size(unsigned int rank_index);
unsigned int get_emi_bwst(unsigned int bw_index);
unsigned int get_emi_bwvl(unsigned int bw_index);
extern void __iomem *mt_cen_emi_base_get(void);
extern void __iomem *mt_emi_base_get(void);
extern void __iomem *mt_chn_emi_base_get(unsigned int channel_index);
extern void __iomem *mt_emi_mpu_base_get(void);
extern void dump_last_bm(char *buf, unsigned int leng);

#include <mpu_v1.h>
#include <pasr_api_v1.h>

#endif /* __MT_EMI_API_H__ */

