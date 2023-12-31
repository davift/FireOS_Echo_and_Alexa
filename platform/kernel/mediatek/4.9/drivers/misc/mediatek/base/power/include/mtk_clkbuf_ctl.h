/*
 * Copyright (C) 2017 MediaTek Inc.
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

/**
 * @file    mtk_clk_buf_ctl.h
 * @brief   Driver for clock buffer control
 *
 */
#ifndef __MTK_CLK_BUF_CTL_H__
#define __MTK_CLK_BUF_CTL_H__

#include <linux/kernel.h>
#include <linux/mutex.h>
#include "mtk_clkbuf_hw.h"

enum CLK_BUF_SWCTRL_STATUS_T {
	CLK_BUF_SW_DISABLE = 0,
	CLK_BUF_SW_ENABLE  = 1,
};

#define CLKBUF_NUM      XO_NUMBER

#define STA_CLK_ON      1
#define STA_CLK_OFF     0

int clk_buf_init(void);
bool clk_buf_ctrl(enum clk_buf_id id, bool onoff);
void clk_buf_get_swctrl_status(enum CLK_BUF_SWCTRL_STATUS_T *status);
void clk_buf_get_rf_drv_curr(void *rf_drv_curr);
void clk_buf_set_by_flightmode(bool is_flightmode_on);
void clk_buf_save_afc_val(unsigned int afcdac);
void clk_buf_write_afcdac(void);
void clk_buf_control_bblpm(bool on);
u32 clk_buf_bblpm_enter_cond(void);
void clk_buf_dump_clkbuf_log(void);
bool is_clk_buf_under_flightmode(void);
bool is_clk_buf_from_pmic(void);

#endif

