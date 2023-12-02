// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (c) 2021 MediaTek Inc.
// Author:Wentao He <wentao.he@mediatek.com>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "clk-mtk.h"
#include "clk-mux.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt8519-clk.h>

static DEFINE_SPINLOCK(mt8519_clk_lock);

static const struct mtk_fixed_factor top_divs[] = {
	FACTOR(CLK_TOP_SYSPLL_D2, "syspll_d2", "mainpll", 1, 2),
	FACTOR(CLK_TOP_SYSPLL1_D2, "syspll1_d2", "syspll_d2", 1, 2),
	FACTOR(CLK_TOP_SYSPLL1_D4, "syspll1_d4", "syspll_d2", 1, 4),
	FACTOR(CLK_TOP_SYSPLL1_D8, "syspll1_d8", "syspll_d2", 1, 8),
	FACTOR(CLK_TOP_SYSPLL_D3, "syspll_d3", "mainpll", 1, 3),
	FACTOR(CLK_TOP_SYSPLL2_D2, "syspll2_d2", "syspll_d3", 1, 2),
	FACTOR(CLK_TOP_SYSPLL2_D4, "syspll2_d4", "syspll_d3", 1, 4),
	FACTOR(CLK_TOP_SYSPLL2_D8, "syspll2_d8", "syspll_d3", 1, 8),
	FACTOR(CLK_TOP_SYSPLL_D5, "syspll_d5", "mainpll", 1, 5),
	FACTOR(CLK_TOP_SYSPLL3_D4, "syspll3_d4", "syspll_d5", 1, 4),
	FACTOR(CLK_TOP_SYSPLL_D7, "syspll_d7", "mainpll", 1, 7),
	FACTOR(CLK_TOP_SYSPLL4_D2, "syspll4_d2", "syspll_d7", 1, 2),
	FACTOR(CLK_TOP_UNIVPLL_D4, "univpll_d4", "univpll", 1, 4),
	FACTOR(CLK_TOP_UNIVPLL1_D2, "univpll1_d2", "univpll_d4", 1, 2),
	FACTOR(CLK_TOP_UNIVPLL1_D4, "univpll1_d4", "univpll_d4", 1, 4),
	FACTOR(CLK_TOP_UNIVPLL1_D8, "univpll1_d8", "univpll_d4", 1, 8),
	FACTOR(CLK_TOP_UNIVPLL_D6, "univpll_d6", "univpll", 1, 6),
	FACTOR(CLK_TOP_UNIVPLL2_D2, "univpll2_d2", "univpll_d6", 1, 2),
	FACTOR(CLK_TOP_UNIVPLL2_D4, "univpll2_d4", "univpll_d6", 1, 4),
	FACTOR(CLK_TOP_UNIVPLL2_D8, "univpll2_d8", "univpll_d6", 1, 8),
	FACTOR(CLK_TOP_UNIVPLL_D5, "univpll_d5", "univpll", 1, 5),
	FACTOR(CLK_TOP_UNIVPLL3_D4, "univpll3_d4", "univpll_d5", 1, 4),
	FACTOR(CLK_TOP_UNIVPLL3_D8, "univpll3_d8", "univpll_d5", 1, 8),
	FACTOR(CLK_TOP_SYS_26M_D2, "sys_26m_d2", "clk26m", 1, 2),
	FACTOR(CLK_TOP_CLKRTC_INT, "clkrtc_int", "clk26m", 1, 813),
	FACTOR(CLK_TOP_MSDCPLL, "msdcpll_ck", "msdcpll", 1, 1),
	FACTOR(CLK_TOP_MSDCPLL_D2, "msdcpll_d2", "msdcpll", 1, 2),
	FACTOR(CLK_TOP_MSDCPLL_D4, "msdcpll_d4", "msdcpll", 1, 4),
	FACTOR(CLK_TOP_APLL1, "apll1_ck", "apll1", 1, 1),
	FACTOR(CLK_TOP_APLL1_D2, "apll1_d2", "apll1", 1, 2),
	FACTOR(CLK_TOP_APLL1_D3, "apll1_d3", "apll1", 1, 3),
	FACTOR(CLK_TOP_APLL1_D4, "apll1_d4", "apll1", 1, 4),
	FACTOR(CLK_TOP_APLL1_D8, "apll1_d8", "apll1", 1, 8),
	FACTOR(CLK_TOP_APLL1_D16, "apll1_d16", "apll1", 1, 16),
	FACTOR(CLK_TOP_APLL2, "apll2_ck", "apll2", 1, 1),
	FACTOR(CLK_TOP_APLL2_D2, "apll2_d2", "apll2", 1, 2),
	FACTOR(CLK_TOP_APLL2_D3, "apll2_d3", "apll2", 1, 3),
	FACTOR(CLK_TOP_APLL2_D4, "apll2_d4", "apll2", 1, 4),
	FACTOR(CLK_TOP_APLL2_D8, "apll2_d8", "apll2", 1, 8),
	FACTOR(CLK_TOP_APLL2_D16, "apll2_d16", "apll2", 1, 16),
	FACTOR(CLK_TOP_MPLL_D2, "mpll_d2", "mpll", 1, 2),
	FACTOR(CLK_TOP_MPLL_D4, "mpll_d4", "mpll", 1, 4),
	FACTOR(CLK_TOP_DSPPLL, "dsppll_ck", "dsppll", 1, 1),
	FACTOR(CLK_TOP_DSPPLL_D2, "dsppll_d2", "dsppll", 1, 2),
	FACTOR(CLK_TOP_DSPPLL_D4, "dsppll_d4", "dsppll", 1, 4),
	FACTOR(CLK_TOP_IPPLL, "ippll_ck", "ippll", 1, 1),
	FACTOR(CLK_TOP_IPPLL_D2, "ippll_d2", "ippll", 1, 2),
	FACTOR(CLK_TOP_IPPLL_D3, "ippll_d3", "ippll", 1, 3),
	FACTOR(CLK_TOP_IPPLL_D4, "ippll_d4", "ippll", 1, 4),
	FACTOR(CLK_TOP_IPPLL_D8, "ippll_d8", "ippll", 1, 8),
	FACTOR(CLK_TOP_NNAPLL, "nnapll_ck", "nnapll", 1, 1),
	FACTOR(CLK_TOP_NNAPLL_D2, "nnapll_d2", "nnapll", 1, 2),
	FACTOR(CLK_TOP_NNAPLL_D4, "nnapll_d4", "nnapll", 1, 4),
	FACTOR(CLK_TOP_NNAPLL_D8, "nnapll_d8", "nnapll", 1, 8),
	FACTOR(CLK_TOP_BIST2FPC, "bist2fpc_ck", "univpll", 1, 12),
};

static const char * const axi_parents[] = {
	"clk26m",
	"syspll1_d4",
	"syspll_d7",
	"univpll1_d8",
	"sys_26m_d2",
	"clkrtc_sel"
};

static const char * const mem_parents[] = {
	"clk26m",
	"dsppll_d2",
	"ippll_d2",
	"univpll_d6"
};

static const char * const uart_parents[] = {
	"clk26m",
	"univpll2_d8"
};

static const char * const spi_parents[] = {
	"clk26m",
	"univpll2_d2",
	"syspll2_d2",
	"univpll1_d4",
	"syspll1_d4",
	"univpll3_d4",
	"univpll2_d4",
	"syspll4_d2"
};

static const char * const spis_parents[] = {
	"clk26m",
	"univpll_d6",
	"syspll_d3",
	"univpll1_d2",
	"univpll2_d2",
	"univpll1_d4",
	"univpll2_d4",
	"syspll4_d2"
};

static const char * const msdc50_0_h_parents[] = {
	"clk26m",
	"syspll1_d2",
	"univpll1_d4",
	"syspll2_d2"
};

static const char * const rv33_parents[] = {
	"clk26m",
	"univpll_d5",
	"univpll_d6",
	"syspll_d3",
	"univpll1_d2",
	"syspll1_d2",
	"syspll_d5"
};

static const char * const msdc50_0_parents[] = {
	"clk26m",
	"msdcpll_ck",
	"univpll_d6",
	"univpll1_d2",
	"syspll1_d2",
	"msdcpll_d2",
	"syspll2_d2",
	"univpll1_d4"
};

static const char * const msdc30_1_parents[] = {
	"clk26m",
	"msdcpll_d2",
	"univpll2_d2",
	"syspll2_d2",
	"univpll1_d4",
	"syspll1_d4",
	"syspll2_d4",
	"univpll2_d8"
};

static const char * const audio_parents[] = {
	"clk26m",
	"univpll2_d8",
	"apll1_d4",
	"apll2_d4"
};

static const char * const aud_intbus_parents[] = {
	"clk26m",
	"syspll1_d4",
	"univpll3_d4",
	"apll2_d3",
	"univpll_d6",
	"syspll_d3",
	"apll1_d3"
};

static const char * const apll1_parents[] = {
	"clk26m",
	"apll1_ck",
	"apll1_d2",
	"apll1_d3",
	"apll1_d4",
	"apll1_d8",
	"apll1_d16",
	"sys_26m_d2"
};

static const char * const apll2_parents[] = {
	"clk26m",
	"apll2_ck",
	"apll2_d2",
	"apll2_d3",
	"apll2_d4",
	"apll2_d8",
	"apll2_d16",
	"sys_26m_d2"
};

static const char * const a2sys_parents[] = {
	"clk26m",
	"apll1_ck",
	"apll1_d2",
	"apll1_d3",
	"apll1_d4",
	"apll1_d8",
	"apll1_d16",
	"sys_26m_d2"
};

static const char * const a1sys_parents[] = {
	"clk26m",
	"apll2_ck",
	"apll2_d2",
	"apll2_d3",
	"apll2_d4",
	"apll2_d8",
	"apll2_d16",
	"sys_26m_d2"
};

static const char * const asm_l_parents[] = {
	"clk26m",
	"univpll2_d4",
	"univpll2_d2",
	"syspll_d5"
};

static const char * const asm_m_parents[] = {
	"clk26m",
	"univpll2_d4",
	"univpll2_d2",
	"syspll_d5"
};

static const char * const asm_h_parents[] = {
	"clk26m",
	"univpll2_d4",
	"univpll2_d2",
	"syspll_d5"
};

static const char * const ud_spdif_in_parents[] = {
	"clk26m",
	"univpll_d4",
	"syspll_d2"
};

static const char * const aud_1_parents[] = {
	"clk26m",
	"apll1_ck"
};

static const char * const aud_2_parents[] = {
	"clk26m",
	"apll2_ck"
};

static const char * const ssusb_sys_parents[] = {
	"clk26m",
	"univpll3_d8",
	"univpll2_d4",
	"univpll3_d4"
};

static const char * const ssusb_xhci_parents[] = {
	"clk26m",
	"univpll3_d8",
	"univpll2_d4",
	"univpll3_d4"
};

static const char * const spm_parents[] = {
	"clk26m",
	"syspll1_d8"
};

static const char * const i2c_ao_parents[] = {
	"clk26m",
	"sys_26m_d2",
	"univpll3_d8",
	"univpll3_d4",
	"syspll2_d8"
};

static const char * const pwm_parents[] = {
	"clk26m",
	"univpll3_d8",
	"univpll2_d4",
	"sys_26m_d2",
	"clkrtc_sel"
};

static const char * const dsp_parents[] = {
	"clk26m",
	"dsppll_ck",
	"dsppll_d2",
	"dsppll_d4",
	"univpll1_d2",
	"apll2_d4",
	"sys_26m_d2",
	"clkrtc_sel"
};

static const char * const nfi1x_parents[] = {
	"clk26m",
	"univpll3_d4",
	"syspll4_d2",
	"syspll2_d4",
	"univpll2_d4",
	"syspll1_d4",
	"syspll_d7",
	"syspll2_d2"
};

static const char * const spinfi_bclk_parents[] = {
	"clk26m",
	"univpll2_d8",
	"univpll3_d8",
	"syspll1_d8",
	"syspll4_d2",
	"syspll2_d4",
	"univpll2_d4",
	"univpll3_d4"
};

static const char * const ecc_parents[] = {
	"clk26m",
	"syspll_d5",
	"syspll_d3",
	"univpll_d6"
};

static const char * const gcpu_parents[] = {
	"clk26m",
	"syspll_d3",
	"univpll1_d2",
	"syspll1_d2",
	"univpll2_d2"
};

static const char * const ip0_nna_parents[] = {
	"clk26m",
	"dsppll_ck",
	"dsppll_d2",
	"dsppll_d4",
	"ippll_ck",
	"nnapll_ck",
	"ippll_d2",
	"msdcpll_d4",
	"univpll_d4",
	"univpll_d6",
	"univpll2_d2",
	"ippll_d4",
	"ippll_d8"
};

static const char * const ip1_nna_parents[] = {
	"clk26m",
	"dsppll_ck",
	"dsppll_d2",
	"dsppll_d4",
	"nnapll_ck",
	"ippll_ck",
	"nnapll_d2",
	"msdcpll_d4",
	"univpll_d4",
	"univpll_d6",
	"univpll2_d2",
	"nnapll_d4",
	"nnapll_d8"
};

static const char * const adsp_bus_parents[] = {
	"clk26m",
	"ulposc1_d2",
	"syspll_d5",
	"syspll1_d2",
	"syspll_d3",
	"syspll_d2",
	"univpll_d6"
};

static const char * const cupm_parents[] = {
	"clk26m",
	"syspll2_d2",
	"syspll2_d4",
	"univpll1_d8"
};

static const char * const i2c_pwr_parents[] = {
	"clk26m",
	"sys_26m_d2",
	"univpll3_d8",
	"univpll3_d4",
	"syspll2_d8"
};

static const char * const clkrtc_parents[] = {
	"clkrtc_int",
	"clkrtc_ext"
};

static const char * const i2si1_mck_pre_parents[] = {
	"aud_1_sel",
	"aud_2_sel"
};

static const char * const tdmin_mck_pre_parents[] = {
	"aud_1_sel",
	"aud_2_sel"
};

static const char * const i2so1_mck_pre_parents[] = {
	"aud_1_sel",
	"aud_2_sel"
};

static const struct mtk_mux top_mtk_muxes[] = {
	/* CLK_CFG_0 */
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_TOP_AXI_SEL, "axi_sel",
		axi_parents, 0x040, 0x044, 0x048, 0, 3, 7, 0x4, 0, CLK_IS_CRITICAL),
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_TOP_MEM_SEL, "mem_sel",
		mem_parents, 0x040, 0x044, 0x048, 8, 2, 15, 0x4, 1, CLK_IS_CRITICAL),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_UART_SEL, "uart_sel",
		uart_parents, 0x040, 0x044, 0x048, 16, 1, 23, 0x4, 2),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SPI_SEL, "spi_sel",
		spi_parents, 0x040, 0x044, 0x048, 24, 3, 31, 0x4, 3),
	/* CLK_CFG_1 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SPIS_SEL, "spis_sel",
		spis_parents, 0x050, 0x054, 0x058, 0, 3, 7, 0x4, 4),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC50_0_HCLK_SEL, "msdc50_0_h_sel",
		msdc50_0_h_parents, 0x050, 0x054, 0x058, 8, 2, 15, 0x4, 5),
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_TOP_RV33_SEL, "rv33_sel",
		rv33_parents, 0x050, 0x054, 0x058, 16, 3, 23, 0x4, 6, CLK_IS_CRITICAL),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC50_0_SEL, "msdc50_0_sel",
		msdc50_0_parents, 0x050, 0x054, 0x058, 24, 3, 31, 0x4, 7),
	/* CLK_CFG_2 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_MSDC30_1_SEL, "msdc30_1_sel",
		msdc30_1_parents, 0x060, 0x064, 0x068, 8, 3, 15, 0x4, 9),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUDIO_SEL, "audio_sel",
		audio_parents, 0x060, 0x064, 0x068, 16, 2, 23, 0x4, 10),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_INTBUS_SEL, "aud_intbus_sel",
		aud_intbus_parents, 0x060, 0x064, 0x068, 24, 3, 31, 0x4, 11),
	/* CLK_CFG_3 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_APLL1_SEL, "apll1_sel",
		apll1_parents, 0x070, 0x074, 0x078, 0, 3, 7, 0x4, 12),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_APLL2_SEL, "apll2_sel",
		apll2_parents, 0x070, 0x074, 0x078, 8, 3, 15, 0x4, 13),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_A2SYS_SEL, "a2sys_sel",
		a2sys_parents, 0x070, 0x074, 0x078, 16, 3, 23, 0x4, 14),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_A1SYS_SEL, "a1sys_sel",
		a1sys_parents, 0x070, 0x074, 0x078, 24, 3, 31, 0x4, 15),
	/* CLK_CFG_4 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ASM_L_SEL, "asm_l_sel",
		asm_l_parents, 0x080, 0x084, 0x088, 0, 2, 7, 0x4, 16),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ASM_M_SEL, "asm_m_sel",
		asm_m_parents, 0x080, 0x084, 0x088, 8, 2, 15, 0x4, 17),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ASM_H_SEL, "asm_h_sel",
		asm_h_parents, 0x080, 0x084, 0x088, 16, 2, 23, 0x4, 18),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_UD_SPDIF_IN_SEL, "ud_spdif_in_sel",
		ud_spdif_in_parents, 0x080, 0x084, 0x088, 24, 2, 31, 0x4, 19),
	/* CLK_CFG_5 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_1_SEL, "aud_1_sel",
		aud_1_parents, 0x090, 0x094, 0x098, 0, 1, 7, 0x4, 20),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_AUD_2_SEL, "aud_2_sel",
		aud_2_parents, 0x090, 0x094, 0x098, 8, 1, 15, 0x4, 21),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SSUSB_SYS_SEL, "ssusb_sys_sel",
		ssusb_sys_parents, 0x090, 0x094, 0x098, 16, 2, 23, 0x4, 22),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SSUSB_XHCI_SEL, "ssusb_xhci_sel",
		ssusb_xhci_parents, 0x090, 0x094, 0x098, 24, 2, 31, 0x4, 23),
	/* CLK_CFG_6 */
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_TOP_SPM_SEL, "spm_sel",
		spm_parents, 0x0a0, 0x0a4, 0x0a8, 0, 1, 7, 0x4, 24, CLK_IS_CRITICAL),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_I2C_AO_SEL, "i2c_ao_sel",
		i2c_ao_parents, 0x0a0, 0x0a4, 0x0a8, 8, 3, 15, 0x4, 25),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_PWM_SEL, "pwm_sel",
		pwm_parents, 0x0a0, 0x0a4, 0x0a8, 16, 3, 23, 0x4, 26),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_DSP_SEL, "dsp_sel",
		dsp_parents, 0x0a0, 0x0a4, 0x0a8, 24, 3, 31, 0x4, 27),
	/* CLK_CFG_7 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_NFI1X_SEL, "nfi1x_sel",
		nfi1x_parents, 0x0b0, 0x0b4, 0x0b8, 0, 3, 7, 0x4, 28),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_SPINFI_BCLK_SEL, "spinfi_bclk_sel",
		spinfi_bclk_parents, 0x0b0, 0x0b4, 0x0b8, 8, 3, 15, 0x4, 29),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ECC_SEL, "ecc_sel",
		ecc_parents, 0x0b0, 0x0b4, 0x0b8, 16, 2, 23, 0x4, 30),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_GCPU_SEL, "gcpu_sel",
		gcpu_parents, 0x0b0, 0x0b4, 0x0b8, 24, 3, 31, 0x8, 0),
	/* CLK_CFG_8 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IP0_NNA_SEL, "ip0_nna_sel",
		ip0_nna_parents, 0x0c0, 0x0c4, 0x0c8, 0, 4, 7, 0x8, 1),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_IP1_NNA_SEL, "ip1_nna_sel",
		ip1_nna_parents, 0x0c0, 0x0c4, 0x0c8, 8, 4, 15, 0x8, 2),
	MUX_GATE_CLR_SET_UPD(CLK_TOP_ADSP_BUS_SEL, "adsp_bus_sel",
		adsp_bus_parents, 0x0c0, 0x0c4, 0x0c8, 16, 3, 23, 0x8, 3),
	MUX_GATE_CLR_SET_UPD_FLAGS(CLK_TOP_CUPM_SEL, "cupm_sel",
		cupm_parents, 0x0c0, 0x0c4, 0x0c8, 24, 2, 31, 0x8, 4, CLK_IS_CRITICAL),
	/* CLK_CFG_9 */
	MUX_GATE_CLR_SET_UPD(CLK_TOP_I2C_PWR_SEL, "i2c_pwr_sel",
		i2c_pwr_parents, 0x0d0, 0x0d4, 0x0d8, 0, 3, 7, 0x8, 5),
};

static struct mtk_composite top_muxes[] = {
	/* CLK26CALI_2 */
	MUX(CLK_TOP_CLKRTC_SEL, "clkrtc_sel", clkrtc_parents, 0x228, 12, 1),
	/* CLK_AUDDIV_4 */
	MUX(CLK_TOP_I2SI1_MCK_PRE, "i2si1_mck_pre", i2si1_mck_pre_parents, 0x340, 11, 1),
	MUX(CLK_TOP_TDMIN_MCK_PRE, "tdmin_mck_pre", tdmin_mck_pre_parents, 0x340, 12, 1),
	MUX(CLK_TOP_I2SO1_MCK_PRE, "i2so1_mck_pre", i2so1_mck_pre_parents, 0x340, 13, 1),
};

static const struct mtk_composite top_adj_divs[] = {
	DIV_GATE_FLAGS(CLK_TOP_I2SI1_MCK, "i2si1_mck", "i2si1_mck_pre",
		0x340, 0, 0x344, 8, 0, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_GATE_FLAGS(CLK_TOP_TDMIN_MCK, "tdmin_mck", "tdmin_mck_pre",
		0x340, 1, 0x344, 8, 8, CLK_DIVIDER_ROUND_CLOSEST),
	DIV_GATE_FLAGS(CLK_TOP_I2SO1_MCK, "i2so1_mck", "i2so1_mck_pre",
		0x340, 2, 0x344, 8, 16, CLK_DIVIDER_ROUND_CLOSEST),
};

static const struct mtk_gate_regs infra_ao0_cg_regs = {
	.set_ofs = 0x200,
	.clr_ofs = 0x200,
	.sta_ofs = 0x200,
};

static const struct mtk_gate_regs infra_ao1_cg_regs = {
	.set_ofs = 0x74,
	.clr_ofs = 0x74,
	.sta_ofs = 0x74,
};

static const struct mtk_gate_regs infra_ao2_cg_regs = {
	.set_ofs = 0x80,
	.clr_ofs = 0x84,
	.sta_ofs = 0x90,
};

static const struct mtk_gate_regs infra_ao3_cg_regs = {
	.set_ofs = 0x88,
	.clr_ofs = 0x8c,
	.sta_ofs = 0x94,
};

static const struct mtk_gate_regs infra_ao4_cg_regs = {
	.set_ofs = 0xa4,
	.clr_ofs = 0xa8,
	.sta_ofs = 0xac,
};

static const struct mtk_gate_regs infra_ao5_cg_regs = {
	.set_ofs = 0xc0,
	.clr_ofs = 0xc4,
	.sta_ofs = 0xc8,
};

static const struct mtk_gate_regs infra_ao6_cg_regs = {
	.set_ofs = 0xd0,
	.clr_ofs = 0xd4,
	.sta_ofs = 0xd8,
};

#define GATE_INFRA_AO0(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao0_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr_inv)

#define GATE_INFRA_AO2(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao2_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO2_FLAGS(_id, _name, _parent, _shift, _flags)		\
	GATE_MTK_FLAGS(_id, _name, _parent, &infra_ao2_cg_regs, _shift,			\
	&mtk_clk_gate_ops_setclr, _flags)

#define GATE_INFRA_AO3(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao3_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO3_FLAGS(_id, _name, _parent, _shift, _flags)		\
	GATE_MTK_FLAGS(_id, _name, _parent, &infra_ao3_cg_regs, _shift,		\
	&mtk_clk_gate_ops_setclr, _flags)

#define GATE_INFRA_AO4(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao4_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO5(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao5_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_INFRA_AO5_FLAGS(_id, _name, _parent, _shift, _flags)		\
	GATE_MTK_FLAGS(_id, _name, _parent, &infra_ao5_cg_regs, _shift,			\
	&mtk_clk_gate_ops_setclr, _flags)

#define GATE_INFRA_AO6(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &infra_ao6_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate infra_ao_clks[] = {
	/* INFRA_AO0 */
	GATE_INFRA_AO0(CLK_INFRA_AO_TOPAXI_DISABLE, "infra_ao_topaxi_disable", "axi_sel", 31),
	/* INFRA_AO2 */
	GATE_INFRA_AO2_FLAGS(CLK_INFRA_AO_SEJ_SET, "infra_ao_sej_set", "axi_sel", 5,
		CLK_IS_CRITICAL),
	GATE_INFRA_AO2(CLK_INFRA_AO_ICUSB_SET, "infra_ao_icusb_set", "axi_sel", 8),
	GATE_INFRA_AO2(CLK_INFRA_AO_THERM_SET, "infra_ao_therm_set", "axi_sel", 10),
	GATE_INFRA_AO2(CLK_INFRA_AO_PWM1_SET, "infra_ao_pwm1_set", "pwm_sel", 16),
	GATE_INFRA_AO2(CLK_INFRA_AO_PWM2_SET, "infra_ao_pwm2_set", "pwm_sel", 17),
	GATE_INFRA_AO2(CLK_INFRA_AO_PWM3_SET, "infra_ao_pwm3_set", "pwm_sel", 18),
	GATE_INFRA_AO2(CLK_INFRA_AO_PWM4_SET, "infra_ao_pwm4_set", "pwm_sel", 19),
	GATE_INFRA_AO2(CLK_INFRA_AO_PWM5_SET, "infra_ao_pwm5_set", "pwm_sel", 20),
	GATE_INFRA_AO2(CLK_INFRA_AO_PWM_SET, "infra_ao_pwm_set", "pwm_sel", 21),
	GATE_INFRA_AO2(CLK_INFRA_AO_UART0_SET, "infra_ao_uart0_set", "uart_sel", 22),
	GATE_INFRA_AO2(CLK_INFRA_AO_UART1_SET, "infra_ao_uart1_set", "uart_sel", 23),
	GATE_INFRA_AO2(CLK_INFRA_AO_UART2_SET, "infra_ao_uart2_set", "uart_sel", 24),
	/* INFRA_AO3 */
	GATE_INFRA_AO3(CLK_INFRA_AO_SPI_SET, "infra_ao_spi_set", "spi_sel", 1),
	GATE_INFRA_AO3(CLK_INFRA_AO_MSDC0_SET, "infra_ao_msdc0_set", "msdc50_0_h_sel", 2),
	GATE_INFRA_AO3(CLK_INFRA_AO_MSDC1_SET, "infra_ao_msdc1_set", "axi_sel", 4),
	GATE_INFRA_AO3(CLK_INFRA_AO_GCPU_SET, "infra_ao_gcpu_set", "axi_sel", 8),
	GATE_INFRA_AO3(CLK_INFRA_AO_AUXADC_SET, "infra_ao_auxadc_set", "clk26m", 10),
	GATE_INFRA_AO3(CLK_INFRA_AO_CPUM_SET, "infra_ao_cpum_set", "axi_sel", 11),
	GATE_INFRA_AO3(CLK_INFRA_AO_AUXADC_MD_SET, "infra_ao_auxadc_md_set", "clk26m", 14),
	GATE_INFRA_AO3(CLK_INFRA_AO_RG_PEMI_IDLE_INFRA_HCLK_CK, "infra_ao_pemi_idle_infra_h_ck", "mem_466m_sel", 17),
	GATE_INFRA_AO3_FLAGS(CLK_INFRA_AO_AP_DMA_SET, "infra_ao_ap_dma_set", "axi_sel", 18,
		CLK_IS_CRITICAL),
	GATE_INFRA_AO3_FLAGS(CLK_INFRA_AO_DEVICE_APC_SET, "infra_ao_device_apc_set", "axi_sel", 20,
		CLK_IS_CRITICAL),
	GATE_INFRA_AO3(CLK_INFRA_AO_DEBUGSYS_SET, "infra_ao_debugsys_set", "axi_sel", 24),
	GATE_INFRA_AO3(CLK_INFRA_AO_AUDIO_SET, "infra_ao_audio_set", "axi_sel", 25),
	GATE_INFRA_AO3(CLK_INFRA_AO_EMI_SET, "infra_ao_emi_set", "mem_466m_sel", 27),
	GATE_INFRA_AO3(CLK_INFRA_AO_PEMI_SET, "infra_ao_pemi_set", "mem_466m_sel", 28),
	GATE_INFRA_AO3(CLK_INFRA_AO_RG_PEMI_IDLE_HCLK_CK, "infra_ao_pemi_idle_h_ck", "mem_466m_sel", 30),
	GATE_INFRA_AO3(CLK_INFRA_AO_DRAMC_F26M_SET, "infra_ao_dramc_f26m_set", "clk26m", 31),
	/* INFRA_AO4 */
	GATE_INFRA_AO4(CLK_INFRA_AO_RG_PWM_FBCLK6_CK_SET, "infra_ao_pwm_fbclk6_set", "pwm_sel", 0),
	GATE_INFRA_AO4(CLK_INFRA_AO_RG_PWM_FBCLK7_CK_SET, "infra_ao_pwm_fbclk7_set", "pwm_sel", 1),
	GATE_INFRA_AO4(CLK_INFRA_AO_RG_I2C_PWR_BCLK_CK_SET, "infra_ao_i2c_pwr_bclk_set", "i2c_pwr_sel", 5),
	GATE_INFRA_AO4(CLK_INFRA_AO_SPIS_SET, "infra_ao_spis_set", "spi_sel", 6),
	GATE_INFRA_AO4(CLK_INFRA_AO_BIST2FPC_SET, "infra_ao_bist2fpc_set", "bist2fpc_ck", 28),
	/* INFRA_AO5 */
	GATE_INFRA_AO5(CLK_INFRA_AO_AP_MSDC0_SET, "infra_ao_ap_msdc0_set", "msdc50_0_sel", 7),
	GATE_INFRA_AO5(CLK_INFRA_AO_MD_MSDC0_SET, "infra_ao_md_msdc0_set", "msdc50_0_sel", 8),
	GATE_INFRA_AO5(CLK_INFRA_AO_MSDC0_SRC_SET, "infra_ao_msdc0_src_set", "msdc50_0_sel", 9),
	GATE_INFRA_AO5(CLK_INFRA_AO_MSDC1_SRC_SET, "infra_ao_msdc1_src_set", "msdc30_1_sel", 10),
	GATE_INFRA_AO5_FLAGS(CLK_INFRA_AO_SEJ_F13M_CK_SET, "infra_ao_sej_f13m_set", "clk26m", 15,
		CLK_IS_CRITICAL),
	GATE_INFRA_AO5(CLK_INFRA_AO_IRRX_PERI_26M_SET, "infra_ao_irrx_peri_26m_set", "clk26m", 22),
	GATE_INFRA_AO5(CLK_INFRA_AO_IRRX_PERI_32K_SET, "infra_ao_irrx_peri_32k_set", "clkrtc_sel", 23),
	GATE_INFRA_AO5(CLK_INFRA_AO_RG_I2C0_BCLK_CK, "infra_ao_i2c0_bclk_ck", "i2c_ao_sel", 24),
	GATE_INFRA_AO5(CLK_INFRA_AO_RG_IIC_AO_BCLK_CK_SET, "infra_ao_iic_ao_bclk_set", "axi_sel", 28),
	GATE_INFRA_AO5(CLK_INFRA_AO_RG_IIC_PERI_BCLK_CK_SET, "infra_ao_iic_peri_bclk_set", "axi_sel", 29),
	/* INFRA_AO6 */
	GATE_INFRA_AO6(CLK_INFRA_AO_NFI_BCLK_CK_SET, "infra_ao_nfi_bclk_set", "nfi1x_sel", 1),
	GATE_INFRA_AO6(CLK_INFRA_AO_NFIECC_BCLK_CK_SET, "infra_ao_nfiecc_bclk_set", "nfi1x_sel", 2),
	GATE_INFRA_AO6(CLK_INFRA_AO_NFI_HCLK_CK_SET, "infra_ao_nfi_h_set", "axi_sel", 3),
	GATE_INFRA_AO6(CLK_INFRA_AO_SSUSB_TOP_SYS_CK_SET, "infra_ao_ssusb_sys_set", "ssusb_sys_sel", 9),
	GATE_INFRA_AO6(CLK_INFRA_AO_SSUSB_TOP_XHCI_CK_SET, "infra_ao_ssusb_xhci_set", "ssusb_xhci_sel", 11),
	GATE_INFRA_AO6(CLK_INFRA_AO_NFI_INFRA_BCLK_CK_SET, "infra_ao_nfi_infra_bclk_set", "spinfi_bclk_sel", 16),
	GATE_INFRA_AO6(CLK_INFRA_AO_INFRA_SPIS_CK_SET, "infra_ao_infra_spis_set", "spis_sel", 17),
};

static const struct mtk_gate_regs top0_cg_regs = {
	.set_ofs = 0X0,
	.clr_ofs = 0X0,
	.sta_ofs = 0X0,
};

static const struct mtk_gate_regs top1_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x104,
	.sta_ofs = 0x104,
};

static const struct mtk_gate_regs top2_cg_regs = {
	.set_ofs = 0x220,
	.clr_ofs = 0x220,
	.sta_ofs = 0x220,
};

static const struct mtk_gate_regs top3_cg_regs = {
	.set_ofs = 0x338,
	.clr_ofs = 0x338,
	.sta_ofs = 0x338,
};

static const struct mtk_gate_regs top4_cg_regs = {
	.set_ofs = 0x33c,
	.clr_ofs = 0x33c,
	.sta_ofs = 0x33c,
};

static const struct mtk_gate_regs top5_cg_regs = {
	.set_ofs = 0x340,
	.clr_ofs = 0x340,
	.sta_ofs = 0x340,
};

static const struct mtk_gate_regs top6_cg_regs = {
	.set_ofs = 0x34c,
	.clr_ofs = 0x34c,
	.sta_ofs = 0x34c,
};

static const struct mtk_gate_regs top7_cg_regs = {
	.set_ofs = 0x360,
	.clr_ofs = 0x360,
	.sta_ofs = 0x360,
};

#define GATE_TOP1(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &top1_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr_inv)

#define GATE_TOP3(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &top3_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr_inv)

#define GATE_TOP4(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &top4_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr_inv)

#define GATE_TOP5(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &top5_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr_inv)

#define GATE_TOP7(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &top7_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr_inv)

#define GATE_TOP1_FLAGS(_id, _name, _parent, _shift, _flags)			\
	GATE_MTK_FLAGS(_id, _name, _parent, &top1_cg_regs, _shift,	\
	&mtk_clk_gate_ops_no_setclr_inv, _flags)

#define GATE_TOP2_FLAGS(_id, _name, _parent, _shift, _flags)			\
	GATE_MTK_FLAGS(_id, _name, _parent, &top2_cg_regs, _shift,	\
	&mtk_clk_gate_ops_no_setclr_inv, _flags)

static const struct mtk_gate top_clks[] = {
	/* TOP1 */
	GATE_TOP1_FLAGS(CLK_TOP_MPLL_104M_EN, "mpll_104m_en", "mpll_d2", 10,
		CLK_IS_CRITICAL),
	GATE_TOP1_FLAGS(CLK_TOP_MPLL_52M_EN, "mpll_52m_en", "mpll_d4", 11,
		CLK_IS_CRITICAL),
	GATE_TOP1(CLK_TOP_F_BIST2FPC_CK_EN, "bist2fpc", "bist2fpc_ck", 16),
	GATE_TOP1(CLK_TOP_RG_SSUSB_TOP_REF_CK_EN, "rg_ssusb_ref", "clk26m", 22),
	GATE_TOP1(CLK_TOP_RG_SSUSB_PHY_REF_CK_EN, "rg_ssusb_phy_ref", "clk26m", 23),
	/* TOP2 */
	GATE_TOP2_FLAGS(CLK_TOP_CLKRTC_INT_EN, "clkrtc_int_en", "clkrtc_int",
		30, CLK_IS_CRITICAL),
	/* TOP3 */
	GATE_TOP3(CLK_TOP_APLL1_D3_EN, "apll1_d3_en", "apll1_d3", 8),
	GATE_TOP3(CLK_TOP_APLL2_D3_EN, "apll2_d3_en", "apll2_d3", 9),
	/* TOP4 */
	GATE_TOP4(CLK_TOP_IPPLL_D3_EN, "ippll_d3_en", "ippll_d3", 0),
	GATE_TOP4(CLK_TOP_IPSYS_F26M_CGEN, "ipsys_f26m_cgen", "clk26m", 4),
	/* TOP5 */
	GATE_TOP5(CLK_TOP_APLL12_DIV7, "apll12_div7", "i2si1_mck", 0),
	GATE_TOP5(CLK_TOP_APLL12_DIV8, "apll12_div8", "tdmin_mck", 1),
	GATE_TOP5(CLK_TOP_APLL12_DIV9, "apll12_div9", "i2so1_mck", 2),
	/* TOP7 */
	GATE_TOP7(CLK_TOP_SSUSB_U2_PHY_REF_CK_1P, "ssusb_u2_phy_ref_1p", "clk26m", 0),
};

#define MT8519_PLL_FMAX		(3800UL * MHZ)
#define MT8519_PLL_FMIN		(1500UL * MHZ)
#define MT8519_INTEGER_BITS	8

#define PLL(_id, _name, _reg, _pwr_reg, _en_mask, _flags,	\
			_rst_bar_mask, _pcwbits, _pd_reg, _pd_shift,	\
			_tuner_reg, _tuner_en_reg, _tuner_en_bit,	\
			_pcw_reg, _pcw_shift, _pcw_chg_reg,				\
			_en_reg, _pll_en_bit) {					\
		.id = _id,						\
		.name = _name,						\
		.reg = _reg,						\
		.pwr_reg = _pwr_reg,					\
		.en_mask = _en_mask,					\
		.flags = _flags,					\
		.rst_bar_mask = _rst_bar_mask,				\
		.fmax = MT8519_PLL_FMAX,				\
		.fmin = MT8519_PLL_FMIN,				\
		.pcwbits = _pcwbits,					\
		.pcwibits = MT8519_INTEGER_BITS,			\
		.pd_reg = _pd_reg,					\
		.pd_shift = _pd_shift,					\
		.tuner_reg = _tuner_reg,				\
		.tuner_en_reg = _tuner_en_reg,				\
		.tuner_en_bit = _tuner_en_bit,				\
		.pcw_reg = _pcw_reg,					\
		.pcw_shift = _pcw_shift,				\
		.pcw_chg_reg = _pcw_chg_reg,				\
		.en_reg = _en_reg,					\
		.pll_en_bit = _pll_en_bit,				\
	}

static const struct mtk_pll_data plls[] = {
	PLL(CLK_APMIXED_MAINPLL, "mainpll", 0x0228, 0x0234, 0xff000000,
		HAVE_RST_BAR, BIT(23), 22, 0x022C, 24, 0, 0, 0, 0x022C, 0, 0, 0, 0),
	PLL(CLK_APMIXED_UNIVPLL, "univpll", 0x0208, 0x0214, 0xff000000,
		HAVE_RST_BAR, BIT(23), 22, 0x020C, 24, 0, 0, 0, 0x020C, 0, 0, 0, 0),
	PLL(CLK_APMIXED_MSDCPLL, "msdcpll", 0x0350, 0x035C, 0,
		0, 0, 22, 0x0354, 24, 0, 0, 0, 0x0354, 0, 0, 0, 0),
	PLL(CLK_APMIXED_APLL1, "apll1", 0x031C, 0x032C, 0,
		0, 0, 32, 0x0320, 24, 0x0040, 0x000C, 0, 0x0324, 0, 0, 0, 0),
	PLL(CLK_APMIXED_APLL2, "apll2", 0x0360, 0x0370, 0,
		0, 0, 32, 0x0364, 24, 0x004C, 0x000C, 5, 0x0368, 0, 0, 0, 0),
	PLL(CLK_APMIXED_MPLL, "mpll", 0x0340, 0x034C, 0,
		0, 0, 22, 0x0344, 24, 0, 0, 0, 0x0344, 0, 0, 0, 0),
	PLL(CLK_APMIXED_IPPLL, "ippll", 0x0374, 0x0380, 0,
		0, 0, 22, 0x0378, 24, 0, 0, 0, 0x0378, 0, 0, 0, 0),
	PLL(CLK_APMIXED_NNAPLL, "nnapll", 0x0330, 0x033C, 0,
		0, 0, 22, 0x0334, 24, 0, 0, 0, 0x0334, 0, 0, 0, 0),
	PLL(CLK_APMIXED_DSPPLL, "dsppll", 0x0390, 0x039C, 0xff000000,
		0, 0, 22, 0x0394, 24, 0, 0, 0, 0x0394, 0, 0, 0, 0),
};

static int clk_mt8519_top_probe(struct platform_device *pdev)
{
	struct clk_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;
	int r;
	void __iomem *base;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk_data = mtk_alloc_clk_data(CLK_TOP_NR_CLK);
	if (!clk_data)
		return -ENOMEM;

	mtk_clk_register_factors(top_divs, ARRAY_SIZE(top_divs), clk_data);
	mtk_clk_register_muxes(top_mtk_muxes, ARRAY_SIZE(top_mtk_muxes), node,
			&mt8519_clk_lock, clk_data);

	mtk_clk_register_composites(top_muxes, ARRAY_SIZE(top_muxes), base,
			&mt8519_clk_lock, clk_data);
	mtk_clk_register_composites(top_adj_divs, ARRAY_SIZE(top_adj_divs), base,
			&mt8519_clk_lock, clk_data);
	r = mtk_clk_register_gates(node, top_clks, ARRAY_SIZE(top_clks), clk_data);
	if (r)
		return r;

	return of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
}

static int clk_mt8519_infra_ao_probe(struct platform_device *pdev)
{
	struct clk_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;
	int r;

	clk_data = mtk_alloc_clk_data(CLK_INFRA_AO_NR_CLK);
	if (!clk_data)
		return -ENOMEM;

	r = mtk_clk_register_gates(node, infra_ao_clks, ARRAY_SIZE(infra_ao_clks), clk_data);
	if (r)
		return r;

	return of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
}

static int clk_mt8519_apmixed_probe(struct platform_device *pdev)
{
	struct clk_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;

	clk_data = mtk_alloc_clk_data(CLK_APMIXED_NR_CLK);
	if (!clk_data)
		return -ENOMEM;

	mtk_clk_register_plls(node, plls, ARRAY_SIZE(plls), clk_data);

	return of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
}

static const struct of_device_id of_match_clk_mt8519[] = {
	{
		.compatible = "mediatek,mt8519-apmixedsys",
		.data = clk_mt8519_apmixed_probe,
	}, {
		.compatible = "mediatek,mt8519-topckgen",
		.data = clk_mt8519_top_probe,
	}, {
		.compatible = "mediatek,mt8519-infracfg_ao",
		.data = clk_mt8519_infra_ao_probe,
	}, {
		/* sentinel */
	}
};

static int clk_mt8519_probe(struct platform_device *pdev)
{
	int (*clk_probe)(struct platform_device *pdev);
	int r;

	clk_probe = of_device_get_match_data(&pdev->dev);
	if (!clk_probe)
		return -EINVAL;

	r = clk_probe(pdev);
	if (r)
		dev_err(&pdev->dev,
			"could not register clock provider: %s: %d\n",
			pdev->name, r);

	return r;
}

static struct platform_driver clk_mt8519_drv = {
	.probe = clk_mt8519_probe,
	.driver = {
		.name = "clk-mt8519",
		.of_match_table = of_match_clk_mt8519,
	},
};

static int __init clk_mt8519_init(void)
{
	return platform_driver_register(&clk_mt8519_drv);
}

static void __exit clk_mt8519_exit(void)
{
	platform_driver_unregister(&clk_mt8519_drv);
}

arch_initcall(clk_mt8519_init);
module_exit(clk_mt8519_exit);
MODULE_LICENSE("GPL");
