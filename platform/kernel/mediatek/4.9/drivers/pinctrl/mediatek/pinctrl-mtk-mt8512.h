/*
 * Copyright (C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __PINCTRL_MTK_MT8512_H
#define __PINCTRL_MTK_MT8512_H

#include <linux/pinctrl/pinctrl.h>
#include "pinctrl-mtk-common.h"

static const struct mtk_desc_pin mtk_pins_mt8512[] = {
	MTK_PIN(
		PINCTRL_PIN(0, "GPIO0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 0),
		MTK_FUNCTION(0, "GPIO0"),
		MTK_FUNCTION(1, "DMIC_CLK0"),
		MTK_FUNCTION(3, "CONN_MCU_AICE_TMSC"),
		MTK_FUNCTION(5, "UDI_NTRST_XI"),
		MTK_FUNCTION(7, "DBG_MON_A_0")
	),
	MTK_PIN(
		PINCTRL_PIN(1, "GPIO1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 1),
		MTK_FUNCTION(0, "GPIO1"),
		MTK_FUNCTION(1, "DMIC_DAT0"),
		MTK_FUNCTION(3, "CONN_MCU_AICE_TCKC"),
		MTK_FUNCTION(5, "UDI_TMS_XI"),
		MTK_FUNCTION(7, "DBG_MON_A_1")
	),
	MTK_PIN(
		PINCTRL_PIN(2, "GPIO2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 2),
		MTK_FUNCTION(0, "GPIO2"),
		MTK_FUNCTION(1, "EPDC_D11"),
		MTK_FUNCTION(5, "UDI_TCK_XI"),
		MTK_FUNCTION(7, "DBG_MON_A_2")
	),
	MTK_PIN(
		PINCTRL_PIN(3, "GPIO3"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 3),
		MTK_FUNCTION(0, "GPIO3"),
		MTK_FUNCTION(1, "EPDC_D13"),
		MTK_FUNCTION(5, "UDI_TDI_XI"),
		MTK_FUNCTION(7, "DBG_MON_A_3")
	),
	MTK_PIN(
		PINCTRL_PIN(4, "GPIO4"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 4),
		MTK_FUNCTION(0, "GPIO4"),
		MTK_FUNCTION(1, "EPDC_D0"),
		MTK_FUNCTION(5, "UDI_TDO"),
		MTK_FUNCTION(7, "DBG_MON_A_4")
	),
	MTK_PIN(
		PINCTRL_PIN(5, "GPIO5"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 5),
		MTK_FUNCTION(0, "GPIO5"),
		MTK_FUNCTION(1, "EPDC_D2"),
		MTK_FUNCTION(7, "DBG_MON_A_5")
	),
	MTK_PIN(
		PINCTRL_PIN(6, "GPIO6"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 6),
		MTK_FUNCTION(0, "GPIO6"),
		MTK_FUNCTION(1, "EPDC_D4"),
		MTK_FUNCTION(2, "SDA0_0"),
		MTK_FUNCTION(3, "DMIC_DAT4"),
		MTK_FUNCTION(7, "DBG_MON_A_6")
	),
	MTK_PIN(
		PINCTRL_PIN(7, "GPIO7"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 7),
		MTK_FUNCTION(0, "GPIO7"),
		MTK_FUNCTION(1, "EPDC_SDOE"),
		MTK_FUNCTION(2, "SCL0_0"),
		MTK_FUNCTION(3, "DMIC_DAT5"),
		MTK_FUNCTION(4, "WIFI_TXD"),
		MTK_FUNCTION(5, "IRRX"),
		MTK_FUNCTION(7, "DBG_MON_A_7")
	),
	MTK_PIN(
		PINCTRL_PIN(8, "GPIO8"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 8),
		MTK_FUNCTION(0, "GPIO8"),
		MTK_FUNCTION(1, "KPCOL0"),
		MTK_FUNCTION(2, "SPDIF_IN0"),
		MTK_FUNCTION(3, "DMIC_DAT6"),
		MTK_FUNCTION(4, "CONN_UART0_RXD"),
		MTK_FUNCTION(5, "IRRX")
	),
	MTK_PIN(
		PINCTRL_PIN(9, "GPIO9"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 9),
		MTK_FUNCTION(0, "GPIO9"),
		MTK_FUNCTION(1, "KPCOL1"),
		MTK_FUNCTION(2, "SPDIF_IN1"),
		MTK_FUNCTION(3, "DMIC_DAT7"),
		MTK_FUNCTION(4, "CONN_UART0_TXD"),
		MTK_FUNCTION(5, "WIFI_TXD"),
		MTK_FUNCTION(6, "SRCLKENA0")
	),
	MTK_PIN(
		PINCTRL_PIN(10, "GPIO10"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 10),
		MTK_FUNCTION(0, "GPIO10"),
		MTK_FUNCTION(1, "KPCOL2"),
		MTK_FUNCTION(2, "SPDIF_IN2"),
		MTK_FUNCTION(3, "DMIC_CLK2")
	),
	MTK_PIN(
		PINCTRL_PIN(11, "GPIO11"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 11),
		MTK_FUNCTION(0, "GPIO11"),
		MTK_FUNCTION(1, "KPCOL3"),
		MTK_FUNCTION(3, "DMIC_CLK3")
	),
	MTK_PIN(
		PINCTRL_PIN(12, "GPIO12"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 12),
		MTK_FUNCTION(0, "GPIO12"),
		MTK_FUNCTION(3, "PWM_4"),
		MTK_FUNCTION(5, "CONN_MCU_DBGACK_N"),
		MTK_FUNCTION(6, "ANT_SEL4")
	),
	MTK_PIN(
		PINCTRL_PIN(13, "GPIO13"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 13),
		MTK_FUNCTION(0, "GPIO13"),
		MTK_FUNCTION(3, "PWM_5"),
		MTK_FUNCTION(5, "CONN_MCU_DBGI_N"),
		MTK_FUNCTION(6, "ANT_SEL5")
	),
	MTK_PIN(
		PINCTRL_PIN(14, "GPIO14"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 14),
		MTK_FUNCTION(0, "GPIO14"),
		MTK_FUNCTION(3, "PWM_6"),
		MTK_FUNCTION(4, "I2SIN_DAT3"),
		MTK_FUNCTION(5, "CONN_MCU_TCK"),
		MTK_FUNCTION(6, "ANT_SEL6")
	),
	MTK_PIN(
		PINCTRL_PIN(15, "GPIO15"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 15),
		MTK_FUNCTION(0, "GPIO15"),
		MTK_FUNCTION(1, "SPI_DAT0_NOR"),
		MTK_FUNCTION(2, "SPI_DAT0_NAND"),
		MTK_FUNCTION(4, "I2SIN_DAT2"),
		MTK_FUNCTION(5, "CONN_MCU_TDI"),
		MTK_FUNCTION(6, "ANT_SEL7")
	),
	MTK_PIN(
		PINCTRL_PIN(16, "GPIO16"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 16),
		MTK_FUNCTION(0, "GPIO16"),
		MTK_FUNCTION(1, "SPI_DAT1_NOR"),
		MTK_FUNCTION(2, "SPI_DAT1_NAND"),
		MTK_FUNCTION(4, "I2SIN_DAT1"),
		MTK_FUNCTION(5, "CONN_MCU_TRST_B"),
		MTK_FUNCTION(6, "ANT_SEL0")
	),
	MTK_PIN(
		PINCTRL_PIN(17, "GPIO17"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 17),
		MTK_FUNCTION(0, "GPIO17"),
		MTK_FUNCTION(1, "SPI_DAT2_NOR"),
		MTK_FUNCTION(2, "SPI_DAT2_NAND"),
		MTK_FUNCTION(4, "I2SIN_DAT0"),
		MTK_FUNCTION(5, "CONN_MCU_TMS"),
		MTK_FUNCTION(6, "ANT_SEL1")
	),
	MTK_PIN(
		PINCTRL_PIN(18, "GPIO18"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 18),
		MTK_FUNCTION(0, "GPIO18"),
		MTK_FUNCTION(1, "SPI_DAT3_NOR"),
		MTK_FUNCTION(2, "SPI_DAT3_NAND"),
		MTK_FUNCTION(4, "I2SIN_MCK"),
		MTK_FUNCTION(5, "CONN_MCU_TDO"),
		MTK_FUNCTION(6, "ANT_SEL2")
	),
	MTK_PIN(
		PINCTRL_PIN(19, "GPIO19"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 19),
		MTK_FUNCTION(0, "GPIO19"),
		MTK_FUNCTION(1, "SPI_CSB_NOR"),
		MTK_FUNCTION(2, "SPI_CSB_NAND"),
		MTK_FUNCTION(4, "I2SIN_LRCK"),
		MTK_FUNCTION(6, "ANT_SEL3")
	),
	MTK_PIN(
		PINCTRL_PIN(20, "GPIO20"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 20),
		MTK_FUNCTION(0, "GPIO20"),
		MTK_FUNCTION(1, "SPI_CLK_NOR"),
		MTK_FUNCTION(2, "SPI_CLK_NAND"),
		MTK_FUNCTION(4, "I2SIN_BCK")
	),
	MTK_PIN(
		PINCTRL_PIN(21, "AUDIO_SYNC"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 21),
		MTK_FUNCTION(0, "GPIO21"),
		MTK_FUNCTION(1, "CONN_WF_CTRL2"),
		MTK_FUNCTION(6, "TSF_IN"),
		MTK_FUNCTION(7, "DBG_MON_B_16")
	),
	MTK_PIN(
		PINCTRL_PIN(22, "WIFI_INTB"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 22),
		MTK_FUNCTION(0, "GPIO22"),
		MTK_FUNCTION(1, "CONN_WF_CTRL0"),
		MTK_FUNCTION(7, "DBG_MON_B_17")
	),
	MTK_PIN(
		PINCTRL_PIN(23, "BT_INTB"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 23),
		MTK_FUNCTION(0, "GPIO23"),
		MTK_FUNCTION(1, "CONN_BT_CLK"),
		MTK_FUNCTION(6, "DVFSRC_EXT_REQ"),
		MTK_FUNCTION(7, "DBG_MON_B_18")
	),
	MTK_PIN(
		PINCTRL_PIN(24, "BT_STEREO"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 24),
		MTK_FUNCTION(0, "GPIO24"),
		MTK_FUNCTION(1, "CONN_WF_CTRL1"),
		MTK_FUNCTION(7, "DBG_MON_B_19")
	),
	MTK_PIN(
		PINCTRL_PIN(25, "RSTNB"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 25),
		MTK_FUNCTION(0, "GPIO25"),
		MTK_FUNCTION(1, "CONN_BT_DATA"),
		MTK_FUNCTION(7, "DBG_MON_B_20")
	),
	MTK_PIN(
		PINCTRL_PIN(26, "USB_ID"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 26),
		MTK_FUNCTION(0, "GPIO26"),
		MTK_FUNCTION(1, "USB_IDDIG"),
		MTK_FUNCTION(6, "DVFSRC_EXT_REQ"),
		MTK_FUNCTION(7, "DBG_MON_B_21")
	),
	MTK_PIN(
		PINCTRL_PIN(27, "USB_DRV"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 27),
		MTK_FUNCTION(0, "GPIO27"),
		MTK_FUNCTION(1, "USB_DRVVBUS"),
		MTK_FUNCTION(5, "ADSP_JTAG_TMS"),
		MTK_FUNCTION(7, "DBG_MON_B_22")
	),
	MTK_PIN(
		PINCTRL_PIN(28, "EINT_GAUGEING"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 28),
		MTK_FUNCTION(0, "GPIO28"),
		MTK_FUNCTION(1, "URXD2"),
		MTK_FUNCTION(2, "PWM_0"),
		MTK_FUNCTION(3, "KPCOL0"),
		MTK_FUNCTION(5, "ADSP_JTAG_TCK"),
		MTK_FUNCTION(7, "DBG_MON_B_23")
	),
	MTK_PIN(
		PINCTRL_PIN(29, "CHG_IRQ"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 29),
		MTK_FUNCTION(0, "GPIO29"),
		MTK_FUNCTION(1, "UTXD2"),
		MTK_FUNCTION(2, "PWM_1"),
		MTK_FUNCTION(3, "KPCOL1"),
		MTK_FUNCTION(5, "ADSP_JTAG_TDI"),
		MTK_FUNCTION(7, "DBG_MON_B_24")
	),
	MTK_PIN(
		PINCTRL_PIN(30, "CHG_OTG"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 30),
		MTK_FUNCTION(0, "GPIO30"),
		MTK_FUNCTION(1, "UTXD0"),
		MTK_FUNCTION(2, "PWM_2"),
		MTK_FUNCTION(3, "KPCOL2"),
		MTK_FUNCTION(4, "SDA1_0"),
		MTK_FUNCTION(5, "ADSP_JTAG_TDO"),
		MTK_FUNCTION(7, "DBG_MON_B_12")
	),
	MTK_PIN(
		PINCTRL_PIN(31, "CHG_CEB"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 31),
		MTK_FUNCTION(0, "GPIO31"),
		MTK_FUNCTION(1, "URXD0"),
		MTK_FUNCTION(2, "PWM_3"),
		MTK_FUNCTION(3, "KPCOL3"),
		MTK_FUNCTION(4, "SCL1_0"),
		MTK_FUNCTION(5, "ADSP_JTAG_TRST"),
		MTK_FUNCTION(7, "DBG_MON_B_6")
	),
	MTK_PIN(
		PINCTRL_PIN(32, "FL_EN"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 32),
		MTK_FUNCTION(0, "GPIO32"),
		MTK_FUNCTION(2, "SPDIF_IN0"),
		MTK_FUNCTION(3, "SPDIF_IN1"),
		MTK_FUNCTION(4, "SPDIF_IN2"),
		MTK_FUNCTION(5, "IRRX")
	),
	MTK_PIN(
		PINCTRL_PIN(33, "WAN_SMS_RDY"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 33),
		MTK_FUNCTION(0, "GPIO33"),
		MTK_FUNCTION(1, "KPROW0"),
		MTK_FUNCTION(2, "PWM_4"),
		MTK_FUNCTION(4, "PCM_CLK"),
		MTK_FUNCTION(5, "WATCHDOG"),
		MTK_FUNCTION(6, "SPI_CSB"),
		MTK_FUNCTION(7, "DBG_MON_B_3")
	),
	MTK_PIN(
		PINCTRL_PIN(34, "SOC2WAN_RESET"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 34),
		MTK_FUNCTION(0, "GPIO34"),
		MTK_FUNCTION(1, "KPROW1"),
		MTK_FUNCTION(2, "PWM_5"),
		MTK_FUNCTION(4, "PCM_SYNC"),
		MTK_FUNCTION(5, "SRCLKENA0"),
		MTK_FUNCTION(6, "SPI_CLK"),
		MTK_FUNCTION(7, "DBG_MON_B_2")
	),
	MTK_PIN(
		PINCTRL_PIN(35, "WAN_FM_RDY"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 35),
		MTK_FUNCTION(0, "GPIO35"),
		MTK_FUNCTION(1, "KPCOL0"),
		MTK_FUNCTION(2, "PWM_6"),
		MTK_FUNCTION(4, "PCM_RX"),
		MTK_FUNCTION(5, "CONN_UART0_RXD"),
		MTK_FUNCTION(6, "SPI_MI"),
		MTK_FUNCTION(7, "DBG_MON_B_14")
	),
	MTK_PIN(
		PINCTRL_PIN(36, "WAN_DIS"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 36),
		MTK_FUNCTION(0, "GPIO36"),
		MTK_FUNCTION(1, "KPCOL1"),
		MTK_FUNCTION(4, "PCM_TX"),
		MTK_FUNCTION(5, "CONN_UART0_TXD"),
		MTK_FUNCTION(6, "SPI_MO"),
		MTK_FUNCTION(7, "DBG_MON_B_15")
	),
	MTK_PIN(
		PINCTRL_PIN(37, "WAN_VBUS_EN"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 37),
		MTK_FUNCTION(0, "GPIO37")
	),
	MTK_PIN(
		PINCTRL_PIN(38, "WAN_VBAT_EN"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 38),
		MTK_FUNCTION(0, "GPIO38")
	),
	MTK_PIN(
		PINCTRL_PIN(39, "WAN_PWR_EN"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 39),
		MTK_FUNCTION(0, "GPIO39")
	),
	MTK_PIN(
		PINCTRL_PIN(40, "KPROW0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 40),
		MTK_FUNCTION(0, "GPIO40"),
		MTK_FUNCTION(1, "KPROW0"),
		MTK_FUNCTION(2, "KPCOL2")
	),
	MTK_PIN(
		PINCTRL_PIN(41, "KPROW1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 41),
		MTK_FUNCTION(0, "GPIO41"),
		MTK_FUNCTION(1, "KPROW1"),
		MTK_FUNCTION(2, "KPCOL3")
	),
	MTK_PIN(
		PINCTRL_PIN(42, "KPCOL0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 42),
		MTK_FUNCTION(0, "GPIO42"),
		MTK_FUNCTION(1, "KPCOL0"),
		MTK_FUNCTION(7, "DBG_MON_A_31")
	),
	MTK_PIN(
		PINCTRL_PIN(43, "KPCOL1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 43),
		MTK_FUNCTION(0, "GPIO43"),
		MTK_FUNCTION(1, "KPCOL1"),
		MTK_FUNCTION(7, "DBG_MON_A_32")
	),
	MTK_PIN(
		PINCTRL_PIN(44, "PWM0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 44),
		MTK_FUNCTION(0, "GPIO44"),
		MTK_FUNCTION(1, "PWM_0"),
		MTK_FUNCTION(5, "SPDIF_IN0"),
		MTK_FUNCTION(7, "DBG_MON_A_25")
	),
	MTK_PIN(
		PINCTRL_PIN(45, "PWM1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 45),
		MTK_FUNCTION(0, "GPIO45"),
		MTK_FUNCTION(1, "PWM_1"),
		MTK_FUNCTION(5, "SPDIF_IN1"),
		MTK_FUNCTION(7, "DBG_MON_A_26")
	),
	MTK_PIN(
		PINCTRL_PIN(46, "PWM2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 46),
		MTK_FUNCTION(0, "GPIO46"),
		MTK_FUNCTION(1, "PWM_2"),
		MTK_FUNCTION(5, "SPDIF_IN2"),
		MTK_FUNCTION(6, "DSP_TEST_CK"),
		MTK_FUNCTION(7, "DBG_MON_A_27")
	),
	MTK_PIN(
		PINCTRL_PIN(47, "PWM3"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 47),
		MTK_FUNCTION(0, "GPIO47"),
		MTK_FUNCTION(1, "PWM_3"),
		MTK_FUNCTION(5, "IRRX"),
		MTK_FUNCTION(6, "DVFSRC_EXT_REQ"),
		MTK_FUNCTION(7, "DBG_MON_A_28")
	),
	MTK_PIN(
		PINCTRL_PIN(48, "JTMS"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 48),
		MTK_FUNCTION(0, "GPIO48"),
		MTK_FUNCTION(1, "JTMS"),
		MTK_FUNCTION(2, "DFD_TMS_XI"),
		MTK_FUNCTION(4, "SPIS_CSB"),
		MTK_FUNCTION(5, "CONN_MCU_TMS")
	),
	MTK_PIN(
		PINCTRL_PIN(49, "JTCK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 49),
		MTK_FUNCTION(0, "GPIO49"),
		MTK_FUNCTION(1, "JTCK"),
		MTK_FUNCTION(2, "DFD_TCK_XI"),
		MTK_FUNCTION(4, "SPIS_CLK"),
		MTK_FUNCTION(5, "CONN_MCU_TCK")
	),
	MTK_PIN(
		PINCTRL_PIN(50, "JTDI"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 50),
		MTK_FUNCTION(0, "GPIO50"),
		MTK_FUNCTION(1, "JTDI"),
		MTK_FUNCTION(2, "DFD_TDI_XI"),
		MTK_FUNCTION(4, "SPIS_SO"),
		MTK_FUNCTION(5, "CONN_MCU_TDI")
	),
	MTK_PIN(
		PINCTRL_PIN(51, "JTDO"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 51),
		MTK_FUNCTION(0, "GPIO51"),
		MTK_FUNCTION(1, "JTDO"),
		MTK_FUNCTION(2, "DFD_TDO"),
		MTK_FUNCTION(4, "SPIS_SI"),
		MTK_FUNCTION(5, "CONN_MCU_TDO")
	),
	MTK_PIN(
		PINCTRL_PIN(52, "URXD0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 52),
		MTK_FUNCTION(0, "GPIO52"),
		MTK_FUNCTION(1, "URXD0"),
		MTK_FUNCTION(2, "UTXD0"),
		MTK_FUNCTION(5, "DSP_URXD")
	),
	MTK_PIN(
		PINCTRL_PIN(53, "UTXD0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 53),
		MTK_FUNCTION(0, "GPIO53"),
		MTK_FUNCTION(1, "UTXD0"),
		MTK_FUNCTION(2, "URXD0"),
		MTK_FUNCTION(5, "DSP_UTXD"),
		MTK_FUNCTION(6, "DVFSRC_EXT_REQ"),
		MTK_FUNCTION(7, "DBG_MON_B_7")
	),
	MTK_PIN(
		PINCTRL_PIN(54, "URXD1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 54),
		MTK_FUNCTION(0, "GPIO54"),
		MTK_FUNCTION(1, "URXD1"),
		MTK_FUNCTION(2, "KPCOL0"),
		MTK_FUNCTION(3, "PWM_0"),
		MTK_FUNCTION(4, "CONN_TCXO_REQ"),
		MTK_FUNCTION(6, "ANT_SEL0"),
		MTK_FUNCTION(7, "DBG_MON_B_8")
	),
	MTK_PIN(
		PINCTRL_PIN(55, "UTXD1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 55),
		MTK_FUNCTION(0, "GPIO55"),
		MTK_FUNCTION(1, "UTXD1"),
		MTK_FUNCTION(2, "KPCOL1"),
		MTK_FUNCTION(3, "PWM_1"),
		MTK_FUNCTION(5, "CONN_MCU_TRST_B"),
		MTK_FUNCTION(6, "ANT_SEL1"),
		MTK_FUNCTION(7, "DBG_MON_B_9")
	),
	MTK_PIN(
		PINCTRL_PIN(56, "URTS1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 56),
		MTK_FUNCTION(0, "GPIO56"),
		MTK_FUNCTION(1, "URTS1"),
		MTK_FUNCTION(2, "KPCOL2"),
		MTK_FUNCTION(3, "PWM_2"),
		MTK_FUNCTION(4, "DSP_URXD"),
		MTK_FUNCTION(5, "CONN_MCU_DBGACK_N"),
		MTK_FUNCTION(6, "ANT_SEL2"),
		MTK_FUNCTION(7, "DBG_MON_B_10")
	),
	MTK_PIN(
		PINCTRL_PIN(57, "UCTS1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 57),
		MTK_FUNCTION(0, "GPIO57"),
		MTK_FUNCTION(1, "UCTS1"),
		MTK_FUNCTION(2, "KPCOL3"),
		MTK_FUNCTION(3, "PWM_3"),
		MTK_FUNCTION(4, "DSP_UTXD"),
		MTK_FUNCTION(5, "CONN_MCU_DBGI_N"),
		MTK_FUNCTION(6, "ANT_SEL3"),
		MTK_FUNCTION(7, "DBG_MON_B_11")
	),
	MTK_PIN(
		PINCTRL_PIN(58, "RTC32K_CK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 58),
		MTK_FUNCTION(0, "GPIO58"),
		MTK_FUNCTION(1, "RTC32K_CK"),
		MTK_FUNCTION(3, "PWM_4")
	),
	MTK_PIN(
		PINCTRL_PIN(59, "PMIC_DVS_REQ0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 59),
		MTK_FUNCTION(0, "GPIO59"),
		MTK_FUNCTION(3, "PWM_5"),
		MTK_FUNCTION(7, "DBG_MON_B_4")
	),
	MTK_PIN(
		PINCTRL_PIN(60, "PMIC_DVS_REQ1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 60),
		MTK_FUNCTION(0, "GPIO60"),
		MTK_FUNCTION(3, "PWM_6"),
		MTK_FUNCTION(4, "CONN_TCXO_REQ"),
		MTK_FUNCTION(7, "DBG_MON_B_5")
	),
	MTK_PIN(
		PINCTRL_PIN(61, "WATCHDOG"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 61),
		MTK_FUNCTION(0, "GPIO61"),
		MTK_FUNCTION(1, "WATCHDOG")
	),
	MTK_PIN(
		PINCTRL_PIN(62, "PMIC_INT"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 62),
		MTK_FUNCTION(0, "GPIO62"),
		MTK_FUNCTION(1, "SRCLKENA1")
	),
	MTK_PIN(
		PINCTRL_PIN(63, "SUSPEND"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 63),
		MTK_FUNCTION(0, "GPIO63"),
		MTK_FUNCTION(1, "SRCLKENA0")
	),
	MTK_PIN(
		PINCTRL_PIN(64, "SDA0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 64),
		MTK_FUNCTION(0, "GPIO64"),
		MTK_FUNCTION(1, "SDA0_0"),
		MTK_FUNCTION(2, "SDA1_0"),
		MTK_FUNCTION(7, "DBG_MON_A_29")
	),
	MTK_PIN(
		PINCTRL_PIN(65, "SCL0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 65),
		MTK_FUNCTION(0, "GPIO65"),
		MTK_FUNCTION(1, "SCL0_0"),
		MTK_FUNCTION(2, "SCL1_0"),
		MTK_FUNCTION(7, "DBG_MON_A_30")
	),
	MTK_PIN(
		PINCTRL_PIN(66, "SDA1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 66),
		MTK_FUNCTION(0, "GPIO66"),
		MTK_FUNCTION(1, "SDA1_0"),
		MTK_FUNCTION(2, "SDA0_0"),
		MTK_FUNCTION(6, "DBG_SDA")
	),
	MTK_PIN(
		PINCTRL_PIN(67, "SCL1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 67),
		MTK_FUNCTION(0, "GPIO67"),
		MTK_FUNCTION(1, "SCL1_0"),
		MTK_FUNCTION(2, "SCL0_0"),
		MTK_FUNCTION(6, "DBG_SCL"),
		MTK_FUNCTION(7, "DBG_MON_B_0")
	),
	MTK_PIN(
		PINCTRL_PIN(68, "SDA2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 68),
		MTK_FUNCTION(0, "GPIO68"),
		MTK_FUNCTION(1, "SDA2_0"),
		MTK_FUNCTION(2, "SDA1_0"),
		MTK_FUNCTION(7, "DBG_MON_B_25")
	),
	MTK_PIN(
		PINCTRL_PIN(69, "SCL2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 69),
		MTK_FUNCTION(0, "GPIO69"),
		MTK_FUNCTION(1, "SCL2_0"),
		MTK_FUNCTION(2, "SCL1_0"),
		MTK_FUNCTION(7, "DBG_MON_B_26")
	),
	MTK_PIN(
		PINCTRL_PIN(70, "MSDC1_CMD"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 70),
		MTK_FUNCTION(0, "GPIO70"),
		MTK_FUNCTION(1, "MSDC1_CMD"),
		MTK_FUNCTION(3, "UDI_TMS_XI"),
		MTK_FUNCTION(4, "I2SO_BCK"),
		MTK_FUNCTION(5, "DMIC_CLK0"),
		MTK_FUNCTION(6, "DFD_TMS_XI"),
		MTK_FUNCTION(7, "ADSP_JTAG_TMS")
	),
	MTK_PIN(
		PINCTRL_PIN(71, "MSDC1_CLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 71),
		MTK_FUNCTION(0, "GPIO71"),
		MTK_FUNCTION(1, "MSDC1_CLK"),
		MTK_FUNCTION(3, "UDI_TCK_XI"),
		MTK_FUNCTION(4, "I2SO_LRCK"),
		MTK_FUNCTION(5, "DMIC_DAT0"),
		MTK_FUNCTION(6, "DFD_TCK_XI"),
		MTK_FUNCTION(7, "ADSP_JTAG_TCK")
	),
	MTK_PIN(
		PINCTRL_PIN(72, "MSDC1_DAT0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 72),
		MTK_FUNCTION(0, "GPIO72"),
		MTK_FUNCTION(1, "MSDC1_DAT0"),
		MTK_FUNCTION(3, "UDI_TDI_XI"),
		MTK_FUNCTION(4, "I2SO_MCK"),
		MTK_FUNCTION(5, "DMIC_DAT1"),
		MTK_FUNCTION(6, "DFD_TDI_XI"),
		MTK_FUNCTION(7, "ADSP_JTAG_TDI")
	),
	MTK_PIN(
		PINCTRL_PIN(73, "MSDC1_DAT1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 73),
		MTK_FUNCTION(0, "GPIO73"),
		MTK_FUNCTION(1, "MSDC1_DAT1"),
		MTK_FUNCTION(3, "UDI_TDO"),
		MTK_FUNCTION(4, "I2SO_DAT0"),
		MTK_FUNCTION(5, "DMIC_CLK1"),
		MTK_FUNCTION(6, "DFD_TDO"),
		MTK_FUNCTION(7, "ADSP_JTAG_TDO")
	),
	MTK_PIN(
		PINCTRL_PIN(74, "MSDC1_DAT2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 74),
		MTK_FUNCTION(0, "GPIO74"),
		MTK_FUNCTION(1, "MSDC1_DAT2"),
		MTK_FUNCTION(3, "UDI_NTRST_XI"),
		MTK_FUNCTION(4, "I2SO_DAT1"),
		MTK_FUNCTION(5, "DMIC_DAT2"),
		MTK_FUNCTION(6, "DFD_NTRST_XI"),
		MTK_FUNCTION(7, "ADSP_JTAG_TRST")
	),
	MTK_PIN(
		PINCTRL_PIN(75, "MSDC1_DAT3"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 75),
		MTK_FUNCTION(0, "GPIO75"),
		MTK_FUNCTION(1, "MSDC1_DAT3"),
		MTK_FUNCTION(4, "I2SO_DAT2"),
		MTK_FUNCTION(5, "DMIC_DAT3")
	),
	MTK_PIN(
		PINCTRL_PIN(76, "MSDC0_DAT7"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 76),
		MTK_FUNCTION(0, "GPIO76"),
		MTK_FUNCTION(1, "MSDC0_DAT7"),
		MTK_FUNCTION(2, "SPI_DAT0_NOR"),
		MTK_FUNCTION(3, "SPI_DAT0_NAND"),
		MTK_FUNCTION(5, "CONN_MCU_AICE_TMSC")
	),
	MTK_PIN(
		PINCTRL_PIN(77, "MSDC0_DAT6"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 77),
		MTK_FUNCTION(0, "GPIO77"),
		MTK_FUNCTION(1, "MSDC0_DAT6"),
		MTK_FUNCTION(2, "SPI_DAT1_NOR"),
		MTK_FUNCTION(3, "SPI_DAT1_NAND"),
		MTK_FUNCTION(5, "CONN_MCU_AICE_TCKC"),
		MTK_FUNCTION(7, "DBG_MON_B_1")
	),
	MTK_PIN(
		PINCTRL_PIN(78, "MSDC0_DAT5"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 78),
		MTK_FUNCTION(0, "GPIO78"),
		MTK_FUNCTION(1, "MSDC0_DAT5"),
		MTK_FUNCTION(2, "SPI_DAT2_NOR"),
		MTK_FUNCTION(3, "SPI_DAT2_NAND"),
		MTK_FUNCTION(7, "DBG_MON_B_27")
	),
	MTK_PIN(
		PINCTRL_PIN(79, "MSDC0_DAT4"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 79),
		MTK_FUNCTION(0, "GPIO79"),
		MTK_FUNCTION(1, "MSDC0_DAT4"),
		MTK_FUNCTION(2, "SPI_DAT3_NOR"),
		MTK_FUNCTION(3, "SPI_DAT3_NAND"),
		MTK_FUNCTION(7, "DBG_MON_B_28")
	),
	MTK_PIN(
		PINCTRL_PIN(80, "MSDC0_RSTB"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 80),
		MTK_FUNCTION(0, "GPIO80"),
		MTK_FUNCTION(1, "MSDC0_RSTB"),
		MTK_FUNCTION(2, "SPI_CLK_NOR"),
		MTK_FUNCTION(3, "SPI_CSB_NAND")
	),
	MTK_PIN(
		PINCTRL_PIN(81, "MSDC0_CMD"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 81),
		MTK_FUNCTION(0, "GPIO81"),
		MTK_FUNCTION(1, "MSDC0_CMD"),
		MTK_FUNCTION(5, "IRRX"),
		MTK_FUNCTION(7, "DBG_MON_B_29")
	),
	MTK_PIN(
		PINCTRL_PIN(82, "MSDC0_CLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 82),
		MTK_FUNCTION(0, "GPIO82"),
		MTK_FUNCTION(1, "MSDC0_CLK"),
		MTK_FUNCTION(2, "SPI_CSB_NOR"),
		MTK_FUNCTION(3, "SPI_CLK_NAND")
	),
	MTK_PIN(
		PINCTRL_PIN(83, "MSDC0_DAT3"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 83),
		MTK_FUNCTION(0, "GPIO83"),
		MTK_FUNCTION(1, "MSDC0_DAT3"),
		MTK_FUNCTION(2, "KPROW0"),
		MTK_FUNCTION(3, "PWM_3"),
		MTK_FUNCTION(7, "DBG_MON_B_30")
	),
	MTK_PIN(
		PINCTRL_PIN(84, "MSDC0_DAT2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 84),
		MTK_FUNCTION(0, "GPIO84"),
		MTK_FUNCTION(1, "MSDC0_DAT2"),
		MTK_FUNCTION(2, "KPROW1"),
		MTK_FUNCTION(3, "PWM_4"),
		MTK_FUNCTION(7, "DBG_MON_B_13")
	),
	MTK_PIN(
		PINCTRL_PIN(85, "MSDC0_DAT1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 85),
		MTK_FUNCTION(0, "GPIO85"),
		MTK_FUNCTION(1, "MSDC0_DAT1"),
		MTK_FUNCTION(2, "KPCOL0"),
		MTK_FUNCTION(3, "PWM_5"),
		MTK_FUNCTION(7, "DBG_MON_B_31")
	),
	MTK_PIN(
		PINCTRL_PIN(86, "MSDC0_DAT0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 86),
		MTK_FUNCTION(0, "GPIO86"),
		MTK_FUNCTION(1, "MSDC0_DAT0"),
		MTK_FUNCTION(2, "KPCOL1"),
		MTK_FUNCTION(3, "PWM_6"),
		MTK_FUNCTION(7, "DBG_MON_B_32")
	),
	MTK_PIN(
		PINCTRL_PIN(87, "SPDIF"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 87),
		MTK_FUNCTION(0, "GPIO87"),
		MTK_FUNCTION(1, "SPDIF_IN0"),
		MTK_FUNCTION(2, "SPDIF_IN1"),
		MTK_FUNCTION(3, "EPDC_D9"),
		MTK_FUNCTION(4, "SPDIF_IN2"),
		MTK_FUNCTION(5, "IRRX"),
		MTK_FUNCTION(7, "DBG_MON_A_24")
	),
	MTK_PIN(
		PINCTRL_PIN(88, "PCM_CLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 88),
		MTK_FUNCTION(0, "GPIO88"),
		MTK_FUNCTION(2, "PCM_CLK"),
		MTK_FUNCTION(3, "CONN_TOP_CLK")
	),
	MTK_PIN(
		PINCTRL_PIN(89, "PCM_SYNC"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 89),
		MTK_FUNCTION(0, "GPIO89"),
		MTK_FUNCTION(2, "PCM_SYNC"),
		MTK_FUNCTION(3, "CONN_WB_PTA")
	),
	MTK_PIN(
		PINCTRL_PIN(90, "PCM_RX"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 90),
		MTK_FUNCTION(0, "GPIO90"),
		MTK_FUNCTION(2, "PCM_RX"),
		MTK_FUNCTION(3, "CONN_TOP_DATA")
	),
	MTK_PIN(
		PINCTRL_PIN(91, "PCM_TX"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 91),
		MTK_FUNCTION(0, "GPIO91"),
		MTK_FUNCTION(1, "IOSEL_DBGOUT"),
		MTK_FUNCTION(2, "PCM_TX"),
		MTK_FUNCTION(3, "CONN_HRST_B")
	),
	MTK_PIN(
		PINCTRL_PIN(92, "I2SIN_MCLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 92),
		MTK_FUNCTION(0, "GPIO92"),
		MTK_FUNCTION(1, "I2SIN_MCK"),
		MTK_FUNCTION(2, "TDMIN_MCLK"),
		MTK_FUNCTION(3, "EPDC_D15")
	),
	MTK_PIN(
		PINCTRL_PIN(93, "I2SIN_LRCK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 93),
		MTK_FUNCTION(0, "GPIO93"),
		MTK_FUNCTION(1, "I2SIN_LRCK"),
		MTK_FUNCTION(3, "EPDC_GDOE"),
		MTK_FUNCTION(4, "SPLIN_LRCK")
	),
	MTK_PIN(
		PINCTRL_PIN(94, "I2SIN_BCK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 94),
		MTK_FUNCTION(0, "GPIO94"),
		MTK_FUNCTION(1, "I2SIN_BCK"),
		MTK_FUNCTION(3, "EPDC_GDSP"),
		MTK_FUNCTION(4, "SPLIN_BCK")
	),
	MTK_PIN(
		PINCTRL_PIN(95, "I2SIN_DAT0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 95),
		MTK_FUNCTION(0, "GPIO95"),
		MTK_FUNCTION(1, "I2SIN_DAT0"),
		MTK_FUNCTION(3, "EPDC_D12"),
		MTK_FUNCTION(4, "SPLIN_D0")
	),
	MTK_PIN(
		PINCTRL_PIN(96, "I2SIN_DAT1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 96),
		MTK_FUNCTION(0, "GPIO96"),
		MTK_FUNCTION(1, "I2SIN_DAT1"),
		MTK_FUNCTION(2, "TDMIN_BCK"),
		MTK_FUNCTION(3, "EPDC_D10"),
		MTK_FUNCTION(4, "SPLIN_D1"),
		MTK_FUNCTION(7, "DBG_MON_A_15")
	),
	MTK_PIN(
		PINCTRL_PIN(97, "I2SIN_DAT2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 97),
		MTK_FUNCTION(0, "GPIO97"),
		MTK_FUNCTION(1, "I2SIN_DAT2"),
		MTK_FUNCTION(2, "TDMIN_LRCK"),
		MTK_FUNCTION(3, "EPDC_D8"),
		MTK_FUNCTION(4, "SPLIN_D2"),
		MTK_FUNCTION(7, "DBG_MON_A_16")
	),
	MTK_PIN(
		PINCTRL_PIN(98, "I2SIN_DAT3"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 98),
		MTK_FUNCTION(0, "GPIO98"),
		MTK_FUNCTION(1, "I2SIN_DAT3"),
		MTK_FUNCTION(2, "TDMIN_DI"),
		MTK_FUNCTION(3, "EPDC_D6"),
		MTK_FUNCTION(4, "SPLIN_D3"),
		MTK_FUNCTION(7, "DBG_MON_A_17")
	),
	MTK_PIN(
		PINCTRL_PIN(99, "DMIC0_CLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 99),
		MTK_FUNCTION(0, "GPIO99"),
		MTK_FUNCTION(1, "DMIC_CLK0"),
		MTK_FUNCTION(3, "EPDC_GDCLK"),
		MTK_FUNCTION(7, "DBG_MON_A_18")
	),
	MTK_PIN(
		PINCTRL_PIN(100, "DMIC0_DAT0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 100),
		MTK_FUNCTION(0, "GPIO100"),
		MTK_FUNCTION(1, "DMIC_DAT0"),
		MTK_FUNCTION(7, "DBG_MON_A_19")
	),
	MTK_PIN(
		PINCTRL_PIN(101, "DMIC0_DAT1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 101),
		MTK_FUNCTION(0, "GPIO101"),
		MTK_FUNCTION(1, "DMIC_DAT1"),
		MTK_FUNCTION(2, "TDMIN_DI"),
		MTK_FUNCTION(3, "EPDC_D14"),
		MTK_FUNCTION(7, "DBG_MON_A_20")
	),
	MTK_PIN(
		PINCTRL_PIN(102, "DMIC1_CLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 102),
		MTK_FUNCTION(0, "GPIO102"),
		MTK_FUNCTION(1, "DMIC_CLK1"),
		MTK_FUNCTION(2, "TDMIN_MCLK"),
		MTK_FUNCTION(3, "EPDC_SDCE1"),
		MTK_FUNCTION(7, "DBG_MON_A_21")
	),
	MTK_PIN(
		PINCTRL_PIN(103, "DMIC1_DAT0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 103),
		MTK_FUNCTION(0, "GPIO103"),
		MTK_FUNCTION(1, "DMIC_DAT2"),
		MTK_FUNCTION(2, "TDMIN_BCK"),
		MTK_FUNCTION(3, "EPDC_SDCE2"),
		MTK_FUNCTION(7, "DBG_MON_A_22")
	),
	MTK_PIN(
		PINCTRL_PIN(104, "DMIC1_DAT1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 104),
		MTK_FUNCTION(0, "GPIO104"),
		MTK_FUNCTION(1, "DMIC_DAT3"),
		MTK_FUNCTION(2, "TDMIN_LRCK"),
		MTK_FUNCTION(3, "EPDC_SDCE3"),
		MTK_FUNCTION(7, "DBG_MON_A_23")
	),
	MTK_PIN(
		PINCTRL_PIN(105, "I2SO_BCK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 105),
		MTK_FUNCTION(0, "GPIO105"),
		MTK_FUNCTION(1, "I2SO_BCK"),
		MTK_FUNCTION(3, "EPDC_D5"),
		MTK_FUNCTION(7, "DBG_MON_A_8")
	),
	MTK_PIN(
		PINCTRL_PIN(106, "I2SO_LRCK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 106),
		MTK_FUNCTION(0, "GPIO106"),
		MTK_FUNCTION(1, "I2SO_LRCK"),
		MTK_FUNCTION(3, "EPDC_D7"),
		MTK_FUNCTION(7, "DBG_MON_A_9")
	),
	MTK_PIN(
		PINCTRL_PIN(107, "I2SO_MCLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 107),
		MTK_FUNCTION(0, "GPIO107"),
		MTK_FUNCTION(1, "I2SO_MCK"),
		MTK_FUNCTION(3, "EPDC_SDCLK"),
		MTK_FUNCTION(7, "DBG_MON_A_10")
	),
	MTK_PIN(
		PINCTRL_PIN(108, "I2SO_DAT0"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 108),
		MTK_FUNCTION(0, "GPIO108"),
		MTK_FUNCTION(1, "I2SO_DAT0"),
		MTK_FUNCTION(3, "EPDC_D3"),
		MTK_FUNCTION(7, "DBG_MON_A_11")
	),
	MTK_PIN(
		PINCTRL_PIN(109, "I2SO_DAT1"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 109),
		MTK_FUNCTION(0, "GPIO109"),
		MTK_FUNCTION(1, "I2SO_DAT1"),
		MTK_FUNCTION(3, "EPDC_D1"),
		MTK_FUNCTION(7, "DBG_MON_A_12")
	),
	MTK_PIN(
		PINCTRL_PIN(110, "I2SO_DAT2"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 110),
		MTK_FUNCTION(0, "GPIO110"),
		MTK_FUNCTION(1, "I2SO_DAT2"),
		MTK_FUNCTION(2, "SDA0_0"),
		MTK_FUNCTION(3, "EPDC_SDCE0"),
		MTK_FUNCTION(4, "URXD1"),
		MTK_FUNCTION(7, "DBG_MON_A_13")
	),
	MTK_PIN(
		PINCTRL_PIN(111, "I2SO_DAT3"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 111),
		MTK_FUNCTION(0, "GPIO111"),
		MTK_FUNCTION(1, "I2SO_DAT3"),
		MTK_FUNCTION(2, "SCL0_0"),
		MTK_FUNCTION(3, "EPDC_SDLE"),
		MTK_FUNCTION(4, "UTXD1"),
		MTK_FUNCTION(7, "DBG_MON_A_14")
	),
	MTK_PIN(
		PINCTRL_PIN(112, "SPI_CSB"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 112),
		MTK_FUNCTION(0, "GPIO112"),
		MTK_FUNCTION(1, "SPI_CSB"),
		MTK_FUNCTION(3, "SPI_CSB_NOR"),
		MTK_FUNCTION(4, "JTMS"),
		MTK_FUNCTION(5, "I2SO_BCK")
	),
	MTK_PIN(
		PINCTRL_PIN(113, "SPI_CLK"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 113),
		MTK_FUNCTION(0, "GPIO113"),
		MTK_FUNCTION(1, "SPI_CLK"),
		MTK_FUNCTION(3, "SPI_CLK_NOR"),
		MTK_FUNCTION(4, "JTCK"),
		MTK_FUNCTION(5, "I2SO_LRCK")
	),
	MTK_PIN(
		PINCTRL_PIN(114, "SPI_MISO"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 114),
		MTK_FUNCTION(0, "GPIO114"),
		MTK_FUNCTION(1, "SPI_MI"),
		MTK_FUNCTION(3, "SPI_DAT0_NOR"),
		MTK_FUNCTION(4, "JTDI"),
		MTK_FUNCTION(5, "I2SO_MCK")
	),
	MTK_PIN(
		PINCTRL_PIN(115, "SPI_MOSI"),
		NULL, "mt8512",
		MTK_EINT_FUNCTION(0, 115),
		MTK_FUNCTION(0, "GPIO115"),
		MTK_FUNCTION(1, "SPI_MO"),
		MTK_FUNCTION(3, "SPI_DAT1_NOR"),
		MTK_FUNCTION(4, "JTDO"),
		MTK_FUNCTION(5, "I2SO_DAT0")
	),
};

#endif/* __PINCTRL_MTK_MT8512_H */
