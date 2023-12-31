/*
 * ALSA TLV320ADC5140 Sound driver
 *
 * Author: C. Omer Rafique <rafiquec@amazon.com>
 *
 * Based on draft code from Dan Murphy <dmurphy@ti.com>
 *
 * Copyright (C) 2018 Amazon Lab126 Inc - http://www.lab126.com/
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 * This TLV320ADC5140 driver supports upto two codec chips which can be
 * configured as Slave-Slave or Master-Slave on TDM bus.
 *
 */

#ifndef _TLV320ADC5140_H
#define _TLV320ADC5140_H

#define ADC5140_RATES	SNDRV_PCM_RATE_8000_192000

#define ADC5140_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_3LE | \
			 SNDRV_PCM_FMTBIT_S24_LE | \
			 SNDRV_PCM_FMTBIT_S32_LE)

#define ADC5140_PAGE_SELECT	0x00
#define ADC5140_SW_RESET	0x01
#define ADC5140_SLEEP_CFG	0x02
#define ADC5140_SHDN_CFG	0x05
#define ADC5140_ASI_CFG0	0x07
#define ADC5140_ASI_CFG1	0x08
#define ADC5140_ASI_CFG2	0x09
#define ADC5140_ASI_CH1	0x0b
#define ADC5140_ASI_CH2	0x0c
#define ADC5140_ASI_CH3	0x0d
#define ADC5140_ASI_CH4	0x0e
#define ADC5140_ASI_CH5	0x0f
#define ADC5140_ASI_CH6	0x10
#define ADC5140_ASI_CH7	0x11
#define ADC5140_ASI_CH8	0x12
#define ADC5140_MST_CFG0	0x13
#define ADC5140_MST_CFG1	0x14
#define ADC5140_ASI_STS		0x15
#define ADC5140_PDMCLK_CFG	0x1f
#define ADC5140_PDM_CFG		0x20
#define ADC5140_GPIO_CFG0	0x21
#define ADC5140_GPIO_CFG1	0x22
#define ADC5140_GPIO_CFG2	0x23
#define ADC5140_GPIO_CFG3	0x24
#define ADC5140_GPIO_CFG4	0x25
#define ADC5140_GPIO_VAL	0x29
#define ADC5140_GPIO_MON	0x2a
#define ADC5140_GPI_CFG0	0x2b
#define ADC5140_GPI_CFG1	0x2c
#define ADC5140_GPI_MON	0x2f
#define ADC5140_INT_CFG	0x32
#define ADC5140_INT_MASK0	0x33
#define ADC5140_INT_LTCH0	0x36
#define ADC5140_BIAS_CFG	0x3b
#define ADC5140_CH1_CFG0	0x3c
#define ADC5140_CH1_CFG1	0x3d
#define ADC5140_CH1_CFG2	0x3e
#define ADC5140_CH1_CFG3	0x3f
#define ADC5140_CH1_CFG4	0x40
#define ADC5140_CH2_CFG0	0x41
#define ADC5140_CH2_CFG1	0x42
#define ADC5140_CH2_CFG2	0x43
#define ADC5140_CH2_CFG3	0x44
#define ADC5140_CH2_CFG4	0x45
#define ADC5140_CH3_CFG0	0x46
#define ADC5140_CH3_CFG1	0x47
#define ADC5140_CH3_CFG2	0x48
#define ADC5140_CH3_CFG3	0x49
#define ADC5140_CH3_CFG4	0x4a
#define ADC5140_CH4_CFG0	0x4b
#define ADC5140_CH4_CFG1	0x4c
#define ADC5140_CH4_CFG2	0x4d
#define ADC5140_CH4_CFG3	0x4e
#define ADC5140_CH4_CFG4	0x4f
#define ADC5140_CH5_CFG0	0x50
#define ADC5140_CH5_CFG1	0x51
#define ADC5140_CH5_CFG2	0x52
#define ADC5140_CH5_CFG3	0x53
#define ADC5140_CH5_CFG4	0x54
#define ADC5140_CH6_CFG0	0x55
#define ADC5140_CH6_CFG1	0x56
#define ADC5140_CH6_CFG2	0x57
#define ADC5140_CH6_CFG3	0x58
#define ADC5140_CH6_CFG4	0x59
#define ADC5140_CH7_CFG0	0x5a
#define ADC5140_CH7_CFG1	0x5b
#define ADC5140_CH7_CFG2	0x5c
#define ADC5140_CH7_CFG3	0x5d
#define ADC5140_CH7_CFG4	0x5e
#define ADC5140_CH8_CFG0	0x5f
#define ADC5140_CH8_CFG1	0x60
#define ADC5140_CH8_CFG2	0x61
#define ADC5140_CH8_CFG3	0x62
#define ADC5140_CH8_CFG4	0x63
#define ADC5140_DSP_CFG0	0x6b
#define ADC5140_DSP_CFG1	0x6c
#define ADC5140_DRE_CFG0	0x6d
#define ADC5140_IN_CH_EN	0x73
#define ADC5140_ASI_OUT_CH_EN	0x74
#define ADC5140_PWR_CFG		0x75
#define ADC5140_DEV_STS0	0x76
#define ADC5140_DEV_STS1	0x77

#define ADC5140_FREQ_12000000 12000000
#define ADC5140_FREQ_12288000 12288000
#define ADC5140_FREQ_13000000 13000000
#define ADC5140_FREQ_16000000 16000000
#define ADC5140_FREQ_18500000 18500000
#define ADC5140_FREQ_19200000 19200000
#define ADC5140_FREQ_19268000 19268000
#define ADC5140_FREQ_24000000 24000000
#define ADC5140_FREQ_24576000 24576000

void adc5140_power_down(struct snd_soc_dai *codec_dai);

#endif /* _TLV320ADC5140_ */
