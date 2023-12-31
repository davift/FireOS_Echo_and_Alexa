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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tlv320adc5140.h"

#define ADC5140_RESET   0x01

#define ADC5140_BCLKINV_BIT 0x04
#define ADC5140_BCLK_FSYNC_MASTER   0x80
#define ADC5140_I2S_MODE_BIT    0x40
#define ADC5140_LEFT_JUST_BIT   0x80
#define ADC5140_ASI_FMT_MSK 0xC0
#define ADC5140_MCLK_MSK    0x07
#define ADC5140_FS_RATE_SHIFT   4
#define ADC5140_MCLK_INPUT 0xA2

#define ADC5140_20_BIT_WORD 0x10
#define ADC5140_24_BIT_WORD 0x20
#define ADC5140_32_BIT_WORD 0x30
#define ADC5140_WORD_LEN_MSK 0x30
#define ADC5140_TX_FILL 0x01
#define ADC5140_TX_EDGE_MSK 0x02
#define ADC5140_TX_EDGE_ENABLE 0x02

#define ADC5140_TX_CFG1_MSK 0xE0
#define ADC5140_CH_SLOT_MSK 0x1F

#define ADC5140_PWR_DOWN 0x00
#define ADC5140_PWR_MASK_ADC 0x40
#define ADC5140_PWR_MASK_PLL 0x20
#define ADC5140_PWR_UP 0x60
#define ADC5140_PWR_ON 0xf0

#define ADC5140_CH_INPUT_EN 0xf0
#define ADC5140_CH_OUTPUT_EN 0xf0

#define ADC5140_BUS_KEEPER 0x20
#define ADC5140_LSB_HI_Z 0x80

#define ADC5140_WAKE_UP 0x81
#define ADC5140_SLEEP_MODE 0x80

#define ADC5140_DREG_PWR_MSK 0x0C
#define ADC5140_DREG_OFF_IMMEDIATE 0

#define SW_RESET_ENABLED 0
#define SHUTDOWN_ENABLED 1

/* Private struct for internal data */
struct adc5140_priv {
	struct regulator *reset_adc_regulator;
	struct regmap *regmap;
	struct device *dev;
	int adc_num;
	bool disable_suspend;
};

static const struct reg_default adc5140_reg_defaults[] = {
	{ADC5140_PAGE_SELECT, 0x00},
};

static int adc5140_reset_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol);
static int adc5140_reset_set(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol);

static const struct regmap_range_cfg adc5140_ranges[] = {
	{
		.range_min = 0,
		.range_max = 12 * 128,
		.selector_reg = ADC5140_PAGE_SELECT,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static const struct regmap_config adc5140_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_defaults = adc5140_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(adc5140_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
	.ranges = adc5140_ranges,
	.num_ranges = ARRAY_SIZE(adc5140_ranges),
	.max_register = 12 * 128,
};

static const char * const mic_select_text[] = {
	"2.5 kOhm", "10 kOhm", "20 kOhm"
};

static const char * const input_config_text[] = {
	"Diff", "Single", "Digital"
};

static const DECLARE_TLV_DB_MINMAX_MUTE(adc_fgain_tlv, -10000, 2700);
static const DECLARE_TLV_DB_LINEAR(adc_chgain_tlv, 0, 4200);

static const struct soc_enum adc5140_enum[] = {
	SOC_ENUM_SINGLE(ADC5140_CH1_CFG0, 2, ARRAY_SIZE(mic_select_text),
		mic_select_text),
	SOC_ENUM_SINGLE(ADC5140_CH2_CFG0, 2, ARRAY_SIZE(mic_select_text),
		mic_select_text),
	SOC_ENUM_SINGLE(ADC5140_CH3_CFG0, 2, ARRAY_SIZE(mic_select_text),
		mic_select_text),
	SOC_ENUM_SINGLE(ADC5140_CH4_CFG0, 2, ARRAY_SIZE(mic_select_text),
		mic_select_text),
	SOC_ENUM_SINGLE(ADC5140_CH1_CFG0, 4, ARRAY_SIZE(input_config_text),
		input_config_text),
	SOC_ENUM_SINGLE(ADC5140_CH2_CFG0, 4, ARRAY_SIZE(input_config_text),
		input_config_text),
	SOC_ENUM_SINGLE(ADC5140_CH3_CFG0, 4, ARRAY_SIZE(input_config_text),
		input_config_text),
	SOC_ENUM_SINGLE(ADC5140_CH4_CFG0, 4, ARRAY_SIZE(input_config_text),
		input_config_text),
};

static const struct snd_soc_dapm_widget adc5140_dapm_widgets_a[] = {
	/* Inputs */
	SND_SOC_DAPM_INPUT("MIC1"),

	SND_SOC_DAPM_ADC("ADC", "Capture", 0, 0, 0),
};

static const struct snd_soc_dapm_widget adc5140_dapm_widgets_b[] = {
	/* Inputs */
	SND_SOC_DAPM_INPUT("MIC1"),

	SND_SOC_DAPM_ADC("ADC", "Capture", 0, 0, 0),
};

static const struct snd_soc_dapm_route adc5140_audio_map_a[] = {
	/* Mic input */
	{"MIC1", NULL, "ADC"},
};

static const struct snd_soc_dapm_route adc5140_audio_map_b[] = {
	/* Mic input */
	{"MIC1", NULL, "ADC"},
};

static const struct snd_kcontrol_new adc5140_snd_controls_a[] = {
	SOC_SINGLE_TLV("Ch1 Digital Capture Vol A", ADC5140_CH1_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch2 Digital Capture Vol A", ADC5140_CH2_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch3 Digital Capture Vol A", ADC5140_CH3_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch4 Digital Capture Vol A", ADC5140_CH4_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch5 Digital Capture Vol A", ADC5140_CH5_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch6 Digital Capture Vol A", ADC5140_CH6_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch7 Digital Capture Vol A", ADC5140_CH7_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch8 Digital Capture Vol A", ADC5140_CH8_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),

	/* Channel Gain - 0 to +42db */
	SOC_SINGLE_TLV("Ch1 Gain A", ADC5140_CH1_CFG1, 2, 42, 0,
		       adc_chgain_tlv),
	SOC_SINGLE_TLV("Ch2 Gain A", ADC5140_CH2_CFG1, 2, 42, 0,
		       adc_chgain_tlv),
	SOC_SINGLE_TLV("Ch3 Gain A", ADC5140_CH3_CFG1, 2, 42, 0,
		       adc_chgain_tlv),
	SOC_SINGLE_TLV("Ch4 Gain A", ADC5140_CH4_CFG1, 2, 42, 0,
		       adc_chgain_tlv),

	/* Input Impedance (Applicable for Analog Input)*/
	SOC_ENUM("Ch1 Input Impedance A", adc5140_enum[0]),
	SOC_ENUM("Ch2 Input Impedance A", adc5140_enum[1]),
	SOC_ENUM("Ch3 Input Impedance A", adc5140_enum[2]),
	SOC_ENUM("Ch4 Input Impedance A", adc5140_enum[3]),

	/* Input Configuration. Differential, Single-Ended, Digital */
	SOC_ENUM("Ch1 Input Config A", adc5140_enum[4]),
	SOC_ENUM("Ch2 Input Config A", adc5140_enum[5]),
	SOC_ENUM("Ch3 Input Config A", adc5140_enum[6]),
	SOC_ENUM("Ch4 Input Config A", adc5140_enum[7]),

	/* DRE - AGC Enabler */
	SOC_SINGLE("Ch1 DRE Enable A", ADC5140_CH1_CFG0, 0, 1, 0),
	SOC_SINGLE("Ch2 DRE Enable A", ADC5140_CH2_CFG0, 0, 1, 0),
	SOC_SINGLE("Ch3 DRE Enable A", ADC5140_CH3_CFG0, 0, 1, 0),
	SOC_SINGLE("Ch4 DRE Enable A", ADC5140_CH4_CFG0, 0, 1, 0),

	/* DRE - Params */
	SOC_SINGLE("DRE Trigger Level A", ADC5140_DRE_CFG0, 4, 15, 0),
	SOC_SINGLE("DRE Max Gain A", ADC5140_DRE_CFG0, 0, 15, 0),
#ifdef DEBUG_CHANNELS
	/* Input Channel */
	SOC_SINGLE("Ch1 Input Enable A", ADC5140_IN_CH_EN, 7, 1, 0),
	SOC_SINGLE("Ch2 Input Enable A", ADC5140_IN_CH_EN, 6, 1, 0),
	SOC_SINGLE("Ch3 Input Enable A", ADC5140_IN_CH_EN, 5, 1, 0),
	SOC_SINGLE("Ch4 Input Enable A", ADC5140_IN_CH_EN, 4, 1, 0),

	/* Output Slot Enable */
	SOC_SINGLE("Ch1 Slot Enable A", ADC5140_ASI_OUT_CH_EN, 7, 1, 0),
	SOC_SINGLE("Ch2 Slot Enable A", ADC5140_ASI_OUT_CH_EN, 6, 1, 0),
	SOC_SINGLE("Ch3 Slot Enable A", ADC5140_ASI_OUT_CH_EN, 5, 1, 0),
	SOC_SINGLE("Ch4 Slot Enable A", ADC5140_ASI_OUT_CH_EN, 4, 1, 0),
#endif
	/* Slot selection on TDM bus */
	SOC_SINGLE("Ch1 Slot Offset A", ADC5140_ASI_CH1, 0, 63, 0),
	SOC_SINGLE("Ch2 Slot Offset A", ADC5140_ASI_CH2, 0, 63, 0),
	SOC_SINGLE("Ch3 Slot Offset A", ADC5140_ASI_CH3, 0, 63, 0),
	SOC_SINGLE("Ch4 Slot Offset A", ADC5140_ASI_CH4, 0, 63, 0),

	/* Reset codecs */
	SOC_SINGLE_BOOL_EXT("ADC Reset A", 0, adc5140_reset_get,
		adc5140_reset_set),
};

static const struct snd_kcontrol_new adc5140_snd_controls_b[] = {
	SOC_SINGLE_TLV("Ch1 Digital Capture Vol B", ADC5140_CH1_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch2 Digital Capture Vol B", ADC5140_CH2_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch3 Digital Capture Vol B", ADC5140_CH3_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch4 Digital Capture Vol B", ADC5140_CH4_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch5 Digital Capture Vol B", ADC5140_CH5_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch6 Digital Capture Vol B", ADC5140_CH6_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch7 Digital Capture Vol B", ADC5140_CH7_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),
	SOC_SINGLE_TLV("Ch8 Digital Capture Vol B", ADC5140_CH8_CFG2,
			0, 0xFF, 0, adc_fgain_tlv),

	/* Channel Gain - 0 to +42db */
	SOC_SINGLE_TLV("Ch1 Gain B", ADC5140_CH1_CFG1, 2, 42, 0,
		       adc_chgain_tlv),
	SOC_SINGLE_TLV("Ch2 Gain B", ADC5140_CH2_CFG1, 2, 42, 0,
		       adc_chgain_tlv),
	SOC_SINGLE_TLV("Ch3 Gain B", ADC5140_CH3_CFG1, 2, 42, 0,
		       adc_chgain_tlv),
	SOC_SINGLE_TLV("Ch4 Gain B", ADC5140_CH4_CFG1, 2, 42, 0,
		       adc_chgain_tlv),

	/* Input Impedance (Applicable for Analog Input)*/
	SOC_ENUM("Ch1 Input Impedance B", adc5140_enum[0]),
	SOC_ENUM("Ch2 Input Impedance B", adc5140_enum[1]),
	SOC_ENUM("Ch3 Input Impedance B", adc5140_enum[2]),
	SOC_ENUM("Ch4 Input Impedance B", adc5140_enum[3]),

	/* Input Configuration. Differential, Single-Ended, Digital */
	SOC_ENUM("Ch1 Input Config B", adc5140_enum[4]),
	SOC_ENUM("Ch2 Input Config B", adc5140_enum[5]),
	SOC_ENUM("Ch3 Input Config B", adc5140_enum[6]),
	SOC_ENUM("Ch4 Input Config B", adc5140_enum[7]),

	/* DRE - AGC Enabler */
	SOC_SINGLE("Ch1 DRE Enable B", ADC5140_CH1_CFG0, 0, 1, 0),
	SOC_SINGLE("Ch2 DRE Enable B", ADC5140_CH2_CFG0, 0, 1, 0),
	SOC_SINGLE("Ch3 DRE Enable B", ADC5140_CH3_CFG0, 0, 1, 0),
	SOC_SINGLE("Ch4 DRE Enable B", ADC5140_CH4_CFG0, 0, 1, 0),

	/* DRE - Params */
	SOC_SINGLE("DRE Trigger Level B", ADC5140_DRE_CFG0, 4, 15, 0),
	SOC_SINGLE("DRE Max Gain B", ADC5140_DRE_CFG0, 0, 15, 0),

#ifdef DEBUG_CHANNELS
	/* Input Channel */
	SOC_SINGLE("Ch1 Input Enable B", ADC5140_IN_CH_EN, 7, 1, 0),
	SOC_SINGLE("Ch2 Input Enable B", ADC5140_IN_CH_EN, 6, 1, 0),
	SOC_SINGLE("Ch3 Input Enable B", ADC5140_IN_CH_EN, 5, 1, 0),
	SOC_SINGLE("Ch4 Input Enable B", ADC5140_IN_CH_EN, 4, 1, 0),

	/* Output Slot Enable */
	SOC_SINGLE("Ch1 Slot Enable B", ADC5140_ASI_OUT_CH_EN, 7, 1, 0),
	SOC_SINGLE("Ch2 Slot Enable B", ADC5140_ASI_OUT_CH_EN, 6, 1, 0),
	SOC_SINGLE("Ch3 Slot Enable B", ADC5140_ASI_OUT_CH_EN, 5, 1, 0),
	SOC_SINGLE("Ch4 Slot Enable B", ADC5140_ASI_OUT_CH_EN, 4, 1, 0),
#endif
	/* Slot selection on TDM bus */
	SOC_SINGLE("Ch1 Slot Offset B", ADC5140_ASI_CH1, 0, 63, 0),
	SOC_SINGLE("Ch2 Slot Offset B", ADC5140_ASI_CH2, 0, 63, 0),
	SOC_SINGLE("Ch3 Slot Offset B", ADC5140_ASI_CH3, 0, 63, 0),
	SOC_SINGLE("Ch4 Slot Offset B", ADC5140_ASI_CH4, 0, 63, 0),

	/* Reset codecs */
	SOC_SINGLE_BOOL_EXT("ADC Reset B", 0, adc5140_reset_get,
		adc5140_reset_set),
};

static void adc5140_reset(struct snd_soc_codec *codec)
{
#if SW_RESET_ENABLED
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
#endif
	dev_info(codec->dev, "%s:+\n", __func__);

	snd_soc_write(codec, ADC5140_SLEEP_CFG,
			ADC5140_WAKE_UP);
	mdelay(10);

#if SW_RESET_ENABLED
	regcache_cache_only(adc5140->regmap, true);
	regcache_mark_dirty(adc5140->regmap);

	snd_soc_write(codec, ADC5140_SW_RESET,
			ADC5140_RESET);
	mdelay(20);

	regcache_cache_only(adc5140->regmap, false);
	regcache_sync(adc5140->regmap);

	snd_soc_write(codec, ADC5140_SLEEP_CFG,
			ADC5140_WAKE_UP);
	mdelay(10);
#endif

	dev_info(codec->dev, "%s:-\n", __func__);
}

static int adc5140_reset_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int adc5140_reset_set(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s: codec %d\n", __func__, adc5140->adc_num);

	if (ucontrol->value.integer.value[0])
		adc5140_reset(codec);
	else
		dev_info(codec->dev, "%s: 0 value won't reset ADCs\n",
			__func__);

	return 0;
}

static int adc5140_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *codec_dai)
{
	struct snd_soc_component *component = codec_dai->component;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	u8 wlen = 0, fs_rate = 0, fs_bclk_ratio = 0, tx_edge = 0;
	unsigned int width = params_width(params);
	unsigned int channels = params_channels(params);
	unsigned int bclk_freq = 0;

	dev_info(component->dev, "%s+: width %d rate %d ch %d codec %d\n",
		__func__, width, params_rate(params),
		channels, adc5140->adc_num);

	switch (width) {
	case 16:
		break;
	case 20:
		wlen = ADC5140_20_BIT_WORD;
		break;
	case 24:
		wlen = ADC5140_24_BIT_WORD;
		break;
	case 32:
		wlen = ADC5140_32_BIT_WORD;
		break;
	default:
		dev_err(component->dev, "%s: Unsupported width %d\n",
			__func__, params_width(params));
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 16000:
		fs_rate = 1 << ADC5140_FS_RATE_SHIFT;
	break;
	case 96000:
		fs_rate = 5 << ADC5140_FS_RATE_SHIFT;
	break;
	case 48000:
	default:
		/* 48k is also default */
		fs_rate = 4 << ADC5140_FS_RATE_SHIFT;
	}

	/* For odd number of channels, subtract 1 for reference ch */
	if (channels % 2)
		channels--;

	switch (width * channels) {
	case 64:
		fs_bclk_ratio = 4;
		break;
	case 96:
		fs_bclk_ratio = 5;
		break;
	case 128:
		fs_bclk_ratio = 6;
		break;
	case 192:
		fs_bclk_ratio = 7;
		break;
	case 256:
		fs_bclk_ratio = 8;
		break;
	case 384:
		fs_bclk_ratio = 9;
		break;
	case 512:
		fs_bclk_ratio = 10;
		break;
	case 1024:
		fs_bclk_ratio = 11;
		break;
	case 2048:
		fs_bclk_ratio = 12;
		break;
	default:
		dev_err(component->dev, "%s: Unsupported fs_bclk_ratio %d\n",
			__func__, width * channels);
		return -EINVAL;
	}

	bclk_freq = width * channels * params_rate(params);
	if (bclk_freq > ADC5140_FREQ_18500000)
		tx_edge = ADC5140_TX_EDGE_ENABLE;

	dev_info(component->dev, "%s: wlen 0x%X rate 0x%X fs_bclk_ratio 0x%X tx_edge %d\n",
		__func__, wlen, fs_rate, fs_bclk_ratio, tx_edge);

	snd_soc_component_update_bits(component, ADC5140_ASI_CFG0,
		(ADC5140_WORD_LEN_MSK | ADC5140_TX_FILL | ADC5140_TX_EDGE_MSK),
		(wlen | ADC5140_TX_FILL) | tx_edge);

	/* Only Master needs clock rate info programmed */
	if (adc5140->adc_num == 0)
		snd_soc_component_write(component, ADC5140_MST_CFG1,
			fs_rate | fs_bclk_ratio);


	dev_info(adc5140->dev, "%s-\n", __func__);

	return 0;
}

static int adc5140_set_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_component *component = codec_dai->component;
	u8 iface_reg1 = 0;
	u8 iface_reg2 = 0;

	dev_info(component->dev, "%s: fmt 0x%X  codec %d\n",
			__func__, fmt, adc5140->adc_num);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface_reg2 = ADC5140_BCLK_FSYNC_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
		break;
	default:
		dev_err(component->dev, "%s: Invalid DAI master/slave interface\n",
			__func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface_reg1 |= ADC5140_BCLKINV_BIT;
		break;
	default:
		dev_err(component->dev, "%s: Invalid DAI clock signal polarity\n",
			__func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_B:
		/* Treat I2S same as TDM */
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg1 |= ADC5140_LEFT_JUST_BIT;
		break;
	default:
		dev_info(component->dev, "%s: No Format. Using default TDM\n",
			__func__);
	}

	snd_soc_component_update_bits(component, ADC5140_ASI_CFG0,
		  (ADC5140_BCLKINV_BIT|ADC5140_ASI_FMT_MSK), iface_reg1);
	snd_soc_component_update_bits(component, ADC5140_MST_CFG0,
			ADC5140_BCLK_FSYNC_MASTER, iface_reg2);

	if (adc5140->adc_num == 0)
		snd_soc_component_update_bits(component, ADC5140_ASI_CFG1,
			ADC5140_TX_CFG1_MSK,
			(ADC5140_LSB_HI_Z | ADC5140_BUS_KEEPER));
	else
		snd_soc_component_update_bits(component, ADC5140_ASI_CFG1,
			ADC5140_TX_CFG1_MSK, ADC5140_LSB_HI_Z);

	return 0;
}

static int adc5140_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 mclk_freq_conf = 0;

	dev_info(component->dev, "%s: clk_id %d, freq %d, dir %d\n",
		__func__, clk_id, freq, dir);

	switch (freq) {
	case ADC5140_FREQ_24576000:
		mclk_freq_conf = 7;
		break;
	case ADC5140_FREQ_24000000:
		mclk_freq_conf = 6;
		break;
	case ADC5140_FREQ_19268000:
		mclk_freq_conf = 5;
		break;
	case ADC5140_FREQ_19200000:
		mclk_freq_conf = 4;
		break;
	case ADC5140_FREQ_16000000:
		mclk_freq_conf = 3;
		break;
	case ADC5140_FREQ_13000000:
		mclk_freq_conf = 2;
		break;
	case ADC5140_FREQ_12288000:
		mclk_freq_conf = 1;
		break;
	case ADC5140_FREQ_12000000:
		mclk_freq_conf = 0;
		break;
	default:
		dev_info(component->dev, "%s: Invalid freq %d\n",
			__func__, freq);
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ADC5140_MST_CFG0,
		  ADC5140_MCLK_MSK, mclk_freq_conf);

	snd_soc_component_write(component, ADC5140_GPIO_CFG0,
		  ADC5140_MCLK_INPUT);

	return 0;
}

static int adc5140_dai_prepare(struct snd_pcm_substream *substream,
			struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s: codec %d\n", __func__,
		adc5140->adc_num);

	/* Enable Channels Inputs*/
	snd_soc_write(codec, ADC5140_IN_CH_EN,
				ADC5140_CH_INPUT_EN);
	/* Enable Channels Slot Output */
	snd_soc_write(codec, ADC5140_ASI_OUT_CH_EN,
				ADC5140_CH_OUTPUT_EN);
	/* Power On */
	snd_soc_update_bits(codec, ADC5140_PWR_CFG,
		(ADC5140_PWR_MASK_ADC|ADC5140_PWR_MASK_PLL), ADC5140_PWR_UP);

	return 0;
}

static int adc5140_hw_free(struct snd_pcm_substream *substream,
			struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	int value;

	dev_info(codec->dev, "%s:+ codec %d\n", __func__,
		adc5140->adc_num);

	value = snd_soc_read(codec, ADC5140_DEV_STS0);
	if (value != -1 && value & ADC5140_PWR_ON)
		adc5140_power_down(codec_dai);

	dev_info(codec->dev, "%s:- codec %d value %x\n", __func__,
		adc5140->adc_num, value);

	return 0;
}

void adc5140_power_down(struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s: codec %d\n", __func__,
		adc5140->adc_num);
#if SHUTDOWN_ENABLED
	snd_soc_write(codec, ADC5140_IN_CH_EN, 0);
	/* 20ms recommended by TI */
	mdelay(20);
	snd_soc_write(codec, ADC5140_ASI_OUT_CH_EN, 0);
	snd_soc_update_bits(codec, ADC5140_PWR_CFG,
		(ADC5140_PWR_MASK_ADC|ADC5140_PWR_MASK_PLL), ADC5140_PWR_DOWN);

	/* Sleep and wake up is needed to fix weakness in shutdown of ADC5140 */
	snd_soc_write(codec, ADC5140_SLEEP_CFG, ADC5140_SLEEP_MODE);
	/* 1ms recommended by TI */
	mdelay(1);
	snd_soc_write(codec, ADC5140_SLEEP_CFG, ADC5140_WAKE_UP);
	/* 10ms recommended by TI */
	mdelay(10);
#endif
}

static void adc5140_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s: codec %d\n", __func__,
		adc5140->adc_num);

	snd_soc_update_bits(codec, ADC5140_SHDN_CFG,
		ADC5140_DREG_PWR_MSK, ADC5140_DREG_OFF_IMMEDIATE);
}

static const struct snd_soc_dai_ops adc5140_dai_ops = {
	.hw_params	= adc5140_hw_params,
	.set_sysclk	= adc5140_set_dai_sysclk,
	.set_fmt	= adc5140_set_dai_fmt,
	.prepare    = adc5140_dai_prepare,
	.hw_free	= adc5140_hw_free,
	.shutdown   = adc5140_shutdown,
};

static int adc5140_codec_probe(struct snd_soc_codec *codec)
{
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	int ret;

	dev_info(adc5140->dev, "%s:+ codec %d\n", __func__, adc5140->adc_num);

	if (adc5140->adc_num == 0)
		ret = snd_soc_add_codec_controls(codec, adc5140_snd_controls_a,
				ARRAY_SIZE(adc5140_snd_controls_a));
	else
		ret = snd_soc_add_codec_controls(codec, adc5140_snd_controls_b,
				ARRAY_SIZE(adc5140_snd_controls_b));

	if (ret < 0) {
		dev_err(adc5140->dev, "%s: could not add kcontrol for codec %d (err=%d)\n",
			 __func__, adc5140->adc_num, ret);
		return -ret;
	}

	/* Only log error. DAPM is not necessary to operate this driver */
	if (adc5140->adc_num == 0)
		ret = snd_soc_dapm_new_controls(dapm, adc5140_dapm_widgets_a,
				ARRAY_SIZE(adc5140_dapm_widgets_a));
	else
		ret = snd_soc_dapm_new_controls(dapm, adc5140_dapm_widgets_b,
			ARRAY_SIZE(adc5140_dapm_widgets_b));
	if (ret < 0)
		dev_err(adc5140->dev,
			"%s: could not add dapm for codec %d (err=%d)\n",
			 __func__, adc5140->adc_num, ret);

	if (adc5140->adc_num == 0)
		ret = snd_soc_dapm_add_routes(dapm, adc5140_audio_map_a,
					ARRAY_SIZE(adc5140_audio_map_a));
	else
		ret = snd_soc_dapm_add_routes(dapm, adc5140_audio_map_b,
					ARRAY_SIZE(adc5140_audio_map_b));
	if (ret < 0)
		dev_err(adc5140->dev,
			"%s: could not add routes for codec %d (err=%d)\n",
			 __func__, adc5140->adc_num, ret);

	dev_info(adc5140->dev, "%s:-\n", __func__);

	return 0;
}

static int adc5140_codec_remove(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "%s:\n", __func__);
	return 0;
}

static int adc5140_codec_suspend(struct snd_soc_codec *codec)
{
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_info(codec->dev, "%s: codec %d disable suspend %d\n", __func__,
			adc5140->adc_num, adc5140->disable_suspend);

	if (adc5140->disable_suspend)
		return 0;

	regcache_cache_only(adc5140->regmap, true);
	regcache_mark_dirty(adc5140->regmap);

	if (!IS_ERR(adc5140->reset_adc_regulator)) {
		ret = regulator_disable(adc5140->reset_adc_regulator);
		if (ret)
			dev_err(codec->dev, "%s failed to disable audio regulator(%d)",
				__func__, ret);
	}
	return ret;
};

static int adc5140_codec_resume(struct snd_soc_codec *codec)
{
	struct adc5140_priv *adc5140 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_info(codec->dev, "%s: codec %d disable suspend %d\n", __func__,
			adc5140->adc_num, adc5140->disable_suspend);

	if (adc5140->disable_suspend)
		return 0;

	if (!IS_ERR(adc5140->reset_adc_regulator)) {
		ret = regulator_enable(adc5140->reset_adc_regulator);
		if (ret) {
			dev_err(codec->dev, "%s failed to enable audio regulator(%d)\n",
				__func__, ret);
			return ret;
		}
	}

	regcache_cache_only(adc5140->regmap, false);

	ret = regcache_sync(adc5140->regmap);
	if (ret)
		dev_err(codec->dev, "%s failed to sync registers(%d)\n",
			__func__, ret);

	return ret;
};

static struct snd_soc_codec_driver soc_codec_driver_adc5140 = {
	.probe			= adc5140_codec_probe,
	.remove			= adc5140_codec_remove,
	.suspend		= adc5140_codec_suspend,
	.resume			= adc5140_codec_resume,
};

static struct snd_soc_dai_driver adc5140_dai_driver_a[] = {
	{
		.name = "tlv320adc5140-codec-a",
		.capture = {
			.stream_name	 = "Capture",
			.channels_min	 = 2,
			.channels_max	 = 9,
			.rates		 = ADC5140_RATES,
			.formats	 = ADC5140_FORMATS,
		},
		.ops = &adc5140_dai_ops,
		.symmetric_rates = 1,
	}
};

static struct snd_soc_dai_driver adc5140_dai_driver_b[] = {
	{
		.name = "tlv320adc5140-codec-b",
		.capture = {
			.stream_name	 = "Capture",
			.channels_min	 = 2,
			.channels_max	 = 9,
			.rates		 = ADC5140_RATES,
			.formats	 = ADC5140_FORMATS,
		},
		.ops = &adc5140_dai_ops,
		.symmetric_rates = 1,
	}
};

static const struct of_device_id tlv320adc5140_of_match[] = {
	{ .compatible = "ti,tlv320adc5140" },
	{},
};
MODULE_DEVICE_TABLE(of, tlv320adc5140_of_match);

static int adc5140_parse_node(struct adc5140_priv *adc5140)
{
	struct i2c_client *client = to_i2c_client(adc5140->dev);
	int ret;

	adc5140->reset_adc_regulator = devm_regulator_get(adc5140->dev,
					"reset-adc-regulator");

	if (IS_ERR(adc5140->reset_adc_regulator)) {
		dev_warn(adc5140->dev, "failed to get adc regulator\n");
	} else {
		/*
		 * TI adc5140 hw errata: when release reset pin from
		 * low to high, if we have active i2c transactions,chip will
		 * hang. So we block all i2c transactions during reset
		 */
		i2c_lock_adapter(client->adapter);
		/*
		 * Need to set dt regulator "startup-delay-us" property for
		 * adding correct reset delay value
		 */
		ret = regulator_enable(adc5140->reset_adc_regulator);
		if (ret) {
			dev_err(adc5140->dev,
				"failed to regulator_enable(%d)\n", ret);
			i2c_unlock_adapter(client->adapter);
			return ret;
		}
		/*
		 * After releasing reset pin, add extra 1ms delay as margin
		 * before unblocking i2c transactions
		 */
		usleep_range(1000, 1100);
		i2c_unlock_adapter(client->adapter);
		/*
		 * After releasing reset pin, adc5140 required 10ms delay (value
		 * suggested by audio team) before any i2c operations
		 */
		usleep_range(10000, 11000);
	}

	ret =  device_property_read_u32(adc5140->dev, "adc-num",
						 &adc5140->adc_num);
	if (ret) {
		dev_err(adc5140->dev, "%s: adc_num DT property missing\n",
			__func__);
		return ret;
	}

	if (device_property_read_bool(adc5140->dev, "disable-suspend"))
		adc5140->disable_suspend = true;
	dev_info(adc5140->dev, "%s: suspend disabled %d\n", __func__,
		adc5140->disable_suspend);

	dev_info(adc5140->dev, "%s:- codec %d\n", __func__, adc5140->adc_num);

	return ret;
}

static int adc5140_i2c_probe(struct i2c_client *i2c,
				 const struct i2c_device_id *id)
{
	int ret;
	struct adc5140_priv *adc5140_data;

	dev_info(&i2c->dev, "%s: %s+ codec_type = %d\n", __func__,
		id->name, (int)id->driver_data);

	adc5140_data = devm_kzalloc(&i2c->dev, sizeof(struct adc5140_priv),
						GFP_KERNEL);
	if (!adc5140_data)
		return -ENOMEM;

	adc5140_data->regmap = devm_regmap_init_i2c(i2c, &adc5140_i2c_regmap);
	if (IS_ERR(adc5140_data->regmap)) {
		ret = PTR_ERR(adc5140_data->regmap);
		dev_err(&i2c->dev, "Failed to allocate 1st register map: %d\n",
			ret);
		return ret;
	}
	adc5140_data->dev = &i2c->dev;

	i2c_set_clientdata(i2c, adc5140_data);

	adc5140_parse_node(adc5140_data);

	if (adc5140_data->adc_num == 0)
		ret = snd_soc_register_codec(&i2c->dev,
					&soc_codec_driver_adc5140,
					adc5140_dai_driver_a, 1);
	else
		ret = snd_soc_register_codec(&i2c->dev,
					&soc_codec_driver_adc5140,
					adc5140_dai_driver_b, 1);

	if (ret)
		dev_err(&i2c->dev, "Failed to register codec %d. Error %d\n",
			adc5140_data->adc_num, ret);

	dev_info(&i2c->dev, "%s:- codec  = %d\n", __func__,
		adc5140_data->adc_num);

	return ret;
}

static void adc5140_i2c_shutdown(struct i2c_client *i2c)
{
	struct adc5140_priv *adc5140_data;
	int ret;

	dev_info(&i2c->dev, "%s\n", __func__);

	adc5140_data = dev_get_drvdata(&i2c->dev);

	if (IS_ERR(adc5140_data->reset_adc_regulator)) {
		dev_err(&i2c->dev, "Failed to get adc regulator. codec %d\n",
			adc5140_data->adc_num);
		return;
	}

	ret = regulator_disable(adc5140_data->reset_adc_regulator);
	if (ret)
		dev_err(&i2c->dev, "%s failed to disable adc regulator(%d). codec %d\n",
			__func__, ret, adc5140_data->adc_num);
}

static int adc5140_i2c_remove(struct i2c_client *i2c)
{
	pr_info("%s+\n", __func__);
	snd_soc_unregister_codec(&i2c->dev);
	pr_info("%s-\n", __func__);

	return 0;
}

static const struct i2c_device_id adc5140_i2c_id[] = {
	{ "tlv320adc5140", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adc5140_i2c_id);

static struct i2c_driver adc5140_i2c_driver = {
	.driver = {
		.name	= "tlv320adc5140",
		.of_match_table = tlv320adc5140_of_match,
	},
	.probe		= adc5140_i2c_probe,
	.shutdown   = adc5140_i2c_shutdown,
	.remove   = adc5140_i2c_remove,
	.id_table	= adc5140_i2c_id,
};
module_i2c_driver(adc5140_i2c_driver);

MODULE_DESCRIPTION("ASoC TLV320ADC5140 Codec Driver");
MODULE_AUTHOR("C. Omer Rafique <rafiquec@amazon.com>");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_LICENSE("GPL v2");
