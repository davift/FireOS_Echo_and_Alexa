// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021 MediaTek Inc.
 * Author: Zhiyong Tao <zhiyong.tao@mediatek.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/iio/iio.h>

/* Register definitions */
#define MT6577_AUXADC_CON0                    0x00
#define MT6577_AUXADC_CON1                    0x04
#define MT6577_AUXADC_CON2                    0x10
#define MT6577_AUXADC_STA                     BIT(0)

#define MT6577_AUXADC_DAT0                    0x14
#define MT6577_AUXADC_RDY0                    BIT(12)

#define MT6577_AUXADC_MISC                    0x94
#define MT6577_AUXADC_PDN_EN                  BIT(14)

#define MT6577_AUXADC_DAT_MASK                0xfff
#define MT6577_AUXADC_SLEEP_US                1000
#define MT6577_AUXADC_TIMEOUT_US              10000
#define MT6577_AUXADC_POWER_READY_MS          1
#define MT6577_AUXADC_SAMPLE_READY_US         25

#define VOLTAGE_FULL_RANGE                    1500
#define AUXADC_PRECISE                        4096

struct mtk_auxadc_compatible {
	bool sample_data_cali;
	bool check_global_idle;
};

#define SAMPLE_AMOUNT			      16

struct mt6577_auxadc_device {
	void __iomem *reg_base;
	struct clk *adc_clk;
	struct mutex lock;
	const struct mtk_auxadc_compatible *dev_comp;
	bool dts_error_correction;
	bool sysfs_error_correction;
	int exp_div_voltage_raw;
	int ref_voltage_channel;
};

static const struct mtk_auxadc_compatible mt8173_compat = {
	.sample_data_cali = false,
	.check_global_idle = false,
};

static const struct mtk_auxadc_compatible mt6765_compat = {
	.sample_data_cali = true,
	.check_global_idle = false,
};

#define MT6577_AUXADC_CHANNEL(idx) {				    \
		.type = IIO_VOLTAGE,				    \
		.indexed = 1,					    \
		.channel = (idx),				    \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	    \
				      BIT(IIO_CHAN_INFO_PROCESSED) | \
					  BIT(IIO_CHAN_INFO_SCALE),	\
}

static const struct iio_chan_spec mt6577_auxadc_iio_channels[] = {
	MT6577_AUXADC_CHANNEL(0),
	MT6577_AUXADC_CHANNEL(1),
	MT6577_AUXADC_CHANNEL(2),
	MT6577_AUXADC_CHANNEL(3),
	MT6577_AUXADC_CHANNEL(4),
	MT6577_AUXADC_CHANNEL(5),
	MT6577_AUXADC_CHANNEL(6),
	MT6577_AUXADC_CHANNEL(7),
	MT6577_AUXADC_CHANNEL(8),
	MT6577_AUXADC_CHANNEL(9),
	MT6577_AUXADC_CHANNEL(10),
	MT6577_AUXADC_CHANNEL(11),
	MT6577_AUXADC_CHANNEL(12),
	MT6577_AUXADC_CHANNEL(13),
	MT6577_AUXADC_CHANNEL(14),
	MT6577_AUXADC_CHANNEL(15),
};

/* For Voltage calculation */
#define VOLTAGE_FULL_RANGE  1500	/* VA voltage */
#define AUXADC_PRECISE      4096	/* 12 bits */

static int mt_auxadc_get_cali_data(int rawdata, bool enable_cali)
{
	return rawdata;
}

static inline void mt6577_auxadc_mod_reg(void __iomem *reg,
					 u32 or_mask, u32 and_mask)
{
	u32 val;

	val = readl(reg);
	val |= or_mask;
	val &= ~and_mask;
	writel(val, reg);
}

static ssize_t _show_error_correction_status(struct device *dev, struct device_attribute *devattr,
						char *buf)
{
	int n = 0;
	struct mt6577_auxadc_device *pdata = dev_get_platdata(dev);

	if (pdata == NULL) {
		dev_err(dev, "Couldn't find platform data for auxadc device\n");
		return -EINVAL;
	}

	if (pdata->sysfs_error_correction)
		n += sprintf(buf, "enabled\n");
	else
		n += sprintf(buf, "disabled\n");

	return n;
}

static ssize_t _store_error_correction_status(struct device *dev, struct device_attribute *devattr,
						const char *buf, size_t count)
{
	char status[20];
	struct mt6577_auxadc_device *pdata = dev_get_platdata(dev);

	if (pdata == NULL) {
		dev_err(dev, "Couldn't find platform data for auxadc device\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%s", status) == 1) {
		if (!strcasecmp(status, "enabled")) {
			pdata->sysfs_error_correction = true;
		} else if (!strcasecmp(status, "disabled")) {
			pdata->sysfs_error_correction = false;
		}
		return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(error_correction, 0644, _show_error_correction_status,
		_store_error_correction_status);

static int mt6577_auxadc_read(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan)
{
	u32 val;
	void __iomem *reg_channel;
	int ret;
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	reg_channel = adc_dev->reg_base + MT6577_AUXADC_DAT0 +
		      chan->channel * 0x04;

	mutex_lock(&adc_dev->lock);

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_CON1,
			      0, 1 << chan->channel);

	/* read channel and make sure old ready bit == 0 */
	ret = readl_poll_timeout(reg_channel, val,
				 ((val & MT6577_AUXADC_RDY0) == 0),
				 MT6577_AUXADC_SLEEP_US,
				 MT6577_AUXADC_TIMEOUT_US);
	if (ret < 0) {
		dev_err(indio_dev->dev.parent,
			"wait for channel[%d] ready bit clear time out\n",
			chan->channel);
		goto err_timeout;
	}

	/* set bit to trigger sample */
	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_CON1,
			      1 << chan->channel, 0);

	/* we must delay here for hardware sample channel data */
	udelay(MT6577_AUXADC_SAMPLE_READY_US);

	if (adc_dev->dev_comp->check_global_idle) {
		/* check MTK_AUXADC_CON2 if auxadc is idle */
		ret = readl_poll_timeout(adc_dev->reg_base + MT6577_AUXADC_CON2,
					 val, ((val & MT6577_AUXADC_STA) == 0),
					 MT6577_AUXADC_SLEEP_US,
					 MT6577_AUXADC_TIMEOUT_US);
		if (ret < 0) {
			dev_err(indio_dev->dev.parent,
				"wait for auxadc idle time out\n");
			goto err_timeout;
		}
	}

	/* read channel and make sure ready bit == 1 */
	ret = readl_poll_timeout(reg_channel, val,
				 ((val & MT6577_AUXADC_RDY0) != 0),
				 MT6577_AUXADC_SLEEP_US,
				 MT6577_AUXADC_TIMEOUT_US);
	if (ret < 0) {
		dev_err(indio_dev->dev.parent,
			"wait for channel[%d] data ready time out\n",
			chan->channel);
		goto err_timeout;
	}

	/* read data */
	val = readl(reg_channel) & MT6577_AUXADC_DAT_MASK;

	mutex_unlock(&adc_dev->lock);

	return val;

err_timeout:

	mutex_unlock(&adc_dev->lock);

	return -ETIMEDOUT;
}

static int mt6577_auxadc_error_correction_read(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan)
{
	int actual_div_voltage_raw, actual_val, read_val, average_val, i;
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	average_val = 0;

	if (!adc_dev->sysfs_error_correction || !adc_dev->dts_error_correction
		|| chan->channel == adc_dev->ref_voltage_channel) {
		read_val = mt6577_auxadc_read(indio_dev, chan);
		return read_val;
	}

	for (i = 0; i < SAMPLE_AMOUNT; i++) {
		read_val = mt6577_auxadc_read(indio_dev,
				&mt6577_auxadc_iio_channels[adc_dev->ref_voltage_channel]);

		if (read_val < 0)
			return read_val;

		average_val += read_val;
	}

	actual_div_voltage_raw = div_s64((s64)average_val, SAMPLE_AMOUNT);

	if (actual_div_voltage_raw <= 0) {
		dev_err(indio_dev->dev.parent,
				"error factor found was %d when reading from ADC channel [%d]\n",
				actual_div_voltage_raw,
				chan->channel);
		return actual_div_voltage_raw;
	}

	read_val = mt6577_auxadc_read(indio_dev, chan);

	if (read_val < 0)
		return read_val;

	actual_val = div_s64((s64)(read_val) * adc_dev->exp_div_voltage_raw, actual_div_voltage_raw);

	dev_dbg(indio_dev->dev.parent, "Read voltage on ref channel: %d, Actual voltage ref channel: %d, Read voltage on other ADC channel: %d, Actual voltage on other ADC Channel: %d\n",
		actual_div_voltage_raw, adc_dev->exp_div_voltage_raw, read_val, actual_val);

	return actual_val;
}

static int mt6577_auxadc_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val,
				  int *val2,
				  long info)
{
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	if (val == NULL)
		return -EINVAL;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		*val = mt6577_auxadc_read(indio_dev, chan);
		if (*val < 0) {
			dev_notice(indio_dev->dev.parent,
				"failed to sample data on channel[%d]\n",
				chan->channel);
			return *val;
		}
		if (adc_dev->dev_comp->sample_data_cali)
			*val = mt_auxadc_get_cali_data(*val, true);

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		*val = mt6577_auxadc_error_correction_read(indio_dev, chan);

		if (*val < 0) {
			dev_err(indio_dev->dev.parent,
				"failed to sample data on channel[%d]\n",
				chan->channel);
			return *val;
		}
		if (adc_dev->dev_comp->sample_data_cali)
			*val = mt_auxadc_get_cali_data(*val, true);

		/* Convert adc raw data to voltage: 0 - 1500 mV */
		*val = *val * VOLTAGE_FULL_RANGE / AUXADC_PRECISE;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = VOLTAGE_FULL_RANGE;
		*val2 = AUXADC_PRECISE;
		return IIO_VAL_FRACTIONAL;

	default:
		return -EINVAL;
	}
}

static const struct iio_info mt6577_auxadc_info = {
	.read_raw = &mt6577_auxadc_read_raw,
};

static int __maybe_unused mt6577_auxadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);
	int ret;

	ret = clk_prepare_enable(adc_dev->adc_clk);
	if (ret) {
		pr_err("failed to enable auxadc clock\n");
		return ret;
	}

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      MT6577_AUXADC_PDN_EN, 0);
	mdelay(MT6577_AUXADC_POWER_READY_MS);

	return 0;
}

static int __maybe_unused mt6577_auxadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      0, MT6577_AUXADC_PDN_EN);
	clk_disable_unprepare(adc_dev->adc_clk);

	return 0;
}

static int mt6577_auxadc_probe(struct platform_device *pdev)
{
	struct mt6577_auxadc_device *adc_dev;
	struct device_node *adc_dev_node;
	unsigned long adc_clk_rate;
	struct iio_dev *indio_dev;
	int ret;
	u32 ref_voltage, ref_voltage_channel;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc_dev));
	if (!indio_dev)
		return -ENOMEM;

	adc_dev = iio_priv(indio_dev);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &mt6577_auxadc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mt6577_auxadc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(mt6577_auxadc_iio_channels);

	adc_dev_node = pdev->dev.of_node;

	adc_dev->sysfs_error_correction = true;

	adc_dev->dts_error_correction = of_property_read_bool(adc_dev_node, "has-error-correction");

	if (!of_property_read_u32(adc_dev_node, "expected-divider-voltage-raw", &ref_voltage)) {
		adc_dev->exp_div_voltage_raw = div_s64((s64)(ref_voltage * AUXADC_PRECISE), VOLTAGE_FULL_RANGE);
	} else {
		adc_dev->dts_error_correction = false;
		dev_err(&pdev->dev, "failed to obtain expected divider voltage from device tree\n");
	}

	if (!of_property_read_u32(adc_dev_node, "ref-voltage-adc-channel", &ref_voltage_channel)) {
		adc_dev->ref_voltage_channel = (int)ref_voltage_channel;
	} else {
		adc_dev->dts_error_correction = false;
		dev_err(&pdev->dev, "failed to obtain reference voltage adc channel from device tree\n");
	}

	adc_dev->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc_dev->reg_base)) {
		dev_err(&pdev->dev, "failed to get auxadc base address\n");
		return PTR_ERR(adc_dev->reg_base);
	}

	adc_dev->adc_clk = devm_clk_get(&pdev->dev, "main");
	if (IS_ERR(adc_dev->adc_clk)) {
		dev_err(&pdev->dev, "failed to get auxadc clock\n");
		return PTR_ERR(adc_dev->adc_clk);
	}

	ret = clk_prepare_enable(adc_dev->adc_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable auxadc clock\n");
		return ret;
	}

	adc_clk_rate = clk_get_rate(adc_dev->adc_clk);
	if (!adc_clk_rate) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "null clock rate\n");
		goto err_disable_clk;
	}

	adc_dev->dev_comp = device_get_match_data(&pdev->dev);

	mutex_init(&adc_dev->lock);

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      MT6577_AUXADC_PDN_EN, 0);
	mdelay(MT6577_AUXADC_POWER_READY_MS);

	platform_set_drvdata(pdev, indio_dev);

	pdev->dev.platform_data = adc_dev;

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register iio device\n");
		goto err_power_off;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_error_correction);
	if (ret)
		dev_err(&pdev->dev, "%s Failed to create error_correction attr - error (%d)\n",
			__func__, ret);

	return ret;

err_power_off:
	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      0, MT6577_AUXADC_PDN_EN);
err_disable_clk:
	clk_disable_unprepare(adc_dev->adc_clk);
	return ret;
}

static int mt6577_auxadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      0, MT6577_AUXADC_PDN_EN);

	clk_disable_unprepare(adc_dev->adc_clk);

	return 0;
}

static const struct dev_pm_ops mt6577_auxadc_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(mt6577_auxadc_suspend,
					mt6577_auxadc_resume)
};

static const struct of_device_id mt6577_auxadc_of_match[] = {
	{ .compatible = "mediatek,mt2701-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt2712-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt7622-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt8173-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt6765-auxadc", .data = &mt6765_compat},
	{ }
};
MODULE_DEVICE_TABLE(of, mt6577_auxadc_of_match);

static struct platform_driver mt6577_auxadc_driver = {
	.driver = {
		.name   = "mt6577-auxadc",
		.of_match_table = mt6577_auxadc_of_match,
		.pm = &mt6577_auxadc_pm_ops,
	},
	.probe	= mt6577_auxadc_probe,
	.remove	= mt6577_auxadc_remove,
};
module_platform_driver(mt6577_auxadc_driver);

MODULE_AUTHOR("Zhiyong Tao <zhiyong.tao@mediatek.com>");
MODULE_DESCRIPTION("MTK AUXADC Device Driver");
MODULE_LICENSE("GPL v2");
