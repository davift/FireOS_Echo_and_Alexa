/*
 * Device driver for monitoring ambient light intensity in (lux)
 * for the TAOS TSL2540 device.
 *
 * Copyright (c) 2012, TAOS Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA        02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "tsl2540.h"

/* TSL2540 Device ID */
#define TSL2540_ID_CHIP 0xE4
#define TSL2540_ID_AUX  0x01
#define TSL2540_ID_REV  0x61

/* TSL2540 Config Register Constants */
#define TSL2540_MIN_CONFIG_REG_ADDR    0x80
#define TSL2540_MAX_CONFIG_REG_ADDR    0x9F

#define TSL2540_CONFIG_COUNT           32

/* TSL2540 Device Registers */
#define	TSL2540_REG_ENABLE             0x80
#define	TSL2540_REG_ATIME              0x81
#define	TSL2540_REG_WTIME              0x83
#define	TSL2540_REG_AILT               0x84
#define	TSL2540_REG_AILT_HI            0x85
#define	TSL2540_REG_AIHT               0x86
#define	TSL2540_REG_AIHT_HI            0x87
#define	TSL2540_REG_PERS               0x8C
#define	TSL2540_REG_CFG0               0x8D
#define	TSL2540_REG_CFG1               0x90
#define	TSL2540_REG_REVID              0x91
#define	TSL2540_REG_ID                 0x92
#define	TSL2540_REG_STATUS             0x93
#define	TSL2540_REG_CH0DATA            0x94
#define	TSL2540_REG_CH0DATA_HI         0x95
#define	TSL2540_REG_CH1DATA            0x96
#define	TSL2540_REG_CH1DATA_HI         0x97
#define	TSL2540_REG_REVID_2            0x9E
#define	TSL2540_REG_CFG2               0x9F
#define	TSL2540_REG_CFG3               0xAB
#define	TSL2540_REG_AZ_CONFIG          0xD6
#define	TSL2540_REG_INTENAB            0xDD

/* Register Masks */
#define TSL2540_ENABLE_PON 0x01
#define TSL2540_ENABLE_AEN 0x02
#define TSL2540_ENABLE_WEN 0x08

#define TSL2540_STATUS_CAL_INT 0x08
#define TSL2540_STATUS_ALS_INT 0x10
#define TSL2540_STATUS_SAT_INT 0x80

#define TSL2540_CFG0_WAIT_LONG 0x04
#define TSL2540_CFG0_DEFAULT 0x80

/* ALS constants Definitions */
#define TSL2540_INTEGRATION_CYCLE 2816

#define TSL2540_MIN_ITIME 3
#define TSL2540_MAX_ITIME 721

#define TSL2540_LONG_WAIT_MULTIPLIER 12

#define TSL2540_MAX_AZ_ITERATIONS               127
#define TSL2540_DEFAULT_AZ_ITERATIONS           64

/* Cap als_saturation to 90% of Max value for CH0/CH1 i.e. 65535 */
#define MAX_ALS_SATURATION	58801

enum {
	TSL2540_CHIP_UNKNOWN = 0,
	TSL2540_CHIP_WORKING = 1,
	TSL2540_CHIP_SUSPENDED = 2
};

struct tsl2540_parse_result {
	int integer;
	int fract;
};

/* Per-device data */
struct tsl2540_als_info {
	u16 als_ch0;
	u16 als_ch1;
};

struct tsl2540_chip_info {
	int chan_table_elements;
	struct iio_chan_spec		channel[4];
	const struct iio_info		*info;
};

struct tsl2540_chip {
	kernel_ulong_t id;
	struct mutex als_mutex;
	struct i2c_client *client;
	struct tsl2540_als_info als_cur_info;
	struct tsl2540_settings tsl2540_settings;
	struct tsl2540_platform_data *pdata;
	int als_saturation;
	int tsl2540_chip_status;
	u8 tsl2540_config[TSL2540_CONFIG_COUNT];
	const struct tsl2540_chip_info	*chip_info;
	const struct iio_info *info;
	s64 event_timestamp;
	u32 autozero_iterations;
	bool autozero;
	unsigned long last_again_check_time;
	unsigned long last_raw_access_time;
	bool wakeup;
};

static const struct tsl2540_settings tsl2540_default_settings = {
	.als_time = 721, /* 721 ms */
	.als_gain = 5,
	.wait_time = 106, /* 300 ms */
	.wait_en = false,
	.wait_long = false,
	.als_thresh_low = 0,
	.als_thresh_high = 65535,
	.persistence = 0, /* generate interrupt on every ALS cycle */
	.interrupts_en = 0x0, /* disabled by default */
	.als_auto_gain = false,
};

/* als gain struct */
struct tsl2540_als_gain_t {
	s16 modified_value; // actual gain value * 2
	u8 cfg1;            // value for cfg1 reg (bits 1:0)
	u8 cfg2;            // value for cfg2 reg (AGAINMAX: bit 4 and AGAINL: bit 2)
};

/* als gain array */
static const struct tsl2540_als_gain_t tsl2540_als_gains[] = {
	{ 1,   0x0, 0x0  }, //  .5x
	{ 2,   0x0, 0x4  }, //  1x
	{ 8,   0x1, 0x4  }, //  4x
	{ 32,  0x2, 0x4  }, //  16x
	{ 128, 0x3, 0x4  }, //  64x
	{ 256, 0x3, 0x14 }, //  128x
};

static int tsl2540_invoke_change(struct iio_dev *indio_dev);

/**
 * tsl2540_i2c_read_byte() - Read a byte from a register.
 * @client:	i2c client
 * @reg:	device register to read from
 * @*val:	pointer to location to store register contents.
 *
 */
static int
tsl2540_i2c_read_byte(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf = reg;
	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = sizeof(buf);
	msg[0].buf = &buf;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = sizeof(*val);
	msg[1].buf = val;

	return i2c_transfer(client->adapter, msg, 2);
}

/**
 * tsl2540_get_channel_data() - Reads raw data from visual and ir data channels
 * @indio_dev:	pointer to IIO device
 *
 * The raw ch0 and ch1 values of the ambient light sensed in the last
 * integration cycle are read from the device.
 * If data is saturated then the channels will not contain new data until the next cycle.
 * If auto gain is enabled, the gain level will be adjusted to get a
 * more accurate reading if data is too close to 0 or is saturated.
 */
static void tsl2540_get_channel_data(struct iio_dev *indio_dev) {
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	u8 status;
	u8 data[4];
	u16 ch0;
	u16 ch1;
	int invoke_change = 0;
	int ret = 0;

	if (mutex_trylock(&chip->als_mutex) == 0)
		goto out_unlock; /* busy, so return LAST VALUE */

	ret = tsl2540_i2c_read_byte(chip->client, TSL2540_REG_STATUS, &status);
	if (ret < 0) {
		dev_err(&chip->client->dev,
				"%s: Failed to read STATUS Reg\n", __func__);
		goto out_unlock;
	}

	ret = i2c_smbus_read_i2c_block_data(chip->client, TSL2540_REG_CH0DATA, 4, data);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: failed to read. err=%x\n",
			__func__, ret);
		goto out_unlock;
	}

	/* clear any existing interrupt status */
	ret = i2c_smbus_write_byte_data(chip->client, TSL2540_REG_STATUS, status);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"i2c_write_command failed - err = %d\n", ret);
		goto out_unlock; /* have no data, so return failure */
	}

	/* extract ALS/lux data */
	ch0 = le16_to_cpup((const __le16 *)&data[0]);
	ch1 = le16_to_cpup((const __le16 *)&data[2]);

	/* update als_cur_info ch0 and ch1 */
	chip->als_cur_info.als_ch0 = ch0;
	chip->als_cur_info.als_ch1 = ch1;

	/* get the last time after raw data values are obtained */
	chip->last_raw_access_time = jiffies;

	/* handle auto gain */
	if (chip->tsl2540_settings.als_auto_gain) {
		/* Wait for atleast 2 integration cycles before reconsidering gain */
		if (!time_after_eq(jiffies, chip->last_again_check_time +
					(2 * msecs_to_jiffies(chip->tsl2540_settings.als_time))))
			goto out_unlock;

		if ((ch0 >= chip->als_saturation) || (ch1 >= chip->als_saturation)) {
			if (chip->tsl2540_settings.als_gain > 0) {
				chip->tsl2540_settings.als_gain -= 1;
				invoke_change = true;
			}
		} else if ((ch0 <= (chip->als_saturation * 5) / 900) ||
			(ch1 <= (chip->als_saturation * 5) / 900)) {
			if (chip->tsl2540_settings.als_gain <
				ARRAY_SIZE(tsl2540_als_gains) - 1) {
				chip->tsl2540_settings.als_gain += 1;
				invoke_change = true;
			}
		}
	}

out_unlock:
	mutex_unlock(&chip->als_mutex);

	if (invoke_change)
		tsl2540_invoke_change(indio_dev);
}

/**
 * tsl2540_defaults() - Populates the device nominal operating parameters
 *                      with those provided by a 'platform' data struct or
 *                      with prefined defaults.
 *
 * @chip:               pointer to device structure.
 */
static int tsl2540_defaults(struct tsl2540_chip *chip)
{
	struct device_node *np = chip->client->dev.of_node;

	/* If Operational settings defined elsewhere.. */
	if (chip->pdata && chip->pdata->platform_default_settings)
		memcpy(&chip->tsl2540_settings,
		       chip->pdata->platform_default_settings,
		       sizeof(tsl2540_default_settings));
	else
		memcpy(&chip->tsl2540_settings,
		       &tsl2540_default_settings,
		       sizeof(tsl2540_default_settings));

	if (of_property_read_bool(np, "auto-gain"))
		chip->tsl2540_settings.als_auto_gain = true;

	if (of_property_read_bool(np, "enable-interrupts"))
		chip->tsl2540_settings.interrupts_en = TSL2540_STATUS_ALS_INT;

	if (of_property_read_bool(np, "enable-wait"))
		chip->tsl2540_settings.wait_en = true;

	if (of_property_read_bool(np, "wait-long"))
		chip->tsl2540_settings.wait_long = true;

	return 0;
}

static int tsl2540_chip_on(struct iio_dev *indio_dev)
{
	int ret = 0;
	int als_cycles;
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	u8 reg_val = 0;

	if (chip->pdata && chip->pdata->power_on)
		chip->pdata->power_on(indio_dev);

	/* Non calculated parameters */
	chip->tsl2540_config[TSL2540_REG_WTIME - TSL2540_MIN_CONFIG_REG_ADDR] =
			chip->tsl2540_settings.wait_time;
	if (chip->tsl2540_settings.wait_long) {
		chip->tsl2540_config[TSL2540_REG_CFG0 - TSL2540_MIN_CONFIG_REG_ADDR] =
			TSL2540_CFG0_WAIT_LONG | TSL2540_CFG0_DEFAULT;
	} else {
		chip->tsl2540_config[TSL2540_REG_CFG0 - TSL2540_MIN_CONFIG_REG_ADDR] =
			TSL2540_CFG0_DEFAULT;
	}

	chip->tsl2540_config[TSL2540_REG_AILT - TSL2540_MIN_CONFIG_REG_ADDR] =
		(chip->tsl2540_settings.als_thresh_low) & 0xFF;
	chip->tsl2540_config[TSL2540_REG_AILT_HI - TSL2540_MIN_CONFIG_REG_ADDR] =
		(chip->tsl2540_settings.als_thresh_low >> 8) & 0xFF;
	chip->tsl2540_config[TSL2540_REG_AIHT - TSL2540_MIN_CONFIG_REG_ADDR] =
		(chip->tsl2540_settings.als_thresh_high) & 0xFF;
	chip->tsl2540_config[TSL2540_REG_AIHT_HI - TSL2540_MIN_CONFIG_REG_ADDR] =
		(chip->tsl2540_settings.als_thresh_high >> 8) & 0xFF;
	chip->tsl2540_config[TSL2540_REG_PERS - TSL2540_MIN_CONFIG_REG_ADDR] =
		chip->tsl2540_settings.persistence;

	/* and make sure we're not already on */
	if (chip->tsl2540_chip_status == TSL2540_CHIP_WORKING) {
		/* if forcing a register update - turn off, then on */
		dev_info(&chip->client->dev, "device is already enabled\n");
		return -EINVAL;
	}

	/* determine als integration register */
	/* als_time in ms, cycles are 2.81 ms */
	als_cycles =
		((chip->tsl2540_settings.als_time * 1000) / TSL2540_INTEGRATION_CYCLE);

	chip->tsl2540_config[TSL2540_REG_ATIME - TSL2540_MIN_CONFIG_REG_ADDR] =
		als_cycles - 1;

	/* Set the gain based on tsl2540_settings struct */
	chip->tsl2540_config[TSL2540_REG_CFG1 - TSL2540_MIN_CONFIG_REG_ADDR] =
		tsl2540_als_gains[chip->tsl2540_settings.als_gain].cfg1;
	chip->tsl2540_config[TSL2540_REG_CFG2 - TSL2540_MIN_CONFIG_REG_ADDR] =
		tsl2540_als_gains[chip->tsl2540_settings.als_gain].cfg2;

	/* set chip struct re scaling and saturation */
	chip->als_saturation = als_cycles * 922; /* 90% of full scale (1024) */
	chip->als_saturation = min(chip->als_saturation, MAX_ALS_SATURATION);

	/*
	 * TSL2540 Specific power-on / register write sequence
	 * Write the config registers first.
	 */
	ret = i2c_smbus_write_i2c_block_data(chip->client,
		TSL2540_MIN_CONFIG_REG_ADDR,
		TSL2540_CONFIG_COUNT,
		chip->tsl2540_config);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to update regs\n");
		return ret;
	}

	/*
	 * Setup interrupts
	 */
	if (chip->tsl2540_settings.interrupts_en != 0) {
		dev_info(&chip->client->dev, "Setting Up Interrupt(s)\n");

		ret = i2c_smbus_write_byte_data(chip->client,
			TSL2540_REG_INTENAB,
			chip->tsl2540_settings.interrupts_en);
		if (ret < 0) {
			dev_err(&chip->client->dev, "failed to write INTENAB reg\n");
			return ret;
		}

		/* Clear out any initial interrupts  */
		ret = tsl2540_i2c_read_byte(chip->client, TSL2540_REG_STATUS, &reg_val);
		if (ret < 0) {
			dev_err(&chip->client->dev, "failed to read STATUS reg\n");
			return ret;
		}

		ret = i2c_smbus_write_byte_data(chip->client, TSL2540_REG_STATUS, reg_val);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"%s: Failed to clear Int status\n",
				__func__);
		return ret;
		}
	}

	/*
	 * Setup autozero
	 */
	if (chip->autozero) {
		dev_info(&chip->client->dev, "Setting Up Auto Zero\n");

		ret = i2c_smbus_write_byte_data(chip->client, TSL2540_REG_AZ_CONFIG,
			chip->autozero_iterations);
		if (ret < 0) {
			dev_err(&chip->client->dev,
				"%s: Failed to write autozero configuration\n",
				__func__);
			return ret;
		}
	}

	/*
	 * All registers written. Power on the device.
	 */
	ret = i2c_smbus_write_byte_data(chip->client, TSL2540_REG_ENABLE, TSL2540_ENABLE_PON);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed on ENABLE reg.\n", __func__);
		return ret;
	}

	mdelay(3); /* Power-on settling time */

	/*
	 * NOW enable the ADC
	 * initialize the desired mode of operation
	 */
	if (chip->tsl2540_settings.wait_en) {
		ret = i2c_smbus_write_byte_data(chip->client,
			TSL2540_REG_ENABLE,
			TSL2540_ENABLE_PON | TSL2540_ENABLE_AEN | TSL2540_ENABLE_WEN);
	} else {
		ret = i2c_smbus_write_byte_data(chip->client,
			TSL2540_REG_ENABLE,
			TSL2540_ENABLE_PON | TSL2540_ENABLE_AEN);
	}
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to enable adc reg.\n", __func__);
		return ret;
	}

	chip->tsl2540_chip_status = TSL2540_CHIP_WORKING;

	return ret;
}

static int tsl2540_chip_off(struct iio_dev *indio_dev)
{
	int ret;
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	/* turn device off */
	chip->tsl2540_chip_status = TSL2540_CHIP_SUSPENDED;

	ret = i2c_smbus_write_byte_data(chip->client, TSL2540_REG_ENABLE, 0x00);

	if (chip->pdata && chip->pdata->power_off)
		chip->pdata->power_off(chip->client);

	return ret;
}

/**
 * tsl2540_invoke_change
 * @indio_dev:	pointer to IIO device
 *
 * Obtain and lock ALS resources,
 * determine and save device state (On/Off),
 * cycle device to implement updated parameter,
 * put device back into proper state, and unlock
 * resource.
 */
static
int tsl2540_invoke_change(struct iio_dev *indio_dev)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int device_status;

	mutex_lock(&chip->als_mutex);

	device_status = chip->tsl2540_chip_status;

	if (device_status == TSL2540_CHIP_WORKING)
		tsl2540_chip_off(indio_dev);

	tsl2540_chip_on(indio_dev);

	if (device_status != TSL2540_CHIP_WORKING)
		tsl2540_chip_off(indio_dev);

	if (chip->tsl2540_settings.als_auto_gain)
		chip->last_again_check_time = jiffies;

	mutex_unlock(&chip->als_mutex);

	return 0;
}

static ssize_t tsl2540_power_state_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->tsl2540_chip_status);
}

static ssize_t tsl2540_power_state_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	if (value)
		tsl2540_chip_on(indio_dev);
	else
		tsl2540_chip_off(indio_dev);

	mutex_unlock(&chip->als_mutex);

	return len;
}

static ssize_t tsl2540_gain_available_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", "1 2 8 32 128 256");
}

static ssize_t tsl2540_als_time_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));
	int y, z;

	y = chip->tsl2540_settings.als_time;
	z = chip->tsl2540_settings.als_time;

	y /= 1000;
	z %= 1000;

	return snprintf(buf, PAGE_SIZE, "%d.%03d\n", y, z);
}

static ssize_t tsl2540_als_time_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	struct tsl2540_parse_result result;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100, &result.integer, &result.fract);

	if (ret)
		return ret;
	if (result.integer > 0)
		return -EINVAL;
	if (result.fract < TSL2540_MIN_ITIME || result.fract > TSL2540_MAX_ITIME)
		return -EINVAL;

	dev_dbg(&chip->client->dev, "%s: result.int = %d, result.fract = %d",
		__func__, result.integer, result.fract);

	chip->tsl2540_settings.als_time = result.fract;

	dev_dbg(&chip->client->dev, "%s: als time = %d",
		 __func__, chip->tsl2540_settings.als_time);

	tsl2540_invoke_change(indio_dev);

	return len;
}

static IIO_CONST_ATTR(in_intensity0_integration_time_available,
		".003 - .721");

/* wait time enable settings */
static ssize_t tsl2540_als_wait_time_en_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->tsl2540_settings.wait_en);
}

static ssize_t tsl2540_als_wait_time_en_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	chip->tsl2540_settings.wait_en = value;

	tsl2540_invoke_change(indio_dev);
	return len;
}

/* effective wait time settings */
static ssize_t tsl2540_als_wait_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));
	int y, z, wait_time;

	wait_time = TSL2540_MIN_ITIME * chip->tsl2540_settings.wait_time;

	/* when WLONG is asserted, wait time is 12x longer */
	if (chip->tsl2540_settings.wait_long)
		wait_time *= TSL2540_LONG_WAIT_MULTIPLIER;

	y = wait_time / 1000;
	z = wait_time % 1000;

	return snprintf(buf, PAGE_SIZE, "%d.%03d\n", y, z);
}

static ssize_t tsl2540_als_wait_time_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	struct tsl2540_parse_result result;
	int wait_time;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100, &result.integer, &result.fract);
	if (ret)
		return ret;

	wait_time = DIV_ROUND_UP((result.integer * 1000) + result.fract,
		TSL2540_MIN_ITIME);

	/* when WLONG is asserted, wait time is 12x longer */
	if (chip->tsl2540_settings.wait_long)
		wait_time = DIV_ROUND_UP(wait_time,
			TSL2540_LONG_WAIT_MULTIPLIER);

	if (wait_time != (u8)wait_time) {
		dev_info(&chip->client->dev, "%s: wait time out of range: %d",
			 __func__, wait_time);
		return -EINVAL;
	}

	chip->tsl2540_settings.wait_time = wait_time;

	dev_info(&chip->client->dev, "%s: als wait time = %d",
		 __func__, wait_time);

	tsl2540_invoke_change(indio_dev);
	return len;
}

/* persistence settings */
static ssize_t tsl2540_als_persistence_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));
	int pers_value;

	pers_value = chip->tsl2540_settings.persistence & 0x0F;

	return snprintf(buf, PAGE_SIZE, "%d\n", pers_value);
}

static ssize_t tsl2540_als_persistence_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));
	unsigned int pers_val = 0;
	int ret = kstrtouint(buf, 10, &pers_val);

	if (ret) {
		dev_err(&chip->client->dev,
			"could not parse persistence value: %d\n",
			ret);
		return ret;
	}

	if (pers_val > 0xF) {
		dev_err(&chip->client->dev,
			"invalid persistence value: %d\n",
			pers_val);
		return -EINVAL;
	}

	chip->tsl2540_settings.persistence = pers_val;

	tsl2540_invoke_change(indio_dev);

	return len;
}

static ssize_t tsl2540_auto_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->tsl2540_settings.als_auto_gain);
}

static ssize_t tsl2540_auto_gain_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct tsl2540_chip *chip = iio_priv(dev_to_iio_dev(dev));
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	chip->tsl2540_settings.als_auto_gain = value;

	return len;
}

static int tsl2540_read_interrupt_config(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int ret;

	ret = !!(chip->tsl2540_settings.interrupts_en & TSL2540_STATUS_ALS_INT);

	return ret;
}

static int tsl2540_write_interrupt_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir,
					  int val)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	if (val) {
		chip->tsl2540_settings.interrupts_en = TSL2540_STATUS_ALS_INT;
	} else {
		chip->tsl2540_settings.interrupts_en = 0x00;
	}

	tsl2540_invoke_change(indio_dev);

	return 0;
}

static int tsl2540_write_thresh(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info,
				int val, int val2)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	switch (dir) {
	case IIO_EV_DIR_RISING:
		chip->tsl2540_settings.als_thresh_high = val;
		break;
	case IIO_EV_DIR_FALLING:
		chip->tsl2540_settings.als_thresh_low = val;
		break;
	default:
		return -EINVAL;
	}

	tsl2540_invoke_change(indio_dev);

	return 0;
}

static int tsl2540_read_thresh(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir,
				   enum iio_event_info info,
			       int *val, int *val2)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	switch (dir) {
	case IIO_EV_DIR_RISING:
		*val = chip->tsl2540_settings.als_thresh_high;
		break;
	case IIO_EV_DIR_FALLING:
		*val = chip->tsl2540_settings.als_thresh_low;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int tsl2540_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	int ret = -EINVAL;
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_INTENSITY:
			/*
			 * Avoid accessing the channels again within 1 als cycle
			 * as raw channel data will only be updated upon end of
			 * each als cycle.
			 */
			if (time_after_eq(jiffies,
					  chip->last_raw_access_time +
						  msecs_to_jiffies(
							  chip->tsl2540_settings
								  .als_time))) {
				tsl2540_get_channel_data(indio_dev);
			}
			if (chan->channel == 0)
				*val = chip->als_cur_info.als_ch0;
			else
				*val = chip->als_cur_info.als_ch1;
			ret = IIO_VAL_INT;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val =
		tsl2540_als_gains[chip->tsl2540_settings.als_gain].modified_value;
		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int tsl2540_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		switch (val) {
		case 1:
			chip->tsl2540_settings.als_gain = 0;
			break;
		case 2:
			chip->tsl2540_settings.als_gain = 1;
			break;
		case 8:
			chip->tsl2540_settings.als_gain = 2;
			break;
		case 32:
			chip->tsl2540_settings.als_gain = 3;
			break;
		case 128:
			chip->tsl2540_settings.als_gain = 4;
			break;
		case 256:
			chip->tsl2540_settings.als_gain = 5;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	tsl2540_invoke_change(indio_dev);

	return 0;
}

static DEVICE_ATTR(power_state, S_IRUGO | S_IWUSR,
		tsl2540_power_state_show, tsl2540_power_state_store);

static DEVICE_ATTR(in_intensity0_calibscale_available, S_IRUGO,
		tsl2540_gain_available_show, NULL);

static DEVICE_ATTR(in_intensity_integration_time, S_IRUGO | S_IWUSR,
		tsl2540_als_time_show, tsl2540_als_time_store);

static DEVICE_ATTR(in_intensity_wait_time_enable, S_IRUGO | S_IWUSR,
		tsl2540_als_wait_time_en_show, tsl2540_als_wait_time_en_store);

static DEVICE_ATTR(in_intensity_wait_time, S_IRUGO | S_IWUSR,
		tsl2540_als_wait_time_show, tsl2540_als_wait_time_store);

static DEVICE_ATTR(in_intensity0_thresh_period, S_IRUGO | S_IWUSR,
		tsl2540_als_persistence_show, tsl2540_als_persistence_store);

static DEVICE_ATTR(als_auto_gain_enable, S_IRUGO | S_IWUSR,
		tsl2540_auto_gain_show, tsl2540_auto_gain_store);

/* Use the default register values to identify the Taos device */
static int tsl2540_device_id(u8 chip_id, u8 rev_id, u8 aux_id)
{
	chip_id &= 0xfc; /* clear the 2 LSbits, they indicate the bus voltage */
	rev_id &= 0xe7; /* clear all but fuse bits */
	return (chip_id == TSL2540_ID_CHIP) && (rev_id == TSL2540_ID_REV) && (aux_id == TSL2540_ID_AUX);
}

static irqreturn_t tsl2540_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	s64 timestamp = iio_get_time_ns(indio_dev);
	int ret;
	u8 status;

	ret = tsl2540_i2c_read_byte(chip->client, TSL2540_REG_STATUS, &status);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: Failed to read STATUS Reg\n", __func__);
	}

	/* No interrupt has been generated from us */
	if (status == 0)
		return IRQ_NONE;

	if (status & (TSL2540_STATUS_ALS_INT | TSL2540_STATUS_SAT_INT)) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_INTENSITY,
						    0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       timestamp);
	}
	/* Clear interrupt now that we have handled it. */
	ret = i2c_smbus_write_byte_data(chip->client, TSL2540_REG_STATUS, status);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"i2c_write_command failed - err = %d\n", ret);
	}

	return IRQ_HANDLED;
}

static struct attribute *tsl2540_ALS_device_attrs[] = {
	&dev_attr_power_state.attr,
	&dev_attr_in_intensity0_calibscale_available.attr,
	&iio_const_attr_in_intensity0_integration_time_available.dev_attr.attr,
	&dev_attr_in_intensity_integration_time.attr,
	&dev_attr_in_intensity_wait_time_enable.attr,
	&dev_attr_in_intensity_wait_time.attr,
	&dev_attr_als_auto_gain_enable.attr,
	NULL
};

static struct attribute *tsl2540_ALS_event_attrs[] = {
	&dev_attr_in_intensity0_thresh_period.attr,
	NULL,
};

static const struct attribute_group tsl2540_device_attr_group = {
	.attrs = tsl2540_ALS_device_attrs,
};

static struct attribute_group tsl2540_event_attr_group = {
	.attrs = tsl2540_ALS_event_attrs,
	.name = "events",
};

static const struct iio_info tsl2540_device_info = {
	.attrs = &tsl2540_device_attr_group,
	.event_attrs = &tsl2540_event_attr_group,
	.read_raw = &tsl2540_read_raw,
	.write_raw = &tsl2540_write_raw,
	.read_event_value = &tsl2540_read_thresh,
	.write_event_value = &tsl2540_write_thresh,
	.read_event_config = &tsl2540_read_interrupt_config,
	.write_event_config = &tsl2540_write_interrupt_config,
};

static const struct iio_event_spec tsl2540_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct tsl2540_chip_info tsl2540_chip_info = {
	.channel = {
			{
			.type = IIO_INTENSITY,
			.indexed = 1,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_CALIBSCALE),
			.event_spec = tsl2540_events,
			.num_event_specs = ARRAY_SIZE(tsl2540_events),
			}, {
			.type = IIO_INTENSITY,
			.indexed = 1,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			},
	},
	.chan_table_elements = 2,
	.info = &tsl2540_device_info,
};

static int tsl2540_probe(struct i2c_client *clientp,
			 const struct i2c_device_id *id)
{
	int ret;
	u8 device_id;
	u8 aux_id;
	u8 rev_id;
	struct iio_dev *indio_dev;
	struct tsl2540_chip *chip;

	indio_dev = devm_iio_device_alloc(&clientp->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	chip->client = clientp;
	i2c_set_clientdata(clientp, indio_dev);
	chip->autozero = false;
	if (of_property_read_bool(clientp->dev.of_node, "autozero"))
		chip->autozero = true;

	dev_info(&chip->client->dev, "autozero value is  %d ", chip->autozero);

	/* Validate tsl2540 ids */
	ret = tsl2540_i2c_read_byte(chip->client, TSL2540_REG_ID, &device_id);
	if (ret < 0)
		return ret;

	ret = tsl2540_i2c_read_byte(chip->client, TSL2540_REG_REVID, &rev_id);
	if (ret < 0)
		return ret;

	ret = tsl2540_i2c_read_byte(chip->client, TSL2540_REG_REVID_2, &aux_id);
	if (ret < 0)
		return ret;

	if (!tsl2540_device_id(device_id, rev_id, aux_id)) {
		dev_info(&chip->client->dev,
			 "%s: i2c device found does not match expected id\n",
				__func__);
		return -EINVAL;
	}

	/*
	 * ALS functions can be invoked via user space poll
	 * or H/W interrupt. If busy return last sample.
	 */
	mutex_init(&chip->als_mutex);

	chip->tsl2540_chip_status = TSL2540_CHIP_UNKNOWN;
	chip->pdata = dev_get_platdata(&clientp->dev);
	chip->id = id->driver_data;
	chip->chip_info = &tsl2540_chip_info;

	indio_dev->info = chip->chip_info->info;
	indio_dev->dev.parent = &clientp->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = chip->client->name;
	indio_dev->channels = chip->chip_info->channel;
	indio_dev->num_channels = chip->chip_info->chan_table_elements;

	if (clientp->irq) {
		ret = devm_request_threaded_irq(&clientp->dev, clientp->irq,
						NULL,
						&tsl2540_event_handler,
						IRQF_ONESHOT |
						IRQF_SHARED,
						"TSL2540_event",
						indio_dev);
		if (ret) {
			dev_err(&clientp->dev,
				"%s: irq request failed", __func__);
			return ret;
		}

		chip->wakeup = of_property_read_bool(clientp->dev.of_node, "wakeup-source");
		dev_dbg(&clientp->dev, "wakeup-source=%d", chip->wakeup);

		if (chip->wakeup) {
			enable_irq_wake(clientp->irq);
			device_init_wakeup(&clientp->dev, true);
		}
	}

	/* Set up autozero configuration */
	if (chip->autozero) {
		ret = of_property_read_u32(clientp->dev.of_node, "autozero-iterations",
						&chip->autozero_iterations);
		if (ret < 0) {
			chip->autozero_iterations = TSL2540_DEFAULT_AZ_ITERATIONS;
			dev_warn(&clientp->dev, "Autozero iterations not specified in "
				"device tree, defaulting to %u\n",
				chip->autozero_iterations);
		}
		chip->autozero_iterations &= TSL2540_MAX_AZ_ITERATIONS;
		dev_info(&clientp->dev, "Autozero iterations set to %u\n",
			chip->autozero_iterations);
	}

	/* Load up the defaults */
	ret = tsl2540_defaults(chip);
	if (ret) {
		pr_err("tsl2540: insufficient default data to initialize");
		return ret;
	}

	/* Make sure the chip is on */
	tsl2540_chip_on(indio_dev);

	chip->last_again_check_time = jiffies;
	chip->last_raw_access_time = jiffies;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&clientp->dev,
			"%s: iio registration failed\n", __func__);
		return ret;
	}

	dev_info(&clientp->dev, "%s Light sensor found.\n", id->name);

	return 0;
}

static void tsl2540_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	disable_irq(client->irq);
	tsl2540_chip_off(indio_dev);
}

static int tsl2540_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int ret = 0;

	/*
	 * When used as a wakeup source, suspend and resume functions should
	 * not be used. Therefore, we return directly here
	 */
	if (chip->wakeup) {
		return ret;
	}

	if (chip->tsl2540_chip_status == TSL2540_CHIP_WORKING) {
		ret = tsl2540_chip_off(indio_dev);
		chip->tsl2540_chip_status = TSL2540_CHIP_SUSPENDED;
	}

	if (chip->pdata && chip->pdata->platform_power) {
		pm_message_t pmm = {PM_EVENT_SUSPEND};

		chip->pdata->platform_power(dev, pmm);
	}

	return ret;
}

static int tsl2540_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int ret = 0;

	/*
	 * When used as a wakeup source, suspend and resume functions should
	 * not be used. Therefore, we return directly here
	 */
	if (chip->wakeup) {
		return ret;
	}

	if (chip->pdata && chip->pdata->platform_power) {
		pm_message_t pmm = {PM_EVENT_RESUME};

		chip->pdata->platform_power(dev, pmm);
	}

	if (chip->tsl2540_chip_status == TSL2540_CHIP_SUSPENDED)
		ret = tsl2540_chip_on(indio_dev);

	return ret;
}

static int tsl2540_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	tsl2540_chip_off(indio_dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct i2c_device_id tsl2540_idtable[] = {
	{ "tsl2540", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tsl2540_idtable);

static const struct dev_pm_ops tsl2540_pm_ops = {
	.suspend = tsl2540_suspend,
	.resume  = tsl2540_resume,
};

/* Driver definition */
static struct i2c_driver tsl2540_driver = {
	.driver = {
		.name = "tsl2540",
		.pm = &tsl2540_pm_ops,
	},
	.id_table = tsl2540_idtable,
	.probe = tsl2540_probe,
	.remove = tsl2540_remove,
	.shutdown = tsl2540_shutdown,
};

module_i2c_driver(tsl2540_driver);

MODULE_AUTHOR("J. August Brenner<jbrenner@taosinc.com>");
MODULE_DESCRIPTION("TAOS tsl2540 ambient light sensor driver");
MODULE_LICENSE("GPL");
