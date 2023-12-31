/*
 * Thermal Framework Driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Dan Murphy <DMurphy@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#ifndef __LINUX_THERMAL_FRAMEWORK_H__
#define __LINUX_THERMAL_FRAMEWORK_H__

#define NUM_COOLERS 10
#include <linux/seq_file.h>

struct thermal_dev;
struct cooling_device;

/**
 * struct virtual_sensor_params  - Structure for each virtual sensor params.
 * @alpha:  Moving average coefficient
 * @offset: Temperature offset
 * @weight: Weight
 */
struct thermal_dev_params {
	int offset;
	int alpha;
	int weight;
};

/**
 * struct thermal_dev_ops  - Structure for device operation call backs
 * @get_temp: A temp sensor call back to get the current temperature.
 *		temp is reported in milli degrees.
 * @is_temp_in_range: A temp sensor call back to check if temperature read is
 *              in supported range of the thermal sensor.
 */
struct thermal_dev_ops {
	int (*get_temp)(struct thermal_dev *tdev, int *temp);
	int (*is_temp_in_range)(struct thermal_dev *tdev, bool *temp_in_range);
};

/**
 * struct thermal_dev  - Structure for each thermal device.
 * @name: The name of the device that is registering to the framework
 * @dev: Device node
 * @dev_ops: The device specific operations for the sensor, governor and cooling
 *           agents.
 * @node: The list node of the
 * @current_temp: The current temperature reported for the specific domain
 * @vs: The virtual sensor to which thermal sensor links to.
 *
 */
struct thermal_dev {
	const char *name;
	struct device *dev;
	struct thermal_dev_ops *dev_ops;
	struct list_head node;
	int current_temp;
	int vs;
};
static inline int thermal_dev_register(struct thermal_dev *tdev)
{
	return -EINVAL;
};

static inline int thermal_dev_deregister(struct thermal_dev *tdev)
{
	return -EINVAL;
};
#endif /* __LINUX_THERMAL_FRAMEWORK_H__ */
