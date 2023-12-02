/*
 * Copyright 2019 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <dt-bindings/i2c/i2c.h>

static struct i2c_client *i2c_register_device(struct i2c_adapter *adap,
						 struct device_node *node)
{
	struct i2c_client *result;
	struct i2c_board_info info = {};
	struct dev_archdata dev_ad = {};
	u32 addr;

	dev_dbg(&adap->dev, "of_i2c: register %s\n", node->full_name);

	if (of_modalias_node(node, info.type, sizeof(info.type)) < 0) {
		dev_err(&adap->dev, "of_i2c: modalias failure on %s\n",
			node->full_name);
		return ERR_PTR(-EINVAL);
	}

	if (of_property_read_u32(node, "reg", &addr)) {
		dev_err(&adap->dev, "%s: failed to read reg property\n",
			node->full_name);
		return ERR_PTR(-EINVAL);
	}

	if (addr & I2C_TEN_BIT_ADDRESS) {
		addr &= ~I2C_TEN_BIT_ADDRESS;
		info.flags |= I2C_CLIENT_TEN;
	}

	if (addr & I2C_OWN_SLAVE_ADDRESS) {
		addr &= ~I2C_OWN_SLAVE_ADDRESS;
		info.flags |= I2C_CLIENT_SLAVE;
	}

	info.addr = addr;
	info.of_node = of_node_get(node);
	info.archdata = &dev_ad;

	if (of_get_property(node, "wakeup-source", NULL))
		info.flags |= I2C_CLIENT_WAKE;

	result = i2c_new_device(adap, &info);
	if (result == NULL) {
		dev_err(&adap->dev, "of_i2c: Failure registering %s\n",
			node->full_name);
		of_node_put(node);
		return ERR_PTR(-EINVAL);
	}
	return result;
}

static int i2c_detect_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct device_node *node;
	u32 addr, id_reg, id_val;
	int ret;
	union i2c_smbus_data data;
	struct i2c_adapter *adap = client->adapter;

	for_each_available_child_of_node(client->dev.of_node, node) {
		if (of_property_read_u32(node, "reg", &addr)) {
			dev_err(&adap->dev, "%s: invalid reg\n",
				node->full_name);
			continue;
		}

		if (of_property_read_u32(node, "chipid-reg", &id_reg)) {
			dev_err(&adap->dev, "%s: invalid chipid_reg\n",
				node->full_name);
			continue;
		}

		if (of_property_read_u32(node, "chipid", &id_val)) {
			dev_err(&adap->dev, "%s: invalid chipid_val\n",
				node->full_name);
			continue;
		}

		ret = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, id_reg,
				I2C_SMBUS_BYTE_DATA, &data);
		if (ret < 0) {
			dev_err(&adap->dev, "%s: failed to read chipid_val\n",
				node->full_name);
			continue;
		}

		if (data.byte == id_val)
			i2c_register_device(adap, node);
	}

	return -ENODEV;
}

static const struct i2c_device_id i2c_detect_ids[] = {
	{ "i2c_detect", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_detect_ids);

static const struct of_device_id i2c_detect_of_match[] = {
	{ .compatible = "amz,i2cdetect" },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_detect_of_match);

static struct i2c_driver i2c_detect_driver = {
	.driver = {
		.name = "i2cdetect",
		.of_match_table = of_match_ptr(i2c_detect_of_match),
	},
	.probe = i2c_detect_probe,
	.id_table	= i2c_detect_ids,
};
module_i2c_driver(i2c_detect_driver);

MODULE_AUTHOR("Yadwinder Singh Brar<ybbrar@amazon.com>");
MODULE_DESCRIPTION("i2c device detected creation driver");
MODULE_LICENSE("GPL v2");
