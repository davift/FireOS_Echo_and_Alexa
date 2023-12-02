// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/drivers/leds-pwm-rgb.c
 *
 * simple PWM based RGB LED control
 *
 * Copyright 2009 Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *
 * based on leds-gpio.c by Raphael Assenat <raph@8d.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>

enum rgb_index {
	red_index,
	green_index,
	blue_index,
};

struct led_pwm_data {
	struct led_classdev	cdev;
	struct device		*dev;
	struct pwm_device	*pwm;
	unsigned int		active_low;
	unsigned int		period;
	int			duty;
};

struct led_pwm_priv {
	int num_leds;
	struct mutex		led_access;
	struct led_pwm_data leds[0];
};

struct led_para {
	int color;
	int brightness;
};

//index 0->red, 1->green, 2->blue
struct rgb_leds {
	struct led_para para[3];
};

struct rgb_leds rgb_led;

static void __led_pwm_set(struct led_pwm_data *led_dat)
{
	int new_duty = led_dat->duty;

	dev_info(led_dat->dev, "rgb pwm num = %d, rgb new duty = %d, rgb pwm period = %d\n",
				led_dat->pwm->hwpwm, new_duty, led_dat->period);

	pwm_config(led_dat->pwm, new_duty, led_dat->period);

	if (new_duty == 0)
		pwm_disable(led_dat->pwm);
	else
		pwm_enable(led_dat->pwm);
}

static int led_pwm_set(struct led_classdev *led_cdev,
		       enum led_brightness brightness)
{
	struct led_pwm_data *led_dat =
		container_of(led_cdev, struct led_pwm_data, cdev);
	unsigned int max = led_dat->cdev.max_brightness;
	unsigned long long duty =  led_dat->period;

	duty *= brightness;
	do_div(duty, max);

	if (led_dat->active_low)
		duty = led_dat->period - duty;

	led_dat->duty = duty;

	__led_pwm_set(led_dat);

	return 0;
}

static int rgb_pwm_set(struct led_classdev *led_cdev, struct rgb_leds *leds, int led_index)
{
	int tmp_bright = 0;
	unsigned long long tmp;

	tmp = leds->para[led_index].brightness;
	tmp *= leds->para[led_index].color;

	do_div(tmp, 255);
	tmp_bright = tmp;

	led_pwm_set(led_cdev, tmp_bright);

	return 0;
}

static ssize_t rgb_color_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int index = 0;
	char color_info[64];

	sprintf(color_info, "red color = %d, green color = %d, blue color = %d\n",
						rgb_led.para[red_index].color,
						rgb_led.para[green_index].color,
						rgb_led.para[blue_index].color);

	memcpy(buf, color_info, strlen(color_info));
	index = strlen(color_info);

	return index;
}

static ssize_t rgb_color_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int color[3], i, ret;
	struct led_pwm_priv *pdata = dev_get_drvdata(dev);

	mutex_lock(&pdata->led_access);
	ret = sscanf(buf, "%d %d %d", &color[red_index], &color[green_index], &color[blue_index]);

	if (ret != 3) {
		dev_err(dev, "\trgb color need 3 paras\n");
		goto unlock;
	}

	//Set Red,Green and Blue LED color one by one
	for (i = 0; i < pdata->num_leds; i++) {
		rgb_led.para[i].color = color[i];
		if (&pdata->leds[i].cdev != NULL)
			rgb_pwm_set(&pdata->leds[i].cdev, &rgb_led, i);
	}

unlock:
	mutex_unlock(&pdata->led_access);

	return count;
}
static DEVICE_ATTR_RW(rgb_color);

static ssize_t rgb_brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int index = 0;
	char color_info[128];

	sprintf(color_info, "red brightness = %d, green brightness = %d, blue brightness = %d\n",
						rgb_led.para[red_index].brightness,
						rgb_led.para[green_index].brightness,
						rgb_led.para[blue_index].brightness);

	memcpy(buf, color_info, strlen(color_info));
	index = strlen(color_info);

	return index;
}

static ssize_t rgb_brightness_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long state;
	int brightness[3], ret, i;
	struct led_pwm_priv *pdata = dev_get_drvdata(dev);

	mutex_lock(&pdata->led_access);
	ret = kstrtoul(buf, 10, &state);

	if (ret)
		goto unlock;

	//Set Red,Green and Blue LED brightness one by one
	for (i = 0; i < pdata->num_leds; i++) {
		brightness[i] = state & 0xFF;
		rgb_led.para[i].brightness = brightness[i];

		if (&pdata->leds[i].cdev != NULL)
			rgb_pwm_set(&pdata->leds[i].cdev, &rgb_led, i);
	}

unlock:
	mutex_unlock(&pdata->led_access);

	return count;

}
static DEVICE_ATTR_RW(rgb_brightness);

static int led_pwm_add(struct device *dev, struct led_pwm_priv *priv,
		       struct led_pwm *led, struct fwnode_handle *fwnode)
{
	struct led_pwm_data *led_data = &priv->leds[priv->num_leds];
	struct pwm_args pargs;
	int ret;

	led_data->active_low = led->active_low;
	led_data->cdev.name = led->name;
	led_data->cdev.default_trigger = led->default_trigger;
	led_data->cdev.brightness = LED_OFF;
	led_data->cdev.max_brightness = led->max_brightness;
	led_data->cdev.flags = LED_CORE_SUSPENDRESUME;
	led_data->dev = dev;

	if (fwnode)
		led_data->pwm = devm_fwnode_pwm_get(dev, fwnode, NULL);
	else
		led_data->pwm = devm_pwm_get(dev, led->name);
	if (IS_ERR(led_data->pwm)) {
		ret = PTR_ERR(led_data->pwm);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "unable to request PWM for %s: %d\n",
				led->name, ret);
		return ret;
	}

	led_data->cdev.brightness_set_blocking = led_pwm_set;

	/*
	 * FIXME: pwm_apply_args() should be removed when switching to the
	 * atomic PWM API.
	 */
	pwm_apply_args(led_data->pwm);

	pwm_get_args(led_data->pwm, &pargs);

	led_data->period = pargs.period;
	if (!led_data->period && (led->pwm_period_ns > 0))
		led_data->period = led->pwm_period_ns;

	priv->num_leds++;
	led_pwm_set(&led_data->cdev, led_data->cdev.brightness);

	return ret;
}

static int led_pwm_create_fwnode(struct device *dev, struct led_pwm_priv *priv)
{
	struct fwnode_handle *fwnode;
	struct led_pwm led;
	int ret = 0;

	memset(&led, 0, sizeof(led));

	device_for_each_child_node(dev, fwnode) {
		ret = fwnode_property_read_string(fwnode, "label", &led.name);
		if (ret && is_of_node(fwnode))
			led.name = to_of_node(fwnode)->name;

		if (!led.name) {
			fwnode_handle_put(fwnode);
			return -EINVAL;
		}

		fwnode_property_read_string(fwnode, "linux,default-trigger",
					    &led.default_trigger);

		led.active_low = fwnode_property_read_bool(fwnode,
							   "active-low");
		fwnode_property_read_u32(fwnode, "max-brightness",
					 &led.max_brightness);

		ret = led_pwm_add(dev, priv, &led, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			break;
		}
	}

	return ret;
}

static int led_pwm_probe(struct platform_device *pdev)
{
	struct led_pwm_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct led_pwm_priv *priv;
	int count, i;
	int ret = 0;

	if (pdata)
		count = pdata->num_leds;
	else
		count = device_get_child_node_count(&pdev->dev);

	if (!count)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev, struct_size(priv, leds, count),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (pdata) {
		for (i = 0; i < count; i++) {
			ret = led_pwm_add(&pdev->dev, priv, &pdata->leds[i],
					  NULL);
			if (ret)
				break;
		}
	} else {
		ret = led_pwm_create_fwnode(&pdev->dev, priv);
	}

	if (ret)
		return ret;

	mutex_init(&priv->led_access);

	ret = device_create_file(&pdev->dev, &dev_attr_rgb_color);
	if (ret) {
		dev_err(&pdev->dev, "\tdevice create rgb color file error\n");
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_rgb_brightness);
	if (ret) {
		dev_err(&pdev->dev, "\tdevice create rgb color file error\n");
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static const struct of_device_id of_pwm_leds_match[] = {
	{ .compatible = "pwm-rgb-leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_leds_match);

static struct platform_driver led_pwm_driver = {
	.probe		= led_pwm_probe,
	.driver		= {
		.name	= "pwm_rgb_leds",
		.of_match_table = of_pwm_leds_match,
	},
};

module_platform_driver(led_pwm_driver);

MODULE_AUTHOR("Luotao Fu <l.fu@pengutronix.de>");
MODULE_DESCRIPTION("generic PWM LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:leds-pwm");
