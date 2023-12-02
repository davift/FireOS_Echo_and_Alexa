/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/capability.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/rtnetlink.h>
#include <linux/spinlock.h>

#include "br_eero.h"

#define to_bridge(cd) ((struct net_bridge *)netdev_priv(to_net_dev(cd)))

static ssize_t stamp_state_show(struct device *d, struct device_attribute *attr,
				char *buf)
{
	struct net_bridge *br = to_bridge(d);

	return sprintf(buf, "%d\n", br->eero_ext.eero_br->seg_enabled);
}

static ssize_t stamp_state_store(struct device *d,
				 struct device_attribute *attr, const char *buf,
				 size_t len)
{
	struct net_bridge *br = to_bridge(d);
	char *endp;
	unsigned long val;

	if (!ns_capable(dev_net(br->dev)->user_ns, CAP_NET_ADMIN))
		return -EPERM;

	val = simple_strtoul(buf, &endp, 0);
	if (endp == buf)
		return -EINVAL;

	if (!br->eero_ext.eero_br)
		return -EINVAL;

	if (!rtnl_trylock())
		return restart_syscall();
	br_eero_stamp_set_enabled(br->eero_ext.eero_br, val);
	rtnl_unlock();

	return len;
}
static DEVICE_ATTR_RW(stamp_state);

static ssize_t eero_roam_state_show(struct device *d,
				    struct device_attribute *attr, char *buf)
{
	struct net_bridge *br = to_bridge(d);

	return sprintf(buf, "%d\n", br->eero_ext.eero_br->eero_roam_enabled);
}

static ssize_t eero_roam_state_store(struct device *d,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct net_bridge *br = to_bridge(d);
	char *endp;
	unsigned long val;

	if (!ns_capable(dev_net(br->dev)->user_ns, CAP_NET_ADMIN))
		return -EPERM;

	val = simple_strtoul(buf, &endp, 0);
	if (endp == buf)
		return -EINVAL;

	if (!br->eero_ext.eero_br)
		return -EINVAL;

	if (!rtnl_trylock())
		return restart_syscall();
	br_eero_roam_set_enabled(br->eero_ext.eero_br, val);
	rtnl_unlock();

	return len;
}
static DEVICE_ATTR_RW(eero_roam_state);

static ssize_t stamp_fudge_metric_show(struct device *d,
				       struct device_attribute *attr, char *buf)
{
	struct net_bridge *br = to_bridge(d);

	return sprintf(buf, "%d\n", br->eero_ext.eero_br->fudged_stamp_metric);
}

static ssize_t stamp_fudge_metric_store(struct device *d,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct net_bridge *br = to_bridge(d);
	char *endp;
	unsigned long val;

	val = simple_strtoul(buf, &endp, 0);
	if (endp == buf)
		return -EINVAL;

	if (!br->eero_ext.eero_br)
		return -EINVAL;

	if (!rtnl_trylock())
		return restart_syscall();

	br->eero_ext.eero_br->fudged_stamp_metric = val;
	rtnl_unlock();

	return len;
}
static DEVICE_ATTR_RW(stamp_fudge_metric);

static struct attribute *eero_br_attrs[] = { &dev_attr_stamp_state.attr,
					     &dev_attr_eero_roam_state.attr,
					     &dev_attr_stamp_fudge_metric.attr,
					     NULL };

static struct attribute_group eero_bridge_group = {
	.name = SYSFS_EERO_BR_ATTR,
	.attrs = eero_br_attrs,
};

int br_eero_add_sysfs(struct net_device *dev, struct br_eero_br *eero_br)
{
	struct kobject *kobj = &dev->dev.kobj;
	int err;

	err = sysfs_create_group(kobj, &eero_bridge_group);
	if (err) {
		br_eero_debug(BR_EERO_LOG, eero_br,
			      "%s: can't create group %s/%s\n", __func__,
			      dev->name, eero_bridge_group.name);
		return err;
	}

	br_eero_debug(BR_EERO_LOG, eero_br, "sysfs inited");
	return 0;
}

int br_eero_del_sysfs(struct net_device *dev, struct br_eero_br *eero_br)
{
	struct kobject *kobj = &dev->dev.kobj;

	sysfs_remove_group(kobj, &eero_bridge_group);

	br_eero_debug(BR_EERO_LOG, eero_br, "sysfs deinited");

	return 0;
}
