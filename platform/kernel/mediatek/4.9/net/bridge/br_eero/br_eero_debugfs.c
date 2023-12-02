/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/debugfs.h>

#include "br_eero.h"
#include "../br_private.h"

static struct dentry *br_eero_debugfs_root;

/* Note that user context is going to be accessing here and these functions
 * should be protected with _bh lock calls
 */
static ssize_t br_eero_log_level_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t br_eero_log_level_read(struct file *file, char __user *userbuf,
				      size_t count, loff_t *pos)
{
	struct br_eero_br *eero_br = file->private_data;
	char buf[8];
	ssize_t ret = -EINVAL;

	read_lock(&dev_base_lock);
	ret = snprintf(buf, sizeof(buf), "0x%02X\n", eero_br->log_level);
	read_unlock(&dev_base_lock);

	if (ret >= 0)
		ret = simple_read_from_buffer(userbuf, count, pos, buf, ret);

	return ret;
}

static ssize_t br_eero_log_level_write(struct file *file,
				       char const __user *userbuf, size_t count,
				       loff_t *pos)
{
	char buf[8];
	ssize_t ret;
	u32 val;
	struct br_eero_br *eero_br = file->private_data;

	if (count >= sizeof(buf))
		return -E2BIG;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = -ENODEV;
	rtnl_lock();
	ret = kstrtou32(buf, 0, &val);
	if (ret) {
		rtnl_unlock();
		return ret;
	}

	eero_br->log_level = val;
	ret = (ssize_t)count;
	rtnl_unlock();

	return ret;
}

static struct br_eero_debuginfo br_eero_debuginfo_stamp_log_level = {
	.attr = {
		.name = "log_level",
		.mode = S_IRUGO | S_IWUSR,
	},
	.fops = { .owner = THIS_MODULE,
		.read = br_eero_log_level_read,
		.write = br_eero_log_level_write,
		.open = br_eero_log_level_open,
		.llseek = no_llseek,
	}
};

static ssize_t br_eero_fdb_update_addr_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	file->private_data = inode->i_private;

	return 0;
}

extern int br_eero_fdb_delete(struct net_bridge_port *p,
		                const unsigned char *addr, u16 vid);

static ssize_t br_eero_fdb_update_addr_write(struct file *file,
				char const __user *userbuf, size_t count,
				loff_t *pos)
{
	char buf[60] = { 0 };
	char intf_name[IFNAMSIZ] = { 0 };
	unsigned char addr[6];

	struct net_device *dev = (struct net_device *)NULL;
	struct net_device *br_dev = (struct net_device *)NULL;
	struct net_bridge *br = (struct net_bridge *)NULL;
	struct net_bridge_port *p = (struct net_bridge_port *)NULL;

	ssize_t ret = 0;

	if (count >= sizeof(buf))
		return -E2BIG;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = '\0';
	ret = (ssize_t)count;

	rtnl_lock();

	sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x %s",
		(unsigned int *)&addr[0], (unsigned int *)&addr[1], (unsigned int *)&addr[2],
		(unsigned int *)&addr[3], (unsigned int *)&addr[4], (unsigned int *)&addr[5],
		intf_name);

	if (!ret) {
		rtnl_unlock();
		return ret;
	}

	rcu_read_lock();

	/* find a netdevice that has name intf_name */
	if (!(dev = dev_get_by_name(&init_net, intf_name))) {
		ret = -EFAULT;
		goto br_err;
	}

	/* find br netdev */
	if (!(br_dev = netdev_master_upper_dev_get_rcu(dev))) {
		ret = -EFAULT;
		goto br_err;
	}

	/* get bridge object */
	if (!(br = netdev_priv(br_dev))) {
		ret = -EFAULT;
		goto br_err;
	}

	/* find bridge port */
	if (!(p = br_port_get_rcu(dev))) {
		ret = -EFAULT;
		goto br_err;
	}

	/* updating fdb entries */
	br_eero_fdb_delete(p, addr, 0);

br_err:
	rcu_read_unlock();
	rtnl_unlock();

	return ret;
}

struct br_eero_debuginfo br_eero_fdb_update_addr = {
	.attr = {
		.name = "fdb_update_addr",
		.mode = S_IRUGO | S_IWUSR,
	},
	.fops = { .owner = THIS_MODULE,
		.write = br_eero_fdb_update_addr_write,
		.open = br_eero_fdb_update_addr_open,
		.llseek = no_llseek,
	}
};

static ssize_t br_eero_debuginfo_neigh_write(struct file *file,
			char const __user *userbuf, size_t count,
			loff_t *pos)
{
	size_t ret = 0;
	char addr_str[30] = { 0 };
	char addr[ETH_ALEN] = { 0 };

        if (count >= sizeof(addr_str))
                return -E2BIG;

        if (copy_from_user(addr_str, userbuf, count))
                return -EFAULT;

        addr_str[count] = '\0';
        ret = (ssize_t)count;

	if (ret < 0)
		return ret;

	pr_debug("%s: %s\n", __func__, addr_str);
	sscanf(addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
		(unsigned int *)&addr[0], (unsigned int *)&addr[1], (unsigned int *)&addr[2],
		(unsigned int *)&addr[3], (unsigned int *)&addr[4], (unsigned int *)&addr[5]);

	br_eero_add_mac_addr(addr, NEIGH_LIST);

	return ret;
}

static ssize_t br_eero_debuginfo_eero_oui_write(struct file *file,
			char const __user *userbuf, size_t count,
			loff_t *pos)
{
	size_t ret = 0;
	char addr_str[30] = { 0 };
	char addr[ETH_ALEN] = { 0 };

        if (count >= sizeof(addr_str))
                return -E2BIG;

        if (copy_from_user(addr_str, userbuf, count))
                return -EFAULT;

        addr_str[count] = '\0';
        ret = (ssize_t)count;

	if (ret < 0)
		return ret;

	pr_debug("%s: %s\n", __func__, addr_str);
	sscanf(addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
		(unsigned int *)&addr[0], (unsigned int *)&addr[1], (unsigned int *)&addr[2],
		(unsigned int *)&addr[3], (unsigned int *)&addr[4], (unsigned int *)&addr[5]);

	br_eero_add_mac_addr(addr, EERO_OUI_LIST);

	return ret;
}

static ssize_t br_eero_debuginfo_sonos_oui_write(struct file *file,
			char const __user *userbuf, size_t count,
			loff_t *pos)
{
	size_t ret = 0;
	char addr_str[30] = { 0 };
	char addr[ETH_ALEN] = { 0 };

        if (count >= sizeof(addr_str))
                return -E2BIG;

        if (copy_from_user(addr_str, userbuf, count))
                return -EFAULT;

        addr_str[count] = '\0';
        ret = (ssize_t)count;

	if (ret < 0)
		return ret;

	pr_debug("%s: %s\n", __func__, addr_str);
	sscanf(addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
		(unsigned int *)&addr[0], (unsigned int *)&addr[1], (unsigned int *)&addr[2],
		(unsigned int *)&addr[3], (unsigned int *)&addr[4], (unsigned int *)&addr[5]);

	br_eero_add_mac_addr(addr, SONOS_OUI_LIST);

	return ret;
}

static ssize_t br_eero_debuginfo_segments_write(struct file *file,
			char const __user *userbuf, size_t count,
			loff_t *pos)
{
	/* not supported for this debugfs file */
	return -EOPNOTSUPP;
}

static ssize_t br_eero_debuginfo_segments_ext_write(struct file *file,
			char const __user *userbuf, size_t count,
			loff_t *pos)
{
	/* not supported for this debugfs file */
	return -EOPNOTSUPP;
}

static ssize_t br_eero_debuginfo_segments_id_write(struct file *file,
			char const __user *userbuf, size_t count,
			loff_t *pos)
{
	/* not supported for this debugfs file */
	return -EOPNOTSUPP;
}

static ssize_t br_eero_debuginfo_segment_cache_write(struct file *file,
			char const __user *userbuf, size_t count,
			loff_t *pos)
{
	/* not supported for this debugfs file */
	return -EOPNOTSUPP;
}
#define CREATE_DEBUGFS_FUNC(struct_name, cb_fn)                                \
	static ssize_t struct_name##_open(struct inode *inode,                 \
					  struct file *file)                   \
	{                                                                      \
		struct net_device *net_dev =                                   \
			(struct net_device *)inode->i_private;                 \
                                                                               \
		return single_open(file, cb_fn, net_dev);                      \
	}

#define CREATE_DEBUGFS_STRUCT(struct_name, fs_name)                            \
	struct br_eero_debuginfo struct_name = {			\
	.attr = { .name = #fs_name,				\
		.mode = S_IRUGO | S_IWUGO, },			\
	.fops = { .owner = THIS_MODULE,				\
		.open = struct_name## _open,			\
		.read = seq_read,				\
		.llseek = seq_lseek,				\
		.release = single_release,			\
		.write = struct_name## _write,			\
	}							\
}

#define CREATE_DEBUGFS_BUNDLE(struct_name, fs_name, cb_fn)                     \
	CREATE_DEBUGFS_FUNC(struct_name, cb_fn)                                \
	CREATE_DEBUGFS_STRUCT(struct_name, fs_name)

CREATE_DEBUGFS_BUNDLE(br_eero_debuginfo_segments, segments_debug,
		      br_eero_print_segment_text);
CREATE_DEBUGFS_BUNDLE(br_eero_debuginfo_segments_ext, segments_debug_ext,
		      br_eero_print_segment_ext_text);
CREATE_DEBUGFS_BUNDLE(br_eero_debuginfo_segments_id, segments_id_debug,
		      br_eero_print_segment_id_text);
CREATE_DEBUGFS_BUNDLE(br_eero_debuginfo_sonos_oui, sonos_oui_list,
		      br_eero_print_sonos_oui_text);
CREATE_DEBUGFS_BUNDLE(br_eero_debuginfo_eero_oui, eero_oui_list,
		      br_eero_print_eero_oui_text);
CREATE_DEBUGFS_BUNDLE(br_eero_debuginfo_neigh, neigh_list,
		      br_eero_print_neigh_text);
CREATE_DEBUGFS_BUNDLE(br_eero_debuginfo_segment_cache, segment_cache,
		      br_eero_print_segment_cache);

#define CREATE_DEBUGFS_FILE(structure, dfile, private)                         \
	do {                                                                   \
		file = debugfs_create_file(structure.attr.name,                \
					   structure.attr.mode, dfile,         \
					   private, &structure.fops);          \
		if (!file)                                                     \
			goto err;                                              \
	} while (0)

void br_eero_debugfs_init(struct br_eero_main *brm)
{
	struct dentry *file;

	if (!br_eero_debugfs_root) {
		br_eero_debugfs_root =
			debugfs_create_dir(DEBUGFS_EERO_BR_DIR, NULL);
		if (br_eero_debugfs_root == ERR_PTR(-ENODEV))
			br_eero_debugfs_root = NULL;

		if (!br_eero_debugfs_root)
			goto err;
	}

	CREATE_DEBUGFS_FILE(br_eero_debuginfo_sonos_oui, br_eero_debugfs_root,
			    brm);
	CREATE_DEBUGFS_FILE(br_eero_debuginfo_eero_oui, br_eero_debugfs_root,
			    brm);
	CREATE_DEBUGFS_FILE(br_eero_debuginfo_neigh, br_eero_debugfs_root, brm);

	return;
err:
	debugfs_remove_recursive(br_eero_debugfs_root);
	br_eero_debugfs_root = NULL;
}

void br_eero_debugfs_deinit(void)
{
	debugfs_remove_recursive(br_eero_debugfs_root);
	br_eero_debugfs_root = NULL;
}

void br_eero_debugfs_init_per_eero_br(struct br_eero_br *eero_br)
{
	struct dentry *file, *br_eero_debugfs;
	int err;

	if (!br_eero_debugfs_root)
		goto err;

	eero_br->debugfs_root = br_eero_debugfs_root;

	br_eero_debugfs = debugfs_create_dir(eero_br->br->dev->name,
					     br_eero_debugfs_root);
	if (br_eero_debugfs == ERR_PTR(-ENODEV))
		br_eero_debugfs = NULL;

	if (!br_eero_debugfs)
		goto err;

	eero_br->debugfs_br = br_eero_debugfs;

	err = br_eero_debug_log_setup(eero_br->debugfs_br, eero_br);
	if (err) {
		br_eero_error(BR_EERO_LOG, eero_br,
			      "log file debugfs init failed\n");
		goto err;
	}

	CREATE_DEBUGFS_FILE(br_eero_debuginfo_stamp_log_level,
			    eero_br->debugfs_br, eero_br);
	CREATE_DEBUGFS_FILE(br_eero_fdb_update_addr,
			    eero_br->debugfs_br, eero_br);

	br_eero_debug(BR_EERO_LOG, eero_br, "debugfs init completed\n");
	return;
err:
	br_eero_debug(BR_EERO_LOG, eero_br, "debugfs init failed\n");

	debugfs_remove_recursive(eero_br->debugfs_br);
	eero_br->debugfs_br = NULL;
}

void br_eero_debugfs_deinit_per_eero_br(struct br_eero_br *eero_br)
{
	br_eero_debug_log_cleanup(eero_br);

	debugfs_remove_recursive(eero_br->debugfs_br);
	eero_br->debugfs_br = NULL;
	br_eero_debug(BR_EERO_LOG, eero_br, "debugfs deinit completed\n");
}

void br_eero_debugfs_init_stamp(struct br_eero_br *eero_br)
{
	struct dentry *file, *br_eero_debugfs_stamp;

	if (!eero_br->debugfs_br) {
		br_eero_error(BR_EERO_LOG, eero_br,
			      "debugfs has not been enabled yet\n");
		return;
	}

	br_eero_debugfs_stamp =
		debugfs_create_dir("stamp", eero_br->debugfs_br);
	if (br_eero_debugfs_stamp == ERR_PTR(-ENODEV))
		br_eero_debugfs_stamp = NULL;

	if (!br_eero_debugfs_stamp) {
		br_eero_error(BR_EERO_LOG, eero_br,
			      "stamp debugfs init failed while dir create\n");
		return;
	}

	eero_br->debugfs_stamp = br_eero_debugfs_stamp;

	CREATE_DEBUGFS_FILE(br_eero_debuginfo_segments, eero_br->debugfs_stamp,
			    eero_br);
	CREATE_DEBUGFS_FILE(br_eero_debuginfo_segments_ext,
			    eero_br->debugfs_stamp, eero_br);
	CREATE_DEBUGFS_FILE(br_eero_debuginfo_segments_id,
			    eero_br->debugfs_stamp, eero_br);
	CREATE_DEBUGFS_FILE(br_eero_debuginfo_segment_cache,
			    eero_br->debugfs_stamp, eero_br);

	br_eero_debug(BR_EERO_LOG, eero_br, "stamp debugfs init completed\n");
	return;
err:
	debugfs_remove_recursive(eero_br->debugfs_stamp);
	eero_br->debugfs_stamp = NULL;
	br_eero_error(BR_EERO_LOG, eero_br, "stamp debugfs init failed\n");
}

void br_eero_debugfs_deinit_stamp(struct br_eero_br *eero_br)
{
	debugfs_remove_recursive(eero_br->debugfs_stamp);
	eero_br->debugfs_stamp = NULL;
	br_eero_debug(BR_EERO_LOG, eero_br, "stamp debugfs deinit completed\n");
}
