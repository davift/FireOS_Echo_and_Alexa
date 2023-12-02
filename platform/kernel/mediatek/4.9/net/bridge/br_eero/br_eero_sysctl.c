/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/sysctl.h>

#include "br_eero.h"

static struct ctl_table_header *br_eero_sysctl_header;
static int br_eero_seg_ann_interval __read_mostly = EERO_SEG_ANNOUNCEMENTS;
static int br_eero_seg_id_ann_interval __read_mostly =
	EERO_SEG_ID_ANNOUNCEMENTS;
static int br_eero_all_stp_through_stamp_val __read_mostly;

int br_eero_get_seg_ann_interval(void)
{
	return br_eero_seg_ann_interval;
}

int br_eero_get_seg_id_ann_interval(void)
{
	return br_eero_seg_id_ann_interval;
}

int br_eero_all_stp_through_stamp(void)
{
	return br_eero_all_stp_through_stamp_val;
}
EXPORT_SYMBOL(br_eero_all_stp_through_stamp);

static struct ctl_table br_eero_table[] = {
	{
		.procname = "seg_announcement_interval",
		.data = &br_eero_seg_ann_interval,
		.maxlen = sizeof(int),
		.mode = 0644,
		.proc_handler = proc_dointvec,
	},
	{
		.procname = "seg_id_announcement_interval",
		.data = &br_eero_seg_id_ann_interval,
		.maxlen = sizeof(int),
		.mode = 0644,
		.proc_handler = proc_dointvec,
	},
	{
		.procname = "all_stp_through_stamp",
		.data = &br_eero_all_stp_through_stamp_val,
		.maxlen = sizeof(int),
		.mode = 0644,
		.proc_handler = proc_dointvec,
	},

	{}
};

int __init br_eero_sysctl_init(void)
{
	br_eero_sysctl_header = register_net_sysctl(
		&init_net, "net/eero_bridge", br_eero_table);
	if (!br_eero_sysctl_header)
		return -ENOMEM;
	return 0;
}

void br_eero_sysctl_fini(void)
{
	unregister_net_sysctl_table(br_eero_sysctl_header);
}
