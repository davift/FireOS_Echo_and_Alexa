/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifdef CONFIG_BIFROST_MESH
#ifndef _LINUX_EERO_BRIDGE_H
#define _LINUX_EERO_BRIDGE_H

#include <linux/types.h>
#include <linux/etherdevice.h>

#define EERO_SEC_MARK_INIT 0x0000
#define EERO_SEC_MARK_UNICAST_FLOOD 0x0001
#define EERO_SEC_MARK_SAME_ETHERNET_DOMAIN 0x0002
#define EERO_SEC_MARK_END 0xFFFF

/* EERO identifier types*/
#define EERO_NULL 0x00
#define EERO_ETH_DETECTION 0x01
#define EERO_SEG_IDENTIFIER 0x02 /* Used for unique segment ID numbers */
#define EERO_ROAM_NOTIFIER 0x03 /* Used for spreading roam information */
#define EERO_ERR 0xff /* Do not use */

struct br_eero_pkt {
	u8 identifier;
	u8 *data;
} __packed;

struct br_eero_seg_pkt {
	u8 identifier;
	u8 designated_root[ETH_ALEN];
	u8 forwarder[ETH_ALEN];
	u32 mesh_conn_value;
	u32 mbss;
} __packed;

struct br_eero_seg_id_pkt {
	u8 identifier;
	u8 unique_id[ETH_ALEN];
} __packed;

struct br_eero_roam_pkt {
	u8 identifier;
	u8 proxy_mesh[ETH_ALEN];
	u8 sta[ETH_ALEN];
} __packed;

bool br_eero_is_eero(u8 *);
#endif
#endif /* CONFIG_BIFROST_MESH */
