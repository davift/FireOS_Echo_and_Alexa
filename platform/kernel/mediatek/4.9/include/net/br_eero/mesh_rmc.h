/*
 * Copyright (c) 2016 eero Ltd.
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * Authors:    Mete Rodoper <mete@eero.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_BIFROST_MESH
#ifndef MESH_RMC_H
#define MESH_RMC_H

#include <linux/skbuff.h>
#include <linux/netdevice.h>

/**
 * enum pkt_type - different packet types that flow inside mac80211
 *
 *
 *
 * @PKT_8023: Ethernet packet
 * @PKT80211S: Mesh packet
 * @PKT80211: WiFi packet
 */
enum pkt_type {
	PKT_8023 = 0,
	PKT_80211S = 1,
	PKT_80211 = 2,
	PKT_UNICAST_FLOOD = 3, /* 80211S header type */
};

/**
 * enum mesh_rmc_path - different paths RMC can be called
 *
 *
 *
 * @RMC_PATH_RX =  0,
 * @RMC_PATH_TX =  1,
 * @RMC_PATH_FWD = 2,
 */

enum mesh_rmc_path {
	RMC_PATH_MAC_RX = 0,
	RMC_PATH_MAC_TX = 1,
	RMC_PATH_MAC_FWD = 2,
	RMC_PATH_BR_RX = 3,
	RMC_PATH_BR_RX_UNICAST = 4,
	RMC_PATH_MAC_FWD_UNICAST = 5,
};

typedef int (*rmc_cb_t)(void *, const u8 *, struct sk_buff *,
			enum mesh_rmc_path, enum pkt_type);

void br_register_rmc(struct net_device *brdev, void *priv, rmc_cb_t cb);
void br_unregister_rmc(struct net_device *brdev);

#endif /* MESH_RMC_H */
#endif /* CONFIG_BIFROST_MESH */
