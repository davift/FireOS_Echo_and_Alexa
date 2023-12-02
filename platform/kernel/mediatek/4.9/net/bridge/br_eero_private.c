/* eero Linux Ethernet Bridge extensions.
 * Copyright (c) 2019 eero Inc.
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * Authors:
 * Mete Rodoper (mete@eero.com)
 * Jas Strong (jas@eero.com)
 * Thia Wyrod (thia@eero.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include "br_eero_private.h"
#include "br_private.h"

#include <linux/if_ether.h>
#include <asm/bug.h>

int br_eero_net_bridge_ext_init(struct br_eero_net_bridge_ext *ext)
{
	ext->num_local_entries = 0;
	ext->eero_br = NULL;
	ext->rmc_cb = NULL;
	ext->rmc_priv = NULL;
	return 0;
}

int br_eero_net_bridge_ext_check_rmc(struct br_eero_net_bridge_ext const *ext,
				     struct sk_buff *skb)
{
	const u8 *src_addr;

	if (!ext->rmc_cb) {
		return 0;
	}

	src_addr = eth_hdr(skb)->h_source;
	return ext->rmc_cb(ext->rmc_priv, src_addr, skb, RMC_PATH_BR_RX,
			   PKT_8023);
}

void br_eero_net_bridge_ext_register_rmc(struct br_eero_net_bridge_ext *ext,
					 void *priv, rmc_cb_t cb)
{
	ext->rmc_priv = priv;
	ext->rmc_cb = cb;
}

void br_eero_net_bridge_ext_unregister_rmc(struct br_eero_net_bridge_ext *ext)
{
	ext->rmc_priv = NULL;
	ext->rmc_cb = NULL;
}

int br_rmc_check(struct net_bridge const *br, struct sk_buff *skb)
{
	if (!br) {
		return 0;
	}
	return br_eero_net_bridge_ext_check_rmc(&br->eero_ext, skb);
}

void br_register_rmc(struct net_device *brdev, void *priv, rmc_cb_t cb)
{
	struct net_bridge *br = netdev_priv(brdev);
	if (!br)
		return;
	return br_eero_net_bridge_ext_register_rmc(&br->eero_ext, priv, cb);
}
EXPORT_SYMBOL(br_register_rmc);

void br_unregister_rmc(struct net_device *brdev)
{
	struct net_bridge *br = netdev_priv(brdev);
	if (!br)
		return;
	return br_eero_net_bridge_ext_unregister_rmc(&br->eero_ext);
}
EXPORT_SYMBOL(br_unregister_rmc);

int br_check_skb_validity(struct sk_buff const *skb)
{
	if (!skb)
		return -1;

	if (unlikely(WARN_ONCE(!skb->data, "BR got no data in skb")))
		return -1;

	if (WARN_ONCE(!skb->tail, "BR skb had no tail"))
		return -1;

	if (WARN_ONCE(skb->len < sizeof(struct ethhdr),
		      "BR skb length was too small"))
		return -1;

	return 0;
}

bool br_eero_should_drop_frame(struct net_bridge const *br,
                               struct sk_buff *skb)
{
	return br->eero_ext.num_local_entries > 1 &&
	       is_multicast_ether_addr(eth_hdr(skb)->h_dest) &&
	       br_eero_net_bridge_ext_check_rmc(&br->eero_ext, skb);
}
