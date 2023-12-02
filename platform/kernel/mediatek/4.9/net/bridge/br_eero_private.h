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
#ifdef CONFIG_BIFROST_MESH
#ifndef _EERO_BR_PRIVATE_H
#define _EERO_BR_PRIVATE_H

#include <linux/types.h>
#include <net/br_eero/mesh_rmc.h>

#define EERO_FDB_UPDATE_HOLDOFF 20 /* in ms */

struct br_eero_net_bridge_nf_ext {
	u32 mark_to_call_iptables;
};

struct br_eero_net_bridge_ext {
	u8 num_local_entries;
	/* eero bifrost RMC things */
	void *rmc_priv; /* RMC context */
	rmc_cb_t rmc_cb; /* RMC implementation */

	struct br_eero_br *eero_br;
	struct list_head list;
};

#ifdef CONFIG_EERO_BR
int br_eero_net_bridge_ext_init(struct br_eero_net_bridge_ext *ext);
#else
#define BR_EERO_INIT_SUCCESS 0
static inline int br_eero_net_bridge_ext_init(struct br_eero_net_bridge_ext *ext) {
	return BR_EERO_INIT_SUCCESS;
}
#endif
int br_eero_net_bridge_ext_check_rmc(struct br_eero_net_bridge_ext const *ext,
				     struct sk_buff *skb);
void br_eero_net_bridge_ext_register_rmc(struct br_eero_net_bridge_ext *ext,
					 void *priv, rmc_cb_t cb);
void br_eero_net_bridge_ext_unregister_rmc(struct br_eero_net_bridge_ext *ext);

/* NB: This function is unused. It was possibly used to debug at some point
 * during eero bridge bringup?
 */
int br_check_skb_validity(struct sk_buff const *skb);

#endif /* _EERO_BR_PRIVATE_H */
#endif /* CONFIG_BIFROST_MESH */
