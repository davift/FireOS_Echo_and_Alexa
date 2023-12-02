/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include "../br_private.h"
#include "br_eero.h"

static void br_eero_roam_send_pkt(struct br_eero_br *, u8 *);

/* This node has a new wifi station attached */
/* Called from mac80211, so fdb fixup should be completed */
void br_eero_roam_new_sta(u8 *sta_addr, struct net_device *dev)
{
	struct net_bridge_port *p = NULL;
	struct net_device *br_dev;
	struct net_bridge *br;
	struct br_eero_br *eero_br;

	rcu_read_lock();
	br_dev = netdev_master_upper_dev_get_rcu(dev);
	if (!br_dev) {
		rcu_read_unlock();
		return;
	}

	br = netdev_priv(br_dev);
	if (!br || !br->eero_ext.eero_br) {
		rcu_read_unlock();
		return;
	}
	rcu_read_unlock();

	eero_br = br->eero_ext.eero_br;
	if (!eero_br->roam || !(eero_br->roam->br) ||
	    !(eero_br->roam->eero_br) ||
	    eero_br->eero_roam_enabled == BR_EERO_ROAM_OFF) {
		roam_log_info("roam not enabled yet\n");
		return;
	}

	if (!eero_br->fdb_handler_cb) {
		roam_log_info("fdb callback not registered yet\n");
		return;
	}

	if (!sta_addr || !is_unicast_ether_addr(sta_addr)) {
		roam_log_info("incorrect sta addr\n");
		return;
	}

	if (br_eero_is_eero(sta_addr)) {
		roam_log_info("received notification for eero-sta: %pM\n",
			      sta_addr);
		return;
	}

	if (!dev) {
		roam_log_info("dev is NULL\n");
		return;
	}

	/* Update our fdb */
	rcu_read_lock();
	p = br_port_get_rcu(dev);

	if (!p) {
		rcu_read_unlock();
		roam_log_info("could not get port\n");
		return;
	}

	/* Currently we do not support vlans and that's why the vid below is
	 * set to 0. When we have vlan support we need to implement a
	 * filtering logic here. A similar filtering logic is in br_input.c
	 * br_handle_frame_finish.
	 */
	eero_br->fdb_handler_cb(eero_br->roam->br, p, sta_addr, 0, false);
	rcu_read_unlock();

	/* Send the network a new message*/
	roam_log_info("sending roam notification for sta: %pM\n", sta_addr);
	br_eero_roam_send_pkt(eero_br, sta_addr);
}
EXPORT_SYMBOL(br_eero_roam_new_sta);

/* Handle the incoming roaming alert messages */
int br_eero_roam_handle_pkt(struct br_eero_br *eero_br,
			    const struct sk_buff *skb)
{
	struct br_eero_roam_pkt *roam_pkt;
	struct net_bridge_port *p;

	roam_pkt = (struct br_eero_roam_pkt *)skb->data;

	p = br_port_get_rcu(skb->dev);
	/* We do not need vlan filtering logic here since this call is from
	 * br_handle_frame_finish and it already means it has passed through
	 * the check.
	 */
	roam_log_info("handling pkt for sta %pM from proxy_mesh %pM on %s\n",
		      roam_pkt->sta, roam_pkt->proxy_mesh, p->dev->name);
	eero_br->fdb_handler_cb(eero_br->roam->br, p, roam_pkt->sta, 0, false);
	return BR_EERO_FWD_PKT;
}

static void br_eero_create_roam_pkt(struct br_eero_br *eero_br,
				    struct br_eero_roam_pkt *data, u8 *sta_addr,
				    struct net_bridge_port *mesh_p)
{
	if (!data)
		return;
	data->identifier = EERO_ROAM_NOTIFIER;
	ether_addr_copy(data->proxy_mesh, mesh_p->dev->dev_addr);
	ether_addr_copy(data->sta, sta_addr);
	roam_log_debug("created pkt\n");
}

static void br_eero_roam_send_pkt(struct br_eero_br *eero_br, u8 *sta_addr)
{
	struct br_eero_roam_pkt data;
	struct net_bridge_port *mesh_p;

	rcu_read_lock();
	mesh_p = br_eero_get_mesh_port(eero_br->roam->br);
	if (!mesh_p) {
		roam_log_info("no mesh port yet\n");
		rcu_read_unlock();
		return;
	}

	br_eero_create_roam_pkt(eero_br, &data, sta_addr, mesh_p);
	rcu_read_unlock();
	br_eero_send_pkt_directly(eero_br->roam->br, (u8 *)&data,
				  sizeof(struct br_eero_roam_pkt));
	roam_log_debug("sent to all ifaces\n");
}

static struct br_eero_roam *br_eero_init_roam(struct br_eero_br *eero_br)
{
	struct br_eero_roam *roam;

	roam = kzalloc(sizeof(*roam), GFP_ATOMIC);
	if (!roam)
		return NULL;

	/* init roam structure internals */
	roam->br = eero_br->br;
	roam->eero_br = eero_br;

	roam_log_info("initialized\n");

	return roam;
}

static void br_eero_fini_roam(struct br_eero_br *eero_br)
{
	roam_log_info("deinitialized\n");
	kfree(eero_br->roam);
}

void br_eero_roam_set_enabled(struct br_eero_br *eero_br, unsigned long val)
{
	ASSERT_RTNL();

	if (val) {
		if (eero_br->eero_roam_enabled == BR_EERO_ROAM_OFF) {
			eero_br->roam = br_eero_init_roam(eero_br);
			eero_br->eero_roam_enabled = BR_EERO_ROAM_ON;
		}
	} else {
		if (eero_br->eero_roam_enabled == BR_EERO_ROAM_ON) {
			br_eero_fini_roam(eero_br);
			eero_br->eero_roam_enabled = BR_EERO_ROAM_OFF;
		}
	}
}
EXPORT_SYMBOL(br_eero_roam_set_enabled);
