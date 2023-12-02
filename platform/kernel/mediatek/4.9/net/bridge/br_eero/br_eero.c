/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/module.h>
#include <net/cfg80211.h>
#include <linux/netfilter_bridge.h>
#include <linux/pkt_sched.h>
#include <linux/slab.h>
#include <net/br_eero/skb_tracer.h>
#include "br_eero.h"
#include "../br_private.h"
#include <uapi/linux/br_eero_if_ether.h>

static struct br_eero_main *brm;

static DEFINE_RWLOCK(db_lock);

static void br_eero_send_pkt_to_all(struct net_bridge *, struct net_device *,
				    const u8 *, int, bool);

int br_eero_handle_skb(const struct sk_buff *skb, struct br_eero_br *eero_br)
{
	u8 type = br_eero_return_eero_type(skb);
	int ret = BR_EERO_NO_FW_PKT;

	if (type) {
		switch (type) {
		case EERO_NULL:
			br_eero_debug(BR_EERO_LOG, eero_br, "NULL eero type\n");
			break;
		case EERO_ETH_DETECTION:
			if (eero_br->seg_enabled == BR_EERO_SEG_ON)
				ret = br_eero_handle_seg_pkt(eero_br, skb);
			break;
		case EERO_SEG_IDENTIFIER:
			if (eero_br->seg_enabled == BR_EERO_SEG_ON)
				ret = br_eero_handle_seg_id_pkt(eero_br, skb);
			break;
		case EERO_ROAM_NOTIFIER:
			if (eero_br->eero_roam_enabled == BR_EERO_ROAM_ON)
				ret = br_eero_roam_handle_pkt(eero_br, skb);
			break;
		case EERO_ERR:
			br_eero_debug(BR_EERO_LOG, eero_br, "ERR eero type\n");
			break;
		}
	} else {
		br_eero_error(BR_EERO_LOG, eero_br, "Unknown packet type:%x\n",
			      type);
	}
	return ret;
}
EXPORT_SYMBOL(br_eero_handle_skb);

bool br_eero_is_mesh_interface_dev(const struct net_device *dev)
{
	struct wireless_dev *w_ptr = dev->ieee80211_ptr;

	if (dev->device_ext_tag == NET_DEVICE_EXT_EERO &&
	    ((dev->device_ext.eero.features & EERO_IFF_MBSS_PORT) != 0)) {
		return true;
	}
	/* in case modules don't know about EERO_IFF_MBSS_PORT yet,
	 * fall back to old behavior */
	if (w_ptr && w_ptr->iftype &&
	    w_ptr->iftype == NL80211_IFTYPE_MESH_POINT) {
		return true;
	}
	return false;
}

bool br_eero_is_mesh_interface(const struct net_bridge_port *p)
{
	return br_eero_is_mesh_interface_dev(p->dev);
}

bool br_eero_is_eth_interface_dev(const struct net_device *dev)
{
	if (dev->ieee80211_ptr || br_eero_is_mesh_interface_dev(dev)) {
		return false;
	}
	return true;
}

bool br_eero_is_eth_interface(const struct net_bridge_port *p)
{
	return br_eero_is_eth_interface_dev(p->dev);
}

bool br_eero_is_ap_interface(const struct net_bridge_port *p)
{
	struct net_device *dev = p->dev;
	struct wireless_dev *w_ptr = dev->ieee80211_ptr;

	if (w_ptr && w_ptr->iftype && w_ptr->iftype == NL80211_IFTYPE_AP)
		return true;
	return false;
}
EXPORT_SYMBOL(br_eero_is_ap_interface);

struct net_bridge_port *br_eero_get_mesh_port(struct net_bridge *br)
{
	struct net_bridge_port *p;

	list_for_each_entry_rcu (p, &br->port_list, list) {
		if (p->state != BR_STATE_DISABLED &&
		    br_eero_is_mesh_interface(p)) {
			return p;
		}
	}
	return NULL;
}

bool br_eero_is_eero_type(const struct sk_buff *skb)
{
	if (eth_hdr(skb)->h_proto == htons(ETH_P_BIFROST))
		return true;
	return false;
}

void br_eero_send_pkt_directly(struct net_bridge *br, const u8 *data,
			       int length)
{
	br_eero_send_pkt_to_all(br, br->dev, data, length, true);
}

void br_eero_send_pkt_all(struct net_bridge *br, const u8 *data, int length)
{
	/* Note that we are not sending packets to non mesh or eth interfaces,
	 * for example ap.
	 */
	br_eero_send_pkt_mesh(br, data, length);
	br_eero_send_pkt_eth(br, data, length, NULL);
}

void br_eero_send_pkt_mesh(struct net_bridge *br, const u8 *data, int length)
{
	struct net_bridge_port *p;

	list_for_each_entry_rcu (p, &br->port_list, list) {
		if (p->state != BR_STATE_DISABLED &&
		    br_eero_is_mesh_interface(p)) {
			br_eero_debug(BR_EERO_LOG, p->br->eero_ext.eero_br,
				      "sending to mesh ifaces\n");
			br_eero_send_pkt_to_all(br, p->dev, data, length,
						false);
		}
	}
}

void br_eero_send_pkt_eth(struct net_bridge *br, const u8 *data, int length,
			  struct net_bridge_port *p_no_send)
{
	struct net_bridge_port *p;

	list_for_each_entry_rcu (p, &br->port_list, list) {
		if (p->state != BR_STATE_DISABLED &&
		    br_eero_is_eth_interface(p) && p != p_no_send) {
			br_eero_debug(BR_EERO_LOG, p->br->eero_ext.eero_br,
				      "sending to eth ifaces\n");
			br_eero_send_pkt_to_all(br, p->dev, data, length,
						false);
		}
	}
}

static void br_eero_send_pkt_to_all(struct net_bridge *br,
				    struct net_device *dev, const u8 *data,
				    int length, bool unicast)
{
	struct sk_buff *skb, *skb2;
	struct ethhdr *ethhdr;
	struct mac_addr_list_entry *entry;

	skb = dev_alloc_skb(length + ETH_HLEN + NET_IP_ALIGN);
	if (!skb)
		return;

	skb->dev = dev;
	skb->protocol = htons(ETH_P_BIFROST);
	skb->priority = TC_PRIO_CONTROL;

	skb_reserve(skb, ETH_HLEN + NET_IP_ALIGN);
	memcpy(__skb_put(skb, length), data, length);

	skb_push(skb, ETH_HLEN);
	skb_reset_mac_header(skb);

	ethhdr = (struct ethhdr *)skb_mac_header(skb);
	ether_addr_copy(ethhdr->h_source, br->dev->dev_addr);
	ether_addr_copy(ethhdr->h_dest, br->dev->broadcast);
	ethhdr->h_proto = htons(ETH_P_BIFROST);
	skb_set_network_header(skb, ETH_HLEN);
	if (skb->len < SKB_MIN_SIZE) {
		skb_pad(skb, SKB_MIN_SIZE - skb->len);
		skb->len = SKB_MIN_SIZE;
	}

	if (!unicast) {
		br_eero_debug(BR_EERO_LOG, br->eero_ext.eero_br, "sending broadcast\n");
		dev_queue_xmit(skb);
		return;
	}

	read_lock(&db_lock);
	list_for_each_entry (entry, &brm->neigh_list, list) {
		skb2 = skb_copy(skb, GFP_ATOMIC);
		if (!skb2)
			goto exit;
		ethhdr = (struct ethhdr *)skb_mac_header(skb2);
		ether_addr_copy(ethhdr->h_dest, entry->addr);
		br_eero_debug(BR_EERO_LOG, br->eero_ext.eero_br,
			      "sending unicast to: %pM\n", entry->addr);
		dev_queue_xmit(skb2);
	}
exit:
	read_unlock(&db_lock);
	kfree_skb(skb);
}

/* This function returns false if @skb should be dropped according to eero
 * bridge (STAMP) rules, true if it should be received for further processing.
 */
bool br_eero_should_rx(struct net_bridge *br, struct sk_buff *skb)
{
	struct net_bridge_port *p = br_port_get_rcu(skb->dev);
	const unsigned char *dest = eth_hdr(skb)->h_dest;
	u8 addr[ETH_ALEN];

	/* only limit RX from mesh */
	if (!br->eero_ext.eero_br || !br->eero_ext.eero_br->seg_enabled ||
	    !br_eero_is_mesh_interface(p))
		return true;

	WARN_ON(!rcu_read_lock_held());
	br_eero_get_unique_seg_id(br->eero_ext.eero_br, addr);
	if (br_eero_is_self_designated_node(br, addr)) {
		/* this eero is the segment forwarder */
		if (skb->secmark & EERO_SEC_MARK_SAME_ETHERNET_DOMAIN) {
			/* skb arrived from a mesh proxy on this segment */
			skb_trace(skb, eth_hdr(skb)->h_dest,
				  eth_hdr(skb)->h_source);
			return false;
		}
	} else {
		/* if not forwarder, drop multicast or unicast flood */
		if (is_multicast_ether_addr(dest) ||
		    skb->secmark & EERO_SEC_MARK_UNICAST_FLOOD) {
			skb_trace(skb, eth_hdr(skb)->h_dest,
				  eth_hdr(skb)->h_source);
			return false;
		}
	}
	return true;
}
EXPORT_SYMBOL_GPL(br_eero_should_rx);

/* This function returns NET_RX_SUCCESS if fwd on this port is allowed based on
 * the criteria we decided for eth/mesh forwarding.
 */
/* called under bridge RCU lock */
bool br_eero_is_fwd_eligible(struct net_bridge *br, struct sk_buff *skb,
			     struct net_bridge_port *p)
{
	u8 seg_id_addr[ETH_ALEN];
	unsigned long update_time;
	bool is_self_designated;
	bool out_p_wireless_interface = false;
	bool out_p_mesh_interface = false;
	bool in_p_wireless_interface = false;
	bool in_p_mesh_interface = false;

	WARN_ON(!rcu_read_lock_held());
	update_time = br_eero_get_unique_seg_id(br->eero_ext.eero_br, seg_id_addr);
	is_self_designated = br_eero_is_self_designated_node(br, seg_id_addr);

	/* Update the flags */
	out_p_wireless_interface = !br_eero_is_eth_interface(p);
	out_p_mesh_interface = br_eero_is_mesh_interface(p);

	in_p_wireless_interface = !br_eero_is_eth_interface_dev(skb->dev);
	in_p_mesh_interface = br_eero_is_mesh_interface_dev(skb->dev);

	if (!(skb->secmark & EERO_SEC_MARK_UNICAST_FLOOD) &&
	    !is_multicast_ether_addr(eth_hdr(skb)->h_dest))
		return NET_RX_SUCCESS;

	/* Do the stamp conditional checks */
	/* Check if this is an STP frame and SONOS src. if so, do not run thru
	 * STAMP
	 */
	if (unlikely(is_link_local_ether_addr(eth_hdr(skb)->h_dest))) {
		if (!br_eero_all_stp_through_stamp() &&
		    (eth_hdr(skb)->h_dest)[5] == 0x00 &&
		    br_eero_is_sonos(br->eero_ext.eero_br, eth_hdr(skb)->h_source))
			return NET_RX_SUCCESS;
	}

	if (!is_self_designated) {
		if (in_p_mesh_interface && !out_p_wireless_interface)
			return NET_RX_DROP;
		if (!in_p_wireless_interface && out_p_mesh_interface)
			return NET_RX_DROP;
	} else {
		if (in_p_mesh_interface && !out_p_wireless_interface) {
			if (skb->secmark & EERO_SEC_MARK_SAME_ETHERNET_DOMAIN) {
				printk(KERN_ERR
				       "designated egress same eth src:%pM addr:%pM\n",
				       eth_hdr(skb)->h_source,
				       skb->dev->dev_addr);
				return NET_RX_DROP;
			}
		}
	}

	return NET_RX_SUCCESS;
}
EXPORT_SYMBOL(br_eero_is_fwd_eligible);

/* Note that these are loaded once and do not need to delete them as long as
 * the eero bridge is the same
 */
bool br_eero_add_mac_addr(const u8 *addr, int list_type)
{
	struct mac_addr_list_entry *entry = kmalloc(sizeof(*entry), GFP_KERNEL);

	if (!entry)
		return false;

	INIT_LIST_HEAD(&entry->list);
	ether_addr_copy(entry->addr, addr);
	write_lock_bh(&db_lock);
	if (list_type == EERO_OUI_LIST)
		list_add(&entry->list, &brm->eero_oui_list);
	else if (list_type == NEIGH_LIST)
		list_add(&entry->list, &brm->neigh_list);
	else if (list_type == SONOS_OUI_LIST)
		list_add(&entry->list, &brm->sonos_oui_list);
	write_unlock_bh(&db_lock);

	return true;
}

/* called under bridge RCU lock from eligible and input locations */
bool br_eero_is_sonos(struct br_eero_br *eero_br, u8 *addr)
{
	struct mac_addr_list_entry *entry;

	read_lock(&db_lock);
	list_for_each_entry(entry, &brm->sonos_oui_list, list) {
		if (ether_addr_equal_oui(addr, entry->addr)) {
			read_unlock(&db_lock);
			return true;
		}
	}
	read_unlock(&db_lock);
	return false;
}
EXPORT_SYMBOL(br_eero_is_sonos);

/* called under bridge RCU lock from eligible and input locations */
/* Accessed from the process context, so use _bh for locking */
int br_eero_print_add_list_text(struct seq_file *seq, void *offset,
				struct list_head *head)
{
	struct mac_addr_list_entry *entry;

	if (!head)
		return 1;

	read_lock_bh(&db_lock);
	list_for_each_entry (entry, head, list) {
		seq_printf(seq, "addr=%pM\n", entry->addr);
	}
	read_unlock_bh(&db_lock);
	return 0;
}

bool br_eero_is_eero(u8 *addr)
{
	struct mac_addr_list_entry *entry;

	if (list_empty(&brm->eero_oui_list)) {
		return true;
	}
	read_lock(&db_lock);
	list_for_each_entry(entry, &brm->eero_oui_list, list) {
		if (ether_addr_equal_oui(addr, entry->addr)) {
			read_unlock(&db_lock);
			return true;
		}
	}
	read_unlock(&db_lock);
	return false;
}
EXPORT_SYMBOL(br_eero_is_eero);

static void br_eero_deinit_br(struct br_eero_br *eero_br)
{
	kfree(eero_br);
	eero_br = NULL;
}

static struct br_eero_br *br_eero_init_br(void)
{
	struct br_eero_br *eero_br;

	eero_br = kzalloc(sizeof(*eero_br), GFP_ATOMIC);

	if (!eero_br)
		return NULL;

	/* initialize the variables */
	eero_br->br = NULL;
	eero_br->fdb_handler_cb = NULL;
	eero_br->seg_enabled = BR_EERO_SEG_OFF;
	eero_br->eero_roam_enabled = BR_EERO_ROAM_OFF;
	eero_br->log_level = BR_EERO_DEFAULT_LOG_LEVEL;

	br_eero_debug(BR_EERO_LOG, eero_br, "eero_br struct inited");
	return eero_br;
}

struct br_eero_br *br_eero_add_br(int (*cb)(struct net_bridge *br,
					    struct net_bridge_port *source,
					    const unsigned char *addr, u16 vid,
					    bool added_by_user),
				  struct net_device *dev)
{
	struct net_bridge *br = netdev_priv(dev);
	struct br_eero_br *eero_br;
	int err;

	eero_br = br_eero_init_br();
	if (!eero_br)
		return NULL;

	eero_br->fdb_handler_cb = cb;
	eero_br->br = br;

	err = br_eero_add_sysfs(dev, eero_br);
	if (err) {
		br_eero_deinit_br(eero_br);
		return NULL;
	}

	br_eero_debugfs_init_per_eero_br(eero_br);

	list_add(&br->eero_ext.list, &brm->br_list);

	return eero_br;
}
EXPORT_SYMBOL(br_eero_add_br);

int br_eero_rm_br(struct net_device *dev)
{
	int err = 0;
	struct net_bridge *br = netdev_priv(dev);
	struct br_eero_br *eero_br = br->eero_ext.eero_br;

	if (eero_br && eero_br->seg_enabled == BR_EERO_SEG_ON)
		br_eero_stamp_set_enabled(eero_br, 0);

	if (eero_br && eero_br->eero_roam_enabled == BR_EERO_ROAM_ON)
		br_eero_roam_set_enabled(eero_br, 0);

	br_eero_debugfs_deinit_per_eero_br(eero_br);

	err = br_eero_del_sysfs(dev, eero_br);
	if (err)
		return err;

	if (eero_br) {
		eero_br->fdb_handler_cb = NULL;
		eero_br->br = NULL;
		br_eero_debug(BR_EERO_LOG, br->eero_ext.eero_br,
			      "eero_br struct nulled");
	}

	br_eero_deinit_br(eero_br);

	list_del(&br->eero_ext.list);

	return 0;
}
EXPORT_SYMBOL(br_eero_rm_br);

static int __init br_eero_init(void)
{
	brm = kzalloc(sizeof(*brm), GFP_ATOMIC);
	INIT_LIST_HEAD(&brm->br_list);
	INIT_LIST_HEAD(&brm->sonos_oui_list);
	INIT_LIST_HEAD(&brm->eero_oui_list);
	INIT_LIST_HEAD(&brm->neigh_list);

	/* TODO: Why do we need to init here? it crashed when trying to init
	 * in add_br
	 */
	br_eero_sysctl_init();

	br_eero_debugfs_init(brm);

	br_eero_genl_init();

	return 0;
}

/* Before removing this module,  bridge interfaces should be removed. */
static void __exit br_eero_deinit(void)
{
	kfree(brm);
	brm = NULL;
	br_eero_genl_deinit();
	br_eero_debugfs_deinit();
	br_eero_sysctl_fini();
}

module_init(br_eero_init);
module_exit(br_eero_deinit);
MODULE_LICENSE("GPL");
MODULE_ALIAS_RTNL_LINK("bridge");
