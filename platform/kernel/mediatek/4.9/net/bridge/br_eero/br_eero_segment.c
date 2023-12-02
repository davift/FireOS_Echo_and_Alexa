/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <net/cfg80211.h>
#include <linux/list.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/rwlock_types.h>

#include "../br_private.h"
#include "br_eero.h"

static struct kmem_cache *br_eero_segment_cache __read_mostly;

static DEFINE_RWLOCK(seg_lock);
static DEFINE_RWLOCK(seg_lock_ext);
static DEFINE_RWLOCK(seg_lock_same_seg);

void br_eero_choose_designated_root(struct br_eero_br *, u8 *, bool);

/* Segment/member list operations */
/* called under write_lock */
static struct br_eero_seg_member *
br_eero_add_member_to_segment(struct br_eero_br *eero_br,
			      struct br_eero_segment_entry *seg,
			      u8 *sender_addr, u32 mesh_conn_value, u32 mbss)
{
	struct br_eero_seg_member *member;

	member = kmalloc(sizeof(*member), GFP_ATOMIC);
	if (!member)
		return NULL;
	ether_addr_copy(member->addr, sender_addr);
	member->mesh_conn_value = mesh_conn_value;
	member->mbss = mbss;
	member->update_time = jiffies;
	list_add(&member->list, &seg->member_list);
	segment_log_info("added member: %pM to segment: %pM\n", sender_addr,
			 seg->designated_root);

	return member;
}

/* called under write_lock */
static struct br_eero_segment_entry *
br_eero_add_segment_list(struct br_eero_br *eero_br, u8 *designated_root,
			 u8 *forwarder, u32 mesh_conn_value, u32 mbss)
{
	struct br_eero_segment_entry *seg;

	seg = kmem_cache_zalloc(br_eero_segment_cache, GFP_ATOMIC);
	if (!seg)
		return NULL;
	/* Init the member list */
	ether_addr_copy(seg->designated_root, designated_root);
	INIT_LIST_HEAD(&seg->member_list);
	if (!br_eero_add_member_to_segment(eero_br, seg, forwarder,
					   mesh_conn_value, mbss)) {
		kfree(seg);
		return NULL;
	}
	list_add(&seg->list, &eero_br->segment->seg_list);
	segment_log_info("added segment: %pM\n", designated_root);
	return seg;
}

/* Determines if an ethernet address is on the same node as another ethernet
 * address. This is needed because we sometimes see frames from the other mesh
 * interface, which is not in the bridge, and therefore we don't know about it.
 */
static inline bool eero_janky_ether_equal(const u8 *addr1, const u8 *addr2)
{
	const u16 *a = (const u16 *)addr1;
	const u16 *b = (const u16 *)addr2;

	if (likely((a[0] ^ b[0]) | (a[1] ^ b[1]) | (addr1[4] ^ addr2[4])))
		return false;

	if ((addr1[5] ^ addr2[5]) < 0x10)
		return true;

	return false;
}

static bool br_eero_is_active(struct br_eero_br *eero_br,
			      struct br_eero_seg_member *member)
{
	return time_before(jiffies,
			   member->update_time +
				   eero_br->segment->member_timeout_duration);
}

static struct br_eero_seg_member *
br_eero_find_member_in_segment(struct br_eero_br *eero_br,
			       struct br_eero_segment_entry *seg,
			       u8 *search_addr, bool has_to_be_active)
{
	struct br_eero_seg_member *member;

	list_for_each_entry (member, &seg->member_list, list) {
		if (eero_janky_ether_equal(member->addr, search_addr)) {
			if (has_to_be_active) {
				if (br_eero_is_active(eero_br, member))
					return member;
				continue;
			}
			return member;
		}
	}
	return NULL;
}

/* called under write_lock */
static struct br_eero_seg_member *
br_eero_update_member(struct br_eero_br *eero_br,
		      struct br_eero_segment_entry *seg, u8 *forwarder,
		      u32 mesh_conn_value, u32 mbss)
{
	struct br_eero_seg_member *curr_member;

	curr_member =
		br_eero_find_member_in_segment(eero_br, seg, forwarder, false);
	if (!curr_member) {
		curr_member = br_eero_add_member_to_segment(
			eero_br, seg, forwarder, mesh_conn_value, mbss);
		return curr_member;
	}
	ether_addr_copy(curr_member->addr, forwarder);
	curr_member->mesh_conn_value = mesh_conn_value;
	curr_member->mbss = mbss;
	curr_member->update_time = jiffies;
	segment_log_debug("updated member: %pM in segment: %pM\n", forwarder,
			  seg->designated_root);
	return curr_member;
}

/* called under lock */
static struct br_eero_segment_entry *
br_eero_find_segment_list(struct br_eero_br *eero_br, u8 *designated_root)
{
	struct br_eero_segment_entry *seg;

	if (!eero_br->segment)
		return NULL;
	list_for_each_entry (seg, &eero_br->segment->seg_list, list) {
		if (ether_addr_equal(seg->designated_root, designated_root))
			return seg;
	}
	return NULL;
}

void br_eero_reset_segment_cache(struct br_eero_br *eero_br)
{
	write_lock(&seg_lock_same_seg);
	segment_log_debug("retiring same segment cache\n");
	eth_zero_addr(eero_br->segment->same_seg_cache);
	write_unlock(&seg_lock_same_seg);
}

static void br_eero_expire_members(struct br_eero_br *eero_br)
{
	struct br_eero_segment_entry *seg, *tmp_seg;
	struct br_eero_seg_member *member, *tmp_member;

	write_lock_bh(&seg_lock);
	list_for_each_entry_safe (seg, tmp_seg, &eero_br->segment->seg_list,
				  list) {
		list_for_each_entry_safe (member, tmp_member, &seg->member_list,
					  list) {
			if (br_eero_is_active(eero_br, member)) {
				continue;
			}
			segment_log_info(
				"expiring member: %pM in segment: %pM\n",
				member->addr, seg->designated_root);
			list_del(&member->list);
			kfree(member);
			br_eero_reset_segment_cache(eero_br);
		}
		if (list_empty(&seg->member_list)) {
			segment_log_info(
				"no members left, removing segment: %pM\n",
				seg->designated_root);
			list_del(&seg->list);
			kmem_cache_free(br_eero_segment_cache, seg);
		}
	}
	write_unlock_bh(&seg_lock);
}

/* called under write_lock */
static void br_eero_del_member_list(struct br_eero_segment_entry *seg)
{
	struct br_eero_seg_member *member, *tmp_member;

	if (!seg)
		return;

	list_for_each_entry_safe (member, tmp_member, &seg->member_list, list) {
		list_del(&member->list);
		kfree(member);
	}
}

#ifdef NON_EERO_INSIDE_CODE
/* This API is not used and it is to avoid a compile error */
static void br_eero_del_segment_list(struct br_eero_br *eero_br,
				     u8 *designated_root)
{
	struct br_eero_segment_entry *seg;

	write_lock(&seg_lock);
	seg = br_eero_find_segment_list(eero_br, designated_root);
	if (seg) {
		br_eero_del_member_list(seg);
		list_del(&seg->list);
		kmem_cache_free(br_eero_segment_cache, seg);
	}
	write_unlock(&seg_lock);
}
#endif

/* called under read_lock */
void br_eero_print_member_list(struct br_eero_segment_entry *seg)
{
	struct br_eero_seg_member *member;

	if (!seg)
		return;
	printk(KERN_ERR "seg designated_root:%pM\n", seg->designated_root);
	list_for_each_entry (member, &seg->member_list, list) {
		printk(KERN_ERR "member addr:%pM mesh_conn_value:%d\n",
		       member->addr, member->mesh_conn_value);
	}
}

/*TODO Write test functions */
static struct br_eero_segment_entry_ext *
br_eero_find_segment_list_ext(struct br_eero_br *eero_br, u8 *designated_root)
{
	struct br_eero_segment_entry_ext *seg_ext;

	if (!eero_br->segment && !(&eero_br->segment->seg_list_ext))
		return NULL;
	list_for_each_entry (seg_ext, &eero_br->segment->seg_list_ext, list) {
		if (ether_addr_equal(seg_ext->designated_root, designated_root))
			return seg_ext;
	}
	return NULL;
}

/* called under write_lock */
struct br_eero_segment_entry_ext *
br_eero_add_segment_list_ext(struct br_eero_br *eero_br,
			     struct br_eero_segment_entry *seg)
{
	struct br_eero_segment_entry_ext *seg_ext;

	seg_ext = kmalloc(sizeof(*seg_ext), GFP_ATOMIC);
	if (!seg_ext)
		return NULL;
	/* Init the member list */
	ether_addr_copy(seg_ext->designated_root, seg->designated_root);
	ether_addr_copy(seg_ext->forwarder, seg->forwarder);
	list_add(&seg_ext->list, &eero_br->segment->seg_list_ext);
	return seg_ext;
}

/* called under write_lock */
struct br_eero_segment_entry_ext *
br_eero_update_segment_list_ext(struct br_eero_segment_entry_ext *seg_ext,
				struct br_eero_segment_entry *seg)
{
	if (!seg_ext)
		return NULL;
	/* Init the member list */
	ether_addr_copy(seg_ext->designated_root, seg->designated_root);
	ether_addr_copy(seg_ext->forwarder, seg->forwarder);
	return seg_ext;
}

/* called under write_lock */
void br_eero_del_segment_list_ext(struct br_eero_segment_entry_ext *seg_ext)
{
	if (!seg_ext)
		return;
	/* Init the member list */
	list_del(&seg_ext->list);
	kfree(seg_ext);
}

/* Frame operation */
static int br_eero_pull_conn_value_internal(struct net_device *dev)
{
	return 0;
}

int (*br_eero_pull_conn_dummy)(struct net_device *dev) =
	&br_eero_pull_conn_value_internal;
EXPORT_SYMBOL(br_eero_pull_conn_dummy);
int (*br_eero_pull_conn_value)(struct net_device *dev) =
	&br_eero_pull_conn_value_internal;
EXPORT_SYMBOL(br_eero_pull_conn_value);

static int br_eero_pull_stamp_metric(struct br_eero_br *eero_br,
				     struct net_device *dev)
{
	if (eero_br->fudged_stamp_metric)
		return eero_br->fudged_stamp_metric;

	return br_eero_pull_conn_value(dev);
}

static void br_eero_create_seg_buf(struct br_eero_br *eero_br,
				   struct net_bridge_port *port,
				   struct br_eero_seg_pkt *data)
{
	struct net_bridge_port *p = port;
	u8 seg_id_addr[ETH_ALEN];

	data->identifier = EERO_ETH_DETECTION;
	br_eero_get_unique_seg_id(eero_br, seg_id_addr);
	read_lock(&seg_lock);
	ether_addr_copy(data->designated_root, seg_id_addr);
	ether_addr_copy(data->forwarder, p->dev->dev_addr);
	data->mesh_conn_value = br_eero_pull_stamp_metric(eero_br, p->dev);
	data->mbss = 0;
	read_unlock(&seg_lock);
	segment_log_debug("seg frame: root:%pM fwd: %pM metric: %d\n",
			  data->designated_root, data->forwarder,
			  data->mesh_conn_value);
}

static void br_eero_update_self_designated_root(struct br_eero_br *eero_br,
						struct net_bridge_port *p)
{
	struct br_eero_segment_entry *seg;
	struct br_eero_eth_segment *segment;
	struct net_bridge *br;
	u8 seg_id_addr[ETH_ALEN];

	if (!eero_br->segment || !eero_br->segment->br)
		return;
	segment = eero_br->segment;
	br = eero_br->segment->br;
	br_eero_get_unique_seg_id(eero_br, seg_id_addr);
	write_lock(&seg_lock);
	seg = br_eero_find_segment_list(eero_br, seg_id_addr);
	if (seg) {
		if (!p)
			br_eero_update_member(eero_br, seg, br->dev->dev_addr,
					      0, 0);
		else
			br_eero_update_member(
				eero_br, seg, p->dev->dev_addr,
				br_eero_pull_stamp_metric(eero_br, p->dev), 0);
	} else {
		if (!p)
			br_eero_add_segment_list(eero_br, seg_id_addr,
						 br->dev->dev_addr, 0, 0);
		else
			br_eero_add_segment_list(
				eero_br, seg_id_addr, p->dev->dev_addr,
				br_eero_pull_stamp_metric(eero_br, p->dev), 0);
	}
	br_eero_choose_designated_root(eero_br, seg_id_addr, true);
	write_unlock(&seg_lock);
}

static void br_eero_send_seg_frame(struct br_eero_br *eero_br)
{
	struct br_eero_seg_pkt data;
	struct net_bridge_port *p;

	segment_log_debug("sending seg frame\n");
	p = br_eero_get_mesh_port(eero_br->br);
	if (p) {
		br_eero_create_seg_buf(eero_br, p, &data);
		br_eero_send_pkt_mesh(eero_br->br, (u8 *)&data,
				      sizeof(struct br_eero_seg_pkt));
	}
	br_eero_update_self_designated_root(eero_br, p);
}

/* Interface and port operations */
/* Gets in designated_root address and checks if this bridge is the flooder for
 * this designated_root.
 */
bool br_eero_is_self_designated_node(struct net_bridge *br, u8 *designated_root)
{
	struct br_eero_segment_entry *seg_ext;
	struct net_bridge_port *p;

	if (!br->eero_ext.eero_br->segment || !(&br->eero_ext.eero_br->segment->seg_list_ext))
		return false;
	p = br_eero_get_mesh_port(br);
	if (!p)
		return false;
	read_lock(&seg_lock_ext);
	list_for_each_entry (seg_ext, &br->eero_ext.eero_br->segment->seg_list_ext,
			     list) {
		if (ether_addr_equal(seg_ext->designated_root,
				     designated_root) &&
		    ether_addr_equal(seg_ext->forwarder, p->dev->dev_addr)) {
			read_unlock(&seg_lock_ext);
			return true;
		}
	}
	read_unlock(&seg_lock_ext);
	return false;
}
EXPORT_SYMBOL(br_eero_is_self_designated_node);

/* Handles different eero messages */
int br_eero_handle_seg_pkt(struct br_eero_br *eero_br,
			   const struct sk_buff *skb)
{
	struct br_eero_seg_pkt *seg_pkt;
	struct br_eero_segment_entry *seg;

	segment_log_debug("handling seg frame\n");
	seg_pkt = (struct br_eero_seg_pkt *)skb->data;
	write_lock(&seg_lock);
	seg = br_eero_find_segment_list(eero_br, seg_pkt->designated_root);
	if (seg)
		br_eero_update_member(eero_br, seg, seg_pkt->forwarder,
				      seg_pkt->mesh_conn_value, 0);
	else
		br_eero_add_segment_list(eero_br, seg_pkt->designated_root,
					 seg_pkt->forwarder,
					 seg_pkt->mesh_conn_value, 0);
	br_eero_choose_designated_root(eero_br, seg_pkt->designated_root,
				       false);
	write_unlock(&seg_lock);
	return BR_EERO_NO_FW_PKT;
}

void update_ext_seg_list(struct br_eero_br *eero_br,
			 struct br_eero_segment_entry *seg)
{
	struct br_eero_segment_entry_ext *seg_ext;

	read_lock(&seg_lock_ext);
	seg_ext = br_eero_find_segment_list_ext(eero_br, seg->designated_root);
	read_unlock(&seg_lock_ext);
	if (seg_ext) {
		if (!ether_addr_equal(seg->forwarder, seg_ext->forwarder)) {
			write_lock(&seg_lock_ext);
			br_eero_update_segment_list_ext(seg_ext, seg);
			write_unlock(&seg_lock_ext);
		}
		return;
	}
	write_lock(&seg_lock_ext);
	br_eero_add_segment_list_ext(eero_br, seg);
	write_unlock(&seg_lock_ext);
}

inline int eero_janky_ether_compare(u8 *a, u8 *b)
{
	u32 aa = (a[5] << 24) | (a[4] << 16) | (a[3] << 8) | a[2];
	u32 bb = (b[5] << 24) | (b[4] << 16) | (b[3] << 8) | b[2];

	return aa - bb;
}

/* called under write_lock */
void br_eero_choose_designated_root(struct br_eero_br *eero_br,
				    u8 *designated_root, bool retire)
{
	struct br_eero_segment_entry *seg;
	struct br_eero_seg_member *member;
	int metric_difference = 0;
	struct br_eero_seg_member *best_member = NULL;
	__be32 highest_metric = 0;

	seg = br_eero_find_segment_list(eero_br, designated_root);
	if (!seg)
		return;

	list_for_each_entry (member, &seg->member_list, list) {
		if (member->mesh_conn_value >= highest_metric &&
		    br_eero_is_active(eero_br, member)) {
			if (best_member && best_member->mesh_conn_value ==
						   member->mesh_conn_value) {
				/* The members may not be in the same order
				 * in other nodes' lists, so we have to have a
				 * tiebreaker that's universally decided the
				 * same way.
				 */
				if (eero_janky_ether_compare(best_member->addr,
							     member->addr) > 0)
					continue;
			}
			best_member = member;
			highest_metric = member->mesh_conn_value;
		} else if (retire && !br_eero_is_active(eero_br, member)) {
			read_lock(&seg_lock_same_seg);
			if (ether_addr_equal(eero_br->segment->same_seg_cache,
					     member->addr)) {
				read_unlock(&seg_lock_same_seg);
				br_eero_reset_segment_cache(eero_br);
			} else
				read_unlock(&seg_lock_same_seg);
		}
	}
	if (!best_member) {
		/*TODO Do we need to delete the entry when this happens */
		return;
	}

	metric_difference =
		abs(best_member->mesh_conn_value - seg->mesh_conn_value);

	if (!is_zero_ether_addr(seg->forwarder) &&
	    !(seg->mesh_conn_value < EERO_CONN_METRIC_GUARDBAND) &&
	    metric_difference < EERO_CONN_METRIC_GUARDBAND)
		goto dontswitch;

	ether_addr_copy(seg->forwarder, best_member->addr);
	seg->mesh_conn_value = best_member->mesh_conn_value;
	seg->mbss = best_member->mbss;

dontswitch:
	seg->update_time = best_member->update_time;
	update_ext_seg_list(eero_br, seg);
	segment_log_debug("updated forwarder: %pM seg: %pM\n", seg->forwarder,
			  designated_root);
}

/* Timer for sending seg messages */
static void br_eero_seg_timer(unsigned long arg)
{
	struct br_eero_br *eero_br = (struct br_eero_br *)arg;

	if (eero_br->seg_enabled == BR_EERO_SEG_ON)
		br_eero_send_seg_frame(eero_br);

	/* Update the timer based on sysctl value */
	eero_br->segment->seg_announce_duration =
		br_eero_get_seg_ann_interval() * HZ;
	eero_br->segment->member_timeout_duration =
		EERO_SEG_MEMBER_VALID_MULTIPLIER *
		eero_br->segment->seg_announce_duration;

	mod_timer(&eero_br->segment->seg_timer,
		  round_jiffies(jiffies +
				eero_br->segment->seg_announce_duration));
}

static void br_eero_delayed_worker(struct work_struct *d_work)
{
	struct br_eero_eth_segment *segment = container_of(
		to_delayed_work(d_work), struct br_eero_eth_segment, d_work);
	struct br_eero_br *eero_br = segment->br->eero_ext.eero_br;

	segment_log_debug("segment worker is called\n");
	br_eero_expire_members(eero_br);
	queue_delayed_work(segment->wq, &segment->d_work,
			   BR_EERO_WORK_Q_INTERVAL);
}

/* Segment init and deinit code */
static struct br_eero_eth_segment *
br_eero_init_segment_list(struct br_eero_br *eero_br)
{
	struct br_eero_eth_segment *segment;

	br_eero_segment_cache =
		kmem_cache_create("br_eero_segment_cache",
				  sizeof(struct br_eero_segment_entry), 0,
				  SLAB_HWCACHE_ALIGN, NULL);
	if (!br_eero_segment_cache)
		return NULL;

	segment = kzalloc(sizeof(*segment), GFP_ATOMIC);

	/* Init the structures */
	INIT_LIST_HEAD(&segment->seg_list);
	INIT_LIST_HEAD(&segment->seg_list_ext);
	segment->br = eero_br->br;
	segment->seg_announce_duration = EERO_SEG_ANNOUNCEMENTS * HZ;
	segment->member_timeout_duration = EERO_SEG_MEMBER_VALID_MULTIPLIER *
					   segment->seg_announce_duration;

	if (segment->wq) {
		segment_log_error("work queue already exists\n");
		return NULL;
	}

	/* Use a new workqueue, not the kernel global one, so that this can take
	 * over some more loads later on, such as sending packets through this
	 * workqueue, instead of timer context, which is the current modtimer
	 * design.
	 */
	segment->wq = create_singlethread_workqueue(BRIDGE_DIR_NAME);
	INIT_DELAYED_WORK(&segment->d_work, br_eero_delayed_worker);
	queue_delayed_work(segment->wq, &segment->d_work,
			   BR_EERO_WORK_Q_INTERVAL);

	setup_timer(&segment->seg_timer, br_eero_seg_timer,
		    (unsigned long)eero_br);
	mod_timer(&segment->seg_timer,
		  round_jiffies(jiffies + segment->seg_announce_duration));
	br_eero_debugfs_init_stamp(eero_br);

	segment_log_info("init seg completed\n");
	return segment;
}

static void br_eero_fini_segment_list(struct br_eero_br *eero_br)
{
	struct br_eero_segment_entry *seg, *tmp_seg;

	if (!eero_br->segment)
		return;

	br_eero_debugfs_deinit_stamp(eero_br);

	if (&eero_br->segment->seg_timer)
		del_timer(&eero_br->segment->seg_timer);

	if (eero_br->segment->wq)
		destroy_workqueue(eero_br->segment->wq);

	write_lock(&seg_lock);
	list_for_each_entry_safe (seg, tmp_seg, &eero_br->segment->seg_list,
				  list) {
		br_eero_del_member_list(seg);
		list_del(&seg->list);
		kmem_cache_free(br_eero_segment_cache, seg);
	}
	kmem_cache_destroy(br_eero_segment_cache);
	kfree(eero_br->segment);
	eero_br->segment = NULL;
	write_unlock(&seg_lock);
	segment_log_info("fini seg completed\n");
}

/* Exported functions */
bool br_same_ethernet_domain(struct net_device *dev, u8 *search_addr)
{
	struct net_device *br_dev;
	struct net_bridge *br;
	struct br_eero_segment_entry *seg;
	bool ret = false;
	struct br_eero_br *eero_br;
	u8 seg_id_addr[ETH_ALEN];

	rcu_read_lock();
	br_dev = netdev_master_upper_dev_get_rcu(dev);
	if (!br_dev) {
		rcu_read_unlock();
		return false;
	}

	br = netdev_priv(br_dev);
	if (!br) {
		rcu_read_unlock();
		return false;
	}
	rcu_read_unlock();

	eero_br = br->eero_ext.eero_br;
	if (!eero_br || !eero_br->seg_enabled)
		return false;
	read_lock_bh(&seg_lock_same_seg);
	if (ether_addr_equal(eero_br->segment->same_seg_cache, search_addr)) {
		read_unlock_bh(&seg_lock_same_seg);
		return true;
	}
	read_unlock_bh(&seg_lock_same_seg);
	br_eero_get_unique_seg_id(eero_br, seg_id_addr);
	read_lock_bh(&seg_lock);
	seg = br_eero_find_segment_list(eero_br, seg_id_addr);

	if (!seg)
		goto get_out;

	if (br_eero_find_member_in_segment(eero_br, seg, search_addr, true)) {
		write_lock_bh(&seg_lock_same_seg);
		ether_addr_copy(eero_br->segment->same_seg_cache, search_addr);
		write_unlock_bh(&seg_lock_same_seg);
		ret = true;
	}
get_out:
	read_unlock_bh(&seg_lock);
	return ret;
}
EXPORT_SYMBOL(br_same_ethernet_domain);

/* Debugfs functions */
/* Accessed from the process context, so use _bh for locking */
int br_eero_print_segment_text(struct seq_file *seq, void *offset)
{
	struct br_eero_segment_entry *seg;
	struct br_eero_seg_member *member;
	struct br_eero_br *eero_br;
	struct br_eero_eth_segment *segment;

	eero_br = (struct br_eero_br *)seq->private;

	if (!eero_br->segment || !(&eero_br->segment->seg_list)) {
		seq_puts(seq, "No segment or segment_list\n");
		return 0;
	}
	segment = eero_br->segment;
	read_lock_bh(&seg_lock);
	list_for_each_entry (seg, &segment->seg_list, list) {
		seq_printf(
			seq,
			"segment segment_id=%pM forwarder=%pM metric=%d MBSS=%d update_time=%lu\n",
			seg->designated_root, seg->forwarder,
			seg->mesh_conn_value, seg->mbss, seg->update_time);
		list_for_each_entry (member, &seg->member_list, list) {
			seq_printf(
				seq,
				"\tmember member_addr=%pM metric=%d MBSS=%d update_time=%lu active=%s\n",
				member->addr, member->mesh_conn_value,
				member->mbss, member->update_time,
				br_eero_is_active(eero_br, member) ? "true" :
								     "false");
		}
	}
	read_unlock_bh(&seg_lock);
	return 0;
}

/* Accessed from the process context, so use _bh for locking */
int br_eero_print_segment_ext_text(struct seq_file *seq, void *offset)
{
	struct br_eero_segment_entry_ext *seg_ext;
	struct br_eero_br *eero_br;

	eero_br = (struct br_eero_br *)seq->private;

	if (!eero_br->segment || !(&eero_br->segment->seg_list_ext)) {
		seq_puts(seq, "No segment or segment_list_ext\n");
		return 0;
	}
	read_lock_bh(&seg_lock_ext);
	list_for_each_entry (seg_ext, &eero_br->segment->seg_list_ext, list) {
		seq_printf(seq, "segment segment_id=%pM forwarder=%pM\n",
			   seg_ext->designated_root, seg_ext->forwarder);
	}
	read_unlock_bh(&seg_lock_ext);
	return 0;
}

/* Accessed from the process context, so use _bh for locking */
int br_eero_print_segment_cache(struct seq_file *seq, void *offset)
{
	struct br_eero_br *eero_br;

	eero_br = (struct br_eero_br *)seq->private;

	if (!eero_br->segment) {
		seq_puts(seq, "No segment\n");
		return 0;
	}
	read_lock_bh(&seg_lock_same_seg);
	seq_printf(seq, "same_seg_cache=%pM\n",
		   eero_br->segment->same_seg_cache);
	read_unlock_bh(&seg_lock_same_seg);
	return 0;
}

void br_eero_stamp_set_enabled(struct br_eero_br *eero_br, unsigned long val)
{
	ASSERT_RTNL();

	if (val) {
		if (eero_br->seg_enabled == BR_EERO_SEG_OFF) {
			eero_br->segment = br_eero_init_segment_list(eero_br);
			br_eero_init_seg_id(eero_br);
			eero_br->seg_enabled = BR_EERO_SEG_ON;
		}
	} else {
		if (eero_br->seg_enabled == BR_EERO_SEG_ON) {
			br_eero_fini_seg_id(eero_br);
			br_eero_fini_segment_list(eero_br);
			eero_br->seg_enabled = BR_EERO_SEG_OFF;
		}
	}
}
EXPORT_SYMBOL(br_eero_stamp_set_enabled);
