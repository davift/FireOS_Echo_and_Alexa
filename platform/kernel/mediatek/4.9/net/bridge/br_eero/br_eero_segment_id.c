/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include "br_eero.h"

static void br_eero_set_unique_seg_id(struct br_eero_br *, u8 *);

static void br_eero_create_seg_id_buf(struct br_eero_br *eero_br,
				      struct br_eero_seg_id_pkt *data)
{
	u8 seg_id_addr[ETH_ALEN];

	br_eero_get_unique_seg_id(eero_br, seg_id_addr);
	data->identifier = EERO_SEG_IDENTIFIER;
	ether_addr_copy(data->unique_id, seg_id_addr);
}

static void br_eero_send_seg_id_frame(struct br_eero_br *eero_br,
				      struct net_bridge_port *in_p)
{
	struct br_eero_seg_id_pkt data;

	br_eero_create_seg_id_buf(eero_br, &data);
	br_eero_send_pkt_eth(eero_br->segment->br, (u8 *)&data,
			     sizeof(struct br_eero_seg_id_pkt), in_p);
}

static void br_eero_update_seg_id(struct br_eero_br *eero_br)
{
	unsigned long update_time;
	u8 seg_id_addr[ETH_ALEN];
	u8 *dev_addr = eero_br->segment->br->dev->dev_addr;

	update_time = br_eero_get_unique_seg_id(eero_br, seg_id_addr);

	if (ether_addr_equal(seg_id_addr, dev_addr))
		segment_id_log_debug("update own seg id: %pM\n", seg_id_addr);
	else if (time_after(jiffies,
			    update_time +
				    eero_br->segment->seg_id_timeout_duration))
		segment_id_log_info("reset the seg id: %pM\n", dev_addr);
	else
		return;

	br_eero_set_unique_seg_id(eero_br, dev_addr);
	br_eero_send_seg_id_frame(eero_br, NULL);
}

int br_eero_handle_seg_id_pkt(struct br_eero_br *eero_br,
			      const struct sk_buff *skb)
{
	struct br_eero_seg_id_pkt *seg_id_pkt =
		(struct br_eero_seg_id_pkt *)skb->data;
	struct net_bridge_port *in_p = br_port_get_rcu(skb->dev);
	struct net_bridge *br = eero_br->segment->br;
	u8 seg_id_addr[ETH_ALEN];

	segment_id_log_debug("received seg id frame\n");
	br_eero_get_unique_seg_id(eero_br, seg_id_addr);

	if (memcmp(seg_id_addr, seg_id_pkt->unique_id, ETH_ALEN) < 0)
		segment_id_log_info("set a new seg id: %pM\n",
				    seg_id_pkt->unique_id);
	else if (ether_addr_equal(seg_id_addr, seg_id_pkt->unique_id) &&
		 !ether_addr_equal(seg_id_pkt->unique_id, br->dev->dev_addr))
		segment_id_log_debug("update seg id: %pM\n", seg_id_addr);
	else
		return BR_EERO_NO_FW_PKT;

	br_eero_set_unique_seg_id(eero_br, seg_id_pkt->unique_id);
	br_eero_send_seg_id_frame(eero_br, in_p);
	return BR_EERO_NO_FW_PKT;
}

/* Timer for sending seg messages */
static void br_eero_seg_id_timer(unsigned long arg)
{
	struct br_eero_br *eero_br = (struct br_eero_br *)arg;

	if (eero_br->seg_enabled == BR_EERO_SEG_ON)
		br_eero_update_seg_id(eero_br);

	/* Update the timer based on sysctl value of seg announce - not seg id*/
	eero_br->segment->seg_id_announce_duration =
		br_eero_get_seg_id_ann_interval() * HZ;
	eero_br->segment->seg_id_timeout_duration =
		EERO_SEG_MEMBER_VALID_MULTIPLIER *
		eero_br->segment->seg_id_announce_duration;

	mod_timer(&eero_br->segment->seg_id_timer,
		  round_jiffies(jiffies +
				eero_br->segment->seg_id_announce_duration));
}

void br_eero_init_seg_id(struct br_eero_br *eero_br)
{
	rwlock_init(&eero_br->segment->seg_id_lock);
	br_eero_set_unique_seg_id(eero_br, eero_br->segment->br->dev->dev_addr);
	eero_br->segment->seg_id_announce_duration =
		br_eero_get_seg_id_ann_interval() * HZ;
	eero_br->segment->seg_id_timeout_duration =
		EERO_SEG_MEMBER_VALID_MULTIPLIER *
		eero_br->segment->seg_id_announce_duration;

	setup_timer(&eero_br->segment->seg_id_timer, br_eero_seg_id_timer,
		    (unsigned long)eero_br);
	mod_timer(&eero_br->segment->seg_id_timer,
		  round_jiffies(jiffies +
				eero_br->segment->seg_id_announce_duration));
	segment_id_log_info("init seg id completed: %pM\n",
			    eero_br->segment->br->dev->dev_addr);
}

void br_eero_fini_seg_id(struct br_eero_br *eero_br)
{
	if (!eero_br->segment)
		return;

	if (&eero_br->segment->seg_id_timer)
		del_timer(&eero_br->segment->seg_id_timer);

	segment_id_log_info("fini seg id completed\n");
}

/* Accessed from the process context, so use _bh for locking */
int br_eero_print_segment_id_text(struct seq_file *seq, void *offset)
{
	struct br_eero_br *eero_br;
	u8 seg_id_addr[ETH_ALEN];

	eero_br = (struct br_eero_br *)seq->private;

	if (!eero_br || !eero_br->segment) {
		seq_puts(seq, "No segment or segment_list_ext\n");
		return 0;
	}
	br_eero_get_unique_seg_id(eero_br, seg_id_addr);
	seq_printf(seq, "segment_id=%pM\n", seg_id_addr);
	return 0;
}

static void br_eero_set_unique_seg_id(struct br_eero_br *eero_br, u8 *addr)
{
	if (!eero_br)
		return;
	write_lock_bh(&eero_br->segment->seg_id_lock);
	ether_addr_copy(eero_br->segment->seg_id, addr);
	eero_br->segment->seg_id_update_time = jiffies;
	write_unlock_bh(&eero_br->segment->seg_id_lock);
}

unsigned long br_eero_get_unique_seg_id(struct br_eero_br *eero_br, u8 *addr)
{
	unsigned long update_time;

	if (!eero_br)
		return 0;
	read_lock_bh(&eero_br->segment->seg_id_lock);
	ether_addr_copy(addr, eero_br->segment->seg_id);
	update_time = eero_br->segment->seg_id_update_time;
	read_unlock_bh(&eero_br->segment->seg_id_lock);
	return update_time;
}
EXPORT_SYMBOL(br_eero_get_unique_seg_id);
