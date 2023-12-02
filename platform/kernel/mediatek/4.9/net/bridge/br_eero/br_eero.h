/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifdef CONFIG_BIFROST_MESH
#ifndef _BR_EERO_H
#define _BR_EERO_H

#include <linux/netdevice.h>
#include <linux/eero_bridge.h>
#include "../br_private.h"

#define EERO_SEG_ANNOUNCEMENTS 15 /* In seconds */
#define EERO_SEG_ID_ANNOUNCEMENTS 15 /* In seconds */
#define EERO_SEG_MEMBER_VALID_MULTIPLIER 3
#define EERO_CONN_METRIC_GUARDBAND 12 /* in megabits per second */

#define BR_EERO_NO_FW_PKT 0
#define BR_EERO_FWD_PKT 1

#define BRIDGE_DIR_NAME "eero_bridge"
#define DEBUGFS_EERO_BR_DIR BRIDGE_DIR_NAME
#define SYSFS_EERO_BR_ATTR BRIDGE_DIR_NAME
#define SYSFS_EERO_BRIDGE_SUBDIR BRIDGE_DIR_NAME

/* Netlink/Genl defs */
#define BR_EERO_GENL_NAME BRIDGE_DIR_NAME
#define BR_EERO_GENL_VERSION 1
#define BR_EERO_GENL_ADDR_MAX_SIZE (ETH_ALEN + 1)

#define BR_EERO_DEFAULT_LOG_LEVEL 0x02

#define BR_EERO_WORK_Q_INTERVAL (60 * HZ)

#define SKB_MIN_SIZE 64

/* This is a basic structure for lists */
struct mac_addr_list_entry {
	struct list_head list;
	u8 addr[ETH_ALEN];
};

/* Enum used for signaling what list to add the data to */
enum list_name {
	EERO_OUI_LIST,
	NEIGH_LIST,
	SONOS_OUI_LIST,
};

/* This is a structure for keeping the information for roam feature.
 *
 * br: br structure that this segment is running on (e.g. br-lan)
 * eero_br: pointer to eero bridge extension module
 */
struct br_eero_roam {
	struct net_bridge *br;
	struct br_eero_br *eero_br;
};

/* This structure is the main structure for the segment operations.
 * This is the main STAMP data structure and all other structures are
 * part of this or can be iterated through this structure.
 *
 * seg_list: List of segments seen by this node
 * seg_list_ext: List of segments seen by this node but additionally this
 * structure is used for fast lookups since it is protected by another lock.
 * br: br structure that this segment is running on (e.g. br-lan)
 * seg_announce_duration: How often we sent the segment announcements.
 * seg_timer: The timer object that runs the seg announcements.
 * member_timeout_duration: The timeout duration to declare a member is no
 * longer visible.
 * seg_id: the unique id for the segment this node belongs to.
 * seg_id_update_time: Tells when the seg_id is updated
 * seg_id_timeout_duration: Duration which indicates when to expire a seg_id
 * seg_id_announce_duration: How often we sent the segment id announcements.
 * seg_id_timer: The timer object that runs the seg id announcements.
 * same_seg_cache: Keeps the last forwarded packet onto the eth.
 * wq: workqueue that is used for the segment operations.
 * d_work: the work struct that has the delayed work.
 */
struct br_eero_eth_segment {
	struct list_head seg_list;
	struct list_head seg_list_ext;

	struct net_bridge *br;

	unsigned long seg_announce_duration;
	struct timer_list seg_timer;

	unsigned long member_timeout_duration;

	/* Below are used for converging to a common unique id for the
	 * segment
	 * seg_id_lock: lock to protect seg_id and seg_id_update_time
	 * seg_id: unique id number for this eth segment
	 * seg_id_update_time: the time this seg_id is last updated
	 * seg_id_timeout_duration: in sec, when this id will expire if there
	 * is no update for it.
	 */
	rwlock_t seg_id_lock;
	u8 seg_id[ETH_ALEN];
	unsigned long seg_id_update_time;
	unsigned long seg_id_timeout_duration;

	unsigned long seg_id_announce_duration;
	struct timer_list seg_id_timer;

	/* For the same segment check, keep the last addr in cache */
	u8 same_seg_cache[ETH_ALEN];

	struct workqueue_struct *wq;
	struct delayed_work d_work;
};

/* This structure is used for uniquely identifying individual
 * ethernet segments.
 *
 * designated_port: The ethernet segment identifier calculated
 * by the STP.
 * forwarder: MAC address of a ethernet segment forwarder
 * mesh_conn_value: Highest mesh_conn_value of a node.
 * mbss: Hash of an mbss segment that the wireless message
 * received from.
 */
struct br_eero_segment_entry {
	struct list_head list;

	u8 designated_root[ETH_ALEN];
	u8 forwarder[ETH_ALEN];
	u32 mesh_conn_value;
	u32 mbss;
	unsigned long update_time;

	struct list_head member_list;
};

/* This structure is used for uniquely identifying individual
 * ethernet segments that are visible to external queries. For
 * example when mac80211 asks if a mesh proxy point is part of
 * the same segment or not. The reason we have this structure
 * other than br_eero_segment_entry is to separate updating
 * structures from tha actual lookups.
 *
 * designated_port: The ethernet segment identifier calculated
 * by the STP.
 * forwarder: MAC address of a ethernet segment forwarder
 */
struct br_eero_segment_entry_ext {
	struct list_head list;

	u8 designated_root[ETH_ALEN];
	u8 forwarder[ETH_ALEN];
};

struct br_eero_seg_member {
	struct list_head list;

	u8 addr[ETH_ALEN];
	u32 mesh_conn_value;
	u32 mbss;
	unsigned long update_time;
};

/* This is a structure for keeping the information for eero bridge.
 *
 * br: br structure that this segment is running on (e.g. br-lan)
 * fdb_handler_cb: the fdb callback that eero bridge makes
 * seg_enabled: enum that keeps track of whether stamp/segment code is enabled
 * or not.
 * eero_roam_enabled: enum that keeps track of whether roam code is enabled or
 * not.
 * kobj: the kobject that will form the basis for the sysfs for eero bridge.
 * log_level: log level for eero bridge code.
 */
struct br_eero_br {
	struct net_bridge *br;
	int (*fdb_handler_cb)(struct net_bridge *, struct net_bridge_port *,
			      const unsigned char *, u16, bool);

	enum { BR_EERO_SEG_OFF, /* no eth segmentation */
	       BR_EERO_SEG_ON, /* eth segmentation on */
	} seg_enabled;

	enum { BR_EERO_ROAM_OFF, /* eero roam feature off */
	       BR_EERO_ROAM_ON, /* eero roam feature on */
	} eero_roam_enabled;

	struct kobject *kobj;
	u8 log_level;
	struct br_eero_eth_segment *segment;
	struct br_eero_roam *roam;
#if IS_ENABLED(CONFIG_EERO_BR_LOG_FILE)
	struct br_eero_file_log *file_log;
#endif /* IS_ENABLED(CONFIG_EERO_BR_LOG_FILE) */
	struct dentry *debugfs_root;
	struct dentry *debugfs_br;
	struct dentry *debugfs_stamp;

	u32 fudged_stamp_metric;
};

/* General structure for this module.
 *
 * br_list: List of bridges that have the eero bridge enabled.
 */
struct br_eero_main {
	struct list_head br_list;

	struct list_head eero_oui_list;
	struct list_head sonos_oui_list;
	struct list_head neigh_list;
};

/* Segment list operations */
void br_eero_stamp_set_enabled(struct br_eero_br *, unsigned long);
void br_eero_print_member_list(struct br_eero_segment_entry *);
u8 *br_eero_print_all_segment_list(struct br_eero_segment_entry *);

/* Frame operation */
bool br_eero_is_eero_type(const struct sk_buff *);
static inline u8 br_eero_return_eero_type(const struct sk_buff *skb)
{
	struct br_eero_pkt *eero_pkt;

	if (br_eero_is_eero_type(skb)) {
		eero_pkt = (struct br_eero_pkt *)skb->data;
		return eero_pkt->identifier;
	}
	return EERO_ERR;
}

/* Interface and port operations */
bool br_eero_is_mesh_interface(const struct net_bridge_port *);
bool br_eero_is_mesh_interface_dev(const struct net_device *);
bool br_eero_is_eth_interface(const struct net_bridge_port *);
bool br_eero_is_eth_interface_dev(const struct net_device *);
struct net_bridge_port *br_eero_get_mesh_port(struct net_bridge *);

/* Handles different eero messages */
int br_eero_handle_seg_pkt(struct br_eero_br *, const struct sk_buff *);
int br_eero_handle_seg_id_pkt(struct br_eero_br *, const struct sk_buff *);
void br_eero_choose_designated_root(struct br_eero_br *, u8 *, bool);

void br_eero_send_pkt_mesh(struct net_bridge *, const u8 *, int);
void br_eero_send_pkt_eth(struct net_bridge *, const u8 *, int,
			  struct net_bridge_port *);
void br_eero_send_pkt_all(struct net_bridge *, const u8 *, int);
void br_eero_send_pkt_directly(struct net_bridge *, const u8 *, int);

/* Test */
void br_eero_test_seg_op(struct net_bridge *br);
void br_eero_test_seg_op_multi(struct net_bridge *br);

#if IS_ENABLED(CONFIG_EERO_BR)
/* Exported functions */
unsigned long br_eero_get_unique_seg_id(struct br_eero_br *, u8 *);
int br_eero_handle_skb(const struct sk_buff *, struct br_eero_br *);
bool br_eero_is_sonos(struct br_eero_br *, u8 *);
bool br_eero_is_self_designated_node(struct net_bridge *, u8 *);
bool br_same_ethernet_domain(struct net_device *, u8 *);
int br_eero_all_stp_through_stamp(void);
bool br_eero_is_fwd_eligible(struct net_bridge *, struct sk_buff *,
			     struct net_bridge_port *);
struct br_eero_br *br_eero_add_br(int (*cb)(struct net_bridge *,
					    struct net_bridge_port *,
					    const unsigned char *, u16, bool),
				  struct net_device *);
int br_eero_rm_br(struct net_device *);
bool br_eero_should_rx(struct net_bridge *br, struct sk_buff *skb);
bool br_eero_is_ap_interface(const struct net_bridge_port *);
#else
static inline unsigned long
br_eero_get_unique_seg_id(struct br_eero_br *eero_br, u8 *addr)
{
	return 0;
};

static inline int br_eero_handle_skb(const struct sk_buff *skb,
				     struct br_eero_br *eero_br)
{
	return BR_EERO_NO_FW_PKT;
};

static inline bool br_eero_is_sonos(struct br_eero_br *eero_br, u8 *addr)
{
	return false;
};

static inline bool br_eero_is_self_designated_node(struct net_bridge *br,
						   u8 *designated_root)
{
	return false;
};

static inline bool br_same_ethernet_domain(struct net_device *br, u8 *adr)
{
	return false;
};

static inline int br_eero_all_stp_through_stamp(void)
{
	return 0;
};

static inline struct net_bridge *br_eero_get_bridge(struct net_device *net)
{
	return NULL;
};

static inline bool br_eero_is_fwd_eligible(struct net_bridge *br,
					   struct sk_buff *skb,
					   struct net_bridge_port *p)
{
	return NET_RX_SUCCESS;
};

static inline struct br_eero_br *br_eero_add_br(
	int (*cb)(struct net_bridge *br, struct net_bridge_port *source,
		  const unsigned char *addr, u16 vid, bool added_by_user),
	struct net_device *dev)
{
	return NULL;
};

static inline int br_eero_rm_br(struct net_device *dev)
{
	return 0;
};

static inline bool br_eero_should_rx(struct net_bridge *br, struct sk_buff *skb)
{
	return true;
};

static inline bool br_eero_is_ap_interface(const struct net_bridge_port *p) {
	return false;
}
#endif /* CONFIG_EERO_BR */

/* Netlink functions */
int br_eero_genl_init(void);
void br_eero_genl_deinit(void);

/* Debugfs functions */
void br_eero_debugfs_init(struct br_eero_main *);
void br_eero_debugfs_deinit(void);
void br_eero_debugfs_init_per_eero_br(struct br_eero_br *);
void br_eero_debugfs_deinit_per_eero_br(struct br_eero_br *);
void br_eero_debugfs_init_stamp(struct br_eero_br *);
void br_eero_debugfs_deinit_stamp(struct br_eero_br *);
int br_eero_print_segment_text(struct seq_file *, void *);
int br_eero_print_segment_ext_text(struct seq_file *, void *);
int br_eero_print_segment_id_text(struct seq_file *, void *);
int br_eero_print_add_list_text(struct seq_file *, void *, struct list_head *);
int br_eero_print_segment_cache(struct seq_file *seq, void *offset);
static inline int br_eero_print_sonos_oui_text(struct seq_file *file,
					       void *offset)
{
	struct br_eero_main *brm = (struct br_eero_main *)file->private;

	return br_eero_print_add_list_text(file, offset, &brm->sonos_oui_list);
};

static inline int br_eero_print_eero_oui_text(struct seq_file *file,
					      void *offset)
{
	struct br_eero_main *brm = (struct br_eero_main *)file->private;

	return br_eero_print_add_list_text(file, offset, &brm->eero_oui_list);
};

static inline int br_eero_print_neigh_text(struct seq_file *file, void *offset)
{
	struct br_eero_main *brm = (struct br_eero_main *)file->private;

	return br_eero_print_add_list_text(file, offset, &brm->neigh_list);
};

void br_eero_debug_log(struct br_eero_br *, const char *, ...);
#if IS_ENABLED(CONFIG_EERO_BR_LOG_FILE)
int br_eero_debug_log_setup(struct dentry *, struct br_eero_br *);
void br_eero_debug_log_cleanup(struct br_eero_br *);
#else
static inline int br_eero_debug_log_setup(struct dentry *file,
					  struct br_eero_br *eero_br)
{
	return -EAGAIN;
};

static inline void br_eero_debug_log_cleanup(struct br_eero_br *eero_br)
{
	return;
};
#endif /* IS_ENABLED(CONFIG_EERO_BR_LOG_FILE) */

/* Sysctl functions */
int br_eero_sysctl_init(void);
void br_eero_sysctl_fini(void);
int br_eero_get_seg_ann_interval(void);
int br_eero_get_seg_id_ann_interval(void);

/* Sysfs functions */
int br_eero_add_sysfs(struct net_device *, struct br_eero_br *);
int br_eero_del_sysfs(struct net_device *, struct br_eero_br *);

/* Segment id functions */
void br_eero_init_seg_id(struct br_eero_br *);
void br_eero_fini_seg_id(struct br_eero_br *);

/* br_eero internal functions */
void br_eero_reset_segment_cache(struct br_eero_br *eero_br);

/* Helper functions */
/* The original version of this function is copied from 4.8 kernel code
 *
 * ether_addr_equal_masked - Compare two Ethernet addresses with a mask
 * @addr1: Pointer to a six-byte array containing the 1st Ethernet address
 * @addr2: Pointer to a six-byte array containing the 2nd Ethernet address
 * @mask: Pointer to a six-byte array containing the Ethernet address bitmask
 *
 * Compare two Ethernet addresses with a mask, returns true if for every bit
 * set in the bitmask the equivalent bits in the ethernet addresses are equal.
 * Using a mask with all bits set is a slower ether_addr_equal.
 */
static inline bool ether_addr_equal_oui(const u8 *addr1, const u8 *addr2)
{
	/* This is a routine designed to return false faster in the usual case,
	 * ie., where the two addresses do not match.  It's also somewhat tweaked
	 * to compile to simpler assembler instructions that can be pipelined
	 * more effectively on processors that do not have register renaming,
	 * like Cortex-A7.
	 */
#if defined(__LINUX_ARM_ARCH__)
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	// clang-format off
	register u32 res = 0;
	register u32 a1 = (u32) addr1;
	register u32 a2 = (u32) addr2;

	__asm__ volatile (
		" LDR  r2, [%1]"           "\n\t" /* r2 - address 1 */
		" LDR  r3, [%2]"           "\n\t" /* r3 - address 2 */
		" BIC r2, r2, #0xff000000" "\n\t" /* mask out the non-oui byte */
		" BIC r3, r3, #0xff000000" "\n\t" /* mask out the non-oui byte */
		" CMP  r2, r3"             "\n\t" /* compare the ouis */
		" MOVEQ %0, #0"            "\n\t" /* If they're the same, return 0 */
		" MVNNE %0, #0"            "\n\t" /* if they're different, return -1 */
		: "=&r"(res)       /* result registers */
		: "r"(a1), "r"(a2) /* operand registers */
		: "cc", "r2", "r3" /* clobber list */
	);
	return !res;
// clang-format on
#endif /* CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS */
#if !defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	// clang-format off
	register u32 res = 0;
	register u32 a1 = (u32) addr1;
	register u32 a2 = (u32) addr2;

	__asm__ volatile (
		" LDRH  r2, [%1]"      "\n\t" /* r2 <- address 1 */
		" LDRH  r3, [%2]"      "\n\t" /* r3 <- address 2 */
		" CMP  r2, r3"         "\n\t" /* we always need to compare this piece */
		" LDREQB r2, [%1, #2]" "\n\t" /* if they match, fetch the third byte */
		" LDREQB r3, [%2, #2]" "\n\t"
		" CMPEQ r2, r3"        "\n\t" /* ... and compare them */
		" MOVEQ %0, #0"        "\n\t" /* If they're the same, return 0 */
		" MVNNE %0, #0"        "\n\t" /* if they're different, return -1 */
		: "=&r"(res)       /* result registers */
		: "r"(a1), "r"(a2) /* operand registers */
		: "cc", "r2", "r3" /* clobber list */
	);
	// clang-format on
	return !res;
#endif
#else /* Not __LINUX_ARM_ARCH__ */
	int i;
	const u8 mask[] = {0xff, 0xff, 0xff, 0, 0, 0, };

	for (i = 0; i < ETH_ALEN; i++) {
		if ((addr1[i] ^ addr2[i]) & mask[i])
			return false;
	}

	return true;
#endif
}

bool br_eero_add_mac_addr(const u8 *, const int);

/* Roam functions */
void br_eero_roam_new_sta(u8 *sta_addr, struct net_device *);
int br_eero_roam_handle_pkt(struct br_eero_br *, const struct sk_buff *);
void br_eero_roam_set_enabled(struct br_eero_br *, unsigned long);

/* Log functions */
struct br_eero_debuginfo {
	struct attribute attr;
	const struct file_operations fops;
};

enum stamp_dbg_mask {
	EERO_LOG_ERROR = BIT(0),
	EERO_LOG_INFO = BIT(1),
	EERO_LOG_DEBUG = BIT(2),
};

static const char *const stamp_dbg_strs[] = {
	[EERO_LOG_ERROR] = "error",
	[EERO_LOG_INFO] = "info",
	[EERO_LOG_DEBUG] = "debug",
};

#define STAMP_LOG "stamp"
#define STAMP_ID_LOG "stamp_id"
#define ROAM_LOG "roam"
#define BR_EERO_LOG "br_eero"

#if IS_ENABLED(CONFIG_EERO_BR_LOG_FILE)
#define EERO_BR_LOG_BUF_LEN (8 * 1024)
/**
 * struct br_eero_debug_log - debug logging data
 * @log_buff: buffer holding the logs (ring bufer)
 * @log_start: index of next character to read
 * @log_end: index of next character to write
 * @lock: lock protecting log_buff, log_start & log_end
 * @queue_wait: log reader's wait queue
 */
struct br_eero_file_log {
	char log_buff[EERO_BR_LOG_BUF_LEN];
	unsigned long log_start;
	unsigned long log_end;
	spinlock_t lock; /* protects log_buff, log_start and log_end */
	wait_queue_head_t queue_wait;
};

#define br_eero_print(bit, eero_br, component, fmt, ...)                       \
	do {                                                                   \
		if (unlikely(eero_br->log_level & bit)) {                      \
			br_eero_debug_log(eero_br, "%s: " fmt, component,      \
					  ##__VA_ARGS__);                      \
		}                                                              \
	} while (0)
#else
#define br_eero_print(bit, eero_br, component, fmt, ...)                       \
	do {                                                                   \
		if (unlikely(eero_br->log_level & bit)) {                      \
			pr_info("%s %s: " fmt, stamp_dbg_strs[bit], component, \
				##__VA_ARGS__);                                \
		}                                                              \
	} while (0)
#endif /* IS_ENABLED(CONFIG_EERO_BR_LOG_FILE) */

#define br_eero_debug(component, eero_br, fmt, ...)                            \
	br_eero_print(EERO_LOG_DEBUG, eero_br, component, fmt, ##__VA_ARGS__)

#define br_eero_info(component, eero_br, fmt, ...)                             \
	br_eero_print(EERO_LOG_INFO, eero_br, component, fmt, ##__VA_ARGS__)

#define br_eero_error(component, eero_br, fmt, ...)                            \
	br_eero_print(EERO_LOG_ERROR, eero_br, component, fmt, ##__VA_ARGS__)

/* segment/stamp related logs */
#define segment_log_debug(fmt, ...)                                            \
	br_eero_debug(STAMP_LOG, eero_br, fmt, ##__VA_ARGS__)

#define segment_log_info(fmt, ...)                                             \
	br_eero_info(STAMP_LOG, eero_br, fmt, ##__VA_ARGS__)

#define segment_log_error(fmt, ...)                                            \
	br_eero_error(STAMP_LOG, eero_br, fmt, ##__VA_ARGS__)

/* segment_id/stamp related logs */
#define segment_id_log_debug(fmt, ...)                                         \
	br_eero_debug(STAMP_ID_LOG, eero_br, fmt, ##__VA_ARGS__)

#define segment_id_log_info(fmt, ...)                                          \
	br_eero_info(STAMP_ID_LOG, eero_br, fmt, ##__VA_ARGS__)

#define segment_id_log_error(fmt, ...)                                         \
	br_eero_error(STAMP_ID_LOG, eero_br, fmt, ##__VA_ARGS__)

/* roam related logs */
#define roam_log_debug(fmt, ...)                                               \
	br_eero_debug(ROAM_LOG, eero_br, fmt, ##__VA_ARGS__)

#define roam_log_info(fmt, ...)                                                \
	br_eero_info(ROAM_LOG, eero_br, fmt, ##__VA_ARGS__)

#define roam_log_error(fmt, ...)                                               \
	br_eero_error(ROAM_LOG, eero_br, fmt, ##__VA_ARGS__)

/* Netlink/Genl structs/functions */
/**
 * enum br_eero_genl_attrs - supported br eero attributes
 *
 * @BR_EERO_GENL_ATTR_UNSPEC: unspecified attribute
 * @BR_EERO_GENL_ATTR_MAC_ADDR: mac address sent from above
 */
enum br_eero_genl_attrs {
	BR_EERO_GENL_ATTR_UNSPEC,
	BR_EERO_GENL_ATTR_MAC_ADDR,
	/* private: internal use only */
	__BR_EERO_GENL_ATTR_AFTER_LAST,
};

#define BR_EERO_GENL_ATTR_MAX (__BR_EERO_GENL_ATTR_AFTER_LAST - 1)

/**
 * enum br_eero_genl_commands - supported br eero commands
 *
 * @BR_EERO_GENL_CMD_UNSPEC: unspecified command
 * @BR_EERO_GENL_CMD_EERO_OUI: eero oui
 * @BR_EERO_GENL_CMD_NEIGH: neighbor address used for unicast
 * @BR_EERO_GENL_CMD_SONOS_OUI: sonos oui
 *
 */
enum br_eero_genl_commands {
	BR_EERO_GENL_CMD_UNSPEC,
	BR_EERO_GENL_CMD_EERO_OUI,
	BR_EERO_GENL_CMD_NEIGH,
	BR_EERO_GENL_CMD_SONOS_OUI,
	/* private: internal use only */
	__BR_EERO_GENL_CMD_AFTER_LAST
};

#define BR_EERO_GENL_CMD_MAX (__BR_EERO_GENL_CMD_AFTER_LAST - 1)

#endif
#endif /* CONFIG_BIFROST_MESH */
