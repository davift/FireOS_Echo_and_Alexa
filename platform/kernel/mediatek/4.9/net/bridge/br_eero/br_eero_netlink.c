/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <net/genetlink.h>
#include "br_eero.h"

static const struct nla_policy br_eero_genl_policy[BR_EERO_GENL_ATTR_MAX + 1] = {
	[BR_EERO_GENL_ATTR_MAC_ADDR] = { .type = NLA_STRING,
					 .len = BR_EERO_GENL_ADDR_MAX_SIZE },
};

static int br_eero_genl_set_mac_addr(struct sk_buff *skb,
				     struct genl_info *info, int list_type)
{
	struct nlattr *na;
	char *addr;

	if (!info)
		return 1;

	na = info->attrs[BR_EERO_GENL_ATTR_MAC_ADDR];
	if (na) {
		addr = (char *)nla_data(na);
		if (!addr)
			pr_debug("error while receiving data\n");
		else
			br_eero_add_mac_addr(addr, list_type);
	} else {
		pr_debug("no info->attrs %i\n",
		       BR_EERO_GENL_ATTR_MAC_ADDR);
	}
	return 0;
}

static int br_eero_genl_set_eero_oui(struct sk_buff *skb,
				     struct genl_info *info)
{
	return br_eero_genl_set_mac_addr(skb, info, EERO_OUI_LIST);
}

static int br_eero_genl_set_sonos_oui(struct sk_buff *skb,
				      struct genl_info *info)
{
	return br_eero_genl_set_mac_addr(skb, info, SONOS_OUI_LIST);
}

static int br_eero_genl_set_neigh(struct sk_buff *skb, struct genl_info *info)
{
	return br_eero_genl_set_mac_addr(skb, info, NEIGH_LIST);
}

static const struct genl_ops br_eero_genl_ops[] = {
	{
		.cmd = BR_EERO_GENL_CMD_EERO_OUI,
		.doit = br_eero_genl_set_eero_oui,
		.dumpit = NULL,
		.policy = br_eero_genl_policy,
	},
	{
		.cmd = BR_EERO_GENL_CMD_NEIGH,
		.doit = br_eero_genl_set_neigh,
		.dumpit = NULL,
		.policy = br_eero_genl_policy,
	},
	{
		.cmd = BR_EERO_GENL_CMD_SONOS_OUI,
		.doit = br_eero_genl_set_sonos_oui,
		.dumpit = NULL,
		.policy = br_eero_genl_policy,
	},
};

static struct genl_family br_eero_genl_family = {
	.id = GENL_ID_GENERATE,
	.hdrsize = 0,
	.name = BR_EERO_GENL_NAME,
	.version = BR_EERO_GENL_VERSION,
	.maxattr = BR_EERO_GENL_ATTR_MAX,
	.ops = br_eero_genl_ops,
	.n_ops = ARRAY_SIZE(br_eero_genl_ops),
};

int br_eero_genl_init(void)
{
	return genl_register_family(&br_eero_genl_family);
}

void br_eero_genl_deinit(void)
{
	genl_unregister_family(&br_eero_genl_family);
}
