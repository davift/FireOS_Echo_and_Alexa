/*
 * Copyright (c) 2018 eero Ltd.
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
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
#ifndef _EERO_SKB_TRACER_H
#define _EERO_SKB_TRACER_H

#define SKB_MARK_PRINT 0x1

#if IS_ENABLED(CONFIG_SKB_TRACER)

#include <net/net_namespace.h>
#include <linux/printk.h>

#define skb_trace(skb, dst, src)                                                   \
	do {                                                                       \
		if (unlikely(init_net.core.sysctl_skb_trace) &&                    \
		    skb->mark & SKB_MARK_PRINT) {                                  \
			if (!skb->csum)                                            \
				skb->csum = skb_checksum(skb, 0, skb->len, 0);     \
			printk("%s:%d: dev %s frame %x mark %x dst %pM src %pM\n", \
			       __func__, __LINE__,                                 \
			       skb->dev ? skb->dev->name : NULL, skb->csum,        \
			       skb->mark, dst, src);                               \
		}                                                                  \
	} while (0)

#else
static inline void skb_trace(struct sk_buff const *skb, u8 const *dst,
			     u8 const *src)
{
	return;
};
#endif /* CONFIG_SKB_TRACER */

#endif /* _EERO_SKB_TRACER_H */
#endif /* CONFIG_BIFROST_MESH */
