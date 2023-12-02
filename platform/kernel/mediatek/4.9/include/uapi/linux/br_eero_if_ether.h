/*
 *  Additional global definitions for BiFrost extenstions to
 *  Ethernet IEEE 802.3 interface.
 *
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * Version:  @(#)if_ether.h  1.0.1a  02/08/94
 *
 * Author:  Mete Rodoper, <mrodoper@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifdef CONFIG_BIFROST_MESH
#ifndef _BR_EERO_UAPI_LINUX_IF_ETHER_H
#define _BR_EERO_UAPI_LINUX_IF_ETHER_H

#include <uapi/linux/if_ether.h>

#define ETH_P_BIFROST                                                          \
	0x9104 /* BIFROST protocol packet [ NOT AN OFFICIALLY REGISTERED ID ] */

#endif /* _BR_EERO_UAPI_LINUX_IF_ETHER_H */
#endif /* CONFIG_BIFROST_MESH */
