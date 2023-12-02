/* eero-specific netdevice extensions.
 * Copyright (c) 2019 eero Inc.
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * Authors: Thia Wyrod (thia@eero.com)
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifdef CONFIG_BIFROST_MESH
#ifndef _EERO_LINUX_NETDEVICE_H
#define _EERO_LINUX_NETDEVICE_H

#include <linux/netdev_features.h>

enum { EERO_IFF_MBSS_PORT = 1 << 0, /* eero mesh device */
};

/* eero device-specific extensions to net_device */
struct eero_net_device_ext {
	netdev_features_t features;
};
typedef struct eero_net_device_ext eero_net_device_ext_t;
#endif /* _EERO_LINUX_NETDEVICE_H */
#endif /* CONFIG_BIFROST_MESH */
