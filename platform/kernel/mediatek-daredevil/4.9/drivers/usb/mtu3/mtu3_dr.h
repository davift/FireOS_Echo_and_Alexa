/*
 * mtu3_dr.h - dual role switch and host glue layer header
 *
 * Copyright (C) 2016 MediaTek Inc.
 *
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MTU3_DR_H_
#define _MTU3_DR_H_

#if IS_ENABLED(CONFIG_USB_MTU3_HOST) || IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)

int ssusb_host_init(struct ssusb_mtk *ssusb, struct device_node *parent_dn);
void ssusb_host_exit(struct ssusb_mtk *ssusb);
int ssusb_wakeup_of_property_parse(struct ssusb_mtk *ssusb,
				struct device_node *dn);
int ssusb_host_enable(struct ssusb_mtk *ssusb);
int ssusb_host_disable(struct ssusb_mtk *ssusb, bool suspend);
int ssusb_wakeup_enable(struct ssusb_mtk *ssusb);
void ssusb_wakeup_disable(struct ssusb_mtk *ssusb);

#else

static inline int ssusb_host_init(struct ssusb_mtk *ssusb,

	struct device_node *parent_dn)
{
	return 0;
}

static inline void ssusb_host_exit(struct ssusb_mtk *ssusb)
{}

static inline int ssusb_wakeup_of_property_parse(
	struct ssusb_mtk *ssusb, struct device_node *dn)
{
	return 0;
}

static inline int ssusb_host_enable(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline int ssusb_host_disable(struct ssusb_mtk *ssusb, bool suspend)
{
	return 0;
}

static inline int ssusb_wakeup_enable(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline void ssusb_wakeup_disable(struct ssusb_mtk *ssusb)
{}

#endif


#if IS_ENABLED(CONFIG_USB_MTU3_GADGET) || IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)
int ssusb_gadget_init(struct ssusb_mtk *ssusb);
void ssusb_gadget_exit(struct ssusb_mtk *ssusb);
#else
static inline int ssusb_gadget_init(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline void ssusb_gadget_exit(struct ssusb_mtk *ssusb)
{}
#endif


#if IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)
int ssusb_otg_switch_init(struct ssusb_mtk *ssusb);
void ssusb_otg_switch_exit(struct ssusb_mtk *ssusb);
int ssusb_set_vbus(struct otg_switch_mtk *otg_sx, int is_on);
void ssusb_set_force_mode(struct ssusb_mtk *ssusb,
			  enum mtu3_dr_force_mode mode);
#if IS_ENABLED(CONFIG_USB_MTU3_PLAT_PHONE)
extern u32 mtu3_speed;
#endif

#else

static inline int ssusb_otg_switch_init(struct ssusb_mtk *ssusb)
{
	return 0;
}

static inline void ssusb_otg_switch_exit(struct ssusb_mtk *ssusb)
{}

static inline int ssusb_set_vbus(struct otg_switch_mtk *otg_sx, int is_on)
{
	return 0;
}
static inline void
ssusb_set_force_mode(struct ssusb_mtk *ssusb, enum mtu3_dr_force_mode mode)
{}

#endif

#if IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)
#if IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)
void mtu3_drp_to_none(struct mtu3 *mtu3);
void mtu3_drp_to_device(struct mtu3 *mtu3);
void mtu3_drp_to_host(struct mtu3 *mtu3);
#else
static inline void mtu3_drp_to_none(struct mtu3 *mtu3)
{}

static inline void mtu3_drp_to_device(struct mtu3 *mtu3)
{}

static inline void mtu3_drp_to_host(struct mtu3 *mtu3)
{}
#endif
#endif
#endif		/* _MTU3_DR_H_ */
