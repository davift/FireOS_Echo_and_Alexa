/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2016 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/*! \file   gl_mesh_os.h
*    \brief  This file is for WLAN MESH MAC80211 support.
*
*/

#ifndef _GL_MESH_MAC80211_H
#define _GL_MESH_MAC80211_H

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <net/mac80211.h>

#include "gl_os.h"

/*******************************************************************************
*                    E X T E R N A L   V A R I A B L E
********************************************************************************
*/

#if CFG_ENABLE_WIFI_MESH
/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/
#define MESH_SUPPORTED_FILTERS		(FIF_ALLMULTI | FIF_BCN_PRBRESP_PROMISC)

/* MUST keep sync with struct mtk_rates */
enum ENUM_MAC80211_SW_RATE_INDEX_T {
	MAC80211_RATE_1M_SW_INDEX = 0,	/* 1M */
	MAC80211_RATE_2M_SW_INDEX,	/* 2M */
	MAC80211_RATE_5_5M_SW_INDEX,	/* 5.5M */
	MAC80211_RATE_11M_SW_INDEX,	/* 11M */
	MAC80211_RATE_6M_SW_INDEX,	/* 6M */
	MAC80211_RATE_9M_SW_INDEX,	/* 9M */
	MAC80211_RATE_12M_SW_INDEX,	/* 12M */
	MAC80211_RATE_18M_SW_INDEX,	/* 18M */
	MAC80211_RATE_24M_SW_INDEX,	/* 24M */
	MAC80211_RATE_36M_SW_INDEX,	/* 36M */
	MAC80211_RATE_48M_SW_INDEX,	/* 48M */
	MAC80211_RATE_54M_SW_INDEX,	/* 54M */
	MAC80211_RATE_HT_PHY_SW_INDEX,	/* BSS Selector - HT PHY */
	MAC80211_RATE_NUM_SW		/* 13 */
};

/* Rate set bit definitions */
#define MAC80211_RATE_SET_BIT_1M         BIT(MAC80211_RATE_1M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_2M         BIT(MAC80211_RATE_2M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_5_5M       BIT(MAC80211_RATE_5_5M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_11M        BIT(MAC80211_RATE_11M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_6M         BIT(MAC80211_RATE_6M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_9M         BIT(MAC80211_RATE_9M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_12M        BIT(MAC80211_RATE_12M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_18M        BIT(MAC80211_RATE_18M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_24M        BIT(MAC80211_RATE_24M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_36M        BIT(MAC80211_RATE_36M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_48M        BIT(MAC80211_RATE_48M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_54M        BIT(MAC80211_RATE_54M_SW_INDEX)
#define MAC80211_RATE_SET_BIT_HT_PHY     BIT(MAC80211_RATE_HT_PHY_SW_INDEX)

#define MAC80211_RATE_SET_HR_DSSS   (MAC80211_RATE_SET_BIT_1M | \
				     MAC80211_RATE_SET_BIT_2M | \
				     MAC80211_RATE_SET_BIT_5_5M | \
				     MAC80211_RATE_SET_BIT_11M)

#define MAC80211_RATE_SET_OFDM      (MAC80211_RATE_SET_BIT_6M | \
				     MAC80211_RATE_SET_BIT_9M | \
				     MAC80211_RATE_SET_BIT_12M | \
				     MAC80211_RATE_SET_BIT_18M | \
				     MAC80211_RATE_SET_BIT_24M | \
				     MAC80211_RATE_SET_BIT_36M | \
				     MAC80211_RATE_SET_BIT_48M | \
				     MAC80211_RATE_SET_BIT_54M)

#define MAC80211_RATE_SET_ERP       (MAC80211_RATE_SET_BIT_1M | \
				     MAC80211_RATE_SET_BIT_2M | \
				     MAC80211_RATE_SET_BIT_5_5M | \
				     MAC80211_RATE_SET_BIT_11M | \
				     MAC80211_RATE_SET_BIT_6M | \
				     MAC80211_RATE_SET_BIT_9M | \
				     MAC80211_RATE_SET_BIT_12M | \
				     MAC80211_RATE_SET_BIT_18M | \
				     MAC80211_RATE_SET_BIT_24M | \
				     MAC80211_RATE_SET_BIT_36M | \
				     MAC80211_RATE_SET_BIT_48M | \
				     MAC80211_RATE_SET_BIT_54M)

#define MAC80211_RATE_SET_ALL_ABG             MAC80211_RATE_SET_ERP

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/
#define mesh_rate_set_size	(ARRAY_SIZE(mesh_mtk_rate_set))

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
struct MESH_SCAN_IES {
	uint16_t u2CommonIeLen;
	uint16_t u2Band2G4IeLen;
	uint16_t u2Band5GIeLen;
	uint8_t ie[];
};

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
/* mac80211 hooks */
int mtk_mac80211_txinfo_backup_in_skb(void *prSkb);

int mtk_mac80211_txinfo_backup_in_msduinfo(void *prSkb,
					   struct MSDU_INFO *prMsduInfo);

int mtk_mac80211_txinfo_restore_fr_msduinfo(void *prSkb,
					    struct MSDU_INFO *prMsduInfo);

void mtk_mesh_mac80211_tx(struct ieee80211_hw *hw,
			  struct ieee80211_tx_control *control,
			  struct sk_buff *skb);

int mtk_mesh_mac80211_start(struct ieee80211_hw *hw);

void mtk_mesh_mac80211_stop(struct ieee80211_hw *hw);

int mtk_mesh_mac80211_add_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif);

void mtk_mesh_mac80211_remove_interface(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif);

int mtk_mesh_mac80211_config(struct ieee80211_hw *hw, u32 changed);

void mtk_mesh_mac80211_configure_filter(struct ieee80211_hw *hw,
					unsigned int changed_flags,
					unsigned int *total_flags,
					u64 multicast);

int mtk_mesh_mac80211_sta_add(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta);

int mtk_mesh_mac80211_sta_remove(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta);

void mtk_mesh_mac80211_sta_notify(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  enum sta_notify_cmd cmd,
				  struct ieee80211_sta *sta);

int mtk_mesh_mac80211_hw_scan(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_scan_request *req);

void  mtk_mesh_mac80211_cancel_hw_scan(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif);

int mtk_mesh_mac80211_set_rts_threshold(struct ieee80211_hw *hw, u32 value);

int mtk_mesh_mac80211_mbss_beacon_update(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif);

int mtk_mesh_mac80211_mbss_beacon_enable(struct ieee80211_hw *hw,
						 struct ieee80211_vif *vif);

int mtk_mesh_mac80211_mbss_beacon_disable(struct ieee80211_hw *hw,
						  struct ieee80211_vif *vif);

void mtk_mesh_mac80211_bss_info_changed(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif,
					struct ieee80211_bss_conf *bss_conf,
					u32 changed);

int mtk_mesh_mac80211_add_key(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta,
			      struct ieee80211_key_conf *key);

int mtk_mesh_mac80211_delete_key(struct ieee80211_hw *hw,
				 struct ieee80211_sta *sta,
				 struct ieee80211_key_conf *key);

int mtk_mesh_mac80211_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta,
			      struct ieee80211_key_conf *key);

int mtk_mesh_mac80211_suspend(struct ieee80211_hw *hw,
			      struct cfg80211_wowlan *wowlan);

unsigned long long mtk_mesh_mac80211_get_tsf(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif);

int mtk_mesh_mac80211_add_chanctx(struct ieee80211_hw *hw,
				  struct ieee80211_chanctx_conf *ctx);

int mtk_mesh_mac80211_get_survey(struct ieee80211_hw *hw,
				  int idx, struct survey_info *survey);

void mtk_mesh_mac80211_remove_chanctx(struct ieee80211_hw *hw,
				      struct ieee80211_chanctx_conf *ctx);

void mtk_mesh_mac80211_change_chanctx(struct ieee80211_hw *hw,
				      struct ieee80211_chanctx_conf *ctx,
				      u32 changed);

int mtk_mesh_mac80211_assign_vif_chanctx(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif,
					 struct ieee80211_chanctx_conf *ctx);

void mtk_mesh_mac80211_unassign_vif_chanctx(struct ieee80211_hw *hw,
					    struct ieee80211_vif *vif,
					    struct ieee80211_chanctx_conf *ctx);

int mtk_mesh_mac80211_set_tim(struct ieee80211_hw *hw,
			      struct ieee80211_sta *sta, bool set);

#if KERNEL_VERSION(4, 8, 0) <= CFG80211_VERSION_CODE
unsigned int mtk_mesh_mac80211_get_throughput(struct ieee80211_hw *hw,
					 struct ieee80211_sta *sta);
#else
unsigned int mtk_mesh_mac80211_get_throughput(struct ieee80211_sta *sta);
#endif

void mtk_mesh_mac80211_hw_scan_work(struct work_struct *work);

void getMeshChan(struct cfg80211_chan_def *chandef, unsigned char *channum,
		 enum ENUM_BAND *prBand, enum ENUM_CHNL_EXT *prChnlSco);

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

#endif	/* CFG_ENABLE_WIFI_MESH */
#endif	/* _GL_MESH_MAC80211_H */
