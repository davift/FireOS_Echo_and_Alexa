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
/*! \file   gl_mesh_mac80211.c
*    \brief  Main routines of Linux driver interface for Wi-Fi Mesh using
*	    mac80211 interface
*
*    This file contains the main routines of Linux driver for MediaTek Inc.
*    802.11 Wireless LAN Adapters.
*/

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
#include "precomp.h"

#if CFG_ENABLE_WIFI_MESH
/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/
/* Rate same as in mtk_rates[] */
int mesh_mtk_rate_set[] = {
	RATE_SET_BIT_1M,
	RATE_SET_BIT_2M,
	RATE_SET_BIT_5_5M,
	RATE_SET_BIT_11M,
	RATE_SET_BIT_6M,
	RATE_SET_BIT_9M,
	RATE_SET_BIT_12M,
	RATE_SET_BIT_18M,
	RATE_SET_BIT_24M,
	RATE_SET_BIT_36M,
	RATE_SET_BIT_48M,
	RATE_SET_BIT_54M,
};

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/
int mtk_mac80211_txinfo_backup_in_skb(void *prSkb)
{
	struct sk_buff *skb = (struct sk_buff *) prSkb;
	struct ieee80211_tx_info *info;
	uint8_t *info_in_skb;

	if (!skb) {
		DBGLOG(MESH, ERROR, "skb NOT exist\n");
		return WLAN_STATUS_FAILURE;
	}

	info = IEEE80211_SKB_CB(skb);

	if (!info) {
		DBGLOG(MESH, ERROR, "tx_info NOT exist\n");
		return WLAN_STATUS_FAILURE;
	}

	if (skb_headroom(skb) < MAC80211_TX_INFO_LEN) {
		DBGLOG(MESH, ERROR, "skb headroom NOT enough backup tx_info\n");
		return WLAN_STATUS_FAILURE;
	}

	/* backup mac80211 tx_info into skb before backup to MsduInfo */
	info_in_skb = skb_push(skb, MAC80211_TX_INFO_LEN);
	kalMemCopy(info_in_skb, info, MAC80211_TX_INFO_LEN);

	return WLAN_STATUS_SUCCESS;
}

int mtk_mac80211_txinfo_backup_in_msduinfo(void *prSkb,
					   struct MSDU_INFO *prMsduInfo)
{
	struct sk_buff *skb = (struct sk_buff *) prSkb;

	if (!skb || !prMsduInfo) {
		DBGLOG(MESH, ERROR, "skb or MsduInfo NOT exist\n");
		return WLAN_STATUS_FAILURE;
	}

	kalMemCopy(prMsduInfo->tx_info, skb->data, MAC80211_TX_INFO_LEN);
	skb_pull(skb, MAC80211_TX_INFO_LEN);

	return WLAN_STATUS_SUCCESS;
}

int mtk_mac80211_txinfo_restore_fr_msduinfo(void *prSkb,
					    struct MSDU_INFO *prMsduInfo)
{
	struct sk_buff *skb = (struct sk_buff *) prSkb;
	struct ieee80211_tx_info *tx_info;

	if (!skb || !prMsduInfo) {
		DBGLOG(MESH, ERROR, "skb or MsduInfo NOT exist\n");
		return WLAN_STATUS_FAILURE;
	}

	tx_info = IEEE80211_SKB_CB(skb);

	if (!tx_info) {
		DBGLOG(MESH, ERROR, "tx_info NOT exist\n");
		return WLAN_STATUS_FAILURE;
	}

	kalMemCopy(tx_info, prMsduInfo->tx_info, MAC80211_TX_INFO_LEN);

	return WLAN_STATUS_SUCCESS;
}

void mtk_mesh_mac80211_tx(struct ieee80211_hw *hw,
			  struct ieee80211_tx_control *control,
			  struct sk_buff *skb)
{
	struct ADAPTER *prAdapter;
	struct GLUE_INFO *prGlueInfo;
	struct MESH_INFO *prMeshInfo;
	struct STA_RECORD *prStaRec;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	uint8_t ucStaRecIndex = STA_REC_INDEX_NOT_FOUND;
	unsigned int result = WLAN_STATUS_SUCCESS;

	ASSERT(hw);
	ASSERT(control);
	ASSERT(skb);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	if (ieee80211_is_data(hdr->frame_control))
		DBGLOG(MESH, LOUD, "    Data Frame...\n");
	else if (ieee80211_is_mgmt(hdr->frame_control))
		DBGLOG(MESH, LOUD, "    Mgmt Frame...\n");
	if (ieee80211_is_action(hdr->frame_control))
		DBGLOG(MESH, LOUD, "    Action Frame...\n");
	if (ieee80211_is_auth(hdr->frame_control))
		DBGLOG(MESH, LOUD, "    Auth Frame...\n");
	if (ieee80211_is_probe_req(hdr->frame_control))
		DBGLOG(MESH, LOUD, "    Probe Req Frame...\n");

	DBGLOG(MESH, LOUD, "Tx Dest:"MACSTR"\n", MAC2STR(hdr->addr1));
	DBGLOG(MESH, LOUD, "sk_buff->len: %d 0x%p\n", skb->len, skb);

	/* MESH TX Stats */
	GLUE_INC_REF_CNT(prAdapter->rTxCtrl.incomingMac80211Num);

	/* Do frame field sanity check.
	 * On kernel v4.8.17 + wpa_supplicant 2017/06/12, driver sometimes
	 * receives authentication frames with wrong authentication algorithm
	 * field. If driver transmits the frame, it would lead mesh peering
	 * fail this time.
	 */
	if (ieee80211_is_auth(hdr->frame_control)) {
		struct ieee80211_mgmt *mgmt =
					(struct ieee80211_mgmt *) skb->data;

		if (mgmt->u.auth.auth_alg != WLAN_AUTH_SAE) {
			DBGLOG(MESH, ERROR, "Drop wrong mac80211 auth frame\n");
			goto tx_fail;
		}
	}

	/* for TXD */
	if (skb_headroom(skb) < KAL_GET_MAX(NIC_TX_DESC_AND_PADDING_LENGTH
					+ prAdapter->chip_info->txd_append_size,
					sizeof(struct ieee80211_tx_info))) {
		DBGLOG(MESH, ERROR, "skb headroom NOT enough for TXD\n");
		result = WLAN_STATUS_BUFFER_TOO_SHORT;
		goto tx_fail;
	}

	if (!is_multicast_ether_addr(hdr->addr1)) {
		prStaRec = cnmGetStaRecByAddress(prAdapter,
						 prMeshInfo->ucBssIndex,
						 hdr->addr1);

		/* Drop Unicast Data Frames if StaRec doesn't exist */
		if (!prStaRec && ieee80211_is_data(hdr->frame_control)) {
			DBGLOG(MESH, ERROR,
			       "NO StaRec for this UC data\n");
			result = WLAN_STATUS_INVALID_PACKET;
			goto tx_fail;
		}

		if (prStaRec)
			ucStaRecIndex = prStaRec->ucIndex;
	} else
		ucStaRecIndex = STA_REC_INDEX_BMCAST;

	if (ieee80211_is_data(hdr->frame_control))
		result = wlanEnqueueMac80211TxData(prAdapter, skb,
						   ucStaRecIndex);
	else if (ieee80211_is_mgmt(hdr->frame_control))
		result = wlanEnqueueMac80211TxMgmt(prAdapter, skb,
						   ucStaRecIndex);
	else {
		DBGLOG(MESH, ERROR, "invalid pkt type\n");
		result = WLAN_STATUS_INVALID_PACKET;
	}

	if (result != WLAN_STATUS_SUCCESS)
		goto tx_fail;

	/* TODO incref count total packets --considering AP-Mesh concurrency.
	 * --Add flow control?
	 */

	/* TODO: [mesh] replace with TRX direct in USB mode */
	kalSetEvent(prGlueInfo);

	return;

tx_fail:
	DBGLOG(MESH, INFO, "TX fail err=0x%X\n", result);

	kalFreeMac80211Packet(prGlueInfo, (void *) skb);
}

int mtk_mesh_mac80211_start(struct ieee80211_hw *hw)
{
	/* Nothing to do.
	 * The reason is that HW init is already done in STA IF registration
	 * flow.
	 */
	struct GLUE_INFO *prGlueInfo = NULL;

	DBGLOG(MESH, INFO, "mtk_mesh_mac80211_start\n");

	ASSERT(hw);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	kalSurveyDataControl(prGlueInfo, KAL_SURVEY_DATA_ENABLE);

	return 0;
}

void mtk_mesh_mac80211_stop(struct ieee80211_hw *hw)
{
	/* Nothing to do.
	 * The reason is that there may be some other IFs exist, so we should
	 * NOT turn off HW directly.
	 */
	struct GLUE_INFO *prGlueInfo = NULL;

	DBGLOG(MESH, INFO, "mtk_mesh_mac80211_stop\n");

	ASSERT(hw);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	kalSurveyDataControl(prGlueInfo, KAL_SURVEY_DATA_DISABLE);
}

/* The assumption here is that only mesh IF is hooked to mac80211 while other
 * IFs are hooked to cfg80211. When this callback is called, the intention is to
 * add mesh IF. That's why there's NO ieee80211_vif.type check.
 */
int mtk_mesh_mac80211_add_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif)
{
	struct ADAPTER *prAdapter;
	struct GLUE_INFO *prGlueInfo;
	struct GL_P2P_INFO *prGlP2pInfo;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;
	struct MESH_FSM_INFO *prMeshFsmInfo;
	struct MSG_MESH_FSM_TRANSITION *prMsg;
	int i = 0;
	const uint32_t u4Timeout = MESH_FSM_TRANSITION_TIMEOUT_MS;

	DBGLOG(MESH, INFO, "add IF\n");

	ASSERT(hw);
	ASSERT(vif);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prMeshFsmInfo = &prMeshInfo->rMeshFsmInfo;

	/* Check if the AP interface has the same address */
	for (i = 0; i < KAL_P2P_NUM; i++) {
		prGlP2pInfo = prGlueInfo->prP2PInfo[i];
		if (prGlP2pInfo &&
		    prGlP2pInfo->prDevHandler &&
		    prGlP2pInfo->prDevHandler->dev_addr &&
		    prGlP2pInfo->prDevHandler->flags & IFF_UP &&
		    ether_addr_equal(vif->addr,
				     prGlP2pInfo->prDevHandler->dev_addr)) {
			DBGLOG(MESH, ERROR, "p2p & mesh same mac addr\n");
			return -ENOTUNIQ;
		}
	}

	prMsg = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
			    sizeof(struct MSG_MESH_FSM_TRANSITION));
	if (!prMsg) {
		DBGLOG(MESH, ERROR, "can't alloc MSG\n");
		return -ENOBUFS;
	}

	prGlMeshInfo->vif = vif;

	vif->cab_queue = 0;
	vif->hw_queue[IEEE80211_AC_VO] = 0;
	vif->hw_queue[IEEE80211_AC_VI] = 1;
	vif->hw_queue[IEEE80211_AC_BE] = 2;
	vif->hw_queue[IEEE80211_AC_BK] = 3;

	/* We guarantee that any request from mac80211 related to mesh FSM would
	 * be processed sequentially by push a message into mailbox.
	 */
	prMsg->rMsgHdr.eMsgId = MID_MNY_MESH_FSM_TRANSITION;
	prMsg->eNextState = MESH_STATE_IDLE;

	mboxSendMsg(prAdapter, MBOX_ID_0, (struct MSG_HDR *) prMsg,
		    MSG_SEND_METHOD_BUF);

	if (!wait_event_timeout(prGlMeshInfo->mesh_wait,
				IS_MESH_FSM_STATE(prAdapter, MESH_STATE_IDLE),
				MSEC_TO_JIFFIES(u4Timeout))) {
		DBGLOG(MESH, ERROR, "add IF fail\n");
		return -EBUSY;
	}

	return 0;
}

/* The assumption here is that only mesh IF is hooked to mac80211 while other
 * IFs are hooked to cfg80211. When this callback is called, the intention is to
 * remove mesh IF. That's why there's NO ieee80211_vif.type check.
 */
void mtk_mesh_mac80211_remove_interface(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif)
{
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;
	struct MESH_FSM_INFO *prMeshFsmInfo;
	struct MSG_MESH_FSM_TRANSITION *prMsg;
	const uint32_t u4Timeout = MESH_FSM_TRANSITION_TIMEOUT_MS;

	DBGLOG(MESH, INFO, "remove IF\n");

	ASSERT(hw);
	ASSERT(vif);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prMeshFsmInfo = &prMeshInfo->rMeshFsmInfo;

	prMsg = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
			    sizeof(struct MSG_MESH_FSM_TRANSITION));
	if (!prMsg) {
		DBGLOG(MESH, ERROR, "can't alloc MSG\n");
		ASSERT(0);
	}

	/* We guarantee that any request from mac80211 related to mesh FSM would
	 * be processed sequentially by push a message into mailbox.
	 */
	prMsg->rMsgHdr.eMsgId = MID_MNY_MESH_FSM_TRANSITION;
	prMsg->eNextState = MESH_STATE_DISABLE;

	mboxSendMsg(prAdapter, MBOX_ID_0, (struct MSG_HDR *) prMsg,
		    MSG_SEND_METHOD_BUF);

	if (!wait_event_timeout(prGlMeshInfo->mesh_wait,
				IS_MESH_FSM_STATE(prAdapter,
						  MESH_STATE_DISABLE),
				MSEC_TO_JIFFIES(u4Timeout)))
		DBGLOG(MESH, ERROR, "add IF fail\n");
}

int mtk_mesh_mac80211_config(struct ieee80211_hw *hw, u32 changed)
{
	struct GLUE_INFO *prGlueInfo;
	/* struct ieee80211_conf *conf = &hw->conf; */
	int rStatus = 0;

	DBGLOG(MESH, INFO, "config\n");

	ASSERT(hw);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	/**
	 * enum ieee80211_conf_changed - denotes which configuration changed
	 *
	 * @IEEE80211_CONF_CHANGE_LISTEN_INTERVAL: the listen interval changed
	 * @IEEE80211_CONF_CHANGE_MONITOR: the monitor flag changed
	 * @IEEE80211_CONF_CHANGE_PS: the PS flag or dynamic PS timeout changed
	 * @IEEE80211_CONF_CHANGE_POWER: the TX power changed
	 * @IEEE80211_CONF_CHANGE_CHANNEL: the channel/channel_type changed
	 * @IEEE80211_CONF_CHANGE_RETRY_LIMITS: retry limits changed
	 * @IEEE80211_CONF_CHANGE_IDLE: Idle flag changed
	 * @IEEE80211_CONF_CHANGE_SMPS: Spatial multiplexing powersave mode
	 *	changed. Note that this is only valid if channel contexts are
	 *	not used, otherwise each channel context has the number of
	 *	chains listed.
	 */
	/* TODO: [mesh] */

	return rStatus;
}

void mtk_mesh_mac80211_configure_filter(struct ieee80211_hw *hw,
					unsigned int changed_flags,
					unsigned int *total_flags,
					u64 multicast)
{
	struct GLUE_INFO *prGlueInfo;
	struct GL_MESH_INFO *prGlMeshInfo;
	unsigned int u4PacketFilter = 0;
	unsigned int u4SetInfoLen;
	int u4Status = WLAN_STATUS_SUCCESS;

	ASSERT(hw);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);

	DBGLOG(MESH, INFO, "config filter\n");

	changed_flags &= MESH_SUPPORTED_FILTERS;
	*total_flags &= MESH_SUPPORTED_FILTERS;

	prGlMeshInfo->rx_filter = *total_flags;

	/* No matter what, allways accepts BC frames */
	u4PacketFilter |= PARAM_PACKET_FILTER_BROADCAST;

	if (*total_flags & FIF_ALLMULTI) {
		u4PacketFilter |= PARAM_PACKET_FILTER_ALL_MULTICAST;
		DBGLOG(MESH, INFO, "    SET FIF_ALLMULTI\n");
	}

	if (*total_flags & FIF_BCN_PRBRESP_PROMISC) {
		/* For beacon frames,
		 * HW supports drop them from different BSSID, but fw handles it
		 * automatically in mesh. FW would default drop beacon frames
		 * from different BSSID unless there are active BSSs belonging
		 * to AP or IBSS or mesh or BOW.
		 *
		 * For probe responses,
		 * HW does NOT support RX filter control.
		 */
		u4PacketFilter |= PARAM_PACKET_FILTER_BROADCAST;
		DBGLOG(MESH, INFO, "    SET FIF_BCN_PRBRESP_PROMISC\n");
	}

	/* @FIF_FCSFAIL: HW support but NO requirement
	 *
	 * @FIF_PLCPFAIL: HW NOT support
	 *
	 * @FIF_CONTROL: HW support but NO requirement
	 *
	 * @FIF_OTHER_BSS: Need further check
	 *
	 * @FIF_PSPOLL: HW NOT support
	 *
	 * @FIF_PROBE_REQ: HW support but fw handles it automatically in mesh
	 * Fw would default drop probe requests unless there are active BSSs
	 * belongin to AP or IBSS or mesh or BOW.
	 */

	/* HW does NOT support specific action frame RX filter. */

	u4Status = kalIoctl(prGlueInfo,
			    wlanoidSetCurrentPacketFilter,
			    &u4PacketFilter,
			    sizeof(u4PacketFilter),
			    FALSE,
			    FALSE,
			    TRUE,
			    &u4SetInfoLen);

	if (u4Status)
		DBGLOG(MESH, ERROR, "set RX filter fail err=0x%X\n", u4Status);
}

/* TODO: [mesh] HOWTO prevent connect more peers when NO resource available? */
int mtk_mesh_mac80211_sta_add(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta)
{
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	struct GL_MESH_INFO *prGlMeshInfo;
	int rStatus = WLAN_STATUS_SUCCESS;
	unsigned int u4BufLen;
	struct MESH_CMD_PEER_ADD rMeshCmdPeerAdd;
	enum nl80211_band band = vif->bss_conf.chandef.chan->band;
	struct ieee80211_sta_ht_cap *ht_cap = &sta->ht_cap;
	struct ieee80211_mcs_info *mcs = &ht_cap->mcs;
	struct ieee80211_sta_vht_cap *vht_cap = &sta->vht_cap;
	struct ieee80211_vht_mcs_info *vht_mcs = &vht_cap->vht_mcs;
	struct mtk_sta_priv *prStaPriv;

	DBGLOG(MESH, INFO, "Add peer:"MACSTR"\n", MAC2STR(sta->addr));
	DBGLOG(MESH, INFO, "BW=%d\n", sta->bandwidth);
	DBGLOG(MESH, INFO, "support rates 2.4G=0x%X 5G=0x%X 60G=0x%X\n",
			   sta->supp_rates[0],
			   sta->supp_rates[1],
			   sta->supp_rates[2]);
	DBGLOG(MESH, INFO, "BSS basic rates=0x%X\n", vif->bss_conf.basic_rates);

	ASSERT(hw);
	ASSERT(vif);
	ASSERT(sta);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prStaPriv = (struct mtk_sta_priv *) sta->drv_priv;

	prStaPriv->prGlueInfo = prGlueInfo;

	kalMemZero(&rMeshCmdPeerAdd, sizeof(rMeshCmdPeerAdd));

	/* Copy MAC address */
	kalMemCopy(rMeshCmdPeerAdd.aucPeerMac, sta->addr, MAC_ADDR_LEN);

	/* Copy HT Params */
	/* This u2HtCapInfo is NOT really information copy from IE in
	 * MT6632/MT7668 design but an operation mode after all the intersection
	 * of both STAs.
	 * Ex: FW AR set TX rate BW according to bit 1
	 * HT_CAP_INFO_SUP_CHNL_WIDTH.
	 */
	rMeshCmdPeerAdd.u2HtCapInfo = ht_cap->cap;
	if (sta->bandwidth == IEEE80211_STA_RX_BW_20)
		rMeshCmdPeerAdd.u2HtCapInfo &= ~HT_CAP_INFO_SUP_CHNL_WIDTH;

	rMeshCmdPeerAdd.ucAmpduParam = (ht_cap->ampdu_density <<
					AMPDU_PARAM_MIN_START_SPACING_OFFSET) |
				       ht_cap->ampdu_factor;
	rMeshCmdPeerAdd.ucMcsSet = mcs->rx_mask[0] &
				   prGlMeshInfo->u4MeshHtMcsMask[band];
	kalMemCopy(rMeshCmdPeerAdd.aucRxMcsBitmask, mcs->rx_mask,
		   sizeof(rMeshCmdPeerAdd.aucRxMcsBitmask));

	rMeshCmdPeerAdd.u4VhtCapInfo = vht_cap->cap;
	rMeshCmdPeerAdd.u2VhtRxMcsMap = vht_mcs->rx_mcs_map;
	rMeshCmdPeerAdd.u2VhtRxHighestSupportedDataRate = vht_mcs->rx_highest;
	rMeshCmdPeerAdd.u2VhtTxMcsMap = vht_mcs->tx_mcs_map;
	rMeshCmdPeerAdd.u2VhtTxHighestSupportedDataRate = vht_mcs->tx_highest;
	/* This field name is misleading. It does NOT represent VHT Operation
	 * Information subfield in VHT Operation IE. It represents Operation
	 * Mode subfield in Operation Mode Noficiation IE.
	 */
	rMeshCmdPeerAdd.ucVhtOpMode = ((sta->bandwidth <<
					VHT_OP_MODE_CHANNEL_WIDTH_OFFSET) &
					VHT_OP_MODE_CHANNEL_WIDTH) |
				      ((sta->rx_nss <<
					VHT_OP_MODE_RX_NSS_OFFSET) &
					VHT_OP_MODE_RX_NSS);

	switch (band) {
	case KAL_BAND_2GHZ:
		rMeshCmdPeerAdd.u2BSSBasicRateSet = vif->bss_conf.basic_rates;
		rMeshCmdPeerAdd.u2OperationalRateSet = sta->supp_rates[band];

		/* logic here is refereced from assocProcessRxAssocReqFrame() */
		if (rMeshCmdPeerAdd.u2OperationalRateSet &
		    MAC80211_RATE_SET_OFDM)
			rMeshCmdPeerAdd.ucPhyTypeSet |= PHY_TYPE_BIT_ERP;
		if (rMeshCmdPeerAdd.u2OperationalRateSet &
		    MAC80211_RATE_SET_HR_DSSS)
			rMeshCmdPeerAdd.ucPhyTypeSet |= PHY_TYPE_BIT_HR_DSSS;
		break;

	case KAL_BAND_5GHZ:
		rMeshCmdPeerAdd.u2BSSBasicRateSet = vif->bss_conf.basic_rates <<
						    MAC80211_RATE_6M_SW_INDEX;
		rMeshCmdPeerAdd.u2OperationalRateSet = sta->supp_rates[band] <<
						      MAC80211_RATE_6M_SW_INDEX;

		if (rMeshCmdPeerAdd.u2OperationalRateSet &
		    MAC80211_RATE_SET_OFDM)
			rMeshCmdPeerAdd.ucPhyTypeSet |= PHY_TYPE_BIT_OFDM;
		break;

	default:
		ASSERT(0);
		break;
	}

	rMeshCmdPeerAdd.u2DesiredNonHTRateSet =
	       rMeshCmdPeerAdd.u2OperationalRateSet & MAC80211_RATE_SET_ALL_ABG;

	if (ht_cap->ht_supported)
		rMeshCmdPeerAdd.ucPhyTypeSet |= PHY_TYPE_BIT_HT;

	if (vht_cap->vht_supported)
		rMeshCmdPeerAdd.ucPhyTypeSet |= PHY_TYPE_BIT_VHT;

	rMeshCmdPeerAdd.ucDesiredPhyTypeSet = rMeshCmdPeerAdd.ucPhyTypeSet &
				prAdapter->rWifiVar.ucAvailablePhyTypeSet;

	/* Copy Assoc Id*/
	rMeshCmdPeerAdd.u2AssocId = sta->aid;

	DBGLOG(MESH, INFO,
		"BasicRate=0x%X OperRate=0x%X PhyType=0x%X DesirePhyType=0x%X\n",
		rMeshCmdPeerAdd.u2BSSBasicRateSet,
		rMeshCmdPeerAdd.u2OperationalRateSet,
		rMeshCmdPeerAdd.ucPhyTypeSet,
		rMeshCmdPeerAdd.ucDesiredPhyTypeSet);
	DBGLOG(MESH, INFO,
		"HtCapInfo=0x%X AmpduParam=%u McsSet=0x%X NonHTRate=0x%X\n",
		rMeshCmdPeerAdd.u2HtCapInfo,
		rMeshCmdPeerAdd.ucAmpduParam,
		rMeshCmdPeerAdd.ucMcsSet,
		rMeshCmdPeerAdd.u2DesiredNonHTRateSet);
	DBGLOG(MESH, INFO,
		"VhtCapInfo=0x%X VhtRxMcsMap=0x%X VhtRxHighestRate=0x%X\n",
		rMeshCmdPeerAdd.u4VhtCapInfo,
		rMeshCmdPeerAdd.u2VhtRxMcsMap,
		rMeshCmdPeerAdd.u2VhtRxHighestSupportedDataRate);
	DBGLOG(MESH, INFO,
		"VhtTxMcsMap=0x%X VhtTxHighestRate=0x%X VhtOpMode=0x%X\n",
		rMeshCmdPeerAdd.u2VhtTxMcsMap,
		rMeshCmdPeerAdd.u2VhtTxHighestSupportedDataRate,
		rMeshCmdPeerAdd.ucVhtOpMode);

	/* create a Mesh peer record */
	rStatus = kalIoctl(prGlueInfo, MeshPeerAdd, &rMeshCmdPeerAdd,
			   sizeof(struct MESH_CMD_PEER_ADD), FALSE, FALSE,
			   FALSE, &u4BufLen);

	if (rStatus) {
		DBGLOG(MESH, ERROR, "Add peer fail err=0x%X\n", rStatus);
		return -ENOBUFS;
	}

	return 0;
}

int mtk_mesh_mac80211_sta_remove(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta)
{
	struct GLUE_INFO *prGlueInfo;
	int rStatus = WLAN_STATUS_SUCCESS;
	unsigned int u4BufLen;
	struct MESH_CMD_PEER_REMOVE rMeshCmdPeerRemove;

	DBGLOG(MESH, INFO, "Remove peer:"MACSTR"\n", MAC2STR(sta->addr));

	ASSERT(hw);
	ASSERT(vif);
	ASSERT(sta);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	kalMemZero(&rMeshCmdPeerRemove, sizeof(rMeshCmdPeerRemove));

	/* Copy MAC address */
	kalMemCopy(rMeshCmdPeerRemove.aucPeerMac, sta->addr, MAC_ADDR_LEN);

	rStatus = kalIoctl(prGlueInfo, MeshPeerRemove, &rMeshCmdPeerRemove,
			   sizeof(struct MESH_CMD_PEER_REMOVE), FALSE, FALSE,
			   FALSE, &u4BufLen);

	if (rStatus) {
		DBGLOG(MESH, INFO, "Remove peer fail err=0x%X\n", rStatus);
		return -1;
	}

	return 0;
}

void mtk_mesh_mac80211_sta_notify(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  enum sta_notify_cmd cmd,
				  struct ieee80211_sta *sta)
{
	/* Maybe should be implemented when mesh low power is considered. */
}

int mtk_mesh_mac80211_hw_scan(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_scan_request *req)
{
	struct GLUE_INFO *prGlueInfo;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct BSS_INFO *prMeshBssInfo;
	struct cfg80211_scan_request *request;
	struct PARAM_SCAN_REQUEST_EXT *prScanReq;
	struct ADAPTER *prAdapter;
	struct MESH_INFO *prMeshInfo;

	ASSERT(hw);
	ASSERT(vif);
	ASSERT(req);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	prScanReq = &prGlMeshInfo->rScanRequest;

	DBGLOG(MESH, INFO, "hw scan req\n");

	/* check if there is any pending scan not yet finished */
	if (prGlMeshInfo->prScanRequest) {
		DBGLOG(MESH, ERROR, "Pending scan exist\n");
		return -EBUSY;
	}

	prMeshBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMeshInfo->ucBssIndex);
	ASSERT(prMeshBssInfo);
	if (prMeshBssInfo->fgIsBeaconActivated) {
		DBGLOG(MESH, ERROR, "Mesh already active\n");
		return -EBUSY;
	}

	request = &(req->req);

	kalMemZero(prScanReq, sizeof(struct PARAM_SCAN_REQUEST_EXT));

	if (request->n_ssids == 0)
		prScanReq->rSsid.u4SsidLen = 0;
	else if (request->n_ssids == 1)
		COPY_SSID(prScanReq->rSsid.aucSsid,
			  prScanReq->rSsid.u4SsidLen,
			  request->ssids[0].ssid, request->ssids[0].ssid_len);
	else {
		DBGLOG(MESH, ERROR, "Too many SSIDs %d\n", request->n_ssids);
		return -EINVAL;
	}

	prScanReq->u4IELength = sizeof(struct MESH_SCAN_IES) + request->ie_len;

	/*
	 * passing ieee80211_scan_req in pucIE, will do the segregation of
	 * IEs for different bands in wlanoidSetBssidListScanExtMesh().
	 */
	prScanReq->pucIE = (unsigned char *) &req->ies;

	/* TODO: [mesh] Do we need spin lock protect? */
	prGlMeshInfo->prScanRequest = request;

	/* TODO: [mesh] if scan fail, report err to kernel is better */
	ieee80211_queue_delayed_work(hw, &prGlMeshInfo->hw_scan, 0);

	return 0;
}

void mtk_mesh_mac80211_cancel_hw_scan(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif)
{
	/* Maybe should be implemented after WoW in mesh is considered */
}

int mtk_mesh_mac80211_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	/* If HW rate control is available, then this callback is required. This
	 * check is done in ieee80211_init_rate_ctrl_alg and would give a
	 * warning on it if the callback is NOT available.
	 */

	/* TODO: [mesh] */

	return 0;
}

int mtk_mesh_mac80211_mbss_beacon_update(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif)
{
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	struct BSS_INFO *prBssInfo;
	struct MESH_CONNECTION_SETTINGS *prConnSettings;
	struct MESH_INFO *prMeshInfo;
	struct MSDU_INFO *prMsduInfo;
	struct MSG_MESH_BCN_UPDATE *prMeshBcnUpdateMsg;
	struct sk_buff *beacon;
	struct ieee80211_bss_conf *bss_conf;
	struct ieee80211_mgmt *mgmt;

	DBGLOG(MESH, INFO, "beacon update\n");

	ASSERT(hw);
	ASSERT(vif);

	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	prConnSettings = prAdapter->rWifiVar.prMeshConnSettings;
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMeshInfo->ucBssIndex);
	ASSERT(prBssInfo);

	prMsduInfo = prBssInfo->prBeacon;
	ASSERT(prMsduInfo);

	prMeshBcnUpdateMsg = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
					 sizeof(struct MSG_MESH_BCN_UPDATE));
	ASSERT(prMeshBcnUpdateMsg);

	beacon = ieee80211_beacon_get(hw, vif);
	ASSERT(beacon);
	bss_conf = &vif->bss_conf;

	/* 4 <1> Configure MBSS */
	prConnSettings->u2BeaconPeriod = bss_conf->beacon_int;
	prConnSettings->ucDTIMPeriod = bss_conf->dtim_period;

	mgmt = (struct ieee80211_mgmt *) beacon->data;
	ASSERT(mgmt);
	if (mgmt->u.beacon.capab_info & WLAN_CAPABILITY_PRIVACY)
		prConnSettings->eAuthMode = AUTH_MODE_SAE;
	else
		prConnSettings->eAuthMode = AUTH_MODE_OPEN;

	/* 4 <2> Update MBSS Beacon content */
	/* Update the TSF adjust value here, the HW will add this value for
	 * every beacon.
	 * beacon: [control + duration + da + sa + bssid +seq]
	 *	   [timestamp + beacon_int + cap][IE]
	 */
	kalMemCopy(prMsduInfo->prPacket, beacon->data, beacon->len);
	prMsduInfo->u2FrameLength = beacon->len;

	/* Free the beacon skb, obtained from mac80211 */
	dev_kfree_skb_any(beacon);

	/* TODO: [mesh] Is it better to use kalIoctl instead of module msg? */
	prMeshBcnUpdateMsg->rMsgHdr.eMsgId = MID_MNY_MESH_BEACON_UPDATE;
	/* pucBuffer = prMeshBcnUpdateMsg->aucBuffer; */
	prMeshBcnUpdateMsg->u4BcnHdrLen = 0;
	prMeshBcnUpdateMsg->pucBcnHdr = NULL;
	prMeshBcnUpdateMsg->u4BcnBodyLen = 0;
	prMeshBcnUpdateMsg->pucBcnBody = NULL;
	mboxSendMsg(prAdapter, MBOX_ID_0,
		    (struct MSG_HDR *) prMeshBcnUpdateMsg, MSG_SEND_METHOD_BUF);

	return 0;
}

int mtk_mesh_mac80211_mbss_beacon_enable(struct ieee80211_hw *hw,
						 struct ieee80211_vif *vif)
{
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	struct BSS_INFO *prBssInfo;
	struct MESH_CONNECTION_SETTINGS *prConnSettings;
	struct MESH_INFO *prMeshInfo;
	struct MSDU_INFO *prMsduInfo;
	int rStatus = WLAN_STATUS_SUCCESS;
	struct sk_buff *beacon;
	struct ieee80211_bss_conf *bss_conf;
	struct ieee80211_mgmt *mgmt;
	struct cfg80211_chan_def *chandef;

	DBGLOG(MESH, INFO, "Beacon enable\n");

	ASSERT(hw);
	ASSERT(vif);

	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	prConnSettings = prAdapter->rWifiVar.prMeshConnSettings;
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMeshInfo->ucBssIndex);
	ASSERT(prBssInfo);

	prMsduInfo = prBssInfo->prBeacon;
	ASSERT(prMsduInfo);

	beacon = ieee80211_beacon_get(hw, vif);
	ASSERT(beacon);
	bss_conf = &vif->bss_conf;
	chandef = &bss_conf->chandef;

	/* 4 <1> Configure MBSS */
	prConnSettings->u2BeaconPeriod = bss_conf->beacon_int;
	prConnSettings->ucDTIMPeriod = bss_conf->dtim_period;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_80P80:
		prConnSettings->ucVhtChannelWidth = VHT_OP_CHANNEL_WIDTH_80P80;
		break;
	case NL80211_CHAN_WIDTH_160:
		prConnSettings->ucVhtChannelWidth = VHT_OP_CHANNEL_WIDTH_160;
		break;
	case NL80211_CHAN_WIDTH_80:
		prConnSettings->ucVhtChannelWidth = VHT_OP_CHANNEL_WIDTH_80;
		break;
	default:
		prConnSettings->ucVhtChannelWidth = VHT_OP_CHANNEL_WIDTH_20_40;
		break;
	}
	prConnSettings->ucVhtChannelFrequencyS1 = chandef->center_freq1;
	prConnSettings->ucVhtChannelFrequencyS2 = chandef->center_freq2;

	mgmt = (struct ieee80211_mgmt *)beacon->data;
	ASSERT(mgmt);
	if (mgmt->u.beacon.capab_info & WLAN_CAPABILITY_PRIVACY)
		prConnSettings->eAuthMode = AUTH_MODE_SAE;
	else
		prConnSettings->eAuthMode = AUTH_MODE_OPEN;

	/* 4 <2> Update MBSS Beacon content */
	/* Update the TSF adjust value here, the HW will add this value for
	 * every beacon.
	 * beacon: [control + duration + da + sa + bssid +seq]
	 *	   [timestamp + beacon_int + cap][IE]
	 */
	kalMemCopy(prMsduInfo->prPacket, beacon->data, beacon->len);
	prMsduInfo->u2FrameLength = beacon->len;

	/* Free the beacon skb, obtained from mac80211 */
	dev_kfree_skb_any(beacon);

	rStatus = wlanoidSetMbss(prAdapter);

	if (rStatus)
		DBGLOG(MESH, ERROR, "Create Mesh BSS fail err=0x%X\n", rStatus);

	return rStatus;
}

int mtk_mesh_mac80211_mbss_beacon_disable(struct ieee80211_hw *hw,
						  struct ieee80211_vif *vif)
{
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	int rStatus = WLAN_STATUS_SUCCESS;

	DBGLOG(MESH, INFO, "Beacon disable\n");

	ASSERT(hw);
	ASSERT(vif);

	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	rStatus = wlanoidSetMbssLeave(prAdapter);

	if (rStatus)
		DBGLOG(REQ, WARN, "Leave MBSS:%lx\n", rStatus);

	return rStatus;
}

void mtk_mesh_mac80211_bss_info_changed(struct ieee80211_hw *hw,
					struct ieee80211_vif *vif,
					struct ieee80211_bss_conf *bss_conf,
					u32 changed)
{
	ASSERT(hw);
	ASSERT(vif);
	ASSERT(bss_conf);

	DBGLOG(MESH, INFO, "BSS info changed=0x%X\n", changed);

	if (changed & BSS_CHANGED_ASSOC)
		DBGLOG(MESH, INFO, "BSS_CHANGED_ASSOC\n");

	if (changed & BSS_CHANGED_BEACON) {
		DBGLOG(MESH, INFO, "BSS_CHANGED_BEACON\n");
		mtk_mesh_mac80211_mbss_beacon_update(hw, vif);
	}

	if (changed & BSS_CHANGED_BEACON_ENABLED) {
		DBGLOG(MESH, INFO, "BSS_CHANGED_BEACON_ENABLED: %d\n",
			bss_conf->enable_beacon);

		if (vif->type == NL80211_IFTYPE_MESH_POINT) {
			if (bss_conf->enable_beacon)
				mtk_mesh_mac80211_mbss_beacon_enable(hw, vif);
			else
				mtk_mesh_mac80211_mbss_beacon_disable(hw, vif);
		}
	}

	if (changed & BSS_CHANGED_ERP_SLOT)
		DBGLOG(MESH, INFO, "BSS_CHANGED_ERP_SLOT\n");
}

int mtk_mesh_mac80211_add_key(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta,
			      struct ieee80211_key_conf *key)
{
	struct PARAM_KEY rKey;
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	struct MESH_INFO *prMeshInfo;
	int rStatus = WLAN_STATUS_SUCCESS;
	unsigned int u4BufLen = 0;

	DBGLOG(MESH, INFO, "Add key\n");

	ASSERT(hw);
	ASSERT(vif);
	ASSERT(key);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	kalMemZero(&rKey, sizeof(struct PARAM_KEY));

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		DBGLOG(MESH, INFO, "WEP40 key\n");
		rKey.ucCipher = CIPHER_SUITE_WEP40;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		DBGLOG(MESH, INFO, "WEP104 key\n");
		rKey.ucCipher = CIPHER_SUITE_WEP104;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		DBGLOG(MESH, INFO, "TKIP key\n");
		rKey.ucCipher = CIPHER_SUITE_TKIP;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		DBGLOG(MESH, INFO, "CCMP key\n");
		rKey.ucCipher = CIPHER_SUITE_CCMP;
		break;
	case WLAN_CIPHER_SUITE_SMS4:
		rKey.ucCipher = CIPHER_SUITE_WPI;
		DBGLOG(MESH, INFO, "WPI key\n");
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		DBGLOG(MESH, INFO, "BIP key(Offload to mac80211)\n");
		return -EOPNOTSUPP;
	default:
		DBGLOG(MESH, ERROR, "Not support key(0x%X)\n", key->cipher);
		return -EOPNOTSUPP;
	}

	if (!(key->flags & IEEE80211_KEY_FLAG_PAIRWISE)) {
		DBGLOG(MESH, INFO, "Offloading CCMP for BMC to mac80211\n");
		if (!sta)
			DBGLOG(MESH, INFO, "Own Group Key MAC="MACSTR"\n",
					MAC2STR(vif->addr));
		else
			DBGLOG(MESH, INFO, "Peer Group Key MAC="MACSTR"\n",
					MAC2STR(sta->addr));

		return -EOPNOTSUPP;
	}

	DBGLOG(MESH, INFO, "Pairwise key\n");
	DBGLOG(MESH, INFO, "Index=%d, Len=%d\n", key->keyidx,
				key->keylen);
	DBGLOG(MESH, INFO, "Peer MAC="MACSTR"\n", MAC2STR(sta->addr));

	COPY_MAC_ADDR(rKey.arBSSID, sta->addr);
	rKey.u4KeyIndex = key->keyidx;
	rKey.u4KeyIndex |= IS_UNICAST_KEY;
	rKey.u4KeyIndex |= IS_TRANSMIT_KEY;

	if (key->keylen)
		kalMemCopy(rKey.aucKeyMaterial, key->key, key->keylen);

	rKey.ucBssIdx = prMeshInfo->ucBssIndex;
	rKey.u4KeyLength = key->keylen;

	rKey.u4Length = ((unsigned long) &(((struct PARAM_KEY *)
				     0)->aucKeyMaterial)) + rKey.u4KeyLength;

	rStatus = kalIoctl(prGlueInfo, wlanoidSetAddKey, &rKey, rKey.u4Length,
			   FALSE, FALSE, TRUE, &u4BufLen);

	if (rStatus) {
		DBGLOG(MESH, ERROR, "OID add key fail err=0x%X\n", rStatus);
		return -EOPNOTSUPP;
	}

	/* TODO: [mesh] set eEncStatus according to selected cipher?
	 * when should reset it?
	 */
	prAdapter->rWifiVar.prMeshConnSettings->eEncStatus =
						ENUM_ENCRYPTION3_ENABLED;

	/* TODO: [mesh]
	 * if add key successfully, then should we setup below data types just
	 * like mtk_cfg80211_connect case?
	 * - prGlueInfo->rWpaInfo.u4CipherGroup
	 * - prGlueInfo->rWpaInfo.u4CipherPairwise
	 * - wlanoidSetEncryptionStatus, ... etc
	 */

	return 0;
}

int mtk_mesh_mac80211_delete_key(struct ieee80211_hw *hw,
				 struct ieee80211_sta *sta,
				 struct ieee80211_key_conf *key)
{
	struct PARAM_REMOVE_KEY rRemoveKey;
	struct GLUE_INFO *prGlueInfo;
	struct MESH_INFO *prMeshInfo;
	struct ADAPTER *prAdapter;
	int rStatus = WLAN_STATUS_SUCCESS;
	unsigned int u4BufLen = 0;

	DBGLOG(MESH, INFO, "Del key\n");

	ASSERT(hw);
	ASSERT(sta);
	ASSERT(key);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prGlueInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		DBGLOG(MESH, INFO, "WEP40 key\n");
		break;

	case WLAN_CIPHER_SUITE_WEP104:
		DBGLOG(MESH, INFO, "WEP104 key\n");
		break;

	case WLAN_CIPHER_SUITE_TKIP:
		DBGLOG(MESH, INFO, "TKIP key\n");
		break;

	case WLAN_CIPHER_SUITE_CCMP:
		DBGLOG(MESH, INFO, "CCMP key\n");
		break;

	case WLAN_CIPHER_SUITE_AES_CMAC:
		DBGLOG(MESH, ERROR, "Invalid Req BIP handled in mac80211");
		return -EOPNOTSUPP;

	default:
		DBGLOG(MESH, INFO, "Not support key(0x%X)\n", key->cipher);
		return -EOPNOTSUPP;
	}

	if (!(key->flags & IEEE80211_KEY_FLAG_PAIRWISE)) {
		DBGLOG(MESH, ERROR, "Invalid Req BMC CCMP handled in mac80211");
		return -EOPNOTSUPP;
	}

	DBGLOG(MESH, INFO, "Del PTK Index=%d, Key Len=%d Peer MAC="
			   MACSTR"\n",
		key->keyidx, key->keylen, MAC2STR(sta->addr));

	kalMemZero(&rRemoveKey, sizeof(struct PARAM_REMOVE_KEY));
	rRemoveKey.u4KeyIndex = key->keyidx;
	rRemoveKey.u4KeyIndex |= IS_UNICAST_KEY;
	rRemoveKey.u4KeyIndex |= IS_TRANSMIT_KEY;
	rRemoveKey.u4Length = sizeof(struct PARAM_REMOVE_KEY);
	COPY_MAC_ADDR(rRemoveKey.arBSSID, sta->addr);
	rRemoveKey.ucBssIdx = prMeshInfo->ucBssIndex;

	rStatus = kalIoctl(prGlueInfo, wlanoidSetRemoveKey, &rRemoveKey,
			   rRemoveKey.u4Length, FALSE, FALSE, TRUE, &u4BufLen);

	if (rStatus) {
		DBGLOG(MESH, ERROR, "OID del key fail err=0x%X\n", rStatus);
		return -EOPNOTSUPP;
	}

	return 0;
}

int mtk_mesh_mac80211_set_key(struct ieee80211_hw *hw,
			      enum set_key_cmd cmd,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta,
			      struct ieee80211_key_conf *key)
{
	int ret = 0;

	switch (cmd) {
	case SET_KEY:
		ret = mtk_mesh_mac80211_add_key(hw, vif, sta, key);
		break;
	case DISABLE_KEY:
		ret = mtk_mesh_mac80211_delete_key(hw, sta, key);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

int mtk_mesh_mac80211_suspend(struct ieee80211_hw *hw,
			      struct cfg80211_wowlan *wowlan)
{
	/* Maybe should be implemented after WoW in mesh is considered */

	return 0;
}

unsigned long long mtk_mesh_mac80211_get_tsf(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif)
{
	/* TODO: [mesh] implement when synchronization is considered */

	return 0;
}

int mtk_mesh_mac80211_get_survey(struct ieee80211_hw *hw,
                int idx, struct survey_info *survey)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	struct GL_MESH_INFO *prMeshInfo;
	struct wiphy *wiphy;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *chan;
	struct ieee80211_bss_conf *bss_conf;
	int ret = 0;
	struct cfg80211_chan_def *chandef;

	DBGLOG(MESH, INFO, "mtk_mesh_mac80211_get_survey, idx=%d\n", idx);

	ASSERT(hw);
	ASSERT(survey);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);
	prMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prMeshInfo);

	if (prMeshInfo && prMeshInfo->vif) {
		bss_conf = &(prMeshInfo->vif->bss_conf);
	} else {
		ret = -ENOENT;
		goto out;
	}

	chandef = &bss_conf->chandef;
	wiphy = hw->wiphy;
	ASSERT(wiphy);
	sband = wiphy->bands[KAL_BAND_2GHZ];
	ASSERT(sband);

	if (idx >= sband->n_channels) {
		idx -= sband->n_channels;
		sband = wiphy->bands[KAL_BAND_5GHZ];
	}

	if (idx >= sband->n_channels) {
		ret = -ENOENT;
		goto out;
	}

	chan = &sband->channels[idx];
	ASSERT(chan);

	memset(survey, 0, sizeof(*survey));

	survey->channel = chan;

	if (chan == chandef->chan) {
		kalSurveyDataControl(prGlueInfo, KAL_SURVEY_DATA_GET);
		kalSurveyGetNoise(prGlueInfo);
		survey->filled = SURVEY_INFO_IN_USE | SURVEY_INFO_TIME | SURVEY_INFO_TIME_BUSY
					| SURVEY_INFO_TIME_RX | SURVEY_INFO_TIME_TX | SURVEY_INFO_NOISE_DBM;

		survey->time = prGlueInfo->rGetSurveyData.u4Time;
		survey->time_busy = prGlueInfo->rGetSurveyData.u4TimeBusy;
		survey->time_rx = prGlueInfo->rGetSurveyData.u4TimeRx;
		survey->time_tx = prGlueInfo->rGetSurveyData.u4TimeTx;
		survey->noise = prGlueInfo->rGetSurveyData.icNoise;

	}

out:
	return ret;
}

int mtk_mesh_mac80211_add_chanctx(struct ieee80211_hw *hw,
				  struct ieee80211_chanctx_conf *ctx)
{
	struct GLUE_INFO *prGlueInfo = hw->priv;
	unsigned char ret = 0;
	struct mtk_chanctx_priv **ptr;
	struct MESH_CONNECTION_SETTINGS *prConnSettings;
	struct ADAPTER *prAdapter = prGlueInfo->prAdapter;

	ASSERT(hw);
	ASSERT(ctx);
	ASSERT(prGlueInfo);
	ASSERT(prAdapter);
	prConnSettings = prAdapter->rWifiVar.prMeshConnSettings;
	ASSERT(prConnSettings);

	DBGLOG(MESH, INFO, "Add ch context %dMHz/width:%d/cfreq:%d/%dMHz\n",
				ctx->def.chan->center_freq, ctx->def.width,
				ctx->def.center_freq1, ctx->def.center_freq2);

	/* TODO: [mesh] need further check kernel flow for channel MGMT */
	/* Benten uses wlanModifyChanctx for channel management in Soft AP and
	 * mesh concurrent mode. If Soft AP and mesh want to operate in
	 * different channel, then driver would block this request. But MT7668
	 * supports MCC, it seems we should NOT use wlanModifyChanctx for
	 * channel MGMT.
	 */
#if 0
	/* FIXME: check current usage of channel only one operate channel */
	ret = wlanModifyChanctx(&prGlueInfo->rmtk_chanctx_priv,
				ctx->def.chan->center_freq, 1, 1);
	if (ret != 0)
		return -ENOSPC;
#endif

	ptr = (void *) ctx->drv_priv;
	*ptr = &prGlueInfo->rmtk_chanctx_priv;
	(*ptr)->magic = 0xdeadbabe;

	/* Set the Channel From Chandef */
	getMeshChan(&ctx->def, &prConnSettings->ucAdHocChannelNum,
		    &prConnSettings->eAdHocBand, &prConnSettings->eChnlSco);

	return ret;
}

void mtk_mesh_mac80211_remove_chanctx(struct ieee80211_hw *hw,
				      struct ieee80211_chanctx_conf *ctx)
{
	struct mtk_chanctx_priv **ptr = (void *) ctx->drv_priv;

	ASSERT(ctx);

	/* FIXME: check current usage of channel only one operate channel */
	/* wlanModifyChanctx((struct mtk_chanctx_priv *)(*ptr),
	 *		     ctx->def.chan->center_freq, 1, 0);
	 */

	(*ptr)->magic = 0;
	*ptr = NULL;
}

void mtk_mesh_mac80211_change_chanctx(struct ieee80211_hw *hw,
				      struct ieee80211_chanctx_conf *ctx,
				      u32 changed)
{
	/* This callback would be triggered if any of following channel context
	 * change happens
	 * - IEEE80211_CHANCTX_CHANGE_WIDTH
	 * - IEEE80211_CHANCTX_CHANGE_RX_CHAINS
	 * - IEEE80211_CHANCTX_CHANGE_RADAR
	 * - IEEE80211_CHANCTX_CHANGE_CHANNEL
	 * - IEEE80211_CHANCTX_CHANGE_MIN_WIDTH
	 *
	 * So, if these changes are possible in mesh scenario, then we should
	 * implement it.
	 */
}

int mtk_mesh_mac80211_assign_vif_chanctx(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif,
					 struct ieee80211_chanctx_conf *ctx)
{
	return 0;
}


void mtk_mesh_mac80211_unassign_vif_chanctx(struct ieee80211_hw *hw,
					    struct ieee80211_vif *vif,
					    struct ieee80211_chanctx_conf *ctx)
{
}

int mtk_mesh_mac80211_set_tim(struct ieee80211_hw *hw,
			      struct ieee80211_sta *sta, bool set)
{
	/* TODO: [mesh] should be implemented for mesh low power */

	return 0;
}

#if CFG_SUPPORT_MESH_GET_TP
#if KERNEL_VERSION(4, 8, 0) <= CFG80211_VERSION_CODE
unsigned int mtk_mesh_mac80211_get_throughput(struct ieee80211_hw *hw,
					 struct ieee80211_sta *sta)
#else
unsigned int mtk_mesh_mac80211_get_throughput(struct ieee80211_sta *sta)
#endif
{
	struct mtk_sta_priv *prStaPriv;
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	struct MESH_INFO *prMeshInfo;
	struct STA_RECORD *prStaRec;
	/* If no data packet is transmitted until now, then we cannot estimate
	 * the expected throughput and thus would report 0 to mac80211.
	 * In this situation, mac80211 would use TX status of management frames
	 * if available to do the calculation.
	 */
	unsigned int u4Throughput = 0;	/* in Kbps */
	const unsigned short u2LinkSpeed2Throughput = 500;

	ASSERT(sta);
	prStaPriv = (struct mtk_sta_priv *) sta->drv_priv;
	ASSERT(prStaPriv);
	prGlueInfo = prStaPriv->prGlueInfo;
	/* In case the peer add is not done yet. */
	if(!prGlueInfo)
		return u4Throughput;

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	prStaRec = cnmGetStaRecByAddress(prAdapter,
					 prMeshInfo->ucBssIndex,
					 sta->addr);

	if (prStaRec) {
		if (prStaRec->ucAvePer <= PERCENTAGE_MAX)
			u4Throughput = ((unsigned int) prStaRec->u2LinkSpeed) *
				       u2LinkSpeed2Throughput *
				       (PERCENTAGE_MAX - prStaRec->ucAvePer) /
				       PERCENTAGE_MAX;
	} else
		DBGLOG(MESH, ERROR, "Cannot find this peer\n");

	DBGLOG(MESH, INFO, "Peer [" MACSTR "] throughput=%d Kbps\n",
		MAC2STR(sta->addr), u4Throughput);

	return u4Throughput;
}
#endif /* CFG_SUPPORT_MESH_GET_TP */

void mtk_mesh_mac80211_hw_scan_work(struct work_struct *work)
{
	struct GLUE_INFO *prGlueInfo;
	struct GL_MESH_INFO *prMeshInfo;
	struct ieee80211_hw *hw;
	int rStatus = WLAN_STATUS_SUCCESS;
	unsigned int u4BufLen;

	prMeshInfo = container_of(work, struct GL_MESH_INFO, hw_scan.work);
	ASSERT(prMeshInfo);

	hw = prMeshInfo->hw;
	ASSERT(hw);
	prGlueInfo = hw->priv;
	ASSERT(prGlueInfo);

	DBGLOG(MESH, INFO, "Scan work...\n");

	rStatus = kalIoctl(prGlueInfo, wlanoidSetBssidListScanExtMesh,
			   &prMeshInfo->rScanRequest,
			   sizeof(struct PARAM_SCAN_REQUEST_EXT), FALSE, FALSE,
			   FALSE, &u4BufLen);

	if (rStatus)
		DBGLOG(MESH, ERROR, "scan error:0x%x\n", rStatus);
}

void getMeshChan(struct cfg80211_chan_def *chandef, unsigned char *channum,
		 enum ENUM_BAND *prBand, enum ENUM_CHNL_EXT *prChnlSco)
{
	struct ieee80211_channel *channel = chandef->chan;

	DBGLOG(MESH, INFO, "Apply channel info from upper layer\n");

	ASSERT(chandef);
	ASSERT(channum);
	ASSERT(prBand);
	ASSERT(prChnlSco);
	ASSERT(channel);

	/*SCO*/
	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
		*prChnlSco = CHNL_EXT_SCN;
		break;
	case NL80211_CHAN_WIDTH_40:
		*prChnlSco = channel->center_freq > chandef->center_freq1 ?
			     CHNL_EXT_SCB : CHNL_EXT_SCA;
		break;
	default:
		/* TODO: [mesh] handle the rest case */
		*prChnlSco = CHNL_EXT_SCN;
		break;
	}

	switch (channel->band) {
	case KAL_BAND_2GHZ:
		*prBand = BAND_2G4;
		break;
	case KAL_BAND_5GHZ:
		*prBand = BAND_5G;
		break;
	default:
		ASSERT(0);
		break;
	}

	*channum = ieee80211_frequency_to_channel(channel->center_freq);
}
#endif	/* CFG_ENABLE_WIFI_MESH */
