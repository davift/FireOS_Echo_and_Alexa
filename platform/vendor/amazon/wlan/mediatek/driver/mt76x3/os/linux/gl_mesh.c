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
/*! gl_mesh.c
* Main routines of Linux driver interface for Wi-Fi MESH
*
* This file contains the main routines of Linux driver for MediaTek Inc. 802.11
* Wireless LAN Adapters.
*/

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include "precomp.h"
#include "gl_mesh_mac80211.h"
#include "gl_mesh_os.h"
#include "gl_cfg80211.h"

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
static const struct ieee80211_ops ieee80211_mtk_mesh_ops = {
	.tx			       = mtk_mesh_mac80211_tx,
	.start			       = mtk_mesh_mac80211_start,
	.stop			       = mtk_mesh_mac80211_stop,
	.add_interface		       = mtk_mesh_mac80211_add_interface,
	.remove_interface	       = mtk_mesh_mac80211_remove_interface,
	.config			       = mtk_mesh_mac80211_config,
	.configure_filter	       = mtk_mesh_mac80211_configure_filter,
	.sta_add		       = mtk_mesh_mac80211_sta_add,
	.sta_remove		       = mtk_mesh_mac80211_sta_remove,
	.sta_notify		       = mtk_mesh_mac80211_sta_notify,
	.hw_scan		       = mtk_mesh_mac80211_hw_scan,
	.cancel_hw_scan		       = mtk_mesh_mac80211_cancel_hw_scan,
	.set_rts_threshold	       = mtk_mesh_mac80211_set_rts_threshold,
	.bss_info_changed	       = mtk_mesh_mac80211_bss_info_changed,
	.set_key		       = mtk_mesh_mac80211_set_key,
	.suspend		       = mtk_mesh_mac80211_suspend,
	.get_tsf		       = mtk_mesh_mac80211_get_tsf,
	.get_survey		       = mtk_mesh_mac80211_get_survey,
	.add_chanctx		       = mtk_mesh_mac80211_add_chanctx,
	.remove_chanctx		       = mtk_mesh_mac80211_remove_chanctx,
	.change_chanctx		       = mtk_mesh_mac80211_change_chanctx,
	.assign_vif_chanctx	       = mtk_mesh_mac80211_assign_vif_chanctx,
	.unassign_vif_chanctx	       = mtk_mesh_mac80211_unassign_vif_chanctx,
	.set_tim		       = mtk_mesh_mac80211_set_tim,
#if CFG_SUPPORT_MESH_GET_TP
	.get_expected_throughput       = mtk_mesh_mac80211_get_throughput,
#endif
};

static const struct ieee80211_iface_limit mtk_if_limits[] = {
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_STATION),
	},
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_MESH_POINT),
	},
};

/* check in static int wiphy_verify_combinations
 * @limits: limits for the given interface types
 * @n_limits: number of limitations
 * @num_different_channels: can use up to this many different channels
 * @max_interfaces: maximum number of interfaces in total allowed in this group
 * @beacon_int_infra_match: In this combination, the beacon intervals between
 *			infrastructure and AP types must match. This is required
 *			only in special cases.
 * @radar_detect_widths: bitmap of channel widths supported for radar detection
 */
static const struct ieee80211_iface_combination mtk_if_comb[] = {
	{
		.limits = mtk_if_limits,
		.n_limits = ARRAY_SIZE(mtk_if_limits),
		 /* Combinations with just one interface aren't real */
		.max_interfaces = 2,
		/* Need at least one channel */
		.num_different_channels = 1,
		.beacon_int_infra_match = true,
#if 0
		.radar_detect_widths = BIT(NL80211_CHAN_WIDTH_20_NOHT) |
				       BIT(NL80211_CHAN_WIDTH_20) |
				       BIT(NL80211_CHAN_WIDTH_40) |
				       BIT(NL80211_CHAN_WIDTH_80),
#endif
	},
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

/*----------------------------------------------------------------------------*/
/*!
* \brief Allocate memory for MESHINFO, GL_MESH_INFO_T, MESH_CONNECTION_SETTINGS
*                                          MESH_SPECIFIC_BSS_INFO, MESH_FSM_INFO
*
* \param[in] prGlueInfo      Pointer to glue info
*
* \return   TRUE
*           FALSE
*/
/*----------------------------------------------------------------------------*/
static bool meshAllocInfo(IN struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	struct WIFI_VAR *prWifiVar;
	struct GL_MESH_INFO **pprGlMeshInfo;
	struct MESH_INFO **pprMeshInfo;
	struct MESH_CONNECTION_SETTINGS **pprMeshConnSet;
	struct MESH_SPECIFIC_BSS_INFO **pprMeshSpecBssInfo;

	ASSERT(prGlueInfo);

	prAdapter = prGlueInfo->prAdapter;
	pprGlMeshInfo = &prGlueInfo->prMeshInfo;

	ASSERT(prAdapter);

	prWifiVar = &prAdapter->rWifiVar;
	pprMeshInfo = &prAdapter->prMeshInfo;

	ASSERT(prWifiVar);

	pprMeshConnSet = &(prWifiVar->prMeshConnSettings);
	pprMeshSpecBssInfo = &(prWifiVar->prMeshSpecificBssInfo);

	if (!*pprGlMeshInfo) {
		/* alloc mem for mesh info */
		*pprGlMeshInfo = kalMemAlloc(sizeof(struct GL_MESH_INFO),
					     VIR_MEM_TYPE);
		if (!*pprGlMeshInfo)
			goto fail;

		*pprMeshInfo = kalMemAlloc(sizeof(struct MESH_INFO),
					     VIR_MEM_TYPE);
		if (!*pprMeshInfo)
			goto fail;

		*pprMeshConnSet =
			kalMemAlloc(sizeof(struct MESH_CONNECTION_SETTINGS),
				    VIR_MEM_TYPE);
		if (!*pprMeshConnSet)
			goto fail;

		*pprMeshSpecBssInfo =
			kalMemAlloc(sizeof(struct MESH_SPECIFIC_BSS_INFO),
				    VIR_MEM_TYPE);
		if (!*pprMeshSpecBssInfo)
			goto fail;
	} else {
		ASSERT(*pprMeshInfo);
		ASSERT(*pprMeshConnSet);
		ASSERT(*pprMeshSpecBssInfo);
	}

	/* MUST set memory to 0 */
	kalMemZero(*pprGlMeshInfo, sizeof(struct GL_MESH_INFO));
	kalMemZero(*pprMeshInfo, sizeof(struct MESH_INFO));
	kalMemZero(*pprMeshConnSet, sizeof(struct MESH_CONNECTION_SETTINGS));
	kalMemZero(*pprMeshSpecBssInfo,
		   sizeof(struct MESH_SPECIFIC_BSS_INFO));

	return TRUE;

fail:
	if (*pprGlMeshInfo) {
		kalMemFree(*pprGlMeshInfo, VIR_MEM_TYPE,
			   sizeof(struct GL_MESH_INFO));
		*pprGlMeshInfo = NULL;
	}

	if (*pprMeshInfo) {
		kalMemFree(*pprMeshInfo, VIR_MEM_TYPE,
			   sizeof(struct MESH_INFO));
		*pprMeshInfo = NULL;
	}

	if (*pprMeshConnSet) {
		kalMemFree(*pprMeshConnSet, VIR_MEM_TYPE,
			   sizeof(struct MESH_CONNECTION_SETTINGS));
		*pprMeshConnSet = NULL;
	}

	if (*pprMeshSpecBssInfo) {
		kalMemFree(*pprMeshSpecBssInfo, VIR_MEM_TYPE,
			   sizeof(struct MESH_SPECIFIC_BSS_INFO));
		*pprMeshSpecBssInfo = NULL;
	}

	return FALSE;
}

/*----------------------------------------------------------------------------*/
/*!
* \brief Free memory for prMESHInfo
*
* \param[in] prGlueInfo      Pointer to glue info
*
* \return   TRUE
*           FALSE
*/
/*----------------------------------------------------------------------------*/
static bool meshFreeInfo(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	struct GL_MESH_INFO **pprGlMeshInfo;
	struct MESH_INFO **pprMeshInfo;
	struct MESH_CONNECTION_SETTINGS **pprConnSetting;
	struct MESH_SPECIFIC_BSS_INFO **pprSpecificBssInfo;
	bool ret = TRUE;

	ASSERT(prGlueInfo);

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	pprGlMeshInfo = &prGlueInfo->prMeshInfo;
	pprMeshInfo = &prAdapter->prMeshInfo;
	pprConnSetting = &prAdapter->rWifiVar.prMeshConnSettings;
	pprSpecificBssInfo = &prAdapter->rWifiVar.prMeshSpecificBssInfo;

	kalMemFree(*pprGlMeshInfo, VIR_MEM_TYPE, sizeof(struct GL_MESH_INFO));
	kalMemFree(*pprMeshInfo, VIR_MEM_TYPE, sizeof(struct MESH_INFO));
	kalMemFree(*pprConnSetting, VIR_MEM_TYPE,
		   sizeof(struct MESH_CONNECTION_SETTINGS));
	kalMemFree(*pprSpecificBssInfo, VIR_MEM_TYPE,
		   sizeof(struct MESH_SPECIFIC_BSS_INFO));

	/*reset all pointer to NULL */
	*pprGlMeshInfo = NULL;
	*pprMeshInfo = NULL;
	*pprConnSetting = NULL;
	*pprSpecificBssInfo = NULL;

	return ret;
}

static void glMeshHtCapInit(struct WIFI_VAR *prWifiVar,
			    struct ieee80211_sta_ht_cap *ht_cap)
{
	struct ieee80211_mcs_info *mcs = &ht_cap->mcs;
	uint32_t i;

	/* General rule applied here is
	 * - If HW/FW does NOT support the feature, then do NOT report the
	 *   capability. Otherwise,
	 * - If wifi.cfg disable the feature, then do NOT report the capability.
	 *   Otherwise, report the maximum capability.
	 */

	ASSERT(prWifiVar);
	ASSERT(ht_cap);
	kalMemZero(ht_cap, sizeof(struct ieee80211_sta_ht_cap));

	if (IS_FEATURE_ENABLED(prWifiVar->ucRxLdpc))
		ht_cap->cap |= HT_CAP_INFO_LDPC_CAP;
	if (prWifiVar->ucStaBandwidth >= MAX_BW_40MHZ)
		ht_cap->cap |= HT_CAP_INFO_SUP_CHNL_WIDTH;
	ht_cap->cap |= HT_CAP_INFO_SM_POWER_SAVE_DISABLE;
	/* Use wpa_supplicant "disable_sgi" for controlling this feature. */
	ht_cap->cap |= HT_CAP_INFO_SHORT_GI_20M;
	ht_cap->cap |= HT_CAP_INFO_SHORT_GI_40M;
	if (IS_FEATURE_ENABLED(prWifiVar->ucTxStbc))
		ht_cap->cap |= HT_CAP_INFO_TX_STBC;
	/* TODO: [mesh] This code makes assumption that ucRxStbcNss is correct.
	 *	 Unfortunately, ucRxStbcNss depends lots of other things, ex:
	 *	 prWifiVar->ucNss, wlanGetSupportNss.
	 */
	if (IS_FEATURE_ENABLED(prWifiVar->ucRxStbc))
		ht_cap->cap |= (prWifiVar->ucRxStbcNss <<
				HT_CAP_INFO_RX_STBC_OFFSET);
	ht_cap->cap |= HT_CAP_INFO_MAX_AMSDU_LEN;

	ht_cap->ht_supported = TRUE;

	/* align STA settings */
	ht_cap->ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
	ht_cap->ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;

	for (i = 0; i < prWifiVar->ucNSS; i++)
		mcs->rx_mask[i] = 0xff;

	mcs->rx_highest = 0;

	mcs->tx_params |= IEEE80211_HT_MCS_TX_DEFINED;
}

static void glMeshVhtCapInit(struct WIFI_VAR *prWifiVar,
			     struct ieee80211_sta_vht_cap *vht_cap)
{
	struct ieee80211_vht_mcs_info *mcs = &vht_cap->vht_mcs;
	uint32_t i;
	const uint8_t ucMcsMapStrOfst[] = {
		RX_MCS_MAP_STR1_OFST,
		RX_MCS_MAP_STR2_OFST,
		RX_MCS_MAP_STR3_OFST,
		RX_MCS_MAP_STR4_OFST,
		RX_MCS_MAP_STR5_OFST,
		RX_MCS_MAP_STR6_OFST,
		RX_MCS_MAP_STR7_OFST,
		RX_MCS_MAP_STR8_OFST,
	};
	const uint8_t uc80211acStrMax = sizeof(ucMcsMapStrOfst) / sizeof(uint8_t);

	/* General rule applied here is
	 * - If HW/FW does NOT support the feature, then do NOT report the
	 *   capability. Otherwise,
	 * - If wifi.cfg disable the feature, then do NOT report the capability.
	 *   Otherwise, report the maximum capability.
	 */

	ASSERT(prWifiVar);
	ASSERT(vht_cap);
	kalMemZero(vht_cap, sizeof(struct ieee80211_sta_vht_cap));

	if (!prWifiVar->ucStaVht)
		return;

	vht_cap->vht_supported = TRUE;

	vht_cap->cap |= IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_7991;
	if (IS_FEATURE_ENABLED(prWifiVar->ucRxLdpc))
		vht_cap->cap |= IEEE80211_VHT_CAP_RXLDPC;
	/* Use wpa_supplicant "disable_sgi" for controlling this feature. */
	vht_cap->cap |= IEEE80211_VHT_CAP_SHORT_GI_80;
	if (IS_FEATURE_ENABLED(prWifiVar->ucTxStbc))
		vht_cap->cap |= IEEE80211_VHT_CAP_TXSTBC;
	/* TODO: [mesh] This code makes assumption that ucRxStbcNss is correct.
	 *	 Unfortunately, ucRxStbcNss depends lots of other things, ex:
	 *	 prWifiVar->ucNss, wlanGetSupportNss.
	 */
	if (IS_FEATURE_ENABLED(prWifiVar->ucRxStbc))
		vht_cap->cap |= (prWifiVar->ucRxStbcNss <<
				 VHT_CAP_INFO_RX_STBC_OFFSET);
	if (IS_FEATURE_ENABLED(prWifiVar->ucStaVhtBfer)) {
		vht_cap->cap |= IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE;
		vht_cap->cap |=
			VHT_CAP_INFO_NUMBER_OF_SOUNDING_DIMENSIONS_2_SUPPORTED;
	}
	if (IS_FEATURE_ENABLED(prWifiVar->ucStaVhtBfee)) {
		vht_cap->cap |= IEEE80211_VHT_CAP_SU_BEAMFORMEE_CAPABLE;
		vht_cap->cap |=
			VHT_CAP_INFO_COMPRESSED_STEERING_NUMBER_OF_BEAMFORMER_ANTENNAS_4_SUP;
	}
	vht_cap->cap |= (AMPDU_PARAM_MAX_AMPDU_LEN_1024K <<
			 VHT_CAP_INFO_MAX_AMPDU_LENGTH_OFFSET);

	for (i = 0; i < prWifiVar->ucNSS; i++) {
		mcs->rx_mcs_map |= (IEEE80211_VHT_MCS_SUPPORT_0_9 <<
				    ucMcsMapStrOfst[i]);
		mcs->tx_mcs_map |= (IEEE80211_VHT_MCS_SUPPORT_0_9 <<
				    ucMcsMapStrOfst[i]);
	}
	for (i = prWifiVar->ucNSS; i < uc80211acStrMax; i++) {
		mcs->rx_mcs_map |= (IEEE80211_VHT_MCS_NOT_SUPPORTED <<
				    ucMcsMapStrOfst[i]);
		mcs->tx_mcs_map |= (IEEE80211_VHT_MCS_NOT_SUPPORTED <<
				    ucMcsMapStrOfst[i]);
	}

	mcs->rx_highest = 0;
	mcs->tx_highest = 0;
}

static void glMesh2gBandCapInit(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	struct WIFI_VAR *prWifiVar;
	struct ieee80211_sta_ht_cap *ht_cap;

	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prWifiVar = &prAdapter->rWifiVar;
	ht_cap = &mtk_mesh_band_2ghz.ht_cap;

	glMeshHtCapInit(prWifiVar, ht_cap);
}

static void glMesh5gBandCapInit(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	struct WIFI_VAR *prWifiVar;
	struct ieee80211_sta_ht_cap *ht_cap;
	struct ieee80211_sta_vht_cap *vht_cap;

	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prWifiVar = &prAdapter->rWifiVar;
	ht_cap = &mtk_mesh_band_5ghz.ht_cap;
	vht_cap = &mtk_mesh_band_5ghz.vht_cap;

	glMeshHtCapInit(prWifiVar, ht_cap);
	glMeshVhtCapInit(prWifiVar, vht_cap);
}

static void glMesh2g5gBandCapInit(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;

	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	glMesh2gBandCapInit(prGlueInfo);
	if (!prAdapter->fgIsHw5GBandDisabled)
		glMesh5gBandCapInit(prGlueInfo);
}

/*----------------------------------------------------------------------------*/
/*!
* \brief Register Net Device for Wi-Fi MESH
*
* \param[in] prGlueInfo      Pointer to glue info
*
* \return   TRUE
*           FALSE
*/
/*----------------------------------------------------------------------------*/
int32_t glRegisterMESH(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct ieee80211_hw *hw;
	struct wiphy *wiphy;
	int32_t ret = 0;
	uint8_t aucOwnMacAddr[MAC_ADDR_LEN];
	struct mt66xx_chip_info *prChipInfo;

	//GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	prChipInfo = prAdapter->chip_info;
	ASSERT(prChipInfo);

	DBGLOG(MESH, INFO, "STEP 1...\n");

	/* 1. allocate meshinfo */
	if (!meshAllocInfo(prGlueInfo)) {
        //GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
		DBGLOG(MESH, ERROR, "Mesh Allocate Info Fail...\n");
		ret = -ENOMEM;
		goto fail;
	}

	DBGLOG(MESH, INFO, "STEP 2...\n");
	//GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);
	DBGLOG(MESH, INFO, "STEP 3...\n");

	/* 2. allocate ieee80211_hw(wiphy->ieee80211_local->ieee80211_hw) */
#if KERNEL_VERSION(3, 19, 0) <= CFG80211_VERSION_CODE
	hw = ieee80211_alloc_hw_nm(sizeof(struct GLUE_INFO),
				   &ieee80211_mtk_mesh_ops,
				   MAC80211_WIPHY_NAME);
#else
	hw = ieee80211_alloc_hw(sizeof(struct GLUE_INFO), &ieee80211_mtk_mesh_ops);
#endif	/* CFG80211_VERSION_CODE */
	DBGLOG(MESH, INFO, "STEP 4...\n");

	if (!hw) {
		DBGLOG(MESH, ERROR, "ieee80211_alloc_hw...fail\n");
		ret = -ENOMEM;
		goto fail;
	}

	/* 2.1 fill hw and wiphy parameters */
	DBGLOG(MESH, INFO, "Fill ieee80211_hw and wiphy\n");
	hw->priv = prGlueInfo;
	prGlueInfo->prMeshInfo->hw = hw;
	wiphy = hw->wiphy;
	ASSERT(wiphy);

/* TODO: [mesh]
 * NOT need
 * - IEEE80211_HW_RX_INCLUDES_FCS
 * - IEEE80211_HW_SIGNAL_UNSPEC
 * - IEEE80211_HW_NEED_DTIM_BEFORE_ASSOC
 * - IEEE80211_HW_WANT_MONITOR_VIF
 * - IEEE80211_HW_SW_CRYPTO_CONTROL
 * - IEEE80211_HW_SUPPORT_FAST_XMIT
 * - IEEE80211_HW_AP_LINK_PS
 * - IEEE80211_HW_SUPPORTS_RC_TABLE
 * - IEEE80211_HW_P2P_DEV_ADDR_FOR_INTF
 * - IEEE80211_HW_TIMING_BEACON_ONLY
 * - IEEE80211_HW_CHANCTX_STA_CSA
 * - IEEE80211_HW_SPECTRUM_MGMT
 * - IEEE80211_HW_SUPPORTS_CLONED_SKBS
 * - IEEE80211_HW_TDLS_WIDER_BW
 * - IEEE80211_HW_SUPPORTS_AMSDU_IN_AMPDU
 *
 *
 * Currently NOT implement and should be reviewed in the future
 * - IEEE80211_HW_SUPPORTS_PS
 * - IEEE80211_HW_SUPPORTS_DYNAMIC_PS
 * - IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING
 * - IEEE80211_HW_PS_NULLFUNC_STACK
 * - IEEE80211_HW_NO_AUTO_VIF
 * - IEEE80211_HW_SINGLE_SCAN_ON_ALL_BANDS
 * - IEEE80211_HW_BEACON_TX_STATUS
 */
	DBGLOG(MESH, INFO, "STEP 5...\n");
#if KERNEL_VERSION(4, 2, 0) <= LINUX_VERSION_CODE
	ieee80211_hw_set(hw, HAS_RATE_CONTROL);
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, AMPDU_AGGREGATION);
	ieee80211_hw_set(hw, MFP_CAPABLE);
	ieee80211_hw_set(hw, CONNECTION_MONITOR);
	ieee80211_hw_set(hw, QUEUE_CONTROL);
	ieee80211_hw_set(hw, SUPPORTS_PER_STA_GTK);
	ieee80211_hw_set(hw, TX_AMPDU_SETUP_IN_HW);
	ieee80211_hw_set(hw, SUPPORTS_HT_CCK_RATES);
	ieee80211_hw_set(hw, NO_AUTO_VIF);

#if CFG_SUPPORT_MESH_REPORT_TXS
	ieee80211_hw_set(hw, REPORTS_TX_ACK_STATUS);
#endif

#else
	hw->flags |= IEEE80211_HW_HAS_RATE_CONTROL |
		     IEEE80211_HW_SIGNAL_DBM |
		     IEEE80211_HW_AMPDU_AGGREGATION |
		     IEEE80211_HW_MFP_CAPABLE |
#if CFG_SUPPORT_MESH_REPORT_TXS
		     IEEE80211_HW_REPORTS_TX_ACK_STATUS |
#endif
		     IEEE80211_HW_CONNECTION_MONITOR |
		     IEEE80211_HW_QUEUE_CONTROL |
		     IEEE80211_HW_SUPPORTS_PER_STA_GTK |
		     IEEE80211_HW_TX_AMPDU_SETUP_IN_HW |
		     IEEE80211_HW_SUPPORTS_HT_CCK_RATES;

#endif

#if 0 /* special for benten? */
	hw->flags2 = IEEE80211_HW_CLOCK_SYNC |
		     IEEE80211_HW_MESH_BEACON_RX_FILTERING |
		     IEEE80211_HW_PROBE_RESPONSE_OFFLOAD |
		     IEEE80211_HW_MESH_RADIO_POWER_SAVE;
#endif

	DBGLOG(MESH, INFO, "STEP 6...\n");
	wiphy->n_addresses = 1;
	/* set address */
	COPY_MAC_ADDR(aucOwnMacAddr, prAdapter->rMyMacAddr);
	/* may not need replaced by add_interface */
	if (aucOwnMacAddr[0] & 0x4)
		aucOwnMacAddr[0] &= ~0x4;
	else
		aucOwnMacAddr[0] |= 0x4;
	SET_IEEE80211_PERM_ADDR(hw, aucOwnMacAddr);
	DBGLOG(MESH, INFO, "STEP 7...\n");

	/* set band */
	glMesh2g5gBandCapInit(prGlueInfo);
	DBGLOG(MESH, INFO, "STEP 8...\n");
	wiphy->bands[KAL_BAND_2GHZ] = &mtk_mesh_band_2ghz;
	if (!prAdapter->fgIsHw5GBandDisabled)
		wiphy->bands[KAL_BAND_5GHZ] = &mtk_mesh_band_5ghz;

	/* set reg */
	wiphy->regulatory_flags = REGULATORY_CUSTOM_REG;

	/* Allow NL80211_IFTYPE_STATION for the sake of wpa_supplicant. */
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				     BIT(NL80211_IFTYPE_MESH_POINT);
	/* set if combination */
	wiphy->iface_combinations = mtk_if_comb;
	wiphy->n_iface_combinations = ARRAY_SIZE(mtk_if_comb);

	/* set scan cap */
	wiphy->max_scan_ssids = 1;    /* FIXME: for combo scan */
	wiphy->max_scan_ie_len = 512;
	hw->vif_data_size = sizeof(struct mtk_vif_priv);
	hw->sta_data_size = sizeof(struct mtk_sta_priv);
	hw->chanctx_data_size = sizeof(void *);
	/* TODO: [mesh] check kernel DOC:HW queue control for correct setting */
	/* set hw queue */
	hw->queues = 4;
	/* @offchannel_tx_hw_queue: HW queue ID to use for offchannel TX
	 * (if IEEE80211_HW_QUEUE_CONTROL is set)
	 */
	hw->offchannel_tx_hw_queue = 3;

	/* TODO: [mesh] check kernel wiphy_flags for correct setting */
	/* set chipher suit */
	wiphy->flags |= WIPHY_FLAG_IBSS_RSN;

	/* add extra tx headrrom for data */
	hw->extra_tx_headroom = KAL_GET_MAX(NIC_TX_DESC_AND_PADDING_LENGTH
					+ prAdapter->chip_info->txd_append_size,
					sizeof(struct ieee80211_tx_info));

	DBGLOG(MESH, INFO, "STEP 9...\n");
	cfg80211_regd_set_wiphy(wiphy);

	/* 3. register ieee80211 hw */
	DBGLOG(MESH, INFO, "Call ieee80211_register_hw\n");

	ret = ieee80211_register_hw(hw);
	if (ret) {
		DBGLOG(MESH, ERROR, "ieee80211_register_hw fail... ret 0x%x\n",
		       ret);
		ieee80211_free_hw(hw);

		goto fail;
	}

	prAdapter->fgIsMeshRegister = TRUE;

	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	DBGLOG(MESH, INFO, "STEP10...\n");
	init_waitqueue_head(&prGlMeshInfo->mesh_wait);

	//GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

	DBGLOG(MESH, INFO, "STEP11...\n");
	return ret;

fail:
	prAdapter->fgIsMeshRegister = FALSE;

	DBGLOG(MESH, INFO, "STEP FAIL...\n");
	//GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

	return ret;
}

/*----------------------------------------------------------------------------*/
/*!
* \brief Unregister Net Device for Wi-Fi MESH
*
* \param[in] prGlueInfo      Pointer to glue info
*
* \return   TRUE
*           FALSE
*/
/*----------------------------------------------------------------------------*/
int32_t glUnregisterMESH(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;

	//GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	//GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

	if (kalIsResetting()) {
		UNSET_NET_ACTIVE(prAdapter, prMeshInfo->ucBssIndex);
#if CFG_SUPPORT_MESH_GET_TP
		flush_delayed_work(&prGlMeshInfo->rGetLinkSpeed);
#endif /* CFG_SUPPORT_MESH_GET_TP */
		cnmTimerStopTimer(prAdapter, &(prMeshInfo->rFlushMsduInfoTimer));
	}

	ieee80211_unregister_hw(prGlMeshInfo->hw);
	ieee80211_free_hw(prGlMeshInfo->hw);
	meshFreeInfo(prGlueInfo);

	prAdapter->fgIsMeshRegister = FALSE;

	//GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_NET_DEV);

	return 0;
}
#endif	/* CFG_ENABLE_WIFI_MESH */
