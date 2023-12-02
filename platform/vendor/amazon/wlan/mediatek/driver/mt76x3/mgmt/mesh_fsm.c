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

/*! \file   "mesh_fsm.c"
 *  \brief  This file defines the FSM for Mesh Module.
 *
 *  This file defines the FSM for Mesh Module.
 */

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"

#if CFG_ENABLE_WIFI_MESH
/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */
static uint8_t *apucDebugMeshState[MESH_STATE_NUM + 1] = {
	(uint8_t *) DISP_STRING("MESH_STATE_DISABLE"),
	(uint8_t *) DISP_STRING("MESH_STATE_IDLE"),
	(uint8_t *) DISP_STRING("MESH_STATE_SCAN"),
	(uint8_t *) DISP_STRING("MESH_STATE_ONLINE_SCAN"),
	(uint8_t *) DISP_STRING("MESH_STATE_MBSS_INIT"),
	(uint8_t *) DISP_STRING("MESH_STATE_MBSS_ACTIVATE"),
	(uint8_t *) DISP_STRING("MESH_STATE_MBSS_LEAVE"),
	(uint8_t *) DISP_STRING("MESH_STATE_INVALID")
};

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */
void meshFuncAcquireChnl(IN struct ADAPTER *prAdapter, IN uint8_t ucBssIdx,
			 IN struct MESH_CHNL_REQ_INFO *prChnlReqInfo)
{
	struct MSG_CH_REQ *prMsgChReq;

	ASSERT(prAdapter);
	ASSERT(prChnlReqInfo);

	/* send message to CNM for acquiring channel */
	prMsgChReq = cnmMemAlloc(prAdapter, RAM_TYPE_MSG, sizeof(struct MSG_CH_REQ));

	ASSERT(prMsgChReq);

	meshFuncAbortChnl(prAdapter, ucBssIdx, prChnlReqInfo);

	prMsgChReq->rMsgHdr.eMsgId = MID_MNY_CNM_CH_REQ;
	prMsgChReq->ucBssIndex = ucBssIdx;
	prMsgChReq->ucTokenID = ++prChnlReqInfo->ucSeqNumOfChReq;
	prMsgChReq->eReqType = prChnlReqInfo->eChnlReqType;
	prMsgChReq->u4MaxInterval = prChnlReqInfo->u4MaxInterval;
	prMsgChReq->ucPrimaryChannel = prChnlReqInfo->ucReqChnlNum;
	prMsgChReq->eRfSco = prChnlReqInfo->eChnlSco;
	prMsgChReq->eRfBand = prChnlReqInfo->eBand;
	prMsgChReq->eRfChannelWidth = prChnlReqInfo->eChannelWidth;
	prMsgChReq->ucRfCenterFreqSeg1 = prChnlReqInfo->ucCenterFreqS1;
	prMsgChReq->ucRfCenterFreqSeg2 = prChnlReqInfo->ucCenterFreqS2;
#if CFG_SUPPORT_DBDC
	prMsgChReq->eDBDCBand = ENUM_BAND_AUTO;
#endif /*CFG_SUPPORT_DBDC*/
	/* Channel request join BSSID. */

	mboxSendMsg(prAdapter, MBOX_ID_0, (struct MSG_HDR *) prMsgChReq,
		    MSG_SEND_METHOD_BUF);

	prChnlReqInfo->fgIsChannelRequested = TRUE;
}

void meshFuncAbortChnl(IN struct ADAPTER *prAdapter, IN uint8_t ucBssIdx,
		       IN struct MESH_CHNL_REQ_INFO *prChnlReqInfo)
{
	struct MSG_CH_ABORT *prMsgChAbort = NULL;

	ASSERT(prAdapter);
	ASSERT(prChnlReqInfo);

	if (!prChnlReqInfo->fgIsChannelRequested) {
		DBGLOG(MESH, ERROR, "Cannot abort due to NO ch req issued\n");
		return;
	}

	/* return channel privilege to CNM immediately */
	prMsgChAbort = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
				   sizeof(struct MSG_CH_ABORT));

	ASSERT(prMsgChAbort);

	prChnlReqInfo->fgIsChannelRequested = FALSE;

	prMsgChAbort->rMsgHdr.eMsgId = MID_MNY_CNM_CH_ABORT;
	prMsgChAbort->ucBssIndex = ucBssIdx;
	prMsgChAbort->ucTokenID = prChnlReqInfo->ucSeqNumOfChReq++;
#if CFG_SUPPORT_DBDC
	prMsgChAbort->eDBDCBand = ENUM_BAND_AUTO;
#endif /*CFG_SUPPORT_DBDC*/

	mboxSendMsg(prAdapter, MBOX_ID_0, (struct MSG_HDR *) prMsgChAbort,
		    MSG_SEND_METHOD_BUF);
}

uint32_t meshFuncTxMgmtFrame(IN struct ADAPTER *prAdapter,
				IN struct MSDU_INFO *prMgmtTxMsdu)
{
	uint32_t rWlanStatus = WLAN_STATUS_SUCCESS;
	uint8_t ucRetryLimit = 3;	/* TX_DESC_TX_COUNT_NO_LIMIT; */

	ASSERT(prAdapter);
	ASSERT(prMgmtTxMsdu);

/* previous settings will be replaced, which may lead the flow become chaos */
#if 0
	TX_SET_MMPDU(prAdapter, prMgmtTxMsdu, prMgmtTxMsdu->ucBssIndex,
		     prMgmtTxMsdu->ucStaRecIndex, WLAN_MAC_MGMT_HEADER_LEN,
		     prMgmtTxMsdu->u2FrameLength, prMgmtTxMsdu->pfTxDoneHandler,
		     MSDU_RATE_MODE_AUTO);
#endif

	nicTxConfigPktControlFlag(prMgmtTxMsdu, MSDU_CONTROL_FLAG_FORCE_TX,
				  TRUE);
	nicTxSetPktRetryLimit(prMgmtTxMsdu, ucRetryLimit);
	/* send to TX queue */
	nicTxEnqueueMsdu(prAdapter, prMgmtTxMsdu);

	return rWlanStatus;
}

void meshPrepReqChnl(IN struct ADAPTER *prAdapter, IN struct BSS_INFO *prBssInfo,
		     OUT struct MESH_CHNL_REQ_INFO *prChnlReqInfo)
{
	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prChnlReqInfo);

	prChnlReqInfo->u8Cookie = 0;
	prChnlReqInfo->ucReqChnlNum = prBssInfo->ucPrimaryChannel;
	prChnlReqInfo->eBand = prBssInfo->eBand;
	prChnlReqInfo->eChnlSco = prBssInfo->eBssSCO;
	prChnlReqInfo->u4MaxInterval = MESH_CHNL_HOLD_TIME_MS;
	/* TODO: [mesh] */
	prChnlReqInfo->eChnlReqType = CH_REQ_TYPE_GO_START_BSS;

	prChnlReqInfo->eChannelWidth = prBssInfo->ucVhtChannelWidth;
	if (prChnlReqInfo->eChannelWidth == VHT_OP_CHANNEL_WIDTH_80P80) {
		/* TODO: BW80+80 support */
		DBGLOG(MESH, WARN,
		       "BW80+80 not support. Fallback to BW20/40\n");
		prChnlReqInfo->eChannelWidth = VHT_OP_CHANNEL_WIDTH_20_40;
		prChnlReqInfo->ucCenterFreqS1 = 0;
		prChnlReqInfo->ucCenterFreqS2 = 0;
	} else {
		prChnlReqInfo->ucCenterFreqS1 =
					prBssInfo->ucVhtChannelFrequencyS1;
		prChnlReqInfo->ucCenterFreqS2 =
					prBssInfo->ucVhtChannelFrequencyS2;
	}

	/* If the S1 is invalid, force to change bandwidth */
	if (prBssInfo->eBand == BAND_5G &&
	    prChnlReqInfo->ucCenterFreqS1 == 0)
		prChnlReqInfo->eChannelWidth = VHT_OP_CHANNEL_WIDTH_20_40;
}

void meshFsmStateAbort_SCAN(struct ADAPTER *prAdapter)
{
	struct MSG_SCN_SCAN_CANCEL *prScanCancelMsg;
	struct MESH_INFO *prMeshInfo;

	ASSERT(prAdapter);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	/* Abort JOIN process. */
	prScanCancelMsg = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
				      sizeof(struct MSG_SCN_SCAN_CANCEL));
	if (!prScanCancelMsg) {
		/* Can't abort SCN FSM */
		ASSERT(0);
		return;
	}

	prScanCancelMsg->rMsgHdr.eMsgId = MID_MESH_SCN_SCAN_CANCEL;
	prScanCancelMsg->ucSeqNum = prMeshInfo->ucSeqNumOfScanReq;
	prScanCancelMsg->ucBssIndex = prMeshInfo->ucBssIndex;
	prScanCancelMsg->fgIsChannelExt = FALSE;

	/* unbuffered message to guarantee scan is cancelled in sequence */
	mboxSendMsg(prAdapter, MBOX_ID_0, (struct MSG_HDR *) prScanCancelMsg,
		    MSG_SEND_METHOD_UNBUF);
}

void meshFsmRunEventScanDone(struct ADAPTER *prAdapter, struct MSG_HDR *prMsgHdr)
{
	struct MSG_SCN_SCAN_DONE *prScanDoneMsg;
	uint8_t ucSeqNumOfCompMsg;
	struct GLUE_INFO *prGlueInfo;
	struct MESH_INFO *prMeshInfo;
	struct MESH_CONNECTION_SETTINGS *prConnSetting;

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);
	if (!prAdapter->fgIsMeshRegister) {
		DBGLOG(MESH, ERROR, "Mesh NOT register, igonore Scan Done\n");
		return;
	}
	prGlueInfo = prAdapter->prGlueInfo;
	ASSERT(prGlueInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prConnSetting = prAdapter->rWifiVar.prMeshConnSettings;
	ASSERT(prConnSetting);
	prScanDoneMsg = (struct MSG_SCN_SCAN_DONE *) prMsgHdr;
	ucSeqNumOfCompMsg = prScanDoneMsg->ucSeqNum;

	ASSERT(prConnSetting->fgIsScanReqIssued);
	ASSERT(prScanDoneMsg->ucBssIndex == prMeshInfo->ucBssIndex);

	if (ucSeqNumOfCompMsg != prMeshInfo->ucSeqNumOfScanReq) {
		DBGLOG(MESH, ERROR,
		       "SEQ NO of MESH SCN DONE MSG is not matched %d %d.\n",
		       ucSeqNumOfCompMsg, prMeshInfo->ucSeqNumOfScanReq);
		ASSERT(0);
	}

	DBGLOG(MESH, LOUD, "EVENT-SCAN DONE: Current Time = %lu\n",
	       (unsigned long) kalGetTimeTick());

	/* Stop the Scan Done Timeout Timer */
	cnmTimerStopTimer(prAdapter, &prMeshInfo->rScanDoneTimer);

	cnmMemFree(prAdapter, prMsgHdr);

	kalMeshScanDone(prGlueInfo, WLAN_STATUS_SUCCESS);

	prConnSetting->fgIsScanReqIssued = FALSE;
}

void meshFsmRunEventScanDoneTimeOut(struct ADAPTER *prAdapter, uint32_t u4Param)
{
	struct GLUE_INFO *prGlueInfo;
	struct MESH_CONNECTION_SETTINGS *prConnSetting;

	ASSERT(prAdapter);
	if (!prAdapter->fgIsMeshRegister) {
		DBGLOG(MESH, ERROR,
		       "Mesh NOT register, igonore Scan Done Timeout\n");
		return;
	}
	prGlueInfo = prAdapter->prGlueInfo;
	ASSERT(prGlueInfo);
	prConnSetting = prAdapter->rWifiVar.prMeshConnSettings;
	ASSERT(prConnSetting);

	kalMeshScanDone(prGlueInfo, WLAN_STATUS_FAILURE);

	/* try to stop scan in Firmware  */
	meshFsmStateAbort_SCAN(prAdapter);

	prConnSetting->fgIsScanReqIssued = FALSE;
}

void meshRunEventFlushMsduInfoTimeOut(struct ADAPTER *prAdapter, uint32_t u4Param)
{
	/* TODO: [mesh] Tx related */
}

uint32_t meshFsmRunEventTxDone(IN struct ADAPTER *prAdapter,
				  IN struct MSDU_INFO *prMsduInfo,
				  IN enum ENUM_TX_RESULT_CODE rTxDoneStatus)
{
	uint8_t ucIndTxDoneOk = 1;

	ASSERT(prAdapter);
	ASSERT(prMsduInfo);

	if (rTxDoneStatus != TX_RESULT_SUCCESS) {
		if (prMsduInfo->ucPacketType == TX_PACKET_TYPE_DATA)
			DBGLOG(MESH, ERROR, "MAC80211 DATA TX fail\n");
		else if (prMsduInfo->ucPacketType == TX_PACKET_TYPE_MGMT)
			DBGLOG(MESH, ERROR, "MAC80211 MGMT TX fail\n");

		DBGLOG(MESH, ERROR,
		       "PKT[0x%08X] WIDX:PID[%u:%u] Status[%u] type %u 0x%p\n",
		       prMsduInfo->u4TxDoneTag, prMsduInfo->ucWlanIndex,
		       prMsduInfo->ucPID, rTxDoneStatus, prMsduInfo->eSrc,
		       prMsduInfo->prPacket);

		ucIndTxDoneOk = 0;
	}

	if (prMsduInfo->prPacket) {
		/* After indicate the TX_status, skb will be no longer
		   available. */
		kalIndicateMac80211TxStatus(prAdapter->prGlueInfo,
					ucIndTxDoneOk, prMsduInfo,
					(uint32_t) prMsduInfo->u2FrameLength);
	}

	return WLAN_STATUS_SUCCESS;
}

void meshFsmRunEventTransition(IN struct ADAPTER *prAdapter,
			       IN struct MSG_HDR *prMsgHdr)
{
	struct MSG_MESH_FSM_TRANSITION *prMsg;
	struct MESH_INFO *prMeshInfo;
	struct MESH_FSM_INFO *prMeshFsmInfo;

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);
	prMsg = (struct MSG_MESH_FSM_TRANSITION *) prMsgHdr;
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prMeshFsmInfo = &prMeshInfo->rMeshFsmInfo;

	meshFsmTransition(prAdapter, prMeshFsmInfo, prMsg->eNextState);

	cnmMemFree(prAdapter, prMsgHdr);
}

void meshFsmRunEventMgmtFrameTx(IN struct ADAPTER *prAdapter,
				IN struct MSG_HDR *prMsgHdr)
{
	struct MSG_MGMT_TX_REQUEST *prMgmtTxMsg;

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);

	prMgmtTxMsg = (struct MSG_MGMT_TX_REQUEST *) prMsgHdr;

	meshFuncTxMgmtFrame(prAdapter, prMgmtTxMsg->prMgmtMsduInfo);

	cnmMemFree(prAdapter, prMsgHdr);
}

void meshFsmRunEventBeaconUpdate(struct ADAPTER *prAdapter, struct MSG_HDR *prMsgHdr)
{
	struct MESH_INFO *prMeshInfo;

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	bssUpdateBeaconContentMesh(prAdapter, prMeshInfo->ucBssIndex);

	cnmMemFree(prAdapter, prMsgHdr);
}

void meshFsmRunEventChGrant(IN struct ADAPTER *prAdapter, IN struct MSG_HDR *prMsgHdr)
{
	struct MSG_CH_GRANT *prMsgChGrant;
	struct BSS_INFO *prBssInfo;
	struct MESH_FSM_INFO *prMeshFsmInfo;
	struct MESH_INFO *prMeshInfo;

	DBGLOG(MESH, INFO, "Run channel grant event\n");

	ASSERT(prAdapter);
	ASSERT(prMsgHdr);

	prMsgChGrant = (struct MSG_CH_GRANT *) prMsgHdr;

	ASSERT(prMsgChGrant->ucBssIndex <= MAX_BSS_INDEX);

	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMsgChGrant->ucBssIndex);

	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prMeshFsmInfo = &prMeshInfo->rMeshFsmInfo;

#if CFG_SISO_SW_DEVELOP
	/* Driver record granted CH in BSS info */
	prBssInfo->fgIsGranted = TRUE;
	prBssInfo->eBandGranted = prMsgChGrant->eRfBand;
	prBssInfo->ucPrimaryChannelGranted = prMsgChGrant->ucPrimaryChannel;
#endif

	meshFsmTransition(prAdapter, prMeshFsmInfo, MESH_STATE_MBSS_ACTIVATE);

	cnmMemFree(prAdapter, prMsgHdr);
}

#if CFG_SUPPORT_MESH_GET_TP
void meshWorkGetLinkSpeed(struct work_struct *work)
{
	//static struct STA_RECORD *prStaRec;
	struct PARAM_GET_STA_STATISTICS rQueryStaStat = {};
	struct GLUE_INFO *prGlueInfo;
	struct ADAPTER *prAdapter;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;
	struct BSS_INFO *prBssInfo;
	uint8_t ucBssIdx;
	struct LINK *prClientList;
	struct STA_RECORD *prCurrStaRec;
	uint32_t u4BufLen;
	//BOOLEAN fgIsStaRecValid = FALSE;
	uint32_t rStatus;
	uint16_t u2Period;
	uint32_t u4PeerElem;
	uint32_t u4ElemIdx;

	KAL_SPIN_LOCK_DECLARATION();

	prGlMeshInfo = container_of(work, struct GL_MESH_INFO,
				    rGetLinkSpeed.work);
	ASSERT(prGlMeshInfo);
	prGlueInfo = prGlMeshInfo->hw->priv;
	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	ucBssIdx = prMeshInfo->ucBssIndex;
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, ucBssIdx);
	prClientList = &prBssInfo->rStaRecOfClientList;
	u2Period = prGlMeshInfo->u2GetLinkSpeedPeriod;

	if (!IS_NET_ACTIVE(prAdapter, ucBssIdx))
		return;

#if 0
		KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_STA_REC);

		if (prClientList->u4NumElem > 0) {
			P_LINK_ENTRY_T prLinkEntry;

			LINK_FOR_EACH_ENTRY(prCurrStaRec, prClientList, rLinkEntry,
						struct STA_RECORD) {
				if (prCurrStaRec == prStaRec) {
					fgIsStaRecValid = TRUE;
					break;
				}
			}

			if (fgIsStaRecValid)
				prLinkEntry = &prStaRec->rLinkEntry;
			else
				prLinkEntry = (P_LINK_ENTRY_T) prClientList;

			prStaRec = LINK_ENTRY(prLinkEntry->prNext,
						  struct STA_RECORD, rLinkEntry);
			prLinkEntry = &prStaRec->rLinkEntry;

			if (prLinkEntry == (P_LINK_ENTRY_T) prClientList)
				prStaRec = LINK_ENTRY(prLinkEntry->prNext,
							  struct STA_RECORD, rLinkEntry);

			KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_STA_REC);

			DBGLOG(MESH, INTO, "Get peer [" MACSTR "] link speed\n",
				   MAC2STR(prStaRec->aucMacAddr));

			kalMemZero(&rQueryStaStat, sizeof(rQueryStaStat));
			COPY_MAC_ADDR(rQueryStaStat.aucMacAddr,
					  prStaRec->aucMacAddr);
			rQueryStaStat.ucReadClear = TRUE;

			rStatus = kalIoctl(prGlueInfo,
					   wlanoidQueryStaStatistics,
					   &rQueryStaStat, sizeof(rQueryStaStat), TRUE,
					   FALSE, TRUE, &u4BufLen);

			if (rStatus)
				DBGLOG(MESH, ERROR,
					   "Unable to get link speed status code = 0x%X\n",
					   rStatus);
			else {
				if (rQueryStaStat.ucAvePer <= PERCENTAGE_MAX) {
					prStaRec->u2LinkSpeed =
								  rQueryStaStat.u2LinkSpeed;
					prStaRec->ucAvePer = rQueryStaStat.ucAvePer;
				}

				DBGLOG(MESH, TRACE, "Speed=%dMbps, AvePER=%d\n",
					   prStaRec->u2LinkSpeed >> 1, prStaRec->ucAvePer);
			}
		}
#else
	if (prClientList->u4NumElem > 0) {
		DBGLOG(MESH, INFO, "# of peers=%d\n", prClientList->u4NumElem);

		for(u4PeerElem = 1; u4PeerElem <= prClientList->u4NumElem; u4PeerElem++) {

			KAL_ACQUIRE_SPIN_LOCK(prAdapter, SPIN_LOCK_STA_REC);

			prCurrStaRec = LINK_ENTRY((prClientList)->prNext, struct STA_RECORD, rLinkEntry);
			if(!LINK_ENTRY_IS_VALID(&prCurrStaRec->rLinkEntry)) {
				KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_STA_REC);
				DBGLOG(MESH, TRACE, "A peers just disconnected\n");
				return;
			}

			u4ElemIdx = u4PeerElem - 1;
			while(u4ElemIdx > 0) {
				prCurrStaRec = LINK_ENTRY((prCurrStaRec)->rLinkEntry.prNext, struct STA_RECORD, rLinkEntry);
				u4ElemIdx--;
				if(!LINK_ENTRY_IS_VALID(&prCurrStaRec->rLinkEntry)) {
					KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_STA_REC);
					DBGLOG(MESH, TRACE, "A peers just disconnected\n");
					return;
				}
			}
			DBGLOG(MESH, TRACE, "Get peer [" MACSTR "] link speed\n",
			       MAC2STR(prCurrStaRec->aucMacAddr));

			kalMemZero(&rQueryStaStat, sizeof(rQueryStaStat));
			COPY_MAC_ADDR(rQueryStaStat.aucMacAddr, prCurrStaRec->aucMacAddr);

			KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_STA_REC);

			rQueryStaStat.ucReadClear = TRUE;

			rStatus = kalIoctl(prGlueInfo,
					   wlanoidQueryStaStatistics,
					   &rQueryStaStat, sizeof(rQueryStaStat), TRUE,
					   FALSE, TRUE, &u4BufLen);

			if (rStatus)
				DBGLOG(MESH, ERROR,
					   "Unable to get link speed status code = 0x%X\n", rStatus);
		}
	}

#endif

	else {

		//KAL_RELEASE_SPIN_LOCK(prAdapter, SPIN_LOCK_CLIENT_LIST);
		DBGLOG(MESH, INFO, "No peers connected\n");
	}
	queue_delayed_work(system_long_wq, &prGlMeshInfo->rGetLinkSpeed,
			   MSEC_TO_JIFFIES(u2Period));

	return;
}
#endif /* CFG_SUPPORT_MESH_GET_TP */

/* TODO: [mesh] The BSS_INFO members listed below are NOT yet set in mesh and
 * needed to be reviewd in the future.
 */
#if 0
struct _BSS_INFO_T {
	uint32_t u4PrivateData;
	uint8_t ucOwnMacIndex;
	uint16_t u2ATIMWindow;
	BOOLEAN fgIsShortPreambleAllowed;
	BOOLEAN fgUseShortPreamble;
	BOOLEAN fgUseShortSlotTime;
	uint8_t ucAssocClientCnt;
	BOOLEAN fgIsNetAbsent;
	uint8_t ucOpChangeChannelWidth;
	BOOLEAN fgIsOpChangeChannelWidth;
	PM_PROFILE_SETUP_INFO_T rPmProfSetupInfo;
	uint8_t ucWmmParamSetCount;
	AC_QUE_PARMS_T arACQueParms[WMM_AC_INDEX_NUM];
	uint8_t aucCWminLog2ForBcast[WMM_AC_INDEX_NUM];
	uint8_t aucCWmaxLog2ForBcast[WMM_AC_INDEX_NUM];
	AC_QUE_PARMS_T arACQueParmsForBcast[WMM_AC_INDEX_NUM];
	uint8_t ucWmmQueSet;
	ENUM_BAND_T eBand;
	uint8_t ucPrimaryChannel;
	uint8_t ucHtOpInfo1;
	uint8_t ucHtPeerOpInfo1;
	uint16_t u2HtOpInfo2;
	uint16_t u2HtOpInfo3;
	uint8_t ucNss;
	uint16_t u2VhtBasicMcsSet;
	uint8_t ucVhtPeerChannelWidth;
	uint8_t ucVhtPeerChannelFrequencyS1;
	uint8_t ucVhtPeerChannelFrequencyS2;

	BOOLEAN fgIsNetActive;

	uint32_t u4RsnSelectedGroupCipher;
	uint32_t u4RsnSelectedPairwiseCipher;
	uint32_t u4RsnSelectedAKMSuite;
	uint16_t u2RsnSelectedCapInfo;

	BOOLEAN fgErpProtectMode;
	ENUM_HT_PROTECT_MODE_T eHtProtectMode;
	ENUM_GF_MODE_T eGfOperationMode;
	ENUM_RIFS_MODE_T eRifsOperationMode;
	BOOLEAN fgObssErpProtectMode;
	ENUM_HT_PROTECT_MODE_T eObssHtProtectMode;
	ENUM_GF_MODE_T eObssGfOperationMode;
	BOOLEAN fgObssRifsOperationMode;
	BOOLEAN fgAssoc40mBwAllowed;
	BOOLEAN fg40mBwAllowed;
	ENUM_CHNL_EXT_T eBssSCO;
	uint8_t auc2G_20mReqChnlList[CHNL_LIST_SZ_2G + 1];
	uint8_t auc2G_NonHtChnlList[CHNL_LIST_SZ_2G + 1];
	uint8_t auc2G_PriChnlList[CHNL_LIST_SZ_2G + 1];
	uint8_t auc2G_SecChnlList[CHNL_LIST_SZ_2G + 1];

	uint8_t auc5G_20mReqChnlList[CHNL_LIST_SZ_5G + 1];
	uint8_t auc5G_NonHtChnlList[CHNL_LIST_SZ_5G + 1];
	uint8_t auc5G_PriChnlList[CHNL_LIST_SZ_5G + 1];
	uint8_t auc5G_SecChnlList[CHNL_LIST_SZ_5G + 1];

	TIMER_T rObssScanTimer;
	uint16_t u2ObssScanInterval;

	BOOLEAN fgObssActionForcedTo20M;
	BOOLEAN fgObssBeaconForcedTo20M;

	uint16_t u2HwDefaultFixedRateCode;
	uint16_t u2HwLPWakeupGuardTimeUsec;

	uint8_t ucBssFreeQuota;

#if CFG_ENABLE_GTK_FRAME_FILTER
	P_IPV4_NETWORK_ADDRESS_LIST prIpV4NetAddrList;
#endif
	uint16_t u2DeauthReason;

#if CFG_SUPPORT_TDLS
	BOOLEAN fgTdlsIsProhibited;
	BOOLEAN fgTdlsIsChSwProhibited;
#endif
#if CFG_SUPPORT_PNO
	BOOLEAN fgIsPNOEnable;
	BOOLEAN fgIsNetRequestInActive;
#endif

	WIFI_WMM_AC_STAT_T arLinkStatistics[WMM_AC_INDEX_NUM];

	ENUM_DBDC_BN_T eDBDCBand;

	uint32_t u4CoexPhyRateLimit;

#if CFG_SUPPORT_ROAMING_SKIP_ONE_AP
	uint8_t	ucRoamSkipTimes;
	BOOLEAN fgGoodRcpiArea;
	BOOLEAN fgPoorRcpiArea;
#endif

	BOOLEAN fgIsGranted;
	ENUM_BAND_T eBandGranted;
	uint8_t ucPrimaryChannelGranted;
}
#endif

void meshFsmDisable(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo)
{
	struct GLUE_INFO *prGlueInfo;
	struct MESH_INFO *prMeshInfo;
	uint8_t ucBssIdx;

	DBGLOG(MESH, INFO, "Mesh disable\n");

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	prGlueInfo = prAdapter->prGlueInfo;
	ASSERT(prGlueInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	ucBssIdx = prBssInfo->ucBssIndex;
	ASSERT(ucBssIdx == prMeshInfo->ucBssIndex);

	UNSET_NET_ACTIVE(prAdapter, ucBssIdx);

	cnmTimerStopTimer(prAdapter, &(prMeshInfo->rFlushMsduInfoTimer));

	SET_NET_PWR_STATE_IDLE(prAdapter, ucBssIdx);

	kalClearMgmtFramesByBssIdx(prGlueInfo, ucBssIdx);

	kalClearSecurityFramesByBssIdx(prGlueInfo, ucBssIdx);

	wlanReleasePendingCMDbyBssIdx(prAdapter, ucBssIdx);

	nicFreePendingTxMsduInfo(prAdapter, ucBssIdx,
		MSDU_REMOVE_BY_BSS_INDEX);

	/* Deactivate BSS. */
	nicDeactivateNetwork(prAdapter, ucBssIdx);

	if (prBssInfo->prBeacon) {
		cnmMgtPktFree(prAdapter, prBssInfo->prBeacon);
		prBssInfo->prBeacon = NULL;
	}

	cnmFreeBssInfo(prAdapter, prBssInfo);

	prGlueInfo->prMeshInfo->vif = NULL;

	prAdapter->meshStarted = FALSE;
}

void meshFsmIdle(struct ADAPTER *prAdapter)
{
	struct GLUE_INFO *prGlueInfo;
	struct WIFI_VAR *prWifiVar;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;
	struct BSS_INFO *prBssInfo;
	struct MESH_SPECIFIC_BSS_INFO *prMeshSpecificBssInfo;
	uint8_t *pucAdHocAPMode;
	uint8_t *pucBasicPhyType;
	struct ieee80211_vif *vif;

	DBGLOG(MESH, INFO, "Mesh idle\n");

	ASSERT(prAdapter);
	prGlueInfo = prAdapter->prGlueInfo;
	ASSERT(prGlueInfo);
	prWifiVar = &prAdapter->rWifiVar;
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prMeshSpecificBssInfo = prWifiVar->prMeshSpecificBssInfo;
	ASSERT(prMeshSpecificBssInfo);
	vif = prGlMeshInfo->vif;
	ASSERT(vif);

	if (prAdapter->meshStarted)
		return;

	prBssInfo = cnmGetBssInfoAndInit(prAdapter, NETWORK_TYPE_MESH, false);
	ASSERT(prBssInfo);

	prMeshInfo->ucBssIndex = prBssInfo->ucBssIndex;

	prGlMeshInfo->u4MeshHtMcsMask[0] = 0xff;
	prGlMeshInfo->u4MeshHtMcsMask[1] = 0xff;
	prGlMeshInfo->u4MeshLegacyRateSet[0] = RATE_SET_ERP;
	prGlMeshInfo->u4MeshLegacyRateSet[1] = RATE_SET_ERP_P2P;
	prGlMeshInfo->u2GetLinkSpeedPeriod =
			(uint16_t) wlanCfgGetUint32(prAdapter,
						   "MeshGetLinkSpeedPeriod",
						   MESH_LINK_SPEED_PERIOD_MSEC);

	pucAdHocAPMode = &prBssInfo->ucConfigAdHocAPMode;
	pucBasicPhyType = &prBssInfo->ucNonHTBasicPhyType;

	cnmTimerInitTimer(prAdapter, &prMeshInfo->rScanDoneTimer,
		(PFN_MGMT_TIMEOUT_FUNC)meshFsmRunEventScanDoneTimeOut,
			  (unsigned long) NULL);

	/*Init the Flush MsduInfo Timer*/
	cnmTimerInitTimer(prAdapter, &prMeshInfo->rFlushMsduInfoTimer,
		(PFN_MGMT_TIMEOUT_FUNC)meshRunEventFlushMsduInfoTimeOut,
			  (unsigned long) NULL);

	/*Start the Flush MsduInfo Timer*/
	cnmTimerStartTimer(prAdapter, &prMeshInfo->rFlushMsduInfoTimer,
			   MESH_FLUSH_MSDUINFO_TIMER_MSEC);

	BSS_INFO_INIT(prAdapter, prBssInfo);

	*pucAdHocAPMode = MESH_MODE_11G;
	prBssInfo->u2HwDefaultFixedRateCode = RATE_OFDM_6M;
	*pucBasicPhyType = (uint8_t)
		rNonHTMeshModeAttributes[*pucAdHocAPMode].ePhyTypeIndex;
	prBssInfo->u2BSSBasicRateSet =
		rNonHTMeshModeAttributes[*pucAdHocAPMode].u2BSSBasicRateSet;
	prBssInfo->u2OperationalRateSet =
		rNonHTPhyAttributes[*pucBasicPhyType].u2SupportedRateSet;
	rateGetDataRatesFromRateSet(prBssInfo->u2OperationalRateSet,
				    prBssInfo->u2BSSBasicRateSet,
				    prBssInfo->aucAllSupportedRates,
				    &prBssInfo->ucAllSupportedRatesLen);

	prBssInfo->prBeacon = cnmMgtPktAlloc(prAdapter,
					     OFFSET_OF(struct WLAN_BEACON_FRAME,
						       aucInfoElem[0]) +
						       MAX_IE_LENGTH);

	if (prBssInfo->prBeacon) {
		prBssInfo->prBeacon->eSrc = TX_PACKET_MGMT;
		prBssInfo->prBeacon->ucStaRecIndex = 0xFF; /* NULL STA_REC */
		prBssInfo->prBeacon->ucBssIndex = prMeshInfo->ucBssIndex;
	} else {
		/* Out of memory. */
		ASSERT(FALSE);
	}
	prBssInfo->eCurrentOPMode = OP_MODE_NUM;

	prBssInfo->rPmProfSetupInfo.ucBmpDeliveryAC = PM_UAPSD_ALL;
	prBssInfo->rPmProfSetupInfo.ucBmpTriggerAC = PM_UAPSD_ALL;
	prBssInfo->rPmProfSetupInfo.ucUapsdSp = WMM_MAX_SP_LENGTH_2;
	prBssInfo->ucPrimaryChannel = P2P_DEFAULT_LISTEN_CHANNEL;
	prBssInfo->eBand = BAND_2G4;
	prBssInfo->eBssSCO = CHNL_EXT_SCN;

	if (IS_FEATURE_ENABLED(prWifiVar->ucQoS))
		prBssInfo->fgIsQBSS = TRUE;
	else
		prBssInfo->fgIsQBSS = FALSE;

	COPY_MAC_ADDR(prBssInfo->aucOwnMacAddr, vif->addr);

	SET_NET_PWR_STATE_IDLE(prAdapter, prMeshInfo->ucBssIndex);

	INIT_DELAYED_WORK(&prGlMeshInfo->hw_scan,
			  mtk_mesh_mac80211_hw_scan_work);
	/* TODO: [mesh] Is uninit work needed in remove IF? */

#if CFG_SUPPORT_MESH_GET_TP
	INIT_DELAYED_WORK(&prGlMeshInfo->rGetLinkSpeed,
			  meshWorkGetLinkSpeed);
#endif /* CFG_SUPPORT_MESH_GET_TP */

	prAdapter->meshStarted = TRUE;
}

void meshFsmScan(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
		 struct MESH_FSM_INFO *prMeshFsmInfo)
{
	DBGLOG(MESH, INFO, "Mesh scan\n");

	/* TODO: [mesh] */
}

void meshFsmOnlineScan(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
		       struct MESH_FSM_INFO *prMeshFsmInfo)
{
	DBGLOG(MESH, INFO, "Mesh online scan\n");

	/* TODO: [mesh] */
}

void meshFsmMbssInit(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
		     struct MESH_FSM_INFO *prMeshFsmInfo)
{
	struct MESH_INFO *prMeshInfo;

	DBGLOG(MESH, INFO, "Mesh update BssInfo and request channel\n");

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMeshFsmInfo);

	prMeshInfo = prAdapter->prMeshInfo;

	SET_NET_ACTIVE(prAdapter, prMeshInfo->ucBssIndex);
	SET_NET_PWR_STATE_ACTIVE(prAdapter, prMeshInfo->ucBssIndex);

	/* sync with firmware */
	nicActivateNetwork(prAdapter, prMeshInfo->ucBssIndex);

	bssInitForMesh(prAdapter, prBssInfo);

	meshPrepReqChnl(prAdapter, prBssInfo, &prMeshFsmInfo->rChnlReqInfo);

	meshFuncAcquireChnl(prAdapter, prBssInfo->ucBssIndex,
			    &prMeshFsmInfo->rChnlReqInfo);
}

void meshFsmMbssActivate(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
			 struct MESH_FSM_INFO *prMeshFsmInfo)
{
	struct GLUE_INFO *prGlueInfo;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;

	DBGLOG(MESH, INFO, "Mesh MBSS activate\n");

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	ASSERT(prMeshFsmInfo);
	prGlueInfo = prAdapter->prGlueInfo;
	ASSERT(prGlueInfo);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	nicQmUpdateWmmParms(prAdapter, prBssInfo->ucBssIndex);

	meshChangeMediaState(prBssInfo, PARAM_MEDIA_STATE_CONNECTED);

	/* use command packets to inform firmware */
	nicUpdateBss(prAdapter, prMeshInfo->ucBssIndex);

	bssUpdateBeaconContentMesh(prAdapter, prMeshInfo->ucBssIndex);

	/* enable beaconing */
	nicPmIndicateBssCreated(prAdapter, prMeshInfo->ucBssIndex);

	/* Set ACTIVE flag */
	prBssInfo->fgIsBeaconActivated = TRUE;
	prBssInfo->fgHoldSameBssidForIBSS = TRUE;

	meshFuncAbortChnl(prAdapter, prBssInfo->ucBssIndex,
			  &prMeshFsmInfo->rChnlReqInfo);

#if CFG_SUPPORT_MESH_GET_TP
	queue_delayed_work(system_long_wq, &prGlMeshInfo->rGetLinkSpeed,
			   MSEC_TO_JIFFIES(prGlMeshInfo->u2GetLinkSpeedPeriod));
#endif /* CFG_SUPPORT_MESH_GET_TP */
}

void meshFsmMbssLeave(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo)
{
	struct GLUE_INFO *prGlueInfo;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;
	struct MSG_MESH_FSM_TRANSITION *prMsg;

	DBGLOG(MESH, INFO, "Mesh MBSS leave\n");

	ASSERT(prAdapter);
	ASSERT(prBssInfo);
	prGlueInfo = prAdapter->prGlueInfo;
	ASSERT(prGlueInfo);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);

	prMsg = cnmMemAlloc(prAdapter, RAM_TYPE_MSG,
			    sizeof(struct MSG_MESH_FSM_TRANSITION));
	if (!prMsg) {
		DBGLOG(MESH, ERROR, "can't alloc MSG\n");
		return;
	}

	UNSET_NET_ACTIVE(prAdapter, prMeshInfo->ucBssIndex);
	SET_NET_PWR_STATE_IDLE(prAdapter, prMeshInfo->ucBssIndex);

#if CFG_SUPPORT_MESH_GET_TP
	flush_delayed_work(&prGlMeshInfo->rGetLinkSpeed);
#endif /* CFG_SUPPORT_MESH_GET_TP */

	nicPmIndicateBssAbort(prAdapter, prMeshInfo->ucBssIndex);

	if (prBssInfo->fgIsBeaconActivated) {
		nicUpdateBeaconIETemplate(prAdapter, IE_UPD_METHOD_DELETE_ALL,
					  prMeshInfo->ucBssIndex, 0, NULL, 0);
		prBssInfo->fgIsBeaconActivated = FALSE;
	}

	rlmBssAborted(prAdapter, prBssInfo);
	meshChangeMediaState(prBssInfo, PARAM_MEDIA_STATE_DISCONNECTED);

	/* sync. with firmware */
	nicUpdateBss(prAdapter, prMeshInfo->ucBssIndex);
	nicDeactivateNetwork(prAdapter, prMeshInfo->ucBssIndex);

	/* TODO: [mesh] Is it possible that simultaneous transition of requests
	 * from mac80211 callback and mesh module lead race condition?
	 */
	/* Send message to self instead of calling meshFsmTransition() directly.
	 * In this way, meshFsmTransition would NOT be executed in a nested way,
	 * although this is tedious.
	 */
	prMsg->rMsgHdr.eMsgId = MID_MNY_MESH_FSM_TRANSITION;
	prMsg->eNextState = MESH_STATE_IDLE;

	mboxSendMsg(prAdapter, MBOX_ID_0, (struct MSG_HDR *) prMsg,
		    MSG_SEND_METHOD_BUF);
}

void meshFsmTransition(struct ADAPTER *prAdapter,
		       struct MESH_FSM_INFO *prMeshFsmInfo,
		       enum ENUM_MESH_STATE eNextState)
{
	struct GLUE_INFO *prGlueInfo;
	struct GL_MESH_INFO *prGlMeshInfo;
	struct MESH_INFO *prMeshInfo;
	struct BSS_INFO *prBssInfo;
	struct MESH_CHNL_REQ_INFO *prChnlReqInfo;
	enum ENUM_MESH_STATE ePrevState;

	ASSERT(prAdapter);
	ASSERT(prMeshFsmInfo);
	prGlueInfo = prAdapter->prGlueInfo;
	ASSERT(prGlueInfo);
	prGlMeshInfo = prGlueInfo->prMeshInfo;
	ASSERT(prGlMeshInfo);
	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMeshInfo->ucBssIndex);
	ASSERT(prBssInfo);
	prChnlReqInfo = &prMeshFsmInfo->rChnlReqInfo;

	ePrevState = prMeshFsmInfo->eCurrentState;
	DBGLOG(MESH, INFO, "Mesh FSM transition [%s] -> [%s]\n",
						apucDebugMeshState[ePrevState],
						apucDebugMeshState[eNextState]);

	switch (eNextState) {
	case MESH_STATE_DISABLE:
		meshFsmDisable(prAdapter, prBssInfo);
		break;

	case MESH_STATE_IDLE:
		meshFsmIdle(prAdapter);
		break;

	case MESH_STATE_SCAN:
		meshFsmScan(prAdapter, prBssInfo, prMeshFsmInfo);
		break;

	case MESH_STATE_ONLINE_SCAN:
		meshFsmOnlineScan(prAdapter, prBssInfo, prMeshFsmInfo);
		break;

	case MESH_STATE_MBSS_INIT:
		meshFsmMbssInit(prAdapter, prBssInfo, prMeshFsmInfo);
		break;

	case MESH_STATE_MBSS_ACTIVATE:
		meshFsmMbssActivate(prAdapter, prBssInfo, prMeshFsmInfo);
		break;

	case MESH_STATE_MBSS_LEAVE:
		meshFsmMbssLeave(prAdapter, prBssInfo);
		break;

	default:
		ASSERT(0);
		break;
	}

	prMeshFsmInfo->eCurrentState = eNextState;

	wake_up(&prGlMeshInfo->mesh_wait);
}

uint32_t MeshPeerAdd(struct ADAPTER *prAdapter, void *pvSetBuffer,
			uint32_t u4SetBufferLen, uint32_t *pu4SetInfoLen)
{
	struct MESH_CMD_PEER_ADD *prMeshCmdPeerAdd;
	struct STA_RECORD *prStaRec;
	struct MESH_INFO *prMeshInfo;
	struct BSS_INFO *prBssInfo;

	ASSERT(prAdapter);

	if(!pvSetBuffer) {
		DBGLOG(MESH, ERROR, "Add Mesh peer without MAC_ADDR\n");
		return WLAN_STATUS_INVALID_DATA;
	}

	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMeshInfo->ucBssIndex);
	ASSERT(prBssInfo);
	*pu4SetInfoLen = sizeof(struct MESH_CMD_PEER_ADD);
	prMeshCmdPeerAdd = pvSetBuffer;

	prStaRec = cnmGetStaRecByAddress(prAdapter, prMeshInfo->ucBssIndex,
					 prMeshCmdPeerAdd->aucPeerMac);

	if (!prStaRec) {
		/* TODO: [mesh] Check sta type in firmware */
		prStaRec = cnmStaRecAlloc(prAdapter, STA_TYPE_MESH_PEER,
					  prMeshInfo->ucBssIndex,
					  prMeshCmdPeerAdd->aucPeerMac);

		if (!prStaRec) {
			DBGLOG(MESH, ERROR, "Mesh alloc StaRec fail\n");

			return WLAN_STATUS_RESOURCES;
		}
	}

	prStaRec->u2BSSBasicRateSet = prMeshCmdPeerAdd->u2BSSBasicRateSet;
	prStaRec->u2DesiredNonHTRateSet =
					prMeshCmdPeerAdd->u2DesiredNonHTRateSet;
	prStaRec->u2OperationalRateSet = prMeshCmdPeerAdd->u2OperationalRateSet;
	prStaRec->ucPhyTypeSet = prMeshCmdPeerAdd->ucPhyTypeSet;
	prStaRec->ucDesiredPhyTypeSet = prMeshCmdPeerAdd->ucDesiredPhyTypeSet;

	/* HT PARAMS */
	prStaRec->u2HtCapInfo =  prMeshCmdPeerAdd->u2HtCapInfo;
	prStaRec->ucAmpduParam = prMeshCmdPeerAdd->ucAmpduParam;
	prStaRec->ucMcsSet = prMeshCmdPeerAdd->ucMcsSet;
	kalMemCopy(prStaRec->aucRxMcsBitmask, prMeshCmdPeerAdd->aucRxMcsBitmask,
		   sizeof(prStaRec->aucRxMcsBitmask));

	/* VHT PARAMS */
	prStaRec->u4VhtCapInfo = prMeshCmdPeerAdd->u4VhtCapInfo;
	prStaRec->u2VhtRxMcsMap = prMeshCmdPeerAdd->u2VhtRxMcsMap;
	prStaRec->u2VhtRxHighestSupportedDataRate =
			prMeshCmdPeerAdd->u2VhtRxHighestSupportedDataRate;
	prStaRec->u2VhtTxMcsMap = prMeshCmdPeerAdd->u2VhtTxMcsMap;
	prStaRec->u2VhtTxHighestSupportedDataRate =
			prMeshCmdPeerAdd->u2VhtTxHighestSupportedDataRate;
	prStaRec->ucVhtOpMode = prMeshCmdPeerAdd->ucVhtOpMode;

	/* ASSOC ID */
	prStaRec->u2AssocId = prMeshCmdPeerAdd->u2AssocId;
	/* Update default Tx rate */
	nicTxUpdateStaRecDefaultRate(prStaRec);

	bssAddClient(prAdapter, prBssInfo, prStaRec);

	cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_3);

	return WLAN_STATUS_SUCCESS;
}

uint32_t MeshPeerRemove(struct ADAPTER *prAdapter, void *pvSetBuffer,
			   uint32_t u4SetBufferLen, uint32_t *pu4SetInfoLen)
{
	struct MESH_INFO *prMeshInfo;
	struct BSS_INFO *prBssInfo;
	struct MESH_CMD_PEER_REMOVE *prMeshCmdPeerRemove;
	struct STA_RECORD *prStaRec;

	ASSERT(prAdapter);

	if(!pvSetBuffer) {
		DBGLOG(MESH, ERROR, "Remove Mesh peer without MAC_ADDR\n");
		return WLAN_STATUS_INVALID_DATA;
	}

	prMeshInfo = prAdapter->prMeshInfo;
	ASSERT(prMeshInfo);
	prBssInfo = GET_BSS_INFO_BY_INDEX(prAdapter, prMeshInfo->ucBssIndex);
	ASSERT(prBssInfo);
	*pu4SetInfoLen = sizeof(struct MESH_CMD_PEER_REMOVE);
	prMeshCmdPeerRemove = pvSetBuffer;

	prStaRec = cnmGetStaRecByAddress(prAdapter, prMeshInfo->ucBssIndex,
					 prMeshCmdPeerRemove->aucPeerMac);

	if (!prStaRec) {
		DBGLOG(MESH, ERROR, "NO corresponding StaRec to be removed\n");

		return WLAN_STATUS_FAILURE;
	}

	bssRemoveClient(prAdapter, prBssInfo, prStaRec);
	/* DO this, before freeing */
	cnmStaRecChangeState(prAdapter, prStaRec, STA_STATE_1);
	cnmStaRecFree(prAdapter, prStaRec);

	return WLAN_STATUS_SUCCESS;
}
#endif	/* CFG_ENABLE_WIFI_MESH */
