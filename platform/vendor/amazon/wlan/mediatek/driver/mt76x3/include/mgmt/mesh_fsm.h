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

/*! \file   mesh_fsm.h
 *  \brief  Declaration of functions and finite state machine for MESH Module.
 *
 *  Declaration of functions and finite state machine for MESH Module.
 */

#ifndef _MESH_FSM_H
#define _MESH_FSM_H

#if CFG_ENABLE_WIFI_MESH
/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
/* 50 msec : Flush Timer is scheduled every 50 msec */
#define MESH_FLUSH_MSDUINFO_TIMER_MSEC		(50)

#define MESH_CHNL_HOLD_TIME_MS			(5000)

#define MESH_FSM_TRANSITION_TIMEOUT_MS		(1000)

/* align AIS setting */
#define MESH_SCN_DONE_TIMEOUT_SEC		(15)

#define MESH_LINK_SPEED_PERIOD_MSEC		(5000)

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */
#define meshChangeMediaState(_prBssInfo, _eNewMediaState) \
			(_prBssInfo->eConnectionState = (_eNewMediaState))

#define meshFsmGetState(_prAdapter)	\
			(_prAdapter->prMeshInfo->rMeshFsmInfo.eCurrentState)

#define IS_MESH_FSM_STATE(_prAdapter, _eState) \
			(meshFsmGetState(_prAdapter) == _eState)

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */
enum ENUM_MESH_STATE {
	MESH_STATE_DISABLE = 0,
	MESH_STATE_IDLE,
	MESH_STATE_SCAN,
	MESH_STATE_ONLINE_SCAN,
	MESH_STATE_MBSS_INIT,
	MESH_STATE_MBSS_ACTIVATE,
	MESH_STATE_MBSS_LEAVE,
	MESH_STATE_NUM
};

struct MSG_MESH_BCN_UPDATE {
	struct MSG_HDR rMsgHdr;
	uint32_t u4BcnHdrLen;
	uint32_t u4BcnBodyLen;
	uint8_t *pucBcnHdr;
	uint8_t *pucBcnBody;
	uint8_t aucBuffer[1];       /* Header & Body are put here. */
};

/* used to add StaRec */
struct MESH_CMD_PEER_ADD {
	uint8_t aucPeerMac[MAC_ADDR_LEN];
	uint16_t u2BSSBasicRateSet;
	uint16_t u2DesiredNonHTRateSet;
	uint16_t u2OperationalRateSet;
	uint8_t ucPhyTypeSet;
	uint16_t u2HtCapInfo;
	uint32_t u4VhtCapInfo;
	uint16_t u2VhtRxMcsMap;
	uint16_t u2VhtRxHighestSupportedDataRate;
	uint16_t u2VhtTxMcsMap;
	uint16_t u2VhtTxHighestSupportedDataRate;
	/* This field name is misleading. It does NOT represent VHT Operation
	 * Information subfield in VHT Operation IE. It represents Operation
	 * Mode subfield in Operation Mode Noficiation IE.
	 */
	uint8_t ucVhtOpMode;
	uint8_t ucAmpduParam;
	uint8_t ucMcsSet;
	uint8_t aucRxMcsBitmask[SUP_MCS_RX_BITMASK_OCTET_NUM];
	uint8_t ucDesiredPhyTypeSet;
	uint16_t u2AssocId;
};

/* used to remove StaRec */
struct MESH_CMD_PEER_REMOVE {
	uint8_t aucPeerMac[MAC_ADDR_LEN];
};

struct MESH_CHNL_REQ_INFO {
	struct LINK rP2pChnlReqLink;
	bool fgIsChannelRequested;
	uint8_t ucSeqNumOfChReq;
	uint64_t u8Cookie;
	uint8_t ucReqChnlNum;
	enum ENUM_BAND eBand;
	enum ENUM_CHNL_EXT eChnlSco;
	uint8_t ucOriChnlNum;
	enum ENUM_CHANNEL_WIDTH eChannelWidth;	/*VHT operation ie */
	uint8_t ucCenterFreqS1;
	uint8_t ucCenterFreqS2;
	enum ENUM_BAND eOriBand;
	enum ENUM_CHNL_EXT eOriChnlSco;
	uint32_t u4MaxInterval;
	enum ENUM_CH_REQ_TYPE eChnlReqType;
};

struct MESH_FSM_INFO {
	enum ENUM_MESH_STATE eCurrentState;
	struct MESH_CHNL_REQ_INFO rChnlReqInfo;
};

struct MSG_MESH_FSM_TRANSITION {
	struct MSG_HDR rMsgHdr;	/* Must be the first member */
	enum ENUM_MESH_STATE eNextState;
};

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
void meshFuncAcquireChnl(IN struct ADAPTER *prAdapter, IN uint8_t ucBssIdx,
			 IN struct MESH_CHNL_REQ_INFO *prChnlReqInfo);

void meshFuncAbortChnl(IN struct ADAPTER *prAdapter, IN uint8_t ucBssIdx,
		       IN struct MESH_CHNL_REQ_INFO *prChnlReqInfo);

uint32_t meshFuncTxMgmtFrame(IN struct ADAPTER *prAdapter,
				IN struct MSDU_INFO *prMgmtTxMsdu);

void meshPrepReqChnl(IN struct ADAPTER *prAdapter, IN struct BSS_INFO *prBssInfo,
		     OUT struct MESH_CHNL_REQ_INFO *prChnlReqInfo);

void meshFsmStateAbort_SCAN(struct ADAPTER *prAdapter);

void meshFsmRunEventScanDone(struct ADAPTER *prAdapter, struct MSG_HDR *prMsgHdr);

void meshFsmRunEventScanDoneTimeOut(struct ADAPTER *prAdapter, uint32_t u4Param);

void meshRunEventFlushMsduInfoTimeOut(struct ADAPTER *prAdapter, uint32_t u4Param);

uint32_t meshFsmRunEventTxDone(IN struct ADAPTER *prAdapter,
				  IN struct MSDU_INFO *prMsduInfo,
				  IN enum ENUM_TX_RESULT_CODE rTxDoneStatus);

void meshFsmRunEventTransition(IN struct ADAPTER *prAdapter,
			       IN struct MSG_HDR *prMsgHdr);

void meshFsmRunEventMgmtFrameTx(IN struct ADAPTER *prAdapter,
				IN struct MSG_HDR *prMsgHdr);

void meshFsmRunEventBeaconUpdate(struct ADAPTER *prAdapter, struct MSG_HDR *prMsgHdr);

void meshFsmRunEventChGrant(IN struct ADAPTER *prAdapter, IN struct MSG_HDR *prMsgHdr);

void meshFsmDisable(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo);

void meshFsmIdle(struct ADAPTER *prAdapter);

void meshFsmScan(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
		 struct MESH_FSM_INFO *prMeshFsmInfo);

void meshFsmOnlineScan(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
		       struct MESH_FSM_INFO *prMeshFsmInfo);

void meshFsmMbssInit(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
		     struct MESH_FSM_INFO *prMeshFsmInfo);

void meshFsmMbssReqChnl(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
			struct MESH_FSM_INFO *prMeshFsmInfo);

void meshFsmMbssEnHw(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
		     struct MESH_FSM_INFO *prMeshFsmInfo);

void meshFsmMbssRelChnl(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
			struct MESH_FSM_INFO *prMeshFsmInfo);

void meshFsmMbssComplete(struct ADAPTER *prAdapter, struct BSS_INFO *prBssInfo,
			 struct MESH_FSM_INFO *prMeshFsmInfo);

void meshFsmTransition(struct ADAPTER *prAdapter,
		       struct MESH_FSM_INFO *prMeshFsmInfo,
		       enum ENUM_MESH_STATE eNextState);

uint32_t MeshPeerAdd(struct ADAPTER *prAdapter, void *pvSetBuffer,
			uint32_t u4SetBufferLen, uint32_t *pu4SetInfoLen);

uint32_t MeshPeerRemove(struct ADAPTER *prAdapter, void *pvSetBuffer,
			   uint32_t u4SetBufferLen, uint32_t *pu4SetInfoLen);

#endif	/* CFG_ENABLE_WIFI_MESH */
#endif	/* _MESH_FSM_H */
