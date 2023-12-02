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

#ifndef _MESH_H
#define _MESH_H
/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include "mesh_fsm.h"

/*******************************************************************************
*                    E X T E R N A L   V A R I A B L E
********************************************************************************
*/

#if CFG_ENABLE_WIFI_MESH
/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
struct MESH_INFO {
	uint8_t  ucSeqNumOfScanReq;
	struct TIMER rScanDoneTimer;
	struct TIMER rFlushMsduInfoTimer;
	struct MESH_FSM_INFO rMeshFsmInfo;
	uint8_t ucBssIndex;
};

struct MESH_SPECIFIC_BSS_INFO {
	uint16_t u2Reserved; /* Reserved for Data Type Check */
};

struct MESH_CONNECTION_SETTINGS {
	enum ENUM_CHNL_EXT eChnlSco;
	uint8_t  ucSSIDLen;
	uint8_t  aucSSID[ELEM_MAX_LEN_SSID];
	enum ENUM_PARAM_AUTH_MODE eAuthMode;
	enum ENUM_WEP_STATUS eEncStatus;
	bool fgIsScanReqIssued;
	/* MIB attributes */
	uint16_t u2BeaconPeriod;
	uint8_t  ucAdHocChannelNum; /* For AdHoc */
	enum ENUM_BAND eAdHocBand; /* For AdHoc */
	/* ATIM windows using for IBSS power saving function */
	uint16_t u2AtimWindow;
	uint8_t  ucDTIMPeriod;
	uint8_t  ucVhtChannelWidth;
	uint8_t  ucVhtChannelFrequencyS1;
	uint8_t  ucVhtChannelFrequencyS2;
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
#endif	/* CFG_ENABLE_WIFI_MESH */
#endif	/* _MESH_H */
