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

/*! \file   "nic_rate.c"
 *    \brief  This file contains the transmission rate handling routines.
 *
 *    This file contains the transmission rate handling routines for setting up
 *    ACK/CTS Rate, Highest Tx Rate, Lowest Tx Rate, Initial Tx Rate and do
 *    conversion between Rate Set and Data Rates.
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

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
/* in uint of 100kb/s */
uint16_t au2CckDataRate[] =
{
	10,
	20,
	55,
	110
};

/* in uint of 100kb/s */
uint16_t au2OfdmDataRate[] =
{
	60,
	90,
	120,
	180,
	240,
	360,
	480,
	540
};

/* in uint of 100kb/s */
HAL_HT_VHT_DATA_RATE_INFO arHtVhtDataRateInfo1SS[] =
{
	{  65,   72,  135,  150,  293,  325,  585,  650},		// HT: MCS0,  VHT: 1SS MCS0
	{  130,  144,  270,  300,  585,  650, 1170, 1300},		// HT: MCS1,  VHT: 1SS MCS1
	{  195,  217,  405,  450,  878,  975, 1755, 1950},		// HT: MCS2,  VHT: 1SS MCS2
	{  260,  289,  540,  600, 1170, 1300, 2340, 2600},		// HT: MCS3,  VHT: 1SS MCS3
	{  390,  433,  810,  900, 1755, 1950, 3510, 3900},		// HT: MCS4,  VHT: 1SS MCS4
	{  520,  578, 1080, 1200, 2340, 2600, 4680, 5200},		// HT: MCS5,  VHT: 1SS MCS5
	{  585,  650, 1215, 1350, 2633, 2925, 5265, 5850},		// HT: MCS6,  VHT: 1SS MCS6
	{  650,  722, 1350, 1500, 2925, 3250, 5850, 6500},		// HT: MCS7,  VHT: 1SS MCS7
	{  780,  867, 1620, 1800, 3510, 3900, 7020, 7800},		//            VHT: 1SS MCS8
	{  867,  963, 1800, 2000, 3900, 4333, 7800, 8667},		//            VHT: 1SS MCS9
	{  975, 1083, 2025, 2250, 4388, 4875, 8775, 9750},		//            VHT: 1SS MCS10
	{ 1083, 1204, 2250, 2500, 4875, 5417, 9750, 10833}		//            VHT: 1SS MCS11
};

/* in uint of 100kb/s */
HAL_HT_VHT_DATA_RATE_INFO arHtVhtDataRateInfo2SS[] =
{
	{  130,  144,  270,  300,  585,  650, 1170, 1300},		// HT: MCS8,  VHT: 2SS MCS0
	{  260,  289,  540,  600, 1170, 1300, 2340, 2600},		// HT: MCS9,  VHT: 2SS MCS1
	{  390,  433,  810,  900, 1755, 1950, 3510, 3900},		// HT: MCS10, VHT: 2SS MCS2
	{  520,  578, 1080, 1200, 2340, 2600, 4680, 5200},		// HT: MCS11, VHT: 2SS MCS3
	{  780,  867, 1620, 1800, 3510, 3900, 7020, 7800},		// HT: MCS12, VHT: 2SS MCS4
	{ 1040, 1156, 2160, 2400, 4680, 5200, 9360, 10400}, 		// HT: MCS13, VHT: 2SS MCS5
	{ 1170, 1300, 2430, 2700, 5265, 5850, 10530, 11700},		// HT: MCS14, VHT: 2SS MCS6
	{ 1300, 1444, 2700, 3000, 5850, 6500, 11700, 13000},		// HT: MCS15, VHT: 2SS MCS7
	{ 1560, 1733, 3240, 3600, 7020, 7800, 14040, 15600},		//            VHT: 2SS MCS8
	{ 1733, 1926, 3600, 4000, 7800, 8667, 15600, 17333},	        //            VHT: 2SS MCS9
	{ 1950, 2167, 4050, 4500, 8775, 9750, 17550, 19500},		//            VHT: 2SS MCS10
	{ 2167, 2407, 4500, 5000, 9750, 10833, 19500, 21667}		//            VHT: 2SS MCS11
};

/* in uint of 100kb/s */
HAL_HT_VHT_DATA_RATE_INFO arHtVhtDataRateInfo3SS[] =
{
	{  195,  217,  405,  450,  878,  975, 1755, 1950},		// HT: MCS16, VHT: 3SS MCS0
	{  390,  433,  810,  900, 1755, 1950, 3510, 3900},		// HT: MCS17, VHT: 3SS MCS1
	{  585,  650, 1215, 1350, 2633, 2925, 5265, 5850},		// HT: MCS18, VHT: 3SS MCS2
	{  780,  867, 1620, 1800, 3510, 3900, 7020, 7800},		// HT: MCS19, VHT: 3SS MCS3
	{ 1170, 1300, 2430, 2700, 5265, 5850, 10530, 11700},		// HT: MCS20, VHT: 3SS MCS4
	{ 1560, 1733, 3240, 3600, 7020, 7800, 14040, 15600},		// HT: MCS21, VHT: 3SS MCS5
	{ 1755, 1950, 3645, 4050, 7896, 8775, 15795, 17550},		// HT: MCS22, VHT: 3SS MCS6
	{ 1950, 2167, 4050, 4500, 8775, 9750, 17550, 19500},		// HT: MCS23, VHT: 3SS MCS7
	{ 2340, 2600, 4860, 5400, 10530, 11700, 21600, 23400},		//            VHT: 3SS MCS8
	{ 2600, 2889, 5400, 6000, 11700, 13000, 23400, 26000},		//            VHT: 3SS MCS9
	{ 2925, 3250, 6075, 6750, 13163, 14625, 26325, 29250},		//            VHT: 3SS MCS10
	{ 3250, 3611, 6750, 7500, 14625, 16250, 29250, 32500}		//            VHT: 3SS MCS11
};

/* in uint of 100kb/s */
HAL_HT_VHT_DATA_RATE_INFO arHtVhtDataRateInfo4SS[] =
{
	{  260,  289,  540,  600, 1170, 1300, 2340, 2600},		// HT: MCS24, VHT: 4SS MCS0
	{  520,  578, 1080, 1200, 2340, 2600, 4680, 5200},		// HT: MCS25, VHT: 4SS MCS1
	{  780,  867, 1620, 1800, 3510, 3900, 7020, 7800},		// HT: MCS26, VHT: 4SS MCS2
	{ 1040, 1156, 2160, 2400, 4680, 5200, 9360, 10400}, 		// HT: MCS27, VHT: 4SS MCS3
	{ 1560, 1733, 3240, 3600, 7020, 7800, 14040, 15600},		// HT: MCS28, VHT: 4SS MCS4
	{ 2080, 2311, 4320, 4800, 9360, 10400, 18720, 20800},		// HT: MCS29, VHT: 4SS MCS5
	{ 2340, 2600, 4860, 5400, 10530, 11700, 21060, 23400},		// HT: MCS30, VHT: 4SS MCS6
	{ 2600, 2889, 5400, 6000, 11700, 13000, 23400, 26000},		// HT: MCS31, VHT: 4SS MCS7
	{ 3120, 3467, 6480, 7200, 14040, 15600, 28080, 31200},		//            VHT: 4SS MCS8
	{ 3467, 3852, 7200, 8000, 15600, 17333, 31200, 34667},		//            VHT: 4SS MCS9
	{ 3900, 4333, 8100, 9000, 17550, 19500, 35100, 39000},		//            VHT: 4SS MCS10
	{ 4333, 4815, 9000, 10000, 19500, 21668, 39000, 43333}		//            VHT: 4SS MCS11
};

const uint16_t au2RateCCKLong[CCK_RATE_NUM] = {
	RATE_CCK_1M_LONG,	/* RATE_1M_INDEX = 0 */
	RATE_CCK_2M_LONG,	/* RATE_2M_INDEX */
	RATE_CCK_5_5M_LONG,	/* RATE_5_5M_INDEX */
	RATE_CCK_11M_LONG	/* RATE_11M_INDEX */
};

const uint16_t au2RateCCKShort[CCK_RATE_NUM] = {
	RATE_CCK_1M_LONG,	/* RATE_1M_INDEX = 0 */
	RATE_CCK_2M_SHORT,	/* RATE_2M_INDEX */
	RATE_CCK_5_5M_SHORT,	/* RATE_5_5M_INDEX */
	RATE_CCK_11M_SHORT	/* RATE_11M_INDEX */
};

const uint16_t au2RateOFDM[OFDM_RATE_NUM] = {
	RATE_OFDM_6M,		/* RATE_6M_INDEX */
	RATE_OFDM_9M,		/* RATE_9M_INDEX */
	RATE_OFDM_12M,		/* RATE_12M_INDEX */
	RATE_OFDM_18M,		/* RATE_18M_INDEX */
	RATE_OFDM_24M,		/* RATE_24M_INDEX */
	RATE_OFDM_36M,		/* RATE_36M_INDEX */
	RATE_OFDM_48M,		/* RATE_48M_INDEX */
	RATE_OFDM_54M		/* RATE_54M_INDEX */
};

const uint16_t au2RateHTMixed[HT_RATE_NUM] = {
	RATE_MM_MCS_32,		/* RATE_MCS32_INDEX, */
	RATE_MM_MCS_0,		/* RATE_MCS0_INDEX, */
	RATE_MM_MCS_1,		/* RATE_MCS1_INDEX, */
	RATE_MM_MCS_2,		/* RATE_MCS2_INDEX, */
	RATE_MM_MCS_3,		/* RATE_MCS3_INDEX, */
	RATE_MM_MCS_4,		/* RATE_MCS4_INDEX, */
	RATE_MM_MCS_5,		/* RATE_MCS5_INDEX, */
	RATE_MM_MCS_6,		/* RATE_MCS6_INDEX, */
	RATE_MM_MCS_7		/* RATE_MCS7_INDEX, */
};

const uint16_t au2RateHTGreenField[HT_RATE_NUM] = {
	RATE_GF_MCS_32,		/* RATE_MCS32_INDEX, */
	RATE_GF_MCS_0,		/* RATE_MCS0_INDEX, */
	RATE_GF_MCS_1,		/* RATE_MCS1_INDEX, */
	RATE_GF_MCS_2,		/* RATE_MCS2_INDEX, */
	RATE_GF_MCS_3,		/* RATE_MCS3_INDEX, */
	RATE_GF_MCS_4,		/* RATE_MCS4_INDEX, */
	RATE_GF_MCS_5,		/* RATE_MCS5_INDEX, */
	RATE_GF_MCS_6,		/* RATE_MCS6_INDEX, */
	RATE_GF_MCS_7,		/* RATE_MCS7_INDEX, */
};

const uint16_t au2RateVHT[VHT_RATE_NUM] = {
	RATE_VHT_MCS_0,		/* RATE_MCS0_INDEX, */
	RATE_VHT_MCS_1,		/* RATE_MCS1_INDEX, */
	RATE_VHT_MCS_2,		/* RATE_MCS2_INDEX, */
	RATE_VHT_MCS_3,		/* RATE_MCS3_INDEX, */
	RATE_VHT_MCS_4,		/* RATE_MCS4_INDEX, */
	RATE_VHT_MCS_5,		/* RATE_MCS5_INDEX, */
	RATE_VHT_MCS_6,		/* RATE_MCS6_INDEX, */
	RATE_VHT_MCS_7,		/* RATE_MCS7_INDEX, */
	RATE_VHT_MCS_8,		/* RATE_MCS8_INDEX, */
	RATE_VHT_MCS_9		/* RATE_MCS9_INDEX, */
};

/* in unit of 100kb/s */
const struct EMU_MAC_RATE_INFO arMcsRate2PhyRate[] = {
	/* Phy Rate Code,
	 * BW20,  BW20 SGI, BW40, BW40 SGI, BW80, BW80 SGI, BW160, BW160 SGI
	 */
	RATE_INFO(PHY_RATE_MCS0, 65, 72, 135, 150, 293, 325, 585, 650),
	RATE_INFO(PHY_RATE_MCS1, 130, 144, 270, 300, 585, 650, 1170, 1300),
	RATE_INFO(PHY_RATE_MCS2, 195, 217, 405, 450, 878, 975, 1755, 1950),
	RATE_INFO(PHY_RATE_MCS3, 260, 289, 540, 600, 1170, 1300, 2340, 2600),
	RATE_INFO(PHY_RATE_MCS4, 390, 433, 810, 900, 1755, 1950, 3510, 3900),
	RATE_INFO(PHY_RATE_MCS5, 520, 578, 1080, 1200, 2340, 2600, 4680, 5200),
	RATE_INFO(PHY_RATE_MCS6, 585, 650, 1215, 1350, 2633, 2925, 5265, 5850),
	RATE_INFO(PHY_RATE_MCS7, 650, 722, 1350, 1500, 2925, 3250, 5850, 6500),
	RATE_INFO(PHY_RATE_MCS8, 780, 867, 1620, 1800, 3510, 3900, 7020, 7800),
	RATE_INFO(PHY_RATE_MCS9, 867, 963, 1800, 2000, 3900, 4333, 7800, 8667),
	RATE_INFO(PHY_RATE_MCS32, 0, 0, 60, 67, 0, 0, 0, 0)
};

/* in uint of 500kb/s */
const uint8_t aucHwRate2PhyRate[] = {
	RATE_1M,		/*1M long */
	RATE_2M,		/*2M long */
	RATE_5_5M,		/*5.5M long */
	RATE_11M,		/*11M long */
	RATE_1M,		/*1M short invalid */
	RATE_2M,		/*2M short */
	RATE_5_5M,		/*5.5M short */
	RATE_11M,		/*11M short */
	RATE_48M,		/*48M */
	RATE_24M,		/*24M */
	RATE_12M,		/*12M */
	RATE_6M,		/*6M */
	RATE_54M,		/*54M */
	RATE_36M,		/*36M */
	RATE_18M,		/*18M */
	RATE_9M			/*9M */
};

const uint16_t au2PhyCodeOFDM[OFDM_RATE_NUM] = {
	PHY_RATE_6M,		/* RATE_6M_INDEX */
	PHY_RATE_9M,		/* RATE_9M_INDEX */
	PHY_RATE_12M,		/* RATE_12M_INDEX */
	PHY_RATE_18M,		/* RATE_18M_INDEX */
	PHY_RATE_24M,		/* RATE_24M_INDEX */
	PHY_RATE_36M,		/* RATE_36M_INDEX */
	PHY_RATE_48M,		/* RATE_48M_INDEX */
	PHY_RATE_54M		/* RATE_54M_INDEX */
};

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
uint32_t
nicGetPhyRateByMcsRate(
	IN uint8_t ucIdx,
	IN uint8_t ucBw,
	IN uint8_t ucGI)
{
	return	arMcsRate2PhyRate[ucIdx].u4PhyRate[ucBw][ucGI];
}

uint32_t
nicGetHwRateByPhyRate(
	IN uint8_t ucIdx)
{
	return	aucHwRate2PhyRate[ucIdx]; /* uint : 500 kbps */
}

uint32_t
nicSwIndex2RateIndex(
	IN uint8_t ucRateSwIndex,
	OUT uint8_t *pucRateIndex,
	OUT uint8_t *pucPreambleOption
)
{
	ASSERT(pucRateIndex);
	ASSERT(pucPreambleOption);

	if (ucRateSwIndex >= RATE_6M_SW_INDEX) {
		*pucRateIndex = ucRateSwIndex - RATE_6M_SW_INDEX;
		*pucPreambleOption = PREAMBLE_OFDM_MODE;
	} else {
		*pucRateIndex = ucRateSwIndex;
		*pucPreambleOption = PREAMBLE_DEFAULT_LONG_NONE;
	}
	return WLAN_STATUS_SUCCESS;
}

uint32_t nicRateIndex2RateCode(IN uint8_t ucPreambleOption,
	IN uint8_t ucRateIndex, OUT uint16_t *pu2RateCode)
{
	switch (ucPreambleOption) {
	case PREAMBLE_DEFAULT_LONG_NONE:
		if (ucRateIndex >= CCK_RATE_NUM)
			return WLAN_STATUS_INVALID_DATA;
		*pu2RateCode = au2RateCCKLong[ucRateIndex];
		break;

	case PREAMBLE_OPTION_SHORT:
		if (ucRateIndex >= CCK_RATE_NUM)
			return WLAN_STATUS_INVALID_DATA;
		*pu2RateCode = au2RateCCKShort[ucRateIndex];
		break;

	case PREAMBLE_OFDM_MODE:
		if (ucRateIndex >= OFDM_RATE_NUM)
			return WLAN_STATUS_INVALID_DATA;
		*pu2RateCode = au2RateOFDM[ucRateIndex];
		break;

	case PREAMBLE_HT_MIXED_MODE:
		if (ucRateIndex >= HT_RATE_NUM)
			return WLAN_STATUS_INVALID_DATA;
		*pu2RateCode = au2RateHTMixed[ucRateIndex];
		break;

	case PREAMBLE_HT_GREEN_FIELD:
		if (ucRateIndex >= HT_RATE_NUM)
			return WLAN_STATUS_INVALID_DATA;
		*pu2RateCode = au2RateHTGreenField[ucRateIndex];
		break;

	case PREAMBLE_VHT_FIELD:
		if (ucRateIndex >= VHT_RATE_NUM)
			return WLAN_STATUS_INVALID_DATA;
		*pu2RateCode = au2RateVHT[ucRateIndex];
		break;

	default:
		return WLAN_STATUS_INVALID_DATA;
	}

	return WLAN_STATUS_SUCCESS;
}

uint32_t
nicRateCode2PhyRate(
	IN uint16_t  u2RateCode,
	IN uint8_t   ucBandwidth,
	IN uint8_t   ucGI,
	IN uint8_t   ucRateNss)
{
	uint8_t ucPhyRate;
	uint16_t u2TxMode;
	uint32_t u4PhyRateBy1SS, u4PhyRateIn100Kbps = 0;

	ucPhyRate = RATE_CODE_GET_PHY_RATE(u2RateCode);
	u2TxMode = u2RateCode & RATE_TX_MODE_MASK;
	ucRateNss = ucRateNss + AR_SS_1; /* change to be base=1 */

	if ((u2TxMode == TX_MODE_HT_GF)
	    || (u2TxMode == TX_MODE_HT_MM)) {

		if (ucPhyRate > PHY_RATE_MCS7)
			u2RateCode = u2RateCode - HT_RATE_MCS7_INDEX;
		else
			ucRateNss = AR_SS_1;

	} else if ((u2TxMode == TX_MODE_OFDM)
		   || (u2TxMode == TX_MODE_CCK)) {
		ucRateNss = AR_SS_1;
	}
	DBGLOG(NIC, LOUD,
	       "Coex:nicRateCode2PhyRate,RC:%x,B:%d,I:%d\n",
	       u2RateCode, ucBandwidth, ucGI);

	u4PhyRateBy1SS = nicRateCode2DataRate(u2RateCode,
					      ucBandwidth, ucGI);
	u4PhyRateIn100Kbps = u4PhyRateBy1SS * ucRateNss;

	DBGLOG(NIC, LOUD,
	       "Coex:nicRateCode2PhyRate,1ss R:%d,PHY R:%d\n",
	       u4PhyRateBy1SS, u4PhyRateIn100Kbps);

	return u4PhyRateIn100Kbps;
}

uint32_t
nicRateCode2DataRate(
	IN uint16_t  u2RateCode,
	IN uint8_t   ucBandwidth,
	IN uint8_t   ucGI)
{
	uint8_t ucPhyRate, ucIdx, ucBw = 0;
	uint32_t u4PhyRateIn100Kbps = 0;
	uint16_t u2TxMode;

	if ((ucBandwidth == FIX_BW_NO_FIXED)
	    || (ucBandwidth == FIX_BW_20))
		ucBw = MAC_BW_20;
	else if (ucBandwidth == FIX_BW_40)
		ucBw = MAC_BW_40;
	else if (ucBandwidth == FIX_BW_80)
		ucBw = MAC_BW_80;
	else if (ucBandwidth == FIX_BW_160)
		ucBw = MAC_BW_160;

	ucPhyRate = RATE_CODE_GET_PHY_RATE(u2RateCode);
	u2TxMode = u2RateCode & RATE_TX_MODE_MASK;
	/* Set MMSS parameter if HT/VHT rate */
	if ((u2TxMode == TX_MODE_HT_GF) ||
	    (u2TxMode == TX_MODE_HT_MM) ||
	    (u2TxMode == TX_MODE_VHT)) {
		/* No SGI Greenfield for 1T */
		/* Refer to section 20.3.11.11.6 of IEEE802.11-2012 */
		if (u2TxMode == TX_MODE_HT_GF)
			ucGI = MAC_GI_NORMAL;

		ucIdx = ucPhyRate;

		if (ucIdx == PHY_RATE_MCS32)
			ucIdx = 10;

		u4PhyRateIn100Kbps = nicGetPhyRateByMcsRate(ucIdx, ucBw,
				     ucGI);
	} else if ((u2TxMode == TX_MODE_OFDM) ||
		   (u2TxMode == TX_MODE_CCK)) {
		u4PhyRateIn100Kbps = (nicGetHwRateByPhyRate(
					      ucPhyRate & BITS(0, 3))) * 5;
	} else {
		ASSERT(FALSE);
	}
	return u4PhyRateIn100Kbps;
}

u_int8_t
nicGetRateIndexFromRateSetWithLimit(
	IN uint16_t u2RateSet,
	IN uint32_t u4PhyRateLimit,
	IN u_int8_t fgGetLowest,
	OUT uint8_t *pucRateSwIndex)
{
	uint32_t i;
	uint32_t u4CurPhyRate, u4TarPhyRate, u4HighestPhyRate,
		 u4LowestPhyRate;
	uint8_t ucRateIndex, ucRatePreamble, ucTarRateSwIndex,
		ucHighestPhyRateSwIdx, ucLowestPhyRateSwIdx;
	uint16_t u2CurRateCode;
	uint32_t u4Status;

	/* Set init value */
	if (fgGetLowest) {
		u4TarPhyRate = 0xFFFFFFFF;
		u4HighestPhyRate = 0;
		ucHighestPhyRateSwIdx = RATE_NUM_SW;
	} else {
		u4TarPhyRate = 0;
		u4LowestPhyRate = 0xFFFFFFFF;
		ucLowestPhyRateSwIdx = RATE_NUM_SW;
	}

	ucTarRateSwIndex = RATE_NUM_SW;

	/* Find SW rate index by limitation */
	for (i = RATE_1M_SW_INDEX; i <= RATE_54M_SW_INDEX; i++) {
		if (u2RateSet & BIT(i)) {

			/* Convert SW rate index to phy rate in 100kbps */
			nicSwIndex2RateIndex(i, &ucRateIndex, &ucRatePreamble);
			u4Status = nicRateIndex2RateCode(ucRatePreamble,
				ucRateIndex, &u2CurRateCode);

			if (u4Status != WLAN_STATUS_SUCCESS)
				continue;

			u4CurPhyRate =
				nicRateCode2DataRate(u2CurRateCode, MAC_BW_20,
						     MAC_GI_NORMAL);

			/* Compare */
			if (fgGetLowest) {
				if (u4HighestPhyRate < u4CurPhyRate) {
					u4HighestPhyRate = u4CurPhyRate;
					ucHighestPhyRateSwIdx = i;
				}
				if ((u4CurPhyRate >= u4PhyRateLimit)
				    && (u4CurPhyRate <= u4TarPhyRate)) {
					u4TarPhyRate = u4CurPhyRate;
					ucTarRateSwIndex = i;
				}
			} else {
				if (u4LowestPhyRate > u4CurPhyRate) {
					u4LowestPhyRate = u4CurPhyRate;
					ucLowestPhyRateSwIdx = i;
				}
				if ((u4CurPhyRate <= u4PhyRateLimit)
				    && (u4CurPhyRate >= u4TarPhyRate)) {
					u4TarPhyRate = u4CurPhyRate;
					ucTarRateSwIndex = i;
				}
			}
		}
	}

	/* Return target SW rate index */
	if (ucTarRateSwIndex < RATE_NUM_SW) {
		*pucRateSwIndex = ucTarRateSwIndex;
	} else {
		if (fgGetLowest)
			*pucRateSwIndex = ucHighestPhyRateSwIdx;
		else
			*pucRateSwIndex = ucLowestPhyRateSwIdx;
	}
	return TRUE;
}

/* do the mapping of RX rate in RXV to ieee80211_rx_status.rate_idx */
u_int8_t
nicRateCode2RateIdx(
	IN u_int8_t ucRateCode,
	IN u_int8_t ucRxMode,
	IN enum ENUM_BAND eBand
)
{
	u_int8_t ucIdx = 0;
	int i = 0;

	switch (ucRxMode) {
	case RX_VT_LEGACY_CCK:
		ucIdx = ucRateCode & RX_VT_RX_RATE_CCK_MASK;
		break;

	case RX_VT_LEGACY_OFDM:
		for (i = 0; i < OFDM_RATE_NUM; i++)
			if (au2PhyCodeOFDM[i] ==
			    (ucRateCode & RX_VT_RX_RATE_OFDM_MASK))
				break;
		if (i == OFDM_RATE_NUM) {
			DBGLOG(NIC, WARN, "OFDM rate index not found\n");
			ucIdx = 0;
		} else
			ucIdx = i;

		switch (eBand) {
		case BAND_2G4:
			ucIdx += 4;
			break;

		case BAND_5G:
			break;

		default:
			DBGLOG(NIC, WARN, "wrong band information\n");
			break;
		}

		break;

	case RX_VT_MIXED_MODE:
	case RX_VT_GREEN_MODE:
		ucIdx = ucRateCode & RX_VT_RX_RATE_MASK;
		break;

	case RX_VT_VHT_MODE:
		ucIdx = ucRateCode & RX_VT_RX_RATE_AC_MASK;
		break;

	default:
		DBGLOG(NIC, WARN, "wrong RX mode\n");
		ucIdx = 0;
		break;
	}

	return ucIdx;
}

static u_int16_t hw_rate_ofdm(u_int8_t ofdm_idx)
{
	switch (ofdm_idx) {
	case 11: /* 6M */
		return au2OfdmDataRate[0];
	case 15: /* 9M */
		return au2OfdmDataRate[1];
	case 10: /* 12M */
		return au2OfdmDataRate[2];
	case 14: /* 18M */
		return au2OfdmDataRate[3];
	case 9: /* 24M */
		return au2OfdmDataRate[4];
	case 13: /* 36M */
		return au2OfdmDataRate[5];
	case 8: /* 48M */
		return au2OfdmDataRate[6];
	case 12: /* 54M */
		return au2OfdmDataRate[7];
	default:
		return au2OfdmDataRate[0];
	}
}

/*----------------------------------------------------------------------------*/
/*!
* \brief     get data rate by mode, mcs, bw, sgi
*
* \param[in] ucMode
* \param[in] ucMcs
* \param[in] ucVhtNss
* \param[in] ucBw
* \param[in] ucSgi
*
* \return    Phy rate in uint of 100kb/s
*/
/*----------------------------------------------------------------------------*/
u_int16_t
nicGetDataRate(
    IN u_int32_t ucMode,
    IN u_int32_t ucMcs,
    IN u_int32_t ucVhtNss,
    IN u_int32_t ucBw,
    IN u_int32_t ucSgi
)
{
	P_HAL_HT_VHT_DATA_RATE_INFO prPhyRateInfo;
	u_int16_t u4PhyRate = 0;
	u_int8_t ucNss = 1;
	u_int8_t ucIdx;

	DBGLOG(NIC, LOUD, "RX mode %d, mcs %d, nss %d, BW %d, gi %d\n",
				ucMode,
				ucMcs,
				ucVhtNss,
				ucBw,
				ucSgi);

	if (ucMode == TX_RATE_MODE_CCK)
	{
		if (ucMcs >= CCK_RATE_NUM)
		{
			return 0;
		}

		return au2CckDataRate[ucMcs];
	}
	else if (ucMode == TX_RATE_MODE_OFDM)
	{
		return hw_rate_ofdm(ucMcs);
	}
	else if (ucMode == TX_RATE_MODE_HTMIX || ucMode == TX_RATE_MODE_HTGF)
	{
		if (ucMcs == RX_VT_RX_RATE_MCS32)
		{
			/* For MCS32 LGI return 6 (6 Mbps) and for SGI return 67 (6.7 Mbps) */
			return ucSgi == 0 ? 60 : 67;
		}
		else if (ucMcs <= RX_VT_RX_RATE_MCS31)
		{
			ucNss += (ucMcs >> 3);
			ucIdx = ucMcs & 0x7;
		}
		else
		{
			return 0;
		}
	}
	else if (ucMode == TX_RATE_MODE_VHT)
	{
		if (ucMcs <= VHT_RATE_MCS9_INDEX)
		{
			ucNss = ucVhtNss;
			ucIdx = ucMcs;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}

	switch (ucNss)
	{
		case 1:
			prPhyRateInfo = &arHtVhtDataRateInfo1SS[ucIdx];
			break;

		case 2:
			prPhyRateInfo = &arHtVhtDataRateInfo2SS[ucIdx];
			break;

		case 3:
			prPhyRateInfo = &arHtVhtDataRateInfo3SS[ucIdx];
			break;

		case 4:
			prPhyRateInfo = &arHtVhtDataRateInfo4SS[ucIdx];
			break;

		default:
			return 0;
	}

	switch (ucBw)
	{
		case RX_VT_FR_MODE_20:
			u4PhyRate = (ucSgi ? prPhyRateInfo->u2Bw20SGI : prPhyRateInfo->u2Bw20LGI);
			break;

		case RX_VT_FR_MODE_40:
			u4PhyRate = (ucSgi ? prPhyRateInfo->u2Bw40SGI : prPhyRateInfo->u2Bw40LGI);
			break;

		case RX_VT_FR_MODE_80:
			u4PhyRate = (ucSgi ? prPhyRateInfo->u2Bw80SGI : prPhyRateInfo->u2Bw80LGI);
			break;

		case RX_VT_FR_MODE_160:
			u4PhyRate = (ucSgi ? prPhyRateInfo->u2Bw160SGI : prPhyRateInfo->u2Bw160LGI);
			break;

		default:
			return 0;
	}

	return u4PhyRate;
}

