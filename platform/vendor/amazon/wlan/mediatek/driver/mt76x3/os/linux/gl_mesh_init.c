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
/*! \file   gl_mesh_init.c
* brief  init and exit routines of Linux driver interface for Wi-Fi Mesh
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
static struct MESH_WORK_DATA rMeshWorkData;
static struct work_struct rMeshWork;

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
uint32_t meshLaunch(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	uint32_t ret = WLAN_STATUS_SUCCESS;

	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	DBGLOG(MESH, INFO, "Mesh Launch...\n");
	if (prAdapter->fgIsMeshRegister) {
		DBGLOG(MESH, ERROR, "Mesh Already Register...\n");

		ret = WLAN_STATUS_NOT_ACCEPTED;
	} else if (!glRegisterMESH(prGlueInfo)) {
		DBGLOG(MESH, INFO, "Mesh Launch Success...\n");

		ret = WLAN_STATUS_SUCCESS;
	} else {
		DBGLOG(MESH, ERROR, "Mesh Launch Fail...\n");

		ret = WLAN_STATUS_FAILURE;
	}

	return ret;
}

uint32_t meshRemove(struct GLUE_INFO *prGlueInfo)
{
	struct ADAPTER *prAdapter;
	uint32_t ret = WLAN_STATUS_SUCCESS;

	ASSERT(prGlueInfo);
	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);

	if (!prAdapter->fgIsMeshRegister) {
		DBGLOG(MESH, ERROR, "Mesh Already Removed...\n");

		ret = WLAN_STATUS_NOT_ACCEPTED;
	} else if (!glUnregisterMESH(prGlueInfo)) {
		DBGLOG(MESH, INFO, "Mesh Removed...\n");

		ret = WLAN_STATUS_SUCCESS;
	} else {
		DBGLOG(MESH, ERROR, "Mesh Removed Fail...\n");

		ret = WLAN_STATUS_FAILURE;
	}

	return ret;
}

static int meshModeHandler(struct net_device *netdev,
			   struct PARAM_CUSTOM_MESH_SET *prSetMESH)
{
	struct GLUE_INFO *prGlueInfo = *((struct GLUE_INFO **) netdev_priv(netdev));
	struct WIFI_VAR *prWifiVar;

	ASSERT(prSetMESH);

	if (!prGlueInfo) {
		DBGLOG(MESH, ERROR, "Glueinfo is NULL, return\n");
		return -1;
	}

	DBGLOG(MESH, INFO, "PRIV_CMD_MESH_MODE=%u\n",
	       (uint32_t) prSetMESH->u4Enable);

	prWifiVar = &prGlueInfo->prAdapter->rWifiVar;

	if (!prSetMESH->u4Enable) {
		DBGLOG(MESH, ERROR, "meshRemove from wq\n");
		meshRemove(prGlueInfo);
	} else {
#if 0
		if (prGlueInfo->prDevHandler &&
		    (prGlueInfo->prDevHandler->flags & IFF_UP)) {
			DBGLOG(MESH, ERROR,
			       "STA IF is UP, Can't Launch Mesh\n");
			return -1;
		} else
#endif
		if (!prWifiVar->fgCapMesh) {
			DBGLOG(MESH, ERROR, "hw NOT support mesh\n");

			return -1;
		}

		meshLaunch(prGlueInfo);
	}

	return 0;
}

static void meshHandleWq(struct work_struct *work)
{
	struct MESH_WORK_DATA *prMeshWorkData = &rMeshWorkData;

	meshModeHandler(prMeshWorkData->netdev, &prMeshWorkData->rSetMESH);
}

int meshWorkSchedule(struct net_device *netdev,
		     struct PARAM_CUSTOM_MESH_SET *prSetMESH)
{
	struct GLUE_INFO *prGlueInfo  = *((struct GLUE_INFO **) netdev_priv(netdev));
	struct ADAPTER *prAdapter;

	if (prGlueInfo == NULL) {
		DBGLOG(MESH, ERROR, "Mesh No GlueInfo\n");
		return -EOPNOTSUPP;
	}

	prAdapter = prGlueInfo->prAdapter;
	ASSERT(prAdapter);
	ASSERT(prSetMESH);

	rMeshWorkData.netdev = netdev;
	kalMemCopy(&rMeshWorkData.rSetMESH, prSetMESH,
		   sizeof(struct PARAM_CUSTOM_MESH_SET));
	/* TODO: [mesh] It seems we should add some condition check before
	 * calling INIT_WORK
	 */
	/* Reference: https://lwn.net/Articles/23634/
	 * INIT_WORK must be used at least once before queueing the work_struct
	 * structure, but should not be used if the work_struct might already be
	 * in a workqueue.
	 */
	INIT_WORK(&rMeshWork, meshHandleWq);

	if (!prSetMESH->u4Enable && prAdapter->fgIsMeshRegister)
		schedule_work(&rMeshWork);
	else if (prSetMESH->u4Enable && !prAdapter->fgIsMeshRegister)
		schedule_work(&rMeshWork);

	return 0;
}
#endif	/* CFG_ENABLE_WIFI_MESH */
