/*
 * Copyright (c) 2015-2016 MediaTek Inc.
 * Author: Houlong Wei <houlong.wei@mediatek.com>
 *         Ming Hsiu Tsai <minghsiu.tsai@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MTK_MDP_IPI_H__
#define __MTK_MDP_IPI_H__

#define MTK_MDP_MAX_NUM_PLANE		3

#define MTK_MDP_CONT_FLAG		BIT(1)
#define MTK_MDP_BRIT_FLAG		BIT(2)

enum mdp_ipi_msgid {
	AP_MDP_INIT		= 0xd000,
	AP_MDP_DEINIT		= 0xd001,
	AP_MDP_PROCESS		= 0xd002,
	AP_MDP_CMDQ_DONE	= 0xd003,
	AP_MDP_TUNING		= 0xd004,
	AP_MDP_PQ_CONFIG	= 0xd005,

	VPU_MDP_INIT_ACK	= 0xe000,
	VPU_MDP_DEINIT_ACK	= 0xe001,
	VPU_MDP_PROCESS_ACK	= 0xe002,
	VPU_MDP_CMDQ_DONE_ACK	= 0xe003,
	VPU_MDP_TUNING_ACK	= 0xe004,
	VPU_MDP_PQ_CONFIG_ACK	= 0xe005
};

#pragma pack(push, 4)

/**
 * struct mdp_ipi_init - for AP_MDP_INIT
 * @msg_id   : AP_MDP_INIT
 * @ipi_id   : IPI_MDP
 * @ap_inst  : AP mtk_mdp_vpu address
 */
struct mdp_ipi_init {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint64_t ap_inst;
};

/**
 * struct mdp_ipi_comm - for AP_MDP_PROCESS, AP_MDP_DEINIT
 * @msg_id        : AP_MDP_PROCESS, AP_MDP_DEINIT
 * @ipi_id        : IPI_MDP
 * @ap_inst       : AP mtk_mdp_vpu address
 * @vpu_inst_addr : VPU MDP instance address
 */
struct mdp_ipi_comm {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint64_t ap_inst;
	uint64_t vpu_inst_addr;
};

/**
 * struct mdp_ipi_comm_ack - for VPU_MDP_DEINIT_ACK, VPU_MDP_PROCESS_ACK
 * @msg_id        : VPU_MDP_DEINIT_ACK, VPU_MDP_PROCESS_ACK
 * @ipi_id        : IPI_MDP
 * @ap_inst       : AP mtk_mdp_vpu address
 * @vpu_inst_addr : VPU MDP instance address
 * @status        : VPU exeuction result
 */
struct mdp_ipi_comm_ack {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint64_t ap_inst;
	uint64_t vpu_inst_addr;
	int32_t status;
};

/**
 * struct mdp_config - configured for source/destination image
 * @x        : left
 * @y        : top
 * @w        : width
 * @h        : height
 * @crop_x   : cropped left
 * @crop_y   : cropped top
 * @crop_w   : cropped width
 * @crop_h   : cropped height
 * @format   : color format
 * @pitch    : bytes per line for each plane
 */
struct mdp_config {
	int32_t x;
	int32_t y;
	int32_t w;
	int32_t h;
	int32_t crop_x;
	int32_t crop_y;
	int32_t crop_w;
	int32_t crop_h;
	int32_t format;
	uint32_t pitch[MTK_MDP_MAX_NUM_PLANE];
};

struct mdp_buffer {
	uint64_t addr_mva[MTK_MDP_MAX_NUM_PLANE];
	int32_t plane_size[MTK_MDP_MAX_NUM_PLANE];
	int32_t plane_num;
};

struct mdp_config_misc {
	int32_t orientation; /* 0, 90, 180, 270 */
	int32_t hflip; /* 1 will enable the flip */
	int32_t vflip; /* 1 will enable the flip */
	int32_t alpha; /* global alpha */
};

/**
 * struct mdp_cmdq_info - command queue information
 * @engine_flag      : bit flag of engines used.
 * @vpu_buf_addr     : pointer to instruction buffer in vpu.
 *                          This must point to an 64-bit aligned uint32_t array.
 * @ap_buf_addr      : pointer to instruction buffer in ap.
 * @ap_buf_pa        : physical address of instruction buffer in ap.
 * @buf_size         : size of buffer, in bytes.
 * @cmd_offset       : offset of real instruction start pointer relative to
 *                     buf_addr.
 * @cmd_size         : used size, in bytes.
 * @regr_count       : number of requesting to read register values
 * @regr_addr_offset : offset of read registers addresse relative to buf_addr.
 * @regr_val_offset  : offset of read back registers values address relative to
 *                     buf_addr.
 */
struct mdp_cmdq_info {
	uint64_t engine_flag;
	uint64_t vpu_buf_addr;
	uint64_t ap_buf_addr;
	uint64_t ap_buf_pa;
	uint32_t buf_size;
	uint32_t cmd_offset;
	uint32_t cmd_size;
	uint32_t reserved;
};

/**
 * struct mdp_pq_info - picture quality information
 * @sharpness_enable: sharpness enable.
 * @sharpness_level: sharpness level.
 * @dc_enable: dynamic contrast enable.
 * @brightness_enable:brightness enable
 * @brightness_level: brightness level
 * @contrast_enable: contrast enable
 * @contrast_level: contrast level
 */
struct mdp_pq_info {
	uint32_t sharpness_enable;
	uint32_t sharpness_level;
	uint32_t dynamic_contrast_enable;
	uint32_t brightness_enable:1;
	uint32_t brightness_level:15;
	uint32_t contrast_enable:1;
	uint32_t contrast_level:15;
	uint32_t gamma_enable;
	uint32_t gamma_type;
	uint8_t gamma_table[256];
	uint32_t invert;
	uint32_t dth_enable;
	uint32_t dth_algo;
	uint32_t rsz_algo;
};

/**
 * struct mdp_blur_info - rsz blur information
 * @hor_blur_enable : horizontal blur enable.
 * @ver_blur_enable : vertical blur enable.
 * @hor_blur_level  : horizontal blur level.
 * @ver_blur_level  : vertical blur level.
 */
struct mdp_blur_info {
	uint32_t hor_blur_enable;
	uint32_t ver_blur_enable;
	uint32_t hor_blur_level;
	uint32_t ver_blur_level;
};

struct mdp_process_vsi {
	struct mdp_config src_config;
	struct mdp_buffer src_buffer;
	struct mdp_config dst_config;
	struct mdp_buffer dst_buffer;
	struct mdp_config_misc misc;
	struct mdp_cmdq_info cmdq;
	struct mdp_pq_info pq;
	struct mdp_blur_info blur;
	uint32_t reserved[32];
};

/**
 * struct mdp_pq_cfg - mdp pq config information
 * @msg_id        : AP_MDP_PQ_CONFIG
 * @ipi_id        : IPI_MDP
 * @store_para_type : store pq info (brightness | contrast).
 * @get_para_type : get pq info (brightness | contrast).
 * @store_cont_level  : store contrast level.
 * @store_brit_level  : store brightness  level.
 * @get_ont_level  : get contrast level.
 * @get_brit_level  : get brightness  level.
 */
struct mdp_pq_cfg {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint32_t store_para_type;
	uint32_t get_para_type;
	uint32_t store_cont_level;
	uint32_t store_brit_level;
	uint32_t get_cont_level;
	uint32_t get_brit_level;
	uint64_t ap_inst;
};

struct mdp_pq_cfg_ack {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint32_t get_para_type;
	uint32_t get_cont_level;
	uint32_t get_brit_level;
	int32_t status;
	uint64_t ap_inst;
};

#pragma pack(pop)

#endif /* __MTK_MDP_IPI_H__ */
