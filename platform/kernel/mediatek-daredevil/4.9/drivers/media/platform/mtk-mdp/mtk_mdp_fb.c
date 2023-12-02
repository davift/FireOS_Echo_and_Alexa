/*
 * Copyright (c) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/errno.h>

#include "mtk_mdp_core.h"
#include "mtk_mdp_type.h"
#ifdef CONFIG_VIDEO_MEDIATEK_VCU
#include "mtk_mdp_vpu.h"
#else
#include "mtk_vpu.h"
#endif


static struct mtk_mdp_dev *g_mdp_dev;

void mtk_mdp_fb_init(struct mtk_mdp_dev *mdp_dev)
{
	g_mdp_dev = mdp_dev;
	mtk_mdp_dbg(0, "mdp_dev=%p", g_mdp_dev);
}

void mtk_mdp_fb_deinit(void)
{
	g_mdp_dev = NULL;
}

struct mtk_mdp_ctx *mtk_mdp_create_ctx(void)
{
	struct mtk_mdp_dev *mdp = g_mdp_dev;
	struct mtk_mdp_ctx *ctx = NULL;
	int ret;

	if (mdp == NULL) {
		mtk_mdp_err("mtk-mdp is not initialized");
		return NULL;
	}

	if (mutex_lock_interruptible(&mdp->lock)) {
		mtk_mdp_err("lock failed");
		return NULL;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		goto err_ctx_alloc;

	mutex_init(&ctx->slock);
	ctx->id = mdp->id_counter++;
	ctx->mdp_dev = mdp;
	if (mdp->ctx_num++ == 0) {
		ret = vpu_load_firmware(mdp->vpu_dev);
		if (ret < 0) {
			dev_info(&mdp->pdev->dev,
				"vpu_load_firmware failed %d\n", ret);
			goto err_load_vpu;
		}

		ret = mtk_mdp_vpu_register(mdp->pdev);
		if (ret < 0) {
			dev_info(&mdp->pdev->dev,
				"mdp_vpu register failed %d\n", ret);
			goto err_load_vpu;
		}
	}

	mtk_mdp_dbg(0, "ctx_num=%d", mdp->ctx_num);

	ret = mtk_mdp_vpu_init(&ctx->vpu);
	if (ret < 0) {
		dev_info(&mdp->pdev->dev, "Initialize vpu failed %d\n", ret);
		goto err_load_vpu;
	}

	mutex_unlock(&mdp->lock);

	mtk_mdp_dbg(0, "%s [%d]", dev_name(&mdp->pdev->dev), ctx->id);

	return ctx;

err_load_vpu:
	mdp->ctx_num--;
	kfree(ctx);
err_ctx_alloc:
	mutex_unlock(&mdp->lock);

	return NULL;
}
EXPORT_SYMBOL(mtk_mdp_create_ctx);

int mtk_mdp_destroy_ctx(struct mtk_mdp_ctx *ctx)
{
	struct mtk_mdp_dev *mdp = ctx->mdp_dev;

	mutex_lock(&mdp->lock);
	mtk_mdp_vpu_deinit(&ctx->vpu);
	mdp->ctx_num--;

	mtk_mdp_dbg(0, "%s [%d]", dev_name(&mdp->pdev->dev), ctx->id);
	mtk_mdp_dbg(0, "ctx_num=%d", mdp->ctx_num);

	mutex_unlock(&mdp->lock);
	kfree(ctx);

	return 0;
}
EXPORT_SYMBOL(mtk_mdp_destroy_ctx);

void mtk_mdp_set_input_addr(struct mtk_mdp_ctx *ctx,
				struct mtk_mdp_addr *addr)
{
	struct mdp_buffer *src_buf = &ctx->vpu.vsi->src_buffer;
	int i;

	for (i = 0; i < ARRAY_SIZE(addr->addr); i++)
		src_buf->addr_mva[i] = (uint64_t)addr->addr[i];
}
EXPORT_SYMBOL(mtk_mdp_set_input_addr);

void mtk_mdp_set_output_addr(struct mtk_mdp_ctx *ctx,
				struct mtk_mdp_addr *addr)
{
	struct mdp_buffer *dst_buf = &ctx->vpu.vsi->dst_buffer;
	int i;

	for (i = 0; i < ARRAY_SIZE(addr->addr); i++)
		dst_buf->addr_mva[i] = (uint64_t)addr->addr[i];
}
EXPORT_SYMBOL(mtk_mdp_set_output_addr);

void mtk_mdp_set_in_size(struct mtk_mdp_ctx *ctx,
				struct mtk_mdp_frame *frame)
{
	struct mdp_config *config = &ctx->vpu.vsi->src_config;

	/* Set input pixel offset */
	config->crop_x = frame->crop.left;
	config->crop_y = frame->crop.top;

	/* Set input cropped size */
	config->crop_w = frame->crop.width;
	config->crop_h = frame->crop.height;

	/* Set input original size */
	config->x = 0;
	config->y = 0;
	config->w = frame->width;
	config->h = frame->height;

	config->pitch[0] = frame->pitch[0];
	config->pitch[1] = 1;
	config->pitch[2] = 2;
}
EXPORT_SYMBOL(mtk_mdp_set_in_size);

void mtk_mdp_set_in_image_format(struct mtk_mdp_ctx *ctx,
				struct mtk_mdp_frame *frame)
{
	unsigned int i;
	struct mdp_config *config = &ctx->vpu.vsi->src_config;
	struct mdp_buffer *src_buf = &ctx->vpu.vsi->src_buffer;

	src_buf->plane_num = frame->fmt->num_planes;
	config->format = frame->fmt->pixelformat;

	for (i = 0; i < src_buf->plane_num; i++)
		src_buf->plane_size[i] = frame->payload[i];
}
EXPORT_SYMBOL(mtk_mdp_set_in_image_format);

void mtk_mdp_set_out_size(struct mtk_mdp_ctx *ctx,
				struct mtk_mdp_frame *frame)
{
	struct mdp_config *config = &ctx->vpu.vsi->dst_config;

	config->crop_x = frame->crop.left;
	config->crop_y = frame->crop.top;
	config->crop_w = frame->crop.width;
	config->crop_h = frame->crop.height;
	config->x = 0;
	config->y = 0;
	config->w = frame->width;
	config->h = frame->height;

	config->pitch[0] = frame->pitch[0];
	config->pitch[1] = 1;
	config->pitch[2] = 2;
}
EXPORT_SYMBOL(mtk_mdp_set_out_size);

void mtk_mdp_set_out_image_format(struct mtk_mdp_ctx *ctx,
				struct mtk_mdp_frame *frame)
{
	unsigned int i;
	struct mdp_config *config = &ctx->vpu.vsi->dst_config;
	struct mdp_buffer *dst_buf = &ctx->vpu.vsi->dst_buffer;

	dst_buf->plane_num = frame->fmt->num_planes;
	config->format = frame->fmt->pixelformat;
	for (i = 0; i < dst_buf->plane_num; i++)
		dst_buf->plane_size[i] = frame->payload[i];
}
EXPORT_SYMBOL(mtk_mdp_set_out_image_format);

void mtk_mdp_set_rotation(struct mtk_mdp_ctx *ctx,
				int rotate, int hflip, int vflip)
{
	struct mdp_config_misc *misc = &ctx->vpu.vsi->misc;

	misc->orientation = rotate;
	misc->hflip = hflip;
	misc->vflip = vflip;
}
EXPORT_SYMBOL(mtk_mdp_set_rotation);

void mtk_mdp_set_global_alpha(struct mtk_mdp_ctx *ctx,
				int alpha)
{
	struct mdp_config_misc *misc = &ctx->vpu.vsi->misc;

	misc->alpha = alpha;
}
EXPORT_SYMBOL(mtk_mdp_set_global_alpha);

void mtk_mdp_set_pq_info(struct mtk_mdp_ctx *ctx, struct mdp_pq_info *pq)
{
	struct mdp_pq_info *vsi_pq = &ctx->vpu.vsi->pq;

	vsi_pq->sharpness_enable = pq->sharpness_enable;
	vsi_pq->sharpness_level = pq->sharpness_level;
	vsi_pq->dynamic_contrast_enable = pq->dynamic_contrast_enable;
	vsi_pq->brightness_enable = pq->brightness_enable;
	vsi_pq->brightness_level = pq->brightness_level;
	vsi_pq->contrast_enable = pq->contrast_enable;
	vsi_pq->contrast_level = pq->contrast_level;
	vsi_pq->gamma_enable = pq->gamma_enable;
	vsi_pq->gamma_type = pq->gamma_type;
	memcpy(vsi_pq->gamma_table, pq->gamma_table,
			vsi_pq->gamma_type == 0 ? 256 : 16);
	vsi_pq->invert = pq->invert;
	vsi_pq->dth_enable = pq->dth_enable;
	vsi_pq->dth_algo = pq->dth_algo;
	vsi_pq->rsz_algo = pq->rsz_algo;
}
EXPORT_SYMBOL(mtk_mdp_set_pq_info);

int easy_mtk_mdp_dth_inv(
		u32 src_w, u32 src_h,
		u32 dst_w, u32 dst_h,
		u32 src_fmt, u32 dst_fmt,
		u32 src_pitch, u32 dst_pitch,
		u32 src_mva, u32 dst_mva,
		u32 crop_x, u32 crop_y,
		u32 crop_w, u32 crop_h,
		u8 dth_en, u8 dth_algo, u8 invert)
{
	struct mtk_mdp_addr src_addr;
	struct mtk_mdp_frame src_frame;
	struct mtk_mdp_fmt mdp_src_fmt;
	struct mtk_mdp_addr dst_addr;
	struct mtk_mdp_frame dst_frame;
	struct mtk_mdp_fmt mdp_dst_fmt;
	struct mdp_pq_info pq;
	struct mtk_mdp_ctx *ctx = mtk_mdp_create_ctx();
	int ret;

	memset(&src_addr, 0, sizeof(src_addr));
	src_addr.addr[0] = src_mva;
	src_addr.addr[1] = 0;
	src_addr.addr[2] = 0;

	memset(&dst_addr, 0, sizeof(dst_addr));
	dst_addr.addr[0] = dst_mva;
	dst_addr.addr[1] = 0;
	dst_addr.addr[2] = 0;

	memset(&src_frame, 0, sizeof(src_frame));
	src_frame.crop.left = crop_x;
	src_frame.crop.top = crop_y;
	src_frame.crop.width = crop_w;
	src_frame.crop.height = crop_h;
	src_frame.width = src_w;
	src_frame.height = src_h;
	src_frame.pitch[0] = src_pitch;
	mdp_src_fmt.pixelformat = src_fmt;
	src_frame.fmt = &mdp_src_fmt;

	memset(&dst_frame, 0, sizeof(dst_frame));
	dst_frame.crop.left = 0;
	dst_frame.crop.top = 0;
	dst_frame.crop.width = dst_w;
	dst_frame.crop.height = dst_h;
	dst_frame.width = dst_w;
	dst_frame.height = dst_h;
	dst_frame.pitch[0] = dst_pitch;
	mdp_dst_fmt.pixelformat = dst_fmt;
	src_frame.fmt = &mdp_dst_fmt;

	memset(&pq, 0, sizeof(pq));
	pq.invert = invert;
	pq.dth_enable = dth_en;
	pq.dth_algo = dth_algo;

	mtk_mdp_set_input_addr(ctx, &src_addr);
	mtk_mdp_set_in_size(ctx, &src_frame);
	mtk_mdp_set_in_image_format(ctx, &src_frame);

	mtk_mdp_set_output_addr(ctx, &dst_addr);
	mtk_mdp_set_out_size(ctx, &dst_frame);
	mtk_mdp_set_out_image_format(ctx, &dst_frame);

	mtk_mdp_set_rotation(ctx, 0, 0, 0);
	mtk_mdp_set_pq_info(ctx, &pq);

	ret = mtk_mdp_vpu_process(&ctx->vpu);

	mtk_mdp_destroy_ctx(ctx);

	return ret;
}
EXPORT_SYMBOL(easy_mtk_mdp_dth_inv);
