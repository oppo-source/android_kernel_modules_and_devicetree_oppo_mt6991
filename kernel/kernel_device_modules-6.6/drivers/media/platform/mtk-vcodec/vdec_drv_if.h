/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 *                 Tiffany Lin <tiffany.lin@mediatek.com>
 */

#ifndef _VDEC_DRV_IF_H_
#define _VDEC_DRV_IF_H_

#include "mtk_vcodec_drv.h"
#include "mtk_vcodec_dec.h"
#include "mtk_vcodec_util.h"
#include "vdec_vcu_if.h"
#include "vdec_ipi_msg.h"

/**
 * struct vdec_inst - decoder instance
 * @num_nalu : how many nalus be decoded
 * @ctx      : point to mtk_vcodec_ctx
 * @vcu      : VCU instance
 * @vsi      : VCU shared information
 */
struct vdec_inst {
	unsigned int num_nalu;
	struct mtk_vcodec_ctx *ctx;
	struct vdec_vcu_inst vcu;
	struct vdec_vsi *vsi;

	bool put_frame_async;
	struct ring_fb_list list_disp_fb;
	struct mutex list_disp_fb_lock;
	struct ring_fb_list list_free_fb;
	struct mutex list_free_fb_lock;
};

/**
 * struct vdec_fb_node  - decoder frame buffer node
 * @list        : list to hold this node
 * @fb  : point to frame buffer (vdec_fb), fb could point to frame buffer and
 *      working buffer this is for maintain buffers in different state
 */
struct vdec_fb_node {
	struct list_head list;
	struct vdec_fb *fb;
};

extern struct mtk_video_fmt
	mtk_vdec_formats[MTK_MAX_DEC_CODECS_SUPPORT];
extern struct mtk_codec_framesizes
	mtk_vdec_framesizes[MTK_MAX_DEC_CODECS_SUPPORT];
extern struct v4l2_vdec_max_buf_info mtk_vdec_max_buf_info;
extern struct mtk_video_frame_frameintervals mtk_vdec_frameintervals;

/**
 * vdec_if_init() - initialize decode driver
 * @ctx : [in] v4l2 context
 * @fourcc      : [in] video format fourcc, V4L2_PIX_FMT_H264/VP8/VP9..
 */
int vdec_if_init(struct mtk_vcodec_ctx *ctx, unsigned int fourcc);

/**
 * vdec_if_deinit() - deinitialize decode driver
 * @ctx : [in] v4l2 context
 *
 */
void vdec_if_deinit(struct mtk_vcodec_ctx *ctx);

/**
 * vdec_if_decode() - trigger decode
 * @ctx : [in] v4l2 context
 * @bs  : [in] input bitstream
 * @fb  : [in] frame buffer to store decoded frame, when null menas parse
 *      header only
 * @res_chg     : [out] resolution change happens if current bs have different
 *      picture width/height
 * Note: To flush the decoder when reaching EOF, set input bitstream as NULL.
 */
int vdec_if_decode(struct mtk_vcodec_ctx *ctx, struct mtk_vcodec_mem *bs,
				   struct vdec_fb *fb, unsigned int *src_chg);

/**
 * vdec_if_get_param() - get driver's parameter
 * @ctx : [in] v4l2 context
 * @type        : [in] input parameter type
 * @out : [out] buffer to store query result
 */
int vdec_if_get_param(struct mtk_vcodec_ctx *ctx, enum vdec_get_param_type type,
					  void *out);

/*
 * vdec_if_set_param - Set parameter to driver
 * @ctx : [in] v4l2 context
 * @type        : [in] input parameter type
 * @out : [in] input parameter
 * Return: 0 if setting param successfully, otherwise it is failed.
 */
int vdec_if_set_param(struct mtk_vcodec_ctx *ctx,
					  enum vdec_set_param_type type,
					  void *in);
/*
 * vdec_if_flush - Set parameter to driver
 * @ctx  : [in] v4l2 context
 * @bs  : [in] input bitstream
 * @fb  : [in] frame buffer to store decoded frame, when null menas parse
 *      header only
 * @type : [in] flush from bitstream or frame buffer
 * Return: 0 if flush successfully, otherwise it is failed.
 */
int vdec_if_flush(struct mtk_vcodec_ctx *ctx, struct mtk_vcodec_mem *bs,
				   struct vdec_fb *fb, enum vdec_flush_type type);

int vdec_if_dev_ctx_init(struct mtk_vcodec_dev *dev);
void vdec_if_dev_ctx_deinit(struct mtk_vcodec_dev *dev);

void vdec_decode_prepare(void *ctx_prepare,
	unsigned int hw_id);
void vdec_decode_unprepare(void *ctx_unprepare,
	unsigned int hw_id);
void vdec_check_release_lock(void *ctx_check);

#endif
