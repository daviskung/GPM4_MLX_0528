/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2014 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_l1_scaler.h"
#include "drv_l2_scaler.h"
#include "portable.h"
#include "drv_l2_scaler_version.h"

#if (defined _DRV_L2_SCALER) && (_DRV_L2_SCALER == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#ifndef DISABLE
#define DISABLE     0
#endif

#ifndef ENABLE
#define ENABLE      1
#endif

#ifndef DBG_PRINT
#define DBG_PRINT   printf
#endif

#ifndef DEBUG
#define DEBUG				DBG_PRINT
#else
#undef DEBUG
#define DEBUG(...)
#endif

#define RETURN(x, msg)\
{\
	ret = x;\
	DBG_PRINT(msg);\
	goto __exit;\
}

//#define gp_malloc(size)                 pvPortMalloc(size)
//#define gp_malloc_align(size, align)    gp_malloc(size)
//#define gp_free(ptr)                    vPortFree((void *)ptr);

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static INT32U ext_line_buf[SCALER_MAX];
static INT32U step_out_width;

static INT32U fd_step_callback_set(INT32U value)
{
    step_out_width = value;
    return 0;
}

static INT32U fd_step_callback_get(void)
{
    if(step_out_width)
        return (step_out_width - 1);
    else
        return 0;
}

static INT32S drv_l2_scaler_set_para(INT32U scale_dev, ScalerPara_t *argp)
{
	drv_l1_scaler_out_of_boundary_mode_set(scale_dev, argp->boundary_mode);
	drv_l1_scaler_out_of_boundary_color_set(scale_dev, argp->boundary_color);
	drv_l1_scaler_YUV_type_set(scale_dev, argp->yuv_type);
	if(argp->gamma_en) {
		int i;

		drv_l1_scaler_gamma_switch(scale_dev, TRUE);
		for(i=0; i<256; i++) {
			drv_l1_scaler_Y_gamma_config(scale_dev, i, argp->gamma_table[i]);
		}
	} else {
		drv_l1_scaler_gamma_switch(scale_dev, FALSE);
	}

	if(argp->color_matrix_en) {
		drv_l1_scaler_color_matrix_switch(scale_dev, TRUE);
		drv_l1_scaler_color_matrix_config(scale_dev, &argp->matrix);
	} else {
		drv_l1_scaler_color_matrix_switch(scale_dev, FALSE);
	}

    if(argp->scaler_intmode == 1)
        drv_l1_scaler_integration_mode_set(scale_dev, C_SCALER_NEW_INTEGRATION_MODE1);
    else if(argp->scaler_intmode == 3)
        drv_l1_scaler_integration_mode_set(scale_dev, C_SCALER_NEW_INTEGRATION_MODE3);
    else
        drv_l1_scaler_integration_mode_set(scale_dev, C_SCALER_NEW_INTEGRATION_MODE0);

	return STATUS_OK;
}

/**
 * @brief   scale init
 * @param   none
 * @return 	none
 */
void drv_l2_scaler_init(void)
{
	INT32U i;

	for(i=0; i<SCALER_MAX; i++) {
		ext_line_buf[i] = 0;
	}
#if SCALER0_EN == 1
	drv_l1_scaler_init(SCALER_0);
#endif
#if SCALER1_EN == 1
	drv_l1_scaler_init(SCALER_1);
#endif

    // default no callback
    drv_l1_scaler_isr_callback_set(0);
    drv_l1_scaler_new_int_callback_set(0);
}

/**
 * @brief   scale clip image
 * @param   scale_dev[in]: scaler device
 * @param   wait_done[in]: wait scaler finish
 * @param   src[in]: source image
 * @param   dst[in]: destination image
 * @param   clip[in]: clip range
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_scaler_clip(INT8U scale_dev, INT8U wait_done, gpImage *src, gpImage *dst, gpRect *clip)
{
	INT32S src_width, dst_width;
	INT32S in_format, out_format;
	ScalerFormat_t scale;
	ScalerPara_t para;

	switch(src->format)
	{
	case IMG_FMT_GRAY:
		src_width = src->widthStep >> 1 << 1; //align2
		in_format = C_SCALER_CTRL_IN_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_YUYV;
		break;
	default:
        return STATUS_FAIL;
	}

	switch(dst->format)
	{
	case IMG_FMT_GRAY:
		dst_width = dst->widthStep >> 1 << 1; //align2
		out_format = C_SCALER_CTRL_OUT_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_YUYV;
		break;
	default:
		return STATUS_FAIL;
	}

    memset((void *)&scale, 0x00, sizeof(ScalerFormat_t));
	scale.input_format = in_format;
	scale.input_width = src_width;
	scale.input_height = src->height;
	scale.input_visible_width = clip->x + clip->width;
	scale.input_visible_height = clip->y + clip->height;
	scale.input_x_offset = clip->x;
	scale.input_y_offset = clip->y;

	scale.input_y_addr = (INT32U)src->ptr;
	scale.input_u_addr = 0;
	scale.input_v_addr = 0;

    scale.input_b_y_addr = 0;
    scale.input_b_u_addr = 0;
    scale.input_b_v_addr = 0;

	scale.output_format = out_format;
	scale.output_width = dst_width;
	scale.output_height = dst->height;
	scale.output_buf_width = dst_width;
	scale.output_buf_height = dst->height;
	scale.output_x_offset = 0;

	scale.output_y_addr = (INT32U)dst->ptr;
	scale.output_u_addr = 0;
	scale.output_v_addr = 0;

    scale.callback = 0;
	scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
	scale.scale_mode = C_SCALER_FULL_SCREEN_BY_RATIO;
	scale.digizoom_m = 10;
	scale.digizoom_n = 10;

	memset((void *)&para, 0x00, sizeof(para));
	para.boundary_color = 0x008080;

	return drv_l2_scaler_trigger(scale_dev, wait_done, &scale, &para);
}

/**
 * @brief   FD scale clip image
 * @param   scale_dev[in]: scaler device
 * @param   wait_done[in]: wait scaler finish
 * @param   src[in]: source image
 * @param   dst[in]: destination image
 * @param   clip[in]: clip range
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_FD_scaler_clip(INT8U scale_dev, INT8U wait_done, gpImage *src, gpImage *dst, gpRect *clip)
{
	INT32S src_width, dst_width;
	INT32S in_format, out_format;
	ScalerFormat_t scale;
	ScalerPara_t para;

	switch(src->format)
	{
	case IMG_FMT_GRAY:
		src_width = src->widthStep >> 1 << 1; //align2
		in_format = C_SCALER_CTRL_IN_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_YUYV;
		break;
	default:
        return STATUS_FAIL;
	}

	switch(dst->format)
	{
	case IMG_FMT_GRAY:
		dst_width = dst->widthStep >> 1 << 1; //align2
		out_format = C_SCALER_CTRL_OUT_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_YUYV;
		break;
    case IMG_FMT_INTEGRAL_Y:
		dst_width = dst->widthStep >> 1 << 1; //align2
		out_format = C_SCALER_CTRL_OUT_Y_ONLY_INTEGRAL;
	    break;
	default:
		return STATUS_FAIL;
	}

    memset((void *)&scale, 0x00, sizeof(ScalerFormat_t));
	scale.input_format = in_format;
	scale.input_width = src_width;
	scale.input_height = src->height;
	scale.input_visible_width = clip->width;
	scale.input_visible_height = clip->height;
	scale.input_x_offset = 0;
	scale.input_y_offset = 0;

	scale.input_y_addr = (INT32U)src->ptr;
	scale.input_u_addr = 0;
	scale.input_v_addr = 0;

    scale.input_b_y_addr = 0;
    scale.input_b_u_addr = 0;
    scale.input_b_v_addr = 0;

	scale.output_format = out_format;
	scale.output_width = dst_width;
	scale.output_height = dst->height;
	scale.output_buf_width = dst_width;
	scale.output_buf_height = dst->height;
	scale.output_x_offset = 0;

	scale.fd_stepout_width = dst->width;
	fd_step_callback_set(scale.fd_stepout_width);
	scale.output_y_addr = (INT32U)dst->ptr;
	scale.output_u_addr = 0;
	scale.output_v_addr = 0;
	scale.new_int_callback = fd_step_callback_get;

    scale.callback = 0;
	scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
	scale.scale_mode = C_SCALER_RATIO_FD;
	scale.digizoom_m = 10;
	scale.digizoom_n = 10;

	memset((void *)&para, 0x00, sizeof(para));
	para.boundary_mode = 0;
    para.boundary_color = 0;

    if(out_format == C_SCALER_CTRL_OUT_Y_ONLY_INTEGRAL)
        para.scaler_intmode = 0;
    else
        para.scaler_intmode = 1;

	return drv_l2_scaler_trigger(scale_dev, wait_done, &scale, &para);
}

/**
 * @brief   scale image
 * @param   scale_dev[in]: scaler device
 * @param   wait_done[in]: wait scaler finish
 * @param   src[in]: source image
 * @param   dst[in]: destination image
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_scaler_once(INT8U scale_dev, INT8U wait_done, gpImage *src, gpImage *dst)
{
	INT32S src_width, dst_width;
	INT32S in_format, out_format;
	ScalerFormat_t scale;
	ScalerPara_t para;

	switch(src->format)
	{
	case IMG_FMT_GRAY:
		src_width = src->widthStep >> 1 << 1; //align2
		in_format = C_SCALER_CTRL_IN_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_YUYV;
		break;
	default:
        return STATUS_FAIL;
	}

	switch(dst->format)
	{
	case IMG_FMT_GRAY:
		dst_width = dst->widthStep >> 1 << 1; //align2
		out_format = C_SCALER_CTRL_OUT_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_YUYV;
		break;
	default:
		return STATUS_FAIL;
	}

    memset((void *)&scale, 0x00, sizeof(ScalerFormat_t));
	scale.input_format = in_format;
	scale.input_width = src_width;
	scale.input_height = src->height;
	scale.input_visible_width = src->width;
	scale.input_visible_height = src->height;
	scale.input_x_offset = 0;
	scale.input_y_offset = 0;

	scale.input_y_addr = (INT32U)src->ptr;
	scale.input_u_addr = 0;
	scale.input_v_addr = 0;

    scale.input_b_y_addr = 0;
    scale.input_b_u_addr = 0;
    scale.input_b_v_addr = 0;

	scale.output_format = out_format;
	scale.output_width = dst_width;
	scale.output_height = dst->height;
	scale.output_buf_width = dst_width;
	scale.output_buf_height = dst->height;
	scale.output_x_offset = 0;

	scale.output_y_addr = (INT32U)dst->ptr;
	scale.output_u_addr = 0;
	scale.output_v_addr = 0;

    scale.callback = 0;
	scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
	scale.scale_mode = C_SCALER_FULL_SCREEN_BY_RATIO;
	scale.digizoom_m = 10;
	scale.digizoom_n = 10;

	memset((void *)&para, 0x00, sizeof(para));
	para.boundary_color = 0x008080;

	return drv_l2_scaler_trigger(scale_dev, wait_done, &scale, &para);
}

/**
 * @brief   scale image
 * @param   scale_dev[in]: scaler device
 * @param   wait_done[in]: wait scaler finish
 * @param   src[in]: source image
 * @param   dst[in]: destination image
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_scaler_full_screen(INT8U scale_dev, INT8U wait_done, gpImage *src, gpImage *dst)
{
	INT32S src_width, dst_width;
	INT32S in_format, out_format;
	ScalerFormat_t scale;
	ScalerPara_t para;

	switch(src->format)
	{
	case IMG_FMT_GRAY:
		src_width = src->widthStep >> 1 << 1; //align2
		in_format = C_SCALER_CTRL_IN_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_YUYV;
		break;
	case IMG_FMT_YUV444:
		src_width = src->widthStep >> 2 << 1; //align2
		in_format = C_SCALER_CTRL_IN_YUV444;
		break;
	default:
        return STATUS_FAIL;
	}

	switch(dst->format)
	{
	case IMG_FMT_GRAY:
		dst_width = dst->widthStep >> 1 << 1; //align2
		out_format = C_SCALER_CTRL_OUT_Y_ONLY;
		break;
	case IMG_FMT_YUYV:
	case IMG_FMT_YCbYCr:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_UYVY;
		break;
	case IMG_FMT_UYVY:
	case IMG_FMT_CbYCrY:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_YUYV;
		break;
	case IMG_FMT_YUV444:
		dst_width = dst->widthStep >> 4 << 3; //align8
		out_format = C_SCALER_CTRL_OUT_YUV444;
		break;
	default:
		return STATUS_FAIL;
	}

    memset((void *)&scale, 0x00, sizeof(ScalerFormat_t));
	scale.input_format = in_format;
	scale.input_width = src_width;
	scale.input_height = src->height;
	scale.input_visible_width = src->width;
	scale.input_visible_height = src->height;
	scale.input_x_offset = 0;
	scale.input_y_offset = 0;

    scale.input_y_addr = (INT32U)src->ptr;
    if(in_format == C_SCALER_CTRL_IN_YUV444)
    {
        scale.input_u_addr = (INT32U)src->ptr_u;
        scale.input_v_addr = (INT32U)src->ptr_v;
    }
    else
    {
        scale.input_u_addr = 0;
        scale.input_v_addr = 0;
    }

    scale.input_b_y_addr = 0;
    scale.input_b_u_addr = 0;
    scale.input_b_v_addr = 0;

	scale.output_format = out_format;
	scale.output_width = dst_width;
	scale.output_height = dst->height;
	scale.output_buf_width = dst_width;
	scale.output_buf_height = dst->height;
	scale.output_x_offset = 0;

    scale.output_y_addr = (INT32U)dst->ptr;
    if(out_format == C_SCALER_CTRL_OUT_YUV444)
    {
        scale.output_u_addr = (INT32U)dst->ptr_u;
        scale.output_v_addr = (INT32U)dst->ptr_v;
    }
    else
    {
        scale.output_u_addr = 0;
        scale.output_v_addr = 0;
    }

    scale.callback = 0;
	scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
	scale.scale_mode = C_SCALER_FULL_SCREEN;
	scale.digizoom_m = 10;
	scale.digizoom_n = 10;

	memset((void *)&para, 0x00, sizeof(para));
	para.boundary_color = 0;

	return drv_l2_scaler_trigger(scale_dev, wait_done, &scale, &para);
}

/**
 * @brief   scale wait done
 * @param   scale_dev[in]: scaler device
 * @param   wait_done[in]: wait scaler finish
 * @param   pScale[in]: scale parameter
 * @return 	scaler status
 */
INT32S drv_l2_scaler_wait_done(INT8U scale_dev, ScalerFormat_t *pScale)
{
	// scale a
	if(pScale->scale_status == C_SCALER_STATUS_STOP) {
		return C_SCALER_STATUS_STOP;
	}

	// scale done
	if(pScale->scale_status == C_SCALER_STATUS_DONE) {
		pScale->scale_status = C_SCALER_STATUS_STOP;
		drv_l2_scaler_stop(scale_dev);
		return C_SCALER_STATUS_STOP;
	}

	pScale->scale_status = drv_l1_scaler_wait_idle(scale_dev);
	if(pScale->scale_status == C_SCALER_STATUS_TIMEOUT) {
		pScale->scale_status = C_SCALER_STATUS_STOP;
		drv_l2_scaler_stop(scale_dev);
		return C_SCALER_STATUS_TIMEOUT;
	}

	return pScale->scale_status;
}

/**
 * @brief   scale stop
 * @param   scale_dev[in]: scaler device
 * @return 	scaler status
 */
void drv_l2_scaler_stop(INT8U scale_dev)
{
	drv_l1_scaler_stop(scale_dev);

	// free line buffer
	if(ext_line_buf[scale_dev]) {
		gp_free((void *)ext_line_buf[scale_dev]);
		ext_line_buf[scale_dev] = 0;
	}

    // default no callback
	drv_l1_scaler_isr_callback_set(0);
    drv_l1_scaler_new_int_callback_set(0);

	// hardware unlock
	drv_l1_scaler_unlock(scale_dev);
}

/**
 * @brief   scale image trigger
 * @param   scale_dev[in]: scaler device
 * @param   wait_done[in]: wait scaler finish
 * @param   pScale_dev[in]: scale parameter
 * @param   boundary_color[in]: boundary color
 * @return 	scaler status
 */
INT32S drv_l2_scaler_trigger(INT8U scale_dev, INT8U wait_done, ScalerFormat_t *pScale_dev, ScalerPara_t *pPara)
{
	INT8U mode = 0;
	INT32S ret;
	INT32U tempx = 0;
	INT32U tempy = 0;
    ScalerFormat_t *pScale = (ScalerFormat_t *)pScale_dev;

	DEBUG("%s\r\n", __func__);
#if GPM4_AUTHENTICATION == 1
    {
        INT32U patch_id;

        patch_id = *((volatile INT32U *) 0xC015000C);
        if ((patch_id & 0x00000001) != 0x00000001)
                RETURN(C_SCALER_START_ERR, "SCALER_START_ERR\r\n");
	}
#endif

#if SCALER0_EN == 0
    if(scale_dev == 0)
        return -1;
#endif

#if SCALER1_EN == 0
    if(scale_dev)
        return -1;
#endif
	// times
	pScale->reserved0 = 1;

	// scaler lock
	drv_l1_scaler_lock(scale_dev);

	if(pScale->input_visible_width > pScale->input_width) {
		pScale->input_visible_width = 0;
	}

	if(pScale->input_visible_height > pScale->input_height) {
		pScale->input_visible_height = 0;
	}

	if(pScale->output_width > pScale->output_buf_width) {
		pScale->output_width = pScale->output_buf_width;
	}

	if(pScale->output_height > pScale->output_buf_height) {
		pScale->output_height = pScale->output_buf_height;
	}

	// scale mode setting
	drv_l1_scaler_init(scale_dev);

	drv_l1_scaler_isr_callback_set(pScale->callback);
    drv_l1_scaler_new_int_callback_set(pScale->new_int_callback);

    if((pPara->scaler_intmode == 1) || (pPara->scaler_intmode == 3))
    {
        if(pScale->scale_mode != C_SCALER_RATIO_FD)
            RETURN(C_SCALER_INTEGRATION_ERR, "SCALER_INTEGRATION_ERR\r\n");
    }

    if(pScale->output_format == C_SCALER_CTRL_OUT_Y_ONLY_INTEGRAL)
    {
        if((pScale->input_width != pScale->output_width) || (pScale->input_height != pScale->output_height))
            RETURN(C_SCALER_INTEGRATION_ERR, "SCALER_INTEGRATION_ERR\r\n");
    }

	switch(pScale->scale_mode)
	{
	case C_SCALER_FULL_SCREEN:
		ret = drv_l1_scaler_image_pixels_set(scale_dev, pScale->input_width, pScale->input_height, pScale->output_buf_width, pScale->output_buf_height);
		if(ret < 0) {
			RETURN(C_SCALER_START_ERR, "SCALER_START_ERR\r\n");
		}

        if(pScale->force_intp_en)
        {
            tempx = 0x8000;
            tempy = 0x8000;
            ret = drv_l1_scaler_input_offset_set(scale_dev, tempx, tempy);
        }
		break;

	case C_SCALER_BY_RATIO:
		mode = 1;
	case C_SCALER_FULL_SCREEN_BY_RATIO:
		ret = drv_l1_scaler_input_pixels_set(scale_dev, pScale->input_width, pScale->input_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_SIZE_ERR\r\n");
		}

		ret = drv_l1_scaler_input_visible_pixels_set(scale_dev, pScale->input_visible_width, pScale->input_visible_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_VISIBLE_SIZE_ERR\r\n");
		}

		if(pScale->input_visible_width) {
			tempx = pScale->input_visible_width;
		} else {
			tempx = pScale->input_width;
		}

		if(pScale->input_visible_height) {
			tempy = pScale->input_visible_height;
		} else {
			tempy = pScale->input_height;
		}

		if(pScale->input_x_offset > tempx) {
			pScale->input_x_offset = 0;
		}

		if(pScale->input_y_offset > tempy) {
			pScale->input_y_offset = 0;
		}

		if(pScale->input_x_offset) {
			tempx -= pScale->input_x_offset;
		}

		if(pScale->input_y_offset) {
			tempy -= pScale->input_y_offset;
		}

		if(mode) {
			/* scale by ratio */
			tempx = (tempx << 16) / pScale->output_width;
			tempy = (tempy << 16) / pScale->output_height;
		} else {
			/* scale full screen by ratio */
			if (pScale->output_buf_height*tempx > pScale->output_buf_width*tempy) {
		      	tempx = tempy = (tempx << 16) / pScale->output_buf_width;
		      	pScale->output_height = (pScale->output_buf_height << 16) / tempx;
		    } else {
		      	tempx = tempy = (tempy << 16) / pScale->output_buf_height;
		      	pScale->output_width = (pScale->output_buf_width << 16) / tempy;
			}
		}

		ret = drv_l1_scaler_output_pixels_set(scale_dev, tempx, tempy, pScale->output_buf_width, pScale->output_buf_height);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_SIZE_ERR, "SCALER_OUTPUT_SIZE_ERR\r\n");
		}

		tempx = pScale->input_x_offset << 16;
		tempy = pScale->input_y_offset << 16;
        if(pScale->force_intp_en)
        {
            tempx += 0x8000;
            tempy += 0x8000;
        }
		ret = drv_l1_scaler_input_offset_set(scale_dev, tempx, tempy);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_OFFSET_ERR, "SCALER_INPUT_OFFSET_ERR\r\n");
		}
		break;

	case C_SCALER_FULL_SCREEN_BY_DIGI_ZOOM:
		ret = drv_l1_scaler_input_pixels_set(scale_dev, pScale->input_width, pScale->input_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_SIZE_ERR\r\n");
		}

		ret = drv_l1_scaler_input_visible_pixels_set(scale_dev, pScale->input_visible_width, pScale->input_visible_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_VISIBLE_SIZE_ERR\r\n");
		}

		/* mutiple * 100 */
		if(pScale->digizoom_n == 0) {
			pScale->digizoom_n = 10;
		}

		if(pScale->digizoom_m == 0) {
			pScale->digizoom_m = 10;
		}

		tempx = 100 * (pScale->output_width * pScale->digizoom_m) / (pScale->input_width * pScale->digizoom_n);
		tempx = 65536 * 100 / tempx;
		ret = drv_l1_scaler_output_pixels_set(scale_dev, tempx, tempx, pScale->output_buf_width, pScale->output_buf_height);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_SIZE_ERR, "SCALER_OUTPUT_SIZE_ERR\n");
		}

		tempx = pScale->input_width * (ABS(pScale->digizoom_m - pScale->digizoom_n));
		tempx >>= 1;
		tempx =	(tempx << 16) / pScale->digizoom_m;
		tempy = pScale->input_height * (ABS(pScale->digizoom_m - pScale->digizoom_n));
		tempy >>= 1;
		tempy = (tempy << 16) / pScale->digizoom_m;
		ret = drv_l1_scaler_input_offset_set(scale_dev, tempx, tempy);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_OFFSET_ERR, "SCALER_INPUT_OFFSET_ERR\r\n");
		}
		break;

	case C_SCALER_RATIO_USER:
		ret = drv_l1_scaler_input_pixels_set(scale_dev, pScale->input_width, pScale->input_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_SIZE_ERR\r\n");
		}

		ret = drv_l1_scaler_input_visible_pixels_set(scale_dev, pScale->input_width, pScale->input_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_VISIBLE_SIZE_ERR\r\n");
		}

		if(pScale->input_visible_width) {
			tempx = pScale->input_visible_width;
		} else {
			tempx = pScale->input_width;
		}

		if(pScale->input_visible_height) {
			tempy = pScale->input_visible_height;
		} else {
			tempy = pScale->input_height;
		}

		/* scale by ratio */
		tempx = (tempx << 16) / pScale->output_width;
		tempy = (tempy << 16) / pScale->output_height;

		ret = drv_l1_scaler_output_pixels_set(scale_dev, tempx, tempy, pScale->output_buf_width, pScale->output_buf_height);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_SIZE_ERR, "SCALER_OUTPUT_SIZE_ERR\r\n");
		}

		tempx = pScale->input_x_offset;
		tempy = pScale->input_y_offset;
        if(pScale->force_intp_en)
        {
            tempx += 0x8000;
            tempy += 0x8000;
        }
		ret = drv_l1_scaler_input_offset_set(scale_dev, tempx, tempy);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_OFFSET_ERR, "SCALER_OUTPUT_OFFSET_ERR\r\n");
		}
		break;

	case C_SCALER_RATIO_FD:
		if((pScale->output_buf_width > pScale->input_width) || (pScale->output_buf_height > pScale->input_height))
            RETURN(C_SCALER_INTEGRATION_ERR, "SCALER_INTEGRATION_ERR\r\n");

		ret = drv_l1_scaler_input_pixels_set(scale_dev, pScale->input_width, pScale->input_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_SIZE_ERR\r\n");
		}

        if((pScale->input_visible_width == 0) || (pScale->input_visible_height == 0) )
            ret = drv_l1_scaler_input_visible_pixels_set(scale_dev, pScale->input_width, pScale->input_height);
        else
            ret = drv_l1_scaler_input_visible_pixels_set(scale_dev, pScale->input_visible_width, pScale->input_visible_height);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_SIZE_ERR, "SCALER_INPUT_VISIBLE_SIZE_ERR\r\n");
		}

		if(pScale->input_visible_width) {
			tempx = pScale->input_visible_width;
		} else {
			tempx = pScale->input_width;
		}

		if(pScale->input_visible_height) {
			tempy = pScale->input_visible_height;
		} else {
			tempy = pScale->input_height;
		}

        /* scale by ratio */
        // for Integration mode
        tempx = (tempx << 16) / pScale->output_width;
        tempy = (tempy << 16) / pScale->output_height;
        ret = drv_l1_scaler_output_pixels_set(scale_dev, tempx, tempy, pScale->output_buf_width, pScale->output_buf_height);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_SIZE_ERR, "SCALER_OUTPUT_SIZE_ERR\r\n");
		}

        tempx = 0x8000;
        tempy = 0x8000;
		ret = drv_l1_scaler_input_offset_set(scale_dev, tempx, tempy);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_OFFSET_ERR, "SCALER_OUTPUT_OFFSET_ERR\r\n");
		}
        break;

	default:
		RETURN(C_SCALER_START_ERR, "scaler mode fail!\r\n");
	}

	// alloc ext buffer for line buffer use
	if(pScale->output_buf_width >= 1024) {
		if(ext_line_buf[scale_dev]) {
			gp_free((void *)ext_line_buf[scale_dev]);
			ext_line_buf[scale_dev] = 0;
		}

		ext_line_buf[scale_dev] = (INT32U)gp_malloc_align(pScale->output_buf_width*4*2, 4);
		if(ext_line_buf[scale_dev] == 0) {
			RETURN(C_SCALER_EXT_BUF_ERR, "SCALER_EXT_BUF_ERR\r\n");
		}

		drv_l1_scaler_external_line_buffer_set(scale_dev, ext_line_buf[scale_dev]);
		ret = drv_l1_scaler_line_buffer_mode_set(scale_dev, C_SCALER_EXTERNAL_LINE_BUFFER);
		if(ret < 0) {
			RETURN(C_SCALER_EXT_BUF_ERR, "SCALER_EXT_BUF_ERR\r\n");
		}
	}

	ret = drv_l1_scaler_output_offset_set(scale_dev, pScale->output_x_offset);
	if(ret < 0) {
		RETURN(C_SCALER_OUTPUT_BUF_ERR, "SCALER_OUTPUT_BUF_ERR\r\n");
	}

	ret = drv_l1_scaler_input_A_addr_set(scale_dev, pScale->input_y_addr, pScale->input_u_addr, pScale->input_v_addr);
	if(ret < 0) {
		RETURN(C_SCALER_INPUT_BUF_ERR, "SCALER_INPUT_BUF_ERR\r\n");
	}

	ret = drv_l1_scaler_input_B_addr_set(scale_dev, pScale->input_b_y_addr, pScale->input_b_u_addr, pScale->input_b_v_addr);
	if(ret < 0) {
		RETURN(C_SCALER_INPUT_BUF_ERR, "SCALER_INPUT_B_BUF_ERR\r\n");
	}

	ret = drv_l1_scaler_output_addr_set(scale_dev, pScale->output_y_addr, pScale->output_u_addr, pScale->output_v_addr);
	if(ret < 0) {
		RETURN(C_SCALER_OUTPUT_BUF_ERR, "SCALER_OUTPUT_BUF_ERR\r\n");
	}

	ret = drv_l1_scaler_input_format_set(scale_dev, pScale->input_format);
	if(ret < 0) {
		RETURN(C_SCALER_INPUT_FMT_ERR, "SCALER_INPUT_FMT_ERR\r\n");
	}

	ret = drv_l1_scaler_output_format_set(scale_dev, pScale->output_format);
	if(ret < 0) {
		RETURN(C_SCALER_OUTPUT_FMT_ERR, "SCALER_OUTPUT_FMT_ERR\r\n");
	}

	switch(pScale->fifo_mode)
	{
	case C_SCALER_CTRL_FIFO_DISABLE:
		ret = drv_l1_scaler_input_fifo_line_set(scale_dev, C_SCALER_CTRL_IN_FIFO_DISABLE);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_FIFO_ERR, "SCALER_INPUT_FIFO_ERR\r\n");
		}

		ret = drv_l1_scaler_output_fifo_line_set(scale_dev, C_SCALER_CTRL_OUT_FIFO_DISABLE);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_FIFO_ERR, "SCALER_OUTPUT_FIFO_ERR\r\n");
		}
		break;

	case C_SCALER_CTRL_IN_FIFO_16LINE:
	case C_SCALER_CTRL_IN_FIFO_32LINE:
	case C_SCALER_CTRL_IN_FIFO_64LINE:
	case C_SCALER_CTRL_IN_FIFO_128LINE:
	case C_SCALER_CTRL_IN_FIFO_256LINE:
		ret = drv_l1_scaler_input_fifo_line_set(scale_dev, pScale->fifo_mode);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_FIFO_ERR, "SCALER_INPUT_FIFO_ERR\r\n");
		}

		ret = drv_l1_scaler_output_fifo_line_set(scale_dev, C_SCALER_CTRL_OUT_FIFO_DISABLE);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_FIFO_ERR, "SCALER_OUTPUT_FIFO_ERR\r\n");
		}
		break;

	case C_SCALER_CTRL_OUT_FIFO_16LINE:
	case C_SCALER_CTRL_OUT_FIFO_32LINE:
	case C_SCALER_CTRL_OUT_FIFO_64LINE:
		ret = drv_l1_scaler_input_fifo_line_set(scale_dev, C_SCALER_CTRL_IN_FIFO_DISABLE);
		if(ret < 0) {
			RETURN(C_SCALER_INPUT_FIFO_ERR, "SCALER_INPUT_FIFO_ERR\r\n");
		}

		ret = drv_l1_scaler_output_fifo_line_set(scale_dev, pScale->fifo_mode);
		if(ret < 0) {
			RETURN(C_SCALER_OUTPUT_FIFO_ERR, "SCALER_OUTPUT_FIFO_ERR\r\n");
		}
		break;
	}

	/* set scale other parameters */
	if(pPara) {
		drv_l2_scaler_set_para(scale_dev, pPara);
	}

	// scaler start
	ret = drv_l1_scaler_status_polling(scale_dev);
	if(ret != C_SCALER_STATUS_STOP) {
		pScale->scale_status = C_SCALER_STATUS_INIT_ERR;
		RETURN(C_SCALER_START_ERR, "SCALE2_STATUS != STOP\r\n");
	}

	DEBUG("ScaleStart\r\n");
	ret = drv_l1_scaler_start(scale_dev, ENABLE);
	if(ret < 0) {
		pScale->scale_status = C_SCALER_STATUS_INIT_ERR;
		RETURN(C_SCALER_START_ERR, "C_SCALER_START_ERR\r\n");
	} else {
		pScale->scale_status = C_SCALER_STATUS_BUSY;
	}

	// wait done
	if(wait_done) {
		DEBUG("ScaleWaitDone\r\n");
		ret = drv_l1_scaler_wait_idle(scale_dev);
		if(ret == C_SCALER_STATUS_DONE) {
			pScale->scale_status = C_SCALER_STATUS_STOP;
			ret = C_SCALER_STATUS_STOP;
			drv_l2_scaler_stop(scale_dev);
		} else if(ret == C_SCALER_STATUS_TIMEOUT) {
			pScale->scale_status = C_SCALER_STATUS_TIMEOUT;
			RETURN(C_SCALER_START_ERR, "C_SCALER_START_ERR\r\n");
        } else if(ret == C_SCALER_STATUS_INPUT_EMPTY) {
			pScale->scale_status = C_SCALER_STATUS_INPUT_EMPTY;
			ret = C_SCALER_STATUS_INPUT_EMPTY;
		} else {
		    DEBUG("Scale_Error_State:0x%x\r\n", ret);
            RETURN(C_SCALER_START_ERR, "C_SCALER_START_ERR\r\n");
        }
	}

	pScale->scale_status = ret;

__exit:
	if(ret < 0) {
		DBG_PRINT("ScalerFail[%d]!\r\n", scale_dev);
		DBG_PRINT("InWH[%d, %d]\r\n", pScale->input_width, pScale->input_height);
		DBG_PRINT("InVisWH[%d, %d]\r\n", pScale->input_visible_width, pScale->input_visible_height);
		DBG_PRINT("InXYOffset[%x, %x]\r\n", pScale->input_x_offset, pScale->input_y_offset);
		DBG_PRINT("outWH[%d, %d]\r\n", pScale->output_width, pScale->output_height);
		DBG_PRINT("outBufWH[%d, %d]\r\n", pScale->output_buf_width, pScale->output_buf_height);
		DBG_PRINT("format[0x%x, 0x%x]\r\n", pScale->input_format, pScale->output_format);
		DBG_PRINT("outXOffset = 0x%x\r\n", pScale->output_x_offset);
		DBG_PRINT("InYAddr = 0x%x\r\n", pScale->input_y_addr);
		DBG_PRINT("InUAddr = 0x%x\r\n", pScale->input_u_addr);
		DBG_PRINT("InVAddr = 0x%x\r\n", pScale->input_v_addr);
		DBG_PRINT("OutYAddr = 0x%x\r\n", pScale->output_y_addr);
		DBG_PRINT("OutUAddr = 0x%x\r\n", pScale->output_u_addr);
		DBG_PRINT("OutVAddr = 0x%x\r\n", pScale->output_v_addr);
		DBG_PRINT("ScaleMode = 0x%x\r\n", pScale->scale_mode);
		drv_l2_scaler_stop(scale_dev);
	}

	return ret;
}

/**
 * @brief   scale image retrigger for fifo mode use
 * @param   scale_dev[in]: scaler device
 * @param   pScale[in]: scale parameter
 * @return 	scaler status
 */
INT32S drv_l2_scaler_retrigger(INT8U scale_dev, ScalerFormat_t *pScale)
{
	INT32S ret;

	DEBUG("%s\r\n", __func__);
	if(pScale->fifo_mode == C_SCALER_CTRL_FIFO_DISABLE) {
		return C_SCALER_STATUS_STOP;
	}

	if(pScale->scale_status == C_SCALER_STATUS_STOP) {
		return C_SCALER_STATUS_STOP;
	}

	if(pScale->scale_status == C_SCALER_STATUS_DONE) {
		pScale->scale_status = C_SCALER_STATUS_STOP;
		ret = C_SCALER_STATUS_STOP;
		drv_l2_scaler_stop(scale_dev);
		goto __exit;
	}

	/* times */
	pScale->reserved0++;

	/* scale restart */
	ret = drv_l1_scaler_restart(scale_dev);
	if(ret < 0) {
		pScale->scale_status = C_SCALER_STATUS_STOP;
		drv_l2_scaler_stop(scale_dev);
		RETURN(C_SCALER_START_ERR, "ScalerRestartFail\r\n");
	} else if(ret == 1) {
		/* already finish */
		DEBUG("ScalerAlreadyDone\r\n");
		pScale->scale_status = C_SCALER_STATUS_STOP;
		ret = C_SCALER_STATUS_STOP;
		drv_l2_scaler_stop(scale_dev);
		goto __exit;
	}

	/* waiting for done */
	DEBUG("ScalerWaitFifo\r\n");
	ret = drv_l1_scaler_wait_idle(scale_dev);
	if(ret == C_SCALER_STATUS_DONE) {
		pScale->scale_status = C_SCALER_STATUS_STOP;
		ret = C_SCALER_STATUS_STOP;
		drv_l2_scaler_stop(scale_dev);
	} else if(ret == C_SCALER_STATUS_TIMEOUT) {
		RETURN(C_SCALER_STATUS_TIMEOUT, "SCALER_STATUS_TIMEOUT\r\n");
	} else {
		pScale->scale_status = ret;
	}

__exit:
	if(ret < 0) {
		DBG_PRINT("ScalerFail[%d]!\r\n", scale_dev);
		DBG_PRINT("InWH[%d, %d]\r\n", pScale->input_width, pScale->input_height);
		DBG_PRINT("InVisWH[%d, %d]\r\n", pScale->input_visible_width, pScale->input_visible_height);
		DBG_PRINT("InXYOffset[%x, %x]\r\n", pScale->input_x_offset, pScale->input_y_offset);
		DBG_PRINT("outWH[%d, %d]\r\n", pScale->output_width, pScale->output_height);
		DBG_PRINT("outBufWH[%d, %d]\r\n", pScale->output_buf_width, pScale->output_buf_height);
		DBG_PRINT("format[0x%x, 0x%x]\r\n", pScale->input_format, pScale->output_format);
		DBG_PRINT("outXOffset = 0x%x\r\n", pScale->output_x_offset);
		DBG_PRINT("InYAddr = 0x%x\r\n", pScale->input_y_addr);
		DBG_PRINT("InUAddr = 0x%x\r\n", pScale->input_u_addr);
		DBG_PRINT("InVAddr = 0x%x\r\n", pScale->input_v_addr);
		DBG_PRINT("OutYAddr = 0x%x\r\n", pScale->output_y_addr);
		DBG_PRINT("OutUAddr = 0x%x\r\n", pScale->output_u_addr);
		DBG_PRINT("OutVAddr = 0x%x\r\n", pScale->output_v_addr);
		DBG_PRINT("ScaleMode = 0x%x\r\n", pScale->scale_mode);
		drv_l2_scaler_stop(scale_dev);
	}

	return ret;
}
void drv_l2_scaler_get_version(void)
{
    DBG_PRINT("L2_SCALER_SVN_VERSION: %s\r\n", L2_SCALER_SVN_VERSION);
    DBG_PRINT("L2_SCALER_LIB_BUILD_TIME: %s\r\n", L2_SCALER_LIB_BUILD_TIME);
}

#endif //(defined _DRV_L2_DISP) && (_DRV_L2_DISP == 1)
