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
#include <string.h>
#include "application.h"
#include "drv_l1_cache.h"
#include "drv_l1_csi.h"
#include "drv_l2_sensor.h"
#include "project.h"
#include "drv_l1_clock.h"
#include "drv_l2_csi.h"
#include "gplib_mm_gplus.h"

#if (defined _DRV_L2_CSI) && (_DRV_L2_CSI == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
static INT32U INT_FLAG;

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#define __FUNCTION__		__func__

#ifndef DEBUG
#include "gplib_print_string.h"
#define DEBUG				DBG_PRINT
#else
#undef DEBUG
#define DEBUG(...)
#endif

#define MEMCPY				gp_memcpy
#define CLEAR(ptr, cblen)	gp_memset((INT8S *)ptr, 0x00, cblen)
#define RETURN(x)			ret = x; goto __exit;
#define ABS(x)				((x) >= 0 ? (x) : -(x))
#define gp_malloc(size)                 pvPortMalloc(size)
#define gp_malloc_align(size, align)    gp_malloc(size)
#define gp_free(ptr)                    vPortFree((void *)ptr);

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/


/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/


/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/


/**************************************************************************
 *               F U N C T I O N                                          *
 **************************************************************************/
/**
 * @brief	csi set input parameter
 * @param	pFmt[in]: input parameter
 * @return	0: Success, -1: Fail
 */
INT32S drv_l2_csi_set_fmt(gpCsiFmt_t *pFmt)
{
    INT32S ret = STATUS_OK;

    if(pFmt == 0)
        return -1;

    drvl1_csi_fifo_mode_set(pFmt->fifo_size);
    drvl1_csi_jpeg_mode_set(pFmt->jpeg_mode);
    drvl1_csi_preview_mode_set(pFmt->preview_mode);
    drvl1_csi_clock_priority_set(pFmt->clk_pri);
    drvl1_csi_clock_stop_set(pFmt->clk_stopb);
    drvl1_csi_cubic_output_set(pFmt->cubicen, pFmt->cubicen_size);

	return ret;
}

/**
 * @brief	cdsp stream on
 * @param	bufA[in]: buffer a
 * @param	bufB[in]: buffer b
 * @return	0: Success, -1: Fail
 */
INT32S drv_l2_csi_stream_on(gpCSIPara_t *csi_info, INT32U bufA, INT32U bufB)
{
	INT32S ret = STATUS_OK;
	INT32U int_flag = 0;
    drv_l2_sensor_para_t *SensorPara;

    if(bufA == 0 || csi_info == 0)
        return -1;

    INT_FLAG = 0;
    drv_l1_csi_set_buf(bufA);
	drvl1_csi_input_set(csi_info->csi_fmt.interface_mode, csi_info->csi_fmt.interlace_mode, csi_info->csi_tim.clki_inv, csi_info->csi_fmt.input_format);
    drvl1_csi_field_invert_set(csi_info->csi_fmt.fieldinv);
    drvl1_csi_input_latch_timing1_set(csi_info->csi_tim.hl_start, csi_info->csi_tim.vl0_start, csi_info->csi_tim.vl1_start, csi_info->csi_tim.d_type);
    drvl1_csi_input_latch_timing2_set(csi_info->csi_tim.fget_mode, csi_info->csi_tim.vrst_mode, csi_info->csi_tim.vadd_mode, csi_info->csi_tim.hrst_mode);
    ret = drvl1_csi_clko_invert_set(csi_info->csi_tim.clko_inv);

    drvl1_csi_blue_screen_set(csi_info->csi_fmt.bs_mode, csi_info->bs_upper, csi_info->bs_lower);
    drvl1_csi_black_screen_set(csi_info->h_start, csi_info->h_end, csi_info->v_start, csi_info->v_end);
    drvl1_csi_cut_screen_set(csi_info->csi_fmt.cuten, csi_info->cut_start, csi_info->cut_size);
    drvl1_csi_input_resolution_set(csi_info->target_w, csi_info->target_h);
    if(csi_info->target_w & 0x1F)
        drvl1_csi_long_burst_set(1);
    else
        drvl1_csi_long_burst_set(0);

    drvl1_csi_scale_down_set(csi_info->hratio, csi_info->vratio);
    drvl1_csi_output_data_format_set(csi_info->csi_fmt.output_format);

    // Open CDSP data path
    SensorPara = drv_l2_sensor_get_para();
    if(SensorPara->pscaler_src_mode == MODE_PSCALER_SRC_CSI)
        drvl1_csi_input_pscaler_set(1);

    if(SensorPara->md_mode)
    {
        if(SensorPara->md_block_size == MD_8_SIZE)
            drv_l1_csi_md_set_parameter(1, csi_info->target_w, csi_info->target_h,0);
        else
            drv_l1_csi_md_set_parameter(1, csi_info->target_w, csi_info->target_h,1);

        drv_l1_csi_md_set_buf(SensorPara->md_buf);
        drv_l1_csi_md_set_threshold(SensorPara->md_threshold);
        drv_l1_register_csi_cbk((CSI_CALLBACK_FUN)SensorPara->md_csi_callback);
        INT_FLAG |= MASK_CSI_MOTION_DET_FLAG;
    }

    drv_l1_csi_set_irq(1);
    drv_l1_csi_state_clear(0x3F);
    INT_FLAG |= MASK_CSI_FRAME_END_FLAG;
    drv_l1_csi_irq_enable(1, INT_FLAG);
    drvl1_csi_enable(1);

	return ret;
}

/**
 * @brief	csi stream off
 * @param	none
 * @return	0: Success, -1: Fail
 */
INT32S drv_l2_csi_stream_off(void)
{
    drv_l1_csi_irq_enable(0, INT_FLAG);
    drv_l1_csi_set_irq(0);
    drv_l1_csi_state_clear(0x3F);
    drvl1_csi_enable(0);

	return 0;
}

/**
 * @brief	csi control off
 * @param	none
 * @return	0: Success, -1: Fail
 */
INT32S drv_l2_csi_stop(void)
{
    drvl1_csi_clockout_enable(0);
    drvl1_csi_enable(0);

	return 0;
}
#endif //(defined _DRV_L2_CDSP) && (_DRV_L2_CDSP == 1)
