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
/********************************************************************
* Purpose:  COMS censor interface layer 1 driver
* Author:
* Date: 	2014/09/09
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
* Version : 1.00
* History :
*********************************************************************/
#include "drv_l1_sfr.h"
#include "drv_l1_i2c.h"
#include "drv_l1_csi.h"

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined _DRV_L1_CSI) && (_DRV_L1_CSI == 1)                 //
//================================================================//
/****************************************************
*		Definition 									*
****************************************************/

/****************************************************
*		varaible and data declaration				*
****************************************************/
static	CSI_CALLBACK_FUN csi_callback = NULL;

/*****************************************
*			CSI functions				 *
*****************************************/
void drvl1_csi_input_set(CSI_INPUT_INTERFACE_DEF interface, CSI_INTERLACE_MODE_DEF interlace, INT32U invert_clk, CSI_INPUT_DATA_DEF data_format)
{
	if (interface == CSI_HREF) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_HREF;
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CCIR656;
	} else if (interface == CSI_HSYNC_CCIR_656) {
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_HREF;
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CCIR656;
	} else {		// ENUM_CSI_HSYNC_CCIR_601
		R_CSI_TG_CTRL0 &= ~(MASK_CSI_CTRL0_CCIR656 | MASK_CSI_CTRL0_HREF);
	}

	if (interlace == CSI_NON_INTERLACE) {
		R_CSI_TG_CTRL0 &= ~(MASK_CSI_CTRL0_INTL | MASK_CSI_CTRL0_FIELDINV);
	} else if (interlace == CSI_INTERLACE) {
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_FIELDINV;
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_INTL;
	} else {	// ENUM_CSI_INTERLACE_INVERT_FIELD
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_INTL | MASK_CSI_CTRL0_FIELDINV;
	}

	if (invert_clk) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CLKIINV;
	} else {
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CLKIINV;
	}

	if (data_format < CSI_IN_UYVY) {
		// RGB
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_YUVIN;
		if (data_format == CSI_IN_RGB888_BGRG || data_format == CSI_IN_RGB888_GBGR) {
			R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CCIR656;
		} else {
			R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CCIR656;
		}

		if (data_format == CSI_IN_RGB888_BGRG || data_format == CSI_IN_RGB888_GBGR) {
			R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_YUVTYPE;
		} else {
			R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVTYPE;
		}
	} else {
        // YUV
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVIN;
		switch(data_format)
		{
            case CSI_IN_UYVY:
                R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_YUVTYPE;
                R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_INVYUVI;
                break;
            case CSI_IN_UYVY_INVERT_UV7:
                R_CSI_TG_CTRL0 &= MASK_CSI_CTRL0_YUVTYPE;
                R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_INVYUVI;
                break;

            case CSI_IN_YUYV:
                R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVTYPE;
                R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_INVYUVI;
                break;

            case CSI_IN_YUYV_INVERT_UV7:
                R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVTYPE;
                R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_INVYUVI;
                break;
		}
	}
}

INT32S drvl1_csi_output_data_format_set(CSI_OUTPUT_DATA_DEF data_format)
{
	if(data_format < CSI_OUT_VYUY)
	{
        // RGB
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_YUVOUT;
		if (data_format == CSI_OUT_RGB565) {
			R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_RGB1555;
		} else {
			R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_RGB1555;
		}
	} else {		// YUV
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVOUT;
		R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_YONLY;
		switch(data_format)
		{
            case CSI_OUT_VYUY:
                R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_INVYUVO;
                R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_RGB1555;
                break;
            case CSI_OUT_VYUY_INVERT_UV7:
                R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_INVYUVO;
                R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_RGB1555;
                break;

            case CSI_OUT_YUYV:
                R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_INVYUVO;
                R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_RGB1555;
                break;

            case CSI_OUT_YUYV_INVERT_UV7:
                R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_INVYUVO;
                R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_RGB1555;
                break;

            case CSI_OUT_Y_ONLY:
                R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_YONLY;
                break;
		}
	}

	return 0;
}

INT32S drvl1_csi_scale_down_set(INT32U ho, INT32U vo)
{
    R_CSI_TG_HRATIO = ho;
    R_CSI_TG_VRATIO = vo;

	return 0;
}

INT32S drvl1_csi_cubic_output_set(INT32U enable, INT32U size)
{
	if(enable)
	{
        R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_CUNICEN;
        if(size)
            R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_CUNIC_32_32;
        else
            R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CUNIC_32_32;
	}
    else
    {
        R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CUNICEN;
        R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CUNIC_32_32;
    }

	return 0;
}

INT32S drvl1_csi_cut_screen_set(INT32U enable, INT32U cut_s, INT32U cut_e)
{
	if(enable)
	{
        R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_CUTEN;
        R_CSI_TG_CUTSTART = cut_s;
        R_CSI_TG_CUTSIZE = cut_e;
	}
    else
    {
        R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CUTEN;
        R_CSI_TG_CUTSTART = 0;
        R_CSI_TG_CUTSIZE = 0;
    }

	return 0;
}

INT32S drvl1_csi_black_screen_set(INT32U hs, INT32U he, INT32U vs, INT32U ve)
{
    R_CSI_TG_HSTART = hs;
    R_CSI_TG_HEND = he;
    R_CSI_TG_VSTART = vs;
    R_CSI_TG_VEND = ve;

	return 0;
}

INT32S drvl1_csi_blue_screen_set(INT32U enable, INT32U bs_upper, INT32U bs_lower)
{
	if(enable)
	{
        R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_BSEN;
        R_CSI_TG_BSUPPER = bs_upper;
        R_CSI_TG_BSLOWER = bs_lower;
	}
    else
    {
        R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_BSEN;
        R_CSI_TG_BSUPPER = 0;
        R_CSI_TG_BSLOWER = 0;
    }

	return 0;
}

INT32S drvl1_csi_enable(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CSIEN;
    else
        R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CSIEN;

	return 0;
}

INT32S drvl1_csi_clockout_enable(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_CLKOEN;
    else
        R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CLKOEN;

	return 0;
}

INT32S drvl1_csi_field_invert_set(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_FIELDINV;
    else
        R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_FIELDINV;

	return 0;
}

INT32S drvl1_csi_clock_stop_set(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_CLK_STOPB;
    else
        R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CLK_STOPB;

	return 0;
}

INT32S drvl1_csi_clock_priority_set(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_CLK_PRI;
    else
        R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CLK_PRI;

	return 0;
}

INT32S drvl1_csi_preview_mode_set(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CAP;
    else
        R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CAP;

	return 0;
}

INT32S drvl1_csi_long_burst_set(INT32U enable)
{
	if(enable)
        R_CSI_SEN_CTRL |= MASK_CSI_SEN_LONG_BURST;
    else
        R_CSI_SEN_CTRL &= ~MASK_CSI_SEN_LONG_BURST;

	return 0;
}

INT32S drvl1_csi_jpeg_mode_set(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_JPEG;
    else
        R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_JPEG;

	return 0;
}

INT32S drvl1_csi_clko_invert_set(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_CLKOINV;
    else
        R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CLKOINV;

	return 0;
}

INT32S drvl1_csi_fifo_mode_set(INT32U mode)
{
    R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_FIFO;
    R_CSI_TG_CTRL0 |= mode;

	return 0;
}

INT32S drvl1_csi_input_resolution_set(INT32U width, INT32U height)
{
	if (width>4095 || height>4095) {
		return -1;
	}

	R_CSI_TG_HWIDTH = width;
	R_CSI_TG_VHEIGHT = height;

	return 0;
}

INT32S drvl1_csi_input_pscaler_set(INT32U enable)
{
	if(enable)
        R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_INPUT_TO_PSCALER;
    else
        R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_INPUT_TO_PSCALER;

	return 0;
}

// Latch control registers are used in HSYNC mode only. It is not used in HREF mode
void drvl1_csi_input_latch_timing1_set(INT32U h_start, INT32U field0_vstart, INT32U field1_vstart, CSI_LATCH_DELAY_ENUM delay)
{
	R_CSI_TG_HLSTART = h_start & 0x0FFF;  				// Horizontal latch start
	R_CSI_TG_VL0START = field0_vstart & 0x0FFF;  	// Field 0 vertical latch start
	R_CSI_TG_VL1START = field1_vstart & 0x0FFF;		// Field 1 vertical latch start when interlace mode is used

	R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_LATCH_DELAY_MASK;
	R_CSI_TG_CTRL1 |= delay;
}

void drvl1_csi_input_latch_timing2_set(CSI_LATCH_EDGE_ENUM field, CSI_LATCH_EDGE_ENUM v_reset, CSI_LATCH_EDGE_ENUM v_increase, CSI_LATCH_EDGE_ENUM h_reset)
{
	// Latch field at falling(0) or rising(1) edge of VSYNC
	if (field == ENUM_CSI_FALLING_EDGE) {
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_FGET;
	} else {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_FGET;
	}

	// Vertical counter increase at falling(0) or rising(1) edge of HSYNC
	if (v_increase == ENUM_CSI_FALLING_EDGE) {
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_VADD;
	} else {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_VADD;
	}

	// Vertical counter reset at falling(0) or rising(1) edge of VSYNC
	if (v_reset == ENUM_CSI_FALLING_EDGE) {
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_VRST;;
	} else {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_VRST;
	}

	// Horizontal counter reset at falling(0) or rising(1) edge of HSYNC
	if (h_reset == ENUM_CSI_FALLING_EDGE) {
		R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_HRST;
	} else {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_HRST;
	}
}

void drv_l1_register_csi_cbk(CSI_CALLBACK_FUN fun)
{
	csi_callback = fun;
}

void drv_l1_csi_state_clear(INT32U flag)
{
    R_TGR_IRQ_STATUS = flag;
}

void CSI_IRQHandler(void)
{
	INT32U flag = R_TGR_IRQ_STATUS;
	INT32U enable = R_TGR_IRQ_EN;
	INT32U event = CSI_NONE_EVENT;

	flag &= enable;
	if(flag & MASK_CSI_OUTPUT_FIFO_FLAG)
	{
		/* Sensor FIFO mode occurred */
		R_TGR_IRQ_STATUS = MASK_CSI_OUTPUT_FIFO_FLAG;
		event = CSI_SENSOR_OUTPUT_FIFO_EVENT;
	}
	else if(flag & MASK_CSI_FIFO_OVERFLOW_FLAG)
	{
		/* Sensor under run */
		R_TGR_IRQ_STATUS = MASK_CSI_FIFO_OVERFLOW_FLAG;
		event = CSI_SENSOR_FIFO_OVERFLOW_EVENT;
	}
	else if(flag & MASK_CSI_FRAME_END_FLAG)
	{
		/* Sensor frame end occurred */
		R_TGR_IRQ_STATUS = MASK_CSI_FRAME_END_FLAG;
		event = CSI_SENSOR_FRAME_END_EVENT;
	}
	else if(flag & MASK_CSI_MOTION_DET_FLAG)
    {
		/* Sensor frame end occurred */
		R_TGR_IRQ_STATUS = MASK_CSI_MOTION_DET_FLAG;
		event = CSI_MOTION_DET_EVENT;
    }
	else {
		R_TGR_IRQ_STATUS = flag;
	}

	if(event != CSI_NONE_EVENT && csi_callback != NULL)
	{
		/* Call callback function come from sensor */
		csi_callback(event);
	}
}

void drv_l1_csi_set_buf(INT32U buf)
{
	R_CSI_TG_FBSADDR = buf;
}

INT32U drv_l1_csi_get_buf(void)
{
	return (INT32U)R_CSI_TG_FBSADDR;
}

void drv_l1_csi_set_irq(INT32U enable)
{
    if(enable) {
        NVIC_SetPriority(CSI_IRQn, 5);
        NVIC_EnableIRQ(CSI_IRQn);
    }
    else {
        NVIC_DisableIRQ(CSI_IRQn);
    }
}

void drv_l1_csi_irq_enable(INT32U enable, INT32U flag)
{
    if(enable)
    {
        R_TGR_IRQ_EN |= flag;
        R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_OWN_IRQ;
    }
    else
    {
        R_TGR_IRQ_EN &= flag;
        R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_OWN_IRQ;
    }
}

void drv_l1_csi_md_set_threshold(INT32U value)
{
    R_CSI_MD_CTRL |= ((value << 9) & (0x7F << 9));
}

void drv_l1_csi_md_set_mode(INT32U value)
{
    R_CSI_MD_CTRL |= ((value << 6) & (0x3 << 6));
}

void drv_l1_csi_md_set_frame(INT32U value)
{
    R_CSI_MD_CTRL |= ((value << 2) & (0x3 << 6));
}

void drv_l1_csi_md_set_parameter(INT32U enable, INT32U img_h, INT32U img_v, INT32 char_mode)
{
    INT32U value = 0;
    INT32U size,temp,temp1;

    if(enable)
    {
        if((char_mode == 0) && ((img_h == 320) && (img_v == 240)))
            size = 8;
        else
            size = 16;

        if((img_h == 640) && (img_v == 480))
            value |= (1 << 4);
        else if((img_h == 320) && (img_v == 240))
            value &= ~(1 << 4);
        else
        {
            // block h number
            temp = (img_h / size);
            // h multiple of 4
            temp1 = (temp % 4);
            if(temp1 == 1)
                temp+=3;
            else if(temp1 == 2)
                temp+=2;
            else if(temp1)
                temp+=1;
            value |= ((temp-1) & 0x3F) << 16;
            // block v number
            temp = (img_v / size);
            value |= ((temp-1) & 0x3F) << 24;
        }
        value |= (1 << 1);
    }

    R_CSI_MD_CTRL = value;
}

void drv_l1_csi_md_set_buf(INT32U buf)
{
    if(buf)
        R_CSI_MD_FBADDR = buf;
}

INT32U drv_l1_csi_md_get_buf(void)
{
    return  (INT32U)R_CSI_MD_FBADDR;
}

INT32U drv_l1_csi_md_get_block_size(INT32U img_h, INT32U img_v, INT32 char_mode)
{
    INT32U h,v,size,temp;

    if((char_mode == 0) && ((img_h == 320) && (img_v == 240)))
        size = 8;
    else
        size = 16;

    // block h number
    temp = (img_h / size);
    v = (temp % 4);
    if(v == 1)
        temp+=3;
    else if(v == 2)
        temp+=2;
    else if(v)
        temp+=1;
    h = temp;

    // block v number
    v = (img_v / size);

    return  (INT32U)(h*v);
}
#endif  //(defined _DRV_L1_CSI) && (_DRV_L1_CSI == 1)
