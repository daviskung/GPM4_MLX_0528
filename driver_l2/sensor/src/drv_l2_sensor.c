/******************************************************
* drv_l2_sensor.c
*
* Purpose: Sensor L2 driver
*
* Author: Eugene Hsu
*
* Date: 2014/08/25
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/

/*******************************************************
    Include file
*******************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_l1_sfr.h"
#include "drv_l1_gpio.h"
#include "drv_l1_csi.h"
#include "drv_l2_sensor.h"

#if (defined _DRV_L2_SENSOR) && (_DRV_L2_SENSOR == 1)
/******************************************************
    Definitions and variable declaration
*******************************************************/
static drv_l2_sensor_ops_t* sensor_obj[MAX_SENSOR_NUM];
static drv_l2_sensor_para_t sensor_para = {0};

/**********************************************
*	Extern variables and functions
**********************************************/

/*********************************************
*	Variables declaration
*********************************************/
#ifndef DBG_PRINT
#define DBG_PRINT   printf
#endif

/*****************************************************
    Utility functions
******************************************************/

/*+++++++++++++++++++++++++++++++*/
/*Select csi data pin & ctrl*/
void function_position_sel(INT32U mclk_pos, INT32U ctrl_pos, INT32U data_pos1, INT32U data_pos0)
{
    INT32U isp_reg;

    //isp_reg = R_FUNPOS1;
    //isp_reg &= ~0x03F00F80;
isp_reg = 0;

    switch(mclk_pos)
    {   //ISP Path
        case ISP_CLKO__IOC9:
            isp_reg |= ((1<<23) | (1<<20));
        break;

        case ISP_CLKO__IOD7:
            isp_reg |= ((1<<24) | (1<<21));
        break;

        case ISP_CLKO__IOD12:
            isp_reg |= ((1<<25) | (1<<22));
        break;
        //CSI Path
        case CSI_CLKO__IOC9:
            isp_reg |= ((0<<23) | (1<<20));
        break;

        case CSI_CLKO__IOD7:
            isp_reg |= ((0<<24) | (1<<21));
        break;

        case CSI_CLKO__IOD12:
            isp_reg |= ((0<<25) | (1<<22));
        break;
    }

    switch(ctrl_pos)
    {   //ISP path
        case ISP_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11:
            isp_reg |= (0<<7);
        break;

        case ISP_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9:
            isp_reg |= (1<<7);
        break;

        //CSI path
        case CSI_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11:
            isp_reg |= (0<<2);
        break;

        case CSI_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9:
            isp_reg |= (1<<2);
        break;
    }

    switch(data_pos1)
    {   //ISP path
        case ISP_DATA2_9__IOC0_7:
            isp_reg |= (0<<8);
        break;

        case ISP_DATA2_9__IOB8_15:
            isp_reg |= (1<<8);
        break;

        case ISP_DATA2_9__IOE0_7:
            isp_reg |= (2<<8);
        break;

        //CSI path
        case CSI_DATA2_9__IOC0_7:
            isp_reg |= (0<<3);
        break;

        case CSI_DATA2_9__IOB8_15:
            isp_reg |= (1<<3);
        break;

        case CSI_DATA2_9__IOE0_7:
            isp_reg |= (2<<3);
        break;

    }

    switch(data_pos0)
    {   //ISP only
        case ISP_DATA0_1__IOB7_6:
            //isp_reg |= (0<<10);
            isp_reg &= ~(1<<10);
        break;

        case ISP_DATA0_1__IOB5_4:
            isp_reg |= (1<<10);
        break;

         case ISP_DATA0_1__IOC13_12:
            isp_reg |= (2<<10);
        break;
    }

    R_FUNPOS1 = isp_reg;
}

/*+++++++++++++++++++++++++++++++*/
/*Select csi clk pin*/
void drv_l2_sensor_clkout_position_set(INT32U mclk_pos)
{
    INT32U isp_reg = R_FUNPOS1;

    switch(mclk_pos)
    {   //ISP Path
        case ISP_CLKO__IOC9:
            isp_reg |= ((1<<23) | (1<<20));
        break;

        case ISP_CLKO__IOD7:
            isp_reg |= ((1<<24) | (1<<21));
        break;

        case ISP_CLKO__IOD12:
            isp_reg |= ((1<<25) | (1<<22));
        break;

        //CSI Path
        case CSI_CLKO__IOC9:
            isp_reg &=~(1<<23);
            isp_reg |= (1<<20);
        break;

        case CSI_CLKO__IOD7:
            isp_reg &=~(1<<24);
            isp_reg |= (1<<21);
        break;

        case CSI_CLKO__IOD12:
            isp_reg &=~(1<<25);
            isp_reg |= (1<<22);
        break;
    }

     R_FUNPOS1 = isp_reg;
}

/*-------------------------------*/
/**
 * @brief   Register sensor operation function table
 * @param   sensor : operation function table
 * @return 	none
 */
void drv_l2_sensor_register_ops(drv_l2_sensor_ops_t* sensor)
{
	INT32U i;

	for(i=0; i<MAX_SENSOR_NUM; i++) {
		if(sensor_obj[i] == 0) {
			sensor_obj[i] = sensor;
			break;
		}
	}

	if(i == MAX_SENSOR_NUM) {
		DBG_PRINT("sensor_obj not find the empty when register ops\r\n");
	}
}
/**
 * @brief   Get sensor operation function table
 * @param   index : sensor index
 * @return 	sensor operation function table
 */
drv_l2_sensor_ops_t* drv_l2_sensor_get_ops(INT32U index)
{
	if(index > MAX_SENSOR_NUM) {
		DBG_PRINT("Index[%d] is wrong when get ops\r\n", index);
		return NULL;
	}

	return sensor_obj[index];
}

/**
 * @brief   Get sensor parameter function index
 * @param   none
 * @return 	sensor parameter function index
 */
drv_l2_sensor_para_t* drv_l2_sensor_get_para(void)
{
	return &sensor_para;
}

/**
 * @brief   Sensor driver layer 2 initialization function
 * @param   none
 * @return 	none
 */
void drv_l2_sensor_init(void)
{
	INT32U i;
	/* reset sensor_obj pointer to NULL */
	for(i=0; i<MAX_SENSOR_NUM; i++) {
		sensor_obj[i] = NULL;
	}

	/* register sensor ops for specified sensor */

#if (defined _SENSOR_OV7670_CSI) && (_SENSOR_OV7670_CSI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov7670_sensor_csi_ops);
#endif
#if (defined _SENSOR_GC0308_CSI) && (_SENSOR_GC0308_CSI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&gc0308_sensor_csi_ops);
#endif
#if (defined _SENSOR_OV3640_CSI) && (_SENSOR_OV3640_CSI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov3640_sensor_csi_ops);
#endif
#if (defined _SENSOR_OV7670_CDSP) && (_SENSOR_OV7670_CDSP == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov7670_cdsp_ops);
#endif
#if (defined _SENSOR_OV5650_CDSP_MIPI) && (_SENSOR_OV5650_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov5650_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_OV9712_CDSP) && (_SENSOR_OV9712_CDSP == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov9712_cdsp_ops);
#endif
#if (defined _SENSOR_OV3640_CSI_MIPI) && (_SENSOR_OV3640_CSI_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov3640_csi_mipi_ops);
#endif
#if (defined _SENSOR_OV3640_CDSP_MIPI) && (_SENSOR_OV3640_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov3640_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_AR0330_CDSP_MIPI) && (_SENSOR_AR0330_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ar0330_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_JXF02_CDSP_MIPI) && (_SENSOR_JXF02_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&jxf02_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_SOI_H22_CDSP_MIPI) && (_SENSOR_SOI_H22_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&soi_h22_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_GC1004_CDSP_MIPI) && (_SENSOR_GC1004_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&gc1004_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_GC1004_CDSP_DVP) && (_SENSOR_GC1004_CDSP_DVP == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&gc1004_cdsp_dvp_ops);
#endif
#if (defined _SENSOR_JXH22_CDSP_DVP) && (_SENSOR_JXH22_CDSP_DVP == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&jxh22_cdsp_dvp_ops);
#endif
#if (defined _SENSOR_H42_CDSP_MIPI) && (_SENSOR_H42_CDSP_MIPI == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&h42_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_H62_CDSP_MIPI) && (_SENSOR_H62_CDSP_MIPI == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&h62_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_GC1014_CDSP_DVP) && (_SENSOR_GC1014_CDSP_DVP == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&gc1014_cdsp_dvp_ops);
#endif
#if (defined _SENSOR_GC1014_CDSP_MIPI) && (_SENSOR_GC1014_CDSP_MIPI == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&gc1014_cdsp_mipi_ops);
#endif
#if (defined _TVIN_TVP5150_CSI) && (_TVIN_TVP5150_CSI == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&tvp5150_csi_ops);
#endif
#if (defined _SENSOR_OV5658_CDSP_MIPI) && (_SENSOR_OV5658_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&ov5658_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_SP5506_CDSP_MIPI) && (_SENSOR_SP5506_CDSP_MIPI == 1)
	drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&sp5506_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_GC1064_CDSP_MIPI) && (_SENSOR_GC1064_CDSP_MIPI == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&GC1064_cdsp_mipi_ops);
#endif
#if (defined _SENSOR_GC5025_CDSP_MIPI) && (_SENSOR_GC5025_CDSP_MIPI == 1)
    drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&gc5025_cdsp_mipi_ops);
#endif

#if (defined _SENSOR_MXL90640_THERMOPILE) && (_SENSOR_MXL90640_THERMOPILE == 1)
		drv_l2_sensor_register_ops((drv_l2_sensor_ops_t*)&mlx90640_sensor_thermal_ops);
#endif


}

/**
* @brief	select internal data path
* @param	sen_path: csi or cdsp,
* @return	none
*/
void drv_l2_sensor_set_path(INT32U sen_path)
{
        if (sen_path == 1) {
		R_SYSTEM_MISC_CTRL5 |= (1<<4);
	} else {
		R_SYSTEM_MISC_CTRL5 &= ~(1<<4);
	}
}

/**
 * @brief   set csi input source path
 * @param   image_src[in]: 0: from dvp sensor(parallel sensor), 1: from mipi sensor
 * @return 	none
 */
void drv_l2_sensor_set_csi_source(INT32U image_src)
{
	if(image_src == 0) {
		R_SYSTEM_MISC_CTRL0 &= ~(1 << 0);	//form parallel sensor interface
	} else {
		R_SYSTEM_MISC_CTRL0 |= (1 << 0);	//form mipi interface
	}
}

/**
* @brief	select mclk out clock speed
* @param	mclk_sel:
* @return	speed
*/
INT32S drv_l2_sensor_set_mclkout(INT32U mclk_sel)
{
	INT32U mclk_out;

	//enable 48MHz
	R_SYSTEM_CTRL |= (1 << 11);

	R_FUNPOS1 |= ((1<<23)|(1<<20));			//IOC9 MCLK Enable, Disable for mipi to IOD12
	//R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<7));	//IOD7 MCLK for dvp + cdsp([7]=1)	//u2 mark

	// set mclk
	switch(mclk_sel)
	{
	case MCLK_NONE:
		R_CSI_TG_CTRL1 &= ~(1 << 7);
		R_SYSTEM_CTRL &= ~(1 << 8);
		return 0;

	case MCLK_12M:
        R_SYSTEM_CLK_CTRL |= 0x08;	//Enable Mclk (96Mhz)
		R_SYSTEM_CTRL |= (1 << 14) | (1 << 8);
		R_CSI_TG_CTRL1 |= (1 << 11);
		mclk_out = 12000000;
		break;

	case MCLK_13_5M:
		R_SYSTEM_CTRL |= (1 << 14) | (1 << 11) | (1 << 8);
		R_CSI_TG_CTRL1 &= ~(1 << 11);
		mclk_out = 13500000;
		break;

	case MCLK_27M:
		R_SYSTEM_CTRL &= ~(1 << 14);
		R_SYSTEM_CTRL |= (1 << 11) | (1 << 8);
		R_CSI_TG_CTRL1 &= ~(1 << 11);
		mclk_out = 27000000;
		break;

	case MCLK_24M:
		// 0xD000000C[14][11][8] set 1, from XCKGEN 24Mhz
		R_SYSTEM_CTRL |= (1 << 14) | (1 << 11) | (1 << 8);
		R_CSI_TG_CTRL1 |= (1 << 11);
		mclk_out = 24000000;
		break;

	case MCLK_48M:
		R_SYSTEM_CTRL &= ~(1 << 14);
		R_SYSTEM_CTRL |= (1 << 11) | (1 << 8);
		R_CSI_TG_CTRL1 |= (1 << 11);
		mclk_out = 48000000;
		break;

	default:
		return 0;
	}

	// Enable CSI MCLK out
	R_CSI_TG_CTRL1 |= (1 << 7);
	return mclk_out;
}

void drv_l2_mipi_ctrl_set_clk(INT8U mipiclk_on, INT32U mipiclk_div)
{
    INT16U reg;

    R_SYSTEM_MIPI_CTRL_CLK = 0;

    if (mipiclk_on)
    {
        reg = (1 << 7);
        reg |= (mipiclk_div - 1);

        R_SYSTEM_MIPI_CTRL_CLK = reg;
    }
}


void drv_l2_user_sensor_init(INT32U isp_bufferA, INT32U isp_bufferB)
{
    drv_l2_sensor_init();

#if (defined _SENSOR_OV7670_CSI) && (_SENSOR_OV7670_CSI == 1)
    ov7670_csi_init();
	ov7670_csi_stream_start(0,isp_bufferA,isp_bufferB);
#endif

#if (defined _SENSOR_GC0308_CSI) && (_SENSOR_GC0308_CSI == 1)
	gc0308_csi_init();
	gc0308_csi_stream_start(0,isp_bufferA,isp_bufferB);
#endif

#if (defined _SENSOR_OV3640_CDSP_MIPI) && (_SENSOR_OV3640_CDSP_MIPI == 1)
    ov3640_cdsp_mipi_init();
    ov3640_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif


#if (defined _SENSOR_AR0330_CDSP_MIPI) && (_SENSOR_AR0330_CDSP_MIPI == 1)
    ar0330_cdsp_mipi_init();
    ar0330_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_JXF02_CDSP_MIPI) && (_SENSOR_JXF02_CDSP_MIPI == 1)
    jxf02_cdsp_mipi_init();
    jxf02_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif


#if (defined _SENSOR_OV5650_CDSP_MIPI) && (_SENSOR_OV5650_CDSP_MIPI == 1)
    ov5650_cdsp_mipi_init();
    ov5650_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_JXH22_CDSP_DVP) && (_SENSOR_JXH22_CDSP_DVP == 1)
    jxh22_cdsp_dvp_init();
    jxh22_cdsp_dvp_stream_on(3, isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_H42_CDSP_MIPI) && (_SENSOR_H42_CDSP_MIPI == 1)
    h42_cdsp_mipi_init();
    h42_cdsp_mipi_stream_on(3,isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_H62_CDSP_MIPI) && (_SENSOR_H62_CDSP_MIPI == 1)
    h62_cdsp_mipi_init();
    h62_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_JXH42_CDSP_DVP) && (_SENSOR_JXH42_CDSP_DVP == 1)
    jxh42_cdsp_dvp_init();
    jxh42_cdsp_dvp_stream_on(2, isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_GC1004_CDSP_DVP) && (_SENSOR_GC1004_CDSP_DVP == 1)
    gc1004_cdsp_dvp_init();
    gc1004_cdsp_dvp_stream_on(2, isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_GC1004_CDSP_MIPI) && (_SENSOR_GC1004_CDSP_MIPI == 1)
    gc1004_cdsp_mipi_init();
    gc1004_cdsp_mipi_stream_on(3, isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_GC1014_CDSP_DVP) && (_SENSOR_GC1014_CDSP_DVP == 1)
    gc1014_cdsp_dvp_init();
    gc1014_cdsp_dvp_stream_on(2, isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_GC1014_CDSP_MIPI) && (_SENSOR_GC1014_CDSP_MIPI == 1)
    gc1014_cdsp_mipi_init();
    gc1014_cdsp_mipi_stream_on(3, isp_bufferA, isp_bufferB);
#endif

#if (defined _TVIN_TVP5150_CSI) && (_TVIN_TVP5150_CSI == 1)
    tvp5150_csi_init();
    tvp5150_csi_stream_start(0,isp_bufferA,isp_bufferB);
#endif

/*
#if (defined _SENSOR_GC0308_CSI) && (_SENSOR_GC0308_CSI == 1)
    gc0308_csi_init();
	gc0308_csi_stream_start(0,isp_bufferA,isp_bufferB);
#endif
*/

#if (defined _SENSOR_OV5658_CDSP_MIPI) && (_SENSOR_OV5658_CDSP_MIPI == 1)
    ov5658_cdsp_mipi_init();
    ov5658_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif
#if (defined _SENSOR_SP5506_CDSP_MIPI) && (_SENSOR_SP5506_CDSP_MIPI == 1)
    sp5506_cdsp_mipi_init();
    sp5506_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif

#if (defined _SENSOR_GC1064_CDSP_MIPI) && (_SENSOR_GC1064_CDSP_MIPI == 1)
    GC1064_cdsp_mipi_init();
    GC1064_cdsp_mipi_stream_on(2, isp_bufferA, isp_bufferB);
#endif
#if (defined _SENSOR_GC5025_CDSP_MIPI) && (_SENSOR_GC5025_CDSP_MIPI == 1)
    gc5025_cdsp_mipi_init();
    gc5025_cdsp_mipi_stream_on(1,isp_bufferA, isp_bufferB);
#endif
}
#endif //_DRV_L2_SENSOR
