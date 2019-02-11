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
#include "drv_l1_sfr.h"
#include "drv_l1_csi.h"
#include "drv_l1_mipi.h"
#include "drv_l1_i2c.h"
#include "drv_l1_gpio.h"
#include "drv_l2_sensor.h"
#include "drv_l2_sccb.h"
#include "drv_l2_cdsp.h"



#if (defined _SENSOR_SP5506_CDSP_MIPI) && (_SENSOR_SP5506_CDSP_MIPI == 1)
#include "drv_l2_user_preference.h"
#include "drv_l2_user_calibration.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_TEST        0
#define COLOR_BAR_EN            0
#define MIPI_ISR_TEST           1
#define MIPI_LANE_NO			2	        // 1 or 2 lane
#define MIPI_DEV_NO             0           //0:MIPI_0 or 1:MIPI_1

#define RAW_MODE 0

#define	SP5506_ID						0x6C //0x20// 0x6C
#define SP5506_WIDTH				    2560//1920//2560// (2560)  -->MIPI RX input
#define SP5506_HEIGHT				    (1944-8)//1096//1944-8// (1920-8)

#define SP5506_OUT_WIDTH				2560//1280//2560 //--->CDSP YUV output
#define SP5506_OUT_HEIGHT				(1944-16)//720//1944-16

// sccb interface
#define SCCB_GPIO 		0
#define SCCB_HW_I2C 	1
#define SCCB_MODE 	  SCCB_GPIO//SCCB_HW_I2C//SCCB_GPIO



#define SP5506_25FPS_50HZ_DAY_EV_IDX 			146
#define SP5506_25FPS_50HZ_NIGHT_EV_IDX			190
#define SP5506_25FPS_50HZ_EXP_TIME_TOTAL		214
#define SP5506_25FPS_50HZ_INIT_EV_IDX 			((SP5506_25FPS_50HZ_DAY_EV_IDX + SP5506_25FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define SP5506_25FPS_50HZ_MAX_EV_IDX			  (SP5506_25FPS_50HZ_EXP_TIME_TOTAL - 12)




#define SP5506_25FPS_60HZ_DAY_EV_IDX 			146
#define SP5506_25FPS_60HZ_NIGHT_EV_IDX			190
#define SP5506_25FPS_60HZ_EXP_TIME_TOTAL		252
#define SP5506_25FPS_60HZ_INIT_EV_IDX 			((SP5506_25FPS_60HZ_DAY_EV_IDX + SP5506_25FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define SP5506_25FPS_60HZ_MAX_EV_IDX			(SP5506_25FPS_60HZ_EXP_TIME_TOTAL - 12)
/*
#define SP5506_30FPS_50HZ_DAY_EV_IDX 			132
#define SP5506_30FPS_50HZ_NIGHT_EV_IDX			178
#define SP5506_30FPS_50HZ_EXP_TIME_TOTAL		246
#define SP5506_30FPS_50HZ_INIT_EV_IDX 			180//((SP5506_30FPS_50HZ_DAY_EV_IDX + SP5506_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define SP5506_30FPS_50HZ_MAX_EV_IDX			(SP5506_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define SP5506_30FPS_60HZ_DAY_EV_IDX 			132
#define SP5506_30FPS_60HZ_NIGHT_EV_IDX			178
#define SP5506_30FPS_60HZ_EXP_TIME_TOTAL		    249
#define SP5506_30FPS_60HZ_INIT_EV_IDX 			180//((SP5506_30FPS_60HZ_DAY_EV_IDX + SP5506_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define SP5506_30FPS_60HZ_MAX_EV_IDX			    (SP5506_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define SP5506_24FPS_50HZ_DAY_EV_IDX 			138
#define SP5506_24FPS_50HZ_NIGHT_EV_IDX			194
#define SP5506_24FPS_50HZ_EXP_TIME_TOTAL		    254
#define SP5506_24FPS_50HZ_INIT_EV_IDX 			((SP5506_24FPS_50HZ_DAY_EV_IDX + SP5506_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define SP5506_24FPS_50HZ_MAX_EV_IDX			    (SP5506_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define SP5506_24FPS_60HZ_DAY_EV_IDX 			135
#define SP5506_24FPS_60HZ_NIGHT_EV_IDX			195
#define SP5506_24FPS_60HZ_EXP_TIME_TOTAL		    255
#define SP5506_24FPS_60HZ_INIT_EV_IDX 			((SP5506_24FPS_60HZ_DAY_EV_IDX + SP5506_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define SP5506_24FPS_60HZ_MAX_EV_IDX			    (SP5506_24FPS_60HZ_EXP_TIME_TOTAL - 10)

*/


/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct regval16_s
{
	INT16U reg_num;
	INT8U value;
} regval16_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static int ob_cnt = 0;
static int *p_expTime_table;
static sensor_exposure_t 	seInfo;
static int pre_sensor_a_gain=0, pre_sensor_time=0;


#if SCCB_MODE == SCCB_GPIO
	static void *sp5506_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t sp5506_handle;
#endif

static mipi_config_t sp5506_mipi_cfg =
{
	DISABLE, 			/* low power, 0:disable, 1:enable */

#if MIPI_LANE_NO == 1
	D_PHY_SAMPLE_NEG, 	/* byte clock edge, 0:posedge, 1:negedge */
	MIPI_1_LANE,		/* lane */
	GENERATE_PIXEL_CLK,	/* byte clock source */
#elif MIPI_LANE_NO == 2
	D_PHY_SAMPLE_POS,
	MIPI_2_LANE,		/* lane */
	GENERATE_PIXEL_CLK,	/* byte clock source */
#endif

	MIPI_AUTO_DETECT,	/* data mode, 0:auto detect, 1:user define */
	MIPI_RAW10,			/* data type, valid when data mode is 1*/
	MIPI_DATA_TO_CDSP,	/* data type, 1:data[7:0]+2':00 to cdsp, 0: 2'00+data[7:0] to csi */
	0,				/* RSD 2 */

	SP5506_WIDTH,		/* width, 0~0xFFFF */
	SP5506_HEIGHT,		/* height, 0~0xFFFF */
	4, 					/* back porch, 0~0xF */
	4,					/* front porch, 0~0xF */
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,				/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	200, //200,				/* data mask, unit is ns ; mask_count = ns *193.5/1000 */
	MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};

static INT32S sp5506_sccb_read(INT16U reg, INT8U *value);
static INT32S sp5506_sccb_write(INT16U reg, INT8U value);

void dump_sensor_clock(INT16U srcMHz)
{
    INT16U refClock = srcMHz;
    INT16U pll1, pll2;
    INT16U div;
    INT16U multi;
    INT8U  value;
    INT16U halfword_value;
    INT8U  byte_value;
    INT16U mipi_phy_clock;
    INT16U mipi_pclk;
    INT16U sclk;
    INT16U dac_clk;

    sp5506_sccb_read(0x0307, &value);
    div = (value & 0x01)? 2 : 1;
    pll1 = refClock / div;
    //DBG_PRINT("1-> %f\r\n", pll1);

    sp5506_sccb_read(0x0300, &value);
    switch(value & 0x7){
        case 0:
            div = 10;
            break;
        case 1:
            div = 15;
            break;
        case 2:
            div = 20;
            break;
        case 3:
            div = 25;
            break;
        case 4:
            div = 30;
            break;
        case 5:
            div = 40;
            break;
        case 6:
            div = 60;
            break;
        case 7:
            div = 80;
            break;
    }

    pll1 = pll1 * 10 /div;
    //DBG_PRINT("2-> %f\r\n", pll1);

    sp5506_sccb_read(0x0301, &value);
    halfword_value = (value&0x3) << 8;
    sp5506_sccb_read(0x0302, &value);
    halfword_value += value;

    pll1 *= halfword_value;
    //DBG_PRINT("3-> %f\r\n", pll1);

    sp5506_sccb_read(0x0303, &value);
    byte_value = 1 + (value & 0x3);

    pll1 /= byte_value;
    mipi_phy_clock = pll1;
    //DBG_PRINT("mipi_phy_clock %d\r\n", mipi_phy_clock);

    sp5506_sccb_read(0x0304, &value);
    byte_value = (value & 0x1)? 5 : 4;
    mipi_pclk = pll1 / byte_value;

    sp5506_sccb_read(0x3020, &value);
    byte_value = (value & 0x8)? 2 : 1;
    mipi_pclk = pll1 / byte_value;
    //DBG_PRINT("mipi_pclk %d\r\n", mipi_pclk);

    sp5506_sccb_read(0x0305, &value);
    byte_value = (value & 0x1)? 5 : 4;
    sclk = pll1 / byte_value;

    ///////////////////////////////

    sp5506_sccb_read(0x0312, &value);
    byte_value = (value & 0x10)? 2: 1;
    pll2 = refClock / byte_value;

    sp5506_sccb_read(0x030B, &value);
    switch(value & 0x7){
        case 0:
            div = 10;
            break;
        case 1:
            div = 15;
            break;
        case 2:
            div = 20;
            break;
        case 3:
            div = 25;
            break;
        case 4:
            div = 30;
            break;
        case 5:
            div = 40;
            break;
        case 6:
            div = 60;
            break;
        case 7:
            div = 80;
            break;
    }
    pll2 = pll2 * 10 / div;

    sp5506_sccb_read(0x030C, &value);
    halfword_value = (value&0x3) << 8;
    sp5506_sccb_read(0x030D, &value);
    halfword_value += value;

    pll2 *= halfword_value;

    sp5506_sccb_read(0x3661, &value);
    byte_value = (value & 0x80)? 2 : 1;
    dac_clk = pll2 / byte_value;

    sp5506_sccb_read(0x030f, &value);
    byte_value = (value & 0x3) + 5;
    pll2 /= byte_value;

    sp5506_sccb_read(0x3660, &value);
    if(value & 0x10){
        sclk = pll2;
    }

    sp5506_sccb_read(0x3106, &value);

    switch((value & 0x0C) >> 2)
    {
        case 0:
            byte_value = 1;
            break;
        case 1:
            byte_value = 2;
            break;
        case 2:
            byte_value = 4;
            break;
        case 3:
            byte_value = 8;
            break;
    }

    sclk /= byte_value;

    DBG_PRINT("=> mipi_phy_clock = %d MHz\r\n", mipi_phy_clock);
    DBG_PRINT("=> mipi_pclk      = %d MHz\r\n", mipi_pclk);
    DBG_PRINT("=> sclk           = %d MHz\r\n", sclk);
    DBG_PRINT("=> dac_clk        = %d MHz\r\n", dac_clk);
}

void sp5506_set_exposure_time(INT32U ex_value)
{
    INT8U value;

    value = ex_value & 0xFF;
    sp5506_sccb_write(0x3502, value);
    ex_value >>= 8;

    value = ex_value & 0xFF;
    sp5506_sccb_write(0x3501, value);
    ex_value >>= 8;

    value = ex_value & 0xF;
    sp5506_sccb_write(0x3500, value);
}

void dump_exposure_time(void)
{
    INT32U ex_value;
    INT8U  value;

    sp5506_sccb_read(0x3500, &value);
    ex_value = (value & 0xF) << 16;

    sp5506_sccb_read(0x3501, &value);
    ex_value += value << 8;

    sp5506_sccb_read(0x3502, &value);
    ex_value += value;

    DBG_PRINT("Exposure Register Value: 0x%05x\r\n", ex_value);
}

void dump_current_sp5506_resolution(void)
{
    INT16U array_H_start;
    INT16U array_V_start;
    INT16U array_H_end;
    INT16U array_V_end;
    INT16U isp_output_H_width;
    INT16U isp_output_V_height;
    INT16U vsync_start;
    INT16U vsync_end;
    INT8U  value;

    sp5506_sccb_read(0x3800, &value);
    array_H_start = value << 8;
    sp5506_sccb_read(0x3801, &value);
    array_H_start += value;

    sp5506_sccb_read(0x3804, &value);
    array_H_end = value << 8;
    sp5506_sccb_read(0x3805, &value);
    array_H_end += value;

    DBG_PRINT(">> array H start : %d\r\n", array_H_start);
    DBG_PRINT(">> array H end   : %d\r\n", array_H_end);
    DBG_PRINT(">> array H width : %d\r\n", array_H_end - array_H_start);

    sp5506_sccb_read(0x3802, &value);
    array_V_start = value << 8;
    sp5506_sccb_read(0x3803, &value);
    array_V_start += value;

    sp5506_sccb_read(0x3806, &value);
    array_V_end = value << 8;
    sp5506_sccb_read(0x3807, &value);
    array_V_end += value;

    DBG_PRINT(">> array V start  : %d\r\n", array_V_start);
    DBG_PRINT(">> array V end    : %d\r\n", array_V_end);
    DBG_PRINT(">> array V Height : %d\r\n", array_V_end - array_V_start);

    sp5506_sccb_read(0x3808, &value);
    isp_output_H_width = value << 8;
    sp5506_sccb_read(0x3809, &value);
    isp_output_H_width += value;

    sp5506_sccb_read(0x380a, &value);
    isp_output_V_height = value << 8;
    sp5506_sccb_read(0x380b, &value);
    isp_output_V_height += value;

    DBG_PRINT(">> ISP Output H Width : %d\r\n", isp_output_H_width);
    DBG_PRINT(">> ISP Output V Height: %d\r\n", isp_output_V_height);

    sp5506_sccb_read(0x3818, &value);
    vsync_start = value << 8;
    sp5506_sccb_read(0x3819, &value);
    vsync_start += value;

    sp5506_sccb_read(0x381a, &value);
    vsync_end = value << 8;
    sp5506_sccb_read(0x381b, &value);
    vsync_end += value;

    DBG_PRINT(">> vsync start:   %d\r\n", vsync_start);
    DBG_PRINT(">> vsync end  :   %d\r\n", vsync_end);
    DBG_PRINT(">> vsync length : %d\r\n", vsync_end - vsync_start);
}

const regval16_t sp5506_Mipi_Raw10_5M_2Lane_25fps[] =
{
    { 0x0103, 0x01 },
    { 0x0100, 0x00 },
    { 0x0300, 0x05 },//
    { 0x0302, 0x80 },
    { 0x0303, 0x00 },//0x00
    { 0x3002, 0x21 },
    { 0x3107, 0x23 },
    { 0x3501, 0x20 },
    { 0x3503, 0x0c },
    { 0x3508, 0x03 },
    { 0x3509, 0x00 },
    { 0x3600, 0x66 },
    { 0x3602, 0x30 },
    { 0x3610, 0xa5 },
    { 0x3612, 0x93 },
    { 0x3620, 0x80 },
    { 0x3642, 0x0e },
    { 0x3661, 0x00 },
    { 0x3662, 0x10 },
    { 0x3664, 0xf3 },
    { 0x3665, 0x9e },
    { 0x3667, 0xa5 },
    { 0x366e, 0x55 },
    { 0x366f, 0x55 },
    { 0x3670, 0x11 },
    { 0x3671, 0x11 },
    { 0x3672, 0x11 },
    { 0x3673, 0x11 },
    { 0x3714, 0x24 },
    { 0x371a, 0x3e },
    { 0x3733, 0x10 },
    { 0x3734, 0x00 },
    { 0x373d, 0x24 },
    { 0x3764, 0x20 },
    { 0x3765, 0x20 },
    { 0x3766, 0x12 },
    { 0x37a1, 0x14 },
    { 0x37a8, 0x1c },
    { 0x37ab, 0x0f },
    { 0x37c2, 0x04 },
    { 0x37cb, 0x00 },
    { 0x37cc, 0x00 },
    { 0x37cd, 0x00 },
    { 0x37ce, 0x00 },
    { 0x37d8, 0x02 },
    { 0x37d9, 0x08 },
    { 0x37dc, 0x04 },
    { 0x3800, 0x00 },
    { 0x3801, 0x00 },
    { 0x3802, 0x00 },
    { 0x3803, 0x04 },
    { 0x3804, 0x0a },
    { 0x3805, 0x3f },
    { 0x3806, 0x07 },
    { 0x3807, 0xb3 },

    { 0x3808, 0x0a },
    //{ 0x3809, 0x20 },
    { 0x3809, 0x00 },
    { 0x380a, 0x07 },
    { 0x380b, 0x98 },

   /* { 0x3808, 0x07 }, //1080P
    { 0x3809, 0x80 },
    { 0x380a, 0x04 },
    { 0x380b, 0x48 },*/


    /*{ 0x3808, 0x07 }, //720P
    { 0x3809, 0x00 },
    { 0x380a, 0x02 },
    { 0x380b, 0xd0 },*/

    { 0x380c, 0x03 },
    { 0x380d, 0x7e },
    { 0x380e, 0x07 },
    { 0x380f, 0xd0 },
    { 0x3811, 0x10 },
    { 0x3813, 0x0c },
    { 0x3814, 0x01 },
    { 0x3815, 0x01 },
    { 0x3816, 0x01 },
    { 0x3817, 0x01 },
    { 0x381e, 0x02 },
    { 0x3820, 0x88 },
    { 0x3821, 0x01 },
    { 0x3832, 0x04 },
    { 0x3c80, 0x01 },
    { 0x3c82, 0x00 },
    { 0x3c83, 0xc8 },
    { 0x3c8c, 0x0f },
    { 0x3c8d, 0xa0 },
    { 0x3c90, 0x07 },
    { 0x3c91, 0x00 },
    { 0x3c92, 0x00 },
    { 0x3c93, 0x00 },
    { 0x3c94, 0xd0 },
    { 0x3c95, 0x50 },
    { 0x3c96, 0x35 },
    { 0x3c97, 0x00 },
    { 0x4001, 0xe0 },
    { 0x4008, 0x02 },
    { 0x4009, 0x0d },
    { 0x400f, 0x80 },
    { 0x4013, 0x02 },
    { 0x4040, 0x00 },
    { 0x4041, 0x07 },
    { 0x404c, 0x50 },
    { 0x404e, 0x20 },
    { 0x4500, 0x06 },
    { 0x4503, 0x00 },
    { 0x450a, 0x04 },
    { 0x4809, 0x04 },
    { 0x480c, 0x12 },
    { 0x4819, 0x70 },
    { 0x4825, 0x32 },
    { 0x4826, 0x32 },
    { 0x482a, 0x06 },
    { 0x4833, 0x08 },
    { 0x4837, 0x0d },
    { 0x5000, 0x77 },
    { 0x5b00, 0x01 },
    { 0x5b01, 0x10 },
    { 0x5b02, 0x01 },
    { 0x5b03, 0xdb },
    { 0x5b05, 0x6c },
    { 0x5e10, 0xfc },
    { 0x3500, 0x00 },
    { 0x3501, 0x3E },
    { 0x3502, 0x60 },
    { 0x3503, 0x08 },
    { 0x3508, 0x04 },
    { 0x3509, 0x00 },
    { 0x3832, 0x48 },
    { 0x3c90, 0x00 },
    { 0x5780, 0x3e },
    { 0x5781, 0x0f },
    { 0x5782, 0x44 },
    { 0x5783, 0x02 },
    { 0x5784, 0x01 },
    { 0x5785, 0x01 },
    { 0x5786, 0x00 },
    { 0x5787, 0x04 },
    { 0x5788, 0x02 },
    { 0x5789, 0x0f },
    { 0x578a, 0xfd },
    { 0x578b, 0xf5 },
    { 0x578c, 0xf5 },
    { 0x578d, 0x03 },
    { 0x578e, 0x08 },
    { 0x578f, 0x0c },
    { 0x5790, 0x08 },
    { 0x5791, 0x06 },
    { 0x5792, 0x00 },
    { 0x5793, 0x52 },
    { 0x5794, 0xa3 },
    { 0x4003, 0x40 },
    { 0x3107, 0x01 },
    { 0x3c80, 0x08 },
    { 0x3c83, 0xb1 },
    { 0x3c8c, 0x10 },
    { 0x3c8d, 0x00 },
    { 0x3c90, 0x00 },
    { 0x3c94, 0x00 },
    { 0x3c95, 0x00 },
    { 0x3c96, 0x00 },
    { 0x3d8c, 0x71 },
    { 0x3d8d, 0xe7 },
    { 0x37cb, 0x09 },
    { 0x37cc, 0x15 },
    { 0x37cd, 0x1f },
    { 0x37ce, 0x1f },
    { 0x0100, 0x01 },

    { 0x0000,  0x00 }
};

const regval16_t sp5506_Mipi_Raw10_5M_2Lane_30fps[] =
{
    { 0x0100, 0x00 },
    { 0x0103, 0x01 },
    { 0x0300, 0x05 },
    { 0x0302, 0x96 },
    //{ 0x0303, 0x00 },
    { 0x0303, 0x01 },
    { 0x3002, 0x21 },
    { 0x3107, 0x01 },
    { 0x3500, 0x00 },
    { 0x3501, 0x1c },
    { 0x3502, 0x20 },
    { 0x3503, 0x08 },
    { 0x3508, 0x04 },
    { 0x3509, 0x00 },
    { 0x3600, 0x66 },
    { 0x3602, 0x30 },
    { 0x3610, 0xa5 },
    { 0x3612, 0x93 },
    { 0x3620, 0x80 },
    { 0x3642, 0x0e },
    { 0x3661, 0x00 },
    { 0x3662, 0x10 },
    { 0x3664, 0xf3 },
    { 0x3665, 0x9e },
    { 0x3667, 0xa5 },
    { 0x366e, 0x55 },
    { 0x366f, 0x55 },
    { 0x3670, 0x11 },
    { 0x3671, 0x11 },
    { 0x3672, 0x11 },
    { 0x3673, 0x11 },
    { 0x3714, 0x24 },
    { 0x371a, 0x3e },
    { 0x3733, 0x10 },
    { 0x3734, 0x00 },
    { 0x373d, 0x24 },
    { 0x3764, 0x20 },
    { 0x3765, 0x20 },
    { 0x3766, 0x12 },
    { 0x37a1, 0x14 },
    { 0x37a8, 0x1c },
    { 0x37ab, 0x0f },
    { 0x37c2, 0x04 },
    { 0x37cb, 0x09 },
    { 0x37cc, 0x15 },
    { 0x37cd, 0x1f },
    { 0x37ce, 0x1f },
    { 0x37d8, 0x02 },
    { 0x37d9, 0x08 },
    { 0x37dc, 0x04 },
    { 0x3800, 0x00 },
    { 0x3801, 0x00 },
    { 0x3802, 0x00 },
    { 0x3803, 0x04 },
    { 0x3804, 0x0a },
    { 0x3805, 0x3f },
    { 0x3806, 0x07 },
    { 0x3807, 0xb3 },
    { 0x3808, 0x0a },
    //{ 0x3809, 0x20 },
    { 0x3809, 0x00 }, //2560
    { 0x380a, 0x07 },
    { 0x380b, 0x98 },
    { 0x380c, 0x05 },
    { 0x380d, 0xdc },
    { 0x380e, 0x07 },
    { 0x380f, 0xd0 },
    { 0x3811, 0x10 },
    { 0x3813, 0x0c },
    { 0x3814, 0x01 },
    { 0x3815, 0x01 },
    { 0x3816, 0x01 },
    { 0x3817, 0x01 },
    { 0x381e, 0x02 },
    { 0x3820, 0x88 },
    { 0x3821, 0x01 },
    { 0x3832, 0x48 },
    { 0x3c80, 0x08 },
    { 0x3c82, 0x00 },
    { 0x3c83, 0xb1 },
    { 0x3c8c, 0x10 },
    { 0x3c8d, 0x00 },
    { 0x3c90, 0x00 },
    { 0x3c91, 0x00 },
    { 0x3c92, 0x00 },
    { 0x3c93, 0x00 },
    { 0x3c94, 0x00 },
    { 0x3c95, 0x00 },
    { 0x3c96, 0x00 },
    { 0x3c97, 0x00 },
    { 0x3d8c, 0x71 },
    { 0x3d8d, 0xE7 },
    { 0x4001, 0xe0 },
    { 0x4003, 0x40 },
    { 0x4008, 0x02 },
    { 0x4009, 0x0d },
    { 0x400f, 0x80 },
    { 0x4013, 0x02 },
    { 0x4040, 0x00 },
    { 0x4041, 0x07 },
    { 0x404c, 0x50 },
    { 0x404e, 0x20 },
    { 0x4500, 0x06 },
    { 0x4503, 0x00 },
    { 0x450a, 0x04 },
    { 0x4809, 0x04 },
    { 0x480c, 0x12 },
    { 0x4819, 0x70 },
    { 0x4825, 0x32 },
    { 0x4826, 0x32 },
    { 0x482a, 0x06 },
    { 0x4833, 0x08 },
    { 0x4837, 0x0d },
    { 0x5000, 0x77 },
    { 0x5780, 0x3e },
    { 0x5781, 0x0f },
    { 0x5782, 0x44 },
    { 0x5783, 0x02 },
    { 0x5784, 0x01 },
    { 0x5785, 0x01 },
    { 0x5786, 0x00 },
    { 0x5787, 0x04 },
    { 0x5788, 0x02 },
    { 0x5789, 0x0f },
    { 0x578a, 0xfd },
    { 0x578b, 0xf5 },
    { 0x578c, 0xf5 },
    { 0x578d, 0x03 },
    { 0x578e, 0x08 },
    { 0x578f, 0x0c },
    { 0x5790, 0x08 },
    { 0x5791, 0x06 },
    { 0x5792, 0x00 },
    { 0x5793, 0x52 },
    { 0x5794, 0xa3 },
    { 0x5b00, 0x01 },
    { 0x5b01, 0x10 },
    { 0x5b02, 0x01 },
    { 0x5b03, 0xdb },
    { 0x5b05, 0x6c },
    { 0x5e10, 0xfc },
    { 0x0100, 0x01 },

    { 0x0000,  0x00 }
};

// 2592x1944
const regval16_t sp5506_Mipi_Raw10_5M_25fps[] =
{
    {0x0103,0x01},
    {0xffff,0xff},
    {0x0300,0x05},
    {0x0302,0x96},
    {0x0303,0x00},
    {0x030e,0x07},
    {0x3002,0x21},
    {0x3016,0x12},
    //{0x3106,0x19},
    {0x3106,0x19},
    {0x3107,0x23},
    {0x3501,0x20},
    {0x3503,0x0c},
    {0x3508,0x03},
    {0x3509,0x00},
    {0x3600,0x66},
    {0x3602,0x30},
    {0x3610,0xa5},
    {0x3612,0x93},
    {0x3620,0x80},
    {0x3642,0x0e},
    {0x3661,0x00},
    {0x3662,0x10},
    {0x3664,0xf3},
    {0x3665,0x9e},
    {0x3667,0xa5},
    {0x366e,0x55},
    {0x366f,0x55},
    {0x3670,0x11},
    {0x3671,0x11},
    {0x3672,0x11},
    {0x3673,0x11},
    {0x3714,0x24},
    {0x371a,0x3e},
    {0x3733,0x10},
    {0x3734,0x00},
    {0x373d,0x24},
    {0x3764,0x20},
    {0x3765,0x20},
    {0x3766,0x12},
    {0x37a1,0x14},
    {0x37a8,0x1c},
    {0x37ab,0x0f},
    {0x37c2,0x04},
    {0x37cb,0x09},
    {0x37cc,0x15},
    {0x37cd,0x1f},
    {0x37ce,0x1f},
    {0x37d8,0x02},
    {0x37d9,0x08},
    {0x37dc,0x04},
    {0x3800,0x00},
    {0x3801,0x00},
    {0x3802,0x00},
    {0x3803,0x04},
    {0x3804,0x0a},
    {0x3805,0x3f},
    {0x3806,0x07},
    {0x3807,0xb3},
    {0x3808,0x0a},
    {0x3809,0x00},
    {0x380a,0x07},
    {0x380b,0x98},
    {0x380c,0x05},
    {0x380d,0xdc},
    {0x380e,0x07},
    {0x380f,0xd0},
    {0x3811,0x10},
    {0x3813,0x0c},
    {0x3814,0x01},
    {0x3815,0x01},
    {0x3816,0x01},
    {0x3817,0x01},
    {0x381e,0x02},
    {0x3820,0x88},
    {0x3821,0x01},
    {0x3832,0x04},
    {0x3c80,0x01},
    {0x3c82,0x00},
    {0x3c83,0xc8},
    {0x3c8c,0x0f},
    {0x3c8d,0xa0},
    {0x3c90,0x07},
    {0x3c91,0x00},
    {0x3c92,0x00},
    {0x3c93,0x00},
    {0x3c94,0xd0},
    {0x3c95,0x50},
    {0x3c96,0x35},
    {0x3c97,0x00},
    {0x4001,0xe0},
    {0x4008,0x02},
    {0x4009,0x0d},
    {0x400f,0x80},


    {0x4013,0x02},
    {0x4040,0x00},
    {0x4041,0x07},
    {0x404c,0x50},
    {0x404e,0x20},
    {0x4500,0x06},
    {0x4503,0x00},
    {0x450a,0x04},
    {0x4809,0x04},
    {0x480c,0x12},
    {0x4819,0x70},
    {0x4825,0x32},
    {0x4826,0x32},
    {0x482a,0x06},
    {0x4833,0x08},
    {0x4837,0x0d},
    {0x5000,0x77},
    {0x5b00,0x01},
    {0x5b01,0x10},
    {0x5b02,0x01},
    {0x5b03,0xdb},
    {0x5b05,0x6c},
    {0x5e10,0xfc},
    {0x3500,0x00},
    {0x3501,0x3E},
    {0x3502,0x60},
    {0x3503,0x08},
    {0x3508,0x04},
    {0x3509,0x00},
    {0x3832,0x48},
    {0x3c90,0x00},
    {0x5780,0x3e},
    {0x5781,0x0f},
    {0x5782,0x44},
    {0x5783,0x02},
    {0x5784,0x01},
    {0x5785,0x01},
    {0x5786,0x00},
    {0x5787,0x04},
    {0x5788,0x02},
    {0x5789,0x0f},
    {0x578a,0xfd},
    {0x578b,0xf5},
    {0x578c,0xf5},
    {0x578d,0x03},
    {0x578e,0x08},
    {0x578f,0x0c},
    {0x5790,0x08},
    {0x5791,0x06},
    {0x5792,0x00},
    {0x5793,0x52},
    {0x5794,0xa3},
    {0x4003,0x40},
    {0x3107,0x01},
    {0x3c80,0x08},
    {0x3c83,0xb1},
    {0x3c8c,0x10},
    {0x3c8d,0x00},
    {0x3c90,0x00},
    {0x3c94,0x00},
    {0x3c95,0x00},
    {0x3c96,0x00},
    {0x3d8c,0x71},
    {0x3d8d,0xE7},
 //   {0x4503,0x80},//Enable Test Pattern
    {0x0100,  0x00},
     {0xFFFF,  0xFF},
    {0x0100,0x01},

    {0x0000,  0x00}
};

//234 ~ 256
static const  int sp5506_mipi_25fps_exp_time_gain_60Hz[SP5506_25FPS_60HZ_EXP_TIME_TOTAL][3] =
{
{4, (int)(1.00*256), (int)(1.00*256)},
{5, (int)(1.00*256), (int)(1.00*256)},
{5, (int)(1.00*256), (int)(1.00*256)},
{5, (int)(1.00*256), (int)(1.00*256)},
{5, (int)(1.00*256), (int)(1.00*256)},
{5, (int)(1.00*256), (int)(1.00*256)},
{5, (int)(1.00*256), (int)(1.00*256)},
{6, (int)(1.00*256), (int)(1.00*256)},
{6, (int)(1.00*256), (int)(1.00*256)},
{6, (int)(1.00*256), (int)(1.00*256)},
{6, (int)(1.00*256), (int)(1.00*256)},
{6, (int)(1.00*256), (int)(1.00*256)},
{7, (int)(1.00*256), (int)(1.00*256)},
{7, (int)(1.00*256), (int)(1.00*256)},
{7, (int)(1.00*256), (int)(1.00*256)},
{7, (int)(1.00*256), (int)(1.00*256)},
{8, (int)(1.00*256), (int)(1.00*256)},
{8, (int)(1.00*256), (int)(1.00*256)},
{8, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
{12, (int)(1.00*256), (int)(1.00*256)},
{12, (int)(1.00*256), (int)(1.00*256)},
{12, (int)(1.00*256), (int)(1.00*256)},
{13, (int)(1.00*256), (int)(1.00*256)},
{13, (int)(1.00*256), (int)(1.00*256)},
{14, (int)(1.00*256), (int)(1.00*256)},
{14, (int)(1.00*256), (int)(1.00*256)},
{15, (int)(1.00*256), (int)(1.00*256)},
{15, (int)(1.00*256), (int)(1.00*256)},
{16, (int)(1.00*256), (int)(1.00*256)},
{16, (int)(1.00*256), (int)(1.00*256)},
{17, (int)(1.00*256), (int)(1.00*256)},
{18, (int)(1.00*256), (int)(1.00*256)},
{18, (int)(1.00*256), (int)(1.00*256)},
{19, (int)(1.00*256), (int)(1.00*256)},
{20, (int)(1.00*256), (int)(1.00*256)},
{20, (int)(1.00*256), (int)(1.00*256)},
{21, (int)(1.00*256), (int)(1.00*256)},
{22, (int)(1.00*256), (int)(1.00*256)},
{22, (int)(1.00*256), (int)(1.00*256)},
{23, (int)(1.00*256), (int)(1.00*256)},
{24, (int)(1.00*256), (int)(1.00*256)},
{25, (int)(1.00*256), (int)(1.00*256)},
{26, (int)(1.00*256), (int)(1.00*256)},
{27, (int)(1.00*256), (int)(1.00*256)},
{28, (int)(1.00*256), (int)(1.00*256)},
{29, (int)(1.00*256), (int)(1.00*256)},
{30, (int)(1.00*256), (int)(1.00*256)},
{31, (int)(1.00*256), (int)(1.00*256)},
{32, (int)(1.00*256), (int)(1.00*256)},
{33, (int)(1.00*256), (int)(1.00*256)},
{34, (int)(1.00*256), (int)(1.00*256)},
{35, (int)(1.00*256), (int)(1.00*256)},
{37, (int)(1.00*256), (int)(1.00*256)},
{38, (int)(1.00*256), (int)(1.00*256)},
{39, (int)(1.00*256), (int)(1.00*256)},
{41, (int)(1.00*256), (int)(1.00*256)},
{42, (int)(1.00*256), (int)(1.00*256)},
{43, (int)(1.00*256), (int)(1.00*256)},
{45, (int)(1.00*256), (int)(1.00*256)},
{47, (int)(1.00*256), (int)(1.00*256)},
{48, (int)(1.00*256), (int)(1.00*256)},
{50, (int)(1.00*256), (int)(1.00*256)},
{52, (int)(1.00*256), (int)(1.00*256)},
{53, (int)(1.00*256), (int)(1.00*256)},
{55, (int)(1.00*256), (int)(1.00*256)},
{57, (int)(1.00*256), (int)(1.00*256)},
{59, (int)(1.00*256), (int)(1.00*256)},
{61, (int)(1.00*256), (int)(1.00*256)},
{64, (int)(1.00*256), (int)(1.00*256)},
{66, (int)(1.00*256), (int)(1.00*256)},
{68, (int)(1.00*256), (int)(1.00*256)},
{71, (int)(1.00*256), (int)(1.00*256)},
{73, (int)(1.00*256), (int)(1.00*256)},
{76, (int)(1.00*256), (int)(1.00*256)},
{78, (int)(1.00*256), (int)(1.00*256)},
{81, (int)(1.00*256), (int)(1.00*256)},
{84, (int)(1.00*256), (int)(1.00*256)},
{87, (int)(1.00*256), (int)(1.00*256)},
{90, (int)(1.00*256), (int)(1.00*256)},
{93, (int)(1.00*256), (int)(1.00*256)},
{96, (int)(1.00*256), (int)(1.00*256)},
{100, (int)(1.00*256), (int)(1.00*256)},
{103, (int)(1.00*256), (int)(1.00*256)},
{107, (int)(1.00*256), (int)(1.00*256)},
{111, (int)(1.00*256), (int)(1.00*256)},
{115, (int)(1.00*256), (int)(1.00*256)},
{119, (int)(1.00*256), (int)(1.00*256)},
{123, (int)(1.00*256), (int)(1.00*256)},
{127, (int)(1.00*256), (int)(1.00*256)},
{132, (int)(1.00*256), (int)(1.00*256)},
{136, (int)(1.00*256), (int)(1.00*256)},
{141, (int)(1.00*256), (int)(1.00*256)},
{146, (int)(1.00*256), (int)(1.00*256)},
{151, (int)(1.00*256), (int)(1.00*256)},
{156, (int)(1.00*256), (int)(1.00*256)},
{162, (int)(1.00*256), (int)(1.00*256)},
{168, (int)(1.00*256), (int)(1.00*256)},
{174, (int)(1.00*256), (int)(1.00*256)},
{180, (int)(1.00*256), (int)(1.00*256)},
{186, (int)(1.00*256), (int)(1.00*256)},
{193, (int)(1.00*256), (int)(1.00*256)},
{199, (int)(1.00*256), (int)(1.00*256)},
{207, (int)(1.00*256), (int)(1.00*256)},
{214, (int)(1.00*256), (int)(1.00*256)},
{221, (int)(1.00*256), (int)(1.00*256)},
{229, (int)(1.00*256), (int)(1.00*256)},
{237, (int)(1.00*256), (int)(1.00*256)},
{246, (int)(1.00*256), (int)(1.00*256)},
{254, (int)(1.00*256), (int)(1.00*256)},
{263, (int)(1.00*256), (int)(1.00*256)},
{272, (int)(1.00*256), (int)(1.00*256)},
{282, (int)(1.00*256), (int)(1.00*256)},
{292, (int)(1.00*256), (int)(1.00*256)},
{302, (int)(1.00*256), (int)(1.00*256)},
{313, (int)(1.00*256), (int)(1.00*256)},
{324, (int)(1.00*256), (int)(1.00*256)},
{335, (int)(1.00*256), (int)(1.00*256)},
{347, (int)(1.00*256), (int)(1.00*256)},
{360, (int)(1.00*256), (int)(1.00*256)},
{372, (int)(1.00*256), (int)(1.00*256)},
{385, (int)(1.00*256), (int)(1.00*256)},
{399, (int)(1.00*256), (int)(1.00*256)},
{413, (int)(1.00*256), (int)(1.00*256)},
{413, (int)(1.0625*256), (int)(1.00*256)},
{413, (int)(1.0625*256), (int)(1.00*256)},
{413, (int)(1.125*256), (int)(1.00*256)},
{413, (int)(1.125*256), (int)(1.00*256)},
{413, (int)(1.1875*256), (int)(1.00*256)},
{413, (int)(1.25*256), (int)(1.00*256)},
{413, (int)(1.25*256), (int)(1.00*256)},
{413, (int)(1.3125*256), (int)(1.00*256)},
{413, (int)(1.375*256), (int)(1.00*256)},
{413, (int)(1.4375*256), (int)(1.00*256)},
{413, (int)(1.4375*256), (int)(1.00*256)},
{413, (int)(1.5*256), (int)(1.00*256)},
{413, (int)(1.5625*256), (int)(1.00*256)},
{413, (int)(1.625*256), (int)(1.00*256)},
{413, (int)(1.6875*256), (int)(1.00*256)},
{413, (int)(1.75*256), (int)(1.00*256)},
{413, (int)(1.8125*256), (int)(1.00*256)},
{413, (int)(1.875*256), (int)(1.00*256)},
{413, (int)(1.9375*256), (int)(1.00*256)},
{827, (int)(1.00*256), (int)(1.00*256)},
{827, (int)(1.0625*256), (int)(1.00*256)},
{827, (int)(1.0625*256), (int)(1.00*256)},
{827, (int)(1.125*256), (int)(1.00*256)},
{827, (int)(1.125*256), (int)(1.00*256)},
{827, (int)(1.1875*256), (int)(1.00*256)},
{827, (int)(1.25*256), (int)(1.00*256)},
{827, (int)(1.25*256), (int)(1.00*256)},
{827, (int)(1.3125*256), (int)(1.00*256)},
{827, (int)(1.375*256), (int)(1.00*256)},
{827, (int)(1.4375*256), (int)(1.00*256)},
{827, (int)(1.4375*256), (int)(1.00*256)},
{827, (int)(1.5*256), (int)(1.00*256)},
{827, (int)(1.5625*256), (int)(1.00*256)},
{827, (int)(1.625*256), (int)(1.00*256)},
{827, (int)(1.6875*256), (int)(1.00*256)},
{827, (int)(1.75*256), (int)(1.00*256)},
{827, (int)(1.8125*256), (int)(1.00*256)},
{827, (int)(1.875*256), (int)(1.00*256)},
{827, (int)(1.9375*256), (int)(1.00*256)},
{1240, (int)(1.3125*256), (int)(1.00*256)},
{1240, (int)(1.375*256), (int)(1.00*256)},
{1240, (int)(1.4375*256), (int)(1.00*256)},
{1240, (int)(1.4375*256), (int)(1.00*256)},
{1240, (int)(1.5*256), (int)(1.00*256)},
{1240, (int)(1.5625*256), (int)(1.00*256)},
{1240, (int)(1.625*256), (int)(1.00*256)},
{1240, (int)(1.6875*256), (int)(1.00*256)},
{1240, (int)(1.75*256), (int)(1.00*256)},
{1240, (int)(1.8125*256), (int)(1.00*256)},
{1240, (int)(1.875*256), (int)(1.00*256)},
{1240, (int)(1.9375*256), (int)(1.00*256)},
{1653, (int)(1.5*256), (int)(1.00*256)},
{1653, (int)(1.5625*256), (int)(1.00*256)},
{1653, (int)(1.625*256), (int)(1.00*256)},
{1653, (int)(1.6875*256), (int)(1.00*256)},
{1653, (int)(1.75*256), (int)(1.00*256)},
{1653, (int)(1.8125*256), (int)(1.00*256)},
{1653, (int)(1.875*256), (int)(1.00*256)},
{1653, (int)(1.9375*256), (int)(1.00*256)},
{1653, (int)(2.0*256), (int)(1.00*256)},
{1653, (int)(2.07*256), (int)(1.00*256)},
{1653, (int)(2.14*256), (int)(1.00*256)},
{1653, (int)(2.22*256), (int)(1.00*256)},
{1653, (int)(2.30*256), (int)(1.00*256)},
{1653, (int)(2.38*256), (int)(1.00*256)},
{1653, (int)(2.46*256), (int)(1.00*256)},
{1653, (int)(2.55*256), (int)(1.00*256)},
{1653, (int)(2.64*256), (int)(1.00*256)},
{1653, (int)(2.73*256), (int)(1.00*256)},
{1653, (int)(2.83*256), (int)(1.00*256)},
{1653, (int)(2.93*256), (int)(1.00*256)},
{1653, (int)(3.03*256), (int)(1.00*256)},
{1653, (int)(3.14*256), (int)(1.00*256)},
{1653, (int)(3.25*256), (int)(1.00*256)},
{1653, (int)(3.36*256), (int)(1.00*256)},
{1653, (int)(3.48*256), (int)(1.00*256)},
{1653, (int)(3.61*256), (int)(1.00*256)},
{1653, (int)(3.73*256), (int)(1.00*256)},
{1653, (int)(3.86*256), (int)(1.00*256)},
{1653, (int)(4.00*256), (int)(1.00*256)},
{1653, (int)(4.14*256), (int)(1.00*256)},
{1653, (int)(4.29*256), (int)(1.00*256)},
{1653, (int)(4.44*256), (int)(1.00*256)},
{1653, (int)(4.59*256), (int)(1.00*256)},
{1653, (int)(4.76*256), (int)(1.00*256)},
{1653, (int)(4.92*256), (int)(1.00*256)},
{1653, (int)(5.10*256), (int)(1.00*256)},
{1653, (int)(5.28*256), (int)(1.00*256)},
{1653, (int)(5.46*256), (int)(1.00*256)},
{1653, (int)(5.66*256), (int)(1.00*256)},
{1653, (int)(5.86*256), (int)(1.00*256)},
{1653, (int)(6.06*256), (int)(1.00*256)},
{1653, (int)(6.28*256), (int)(1.00*256)},
{1653, (int)(6.50*256), (int)(1.00*256)},
{1653, (int)(6.73*256), (int)(1.00*256)},
{1653, (int)(6.96*256), (int)(1.00*256)},
{1653, (int)(7.21*256), (int)(1.00*256)},
{1653, (int)(7.46*256), (int)(1.00*256)},
{1653, (int)(7.73*256), (int)(1.00*256)},
{1653, (int)(8.00*256), (int)(1.00*256)},
{1653, (int)(8.28*256), (int)(1.00*256)},
{1653, (int)(8.57*256), (int)(1.00*256)},
{1653, (int)(8.88*256), (int)(1.00*256)},
{1653, (int)(9.19*256), (int)(1.00*256)},
{1653, (int)(9.51*256), (int)(1.00*256)},
{1653, (int)(9.85*256), (int)(1.00*256)},
{1653, (int)(10.20*256), (int)(1.00*256)},
{1653, (int)(10.56*256), (int)(1.00*256)},
{1653, (int)(10.93*256), (int)(1.00*256)},
{1653, (int)(11.31*256), (int)(1.00*256)},
{1653, (int)(11.71*256), (int)(1.00*256)},
{1653, (int)(12.13*256), (int)(1.00*256)},
{1653, (int)(12.55*256), (int)(1.00*256)},
{1653, (int)(13.00*256), (int)(1.00*256)},
{1653, (int)(13.45*256), (int)(1.00*256)},
{1653, (int)(13.93*256), (int)(1.00*256)},
{1653, (int)(14.42*256), (int)(1.00*256)},
{1653, (int)(14.93*256), (int)(1.00*256)},
{1653, (int)(15.45*256), (int)(1.00*256)},
{1653, (int)(16.00*256), (int)(1.00*256)}

};


static const  int sp5506_mipi_25fps_exp_time_gain_50Hz[SP5506_25FPS_50HZ_EXP_TIME_TOTAL][3] =
{
{12, (int)(1.00*256), (int)(1.00*32)},
{13, (int)(1.00*256), (int)(1.00*32)},
{13, (int)(1.00*256), (int)(1.00*32)},
{14, (int)(1.00*256), (int)(1.00*32)},
{14, (int)(1.00*256), (int)(1.00*32)},
{15, (int)(1.00*256), (int)(1.00*32)},
{15, (int)(1.00*256), (int)(1.00*32)},
{16, (int)(1.00*256), (int)(1.00*32)},
{16, (int)(1.00*256), (int)(1.00*32)},
{17, (int)(1.00*256), (int)(1.00*32)},
{17, (int)(1.00*256), (int)(1.00*32)},
{18, (int)(1.00*256), (int)(1.00*32)},
{19, (int)(1.00*256), (int)(1.00*32)},
{19, (int)(1.00*256), (int)(1.00*32)},
{20, (int)(1.00*256), (int)(1.00*32)},
{21, (int)(1.00*256), (int)(1.00*32)},
{21, (int)(1.00*256), (int)(1.00*32)},
{22, (int)(1.00*256), (int)(1.00*32)},
{23, (int)(1.00*256), (int)(1.00*32)},
{24, (int)(1.00*256), (int)(1.00*32)},
{25, (int)(1.00*256), (int)(1.00*32)},
{26, (int)(1.00*256), (int)(1.00*32)},
{26, (int)(1.00*256), (int)(1.00*32)},
{27, (int)(1.00*256), (int)(1.00*32)},
{28, (int)(1.00*256), (int)(1.00*32)},
{29, (int)(1.00*256), (int)(1.00*32)},
{30, (int)(1.00*256), (int)(1.00*32)},
{31, (int)(1.00*256), (int)(1.00*32)},
{33, (int)(1.00*256), (int)(1.00*32)},
{34, (int)(1.00*256), (int)(1.00*32)},
{35, (int)(1.00*256), (int)(1.00*32)},
{36, (int)(1.00*256), (int)(1.00*32)},
{37, (int)(1.00*256), (int)(1.00*32)},
{39, (int)(1.00*256), (int)(1.00*32)},
{40, (int)(1.00*256), (int)(1.00*32)},
{41, (int)(1.00*256), (int)(1.00*32)},
{43, (int)(1.00*256), (int)(1.00*32)},
{44, (int)(1.00*256), (int)(1.00*32)},
{46, (int)(1.00*256), (int)(1.00*32)},
{48, (int)(1.00*256), (int)(1.00*32)},
{49, (int)(1.00*256), (int)(1.00*32)},
{51, (int)(1.00*256), (int)(1.00*32)},
{53, (int)(1.00*256), (int)(1.00*32)},
{55, (int)(1.00*256), (int)(1.00*32)},
{57, (int)(1.00*256), (int)(1.00*32)},
{59, (int)(1.00*256), (int)(1.00*32)},
{61, (int)(1.00*256), (int)(1.00*32)},
{63, (int)(1.00*256), (int)(1.00*32)},
{65, (int)(1.00*256), (int)(1.00*32)},
{67, (int)(1.00*256), (int)(1.00*32)},
{70, (int)(1.00*256), (int)(1.00*32)},
{72, (int)(1.00*256), (int)(1.00*32)},
{75, (int)(1.00*256), (int)(1.00*32)},
{77, (int)(1.00*256), (int)(1.00*32)},
{80, (int)(1.00*256), (int)(1.00*32)},
{83, (int)(1.00*256), (int)(1.00*32)},
{86, (int)(1.00*256), (int)(1.00*32)},
{89, (int)(1.00*256), (int)(1.00*32)},
{92, (int)(1.00*256), (int)(1.00*32)},
{95, (int)(1.00*256), (int)(1.00*32)},
{99, (int)(1.00*256), (int)(1.00*32)},
{102, (int)(1.00*256), (int)(1.00*32)},
{106, (int)(1.00*256), (int)(1.00*32)},
{109, (int)(1.00*256), (int)(1.00*32)},
{113, (int)(1.00*256), (int)(1.00*32)},
{117, (int)(1.00*256), (int)(1.00*32)},
{121, (int)(1.00*256), (int)(1.00*32)},
{126, (int)(1.00*256), (int)(1.00*32)},
{130, (int)(1.00*256), (int)(1.00*32)},
{135, (int)(1.00*256), (int)(1.00*32)},
{140, (int)(1.00*256), (int)(1.00*32)},
{144, (int)(1.00*256), (int)(1.00*32)},
{150, (int)(1.00*256), (int)(1.00*32)},
{155, (int)(1.00*256), (int)(1.00*32)},
{160, (int)(1.00*256), (int)(1.00*32)},
{166, (int)(1.00*256), (int)(1.00*32)},
{172, (int)(1.00*256), (int)(1.00*32)},
{178, (int)(1.00*256), (int)(1.00*32)},
{184, (int)(1.00*256), (int)(1.00*32)},
{191, (int)(1.00*256), (int)(1.00*32)},
{197, (int)(1.00*256), (int)(1.00*32)},
{204, (int)(1.00*256), (int)(1.00*32)},
{211, (int)(1.00*256), (int)(1.00*32)},
{219, (int)(1.00*256), (int)(1.00*32)},
{227, (int)(1.00*256), (int)(1.00*32)},
{235, (int)(1.00*256), (int)(1.00*32)},
{243, (int)(1.00*256), (int)(1.00*32)},
{251, (int)(1.00*256), (int)(1.00*32)},
{260, (int)(1.00*256), (int)(1.00*32)},
{270, (int)(1.00*256), (int)(1.00*32)},
{279, (int)(1.00*256), (int)(1.00*32)},
{289, (int)(1.00*256), (int)(1.00*32)},
{299, (int)(1.00*256), (int)(1.00*32)},
{310, (int)(1.00*256), (int)(1.00*32)},
{321, (int)(1.00*256), (int)(1.00*32)},
{332, (int)(1.00*256), (int)(1.00*32)},
{344, (int)(1.00*256), (int)(1.00*32)},
{356, (int)(1.00*256), (int)(1.00*32)},
{368, (int)(1.00*256), (int)(1.00*32)},
{381, (int)(1.00*256), (int)(1.00*32)},
{395, (int)(1.00*256), (int)(1.00*32)},
{409, (int)(1.00*256), (int)(1.00*32)},
{423, (int)(1.00*256), (int)(1.00*32)},
{438, (int)(1.00*256), (int)(1.00*32)},
{453, (int)(1.00*256), (int)(1.00*32)},
{469, (int)(1.00*256), (int)(1.00*32)},
{486, (int)(1.00*256), (int)(1.00*32)},
{503, (int)(1.00*256), (int)(1.00*32)},
{503, (int)(1.0625*256), (int)(1.00*32)},
{503, (int)(1.0625*256), (int)(1.00*32)},
{503, (int)(1.125*256), (int)(1.00*32)},
{503, (int)(1.125*256), (int)(1.00*32)},
{503, (int)(1.1875*256), (int)(1.00*32)},
{503, (int)(1.25*256), (int)(1.00*32)},
{503, (int)(1.25*256), (int)(1.00*32)},
{503, (int)(1.3125*256), (int)(1.00*32)},
{503, (int)(1.375*256), (int)(1.00*32)},
{503, (int)(1.4375*256), (int)(1.00*32)},
{503, (int)(1.4375*256), (int)(1.00*32)},
{503, (int)(1.5*256), (int)(1.00*32)},
{503, (int)(1.5625*256), (int)(1.00*32)},
{503, (int)(1.625*256), (int)(1.00*32)},
{503, (int)(1.6875*256), (int)(1.00*32)},
{503, (int)(1.75*256), (int)(1.00*32)},
{503, (int)(1.8125*256), (int)(1.00*32)},
{503, (int)(1.875*256), (int)(1.00*32)},
{503, (int)(1.9375*256), (int)(1.00*32)},
{1006, (int)(1.00*256), (int)(1.00*32)},
{1006, (int)(1.0625*256), (int)(1.00*32)},
{1006, (int)(1.0625*256), (int)(1.00*32)},
{1006, (int)(1.125*256), (int)(1.00*32)},
{1006, (int)(1.125*256), (int)(1.00*32)},
{1006, (int)(1.1875*256), (int)(1.00*32)},
{1006, (int)(1.25*256), (int)(1.00*32)},
{1006, (int)(1.25*256), (int)(1.00*32)},
{1006, (int)(1.3125*256), (int)(1.00*32)},
{1006, (int)(1.375*256), (int)(1.00*32)},
{1006, (int)(1.4375*256), (int)(1.00*32)},
{1006, (int)(1.4375*256), (int)(1.00*32)},
{1006, (int)(1.5*256), (int)(1.00*32)},
{1006, (int)(1.5625*256), (int)(1.00*32)},
{1006, (int)(1.625*256), (int)(1.00*32)},
{1006, (int)(1.6875*256), (int)(1.00*32)},
{1006, (int)(1.75*256), (int)(1.00*32)},
{1006, (int)(1.8125*256), (int)(1.00*32)},
{1006, (int)(1.875*256), (int)(1.00*32)},
{1006, (int)(1.9375*256), (int)(1.00*32)},
{1509, (int)(1.3125*256), (int)(1.00*32)},
{1509, (int)(1.375*256), (int)(1.00*32)},
{1509, (int)(1.4375*256), (int)(1.00*32)},
{1509, (int)(1.4375*256), (int)(1.00*32)},
{1509, (int)(1.5*256), (int)(1.00*32)},
{1509, (int)(1.5625*256), (int)(1.00*32)},
{1509, (int)(1.625*256), (int)(1.00*32)},
{1509, (int)(1.6875*256), (int)(1.00*32)},
{1509, (int)(1.75*256), (int)(1.00*32)},
{1509, (int)(1.8125*256), (int)(1.00*32)},
{1509, (int)(1.875*256), (int)(1.00*32)},
{1509, (int)(1.9375*256), (int)(1.00*32)},
{2012, (int)(1.5*256), (int)(1.00*32)},
{2012, (int)(1.5625*256), (int)(1.00*32)},
{2012, (int)(1.625*256), (int)(1.00*32)},
{2012, (int)(1.6875*256), (int)(1.00*32)},
{2012, (int)(1.75*256), (int)(1.00*32)},
{2012, (int)(1.8125*256), (int)(1.00*32)},
{2012, (int)(1.875*256), (int)(1.00*32)},
{2012, (int)(1.9375*256), (int)(1.00*32)},
{2012, (int)(2.0*256), (int)(1.00*32)},
{2012, (int)(2.07*256), (int)(1.00*32)},
{2012, (int)(2.14*256), (int)(1.00*32)},
{2012, (int)(2.22*256), (int)(1.00*32)},
{2012, (int)(2.30*256), (int)(1.00*32)},
{2012, (int)(2.38*256), (int)(1.00*32)},
{2012, (int)(2.46*256), (int)(1.00*32)},
{2012, (int)(2.55*256), (int)(1.00*32)},
{2012, (int)(2.64*256), (int)(1.00*32)},
{2012, (int)(2.73*256), (int)(1.00*32)},
{2012, (int)(2.83*256), (int)(1.00*32)},
{2012, (int)(2.93*256), (int)(1.00*32)},
{2012, (int)(3.03*256), (int)(1.00*32)},
{2012, (int)(3.14*256), (int)(1.00*32)},
{2012, (int)(3.25*256), (int)(1.00*32)},
{2012, (int)(3.36*256), (int)(1.00*32)},
{2012, (int)(3.48*256), (int)(1.00*32)},
{2012, (int)(3.61*256), (int)(1.00*32)},
{2012, (int)(3.73*256), (int)(1.00*32)},
{2012, (int)(3.86*256), (int)(1.00*32)},
{2012, (int)(4.00*256), (int)(1.00*32)},
{2012, (int)(4.14*256), (int)(1.00*32)},
{2012, (int)(4.29*256), (int)(1.00*32)},
{2012, (int)(4.44*256), (int)(1.00*32)},
{2012, (int)(4.59*256), (int)(1.00*32)},
{2012, (int)(4.76*256), (int)(1.00*32)},
{2012, (int)(4.92*256), (int)(1.00*32)},
{2012, (int)(5.10*256), (int)(1.00*32)},
{2012, (int)(5.28*256), (int)(1.00*32)},
{2012, (int)(5.46*256), (int)(1.00*32)},
{2012, (int)(5.66*256), (int)(1.00*32)},
{2012, (int)(5.86*256), (int)(1.00*32)},
{2012, (int)(6.06*256), (int)(1.00*32)},
{2012, (int)(6.28*256), (int)(1.00*32)},
{2012, (int)(6.50*256), (int)(1.00*32)},
{2012, (int)(6.73*256), (int)(1.00*32)},
{2012, (int)(6.96*256), (int)(1.00*32)},
{2012, (int)(7.21*256), (int)(1.00*32)},
{2012, (int)(7.46*256), (int)(1.00*32)},
{2012, (int)(7.73*256), (int)(1.00*32)},
{2012, (int)(8.00*256), (int)(1.00*32)},
{2012, (int)(8.28*256), (int)(1.00*32)},
{2012, (int)(8.57*256), (int)(1.00*32)},
{2012, (int)(8.88*256), (int)(1.00*32)},
{2012, (int)(9.19*256), (int)(1.00*32)},
{2012, (int)(9.51*256), (int)(1.00*32)},
{2012, (int)(9.85*256), (int)(1.00*32)},
/*{2012, (int)(10.20*256), (int)(1.00*32)},
{2012, (int)(10.56*256), (int)(1.00*32)},
{2012, (int)(10.93*256), (int)(1.00*32)},
{2012, (int)(11.31*256), (int)(1.00*32)},
{2012, (int)(11.71*256), (int)(1.00*32)},
{2012, (int)(12.13*256), (int)(1.00*32)},
{2012, (int)(12.55*256), (int)(1.00*32)},
{2012, (int)(13.00*256), (int)(1.00*32)},
{2012, (int)(13.45*256), (int)(1.00*32)},
{2012, (int)(13.93*256), (int)(1.00*32)},
{2012, (int)(14.42*256), (int)(1.00*32)},
{2012, (int)(14.93*256), (int)(1.00*32)},
{2012, (int)(15.45*256), (int)(1.00*32)},
{2012, (int)(16.00*256), (int)(1.00*32)},*/
};




////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S sp5506_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	sp5506_handle = drv_l2_sccb_open(SP5506_ID, 16, 8);
	if(sp5506_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
    sp5506_handle.devNumber = I2C_0;
	sp5506_handle.slaveAddr = sp5506_ID;
	sp5506_handle.clkRate = 400;

	//R_FUNPOS1 &= (~MASK_I2C0_TYPE);

//	gpio_init_io(IO_C1, GPIO_OUTPUT);
//	gpio_set_port_attribute(IO_C1, ATTRIBUTE_HIGH);
//	gpio_write_io(IO_C1, DATA_HIGH);

	drv_l1_i2c_init(sp5506_handle.devNumber);
#endif

    DBG_PRINT("I2C open successful\r\n");

	return STATUS_OK;
}

static void sp5506_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(sp5506_handle) {
		drv_l2_sccb_close(sp5506_handle);
		sp5506_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	sp5506_handle.slaveAddr = 0;
	sp5506_handle.clkRate = 0;
#endif
}

static INT32S sp5506_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(sp5506_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&sp5506_handle, data, 3);
#endif
}



static INT32S sp5506_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(sp5506_handle, reg, &data) >= 0) {
		*value = (INT8U)data;
		//DBG_PRINT("Read 0x%04x, value 0x%x\r\n", reg, *value);
		return STATUS_OK;
	} else {
		*value = 0xFF;
		DBG_PRINT("i2C read fail!\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[2];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	if(drv_l1_i2c_bus_write(&sp5506_handle, &data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&sp5506_handle, &data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif

	return STATUS_OK;
}


static INT32S sp5506_sccb_write_table(regval16_t *pTable)
{
	INT8U value;
	INT16U cnt= 0;
	while(1)
	{
		if (pTable->reg_num == 0 && pTable->value == 0) {
			break;
		}
		else if (pTable->reg_num == 0xffff && pTable->value == 0xff) {
			osDelay(5);
			pTable++;
			continue;
		}

		//DBG_PRINT("0x%04x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(sp5506_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
//	    sp5506_sccb_read(pTable->reg_num, &value);
//    	DBG_PRINT("sp5506 reg addr 0x%x, data 0x%x \r\n", pTable->reg_num, value);

		pTable++;
		cnt++;
		if (cnt == 210){
		DBG_PRINT("sp5506 initial step %d \r\n", cnt);
		}
	}
	return STATUS_OK;
}




static int sp5506_cvt_agc_gain(int agc_gain)
{
	int sp5506_agc_gain, i;

	sp5506_agc_gain = 0;
	i = 5;
	do {
		if(agc_gain <= 0x1f)
			break;

		agc_gain >>= 1;
		sp5506_agc_gain <<= 1;
		sp5506_agc_gain |= 0x10;

		i--;
	} while(i != 0);

	agc_gain -= 0x10;
	if(agc_gain < 0) agc_gain = 0;
	sp5506_agc_gain += agc_gain;

	return sp5506_agc_gain;
}

extern INT32U g_flag_update;
static unsigned char group_id = 0;
static unsigned char cnt = 0;
static int sp5506_set_xfps_exposure_time(int ae_ev_step)
{
	unsigned char t1, t2;
	int idx, temp, ret;
	int shutter;
	int current_group;
	sensor_exposure_t *si;


    if(g_flag_update!=0) {
        DBG_PRINT("%s: not set exposure time\n\r", __FUNCTION__);
        return 0;
    }

    si = &seInfo;
	//DBG_PRINT("%s\n\r", __FUNCTION__);
	#if 1
	si->sensor_ev_idx += ae_ev_step;
	if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
	if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;

	//si->sensor_ev_idx = 10;
#if RAW_MODE
	idx = (g_raw_mode_idx < 0) ? si->sensor_ev_idx * 3 : g_raw_mode_idx * 3;
	printk("%s:%d %d\n", __FUNCTION__, __LINE__, g_raw_mode_idx);
#else
	idx = si->sensor_ev_idx * 3;
#endif
	si ->time = p_expTime_table[idx];
	si ->analog_gain = p_expTime_table[idx+1];
    //DBG_PRINT("[EV=%d, offset=%d]: time = 0x%x, analog gain =0x%x\n\r", si->sensor_ev_idx, ae_ev_step, si->time, si->analog_gain);
/*
    current_group = group_id;
    group_id++;
    group_id = group_id & 0x03;
    ret = sp5506_sccb_write(0x3208, current_group); // Group hold enable
	if(ret < 0) return ret;
*/

cnt++;
	// exposure time
	if(si ->time != pre_sensor_time)
	//if( cnt < 3)
	{
        DBG_PRINT("time!!%d\r\n", si->time);

      // sp5506_sccb_write(0x380e, ((si->time+0x27ff) >> 8)& 0xFF);
      // sp5506_sccb_write(0x380f, (si->time+0x27ff) & 0xFF);

     //  sp5506_sccb_write(0x380e, ((si->time+4) >> 8)& 0xFF);
      // sp5506_sccb_write(0x380f, (si->time+4) & 0xFF);

        //sp5506_sccb_write(0x380e, (0xfff >> 8)& 0xFF);
        //sp5506_sccb_write(0x380f, 0xfff & 0xFF);

        //sp5506_sccb_write(0x380e, (0xfff >> 8)& 0xFF);
        //sp5506_sccb_write(0x380f, 0xfff & 0xFF);

		pre_sensor_time = si->time;

		if(si->time > 2000){
            si->time = 2000;
		}

		shutter = si->time>>1;
        shutter = shutter&0xffff;
        if(shutter < 4) shutter = 4;
DBG_PRINT("set time %d\r\n", shutter);
		temp = (shutter & 0x0f);
		temp = temp << 4;
       // t2 = (temp >> 4) & 0x00ff;
        ret = sp5506_sccb_write(0x3502, temp);
       if(ret < 0) {
        DBG_PRINT("ret error!!%d\r\n", __LINE__);
        return ret;
       }

		temp = (shutter & 0xfff);
		temp = temp >> 4;
        ret = sp5506_sccb_write(0x3501, temp);
       if(ret < 0) {
        DBG_PRINT("ret error!!%d\r\n", __LINE__);
        return ret;
       }

        temp =  (shutter >> 12) & 0x000f;
        ret = sp5506_sccb_write(0x3500, temp);
       if(ret < 0) {
        DBG_PRINT("ret error!!%d\r\n", __LINE__);
        return ret;
       }
	}

	if(si ->analog_gain != pre_sensor_a_gain)
	//if(0)
	//if( cnt < 3)
	{	// gain
		pre_sensor_a_gain = si->analog_gain;
       // cnt = 0;
		temp = si->analog_gain>>1;
//		temp = sp5506_cvt_agc_gain(temp);
		DBG_PRINT(", cvt a gain = %d\r\n", temp);
		temp = temp&0x1fff;
		if(temp < 0x80)
		{
			temp = 0x80;
		}
		if(temp > 0x7c0)
		{
			temp = 0x7c0;
		}
		//t1 = temp & 0x00ff;
        //t2 = (temp >> 8) & 0x0f;
//m
        // Gain = (0x350A[0]+1) กั (0x350B[7]+1) กั(0x350B[6]+1) กั (0x350B[5]+1) กั(0x350B[4]+1) กั (0x350B[3:0]/16+1)
        ret = sp5506_sccb_write(0x3509, temp&0xff);
       if(ret < 0) {
        DBG_PRINT("ret error!!%d\r\n", __LINE__);
        return ret;
       }

        ret = sp5506_sccb_write(0x3508, temp>>8);
       if(ret < 0) {
        DBG_PRINT("ret error!!%d\r\n", __LINE__);
        return ret;
       }
    }
/*
    t1 = 0x10 | current_group;
    ret = sp5506_sccb_write(0x3208, t1); //Group latch end
	if(ret < 0) return ret;
	t1 = 0xe0 | current_group;
	ret = sp5506_sccb_write(0x3208, t1); // Group Latch Launch
	if(ret < 0) return ret;
*/
	#endif
	/*
	{
	INT8U value;
	    sp5506_sccb_read(0x3500, &value);
    	DBG_PRINT("sp5506 reg addr 0x3500, data 0x%x \r\n",  value);
    		    sp5506_sccb_read(0x3501, &value);
    	DBG_PRINT("sp5506 reg addr 0x3501, data 0x%x \r\n", value);
    		    sp5506_sccb_read(0x3502, &value);
    	DBG_PRINT("sp5506 reg addr 0x3502, data 0x%x \r\n", value);
    }
    */
	return 0;
}



static void sp5506_set_ae(int ev_step)
{
    sp5506_set_xfps_exposure_time(ev_step);
}


void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)sp5506_set_ae;
}


void sensor_get_ae_info(sensor_exposure_t *si)
{
    memcpy(si, &seInfo, sizeof(sensor_exposure_t));
}


void sensor_set_max_lum(int max_lum)
{
    seInfo.max_ev_idx = seInfo.total_ev_idx - (64 - max_lum);
}

#ifdef MIPI_ISR_TEST
static void mipi_sp5506_handle(INT32U event)
{
	if(event & MIPI_LANE0_SOT_SYNC_ERR) {
		DBG_PRINT("LANE0_SOT_SYNC_ERR\r\n");
	}

	if(event & MIPI_HD_1BIT_ERR) {
		DBG_PRINT("HD_1BIT_ERR\r\n");
	}

	if(event & MIPI_HD_NBIT_ERR) {
		DBG_PRINT("HD_NBIT_ERR\r\n");
	}

	if(event & MIPI_DATA_CRC_ERR) {
		DBG_PRINT("DATA_CRC_ERR\r\n");
	}

	if(event & MIPI_LANE1_SOT_SYNC_ERR) {
		DBG_PRINT("LANE1_SOT_SYNC_ERR\r\n");
	}

	if(event & MIPI_CCIR_SOF) {
		DBG_PRINT("CCIR_SOF\r\n");
	}
}
#endif


void sp5506_seinfo_init(void)
{
    seInfo.sensor_ev_idx = SP5506_25FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= SP5506_25FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= SP5506_25FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = SP5506_25FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = SP5506_25FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)sp5506_mipi_25fps_exp_time_gain_50Hz;


	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x00;

    sensor_set_max_lum(g_FavTable.max_lum);
}

/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void sp5506_cdsp_mipi_init(void)
{
    int ret;

	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

    sp5506_seinfo_init();


	// cdsp init
	drv_l2_cdsp_open();

	// set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk
	//R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	//R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<3)|(1<<2));//0x02400042;


	// mclk output
	//drv_l2_sensor_set_mclkout(sp5506_cdsp_mipi_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);
/*    osDelay(10);

	// Reset
	gpio_set_port_attribute(IO_A14, ATTRIBUTE_HIGH);
	gpio_init_io(IO_A14, GPIO_OUTPUT);
	gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);
    gpio_write_io(IO_A14, DATA_LOW);
    osDelay(20);
    gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);*/

	// cdsp init
	//drv_l2_cdsp_open();

	// mipi enable
#if (MIPI_DEV_NO == 1)
	drv_l1_mipi_init(MIPI_1,ENABLE);
    //drv_l1_mipi_init(MIPI_1,DISABLE);
#else
    drv_l1_mipi_init(MIPI_0,ENABLE);
    //drv_l1_mipi_init(MIPI_0,DISABLE);
#endif

    DBG_PRINT("Sensor sp5506 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void sp5506_cdsp_mipi_uninit(void)
{
	// disable mclk
	drv_l2_sensor_set_mclkout(MCLK_NONE);

	// cdsp disable
	//drv_l2_cdsp_close();

	// mipi disable
        #if (MIPI_DEV_NO == 1)
          drv_l1_mipi_init(MIPI_1, DISABLE);
        #else
          drv_l1_mipi_init(MIPI_0, DISABLE);
        #endif

	// release sccb
	sp5506_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void sp5506_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
    INT16U R0x30F0, R0x3072, R0x300E;
    int ret;
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;
    INT16U reg;
    INT8U value;
	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);
	drv_l2_sensor_set_mclkout(MCLK_24M);


    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 2);

	drv_l2_sensor_set_mclkout(sp5506_cdsp_mipi_ops.info[index].mclk);


    // set cdsp format
	format.image_source = C_CDSP_MIPI;
	format.input_format =  sp5506_cdsp_mipi_ops.info[index].input_format;
	format.output_format = sp5506_cdsp_mipi_ops.info[index].output_format;
	target_w = sp5506_cdsp_mipi_ops.info[index].target_w;
	target_h = sp5506_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = sp5506_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = sp5506_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = sp5506_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = sp5506_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = sp5506_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = sp5506_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = sp5506_cdsp_mipi_ops.info[index].vsync_active;

    if(drv_l2_cdsp_set_fmt(&format) < 0)
    {
		DBG_PRINT("cdsp set fmt err!!!\r\n");
	}

	// set scale down
	if((sensor_w > target_w) || (sensor_h > target_h)) {
		drv_l2_cdsp_set_yuv_scale(target_w, target_h);
	}



	// set mipi format.
#if 1
    sp5506_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
    sp5506_mipi_cfg.data_type = MIPI_RAW10;
    sp5506_mipi_cfg.h_back_porch = 4;

	sp5506_mipi_cfg.h_size = format.hpixel;
	sp5506_mipi_cfg.v_size = format.vline+format.voffset ;
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      drv_l1_mipi_set_parameter(MIPI_1, &sp5506_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      drv_l1_mipi_set_parameter(MIPI_0, &sp5506_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
    #endif



    // reguest sccb
	sp5506_sccb_open();
	osDelay(2);

//	gpio_write_io(IO_A12, DATA_HIGH);
//	osDelay(2);


	// init sensor
	DBG_PRINT("Default sensor clock\r\n");

	//sp5506_sccb_write_table((regval16_t *)sp5506_Mipi_Raw10_5M_25fps);
	//sp5506_sccb_write_table((regval16_t *)sp5506_Mipi_Raw10_5M_2Lane_30fps);
	sp5506_sccb_write_table((regval16_t *)sp5506_Mipi_Raw10_5M_2Lane_25fps);

    reg = 0x4800;
    sp5506_sccb_read((INT16U) reg, &value);
    DBG_PRINT("sp5506 reg addr 0x%x, data 0x%x \r\n", reg, value);

    DBG_PRINT("Sensor clock after setting\r\n");

    //dump_sensor_clock(24);
    //dump_current_sp5506_resolution();

    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    sp5506_set_xfps_exposure_time(0);

    // cdsp start
	drv_l2_CdspTableRegister((gpCisCali_t *)&g_cali);
	drv_l2_cdsp_stream_on(ENABLE, bufA, bufB);


	drv_l2_cdsp_enable(&g_FavTable, sensor_w, sensor_h, target_w, target_h);


}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void sp5506_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();

	sp5506_sccb_close();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* sp5506_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&sp5506_cdsp_mipi_ops.info[index];
	}
}


/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t sp5506_cdsp_mipi_ops =
{
	SENSOR_SP5506_CDSP_MIPI_NAME,		/* sensor name */
	sp5506_cdsp_mipi_init,
	sp5506_cdsp_mipi_uninit,
	sp5506_cdsp_mipi_stream_on,
	sp5506_cdsp_mipi_stream_off,
	sp5506_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SGBRG10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			SP5506_WIDTH,				/* sensor width */
			SP5506_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 2st info */
		{
			MCLK_24M,					/* MCLK */
			//V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			SP5506_OUT_WIDTH,			/* target width */
			SP5506_OUT_HEIGHT, 		/* target height */
			SP5506_WIDTH,				/* sensor width */
			SP5506_HEIGHT, 				/* sensor height */
			4,							/* sensor h offset */
			2,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 3st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_YUYV,			/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			SP5506_WIDTH,				/* sensor width */
			SP5506_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 4st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_YUYV,			/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			SP5506_OUT_WIDTH,			/* target width */
			SP5506_OUT_HEIGHT, 			/* target height */
			SP5506_WIDTH,				/* sensor width */
			SP5506_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};

#endif //(defined _SENSOR_SP5506_CDSP_MIPI) && (_SENSOR_SP5506_CDSP_MIPI == 1)
