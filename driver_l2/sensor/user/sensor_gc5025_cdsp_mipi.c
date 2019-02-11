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
#include "drv_l2_sensor.h"
#include "drv_l2_sccb.h"
#include "drv_l2_cdsp.h"


#if (defined _SENSOR_GC5025_CDSP_MIPI) && (_SENSOR_GC5025_CDSP_MIPI == 1)
#include "drv_l2_user_preference.h"
#include "drv_l2_user_calibration.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_TEST                0
#define COLOR_BAR_EN                    0
#define MIPI_ISR_TEST                   1
#define MIPI_LANE_NO			        2           // 1 or 2 lane
#if (MIPI_LANE_NO == 2)
#define MIPI_DEV_NO                     0           //2Lane only selcet MIPI_0
#else
#define MIPI_DEV_NO                     0           //1Lane can 0:MIPI_0 or 1:MIPI_1
#endif

#define	GC5025_ID							(0x6E)
#define GC5025_WIDTH						 2560
#define GC5025_HEIGHT					    (1944-12)
#define GC5025_OUT_WIDTH					 2560
#define GC5025_OUT_HEIGHT					(1944-16)

//Exposure Time Setting
#define GC5025_20FPS_50HZ_DAY_EV_IDX 			143
#define GC5025_20FPS_50HZ_NIGHT_EV_IDX			194
#define GC5025_20FPS_50HZ_EXP_TIME_TOTAL		254
#define GC5025_20FPS_50HZ_INIT_EV_IDX 			((GC5025_20FPS_50HZ_DAY_EV_IDX + GC5025_20FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define GC5025_20FPS_50HZ_MAX_EV_IDX			(GC5025_20FPS_50HZ_EXP_TIME_TOTAL - 4)

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct regval8_s
{
	INT8U reg_num;
	INT8U value;
} regval8_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static int ob_cnt = 0;
static int *p_expTime_table;
static sensor_exposure_t  seInfo;
static int pre_sensor_a_gain, pre_sensor_time;
static INT32U gc5025_analog_gain;

static INT8U ae_flag = 0;

#if SCCB_MODE == SCCB_GPIO
	static void *gc5025_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t gc5025_handle;
#endif

static mipi_config_t gc5025_mipi_cfg =
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
	MIPI_RAW10,//MIPI_RAW8,			/* data type, valid when data mode is 1*/
	MIPI_DATA_TO_CDSP,	/* data type, 1: data[7:0]+2':00 to cdsp,  0: 2'00+data[7:0] to csi */
	0,//NULL,			/* RSD 2 */

	GC5025_WIDTH,		/* width, 0~0xFFFF */
	GC5025_HEIGHT,		/* height, 0~0xFFFF */
	4,
	4,

	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,//NULL,			/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	200,				/* data mask, unit is ns */
	MIPI_CHECK_LP_00//MIPI_CHECK_HS_SEQ	//MIPI_CHECK_LP_00///* check hs sequence or LP00 for clock lane */
};

static const regval8_t gc5025_reset_table[] =
{// SYS
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0xFF, 0xFF}
};

static const regval8_t gc5025_15fps_624M_2560x1944[] =
{
    //MCLK=24MHz,MIPI_Clock=624MHz,frame_rate=15 fps
    //Actual_window_size=2560*1944
    //pixel_line=3713,frame_line=2800,row_time=23.8us
    {0xfe,0x80},
    {0xfe,0x80},
    {0xfe,0x80},
    {0xf7,0x01},
    {0xf8,0x0c},
    {0xf9,0x00},
    {0xfa,0xa0},
    {0xfc,0x2a},
    {0xfe,0x03},
    {0x01,0x07},
    {0xfc,0x2e},
    {0xfe,0x00},
    {0x88,0x03},
    {0xe7,0xcc},
    {0x03,0x07},
    {0x04,0x08},
    {0x05,0x02},
    {0x06,0x6b},
    {0x07,0x03},
    {0x08,0x3c},
    {0x09,0x00},
    {0x0a,0x1c},
    {0x0b,0x00},
    {0x0c,0x04},
    {0x0d,0x07},
    {0x0e,0x9c},  //1948=1944+4
    {0x0f,0x0a},
    {0x10,0x30},  //2608=2592+16
    {0x17,0xc0},
    {0x18,0x02},
    {0x19,0x17},
    {0x1a,0x1a},
    {0x1c,0x1c},//28
    {0x1d,0x13},
    {0x1e,0x90},
    {0x1f,0xb0},
    {0x20,0x2b},
    {0x21,0x2b},
    {0x26,0x2b},
    {0x25,0xc1},
    {0x27,0x64},
    {0x28,0x00},
    {0x29,0x3f},
    {0x2b,0x80},
    {0x2f,0x4a},//4d
    {0x30,0x11},//44
    {0x31,0x20},//40
    {0x32,0xa0},//c0
    {0x33,0x00},
    {0x34,0x55},
    {0x38,0x02},//08
    {0x39,0x00},//08
    {0x3a,0x00},//0a
    {0x3b,0x00},//09
    {0x3c,0x02},//0d
    {0x3d,0x02},
    {0x81,0x60},
    {0xcb,0x02},
    {0xcd,0xad},//af
    {0xcf,0x50},
    {0xd0,0xb3},//b2
    {0xd1,0x18},//00
    {0xd3,0xcc},//cb
    {0xd9,0xaa},
    {0xdc,0x03},
    {0xdd,0xaa},
    {0xe0,0x00},
    {0xe1,0x0a},
    {0xe3,0x2a},
    {0xe4,0xa0},//c0
    {0xe5,0x06},//09
    {0xe6,0x10},
    {0xe7,0xc2},//cc
    {0x80,0x10},
    {0x89,0x03},
    {0xfe,0x01},
    {0x88,0xf7},
    {0x8a,0x03},
    {0x8e,0xc7},
    {0xfe,0x00},
    {0x40,0x22},
    {0x43,0x03},
    {0xae,0x40},
    {0x60,0x00},
    {0x61,0x80},
    {0xb0,0x58},
    {0xb1,0x01},
    {0xb2,0x00},
    {0xb6,0x00},
    {0x90,0x01},
    {0x91,0x00},
    {0x92,0x03},
    {0x93,0x00},
    {0x94,0x04},
    {0x95,0x07},
    {0x96,0x98}, // 1944
    {0x97,0x0a},
    {0x98,0x00}, // 2560
    {0xfe,0x00},
    {0x3f,0x91},
    {0xfe,0x03},
    {0x02,0x07},
    {0x03,0x8e},
    {0x06,0x80},

    {0x12,0x80},
    {0x13,0x0c},

    {0x15,0x02},
    {0x16,0x09},
    {0x18,0x0a},
    {0x21,0x10},
    {0x22,0x05},
    {0x23,0x20},
    {0x24,0x02},
    {0x25,0x20},
    {0x26,0x08},
    {0x29,0x06},
    {0x2a,0x0a},
    {0x2b,0x08},
    {0xfe,0x00},

    {0xFF, 0xFF}
};

static const regval8_t gc5025_20fps_624M_2560x1944[] =
{
    //MCLK=24MHz,MIPI_Clock=624MHz,frame_rate=20 fps
    //Actual_window_size=2560*1944
    //pixel_line=3713,frame_line=2100,row_time=23.8us
    {0xfe,0x80},
    {0xfe,0x80},
    {0xfe,0x80},
    {0xf7,0x01},
    {0xf8,0x0c},
    {0xf9,0x00},
    {0xfa,0xa0},
    {0xfc,0x2a},
    {0xfe,0x03},
    {0x01,0x07},
    {0xfc,0x2e},
    {0xfe,0x00},
    {0x88,0x03},
    {0xe7,0xcc},
    {0x03,0x07},
    {0x04,0x08},
    {0x05,0x02},
    {0x06,0x6b},
    {0x07,0x00},
    {0x08,0x80},
    {0x09,0x00},
    {0x0a,0x1c},
    {0x0b,0x00},
    {0x0c,0x04},
    {0x0d,0x07},
    {0x0e,0x9c},  //1948=1944+4
    {0x0f,0x0a},
    {0x10,0x30},  //2608=2592+16
    {0x17,0xc0},
    {0x18,0x02},
    {0x19,0x17},
    {0x1a,0x1a},
    {0x1c,0x1c},//28
    {0x1d,0x13},
    {0x1e,0x90},
    {0x1f,0xb0},
    {0x20,0x2b},
    {0x21,0x2b},
    {0x26,0x2b},
    {0x25,0xc1},
    {0x27,0x64},
    {0x28,0x00},
    {0x29,0x3f},
    {0x2b,0x80},
    {0x2f,0x4a},//4d
    {0x30,0x11},//44
    {0x31,0x20},//40
    {0x32,0xa0},//c0
    {0x33,0x00},
    {0x34,0x55},
    {0x38,0x02},//08
    {0x39,0x00},//08
    {0x3a,0x00},//0a
    {0x3b,0x00},//09
    {0x3c,0x02},//0d
    {0x3d,0x02},
    {0x81,0x60},
    {0xcb,0x02},
    {0xcd,0xad},//af
    {0xcf,0x50},
    {0xd0,0xb3},//b2
    {0xd1,0x18},//00
    {0xd3,0xcc},//cb
    {0xd9,0xaa},
    {0xdc,0x03},
    {0xdd,0xaa},
    {0xe0,0x00},
    {0xe1,0x0a},
    {0xe3,0x2a},
    {0xe4,0xa0},//c0
    {0xe5,0x06},//09
    {0xe6,0x10},
    {0xe7,0xc2},//cc
    {0x80,0x10},
    {0x89,0x03},
    {0xfe,0x01},
    {0x88,0xf7},
    {0x8a,0x03},
    {0x8e,0xc7},
    {0xfe,0x00},
    {0x40,0x22},
    {0x43,0x03},
    {0xae,0x40},
    {0x60,0x00},
    {0x61,0x80},
    {0xb0,0x58},
    {0xb1,0x01},
    {0xb2,0x00},
    {0xb6,0x00},
    {0x90,0x01},
    {0x91,0x00},
    {0x92,0x03},
    {0x93,0x00},
    {0x94,0x04},
    {0x95,0x07},
    {0x96,0x98}, // 1944
    {0x97,0x0a},
    {0x98,0x00}, // 2560
    {0xfe,0x00},
    {0x3f,0x91},
    {0xfe,0x03},
    {0x02,0x07},
    {0x03,0x8e},
    {0x06,0x80},

    {0x12,0x80},
    {0x13,0x0c},

    {0x15,0x02},
    {0x16,0x09},
    {0x18,0x0a},
    {0x21,0x10},
    {0x22,0x05},
    {0x23,0x20},
    {0x24,0x02},
    {0x25,0x20},
    {0x26,0x08},
    {0x29,0x06},
    {0x2a,0x0a},
    {0x2b,0x08},
    {0xfe,0x00},

    {0xFF, 0xFF}
};

static const regval8_t gc5025_25fps_672M_2560x1944[] =
{
    //MCLK=24MHz,MIPI_Clock=672MHz,frame_rate=25 fps
    //Actual_window_size=2560*1944
    //pixel_line=3456,frame_line=1980,row_time=20.57us
    {0xfe,0x80},
    {0xfe,0x80},
    {0xfe,0x80},
    {0xf7,0x01},
    {0xf8,0x0d},
    {0xf9,0x00},
    {0xfa,0xa0},
    {0xfc,0x2a},
    {0xfe,0x03},
    {0x01,0x07},
    {0xfc,0x2e},
    {0xfe,0x00},
    {0x88,0x03},
    {0xe7,0xcc},
    {0x03,0x07},
    {0x04,0x08},
    {0x05,0x02},
    {0x06,0x40},
    {0x07,0x00},
    {0x08,0x08},
    {0x09,0x00},
    {0x0a,0x1c},
    {0x0b,0x00},
    {0x0c,0x04},
    {0x0d,0x07},
    {0x0e,0x9c},  //1948=1944+4
    {0x0f,0x0a},
    {0x10,0x30},  //2608=2592+16
    {0x17,0xc0},
    {0x18,0x02},
    {0x19,0x17},
    {0x1a,0x1a},
    {0x1c,0x1c},//28
    {0x1d,0x13},
    {0x1e,0x90},
    {0x1f,0xb0},
    {0x20,0x2b},
    {0x21,0x2b},
    {0x26,0x2b},
    {0x25,0xc1},
    {0x27,0x64},
    {0x28,0x00},
    {0x29,0x3f},
    {0x2b,0x80},
    {0x2f,0x4a},//4d
    {0x30,0x11},//44
    {0x31,0x20},//40
    {0x32,0xa0},//c0
    {0x33,0x00},
    {0x34,0x55},
    {0x38,0x02},//08
    {0x39,0x00},//08
    {0x3a,0x00},//0a
    {0x3b,0x00},//09
    {0x3c,0x02},//0d
    {0x3d,0x02},
    {0x81,0x60},
    {0xcb,0x02},
    {0xcd,0xad},//af
    {0xcf,0x50},
    {0xd0,0xb3},//b2
    {0xd1,0x18},//00
    {0xd3,0xcc},//cb
    {0xd9,0xaa},
    {0xdc,0x03},
    {0xdd,0xaa},
    {0xe0,0x00},
    {0xe1,0x0a},
    {0xe3,0x2a},
    {0xe4,0xa0},//c0
    {0xe5,0x06},//09
    {0xe6,0x10},
    {0xe7,0xc2},//cc
    {0x80,0x10},
    {0x89,0x03},
    {0xfe,0x01},
    {0x88,0xf7},
    {0x8a,0x03},
    {0x8e,0xc7},
    {0xfe,0x00},
    {0x40,0x22},
    {0x43,0x03},
    {0xae,0x40},
    {0x60,0x00},
    {0x61,0x80},
    {0xb0,0x58},
    {0xb1,0x01},
    {0xb2,0x00},
    {0xb6,0x00},
    {0x90,0x01},
    {0x91,0x00},
    {0x92,0x03},
    {0x93,0x00},
    {0x94,0x04},
    {0x95,0x07},
    {0x96,0x98}, // 1944
    {0x97,0x0a},
    {0x98,0x00}, // 2560
    {0xfe,0x00},
    {0x3f,0x91},
    {0xfe,0x03},
    {0x02,0x07},
    {0x03,0x8e},
    {0x06,0x80},

    {0x12,0x80},
    {0x13,0x0c},

    {0x15,0x02},
    {0x16,0x09},
    {0x18,0x0a},
    {0x21,0x10},
    {0x22,0x05},
    {0x23,0x20},
    {0x24,0x02},
    {0x25,0x20},
    {0x26,0x08},
    {0x29,0x06},
    {0x2a,0x0a},
    {0x2b,0x08},
    {0xfe,0x00},
    {0xFF, 0xFF}
};

static const int gc5025_20fps_exp_time_gain_50Hz[GC5025_20FPS_50HZ_EXP_TIME_TOTAL][3] =
{ //AGain max 23, DGain max
    { 4	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 4	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 4	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 5	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 5	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 5	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 5	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 5	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 6	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 6	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 6	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 6	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 6	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 7	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 7	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 7	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 7	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 8	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 8	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 8	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 8	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 9	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 9	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 9	  ,     (int)(1.00*64),   (int)(1.00*32) },
    { 10  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 10  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 10  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 11  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 11  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 11  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 12  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 12  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 13  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 13  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 14  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 14  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 15  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 15  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 16  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 16  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 17  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 17  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 18  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 19  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 19  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 20  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 21  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 21  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 22  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 23  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 24  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 24  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 25  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 26  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 27  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 28  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 29  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 30  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 31  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 32  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 33  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 35  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 36  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 37  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 38  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 40  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 41  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 43  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 44  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 46  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 47  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 49  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 51  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 53  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 54  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 56  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 58  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 60  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 62  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 65  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 67  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 69  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 72  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 74  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 77  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 80  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 82  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 85  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 88  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 91  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 95  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 98  ,	    (int)(1.00*64),   (int)(1.00*32) },
    { 101  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 105  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 109  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 113  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 117  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 121  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 125  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 129  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 134  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 139  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 143  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 148  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 154  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 159  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 165  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 171  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 177  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 183  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 189  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 196  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 203  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 210  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 217  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 225  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 233  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 241  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 250  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 259  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 268  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 277  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 287  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 297  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 307  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 318  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 330  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 341  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 353  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 366  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 379  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 392  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 406  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 420  ,	(int)(1.00*64),   (int)(1.00*32) },
    { 420  ,	(int)(1.0625*64), (int)(1.00*32) },
    { 420  ,	(int)(1.0625*64), (int)(1.00*32) },
    { 420  ,	(int)(1.125*64),  (int)(1.00*32) },
    { 420  ,	(int)(1.125*64),  (int)(1.00*32) },
    { 420  ,	(int)(1.1875*64), (int)(1.00*32) },
    { 420  ,	(int)(1.25*64),   (int)(1.00*32) },
    { 420  ,	(int)(1.25*64),   (int)(1.00*32) },
    { 420  ,	(int)(1.3125*64), (int)(1.00*32) },
    { 420  ,	(int)(1.375*64),  (int)(1.00*32) },
    { 420  ,	(int)(1.4375*64), (int)(1.00*32) },
    { 420  ,	(int)(1.4375*64), (int)(1.00*32) },
    { 420  ,	(int)(1.5*64),    (int)(1.00*32) },
    { 420  ,	(int)(1.5625*64), (int)(1.00*32) },
    { 420  ,	(int)(1.625*64),  (int)(1.00*32) },
    { 420  ,	(int)(1.6875*64), (int)(1.00*32) },
    { 420  ,	(int)(1.75*64), (int)(1.00*32)   },
    { 420  ,	(int)(1.8125*64), (int)(1.00*32) },
    { 420  ,	(int)(1.875*64), (int)(1.00*32)  },
    { 420  ,	(int)(1.9375*64), (int)(1.00*32) },
    { 840  ,	(int)(1.00*64), (int)(1.00*32)   },
    { 840  ,	(int)(1.0625*64), (int)(1.00*32) },
    { 840  ,	(int)(1.0625*64), (int)(1.00*32) },
    { 840  ,	(int)(1.125*64), (int)(1.00*32)  },
    { 840  ,	(int)(1.125*64), (int)(1.00*32)  },
    { 840  ,	(int)(1.1875*64), (int)(1.00*32) },
    { 840  ,	(int)(1.25*64), (int)(1.00*32)   },
    { 840  ,	(int)(1.25*64), (int)(1.00*32)   },
    { 840  ,	(int)(1.3125*64), (int)(1.00*32) },
    { 840  ,	(int)(1.375*64), (int)(1.00*32)  },
    { 840  ,	(int)(1.4375*64), (int)(1.00*32) },
    { 840  ,	(int)(1.4375*64), (int)(1.00*32) },
    { 840  ,	(int)(1.5*64), (int)(1.00*32)    },
    { 840  ,	(int)(1.5625*64), (int)(1.00*32) },
    { 840  ,	(int)(1.625*64), (int)(1.00*32)  },
    { 840  ,	(int)(1.6875*64), (int)(1.00*32) },
    { 840  ,	(int)(1.75*64), (int)(1.00*32)   },
    { 840  ,	(int)(1.8125*64), (int)(1.00*32) },
    { 840  ,	(int)(1.875*64), (int)(1.00*32)  },
    { 840  ,	(int)(1.9375*64), (int)(1.00*32) },
    { 1260 ,	(int)(1.3125*64), (int)(1.00*32) },
    { 1260 ,	(int)(1.375*64), (int)(1.00*32)  },
    { 1260 ,	(int)(1.4375*64), (int)(1.00*32) },
    { 1260 ,	(int)(1.4375*64), (int)(1.00*32) },
    { 1260 ,	(int)(1.5*64), (int)(1.00*32)    },
    { 1260 ,	(int)(1.5625*64), (int)(1.00*32) },
    { 1260 ,	(int)(1.625*64), (int)(1.00*32)  },
    { 1260 ,	(int)(1.6875*64), (int)(1.00*32) },
    { 1260 ,	(int)(1.75*64), (int)(1.00*32)   },
    { 1260 ,	(int)(1.8125*64), (int)(1.00*32) },
    { 1260 ,	(int)(1.875*64), (int)(1.00*32)  },
    { 1260 ,	(int)(1.9375*64), (int)(1.00*32) },
    { 1680 ,	(int)(1.5*64), (int)(1.00*32)    },
    { 1680 ,	(int)(1.5625*64), (int)(1.00*32) },
    { 1680 ,	(int)(1.625*64), (int)(1.00*32)  },
    { 1680 ,	(int)(1.6875*64), (int)(1.00*32) },
    { 1680 ,	(int)(1.75*64), (int)(1.00*32)   },
    { 1680 ,	(int)(1.8125*64), (int)(1.00*32) },
    { 1680 ,	(int)(1.875*64), (int)(1.00*32)  },
    { 1680 ,	(int)(1.9375*64), (int)(1.00*32) },
    { 1680 ,	(int)(2.0*64), (int)(1.00*32)    },
    { 1680 ,	(int)(2.07*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.14*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.22*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.30*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.38*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.46*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.55*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.64*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.73*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.83*64), (int)(1.00*32)   },
    { 1680 ,	(int)(2.93*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.03*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.14*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.25*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.36*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.48*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.61*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.73*64), (int)(1.00*32)   },
    { 1680 ,	(int)(3.86*64), (int)(1.00*32)   },
    { 1680 ,	(int)(4.00*64), (int)(1.00*32)   },
    { 1680 ,	(int)(4.14*64), (int)(1.00*32)   },
    { 1680 ,	(int)(4.29*64), (int)(1.00*32)   },
    { 1680 ,	(int)(4.44*64), (int)(1.00*32)   },
    { 1680 ,	(int)(4.59*64), (int)(1.00*32)   },
    { 1680 ,	(int)(4.76*64), (int)(1.00*32)   },
    { 1680 ,	(int)(4.92*64), (int)(1.00*32)   },
    { 1680 ,	(int)(5.10*64), (int)(1.00*32)   },
    { 1680 ,	(int)(5.28*64), (int)(1.00*32)   },
    { 1680 ,	(int)(5.46*64), (int)(1.00*32)   },
    { 1680 ,	(int)(5.66*64), (int)(1.00*32)   },
    { 1680 ,	(int)(5.86*64), (int)(1.00*32)   },
    { 1680 ,	(int)(6.06*64), (int)(1.00*32)   },
    { 1680 ,	(int)(6.28*64), (int)(1.00*32)   },
    { 1680 ,	(int)(6.50*64), (int)(1.00*32)   },
    { 1680 ,	(int)(6.73*64), (int)(1.00*32)   },
    { 1680 ,	(int)(6.96*64), (int)(1.00*32)   },
    { 1680 ,	(int)(7.21*64), (int)(1.00*32)   },
    { 1680 ,	(int)(7.46*64), (int)(1.00*32)   },
    { 1680 ,	(int)(7.73*64), (int)(1.00*32)   },
    { 1680 ,	(int)(8.00*64), (int)(1.00*32)   },
    { 1680 ,	(int)(8.28*64), (int)(1.00*32)   },
    { 1680 ,	(int)(8.57*64), (int)(1.00*32)   },
    { 1680 ,	(int)(8.88*64), (int)(1.00*32)   },
    { 1680 ,	(int)(9.19*64), (int)(1.00*32)   },
    { 1680 ,	(int)(9.51*64), (int)(1.00*32)   },
    { 1680 ,	(int)(9.85*64), (int)(1.00*32)   },
    { 1680 ,	(int)(10.20*64), (int)(1.00*32)  },
    { 1680 ,	(int)(10.56*64), (int)(1.00*32)  },
    { 1680 ,	(int)(10.93*64), (int)(1.00*32)  },
    { 1680 ,	(int)(11.31*64), (int)(1.00*32)  },
    { 1680 ,	(int)(11.71*64), (int)(1.00*32)  },
    { 1680 ,	(int)(12.13*64), (int)(1.00*32)  },
    { 1680 ,	(int)(12.55*64), (int)(1.00*32)  },
    { 1680 ,	(int)(13.00*64), (int)(1.00*32)  },
    { 1680 ,	(int)(13.45*64), (int)(1.00*32)  },
    { 1680 ,	(int)(13.93*64), (int)(1.00*32)  },
    { 1680 ,	(int)(14.42*64), (int)(1.00*32)  },
    { 1680 ,	(int)(14.93*64), (int)(1.00*32)  },
    { 1680 ,	(int)(15.45*64), (int)(1.00*32)  },
    { 1680 ,	(int)(16.00*64), (int)(1.00*32)  }
};//50Hz

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S gc5025_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	gc5025_handle = drv_l2_sccb_open(GC5025_ID, 8, 8);
	if(gc5025_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	gc5025_handle.devNumber = I2C_0;
	gc5025_handle.slaveAddr = GC5025_ID;
	gc5025_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void gc5025_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(gc5025_handle) {
		drv_l2_sccb_close(gc5025_handle);
		gc5025_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	gc5025_handle.slaveAddr = 0;
	gc5025_handle.clkRate = 0;
#endif
}

static INT32S gc5025_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(gc5025_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&gc5025_handle, data, 3);
#endif
}


static INT32S gc5025_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(gc5025_handle, reg, &data) >= 0) {
		*value = (INT8U)data;
		return STATUS_OK;
	} else {
		*value = 0xFF;
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[2];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	if(drv_l1_i2c_bus_write(&gc5025_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&gc5025_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}

static INT32S gc5025_sccb_write_table(regval8_t *pTable)
{
	while(1) {
		if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
			break;
		}

		//DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(gc5025_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}

		pTable++;
	}

	DBG_PRINT("\r\n\r\n");

	return STATUS_OK;
}

static INT32S gc5025_sccb_read_table(regval8_t *pTable)
{
    INT8U *Rdata;
	while(1) {
		if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
			break;
		}

		if(gc5025_sccb_read(pTable->reg_num, Rdata) < 0) {
			DBG_PRINT("sccb read fail.\r\n");
			continue;
		}
		DBG_PRINT("0x%02x, 0x%02x = 0x%02x\r\n", pTable->reg_num, pTable->value, *Rdata);
		pTable++;
	}
	DBG_PRINT("\r\n\r\n");

	return STATUS_OK;
}

/*****************************************************************************************
+++++++++++++++++AEC/AGC
*****************************************************************************************/
static int GC2023_cvt_analog_gain(int analog_gain)
{
	//unsigned int Analog_Multiple[7]={1000, 1400, 1900, 2750, 3900,5550,6660};
	//unsigned int Analog_Multiple[12]={500,700,1000, 1420, 2040, 2990, 4330,6300,8800, 12500, 17800,26000};
	unsigned int Analog_Multiple[12]={1.0*2560,1.42*2560,1.99*2560, 2.86*2560, 4.05*2560, 5.78*2560, 8.23*2560,11.73*2560,16.55*2560, 23.28*2560,33.27*2560};
	unsigned int Analog_Index;
	unsigned int Digital_Gain;
	unsigned int Decimal;
	unsigned int gc2023_analog_gain;

	gc2023_analog_gain = analog_gain*10;

	Analog_Index=0;
	while(Analog_Index<11)
	{
		if(gc2023_analog_gain<Analog_Multiple[Analog_Index])
		{
			break;
		}
		else
		{
			Analog_Index++;
		}
	}
	//Analog_Index = 5;


	Digital_Gain = gc2023_analog_gain*1000/Analog_Multiple[Analog_Index-1];
	Decimal=(Digital_Gain*64)/1000;

//DBG_PRINT("set analog idx %d, %d, %d\r\n", Analog_Index-1, Decimal>>6, (Decimal)&0x3f);
	gc5025_sccb_write(0xb6,   Analog_Index-1);
	gc5025_sccb_write(0xb1,  Decimal>>6);
	gc5025_sccb_write(0xb2,  (Decimal<<2)&0xfc);
	//gc5025_sccb_write(0x61,  0x80);

	return 0;
}

static int gc5025_cvt_agc_gain(int agc_gain)
{
    INT32U Analog_Multiple[2]={1000, 1445};

    INT32U Analog_Index;
    INT32U Digital_Gain;
    INT32U Decimal;
    INT32U ret;

    //DBG_PRINT("agc_gain: %d\r\n", agc_gain);

    gc5025_analog_gain = agc_gain*10;

    Analog_Index=0;

    while(Analog_Index<2)
    {
        if(gc5025_analog_gain<Analog_Multiple[Analog_Index])
        {
            break;
        }
        else
        {
            Analog_Index++;
        }
    }

    Digital_Gain = gc5025_analog_gain*1000/Analog_Multiple[Analog_Index-1];
    Decimal=(Digital_Gain*64)/1000;

    ret = gc5025_sccb_write(0xb1,  Decimal>>6);
    if(ret < 0) return ret;

    ret = gc5025_sccb_write(0xb2,  (Decimal<<2)&0xfc);
    if(ret < 0) return ret;

    ret = gc5025_sccb_write(0xb6,   Analog_Index-1);
    if(ret < 0) return ret;

    return 0;
}

static void gc5025_set_shutter(INT16U shutter)
{
	// Update Shutter
	if(shutter > 8191) shutter = 8191;
	if(shutter <1) shutter = 1;
	shutter=shutter/2;
	shutter=shutter*2;
	//Update Shutter
	gc5025_sccb_write(0xfe,0x00);
	gc5025_sccb_write(0x03,(shutter>>8) & 0x3F);
	gc5025_sccb_write(0x04,shutter & 0xFF);
}

#define ANALOG_GAIN_1 64   // 1.00x
#define ANALOG_GAIN_2 92   // 1.445x

static INT16U gc5025_set_gain(INT16U gain)
{
	INT16U iReg,temp;

	iReg = gain;

	if(iReg < 0x40)
		iReg = 0x40;

	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	//if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_1*2))
	{
		gc5025_sccb_write(0xfe,  0x00);
		gc5025_sccb_write(0xb6,  0x00);
		temp = iReg;
		gc5025_sccb_write(0xb1, temp>>6);
		gc5025_sccb_write(0xb2, (temp<<2)&0xfc);
	}
	else
	{
		gc5025_sccb_write(0xfe,  0x00);
		gc5025_sccb_write(0xb6,  0x01);
		temp = 64*iReg/ANALOG_GAIN_2;
		gc5025_sccb_write(0xb1, temp>>6);
		gc5025_sccb_write(0xb2, (temp<<2)&0xfc);
	}
	return gain;
}

extern INT32U g_flag_update;
extern int ev_step_index;

static int gc5025_set_xfps_exposure_time(sensor_exposure_t *si)
{
    unsigned char t1, t2;
    int idx, temp, ret;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    if(g_flag_update!=0) {
        DBG_PRINT("%s: not set exposure time\n\r", __FUNCTION__);
        return 0;
    }
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //DBG_PRINT("%s:%d\n", __FUNCTION__, __LINE__);

    si->sensor_ev_idx += si->ae_ev_step;
    if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
    if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;

    //si->sensor_ev_idx = ev_step_index; // for saving raw
    //si->sensor_ev_idx = 250;

    #if RAW_MODE
    idx = (g_raw_mode_idx < 0) ? si->sensor_ev_idx * 3 : g_raw_mode_idx * 3;
    DBG_PRINT("%s:%d %d\n", __FUNCTION__, __LINE__, g_raw_mode_idx);
    #else
    idx = si->sensor_ev_idx * 3;
    #endif

    si ->time = p_expTime_table[idx];
    si ->analog_gain = (int)(p_expTime_table[idx+1]*1.1);
    si ->digital_gain = 32;

    //DBG_PRINT("[EV=%d, offset=%d]: time = %d, analog gain =0x%x, digital_gain=0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain, si ->digital_gain);
    //crf
    // exposure time

    if(si ->time != pre_sensor_time)
    {
        pre_sensor_time = si->time;// = 840*2;
        gc5025_set_shutter(pre_sensor_time);
        //DBG_PRINT("gc5025_set_xfps_exposure_time: time = %d!!!\r\n", si->time);

  /*
        temp = si->time;
        t1 = (temp & 0xff);
        t2 = (temp >> 8) & 0x00ff;

        ret = gc5025_sccb_write(0x04, t1);
        if(ret < 0) return ret;
        ret = gc5025_sccb_write(0x03, t2);
        if(ret < 0) {
          DBG_PRINT("ERR: write sensor time = 0x%x!!!\r\n", temp);
                return ret;
        }*/
    }

    //gain
    if(si ->analog_gain != pre_sensor_a_gain)
    {
        // gain
        pre_sensor_a_gain = si->analog_gain;// = 64*1.36*1.3;//92;
        gc5025_set_gain(pre_sensor_a_gain);
        //DBG_PRINT("gc5025_set_xfps_exposure_time: analog_gain = %d!!!\r\n", si->analog_gain);
   /*
        temp = si->analog_gain;
        ret = gc5025_cvt_agc_gain(temp);
        //ret = GC2023_cvt_analog_gain(temp);
        if(ret < 0) {
          DBG_PRINT("ERR: write sensor analog agin = 0x%x!!!\r\n", temp);
                return ret;
        }*/
    }

    return 0;
}

static void gc5025_set_ae(int ev_step)
{
    seInfo.ae_ev_step = ev_step;
    gc5025_set_xfps_exposure_time(&seInfo);
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)gc5025_set_ae;
}

void sensor_get_ae_info(sensor_exposure_t *si)
{
    memcpy(si, &seInfo, sizeof(sensor_exposure_t));
}

void sensor_set_max_lum(int max_lum)
{
    seInfo.max_ev_idx = seInfo.total_ev_idx - (64 - max_lum);
}

/*****************************************************************************************
-------------------AEC/AGC
*****************************************************************************************/

#ifdef MIPI_ISR_TEST
static void mipi_gc5025_handle(INT32U event)
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++
void gc5025_seinfo_init(void)
{
    seInfo.sensor_ev_idx = GC5025_20FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= GC5025_20FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= GC5025_20FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = GC5025_20FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = GC5025_20FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)gc5025_20fps_exp_time_gain_50Hz;

	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x20;
}

static void gc5025_mipi_sensor_init(void)
{
	// reguest sccb
	gc5025_sccb_open();

	// reset sensor
	gc5025_sccb_write_table((regval8_t *)gc5025_reset_table);
    osDelay(200);

	// init sensor
    //gc5025_sccb_write_table((regval8_t *)gc5025_15fps_624M_2560x1944);
     gc5025_sccb_write_table((regval8_t *)gc5025_20fps_624M_2560x1944);
    //gc5025_sccb_write_table((regval8_t *)gc5025_25fps_672M_2560x1944);

    osDelay(500);

    DBG_PRINT("Sensor gc5025 mipi init completed\r\n");
}


/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
//static void gc5025_cdsp_mipi_init(void)
void gc5025_cdsp_mipi_init(void)
{
	//ae init
	gc5025_seinfo_init();

    //cdsp init
	drv_l2_cdsp_open();

	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

    // set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk
    R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;

    // mclk output
    drv_l2_sensor_set_mclkout(MCLK_24M);

	// Reset
	gpio_set_port_attribute(IO_A14, ATTRIBUTE_HIGH);
	gpio_init_io(IO_A14, GPIO_OUTPUT);
	gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);
    gpio_write_io(IO_A14, DATA_LOW);
    osDelay(20);
    gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);


	// mipi enable
	#if (MIPI_DEV_NO == 1)
		drv_l1_mipi_init(MIPI_1,ENABLE);
	#else
	    drv_l1_mipi_init(MIPI_0,ENABLE);
	#endif

    DBG_PRINT("Sensor gc5025 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void gc5025_cdsp_mipi_uninit(void)
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
	gc5025_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void gc5025_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);
	drv_l2_sensor_set_mclkout(MCLK_24M);

    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 2);

#if 1   // change mclk
	drv_l2_sensor_set_mclkout(gc5025_cdsp_mipi_ops.info[index].mclk);
#else
    drv_l2_sensor_set_mclkout(MCLK_24M);    //OLDWU
#endif
    osDelay(300);

    // set cdsp format
    #if MIPI_DEV_NO == 0
	format.image_source = C_CDSP_MIPI;
	#else
	format.image_source = C_CDSP_MIPI1;
	#endif
	format.input_format =  gc5025_cdsp_mipi_ops.info[index].input_format;
	format.output_format = gc5025_cdsp_mipi_ops.info[index].output_format;
	target_w = gc5025_cdsp_mipi_ops.info[index].target_w;
	target_h = gc5025_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = gc5025_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = gc5025_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = gc5025_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = gc5025_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = gc5025_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = gc5025_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = gc5025_cdsp_mipi_ops.info[index].vsync_active;


    if(drv_l2_cdsp_set_fmt(&format) < 0)
    {
		DBG_PRINT("cdsp set fmt err!!!\r\n");
	}

	// set scale down
	if((format.hpixel > target_w) || (format.vline > target_h)) {
		drv_l2_cdsp_set_yuv_scale(target_w, target_h);
	}

    gc5025_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
    gc5025_mipi_cfg.data_type = MIPI_RAW10;
    gc5025_mipi_cfg.h_back_porch = 4;

	gc5025_mipi_cfg.h_size = format.hpixel;
	gc5025_mipi_cfg.v_size = format.vline+format.voffset ;

    	//mipi start
    #if (MIPI_DEV_NO == 1)
      drv_l1_mipi_set_parameter(MIPI_1, &gc5025_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      drv_l1_mipi_set_parameter(MIPI_0, &gc5025_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
    #endif

	//Enable MCLK & Init Sensor
    gc5025_mipi_sensor_init();

    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    gc5025_set_xfps_exposure_time(&seInfo);

    // cdsp start
	drv_l2_CdspTableRegister((gpCisCali_t*)&g_cali);
	drv_l2_cdsp_stream_on(ENABLE, bufA, bufB);
	drv_l2_cdsp_enable(&g_FavTable, sensor_w, sensor_h, target_w, target_h);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void gc5025_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* gc5025_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&gc5025_cdsp_mipi_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t gc5025_cdsp_mipi_ops =
{
	SENSOR_GC5025_CDSP_MIPI_NAME,		/* sensor name */
	gc5025_cdsp_mipi_init,
	gc5025_cdsp_mipi_uninit,
	gc5025_cdsp_mipi_stream_on,
	gc5025_cdsp_mipi_stream_off,
	gc5025_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SGBRG10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			320,						/* target width */
			240,  						/* target height */
			GC5025_WIDTH,				/* sensor width */
			GC5025_HEIGHT,  				/* sensor height */
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
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			640,			        	/* target width */
			480, 			    		/* target height */
			GC5025_WIDTH,				/* sensor width */
			GC5025_HEIGHT, 				/* sensor height */
			1,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 3st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			20,							/* FPS in sensor */
			GC5025_OUT_WIDTH,			/* target width */
			GC5025_OUT_HEIGHT,  		/* target height */
			GC5025_WIDTH,				/* sensor width */
			GC5025_HEIGHT,  			/* sensor height */
			4,
			0,
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		}
	}
};
#endif //(defined _SENSOR_GC5025_CDSP_MIPI) && (_SENSOR_GC5025_CDSP_MIPI == 1)
