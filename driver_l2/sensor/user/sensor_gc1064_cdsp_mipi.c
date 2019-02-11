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


#if (defined _SENSOR_GC1064_CDSP_MIPI) && (_SENSOR_GC1064_CDSP_MIPI == 1)

#include "drv_l2_user_calibration.h"
#include "drv_l2_user_preference.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_TEST                0
#define COLOR_BAR_EN                    0
#define MIPI_ISR_TEST                   0
#define MIPI_LANE_NO			        1           // 1 or 2 lane
#if (MIPI_LANE_NO == 2)
#define MIPI_DEV_NO                     0           //2Lane only selcet MIPI_0
#else
#define MIPI_DEV_NO                     0           //1Lane can 0:MIPI_0 or 1:MIPI_1
#endif

#define	GC1064_ID							0x78
#define GC1064_WIDTH						1280
#define GC1064_HEIGHT						722
#define GC1064_OUT_WIDTH					1280
#define GC1064_OUT_HEIGHT					720


#define GC1064_30FPS_50HZ_DAY_EV_IDX 			100
#define GC1064_30FPS_50HZ_NIGHT_EV_IDX			155
#define GC1064_30FPS_50HZ_EXP_TIME_TOTAL		200
#define GC1064_30FPS_50HZ_INIT_EV_IDX 			((GC1064_30FPS_50HZ_DAY_EV_IDX + GC1064_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define GC1064_30FPS_50HZ_MAX_EV_IDX			    (GC1064_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define GC1064_30FPS_60HZ_DAY_EV_IDX 			100
#define GC1064_30FPS_60HZ_NIGHT_EV_IDX			170
#define GC1064_30FPS_60HZ_EXP_TIME_TOTAL		202
#define GC1064_30FPS_60HZ_INIT_EV_IDX 			((GC1064_30FPS_60HZ_DAY_EV_IDX + GC1064_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define GC1064_30FPS_60HZ_MAX_EV_IDX			    (GC1064_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define GC1064_24FPS_50HZ_DAY_EV_IDX 			138
#define GC1064_24FPS_50HZ_NIGHT_EV_IDX			194
#define GC1064_24FPS_50HZ_EXP_TIME_TOTAL		    254
#define GC1064_24FPS_50HZ_INIT_EV_IDX 			((GC1064_24FPS_50HZ_DAY_EV_IDX + GC1064_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define GC1064_24FPS_50HZ_MAX_EV_IDX			    (GC1064_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define GC1064_24FPS_60HZ_DAY_EV_IDX 			135
#define GC1064_24FPS_60HZ_NIGHT_EV_IDX			195
#define GC1064_24FPS_60HZ_EXP_TIME_TOTAL		    255
#define GC1064_24FPS_60HZ_INIT_EV_IDX 			((GC1064_24FPS_60HZ_DAY_EV_IDX + GC1064_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define GC1064_24FPS_60HZ_MAX_EV_IDX			    (GC1064_24FPS_60HZ_EXP_TIME_TOTAL - 10)


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
static sensor_exposure_t 	seInfo;
static int pre_sensor_a_gain, pre_sensor_time;
static INT32U GC1064_analog_gain = 0x100;

static INT8U ae_flag = 0;


#if SCCB_MODE == SCCB_GPIO
	static void *GC1064_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t GC1064_handle;
#endif

static mipi_config_t GC1064_mipi_cfg =
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
	MIPI_DATA_TO_CDSP,	/* data type, 1:data[7:0]+2':00 to cdsp, 0: 2'00+data[7:0] to csi */
	0,//NULL,			/* RSD 2 */

	GC1064_WIDTH,		/* width, 0~0xFFFF */
	GC1064_HEIGHT,		/* height, 0~0xFFFF */
	#if SENSOR_FLIP
	2,
	0,
	#else
	4,//2, 					/* back porch, 0~0xF */ //If Set 4 have show paper color at left side.u2.20160119
	6,
	#endif
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,//NULL,			/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	250,				/* data mask, unit is ns */
	MIPI_CHECK_LP_00//MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};

static const regval8_t GC1064_reset_table[] =
	{// SYS
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0x00, 0x00},
};

static const regval8_t GC1064_init_table[] =
{
/////////////////////////////////////////////////////
//////////////////////   SYS   //////////////////////
/////////////////////////////////////////////////////
{0xfe,0x80},
{0xfe,0x80},
{0xfe,0x80},
{0xf2,0x00}, //sync_pad_io_ebi
{0xf6,0x00}, //up down
{0xfc,0xc6},
{0xf7,0x19}, //pll enable
{0xf8,0x03}, //Pll mode 2
{0xf9,0x0e}, //[0] pll enable
{0xfa,0x00}, //div
{0xfe,0x00},

////////////////////////////////////////////////////////
///////////////////   ANALOG & CISCTL   ////////////////
////////////////////////////////////////////////////////
{0x03,0x03},
{0x04,0x64},
{0x05,0x01},
{0x06,0x7c},
{0x07,0x00},
{0x08,0x04},
{0x0d,0x02}, //height
{0x0e,0xe8},
{0x0f,0x05}, //widths
{0x10,0x10},

{0x11,0x00},
{0x12,0x18},
{0x16,0xc0},
{0x17,0x14},
{0x18,0x0a}, //sdark off
{0x19,0x06},
{0x1b,0x4f},
{0x1c,0x41},
{0x1d,0xe0},//f0
{0x1e,0xfc}, //fe The dark stripes
{0x1f,0x38}, //08//comv_r
{0x20,0x81},
{0x21,0x2f}, //2f//20//rsg
{0x22,0xc2},
{0x23,0xf2}, //38
{0x24,0x2f}, //PAD drive
{0x25,0xd4},
{0x26,0xa8},
{0x29,0x3f},
{0x2a,0x00},
{0x2c,0xd0},
{0x2d,0x0f},
{0x2e,0x00},
{0x2f,0x1f},
{0xcc,0x25},
{0xce,0xf3},
{0x3f,0x18},
{0x30,0x00},
{0x31,0x01},
{0x32,0x02},
{0x33,0x03},
{0x34,0x04},
{0x35,0x05},
{0x36,0x06},
{0x37,0x07},
{0x38,0x0f},
{0x39,0x17},
{0x3a,0x1f},
{0x3f,0x18},
////////////////////////////////////////////////////////
/////////////////////////   ISP   //////////////////////
////////////////////////////////////////////////////////
{0xfe,0x00},
{0x8a,0x00},
{0x8c,0x07},
{0x8e,0x02}, //luma value not normal
{0x90,0x01},
{0x94,0x02},
{0x95,0x02},
{0x96,0xd2},//deyue0x2d0
{0x97,0x05},
{0x98,0x00},

////////////////////////////////////////////////////////
/////////////////////////	 BLK	/////////////////////
////////////////////////////////////////////////////////
{0xfe,0x00},
{0x18,0x02},
{0x1a,0x11},
{0x40,0x23},
{0x5e,0x00},
{0x66,0x80},

////////////////////////////////////////////////////////
///////////////////////// Dark SUN /////////////////////
////////////////////////////////////////////////////////

{0xfe,0x00},
{0xcc,0x25},
{0xce,0xf3},

////////////////////////////////////////////////////////
/////////////////////////	 Gain	/////////////////////
////////////////////////////////////////////////////////
{0xfe,0x00},
{0xb0,0x50},
{0xb3,0x40},
{0xb4,0x40},
{0xb5,0x40},
{0xfe,0x00},

////////////////////////////////////////////////////////
/////////////////////////	 MIPI	/////////////////////
{0xfe,0x03},
{0x01,0x83},
{0x02,0x44},
{0x03,0x23},
{0x04,0x01},
{0x05,0x00},
{0x06,0xa4},

{0x11,0x2b},
{0x12,0x40},
{0x13,0x06},
{0x15,0x02},
{0x21,0x10},
{0x22,0x03},
{0x23,0x20},
{0x24,0x02},
{0x25,0x10},
{0x26,0x05},
{0x29,0x06},
{0x2a,0x0a},
{0x2b,0x04},
{0x10,0x90},
{0xfe,0x00},
{0xfe,0x00},
{0xb0,0x50},
{0x00,0x00},

//write Done
{0xff,0xff},
};


static const regval8_t GC1064_720p_table[] =
{

};

#if 0
static const regval8_t GC1064_suspend_table[] =
{

};

static const regval8_t GC1064_resume_table[] =
{

};
#endif

static const int GC1064__30fps_exp_time_gain_50Hz[GC1064_30FPS_50HZ_EXP_TIME_TOTAL][3] =
{ // {time, analog gain, digital gain}
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
{36, (int)(1.00*256), (int)(1.00*256)},
{38, (int)(1.00*256), (int)(1.00*256)},
{39, (int)(1.00*256), (int)(1.00*256)},
{40, (int)(1.00*256), (int)(1.00*256)},
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
{70, (int)(1.00*256), (int)(1.00*256)},
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
{114, (int)(1.00*256), (int)(1.00*256)},
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
{206, (int)(1.00*256), (int)(1.00*256)},
{214, (int)(1.00*256), (int)(1.00*256)},
{221, (int)(1.00*256), (int)(1.00*256)},
{229, (int)(1.00*256), (int)(1.00*256)},
{229, (int)(1.0625*256), (int)(1.00*256)},
{229, (int)(1.0625*256), (int)(1.00*256)},
{229, (int)(1.125*256), (int)(1.00*256)},
{229, (int)(1.125*256), (int)(1.00*256)},
{229, (int)(1.1875*256), (int)(1.00*256)},
{229, (int)(1.25*256), (int)(1.00*256)},
{229, (int)(1.25*256), (int)(1.00*256)},
{229, (int)(1.3125*256), (int)(1.00*256)},
{229, (int)(1.375*256), (int)(1.00*256)},
{229, (int)(1.4375*256), (int)(1.00*256)},
{229, (int)(1.4375*256), (int)(1.00*256)},
{229, (int)(1.5*256), (int)(1.00*256)},
{229, (int)(1.5625*256), (int)(1.00*256)},
{229, (int)(1.625*256), (int)(1.00*256)},
{229, (int)(1.6875*256), (int)(1.00*256)},
{229, (int)(1.75*256), (int)(1.00*256)},
{229, (int)(1.8125*256), (int)(1.00*256)},
{229, (int)(1.875*256), (int)(1.00*256)},
{229, (int)(1.9375*256), (int)(1.00*256)},
{458, (int)(1.00*256), (int)(1.00*256)},
{458, (int)(1.0625*256), (int)(1.00*256)},
{458, (int)(1.0625*256), (int)(1.00*256)},
{458, (int)(1.125*256), (int)(1.00*256)},
{458, (int)(1.125*256), (int)(1.00*256)},
{458, (int)(1.1875*256), (int)(1.00*256)},
{458, (int)(1.25*256), (int)(1.00*256)},
{458, (int)(1.25*256), (int)(1.00*256)},
{458, (int)(1.3125*256), (int)(1.00*256)},
{458, (int)(1.375*256), (int)(1.00*256)},
{458, (int)(1.4375*256), (int)(1.00*256)},
{458, (int)(1.4375*256), (int)(1.00*256)},
{458, (int)(1.5*256), (int)(1.00*256)},
{458, (int)(1.5625*256), (int)(1.00*256)},
{458, (int)(1.625*256), (int)(1.00*256)},
{458, (int)(1.6875*256), (int)(1.00*256)},
{458, (int)(1.75*256), (int)(1.00*256)},
{458, (int)(1.8125*256), (int)(1.00*256)},
{458, (int)(1.875*256), (int)(1.00*256)},
{458, (int)(1.9375*256), (int)(1.00*256)},
{686, (int)(1.3125*256), (int)(1.00*256)},
{686, (int)(1.375*256), (int)(1.00*256)},
{686, (int)(1.4375*256), (int)(1.00*256)},
{686, (int)(1.4375*256), (int)(1.00*256)},
{686, (int)(1.5*256), (int)(1.00*256)},
{686, (int)(1.5625*256), (int)(1.00*256)},
{686, (int)(1.625*256), (int)(1.00*256)},
{686, (int)(1.6875*256), (int)(1.00*256)},
{686, (int)(1.75*256), (int)(1.00*256)},
{686, (int)(1.8125*256), (int)(1.00*256)},
{686, (int)(1.875*256), (int)(1.00*256)},
{686, (int)(1.9375*256), (int)(1.00*256)},
{686, (int)(2.0*256), (int)(1.00*256)},
{686, (int)(2.07*256), (int)(1.00*256)},
{686, (int)(2.14*256), (int)(1.00*256)},
{686, (int)(2.22*256), (int)(1.00*256)},
{686, (int)(2.30*256), (int)(1.00*256)},
{686, (int)(2.38*256), (int)(1.00*256)},
{686, (int)(2.46*256), (int)(1.00*256)},
{686, (int)(2.55*256), (int)(1.00*256)},
{686, (int)(2.64*256), (int)(1.00*256)},
{686, (int)(2.73*256), (int)(1.00*256)},
{686, (int)(2.83*256), (int)(1.00*256)},
{686, (int)(2.93*256), (int)(1.00*256)},
{686, (int)(3.03*256), (int)(1.00*256)},
{686, (int)(3.14*256), (int)(1.00*256)},
{686, (int)(3.25*256), (int)(1.00*256)},
{686, (int)(3.36*256), (int)(1.00*256)},
{686, (int)(3.48*256), (int)(1.00*256)},
{686, (int)(3.61*256), (int)(1.00*256)},
{686, (int)(3.73*256), (int)(1.00*256)},
{686, (int)(3.86*256), (int)(1.00*256)},
{686, (int)(4.00*256), (int)(1.00*256)},
{686, (int)(4.14*256), (int)(1.00*256)},
{686, (int)(4.29*256), (int)(1.00*256)},
{686, (int)(4.44*256), (int)(1.00*256)},
{686, (int)(4.59*256), (int)(1.00*256)},
{686, (int)(4.76*256), (int)(1.00*256)},
{686, (int)(4.92*256), (int)(1.00*256)},
{686, (int)(5.10*256), (int)(1.00*256)},
{686, (int)(5.28*256), (int)(1.00*256)},
{686, (int)(5.46*256), (int)(1.00*256)},
{686, (int)(5.66*256), (int)(1.00*256)},
{686, (int)(5.86*256), (int)(1.00*256)},
{686, (int)(6.06*256), (int)(1.00*256)},
{686, (int)(6.28*256), (int)(1.00*256)},
{686, (int)(6.50*256), (int)(1.00*256)},
{686, (int)(6.73*256), (int)(1.00*256)},
{686, (int)(6.96*256), (int)(1.00*256)},
{686, (int)(7.21*256), (int)(1.00*256)},
{686, (int)(7.46*256), (int)(1.00*256)},
{686, (int)(7.73*256), (int)(1.00*256)},
{686, (int)(8.00*256), (int)(1.00*256)},
/*{686, (int)(8.28*256), (int)(1.00*256)},
{686, (int)(8.57*256), (int)(1.00*256)},
{686, (int)(8.88*256), (int)(1.00*256)},
{686, (int)(9.19*256), (int)(1.00*256)},
{686, (int)(9.51*256), (int)(1.00*256)},
{686, (int)(9.85*256), (int)(1.00*256)},
{686, (int)(10.20*256), (int)(1.00*256)},
{686, (int)(10.56*256), (int)(1.00*256)},
{686, (int)(10.93*256), (int)(1.00*256)},
{686, (int)(11.31*256), (int)(1.00*256)},
{686, (int)(11.71*256), (int)(1.00*256)},
{686, (int)(12.13*256), (int)(1.00*256)},
{686, (int)(12.55*256), (int)(1.00*256)},
{686, (int)(13.00*256), (int)(1.00*256)},
{686, (int)(13.45*256), (int)(1.00*256)},
{686, (int)(13.93*256), (int)(1.00*256)},
{686, (int)(14.42*256), (int)(1.00*256)},
{686, (int)(14.93*256), (int)(1.00*256)},
{686, (int)(15.45*256), (int)(1.00*256)},
{686, (int)(16.00*256), (int)(1.00*256)},
{686, (int)(16.56*256), (int)(1.00*256)},
{686, (int)(17.15*256), (int)(1.00*256)},
{686, (int)(17.75*256), (int)(1.00*256)},
{686, (int)(18.83*256), (int)(1.00*256)},
{686, (int)(19.03*256), (int)(1.00*256)},
{686, (int)(19.70*256), (int)(1.00*256)},
{686, (int)(20.39*256), (int)(1.00*256)},
{686, (int)(21.11*256), (int)(1.00*256)},
{686, (int)(21.86*256), (int)(1.00*256)},
{686, (int)(22.63*256), (int)(1.00*256)},
{686, (int)(23.43*256), (int)(1.00*256)},
{686, (int)(24.25*256), (int)(1.00*256)},
{686, (int)(25.11*256), (int)(1.00*256)},
{686, (int)(25.99*256), (int)(1.00*256)},
{686, (int)(26.91*256), (int)(1.00*256)},
{686, (int)(27.86*256), (int)(1.00*256)},
{686, (int)(28.84*256), (int)(1.00*256)},
{686, (int)(29.86*256), (int)(1.00*256)},
{686, (int)(30.91*256), (int)(1.00*256)},
{686, (int)(32.00*256), (int)(1.00*256)},*/
};//50Hz

static const  int GC1064_30fps_exp_time_gain_60Hz[GC1064_30FPS_60HZ_EXP_TIME_TOTAL][3] =
{// {time, analog gain, digital gain}
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
{8, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{9, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{10, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
{11, (int)(1.00*256), (int)(1.00*256)},
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
{17, (int)(1.00*256), (int)(1.00*256)},
{18, (int)(1.00*256), (int)(1.00*256)},
{19, (int)(1.00*256), (int)(1.00*256)},
{19, (int)(1.00*256), (int)(1.00*256)},
{20, (int)(1.00*256), (int)(1.00*256)},
{21, (int)(1.00*256), (int)(1.00*256)},
{21, (int)(1.00*256), (int)(1.00*256)},
{22, (int)(1.00*256), (int)(1.00*256)},
{23, (int)(1.00*256), (int)(1.00*256)},
{24, (int)(1.00*256), (int)(1.00*256)},
{25, (int)(1.00*256), (int)(1.00*256)},
{25, (int)(1.00*256), (int)(1.00*256)},
{26, (int)(1.00*256), (int)(1.00*256)},
{27, (int)(1.00*256), (int)(1.00*256)},
{28, (int)(1.00*256), (int)(1.00*256)},
{29, (int)(1.00*256), (int)(1.00*256)},
{30, (int)(1.00*256), (int)(1.00*256)},
{31, (int)(1.00*256), (int)(1.00*256)},
{32, (int)(1.00*256), (int)(1.00*256)},
{34, (int)(1.00*256), (int)(1.00*256)},
{35, (int)(1.00*256), (int)(1.00*256)},
{36, (int)(1.00*256), (int)(1.00*256)},
{37, (int)(1.00*256), (int)(1.00*256)},
{39, (int)(1.00*256), (int)(1.00*256)},
{40, (int)(1.00*256), (int)(1.00*256)},
{41, (int)(1.00*256), (int)(1.00*256)},
{43, (int)(1.00*256), (int)(1.00*256)},
{44, (int)(1.00*256), (int)(1.00*256)},
{46, (int)(1.00*256), (int)(1.00*256)},
{47, (int)(1.00*256), (int)(1.00*256)},
{49, (int)(1.00*256), (int)(1.00*256)},
{51, (int)(1.00*256), (int)(1.00*256)},
{53, (int)(1.00*256), (int)(1.00*256)},
{55, (int)(1.00*256), (int)(1.00*256)},
{56, (int)(1.00*256), (int)(1.00*256)},
{58, (int)(1.00*256), (int)(1.00*256)},
{61, (int)(1.00*256), (int)(1.00*256)},
{63, (int)(1.00*256), (int)(1.00*256)},
{65, (int)(1.00*256), (int)(1.00*256)},
{67, (int)(1.00*256), (int)(1.00*256)},
{70, (int)(1.00*256), (int)(1.00*256)},
{72, (int)(1.00*256), (int)(1.00*256)},
{75, (int)(1.00*256), (int)(1.00*256)},
{77, (int)(1.00*256), (int)(1.00*256)},
{80, (int)(1.00*256), (int)(1.00*256)},
{83, (int)(1.00*256), (int)(1.00*256)},
{86, (int)(1.00*256), (int)(1.00*256)},
{89, (int)(1.00*256), (int)(1.00*256)},
{92, (int)(1.00*256), (int)(1.00*256)},
{95, (int)(1.00*256), (int)(1.00*256)},
{98, (int)(1.00*256), (int)(1.00*256)},
{102, (int)(1.00*256), (int)(1.00*256)},
{105, (int)(1.00*256), (int)(1.00*256)},
{109, (int)(1.00*256), (int)(1.00*256)},
{113, (int)(1.00*256), (int)(1.00*256)},
{117, (int)(1.00*256), (int)(1.00*256)},
{121, (int)(1.00*256), (int)(1.00*256)},
{125, (int)(1.00*256), (int)(1.00*256)},
{130, (int)(1.00*256), (int)(1.00*256)},
{134, (int)(1.00*256), (int)(1.00*256)},
{139, (int)(1.00*256), (int)(1.00*256)},
{144, (int)(1.00*256), (int)(1.00*256)},
{149, (int)(1.00*256), (int)(1.00*256)},
{154, (int)(1.00*256), (int)(1.00*256)},
{160, (int)(1.00*256), (int)(1.00*256)},
{165, (int)(1.00*256), (int)(1.00*256)},
{171, (int)(1.00*256), (int)(1.00*256)},
{177, (int)(1.00*256), (int)(1.00*256)},
{184, (int)(1.00*256), (int)(1.00*256)},
{190, (int)(1.00*256), (int)(1.00*256)},
{190, (int)(1.0625*256), (int)(1.00*256)},
{190, (int)(1.0625*256), (int)(1.00*256)},
{190, (int)(1.125*256), (int)(1.00*256)},
{190, (int)(1.125*256), (int)(1.00*256)},
{190, (int)(1.1875*256), (int)(1.00*256)},
{190, (int)(1.25*256), (int)(1.00*256)},
{190, (int)(1.25*256), (int)(1.00*256)},
{190, (int)(1.3125*256), (int)(1.00*256)},
{190, (int)(1.375*256), (int)(1.00*256)},
{190, (int)(1.4375*256), (int)(1.00*256)},
{190, (int)(1.4375*256), (int)(1.00*256)},
{190, (int)(1.5*256), (int)(1.00*256)},
{190, (int)(1.5625*256), (int)(1.00*256)},
{190, (int)(1.625*256), (int)(1.00*256)},
{190, (int)(1.6875*256), (int)(1.00*256)},
{190, (int)(1.75*256), (int)(1.00*256)},
{190, (int)(1.8125*256), (int)(1.00*256)},
{190, (int)(1.875*256), (int)(1.00*256)},
{190, (int)(1.9375*256), (int)(1.00*256)},
{381, (int)(1.00*256), (int)(1.00*256)},
{380, (int)(1.0625*256), (int)(1.00*256)},
{380, (int)(1.0625*256), (int)(1.00*256)},
{380, (int)(1.125*256), (int)(1.00*256)},
{380, (int)(1.125*256), (int)(1.00*256)},
{380, (int)(1.1875*256), (int)(1.00*256)},
{380, (int)(1.25*256), (int)(1.00*256)},
{380, (int)(1.25*256), (int)(1.00*256)},
{380, (int)(1.3125*256), (int)(1.00*256)},
{380, (int)(1.375*256), (int)(1.00*256)},
{380, (int)(1.4375*256), (int)(1.00*256)},
{380, (int)(1.4375*256), (int)(1.00*256)},
{380, (int)(1.5*256), (int)(1.00*256)},
{380, (int)(1.5625*256), (int)(1.00*256)},
{380, (int)(1.625*256), (int)(1.00*256)},
{380, (int)(1.6875*256), (int)(1.00*256)},
{380, (int)(1.75*256), (int)(1.00*256)},
{380, (int)(1.8125*256), (int)(1.00*256)},
{380, (int)(1.875*256), (int)(1.00*256)},
{380, (int)(1.9375*256), (int)(1.00*256)},
{571, (int)(1.3125*256), (int)(1.00*256)},
{571, (int)(1.375*256), (int)(1.00*256)},
{571, (int)(1.4375*256), (int)(1.00*256)},
{571, (int)(1.4375*256), (int)(1.00*256)},
{571, (int)(1.5*256), (int)(1.00*256)},
{571, (int)(1.5625*256), (int)(1.00*256)},
{571, (int)(1.625*256), (int)(1.00*256)},
{571, (int)(1.6875*256), (int)(1.00*256)},
{571, (int)(1.75*256), (int)(1.00*256)},
{571, (int)(1.8125*256), (int)(1.00*256)},
{571, (int)(1.875*256), (int)(1.00*256)},
{571, (int)(1.9375*256), (int)(1.00*256)},
{762, (int)(1.5*256), (int)(1.00*256)},
{762, (int)(1.5625*256), (int)(1.00*256)},
{762, (int)(1.625*256), (int)(1.00*256)},
{762, (int)(1.6875*256), (int)(1.00*256)},
{762, (int)(1.75*256), (int)(1.00*256)},
{762, (int)(1.8125*256), (int)(1.00*256)},
{762, (int)(1.875*256), (int)(1.00*256)},
{762, (int)(1.9375*256), (int)(1.00*256)},
{762, (int)(2.0*256), (int)(1.00*256)},
{762, (int)(2.07*256), (int)(1.00*256)},
{762, (int)(2.14*256), (int)(1.00*256)},
{762, (int)(2.22*256), (int)(1.00*256)},
{762, (int)(2.30*256), (int)(1.00*256)},
{762, (int)(2.38*256), (int)(1.00*256)},
{762, (int)(2.46*256), (int)(1.00*256)},
{762, (int)(2.55*256), (int)(1.00*256)},
{762, (int)(2.64*256), (int)(1.00*256)},
{762, (int)(2.73*256), (int)(1.00*256)},
{762, (int)(2.83*256), (int)(1.00*256)},
{762, (int)(2.93*256), (int)(1.00*256)},
{762, (int)(3.03*256), (int)(1.00*256)},
{762, (int)(3.14*256), (int)(1.00*256)},
{762, (int)(3.25*256), (int)(1.00*256)},
{762, (int)(3.36*256), (int)(1.00*256)},
{762, (int)(3.48*256), (int)(1.00*256)},
{762, (int)(3.61*256), (int)(1.00*256)},
{762, (int)(3.73*256), (int)(1.00*256)},
{762, (int)(3.86*256), (int)(1.00*256)},
{762, (int)(4.00*256), (int)(1.00*256)},
{762, (int)(4.14*256), (int)(1.00*256)},
{762, (int)(4.29*256), (int)(1.00*256)},
{762, (int)(4.44*256), (int)(1.00*256)},
{762, (int)(4.59*256), (int)(1.00*256)},
{762, (int)(4.76*256), (int)(1.00*256)},
{762, (int)(4.92*256), (int)(1.00*256)},
{762, (int)(5.10*256), (int)(1.00*256)},
{762, (int)(5.28*256), (int)(1.00*256)},
{762, (int)(5.46*256), (int)(1.00*256)},
{762, (int)(5.66*256), (int)(1.00*256)},
{762, (int)(5.86*256), (int)(1.00*256)},
{762, (int)(6.06*256), (int)(1.00*256)},
{762, (int)(6.28*256), (int)(1.00*256)},
{762, (int)(6.50*256), (int)(1.00*256)},
{762, (int)(6.73*256), (int)(1.00*256)},
{762, (int)(6.96*256), (int)(1.00*256)},
{762, (int)(7.21*256), (int)(1.00*256)},
{762, (int)(7.46*256), (int)(1.00*256)},
{762, (int)(7.73*256), (int)(1.00*256)},
{762, (int)(8.00*256), (int)(1.00*256)},
/*{762, (int)(8.28*256), (int)(1.00*256)},
{762, (int)(8.57*256), (int)(1.00*256)},
{762, (int)(8.88*256), (int)(1.00*256)},
{762, (int)(9.19*256), (int)(1.00*256)},
{762, (int)(9.51*256), (int)(1.00*256)},
{762, (int)(9.85*256), (int)(1.00*256)},
{762, (int)(10.20*256), (int)(1.00*256)},
{762, (int)(10.56*256), (int)(1.00*256)},
{762, (int)(10.93*256), (int)(1.00*256)},
{762, (int)(11.31*256), (int)(1.00*256)},
{762, (int)(11.71*256), (int)(1.00*256)},
{762, (int)(12.13*256), (int)(1.00*256)},
{762, (int)(12.55*256), (int)(1.00*256)},
{762, (int)(13.00*256), (int)(1.00*256)},
{762, (int)(13.45*256), (int)(1.00*256)},
{762, (int)(13.93*256), (int)(1.00*256)},
{762, (int)(14.42*256), (int)(1.00*256)},
{762, (int)(14.93*256), (int)(1.00*256)},
{762, (int)(15.45*256), (int)(1.00*256)},
{762, (int)(16.00*256), (int)(1.00*256)},
{762, (int)(16.56*256), (int)(1.00*256)},
{762, (int)(17.15*256), (int)(1.00*256)},
{762, (int)(17.75*256), (int)(1.00*256)},
{762, (int)(18.83*256), (int)(1.00*256)},
{762, (int)(19.03*256), (int)(1.00*256)},
{762, (int)(19.70*256), (int)(1.00*256)},
{762, (int)(20.39*256), (int)(1.00*256)},
{762, (int)(21.11*256), (int)(1.00*256)},
{762, (int)(21.86*256), (int)(1.00*256)},
{762, (int)(22.63*256), (int)(1.00*256)},
{762, (int)(23.43*256), (int)(1.00*256)},
{762, (int)(24.25*256), (int)(1.00*256)},
{762, (int)(25.11*256), (int)(1.00*256)},
{762, (int)(25.99*256), (int)(1.00*256)},
{762, (int)(26.91*256), (int)(1.00*256)},
{762, (int)(27.86*256), (int)(1.00*256)},
{762, (int)(28.84*256), (int)(1.00*256)},
{762, (int)(29.86*256), (int)(1.00*256)},
{762, (int)(30.91*256), (int)(1.00*256)},
{762, (int)(32.00*256), (int)(1.00*256)},*/
};

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S GC1064_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	GC1064_handle = drv_l2_sccb_open(GC1064_ID, 8, 8);
	if(GC1064_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	GC1064_handle.devNumber = I2C_0;
	GC1064_handle.slaveAddr = GC1064_ID;
	GC1064_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void GC1064_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(GC1064_handle) {
		drv_l2_sccb_close(GC1064_handle);
		GC1064_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	GC1064_handle.slaveAddr = 0;
	GC1064_handle.clkRate = 0;
#endif
}

static INT32S GC1064_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(GC1064_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&GC1064_handle, data, 3);
#endif
}


static INT32S GC1064_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(GC1064_handle, reg, &data) >= 0) {
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
	if(drv_l1_i2c_bus_write(&GC1064_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&GC1064_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}

static INT32S GC1064_sccb_write_table(regval8_t *pTable)
{
	while(1) {
		if(pTable->reg_num == 0x00 && pTable->value == 0x00) {
			break;
		}

		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(GC1064_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
		if(pTable->reg_num == 0x10 && pTable->value == 0x80) {
			osDelay(200);	//wait 2ms
		}
		pTable++;
	}
	DBG_PRINT("\r\n\r\n");

	return STATUS_OK;
}

static INT32S GC1064_sccb_read_table(regval8_t *pTable)
{
    //INT8U *Rdata;
    INT8U Rdata;
	while(1) {
		if(pTable->reg_num == 0x00 && pTable->value == 0x00) {
			break;
		}

		if(GC1064_sccb_read(pTable->reg_num, &Rdata) < 0) {
			DBG_PRINT("sccb read fail.\r\n");
			continue;
		}
		DBG_PRINT("0x%02x, 0x%02x = 0x%02x\r\n", pTable->reg_num, pTable->value, Rdata);
		pTable++;
	}
	DBG_PRINT("\r\n\r\n");

	return STATUS_OK;
}

/*****************************************************************************************
+++++++++++++++++AEC/AGC
*****************************************************************************************/
static int GC1064_cvt_agc_gain_kehu(int agc_gain)
{

	INT32U Analog_Multiple[9]={1000, 1400, 1800, 2560, 3400,4800,6840,9400,13200};

	INT32U Analog_Index;
	INT32U Digital_Gain;
	INT32U Decimal;
	INT32U ret;


	GC1064_analog_gain = agc_gain*10;

    if(GC1064_analog_gain>7200)   GC1064_analog_gain=7200; // max_gain=7.2
    //if(GC1064_analog_gain>5200)   GC1064_analog_gain=5200; // max_gain=5.2

	Analog_Index=0;

	while(Analog_Index<9)
	{
	  if(GC1064_analog_gain<Analog_Multiple[Analog_Index])
	  {
		break;
	  }
	  else
	  {
		Analog_Index++;
	  }
	}

	Digital_Gain = GC1064_analog_gain*1000/Analog_Multiple[Analog_Index-1];
	Decimal=(Digital_Gain*64)/1000;

	if(GC1064_analog_gain>4000)
	{
      ret = GC1064_sccb_write(0x66,  0x1c);
      if(ret < 0) return ret;
	}
	if(GC1064_analog_gain<3250)
    {
      ret = GC1064_sccb_write(0x66,  0x20);
      if(ret < 0) return ret;
	}


#if 1
	ret = GC1064_sccb_write(0xb1,  Decimal>>6);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb2,  (Decimal<<2)&0xfc);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb6,   Analog_Index-1);
	if(ret < 0) return ret;
#else
	ret = GC1064_sccb_write(0xb1, 0x01);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb2, 0x00);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb6, 0x06);
	if(ret < 0) return ret;
#endif

}

static int GC1064_cvt_agc_gain(int agc_gain)
{

  INT32U Analog_Multiple[11]={1000, 1650, 1870, 3080, 3500,5820,6700,10700,15800,21400,30800};

	INT32U Analog_Index;
	INT32U Digital_Gain;
	INT32U Decimal;
	INT32U ret;


	GC1064_analog_gain = agc_gain*10;
	Analog_Index=0;

	while(Analog_Index<11)
	{
	  if(GC1064_analog_gain<Analog_Multiple[Analog_Index])
	  {
		break;
	  }
	  else
	  {
		Analog_Index++;
	  }
	}

	Digital_Gain = GC1064_analog_gain*1000/Analog_Multiple[Analog_Index-1];
	Decimal=(Digital_Gain*64)/1000;

#if 1
	ret = GC1064_sccb_write(0xb1,  Decimal>>6);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb2,  (Decimal<<2)&0xfc);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb6,   Analog_Index-1);
	if(ret < 0) return ret;
#else
	ret = GC1064_sccb_write(0xb1, 0x01);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb2, 0x00);
	if(ret < 0) return ret;
	ret = GC1064_sccb_write(0xb6, 0x06);
	if(ret < 0) return ret;
#endif

}


INT8S g_ex_idx_offset = 0;

static int GC1064_set_xfps_exposure_time(sensor_exposure_t *si)
{
	unsigned char t1, t2;
	int idx, temp, ret;

	//DBG_PRINT("%s:%d\n", __FUNCTION__, __LINE__);
	si->sensor_ev_idx += si->ae_ev_step;
	if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
	if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;
	//idx = 150;
#if RAW_MODE
	idx = (g_raw_mode_idx < 0) ? si->sensor_ev_idx * 3 : g_raw_mode_idx * 3;
	DBG_PRINT("%s:%d %d\n", __FUNCTION__, __LINE__, g_raw_mode_idx);
#else
	idx = (si->sensor_ev_idx+g_ex_idx_offset) * 3;
#endif
	si ->time = p_expTime_table[idx];
	si ->analog_gain = (p_expTime_table[idx+1]*3)>>1;
	//si ->digital_gain = p_expTime_table[idx+2];
    //DBG_PRINT("[EV=%d, offset=%d]: time = %d, analog gain =0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);
    //crf
    // exposure time
	if(si ->time != pre_sensor_time)
	{
		pre_sensor_time = si->time;
		temp = si->time;

		t1 = (temp & 0xff);
		t2 = (temp >> 8) & 0x00ff;

        ret = GC1064_sccb_write(0x04, t1);
        if(ret < 0) return ret;
        ret = GC1064_sccb_write(0x03, t2);
		if(ret < 0) {
		    DBG_PRINT("ERR: write sensor time = 0x%x!!!\r\n", temp);
            return ret;
		}
	}

    //gain
    if(si ->analog_gain != pre_sensor_a_gain)
	{
		// gain
		pre_sensor_a_gain = si->analog_gain;

		temp = si->analog_gain;
		//ret = GC1064_cvt_agc_gain(temp);
		ret = GC1064_cvt_agc_gain(temp);
		if(ret < 0) {
		    DBG_PRINT("ERR: write sensor analog agin = 0x%x!!!\r\n", temp);
            return ret;
		}
	}

    return 0;
}


static void GC1064_set_ae(int ev_step)
{
    seInfo.ae_ev_step = ev_step;
    GC1064_set_xfps_exposure_time(&seInfo);
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)GC1064_set_ae;
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
static void mipi_GC1064_handle(INT32U event)
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
void GC1064_seinfo_init(void)
{
    seInfo.sensor_ev_idx = GC1064_30FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= GC1064_30FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= GC1064_30FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = GC1064_30FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = GC1064_30FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)GC1064__30fps_exp_time_gain_50Hz;

	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x00;

}

static void GC1064_mipi_sensor_init(void)
{
	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);
    #if MIPI_DEV_NO == 0
	/* set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk*/
	//R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<1));//0x02400042;
	/* set Mclk(CSI_CLKO) to IOD7, for MIPI Mclk*/
	R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	#else
    // set Mclk(CSI_CLKO) to IOD7, for MIPI Mclk
	R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<1));
	#endif

    //GC1064_set_favorite();
	// mclk output
	//drv_l2_sensor_set_mclkout(GC1064_cdsp_mipi_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);

	// reguest sccb
	GC1064_sccb_open();

	// reset sensor
	GC1064_sccb_write_table((regval8_t *)GC1064_reset_table);
    osDelay(200);

	// init sensor
	GC1064_sccb_write_table((regval8_t *)GC1064_init_table);

	//GC1064_sccb_read_table((regval8_t *)GC1064_init_table);

    DBG_PRINT("Sensor GC1064 mipi init completed\r\n");
}


/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
//static void GC1064_cdsp_mipi_init(void)
void GC1064_cdsp_mipi_init(void)
{
	//ae init
	GC1064_seinfo_init();
    //cdsp init
	drv_l2_cdsp_open();
	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

	// Reset
	gpio_set_port_attribute(IO_A14, ATTRIBUTE_HIGH);
	gpio_init_io(IO_A14, GPIO_OUTPUT);
	gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);
    gpio_write_io(IO_A14, DATA_LOW);
    osDelay(2000);
    gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(1000);


	// mipi enable
	#if (MIPI_DEV_NO == 1)
		drv_l1_mipi_init(MIPI_1,ENABLE);
	#else
	    drv_l1_mipi_init(MIPI_0,ENABLE);
	#endif

    DBG_PRINT("Sensor GC1064 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void GC1064_cdsp_mipi_uninit(void)
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
	GC1064_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void GC1064_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);
	drv_l2_sensor_set_mclkout(MCLK_24M);

/*	//Only 720p
    switch(index)
	{
	case 0:
        GC1064_sccb_write_table((regval16_t *)GC1064_scale_vga_table);
		break;

	case 1:
		GC1064_sccb_write_table((regval16_t *)GC1064_720p_table);
		break;

	default:
		while(1);
	}
*/
    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 4);    //u2.added.20151207
    //osDelay(200);

#if 1   // change mclk
	drv_l2_sensor_set_mclkout(GC1064_cdsp_mipi_ops.info[index].mclk);
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
	format.input_format =  GC1064_cdsp_mipi_ops.info[index].input_format;
	format.output_format = GC1064_cdsp_mipi_ops.info[index].output_format;
	target_w = GC1064_cdsp_mipi_ops.info[index].target_w;
	target_h = GC1064_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = GC1064_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = GC1064_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = GC1064_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = GC1064_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = GC1064_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = GC1064_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = GC1064_cdsp_mipi_ops.info[index].vsync_active;


    if(drv_l2_cdsp_set_fmt(&format) < 0)
    {
		DBG_PRINT("cdsp set fmt err!!!\r\n");
	}

	// set scale down
	if((format.hpixel > target_w) || (format.vline > target_h)) {
		drv_l2_cdsp_set_yuv_scale(target_w, target_h);
	}

    // cdsp start
	drv_l2_CdspTableRegister((gpCisCali_t*)&g_cali);
	drv_l2_cdsp_stream_on(ENABLE, bufA, bufB);
	drv_l2_cdsp_enable(&g_FavTable, sensor_w, sensor_h, target_w, target_h);


	// set mipi format.
#if 1
    //switch(format.input_format)
    switch(3)
	{
	case 0:
	case 1:
		GC1064_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		GC1064_mipi_cfg.data_type = MIPI_RAW8;
		//GC1064_mipi_cfg.h_back_porch = 4;
		break;

	case 2:
	case 3:
		GC1064_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		GC1064_mipi_cfg.data_type = MIPI_RAW10;
		//GC1064_mipi_cfg.h_back_porch = 2;//4;
        GC1064_mipi_cfg.pixel_clk_sel = 0;
		break;

	default:
    	GC1064_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		GC1064_mipi_cfg.data_type = MIPI_RAW8;
		//GC1064_mipi_cfg.h_back_porch = 4;
	}
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      if (drv_l1_mipi_set_parameter(MIPI_1, &GC1064_mipi_cfg) < 0)
      {
        DBG_PRINT("MIPI1 init fail !!!\r\n");
      }
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_GC1064_handle);
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      if (drv_l1_mipi_set_parameter(MIPI_0, &GC1064_mipi_cfg) < 0)
      {
        DBG_PRINT("MIPI0 init fail !!!\r\n");
      }
      else
      {
        DBG_PRINT("MIPI0 init completed.\r\n");
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_GC1064_handle);
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
      }
    #endif

	//Enable MCLK & Init Sensor
    GC1064_mipi_sensor_init();


    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    GC1064_set_xfps_exposure_time(&seInfo);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void GC1064_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* GC1064_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&GC1064_cdsp_mipi_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t GC1064_cdsp_mipi_ops =
{
	SENSOR_GC1064_CDSP_MIPI_NAME,		/* sensor name */
	GC1064_cdsp_mipi_init,
	GC1064_cdsp_mipi_uninit,
	GC1064_cdsp_mipi_stream_on,
	GC1064_cdsp_mipi_stream_off,
	GC1064_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SRGGB10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			320,							/* target width */
			240,  						/* target height */
			GC1064_WIDTH,				/* sensor width */
			GC1064_HEIGHT,  				/* sensor height */
			#if SENSOR_FLIP
			11,							/* sensor h offset */
			1,							/* sensor v offset */
			#else
			4,//8,
			0,
			#endif
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 2st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SRGGB10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			640,			        			/* target width */
			480, 			    			/* target height */
			GC1064_WIDTH,				/* sensor width */
			GC1064_HEIGHT, 				/* sensor height */
			#if SENSOR_FLIP
			11,							/* sensor h offset */
			1,							/* sensor v offset */
			#else
			4,//8,
			0,
			#endif
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 3st info */
		{
			MCLK_24M,					/* MCLK */
			//V4L2_PIX_FMT_SBGGR10,		/* input format */
           		//V4L2_PIX_FMT_SGRBG10,//0
             		V4L2_PIX_FMT_SRGGB10,//1 //10
            		//V4L2_PIX_FMT_SBGGR10,//2
           		//V4L2_PIX_FMT_SGBRG10,//3
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			GC1064_OUT_WIDTH,			/* target width */
			GC1064_OUT_HEIGHT,  		/* target height */
			GC1064_WIDTH,				/* sensor width */
			GC1064_HEIGHT,  				/* sensor height */
			#if SENSOR_FLIP
			11,							/* sensor h offset */
			1,							/* sensor v offset */
			#else
			2,//4,//8,
			0,
			#endif
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 4st info */
		{
			MCLK_24M,					/* MCLK */
            		V4L2_PIX_FMT_SRGGB10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			GC1064_OUT_WIDTH,			/* target width */
			GC1064_OUT_HEIGHT,  		/* target height */
			GC1064_WIDTH,				/* sensor width */
			GC1064_HEIGHT, 				/* sensor height */
			#if SENSOR_FLIP
			8,							/* sensor h offset */
			0,							/* sensor v offset */
			#else
			8,//4,
			0,
			#endif
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_LOW,			/* hsync pin active level */
			MODE_ACTIVE_LOW,			/* vsync pin active level */
		},
	}
};
#endif //(defined _SENSOR_GC1064_CDSP_MIPI) && (_SENSOR_GC1064_CDSP_MIPI == 1)
