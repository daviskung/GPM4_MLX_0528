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


#if (defined _SENSOR_H42_CDSP_MIPI) && (_SENSOR_H42_CDSP_MIPI == 1)

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

#define	H42_ID							0x60	//0x64,0x68,0x6C
#define H42_WIDTH						1280
#define H42_HEIGHT						720 + 2
#define H42_OUT_WIDTH					1280
#define H42_OUT_HEIGHT					720

#ifndef DISABLE
#define DISABLE     					0
#endif

#ifndef ENABLE
#define ENABLE      					1
#endif



#define H42_30FPS_50HZ_DAY_EV_IDX 			140
#define H42_30FPS_50HZ_NIGHT_EV_IDX			190
#define H42_30FPS_50HZ_EXP_TIME_TOTAL		209
#define H42_30FPS_50HZ_INIT_EV_IDX 			((H42_30FPS_50HZ_DAY_EV_IDX + H42_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define H42_30FPS_50HZ_MAX_EV_IDX			    (H42_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define H42_30FPS_60HZ_DAY_EV_IDX 			137
#define H42_30FPS_60HZ_NIGHT_EV_IDX			190
#define H42_30FPS_60HZ_EXP_TIME_TOTAL		215
#define H42_30FPS_60HZ_INIT_EV_IDX 			((H42_30FPS_60HZ_DAY_EV_IDX + H42_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define H42_30FPS_60HZ_MAX_EV_IDX			 (H42_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define H42_24FPS_50HZ_DAY_EV_IDX 			138
#define H42_24FPS_50HZ_NIGHT_EV_IDX			194
#define H42_24FPS_50HZ_EXP_TIME_TOTAL		254
#define H42_24FPS_50HZ_INIT_EV_IDX 			((H42_24FPS_50HZ_DAY_EV_IDX + H42_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define H42_24FPS_50HZ_MAX_EV_IDX			 (H42_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define H42_24FPS_60HZ_DAY_EV_IDX 			135
#define H42_24FPS_60HZ_NIGHT_EV_IDX			195
#define H42_24FPS_60HZ_EXP_TIME_TOTAL		255
#define H42_24FPS_60HZ_INIT_EV_IDX 			((H42_24FPS_60HZ_DAY_EV_IDX + H42_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define H42_24FPS_60HZ_MAX_EV_IDX			(H42_24FPS_60HZ_EXP_TIME_TOTAL - 10)


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

static INT8U ae_flag = 0;

#if SCCB_MODE == SCCB_GPIO
	static void *h42_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t h42_handle;
#endif

static mipi_config_t h42_mipi_cfg =
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
	0,//NULL,				/* RSD 2 */

	H42_WIDTH,		/* width, 0~0xFFFF */
	H42_HEIGHT,		/* height, 0~0xFFFF */
	2, 					/* back porch, 0~0xF */ //If Set 4 have show paper color at left side.u2.20160119
	4,					/* front porch, 0~0xF */
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,//NULL,				/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	190,//250,				/* data mask, unit is ns */
	MIPI_CHECK_LP_00//MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};

static const regval8_t h42_reset_table[] =
{
	{ 0x12, 0x80 },
	{ 0xFF, 0xFF },
};

static const regval8_t h42_init_table[] =
{
	//INI Start
	{0x12,0x40},
	//DVP Setting
	{0x0D,0x40},
	{0x1F,0x04},
	//PLL Setting
	{0x0E,0x1d},
	{0x0F,0x09},
	{0x10,0x1E},
	{0x11,0x80},
	//Frame/Window
	#if 1 	//30fps
	{0x20,0x40},
	{0x21,0x06},
	{0x22,0xEE},
	{0x23,0x02},
	#else 	// 26 fps
	{0x20,0xD8},
	{0x21,0x06},
	{0x22,0x13},
	{0x23,0x03},
	#endif
	{0x24,0x04}, //w
	{0x25,0xD4}, //h
	{0x26,0x25},
	{0x27,0x47},//49}
	{0x28,0x0D},
	{0x29,0x01},
	{0x2A,0x24},
	{0x2B,0x29},
	{0x2C,0x00},
	{0x2D,0x00},
	{0x2E,0xB9},
	{0x2F,0x00},
	//Sensor Timing
	{0x30,0x92},
	{0x31,0x0A},
	{0x32,0xAA},
	{0x33,0x14},
	{0x34,0x38},
	{0x35,0x54},
	{0x42,0x41},
	{0x43,0x50},
	//Interface
	{0x1D,0x00},
	{0x1E,0x00},
	{0x6C,0x50},
	{0x73,0x33},
	{0x70,0x68},
	{0x76,0x40},
	{0x77,0x06},
	{0x78,0x18},
	//Array/AnADC/PWC
	{0x48,0x40},
	{0x60,0xA4},
	{0x61,0xFF},
	{0x62,0x40},
	{0x65,0x00},
	{0x66,0x20},
	{0x67,0x30},
	{0x68,0x04},
	{0x69,0x74},
	{0x6F,0x04},
	// for black sun
	{0x63,0x19},
	{0x6a,0x09},
	//AE/AG/ABLC
	{0x13,0x87},
	{0x14,0x80},
	{0x16,0xC0},
	{0x17,0x40},
	{0x18,0xBB},
	{0x19,0x20},		//Gain delay
	{0x38,0x35},
	{0x39,0x98},
	{0x4a,0x03},
	{0x49,0x10},
	//INI End
	#if SENSOR_FLIP
	{0x12,0x30},
	#else
	{0x12,0x00},
	#endif
	//PWDN Setting
    //write Done
	{0xff,0xff}
};


static const regval8_t h42_720p_table[] =
{

};

#if 0
static const regval8_t h42_suspend_table[] =
{

};

static const regval8_t h42_resume_table[] =
{

};
#endif

static const int jxh42_30fps_exp_time_gain_50Hz[H42_30FPS_50HZ_EXP_TIME_TOTAL][3] =
{ // {time, analog gain, digital gain}
{8	,(int)(1.00*16), (int)(1.00*16)},
{8	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{11	,(int)(1.00*16), (int)(1.00*16)},
{11	,(int)(1.00*16), (int)(1.00*16)},//10
{11	,(int)(1.00*16), (int)(1.00*16)},
{12	,(int)(1.00*16), (int)(1.00*16)},
{12	,(int)(1.00*16), (int)(1.00*16)},
{13	,(int)(1.00*16), (int)(1.00*16)},
{13	,(int)(1.00*16), (int)(1.00*16)},
{14	,(int)(1.00*16), (int)(1.00*16)},
{14	,(int)(1.00*16), (int)(1.00*16)},
{15	,(int)(1.00*16), (int)(1.00*16)},
{15	,(int)(1.00*16), (int)(1.00*16)},
{16	,(int)(1.00*16), (int)(1.00*16)},//20
{16	,(int)(1.00*16), (int)(1.00*16)},
{17	,(int)(1.00*16), (int)(1.00*16)},
{17	,(int)(1.00*16), (int)(1.00*16)},
{18	,(int)(1.00*16), (int)(1.00*16)},
{19	,(int)(1.00*16), (int)(1.00*16)},
{19	,(int)(1.00*16), (int)(1.00*16)},
{20	,(int)(1.00*16), (int)(1.00*16)},
{21	,(int)(1.00*16), (int)(1.00*16)},
{21	,(int)(1.00*16), (int)(1.00*16)},
{22	,(int)(1.00*16), (int)(1.00*16)},//30
{23	,(int)(1.00*16), (int)(1.00*16)},
{24	,(int)(1.00*16), (int)(1.00*16)},
{24	,(int)(1.00*16), (int)(1.00*16)},
{25	,(int)(1.00*16), (int)(1.00*16)},
{26	,(int)(1.00*16), (int)(1.00*16)},
{27	,(int)(1.00*16), (int)(1.00*16)},
{28	,(int)(1.00*16), (int)(1.00*16)},
{29	,(int)(1.00*16), (int)(1.00*16)},
{30	,(int)(1.00*16), (int)(1.00*16)},
{31	,(int)(1.00*16), (int)(1.00*16)},//40
{32	,(int)(1.00*16), (int)(1.00*16)},
{33	,(int)(1.00*16), (int)(1.00*16)},
{35	,(int)(1.00*16), (int)(1.00*16)},
{36	,(int)(1.00*16), (int)(1.00*16)},
{37	,(int)(1.00*16), (int)(1.00*16)},
{38	,(int)(1.00*16), (int)(1.00*16)},
{40	,(int)(1.00*16), (int)(1.00*16)},
{41	,(int)(1.00*16), (int)(1.00*16)},
{43	,(int)(1.00*16), (int)(1.00*16)},
{44	,(int)(1.00*16), (int)(1.00*16)},//50
{46	,(int)(1.00*16), (int)(1.00*16)},
{47	,(int)(1.00*16), (int)(1.00*16)},
{49	,(int)(1.00*16), (int)(1.00*16)},
{51	,(int)(1.00*16), (int)(1.00*16)},
{52	,(int)(1.00*16), (int)(1.00*16)},
{54	,(int)(1.00*16), (int)(1.00*16)},
{56	,(int)(1.00*16), (int)(1.00*16)},
{58	,(int)(1.00*16), (int)(1.00*16)},
{60	,(int)(1.00*16), (int)(1.00*16)},
{62	,(int)(1.00*16), (int)(1.00*16)},//60
{65	,(int)(1.00*16), (int)(1.00*16)},
{67	,(int)(1.00*16), (int)(1.00*16)},
{69	,(int)(1.00*16), (int)(1.00*16)},
{72	,(int)(1.00*16), (int)(1.00*16)},
{74	,(int)(1.00*16), (int)(1.00*16)},
{77	,(int)(1.00*16), (int)(1.00*16)},
{80	,(int)(1.00*16), (int)(1.00*16)},
{82	,(int)(1.00*16), (int)(1.00*16)},
{85	,(int)(1.00*16), (int)(1.00*16)},
{88	,(int)(1.00*16), (int)(1.00*16)},//70
{91	,(int)(1.00*16), (int)(1.00*16)},
{95	,(int)(1.00*16), (int)(1.00*16)},
{98	,(int)(1.00*16), (int)(1.00*16)},
{101,(int)(1.00*16), (int)(1.00*16)},
{105,(int)(1.00*16), (int)(1.00*16)},
{109,(int)(1.00*16), (int)(1.00*16)},
{112,(int)(1.00*16), (int)(1.00*16)},
{116,(int)(1.00*16), (int)(1.00*16)},
{121,(int)(1.00*16), (int)(1.00*16)},
{125,(int)(1.00*16), (int)(1.00*16)},//80
{129,(int)(1.00*16), (int)(1.00*16)},
{134,(int)(1.00*16), (int)(1.00*16)},
{139,(int)(1.00*16), (int)(1.00*16)},
{143,(int)(1.00*16), (int)(1.00*16)},
{148,(int)(1.00*16), (int)(1.00*16)},
{154,(int)(1.00*16), (int)(1.00*16)},
{159,(int)(1.00*16), (int)(1.00*16)},
{165,(int)(1.00*16), (int)(1.00*16)},
{171,(int)(1.00*16), (int)(1.00*16)},
{177,(int)(1.00*16), (int)(1.00*16)},//90
{183,(int)(1.00*16), (int)(1.00*16)},
{189,(int)(1.00*16), (int)(1.00*16)},
{196,(int)(1.00*16), (int)(1.00*16)},
{203,(int)(1.00*16), (int)(1.00*16)},
{210,(int)(1.00*16), (int)(1.00*16)},
{217,(int)(1.00*16), (int)(1.00*16)},
{225,(int)(1.00*16), (int)(1.00*16)},
{225,(int)(1.04*16), (int)(1.00*16)},
{225,(int)(1.06*16), (int)(1.00*16)},
{225,(int)(1.11*16), (int)(1.00*16)},//100
{225,(int)(1.13*16), (int)(1.00*16)},
{225,(int)(1.19*16), (int)(1.00*16)},
{225,(int)(1.25*16), (int)(1.00*16)},
{225,(int)(1.27*16), (int)(1.00*16)},
{225,(int)(1.31*16), (int)(1.00*16)},
{225,(int)(1.38*16), (int)(1.00*16)},
{225,(int)(1.41*16), (int)(1.00*16)},
{225,(int)(1.44*16), (int)(1.00*16)},
{225,(int)(1.50*16), (int)(1.00*16)},
{225,(int)(1.56*16), (int)(1.00*16)},//110
{225,(int)(1.63*16), (int)(1.00*16)},
{225,(int)(1.69*16), (int)(1.00*16)},
{225,(int)(1.75*16), (int)(1.00*16)},
{225,(int)(1.81*16), (int)(1.00*16)},
{225,(int)(1.88*16), (int)(1.00*16)},
{225,(int)(1.94*16), (int)(1.00*16)},
{450,(int)(1.00*16), (int)(1.00*16)},
{450,(int)(1.04*16), (int)(1.00*16)},
{450,(int)(1.06*16), (int)(1.00*16)},
{450,(int)(1.11*16), (int)(1.00*16)},//120
{450,(int)(1.13*16), (int)(1.00*16)},
{450,(int)(1.19*16), (int)(1.00*16)},
{450,(int)(1.25*16), (int)(1.00*16)},
{450,(int)(1.27*16), (int)(1.00*16)},
{450,(int)(1.31*16), (int)(1.00*16)},
{450,(int)(1.38*16), (int)(1.00*16)},
{450,(int)(1.41*16), (int)(1.00*16)},
{450,(int)(1.44*16), (int)(1.00*16)},
{450,(int)(1.50*16), (int)(1.00*16)},
{450,(int)(1.56*16), (int)(1.00*16)},//130
{450,(int)(1.63*16), (int)(1.00*16)},
{450,(int)(1.69*16), (int)(1.00*16)},
{450,(int)(1.75*16), (int)(1.00*16)},
{450,(int)(1.81*16), (int)(1.00*16)},
{450,(int)(1.88*16), (int)(1.00*16)},
{450,(int)(1.94*16), (int)(1.00*16)},
{675,(int)(1.31*16), (int)(1.00*16)},
{675,(int)(1.38*16), (int)(1.00*16)},
{675,(int)(1.41*16), (int)(1.00*16)},
{675,(int)(1.44*16), (int)(1.00*16)},//140
{675,(int)(1.50*16), (int)(1.00*16)},
{675,(int)(1.56*16), (int)(1.00*16)},
{675,(int)(1.63*16), (int)(1.00*16)},
{675,(int)(1.69*16), (int)(1.00*16)},
{675,(int)(1.75*16), (int)(1.00*16)},
{675,(int)(1.81*16), (int)(1.00*16)},
{675,(int)(1.88*16), (int)(1.00*16)},
{675,(int)(1.94*16), (int)(1.00*16)},
{675,(int)(2.00*16), (int)(1.00*16)},
{675,(int)(2.07*16), (int)(1.00*16)},//150
{675,(int)(2.14*16), (int)(1.00*16)},
{675,(int)(2.22*16), (int)(1.00*16)},
{675,(int)(2.30*16), (int)(1.00*16)},
{675,(int)(2.38*16), (int)(1.00*16)},
{675,(int)(2.46*16), (int)(1.00*16)},
{675,(int)(2.55*16), (int)(1.00*16)},
{675,(int)(2.64*16), (int)(1.00*16)},
{675,(int)(2.73*16), (int)(1.00*16)},
{675,(int)(2.83*16), (int)(1.00*16)},
{675,(int)(2.93*16), (int)(1.00*16)},//160
{675,(int)(3.03*16), (int)(1.00*16)},
{675,(int)(3.14*16), (int)(1.00*16)},
{675,(int)(3.25*16), (int)(1.00*16)},
{675,(int)(3.36*16), (int)(1.00*16)},
{675,(int)(3.48*16), (int)(1.00*16)},
{675,(int)(3.61*16), (int)(1.00*16)},
{675,(int)(3.73*16), (int)(1.00*16)},
{675,(int)(3.86*16), (int)(1.00*16)},
{675,(int)(4.00*16), (int)(1.00*16)},
{675,(int)(4.14*16), (int)(1.00*16)},//170
{675,(int)(4.29*16), (int)(1.00*16)},
{675,(int)(4.44*16), (int)(1.00*16)},
{675,(int)(4.59*16), (int)(1.00*16)},
{675,(int)(4.76*16), (int)(1.00*16)},
{675,(int)(4.92*16), (int)(1.00*16)},
{675,(int)(5.10*16), (int)(1.00*16)},
{675,(int)(5.28*16), (int)(1.00*16)},
{675,(int)(5.46*16), (int)(1.00*16)},
{675,(int)(5.66*16), (int)(1.00*16)},
{675,(int)(5.86*16), (int)(1.00*16)},//180
{675,(int)(6.06*16), (int)(1.00*16)},
{675,(int)(6.28*16), (int)(1.00*16)},
{675,(int)(6.50*16), (int)(1.00*16)},
{675,(int)(6.73*16), (int)(1.00*16)},
{675,(int)(6.96*16), (int)(1.00*16)},
{675,(int)(7.21*16), (int)(1.00*16)},
{675,(int)(7.46*16), (int)(1.00*16)},
{675,(int)(7.73*16), (int)(1.00*16)},
{675,(int)(8.00*16), (int)(1.00*16)}, //30fps*8x
{675,(int)(8.28*16), (int)(1.00*16)},//190
{675,(int)(8.57*16), (int)(1.00*16)},
{675,(int)(8.88*16), (int)(1.00*16)},
{675,(int)(9.19*16), (int)(1.00*16)},
{675,(int)(9.51*16), (int)(1.00*16)},
{675,(int)(9.85*16), (int)(1.00*16)},
{675,(int)(10.20*16), (int)(1.00*16)},
{675,(int)(10.56*16), (int)(1.00*16)},
{675,(int)(10.93*16), (int)(1.00*16)},
{675,(int)(11.31*16), (int)(1.00*16)},
{675,(int)(11.71*16), (int)(1.00*16)},//200
{675,(int)(12.13*16), (int)(1.00*16)},
{675,(int)(12.55*16), (int)(1.00*16)},
{675,(int)(13.00*16), (int)(1.00*16)},
{675,(int)(13.45*16), (int)(1.00*16)},
{675,(int)(13.93*16), (int)(1.00*16)},
{675,(int)(14.42*16), (int)(1.00*16)},
{675,(int)(14.93*16), (int)(1.00*16)},
{675,(int)(15.45*16), (int)(1.00*16)},
{675,(int)(16.00*16), (int)(1.00*16)} //209,30fps*16x
};

static const  int jxh42_30fps_exp_time_gain_60Hz[H42_30FPS_60HZ_EXP_TIME_TOTAL][3] =
{ // {time, analog gain, digital gain}
{8	,(int)(1.00*16), (int)(1.00*16)},
{8	,(int)(1.00*16), (int)(1.00*16)},
{8	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{11	,(int)(1.00*16), (int)(1.00*16)}, //10
{11	,(int)(1.00*16), (int)(1.00*16)},
{11	,(int)(1.00*16), (int)(1.00*16)},
{12	,(int)(1.00*16), (int)(1.00*16)},
{12	,(int)(1.00*16), (int)(1.00*16)},
{13	,(int)(1.00*16), (int)(1.00*16)},
{13	,(int)(1.00*16), (int)(1.00*16)},
{13	,(int)(1.00*16), (int)(1.00*16)},
{14	,(int)(1.00*16), (int)(1.00*16)},
{14	,(int)(1.00*16), (int)(1.00*16)},
{15	,(int)(1.00*16), (int)(1.00*16)},//20
{16	,(int)(1.00*16), (int)(1.00*16)},
{16	,(int)(1.00*16), (int)(1.00*16)},
{17	,(int)(1.00*16), (int)(1.00*16)},
{17	,(int)(1.00*16), (int)(1.00*16)},
{18	,(int)(1.00*16), (int)(1.00*16)},
{18	,(int)(1.00*16), (int)(1.00*16)},
{19	,(int)(1.00*16), (int)(1.00*16)},
{20	,(int)(1.00*16), (int)(1.00*16)},
{20	,(int)(1.00*16), (int)(1.00*16)},
{21	,(int)(1.00*16), (int)(1.00*16)},//30
{22	,(int)(1.00*16), (int)(1.00*16)},
{23	,(int)(1.00*16), (int)(1.00*16)},
{24	,(int)(1.00*16), (int)(1.00*16)},
{24	,(int)(1.00*16), (int)(1.00*16)},
{25	,(int)(1.00*16), (int)(1.00*16)},
{26	,(int)(1.00*16), (int)(1.00*16)},
{27	,(int)(1.00*16), (int)(1.00*16)},
{28	,(int)(1.00*16), (int)(1.00*16)},
{29	,(int)(1.00*16), (int)(1.00*16)},
{30	,(int)(1.00*16), (int)(1.00*16)},//40
{31	,(int)(1.00*16), (int)(1.00*16)},
{32	,(int)(1.00*16), (int)(1.00*16)},
{33	,(int)(1.00*16), (int)(1.00*16)},
{34	,(int)(1.00*16), (int)(1.00*16)},
{36	,(int)(1.00*16), (int)(1.00*16)},
{37	,(int)(1.00*16), (int)(1.00*16)},
{38	,(int)(1.00*16), (int)(1.00*16)},
{40	,(int)(1.00*16), (int)(1.00*16)},
{41	,(int)(1.00*16), (int)(1.00*16)},
{42	,(int)(1.00*16), (int)(1.00*16)},//50
{44	,(int)(1.00*16), (int)(1.00*16)},
{45	,(int)(1.00*16), (int)(1.00*16)},
{47	,(int)(1.00*16), (int)(1.00*16)},
{49	,(int)(1.00*16), (int)(1.00*16)},
{50	,(int)(1.00*16), (int)(1.00*16)},
{52	,(int)(1.00*16), (int)(1.00*16)},
{54	,(int)(1.00*16), (int)(1.00*16)},
{56	,(int)(1.00*16), (int)(1.00*16)},
{58	,(int)(1.00*16), (int)(1.00*16)},
{60	,(int)(1.00*16), (int)(1.00*16)},//60
{62	,(int)(1.00*16), (int)(1.00*16)},
{64	,(int)(1.00*16), (int)(1.00*16)},
{66	,(int)(1.00*16), (int)(1.00*16)},
{69	,(int)(1.00*16), (int)(1.00*16)},
{71	,(int)(1.00*16), (int)(1.00*16)},
{74	,(int)(1.00*16), (int)(1.00*16)},
{76	,(int)(1.00*16), (int)(1.00*16)},
{79	,(int)(1.00*16), (int)(1.00*16)},
{82	,(int)(1.00*16), (int)(1.00*16)},
{85	,(int)(1.00*16), (int)(1.00*16)},//70
{88	,(int)(1.00*16), (int)(1.00*16)},
{91	,(int)(1.00*16), (int)(1.00*16)},
{94	,(int)(1.00*16), (int)(1.00*16)},
{97	,(int)(1.00*16), (int)(1.00*16)},
{101,(int)(1.00*16), (int)(1.00*16)},
{104,(int)(1.00*16), (int)(1.00*16)},
{108,(int)(1.00*16), (int)(1.00*16)},
{112,(int)(1.00*16), (int)(1.00*16)},
{116,(int)(1.00*16), (int)(1.00*16)},
{120,(int)(1.00*16), (int)(1.00*16)}, //80
{124,(int)(1.00*16), (int)(1.00*16)},
{128,(int)(1.00*16), (int)(1.00*16)},
{133,(int)(1.00*16), (int)(1.00*16)},
{138,(int)(1.00*16), (int)(1.00*16)},
{142,(int)(1.00*16), (int)(1.00*16)},
{148,(int)(1.00*16), (int)(1.00*16)},
{153,(int)(1.00*16), (int)(1.00*16)},
{158,(int)(1.00*16), (int)(1.00*16)},
{164,(int)(1.00*16), (int)(1.00*16)},
{169,(int)(1.00*16), (int)(1.00*16)}, //90
{175,(int)(1.00*16), (int)(1.00*16)},
{182,(int)(1.00*16), (int)(1.00*16)},
{188,(int)(1.00*16), (int)(1.00*16)},
{188,(int)(1.04*16), (int)(1.00*16)},
{188,(int)(1.06*16), (int)(1.00*16)},
{188,(int)(1.11*16), (int)(1.00*16)},
{188,(int)(1.13*16), (int)(1.00*16)},
{188,(int)(1.19*16), (int)(1.00*16)},
{188,(int)(1.25*16), (int)(1.00*16)},
{188,(int)(1.27*16), (int)(1.00*16)}, //100
{188,(int)(1.31*16), (int)(1.00*16)},
{188,(int)(1.38*16), (int)(1.00*16)},
{188,(int)(1.41*16), (int)(1.00*16)},
{188,(int)(1.44*16), (int)(1.00*16)},
{188,(int)(1.50*16), (int)(1.00*16)},
{188,(int)(1.56*16), (int)(1.00*16)},
{188,(int)(1.63*16), (int)(1.00*16)},
{188,(int)(1.69*16), (int)(1.00*16)},
{188,(int)(1.75*16), (int)(1.00*16)},
{188,(int)(1.81*16), (int)(1.00*16)}, //110
{188,(int)(1.88*16), (int)(1.00*16)},
{188,(int)(1.94*16), (int)(1.00*16)},
{375,(int)(1.00*16), (int)(1.00*16)},
{375,(int)(1.04*16), (int)(1.00*16)},
{375,(int)(1.06*16), (int)(1.00*16)},
{375,(int)(1.11*16), (int)(1.00*16)},
{375,(int)(1.13*16), (int)(1.00*16)},
{375,(int)(1.19*16), (int)(1.00*16)},
{375,(int)(1.25*16), (int)(1.00*16)},
{375,(int)(1.27*16), (int)(1.00*16)}, //120
{375,(int)(1.31*16), (int)(1.00*16)},
{375,(int)(1.38*16), (int)(1.00*16)},
{375,(int)(1.41*16), (int)(1.00*16)},
{375,(int)(1.44*16), (int)(1.00*16)},
{375,(int)(1.50*16), (int)(1.00*16)},
{375,(int)(1.56*16), (int)(1.00*16)},
{375,(int)(1.63*16), (int)(1.00*16)},
{375,(int)(1.69*16), (int)(1.00*16)},
{375,(int)(1.75*16), (int)(1.00*16)},
{375,(int)(1.81*16), (int)(1.00*16)}, //130
{375,(int)(1.88*16), (int)(1.00*16)},
{375,(int)(1.94*16), (int)(1.00*16)},
{563,(int)(1.31*16), (int)(1.00*16)},
{563,(int)(1.38*16), (int)(1.00*16)},
{563,(int)(1.41*16), (int)(1.00*16)},
{563,(int)(1.44*16), (int)(1.00*16)},
{563,(int)(1.50*16), (int)(1.00*16)},
{563,(int)(1.56*16), (int)(1.00*16)},
{563,(int)(1.63*16), (int)(1.00*16)},
{563,(int)(1.69*16), (int)(1.00*16)}, //140
{563,(int)(1.75*16), (int)(1.00*16)},
{750,(int)(1.31*16), (int)(1.00*16)},
{750,(int)(1.38*16), (int)(1.00*16)},
{750,(int)(1.41*16), (int)(1.00*16)},
{750,(int)(1.44*16), (int)(1.00*16)},
{750,(int)(1.50*16), (int)(1.00*16)},
{750,(int)(1.56*16), (int)(1.00*16)},
{750,(int)(1.63*16), (int)(1.00*16)},
{750,(int)(1.69*16), (int)(1.00*16)},
{750,(int)(1.75*16), (int)(1.00*16)}, //150
{750,(int)(1.81*16), (int)(1.00*16)},
{750,(int)(1.88*16), (int)(1.00*16)},
{750,(int)(1.94*16), (int)(1.00*16)},
{750,(int)(2.00*16), (int)(1.00*16)},
{750,(int)(2.07*16), (int)(1.00*16)},
{750,(int)(2.14*16), (int)(1.00*16)},
{750,(int)(2.22*16), (int)(1.00*16)},
{750,(int)(2.30*16), (int)(1.00*16)},
{750,(int)(2.38*16), (int)(1.00*16)},
{750,(int)(2.46*16), (int)(1.00*16)},//160
{750,(int)(2.55*16), (int)(1.00*16)},
{750,(int)(2.64*16), (int)(1.00*16)},
{750,(int)(2.73*16), (int)(1.00*16)},
{750,(int)(2.83*16), (int)(1.00*16)},
{750,(int)(2.93*16), (int)(1.00*16)},
{750,(int)(3.03*16), (int)(1.00*16)},
{750,(int)(3.14*16), (int)(1.00*16)},
{750,(int)(3.25*16), (int)(1.00*16)},
{750,(int)(3.36*16), (int)(1.00*16)},
{750,(int)(3.48*16), (int)(1.00*16)},//170
{750,(int)(3.61*16), (int)(1.00*16)},
{750,(int)(3.73*16), (int)(1.00*16)},
{750,(int)(3.86*16), (int)(1.00*16)},
{750,(int)(4.00*16), (int)(1.00*16)},
{750,(int)(4.14*16), (int)(1.00*16)},
{750,(int)(4.29*16), (int)(1.00*16)},
{750,(int)(4.44*16), (int)(1.00*16)},
{750,(int)(4.59*16), (int)(1.00*16)},
{750,(int)(4.76*16), (int)(1.00*16)},
{750,(int)(4.92*16), (int)(1.00*16)},//180
{750,(int)(5.10*16), (int)(1.00*16)},
{750,(int)(5.28*16), (int)(1.00*16)},
{750,(int)(5.46*16), (int)(1.00*16)},
{750,(int)(5.66*16), (int)(1.00*16)},
{750,(int)(5.86*16), (int)(1.00*16)},
{750,(int)(6.06*16), (int)(1.00*16)},
{750,(int)(6.28*16), (int)(1.00*16)},
{750,(int)(6.50*16), (int)(1.00*16)},
{750,(int)(6.73*16), (int)(1.00*16)},
{750,(int)(6.96*16), (int)(1.00*16)},//190
{750,(int)(7.21*16), (int)(1.00*16)},//50hz,30fps,8x
{750,(int)(7.46*16), (int)(1.00*16)},
{750,(int)(7.73*16), (int)(1.00*16)},
{750,(int)(8.00*16), (int)(1.00*16)},
{750,(int)(8.28*16), (int)(1.00*16)},
{750,(int)(8.57*16), (int)(1.00*16)},
{750,(int)(8.88*16), (int)(1.00*16)},
{750,(int)(9.19*16), (int)(1.00*16)},
{750,(int)(9.51*16), (int)(1.00*16)},
{750,(int)(9.85*16), (int)(1.00*16)},//200
{750,(int)(10.20*16), (int)(1.00*16)},
{750,(int)(10.56*16), (int)(1.00*16)},
{750,(int)(10.93*16), (int)(1.00*16)},
{750,(int)(11.31*16), (int)(1.00*16)},
{750,(int)(11.71*16), (int)(1.00*16)},
{750,(int)(12.13*16), (int)(1.00*16)},
{750,(int)(12.55*16), (int)(1.00*16)},
{750,(int)(13.00*16), (int)(1.00*16)},
{750,(int)(13.45*16), (int)(1.00*16)},
{750,(int)(13.93*16), (int)(1.00*16)},//210
{750,(int)(14.42*16), (int)(1.00*16)},//50hz,30fps,16x
{750,(int)(14.93*16), (int)(1.00*16)},
{750,(int)(15.45*16), (int)(1.00*16)},
{750,(int)(16.00*16), (int)(1.00*16)},
};

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S h42_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
#if CP_DEMO_EN == 0
	h42_handle = drv_l2_sccb_open(H42_ID, 8, 8);
	if(h42_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#else
	sccb_config_t cfg;

	cfg.scl_port = IO_B4;
	cfg.scl_drv = IOD_DRV_8mA;
	cfg.sda_port = IO_B5;
	cfg.sda_drv = IOD_DRV_8mA;
	cfg.pwdn_port = 0;
	cfg.pwdn_drv = 0;
	cfg.have_pwdn = 0;
	cfg.RegBits = 8;
	cfg.DataBits = 8;
	cfg.slaveAddr = H42_ID;
	cfg.timeout = 0x20000;
	cfg.clock_rate = 100;

	h42_handle = drv_l2_sccb_open_ext(&cfg);

	//h42_handle = drv_l2_sccb_open(H42_ID, 8, 8);
	if(h42_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#endif
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	h42_handle.devNumber = I2C_0;
	h42_handle.slaveAddr = H42_ID;
	h42_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void h42_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(h42_handle) {
		drv_l2_sccb_close(h42_handle);
		h42_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	h42_handle.slaveAddr = 0;
	h42_handle.clkRate = 0;
#endif
}

static INT32S h42_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(h42_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&h42_handle, data, 3);
#endif
}


static INT32S h42_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(h42_handle, reg, &data) >= 0) {
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
	if(drv_l1_i2c_bus_write(&h42_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&h42_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}

static INT32S h42_sccb_write_table(regval8_t *pTable)
{
	while(1) {
		if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
			break;
		}

		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(h42_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
		pTable++;
	}
	return STATUS_OK;
}

static INT32S h42_sccb_read_table(regval8_t *pTable)
{
    INT8U *Rdata;
	while(1) {
		if(pTable->reg_num == 0x00 && pTable->value == 0x00) {
			break;
		}

		if(h42_sccb_read(pTable->reg_num, Rdata) < 0) {
			DBG_PRINT("sccb read fail.\r\n");
			continue;
		}
		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, *Rdata);
		pTable++;
	}
	return STATUS_OK;
}

/*****************************************************************************************
+++++++++++++++++AEC/AGC
*****************************************************************************************/
static int h42_cvt_agc_gain(int agc_gain)
{

	INT32U h42_agc_gain, i;
    INT32U pow2;

    pow2 = 0;
    i = 5;
    do {
	    if(agc_gain <= 0x1f)
           break;

	    agc_gain >>= 1;
	    pow2++;

	    i--;
    } while(i != 0);

    agc_gain -= 0x10;

    if(agc_gain < 0) agc_gain = 0;
    h42_agc_gain = (pow2 << 4) + agc_gain;

	return h42_agc_gain;
}

#if 0
static int h42_set_xfps_exposure_time(sensor_exposure_t *si)
{
	unsigned char t1, t2;
	int idx, temp, ret;

	//DBG_PRINT("%s:%d\n", __FUNCTION__, __LINE__);
	si->sensor_ev_idx += si->ae_ev_step;
	if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
	if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;
#if RAW_MODE
	idx = (g_raw_mode_idx < 0) ? si->sensor_ev_idx * 3 : g_raw_mode_idx * 3;
	DBG_PRINT("%s:%d %d\n", __FUNCTION__, __LINE__, g_raw_mode_idx);
#else
	idx = si->sensor_ev_idx * 3;
#endif
	si ->time = p_expTime_table[idx];
	si ->analog_gain = p_expTime_table[idx+1];
	//si ->digital_gain = p_expTime_table[idx+2];
    //DBG_PRINT("[EV=%d, offset=%d]: time = 0x%x, analog gain =0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);

    if (ae_flag == 0)
    {
        // exposure time
        if(si ->time != pre_sensor_time)
        {
            pre_sensor_time = si->time;
            temp = si->time;

            t1 = (temp & 0xff);
            t2 = (temp >> 8) & 0x00ff;

            ret = h42_sccb_write(0x01, t1);
            if(ret < 0) return ret;
            ret = h42_sccb_write(0x02, t2);

            if(ret < 0) {
                DBG_PRINT("ERR: write sensor time = 0x%x!!!\r\n", temp);
                return ret;
            }
        }
        ae_flag = 1;
    }

    if(ae_flag == 1)
    {   //Gain
        if(si ->analog_gain != pre_sensor_a_gain)
        {
            pre_sensor_a_gain = si->analog_gain;

            temp = si->analog_gain;
            temp = h42_cvt_agc_gain(temp);
            t1 = temp & 0x007f;
            ret = h42_sccb_write(0x00, t1);
            if(ret < 0) {
                DBG_PRINT("ERR: write sensor gain = 0x%x, 0x%x, 0x%x !!!\r\n", t1, temp, si->analog_gain);
                return ret;
            }
        }
        ae_flag = 0;
    }

	return 0;
}
#else   //Group Write
static int h42_set_xfps_exposure_time(sensor_exposure_t *si)
{
	unsigned char t1, t2;
	int idx, temp, ret;

	//DBG_PRINT("%s:%d\n", __FUNCTION__, __LINE__);
	si->sensor_ev_idx += si->ae_ev_step;
	if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
	if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;
#if RAW_MODE
	idx = (g_raw_mode_idx < 0) ? si->sensor_ev_idx * 3 : g_raw_mode_idx * 3;
	DBG_PRINT("%s:%d %d\n", __FUNCTION__, __LINE__, g_raw_mode_idx);
#else
	idx = si->sensor_ev_idx * 3;
#endif
	si ->time = p_expTime_table[idx];
	si ->analog_gain = p_expTime_table[idx+1];
	//si ->digital_gain = p_expTime_table[idx+2];
    //DBG_PRINT("[EV=%d, offset=%d]: time = 0x%x, analog gain =0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);

    // exposure time
	if(si ->time != pre_sensor_time)
	{
		pre_sensor_time = si->time;
		temp = si->time;

		t1 = (temp & 0xff);
		t2 = (temp >> 8) & 0x00ff;
        #if 1
		ret = h42_sccb_write(0x01, t1);
		if(ret < 0) return ret;
		ret = h42_sccb_write(0x02, t2);
		#else//group write
		ret = h42_sccb_write(0xC0, 0x01);
		if(ret < 0) return ret;
		ret = h42_sccb_write(0xC1, t1);
		if(ret < 0) return ret;
		ret = h42_sccb_write(0xC2, 0x02);
		if(ret < 0) return ret;
		ret = h42_sccb_write(0xC3, t2);
		#endif

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
		temp = h42_cvt_agc_gain(temp);
		t1 = temp & 0x007f;
        #if 1
		ret = h42_sccb_write(0x00, t1);
		#else   //group write
		ret = h42_sccb_write(0xC4, 0x00);
		if(ret < 0) return ret;
		ret = h42_sccb_write(0xC5, t1);
		#endif
		if(ret < 0) {
            DBG_PRINT("ERR: write sensor gain = 0x%x, 0x%x, 0x%x !!!\r\n", t1, temp, si->analog_gain);
            return ret;
        }

	}
	#if 0//group write
        #if SENSOR_FLIP
        ret = h42_sccb_write(0x12, 0x38);   //group write enable
        #else
        ret = h42_sccb_write(0x12, 0x08);   //group write enable
        #endif
        if(ret < 0) {
            DBG_PRINT("ERR: triger group write FAIL!!!\r\n");
            return ret;
        }
    #endif

    return 0;
}
#endif

static void h42_set_ae(int ev_step)
{
    seInfo.ae_ev_step = ev_step;
    h42_set_xfps_exposure_time(&seInfo);
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)h42_set_ae;
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
static void mipi_h42_handle(INT32U event)
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
void jxh42_seinfo_init(void)
{
    seInfo.sensor_ev_idx = H42_30FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= H42_30FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= H42_30FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = H42_30FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = H42_30FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)jxh42_30fps_exp_time_gain_50Hz;

	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x00;

}

static void h42_mipi_sensor_init(void)
{
	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);
    #if MIPI_DEV_NO == 0
	/* set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk*/
	//R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<1));//0x02400042;
	/* set Mclk(CSI_CLKO) to IOD7, for MIPI Mclk*/
	//R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	#else
    // set Mclk(CSI_CLKO) to IOD7, for MIPI Mclk
	R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<1));
	#endif

    //H42_set_favorite();
	// mclk output
	//drv_l2_sensor_set_mclkout(h42_cdsp_mipi_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);

	// reguest sccb
	h42_sccb_open();

	// reset sensor
	h42_sccb_write_table((regval8_t *)h42_reset_table);
    osDelay(200);

	// init sensor
	h42_sccb_write_table((regval8_t *)h42_init_table);

	//h42_sccb_read_table((regval8_t *)h42_init_table);

    DBG_PRINT("Sensor H42 sensor mipi init completed\r\n");
}


/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
//static void h42_cdsp_mipi_init(void)
void h42_cdsp_mipi_init(void)
{
	//ae init
	jxh42_seinfo_init();
    //cdsp init
	drv_l2_cdsp_open();
	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

	// mipi enable
	#if (MIPI_DEV_NO == 1)
		drv_l1_mipi_init(MIPI_1,ENABLE);
	#else
	    drv_l1_mipi_init(MIPI_0,ENABLE);
	#endif

    DBG_PRINT("Sensor H42 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void h42_cdsp_mipi_uninit(void)
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
	h42_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void h42_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
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
        h42_sccb_write_table((regval16_t *)h42_scale_vga_table);
		break;

	case 1:
		h42_sccb_write_table((regval16_t *)h42_720p_table);
		break;

	default:
		while(1);
	}
*/
    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 4);    //u2.added.20151207
    //osDelay(200);

#if 1   // change mclk
	drv_l2_sensor_set_mclkout(h42_cdsp_mipi_ops.info[index].mclk);
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
	format.input_format =  h42_cdsp_mipi_ops.info[index].input_format;
	format.output_format = h42_cdsp_mipi_ops.info[index].output_format;
	target_w = h42_cdsp_mipi_ops.info[index].target_w;
	target_h = h42_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = h42_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = h42_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = h42_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = h42_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = h42_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = h42_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = h42_cdsp_mipi_ops.info[index].vsync_active;


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
		h42_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		h42_mipi_cfg.data_type = MIPI_RAW8;
		//h42_mipi_cfg.h_back_porch = 4;
		break;

	case 2:
	case 3:
		h42_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		h42_mipi_cfg.data_type = MIPI_RAW10;
		//h42_mipi_cfg.h_back_porch = 2;//4;
        h42_mipi_cfg.pixel_clk_sel = 0;
		break;

	default:
    	/* #if (RD_SIM == 1)
        h42_mipi_cfg.blanking_line_en = 1;
        h42_mipi_cfg.byte_clk_edge = 0;
        h42_mipi_cfg.check_hs_seq = 0;
        h42_mipi_cfg.data_from_mmr = 0;
        h42_mipi_cfg.data_mask_time = 250;
        h42_mipi_cfg.data_type = 3;
        h42_mipi_cfg.data_type_to_cdsp = 1;
        h42_mipi_cfg.ecc_check_en = 1;
        h42_mipi_cfg.ecc_order = 3;
        h42_mipi_cfg.h_front_porch = 4;
        h42_mipi_cfg.low_power_en = 0;
        h42_mipi_cfg.mipi_lane_no = 1;
        h42_mipi_cfg.pixel_clk_sel = 1;
        #endif
        */
        h42_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		h42_mipi_cfg.data_type = MIPI_RAW8;
		//h42_mipi_cfg.h_back_porch = 4;
	}

	//h42_mipi_cfg.h_size = format.hpixel;
	//h42_mipi_cfg.v_size = format.vline;
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      if (drv_l1_mipi_set_parameter(MIPI_1, &h42_mipi_cfg) < 0)
      {
        DBG_PRINT("MIPI1 init fail !!!\r\n");
      }
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_h42_handle);
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      if (drv_l1_mipi_set_parameter(MIPI_0, &h42_mipi_cfg) < 0)
      {
        DBG_PRINT("MIPI0 init fail !!!\r\n");
      }
      else
      {
        DBG_PRINT("MIPI0 init completed.\r\n");
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_h42_handle);
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
      }
    #endif

	//Enable MCLK & Init Sensor
    h42_mipi_sensor_init();


    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    h42_set_xfps_exposure_time(&seInfo);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void h42_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* h42_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&h42_cdsp_mipi_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t h42_cdsp_mipi_ops =
{
	SENSOR_H42_CDSP_MIPI_NAME,		/* sensor name */
	h42_cdsp_mipi_init,
	h42_cdsp_mipi_uninit,
	h42_cdsp_mipi_stream_on,
	h42_cdsp_mipi_stream_off,
	h42_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			H42_WIDTH,					/* target width */
			H42_HEIGHT,  				/* target height */
			H42_WIDTH,				/* sensor width */
			H42_HEIGHT,  				/* sensor height */
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
			V4L2_PIX_FMT_SBGGR10,//V4L2_PIX_FMT_SBGGR8,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			H42_OUT_WIDTH,			        /* target width */
			H42_OUT_HEIGHT, 			    /* target height */
			H42_WIDTH,				/* sensor width */
			H42_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
		/* 3st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SRGGB10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			H42_OUT_WIDTH,					/* target width */
			H42_OUT_HEIGHT,  				/* target height */
			H42_WIDTH,				/* sensor width */
			H42_HEIGHT,  				/* sensor height */
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
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			H42_OUT_WIDTH,			    /* target width */
			H42_OUT_HEIGHT,			    /* target height */
			H42_WIDTH,				    /* sensor width */
			H42_HEIGHT, 				/* sensor height */
			2,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};
#endif //(defined _SENSOR_H42_CDSP_MIPI) && (_SENSOR_H42_CDSP_MIPI == 1)
