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
//#include "drv_l1_mipi.h"
#include "drv_l1_i2c.h"
#include "drv_l2_sensor.h"
#include "drv_l2_sccb.h"
#include "drv_l2_cdsp.h"


#if (defined _SENSOR_JXH22_CDSP_DVP) && (_SENSOR_JXH22_CDSP_DVP == 1)

#include "drv_l2_user_calibration.h"
#include "drv_l2_user_preference.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/

#define	JXH22_ID						0x60	//0x64,0x68,0x6C
#define JXH22_WIDTH						1280
#define JXH22_HEIGHT					720
#define JXH22_OUT_WIDTH					320//1280
#define JXH22_OUT_HEIGHT				240//720

#ifndef DISABLE
#define DISABLE     					0
#endif

#ifndef ENABLE
#define ENABLE      					1
#endif



#define JXH22_30FPS_50HZ_DAY_EV_IDX 			140
#define JXH22_30FPS_50HZ_NIGHT_EV_IDX			190
#define JXH22_30FPS_50HZ_EXP_TIME_TOTAL			211
#define JXH22_30FPS_50HZ_INIT_EV_IDX 			((JXH22_30FPS_50HZ_DAY_EV_IDX + JXH22_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define JXH22_30FPS_50HZ_MAX_EV_IDX				(JXH22_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define JXH22_30FPS_60HZ_DAY_EV_IDX 			137
#define JXH22_30FPS_60HZ_NIGHT_EV_IDX			190
#define JXH22_30FPS_60HZ_EXP_TIME_TOTAL			215
#define JXH22_30FPS_60HZ_INIT_EV_IDX 			((JXH22_30FPS_60HZ_DAY_EV_IDX + JXH22_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define JXH22_30FPS_60HZ_MAX_EV_IDX			 	(JXH22_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define JXH22_24FPS_50HZ_DAY_EV_IDX 			138
#define JXH22_24FPS_50HZ_NIGHT_EV_IDX			194
#define JXH22_24FPS_50HZ_EXP_TIME_TOTAL			254
#define JXH22_24FPS_50HZ_INIT_EV_IDX 			((JXH22_24FPS_50HZ_DAY_EV_IDX + JXH22_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define JXH22_24FPS_50HZ_MAX_EV_IDX				(JXH22_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define JXH22_24FPS_60HZ_DAY_EV_IDX 			135
#define JXH22_24FPS_60HZ_NIGHT_EV_IDX			195
#define JXH22_24FPS_60HZ_EXP_TIME_TOTAL			255
#define JXH22_24FPS_60HZ_INIT_EV_IDX 			((JXH22_24FPS_60HZ_DAY_EV_IDX + JXH22_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define JXH22_24FPS_60HZ_MAX_EV_IDX				(JXH22_24FPS_60HZ_EXP_TIME_TOTAL - 10)


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
	static void *jxh22_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t jxh22_handle;
#endif

static const regval8_t jxh22_reset_table[] =
{
	{ 0x12, 0x80 },
	{ 0xff, 0xff },
};

/***********************************
;INI Create Date : 2014/7/28
;Create By easonlin

;;Output Detail:
;;MCLK:24 MHz
;;PCLK:38.4
;;VCO:384
;;FrameW:1500
;;FrameH:854
************************************/
static const regval8_t jxh22_720P_30_init_table[] =
{	//38.4MHz
	//;;INI Start
	{0x12,0x40},
	//;;DVP Setting
	{0x1F,0x00},
	//;;PLL Setting
	{0x0E,0x1D},
	{0x0F,0x09},
	{0x10,0x20},
	{0x11,0x80},
	//;;Frame/Window
	{0x20,0xDC},
	{0x21,0x05},
#if 0//25fps
	{0x22,0x00},
	{0x23,0x04},
	{0x27,0xBB},
	{0x2C,0x01},
#else //30fps
	{0x22,0x56},//H:816(L)=0x330,840=0x348
	{0x23,0x03},//H:816(M)
	{0x27,0xC1},
	{0x2C,0x00},
#endif
	{0x24,0x04}, //0x00},
	{0x25,0xD4}, //0xD0},
	{0x26,0x25},

	{0x28,0x0D},
	{0x29,0x00},
	{0x2A,0xAC},
	{0x2B,0x10},

	{0x2D,0x0A},
	{0x2E,0xC2},
	{0x2F,0x20},
	//;;AE/AG/ABLC
	{0x13,0x01},//0xC6},   //0xc6: AE on, 0x01: AE off
	{0x14,0x80},
	{0x16,0xC0},
	{0x17,0x40},
	{0x18,0xD5},
	{0x19,0x00},
	{0x37,0x36},
	{0x38,0x98},
	{0x4a,0x03},
	{0x49,0x10},
	//;;Interface
	{0x1D,0xFF},
	{0x1E,0x9F},//0x9f
	{0x6C,0x90},
	{0x73,0xB3},
	{0x70,0x69},
	{0x76,0x40},
	{0x77,0x06},
	//{0x67,0x31},
	//{0x6D,0x09},
#if 1	//For 太陽黑子 
	{0x0C,0x00}, //[6],第一段開關 
	{0x6A,0x4A}, //[3:0],第一段微調,豎線條為避掉太陽黑子而出現的Side effect,從0x4C 往下,0x49~0x4C
	{0x2B,0x90}, //[7],第二段開關 
	{0x2F,0x80}, //[7~4]第二段微調normal Light
#endif
	//;;INI End
	#if (SENSOR_FLIP == 1)
	{0x12,0x30},
	#else
	{0x12,0x00},
	#endif
	/*PWDN Setting*/
	//{0x12,0x40},
	//{0x0E,0x9D},
	//{0x67,0x67},
	//{0x6C,0x90},
	//{0x1D,0x00},
	//{0x1E,0x00},
    //write Done
	{0xff,0xff},
};


static const regval8_t jxh22_720p_table[] =
{

};

#if 0
static const regval8_t jxh22_suspend_table[] =
{

};

static const regval8_t jxh22_resume_table[] =
{

};
#endif

static const int jxh22__30fps_exp_time_gain_50Hz[JXH22_30FPS_50HZ_EXP_TIME_TOTAL][3] =
{ // {time, analog gain, digital gain}
	{8	,(int)(1.00*16), (int)(1.00*16)},	//0
	{9	,(int)(1.00*16), (int)(1.00*16)},
	{9	,(int)(1.00*16), (int)(1.00*16)},
	{9	,(int)(1.00*16), (int)(1.00*16)},
	{10	,(int)(1.00*16), (int)(1.00*16)},
	{10	,(int)(1.00*16), (int)(1.00*16)},
	{10	,(int)(1.00*16), (int)(1.00*16)},
	{11	,(int)(1.00*16), (int)(1.00*16)},
	{11	,(int)(1.00*16), (int)(1.00*16)},
	{11	,(int)(1.00*16), (int)(1.00*16)},
	{12	,(int)(1.00*16), (int)(1.00*16)},   //10
	{12	,(int)(1.00*16), (int)(1.00*16)},
	{13	,(int)(1.00*16), (int)(1.00*16)},
	{13	,(int)(1.00*16), (int)(1.00*16)},
	{13	,(int)(1.00*16), (int)(1.00*16)},
	{14	,(int)(1.00*16), (int)(1.00*16)},
	{14	,(int)(1.00*16), (int)(1.00*16)},
	{15	,(int)(1.00*16), (int)(1.00*16)},
	{15	,(int)(1.00*16), (int)(1.00*16)},
	{16	,(int)(1.00*16), (int)(1.00*16)},
	{17	,(int)(1.00*16), (int)(1.00*16)},   //20
	{17	,(int)(1.00*16), (int)(1.00*16)},
	{18	,(int)(1.00*16), (int)(1.00*16)},
	{18	,(int)(1.00*16), (int)(1.00*16)},
	{19	,(int)(1.00*16), (int)(1.00*16)},
	{20	,(int)(1.00*16), (int)(1.00*16)},
	{20	,(int)(1.00*16), (int)(1.00*16)},
	{21	,(int)(1.00*16), (int)(1.00*16)},
	{22	,(int)(1.00*16), (int)(1.00*16)},
	{23	,(int)(1.00*16), (int)(1.00*16)},
	{23	,(int)(1.00*16), (int)(1.00*16)},   //30
	{24	,(int)(1.00*16), (int)(1.00*16)},
	{25	,(int)(1.00*16), (int)(1.00*16)},
	{26	,(int)(1.00*16), (int)(1.00*16)},
	{27	,(int)(1.00*16), (int)(1.00*16)},
	{28	,(int)(1.00*16), (int)(1.00*16)},
	{29	,(int)(1.00*16), (int)(1.00*16)},
	{30	,(int)(1.00*16), (int)(1.00*16)},
	{31	,(int)(1.00*16), (int)(1.00*16)},
	{32	,(int)(1.00*16), (int)(1.00*16)},
	{33	,(int)(1.00*16), (int)(1.00*16)},   //40
	{34	,(int)(1.00*16), (int)(1.00*16)},
	{36	,(int)(1.00*16), (int)(1.00*16)},
	{37	,(int)(1.00*16), (int)(1.00*16)},
	{38	,(int)(1.00*16), (int)(1.00*16)},
	{39	,(int)(1.00*16), (int)(1.00*16)},
	{41	,(int)(1.00*16), (int)(1.00*16)},
	{42	,(int)(1.00*16), (int)(1.00*16)},
	{44	,(int)(1.00*16), (int)(1.00*16)},
	{45	,(int)(1.00*16), (int)(1.00*16)},
	{47	,(int)(1.00*16), (int)(1.00*16)},   //50
	{49	,(int)(1.00*16), (int)(1.00*16)},
	{50	,(int)(1.00*16), (int)(1.00*16)},
	{52	,(int)(1.00*16), (int)(1.00*16)},
	{54	,(int)(1.00*16), (int)(1.00*16)},
	{56	,(int)(1.00*16), (int)(1.00*16)},
	{58	,(int)(1.00*16), (int)(1.00*16)},
	{60	,(int)(1.00*16), (int)(1.00*16)},
	{62	,(int)(1.00*16), (int)(1.00*16)},
	{64	,(int)(1.00*16), (int)(1.00*16)},
	{66	,(int)(1.00*16), (int)(1.00*16)},   //60
	{69	,(int)(1.00*16), (int)(1.00*16)},
	{71	,(int)(1.00*16), (int)(1.00*16)},
	{74	,(int)(1.00*16), (int)(1.00*16)},
	{76	,(int)(1.00*16), (int)(1.00*16)},
	{79	,(int)(1.00*16), (int)(1.00*16)},
	{82	,(int)(1.00*16), (int)(1.00*16)},
	{84	,(int)(1.00*16), (int)(1.00*16)},
	{87	,(int)(1.00*16), (int)(1.00*16)},
	{91	,(int)(1.00*16), (int)(1.00*16)},
	{94	,(int)(1.00*16), (int)(1.00*16)},   //70
	{97	,(int)(1.00*16), (int)(1.00*16)},
	{100,(int)(1.00*16), (int)(1.00*16)},
	{104,(int)(1.00*16), (int)(1.00*16)},
	{108,(int)(1.00*16), (int)(1.00*16)},
	{111,(int)(1.00*16), (int)(1.00*16)},
	{115,(int)(1.00*16), (int)(1.00*16)},
	{119,(int)(1.00*16), (int)(1.00*16)},
	{124,(int)(1.00*16), (int)(1.00*16)},
	{128,(int)(1.00*16), (int)(1.00*16)},
	{133,(int)(1.00*16), (int)(1.00*16)},   //80
	{137,(int)(1.00*16), (int)(1.00*16)},
	{142,(int)(1.00*16), (int)(1.00*16)},
	{147,(int)(1.00*16), (int)(1.00*16)},
	{152,(int)(1.00*16), (int)(1.00*16)},
	{158,(int)(1.00*16), (int)(1.00*16)},
	{163,(int)(1.00*16), (int)(1.00*16)},
	{169,(int)(1.00*16), (int)(1.00*16)},
	{175,(int)(1.00*16), (int)(1.00*16)},
	{181,(int)(1.00*16), (int)(1.00*16)},
	{187,(int)(1.00*16), (int)(1.00*16)},   //90
	{194,(int)(1.00*16), (int)(1.00*16)},
	{201,(int)(1.00*16), (int)(1.00*16)},
	{208,(int)(1.00*16), (int)(1.00*16)},
	{215,(int)(1.00*16), (int)(1.00*16)},
	{223,(int)(1.00*16), (int)(1.00*16)},
	{231,(int)(1.00*16), (int)(1.00*16)},
	{239,(int)(1.00*16), (int)(1.00*16)},
	{247,(int)(1.00*16), (int)(1.00*16)},
	{256,(int)(1.00 *16), (int)(1.00*16)},
	{256,(int)(1.04 *16), (int)(1.00*16)},  //100
	{256,(int)(1.07 *16), (int)(1.00*16)},
	{256,(int)(1.11 *16), (int)(1.00*16)},
	{256,(int)(1.15 *16), (int)(1.00*16)},
	{256,(int)(1.19 *16), (int)(1.00*16)},
	{256,(int)(1.23 *16), (int)(1.00*16)},
	{256,(int)(1.27 *16), (int)(1.00*16)},
	{256,(int)(1.32 *16), (int)(1.00*16)},
	{256,(int)(1.37 *16), (int)(1.00*16)},
	{256,(int)(1.41 *16), (int)(1.00*16)},
	{256,(int)(1.46 *16), (int)(1.00*16)},  //110
	{256,(int)(1.52 *16), (int)(1.00*16)},
	{256,(int)(1.57 *16), (int)(1.00*16)},
	{256,(int)(1.62 *16), (int)(1.00*16)},
	{256,(int)(1.68 *16), (int)(1.00*16)},
	{256,(int)(1.74 *16), (int)(1.00*16)},
	{256,(int)(1.80 *16), (int)(1.00*16)},
	{256,(int)(1.87 *16), (int)(1.00*16)},
	{256,(int)(1.93 *16), (int)(1.00*16)},
	{512,(int)(1.00 *16), (int)(1.00*16)},
	{512,(int)(1.04 *16), (int)(1.00*16)},  //120
	{512,(int)(1.07 *16), (int)(1.00*16)},
	{512,(int)(1.11 *16), (int)(1.00*16)},
	{512,(int)(1.15 *16), (int)(1.00*16)},
	{512,(int)(1.19 *16), (int)(1.00*16)},
	{512,(int)(1.23 *16), (int)(1.00*16)},
	{512,(int)(1.27 *16), (int)(1.00*16)},
	{512,(int)(1.32 *16), (int)(1.00*16)},
	{512,(int)(1.37 *16), (int)(1.00*16)},
	{512,(int)(1.41 *16), (int)(1.00*16)},
	{512,(int)(1.46 *16), (int)(1.00*16)},  //130
	{512,(int)(1.52 *16), (int)(1.00*16)},
	{512,(int)(1.57 *16), (int)(1.00*16)},
	{512,(int)(1.62 *16), (int)(1.00*16)},
	{512,(int)(1.68 *16), (int)(1.00*16)},
	{512,(int)(1.74 *16), (int)(1.00*16)},
	{512,(int)(1.80 *16), (int)(1.00*16)},
	{512,(int)(1.87 *16), (int)(1.00*16)},
	{512,(int)(1.93 *16), (int)(1.00*16)},
	{768,(int)(1.32 *16), (int)(1.00*16)},
	{768,(int)(1.37 *16), (int)(1.00*16)},  //140
	{768,(int)(1.41 *16), (int)(1.00*16)},
	{768,(int)(1.46 *16), (int)(1.00*16)},
	{768,(int)(1.52 *16), (int)(1.00*16)},
	{768,(int)(1.57 *16), (int)(1.00*16)},
	{768,(int)(1.62 *16), (int)(1.00*16)},
	{768,(int)(1.68 *16), (int)(1.00*16)},
	{768,(int)(1.74 *16), (int)(1.00*16)},
	{768,(int)(1.80 *16), (int)(1.00*16)},
	{768,(int)(1.87 *16), (int)(1.00*16)},
	{768,(int)(1.93 *16), (int)(1.00*16)},  //150
	{768,(int)(2.00 *16), (int)(1.00*16)},
	{768,(int)(2.07 *16), (int)(1.00*16)},
	{768,(int)(2.14 *16), (int)(1.00*16)},
	{768,(int)(2.22 *16), (int)(1.00*16)},
	{768,(int)(2.30 *16), (int)(1.00*16)},
	{768,(int)(2.38 *16), (int)(1.00*16)},
	{768,(int)(2.46 *16), (int)(1.00*16)},
	{768,(int)(2.55 *16), (int)(1.00*16)},
	{768,(int)(2.64 *16), (int)(1.00*16)},
	{768,(int)(2.73 *16), (int)(1.00*16)},  //160
	{768,(int)(2.83 *16), (int)(1.00*16)},
	{768,(int)(2.93 *16), (int)(1.00*16)},
	{768,(int)(3.03 *16), (int)(1.00*16)},
	{768,(int)(3.14 *16), (int)(1.00*16)},
	{768,(int)(3.25 *16), (int)(1.00*16)},
	{768,(int)(3.36 *16), (int)(1.00*16)},
	{768,(int)(3.48 *16), (int)(1.00*16)},
	{768,(int)(3.61 *16), (int)(1.00*16)},
	{768,(int)(3.73 *16), (int)(1.00*16)},
	{768,(int)(3.86 *16), (int)(1.00*16)},  //170
	{768,(int)(4.00 *16), (int)(1.00*16)},
	{768,(int)(4.14 *16), (int)(1.00*16)},
	{768,(int)(4.29 *16), (int)(1.00*16)},
	{768,(int)(4.44 *16), (int)(1.00*16)},
	{768,(int)(4.59 *16), (int)(1.00*16)},
	{768,(int)(4.76 *16), (int)(1.00*16)},
	{768,(int)(4.92 *16), (int)(1.00*16)},
	{768,(int)(5.10 *16), (int)(1.00*16)},
	{768,(int)(5.28 *16), (int)(1.00*16)},
	{768,(int)(5.46 *16), (int)(1.00*16)},  //180
	{768,(int)(5.66 *16), (int)(1.00*16)},
	{768,(int)(5.86 *16), (int)(1.00*16)},
	{768,(int)(6.06 *16), (int)(1.00*16)},
	{768,(int)(6.28 *16), (int)(1.00*16)},
	{768,(int)(6.50 *16), (int)(1.00*16)},
	{768,(int)(6.73 *16), (int)(1.00*16)},
	{768,(int)(6.96 *16), (int)(1.00*16)},
	{768,(int)(7.21 *16), (int)(1.00*16)},
	{768,(int)(7.46 *16), (int)(1.00*16)},
	{768,(int)(7.73 *16), (int)(1.00*16)},  //190
	{768,(int)(8.00 *16), (int)(1.00*16)},
	{768,(int)(8.28 *16), (int)(1.00*16)},
	{768,(int)(8.57 *16), (int)(1.00*16)},
	{768,(int)(8.88 *16), (int)(1.00*16)},
	{768,(int)(9.19 *16), (int)(1.00*16)},
	{768,(int)(9.51 *16), (int)(1.00*16)},
	{768,(int)(9.85 *16), (int)(1.00*16)},
	{768,(int)(10.20*16), (int)(1.00*16)},
	{768,(int)(10.56*16), (int)(1.00*16)},
	{768,(int)(10.93*16), (int)(1.00*16)},  //200
	{768,(int)(11.31*16), (int)(1.00*16)},
	{768,(int)(11.71*16), (int)(1.00*16)},
	{768,(int)(12.13*16), (int)(1.00*16)},
	{768,(int)(12.55*16), (int)(1.00*16)},
	{768,(int)(13.00*16), (int)(1.00*16)},
	{768,(int)(13.45*16), (int)(1.00*16)},
	{768,(int)(13.93*16), (int)(1.00*16)},
	{768,(int)(14.42*16), (int)(1.00*16)},
	{768,(int)(14.93*16), (int)(1.00*16)},
	{768,(int)(15.45*16), (int)(1.00*16)},  //210
	{768,(int)(16.00*16), (int)(1.00*16)},
};

static const  int jxh22_30fps_exp_time_gain_60Hz[JXH22_30FPS_60HZ_EXP_TIME_TOTAL][3] =
{ // {time, analog gain, digital gain}
{8	,(int)(1.00*16), (int)(1.00*16)},	//0
{9	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{9	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{10	,(int)(1.00*16), (int)(1.00*16)},
{11	,(int)(1.00*16), (int)(1.00*16)},
{11	,(int)(1.00*16), (int)(1.00*16)},
{12	,(int)(1.00*16), (int)(1.00*16)},
{12	,(int)(1.00*16), (int)(1.00*16)},	//10
{12	,(int)(1.00*16), (int)(1.00*16)},
{13	,(int)(1.00*16), (int)(1.00*16)},
{13	,(int)(1.00*16), (int)(1.00*16)},
{14	,(int)(1.00*16), (int)(1.00*16)},
{14	,(int)(1.00*16), (int)(1.00*16)},
{15	,(int)(1.00*16), (int)(1.00*16)},
{15	,(int)(1.00*16), (int)(1.00*16)},
{16	,(int)(1.00*16), (int)(1.00*16)},
{16	,(int)(1.00*16), (int)(1.00*16)},
{17	,(int)(1.00*16), (int)(1.00*16)},	//20
{18	,(int)(1.00*16), (int)(1.00*16)},
{18	,(int)(1.00*16), (int)(1.00*16)},
{19	,(int)(1.00*16), (int)(1.00*16)},
{19	,(int)(1.00*16), (int)(1.00*16)},
{20	,(int)(1.00*16), (int)(1.00*16)},
{21	,(int)(1.00*16), (int)(1.00*16)},
{22	,(int)(1.00*16), (int)(1.00*16)},
{22	,(int)(1.00*16), (int)(1.00*16)},
{23	,(int)(1.00*16), (int)(1.00*16)},
{24	,(int)(1.00*16), (int)(1.00*16)},	//30
{25	,(int)(1.00*16), (int)(1.00*16)},
{26	,(int)(1.00*16), (int)(1.00*16)},
{27	,(int)(1.00*16), (int)(1.00*16)},
{28	,(int)(1.00*16), (int)(1.00*16)},
{29	,(int)(1.00*16), (int)(1.00*16)},
{30	,(int)(1.00*16), (int)(1.00*16)},
{31	,(int)(1.00*16), (int)(1.00*16)},
{32	,(int)(1.00*16), (int)(1.00*16)},
{33	,(int)(1.00*16), (int)(1.00*16)},
{34	,(int)(1.00*16), (int)(1.00*16)},	//40
{35	,(int)(1.00*16), (int)(1.00*16)},
{36	,(int)(1.00*16), (int)(1.00*16)},
{38	,(int)(1.00*16), (int)(1.00*16)},
{39	,(int)(1.00*16), (int)(1.00*16)},
{40	,(int)(1.00*16), (int)(1.00*16)},
{42	,(int)(1.00*16), (int)(1.00*16)},
{43	,(int)(1.00*16), (int)(1.00*16)},
{45	,(int)(1.00*16), (int)(1.00*16)},
{46	,(int)(1.00*16), (int)(1.00*16)},
{48	,(int)(1.00*16), (int)(1.00*16)},	//50
{50	,(int)(1.00*16), (int)(1.00*16)},
{51	,(int)(1.00*16), (int)(1.00*16)},
{53	,(int)(1.00*16), (int)(1.00*16)},
{55	,(int)(1.00*16), (int)(1.00*16)},
{57	,(int)(1.00*16), (int)(1.00*16)},
{59	,(int)(1.00*16), (int)(1.00*16)},
{61	,(int)(1.00*16), (int)(1.00*16)},
{63	,(int)(1.00*16), (int)(1.00*16)},
{66	,(int)(1.00*16), (int)(1.00*16)},
{68	,(int)(1.00*16), (int)(1.00*16)},	//60
{70	,(int)(1.00*16), (int)(1.00*16)},
{73	,(int)(1.00*16), (int)(1.00*16)},
{75	,(int)(1.00*16), (int)(1.00*16)},
{78	,(int)(1.00*16), (int)(1.00*16)},
{81	,(int)(1.00*16), (int)(1.00*16)},
{84	,(int)(1.00*16), (int)(1.00*16)},
{87	,(int)(1.00*16), (int)(1.00*16)},
{90	,(int)(1.00*16), (int)(1.00*16)},
{93	,(int)(1.00*16), (int)(1.00*16)},
{96	,(int)(1.00*16), (int)(1.00*16)},	//70
{99	,(int)(1.00*16), (int)(1.00*16)},
{103,(int)(1.00*16), (int)(1.00*16)},
{106,(int)(1.00*16), (int)(1.00*16)},
{110,(int)(1.00*16), (int)(1.00*16)},
{114,(int)(1.00*16), (int)(1.00*16)},
{118,(int)(1.00*16), (int)(1.00*16)},
{122,(int)(1.00*16), (int)(1.00*16)},
{127,(int)(1.00*16), (int)(1.00*16)},
{131,(int)(1.00*16), (int)(1.00*16)},
{136,(int)(1.00*16), (int)(1.00*16)},	//80
{141,(int)(1.00*16), (int)(1.00*16)},
{145,(int)(1.00*16), (int)(1.00*16)},
{151,(int)(1.00*16), (int)(1.00*16)},
{156,(int)(1.00*16), (int)(1.00*16)},
{161,(int)(1.00*16), (int)(1.00*16)},
{167,(int)(1.00*16), (int)(1.00*16)},
{173,(int)(1.00*16), (int)(1.00*16)},
{179,(int)(1.00*16), (int)(1.00*16)},
{185,(int)(1.00*16), (int)(1.00*16)},
{192,(int)(1.00*16), (int)(1.00*16)},	//90
{199,(int)(1.00*16), (int)(1.00*16)},
{206,(int)(1.00*16), (int)(1.00*16)},
{213,(int)(1.00*16), (int)(1.00*16)},
{213,(int)(1.04*16), (int)(1.00*16)},
{213,(int)(1.07*16), (int)(1.00*16)},
{213,(int)(1.11*16), (int)(1.00*16)},
{213,(int)(1.15*16), (int)(1.00*16)},
{213,(int)(1.19*16), (int)(1.00*16)},
{213,(int)(1.23*16), (int)(1.00*16)},
{213,(int)(1.27*16), (int)(1.00*16)},	//100
{213,(int)(1.32*16), (int)(1.00*16)},
{213,(int)(1.37*16), (int)(1.00*16)},
{213,(int)(1.41*16), (int)(1.00*16)},
{213,(int)(1.46*16), (int)(1.00*16)},
{213,(int)(1.52*16), (int)(1.00*16)},
{213,(int)(1.57*16), (int)(1.00*16)},
{213,(int)(1.62*16), (int)(1.00*16)},
{213,(int)(1.68*16), (int)(1.00*16)},
{213,(int)(1.74*16), (int)(1.00*16)},
{213,(int)(1.80*16), (int)(1.00*16)},	//110
{213,(int)(1.87*16), (int)(1.00*16)},
{213,(int)(1.93*16), (int)(1.00*16)},
{427,(int)(1.00*16), (int)(1.00*16)},
{427,(int)(1.04*16), (int)(1.00*16)},
{427,(int)(1.07*16), (int)(1.00*16)},
{427,(int)(1.11*16), (int)(1.00*16)},
{427,(int)(1.15*16), (int)(1.00*16)},
{427,(int)(1.19*16), (int)(1.00*16)},
{427,(int)(1.23*16), (int)(1.00*16)},
{427,(int)(1.27*16), (int)(1.00*16)},	//120
{427,(int)(1.32*16), (int)(1.00*16)},
{427,(int)(1.37*16), (int)(1.00*16)},
{427,(int)(1.41*16), (int)(1.00*16)},
{427,(int)(1.46*16), (int)(1.00*16)},
{427,(int)(1.52*16), (int)(1.00*16)},
{427,(int)(1.57*16), (int)(1.00*16)},
{427,(int)(1.62*16), (int)(1.00*16)},
{427,(int)(1.68*16), (int)(1.00*16)},
{427,(int)(1.74*16), (int)(1.00*16)},
{427,(int)(1.80*16), (int)(1.00*16)},	//130
{427,(int)(1.87*16), (int)(1.00*16)},
{427,(int)(1.93*16), (int)(1.00*16)},
{640,(int)(1.32*16), (int)(1.00*16)},
{640,(int)(1.37*16), (int)(1.00*16)},
{640,(int)(1.41*16), (int)(1.00*16)},
{640,(int)(1.46*16), (int)(1.00*16)},
{640,(int)(1.52*16), (int)(1.00*16)},
{640,(int)(1.57*16), (int)(1.00*16)},
{640,(int)(1.62*16), (int)(1.00*16)},
{640,(int)(1.68*16), (int)(1.00*16)},	//140
{640,(int)(1.74*16), (int)(1.00*16)},
{640,(int)(1.80*16), (int)(1.00*16)},
{853,(int)(1.37*16), (int)(1.00*16)},
{853,(int)(1.41*16), (int)(1.00*16)},
{853,(int)(1.46*16), (int)(1.00*16)},
{853,(int)(1.52*16), (int)(1.00*16)},
{853,(int)(1.57*16), (int)(1.00*16)},
{853,(int)(1.62*16), (int)(1.00*16)},
{853,(int)(1.68*16), (int)(1.00*16)},
{853,(int)(1.74*16), (int)(1.00*16)},	//150
{853,(int)(1.80*16), (int)(1.00*16)},
{853,(int)(1.87*16), (int)(1.00*16)},
{853,(int)(1.93*16), (int)(1.00*16)},
{853,(int)(2.00*16), (int)(1.00*16)},
{853,(int)(2.07*16), (int)(1.00*16)},
{853,(int)(2.14*16), (int)(1.00*16)},
{853,(int)(2.22*16), (int)(1.00*16)},
{853,(int)(2.30*16), (int)(1.00*16)},
{853,(int)(2.38*16), (int)(1.00*16)},
{853,(int)(2.46*16), (int)(1.00*16)},	//160
{853,(int)(2.55*16), (int)(1.00*16)},
{853,(int)(2.64*16), (int)(1.00*16)},
{853,(int)(2.73*16), (int)(1.00*16)},
{853,(int)(2.83*16), (int)(1.00*16)},
{853,(int)(2.93*16), (int)(1.00*16)},
{853,(int)(3.03*16), (int)(1.00*16)},
{853,(int)(3.14*16), (int)(1.00*16)},
{853,(int)(3.25*16), (int)(1.00*16)},
{853,(int)(3.36*16), (int)(1.00*16)},
{853,(int)(3.48*16), (int)(1.00*16)},	//170
{853,(int)(3.61*16), (int)(1.00*16)},
{853,(int)(3.73*16), (int)(1.00*16)},
{853,(int)(3.86*16), (int)(1.00*16)},
{853,(int)(4.00*16), (int)(1.00*16)},
{853,(int)(4.14*16), (int)(1.00*16)},
{853,(int)(4.29*16), (int)(1.00*16)},
{853,(int)(4.44*16), (int)(1.00*16)},
{853,(int)(4.59*16), (int)(1.00*16)},
{853,(int)(4.76*16), (int)(1.00*16)},
{853,(int)(4.92*16), (int)(1.00*16)},	//180
{853,(int)(5.10*16), (int)(1.00*16)},
{853,(int)(5.28*16), (int)(1.00*16)},
{853,(int)(5.46*16), (int)(1.00*16)},
{853,(int)(5.66*16), (int)(1.00*16)},
{853,(int)(5.86*16), (int)(1.00*16)},
{853,(int)(6.06*16), (int)(1.00*16)},
{853,(int)(6.28*16), (int)(1.00*16)},
{853,(int)(6.50*16), (int)(1.00*16)},
{853,(int)(6.73*16), (int)(1.00*16)},
{853,(int)(6.96*16), (int)(1.00*16)},	//190
{853,(int)(7.21*16), (int)(1.00*16)},
{853,(int)(7.46*16), (int)(1.00*16)},
{853,(int)(7.73*16), (int)(1.00*16)},
{853,(int)(8.00*16), (int)(1.00*16)},
{853,(int)(8.28*16), (int)(1.00*16)},
{853,(int)(8.57*16), (int)(1.00*16)},
{853,(int)(8.88*16), (int)(1.00*16)},
{853,(int)(9.19*16), (int)(1.00*16)},
{853,(int)(9.51*16), (int)(1.00*16)},
{853,(int)(9.85*16), (int)(1.00*16)},	//200
{853,(int)(10.20*16), (int)(1.00*16)},
{853,(int)(10.56*16), (int)(1.00*16)},
{853,(int)(10.93*16), (int)(1.00*16)},
{853,(int)(11.31*16), (int)(1.00*16)},
{853,(int)(11.71*16), (int)(1.00*16)},
{853,(int)(12.13*16), (int)(1.00*16)},
{853,(int)(12.55*16), (int)(1.00*16)},
{853,(int)(13.00*16), (int)(1.00*16)},
{853,(int)(13.45*16), (int)(1.00*16)},
{853,(int)(13.93*16), (int)(1.00*16)},	//210
{853,(int)(14.42*16), (int)(1.00*16)},
{853,(int)(14.93*16), (int)(1.00*16)},
{853,(int)(15.45*16), (int)(1.00*16)},
{853,(int)(16.00*16), (int)(1.00*16)},	//214
};

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S jxh22_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	jxh22_handle = drv_l2_sccb_open(JXH22_ID, 8, 8);
	if(jxh22_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	jxh22_handle.devNumber = I2C_0;
	jxh22_handle.slaveAddr = JXH22_ID;
	jxh22_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void jxh22_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(jxh22_handle) {
		drv_l2_sccb_close(jxh22_handle);
		jxh22_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	jxh22_handle.slaveAddr = 0;
	jxh22_handle.clkRate = 0;
#endif
}

static INT32S jxh22_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(jxh22_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&jxh22_handle, data, 3);
#endif
}


static INT32S jxh22_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(jxh22_handle, reg, &data) >= 0) {
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
	if(drv_l1_i2c_bus_write(&jxh22_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&jxh22_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}

static INT32S jxh22_sccb_write_table(regval8_t *pTable)
{
    /*
    INT32U i;
	for (i=0; i<(sizeof(&pTable)/2); i++)
	{
		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(jxh22_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
		pTable++;
	}
	*/

	while(1) {
		if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
			break;
		}

		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(jxh22_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
		pTable++;
	}
DBG_PRINT("\r\n\r\n");
	return STATUS_OK;
}

static INT32S jxh22_sccb_read_table(regval8_t *pTable)
{
    INT8U *Rdata;
    /*
    INT32U i;
	for (i=0; i<(sizeof(pTable)/2); i++)
	{
		if(jxh22_sccb_read(pTable->reg_num, Rdata) < 0) {
			DBG_PRINT("sccb read fail.\r\n");
			continue;
		}
		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, *Rdata);
		pTable++;
	}
	*/

	while(1) {
		if(pTable->reg_num == 0x00 && pTable->value == 0xff) {
			break;
		}

		if(jxh22_sccb_read(pTable->reg_num, Rdata) < 0) {
			DBG_PRINT("sccb read fail.\r\n");
			continue;
		}
		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, *Rdata);
		pTable++;
	}

DBG_PRINT("\r\n\r\n");
	return STATUS_OK;
}

/*****************************************************************************************
+++++++++++++++++AEC/AGC
*****************************************************************************************/
static int jxh22_cvt_agc_gain(int agc_gain)
{

	INT32U jxh22_agc_gain, i;
    INT32U pow2,set_agc;

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

    switch(pow2)
    {
    	case 0:
    		set_agc = 0;
    	break;

    	case 1:
    		set_agc	= 1;
    	break;

    	case 2:
    		set_agc	= 3;
    	break;

    	case 3:
    		set_agc = 7;
    	break;

    	default:
    		set_agc	= 0;
    	break;
    }

    jxh22_agc_gain = (set_agc << 4) + agc_gain;

	return jxh22_agc_gain;
}

#if 0
static int jxh22_set_xfps_exposure_time(sensor_exposure_t *si)
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
    DBG_PRINT("[EV=%d, offset=%d]: time = 0x%x, analog gain =0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);

    if (ae_flag == 0)
    {
        // exposure time
        if(si ->time != pre_sensor_time)
        {
            pre_sensor_time = si->time;
            temp = si->time;

            t1 = (temp & 0xff);
            t2 = (temp >> 8) & 0x00ff;

            ret = jxh22_sccb_write(0x01, t1);
            if(ret < 0) return ret;
            ret = jxh22_sccb_write(0x02, t2);

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
            temp = jxh22_cvt_agc_gain(temp);
            t1 = temp & 0x007f;
            ret = jxh22_sccb_write(0x00, t1);
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
static int jxh22_set_xfps_exposure_time(sensor_exposure_t *si)
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
    DBG_PRINT("[EV=%d, offset=%d]: time = 0x%x, analog gain =0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);

    // exposure time
	if(si ->time != pre_sensor_time)
	{
		pre_sensor_time = si->time;
		temp = si->time;

		t1 = (temp & 0xff);
		t2 = (temp >> 8) & 0x00ff;
        #if 1 //group write
            ret = jxh22_sccb_write(0xC0, 0x01);
            if(ret < 0) return ret;
            ret = jxh22_sccb_write(0xC1, t1);
            if(ret < 0) return ret;
            ret = jxh22_sccb_write(0xC2, 0x02);
            if(ret < 0) return ret;
            ret = jxh22_sccb_write(0xC3, t2);
        #else
            ret = jxh22_sccb_write(0x01, t1);
            if(ret < 0) return ret;
            ret = jxh22_sccb_write(0x02, t2);
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
		temp = jxh22_cvt_agc_gain(temp);
		t1 = temp & 0x007f;
        #if 1   //group write
		ret = jxh22_sccb_write(0xC4, 0x00);
		if(ret < 0) return ret;
		ret = jxh22_sccb_write(0xC5, t1);
		#else
		ret = jxh22_sccb_write(0x00, t1);
		#endif
		if(ret < 0) {
            DBG_PRINT("ERR: write sensor gain = 0x%x, 0x%x, 0x%x !!!\r\n", t1, temp, si->analog_gain);
            return ret;
        }

	}
	#if 1//group write
        #if SENSOR_FLIP
        ret = jxh22_sccb_write(0x12, 0x38);   //group write enable
        #else
        ret = jxh22_sccb_write(0x12, 0x08);   //group write enable
        #endif
        if(ret < 0) {
            DBG_PRINT("ERR: triger group write FAIL!!!\r\n");
            return ret;
        }
    #endif

    return 0;
}
#endif

static void jxh22_set_ae(int ev_step)
{
    seInfo.ae_ev_step = ev_step;
    jxh22_set_xfps_exposure_time(&seInfo);
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)jxh22_set_ae;
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


//++++++++++++++++++++++++++++++++++++++++++++++++++++++
void jxh22_seinfo_init(void)
{
    seInfo.sensor_ev_idx = JXH22_30FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= JXH22_30FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= JXH22_30FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = JXH22_30FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = JXH22_30FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)jxh22_30fps_exp_time_gain_60Hz;

	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x00;

}

static void jxh22_dvp_sensor_init(void)
{
	DBG_PRINT("%s\r\n", __func__);

	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);
    /*Select csi data pin & ctrl*/
    //function_position_sel(ISP_CLKO__IOD7, ISP_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9, ISP_DATA2_9__IOB8_15, ISP_DATA0_1__IOB7_6);

    //JXH22_set_favorite();
	// mclk output
	//drv_l2_sensor_set_mclkout(jxh22_cdsp_dvp_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);

	// reguest sccb
	jxh22_sccb_open();

	// reset sensor
	jxh22_sccb_write_table((regval8_t *)jxh22_reset_table);
    osDelay(200);

	// init sensor
	jxh22_sccb_write_table((regval8_t *)jxh22_720P_30_init_table);

	//jxh22_sccb_read_table((regval8_t *)jxh22_720P_30_init_table);

    DBG_PRINT("Sensor JXH22 dvp init completed\r\n");
}


/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
//static void jxh22_cdsp_dvp_init(void)
void jxh22_cdsp_dvp_init(void)
{
	DBG_PRINT("%s\r\n", __func__);
	//ae init
	jxh22_seinfo_init();
    //cdsp init
	drv_l2_cdsp_open();
	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

    DBG_PRINT("cdspinit completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void jxh22_cdsp_dvp_uninit(void)
{
	DBG_PRINT("%s\r\n", __func__);
	// disable mclk
	drv_l2_sensor_set_mclkout(MCLK_NONE);

	// cdsp disable
	//drv_l2_cdsp_close();

	// release sccb
	jxh22_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void jxh22_cdsp_dvp_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);

	drv_l2_sensor_set_mclkout(jxh22_cdsp_dvp_ops.info[index].mclk);

    osDelay(300);

/*	//Only 720p
    switch(index)
	{
	case 0:
        jxh22_sccb_write_table((regval8_t *)jxh22_scale_vga_table);
		break;

	case 1:
		jxh22_sccb_write_table((regval8_t *)jxh22_720p_table);
		break;

	default:
		while(1);
	}
*/

    // set cdsp format
	format.image_source = C_CDSP_DVP;

	format.input_format =  jxh22_cdsp_dvp_ops.info[index].input_format;
	format.output_format = jxh22_cdsp_dvp_ops.info[index].output_format;
	target_w = jxh22_cdsp_dvp_ops.info[index].target_w;
	target_h = jxh22_cdsp_dvp_ops.info[index].target_h;
	format.hpixel = sensor_w = jxh22_cdsp_dvp_ops.info[index].sensor_w;
	format.vline = sensor_h = jxh22_cdsp_dvp_ops.info[index].sensor_h;
	format.hoffset = jxh22_cdsp_dvp_ops.info[index].hoffset;
	format.voffset = jxh22_cdsp_dvp_ops.info[index].voffset;
	format.sensor_timing_mode = jxh22_cdsp_dvp_ops.info[index].interface_mode;
	format.sensor_hsync_mode = jxh22_cdsp_dvp_ops.info[index].hsync_active;
	format.sensor_vsync_mode = jxh22_cdsp_dvp_ops.info[index].vsync_active;


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


	//Enable MCLK & Init Sensor
    jxh22_dvp_sensor_init();

    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    jxh22_set_xfps_exposure_time(&seInfo);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void jxh22_cdsp_dvp_stream_off(void)
{
	DBG_PRINT("%s:no function \r\n", __func__);
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* jxh22_cdsp_dvp_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&jxh22_cdsp_dvp_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t jxh22_cdsp_dvp_ops =
{
	SENSOR_JXH22_CDSP_DVP_NAME,		/* sensor name */
	jxh22_cdsp_dvp_init,
	jxh22_cdsp_dvp_uninit,
	jxh22_cdsp_dvp_stream_on,
	jxh22_cdsp_dvp_stream_off,
	jxh22_cdsp_dvp_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SBGGR8,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			320,						/* target width */
			240,  						/* target height */
			JXH22_WIDTH,				/* sensor width */
			JXH22_HEIGHT,  				/* sensor height */
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
			V4L2_PIX_FMT_SBGGR8,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			640,			        	/* target width */
			480, 			    		/* target height */
			JXH22_WIDTH,				/* sensor width */
			JXH22_HEIGHT, 				/* sensor height */
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
			30,							/* FPS in sensor */
			JXH22_OUT_WIDTH,			/* target width */
			JXH22_OUT_HEIGHT,  			/* target height */
			JXH22_WIDTH,				/* sensor width */
			JXH22_HEIGHT,  				/* sensor height */
			#if SENSOR_FLIP
			219,              			/* sensor h offset */
			1,      					/* sensor v offset */
			#else
			216,//2,  //216,	(Hsync_low)
			2,  //1,      //2,
			#endif
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_LOW,			/* hsync pin active level */
			MODE_ACTIVE_LOW,			/* vsync pin active level */
		},
		/* 4st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			JXH22_OUT_WIDTH,			   			/* target width */
			JXH22_OUT_HEIGHT,			    		/* target height */
			JXH22_WIDTH,				/* sensor width */
			JXH22_HEIGHT, 				/* sensor height */
			#if SENSOR_FLIP
			3,              			/* sensor h offset */
			2,      					/* sensor v offset */
			#else
			4,
			3,
			#endif							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};
#endif //(defined _SENSOR_JXH22_CDSP_DVP) && (_SENSOR_JXH22_CDSP_DVP == 1)
