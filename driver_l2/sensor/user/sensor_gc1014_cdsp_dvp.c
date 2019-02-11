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


#if (defined _SENSOR_GC1014_CDSP_DVP) && (_SENSOR_GC1014_CDSP_DVP == 1)

#include "drv_l2_user_calibration.h"
#include "drv_l2_user_preference.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define SENSOR_FLIP                         1

#define	GC1014_ID						0x78
#define GC1014_WIDTH					1280
#define GC1014_HEIGHT					720
#define GC1014_OUT_WIDTH				1280
#define GC1014_OUT_HEIGHT				720


#define GC1014_30FPS_50HZ_DAY_EV_IDX 			100
#define GC1014_30FPS_50HZ_NIGHT_EV_IDX			155
#define GC1014_30FPS_50HZ_EXP_TIME_TOTAL		204
#define GC1014_30FPS_50HZ_INIT_EV_IDX 			((GC1014_30FPS_50HZ_DAY_EV_IDX + GC1014_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define GC1014_30FPS_50HZ_MAX_EV_IDX			(GC1014_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define GC1014_30FPS_60HZ_DAY_EV_IDX 			100
#define GC1014_30FPS_60HZ_NIGHT_EV_IDX			170
#define GC1014_30FPS_60HZ_EXP_TIME_TOTAL		198
#define GC1014_30FPS_60HZ_INIT_EV_IDX 			((GC1014_30FPS_60HZ_DAY_EV_IDX + GC1014_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define GC1014_30FPS_60HZ_MAX_EV_IDX			 (GC1014_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define GC1014_24FPS_50HZ_DAY_EV_IDX 			138
#define GC1014_24FPS_50HZ_NIGHT_EV_IDX			194
#define GC1014_24FPS_50HZ_EXP_TIME_TOTAL		254
#define GC1014_24FPS_50HZ_INIT_EV_IDX 			((GC1014_24FPS_50HZ_DAY_EV_IDX + GC1014_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define GC1014_24FPS_50HZ_MAX_EV_IDX			(GC1014_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define GC1014_24FPS_60HZ_DAY_EV_IDX 			135
#define GC1014_24FPS_60HZ_NIGHT_EV_IDX			195
#define GC1014_24FPS_60HZ_EXP_TIME_TOTAL		255
#define GC1014_24FPS_60HZ_INIT_EV_IDX 			((GC1014_24FPS_60HZ_DAY_EV_IDX + GC1014_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define GC1014_24FPS_60HZ_MAX_EV_IDX			(GC1014_24FPS_60HZ_EXP_TIME_TOTAL - 10)


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
static INT32U gc1014_analog_gain = 0x100;

static INT8U ae_flag = 0;

#if SCCB_MODE == SCCB_GPIO
	static void *gc1014_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t gc1014_handle;
#endif

static const regval8_t gc1014_reset_table[] =
{	// SYS
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0xff, 0xff},
};

/***********************************
;INI Create Date : 2014/7/28
;Create By easonlin

;;Output Detail:
;;MCLK:24 MHz
;;PCLK:48
;;VCO:384
;;FrameW:1282
;;FrameH:752
************************************/
static const regval8_t gc1014_720P_30_init_table[] =
{	//48MHz
	/////////////////////////////////////////////////////
	////////////////SYS/////////////////////////////////
	/////////////////////////////////////////////////////
	{0xf2,0x02},
	{0xf6,0x00},
	{0xfc,0xc6},
	{0xf7,0xb9},
	{0xf8,0x03},
	{0xf9,0x0e},
	{0xfa,0x00},
	{0xfe,0x00},
	/////////////////////////////////////////////////////
	////////////////ANALOG CISCTL////////////////
	/////////////////////////////////////////////////////
	{0x03,0x02},
	{0x04,0xb5},
	{0x05,0x01},
	{0x06,0x80}, //HB
	{0x07,0x00},
	{0x08,0x08}, //VB
	{0x0d,0x02},
	{0x0e,0xf0},//d8
	{0x0f,0x05},
	{0x10,0x10},//08

	{0x11,0x00},
	{0x12,0x18},

#if SENSOR_FLIP
	{0x17, 0x17},		// [0]mirror [1]flip
#else
	{0x17, 0x14},		// [0]mirror [1]flip
#endif

	{0x19,0x06},
	{0x1b,0x4f},
	{0x1c,0x41},
	{0x1d,0xe0},
	{0x1e,0xfc},
	{0x1f,0x38},
	{0x20,0x81},
  	{0x21,0x2f},
	{0x22,0xc0},
	{0x23,0xf2},
	{0x24,0x2f},
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
	{0x3f,0x08},
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
	/////////////////////////////////////////////////////
	//////////////////////ISP///////////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x00},
	{0x8a,0x00},
	{0x8c,0x00},
	{0x8e,0x02},
	{0x8f,0x15},
	{0x90,0x01},
	{0x92,0x10},		//crop Y offset
	{0x94,0x02},
	{0x95,0x02},
	{0x96,0xD8},//0xD0, by gc @0820
	{0x97,0x05},
	{0x98,0x08},
	/////////////////////////////////////////////////////
	//////////////////////	MIPI	/////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x03},
	{0x01,0x00},
	{0x02,0x00},
	{0x03,0x00},
	{0x06,0x00},
	{0x10,0x00},
	{0x15,0x00},
	/////////////////////////////////////////////////////
	//////////////////////	BLK	/////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x00},
  	{0x18,0x0a},
	{0x1a,0x11},
  	{0x40,0x2b},
	{0x5e,0x00},
	{0x66,0x80},
	/////////////////////////////////////////////////////
	//////////////////////Dark SUN/////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x00},
	{0xcc,0x25},
    {0xce,0xf3},
	{0x3f,0x08},

	/////////////////////////////////////////////////////
	//////////////////////	Gain	/////////////////////
	/////////////////////////////////////////////////////
	{0xfe,0x00},
	{0xb0,0x50},
	{0xb3,0x40},
	{0xb4,0x40},
	{0xb5,0x40},
	{0xb6,0x00},
	/////////////////////////////////////////////////////
	//////////////////////,pad enable///////////////
	/////////////////////////////////////////////////////
	{0xf2,0x0f},
	{0xfe,0x00},
	//write done
	{0xff,0xff},
};


static const regval8_t gc1014_720p_table[] =
{

};

#if 0
static const regval8_t gc1014_suspend_table[] =
{

};

static const regval8_t gc1014_resume_table[] =
{

};
#endif

static const int gc1014__30fps_exp_time_gain_50Hz[GC1014_30FPS_50HZ_EXP_TIME_TOTAL][3] =
{ // {time, analog gain, digital gain}
	{8  ,(int)(1.00*100),(int)(1.00*256)},
	{9  ,(int)(1.00*100),(int)(1.00*256)},
	{9  ,(int)(1.00*100),(int)(1.00*256)},
	{9  ,(int)(1.00*100),(int)(1.00*256)},
	{10 ,(int)(1.00*100),(int)(1.00*256)},
	{10 ,(int)(1.00*100),(int)(1.00*256)},
	{10 ,(int)(1.00*100),(int)(1.00*256)},
	{11 ,(int)(1.00*100),(int)(1.00*256)},
	{11 ,(int)(1.00*100),(int)(1.00*256)},
	{11 ,(int)(1.00*100),(int)(1.00*256)},  //10
	{12 ,(int)(1.00*100),(int)(1.00*256)},
	{12 ,(int)(1.00*100),(int)(1.00*256)},
	{13 ,(int)(1.00*100),(int)(1.00*256)},
	{13 ,(int)(1.00*100),(int)(1.00*256)},
	{14 ,(int)(1.00*100),(int)(1.00*256)},
	{14 ,(int)(1.00*100),(int)(1.00*256)},
	{15 ,(int)(1.00*100),(int)(1.00*256)},
	{15 ,(int)(1.00*100),(int)(1.00*256)},
	{16 ,(int)(1.00*100),(int)(1.00*256)},
	{16 ,(int)(1.00*100),(int)(1.00*256)},  //20
	{17 ,(int)(1.00*100),(int)(1.00*256)},
	{17 ,(int)(1.00*100),(int)(1.00*256)},
	{18 ,(int)(1.00*100),(int)(1.00*256)},
	{19 ,(int)(1.00*100),(int)(1.00*256)},
	{19 ,(int)(1.00*100),(int)(1.00*256)},
	{20 ,(int)(1.00*100),(int)(1.00*256)},
	{21 ,(int)(1.00*100),(int)(1.00*256)},
	{21 ,(int)(1.00*100),(int)(1.00*256)},
	{22 ,(int)(1.00*100),(int)(1.00*256)},
	{23 ,(int)(1.00*100),(int)(1.00*256)},
	{24 ,(int)(1.00*100),(int)(1.00*256)},
	{25 ,(int)(1.00*100),(int)(1.00*256)},
	{25 ,(int)(1.00*100),(int)(1.00*256)},
	{26 ,(int)(1.00*100),(int)(1.00*256)},
	{27 ,(int)(1.00*100),(int)(1.00*256)},
	{28 ,(int)(1.00*100),(int)(1.00*256)},
	{29 ,(int)(1.00*100),(int)(1.00*256)},
	{30 ,(int)(1.00*100),(int)(1.00*256)},
	{31 ,(int)(1.00*100),(int)(1.00*256)},
	{32 ,(int)(1.00*100),(int)(1.00*256)},  //40
	{34 ,(int)(1.00*100),(int)(1.00*256)},
	{35 ,(int)(1.00*100),(int)(1.00*256)},
	{36 ,(int)(1.00*100),(int)(1.00*256)},
	{37 ,(int)(1.00*100),(int)(1.00*256)},
	{39 ,(int)(1.00*100),(int)(1.00*256)},
	{40 ,(int)(1.00*100),(int)(1.00*256)},
	{41 ,(int)(1.00*100),(int)(1.00*256)},
	{43 ,(int)(1.00*100),(int)(1.00*256)},
	{44 ,(int)(1.00*100),(int)(1.00*256)},
	{46 ,(int)(1.00*100),(int)(1.00*256)},
	{48 ,(int)(1.00*100),(int)(1.00*256)},
	{49 ,(int)(1.00*100),(int)(1.00*256)},
	{51 ,(int)(1.00*100),(int)(1.00*256)},
	{53 ,(int)(1.00*100),(int)(1.00*256)},
	{55 ,(int)(1.00*100),(int)(1.00*256)},
	{57 ,(int)(1.00*100),(int)(1.00*256)},
	{58 ,(int)(1.00*100),(int)(1.00*256)},
	{61 ,(int)(1.00*100),(int)(1.00*256)},
	{63 ,(int)(1.00*100),(int)(1.00*256)},
	{65 ,(int)(1.00*100),(int)(1.00*256)},  //60
	{67 ,(int)(1.00*100),(int)(1.00*256)},
	{70 ,(int)(1.00*100),(int)(1.00*256)},
	{72 ,(int)(1.00*100),(int)(1.00*256)},
	{75 ,(int)(1.00*100),(int)(1.00*256)},
	{77 ,(int)(1.00*100),(int)(1.00*256)},
	{80 ,(int)(1.00*100),(int)(1.00*256)},
	{83 ,(int)(1.00*100),(int)(1.00*256)},
	{86 ,(int)(1.00*100),(int)(1.00*256)},
	{89 ,(int)(1.00*100),(int)(1.00*256)},
	{92 ,(int)(1.00*100),(int)(1.00*256)},
	{95 ,(int)(1.00*100),(int)(1.00*256)},
	{98 ,(int)(1.00*100),(int)(1.00*256)},
	{102,(int)(1.00*100),(int)(1.00*256)},
	{105,(int)(1.00*100),(int)(1.00*256)},
	{109,(int)(1.00*100),(int)(1.00*256)},
	{113,(int)(1.00*100),(int)(1.00*256)},
	{117,(int)(1.00*100),(int)(1.00*256)},
	{121,(int)(1.00*100),(int)(1.00*256)},
	{125,(int)(1.00*100),(int)(1.00*256)},
	{130,(int)(1.00*100),(int)(1.00*256)},  //80
	{134,(int)(1.00*100),(int)(1.00*256)},
	{139,(int)(1.00*100),(int)(1.00*256)},
	{144,(int)(1.00*100),(int)(1.00*256)},
	{149,(int)(1.00*100),(int)(1.00*256)},
	{154,(int)(1.00*100),(int)(1.00*256)},
	{160,(int)(1.00*100),(int)(1.00*256)},
	{165,(int)(1.00*100),(int)(1.00*256)},
	{171,(int)(1.00*100),(int)(1.00*256)},
	{177,(int)(1.00*100),(int)(1.00*256)},
	{184,(int)(1.00*100),(int)(1.00*256)},  //90
	{190,(int)(1.00*100),(int)(1.00*256)},
	{197,(int)(1.00*100),(int)(1.00*256)},
	{204,(int)(1.00*100),(int)(1.00*256)},
	{211,(int)(1.00*100),(int)(1.00*256)},
	{218,(int)(1.00*100),(int)(1.00*256)},
	// 10ms
	{226,(int)(1.00*100),(int)(1.00*256)},  //96,daylight
	{226,(int)(1.04*100),(int)(1.00*256)},
	{226,(int)(1.06*100),(int)(1.00*256)},
	{226,(int)(1.11*100),(int)(1.00*256)},
	{226,(int)(1.15*100),(int)(1.00*256)},  //100
	{226,(int)(1.19*100),(int)(1.00*256)},
	{226,(int)(1.23*100),(int)(1.00*256)},
	{226,(int)(1.27*100),(int)(1.00*256)},
	{226,(int)(1.31*100),(int)(1.00*256)},
	{226,(int)(1.37*100),(int)(1.00*256)},
	{226,(int)(1.41*100),(int)(1.00*256)},
	{226,(int)(1.46*100),(int)(1.00*256)},
	{226,(int)(1.52*100),(int)(1.00*256)},
	{226,(int)(1.57*100),(int)(1.00*256)},
	{226,(int)(1.62*100),(int)(1.00*256)},  //110
	{226,(int)(1.68*100),(int)(1.00*256)},
	{226,(int)(1.74*100),(int)(1.00*256)},
	{226,(int)(1.80*100),(int)(1.00*256)},
	//20 ms
	{452,(int)(1.00*100),(int)(1.00*256)},
	{452,(int)(1.04*100),(int)(1.00*256)},
	{452,(int)(1.06*100),(int)(1.00*256)},
	{452,(int)(1.11*100),(int)(1.00*256)},
	{452,(int)(1.15*100),(int)(1.00*256)},
	{452,(int)(1.19*100),(int)(1.00*256)},
	{452,(int)(1.23*100),(int)(1.00*256)}, //120
	{452,(int)(1.27*100),(int)(1.00*256)},
	{452,(int)(1.31*100),(int)(1.00*256)},
	{452,(int)(1.37*100),(int)(1.00*256)},
	{452,(int)(1.41*100),(int)(1.00*256)},
	{452,(int)(1.46*100),(int)(1.00*256)},
	{452,(int)(1.52*100),(int)(1.00*256)},
	{452,(int)(1.57*100),(int)(1.00*256)},
	{452,(int)(1.62*100),(int)(1.00*256)},
	{452,(int)(1.68*100),(int)(1.00*256)},
	{452,(int)(1.74*100),(int)(1.00*256)}, //130
	{452,(int)(1.80*100),(int)(1.00*256)},
	{452,(int)(1.87*100),(int)(1.00*256)},
	{452,(int)(1.93*100),(int)(1.00*256)},
	{452,(int)(2.00*100),(int)(1.00*256)},
	//30 ms
	{678,(int)(1.48*100),(int)(1.00*256)},
	{678,(int)(1.54*100),(int)(1.00*256)},
	{678,(int)(1.60*100),(int)(1.00*256)},
	{678,(int)(1.76*100),(int)(1.00*256)},
	{678,(int)(1.82*100),(int)(1.00*256)},
	{678,(int)(1.88*100),(int)(1.00*256)},  //140
	{678,(int)(1.94*100),(int)(1.00*256)},
	{678,(int)(2.00*100),(int)(1.00*256)},
	{678,(int)(2.06*100),(int)(1.00*256)},
	{678,(int)(2.12*100),(int)(1.00*256)},
	{678,(int)(2.18*100),(int)(1.00*256)},
	{678,(int)(2.24*100),(int)(1.00*256)},
	{678,(int)(2.30*100),(int)(1.00*256)},
	{678,(int)(2.36*100),(int)(1.00*256)},
	//40ms
	{904,(int)(1.83*100),(int)(1.00*256)},
	{904,(int)(1.89*100),(int)(1.00*256)},  //150
	{904,(int)(1.95*100),(int)(1.00*256)},
	{904,(int)(2.01*100),(int)(1.00*256)},
	{904,(int)(2.07*100),(int)(1.00*256)},
	{904,(int)(2.14*100),(int)(1.00*256)},
	{904,(int)(2.22*100),(int)(1.00*256)},
	{904,(int)(2.30*100),(int)(1.00*256)},
	{904,(int)(2.38*100),(int)(1.00*256)},
	{904,(int)(2.46*100),(int)(1.00*256)},
	{904,(int)(2.55*100),(int)(1.00*256)},
	{904,(int)(2.64*100),(int)(1.00*256)},  //160
	{904,(int)(2.73*100),(int)(1.00*256)},
	{904,(int)(2.83*100),(int)(1.00*256)},
	{904,(int)(2.93*100),(int)(1.00*256)},
	{904,(int)(3.03*100),(int)(1.00*256)},
	{904,(int)(3.14*100),(int)(1.00*256)},
	{904,(int)(3.25*100),(int)(1.00*256)},
	{904,(int)(3.36*100),(int)(1.00*256)},
	{904,(int)(3.48*100),(int)(1.00*256)},
	{904,(int)(3.61*100),(int)(1.00*256)},
	{904,(int)(3.73*100),(int)(1.00*256)}, //170
	{904,(int)(3.86*100),(int)(1.00*256)},
	{904,(int)(4.00*100),(int)(1.00*256)},   // GC_BLK night
	{904,(int)(4.14*100),(int)(1.00*256)},
	{904,(int)(4.29*100),(int)(1.00*256)},
	{904,(int)(4.44*100),(int)(1.00*256)},
	{904,(int)(4.59*100),(int)(1.00*256)},
	{904,(int)(4.76*100),(int)(1.00*256)},
	{904,(int)(4.92*100),(int)(1.00*256)},
	{904,(int)(5.10*100),(int)(1.00*256)},
	{904,(int)(5.28*100),(int)(1.00*256)}, //180
	{904,(int)(5.46*100),(int)(1.00*256)},
	{904,(int)(5.66*100),(int)(1.00*256)},
	{904,(int)(5.86*100),(int)(1.00*256)},
	{904,(int)(6.06*100),(int)(1.00*256)},
	{904,(int)(6.28*100),(int)(1.00*256)},
	{904,(int)(6.50*100),(int)(1.00*256)},
	{904,(int)(6.73*100),(int)(1.00*256)},
	{904,(int)(6.96*100),(int)(1.00*256)},
	{904,(int)(7.21*100),(int)(1.00*256)}, //189
	{904,(int)(7.46*100),(int)(1.00*256)}, //190
	{904,(int)(7.73*100),(int)(1.00*256)},
	{904,(int)(8.00*100),(int)(1.00*256)},
	{904,(int)(8.28*100),(int)(1.00*256)},
	{904,(int)(8.57*100),(int)(1.00*256)},
	{904,(int)(8.88*100),(int)(1.00*256)},
	{904,(int)(9.19*100),(int)(1.00*256)},
	{904,(int)(9.51*100),(int)(1.00*256)},
	{904,(int)(9.85*100),(int)(1.00*256)},
	{904,(int)(9.91*100),(int)(1.00*256)},
	{904,(int)(9.97*100),(int)(1.00*256)}, //200
	{904,(int)(10.03*100),(int)(1.00*256)},
	{904,(int)(10.09*100),(int)(1.00*256)},
	{904,(int)(10.15*100),(int)(1.00*256)} //203
};

static const  int gc1014_30fps_exp_time_gain_60Hz[GC1014_30FPS_60HZ_EXP_TIME_TOTAL][3] =
{ // {time, analog gain, digital gain}
	{8	,(int)(1.00*100), (int)(1.00*256)},//1
	{9	,(int)(1.00*100), (int)(1.00*256)},
	{9	,(int)(1.00*100), (int)(1.00*256)},
	{9	,(int)(1.00*100), (int)(1.00*256)},
	{10	,(int)(1.00*100), (int)(1.00*256)},
	{10	,(int)(1.00*100), (int)(1.00*256)},
	{10	,(int)(1.00*100), (int)(1.00*256)},
	{11	,(int)(1.00*100), (int)(1.00*256)},
	{11	,(int)(1.00*100), (int)(1.00*256)},
	{11	,(int)(1.00*100), (int)(1.00*256)},//10
	{12	,(int)(1.00*100), (int)(1.00*256)},
	{12	,(int)(1.00*100), (int)(1.00*256)},
	{13	,(int)(1.00*100), (int)(1.00*256)},
	{13	,(int)(1.00*100), (int)(1.00*256)},
	{13	,(int)(1.00*100), (int)(1.00*256)},
	{14	,(int)(1.00*100), (int)(1.00*256)},
	{14	,(int)(1.00*100), (int)(1.00*256)},
	{15	,(int)(1.00*100), (int)(1.00*256)},
	{16	,(int)(1.00*100), (int)(1.00*256)},
	{16	,(int)(1.00*100), (int)(1.00*256)},//20
	{17	,(int)(1.00*100), (int)(1.00*256)},
	{17	,(int)(1.00*100), (int)(1.00*256)},
	{18	,(int)(1.00*100), (int)(1.00*256)},
	{18	,(int)(1.00*100), (int)(1.00*256)},
	{19	,(int)(1.00*100), (int)(1.00*256)},
	{20	,(int)(1.00*100), (int)(1.00*256)},
	{20	,(int)(1.00*100), (int)(1.00*256)},
	{21	,(int)(1.00*100), (int)(1.00*256)},
	{22	,(int)(1.00*100), (int)(1.00*256)},
	{23	,(int)(1.00*100), (int)(1.00*256)},//30
	{24	,(int)(1.00*100), (int)(1.00*256)},
	{24	,(int)(1.00*100), (int)(1.00*256)},
	{25	,(int)(1.00*100), (int)(1.00*256)},
	{26	,(int)(1.00*100), (int)(1.00*256)},
	{27	,(int)(1.00*100), (int)(1.00*256)},
	{28	,(int)(1.00*100), (int)(1.00*256)},
	{29	,(int)(1.00*100), (int)(1.00*256)},
	{30	,(int)(1.00*100), (int)(1.00*256)},
	{31	,(int)(1.00*100), (int)(1.00*256)},
	{32	,(int)(1.00*100), (int)(1.00*256)},//40
	{33	,(int)(1.00*100), (int)(1.00*256)},
	{34	,(int)(1.00*100), (int)(1.00*256)},
	{36	,(int)(1.00*100), (int)(1.00*256)},
	{37	,(int)(1.00*100), (int)(1.00*256)},
	{38	,(int)(1.00*100), (int)(1.00*256)},
	{40	,(int)(1.00*100), (int)(1.00*256)},
	{41	,(int)(1.00*100), (int)(1.00*256)},
	{42	,(int)(1.00*100), (int)(1.00*256)},
	{44	,(int)(1.00*100), (int)(1.00*256)},
	{45	,(int)(1.00*100), (int)(1.00*256)},//50
	{47	,(int)(1.00*100), (int)(1.00*256)},
	{49	,(int)(1.00*100), (int)(1.00*256)},
	{50	,(int)(1.00*100), (int)(1.00*256)},
	{52	,(int)(1.00*100), (int)(1.00*256)},
	{54	,(int)(1.00*100), (int)(1.00*256)},
	{56	,(int)(1.00*100), (int)(1.00*256)},
	{58	,(int)(1.00*100), (int)(1.00*256)},
	{60	,(int)(1.00*100), (int)(1.00*256)},
	{62	,(int)(1.00*100), (int)(1.00*256)},
	{64	,(int)(1.00*100), (int)(1.00*256)},//60
	{66	,(int)(1.00*100), (int)(1.00*256)},
	{69	,(int)(1.00*100), (int)(1.00*256)},
	{71	,(int)(1.00*100), (int)(1.00*256)},
	{74	,(int)(1.00*100), (int)(1.00*256)},
	{76	,(int)(1.00*100), (int)(1.00*256)},
	{79	,(int)(1.00*100), (int)(1.00*256)},
	{82	,(int)(1.00*100), (int)(1.00*256)},
	{85	,(int)(1.00*100), (int)(1.00*256)},
	{88	,(int)(1.00*100), (int)(1.00*256)},
	{91	,(int)(1.00*100), (int)(1.00*256)},//70
	{94	,(int)(1.00*100), (int)(1.00*256)},
	{97	,(int)(1.00*100), (int)(1.00*256)},
	{101,(int)(1.00*100), (int)(1.00*256)},
	{104,(int)(1.00*100), (int)(1.00*256)},
	{108,(int)(1.00*100), (int)(1.00*256)},
	{112,(int)(1.00*100), (int)(1.00*256)},
	{116,(int)(1.00*100), (int)(1.00*256)},
	{120,(int)(1.00*100), (int)(1.00*256)},
	{124,(int)(1.00*100), (int)(1.00*256)},
	{128,(int)(1.00*100), (int)(1.00*256)},//80
	{133,(int)(1.00*100), (int)(1.00*256)},
	{138,(int)(1.00*100), (int)(1.00*256)},
	{142,(int)(1.00*100), (int)(1.00*256)},
	{148,(int)(1.00*100), (int)(1.00*256)},
	{153,(int)(1.00*100), (int)(1.00*256)},
	{158,(int)(1.00*100), (int)(1.00*256)},
	{164,(int)(1.00*100), (int)(1.00*256)},
	{169,(int)(1.00*100), (int)(1.00*256)},
	{175,(int)(1.00*100), (int)(1.00*256)},
	{182,(int)(1.00*100), (int)(1.00*256)},//90
	{188,(int)(1.00*100), (int)(1.00*256)},
	{188,(int)(1.04*100), (int)(1.00*256)},
	{188,(int)(1.06*100), (int)(1.00*256)},
	{188,(int)(1.11*100), (int)(1.00*256)},
	{188,(int)(1.15*100), (int)(1.00*256)},//95,daylight
	{188,(int)(1.19*100), (int)(1.00*256)},
	{188,(int)(1.23*100), (int)(1.00*256)},
	{188,(int)(1.27*100), (int)(1.00*256)},
	{188,(int)(1.31*100), (int)(1.00*256)},
	{188,(int)(1.37*100), (int)(1.00*256)},//100
	{188,(int)(1.41*100), (int)(1.00*256)},
	{188,(int)(1.46*100), (int)(1.00*256)},
	{188,(int)(1.52*100), (int)(1.00*256)},
	{188,(int)(1.57*100), (int)(1.00*256)},
	{188,(int)(1.62*100), (int)(1.00*256)},
	{188,(int)(1.68*100), (int)(1.00*256)},
	{188,(int)(1.74*100), (int)(1.00*256)},
	{210,(int)(1.68*100), (int)(1.00*256)},
	{220,(int)(1.65*100), (int)(1.00*256)},
	{250,(int)(1.50*100), (int)(1.00*256)},//110
	{280,(int)(1.40*100), (int)(1.00*256)},
	{310,(int)(1.30*100), (int)(1.00*256)},
	{340,(int)(1.23*100), (int)(1.00*256)},
	{376,(int)(1.11*100), (int)(1.00*256)},
	{376,(int)(1.15*100), (int)(1.00*256)},
	{376,(int)(1.19*100), (int)(1.00*256)},
	{376,(int)(1.23*100), (int)(1.00*256)},
	{376,(int)(1.27*100), (int)(1.00*256)},
	{376,(int)(1.31*100), (int)(1.00*256)},
	{376,(int)(1.37*100), (int)(1.00*256)},//120
	{376,(int)(1.41*100), (int)(1.00*256)},
	{376,(int)(1.46*100), (int)(1.00*256)},//121,init
	{376,(int)(1.52*100), (int)(1.00*256)},
	{376,(int)(1.57*100), (int)(1.00*256)},
	{376,(int)(1.62*100), (int)(1.00*256)},
	{376,(int)(1.68*100), (int)(1.00*256)},
	{376,(int)(1.74*100), (int)(1.00*256)},
	{376,(int)(1.80*100), (int)(1.00*256)},
	{406,(int)(1.73*100), (int)(1.00*256)},
	{440,(int)(1.65*100), (int)(1.00*256)},//130
	{475,(int)(1.57*100), (int)(1.00*256)},//
	{516,(int)(1.50*100), (int)(1.00*256)},
	{546,(int)(1.46*100), (int)(1.00*256)},
	{564,(int)(1.46*100), (int)(1.00*256)},
	{564,(int)(1.52*100), (int)(1.00*256)},
	{564,(int)(1.57*100), (int)(1.00*256)},
	{564,(int)(1.62*100), (int)(1.00*256)},
	{564,(int)(1.68*100), (int)(1.00*256)},
	{564,(int)(1.74*100), (int)(1.00*256)},
	{564,(int)(1.80*100), (int)(1.00*256)},//140
	{600,(int)(1.76*100), (int)(1.00*256)},
	{630,(int)(1.74*100), (int)(1.00*256)},
	{670,(int)(1.71*100), (int)(1.00*256)},
	{700,(int)(1.69*100), (int)(1.00*256)},
	{752,(int)(1.62*100), (int)(1.00*256)},
	{752,(int)(1.68*100), (int)(1.00*256)},
	{752,(int)(1.74*100), (int)(1.00*256)},
	{752,(int)(1.80*100), (int)(1.00*256)},
	{752,(int)(1.87*100), (int)(1.00*256)},
	{752,(int)(1.93*100), (int)(1.00*256)},//150, night
	{752,(int)(2.00*100), (int)(1.00*256)},
	{752,(int)(2.07*100), (int)(1.00*256)},
	{752,(int)(2.14*100), (int)(1.00*256)},
	{752,(int)(2.22*100), (int)(1.00*256)},
	{752,(int)(2.30*100), (int)(1.00*256)},
	{752,(int)(2.38*100), (int)(1.00*256)},
	{752,(int)(2.46*100), (int)(1.00*256)},
	{752,(int)(2.55*100), (int)(1.00*256)},
	{752,(int)(2.64*100), (int)(1.00*256)},
	{752,(int)(2.73*100), (int)(1.00*256)},//160
	{752,(int)(2.83*100), (int)(1.00*256)},
	{752,(int)(2.93*100), (int)(1.00*256)},
	{752,(int)(3.03*100), (int)(1.00*256)},
	{752,(int)(3.14*100), (int)(1.00*256)},
	{752,(int)(3.25*100), (int)(1.00*256)},
	{752,(int)(3.36*100), (int)(1.00*256)},
	{752,(int)(3.48*100), (int)(1.00*256)},
	{752,(int)(3.61*100), (int)(1.00*256)},
	{752,(int)(3.73*100), (int)(1.00*256)},
	{752,(int)(3.86*100), (int)(1.00*256)},//170
	{752,(int)(4.00*100), (int)(1.00*256)},
	{752,(int)(4.14*100), (int)(1.00*256)},
	{752,(int)(4.29*100), (int)(1.00*256)},
	{752,(int)(4.44*100), (int)(1.00*256)},
	{752,(int)(4.59*100), (int)(1.00*256)},
	{752,(int)(4.76*100), (int)(1.00*256)},
	{752,(int)(4.92*100), (int)(1.00*256)},
	{752,(int)(5.10*100), (int)(1.00*256)},
	{752,(int)(5.28*100), (int)(1.00*256)},
	{752,(int)(5.46*100), (int)(1.00*256)},//180
	{752,(int)(5.66*100), (int)(1.00*256)},
	{752,(int)(5.86*100), (int)(1.00*256)},
	{752,(int)(6.06*100), (int)(1.00*256)},
	{752,(int)(6.28*100), (int)(1.00*256)},
	{752,(int)(6.50*100), (int)(1.00*256)},
	{752,(int)(6.73*100), (int)(1.00*256)},
	{752,(int)(6.96*100), (int)(1.00*256)},
	{752,(int)(7.21*100), (int)(1.00*256)},
	{752,(int)(7.46*100), (int)(1.00*256)},
	{752,(int)(7.73*100), (int)(1.00*256)},//190
	{752,(int)(8.00*100), (int)(1.00*256)},
	{752,(int)(8.28*100), (int)(1.00*256)},
	{752,(int)(8.57*100), (int)(1.00*256)},
	{752,(int)(8.88*100), (int)(1.00*256)},
	{752,(int)(9.19*100), (int)(1.00*256)},
	{752,(int)(9.51*100), (int)(1.00*256)},
	{752,(int)(9.85*100), (int)(1.00*256)} //197
};

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S gc1014_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	gc1014_handle = drv_l2_sccb_open(GC1014_ID, 8, 8);
	if(gc1014_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	gc1014_handle.devNumber = I2C_0;
	gc1014_handle.slaveAddr = GC1014_ID;
	gc1014_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void gc1014_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(gc1014_handle) {
		drv_l2_sccb_close(gc1014_handle);
		gc1014_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	gc1014_handle.slaveAddr = 0;
	gc1014_handle.clkRate = 0;
#endif
}

static INT32S gc1014_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(gc1014_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&gc1014_handle, data, 3);
#endif
}


static INT32S gc1014_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(gc1014_handle, reg, &data) >= 0) {
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
	if(drv_l1_i2c_bus_write(&gc1014_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&gc1014_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}

static INT32S gc1014_sccb_write_table(regval8_t *pTable)
{
	while(1) {
		if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
			break;
		}

		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(gc1014_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
		pTable++;
	}
DBG_PRINT("\r\n\r\n");
	return STATUS_OK;
}

static INT32S gc1014_sccb_read_table(regval8_t *pTable)
{
    INT8U *Rdata;

	while(1) {
		if(pTable->reg_num == 0x00 && pTable->value == 0xff) {
			break;
		}

		if(gc1014_sccb_read(pTable->reg_num, Rdata) < 0) {
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
static int gc1014_cvt_agc_gain(int agc_gain)
{

	INT32U Analog_Multiple[9]={1000, 1706, 2000, 3414, 4026,6870,8127,13869,19610}; //GC1014-TEST-GAIN

	INT32U Analog_Index;
	INT32U Digital_Gain;
	INT32U Decimal;
	INT32U ret;

	gc1014_analog_gain = agc_gain*10;

    #if 0   //Abner-test-20150727
	    if(gc1014_analog_gain>9500)  gc1014_analog_gain=9500; //limit max gain , added by  simon 20141208
	#endif

	Analog_Index=0;

	while(Analog_Index<9)
	{
	  if(gc1014_analog_gain<Analog_Multiple[Analog_Index])
	  {
		break;
	  }
	  else
	  {
		Analog_Index++;
	  }
	}

	Digital_Gain = gc1014_analog_gain*1000/Analog_Multiple[Analog_Index-1];
	Decimal=(Digital_Gain*64)/1000;

#if 1
	ret = gc1014_sccb_write(0xb1,  Decimal>>6);
	if(ret < 0) return ret;
	ret = gc1014_sccb_write(0xb2,  (Decimal<<2)&0xfc);
	if(ret < 0) return ret;
	ret = gc1014_sccb_write(0xb6,   Analog_Index-1);
	if(ret < 0) return ret;
#else
	ret = gc1014_sccb_write(0xb1, 0x01);
	if(ret < 0) return ret;
	ret = gc1014_sccb_write(0xb2, 0x00);
	if(ret < 0) return ret;
	ret = gc1014_sccb_write(0xb6, 0x06);
	if(ret < 0) return ret;
#endif

}


static int gc1014_set_xfps_exposure_time(sensor_exposure_t *si)
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
    DBG_PRINT("[EV=%d, offset=%d]: time = %d, analog gain =0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);

    // exposure time
	if(si ->time != pre_sensor_time)
	{
		pre_sensor_time = si->time;
		temp = si->time;

		if (temp < 0x0A) temp = 0x0A;//added by gc fae abner

		t1 = (temp & 0xff);
		t2 = (temp >> 8) & 0x00ff;

        ret = gc1014_sccb_write(0x04, t1);
        if(ret < 0) return ret;
        ret = gc1014_sccb_write(0x03, t2);
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
		ret = gc1014_cvt_agc_gain(temp);
		if(ret < 0) {
		    DBG_PRINT("ERR: write sensor analog agin = 0x%x!!!\r\n", temp);
            return ret;
		}
	}

    return 0;
}


static void gc1014_set_ae(int ev_step)
{
    seInfo.ae_ev_step = ev_step;
    gc1014_set_xfps_exposure_time(&seInfo);
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)gc1014_set_ae;
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
void gc1014_seinfo_init(void)
{
    seInfo.sensor_ev_idx = GC1014_30FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= GC1014_30FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= GC1014_30FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = GC1014_30FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = GC1014_30FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)gc1014_30fps_exp_time_gain_60Hz;

	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x00;

}

static void gc1014_dvp_sensor_init(void)
{
	DBG_PRINT("%s\r\n", __func__);

	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);
    /*Select csi data pin & ctrl*/
    //function_position_sel(ISP_CLKO__IOD7, ISP_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9, ISP_DATA2_9__IOB8_15, ISP_DATA0_1__IOB7_6);

    //GC1014_set_favorite();
	// mclk output
	//drv_l2_sensor_set_mclkout(gc1014_cdsp_dvp_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);

	// reguest sccb
	gc1014_sccb_open();

	// reset sensor
	gc1014_sccb_write_table((regval8_t *)gc1014_reset_table);
    osDelay(200);

	// init sensor
	gc1014_sccb_write_table((regval8_t *)gc1014_720P_30_init_table);

	//gc1014_sccb_read_table((regval8_t *)gc1014_720P_30_init_table);

    DBG_PRINT("Sensor GC1004 dvp init completed\r\n");
}


/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
//static void gc1014_cdsp_dvp_init(void)
void gc1014_cdsp_dvp_init(void)
{
	DBG_PRINT("%s\r\n", __func__);
	//ae init
	gc1014_seinfo_init();
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
static void gc1014_cdsp_dvp_uninit(void)
{
	DBG_PRINT("%s\r\n", __func__);
	// disable mclk
	drv_l2_sensor_set_mclkout(MCLK_NONE);

	// cdsp disable
	//drv_l2_cdsp_close();

	// release sccb
	gc1014_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void gc1014_cdsp_dvp_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);

	drv_l2_sensor_set_mclkout(gc1014_cdsp_dvp_ops.info[index].mclk);

    osDelay(300);

/*	//Only 720p
    switch(index)
	{
	case 0:
        gc1014_sccb_write_table((regval8_t *)gc1014_scale_vga_table);
		break;

	case 1:
		gc1014_sccb_write_table((regval8_t *)gc1014_720p_table);
		break;

	default:
		while(1);
	}
*/

    // set cdsp format
	format.image_source = C_CDSP_DVP;

	format.input_format =  gc1014_cdsp_dvp_ops.info[index].input_format;
	format.output_format = gc1014_cdsp_dvp_ops.info[index].output_format;
	target_w = gc1014_cdsp_dvp_ops.info[index].target_w;
	target_h = gc1014_cdsp_dvp_ops.info[index].target_h;
	format.hpixel = sensor_w = gc1014_cdsp_dvp_ops.info[index].sensor_w;
	format.vline = sensor_h = gc1014_cdsp_dvp_ops.info[index].sensor_h;
	format.hoffset = gc1014_cdsp_dvp_ops.info[index].hoffset;
	format.voffset = gc1014_cdsp_dvp_ops.info[index].voffset;
	format.sensor_timing_mode = gc1014_cdsp_dvp_ops.info[index].interface_mode;
	format.sensor_hsync_mode = gc1014_cdsp_dvp_ops.info[index].hsync_active;
	format.sensor_vsync_mode = gc1014_cdsp_dvp_ops.info[index].vsync_active;


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
    gc1014_dvp_sensor_init();

    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    gc1014_set_xfps_exposure_time(&seInfo);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void gc1014_cdsp_dvp_stream_off(void)
{
	DBG_PRINT("%s:no function \r\n", __func__);
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* gc1014_cdsp_dvp_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&gc1014_cdsp_dvp_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t gc1014_cdsp_dvp_ops =
{
	SENSOR_GC1014_CDSP_DVP_NAME,		/* sensor name */
	gc1014_cdsp_dvp_init,
	gc1014_cdsp_dvp_uninit,
	gc1014_cdsp_dvp_stream_on,
	gc1014_cdsp_dvp_stream_off,
	gc1014_cdsp_dvp_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SBGGR8,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			320,						/* target width */
			240,  						/* target height */
			GC1014_WIDTH,				/* sensor width */
			GC1014_HEIGHT,  				/* sensor height */
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
			GC1014_WIDTH,				/* sensor width */
			GC1014_HEIGHT, 				/* sensor height */
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
			GC1014_OUT_WIDTH,			/* target width */
			GC1014_OUT_HEIGHT,  		/* target height */
			GC1014_WIDTH,				/* sensor width */
			GC1014_HEIGHT,  			/* sensor height */
			#if SENSOR_FLIP
			11,							/* sensor h offset */
			1,							/* sensor v offset */
			#else
			12,
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
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			320,			   			/* target width */
			240,			    		/* target height */
			GC1014_WIDTH,				/* sensor width */
			GC1014_HEIGHT, 				/* sensor height */
			#if SENSOR_FLIP
			11,							/* sensor h offset */
			1,							/* sensor v offset */
			#else
			12,
			0,
			#endif
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};
#endif //(defined _SENSOR_GC1014_CDSP_DVP) && (_SENSOR_GC1014_CDSP_DVP == 1)
