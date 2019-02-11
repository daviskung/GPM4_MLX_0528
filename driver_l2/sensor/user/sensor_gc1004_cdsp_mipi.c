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


#if (defined _SENSOR_GC1004_CDSP_MIPI) && (_SENSOR_GC1004_CDSP_MIPI == 1)

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

#define	GC1004_ID							0x78
#define GC1004_WIDTH						1280
#define GC1004_HEIGHT						720
#define GC1004_OUT_WIDTH					1280
#define GC1004_OUT_HEIGHT					720


#define GC1004_30FPS_50HZ_DAY_EV_IDX 			100
#define GC1004_30FPS_50HZ_NIGHT_EV_IDX			155
#define GC1004_30FPS_50HZ_EXP_TIME_TOTAL		204
#define GC1004_30FPS_50HZ_INIT_EV_IDX 			((GC1004_30FPS_50HZ_DAY_EV_IDX + GC1004_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define GC1004_30FPS_50HZ_MAX_EV_IDX			    (GC1004_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define GC1004_30FPS_60HZ_DAY_EV_IDX 			100
#define GC1004_30FPS_60HZ_NIGHT_EV_IDX			170
#define GC1004_30FPS_60HZ_EXP_TIME_TOTAL		198
#define GC1004_30FPS_60HZ_INIT_EV_IDX 			((GC1004_30FPS_60HZ_DAY_EV_IDX + GC1004_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define GC1004_30FPS_60HZ_MAX_EV_IDX			    (GC1004_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define GC1004_24FPS_50HZ_DAY_EV_IDX 			138
#define GC1004_24FPS_50HZ_NIGHT_EV_IDX			194
#define GC1004_24FPS_50HZ_EXP_TIME_TOTAL		    254
#define GC1004_24FPS_50HZ_INIT_EV_IDX 			((GC1004_24FPS_50HZ_DAY_EV_IDX + GC1004_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define GC1004_24FPS_50HZ_MAX_EV_IDX			    (GC1004_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define GC1004_24FPS_60HZ_DAY_EV_IDX 			135
#define GC1004_24FPS_60HZ_NIGHT_EV_IDX			195
#define GC1004_24FPS_60HZ_EXP_TIME_TOTAL		    255
#define GC1004_24FPS_60HZ_INIT_EV_IDX 			((GC1004_24FPS_60HZ_DAY_EV_IDX + GC1004_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define GC1004_24FPS_60HZ_MAX_EV_IDX			    (GC1004_24FPS_60HZ_EXP_TIME_TOTAL - 10)


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
static INT32U gc1004_analog_gain = 0x100;

static INT8U ae_flag = 0;

#if SCCB_MODE == SCCB_GPIO
	static void *gc1004_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t gc1004_handle;
#endif

static mipi_config_t gc1004_mipi_cfg =
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

	GC1004_WIDTH,		/* width, 0~0xFFFF */
	GC1004_HEIGHT,		/* height, 0~0xFFFF */
	#if SENSOR_FLIP
	4,
	4,
	#else
	4,//2, 					/* back porch, 0~0xF */ //If Set 4 have show paper color at left side.u2.20160119
	6,
	#endif
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,//NULL,			/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	190,//250,				/* data mask, unit is ns */
	MIPI_CHECK_LP_00//MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};

static const regval8_t gc1004_reset_table[] =
	{// SYS
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0xFE, 0x80},
	{0xFF, 0xFF},
};

static const regval8_t gc1004_init_table[] =
{
	{0xF2, 0x00}, // sync_pad_io_ebi
	{0xF6, 0x00}, // up down
	{0xFC, 0xC6},
	{0xF7, 0x19}, // pll enable
	{0xF8, 0x03}, // 0x03,Pll mode 2
	{0xF9, 0x2E}, // [3]isp all clock enable [0] pll enable
	{0xFA, 0x00}, // div
	{0xFE, 0x00},

	// ANALOG & CISCTL
	{0x03, 0x02}, //ExposureTime[15:08]
	{0x04, 0xfa}, //ExposureTime[07:00]

	// H-blank
	{0x05 ,0x01},
	//{0x06 ,0x8f}, //0x8f
	{0x06 ,0x70},

	// VB
	{0x07 ,0x00},
	//{0x08 ,0x5a}, //0x30:27fps,0x5a:23fps,0x08
	{0x08 ,0x08},

	// Win height //for expsure time
	{0x0D ,0x02}, //height
	{0x0E ,0xf0}, //0xf0
	{0x0C ,0x00},
	// Win width
	{0x0F ,0x05}, //width
	{0x10 ,0x20}, //0x18

	// Sh-delay
	{0x11 ,0x00},
	{0x12 ,0x18}, //0x18 //0x1a

	{0x13, 0x10}, //0x10 St
	{0x14, 0x04}, //0x04 Et

	#if SENSOR_FLIP
	{0x17, 0x17}, // [0]mirror [1]flip
	#else
	{0x17, 0x14}, // [0]mirror [1]flip
	#endif

	{0x18, 0x0A}, // sdark off
	{0x19, 0x06},
	{0x1A, 0x09},
	{0x1B, 0x4F},
	{0x1C, 0x41}, //0x21
	{0x1D, 0xE0},

	{0x1E, 0x7C}, // The dark stripes
	//{0x1E, 0xfC},

	{0x1F, 0x08}, // comv_r
	{0x20, 0xA5}, // A vertical bar on the right
	{0x21, 0x2F}, // rsg
	{0x22, 0xB0},
	//{0x22, 0xc0},
	{0x23, 0x32},
	{0x24, 0x2F}, // PAD drive
	{0x2A, 0x00},
	{0x2C, 0xC0},
	{0x2D, 0x0F},
	{0x2E, 0xf0},
	{0x2F, 0x1F}, // exp not work
	{0x25, 0xC0},
	//{0x1F, 0x08}, //0x08->0x0F:¨¾¤î¹q¤l·¸¥X, by gc @0820
	{0x36, 0x0b}, // exp not work
	{0x37, 0x13},
	{0x38, 0x1b},

	// ISP
	{0xFE, 0x00}, //Page select:0
	{0x8A, 0x00},
	{0x8C, 0x07},//2}, //Hsync Polarity falling
	{0x8E, 0x02}, // luma value not normal

	// Imagae output
	{0x90, 0x01}, //crop en
	#if SENSOR_FLIP
	{0x92, 0x0b}, //0x10},crop y offset
	{0x94, 0x0f}, //crop x offset
	#else
	{0x92, 0x02}, //crop y offset
	{0x94, 0x04}, //crop x offset
	#endif

	{0x95, 0x02},   //out height
	{0x96, 0xD8},//0xD0, by gc @0820
	{0x97, 0x05},   //out width
	{0x98, 0x10},//0x10

	// Dark SUN
	{0xfe, 0x02},
	{0x40, 0x37},
	{0x49, 0x23},
	{0xa4, 0x00},
	{0xfe, 0x00},

	//	 Gain
	{0xb0, 0x50},
	{0xb3, 0x40},
	{0xb4, 0x40},
	{0xb5, 0x40},
	{0xfe, 0x00},

	// MIPI
	{0xfe, 0x03}, //Page select:3
	{0x01, 0x03},
	{0x02, 0x44}, //0x17;//0x77
	{0x03, 0x30},//{0x03, 0x30},
	{0x04, 0x01},
	{0x05, 0x00},
	{0x06, 0xa4},

	{0x20, 0x80}, //T_init_set
	{0x21, 0x10}, // T_LPX
	{0x22, 0x05}, // T clk HS prepare
	{0x23, 0x30}, // T clk zero
	{0x24, 0x02}, // T clk hs pre of data
	{0x25, 0x10}, // T clk hs post of data
	{0x26, 0x08}, // T clk trail

	{0x27, 0x10}, // T hs exit
	{0x28, 0xa0}, // T wakeup
	{0x29, 0x00}, // T d hs prepare
	{0x2a, 0x30}, // T d hs zero
	{0x2b, 0x08}, // T d hs trail
	#if (SENSOR_INTERFACE == MIPI_RAW8)	//RAW8
	{0x10, 0x84}, //0x90

	//need delay 1ms
	{0x10, 0x94},

	{0x11, 0x2a},
	{0x12, 0x10}, //RAW8 buffer[7:0] 1296*1=1296
	{0x13, 0x05}, //RAW8 buffer[15:8]
	#else	//RAW10
	{0x10, 0x80}, //0x90

	//need delay 1ms
	{0x10, 0x90},

	{0x11, 0x2b},
	{0x12, 0x54}, //RAW10 buffer[7:0] 1288=0x64a
	{0x13, 0x06}, //RAW10 buffer[15:8]
	#endif
	{0x15, 0x72},

	{0x21, 0x08},
	{0x22, 0x08},
	//{0x29, 0x07},
	{0x35, 0x0c},
	{0xfe, 0x00},

    //write Done
	{0xff,0xff}
};


static const regval8_t gc1004_720p_table[] =
{

};

#if 0
static const regval8_t gc1004_suspend_table[] =
{

};

static const regval8_t gc1004_resume_table[] =
{

};
#endif

static const int gc1004__30fps_exp_time_gain_50Hz[GC1004_30FPS_50HZ_EXP_TIME_TOTAL][3] =
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

static const  int gc1004_30fps_exp_time_gain_60Hz[GC1004_30FPS_60HZ_EXP_TIME_TOTAL][3] =
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

static INT32S gc1004_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	gc1004_handle = drv_l2_sccb_open(GC1004_ID, 8, 8);
	if(gc1004_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	gc1004_handle.devNumber = I2C_0;
	gc1004_handle.slaveAddr = GC1004_ID;
	gc1004_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void gc1004_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(gc1004_handle) {
		drv_l2_sccb_close(gc1004_handle);
		gc1004_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	gc1004_handle.slaveAddr = 0;
	gc1004_handle.clkRate = 0;
#endif
}

static INT32S gc1004_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(gc1004_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&gc1004_handle, data, 3);
#endif
}


static INT32S gc1004_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(gc1004_handle, reg, &data) >= 0) {
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
	if(drv_l1_i2c_bus_write(&gc1004_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&gc1004_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}

static INT32S gc1004_sccb_write_table(regval8_t *pTable)
{
	while(1) {
		if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
			break;
		}

		DBG_PRINT("0x%02x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(gc1004_sccb_write(pTable->reg_num, pTable->value) < 0) {
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

static INT32S gc1004_sccb_read_table(regval8_t *pTable)
{
    INT8U *Rdata;
	while(1) {
		if(pTable->reg_num == 0x00 && pTable->value == 0x00) {
			break;
		}

		if(gc1004_sccb_read(pTable->reg_num, Rdata) < 0) {
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
static int gc1004_cvt_agc_gain(int agc_gain)
{

	INT32U Analog_Multiple[9]={1000, 1400, 1800, 2560, 3400,4800,6840,9400,13200};

	INT32U Analog_Index;
	INT32U Digital_Gain;
	INT32U Decimal;
	INT32U ret;


	gc1004_analog_gain = agc_gain*10;

    if(gc1004_analog_gain>7200)   gc1004_analog_gain=7200; // max_gain=7.2
    //if(gc1004_analog_gain>5200)   gc1004_analog_gain=5200; // max_gain=5.2

	Analog_Index=0;

	while(Analog_Index<9)
	{
	  if(gc1004_analog_gain<Analog_Multiple[Analog_Index])
	  {
		break;
	  }
	  else
	  {
		Analog_Index++;
	  }
	}

	Digital_Gain = gc1004_analog_gain*1000/Analog_Multiple[Analog_Index-1];
	Decimal=(Digital_Gain*64)/1000;

	if(gc1004_analog_gain>4000)
	{
      ret = gc1004_sccb_write(0x66,  0x1c);
      if(ret < 0) return ret;
	}
	if(gc1004_analog_gain<3250)
    {
      ret = gc1004_sccb_write(0x66,  0x20);
      if(ret < 0) return ret;
	}


#if 1
	ret = gc1004_sccb_write(0xb1,  Decimal>>6);
	if(ret < 0) return ret;
	ret = gc1004_sccb_write(0xb2,  (Decimal<<2)&0xfc);
	if(ret < 0) return ret;
	ret = gc1004_sccb_write(0xb6,   Analog_Index-1);
	if(ret < 0) return ret;
#else
	ret = gc1004_sccb_write(0xb1, 0x01);
	if(ret < 0) return ret;
	ret = gc1004_sccb_write(0xb2, 0x00);
	if(ret < 0) return ret;
	ret = gc1004_sccb_write(0xb6, 0x06);
	if(ret < 0) return ret;
#endif

}


static int gc1004_set_xfps_exposure_time(sensor_exposure_t *si)
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

		t1 = (temp & 0xff);
		t2 = (temp >> 8) & 0x00ff;

        ret = gc1004_sccb_write(0x04, t1);
        if(ret < 0) return ret;
        ret = gc1004_sccb_write(0x03, t2);
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
		ret = gc1004_cvt_agc_gain(temp);
		if(ret < 0) {
		    DBG_PRINT("ERR: write sensor analog agin = 0x%x!!!\r\n", temp);
            return ret;
		}
	}

    return 0;
}


static void gc1004_set_ae(int ev_step)
{
    seInfo.ae_ev_step = ev_step;
    gc1004_set_xfps_exposure_time(&seInfo);
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)gc1004_set_ae;
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
static void mipi_gc1004_handle(INT32U event)
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
void gc1004_seinfo_init(void)
{
    seInfo.sensor_ev_idx = GC1004_30FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= GC1004_30FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= GC1004_30FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = GC1004_30FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = GC1004_30FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)gc1004_30fps_exp_time_gain_60Hz;

	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x00;

}

static void gc1004_mipi_sensor_init(void)
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

    //GC1004_set_favorite();
	// mclk output
	//drv_l2_sensor_set_mclkout(gc1004_cdsp_mipi_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);

	// reguest sccb
	gc1004_sccb_open();

	// reset sensor
	gc1004_sccb_write_table((regval8_t *)gc1004_reset_table);
    //osDelay(200);

	// init sensor
	gc1004_sccb_write_table((regval8_t *)gc1004_init_table);

	//gc1004_sccb_read_table((regval8_t *)gc1004_init_table);

    DBG_PRINT("Sensor gc1004 mipi init completed\r\n");
}


/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
//static void gc1004_cdsp_mipi_init(void)
void gc1004_cdsp_mipi_init(void)
{
	//ae init
	gc1004_seinfo_init();
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
    osDelay(20);
    gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);


	// mipi enable
	#if (MIPI_DEV_NO == 1)
		drv_l1_mipi_init(MIPI_1,ENABLE);
	#else
	    drv_l1_mipi_init(MIPI_0,ENABLE);
	#endif

    DBG_PRINT("Sensor GC1004 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void gc1004_cdsp_mipi_uninit(void)
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
	gc1004_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void gc1004_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
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
        gc1004_sccb_write_table((regval16_t *)gc1004_scale_vga_table);
		break;

	case 1:
		gc1004_sccb_write_table((regval16_t *)gc1004_720p_table);
		break;

	default:
		while(1);
	}
*/
    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 4);    //u2.added.20151207
    //osDelay(200);

#if 1   // change mclk
	drv_l2_sensor_set_mclkout(gc1004_cdsp_mipi_ops.info[index].mclk);
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
	format.input_format =  gc1004_cdsp_mipi_ops.info[index].input_format;
	format.output_format = gc1004_cdsp_mipi_ops.info[index].output_format;
	target_w = gc1004_cdsp_mipi_ops.info[index].target_w;
	target_h = gc1004_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = gc1004_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = gc1004_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = gc1004_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = gc1004_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = gc1004_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = gc1004_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = gc1004_cdsp_mipi_ops.info[index].vsync_active;


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
		gc1004_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		gc1004_mipi_cfg.data_type = MIPI_RAW8;
		//gc1004_mipi_cfg.h_back_porch = 4;
		break;

	case 2:
	case 3:
		gc1004_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		gc1004_mipi_cfg.data_type = MIPI_RAW10;
		//gc1004_mipi_cfg.h_back_porch = 2;//4;
        gc1004_mipi_cfg.pixel_clk_sel = 0;
		break;

	default:
    	/* #if (RD_SIM == 1)
        gc1004_mipi_cfg.blanking_line_en = 1;
        gc1004_mipi_cfg.byte_clk_edge = 0;
        gc1004_mipi_cfg.check_hs_seq = 0;
        gc1004_mipi_cfg.data_from_mmr = 0;
        gc1004_mipi_cfg.data_mask_time = 250;
        gc1004_mipi_cfg.data_type = 3;
        gc1004_mipi_cfg.data_type_to_cdsp = 1;
        gc1004_mipi_cfg.ecc_check_en = 1;
        gc1004_mipi_cfg.ecc_order = 3;
        gc1004_mipi_cfg.h_front_porch = 4;
        gc1004_mipi_cfg.low_power_en = 0;
        gc1004_mipi_cfg.mipi_lane_no = 1;
        gc1004_mipi_cfg.pixel_clk_sel = 1;
        #endif
        */
        gc1004_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		gc1004_mipi_cfg.data_type = MIPI_RAW8;
		//gc1004_mipi_cfg.h_back_porch = 4;
	}
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      if (drv_l1_mipi_set_parameter(MIPI_1, &gc1004_mipi_cfg) < 0)
      {
        DBG_PRINT("MIPI1 init fail !!!\r\n");
      }
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_gc1004_handle);
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      if (drv_l1_mipi_set_parameter(MIPI_0, &gc1004_mipi_cfg) < 0)
      {
        DBG_PRINT("MIPI0 init fail !!!\r\n");
      }
      else
      {
        DBG_PRINT("MIPI0 init completed.\r\n");
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_gc1004_handle);
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
      }
    #endif

	//Enable MCLK & Init Sensor
    gc1004_mipi_sensor_init();


    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    gc1004_set_xfps_exposure_time(&seInfo);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void gc1004_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* gc1004_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&gc1004_cdsp_mipi_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t gc1004_cdsp_mipi_ops =
{
	SENSOR_GC1004_CDSP_MIPI_NAME,		/* sensor name */
	gc1004_cdsp_mipi_init,
	gc1004_cdsp_mipi_uninit,
	gc1004_cdsp_mipi_stream_on,
	gc1004_cdsp_mipi_stream_off,
	gc1004_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SBGGR8,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			320,						/* target width */
			240,  						/* target height */
			GC1004_WIDTH,				/* sensor width */
			GC1004_HEIGHT,  				/* sensor height */
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
			GC1004_WIDTH,				/* sensor width */
			GC1004_HEIGHT, 				/* sensor height */
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
			GC1004_OUT_WIDTH,			/* target width */
			GC1004_OUT_HEIGHT,  		/* target height */
			GC1004_WIDTH,				/* sensor width */
			GC1004_HEIGHT,  			/* sensor height */
			#if SENSOR_FLIP
			11,							/* sensor h offset */
			1,							/* sensor v offset */
			#else
			8,
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
			GC1004_OUT_WIDTH,			/* target width */
			GC1004_OUT_HEIGHT,  		/* target height */
			GC1004_WIDTH,				/* sensor width */
			GC1004_HEIGHT, 				/* sensor height */
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
#endif //(defined _SENSOR_GC1004_CDSP_MIPI) && (_SENSOR_GC1004_CDSP_MIPI == 1)
