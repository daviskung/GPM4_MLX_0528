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


#if (defined _SENSOR_JXF02_CDSP_MIPI) && (_SENSOR_JXF02_CDSP_MIPI == 1)

#include "drv_l2_user_calibration.h"
#include "drv_l2_user_preference.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_TEST        0
#define COLOR_BAR_EN            0
#define MIPI_ISR_TEST           1
#define MIPI_LANE_NO			      2	        // 1 or 2 lane
#define MIPI_DEV_NO             0           //0:MIPI_0 or 1:MIPI_1

#define	JXF02_ID						0x80
#define JXF02_WIDTH				    	1932
#define JXF02_HEIGHT				    1092

#define JXF02_OUT_WIDTH					1920
#define JXF02_OUT_HEIGHT				1088




#define JXF02_30FPS_50HZ_DAY_EV_IDX 			140
#define JXF02_30FPS_50HZ_NIGHT_EV_IDX			190
#define JXF02_30FPS_50HZ_EXP_TIME_TOTAL		    246
#define JXF02_30FPS_50HZ_INIT_EV_IDX 			((JXF02_30FPS_50HZ_DAY_EV_IDX + JXF02_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define JXF02_30FPS_50HZ_MAX_EV_IDX			    (JXF02_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define JXF02_30FPS_60HZ_DAY_EV_IDX 			137
#define JXF02_30FPS_60HZ_NIGHT_EV_IDX			190
#define JXF02_30FPS_60HZ_EXP_TIME_TOTAL		    249
#define JXF02_30FPS_60HZ_INIT_EV_IDX 			((JXF02_30FPS_60HZ_DAY_EV_IDX + JXF02_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define JXF02_30FPS_60HZ_MAX_EV_IDX			    (JXF02_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define JXF02_24FPS_50HZ_DAY_EV_IDX 			138
#define JXF02_24FPS_50HZ_NIGHT_EV_IDX			194
#define JXF02_24FPS_50HZ_EXP_TIME_TOTAL		    254
#define JXF02_24FPS_50HZ_INIT_EV_IDX 			((JXF02_24FPS_50HZ_DAY_EV_IDX + JXF02_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define JXF02_24FPS_50HZ_MAX_EV_IDX			    (JXF02_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define JXF02_24FPS_60HZ_DAY_EV_IDX 			135
#define JXF02_24FPS_60HZ_NIGHT_EV_IDX			195
#define JXF02_24FPS_60HZ_EXP_TIME_TOTAL		    255
#define JXF02_24FPS_60HZ_INIT_EV_IDX 			((JXF02_24FPS_60HZ_DAY_EV_IDX + JXF02_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define JXF02_24FPS_60HZ_MAX_EV_IDX			    (JXF02_24FPS_60HZ_EXP_TIME_TOTAL - 10)




/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct regval16_s
{
	INT8U reg_num;
	INT8U value;
} regval16_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static int ob_cnt = 0;
static int *p_expTime_table;
static sensor_exposure_t 	seInfo;
static int pre_sensor_a_gain, pre_sensor_time;


#if SCCB_MODE == SCCB_GPIO
	static void *jxf02_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t jxf02_handle;
#endif

static mipi_config_t jxf02_mipi_cfg =
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

	JXF02_WIDTH,		/* width, 0~0xFFFF */
	JXF02_HEIGHT,		/* height, 0~0xFFFF */
	4, 					/* back porch, 0~0xF */
	4,					/* front porch, 0~0xF */
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,				/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	190,//250,				/* data mask, unit is ns */
	MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};




const regval16_t jxf02_Mipi_Raw10_1080P_30fps[] =
{// [JXF02_1932x1092x30_Mipi_2L_10b.reg]
{0x12,0x40},
{0x0D,0xA4},
{0x0E,0x10},
{0x0F,0x19},
{0x10,0x22},
{0x11,0x80},
{0x20,0x5B},
{0x21,0x02},
{0x22,0x68},
{0x23,0x04},
{0x24,0xE3},
{0x25,0x44},
{0x26,0x41},
{0x27,0xC7},
{0x28,0x12},
{0x29,0x00},
{0x2A,0xC1},
{0x2B,0x18},
{0x2C,0x01},
{0x2D,0x00},
{0x2E,0x14},
{0x2F,0x44},
{0x30,0x8C},
{0x31,0x06},
{0x32,0x10},
{0x33,0x0C},
{0x34,0x66},
{0x35,0xD1},
{0x36,0x0E},
{0x37,0x34},
{0x38,0x14},
{0x39,0x82},
{0x3A,0x08},
{0x1D,0x00},
{0x1E,0x00},
{0x6C,0x40},
{0x70,0x61},
{0x71,0x4A},
{0x72,0x4A},
{0x73,0x33},
{0x74,0x52},
{0x75,0x2B},
{0x76,0x6F},
{0x77,0x09},
{0x78,0x0B},
{0x41,0xD2},
{0x42,0x23},
{0x67,0x70},
{0x68,0x02},
{0x69,0x74},
{0x6A,0x48},
{0x5A,0x04},
{0x5C,0x8f},
{0x5D,0xB0},
{0x5E,0xe6},
{0x60,0x16},
{0x62,0x30},
{0x64,0x58},
{0x65,0x00},
{0x12,0x00},
{0x47,0x80},
{0x50,0x02},
{0x13,0x85},
{0x14,0x80},
{0x16,0xC0},
{0x17,0x40},
{0x18,0x19},
{0x19,0xa1},
{0x4a,0x03},
{0x49,0x10},
{0x45,0x19},
{0x1F,0x01},
//PWDN Setting


// time
{0x01, 0x80},
{0x02, 0x03},



//None
{ 0x00, 0x00 }, //end
};



const regval16_t jxf02_Mipi_Raw10_1080P_24fps[] =
{
#if 1
{0x12,0x40},
{0x0D,0xA4},
{0x0E,0x10},
{0x0F,0x19},
{0x10,0x22},
{0x11,0x80},
{0x20,0x5B},
{0x21,0x02},
{0x22,0x82},
{0x23,0x05},
{0x24,0xE3},
{0x25,0x44},
{0x26,0x41},
{0x27,0xC7},
{0x28,0x12},
{0x29,0x00},
{0x2A,0xC1},
{0x2B,0x18},
{0x2C,0x01},
{0x2D,0x00},
{0x2E,0x14},
{0x2F,0x44},
{0x30,0x8C},
{0x31,0x06},
{0x32,0x10},
{0x33,0x0C},
{0x34,0x66},
{0x35,0xD1},
{0x36,0x0E},
{0x37,0x34},
{0x38,0x14},
{0x39,0x82},
{0x3A,0x08},
{0x1D,0x00},
{0x1E,0x00},
{0x6C,0x40},
{0x70,0x61},
{0x71,0x4A},
{0x72,0x4A},
{0x73,0x33},
{0x74,0x52},
{0x75,0x2B},
{0x76,0x6F},
{0x77,0x09},
{0x78,0x0B},
{0x41,0xD2},
{0x42,0x23},
{0x67,0x70},
{0x68,0x02},
{0x69,0x74},
{0x6A,0x48},
{0x5A,0x04},
{0x5C,0x8f},
{0x5D,0xB0},
{0x5E,0xe6},
{0x60,0x16},
{0x62,0x30},
{0x64,0x58},
{0x65,0x00},
{0x12,0x00},
{0x47,0x80},
{0x50,0x02},
{0x13,0x85},
{0x14,0x80},
{0x16,0xC0},
{0x17,0x40},
{0x18,0x19},
{0x19,0xa1},
{0x4a,0x03},
{0x49,0x10},
{0x45,0x19},
{0x1F,0x01},
#else
//[JXF02_1932x1092x24_Mipi_1L_10b.reg]
//INI Start
{0x12,0x40},
//DVP Setting
{0x0D,0xA4},
//PLL Setting
{0x0E,0x10},
{0x0F,0x19},
{0x10,0x1b},
{0x11,0x80},
//Frame/Window
{0x20,0x5B},
{0x21,0x02},
{0x22,0x60},
{0x23,0x04},
{0x24,0xE3},
{0x25,0x44},
{0x26,0x41},
{0x27,0xC7},
{0x28,0x0E},
{0x29,0x00},
{0x2A,0xC1},
{0x2B,0x38},
{0x2C,0x01},
{0x2D,0x00},
{0x2E,0x14},
{0x2F,0x44},
//Sensor Timing
{0x30,0x8C},
{0x31,0x06},
{0x32,0x10},
{0x33,0x0C},
{0x34,0x66},
{0x35,0xD1},
{0x36,0x0E},
{0x37,0x34},
{0x38,0x14},
{0x39,0x82},
{0x3A,0x08},
{0x3B,0x00},
{0x3C,0xFF},
{0x3D,0x90},
{0x3E,0xA0},

//Interface
{0x1D,0x00},
{0x1E,0x00},
{0x6C,0x40},
{0x70,0x61},
{0x71,0x4A},
{0x72,0x4A},
{0x73,0x33},
{0x74,0x52},
{0x75,0x2B},
{0x76,0x6F},
{0x77,0x09},
{0x78,0x0B},
//Array/AnADC/PWC
{0x41,0xD3},
{0x42,0x23},
{0x67,0x70},
{0x68,0x02},
{0x69,0x74},
{0x6A,0x48},
//SRAM/RAMP
{0x5A,0x04},
{0x5C,0x8f},
{0x5D,0xB0},
{0x5E,0xe6},
{0x60,0x16},
{0x62,0x30},
{0x63,0x80},
{0x64,0x58},
{0x65,0x00},
//INI End},
{0x12,0x00},
//AE/AG/ABLC
{0x47,0x80},
{0x50,0x02},
{0x13,0x85},
{0x14,0x80},
{0x16,0xC0},
{0x17,0x40},
{0x18,0xDF},
{0x19,0xa0},
{0x4a,0x03},
{0x49,0x10},
//Auto Vramp
{0x45,0x19},
{0x1F,0x01},
//PWDN Setting
#endif

// time
{0x01, 0x80},
{0x02, 0x03},

//None
{ 0x00, 0x00 }, //end
};


static const  int jxf02_30fps_exp_time_gain_60Hz[JXF02_30FPS_60HZ_EXP_TIME_TOTAL][3] =
{
{3, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{8, (int)(1.00*16), (int)(1.00*16)},
{8, (int)(1.00*16), (int)(1.00*16)},
{8, (int)(1.00*16), (int)(1.00*16)},
{8, (int)(1.00*16), (int)(1.00*16)},
{9, (int)(1.00*16), (int)(1.00*16)},
{9, (int)(1.00*16), (int)(1.00*16)},
{9, (int)(1.00*16), (int)(1.00*16)},
{10, (int)(1.00*16), (int)(1.00*16)},
{10, (int)(1.00*16), (int)(1.00*16)},
{10, (int)(1.00*16), (int)(1.00*16)},
{11, (int)(1.00*16), (int)(1.00*16)},
{11, (int)(1.00*16), (int)(1.00*16)},
{12, (int)(1.00*16), (int)(1.00*16)},
{12, (int)(1.00*16), (int)(1.00*16)},
{12, (int)(1.00*16), (int)(1.00*16)},
{13, (int)(1.00*16), (int)(1.00*16)},
{13, (int)(1.00*16), (int)(1.00*16)},
{14, (int)(1.00*16), (int)(1.00*16)},
{14, (int)(1.00*16), (int)(1.00*16)},
{15, (int)(1.00*16), (int)(1.00*16)},
{15, (int)(1.00*16), (int)(1.00*16)},
{16, (int)(1.00*16), (int)(1.00*16)},
{16, (int)(1.00*16), (int)(1.00*16)},
{17, (int)(1.00*16), (int)(1.00*16)},
{17, (int)(1.00*16), (int)(1.00*16)},
{18, (int)(1.00*16), (int)(1.00*16)},
{19, (int)(1.00*16), (int)(1.00*16)},
{19, (int)(1.00*16), (int)(1.00*16)},
{20, (int)(1.00*16), (int)(1.00*16)},
{21, (int)(1.00*16), (int)(1.00*16)},
{21, (int)(1.00*16), (int)(1.00*16)},
{22, (int)(1.00*16), (int)(1.00*16)},
{23, (int)(1.00*16), (int)(1.00*16)},
{24, (int)(1.00*16), (int)(1.00*16)},
{25, (int)(1.00*16), (int)(1.00*16)},
{26, (int)(1.00*16), (int)(1.00*16)},
{26, (int)(1.00*16), (int)(1.00*16)},
{27, (int)(1.00*16), (int)(1.00*16)},
{28, (int)(1.00*16), (int)(1.00*16)},
{29, (int)(1.00*16), (int)(1.00*16)},
{30, (int)(1.00*16), (int)(1.00*16)},
{31, (int)(1.00*16), (int)(1.00*16)},
{33, (int)(1.00*16), (int)(1.00*16)},
{34, (int)(1.00*16), (int)(1.00*16)},
{35, (int)(1.00*16), (int)(1.00*16)},
{36, (int)(1.00*16), (int)(1.00*16)},
{37, (int)(1.00*16), (int)(1.00*16)},
{39, (int)(1.00*16), (int)(1.00*16)},
{40, (int)(1.00*16), (int)(1.00*16)},
{41, (int)(1.00*16), (int)(1.00*16)},
{43, (int)(1.00*16), (int)(1.00*16)},
{44, (int)(1.00*16), (int)(1.00*16)},
{46, (int)(1.00*16), (int)(1.00*16)},
{48, (int)(1.00*16), (int)(1.00*16)},
{49, (int)(1.00*16), (int)(1.00*16)},
{51, (int)(1.00*16), (int)(1.00*16)},
{53, (int)(1.00*16), (int)(1.00*16)},
{55, (int)(1.00*16), (int)(1.00*16)},
{57, (int)(1.00*16), (int)(1.00*16)},
{59, (int)(1.00*16), (int)(1.00*16)},
{61, (int)(1.00*16), (int)(1.00*16)},
{63, (int)(1.00*16), (int)(1.00*16)},
{65, (int)(1.00*16), (int)(1.00*16)},
{67, (int)(1.00*16), (int)(1.00*16)},
{70, (int)(1.00*16), (int)(1.00*16)},
{72, (int)(1.00*16), (int)(1.00*16)},
{75, (int)(1.00*16), (int)(1.00*16)},
{77, (int)(1.00*16), (int)(1.00*16)},
{80, (int)(1.00*16), (int)(1.00*16)},
{83, (int)(1.00*16), (int)(1.00*16)},
{86, (int)(1.00*16), (int)(1.00*16)},
{89, (int)(1.00*16), (int)(1.00*16)},
{92, (int)(1.00*16), (int)(1.00*16)},
{95, (int)(1.00*16), (int)(1.00*16)},
{99, (int)(1.00*16), (int)(1.00*16)},
{102, (int)(1.00*16), (int)(1.00*16)},
{106, (int)(1.00*16), (int)(1.00*16)},
{110, (int)(1.00*16), (int)(1.00*16)},
{113, (int)(1.00*16), (int)(1.00*16)},
{117, (int)(1.00*16), (int)(1.00*16)},
{122, (int)(1.00*16), (int)(1.00*16)},
{126, (int)(1.00*16), (int)(1.00*16)},
{130, (int)(1.00*16), (int)(1.00*16)},
{135, (int)(1.00*16), (int)(1.00*16)},
{140, (int)(1.00*16), (int)(1.00*16)},
{144, (int)(1.00*16), (int)(1.00*16)},
{150, (int)(1.00*16), (int)(1.00*16)},
{155, (int)(1.00*16), (int)(1.00*16)},
{160, (int)(1.00*16), (int)(1.00*16)},
{166, (int)(1.00*16), (int)(1.00*16)},
{172, (int)(1.00*16), (int)(1.00*16)},
{178, (int)(1.00*16), (int)(1.00*16)},
{184, (int)(1.00*16), (int)(1.00*16)},
{191, (int)(1.00*16), (int)(1.00*16)},
{197, (int)(1.00*16), (int)(1.00*16)},
{204, (int)(1.00*16), (int)(1.00*16)},
{212, (int)(1.00*16), (int)(1.00*16)},
{219, (int)(1.00*16), (int)(1.00*16)},
{227, (int)(1.00*16), (int)(1.00*16)},
{235, (int)(1.00*16), (int)(1.00*16)},
{243, (int)(1.00*16), (int)(1.00*16)},
{252, (int)(1.00*16), (int)(1.00*16)},
{260, (int)(1.00*16), (int)(1.00*16)},
{270, (int)(1.00*16), (int)(1.00*16)},
{279, (int)(1.00*16), (int)(1.00*16)},
{282, (int)(1.00*16), (int)(1.00*16)},
{282, (int)(1.0625*16), (int)(1.00*16)},
{282, (int)(1.0625*16), (int)(1.00*16)},
{282, (int)(1.125*16), (int)(1.00*16)},
{282, (int)(1.125*16), (int)(1.00*16)},
{282, (int)(1.1875*16), (int)(1.00*16)},
{282, (int)(1.25*16), (int)(1.00*16)},
{282, (int)(1.25*16), (int)(1.00*16)},
{282, (int)(1.3125*16), (int)(1.00*16)},
{282, (int)(1.375*16), (int)(1.00*16)},
{282, (int)(1.4375*16), (int)(1.00*16)},
{282, (int)(1.4375*16), (int)(1.00*16)},
{282, (int)(1.5*16), (int)(1.00*16)},
{282, (int)(1.5625*16), (int)(1.00*16)},
{282, (int)(1.625*16), (int)(1.00*16)},
{282, (int)(1.6875*16), (int)(1.00*16)},
{282, (int)(1.75*16), (int)(1.00*16)},
{282, (int)(1.8125*16), (int)(1.00*16)},
{282, (int)(1.875*16), (int)(1.00*16)},
{282, (int)(1.9375*16), (int)(1.00*16)},
{564, (int)(1.00*16), (int)(1.00*16)},
{564, (int)(1.0625*16), (int)(1.00*16)},
{564, (int)(1.0625*16), (int)(1.00*16)},
{564, (int)(1.125*16), (int)(1.00*16)},
{564, (int)(1.125*16), (int)(1.00*16)},
{564, (int)(1.1875*16), (int)(1.00*16)},
{564, (int)(1.25*16), (int)(1.00*16)},
{564, (int)(1.25*16), (int)(1.00*16)},
{564, (int)(1.3125*16), (int)(1.00*16)},
{564, (int)(1.375*16), (int)(1.00*16)},
{564, (int)(1.4375*16), (int)(1.00*16)},
{564, (int)(1.4375*16), (int)(1.00*16)},
{564, (int)(1.5*16), (int)(1.00*16)},
{564, (int)(1.5625*16), (int)(1.00*16)},
{564, (int)(1.625*16), (int)(1.00*16)},
{564, (int)(1.6875*16), (int)(1.00*16)},
{564, (int)(1.75*16), (int)(1.00*16)},
{564, (int)(1.8125*16), (int)(1.00*16)},
{564, (int)(1.875*16), (int)(1.00*16)},
{564, (int)(1.9375*16), (int)(1.00*16)},
{846, (int)(1.3125*16), (int)(1.00*16)},
{846, (int)(1.375*16), (int)(1.00*16)},
{846, (int)(1.4375*16), (int)(1.00*16)},
{846, (int)(1.4375*16), (int)(1.00*16)},
{846, (int)(1.5*16), (int)(1.00*16)},
{846, (int)(1.5625*16), (int)(1.00*16)},
{846, (int)(1.625*16), (int)(1.00*16)},
{846, (int)(1.6875*16), (int)(1.00*16)},
{846, (int)(1.75*16), (int)(1.00*16)},
{846, (int)(1.8125*16), (int)(1.00*16)},
{846, (int)(1.875*16), (int)(1.00*16)},
{846, (int)(1.9375*16), (int)(1.00*16)},
{1128, (int)(1.5*16), (int)(1.00*16)},
{1128, (int)(1.5625*16), (int)(1.00*16)},
{1128, (int)(1.625*16), (int)(1.00*16)},
{1128, (int)(1.6875*16), (int)(1.00*16)},
{1128, (int)(1.75*16), (int)(1.00*16)},
{1128, (int)(1.8125*16), (int)(1.00*16)},
{1128, (int)(1.875*16), (int)(1.00*16)},
{1128, (int)(1.9375*16), (int)(1.00*16)},
{1128, (int)(2.0*16), (int)(1.00*16)},
{1128, (int)(2.07*16), (int)(1.00*16)},
{1128, (int)(2.14*16), (int)(1.00*16)},
{1128, (int)(2.22*16), (int)(1.00*16)},
{1128, (int)(2.30*16), (int)(1.00*16)},
{1128, (int)(2.38*16), (int)(1.00*16)},
{1128, (int)(2.46*16), (int)(1.00*16)},
{1128, (int)(2.55*16), (int)(1.00*16)},
{1128, (int)(2.64*16), (int)(1.00*16)},
{1128, (int)(2.73*16), (int)(1.00*16)},
{1128, (int)(2.83*16), (int)(1.00*16)},
{1128, (int)(2.93*16), (int)(1.00*16)},
{1128, (int)(3.03*16), (int)(1.00*16)},
{1128, (int)(3.14*16), (int)(1.00*16)},
{1128, (int)(3.25*16), (int)(1.00*16)},
{1128, (int)(3.36*16), (int)(1.00*16)},
{1128, (int)(3.48*16), (int)(1.00*16)},
{1128, (int)(3.61*16), (int)(1.00*16)},
{1128, (int)(3.73*16), (int)(1.00*16)},
{1128, (int)(3.86*16), (int)(1.00*16)},
{1128, (int)(4.00*16), (int)(1.00*16)},
{1128, (int)(4.14*16), (int)(1.00*16)},
{1128, (int)(4.29*16), (int)(1.00*16)},
{1128, (int)(4.44*16), (int)(1.00*16)},
{1128, (int)(4.59*16), (int)(1.00*16)},
{1128, (int)(4.76*16), (int)(1.00*16)},
{1128, (int)(4.92*16), (int)(1.00*16)},
{1128, (int)(5.10*16), (int)(1.00*16)},
{1128, (int)(5.28*16), (int)(1.00*16)},
{1128, (int)(5.46*16), (int)(1.00*16)},
{1128, (int)(5.66*16), (int)(1.00*16)},
{1128, (int)(5.86*16), (int)(1.00*16)},
{1128, (int)(6.06*16), (int)(1.00*16)},
{1128, (int)(6.28*16), (int)(1.00*16)},
{1128, (int)(6.50*16), (int)(1.00*16)},
{1128, (int)(6.73*16), (int)(1.00*16)},
{1128, (int)(6.96*16), (int)(1.00*16)},
{1128, (int)(7.21*16), (int)(1.00*16)},
{1128, (int)(7.46*16), (int)(1.00*16)},
{1128, (int)(7.73*16), (int)(1.00*16)},
{1128, (int)(8.00*16), (int)(1.00*16)},
{1128, (int)(8.28*16), (int)(1.00*16)},
{1128, (int)(8.57*16), (int)(1.00*16)},
{1128, (int)(8.88*16), (int)(1.00*16)},
{1128, (int)(9.19*16), (int)(1.00*16)},
{1128, (int)(9.51*16), (int)(1.00*16)},
{1128, (int)(9.85*16), (int)(1.00*16)},
{1128, (int)(10.20*16), (int)(1.00*16)},
{1128, (int)(10.56*16), (int)(1.00*16)},
{1128, (int)(10.93*16), (int)(1.00*16)},
{1128, (int)(11.31*16), (int)(1.00*16)},
{1128, (int)(11.71*16), (int)(1.00*16)},
{1128, (int)(12.13*16), (int)(1.00*16)},
{1128, (int)(12.55*16), (int)(1.00*16)},
{1128, (int)(13.00*16), (int)(1.00*16)},
{1128, (int)(13.45*16), (int)(1.00*16)},
{1128, (int)(13.93*16), (int)(1.00*16)},
{1128, (int)(14.42*16), (int)(1.00*16)},
{1128, (int)(14.93*16), (int)(1.00*16)},
{1128, (int)(15.45*16), (int)(1.00*16)},
{1128, (int)(16.00*16), (int)(1.00*16)}
};

static const  int jxf02_30fps_exp_time_gain_50Hz[JXF02_30FPS_50HZ_EXP_TIME_TOTAL][3] =
{
{3, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{4, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{5, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{6, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{7, (int)(1.00*16), (int)(1.00*16)},
{8, (int)(1.00*16), (int)(1.00*16)},
{8, (int)(1.00*16), (int)(1.00*16)},
{8, (int)(1.00*16), (int)(1.00*16)},
{9, (int)(1.00*16), (int)(1.00*16)},
{9, (int)(1.00*16), (int)(1.00*16)},
{9, (int)(1.00*16), (int)(1.00*16)},
{9, (int)(1.00*16), (int)(1.00*16)},
{10, (int)(1.00*16), (int)(1.00*16)},
{10, (int)(1.00*16), (int)(1.00*16)},
{10, (int)(1.00*16), (int)(1.00*16)},
{11, (int)(1.00*16), (int)(1.00*16)},
{11, (int)(1.00*16), (int)(1.00*16)},
{12, (int)(1.00*16), (int)(1.00*16)},
{12, (int)(1.00*16), (int)(1.00*16)},
{12, (int)(1.00*16), (int)(1.00*16)},
{13, (int)(1.00*16), (int)(1.00*16)},
{13, (int)(1.00*16), (int)(1.00*16)},
{14, (int)(1.00*16), (int)(1.00*16)},
{14, (int)(1.00*16), (int)(1.00*16)},
{15, (int)(1.00*16), (int)(1.00*16)},
{15, (int)(1.00*16), (int)(1.00*16)},
{16, (int)(1.00*16), (int)(1.00*16)},
{16, (int)(1.00*16), (int)(1.00*16)},
{17, (int)(1.00*16), (int)(1.00*16)},
{18, (int)(1.00*16), (int)(1.00*16)},
{18, (int)(1.00*16), (int)(1.00*16)},
{19, (int)(1.00*16), (int)(1.00*16)},
{20, (int)(1.00*16), (int)(1.00*16)},
{20, (int)(1.00*16), (int)(1.00*16)},
{21, (int)(1.00*16), (int)(1.00*16)},
{22, (int)(1.00*16), (int)(1.00*16)},
{22, (int)(1.00*16), (int)(1.00*16)},
{23, (int)(1.00*16), (int)(1.00*16)},
{24, (int)(1.00*16), (int)(1.00*16)},
{25, (int)(1.00*16), (int)(1.00*16)},
{26, (int)(1.00*16), (int)(1.00*16)},
{27, (int)(1.00*16), (int)(1.00*16)},
{28, (int)(1.00*16), (int)(1.00*16)},
{29, (int)(1.00*16), (int)(1.00*16)},
{30, (int)(1.00*16), (int)(1.00*16)},
{31, (int)(1.00*16), (int)(1.00*16)},
{32, (int)(1.00*16), (int)(1.00*16)},
{33, (int)(1.00*16), (int)(1.00*16)},
{34, (int)(1.00*16), (int)(1.00*16)},
{35, (int)(1.00*16), (int)(1.00*16)},
{36, (int)(1.00*16), (int)(1.00*16)},
{38, (int)(1.00*16), (int)(1.00*16)},
{39, (int)(1.00*16), (int)(1.00*16)},
{40, (int)(1.00*16), (int)(1.00*16)},
{42, (int)(1.00*16), (int)(1.00*16)},
{43, (int)(1.00*16), (int)(1.00*16)},
{45, (int)(1.00*16), (int)(1.00*16)},
{46, (int)(1.00*16), (int)(1.00*16)},
{48, (int)(1.00*16), (int)(1.00*16)},
{50, (int)(1.00*16), (int)(1.00*16)},
{52, (int)(1.00*16), (int)(1.00*16)},
{53, (int)(1.00*16), (int)(1.00*16)},
{55, (int)(1.00*16), (int)(1.00*16)},
{57, (int)(1.00*16), (int)(1.00*16)},
{59, (int)(1.00*16), (int)(1.00*16)},
{61, (int)(1.00*16), (int)(1.00*16)},
{64, (int)(1.00*16), (int)(1.00*16)},
{66, (int)(1.00*16), (int)(1.00*16)},
{68, (int)(1.00*16), (int)(1.00*16)},
{70, (int)(1.00*16), (int)(1.00*16)},
{73, (int)(1.00*16), (int)(1.00*16)},
{76, (int)(1.00*16), (int)(1.00*16)},
{78, (int)(1.00*16), (int)(1.00*16)},
{81, (int)(1.00*16), (int)(1.00*16)},
{84, (int)(1.00*16), (int)(1.00*16)},
{87, (int)(1.00*16), (int)(1.00*16)},
{90, (int)(1.00*16), (int)(1.00*16)},
{93, (int)(1.00*16), (int)(1.00*16)},
{96, (int)(1.00*16), (int)(1.00*16)},
{100, (int)(1.00*16), (int)(1.00*16)},
{103, (int)(1.00*16), (int)(1.00*16)},
{107, (int)(1.00*16), (int)(1.00*16)},
{111, (int)(1.00*16), (int)(1.00*16)},
{114, (int)(1.00*16), (int)(1.00*16)},
{119, (int)(1.00*16), (int)(1.00*16)},
{123, (int)(1.00*16), (int)(1.00*16)},
{127, (int)(1.00*16), (int)(1.00*16)},
{131, (int)(1.00*16), (int)(1.00*16)},
{136, (int)(1.00*16), (int)(1.00*16)},
{141, (int)(1.00*16), (int)(1.00*16)},
{146, (int)(1.00*16), (int)(1.00*16)},
{151, (int)(1.00*16), (int)(1.00*16)},
{156, (int)(1.00*16), (int)(1.00*16)},
{162, (int)(1.00*16), (int)(1.00*16)},
{168, (int)(1.00*16), (int)(1.00*16)},
{173, (int)(1.00*16), (int)(1.00*16)},
{180, (int)(1.00*16), (int)(1.00*16)},
{186, (int)(1.00*16), (int)(1.00*16)},
{193, (int)(1.00*16), (int)(1.00*16)},
{199, (int)(1.00*16), (int)(1.00*16)},
{206, (int)(1.00*16), (int)(1.00*16)},
{214, (int)(1.00*16), (int)(1.00*16)},
{221, (int)(1.00*16), (int)(1.00*16)},
{229, (int)(1.00*16), (int)(1.00*16)},
{237, (int)(1.00*16), (int)(1.00*16)},
{245, (int)(1.00*16), (int)(1.00*16)},
{254, (int)(1.00*16), (int)(1.00*16)},
{263, (int)(1.00*16), (int)(1.00*16)},
{272, (int)(1.00*16), (int)(1.00*16)},
{282, (int)(1.00*16), (int)(1.00*16)},
{292, (int)(1.00*16), (int)(1.00*16)},
{302, (int)(1.00*16), (int)(1.00*16)},
{313, (int)(1.00*16), (int)(1.00*16)},
{324, (int)(1.00*16), (int)(1.00*16)},
{335, (int)(1.00*16), (int)(1.00*16)},
{338, (int)(1.00*16), (int)(1.00*16)},
{338, (int)(1.0625*16), (int)(1.00*16)},
{338, (int)(1.0625*16), (int)(1.00*16)},
{338, (int)(1.125*16), (int)(1.00*16)},
{338, (int)(1.125*16), (int)(1.00*16)},
{338, (int)(1.1875*16), (int)(1.00*16)},
{338, (int)(1.25*16), (int)(1.00*16)},
{338, (int)(1.25*16), (int)(1.00*16)},
{338, (int)(1.3125*16), (int)(1.00*16)},
{338, (int)(1.375*16), (int)(1.00*16)},
{338, (int)(1.4375*16), (int)(1.00*16)},
{338, (int)(1.4375*16), (int)(1.00*16)},
{338, (int)(1.5*16), (int)(1.00*16)},
{338, (int)(1.5625*16), (int)(1.00*16)},
{338, (int)(1.625*16), (int)(1.00*16)},
{338, (int)(1.6875*16), (int)(1.00*16)},
{338, (int)(1.75*16), (int)(1.00*16)},
{338, (int)(1.8125*16), (int)(1.00*16)},
{338, (int)(1.875*16), (int)(1.00*16)},
{338, (int)(1.9375*16), (int)(1.00*16)},
{676, (int)(1.00*16), (int)(1.00*16)},
{676, (int)(1.0625*16), (int)(1.00*16)},
{676, (int)(1.0625*16), (int)(1.00*16)},
{676, (int)(1.125*16), (int)(1.00*16)},
{676, (int)(1.125*16), (int)(1.00*16)},
{676, (int)(1.1875*16), (int)(1.00*16)},
{676, (int)(1.25*16), (int)(1.00*16)},
{676, (int)(1.25*16), (int)(1.00*16)},
{676, (int)(1.3125*16), (int)(1.00*16)},
{676, (int)(1.375*16), (int)(1.00*16)},
{676, (int)(1.4375*16), (int)(1.00*16)},
{676, (int)(1.4375*16), (int)(1.00*16)},
{676, (int)(1.5*16), (int)(1.00*16)},
{676, (int)(1.5625*16), (int)(1.00*16)},
{676, (int)(1.625*16), (int)(1.00*16)},
{676, (int)(1.6875*16), (int)(1.00*16)},
{676, (int)(1.75*16), (int)(1.00*16)},
{676, (int)(1.8125*16), (int)(1.00*16)},
{676, (int)(1.875*16), (int)(1.00*16)},
{676, (int)(1.9375*16), (int)(1.00*16)},
{1015, (int)(1.3125*16), (int)(1.00*16)},
{1015, (int)(1.375*16), (int)(1.00*16)},
{1015, (int)(1.4375*16), (int)(1.00*16)},
{1015, (int)(1.4375*16), (int)(1.00*16)},
{1015, (int)(1.5*16), (int)(1.00*16)},
{1015, (int)(1.5625*16), (int)(1.00*16)},
{1015, (int)(1.625*16), (int)(1.00*16)},
{1015, (int)(1.6875*16), (int)(1.00*16)},
{1015, (int)(1.75*16), (int)(1.00*16)},
{1015, (int)(1.8125*16), (int)(1.00*16)},
{1015, (int)(1.875*16), (int)(1.00*16)},
{1015, (int)(1.9375*16), (int)(1.00*16)},
{1015, (int)(2.0*16), (int)(1.00*16)},
{1015, (int)(2.07*16), (int)(1.00*16)},
{1015, (int)(2.14*16), (int)(1.00*16)},
{1015, (int)(2.22*16), (int)(1.00*16)},
{1015, (int)(2.30*16), (int)(1.00*16)},
{1015, (int)(2.38*16), (int)(1.00*16)},
{1015, (int)(2.46*16), (int)(1.00*16)},
{1015, (int)(2.55*16), (int)(1.00*16)},
{1015, (int)(2.64*16), (int)(1.00*16)},
{1015, (int)(2.73*16), (int)(1.00*16)},
{1015, (int)(2.83*16), (int)(1.00*16)},
{1015, (int)(2.93*16), (int)(1.00*16)},
{1015, (int)(3.03*16), (int)(1.00*16)},
{1015, (int)(3.14*16), (int)(1.00*16)},
{1015, (int)(3.25*16), (int)(1.00*16)},
{1015, (int)(3.36*16), (int)(1.00*16)},
{1015, (int)(3.48*16), (int)(1.00*16)},
{1015, (int)(3.61*16), (int)(1.00*16)},
{1015, (int)(3.73*16), (int)(1.00*16)},
{1015, (int)(3.86*16), (int)(1.00*16)},
{1015, (int)(4.00*16), (int)(1.00*16)},
{1015, (int)(4.14*16), (int)(1.00*16)},
{1015, (int)(4.29*16), (int)(1.00*16)},
{1015, (int)(4.44*16), (int)(1.00*16)},
{1015, (int)(4.59*16), (int)(1.00*16)},
{1015, (int)(4.76*16), (int)(1.00*16)},
{1015, (int)(4.92*16), (int)(1.00*16)},
{1015, (int)(5.10*16), (int)(1.00*16)},
{1015, (int)(5.28*16), (int)(1.00*16)},
{1015, (int)(5.46*16), (int)(1.00*16)},
{1015, (int)(5.66*16), (int)(1.00*16)},
{1015, (int)(5.86*16), (int)(1.00*16)},
{1015, (int)(6.06*16), (int)(1.00*16)},
{1015, (int)(6.28*16), (int)(1.00*16)},
{1015, (int)(6.50*16), (int)(1.00*16)},
{1015, (int)(6.73*16), (int)(1.00*16)},
{1015, (int)(6.96*16), (int)(1.00*16)},
{1015, (int)(7.21*16), (int)(1.00*16)},
{1015, (int)(7.46*16), (int)(1.00*16)},
{1015, (int)(7.73*16), (int)(1.00*16)},
{1015, (int)(8.00*16), (int)(1.00*16)},
{1015, (int)(8.28*16), (int)(1.00*16)},
{1015, (int)(8.57*16), (int)(1.00*16)},
{1015, (int)(8.88*16), (int)(1.00*16)},
{1015, (int)(9.19*16), (int)(1.00*16)},
{1015, (int)(9.51*16), (int)(1.00*16)},
{1015, (int)(9.85*16), (int)(1.00*16)},
{1015, (int)(10.20*16), (int)(1.00*16)},
{1015, (int)(10.56*16), (int)(1.00*16)},
{1015, (int)(10.93*16), (int)(1.00*16)},
{1015, (int)(11.31*16), (int)(1.00*16)},
{1015, (int)(11.71*16), (int)(1.00*16)},
{1015, (int)(12.13*16), (int)(1.00*16)},
{1015, (int)(12.55*16), (int)(1.00*16)},
{1015, (int)(13.00*16), (int)(1.00*16)},
{1015, (int)(13.45*16), (int)(1.00*16)},
{1015, (int)(13.93*16), (int)(1.00*16)},
{1015, (int)(14.42*16), (int)(1.00*16)},
{1015, (int)(14.93*16), (int)(1.00*16)},
{1015, (int)(15.45*16), (int)(1.00*16)},
{1015, (int)(16.00*16), (int)(1.00*16)}
};


////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S jxf02_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	jxf02_handle = drv_l2_sccb_open(JXF02_ID, 8, 8);
	if(jxf02_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C

#define	B_I2C1_TYPE                           15
#define	B_I2C1_IOC0IOC1_EN              (0x0 << B_I2C1_TYPE)
#define	B_I2C1_IOD4IOD5_EN              (0x1 << B_I2C1_TYPE)
#define MASK_I2C1_TYPE                	(0x1 << B_I2C1_TYPE)

    jxf02_handle.devNumber = I2C_1;
	jxf02_handle.slaveAddr = JXF02_ID;
	jxf02_handle.clkRate = 100;

	R_FUNPOS1 &= (~MASK_I2C1_TYPE);

//	gpio_init_io(IO_C1, GPIO_OUTPUT);
//	gpio_set_port_attribute(IO_C1, ATTRIBUTE_HIGH);
//	gpio_write_io(IO_C1, DATA_HIGH);

	drv_l1_i2c_init(jxf02_handle.devNumber);
#endif
	return STATUS_OK;
}

static void jxf02_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(jxf02_handle) {
		drv_l2_sccb_close(jxf02_handle);
		jxf02_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_1);
	jxf02_handle.slaveAddr = 0;
	jxf02_handle.clkRate = 0;
#endif
}

static INT32S jxf02_sccb_write(INT8U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(jxf02_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[2];

	data[0] = reg & 0xFF;
	data[1] = value & 0xFF;
	return drv_l1_i2c_bus_write(&jxf02_handle, data, 2);
#endif
}



static INT32S jxf02_sccb_read(INT8U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(jxf02_handle, reg, &data) >= 0) {
		*value = (INT16U)data;
		return STATUS_OK;
	} else {
		*value = 0xFF;
		DBG_PRINT("i2C read fail!\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data;

	data = reg & 0xFF;
	if(drv_l1_i2c_bus_write(&jxf02_handle, &data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&jxf02_handle, &data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data;
#endif
	return STATUS_OK;
}


static INT32S jxf02_sccb_write_table(regval16_t *pTable)
{

   /* jxf02_sccb_write(pTable->reg_num, 0x1234);
    while(1)
    {
        unsigned short tt;
        jxf02_sccb_read(pTable->reg_num, &tt);
		DBG_PRINT("sensor i2c reg[0x%04x] = 0, read = 0x%04x\r\n", pTable->reg_num,  tt);
    }*/

	while(1)
	{
        unsigned short tt;

		if (pTable->reg_num == 0 && pTable->value == 0) {
			break;
		}

		//DBG_PRINT("0x%04x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(jxf02_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("%s: sccb write fail: reg = 0x%x, val = 0x%x\r\n", __FUNCTION__, pTable->reg_num, pTable->value);
			continue;
		}

		//jxf02_sccb_read(pTable->reg_num, &tt);
		//DBG_PRINT("sensor i2c reg[0x%04x] = 0x%04x, read = 0x%04x\r\n", pTable->reg_num, pTable->value, tt);

		pTable++;
	}
	return STATUS_OK;
}




static int jxf02_cvt_agc_gain(int agc_gain)
{
	int jxf02_agc_gain, i;
	int pow2;

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

	jxf02_agc_gain = (pow2 << 4) + agc_gain;

	return jxf02_agc_gain;
}

static int jxf02_get_real_agc_gain(int agc_gain)
{
	int real_agc_gain;

	real_agc_gain = 0x10 + (agc_gain & 0x0f);
	real_agc_gain = real_agc_gain * (1 + ((agc_gain >> 4) & 1)) * (1 + ((agc_gain >> 5) & 1))
			* (1 + ((agc_gain >> 6) & 1)) * (1 + ((agc_gain >> 7) & 1)) * (1 + ((agc_gain >> 8) & 1));

	return real_agc_gain;
}


static int jxf02_set_xfps_exposure_time(sensor_exposure_t *si)
{
	unsigned char t1, t2;
	int idx, temp, ret;

	//printk("%s:%d\n", __FUNCTION__, __LINE__);
	si->sensor_ev_idx += si->ae_ev_step;
	if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
	if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;
#if RAW_MODE
	idx = (g_raw_mode_idx < 0) ? si->sensor_ev_idx * 3 : g_raw_mode_idx * 3;
	printk("%s:%d %d\n", __FUNCTION__, __LINE__, g_raw_mode_idx);
#else
	idx = si->sensor_ev_idx * 3;
#endif
	si ->time = p_expTime_table[idx];
	si ->analog_gain = p_expTime_table[idx+1];
    //DBG_PRINT("[EV=%d, offset=%d]: time = 0x%x, analog gain =0x%x\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);


	// exposure time
	if(si ->time != pre_sensor_time)
	{
		pre_sensor_time = si->time;
		temp = si->time;

		t1 = (temp & 0xff);
		t2 = (temp >> 8) & 0x00ff;

		ret = jxf02_sccb_write(0x01, t1);
		if(ret < 0) return ret;

		ret = jxf02_sccb_write(0x02, t2);
		if(ret < 0) {
		    DBG_PRINT("ERR: write sensor time = 0x%x!!!\r\n", temp);
            return ret;
		}
	}

	if(si ->analog_gain != pre_sensor_a_gain)
	{
		// gain
		pre_sensor_a_gain = si->analog_gain;

		temp = si->analog_gain;
		temp = jxf02_cvt_agc_gain(temp);
		t1 = temp & 0x007f;

		ret = jxf02_sccb_write(0x00, t1);
		if(ret < 0) {
            DBG_PRINT("ERR: write sensor gain = 0x%x, 0x%x, 0x%x !!!\r\n", t1, temp, si->analog_gain);
            return ret;
        }
	}

	return 0;
}



static void jxf02_set_ae(int ev_step)
{
    seInfo.ae_ev_step = ev_step;
    jxf02_set_xfps_exposure_time(&seInfo);
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)jxf02_set_ae;
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
static void mipi_jxf02_handle(INT32U event)
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


void jxf02_seinfo_init(void)
{
    seInfo.sensor_ev_idx = JXF02_30FPS_60HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= JXF02_30FPS_60HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= JXF02_30FPS_60HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = JXF02_30FPS_60HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = JXF02_30FPS_60HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)jxf02_30fps_exp_time_gain_60Hz;

	pre_sensor_time = 1;
	pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x00;

}

/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void jxf02_cdsp_mipi_init(void)
{
    int ret;

	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

    jxf02_seinfo_init();


	// cdsp init
	drv_l2_cdsp_open();

	// set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk
	//R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	//R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<3)|(1<<2));//0x02400042;


	// mclk output
	//drv_l2_sensor_set_mclkout(jxf02_cdsp_mipi_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);
    osDelay(10);

	// Reset
	gpio_set_port_attribute(IO_A14, ATTRIBUTE_HIGH);
	gpio_init_io(IO_A14, GPIO_OUTPUT);
	gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);
    gpio_write_io(IO_A14, DATA_LOW);
    osDelay(20);
    gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);

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

    DBG_PRINT("Sensor JXF02 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void jxf02_cdsp_mipi_uninit(void)
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
	jxf02_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void jxf02_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
    INT16U R0x30F0, R0x3072, R0x300E;
    int ret;
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);
	drv_l2_sensor_set_mclkout(MCLK_24M);


    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 3);

	drv_l2_sensor_set_mclkout(jxf02_cdsp_mipi_ops.info[index].mclk);
    osDelay(30);

    // set cdsp format
	format.image_source = C_CDSP_MIPI;
	format.input_format =  jxf02_cdsp_mipi_ops.info[index].input_format;
	format.output_format = jxf02_cdsp_mipi_ops.info[index].output_format;
	target_w = jxf02_cdsp_mipi_ops.info[index].target_w;
	target_h = jxf02_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = jxf02_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = jxf02_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = jxf02_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = jxf02_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = jxf02_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = jxf02_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = jxf02_cdsp_mipi_ops.info[index].vsync_active;


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
    jxf02_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
    jxf02_mipi_cfg.data_type = MIPI_RAW10;
    jxf02_mipi_cfg.h_back_porch = 4;

	jxf02_mipi_cfg.h_size = format.hpixel;
	jxf02_mipi_cfg.v_size = format.vline;
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      drv_l1_mipi_set_parameter(MIPI_1, &jxf02_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      drv_l1_mipi_set_parameter(MIPI_0, &jxf02_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
    #endif


  // reguest sccb
	jxf02_sccb_open();

	// init sensor
	jxf02_sccb_write_table((regval16_t *)jxf02_Mipi_Raw10_1080P_30fps);

    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    jxf02_set_xfps_exposure_time(&seInfo);

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
static void jxf02_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();

	jxf02_sccb_close();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* jxf02_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&jxf02_cdsp_mipi_ops.info[index];
	}
}


/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t jxf02_cdsp_mipi_ops =
{
	SENSOR_JXF02_CDSP_MIPI_NAME,		/* sensor name */
	jxf02_cdsp_mipi_init,
	jxf02_cdsp_mipi_uninit,
	jxf02_cdsp_mipi_stream_on,
	jxf02_cdsp_mipi_stream_off,
	jxf02_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SGBRG10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			JXF02_WIDTH,				/* sensor width */
			JXF02_HEIGHT, 				/* sensor height */
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
			JXF02_OUT_WIDTH,			/* target width */
			JXF02_OUT_HEIGHT, 		/* target height */
			JXF02_WIDTH,				/* sensor width */
			JXF02_HEIGHT, 				/* sensor height */
			4,							/* sensor h offset */
			0,							/* sensor v offset */
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
			JXF02_WIDTH,				/* sensor width */
			JXF02_HEIGHT, 				/* sensor height */
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
			JXF02_OUT_WIDTH,			/* target width */
			JXF02_OUT_HEIGHT, 			/* target height */
			JXF02_WIDTH,				/* sensor width */
			JXF02_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};

#endif //(defined _SENSOR_JXF02_CDSP_MIPI) && (_SENSOR_JXF02_CDSP_MIPI == 1)
