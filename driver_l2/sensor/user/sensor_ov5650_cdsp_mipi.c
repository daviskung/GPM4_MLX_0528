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
#include "drv_l2_user_preference.h"


#if (defined _SENSOR_OV5650_CDSP_MIPI) && (_SENSOR_OV5650_CDSP_MIPI == 1)

#include "drv_l2_user_calibration.h"
#include "drv_l2_user_preference.h"
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_TEST        0
#define COLOR_BAR_EN            0
#define MIPI_ISR_TEST           1
#define MIPI_LANE_NO			2	        // 1 or 2 lane
#define MIPI_DEV_NO             0           //0:MIPI_0 or 1:MIPI_1

#define	OV5650_ID						0x6C
#define OV5650_WIDTH				    2040// (2560)
#define OV5650_HEIGHT				    1150// (1920-8)

#define OV5650_OUT_WIDTH				1920
#define OV5650_OUT_HEIGHT				1088


#define OV5650_30FPS_50HZ_DAY_EV_IDX 			132
#define OV5650_30FPS_50HZ_NIGHT_EV_IDX			178
#define OV5650_30FPS_50HZ_EXP_TIME_TOTAL		    246
#define OV5650_30FPS_50HZ_INIT_EV_IDX 			180//((OV5650_30FPS_50HZ_DAY_EV_IDX + OV5650_30FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define OV5650_30FPS_50HZ_MAX_EV_IDX			    (OV5650_30FPS_50HZ_EXP_TIME_TOTAL - 10)


#define OV5650_30FPS_60HZ_DAY_EV_IDX 			132
#define OV5650_30FPS_60HZ_NIGHT_EV_IDX			178
#define OV5650_30FPS_60HZ_EXP_TIME_TOTAL		    249
#define OV5650_30FPS_60HZ_INIT_EV_IDX 			180//((OV5650_30FPS_60HZ_DAY_EV_IDX + OV5650_30FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define OV5650_30FPS_60HZ_MAX_EV_IDX			    (OV5650_30FPS_60HZ_EXP_TIME_TOTAL - 10)


#define OV5650_24FPS_50HZ_DAY_EV_IDX 			138
#define OV5650_24FPS_50HZ_NIGHT_EV_IDX			194
#define OV5650_24FPS_50HZ_EXP_TIME_TOTAL		    254
#define OV5650_24FPS_50HZ_INIT_EV_IDX 			((OV5650_24FPS_50HZ_DAY_EV_IDX + OV5650_24FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define OV5650_24FPS_50HZ_MAX_EV_IDX			    (OV5650_24FPS_50HZ_EXP_TIME_TOTAL - 10)


#define OV5650_24FPS_60HZ_DAY_EV_IDX 			135
#define OV5650_24FPS_60HZ_NIGHT_EV_IDX			195
#define OV5650_24FPS_60HZ_EXP_TIME_TOTAL		    255
#define OV5650_24FPS_60HZ_INIT_EV_IDX 			((OV5650_24FPS_60HZ_DAY_EV_IDX + OV5650_24FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define OV5650_24FPS_60HZ_MAX_EV_IDX			    (OV5650_24FPS_60HZ_EXP_TIME_TOTAL - 10)




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
static int pre_sensor_a_gain, pre_sensor_time;


#if SCCB_MODE == SCCB_GPIO
	static void *ov5650_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t ov5650_handle;
#endif

static mipi_config_t ov5650_mipi_cfg =
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

	OV5650_WIDTH,		/* width, 0~0xFFFF */
	OV5650_HEIGHT,		/* height, 0~0xFFFF */
	4, 					/* back porch, 0~0xF */
	4,					/* front porch, 0~0xF */
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,				/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	160,				/* data mask, unit is ns */
	MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};



// 2560x1920
const regval16_t ov5650_Mipi_Raw10_5M_15fps[] =
{
{0x3008, 0x82},

{0xffff, 0xff}, // delay 1ms

{0x3008, 0x42},
{0x3000, 0x30},
{0x3103, 0x93},
{0x3007, 0x3B},

{0x3010, 0x10},
{0x3011, 0x14},
{0x3012, 0x02},

{0x3706, 0x41},
{0x3703, 0xe6},
{0x3613, 0x44},
{0x3630, 0x22},
{0x3605, 0x04},
{0x3606, 0x3f},
{0x3712, 0x13},
{0x370e, 0x00},
{0x370b, 0x40},
{0x3600, 0x54},
{0x3601, 0x05},
{0x3713, 0x22},
{0x3714, 0x27},
{0x3631, 0x36},
{0x3612, 0x1a},
{0x3604, 0x40},
{0x3705, 0xda},
{0x3709, 0x40},
{0x370a, 0x40},
{0x370c, 0x00},
{0x3710, 0x28},
{0x3702, 0x3a},
{0x3704, 0x18},
{0x3603, 0xa7},
{0x3615, 0x52},
{0x3632, 0x5f},
{0x3620, 0x56},

//{0x3621, 0x2f},  // B[4] = 0 for mirror on
{0x3621, 0x3f}, // B[4] = 1 for mirror off

{0x3711, 0x24},
{0x370d, 0x04},
{0x3623, 0x01},
{0x3633, 0x24},
{0x3c01, 0x34},
{0x3c04, 0x28},
{0x3c05, 0x98},
{0x3c07, 0x07},
{0x3c09, 0xc2},
{0x3a18, 0x0},
{0x3a19, 0xf8},
{0x3a00, 0x38},
{0x3a08, 0x16},
{0x3a09, 0x48},
{0x3a0a, 0x12},
{0x3a0b, 0x90},
{0x3a0d, 0x3},
{0x3a0e, 0x3},
{0x3a13, 0x54},
{0x3a1a, 0x6},
{0x3503, 0x37}, // AE off

// HREF start point
{0x3800, 0x2},
{0x3801, 0x22},

// VREF start point
{0x3802, 0x00},
{0x3803, 0x10},

// HREF width
{0x3804, 0xa},
{0x3805, 0x00},

// VREF Height
{0x3806, 0x7},
{0x3807, 0x80},

// DVP horizontal output size
{0x3808, 0xa},
{0x3809, 0x00},

// DVP vertical output size
{0x380a, 0x7},
{0x380b, 0x80}, // 1092+4

// total horizontal size
{0x380c, 0x12},
{0x380d, 0x7b},

// total vertical size
{0x380e, 0x7},
{0x380f, 0x90},

{0x3810, 0x40},
{0x381c, 0x00},
{0x381d, 0x10},
{0x381e, 0x4},
{0x381f, 0xa0},
{0x3820, 0x0},
{0x3821, 0x20},
{0x3824, 0x22},
{0x3825, 0x64},
{0x3826, 0x0},
{0x3827, 0xa},
{0x3815, 0x82},
{0x3830, 0x50},
{0x3836, 0x00},
{0x381a, 0x3c},

//{0x3818, 0xc0},
{0x3818, 0xa0}, // flip only



{0x401f, 0x03},
{0x4000, 0x05}, // BLC enable

// BLC target
{0x4006, 0x00},
{0x4007, 0x10},

// R401D[3]=0:BLC keep every frame;1:BLC trig by Gain change
// R401D[5]=1 for auto BLC update by gain changed at preview life video mode and
// R401D[5]=0 for BLC auto update by every frame at still capture mode
{0x401d, 0x28},

{0x4001, 0x02},
{0x401c, 0x46},

// R5000[2]=1 for black pixels compensate
// R5000[1]=1 for white pixels compensate
// suggest only enable white pixels compensate and disable black pixels compensate
{0x5000, 0x6},

{0x5001, 0x01},
{0x5002, 0x0},
{0x503d, 0x0},
{0x5046, 0x81},
{0x505f, 0x04},
{0x4801, 0x0f},
{0x3003, 0x03},
{0x300e, 0x0c},
{0x4803, 0x50},
{0x4800, 0x4},
{0x4837, 0x15},
{0x300f, 0x8f},
{0x3003, 0x1},
{0x3008, 0x2},



// exposure time
{0x3500, 0x00},
{0x3501, 0x43},
{0x3502, 0x80},

// AGC gain
{0x350a, 0x00},
{0x350b, 0x00},

// wb gain
{0x3406, 0x01},
{0x3400, 0x04},
{0x3401, 0x00},
{0x3402, 0x04},
{0x3403, 0x00},
{0x3404, 0x04},
{0x3405, 0x00},

{0x0000, 0x00}
};


const regval16_t ov5650_Mipi_Raw10_1080P_30fps[] =
{
{0x3008, 0x82},

{0xffff, 0xff}, // delay 1ms

{0x3008, 0x42},
{0x3000, 0x30},
{0x3103, 0x93},
{0x3007, 0x3B},

{0x3010, 0x10},
{0x3011, 0x14},
{0x3012, 0x02},

{0x3706, 0x41},
{0x3703, 0xe6},
{0x3613, 0x44},
{0x3630, 0x22},
{0x3605, 0x04},
{0x3606, 0x3f},
{0x3712, 0x13},
{0x370e, 0x00},
{0x370b, 0x40},
{0x3600, 0x54},
{0x3601, 0x05},
{0x3713, 0x22},
{0x3714, 0x27},
{0x3631, 0x36},
{0x3612, 0x1a},
{0x3604, 0x40},
{0x3705, 0xda},
{0x3709, 0x40},
{0x370a, 0x40},
{0x370c, 0x00},
{0x3710, 0x28},
{0x3702, 0x3a},
{0x3704, 0x18},
{0x3603, 0xa7},
{0x3615, 0x52},
{0x3632, 0x5f},
{0x3620, 0x56},

//{0x3621, 0x2f},  // B[4] = 0 for mirror on
{0x3621, 0x3f}, // B[4] = 1 for mirror off

{0x3711, 0x24},
{0x370d, 0x04},
{0x3623, 0x01},
{0x3633, 0x24},
{0x3c01, 0x34},
{0x3c04, 0x28},
{0x3c05, 0x98},
{0x3c07, 0x07},
{0x3c09, 0xc2},
{0x3a18, 0x0},
{0x3a19, 0xf8},
{0x3a00, 0x38},
{0x3a08, 0x16},
{0x3a09, 0x48},
{0x3a0a, 0x12},
{0x3a0b, 0x90},
{0x3a0d, 0x3},
{0x3a0e, 0x3},
{0x3a13, 0x54},
{0x3a1a, 0x6},
{0x3503, 0x37}, // AE off

// HREF start point
{0x3800, 0x2},
{0x3801, 0x22},

// VREF start point
{0x3802, 0x00},
{0x3803, 0x10},

// HREF width
{0x3804, 0x7},
{0x3805, 0xf8},

// VREF Height
{0x3806, 0x4},
{0x3807, 0x80},

// DVP horizontal output size
{0x3808, 0x7},
{0x3809, 0xf8},

// DVP vertical output size
{0x380a, 0x4},
{0x380b, 0x80}, // 1092+4

// total horizontal size
{0x380c, 0x0a},
{0x380d, 0xb3},

// total vertical size
{0x380e, 0x4},
{0x380f, 0xa8},

{0x3810, 0x40},
{0x381c, 0x31},
{0x381d, 0xa4},
{0x381e, 0x4},
{0x381f, 0xa0},
{0x3820, 0x4},
{0x3821, 0x1a},
{0x3824, 0x22},
{0x3825, 0xa8},
{0x3827, 0xa},
{0x3815, 0x82},
{0x3830, 0x50},
{0x3836, 0x00},
{0x381a, 0x3c},

//{0x3818, 0xc0},
{0x3818, 0xa0}, // flip only



{0x401f, 0x03},
{0x4000, 0x05}, // BLC enable

// BLC target
{0x4006, 0x00},
{0x4007, 0x10},

// R401D[3]=0:BLC keep every frame;1:BLC trig by Gain change
// R401D[5]=1 for auto BLC update by gain changed at preview life video mode and
// R401D[5]=0 for BLC auto update by every frame at still capture mode
{0x401d, 0x28},

{0x4001, 0x02},
{0x401c, 0x46},

// R5000[2]=1 for black pixels compensate
// R5000[1]=1 for white pixels compensate
// suggest only enable white pixels compensate and disable black pixels compensate
{0x5000, 0x6},

{0x5001, 0x01},
{0x5002, 0x0},
{0x503d, 0x0},
{0x5046, 0x81},
{0x505f, 0x04},
{0x4801, 0x0f},
{0x3003, 0x03},
{0x300e, 0x0c},
{0x4803, 0x50},
{0x4800, 0x4},
{0x4837, 0x15},
{0x300f, 0x8f},
{0x3003, 0x1},
{0x3008, 0x2},



// exposure time
{0x3500, 0x00},
{0x3501, 0x43},
{0x3502, 0x80},

// AGC gain
{0x350a, 0x00},
{0x350b, 0x00},

// wb gain
{0x3406, 0x01},
{0x3400, 0x04},
{0x3401, 0x00},
{0x3402, 0x04},
{0x3403, 0x00},
{0x3404, 0x04},
{0x3405, 0x00},

{0x0000, 0x00}
};



const regval16_t ov5650_Mipi_Raw10_1080P_24fps[] =
{
};


static const  int ov5650_30fps_exp_time_gain_60Hz[OV5650_30FPS_60HZ_EXP_TIME_TOTAL][3] =
{
{4, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{9, (int)(1.00*16), (int)(1.00*256)},
{9, (int)(1.00*16), (int)(1.00*256)},
{9, (int)(1.00*16), (int)(1.00*256)},
{10, (int)(1.00*16), (int)(1.00*256)},
{10, (int)(1.00*16), (int)(1.00*256)},
{10, (int)(1.00*16), (int)(1.00*256)},
{11, (int)(1.00*16), (int)(1.00*256)},
{11, (int)(1.00*16), (int)(1.00*256)},
{11, (int)(1.00*16), (int)(1.00*256)},
{12, (int)(1.00*16), (int)(1.00*256)},
{12, (int)(1.00*16), (int)(1.00*256)},
{13, (int)(1.00*16), (int)(1.00*256)},
{13, (int)(1.00*16), (int)(1.00*256)},
{14, (int)(1.00*16), (int)(1.00*256)},
{14, (int)(1.00*16), (int)(1.00*256)},
{15, (int)(1.00*16), (int)(1.00*256)},
{15, (int)(1.00*16), (int)(1.00*256)},
{16, (int)(1.00*16), (int)(1.00*256)},
{16, (int)(1.00*16), (int)(1.00*256)},
{17, (int)(1.00*16), (int)(1.00*256)},
{17, (int)(1.00*16), (int)(1.00*256)},
{18, (int)(1.00*16), (int)(1.00*256)},
{19, (int)(1.00*16), (int)(1.00*256)},
{19, (int)(1.00*16), (int)(1.00*256)},
{20, (int)(1.00*16), (int)(1.00*256)},
{21, (int)(1.00*16), (int)(1.00*256)},
{21, (int)(1.00*16), (int)(1.00*256)},
{22, (int)(1.00*16), (int)(1.00*256)},
{23, (int)(1.00*16), (int)(1.00*256)},
{24, (int)(1.00*16), (int)(1.00*256)},
{24, (int)(1.00*16), (int)(1.00*256)},
{25, (int)(1.00*16), (int)(1.00*256)},
{26, (int)(1.00*16), (int)(1.00*256)},
{27, (int)(1.00*16), (int)(1.00*256)},
{28, (int)(1.00*16), (int)(1.00*256)},
{29, (int)(1.00*16), (int)(1.00*256)},
{30, (int)(1.00*16), (int)(1.00*256)},
{31, (int)(1.00*16), (int)(1.00*256)},
{32, (int)(1.00*16), (int)(1.00*256)},
{33, (int)(1.00*16), (int)(1.00*256)},
{35, (int)(1.00*16), (int)(1.00*256)},
{36, (int)(1.00*16), (int)(1.00*256)},
{37, (int)(1.00*16), (int)(1.00*256)},
{38, (int)(1.00*16), (int)(1.00*256)},
{40, (int)(1.00*16), (int)(1.00*256)},
{41, (int)(1.00*16), (int)(1.00*256)},
{43, (int)(1.00*16), (int)(1.00*256)},
{44, (int)(1.00*16), (int)(1.00*256)},
{46, (int)(1.00*16), (int)(1.00*256)},
{47, (int)(1.00*16), (int)(1.00*256)},
{49, (int)(1.00*16), (int)(1.00*256)},
{51, (int)(1.00*16), (int)(1.00*256)},
{53, (int)(1.00*16), (int)(1.00*256)},
{54, (int)(1.00*16), (int)(1.00*256)},
{56, (int)(1.00*16), (int)(1.00*256)},
{58, (int)(1.00*16), (int)(1.00*256)},
{60, (int)(1.00*16), (int)(1.00*256)},
{62, (int)(1.00*16), (int)(1.00*256)},
{65, (int)(1.00*16), (int)(1.00*256)},
{67, (int)(1.00*16), (int)(1.00*256)},
{69, (int)(1.00*16), (int)(1.00*256)},
{72, (int)(1.00*16), (int)(1.00*256)},
{74, (int)(1.00*16), (int)(1.00*256)},
{77, (int)(1.00*16), (int)(1.00*256)},
{80, (int)(1.00*16), (int)(1.00*256)},
{82, (int)(1.00*16), (int)(1.00*256)},
{85, (int)(1.00*16), (int)(1.00*256)},
{88, (int)(1.00*16), (int)(1.00*256)},
{91, (int)(1.00*16), (int)(1.00*256)},
{95, (int)(1.00*16), (int)(1.00*256)},
{98, (int)(1.00*16), (int)(1.00*256)},
{101, (int)(1.00*16), (int)(1.00*256)},
{105, (int)(1.00*16), (int)(1.00*256)},
{109, (int)(1.00*16), (int)(1.00*256)},
{113, (int)(1.00*16), (int)(1.00*256)},
{117, (int)(1.00*16), (int)(1.00*256)},
{121, (int)(1.00*16), (int)(1.00*256)},
{125, (int)(1.00*16), (int)(1.00*256)},
{129, (int)(1.00*16), (int)(1.00*256)},
{134, (int)(1.00*16), (int)(1.00*256)},
{139, (int)(1.00*16), (int)(1.00*256)},
{143, (int)(1.00*16), (int)(1.00*256)},
{149, (int)(1.00*16), (int)(1.00*256)},
{154, (int)(1.00*16), (int)(1.00*256)},
{159, (int)(1.00*16), (int)(1.00*256)},
{165, (int)(1.00*16), (int)(1.00*256)},
{171, (int)(1.00*16), (int)(1.00*256)},
{177, (int)(1.00*16), (int)(1.00*256)},
{183, (int)(1.00*16), (int)(1.00*256)},
{189, (int)(1.00*16), (int)(1.00*256)},
{196, (int)(1.00*16), (int)(1.00*256)},
{203, (int)(1.00*16), (int)(1.00*256)},
{210, (int)(1.00*16), (int)(1.00*256)},
{217, (int)(1.00*16), (int)(1.00*256)},
{225, (int)(1.00*16), (int)(1.00*256)},
{233, (int)(1.00*16), (int)(1.00*256)},
{241, (int)(1.00*16), (int)(1.00*256)},
{250, (int)(1.00*16), (int)(1.00*256)},
{259, (int)(1.00*16), (int)(1.00*256)},
{268, (int)(1.00*16), (int)(1.00*256)},
{277, (int)(1.00*16), (int)(1.00*256)},
{287, (int)(1.00*16), (int)(1.00*256)},
{297, (int)(1.00*16), (int)(1.00*256)},
{297, (int)(1.0625*16), (int)(1.00*256)},
{297, (int)(1.0625*16), (int)(1.00*256)},
{297, (int)(1.125*16), (int)(1.00*256)},
{297, (int)(1.125*16), (int)(1.00*256)},
{297, (int)(1.1875*16), (int)(1.00*256)},
{297, (int)(1.25*16), (int)(1.00*256)},
{297, (int)(1.25*16), (int)(1.00*256)},
{297, (int)(1.3125*16), (int)(1.00*256)},
{297, (int)(1.375*16), (int)(1.00*256)},
{297, (int)(1.4375*16), (int)(1.00*256)},
{297, (int)(1.4375*16), (int)(1.00*256)},
{297, (int)(1.5*16), (int)(1.00*256)},
{297, (int)(1.5625*16), (int)(1.00*256)},
{297, (int)(1.625*16), (int)(1.00*256)},
{297, (int)(1.6875*16), (int)(1.00*256)},
{297, (int)(1.75*16), (int)(1.00*256)},
{297, (int)(1.8125*16), (int)(1.00*256)},
{297, (int)(1.875*16), (int)(1.00*256)},
{297, (int)(1.9375*16), (int)(1.00*256)},
{594, (int)(1.00*16), (int)(1.00*256)},
{594, (int)(1.0625*16), (int)(1.00*256)},
{594, (int)(1.0625*16), (int)(1.00*256)},
{594, (int)(1.125*16), (int)(1.00*256)},
{594, (int)(1.125*16), (int)(1.00*256)},
{594, (int)(1.1875*16), (int)(1.00*256)},
{594, (int)(1.25*16), (int)(1.00*256)},
{594, (int)(1.25*16), (int)(1.00*256)},
{594, (int)(1.3125*16), (int)(1.00*256)},
{594, (int)(1.375*16), (int)(1.00*256)},
{594, (int)(1.4375*16), (int)(1.00*256)},
{594, (int)(1.4375*16), (int)(1.00*256)},
{594, (int)(1.5*16), (int)(1.00*256)},
{594, (int)(1.5625*16), (int)(1.00*256)},
{594, (int)(1.625*16), (int)(1.00*256)},
{594, (int)(1.6875*16), (int)(1.00*256)},
{594, (int)(1.75*16), (int)(1.00*256)},
{594, (int)(1.8125*16), (int)(1.00*256)},
{594, (int)(1.875*16), (int)(1.00*256)},
{594, (int)(1.9375*16), (int)(1.00*256)},
{891, (int)(1.3125*16), (int)(1.00*256)},
{891, (int)(1.375*16), (int)(1.00*256)},
{891, (int)(1.4375*16), (int)(1.00*256)},
{891, (int)(1.4375*16), (int)(1.00*256)},
{891, (int)(1.5*16), (int)(1.00*256)},
{891, (int)(1.5625*16), (int)(1.00*256)},
{891, (int)(1.625*16), (int)(1.00*256)},
{891, (int)(1.6875*16), (int)(1.00*256)},
{891, (int)(1.75*16), (int)(1.00*256)},
{891, (int)(1.8125*16), (int)(1.00*256)},
{891, (int)(1.875*16), (int)(1.00*256)},
{891, (int)(1.9375*16), (int)(1.00*256)},
{1188, (int)(1.5*16), (int)(1.00*256)},
{1188, (int)(1.5625*16), (int)(1.00*256)},
{1188, (int)(1.625*16), (int)(1.00*256)},
{1188, (int)(1.6875*16), (int)(1.00*256)},
{1188, (int)(1.75*16), (int)(1.00*256)},
{1188, (int)(1.8125*16), (int)(1.00*256)},
{1188, (int)(1.875*16), (int)(1.00*256)},
{1188, (int)(1.9375*16), (int)(1.00*256)},
{1188, (int)(2.0*16), (int)(1.00*256)},
{1188, (int)(2.07*16), (int)(1.00*256)},
{1188, (int)(2.14*16), (int)(1.00*256)},
{1188, (int)(2.22*16), (int)(1.00*256)},
{1188, (int)(2.30*16), (int)(1.00*256)},
{1188, (int)(2.38*16), (int)(1.00*256)},
{1188, (int)(2.46*16), (int)(1.00*256)},
{1188, (int)(2.55*16), (int)(1.00*256)},
{1188, (int)(2.64*16), (int)(1.00*256)},
{1188, (int)(2.73*16), (int)(1.00*256)},
{1188, (int)(2.83*16), (int)(1.00*256)},
{1188, (int)(2.93*16), (int)(1.00*256)},
{1188, (int)(3.03*16), (int)(1.00*256)},
{1188, (int)(3.14*16), (int)(1.00*256)},
{1188, (int)(3.25*16), (int)(1.00*256)},
{1188, (int)(3.36*16), (int)(1.00*256)},
{1188, (int)(3.48*16), (int)(1.00*256)},
{1188, (int)(3.61*16), (int)(1.00*256)},
{1188, (int)(3.73*16), (int)(1.00*256)},
{1188, (int)(3.86*16), (int)(1.00*256)},
{1188, (int)(4.00*16), (int)(1.00*256)},
{1188, (int)(4.14*16), (int)(1.00*256)},
{1188, (int)(4.29*16), (int)(1.00*256)},
{1188, (int)(4.44*16), (int)(1.00*256)},
{1188, (int)(4.59*16), (int)(1.00*256)},
{1188, (int)(4.76*16), (int)(1.00*256)},
{1188, (int)(4.92*16), (int)(1.00*256)},
{1188, (int)(5.10*16), (int)(1.00*256)},
{1188, (int)(5.28*16), (int)(1.00*256)},
{1188, (int)(5.46*16), (int)(1.00*256)},
{1188, (int)(5.66*16), (int)(1.00*256)},
{1188, (int)(5.86*16), (int)(1.00*256)},
{1188, (int)(6.06*16), (int)(1.00*256)},
{1188, (int)(6.28*16), (int)(1.00*256)},
{1188, (int)(6.50*16), (int)(1.00*256)},
{1188, (int)(6.73*16), (int)(1.00*256)},
{1188, (int)(6.96*16), (int)(1.00*256)},
{1188, (int)(7.21*16), (int)(1.00*256)},
{1188, (int)(7.46*16), (int)(1.00*256)},
{1188, (int)(7.73*16), (int)(1.00*256)},
{1188, (int)(8.00*16), (int)(1.00*256)},
{1188, (int)(8.28*16), (int)(1.00*256)},
{1188, (int)(8.57*16), (int)(1.00*256)},
{1188, (int)(8.88*16), (int)(1.00*256)},
{1188, (int)(9.19*16), (int)(1.00*256)},
{1188, (int)(9.51*16), (int)(1.00*256)},
{1188, (int)(9.85*16), (int)(1.00*256)},
{1188, (int)(10.20*16), (int)(1.00*256)},
{1188, (int)(10.56*16), (int)(1.00*256)},
{1188, (int)(10.93*16), (int)(1.00*256)},
{1188, (int)(11.31*16), (int)(1.00*256)},
{1188, (int)(11.71*16), (int)(1.00*256)},
{1188, (int)(12.13*16), (int)(1.00*256)},
{1188, (int)(12.55*16), (int)(1.00*256)},
{1188, (int)(13.00*16), (int)(1.00*256)},
{1188, (int)(13.45*16), (int)(1.00*256)},
{1188, (int)(13.93*16), (int)(1.00*256)},
{1188, (int)(14.42*16), (int)(1.00*256)},
{1188, (int)(14.93*16), (int)(1.00*256)},
{1188, (int)(15.45*16), (int)(1.00*256)},
{1188, (int)(16.00*16), (int)(1.00*256)},
{1188, (int)(16.56*16), (int)(1.00*256)},
{1188, (int)(17.15*16), (int)(1.00*256)},
{1188, (int)(17.75*16), (int)(1.00*256)},
{1188, (int)(18.83*16), (int)(1.00*256)},
{1188, (int)(19.03*16), (int)(1.00*256)},
{1188, (int)(19.70*16), (int)(1.00*256)},
{1188, (int)(20.39*16), (int)(1.00*256)}
};

static const  int ov5650_30fps_exp_time_gain_50Hz[OV5650_30FPS_50HZ_EXP_TIME_TOTAL][3] =
{
{4, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{5, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{6, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{7, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{8, (int)(1.00*16), (int)(1.00*256)},
{9, (int)(1.00*16), (int)(1.00*256)},
{9, (int)(1.00*16), (int)(1.00*256)},
{9, (int)(1.00*16), (int)(1.00*256)},
{10, (int)(1.00*16), (int)(1.00*256)},
{10, (int)(1.00*16), (int)(1.00*256)},
{10, (int)(1.00*16), (int)(1.00*256)},
{11, (int)(1.00*16), (int)(1.00*256)},
{11, (int)(1.00*16), (int)(1.00*256)},
{12, (int)(1.00*16), (int)(1.00*256)},
{12, (int)(1.00*16), (int)(1.00*256)},
{12, (int)(1.00*16), (int)(1.00*256)},
{13, (int)(1.00*16), (int)(1.00*256)},
{13, (int)(1.00*16), (int)(1.00*256)},
{14, (int)(1.00*16), (int)(1.00*256)},
{14, (int)(1.00*16), (int)(1.00*256)},
{15, (int)(1.00*16), (int)(1.00*256)},
{15, (int)(1.00*16), (int)(1.00*256)},
{16, (int)(1.00*16), (int)(1.00*256)},
{16, (int)(1.00*16), (int)(1.00*256)},
{17, (int)(1.00*16), (int)(1.00*256)},
{18, (int)(1.00*16), (int)(1.00*256)},
{18, (int)(1.00*16), (int)(1.00*256)},
{19, (int)(1.00*16), (int)(1.00*256)},
{19, (int)(1.00*16), (int)(1.00*256)},
{20, (int)(1.00*16), (int)(1.00*256)},
{21, (int)(1.00*16), (int)(1.00*256)},
{22, (int)(1.00*16), (int)(1.00*256)},
{22, (int)(1.00*16), (int)(1.00*256)},
{23, (int)(1.00*16), (int)(1.00*256)},
{24, (int)(1.00*16), (int)(1.00*256)},
{25, (int)(1.00*16), (int)(1.00*256)},
{26, (int)(1.00*16), (int)(1.00*256)},
{27, (int)(1.00*16), (int)(1.00*256)},
{27, (int)(1.00*16), (int)(1.00*256)},
{28, (int)(1.00*16), (int)(1.00*256)},
{29, (int)(1.00*16), (int)(1.00*256)},
{30, (int)(1.00*16), (int)(1.00*256)},
{32, (int)(1.00*16), (int)(1.00*256)},
{33, (int)(1.00*16), (int)(1.00*256)},
{34, (int)(1.00*16), (int)(1.00*256)},
{35, (int)(1.00*16), (int)(1.00*256)},
{36, (int)(1.00*16), (int)(1.00*256)},
{38, (int)(1.00*16), (int)(1.00*256)},
{39, (int)(1.00*16), (int)(1.00*256)},
{40, (int)(1.00*16), (int)(1.00*256)},
{42, (int)(1.00*16), (int)(1.00*256)},
{43, (int)(1.00*16), (int)(1.00*256)},
{45, (int)(1.00*16), (int)(1.00*256)},
{46, (int)(1.00*16), (int)(1.00*256)},
{48, (int)(1.00*16), (int)(1.00*256)},
{50, (int)(1.00*16), (int)(1.00*256)},
{51, (int)(1.00*16), (int)(1.00*256)},
{53, (int)(1.00*16), (int)(1.00*256)},
{55, (int)(1.00*16), (int)(1.00*256)},
{57, (int)(1.00*16), (int)(1.00*256)},
{59, (int)(1.00*16), (int)(1.00*256)},
{61, (int)(1.00*16), (int)(1.00*256)},
{63, (int)(1.00*16), (int)(1.00*256)},
{65, (int)(1.00*16), (int)(1.00*256)},
{68, (int)(1.00*16), (int)(1.00*256)},
{70, (int)(1.00*16), (int)(1.00*256)},
{72, (int)(1.00*16), (int)(1.00*256)},
{75, (int)(1.00*16), (int)(1.00*256)},
{78, (int)(1.00*16), (int)(1.00*256)},
{80, (int)(1.00*16), (int)(1.00*256)},
{83, (int)(1.00*16), (int)(1.00*256)},
{86, (int)(1.00*16), (int)(1.00*256)},
{89, (int)(1.00*16), (int)(1.00*256)},
{92, (int)(1.00*16), (int)(1.00*256)},
{96, (int)(1.00*16), (int)(1.00*256)},
{99, (int)(1.00*16), (int)(1.00*256)},
{103, (int)(1.00*16), (int)(1.00*256)},
{106, (int)(1.00*16), (int)(1.00*256)},
{110, (int)(1.00*16), (int)(1.00*256)},
{114, (int)(1.00*16), (int)(1.00*256)},
{118, (int)(1.00*16), (int)(1.00*256)},
{122, (int)(1.00*16), (int)(1.00*256)},
{126, (int)(1.00*16), (int)(1.00*256)},
{131, (int)(1.00*16), (int)(1.00*256)},
{135, (int)(1.00*16), (int)(1.00*256)},
{140, (int)(1.00*16), (int)(1.00*256)},
{145, (int)(1.00*16), (int)(1.00*256)},
{150, (int)(1.00*16), (int)(1.00*256)},
{155, (int)(1.00*16), (int)(1.00*256)},
{161, (int)(1.00*16), (int)(1.00*256)},
{167, (int)(1.00*16), (int)(1.00*256)},
{172, (int)(1.00*16), (int)(1.00*256)},
{179, (int)(1.00*16), (int)(1.00*256)},
{185, (int)(1.00*16), (int)(1.00*256)},
{191, (int)(1.00*16), (int)(1.00*256)},
{198, (int)(1.00*16), (int)(1.00*256)},
{205, (int)(1.00*16), (int)(1.00*256)},
{212, (int)(1.00*16), (int)(1.00*256)},
{220, (int)(1.00*16), (int)(1.00*256)},
{228, (int)(1.00*16), (int)(1.00*256)},
{236, (int)(1.00*16), (int)(1.00*256)},
{244, (int)(1.00*16), (int)(1.00*256)},
{252, (int)(1.00*16), (int)(1.00*256)},
{261, (int)(1.00*16), (int)(1.00*256)},
{271, (int)(1.00*16), (int)(1.00*256)},
{280, (int)(1.00*16), (int)(1.00*256)},
{290, (int)(1.00*16), (int)(1.00*256)},
{300, (int)(1.00*16), (int)(1.00*256)},
{311, (int)(1.00*16), (int)(1.00*256)},
{322, (int)(1.00*16), (int)(1.00*256)},
{333, (int)(1.00*16), (int)(1.00*256)},
{345, (int)(1.00*16), (int)(1.00*256)},
{357, (int)(1.00*16), (int)(1.00*256)},
{357, (int)(1.0625*16), (int)(1.00*256)},
{357, (int)(1.0625*16), (int)(1.00*256)},
{357, (int)(1.125*16), (int)(1.00*256)},
{357, (int)(1.125*16), (int)(1.00*256)},
{357, (int)(1.1875*16), (int)(1.00*256)},
{357, (int)(1.25*16), (int)(1.00*256)},
{357, (int)(1.25*16), (int)(1.00*256)},
{357, (int)(1.3125*16), (int)(1.00*256)},
{357, (int)(1.375*16), (int)(1.00*256)},
{357, (int)(1.4375*16), (int)(1.00*256)},
{357, (int)(1.4375*16), (int)(1.00*256)},
{357, (int)(1.5*16), (int)(1.00*256)},
{357, (int)(1.5625*16), (int)(1.00*256)},
{357, (int)(1.625*16), (int)(1.00*256)},
{357, (int)(1.6875*16), (int)(1.00*256)},
{357, (int)(1.75*16), (int)(1.00*256)},
{357, (int)(1.8125*16), (int)(1.00*256)},
{357, (int)(1.875*16), (int)(1.00*256)},
{357, (int)(1.9375*16), (int)(1.00*256)},
{714, (int)(1.00*16), (int)(1.00*256)},
{714, (int)(1.0625*16), (int)(1.00*256)},
{714, (int)(1.0625*16), (int)(1.00*256)},
{714, (int)(1.125*16), (int)(1.00*256)},
{714, (int)(1.125*16), (int)(1.00*256)},
{714, (int)(1.1875*16), (int)(1.00*256)},
{714, (int)(1.25*16), (int)(1.00*256)},
{714, (int)(1.25*16), (int)(1.00*256)},
{714, (int)(1.3125*16), (int)(1.00*256)},
{714, (int)(1.375*16), (int)(1.00*256)},
{714, (int)(1.4375*16), (int)(1.00*256)},
{714, (int)(1.4375*16), (int)(1.00*256)},
{714, (int)(1.5*16), (int)(1.00*256)},
{714, (int)(1.5625*16), (int)(1.00*256)},
{714, (int)(1.625*16), (int)(1.00*256)},
{714, (int)(1.6875*16), (int)(1.00*256)},
{714, (int)(1.75*16), (int)(1.00*256)},
{714, (int)(1.8125*16), (int)(1.00*256)},
{714, (int)(1.875*16), (int)(1.00*256)},
{714, (int)(1.9375*16), (int)(1.00*256)},
{1071, (int)(1.3125*16), (int)(1.00*256)},
{1071, (int)(1.375*16), (int)(1.00*256)},
{1071, (int)(1.4375*16), (int)(1.00*256)},
{1071, (int)(1.4375*16), (int)(1.00*256)},
{1071, (int)(1.5*16), (int)(1.00*256)},
{1071, (int)(1.5625*16), (int)(1.00*256)},
{1071, (int)(1.625*16), (int)(1.00*256)},
{1071, (int)(1.6875*16), (int)(1.00*256)},
{1071, (int)(1.75*16), (int)(1.00*256)},
{1071, (int)(1.8125*16), (int)(1.00*256)},
{1071, (int)(1.875*16), (int)(1.00*256)},
{1071, (int)(1.9375*16), (int)(1.00*256)},
{1071, (int)(2.0*16), (int)(1.00*256)},
{1071, (int)(2.07*16), (int)(1.00*256)},
{1071, (int)(2.14*16), (int)(1.00*256)},
{1071, (int)(2.22*16), (int)(1.00*256)},
{1071, (int)(2.30*16), (int)(1.00*256)},
{1071, (int)(2.38*16), (int)(1.00*256)},
{1071, (int)(2.46*16), (int)(1.00*256)},
{1071, (int)(2.55*16), (int)(1.00*256)},
{1071, (int)(2.64*16), (int)(1.00*256)},
{1071, (int)(2.73*16), (int)(1.00*256)},
{1071, (int)(2.83*16), (int)(1.00*256)},
{1071, (int)(2.93*16), (int)(1.00*256)},
{1071, (int)(3.03*16), (int)(1.00*256)},
{1071, (int)(3.14*16), (int)(1.00*256)},
{1071, (int)(3.25*16), (int)(1.00*256)},
{1071, (int)(3.36*16), (int)(1.00*256)},
{1071, (int)(3.48*16), (int)(1.00*256)},
{1071, (int)(3.61*16), (int)(1.00*256)},
{1071, (int)(3.73*16), (int)(1.00*256)},
{1071, (int)(3.86*16), (int)(1.00*256)},
{1071, (int)(4.00*16), (int)(1.00*256)},
{1071, (int)(4.14*16), (int)(1.00*256)},
{1071, (int)(4.29*16), (int)(1.00*256)},
{1071, (int)(4.44*16), (int)(1.00*256)},
{1071, (int)(4.59*16), (int)(1.00*256)},
{1071, (int)(4.76*16), (int)(1.00*256)},
{1071, (int)(4.92*16), (int)(1.00*256)},
{1071, (int)(5.10*16), (int)(1.00*256)},
{1071, (int)(5.28*16), (int)(1.00*256)},
{1071, (int)(5.46*16), (int)(1.00*256)},
{1071, (int)(5.66*16), (int)(1.00*256)},
{1071, (int)(5.86*16), (int)(1.00*256)},
{1071, (int)(6.06*16), (int)(1.00*256)},
{1071, (int)(6.28*16), (int)(1.00*256)},
{1071, (int)(6.50*16), (int)(1.00*256)},
{1071, (int)(6.73*16), (int)(1.00*256)},
{1071, (int)(6.96*16), (int)(1.00*256)},
{1071, (int)(7.21*16), (int)(1.00*256)},
{1071, (int)(7.46*16), (int)(1.00*256)},
{1071, (int)(7.73*16), (int)(1.00*256)},
{1071, (int)(8.00*16), (int)(1.00*256)},
{1071, (int)(8.28*16), (int)(1.00*256)},
{1071, (int)(8.57*16), (int)(1.00*256)},
{1071, (int)(8.88*16), (int)(1.00*256)},
{1071, (int)(9.19*16), (int)(1.00*256)},
{1071, (int)(9.51*16), (int)(1.00*256)},
{1071, (int)(9.85*16), (int)(1.00*256)},
{1071, (int)(10.20*16), (int)(1.00*256)},
{1071, (int)(10.56*16), (int)(1.00*256)},
{1071, (int)(10.93*16), (int)(1.00*256)},
{1071, (int)(11.31*16), (int)(1.00*256)},
{1071, (int)(11.71*16), (int)(1.00*256)},
{1071, (int)(12.13*16), (int)(1.00*256)},
{1071, (int)(12.55*16), (int)(1.00*256)},
{1071, (int)(13.00*16), (int)(1.00*256)},
{1071, (int)(13.45*16), (int)(1.00*256)},
{1071, (int)(13.93*16), (int)(1.00*256)},
{1071, (int)(14.42*16), (int)(1.00*256)},
{1071, (int)(14.93*16), (int)(1.00*256)},
{1071, (int)(15.45*16), (int)(1.00*256)},
{1071, (int)(16.00*16), (int)(1.00*256)},
{1071, (int)(16.56*16), (int)(1.00*256)},
{1071, (int)(17.15*16), (int)(1.00*256)},
{1071, (int)(17.75*16), (int)(1.00*256)},
{1071, (int)(18.83*16), (int)(1.00*256)},
{1071, (int)(19.03*16), (int)(1.00*256)},
{1071, (int)(19.70*16), (int)(1.00*256)},
{1071, (int)(20.39*16), (int)(1.00*256)}
};


////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S ov5650_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	ov5650_handle = drv_l2_sccb_open(OV5650_ID, 16, 8);
	if(ov5650_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
    ov5650_handle.devNumber = I2C_0;
	ov5650_handle.slaveAddr = OV5650_ID;
	ov5650_handle.clkRate = 400;

	//R_FUNPOS1 &= (~MASK_I2C0_TYPE);

//	gpio_init_io(IO_C1, GPIO_OUTPUT);
//	gpio_set_port_attribute(IO_C1, ATTRIBUTE_HIGH);
//	gpio_write_io(IO_C1, DATA_HIGH);

	drv_l1_i2c_init(ov5650_handle.devNumber);
#endif

    DBG_PRINT("I2C open successful\r\n");

	return STATUS_OK;
}

static void ov5650_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(ov5650_handle) {
		drv_l2_sccb_close(ov5650_handle);
		ov5650_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	ov5650_handle.slaveAddr = 0;
	ov5650_handle.clkRate = 0;
#endif
}

static INT32S ov5650_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(ov5650_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&ov5650_handle, data, 3);
#endif
}



static INT32S ov5650_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(ov5650_handle, reg, &data) >= 0) {
		*value = (INT8U)data;
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
	if(drv_l1_i2c_bus_write(&ov5650_handle, &data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&ov5650_handle, &data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}


static INT32S ov5650_sccb_write_table(regval16_t *pTable)
{
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
		if(ov5650_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}

		pTable++;
	}
	return STATUS_OK;
}




static int ov5650_cvt_agc_gain(int agc_gain)
{
	int ov5650_agc_gain, i;

	ov5650_agc_gain = 0;
	i = 5;
	do {
		if(agc_gain <= 0x1f)
			break;

		agc_gain >>= 1;
		ov5650_agc_gain <<= 1;
		ov5650_agc_gain |= 0x10;

		i--;
	} while(i != 0);

	agc_gain -= 0x10;
	if(agc_gain < 0) agc_gain = 0;
	ov5650_agc_gain += agc_gain;

	return ov5650_agc_gain;
}


static unsigned char group_id = 0;
static int ov5650_set_xfps_exposure_time(int ae_ev_step)
{
	unsigned char t1, t2;
	int idx, temp, ret;
	int current_group;
	sensor_exposure_t *si;

    si = &seInfo;
	//DBG_PRINT("%s\n\r", __FUNCTION__);
	#if 1
	si->sensor_ev_idx += ae_ev_step;
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
    //DBG_PRINT("[EV=%d, offset=%d]: time = 0x%x, analog gain =0x%x", si->sensor_ev_idx, ae_ev_step, si->time, si->analog_gain);

    current_group = group_id;
    group_id++;
    group_id = group_id & 0x03;
    ret = ov5650_sccb_write(0x3212, current_group); // Group hold enable
	if(ret < 0) return ret;

	// exposure time
	//if(si ->time != pre_sensor_time)
	{
	//	pre_sensor_time = si->time;
		temp = si->time;

		t1 = (temp & 0x0f) << 4;
        t2 = (temp >> 4) & 0x00ff;
        ret = ov5650_sccb_write(0x3502, t1);
        if(ret < 0) return ret;
        ret = ov5650_sccb_write(0x3501, t2);
        if(ret < 0) return ret;
        t1 =  (temp >> 12) & 0x000f;
        ret = ov5650_sccb_write(0x3500, t1);
        if(ret < 0) return ret;
	}

	//if(si ->analog_gain != pre_sensor_a_gain)
	{	// gain
	//	pre_sensor_a_gain = si->analog_gain;

		temp = si->analog_gain;
		temp = ov5650_cvt_agc_gain(temp);
		//DBG_PRINT(", cvt a gain = 0x%x\r\n", temp);
		t1 = temp & 0x00ff;
        t2 = (temp >> 8) & 0x01;

        // Gain = (0x350A[0]+1) กั (0x350B[7]+1) กั(0x350B[6]+1) กั (0x350B[5]+1) กั(0x350B[4]+1) กั (0x350B[3:0]/16+1)
        ret = ov5650_sccb_write(0x350b, t1);
        if(ret < 0) return ret;
        ret = ov5650_sccb_write(0x350a, t2);
        if(ret < 0) return ret;
    }

    t1 = 0x10 | current_group;
    ret = ov5650_sccb_write(0x3212, t1); //Group latch end
	if(ret < 0) return ret;
	t1 = 0xa0 | current_group;
	ret = ov5650_sccb_write(0x3212, t1); // Group Latch Launch
	if(ret < 0) return ret;

	#endif

	return 0;
}



static void ov5650_set_ae(int ev_step)
{
    ov5650_set_xfps_exposure_time(ev_step);
}


void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = (INT32U)ov5650_set_ae;
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
static void mipi_ov5650_handle(INT32U event)
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


void ov5650_seinfo_init(void)
{
    seInfo.sensor_ev_idx = OV5650_30FPS_60HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= OV5650_30FPS_60HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= OV5650_30FPS_60HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = OV5650_30FPS_60HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = OV5650_30FPS_60HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)ov5650_30fps_exp_time_gain_60Hz;

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
void ov5650_cdsp_mipi_init(void)
{
    int ret;

	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

    ov5650_seinfo_init();


	// cdsp init
	drv_l2_cdsp_open();

	// set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk
	//R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	//R_FUNPOS1 |= ((1<<24)|(1<<21)|(1<<3)|(1<<2));//0x02400042;


	// mclk output
	//drv_l2_sensor_set_mclkout(ov5650_cdsp_mipi_ops.info[0].mclk);
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

    DBG_PRINT("Sensor OV5650 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void ov5650_cdsp_mipi_uninit(void)
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
	ov5650_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void ov5650_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
    INT16U R0x30F0, R0x3072, R0x300E;
    int ret;
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);
	drv_l2_sensor_set_mclkout(MCLK_24M);


    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 2);

	drv_l2_sensor_set_mclkout(ov5650_cdsp_mipi_ops.info[index].mclk);
    //osDelay(30);

    // set cdsp format
	format.image_source = C_CDSP_MIPI;
	format.input_format =  ov5650_cdsp_mipi_ops.info[index].input_format;
	format.output_format = ov5650_cdsp_mipi_ops.info[index].output_format;
	target_w = ov5650_cdsp_mipi_ops.info[index].target_w;
	target_h = ov5650_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = ov5650_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = ov5650_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = ov5650_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = ov5650_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = ov5650_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = ov5650_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = ov5650_cdsp_mipi_ops.info[index].vsync_active;

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
    ov5650_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
    ov5650_mipi_cfg.data_type = MIPI_RAW10;
    ov5650_mipi_cfg.h_back_porch = 4;

	ov5650_mipi_cfg.h_size = format.hpixel;
	ov5650_mipi_cfg.v_size = format.vline+format.voffset ;
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      drv_l1_mipi_set_parameter(MIPI_1, &ov5650_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      drv_l1_mipi_set_parameter(MIPI_0, &ov5650_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
    #endif



    // reguest sccb
	ov5650_sccb_open();

	// init sensor
	ov5650_sccb_write_table((regval16_t *)ov5650_Mipi_Raw10_1080P_30fps);

    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    ov5650_set_xfps_exposure_time(0);

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
static void ov5650_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();

	ov5650_sccb_close();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* ov5650_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&ov5650_cdsp_mipi_ops.info[index];
	}
}


/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t ov5650_cdsp_mipi_ops =
{
	SENSOR_OV5650_CDSP_MIPI_NAME,		/* sensor name */
	ov5650_cdsp_mipi_init,
	ov5650_cdsp_mipi_uninit,
	ov5650_cdsp_mipi_stream_on,
	ov5650_cdsp_mipi_stream_off,
	ov5650_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SGBRG10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			OV5650_WIDTH,				/* sensor width */
			OV5650_HEIGHT, 				/* sensor height */
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
			V4L2_PIX_FMT_SGRBG10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			OV5650_OUT_WIDTH,			/* target width */
			OV5650_OUT_HEIGHT, 		/* target height */
			OV5650_WIDTH,				/* sensor width */
			OV5650_HEIGHT, 				/* sensor height */
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
			OV5650_WIDTH,				/* sensor width */
			OV5650_HEIGHT, 				/* sensor height */
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
			OV5650_OUT_WIDTH,			/* target width */
			OV5650_OUT_HEIGHT, 			/* target height */
			OV5650_WIDTH,				/* sensor width */
			OV5650_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};

#endif //(defined _SENSOR_OV5650_CDSP_MIPI) && (_SENSOR_OV5650_CDSP_MIPI == 1)
