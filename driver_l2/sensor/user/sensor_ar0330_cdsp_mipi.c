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
#if 0
#include "driver_l1.h"
#include "drv_l1_power.h"
#include "drv_l1_interrupt.h"
#include "drv_l1_timer.h"
#include "drv_l1_i2c.h"
#include "drv_l1_csi.h"
#include "drv_l1_mipi.h"
#include "drv_l1_cdsp.h"
#include "drv_l1_front.h"
#include "drv_l2_cdsp.h"
#include "drv_l2_cdsp_config.h"
#include "drv_l2_sensor.h"
#include "drv_l2_sccb.h"
#else
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
#endif

#if (defined _SENSOR_AR0330_CDSP_MIPI) && (_SENSOR_AR0330_CDSP_MIPI == 1)
#include "drv_l2_user_calibration.h"
#include "drv_l2_user_preference.h"

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_TEST                0
#define COLOR_BAR_EN                    0
#define MIPI_ISR_TEST                   1
#define MIPI_LANE_NO			        2	        // 1 or 2 lane
#define MIPI_DEV_NO                     0           //0:MIPI_0 or 1:MIPI_1

#define	AR0330_ID						0x20
#define AR0330_WIDTH				    2304
#define AR0330_OUT_WIDTH				2304
#define AR0330_HEIGHT				    1296
#define AR0330_OUT_HEIGHT				1296

// sccb interface
#define SCCB_GPIO 		0
#define SCCB_HW_I2C 	1
#define SCCB_MODE 	  SCCB_GPIO//SCCB_HW_I2C//SCCB_GPIO

#define DBG_PRINT	print_string

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

// 30fps
#define AR0330_30FPS_50HZ_INIT_EV_IDX 			135
#define AR0330_30FPS_50HZ_DAY_EV_IDX 			114
#define AR0330_30FPS_50HZ_NIGHT_EV_IDX		174
#define AR0330_30FPS_50HZ_EXP_TIME_TOTAL		224
#define AR0330_30FPS_50HZ_MAX_EV_IDX			(AR0330_30FPS_50HZ_EXP_TIME_TOTAL - 27) //15)


#define AR0330_30FPS_60HZ_INIT_EV_IDX 			135
#define AR0330_30FPS_60HZ_DAY_EV_IDX 			114
#define AR0330_30FPS_60HZ_NIGHT_EV_IDX		174
#define AR0330_30FPS_60HZ_EXP_TIME_TOTAL		226
#define AR0330_30FPS_60HZ_MAX_EV_IDX			(AR0330_30FPS_60HZ_EXP_TIME_TOTAL - 27) //15)


#define AR0330_MIN_D_GAIN						(1.65)



/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct regval16_s
{
	INT16U reg_num;
	INT16U value;
} regval16_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static int ob_cnt = 0;
static int *p_expTime_table;
static sensor_exposure_t 	seInfo;
static int pre_sensor_a_gain, pre_sensor_time, pre_sensor_d_gain;


#if SCCB_MODE == SCCB_GPIO
/*
	sccb_config_t ar0330_cdsp_mipi_sccb_config = {
		FRONT_SENSOR_GPIO_SCL,		//scl_port
		FRONT_SENSOR_GPIO_DRIVING,	//scl_drv
		FRONT_SENSOR_GPIO_SDA,		//sda_port
		FRONT_SENSOR_GPIO_DRIVING,	//sda_drv
		0,							//pwdn_port
		0,							//pwdn_drv
		0,							//have_pwdn
		16,							//RegBits
		16,							//DataBits
		AR0330_ID,						//slaveAddr
		0x20000,					//timeout
		100							//clock_rate
	};
*/
	static void *ar0330_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t ar0330_handle;
#endif

static mipi_config_t ar0330_mipi_cfg =
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

	AR0330_WIDTH,		/* width, 0~0xFFFF */
	AR0330_HEIGHT,		/* height, 0~0xFFFF */
	4, 					/* back porch, 0~0xF */
	4,					/* front porch, 0~0xF */
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	0,				/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	190,//250,				/* data mask, unit is ns */
	MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};



const regval16_t AR0330_Reset_Cmd[] =
{
	{0x301A, 0x0059}, // SW Reset

	{-2,-2},
	{0x3052, 0xA114}, // fix low temp OTPM wrong issue
	{0x304A, 0x0070}, // fix low temp OTPM wrong issue
	{-2,-2},
	{0x31AE, 0x0202}, //SERIAL_FORMAT = 513
	{0x301A, 0x0058}, // Disable Streaming
	{-3,-3},

	{0x3064, 0x1802}, // Disable Embedded Data
	{0x3078,	0x0001},
	{0x31E0,	0x0200},
	//{0x306E		N/A
	{0x3046,	0x4038}, // for DBG_PRINT
	{0x3048,	0x8480}, // for DBG_PRINT

	{0, 0} // End
};


const regval16_t AR0330_OTPM_V1[] =
{
	{0x30BA, 0x2C},
	{0x30FE, 0x0080},
	{0x31E0, 0x0003},
	{0x3ECE, 0xFF},
	{0x3ED0, 0xE4F6},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x8822},
	{0x3EDC, 0x2222},
	{0x3EDE, 0x22C0},
	{0x3EE0, 0x1500},
	{0x3EE6, 0x0080},
	{0x3EE8, 0x2027},
	{0x3EEA, 0x001D},
	{0x3F06, 0x046A},
	{0x305E, 0x00A0},
	{0, 0} // End
};

const regval16_t AR0330_OTPM_V2[] =
{
	{0x30BA, 0x2C},
	{0x30FE, 0x0080},
	{0x31E0, 0x0003},
	{0x3ECE, 0xFF},
	{0x3ED0, 0xE4F6},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x889B},
	{0x3EDC, 0x8863},
	{0x3EDE, 0xAA04},
	{0x3EE0, 0x15F0},
	{0x3EE6, 0x008C},
	{0x3EE8, 0x2024},
	{0x3EEA, 0xFF1F},
	{0x3F06, 0x046A},
	{0x305E, 0x00A0},
	{0, 0} // End
};

const regval16_t AR0330_OTPM_V3[] =
{
	{0x31E0, 0x0003},
	{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},
	{0, 0} // End
};

const regval16_t AR0330_OTPM_V4[] =
{
	{0x31E0, 0x0003},
	{0x3ED2, 0x0146},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},
	{0, 0} // End
};


const regval16_t AR0330_OTPM_V5[] =
{
	{0x3ED2, 0x0146},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},
	{0, 0} // End
};


const regval16_t AR0330_SequencerA[] =
{
	{0x3088, 0x8000},
	{0x3086, 0x4540},
	{0x3086, 0x6134},
	{0x3086, 0x4A31},
	{0x3086, 0x4342},
	{0x3086, 0x4560},
	{0x3086, 0x2714},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DEA},
	{0x3086, 0x2704},
	{0x3086, 0x3D10},
	{0x3086, 0x2705},
	{0x3086, 0x3D10},
	{0x3086, 0x2715},
	{0x3086, 0x3527},
	{0x3086, 0x053D},
	{0x3086, 0x1045},
	{0x3086, 0x4027},
	{0x3086, 0x0427},
	{0x3086, 0x143D},
	{0x3086, 0xFF3D},
	{0x3086, 0xFF3D},
	{0x3086, 0xEA62},
	{0x3086, 0x2728},
	{0x3086, 0x3627},
	{0x3086, 0x083D},
	{0x3086, 0x6444},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C2C},
	{0x3086, 0x4B01},
	{0x3086, 0x432D},
	{0x3086, 0x4643},
	{0x3086, 0x1647},
	{0x3086, 0x435F},
	{0x3086, 0x4F50},
	{0x3086, 0x2604},
	{0x3086, 0x2684},
	{0x3086, 0x2027},
	{0x3086, 0xFC53},
	{0x3086, 0x0D5C},
	{0x3086, 0x0D60},
	{0x3086, 0x5754},
	{0x3086, 0x1709},
	{0x3086, 0x5556},
	{0x3086, 0x4917},
	{0x3086, 0x145C},
	{0x3086, 0x0945},
	{0x3086, 0x0045},
	{0x3086, 0x8026},
	{0x3086, 0xA627},
	{0x3086, 0xF817},
	{0x3086, 0x0227},
	{0x3086, 0xFA5C},
	{0x3086, 0x0B5F},
	{0x3086, 0x5307},
	{0x3086, 0x5302},
	{0x3086, 0x4D28},
	{0x3086, 0x6C4C},
	{0x3086, 0x0928},
	{0x3086, 0x2C28},
	{0x3086, 0x294E},
	{0x3086, 0x1718},
	{0x3086, 0x26A2},
	{0x3086, 0x5C03},
	{0x3086, 0x1744},
	{0x3086, 0x2809},
	{0x3086, 0x27F2},
	{0x3086, 0x1714},
	{0x3086, 0x2808},
	{0x3086, 0x164D},
	{0x3086, 0x1A26},
	{0x3086, 0x8317},
	{0x3086, 0x0145},
	{0x3086, 0xA017},
	{0x3086, 0x0727},
	{0x3086, 0xF317},
	{0x3086, 0x2945},
	{0x3086, 0x8017},
	{0x3086, 0x0827},
	{0x3086, 0xF217},
	{0x3086, 0x285D},
	{0x3086, 0x27FA},
	{0x3086, 0x170E},
	{0x3086, 0x2681},
	{0x3086, 0x5300},
	{0x3086, 0x17E6},
	{0x3086, 0x5302},
	{0x3086, 0x1710},
	{0x3086, 0x2683},
	{0x3086, 0x2682},
	{0x3086, 0x4827},
	{0x3086, 0xF24D},
	{0x3086, 0x4E28},
	{0x3086, 0x094C},
	{0x3086, 0x0B17},
	{0x3086, 0x6D28},
	{0x3086, 0x0817},
	{0x3086, 0x014D},
	{0x3086, 0x1A17},
	{0x3086, 0x0126},
	{0x3086, 0x035C},
	{0x3086, 0x0045},
	{0x3086, 0x4027},
	{0x3086, 0x9017},
	{0x3086, 0x2A4A},
	{0x3086, 0x0A43},
	{0x3086, 0x160B},
	{0x3086, 0x4327},
	{0x3086, 0x9445},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9517},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x0827},
	{0x3086, 0x905D},
	{0x3086, 0x2808},
	{0x3086, 0x530D},
	{0x3086, 0x2645},
	{0x3086, 0x5C01},
	{0x3086, 0x2798},
	{0x3086, 0x4B12},
	{0x3086, 0x4452},
	{0x3086, 0x5117},
	{0x3086, 0x0260},
	{0x3086, 0x184A},
	{0x3086, 0x0343},
	{0x3086, 0x1604},
	{0x3086, 0x4316},
	{0x3086, 0x5843},
	{0x3086, 0x1659},
	{0x3086, 0x4316},
	{0x3086, 0x5A43},
	{0x3086, 0x165B},
	{0x3086, 0x4327},
	{0x3086, 0x9C45},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9D17},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x1027},
	{0x3086, 0x9817},
	{0x3086, 0x2022},
	{0x3086, 0x4B12},
	{0x3086, 0x442C},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C00},

	{0, 0} // End
};





const regval16_t AR0330_SequencerB[] =
{
	{0x3088, 0x8000},
	{0x3086, 0x4A03},
	{0x3086, 0x4316},
	{0x3086, 0x0443},
	{0x3086, 0x1645},
	{0x3086, 0x4045},
	{0x3086, 0x6017},
	{0x3086, 0x2045},
	{0x3086, 0x404B},
	{0x3086, 0x1244},
	{0x3086, 0x6134},
	{0x3086, 0x4A31},
	{0x3086, 0x4342},
	{0x3086, 0x4560},
	{0x3086, 0x2714},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DFF},
	{0x3086, 0x3DEA},
	{0x3086, 0x2704},
	{0x3086, 0x3D10},
	{0x3086, 0x2705},
	{0x3086, 0x3D10},
	{0x3086, 0x2715},
	{0x3086, 0x3527},
	{0x3086, 0x053D},
	{0x3086, 0x1045},
	{0x3086, 0x4027},
	{0x3086, 0x0427},
	{0x3086, 0x143D},
	{0x3086, 0xFF3D},
	{0x3086, 0xFF3D},
	{0x3086, 0xEA62},
	{0x3086, 0x2728},
	{0x3086, 0x3627},
	{0x3086, 0x083D},
	{0x3086, 0x6444},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C2C},
	{0x3086, 0x4B01},
	{0x3086, 0x432D},
	{0x3086, 0x4643},
	{0x3086, 0x1647},
	{0x3086, 0x435F},
	{0x3086, 0x4F50},
	{0x3086, 0x2604},
	{0x3086, 0x2684},
	{0x3086, 0x2027},
	{0x3086, 0xFC53},
	{0x3086, 0x0D5C},
	{0x3086, 0x0D57},
	{0x3086, 0x5417},
	{0x3086, 0x0955},
	{0x3086, 0x5649},
	{0x3086, 0x5307},
	{0x3086, 0x5302},
	{0x3086, 0x4D28},
	{0x3086, 0x6C4C},
	{0x3086, 0x0928},
	{0x3086, 0x2C28},
	{0x3086, 0x294E},
	{0x3086, 0x5C09},
	{0x3086, 0x6045},
	{0x3086, 0x0045},
	{0x3086, 0x8026},
	{0x3086, 0xA627},
	{0x3086, 0xF817},
	{0x3086, 0x0227},
	{0x3086, 0xFA5C},
	{0x3086, 0x0B17},
	{0x3086, 0x1826},
	{0x3086, 0xA25C},
	{0x3086, 0x0317},
	{0x3086, 0x4427},
	{0x3086, 0xF25F},
	{0x3086, 0x2809},
	{0x3086, 0x1714},
	{0x3086, 0x2808},
	{0x3086, 0x1701},
	{0x3086, 0x4D1A},
	{0x3086, 0x2683},
	{0x3086, 0x1701},
	{0x3086, 0x27FA},
	{0x3086, 0x45A0},
	{0x3086, 0x1707},
	{0x3086, 0x27FB},
	{0x3086, 0x1729},
	{0x3086, 0x4580},
	{0x3086, 0x1708},
	{0x3086, 0x27FA},
	{0x3086, 0x1728},
	{0x3086, 0x5D17},
	{0x3086, 0x0E26},
	{0x3086, 0x8153},
	{0x3086, 0x0117},
	{0x3086, 0xE653},
	{0x3086, 0x0217},
	{0x3086, 0x1026},
	{0x3086, 0x8326},
	{0x3086, 0x8248},
	{0x3086, 0x4D4E},
	{0x3086, 0x2809},
	{0x3086, 0x4C0B},
	{0x3086, 0x6017},
	{0x3086, 0x2027},
	{0x3086, 0xF217},
	{0x3086, 0x535F},
	{0x3086, 0x2808},
	{0x3086, 0x164D},
	{0x3086, 0x1A17},
	{0x3086, 0x0127},
	{0x3086, 0xFA26},
	{0x3086, 0x035C},
	{0x3086, 0x0145},
	{0x3086, 0x4027},
	{0x3086, 0x9817},
	{0x3086, 0x2A4A},
	{0x3086, 0x0A43},
	{0x3086, 0x160B},
	{0x3086, 0x4327},
	{0x3086, 0x9C45},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9D17},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x0827},
	{0x3086, 0x985D},
	{0x3086, 0x2645},
	{0x3086, 0x4B17},
	{0x3086, 0x0A28},
	{0x3086, 0x0853},
	{0x3086, 0x0D52},
	{0x3086, 0x5112},
	{0x3086, 0x4460},
	{0x3086, 0x184A},
	{0x3086, 0x0343},
	{0x3086, 0x1604},
	{0x3086, 0x4316},
	{0x3086, 0x5843},
	{0x3086, 0x1659},
	{0x3086, 0x4316},
	{0x3086, 0x5A43},
	{0x3086, 0x165B},
	{0x3086, 0x4327},
	{0x3086, 0x9C45},
	{0x3086, 0x6017},
	{0x3086, 0x0727},
	{0x3086, 0x9D17},
	{0x3086, 0x2545},
	{0x3086, 0x4017},
	{0x3086, 0x1027},
	{0x3086, 0x9817},
	{0x3086, 0x2022},
	{0x3086, 0x4B12},
	{0x3086, 0x442C},
	{0x3086, 0x2C2C},
	{0x3086, 0x2C00},
	{0x3086, 0x0000},

	{0, 0} // End
};


const regval16_t AR0330_SequencerPatch1[] =
{
	{0x3088, 0x800C},
	{0x3086, 0x2045},

	{0, 0} // End
};



const regval16_t AR0330_Mipi_Raw10_30fps[] =
{
	{0x301A, 0x0018},

	{0x302A, 0x0005}, //VT_PIX_CLK_DIV = 5
	{0x302C, 0x0002}, //VT_SYS_CLK_DIV = 2
	{0x302E, 0x0001}, //PRE_PLL_CLK_DIV = 1
	{0x3030, 0x0015}, //PLL_MULTIPLIER = 32
	{0x3036, 0x000a}, //OP_PIX_CLK_DIV = 10
	{0x3038, 0x0001}, //OP_SYS_CLK_DIV = 1
	{0x31AC, 0x0A0A}, //DATA_FORMAT_BITS = 10



	//MIPI Port Timing
	{0x31B0, 0x003E}, //FRAME_PREAMBLE = 62
	{0x31B2, 0x0018}, //LINE_PREAMBLE = 24
	{0x31B4, 0x4F66}, //MIPI_TIMING_0
	{0x31B6, 0x4215}, //MIPI_TIMING_1
	{0x31B8, 0x308B}, //MIPI_TIMING_2
	{0x31BA, 0x028a}, //MIPI_TIMING_3
	{0x31BC, 0x8008}, //MIPI_TIMING_4
	{0x31BE, 0x2003},	 // MIPI CONFIG STATUS

	{0x301A, 0x0058},

	//Timing_settings

	// CA
	{0x3002, 0x0078}, //Y_ADDR_START = 120
	{0x3004, 0x0006}, //X_ADDR_START = 6
	{0x3006, 0x0587}, //Y_ADDR_END = 1415
	{0x3008, 0x0905}, //X_ADDR_END = 2309

	{0x300A, 0x0543}, //FRAME_LENGTH_LINES = 2053(30fps)
	//{0x300A, 0x0A04}, //FRAME_LENGTH_LINES = 2564 (24fps)
	{0x300C, 0x04E0}, //LINE_LENGTH_PCK = 1248
	{0x30A2, 0x0001}, //X_ODD_INC = 1
	{0x30A6, 0x0001}, //Y_ODD_INC = 1

	//CB
	{0x308C, 0x0078}, //Y_ADDR_START_CB = 120
	{0x308A, 0x0006}, //X_ADDR_START_CB = 6
	{0x3090, 0x0587}, //Y_ADDR_END_CB = 1415
	{0x308E, 0x0905}, //X_ADDR_END_CB = 2309

	{0x30AA, 0x0805}, //FRAME_LENGTH_LINES_CB = 2053
	{0x303E, 0x04E0}, //LINE_LENGTH_PCK_CB = 1248
	{0x30AE, 0x0001}, //X_ODD_INC_CB = 1
	{0x30A8, 0x0001}, //Y_ODD_INC_CB = 1


	{0x3040, 0x0000}, //READ_MODE = 0
	{0x3042, 0x0000}, //EXTRA_DELAY = 0; //1183
	{0x30BA, 0x002C}, //DIGITAL_CTRL = 44


	//{0x30CE, 0x0020}, // The maximum integration time can be limited to the frame time by setting R0x30CE[5] to 1.
	{0x30CE, 0x0000},

	// Context A
	{0x3014, 0x0000},	// FINE_INTEGRATION_TIME  		// REG=0x3014, 0			// FINE_INTEGRATION_TIME
	{0x3012, 0x0602},	// COARSE_INTEGRATION_TIME		// REG=0x3012, 1092			// Coarse_Integration_Time


	// Context B
	{0x3018, 0x0000},	// FINE_INTEGRATION_TIME  		// REG=0x3014, 0			// FINE_INTEGRATION_TIME
	{0x3016, 0x0602},	// COARSE_INTEGRATION_TIME		// REG=0x3012, 1092			// Coarse_Integration_Time



	// Analog gain
	{0x3060, 0x100},	// analog gain = 1x

	// Digital gain
	{0x305E, 0x80}, 	// Digital GLOBAL_GAIN, max = 0x7ff, default = 0x80, Context A
	{0x30C4, 0x80}, 	// Digital GLOBAL_GAIN, max = 0x7ff, default = 0x80, Context B

	//Flip & Mirror
#if FLIP_MIRROR
	{0x3040, 0xC000},
#endif
	{0x30BA, 0x002C}, 	// DIGITAL_CTRL// BITFIELD=0x30BA,0x0040,0	// Digital_Ctrl_Adc_High_Speed


	{0x30B0, 0x8000},  // use context A

	// r, g, b gain
	//CA
	{0x3056, 0x80}, // green1 gain
	{0x3058, 0x80}, // blue gain
	{0x305A, 0x80}, // red gain
	{0x305C, 0x80}, // green2 gain

	//CB
	{0x30BC, 0x80}, // green1 gain
	{0x30BE, 0x80}, // blue gain
	{0x30C0, 0x80}, // red gain
	{0x30C2, 0x80}, // green2 gain


	//Recommended Configuration
	{0x3064, 0x1802},
	{0x3078, 0x0001},
	{0x31E0, 0x0703},
	//{0x31E0, 0x0203}, // Disable on chip noise correction
	/*{0x3ED2, 0x0146},
	{0x3ED4, 0x8F6C},
	{0x3ED6, 0x66CC},
	{0x3ED8, 0x8C42},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},*/

	{0x3088, 0x80BA}, 	// SEQ_CTRL_PORT
	{0x3086, 0x0253},


	{0x301A, 0x025C}, //Enable Streaming
	{-1,-1},
	{0, 0},			  //End
};



static const int ar0330_analog_gain_table[29] =
{
	// coarse gain = 0
	(int)(1.00*256+0.5), (int)(1.03*256+0.5), (int)(1.07*256+0.5), (int)(1.10*256+0.5),
	(int)(1.14*256+0.5), (int)(1.19*256+0.5), (int)(1.23*256+0.5), (int)(1.28*256+0.5),
	(int)(1.33*256+0.5), (int)(1.39*256+0.5), (int)(1.45*256+0.5), (int)(1.52*256+0.5),
	(int)(1.60*256+0.5), (int)(1.68*256+0.5), (int)(1.78*256+0.5), (int)(1.88*256+0.5),

	// coarse gain = 1
	(int)(2.00*256+0.5), (int)(2.13*256+0.5), (int)(2.29*256+0.5), (int)(2.46*256+0.5),
	(int)(2.67*256+0.5), (int)(2.91*256+0.5), (int)(3.20*256+0.5), (int)(3.56*256+0.5),

	// coarse gain = 2
	(int)(4.00*256+0.5), (int)(4.57*256+0.5), (int)(5.33*256+0.5), (int)(6.40*256+0.5),

	// coarse gain = 3
	(int)(8.00*256+0.5)
};



static const  int ar0330_30fps_exp_time_gain_50Hz[AR0330_30FPS_50HZ_EXP_TIME_TOTAL][3] =
{
	{8, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{9, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{9, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{9, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{10, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{10, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{10, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{11, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{11, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{11, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{12, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{12, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{13, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{13, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{14, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{14, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{15, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{15, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{16, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{16, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{17, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{17, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{18, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{19, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{19, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{20, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{21, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{21, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{22, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{23, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{24, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{25, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{25, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{26, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{27, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{28, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{29, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{30, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{31, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{32, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{34, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{35, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{36, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{37, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{39, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{40, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{41, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{43, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{44, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{46, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{48, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{49, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{51, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{53, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{55, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{57, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{59, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{61, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{63, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{65, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{67, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{70, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{72, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{75, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{77, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{80, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{83, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{86, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{89, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{92, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{95, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{98, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{102, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{106, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{109, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{113, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{117, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{121, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{126, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{130, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{135, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{139, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{144, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{149, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{155, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{160, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{166, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{171, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{178, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{184, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{190, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{197, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{204, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{211, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{219, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{226, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{234, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{243, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{251, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{260, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{269, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{279, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{288, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{299, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{309, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{320, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{331, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{343, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{355, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{368, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{381, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.03*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.07*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.10*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.14*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.19*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.23*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.28*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.33*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.33*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.39*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.45*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.52*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.52*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.60*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.68*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.68*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.78*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.88*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{394, (int)(1.88*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.00*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.03*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.07*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.10*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.14*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.19*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.23*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.28*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.33*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.33*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.39*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.45*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.52*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.52*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.60*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.68*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.68*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.78*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{788, (int)(1.88*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.28*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.33*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.33*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.39*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.45*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.52*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.52*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.60*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.68*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.68*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.78*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.88*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(1.88*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.0*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.0*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.13*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.13*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.29*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.29*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.46*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.46*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.67*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.67*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.67*256), (int)(1.07*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.91*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.91*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(2.91*256), (int)(1.07*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(3.20*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(3.20*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(3.20*256), (int)(1.07*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(3.56*256), (int)(1.00*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(3.56*256), (int)(1.04*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(3.56*256), (int)(1.07*AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.00 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.04 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.07 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.11 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.15 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.19 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.23 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.27 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.32 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.37 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.41 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.46 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.52 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.57 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.62 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.68 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.74 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.80 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.87 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(1.93 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.00 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.07 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.14 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.22 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.30 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.38 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.46 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.55 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.64 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.73 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.83 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(2.93 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.03 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.14 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.25 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.36 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.48 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.61 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.73 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(3.86 *AR0330_MIN_D_GAIN*32)},
	{1182, (int)(4.0*256), (int)(4.00 *AR0330_MIN_D_GAIN*32)}


};


////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////

static INT32S ar0330_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	ar0330_handle = drv_l2_sccb_open(AR0330_ID, 16, 16);
	if(ar0330_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C

#define	B_I2C0_TYPE                           15
#define	B_I2C0_IOC0IOC1_EN              (0x0 << B_I2C0_TYPE)
#define	B_I2C0_IOD4IOD5_EN              (0x1 << B_I2C0_TYPE)
#define MASK_I2C0_TYPE                	(0x1 << B_I2C0_TYPE)

    ar0330_handle.devNumber = I2C_0;
	ar0330_handle.slaveAddr = AR0330_ID;
	ar0330_handle.clkRate = 200;

	R_FUNPOS1 &= (~MASK_I2C0_TYPE);

//	gpio_init_io(IO_C1, GPIO_OUTPUT);
//	gpio_set_port_attribute(IO_C1, ATTRIBUTE_HIGH);
//	gpio_write_io(IO_C1, DATA_HIGH);

	drv_l1_i2c_init(ar0330_handle.devNumber);
#endif
	return STATUS_OK;
}

static void ar0330_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(ar0330_handle) {
		drv_l2_sccb_close(ar0330_handle);
		ar0330_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	ar0330_handle.slaveAddr = 0;
	ar0330_handle.clkRate = 0;
#endif
}

static INT32S ar0330_sccb_write(INT16U reg, INT16U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(ar0330_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[4];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = (value >> 8) & 0xff;
	data[3] = value & 0xff;
	return drv_l1_i2c_bus_write(&ar0330_handle, data, 4);
#endif
}



static INT32S ar0330_sccb_read(INT16U reg, INT16U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(ar0330_handle, reg, &data) >= 0) {
		*value = (INT16U)data;
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
	if(drv_l1_i2c_bus_write(&ar0330_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&ar0330_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = (INT16U)data[1] | ((INT16U)data[0] << 8);
#endif
	return STATUS_OK;
}


static INT32S ar0330_sccb_write_table(regval16_t *pTable)
{

   /* ar0330_sccb_write(pTable->reg_num, 0x1234);
    while(1)
    {
        unsigned short tt;
        ar0330_sccb_read(pTable->reg_num, &tt);
		DBG_PRINT("sensor i2c reg[0x%04x] = 0, read = 0x%04x\r\n", pTable->reg_num,  tt);
    }*/

	while(1)
	{
        unsigned short tt;

		if (pTable->reg_num == 0 && pTable->value == 0) {
			break;
		} else if (pTable->reg_num == 0xffff && pTable->value == 0xffff) {
			osDelay(1);
			pTable++;
			continue;
		}
		else if (pTable->reg_num == 0xfffe && pTable->value == 0xfffe) {
			osDelay(25);//comi
			pTable++;
			continue;
		}
		else if (pTable->reg_num == 0xfffd && pTable->value == 0xfffd) {
			osDelay(35);//comi
			pTable++;
			continue;
		}

		//DBG_PRINT("0x%04x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(ar0330_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}

		//ar0330_sccb_read(pTable->reg_num, &tt);
		//DBG_PRINT("sensor i2c reg[0x%04x] = 0x%04x, read = 0x%04x\r\n", pTable->reg_num, pTable->value, tt);

		pTable++;
	}
	return STATUS_OK;
}


static int ar0330_cvt_analog_gain(int analog_gain)
{
	int i;
	int coarse_gain, fine_gain;
	int *p_ar0330_analog_gain_table = (int *)ar0330_analog_gain_table;

	for( i = 0 ; i < 29 ; i++)
	{
		if(analog_gain >= p_ar0330_analog_gain_table[i] && analog_gain < p_ar0330_analog_gain_table[i+1])
			break;
	}

	if( i < 16 )
	{
		coarse_gain = 0;
		fine_gain = i;
	}
	else if(i < (16+8))
	{
		coarse_gain = 1;
		fine_gain = (i - 16) << 1;
	}
	else if(i < (16 + 8 + 4))
	{
		coarse_gain = 2;
		fine_gain = (i - 16 - 8) << 2;
	}
	else
	{
		coarse_gain = 3;
		fine_gain = 0;
	}


	return ((coarse_gain << 4) | fine_gain);
}

static int ar0330_get_real_analog_gain(int analog_gain, int context)
{
	int real_analog_gain;
	int coarse_gain, fine_gain;

	if(context == 0)
	{	// context A
		coarse_gain = (analog_gain >> 4) & 0x3;
		fine_gain = (analog_gain & 0xf);
	}
	else
	{	// context B
		coarse_gain = (analog_gain >> 12) & 0x3;
		fine_gain = (analog_gain >> 8) & 0xf;
	}

	if(coarse_gain == 0)
	{
		real_analog_gain = ar0330_analog_gain_table[fine_gain];
	}
	else	if(coarse_gain == 1)
	{
		real_analog_gain = ar0330_analog_gain_table[16 + (fine_gain >> 1)];
	}
	else	if(coarse_gain == 2)
	{
		real_analog_gain = ar0330_analog_gain_table[16 + 8 + (fine_gain >> 2)];
	}
	else	if(coarse_gain == 3)
	{
		real_analog_gain = ar0330_analog_gain_table[16+8+4];
	}
	else
	{
		DBG_PRINT("AR0330 analog gain Err!\r\n");
	}


	return real_analog_gain;
}



int ar0330_set_xfps_exposure_time(sensor_exposure_t *si)
{
	int ret = 0;
	unsigned short tmp;
	int analog_gain;
	int idx;
	int context;

#if 1
	si->sensor_ev_idx += si->ae_ev_step;
	if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
	if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;

#if 0//RAW_MODE
	//idx = (g_raw_mode_idx < 0) ? si->sensor_ev_idx * 3 : g_raw_mode_idx * 3;
#else
	idx = si->sensor_ev_idx * 3;
#endif
	si ->time = p_expTime_table[idx];
	si ->analog_gain = p_expTime_table[idx+1];
	//si ->digital_gain = p_expTime_table[idx+2];

	//DBG_PRINT("new: sensor_ev_idx = %d, ae_ev_idx = %d\n", si->sensor_ev_idx, si->ae_ev_idx);
	//DBG_PRINT("evidx[%d]: time = 0x%x, a_gain = 0x%x, d_gain = 0x%x\n", si->sensor_ev_idx, si->time, si->analog_gain, si->digital_gain);


/*
	if(sensor_fps == V4L2_TC_TYPE_25FPS_30FPS)
	{
		int fps;

		if(si ->time >= sensor_switch_time)	fps = 25;
		else	fps = 30;

		if(current_frame_rate != fps)
		{
			ret = ar0330_change_fps(fps);
			if(ret < 0) return ret;

			DBG_PRINT("switch to %dfps\r\n", fps);

			current_frame_rate = fps;
		}

		//ret = AR0330_write(0x30AA, tmp); // CB
		//if(ret < 0) return ret;
	}
*/
	analog_gain = ar0330_cvt_analog_gain(si->analog_gain);


	// Context Switch
	ret = ar0330_sccb_read( 0x30B0, &tmp );
	if(ret < 0) return ret;

	// set exposure time
	if(pre_sensor_time != si->time)
	{
		ret = ar0330_sccb_write(0x3012 , si->time );
		if(ret < 0) return ret;

		pre_sensor_time = si->time;
	}

	// set Analog gain
	if(pre_sensor_a_gain != analog_gain)
	{
		ret = ar0330_sccb_write(0x3060, analog_gain );
		if(ret < 0) return ret;

		pre_sensor_a_gain = analog_gain;
	}


	// set Digital gain
/*	if(pre_sensor_d_gain != si->digital_gain)
	{
		ret = AR0330_write(0x305E, si->digital_gain);
		if(ret < 0) return ret;

		pre_sensor_d_gain = si->digital_gain;
	}*/

	if((tmp | 0x2000) != 0)
	{
		tmp = tmp & (~0x2000);
		ret = ar0330_sccb_write(0x30B0, tmp);
	}
#endif

	return ret;
}



static int ar0330_reset_ob(void)
{
	unsigned short tmp;
	int ret = 0;
	int thr;

	thr = 20;
	if(seInfo.sensor_ev_idx > seInfo.night_ev_idx) 	thr = 8;

	ob_cnt++;
	if(ob_cnt >= thr)
	{
		// ob calibration
		ret = ar0330_sccb_read( 0x3180, &tmp );
		if(ret < 0) return ret;

		tmp |= 0x2000;
		ret = ar0330_sccb_write(0x3180, tmp);
		if(ret < 0 ) return ret;

		ob_cnt = 0;
	}

	return ret;
}



void test_delay (INT16U i)
{
	INT16U j;

	for (j=0;j<(i<<4);j++)
		i=i;
}

#ifdef MIPI_ISR_TEST
static void mipi_ar0330_handle(INT32U event)
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
static void ar0330_set_ae(int ev_step)
{
	//DBG_PRINT("AR0330 %s!\r\n", __func__);

    seInfo.ae_ev_step = ev_step;
    ar0330_set_xfps_exposure_time(&seInfo);
}

//void sensor_register_ae_ctrl(INT32U *handle) __attribute__((weak));
void sensor_register_ae_ctrl(INT32U *handle)
{
	//DBG_PRINT("AR0330 %s!\r\n", __func__);

	*handle = (INT32U) ar0330_set_ae;
}

//void sensor_get_ae_info(sensor_exposure_t *si) __attribute__((weak));
void sensor_get_ae_info(sensor_exposure_t *si)
{
	//DBG_PRINT("AR0330 %s!\r\n", __func__);

    memcpy(si, &seInfo, sizeof(sensor_exposure_t));
}

void sensor_set_max_lum(int max_lum)
{
    seInfo.max_ev_idx = seInfo.total_ev_idx - (64 - max_lum);
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ar0330_seinfo_init(void)
{
    seInfo.sensor_ev_idx = AR0330_30FPS_50HZ_INIT_EV_IDX;
	seInfo.ae_ev_step = 0;
	seInfo.daylight_ev_idx= AR0330_30FPS_50HZ_DAY_EV_IDX;
	seInfo.night_ev_idx= AR0330_30FPS_50HZ_NIGHT_EV_IDX;
	seInfo.max_ev_idx = AR0330_30FPS_50HZ_MAX_EV_IDX;
	seInfo.total_ev_idx = AR0330_30FPS_50HZ_EXP_TIME_TOTAL;

	p_expTime_table = (int *)ar0330_30fps_exp_time_gain_50Hz;

	//pre_sensor_time = 1;
	//pre_sensor_a_gain = 0xff;

	seInfo.time = 1;
	seInfo.analog_gain = 0xff;
	seInfo.digital_gain = 0x20;

}

/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void ar0330_cdsp_mipi_init(void)
{
    int ret;

	unsigned short R0x300E, R0x30F0, R0x3072;

	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);


    ar0330_seinfo_init();

	p_expTime_table = (int *)ar0330_30fps_exp_time_gain_50Hz;

	// cdsp init
	drv_l2_cdsp_open();

	// set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk
	R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<3)|(1<<2)|(1<<1));//0x02400042;
	//R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<1));//0x02400042;

	// mclk output
	//drv_l2_sensor_set_mclkout(ar0330_cdsp_mipi_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_24M);
    osDelay(30);

	// Reset
	gpio_set_port_attribute(IO_A14, ATTRIBUTE_HIGH);
	gpio_init_io(IO_A14, GPIO_OUTPUT);
	gpio_write_io(IO_A14, DATA_HIGH);
    osDelay(10);
    gpio_write_io(IO_A14, DATA_LOW);
    osDelay(10);
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

    DBG_PRINT("Sensor AR0330 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void ar0330_cdsp_mipi_uninit(void)
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
	ar0330_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void ar0330_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
//static void ar0330_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
    INT16U R0x30F0, R0x3072, R0x300E;
    int ret;
	INT16U target_w, target_h, sensor_w, sensor_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);
	drv_l2_sensor_set_mclkout(MCLK_24M);

    switch(index)
	{
	case 0:
		break;

	case 1:
		break;

	case 2:
		break;

	case 3:
		break;

	default:
		while(1);
	}
    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 2);
    //osDelay(200);

#if 1   // change mclk
	drv_l2_sensor_set_mclkout(ar0330_cdsp_mipi_ops.info[index].mclk);
    //drv_l2_sensor_set_mclkout(MCLK_12M);
#else
    drv_l2_sensor_set_mclkout(MCLK_24M);    //OLDWU
#endif
    osDelay(300);
    // set cdsp format
	format.image_source = C_CDSP_MIPI;
	format.input_format =  ar0330_cdsp_mipi_ops.info[index].input_format;
	format.output_format = ar0330_cdsp_mipi_ops.info[index].output_format;
	target_w = ar0330_cdsp_mipi_ops.info[index].target_w;
	target_h = ar0330_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = sensor_w = ar0330_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = sensor_h = ar0330_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = ar0330_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = ar0330_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = ar0330_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = ar0330_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = ar0330_cdsp_mipi_ops.info[index].vsync_active;


    if(drv_l2_cdsp_set_fmt(&format) < 0)
    {
		DBG_PRINT("cdsp set fmt err!!!\r\n");
	}

	// set scale down
	if((format.hpixel > target_w) || (format.vline > target_h)) {
		drv_l2_cdsp_set_yuv_scale(target_w, target_h);
	}

	// cdsp start
	//drv_l2_cdsp_enable(target_w, target_h, myFav);
	//drv_l2_cdsp_stream_on(DISABLE, bufA, bufB);
	    // cdsp start
	drv_l2_CdspTableRegister((gpCisCali_t*)&g_cali);
	drv_l2_cdsp_stream_on(ENABLE, bufA, bufB);
	drv_l2_cdsp_enable(&g_FavTable, sensor_w, sensor_h, target_w, target_h);

	// set mipi format.
#if 1
    //switch(format.input_format)
    switch(5)
	{
	case V4L2_PIX_FMT_YUYV: //(LSB -> MSB), YUYV --> UYVY
		ar0330_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ar0330_mipi_cfg.data_type = MIPI_YUV422;
		ar0330_mipi_cfg.h_back_porch = 1;
		break;

	case V4L2_PIX_FMT_YVYU:	//(LSB -> MSB), YVYU --> UYVY
		ar0330_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ar0330_mipi_cfg.data_type = MIPI_YUV422;
		ar0330_mipi_cfg.h_back_porch = 3;
		break;

	case V4L2_PIX_FMT_UYVY: //(LSB -> MSB), UYVY --> UYVY
		ar0330_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ar0330_mipi_cfg.data_type = MIPI_YUV422;
		ar0330_mipi_cfg.h_back_porch = 4;
		break;

	case V4L2_PIX_FMT_VYUY: //(LSB -> MSB), VYUY --> UYVY
		ar0330_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ar0330_mipi_cfg.data_type = MIPI_YUV422;
		ar0330_mipi_cfg.h_back_porch = 2;
		break;

	default:
    	/* #if (RD_SIM == 1)
        ar0330_mipi_cfg.blanking_line_en = 1;
        ar0330_mipi_cfg.byte_clk_edge = 0;
        ar0330_mipi_cfg.check_hs_seq = 0;
        ar0330_mipi_cfg.data_from_mmr = 0;
        ar0330_mipi_cfg.data_mask_time = 250;
        ar0330_mipi_cfg.data_type = 3;
        ar0330_mipi_cfg.data_type_to_cdsp = 1;
        ar0330_mipi_cfg.ecc_check_en = 1;
        ar0330_mipi_cfg.ecc_order = 3;
        ar0330_mipi_cfg.h_front_porch = 4;
        ar0330_mipi_cfg.low_power_en = 0;
        ar0330_mipi_cfg.mipi_lane_no = 1;
        ar0330_mipi_cfg.pixel_clk_sel = 1;
        #endif
        */
        ar0330_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		ar0330_mipi_cfg.data_type = MIPI_RAW10;
		ar0330_mipi_cfg.h_back_porch = 4;
	}

	ar0330_mipi_cfg.h_size = format.hpixel;
	ar0330_mipi_cfg.v_size = format.vline;
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      drv_l1_mipi_set_parameter(MIPI_1, &ar0330_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_ar0330_handle);
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      drv_l1_mipi_set_parameter(MIPI_0, &ar0330_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_ar0330_handle);
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
    #endif

    //drv_l2_cdsp_stream_on(DISABLE, bufA, bufB);


    // reguest sccb
	ar0330_sccb_open();

	// reset sensor
	ar0330_sccb_write_table((regval16_t *)AR0330_Reset_Cmd);

	ret = ar0330_sccb_read( 0x30f0, &R0x30F0 );
	//if(ret < 0) return ret;
	ret = ar0330_sccb_read( 0x3072, &R0x3072 );
	//if(ret < 0) return ret;
	//ret = ar0330_sccb_read( 0x300E, &R0x300E );
	//if(ret < 0) return ret;

	//DBG_PRINT("R0x300E = 0x%x, R0x30F0 = 0x%x, R0x3072 = 0x%x\r\n", R0x300E, R0x30F0, R0x3072);
	if(/*R0x300E == 0x10 && */R0x30F0 == 0x1200 && R0x3072 == 0x00)
	{	// rev 1
		ar0330_sccb_write_table((regval16_t *)AR0330_OTPM_V1);
		ar0330_sccb_write_table((regval16_t *)AR0330_SequencerA);
		DBG_PRINT("OTPM_V1\r\n");
	}
	else if(R0x30F0 == 0x1208 && R0x3072 == 0x00)
	{	// rev 2
		ar0330_sccb_write_table((regval16_t *)AR0330_OTPM_V2);
		ar0330_sccb_write_table((regval16_t *)AR0330_SequencerB);
		DBG_PRINT("OTPM_V2\r\n");
	}
	else if(/*R0x300E == 0x20 && */R0x30F0 == 0x1208 && R0x3072 == 0x06)
	{	// rev 2.1
		ar0330_sccb_write_table((regval16_t *)AR0330_OTPM_V3);
		ar0330_sccb_write_table((regval16_t *)AR0330_SequencerPatch1);
		DBG_PRINT("OTPM_V3\r\n");
	}
	else if(/*R0x300E == 0x20 && */R0x30F0 == 0x1208 && R0x3072 == 0x07)
	{	// rev 2.1
		ar0330_sccb_write_table((regval16_t *)AR0330_OTPM_V4);
		DBG_PRINT("OTPM_V4\r\n");
	}
	else if(/*R0x300E == 0x20 && */R0x30F0 == 0x1208 && R0x3072 == 0x08)
	{	// rev 2.1
		ar0330_sccb_write_table((regval16_t *)AR0330_OTPM_V5);
		DBG_PRINT("OTPM_V5\r\n");
	}
	else
	{
		DBG_PRINT("OTPM unknow: R0x30F0 = 0x%x, R0x3072 = 0x%x\r\n", R0x30F0, R0x3072);
		ar0330_sccb_write_table((regval16_t *)AR0330_OTPM_V5);
		//return -1;
	}


    osDelay(200);

	// init sensor
	ar0330_sccb_write_table((regval16_t *)AR0330_Mipi_Raw10_30fps);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void ar0330_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* ar0330_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&ar0330_cdsp_mipi_ops.info[index];
	}
}


/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t ar0330_cdsp_mipi_ops =
{
	SENSOR_AR0330_CDSP_MIPI_NAME,		/* sensor name */
	ar0330_cdsp_mipi_init,
	ar0330_cdsp_mipi_uninit,
	ar0330_cdsp_mipi_stream_on,
	ar0330_cdsp_mipi_stream_off,
	ar0330_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SBGGR10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			AR0330_WIDTH,				/* sensor width */
			AR0330_HEIGHT, 				/* sensor height */
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
			V4L2_PIX_FMT_SGRBG10,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			AR0330_OUT_WIDTH,			/* target width */
			AR0330_OUT_HEIGHT, 		/* target height */
			AR0330_WIDTH,				/* sensor width */
			AR0330_HEIGHT, 				/* sensor height */
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
			V4L2_PIX_FMT_YUYV,			/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			AR0330_WIDTH,				/* sensor width */
			AR0330_HEIGHT, 				/* sensor height */
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
			AR0330_OUT_WIDTH,			/* target width */
			AR0330_OUT_HEIGHT, 			/* target height */
			AR0330_WIDTH,				/* sensor width */
			AR0330_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};

#endif //(defined _SENSOR_AR0330_CDSP_MIPI) && (_SENSOR_AR0330_CDSP_MIPI == 1)
