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
#include "drv_l2_sensor.h"
#include "drv_l2_sccb.h"
#include "drv_l2_cdsp.h"
#endif

#if (defined _SENSOR_OV3640_CDSP_MIPI) && (_SENSOR_OV3640_CDSP_MIPI == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_TEST                0
#define COLOR_BAR_EN                    0
#define MIPI_ISR_TEST                   1
#define MIPI_LANE_NO			        2	        // 1 or 2 lane
#define MIPI_DEV_NO                     0           //0:MIPI_0 or 1:MIPI_1

#define	OV3640_ID					0x78
#define OV3640_WIDTH				2048
#define OV3640_HEIGHT				1536

#define OV3640_VYUY 				0x00
#define OV3640_UYVY					0x02
#define OV3640_BGGR					0x18
#define OV3640_GBRG					0x19
#define OV3640_GRBG					0x1A
#define OV3640_RGGB					0x1B

#ifndef DISABLE
#define DISABLE     0
#endif

#ifndef ENABLE
#define ENABLE      1
#endif
/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

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
#if SCCB_MODE == SCCB_GPIO
	static void *ov3640_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t ov3640_handle;
#endif

static mipi_config_t ov3640_mipi_cfg =
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
	MIPI_RAW8,			/* data type, valid when data mode is 1*/
	MIPI_DATA_TO_CDSP,	/* data type, 1:data[7:0]+2':00 to cdsp, 0: 2'00+data[7:0] to csi */
	NULL,				/* RSD 2 */

	OV3640_WIDTH,		/* width, 0~0xFFFF */
	OV3640_HEIGHT,		/* height, 0~0xFFFF */
	4, 					/* back porch, 0~0xF */
	4,					/* front porch, 0~0xF */
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	NULL,				/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	190,//250,				/* data mask, unit is ns */
	MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};

static const regval16_t ov3640_reset_table[] =
{
	{ 0x3012, 0x80 },
	{ 0xFFFF, 0xFF },
};
#if 0
static const regval16_t ov3640_init_table[] =
{
	{ 0x304d, 0x41 },
	{ 0x3087, 0x16 },
	{ 0x30aa, 0x45 },
	{ 0x30b0, 0xff },
	{ 0x30b1, 0xff },
	{ 0x30b2, 0x10 },
	{ 0x30d7, 0x10 },

	{ 0x309e, 0x00 },
	{ 0x3602, 0x26 }, /* SOL/EOL on */
	{ 0x3603, 0x4D }, /* ecc */
	{ 0x364c, 0x04 }, /* ecc */
	{ 0x360c, 0x12 }, /* irtual channel 0 */
	{ 0x361e, 0x00 },
	{ 0x361f, 0x11 }, /* pclk_period, terry */
	{ 0x3633, 0x32 }, /* increase hs_prepare */
	{ 0x3629, 0x3c }, /* increase clk_prepare */

#if CONFIG_FPGA_TEST
	//{ 0x300e, 0x38 },
	//{ 0x300f, 0xa2 }, /*a1:1.5 , a2:2 */
	{ 0x300e, 0x37 }, //15fps
	{ 0x300f, 0xa3 },
 #else
	#if 0
	{ 0x300e, 0x39 },
	{ 0x300f, 0xa1 },
	#elif 0 //13MHz
	{ 0x300e, 0x37 }, //15fps
	{ 0x300f, 0xa3 },
	#elif 1
	{ 0x300e, 0x3c },
	{ 0x300f, 0xa3 },
	#else //835KHz
	{ 0x300e, 0x3b }, //bypass PLL
	{ 0x300f, 0xa8 },
	#endif
#endif
#if 0
	{ 0x3010, 0x80 }, /* 1 lane, high mipi spd, 81 */
#else
	{ 0x3010, 0xa0 }, /* 2 lane */
#endif
	{ 0x3011, 0x00 },
	{ 0x304c, 0x81 },

	{ 0x3018, 0x38 }, /* aec */
	{ 0x3019, 0x30 },
	{ 0x301a, 0x61 },
	{ 0x307d, 0x00 },
	{ 0x3087, 0x02 },
	{ 0x3082, 0x20 },

	{ 0x303c, 0x08 }, /* aec weight */
	{ 0x303d, 0x18 },
	{ 0x303e, 0x06 },
	{ 0x303f, 0x0c },
	{ 0x3030, 0x62 },
	{ 0x3031, 0x26 },
	{ 0x3032, 0xe6 },
	{ 0x3033, 0x6e },
	{ 0x3034, 0xea },
	{ 0x3035, 0xae },
	{ 0x3036, 0xa6 },
	{ 0x3037, 0x6a },

	{ 0x3015, 0x12 },
	{ 0x3014, 0x04 },
	{ 0x3013, 0xf7 },

	{ 0x3104, 0x02 },
	{ 0x3105, 0xfd },
	{ 0x3106, 0x00 },
	{ 0x3107, 0xff },
	{ 0x3308, 0xa5 },
	{ 0x3316, 0xff },
	{ 0x3317, 0x00 },
	{ 0x3087, 0x02 },
	{ 0x3082, 0x20 },
	{ 0x3300, 0x13 },
	{ 0x3301, 0xde },
	{ 0x3302, 0xef },

	{ 0x30b8, 0x20 },
	{ 0x30b9, 0x17 },
	{ 0x30ba, 0x04 },
	{ 0x30bb, 0x08 },

	{ 0x3100, 0x02 }, /* set raw format */
	{ 0x3304, 0x00 },
	{ 0x3400, 0x00 },
	{ 0x3404, OV3640_BGGR},
 	//{ 0x3404, OV3640_UYVY},

	{ 0x3020, 0x01 }, /* Size, 2048x1536, QXGA */
	{ 0x3021, 0x1d },
	{ 0x3022, 0x00 },
	{ 0x3023, 0x0a },
	{ 0x3024, 0x08 },
	{ 0x3025, 0x18 },
	{ 0x3026, 0x06 },
	{ 0x3027, 0x0c },

	{ 0x335f, 0x68 },
	{ 0x3360, 0x18 },
	{ 0x3361, 0x0c },
	{ 0x3362, 0x68 },
	{ 0x3363, 0x08 },
	{ 0x3364, 0x04 },
	{ 0x3403, 0x42 },

	{ 0x3088, 0x08 },
	{ 0x3089, 0x00 },
	{ 0x308a, 0x06 },
	{ 0x308b, 0x00 },

	{ 0x3507, 0x06 },
	{ 0x350a, 0x4f },
	{ 0x3600, 0xc4 },

#if COLOR_BAR_TEST
	{ 0x307B, 0x4a },   //color bar[1:0]
	{ 0x307D, 0xa0 },   //color bar[7]
	{ 0x306C, 0x00 },   //color bar[4]
	{ 0x3080, 0x11 },   //color bar[7] enable
#endif
	{ 0xffff, 0xff },
};
#else
static const regval16_t ov3640_init_table[] =
{
	{ 0x304d, 0x41 },
	{ 0x3087, 0x16 },
	{ 0x30aa, 0x45 },
	{ 0x30b0, 0xff },
	{ 0x30b1, 0xff },
	{ 0x30b2, 0x10 },
	{ 0x30d7, 0x10 },

	{ 0x309e, 0x00 },
	{ 0x3602, 0x26 }, /* SOL/EOL on */
	{ 0x3603, 0x4D }, /* ecc */
	{ 0x364c, 0x04 }, /* ecc */
	{ 0x360c, 0x12 }, /* irtual channel 0 */
	{ 0x361e, 0x00 },
	{ 0x361f, 0x11 }, /* pclk_period, terry */
	{ 0x3633, 0x32 }, /* increase hs_prepare */
	{ 0x3629, 0x3c }, /* increase clk_prepare */

#if 1//CONFIG_FPGA_EN == 0      //OLDWU
	//{ 0x300e, 0x39 }, //30fps
	//{ 0x300f, 0xa1 }, /* a1:1.5 , a2:2 */
	//{ 0x300e, 0x3a },
	//{ 0x300f, 0xa3 },
	/*ok*/
    { 0x300e, 0x3a },   //vga:19fps,plck36, 2048*1536:10fps
	{ 0x300f, 0xa2 },

#else
#if  0 //18MHz
	{ 0x300e, 0x3a },
	{ 0x300f, 0xa2 },
#elif 0 //15fps
	{ 0x300e, 0x3a },
	{ 0x300f, 0x62 },
#else
	#if 0//MIPI_LANE_NO == 1
	{ 0x300e, 0x3A }, //0x3A +0xa2 = PCLK 36MHz @ 1lane
	{ 0x300f, 0xa2 }, //0xa2 inclk 12mclk
	#elif 0//MIPI_LANE_NO == 2
	{ 0x300e, 0x39 }, //0x39 +0xa2 = PCLK 20MHz @ 2lane
	{ 0x300f, 0xa2 }, //0xa2 inclk 12mclk
	#else
	{ 0x300e, 0x3b }, //bypass PLL
	{ 0x300f, 0xa8 },
	#endif
#endif
#endif

#if MIPI_LANE_NO == 1
	{ 0x3010, 0x80 }, /* 1 lane, high mipi spd, 81 */
#elif MIPI_LANE_NO == 2
	{ 0x3010, 0xa0 }, /* 2 lane */
#endif
	{ 0x3011, 0x00 },
	{ 0x304c, 0x81 },

	{ 0x3018, 0x38 }, /* aec */
	{ 0x3019, 0x30 },
	{ 0x301a, 0x61 },
	{ 0x307d, 0x00 },
	{ 0x3087, 0x02 },
	{ 0x3082, 0x20 },

	{ 0x303c, 0x08 }, /* aec weight */
	{ 0x303d, 0x18 },
	{ 0x303e, 0x06 },
	{ 0x303f, 0x0c },
#if 0
	{ 0x3030, 0x62 },
	{ 0x3031, 0x26 },
	{ 0x3032, 0xe6 },
	{ 0x3033, 0x6e },
	{ 0x3034, 0xea },
	{ 0x3035, 0xae },
	{ 0x3036, 0xa6 },
	{ 0x3037, 0x6a },
#else
	{ 0x3030, 0x11 },
	{ 0x3031, 0x11 },
	{ 0x3032, 0x11 },
	{ 0x3033, 0x11 },
	{ 0x3034, 0x11 },
	{ 0x3035, 0x11 },
	{ 0x3036, 0x11 },
	{ 0x3037, 0x11 },
	{ 0x3038, 0x01 },
	{ 0x3039, 0x1d },
	{ 0x303a, 0x00 },
	{ 0x303b, 0x0a },
#endif

	{ 0x3015, 0x12 },
	{ 0x3014, 0x04 },
	{ 0x3013, 0xf7 },

	{ 0x3104, 0x02 },
	{ 0x3105, 0xfd },
	{ 0x3106, 0x00 },
	{ 0x3107, 0xff },
	{ 0x3308, 0xa5 },
	{ 0x3316, 0xff },
	{ 0x3317, 0x00 },
	{ 0x3087, 0x02 },
	{ 0x3082, 0x20 },
	{ 0x3300, 0x13 },
	{ 0x3301, 0xde },
	{ 0x3302, 0xef },

	{ 0x30b8, 0x20 },
	{ 0x30b9, 0x17 },
	{ 0x30ba, 0x04 },
	{ 0x30bb, 0x08 },

	{ 0x3100, 0x02 }, /* set raw format */
	{ 0x3304, 0x00 },
	{ 0x3400, 0x00 },
	{ 0x3404, OV3640_BGGR},

	{ 0x3020, 0x01 }, /* Size, 2048x1536, QXGA */
	{ 0x3021, 0x1d },
	{ 0x3022, 0x00 },
	{ 0x3023, 0x0a },
	{ 0x3024, 0x08 },
	{ 0x3025, 0x18 },
	{ 0x3026, 0x06 },
	{ 0x3027, 0x0c },

	{ 0x335f, 0x68 },
	{ 0x3360, 0x18 },
	{ 0x3361, 0x0c },
	{ 0x3362, 0x68 },
	{ 0x3363, 0x08 },
	{ 0x3364, 0x04 },
	{ 0x3403, 0x42 },

	{ 0x3088, 0x08 },
	{ 0x3089, 0x00 },
	{ 0x308a, 0x06 },
	{ 0x308b, 0x00 },

	{ 0x3507, 0x06 },
	{ 0x350a, 0x4f },
	{ 0x3600, 0xc4 },

#if COLOR_BAR_EN == 1
	{ 0x307B, 0x4a },   //color bar[1:0]
	{ 0x307D, 0xa0 },   //color bar[7]
	{ 0x306C, 0x00 },   //color bar[4]
	{ 0x3080, 0x11 },   //color bar[7] enable
#endif
	{ 0xffff, 0xff },
};
#endif

static const regval16_t ov3640_raw_fmt_table[] =
{
	{ 0x3100, 0x22 },
	{ 0x3304, 0x01 },
 	{ 0x3400, 0x03 },
 	{ 0x3600, 0xC4 },
 	{ 0x3404, OV3640_BGGR },
 	{ 0xffff, 0xff },
};

static const regval16_t ov3640_yuv_fmt_table[] =
{
	{ 0x3100, 0x02 },
	{ 0x3304, 0x00 },
	{ 0x3400, 0x00 },
	{ 0x3404, OV3640_UYVY},
	{ 0xffff, 0xff },
};

static const regval16_t ov3640_scale_vga_table[] =
{
	{ 0x3012, 0x10 }, //xga

	{ 0x3020, 0x01 },
	{ 0x3021, 0x1d },
	{ 0x3022, 0x00 },
	{ 0x3023, 0x06 },
	{ 0x3024, 0x08 },
	{ 0x3025, 0x18 },
	{ 0x3026, 0x03 },
	{ 0x3027, 0x04 },
	{ 0x302a, 0x03 },
	{ 0x302b, 0x10 },
	{ 0x3075, 0x24 },
	{ 0x300d, 0x01 },
	{ 0x30d7, 0x90 },
	{ 0x3069, 0x04 },

	{ 0x3302, 0xef },
	{ 0x335f, 0x34 },
	{ 0x3360, 0x0c },
	{ 0x3361, 0x04 },
#if 1   //648*484
	{ 0x3362, 0x12 }, /*[6:4]=V[10:8]=1*/  /*[3:0]=H[11:8]=2*/
	{ 0x3363, 0x88 }, //H:0x288=648 /*H[7:0]*/
	{ 0x3364, 0xe8 }, //V:0x1e4=484 /*V[7:0]*/
#else   //1288x724
	{ 0x3362, 0x25 }, /*[6:4]=V[10:8]=1*/  /*[3:0]=H[11:8]=2*/
	{ 0x3363, 0x08 }, //H:0x508=1288 /*H[7:0]*/
	{ 0x3364, 0xd4 }, //V:0x2d4=724 /*V[7:0]*/
#endif
	{ 0x3403, 0x42 },

	{ 0x302c, 0x0e }, /* EXHTS */
	{ 0x302d, 0x00 }, /* EXVTS[15:8] */
	{ 0x302e, 0x10 }, /* EXVTS[7:0] */

	{ 0x3088, 0x02 },
	{ 0x3089, 0x80 },
	{ 0x308a, 0x01 },
#if 0
	{ 0x308b, 0xe0 },
#else
	{ 0x308b, 0xe2 }, /* if mipi clock will auto stop, must add 1 vertical line. */
#endif
	{ 0xffff, 0xff },
};

static const regval16_t ov3640_qxga_table[] =
{
	{ 0x3012, 0x00 }, //qxga mode

	{ 0x3020, 0x01 }, /* Size, 2048x1536, QXGA */
	{ 0x3021, 0x1d },
	{ 0x3022, 0x00 },
	{ 0x3023, 0x0a },
	{ 0x3024, 0x08 },
	{ 0x3025, 0x18 },
	{ 0x3026, 0x06 },
	{ 0x3027, 0x0c },

	{ 0x335f, 0x68 },
	{ 0x3360, 0x18 },
	{ 0x3361, 0x0c },
	{ 0x3362, 0x68 },//V+H
	{ 0x3363, 0x08 },//H
	{ 0x3364, 0x04 },//V
	{ 0x3403, 0x42 },

	{ 0x302c, 0x00 }, /* EXHTS */
	{ 0x302d, 0x00 }, /* EXVTS[15:8] */
	{ 0x302e, 0x00 }, /* EXVTS[7:0] */

	{ 0x3088, 0x08 },
	{ 0x3089, 0x00 },
	{ 0x308a, 0x06 },
#if 0
	{ 0x308b, 0x00 },
#else
	{ 0x308b, 0x02 }, /* if mipi clock will auto stop, must add 1 vertical line. */
#endif
	{ 0xffff, 0xff },
};

#if 0
static const regval16_t ov3640_suspend_table[] =
{
	{0x300e, 0xb2},
	{0x308d, 0x14},
	{0x3086, 0x0f},
	{0xffff, 0xff},
};

static const regval16_t ov3640_resume_table[] =
{
	{0x3086, 0x08},
	{0x308d, 0x14},
	{0x300e, 0x32},
	{0xffff, 0xff},
};
#endif

void test_delay (INT16U i)
{
	INT16U j;

	for (j=0;j<(i<<4);j++)
		i=i;
}

static INT32S ov3640_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	ov3640_handle = drv_l2_sccb_open(OV3640_ID, 16, 8);
	if(ov3640_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	ov3640_handle.devNumber = I2C_0;
	ov3640_handle.slaveAddr = OV3640_ID;
	ov3640_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void ov3640_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(ov3640_handle) {
		drv_l2_sccb_close(ov3640_handle);
		ov3640_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	ov3640_handle.slaveAddr = 0;
	ov3640_handle.clkRate = 0;
#endif
}

static INT32S ov3640_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(ov3640_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[3];

	data[0] = (reg >> 8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value;
	return drv_l1_i2c_bus_write(&ov3640_handle, data, 3);
#endif
}

#if 0
static INT32S ov3640_sccb_read(INT16U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(ov3640_handle, reg, &data) >= 0) {
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
	if(drv_l1_i2c_bus_write(&ov3640_handle, data, 2) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&ov3640_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}
#endif

static INT32S ov3640_sccb_write_table(regval16_t *pTable)
{
	while(1) {
		if(pTable->reg_num == 0xFFFF && pTable->value == 0xFF) {
			break;
		}

		DBG_PRINT("0x%04x, 0x%02x\r\n", pTable->reg_num, pTable->value);
		if(ov3640_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
		pTable++;
	}
	return STATUS_OK;
}

#ifdef MIPI_ISR_TEST
static void mipi_ov3640_handle(INT32U event)
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

/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
//static void ov3640_cdsp_mipi_init(void)
void ov3640_cdsp_mipi_init(void)
{
	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

	// cdsp init
	drv_l2_cdsp_open();

	// set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk
	R_FUNPOS1 |= ((1<<25)|(1<<22)|(1<<6)|(1<<1));//0x02400042;

	// mclk output
	//drv_l2_sensor_set_mclkout(ov3640_cdsp_mipi_ops.info[0].mclk);
	drv_l2_sensor_set_mclkout(MCLK_12M);

	// reguest sccb
	ov3640_sccb_open();

	// reset sensor
	ov3640_sccb_write_table((regval16_t *)ov3640_reset_table);
    osDelay(200);

	// init sensor
	ov3640_sccb_write_table((regval16_t *)ov3640_init_table);

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

    DBG_PRINT("Sensor OV3640 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void ov3640_cdsp_mipi_uninit(void)
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
	ov3640_sccb_close();

	/* Turn off LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
void ov3640_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
//static void ov3640_cdsp_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
	INT16U target_w, target_h;
	gpCdspFmt_t format;

	// set sensor size
	DBG_PRINT("%s = %d\r\n", __func__, index);
	drv_l2_sensor_set_mclkout(MCLK_12M);

    switch(index)
	{
	case 0:
		ov3640_sccb_write_table((regval16_t *)ov3640_raw_fmt_table);
        ov3640_sccb_write_table((regval16_t *)ov3640_scale_vga_table);
		break;

	case 1:
		ov3640_sccb_write_table((regval16_t *)ov3640_raw_fmt_table);
		ov3640_sccb_write_table((regval16_t *)ov3640_qxga_table);
		break;

	case 2:
		ov3640_sccb_write_table((regval16_t *)ov3640_yuv_fmt_table);
		ov3640_sccb_write_table((regval16_t *)ov3640_scale_vga_table);
		break;

	case 3:
		ov3640_sccb_write_table((regval16_t *)ov3640_yuv_fmt_table);
		ov3640_sccb_write_table((regval16_t *)ov3640_qxga_table);
		break;

	default:
		while(1);
	}
    //Enabel mipi clk, set mipi clk
    drv_l2_mipi_ctrl_set_clk(ENABLE, 4);    //u2.added.20151207
    //osDelay(200);

#if 1   // change mclk
	drv_l2_sensor_set_mclkout(ov3640_cdsp_mipi_ops.info[index].mclk);
    //drv_l2_sensor_set_mclkout(MCLK_12M);
#else
    drv_l2_sensor_set_mclkout(MCLK_24M);    //OLDWU
#endif
    osDelay(300);
    // set cdsp format
	format.image_source = C_CDSP_MIPI;
	format.input_format =  ov3640_cdsp_mipi_ops.info[index].input_format;
	format.output_format = ov3640_cdsp_mipi_ops.info[index].output_format;
	target_w = ov3640_cdsp_mipi_ops.info[index].target_w;
	target_h = ov3640_cdsp_mipi_ops.info[index].target_h;
	format.hpixel = ov3640_cdsp_mipi_ops.info[index].sensor_w;
	format.vline = ov3640_cdsp_mipi_ops.info[index].sensor_h;
	format.hoffset = ov3640_cdsp_mipi_ops.info[index].hoffset;
	format.voffset = ov3640_cdsp_mipi_ops.info[index].voffset;
	format.sensor_timing_mode = ov3640_cdsp_mipi_ops.info[index].interface_mode;
	format.sensor_hsync_mode = ov3640_cdsp_mipi_ops.info[index].hsync_active;
	format.sensor_vsync_mode = ov3640_cdsp_mipi_ops.info[index].vsync_active;


    if(drv_l2_cdsp_set_fmt(&format) < 0)
    {
		DBG_PRINT("cdsp set fmt err!!!\r\n");
	}

	// set scale down
	if((format.hpixel > target_w) || (format.vline > target_h)) {
		drv_l2_cdsp_set_yuv_scale(target_w, target_h);
	}

	// cdsp start
	//drv_l2_cdsp_enable(&g_FavTable, target_w, target_h, myFav);
	drv_l2_cdsp_stream_on(DISABLE, bufA, bufB);

	// set mipi format.
#if 1
    //switch(format.input_format)
    switch(5)
	{
	case V4L2_PIX_FMT_YUYV: //(LSB -> MSB), YUYV --> UYVY
		ov3640_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ov3640_mipi_cfg.data_type = MIPI_YUV422;
		ov3640_mipi_cfg.h_back_porch = 1;
		break;

	case V4L2_PIX_FMT_YVYU:	//(LSB -> MSB), YVYU --> UYVY
		ov3640_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ov3640_mipi_cfg.data_type = MIPI_YUV422;
		ov3640_mipi_cfg.h_back_porch = 3;
		break;

	case V4L2_PIX_FMT_UYVY: //(LSB -> MSB), UYVY --> UYVY
		ov3640_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ov3640_mipi_cfg.data_type = MIPI_YUV422;
		ov3640_mipi_cfg.h_back_porch = 4;
		break;

	case V4L2_PIX_FMT_VYUY: //(LSB -> MSB), VYUY --> UYVY
		ov3640_mipi_cfg.data_from_mmr = MIPI_USER_DEFINE;
		ov3640_mipi_cfg.data_type = MIPI_YUV422;
		ov3640_mipi_cfg.h_back_porch = 2;
		break;

	default:
    	/* #if (RD_SIM == 1)
        ov3640_mipi_cfg.blanking_line_en = 1;
        ov3640_mipi_cfg.byte_clk_edge = 0;
        ov3640_mipi_cfg.check_hs_seq = 0;
        ov3640_mipi_cfg.data_from_mmr = 0;
        ov3640_mipi_cfg.data_mask_time = 250;
        ov3640_mipi_cfg.data_type = 3;
        ov3640_mipi_cfg.data_type_to_cdsp = 1;
        ov3640_mipi_cfg.ecc_check_en = 1;
        ov3640_mipi_cfg.ecc_order = 3;
        ov3640_mipi_cfg.h_front_porch = 4;
        ov3640_mipi_cfg.low_power_en = 0;
        ov3640_mipi_cfg.mipi_lane_no = 1;
        ov3640_mipi_cfg.pixel_clk_sel = 1;
        #endif
        */
        ov3640_mipi_cfg.data_from_mmr = MIPI_AUTO_DETECT;
		ov3640_mipi_cfg.data_type = MIPI_RAW8;
		ov3640_mipi_cfg.h_back_porch = 4;
	}

	ov3640_mipi_cfg.h_size = format.hpixel;
	ov3640_mipi_cfg.v_size = format.vline;
#endif
	//mipi start
    #if (MIPI_DEV_NO == 1)
      drv_l1_mipi_set_parameter(MIPI_1, &ov3640_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_ov3640_handle);
      drv_l1_mipi_set_irq_enable(MIPI_1, ENABLE, MIPI_INT_ALL);
      #endif
    #else
      drv_l1_mipi_set_parameter(MIPI_0, &ov3640_mipi_cfg);
      #ifdef MIPI_ISR_TEST
      //drv_l1_mipi_isr_register(mipi_ov3640_handle);
      drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
      #endif
    #endif

    //drv_l2_cdsp_stream_on(DISABLE, bufA, bufB);
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void ov3640_cdsp_mipi_stream_off(void)
{
	//drv_l2_cdsp_stream_off();
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* ov3640_cdsp_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&ov3640_cdsp_mipi_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t ov3640_cdsp_mipi_ops =
{
	SENSOR_OV3640_CDSP_MIPI_NAME,		/* sensor name */
	ov3640_cdsp_mipi_init,
	ov3640_cdsp_mipi_uninit,
	ov3640_cdsp_mipi_stream_on,
	ov3640_cdsp_mipi_stream_off,
	ov3640_cdsp_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_24M,					/* MCLK */
			V4L2_PIX_FMT_SGRBG8,		/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			WIDTH_640,					/* sensor width */
			HEIGHT_480, 				/* sensor height */
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
			320,//OV3640_WIDTH,			/* target width */
			240,//OV3640_HEIGHT, 		/* target height */
			OV3640_WIDTH,				/* sensor width */
			OV3640_HEIGHT, 				/* sensor height */
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
			V4L2_PIX_FMT_VYUY,			/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			WIDTH_640,					/* sensor width */
			HEIGHT_480, 				/* sensor height */
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
			V4L2_PIX_FMT_VYUY,			/* input format */
			V4L2_PIX_FMT_YUYV,			/* output format */
			30,							/* FPS in sensor */
			OV3640_WIDTH,				/* target width */
			OV3640_HEIGHT, 				/* target height */
			OV3640_WIDTH,				/* sensor width */
			OV3640_HEIGHT, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_601,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_HIGH,			/* vsync pin active level */
		},
	}
};
#endif //(defined _SENSOR_OV3640_CDSP_MIPI) && (_SENSOR_OV3640_CDSP_MIPI == 1)
