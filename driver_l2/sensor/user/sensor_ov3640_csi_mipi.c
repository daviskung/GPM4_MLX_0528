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

#if (defined _SENSOR_OV3640_CSI_MIPI) && (_SENSOR_OV3640_CSI_MIPI == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CONFIG_FPGA_EN 				1
#define COLOR_BAR_EN				0
#define MIPI_ISR_EN					0
#define MIPI_LANE_NO				2	// 1 or 2 lane

#define	OV3640_ID					0x78
#define OV3640_WIDTH				2048
#define OV3640_HEIGHT				1536

#define OV3640_VYUY 				0x00
#define OV3640_UYVY					0x02
#define OV3640_BGGR					0x18
#define OV3640_GBRG					0x19
#define OV3640_GRBG					0x1A
#define OV3640_RGGB					0x1B

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#ifndef DISABLE
#define DISABLE     0
#endif

#ifndef ENABLE
#define ENABLE      1
#endif

#ifndef DBG_PRINT
#define DBG_PRINT   printf
#endif

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

	MIPI_USER_DEFINE,	/* data mode, 0:auto detect, 1:user define */
	MIPI_YUV422,		/* data type, valid when data mode is 1*/
	MIPI_DATA_TO_CSI,	/* data type, 1:data[7:0]+2':00 to cdsp, 0: 2'00+data[7:0] to csi */
	NULL,				/* RSD 2 */

	OV3640_WIDTH,		/* width, 0~0xFFFF */
	OV3640_HEIGHT,		/* height, 0~0xFFFF */
	4, 					/* back porch, 0~0xF */
	4,					/* front porch, 0~0xF */
	ENABLE,				/* blanking_line_en, 0:disable, 1:enable */
	NULL,				/* RSD 3 */

	ENABLE,				/* ecc, 0:disable, 1:enable */
	MIPI_ECC_ORDER3,	/* ecc order */
	150,				/* data mask, unit is ns */
	MIPI_CHECK_HS_SEQ	/* check hs sequence or LP00 for clock lane */
};

static const regval16_t ov3640_reset_table[] =
{
	{ 0x3012, 0x80 },
	{ 0xFFFF, 0xFF },
};

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

#if CONFIG_FPGA_EN == 0
	//{ 0x300e, 0x38 }, //30fps
	//{ 0x300f, 0xa2 }, /* a1:1.5 , a2:2 */
	{ 0x300e, 0x37 }, //15fps
	{ 0x300f, 0xa3 },
#else
#if 0
	{ 0x300e, 0x39 },
	{ 0x300f, 0xa1 },
#elif 0
	{ 0x300e, 0x37 }, //15fps
	{ 0x300f, 0xa3 },
#else
	#if MIPI_LANE_NO == 1
	{ 0x300e, 0x3A }, //0x3A +0xa2 = PCLK 36MHz @ 1lane
	{ 0x300f, 0xa2 }, //0xa2 inclk 12mclk
	#elif MIPI_LANE_NO == 2
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
	{ 0x3404, OV3640_VYUY},

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

static const regval16_t ov3640_yuv_fmt_table[] =
{
#if 1
	{ 0x3100, 0x02 },
	{ 0x3304, 0x00 },
	{ 0x3400, 0x00 },
	{ 0x3404, OV3640_VYUY},
#else
	{ 0x3100, 0x22 },
	{ 0x3304, 0x01 },
	{ 0x3400, 0x03 },
	{ 0x3600, 0xC4 },
	{ 0x3404, OV3640_GRBG},
#endif
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
	{ 0x3362, 0x12 },
	{ 0x3363, 0x88 },
	{ 0x3364, 0xe4 },
	{ 0x3403, 0x42 },

	{ 0x302c, 0x0e }, /* EXHTS */
	{ 0x302d, 0x00 }, /* EXVTS[15:8] */
	{ 0x302e, 0x10 }, /* EXVTS[7:0] */

	{ 0x3088, 0x02 },
	{ 0x3089, 0x80 },
	{ 0x308a, 0x01 },
#if 1
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
	{ 0x3362, 0x68 },
	{ 0x3363, 0x08 },
	{ 0x3364, 0x04 },
	{ 0x3403, 0x42 },

	{ 0x302c, 0x00 }, /* EXHTS */
	{ 0x302d, 0x00 }, /* EXVTS[15:8] */
	{ 0x302e, 0x00 }, /* EXVTS[7:0] */

	{ 0x3088, 0x08 },
	{ 0x3089, 0x00 },
	{ 0x308a, 0x06 },
#if 1
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
	ov3640_handle.clkRate = 50;
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

#if MIPI_ISR_EN == 1
static void mipi_ov3640_handle(INT32U dev_no, INT32U event)
{
	if(event & MIPI_LANE0_SOT_SYNC_ERR) {
		DBG_PRINT("LANE0_SOT_SYNC_ERR[%d]\r\n", dev_no);
	}

	if(event & MIPI_HD_1BIT_ERR) {
		DBG_PRINT("HD_1BIT_ERR[%d]\r\n", dev_no);
	}

	if(event & MIPI_HD_NBIT_ERR) {
		DBG_PRINT("HD_NBIT_ERR[%d]\r\n", dev_no);
	}

	if(event & MIPI_DATA_CRC_ERR) {
		DBG_PRINT("DATA_CRC_ERR[%d]\r\n", dev_no);
	}

	if(event & MIPI_LANE1_SOT_SYNC_ERR) {
		DBG_PRINT("LANE1_SOT_SYNC_ERR[%d]\r\n", dev_no);
	}

	if(event & MIPI_CCIR_SOF) {
		DBG_PRINT("CCIR_SOF[%d]\r\n", dev_no);
	}
}
#endif

/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void ov3640_csi_mipi_init(void)
{
	// Turn on LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

	// set Mclk(CSI_CLKO) to IOD12, for MIPI Mclk
	R_FUNPOS1 |= 0x42;

	/* set horizontal/vertical scale ratio to 0 */
	R_CSI_TG_HRATIO = 0;
	R_CSI_TG_VRATIO = 0;

	/* Sensor field 0 vertical latch start register */
	R_CSI_TG_VL0START = 0x0000;
	/* *P_Sensor_TG_V_L1Start = 0x0000 */
	R_CSI_TG_VL1START = 0x0000;
	/* Sensor horizontal start register */
	R_CSI_TG_HSTART = 0x0000;

	/* reset control 0/1 */
	R_CSI_TG_CTRL0 = 0;
	R_CSI_TG_CTRL1 = 0;

	/* mclk output */
	drv_l2_sensor_set_mclkout(ov3640_csi_mipi_ops.info[0].mclk);

	// reguest sccb
	ov3640_sccb_open();

	// reset sensor
	ov3640_sccb_write_table((regval16_t *)ov3640_reset_table);
	osDelay(200);

	// init sensor
	DBG_PRINT("Sensor OV3640 use %d Lane\r\n", MIPI_LANE_NO);
	ov3640_sccb_write_table((regval16_t *)ov3640_init_table);

	// mipi enable
	drv_l1_mipi_init(MIPI_0, ENABLE);
	drv_l2_sensor_set_csi_source(MIPI_INTERFACE);
	DBG_PRINT("Sensor OV3640 csi mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
static void ov3640_csi_mipi_uninit(void)
{
	// disable mclk
	drv_l2_sensor_set_mclkout(MCLK_NONE);

	// csi disabale
	R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CSIEN;	//disable sensor controller
	R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CLKOEN;	//disable sensor clock out

	// mipi disable
	drv_l1_mipi_init(MIPI_0, DISABLE);
	drv_l2_sensor_set_csi_source(0);

	// release sccb
	ov3640_sccb_close();

	// Turn off LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   stream start function
 * @param   info index
 *
 * @return 	none
 */
static void ov3640_csi_mipi_stream_on(INT32U index, INT32U bufA, INT32U bufB)
{
	/* enable CSI output clock for SCCB */
	drv_l2_sensor_set_mclkout(ov3640_csi_mipi_ops.info[index].mclk);

	/* set start frame buffer address */
	if(bufA) {
		drv_l1_csi_set_buf(bufA);
	} else {
		DBG_PRINT("Input frame buffer address error, fail to start %s\r\n", ov3640_csi_mipi_ops.name);
		ov3640_csi_mipi_ops.stream_stop();
	}

	/* Set Horizontal/Vertical counter reset, Vertical counter incresase selection, sensor owner inerrupt enable, capture/preview mode */
	R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_HRST | MASK_CSI_CTRL0_VADD | MASK_CSI_CTRL0_VRST | MASK_CSI_CTRL0_OWN_IRQ | MASK_CSI_CTRL0_CAP;

	/* Set delay clock 244[0:3] */
	R_CSI_TG_CTRL1 |= DELAY_1CLOCK;

	R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_RGB1555;

	/* According to the specific information, set CSI register */
	/* set input interface */
	if(ov3640_csi_mipi_ops.info[index].interface_mode == MODE_CCIR_HREF) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_HREF;
	}

	/* set input & output format */
	if(ov3640_csi_mipi_ops.info[index].input_format == V4L2_PIX_FMT_VYUY) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVIN | MASK_CSI_CTRL0_YUVTYPE;
	}

	if(ov3640_csi_mipi_ops.info[index].input_format == V4L2_PIX_FMT_UYVY) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVIN;
	}

	if(ov3640_csi_mipi_ops.info[index].output_format == V4L2_PIX_FMT_VYUY) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVOUT;
	}

	/* Set sensor's registers via SCCB */
	if(ov3640_sccb_write_table((regval16_t *)ov3640_yuv_fmt_table) < 0) {
		DBG_PRINT("ov3640 init fail!!!\r\n");
	}

	DBG_PRINT("%s = %d\r\n", __func__, index);
	switch(index)
	{
	case 0:
		//ov3640_sccb_write_table((regval16_t *)ov3640_scale_vga_table);
		R_CSI_TG_HWIDTH = ov3640_csi_mipi_ops.info[index].target_w;
		R_CSI_TG_VHEIGHT = ov3640_csi_mipi_ops.info[index].target_h;
		R_CSI_TG_HRATIO = 0;		// No Scale
		R_CSI_TG_VRATIO = 0;		// No Scale
		break;

	case 1:
		//ov3640_sccb_write_table((regval16_t *)ov3640_qxga_table);
		R_CSI_TG_HWIDTH = ov3640_csi_mipi_ops.info[index].target_w;
		R_CSI_TG_VHEIGHT = ov3640_csi_mipi_ops.info[index].target_h;
		R_CSI_TG_HRATIO = 0;		// No Scale
		R_CSI_TG_VRATIO = 0;		// No Scale
		break;

	default:
		while(1);
	}

	/* csi long burst width need 32 align */
	if(ov3640_csi_mipi_ops.info[index].target_w & 0x1F) {
		R_CSI_SEN_CTRL &= ~(1 << 7);
	} else {
		R_CSI_SEN_CTRL |= 1 << 7;
	}

	/* Enable interrupt */
    drv_l1_csi_set_irq(1);

	/* Clear sensor interrupt status */
	R_TGR_IRQ_STATUS = 0x3F;

	/* enable frame end interrupt */
	R_TGR_IRQ_EN |= MASK_CSI_FRAME_END_FLAG | MASK_CSI_FIFO_OVERFLOW_FLAG;

	/* enable CSI module */
	R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CSIEN;

	// mipi start
	ov3640_mipi_cfg.h_size = ov3640_csi_mipi_ops.info[index].sensor_w;
	ov3640_mipi_cfg.v_size = ov3640_csi_mipi_ops.info[index].sensor_h;
	drv_l1_mipi_set_parameter(MIPI_0, &ov3640_mipi_cfg);

#if MIPI_ISR_EN == 1
	drv_l1_mipi_isr_register(mipi_ov3640_handle);
	drv_l1_mipi_set_irq_enable(MIPI_0, ENABLE, MIPI_INT_ALL);
#endif
}

/**
 * @brief   stream stop function
 * @param   none
 * @return 	none
 */
static void ov3640_csi_mipi_stream_off(void)
{
	/* Disable interrupt */
    drv_l1_csi_set_irq(0);

	/* Disable sensor interrupt */
	R_TGR_IRQ_EN &= ~MASK_CSI_FRAME_END_ENABLE;

	/* Disable CSI module */
	R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CSIEN;

	/* Disable sensor output clock*/
	R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CLKOEN;

	/* Clear sensor interrupt status */
	R_TGR_IRQ_STATUS = 0x3F;
}

/**
 * @brief   get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
static drv_l2_sensor_info_t* ov3640_csi_mipi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1)) {
		return NULL;
	} else {
		return (drv_l2_sensor_info_t*)&ov3640_csi_mipi_ops.info[index];
	}
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t ov3640_csi_mipi_ops =
{
	SENSOR_OV3640_CSI_MIPI_NAME,		/* sensor name */
	ov3640_csi_mipi_init,
	ov3640_csi_mipi_uninit,
	ov3640_csi_mipi_stream_on,
	ov3640_csi_mipi_stream_off,
	ov3640_csi_mipi_get_info,
	{
		/* 1st info */
		{
			MCLK_12M,					/* MCLK */
			V4L2_PIX_FMT_UYVY,			/* input format */
			V4L2_PIX_FMT_VYUY,			/* output format */
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
			MCLK_12M,					/* MCLK */
			V4L2_PIX_FMT_UYVY,			/* input format */
			V4L2_PIX_FMT_VYUY,			/* output format */
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
#endif //(defined _SENSOR_OV3640_CSI_MIPI) && (_SENSOR_OV3640_CSI_MIPI == 1)
