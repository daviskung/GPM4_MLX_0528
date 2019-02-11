/******************************************************
* sensor_ov7670_csi.c
*
* Purpose: OV7670 sensor driver for CSI path
*
* Author: Eugene Hsu
*
* Date: 2014/08/25
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/

/*******************************************************
    Include file
*******************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board_config.h"
#include "drv_l1_sfr.h"
#include "drv_l1_csi.h"
#include "drv_l1_i2c.h"
#include "drv_l2_sensor.h"
#include "drv_l2_csi.h"
#include "drv_l2_sccb.h"

#include "gp_aeawb.h"

#if (defined _SENSOR_OV7670_CSI) && (_SENSOR_OV7670_CSI == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define OV7670_ID		0x42

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
typedef struct regval8_s
{
	INT8U reg_num;
	INT8U value;
} regval8_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
#if SCCB_MODE == SCCB_GPIO
	static void *ov7670_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t ov7670_handle;
#endif

/************************
 MCLK= 24Mhz;
 PCLK=   Mhz
 Frame width= 640
 Frame Height= 480
*************************/
/* 300K YUV 15fps */
static const regval8_t OV7670_YUV_CONFIG[] =
{
		{0x11, 0x02},	 // 30fps

		//{0x11, 0x04},    // 27fps
		//{0x11, 0x06},    // 14.5fps
		//{0x11, 0x09},    // 13.5fps
		//{0x11, 0x08},    // 15fps
		//{0x11, 0x06},    // 10fps
		//{0x11, 0x09},    // 6.7fps

		{0x3a, 0x04},
		{0x12, 0x00},
		{0x17, 0x13},
		{0x18, 0x01},
		{0x32, 0xb6},
		{0x19, 0x02},
		{0x1a, 0x7a},
		{0x03, 0x0a},
		{0x0c, 0x00},
		{0x3e, 0x00},
		{0x70, 0x3a},
		{0x71, 0x35},
		{0x72, 0x11},
		{0x73, 0xf0},
		{0xa2, 0x02},

		{0x7a, 0x24},
		{0x7b, 0x04},
		{0x7c, 0x0a},
		{0x7d, 0x17},
		{0x7e, 0x32},
		{0x7f, 0x3f},
		{0x80, 0x4c},
		{0x81, 0x58},
		{0x82, 0x64},
		{0x83, 0x6f},
		{0x84, 0x7a},
		{0x85, 0x8c},
		{0x86, 0x9e},
		{0x87, 0xbb},
		{0x88, 0xd2},
		{0x89, 0xe5},

		{0x13, 0xe0},
		{0x00, 0x00},
		{0x10, 0x00},
		{0x0d, 0x40},
		{0x14, 0x18},
		{0xa5, 0x02},
		{0xab, 0x03},
		{0x24, 0x95},
		{0x25, 0x33},
		{0x26, 0xe3},
		{0x9f, 0x78},
		{0xa0, 0x68},
		{0xa1, 0x03},
		{0xa6, 0xd8},
		{0xa7, 0xd8},
		{0xa8, 0xf0},
		{0xa9, 0x90},
		{0xaa, 0x94},
		{0x13, 0xe5},

		{0x0e, 0x61},
		{0x0f, 0x4b},
		{0x16, 0x02},
		{0x1e, 0x3f}, 	//Flip & Mirror
		{0x21, 0x02},
		{0x22, 0x91},
		{0x29, 0x07},
		{0x33, 0x0b},
		{0x35, 0x0b},
		{0x37, 0x1d},
		{0x38, 0x71},
		{0x39, 0x2a},
		{0x3c, 0x78},
		{0x4d, 0x40},
		{0x4e, 0x20},
		{0x69, 0x00},

		/* 30FPS */
		{0x6b, 0x8a},	// pclk*6
		{0x2d, 0x40},	// dummy byte
		{0x2e, 0x00},

		/* 27 FPS */
		//{0x6b, 0xca},	// pclk*8

		/* 15 FPS */
		//{0x6b, 0xca},	// pclk*8

		/* 10 FPS */
		//{0x6b, 0x4a},	// pclk*4

		/* 7 FPS */
		//{0x6b, 0x4a},	// pclk*4

		{0x74, 0x10},
		{0x8d, 0x4f},
		{0x8e, 0x00},
		{0x8f, 0x00},
		{0x90, 0x00},
		{0x91, 0x00},

		{0x96, 0x00},
		{0x9a, 0x80},
		{0xb0, 0x84},
		{0xb1, 0x0c},
		{0xb2, 0x0e},
		{0xb3, 0x82},
		{0xb8, 0x0a},

		{0x43, 0x0a},
		{0x44, 0xf0},
		{0x45, 0x44},
		{0x46, 0x7a},
		{0x47, 0x27},
		{0x48, 0x3c},
		{0x59, 0xbc},
		{0x5a, 0xde},
		{0x5b, 0x54},
		{0x5c, 0x8a},
		{0x5d, 0x4b},
		{0x5e, 0x0f},
		{0x6c, 0x0a},
		{0x6d, 0x55},
		{0x6e, 0x11},
		{0x6f, 0x9e},

		{0x6a, 0x40},
		{0x01, 0x40},
		{0x02, 0x40},
		{0x13, 0xe7},

		{0x4f, 0x80},
		{0x50, 0x80},
		{0x51, 0x00},
		{0x52, 0x22},
		{0x53, 0x5e},
		{0x54, 0x80},
		{0x58, 0x9e},

		{0x62, 0x08},
		{0x63, 0x8f},
		{0x65, 0x00},
		{0x64, 0x08},
		{0x94, 0x08},
		{0x95, 0x0c},
		{0x66, 0x05},

		{0x41, 0x08},
		{0x3f, 0x00},
		{0x75, 0x05},
		{0x76, 0xe1},
		{0x4c, 0x00},
		{0x77, 0x01},
		{0x3d, 0xc2},
		{0x4b, 0x09},
		{0xc9, 0x60},
		{0x41, 0x38},
		{0x56, 0x40},

		{0x34, 0x11},

		{0x3b, 0xca},	//enable night mode
		//{0x3b, 0x4a},	//disable night mode

		{0xa4, 0x88},
		{0x96, 0x00},
		{0x97, 0x30},
		{0x98, 0x20},
		{0x99, 0x30},
		{0x9a, 0x84},
		{0x9b, 0x29},
		{0x9c, 0x03},
		{0x9d, 0x98},
		{0x9e, 0x7f},
		{0x78, 0x04},

		{0x79, 0x01},
		{0xc8, 0xf0},
		{0x79, 0x0f},
		{0xc8, 0x00},
		{0x79, 0x10},
		{0xc8, 0x7e},
		{0x79, 0x0a},
		{0xc8, 0x80},
		{0x79, 0x0b},
		{0xc8, 0x01},
		{0x79, 0x0c},
		{0xc8, 0x0f},
		{0x79, 0x0d},
		{0xc8, 0x20},
		{0x79, 0x09},
		{0xc8, 0x80},
		{0x79, 0x02},
		{0xc8, 0xc0},
		{0x79, 0x03},
		{0xc8, 0x40},
		{0x79, 0x05},
		{0xc8, 0x30},
		{0x79, 0x26},

//		{0x3b, 0x00},    // 60Hz = 0x00, 50Hz = 0x08
		{0x3b, 0x08},    // 60Hz = 0x00, 50Hz = 0x08
//		{0x6b, 0x4a},
//		{0x6b, 0x0a},

/*
		{0x11, 0x81},    // 15fps
		{0x6b, 0x0a},
		{0x92, 0x40},
		{0x9d, 0x55},
		{0x9e, 0x47},
		{0xa5, 0x02},
		{0xab, 0x03},
*/
		{0xFF, 0xFF}
};

static INT32S ov7670_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	ov7670_handle = drv_l2_sccb_open(OV7670_ID, 8, 8);
	if(ov7670_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init(I2C_0);
	ov7670_handle.devNumber = I2C_0;
	ov7670_handle.slaveAddr = OV7670_ID;
	ov7670_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void ov7670_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(ov7670_handle) {
		drv_l2_sccb_close(ov7670_handle);
		ov7670_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_0);
	ov7670_handle.devNumber = 0;
	ov7670_handle.slaveAddr = 0;
	ov7670_handle.clkRate = 0;
#endif
}

static INT32S ov7670_sccb_write(INT8U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(ov7670_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[2];

	data[0] = reg;
	data[1] = value;
	return drv_l1_i2c_bus_write(&ov7670_handle, data, 2);
#endif
}

#if 0
static INT32S ov7670_sccb_read(INT8U reg, INT8U *value)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(ov7670_handle, reg, &data) >= 0) {
		*value = (INT8U)data;
		return STATUS_OK;
	} else {
		*value = 0xFF;
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[1];

	data[0] = reg;
	if(drv_l1_i2c_bus_write(&ov7670_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&ov7670_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}
#endif

static INT32S ov7670_sccb_write_table(regval8_t *pTable)
{
	while(1) {
		if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
			break;
		}

		if(ov7670_sccb_write(pTable->reg_num, pTable->value) < 0) {
			DBG_PRINT("sccb write fail.\r\n");
			continue;
		}
		pTable++;
	}
	return STATUS_OK;
}

/**
 * @brief   ov7670 initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void ov7670_csi_init(void)
{
	/*Select csi data pin & ctrl*/
	//function_position_sel(ISP_CLKO__IOD7, CSI_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9, CSI_DATA2_9__IOB8_15, ISP_DATA0_1__IOB7_6);

	/* Turn on LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

	/* request SCCB */
	ov7670_sccb_open();

	DBG_PRINT("Sensor OV7670 csi interface init completed\r\n");
}

/**
 * @brief   ov7670 un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void ov7670_csi_uninit(void)
{
	// disable mclk
	drv_l2_sensor_set_mclkout(MCLK_NONE);

	// csi disable
    drv_l2_csi_stop();

	// release SCCB
	ov7670_sccb_close();
	// Turn off LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
}

/**
 * @brief   ov7670 stream start function
 * @param   info index
 *
 * @return 	none
 */
void ov7670_csi_stream_start(INT32U index, INT32U bufA, INT32U bufB)
{
    gpCSIPara_t csi_Para;

	/* mclk output */
	if(CSI_MIPI_CLKO_POS == CSI_MIPI_CLKO_MUX0)
        drv_l2_sensor_clkout_position_set(ISP_CLKO__IOC9);
    else if(CSI_MIPI_CLKO_POS == CSI_MIPI_CLKO_MUX1)
        drv_l2_sensor_clkout_position_set(ISP_CLKO__IOD7);
    else
        drv_l2_sensor_clkout_position_set(ISP_CLKO__IOD12);
    /* enable CSI output clock for SCCB */
	drv_l2_sensor_set_mclkout(ov7670_sensor_csi_ops.info[index].mclk);

	gp_memset((INT8S *)&csi_Para, 0, sizeof(gpCSIPara_t));
	/* set input & output format */
	if(ov7670_sensor_csi_ops.info[index].input_format == V4L2_PIX_FMT_VYUY) {
		csi_Para.csi_fmt.input_format = CSI_IN_YUYV;
	}

	if(ov7670_sensor_csi_ops.info[index].output_format == V4L2_PIX_FMT_VYUY) {
		csi_Para.csi_fmt.output_format = CSI_OUT_YUYV;
	}

    csi_Para.csi_fmt.interface_mode = CSI_HREF;
    csi_Para.csi_fmt.preview_mode = CSI_CAPTURE_MODE;
    csi_Para.csi_tim.hrst_mode = ENUM_CSI_RISING_EDGE;
    csi_Para.csi_tim.vadd_mode = ENUM_CSI_RISING_EDGE;
    csi_Para.csi_tim.vrst_mode = ENUM_CSI_RISING_EDGE;
    csi_Para.csi_tim.d_type = DELAY_1CLOCK;
	csi_Para.h_start = csi_Para.v_start = 0x0;
    csi_Para.h_end = csi_Para.v_end = 0xfff;

	switch(index)
	{
	case 0:
		csi_Para.target_w = ov7670_sensor_csi_ops.info[index].target_w;
		csi_Para.target_h = ov7670_sensor_csi_ops.info[index].target_h;
		csi_Para.hratio = 0;		// No Scale
		csi_Para.vratio = 0;		// No Scale
		break;

	case 1:
		csi_Para.target_w = ov7670_sensor_csi_ops.info[index].target_w;
		csi_Para.target_h = ov7670_sensor_csi_ops.info[index].target_h * 2;
		csi_Para.hratio = 0x0102;	// Scale to 1/2
		csi_Para.vratio = 0x0102;	// Scale to 1/2
		break;

	case 2:
		csi_Para.target_w = ov7670_sensor_csi_ops.info[index].target_w;
		csi_Para.target_h = ov7670_sensor_csi_ops.info[index].target_h * 2;
		csi_Para.hratio = 0x0104;	// Scale to 1/4
		csi_Para.vratio = 0x0104;	// Scale to 1/4
		break;

	default:
		while(1);
	}

	/* Set sensor's registers via SCCB */
	if(ov7670_sccb_write_table((regval8_t *)OV7670_YUV_CONFIG) < 0) {
		DBG_PRINT("ov7670 init fail!!!\r\n");
	}

	DBG_PRINT("%s = %d\r\n", __func__, index);

    if(drv_l2_csi_set_fmt(&csi_Para.csi_fmt) < 0)
    {
		DBG_PRINT("csi set fmt err!!!\r\n");
	}

	if(bufA) {
		drv_l2_csi_stream_on((gpCSIPara_t *)&csi_Para, bufA, 0);
	} else {
		DBG_PRINT("Input frame buffer address error, fail to start %s\r\n", ov7670_sensor_csi_ops.name);
		ov7670_sensor_csi_ops.stream_stop();
	}
}

/**
 * @brief   ov7670 stream stop function
 * @param   none
 * @return 	none
 */
void ov7670_csi_stream_stop(void)
{
    drv_l2_csi_stream_off();
}

/**
 * @brief   ov7670 get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
drv_l2_sensor_info_t* ov7670_csi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1))
		return NULL;
	else
		return (drv_l2_sensor_info_t*)&ov7670_sensor_csi_ops.info[index];
}

void sensor_register_ae_ctrl(INT32U *handle) __attribute__((weak));
void sensor_register_ae_ctrl(INT32U *handle)
{
	*handle = (INT32U) NULL;
}

void sensor_get_ae_info(sensor_exposure_t *si) __attribute__((weak));
void sensor_get_ae_info(sensor_exposure_t *si)
{
    //memcpy(si, &seInfo, sizeof(sensor_exposure_t));
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t ov7670_sensor_csi_ops =
{
	SENSOR_OV7670_CSI_NAME,				/* sensor name */
	ov7670_csi_init,
	ov7670_csi_uninit,
	ov7670_csi_stream_start,
	ov7670_csi_stream_stop,
	ov7670_csi_get_info,
	{
		/* 1nd info */
		{
			MCLK_24M,					/* CSI clock */
			V4L2_PIX_FMT_VYUY,			/* input format */
			V4L2_PIX_FMT_VYUY,			/* output format */
			CSI_SENSOR_FPS_30,			/* FPS in sensor */
			WIDTH_640,					/* target width */
			HEIGHT_480, 				/* target height */
			WIDTH_640,					/* sensor width */
			HEIGHT_480, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_HREF,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_LOW,			/* vsync pin active level */
		},
		/* 2st info */
		{
			MCLK_24M,					/* CSI clock */
			V4L2_PIX_FMT_VYUY,			/* input format */
			V4L2_PIX_FMT_VYUY,			/* output format */
			CSI_SENSOR_FPS_30,			/* FPS in sensor */
			WIDTH_320,					/* target width */
			HEIGHT_240, 				/* target height */
			WIDTH_640,					/* sensor width */
			HEIGHT_480, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_HREF,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_LOW,			/* vsync pin active level */
		},
		/* 3st info */
		{
			MCLK_24M,					/* CSI clock */
			V4L2_PIX_FMT_VYUY,			/* input format */
			V4L2_PIX_FMT_VYUY,			/* output format */
			CSI_SENSOR_FPS_30,			/* FPS in sensor */
			160,						/* target width */
			120,		 				/* target height */
			WIDTH_640,					/* sensor width */
			HEIGHT_480, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_HREF,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_LOW,			/* vsync pin active level */
		}
	}
};
#endif //(defined _SENSOR_OV7670_CSI) && (_SENSOR_OV7670_CSI == 1)
