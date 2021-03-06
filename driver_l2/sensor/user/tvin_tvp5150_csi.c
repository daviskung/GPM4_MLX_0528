/******************************************************
* sensor_ov5150_csi.c
*
* Purpose: tvp5150 sensor driver for CSI path
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
#include "drv_l1_sfr.h"
#include "drv_l1_csi.h"
#include "drv_l1_i2c.h"
#include "drv_l2_sensor.h"
#include "drv_l2_sccb.h"

#include "gp_aeawb.h"

#if (defined _TVIN_TVP5150_CSI) && (_TVIN_TVP5150_CSI == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#if TVP5150_SLAVE_ID_SEL == 0
#define TVP5150_ID		0xBA
#else
#define TVP5150_ID		0xB8
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
	static void *tvp5150_handle;
#elif SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t tvp5150_handle;
#endif

static INT32S tvp5150_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
	tvp5150_handle = drv_l2_sccb_open(TVP5150_ID, 8, 8);
	if(tvp5150_handle == 0) {
		DBG_PRINT("Sccb open fail.\r\n");
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_init();
	tvp5150_handle.slaveAddr = TVP5150_ID;
	tvp5150_handle.clkRate = 100;
#endif
	return STATUS_OK;
}

static void tvp5150_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
	if(tvp5150_handle) {
		drv_l2_sccb_close(tvp5150_handle);
		tvp5150_handle = NULL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit();
	tvp5150_handle.slaveAddr = 0;
	tvp5150_handle.clkRate = 0;
#endif
}

static INT32S tvp5150_sccb_write(INT8U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(tvp5150_handle, reg, value);

#elif SCCB_MODE == SCCB_HW_I2C
	INT8U data[2];

	data[0] = reg;
	data[1] = value;
	return drv_l1_i2c_bus_write(&tvp5150_handle, data, 2);
#endif
}

//static INT32S tvp5150_sccb_read(INT16U reg, INT8U *value)
static INT32S tvp5150_sccb_read(INT16U reg)
{
#if SCCB_MODE == SCCB_GPIO
	INT16U data;
    INT8U *value = 0;
	if(drv_l2_sccb_read(tvp5150_handle, reg, &data) >= 0) {
		*value = (INT8U)data;
		DBG_PRINT("0x%02x, 0x%02x\r\n", reg, *value);
		return STATUS_OK;
	} else {
		*value = 0xFF;
		return STATUS_FAIL;
	}
#elif SCCB_MODE == SCCB_HW_I2C
	return STATUS_FAIL;
#endif
	return STATUS_OK;
}

static INT32S tvp5150_init(void)
{
	tvp5150_sccb_write(0x05, 0x01);
	osDelay(200);
	tvp5150_sccb_read(0x05);

	tvp5150_sccb_write(0x05, 0x00);
	//osDelay(200);
	tvp5150_sccb_read(0x05);

	tvp5150_sccb_write(0x02, 0x00);
	//osDelay(200);
	tvp5150_sccb_read(0x02);

	tvp5150_sccb_write(0xD0, 0xFF);
	//osDelay(200);
	tvp5150_sccb_read(0xD0);

	tvp5150_sccb_write(0x0f, 0x0a);
	//osDelay(200);
	tvp5150_sccb_read(0x0f);

	tvp5150_sccb_write(0x03, 0x6d);
	//osDelay(200);
	tvp5150_sccb_read(0x03);
	return tvp5150_sccb_write(0x0d, 0x47);
}

/**
 * @brief   tvp5150 initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void tvp5150_csi_init(void)
{
	/*Select csi data pin & ctrl*/
    //function_position_sel(ISP_CLKO__IOD7, CSI_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9, CSI_DATA2_9__IOB8_15, ISP_DATA0_1__IOB7_6);
	/* Turn on LDO 2.8V for CSI sensor */
	//drv_l1_power_ldo_28_ctrl(1, LDO_LDO28_2P8V);

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
	R_CSI_TG_CTRL0 &= ~0XFFFFFF;
	R_CSI_TG_CTRL1 = 0;

	/* register CSI interrupt */
	//vic_irq_register(VIC_CSI, drv_l1_csi_isr);

	/* mclk output */
	drv_l2_sensor_set_mclkout(tvp5150_csi_ops.info[0].mclk);

	/* request SCCB */
	tvp5150_sccb_open();
	DBG_PRINT("Sensor tvp5150 csi interface init completed\r\n");
}

/**
 * @brief   tvp5150 un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void tvp5150_csi_uninit(void)
{
	// disable mclk
	drv_l2_sensor_set_mclkout(MCLK_NONE);

	// csi disable
	R_CSI_TG_CTRL0 &= ~MASK_CSI_CTRL0_CSIEN;	//disable sensor controller
	R_CSI_TG_CTRL1 &= ~MASK_CSI_CTRL1_CLKOEN;	//disable sensor clock out

	// release SCCB
	tvp5150_sccb_close();

	// Turn off LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_3P3V);
}

/**
 * @brief   tvp5150 stream start function
 * @param   info index
 *
 * @return 	none
 */
void tvp5150_csi_stream_start(INT32U index, INT32U bufA, INT32U bufB)
{
	drv_l2_sensor_para_t *pPara;

    pPara = drv_l2_sensor_get_para();

	/* enable CSI output clock for SCCB */
	drv_l2_sensor_set_mclkout(tvp5150_csi_ops.info[index].mclk);

	/* set start frame buffer address */
	if(bufA) {
		drv_l1_csi_set_buf(bufA);
	} else {
		DBG_PRINT("Input frame buffer address error, fail to start %s\r\n", tvp5150_csi_ops.name);
		tvp5150_csi_ops.stream_stop();
	}

    drvl1_csi_input_set(tvp5150_csi_ops.info[index].interface_mode, tvp5150_csi_ops.info[index].interlace_mode, 0, tvp5150_csi_ops.info[index].input_format);
#if 0
/*Interlace*/
	if(tvp5150_csi_ops.info[index].interlace_mode == CSI_INTERLACE) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_INTL;
	} else {
        R_CSI_TG_CTRL0 &= ~(MASK_CSI_CTRL0_INTL |MASK_CSI_CTRL0_FIELDINV);
	}
    DBG_PRINT("interlace_mode: %d\r\n", tvp5150_csi_ops.info[index].interlace_mode);
/*Interface*///++++++++++++++
	if(tvp5150_csi_ops.info[index].interface_mode == CSI_HREF) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_HREF;
	}

	if(tvp5150_csi_ops.info[index].interface_mode == CSI_HSYNC_CCIR_656) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CCIR656;
	}
//----------------------------
    /* set input & output format */
	if(tvp5150_csi_ops.info[index].input_format == V4L2_PIX_FMT_VYUY) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVIN | MASK_CSI_CTRL0_YUVTYPE;
		//R_CSI_TG_CTRL1 |= DELAY_1CLOCK;
	}
#endif
	/* Set Horizontal/Vertical counter reset, Vertical counter incresase selection, sensor owner inerrupt enable, capture/preview mode */
	R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_OWN_IRQ | MASK_CSI_CTRL0_CAP;

    drvl1_csi_input_latch_timing1_set(0x0, 0x0, 0x0, ENUM_CSI_LATCH_DELAY_3_CLOCK);
	drvl1_csi_input_latch_timing2_set(tvp5150_csi_ops.info[index].hsync_active, tvp5150_csi_ops.info[index].vsync_active, tvp5150_csi_ops.info[index].vsync_active, tvp5150_csi_ops.info[index].hsync_active);

#if 0
	if(tvp5150_csi_ops.info[index].hsync_active == MODE_ACTIVE_HIGH) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_HRST | MASK_CSI_CTRL0_VADD;
	}

	if(tvp5150_csi_ops.info[index].vsync_active == MODE_ACTIVE_LOW) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_VRST;
	} else {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_FGET;
	}

#endif
	if(tvp5150_csi_ops.info[index].output_format == V4L2_PIX_FMT_VYUY) {
		R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_YUVOUT;
		R_CSI_TG_CTRL1 |= MASK_CSI_CTRL1_RGB1555;
	}

	/* Set sensor's registers via SCCB */
	DBG_PRINT("%s = %d\r\n", __func__, index);
	switch(index)
	{

	case 0:
        if(tvp5150_init() < 0) {
            DBG_PRINT("tvp5150 init fail!!!\r\n");
		}

		R_CSI_TG_HWIDTH = tvp5150_csi_ops.info[index].target_w;
		R_CSI_TG_VHEIGHT = tvp5150_csi_ops.info[index].target_h;
		R_CSI_TG_HRATIO = 0;		// No Scale
		R_CSI_TG_VRATIO = 0;		// No Scale
		break;

	case 1:
        if(tvp5150_init() < 0) {
            DBG_PRINT("tvp5150 init fail!!!\r\n");
		}

		R_CSI_TG_HWIDTH = tvp5150_csi_ops.info[index].target_w;
		R_CSI_TG_VHEIGHT = tvp5150_csi_ops.info[index].target_h * 2;
		R_CSI_TG_HRATIO = 0;		// No Scale
		R_CSI_TG_VRATIO = 0;		// No Scale
		break;

	default:
		while(1);
	}

    if(pPara->pscaler_src_mode == MODE_PSCALER_SRC_CSI)
        drvl1_csi_input_pscaler_set(1);
	/* csi long burst width need 32 align */
	if(tvp5150_csi_ops.info[index].target_w & 0x1F) {
		R_CSI_SEN_CTRL &= ~(1 << 7);
	} else {
		R_CSI_SEN_CTRL |= 1 << 7;
	}

	/* Enable interrupt */
    drv_l1_csi_set_irq(ENABLE);

	/* Clear sensor interrupt status */
	R_TGR_IRQ_STATUS = 0x3F;

	/* enable frame end interrupt */
	R_TGR_IRQ_EN |= MASK_CSI_FRAME_END_FLAG|MASK_CSI_FIFO_OVERFLOW_FLAG;

	/* enable CSI module */
	R_CSI_TG_CTRL0 |= MASK_CSI_CTRL0_CSIEN;
}

/**
 * @brief   tvp5150 stream stop function
 * @param   none
 * @return 	none
 */
void tvp5150_csi_stream_stop(void)
{
	/* Disable interrupt */
	drv_l1_csi_set_irq(DISABLE);

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
 * @brief   tvp5150 get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
drv_l2_sensor_info_t* tvp5150_csi_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1))
		return NULL;
	else
		return (drv_l2_sensor_info_t*)&tvp5150_csi_ops.info[index];
}

void sensor_register_ae_ctrl(INT32U *handle)
{
    *handle = NULL;
}


void sensor_get_ae_info(sensor_exposure_t *si)
{
    //memcpy(si, &seInfo, sizeof(sensor_exposure_t));
}



/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t tvp5150_csi_ops =
{
	TVIN_TVP5150_CSI_NAME,				/* sensor name */
	tvp5150_csi_init,
	tvp5150_csi_uninit,
	tvp5150_csi_stream_start,
	tvp5150_csi_stream_stop,
	tvp5150_csi_get_info,
	{
		/* 1nd info */
		{
			MCLK_24M,					/* CSI clock */
			CSI_IN_UYVY_INVERT_UV7,			/* input format */
			V4L2_PIX_FMT_YUYV,//V4L2_PIX_FMT_VYUY,			/* output format */
			CSI_SENSOR_FPS_30,			/* FPS in sensor */
			720,						/* target width */
			240,//480,		 				/* target height */
			720,						/* sensor width */
			240,//480, 						/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			CSI_HSYNC_CCIR_656,				/* input interface */
			CSI_NON_INTERLACE,				/* interlace mode */
			ENUM_CSI_FALLING_EDGE,			/* hsync pin active level */
			ENUM_CSI_RISING_EDGE,			/* vsync pin active level */
		},
		/* 2st info */
		{
			MCLK_24M,					/* CSI clock */
			CSI_IN_UYVY_INVERT_UV7,			/* input format */
			V4L2_PIX_FMT_VYUY,			/* output format */
			CSI_SENSOR_FPS_30,			/* FPS in sensor */
			720,						/* target width */
			576,		 				/* target height */
			720,						/* sensor width */
			576, 						/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			CSI_HSYNC_CCIR_656,				/* input interface */
			CSI_NON_INTERLACE,				/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_LOW,			/* vsync pin active level */
		},
	}
};
#endif //(defined _SENSOR_TVP5150_CSI) && (_SENSOR_TVP5150_CSI == 1)
