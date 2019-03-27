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
#ifndef __drv_l1_I2C_H__
#define __drv_l1_I2C_H__

/****************************************************
*		include file								*
****************************************************/
#include "drv_l1.h"

/****************************************************
*	Definition	 									*
****************************************************/
#define I2C_TIME_OUT		10	//ms
#define	NO_ACK_TIMEOUT		10	//ms

#define I2C_BUS_WRITE	0
#define I2C_BUS_READ	1

#define I2C_ICCR_INIT				0x00
#define I2C_IDEBCLK_INIT			0x04
#define I2C_ICCR_TXCLKMSB_MASK		0x0F

#define I2C_ICSR_MASTER_TX			0xC0
#define I2C_ICSR_MASTER_RX			0x80
#define I2C_ICSR_SLAVE_TX			0x40
#define I2C_ICSR_SLAVE_RX			0x00

#define I2C_MISC_PINMUX_EN			0x01
#define I2C_MISC_ACK_DONTCARE		0x02

//mode[in]:i2c restart without stop or not: 
// 1:I2C_RESTART_WITHOUT_STOP 0:I2C_RESTART_WITH_STOP

#define TH32x32_I2C_RESTART_MODE	    1



typedef enum
{
    I2C_0,
	I2C_1,
	I2C_2,
    I2C_MAX_NUMS
} I2C_DEVICE;

/****************************************************
*		Data structure 								*
*****************************************************/
typedef struct drv_l1_i2c_bus_handle_s
{
	INT32U		devNumber;
	INT16U      slaveAddr;			/* slave device address */
	INT16U      clkRate;			/* i2c bus clock rate */
} drv_l1_i2c_bus_handle_t;

/****************************************************
*		external function declarations				*
****************************************************/
extern void drv_l1_i2c_init(INT32U DevNo);
extern void drv_l1_i2c_uninit(INT32U DevNo);
extern INT32S drv_l1_i2c_bus_write(drv_l1_i2c_bus_handle_t *handle, INT8U* data, INT8U len);
extern INT32S drv_l1_i2c_bus_read(drv_l1_i2c_bus_handle_t *handle, INT8U* data, INT8U len);
extern INT32S drv_l1_reg_1byte_data_1byte_write(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT8U value);
extern INT32S drv_l1_reg_1byte_data_1byte_read(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT8U *value);
extern INT32S drv_l1_reg_1byte_data_2byte_write(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT16U value);
extern INT32S drv_l1_reg_1byte_data_2byte_read(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT16U *value);
extern INT32S drv_l1_i2c_multi_write(drv_l1_i2c_bus_handle_t *handle, INT8U *subaddr, INT32U length, INT8U *data, INT32U data_len, INT32U mode);
extern INT32S drv_l1_i2c_multi_read(drv_l1_i2c_bus_handle_t *handle, INT8U *subaddr, INT32U length, INT8U *data, INT32U data_len, INT32U mode);
extern INT32S drv_l1_reg_2byte_data_2byte_write(drv_l1_i2c_bus_handle_t *handle, INT16U reg, INT16U value);
extern INT32S drv_l1_reg_2byte_data_2byte_read(drv_l1_i2c_bus_handle_t *handle, INT16U reg, INT16U *value);

#endif	/*__drv_l1_I2C_H__*/
