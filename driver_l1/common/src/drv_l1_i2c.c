/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2015 by Generalplus Inc.                         *
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
#include "project.h"
#include "drv_l1_clock.h"
#include "drv_l1_sfr.h"
#include "drv_l1_i2c.h"

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined _DRV_L1_I2C) && (_DRV_L1_I2C == 1)
/****************************************************
*		Register bitwise definition 				*
****************************************************/
/********************* Define R_I2C_ICCR bit mask (0xC00B0000, I2C control register) *****************/
#define MASK_I2C_CR_ACKEN		(1 << 7)		/* I2C-bus acknowledge enable */
#define MASK_I2C_CR_CLKSELPRE	(1 << 6)		/* Source clock of I2C-bus transmit clock prescaler selection bit */
#define MASK_I2C_CR_INTREN		(1 << 5)		/* I2C-bus Tx/Rx interrupt enable */
#define MASK_I2C_CR_INTRPEND	(1 << 4)		/* I2C-bus Tx/Rx interrupt pending flag */

/********************* Define R_I2C_ICSR bit mask (0xC00B0004, I2C control/status register) *****************/
#define MASK_I2C_SR_BUSYSTA		(1 << 5)		/* I2C-bus busy signal status bit, when write 1: START signal generation, 0: STOP signal generation */
#define MASK_I2C_SR_DOEN		(1 << 4)		/* I2C-bus data output enable/disable bit */
#define MASK_I2C_SR_APBIT		(1 << 3)		/* I2C-bus arbitration procedure status flag bit */
#define MASK_I2C_SR_ASBIT		(1 << 2)		/* I2C-bus address as slave status flag bit */
#define MASK_I2C_SR_AZBIT		(1 << 1)		/* I2C-bus address zero status flag bit */
#define MASK_I2C_SR_LRBIT		(1 << 0)		/* I2C-bus last received bit status flag bit. 0: ACK received, 1: ACK not received */

/********************* Define R_I2C_MISC bit mask (0xC00B0018, I2C miscellaneous control register) *****************/
#define MASK_I2C_MISC_IGNOREACK	(1 << 1)		/* I2C-bus ignore ACK control bit */
#define MASK_I2C_MISC_GPIOEN	(1 << 0)		/* I2C-bus output via GPIO enable */

/****************************************************
*		Definition 									*
****************************************************/
#define I2C_RESTART_WITHOUT_STOP     0x01
#define DBG_I2C_ENABLE		         0

#if DBG_I2C_ENABLE == 1
#define DRV_I2C_DBG		DBG_PRINT
#else
#define DRV_I2C_DBG(...)
#endif

#define DBG_I2C_PIN_EN               0
#define I2C_CLOCK_SOURCE             8
#define I2C_CLOCK_ENABLE             1
#define I2C_CLOCK_DISABLE            0
/****************************************************
*		varaible and data declaration				*
****************************************************/
typedef struct i2cReg_s
{
	volatile INT32U	ICCR;		//0xC00B0000
	volatile INT32U	ICSR;	    //0xC00B0004
	volatile INT32U	IAR;	    //0xC00B0008
	volatile INT32U	IDSR;	    //0xC00B000C
	volatile INT32U	IDEBCLK;	//0xC00B0010
	volatile INT32U	TXCLKLSB;	//0xC00B0014
    volatile INT32U	MISC;       //0xC00B0018
} i2cReg_t;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
static i2cReg_t* _i2c_select_sfr_base(INT32S DevNo);
static void _i2c_msec_wait(INT32U msec);
static INT32S _i2c_busy_waiting(i2cReg_t *pI2CDev, INT32S ms);
static void _i2cSetClkRate(i2cReg_t *pI2CDev, INT16U clkRate);
static INT32S _i2cStartTran(i2cReg_t *pI2CDev, INT16U slaveAddr, INT16U clkRate, INT8U cmd, INT8U aAck);
static void _i2cStopTran(i2cReg_t *pI2CDev, INT8U cmd);
static INT32S _i2cMiddleTran(i2cReg_t *pI2CDev, INT8U *data, INT8U cmd, INT8U aAck);
static INT32S _i2c_bus_xfer(drv_l1_i2c_bus_handle_t *hd, INT8U* data, INT8U len, INT8U cmd);

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static INT16S i2c_clock_set(INT16U enable)
{
    return drv_l1_clock_set_system_clk_en(I2C_CLOCK_SOURCE, enable);
}

static i2cReg_t* _i2c_select_sfr_base(INT32S DevNo)
{
    if(DevNo == I2C_0) {
        return (i2cReg_t *)I2C0_BASE;
    }
    else if(DevNo == I2C_1) {
        return (i2cReg_t *)I2C1_BASE;
    }
    else if(DevNo == I2C_2) {
        return (i2cReg_t *)I2C2_BASE;
    }

	return 0;
}

static void _i2c_msec_wait(INT32U msec)
{
#if 1
	INT32U cnt = msec * (SystemCoreClock >> 13); //0.64ms
	//volatile INT32U i;
    INT32U i;

	for(i=0; i<cnt; i++) {
		i = i;
	}
#else
    #if 1
    osDelay(msec);
    #else
	drv_msec_wait(msec);
    #endif
#endif
}

static INT32S _i2c_busy_waiting(i2cReg_t *pI2CDev, INT32S ms)
{
	INT32S timeout = ms * 75;

	while((pI2CDev->ICSR & MASK_I2C_SR_BUSYSTA))
	{
		timeout--;
		if(timeout < 0) {
			return -1;
		}
		_i2c_msec_wait(1);
	}

	return 0;
}

static INT32S _i2c_intpend_waiting(i2cReg_t *pI2CDev, INT32S ms)
{
	INT32S timeout = ms * 75;

	while((pI2CDev->ICCR & MASK_I2C_CR_INTRPEND) == 0)
	{
		timeout--;
		if(timeout < 0) {
			return -1;
		}
		_i2c_msec_wait(1);
	}

	return 0;
}

static void _i2cSetClkRate(i2cReg_t *pI2CDev, INT16U clkRate)
{
	//==================================================================================
	// I2C_Clock = SYSCLK /( { ( TXCLKPRE[3:0], TXCLKLSB[7:0] ) + 1 } * 2)
	// ( TXCLKPRE[3:0], TXCLKLSB[7:0] ) means [4 + 8 = 12] bits combined as following
	//
	// -------------------------------------------------
	// | TXCLKPRE[3:0] |         TXCLKLSB[7:0]         |
	// -------------------------------------------------
	// | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	// -------------------------------------------------
	//
	// ( TXCLKPRE[3:0], TXCLKLSB[7:0] ) = ( SYSCLK / 2 / I2C_Clock ) - 1
	//==================================================================================
	INT32U tmp = 0;

	tmp = SystemCoreClock / (2*clkRate*1000);
	if (tmp > 0xFFF) {
		tmp = 0xFFF;
	} else if (tmp > 0) {
		tmp--;
	}

	// Setup TXCLKLSB[7:0]
	pI2CDev->TXCLKLSB = (tmp & 0xFF);

#if DBG_I2C_PIN_EN == 1
    pI2CDev->IDEBCLK =  ((tmp / 5) & 0xFF);
#endif

	// Setup TXCLKPRE[3:0] in ICCR
	pI2CDev->ICCR &= ~0xF;
	pI2CDev->ICCR |= ((tmp & 0xF00) >> 8);
}

static INT32S _i2cStartTran(i2cReg_t *pI2CDev, INT16U slaveAddr, INT16U clkRate, INT8U cmd, INT8U aAck)
{
	INT32S ret = 1;
	INT32U iccr = 0, ctrl = 0;
	INT32S timeout;

	/* check i2c bus is idle or not */
	ret = _i2c_busy_waiting(pI2CDev, I2C_TIME_OUT);
	if (ret < 0) {
		DRV_I2C_DBG("[%s]-- I2C bus is busy\r\n", __func__);
		return -1;
	}

	_i2cSetClkRate(pI2CDev, clkRate);

	iccr = pI2CDev->ICCR;
	//iccr = (iccr & I2C_ICCR_TXCLKMSB_MASK) | MASK_I2C_CR_INTREN;
	// [TODO]
	// TXCLKMSB_MASK was set in _i2cSetClkRate() and I2C does not use interrupt.
	// The following line should be un-necessary.
	//												by Craig 2012.6.28
	iccr &= I2C_ICCR_TXCLKMSB_MASK | MASK_I2C_CR_INTREN;

	switch (cmd)
	{
		case I2C_BUS_WRITE:
			iccr |= I2C_ICCR_INIT;
			ctrl = I2C_ICSR_MASTER_TX | MASK_I2C_SR_DOEN;
			break;

		case I2C_BUS_READ:
			iccr |= MASK_I2C_CR_ACKEN;
			ctrl = I2C_ICSR_MASTER_RX | MASK_I2C_SR_DOEN;
			break;

		default:
			return -1;
			break;
	}

	pI2CDev->ICCR = iccr;
	pI2CDev->ICSR = ctrl;

	if (cmd == I2C_BUS_READ) {
		pI2CDev->IDSR = (slaveAddr & 0xFF) | 0x01;
	}
	else {
		pI2CDev->IDSR = slaveAddr & 0xFE;
	}

    // I2C Start to Send Start Signal
	pI2CDev->ICSR |= MASK_I2C_SR_BUSYSTA;

	ret = _i2c_intpend_waiting(pI2CDev, I2C_TIME_OUT);
	if (ret < 0) {
		DRV_I2C_DBG("[%s]-- Interrupt is not received\r\n", __func__);
		return -1;
	}

	timeout = NO_ACK_TIMEOUT * 75;
	while ((pI2CDev->ICSR & MASK_I2C_SR_LRBIT) && aAck)
	{
		// Waiting for ACK
		timeout--;
		if(timeout < 0) {
			DRV_I2C_DBG("[%s]-- Waiting ACK timeout\r\n", __func__);
			ret = -1;
			break;
		}
		_i2c_msec_wait(1);
	}
	return ret;
}

static INT32S _i2cBusRestartTran(i2cReg_t *pI2CDev, INT16U slaveAddr)
{
	INT32U ctrl = 0;
	INT32U iccr = 0;
	INT32S ret = 0;

	iccr = pI2CDev->ICCR;
	iccr &= I2C_ICCR_TXCLKMSB_MASK | MASK_I2C_CR_INTREN;

    pI2CDev->IDSR = (slaveAddr & 0xFF) | 0x01;

    iccr |= MASK_I2C_CR_ACKEN;
    ctrl = MASK_I2C_SR_BUSYSTA | I2C_ICSR_MASTER_RX | MASK_I2C_SR_DOEN;

	pI2CDev->ICCR = iccr;
	pI2CDev->ICSR = ctrl;

	ret = _i2c_intpend_waiting(pI2CDev, I2C_TIME_OUT);
	if (ret < 0) {
		DRV_I2C_DBG("[%s]-- Interrupt is not received\r\n", __func__);
		return -1;
	}

	return ret;
}

static void _i2cStopTran(i2cReg_t *pI2CDev, INT8U cmd)
{
	INT32S ret=0;
	INT32U ctrl = 0;

	if (cmd == I2C_BUS_WRITE) {
		ctrl = I2C_ICSR_MASTER_TX | MASK_I2C_SR_DOEN;
	} else {
		ctrl = I2C_ICSR_MASTER_RX | MASK_I2C_SR_DOEN;
	}

	pI2CDev->ICSR = ctrl;

	// Stop transmition
	ret = _i2c_busy_waiting(pI2CDev, I2C_TIME_OUT);
	if (ret < 0) {
		DRV_I2C_DBG("[%s]-- I2C bus is busy\r\n", __func__);
		return;
	}
}

static INT32S _i2cMiddleTran(i2cReg_t *pI2CDev, INT8U *data, INT8U cmd, INT8U aAck)
{
	INT32S ret = 1;
	INT32U iccr = 0;
	INT32S timeout;

	iccr = pI2CDev->ICCR;
	iccr &= I2C_ICCR_TXCLKMSB_MASK;

	switch (cmd)
	{
		case I2C_BUS_WRITE:
			pI2CDev->IDSR = *data & 0xFF;
			iccr |= MASK_I2C_CR_INTRPEND;
			break;

		case I2C_BUS_READ:
            if (aAck == 1) {
				iccr |= MASK_I2C_CR_ACKEN | MASK_I2C_CR_INTRPEND;
            }
            else {
				iccr |= MASK_I2C_CR_INTRPEND;
            }
            break;

		default:
			return -1;
			break;
	}

	/* clear irq */
	pI2CDev->ICCR = iccr;

	// Interrupt Pending
	ret = _i2c_intpend_waiting(pI2CDev, I2C_TIME_OUT);
	if (ret < 0) {
		DRV_I2C_DBG("[%s]-- Interrupt is not received\r\n", __func__);
		return -1;
	}

	timeout = NO_ACK_TIMEOUT * 75;
	// Ack Received ?
	while ((pI2CDev->ICSR & MASK_I2C_SR_LRBIT) && aAck)
	{
		// Waiting for ACK
		timeout--;
		if(timeout < 0) {
			DRV_I2C_DBG("[%s]-- Waiting ACK timeout\r\n", __func__);
			ret = -1;
			break;
		}
		_i2c_msec_wait(1);
	}

	*data = (pI2CDev->IDSR & 0xFF);
	return ret;
}

static INT32S _i2c_bus_xfer(drv_l1_i2c_bus_handle_t *hd, INT8U* data, INT8U len, INT8U cmd)
{
	INT32S i=0, ret = -1;
    i2cReg_t *pI2CDev = _i2c_select_sfr_base(hd->devNumber);

	pI2CDev->MISC |= 0x1;
	if (hd->clkRate == 0) {
		DRV_I2C_DBG("[%s]-- Error: i2c clock rate must to be more than zero\r\n", __func__);
		goto __exit;
	}

	if (cmd == I2C_BUS_WRITE) {
		ret = _i2cStartTran(pI2CDev, hd->slaveAddr, hd->clkRate, I2C_BUS_WRITE, 0);
		if (ret < 0) {
			DRV_I2C_DBG("WRITE-Error: write slave device address fail\r\n", __func__);
			goto __exit;
		}

		for (i = 0; i < len; i++)
		{
			if (i==(len-1)) {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_WRITE, 0);
			}
            else {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_WRITE, 1);
			}

			if (ret < 0) {
				DRV_I2C_DBG("WRITE-Error: write data fail\r\n", __func__);
				goto __exit;
			}
		}

		_i2cStopTran(pI2CDev, I2C_BUS_WRITE);
	}
	else if (cmd == I2C_BUS_READ) {
		ret = _i2cStartTran(pI2CDev, hd->slaveAddr, hd->clkRate, I2C_BUS_READ, 1);
		if (ret < 0) {
			DRV_I2C_DBG("READ-Error: write slave device address fail\r\n", __func__);
			goto __exit;
		}

		for (i = 0; i < len; i++)
		{
			if (i == (len-1)) {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_READ, 0);
			}
            else {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_READ, 1);
			}

			if (ret < 0) {
				DRV_I2C_DBG("READ-Error: read data fail\r\n", __func__);
				goto __exit;
			}
		}

		_i2cStopTran(pI2CDev, I2C_BUS_READ);
	}

	ret = i;
__exit:
    pI2CDev->MISC &= ~0x1;

	return ret;
}

/**
 * @brief   I2C bus data transfer function
 * @param   handle[in]: i2c bus handle
 * @param   subaddr[in]: sub-address buffer
 * @param   length[in]: sub-address length
 * @param   data[in]: data buffer
 * @param	data_len[in]: data length
 * @param   cmd[in]: I2C_BUS_WRITE/I2C_BUS_READ
 * @param	mode[in]:i2c restart without stop or not: 1、I2C_RESTART_WITHOUT_STOP 0、I2C_RESTART_WITH_STOP
 * @return  data length/ERROR_ID
 */
static int i2c_multi_xfer(drv_l1_i2c_bus_handle_t *hd,  INT8U* subaddr, INT32U length, INT8U* data, INT32U data_len, INT8U cmd, INT32U mode)
{
    #define I2C_MULTI_WRITE_MODE        1
    #define I2C_MULTI_READ_MODE         1
	INT32S i=0, ret = -1;
    i2cReg_t *pI2CDev = _i2c_select_sfr_base(hd->devNumber);

	pI2CDev->MISC |= 0x1;
	if (hd->clkRate == 0) {
		DRV_I2C_DBG("[%s]-- Error: i2c clock rate must to be more than zero\r\n", __func__);
		goto __exit;
	}

	if (cmd == I2C_BUS_WRITE) {
#if I2C_MULTI_WRITE_MODE == 1
        ret = _i2cStartTran(pI2CDev, hd->slaveAddr, hd->clkRate, I2C_BUS_WRITE, 1);
#else
		ret = _i2cStartTran(pI2CDev, hd->slaveAddr, hd->clkRate, I2C_BUS_WRITE, 0);
#endif
		if (ret < 0) {
			DRV_I2C_DBG("WRITE-Error: write slave device address fail\r\n", __func__);
			goto __exit;
		}

		for (i = 0; i < length; i++)
		{
#if I2C_MULTI_WRITE_MODE == 1
			if (i==(length-1)) {
				ret = _i2cMiddleTran(pI2CDev, (subaddr+i), I2C_BUS_WRITE, 0);
			}
            else {
				ret = _i2cMiddleTran(pI2CDev, (subaddr+i), I2C_BUS_WRITE, 1);
			}
#else
            ret = _i2cMiddleTran(pI2CDev, (subaddr+i), I2C_BUS_WRITE, 0);
#endif

			if (ret < 0) {
				DRV_I2C_DBG("WRITE-Error: write subaddr fail\r\n", __func__);
				goto __exit;
			}
		}

		for (i = 0; i < data_len; i++)
		{
#if I2C_MULTI_WRITE_MODE == 1
			if (i==(data_len-1)) {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_WRITE, 0);
			}
            else {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_WRITE, 1);
			}
#else
            ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_WRITE, 0);
#endif
			if (ret < 0) {
				DRV_I2C_DBG("WRITE-Error: write data fail\r\n", __func__);
				goto __exit;
			}
		}

		_i2cStopTran(pI2CDev, I2C_BUS_WRITE);
	}
	else if (cmd == I2C_BUS_READ) {
		ret = _i2cStartTran(pI2CDev, hd->slaveAddr, hd->clkRate, I2C_BUS_WRITE, 1);
		if (ret < 0) {
			DRV_I2C_DBG("READ-Error: write slave device address fail\r\n", __func__);
			goto __exit;
		}

		for (i = 0; i < length; i++)
		{
#if I2C_MULTI_READ_MODE == 1
			if (i == (length-1)) {
				ret = _i2cMiddleTran(pI2CDev, (subaddr+i), I2C_BUS_WRITE, 0);
			}
            else {
				ret = _i2cMiddleTran(pI2CDev, (subaddr+i), I2C_BUS_WRITE, 1);
			}
#else
            ret = _i2cMiddleTran(pI2CDev, (subaddr+i), I2C_BUS_WRITE, 0);
#endif
			if (ret < 0) {
				DRV_I2C_DBG("READ-Error: read subaddr fail\r\n", __func__);
				goto __exit;
			}
		}

		if(mode == I2C_RESTART_WITHOUT_STOP) {
			ret = _i2cBusRestartTran(pI2CDev, hd->slaveAddr);
			if (ret < 0) {
				DRV_I2C_DBG("ERROR: restart signal generate fail\n");
				goto __exit;
			}
		} else {
			_i2cStopTran(pI2CDev, I2C_BUS_WRITE);

			ret = _i2cStartTran(pI2CDev, hd->slaveAddr, hd->clkRate, I2C_BUS_READ, 1);
			if (ret < 0) {
				DRV_I2C_DBG("ERROR: read slave device address fail\n");
				goto __exit;
			}
		}

		for (i = 0; i < data_len; i++)
		{
#if I2C_MULTI_READ_MODE == 1
			if (i == (data_len-1)) {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_READ, 0);
			}
            else {
				ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_READ, 1);
			}
#else
            ret = _i2cMiddleTran(pI2CDev, (data+i), I2C_BUS_READ, 0);
#endif
			if (ret < 0) {
				DRV_I2C_DBG("READ-Error: read data fail\r\n", __func__);
				goto __exit;
			}
		}

		_i2cStopTran(pI2CDev, I2C_BUS_READ);
	}

	ret = i;
__exit:
    pI2CDev->MISC &= ~0x1;

	return ret;
}

/*****************************************
*			I2C APIs    				 *
*****************************************/
/**
 * @brief   driver layer 1 I2C initilization
 * @param   none
 * @return 	none
 */
void drv_l1_i2c_init(INT32U DevNo)
{
    i2cReg_t *pI2CDev = _i2c_select_sfr_base(DevNo);

	i2c_clock_set(I2C_CLOCK_ENABLE);
	pI2CDev->MISC = MASK_I2C_MISC_GPIOEN;
	pI2CDev->ICCR = I2C_ICCR_INIT;
	pI2CDev->IDEBCLK = I2C_IDEBCLK_INIT;
}

/**
 * @brief   driver layer 1 I2C un-initilization
 * @param   none
 * @return 	none
 */
void drv_l1_i2c_uninit(INT32U DevNo)
{
    i2cReg_t *pI2CDev = _i2c_select_sfr_base(DevNo);

	pI2CDev->MISC &= ~MASK_I2C_MISC_GPIOEN;
	i2c_clock_set(I2C_CLOCK_DISABLE);
}

/**
 * @brief   I2C bus write function
 * @param   handle: hanle for I2C clock and slave address
 * @param   data: data want to write
 * @param   len: data length
 * @return 	0(successful) or -1(failed)
 */
INT32S drv_l1_i2c_bus_write(drv_l1_i2c_bus_handle_t *handle, INT8U* data, INT8U len)
{
	return _i2c_bus_xfer(handle, data, len, (INT8U)I2C_BUS_WRITE);
}
/**
 * @brief   I2C bus read function
 * @param   handle: hanle for I2C clock and slave address
 * @param   data: the buffer where I2C write the read data into
 * @param   len: data length
 * @return 	0(successful) or -1(failed)
 */
INT32S drv_l1_i2c_bus_read(drv_l1_i2c_bus_handle_t *handle, INT8U* data, INT8U len)
{
	return _i2c_bus_xfer(handle, data, len, (INT8U)I2C_BUS_READ);
}

/**
 * @brief   I2C bus write 1byte then write 1 bytes
 * @param   handle: hanle for I2C clock and slave address
 * @param   reg: the first data want to write
 * @param   value: the second data wnat to write
 * @return 	0(successful) or -1(failed)
 */

INT32S drv_l1_reg_1byte_data_1byte_write(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT8U value)
{
	INT8U data[2]={0};

	data[0] = reg & 0xFF;
	data[1] = value & 0xFF;
	DRV_I2C_DBG("[%s]-- addr=0x%02X, data=0x%02X\r\n", __func__, reg, value);

	return drv_l1_i2c_bus_write(handle, data, 2);
}

/**
 * @brief   I2C bus write 1byte then read 1 bytes
 * @param   handle: hanle for I2C clock and slave address
 * @param   reg: the data want to write
 * @param   value: the buffer where I2C write the read data into
 * @return 	0(successful) or -1(failed)
 */
INT32S drv_l1_reg_1byte_data_1byte_read(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT8U *value)
{
	INT32S ret=0;
	INT8U addr[1]={0}, data[1]={0};

	addr[0] = reg & 0xFF;

	ret = drv_l1_i2c_bus_write(handle, addr, 1);
	ret = drv_l1_i2c_bus_read(handle, data, 1);
	*value = data[0];
	DRV_I2C_DBG("[%s]-- addr=0x%02X, data=0x%02X\r\n", __func__, reg, data[0]);

	return ret;
}

/**
 * @brief   I2C bus write 1byte then write 2 bytes
 * @param   handle: hanle for I2C clock and slave address
 * @param   reg: the first data want to write
 * @param   value: contains 2 bytes data want to write
 * @return 	0(successful) or -1(failed)
 */
INT32S drv_l1_reg_1byte_data_2byte_write(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT16U value)
{
	INT8U data[3]={0};

	data[0] = reg & 0xFF;
	data[1] = (value >> 8) & 0xFF;
	data[2] = value & 0xFF;
	DRV_I2C_DBG("[%s]-- addr=0x%02X, data=0x%04X\r\n", __func__, reg, value);

	return drv_l1_i2c_bus_write(handle, data, 3);
}

/**
 * @brief   I2C bus write 1byte then read 2 bytes
 * @param   handle: hanle for I2C clock and slave address
 * @param   reg: the first data want to write
 * @param   value: the buffer where I2C write the read 2 bytes data into
 * @return 	0(successful) or -1(failed)
 */
INT32S drv_l1_reg_1byte_data_2byte_read(drv_l1_i2c_bus_handle_t *handle, INT8U reg, INT16U *value)
{
	INT32S ret=0;
	INT8U addr[1]={0}, data[2]={0};

	addr[0] = reg & 0xFF;

	ret = drv_l1_i2c_bus_write(handle, addr, 1);
	ret = drv_l1_i2c_bus_read(handle, data, 2);
	*value = (((INT16U)data[0]) << 8) | (data[1]);
	DRV_I2C_DBG("[%s]-- addr=0x%02X, data=0x%04X\r\n", __func__, reg, *value);

	return ret;
}

/**
 * @brief   I2C bus write 2byte then write 1 bytes
 * @param   handle: hanle for I2C clock and slave address
 * @param   reg: contains 2 bytes data want to write
 * @param   value: the third byte want to write
 * @return 	0(successful) or -1(failed)
 */
INT32S drv_l1_reg_2byte_data_1byte_write(drv_l1_i2c_bus_handle_t *handle, INT16U reg, INT8U value)
{
	INT8U data[3]={0};

	data[0] = (reg>>8) & 0xFF;
	data[1] = reg & 0xFF;
	data[2] = value & 0xFF;
	DRV_I2C_DBG("[%s]-- addr=0x%04X, data=0x%02X\r\n", __func__, reg, value);

	return drv_l1_i2c_bus_write(handle, data, 3);
}

/**
 * @brief   I2C bus write 2byte then read 1 bytes
 * @param   handle: hanle for I2C clock and slave address
 * @param   reg: contains 2 bytes data want to write
 * @param   value: the buffer where I2C write the read data into
 * @return 	0(successful) or -1(failed)
 */
INT32S drv_l1_reg_2byte_data_1byte_read(drv_l1_i2c_bus_handle_t *handle, INT16U reg, INT8U *value)
{
	INT32S ret=0;
	INT8U addr[2]={0}, data[1]={0};

	addr[0] = (reg>>8) & 0xFF;
	addr[1] = reg & 0xFF;

	ret = drv_l1_i2c_bus_write(handle, addr, 2);
	ret = drv_l1_i2c_bus_read(handle, data, 1);
	*value = data[0];
	DRV_I2C_DBG("[%s]-- addr=0x%04X, data=0x%02X\r\n", __func__, reg, *value);

	return ret;
}

/**
 * @brief   I2C bus write function
 * @param   handle[in]: i2c bus handle
 * @param   subaddr[in]: sub-address buffer
 * @param   length[in]: sub-address length
 * @param   data[in]: data buffer
 * @param	data_len[in]: data length
 * @param	mode[in]:i2c restart without stop or not: 1、I2C_RESTART_WITHOUT_STOP 0、I2C_RESTART_WITH_STOP
 * @return  data length/ERROR_ID
 */
INT32S drv_l1_i2c_multi_write(drv_l1_i2c_bus_handle_t *handle, INT8U *subaddr, INT32U length, INT8U *data, INT32U data_len, INT32U mode)
{
	return	i2c_multi_xfer(handle, subaddr, length, data, data_len, I2C_BUS_WRITE, mode);
}

/**
 * @brief   I2C bus read function
 * @param   handle[in]: i2c bus handle
 * @param   subaddr[in]: sub-address buffer
 * @param   length[in]: sub-address length
 * @param   data[in]: data buffer
 * @param	data_len[in]: data length
 * @param	mode[in]:i2c restart without stop or not: 1、I2C_RESTART_WITHOUT_STOP 0、I2C_RESTART_WITH_STOP
 * @return  data length/ERROR_ID
 */
INT32S drv_l1_i2c_multi_read(drv_l1_i2c_bus_handle_t *handle, INT8U *subaddr, INT32U length, INT8U *data, INT32U data_len, INT32U mode)
{
	return	i2c_multi_xfer(handle, subaddr, length, data, data_len, I2C_BUS_READ, mode);
}

#endif //(defined _DRV_L1_I2C) && (_DRV_L1_I2C == 1)
