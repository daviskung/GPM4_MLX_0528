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
#ifndef __drv_l1_CEC_H__
#define __drv_l1_CEC_H__

/****************************************************
*		include file								*
****************************************************/
#include "drv_l1.h"

/****************************************************
*	Definition	 									*
****************************************************/

#define B_RXBR          (0x1<<0)
#define B_RXEND         (0x1<<1)
#define B_RXOVR         (0x1<<2)
#define B_BRE           (0x1<<3)
#define B_SBPE          (0x1<<4)
#define B_LBPE          (0x1<<5)
#define B_RXACKE        (0x1<<6)
#define B_ARBLST        (0x1<<7)
#define B_TXBR          (0x1<<8)
#define B_TXEND         (0x1<<9)
#define B_TXUDR         (0x1<<10)
#define B_TXERR         (0x1<<11)
#define B_TXACKE        (0x1<<12)

#define B_TX_SOM        (0x1<<1)

#define BRE_STP 		0x1<<0x4
#define BRE_GEN			0x1<<0x5
#define LBPE_GEN		0x1<<0x6
#define BRDNOGEN		0x1<<0x7
#define ERR_BIT_CFG_NONE 0x00

#define SFT_BFRT 0
#define SFT_AFTT 1
#define SFT_AUTO 0
#define SFT_0P5DBP 1
#define SFT_1P5DBP 2
#define SFT_2P5DBP 3
#define SFT_3P5DBP 4
#define SFT_4P5DBP 5
#define SFT_5P5DBP 6
#define SFT_6P5DBP 7

#define CEC_IOC0    0
#define CEC_IOC12   1


typedef enum {
	CEC_ISR_RXBR =0,
	CEC_ISR_RXEND,
	CEC_ISR_RXOVR,
	CEC_ISR_BRE,
	CEC_ISR_SBPE,
	CEC_ISR_LBPE,
	CEC_ISR_RXACKE,
	CEC_ISR_ARBLST,
	CEC_ISR_TXBR,
	CEC_ISR_TXEND,
	CEC_ISR_TXUDR,
	CEC_ISR_TXERR,
	CEC_ISR_TXACKE,
	CEC_ISR_MAX
}CEC_ISR_ENUM;

extern void drv_l1_cec_init(void);
extern void drv_l1_cec_enable(INT8U enable);
extern void drv_l1_cec_listen(INT8U mode);
extern void drv_l1_cec_own_address_config(INT16U own_bit);
extern void drv_l1_cec_sft_set(INT8U option, INT8U sft);
extern void drv_l1_cec_rx_tolerance(INT8U tol);
extern void drv_l1_cec_error_bit_config(INT8U enable_bit);
extern INT32S drv_l1_cec_tx(INT8U data, INT8U eom, INT8U som);
extern void drv_l1_cec_assign_rx_buffer(INT8U *rx_data_ptr, INT32U data_length);
extern void drv_l1_cec_register_handler( INT32U handler_idx, void (*handler)(void) );
extern void drv_l1_cec_unregister_handler( INT32U handler_idx);
extern void drv_l1_cec_interrupt_enable(INT32U enable);
extern INT32U drv_l1_cec_get_rx_count(void);
extern void drv_l1_cec_tx2(INT8U data,INT8U data2);
#endif
