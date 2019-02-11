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
#ifndef __drv_l1_CAN_H__
#define __drv_l1_CAN_H__

/****************************************************
*		include file								*
****************************************************/
#include "drv_l1.h"

/****************************************************
*	Definition	 									*
****************************************************/

//--Master control Register
#define	C_M_EN          		0x00000001
#define	C_ACT_EN        		0x00000004
#define	C_TX_REN        		0x00000008
#define	C_S_RST         		0x00000010
#define	C_TX_ABOR               0x00000020

//--TX/RX Buffer Identifier
#define C_IDE                   0x40000000
#define C_RTR                   0x20000000
#define C_nRTR                  0x00000000

//--RX Buffer Control and Status Register
#define C_RX_BF                 0x00000020
#define C_RX_VLD                0x00000080
#define C_RX_Overwrite          0x00000100
#define C_RX_BIT_ERR            0x00000200
#define C_RX_STUFF_ERR          0x00000400
#define C_RX_CRC_ERR            0x00000800
#define C_RX_FORM_ERR           0x00001000
#define C_TX_ABORT              0x00000100
#define C_TX_BIT_ERR            0x00000200
#define C_TX_STUFF_ERR          0x00000400
#define C_TX_FORM_ERR           0x00001000
#define C_TX_ACK_ERR            0x00002000

#define C_Stand_ID              0x00000000
#define C_Extend_ID             0x40000000

#define C_Byte_DLC              0x00000010
#define C_Byte_D0               0x00000000

#define C_TX0_EN                0x0100
#define C_TX1_EN                0x0200
#define C_RX0_EN                0x4000
#define C_RX1_EN                0x8000

//--Interrupt Source register
#define C_TX_ERR                (0x1<<0)
#define C_RX_ERR                (0x1<<1)
#define C_BUSOFF                (0x1<<2)
#define C_TX_WARN               (0x1<<3)
#define C_RX_WARN               (0x1<<4)
#define C_TX_ERRP               (0x1<<5)
#define C_RX_ERRP               (0x1<<6)
#define C_TX0_COM               (0x1<<8)//0x00000100
#define C_TX1_COM               (0x1<<9)//0x00000200
#define C_RX0_COM               (0x1<<14)//0x00004000
#define C_RX1_COM               (0x1<<15)//0x00008000

#define TX_Buf0                 0
#define TX_Buf1                 1

#define RX_Buf0                 0
#define RX_Buf1                 1

#define TRUE  1
#define FALSE 0

#define BASE_SEG1 2
#define BASE_SEG2 2

#define CAN_TIMEOUT 5000

//144M
//#define CAN_DIV     143  //125K, 144M sysclk
//#define CAN_DIV     35  //500K, 144M sysclk
//#define CAN_DIV     17  //1M, 144M sysclk
//193M
#define CAN_DIV   192 //125K

#define CAN_SEG1    4
#define CAN_SEG2    2

typedef struct
{
	INT32U	*D3toD0_Ptr;
	INT32U	*D7toD4_Ptr;
	INT16U	RX_Basic_ID;
	INT32U	RX_Extend_ID;
	INT16U	RX_Mask_BID;
	INT32U	RX_Mask_EID;
	INT32U	Byte_Order;
	INT8U	RX_Buf_Src;
	INT32U	RX_ExtendID_Set;
}CAN_RX_Struct;

typedef struct
{
	INT32U	D3toD0_data;
	INT32U	D7toD4_data;
	INT16U	TX_Basic_ID;
	INT32U	TX_Extend_ID;
	INT32U	Byte_Order;
	INT8U	TX_Buf_Src;
	INT32U	RemoteTransmitReq;
	INT8U	DataLengthControl;
	INT32U	TX_ExtendID_Set;
}CAN_TX_Struct;

typedef enum {
	CAN_ISR_TXERR = 0,
	CAN_ISR_RXERR,
	CAN_ISR_BUSOFF,
	CAN_ISR_TXWARN,
	CAN_ISR_RXWARN,
	CAN_ISR_TXERRP,
	CAN_ISR_RXERRP,
	CAN_ISR_TX0COM,
	CAN_ISR_TX1COM,
	CAN_ISR_RX0COM,
	CAN_ISR_RX1COM,
	CAN_ISR_MAX
}CAN_ISR_ENUM;


extern void drv_l1_can_rx_byte_order(INT8U byte_order, INT8U RX_buf_src);
extern void drv_l1_can_tx_byte_order(INT8U byte_order , INT8U TX_buf_src);
extern void drv_l1_can_rx_dlc(INT8U DLC_Val, INT8U RX_buf_src);
extern void drv_l1_can_tx_dlc(INT8U DLC_val, INT8U TX_buf_src);
extern void drv_l1_can_rx_wait_complete( INT8U RX_buf_src);
extern void drv_l1_can_tx_wait_complete(INT8U TX_buf_src);
extern INT32U drv_l1_can_rx_chk_status(INT8U RX_buf_src);
extern INT32U drv_l1_can_tx_chk_status(INT8U TX_buf_src);
extern void drv_l1_can_rx_id(INT16U RX_B_ID_Val, INT32U RX_E_ID_Val, INT8U RX_buf_src);
extern void drv_l1_can_tx_id(INT32U TX_B_ID_Val, INT32U TX_E_ID_Val, INT8U TX_buf_src);
extern void drv_l1_can_tx_d3tod0(INT32U TX_D3toD0_Val, INT8U TX_buf_src);
extern void drv_l1_can_tx_d7tod4(INT32U TX_D7toD4_Val, INT8U TX_buf_src);
extern void drv_l1_can_tx_ide_format(INT32U IDE, INT8U TX_buf_src);
extern void drv_l1_can_rx_ide_format(INT32U IDE, INT8U RX_buf_src);
extern void drv_l1_can_rx_id_valid(INT8U RX_ID_Valid_En, INT8U RX_buf_src);
extern void drv_l1_can_rx_data_receive(INT32U* Data0_Val , INT32U* Data1_Val, INT8U RX_buf_src);
extern void drv_l1_can_rx_mask(INT16U RX_BID_Mask_bit, INT32U RX_EID_Mask_bit, INT8U RX_buf_src);
extern void drv_l1_can_rx_buf_free(INT8U en,  INT8U RX_buf_src);
extern void drv_l1_can_tx_en(INT8U TX_buf_src);
extern void drv_l1_can_tx_remote_transmit_request_en(INT32U RTR,INT8U TX_buf_src);

extern void drv_l1_can_init(INT32U pseg2, INT32U pseg1, INT32U div);
extern void drv_l1_can_uninit(void);
extern void drv_l1_can_interrupt_en(INT32U src);
extern void drv_l1_can_tx_ide_format(INT32U IDE,INT8U TX_buf_src);
extern void drv_l1_can_rx_ide_format(INT32U IDE, INT8U RX_buf_src);
extern void drv_l1_can_isr_en(INT32U src);
extern INT32U drv_l1_can_read_once(INT8U RX_buf_src, INT32U Byte_DLC, INT16U B_ID, INT16U E_ID, INT32U* D0toD3, INT32U* D4toD7);
extern INT32U drv_l1_can_write_once(INT8U TX_buf_src, INT32U Byte_DLC,  INT32U RTR, INT16U B_ID, INT16U E_ID, INT32U D0toD3, INT32U D4toD7, INT8U DLC);
extern void drv_l1_can_read(CAN_RX_Struct CAN_Setting);
extern void drv_l1_can_write(CAN_TX_Struct CAN_Setting);
extern void drv_l1_can_register_handler( INT32U handler_idx, void (*handler)(void) );
extern void drv_l1_can_unregister_handler( INT32U handler_idx);

#endif
