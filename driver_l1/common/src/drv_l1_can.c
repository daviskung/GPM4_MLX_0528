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
#include "drv_l1_can.h"
#include "drv_l1_clock.h"

#if (defined _DRV_L1_CAN) && (_DRV_L1_CAN == 1)
/**************************************************************************
*                           C O N S T A N T S                            *
**************************************************************************/

#define TRUE  1
#define FALSE 0

/**************************************************************************
*                          D A T A    T Y P E S                          *
**************************************************************************/



/**************************************************************************
*               F U N C T I O N    D E C L A R A T I O N S               *
**************************************************************************/
void drv_l1_can_init(INT32U pseg2, INT32U pseg1, INT32U div);
void drv_l1_can_rx_byte_order(INT8U byte_order, INT8U RX_buf_src);
void drv_l1_can_tx_byte_order(INT8U byte_order , INT8U TX_buf_src);
void drv_l1_can_rx_dlc(INT8U DLC_Val, INT8U RX_buf_src);
void drv_l1_can_tx_dlc(INT8U DLC_val, INT8U TX_buf_src);
void drv_l1_can_rx_wait_complete( INT8U RX_buf_src);
void drv_l1_can_tx_wait_complete(INT8U TX_buf_src);
INT32U drv_l1_can_rx_chk_status(INT8U RX_buf_src);
INT32U drv_l1_can_tx_chk_status(INT8U TX_buf_src);
void drv_l1_can_rx_id(INT16U RX_B_ID_Val, INT32U RX_E_ID_Val, INT8U RX_buf_src);
void drv_l1_can_tx_id(INT32U TX_B_ID_Val, INT32U TX_E_ID_Val, INT8U TX_buf_src);
void drv_l1_can_tx_d3tod0(INT32U TX_D3toD0_Val, INT8U TX_buf_src);
void drv_l1_can_tx_d7tod4(INT32U TX_D7toD4_Val, INT8U TX_buf_src);
void drv_l1_can_tx_ide_format(INT32U IDE, INT8U TX_buf_src);
void drv_l1_can_rx_ide_format(INT32U IDE, INT8U RX_buf_src);
void drv_l1_can_rx_id_valid(INT8U RX_ID_Valid_En, INT8U RX_buf_src);
void drv_l1_can_rx_data_receive(INT32U* Data0_Val , INT32U* Data1_Val, INT8U RX_buf_src);
void drv_l1_can_rx_mask(INT16U RX_BID_Mask_bit, INT32U RX_EID_Mask_bit, INT8U RX_buf_src);
void drv_l1_can_rx_buf_free(INT8U en,  INT8U RX_buf_src);
void drv_l1_can_tx_en(INT8U TX_buf_src);
void drv_l1_can_tx_remote_transmit_request_en(INT32U RTR,INT8U TX_buf_src);
void drv_l1_can_interrupt_en(INT32U src);
void CANBUS_IRQHandler(void);
/**************************************************************************
*                         G L O B A L    D A T A                         *
**************************************************************************/
static void (*can_isr[CAN_ISR_MAX])(void);


/**************************************************************************
* DESCRIPTION: Set RX buffer free bit, controller should clear it after
*              received a valid message, set RX_BF after retrieve data.
*
* PARAMETERS:  en         - TRUE set RX_BF, FALSE clear RX_BF
*              RX_buf_src - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
*
**************************************************************************/
void drv_l1_can_rx_buf_free(INT8U en , INT8U RX_buf_src)
{
    if (RX_buf_src == RX_Buf0)
    {
        if(en == TRUE)
            R_CAN_RXB0_CS |= C_RX_BF;
        else
            R_CAN_RXB0_CS &= ~C_RX_BF;
    }
    if (RX_buf_src == RX_Buf1)
    {
        if(en == TRUE)
            R_CAN_RXB1_CS |= C_RX_BF;
        else
            R_CAN_RXB1_CS &= ~C_RX_BF;
    }
}

static void drv_l1_can_msec_wait(INT32U msec)
{
	INT32U cnt = msec * (SystemCoreClock >> 13); //0.64ms
	INT32U i;

	for(i=0; i<cnt; i++) {
		i = i;
	}
}

/**************************************************************************
* DESCRIPTION: Check and wait RX buffer source get a valid data (RX complete)
*
* PARAMETERS:  RX_buf_src - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
*
**************************************************************************/
void drv_l1_can_rx_wait_complete(INT8U RX_buf_src)
{
    INT32U timeout=0;
    if (RX_buf_src == RX_Buf0)
        while((R_CAN_INT_SRC&C_RX0_COM)!=C_RX0_COM)
        {
            #if _OPERATING_SYSTEM == _OS_FREERTOS
            timeout++;
            if(timeout==CAN_TIMEOUT)
            {
                DBG_PRINT("RX Buffer 0 time out\r\n");
                break;
            }
            osDelay(1);
            #endif
        }
    if (RX_buf_src == RX_Buf1)
        while((R_CAN_INT_SRC&C_RX1_COM)!=C_RX1_COM)
        {
            #if _OPERATING_SYSTEM == _OS_FREERTOS
            timeout++;
            if(timeout==CAN_TIMEOUT)
            {
                DBG_PRINT("RX Buffer 1 time out\r\n");
                break;
            }
            osDelay(1);
            #endif
        }
}

/**************************************************************************
* DESCRIPTION: Check and wait TX buffer source get a valid data (TX complete)
*
* PARAMETERS:  TX_buf_src - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
*
**************************************************************************/
void drv_l1_can_tx_wait_complete(INT8U TX_buf_src)
{
    INT32U timeout=0;
    if (TX_buf_src == TX_Buf0)
        while((R_CAN_INT_SRC&C_TX0_COM)!=C_TX0_COM)
        {
            #if _OPERATING_SYSTEM == _OS_FREERTOS
            timeout++;
            if(timeout==CAN_TIMEOUT)
            {
                DBG_PRINT("TX Buffer 0 time out\r\n");
                break;
            }
            osDelay(1);
            #endif
        }
    if (TX_buf_src == TX_Buf1)
        while((R_CAN_INT_SRC&C_TX1_COM)!=C_TX1_COM)
        {
            #if _OPERATING_SYSTEM == _OS_FREERTOS
            timeout++;
            if(timeout==CAN_TIMEOUT)
            {
                DBG_PRINT("TX Buffer 1 time out\r\n");
                break;
            }
            osDelay(1);
            #endif
        }
}

/**************************************************************************
* DESCRIPTION: Check TX buffer source status
*
* PARAMETERS:  TX_buf_src - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      Status
*              13       12        11  10         9        8
*              ACK_ERR  FORM_ERR  0   STUFF_ERR  BIT_ERR  TX_ABORT
**************************************************************************/
INT32U drv_l1_can_tx_chk_status(INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
        return(R_CAN_TXB0_CS&0x3700);
    if (TX_buf_src == TX_Buf1)
        return(R_CAN_TXB1_CS&0x3700);
}

/**************************************************************************
* DESCRIPTION: Check RX buffer source status
*
* PARAMETERS:  RX_buf_src - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      Status
*              12        11       10         9        8
*              FORM_ERR  CRC_ERR  STUFF_ERR  BIT_ERR  RX_OVER
**************************************************************************/
INT32U drv_l1_can_rx_chk_status(INT8U RX_buf_src)
{
    if (RX_buf_src == RX_Buf0)
        return(R_CAN_RXB0_CS&0x1F00);
    if (RX_buf_src == RX_Buf1)
        return(R_CAN_RXB1_CS&0x1F00);
}

/**************************************************************************
* DESCRIPTION: Set RX byte first byte received order
*
* PARAMETERS:  byte_order - 0: byte0, 1: DLC
*              RX_buf_src - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_rx_byte_order(INT8U byte_order, INT8U RX_buf_src)
{
    if (RX_buf_src == RX_Buf0)
    {
        R_CAN_RXB0_CS &= ~0x010;
        R_CAN_RXB0_CS |= byte_order;
    }
    if (RX_buf_src == RX_Buf1)
    {
        R_CAN_RXB1_CS &= ~0x010;
        R_CAN_RXB1_CS |= byte_order;
    }
}

/**************************************************************************
* DESCRIPTION: Set TX byte first byte transmitted order
*
* PARAMETERS:  byte_order - 0: byte0, 1: DLC
*              TX_buf_src - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_byte_order(INT8U byte_order , INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
    {
        R_CAN_TXB0_CS &= ~0x010;
        R_CAN_TXB0_CS |= byte_order;
    }
    if (TX_buf_src == TX_Buf1)
    {
        R_CAN_TXB1_CS &= ~0x010;
        R_CAN_TXB1_CS |= byte_order;
    }
}

/**************************************************************************
* DESCRIPTION: Get RX data
*
* PARAMETERS:  Data0_Val  - Data bytes 3~0
*              Data1_Val  - Data bytes 7~4
*              RX_buf_src - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_rx_data_receive(INT32U* Data0_Val , INT32U* Data1_Val, INT8U RX_buf_src )
{
    if (RX_buf_src == RX_Buf0)
    {
        *Data0_Val = R_CAN_RXB0_D0;
        *Data1_Val = R_CAN_RXB0_D1;
    }
    if (RX_buf_src == RX_Buf1)
    {
        *Data0_Val = R_CAN_RXB1_D0;
        *Data1_Val = R_CAN_RXB1_D1;
    }
}

/**************************************************************************
* DESCRIPTION: Set RX data length control(DLC), it automatic updated by
*              received data length.
*
* PARAMETERS:  DLC_Val    - Data length 0~8
*              RX_buf_src - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_rx_dlc(INT8U DLC_Val, INT8U RX_buf_src)
{
    if(RX_buf_src == RX_Buf0)
    {
        R_CAN_RXB0_CS &= ~0x0F;
        R_CAN_RXB0_CS |= DLC_Val;
    }
    if(RX_buf_src == RX_Buf1)
    {
        R_CAN_RXB1_CS &= ~0x0F;
        R_CAN_RXB1_CS |= DLC_Val;
    }
}

/**************************************************************************
* DESCRIPTION: Set RX Identifier Valid, controller will filter the received
*              message with the Identifier and ID_MASK.
*              Only matched message will be write to the buffer.
*
* PARAMETERS:  RX_ID_Valid_En - TRUE: Enable ID_MASK, FALSE: Disable
*              RX_buf_src     - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_rx_id_valid(INT8U RX_ID_Valid_En , INT8U RX_buf_src )
{
    if(RX_buf_src == RX_Buf0)
    {
        if(RX_ID_Valid_En == TRUE)
            R_CAN_RXB0_CS |= C_RX_VLD;
        else
            R_CAN_RXB0_CS &= ~C_RX_VLD;
    }
    if(RX_buf_src == RX_Buf1)
    {
        if(RX_ID_Valid_En == TRUE)
            R_CAN_RXB1_CS |= C_RX_VLD;
        else
            R_CAN_RXB1_CS &= ~C_RX_VLD;
    }
}

/**************************************************************************
* DESCRIPTION: Set RX Identifier setting.
*
* PARAMETERS:  RX_B_ID_Val - 11 bits Basic Identifier
*              RX_E_ID_Val - 18 bits Extend Identifier
*              RX_buf_src  - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_rx_id(INT16U RX_B_ID_Val , INT32U RX_E_ID_Val, INT8U RX_buf_src)
{
    if(RX_buf_src == RX_Buf0)
    {
        R_CAN_RXB0_ID &= 0xf0000000;
        if((R_CAN_RXB0_ID&C_IDE)==0x0)  // Basic ID
        {
            R_CAN_RXB0_ID |= RX_B_ID_Val<<18;
        }
        else
        {
            R_CAN_RXB0_ID |= RX_B_ID_Val;
            R_CAN_RXB0_ID |= RX_E_ID_Val<<11;
        }
    }
    if(RX_buf_src == RX_Buf1)
    {
        R_CAN_RXB1_ID &= 0xf0000000;
        if((R_CAN_RXB1_ID&C_IDE)==0x0)  // Basic ID
        {
            R_CAN_RXB1_ID |= RX_B_ID_Val<<18;
        }
        else
        {
            R_CAN_RXB1_ID |= RX_B_ID_Val;
            R_CAN_RXB1_ID |= RX_E_ID_Val<<11;
        }
    }
}

/**************************************************************************
* DESCRIPTION: Set RX Identifier Mask setting. Ignored masked bit and
*              update it by received id.
*
* PARAMETERS:  RX_BID_Mask_bit - 11 bits Basic Mask Identifier
*              RX_EID_Mask_bit - 18 bits Extend Mask Identifier
*              RX_buf_src  - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_rx_mask(INT16U RX_BID_Mask_bit, INT32U RX_EID_Mask_bit, INT8U RX_buf_src)
{
    if(RX_buf_src == RX_Buf0)
    {
        R_CAN_RXB0_MASK = 0;
        if((R_CAN_RXB0_ID&C_IDE)==0x0)  // Basic ID
        {
            R_CAN_RXB0_MASK = RX_BID_Mask_bit << 18;
        }
        else
        {
            R_CAN_RXB0_MASK |= RX_BID_Mask_bit;
            R_CAN_RXB0_MASK |= RX_EID_Mask_bit<<11;
        }
    }
    if(RX_buf_src == RX_Buf1)
    {
        R_CAN_RXB1_MASK = 0;
        if((R_CAN_RXB1_ID&C_IDE)==0x0)  // Basic ID
        {
            R_CAN_RXB1_MASK = RX_BID_Mask_bit << 18;
        }
        else
        {
            R_CAN_RXB1_MASK |= RX_BID_Mask_bit;
            R_CAN_RXB1_MASK |= RX_EID_Mask_bit<<11;
        }
    }
}

/**************************************************************************
* DESCRIPTION: Set TX Identifier setting.
*
* PARAMETERS:  TX_B_ID_Val - 11 bits Basic Identifier
*              TX_E_ID_Val - 18 bits Extend Identifier
*              TX_buf_src  - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_id(INT32U TX_B_ID_Val, INT32U TX_E_ID_Val, INT8U TX_buf_src )
{
    if(TX_buf_src == TX_Buf0)
    {
        R_CAN_TXB0_ID &= 0xf0000000;
        if((R_CAN_TXB0_ID&C_IDE)==0x0)
        {
            R_CAN_TXB0_ID |= TX_B_ID_Val<<18;
        }
        else
        {
            R_CAN_TXB0_ID |= TX_B_ID_Val;
            R_CAN_TXB0_ID |= TX_E_ID_Val<<11;
        }
    }
    if(TX_buf_src == TX_Buf1)
    {
        R_CAN_TXB1_ID &= 0xf0000000;
        if((R_CAN_TXB1_ID&C_IDE)==0x0)
        {
            R_CAN_TXB1_ID |= TX_B_ID_Val<<18;
        }
        else
        {
            R_CAN_TXB1_ID |= TX_B_ID_Val;
            R_CAN_TXB1_ID |= TX_E_ID_Val<<11;
        }
    }
}

/**************************************************************************
* DESCRIPTION: Set TX data byte 3~0
*
* PARAMETERS:  TX_D3toD0_Val - [31:24]Byte3,[23:16]Byte2,[15:8]byte1,
*                              [7:0]byte0
*              TX_buf_src    - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_d3tod0(INT32U TX_D3toD0_Val, INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
        R_CAN_TXB0_D0 = TX_D3toD0_Val;
    if (TX_buf_src == TX_Buf1)
        R_CAN_TXB1_D0 = TX_D3toD0_Val;
}

/**************************************************************************
* DESCRIPTION: Set TX data byte 7~4
*
* PARAMETERS:  TX_D7toD4_Val - [31:24]Byte7,[23:16]Byte6,[15:8]byte5,
*                              [7:0]byte4
*              TX_buf_src    - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_d7tod4(INT32U TX_D7toD4_Val, INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
        R_CAN_TXB0_D1 = TX_D7toD4_Val;
    if (TX_buf_src == TX_Buf1)
        R_CAN_TXB1_D1 = TX_D7toD4_Val;
}

/**************************************************************************
* DESCRIPTION: Set TX data length control
*
* PARAMETERS:  DLC_val    - 0~8 data byte length
*              TX_buf_src - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_dlc(INT8U DLC_val, INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
    {
        R_CAN_TXB0_CS &= ~0x0F;
        R_CAN_TXB0_CS |= DLC_val;
    }
    if (TX_buf_src == TX_Buf1)
    {
        R_CAN_TXB1_CS &= ~0x0F;
        R_CAN_TXB1_CS |= DLC_val;
    }
}

/**************************************************************************
* DESCRIPTION: Set TX remote transmit request (RTR)
*
* PARAMETERS:  RTR        - 0: Data frame, 1: RTR frame
*              TX_buf_src - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_remote_transmit_request_en(INT32U RTR,INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
    {
        R_CAN_TXB0_ID &= ~C_RTR;
        R_CAN_TXB0_ID |= RTR;
    }
    if (TX_buf_src == TX_Buf1)
    {
        R_CAN_TXB1_ID &= ~C_RTR;
        R_CAN_TXB1_ID |= RTR;
    }
}

/**************************************************************************
* DESCRIPTION: Set TX Enable, start transmit.
*
* PARAMETERS:  TX_buf_src - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_en(INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
        R_CAN_TXB0_CS |= 0x80;
    if (TX_buf_src == TX_Buf1)
        R_CAN_TXB1_CS |= 0x80;
}

/**************************************************************************
* DESCRIPTION: Interrupt enable setting
*
* PARAMETERS:  src - each bit enable corresponding interrupt
*
* 15        14        13       12      11      10       9         8
* R1COM_IEN R0COM_IEN                                    T1COM_IEN T0COM_IEN
* 7         6         5        4       3       2        1         0
*           RXEP_IEN  TXEP_IEN RXW_IEN TXW_IEN BOFF_IEN RERR_IEN  TERR_IEN
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_interrupt_en(INT32U src)
{
    NVIC_EnableIRQ(CANBUS_IRQn);
    R_CAN_INT_EN |= src;
}

/**************************************************************************
* CANBUS IRQ Handler
**************************************************************************/
void CANBUS_IRQHandler(void)
{
INT32U Val;

    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT0)){ // TX_ERR
        if(can_isr[CAN_ISR_TXERR])
            (can_isr[CAN_ISR_TXERR])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT1)){// RX_ERR
        if(can_isr[CAN_ISR_RXERR])
            (can_isr[CAN_ISR_RXERR])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT2)){// BUS_OFF
        if(can_isr[CAN_ISR_BUSOFF])
            (can_isr[CAN_ISR_BUSOFF])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT3)){// TX_WARN
        if(can_isr[CAN_ISR_TXWARN])
            (can_isr[CAN_ISR_TXWARN])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT4)){// RX_WARN
        if(can_isr[CAN_ISR_RXWARN])
            (can_isr[CAN_ISR_RXWARN])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT5)){// TX_ERRP
        if(can_isr[CAN_ISR_TXERRP])
            (can_isr[CAN_ISR_TXERRP])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT6)){// RX_ERRP
        if(can_isr[CAN_ISR_RXERRP])
            (can_isr[CAN_ISR_RXERRP])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT8)){// TX0_COM
        if(can_isr[CAN_ISR_TX0COM])
            (can_isr[CAN_ISR_TX0COM])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT9)){// TX1_COM
        if(can_isr[CAN_ISR_TX1COM])
            (can_isr[CAN_ISR_TX1COM])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT14)){// RX0_COM
        if(can_isr[CAN_ISR_RX0COM])
            (can_isr[CAN_ISR_RX0COM])();
    }
    if((R_CAN_INT_EN)&(R_CAN_INT_SRC)&(BIT15)){// RX1_COM
        if(can_isr[CAN_ISR_RX1COM])
            (can_isr[CAN_ISR_RX1COM])();
    }

    Val = R_CAN_INT_EN;
    R_CAN_INT_SRC = Val;
}

/**************************************************************************
* DESCRIPTION: Set TX identity format.
*
* PARAMETERS:  IDE        - C_Stand_ID, C_Extend_ID
*              TX_buf_src - TX buffer source, TX_Buf0/TX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_tx_ide_format(INT32U IDE, INT8U TX_buf_src)
{
    if (TX_buf_src == TX_Buf0)
    {
        R_CAN_TXB0_ID &= ~C_IDE;
        R_CAN_TXB0_ID |= IDE;
    }
    if (TX_buf_src == TX_Buf1)
    {
        R_CAN_TXB1_ID &= ~C_IDE;
        R_CAN_TXB1_ID |= IDE;
    }
}

/**************************************************************************
* DESCRIPTION: Set RX identity format.
*
* PARAMETERS:  IDE        - C_Stand_ID, C_Extend_ID
*              RX_buf_src - RX buffer source, RX_Buf0/RX_Buf1
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_rx_ide_format(INT32U IDE, INT8U RX_buf_src)
{
    if (RX_buf_src == RX_Buf0)
    {
        R_CAN_RXB0_ID &= ~C_IDE;
        R_CAN_RXB0_ID |= IDE;
    }

    if (RX_buf_src == RX_Buf1)
    {
        R_CAN_RXB1_ID &= ~C_IDE;
        R_CAN_RXB1_ID |= IDE;
    }
}

void drv_l1_can_set_bit_timing(INT32U pseg2, INT32U pseg1, INT32U div)
{
    R_CAN_BCTRL = (pseg2 <<16)|(pseg1<<12)|div;
}

//CAN_SEL 0: IOC0/1, 1: IOC2/3
/**************************************************************************
* DESCRIPTION: CAN BUS initialization
*
* PARAMETERS:  NONE
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_init(INT32U pseg2, INT32U pseg1, INT32U div)
{
	//R_SYSTEM_CLK_EN0 |= 0x1<<8;
    drv_l1_clock_set_system_clk_en(CLK_EN0_CEC_CAN_I2C,1);
    R_CAN_MCTRL &= ~C_M_EN;

    R_CAN_MCTRL |= C_S_RST ;  // soft reset
    //   [22]    [21:20]  [19]     [18:16] [15:12] [11:0]
    //   SYNC_2  SYNC_JW  SAMPLE   PSEG2   PSEG1   CLK_DIV

    R_CAN_BCTRL = ((pseg2&0x7)<<16)|((pseg1&0x0F)<<12)|(div&0xFFF);
    //DBG_PRINT("sysclk = %d\r\n",SystemCoreClock);
    //R_CAN_BCTRL = 0x2408F;
    R_CAN_MCTRL |= C_M_EN;   // master enable

}

/**************************************************************************
* DESCRIPTION: CAN BUS initialization
*
* PARAMETERS:  NONE
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_uninit(void)
{

    R_CAN_MCTRL &= ~C_M_EN;

    R_CAN_MCTRL |= C_S_RST ;  // soft reset

    drv_l1_clock_set_system_clk_en(CLK_EN0_CEC_CAN_I2C,0);
	//R_SYSTEM_CLK_EN0 &= ~(0x1<<8);
}

/**************************************************************************
* DESCRIPTION: CAN interrupt isr register
*
* PARAMETERS:  handler_idx  - CAN_ISR_ENUM (0~11)
*              handler      - isr function
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_register_handler( INT32U handler_idx, void (*handler)(void) )
{
    if(handler_idx >= CAN_ISR_MAX)
    {
        DBG_PRINT("over max index\r\n");
        return;
    }
    can_isr[handler_idx] = handler;
}

/**************************************************************************
* DESCRIPTION: CAN interrupt isr unregister
*
* PARAMETERS:  handler_idx  - CAN_ISR_ENUM
*
* RETURN:      NONE
**************************************************************************/
void drv_l1_can_unregister_handler( INT32U handler_idx)
{
    if(handler_idx >= CAN_ISR_MAX)
    {
        DBG_PRINT("over max index\r\n");
        return;
    }
    can_isr[handler_idx] = NULL;
}

INT32U drv_l1_can_write_once(INT8U TX_buf_src, INT32U Byte_DLC,  INT32U RTR, INT16U B_ID, INT16U E_ID, INT32U D0toD3, INT32U D4toD7, INT8U DLC)
{
    INT32U TX_err;

    drv_l1_can_tx_byte_order(Byte_DLC,TX_buf_src);
    drv_l1_can_tx_remote_transmit_request_en(RTR, TX_buf_src);
    drv_l1_can_tx_id(B_ID,E_ID, TX_buf_src);
    drv_l1_can_tx_d3tod0(D0toD3, TX_buf_src);
    drv_l1_can_tx_d7tod4(D4toD7, TX_buf_src);
    drv_l1_can_tx_dlc(DLC, TX_buf_src);
    drv_l1_can_tx_en(TX_buf_src);
    drv_l1_can_tx_wait_complete(TX_buf_src);
    TX_err = drv_l1_can_tx_chk_status(TX_buf_src);
    return TX_err;
}

void drv_l1_can_write(CAN_TX_Struct CAN_Setting)
{
    drv_l1_can_tx_ide_format(CAN_Setting.TX_ExtendID_Set, CAN_Setting.TX_Buf_Src);
    drv_l1_can_tx_byte_order(CAN_Setting.Byte_Order, CAN_Setting.TX_Buf_Src);
    drv_l1_can_tx_remote_transmit_request_en(CAN_Setting.RemoteTransmitReq, CAN_Setting.TX_Buf_Src);
    drv_l1_can_tx_id(CAN_Setting.TX_Basic_ID, CAN_Setting.TX_Extend_ID, CAN_Setting.TX_Buf_Src);
    drv_l1_can_tx_d3tod0(CAN_Setting.D3toD0_data, CAN_Setting.TX_Buf_Src);
    drv_l1_can_tx_d7tod4(CAN_Setting.D7toD4_data, CAN_Setting.TX_Buf_Src);
    drv_l1_can_tx_dlc(CAN_Setting.DataLengthControl, CAN_Setting.TX_Buf_Src);
    drv_l1_can_tx_en(CAN_Setting.TX_Buf_Src);
    //
    drv_l1_can_tx_wait_complete(CAN_Setting.TX_Buf_Src);
}

void drv_l1_can_read(CAN_RX_Struct CAN_Setting)
{
    drv_l1_can_rx_ide_format(CAN_Setting.RX_ExtendID_Set, CAN_Setting.RX_Buf_Src);
    drv_l1_can_rx_id(CAN_Setting.RX_Basic_ID, CAN_Setting.RX_Extend_ID, CAN_Setting.RX_Buf_Src);
    drv_l1_can_rx_mask(CAN_Setting.RX_Mask_BID, CAN_Setting.RX_Mask_EID, CAN_Setting.RX_Buf_Src);
    drv_l1_can_rx_byte_order(CAN_Setting.Byte_Order, CAN_Setting.RX_Buf_Src);
    drv_l1_can_rx_id_valid(TRUE, CAN_Setting.RX_Buf_Src);
    //
    drv_l1_can_rx_wait_complete(CAN_Setting.RX_Buf_Src);
    drv_l1_can_rx_data_receive(CAN_Setting.D3toD0_Ptr, CAN_Setting.D7toD4_Ptr, CAN_Setting.RX_Buf_Src);
    drv_l1_can_rx_buf_free(TRUE, CAN_Setting.RX_Buf_Src);
    drv_l1_can_rx_id_valid(FALSE, CAN_Setting.RX_Buf_Src);
}

INT32U drv_l1_can_read_once(INT8U RX_buf_src, INT32U Byte_DLC, INT16U B_ID, INT16U E_ID, INT32U* D0toD3, INT32U* D4toD7)
{
    INT32U  Status;

    drv_l1_can_rx_id(B_ID ,E_ID ,RX_buf_src); //may need moified to set mask value and seperate ID and mask to another function
    drv_l1_can_rx_byte_order(Byte_DLC, RX_buf_src);
    drv_l1_can_rx_id_valid(TRUE, RX_buf_src);
    drv_l1_can_rx_wait_complete(RX_buf_src);
    Status = drv_l1_can_rx_chk_status(RX_buf_src);
    drv_l1_can_rx_data_receive(D0toD3, D4toD7, RX_buf_src);
    drv_l1_can_rx_buf_free(TRUE, RX_buf_src);
    drv_l1_can_rx_id_valid(FALSE, RX_buf_src);
    return Status;
}

#endif

