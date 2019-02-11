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
#include "drv_l1_cec.h"
#include "drv_l1_clock.h"

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined _DRV_L1_CEC) && (_DRV_L1_CEC == 1)
/**************************************************************************
*               F U N C T I O N    D E C L A R A T I O N S               *
**************************************************************************/

/**************************************************************************
*                         G L O B A L    D A T A                         *
**************************************************************************/
#if 0
#define DEBUG_MSG(x)	{}
#else
#define DEBUG_MSG(x)	{x;}
#endif

static INT8U *rx_data;
volatile INT32U rxcnt = 0;
volatile INT32U max_rxcnt;
static void (*cec_isr[CEC_ISR_MAX])(void);
static tx_err;
volatile int txrdy, txrq, rxrdy;

volatile int err_cnt=0;

static void drv_l1_cec_msec_wait(INT32U msec)
{
	INT32U cnt = msec * (SystemCoreClock >> 13); //0.64ms
	volatile INT32U i;

	for(i=0; i<cnt; i++) {
		i = i;
	}
}

void drv_l1_cec_enable(INT8U enable)
{
    if(enable==1)
        drv_l1_clock_set_system_clk_en(CLK_EN0_CEC_CAN_I2C,1);
    R_CEC_CTRL &= 0x1;
    R_CEC_CTRL |= enable&0x1;
    if(enable==0)
        drv_l1_clock_set_system_clk_en(CLK_EN0_CEC_CAN_I2C,0);
}

void drv_l1_cec_listen(INT8U mode)
{
    R_CEC_CFGR &= ~(0x1 << 31);
    R_CEC_CFGR |= ((mode&0x1) << 31);
}

void drv_l1_cec_own_address_config(INT16U own_bit)
{
    R_CEC_CFGR &= ~(0x7FFF << 16);
    R_CEC_CFGR |= ((own_bit&0x7FFF) << 16);
}

void drv_l1_cec_sft_set(INT8U option, INT8U sft)
{
    R_CEC_CFGR &= ~0x0107;
    R_CEC_CFGR |= ((option&0x1)<<8)|(sft&0x7);
}

void drv_l1_cec_rx_tolerance(INT8U tol)
{
    R_CEC_CFGR &= ~(0x1<<0x3);
    R_CEC_CFGR |= ((tol&0x1)<<0x3);
}

void drv_l1_cec_error_bit_config(INT8U enable_bit)
{
    R_CEC_CFGR &= ~(0xF<<0x4);
    R_CEC_CFGR |= enable_bit;
}

void drv_l1_cec_tx2(INT8U data,INT8U data2)
{
    txrdy = txrq = 1;
    R_CEC_TX_DATA = data;
    R_CEC_CTRL |= B_TX_SOM | (0 << 2);
    while(txrq);
    R_CEC_TX_DATA = data;
    R_CEC_CTRL |= (1 << 2);
    while(txrdy);

}

INT32S drv_l1_cec_tx(INT8U data, INT8U eom, INT8U som)
{
    INT32S ret = 0;
    INT32U timeout;
    txrdy = txrq = 1;
    R_CEC_TX_DATA = data;
    R_CEC_CTRL |= (som << 1) | (eom << 2);
    timeout = 0;
    tx_err = 0;
    if(eom == 0)
    {
        while(txrq)
        {
            timeout++;
            if(timeout>=150)
            {
                DBG_PRINT("TXrqTO\r\n");
                //R_CEC_CTRL &= ~(0x3<<1);
                ret |= -1;
                break;
            }
            drv_l1_cec_msec_wait(1);
        }
    }
    else
    {
        while(txrdy)
        {
            timeout++;
            if(timeout>=150)
            {
                DBG_PRINT("TXrdyTO\r\n");
                //R_CEC_CTRL &= ~(0x3<<1);
                ret |= -2;
                break;
            }
            drv_l1_cec_msec_wait(1);
        }
    }
    if(tx_err==1)
        return -4;
    else
        return ret;
}

INT32U drv_l1_cec_get_rx_count(void)
{
    return rxcnt;
}

void drv_l1_cec_assign_rx_buffer(INT8U *rx_data_ptr, INT32U data_length)
{
    max_rxcnt = data_length;
    rx_data = rx_data_ptr;
}

void drv_l1_cec_init(void)
{
    txrdy = txrq = rxrdy = rxcnt = 0;

    drv_l1_cec_enable(1);
    R_CEC_CFGR |= 0x1<<3 | 0x7; // RXTOL
    //interrupt
    NVIC_EnableIRQ(CEC_IRQn);
    NVIC_SetPriority(CEC_IRQn, 5);
    //set gpio
    //check mux priority , csi need disable
}

void drv_l1_cec_register_handler( INT32U handler_idx, void (*handler)(void) )
{
    if(handler_idx >= CEC_ISR_MAX)
    {
        DBG_PRINT("over max index\r\n");
        return;
    }
    cec_isr[handler_idx] = handler;
}

void drv_l1_cec_unregister_handler( INT32U handler_idx)
{
    if(handler_idx >= CEC_ISR_MAX)
    {
        DBG_PRINT("over max index\r\n");
        return;
    }
    cec_isr[handler_idx] = NULL;
}

void drv_l1_cec_interrupt_enable(INT32U enable)
{
    R_CEC_IER = enable;
}

void CEC_IRQHandler(void)
{
    INT16U status;
    //                      12      11     10     9      8
    //                      TXACKE  TXERR  TXUDR  TXEND  TXBR
    // 7      6       5     4       3      2      1      0
    // ARBLST RXACKE  LBPE  SBPE    BRE    RXOVR  RXEND  RXBR
    status = R_CEC_ISR;
    //DBG_PRINT("irq 0x%x\r\n", R_CEC_ISR);
    R_CEC_ISR = status;//clear flag

    if(status & B_RXBR){
        *(rx_data + rxcnt)= R_CEC_RX_DATA;
        //DBG_MSG("RX 0x%x\r\n",*(rx_data + rxcnt));
        //DEBUG_MSG(DBG_PRINT("r"));
        rxcnt++;
        if(rxcnt >= max_rxcnt)
        {
            rxcnt = 0;
            DEBUG_MSG(DBG_PRINT("RX buffer oversize, new data will overwrite\r\n"));
        }
        if(cec_isr[CEC_ISR_RXBR])
            (cec_isr[CEC_ISR_RXBR])();
    }
    if(status & B_RXEND) {
        //DEBUG_MSG(DBG_PRINT("e"));
        if(cec_isr[CEC_ISR_RXEND])
            (cec_isr[CEC_ISR_RXEND])();
        rxcnt = 0;
    }
    if(status & B_LBPE){
        err_cnt++;
        rxcnt = 0;
        DEBUG_MSG(DBG_PRINT("RX LBPE "));
        if(cec_isr[CEC_ISR_LBPE])
            (cec_isr[CEC_ISR_LBPE])();
    }
    if(status & B_SBPE){
        err_cnt++;
        rxcnt = 0;
        DEBUG_MSG(DBG_PRINT("RX SBPE "));
        if(cec_isr[CEC_ISR_SBPE])
            (cec_isr[CEC_ISR_SBPE])();
    }
    if(status & B_BRE){
        err_cnt++;
        rxcnt = 0;
        DEBUG_MSG(DBG_PRINT("RX BRE "));
        if(cec_isr[CEC_ISR_BRE])
            (cec_isr[CEC_ISR_BRE])();
    }
    if(status & B_TXBR){
        txrq = 0;
        if(cec_isr[CEC_ISR_TXBR])
            (cec_isr[CEC_ISR_TXBR])();
    }
    if(status & B_TXEND){
        txrdy = 0;
        if(cec_isr[CEC_ISR_TXEND])
            (cec_isr[CEC_ISR_TXEND])();
    }
    if(status & B_TXERR) {
        DEBUG_MSG(DBG_PRINT("B_TXERR\r\n"));
        txrdy = txrq = 0;
        tx_err = 1;
        if(cec_isr[CEC_ISR_TXERR])
            (cec_isr[CEC_ISR_TXERR])();
    }
    if(status & B_ARBLST) {
        DEBUG_MSG(DBG_PRINT("B_ARBLST\r\n"));
        txrdy = txrq = 0;
        tx_err = 1;
    }
    if(status & B_TXUDR) {
        DEBUG_MSG(DBG_PRINT("B_TXUDR\r\n"));
        txrdy = txrq = 0;
        tx_err = 1;
    }
    if(status & B_TXACKE) {
        txrdy = txrq = 0;
        tx_err = 1;
        if(cec_isr[CEC_ISR_TXACKE])
            (cec_isr[CEC_ISR_TXACKE])();
        DEBUG_MSG(DBG_PRINT("B_TXACKE\r\n"));
    }
    if(status & B_RXACKE) {
        rxcnt = 0;
        DEBUG_MSG(DBG_PRINT("B_RXACKE\r\n"));
    }
    if(status & B_RXOVR) {
        rxcnt = 0;
        DEBUG_MSG(DBG_PRINT("B_RXOVR\r\n"));
    }
}

#endif //(defined _DRV_L1_CEC) && (_DRV_L1_CEC == 1)
