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
//Include files
#include "drv_l1.h"
#include "drv_l1_cache.h"
#include "drv_l1_dma.h"
#include "drv_l1_uart.h"
#include "drv_l1_timer.h"
#include "drv_l1_clock.h"
#include "drv_l1_ext_int.h"
#include "drv_l1_spifc.h"
#include "drv_l1_usbd.h"

#if BOARD_TYPE != BOARD_GPM41XXA_EMU_V1_0
#include "drv_l1_spu.h"
#endif
#if _DRV_L1_TV == 1
extern void system_clk_ext_XLAT_12M(void);
#endif

INT32U MCLK;
INT32U MHZ;

void show_system_clock_rate(void)
{
    DBG_PRINT("MHz=%u, MCLK=%u\r\n", MHZ, MCLK);
}

void drv_l1_init(void)
{
    INT8U spifc_id[4];

    R_SHCSR |= 0x00070000;

    MCLK = SystemCoreClock;
    MHZ = MCLK /1000000;
    drv_l1_system_arbiter_init();

    //drv_l1_clock_system_clk_init();
#if _DRV_L1_USBD == 1
    //Note: suspend USB phy to save power;
    //drv_l1_usbd_init() will resume USB phy when USB function is used
    drv_l1_usbd_uphy_suspend(1);
#endif

#if _DRV_L1_CACHE == 1
    cache_init();						// Initiate CPU Cache
#endif

#if _DRV_L1_TV == 1
    system_clk_ext_XLAT_12M();
#endif

#if _DRV_L1_UART0 == 1
    drv_l1_uart0_init();
#endif

#if _DRV_L1_UART1 == 1
    drv_l1_clock_set_system_clk_en(CLK_EN0_UART, 1);
    drv_l1_uart1_init();
    drv_l1_uart1_buad_rate_set(UART0_BAUD_RATE);
    drv_l1_uart1_tx_enable();
    drv_l1_uart1_rx_enable();
#endif

#if _DRV_L1_UART2 == 1
    //R_FUNPOS1 = (R_FUNPOS1 & (~(0x3 << 13))) | (2 << 13); // williamyeo, for I2S board UART2 #2
    drv_l1_clock_set_system_clk_en(CLK_EN0_UART, 1);
    drv_l1_uart2_init();
    drv_l1_uart2_buad_rate_set(UART0_BAUD_RATE);
    drv_l1_uart2_tx_enable();
    drv_l1_uart2_rx_enable();
#endif

#if _DRV_L1_TIMER==1
    drv_l1_clock_set_system_clk_en(CLK_EN0_TIMER, 1);
    timer_init();						// Initiate Timer
    timerD_counter_init();              // Tiny Counter Initial (1 tiny count == 2.67 us)
#endif

#if _DRV_L1_EXT_INT==1
    drv_l1_ext_int_init(EXTA);
    drv_l1_ext_int_init(EXTB);
    drv_l1_ext_int_init(EXTC);
#endif

#if _DRV_L1_DMA == 1
    drv_l1_dma_init();					// Initiate DMA controller
#endif

#if _DRV_L1_DAC == 1
    drv_l1_dac_init();
#endif

#if _DRV_L1_ADC == 1
    drv_l1_adc_init();
#endif

#if _DRV_L1_RTC == 1
    drv_l1_rtc_init();
#endif

#if _DRV_L1_CONV420TO422 == 1
	conv420_init();
#endif

#if _DRV_L1_SPU == 1
    SPU_System_Colok_Set(MCLK);
#endif

#if (defined _DRV_L1_SDC) && (_DRV_L1_SDC == 1)
    drv_l1_clock_set_system_clk_en(CLK_EN0_SDC, 1);
#endif

#if (defined _DRV_L2_NAND) && (_DRV_L2_NAND == 1)
    // NOTE: when using nand, spi flash must be disabled

    // disable spi flash. NOTE: To disable successfully, clock source must enable
    drv_l1_clock_set_system_clk_en(CLK_EN1_SPI_SPIFC, 1);
    R_SPIFC_CTRL &= (~(1<<0));
    drv_l1_clock_set_system_clk_en(CLK_EN1_SPI_SPIFC, 0);

    drv_l1_clock_set_system_clk_en(CLK_EN1_NAND_BCH, 1);
#endif

#if (defined _DRV_L1_SPIFC) && (_DRV_L1_SPIFC == 1)
    drv_l1_spifc_init(12000000);
    drv_l1_spifcflash_read_id((INT8U *)&spifc_id);
    DBG_PRINT("SPI ID = 0x%x 0x%x 0x%x\r\n",spifc_id[0],spifc_id[1],spifc_id[2]);

    drv_l1_spifc_set_read_mode(IO_4BIT,0);
#endif
    show_system_clock_rate();
}
