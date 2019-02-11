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
#include <string.h>
#include "project.h"
#include "typedef.h"
#include "drv_l1_cfg.h"
#include "drv_l1_sfr.h"
#include "drv_l1_clock.h"

INT32U drv_l1_clock_get_system_clock_freq(void)
{
    INT32U systemFreq;

    // each step is 9MHz, plus basis 54MHz, then divide 2
    systemFreq = (((R_SYSTEM_PLLEN & 0x3f) -3) * 9UL) + 54;
    systemFreq = (systemFreq * 1000000) /2;

    return systemFreq;
}

void drv_l1_clock_enable_apll_clock(void)
{
	R_SYSTEM_CKGEN_CTRL |= (1UL<<4); // 0xD0000058[4] enable APPL_EN, Clock source for CODEC ADC and I2S
}

void drv_l1_clock_disable_apll_clock(void)
{
	R_SYSTEM_CKGEN_CTRL &= ~(1UL<<4); // 0xD0000058[4] disable APPL_EN, Clock source for CODEC ADC and I2S
}

INT32U drv_l1_clock_apll_clock_get_enable_state(void)
{
	INT32U state;

	state = (R_SYSTEM_CKGEN_CTRL & (1UL<<4)) == 0 ? 0: 1;

	return state;
}

INT32U drv_l1_clock_get_apll_freq(void)
{
    INT8U byte_value;

    byte_value = ((R_SYSTEM_APLL_FREQ >> 8) & 0x0f);
    if (byte_value == 9)
        return 67738; // 67.7376MHz * 1000
    else
        return 73728; // 73.728MHz * 1000
}

INT32S drv_l1_clock_set_apll_freq(APLL_FREQ_E apll_freq)
{
    INT32S ret = 0;

    // 0xD00000C4[11:8] APDIV R/W Audio PLL output freq selection, 4¡¦ha:73.728MHz, 4¡¦h9:67.7376MHz
    if (apll_freq == APLL_FREQ_67738MHZ)
        R_SYSTEM_APLL_FREQ = (R_SYSTEM_APLL_FREQ & (~(0xfUL << 8))) | (9UL << 8);
    else if (apll_freq == APLL_FREQ_73728MHZ)
        R_SYSTEM_APLL_FREQ = (R_SYSTEM_APLL_FREQ & (~(0xfUL << 8))) | (10UL << 8);
    else
        ret = -1;

    return ret;
}

INT32U drv_l1_clock_get_apll_divider(void)
{
    // 0xD0000074[15:11] I2S MCLK = APLL/(1+MCLK_DIV)
    return ((R_SYSTEM_MISC_CTRL4 >> 11) & 0x1f);
}

INT32S drv_l1_clock_set_apll_divider(INT32U divider)
{
    if (divider > 31)
      return -1;

    R_SYSTEM_MISC_CTRL4 = (R_SYSTEM_MISC_CTRL4 & (~(0x1fUL << 11))) | (divider << 11);

    return 0;
}

INT32U drv_l1_clock_get_rx_use_apll_clock(void)
{
    if ((R_SYSTEM_CODEC_CTRL1 & (1<<15)) == 0)
        return 0;
    else
        return 1;
}

void drv_l1_clock_set_rx_use_apll_clock(INT32U use_apll)
{
    R_SYSTEM_CODEC_CTRL1 = (R_SYSTEM_CODEC_CTRL1 & (~(1UL << 15))) | ((use_apll ? 1UL:0UL) << 15);
}

INT32U drv_l1_clock_get_i2s_main_mclk_freq(void)
{
    INT32U apll_frq;
    INT32U freq;

    apll_frq = drv_l1_clock_get_apll_freq();
    // 0xD0000070[15] RX_USE_APCK=0, 0: divider after APLL, 1: APLL output
    if (drv_l1_clock_get_rx_use_apll_clock() == 0)
    {
        // 0xD0000074[15:11] I2S MCLK = APLL/(1+MCLK_DIV)
        freq = apll_frq / (drv_l1_clock_get_apll_divider() + 1);
    }
    else
        freq = apll_frq;

    return freq;
}

INT32S drv_l1_clock_set_i2s_main_mclk_freq(I2S_MAIN_FREQ_E req_freq)
{
    INT32S ret = 0;

    switch (req_freq)
    {
        case I2S_MAIN_FREQ_73728MHZ: // div 1

            drv_l1_clock_set_apll_freq(APLL_FREQ_73728MHZ);
            //drv_l1_clock_set_apll_divider(0); // ?
            drv_l1_clock_set_rx_use_apll_clock(1);

            break;
        case I2S_MAIN_FREQ_36864MHZ: // div 2
            drv_l1_clock_set_apll_freq(APLL_FREQ_73728MHZ);
            drv_l1_clock_set_apll_divider(1);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_24576MHZ: // div 3
            drv_l1_clock_set_apll_freq(APLL_FREQ_73728MHZ);
            drv_l1_clock_set_apll_divider(2);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_18432MHZ: // div 4
            drv_l1_clock_set_apll_freq(APLL_FREQ_73728MHZ);
            drv_l1_clock_set_apll_divider(3);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_14745MHZ: // div 5
            drv_l1_clock_set_apll_freq(APLL_FREQ_73728MHZ);
            drv_l1_clock_set_apll_divider(4);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_12288MHZ: // div 6
            drv_l1_clock_set_apll_freq(APLL_FREQ_73728MHZ);
            drv_l1_clock_set_apll_divider(5);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;

        case I2S_MAIN_FREQ_67738MHZ: // div 1
            drv_l1_clock_set_apll_freq(APLL_FREQ_67738MHZ);
            //drv_l1_clock_set_apll_divider(0); // ?
            drv_l1_clock_set_rx_use_apll_clock(1);

            break;
        case I2S_MAIN_FREQ_33869MHZ: // div 2
            drv_l1_clock_set_apll_freq(APLL_FREQ_67738MHZ);
            drv_l1_clock_set_apll_divider(1);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_22579MHZ: // div 3
            drv_l1_clock_set_apll_freq(APLL_FREQ_67738MHZ);
            drv_l1_clock_set_apll_divider(2);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_16934MHZ: // div 4
            drv_l1_clock_set_apll_freq(APLL_FREQ_67738MHZ);
            drv_l1_clock_set_apll_divider(3);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_13547MHZ: // div 5
            drv_l1_clock_set_apll_freq(APLL_FREQ_67738MHZ);
            drv_l1_clock_set_apll_divider(4);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        case I2S_MAIN_FREQ_11289MHZ : // div 6
            drv_l1_clock_set_apll_freq(APLL_FREQ_67738MHZ);
            drv_l1_clock_set_apll_divider(5);
            drv_l1_clock_set_rx_use_apll_clock(0);
            break;
        default:
            ret = -1;
            break;
    }
    return ret;
}

void drv_l1_clock_system_clk_init(void)
{
    R_SYSTEM_CLK_EN0 &= (1<<CLK_EN0_SYSTEM_BUS) | (1<<CLK_EN0_MEMORY_DRAM) | (1<<CLK_EN0_CPU);
    R_SYSTEM_CLK_EN1 &= (1<< (CLK_EN1_SYSTEM_CTRL-16)) | (1<< (CLK_EN1_DMA-16));
}

INT16S drv_l1_clock_set_system_clk_en(INT16U module, INT16U enable)
{
    INT16S ret = 0;
    INT16U val;
    INT32U inISR;

    if (module > CLK_EN_MAX)
        return ret = -1;

	inISR = __get_IPSR();
	if(inISR == 0)
	{
		taskENTER_CRITICAL();
	}

    if(module < 16){
        val = R_SYSTEM_CLK_EN0;
        val &= ~ (1<< module);
        val |= (enable<<module);
        R_SYSTEM_CLK_EN0 = val;
    }
    else{
        module -= 16;
        val = R_SYSTEM_CLK_EN1;
        val &= ~ (1<< module);
        val |= (enable<<module);
        R_SYSTEM_CLK_EN1 = val;
    }
	if(inISR == 0)
	{
		taskEXIT_CRITICAL();
	}
    return ret;
}

INT16S drv_l1_clock_set_system_clk_en_isr(INT16U module, INT16U enable)
{
    INT16S ret = 0;
    INT16U val;

    if (module > CLK_EN_MAX)
        return ret = -1;

    if(module < 16){
        val = R_SYSTEM_CLK_EN0;
        val &= ~ (1<< module);
        val |= (enable<<module);
        R_SYSTEM_CLK_EN0 = val;
    }
    else{
        module -= 16;
        val = R_SYSTEM_CLK_EN1;
        val &= ~ (1<< module);
        val |= (enable<<module);
        R_SYSTEM_CLK_EN1 = val;
    }

    return ret;
}
