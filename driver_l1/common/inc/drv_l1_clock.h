#ifndef __DRV_L1_CLOCK_H__
#define __DRV_L1_CLOCK_H__

#include "typedef.h"

typedef enum
{
    I2S_MAIN_FREQ_73728MHZ, // div 1
    I2S_MAIN_FREQ_36864MHZ, // div 2, wolfson wm8988, clkdiv2
    I2S_MAIN_FREQ_24576MHZ, // div 3, wolfson wm8988, clkdiv2
    I2S_MAIN_FREQ_18432MHZ, // div 4
    I2S_MAIN_FREQ_14745MHZ, // div 5
    I2S_MAIN_FREQ_12288MHZ, // div 6, wolfson wm8988, clkbase

    I2S_MAIN_FREQ_67738MHZ, // div 1
    I2S_MAIN_FREQ_33869MHZ, // div 2, wolfson wm8988, clkdiv2
    I2S_MAIN_FREQ_22579MHZ, // div 3, wolfson wm8988, clkdiv2
    I2S_MAIN_FREQ_16934MHZ, // div 4
    I2S_MAIN_FREQ_13547MHZ, // div 5
    I2S_MAIN_FREQ_11289MHZ  // div 6, wolfson wm8988, clkbase
} I2S_MAIN_FREQ_E;

typedef enum
{
    APLL_FREQ_73728MHZ,
    APLL_FREQ_67738MHZ
} APLL_FREQ_E;

typedef enum
{
    CLK_EN0_SYSTEM_BUS,//inlcuding System Bus/ System Control Register/ GPIO/ Interrupt/ Internal RTC/ Ti
    CLK_EN0_MEMORY_DRAM,
    CLK_EN0_PSCA0,
    CLK_EN0_HDMI,
    CLK_EN0_TIMER,
    CLK_EN0_CPU,
    CLK_EN0_DAC_I2STX,
    CLK_EN0_UART,
    CLK_EN0_CEC_CAN_I2C,
    CLK_EN0_PSCA1,
    CLK_EN0_CODEC_ADC_I2SRX,
    CLK_EN0_H264DE,
    CLK_EN0_SPU,
    CLK_EN0_TFT,
    CLK_EN0_ISP,
    CLK_EN0_SDC,
    CLK_EN1_FD_ROTATOR,
    CLK_EN1_USBH20,
    CLK_EN1_USBD20,
    CLK_EN1_PPU,
    CLK_EN1_DMA,
    CLK_EN1_H264EN,
    CLK_EN1_DISPLAY_CSI,
    CLK_EN1_FFT,
    CLK_EN1_SPI_SPIFC,
    CLK_EN1_MIPI,
    CLK_EN1_JPEG,
    CLK_EN1_DISPAY_OUT,
    CLK_EN1_SCALER,
    CLK_EN1_NAND_BCH,
    CLK_EN1_H264_AVS,
    CLK_EN1_SYSTEM_CTRL,
    CLK_EN_MAX
} SYSTEM_CLK_EN_E;

extern INT32U drv_l1_clock_get_system_clock_freq(void);
extern void drv_l1_clock_enable_apll_clock(void);
extern void drv_l1_clock_diable_apll_clock(void);
extern INT32U drv_l1_clock_apll_clock_get_enable_state(void);
extern INT32U drv_l1_clock_get_apll_freq(void);
extern INT32S drv_l1_clock_set_apll_freq(APLL_FREQ_E apll_freq);
extern INT32U drv_l1_clock_get_apll_divider(void);
extern INT32S drv_l1_clock_set_apll_divider(INT32U divider);
extern INT32U drv_l1_clock_get_rx_use_apll_clock(void);
extern void drv_l1_clock_set_rx_use_apll_clock(INT32U use_apll);
extern INT32U drv_l1_clock_get_i2s_main_mclk_freq(void);
extern INT32S drv_l1_clock_set_i2s_main_mclk_freq(I2S_MAIN_FREQ_E req_freq);
extern void drv_l1_clock_system_clk_init(void);
extern INT16S drv_l1_clock_set_system_clk_en(INT16U module, INT16U enable);


#endif
