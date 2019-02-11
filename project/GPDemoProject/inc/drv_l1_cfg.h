#ifndef __DRIVER_L1_CFG_H__
#define __DRIVER_L1_CFG_H__

    #define SDRAM_START_ADDR            0x00000000
    #define SDRAM_END_ADDR              0x01FFFFFF

    #define ISRAM_START_ADDR            0x1FFF0000
    #define ISRAM_END_ADDR              0x1FFFFFFF

    // MCU system configuration
    #define _DRV_L1_CACHE               1
    #define _DRV_L1_DMA                 1
    #define _DRV_L1_GPIO                1
    #define _DRV_L1_EXT_INT             1
    #define _DRV_L1_CONV420TO422        1

    // MCU timer configuration
    #define _DRV_L1_TIMER               1
    #define _DRV_L1_RTC                 1
    #define _DRV_L1_WATCHDOG            1

    // MCU audio configuration
    #define _DRV_L1_DAC                 1
    #define _DRV_L1_ADC                 1
    #define _DRV_L1_MIC                 1
    #define _DRV_L1_SPU                 1
    #define _DRV_L1_I2S_TX              1
    #define _DRV_L1_I2S_RX              1
    #define _DRV_L1_I2S_TX              1
    #define _DRV_L1_I2S_RX              1
    #define _DRV_L1_FFT                 1

    // MCU storage configuration
    #define _DRV_L1_NAND                0
    #define _DRV_L1_SDC                 1

    // MCU peripheral configuration
    #define _DRV_L1_SPIFC               0
    #define _DRV_L1_I2C                 1
    #define _DRV_L1_SPI                 1
    #define _DRV_L1_UART0               0
    #define _DRV_L1_UART1               1
    #define _DRV_L1_UART2               0
    #define _DRV_L1_CAN                 1
    #define _DRV_L1_CEC                 1
    #define _DRV_L1_SPI_SW_CS           1

    // MCU image configuration
    #define _DRV_L1_SCALER              1
    #define _DRV_L1_JPEG                1
    #define _DRV_L1_PPU                 1
	#define _DRV_L1_ROTATOR				1

    // MCU display configuration
    #define _DRV_L1_TFT                 1
    #define _DRV_L1_H264SCALER          1
    #define _DRV_L1_HDMI                1

    // MCU sensor configuration
    #define _DRV_L1_CSI                 1
    #define _DRV_L1_CDSP                1
    #define _DRV_L1_MIPI                1

    // MCU usb configuration
    #define _DRV_L1_USBH_UVC            0
    #define _DRV_L1_USBH                0
    #define _DRV_L1_USBD                0

    // MCU H264
    #define _DRV_L1_H264                1

    // UART interface configuration
    #define UART0_BAUD_RATE             115200

#endif      // __DRIVER_L1_CFG_H__
