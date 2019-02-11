#include "drv_l1_sfr.h"
#include "drv_l1_tft.h"
#include "drv_l1_gpio.h"
#include "drv_l1_timer.h"
#include "drv_l2_display.h"

#if (defined TXD_T144C) && (TXD_T144C == 1)

INT32S tft_txd_t144c_init(void)
{
	INT32U i;

    drv_l1_tft_mem_unit_set(8);
	drv_l1_tft_clk_set(TFT_CLK_DIVIDE_3);
	//drv_l1_tft_clk_set(TFT_CLK_DIVIDE_2);
	//R_TFT_TS_MISC = 0x00101;
	R_TFT_LINE_RGB_ORDER |= 0x0080; //memory mode
#if 0
    R_IOA_DRV = 0xFF;
    R_IOB_DRV = 0xF;
/* TFT interface parameter */
	R_TFT_V_PERIOD = 320-1;
	R_TFT_VS_WIDTH = 20;
	R_TFT_V_START  = 5;
	R_TFT_H_PERIOD = 240-1;
	R_TFT_HS_WIDTH = 3;
	R_TFT_H_START  = 3;

	R_TFT_HS_START = 0;
	R_TFT_HS_END   = 239;
	R_TFT_VS_START = 0;
	R_TFT_VS_END   = 320;

    drv_l1_tft_i80_wr_cmd(0x11);	//sleep out
    //drv_msec_wait (120);
    drv_l1_tft_i80_wr_cmd(0x29);	//display on
//drv_msec_wait (1);
    drv_l1_tft_i80_wr_cmd(0x35);	//tearing effect line on
//drv_msec_wait (1);
    //drv_l1_tft_i80_wr_data (0x01);  //Vsync & Hsync
    drv_l1_tft_i80_wr_data (0x00);  //Vsync & Hsync
//drv_msec_wait (1);
    drv_l1_tft_i80_wr_cmd(0x2c);
//drv_msec_wait (1);
    //R_TFT_TE_HS_WIDTH = 6485;
    //R_TFT_TE_CTRL = 0x2010101;/* set after 2 line and enable MCU write slower than panel read and RGB888 and TE */
    //R_TFT_TE_CTRL = 0x100;
	//drv_l1_tft_clk_set(TFT_CLK_DIVIDE_5);
	//drv_l1_tft_clk_set(TFT_CLK_DIVIDE_6);
	//drv_l1_tft_mode_set(TFT_MODE_MEM_CONTI);
	drv_l1_tft_mode_set(TFT_MODE_MEM_ONCE);
	//drv_l1_tft_en_set(TRUE);
#if 1
    R_GPIO_CTRL |= (1<<6);
    R_TFT_TE_HS_WIDTH = 6485;
    //R_TFT_TE_CTRL = 0x2010101;/* set after 2 line and enable MCU write slower than panel read and RGB888 and TE */
    R_TFT_TE_CTRL = 0x1000101;/* set after 2 line and enable MCU write slower than panel read and RGB888 and TE */
#endif
#else
#if 0	//hw reset
	gpio_init_io(IO_C3, GPIO_OUTPUT);
	gpio_set_port_attribute(IO_C3, ATTRIBUTE_HIGH);
    gpio_write_io(IO_C3, DATA_LOW);
    drv_msec_wait(200);
    drv_msec_wait(200);
    gpio_write_io(IO_C3, DATA_HIGH);
    drv_msec_wait(200);
#endif

/* TFT interface parameter */
	R_TFT_V_PERIOD = 128-1;
	R_TFT_VS_WIDTH = 20;
	R_TFT_V_START  = 5;
	R_TFT_H_PERIOD = 128-1;
	R_TFT_HS_WIDTH = 5;
	R_TFT_H_START  = 3;

	R_TFT_HS_START = 0;
	R_TFT_HS_END   = 127;
	R_TFT_VS_START = 0;
	R_TFT_VS_END   = 128;

    /* Config parameters */
    drv_l1_tft_i80_wr_cmd(0x11);
    drv_msec_wait (120);

    drv_l1_tft_i80_wr_cmd(0xb1);
    drv_l1_tft_i80_wr_data(0x02);
    drv_l1_tft_i80_wr_data(0x35);
    drv_l1_tft_i80_wr_data(0x36);

    drv_l1_tft_i80_wr_cmd(0xb2);
    drv_l1_tft_i80_wr_data(0x02);
    drv_l1_tft_i80_wr_data(0x35);
    drv_l1_tft_i80_wr_data(0x36);

    drv_l1_tft_i80_wr_cmd(0xb3);
    drv_l1_tft_i80_wr_data(0x02);
    drv_l1_tft_i80_wr_data(0x35);
    drv_l1_tft_i80_wr_data(0x36);
    drv_l1_tft_i80_wr_data(0x02);
    drv_l1_tft_i80_wr_data(0x35);
    drv_l1_tft_i80_wr_data(0x36);

    drv_l1_tft_i80_wr_cmd(0xb4);
    drv_l1_tft_i80_wr_data(0x00);

    drv_l1_tft_i80_wr_cmd(0xb6);
    drv_l1_tft_i80_wr_data(0xb4);

    drv_l1_tft_i80_wr_cmd(0xc0);
    drv_l1_tft_i80_wr_data(0xa2);
    drv_l1_tft_i80_wr_data(0x02);
    drv_l1_tft_i80_wr_data(0x84);

    drv_l1_tft_i80_wr_cmd(0xc1);
    drv_l1_tft_i80_wr_data(0xc5);

    drv_l1_tft_i80_wr_cmd(0xc2);
    drv_l1_tft_i80_wr_data(0x0a);
    drv_l1_tft_i80_wr_data(0x00);

    drv_l1_tft_i80_wr_cmd(0xc3);
    drv_l1_tft_i80_wr_data(0x8a);
    drv_l1_tft_i80_wr_data(0x2a);

    drv_l1_tft_i80_wr_cmd(0xc4);
    drv_l1_tft_i80_wr_data(0x8a);
    drv_l1_tft_i80_wr_data(0xee);

    drv_l1_tft_i80_wr_cmd(0xc5);
    drv_l1_tft_i80_wr_data(0x0e);

    drv_l1_tft_i80_wr_cmd(0x36);
    #if 1   //UPSIDE_DOWN == 0
    drv_l1_tft_i80_wr_data(0xc8);
    #else
    drv_l1_tft_i80_wr_data(0x08);		// 上下左右階反轉
    #endif
    drv_l1_tft_i80_wr_cmd(0xe0);
    drv_l1_tft_i80_wr_data(0x12);
    drv_l1_tft_i80_wr_data(0x1c);
    drv_l1_tft_i80_wr_data(0x10);
    drv_l1_tft_i80_wr_data(0x18);
    drv_l1_tft_i80_wr_data(0x33);
    drv_l1_tft_i80_wr_data(0x2c);
    drv_l1_tft_i80_wr_data(0x25);
    drv_l1_tft_i80_wr_data(0x28);
    drv_l1_tft_i80_wr_data(0x28);
    drv_l1_tft_i80_wr_data(0x27);
    drv_l1_tft_i80_wr_data(0x2f);
    drv_l1_tft_i80_wr_data(0x3c);
    drv_l1_tft_i80_wr_data(0x00);
    drv_l1_tft_i80_wr_data(0x03);
    drv_l1_tft_i80_wr_data(0x03);
    drv_l1_tft_i80_wr_data(0x10);

    drv_l1_tft_i80_wr_cmd(0xe1);
    drv_l1_tft_i80_wr_data(0x12);
    drv_l1_tft_i80_wr_data(0x1c);
    drv_l1_tft_i80_wr_data(0x10);
    drv_l1_tft_i80_wr_data(0x18);
    drv_l1_tft_i80_wr_data(0x2d);
    drv_l1_tft_i80_wr_data(0x28);
    drv_l1_tft_i80_wr_data(0x23);
    drv_l1_tft_i80_wr_data(0x28);
    drv_l1_tft_i80_wr_data(0x28);
    drv_l1_tft_i80_wr_data(0x26);
    drv_l1_tft_i80_wr_data(0x2f);
    drv_l1_tft_i80_wr_data(0x3b);
    drv_l1_tft_i80_wr_data(0x00);
    drv_l1_tft_i80_wr_data(0x03);
    drv_l1_tft_i80_wr_data(0x03);
    drv_l1_tft_i80_wr_data(0x10);

    drv_l1_tft_i80_wr_cmd(0x2a);   		// X address setup
    drv_l1_tft_i80_wr_data(0x00);
    drv_l1_tft_i80_wr_data(0x02);
    drv_l1_tft_i80_wr_data(0x00);
    drv_l1_tft_i80_wr_data(0x81);

    drv_l1_tft_i80_wr_cmd(0x2b);
    #if 1   //UPSIDE_DOWN == 0
    drv_l1_tft_i80_wr_data(0x00);
    drv_l1_tft_i80_wr_data(0x03);
    drv_l1_tft_i80_wr_data(0x00);
    drv_l1_tft_i80_wr_data(0x82);
    #else
    drv_l1_tft_i80_wr_data(0x00);	// Y address setup
    drv_l1_tft_i80_wr_data(0x03);
    drv_l1_tft_i80_wr_data(0x00);
    drv_l1_tft_i80_wr_data(0x82);
    #endif

    drv_l1_tft_i80_wr_cmd(0xf0);
    drv_l1_tft_i80_wr_data(0x01);

    drv_l1_tft_i80_wr_cmd(0xf6);
    drv_l1_tft_i80_wr_data(0x00);

    drv_l1_tft_i80_wr_cmd(0x3a);
    drv_l1_tft_i80_wr_data(0x05);

    /* Clear RAM */

    drv_l1_tft_i80_wr_cmd(0x2c);
    for(i = 0; i < 128 * 128; i++)
    {
        drv_l1_tft_i80_wr_data(0x00);
        drv_l1_tft_i80_wr_data(0x00);
    }

    drv_l1_tft_i80_wr_cmd(0x29);

    drv_l1_tft_i80_wr_cmd(0x2c);

 	drv_l1_tft_clk_set(TFT_CLK_DIVIDE_5);
	drv_l1_tft_mode_set(TFT_MODE_MEM_CONTI);
	drv_l1_tft_en_set(TRUE);
#endif

	return STATUS_OK;
}

// tft table
const DispCtrl_t TFT_Param =
{
	128,//240,//128,
	128,//320,//128,
	tft_txd_t144c_init
};
#endif //(defined GPM_LM765A0) && (GPM_LM765A0 == 1)
