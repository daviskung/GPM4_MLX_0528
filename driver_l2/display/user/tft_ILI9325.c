#include <string.h>
#include "drv_l1_sfr.h"
#include "drv_l1_tft.h"
#include "drv_l1_gpio.h"
#include "drv_l1_timer.h"
#include "drv_l2_display.h"

#if (defined ILI9325) && (ILI9325 == 1)
#if BOARD_TYPE == BOARD_GPM41XXA_EMU_V1_0
#define RST_PAD     IO_D5
#else
#define RST_PAD     IO_A15
#endif
#define TFT_TAR_CLK      27000000

static void delay(int ms)
{
#if 1
	drv_msec_wait(ms);
#else
	int i;
	for(i = 0; i < 4800 * ms; i++){};
#endif
}
static void write_16bit_command(unsigned short command)
{
	drv_l1_tft_i80_wr_cmd((command >> 8) &0xff);
	delay(1);
	drv_l1_tft_i80_wr_cmd(command &0xff);
}

static void write_16bit_parameter(unsigned short parameter)
{
	drv_l1_tft_i80_wr_data((parameter >> 8)& 0xff);
	delay(1);
	drv_l1_tft_i80_wr_data(parameter& 0xff);

}

static void write_16bit(unsigned short command,unsigned short parameter)
{
	write_16bit_command(command);
	write_16bit_parameter(parameter);
}

/*static void t80_trans(INT8U flag)
{
	INT8U data = 0x80;
	if(flag & T80_MIRROR) data &= ~(1 << 7);
	if(flag & T80_FLIP) data |= 1 << 6;

	write_command(0x36);
	write_parameter(data);
}*/
static void lcd_reset(void)
{
	gpio_init_io(RST_PAD, GPIO_OUTPUT);
	gpio_set_port_attribute(RST_PAD, ATTRIBUTE_HIGH);
	gpio_write_io(RST_PAD, DATA_HIGH);
	delay(1);
	gpio_write_io(RST_PAD, DATA_LOW);
	delay(10);
	gpio_write_io(RST_PAD, DATA_HIGH);
	delay(120);
}
static INT32S tft_ILI9325_init(void)
{
		//INT32U i;

    R_TFT_CTRL = 0;
		lcd_reset();

    drv_l1_tft_mem_unit_set(8);
		drv_l1_tft_clk_set(TFT_CLK_DIVIDE_3);
		R_TFT_LINE_RGB_ORDER |= 0x0080; //memory mode

		/* TFT interface parameter */
		R_TFT_V_PERIOD = 240-1;//320-1;//240-1;
		R_TFT_VS_WIDTH = 20;
		R_TFT_V_START  = 5;
		R_TFT_H_PERIOD = 320-1;//240-1;//320-1;
		R_TFT_HS_WIDTH = 4;
		R_TFT_H_START  = 4;

		R_TFT_HS_START = 0;
		R_TFT_HS_END   = 320-1;//239;//319;
		R_TFT_VS_START = 0;
		R_TFT_VS_END   = 240;//320;//240;



/*
		R_TFT_VS_WIDTH = 20;  //20;
		R_TFT_V_PERIOD = 240-1;//320;//240-1;
		R_TFT_V_START = 5;

		R_TFT_HS_WIDTH = 4;
		R_TFT_H_PERIOD = 320-1;//240;//320-1;
		R_TFT_H_START = 4;
*/

    R_TFT_TS_MISC= 0x00;

	//write_16bit(0x0001, 0x0100);
	write_16bit(0x0001, 0x0100);
	write_16bit(0x0002, 0x0700);

	write_16bit(0x0003, 0x1098);//0x1098);//0x1030);

	write_16bit(0x0004, 0x0000);
	write_16bit(0x0008, 0x0207);
	write_16bit(0x0009, 0x0000);
	write_16bit(0x000A, 0x0000);
	write_16bit(0x000C, 0x0000);
	write_16bit(0x000D, 0x0000);
	write_16bit(0x000F, 0x0000);

	//------------------power on sequence--------------------//
	write_16bit(0x0010, 0x0000);
	write_16bit(0x0011, 0x0007);
	write_16bit(0x0012, 0x0000);
	write_16bit(0x0013, 0x0000);
	write_16bit(0x0007, 0x0001);
	delay(100);
	write_16bit(0x0010, 0x1490);
	write_16bit(0x0011, 0x0227);
	delay(20);
	write_16bit(0x0012, 0x001A);
	delay(20);
	write_16bit(0x0013, 0x1400);
	write_16bit(0x0029, 0x0019);
	write_16bit(0x002B, 0x000C);   //frame rate
	//write_16bit(0x002B, 0x000D);   //frame rate
	delay(20);

	//write_16bit(0x0020, 0x00EF);  //GRAM address set
	write_16bit(0x0020, 0x0000);  //GRAM address set
	write_16bit(0x0021, 0x0000);

	//--------------adjust the gamma curve------------//
//	write_16bit(0x0030, 0x0000);
//	write_16bit(0x0031, 0x0607);
//	write_16bit(0x0032, 0x0305);
//	write_16bit(0x0035, 0x0000);
//	write_16bit(0x0037, 0x0204);
//	write_16bit(0x0038, 0x0001);
//	write_16bit(0x0039, 0x0707);
//	write_16bit(0x003C, 0x0000);
//	write_16bit(0x003D, 0x000F);
// from ming
	write_16bit(0x0030, 0x0003);
	write_16bit(0x0031, 0x0705);
	write_16bit(0x0032, 0x0007);
	write_16bit(0x0035, 0x0007);
    write_16bit(0x0036, 0x000F);
	write_16bit(0x0037, 0x0007);
	write_16bit(0x0038, 0x0200);
	write_16bit(0x0039, 0x0407);
	write_16bit(0x003C, 0x0700);
	write_16bit(0x003D, 0x1604);


	//----------set gamma area--------------//
	write_16bit(0x0050, 0x0000);
	write_16bit(0x0053, 0x013F);
	//write_16bit(0x0051, 0x00EF);
	write_16bit(0x0052, 0x0000);
	write_16bit(0x0051, 0x00EF);
	//write_16bit(0x0053, 0x013F);

	write_16bit(0x0060, 0xA700);
	write_16bit(0x0061, 0x0001);
	write_16bit(0x006A, 0x0000);

	//--------------partial display control-------------//
	write_16bit(0x0080, 0x0000);
	write_16bit(0x0081, 0x0000);
	write_16bit(0x0082, 0x0000);
	write_16bit(0x0083, 0x0000);
	write_16bit(0x0084, 0x0000);
	write_16bit(0x0085, 0x0000);

	//-------------panel control-----------------//
	write_16bit(0x0090, 0x0010);
	write_16bit(0x0092, 0x0600);
	write_16bit(0x0007, 0x0133);
	//delay(120);
	write_16bit_command(0x0022);
    //write_command(0x29);
	//write_command(0x2c);


		//DBG_PRINT("LCD_ILI9325_Init_end\r\n");
    //R_PPU_ENABLE = 1 << 7;
    //R_TFT_CTRL =  0x20c0 | (7);
    drv_l1_tft_target_clk_set(TFT_TAR_CLK);

	drv_l1_tft_mode_set(TFT_MODE_MEM_CONTI);
	drv_l1_tft_en_set(TRUE);

	return STATUS_OK;
}
// tft table
const DispCtrl_t TFT_Param =
{
	320,
	240,
	tft_ILI9325_init
};
#endif //(defined ILI9325) && (ILI9325 == 1)
