#include "drv_l1_sfr.h"
#include "drv_l1_tft.h"
#include "drv_l1_gpio.h"
#include "drv_l2_display.h"

#if (defined AUO_A027DTN019) && (AUO_A027DTN019 == 1)
static INT8U CSN_n;
static INT8U SCL_n;
static INT8U SDA_n;
#define TFT_TAR_CLK      24000000

static void cmd_delay(INT32U i)
{
	INT32U j, cnt;

	cnt = i*8;
	for (j=0;j<cnt;j++);
}

static void serial_cmd_1(INT32U cmd)
{
	INT32S i;

	gpio_write_io(CSN_n, 1);//CS=1
	gpio_write_io(SCL_n, 0);//SCL=0
	gpio_write_io(SDA_n, 0);//SDA

	// set csn low
	gpio_write_io(CSN_n, 0);//CS=0
	cmd_delay(1);
	for (i=0;i<16;i++) {
		// shift data
		gpio_write_io(SDA_n, ((cmd&0x8000)>>15)); /* SDA */
		cmd = (cmd<<1);
		cmd_delay(1);
		// toggle clock
		gpio_write_io(SCL_n, 1);//SCL=0
		cmd_delay(1);
		gpio_write_io(SCL_n, 0);//SCL=0
		cmd_delay(1);
	}

	// set csn high
	gpio_write_io(CSN_n,1);//CS=1
	cmd_delay(1);
}

const INT8U AUO_REGTable[]={


	0x05,0x44,
	0x05,0x04,
	0x05,0x44,

	0x01,0xb4,//ups 052 mode
    0x00,0x23,
	0x04,0x1b,

	0x05,0x45,
//	0x05,0x5d,
//	0x85,0x80,
	0XFF,0XFF
};
static INT32S tft_auo_a027dtn019_init(void)
{

#if 1//(BOARD_TYPE == BOARD_EMU_GPB51PG)
	/* 320*240 */
    CSN_n = IO_A8;
	SDA_n = IO_A9;
    SCL_n = IO_A10;
#elif 0//(BOARD_TYPE == BOARD_DVP_GPB51PG)
    CSN_n = IO_D7;//IO_D6;
	SDA_n = IO_D8;
    SCL_n = IO_D9;//IO_D7;
#elif 0//(BOARD_TYPE == BOARD_DVP_GPB51PG)
    CSN_n = IO_A12;//IO_D6;
	SDA_n = IO_A11;
    SCL_n = IO_A10;//IO_D7;
#endif

    gpio_init_io (CSN_n, GPIO_OUTPUT);
	gpio_init_io (SCL_n, GPIO_OUTPUT);
	gpio_init_io (SDA_n, GPIO_OUTPUT);

	gpio_set_port_attribute(CSN_n, 1);
	gpio_set_port_attribute(SCL_n, 1);
	gpio_set_port_attribute(SDA_n, 1);



	INT8U i =0;
    while(AUO_REGTable[i] !=0xff)
    {
    serial_cmd_1( (AUO_REGTable[i]<<8) |(AUO_REGTable[i+1]) );
    i+=2;
    }



	R_TFT_HS_WIDTH			= 1;				//	1		=HPW
	R_TFT_H_START			= 1+240;			//	240		=HPW+HBP
	R_TFT_H_END				= 1+240+1280;		//	1520	=HPW+HBP+HDE
	R_TFT_H_PERIOD			= 1560;            	//	1560	=HPW+HBP+HDE+HFP
	R_TFT_VS_WIDTH			= 1;				//	1		=VPW				(DCLK)
	R_TFT_V_START			= 21;				//	21		=VPW+VBP			(LINE)
	R_TFT_V_END				= 21+240;			//	261		=VPW+VBP+VDE		(LINE)
	R_TFT_V_PERIOD			= 1+21+240;			//	262		=VPW+VBP+VDE+VFP	(LINE)
	R_TFT_LINE_RGB_ORDER    = 0x00;

	drv_l1_tft_signal_inv_set(TFT_VSYNC_INV|TFT_HSYNC_INV, (TFT_ENABLE & TFT_VSYNC_INV)|(TFT_ENABLE & TFT_HSYNC_INV));
	drv_l1_tft_mode_set(TFT_MODE_UPS052);
	drv_l1_tft_data_mode_set(TFT_DATA_MODE_8);
	drv_l1_tft_target_clk_set(TFT_TAR_CLK);
	drv_l1_tft_en_set(TRUE);
	return STATUS_OK;
}

// tft table
const DispCtrl_t TFT_Param =
{
	320,
	240,
	tft_auo_a027dtn019_init
};
#endif //(defined AUO_A027DTN019) && (AUO_A027DTN019 == 1)
