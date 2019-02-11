#include "drv_l1_sfr.h"
#include "drv_l1_tft.h"
#include "drv_l1_gpio.h"
#include "drv_l2_display.h"

#if (defined ILI9806E_IDT) && (ILI9806E_IDT == 1)
static INT8U CSN_n;
static INT8U SCL_n;
static INT8U SDA_n;
static INT8U RESET_n;

static void cmd_delay(INT32U i)
{
	INT32U j, cnt;

	cnt = i*2;
	for (j=0;j<cnt;j++);
}

void serial_cmd_write(INT8U mode, INT8U cmd)
{
	INT32S i;
	INT16U temp;

	gpio_write_io(CSN_n, 1);//CS=1
	gpio_write_io(SCL_n,0);//SCL=0
	gpio_write_io(SDA_n,0);//SDA

	// set csn low
	gpio_write_io(CSN_n, 0);//CS=0
	cmd_delay(1);
	if(mode) // for data
		temp = (0x100|cmd);
	else
		temp = cmd;
	for (i=0;i<9;i++) {
		// shift data
		gpio_write_io(SDA_n, ((temp&0x100)>>8)); /* SDA */
		temp = (temp<<1);
		cmd_delay(1);
		// toggle clock
		gpio_write_io(SCL_n,1);//SCL=1
		cmd_delay(1);
		gpio_write_io(SCL_n,0);//SCL=0
		cmd_delay(1);
	}

	// set csn high
	gpio_write_io(CSN_n,1);//CS=1
	cmd_delay(1);
}

void serial_cmd_read(INT8U cmd, INT8U *read_data)
{
	int i,temp;

	serial_cmd_write(0,cmd);

	gpio_write_io(CSN_n, 1);//CS=1
	gpio_write_io(SCL_n,0);//SCL=0
	gpio_write_io(SDA_n,0);//SDA

	// set csn low
	gpio_write_io(CSN_n, 0);//CS=0
	cmd_delay(1);

	for (i=0;i<8;i++) {
		gpio_write_io(SCL_n,0);			/* SCL0 */
		cmd_delay(1);
		gpio_write_io(SCL_n,1);			/* SCL1 */
		temp = gpio_read_io(SDA_n);
		*read_data <<= 1;
		*read_data |= temp;
		cmd_delay(1);
	}

	// set csn high
	gpio_write_io(CSN_n,1);//CS=1
	cmd_delay(1);
}

static INT32S ILI9806E_IDT_IDT_init(void)
{
	INT8U temp;

	serial_cmd_read(0xDB,&temp);
	DBG_PRINT("serial_cmd_read = 0xDA %d \r\n",temp);

	return STATUS_OK;
}

static INT32S tft_ILI9806E_IDT_init(void)
{

#if (BOARD_TYPE == BOARD_EMU_GPB51PG)
	/* 320*240 */
    CSN_n = IO_A10;
	SDA_n = IO_A9;
    SCL_n = IO_A8;
#elif (BOARD_TYPE == BOARD_DVP_GPB51PG)
    CSN_n = IO_D6;
	SDA_n = IO_D8;
    SCL_n = IO_D7;
#endif
	//board
#if 1
	RESET_n = IO_C10;//IO_D13;//IO_D9;
 	CSN_n = IO_D13;//IO_D1;//IO_D8;
	SDA_n = IO_D4;//IO_D2;//IO_D6;
    SCL_n = IO_D5;//IO_D7;
#endif

 	gpio_init_io (RESET_n, GPIO_OUTPUT);
	gpio_set_port_attribute(RESET_n, 1);
	gpio_write_io(RESET_n, 1);//SCL=0
	cmd_delay(10);
	gpio_write_io(RESET_n, 0);//SCL=0
	cmd_delay(50);
	gpio_write_io(RESET_n, 1);//SCL=0
	cmd_delay(200);
#

	R_TFT_V_PERIOD = 1005;
	//R_TFT_VS_WIDTH = 5;
	R_TFT_V_START  = 20;
	R_TFT_V_END	   = 874;

	R_TFT_H_PERIOD = 525;
	//R_TFT_HS_WIDTH = 3;
	R_TFT_H_START  = 5;
	R_TFT_H_END	   = 484;

	//R_TFT_VS_START = 19;
	//R_TFT_VS_END   = 874;
	//R_TFT_HS_START = 7;
	//R_TFT_HS_END   = 485;
    //R_TFT_LINE_RGB_ORDER    = 0x00;

    (*((volatile INT32U *) 0xD02000CC)) &= 0xffffff00; //tft bus priority
    (*((volatile INT32U *) 0xD02000CC)) |= 0x00000005;
    (*((volatile INT32U *) 0xD05001FC)) |= 0x01000000; //tft long burst

	//R_TFT_TS_MISC = 1;

	drv_l1_tft_rb_switch_set(TRUE);
	drv_l1_tft_data_mode_set(TFT_DATA_MODE_888);
	drv_l1_tft_dclk_sel_set(TFT_DCLK_SEL_90);
	drv_l1_tft_vsync_unit_set(TRUE);
	drv_l1_tft_signal_inv_set(TFT_VSYNC_INV|TFT_HSYNC_INV, (TFT_ENABLE & TFT_VSYNC_INV)|(TFT_ENABLE & TFT_HSYNC_INV));
	drv_l1_tft_mode_set(TFT_MODE_PARALLEL);
    drv_l1_tft_clk_set(TFT_CLK_DIVIDE_7);//5
    drv_l1_tft_en_set(TRUE);

	return STATUS_OK;
}

// tft table
const DispCtrl_t TFT_Param =
{
	480,
	854,
	tft_ILI9806E_IDT_init
};
#endif //(defined TPO_TD025THD1) && (TPO_TD025THD1 == 1)
