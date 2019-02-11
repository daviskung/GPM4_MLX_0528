#include "drv_l1_sfr.h"
#include "drv_l1_tft.h"
#include "drv_l1_gpio.h"
#include "drv_l2_display.h"

#if (defined AUO_A043Fl01) && (AUO_A043Fl01 == 1)
#define TFT_TAR_CLK      9000000

static INT32S tft_auo_a043fl01_init(void)
{
	R_TFT_V_PERIOD = 288;
	R_TFT_V_START = 12;
	R_TFT_V_END	= 284;

	R_TFT_H_PERIOD = 533;
	R_TFT_H_START = 45;
	R_TFT_H_END	= 524;

	R_TFT_VS_END = 0;
	R_TFT_HS_END = 0;

	drv_l1_tft_rb_switch_set(TRUE);
	drv_l1_tft_data_mode_set(TFT_DATA_MODE_888);
	drv_l1_tft_dclk_sel_set(TFT_DCLK_SEL_90);
	drv_l1_tft_vsync_unit_set(TRUE);
	drv_l1_tft_signal_inv_set(TFT_VSYNC_INV|TFT_HSYNC_INV, (TFT_ENABLE & TFT_VSYNC_INV)|(TFT_ENABLE & TFT_HSYNC_INV));
	drv_l1_tft_mode_set(TFT_MODE_PARALLEL);
    drv_l1_tft_target_clk_set(TFT_TAR_CLK);
    drv_l1_tft_en_set(TRUE);

	return STATUS_OK;
}

// tft table
const DispCtrl_t TFT_Param =
{
	480,
	272,
	tft_auo_a043fl01_init
};
#endif //(defined TPO_TD025THD1) && (TPO_TD025THD1 == 1)
