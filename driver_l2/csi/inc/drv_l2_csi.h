#ifndef __drv_l2_CSI_H__
#define __drv_l2_CSI_H__

#include "drv_l2.h"
#include "drv_l2_sensor.h"

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/


/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct gpCsiFmt_s
{
	INT32U interface_mode;      /* 0:CCIR601, 1:CCIR656 mode */
	INT32U preview_mode;        /* 0:preview, 1:capture mode */
	INT32U interlace_mode;      /* 0:non-interlace, 1:interlace mode */
	INT32U fieldinv;       /* 0:not invert field input, 1:invert field input mode */
	INT32U input_format;
	INT32U output_format;
	INT32U bs_mode;             /* 0:disable blue screen, 1:enable blue screen */
	INT32U jpeg_mode;           /* 0:Disable JPEG, 1:Enable JPEG Valid mode */
	INT32U fifo_size;           /* 0:No FIFO, 1:8 lines FIFO, 2:16 lines FIFO, 3:32 lines FIFO mode */
	INT32U cubicen;             /* 0:line output, 1:cubic output mode */
	INT32U cubicen_size;        /* 0:64x64, 1:32x32 */
	INT32U cuten;               /* 0:disable cut, 1:enable cut function */
	INT32U clk_pri;             /* 0:Priority is less than display, 1:Priority is over display */
	INT32U clk_stopb;           /* 0:stop clock output when busy, 1:not stop clock output when busy */
	INT32U sen_bldlvl;          /* 0: Fully transparen, 1: 1/64 transparent,...63: 63/64 transparent */
	INT32U sen_bld;             /* 0:Disable sensor blending function, 1:Enable sensor blending function */
	INT32U sen_lvl;             /* 0:Layer 0 (TEXT Layer 0), 1:Layer 1 (Sprite Layer 0),  Layer 2 (TEXT Layer 1), Layer 3 (Sprite Layer 1), Layer 4 (TEXT Layer 2), Layer 5 (Sprite Layer 2), Layer 6 (TEXT Layer 3), Layer 7 (Sprite Layer 3) */
} gpCsiFmt_t;

typedef struct gpCsiTim_s
{
    INT32U hl_start;
	INT32U vl0_start;
	INT32U vl1_start;
    INT32U vrst_mode;           /* 0:reset at the falling edge of VSYNC, 1:reset at the rising edge of VSYNC */
    INT32U vadd_mode;           /* 0:add at the falling edge of HSYNC, 1:add at the rising edge of HSYNC */
    INT32U hrst_mode;           /* 0:reset at the falling edge of HSYNC, 1:reset at the rising edge of HSYNC */
    INT32U fget_mode;           /* 0:latch field at falling edge of VSYHC, 1:latch field at rising edge of VSYHC */
	INT32U clki_inv;            /* 0:not invert input clock, 1:invert input clock */
	INT32U clko_inv;            /* 0:not invert output clock, 1:invert output clock */
	INT32U d_type;              /* 0:Delay 1 clock, 1:Delay 2 clocks, 2:Delay 3 clocks, 4:Delay 4 clocks */
} gpCsiTim_t;

typedef struct gpCSIPara_s
{
 	INT32U target_w; 			/* sensor width */
	INT32U target_h;			/* sensor height */
	INT32U h_start;
	INT32U h_end;
	INT32U v_start;
	INT32U v_end;
    INT32U hratio;
	INT32U vratio;
	INT32U cut_start;
	INT32U cut_size;
	INT32U bs_upper;
	INT32U bs_lower;
    gpCsiFmt_t csi_fmt;
    gpCsiTim_t csi_tim;
} gpCSIPara_t;

extern INT32S drv_l2_csi_set_fmt(gpCsiFmt_t *pFmt);
extern INT32S drv_l2_csi_stream_on(gpCSIPara_t *csi_info, INT32U bufA, INT32U bufB);
extern INT32S drv_l2_csi_stream_off(void);
extern INT32S drv_l2_csi_stop(void);
#endif	/*__drv_l2_CDSP_H__*/
