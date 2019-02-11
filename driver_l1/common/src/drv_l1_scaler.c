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
#include "drv_l1_sfr.h"
#include "drv_l1_clock.h"
#include "drv_l1_scaler.h"

#if (defined _DRV_L1_SCALER) && (_DRV_L1_SCALER == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#ifndef DISABLE
#define DISABLE     0
#endif

#ifndef ENABLE
#define ENABLE      1
#endif

#ifdef NULL
#undef NULL
#endif
#define NULL                    0
#define C_SCALER_QUEUE_SIZE		5

#define SCALER_CLOCK_SOURCE     28

// Out-of-boundry color register
#define C_SCALER_OUT_BOUNDRY_COLOR_MAX		0x00FFFFFF

// Output width and height registers
#define C_SCALER_OUT_WIDTH_MAX				0x00001FFF		// Maximum 8191 pixels
#define C_SCALER_OUT_HEIGHT_MAX				0x00001FFF		// Maximum 8191 pixels

// Scaler factor registers
#define C_SCALER_X_FACTOR_MAX				0x00FFFFFF
#define C_SCALER_Y_FACTOR_MAX				0x00FFFFFF

// Scaler input x/y start offset registers
#define C_SCALER_X_START_MAX				0x3FFFFFFF
#define C_SCALER_Y_START_MAX				0x3FFFFFFF

/* Scaler out x offset registers */
#define C_SCALER_OUT_X_OFFSET_MAX			0x00001FFF

// Input width and height registers
#define C_SCALER_IN_WIDTH_MAX				0x00001FFF		// Maximum 8191 pixels
#define C_SCALER_IN_HEIGHT_MAX				0x00001FFF		// Maximum 8191 pixels

// Input width and height registers
#define C_SCALER_IN_VISIBLE_WIDTH_MAX		0x00001FFF		// Maximum 8191 pixels
#define C_SCALER_IN_VISIBLE_HEIGHT_MAX		0x00001FFF		// Maximum 8191 pixels

// macro
#define SCALER0_Y_GAMMA(x)			*((volatile INT32U *) (SCALER0_BASE + 0x400 + x))

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#define OSQFlush(x)\
{\
    while(1) {\
        osEvent result;\
        result = osMessageGet(x, 1);\
        if(result.status != osEventMessage) {\
            break;\
        }\
    }\
}

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct
{										// Offset
	volatile INT32U	CTRL;           	// 0x0000
	volatile INT32U	OUT_BOND_COLOR;		// 0x0004
	volatile INT32U	OUT_WIDTH;			// 0x0008
	volatile INT32U	OUT_HEIGHT;			// 0x000C
	volatile INT32U	X_FACTOR;			// 0x0010
	volatile INT32U	Y_FACTOR;			// 0x0014
	volatile INT32U	X_START;			// 0x0018
	volatile INT32U	Y_START;			// 0x001C
	volatile INT32U	IN_WIDTH;			// 0x0020
	volatile INT32U	IN_HEIGHT;			// 0x0024
	volatile INT32U	IN_Y_ADDR;			// 0x0028
	volatile INT32U	IN_U_ADDR;			// 0x002C
	volatile INT32U	IN_V_ADDR;			// 0x0030
	volatile INT32U	OUT_Y_ADDR;			// 0x0034
	volatile INT32U	OUT_U_ADDR;			// 0x0038
	volatile INT32U	OUT_V_ADDR;			// 0x003C
	volatile INT32U	CURRENT_LINE;		// 0x0040
	volatile INT32U	A11;				// 0x0044
	volatile INT32U	A12;				// 0x0048
	volatile INT32U	A13;				// 0x004C
	volatile INT32U	A21;				// 0x0050
	volatile INT32U	A22;				// 0x0054
	volatile INT32U	A23;				// 0x0058
	volatile INT32U	A31;				// 0x005C
	volatile INT32U	A32;				// 0x0060
	volatile INT32U	A33;				// 0x0064
	volatile INT32U	IN_VISIBLE_WIDTH;	// 0x0068
	volatile INT32U	IN_VISIBLE_HEIGHT ;	// 0x006C
	volatile INT32U	OUT_X_OFFSET;		// 0x0070
	volatile INT32U	LINE_BUFFER_ADDR;	// 0x0074
	volatile INT32U	RESERVE0[1];
	volatile INT32U	INT_FLAG;			// 0x007C
	volatile INT32U	POST;				// 0x0080
	volatile INT32U	MAX_Y;				// 0x0084
	volatile INT32U	MIN_Y;				// 0x0088
	volatile INT32U	SUM_Y;				// 0x008C
	volatile INT32U	MAX_U;				// 0x0090
	volatile INT32U	MIN_U;				// 0x0094
	volatile INT32U	SUM_U;				// 0x0098
	volatile INT32U	MAX_V;				// 0x009C
	volatile INT32U	MIN_V;				// 0x00A0
	volatile INT32U	SUM_V;				// 0x00A4
	volatile INT32U	RESERVE1[6];		// 0x00A8 ~ 0x00BC
	volatile INT32U	Y_HIS[16];			// 0x00C0 ~ 0x00FC
	volatile INT32U	STATUS_WIRTE1;		// 0x0100
	volatile INT32U	STATUS_READ1;		// 0x0104
	volatile INT32U	STATUS_READ2;		// 0x0108
	volatile INT32U	RESERVE2[1];
	volatile INT32U	IN_Y_ADDR_B;		// 0x0110
	volatile INT32U	IN_U_ADDR_B;		// 0x0114
	volatile INT32U	IN_V_ADDR_B;		// 0x0118
} SCALER_SFR;

typedef struct ScalerDev_s
{
	INT8U init_done;
	INT8U started_flag;
	INT8U bypass_zoom_mode;
	INT8U RSD;
	INT32U register_backup1;
	INT32U register_backup2;
	volatile INT32U interrupt_flag;

	osMessageQId message_queue;
	osMessageQId scaler_q;
	osSemaphoreId scaler_hw_sem;
} ScalerDev_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static ScalerDev_t scaler_dev[SCALER_MAX];
static void (*scaler_isr_callback)(INT32U scaler0_event, INT32U scaler1_event);
static INT32U (*scaler_new_int_callback)(void);

static INT16S drv_l1_scaler_clock_set(INT16U enable)
{
    return drv_l1_clock_set_system_clk_en(SCALER_CLOCK_SOURCE, enable);
}

static SCALER_SFR *_get_scaler_sfr_base(INT8U scaler_num)
{
#if SCALER0_EN == 1
	if (scaler_num == SCALER_0) {
		return (SCALER_SFR *)SCALER0_BASE;
	}
#endif

#if SCALER1_EN == 1
	if (scaler_num == SCALER_1) {
		return (SCALER_SFR *)SCALER1_BASE;
	}
#endif

	return 0;
}

static ScalerDev_t *_get_scaler_device(INT8U scaler_num)
{
#if SCALER0_EN == 1
	if (scaler_num == SCALER_0) {
		return &scaler_dev[SCALER_0];
	}
#endif

#if SCALER1_EN == 1
	if (scaler_num == SCALER_1) {
		return &scaler_dev[SCALER_1];
	}
#endif

	return 0;
}

void SCALAR0_IRQHandler(void)							// Device ISR
{
	INT32U i, flag, event;
	SCALER_SFR *pScaler;
	ScalerDev_t *pScalerDev;

	for(i=0; i<SCALER_MAX; i++) {
		event = 0;
		pScaler = _get_scaler_sfr_base(i);
		pScalerDev = _get_scaler_device(i);
		flag = pScaler->INT_FLAG;
		pScaler->INT_FLAG = flag;

		if ((flag & C_SCALER_INT_PEND) && (pScaler->CTRL & C_SCALER_CTRL_INT_ENABLE)) {
			// Scaler completed or input FIFO is empty
			if (pScalerDev->bypass_zoom_mode) {			// Send Done message to JPEG if bypass zoom mode is enabled
				//jpeg_using_scaler_decode_done();
			} else if (flag & C_SCALER_INT_DONE) {		// Scaler finished its job
				event |= C_SCALER_STATUS_DONE;
			} else {
				event |= C_SCALER_STATUS_INPUT_EMPTY;
			}
		} else if ((flag & C_SCALER_INT_OUT_FULL) && (pScaler->CTRL & C_SCALER_CTRL_OUT_FIFO_INT)) {
			// Output FIFO is full
			event |= C_SCALER_STATUS_OUTPUT_FULL;
		}

		pScalerDev->interrupt_flag |= event;
	#if 1
		if(pScalerDev->message_queue && event) {
            osMessagePut(pScalerDev->message_queue, (uint32_t)&event, osWaitForever);
		}
	#endif
	}



	if(scaler_isr_callback) {
		INT32U scale0_flag = 0, scale1_flag = 0;
	#if SCALER0_EN == 1
		scale0_flag = scaler_dev[SCALER_0].interrupt_flag;
	#endif
	#if SCALER1_EN == 1
		scale1_flag = scaler_dev[SCALER_1].interrupt_flag;
	#endif
		(*scaler_isr_callback)(scale0_flag, scale1_flag);
	}
}

/**
 * @brief   lock scaler semaphore
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	none
 */
void drv_l1_scaler_lock(INT8U scaler_num)
{
#if 1
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);
	osSemaphoreWait(pScalerDev->scaler_hw_sem, osWaitForever);
#endif
}

/**
 * @brief   unlock scaler semaphore
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	none
 */
void drv_l1_scaler_unlock(INT8U scaler_num)
{
#if 1
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);
	osSemaphoreRelease(pScalerDev->scaler_hw_sem);
#endif
}

/**
 * @brief   Initiate Scaler module
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	none
 */
void drv_l1_scaler_init(INT8U scaler_num)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

    drv_l1_scaler_clock_set(ENABLE);

	pScaler->CTRL = C_SCALER_CTRL_RESET;	// Reset scaler
	pScaler->CTRL = 0;						// Disable scaler
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;	// Clear interrupt pending flag
	pScaler->POST = 0;						// Disable color matrix and gamma function

	drv_l1_scaler_input_visible_pixels_set(scaler_num,0, 0);
	drv_l1_scaler_YUV_type_set(scaler_num,C_SCALER_CTRL_TYPE_YCBCR);
	drv_l1_scaler_out_of_boundary_mode_set(scaler_num,0);
	drv_l1_scaler_out_of_boundary_color_set(scaler_num,0x008080);
	drv_l1_scaler_input_offset_set(scaler_num,0, 0);

	pScalerDev->started_flag = 0;
	pScalerDev->bypass_zoom_mode = 0;
	pScalerDev->interrupt_flag = 0;

	if (pScalerDev->init_done == 0) {
		pScalerDev->init_done = 1;
	#if 1
		pScalerDev->message_queue = NULL;

		if(pScalerDev->scaler_q == NULL) {
			osMessageQDef_t scalar_q = { C_SCALER_QUEUE_SIZE, sizeof(INT32U), 0 };

			pScalerDev->scaler_q = osMessageCreate(&scalar_q, NULL);
		}

		if(pScalerDev->scaler_hw_sem == NULL) {
			osSemaphoreDef_t scalar_sem = { 0 };

			pScalerDev->scaler_hw_sem = osSemaphoreCreate(&scalar_sem, 1);
		}
	#endif

        NVIC_SetPriority(SCALAR0_IRQn, 5);
        NVIC_EnableIRQ(SCALAR0_IRQn);
	}
}

/**
 * @brief   UN-Initiate Scaler module
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	none
 */
void drv_l1_scaler_uninit(INT8U scaler_num)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	pScaler->CTRL = C_SCALER_CTRL_RESET;	// Reset scaler
	pScaler->CTRL = 0;						// Disable scaler
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;	// Clear interrupt pending flag
	pScaler->POST = 0;
	drv_l1_scaler_clock_set(DISABLE);
    NVIC_DisableIRQ(SCALAR0_IRQn);
}

/**
 * @brief   Set input image pixel and output pixel(1~8000)
 * @param   scaler_num[in]: SCALER_NUM
 * @param   input_x[in]: intput image width
 * @param   input_y[in]: intput image heigth
 * @param   output_x[in]: output image width
 * @param   output_u[in]: output image heigth
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_image_pixels_set(INT8U scaler_num,INT32U input_x, INT32U input_y, INT32U output_x, INT32U output_y)
{
	INT32U factor;
	INT32S ret;
	SCALER_SFR *pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	if (output_x & 0x7) {		// Make sure output width is at least multiple of 8 pixels
		output_x &= ~0x7;
		ret = -1;
	}
	if (!input_x || !input_y || !output_x || !output_y) {
		input_x = 8;
		input_y = 8;
		output_x = 32;
		output_y = 8;
		ret = -1;
	}
	if (input_x > C_SCALER_IN_WIDTH_MAX) {
		input_x = C_SCALER_IN_WIDTH_MAX;
		ret = -1;
	}
	if (input_y > C_SCALER_IN_HEIGHT_MAX) {
		input_y = C_SCALER_IN_HEIGHT_MAX;
		ret = -1;
	}
	if (output_x > C_SCALER_OUT_WIDTH_MAX) {
		output_x = C_SCALER_OUT_WIDTH_MAX;
		ret = -1;
	}
	if (output_y > C_SCALER_OUT_HEIGHT_MAX) {
		output_y = C_SCALER_OUT_HEIGHT_MAX;
		ret = -1;
	}

	// Set scaler X factor
	factor = (input_x << 16) / output_x;
	if (factor > C_SCALER_X_FACTOR_MAX) {
		factor = C_SCALER_X_FACTOR_MAX;
		ret = -1;
	}
	pScaler->X_FACTOR = factor;

	// Set scaler Y factor
	factor = (input_y << 16) / output_y;
	if (factor > C_SCALER_Y_FACTOR_MAX) {
		factor = C_SCALER_Y_FACTOR_MAX;
		ret = -1;
	}
	pScaler->Y_FACTOR = factor;

	pScaler->IN_WIDTH = input_x - 1;
	pScaler->IN_HEIGHT= input_y - 1;
	pScaler->IN_VISIBLE_WIDTH = 0;
	pScaler->IN_VISIBLE_HEIGHT = 0;

	pScaler->OUT_WIDTH = output_x - 1;
	pScaler->OUT_HEIGHT = output_y - 1;

	return ret;
}

/**
 * @brief   Set scaler input resolution at the X&Y axis, 1~8000 pixels, including the padding pixels
 * @param   scaler_num[in]: SCALER_NUM
 * @param   input_x[in]: input image width
 * @param   input_y[in]: intput image heigth
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_input_pixels_set(INT8U scaler_num,INT32U input_x, INT32U input_y)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	if (!input_x || !input_y) {
		input_x = 8;
		input_y = 8;
		ret = -1;
	}
	if (input_x > C_SCALER_IN_WIDTH_MAX) {
		input_x = C_SCALER_IN_WIDTH_MAX;
		ret = -1;
	}
	if (input_y > C_SCALER_IN_HEIGHT_MAX) {
		input_y = C_SCALER_IN_HEIGHT_MAX;
		ret = -1;
	}
	pScaler->IN_WIDTH = input_x - 1;
	pScaler->IN_HEIGHT = input_y - 1;

	return ret;
}

/**
 * @brief   Set scaler real input resolution at the X&Y axis, 1~8000 pixels, not including the padding pixels
 * @param   scaler_num[in]: SCALER_NUM
 * @param   input_x[in]: input image width
 * @param   input_y[in]: intput image heigth
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_input_visible_pixels_set(INT8U scaler_num,INT32U input_x, INT32U input_y)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	if (input_x > C_SCALER_IN_VISIBLE_WIDTH_MAX) {
		input_x = C_SCALER_IN_VISIBLE_WIDTH_MAX;
		ret = -1;
	}
	if (input_y > C_SCALER_IN_VISIBLE_HEIGHT_MAX) {
		input_y = C_SCALER_IN_VISIBLE_HEIGHT_MAX;
		ret = -1;
	}

	if (input_x) {
		pScaler->IN_VISIBLE_WIDTH = input_x - 1;
		pScaler->CTRL |= C_SCALER_CTRL_VISIBLE_RANGE;
	} else {
		pScaler->IN_VISIBLE_WIDTH = 0;
		pScaler->CTRL &= ~C_SCALER_CTRL_VISIBLE_RANGE;
	}
	if (input_y) {
		pScaler->IN_VISIBLE_HEIGHT = input_y - 1;
	} else {
		pScaler->IN_VISIBLE_HEIGHT = 0;
		pScaler->CTRL &= ~C_SCALER_CTRL_VISIBLE_RANGE;
	}

	return ret;
}

/**
 * @brief   scaler set image input address
 * @param   scaler_num[in]: SCALER_NUM
 * @param   y_addr[in]: Start address of input Y data
 * @param   u_addr[in]: Start address of input U data
 * @param   v_addr[in]: Start address of input V data
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_input_A_addr_set(INT8U scaler_num,INT32U y_addr, INT32U u_addr, INT32U v_addr)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	// Make sure all addresses are 4-byte alignment
	if ((y_addr & 0x3) || (u_addr & 0x3) || (v_addr & 0x3)) {
		y_addr &= ~0x3;
		u_addr &= ~0x3;
		v_addr &= ~0x3;
		ret = -1;
	}

	pScaler->IN_Y_ADDR = y_addr;
	pScaler->IN_U_ADDR = u_addr;
	pScaler->IN_V_ADDR = v_addr;
	return ret;
}

/**
 * @brief   scaler set image input address
 * @param   scaler_num[in]: SCALER_NUM
 * @param   y_addr[in]: Start address of input Y data
 * @param   u_addr[in]: Start address of input U data
 * @param   v_addr[in]: Start address of input V data
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_input_B_addr_set(INT8U scaler_num,INT32U y_addr, INT32U u_addr, INT32U v_addr)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	// Make sure all addresses are 4-byte alignment
	if ((y_addr & 0x3) || (u_addr & 0x3) || (v_addr & 0x3)) {
		y_addr &= ~0x3;
		u_addr &= ~0x3;
		v_addr &= ~0x3;
		ret = -1;
	}

	pScaler->IN_Y_ADDR_B = y_addr;
	pScaler->IN_U_ADDR_B = u_addr;
	pScaler->IN_V_ADDR_B = v_addr;

	return ret;
}

/**
 * @brief   Set input image format
 * @param   scaler_num[in]: SCALER_NUM
 * @param   format[in]: input format
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_input_format_set(INT8U scaler_num, INT32U format)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if ((format != C_SCALER_CTRL_IN_RGB1555) &&
		(format != C_SCALER_CTRL_IN_RGBG) &&
		(format != C_SCALER_CTRL_IN_GRGB) &&

		(format != C_SCALER_CTRL_IN_RGB565) &&
		(format != C_SCALER_CTRL_IN_YUYV) &&
		(format != C_SCALER_CTRL_IN_UYVY) &&
		(format != C_SCALER_CTRL_IN_VYUY) &&

		(format != C_SCALER_CTRL_IN_YUYV8X32) &&
		(format != C_SCALER_CTRL_IN_YUYV8X64) &&
		(format != C_SCALER_CTRL_IN_YUYV16X32) &&
		(format != C_SCALER_CTRL_IN_YUYV16X64) &&
		(format != C_SCALER_CTRL_IN_YUYV32X32) &&
		(format != C_SCALER_CTRL_IN_YUYV64X64) &&
		(format != C_SCALER_CTRL_IN_VYUY8X32) &&
		(format != C_SCALER_CTRL_IN_VYUY8X64) &&
		(format != C_SCALER_CTRL_IN_VYUY16X32) &&
		(format != C_SCALER_CTRL_IN_VYUY16X64) &&
		(format != C_SCALER_CTRL_IN_VYUY32X32) &&

		(format != C_SCALER_CTRL_IN_YUV422) &&
		(format != C_SCALER_CTRL_IN_YUV420) &&
		(format != C_SCALER_CTRL_IN_YUV411) &&
		(format != C_SCALER_CTRL_IN_YUV444) &&
		(format != C_SCALER_CTRL_IN_Y_ONLY) &&
		(format != C_SCALER_CTRL_IN_YUV422V) &&
		(format != C_SCALER_CTRL_IN_YUV411V) &&

		(format != C_SCALER_CTRL_IN_ARGB4444) &&
		(format != C_SCALER_CTRL_IN_ARGB8888) &&
        (format != C_SCALER_CTRL_IN_ARGB4444X32) &&
        (format != C_SCALER_CTRL_IN_ARGB4444X64) ) {
		return -1;
	}

        switch(format)
        {
            case C_SCALER_CTRL_IN_ARGB4444X32:
                      pScaler->CTRL &= ~(C_SCALER_CTRL_IN_MASK);
                      pScaler->CTRL |= C_SCALER_CTRL_IN_YUYV32X32;
                      pScaler->POST |= C_SCALER_NEW_INPUT_COLOR_MODE;
                      break;

            case C_SCALER_CTRL_IN_ARGB4444X64:
                      pScaler->CTRL &= ~(C_SCALER_CTRL_IN_MASK);
                      pScaler->CTRL |= C_SCALER_CTRL_IN_YUYV64X64;
                      pScaler->POST |= C_SCALER_NEW_INPUT_COLOR_MODE;
                      break;

            default:
                pScaler->CTRL &= ~(C_SCALER_CTRL_IN_MASK);
                pScaler->CTRL |= format;
                pScaler->POST &= ~C_SCALER_NEW_INPUT_COLOR_MODE;
                break;
        }

	return 0;
}

/**
 * @brief   Set the input start point on X&Y axis
 * @param   scaler_num[in]: SCALER_NUM
 * @param   offset_x[in]: Input X offset
 * @param   offset_y[in]: Input Y offset
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_input_offset_set(INT8U scaler_num,INT32U offset_x, INT32U offset_y)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if ((offset_x>C_SCALER_X_START_MAX) || (offset_y>C_SCALER_Y_START_MAX)) {
		return -1;
	}

	// Set scaler start x and y position offset
	pScaler->X_START = offset_x;
	pScaler->Y_START = offset_y;

	return 0;
}

/**
 * @brief   Set the output start point
 * @param   scaler_num[in]: SCALER_NUM
 * @param   x_out_offset[in]: Output X offset
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_output_offset_set(INT8U scaler_num,INT32U x_out_offset)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	/* Make sure output width is at least multiple of 8 pixels */
	if(x_out_offset & 0x7)
	{
		x_out_offset &= ~0x7;
		ret = -1;
	}

	if(x_out_offset > C_SCALER_OUT_X_OFFSET_MAX)
	{
		x_out_offset = C_SCALER_OUT_X_OFFSET_MAX;
		ret = -1;
	}

	/* Set scaler factor */
	pScaler->OUT_X_OFFSET = x_out_offset;
	return ret;

}

/**
 * @brief   Set scaler output resolution and scale factor at the X&Y axis
 * @param   scaler_num[in]: SCALER_NUM
 * @param   factor_x[in]: X scale factor
 * @param   factor_y[in]: Y scale factor
 * @param   output_x[in]: Output buffer width
 * @param   output_y[in]: Output buffer height
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_output_pixels_set(INT8U scaler_num,INT32U factor_x, INT32U factor_y, INT32U output_x, INT32U output_y)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	//if (output_x & 0x7) {		// Make sure output width is at least multiple of 8 pixels
        if (output_x & 0x3) {		// Make sure output width is at least multiple of 4 pixels
		output_x &= ~0x7;
		ret = -1;
	}
	if (!factor_x || !factor_y) {
		factor_x = 1;
		factor_y = 1;
		ret = -1;
	}
	if (!output_x || !output_y) {
		output_x = 32;
		output_y = 8;
		ret = -1;
	}
	if (factor_x > C_SCALER_X_FACTOR_MAX) {
		factor_x = C_SCALER_X_FACTOR_MAX;
		ret = -1;
	}
	if (factor_y > C_SCALER_Y_FACTOR_MAX) {
		factor_y = C_SCALER_Y_FACTOR_MAX;
		ret = -1;
	}
	if (output_x > C_SCALER_OUT_WIDTH_MAX) {
		output_x = C_SCALER_OUT_WIDTH_MAX;
		ret = -1;
	}
	if (output_y > C_SCALER_OUT_HEIGHT_MAX) {
		output_y = C_SCALER_OUT_HEIGHT_MAX;
		ret = -1;
	}

	// Set scaler factor
	pScaler->X_FACTOR = factor_x;
	pScaler->Y_FACTOR = factor_y;

	pScaler->OUT_WIDTH = output_x - 1;
	pScaler->OUT_HEIGHT = output_y - 1;

	return ret;
}

/**
 * @brief   Set output image address.
 * @param   scaler_num[in]: SCALER_NUM
 * @param   y_addr[in]: Start address of output Y data
 * @param   u_addr[in]: Start address of output U data
 * @param   v_addr[in]: Start address of output V data
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_output_addr_set(INT8U scaler_num, INT32U y_addr, INT32U u_addr, INT32U v_addr)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	ret = 0;
	// Make sure all addresses are 4-byte alignment
	if ((y_addr & 0x3) || (u_addr & 0x3) || (v_addr & 0x3)) {
		y_addr &= ~0x3;
		u_addr &= ~0x3;
		v_addr &= ~0x3;
		ret = -1;
	}

	pScaler->OUT_Y_ADDR = y_addr;
	pScaler->OUT_U_ADDR = u_addr;
	pScaler->OUT_V_ADDR = v_addr;

	return ret;
}

INT32S drv_l1_scaler_output_addr_get(INT8U scaler_num, INT32U *y_addr, INT32U *u_addr, INT32U *v_addr)
{
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (y_addr)	*y_addr = pScaler->OUT_Y_ADDR;
	if (u_addr)	*u_addr = pScaler->OUT_U_ADDR;
	if (v_addr)	*v_addr = pScaler->OUT_V_ADDR;

	return 0;
}
/**
 * @brief   Set output image format
 * @param   scaler_num[in]: SCALER_NUM
 * @param   format[in]: Output format
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_output_format_set(INT8U scaler_num, INT32U format)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if ((format != C_SCALER_CTRL_OUT_RGB1555) &&
		(format != C_SCALER_CTRL_OUT_RGBG) &&
		(format != C_SCALER_CTRL_OUT_GRGB) &&

		(format != C_SCALER_CTRL_OUT_RGB565) &&
		(format != C_SCALER_CTRL_OUT_YUYV) &&
		(format != C_SCALER_CTRL_OUT_UYVY) &&
		(format != C_SCALER_CTRL_OUT_VYUY) &&

		(format != C_SCALER_CTRL_OUT_YUYV8X32) &&
		(format != C_SCALER_CTRL_OUT_YUYV8X64) &&
		(format != C_SCALER_CTRL_OUT_YUYV16X32) &&
		(format != C_SCALER_CTRL_OUT_YUYV16X64) &&
		(format != C_SCALER_CTRL_OUT_YUYV32X32) &&
		(format != C_SCALER_CTRL_OUT_YUYV64X64) &&
		(format != C_SCALER_CTRL_OUT_VYUY8X32) &&
		(format != C_SCALER_CTRL_OUT_VYUY8X64) &&
		(format != C_SCALER_CTRL_OUT_VYUY16X32) &&
		(format != C_SCALER_CTRL_OUT_VYUY16X64) &&
		(format != C_SCALER_CTRL_OUT_VYUY32X32) &&
		(format != C_SCALER_CTRL_OUT_VYUY64X64) &&

		(format != C_SCALER_CTRL_OUT_YUV422) &&
		(format != C_SCALER_CTRL_OUT_YUV420) &&
		(format != C_SCALER_CTRL_OUT_YUV411) &&
		(format != C_SCALER_CTRL_OUT_YUV444) &&
		(format != C_SCALER_CTRL_OUT_Y_ONLY) &&

		(format != C_SCALER_CTRL_OUT_ARGB4444) &&
        (format != C_SCALER_CTRL_OUT_ARGB8888) &&

        (format != C_SCALER_CTRL_OUT_ARGB4444X32) &&
        (format != C_SCALER_CTRL_OUT_ARGB4444X64) &&
		(format != C_SCALER_CTRL_OUT_Y_ONLY_INTEGRAL)) {
		return -1;
	}

        switch(format)
        {
            case C_SCALER_CTRL_OUT_ARGB4444X32:
                      pScaler->CTRL &= ~(C_SCALER_CTRL_OUT_MASK);
                      pScaler->CTRL |= C_SCALER_CTRL_OUT_YUYV32X32;
                      pScaler->POST |= C_SCALER_NEW_OUTPUT_COLOR_MODE;
                      break;

            case C_SCALER_CTRL_OUT_ARGB4444X64:
                      pScaler->CTRL &= ~(C_SCALER_CTRL_OUT_MASK);
                      pScaler->CTRL |= C_SCALER_CTRL_OUT_YUYV64X64;
                      pScaler->POST |= C_SCALER_NEW_OUTPUT_COLOR_MODE;
                      break;

            case C_SCALER_CTRL_OUT_Y_ONLY_INTEGRAL:
                      pScaler->CTRL &= ~(C_SCALER_CTRL_OUT_MASK);
                      pScaler->CTRL |= C_SCALER_CTRL_OUT_Y_ONLY;
                      pScaler->POST |= C_SCALER_NEW_OUTPUT_COLOR_MODE;
                      break;

            default:
                pScaler->CTRL &= ~(C_SCALER_CTRL_OUT_MASK);
                pScaler->CTRL |= format;
                pScaler->POST &= ~C_SCALER_NEW_OUTPUT_COLOR_MODE;
                break;
        }

	return 0;
}

/**
 * @brief   Set Scaler FIFO for input fifo mode
 * @param   scaler_num[in]: SCALER_NUM
 * @param   mode[in]: input fifo mode
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_fifo_line_set(INT8U scaler_num, INT32U mode)
{
	return drv_l1_scaler_input_fifo_line_set(scaler_num,mode);
}

/**
 * @brief   Set Scaler FIFO for input fifo mode
 * @param   scaler_num[in]: SCALER_NUM
 * @param   mode[in]: input fifo mode
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_input_fifo_line_set(INT8U scaler_num, INT32U mode)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

		if ((mode != C_SCALER_CTRL_IN_FIFO_DISABLE) &&
		(mode != C_SCALER_CTRL_IN_FIFO_16LINE) &&
		(mode != C_SCALER_CTRL_IN_FIFO_32LINE) &&
		(mode != C_SCALER_CTRL_IN_FIFO_64LINE) &&
		(mode != C_SCALER_CTRL_IN_FIFO_128LINE) &&
		(mode != C_SCALER_CTRL_IN_FIFO_256LINE)) {
		return -1;
	}

	pScaler->CTRL &= ~(C_SCALER_CTRL_IN_FIFO_LINE_MASK);
	pScaler->CTRL |= mode;

	return 0;
}

/**
 * @brief   Set Scaler FIFO for output fifo mode
 * @param   scaler_num[in]: SCALER_NUM
 * @param   mode[in]: ouput fifo mode
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_output_fifo_line_set(INT8U scaler_num,INT32U mode)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if ((mode != C_SCALER_CTRL_OUT_FIFO_DISABLE) &&
		(mode != C_SCALER_CTRL_OUT_FIFO_16LINE) &&
		(mode != C_SCALER_CTRL_OUT_FIFO_32LINE) &&
		(mode != C_SCALER_CTRL_OUT_FIFO_64LINE)) {
		return -1;
	}

	pScaler->CTRL &= ~(C_SCALER_CTRL_OUT_FIFO_LINE_MASK);
	pScaler->CTRL |= mode;

	return 0;
}

/**
 * @brief   Set YUV type
 * @param   scaler_num[in]: SCALER_NUM
 * @param   type[in]: YUV or YCBCR
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_YUV_type_set(INT8U scaler_num, INT32U type)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (type == C_SCALER_CTRL_TYPE_YUV) {
		pScaler->CTRL |= C_SCALER_CTRL_TYPE_YUV;
	} else {
		pScaler->CTRL &= ~(C_SCALER_CTRL_TYPE_YUV);
	}

	return 0;
}

/**
 * @brief   Set scaler out of boundary mode
 * @param   scaler_num[in]: SCALER_NUM
 * @param   mode[in]: 0: Use the boundary data of the input picture,
 *					  1: Use Use color defined in scaler_out_of_boundary_color_set()
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_out_of_boundary_mode_set(INT8U scaler_num,INT32U mode)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (mode) {
		pScaler->CTRL |= C_SCALER_CTRL_OUT_OF_BOUNDRY;
	} else {
		pScaler->CTRL &= ~C_SCALER_CTRL_OUT_OF_BOUNDRY;
	}

	return 0;
}

/**
 * @brief   Set scaler out of boundary color value
 * @param   scaler_num[in]: SCALER_NUM
 * @param   ob_color[in]: The format of ob_color is Y-Cb-Cr
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_out_of_boundary_color_set(INT8U scaler_num, INT32U ob_color)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (ob_color > C_SCALER_OUT_BOUNDRY_COLOR_MAX) {
		return -1;
	}

	pScaler->OUT_BOND_COLOR = ob_color;

	return 0;
}

/**
 * @brief   Set scaler line buffer mode
 * @param   scaler_num[in]: SCALER_NUM
 * @param   mode[in]: internal or external line buffer
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_line_buffer_mode_set(INT8U scaler_num, INT32U mode)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (mode!=C_SCALER_INTERNAL_LINE_BUFFER && mode!=C_SCALER_HYBRID_LINE_BUFFER && mode!=C_SCALER_EXTERNAL_LINE_BUFFER) {
		return -1;
	}
	else {
		pScaler->POST &= ~C_SCALER_LINE_BUFFER_MODE_MASK;
		pScaler->POST |= mode;
	}

	return 0;
}

/**
 * @brief   Set scaler integration mode
 * @param   scaler_num[in]: SCALER_NUM
 * @param   mode[in]: Integration mode:
 * 00:
 * Use bi-linear in Horizontal direction
 * Use bi-linear in Vertical direction
 * 01:
 * Use integration in Horizontal direction
 * Use bi-linear in Vertical direction
 * 10: Reserved.
 * 11:
 * Use integration in Horizontal direction
 * Use integration in Vertical direction
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_integration_mode_set(INT8U scaler_num, INT32U mode)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (mode!=C_SCALER_NEW_INTEGRATION_MODE0 && mode!=C_SCALER_NEW_INTEGRATION_MODE1 && mode!=C_SCALER_NEW_INTEGRATION_MODE3) {
		return -1;
	}
	else {
             pScaler->POST &= ~C_SCALER_NEW_INTEGRATION_MODE_MASK;
             pScaler->POST |= mode;
	}

	return 0;
}

/**
 * @brief   Set start addr when C_SCALER_EXTERNAL_LINE_BUFFER mode is selected
 * @param   scaler_num[in]: SCALER_NUM
 * @param   addr[in]: External line buffer start addr
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_external_line_buffer_set(INT8U scaler_num, INT32U addr)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (addr & 0x3) {
		return -1;
	}

	pScaler->LINE_BUFFER_ADDR = addr;

	return 0;
}

/**
 * @brief   Enable scaler interrupt and clear flag
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_bypass_zoom_mode_enable(INT8U scaler_num)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	pScalerDev->bypass_zoom_mode = 1;

	// Clear pending interrupt flags
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;

	pScaler->CTRL |= C_SCALER_CTRL_INT_ENABLE;

	return 0;
}

/**
 * @brief   Start to scale the image data
 * @param   scaler_num[in]: SCALER_NUM
 * @param   os_q_en[in]: use queue
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_start(INT8U scaler_num, INT32U os_q_en)
{
	INT32U input_x,output_x;
	INT32U format;
	INT32S ret;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

  	if (!pScalerDev->init_done || pScalerDev->started_flag) {
  		return -1;
  	}

	ret = 0;
	if ((pScaler->IN_VISIBLE_WIDTH>pScaler->IN_WIDTH) || (pScaler->IN_VISIBLE_HEIGHT>pScaler->IN_HEIGHT)) {
		pScaler->IN_VISIBLE_WIDTH = pScaler->IN_WIDTH;
		pScaler->IN_VISIBLE_HEIGHT = pScaler->IN_HEIGHT;
		ret = -1;
	}

	output_x = pScaler->OUT_WIDTH + 1;
	format = pScaler->CTRL & C_SCALER_CTRL_OUT_MASK;
	if ((format==C_SCALER_CTRL_OUT_YUYV8X32) || (format==C_SCALER_CTRL_OUT_YUYV8X64)) {
		// Must be less than 2047 pixels and be multiple of 8 pixels
		if (output_x > 0x7FF) {
			pScaler->OUT_WIDTH = 2039;
			ret = -1;
		}
	} else if ((format==C_SCALER_CTRL_OUT_YUYV16X32) || (format==C_SCALER_CTRL_OUT_YUYV16X64)) {
		// Must be less than 4095 pixels and be multiple of 8 pixels
		if (output_x > 0xFFF) {
			pScaler->OUT_WIDTH = 4087;
			ret = -1;
		}
	} else if ((format==C_SCALER_CTRL_OUT_YUV422) || (format==C_SCALER_CTRL_OUT_YUV420) || (format==C_SCALER_CTRL_OUT_YUV444)) {
		// Must be multiple of 16 pixels
		if (output_x & 0xF) {
			pScaler->OUT_WIDTH = (output_x & ~0xF) - 1;
			ret = -1;
		}
	} else if (format == C_SCALER_CTRL_OUT_YUV411) {
		// Must be multiple of 32 pixels
		if (output_x & 0x1F) {
			pScaler->OUT_WIDTH = (output_x & ~0x1F) - 1;
			ret = -1;
		}
	}

	if ((pScaler->CTRL & C_SCALER_CTRL_IN_FIFO_LINE_MASK) != C_SCALER_CTRL_IN_FIFO_DISABLE) {
		if ((pScaler->CTRL & C_SCALER_CTRL_OUT_FIFO_LINE_MASK) != C_SCALER_CTRL_OUT_FIFO_DISABLE) {
			/* Input FIFO and output FIFO mode can not be enabled at the same time */
			ret = -1;
		}
	}

	if ((pScaler->CTRL & C_SCALER_CTRL_OUT_FIFO_LINE_MASK) != C_SCALER_CTRL_OUT_FIFO_DISABLE) {
		if ((format==C_SCALER_CTRL_OUT_YUYV8X32) || (format==C_SCALER_CTRL_OUT_YUYV8X64) ||
	 		(format==C_SCALER_CTRL_OUT_YUYV16X32) || (format==C_SCALER_CTRL_OUT_YUYV16X64) ||
	 		(format==C_SCALER_CTRL_OUT_YUYV32X32) || (format==C_SCALER_CTRL_OUT_YUYV64X64))
	 	{
 			/* Output FIFO can not support in YUYVXx32 and YUYVXx64 outpuit format*/
 			return -1;
 		}

		pScaler->CTRL |= C_SCALER_CTRL_OUT_FIFO_INT;
	}

	if(((pScaler->CTRL & C_SCALER_CTRL_IN_FIFO_LINE_MASK) != C_SCALER_CTRL_IN_FIFO_DISABLE)  ||
		((pScaler->CTRL & C_SCALER_CTRL_OUT_FIFO_LINE_MASK) != C_SCALER_CTRL_OUT_FIFO_DISABLE))
	{
		pScaler->Y_START = 0;	// Start Y register must be set to 0 when FIFO mode is used
	}

    if(pScaler->POST & C_SCALER_NEW_INTEGRATION_MODE_MASK)
    {
        if((pScaler->IN_WIDTH == pScaler->OUT_WIDTH) && (pScaler->IN_WIDTH == pScaler->OUT_WIDTH))
        {
            pScaler->POST &= ~C_SCALER_NEW_INTEGRATION_MODE_MASK;
        }
        else if((pScaler->IN_VISIBLE_WIDTH == 0) || (pScaler->IN_VISIBLE_HEIGHT == 0))
        {
            pScaler->POST &= ~C_SCALER_NEW_INTEGRATION_MODE_MASK;
        }
        else
        {
            pScaler->X_START = 0x8000;
            pScaler->Y_START = 0x8000;
            format = pScaler->POST;
            format = (format >> 16);
            if(format == 1)
            {
                #if 1
                    input_x = pScaler->IN_VISIBLE_WIDTH;
                    pScaler->IN_VISIBLE_WIDTH = pScaler->IN_WIDTH;
                #else
                    input_x = pScaler->IN_WIDTH;
                #endif

                if(scaler_new_int_callback)
                    output_x = scaler_new_int_callback();
                else
                    output_x = pScaler->OUT_WIDTH;

                // x facter
                pScaler->X_FACTOR = ((output_x + 1) << 16) / (input_x + 1);
            }
            else
            {
                // x facter
                output_x = pScaler->IN_VISIBLE_WIDTH;
                input_x = pScaler->IN_WIDTH;
                if(output_x != input_x)
                {
                    input_x = output_x;
                }
                output_x = pScaler->OUT_WIDTH;

                pScaler->X_FACTOR = ((output_x + 1) << 16) / (input_x + 1);

                // y facter
                output_x = pScaler->IN_VISIBLE_HEIGHT;
                input_x = pScaler->IN_HEIGHT;
                if(output_x != input_x)
                {
                    input_x = output_x;
                }
                output_x = pScaler->OUT_HEIGHT;

                pScaler->Y_FACTOR = ((output_x + 1) << 16) / (input_x + 1);
            }
        }
    }

	if((pScaler->POST & C_SCALER_NEW_OUTPUT_COLOR_MODE) && (pScaler->CTRL & C_SCALER_CTRL_OUT_Y_ONLY))
    {
        pScaler->X_FACTOR = 0x10000;
        pScaler->Y_FACTOR = 0x10000;
        pScaler->X_START = 0;
        pScaler->Y_START = 0;
        pScaler->POST &= ~C_SCALER_NEW_INTEGRATION_MODE_MASK;
    }

  	pScalerDev->started_flag = 1;
#if 1
  	if((ret >= 0) && os_q_en) {
  		pScalerDev->message_queue = pScalerDev->scaler_q;
  	}
#endif

  	pScaler->CTRL |= (C_SCALER_CTRL_START|C_SCALER_CTRL_INT_ENABLE|C_SCALER_CTRL_RGB1555_TRANSPARENT);
  	return ret;
}

/**
 * @brief   Restart to scaler the image data
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_restart(INT8U scaler_num)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

  	if (!pScalerDev->init_done || !pScalerDev->started_flag) {
  		return -1;
  	}

	if (pScaler->INT_FLAG & C_SCALER_INT_DONE) {		// Already done
		return 1;
	}

	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;	// Clear pending bit
	pScaler->CTRL |= C_SCALER_CTRL_START;

  	return 0;
}

/**
 * @brief   pause to scaler the image data
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	none
 */
void drv_l1_scaler_pause(INT8U scaler_num)
{
#if 0
	INT32S mask;
#endif
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	pScalerDev->register_backup1 = pScaler->STATUS_READ1;		// Backup the value of interal status register
	pScalerDev->register_backup2 = pScaler->STATUS_READ2;		// Backup the value of Y-start register

	pScaler->CTRL = C_SCALER_CTRL_RESET;	// Disable interrupt and reset scaler

	// Clear pending interrupt flags
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;

	pScalerDev->started_flag = 0;
	pScalerDev->bypass_zoom_mode = 0;

#if 0
  	mask = drv_l1_scaler_device_protect();
	pScalerDev->interrupt_flag = 0;
	drv_l1_scaler_device_unprotect(mask);
#endif
}

/**
 * @brief   resume to scaler the image data
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	none
 */
void drv_l1_scaler_resume(INT8U scaler_num)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	pScaler->STATUS_WIRTE1 = pScalerDev->register_backup1;
	pScaler->Y_START = pScalerDev->register_backup2;

	pScaler->CTRL = C_SCALER_CTRL_RESET | C_SCALER_CTRL_CONTINUOUS_MODE;	// Reset scaler
	pScaler->CTRL = 0;						// Disable scaler
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;	// Clear interrupt pending flag
	pScaler->POST = 0;						// Disable color matrix and gamma function

	drv_l1_scaler_input_visible_pixels_set(scaler_num,0, 0);
	drv_l1_scaler_YUV_type_set(scaler_num,C_SCALER_CTRL_TYPE_YCBCR);
	drv_l1_scaler_out_of_boundary_mode_set(scaler_num,1);
	drv_l1_scaler_out_of_boundary_color_set(scaler_num,0x008080);
	pScaler->X_START = 0;

	pScalerDev->started_flag = 0;
	pScalerDev->bypass_zoom_mode = 0;


	if (pScalerDev->init_done) {
	#if 1
		if(pScalerDev->message_queue) {
   			OSQFlush(pScalerDev->message_queue);
   		}
   	#endif
    } else {
        NVIC_EnableIRQ(SCALAR0_IRQn);
		pScalerDev->init_done = 1;
	}

  	pScalerDev->interrupt_flag = 0;
}

/**
 * @brief   stop to scaler the image data
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	none
 */
void drv_l1_scaler_stop(INT8U scaler_num)
{
#if 0
	INT32S mask;
#endif
  	SCALER_SFR *pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	pScaler->CTRL |= C_SCALER_CTRL_RESET;	// Disable interrupt and reset scaler

	// Clear pending interrupt flags
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;

	pScalerDev->started_flag = 0;
	pScalerDev->bypass_zoom_mode = 0;
	pScalerDev->interrupt_flag = 0;

#if 1
	pScalerDev->message_queue = 0;
#endif

#if 0
  	mask = drv_l1_scaler_device_protect();
	pScalerDev->bypass_zoom_mode = 0;
	drv_l1_scaler_device_unprotect(mask);
#endif
}

/**
 * @brief   Restart to scaler the image data in wrap mode use
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_wrap_mode_restart(INT8U scaler_num)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	pScaler->CTRL |= C_SCALER_CTRL_RESET;
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;

	pScalerDev->interrupt_flag = 0;
	pScaler->CTRL |= C_SCALER_CTRL_START;
	return 0;
}

/**
 * @brief   stop scale the image data in wrap mode use
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_wrap_mode_stop(INT8U scaler_num)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	pScaler->CTRL |= C_SCALER_CTRL_RESET;
	pScaler->INT_FLAG = C_SCALER_INT_PEND | C_SCALER_INT_OUT_FULL;
	pScalerDev->interrupt_flag = 0;
	return 0;
}

/**
 * @brief   Disable scaler interrupt
 * @param   none
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_device_protect(void)
{
    NVIC_DisableIRQ(SCALAR0_IRQn);
	return 1;
}

/**
 * @brief   Clear scaler interrupt mask
 * @param   mask[in]: 0: Unmask scaler IRQ 1:Mask scaler IRQ
 * @return 	none
 */
void drv_l1_scaler_device_unprotect(INT32S mask)
{
	if (mask) {
        NVIC_EnableIRQ(SCALAR0_IRQn);
	}
    else {
        NVIC_DisableIRQ(SCALAR0_IRQn);
    }
}

/**
 * @brief   Get the current status of scaler engine
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	Current status of scaler engine
 */
INT32S drv_l1_scaler_wait_idle(INT8U scaler_num)
{
	INT32S message;
	INT32S mask;
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

  	if (!pScalerDev->init_done) {
  		return C_SCALER_STATUS_INIT_ERR;
  	}

	if (!pScalerDev->started_flag) {
		return C_SCALER_STATUS_STOP;
	}

#if 1
	if(pScalerDev->message_queue)
	{
		osEvent result;

        result = osMessageGet(pScalerDev->message_queue, 5000);
        message = result.value.v;
        if(result.status == osEventMessage) {
            return message;
        }
        else if(result.status == osEventTimeout){
            return C_SCALER_STATUS_TIMEOUT;
        }
        else {
            return C_SCALER_STATUS_INIT_ERR;
        }
	}
	else
#endif
	{
  		// TBD: we need a timeout mechanism here
		while (!pScalerDev->interrupt_flag) ;

		mask = drv_l1_scaler_device_protect();
		message = pScalerDev->interrupt_flag;
		pScalerDev->interrupt_flag = 0;
		drv_l1_scaler_device_unprotect(mask);

		return message;
	}
}

/**
 * @brief   Get scaler status
 * @param   scaler_num[in]: SCALER_NUM
 * @return 	scale status
 */
INT32S drv_l1_scaler_status_polling(INT8U scaler_num)
{
	INT32S status;
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	if (!pScalerDev->started_flag) {
		return C_SCALER_STATUS_STOP;
	}

	status = 0;

	if (pScaler->CTRL & C_SCALER_CTRL_BUSY) {
		status |= C_SCALER_STATUS_BUSY;
	}
	if (pScaler->INT_FLAG & C_SCALER_INT_DONE) {
		status |= C_SCALER_STATUS_DONE;
	} else if (pScaler->INT_FLAG & C_SCALER_INT_PEND) {
		status |= C_SCALER_STATUS_INPUT_EMPTY;
	} else if (pScaler->INT_FLAG & C_SCALER_INT_OUT_FULL) {
		status |= C_SCALER_STATUS_OUTPUT_FULL;
	}

	return status;
}

INT32S drv_l1_scaler_gamma_switch(INT8U scaler_num, INT8U gamma_switch)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (gamma_switch == 0) {
    	pScaler->POST &= ~C_SCALER_Y_GAMMA_EN;
    } else {
    	pScaler->POST |= C_SCALER_Y_GAMMA_EN;
    }
    return STATUS_OK;
}

void drv_l1_scaler_Y_gamma_config(INT8U scaler_num, INT8U gamma_table_id, INT8U gain_value)
{
	if(scaler_num == SCALER_0) {
		SCALER0_Y_GAMMA(gamma_table_id*4) = gain_value;
	}
}

INT32S drv_l1_scaler_color_matrix_switch(INT8U scaler_num,INT8U color_matrix_switch)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	if (color_matrix_switch == 0) {
    	pScaler->POST &= ~C_SCALER_COLOR_MATRIX_EN;
    }
    else {
    	pScaler->POST |= C_SCALER_COLOR_MATRIX_EN;
    }

    return STATUS_OK;
}

INT32S drv_l1_scaler_color_matrix_config(INT8U scaler_num, COLOR_MATRIX_CFG *color_matrix)
{
	SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	pScaler->A11 = color_matrix->A11;
    pScaler->A12 = color_matrix->A12;
    pScaler->A13 = color_matrix->A13;
    pScaler->A21 = color_matrix->A21;
    pScaler->A22 = color_matrix->A22;
    pScaler->A23 = color_matrix->A23;
    pScaler->A31 = color_matrix->A31;
    pScaler->A32 = color_matrix->A32;
    pScaler->A33 = color_matrix->A33;
    return STATUS_OK;
}

/**
 * @brief   initial scale hardware
 * @param   scaler_num[in]: scale device
 * @return 	0: Success, -1: Fail
 */
void drv_l1_scaler_hw_init(INT8U scaler_num)
{
    INT32S scaler_status;
    INT32U character_8x8_yuyv_array[SCALER_MAX][16];

    drv_l1_scaler_init(scaler_num);

    drv_l1_scaler_input_pixels_set(scaler_num,8, 8);
    drv_l1_scaler_input_visible_pixels_set(scaler_num,8, 8);
    drv_l1_scaler_input_A_addr_set(scaler_num,(INT32U) &character_8x8_yuyv_array[scaler_num][0], NULL, NULL);
    drv_l1_scaler_input_B_addr_set(scaler_num,(INT32U) &character_8x8_yuyv_array[scaler_num][0], NULL, NULL);
    drv_l1_scaler_input_offset_set(scaler_num,0, 0);
    drv_l1_scaler_input_format_set(scaler_num,C_SCALER_CTRL_OUT_YUYV);
    drv_l1_scaler_output_pixels_set(scaler_num,1<<14, 1<<14, 8, 2);
    drv_l1_scaler_output_addr_set(scaler_num,(INT32U) &character_8x8_yuyv_array[0], NULL, NULL);
    drv_l1_scaler_output_format_set(scaler_num,C_SCALER_CTRL_OUT_YUYV);
    drv_l1_scaler_fifo_line_set(scaler_num,C_SCALER_CTRL_FIFO_DISABLE);

	while (1) {
		scaler_status = drv_l1_scaler_wait_idle(scaler_num);
		if (scaler_status == C_SCALER_STATUS_STOP) {
			drv_l1_scaler_start(scaler_num, DISABLE);
		} else {
			break;
		}
	}
    drv_l1_scaler_stop(scaler_num);
}

INT32S drv_l1_scaler_mas_set(INT8U scaler_num,SCALER_MAS* pMas_en)
{
	//INT32U regValue;
	//SCALER_SFR* pScaler = _get_scaler_sfr_base(scaler_num);

	//regValue = ((pMas_en->mas_3<<6)|(pMas_en->mas_2<<4)|(pMas_en->mas_1<<2)|pMas_en->mas_0);
	//pScaler->MAS_EN = regValue;

    return STATUS_OK;
}

INT32S drv_l1_scaler_idle_check(INT8U scaler_num)
{
	INT32S message;
	INT32S mask;
	ScalerDev_t *pScalerDev = _get_scaler_device(scaler_num);

	if(pScalerDev->interrupt_flag == 0) {
		return C_SCALER_STATUS_BUSY;
	}

	mask = drv_l1_scaler_device_protect();
	message = pScalerDev->interrupt_flag;
	pScalerDev->interrupt_flag = 0;
	drv_l1_scaler_device_unprotect(mask);
	return message;
}

/**
 * @brief   Register scaler call back function
 * @param   callback[in]: callback function
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_scaler_isr_callback_set(void (*callback)(INT32U scaler0_event, INT32U scaler1_event))
{
	scaler_isr_callback = callback;
	return 0;
}

INT32S drv_l1_scaler_new_int_callback_set(INT32U (*callback)(void))
{
	scaler_new_int_callback = callback;
	return 0;
}
#endif //(defined _DRV_L1_SCLAER) && (_DRV_L1_SCLAER == 1)

