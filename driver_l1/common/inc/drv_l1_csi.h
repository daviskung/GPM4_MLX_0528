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
#ifndef __drv_l1_CSI_H__
#define __drv_l1_CSI_H__

/****************************************************
*		include file								*
****************************************************/
#include "drv_l1.h"

/****************************************************
*	Definition	 									*
****************************************************/
typedef	void (*CSI_CALLBACK_FUN)(INT32U event);

typedef enum
{
	CSI_NON_INTERLACE = 0,
	CSI_INTERLACE,
	CSI_INTERLACE_INVERT_FIELD
} CSI_INTERLACE_MODE_DEF;

typedef enum
{
	CSI_IN_RGB888_BGRG = 0,
	CSI_IN_RGB888_GBGR,
	CSI_IN_RGB565_BGRG,
	CSI_IN_RGB565_GBGR,
	CSI_IN_UYVY,
	CSI_IN_UYVY_INVERT_UV7,
	CSI_IN_YUYV,
	CSI_IN_YUYV_INVERT_UV7
} CSI_INPUT_DATA_DEF;

typedef enum
{
	CSI_OUT_RGB565 = 0,
	CSI_OUT_RGB1555,
	CSI_OUT_VYUY,
	CSI_OUT_VYUY_INVERT_UV7,
	CSI_OUT_YUYV,
	CSI_OUT_YUYV_INVERT_UV7,
	CSI_OUT_Y_ONLY
} CSI_OUTPUT_DATA_DEF;

typedef enum
{
	CSI_HREF = 0,
	CSI_HSYNC_CCIR_601,
	CSI_HSYNC_CCIR_656
} CSI_INPUT_INTERFACE_DEF;

typedef enum
{
	CSI_SENSOR_FPS_7 = 0,
	CSI_SENSOR_FPS_10,
	CSI_SENSOR_FPS_15,
	CSI_SENSOR_FPS_27,
	CSI_SENSOR_FPS_30
} CSI_SENSOR_FPS_DEF;

typedef enum
{
	CSI_NONE_EVENT,
	CSI_SENSOR_FRAME_END_EVENT,
	CSI_MOTION_DET_EVENT,
	CSI_SENSOR_POSITION_HIT_EVENT,
	CSI_MOTION_FIFO_UNDERRUN_EVENT,
	CSI_SENSOR_FIFO_OVERFLOW_EVENT,
	CSI_SENSOR_OUTPUT_FIFO_EVENT
} CSI_EVENT_DEF;

typedef enum {
	ENUM_CSI_LATCH_DELAY_1_CLOCK = 0,
	ENUM_CSI_LATCH_DELAY_2_CLOCK,
	ENUM_CSI_LATCH_DELAY_3_CLOCK,
	ENUM_CSI_LATCH_DELAY_4_CLOCK
} CSI_LATCH_DELAY_ENUM;

typedef enum {
	ENUM_CSI_FALLING_EDGE = 0,
	ENUM_CSI_RISING_EDGE
} CSI_LATCH_EDGE_ENUM;

typedef enum {
	CSI_NO_FIFO = 0,
	CSI_FIFO_8,
	CSI_FIFO_16,
	CSI_FIFO_32
} CSI_FIFO_DEF;

typedef enum {
	CSI_CUBIC_64_64 = 0,
	CSI_CUBIC_32_32
} CSI_CUBIC_DEF;

typedef enum {
	CSI_PREVIEW_MODE = 0,
	CSI_CAPTURE_MODE
} CSI_PREVIEW_MODE_DEF;

/********************* Define R_TGR_IRQ_STATUS bit mask (0xD0500238, Sensor IRQ flag register) *****************/
#define MASK_CSI_OUTPUT_FIFO_FLAG		    (1 << 5)		/* Sensor output FIFO mode IRQ flag */
#define MASK_CSI_FIFO_OVERFLOW_FLAG		    (1 << 4)		/* Sensor receive FIFO overflow IRQ flag */
#define MASK_CSI_MFIFO_UNDERRUN_FLAG	    (1 << 3)		/* Motion detect FIFO under-run IRQ flag */
#define MASK_CSI_POSITION_HIT_FLAG		    (1 << 2)		/* Sensor position hit IRQ flag */
#define MASK_CSI_MOTION_DET_FLAG		    (1 << 1)		/* Motion detect frame end IRQ flag */
#define MASK_CSI_FRAME_END_FLAG			    (1 << 0)		/* Sensor frame end IRQ flag */

/********************* Define R_TGR_IRQ_EN bit mask (0xD050023C, Sensor IRQ enable register) *****************/
#define MASK_CSI_OUTPUT_FIFO_ENABLE		    (1 << 5)		/* Sensor output FIFO mode IRQ enable */
#define MASK_CSI_FIFO_OVERFLOW_ENABLE	    (1 << 4)		/* Sensor receive FIFO overflow IRQ enable */
#define MASK_CSI_MFIFO_UNDERRUN_ENABLE	    (1 << 3)		/* Motion detect FIFO under-run IRQ enable */
#define MASK_CSI_POSITION_HIT_ENABLE	    (1 << 2)		/* Sensor position hit IRQ enable */
#define MASK_CSI_MOTION_DET_ENABLE		    (1 << 1)		/* Motion detect frame end IRQ enable */
#define MASK_CSI_FRAME_END_ENABLE		    (1 << 0)		/* Sensor frame end IRQ enable */

/********************* Define R_CSI_TG_CTRL0 bit mask (0xD0500240, Sensor timing generator control register 0) *****************/
#define MASK_CSI_CTRL0_INPUT_TO_PSCALER		(1 << 24)       /* Sensor input selection pscaler mode. */
#define MASK_CSI_CTRL0_FIFO		            ((0x3) << 20)   /* Sensor input selection fifo mode. */
#define MASK_CSI_CTRL0_JPEG		            (1 << 18)       /* Sensor input selection jpeg mode. */
#define MASK_CSI_CTRL0_VGA_TO_D1			(1 << 17)		/* Sensor VGA upscaler to D1 enable.  During this mode, the sensor input must be set to VGA resolution, and the output frame buffer will be D1 size */
#define MASK_CSI_CTRL0_OWN_IRQ				(1 << 16)		/* Sensor・s own interrupt ID enable bit.  When this bit is 1, the sensor related IRQ will go through another interrupt ID and separated from PPU・s IRQ */
#define MASK_CSI_CTRL0_INTL					(1 << 15)		/* Sensor input interlace/non-interlace selection */
#define MASK_CSI_CTRL0_FIELDINV				(1 << 14)		/* Sensor filed invert selection.  Used only in interlace mode */
#define MASK_CSI_CTRL0_YUVTYPE				(1 << 13)		/* YUV input type selection. 0: UYVY or BGRG 1: YUYV or GBGR */
#define MASK_CSI_CTRL0_VRST					(1 << 12)		/* Vertical counter reset selection */
#define MASK_CSI_CTRL0_VADD					(1 << 11)		/* Vertical counter increase selection */
#define MASK_CSI_CTRL0_HRST					(1 << 10)		/* Horizontal counter reset selection */
#define MASK_CSI_CTRL0_FGET					(1 << 9)		/* Field latch timing selection */
#define MASK_CSI_CTRL0_CCIR656				(1 << 8)		/* CCIR656 selection, 0: CCIR601, 1: CCIR656 */
#define MASK_CSI_CTRL0_BSEN				    (1 << 7)		/* 0: disable blue screen effect, 1: enable blue screen effect */
#define MASK_CSI_CTRL0_YUVOUT				(1 << 6)		/* YUV/RGB output selection */
#define MASK_CSI_CTRL0_YUVIN				(1 << 5)		/* YUV/RGB input selection */
#define MASK_CSI_CTRL0_CLKIINV				(1 << 4)		/* Input clock invert selection */
#define MASK_CSI_CTRL0_RGB565				(1 << 3)		/* RGB565 mode input selection.  This bit is useful only when YUVIN is 0 */
#define MASK_CSI_CTRL0_HREF					(1 << 2)		/* HREF mode selection.This bit is useful for OV・s sensor */
#define MASK_CSI_CTRL0_CAP  				(1 << 1)		/* Capture/preview mode selection. This bit must be set to 1 */
#define MASK_CSI_CTRL0_CSIEN 				(1 << 0)		/* Sensor controller enable bit */

/********************* Define R_CSI_TG_CTRL1 bit mask (0xD0500244, Sensor timing generator control register 1) *****************/
#define MASK_CSI_CTRL1_CLK_PRI			    (1 << 15)		/* 0: Priority is less than display, 1: Priority is over display */
#define MASK_CSI_CTRL1_CLK_STOPB			(1 << 14)		/* Clock output stop selection when system is busy, 0: stop clock output when busy, 1: not stop clock output when busy */
#define MASK_CSI_CTRL1_CUNIC_32_32			(1 << 13)		/* 0: 64*64, 1: 32*32 */
#define MASK_CSI_CTRL1_CUNICEN			    (1 << 12)		/* 0: line output mode, 1: cubic output mode */
#define MASK_CSI_CTRL1_CLK_SEL				(1 << 11)		/* Sensor master clock selection */
#define MASK_CSI_CTRL1_YONLY				(1 << 10)		/* Y output only selection */
#define MASK_CSI_CTRL1_INVYUVI				(1 << 9)		/* UV input invert selection, useful only in YUV input mode */
#define MASK_CSI_CTRL1_CUTEN				(1 << 8)		/* Screen CUT enable */
#define MASK_CSI_CTRL1_CLKOEN				(1 << 7)		/* CSICLKO output clock enable */
#define MASK_CSI_CTRL1_INVYUVO				(1 << 6)		/* Invert UV・s bit 7 at YUV output mode, useless in RGB output mode */
#define MASK_CSI_CTRL1_RGB1555				(1 << 4)		/* RGB1555 mode output selection */
#define MASK_CSI_CTRL1_CLKOINV				(1 << 3)		/* Output clock invert selection */

/********************* Define R_CSI_SEN_CTRL bit mask (0xD0500264, Sensor Misc. Control Register1) *****************/
#define MASK_CSI_SEN_LONG_BURST				(1 << 7)		/* 0: Disable sensor blending functio, 1: Enable sensor blending function */


#define MASK_CSI_CTRL1_LATCH_DELAY_MASK	    (0x07)

#define DELAY_1CLOCK         				(0)			    /* Data latch delay 1 clock */
#define DELAY_2CLOCK         				(0x01)		    /* Data latch delay 2 clock */
#define DELAY_3CLOCK         				(0x02)		    /* Data latch delay 3 clock */
#define DELAY_4CLOCK         				(0x03)		    /* Data latch delay 4 clock */

/****************************************************
*		external function declarations				*
****************************************************/
extern void drvl1_csi_input_set(CSI_INPUT_INTERFACE_DEF interface, CSI_INTERLACE_MODE_DEF interlace, INT32U invert_clk, CSI_INPUT_DATA_DEF data_format);
extern void drv_l1_register_csi_cbk(CSI_CALLBACK_FUN fun);
extern void drv_l1_csi_set_buf(INT32U buf);
extern INT32S drvl1_csi_input_pscaler_set(INT32U enable);
extern INT32U drv_l1_csi_get_buf(void);
extern void drv_l1_csi_set_irq(INT32U enable);
extern void drv_l1_csi_md_set_parameter(INT32U enable, INT32U img_h, INT32U img_v, INT32 char_mode);
extern void drv_l1_csi_irq_enable(INT32U enable, INT32U flag);
extern void drv_l1_csi_md_set_threshold(INT32U value);
extern void drv_l1_csi_md_set_mode(INT32U value);
extern void drv_l1_csi_md_set_frame(INT32U value);
extern void drv_l1_csi_md_set_buf(INT32U buf);
extern INT32U drv_l1_csi_md_get_buf(void);
extern INT32U drv_l1_csi_md_get_block_size(INT32U img_h, INT32U img_v, INT32 char_mode);
extern void drvl1_csi_input_latch_timing1_set(INT32U h_start, INT32U field0_vstart, INT32U field1_vstart, CSI_LATCH_DELAY_ENUM delay);
extern void drvl1_csi_input_latch_timing2_set(CSI_LATCH_EDGE_ENUM field, CSI_LATCH_EDGE_ENUM v_reset, CSI_LATCH_EDGE_ENUM v_increase, CSI_LATCH_EDGE_ENUM h_reset);
extern INT32S drvl1_csi_clko_invert_set(INT32U enable);
extern INT32S drvl1_csi_jpeg_mode_set(INT32U enable);
extern INT32S drvl1_csi_fifo_mode_set(INT32U mode);
extern INT32S drvl1_csi_input_resolution_set(INT32U width, INT32U height);
extern INT32S drvl1_csi_preview_mode_set(INT32U enable);
extern INT32S drvl1_csi_clock_priority_set(INT32U enable);
extern INT32S drvl1_csi_clock_stop_set(INT32U enable);
extern INT32S drvl1_csi_cubic_output_set(INT32U enable, INT32U size);
extern INT32S drvl1_csi_field_invert_set(INT32U enable);
extern INT32S drvl1_csi_blue_screen_set(INT32U enable, INT32U bs_upper, INT32U bs_lower);
extern INT32S drvl1_csi_black_screen_set(INT32U hs, INT32U he, INT32U vs, INT32U ve);
extern INT32S drvl1_csi_cut_screen_set(INT32U enable, INT32U cut_s, INT32U cut_e);
extern INT32S drvl1_csi_scale_down_set(INT32U ho, INT32U vo);
extern void drv_l1_csi_state_clear(INT32U flag);
extern INT32S drvl1_csi_enable(INT32U enable);
extern INT32S drvl1_csi_long_burst_set(INT32U enable);
extern INT32S drvl1_csi_output_data_format_set(CSI_OUTPUT_DATA_DEF data_format);
extern INT32S drvl1_csi_clockout_enable(INT32U enable);
#endif	//__drv_l1_CSI_H__
