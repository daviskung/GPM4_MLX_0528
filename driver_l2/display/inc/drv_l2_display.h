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
#ifndef __drv_l2_DISPLAY_H__
#define __drv_l2_DISPLAY_H__
#include "drv_l1.h"

#define _DRV_L2_DISP             _DRV_L1_TFT

// Selection DISDEV_TFT / DISDEV_HDMI_720P / DISDEV_HDMI_480P for display device
#define DISPLAY_DEVICE          DISDEV_TFT

// TFT Device, only one tft can be enable
#define TPO_TD025THD1		     0
#define GPM_LM765A0			     0
#define AUO_A043Fl01             0
#define TXD_T144C                0
#define ILI9806E_IDT		     0
#define AUO_A027DTN019           1      //320x240 stripe RGB mode only
#define ILI9325                  0

// data type
typedef enum
{
	DISDEV_TV_QVGA = 0,
	DISDEV_TV_VGA,
	DISDEV_TV_D1,
	DISDEV_TFT,
	DISDEV_HDMI_480P,
	DISDEV_HDMI_720P,
	DISDEV_MAX
} DISP_DEV;


typedef enum
{
	DIS_TYPE_TV = 1,
	DIS_TYPE_TFT,
	DIS_TYPE_HDMI,
	DIS_TYPE__MAX
} DISP_TYPE;

typedef enum
{
	DISP_FMT_RGB565 = 0,
	DISP_FMT_BGRG,
	DISP_FMT_GBGR,
	DISP_FMT_RGBG,
	DISP_FMT_GRGB,
	DISP_FMT_VYUV,
	DISP_FMT_YVYU,
	DISP_FMT_UYVY,
	DISP_FMT_YUYV,
	DISP_FMT_GP420,
	DISP_FMT_MAX
} DISP_FMT;

typedef enum
{
	UI_FMT_RGB1555 = 0,
	UI_FMT_PALETTE_4,
	UI_FMT_PALETTE_16,
	UI_FMT_PALETTE_256,
	UI_FMT_ARGB444,
	UI_FMT_MAX
} UI_FMT;

typedef struct DispCtrl_s
{
	INT16U width;
	INT16U height;
	INT32S (*init)(void);
} DispCtrl_t;

extern INT32S drv_l2_display_init(void);
extern void drv_l2_display_uninit(void);
extern INT32S drv_l2_display_start(DISP_DEV disp_device, DISP_FMT color_mode);
extern INT32S drv_l2_display_stop(DISP_DEV disp_dev);
extern void drv_l2_display_get_size(DISP_DEV disp_dev, INT16U *width, INT16U *height);
extern INT32U drv_l2_display_get_fmt(DISP_DEV disp_dev);
extern INT32S drv_l2_display_update(DISP_DEV disp_dev, INT32U buffer);
extern INT32S drv_l2_display_buffer_set(DISP_DEV disp_dev, INT32U buffer);
extern INT32S drv_l2_display_buffer_update_state_get(DISP_DEV disp_dev, INT32U wait);
extern INT32S drv_l2_display_UI_init(DISP_DEV disp_dev, DISP_FMT color_mode, INT8U blend_level, INT32U ui_addr, INT32U pal_addr);
extern INT32S drv_l2_display_UI_uninit(DISP_DEV disp_dev);
extern INT32S drv_l2_display_UI_update(DISP_DEV disp_dev, INT8U blend_level, INT32U ui_addr, INT32U pal_addr);
extern INT32S drv_l2_display_UI_stop(void);
extern INT32S drv_l2_ppu_isr_callback_set(void (*callback)(INT32U ppu_event));
#endif  //__drv_l2_DISPLAY_H__
