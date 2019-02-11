/******************************************************
* usbd_uac.h
*
* Purpose: USBD UVC header file
*
* Author: Jimi Yu
*
* Date: 2016/05/06
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/
#ifndef __USBD_UVC_H__
#define __USBD_UVC_H__
#include "cmsis_os.h"
#include "drv_l2_usbd.h"

#define UVC_H264		1		//0: uncompressed

typedef enum
{
	/* 0 for system DMA done */
	USBD_START_VIDEO_EVENT = 1,
	USBD_STOP_VIDEO_EVENT,
	USBD_SEND_VIDEO_EVENT,
	USBD_SEND_VIDEO_DONE_EVENT,
	USBD_START_AUDIO_EVENT,
	USBD_STOP_AUDIO_EVENT,
	USBD_SEND_AUDIO_DONE_EVENT,
	USBD_SET_INTERFACE_EVENT,
	USBD_UAC_TIMER_DONE,
	USBD_NO_EVENT
} UVC_TASK_EVENT_E;


extern void _usbd_ep5_send_done_cbk(void);
extern void _usbd_ep7_send_done_cbk(void);
extern void _usbd_get_set_interface(INT32U video, INT32U audio);
extern INT32U uvc_get_cur_frame(osMessageQId event);
extern INT32U uvc_post_cur_frame(osMessageQId event, INT32U frame_addr);
extern void UVCTask(void const *param);

#endif
