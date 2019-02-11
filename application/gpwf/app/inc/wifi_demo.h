/***************************************************************************
* wifi_demo.h
*
* Purpose: For WiFi demo
*
* Author: Eugene Hsu
*
* Date: 2015/10/05
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 
* History :
*
******************************************************************************/
#ifndef __WIFI_DEMO_H__
#define __WIFI_DEMO_H__

typedef void(*UPDATE_CUR_JPEG)(INT8U* buf, INT32U len);
typedef void(*AUDIO_STREAM_CBK)(short* buf, int len);
typedef INT32U(*AUDIO_STREAM_GET_FRAME_SIZE)(void);

typedef enum WIFI_DEMO_E
{
	WIFI_GP_SOCKET_DEMO = 0,
	WIFI_MJPEG_STREAMER_DEMO,
	WIFI_SOCKET_MJPEG_DEMO,
	WIFI_DOOR_BELL_DEMO,
	WIFI_UDP_SERVER_DEMO,
	WIFI_GOPLUS_DEMO,
	WIFI_AUDIO_STREAM_DEMO,
	WIFI_TUTK_DEMO,
	WIFI_WS_SSL_DEMO,
	WIFI_SSL_DEMO,
	WIFI_NONE_DEMO
} WIFI_DEMO_T;

typedef struct WIFI_MSG_S
{
	INT32U event;
	INT32U len;
	char* buf;
} WIFI_MSG_T;

#define WIFI_WAIT_TIMEOUT	30		/* 30 seconds */
extern void register_update_cur_jpeg_cbk(INT32U cbk);
extern void wifi_start_network(INT32U flag, INT32U mode);
extern void wifi_stop_network(void);
extern void wifi_start_app(INT32U flag);
extern void wifi_stop_app(INT32U flag);
extern void audio_stream_register_cbk(void* cbk, void* get_frame_size);
extern int alaw_enc(char *out_stream, short *in_pcm, int in_size);
#endif	//__WIFI_DEMO_H__