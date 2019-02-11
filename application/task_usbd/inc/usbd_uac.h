/******************************************************
* usbd_uac.h
*
* Purpose: USBD UAC header file
*
* Author: Eugene Hsu
*
* Date: 2013/03/14
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__
#include "cmsis_os.h"
#include "drv_l2_usbd.h"

/* Definition of UAC speaker & microphone sample rates */
//#define USBD_UAC_SPEAKER_SAMPLE_RATE		48000
#define USBD_UAC_SPEAKER_SAMPLE_RATE		48000
#define USBD_UAC_MICROPHONE_SAMPLE_RATE		16000
#define USBD_UAC_SPK_SUBFRAME_BYTE			0x02
#define USBD_UAC_SPK_SUBFRAME_VALID_BIT		0x10
#define EPC_MAX_PKT_SIZE					1024

#define USBD_UAC_DEC_QUEUE_MAX_LEN		32
#define USBD_UAC_DEC_STATE_STACK_SIZE	1024
#define USBD_UAC_ENC_QUEUE_MAX_LEN		32
#define USBD_UAC_ENC_STATE_STACK_SIZE	1024
#define USBD_UAC_CTL_QUEUE_MAX_LEN		32
#define USBD_UAC_CTL_STATE_STACK_SIZE	512
#define USBD_UAC_DET_IO_STACK_SIZE		512
#define USBD_UAC_SPEAKER_PKT_SIZE				(USBD_UAC_SPEAKER_SAMPLE_RATE*USBD_UAC_SPK_SUBFRAME_BYTE*2/1000)
#define USBD_UAC_SPEAKER_RING_BUFF_LEN	10
#define USBD_UAC_SPEAKER_RING_BUFF_SIZE			(USBD_UAC_SPEAKER_PKT_SIZE*USBD_UAC_SPEAKER_RING_BUFF_LEN)
#define USBD_UAC_DEC_BUF_MAX_INTERVAL	7
#define USBD_UAC_DEC_BUF_MID_INTERVAL	5
#define USBD_UAC_DEC_BUF_MIN_INTERVAL	3

#define USBD_UAC_SAMPLE_RATE_MAX_OFFSET		150
#define USBD_UAC_SAMPLE_RATE_MID_OFFSET		100
#define USBD_UAC_SAMPLE_RATE_MIN_OFFSET		100

#define USBD_UAC_MIC_PKT_SIZE				(USBD_UAC_MICROPHONE_SAMPLE_RATE*2/1000)
//#define USBD_UAC_MIC_PKT_SIZE				(16)
#define USBD_UAC_MIC_RING_BUFF_LEN	10
#define USBD_UAC_MIC_RING_BUFF_SIZE			(USBD_UAC_MIC_PKT_SIZE*USBD_UAC_MIC_RING_BUFF_LEN)

#define UACHIDBUFLEN	1		/* HID data buffer length */
#define USBD_DAC_MIN_GAIN_OFFSET	0x8000
#define USBD_DAC_AMP_LEVEL			64	/* 1 << 6 = 64 level */
//#define USBD_DAC_AMP_LEVEL			31	/* 1 << 6 = 64 level */
#define USBD_DAC_AMP_LEVEL_INTERVAL	10	/* 65536 >> 6(64 level) */

#define MSG_UAC_SEND_MIC_DATA		0x10
#define MSG_UAC_SEND_MIC_DATA_DONE	0x11
#define MSG_UAC_SPK_START			0x12
#define MSG_UAC_REC_SPK_DATA_DONE	0x13
#define MSG_UAC_REC_SPK_PKT_DONE	0x14

#define MSG_AUDIO_SPK_START			0x20
#define MSG_AUDIO_SPK_STOP			0x21
#define MSG_AUDIO_MIC_START			0x22
#define MSG_AUDIO_MIC_STOP			0x23

#define UAC_IDLE_STATE				0
#define UAC_SENDING_MIC_STATE		1
#define UAC_WAIT_FOR_SEND_MIC_STATE	2
#define UAC_RECING_SPK_STATE		3

#define UAC_GAIN_TO_DAC_VOL(a)\
		(a-USBD_DAC_MIN_GAIN_OFFSET)>>3;\

#define MAX_BUFFER_NUM 5

typedef struct uac_aud_dec_s
{
	osMessageQId uac_dec_aud_q;
	osMessageQId uac_enc_aud_q;
	osMessageQId uac_ctl_aud_q;

	volatile INT32U	state;

	/* for ISO OUT */
	INT8S*  ppcmbuf;
	INT32U  pcmframesize;
	volatile INT32U	read_index;				/* For audio decode read */
	volatile INT32U	write_index;			/* For USBD write */
	volatile INT32U  bufstate;
	INT32U	channel;
	INT32U	speaker_sample_rate;
	INT32U 	outsendstart;
	INT32U  spkinterval;
	INT32U  spkzero;

	/* for ISO IN */
	INT8S*  pmicbuf;
	INT32U  micframesize;
	INT32U	microphone_sample_rate;
	volatile INT32U	mic_read_index;			/* For USBD read */
	volatile INT32U	mic_write_index;		/* For audio encode write */
	INT32U 	insendstart;

	/* for Interrupt IN */
	INT8S	hidbuf[UACHIDBUFLEN];
	INT8U   hidstatus;
	/* for USB ISO IN/OUT alt interface number */
	INT8U	altinterface1;
	INT8U	altinterface2;
	INT8U  	uac_mute;
	INT8U  	uac_level;
} uac_aud_dec_t;

typedef enum
{
	MSG_UAC_ISO_OUT_EN = 0x6000,
	MSG_UAC_SET_DAC_VOLUME,
	MSG_UAC_HID_SET_IDLE,
	MSG_UAC_HID_IN_PKT_DONE
} MSG_UAC;

typedef enum
{
	UAC_ISO_OUT_IDLE,
	UAC_ISO_OUT_SEND_START,
	UAC_ISO_OUT_STOP
} UAC_ISO_OUT_STA;

typedef enum
{
	MSG_AUDIO_PARSER = 0x1000,
	MSG_AUDIO_START,
	MSG_AUDIO_PAUSE,
	MSG_AUDIO_RESUME,
	MSG_AUDIO_SEEK,
	MSG_AUDIO_STOP,
	MSG_AUDIO_EXIT,
	MSG_AUDIO_MAX
}MSG_AUDIO;

#define WBVAL(x) ((x) & 0xFF),(((x) >> 8) & 0xFF)
#define B3VAL(x) (x & 0xFF),((x >> 8) & 0xFF),((x >> 16) & 0xFF)
#define DBVAL(x) ((x) & 0xFF),(((x) >> 8) & 0xFF),(((x) >> 16) & 0xFF),(((x) >> 24) & 0xFF)

#define MIC_AS_TIMER_A     0
#define MIC_AS_TIMER_B     1
#define MIC_AS_TIMER_C     2
#define MIC_AS_TIMER_D     3
#define MIC_AS_TIMER_E     4
#define MIC_AS_TIMER_F     5

extern void print_string(CHAR *fmt, ...);

#endif
