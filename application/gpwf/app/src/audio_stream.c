/************************************************************
* audio_stream.c
*
* Purpose: audio streaming
*
* Author: Eugene Hsu
*
* Date: 2016/04/21
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.10
* History :
*
************************************************************/
#include "string.h"
#include "stdio.h"
#include "project.h"
#include "drv_l1_gpio.h"
#include "video_encoder.h"
#include "avi_encoder_app.h"
#include "application.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"
#include "wifi_demo.h"
#include "audio_record.h"


/**********************************************
*	Definitions
**********************************************/
#define AUDIO_STREAM_STACK_SIZE		1024
#define AUDIO_STREAM_Q_NUM			16
#define AUDIO_STREAM_PCM_SR			8000
#define AUDIO_STREAM_PACKET_SIZE	800

/**********************************************
*	Extern variables and functions
**********************************************/
int alaw_enc(char *out_stream, short *in_pcm, int in_size);
/*********************************************
*	Variables declaration
*********************************************/
#if(_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *audio_stream_q = NULL;
static OS_EVENT *audio_stream_rx_sem;
static void *audio_stream_q_stack[AUDIO_STREAM_Q_NUM];
static INT32U AudioStreamTaskStack[AUDIO_STREAM_STACK_SIZE];
#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
	static osSemaphoreDef_t  audio_stream_rx_sem_d = {0};
	static osSemaphoreId audio_stream_rx_sem;
	static osMessageQDef_t audio_stream_q_d = { AUDIO_STREAM_Q_NUM, sizeof(INT32U), 0 };
	static osMessageQId audio_stream_q;
#endif
static INT32U audio_stream_first_init = 0;
static INT32U audio_stream_send = 0;

void audio_stream_record_cbk(short* buf, int len)
{
	int ret;
	WIFI_MSG_T* msg = NULL;

	msg = (WIFI_MSG_T*)gp_malloc(sizeof(WIFI_MSG_T));
	if(msg == NULL)
	{
		DBG_PRINT("Alloc stream msg failed\r\n");
		return;
	}

	msg->len = len;		/* Data size in bytes */
	msg->buf = NULL;
	if(msg->len)
	{
		msg->buf = (char*)gp_malloc(msg->len);
		if(msg->buf == NULL)
		{
			DBG_PRINT("msg buffer allocate failed\r\n");
			return;
		}
	}
	ret = alaw_enc(msg->buf, buf, len);
	msg->event = GSPI_AUDIO_STREAM_SEND_EVENT;

	//DBG_PRINT("enc, buf 0x%x ret %d\r\n", msg->buf, ret);
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(audio_stream_q, (void*)msg);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    osMessagePut(audio_stream_q, (uint32_t)&msg, osWaitForever);
    #endif
}

void audio_stream_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
{
	WIFI_MSG_T* msg = NULL;

	msg = (WIFI_MSG_T*)gp_malloc(sizeof(WIFI_MSG_T));
	if(msg == NULL)
	{
		DBG_PRINT("Alloc gspi cbk msg failed\r\n");
		return;
	}

	msg->event = event;
	msg->len = len;		/* Data size in bytes */
	msg->buf = NULL;
	if(msg->len)
	{
		msg->buf = (char*)gp_malloc(msg->len);
		if(msg->buf == NULL)
		{
			DBG_PRINT("gspi cbk msg buffer allocate failed\r\n");
			return;
		}
		memcpy(msg->buf, buf, msg->len);
	}

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(audio_stream_q, (void*)msg);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    osMessagePut(audio_stream_q, (uint32_t)&msg, osWaitForever);
    #endif
}

static void audio_stream_start(void)
{
	INT32S ret;
	MEDIA_SOURCE	media_source;

	if(audio_encode_status() == AUDIO_CODEC_PROCESS_END)
	{
		media_source.type = SOURCE_TYPE_USER_DEFINE;

		media_source.Format.AudioFormat = AUDIO_STREAM_PCM;
		ret = audio_encode_start(media_source, BUILD_IN_MIC, AUDIO_STREAM_PCM_SR, 0);
		if (ret != 0)
		{
			DBG_PRINT( "Audio start failed(error code:%d).\r\n" , ret);
		}
	}
}

INT32U audio_stream_get_audio_frame_size(void)
{
	return AUDIO_STREAM_PACKET_SIZE;
}

static void audio_stream_stop(void)
{
	audio_encode_stop();
}

void audio_stream_task(void *parm)
{
	INT8U err;
	static INT32U audio_rx_len;
	WIFI_MSG_T* msg = NULL;
    #if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif

	DBG_PRINT("Enter Audio streaming task...\r\n");

	audio_stream_register_cbk((void*)audio_stream_record_cbk, (void*)audio_stream_get_audio_frame_size);

	/* Enable recode task */
	audio_encode_entrance();

	while(1)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg = (WIFI_MSG_T*) OSQPend(audio_stream_q, 0, &err);
		if(err != OS_NO_ERR || msg == NULL)
			continue;
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
       		result = osMessageGet(audio_stream_q, osWaitForever);
			msg = (WIFI_MSG_T*)result.value.v;
			if (result.status != osEventMessage)
				continue;
        #endif

		switch(msg->event)
		{
			case GSPI_AUDIO_STREAM_START_EVENT:
				audio_stream_start();
				audio_stream_send = 1;
				//DBG_PRINT("GSPI_AUDIO_STREAM_START_EVENT\r\n");
				break;

			case GSPI_AUDIO_STREAM_STOP_EVENT:
				//DBG_PRINT("GSPI_AUDIO_STREAM_STOP_EVENT\r\n");
				audio_stream_stop();
				audio_stream_send = 0;
				break;

			case GSPI_AUDIO_STREAM_SEND_EVENT:
				if(audio_stream_send)
				{
					//DBG_PRINT("Send RTP event, buf 0x%x, len %d\r\n", msg->buf, msg->len);
					gspi_transfer_audio((INT8U*)msg->buf, msg->len);
				}
				break;

			case GSPI_RTP_RX_LEN_EVENT:
				audio_rx_len = msg->len;
				DBG_PRINT("RTP RX len %d\r\n", audio_rx_len);
				break;

			case GSPI_RTP_RX_DATA_EVENT:
				break;

			default:
				break;
		}

		/* Free message and realted buffers */
		if(msg->buf)
		{
			gp_free(msg->buf);
		}
		gp_free(msg);
	}

	/* Exit task */
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(AUDIO_STREAM_PRIORITY);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadTerminate(NULL);
	#endif
}

void audio_stream_task_init(void)
{
	INT8U err;
	#if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif

	if(audio_stream_first_init)
		return;

	/* Task only init once */
	audio_stream_first_init = 1;

	if(audio_stream_q == NULL)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		audio_stream_q = OSQCreate(audio_stream_q_stack, AUDIO_STREAM_Q_NUM);
		if(audio_stream_q == 0)
		{
			DBG_PRINT("Create audio_stream_q failed\r\n");
			return;
		}
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		audio_stream_q = osMessageCreate(&audio_stream_q_d, NULL);
        if (audio_stream_q == 0)
		{
			DBG_PRINT("Create audio_stream_q failed\r\n");
			return;
        }
		#endif
	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	audio_stream_rx_sem = OSSemCreate(1);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	audio_stream_rx_sem = osSemaphoreCreate(&audio_stream_rx_sem_d, 1);
    #endif
	gspi_register_app_cbk((INT32U)audio_stream_gspi_cbk, GSPI_AUDIO_STREAM_APP);

     #if (_OPERATING_SYSTEM == _OS_UCOS2)
	err = OSTaskCreate(audio_stream_task, (void *)NULL, &AudioStreamTaskStack[AUDIO_STREAM_STACK_SIZE - 1], AUDIO_STREAM_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Can't create audio stream task\n\r");
		return;
	}
	else
	{
		DBG_PRINT("Create audio stream task ok\r\n");
	}
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	//INT32S ret;
	osThreadId id;
	osThreadDef_t audio_stream_task_ent = {"audio_stream_task", (os_pthread) audio_stream_task, osPriorityNormal, 1, AUDIO_STREAM_STACK_SIZE};
	id = osThreadCreate(&audio_stream_task_ent, (void*)NULL);
	if (id== 0)
	{
		DBG_PRINT("Cannot create GSPI task\n\r");
		return;
	}
	else
	{
		DBG_PRINT("Create audio stream task ok\r\n");
	}
	#endif
}

/* G.711 header */
enum
{
	SIGN_BIT = 0x80,  /* Sign bit for a A-law byte. */
	QUANT_MASK = 0xF, /* Quantization field mask. */
	NSEGS = 8,        /* Number of A-law U-law segments. */
	SEG_SHIFT = 4,    /* Left shift for segment number. */
	SEG_MASK = 0x70,  /* Segment field mask. */
	BIAS = 0x84	     /* Bias for linear code. */
};

/*********************************************************************************
 *  Function : search
 *  Purpose :  find A-law U-law segment
 *    val   :  pcm data(16-bit resolution)
 *********************************************************************************/
static int search(int val)
{
    int i;
    short seg_end[NSEGS] = {0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF, 0x3FFF, 0x7FFF};

    for (i = 0; i < NSEGS; ++i)
    {
        if (val <= seg_end[i])
            return i;
    }
    return NSEGS;
}

/*********************************************************************************
 *  Function : linear2alaw
 *             unsigned char linear2alaw(int pcm_val)
 *  Purpose :  Convert a 16-bit linear PCM value to 8-bit A-law
 *  pcm_val :  pcm data(16-bit resolution)
 *  return  :  A-Law data(8-bit resolution)
 *
 *
 *		Linear Input Code	Compressed Code
 *	------------------------	---------------
 *	0000000wxyza			000wxyz
 *	0000001wxyza			001wxyz
 *	000001wxyzab			010wxyz
 *	00001wxyzabc			011wxyz
 *	0001wxyzabcd			100wxyz
 *	001wxyzabcde			101wxyz
 *	01wxyzabcdef			110wxyz
 *	1wxyzabcdefg			111wxyz
 *
 * For further information see John C. Bellamy's Digital Telephony, 1982,
 * John Wiley & Sons, pps 98-111 and 472-476.
 *********************************************************************************/
static unsigned char linear2alaw(short pcm_val)	/* 2's complement (16-bit range) */
{
    int		mask;
    int		seg;
    unsigned char	aval;

    if (pcm_val >= 0)
    {
        mask = 0xD5;		/* sign (7th) bit = 1 */
    }
    else
    {
        mask = 0x55;		/* sign bit = 0 */
        pcm_val = -pcm_val - 8;
    }
    if (pcm_val < 0) pcm_val = 0;

    /* Convert the scaled magnitude to segment number. */
    seg = search(pcm_val);

    /* Combine the sign, segment, and quantization bits. */
    if (seg >= 8)		/* out of range, return maximum value. */
        return (0x7F ^ mask);
    else
    {
        aval = seg << SEG_SHIFT;
        if (seg < 2)
            aval |= (pcm_val >> 4) & QUANT_MASK;
        else
            aval |= (pcm_val >> (seg + 3)) & QUANT_MASK;
        return (aval ^ mask);
    }
}

/*********************************************************************************
 *  Function : alaw_enc
 *  Purpose :  compress speech data	about 64000 bps
 *    y		: pointer of A-Law bitstream
 *    x         : pointer of speech data
 *    size      : total speech length
 *    return    : bitstream length
**********************************************************************************/
int alaw_enc(char *out_stream, short *in_pcm, int in_size)
{
    /* encode variable */
	int size = in_size;

	do {
        *out_stream++ = linear2alaw( *in_pcm++ );
    }while(size-->0);

    return in_size;
}
