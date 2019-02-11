/************************************************************
* tutk_avapi.c
*
* Purpose: For TUTK audio/video streaming service
*
* Author: Eugene Hsu
*
* Date: 2016/04/29
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
//#include "drv_l1_gpio.h"
#include "video_encoder.h"
#include "avi_encoder_app.h"
#include "application.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"
#include "wifi_demo.h"
#include "drv_l1_dma.h"
//#include "drv_l1_dac.h"
#include "drv_l2_display.h"
#include "audio_encoder.h"

/**********************************************
*	Definitions
**********************************************/
#define TUTK_STACK_SIZE				1024*4
#define TUTK_Q_NUM					16
#define TUTK_AUDIO_PCM_SR			8000
#define TUTK_AUDIO_FRAME_SIZE		320
#define TUTK_VIDEO_FPS 				30
#define TUTK_JPEG_Q 				0
#define TUTK_JPEG_WIDTH				320
#define TUTK_JPEG_HEIGHT			240
#define TUTK_TRANSFER_JPEG_SIZE		(14*1024)
#define DIS_WIDTH					320
#define DIS_HEIGHT					240

#define TUTK_VIDEO_BUF_IDLE_STATE	0x00
#define TUTK_BUF_GSPI_TX_STATE		0x01

#define TUTK_SPEAKER_STACK_SIZE			1024
#define TUTK_SPK_Q_NUM					20
#define TUTK_SPEAKER_RING_BUFF_LEN		20
#define TUTK_SPEAKER_WRITE_START_INDEX	5
#define TUTK_SPEAKER_BUFF_SIZE			640
#define TUTK_SPEAKER_SAMPLE_RATE		8000

typedef struct tutk_aud_dec_s
{
	volatile INT32U	read_index;				/* For audio decode read */
	volatile INT32U	write_index;			/* For TUTK audio write */
	volatile INT32U	dac_flag;
	INT32U	 sample_rate;
} tutk_aud_dec_t;

typedef enum
{
	TUTK_SPEAKER_STOP_EVENT = 0x10,
	TUTK_SPEAKER_START_EVENT,
	TUTK_SPEAKER_DATA_EVENT
} TUTK_SPEAKER_EVENT_E;

/**********************************************
*	Extern variables and functions
**********************************************/
/*********************************************
*	Variables declaration
*********************************************/
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *tutk_q = NULL;
static OS_EVENT *tutk_spk_q = NULL;
static OS_EVENT *tutk_rx_sem;
static void *tutk_q_stack[TUTK_Q_NUM];
static void *tutk_spk_q_stack[TUTK_SPK_Q_NUM];
static INT32U TUTKTaskStack[TUTK_STACK_SIZE];
static INT32U TUTKSPKTaskStack[TUTK_SPEAKER_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	static osSemaphoreDef_t  tutk_rx_sem_d = {0};
	static osSemaphoreId tutk_rx_sem;
	static osMessageQDef_t tutk_q_d = { TUTK_Q_NUM, sizeof(INT32U), 0 };
	static osMessageQId tutk_q = NULL;
	static osMessageQDef_t tutk_spk_q_d = { TUTK_SPK_Q_NUM, sizeof(INT32U), 0 };
	static osMessageQId tutk_spk_q = NULL;
	extern osMessageQId video_stream_q;
#endif


static char TUTKSPKDataBuf[TUTK_SPEAKER_RING_BUFF_LEN*TUTK_SPEAKER_BUFF_SIZE];
static INT32U tutk_first_init = 0;
static INT32U tutk_video_send = 0;
static INT32U tutk_audio_send = 0;
static INT32U tutk_video_buf_sta = TUTK_VIDEO_BUF_IDLE_STATE;
static INT32U tutk_cur_video_buf_addr = 0;
static INT32U tutk_cur_video_len = 0;
static INT32U tutk_audio_send_done = 0;

static tutk_aud_dec_t tutk_speaker_ctl =
{
	0,
	0,
	0,
	TUTK_SPEAKER_SAMPLE_RATE
};

int pcm_to_unsign(char *out_stream, short *in_pcm, int in_size)
{
	INT32U i = in_size;

	for(i=0; i<in_size; i++)
	{
		*out_stream++ = (*in_pcm++ ^= 0x8000) >> 8;
	}

	return in_size;
}

void tutk_audio_record_cbk(short* buf, int len)
{
	int ret;
	WIFI_MSG_T* msg = NULL;

	if(tutk_audio_send_done == 0)
		return;

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
	//ret = alaw_enc(msg->buf, buf, len);
	ret = pcm_to_unsign(msg->buf, buf, len);
	msg->event = GSPI_AUDIO_STREAM_SEND_EVENT;

	//DBG_PRINT("enc, buf 0x%x ret %d\r\n", msg->buf, ret);
  #if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(tutk_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(tutk_q, (uint32_t)&msg, osWaitForever);
	#endif
}

void wifi_tutk_display_callback(INT32U (*disp)(INT16U, INT16U, INT32U))
{
	if(disp) {
		videnc_display = disp;
	}
}

static INT32U Display_Callback(INT16U w, INT16U h, INT32U addr)
{
    R_TFT_FBI_ADDR = addr;
}

void tutk_video_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
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
			DBG_PRINT("v msg buffer allocate failed\r\n");
			return;
		}
		memcpy(msg->buf, buf, msg->len);
	}
  #if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(tutk_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(tutk_q, (uint32_t)&msg, osWaitForever);
	#endif

}

void tutk_audio_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
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
			DBG_PRINT("a msg buffer allocate failed\r\n");
			return;
		}
		memcpy(msg->buf, buf, msg->len);
	}

  #if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(tutk_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(tutk_q, (uint32_t)&msg, osWaitForever);
	#endif
}

static void _video_encode_exit(void)
{
	INT32S nRet;

	if(video_encode_status() == VIDEO_CODEC_PROCESSING) {
   		video_encode_stop();
   	}
   	video_encode_preview_stop();

    nRet = avi_encode_state_task_del();
	if(nRet < 0) {
		DBG_PRINT("avi_encode_state_task_del fail !!!");
	}

	nRet = scaler_task_del();
	if(nRet < 0) {
		DBG_PRINT("scaler_task_del fail !!!");
	}

	nRet = video_encode_task_del();
	if(nRet < 0) {
		DBG_PRINT("video_encode_task_del fail !!!");
	}

	DBG_PRINT("avi encode all task delete success!!!\r\n");
}

static void tutk_video_encode_start(void)
{
    // for 0308 re-start
    (*((volatile INT32U *) 0xD0000064)) = 0x1d; // sensor internal power 2.8V
	VIDEO_ARGUMENT arg;
    drv_l2_display_init();
    drv_l2_display_start(DISDEV_TFT, DISP_FMT_RGB565);
    wifi_tutk_display_callback(Display_Callback);

//	tft_init();
//	tft_start(C_DISPLAY_DEVICE);

//	user_defined_video_codec_entrance();
	video_encode_entrance();     //Initialize and allocate the resources needed by Video decoder

	arg.bScaler = 1;
	arg.TargetWidth = TUTK_JPEG_WIDTH;
	arg.TargetHeight = TUTK_JPEG_HEIGHT;
	arg.SensorWidth	= 1280; //640;
	arg.SensorHeight = 720+2;  //480;
	arg.DisplayWidth = DIS_WIDTH;		//display width
	arg.DisplayHeight = DIS_HEIGHT;		//display height
	arg.DisplayBufferWidth = DIS_WIDTH;	//display buffer width
	arg.DisplayBufferHeight = DIS_HEIGHT;	//display buffer height

	arg.VidFrameRate = TUTK_VIDEO_FPS;
	arg.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV; //for display
//	arg.JPEG_Q = TUTK_JPEG_Q;
	video_encode_stream_start(arg);

}

static void tutk_video_encode_stop(void)
{
//	tft_init();
	/* Only stop video related task */
	video_encode_stream_stop();
	video_encode_exit();

	drv_l2_display_uninit();
	(*((volatile INT32U *) 0xD0000064)) = 0x0d; // sensor internal power 2.8V
	DBG_PRINT("End of tutk_video_encode_stop\r\n");
}

void tutk_video_update_current_jpeg(INT8U* buf, INT32U len)
{
	WIFI_MSG_T* msg = NULL;

	if(tutk_video_buf_sta == TUTK_VIDEO_BUF_IDLE_STATE && tutk_video_send && len <= TUTK_TRANSFER_JPEG_SIZE)
	{
		msg = (WIFI_MSG_T*)gp_malloc(sizeof(WIFI_MSG_T));
		if(msg == NULL)
		{
			DBG_PRINT("Alloc video msg failed\r\n");
			return;
		}
		tutk_video_buf_sta = TUTK_BUF_GSPI_TX_STATE;
		msg->event = GSPI_VIDEO_UPDATE_FRAME_EVENT;
		msg->buf = (char*)buf;
		msg->len = len;
		//DBG_PRINT("TUTK JPEG len 0x%x\r\n", len);

	  #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(tutk_q, (void*)msg);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut(tutk_q, (uint32_t)&msg, osWaitForever);
		#endif
	}
	else
	{
		if(len > TUTK_TRANSFER_JPEG_SIZE)
			DBG_PRINT("Big pic len 0x%x > 0x%x\r\n", len, TUTK_TRANSFER_JPEG_SIZE);
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		avi_encode_post_empty(video_stream_q, (INT32U)buf);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut(video_stream_q, (uint32_t)&buf, osWaitForever);
		#endif
	}
}

static void tutk_audio_start(void)
{
	INT32S ret;
	MEDIA_SOURCE	media_source;

	if(audio_encode_status() == AUDIO_CODEC_PROCESS_END)
	{
		media_source.type = SOURCE_TYPE_USER_DEFINE;

		media_source.Format.AudioFormat = AUDIO_STREAM_PCM;
//ming		ret = audio_encode_start(media_source, TUTK_AUDIO_PCM_SR, 0);
		ret = audio_encode_start(media_source, BUILD_IN_MIC, TUTK_AUDIO_PCM_SR, 0);
		if (ret != 0)
		{
			DBG_PRINT( "Audio start failed(error code:%d).\r\n" , ret);
		}
	}
}

INT32U tutk_get_audio_frame_size(void)
{
	return TUTK_AUDIO_FRAME_SIZE;
}

static void tutk_audio_stop(void)
{
	audio_encode_stop();
}

void tutk_speaker_task(void *parm)
{
	INT32U msg, addr;
	INT32S  i, nRet;
	INT8U err;
	#if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif

	DBG_PRINT("Enter tutk_speaker_task...\r\n");

	while(1)
	{
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg = (INT32U) OSQPend(tutk_spk_q, 0, &err);
		if(err != OS_NO_ERR || msg == NULL)
			continue;
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		result = osMessageGet(tutk_spk_q, osWaitForever);
		msg = result.value.v;
		if (result.status != osEventMessage)
			continue;
		#endif


		switch(msg)
		{
			case C_DMA_STATUS_DONE:
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPend(tutk_rx_sem, 0, &err);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreWait(tutk_rx_sem, osWaitForever);
				#endif

				if(tutk_speaker_ctl.read_index != tutk_speaker_ctl.write_index)
				{
					addr = (INT32U)(TUTKSPKDataBuf + (tutk_speaker_ctl.read_index * TUTK_SPEAKER_BUFF_SIZE));
					if(++tutk_speaker_ctl.read_index == TUTK_SPEAKER_RING_BUFF_LEN)
					{
						tutk_speaker_ctl.read_index = 0;
					}
					drv_l1_dac_cha_dbf_set((INT16S *)addr, TUTK_SPEAKER_BUFF_SIZE >> 1);
				}
				else
				{
					tutk_speaker_ctl.dac_flag = 3;
					DBG_PRINT("DAC DMA read empty\r\n");
				}
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(tutk_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        osSemaphoreRelease(tutk_rx_sem);
        #endif

				break;

			case TUTK_SPEAKER_DATA_EVENT:
				if(tutk_speaker_ctl.dac_flag == 0)
				{
					#if (_OPERATING_SYSTEM == _OS_UCOS2)
					OSSemPend(tutk_rx_sem, 0, &err);
					#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
					osSemaphoreWait(tutk_rx_sem, osWaitForever);
					#endif

					/* wait write buffer upto specified number */
					if(tutk_speaker_ctl.write_index >= TUTK_SPEAKER_WRITE_START_INDEX)
					{
						tutk_speaker_ctl.dac_flag = 1;
						addr = (INT32U)(TUTKSPKDataBuf + (tutk_speaker_ctl.read_index * TUTK_SPEAKER_BUFF_SIZE));
						drv_l1_dac_cha_dbf_put((INT16S *)addr, TUTK_SPEAKER_BUFF_SIZE, tutk_spk_q);
						if(++tutk_speaker_ctl.read_index == TUTK_SPEAKER_RING_BUFF_LEN)
						{
							tutk_speaker_ctl.read_index = 0;
						}
					}
					#if (_OPERATING_SYSTEM == _OS_UCOS2)
					OSSemPost(tutk_rx_sem);
					#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        	osSemaphoreRelease(tutk_rx_sem);
        	#endif

				}
				else if(tutk_speaker_ctl.dac_flag == 1)
				{

					#if (_OPERATING_SYSTEM == _OS_UCOS2)
					OSSemPend(tutk_rx_sem, 0, &err);
					#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
					osSemaphoreWait(tutk_rx_sem, osWaitForever);
					#endif

					addr = (INT32U)(TUTKSPKDataBuf + (tutk_speaker_ctl.read_index * TUTK_SPEAKER_BUFF_SIZE));
					if(++tutk_speaker_ctl.read_index == TUTK_SPEAKER_RING_BUFF_LEN)
					{
						tutk_speaker_ctl.read_index = 0;
					}

					tutk_speaker_ctl.dac_flag = 2;
					// mute
					nRet = drv_l1_dac_pga_get();
					drv_l1_dac_pga_set(0);
					// dac start
					drv_l1_dac_cha_dbf_set((INT16S *)addr, TUTK_SPEAKER_BUFF_SIZE >> 1);
					drv_l1_dac_mono_set();
					//dac_stereo_set();
					drv_l1_dac_sample_rate_set(tutk_speaker_ctl.sample_rate);
					// volume up
					for (i=0; i<=nRet; i++) {
						drv_l1_dac_pga_set(i);
					}

					#if (_OPERATING_SYSTEM == _OS_UCOS2)
					OSSemPost(tutk_rx_sem);
					#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        	osSemaphoreRelease(tutk_rx_sem);
        	#endif
				}
				else if(tutk_speaker_ctl.dac_flag == 3)
				{

					#if (_OPERATING_SYSTEM == _OS_UCOS2)
					OSSemPend(tutk_rx_sem, 0, &err);
					#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
					osSemaphoreWait(tutk_rx_sem, osWaitForever);
					#endif

					/* No available audio buffer for DAC, check new arrival audio packet */
					if(tutk_speaker_ctl.read_index != tutk_speaker_ctl.write_index)
					{
						addr = (INT32U)(TUTKSPKDataBuf + (tutk_speaker_ctl.read_index * TUTK_SPEAKER_BUFF_SIZE));
						if(++tutk_speaker_ctl.read_index == TUTK_SPEAKER_RING_BUFF_LEN)
						{
							tutk_speaker_ctl.read_index = 0;
						}
						drv_l1_dac_cha_dbf_set((INT16S *)addr, TUTK_SPEAKER_BUFF_SIZE >> 1);
						tutk_speaker_ctl.dac_flag = 2;
					}
					else
					{
						DBG_PRINT("w == r\r\n");
					}

					#if (_OPERATING_SYSTEM == _OS_UCOS2)
					OSSemPost(tutk_rx_sem);
					#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        	osSemaphoreRelease(tutk_rx_sem);
        	#endif
				}
				break;

			case TUTK_SPEAKER_STOP_EVENT:
				drv_l1_dac_dbf_free();
				break;


			default:
				break;
		}
	}

	/* Exit task */
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(TUTK_SPEAKER_PRIORITY);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	id = osThreadGetId();
  osThreadTerminate(id);
  #endif

}

void tutk_avapi_task(void *parm)
{
	INT8U err;
	INT8S res;
	WIFI_MSG_T* msg = NULL;
	INT32U message, copy_len, copy_addr;
	#if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif


	DBG_PRINT("Enter TUTK AVAPI task...\r\n");

	audio_stream_register_cbk((void*)tutk_audio_record_cbk, (void*)tutk_get_audio_frame_size);

	/* Enable recode task */
	audio_encode_entrance();

	while(1)
	{
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg = (WIFI_MSG_T*) OSQPend(tutk_q, 0, &err);
		if(err != OS_NO_ERR || msg == NULL)
			continue;
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		result = osMessageGet(tutk_q, osWaitForever);
		msg = (WIFI_MSG_T*)result.value.v;
		if (result.status != osEventMessage)
			continue;
		#endif

		switch(msg->event)
		{
			case GSPI_VIDEO_STREAM_START_EVENT:
				if(tutk_video_send == 0)
				{
					DBG_PRINT("VIDEO START event\r\n");
					tutk_video_send = 1;
					tutk_video_buf_sta = TUTK_VIDEO_BUF_IDLE_STATE;
					tutk_video_encode_start();
					stream_enc_start();
				}
				break;

			case GSPI_VIDEO_STREAM_STOP_EVENT:
				if(tutk_video_send == 1)
				{
					DBG_PRINT("VIDEO STOP event\r\n");
					tutk_video_send = 0;
					tutk_video_buf_sta = TUTK_VIDEO_BUF_IDLE_STATE;
					tutk_video_encode_stop();
                    DBG_PRINT("HEAP SIZE = %d \r\n", xPortGetFreeHeapSize());
				}
				break;

			case GSPI_VIDEO_DATA_DONE_EVENT:
				tutk_video_buf_sta = TUTK_VIDEO_BUF_IDLE_STATE;
				//DBG_PRINT("VIDEO DONE event\r\n");
				break;

			case GSPI_VIDEO_UPDATE_FRAME_EVENT:
				/* Send from MJPEG encode task */
				tutk_cur_video_buf_addr = (INT32U)msg->buf;
				tutk_cur_video_len = msg->len;
				/* Send MJPEG header to client and triggle slave to get JPEG data from host */
retry_video:
				res = gspi_transfer_video((INT8U*)msg->buf, msg->len);
				if(res)
				{
					goto retry_video;
				}
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				avi_encode_post_empty(video_stream_q, (INT32U)msg->buf);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                osMessagePut(video_stream_q, (uint32_t)&msg->buf, osWaitForever);
                #endif

				break;

			case GSPI_AUDIO_STREAM_START_EVENT:
				if(tutk_audio_send == 0)
				{
					DBG_PRINT("AUDIO stream START event\r\n");
					tutk_audio_send = 1;
					tutk_audio_start();
					tutk_audio_send_done = 1;
				}
				break;

			case GSPI_AUDIO_STREAM_STOP_EVENT:
				if(tutk_audio_send == 1)
				{
					DBG_PRINT("AUDIO stream STOP event\r\n");
					tutk_audio_send = 0;
					tutk_audio_stop();
					tutk_audio_send_done = 1;
				}
				break;

			case GSPI_AUDIO_STREAM_SEND_EVENT:
				if(tutk_audio_send)
				{
					//DBG_PRINT("Send RTP event len %d, 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", msg->len, msg->buf[0], msg->buf[1], msg->buf[2],
								//msg->buf[3], msg->buf[4], msg->buf[5]);
					gspi_transfer_audio((INT8U*)msg->buf, msg->len);
				}
				break;

			case GSPI_AUDIO_DATA_DONE_EVENT:
				tutk_audio_send_done = 1;
				break;

			case GSPI_AUDIO_SPEAKER_START_EVENT:
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPend(tutk_rx_sem, 0, &err);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreWait(tutk_rx_sem, osWaitForever);
				#endif
				DBG_PRINT("AUDIO speaker START event\r\n");
				tutk_speaker_ctl.read_index = 0;
				tutk_speaker_ctl.write_index = 0;
				tutk_speaker_ctl.dac_flag = 0;
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(tutk_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(tutk_rx_sem);
				#endif
				break;

			case GSPI_AUDIO_SPEAKER_STOP_EVENT:

				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPend(tutk_rx_sem, 0, &err);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreWait(tutk_rx_sem, osWaitForever);
				#endif
				DBG_PRINT("AUDIO speaker STOP event\r\n");
				tutk_speaker_ctl.read_index = 0;
				tutk_speaker_ctl.write_index = 0;
				tutk_speaker_ctl.dac_flag = 0xff;
				message = TUTK_SPEAKER_STOP_EVENT;
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(tutk_spk_q, (void*)message);
				OSSemPost(tutk_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osMessagePut(tutk_spk_q, (uint32_t)&message, osWaitForever);
				osSemaphoreRelease(tutk_rx_sem);
				#endif

				break;

			case GSPI_AUDIO_SPEAKER_DATA_EVENT:

				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPend(tutk_rx_sem, 0, &err);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreWait(tutk_rx_sem, osWaitForever);
				#endif
				if(msg->len > TUTK_SPEAKER_BUFF_SIZE)
					copy_len = TUTK_SPEAKER_BUFF_SIZE;
				else
					copy_len = msg->len;

				copy_addr = (INT32U)(TUTKSPKDataBuf + (tutk_speaker_ctl.write_index * TUTK_SPEAKER_BUFF_SIZE));
				memcpy((char*)copy_addr, msg->buf, copy_len);
				if(++tutk_speaker_ctl.write_index == TUTK_SPEAKER_RING_BUFF_LEN)
				{
					tutk_speaker_ctl.write_index = 0;
				}
				message = TUTK_SPEAKER_DATA_EVENT;


				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(tutk_spk_q, (void*)message);
				OSSemPost(tutk_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osMessagePut(tutk_spk_q, (uint32_t)&message, osWaitForever);
				osSemaphoreRelease(tutk_rx_sem);
				#endif
				//DBG_PRINT("Size %d\r\n", msg->len);
				break;

			default:
				break;
		}

		/* Free message and realted buffers */
		if(msg->buf && msg->event != GSPI_VIDEO_UPDATE_FRAME_EVENT)
		{
			gp_free(msg->buf);
		}
		gp_free(msg);
	}

	/* Exit task */
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(TUTK_PRIORITY);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	id = osThreadGetId();
  osThreadTerminate(id);
  #endif
}

void tutk_avapi_task_init(void)
{
	INT8U err;

	if(tutk_first_init)
		return;

	/* Task only init once */
	tutk_first_init = 1;

	if(tutk_q == NULL)
	{
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		tutk_q = OSQCreate(tutk_q_stack, TUTK_Q_NUM);
		if(tutk_q == 0)
		{
			DBG_PRINT("Create tutk_q failed\r\n");
			return;
		}
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		tutk_q = osMessageCreate(&tutk_q_d, NULL);
		if(tutk_q == 0)
		{
			DBG_PRINT("Create tutk_q failed\r\n");
			return;
		}
		#endif
	}

	if(tutk_spk_q == NULL)
	{
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		tutk_spk_q = OSQCreate(tutk_spk_q_stack, TUTK_SPK_Q_NUM);
		if(tutk_spk_q == 0)
		{
			DBG_PRINT("Create tutk_spk_q failed\r\n");
			return;
		}
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		tutk_spk_q = osMessageCreate(&tutk_spk_q_d, NULL);
		if(tutk_spk_q == 0)
		{
			DBG_PRINT("Create tutk_q failed\r\n");
			return;
		}
		#endif
	}

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	tutk_rx_sem = OSSemCreate(1);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	tutk_rx_sem = osSemaphoreCreate(&tutk_rx_sem_d, 1);
	#endif

	register_update_cur_jpeg_cbk((INT32U)tutk_video_update_current_jpeg);

	gspi_register_app_cbk((INT32U)tutk_video_gspi_cbk, GSPI_VIDEO_STREAM_APP);

	gspi_register_app_cbk((INT32U)tutk_audio_gspi_cbk, GSPI_AUDIO_STREAM_APP);

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	err = OSTaskCreate(tutk_avapi_task, (void *)NULL, &TUTKTaskStack[TUTK_STACK_SIZE - 1], TUTK_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Can't create tutk avapi task\n\r");
		return;
	}
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	osThreadDef_t tutk_avapi_task_ent = {"tutk_avapi_task", (os_pthread) tutk_avapi_task, osPriorityNormal, 1, TUTK_STACK_SIZE};
	id = osThreadCreate(&tutk_avapi_task_ent, (void*)NULL);
	if (id== 0)
	{
		DBG_PRINT("Cannot create tutk avapi task\n\r");
		return;
	}
  #endif
	else
	{
		DBG_PRINT("Create tutk avapi task ok\r\n");
	}

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	err = OSTaskCreate(tutk_speaker_task, (void *)NULL, &TUTKSPKTaskStack[TUTK_SPEAKER_STACK_SIZE - 1], TUTK_SPEAKER_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Can't create tutk speaker task\n\r");
		return;
	}
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadDef_t tutk_speaker_task_ent = {"tutk_speaker_task", (os_pthread) tutk_speaker_task, osPriorityNormal, 1, TUTK_SPEAKER_STACK_SIZE};
	id = osThreadCreate(&tutk_speaker_task_ent, (void*)NULL);
	if (id== 0)
	{
		DBG_PRINT("Cannot create tutk speaker task\n\r");
		return;
	}
  #endif
	else
	{
		DBG_PRINT("Create tutk speaker task ok\r\n");
	}
}
