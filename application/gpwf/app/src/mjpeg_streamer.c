/************************************************************
* mjpeg_streamer.c
*
* Purpose: Demo code for MJPEG streamer
*
* Author: Eugene Hsu
*
* Date: 2015/10/01
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
#include "application.h"
#include "video_encoder.h"
#include "avi_encoder_app.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"
#include "wifi_demo.h"
#include "gspi_master_drv.h"
#include "drv_l2_display.h"
#include "drv_l2_sensor.h"

/**********************************************
*	Definitions
**********************************************/
#define MJPEGSTREAMER_STACK_SIZE	1024*4//
#define MJPEG_FPS 					30
#define MJPEG_Q 					0
#if PATTERN_DEMO_EN == 1
#define MJPEG_JPEG_WIDTH			640
#define MJPEG_JPEG_HEIGHT			480
#define MJPEG_STREAMER_Q_NUM		16
#define DIS_WIDTH					640
#define DIS_HEIGHT					480
#else
#define MJPEG_JPEG_WIDTH			320
#define MJPEG_JPEG_HEIGHT			240
#define MJPEG_STREAMER_Q_NUM		16
#define DIS_WIDTH					320
#define DIS_HEIGHT					240
#endif


#define MJPEG_BUF_IDLE_STATE		0
#define MJPEG_BUF_GSPI_TX_STATE		1
/**********************************************
*	Extern variables and functions
**********************************************/
/*********************************************
*	Variables declaration
*********************************************/

#if (_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *mjpeg_sem;
static OS_EVENT *mjpeg_streamer_q = NULL;
static void *mjpeg_stremer_q_stack[MJPEG_STREAMER_Q_NUM];
static INT32U MjpegStreamerTaskStack[MJPEGSTREAMER_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	static osSemaphoreDef_t  mjpeg_sem_d = {0};
	static osSemaphoreId mjpeg_sem;
	static osMessageQDef_t mjpeg_streamer_q_d = { MJPEG_STREAMER_Q_NUM, sizeof(INT32U), 0 };
	static osMessageQId mjpeg_streamer_q = NULL;
	extern osMessageQId video_stream_q;
#endif

INT32U mjpeg_buf_addr = 0;
INT32U mjpeg_buf_len = 0;
INT32U mjpeg_buf_sta = MJPEG_BUF_IDLE_STATE;
INT32U send_mjpeg = 0;
INT32U mjpeg_req_len = 0;
static INT32U mjpeg_first_init = 0;
volatile INT32U mjpeg_update_addr = 0;
volatile INT32U mjpeg_update_len = 0;
volatile INT32U mjpeg_gspi_tx_addr = 0;
volatile INT32U mjpeg_gspi_tx_len = 0;

void wifi_mjpeg_display_callback(INT32U (*disp)(INT16U, INT16U, INT32U))
{
	if(disp) {
		videnc_display = disp;
	}
}
static INT32U Display_Callback(INT16U w, INT16U h, INT32U addr)
{
    R_TFT_FBI_ADDR = addr;
}


static void mjpeg_video_encode_start(void)
{
	VIDEO_ARGUMENT arg;

    //drv_l2_display_init();
    //drv_l2_display_start(DISDEV_TFT, DISP_FMT_RGB565);
    //wifi_mjpeg_display_callback(Display_Callback);

	//drv_l1_tft_init();
	//tft_start(C_DISPLAY_DEVICE);

//m	user_defined_video_codec_entrance();
	video_encode_entrance();     //Initialize and allocate the resources needed by Video decoder

	arg.bScaler = 1;
	arg.TargetWidth = MJPEG_JPEG_WIDTH;
	arg.TargetHeight = MJPEG_JPEG_HEIGHT;
#if _SENSOR_GC0308_CSI == 1
	arg.SensorWidth	= 640;
	arg.SensorHeight = 480;
#elif _SENSOR_H42_CDSP_MIPI == 1
	arg.SensorWidth	= 1280;
	arg.SensorHeight = 722;
#else
    //---change sensor size---
	arg.SensorWidth	= 1280;
	arg.SensorHeight = 722;
#endif
	arg.DisplayWidth = DIS_WIDTH;		//display width
	arg.DisplayHeight = DIS_HEIGHT;		//display height
	arg.DisplayBufferWidth = DIS_WIDTH;	//display buffer width
	arg.DisplayBufferHeight = DIS_HEIGHT;	//display buffer height

	arg.VidFrameRate = MJPEG_FPS;
	arg.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV; //for display
//	arg.JPEG_Q = MJPEG_Q;
//	video_encode_stream_start(arg);
	R_FUNPOS1 &= ~((0x3<<8)|(0x3<<3));
//	video_encode_preview_start(arg);
	video_encode_stream_start(arg);

}

static void mjpeg_video_encode_stop(void)
{
    drv_l2_display_uninit();//

    video_encode_stream_stop();
	video_encode_exit();

//m	video_encode_stream_stop();
}

static void mjpeg_gspi_cmd_cbk(INT32U buf, INT32U len, INT32U event)
{
	INT32U msg = event;

	if(event == GSPI_MJPEG_GET_DATA_EVENT)
	{
		mjpeg_req_len = len;
	}
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(mjpeg_streamer_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(mjpeg_streamer_q, (uint32_t)&msg, osWaitForever);
	#endif
}

static void mjpeg_stream_fps_info_cbk(void* data)
{
	GP_FPS_T* fpsptr = (GP_FPS_T*)data;
	DBG_PRINT("%2d NET FPS, RSSI %2d\r\n", fpsptr->net_fps, fpsptr->rssi);
}

void mjpeg_update_current_jpeg(INT8U* buf, INT32U len)
{
	INT32U msg;

	if(mjpeg_buf_sta == MJPEG_BUF_IDLE_STATE && send_mjpeg)
	{
		msg = GSPI_MJPEG_UPDATE_FRAME_EVENT;
		mjpeg_update_addr = (INT32U)buf;
		mjpeg_update_len = len;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(mjpeg_streamer_q, (void*)msg);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut(mjpeg_streamer_q, (uint32_t)&msg, osWaitForever);
		#endif
	}
	else
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		avi_encode_post_empty(video_stream_q, (INT32U)buf);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut(video_stream_q, (uint32_t)&buf, osWaitForever);
		#endif
	}
}

void mjpeg_gspi_send_current_jpeg_data(void)
{
	INT32U transfer_len;

	if(mjpeg_gspi_tx_len != 0)
	{
		transfer_len = mjpeg_req_len;
		if(transfer_len > mjpeg_gspi_tx_len)
		{
			transfer_len = mjpeg_gspi_tx_len;
		}

		//DBG_PRINT("send: txbuf 0x%x , len %d\r\n", mjpeg_gspi_tx_addr, transfer_len);

		gspi_transfer_mjpeg((INT8U*)mjpeg_gspi_tx_addr, transfer_len);

		mjpeg_gspi_tx_addr += transfer_len;
		mjpeg_gspi_tx_len -= transfer_len;


		if(mjpeg_gspi_tx_len == 0)
		{
			mjpeg_gspi_tx_addr = 0;
			mjpeg_buf_sta = MJPEG_BUF_IDLE_STATE;
			//DBG_PRINT(" EOF buf 0x%x len %d\r\n", mjpeg_buf_addr, mjpeg_buf_len);
		}
		//DBG_PRINT(" tx[%d] ", mjpeg_gspi_tx_len);
	}
}

void mjpeg_streamer_task(void *parm)
{
	INT32U msg_id;
	INT8U err;
	#if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif
//test m start
//		mjpeg_video_encode_start();//
//test m end

	while(1)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg_id = (INT32U) OSQPend(mjpeg_streamer_q, 0, &err);
		if(err != OS_NO_ERR)
			continue;

		OSSemPend(mjpeg_sem, 0, &err);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		result = osMessageGet(mjpeg_streamer_q, osWaitForever);
		msg_id = result.value.v;
		if (result.status != osEventMessage)
			continue;
		osSemaphoreWait(mjpeg_sem, osWaitForever);
		#endif

		switch(msg_id)
		{
			case GSPI_MJPEG_STOP_EVENT:
				send_mjpeg = 0;
				/* Stop video encode */
				mjpeg_video_encode_stop();
				mjpeg_buf_sta = MJPEG_BUF_IDLE_STATE;
				mjpeg_gspi_tx_addr = 0;
				mjpeg_gspi_tx_len = 0;
				mjpeg_update_addr = 0;
				mjpeg_update_len = 0;
				if(mjpeg_buf_addr)
				{
                    #if (_OPERATING_SYSTEM == _OS_UCOS2)
                    avi_encode_post_empty(video_stream_q, (INT32U)mjpeg_buf_addr);
                    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                    osMessagePut(video_stream_q, (uint32_t)&mjpeg_buf_addr, osWaitForever);
                    #endif

					mjpeg_buf_addr = 0;
				}
				memset((void*)&gp_fps, 0 ,sizeof(GP_FPS_T));
				break;

			case GSPI_MJPEG_START_EVENT:
				/* Start video encode */
				send_mjpeg = 1;
				mjpeg_buf_sta = MJPEG_BUF_IDLE_STATE;
				mjpeg_video_encode_start();//
				osDelay(1);//
// m test start
				stream_enc_start();
// 			avi_enc_start();
//				pAviEncVidPara->video_format = C_MJPG_FORMAT;
//m test end
				break;

			case GSPI_MJPEG_GET_DATA_EVENT:
				//DBG_PRINT("GSPI_MJPEG_GET_DATA_EVENT len %d\r\n", mjpeg_req_len);
				/* Send A/B buffer size of JPEG to slave*/
				mjpeg_gspi_send_current_jpeg_data();
				break;

			case GSPI_MJPEG_TX_DONE_EVENT:
				/* MJPEG TX done via GSPI */
				mjpeg_buf_sta = MJPEG_BUF_IDLE_STATE;
				break;

			case GSPI_MJPEG_UPDATE_FRAME_EVENT:
				/* Send from MJPEG encode task */
				mjpeg_buf_sta = MJPEG_BUF_GSPI_TX_STATE;
				/* release previous buffer */
				if(mjpeg_buf_addr)
				{
					avi_encode_post_empty(video_stream_q, mjpeg_buf_addr);
				}

				mjpeg_buf_addr = (INT32U)mjpeg_update_addr;
				mjpeg_buf_len = mjpeg_update_len;

				mjpeg_gspi_tx_addr = (INT32U)mjpeg_update_addr;
				mjpeg_gspi_tx_len = mjpeg_update_len;

				/* Send MJPEG header to client and triggle slave to get JPEG data from host */
				//DBG_PRINT("TX HEADER CMD: buf 0x%x len %d\r\n", mjpeg_buf_addr, mjpeg_buf_len);
				gspi_tx_cmd(GSPI_TX_MJPEG_NET_HEADER_CMD, mjpeg_buf_len);
				break;

			default:
				break;
		}
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSSemPost(mjpeg_sem);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osSemaphoreRelease(mjpeg_sem);
		#endif

	}

	if(mjpeg_streamer_q)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQDel(mjpeg_streamer_q, OS_DEL_ALWAYS, &err);
		mjpeg_streamer_q = NULL;
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        #endif
	}
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(MJPEG_STREAMER_PRIORITY);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	id = osThreadGetId();
    osThreadTerminate(id);
    #endif
}

void mjpeg_streamer_task_init(void)
{
	INT8U err;

	if(mjpeg_first_init)
		return;

	/* Task only init once */
	mjpeg_first_init = 1;

	gspi_register_app_cbk((INT32U)mjpeg_gspi_cmd_cbk, GSPI_MJPEG_STREAM_APP);

	gp_cmd_regsiter_info_cbk((void*)mjpeg_stream_fps_info_cbk, GPCMD_FPS_INFO);

    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	mjpeg_sem = OSSemCreate(1);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	mjpeg_sem = osSemaphoreCreate(&mjpeg_sem_d, 1);
	#endif

	register_update_cur_jpeg_cbk((INT32U)mjpeg_update_current_jpeg);

	if(mjpeg_streamer_q == NULL)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		mjpeg_streamer_q = OSQCreate(mjpeg_stremer_q_stack, MJPEG_STREAMER_Q_NUM);
		if(mjpeg_streamer_q == 0)
		{
			DBG_PRINT("Create mjpeg_streamer_q failed\r\n");
			return;
		}
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        mjpeg_streamer_q = osMessageCreate(&mjpeg_streamer_q_d, NULL);
        if(mjpeg_streamer_q == 0)
		{
			DBG_PRINT("Create mjpeg_streamer_q failed\r\n");
			return;
		}
        #endif
	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	err = OSTaskCreate(mjpeg_streamer_task, (void *)NULL, &MjpegStreamerTaskStack[MJPEGSTREAMER_STACK_SIZE - 1], MJPEG_STREAMER_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Cannot create MJPEG streamer task\n\r");
		return;
	}
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	osThreadDef_t mjpeg_streamer_task_ent = {"mjpeg_streamer_task", (os_pthread) mjpeg_streamer_task, osPriorityNormal, 1, MJPEGSTREAMER_STACK_SIZE};
	id = osThreadCreate(&mjpeg_streamer_task_ent, (void*)NULL);
	if (id== 0)
	{
		DBG_PRINT("Cannot create GSPI task\n\r");
		return;
	}
        #endif
#if PATTERN_DEMO_EN == 1
    prcess_task_init();
#endif
}
