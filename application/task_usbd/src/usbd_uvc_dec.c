/*******************************************************
    Include file
*******************************************************/
#include "drv_l1_sfr.h"
#include "drv_l1_timer.h"
#include "drv_l1_uart.h"
#include "application.h"
#include "drv_l1_usbd.h"
#include "drv_l2_usbd.h"
#include "drv_l2_usbd_uvc_uc.h"
#include "project.h"
//#include "drv_l1_clock.h"
//#include "drv_l2_display.h"
//#include "drv_l2_ad_key_scan.h"
#include "drv_l1_pscaler.h"
#include "avi_encoder_app.h"
#include "usbd_uac.h"
#include "usbd_uvc.h"

#define UVC_YUV_FRAME_SIZE (0x96000)

#if UVC_H264 == 1//720P

#if (USBD_UVC_CAM_RES == CAM_RES_720P)
    #define TAR_WIDTH		1280
    #define TAR_HEIGHT		720
    #define SNR_WIDTH		1280
    #define SNR_HEIGHT		720
    #define DIP_WIDTH		1280
    #define DIP_HEIGHT		720
#elif(USBD_UVC_CAM_RES == CAM_RES_VGA)
    #define TAR_WIDTH		640
    #define TAR_HEIGHT		480
    #define SNR_WIDTH		640
    #define SNR_HEIGHT		480
    #define DIP_WIDTH		640
    #define DIP_HEIGHT		480
#else//user define
    #define TAR_WIDTH		1280
    #define TAR_HEIGHT		720
    #define SNR_WIDTH		1280
    #define SNR_HEIGHT		720
    #define DIP_WIDTH		1280
    #define DIP_HEIGHT		720
#endif

#define FPS				30
#define H264_TAR_BITRATE	7200
#define H264_GOP_LENGTH		10
#else
#if (USBD_UVC_CAM_RES == CAM_RES_720P)
    #define TAR_WIDTH		1280
    #define TAR_HEIGHT		720
    #define SNR_WIDTH		1280
    #define SNR_HEIGHT		720
    #define DIP_WIDTH		1280
    #define DIP_HEIGHT		720
#elif(USBD_UVC_CAM_RES == CAM_RES_VGA)
    #define TAR_WIDTH		640
    #define TAR_HEIGHT		480
    #define SNR_WIDTH		640
    #define SNR_HEIGHT		480
    #define DIP_WIDTH		640
    #define DIP_HEIGHT		480
#else//user define
    #define TAR_WIDTH		1280
    #define TAR_HEIGHT		720
    #define SNR_WIDTH		1280
    #define SNR_HEIGHT		720
    #define DIP_WIDTH		1280
    #define DIP_HEIGHT		720
#endif
#endif

/*==========================================================================================================================*/
extern osMessageQId uvc_frame_q;
extern osMessageQId UVCTaskQ;
extern uac_aud_dec_t usb_uac_aud_ctlblk;
#if 1//UVC_H264 == 1
osMessageQId uvc_send_ack = NULL;
#endif
INT32U uvc_start = 0;
INT32U uvc_send_done = 0;
INT32U send_video = 0;
INT32U send_audio = 0;
#if UVC_H264 == 1
INT32U uvc_first_send = 0;
VidEncFrame_t		*uvc_vid_enc_frm;
#endif
/*==========================================================================================================================*/
void _usbd_ep5_send_done_cbk(void)
{
	INT32U msg = USBD_SEND_VIDEO_DONE_EVENT;
    osStatus status;

	/* Set EP5 frame end flag for UVC header */
	drv_l1_usbd_set_frame_end_ep5();

	status = osMessagePut(UVCTaskQ, (INT32U)&msg, osWaitForever);
	if(status != osOK)
	{
		DBG_PRINT("EP5 done cbk send event to UVCTaskQ failed\r\n");
	}
}

void _usbd_ep7_send_done_cbk(void)
{
	INT32U msg = USBD_SEND_AUDIO_DONE_EVENT;
	osStatus status;

	status = osMessagePut(UVCTaskQ, (INT32U)&msg, osWaitForever);
	if(status != osOK)
	{
		DBG_PRINT("EP7 done cbk send event to UVCTaskQ failed\r\n");
	}
}

void _usbd_get_set_interface(INT32U video, INT32U audio)
{
    INT32U msg = USBD_NO_EVENT;
    osStatus status;

    /* Check Video */
    if(send_video != video)
    {
        if(video)
            msg = USBD_START_VIDEO_EVENT;
        else
            msg = USBD_STOP_VIDEO_EVENT;

        send_video = video;

        if(msg != USBD_NO_EVENT)
        {
            status = osMessagePut(UVCTaskQ, (INT32U)&msg, osWaitForever);
            if(status != osOK)
            {
                DBG_PRINT("Send event to UVCTaskQ failed\r\n");
            }
        }
    }

    /* Check Audio */
    if(send_audio != audio)
    {
        if(audio)
            msg = MSG_AUDIO_MIC_START;
        else
            msg = MSG_AUDIO_MIC_STOP;

        send_audio = audio;
        if(msg != USBD_NO_EVENT)
        {
            status = osMessagePut(usb_uac_aud_ctlblk.uac_enc_aud_q, (INT32U)&msg, osWaitForever);
            if(status != osOK)
            {
                DBG_PRINT("Send event to audpara->uac_enc_aud_q failed\r\n");
            }
        }
    }
#if 0
    if(msg != USBD_NO_EVENT)
    {
    	status = osMessagePut(UVCTaskQ, (INT32U)&msg, osWaitForever);
		if(status != osOK)
		{
			DBG_PRINT("Send event to UVCTaskQ failed\r\n");
		}
	}
#endif
}
static INT32U Display_Callback(INT16U w, INT16U h, INT32U addr)
{
    R_TFT_FBI_ADDR = addr;
}

static INT32S uvc_video_init(void)
{
	char  path[64];
	INT8U OperationMode;
	VIDEO_ARGUMENT arg;

    video_encode_register_display_callback(Display_Callback);

	if(uvc_frame_q == 0) {

		osMessageQDef_t uvc_f_q = {AVI_ENCODE_DISPALY_BUFFER_NO, sizeof(INT32U), 0};

		uvc_frame_q = osMessageCreate(&uvc_f_q, NULL);
		if(uvc_frame_q == 0) {
			return STATUS_FAIL;
		}
	}
#if UVC_H264 == 1
	if(uvc_send_ack == 0)
	{
		osMessageQDef_t uvc_send_a = {1, sizeof(INT32U), 0};

		uvc_send_ack = osMessageCreate(&uvc_send_a, NULL);
		if(uvc_send_ack == 0) {
			return STATUS_FAIL;
		}
	}
#endif
/*
    memset((void*)&arg, 0, sizeof(VIDEO_ARGUMENT));
	video_encode_entrance();

	arg.bScaler = 1;
	arg.TargetWidth = TAR_WIDTH;
	arg.TargetHeight = TAR_HEIGHT;
	arg.SensorWidth	= SNR_WIDTH;
	arg.SensorHeight = SNR_HEIGHT;
	arg.DisplayWidth = DIP_WIDTH;
	arg.DisplayHeight = DIP_HEIGHT;
#if UVC_H264 == 1
	arg.VidFrameRate = FPS;
#endif
	R_FUNPOS1 &= ~((0x3<<8)|(0x3<<3));
	video_encode_preview_start(arg);
*/
	return STATUS_OK;
}

void uvc_video_start(void){
    VIDEO_ARGUMENT arg;

    memset((void*)&arg, 0, sizeof(VIDEO_ARGUMENT));
    video_encode_entrance();
	arg.bScaler = 1;
	arg.TargetWidth = TAR_WIDTH;
	arg.TargetHeight = TAR_HEIGHT;
	arg.SensorWidth	= SNR_WIDTH;
	arg.SensorHeight = SNR_HEIGHT;
	arg.DisplayWidth = DIP_WIDTH;
	arg.DisplayHeight = DIP_HEIGHT;
#if UVC_H264 == 1
	arg.VidFrameRate = FPS;
#endif
	R_FUNPOS1 &= ~((0x3<<8)|(0x3<<3));
	video_encode_preview_start(arg);
}

INT32U uvc_post_cur_frame(osMessageQId event, INT32U frame_addr)
{
    INT32U  msg;
	osStatus status=STATUS_OK;
    if(uvc_start){
        status = osMessagePut(event, (uint32_t)&frame_addr, osWaitForever);
        if(status != osOK)
        {
            DBG_PRINT("send uvc current frame error!\r\n");
        }

        msg = USBD_SEND_VIDEO_EVENT;
        status = osMessagePut(UVCTaskQ, (INT32U)&msg, osWaitForever);
        if(status != osOK)
        {
            DBG_PRINT("send uvc msg error!\r\n");
        }
    }
	return status;
}

INT32U uvc_get_cur_frame(osMessageQId event)
{
	INT32U frame_addr;
	osEvent result;
    if(uvc_start){
        result = osMessageGet(event, osWaitForever);

        frame_addr = result.value.v;

        if((result.status != osEventMessage) || (frame_addr == 0)) {
            return 0;
        }
    }
	return frame_addr;
}

VidEncFrame_t *uvc_get_cur_enc_frame(osMessageQId event)
{
	//INT32U frame_addr;
	VidEncFrame_t *vid_enc_frame_info;
	osEvent result;
    if(uvc_start){
        result = osMessageGet(event, osWaitForever);
        vid_enc_frame_info = (VidEncFrame_t *)result.value.v;

        if((result.status != osEventMessage) || (vid_enc_frame_info == 0)) {
            return 0;
        }

    }
	//return frame_addr;
	return vid_enc_frame_info;
}

void UVCTask(void const *param)
{
#if UVC_H264 == 1
	INT32U  msg;
	osEvent result;
	INT32U addr = 0;
	INT32U vid_enc_size = 0;
	static INT32U change = 0;
	INT32S err;
	INT32S ret=0;
	MEDIA_SOURCE   src;
	INT32U ack_msg;
	err = uvc_video_init();

	while(1)
	{
		result = osMessageGet(UVCTaskQ, osWaitForever);
		msg = result.value.v;
		if(result.status != osEventMessage)
		{
			continue;
		}

		switch(msg)
		{
			case USBD_START_VIDEO_EVENT:
			DBG_PRINT("UVC H.264 start\r\n");
			drv_l1_pscaler_output_format_set(PSCALER_A, PIPELINE_SCALER_OUTPUT_FORMAT_VYUY);
			OSQFlush(uvc_frame_q);
			OSQFlush(uvc_send_ack);

            ack_msg = 0;//ack ok
            osMessagePut(uvc_send_ack, (INT32U)&ack_msg, osWaitForever);

			src.Format.VideoFormat = H_264;
			video_encode_set_h264_rc(H264_TAR_BITRATE, H264_GOP_LENGTH);

			if (STATUS_OK != video_encode_start(src, 0))
				DBG_PRINT("video encode start fail\r\n");

			uvc_start = 1;
			uvc_send_done = 1;
			uvc_first_send=1;
			break;

            case USBD_SEND_VIDEO_EVENT:
                if(uvc_start==1)
                {
					uvc_first_send = 0;
					addr = 0;
					vid_enc_size = 0;
					uvc_vid_enc_frm = uvc_get_cur_enc_frame(uvc_frame_q);
					addr = uvc_vid_enc_frm->ready_frame;
					vid_enc_size = uvc_vid_enc_frm->encode_size;
					//DBG_PRINT("addr=%x\r\n",addr);
					if(vid_enc_size)
					{
						drv_l1_usbd_frame_iso_ep5_in((void *)addr, vid_enc_size, 1);
						uvc_send_done = 0;
					}
				}

                break;

			case USBD_SEND_VIDEO_DONE_EVENT:
				if(uvc_first_send == 0){
					ack_msg = 0;//ack ok
					osMessagePut(uvc_send_ack, (INT32U)&ack_msg, osWaitForever);
					uvc_send_done = 1;
				}
				break;

			case USBD_STOP_VIDEO_EVENT:

				DBG_PRINT("UVC H.264 end\r\n");
				ack_msg = 0;//ack ok
				osMessagePut(uvc_send_ack, (INT32U)&ack_msg, osWaitForever);
				if(video_encode_status() == VIDEO_CODEC_PROCESSING) {
					video_encode_stop();
				}
				uvc_start = 0;
				uvc_send_done = 1;
				break;

			default:
				break;
		}
	}
#else
	INT32U  msg;
	osEvent result;
    INT32U addr;
	static INT32U change = 0;
	INT32S err;

	//adc_key_scan_init(); //init key scan
    err = uvc_video_init();

	while(1)
	{
		result = osMessageGet(UVCTaskQ, osWaitForever);
		msg = result.value.v;
		if(result.status != osEventMessage)
		{
			continue;
		}

		switch(msg)
		{
			case USBD_START_VIDEO_EVENT:
				DBG_PRINT("YUV start\r\n");
                drv_l1_pscaler_output_format_set(PSCALER_A, PIPELINE_SCALER_OUTPUT_FORMAT_VYUY);
				OSQFlush(uvc_frame_q);
    			uvc_start = 1;
				break;

            case USBD_SEND_VIDEO_EVENT:
                addr = uvc_get_cur_frame(uvc_frame_q);
				drv_l1_usbd_frame_iso_ep5_in((void *)addr, UVC_YUV_FRAME_SIZE, 1);
				uvc_send_done = 0;
                break;

			case USBD_SEND_VIDEO_DONE_EVENT:
                uvc_send_done = 1;
				break;

			case USBD_STOP_VIDEO_EVENT:
                DBG_PRINT("YUV end\r\n");
                uvc_start = 0;
                uvc_send_done = 0;
				break;

			default:
				break;
		}
	}

#endif
}

void uvc_video_stop(void)
{
    if(uvc_start != 0){
        if(video_encode_status() == VIDEO_CODEC_PROCESSING) {
            video_encode_stop();
        }
        uvc_start = 0;
        uvc_send_done = 1;
	}
    video_encode_preview_stop();
    video_encode_exit();
   	send_video = 0;
	send_audio = 0;
}
