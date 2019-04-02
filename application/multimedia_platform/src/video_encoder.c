#include "video_encoder.h"
#include "avi_encoder_app.h"
#include "drv_l2_sensor.h"
extern osMessageQId uvc_frame_q;
#if (defined APP_VIDEO_ENCODER_EN) && (APP_VIDEO_ENCODER_EN == 1)
/**
 * @brief   video encode entrance
 * @param   none
 * @return 	none
 */

#if VIDEO_TIMESTAMP
volatile INT32U vid_global_tick = 0;

void video_timer_update(void)
{
	vid_global_tick++;
}
#endif

void video_encode_entrance(void)
{
	INT32S nRet;

#if VIDEO_TIMESTAMP
	timer_freq_setup(TIMER_A, 1000, 0, video_timer_update);
#endif

    avi_encode_init();
    nRet = avi_encode_state_task_create(AVI_ENC_PRIORITY);
    if(nRet < 0) {
    	DEBUG_MSG("avi_encode_state_task_create fail !!!");
    }

/*    nRet = scaler_task_create(SCALER_PRIORITY);
    if(nRet < 0) {
    	DEBUG_MSG("scaler_task_create fail !!!");
    }

    nRet = video_encode_task_create(JPEG_ENC_PRIORITY);
    if(nRet < 0) {
    	DEBUG_MSG("video_encode_task_create fail !!!");
    }

    nRet = avi_adc_record_task_create(AUD_ENC_PRIORITY);
    if(nRet < 0) {
    	DEBUG_MSG("avi_adc_record_task_create fail !!!");
    }*/
    DEBUG_MSG("avi encode all task create success!!!\r\n");
}

/**
 * @brief   video encode exit
 * @param   none
 * @return 	none
 */
void video_encode_exit(void)
{
	INT32S nRet;

	if(video_encode_status() == VIDEO_CODEC_PROCESSING) {
   		video_encode_stop();
   	}
   	//video_encode_preview_stop();

    nRet = avi_encode_state_task_del();
	if(nRet < 0) {
		DEBUG_MSG("avi_encode_state_task_del fail !!!");
	}

	nRet = scaler_task_del();
	if(nRet < 0) {
		DEBUG_MSG("scaler_task_del fail !!!");
	}

	nRet = video_encode_task_del();
	if(nRet < 0) {
		DEBUG_MSG("video_encode_task_del fail !!!");
	}

	nRet = avi_adc_record_task_del();
	if(nRet < 0) {
		DEBUG_MSG("avi_adc_record_task_del fail !!!");
	}
	DEBUG_MSG("avi encode all task delete success!!!\r\n");

#if VIDEO_TIMESTAMP
	timer_stop(TIMER_A);

	vid_global_tick = 0;
#endif
}

/**
 * @brief   packer start
 * @param   arg[in]: video argument
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_packer_start(MEDIA_SOURCE src,VIDEO_ARGUMENT arg)
{
    INT32S nRet;

    if(avi_packer_memory_alloc() < 0) {
		avi_packer_memory_free();
		DEBUG_MSG("avi packer memory alloc fail!!!\r\n");
		return (STATUS_FAIL);
	}

    pAviEncAudPara->audio_format = AVI_ENCODE_AUDIO_FORMAT;
	pAviEncAudPara->channel_no = 1;
    pAviEncAudPara->audio_sample_rate = arg.AudSampleRate;
    pAviEncVidPara->dwScale = arg.bScaler;
    pAviEncVidPara->dwRate = arg.VidFrameRate;

    pAviEncVidPara->encode_width = arg.TargetWidth;
    pAviEncVidPara->encode_height = arg.TargetHeight;

    /* set video codec*/
	if(src.Format.VideoFormat == MJPEG) {
    	pAviEncVidPara->video_format = C_MJPG_FORMAT;
	} else if(src.Format.VideoFormat == H_264) {
    	pAviEncVidPara->video_format = C_H264_FORMAT;
	} else {
		return RESOURCE_WRITE_ERROR;
	}

	/* set avi packer workmem */
	avi_encode_set_curworkmem((void *)pAviEncPacker0);

	/* set source type */
    if(src.type == SOURCE_TYPE_FS) {
    	if(src.type_ID.FileHandle < 0) {
        	return RESOURCE_NO_FOUND_ERROR;
    	}

    	pAviEncPara->source_type = SOURCE_TYPE_FS;
    	nRet = avi_encode_set_file_handle_and_caculate_free_size(pAviEncPara->AviPackerCur, src.type_ID.FileHandle);
    } else if(src.type == SOURCE_TYPE_USER_DEFINE) {
    	pAviEncPara->source_type = SOURCE_TYPE_USER_DEFINE;
    	nRet = avi_encode_set_file_handle_and_caculate_free_size(pAviEncPara->AviPackerCur, 0x7FFF);
    } else {
        return RESOURCE_WRITE_ERROR;
  	}

  	if(nRet < 0) {
    	return RESOURCE_WRITE_ERROR;
    }

    /* start avi packer */
    nRet = avi_enc_packer_start(pAviEncPara->AviPackerCur);
    if(nRet < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }

    return START_OK;
}

/**
 * @brief   preview stop
 * @param   none
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_packer_stop(void)
{
    INT32S nRet;

    /* stop avi packer */
    nRet = avi_enc_packer_stop(pAviEncPara->AviPackerCur);

    if(nRet < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }
    avi_packer_memory_free();
    return START_OK;
}

/**
 * @brief   preview stop
 * @param   none
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_one_frame(INT8U fourcc,INT8U key_flag,INT32U frame_addr,INT32U frame_size)
{
    INT32S nRet;

	nRet = avi_encode_one_frame(fourcc,key_flag,frame_addr,frame_size);
    if(nRet < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }
	return START_OK;
}

/**
 * @brief   video_encode_capture_disable_difference_size
 * @param   none
 * @return 	none
 */
void video_encode_capture_disable_difference_size(void)
{
    if(pAviEncVidPara->encode_different_flag != 0)
    {
        pAviEncVidPara->encode_different_flag = 0;
        pAviEncVidPara->jpeg_encode_in_format = 0;
        pAviEncVidPara->jpeg_use_addr0_flag = 0;
    }
}

/**
 * @brief   video_encode_capture_enable_difference_size
 * @param   none
 * @return 	none
 */
void video_encode_capture_enable_difference_size(void)
{
    if((pAviEncVidPara->jpeg_encode_width != pAviEncVidPara->encode_width)
    || (pAviEncVidPara->jpeg_encode_height != pAviEncVidPara->encode_height))
    {
        pAviEncVidPara->encode_different_flag = 1;

        if((pAviEncVidPara->jpeg_encode_width > pAviEncVidPara->encode_width)
        || (pAviEncVidPara->jpeg_encode_height > pAviEncVidPara->encode_height))
        {
            pAviEncVidPara->jpeg_encode_in_format = 1;
            pAviEncVidPara->jpeg_use_addr0_flag = 1;
        }
        else
        {
            pAviEncVidPara->jpeg_encode_in_format = 0;
            pAviEncVidPara->jpeg_use_addr0_flag = 0;
        }
    }
}

/**
 * @brief   preview start
 * @param   arg[in]: video argument
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_preview_start(VIDEO_ARGUMENT arg)
{
    INT32S nRet;

    pAviEncAudPara->audio_format = AVI_ENCODE_AUDIO_FORMAT;
#if MIC_INPUT_SRC == C_ADC_LINE_IN || MIC_INPUT_SRC == C_BUILDIN_MIC_IN
	pAviEncAudPara->channel_no = 1;
#elif MIC_INPUT_SRC == C_LINE_IN_LR
	pAviEncAudPara->channel_no = 2;
#endif
    pAviEncAudPara->audio_sample_rate = arg.AudSampleRate;
	//pAviEncVidPara->video_format = AVI_ENCODE_VIDEO_FORMAT;
    pAviEncVidPara->dwScale = arg.bScaler;
    pAviEncVidPara->dwRate = arg.VidFrameRate;

    avi_encode_set_display_format(arg.OutputFormat);

    pAviEncVidPara->sensor_capture_width = arg.SensorWidth;
    pAviEncVidPara->sensor_capture_height = arg.SensorHeight;
    pAviEncVidPara->encode_width = arg.TargetWidth;
    pAviEncVidPara->encode_height = arg.TargetHeight;
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    pAviEncVidPara->display_width = arg.DisplayHeight;
    pAviEncVidPara->display_height = arg.DisplayWidth;
#else
    pAviEncVidPara->display_width = arg.DisplayWidth;
    pAviEncVidPara->display_height = arg.DisplayHeight;
#endif
    pAviEncVidPara->encode_different_flag = arg.bUseEncodeDiffSize;
    pAviEncVidPara->jpeg_encode_width = arg.JPEGTargetWidth;
    pAviEncVidPara->jpeg_encode_height = arg.JPEGTargetHeight;
    if(pAviEncVidPara->encode_different_flag)
    {
        if((pAviEncVidPara->jpeg_encode_width > pAviEncVidPara->encode_width)
        || (pAviEncVidPara->jpeg_encode_height > pAviEncVidPara->encode_height))
        {
            pAviEncVidPara->jpeg_encode_in_format = 1;
            pAviEncVidPara->jpeg_use_addr0_flag = 1;
        }
        else
        {
            pAviEncVidPara->jpeg_encode_in_format = 0;
            pAviEncVidPara->jpeg_use_addr0_flag = 0;
        }
    }
    else
    {
        pAviEncVidPara->jpeg_encode_in_format = 0;
        pAviEncVidPara->jpeg_use_addr0_flag = 0;
    }

    if(arg.DisplayBufferWidth == 0) {
    	arg.DisplayBufferWidth = arg.DisplayWidth;
    } else if(arg.DisplayWidth > arg.DisplayBufferWidth) {
    	arg.DisplayWidth = arg.DisplayBufferWidth;
    }

    if(arg.DisplayBufferHeight == 0) {
    	arg.DisplayBufferHeight = arg.DisplayHeight;
    } else if(arg.DisplayHeight > arg.DisplayBufferHeight) {
    	arg.DisplayHeight = arg.DisplayBufferHeight;
    }
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    pAviEncVidPara->display_buffer_width = arg.DisplayBufferHeight;
    pAviEncVidPara->display_buffer_height = arg.DisplayBufferWidth;
#else
    pAviEncVidPara->display_buffer_width = arg.DisplayBufferWidth;
    pAviEncVidPara->display_buffer_height = arg.DisplayBufferHeight;
#endif


    nRet = scaler_task_create(SCALER_PRIORITY);
    if(nRet < 0) {
    	DEBUG_MSG("scaler_task_create fail !!!");
    }

    nRet = video_encode_task_create(JPEG_ENC_PRIORITY);
    if(nRet < 0) {
    	DEBUG_MSG("video_encode_task_create fail !!!");
    }

    nRet = avi_adc_record_task_create(AUD_ENC_PRIORITY);
    if(nRet < 0) {
    	DEBUG_MSG("avi_adc_record_task_create fail !!!");
    }

	nRet = TH32x32_task_create(TH32x32_TASK_PRIORITY);
    if(nRet < 0)	DEBUG_MSG("TH32x32_task_create fail !!!");
	 else  DBG_PRINT("TH32x32_task_create success !!! \r\n");
	 
#if 1
	nRet = TH32x32_SCALARUP_Task_create(TH32x32_SCALARUP_PRIORITY);
    if(nRet < 0) DBG_PRINT("TH32x32_SCALARUP_Task_create fail !!!");
    else  DBG_PRINT("TH32x32_SCALARUP_Task_create success !!! \r\n");
#endif

   	nRet = vid_enc_preview_start();
   	if(nRet < 0) {
   		return CODEC_START_STATUS_ERROR_MAX;
    }
    return START_OK;
}

/**
 * @brief   preview stop
 * @param   none
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_preview_stop(void)
{
    INT32S result;

    result = vid_enc_preview_stop();
    if(result < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }
    return START_OK;
}

CODEC_START_STATUS video_encode_stream_start(VIDEO_ARGUMENT arg)
{
    INT32S nRet;

    pAviEncVidPara->video_format = AVI_ENCODE_VIDEO_FORMAT;
    pAviEncVidPara->dwScale = arg.bScaler;
    pAviEncVidPara->dwRate = arg.VidFrameRate;

    avi_encode_set_display_format(arg.OutputFormat);
    //avi_encode_set_sensor_format(V4L2_PIX_FMT_YUYV);

    pAviEncVidPara->sensor_capture_width = arg.SensorWidth;
    pAviEncVidPara->sensor_capture_height = arg.SensorHeight;
    pAviEncVidPara->encode_width = arg.TargetWidth;
    pAviEncVidPara->encode_height = arg.TargetHeight;
    pAviEncVidPara->display_width = arg.DisplayWidth;
    pAviEncVidPara->display_height = arg.DisplayHeight;

    if(arg.DisplayBufferWidth == 0) {
    	arg.DisplayBufferWidth = arg.DisplayWidth;
    } else if(arg.DisplayWidth > arg.DisplayBufferWidth) {
    	arg.DisplayWidth = arg.DisplayBufferWidth;
    }

    if(arg.DisplayBufferHeight == 0) {
    	arg.DisplayBufferHeight = arg.DisplayHeight;
    } else if(arg.DisplayHeight > arg.DisplayBufferHeight) {
    	arg.DisplayHeight = arg.DisplayBufferHeight;
   	}

    pAviEncVidPara->display_buffer_width = arg.DisplayBufferWidth;
    pAviEncVidPara->display_buffer_height = arg.DisplayBufferHeight;
  //  pAviEncVidPara->quality_value = arg.JPEG_Q;
  //  avi_encode_set_display_scaler();

  	nRet = scaler_task_create(SCALER_PRIORITY);
  	if(nRet < 0) {
		  DEBUG_MSG("scaler_task_create fail !!!");
  	}

 	 nRet = video_encode_task_create(JPEG_ENC_PRIORITY);
  	if(nRet < 0) {
		  DEBUG_MSG("video_encode_task_create fail !!!");
  	}

  	nRet = avi_adc_record_task_create(AUD_ENC_PRIORITY);
  	if(nRet < 0) {
		  DEBUG_MSG("avi_adc_record_task_create fail !!!");
  	}


    nRet = video_stream_encode_start();
    if(nRet < 0) {
   		return CODEC_START_STATUS_ERROR_MAX;
	}

    return START_OK;
}

CODEC_START_STATUS video_encode_stream_stop(void)
{
    INT32S result;
 //m   result = usb_webcam_stop();
    result = video_stream_encode_stop();
    if(result < 0) {
   		return CODEC_START_STATUS_ERROR_MAX;
    }
    return START_OK;
}

/**
 * @brief   start video encode
 * @param   src[in]: video file source
 *          file_write: write to file or not
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_start(MEDIA_SOURCE src, INT32U file_write)
{
	INT32S nRet;

	/* set video codec*/
	if(src.Format.VideoFormat == MJPEG) {
    	pAviEncVidPara->video_format = C_MJPG_FORMAT;
	} else if(src.Format.VideoFormat == H_264) {
    	pAviEncVidPara->video_format = C_H264_FORMAT;
	} else {
		return RESOURCE_WRITE_ERROR;
	}

	/* set avi packer workmem */
	avi_encode_set_curworkmem((void *)pAviEncPacker0);

	/* set source type */
	if(file_write == 1){
		if(src.type == SOURCE_TYPE_FS) {
			if(src.type_ID.FileHandle < 0) {
				return RESOURCE_NO_FOUND_ERROR;
			}

			pAviEncPara->source_type = SOURCE_TYPE_FS;
			nRet = avi_encode_set_file_handle_and_caculate_free_size(pAviEncPara->AviPackerCur, src.type_ID.FileHandle);
		} else if(src.type == SOURCE_TYPE_USER_DEFINE) {
			pAviEncPara->source_type = SOURCE_TYPE_USER_DEFINE;
			nRet = avi_encode_set_file_handle_and_caculate_free_size(pAviEncPara->AviPackerCur, 0x7FFF);
		} else {
 			return RESOURCE_WRITE_ERROR;
		}

		if(nRet < 0) {
			return RESOURCE_WRITE_ERROR;
		}
	}

    /* start avi packer */
    if(!uvc_frame_q){
		nRet = avi_enc_packer_start(pAviEncPara->AviPackerCur);
		if(nRet < 0) {
			return CODEC_START_STATUS_ERROR_MAX;
		}
    }

	/* start avi encode */
   	nRet = avi_enc_start();
  	if(nRet < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }
    return START_OK;
}

/**
 * @brief   stop video encode
 * @param   none
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_stop(void)
{
    INT32S nRet, nRet1;

    if(avi_encode_get_status() & C_AVI_ENCODE_PAUSE) {
		nRet = avi_enc_resume();
	}
    /* stop avi encode */
    nRet = avi_enc_stop();

    /* stop avi packer */
    if(!uvc_frame_q){
		nRet1 = avi_enc_packer_stop(pAviEncPara->AviPackerCur);

		if(nRet < 0 || nRet1 < 0) {
			return CODEC_START_STATUS_ERROR_MAX;
		}
    }

    return START_OK;
}

/**
 * @brief   get video encode information
 * @param   info[out]: video information
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_Info(VIDEO_INFO *info)
{
	switch(pAviEncAudPara->audio_format)
	{
	case WAV:
		strcpy((void*)info->AudSubFormat, (void *)"pcm");
		info->AudBitRate = pAviEncAudPara->audio_sample_rate * 16;
		break;

	case MICROSOFT_ADPCM:
		strcpy((void*)info->AudSubFormat, (void *)"adpcm");
		info->AudBitRate = pAviEncAudPara->audio_sample_rate * 16 / 4;
		break;

	case IMA_ADPCM:
		strcpy((void*)info->AudSubFormat, (void *)"adpcm");
		info->AudBitRate = pAviEncAudPara->audio_sample_rate * 16 / 4;
		break;

	case ALAW:
		strcpy((void*)info->AudSubFormat, (void *)"alaw");
		info->AudBitRate = pAviEncAudPara->audio_sample_rate * 16 / 4;
		break;

	case MULAW:
		strcpy((void*)info->AudSubFormat, (void *)"mulaw");
		info->AudBitRate = pAviEncAudPara->audio_sample_rate * 16 / 4;
		break;

	default:
		while(1);
	}
	info->AudFormat = (AUDIO_FORMAT)pAviEncAudPara->audio_format;
	info->AudSampleRate = pAviEncAudPara->audio_sample_rate;
	info->AudChannel = pAviEncAudPara->channel_no;

	switch(pAviEncVidPara->video_format)
	{
	case C_MJPG_FORMAT:
		info->VidFormat = MJPEG;
		strcpy((void *)info->VidSubFormat, (void *)"jpg");
		break;

	case C_XVID_FORMAT:
		info->VidFormat = MPEG4;
		strcpy((void *)info->VidSubFormat, (void *)"mp4");
		break;
    case C_H264_FORMAT:
        info->VidFormat = H_264;
        strcpy((void *)info->VidSubFormat, (void *)"264");
		break;
	default:
		while(1);
	}

	info->VidFrameRate = pAviEncVidPara->dwRate;
	info->Width = pAviEncVidPara->encode_width;
	info->Height = pAviEncVidPara->encode_height;
	return	START_OK;
}

/**
 * @brief   get video encode status
 * @param   none
 * @return 	status, see CODEC_START_STATUS
 */
VIDEO_CODEC_STATUS video_encode_status(void)
{
	INT32U status;

	status = avi_encode_get_status();
	if(status & C_AVI_ENCODE_ERR) {
		return VIDEO_CODEC_STATUS_ERR;
    } else if(status & C_AVI_ENCODE_PAUSE) {
		return VIDEO_CODEC_PROCESS_PAUSE;
	} else if(status & C_AVI_ENCODE_START) {
		return VIDEO_CODEC_PROCESSING;
	}

	return VIDEO_CODEC_PROCESS_END;
}

/**
 * @brief   set video encode zoom factor
 * @param   zoom_ratio[in]: zoom factor
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_set_zoom_scaler(INT32U zoom_ratio)
{
	pAviEncVidPara->scaler_zoom_ratio = zoom_ratio;
	//DBG_PRINT("zoom = %d\r\n", (INT32U)pAviEncVidPara->scaler_zoom_ratio);
	return START_OK;
}

CODEC_START_STATUS video_encode_pause(void)
{
    INT32S nRet;

	nRet = avi_enc_pause();
    if(nRet < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }
	return START_OK;
}

CODEC_START_STATUS video_encode_resume(void)
{
    INT32S nRet;

    nRet = avi_enc_resume();
	if(nRet < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }
	return START_OK;
}

CODEC_START_STATUS video_encode_cut_lastframe(void)
{
    INT32S nRet;

    nRet = avi_enc_cut_lastframe();
	if(nRet < 0) {
    	return CODEC_START_STATUS_ERROR_MAX;
    }
	return START_OK;
}
/**
 * @brief   set jpeg encode quality
 * @param   quality_value[in]: 20, 25, 30, 40, 50, 70, 80, 85, 90, 93, 95, 98 and 100
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_set_jpeg_quality(INT8U quality_value)
{
	if(quality_value == pAviEncVidPara->quality_value) {
		return START_OK;
	}

	if(avi_encode_set_jpeg_quality(quality_value)) {
		return START_OK;
	}
	return CODEC_START_STATUS_ERROR_MAX;
}

/**
 * @brief   set h264 rate control parameter
 * @param   bitrate: number of bits per second.
 * @param   gop_len: number of pictures in a GOP. (first is I frame, rest are P frames)
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_set_h264_rc(INT32U bitrate, INT32U gop_len)
{
	pAviEncVidPara->h264_bitrate = bitrate;
	pAviEncVidPara->h264_gop_len = gop_len;

	return START_OK;
}

/**
 * @brief   capture a jpeg file, the resoultion same as video encode size
 * @param   src[in]: file source
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_capture_picture(MEDIA_SOURCE src)
{
#if 1
	INT16S disk_no;
	INT32S size;

	if(src.type == SOURCE_TYPE_FS) {
		if(src.type_ID.FileHandle < 0) {
			return CODEC_START_STATUS_ERROR_MAX;
		}

		pAviEncPara->source_type = SOURCE_TYPE_FS;
		pAviEncPara->memptr = 0;
		pAviEncVidPara->video_format = C_MJPG_FORMAT;
		pAviEncPara->AviPackerCur->file_handle = src.type_ID.FileHandle;
		if(pAviEncVidPara->encode_different_flag)
            size = pAviEncVidPara->jpeg_encode_width * pAviEncVidPara->jpeg_encode_height;
		else
            size = pAviEncVidPara->encode_width * pAviEncVidPara->encode_height;
		disk_no = GetDiskOfFile(src.type_ID.FileHandle);
		if(vfsFreeSpace(disk_no) < size) {
			fs_close(src.type_ID.FileHandle);
			return CODEC_START_STATUS_ERROR_MAX;
		}

		if(avi_enc_save_jpeg() < 0) {
			fs_close(src.type_ID.FileHandle);
			return CODEC_START_STATUS_ERROR_MAX;
		}
	} else if(src.type == SOURCE_TYPE_USER_DEFINE) {
		pAviEncPara->source_type = SOURCE_TYPE_USER_DEFINE;
		pAviEncPara->memptr = src.type_ID.memptr;
		if(avi_enc_save_jpeg() < 0) {
			return CODEC_START_STATUS_ERROR_MAX;
		}
	} else {
		return CODEC_START_STATUS_ERROR_MAX;
	}

	return START_OK;
#else
    return CODEC_START_STATUS_ERROR_MAX;
#endif
}

/**
 * @brief   capture a jpeg file, the resoultion can be set
 * @param   src[in]: file source
 * @param   quality[in]: jpeg quality
 * @param   width[in]: jpeg width
 * @param   height[in]: jpeg height
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_capture_size(MEDIA_SOURCE src, INT8U quality, INT16U width, INT16U height)
{
#if 0
	INT16S disk_no, fd;
	INT32S size;

	if(src.type == SOURCE_TYPE_FS) {
		if(src.type_ID.FileHandle < 0) {
			return CODEC_START_STATUS_ERROR_MAX;
		}

		fd = src.type_ID.FileHandle;
		size = width * height;
		disk_no = GetDiskOfFile(src.type_ID.FileHandle);
		if(vfsFreeSpace(disk_no) < size) {
			fs_close(src.type_ID.FileHandle);
			return CODEC_START_STATUS_ERROR_MAX;
		}

		if(avi_enc_capture(fd, quality, width, height) < 0) {
			return CODEC_START_STATUS_ERROR_MAX;
		}
	} else {
		return CODEC_START_STATUS_ERROR_MAX;
	}

	return START_OK;
#else
    return CODEC_START_STATUS_ERROR_MAX;
#endif
}

/**
 * @brief   video encode fast switch to next file.
 * @param   src[in]: file source
 * @return 	status, see CODEC_START_STATUS
 */
CODEC_START_STATUS video_encode_fast_switch_stop_and_start(MEDIA_SOURCE src)
{
#if AVI_ENCODE_FAST_SWITCH_EN == 1
	INT32S nRet;
	AviEncPacker_t *pNewAviEncPacker, *pOldAviEncPacker;

	/* set video codec*/
	if(src.Format.VideoFormat == MJPEG) {
    	pAviEncVidPara->video_format = C_MJPG_FORMAT;
	} else {
		goto AVI_PACKER_FAIL;
	}

	/* set avi packer workmem */
	if(pAviEncPara->AviPackerCur == pAviEncPacker0) {
    	pOldAviEncPacker = pAviEncPacker0;
    	pNewAviEncPacker = pAviEncPacker1;
    } else {
    	pOldAviEncPacker = pAviEncPacker1;
    	pNewAviEncPacker = pAviEncPacker0;
    }

	/* set source type */
	if(src.type == SOURCE_TYPE_FS) {
    	if(src.type_ID.FileHandle < 0) {
        	goto AVI_PACKER_FAIL;
    	}

    	pAviEncPara->source_type = SOURCE_TYPE_FS;
    	nRet = avi_encode_set_file_handle_and_caculate_free_size(pNewAviEncPacker, src.type_ID.FileHandle);
    } else if(src.type == SOURCE_TYPE_USER_DEFINE) {
    	pAviEncPara->source_type = SOURCE_TYPE_USER_DEFINE;
    	nRet = avi_encode_set_file_handle_and_caculate_free_size(pNewAviEncPacker, 0x7FFF);
    } else {
        goto AVI_PACKER_FAIL;
  	}

   	if(nRet < 0) {
    	goto AVI_PACKER_FAIL;
    }

    /* stop current avi encode */
  	nRet = avi_enc_stop();
	if(nRet < 0) {
		goto AVI_PACKER_FAIL;
	}

    /* change current avi packer priority */
    OSTaskChangePrio(pOldAviEncPacker->task_prio, AVI_PACKER_PRIORITY);

    /* start new avi packer */
    nRet = avi_enc_packer_start(pNewAviEncPacker);
    if(nRet < 0) {
    	goto AVI_PACKER_FAIL;
    }

	/* start new avi packer workmen */
	avi_encode_set_curworkmem((void *)pNewAviEncPacker);
   	nRet = avi_enc_start();
  	if(nRet < 0) {
  		goto AVI_PACKER_FAIL;
    }

    /* stop old avi packer */
    nRet = avi_enc_packer_stop(pOldAviEncPacker);
    if(nRet < 0) {
    	goto AVI_PACKER_FAIL;
    }

    return START_OK;

AVI_PACKER_FAIL:
	avi_enc_stop();
	avi_enc_packer_stop(pNewAviEncPacker);
	avi_enc_packer_stop(pOldAviEncPacker);
#endif
	return CODEC_START_STATUS_ERROR_MAX;
}

/**
 * @brief   video encode register display callback function.
 * @param   disp[in]: display function
 * @return 	none
 */
void video_encode_register_display_callback(INT32U (*disp)(INT16U, INT16U, INT32U))
{
	if(disp) {
		videnc_display = disp;
	}
}

/**
 * @brief   video encode register buffer post callback function with display_frame_q.
 * @param   disp[in]: display buffer post function
 * @return 	none
 */
void video_encode_buffer_post_callback(INT32U (*disp_post)(INT32U))
{
	if(disp_post) {
		videnc_buferr_post = disp_post;
	}
}

/**
 * @brief   video encode register user write callback function.
 * @param   put_data[in]: write function
 * @return 	none
 */
void video_encode_register_user_write_callback(INT32S (*put_data)(void* , unsigned long , long , const void *, int , int))
{
	if(put_data) {
		pfn_avi_encode_put_data = put_data;
	}
}

INT32S video_encode_get_recording_time()
{
	INT32S ret = 0;

	if (pAviEncPara)
		ret = pAviEncPara->tv / (pAviEncVidPara->dwRate * pAviEncAudPara->audio_sample_rate);

	return ret;
}

INT32S video_encode_get_dummy_frame_count()
{
	return (INT32S)pAviEncPara->dummy_cnt;
}
#endif//#if (defined APP_VIDEO_ENCODER_EN) && (APP_VIDEO_ENCDOER_EN == 1)
