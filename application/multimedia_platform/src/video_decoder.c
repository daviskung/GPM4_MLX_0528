#include "video_decoder.h"
#include "task_video_decoder.h"
#include "gplib.h"
#include "drv_l1_scaler.h"
#include "drv_l1_pscaler.h"

#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECODER_EN == 1)

/**
 * @brief   video decode entrance
 * @param   none
 * @return 	none
 */
void video_decode_entrance(void)
{
	if(vid_dec_entry() < 0) {
		while(1);
	}
}

/**
 * @brief   video decode exit
 * @param   none
 * @return 	none
 */
void video_decode_exit(void)
{
	vid_dec_stop();
	vid_dec_parser_stop();
	if(vid_dec_exit() < 0) {
		while(1);
	}
}

/**
 * @brief   audio dac volume ramp up
 * @param   none
 * @return 	none
 */
void avi_audio_decode_rampup(void)
{
/*	//check ramp up
	if((R_DAC_CHA_FIFO & 0x3000) == 0x3000) {
		return;
	}

	drv_l1_dac_init();
	drv_l1_dac_vref_set(1);//enable DAC
	while(R_DAC_PGA&0x0100);

	//enable cha chb
	R_DAC_CHA_CTRL |= 0x2000;
	R_DAC_CHB_CTRL |= 0x2000; */
}

/**
 * @brief   audio dac ramp down
 * @param   none
 * @return 	none
 */
void avi_audio_decode_rampdown(void)
{
//	drv_l1_dac_disable();
}

/**
 * @brief   none
 * @param   none
 * @return 	none
 */
INT32S	avi_audio_get_one_frame(avi_frame_info *audio_frame)
{
	return audio_decode_get_one_frame(audio_frame);
}

/**
 * @brief   none
 * @param   none
 * @return 	none
 */
INT32S	avi_audio_init(void)
{
	return audio_decode_task_init();
}

/**
 * @brief   get video information
 * @param   info[out]: information
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_Info(VIDEO_INFO *info)
{
	INT16U width, height;

	// audio
	switch(p_wave_info->wFormatTag)
	{
	case WAVE_FORMAT_PCM:
		info->AudFormat = WAV;
		gp_strcpy((INT8S*)info->AudSubFormat, (INT8S *)"pcm");
		break;

	case WAVE_FORMAT_MULAW:
	case WAVE_FORMAT_ALAW:
		info->AudFormat = WAV;
		gp_strcpy((INT8S*)info->AudSubFormat, (INT8S *)"adpcm");
		break;

	case WAVE_FORMAT_ADPCM:
		info->AudFormat = MICROSOFT_ADPCM;
		gp_strcpy((INT8S*)info->AudSubFormat, (INT8S *)"adpcm");
		break;

	case WAVE_FORMAT_DVI_ADPCM:
		info->AudFormat = IMA_ADPCM;
		gp_strcpy((INT8S*)info->AudSubFormat, (INT8S *)"adpcm");
		break;

	case WAVE_FORMAT_MPEGLAYER3:
		info->AudFormat = MP3;
		gp_strcpy((INT8S*)info->AudSubFormat, (INT8S *)"mp3");
		break;

	case WAVE_FORMAT_MPEG_ADTS_AAC:
		info->AudFormat = MP3;
		gp_strcpy((INT8S*)info->AudSubFormat, (INT8S *)"aac");
		break;

	default:
		info->AudFormat = AUD_AUTOFORMAT;
		gp_strcpy((INT8S*)info->AudSubFormat, (INT8S *)"none");
	}

	if(p_wave_info) {
		info->AudSampleRate = p_wave_info->nSamplesPerSec;
		info->AudBitRate = p_wave_info->nAvgBytesPerSec;
		info->AudChannel = p_wave_info->nChannels;
		info->AudFrameSize = p_wave_info->nBlockAlign;
	} else {
		info->AudSampleRate = 0;
		info->AudBitRate = 0;
		info->AudChannel = 0;
		info->AudFrameSize = 0;
	}

	// video
	switch(p_bitmap_info->biCompression)
	{
	case C_XVID_FORMAT:
	case C_M4S2_FORMAT:
		info->VidFormat = MPEG4;
		gp_strcpy((INT8S*)info->VidSubFormat, (INT8S *)"mp4");
		break;

	case C_MJPG_FORMAT:
		info->VidFormat = MJPEG;
		gp_strcpy((INT8S*)info->VidSubFormat, (INT8S *)"jpg");
		break;

    case C_H264_FORMAT:
        info->VidFormat = H_264;
        gp_strcpy((INT8S*)info->VidSubFormat, (INT8S *)"264");
        break;

	default:
		info->VidFormat = VID_AUTOFORMAT;
		gp_strcpy((INT8S*)info->VidSubFormat, (INT8S *)"non");
	}

	if(p_bitmap_info) {
		/*if(vid_dec_get_total_time()) {
			info->VidFrameRate = vid_dec_get_total_samples()/vid_dec_get_total_time();
		} else {
			info->VidFrameRate = vid_dec_get_total_samples();
		}*/
		info->VidFrameRate = vid_dec_get_frame_rate();

		vid_dec_get_size(&width, &height);
		info->Width = width;
		info->Height = height;
		info->TotalDuration = vid_dec_get_total_time();
	} else {
		info->VidFrameRate = 0;
		info->Width = 0;
		info->Height = 0;
		info->TotalDuration = 0;
	}
	return VIDEO_CODEC_PROCESSING;
}

/**
 * @brief   parser video file
 * @param   video_info[out]: information
 * @param   arg[in]: video file argument
 * @param   src[in]: video file source
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS	video_decode_paser_header(VIDEO_INFO *video_info, VIDEO_ARGUMENT arg, MEDIA_SOURCE src)
{
	INT32S nRet, video_type;
	struct sfn_info fd_info;

	if(video_decode_status() == VIDEO_CODEC_PROCESS_PAUSE) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	if(video_decode_status() == VIDEO_CODEC_PROCESSING) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	if((src.type != SOURCE_TYPE_FS) && (src.type != SOURCE_TYPE_FS_RESOURCE_IN_FILE)) {
		DEBUG_MSG("Only Support GP File System\r\n");
		return 	VIDEO_CODEC_RESOURCE_NO_FOUND;
	}

	/*if(src.Format.VideoFormat != MJPEG && src.Format.VideoFormat != MPEG4 && src.Format.VideoFormat != H_264) {
		DEBUG_MSG("Only Support MJPEG and H264\r\n");
		return	CHANNEL_ASSIGN_ERROR;
	}*/

	if(src.type_ID.FileHandle < 0) {
		DEBUG_MSG("File handle Error\r\n");
		return RESOURCE_READ_ERROR;
	}

	if(src.type == SOURCE_TYPE_FS_RESOURCE_IN_FILE) {
		video_type = FILE_TYPE_AVI;
		fd_info.f_size = src.type_ID.temp;
	} else {
		sfn_stat(src.type_ID.FileHandle, &fd_info);
		video_type = vid_dec_get_file_format((INT8S*)fd_info.f_extname);
	}

	if(video_type < 0) {
		fs_close(src.type_ID.FileHandle);
		return VIDEO_CODEC_STATUS_ERR;
	}

	nRet = vid_dec_parser_start(src.type_ID.FileHandle, video_type, fd_info.f_size);
	if(nRet < 0) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	video_decode_Info(video_info);
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   stop decode video file
 * @param   none
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_parser_stop(void)
{
	INT32S nTemp;

	nTemp = vid_dec_parser_stop();
	if((nTemp < 0)) {
		return VIDEO_CODEC_STATUS_ERR;
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   start decode video file
 * @param   arg[in]: video file argument
 * @param   src[in]: video file source
 * @return 	status, see VIDEO_CODEC_STATUS
 */
CODEC_START_STATUS video_decode_start(VIDEO_ARGUMENT arg, MEDIA_SOURCE src)
{
	INT16U width, height;
	INT32U video_output_format;

	//display size
	if(arg.DisplayWidth == 0 || arg.DisplayHeight == 0) {
		arg.DisplayWidth = arg.TargetWidth;
		arg.DisplayHeight = arg.TargetHeight;
	}

	if(arg.DisplayBufferWidth == 0 || arg.DisplayBufferHeight == 0) {
		arg.DisplayBufferWidth = arg.TargetWidth;
		arg.DisplayBufferHeight = arg.TargetHeight;
	}

	//format
	switch(arg.OutputFormat)
	{
	case IMAGE_OUTPUT_FORMAT_RGB565:
		video_output_format = C_SCALER_CTRL_OUT_RGB565;
		break;

	case IMAGE_OUTPUT_FORMAT_YUYV:
		video_output_format = C_SCALER_CTRL_OUT_YUYV;
		break;

	case IMAGE_OUTPUT_FORMAT_UYVY:
		video_output_format = C_SCALER_CTRL_OUT_UYVY;
		break;

	default:
		return CODEC_START_STATUS_ERROR_MAX;
	}

	//scaler
	vid_dec_get_size(&width, &height);
	switch(arg.bScaler)
	{
	case 0x01:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = SCALE_FULL_SCREEN;
		vid_dec_set_scaler(SCALE_FULL_SCREEN, video_output_format, arg.DisplayWidth, arg.DisplayHeight, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;

	case 0x02:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = SCALE_FIT_BUF;
		vid_dec_set_scaler(SCALE_FIT_BUF, video_output_format, arg.DisplayWidth, arg.DisplayHeight, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;

	case 0x03:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = NO_SCALE;
		vid_dec_set_scaler(NO_SCALE, video_output_format, width, height, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;

	case 0x04:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = NO_SCALE_AT_CENTER;
		vid_dec_set_scaler(NO_SCALE_AT_CENTER, video_output_format, width, height, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;

	default:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = SCALE_FULL_SCREEN;
		vid_dec_set_scaler(SCALE_FULL_SCREEN, video_output_format, arg.DisplayWidth, arg.DisplayHeight, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;
	}

	//deblock
	if((p_bitmap_info->biCompression != C_MJPG_FORMAT) && (arg.bScaler & 0x80)) {
		vid_dec_set_deblock_flag(TRUE);
	} else {
		vid_dec_set_deblock_flag(FALSE);
	}

	//user define buffer
	vid_dec_set_user_define_buffer(arg.bUseDefBuf, (INT32U)arg.AviDecodeBuf1, (INT32U)arg.AviDecodeBuf2);
	if(vid_dec_start() < 0) {
		video_decode_stop();
		return CODEC_START_STATUS_ERROR_MAX;
	}
	return START_OK;
}

/**
 * @brief   stop decode video file
 * @param   none
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_stop(void)
{
	INT32S nRet, nTemp;

	nRet = vid_dec_stop();
	nTemp = vid_dec_parser_stop();
	if((nRet < 0) || (nTemp < 0)) {
		return VIDEO_CODEC_STATUS_ERR;
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   pause decode video file
 * @param   none
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_pause(void)
{
	if(video_decode_status() != VIDEO_CODEC_PROCESSING) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	if(vid_dec_pause() < 0) {
		return VIDEO_CODEC_STATUS_ERR;
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   resume decode video file
 * @param   none
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_resume(void)
{
	if(video_decode_status() != VIDEO_CODEC_PROCESS_PAUSE) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	if(vid_dec_resume() < 0) {
		return VIDEO_CODEC_STATUS_ERR;
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   set audio dac volume
 * @param   none
 * @return 	status, see VIDEO_CODEC_STATUS
 */
void audio_decode_volume(INT8U volume)
{
	if(volume > 0x3F) {
		volume = 0x3F;
	}
	drv_l1_dac_pga_set(volume);
}

/**
 * @brief   set fast resume play enable
 * @param   none
 * @return 	none
 */
void video_decode_resume_play_enable(void)
{
	vid_dec_enable_resume_play();
}

/**
 * @brief   enable fast resume play store status
 * @param   none
 * @return 	none
 */
void video_decode_store_state_enable(void)
{
	vid_dec_store_play_state();
}

/**
 * @brief   get video decode status
 * @param   none
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_status(void)
{
	if(vid_dec_get_status() & C_VIDEO_DECODE_ERR) {
		return VIDEO_CODEC_PASER_HEADER_FAIL;
	} else if(vid_dec_get_status() & C_VIDEO_DECODE_PAUSE) {
		return VIDEO_CODEC_PROCESS_PAUSE;
	} else if(vid_dec_get_status() & C_VIDEO_DECODE_PLAY) {
		return VIDEO_CODEC_PROCESSING;
	} else if(vid_dec_get_status() & C_VIDEO_DECODE_PARSER) {
		return VIDEO_CODEC_PROCESSING;
	} else if(vid_dec_get_status() & C_VIDEO_DECODE_PARSER_NTH) {
		return VIDEO_CODEC_PROCESSING;
	} else {
		return VIDEO_CODEC_PROCESS_END;
	}
}

/**
 * @brief   set video play seek time
 * @param   SecTime[in]: seek time in second
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_set_play_time(INT32S SecTime)
{
	if(video_decode_status() == VIDEO_CODEC_PASER_HEADER_FAIL){
		return VIDEO_CODEC_STATUS_ERR;
	}

	if(video_decode_status() == VIDEO_CODEC_PROCESS_END) {
		return  VIDEO_CODEC_PROCESS_END;
	}

	if(SecTime < 0) {
		SecTime = 0;
	}

	if(SecTime >= vid_dec_get_total_time()) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	if(vid_dec_set_play_time(SecTime) < 0) {
		return VIDEO_CODEC_STATUS_ERR;
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   set video play speed
 * @param   speed[in]: play speed
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_set_play_speed(FP32 speed)
{
	if(video_decode_status() == VIDEO_CODEC_PROCESSING) {
		if(vid_dec_set_play_speed(speed * 0x10000) < 0) {
			return VIDEO_CODEC_STATUS_ERR;
		}
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   set video play reverse
 * @param   enable[in]: reverse play enable
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_set_reverse_play(INT32U enable)
{
	if(video_decode_status() == VIDEO_CODEC_PROCESSING) {
		if(vid_dec_set_reverse_play(enable) < 0) {
			return VIDEO_CODEC_STATUS_ERR;
		}
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   get video current play time
 * @param   none
 * @return 	time
 */
INT32U	video_decode_get_current_time(void)
{
	return vid_dec_get_current_time();
}

/**
 * @brief   get video current play frame number
 * @param   none
 * @return 	frame number
 */
INT32U	video_decode_get_current_number(void)
{
	return vid_dec_get_current_number();
}

/**
 * @brief   get video one frame buffer
 * @param   video_frame[in]: frame info
 * @return 	status, see VIDEO_CODEC_STATUS
 */
 INT32S	video_decode_get_one_frame(avi_frame_info *video_frame)
{
	return vid_dec_get_one_frame(video_frame);
}

/**
 * @brief   get video first frame buffer
 * @param   video_info[in]: video info
 * @param   arg[in]: video file argument
 * @param   src[in]: video file source
 * @return 	status, see VIDEO_CODEC_STATUS
 */
VIDEO_CODEC_STATUS video_decode_get_nth_video_frame(VIDEO_INFO *video_info, VIDEO_ARGUMENT arg, MEDIA_SOURCE src)
{
	INT16U width, height;
	INT32S nRet, nTemp;
	INT32U video_output_format;

	//check playing
	if(video_decode_status() == VIDEO_CODEC_PROCESS_PAUSE) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	if(video_decode_status() == VIDEO_CODEC_PROCESSING) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	//format
	switch(arg.OutputFormat)
	{
	case IMAGE_OUTPUT_FORMAT_RGB565:
		video_output_format = C_SCALER_CTRL_OUT_RGB565;
		break;

	case IMAGE_OUTPUT_FORMAT_YUYV:
		video_output_format = C_SCALER_CTRL_OUT_YUYV;
		break;

	case IMAGE_OUTPUT_FORMAT_UYVY:
		video_output_format = C_SCALER_CTRL_OUT_UYVY;
		break;

	default:
		return CODEC_START_STATUS_ERROR_MAX;
	}

	if(video_decode_paser_header(video_info, arg, src) != VIDEO_CODEC_STATUS_OK) {
		return VIDEO_CODEC_STATUS_ERR;
	}

	//scaler
	vid_dec_get_size(&width, &height);
	switch(arg.bScaler)
	{
	case 0x01:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = SCALE_FULL_SCREEN;
		vid_dec_set_scaler(SCALE_FULL_SCREEN, video_output_format, arg.DisplayWidth, arg.DisplayHeight, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;

	case 0x02:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = SCALE_FIT_BUF;
		vid_dec_set_scaler(SCALE_FIT_BUF, video_output_format, arg.DisplayWidth, arg.DisplayHeight, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;

	case 0x03:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = NO_SCALE;
		vid_dec_set_scaler(NO_SCALE, video_output_format, arg.DisplayWidth, arg.DisplayHeight, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;

	case 0x04:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = NO_SCALE_AT_CENTER;
		vid_dec_set_scaler(NO_SCALE_AT_CENTER, video_output_format, width, height, width, height);
		break;

	default:
		p_vid_dec_para->scaler_flag = p_vid_dec_para->user_scaler_flag = SCALE_FULL_SCREEN;
		vid_dec_set_scaler(SCALE_FULL_SCREEN, video_output_format, arg.DisplayWidth, arg.DisplayHeight, arg.DisplayBufferWidth, arg.DisplayBufferHeight);
		break;
	}

	//deblock
	vid_dec_set_deblock_flag(FALSE);
	//user define buffer
	vid_dec_set_user_define_buffer(arg.bUseDefBuf, (INT32U)arg.AviDecodeBuf1, (INT32U)arg.AviDecodeBuf2);
	nRet = vid_dec_nth_frame(1);
	nTemp = vid_dec_parser_stop();
	if(nRet < 0 || nTemp < 0) {
		return VIDEO_CODEC_STATUS_ERR;
	}
	return VIDEO_CODEC_STATUS_OK;
}

/**
 * @brief   video decode register display callback function.
 * @param   disp[in]: display function.
 * @return 	none
 */
void video_decode_register_display_callback(INT32U (*disp)(INT16U, INT16U, INT32U))
{
	if(disp) {
		viddec_display = disp;
	}
}

/**
 * @brief   video decode register decode done callback function.
 * @param   end[in]: decode done function.
 * @return 	none
 */
void video_decode_register_end_callback(void (*end)(void))
{
	if(end) {
		viddec_end = end;
	}
}

#endif//#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECDOER_EN == 1)
