#include "avi_encoder_state.h"
#include "drv_l1_csi.h"
//#include "drv_l1_cdsp.h"
#include "drv_l2_cdsp.h"
#include "drv_l2_sensor.h"
#include "drv_l2_display.h"
#include "drv_l1_pscaler.h"
#include "drv_l1_clock.h"
#include "jpeg_header.h"
#include "gp_avcodec.h"
#include "gp_mux.h"
#ifdef VFRAME_MANAGER
#include "gp_vfm.h"
#endif

#if (defined APP_VIDEO_ENCODER_EN) && (APP_VIDEO_ENCODER_EN == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define C_AVI_ENCODE_STATE_STACK_SIZE	4096
#define AVI_ENCODE_QUEUE_MAX_LEN		5
#define C_ACK_QUEUE_MAX				1
#define C_VIDEO_PACKER_STACK_SIZE	2048
#define VIDEO_PACKER_QUEUE_MAX_LEN		AVI_ENCODE_VIDEO_BUFFER_NO+AVI_ENCODE_PCM_BUFFER_NO

#define CAPTURE_STACK_SIZE				2048
#define CAPTURE_QUEUE_SIZE				1

#define CMSIS_MSGQ 	0
/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef enum
{
	MSG_AVI_START_SENSOR = 0x1000,
	MSG_AVI_STOP_SENSOR,
	MSG_AVI_START_ENCODE,
	MSG_AVI_STOP_ENCODE,
	MSG_AVI_RESUME_ENCODE,
	MSG_AVI_PAUSE_ENCODE,
	MSG_AVI_CAPTURE_PICTURE,
	MSG_AVI_STORAGE_FULL,
	MSG_AVI_STORAGE_ERR,
	MSG_AVI_ENCODE_STATE_EXIT,
	MSG_AVI_ONE_FRAME_ENCODE,
	MSG_AVI_START_USB_WEBCAM,
	MSG_AVI_STOP_USB_WEBCAM,
	MSG_AVI_START_VIDEO_STREAM,
	MSG_AVI_STOP_VIDEO_STREAM
} AVI_ENCODE_STATE_ENUM;

#if AVI_PACKER_LIB_EN == 0
typedef enum
{
	MSG_PACKER_VIDEO_WRITE = 0x2000,
	MSG_PACKER_AUDIO_WRITE,
	MSG_PACKER_EXIT
} VIDEO_PACKER_STATE_ENUM;
#endif
/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
static void avi_encode_state_task_entry(void const *para);
static void capture_task_entry(void const *parm);

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static drv_l2_sensor_ops_t *pSencor;
osMessageQId AVIEncodeApQ = NULL;
osMessageQId avi_encode_ack_m = NULL;
INT32S (*pfn_avi_encode_put_data)(void *workmem, unsigned long fourcc, long cbLen, const void *ptr, int nSamples, int ChunkFlag);
INT32S hAVIPacker;
struct gpMux_s* avipacker;
extern osMessageQId uvc_frame_q;
extern osMessageQId video_stream_q;
extern osMessageQId uvc_send_ack;
#if AVI_PACKER_LIB_EN == 0
#if CMSIS_MSGQ
osMessageQId VideoPackerQ = NULL;
#else
MSG_Q_ID VideoPackerQ = NULL;
#endif
osMessageQId video_packer_ack_m = NULL;
extern osMessageQId aud_enc_frameq;
#endif

#if AVI_PACKER_LIB_EN == 1
// callback function
static INT32S video_encode_frame_ready(void* workmem, unsigned long fourcc, long cbLen, const void *ptr, int nSamples, int ChunkFlag)
{
	if(fourcc == 0x63643030) { //"00dc", video frame ready

	} else if(fourcc == 0x62773130) { //"01wb", audio frame ready

	} else {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

static void video_encode_end(void* workmem)
{
	//add description to avi file
	//int AviPackerV3_AddInfoStr(const char *fourcc, const char *info_string);
	AviPackerV3_AddInfoStr(workmem, "ISRC", "Generplus");
	AviPackerV3_AddInfoStr(workmem, "IART", "Generplus");
	AviPackerV3_AddInfoStr(workmem, "ICOP", "Generplus");
	AviPackerV3_AddInfoStr(workmem, "ICRD", "2010-06-29");
}
#else
static void video_encode_end(INT32S hd)
{
	muxAviAddInfoStr(hd, "ISRC", "Generplus");
	muxAviAddInfoStr(hd, "IART", "Generplus");
	muxAviAddInfoStr(hd, "ICOP", "Generplus");
	muxAviAddInfoStr(hd, "ICRD", "2017-01-01");
}
#endif

#if AUDIO_SFX_HANDLE
INT32U video_encode_audio_sfx(INT16U *PCM_Buf, INT32U cbLen)
{
	return (INT32U)PCM_Buf;
}
#endif

static INT32S sensor_init(CHAR *SensorName)
{
	CHAR *p;
	INT32U i;

	// Enable Sensor Clock
	R_SYSTEM_CTRL |= (1 << 11);
	// Change Sensor control pin to IOD6~9
	R_FUNPOS1 |= (1<<21)|(1<<24);
	R_FUNPOS1 |= (1<<7);
	// sensor init
	drv_l2_sensor_init();

	// get sensor module
	if(SensorName == NULL) {
		pSencor = drv_l2_sensor_get_ops(0);
		DEBUG_MSG("SensorName = %s\r\n", pSencor->name);
	} else {
		for(i=0; i<3; i++) {
			pSencor = drv_l2_sensor_get_ops(i);
			DEBUG_MSG("SensorName = %s\r\n", pSencor->name);
			if(strcmp(SensorName, pSencor->name) == 0) {
				pSencor = drv_l2_sensor_get_ops(i);
				break;
			}
		}

		if(i == 3) {
			return STATUS_FAIL;
		}
	}

	// get csi or cdsp
	p = (CHAR *)strrchr((CHAR *)pSencor->name, 'c');
	if(p == 0) {
		return STATUS_FAIL;
	}

	if(strncmp((CHAR *)p, "csi", 3) == 0) {
		pAviEncPara->sensor_interface = CSI_INTERFACE;
	} else if(strncmp((CHAR *)p, "cdsp", 4) == 0) {
		pAviEncPara->sensor_interface = CDSP_INTERFACE;
	} else {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

static INT32S sensor_start(INT16U width, INT16U height, INT32U csi_frame1, INT32U csi_frame2)
{
	INT32U i, idx;
	drv_l2_sensor_info_t *pInfo;

	
	DBG_PRINT("width = 0x%x height = 0x%x\r\n", width,height);
	
	for(i=0; i<3; i++) {
		pInfo = pSencor->get_info(i);
		
	DBG_PRINT("target_w = 0x%x target_h = 0x%x\r\n", pInfo->target_w,pInfo->target_h);
		if(pInfo->target_w == width && pInfo->target_h == height) {
			idx = i;
			break;
		}
	}

	if(i == 3) {
		return STATUS_FAIL;
	}

	#if (PALM_DEMO_EN==1)||(PATTERN_DEMO_EN==1)||(WAFD_DEMO_EN==1)
	//computer vision demo uninit disable some clk, cause CSI clko no output, and not enter csi_eof
	drv_l1_clock_set_system_clk_en(CLK_EN1_PPU,ENABLE);
	drv_l1_clock_set_system_clk_en(CLK_EN1_DISPLAY_CSI,ENABLE);
    drv_l1_clock_set_system_clk_en(CLK_EN1_DISPAY_OUT,ENABLE);
    drv_l1_clock_set_system_clk_en(CLK_EN1_JPEG,ENABLE);
	#endif
	pSencor->init();
	pSencor->stream_start(idx, csi_frame1, csi_frame2);
	avi_encode_set_sensor_format(pInfo->output_format);
	drv_l1_pscaler_start(0);
	//drv_l1_pscaler_start(1);
	return STATUS_OK;
}

static INT32S sensor_stop(void)
{
#if (PALM_DEMO_EN == 1) || (WAFD_DEMO_EN == 1)
#else
	pscaler_exit_0 =  1;
	while(pscaler_exit_0)
        osDelay(1);
#endif
#if (PATTERN_DEMO_EN == 1)
    pscaler_exit_1 =  1;
	while(pscaler_exit_1)
        osDelay(1);
#endif
    //pscaler_exit_1 =  1;
	//while(pscaler_exit_1)
      //  osDelay(1);
	pSencor->stream_stop();
	pSencor->uninit();
	return STATUS_OK;
}

static void eof_handle_register(void)
{
	if(pAviEncPara->sensor_interface == CSI_INTERFACE) {
		drv_l2_sensor_set_path(1);
	#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
		drv_l1_register_csi_cbk(csi_eof_isr);
	#else
		drv_l1_register_csi_cbk(csi_fifo_isr);
	#endif
	}

	if(pAviEncPara->sensor_interface == CDSP_INTERFACE) {
		drv_l2_sensor_set_path(0);
	#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
		pAviEncPara->abBufFlag = 0;
		drv_l2_CdspIsrRegister(C_ISR_EOF, cdsp_eof_isr);
	#else

	#endif
	}
}

static void capture_handle_register(void)
{
	if(pAviEncPara->sensor_interface == CSI_INTERFACE) {
		drv_l1_register_csi_cbk(csi_capture_isr);
	}

	if(pAviEncPara->sensor_interface == CDSP_INTERFACE) {
		pAviEncPara->abBufFlag = 0;
		//drv_l2_CdspIsrRegister(C_ISR_EOF, cdsp_capture_isr);
	}
}

// api function
INT32U avi_enc_packer_start(AviEncPacker_t *pAviEncPacker)
{
#if AVI_PACKER_LIB_EN == 1
	INT32S nRet;
	INT32U bflag;

	if(pAviEncPacker == pAviEncPacker0) {
		bflag = C_AVI_ENCODE_PACKER0;
		pAviEncPacker->task_prio = AVI_PACKER0_PRIORITY;
	} else if(pAviEncPacker == pAviEncPacker1) {
		bflag = C_AVI_ENCODE_PACKER1;
		pAviEncPacker->task_prio = AVI_PACKER1_PRIORITY;
	} else {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
	if((avi_encode_get_status() & bflag) == 0) {
		switch(pAviEncPara->source_type)
		{
		case SOURCE_TYPE_FS:
			avi_encode_set_avi_header(pAviEncPacker);
			nRet = AviPackerV3_Open(pAviEncPacker->avi_workmem,
									pAviEncPacker->file_handle,
									pAviEncPacker->index_handle,
									pAviEncPacker->p_avi_vid_stream_header,
									pAviEncPacker->bitmap_info_cblen,
									pAviEncPacker->p_avi_bitmap_info,
									pAviEncPacker->p_avi_aud_stream_header,
									pAviEncPacker->wave_info_cblen,
									pAviEncPacker->p_avi_wave_info,
									pAviEncPacker->task_prio,
									pAviEncPacker->file_write_buffer,
									pAviEncPacker->file_buffer_size,
									pAviEncPacker->index_write_buffer,
									pAviEncPacker->index_buffer_size);
			AviPackerV3_SetErrHandler(pAviEncPacker->avi_workmem, avi_packer_err_handle);
			pfn_avi_encode_put_data = AviPackerV3_PutData;
			break;
		case SOURCE_TYPE_USER_DEFINE:
			pAviEncPacker->p_avi_wave_info = pAviEncPacker->avi_workmem;
			pfn_avi_encode_put_data = video_encode_frame_ready;
			break;
		}
		avi_encode_set_status(bflag);
		DEBUG_MSG("a.AviPackerOpen[0x%x] = 0x%x\r\n", bflag, nRet);
	} else {
		RETURN(STATUS_FAIL);
	}
Return:
	return nRet;
#elif NEW_VIDEO_PACKER_LIB != VIDEO_PACKER_NONE
    avi_encode_set_avi_header(pAviEncPacker);
#if NEW_VIDEO_PACKER_LIB == VIDEO_PACKER_MOV
    avipacker = &MOV_packer;
#else
    avipacker = &AVI_packer;
#endif

    hAVIPacker = avipacker->open(pAviEncPacker->file_handle);
    avipacker->set(hAVIPacker, MUX_MAX_SIZE, 0x40000000);
    if (pAviEncVidPara->video_format == C_MJPG_FORMAT)
        avipacker->set(hAVIPacker, MUX_VID_TYPE, VIDEO_TYPE_MJPEG);
    else if (pAviEncVidPara->video_format == C_H264_FORMAT)
        avipacker->set(hAVIPacker, MUX_VID_TYPE, VIDEO_TYPE_H264_BP);
    avipacker->set(hAVIPacker, MUX_HEIGHT, pAviEncVidPara->encode_height);
    avipacker->set(hAVIPacker, MUX_WIDTH, pAviEncVidPara->encode_width);
    avipacker->set(hAVIPacker, MUX_FRMRATE, pAviEncVidPara->dwRate);
    avipacker->set(hAVIPacker, MUX_AUD_TYPE, AUD_FORMAT_PCM);
    avipacker->set(hAVIPacker, MUX_AUD_CHANNEL, 1);
    avipacker->set(hAVIPacker, MUX_AUDSR, pAviEncAudPara->audio_sample_rate);
    avipacker->set(hAVIPacker, MUX_AUD_BIT, 16);
    avipacker->set(hAVIPacker, MUX_PATH_TEMP_FILE, (INT32U)pAviEncPacker->index_path);

#if AVI_PACKER_LIB_EN == 0
	if (0 > video_packer_task_create())
		return STATUS_FAIL;
#endif
	return STATUS_OK;
#else
    return STATUS_FAIL;
#endif
}

INT32S avi_encode_one_frame(INT8U fourcc,INT8U key_flag,INT32U frame_addr,INT32U frame_size)
{
	INT8U  err;
	INT32S nRet, msg;
	INT32U video_stream,encode_size;
    INT64S  dwPts = 0;
	gpMuxPkt_t pkt;

	nRet = STATUS_OK;
	if((key_flag != 0) && (frame_size == 0)) {
		DEBUG_MSG("encode_size = 0x%x\r\n", frame_size);
		return STATUS_FAIL;
	}

	video_stream = frame_size + 8 + 2*16;
	if(!avi_encode_disk_size_is_enough(video_stream))
	{
		DEBUG_MSG("avi encode storage full!!!\r\n");
		avi_enc_packer_stop(pAviEncPara->AviPackerCur);
		return STATUS_FAIL;
	}

	if(key_flag != 0)
	{
		key_flag = AVIIF_KEYFRAME;
	}

	if(fourcc)
	{
#if AVI_PACKER_LIB_EN == 1
		nRet = pfn_avi_encode_put_data(	pAviEncPara->AviPackerCur->avi_workmem,
										*(long*)"00dc",
										frame_size,
										(void *)frame_addr,
										1,
										key_flag);
#elif NEW_VIDEO_PACKER_LIB != VIDEO_PACKER_NONE
            dwPts = 1000*pAviEncVidPara->dwScale/(pAviEncVidPara->dwRate);
            pkt.data = (INT8U*)frame_addr;
            pkt.size = frame_size;
            pkt.frameType = key_flag;
            pkt.pts = dwPts;//1000/pAviEncVidPara->dwRate;
            //DBG_PRINT("pts=%d\r\n", pkt.pts);
            nRet = avipacker->pack(hAVIPacker, &pkt, GP_ES_TYPE_VIDEO);
            dwPts = 0;
#endif
		if(nRet >= 0)
		{
			DEBUG_MSG(".");
			return STATUS_OK;
		}
		else
		{
			avi_encode_disk_size_is_enough(-video_stream);
			DEBUG_MSG("VidPutData = %x, size = %d !!!\r\n", nRet-0x80000000, encode_size);
		}

		if(!avi_encode_disk_size_is_enough(8+2*16))
		{
			DEBUG_MSG("avi encode storage full!!!\r\n");
			avi_enc_packer_stop(pAviEncPara->AviPackerCur);
			return STATUS_FAIL;
		}
#if AVI_PACKER_LIB_EN == 1
		nRet = pfn_avi_encode_put_data(	pAviEncPara->AviPackerCur->avi_workmem,
										*(long*)"00dc",
										0,
										(void *)NULL,
										1,
										0x00);
			dwPts += pAviEncPara->delta_Tv;
#else
            dwPts = 1000*pAviEncVidPara->dwScale/(pAviEncVidPara->dwRate);;
            pkt.data = (INT8U*)NULL;
            pkt.size = 0;
            pkt.frameType = key_flag;
            pkt.pts = dwPts;
            nRet = avipacker->pack(hAVIPacker, &pkt, GP_ES_TYPE_VIDEO);
            dwPts = 0;
#endif
		if(nRet >= 0) {
			DEBUG_MSG("N");
			return STATUS_OK;
		}
		else
		{
			avi_encode_disk_size_is_enough(-(8+2*16));
			DEBUG_MSG("VidPutDataNULL = %x!!!\r\n", nRet-0x80000000);
			return STATUS_FAIL;
		}
	}
	else
	{
#if AVI_PACKER_LIB_EN == 1
		nRet = pfn_avi_encode_put_data(	pAviEncPara->AviPackerCur->avi_workmem,
										*(long*)"01wb",
										frame_size,
										(INT16U*)frame_addr,
										frame_size/pAviEncPara->AviPackerCur->p_avi_wave_info->nBlockAlign,
										AVIIF_KEYFRAME);
#elif NEW_VIDEO_PACKER_LIB != VIDEO_PACKER_NONE
        pkt.data = (INT8U*)frame_addr;
        pkt.size = frame_size;
        pkt.frameType = 0;
        pkt.pts = (encode_size >> 1)*1000/pAviEncAudPara->audio_sample_rate;
        nRet = avipacker->pack(hAVIPacker, &pkt, GP_ES_TYPE_AUDIO);
#endif
		if(nRet >= 0)
		{
			DEBUG_MSG("A");
			return STATUS_OK;
		}
		else
		{
			avi_encode_disk_size_is_enough(-video_stream);
			DEBUG_MSG("AudPutData = %x, size = %d!!!\r\n", nRet-0x80000000, frame_size<<1);
			return STATUS_FAIL;
		}
	}
Return:
	return nRet;
}
INT32U avi_enc_packer_stop(AviEncPacker_t *pAviEncPacker)
{
#if AVI_PACKER_LIB_EN == 1
	INT32S nRet=0;
	INT32U bflag;

	if(pAviEncPacker == pAviEncPacker0) {
		bflag = C_AVI_ENCODE_PACKER0;
	} else if(pAviEncPacker == pAviEncPacker1) {
		bflag = C_AVI_ENCODE_PACKER1;
	} else {
		RETURN(STATUS_FAIL);
	}

	if(avi_encode_get_status() & bflag) {
		 avi_encode_clear_status(bflag);
		if(avi_encode_get_status() & C_AVI_ENCODE_ERR) {
			avi_encode_clear_status(C_AVI_ENCODE_ERR);
			switch(pAviEncPara->source_type)
			{
			case SOURCE_TYPE_FS:
				nRet = AviPackerV3_Close(pAviEncPacker->avi_workmem);
				avi_encode_fail_handle_close_file(pAviEncPacker);
				break;
			case SOURCE_TYPE_USER_DEFINE:
				nRet = STATUS_OK;
				break;
			}
		} else {
			switch(pAviEncPara->source_type)
			{
			case SOURCE_TYPE_FS:
				video_encode_end(pAviEncPacker->avi_workmem);
				nRet = AviPackerV3_Close(pAviEncPacker->avi_workmem);
				avi_encode_close_file(pAviEncPacker);
				break;
			case SOURCE_TYPE_USER_DEFINE:
				nRet = STATUS_OK;
				break;
			}
		}

		if(nRet < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("c.AviPackerClose[0x%x] = 0x%x\r\n", bflag, nRet);
	}
	nRet = STATUS_OK;
Return:
	return nRet;
#elif NEW_VIDEO_PACKER_LIB != VIDEO_PACKER_NONE
	video_packer_task_del();
	video_encode_end(hAVIPacker);
    if(avi_encode_get_status() & C_AVI_ENCODE_ERR) {
		avi_encode_clear_status(C_AVI_ENCODE_ERR);
        avipacker->close(hAVIPacker);
        avi_encode_fail_handle_close_file(pAviEncPacker);
    }
    else{
        avipacker->close(hAVIPacker);
        avi_encode_close_file(pAviEncPacker);
    }
	//return STATUS_FAIL;
	return STATUS_OK;
#else
    return STATUS_FAIL;
#endif
}

INT32S vid_enc_preview_start(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
	avi_encode_set_display_scaler();

	//alloc memory
	if(avi_encode_memory_alloc() < 0) {
		avi_encode_memory_free();
		DEBUG_MSG("avi memory alloc fail!!!\r\n");
		RETURN(STATUS_FAIL);
	}

	if(avi_packer_memory_alloc() < 0) {
		avi_packer_memory_free();
		DEBUG_MSG("avi packer memory alloc fail!!!\r\n");
		RETURN(STATUS_FAIL);
	}

	// start scaler
	if((avi_encode_get_status()&C_AVI_ENCODE_SCALER) == 0) {
		if(scaler_task_start() < 0) RETURN(STATUS_FAIL);
		avi_encode_set_status(C_AVI_ENCODE_SCALER);
		DEBUG_MSG("a.scaler start\r\n");
	}

#if AVI_ENCODE_VIDEO_ENCODE_EN == 1
	// start video
	if((avi_encode_get_status()&C_AVI_ENCODE_VIDEO) == 0) {
		if(video_encode_task_start() < 0) RETURN(STATUS_FAIL);
		avi_encode_set_status(C_AVI_ENCODE_VIDEO);
		DEBUG_MSG("b.video start\r\n");
	}
#endif

#if AVI_ENCODE_PRE_ENCODE_EN == 1
	if((avi_encode_get_status() & C_AVI_ENCODE_PRE_START) == 0) {
		video_encode_task_post_q(0);
		avi_encode_set_status(C_AVI_ENCODE_PRE_START);
	}
#endif

	// start sensor
	if((avi_encode_get_status()&C_AVI_ENCODE_SENSOR) == 0) {
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_START_SENSOR, avi_encode_ack_m, 5000);
		avi_encode_set_status(C_AVI_ENCODE_SENSOR);
		DEBUG_MSG("c.sensor start\r\n");
	}

	

#if (AVI_AUDIO_ENCODE_EN == 1) && (AVI_ENCODE_PRE_ENCODE_EN == 1)
	// start audio
	if(pAviEncAudPara->audio_format && ((avi_encode_get_status()&C_AVI_ENCODE_AUDIO) == 0)) {
		if(avi_audio_record_start() < 0) RETURN(STATUS_FAIL);
		avi_encode_set_status(C_AVI_ENCODE_AUDIO);
		DEBUG_MSG("d.audio start\r\n");
	}
#endif
Return:
	return nRet;
}

INT32S vid_enc_preview_stop(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
#if AVI_AUDIO_ENCODE_EN == 1
	// stop audio
	if(avi_encode_get_status()&C_AVI_ENCODE_AUDIO)
	{
		avi_encode_clear_status(C_AVI_ENCODE_AUDIO);
		nRet = avi_audio_record_stop();
		if(nRet < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("a.audio stop\r\n");
	}
#endif

	// stop sensor
	if(avi_encode_get_status()&C_AVI_ENCODE_SENSOR)
	{
		avi_encode_clear_status(C_AVI_ENCODE_SENSOR);
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_STOP_SENSOR, avi_encode_ack_m, 5000);
		DEBUG_MSG("b.sensor stop\r\n");
	}
	// stop video
	if(avi_encode_get_status()&C_AVI_ENCODE_VIDEO)
	{
		avi_encode_clear_status(C_AVI_ENCODE_VIDEO);
		if(video_encode_task_stop() < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("c.video stop\r\n");
	}
	// stop scaler
	if(avi_encode_get_status()&C_AVI_ENCODE_SCALER)
	{
		avi_encode_clear_status(C_AVI_ENCODE_SCALER);
		if(scaler_task_stop() < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("d.scaler stop\r\n");
	}
Return:
	//free memory
	avi_encode_memory_free();
	avi_packer_memory_free();
	DEBUG_MSG("e.free memory\r\n");
	return nRet;
}

INT32S avi_enc_start(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
#if AVI_AUDIO_ENCODE_EN == 1
	// start audio
	if(pAviEncAudPara->audio_format && ((avi_encode_get_status()&C_AVI_ENCODE_AUDIO) == 0)) {
		if(avi_audio_record_start() < 0) {
			RETURN(STATUS_FAIL);
		}
		avi_encode_set_status(C_AVI_ENCODE_AUDIO);
		DEBUG_MSG("b.audio start\r\n");
	}
#endif

#if (AVI_AUDIO_ENCODE_EN == 1) && (AVI_ENCODE_PRE_ENCODE_EN == 1)
	// restart audio
	if(pAviEncAudPara->audio_format && (avi_encode_get_status()&C_AVI_ENCODE_AUDIO)) {
		if(avi_audio_record_restart() < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("b.audio restart\r\n");
	}
#endif
	// start avi encode
	if((avi_encode_get_status()&C_AVI_ENCODE_START) == 0) {
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_START_ENCODE, avi_encode_ack_m, 5000);
		avi_encode_set_status(C_AVI_ENCODE_START);
		avi_encode_set_status(C_AVI_ENCODE_PRE_START);
		DEBUG_MSG("c.encode start\r\n");
	}
Return:
	return nRet;
}

INT32S avi_enc_stop(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
#if (AVI_AUDIO_ENCODE_EN == 1) && (AVI_ENCODE_PRE_ENCODE_EN == 0)
	// stop audio
	if(avi_encode_get_status()&C_AVI_ENCODE_AUDIO) {
		avi_encode_clear_status(C_AVI_ENCODE_AUDIO);
		if(avi_audio_record_stop() < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("a.audio stop\r\n");
	}
#endif

	// stop avi encode
	//empty vid_enc_frame_q and post to video_frame_q.
	if(uvc_frame_q)
		video_encode_task_empty_q();

	if(avi_encode_get_status()&C_AVI_ENCODE_START) {
		avi_encode_clear_status(C_AVI_ENCODE_START);
	#if AVI_ENCODE_PRE_ENCODE_EN == 0
		avi_encode_clear_status(C_AVI_ENCODE_PRE_START);
	#endif
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_STOP_ENCODE, avi_encode_ack_m, 5000);
		DEBUG_MSG("b.encode stop\r\n");
	}
	//empty vid_enc_frame_q and post to video_frame_q.
	if(!uvc_frame_q)
	video_encode_task_empty_q();
	//video_encode_task_reset();
Return:
	return nRet;
}

INT32S avi_enc_pause(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
#if AUDIO_RESTART_EN == 1
#if (AVI_AUDIO_ENCODE_EN == 1) && (AVI_ENCODE_PRE_ENCODE_EN == 0)
	// stop audio
	if(avi_encode_get_status()&C_AVI_ENCODE_AUDIO) {
		avi_encode_clear_status(C_AVI_ENCODE_AUDIO);
		if(avi_audio_record_stop() < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("a.audio stop\r\n");
	}
#endif
#endif
	if(avi_encode_get_status()&C_AVI_ENCODE_START) {
    	POST_MESSAGE(AVIEncodeApQ, MSG_AVI_PAUSE_ENCODE, avi_encode_ack_m, 5000);
    	avi_encode_set_status(C_AVI_ENCODE_PAUSE);
		DEBUG_MSG("encode pause\r\n");
    }
Return:
	return nRet;
}

INT32S avi_enc_resume(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
#if AUDIO_RESTART_EN == 1
#if AVI_AUDIO_ENCODE_EN == 1
	// start audio
	if(pAviEncAudPara->audio_format && ((avi_encode_get_status()&C_AVI_ENCODE_AUDIO) == 0)) {
		if(avi_audio_record_start() < 0) {
			RETURN(STATUS_FAIL);
		}
		avi_encode_set_status(C_AVI_ENCODE_AUDIO);
		DEBUG_MSG("b.audio start\r\n");
	}
#endif

#if (AVI_AUDIO_ENCODE_EN == 1) && (AVI_ENCODE_PRE_ENCODE_EN == 1)
	// restart audio
	if(pAviEncAudPara->audio_format && (avi_encode_get_status()&C_AVI_ENCODE_AUDIO)) {
		if(avi_audio_record_restart() < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("b.audio restart\r\n");
	}
#endif
#endif

	if(avi_encode_get_status()&(C_AVI_ENCODE_START|C_AVI_ENCODE_PAUSE)) {
    	POST_MESSAGE(AVIEncodeApQ, MSG_AVI_RESUME_ENCODE, avi_encode_ack_m, 5000);
    	avi_encode_clear_status(C_AVI_ENCODE_PAUSE);
		DEBUG_MSG("encode resume\r\n");
    }
Return:
	return nRet;
}

INT32S avi_enc_save_jpeg(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
	if((avi_encode_get_status() & C_AVI_ENCODE_PRE_START) == 0) {
		video_encode_task_post_q(0);
		avi_encode_set_status(C_AVI_ENCODE_PRE_START);
	}
	avi_encode_set_status(C_AVI_ENCODE_JPEG);
	POST_MESSAGE(AVIEncodeApQ, MSG_AVI_CAPTURE_PICTURE, avi_encode_ack_m, 5000);
Return:
	//empty vid_enc_frame_q and post to video_frame_q.
	video_encode_task_empty_q();
#if AVI_ENCODE_PRE_ENCODE_EN == 0
	avi_encode_clear_status(C_AVI_ENCODE_PRE_START);
	avi_encode_clear_status(C_AVI_ENCODE_JPEG);
#endif
	return nRet;
}

INT32S avi_enc_cut_lastframe(void)
{
	return avi_audio_record_cut_lastframe();
}

INT32S video_stream_encode_start(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	avi_encode_set_display_scaler();

	//alloc memory
	if(avi_encode_memory_alloc() < 0) {
		avi_encode_memory_free();
		DBG_PRINT("avi memory alloc fail!!!\r\n");
		RETURN(STATUS_FAIL);
	}

	// start scaler
	if((avi_encode_get_status()&C_AVI_ENCODE_SCALER) == 0) {
		if(scaler_task_start() < 0) {
			RETURN(STATUS_FAIL);
		}
		avi_encode_set_status(C_AVI_ENCODE_SCALER);
		DBG_PRINT("a.scaler start\r\n");
	}

	// start video
	if((avi_encode_get_status()&C_AVI_ENCODE_VIDEO) == 0) {
		if(video_encode_task_start() < 0) RETURN(STATUS_FAIL);
		avi_encode_set_status(C_AVI_ENCODE_VIDEO);
		DBG_PRINT("b.video start\r\n");
	}

	// start sensor
	if((avi_encode_get_status()&C_AVI_ENCODE_SENSOR) == 0) {
//		avi_encode_eof_isr_register(csi_eof_isr);
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_START_SENSOR, avi_encode_ack_m, 5000);
		avi_encode_set_status(C_AVI_ENCODE_SENSOR);
		DBG_PRINT("c.sensor start\r\n");
	}

	//video stream
	if((avi_encode_get_status()&C_AVI_ENCODE_STREAM) == 0) {
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_START_VIDEO_STREAM, avi_encode_ack_m, 5000);
		avi_encode_set_status(C_AVI_ENCODE_STREAM);
		DBG_PRINT("d.video stream\r\n");
	}

Return:
	return nRet;
}

INT32S video_stream_encode_stop(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;

	// stop sensor
	if(avi_encode_get_status()&C_AVI_ENCODE_SENSOR) {
		avi_encode_clear_status(C_AVI_ENCODE_SENSOR);
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_STOP_SENSOR, avi_encode_ack_m, 5000);
		DBG_PRINT("b.sensor stop\r\n");
	}

	// stop video
	if(avi_encode_get_status()&C_AVI_ENCODE_VIDEO) {
		avi_encode_clear_status(C_AVI_ENCODE_VIDEO);
		if(video_encode_task_stop() < 0) {
			RETURN(STATUS_FAIL);
		}
		DBG_PRINT("c.video stop\r\n");
	}

	// stop scaler
	if(avi_encode_get_status()&C_AVI_ENCODE_SCALER) {
		avi_encode_clear_status(C_AVI_ENCODE_SCALER);
		if(scaler_task_stop() < 0) {
			RETURN(STATUS_FAIL);
		}
		DBG_PRINT("d.scaler stop\r\n");
	}

	//video stream
	if(avi_encode_get_status()&C_AVI_ENCODE_STREAM) {
		avi_encode_clear_status(C_AVI_ENCODE_STREAM);
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_STOP_VIDEO_STREAM, avi_encode_ack_m, 5000);
		DBG_PRINT("e.video stream stop\r\n");
	}

Return:
	//free memory
	avi_encode_memory_free();
	DBG_PRINT("f.free memory\r\n");
	return nRet;
}

INT32S stream_enc_start(void)
{
	INT32S nRet;

	nRet = STATUS_OK;



	// start avi encode
	//if(avi_encode_get_status() == C_AVI_ENCODE_STREAM) {
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_START_ENCODE, avi_encode_ack_m, 5000);
	//	avi_encode_set_status(C_AVI_ENCODE_START);
	//	avi_encode_set_status(C_AVI_ENCODE_PRE_START);
		DEBUG_MSG("c.encode start\r\n");
	//}
Return:
	return nRet;
}

INT32S stream_enc_stop(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
#if (AVI_AUDIO_ENCODE_EN == 1) && (AVI_ENCODE_PRE_ENCODE_EN == 0)
	// stop audio
	if(avi_encode_get_status()&C_AVI_ENCODE_AUDIO) {
		avi_encode_clear_status(C_AVI_ENCODE_AUDIO);
		if(avi_audio_record_stop() < 0) {
			RETURN(STATUS_FAIL);
		}
		DEBUG_MSG("a.audio stop\r\n");
	}
#endif

	// stop avi encode
	if(avi_encode_get_status()&C_AVI_ENCODE_START) {
		avi_encode_clear_status(C_AVI_ENCODE_START);
	#if AVI_ENCODE_PRE_ENCODE_EN == 0
		avi_encode_clear_status(C_AVI_ENCODE_PRE_START);
	#endif
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_STOP_ENCODE, avi_encode_ack_m, 5000);
		DEBUG_MSG("b.encode stop\r\n");
	}
	//empty vid_enc_frame_q and post to video_frame_q.
	video_encode_task_empty_q();
	//video_encode_task_reset();
Return:
	return nRet;
}


INT32S avi_enc_capture(INT16S fd, INT8U quality, INT16U width, INT16U height)
{
	INT32S nRet;
	osThreadId id;
	osEvent result;
	osMessageQDef_t sensor_q = {CAPTURE_QUEUE_SIZE, sizeof(INT32U), 0};
	osMessageQDef_t ack_q = {CAPTURE_QUEUE_SIZE, sizeof(INT32U), 0};
	osThreadDef_t capture_task = {"capture_task", capture_task_entry, osPriorityNormal, 1, CAPTURE_STACK_SIZE};

	pCap = (catpure_t *)gp_malloc_align(sizeof(catpure_t), 4);
	if(pCap == 0) {
		return STATUS_FAIL;
	}

	pAviEncPara->skip_cnt = 3;
	memset((void *)pCap, 0x00, sizeof(catpure_t));
	pCap->quality = quality;
	pCap->width = width;
	pCap->height = height;

	nRet = width * height * 3;
	pCap->csi_buf = (INT32U)gp_malloc_align(nRet, 32);
	if(pCap->csi_buf == 0) {
		RETURN(STATUS_FAIL);
	}

	pCap->jpeg_buf = pCap->csi_buf + (width * height * 2);
	pCap->Sensor_m = osMessageCreate(&sensor_q, NULL);
	if(pCap->Sensor_m == 0) {
		RETURN(STATUS_FAIL);
	}

	pCap->Ack_m = osMessageCreate(&ack_q, NULL);
	if(pCap->Ack_m == 0) {
		RETURN(STATUS_FAIL);
	}

	// create task
	id = osThreadCreate(&capture_task, (void *)pCap);
    if(id == 0) {
        RETURN(STATUS_FAIL);
    }

	// wait ack
	result = osMessageGet(pCap->Ack_m, 5000);
	if((result.status != osEventMessage) || (result.value.v == ACK_FAIL)) {
		DEBUG_MSG("Ack Fail\r\n");
		RETURN(STATUS_FAIL);
	}

	// save file
	if(fd >= 0) {
		nRet = save_jpeg_file(fd, pCap->jpeg_buf, pCap->jpeg_len);
		if(nRet < 0) {
			RETURN(STATUS_FAIL);
		}
	}

Return:
	if(pCap->csi_buf) {
		gp_free((void *)pCap->csi_buf);
	}

	if(pCap->Sensor_m) {
		//OSMboxDel(pCap->Sensor_m, OS_DEL_ALWAYS, &err);
		vQueueDelete(pCap->Sensor_m);
	}

	if(pCap->Ack_m) {
		//OSMboxDel(pCap->Ack_m, OS_DEL_ALWAYS, &err);
		vQueueDelete(pCap->Ack_m);
	}

	if(pCap) {
		gp_free((void *)pCap);
		pCap = 0;
	}

	return nRet;
}

// avi encode state function
INT32S avi_encode_state_task_create(INT8U pori)
{
	INT32S nRet;
	osThreadId id;
	osThreadDef_t state_task = {"avi_state_task", avi_encode_state_task_entry, osPriorityNormal, 1, C_AVI_ENCODE_STATE_STACK_SIZE};

	if(AVIEncodeApQ == 0) {
        osMessageQDef_t App_q = {AVI_ENCODE_QUEUE_MAX_LEN, sizeof(INT32U), 0};

		AVIEncodeApQ = osMessageCreate(&App_q, NULL);
		if(AVIEncodeApQ == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(avi_encode_ack_m == 0) {
        osMessageQDef_t Ack_q = {C_ACK_QUEUE_MAX, sizeof(INT32U), 0};

		avi_encode_ack_m = osMessageCreate(&Ack_q, NULL);
		if(avi_encode_ack_m == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	id = osThreadCreate(&state_task, (void *) NULL);
	if(id == 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S avi_encode_state_task_del(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(AVIEncodeApQ, MSG_AVI_ENCODE_STATE_EXIT, avi_encode_ack_m, 5000);
Return:
	OSQFlush(AVIEncodeApQ);
	OSQFlush(avi_encode_ack_m);
	vQueueDelete(AVIEncodeApQ);
	vQueueDelete(avi_encode_ack_m);
	AVIEncodeApQ = NULL;
	avi_encode_ack_m = NULL;
	return nRet;
}

// encode one frame
INT32S avi_enc_one_frame(void)
{
	INT32U msg;
	osStatus status;

	msg = MSG_AVI_ONE_FRAME_ENCODE;
	status = osMessagePut(AVIEncodeApQ, (INT32U)&msg, osWaitForever);
	if(status != osOK) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

// error handle
INT32S avi_enc_storage_full(void)
{
	INT32U msg;
	osStatus status;

	DEBUG_MSG("avi encode storage full!!!\r\n");
	msg = MSG_AVI_STORAGE_FULL;
	status = osMessagePut(AVIEncodeApQ, (INT32U)&msg, osWaitForever);
	if(status != osOK) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S avi_packer_err_handle(INT32S ErrCode)
{
#if 1
	/* close avi packer task automatic */
	INT32U msg;
	osStatus status;

	DEBUG_MSG("AviPacker-ErrID = 0x%x!!!\r\n", ErrCode);
	avi_encode_set_status(C_AVI_ENCODE_ERR);

	msg = MSG_AVI_STORAGE_ERR;
	status = osMessagePut(AVIEncodeApQ, (INT32U)&msg, osWaitForever);
	if(status != osOK) {
		return 0;
	}

	return 1;
#else
	/* cloase avipack task by user */
	DEBUG_MSG("AviPacker-ErrID = 0x%x!!!\r\n", ErrCode);
	avi_encode_set_status(C_AVI_ENCODE_ERR);
	return 0;
#endif
}

// avi encode state task function
static void avi_encode_state_task_entry(void const *para)
{
	INT8U   success_flag, key_flag;
	INT32S  nRet;
	INT32U  msg_id, video_frame, video_stream, encode_size;
	INT64S  dwtemp, dwPts = 0;
	VidEncFrame_t *pVideo;
	osEvent result;
	osThreadId id;
#if NEW_VIDEO_PACKER_LIB
	gpMuxPkt_t pkt;
#endif
	INT32U NFno = 0,Fno = 0, fps, video_format;
	extern INT32U encode_time;

    DEBUG_MSG("<<%s>>\r\n", __func__);

	while(1)
	{
		result = osMessageGet(AVIEncodeApQ, osWaitForever);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (msg_id == 0)) {
			continue;
		}

		switch(msg_id)
		{
		case MSG_AVI_START_SENSOR:	//sensor
			DEBUG_MSG("[MSG_AVI_START_SENSOR]\r\n");
			// init sensor
			sensor_init(NULL);
        #if (PALM_DEMO_EN == 0) && (WAFD_DEMO_EN == 0)
			scaler_disp_init();
        #endif
			//sensor_init("ov_7670_csi");
			//sensor_init("ov_7670_cdsp");
			OSQFlush(cmos_frame_q);
		#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
			if(pAviEncPara->sensor_interface == CSI_INTERFACE) {
				video_frame = pAviEncVidPara->csi_frame_addr[0];
				video_stream = 0;
				for(nRet = 1; nRet<AVI_ENCODE_CSI_BUFFER_NO; nRet++) {
					osMessagePut(cmos_frame_q, (INT32U)&pAviEncVidPara->csi_frame_addr[nRet], osWaitForever);
				}
			} else {
			#if C_DMA_CH == 0
				video_frame = pAviEncVidPara->csi_frame_addr[0];
				video_stream = pAviEncVidPara->csi_frame_addr[1];
				for(nRet = 2; nRet<AVI_ENCODE_CSI_BUFFER_NO; nRet++) {
					osMessagePut(cmos_frame_q, (INT32U)&pAviEncVidPara->csi_frame_addr[nRet], osWaitForever);
				}
			#elif C_DMA_CH == 1
				video_frame = pAviEncVidPara->csi_frame_addr[0];
				video_stream = 0;
				for(nRet = 1; nRet<AVI_ENCODE_CSI_BUFFER_NO; nRet++) {
					osMessagePut(cmos_frame_q, (INT32U)&pAviEncVidPara->csi_frame_addr[nRet], osWaitForever);
				}
			#elif C_DMA_CH == 2
				video_frame = 0;
				video_stream = pAviEncVidPara->csi_frame_addr[0];
				for(nRet = 1; nRet<AVI_ENCODE_CSI_BUFFER_NO; nRet++) {
					osMessagePut(cmos_frame_q, (INT32U)&pAviEncVidPara->csi_frame_addr[nRet], osWaitForever);
				}
			#endif
			}
		#elif VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
		/*
			video_frame = avi_encode_get_csi_frame();
			video_stream = avi_encode_get_csi_frame();
			for(nRet = 0; nRet<pAviEncPara->vid_post_cnt; nRet++) {
				pAviEncVidPara->csi_fifo_addr[nRet] = avi_encode_get_csi_frame();
			}
			*/
		#endif
			nRet = sensor_start(pAviEncVidPara->sensor_capture_width,
								pAviEncVidPara->sensor_capture_height,
								0x50000000, 0x50000000);
			
			if(nRet >= 0) {
				eof_handle_register();
				msg_id = ACK_OK;
				DBG_PRINT("sensor_start nRet OK \r\n");
			} else {
				msg_id = ACK_FAIL;
				
				DBG_PRINT("sensor_start nRet fail\r\n");
			}

			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
		//	#if(WIFI_DEMO == 1)
		//	{
//			avi_enc_start();
		//	}
	//		#endif
			
			break;

		case MSG_AVI_STOP_SENSOR:
			DEBUG_MSG("[MSG_AVI_STOP_SENSOR]\r\n");
			nRet = sensor_stop();
			OSQFlush(cmos_frame_q);
			if(nRet >= 0) {
				msg_id = ACK_OK;
			} else {
				msg_id = ACK_FAIL;
			}

			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			break;

		case MSG_AVI_START_VIDEO_STREAM:
			  nRet = video_encode_task_post_q(1);
			  if(nRet >= 0) {
			  	  msg_id = ACK_OK;
				  osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			  } else {
			  	  msg_id = ACK_FAIL;
				  osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			  }
			  break;

		case MSG_AVI_STOP_VIDEO_STREAM:
			  OSQFlush(video_stream_q);
				  	  msg_id = ACK_OK;
				  osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);

             break;


		case MSG_AVI_START_ENCODE:
			DEBUG_MSG("[MSG_AVI_START_ENCODE]\r\n");
			pAviEncVidPara->jpeg_encode_enable_flag = 0;
			video_encode_task_reset();
            pAviEncVidPara->encode_header_size = avi_encode_set_jpeg_quality(pAviEncVidPara->quality_value);
			scaler_video_init();

			if((avi_encode_get_status()&C_AVI_ENCODE_PRE_START) == 0) {
				nRet = video_encode_task_post_q(0);
			}
			//drv_l1_pscaler_start(1);
			pAviEncPara->ta = 0;
			pAviEncPara->tv = 0;
			pAviEncPara->Tv = 0;
			pAviEncPara->pend_cnt = 0;
			pAviEncPara->post_cnt = 0;
			pAviEncPara->dummy_cnt = 0;
			for(nRet=0; nRet<AVI_ENCODE_VIDEO_BUFFER_NO; nRet++) {
				pAviEncPara->video[nRet].ready_frame = 0;
				pAviEncPara->video[nRet].encode_size = 0;
				pAviEncPara->video[nRet].key_flag = 0;
			}

			if(pAviEncPara->AviPackerCur->p_avi_wave_info) {
				pAviEncPara->freq_div = pAviEncAudPara->audio_sample_rate/AVI_ENCODE_TIME_BASE;
				pAviEncPara->tick = (INT64S)pAviEncVidPara->dwRate * pAviEncPara->freq_div;
				pAviEncPara->delta_Tv = pAviEncVidPara->dwScale * pAviEncAudPara->audio_sample_rate;
			} else {
				pAviEncPara->freq_div = 1;
				pAviEncPara->tick = (INT64S)pAviEncVidPara->dwRate * pAviEncPara->freq_div;
				pAviEncPara->delta_Tv = pAviEncVidPara->dwScale * AVI_ENCODE_TIME_BASE;
			}

			avi_encode_video_timer_start();
			msg_id = ACK_OK;
			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			dwPts = 0;
			break;

        case MSG_AVI_RESUME_ENCODE:
            DEBUG_MSG("[MSG_AVI_RESUME_ENCODE]\r\n");
            avi_encode_video_timer_start();
			msg_id = ACK_OK;
			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			break;

        case MSG_AVI_PAUSE_ENCODE:
            DEBUG_MSG("[MSG_AVI_PAUSE_ENCODE]\r\n");
            avi_encode_video_timer_stop();
            msg_id = ACK_OK;
			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
            break;

		case MSG_AVI_STOP_ENCODE:
			DEBUG_MSG("[MSG_AVI_STOP_ENCODE]\r\n");
			//DBG_PRINT("encoded frame = %d, null frame = %d\r\n", Fno, NFno);
			//fps = (Fno*1000*100) / encode_time;
			//DBG_PRINT("encode_time = %d fps = %d.%02d \r\n", encode_time, fps/100, fps - 100*(fps/100));
			avi_encode_video_timer_stop();
            pscaler_exit_1 =  1;
            while(pscaler_exit_1)
                osDelay(1);
            drv_l1_pscaler_clk_ctrl(PSCALER_B, 0);
			msg_id = ACK_OK;
			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			#if VIDEO_TIMESTAMP
			dwPts = 0;
			#endif
			break;

		case MSG_AVI_CAPTURE_PICTURE:
			DEBUG_MSG("[MSG_AVI_CAPTURE_PICTURE]\r\n");
			pAviEncVidPara->jpeg_encode_enable_flag = 1;
			pAviEncVidPara->jpeg_encode_start_flag = 1;
			pAviEncVidPara->jpeg_encode_timeout_exit = 0;
			video_encode_task_reset();
			pAviEncVidPara->encode_header_size = avi_encode_set_jpeg_quality(pAviEncVidPara->quality_value);
			scaler_video_init();
			drv_l1_pscaler_start(1);
			do {
                video_frame = avi_encode_get_empty(vid_enc_frame_q);
				pVideo = (VidEncFrame_t *)video_frame;
				if(pAviEncVidPara->jpeg_encode_timeout_exit)
				{
                    pAviEncVidPara->jpeg_encode_timeout_exit = 0;

                    fs_close(pAviEncPara->AviPackerCur->file_handle);
                    goto AVI_CAPTURE_PICTURE_FAIL;
				}
			} while(video_frame == 0);

            if(pAviEncVidPara->jpeg_encode_enable_flag == 0)
            {
                pscaler_exit_1 = 1;
                while(pscaler_exit_1)
                    osDelay(1);
            }
            //drv_l1_pscaler_clk_ctrl(PSCALER_B, 0);
			nRet = save_jpeg_file(pAviEncPara->AviPackerCur->file_handle,
								pVideo->ready_frame, pVideo->encode_size);
			pAviEncVidPara->jpeg_encode_enable_flag = 0;
			if(pAviEncVidPara->jpeg_use_addr0_flag == 0)
                avi_encode_post_empty(video_frame_q, pVideo->ready_frame);
			if(nRet < 0) {
				goto AVI_CAPTURE_PICTURE_FAIL;
			}
			msg_id = ACK_OK;
			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			break;

AVI_CAPTURE_PICTURE_FAIL:
			msg_id = ACK_FAIL;
			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);
			break;

		case MSG_AVI_STORAGE_ERR:
			DEBUG_MSG("[MSG_AVI_STORAGE_ERR]\r\n");
		case MSG_AVI_STORAGE_FULL:
			DEBUG_MSG("[MSG_AVI_STORAGE_FULL]\r\n");
		#if (AVI_AUDIO_ENCODE_EN == 1) && (AVI_ENCODE_PRE_ENCODE_EN == 0)
			// stop audio
			if(avi_encode_get_status()&C_AVI_ENCODE_AUDIO) {
				avi_encode_clear_status(C_AVI_ENCODE_AUDIO);
				avi_audio_record_stop();
			}
		#endif

			//stop avi encode timer
			if(avi_encode_get_status()&C_AVI_ENCODE_START) {
				avi_encode_clear_status(C_AVI_ENCODE_START);
			#if AVI_ENCODE_PRE_ENCODE_EN == 0
				avi_encode_clear_status(C_AVI_ENCODE_PRE_START);
			#endif
				avi_encode_video_timer_stop();
			}
			//close avi packer
			avi_enc_packer_stop(pAviEncPara->AviPackerCur);
			OSQFlush(AVIEncodeApQ);
			OSQFlush(avi_encode_ack_m);
			break;

		case MSG_AVI_ENCODE_STATE_EXIT:
			DEBUG_MSG("[MSG_AVI_ENCODE_STATE_EXIT]\r\n");
			msg_id = ACK_OK;
			osMessagePut(avi_encode_ack_m, (INT32U)&msg_id, osWaitForever);

			id = osThreadGetId();
    		osThreadTerminate(id);
			break;

		case MSG_AVI_ONE_FRAME_ENCODE:
			success_flag = 0;
#if AVI_PACKER_LIB_EN == 0
			__disable_irq();
			dwtemp = (INT64S)(pAviEncPara->tv - pAviEncPara->Tv);
			__enable_irq();
			if(dwtemp > (pAviEncPara->delta_Tv << 2)) {
				//goto EncodeNullFrame;
			}
#endif

			if(uvc_frame_q){	//////for uvc h.264///////
				INT32U status;
				osEvent t_result;

				pVideo = (VidEncFrame_t *)avi_encode_get_empty(vid_enc_frame_q);
				if (pVideo == NULL)
				{
					goto EndofEncodeOneFrame;
				}
				status = uvc_post_cur_frame(uvc_frame_q, (INT32U)pVideo);
				if(status)
					DBG_PRINT("post frame fail, %x\r\n",status);

				t_result = osMessageGet(uvc_send_ack, 1000);
				if((t_result.status != osEventMessage) || (t_result.value.v == ACK_FAIL)){
					DBG_PRINT("get uvc send done ack fail!!\r\n");
				}

				pVideo->encode_size = 0;
				pVideo->key_flag = 0;
				#ifdef VFRAME_MANAGER
				vfm_post_empty(&pAviEncVidPara->vfm, pVideo->ready_frame);
				#else
				avi_encode_post_empty(video_frame_q, pVideo->ready_frame);
				#endif
				pVideo->ready_frame = 0;

				//success_flag = 1;
				goto EndofEncodeOneFrame;
			}
#if AVI_PACKER_LIB_EN == 0
			if (video_packer_write_video() >= 0)
			{
				//success_flag = 1;
				//Fno++;
			}
#else
			video_frame = avi_encode_get_empty(vid_enc_frame_q);
			if(video_frame == 0) {
				//DEBUG_MSG(DBG_PRINT("one frame = 0x%x\r\n", video_frame));
				goto EndofEncodeOneFrame;
			}

			pVideo = (VidEncFrame_t *)video_frame;
			if(pVideo->encode_size == 0) {
				DEBUG_MSG("encode_size = 0x%x\r\n", pVideo->encode_size);
				if (pVideo->ready_frame != 0x50000000)
#ifdef VFRAME_MANAGER
				vfm_post_empty(&pAviEncVidPara->vfm, pVideo->ready_frame);
#else
				avi_encode_post_empty(video_frame_q, pVideo->ready_frame);
#endif
				pVideo->ready_frame = 0;
				goto EncodeNullFrame;
			}

			video_stream = pVideo->encode_size + 8 + 2*16;

			if(!avi_encode_disk_size_is_enough(video_stream)) {
#ifdef VFRAME_MANAGER
				vfm_post_empty(&pAviEncVidPara->vfm, pVideo->ready_frame);
#else
				avi_encode_post_empty(video_frame_q, pVideo->ready_frame);
#endif
				pVideo->ready_frame = 0;
				avi_enc_storage_full();
				goto EndofEncodeOneFrame;
			}
			video_frame = pVideo->ready_frame;
			encode_size = pVideo->encode_size;
			key_flag = pVideo->key_flag;
			nRet = 0;

			nRet = pfn_avi_encode_put_data(	pAviEncPara->AviPackerCur->avi_workmem,
											*(long*)"00dc",
											encode_size,
											(void *)video_frame,
											1,
											(key_flag ? 0 : AVIIF_KEYFRAME));
			pVideo->encode_size = 0;
			pVideo->key_flag = 0;
#ifdef VFRAME_MANAGER
			vfm_post_empty(&pAviEncVidPara->vfm, pVideo->ready_frame);
#else
			avi_encode_post_empty(video_frame_q, pVideo->ready_frame);
#endif
			pVideo->ready_frame = 0;
			if(nRet >= 0) {
				if (encode_size < 50) DEBUG_MSG("n");
				else
				DEBUG_MSG(".");
				Fno++;
				success_flag = 1;
				goto EndofEncodeOneFrame;
			} else {
				avi_encode_disk_size_is_enough(-video_stream);
				DEBUG_MSG("VidPutData = %x, size = %d !!!\r\n", nRet-0x80000000, encode_size);
			}
EncodeNullFrame:
			avi_encode_disk_size_is_enough(8+2*16);
			nRet = 0;
			nRet = pfn_avi_encode_put_data(	pAviEncPara->AviPackerCur->avi_workmem,
											*(long*)"00dc",
											0,
											(void *)NULL,
											1,
											0x00);
            dwPts += pAviEncPara->delta_Tv;

			if(nRet >= 0) {
				DEBUG_MSG("N");
				success_flag = 1;
				NFno++;
			} else {
				avi_encode_disk_size_is_enough(-(8+2*16));
				DEBUG_MSG("VidPutDataNULL = %x!!!\r\n", nRet-0x80000000);
			}
#endif //AVI_PACKER_LIB_EN == 0
EndofEncodeOneFrame:
			if(success_flag) {
				__disable_irq();
				pAviEncPara->Tv += pAviEncPara->delta_Tv;
				__enable_irq();
			}
			pAviEncPara->pend_cnt++;
			break;
		}
	}
}

// capture task function
static void capture_task_entry(void const *parm)
{
	extern INT8U jpeg_422_header[624];

	INT8U encflag = 0;
	INT32U ack_msg;
	INT32S nRet;
	JpegPara_t jpeg;
	catpure_t *cap = (catpure_t *)parm;
	osEvent result;
	osThreadId id;

	DEBUG_MSG("%s\r\n", __func__);

	// stop sensor
	if(avi_encode_get_status()&C_AVI_ENCODE_SENSOR) {
		avi_encode_clear_status(C_AVI_ENCODE_SENSOR);
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_STOP_SENSOR, avi_encode_ack_m, 5000);
		DEBUG_MSG("a.sensor stop.\r\n");
	}

	// change sensor resolution
	DEBUG_MSG("b.sensor change size.\r\n");
	nRet = sensor_start(cap->width, cap->height, cap->csi_buf, cap->csi_buf);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	// register capture isr
	capture_handle_register();

	// get jpeg header
	jpeg_header_generate(JPEG_IMG_FORMAT_422, cap->quality, cap->width, cap->height);
	memcpy((void *)cap->jpeg_buf, (void *)jpeg_header_get_addr(), jpeg_header_get_size());

	//wait sensor ready
	DEBUG_MSG("c.wait frame buffer ready\r\n");

	result = osMessageGet(cap->Sensor_m, 5000);
	if((result.status != osEventMessage) || (result.value.v == 0)) {
		RETURN(STATUS_FAIL);
	}

	encflag = 1;
Return:
	// start sensor
	if((avi_encode_get_status()&C_AVI_ENCODE_SENSOR) == 0) {
		POST_MESSAGE(AVIEncodeApQ, MSG_AVI_START_SENSOR, avi_encode_ack_m, 5000);
		avi_encode_set_status(C_AVI_ENCODE_SENSOR);
		DEBUG_MSG("d.sensor recovery size\r\n");
	}

	if(encflag) {
		jpeg.quality_value = cap->quality;
		//jpeg.input_format = C_JPEG_FORMAT_YUYV;
		jpeg.width = cap->width;
		jpeg.height = cap->height;
		jpeg.input_buffer_y = cap->csi_buf;
		jpeg.input_buffer_u = 0;
		jpeg.input_buffer_v = 0;
		jpeg.output_buffer = cap->jpeg_buf + sizeof(jpeg_422_header);
		cap->jpeg_len = jpeg_encode_once(&jpeg);
		cap->jpeg_len += sizeof(jpeg_422_header);
		DEBUG_MSG("jpeg encode size = %d\r\n", cap->jpeg_len);
	}

	if(nRet >= 0) {
		ack_msg = ACK_OK;
	} else {
		ack_msg = ACK_FAIL;
	}

	osMessagePut(cap->Ack_m, (INT32U)&ack_msg, osWaitForever);

	// kill task
	id = osThreadGetId();
    osThreadTerminate(id);
}

#if AVI_PACKER_LIB_EN == 0
static void video_packer_task_entry(void const *para)
{
	INT32S  nRet;
	INT32U  msg_id, video_frame, dwPts = 0, next_msg = 0;
	osEvent result;
	INT8U exit = 0, aidx;
	VidEncFrame_t* f;
	gpMuxPkt_t pkt;
	INT32U frame_addr;
	osThreadId id;
	INT32U t;
	INT64S dwtemp;

	DBG_PRINT("video_packer_task_entry \r\n");
	while(1)
	{
		if (next_msg){
			msg_id = next_msg;
			next_msg = 0;
		}
		else {
			//DBG_PRINT("get\r\n");
			#if CMSIS_MSGQ
			result = osMessageGet(VideoPackerQ, osWaitForever);
			msg_id = result.value.v;
			if((result.status != osEventMessage) || (msg_id == 0)) {
			#else
			INT32U para;
			if (0 > msgQReceive(VideoPackerQ, &msg_id, &para, 0)) {
			#endif
				continue;
			}
			//DBG_PRINT("msg_id=%x\r\n", msg_id);
		}
		switch(msg_id)
		{
		case MSG_PACKER_VIDEO_WRITE:
		#if VIDEO_TIMESTAMP == 0
			if (!dwPts)
				dwPts = 1000*pAviEncPara->delta_Tv/(pAviEncVidPara->dwRate*pAviEncAudPara->audio_sample_rate);
			__disable_irq();
			dwtemp = (INT64S)(pAviEncPara->tv - pAviEncPara->Tv);
			__enable_irq();
			if(dwtemp > (pAviEncPara->delta_Tv << 2)) {
				gp_memset((void*)&pkt, 0, sizeof(pkt));
				if (avi_encode_disk_size_is_enough(8 + 16))
				{
					nRet = avipacker->pack(hAVIPacker, &pkt, GP_ES_TYPE_VIDEO);
					if (nRet < 0)
						avi_encode_disk_size_is_enough(-(8 + 16));
					else {
						__disable_irq();
						pAviEncPara->Tv += pAviEncPara->delta_Tv;
						__enable_irq();
						DEBUG_MSG("N");
						pAviEncPara->dummy_cnt++;
					}
				}
			}
		#endif

			f = (VidEncFrame_t*)avi_encode_get_empty(vid_enc_frame_q);
			if (!f)
			{
				if (exit)
					next_msg = MSG_PACKER_AUDIO_WRITE;
				break;
			}
			if(!f->encode_size) {
				frame_addr = 0;
			}
			else {
				frame_addr = f->ready_frame;
			}

			if (avi_encode_disk_size_is_enough(f->encode_size + 8 + 16))
			{
			#if VIDEO_TIMESTAMP
				if (dwPts)
					dwPts = f->pts - dwPts;
				else
					dwPts = 1000*pAviEncPara->delta_Tv/(pAviEncVidPara->dwRate*pAviEncAudPara->audio_sample_rate);
			#endif
				pkt.data = (INT8U*)frame_addr;
				pkt.size = f->encode_size;
				pkt.frameType = f->key_flag;
				pkt.pts = dwPts;
				nRet = avipacker->pack(hAVIPacker, &pkt, GP_ES_TYPE_VIDEO);

				if (nRet < 0)
				{
					#if VIDEO_TIMESTAMP == 0
					__disable_irq();
					pAviEncPara->Tv -= pAviEncPara->delta_Tv;
					__enable_irq();
					#endif
					avi_encode_disk_size_is_enough(-(f->encode_size + 8 + 16));
				}
				else {
				#if VIDEO_TIMESTAMP
					dwPts = f->pts;
				#endif
					if (f->encode_size < 50)
					{
						pAviEncPara->dummy_cnt++;
						DEBUG_MSG("n");
					}
					else
					{
						pAviEncPara->dummy_cnt = 0;
						DEBUG_MSG(".");
					}
				}
			}
			else
				avi_enc_storage_full();
#ifdef VFRAME_MANAGER
			if (f->ready_frame != 0x50000000)
				vfm_post_empty(&pAviEncVidPara->vfm, f->ready_frame);
#else
			avi_encode_post_empty(video_frame_q, f->ready_frame);
#endif
			f->ready_frame = 0;
			f->encode_size = 0;
			f->key_flag = 0;

			next_msg = MSG_PACKER_VIDEO_WRITE;
			break;
		case MSG_PACKER_AUDIO_WRITE:
#if AVI_AUDIO_ENCODE_EN
			nRet = avi_encode_get_empty(aud_enc_frameq);
			if (nRet)
			{
				pkt = *(gpMuxPkt_t*)nRet;
				gp_free((void*)nRet);
				nRet = avipacker->pack(hAVIPacker, &pkt, GP_ES_TYPE_AUDIO);
				next_msg = MSG_PACKER_AUDIO_WRITE;
				if (nRet < 0)
				{
					DEBUG_MSG("X");
					avi_encode_disk_size_is_enough(-pkt.size);
				}
				else
					DEBUG_MSG("A");
			}
			else {
				if (exit)
					next_msg = MSG_PACKER_EXIT;
			}
#endif
			break;

		case MSG_PACKER_EXIT:
			if (!exit) {
				exit = 1;
				next_msg = MSG_PACKER_VIDEO_WRITE;
				break;
			}
			msg_id = ACK_OK;
			osMessagePut(video_packer_ack_m, (INT32U)&msg_id, osWaitForever);
			DBG_PRINT("video_packer_task_entry exit\r\n");
			id = osThreadGetId();
    		osThreadTerminate(id);
			return;
		}
	}
}

INT32S video_packer_task_create()
{
	INT32S nRet;
	osThreadId id;
	osThreadDef_t state_task = {"video_packer_task", video_packer_task_entry, osPriorityHigh, 1, 2048};

	if(VideoPackerQ == 0) {
		#if CMSIS_MSGQ
        osMessageQDef_t App_q = {VIDEO_PACKER_QUEUE_MAX_LEN, sizeof(INT32U), 0};

		VideoPackerQ = osMessageCreate(&App_q, NULL);
		#else
		VideoPackerQ = msgQCreate(VIDEO_PACKER_QUEUE_MAX_LEN, VIDEO_PACKER_QUEUE_MAX_LEN, 0);
		#endif
		if(VideoPackerQ == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(video_packer_ack_m == 0) {
        osMessageQDef_t Ack_q = {C_ACK_QUEUE_MAX, sizeof(INT32U), 0};

		video_packer_ack_m = osMessageCreate(&Ack_q, NULL);
		if(video_packer_ack_m == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	id = osThreadCreate(&state_task, (void *) NULL);
	if(id == 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S video_packer_task_del()
{
	INT32S nRet = STATUS_OK;
	osEvent result;

	#if CMSIS_MSGQ
	POST_MESSAGE(VideoPackerQ, MSG_PACKER_EXIT, video_packer_ack_m, 5000);
	#else
	if (0 > msgQSend(VideoPackerQ, MSG_PACKER_EXIT, 0, 0, MSG_PRI_NORMAL))
		return STATUS_FAIL;
	result = osMessageGet(video_packer_ack_m, 5000);
	if((result.status != osEventMessage) || (result.value.v == ACK_FAIL))
	{
		DEBUG_MSG("GetMsg ack Fail!!! message %x\r\n",MSG_PACKER_EXIT);
		return STATUS_FAIL;
	}
	#endif

Return:
	#if CMSIS_MSGQ
	OSQFlush(VideoPackerQ);
	#endif
	OSQFlush(video_packer_ack_m);
	#if CMSIS_MSGQ
	vQueueDelete(VideoPackerQ);
	#else
	if (VideoPackerQ)
		msgQDelete(VideoPackerQ);
	#endif
	vQueueDelete(video_packer_ack_m);
	VideoPackerQ = NULL;
	video_packer_ack_m = NULL;
	return nRet;
}

INT32S video_packer_msg(INT32U msg, INT32U priority)
{
	#if CMSIS_MSGQ
	osStatus status;

	status = osMessagePut(VideoPackerQ, (INT32U)&msg, 0);
	if(status != osOK)
	#else
	if (0 > msgQSend(VideoPackerQ, msg, 0, 0, priority))
	#endif
	{
		return STATUS_FAIL;
	}

	return STATUS_OK;
}
INT32S video_packer_write_video()
{
	return video_packer_msg(MSG_PACKER_VIDEO_WRITE, MSG_PRI_NORMAL);
}

INT32S video_packer_write_audio()
{
	return video_packer_msg(MSG_PACKER_AUDIO_WRITE, MSG_PRI_URGENT);
}
#endif
#endif
