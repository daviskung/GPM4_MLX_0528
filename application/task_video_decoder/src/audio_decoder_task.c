#include "task_video_decoder.h"
#include "drv_l1_dma.h"
#include "drv_l1_dac.h"
#include "wav_dec.h"

#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECODER_EN == 1)

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define C_AUDIO_DECODE_STACK_SIZE		512
#define C_AUDIO_Q_ACCEPT_MAX			5

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef enum
{
	MSG_AUD_DEC_DMA_DONE = C_DMA_STATUS_DONE,
	MSG_AUD_DEC_START = 0x2000,
	MSG_AUD_DEC_STOP,
    MSG_AUD_NO_DEC_INIT,
	MSG_AUD_NO_DEC_GET_ONE_FRAME,
	MSG_AUD_DEC_EXIT
} MSG_AUD_DEC_ENUM;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
static INT32S aud_dec_special_effect_start(void);
static void aud_dec_special_effect_stop(void);

#if MP3_DECODE_ENABLE == 1
static INT32S aud_dec_mp3_init(void);
static INT32S aud_dec_mp3_one_frame(INT16S* pcm_buffer_addr);
static INT32S aud_dec_mp3_one_output(void *work_mem, INT16S *pcm_buffer_addr, INT32S maxlen);
#endif

static INT32S aud_dec_wave_init(void);
static INT32S aud_dec_wave_one_frame(INT16S* pcm_buffer_addr);
static INT32S aud_dec_wave_one_output(void *work_mem, INT16S *pcm_buffer_addr, INT32S maxlen);

#if AAC_DECODE_ENABLE == 1
static INT32S aud_dec_aac_init(void);
static INT32S aud_dec_aac_one_frame(INT16S* pcm_buffer_addr);
static INT32S aud_dec_acc_one_output(void *work_mem, INT16S *pcm_buffer_addr, INT32S maxlen);
#endif

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
INT32U	audio_decode_stack[C_AUDIO_DECODE_STACK_SIZE];
osMessageQId aud_dec_q;
osMessageQId aud_dec_ack_m;
static avi_frame_info *p_audio_frame;
void *aud_dec_q_buffer[C_AUDIO_Q_ACCEPT_MAX];
#if VOICE_CHANGER_EN == 1
	static INT32S (*pfnVcOutput)(void*, INT16S*, INT32S);
#elif UP_SAMPLE_EN == 1
	static INT32S (*pfnUsOutput)(void*, INT16S*, INT32S);
#endif


INT32S audio_decode_task_start(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(aud_dec_q, MSG_AUD_DEC_START, aud_dec_ack_m, 0, msg, err);
	// audio start play
	if(p_vid_dec_para->upsample_flag) {
		aud_dec_dac_start(p_vid_dec_para->aud_dec_ch_no, p_vid_dec_para->aud_dec_sr);
	} else {
		aud_dec_dac_start(p_wave_info->nChannels, p_wave_info->nSamplesPerSec);
	}
Return:
	return nRet;
}

INT32S audio_decode_task_stop(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(aud_dec_q, MSG_AUD_DEC_STOP, aud_dec_ack_m, 0, msg, err);
Return:
	return nRet;
}

INT32S audio_decode_task_pause(void)
{
	aud_dec_dac_stop();
	return 0;
}

INT32S audio_decode_task_resume(void)
{
	if(p_vid_dec_para->upsample_flag) {
		aud_dec_dac_start(p_vid_dec_para->aud_dec_ch_no, p_vid_dec_para->aud_dec_sr);
	} else {
		aud_dec_dac_start(p_wave_info->nChannels, p_wave_info->nSamplesPerSec);
	}
	return 0;
}

INT32S audio_decode_task_init(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(aud_dec_q, MSG_AUD_NO_DEC_INIT, aud_dec_ack_m, 5000, msg, err);
Return:
	return nRet;
}

INT32S audio_decode_get_one_frame(avi_frame_info *audio_frame)
{
	INT8U  err;
	INT32S nRet, msg;
	p_audio_frame = audio_frame;

	nRet = STATUS_OK;
	SEND_MESSAGE(aud_dec_q, MSG_AUD_NO_DEC_GET_ONE_FRAME, aud_dec_ack_m, 5000, msg, err);
Return:
	return nRet;
}

INT32S audio_decode_task_create(INT8U prio)
{
	INT8U err;
	INT32S nRet;
	osThreadId id;
	osMessageQDef_t aud_dec_qdef = {C_AUDIO_Q_ACCEPT_MAX, sizeof(void*),NULL};
    osMessageQDef_t aud_dec_ack_qdef = {1, sizeof(void*),NULL};
    osThreadDef_t aud_dec_thread_def = { "audio decode task", audio_decode_task_entry, prio, 0, C_AUDIO_DECODE_STACK_SIZE*4 };

    aud_dec_q = osMessageCreate(&aud_dec_qdef, NULL);
	if(!aud_dec_q) {
		RETURN(STATUS_FAIL);
	}

	aud_dec_ack_m = osMessageCreate(&aud_dec_ack_qdef, NULL);
	if(!aud_dec_ack_m) {
		RETURN(STATUS_FAIL);
	}

    id = osThreadCreate(&aud_dec_thread_def, NULL);
	if(id == 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S audio_decode_task_del(INT8U prio)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(aud_dec_q, MSG_AUD_DEC_EXIT, aud_dec_ack_m, 0, msg, err);
Return:
	OSQFlush(aud_dec_q);
	//OSQDel(aud_dec_q, OS_DEL_ALWAYS, &err);
	//OSMboxDel(aud_dec_ack_m, 0, &err);
	vQueueDelete(aud_dec_q);
	vQueueDelete(aud_dec_ack_m);
	return nRet;
}

void audio_decode_task_end(void)
{
	aud_dec_dac_stop();
	drv_l1_dac_dbf_free();
	vid_dec_aud_dec_memory_free();
}

void audio_decode_task_entry(void const *param)
{
	INT8U  err;
	INT32S cwlen=0, nRet, buf_RI=0, buf_WI=0;
	INT32U i, msg_id, pcm_addr=0;
	INT32U buf_addr[2], buf_len[2];
	INT64S delta_ta[AUDIO_FRAME_NO];
	//OS_CPU_SR cpu_sr;
	INT32S (*pfun_decode_one_frame)(INT16S*)=NULL;
	DMA_STRUCT audio_dma;
    INT32S WI,size;

	while(1)
	{
        osEvent event;
        event = osMessageGet(aud_dec_q, osWaitForever);
    	if (event.status != osEventMessage) {
        	continue;
        }

		switch(event.value.v)
		{
		case MSG_AUD_DEC_DMA_DONE:
			//set buffer
			drv_l1_dac_cha_dbf_set((INT16S *)pcm_addr, cwlen);

			// increase audio time
			taskENTER_CRITICAL();
			p_vid_dec_para->ta += delta_ta[buf_RI];
			taskEXIT_CRITICAL();

			buf_RI++;
			if(buf_RI>=AUDIO_FRAME_NO) {
				buf_RI -= AUDIO_FRAME_NO;
			}

			//decoder next buffer
			pcm_addr = vid_dec_get_next_aud_buffer();
			while(1) {
				cwlen = pfun_decode_one_frame((INT16S*) pcm_addr);
 				if(cwlen > 0) {
					// decode success
					break;
				} else if(cwlen < 0) {
					// decode fail
					goto __AudDecEnd;
				} else {
					if(vid_dec_get_status() & C_VIDEO_DECODE_ERR) {
						goto __AudDecEnd;
					}

					if(MultiMediaParser_IsEOA(p_vid_dec_para->media_handle) == 1) {
						DEBUG_MSG("AudDataDecEnd\r\n");
						goto __AudDecEnd;
					}

					if(MultiMediaParser_IsEOA(p_vid_dec_para->media_handle) == 2) {
						DEBUG_MSG("AudDataReadEnd\r\n");
						goto __AudDecEnd;
					}

					//DEBUG_MSG("WaitAudDataGet\r\n");
					osDelay(3);
				}
			}

			// x2 because DAC output rate would be twice of sample rate if mono audio
			// mp3 is hardware decode, force to 2 channel
			if(p_wave_info->wFormatTag == WAVE_FORMAT_MPEGLAYER3) {
				delta_ta[buf_WI] = (INT64S)p_vid_dec_para->VidTickRate * cwlen;
			} else if(p_wave_info->nChannels == 1) {
				delta_ta[buf_WI] = (INT64S)p_vid_dec_para->VidTickRate * (cwlen<<1);
			} else {
				delta_ta[buf_WI] = (INT64S)p_vid_dec_para->VidTickRate * cwlen;
			}

			if(p_vid_dec_para->upsample_flag) {
				delta_ta[buf_WI] >>= 1;
			}

			buf_WI++;
			if(buf_WI >= AUDIO_FRAME_NO) {
				buf_WI -= AUDIO_FRAME_NO;
			}
			break;

		__AudDecEnd:
			//wait dma done
			while((drv_l1_dac_dbf_status_get() == 1) || (drv_l1_dac_dma_status_get() == 1))	{
				osDelay(1);
			}

			if(vid_dec_get_status() & C_AUDIO_DECODE_PLAYING) {
				audio_decode_task_end();
				aud_dec_ramp_down_handle(p_wave_info->nChannels);
				aud_dec_special_effect_stop();
				OSQFlush(aud_dec_q);
				vid_dec_end_callback(C_AUDIO_DECODE_PLAYING);
			}
			break;
		case MSG_AUD_NO_DEC_INIT:
			nRet = vid_dec_set_aud_dec_ring_buffer();
			if(nRet < 0)
			{
				DEBUG_MSG("AudioDecodeInitFail\r\n");
				msg_id = ACK_FAIL;
                osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			}
			WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
			if(!WI)
			{
				DEBUG_MSG("AudioDecodeInitFail\r\n");
				msg_id = ACK_FAIL;
                osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			}
			msg_id = ACK_OK;
            osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			break;
		case MSG_AUD_NO_DEC_GET_ONE_FRAME:
			WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
			size = WI - p_audio_frame->RI;
			if(size == 0)
			{
				if(MultiMediaParser_IsEOA(p_vid_dec_para->media_handle))
				{
					p_audio_frame->FrameSize = -1;
				}
				else
				{
					p_audio_frame->FrameSize = 0;
				}
			}
			else if(size < 0)
			{
				size = PARSER_AUD_BITSTREAM_SIZE - p_audio_frame->RI;
				if(size != 0)
				{
					audio_dma.s_addr = ((INT32U)p_vid_dec_para->ring_buffer_addr) + p_audio_frame->RI;
					audio_dma.t_addr = (INT32U)p_audio_frame->AddrFrame;
					audio_dma.count =  size ;
					audio_dma.width = 1;
					audio_dma.timeout = 128;
					audio_dma.notify = NULL;
					nRet = drv_l1_dma_transfer_wait_ready(&audio_dma);
					if(nRet < 0)
					{
						msg_id = ACK_FAIL;
                        osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
					}
					p_audio_frame->FrameSize += size;
				}

				size = WI;
				if(size != 0)
				{
					audio_dma.s_addr = ((INT32U)p_vid_dec_para->ring_buffer_addr);
					audio_dma.t_addr = ((INT32U)p_audio_frame->AddrFrame + p_audio_frame->FrameSize);
					audio_dma.count =  size ;
					audio_dma.width = 1;
					audio_dma.timeout = 128;
					audio_dma.notify = NULL;
					nRet = drv_l1_dma_transfer_wait_ready(&audio_dma);
					if(nRet < 0)
					{
						msg_id = ACK_FAIL;
                        osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
					}
					p_audio_frame->FrameSize += size;
				}
			}
			else
			{

				audio_dma.s_addr = ((INT32U)p_vid_dec_para->ring_buffer_addr) + p_audio_frame->RI;
				audio_dma.t_addr = (INT32U)p_audio_frame->AddrFrame;
				audio_dma.count =  size ;
				audio_dma.width = 1;
				audio_dma.timeout = 128;
				audio_dma.notify = NULL;
				nRet = drv_l1_dma_transfer_wait_ready(&audio_dma);
				if(nRet < 0)
				{
					msg_id = ACK_FAIL;
                    osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
				}
				p_audio_frame->FrameSize = size;
			}
			p_audio_frame->RI = WI;
			MultiMediaParser_RefillAudBuf(p_vid_dec_para->media_handle, p_audio_frame->RI);
			msg_id = ACK_OK;
            osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			break;
		case MSG_AUD_DEC_START:
			DEBUG_MSG("MSG_AUD_DEC_START\r\n");
			buf_RI = buf_WI = 0;
			aud_dec_special_effect_start();
			switch(p_wave_info->wFormatTag)
			{
			case WAVE_FORMAT_PCM:
			case WAVE_FORMAT_MULAW:
			case WAVE_FORMAT_ALAW:
           	case WAVE_FORMAT_ADPCM:
           	case WAVE_FORMAT_DVI_ADPCM:
				nRet = aud_dec_wave_init();
				pfun_decode_one_frame = aud_dec_wave_one_frame;
				break;

			#if MP3_DECODE_ENABLE == 1
			case WAVE_FORMAT_MPEGLAYER3:
				nRet = aud_dec_mp3_init();
				pfun_decode_one_frame = aud_dec_mp3_one_frame;
				break;
			#endif

			#if AAC_DECODE_ENABLE == 1
			case WAVE_FORMAT_RAW_AAC1:
			case WAVE_FORMAT_MPEG_ADTS_AAC:
				nRet = aud_dec_aac_init();
				pfun_decode_one_frame = aud_dec_aac_one_frame;
				break;
			#endif

			default:
				while(1);
			}

			if(nRet < 0) {
				goto __AudDecStartFail;
			}

			for(i=0; i<AUDIO_FRAME_NO; i++) {
				pcm_addr = vid_dec_get_next_aud_buffer();
				while(1) {
					cwlen = pfun_decode_one_frame((INT16S*) pcm_addr);
	 				if(cwlen > 0) {
						// decode success
						break;
					} else if(cwlen < 0) {
						// decode fail
						goto __AudDecStartFail;//__AudDecEnd;
					} else {
						if(vid_dec_get_status() & C_VIDEO_DECODE_ERR) {
							goto __AudDecStartFail;//__AudDecEnd;
						}

						if(MultiMediaParser_IsEOA(p_vid_dec_para->media_handle) == 1) {
							DEBUG_MSG("AudDataDecEnd\r\n");
							goto __AudDecStartFail;//__AudDecEnd;
						}

						if(MultiMediaParser_IsEOA(p_vid_dec_para->media_handle) == 2) {
							DEBUG_MSG("AudDataReadEnd\r\n");
							goto __AudDecStartFail;//__AudDecEnd;
						}

						DEBUG_MSG("WaitAudDataGet\r\n");
						osDelay(3);
					}
				}

				if(i <= 1) {
					buf_addr[i] = pcm_addr;
					buf_len[i] = cwlen;
				}

				if(p_wave_info->wFormatTag == WAVE_FORMAT_MPEGLAYER3) {
					delta_ta[buf_WI] = (INT64S)p_vid_dec_para->VidTickRate * cwlen;
				} else if(p_wave_info->nChannels == 1) {
					delta_ta[buf_WI] = (INT64S) p_vid_dec_para->VidTickRate * (cwlen << 1);
				} else {
					delta_ta[buf_WI] = (INT64S) p_vid_dec_para->VidTickRate * cwlen;
				}

				if(p_vid_dec_para->upsample_flag) {
					delta_ta[buf_WI] >>= 1;
				}

				buf_WI++;
				if(buf_WI >= AUDIO_FRAME_NO) {
					buf_WI -= AUDIO_FRAME_NO;
				}
			}

			drv_l1_dac_cha_dbf_put((INT16S *)buf_addr[0], buf_len[0], aud_dec_q);
			drv_l1_dac_cha_dbf_set((INT16S *)buf_addr[1], buf_len[1]);
			msg_id = ACK_OK;
			osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			break;

		__AudDecStartFail:
			DEBUG_MSG("AudioDecodeStartFail\r\n");
			audio_decode_task_end();
			aud_dec_ramp_down_handle(p_wave_info->nChannels);
			aud_dec_special_effect_stop();
			msg_id = ACK_FAIL;
			osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			break;

		case MSG_AUD_DEC_STOP:
			DEBUG_MSG("MSG_AUD_DEC_STOP\r\n");
			//wait dma done
			while((drv_l1_dac_dbf_status_get() == 1) || (drv_l1_dac_dma_status_get() == 1))	{
				osDelay(1);
			}

			audio_decode_task_end();
			aud_dec_ramp_down_handle(p_wave_info->nChannels);
			aud_dec_special_effect_stop();
			OSQFlush(aud_dec_q);
			msg_id = ACK_OK;
			osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			break;

		case MSG_AUD_DEC_EXIT:
			DEBUG_MSG("MSG_AUD_DEC_EXIT\r\n");
			msg_id = ACK_OK;
			osMessagePut(aud_dec_ack_m, (INT32U)&msg_id, 0);
			osThreadTerminate(NULL);
			break;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////
static INT32S aud_dec_special_effect_start(void)
{
#if VOICE_CHANGER_EN == 1
	INT32S (*pfnVC)(void*, INT16S*, INT32S);
#elif UP_SAMPLE_EN == 1
	INT32S (*pfnUS)(void*, INT16S*, INT32S);
#endif
#if VOICE_CHANGER_EN == 1
	if(p_vid_dec_para->vc_handle == NULL) {
		p_vid_dec_para->vc_handle = VoiceChanger_Create();
	}
#elif UP_SAMPLE_EN == 1
	if(p_vid_dec_para->us_handle == NULL) {
		p_vid_dec_para->us_handle = UpSample_Create(8192);
	}
#endif
	p_vid_dec_para->upsample_flag = FALSE;
	switch(p_wave_info->wFormatTag)
	{
		case WAVE_FORMAT_PCM:
		case WAVE_FORMAT_MULAW:
		case WAVE_FORMAT_ALAW:
		case WAVE_FORMAT_ADPCM:
		case WAVE_FORMAT_DVI_ADPCM:
		#if VOICE_CHANGER_EN == 1
			pfnVC = aud_dec_wave_one_output;
		#elif UP_SAMPLE_EN == 1
			pfnUS = aud_dec_wave_one_output;
		#endif
			break;
	#if MP3_DECODE_ENABLE == 1
		case WAVE_FORMAT_MPEGLAYER3:
		#if VOICE_CHANGER_EN == 1
	    	pfnVC = aud_dec_mp3_one_output;
		#elif UP_SAMPLE_EN == 1
			pfnUS = aud_dec_mp3_one_output;
		#endif
	    	break;
	#endif
	#if AAC_DECODE_ENABLE == 1
		case WAVE_FORMAT_RAW_AAC1:
		case WAVE_FORMAT_MPEG_ADTS_AAC:
		#if VOICE_CHANGER_EN == 1
	    	pfnVC = aud_dec_acc_one_output;
		#elif UP_SAMPLE_EN == 1
			pfnUS = aud_dec_acc_one_output;
		#endif
			break;
	#endif
    }

#if VOICE_CHANGER_EN == 1
	if(p_vid_dec_para->vc_handle) {
		VoiceChanger_Link(	p_vid_dec_para->vc_handle,
							p_vid_dec_para->work_mem_addr,
						 	pfnVC,
						 	p_wave_info->nSamplesPerSec,
						 	p_wave_info->nChannels);
		VoiceChanger_SetParam(p_vid_dec_para->vc_handle, 12, 12);
		pfnVcOutput = &VoiceChanger_GetOutput;
		p_vid_dec_para->aud_dec_ch_no = VoiceChanger_GetChannel(p_vid_dec_para->vc_handle);
		p_vid_dec_para->aud_dec_sr = VoiceChanger_GetSampleRate(p_vid_dec_para->vc_handle);
	} else {
		pfnVcOutput = pfnVC;
	}
#elif UP_SAMPLE_EN == 1
	if(p_vid_dec_para->us_handle && (p_wave_info->nSamplesPerSec <= 22050)) {
		INT8U channels = 0;

		if(p_wave_info->nChannels == 1) {
			channels = 1;
		}

		UpSample_Link(	p_vid_dec_para->us_handle,
						p_vid_dec_para->work_mem_addr,
						pfnUS,
						p_wave_info->nSamplesPerSec,
						p_wave_info->nChannels,
						2,
						channels);
		pfnUsOutput = &UpSample_GetOutput;
		p_vid_dec_para->aud_dec_ch_no = UpSample_GetChannel(p_vid_dec_para->us_handle);
		p_vid_dec_para->aud_dec_sr = UpSample_GetSampleRate(p_vid_dec_para->us_handle);
		p_vid_dec_para->upsample_flag = TRUE;
		DEBUG_MSG("AudUpSampleCh = %d\r\n", p_vid_dec_para->aud_dec_ch_no);
		DEBUG_MSG("AudUpSampleSR = %d\r\n", p_vid_dec_para->aud_dec_sr);
	} else {
		pfnUsOutput = pfnUS;
	}
#endif
	return STATUS_OK;
}

static void aud_dec_special_effect_stop(void)
{
#if VOICE_CHANGER_EN == 1
	VoiceChanger_Del(p_vid_dec_para->vc_handle);
	p_vid_dec_para->vc_handle = NULL;
#elif UP_SAMPLE_EN
	UpSample_Del(p_vid_dec_para->us_handle);
	p_vid_dec_para->us_handle = NULL;
#endif
}

#if MP3_DECODE_ENABLE == 1
static INT32S aud_dec_mp3_init(void)
{
	INT32U WI, RI;
	INT32S nRet;

	nRet = vid_dec_set_aud_dec_work_mem(MP3_DEC_MEMORY_SIZE + MP3_DECODE_RAM);

	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = vid_dec_set_aud_dec_ring_buffer();
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = mp3_dec_init((void*)p_vid_dec_para->work_mem_addr,
						(void*)p_vid_dec_para->ring_buffer_addr,
						(char*)(p_vid_dec_para->work_mem_addr + MP3_DEC_MEMORY_SIZE));
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	mp3_dec_set_bs_buf_size((void*)p_vid_dec_para->work_mem_addr, PARSER_AUD_BITSTREAM_SIZE);


	WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
	if(!WI) {
		RETURN(STATUS_FAIL);
	}

	while(1) {
		WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
		nRet = mp3_dec_parsing((void*)p_vid_dec_para->work_mem_addr, WI);
		switch(nRet)
		{
			case MP3_DEC_ERR_NONE:
				RETURN(STATUS_OK);
				break;
			case MP3_DEC_ERR_LOSTSYNC:		//not found sync word
			case MP3_DEC_ERR_BADSAMPLERATE:	//reserved sample frequency value
			case MP3_DEC_ERR_BADBITRATE:		//forbidden bitrate value
			case MP3_DEC_ERR_BADLAYER:
			default:
				RETURN(STATUS_FAIL);
		}

		//refill data
		RI = mp3_dec_get_ri((void *)p_vid_dec_para->work_mem_addr);
		MultiMediaParser_RefillAudBuf(p_vid_dec_para->media_handle, RI);

		//audio data decode end
		if(MultiMediaParser_IsEOA(p_vid_dec_para->media_handle) == 1) {
			RETURN(STATUS_FAIL);
		}
	}
Return:
	return nRet;
}

static INT32S  aud_dec_mp3_one_frame(INT16S* pcm_buffer_addr)
{
#if VOICE_CHANGER_EN == 1
	INT32S nRet;
	INT32U RI;

	nRet = pfnVcOutput(p_vid_dec_para->vc_handle, pcm_buffer_addr, MP3_DEC_FRAMESIZE);
	if(nRet < 0) {
		RETURN(nRet);
	}
#elif UP_SAMPLE_EN == 1
	INT32S nRet;
	INT32U RI;

	nRet = pfnUsOutput(p_vid_dec_para->us_handle, pcm_buffer_addr, MP3_DEC_FRAMESIZE);
	if(nRet < 0) {
		RETURN(nRet);
	}
#else
	INT32S nRet;
	INT32U RI;

	nRet = aud_dec_mp3_one_output(p_vid_dec_para->work_mem_addr, pcm_buffer_addr, MP3_DEC_FRAMESIZE);
	if(nRet < 0) {
		RETURN(nRet);
	}
#endif
Return:
	RI = mp3_dec_get_ri((void *)p_vid_dec_para->work_mem_addr);
	MultiMediaParser_RefillAudBuf(p_vid_dec_para->media_handle, RI);
	return nRet;
}

static INT32S aud_dec_mp3_one_output(void *work_mem, INT16S *pcm_buffer_addr, INT32S maxlen)
{
	INT8U last;
	INT32S pcm_point;
	INT32U RI, WI;
	INT32S size, nRet;

	last = 0;
	WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
	RI = mp3_dec_get_ri((void *)p_vid_dec_para->work_mem_addr);

	size = WI - RI;
	if(size == 0) {
		RETURN(0);
	}

	pcm_point = mp3_dec_run((void*)p_vid_dec_para->work_mem_addr, (INT16S*)pcm_buffer_addr, WI);

	if (pcm_point < 0) {
		RETURN(pcm_point);
	}

	nRet = pcm_point * mp3_dec_get_channel((void*)p_vid_dec_para->work_mem_addr);

Return:
	return nRet;
}
#endif

static INT32S aud_dec_wave_init(void)
{
	INT32U WI;
	INT32S nRet;

	nRet = vid_dec_set_aud_dec_work_mem(WAV_DEC_MEMORY_SIZE);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = vid_dec_set_aud_dec_ring_buffer();
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = wav_dec_init((INT8U *)p_vid_dec_para->work_mem_addr, p_vid_dec_para->ring_buffer_addr);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	wav_dec_set_ring_buf_size((INT8U *)p_vid_dec_para->work_mem_addr, PARSER_AUD_BITSTREAM_SIZE);
	nRet = wav_dec_set_param((INT8U *)p_vid_dec_para->work_mem_addr, (INT8U *)p_wave_info);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
	if(!WI) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

static INT32S aud_dec_wave_one_frame(INT16S* pcm_buffer_addr)
{
#if VOICE_CHANGER_EN == 1
	INT32S nRet;
	INT32U RI;

	nRet = pfnVcOutput(p_vid_dec_para->vc_handle, pcm_buffer_addr, WAV_DEC_FRAMESIZE);	//pcm_point*2
	if(nRet <= 0) {
		RETURN(nRet);
	}
#elif UP_SAMPLE_EN == 1
	INT32S nRet;
	INT32U RI;

	nRet = pfnUsOutput(p_vid_dec_para->us_handle, pcm_buffer_addr, WAV_DEC_FRAMESIZE);	//pcm_point*2
	if(nRet <= 0) {
		RETURN(nRet);
	}
#else
	INT32S nRet;
	INT32U RI;

	nRet = aud_dec_wave_one_output(p_vid_dec_para->work_mem_addr, pcm_buffer_addr, WAV_DEC_FRAMESIZE);
	if(nRet <= 0) {
		RETURN(nRet);
	}
#endif
Return:
	RI = wav_dec_get_ri((INT8U *)p_vid_dec_para->work_mem_addr);
	MultiMediaParser_RefillAudBuf(p_vid_dec_para->media_handle, RI);
	return nRet;
}

static INT32S aud_dec_wave_one_output(void *work_mem, INT16S *pcm_buffer_addr, INT32S maxlen)
{
	INT32S pcm_point;
	INT32U RI, WI;
	INT32S nRet, size;

	WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
	RI = wav_dec_get_ri((INT8U *)p_vid_dec_para->work_mem_addr);

	size = WI - RI;
	if(size == 0) {
		RETURN(0);
	}

	// when pcm format, wavelib need to limite output pcm size
	if(p_wave_info->wFormatTag == WAVE_FORMAT_PCM) {
		if(size < 0) {
			size += PARSER_AUD_BITSTREAM_SIZE;
		}

		if(size > (AUDIO_PCM_BUFFER_SIZE >> 1)) {
			WI = RI + (AUDIO_PCM_BUFFER_SIZE >> 1);
			if(WI >= PARSER_AUD_BITSTREAM_SIZE) {
				WI -= PARSER_AUD_BITSTREAM_SIZE;
			}
		}
	}

	pcm_point = wav_dec_run((INT8U *)p_vid_dec_para->work_mem_addr, (INT16S*)pcm_buffer_addr, WI);
	if(pcm_point < 0) {
		RETURN(pcm_point);
	}

	nRet = pcm_point * wav_dec_get_nChannels((INT8U *)p_vid_dec_para->work_mem_addr);
#if UP_SAMPLE_EN == 1
	if(p_vid_dec_para->upsample_flag) {
		nRet = pcm_point * UpSample_GetChannel(p_vid_dec_para->us_handle);
	}
#endif
Return:
	return nRet;
}


#if AAC_DECODE_ENABLE == 1
static INT32S aud_dec_aac_init(void)
{
	INT8U  downMatrix;
	INT32S nRet, WI, RI;

	nRet = vid_dec_set_aud_dec_work_mem(AAC_DEC_MEMORY_BLOCK_SIZE);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = vid_dec_set_aud_dec_ring_buffer();
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	downMatrix = 0;
	nRet = aac_dec_init((void*)p_vid_dec_para->work_mem_addr, downMatrix, (void*)p_vid_dec_para->ring_buffer_addr);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = aac_dec_SetRingBufferSize((void*)p_vid_dec_para->work_mem_addr, PARSER_AUD_BITSTREAM_SIZE);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
	if(!WI) {
		RETURN(STATUS_FAIL);
	}

	while(1) {
		WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
		nRet = aac_dec_parsing(p_vid_dec_para->work_mem_addr, WI);
		switch(nRet)
		{
			case AAC_OK:
				RETURN(STATUS_OK);
				break;
			case UNABLE_TO_FIND_ADTS_SYNCWORD:
			case UNSUPPORTED_FILE_FORMAT_MP4:
			case NOT_MONO_OR_STEREO:
			case NOT_LC_OBJECT_TYPE:
			default:
				RETURN(STATUS_FAIL);
		}

		//refill data
		RI = aac_dec_get_read_index((INT8U *)p_vid_dec_para->work_mem_addr);
		MultiMediaParser_RefillAudBuf(p_vid_dec_para->media_handle, RI);

		//audio data decode end
		if(MultiMediaParser_IsEOA(p_vid_dec_para->media_handle) == 1) {
			RETURN(STATUS_FAIL);
		}
	}
Return:
	return nRet;
}

static INT32S aud_dec_aac_one_frame(INT16S* pcm_buffer_addr)
{
#if VOICE_CHANGER_EN == 1
	INT32S nRet;
	INT32U RI;

	nRet = pfnVcOutput(p_vid_dec_para->vc_handle, pcm_buffer_addr, AAC_DEC_FRAMESIZE);
	if(nRet <= 0) {
		RETURN(nRet);
	}
#elif UP_SAMPLE_EN == 1
	INT32S nRet;
	INT32U RI;

	nRet = pfnUsOutput(p_vid_dec_para->us_handle, pcm_buffer_addr, AAC_DEC_FRAMESIZE);
	if(nRet <= 0) {
		RETURN(nRet);
	}
#else
	INT32S nRet;
	INT32U RI;

	nRet = aud_dec_acc_one_output(p_vid_dec_para->work_mem_addr, pcm_buffer_addr, AAC_DEC_FRAMESIZE);
	if(nRet <= 0) {
		RETURN(nRet);
	}
#endif
Return:
	RI = aac_dec_get_read_index((INT8U *)p_vid_dec_para->work_mem_addr);
	MultiMediaParser_RefillAudBuf(p_vid_dec_para->media_handle, RI);
	return nRet;
}

static INT32S aud_dec_acc_one_output(void *work_mem, INT16S *pcm_buffer_addr, INT32S maxlen)
{
	INT8U  *pdata, channels;
	INT32S pcm_point;
	INT32U RI, WI;
	INT32S nRet, size;

	WI = MultiMediaParser_GetAudRingWI(p_vid_dec_para->media_handle);
	RI = aac_dec_get_read_index((INT8U *)p_vid_dec_para->work_mem_addr);

	size = WI - RI;
	if(size == 0) {
		RETURN(0);
	}

	pcm_point = aac_dec_run((INT8U *)p_vid_dec_para->work_mem_addr, WI, (INT16S*)pcm_buffer_addr);
	if(pcm_point < 0) {
		RETURN(pcm_point);
	}

	// check channel again
	channels = aac_dec_get_channel((INT8U *)p_vid_dec_para->work_mem_addr);
	if(channels != p_wave_info->nChannels) {
		DEBUG_MSG("AAC_Aud_Ch = %d\r\n", channels);
		pdata = (INT8U*)&p_wave_info->nChannels;
		*pdata = channels;
		aud_dec_special_effect_stop();
		aud_dec_special_effect_start();
		if(p_vid_dec_para->upsample_flag) {
			aud_dec_dac_start(p_vid_dec_para->aud_dec_ch_no, p_vid_dec_para->aud_dec_sr);
		} else {
			aud_dec_dac_start(channels, p_wave_info->nSamplesPerSec);
		}
	}

#if UP_SAMPLE_EN == 1
	if(p_vid_dec_para->upsample_flag) {
		channels =  UpSample_GetChannel(p_vid_dec_para->us_handle);
	}
#endif
	nRet = pcm_point * channels;
Return:
	return nRet;
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////
#endif//#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECDOER_EN == 1)
