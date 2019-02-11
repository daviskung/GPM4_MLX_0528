#include "drv_l1_i2s_rx.h"
#include "drv_l1_adc.h"
#include "drv_l1_dma.h"
#include "wav_enc.h"
#include "gp_avcodec.h"
#include "gp_mux.h"
#include "avi_encoder_app.h"
#include "cp_camera_processor.h"

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define CP_ENCODE_PCM_BUFFER_NO	3
#define CP_AUDENC_TIMES 16
/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef enum
{
	MSG_AUDIO_DMA_DONE = C_DMA_STATUS_DONE,
	MSG_AUDIO_RECORD_START = 0x4000,
	MSG_AUDIO_RECORD_STOP,
	MSG_AUDIO_RECORD_PAUSE,
	MSG_AUDIO_RECORD_RESTART,
	MSG_AUDIO_RECORD_EXIT
} MSG_ENCODE_AUDIO_ENUM;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
#if AVI_AUDIO_ENCODE_EN == 1
static INT32S cp_audio_handle_data(INT32U pcm_addr, INT32U pcm_cwlen);
static void cp_audio_record_entry(void const *parm);
static INT32S cp_wave_encode_start(void);
static INT32S cp_wave_encode_stop(void);
static INT32S cp_wave_encode_once(INT16S *pcm_input_addr);
#endif

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
#if AVI_AUDIO_ENCODE_EN == 1
osMessageQId task_aud_q = 0;
osMessageQId task_aud_ack_m = 0;
#endif

extern CP_ENCODER_INFO encinfo;

static AviEncAudPara_t EncAudPara;
static INT8U g_pcm_index;
static INT64U audio_time = 0;

// audio function
static INT32S cp_audio_memory_allocate(INT32U	cblen)
{
	INT16U *ptr;
	INT32S i, j, size, nRet;

	for(i=0; i<CP_ENCODE_PCM_BUFFER_NO; i++) {
		EncAudPara.pcm_input_addr[i] = (INT32U) gp_malloc_align(cblen, 4);
		if(!EncAudPara.pcm_input_addr[i]) {
			return STATUS_FAIL;
		}

		ptr = (INT16U*)EncAudPara.pcm_input_addr[i];
		for(j=0; j<(cblen/2); j++) {
			*ptr++ = 0x8000;
		}
	}

	size = EncAudPara.pack_size * CP_AUDENC_TIMES;
	EncAudPara.pack_buffer_addr = (INT32U) gp_malloc_align(size, 4);
	if(!EncAudPara.pack_buffer_addr) {
		return STATUS_FAIL;
	}
	nRet = STATUS_OK;
Return:
	return nRet;
}

static void cp_audio_memory_free(void)
{
	INT32U i;

	for(i=0; i<CP_ENCODE_PCM_BUFFER_NO; i++) {
		if (EncAudPara.pcm_input_addr[i])
			gp_free((void *)EncAudPara.pcm_input_addr[i]);
		EncAudPara.pcm_input_addr[i] = 0;
	}

	gp_free((void *)EncAudPara.pack_buffer_addr);
	EncAudPara.pack_buffer_addr = 0;
}

static INT32U cp_audio_get_next_buffer(void)
{
	INT32U addr;

	addr = EncAudPara.pcm_input_addr[g_pcm_index++];
	if(g_pcm_index >= CP_ENCODE_PCM_BUFFER_NO) {
		g_pcm_index = 0;
	}
	return addr;
}

// function
INT32S cp_adc_record_task_create()
{
#if AVI_AUDIO_ENCODE_EN == 1
	INT32S nRet;
	osThreadId id;
	osThreadDef_t aud_task = { "audio_task", cp_audio_record_entry, osPriorityRealtime, 1, 2048 };

	if(task_aud_q == 0) {
		osMessageQDef_t aud_q = { 5, sizeof(INT32U), 0 };

		task_aud_q = osMessageCreate(&aud_q, NULL);
		if(task_aud_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(task_aud_ack_m == 0) {
		osMessageQDef_t aud_ack_q = { 1, sizeof(INT32U), 0 };

		task_aud_ack_m = osMessageCreate(&aud_ack_q, NULL);
		if(task_aud_ack_m == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	id = osThreadCreate(&aud_task, (void *)NULL);
	if(id == 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
    return nRet;
#else
	return STATUS_OK;
#endif
}

INT32S cp_adc_record_task_del(void)
{
#if AVI_AUDIO_ENCODE_EN == 1
	INT32U nRet = STATUS_OK;

	POST_MESSAGE(task_aud_q, MSG_AUDIO_RECORD_EXIT, task_aud_ack_m, 5000);

Return:
	OSQFlush(task_aud_q);
	vQueueDelete(task_aud_q);
	task_aud_q = 0;

	OSQFlush(task_aud_ack_m);
	vQueueDelete(task_aud_ack_m);
	task_aud_ack_m = 0;

	return nRet;
#else
	return STATUS_OK;
#endif
}

#if AVI_AUDIO_ENCODE_EN == 1
INT32S cp_audio_record_start(void)
{
	INT32S nRet = STATUS_OK;

	if (!task_aud_q)
		return STATUS_FAIL;

	POST_MESSAGE(task_aud_q, MSG_AUDIO_RECORD_START, task_aud_ack_m, 5000);
Return:
	return nRet;
}

INT32S cp_audio_record_restart(void)
{
	INT32S nRet = STATUS_OK;

	if (!task_aud_q)
		return STATUS_FAIL;

	POST_MESSAGE(task_aud_q, MSG_AUDIO_RECORD_RESTART, task_aud_ack_m, 5000);
Return:
	return nRet;
}

INT32S cp_audio_record_stop(void)
{
	INT32S nRet = STATUS_OK;

	if (!task_aud_q)
		return STATUS_FAIL;

	POST_MESSAGE(task_aud_q, MSG_AUDIO_RECORD_STOP, task_aud_ack_m, 5000);
Return:
	if(nRet < 0) {
		avi_adc_hw_stop();
		cp_audio_memory_free();
	}
	return nRet;
}

INT32S cp_audio_record_pause()
{
	INT32S nRet = STATUS_OK;

	if (!task_aud_q)
		return STATUS_FAIL;

	POST_MESSAGE(task_aud_q, MSG_AUDIO_RECORD_PAUSE, task_aud_ack_m, 5000);
Return:
	if(nRet < 0) {
		avi_adc_hw_stop();
		cp_audio_memory_free();
	}
	return nRet;
}

INT32U cp_get_audio_recording_time()
{
	return (INT32U)(audio_time/1000); // in ms
}

INT32U cp_get_audio_frame_duration()
{
	return (EncAudPara.pcm_input_size * CP_AUDENC_TIMES * 1000) / EncAudPara.audio_sample_rate;
}
#endif //AVI_AUDIO_ENCODE_EN == 1

#if AVI_AUDIO_ENCODE_EN == 1
static void cp_audio_record_entry(void const *parm)
{
	INT8U   audio_flag=0, audio_pause = 0;
	INT32S  ret, audio_stream, encode_size;
	INT32U  msg_id, ack_msg;
	INT32U  pcm_addr=0, pcm_cwlen=0, put_cnt;
	INT32U  i, ready_addr=0, next_addr=0;

	INT32S (*aud_rec_dbf_put)(INT16S *data, INT32U cwlen, osMessageQId os_q);
	INT32S (*aud_rec_dbf_set)(INT16S *data, INT32U cwlen);
	INT32S (*aud_rec_dma_status_get)(void);
	INT32S (*aud_rec_dbf_status_get)(void);
	void (*aud_rec_dbf_free)(void);

	osThreadId id;
	osEvent result;

	INT64U dwPts;

	DEBUG_MSG("<<%s>>\r\n", __func__);

	while(1)
	{
		result = osMessageGet(task_aud_q, osWaitForever);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (	msg_id == 0)) {
			continue;
		}

		switch(msg_id)
		{
			case MSG_AUDIO_DMA_DONE:
				pcm_addr = ready_addr;
				ready_addr = next_addr;
				next_addr = cp_audio_get_next_buffer();
				aud_rec_dbf_set((INT16S*)next_addr, pcm_cwlen);

				#if VARIABLE_FRAME_RATE
				if (!audio_time)
					audio_time = cp_global_tick*1000;
				#endif

				encode_size = cp_audio_handle_data(pcm_addr, pcm_cwlen);
				audio_stream = encode_size + 8 + 2*16;

				// when restart, wait pcm frame end
				if(audio_flag == 1) {
					audio_flag = 0;
					ack_msg = ACK_OK;
					osMessagePut(task_aud_ack_m, (INT32U)&ack_msg, osWaitForever);
				}

				if (encinfo.encode_start == 0 || audio_pause)
				{
					if (audio_pause)
						audio_time += dwPts; //audio time will affect video drop, when audio pause, add time still.
					break;
				}

				if ((cp_status & CP_STATUS_MUX_START) == 0)
					break;

				if(encode_size > 0) {
					frame_info_t* frame;
					frame = gp_malloc(sizeof(frame_info_t));
					frame->enc_out = (INT8U*)EncAudPara.pack_buffer_addr;
					frame->size = encode_size;
					#if VARIABLE_FRAME_RATE
					frame->time = (INT32U)(audio_time/1000);
					#else
					frame->time = (INT32U)(dwPts/1000);
					#endif

					if ( 0 > cp_muxer_audio(frame))
					{
						gp_free(frame);
						if (cp_muxer_get_error())
							cp_muxer_stop(0);
					}
					else
						audio_time += dwPts;
				}
				break;

			case MSG_AUDIO_RECORD_START:
				DEBUG_MSG("[MSG_AUDIO_RECORD_START]\r\n");
				audio_flag = 0;
				put_cnt = 0;
				audio_pause = 0;

				// audio encode set
				ret = cp_wave_encode_start();
				if(ret < 0) {
					goto AUDIO_RECORD_START_FAIL;
				}
				audio_time = 0;

				pcm_cwlen = EncAudPara.pcm_input_size * CP_AUDENC_TIMES;
				encode_size =EncAudPara.pack_size * CP_AUDENC_TIMES;
				dwPts = ((INT64U)pcm_cwlen)*1000000/EncAudPara.audio_sample_rate;

				ret = cp_audio_memory_allocate(pcm_cwlen<<1);
				if(ret < 0) {
					goto AUDIO_RECORD_START_FAIL;
				}

				// analog input set
			#if MIC_INPUT_SRC == C_ADC_LINE_IN
				aud_rec_dbf_put = drv_l1_adc_dbf_put;
				aud_rec_dbf_set = drv_l1_adc_dbf_set;
				aud_rec_dma_status_get = drv_l1_adc_dma_status_get;
				aud_rec_dbf_status_get = drv_l1_adc_dbf_status_get;
				aud_rec_dbf_free = drv_l1_adc_dbf_free;
			#elif MIC_INPUT_SRC == C_BUILDIN_MIC_IN || MIC_INPUT_SRC == C_LINE_IN_LR
				aud_rec_dbf_put = drv_l1_i2s_rx_adc_dbf_put;
				aud_rec_dbf_set = drv_l1_i2s_rx_adc_dbf_set;
				aud_rec_dma_status_get = drv_l1_i2s_rx_adc_dma_status_get;
				aud_rec_dbf_status_get = drv_l1_i2s_rx_adc_dbf_status_get;
				aud_rec_dbf_free = drv_l1_i2s_rx_adc_dbf_free;
			#endif

				ready_addr = cp_audio_get_next_buffer();
				next_addr = cp_audio_get_next_buffer();
				ret = aud_rec_dbf_put((INT16S*)ready_addr, pcm_cwlen, task_aud_q);
				if(ret < 0) {
					goto AUDIO_RECORD_START_FAIL;
				}

				ret = aud_rec_dbf_set((INT16S*)next_addr, pcm_cwlen);
				if(ret < 0) {
					goto AUDIO_RECORD_START_FAIL;
				}

				avi_adc_hw_start(EncAudPara.audio_sample_rate);

				ack_msg = ACK_OK;
				osMessagePut(task_aud_ack_m, (INT32U)&ack_msg, osWaitForever);
				break;

AUDIO_RECORD_START_FAIL:
				avi_adc_hw_stop();
				aud_rec_dbf_free();
				cp_wave_encode_stop();
				cp_audio_memory_free();
				DBG_PRINT("AudEncStartFail!!!\r\n");

				ack_msg = ACK_FAIL;
				osMessagePut(task_aud_ack_m, (INT32U)&ack_msg, osWaitForever);
				break;

			case MSG_AUDIO_RECORD_STOP:
				DEBUG_MSG("[MSG_AUDIO_RECORD_STOP]\r\n");
				// wait dma stop
				while(aud_rec_dbf_status_get() == 1 || aud_rec_dma_status_get() == 1) {
					osDelay(20);
				}

				avi_adc_hw_stop();
				aud_rec_dbf_free();

				// handle the final data
				for(i=0; i<2; i++) {
					frame_info_t* frame;

					if(i == 0) {
						pcm_addr = ready_addr;
						ready_addr = 0;
					} else {
						pcm_addr = next_addr;
						next_addr = 0;
					}

					if(pcm_addr == 0) {
						break;
					}

					encode_size = cp_audio_handle_data(pcm_addr, pcm_cwlen);
					if(encode_size <= 0) {
						break;
					}

					if ((cp_status & CP_STATUS_MUX_START) == 0 || audio_pause)
						break;
					frame = gp_malloc(sizeof(frame_info_t));
					frame->enc_out = (INT8U*)EncAudPara.pack_buffer_addr;
					frame->size = encode_size;
					frame->time = (INT32U)(audio_time/1000);
					#if VARIABLE_FRAME_RATE
					audio_time += ((INT64U)pcm_cwlen)*1000000/EncAudPara.audio_sample_rate;
					#endif
					if ( 0 > cp_muxer_audio(frame))
						gp_free(frame);
				}

				cp_wave_encode_stop();
				cp_audio_memory_free();

				ack_msg = ACK_OK;
				osMessagePut(task_aud_ack_m, (INT32U)&ack_msg, osWaitForever);
				break;

			case MSG_AUDIO_RECORD_RESTART:
				DEBUG_MSG("[MSG_AUDIO_RECORD_RESTART]\r\n");
				audio_flag = 1;
				audio_pause = 0;
				break;
			case MSG_AUDIO_RECORD_PAUSE:
				audio_pause = 1;
				osMessagePut(task_aud_ack_m, (INT32U)&ack_msg, osWaitForever);
				break;
			case MSG_AUDIO_RECORD_EXIT:
				DEBUG_MSG("[MSG_AUDIO_RECORD_EXIT]\r\n");
				OSQFlush(task_aud_q);

				ack_msg = ACK_OK;
				osMessagePut(task_aud_ack_m, (INT32U)&ack_msg, osWaitForever);

				id = osThreadGetId();
    			osThreadTerminate(id);
				break;
		}
	}
}

static INT32S cp_audio_handle_data(INT32U pcm_addr, INT32U pcm_cwlen)
{
#if (MIC_INPUT_SRC == C_ADC_LINE_IN)
	INT16U *pData = (INT16U *)pcm_addr;
	INT16U t, mask = 0x8000;
	INT32U i;

	// invalid cache
	cache_invalid_range(pcm_addr, pcm_cwlen << 1);

	// unsigned to signed
	for(i=0; i<pcm_cwlen; i++)
	{
		t = *pData;
		t ^= mask;
		*pData++ = t;
	}
#endif

	// wave encode
	return cp_wave_encode_once((INT16S*)pcm_addr);
}

static INT32S cp_wave_encode_start(void)
{
#if APP_WAV_CODEC_EN == 1
	INT32S nRet, size;
	INT32U audio_format;

	size = wav_enc_get_mem_block_size();
	EncAudPara.work_mem = (INT8U *)gp_malloc_align(size, 4);
	if(!EncAudPara.work_mem) {
		RETURN(STATUS_FAIL);
	}

	EncAudPara.audio_format = encinfo.cfg.aFormat;
	EncAudPara.audio_sample_rate = encinfo.cfg.sampleRate;
	EncAudPara.channel_no = 1;

	memset((INT8S*)EncAudPara.work_mem, 0, size);
	if(EncAudPara.audio_format == WAV) {
		audio_format = WAVE_FORMAT_PCM;
	} else if(EncAudPara.audio_format == MICROSOFT_ADPCM) {
		audio_format = WAVE_FORMAT_ADPCM;
	} else if(EncAudPara.audio_format == IMA_ADPCM) {
		audio_format = WAVE_FORMAT_IMA_ADPCM;
	} else if(EncAudPara.audio_format == ALAW) {
		audio_format = WAVE_FORMAT_ALAW;
	} else if(EncAudPara.audio_format == ALAW) {
		audio_format = WAVE_FORMAT_MULAW;
	} else {
		RETURN(STATUS_FAIL);
	}

	nRet = wav_enc_Set_Parameter( EncAudPara.work_mem,
								  EncAudPara.channel_no,
								  EncAudPara.audio_sample_rate,
								  audio_format);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = wav_enc_init(EncAudPara.work_mem);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	EncAudPara.pcm_input_size = wav_enc_get_SamplePerFrame(EncAudPara.work_mem);
	switch(EncAudPara.audio_format)
	{
	case WAV:
		EncAudPara.pack_size = EncAudPara.pcm_input_size;
		EncAudPara.pack_size *= 2;
		break;

	case ALAW:
	case MULAW:
	case MICROSOFT_ADPCM:
	case IMA_ADPCM:
		EncAudPara.pack_size = wav_enc_get_BytePerPackage(EncAudPara.work_mem);
		break;
	}

	nRet = STATUS_OK;
Return:
	return nRet;
#else
	return STATUS_FAIL;
#endif
}

static INT32S cp_wave_encode_stop(void)
{
	if(EncAudPara.work_mem) {
		gp_free((void*)EncAudPara.work_mem);
		EncAudPara.work_mem = 0;
	}
	return STATUS_OK;
}

static INT32S cp_wave_encode_once(INT16S *pcm_input_addr)
{
#if APP_WAV_CODEC_EN == 1
	INT8U *encode_output_addr;
	INT32S nRet, encode_size, N;

	encode_size = 0;
	N = CP_AUDENC_TIMES;
	encode_output_addr = (INT8U*)EncAudPara.pack_buffer_addr;
	while(N--) {
		nRet = wav_enc_run(EncAudPara.work_mem, (short *)pcm_input_addr, encode_output_addr);
		if(nRet < 0) {
			return STATUS_FAIL;
		}

		encode_size += nRet;
		pcm_input_addr += EncAudPara.pcm_input_size;
		encode_output_addr += EncAudPara.pack_size;
	}

	return encode_size;
#else
	return  STATUS_FAIL;
#endif
}
#endif //AVI_AUDIO_ENCODE_EN
