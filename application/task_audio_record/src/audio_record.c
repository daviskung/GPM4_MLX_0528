/**************************************************************************************************
audio record to storage:
The encode format can be
1. A1800 (GP)
2. WAVE  (PCM)
3. WAVE  (microsoft ADPCM)
4. WAVE  (IMA ADPCM)
5. MP3	 sample rate, 48k, 44.1K, 32k, 24K, 22.05k, 16K, 12K, 11.025K, 8k
		 bit rate, 32,40,48,56,64,80,96,112,128,160,192,224,256,320 kbps
		  			 8,16,24,32,40,48,56, 64, 80, 96,112,128,144,160 kbps
***************************************************************************************************/
#include "audio_record.h"

#define APP_AUDIO_STREAM_EN 1

/*  define */
#define ADC_RECORD_STACK_SIZE			1024
#define AUD_ENC_QACCEPT_MAX				5
#define AUD_ENC_ACK_QUEUE_MAX			1
#define RECORD_CLUSTER_SHIFT			16
#define RECORD_CLUSTER_SIZE			    (1<<RECORD_CLUSTER_SHIFT)       //64K

/* os stack */
#if _OPERATING_SYSTEM != _OS_NONE
  #if _OPERATING_SYSTEM == _OS_UCOS2
  INT32U  AdcRecordStack[ADC_RECORD_STACK_SIZE];
  INT32U  AdcRecordWriteStack[ADC_RECORD_STACK_SIZE];
  #elif _OPERATING_SYSTEM == _OS_FREERTOS
  #define AUD_TASK_STACKSIZE 1024
  #endif
#endif
/* os global varaible */
#if _OPERATING_SYSTEM != _OS_NONE
#if _OPERATING_SYSTEM == _OS_UCOS2
  void	 	*aud_enc_reqQ_area[AUD_ENC_QACCEPT_MAX];
  void	 	*aud_enc_reqQ_area2[AUD_ENC_QACCEPT_MAX];
  void	 	*aud_enc_writeQ_area[AUD_ENC_QACCEPT_MAX];
  OS_EVENT 	*aud_enc_reqQ;
  OS_EVENT 	*aud_enc_reqQ2;
  OS_EVENT 	*aud_enc_writeQ;
  OS_EVENT    *aud_write_task_ack_m;
  #if RECORD_WITH_DAC_OUT_EN == 1
  void	 	*aud_enc_reqQ_area3[AUD_ENC_QACCEPT_MAX];
  OS_EVENT 	*aud_enc_reqQ3;
  #endif
#elif _OPERATING_SYSTEM == _OS_FREERTOS
  xQueueHandle 	aud_enc_reqQ;
  xQueueHandle 	aud_enc_reqQ2;
  xQueueHandle 	aud_enc_writeQ;
  xQueueHandle  aud_write_task_ack_m;
  #if RECORD_WITH_DAC_OUT_EN == 1
  xQueueHandle 	aud_enc_reqQ3;
  #endif
#endif
#endif

/* global varaible */
static Audio_Encode_Para *pAudio_Encode_Para;

/* function */
static void adc_record_entry(void *param);
static void adc_record_write_entry(void *param);
//dma
#if _OPERATING_SYSTEM != _OS_NONE
#if _OPERATING_SYSTEM == _OS_UCOS2
  static INT32S aud_adc_double_buffer_put(INT16U *data,INT32U cwlen, INT8U aud_in, OS_EVENT *os_q);
#elif _OPERATING_SYSTEM == _OS_FREERTOS
  static INT32S aud_adc_double_buffer_put(INT16U *data,INT32U cwlen, INT8U aud_in, xQueueHandle os_q);
#endif
#endif
static INT32U aud_adc_double_buffer_set(INT16U *data, INT32U cwlen, INT8U aud_in);
static INT32S aud_adc_dma_status_get(INT8U aud_in);
static void aud_adc_double_buffer_free(INT8U aud_in);
static INT32S adc_hardware_start(INT8U InputDevice, INT8U channel, INT32U SampleRate);
static INT32S adc_hardware_stop(INT8U InputDevice, INT8U channel);

//other
static INT32S adc_memory_allocate(INT32U CbLen, INT8U adc_in);
static INT32S adc_memory_free(void);
static void adc_work_mem_free(void);
#if RECORD_WITH_DAC_OUT_EN == 1
static void dac_out_start(INT32U channel, INT32U samplerate);
static void dac_out_stop(INT32U channel);
#endif

//wave
#if 1
static AUD_ENC_WAVE_HEADER	WaveHeadPara;
static void aud_enc_RIFF_init(INT32U samplerate);
static INT32S wave_encode_start(void);
static INT32S wave_encode_stop(void);
static int wave_encode_once(void *workmem, const short* buffer_addr, int cwlen);
#endif

#if APP_A1800_ENCODE_EN
static INT32S a1800_encode_start(void);
static INT32S a1800_encode_stop(void);
static int a1800_encode_once(void *workmem, const short* buffer_addr, int cwlen);
#endif

#if APP_WAV_CODEC_EN
static INT32S wave_encode_lib_start(INT32U AudioFormat);
static INT32S wave_encode_lib_stop(void);
static int wave_encode_lib_once(void *workmem, const short* buffer_addr, int cwlen);
#endif

#if APP_MP3_ENCODE_EN
static INT32S mp3_encode_start(void);
static INT32S mp3_encode_stop(void);
static int mp3_encode_once(void *workmem, const short* buffer_addr, int cwlen);
#endif

#if APP_VR_ENCODE_EN
// VR global info
#include "vr_demo_global.h"

static INT32S vr_encode_start(void);
static INT32S vr_encode_stop(void);
static int vr_encode_once(void *workmem, const short* buffer_addr, int cwlen);
static INT32S audio_stream_encode_start(void);
static INT32S audio_stream_encode_stop(void);

#endif

#if APP_AUDIO_STREAM_EN
static INT32S audio_stream_encode_start(void);
static INT32S audio_stream_encode_stop(void);
static int audio_stream_encode_once(void *workmem, const short* buffer_addr, int cwlen);

#endif

#if BUILD_IN_MIC_EN == 1
extern INT32S mic_timer_stop(INT8U timer_id);
extern void mic_fifo_clear(void);
extern void mic_fifo_level_set(INT8U level);
extern INT32S mic_auto_sample_start(void);
extern INT32S mic_sample_rate_set(INT8U timer_id, INT32U hz);
#endif

#if GPY0050_IN_EN == 1
static INT16U gpy0050_get_value(void);
static void gpy0050_start(void);
static void gpy0050_stop(void);
static void gpy0050_isr(void);
#endif

static INT32U record_size = 0;

extern xSemaphoreHandle lwip_socket_sem;
//===============================================================================================
// 	audio mic/linein initial
//	parameter:	mic/linein.
//	return:		none.
//===============================================================================================
void aud_adc_input_init(INT32U input_sel, INT32U sr_hz)
{

    if(input_sel == BUILD_IN_MIC)
        drv_l1_adc_mic_enable();
        //drv_l1_i2s_rx_adc_set_input_path(0);
    else
        drv_l1_adc_linein_enable();
        //drv_l1_i2s_rx_adc_set_input_path(1);

    //drv_msec_wait(200);
	drv_l1_i2s_rx_adc_init();

    drv_l1_i2s_rx_adc_sample_rate_set(sr_hz);

	if(input_sel == BUILD_IN_MIC) {
		drv_l1_i2s_rx_set_mono(0,1);
	} else {
		drv_l1_i2s_rx_set_mono(0,0);
	}

	drv_l1_i2s_rx_adc_start();

}
//===============================================================================================
// 	audio mic/linein stop
//	parameter:	mic/linein.
//	return:		none.
//===============================================================================================
void aud_adc_input_stop(void)
{
    drv_l1_i2s_rx_adc_stop();
    drv_l1_i2s_rx_adc_exit();
}

//===============================================================================================
// 	audio task create
//	parameter:	priority.
//	return:		none.
//===============================================================================================
INT32S adc_record_task_create(INT8U priority)
{
	INT8U err;

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
        aud_enc_reqQ = OSQCreate(aud_enc_reqQ_area, AUD_ENC_QACCEPT_MAX);
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
        aud_enc_reqQ = xQueueCreate(AUD_ENC_QACCEPT_MAX, sizeof(INT32U));
    #endif
#endif

	if (!aud_enc_reqQ) {
		return STATUS_FAIL;
	}

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
        aud_enc_reqQ2 = OSQCreate(aud_enc_reqQ_area2, AUD_ENC_QACCEPT_MAX);
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
        aud_enc_reqQ2 = xQueueCreate(AUD_ENC_QACCEPT_MAX, sizeof(INT32U));
    #endif
#endif
	if (!aud_enc_reqQ2) {
		return STATUS_FAIL;
	}

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
        aud_enc_writeQ = OSQCreate(aud_enc_writeQ_area, AUD_ENC_QACCEPT_MAX);
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
        aud_enc_writeQ = xQueueCreate(AUD_ENC_QACCEPT_MAX, sizeof(INT32U));
    #endif
#endif
	if (!aud_enc_writeQ) {
		return STATUS_FAIL;
	}

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
        aud_write_task_ack_m = OSMboxCreate(NULL);
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
        aud_write_task_ack_m = xQueueCreate(AUD_ENC_ACK_QUEUE_MAX, sizeof(INT32U));
    #endif
#endif
	if (!aud_write_task_ack_m) {
		return STATUS_FAIL;
	}



#if RECORD_WITH_DAC_OUT_EN == 1

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
        aud_enc_reqQ3 = OSQCreate(aud_enc_reqQ_area3, AUD_ENC_QACCEPT_MAX);
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
        aud_enc_reqQ3 = xQueueCreate(AUD_ENC_QACCEPT_MAX, sizeof(INT32U));
    #endif
#endif
	if (!aud_enc_reqQ3) {
		return STATUS_FAIL;
	}
#endif

	if(pAudio_Encode_Para == 0) {
		pAudio_Encode_Para = (Audio_Encode_Para *)gp_malloc_align(sizeof(Audio_Encode_Para), 4);
		if(pAudio_Encode_Para == 0) {
			return STATUS_FAIL;
		}
	}

	gp_memset((INT8S *)pAudio_Encode_Para, 0, sizeof(Audio_Encode_Para));
#if ((defined MCU_VERSION) && (MCU_VERSION < GPL326XX))
	pAudio_Encode_Para->InputDevice = ADC_LINE_IN;
#else
	pAudio_Encode_Para->InputDevice = BUILD_IN_MIC;
#endif
	pAudio_Encode_Para->Channel = C_MONO_RECORD;

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
	err = OSTaskCreate(adc_record_entry, (void *)pAudio_Encode_Para, (void *)&AdcRecordStack[ADC_RECORD_STACK_SIZE - 1], priority);
	if(err != OS_NO_ERR) {
		return STATUS_FAIL;
	}
    #elif _OPERATING_SYSTEM == _OS_FREERTOS

        err = xTaskCreate(adc_record_entry, "aud_rec", AUD_TASK_STACKSIZE, pAudio_Encode_Para, priority, NULL);

        if(err != pdPASS) {
		return STATUS_FAIL;
	}

    #endif

    #if _OPERATING_SYSTEM == _OS_UCOS2
	err = OSTaskCreate(adc_record_write_entry, (void *)pAudio_Encode_Para, (void *)&AdcRecordWriteStack[ADC_RECORD_STACK_SIZE - 1], priority + 1);
	if(err != OS_NO_ERR) {
		return STATUS_FAIL;
	}
    #elif _OPERATING_SYSTEM == _OS_FREERTOS

        err = xTaskCreate(adc_record_write_entry, "aud_write", AUD_TASK_STACKSIZE, pAudio_Encode_Para, priority, NULL);

        if(err != pdPASS) {
		return STATUS_FAIL;
	}

    #endif

#endif

	DEBUG_MSG("AudioEncodeTaskCreate[%d]\r\n", priority);
	return STATUS_OK;
}

//===============================================================================================
// 	audio task detete
//	parameter:	priority.
//	return:		none.
//===============================================================================================
INT32S adc_record_task_del(INT8U priority)
{
	INT8U err;

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
	if(aud_enc_reqQ) {
		OSQFlush(aud_enc_reqQ);
	}

	if(aud_enc_reqQ2) {
		OSQFlush(aud_enc_reqQ2);
	}

#if RECORD_WITH_DAC_OUT_EN == 1
	if(aud_enc_reqQ3) {
		OSQFlush(aud_enc_reqQ3);
	}
#endif

    err = OSTaskDel(priority);
	DEBUG_MSG("AudioEncodeTaskDel[%d]\r\n", priority);


	if(aud_enc_reqQ) {
		OSQDel(aud_enc_reqQ, OS_DEL_ALWAYS, &err);
		aud_enc_reqQ = 0;
	}

	if(aud_enc_reqQ2) {
		OSQDel(aud_enc_reqQ2, OS_DEL_ALWAYS, &err);
		aud_enc_reqQ2 = 0;
	}

	if(aud_enc_writeQ) {
		OSQDel(aud_enc_writeQ, OS_DEL_ALWAYS, &err);
		aud_enc_writeQ = 0;
	}

	if(aud_write_task_ack_m) {
		OSMboxDel(aud_write_task_ack_m, OS_DEL_ALWAYS, &err);
		aud_write_task_ack_m = 0;
	}

#if RECORD_WITH_DAC_OUT_EN == 1
	if(aud_enc_reqQ3) {
		OSQDel(aud_enc_reqQ3, OS_DEL_ALWAYS, &err);
		aud_enc_reqQ3 = 0;
	}
#endif

    #elif _OPERATING_SYSTEM == _OS_FREERTOS

    {
        INT32U msg;
        osStatus status;

        msg = MSG_AUDIO_ENCODE_EXIT;
        status = osMessagePut(aud_enc_reqQ, (INT32U)&msg, osWaitForever);
        if(status != osOK) {
            DBG_PRINT("MSG_AUDIO_ENCODE_EXIT Error\r\n");
        }

        status = osMessagePut(aud_enc_writeQ, (INT32U)&msg, osWaitForever);
        if(status != osOK) {
            DBG_PRINT("MSG_AUDIO_ENCODE_EXIT Error\r\n");
        }
	}
    osDelay(5);

	if(aud_enc_reqQ) {
		xQueueReset(aud_enc_reqQ);
	}

	if(aud_enc_reqQ2) {
		xQueueReset(aud_enc_reqQ2);
	}

	if(aud_enc_writeQ) {
		xQueueReset(aud_enc_writeQ);
	}

	if(aud_write_task_ack_m) {
		xQueueReset(aud_write_task_ack_m);
	}

#if RECORD_WITH_DAC_OUT_EN == 1
	if(aud_enc_reqQ3) {
		xQueueReset(aud_enc_reqQ3);
	}
#endif

	if(aud_enc_reqQ) {
                vQueueDelete(aud_enc_reqQ);
		aud_enc_reqQ = 0;
	}

	if(aud_enc_reqQ2) {
                vQueueDelete(aud_enc_reqQ2);
		aud_enc_reqQ2 = 0;
	}

    if(aud_enc_writeQ) {
        vQueueDelete(aud_enc_writeQ);
		aud_enc_writeQ = 0;
	}

	if(aud_write_task_ack_m) {
        vQueueDelete(aud_write_task_ack_m);
		aud_write_task_ack_m = 0;
	}
#if RECORD_WITH_DAC_OUT_EN == 1
	if(aud_enc_reqQ3) {
                vQueueDelete(aud_enc_reqQ3);
		aud_enc_reqQ3 = 0;
	}
#endif


    #endif
#endif

	if(pAudio_Encode_Para) {
		gp_free((void *)pAudio_Encode_Para);
		pAudio_Encode_Para = 0;
	}
	return err;
}

//===============================================================================================
// 	adc_envelop_detect_start
//	parameter:	none.
//	return:		status.
//===============================================================================================
INT32S adc_envelop_detect_start(void)
{
	INT8U err;
	INT32S i, status;
	INT32U message;

	// a1800 only support mono record
	if(pAudio_Encode_Para->AudioFormat == WAVE_FORMAT_A1800) {
		pAudio_Encode_Para->Channel = C_MONO_RECORD;
		if(pAudio_Encode_Para->InputDevice == DOUBLE_LINEIN) {
			pAudio_Encode_Para->InputDevice = BUILD_IN_MIC;
		}
	}

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
	err = OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENVELOP_DET_START);
	if(err != OS_NO_ERR) {
		return STATUS_FAIL;
	}

    #elif _OPERATING_SYSTEM == _OS_FREERTOS
	message = MSG_AUDIO_ENVELOP_DET_START;
        err = xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
	if(err != pdPASS) {
		return STATUS_FAIL;
	}

    #endif
#endif

	//wait 10 second
	for(i=0; i<1000; i++) {
		status = audio_record_get_status();
		if(status == C_START_RECORD) {
			return STATUS_OK;
		}

		if(status == C_START_FAIL) {
			return STATUS_FAIL;
		}

        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		OSTimeDly(1);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                vTaskDelay(1);
            #endif
        #endif
	}

	if(i == 1000) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

//===============================================================================================
// 	audio record task start
//	parameter:	none.
//	return:		status.
//===============================================================================================
INT32S adc_record_task_start(void)
{
	INT8U err;
	INT32S i, status;
	INT32U message;

	// a1800 only support mono record
	if(pAudio_Encode_Para->AudioFormat == WAVE_FORMAT_A1800) {
		pAudio_Encode_Para->Channel = C_MONO_RECORD;
		if(pAudio_Encode_Para->InputDevice == DOUBLE_LINEIN) {
			pAudio_Encode_Para->InputDevice = BUILD_IN_MIC;
		}
	}

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
	err = OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_START);
	if(err != OS_NO_ERR) {
		return STATUS_FAIL;
	}

    #elif _OPERATING_SYSTEM == _OS_FREERTOS
	message = MSG_AUDIO_ENCODE_START;
        err = xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
	if(err != pdPASS) {
		return STATUS_FAIL;
	}

    #endif
#endif

	//wait 10 second
	for(i=0; i<1000; i++) {
		status = audio_record_get_status();
		if(status == C_START_RECORD) {
			return STATUS_OK;
		}

		if(status == C_START_FAIL) {
			return STATUS_FAIL;
		}

        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		OSTimeDly(1);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                vTaskDelay(1);
            #endif
        #endif
	}

	if(i == 1000) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

//===============================================================================================
// 	audio record task start
//	parameter:	none.
//	return:		status.
//===============================================================================================
INT32S adc_record_task_stop(void)
{
	INT8U err;
	INT32S i, status;
	INT32U message;

#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
	err = OSQPost(aud_enc_reqQ, (void *)MSG_AUDIO_ENCODE_STOPING);
	if(err != OS_NO_ERR) {
		return STATUS_FAIL;
	}
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
	message = MSG_AUDIO_ENCODE_STOPING;
        err = xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
	if(err != pdPASS) {
		return STATUS_FAIL;
	}

    #endif
#endif

	//wait 10 second
	//m-for(i=0; i<1000; i++) {
	while(1) {
		status = audio_record_get_status();
		if(status == C_STOP_RECORD) {
			return STATUS_OK;
		}

        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		OSTimeDly(1);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                vTaskDelay(1);
            #endif
        #endif
	}

	if(i == 1000) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

//===============================================================================================
// 	audio_record_set_status
//	parameter:	status
//	return:
//===============================================================================================
void audio_record_set_status(INT32U status)
{
	pAudio_Encode_Para->Status = status;
}

//===============================================================================================
// 	audio_record_get_status
//	parameter:
//	return:		status.
//===============================================================================================
INT32U audio_record_get_status(void)
{
	return pAudio_Encode_Para->Status;
}

//===============================================================================================
// 	audio_record_set_source_type
//	parameter:	type = GP_FS/user_define
//	return:
//===============================================================================================
void audio_record_set_source_type(INT32U type)
{
	pAudio_Encode_Para->SourceType = type;
}

//===============================================================================================
// 	audio_record_get_source_type
//	parameter:
//	return:		type.
//===============================================================================================
INT32U audio_record_get_source_type(void)
{
	return pAudio_Encode_Para->SourceType;
}

//===============================================================================================
// 	audio_record_set_file_handle_free_size
//	parameter:	file_handle = wave file handle
//	return:		status.
//===============================================================================================
INT64U audio_record_set_file_handle_free_size(INT16S file_handle)
{
	INT8U  fs_disk;
	INT64U disk_free = 0;

	if(file_handle >= 0) {
#if 1 //m-
		fs_disk = GetDiskOfFile(file_handle);
		disk_free = vfsFreeSpace(fs_disk);
		pAudio_Encode_Para->FileHandle = file_handle;
		pAudio_Encode_Para->disk_free_size = disk_free - 10*1024; //reserved 10K
#else
		disk_free = 100*1024*1024;
		pAudio_Encode_Para->FileHandle = file_handle;
		pAudio_Encode_Para->disk_free_size = disk_free - 10*1024; //reserved 10K
#endif
	} else {
		pAudio_Encode_Para->FileHandle = -1;
		pAudio_Encode_Para->disk_free_size = 0;
	}
	return disk_free;
}

//===============================================================================================
// 	audio_record_set_info
//	parameter:	audio_format = format
//				sample_rate =
//				bit_rate =
//				channel =
//	return:		status.
//===============================================================================================
void audio_record_set_info(INT32U audio_format, INT32U sample_rate, INT32U bit_rate)
{
	pAudio_Encode_Para->AudioFormat = audio_format;
	pAudio_Encode_Para->SampleRate = sample_rate;
	pAudio_Encode_Para->BitRate = bit_rate;
}

void audio_record_set_channel(INT8U device, INT8U channel)
{
	pAudio_Encode_Para->InputDevice = device;
	pAudio_Encode_Para->Channel = channel; //1, mono, 2, stereo
}

INT8U audio_record_set_CutLastFrame(INT8U flag)
{
    INT8U old_flag = pAudio_Encode_Para->CutLastFrame;
	pAudio_Encode_Para->CutLastFrame = flag;
	return old_flag;
}

//===============================================================================================
// 	audio_record_set_down_sample
//	parameter:	b_down_sample = enable/disable
//				ratio = 2~4
//	return:		status.
//===============================================================================================
INT32S audio_record_set_down_sample(BOOLEAN b_down_sample, INT8U ratio)
{
#if	APP_DOWN_SAMPLE_EN
	pAudio_Encode_Para->bEnableDownSample = b_down_sample;
	pAudio_Encode_Para->DownsampleFactor = ratio;
	return AUD_RECORD_STATUS_OK;
#else
	return AUD_RECORD_STATUS_ERR;
#endif
}

//===============================================================================================
// 	pcm_energy_detect
//	parameter:	none.
//	return:		status.
//===============================================================================================
INT32S pcm_energy_detect(INT16S* buffer_addr, INT32U pcm_size)
{
	INT16S temp;
	INT16U avg_value;
	INT64U temp_total;
	INT32S i, cnt, local_cnt;

	temp_total = 0;
	cnt = 0;
	for (i = 0; i < pcm_size; i++) {
		temp = abs(*(buffer_addr + i));
		temp_total += (INT64U)temp;
		if (temp > PCM_LOCAL_THR) {
			cnt += 1;
		}
	}

	// average
	avg_value = (INT64U)temp_total / pcm_size;
	DBG_PRINT("temp_total = 0x%x\r\n", temp_total);
	DBG_PRINT("avg_value = 0x%x, cnt = %d\r\n", avg_value, cnt);

	//samples above Local_Thr to judge as active
	local_cnt = pcm_size / 6;

	if((avg_value > PCM_GLOBAL_THR)||(cnt>local_cnt)) {
		return 0;	//active
	} else {
		return 1; 	//not active
	}
}

//===============================================================================================
// 	audio task write entry
//===============================================================================================
static void adc_record_write_entry(void *param)

{
	INT8U err;
	INT32U msg;
	INT32S ret;
	INT32U RI, WI;
	volatile INT8U *addr;
	Audio_Encode_Para *pAudRec = (Audio_Encode_Para *)param;
	DBG_PRINT("record_write_entry\r\n");

	while(1)
	{
#if _OPERATING_SYSTEM != _OS_NONE
     #if _OPERATING_SYSTEM == _OS_UCOS2
		msg = (AUDIO_RECORD_ENUM)(int)OSQPend(aud_enc_writeQ, 0, &err);
		if(err != OS_NO_ERR) {
			continue;
		}
     #elif _OPERATING_SYSTEM == _OS_FREERTOS
        if(xQueueReceive(aud_enc_writeQ, &msg, portMAX_DELAY) != pdPASS)
        {
			continue;
		}
    #endif
#endif
		if(msg==MSG_AUDIO_ENCODE_WRITE)
		{
            WI = (pAudRec->write_index >> RECORD_CLUSTER_SHIFT) << RECORD_CLUSTER_SHIFT; // 64KB multiple
	        RI = pAudRec->read_index;
	        err = TRUE;
			while(RI!=WI)
        	{
        	    addr = pAudRec->Bit_Stream_Buffer + RI;
        		if(pAudio_Encode_Para->SourceType == C_GP_FS) {
				    ret = fs_write(pAudRec->FileHandle, (INT32U)addr, RECORD_CLUSTER_SIZE);
    			} else {
    				ret = audio_encode_data_write(0, (INT32U)addr, RECORD_CLUSTER_SIZE);
    			}
        		if(ret != RECORD_CLUSTER_SIZE)
        		{
        		   err = FALSE;
        		   break;
        		}
        		RI += RECORD_CLUSTER_SIZE;
        		if(RI>= C_BS_BUFFER_SIZE ) RI -= C_BS_BUFFER_SIZE;

        		vTaskSuspendAll();
        		pAudRec->read_index = RI;
        		xTaskResumeAll();
        	}
        	if(FALSE == err)
        	{
        	    continue;
        	}
            pAudRec->MsgRx ++;
          #if 0
		    {
				DBG_PRINT("RING %6d %6d\r\n",pAudRec->write_index, pAudRec->read_index);
			};
	      #endif
		}
		else if(msg==MSG_AUDIO_ENCODE_WRITE_STOP)
		{
		    addr = pAudRec->Bit_Stream_Buffer + pAudRec->read_index;
            if(pAudRec->read_index > pAudRec->write_index)
            {
                WI = C_BS_BUFFER_SIZE - pAudRec->read_index + pAudRec->write_index;
            }
            else
            {
                WI = pAudRec->write_index - pAudRec->read_index;
            }
            RI = pAudRec->read_index;
            RI += WI;
    		if(RI>= C_BS_BUFFER_SIZE ) RI -= C_BS_BUFFER_SIZE;


    		vTaskSuspendAll();
    		pAudRec->read_index = RI;
    		xTaskResumeAll();
    		if(pAudio_Encode_Para->SourceType == C_GP_FS) {
			    ret = fs_write(pAudRec->FileHandle, (INT32U)addr, WI);
			} else {
				ret = audio_encode_data_write(0, (INT32U)addr, WI);
			}
            if(ret != WI)
            {
                msg = AUD_WRITE_ACK_FAIL;
            }
            else
            {
                msg = AUD_WRITE_ACK_OK;
            }
        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
            OSMboxPost(aud_write_task_ack_m, (void*)&msg);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
             xQueueSend( (xQueueHandle) aud_write_task_ack_m, (void *) &msg, portMAX_DELAY);
            #endif
        #endif

		}
		else if(MSG_AUDIO_ENCODE_EXIT == msg)
        {
            osThreadId id;
            id = osThreadGetId();
    		osThreadTerminate(id);
        }

	}
}


//===============================================================================================
// 	audio task main entry
//===============================================================================================
void adc_record_entry(void *param)
{
	INT8U   dac_flag;
	INT8U   adc_bufidx;
	INT8U   mic_bufidx;
	INT8U   buffer_index;
	INT8U   err, device, channel;
	INT8U   EnvDet_status = C_ENVDET_NO_DETECT;
	INT16U  mask, t, t2;
	INT16U  SampleRate;
	INT32U  msg_id;
	INT32S  i, nRet;
	INT32S  ret;
	INT32U  ready_addr, free_addr, pcm_addr;
	INT16U  *ptr, *ptr1, *ptr2, *temp;
	INT32U  adc_ready_addr, adc_free_addr, adc_pcm_addr, adc_prev_addr;
	INT32U  mic_ready_addr, mic_free_addr, mic_pcm_addr;
	INT32U  encode_addr, aud_dst_buffer;
	void    *hProc;
	INT32S  (*pfn_audio_encode_stop)(void);
	INT32S  (*pfn_audio_encode_once)(void *workmem, const short* buffer_addr, int cwlen);
	Audio_Encode_Para *pAudRec = (Audio_Encode_Para *)param;
    INT32U message;
    INT32S error = 0;

#if 0
	INT32U  L_pcm_ptr, R_pcm_ptr;
#endif

	while(1)
	{
#if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		msg_id = (INT32U) OSQPend(aud_enc_reqQ, 0, &err);
		if(err != OS_NO_ERR) {
			continue;
		}
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                err = (INT32S) xQueueReceive(aud_enc_reqQ, &msg_id, portMAX_DELAY);
                if(err != pdPASS) {
			continue;
		}
            #endif
#endif

		switch(msg_id)
		{
		case MSG_ADC_DMA_DONE:
			switch(device)
			{
			case ADC_LINE_IN:
			case BUILD_IN_MIC:
			case BUILD_LINEIN:
                //DEBUG_MSG("PGA = %x \r\n",drv_l1_adc_dagc_current_pga_gain());
				pcm_addr = ready_addr;
				ready_addr = free_addr;
				buffer_index++;
				if(buffer_index >= C_AUD_PCM_BUFFER_NO) {
					buffer_index = 0;
				}

				if(device == ADC_LINE_IN) {
					free_addr = pAudRec->aud_pcm_buffer[buffer_index];
				} else {
					free_addr =  pAudRec->mic_pcm_buffer[buffer_index];
				}

				if(pAudRec->Status == C_STOP_RECORDING) {
					// check dma is done and stop
					if(aud_adc_dma_status_get(device) == 0) {
						dac_flag = 3;
                                #if _OPERATING_SYSTEM != _OS_NONE
                                    #if _OPERATING_SYSTEM == _OS_UCOS2
						OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_STOP);
                                    #elif _OPERATING_SYSTEM == _OS_FREERTOS
					    message = MSG_AUDIO_ENCODE_STOP;
                                                xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
                                    #endif
                                #endif
					}
					if(pAudio_Encode_Para->CutLastFrame)
					{
                        continue;
					}
				} else {
					// set dma buffer
					aud_adc_double_buffer_set((INT16U *)free_addr, pAudRec->PCMInFrameSize, device);
				}

				// invalid cache
				cache_invalid_range(pcm_addr, pAudRec->PCMInFrameSize << 1);

				// unsigned to signed
				ptr = (INT16U*)pcm_addr;
				for(i=0; i<pAudRec->PCMInFrameSize; i++)
				{
					t = *ptr;
				#if SOFTWARE_UNSIGNED_TO_SIGNED == 1
                    t ^= mask;
                #endif
				#if APP_LPF_ENABLE == 1
					t = (INT16U)LPF_process(t);
				#endif
					*ptr++ = t;
				}

				encode_addr = pcm_addr;
				if(C_ENVDET_NO_DETECT != EnvDet_status)
				{
					if(C_ENVDET_DETECT_RESUME == EnvDet_status )
		            {
		                EnvDet_status= C_ENVDET_DETECTING;
		                continue;
		            }
		            else if(C_ENVDET_DETECT_PAUSE == EnvDet_status )
		            {
		                continue;
		            }

					ret = UpdateEnvelop((const short*)pcm_addr, pAudio_Encode_Para->PCMInFrameSize);
		            if(0 == ret)
		            {
		               adc_prev_addr = pcm_addr;
		               DBG_PRINT("*");
		               if(C_ENVDET_DETECTED == EnvDet_status)
	               	   {
	                   	   EnvDet_status = C_ENVDET_DETECT_END;
		                   DEBUG_MSG("MSG_AUDIO_ENCODE_STOP!!!\r\n");
		                   #if _OPERATING_SYSTEM != _OS_NONE
                                    #if _OPERATING_SYSTEM == _OS_UCOS2
                           OSQPost(aud_enc_reqQ, (void *)MSG_AUDIO_ENCODE_STOP);
                                    #elif _OPERATING_SYSTEM == _OS_FREERTOS
                           message = MSG_AUDIO_ENCODE_STOP;
                                        xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
                                    #endif
                                #endif

		               }
		               else
		               {
		                   continue;
		               }
		            }
		            else
		            {
                        DBG_PRINT("$");
		                 //DBG_PRINT("ENVDET !\r\n");
                       #if 0
		                 if((C_ENVDET_DETECTING == EnvDet_status) && (0 != adc_prev_addr))
		                 {
		                     DEBUG_MSG("adc_prev_addr = 0x%x\r\n",adc_prev_addr);
		                     DEBUG_MSG(".");

							//#if AUDIO_INPUT_SRC != DOUBLE_LINEIN
								nRet = pfn_audio_encode_once(hProc, (const short*)adc_prev_addr, pAudio_Encode_Para->PCMInFrameSize);
							//#endif

							if(nRet < 0)
							{
								OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_ERR);
								continue;
							}

							//check storage is full
							if(pAudio_Encode_Para->SourceType == C_GP_FS)
							{
								if(pAudio_Encode_Para->FileLenth >= pAudio_Encode_Para->disk_free_size)
									OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_STOPING);
							}
		                 }
                        #endif
		                 EnvDet_status = C_ENVDET_DETECTED;
		            }
				}
				break;

			#if GPY0050_IN_EN == 1
			case GPY0050_IN:
				pcm_addr = pAudRec->ready_buffer;
				if(pAudRec->Status == C_STOP_RECORDING) {
					dac_flag = 3;
                                #if _OPERATING_SYSTEM != _OS_NONE
                                    #if _OPERATING_SYSTEM == _OS_UCOS2
					OSQPost(aud_enc_reqQ, (void *)MSG_AUDIO_ENCODE_STOP);
                                    #elif _OPERATING_SYSTEM == _OS_FREERTOS
				     message = MSG_AUDIO_ENCODE_STOP;
                                        xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
                                    #endif
                                #endif
				}

				// invalid cache
				cache_invalid_range(pcm_addr, pAudRec->PCMInFrameSize << 1);

				// unsigned to signed
				ptr = (INT16U*)pcm_addr;
				for(i=0; i<pAudRec->PCMInFrameSize; i++)
				{
					t = *ptr;
					t ^= mask;
				#if APP_LPF_ENABLE == 1
					t = (INT16U)LPF_process(t);
				#endif
					*ptr++ = t;
				}

				encode_addr = pcm_addr;
				break;
			#endif

			case DOUBLE_LINEIN:
                        #if _OPERATING_SYSTEM != _OS_NONE
                            #if _OPERATING_SYSTEM == _OS_UCOS2
				msg_id = (INT32U) OSQPend(aud_enc_reqQ2, 0, &err);
                            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                                err = (INT32S) xQueueReceive(aud_enc_reqQ2, &msg_id, portMAX_DELAY);
                            #endif
                        #endif

				if (msg_id != MSG_ADC_DMA_DONE) {
					DBG_PRINT("MIC DMA error !\r\n");
					continue;
				}

				adc_pcm_addr = adc_ready_addr;
				adc_ready_addr = adc_free_addr;
				mic_pcm_addr = mic_ready_addr;
				mic_ready_addr = mic_free_addr;
				adc_bufidx++;
				mic_bufidx++;
				if(adc_bufidx >= C_AUD_PCM_BUFFER_NO) {
					adc_bufidx = 0;
				}

				if(mic_bufidx >= C_AUD_PCM_BUFFER_NO) {
					mic_bufidx = 0;
				}

				adc_free_addr = pAudRec->aud_pcm_buffer[adc_bufidx];
				mic_free_addr = pAudRec->mic_pcm_buffer[mic_bufidx];
				if(pAudRec->Status == C_STOP_RECORDING) {
					// check dma is done and stop
					if ((aud_adc_dma_status_get(ADC_LINE_IN) == 0) && (aud_adc_dma_status_get(BUILD_IN_MIC) == 0)) {
						dac_flag = 3;
                                        #if _OPERATING_SYSTEM != _OS_NONE
                                            #if _OPERATING_SYSTEM == _OS_UCOS2
						OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_STOP);
                                            #elif _OPERATING_SYSTEM == _OS_FREERTOS
					    message = MSG_AUDIO_ENCODE_STOP;
                                                xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
                                            #endif
                                        #endif
					}
				} else {
					// set dma buffer
					aud_adc_double_buffer_set((INT16U *)adc_free_addr, pAudRec->PCMInFrameSize>>1, ADC_LINE_IN);
					aud_adc_double_buffer_set((INT16U *)mic_free_addr, pAudRec->PCMInFrameSize>>1, BUILD_IN_MIC);
				}

				// invalid cache
				cache_invalid_range(adc_pcm_addr, pAudRec->PCMInFrameSize);
				cache_invalid_range(mic_pcm_addr, pAudRec->PCMInFrameSize);

				// unsigned to signed
				ptr1 = (INT16U*)adc_pcm_addr;
				ptr2 = (INT16U*)mic_pcm_addr;
				temp = (INT16U*)aud_dst_buffer;
				for (i=0; i<(pAudRec->PCMInFrameSize>>1); i++)
				{
					t = *ptr1++;
					t2 = *ptr2++;
				#if ((defined MCU_VERSION) && (MCU_VERSION < GPL327XX))
					t ^= mask;
					t2 ^= mask;
				#endif
				#if APP_LPF_ENABLE == 1
					t = (INT16U)LPF_process(t);
					t2 = (INT16U)LPF_process(t2);
				#endif
					*temp++ = t;
					*temp++ = t2;
				}

				encode_addr = aud_dst_buffer;
				break;
			}

		#if 0
			write(L_pcm_ptr, (INT32U)adc_pcm_addr, pAudRec->PCMInFrameSize);
			write(R_pcm_ptr, (INT32U)mic_pcm_addr, pAudRec->PCMInFrameSize);
		#endif
		#if RECORD_WITH_DAC_OUT_EN == 1
			if(dac_flag == 0) {
				dac_flag = 1;
				drv_l1_dac_cha_dbf_put((INT16S *)encode_addr, pAudRec->PCMInFrameSize, aud_enc_reqQ3);
			} else if(dac_flag == 1) {
				dac_flag = 2;
				// mute
				nRet = drv_l1_dac_pga_get();
				drv_l1_dac_pga_set(0);
				// dac start
				drv_l1_dac_cha_dbf_set((INT16S *)encode_addr, pAudRec->PCMInFrameSize);
				dac_out_start(channel, SampleRate);
				// volume up
				for (i=0; i<=nRet; i++) {
					drv_l1_dac_pga_set(i);
				}
			} else if(dac_flag == 2){
                        #if _OPERATING_SYSTEM != _OS_NONE
                            #if _OPERATING_SYSTEM == _OS_UCOS2
				msg_id = (INT32U) OSQPend(aud_enc_reqQ3, 0, &err);
                            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                                error = (INT32S) xQueueReceive(aud_enc_reqQ3, &msg_id, portMAX_DELAY);
                            #endif
                        #endif
				if (msg_id == MSG_ADC_DMA_DONE) {
					drv_l1_dac_cha_dbf_set((INT16S *)encode_addr, pAudRec->PCMInFrameSize);
				}
			} else {
				dac_flag = 0xFF;
			}
		#endif

			// energy detect
			//pcm_energy_detect((INT16S *)encode_addr, pAudio_Encode_Para->PCMInFrameSize);

			// encode pcm wave
			DBG_PRINT(".");
			nRet = pfn_audio_encode_once(hProc, (const short*)encode_addr, pAudRec->PCMInFrameSize);
			if(nRet < 0) {
                        #if _OPERATING_SYSTEM != _OS_NONE
                            #if _OPERATING_SYSTEM == _OS_UCOS2
				OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_ERR);
                            #elif _OPERATING_SYSTEM == _OS_FREERTOS
			     message = MSG_AUDIO_ENCODE_ERR;
                                xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
                            #endif
                        #endif
				continue;
			}

			// check storage is full
			if(pAudRec->SourceType == C_GP_FS) {
				if(pAudRec->FileLenth >= pAudRec->disk_free_size) {
                                #if _OPERATING_SYSTEM != _OS_NONE
                                    #if _OPERATING_SYSTEM == _OS_UCOS2
					OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_STOPING);
                                    #elif _OPERATING_SYSTEM == _OS_FREERTOS
				    message = MSG_AUDIO_ENCODE_STOPING;
                                        xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
                                    #endif
                                #endif
				}
			}
			break;
        case MSG_AUDIO_ENVELOP_DET_START:
            EnvDet_Initial();
			adc_prev_addr = 0;
			EnvDet_status= C_ENVDET_DETECTING;
		    Set_MICSensitive_Time(0xA6, 0x6000);
		    Set_MICSensitive_Level(0x1A0, 0x0280, 0x01B0);
		    EnvDet_Enable(1);

		case MSG_AUDIO_ENCODE_START:
            if(MSG_AUDIO_ENCODE_START == msg_id)
			{
			    EnvDet_status= C_ENVDET_NO_DETECT;
			}

			DEBUG_MSG("[MSG_AUDIO_ENCODE_START]\r\n");
			dac_flag = 0;
			hProc = 0;
			pAudRec->ring_buffer = 0;
			pAudio_Encode_Para->CutLastFrame = 0;
			pAudRec->read_index = 0;
			pAudRec->write_index = 0;
			pAudRec->MsgTx = 0;
			pAudRec->MsgRx = 0;
			mask = 0x8000;
		#if 0
			L_pcm_ptr = open("L_pcm.pcm", O_WRONLY|O_CREAT|O_TRUNC);
			R_pcm_ptr = open("R_pcm.pcm", O_WRONLY|O_CREAT|O_TRUNC);
		#endif
			switch(pAudRec->AudioFormat)
			{
			#if APP_A1800_ENCODE_EN
			case WAVE_FORMAT_A1800:
				nRet = a1800_encode_start();
				pfn_audio_encode_once = a1800_encode_once;
				pfn_audio_encode_stop = a1800_encode_stop;
				break;
			#endif

			#if 1
			case WAVE_FORMAT_PCM:
				nRet = wave_encode_start();
				pfn_audio_encode_once = wave_encode_once;
				pfn_audio_encode_stop = wave_encode_stop;
				break;
			#endif

			#if	APP_WAV_CODEC_EN
			//case WAVE_FORMAT_PCM:
			case WAVE_FORMAT_IMA_ADPCM:
			case WAVE_FORMAT_ADPCM:
			case WAVE_FORMAT_ALAW:
			case WAVE_FORMAT_MULAW:
				nRet = wave_encode_lib_start(pAudRec->AudioFormat);
				pfn_audio_encode_once = wave_encode_lib_once;
				pfn_audio_encode_stop = wave_encode_lib_stop;
				break;
			#endif

			#if APP_MP3_ENCODE_EN
			case WAVE_FORMAT_MP3:
				nRet = mp3_encode_start();
				pfn_audio_encode_once = mp3_encode_once;
				pfn_audio_encode_stop = mp3_encode_stop;
				break;
			#endif

			#if APP_VR_ENCODE_EN
			case PCM_FORMAT_VR:
				nRet = vr_encode_start();
				pfn_audio_encode_once = vr_encode_once;
				pfn_audio_encode_stop = vr_encode_stop;
				break;
			#endif
			#if APP_AUDIO_STREAM_EN
			case PCM_FORMAT_AUDIO_STREAM:
				nRet = audio_stream_encode_start();
				pfn_audio_encode_once = audio_stream_encode_once;
				pfn_audio_encode_stop = audio_stream_encode_stop;
				break;
			#endif

			default:
				DEBUG_MSG("encode fmt err.\r\n");
				goto __START_FAIL;
			}

			if(nRet < 0) {
				DEBUG_MSG("encode start fail.\r\n");
				goto __START_FAIL;
			}

		#if	APP_DOWN_SAMPLE_EN
			if(pAudRec->bEnableDownSample) {
				if(pAudRec->DownsampleFactor * pAudRec->SampleRate > 48000) {
					pAudRec->DownsampleFactor = 48000 / pAudRec->SampleRate;
				}

				if(pAudRec->DownsampleFactor >= 2) {
					pAudRec->DownSampleWorkMem = DownSample_Create(pAudRec->PCMInFrameSize, pAudRec->DownsampleFactor);

					DownSample_Link(pAudRec->DownSampleWorkMem,
									NULL,
									pfn_audio_encode_once,
									pAudRec->SampleRate,
									pAudRec->Channel,
									pAudRec->DownsampleFactor);

					hProc = pAudRec->DownSampleWorkMem;
					pfn_audio_encode_once = DownSample_PutData;
					pAudRec->SampleRate = DownSample_GetSampleRate(hProc);
					pAudRec->Channel = DownSample_GetChannel(hProc);
					pAudRec->PCMInFrameSize *=  pAudRec->DownsampleFactor;
				}
			}
		#endif

		#if	APP_LPF_ENABLE == 1
			LPF_init(pAudRec->SampleRate, 3);
		#endif

			device = pAudRec->InputDevice;
			channel = pAudRec->Channel;
			SampleRate = pAudRec->SampleRate;
			DBG_PRINT("Dev=0x%x, Ch=%d, SR=%d\r\n", device, channel, SampleRate);
			DBG_PRINT("PCMInFrameSize=%d\r\n", pAudRec->PCMInFrameSize);

			// start dma and adc
			switch(device)
			{
			case ADC_LINE_IN:
				if(channel != 1) {
					DEBUG_MSG("ChErr.\r\n");
					goto __START_FAIL;
				}

				nRet = adc_memory_allocate(pAudRec->PCMInFrameSize<<1, ADC_LINE_IN);
				if(nRet < 0) {
					DEBUG_MSG("alocate memory fail.\r\n");
					goto __START_FAIL;
				}

				ready_addr = pAudRec->aud_pcm_buffer[0];
				free_addr = pAudRec->aud_pcm_buffer[1];
				buffer_index = 1;
				aud_adc_double_buffer_put((INT16U*)ready_addr, pAudRec->PCMInFrameSize, ADC_LINE_IN, aud_enc_reqQ);
				aud_adc_double_buffer_set((INT16U*)free_addr, pAudRec->PCMInFrameSize, ADC_LINE_IN);
				adc_hardware_start(ADC_LINE_IN, 1, SampleRate);
				break;

			case BUILD_IN_MIC:
				if(channel != 1) {
					DEBUG_MSG("ChErr.\r\n");
					goto __START_FAIL;
				}

				nRet = adc_memory_allocate(pAudRec->PCMInFrameSize<<1, BUILD_IN_MIC);
				if(nRet < 0) {
					DEBUG_MSG("alocate memory fail.\r\n");
					goto __START_FAIL;
				}

				ready_addr = pAudRec->mic_pcm_buffer[0];
				free_addr = pAudRec->mic_pcm_buffer[1];
				buffer_index = 1;
				aud_adc_double_buffer_put((INT16U*)ready_addr, pAudRec->PCMInFrameSize, BUILD_IN_MIC, aud_enc_reqQ);
				aud_adc_double_buffer_set((INT16U*)free_addr, pAudRec->PCMInFrameSize, BUILD_IN_MIC);
				adc_hardware_start(BUILD_IN_MIC, 1, SampleRate);
				break;

			#if GPY0050_IN_EN == 1
			case GPY0050_IN:
				if(channel != 1) {
					DEBUG_MSG("ChErr.\r\n");
					goto __START_FAIL;
				}

				nRet = adc_memory_allocate(pAudRec->PCMInFrameSize<<1, ADC_LINE_IN);
				if(nRet < 0) {
					DEBUG_MSG("alocate memory fail.\r\n");
					goto __START_FAIL;
				}

				pAudRec->buffer_index = 0;
				adc_hardware_start(BUILD_IN_MIC, 1, SampleRate);
				break;
			#endif

			case DOUBLE_LINEIN:
				if(channel != 2) {
					DEBUG_MSG("ChErr.\r\n");
					goto __START_FAIL;
				}

				nRet = adc_memory_allocate(pAudRec->PCMInFrameSize, ADC_LINE_IN|BUILD_IN_MIC);
				if(nRet < 0) {
					DEBUG_MSG("alocate memory fail.\r\n");
					goto __START_FAIL;
				}

				aud_dst_buffer = (INT32U)gp_malloc_align(pAudRec->PCMInFrameSize<<1, 32);
				if(aud_dst_buffer == 0) {
					DEBUG_MSG("aud_dst_buffer fail.\r\n");
					goto __START_FAIL;
				}

				adc_ready_addr = pAudRec->aud_pcm_buffer[0];
				adc_free_addr = pAudRec->aud_pcm_buffer[1];
				mic_ready_addr = pAudRec->mic_pcm_buffer[0];
				mic_free_addr = pAudRec->mic_pcm_buffer[1];
				adc_bufidx = 1;
				mic_bufidx = 1;
				aud_adc_double_buffer_put((INT16U*)adc_ready_addr, pAudRec->PCMInFrameSize>>1, ADC_LINE_IN, aud_enc_reqQ);
				aud_adc_double_buffer_set((INT16U*)adc_free_addr, pAudRec->PCMInFrameSize>>1, ADC_LINE_IN);
				aud_adc_double_buffer_put((INT16U*)mic_ready_addr, pAudRec->PCMInFrameSize>>1, BUILD_IN_MIC, aud_enc_reqQ2);
				aud_adc_double_buffer_set((INT16U*)mic_free_addr, pAudRec->PCMInFrameSize>>1, BUILD_IN_MIC);
				adc_hardware_start(DOUBLE_LINEIN, 2, SampleRate);
				break;
			case BUILD_LINEIN:
				if(channel != 2) {
					DEBUG_MSG("ChErr.\r\n");
					goto __START_FAIL;
				}

				nRet = adc_memory_allocate(pAudRec->PCMInFrameSize<<1, BUILD_LINEIN);
				if(nRet < 0) {
					DEBUG_MSG("alocate memory fail.\r\n");
					goto __START_FAIL;
				}

				ready_addr = pAudRec->mic_pcm_buffer[0];
				free_addr = pAudRec->mic_pcm_buffer[1];
				buffer_index = 1;
				DBG_PRINT("LINEIN length %d \r\n", pAudRec->PCMInFrameSize);
				aud_adc_double_buffer_put((INT16U*)ready_addr, pAudRec->PCMInFrameSize, BUILD_LINEIN, aud_enc_reqQ);
				aud_adc_double_buffer_set((INT16U*)free_addr, pAudRec->PCMInFrameSize, BUILD_LINEIN);
				adc_hardware_start(BUILD_LINEIN, 2, SampleRate);
				break;


			default:
				DEBUG_MSG("InputDeviceErr.\r\n");
				goto __START_FAIL;
			}
			pAudRec->Status = C_START_RECORD;
			break;

__START_FAIL:
			DEBUG_MSG("AudioEncodeStartFail!!!\r\n");
			adc_work_mem_free();
			adc_memory_free();
			pAudRec->Status = C_START_FAIL;
			break;

		case MSG_AUDIO_ENCODE_STOPING:
			DEBUG_MSG("[MSG_AUDIO_ENCODE_STOPING]\r\n");
			pAudRec->Status = C_STOP_RECORDING;
			break;

		case MSG_AUDIO_ENCODE_STOP:
			DEBUG_MSG("[MSG_AUDIO_ENCODE_STOP]\r\n");
			adc_hardware_stop(device, channel);
			pfn_audio_encode_stop();
			adc_memory_free();
			if(EnvDet_status != C_ENVDET_NO_DETECT)
			{
                EnvDet_status = C_ENVDET_NO_DETECT;
                EnvDet_Enable(0);
			}
			switch(device)
			{
			case ADC_LINE_IN:
				aud_adc_double_buffer_free(ADC_LINE_IN);
				break;

			case BUILD_IN_MIC:
				aud_adc_double_buffer_free(BUILD_IN_MIC);
				break;

			case DOUBLE_LINEIN:
				aud_adc_double_buffer_free(ADC_LINE_IN);
				aud_adc_double_buffer_free(BUILD_IN_MIC);
				if (aud_dst_buffer) {
					gp_free((void *)aud_dst_buffer);
					aud_dst_buffer = 0;
				}
					break;
				case BUILD_LINEIN:
					aud_adc_double_buffer_free(BUILD_LINEIN);
				break;
			}

		#if APP_DOWN_SAMPLE_EN
			if(pAudRec->bEnableDownSample) {
				DownSample_Del(pAudRec->DownSampleWorkMem);
				pAudRec->bEnableDownSample = FALSE;
				pAudRec->DownsampleFactor = 0;
			}
		#endif
		#if 0
			close(L_pcm_ptr);
			close(R_pcm_ptr);
		#endif
		#if RECORD_WITH_DAC_OUT_EN == 1
			// wait dac dma finish
			while(drv_l1_dac_dbf_status_get() || drv_l1_dac_dma_status_get()) {
                      #if _OPERATING_SYSTEM != _OS_NONE
                          #if _OPERATING_SYSTEM == _OS_UCOS2
				OSTimeDly(1);
                          #elif _OPERATING_SYSTEM == _OS_FREERTOS
                                vTaskDelay(1);
                          #endif
                      #endif
                        }

                        drv_l1_dac_dbf_free();
                        dac_out_stop(channel);

                      #if _OPERATING_SYSTEM != _OS_NONE
                        #if _OPERATING_SYSTEM == _OS_UCOS2
                          OSQFlush(aud_enc_reqQ3);
                        #elif _OPERATING_SYSTEM == _OS_FREERTOS
                          xQueueReset(aud_enc_reqQ3);
                        #endif
                      #endif


                #endif

                #if _OPERATING_SYSTEM != _OS_NONE
                    #if _OPERATING_SYSTEM == _OS_UCOS2
                        OSQFlush(aud_enc_reqQ);
			OSQFlush(aud_enc_reqQ2);
                    #elif _OPERATING_SYSTEM == _OS_FREERTOS
                        xQueueReset(aud_enc_reqQ);
                        xQueueReset(aud_enc_reqQ2);
                    #endif
                #endif


			pAudRec->Status = C_STOP_RECORD;
			break;

		case MSG_AUDIO_ENCODE_ERR:
			DEBUG_MSG("[MSG_AUDIO_ENCODE_ERR]\r\n");
                #if _OPERATING_SYSTEM != _OS_NONE
                    #if _OPERATING_SYSTEM == _OS_UCOS2
			OSQPost(aud_enc_reqQ, (void *) MSG_AUDIO_ENCODE_STOPING);
                    #elif _OPERATING_SYSTEM == _OS_FREERTOS
		      message = MSG_AUDIO_ENCODE_STOPING;
                        xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
                    #endif
                #endif
			break;
        case MSG_AUDIO_ENCODE_EXIT:
        {
            osThreadId id;
            id = osThreadGetId();
    		osThreadTerminate(id);
        }
        break;
		}
	}
}

//===============================================================================================
// 	aud_adc_double_buffer_put
//	parameter:	data = buffer_addr.
//				len  = size in word
//				os_q = OS _EVENT
//	return:		status.
//===============================================================================================
#if _OPERATING_SYSTEM == _OS_UCOS2
static INT32S aud_adc_double_buffer_put(INT16U *data,INT32U cwlen, INT8U aud_in, OS_EVENT *os_q)
#elif _OPERATING_SYSTEM == _OS_FREERTOS
static INT32S aud_adc_double_buffer_put(INT16U *data,INT32U cwlen, INT8U aud_in, xQueueHandle os_q)
#endif
{
	INT32S status;
	DMA_STRUCT *pDma;

	if (aud_in == ADC_LINE_IN) {
		pDma = &pAudio_Encode_Para->adc_dma_dbf;
		pDma->s_addr = (INT32U) P_ADC_ASADC_DATA;
		pDma->t_addr = (INT32U) data;
		pDma->width = DMA_DATA_WIDTH_2BYTE;
		pDma->count = (INT32U) cwlen;
		pDma->notify = NULL;
		pDma->timeout = 0;
	} else if ((aud_in == BUILD_IN_MIC)||(aud_in == BUILD_LINEIN)) {
		pDma = &pAudio_Encode_Para->mic_dma_dbf;
		pDma->s_addr = (INT32U) P_I2SRX_DATA;
		pDma->t_addr = (INT32U) data;
		pDma->width = DMA_DATA_WIDTH_4BYTE;
		pDma->count = (INT32U) cwlen/2;
		pDma->notify = NULL;
		pDma->timeout = 0;
	} else {
		return STATUS_FAIL;
	}

	status = drv_l1_dma_transfer_with_double_buf(pDma, os_q);
	if (status != 0) {
		return status;
	}

	return STATUS_OK;
}

//===============================================================================================
// 	aud_adc_double_buffer_set
//	parameter:	data = buffer_addr.
//				len  = size in word
//	return:		status.
//===============================================================================================
static INT32U aud_adc_double_buffer_set(INT16U *data, INT32U cwlen, INT8U aud_in)
{
	INT32S status;
	DMA_STRUCT *pDma;

	if (aud_in == ADC_LINE_IN) {
		pDma = &pAudio_Encode_Para->adc_dma_dbf;
		pDma->s_addr = (INT32U) P_ADC_ASADC_DATA;
		pDma->t_addr = (INT32U) data;
		pDma->width = DMA_DATA_WIDTH_2BYTE;
		pDma->count = (INT32U) cwlen;
		pDma->notify = NULL;
		pDma->timeout = 0;

	} else if ((aud_in == BUILD_IN_MIC)||(aud_in == BUILD_LINEIN)) {
		pDma = &pAudio_Encode_Para->mic_dma_dbf;
		pDma->s_addr = (INT32U) P_I2SRX_DATA;
		pDma->t_addr = (INT32U) data;
		pDma->width = DMA_DATA_WIDTH_4BYTE;
		pDma->count = (INT32U) cwlen/2;
		pDma->notify = NULL;
		pDma->timeout = 0;
	} else {
		return STATUS_FAIL;
	}

	status = drv_l1_dma_transfer_double_buf_set(pDma);
	if(status != 0) {
		return status;
	}

	return STATUS_OK;
}

//===============================================================================================
// 	aud_adc_dma_status_get
//	parameter:	none
//	return:		status.
//===============================================================================================
static INT32S aud_adc_dma_status_get(INT8U aud_in)
{
	INT32S dbf = 0;
	INT32S status = 0;
	DMA_STRUCT *pDma;

	if (aud_in == ADC_LINE_IN) {
		pDma = &pAudio_Encode_Para->adc_dma_dbf;
	} else if ((aud_in == BUILD_IN_MIC)||(aud_in == BUILD_LINEIN)) {
		pDma = &pAudio_Encode_Para->mic_dma_dbf;
	} else {
		return STATUS_FAIL;
	}

	if (pDma->channel == 0xff) {
		return -1;
	}

	dbf = drv_l1_dma_dbf_status_get(pDma->channel);
	status = drv_l1_dma_status_get(pDma->channel);
	if(dbf || status) {
		return 1;
	}

	return 0;
}

//===============================================================================================
// 	aud_adc_double_buffer_free
//	parameter:	none
//	return:		free double buffer dma channel.
//===============================================================================================
static void aud_adc_double_buffer_free(INT8U aud_in)
{
	DMA_STRUCT *pDma;

	if (aud_in == ADC_LINE_IN) {
		pDma = &pAudio_Encode_Para->adc_dma_dbf;
	} else if ((aud_in == BUILD_IN_MIC)||(aud_in == BUILD_LINEIN)) {
		pDma = &pAudio_Encode_Para->mic_dma_dbf;
	} else {
		return;
	}

	drv_l1_dma_transfer_double_buf_free(pDma);
	pDma->channel = 0xff;
}

void mic_linein_bias_enable(void)
{
    drv_l1_adc_mic_bias_enable(1);
    drv_msec_wait(400);
}

void mic_linein_bias_disable(void)
{
    drv_l1_adc_mic_bias_enable(0);
}
//===============================================================================================
// 	adc_hardware_start
//	parameter:	SampleRate = adc sample rate set.
//	return:		none.
//===============================================================================================
static INT32S adc_hardware_start(INT8U InputDevice, INT8U channel, INT32U SampleRate)
{
	INT32U adc_pin;

	switch(InputDevice)
	{
	#if ADC_LINE_IN_EN == 1
	case ADC_LINE_IN:
		if(channel != 1) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}

		drv_l1_adc_fifo_clear();
		drv_l1_adc_auto_ch_set(C_ADC_USE_CHANNEL);
		drv_l1_adc_fifo_level_set(4);
		drv_l1_adc_auto_sample_start();
		vTaskDelay(50); //wait bais stable
		drv_l1_adc_sample_rate_set(C_ADC_USE_TIMER, SampleRate);
		break;
	#endif

	#if BUILD_IN_MIC_EN == 1
	case BUILD_IN_MIC:
		if(channel != 1) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}
        //dagc init
        drv_l1_adc_dagc_mode(0,0);
        drv_l1_adc_dagc_scale(0,0);
        drv_l1_adc_dagc_att_rel_time(1,0xf8);
        drv_l1_adc_dagc_threshold(0x20);
        drv_l1_adc_mic_boost_pga(0,0);
        drv_l1_adc_high_pass(1);
        drv_l1_adc_dagc_enable(1);

        aud_adc_input_init(BUILD_IN_MIC, SampleRate);

        vTaskDelay(50);
		break;
	#endif

	#if GPY0050_IN_EN == 1
	case GPY0050_IN:
		if(channel != 1) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}

		gpy0050_start();
		timer_freq_setup(C_AUDIO_RECORD_TIMER, SampleRate, 0, gpy0050_isr);
		break;
	#endif

	case DOUBLE_LINEIN:
		if(channel != 2) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}

	#if ADC_LINE_IN_EN == 1

		drv_l1_adc_fifo_clear();
		drv_l1_adc_fifo_level_set(4);
		drv_l1_adc_auto_ch_set(C_ADC_USE_CHANNEL);
		drv_l1_adc_auto_sample_start();
	#endif

	#if BUILD_IN_MIC_EN == 1
        //dagc init
		drv_l1_adc_dagc_mode(0,0);
        drv_l1_adc_dagc_scale(0,0);
        drv_l1_adc_dagc_att_rel_time(1,0xf8);
        drv_l1_adc_dagc_threshold(0x20);
        drv_l1_adc_dagc_enable(1);
		aud_adc_input_init(BUILD_IN_MIC, SampleRate);
	#endif

		// wait bais stable
        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		OSTimeDly(70);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                vTaskDelay(70);
            #endif
        #endif

        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		OS_ENTER_CRITICAL();
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                vPortEnterCritical();
            #endif
        #endif

	#if ADC_LINE_IN_EN == 1 && BUILD_IN_MIC_EN == 1
		drv_l1_adc_sample_rate_set(C_ADC_USE_TIMER, SampleRate);
	#endif

        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		OS_EXIT_CRITICAL();
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
                vPortExitCritical();
            #endif
        #endif
		break;

    #if BUILD_LINEIN_EN == 1
	case BUILD_LINEIN:
		if(channel != 2) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}
        aud_adc_input_init(BUILD_LINEIN, SampleRate);

        vTaskDelay(50);
		break;
	#endif

	default:
		DEBUG_MSG("InputDeviceERR.\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

//===============================================================================================
// 	adc_hardware_start
//	parameter:	SampleRate = adc sample rate set.
//	return:		none.
//===============================================================================================
static INT32S adc_hardware_stop(INT8U InputDevice, INT8U channel)
{
	switch(InputDevice)
	{
	#if ADC_LINE_IN_EN == 1
	case ADC_LINE_IN:
		if(channel != 1) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}
		drv_l1_adc_timer_stop(C_ADC_USE_TIMER);
		break;
	#endif

	#if BUILD_IN_MIC_EN == 1
	case BUILD_IN_MIC:
		if(channel != 1) {
			return STATUS_FAIL;
		}
		aud_adc_input_stop();
		break;
	#endif

	#if GPY0050_IN_EN == 1
	case GPY0050_IN:
		if(channel != 1) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}
		timer_stop(C_AUDIO_RECORD_TIMER);
		gpy0050_stop();
		break;
	#endif

	case DOUBLE_LINEIN:
		if(channel != 2) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}
	#if ADC_LINE_IN_EN == 1
		drv_l1_adc_timer_stop(C_ADC_USE_TIMER);
	#endif
	#if BUILD_IN_MIC_EN == 1
		aud_adc_input_stop();
	#endif
		break;

	case BUILD_LINEIN:
		if(channel != 2) {
			DEBUG_MSG("ChERR.\r\n");
			return STATUS_FAIL;
		}
    #if BUILD_LINEIN_EN == 1
        aud_adc_input_stop();
    #endif
		break;

	default:
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

#if GPY0050_IN_EN == 1
//===============================================================================================
// 	gpy0050 record
//	parameter:	none
//	return:		free double buffer dma channel.
//===============================================================================================
#define SPI0_TXRX_BY_CPU(Txdata, RxData)\
{\
	R_SPI0_TX_DATA = Txdata; \
	while((R_SPI0_RX_STATUS & 0x07) == 0);\
	RxData = R_SPI0_RX_DATA;\
}\

static INT16U gpy0050_get_value(void)
{
	INT8U dummy;
	INT16U temp;

	gpio_write_io(C_GPY0050_SPI_CS_PIN, 0);	//pull cs low
	SPI0_TXRX_BY_CPU(C_CMD_DUMMY_COM, dummy);
	SPI0_TXRX_BY_CPU(C_CMD_ADC_IN4, dummy);
	temp = dummy;
	SPI0_TXRX_BY_CPU(C_CMD_ZERO_COM, dummy);
	temp <<= 8;
	temp |= dummy;
	gpio_write_io(C_GPY0050_SPI_CS_PIN, 1);	//pull cs high

	if(temp <= 0x00FF)
		temp = pAudio_Encode_Para->pre_value;
	else if(temp >= 0xFFC0)
		temp = pAudio_Encode_Para->pre_value;
	else
		pAudio_Encode_Para->pre_value = temp;

	return temp;
}

static void gpy0050_start(void)
{
	INT8U dummy;

	gpio_init_io(C_GPY0050_SPI_CS_PIN, GPIO_OUTPUT);
	gpio_set_port_attribute(C_GPY0050_SPI_CS_PIN, ATTRIBUTE_HIGH);
	gpio_write_io(C_GPY0050_SPI_CS_PIN, DATA_HIGH);	//pull cs high

	//GPL32 SPI IF 1 initialization
	R_SPI0_TX_STATUS = 0x8020;
	R_SPI0_RX_STATUS = 0x8020;
	R_SPI0_CTRL = 0x0800;	//reset SPI0
	R_SPI0_MISC = 0x0100;  //enable smart mode
	R_SPI0_CTRL = 0x8035;	//Master mode 3, SPI_CLK = SYS_CLK/64

	//sw reset gpy0050
	gpio_write_io(C_GPY0050_SPI_CS_PIN, 0);	//pull cs low
	SPI0_TXRX_BY_CPU(C_CMD_RESET_IN4, dummy);
	gpio_write_io(C_GPY0050_SPI_CS_PIN, 1);	//pull cs high

	//enable MIC AGC and ADC
	gpio_write_io(C_GPY0050_SPI_CS_PIN, 0);		//pull cs low
	SPI0_TXRX_BY_CPU(C_CMD_ENABLE_MIC_AGC_ADC, dummy);
	SPI0_TXRX_BY_CPU(C_CMD_ADC_IN4, dummy);
	gpio_write_io(C_GPY0050_SPI_CS_PIN, 1);	//pull cs high

    #if _OPERATING_SYSTEM != _OS_NONE
      #if _OPERATING_SYSTEM == _OS_UCOS2
	OSTimeDly(30);	//wait 300ms after AGC & MIC enable
      #elif _OPERATING_SYSTEM == _OS_FREERTOS
        vTaskDelay(30);
      #endif
    #endif


	//dummy data
	gpy0050_get_value();
	gpy0050_get_value();
}

static void gpy0050_stop(void)
{
	INT8U dummy;

	//power down
	gpio_write_io(C_GPY0050_SPI_CS_PIN, 0);	//pull cs low
	SPI0_TXRX_BY_CPU(C_CMD_POWER_DOWN, dummy);
	gpio_write_io(C_GPY0050_SPI_CS_PIN, 1);	//pull cs high

	//GPL32 SPI disable
	R_SPI0_CTRL = 0;
	R_SPI0_MISC = 0;
}

static void gpy0050_isr(void)
{
	INT16U *ptr;
	INT32U message;
	ptr = (INT16U*)pAudio_Encode_Para->aud_pcm_buffer[pAudio_Encode_Para->buffer_index];
	*(ptr + pAudio_Encode_Para->buffer_cnt++) = gpy0050_get_value();
	if(pAudio_Encode_Para->buffer_cnt >= pAudio_Encode_Para->PCMInFrameSize)
	{
		pAudio_Encode_Para->buffer_cnt = 0;
		pAudio_Encode_Para->ready_buffer = pAudio_Encode_Para->aud_pcm_buffer[pAudio_Encode_Para->buffer_index];
		pAudio_Encode_Para->buffer_index++;
		if(pAudio_Encode_Para->buffer_index >= C_AUD_PCM_BUFFER_NO) {
			pAudio_Encode_Para->buffer_index = 0;
		}

        #if _OPERATING_SYSTEM != _OS_NONE
            #if _OPERATING_SYSTEM == _OS_UCOS2
		OSQPost(aud_enc_reqQ, (void*)MSG_ADC_DMA_DONE);
            #elif _OPERATING_SYSTEM == _OS_FREERTOS
	       message = MSG_ADC_DMA_DONE;
                xQueueSend( (xQueueHandle) aud_enc_reqQ, (void *) &message, portMAX_DELAY);
            #endif
        #endif
	}
}
#endif

//===============================================================================================
// 	save_buffer_to_storage
//	parameter:
//	return:		status
//===============================================================================================
static INT32S save_buffer_to_storage(void)
{
    INT32U message;
#if 1
    if(pAudio_Encode_Para->MsgTx == pAudio_Encode_Para->MsgRx)
	{
        INT32S ret;
#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
        if(OSQPost(aud_enc_writeQ, (void*)MSG_AUDIO_ENCODE_WRITE)!=OS_NO_ERR) // notify file write
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
        message = MSG_AUDIO_ENCODE_WRITE;
        if(xQueueSend( (xQueueHandle) aud_enc_writeQ, (void *) &message, portMAX_DELAY) != pdPASS)
    #endif
#endif
		{
			return (AUD_RECORD_FILE_WRITE_ERR);
		}
		pAudio_Encode_Para->MsgTx++;
	}
#else
	INT8U  *addr;
	INT32S size, write_cblen;

	if(pAudio_Encode_Para->ring_buffer == 0) {
		if(pAudio_Encode_Para->write_index > C_BS_BUFFER_SIZE/2) {
			pAudio_Encode_Para->ring_buffer = 1;
			addr = pAudio_Encode_Para->Bit_Stream_Buffer;
			size = C_BS_BUFFER_SIZE/2;

			if(pAudio_Encode_Para->SourceType == C_GP_FS) {
#if 1 //m-
				write_cblen = fs_write(pAudio_Encode_Para->FileHandle, (INT32U)addr, size);
#else
                                xSemaphoreTake(lwip_socket_sem, portMAX_DELAY);
                                lwip_write(pAudio_Encode_Para->FileHandle, (void*)addr, size);
                                xSemaphoreGive(lwip_socket_sem);

				write_cblen = size;
#endif
			} else {
				write_cblen = audio_encode_data_write(0, (INT32U)addr, size);
			}

			if(write_cblen != size) {
				return 	AUD_RECORD_FILE_WRITE_ERR;
			}
		}
       } else {
                if(pAudio_Encode_Para->write_index < C_BS_BUFFER_SIZE/2) {
                        pAudio_Encode_Para->ring_buffer = 0;
                        addr = pAudio_Encode_Para->Bit_Stream_Buffer + C_BS_BUFFER_SIZE/2;
                        size = C_BS_BUFFER_SIZE/2;

                        if(pAudio_Encode_Para->SourceType == C_GP_FS) {
#if 1 //m-
				write_cblen = fs_write(pAudio_Encode_Para->FileHandle, (INT32U)addr, size);
#else
                                xSemaphoreTake(lwip_socket_sem, portMAX_DELAY);
                                lwip_write(pAudio_Encode_Para->FileHandle, (void*)addr, size);
                                xSemaphoreGive(lwip_socket_sem);

				write_cblen = size;
#endif
                        } else {
				write_cblen = audio_encode_data_write(0, (INT32U)addr, size);
                        }

                        if(write_cblen != size) {
				return 	AUD_RECORD_FILE_WRITE_ERR;
                        }
		}
	}
#endif
	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	save_final_data_to_storage
//	parameter:
//	return:		status
//===============================================================================================
static INT32S save_final_data_to_storage(void)
{
#if 1
	INT8U  err;
	INT32S msg;
	INT32U message;

    xQueueReceive(aud_write_task_ack_m, (void *) &message, 0);
	message = MSG_AUDIO_ENCODE_WRITE_STOP;
    if(xQueueSend(aud_enc_writeQ, (void *) &message, portMAX_DELAY) != pdPASS)
	{
		DBG_PRINT("xQueueSend Fail!!!\r\n");
		return (AUD_RECORD_FILE_WRITE_ERR);
	}
	if(xQueueReceive(aud_write_task_ack_m, &message, 5000/10) != pdPASS)
	{
		DBG_PRINT("xQueueReceive ack Fail!!!\r\n");
		return (AUD_RECORD_FILE_WRITE_ERR);
	}
	if(message != AUD_WRITE_ACK_OK)
	{
		DBG_PRINT("xQueueReceive ack Fail!!!\r\n");
		return (AUD_RECORD_FILE_WRITE_ERR);
	}

    DBG_PRINT("read = %d,write=%d\r\n", pAudio_Encode_Para->read_index, pAudio_Encode_Para->write_index);
#else
	INT8U *addr;
	INT32S size, write_cblen;

	if(pAudio_Encode_Para->write_index > C_BS_BUFFER_SIZE/2) {
		addr = pAudio_Encode_Para->Bit_Stream_Buffer + C_BS_BUFFER_SIZE/2;
		size = pAudio_Encode_Para->write_index - C_BS_BUFFER_SIZE/2;
		if(pAudio_Encode_Para->SourceType == C_GP_FS) {
#if 1 //m-
			write_cblen = fs_write(pAudio_Encode_Para->FileHandle, (INT32U)addr, size);
#else
                        xSemaphoreTake(lwip_socket_sem, portMAX_DELAY);
                        lwip_write(pAudio_Encode_Para->FileHandle, (void*)addr, size);
                        xSemaphoreGive(lwip_socket_sem);

			write_cblen = size;
#endif
		} else {
			write_cblen = audio_encode_data_write(0, (INT32U)addr, size);
		}

		if(write_cblen != size) {
			return 	AUD_RECORD_FILE_WRITE_ERR;
		}
	} else {
		addr = pAudio_Encode_Para->Bit_Stream_Buffer;
		size = pAudio_Encode_Para->write_index;
		if(pAudio_Encode_Para->SourceType == C_GP_FS) {
#if 1 //m-
			write_cblen = fs_write(pAudio_Encode_Para->FileHandle, (INT32U)addr, size);
#else
                        xSemaphoreTake(lwip_socket_sem, portMAX_DELAY);
                        lwip_write(pAudio_Encode_Para->FileHandle, (void*)addr, size);
                        xSemaphoreGive(lwip_socket_sem);

			write_cblen = size;
#endif
		} else {
			write_cblen = audio_encode_data_write(0, (INT32U)addr, size);
		}

		if(write_cblen != size) {
			return 	AUD_RECORD_FILE_WRITE_ERR;
		}
	}
#endif
	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	adc_memory_allocate
//	parameter:	cbLen = adc buffer size in byte.
//	return:		status.
//===============================================================================================
static INT32S adc_memory_allocate(INT32U CbLen, INT8U adc_in)
{
	INT16U *ptr;
	INT32S i, j;

	if(adc_in & ADC_LINE_IN) {
		pAudio_Encode_Para->aud_pcm_buffer[0] =(INT32U) gp_malloc_align(CbLen*C_AUD_PCM_BUFFER_NO, 4);
		if(pAudio_Encode_Para->aud_pcm_buffer[0] == 0) {
			return AUD_RECORD_MEMORY_ALLOC_ERR;
		}

		for(i=0; i<C_AUD_PCM_BUFFER_NO; i++) {
			pAudio_Encode_Para->aud_pcm_buffer[i] = pAudio_Encode_Para->aud_pcm_buffer[0] + (CbLen * i);
			DEBUG_MSG("aud_pcm_buffer[%d] = 0x%x\r\n", i, pAudio_Encode_Para->aud_pcm_buffer[i]);
			ptr = (INT16U*)pAudio_Encode_Para->aud_pcm_buffer[i];
			for(j=0; j<CbLen/2; j++) {
				*ptr++ = 0x8000;
			}
		}
	}

	if((adc_in & BUILD_IN_MIC)||(adc_in & BUILD_LINEIN)) {
		pAudio_Encode_Para->mic_pcm_buffer[0] =(INT32U) gp_malloc_align(CbLen*C_AUD_PCM_BUFFER_NO, 4);
		if(pAudio_Encode_Para->mic_pcm_buffer[0] == 0) {
			return AUD_RECORD_MEMORY_ALLOC_ERR;
		}

		for(i=0; i<C_AUD_PCM_BUFFER_NO; i++) {
			pAudio_Encode_Para->mic_pcm_buffer[i] = pAudio_Encode_Para->mic_pcm_buffer[0] + (CbLen * i);
			DEBUG_MSG("mic_pcm_buffer[%d] = 0x%x\r\n", i, pAudio_Encode_Para->mic_pcm_buffer[i]);
			ptr = (INT16U*)pAudio_Encode_Para->mic_pcm_buffer[i];
			for(j=0; j<CbLen/2; j++) {
				*ptr++ = 0x8000;
			}
		}
	}

	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	adc_memory_free
//	parameter:	none.
//	return:		status.
//===============================================================================================
static INT32S adc_memory_free(void)
{
	INT32S i;

	if(pAudio_Encode_Para->aud_pcm_buffer[0]) {
		gp_free((void *)pAudio_Encode_Para->aud_pcm_buffer[0]);
	}

	if(pAudio_Encode_Para->mic_pcm_buffer[0]) {
		gp_free((void *)pAudio_Encode_Para->mic_pcm_buffer[0]);
	}

	if(pAudio_Encode_Para->Bit_Stream_Buffer) {
		gp_free((void *)pAudio_Encode_Para->Bit_Stream_Buffer);
		pAudio_Encode_Para->Bit_Stream_Buffer = 0;
	}

	for(i=0; i<C_AUD_PCM_BUFFER_NO; i++) {
		pAudio_Encode_Para->aud_pcm_buffer[i] = 0;
		pAudio_Encode_Para->mic_pcm_buffer[i] = 0;
	}

	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	adc_work_mem_free
//	parameter:
//	return:
//===============================================================================================
static void adc_work_mem_free(void)
{
	if(pAudio_Encode_Para->EncodeWorkMem) {
		gp_free((void *)pAudio_Encode_Para->EncodeWorkMem);
		pAudio_Encode_Para->EncodeWorkMem = 0;
	}

	if(pAudio_Encode_Para->Bit_Stream_Buffer) {
		gp_free((void *)pAudio_Encode_Para->Bit_Stream_Buffer);
		pAudio_Encode_Para->Bit_Stream_Buffer = 0;
	}

	if(pAudio_Encode_Para->pack_buffer) {
		gp_free((void *)pAudio_Encode_Para->pack_buffer);
		pAudio_Encode_Para->pack_buffer = 0;
	}
}

#if RECORD_WITH_DAC_OUT_EN == 1
static void dac_out_start(INT32U channel, INT32U samplerate)
{
	if(channel == 1) {
		drv_l1_dac_mono_set();
	} else {
		drv_l1_dac_stereo_set();
	}

	drv_l1_dac_sample_rate_set(samplerate);
}

static void dac_out_stop(INT32U channel)
{
	#define RAMPDOWN_STEP		4
	#define RAMPDOWN_STEP_HOLD	4
	INT16S  last_ldata,last_rdata;
	INT16S  i, temp;

	temp = 0 - RAMPDOWN_STEP;
	last_ldata = R_DAC_CHA_DATA;
	last_rdata = R_DAC_CHB_DATA;

	//unsigned to signed
	last_ldata ^= 0x8000;
	if(channel == 2) {
		last_rdata ^= 0x8000;
	}
	else {
		last_rdata = 0x0;
	}

	//change timer to 44100
	drv_l1_dac_sample_rate_set(44100);
	while(1)
	{
		if (last_ldata > 0x0) {
			last_ldata -= RAMPDOWN_STEP;
		}
		else if (last_ldata < 0x0) {
			last_ldata += RAMPDOWN_STEP;
		}

		if ((last_ldata < RAMPDOWN_STEP)&&(last_ldata > temp)) {
			last_ldata = 0;
		}

		if (channel == 2) {
			if (last_rdata > 0x0) {
				last_rdata -= RAMPDOWN_STEP;
		    }
			else if (last_rdata < 0x0) {
				last_rdata += RAMPDOWN_STEP;
		    }

		    if ((last_rdata < RAMPDOWN_STEP)&&(last_rdata > temp)) {
				last_rdata = 0;
			}
		}

		for(i=0;i<RAMPDOWN_STEP_HOLD;i++) {
			if (channel == 2){
				while(R_DAC_CHA_FIFO & 0x8000);
				R_DAC_CHA_DATA = last_ldata;
				while(R_DAC_CHB_FIFO & 0x8000);
				R_DAC_CHB_DATA = last_rdata;
			} else {
				while(R_DAC_CHA_FIFO & 0x8000);
				R_DAC_CHA_DATA = last_ldata;
			}
		}

		if ((last_ldata == 0x0) && (last_rdata == 0x0)) {
			break;
		}
	}

	drv_l1_dac_timer_stop();
}
#endif

#if 1
//===============================================================================================
// 	aud_enc_RIFF_init
//	parameter:	samplerate:  wave file sample rate.
//	return:		none.
//===============================================================================================
static void aud_enc_RIFF_init(INT32U samplerate)
{
	WaveHeadPara.RIFF_ID[0] ='R';
	WaveHeadPara.RIFF_ID[1] ='I';
	WaveHeadPara.RIFF_ID[2] ='F';
	WaveHeadPara.RIFF_ID[3] ='F';
	WaveHeadPara.RIFF_len = 0;

	WaveHeadPara.type_ID[0] = 'W';
	WaveHeadPara.type_ID[1] = 'A';
	WaveHeadPara.type_ID[2] = 'V';
	WaveHeadPara.type_ID[3] = 'E';

	WaveHeadPara.fmt_ID[0] = 'f';
	WaveHeadPara.fmt_ID[1] = 'm';
	WaveHeadPara.fmt_ID[2] = 't';
	WaveHeadPara.fmt_ID[3] = ' ';

	WaveHeadPara.fmt_len = 16;
	WaveHeadPara.format = 1;
	WaveHeadPara.channel = 1;
	WaveHeadPara.sample_rate = samplerate;

	//8, 16, 24 or 32
	WaveHeadPara.Sign_bit_per_sample = 16;
	//BlockAlign = SignificantBitsPerSample / 8 * NumChannels
	WaveHeadPara.Block_align = 16/8*1;
	//AvgBytesPerSec = SampleRate * BlockAlign
	WaveHeadPara.avg_byte_per_sec = samplerate*2;

	WaveHeadPara.data_ID[0] = 'd';
	WaveHeadPara.data_ID[1] = 'a';
	WaveHeadPara.data_ID[2] = 't';
	WaveHeadPara.data_ID[3] = 'a';

	WaveHeadPara.data_len = 0;
}

//===============================================================================================
// 	wave_encode_start
//	parameter:	none.
//	return:		status.
//===============================================================================================
static INT32S wave_encode_start(void)
{
	INT32S cbLen;
	AUD_ENC_WAVE_HEADER *pWaveHeadPara;

	pAudio_Encode_Para->EncodeWorkMem = NULL;
	pAudio_Encode_Para->Bit_Stream_Buffer = (INT8U *)gp_malloc_align(C_BS_BUFFER_SIZE, 4);
	if(!pAudio_Encode_Para->Bit_Stream_Buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	//copy header
	pWaveHeadPara = &WaveHeadPara;
	cbLen = sizeof(AUD_ENC_WAVE_HEADER);
	aud_enc_RIFF_init(pAudio_Encode_Para->SampleRate);
	gp_memcpy((INT8S*)pAudio_Encode_Para->Bit_Stream_Buffer, (INT8S*)pWaveHeadPara, cbLen);

	pAudio_Encode_Para->write_index = cbLen;
	pAudio_Encode_Para->FileLenth = cbLen;
	pAudio_Encode_Para->PackSize = C_WAVE_RECORD_BUFFER_SIZE<<1;
	pAudio_Encode_Para->pack_buffer = NULL;
	pAudio_Encode_Para->PCMInFrameSize = C_WAVE_RECORD_BUFFER_SIZE;
	pAudio_Encode_Para->OnePCMFrameSize = C_WAVE_RECORD_BUFFER_SIZE;

	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	wave_encode_stop
//	parameter:	none.
//	return:		status.
//===============================================================================================
static INT32S wave_encode_stop(void)
{
	INT8U txbuf[64];

	INT32S nRet;
	AUD_ENC_WAVE_HEADER *pWaveHeadPara;

	nRet = save_final_data_to_storage();
	if(nRet < 0)
	{
		adc_work_mem_free();
		return AUD_RECORD_FILE_WRITE_ERR;
	}

	//write header
	pWaveHeadPara = &WaveHeadPara;
	pAudio_Encode_Para->FileLenth -= sizeof(AUD_ENC_WAVE_HEADER);
	pWaveHeadPara->RIFF_len = sizeof(AUD_ENC_WAVE_HEADER) + pAudio_Encode_Para->FileLenth - 8;//file size -8
	pWaveHeadPara->data_len = pAudio_Encode_Para->FileLenth;
	if(pAudio_Encode_Para->SourceType == C_GP_FS)
	{
#if 1 //m-
		lseek(pAudio_Encode_Para->FileHandle, 0, SEEK_SET);
		fs_write(pAudio_Encode_Para->FileHandle, (INT32U)pWaveHeadPara, sizeof(AUD_ENC_WAVE_HEADER));
		fs_close(pAudio_Encode_Para->FileHandle);
#else

                memset(txbuf, 0, sizeof(txbuf));

                printf("send CLOSE_FILE_0\r\n");

                sprintf(txbuf, "%s", "CLOSE_FILE_0");
                xSemaphoreTake(lwip_socket_sem, portMAX_DELAY);
                lwip_write(pAudio_Encode_Para->FileHandle, (void*)txbuf, strlen(txbuf));
                xSemaphoreGive(lwip_socket_sem);

                memset(txbuf, 0, sizeof(txbuf));
                xSemaphoreTake(lwip_socket_sem, portMAX_DELAY);
                lwip_read(pAudio_Encode_Para->FileHandle, txbuf, 64);
                xSemaphoreGive(lwip_socket_sem);

                printf("recv CLOSE_FILE_1\r\n");

                if (strcmp(txbuf, "CLOSE_FILE_1") == 0)
                {
                        xSemaphoreTake(lwip_socket_sem, portMAX_DELAY);
                        lwip_write(pAudio_Encode_Para->FileHandle, (void*)pWaveHeadPara, sizeof(AUD_ENC_WAVE_HEADER));
                        xSemaphoreGive(lwip_socket_sem);
                }
#endif
	}
	else if(pAudio_Encode_Para->SourceType == C_USER_DEFINE)
	{
		audio_encode_data_write(1, (INT32U)pWaveHeadPara, sizeof(AUD_ENC_WAVE_HEADER));
	}

	//free memory
	adc_work_mem_free();
	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	wave_encode_once
//	parameter:	workmem = work memory
//				buffer_addr = buffer address
//				cwlen = buffer size in word
//	return:		length in word
//===============================================================================================
static int wave_encode_once(void *workmem, const short* buffer_addr, int cwlen)
{
	INT8U  *dest_addr;
	INT32S nRet, temp;
	INT32U cblen, size;
	INT32U  free_size;

	if(pAudio_Encode_Para->read_index < pAudio_Encode_Para->write_index)
    {
        free_size = C_BS_BUFFER_SIZE + pAudio_Encode_Para->read_index - pAudio_Encode_Para->write_index;
    }
    else
    {
        free_size = pAudio_Encode_Para->read_index - pAudio_Encode_Para->write_index;
    }

	cblen = cwlen<<1;
	if(cblen >= free_size)
	{
	    return AUD_RECORD_FILE_WRITE_ERR;
	}

	temp = pAudio_Encode_Para->write_index + cblen;
	if(temp > C_BS_BUFFER_SIZE)
	{
		size = C_BS_BUFFER_SIZE - pAudio_Encode_Para->write_index;
		dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer + pAudio_Encode_Para->write_index;
		gp_memcpy((INT8S*)dest_addr, (INT8S*)buffer_addr, size);

		temp = cblen - size;		//remain size
		if(temp > C_BS_BUFFER_SIZE/2)
		{
		    DBG_PRINT("C_BS_BUFFER_SIZE\r\n");
		}
		dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer;
		buffer_addr += (size>>1); 	//word address
		gp_memcpy((INT8S*)dest_addr, (INT8S*)buffer_addr, temp);
		vTaskSuspendAll();
		pAudio_Encode_Para->write_index = temp;
		xTaskResumeAll();
	}
	else
	{
		dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer + pAudio_Encode_Para->write_index;
		gp_memcpy((INT8S*)dest_addr, (INT8S*)buffer_addr, cblen);
		vTaskSuspendAll();
		pAudio_Encode_Para->write_index += cblen;
		xTaskResumeAll();
	}

	pAudio_Encode_Para->FileLenth += cblen;
	nRet = save_buffer_to_storage();
	if(nRet < 0)
		return AUD_RECORD_FILE_WRITE_ERR;

	return cwlen;
}
#endif

//===============================================================================================
// 	a1800_encode_start
//	parameter:	none.
//	return:		status.
//===============================================================================================
#if APP_A1800_ENCODE_EN
static INT32S a1800_encode_start(void)
{
	INT32S nRet;

	pAudio_Encode_Para->EncodeWorkMem = (INT8U *)gp_malloc(A18_ENC_MEMORY_SIZE);
	if(pAudio_Encode_Para->EncodeWorkMem == NULL)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	gp_memset((INT8S*)pAudio_Encode_Para->EncodeWorkMem, 0, A18_ENC_MEMORY_SIZE);
	pAudio_Encode_Para->Bit_Stream_Buffer = (INT8U *)gp_malloc_align(C_BS_BUFFER_SIZE, 4);
	if(!pAudio_Encode_Para->Bit_Stream_Buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	gp_memset((INT8S*)pAudio_Encode_Para->Bit_Stream_Buffer, 0, 6);
	pAudio_Encode_Para->write_index = 6;
	pAudio_Encode_Para->FileLenth = 6;

	A18_enc_set_BitRate((unsigned char *)pAudio_Encode_Para->EncodeWorkMem, pAudio_Encode_Para->BitRate);
	nRet = A18_enc_get_BitRate((unsigned char *)pAudio_Encode_Para->EncodeWorkMem);
	if(nRet != pAudio_Encode_Para->BitRate)
		return AUD_RECORD_RUN_ERR;

	nRet = A18_enc_init((unsigned char *)pAudio_Encode_Para->EncodeWorkMem);
	if(nRet != A18_ENC_NO_ERROR)
		return AUD_RECORD_RUN_ERR;

	pAudio_Encode_Para->PackSize = A18_enc_get_PackageSize((unsigned char *)pAudio_Encode_Para->EncodeWorkMem);
	pAudio_Encode_Para->pack_buffer = (INT8U *)gp_malloc_align(pAudio_Encode_Para->PackSize, 4);
	if(!pAudio_Encode_Para->pack_buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	pAudio_Encode_Para->PCMInFrameSize = C_A1800_RECORD_BUFFER_SIZE * A1800_TIMES;
	pAudio_Encode_Para->OnePCMFrameSize = C_A1800_RECORD_BUFFER_SIZE;

	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	a1800_encode_stop
//	parameter:	none.
//	return:		status.
//===============================================================================================
static INT32S a1800_encode_stop(void)
{
	INT8U  A1800Header[6];
	INT32S nRet;

	nRet = save_final_data_to_storage();
	if(nRet < 0)
	{
		adc_work_mem_free();
		return AUD_RECORD_FILE_WRITE_ERR;
	}

	//write header
	pAudio_Encode_Para->FileLenth -= 4;		//header length not include file size(4 byte)
	A1800Header[0] = pAudio_Encode_Para->FileLenth & 0xFF; 		  //file length
	A1800Header[1] = (pAudio_Encode_Para->FileLenth >> 8) & 0xFF; //file length
	A1800Header[2] = (pAudio_Encode_Para->FileLenth >> 16) & 0xFF; //file length
	A1800Header[3] = (pAudio_Encode_Para->FileLenth >> 24) & 0xFF; //file length
	A1800Header[4] =  pAudio_Encode_Para->BitRate & 0xFF; 		//bit rate
	A1800Header[5] = (pAudio_Encode_Para->BitRate >> 8) & 0xFF; //bit rate
	if(pAudio_Encode_Para->SourceType == C_GP_FS)
	{
		lseek(pAudio_Encode_Para->FileHandle, 0, SEEK_SET);
		fs_write(pAudio_Encode_Para->FileHandle, (INT32U)A1800Header, 6);
		fs_close(pAudio_Encode_Para->FileHandle);
	}
	else if(pAudio_Encode_Para->SourceType == C_USER_DEFINE)
		audio_encode_data_write(1, (INT32U)A1800Header, 4);

	//free memory
	adc_work_mem_free();
	return  AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	a1800_encode_once
//	parameter:	workmem = work memory
//				buffer_addr = buffer address
//				cwlen = buffer size in word
//	return:		length in word
//===============================================================================================
static int a1800_encode_once(void *workmem, const short* buffer_addr, int cwlen)
{
	volatile INT8U  *dest_addr;
	INT32U N;
	INT32S nRet, cblen, temp, size;
	INT32S  free_size;

	free_size = pAudio_Encode_Para->read_index - pAudio_Encode_Para->write_index;
	if(free_size < 0) free_size += C_BS_BUFFER_SIZE;

	cblen = 0;
	N = cwlen;

	while(N >= pAudio_Encode_Para->OnePCMFrameSize)
	{
		nRet = A18_enc_run((unsigned char *)pAudio_Encode_Para->EncodeWorkMem, (short *)buffer_addr, (unsigned char *)pAudio_Encode_Para->pack_buffer);
		if(nRet != A18_ENC_NO_ERROR)
			return AUD_RECORD_RUN_ERR;

		buffer_addr += pAudio_Encode_Para->OnePCMFrameSize;
		cblen += pAudio_Encode_Para->PackSize;
		N -= pAudio_Encode_Para->OnePCMFrameSize;

		//copy to bit stream buffer
		if(free_size < pAudio_Encode_Para->PackSize)
		{
		    //update free_size
		    free_size = pAudio_Encode_Para->read_index - pAudio_Encode_Para->write_index;
	        if(free_size < 0) free_size += C_BS_BUFFER_SIZE;

	        if(free_size < pAudio_Encode_Para->PackSize)
		    {
		        return AUD_RECORD_FILE_WRITE_ERR;
		    }
		}
		free_size -= pAudio_Encode_Para->PackSize;
		temp = pAudio_Encode_Para->write_index + pAudio_Encode_Para->PackSize;
		if(temp > C_BS_BUFFER_SIZE)
		{
			size = C_BS_BUFFER_SIZE - pAudio_Encode_Para->write_index;
			dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer + pAudio_Encode_Para->write_index;
			gp_memcpy((INT8S*)dest_addr, (INT8S*)pAudio_Encode_Para->pack_buffer, size);

			temp = pAudio_Encode_Para->PackSize - size;		//remain size
			dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer;
			gp_memcpy((INT8S*)dest_addr, (INT8S*)(pAudio_Encode_Para->pack_buffer + size), temp);
			vTaskSuspendAll();
			pAudio_Encode_Para->write_index = temp;
			xTaskResumeAll();
		}
		else
		{
			dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer + pAudio_Encode_Para->write_index;
			gp_memcpy((INT8S*)dest_addr, (INT8S*)pAudio_Encode_Para->pack_buffer, pAudio_Encode_Para->PackSize);
			vTaskSuspendAll();
			pAudio_Encode_Para->write_index += pAudio_Encode_Para->PackSize;
			xTaskResumeAll();

		}
	}

	pAudio_Encode_Para->FileLenth += cblen;
	nRet = save_buffer_to_storage();
	if(nRet < 0)
		return AUD_RECORD_FILE_WRITE_ERR;

	return cwlen;
}
#endif //APP_A1800_ENCODE_EN

//===============================================================================================
// 	wave_encode_lib_start
//	parameter:	none.
//	return:		status.
//===============================================================================================
#if APP_WAV_CODEC_EN
static INT32S wave_encode_lib_start(INT32U AudioFormat)
{
	INT32S nRet, size;

	size = wav_enc_get_mem_block_size();
	pAudio_Encode_Para->EncodeWorkMem = (INT8U *)gp_malloc(size);
	if(pAudio_Encode_Para->EncodeWorkMem == NULL)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	gp_memset((INT8S*)pAudio_Encode_Para->EncodeWorkMem, 0, size);
	pAudio_Encode_Para->Bit_Stream_Buffer = (INT8U *)gp_malloc_align(C_BS_BUFFER_SIZE, 4);
	if(!pAudio_Encode_Para->Bit_Stream_Buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	nRet = wav_enc_Set_Parameter( pAudio_Encode_Para->EncodeWorkMem,
								  pAudio_Encode_Para->Channel,
								  pAudio_Encode_Para->SampleRate,
								  AudioFormat );
	if(nRet < 0)
		return AUD_RECORD_RUN_ERR;

	nRet = wav_enc_init(pAudio_Encode_Para->EncodeWorkMem);
	if(nRet < 0)
		return AUD_RECORD_RUN_ERR;

	//copy header
	size = wav_enc_get_HeaderLength(pAudio_Encode_Para->EncodeWorkMem);
	gp_memset((INT8S*)pAudio_Encode_Para->Bit_Stream_Buffer, 0, size);
	pAudio_Encode_Para->write_index = size;
	pAudio_Encode_Para->FileLenth = size;
	pAudio_Encode_Para->NumSamples = 0;

	size = wav_enc_get_SamplePerFrame(pAudio_Encode_Para->EncodeWorkMem);
	pAudio_Encode_Para->PCMInFrameSize = size * ADPCM_TIMES;
	pAudio_Encode_Para->OnePCMFrameSize = size;

#if 1
	pAudio_Encode_Para->PackSize = size * 2;
#else
	pAudio_Encode_Para->PackSize = wav_enc_get_BytePerPackage(pAudio_Encode_Para->EncodeWorkMem);
#endif

	pAudio_Encode_Para->pack_buffer = (INT8U *)gp_malloc_align(pAudio_Encode_Para->PackSize, 4);
	if(!pAudio_Encode_Para->pack_buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	wave_encode_lib_stop
//	parameter:	none.
//	return:		status.
//===============================================================================================
static INT32S wave_encode_lib_stop(void)
{
	INT8U *pHeader;
	INT32S cbLen, nRet;

	nRet = save_final_data_to_storage();
	if(nRet < 0)
	{
		adc_work_mem_free();
		return AUD_RECORD_FILE_WRITE_ERR;
	}

	//write header
	cbLen = wav_enc_get_HeaderLength(pAudio_Encode_Para->EncodeWorkMem);
	pHeader = (INT8U *)gp_malloc(cbLen);
	if(!pHeader)
	{
		adc_work_mem_free();
		return AUD_RECORD_MEMORY_ALLOC_ERR;
	}

	nRet = wav_enc_get_header(pAudio_Encode_Para->EncodeWorkMem, pHeader, pAudio_Encode_Para->FileLenth, pAudio_Encode_Para->NumSamples);
	if(pAudio_Encode_Para->SourceType == C_GP_FS)
	{
		lseek(pAudio_Encode_Para->FileHandle, 0, SEEK_SET);
		fs_write(pAudio_Encode_Para->FileHandle, (INT32U)pHeader, cbLen);
		fs_close(pAudio_Encode_Para->FileHandle);
	}
	else if(pAudio_Encode_Para->SourceType == C_USER_DEFINE)
	{
		audio_encode_data_write(1, (INT32U)pHeader, cbLen);
	}

	//free memory
	gp_free((void *)pHeader);
	adc_work_mem_free();

	return	AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	wave_encode_lib_once
//	parameter:	workmem = work memory
//				buffer_addr = buffer address
//				cwlen = buffer size in word
//	return:		length in word
//===============================================================================================
static int wave_encode_lib_once(void *workmem, const short* buffer_addr, int cwlen)
{
	INT8U  *dest_addr;
	INT32U N, PackSize;
	INT32S nRet, cblen, temp, size;
	INT32S  free_size;

	free_size = pAudio_Encode_Para->read_index - pAudio_Encode_Para->write_index;
	if(free_size < 0) free_size += C_BS_BUFFER_SIZE;

	cblen = 0;
	N = cwlen;

	while(N >= pAudio_Encode_Para->OnePCMFrameSize)
	{
		nRet = wav_enc_run(pAudio_Encode_Para->EncodeWorkMem, (short *)buffer_addr, pAudio_Encode_Para->pack_buffer);
		if(nRet < 0)
			return  AUD_RECORD_RUN_ERR;

		PackSize = nRet;
		pAudio_Encode_Para->NumSamples += wav_enc_get_SamplePerFrame(pAudio_Encode_Para->EncodeWorkMem);
		buffer_addr += pAudio_Encode_Para->OnePCMFrameSize;
		cblen += PackSize;
		N -= pAudio_Encode_Para->OnePCMFrameSize;

		if(free_size < PackSize)
		{
		    //update free_size
		    free_size = pAudio_Encode_Para->read_index - pAudio_Encode_Para->write_index;
	        if(free_size < 0) free_size += C_BS_BUFFER_SIZE;

	        if(free_size < PackSize)
		    {
		        return AUD_RECORD_FILE_WRITE_ERR;
		    }
		}
		free_size -= PackSize;

		//copy to bit stream buffer
		temp = pAudio_Encode_Para->write_index + PackSize;
		if(temp > C_BS_BUFFER_SIZE)
		{
			size = C_BS_BUFFER_SIZE - pAudio_Encode_Para->write_index;
			dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer + pAudio_Encode_Para->write_index;
			gp_memcpy((INT8S*)dest_addr, (INT8S*)pAudio_Encode_Para->pack_buffer, size);

			temp = PackSize - size;		//remain size
			dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer;
			gp_memcpy((INT8S*)dest_addr, (INT8S*)(pAudio_Encode_Para->pack_buffer + size), temp);
			vTaskSuspendAll();
			pAudio_Encode_Para->write_index = temp;
			xTaskResumeAll();
		}
		else
		{
			dest_addr = pAudio_Encode_Para->Bit_Stream_Buffer + pAudio_Encode_Para->write_index;
			gp_memcpy((INT8S*)dest_addr, (INT8S*)pAudio_Encode_Para->pack_buffer, PackSize);
			vTaskSuspendAll();
			pAudio_Encode_Para->write_index += PackSize;
			xTaskResumeAll();
		}
	}

	pAudio_Encode_Para->FileLenth += cblen;
	nRet = save_buffer_to_storage();
	if(nRet < 0)
		return AUD_RECORD_FILE_WRITE_ERR;

	return cwlen;
}
#endif	//APP_WAV_CODEC_EN

//===============================================================================================
// 	mp3_encode_start
//	parameter:	none.
//	return:		status.
//===============================================================================================
#if APP_MP3_ENCODE_EN
static INT32S mp3_encode_start(void)
{
	INT32S nRet, size;

	size = mp3enc_GetWorkMemSize();
	pAudio_Encode_Para->EncodeWorkMem = (INT8U *)gp_malloc(size);
	if(!pAudio_Encode_Para->EncodeWorkMem)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	gp_memset((INT8S*)pAudio_Encode_Para->EncodeWorkMem, 0, size);
	pAudio_Encode_Para->Bit_Stream_Buffer = (INT8U *)gp_malloc_align(C_BS_BUFFER_SIZE, 4);	//ring buffer
	if(!pAudio_Encode_Para->Bit_Stream_Buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	nRet = mp3enc_init(	pAudio_Encode_Para->EncodeWorkMem,
						pAudio_Encode_Para->Channel,
						pAudio_Encode_Para->SampleRate,
						(pAudio_Encode_Para->BitRate/1000),
						0, //copyright
						(CHAR *)pAudio_Encode_Para->Bit_Stream_Buffer,
						C_BS_BUFFER_SIZE,
						0); //RingWI, for ID3 header skip.
	if(nRet<0)
		return AUD_RECORD_INIT_ERR;

	pAudio_Encode_Para->write_index = 0;
	pAudio_Encode_Para->PackSize = 0;
	pAudio_Encode_Para->pack_buffer = NULL;
	//size = nRet * MP3_TIME * pAudio_Encode_Para->Channel;
	size = nRet * MP3_TIME;
	pAudio_Encode_Para->PCMInFrameSize = size;
	//size = nRet*pAudio_Encode_Para->Channel;
	size = nRet;
	pAudio_Encode_Para->OnePCMFrameSize = size;

	return AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	MP3_encode_stop
//	parameter:	none.
//	return:		status.
//===============================================================================================
static INT32S mp3_encode_stop(void)
{
	INT32S nRet;
	INT32U old_index;

	old_index = pAudio_Encode_Para->write_index;
	nRet = mp3enc_end((void *)pAudio_Encode_Para->EncodeWorkMem);
	if(nRet> 0)
	{
		pAudio_Encode_Para->write_index = nRet;
		nRet = pAudio_Encode_Para->write_index - old_index;
		if(nRet <0)
			nRet += C_BS_BUFFER_SIZE;

		pAudio_Encode_Para->FileLenth += nRet;
		nRet = save_buffer_to_storage();
		if(nRet < 0)
		{
			adc_work_mem_free();
			return AUD_RECORD_FILE_WRITE_ERR;
		}
	}

	nRet = save_final_data_to_storage();
	if(nRet < 0)
	{
		adc_work_mem_free();
		return AUD_RECORD_FILE_WRITE_ERR;
	}

	if(pAudio_Encode_Para->SourceType == C_GP_FS)
		fs_close(pAudio_Encode_Para->FileHandle);
	else
		audio_encode_data_write(1, NULL, 0);

	//free memory
	adc_work_mem_free();
	return  AUD_RECORD_STATUS_OK;
}

//===============================================================================================
// 	mp3_encode_once
//	parameter:	workmem = work memory
//				buffer_addr = buffer address
//				cwlen = buffer size in word
//	return:		length in word
//===============================================================================================
static int mp3_encode_once(void *workmem, const short* buffer_addr, int cwlen)
{
	INT32U N, old_index;
	INT32S nRet, cblen;

	cblen = 0;
	old_index = pAudio_Encode_Para->write_index;
	N = cwlen;

	while(N >= pAudio_Encode_Para->OnePCMFrameSize)
	{
		nRet =  mp3enc_encframe((void *)pAudio_Encode_Para->EncodeWorkMem, (short *)buffer_addr);
		if(nRet<0)
			return AUD_RECORD_RUN_ERR;

		pAudio_Encode_Para->write_index = nRet;
		buffer_addr += pAudio_Encode_Para->OnePCMFrameSize;
		N -= pAudio_Encode_Para->OnePCMFrameSize;
	}

	cblen = pAudio_Encode_Para->write_index - old_index;
	if(cblen <0)
		cblen += C_BS_BUFFER_SIZE;

	pAudio_Encode_Para->FileLenth += cblen;
	nRet = save_buffer_to_storage();
	if(nRet < 0)
		return AUD_RECORD_FILE_WRITE_ERR;

	return cwlen;
}
#endif //APP_MP3_ENCODE_EN




#if APP_VR_ENCODE_EN
INT32S vr_encode_start(void)
{

	pAudio_Encode_Para->Bit_Stream_Buffer = (INT8U *)gp_malloc_align(C_BS_BUFFER_SIZE, 4);
	if(!pAudio_Encode_Para->Bit_Stream_Buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	pAudio_Encode_Para->write_index = (INT32U)NULL;
	pAudio_Encode_Para->FileLenth = (INT32U)NULL;
	pAudio_Encode_Para->PackSize = C_VR_RECORD_BUFFER_SIZE<<1;
	pAudio_Encode_Para->pack_buffer = NULL;
	pAudio_Encode_Para->PCMInFrameSize = C_VR_RECORD_BUFFER_SIZE;
	pAudio_Encode_Para->OnePCMFrameSize = C_VR_RECORD_BUFFER_SIZE;

	return 0;
}

INT32S vr_encode_stop(void)
{
	adc_memory_free();
	return 0 ;
}

extern void show_result_on_IOA(INT16U index);
INT32S vr_encode_once(void *workmem, const short* buffer_addr, int cwlen)
{
	CSpotter_ProcessOnce( (short*) buffer_addr , cwlen ) ;
	return cwlen ;
}

#endif
#if APP_AUDIO_STREAM_EN
#include "wifi_demo.h"
AUDIO_STREAM_CBK audio_stream_cbk = NULL;
AUDIO_STREAM_GET_FRAME_SIZE audio_stream_get_frame_size = NULL;
void audio_stream_register_cbk(void* cbk, void* get_frame_size)
{
	audio_stream_cbk = (AUDIO_STREAM_CBK)cbk;
	audio_stream_get_frame_size = (AUDIO_STREAM_GET_FRAME_SIZE)get_frame_size;
}

INT32S audio_stream_encode_start(void)
{
	INT32U frame_szie = C_AUDIO_STREAM_BUFFER_SIZE;
	pAudio_Encode_Para->Bit_Stream_Buffer = (INT8U *)gp_malloc_align(C_BS_BUFFER_SIZE, 4);
	if(!pAudio_Encode_Para->Bit_Stream_Buffer)
		return AUD_RECORD_MEMORY_ALLOC_ERR;

	if(audio_stream_get_frame_size != NULL)
		frame_szie = audio_stream_get_frame_size();

	pAudio_Encode_Para->read_index = 0;
	pAudio_Encode_Para->FileLenth = 0;
	pAudio_Encode_Para->PackSize = frame_szie<<1;
	pAudio_Encode_Para->pack_buffer = NULL;
	pAudio_Encode_Para->PCMInFrameSize = frame_szie;
	pAudio_Encode_Para->OnePCMFrameSize = frame_szie;

	return 0;
}

INT32S audio_stream_encode_stop(void)
{
	return 0;
}

INT32S audio_stream_encode_once(void *workmem, const short* buffer_addr, int cwlen)
{

	//DBG_PRINT("Stream encode once buf 0x%x, len %d\r\n", buffer_addr, cwlen);
	if(audio_stream_cbk)
	{
		audio_stream_cbk((short*)buffer_addr, cwlen);
	}
	return 0;
}
#endif
