#include "turnkey_audio_decoder_task.h"
#if (APP_AUDIO_BG_EN == 1) || ( APP_G_MIDI_DECODE_EN == 1)
// Constant definitions used in this file only go here
#define AUDIO_QUEUE_MAX  10
#define AUDIO_FS_Q_SIZE  1
#define AUDIO_WRITE_Q_SIZE MAX_DAC_BUFFERS
#define AUDIO_PARA_MAX_LEN  sizeof(STAudioTaskPara)
#define EDD_STEP    6

/* external varaible */
extern STAudioConfirm  aud_con;

/* Task Q declare */
MSG_Q_ID AudioBGTaskQ;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
OS_EVENT	*audio_bg_wq;
void		*write_bg_q[AUDIO_WRITE_Q_SIZE];
OS_EVENT	*audio_bg_fsq;
void		*fs_bg_q[AUDIO_FS_Q_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
xQueueHandle audio_bg_wq;
xQueueHandle audio_bg_fsq;

extern MSG_Q_ID	Audio_FG_status_Q, Audio_BG_status_Q, MIDI_status_Q;
extern xQueueHandle *hAudioDacTaskQ, *aud_send_q, *aud_bg_send_q, *aud_avi_q, *aud_right_q;
extern MSG_Q_ID	fs_msg_q_id, fs_scan_msg_q_id;
#else
#error "Audio Decode Not Support In Non OS System!"
#endif
INT8U       audio_bg_para[AUDIO_PARA_MAX_LEN];

//INT32U      start_play_time;
INT32S      BG_Volume;
INT16S      *pcm_bg_out[MAX_DAC_BUFFERS] = {NULL};
INT32U      pcm_bg_len[MAX_DAC_BUFFERS];
INT16S      *buf_left;
INT8S       edd_step;
static INT8U audio_stop_flag;
static INT32S bg_error_cnt;

INT32U global_midi_index = 0;//Move here for midi random control.

AUDIO_CONTEXT   audio_bg_context;
AUDIO_CONTEXT_P audio_bg_context_p = &audio_bg_context;
AUDIO_CTRL      audio_bg_ctrl;
INT8U  channel;
INT8U  stopped_bg;
INT32U sample_rate;


/* Proto types */
void audio_bg_task_init(void);
void audio_bg_task_entry(void *p_arg);

static void    audio_bg_init(void);

#if APP_G_MIDI_DECODE_EN == 1
static INT32S  audio_midi_play_init(void);
static INT32S  audio_midi_process(void);
#endif

#if APP_WAV_CODEC_BG_EN == 1
static INT32S  audio_wav_dec_play_init(void);
static INT32S  audio_wav_dec_process(void);
#endif

#if APP_MP3_DECODE_BG_EN == 1
static INT32S  audio_mp3_play_init(void);
static INT32S  audio_mp3_process(void);
#endif

#if APP_A1800_DECODE_BG_EN == 1
static INT32S  audio_a1800_play_init(void);//080724
static INT32S  audio_a1800_process(void);
#endif

#if APP_WMA_DECODE_BG_EN == 1
static INT32S  audio_wma_play_init(void);
static INT32S  audio_wma_process(void);
#endif

#if APP_A6400_DECODE_BG_EN == 1
static INT32S  audio_a64_play_init(void);		// added by Bruce, 2008/09/30
static INT32S  audio_a64_process(void);	// added by Bruce, 2008/09/30
#endif

#if APP_A1600_DECODE_BG_EN == 1
static INT32S  audio_a16_play_init(void);		// added by Bruce, 2008/10/02
static INT32S  audio_a16_process(void);	// added by Bruce, 2008/10/02
#endif

#if APP_S880_DECODE_BG_EN == 1
static INT32S  audio_s880_play_init(void);		// added by Bruce, 2008/10/02
static INT32S  audio_s880_process(void);	// added by Bruce, 2008/10/02
#endif

#if APP_AAC_DECODE_BG_EN == 1
static INT32S  audio_aac_play_init(void);
static INT32S  audio_aac_process(void);
#endif

#if APP_OGG_DECODE_BG_EN == 1
static INT32S  audio_ogg_play_init(void);
static INT32S  audio_ogg_process(void);
#endif

#if SUPPORT_MIDI_READ_FROM_SDRAM == 1
INT32S SPU_load_tonecolor_from_SDRAM(INT16S fd_idi, INT32S MIDI_Index);	// added by Bruce, 2008/10/31
INT32S SPU_check_fill_midi_ring_buffer_from_SDRAM(void);						// added by Bruce, 2008/10/31
#endif

static INT32S  audio_bg_write_with_file_srv(INT8U *ring_buf, INT32U wi, INT32U ri);
static INT32S  audio_bg_check_wi(INT32S wi_in, INT32U *wi_out, INT8U wait);

static void    audio_bg_queue_clear(void);
static void    audio_bg_stop_unfinished(void);
static void    audio_bg_send_next_frame_q(void);
static void    audio_spu_restart(void);
static void    audio_bg_ramp_down(void);

static void    audio_bg_start(STAudioTaskPara *pAudioTaskPara);
static void    audio_bg_pause(STAudioTaskPara *pAudioTaskPara);
static void    audio_bg_resume(STAudioTaskPara *pAudioTaskPara);
static void    audio_bg_stop(STAudioTaskPara *pAudioTaskPara);
static void    audio_bg_decode_next_frame(STAudioTaskPara *pAudioTaskPara);
static void    audio_bg_mute_set(STAudioTaskPara *pAudioTaskPara);
static void    audio_bg_volume_set(STAudioTaskPara *pAudioTaskPara);
static INT32S  audio_bg_q_check(void);

#if APP_MP3_DECODE_BG_EN == 1 || APP_WMA_DECODE_BG_EN == 1 || APP_WAV_CODEC_BG_EN == 1
static void    audio_dac_pcm_data_adj(INT16S *pcm_buf, INT32U len);
static INT32S  audio_send_to_spu(void);
#endif

void audio_bg_task_init(void)
{
    /* Create MsgQueue/MsgBox for TASK */
    AudioBGTaskQ = msgQCreate(AUDIO_QUEUE_MAX, AUDIO_QUEUE_MAX, AUDIO_PARA_MAX_LEN);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
    audio_bg_wq = OSQCreate(write_bg_q, AUDIO_WRITE_Q_SIZE);
    audio_bg_fsq = OSQCreate(fs_bg_q, AUDIO_FS_Q_SIZE);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    audio_bg_wq = xQueueCreate(AUDIO_WRITE_Q_SIZE, sizeof(INT32U));
    audio_bg_fsq = xQueueCreate(AUDIO_FS_Q_SIZE, sizeof(INT32U));
#endif
}

void audio_bg_task_entry(void *p_arg)
{
    INT32U  msg_id;
	STAudioTaskPara     *pstAudioTaskPara;

	audio_bg_task_init();
	audio_bg_init();

	while (1)
	{
	    /* Pend task message */
	    msgQReceive(AudioBGTaskQ, &msg_id, (void*)audio_bg_para, AUDIO_PARA_MAX_LEN);
	    pstAudioTaskPara = (STAudioTaskPara*) audio_bg_para;

		switch(msg_id) {
			case MSG_AUD_PLAY: /* by file handle */
			case MSG_AUD_PLAY_BY_SPI:
				audio_bg_start(pstAudioTaskPara);
				break;
			case MSG_AUD_STOP:
				audio_bg_stop(pstAudioTaskPara);
				break;
			case MSG_AUD_PAUSE:
				audio_bg_pause(pstAudioTaskPara);
				break;
			case MSG_AUD_RESUME:
				audio_bg_resume(pstAudioTaskPara);
				break;
			case MSG_AUD_SET_MUTE:
				audio_bg_mute_set(pstAudioTaskPara);
				break;
			case MSG_AUD_VOLUME_SET:
				audio_bg_volume_set(pstAudioTaskPara);
				break;
			case MSG_AUD_DECODE_NEXT_FRAME:
				audio_bg_decode_next_frame(pstAudioTaskPara);
				break;
			case MSG_AUD_SPU_RESTART:
				audio_spu_restart();
				break;

			default:
				break;
		}
	}
}

static void audio_bg_init(void)
{
	audio_bg_ctrl.ring_buf = (INT8U*) gp_malloc(BG_RING_BUF_SIZE);

	audio_bg_ctrl.wi = 0;
	audio_bg_ctrl.ri = 0;
	audio_bg_context_p->state = AUDIO_PLAY_STOP;
	stopped_bg = 1;
	audio_stop_flag = 0;
	BG_Volume = 63;

	decode_end = NULL;
}

static void audio_bg_start(STAudioTaskPara *pAudioTaskPara)
{
	INT32S ret;

	if (audio_bg_context_p->state != AUDIO_PLAY_STOP) {
		audio_bg_stop_unfinished();
	}

	stopped_bg = 1;
	audio_bg_queue_clear();

	audio_bg_context_p->source_type = pAudioTaskPara->src_type;

	if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_FS) {
		DBG_PRINT("Play FS Audio...\n\r");
		ret = audio_play_file_set(pAudioTaskPara->fd, audio_bg_context_p, &audio_bg_ctrl, audio_bg_fsq);
		global_midi_index = pAudioTaskPara ->src_id;
	}
	else if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)	// added by Bruce, 2008/10/27
	{
		DBG_PRINT("Play User Define Audio...\n\r");
		audio_bg_context_p->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_bg_context_p->audio_format == AUDIO_TYPE_NONE) {
		   DBG_PRINT("audio_play_file_set find not support audio.\r\n");
	   	}

		audio_bg_ctrl.file_len = pAudioTaskPara->file_len;
		global_midi_index = pAudioTaskPara ->src_id;
		ret = AUDIO_ERR_NONE;
	}
	else if(audio_bg_context_p->source_type == AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE)	// added by Bruce, 2010/01/22
	{
		audio_bg_context_p->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_bg_context_p->audio_format == AUDIO_TYPE_NONE)
	   	{
		   DBG_PRINT("audio_play_file_set find not support audio.\r\n");
	   	}
		audio_bg_ctrl.file_handle = pAudioTaskPara->fd;
		audio_bg_ctrl.file_len = pAudioTaskPara->file_len;
		global_midi_index = pAudioTaskPara ->src_id;
		ret = AUDIO_ERR_NONE;
	}
   	else { /* resource file*/
		audio_bg_ctrl.file_len = pAudioTaskPara->file_len;
		audio_bg_ctrl.file_handle = pAudioTaskPara ->fd;
		audio_bg_context_p ->audio_format = pAudioTaskPara->audio_format;
		ret = AUDIO_ERR_NONE;
	}

	if (ret == AUDIO_ERR_NONE) {
		if (audio_mem_alloc(audio_bg_context_p->audio_format,&audio_bg_ctrl,pcm_bg_out) != AUDIO_ERR_NONE) {
			DBG_PRINT("audio memory allocate fail\r\n");
			audio_BG_send_result(MSG_AUD_BG_PLAY_RES,AUDIO_ERR_MEM_ALLOC_FAIL);
			return;
		}

		if (audio_bg_context_p->audio_format != AUDIO_TYPE_MIDI) {
		    buf_left = (INT16S*)gp_malloc(8192);
		    if (buf_left == NULL) {
			    DBG_PRINT("left channel memory allocate fail\r\n");
				audio_BG_send_result(MSG_AUD_BG_PLAY_RES,AUDIO_ERR_MEM_ALLOC_FAIL);
			    return;
		    }
		}

		audio_bg_context_p->state = AUDIO_PLAYING;
		audio_bg_ctrl.reading = 0;

		switch(audio_bg_context_p->audio_format) {
		#if APP_MP3_DECODE_BG_EN == 1
			case AUDIO_TYPE_MP3:
				audio_bg_context_p->fp_deocde_init = audio_mp3_play_init;
				audio_bg_context_p->fp_deocde = audio_mp3_process;
				audio_bg_ctrl.ring_size = BG_RING_BUF_SIZE;
				audio_bg_ctrl.frame_size = MP3_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_WAV_CODEC_BG_EN == 1
			case AUDIO_TYPE_WAV:
				audio_bg_context_p->fp_deocde_init = audio_wav_dec_play_init;
				audio_bg_context_p->fp_deocde = audio_wav_dec_process;
				audio_bg_ctrl.ring_size = WAV_DEC_BITSTREAM_BUFFER_SIZE;
				audio_bg_ctrl.frame_size = WAV_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_WMA_DECODE_BG_EN == 1
			case AUDIO_TYPE_WMA:
				DBG_PRINT("Audio Play WMA Init....\r\n");
				audio_bg_context_p->fp_deocde_init = audio_wma_play_init;
				audio_bg_context_p->fp_deocde = audio_wma_process;
				audio_bg_ctrl.ring_size = BG_RING_BUF_SIZE;
				audio_bg_ctrl.frame_size = WMA_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A1800_DECODE_BG_EN == 1
			case AUDIO_TYPE_A1800:
				audio_bg_context_p->fp_deocde_init = audio_a1800_play_init;
				audio_bg_context_p->fp_deocde = audio_a1800_process;
				audio_bg_ctrl.ring_size = A18_DEC_BITSTREAM_BUFFER_SIZE;
				audio_bg_ctrl.frame_size = A1800_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_G_MIDI_DECODE_EN == 1
			case AUDIO_TYPE_MIDI:
				audio_bg_context_p->fp_deocde_init = audio_midi_play_init;
				audio_bg_context_p->fp_deocde = audio_midi_process;
				break;
		#endif
		#if APP_A1600_DECODE_BG_EN == 1
			case AUDIO_TYPE_A1600:
				audio_bg_context_p->fp_deocde_init = audio_a16_play_init;
				audio_bg_context_p->fp_deocde = audio_a16_process;
				audio_bg_ctrl.ring_size = A16_DEC_BITSTREAM_BUFFER_SIZE;
				audio_bg_ctrl.frame_size = A1600_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A6400_DECODE_BG_EN == 1
			case AUDIO_TYPE_A6400:
				audio_bg_context_p->fp_deocde_init = audio_a64_play_init;
				audio_bg_context_p->fp_deocde = audio_a64_process;
				audio_bg_ctrl.ring_size = BG_RING_BUF_SIZE;
				audio_bg_ctrl.frame_size = A6400_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_S880_DECODE_BG_EN == 1
			case AUDIO_TYPE_S880:
				audio_bg_context_p->fp_deocde_init = audio_s880_play_init;
				audio_bg_context_p->fp_deocde = audio_s880_process;
				audio_bg_ctrl.ring_size = S880_DEC_BITSTREAM_BUFFER_SIZE;
				audio_bg_ctrl.frame_size = S880_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_AAC_DECODE_BG_EN == 1
			case AUDIO_TYPE_AAC:
				audio_bg_context_p->fp_deocde_init = audio_aac_play_init;
				audio_bg_context_p->fp_deocde = audio_aac_process;
				audio_bg_ctrl.ring_size = AAC_DEC_BITSTREAM_BUFFER_SIZE;
				audio_bg_ctrl.frame_size = AAC_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_OGG_DECODE_BG_EN == 1
			case AUDIO_TYPE_OGG:
				audio_bg_context_p->fp_deocde_init = audio_ogg_play_init;
				audio_bg_context_p->fp_deocde = audio_ogg_process;
				audio_bg_ctrl.ring_size = OGGVORBIS_DEC_BITSTREAM_BUFFER_SIZE;
				audio_bg_ctrl.frame_size = OGG_PCM_BUF_SIZE;
				break;
		#endif
			default:
				audio_BG_send_result(MSG_AUD_BG_PLAY_RES,AUDIO_ERR_INVALID_FORMAT);//080904
				return;
		}

		ret = audio_bg_context_p->fp_deocde_init();
		if (ret != AUDIO_ERR_NONE) {
			audio_bg_stop_unfinished();
			DBG_PRINT("audio play init failed\r\n");
        }
        else {
        	audio_bg_send_next_frame_q();
		}
	}
	else {
    	ret = AUDIO_ERR_INVALID_FORMAT;
	}

	audio_BG_send_result(MSG_AUD_BG_PLAY_RES,ret);
}

static void audio_bg_stop(STAudioTaskPara *pAudioTaskPara)
{
	if ((audio_bg_context_p->state == AUDIO_PLAY_STOP) || (audio_bg_context_p->state == AUDIO_IDLE)) {
		audio_BG_send_result(MSG_AUD_BG_STOP_RES,AUDIO_ERR_NONE);
		return;
	}

	if (audio_bg_context_p->audio_format != AUDIO_TYPE_MIDI) {
		if (audio_bg_context_p->state == AUDIO_PLAYING) {
	    	audio_bg_ramp_down();
	    }
	}

	audio_bg_stop_unfinished();
	audio_stop_flag = 0;
	audio_BG_send_result(MSG_AUD_BG_STOP_RES,AUDIO_ERR_NONE);
}

static void audio_bg_ramp_down(void)
{
	INT32S    ret = 0;
    INT32U message = MSG_AUD_RAMP_DOWN_START;
	audio_stop_flag = 1;
	edd_step = EDD_STEP;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_RAMP_DOWN_START);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS) //add for porting
    xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
#endif
	while(1) {
		ret = audio_bg_context_p->fp_deocde();
		if (ret != 0) {
			break;
		}
		if ((SPU_ReadEnvelopeData(SPU_LEFT_CH) == 0) && (SPU_ReadEnvelopeData(SPU_RIGHT_CH) == 0)) {
			SPU_StopChannel(SPU_LEFT_CH);
			SPU_StopChannel(SPU_RIGHT_CH);
			break;
		}
	}

	if (ret != 0) {
		audio_BG_send_result(MSG_AUD_BG_PLAY_RES,ret);
	}
}

static void audio_bg_pause(STAudioTaskPara *pAudioTaskPara)
{
	if (audio_bg_context_p->state != AUDIO_PLAYING) {
		audio_BG_send_result(MSG_AUD_BG_PAUSE_RES, AUDIO_ERR_NONE);
		return;
	}

	if (audio_bg_context_p->audio_format == AUDIO_TYPE_MIDI) {
		SPU_MIDI_Pause();
	}
	else {
		INT32U message = MSG_AUD_SPU_RIGHT_DONE;

		audio_bg_ramp_down();
		SPU_Disable_FIQ_Channel(SPU_LEFT_CH);
		SPU_Disable_FIQ_Channel(SPU_RIGHT_CH);
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(aud_right_q, (void *)MSG_AUD_SPU_RIGHT_DONE);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(aud_right_q, &message, portMAX_DELAY);
    #endif
		audio_stop_flag = 0;
		stopped_bg = 1;
	}

	//dac_enable_set(FALSE);
	audio_bg_context_p->state = AUDIO_PLAY_PAUSE;
	audio_BG_send_result(MSG_AUD_BG_PAUSE_RES,AUDIO_ERR_NONE);
}

static void audio_bg_resume(STAudioTaskPara *pAudioTaskPara)
{
#if 0
	OS_Q      *pq;
	INT32U    idx = 0;
	INT32U     i;

	if (audio_bg_context_p->state != AUDIO_PLAY_PAUSE) {
		audio_BG_send_result(MSG_AUD_BG_RESUME_RES, AUDIO_ERR_NONE);
		return;
	}

	audio_bg_context_p->state = AUDIO_PLAYING;

	if (audio_bg_context_p->audio_format == AUDIO_TYPE_MIDI) {
		SPU_MIDI_Resume();
	}
	else {
		pq = (OS_Q *)aud_bg_send_q->OSEventPtr;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
                OSQFlush(audio_bg_wq);
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                xQueueReset(audio_bg_wq);
                #endif
		if (pq->OSQEntries) {
			idx = (INT32U) *pq->OSQOut;
		}

		for (i=0;i<(pq->OSQSize-pq->OSQEntries);i++) {
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
                        OSQPost(audio_bg_wq, (void *) ((idx+pq->OSQEntries+i)%MAX_DAC_BUFFERS));
                        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                        xQueueSend(audio_bg_wq, (void *) ((idx+pq->OSQEntries+i)%MAX_DAC_BUFFERS), portMAX_DELAY);
                        #endif
		}
		OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_SPU_WIDX_SET);

		if (audio_bg_ctrl.reading) {
			audio_bg_check_wi(AUDIO_READ_WAIT, &audio_bg_ctrl.wi, 1);
		}
	}
	audio_bg_send_next_frame_q();
	audio_BG_send_result(MSG_AUD_BG_RESUME_RES,AUDIO_ERR_NONE);
#endif
}

INT8U audio_bg_getstatus(void)
{
	return audio_bg_context_p->state;
}


static void audio_bg_mute_set(STAudioTaskPara *pAudioTaskPara)
{
	if (pAudioTaskPara->mute == TRUE) {
		SPU_Set_MainVolume(0);
		BG_Volume = 0;
	}
	else {
		SPU_Set_MainVolume(pAudioTaskPara->volume);
		BG_Volume = pAudioTaskPara->volume;
	}
	audio_BG_send_result(MSG_AUD_MUTE_SET_RES,AUDIO_ERR_NONE);
}

static void audio_bg_volume_set(STAudioTaskPara *pAudioTaskPara)
{
	if (pAudioTaskPara->volume < 128) {
		BG_Volume = pAudioTaskPara->volume;
		//SPU_Set_MainVolume(pAudioTaskPara->volume);
		SPU_SetVelocity(pAudioTaskPara->volume, SPU_LEFT_CH);
		SPU_SetVelocity(pAudioTaskPara->volume, SPU_RIGHT_CH);
	}
	audio_BG_send_result(MSG_AUD_VOLUME_SET_RES,AUDIO_ERR_NONE);
}

static void audio_bg_decode_next_frame(STAudioTaskPara *pAudioTaskPara)
{
	INT32S ret;
	if (audio_bg_context_p->state != AUDIO_PLAYING) {
		return;
	}

	ret = audio_bg_context_p->fp_deocde();
	if (ret != 0) {
		audio_bg_stop_unfinished();
		if (decode_end != NULL) {
			decode_end(0);	// added by Bruce, 2008/10/03
		}
	}
}

static void audio_bg_send_next_frame_q(void)
{
	if (audio_stop_flag) {
		return;
	}
	msgQSend(AudioBGTaskQ, MSG_AUD_DECODE_NEXT_FRAME, NULL, 0, MSG_PRI_NORMAL);
}

void audio_BG_send_result(INT32S res_type,INT32S result)
{
	aud_con.result_type = res_type;
	aud_con.result = result;
	DBG_PRINT("audio_BG_send_result :res_type =  %x,result = %d\r\n",res_type,result);
	//msgQSend(ApQ, EVENT_APQ_ERR_MSG, (void *)&aud_con, sizeof(STAudioConfirm), MSG_PRI_NORMAL);
	msgQSend(Audio_BG_status_Q, EVENT_APQ_ERR_MSG, (void *)&aud_con, sizeof(STAudioConfirm), MSG_PRI_NORMAL);
}

static void audio_spu_restart(void)
{
	INT32U count;

	if (audio_bg_context_p->state == AUDIO_PLAY_STOP) {
		return;
	}
	DBG_PRINT("SPU restart..\r\n");
	stopped_bg = 1;

	audio_bg_queue_clear();
	for (count=0;count<MAX_DAC_BUFFERS;count++) {
#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_bg_wq, (void *) count);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)   //add for porting
        xQueueSend(audio_bg_wq, (void *) count, portMAX_DELAY);
#endif
	}
	audio_bg_send_next_frame_q();
}

static void audio_bg_stop_unfinished(void)
{
	INT32S i;

	switch(audio_bg_context_p->audio_format)
	{
		case AUDIO_TYPE_MIDI:
#if SUPPORT_MIDI_LOAD_FROM_NVRAM == 0 && SUPPORT_MIDI_READ_FROM_SDRAM == 0
			fs_close(audio_bg_ctrl.file_handle);
#endif
			SPU_MIDI_Stop();
			SPU_Free_Midi();
			break;
		default:
			if (audio_stop_flag == 0) {
			    while(audio_bg_q_check() != AUDIO_SEND_OK) { /* wait queue empty */
                                #if (_OPERATING_SYSTEM == _OS_UCOS2)
                                OSTimeDly(1);
                                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                                vTaskDelay(1);
                                #endif
			    }
			    SPU_StopChannel(SPU_LEFT_CH);
			    SPU_StopChannel(SPU_RIGHT_CH);
			}

			if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_FS) {
			    fs_close(audio_bg_ctrl.file_handle);
			}

			SPU_Disable_FIQ_Channel(SPU_LEFT_CH);
			SPU_Disable_FIQ_Channel(SPU_RIGHT_CH);
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSQPost(aud_right_q, (void *)MSG_AUD_SPU_RIGHT_DONE);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            {
                INT32U message = MSG_AUD_SPU_RIGHT_DONE;
                xQueueSend(aud_right_q, &message, portMAX_DELAY);
            }
            #endif
			/* free memory */
			if(audio_bg_context_p->audio_format != AUDIO_TYPE_MP3)
                gp_free(audio_bg_ctrl.work_mem);
			audio_bg_ctrl.work_mem = NULL;
			gp_free(buf_left);
			buf_left = NULL;
			for (i=0;i<MAX_DAC_BUFFERS;i++) {
				gp_free(*(pcm_bg_out+i));
				*(pcm_bg_out+i) = NULL;
			}
			break;
	}
	//dac_enable_set(FALSE);
	audio_bg_context_p->state = AUDIO_PLAY_STOP;
}

static void audio_bg_queue_clear(void)
{
	//OSQFlush(hAudioDacTaskQ);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQFlush(aud_bg_send_q);
	OSQFlush(audio_bg_wq);
	OSQFlush(audio_bg_fsq);
	OSQFlush(aud_right_q);
        OSQPost(hAudioDacTaskQ, (void *)MSG_AUD_SPU_WIDX_CLEAR);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        INT32U message = MSG_AUD_SPU_WIDX_CLEAR;
        xQueueReset(aud_bg_send_q);
        xQueueReset(audio_bg_wq);
        xQueueReset(audio_bg_fsq);
        xQueueReset(aud_right_q);
        xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
#endif
}

INT32S audio_bg_write_with_file_srv(INT8U *ring_buf, INT32U wi, INT32U ri)
{
	INT32S t;
	INT32S len;
	INT8U  err;
	INT32U msg_id;
	TK_FILE_SERVICE_STRUCT audio_fs_para;

	switch(audio_bg_context_p->source_type) {
		case AUDIO_SRC_TYPE_FS:
			msg_id = MSG_FILESRV_FS_READ;
			break;
		case AUDIO_SRC_TYPE_GPRS:
			msg_id = MSG_FILESRV_NVRAM_AUDIO_GPRS_READ;
			audio_fs_para.spi_para.sec_offset = audio_bg_ctrl.read_secs;
			break;
		case AUDIO_SRC_TYPE_APP_RS:
			msg_id = MSG_FILESRV_NVRAM_AUDIO_APP_READ;
			break;
		case AUDIO_SRC_TYPE_APP_PACKED_RS:
			msg_id = MSG_FILESRV_NVRAM_AUDIO_APP_PACKED_READ;
			break;
		case AUDIO_SRC_TYPE_USER_DEFINE:	// added by Bruce, 2008/10/27
			msg_id = MSG_FILESRV_USER_DEFINE_READ;
			audio_fs_para.data_start_addr = audio_bg_ctrl.data_start_addr;
			audio_fs_para.data_offset = audio_bg_ctrl.data_offset;
			break;
		case AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE:	// added by Bruce, 2010/01/22
			msg_id = MSG_FILESRV_FS_READ;
			break;
		default:
			break;
	}

	audio_fs_para.fd = audio_bg_ctrl.file_handle;
	audio_fs_para.result_queue = audio_bg_fsq;
	audio_fs_para.main_channel = AUDIO_CHANNEL_SPU;
#if 1
	if(wi == 0 && ri == 0) {
		audio_fs_para.buf_addr = (INT32U)ring_buf;
		audio_fs_para.buf_size = audio_bg_ctrl.ring_size/2;
		audio_fs_para.spi_para.sec_cnt = audio_bg_ctrl.ring_size/1024;



            msgQSend(fs_msg_q_id, msg_id, (void *)&audio_fs_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
            len = (INT32S) OSQPend(audio_fsq, 0, &err);
            if ((err != OS_NO_ERR) || len < 0) {
        	    return AUDIO_READ_FAIL;
            }
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            err = (INT8U) xQueueReceive(audio_bg_fsq, &len, portMAX_DELAY);
            if((err != pdPASS) || (len < 0)) {
                return AUDIO_READ_FAIL;
            }
        #endif

			if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
				audio_bg_ctrl.data_offset += len;
            audio_bg_ctrl.read_secs += (len/512);
            wi += len;
            return wi;
    }

    len = ri - wi;
    if (len <= 0) {
    	len += audio_bg_ctrl.ring_size;
    }
    if(len < audio_bg_ctrl.ring_size/2) {
    	return wi;
    }

	t = wi;
	wi += audio_bg_ctrl.ring_size/2;
	if(wi == audio_bg_ctrl.ring_size) {
		wi = 0;
	}
	if(wi == ri) {
		return t;
	}

	audio_fs_para.buf_addr = (INT32U)ring_buf+t;
	audio_fs_para.buf_size = audio_bg_ctrl.ring_size/2;

	audio_fs_para.spi_para.sec_cnt = audio_bg_ctrl.ring_size/1024;

    msgQSend(fs_msg_q_id, msg_id, (void *)&audio_fs_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);

	return AUDIO_READ_PEND;

#else
	len = wi - ri;
    if (len < 0) {
    	len += audio_bg_ctrl.ring_size;
    }

	if ((len < 2048) && audio_bg_ctrl.reading) {
		return AUDIO_READ_WAIT;
	}

	if(ri == wi) {
		audio_fs_para.buf_addr = (INT32U)ring_buf+wi;
		audio_fs_para.buf_size = audio_bg_ctrl.ring_size/2;
		audio_fs_para.spi_para.sec_cnt = audio_bg_ctrl.ring_size/1024;

        msgQSend(fs_msg_q_id, msg_id, (void *)&audio_fs_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);
 #if (_OPERATING_SYSTEM == _OS_UCOS2)
        len = (INT32S) OSQPend(audio_bg_fsq, 0, &err);
        if ((err != OS_NO_ERR) || len < 0) {
        	return AUDIO_READ_FAIL;
        }
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        err = (INT8U) xQueueReceive(audio_bg_fsq, &len, portMAX_DELAY);
        if(err != pdPASS || len < 0) {
            return AUDIO_READ_FAIL;
        }
#endif
		if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
			audio_bg_ctrl.data_offset += len;
        audio_bg_ctrl.read_secs += (len/512);
        wi += len;
        if(wi == audio_bg_ctrl.ring_size) {
			wi = 0;
		}
        return wi;
    }

    len = ri - wi;
    if (len < 0) {
    	len += audio_bg_ctrl.ring_size;
    }
    if(len < audio_bg_ctrl.ring_size/2) {
    	return wi;
    }

    /* length = ring_size/2 */
	t = wi;
	wi += audio_bg_ctrl.ring_size/2;
	if(wi == audio_bg_ctrl.ring_size) {
		wi = 0;
	}
	if(wi == ri) {
		return t;
	}

	if (audio_bg_ctrl.reading) {
		return AUDIO_READ_PEND;
	}

	audio_bg_ctrl.reading = 1;

	audio_fs_para.buf_addr = (INT32U)ring_buf+t;
	audio_fs_para.buf_size = audio_bg_ctrl.ring_size/2;

	audio_fs_para.spi_para.sec_cnt = audio_bg_ctrl.ring_size/1024;

	msgQSend(fs_msg_q_id, msg_id, (void *)&audio_fs_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);

	return AUDIO_READ_PEND;
#endif
}

INT32S audio_bg_check_wi(INT32S wi_in, INT32U *wi_out, INT8U wait)
{
	INT32S len;
	INT8U  err;
	INT32S t;

	if (wi_in >= 0) {
		*wi_out = wi_in;
		return AUDIO_ERR_NONE;
	}
	if (wi_in == AUDIO_READ_FAIL) {
		return AUDIO_READ_FAIL;
	}
	/* AUDIO_READ_PEND or AUDIO_READ_WAIT */
	t = audio_bg_ctrl.wi;

	if (wait) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
	    len = (INT32S) OSQPend(audio_bg_fsq, 0, &err);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            err = (INT8U) xQueueReceive(audio_bg_fsq, &len, portMAX_DELAY);
            #endif
	}
	else {
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
                len = (INT32U)OSQAccept(audio_bg_fsq, &err);
		if (err == OS_Q_EMPTY) {
			*wi_out = *wi_out;
			return 	AUDIO_ERR_NONE;
		}
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                err = (INT8U) xQueueReceive(audio_bg_fsq, &len, 0);
                if(err == pdPASS) {
			*wi_out = *wi_out;
			return 	AUDIO_ERR_NONE;
                }
                #endif
	}

	audio_bg_ctrl.reading = 0;
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	if ((err != OS_NO_ERR) || len < 0) {
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        if ((err != pdPASS) || len < 0) {
        #endif
            return AUDIO_READ_FAIL;
    }
	if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
		audio_bg_ctrl.data_offset += len;

    audio_bg_ctrl.read_secs += (len/512);
	t += len;
	if(t == audio_bg_ctrl.ring_size) {
		t = 0;
	}

	*wi_out = t;
	return 	AUDIO_ERR_NONE;
}

static INT32S audio_send_to_spu(void)
{
    INT32U message = MSG_AUD_SPU_SOFTCH_START;
    //DBG_PRINT("%s %d %d\r\n",__func__ ,stopped_bg,audio_bg_q_check());
	if (stopped_bg && (audio_bg_q_check()==AUDIO_SEND_FAIL)) { /* queuq full */
		stopped_bg = 0;
                #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_SPU_SOFTCH_START);
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
                #endif
	}

	return STATUS_OK;
}

static INT32S audio_bg_q_check(void)
{
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OS_Q      *pq;
	pq = (OS_Q *)aud_bg_send_q->OSEventPtr;

	if (pq->OSQEntries >= 2) {//for fast start
		return AUDIO_SEND_FAIL;
	}
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    INT32U Number = *(INT32U *)((INT32U)aud_bg_send_q + 4*4 + sizeof(xList) * 2);
    if(Number >= 2) {
    	return AUDIO_SEND_FAIL;
    }
#endif
	return AUDIO_SEND_OK;
}

//===============================================================================================================
//   MP3 Playback
//===============================================================================================================
#if APP_MP3_DECODE_BG_EN == 1
INT32S audio_mp3_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.f_last = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;
	bg_error_cnt = 0;
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	// mp3 init

	ret = mp3_dec_init((char*)audio_bg_ctrl.work_mem, (unsigned char*)audio_bg_ctrl.ring_buf, (char*)(audio_bg_ctrl.work_mem + MP3_DEC_MEMORY_SIZE));
	mp3_dec_set_bs_buf_size((void*)audio_bg_ctrl.work_mem, audio_bg_ctrl.ring_size);

	audio_bg_ctrl.wi = 0;
	audio_bg_ctrl.ri = mp3_dec_get_ri((void*)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 50;
	while(1)
	{
		in_length = audio_bg_ctrl.ri;
		ret = mp3_dec_parsing((void*)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);

		audio_bg_ctrl.ri = mp3_dec_get_ri((void*)audio_bg_ctrl.work_mem);
		audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
		if(audio_bg_ctrl.ri < in_length) {
			audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
		}

		switch(ret)
		{
			case MP3_DEC_ERR_NONE:
				break;

			case MP3_DEC_ERR_LOSTSYNC:		//not found sync word
			case MP3_DEC_ERR_BADSAMPLERATE:	//reserved sample frequency value
			case MP3_DEC_ERR_BADBITRATE:		//forbidden bitrate value
			case MP3_DEC_ERR_BADLAYER:
			case MP3_DEC_ERR_BADMPEGID:
				//feed in DecodeInBuffer;
				audio_bg_ctrl.ri = mp3_dec_get_ri((void*)audio_bg_ctrl.work_mem);
				t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

				/* check reading data */
				if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_READ_FAIL;
				}

				if (--count == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
				continue;

			default:
				return AUDIO_ERR_DEC_FAIL;
		}

		if(ret == MP3_DEC_ERR_NONE) {
			break;
		}
	}

	sample_rate = mp3_dec_get_samplerate((void*)audio_bg_ctrl.work_mem);
	channel = mp3_dec_get_channel((void*)audio_bg_ctrl.work_mem);


	DBG_PRINT("bps: %d\r\n",mp3_dec_get_bitrate((void*)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",mp3_dec_get_channel((void*)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n",mp3_dec_get_samplerate((void*)audio_bg_ctrl.work_mem));
	DBG_PRINT("block size: %d\r\n",mp3_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
        xQueueSend(audio_bg_wq, &count, portMAX_DELAY);
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_mp3_process(void)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT8U   wait;
	INT32U  wb_idx;
	INT32S  t_wi;

	audio_bg_ctrl.ri = mp3_dec_get_ri((void*)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

    wb_idx = 0;
	xQueueReceive(audio_bg_wq, &wb_idx, portMAX_DELAY);

	wait = 1;

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		if(audio_bg_ctrl.f_last) {
			return AUDIO_ERR_DEC_FINISH;
		} else {
			audio_bg_ctrl.f_last = 1;
		}
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = mp3_dec_run((void*)audio_bg_ctrl.work_mem, pcm_bg_out[wb_idx], audio_bg_ctrl.wi);

	audio_bg_ctrl.ri = mp3_dec_get_ri((void*)audio_bg_ctrl.work_mem);
	if((audio_bg_ctrl.ri == audio_bg_ctrl.wi) && (audio_bg_ctrl.ri == audio_bg_ctrl.ring_size/2)) {
		// fixed ri for audio_bg_write_with_file_srv
		audio_bg_ctrl.ri = in_length + audio_bg_ctrl.ring_size/2;
		if(audio_bg_ctrl.ri >= audio_bg_ctrl.ring_size) {
			audio_bg_ctrl.ri -= audio_bg_ctrl.ring_size;
		}

		mp3_dec_set_ri((void*)audio_bg_ctrl.work_mem, audio_bg_ctrl.ri);
		audio_bg_ctrl.ri = mp3_dec_get_ri((void*)audio_bg_ctrl.work_mem);
	}

	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	if (pcm_point <= 0) {
		if (pcm_point == 0 && ++bg_error_cnt > MP3_FRAME_ERROR_CNT) {
	        return AUDIO_ERR_DEC_FAIL;
	    }

		if (pcm_point < 0) {
			if (--audio_bg_ctrl.try_cnt == 0) {
				return AUDIO_ERR_DEC_FAIL;
			}
		}

		xQueueSendToFront(audio_bg_wq, &wb_idx, portMAX_DELAY);
		audio_bg_send_next_frame_q();
		return 0;
	} else {
		bg_error_cnt = 0;
	}

	pcm_bg_len[wb_idx] = pcm_point * mp3_dec_get_channel((void*)audio_bg_ctrl.work_mem);


	cache_invalid_range((INT32U)pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	xQueueSend(aud_bg_send_q, &wb_idx, portMAX_DELAY);

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	return 0;
}
#endif // #if APP_MP3_DECODE_BG_EN == 1

//===============================================================================================================
//   WMA Playback
//===============================================================================================================
#if APP_WMA_DECODE_BG_EN == 1
int audio_wma_play_init()
{
	INT32S count;
	INT32U in_length;
	INT32S ret;
	INT32S  t_wi;

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;
	bg_error_cnt = 0;
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	//wma init
	ret = wma_dec_init((char *)audio_bg_ctrl.work_mem, audio_bg_ctrl.ring_buf,(char *)audio_bg_ctrl.work_mem + 8192,audio_bg_ctrl.ring_size);
	audio_bg_ctrl.wi = wma_dec_get_ri((char *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.ri = wma_dec_get_ri((char *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

#if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT == GP_FILE_FORMAT_SET_1)
	for (count=0;count<512;count++) {
        audio_bg_ctrl.ring_buf[count] ^= 0xFF;
    }
#endif

	count = 50;
	while(1)
	{
		in_length = audio_bg_ctrl.ri;
		ret = wma_dec_parsing((char *)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);

		audio_bg_ctrl.ri = wma_dec_get_ri((char *)audio_bg_ctrl.work_mem);
		audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
		if(audio_bg_ctrl.ri < in_length) {
			audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
		}

		switch(ret)
		{
			case WMA_OK:
				break;

			case WMA_E_BAD_PACKET_HEADER:
			case WMA_E_INVALIDHEADER:
			case WMA_E_NOTSUPPORTED:
			case WMA_E_NOTSUPPORTED_CODEC:
				return AUDIO_ERR_INVALID_FORMAT;

			case WMA_E_ONHOLD:
			case WMA_E_DATANOTENOUGH:
				//feed in DecodeInBuffer;
				audio_bg_ctrl.ri = wma_dec_get_ri((char *)audio_bg_ctrl.work_mem);
				t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

				/* check reading data */
				if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_READ_FAIL;
				}

				if (--count == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
				continue;

			default:
				if (--count == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
				break;
		}

		if(ret == WMA_OK) {
			break;
		}
	}

	sample_rate = wma_dec_get_samplerate((char *)audio_bg_ctrl.work_mem);
	channel = wma_dec_get_channel((char *)audio_bg_ctrl.work_mem);

	DBG_PRINT("bps: %d\r\n", wma_dec_get_bitrate((char*)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", wma_dec_get_channel((char *)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", wma_dec_get_samplerate((char*)audio_bg_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		OSQPost(audio_bg_wq, (void *) count);
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_wma_process()
{
	INT32S	pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT8U   wait;
	INT32U  wb_idx;
	INT32S  t_wi;
	INT32S  offset;
	INT32S  len;

	audio_bg_ctrl.ri = wma_dec_get_ri((char *)audio_bg_ctrl.work_mem);

	offset = wma_dec_get_offset((char *)audio_bg_ctrl.work_mem);

	if(offset > 0) {
		len = audio_bg_ctrl.wi - audio_bg_ctrl.ri;
		if(len < 0) {
			len += audio_ctrl.ring_size;
		}

		audio_bg_ctrl.file_cnt += offset;
		if(len > offset) {
			audio_bg_ctrl.ri += offset;
			if(audio_bg_ctrl.ri >= audio_ctrl.ring_size) {
				audio_bg_ctrl.ri -= audio_ctrl.ring_size;
			}

			wma_dec_set_ri((char *)audio_bg_ctrl.work_mem, audio_bg_ctrl.ri);
		} else {
			offset -= len;
			if(lseek(audio_bg_ctrl.file_handle, offset, SEEK_CUR) < 0) {
				return AUDIO_ERR_DEC_FAIL;
			}

			audio_bg_ctrl.wi = audio_bg_ctrl.ri = 0;
			wma_dec_set_ri((char *)audio_bg_ctrl.work_mem, audio_bg_ctrl.ri);
		}

		wma_dec_reset_offset((char *)audio_bg_ctrl.work_mem);
	}

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);

	wait = 0;
	if (t_wi == AUDIO_READ_WAIT) {
		wait = 1; /* pend until read finish */
	}

	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) { /* check reading data */
		return AUDIO_ERR_READ_FAIL;
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = wma_dec_run((char *)audio_bg_ctrl.work_mem,pcm_bg_out[wb_idx],audio_bg_ctrl.wi);

	audio_bg_ctrl.ri = wma_dec_get_ri((char *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len){
		return AUDIO_ERR_DEC_FINISH;
	}

	if (pcm_point <= 0) {
	    if (pcm_point == 0 && ++bg_error_cnt > 50) {
	        return AUDIO_ERR_DEC_FAIL;
	    }

		if (pcm_point < 0) {
			if (wma_dec_get_errno((char *)audio_bg_ctrl.work_mem) == WMA_E_NO_MORE_FRAMES) {
				return AUDIO_ERR_DEC_FINISH;
			} else {
				return AUDIO_ERR_DEC_FAIL;
			}
		}

		OSQPostFront(audio_bg_wq, (void *)wb_idx);
		audio_bg_send_next_frame_q();
		return 0;
	} else {
	    bg_error_cnt = 0;
	}

	pcm_bg_len[wb_idx] = pcm_point * wma_dec_get_channel((char *)audio_bg_ctrl.work_mem);

	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	OSQPost(aud_bg_send_q, (void *) wb_idx);
	audio_send_to_spu();
	audio_bg_send_next_frame_q();

	return 0;
}
#endif	// #if APP_WMA_DECODE_BG_EN == 1

//===============================================================================================================
//   WAVE Playback
//===============================================================================================================
#if APP_WAV_CODEC_BG_EN == 1
INT32S audio_wav_dec_play_init()
{
	INT32U in_length;
	INT32S count;
	INT32S t_wi;
	INT8U  bit_per_samp;

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.try_cnt = 10;
	audio_bg_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	wav_dec_init((INT8U *)audio_bg_ctrl.work_mem, audio_bg_ctrl.ring_buf);
	wav_dec_set_ring_buf_size((INT8U *)audio_bg_ctrl.work_mem, audio_bg_ctrl.ring_size);
	audio_bg_ctrl.wi = wav_dec_get_ri((INT8U *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.ri = wav_dec_get_ri((INT8U *)audio_bg_ctrl.work_mem);

	in_length = audio_bg_ctrl.ri;
	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	audio_bg_ctrl.file_len = wav_dec_parsing((INT8U *)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);
	if((int)(audio_bg_ctrl.file_len) <= 0) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_bg_ctrl.ri = wav_dec_get_ri((INT8U *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	sample_rate = wav_dec_get_SampleRate((INT8U *)audio_bg_ctrl.work_mem);
	channel = wav_dec_get_nChannels((INT8U *)audio_bg_ctrl.work_mem);
	bit_per_samp = wav_dec_get_wBitsPerSample((INT8U *)audio_bg_ctrl.work_mem);

	if(sample_rate == 0 || channel == 0 || bit_per_samp == 0) {
	    return AUDIO_ERR_DEC_FAIL;
	}

	DBG_PRINT("channel: %d\r\n",channel);
	DBG_PRINT("sample rate: %d\r\n",wav_dec_get_SampleRate((INT8U*)audio_bg_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_bg_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_bg_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_wav_dec_process()
{
	INT32S	pcm_point;
	INT32U  in_length;
	INT32S  tmp_len;
	INT32U  wb_idx;
	INT8U   err;
	INT8U   wait;
	INT32S  t_wi;

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		return AUDIO_ERR_DEC_FINISH;
	}

	audio_bg_ctrl.ri = wav_dec_get_ri((INT8U *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);
    //DBG_PRINT("wi %d ri %d twi %d\r\n",audio_bg_ctrl.wi,audio_bg_ctrl.ri,t_wi);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_bg_wq, &wb_idx, portMAX_DELAY);
#endif

	wait = 1; /* pend until read finish */

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		return AUDIO_ERR_DEC_FINISH;
	}

	in_length = audio_bg_ctrl.wi - audio_bg_ctrl.ri;
	if(audio_bg_ctrl.wi < audio_bg_ctrl.ri) {
		in_length += audio_bg_ctrl.ring_size;
	}

	if(in_length > (audio_bg_ctrl.file_len-audio_bg_ctrl.file_cnt)) {
		tmp_len = audio_bg_ctrl.wi;
		tmp_len -= (in_length - audio_bg_ctrl.file_len + audio_bg_ctrl.file_cnt);
		if(tmp_len < 0) {
			tmp_len += audio_bg_ctrl.ring_size;
		}
		audio_bg_ctrl.wi = tmp_len;
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = wav_dec_run((INT8U *)audio_bg_ctrl.work_mem,pcm_bg_out[wb_idx],audio_bg_ctrl.wi);
	if (pcm_point == -1) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_bg_ctrl.ri = wav_dec_get_ri((INT8U *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += WAV_DEC_BITSTREAM_BUFFER_SIZE;
	}

	if (pcm_point <= 0) {
		if (--audio_bg_ctrl.try_cnt == 0) {
			return AUDIO_ERR_DEC_FAIL;
		}
#if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPostFront(audio_bg_wq, (void *)wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		xQueueSendToFront(audio_bg_wq, &wb_idx, portMAX_DELAY);
#endif
		audio_bg_send_next_frame_q();
		return 0;
	}

	pcm_bg_len[wb_idx] = pcm_point * wav_dec_get_nChannels((INT8U *)audio_bg_ctrl.work_mem);
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_bg_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_bg_send_q, &wb_idx, portMAX_DELAY);
#endif

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	return 0;
}
#endif // #if APP_WAV_CODEC_BG_EN == 1

//===============================================================================================================
//   A1800 Playback
//===============================================================================================================
#if APP_A1800_DECODE_BG_EN == 1
INT32S audio_a1800_play_init(void)
{
	INT32U  in_length;
	INT32S  ret;
	INT32U  count;
	INT32S  t_wi;

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;//080724
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	//a1800 init
	ret = a1800dec_GetMemoryBlockSize();//080723
	if(ret != A1800DEC_MEMORY_BLOCK_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = a1800dec_init((void *)audio_bg_ctrl.work_mem, (const unsigned char *)audio_bg_ctrl.ring_buf);//080723
	A18_dec_SetRingBufferSize((void *)audio_bg_ctrl.work_mem, audio_bg_ctrl.ring_size);
	audio_bg_ctrl.wi = a1800dec_read_index((void *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.ri = a1800dec_read_index((void *)audio_bg_ctrl.work_mem);
	in_length = (audio_bg_ctrl.ri);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	ret = a1800dec_parsing((void *)audio_bg_ctrl.work_mem, audio_bg_ctrl.wi);
	if(ret != 1) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_bg_ctrl.ri = a1800dec_read_index((INT8U *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	sample_rate = A18_dec_get_samplerate((INT8U*)audio_bg_ctrl.work_mem);
	channel = A18_dec_get_channel((INT8U*)audio_bg_ctrl.work_mem);

	DBG_PRINT("bps: %d\r\n", A18_dec_get_bitrate((void *)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", A18_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", A18_dec_get_samplerate((CHAR*)audio_bg_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_bg_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_bg_wq, &count, portMAX_DELAY);
    #endif
	}
    DBG_PRINT("AUDIO_ERR_NONE\r\n");
	return AUDIO_ERR_NONE;
}

INT32S  audio_a1800_process(void)
{
	INT32S pcm_point;
	INT32U  in_length;
	INT32U  wb_idx;
	INT8U   err;
	INT8U   wait;
	INT32S  t_wi;

	audio_bg_ctrl.ri = a1800dec_read_index((INT8U *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

    //DBG_PRINT("wi %d ri %d twi %d\r\n",audio_bg_ctrl.wi,audio_bg_ctrl.ri,t_wi);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_bg_wq, &wb_idx, portMAX_DELAY);
#endif

	//wait = 0;
	//if (t_wi == AUDIO_READ_WAIT) {
		wait = 1; /* pend until read finish */
	//}

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
        //DBG_PRINT("AUDIO_ERR_DEC_FAIL\r\n");
        return AUDIO_ERR_DEC_FAIL;
	}

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len){
        //DBG_PRINT("AUDIO_ERR_DEC_FINISH\r\n");
		return AUDIO_ERR_DEC_FINISH;
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = a1800dec_run((CHAR*)audio_bg_ctrl.work_mem, audio_bg_ctrl.wi, pcm_bg_out[wb_idx]);

	audio_bg_ctrl.ri = a1800dec_read_index((CHAR*)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length >= audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	if (pcm_point <= 0) {
		if (pcm_point < 0) {
			if (--audio_bg_ctrl.try_cnt == 0) {
				return AUDIO_ERR_DEC_FAIL;
			}
		}
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
            OSQPostFront(audio_bg_wq, (void *)wb_idx);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            xQueueSendToFront(audio_bg_wq, &wb_idx, portMAX_DELAY);
        #endif
		audio_bg_send_next_frame_q();
		//DBG_PRINT("xQueueSendToFront\r\n");
		return 0;
	}

	pcm_bg_len[wb_idx] = pcm_point;
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPost(aud_bg_send_q, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(aud_bg_send_q, &wb_idx, portMAX_DELAY);
    #endif

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	//DBG_PRINT("wb_idx %d\r\n",wb_idx);
	return 0;
}
#endif // #if APP_A1800_DECODE_BG_EN == 1

//===============================================================================================================
//   MIDI Playback
//===============================================================================================================
#if APP_G_MIDI_DECODE_EN == 1
INT32S  audio_midi_play_init(void)
{
	INT32S MIDI_Index;
	INT32S TotalMidi;
	INT8U mode;

	SPU_MIDI_Set_MIDI_Volume(127);//0~127,set MIDI software volume
	SPU_Set_MainVolume(BG_Volume);//set spu global volume
	SPU_SingleChVolumeSelect(0x03);

#if SUPPORT_MIDI_LOAD_FROM_NVRAM == 1
	if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_GPRS) {
		SPU_resource_read_register((void*)resource_audio_load);
	}
#endif
	MIDI_Index = global_midi_index;
	TotalMidi = mode = 0;
	SPU_Set_midi_ring_buffer_addr((INT32U)audio_bg_ctrl.ring_buf);

	if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)	// added by Bruce, 2008/10/31
	{
		#if SUPPORT_MIDI_READ_FROM_SDRAM == 1
			TotalMidi = SPU_load_tonecolor_from_SDRAM(audio_bg_ctrl.file_handle, MIDI_Index);
		#endif
	}
	else
	{
	#if _SUPPORT_MIDI_IN_ITEM_SELECTION == 1
	if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_FS) {
		mode = 0;
	}
	else if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_GPRS) {
		mode = 1;
	}
	else if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_APP_RS) {
		mode = 2;
	}
	else if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_APP_PACKED_RS) {
		mode = 3;
	}

	TotalMidi = SPU_load_tonecolor(audio_bg_ctrl.file_handle, 0,mode);
	#else
	TotalMidi = SPU_load_tonecolor(audio_bg_ctrl.file_handle, MIDI_Index,3);
	#endif
	}

	if (TotalMidi < 0) {
		DBG_PRINT("load midi tonecolor fail\r\n");
		return AUDIO_ERR_DEC_FAIL;
	}

	#if _SUPPORT_MIDI_IN_ITEM_SELECTION == 0//Lichuanyue modified for nv midi random play.
	if(++global_midi_index >= TotalMidi)
	{
		global_midi_index = 0;
	}
	#endif

	if(TotalMidi > 0) {
		SPU_MIDI_Play(0);
	}
	else
		DBG_PRINT("Play Midi Error...\r\n");

	return AUDIO_ERR_NONE;

}

INT32S  audio_midi_process(void)
{
	INT32S ret;

	if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)		// added by Bruce, 2008/10/31
	{
		#if SUPPORT_MIDI_READ_FROM_SDRAM == 1
			ret = SPU_check_fill_midi_ring_buffer_from_SDRAM();
		#endif
	}
	else
	{
	    ret = SPU_check_fill_midi_ring_buffer();
	}
	if (ret < 0) {
		return AUDIO_ERR_DEC_FAIL;
	}

        #if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSTimeDly(5);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        vTaskDelay(5);
        #endif

	if( (SPU_MIDI_Get_Status() & 0x00000001) == 0 )//stop
	{
		return AUDIO_ERR_DEC_FINISH;
	}

	if (audio_bg_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)		// added by Bruce, 2008/10/31
	{
		#if SUPPORT_MIDI_READ_FROM_SDRAM == 1
			ret = SPU_check_fill_midi_ring_buffer_from_SDRAM();
		#endif
	}
	else
	{
	    ret = SPU_check_fill_midi_ring_buffer();
	}
	if (ret < 0) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_bg_send_next_frame_q();
	return AUDIO_ERR_NONE;
}
#endif

#if APP_MP3_DECODE_BG_EN == 1 || APP_WMA_DECODE_BG_EN == 1 || APP_WAV_CODEC_BG_EN == 1
void audio_dac_pcm_data_adj(INT16S *pcm_buf, INT32U len)
{
	INT32U i;
	INT16U *ptr;
	INT16U data;
	INT16U *temp_ptr;

	ptr = (INT16U *)pcm_buf;

	if (channel == 2) {
		temp_ptr = (INT16U *)buf_left;
		*ptr ^= 0x8000;
		if (*ptr == 0xffff) {
			*ptr = *ptr -1;
		}
		ptr++;
		for(i=1;i<len;i++) {
			data = pcm_buf[i] ^ 0x8000;
			if (data == 0xffff) {
				data--;
			}
			if (i & 1) {
				*temp_ptr++ = data;
			}
			else {
				*ptr++ = data;
			}
		 }
	}
	else {
		temp_ptr = (INT16U *)(pcm_buf+len+1);
		for (i=0;i<len;i++) {
			data = pcm_buf[i] ^ 0x8000;
			if (data == 0xffff) {
				data--;
			}
			*ptr++ = data;
			*temp_ptr++ = data;
		}
		*ptr = 0xFFFF;
		*temp_ptr = 0xFFFF;
		return;
	}

	 *ptr++ = 0xFFFF;

	 gp_memcpy((INT8S*)ptr,(INT8S*)buf_left,len); /* in byte */
	 *(ptr+(len>>1)) = 0xFFFF;
}
#endif

//===============================================================================================================
//   A1600 Playback
//===============================================================================================================
#if APP_A1600_DECODE_BG_EN == 1
INT32S audio_a16_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.f_last = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;//080724
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	//a1600 init
	ret = A16_dec_get_mem_block_size();
	if(ret != A16_DEC_MEMORY_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = A16_dec_init((char *)audio_bg_ctrl.work_mem, (unsigned char *)audio_bg_ctrl.ring_buf);//080723
	audio_bg_ctrl.wi = A16_dec_get_ri((void *)audio_bg_ctrl.work_mem);//after initial the value is 0
	audio_bg_ctrl.ri = A16_dec_get_ri((void *)audio_bg_ctrl.work_mem);
	in_length = audio_bg_ctrl.ri;
	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_bg_ctrl.ri;
		ret = A16_dec_parsing((CHAR*)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);

		audio_bg_ctrl.ri = A16_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
		audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
		if(audio_bg_ctrl.ri < in_length) {
			audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
		}

		switch(ret)
		{
			case A16_OK:
				break;

			case A16_E_NO_MORE_SRCDATA:		//not found sync word
			case A16_E_READ_IN_BUFFER:	//reserved sample frequency value
			case A16_CODE_FILE_FORMAT_ERR:		//forbidden bitrate value
			case A16_E_FILE_END:
			case A16_E_MODE_ERR:
				//feed in DecodeInBuffer;
				audio_bg_ctrl.ri = A16_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
				t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

				/* check reading data */
				if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_READ_FAIL;
				}

				if (--count == 0) {
					return AUDIO_ERR_FAILED;
				}
				continue;

			default:
				return AUDIO_ERR_FAILED;
		}

		if(ret == A16_OK) {
			break;
		}
	}

	sample_rate = A16_dec_get_samplerate((CHAR*)audio_bg_ctrl.work_mem);
	channel = A16_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem);

	DBG_PRINT("bps: %d\r\n",A16_dec_get_bitspersample((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",A16_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", A16_dec_get_samplerate((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("block size: %d\r\n",A16_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		OSQPost(audio_bg_wq, (void *) count);
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_a16_process(void)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT32U  wb_idx;
	INT32S  t_wi;
	INT8U   wait;

	audio_bg_ctrl.ri = A16_dec_get_ri((void *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);

	wait = 0;
	if (t_wi == AUDIO_READ_WAIT) {
		wait = 1; /* pend until read finish */
	}

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}


	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		return AUDIO_ERR_DEC_FINISH;
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = A16_dec_run((CHAR*)audio_bg_ctrl.work_mem, pcm_bg_out[wb_idx], audio_bg_ctrl.wi);

	audio_bg_ctrl.ri = A16_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	if (pcm_point <= 0) {
		if (pcm_point < 0) {
			if (--audio_bg_ctrl.try_cnt == 0) {
				return AUDIO_ERR_DEC_FAIL;
			}
		}

		OSQPostFront(audio_bg_wq, (void *)wb_idx);
		audio_bg_send_next_frame_q();
		return 0;
	}

	pcm_bg_len[wb_idx] = pcm_point;
	OSQPost(aud_bg_send_q, (void *) wb_idx);
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	return 0;
}
#endif // #if APP_A1600_DECODE_BG_EN == 1

//===============================================================================================================
//   A6400 Playback
//===============================================================================================================
#if APP_A6400_DECODE_BG_EN == 1
INT32S audio_a64_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;


	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.f_last = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	//a6400 init
	ret = a6400_dec_get_mem_block_size();
	if(ret != A6400_DEC_MEMORY_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = a6400_dec_init((char*)audio_bg_ctrl.work_mem, (unsigned char*)audio_bg_ctrl.ring_buf, (char*)(audio_bg_ctrl.work_mem + A6400_DEC_MEMORY_SIZE));
	a6400_dec_set_bs_buf_size((char*)audio_bg_ctrl.work_mem, audio_bg_ctrl.ring_size);

	audio_bg_ctrl.wi = a6400_dec_get_ri((void *)audio_bg_ctrl.work_mem);//after initial the value is 0
	audio_bg_ctrl.ri = a6400_dec_get_ri((void *)audio_bg_ctrl.work_mem);
	in_length = audio_bg_ctrl.ri;

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_bg_ctrl.ri;
		ret = a6400_dec_parsing((CHAR*)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);

		audio_bg_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
		audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
		if(audio_bg_ctrl.ri < in_length) {
			audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
		}

		switch(ret)
		{
			case A6400_DEC_ERR_NONE:
				break;

			case A6400_DEC_ERR_LOSTSYNC:		//not found sync word
			case A6400_DEC_ERR_BADSAMPLERATE:	//reserved sample frequency value
			case A6400_DEC_ERR_BADBITRATE:		//forbidden bitrate value
			case A6400_DEC_ERR_BADLAYER:
			case A6400_DEC_ERR_BADMPEGID:
				//feed in DecodeInBuffer;
				audio_bg_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
				t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

				/* check reading data */
				if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_READ_FAIL;
				}

				if (--count == 0) {
					return AUDIO_ERR_FAILED;
				}
				continue;

			default:
				return AUDIO_ERR_FAILED;
		}

		if(ret == A6400_DEC_ERR_NONE) {
			break;
		}
	}

	sample_rate = a6400_dec_get_samplerate((CHAR*)audio_bg_ctrl.work_mem);
	channel = a6400_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem);

	DBG_PRINT("bps: %d\r\n",a6400_dec_get_bitrate((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",a6400_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n",a6400_dec_get_samplerate((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("block size: %d\r\n",a6400_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		OSQPost(audio_bg_wq, (void *) count);
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_a64_process(void)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT32U  wb_idx;
	INT32S  t_wi;
	INT8U   wait;

	audio_bg_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);
	wait = 0;
	if (t_wi == AUDIO_READ_WAIT) {
		wait = 1; /* pend until read finish */
	}

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		if(audio_bg_ctrl.f_last){
			return AUDIO_ERR_DEC_FINISH;
		} else {
			audio_bg_ctrl.f_last = 1;
		}
	}


	in_length = audio_bg_ctrl.ri;
	pcm_point = a6400_dec_run((CHAR*)audio_bg_ctrl.work_mem, pcm_bg_out[wb_idx], audio_bg_ctrl.wi,audio_bg_ctrl.f_last);

	audio_bg_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	if (pcm_point <= 0) {
		if (pcm_point < 0) {
			if (--audio_bg_ctrl.try_cnt == 0) {
				return AUDIO_ERR_DEC_FINISH;
			}
		}

		OSQPostFront(audio_bg_wq, (void *)wb_idx);
		audio_bg_send_next_frame_q();
		return 0;
	}

	pcm_bg_len[wb_idx] = pcm_point * a6400_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem);


	OSQPost(aud_bg_send_q, (void *) wb_idx);
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	return 0;
}
#endif // #if APP_A6400_DECODE_BG_EN == 1

//===============================================================================================================
//   S880 Playback
//===============================================================================================================
#if APP_S880_DECODE_BG_EN == 1
INT32S audio_s880_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.f_last = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;//080724
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	//s880 init
	ret = S880_dec_get_mem_block_size();
	if(ret != S880_DEC_MEMORY_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = S880_dec_init((char *)audio_bg_ctrl.work_mem, (unsigned char *)audio_bg_ctrl.ring_buf);//080723
	audio_bg_ctrl.wi = S880_dec_get_ri((void *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.ri = S880_dec_get_ri((void *)audio_bg_ctrl.work_mem);
	in_length = audio_bg_ctrl.ri;
	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_bg_ctrl.ri;
		ret = S880_dec_parsing((CHAR*)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);

		audio_bg_ctrl.ri = S880_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
		audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
		if(audio_bg_ctrl.ri < in_length) {
			audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
		}

		switch(ret)
		{
			case S880_OK:
				break;

			case S880_E_NO_MORE_SRCDATA:		//not found sync word
			case S880_E_READ_IN_BUFFER:	//reserved sample frequency value
			case S880_CODE_FILE_FORMAT_ERR:		//forbidden bitrate value
			case S880_E_FILE_END:
		//	case S880_E_MODE_ERR:
				//feed in DecodeInBuffer;
				audio_bg_ctrl.ri = S880_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
				t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

				/* check reading data */
				if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_READ_FAIL;
				}

				if (--count == 0) {
					return AUDIO_ERR_FAILED;
				}
				continue;

			default:
				return AUDIO_ERR_FAILED;
		}

		if(ret == S880_OK) {
			break;
		}
	}

	sample_rate = S880_dec_get_samplerate((CHAR*)audio_bg_ctrl.work_mem);
	channel = S880_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem);

	DBG_PRINT("bps: %d\r\n",S880_dec_get_bitspersample((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",S880_dec_get_channel((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", S880_dec_get_samplerate((CHAR*)audio_bg_ctrl.work_mem));
	DBG_PRINT("block size: %d\r\n",S880_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		OSQPost(audio_bg_wq, (void *) count);
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_s880_process(void)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT32U  wb_idx;
	INT32S  t_wi;
	INT8U   wait;

	audio_bg_ctrl.ri = S880_dec_get_ri((void *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);
	wait = 0;

	if (t_wi == AUDIO_READ_WAIT) {
		wait = 1; /* pend until read finish */
	}

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}


	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		return AUDIO_ERR_DEC_FINISH;
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = S880_dec_run((CHAR*)audio_bg_ctrl.work_mem, pcm_bg_out[wb_idx], audio_bg_ctrl.wi);

	audio_bg_ctrl.ri = S880_dec_get_ri((CHAR*)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	if (pcm_point <= 0) {
		if (pcm_point < 0) {
			if (--audio_bg_ctrl.try_cnt == 0) {
				return AUDIO_ERR_DEC_FAIL;
			}
		}

		OSQPostFront(audio_bg_wq, (void *)wb_idx);
		audio_bg_send_next_frame_q();
		return 0;
	}

	pcm_bg_len[wb_idx] = pcm_point;
	OSQPost(aud_bg_send_q, (void *) wb_idx);
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	return 0;
}
#endif // #if APP_S880_DECODE_BG_EN == 1

//===============================================================================================================
//  AAC Playback
//===============================================================================================================
#if APP_AAC_DECODE_BG_EN == 1
INT32S audio_aac_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;
	INT32S  downMatrix = 0;	// 1: input 5.1 channels and output 2 channels

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.f_last = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	// aac init
	ret = aac_dec_init((void *)audio_bg_ctrl.work_mem, downMatrix, (INT8U *)audio_bg_ctrl.ring_buf);
	aac_dec_SetRingBufferSize((void *)audio_bg_ctrl.work_mem,audio_bg_ctrl.ring_size);
	audio_bg_ctrl.wi = 0;
	audio_bg_ctrl.ri = aac_dec_get_read_index((void *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_bg_ctrl.ri;
		ret = aac_dec_parsing((void *)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);

		audio_bg_ctrl.ri = aac_dec_get_read_index((void *)audio_bg_ctrl.work_mem);
		audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
		if(audio_bg_ctrl.ri < in_length) {
			audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
		}

		if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
			return AUDIO_ERR_FAILED;
		}

		switch(ret)
		{
			case AAC_OK:
				break;

			case  UNABLE_TO_FIND_ADTS_SYNCWORD:
				//feed in DecodeInBuffer;
				audio_bg_ctrl.ri = aac_dec_get_read_index((void *)audio_bg_ctrl.work_mem);
				t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

				/* check reading data */
				if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_READ_FAIL;
				}

				if (--count == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
				continue;

			case UNSUPPORTED_FILE_FORMAT_MP4:
			case NOT_MONO_OR_STEREO:
			case NOT_LC_OBJECT_TYPE:
			default:
				return AUDIO_ERR_DEC_FAIL;
		}

		if(ret == AAC_OK) {
			break;
		}
	}

	sample_rate = aac_dec_get_samplerate((void *)audio_bg_ctrl.work_mem);
	channel = aac_dec_get_channel((void *)audio_bg_ctrl.work_mem);

	DBG_PRINT("bps: %d\r\n", aac_dec_get_bitspersample((void *)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", aac_dec_get_channel((void *)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", aac_dec_get_samplerate((void *)audio_bg_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		OSQPost(audio_bg_wq, (void *) count);
	}

	return AUDIO_ERR_NONE;
}

INT32S audio_aac_process(void)
{
	INT32S	pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT32U  wb_idx;
	INT32S  t_wi;
	INT8U   wait;

	audio_bg_ctrl.ri = aac_dec_get_read_index((void *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);
	if (t_wi == AUDIO_READ_WAIT) {
		wait = 1; /* pend until read finish */
	}

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		return AUDIO_ERR_DEC_FINISH;
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = aac_dec_run((void *)audio_bg_ctrl.work_mem, audio_bg_ctrl.wi, pcm_bg_out[wb_idx]);

	audio_bg_ctrl.ri = aac_dec_get_read_index((void *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_ctrl.ri) {
		audio_ctrl.file_cnt += audio_ctrl.ring_size;
	}

	if (pcm_point <= 0) {
		if (pcm_point < 0) {
			if (--audio_bg_ctrl.try_cnt == 0) {
				return AUDIO_ERR_DEC_FAIL;
			}
		}

		OSQPostFront(audio_bg_wq, (void *)wb_idx);
		audio_bg_send_next_frame_q();
		return 0;
	}

	pcm_bg_len[wb_idx] = pcm_point * aac_dec_get_channel((void *)audio_bg_ctrl.work_mem);
	OSQPost(aud_bg_send_q, (void *) wb_idx);
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	return 0;
}
#endif // #if APP_AAC_DECODE_FG_EN == 1

//===============================================================================================================
//  OGG Playback
//===============================================================================================================
#if APP_OGG_DECODE_BG_EN == 1
INT32S audio_ogg_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_bg_ctrl.file_cnt = 0;
	audio_bg_ctrl.f_last = 0;
	audio_bg_ctrl.try_cnt = 20;
	audio_bg_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_bg_ctrl.ring_buf, 0, audio_bg_ctrl.ring_size);

	// aac init
	ret = oggvorbis_dec_init((void *)audio_bg_ctrl.work_mem, (const char*)audio_bg_ctrl.ring_buf, audio_bg_ctrl.ring_size, 0);
	oggvorbis_dec_set_ring_buffer((void *)audio_bg_ctrl.work_mem, (const char*)audio_bg_ctrl.ring_buf, audio_bg_ctrl.ring_size, 0);
	audio_bg_ctrl.wi = 0;
	audio_bg_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_bg_ctrl.work_mem);
	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_bg_ctrl.ri;
		ret = oggvorbis_dec_parsing((void *)audio_bg_ctrl.work_mem , audio_bg_ctrl.wi);

		audio_bg_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_bg_ctrl.work_mem);
		audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
		if(audio_bg_ctrl.ri < in_length) {
			audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
		}

		if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
			return AUDIO_ERR_FAILED;
		}

		switch(ret)
		{
			case OGGVORBIS_PARSING_OK:
				break;

			case OGGVORBIS_MORE_DATA:
				//feed in DecodeInBuffer;
				audio_bg_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_bg_ctrl.work_mem);
				t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

				/* check reading data */
				if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_READ_FAIL;
				}

				if (--count == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
				continue;

			default:
				return AUDIO_ERR_DEC_FAIL;
		}

		if(ret == OGGVORBIS_PARSING_OK) {
			break;
		}
	}

	sample_rate = oggvorbis_dec_get_samplerate((void *)audio_bg_ctrl.work_mem);
	channel = oggvorbis_dec_get_channel((void *)audio_bg_ctrl.work_mem);

	DBG_PRINT("bps: %d\r\n", oggvorbis_dec_get_bitrate((void *)audio_bg_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", oggvorbis_dec_get_channel((void *)audio_bg_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", oggvorbis_dec_get_samplerate((void *)audio_bg_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		OSQPost(audio_bg_wq, (void *) count);
	}

	return AUDIO_ERR_NONE;
}

INT32S audio_ogg_process(void)
{
	INT32S	pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT32U  wb_idx;
	INT32S  t_wi;
	INT8U   wait = 0;

	audio_bg_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_bg_ctrl.work_mem);

	t_wi = audio_bg_write_with_file_srv(audio_bg_ctrl.ring_buf, audio_bg_ctrl.wi, audio_bg_ctrl.ri);

	wb_idx = (INT32U) OSQPend(audio_bg_wq, 0, &err);
	if (t_wi == AUDIO_READ_WAIT) {
		wait = 1; /* pend until read finish */
	}

	/* check reading data */
	if (audio_bg_check_wi(t_wi, &audio_bg_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	if(audio_bg_ctrl.file_cnt >= audio_bg_ctrl.file_len) {
		return AUDIO_ERR_DEC_FINISH;
	}

	in_length = audio_bg_ctrl.ri;
	pcm_point = oggvorbis_dec_run((void *)audio_bg_ctrl.work_mem, pcm_bg_out[wb_idx], audio_bg_ctrl.wi);

	audio_bg_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_bg_ctrl.work_mem);
	audio_bg_ctrl.file_cnt += (audio_bg_ctrl.ri - in_length);
	if(in_length > audio_bg_ctrl.ri) {
		audio_bg_ctrl.file_cnt += audio_bg_ctrl.ring_size;
	}

	if (pcm_point <= 0) {
		if (pcm_point < 0) {
			if (--audio_bg_ctrl.try_cnt == 0) {
				return AUDIO_ERR_DEC_FAIL;
			}
		}

		OSQPostFront(audio_bg_wq, (void *)wb_idx);
		audio_bg_send_next_frame_q();
		return 0;
	}

	pcm_bg_len[wb_idx] = pcm_point * oggvorbis_dec_get_channel((void *)audio_bg_ctrl.work_mem);
	OSQPost(aud_bg_send_q, (void *) wb_idx);
	audio_dac_pcm_data_adj(pcm_bg_out[wb_idx], pcm_bg_len[wb_idx]);

	audio_send_to_spu();
	audio_bg_send_next_frame_q();
	return 0;
}
#endif // #if APP_OGG_DECODE_BG_EN == 1

#if APP_G_MIDI_DECODE_EN == 1
#if SUPPORT_MIDI_READ_FROM_SDRAM == 1
INT32S SPU_read_a_sector_from_SDRAM(INT16S fd, INT32U data_offset_addr, INT32U buffer_addr)
{
	INT32S ret;

	if(audio_move_data != NULL)
	{
		g_audio_data_offset = data_offset_addr;
		ret = audio_bg_move_data(buffer_addr, 512);
	}

	return ret;
}

INT32S	SPU_load_tonecolor_from_SDRAM(INT16S fd_idi, INT32S MIDI_Index)
{
	INT32U inst_start_addr,lib_start_addr,midi_start_addr;
	INT32U inst_tab_start_addr;
	INT32U offset_addr,offset_addr1;
	INT32U total_midi;
	INT32U temp_buffer_addr, inst_temp_buffer_addr, drum_temp_buffer_addr;
	INT32U *U32_ptr;
	INT16U *U16_ptr,*U16_ptr1;
	INT32S ret;
	INT32U temp,temp1;
	INT32S dwSize;
	INT32S i,j;
	INT32U temp_addr,temp_size;
	//-----------------------------------------------------------------------//get every single file 's offset
	static_fd_idi = fd_idi;//080912

	temp_buffer_addr = (INT32U)gp_malloc(512);//512 byte per sector;
	offset_addr = 0;
	ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
	if(ret < 0) {
		return ret;
	}
	U32_ptr = (INT32U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	inst_start_addr = *(U32_ptr++);
	lib_start_addr  = *(U32_ptr++);
	midi_start_addr = *(U32_ptr++);
	//-----------------------------------------------------------------------
	total_midi		= *(U32_ptr++);
	if(MIDI_Index >= total_midi)
	{
		return -1;
	}
	offset_addr += 16;
	//U32_ptr += MIDI_Index;
	offset_addr = offset_addr + MIDI_Index*4;
	//if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )
	//{
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		//offset_addr = offset_addr1;
		U32_ptr = (INT32U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	//}
	inst_tab_start_addr = inst_start_addr + (*U32_ptr) ;
	offset_addr = inst_tab_start_addr;
	ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
	if(ret < 0) {
		return ret;
	}

	U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	// U16_ptr ָinst_tabĿʼ 
	dwSize = (INT32S)*(U16_ptr);
	for(i=0; i<dwSize; i++)
	{
		offset_addr1 = offset_addr + 2;
		if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
		{
			ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
			if(ret < 0) {
				return ret;
			}

			offset_addr = offset_addr1;
			U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		}
		else
		{
			U16_ptr++;
			offset_addr += 2;
		}
		T_InstrumentStartSection[i] = (INT32U)*(U16_ptr);
	}
	//ѽ T_InstrumentStartSection
	offset_addr += 2;
	ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
	if(ret < 0) {
		return ret;
	}

	U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	// U16_ptr ָinst_tab_pitchTableĿʼ 
	dwSize = (INT32S)*(U16_ptr);
	for(i=0; i<dwSize; i++)
	{
		offset_addr1 = offset_addr + 2;
		if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
		{
			ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
			if(ret < 0) {
				return ret;
			}

			offset_addr = offset_addr1;
			U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		}
		else
		{
			U16_ptr++;
			offset_addr += 2;
		}
		T_InstrumentPitchTable[i] = (INT32U)*(U16_ptr);
	}
	//ѽ T_InstrumentPitchTable -----------------------------------------------------OK
	offset_addr += 2;
	ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
	if(ret < 0) {
		return ret;
	}

	U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	temp = (INT32U)*(U16_ptr);//low word of size
	offset_addr1 = offset_addr + 2;
	if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
	{
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		offset_addr = offset_addr1;
		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	}
	else
	{
		U16_ptr++;
		offset_addr += 2;
	}
	temp1 = (INT32U)*(U16_ptr);//high word of size
	dwSize = (INT32S)((temp1<<16) | temp);//size of instrument sectors
	total_inst = dwSize;
	inst_temp_buffer_addr = (INT32U)gp_malloc(dwSize*8);
	U16_ptr1 = (INT16U*)inst_temp_buffer_addr;
	for(i=0; i<dwSize*4; i++)
	{
		offset_addr1 = offset_addr + 2;
		if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
		{
			ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
			if(ret < 0) {
				return ret;
			}
			offset_addr = offset_addr1;
			U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		}
		else
		{
			U16_ptr++;
			offset_addr += 2;
		}
		*U16_ptr1++ = (INT16U)*(U16_ptr);//low word of addr
	}
	//ѽtabеinstrument sectionĵַͳȵһȫinst_temp_buffer_addrbufferУ 
	offset_addr += 2;
	ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
	if(ret < 0) {
		return ret;
	}

	U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	temp = (INT32U)*(U16_ptr);//low word of size
	offset_addr1 = offset_addr + 2;
	if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
	{
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		offset_addr = offset_addr1;
		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	}
	else
	{
		U16_ptr++;
		offset_addr += 2;
	}
	temp1 = (INT32U)*(U16_ptr);//high word of size
	dwSize = (INT32S)((temp1<<16) | temp);//size of drum sectors
	total_drum = dwSize;
	drum_temp_buffer_addr = (INT32U)gp_malloc(dwSize*8);
	U16_ptr1 = (INT16U*)drum_temp_buffer_addr;
	for(i=0; i<dwSize*4; i++)
	{
		offset_addr1 = offset_addr + 2;
		if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
		{
			ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
			if(ret < 0) {
				return ret;
			}

			offset_addr = offset_addr1;
			U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		}
		else
		{
			U16_ptr++;
			offset_addr += 2;
		}
		*U16_ptr1++ = (INT16U)*(U16_ptr);//low word of addr
	}
	//ѽtabеdrum sectionĵַͳȵһȫdrum_temp_buffer_addrbuffer---------OK
	U32_ptr = (INT32U*)inst_temp_buffer_addr;
	for(i=0; i<total_inst; i++)
	{
		temp_addr = (INT32U)*U32_ptr++;
		temp_size = (INT32U)*U32_ptr++;
		temp = (INT32U)gp_malloc(temp_size);
		offset_addr = lib_start_addr + temp_addr;
		j = temp_size;//jʾҪȡֽ 
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		U16_ptr1 = (INT16U*)temp;//
		offset_addr1 = (offset_addr & 0xfffffe00) + 0x00000200;//һsectorʼַ 
		while((offset_addr < offset_addr1) && (j>0))
		{
			*U16_ptr1++ = *U16_ptr++;
			offset_addr += 2;
			j -= 2;
		}
		offset_addr = offset_addr1;//þҲʡ 
		for(;j>=512;)//һζһsector
		{
			ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), (INT32U)U16_ptr1);
			if(ret < 0) {
				return ret;
			}

			j -= 512;
			offset_addr += 512;//512 byte
			U16_ptr1 += 256;//256 word
			if(offset_addr & 0x000001ff) {
				return -1;
			}
		}
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		while(j>0)
		{
			*U16_ptr1++ = *U16_ptr++;
			offset_addr += 2;
			j -= 2;
		}
		T_InstrumentSectionAddr[i] = (INT32U*)temp;
	}
	gp_free((void *)inst_temp_buffer_addr);
	//ѽinstrument sectionsȫRAMУram׵ַдT_InstrumentSectionAddr[]
	U32_ptr = (INT32U*)drum_temp_buffer_addr;
	for(i=0; i<total_drum; i++)
	{
		temp_addr = (INT32U)*U32_ptr++;
		temp_size = (INT32U)*U32_ptr++;
		temp = (INT32U)gp_malloc(temp_size);
		offset_addr = lib_start_addr + temp_addr;
		j = temp_size;//jʾҪȡֽ 
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		U16_ptr1 = (INT16U*)temp;//
		offset_addr1 = (offset_addr & 0xfffffe00) + 0x00000200;//һsectorʼַ 
		while((offset_addr < offset_addr1) && (j>0))
		{
			*U16_ptr1++ = *U16_ptr++;
			offset_addr += 2;
			j -= 2;
		}
		offset_addr = offset_addr1;//þҲʡ 
		for(;j>=512;)//һζһsector
		{
			ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), (INT32U)U16_ptr1);
			if(ret < 0) {
				return ret;
			}

			j -= 512;
			offset_addr += 512;//512 byte
			U16_ptr1 += 256;//256 word
			if(offset_addr & 0x000001ff) {
				return -1;
			}
		}
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
		while(j>0)
		{
			*U16_ptr1++ = *U16_ptr++;
			offset_addr += 2;
			j -= 2;
		}
		T_DrumAddr[i] = (INT32U*)temp;
	}
	gp_free((void *)drum_temp_buffer_addr);
	//ѽdrum sectionsȫRAMУram׵ַдT_DrumAddr[]Уͷ 
	offset_addr = midi_start_addr + 4 + MIDI_Index*8;
	ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
	if(ret < 0) {
		return ret;
	}

	U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	temp = (INT32U)*(U16_ptr);//low word of addr
	offset_addr1 = offset_addr + 2;
	if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
	{
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		offset_addr = offset_addr1;
		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	}
	else
	{
		U16_ptr++;
		offset_addr += 2;
	}
	temp1 = (INT32U)*(U16_ptr);//high word of addr
	temp_addr = (INT32U)((temp1<<16) | temp);//midi offset
	//ѵõmidiƫƵַ 
	offset_addr1 = offset_addr + 2;
	if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
	{
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		offset_addr = offset_addr1;
		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	}
	else
	{
		U16_ptr++;
		offset_addr += 2;
	}
	temp = (INT32U)*(U16_ptr);//low word of size
	offset_addr1 = offset_addr + 2;
	if( (offset_addr&0xfffffe00) != (offset_addr1&0xfffffe00) )//sector
	{
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr1 & 0xfffffe00), temp_buffer_addr);
		if(ret < 0) {
			return ret;
		}

		offset_addr = offset_addr1;
		U16_ptr = (INT16U*)(temp_buffer_addr + ((INT32U)offset_addr & 0x000001ff) );
	}
	else
	{
		U16_ptr++;
		offset_addr += 2;
	}
	temp1 = (INT32U)*(U16_ptr);//high word of size
	midi_length = (temp1<<16) | temp;
	//õmidiĳȣֽ 
	offset_addr = midi_start_addr + temp_addr;
	ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), temp_buffer_addr);
	if(ret < 0) {
		return ret;
	}

	midi_ring_buffer_ri = offset_addr & 0x000001ff;//ʾmidi_ring_bufferĸλÿʼ 
	offset_addr1 = (offset_addr & 0xfffffe00) + 512;//ָһsectorĿʼ 
	U16_ptr = (INT16U*)(temp_buffer_addr + (offset_addr & 0x000001ff));
	U16_ptr1 = (INT16U*)(midi_ring_buffer_addr + (offset_addr & 0x000001ff));
	while((offset_addr < offset_addr1) && (midi_length))//midi_ring_bufferеһsectorԶķʽǰЩհ 
	{
		*U16_ptr1++ = *U16_ptr++;
		offset_addr += 2;
		midi_length -= 2;//ʾɶʣmidi
	}
	offset_addr = offset_addr1;//þʡ 
	midi_ring_buffer_wi = 512;
	while(midi_length)
	{
		ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (offset_addr & 0xfffffe00), (INT32U)U16_ptr1);
		if(ret < 0) {
			return ret;
		}

		offset_addr += 512;//512byte
		U16_ptr1 += 256;//256word
		midi_ring_buffer_wi += 512;
		if(midi_length >= 512)
			midi_length -= 512;//ʾɶʣmidi
		else
			midi_length = 0;
		if(midi_ring_buffer_wi >= MIDI_RING_BUFFER_SIZE)
		{
			break;
		}
	}
	currt_midi_data_offset = offset_addr;
	midi_ring_buffer_wi = 0;//ʾѽmidi_ring_bufferBдΪMIDI_RING_BUFFER_SIZE/2ʾѽA 

	gp_free((void *)temp_buffer_addr);
	return total_midi;
}


INT32S SPU_check_fill_midi_ring_buffer_from_SDRAM(void)
{
	INT32S ret;

	midi_ring_buffer_ri = SPU_get_midi_ring_buffer_ri();
	switch(midi_ring_buffer_wi)
	{
		case 0:
			if(midi_ring_buffer_ri >= (MIDI_RING_BUFFER_SIZE/2) )
			{
				while(midi_length)
				{
					ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (currt_midi_data_offset & 0xfffffe00), (midi_ring_buffer_addr + midi_ring_buffer_wi));
					if(ret < 0) {
						return ret;
					}

					currt_midi_data_offset += 512;
					midi_ring_buffer_wi += 512;
					if(midi_length >= 512)
						midi_length -= 512;//ʾɶʣmidi
					else
						midi_length = 0;
					if(midi_ring_buffer_wi >= (MIDI_RING_BUFFER_SIZE/2) )
					{
						midi_ring_buffer_wi = MIDI_RING_BUFFER_SIZE/2;
						break;
					}
				}
			}
			break;

		case (MIDI_RING_BUFFER_SIZE/2):
			if(midi_ring_buffer_ri < (MIDI_RING_BUFFER_SIZE/2) )
			{
				while(midi_length)
				{
					ret = SPU_read_a_sector_from_SDRAM(static_fd_idi, (currt_midi_data_offset & 0xfffffe00), (midi_ring_buffer_addr + midi_ring_buffer_wi));
					if(ret < 0) {
						return ret;
					}

					currt_midi_data_offset += 512;
					midi_ring_buffer_wi += 512;
					if(midi_length >= 512)
						midi_length -= 512;//ʾɶʣmidi
					else
						midi_length = 0;
					if(midi_ring_buffer_wi >= MIDI_RING_BUFFER_SIZE)
					{
						midi_ring_buffer_wi = 0;
						break;
					}
				}
			}
			break;

		default:break;
	}
	return 0;
}
#endif	//#if SUPPORT_MIDI_READ_FROM_SDRAM == 1
#endif //APP_G_MIDI_DECODE_EN == 1
#endif
