#include <string.h>
#include "turnkey_audio_decoder_task.h"
#include "drv_l1_dac.h"

// Constant definitions used in this file only go here
#define AUDIO_QUEUE_MAX  64*4
#define AUDIO_FS_Q_SIZE  1
#define AUDIO_WRITE_Q_SIZE MAX_I2S_BUFFERS
#define AUDIO_PARA_MAX_LEN  sizeof(STAudioTaskPara)
#define AUDIO_AVI_RINGBUF_LEN	4000
#define RAMP_DOWN_STEP 4
#define RAMP_DOWN_STEP_HOLD 4
#define RAMP_DOWN_STEP_LOW_SR	4*16
#define USE_RAMP_DOWN_RAPID		0
#define SKIP_ID3_TAG            0

/* macro */
#define MAKEFOURCC(ch0, ch1, ch2, ch3)	\
	((INT32U)ch0		|	\
	((INT32U)ch1 << 8)	|	\
	((INT32U)ch2 << 16)	|	\
	((INT32U)ch3 << 24))

#define READSTRING(pData)	\
	(*(pData+0) << 0)	|	\
	(*(pData+1) << 8)	|	\
	(*(pData+2) << 16)	|	\
	(*(pData+3) << 24)

/* external variable */
#if (_OPERATING_SYSTEM == _OS_UCOS2)
extern OS_EVENT  *Mbox_WMA_Play_Seek_Flag;         //add WMA_Seek
extern OS_EVENT  *Mbox_WMA_Play_Seek_Offset;

#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
extern INT32U last_send_idx;
extern xQueueHandle aud_send_q;
extern xQueueHandle aud_i2s_send_q[4];
extern xQueueHandle hAudioDacTaskQ;
//extern xQueueHandle hAudioI2sTaskQ;     //I2S
extern SACM_CTRL G_SACM_Ctrl;
extern MSG_Q_ID fs_msg_q_id;
extern MSG_Q_ID Audio_FG_status_Q;
extern INT32S Snd_GetData(INT32U buf_adr,INT32U buf_size);
extern INT32U Snd_GetLen(void);
extern INT32S Snd_Lseek(INT32S offset,INT16S fromwhere);
extern xQueueHandle aud_i2s_profile_q;   // for i2s
#endif
//extern INT32U i2s_ch_id;            // for i2s
/* Task Q declare */
extern MSG_Q_ID AudioTaskQ;
extern STAudioTaskPara Aud_Para[4];    //4: MAX_I2X_TX_NUM
#if (_OPERATING_SYSTEM == _OS_UCOS2)
OS_EVENT	*audio_wq;
void		*write_q[AUDIO_WRITE_Q_SIZE];
OS_EVENT	*audio_fsq;
void		*fs_q[AUDIO_FS_Q_SIZE];

#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
//xQueueHandle audio_wq;
//xQueueHandle audio_fsq;

xQueueHandle audio_i2s_wq[4];
xQueueHandle audio_i2s_fsq[4];
#else
#error "Audio Decode Not Support In Non OS System!"
#endif

extern INT8U    audio_para[AUDIO_PARA_MAX_LEN];


INT16S		*pcm_i2s_out[4][MAX_I2S_BUFFERS] = {NULL};
INT32U		pcm_i2s_len[4][MAX_I2S_BUFFERS];

extern INT8S		avi_work_mem[WAV_DEC_MEMORY_SIZE];

AUDIO_CONTEXT   audio_i2s_context[4];
AUDIO_CONTEXT_P audio_i2s_context_p[4] = {&audio_i2s_context[0],&audio_i2s_context[1],&audio_i2s_context[2],&audio_i2s_context[3]};

AUDIO_CTRL      audio_i2s_ctrl[4];
//STAudioConfirm  aud_con;
AUDIO_I2S_PROFILE   ogg_i2s_profile[4];
AUDIO_I2S_PROFILE   *p_ogg_i2s_profile[4] = {&ogg_i2s_profile[0], &ogg_i2s_profile[1],&ogg_i2s_profile[2],&ogg_i2s_profile[3]};
#if (_OPERATING_SYSTEM == _OS_UCOS2)
volatile INT8U  *avi_state_mon;
#endif

static struct sfn_info aud_sfn_file;
static struct sfn_info aud_i2s_sfn_file[4];

INT8U   stopped_i2s[4];
extern INT8U	audio_rampdown_disable_flag;

void	(*i2s_decode_end)(INT32U audio_decoder_num, INT32U i2s_ch);
static  INT8U   channel;
static  INT16U   g_audio_sample_rate;
extern INT32S  audio_play_start_time;

static INT32S	(*pfnGetOutput)(void*, short*, int);
static void     *hSrc;

/* Proto types */
void audio_i2s_decode_task_init(void);
void audio_i2s_decode_task_entry(void *p_arg);

static void    audio_i2s_init(void);

#if APP_OGG_DECODE_FG_EN == 1
static INT32S  audio_i2s_ogg_play_init(INT32U i2s_ch);
static INT32S  audio_i2s_ogg_process(INT32U i2s_ch);
#endif
static INT32S  audio_i2s_send_to_dma(INT32U i2s_ch);

static void    audio_i2s_queue_clear(INT32U i2s_ch);
static INT32S  audio_i2s_stop_unfinished(INT32U i2s_ch);
static INT32S  audio_i2s_stop_finished(INT32U i2s_ch);

static void    audio_i2s_send_next_frame_q(INT32U i2s_ch);
static void    audio_i2s_ramp_down(INT32U i2s_ch);
static void    audio_i2s_start(STAudioTaskPara *pAudioTaskPara);
static void    audio_i2s_pause(STAudioTaskPara *pAudioTaskPara);
static void    audio_i2s_resume(STAudioTaskPara *pAudioTaskPara);
static void    audio_i2s_stop(STAudioTaskPara *pAudioTaskPara);

static void    audio_i2s_decode_next_frame(STAudioTaskPara *pAudioTaskPara);
static void    audio_i2s_mute_set(STAudioTaskPara *pAudioTaskPara);
static void    audio_i2s_volume_set(STAudioTaskPara *pAudioTaskPara);
extern void audio_send_result(INT32S res_type,INT32S result);
#if (defined AUDIO_FORMAT_JUDGE_AUTO) && (AUDIO_FORMAT_JUDGE_AUTO == 1)
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static INT32S audio_get_type(INT16S fd,INT8S* file_name, OS_EVENT *ack_fsq);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
static INT32S audio_get_type(INT16S fd,INT8S* file_name, xQueueHandle ack_fsq);
#endif
#else
static INT32S  audio_get_type(INT8S* file_name);
#endif

static INT32S  audio_i2s_q_check(INT32U i2s_ch);
#if USE_RAMP_DOWN_RAPID == 1
static void audio_i2s_ramp_down_rapid(void);
#endif

//extern INT32S audio_mem_alloc(INT32U audio_format,AUDIO_CTRL *aud_ctrl, INT16S *pcm[]);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
extern INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, OS_EVENT *ack_fsq);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
extern INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, xQueueHandle ack_fsq);
#endif
void audio_i2s_decode_task_init(void)
{
    /* Create MsgQueue/MsgBox for TASK */
    INT8U i;

    AudioTaskQ = msgQCreate(AUDIO_QUEUE_MAX, AUDIO_QUEUE_MAX, AUDIO_PARA_MAX_LEN);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
    audio_wq = OSQCreate(write_q, AUDIO_WRITE_Q_SIZE);
	audio_fsq = OSQCreate(fs_q, AUDIO_FS_Q_SIZE);

	avi_state_mon = &audio_context.state;
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    //audio_wq = xQueueCreate(AUDIO_WRITE_Q_SIZE, sizeof(INT32U));
    //audio_fsq = xQueueCreate(AUDIO_FS_Q_SIZE, sizeof(INT32U));

    for(i=0; i<4; i++){
        audio_i2s_wq[i] = xQueueCreate(AUDIO_WRITE_Q_SIZE, sizeof(INT32U));
        audio_i2s_fsq[i] = xQueueCreate(AUDIO_FS_Q_SIZE, sizeof(INT32U));
    }
#endif

}

void audio_i2s_decode_task_entry(void *p_arg)
{
    INT32S  ret;
    INT32U  msg_id;
	STAudioTaskPara     *pstAudioTaskPara;
    INT32U  i2s_ch;

	audio_i2s_decode_task_init();
	audio_i2s_init();

	while (1)
	{
	    /* Pend task message */
	    ret = msgQReceive(AudioTaskQ, &msg_id, (void*)audio_para, AUDIO_PARA_MAX_LEN);
        if(ret < 0) {
            continue;
        }

        pstAudioTaskPara = (STAudioTaskPara*) audio_para;
        i2s_ch = pstAudioTaskPara->i2s_channel;

		switch(msg_id) {
			case MSG_AUD_PLAY: /* by file handle */
			case MSG_AUD_PLAY_BY_SPI:

				audio_i2s_start(pstAudioTaskPara);
				break;
			case MSG_AUD_STOP:
				audio_i2s_stop(pstAudioTaskPara);
				break;
			case MSG_AUD_PAUSE:
				audio_i2s_pause(pstAudioTaskPara);
				break;
			case MSG_AUD_RESUME:
				audio_i2s_resume(pstAudioTaskPara);
				break;
			case MSG_AUD_SET_MUTE:
				audio_i2s_mute_set(pstAudioTaskPara);
				break;
			case MSG_AUD_VOLUME_SET:
                audio_i2s_volume_set(pstAudioTaskPara);
				break;
			case MSG_AUD_DECODE_NEXT_FRAME:
				audio_i2s_decode_next_frame(pstAudioTaskPara);
				break;
            case MSG_AUD_DECODE_STOP_UNFINISHED:
                if(audio_i2s_stop_unfinished(i2s_ch) == AUDIO_ERR_NONE ){
                    audio_send_result(MSG_AUD_STOP_RES,AUDIO_ERR_NONE);
                }
                break;
            case MSG_AUD_DECODE_STOP_FINISHED:
                if(audio_i2s_stop_finished(i2s_ch) == AUDIO_ERR_NONE ){
                    if (i2s_decode_end != NULL) {
                        DBG_PRINT("ch%d i2s decode end\r\n", i2s_ch);
                        i2s_decode_end(1, i2s_ch);
                    }
                }
                break;
            case MSG_AUD_PAUSE_WAIT:
                if(drv_l1_i2s_tx_dma_status_get(i2s_ch) || drv_l1_i2s_tx_dbf_status_get(i2s_ch))
                {
                        Aud_Para[i2s_ch].i2s_channel = i2s_ch;
                        msgQSend(AudioTaskQ, MSG_AUD_PAUSE_WAIT, (void *)&Aud_Para[i2s_ch], 0, MSG_PRI_NORMAL);
                }
                else
                {
                    stopped_i2s[i2s_ch] = 1;
                    drv_l1_i2s_tx_dbf_free(i2s_ch);

                    audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAY_PAUSE;
                    audio_send_result(MSG_AUD_PAUSE_RES, AUDIO_ERR_NONE);
                }
                break;
			default:
				break;
		}
	}
}

static void audio_i2s_init(void)
{
    INT32U  i2s_ch;

    for(i2s_ch=0; i2s_ch<4; i2s_ch++){
        audio_i2s_ctrl[i2s_ch].ring_buf = (INT8U*) gp_iram_malloc(RING_BUF_SIZE);
        if (audio_i2s_ctrl[i2s_ch].ring_buf == NULL) {
            audio_i2s_ctrl[i2s_ch].ring_buf = (INT8U*) gp_malloc(RING_BUF_SIZE);
        }

        audio_i2s_ctrl[i2s_ch].ring_size = RING_BUF_SIZE;
        audio_i2s_ctrl[i2s_ch].wi = 0;
        audio_i2s_ctrl[i2s_ch].ri = 0;
        audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAY_STOP;
        stopped_i2s[i2s_ch] = 1;
	}

	audio_rampdown_disable_flag = 0;
	i2s_decode_end = NULL;
}

static void audio_i2s_start(STAudioTaskPara *pAudioTaskPara)
{
	INT32S ret;
    INT32U i2s_ch = pAudioTaskPara->i2s_channel;

	stopped_i2s[i2s_ch] = 1;
	audio_i2s_queue_clear(i2s_ch);
	audio_i2s_context_p[i2s_ch]->source_type = pAudioTaskPara->src_type;

	if (audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_FS) {
		ret = audio_i2s_play_file_set(pAudioTaskPara->fd, audio_i2s_context_p[i2s_ch], &audio_i2s_ctrl[i2s_ch], audio_i2s_fsq[i2s_ch], i2s_ch);
	}
	else if (audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
	{
		audio_i2s_context_p[i2s_ch]->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_i2s_context_p[i2s_ch]->audio_format == AUDIO_TYPE_NONE) {
		   DBG_PRINT("audio_play_file_set find not support audio3333.\r\n");
	   	}
	   	audio_i2s_ctrl[i2s_ch].file_handle = pAudioTaskPara->fd;
		audio_i2s_ctrl[i2s_ch].file_len = pAudioTaskPara->file_len;
		ret = AUDIO_ERR_NONE;
	}
	else if(audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE)	// added by Bruce, 2010/01/22
	{
		audio_i2s_context_p[i2s_ch]->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_i2s_context_p[i2s_ch]->audio_format == AUDIO_TYPE_NONE) {
		   DBG_PRINT("audio_play_file_set find not support audio4444.\r\n");
	   	}
		audio_i2s_ctrl[i2s_ch].file_handle = pAudioTaskPara->fd;
		audio_i2s_ctrl[i2s_ch].file_len = pAudioTaskPara->file_len;
		ret = AUDIO_ERR_NONE;
	}
	else {
		ret = AUDIO_ERR_NONE;
		audio_i2s_context_p[i2s_ch]->audio_format = pAudioTaskPara->audio_format;
		audio_i2s_ctrl[i2s_ch].file_len = pAudioTaskPara->file_len;
		audio_i2s_ctrl[i2s_ch].file_handle = pAudioTaskPara ->fd;
	}

	if (ret == AUDIO_ERR_NONE) {
		if (audio_i2s_mem_alloc(audio_i2s_context_p[i2s_ch]->audio_format,&audio_i2s_ctrl[i2s_ch],pcm_i2s_out[i2s_ch]) != AUDIO_ERR_NONE) {
			DBG_PRINT("audio memory allocate fail\r\n");
			audio_send_result(MSG_AUD_PLAY_RES,AUDIO_ERR_MEM_ALLOC_FAIL);
			return;
		}

		audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAYING;

		switch(audio_i2s_context_p[i2s_ch]->audio_format) {

		#if APP_OGG_DECODE_FG_EN == 1
			case AUDIO_TYPE_OGG:
				audio_i2s_context_p[i2s_ch]->fp_deocde_init = audio_i2s_ogg_play_init;
				audio_i2s_context_p[i2s_ch]->fp_deocde = audio_i2s_ogg_process;
				audio_i2s_ctrl[i2s_ch].ring_size = OGGVORBIS_DEC_BITSTREAM_BUFFER_SIZE;
				audio_i2s_ctrl[i2s_ch].frame_size = OGG_PCM_BUF_SIZE;
				break;
		#endif
			default:
				audio_send_result(MSG_AUD_BG_PLAY_RES,AUDIO_ERR_INVALID_FORMAT);
				return;
		}

		ret = audio_i2s_context_p[i2s_ch]->fp_deocde_init(i2s_ch);
		if (ret != AUDIO_ERR_NONE) {
			audio_i2s_stop_unfinished(i2s_ch);
			DBG_PRINT("audio play init failed\r\n");
        } else {
        	audio_i2s_send_next_frame_q(i2s_ch);
		}
	} else {
   		ret = AUDIO_ERR_INVALID_FORMAT;
	}

	audio_send_result(MSG_AUD_PLAY_RES,ret);
}

static void audio_i2s_stop(STAudioTaskPara *pAudioTaskPara)
{
    INT32U i2s_ch = pAudioTaskPara->i2s_channel;
	if ((audio_i2s_context_p[i2s_ch]->state == AUDIO_PLAY_STOP)||(audio_i2s_context_p[i2s_ch]->state == AUDIO_IDLE)) {
		audio_send_result(MSG_AUD_STOP_RES,AUDIO_ERR_NONE);
		return;
	}
#if 0//TBD: ramp down hangup issue
	if(audio_rampdown_disable_flag == 0) {
		if (audio_i2s_context_p[i2s_ch]->state == AUDIO_PLAYING) {
		#if (USE_RAMP_DOWN_RAPID == 0)
			audio_i2s_ramp_down(i2s_ch);		//ramp down speed normal
		#else
			audio_i2s_ramp_down_rapid();
		#endif
		}
	}
#endif
    Aud_Para[i2s_ch].i2s_channel = i2s_ch;
    msgQSend(AudioTaskQ, MSG_AUD_DECODE_STOP_UNFINISHED, (void *)&Aud_Para[i2s_ch], 0, MSG_PRI_NORMAL);
}

static void audio_i2s_ramp_down(INT32U i2s_ch)
{
	INT8U   wb_idx;
	INT8U   err, ramp_down_step;
	INT16S  *ptr;
	INT16S  last_ldata,last_rdata;
	INT32U  i, j, buf_len = audio_i2s_ctrl[i2s_ch].frame_size >> 1;
    //INT32U  ch_id = 0;      //temp declare for i2s
	if(g_audio_sample_rate > 16000) {
		ramp_down_step = RAMP_DOWN_STEP;
	} else {
		ramp_down_step = RAMP_DOWN_STEP_LOW_SR;
	}

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQFlush(aud_send_q);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueReset(aud_i2s_send_q[i2s_ch]);
#endif
    last_ldata = *(pcm_i2s_out[i2s_ch][last_send_idx] + pcm_i2s_len[i2s_ch][last_send_idx]-2);
	last_rdata = *(pcm_i2s_out[i2s_ch][last_send_idx] + pcm_i2s_len[i2s_ch][last_send_idx]-1);

	last_ldata = last_ldata & ~(ramp_down_step-1);
	if (channel == 2) {
		last_rdata = last_rdata & ~(ramp_down_step-1);
	} else {
		last_rdata = 0;
	}

	DBG_PRINT("ldata = 0x%x\r\n",last_ldata);
	DBG_PRINT("rdata = 0x%x\r\n",last_rdata);

	while(1) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
        wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        err = (INT8U) xQueueReceive(audio_i2s_wq[i2s_ch], &wb_idx, portMAX_DELAY);
        if(err != pdPASS) {
            continue;
        }
    #endif
		ptr = (INT16S*) pcm_i2s_out[i2s_ch][wb_idx];
		audio_i2s_send_to_dma(i2s_ch);

		for (i=0; i<(buf_len/RAMP_DOWN_STEP_HOLD); i++) {
			for (j=0; j<RAMP_DOWN_STEP_HOLD; j++) {
				*ptr++ = last_ldata;
				if (channel == 2) {
					*ptr++ = last_rdata;
				}
			}

			if (last_ldata > 0x0) {
				last_ldata -= ramp_down_step;
			}
			else if (last_ldata < 0x0) {
				last_ldata += ramp_down_step;
			}

			if (channel == 2) {
				if (last_rdata > 0x0) {
					last_rdata -= ramp_down_step;
		        }
				else if (last_rdata < 0x0) {
					last_rdata += ramp_down_step;
		        }
		    }
	    }

		pcm_i2s_len[i2s_ch][wb_idx] = buf_len * channel;

    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(aud_send_q, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(aud_i2s_send_q[i2s_ch], &wb_idx, portMAX_DELAY);
    #endif
    #if 0   //dac
        if (drv_l1_dac_dma_status_get() == 0) {
    #else   //i2s
        if (drv_l1_i2s_tx_dma_status_get(i2s_ch) == 0){
    #endif
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_DMA_DBF_RESTART);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            INT32U message = MSG_AUD_DMA_DBF_RESTART;
            xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
            xQueueSend(aud_i2s_profile_q, &p_ogg_i2s_profile[i2s_ch], portMAX_DELAY);
        #endif
        }

		if ((last_ldata == 0x0) && (last_rdata == 0x0)) {
			break;
		}
	}
}

#if USE_RAMP_DOWN_RAPID == 1
static void audio_i2s_ramp_down_rapid(void)
{
	INT16S  last_ldata,last_rdata;
	INT16S  i, temp;

	OSQFlush(aud_send_q);
	while((dac_dma_status_get() == 1) || (dac_dbf_status_get() == 1)) {
		OSTimeDly(2);
	}
	//free and reset DMA channel
	dac_dbf_free();

	temp = 0 - RAMP_DOWN_STEP;
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
	dac_sample_rate_set(44100);

	while(1)
	{
		if (last_ldata > 0x0) {
			last_ldata -= RAMP_DOWN_STEP;
		}
		else if (last_ldata < 0x0) {
			last_ldata += RAMP_DOWN_STEP;
		}

		if ((last_ldata < RAMP_DOWN_STEP)&&(last_ldata > temp)) {
			last_ldata = 0;
		}

		if (channel == 2) {
			if (last_rdata > 0x0) {
				last_rdata -= RAMP_DOWN_STEP;
		    }
			else if (last_rdata < 0x0) {
				last_rdata += RAMP_DOWN_STEP;
		    }

		    if ((last_rdata < RAMP_DOWN_STEP)&&(last_rdata > temp)) {
				last_rdata = 0;
			}
		}

		for(i=0;i<RAMP_DOWN_STEP_HOLD;i++) {
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
}
#endif //USE_RAMP_DOWN_RAPID
/*== AVI audio ctrl Setting explain ==*/
// audio_ctrl.f_last => 1:it mean buffer read file over. 0: it mean buffer read file continue
// when audio play over ,the audio_ctrl.ri == audio_ctrl.wi.
/*== ==*/


static void audio_i2s_pause(STAudioTaskPara *pAudioTaskPara)
{
    INT32U  i2s_ch = pAudioTaskPara->i2s_channel;

#if (_OPERATING_SYSTEM == _OS_FREERTOS)
    INT32U message;
#endif
	if (audio_i2s_context_p[i2s_ch]->state != AUDIO_PLAYING) {
		audio_send_result(MSG_AUD_PAUSE_RES, AUDIO_ERR_NONE);
		return;
	}

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(hAudioDacTaskQ, (void *)MSG_AUD_DMA_PAUSE);
    while(dac_dma_status_get() || dac_dbf_status_get()) {
		OSTimeDly(2);
	}
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    message = MSG_AUD_DMA_PAUSE;
    xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
    xQueueSend(aud_i2s_profile_q, &p_ogg_i2s_profile[i2s_ch], portMAX_DELAY);


    Aud_Para[i2s_ch].i2s_channel = i2s_ch;
    msgQSend(AudioTaskQ, MSG_AUD_PAUSE_WAIT, (void *)&Aud_Para[i2s_ch], 0, MSG_PRI_NORMAL);
    audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAY_PAUSE;
#endif

}

static void audio_i2s_resume(STAudioTaskPara *pAudioTaskPara)
{
    INT32U i2s_ch = pAudioTaskPara->i2s_channel;
	if (audio_i2s_context_p[i2s_ch]->state != AUDIO_PLAY_PAUSE) {
		audio_send_result(MSG_AUD_RESUME_RES, AUDIO_ERR_NONE);
		return;
	}
	//dac_enable_set(TRUE);
	audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAYING;
	audio_i2s_send_next_frame_q(i2s_ch);
	audio_send_result(MSG_AUD_RESUME_RES,AUDIO_ERR_NONE);
}

static void audio_i2s_mute_set(STAudioTaskPara *pAudioTaskPara)
{
    INT32U i2s_ch = pAudioTaskPara->i2s_channel;
	if (pAudioTaskPara->mute == TRUE) {
        DBG_PRINT("wolfson mute\r\n");
        wolfson_WM8988_tx_mute(i2s_ch);      //channel 0, temp setting

	}
	else {
        DBG_PRINT("wolfson unmute\r\n");
        wolfson_WM8988_tx_unmute(i2s_ch);

	}
	audio_send_result(MSG_AUD_MUTE_SET_RES,AUDIO_ERR_NONE);
}

static void audio_i2s_volume_set(STAudioTaskPara *pAudioTaskPara)
{
    INT32U i2s_ch = pAudioTaskPara->i2s_channel;
	if (pAudioTaskPara->volume < 80) {
        INT32U amplitude = pAudioTaskPara->volume+48;

            i2s_volume_set(i2s_ch, amplitude);
    }
	audio_send_result(MSG_AUD_VOLUME_SET_RES,AUDIO_ERR_NONE);
}

static void audio_i2s_decode_next_frame(STAudioTaskPara *pAudioTaskPara)
{
	INT32S ret;
	INT32U i2s_ch = pAudioTaskPara->i2s_channel;

    //DBG_PRINT("i2s_ch %d\r\n",i2s_ch);
	if (audio_i2s_context_p[i2s_ch]->state != AUDIO_PLAYING) {
		return;
	}
	ret = audio_i2s_context_p[i2s_ch]->fp_deocde(i2s_ch);
	if (ret != 0) {
        //DBG_PRINT("decode err=%x\n\r\n",ret);
        Aud_Para[i2s_ch].i2s_channel = i2s_ch;
        msgQSend(AudioTaskQ, MSG_AUD_DECODE_STOP_FINISHED, (void *)&Aud_Para[i2s_ch], 0, MSG_PRI_NORMAL);
	}
}

static void audio_i2s_send_next_frame_q(INT32U i2s_ch)
{
	//DBG_PRINT(".");
	Aud_Para[i2s_ch].i2s_channel = i2s_ch;
	msgQSend(AudioTaskQ, MSG_AUD_DECODE_NEXT_FRAME, (void *)&Aud_Para[i2s_ch], 0, MSG_PRI_NORMAL);
}
#if 0
void audio_send_result(INT32S res_type,INT32S result)
{
	aud_con.result_type = res_type;
	aud_con.result = result;
	//DBG_PRINT("audio_send_result :res_type =  %x,result = %d\r\n",res_type,result);
	msgQSend(Audio_FG_status_Q, EVENT_APQ_ERR_MSG, (void *)&aud_con, sizeof(STAudioConfirm), MSG_PRI_NORMAL);
}
#endif
static INT32S audio_i2s_stop_unfinished(INT32U i2s_ch)
{
	INT32S i;

    if(drv_l1_i2s_tx_dma_status_get(i2s_ch) == 1)
    {
        Aud_Para[i2s_ch].i2s_channel = i2s_ch;
        audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAY_STOP;
        msgQSend(AudioTaskQ, MSG_AUD_DECODE_STOP_UNFINISHED, (void *)&Aud_Para[i2s_ch], 0, MSG_PRI_NORMAL);
        return AUDIO_ERR_FAILED;
    }

	if (audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_FS
	    ||audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_USER_DEFINE) {
		if (audio_i2s_ctrl[i2s_ch].file_handle >= 0) {
			fs_close(audio_i2s_ctrl[i2s_ch].file_handle); //who open ,who close
		}
	}
    drv_l1_i2s_tx_dbf_free(i2s_ch);

	/* free memory */
	if(audio_i2s_context_p[i2s_ch]->audio_format != AUDIO_TYPE_AVI)
	{
		gp_free(audio_i2s_ctrl[i2s_ch].work_mem);
	}
	audio_i2s_ctrl[i2s_ch].work_mem = NULL;

	for (i=0;i<MAX_I2S_BUFFERS;i++) {
		gp_free(*(pcm_i2s_out[i2s_ch]+i));
		*(pcm_i2s_out[i2s_ch]+i) = NULL;
	}

	audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAY_STOP;
	audio_play_start_time = 0;

	return AUDIO_ERR_NONE;
}

static INT32S audio_i2s_stop_finished(INT32U i2s_ch)
{
	INT32S i;

    if(drv_l1_i2s_tx_dma_status_get(i2s_ch) == 1)
    {
        Aud_Para[i2s_ch].i2s_channel = i2s_ch;
        audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAY_STOP;
        msgQSend(AudioTaskQ, MSG_AUD_DECODE_STOP_FINISHED, (void *)&Aud_Para[i2s_ch], 0, MSG_PRI_NORMAL);
        return AUDIO_ERR_FAILED;
    }

	if (audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_FS
	    ||audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_USER_DEFINE) {
		if (audio_i2s_ctrl[i2s_ch].file_handle >= 0) {
			fs_close(audio_i2s_ctrl[i2s_ch].file_handle); //who open ,who close
		}
	}
    drv_l1_i2s_tx_dbf_free(i2s_ch);


	/* free memory */
	if(audio_i2s_context_p[i2s_ch]->audio_format != AUDIO_TYPE_AVI)
	{
		gp_free(audio_i2s_ctrl[i2s_ch].work_mem);
	}
	audio_i2s_ctrl[i2s_ch].work_mem = NULL;

	for (i=0;i<MAX_I2S_BUFFERS;i++) {
		gp_free(*(pcm_i2s_out[i2s_ch]+i));
		*(pcm_i2s_out[i2s_ch]+i) = NULL;
	}

	audio_i2s_context_p[i2s_ch]->state = AUDIO_PLAY_STOP;
	audio_play_start_time = 0;

	return AUDIO_ERR_NONE;
}
#if 1
INT32S audio_i2s_mem_alloc(INT32U audio_format,AUDIO_CTRL *aud_ctrl, INT16S *pcm[])
{
	INT32S i;
	INT32U pcm_size;
	INT32U wm_size;

	switch(audio_format) {

		case AUDIO_TYPE_AVI:
			wm_size = WAV_DEC_MEMORY_SIZE;
			pcm_size = WAV_PCM_BUF_SIZE;
			break;
		case AUDIO_TYPE_AVI_MP3:
			wm_size = MP3_DEC_MEMORY_SIZE;
			pcm_size = MP3_PCM_BUF_SIZE;
			break;
	#if APP_OGG_DECODE_FG_EN == 1 || APP_OGG_DECODE_BG_EN == 1
 	  	case AUDIO_TYPE_OGG:
  	  		wm_size = OGGVORBIS_DEC_MEMORY_SIZE;
  	  		pcm_size = OGG_PCM_BUF_SIZE;
  	  		break;
	#endif
		default:
			//return AUDIO_ERR_FAILED;
			return AUDIO_ERR_NONE;//0809003
	}

	if(audio_format == AUDIO_TYPE_AVI) {
		aud_ctrl->work_mem = avi_work_mem;
	} else {
		aud_ctrl->work_mem = (INT8S*) gp_malloc(wm_size);
		if (aud_ctrl->work_mem == NULL) {
			return AUDIO_ERR_FAILED;
		}
	}
	gp_memset(aud_ctrl->work_mem,0,wm_size);
	//DBG_PRINT("decode memory = 0x%x (%d)\r\n",aud_ctrl->work_mem,wm_size);

	pcm_size += 2 /* add for SPU end data */;
	for (i=0;i<MAX_I2S_BUFFERS;i++) {
		pcm[i] = (INT16S*) gp_malloc(pcm_size*2);
		//DBG_PRINT("pcm buffer[%d] = 0x%x (%d)\r\n",i,pcm[i],pcm_size*2);
		if (pcm[i] == NULL) {
			while(i>0) {
				gp_free(pcm[--i]);
				pcm[i] = NULL;
			}
			return AUDIO_ERR_FAILED;
		}
	}

	return AUDIO_ERR_NONE;
}
#endif
#if (_OPERATING_SYSTEM == _OS_UCOS2)
INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, OS_EVENT *ack_fsq)
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
INT32S audio_i2s_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, xQueueHandle ack_fsq, INT32U i2s_ch)
#endif
{
	sfn_stat(fd,&aud_i2s_sfn_file[i2s_ch]);
#if (defined AUDIO_FORMAT_JUDGE_AUTO) && (AUDIO_FORMAT_JUDGE_AUTO == 1)
	aud_context->audio_format = audio_get_type(fd, aud_i2s_sfn_file[i2s_ch].f_extname, ack_fsq);
#else
	aud_context->audio_format = audio_get_type(aud_i2s_sfn_file[i2s_ch].f_extname);
#endif
   	if (aud_context->audio_format == AUDIO_TYPE_NONE) {
	   DBG_PRINT("audio_play_file_set find not support audio5555.\r\n");
   	   return AUDIO_ERR_INVALID_FORMAT;
   	}
	aud_ctrl->file_handle = fd;

	aud_ctrl->file_len = aud_i2s_sfn_file[i2s_ch].f_size;

	return AUDIO_ERR_NONE;
}
#if 0
#if (_OPERATING_SYSTEM == _OS_UCOS2)
INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, OS_EVENT *ack_fsq)
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, xQueueHandle ack_fsq)
#endif
{

	sfn_stat(fd,&aud_sfn_file);
#if (defined AUDIO_FORMAT_JUDGE_AUTO) && (AUDIO_FORMAT_JUDGE_AUTO == 1)
	aud_context->audio_format = audio_get_type(fd, aud_sfn_file.f_extname, ack_fsq);
#else
	aud_context->audio_format = audio_get_type(aud_sfn_file.f_extname);
#endif
   	if (aud_context->audio_format == AUDIO_TYPE_NONE) {
	   DBG_PRINT("audio_play_file_set find not support audio6666.\r\n");
   	   return AUDIO_ERR_INVALID_FORMAT;
   	}
	aud_ctrl->file_handle = fd;
	aud_ctrl->file_len = aud_sfn_file.f_size;

	return AUDIO_ERR_NONE;
}
#endif


#if (defined AUDIO_FORMAT_JUDGE_AUTO) && (AUDIO_FORMAT_JUDGE_AUTO == 1)


#if (_OPERATING_SYSTEM == _OS_UCOS2)
static INT32S audio_get_type(INT16S fd,INT8S* file_name, OS_EVENT *ack_fsq)
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
static INT32S audio_get_type(INT16S fd,INT8S* file_name, xQueueHandle ack_fsq)
#endif
{
   	INT8S temp[5] = {0};
	INT16U i;

	gp_strcpy(temp,file_name);
	for(i=0;i<gp_strlen(temp);i++) {
		temp[i] = gp_toupper(temp[i]);
	}

#if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT == GP_FILE_FORMAT_SET_1)
	if(gp_strcmp(temp, (INT8S *)"GA")==0) {
		return AUDIO_TYPE_WMA;
   	} else {
   	    return AUDIO_TYPE_NONE;
   	}
#endif


    if(gp_strcmp(temp, (INT8S *)"IDI")==0) {
    	return AUDIO_TYPE_MIDI;
    }

    if(gp_strcmp(temp, (INT8S *)"GMD")==0) {
    	return AUDIO_TYPE_MIDI;
    }

#if APP_OGG_DECODE_FG_EN == 1 || APP_OGG_DECODE_BG_EN == 1
   	if(gp_strcmp(temp, (INT8S *)"OGG")==0) {
		return AUDIO_TYPE_OGG;
	}
#endif

   	return AUDIO_TYPE_NONE;
}
#else
static INT32S audio_get_type(INT8S* file_name)
{
   	INT8S  	temp[5] = {0};
   	INT16U 	i;

	gp_strcpy(temp,file_name);
	for (i=0;i<gp_strlen(temp);i++) {
		temp[i] = gp_toupper(temp[i]);
	}

#if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT == GP_FILE_FORMAT_SET_1)
    if(gp_strcmp(temp, (INT8S *)"GA")==0) {
		return AUDIO_TYPE_WMA;
   	} else {
   	    return AUDIO_TYPE_NONE;
   	}
#endif


    if(gp_strcmp(temp, (INT8S *)"IDI")==0) {
    	return AUDIO_TYPE_MIDI;
    }

    if(gp_strcmp(temp, (INT8S *)"GMD")==0) {
    	return AUDIO_TYPE_MIDI;
    }


#if APP_OGG_DECODE_FG_EN == 1 || APP_OGG_DECODE_BG_EN == 1
	if(gp_strcmp(temp, (INT8S *)"OGG")==0) {
		return AUDIO_TYPE_OGG;
	}
#endif

   	return AUDIO_TYPE_NONE;
}
#endif


static void audio_i2s_queue_clear(INT32U i2s_ch)
{
    INT8U i;
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	//OSQFlush(hAudioDacTaskQ);
	OSQFlush(aud_send_q);
	OSQFlush(audio_wq);
	OSQFlush(audio_fsq);
	OSQPost(hAudioDacTaskQ, (void *)MSG_AUD_DMA_WIDX_CLEAR);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    INT32U message = MSG_AUD_DMA_WIDX_CLEAR;

    xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
    xQueueSend(aud_i2s_profile_q, &p_ogg_i2s_profile[i2s_ch], portMAX_DELAY);

    xQueueReset(aud_i2s_send_q[i2s_ch]);
    xQueueReset(audio_i2s_wq[i2s_ch]);
    xQueueReset(audio_i2s_fsq[i2s_ch]);


#endif
}
static INT32S audio_i2s_write_with_file_srv(INT8U *ring_buf, INT32U wi, INT32U ri, INT32U i2s_ch)
{
	INT32S t;
	INT32S len;
	//INT8U  err;
	INT32S  err;
	INT32U msg_id;
	TK_FILE_SERVICE_STRUCT audio_fs_para;

	switch(audio_i2s_context_p[i2s_ch]->source_type) {
		case AUDIO_SRC_TYPE_FS:
			msg_id = MSG_FILESRV_FS_READ;
			break;
		case AUDIO_SRC_TYPE_GPRS:
			msg_id = MSG_FILESRV_NVRAM_AUDIO_GPRS_READ;
			audio_fs_para.spi_para.sec_offset = audio_i2s_ctrl[i2s_ch].read_secs;
			break;
		case AUDIO_SRC_TYPE_APP_RS:
			msg_id = MSG_FILESRV_NVRAM_AUDIO_APP_READ;
			break;
		case AUDIO_SRC_TYPE_APP_PACKED_RS:
			msg_id = MSG_FILESRV_NVRAM_AUDIO_APP_PACKED_READ;
			break;
		case AUDIO_SRC_TYPE_USER_DEFINE:	// added by Bruce, 2008/10/27
			if(G_SACM_Ctrl.Offsettype != SND_OFFSET_TYPE_NONE) {
				msg_id = MSG_FILESRV_MAX;
			} else {
				msg_id = MSG_FILESRV_USER_DEFINE_READ;
			}
			audio_fs_para.data_start_addr = audio_i2s_ctrl[i2s_ch].data_start_addr;
			audio_fs_para.data_offset = audio_i2s_ctrl[i2s_ch].data_offset;
			break;
		case AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE:	// added by Bruce, 2010/01/22
			msg_id = MSG_FILESRV_FS_READ;
			break;
		default:
			break;
	}

	audio_fs_para.fd = audio_i2s_ctrl[i2s_ch].file_handle;
	audio_fs_para.result_queue = audio_i2s_fsq[i2s_ch];
	audio_fs_para.main_channel = AUDIO_CHANNEL_DAC;

	if(wi == 0 && ri == 0) {
		audio_fs_para.buf_addr = (INT32U)ring_buf;
		audio_fs_para.buf_size = audio_i2s_ctrl[i2s_ch].ring_size/2;
		audio_fs_para.spi_para.sec_cnt = audio_i2s_ctrl[i2s_ch].ring_size/1024;

		if(msg_id == MSG_FILESRV_MAX)
		{
			if((G_SACM_Ctrl.Offsettype = SND_OFFSET_TYPE_SIZE)||(G_SACM_Ctrl.Offsettype = SND_OFFSET_TYPE_TIME))
			{
				if(G_SACM_Ctrl.AudioFormat == MP3)
				{
					if(G_SACM_Ctrl.Offset > audio_i2s_ctrl[i2s_ch].file_len)
						return AUDIO_READ_FAIL;

					if(Snd_Lseek(G_SACM_Ctrl.Offset,SEEK_CUR) >= 0)
						audio_i2s_ctrl[i2s_ch].file_cnt += G_SACM_Ctrl.Offset;
					else
						return AUDIO_READ_FAIL;

					len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
					audio_i2s_ctrl[i2s_ch].read_secs += (len/512);
			        wi += len;
				}
				else if(G_SACM_Ctrl.AudioFormat == WMA)                  //add WMA_Seek
				{
				    len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
					audio_i2s_ctrl[i2s_ch].read_secs += (len/512);
			        wi += len;
				}
				else
				{	// use jmp to play
					len = Snd_GetData(audio_fs_para.buf_addr,6);
					audio_fs_para.buf_addr += 6;
					audio_fs_para.buf_size -= 6;
					if(G_SACM_Ctrl.Offset > audio_i2s_ctrl[i2s_ch].file_len)
						return AUDIO_READ_FAIL;

					if(Snd_Lseek(G_SACM_Ctrl.Offset,SEEK_CUR) >= 0)
						audio_i2s_ctrl[i2s_ch].file_cnt += G_SACM_Ctrl.Offset + 6;
					else
						return AUDIO_READ_FAIL;

					len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
					len += 6;
					audio_i2s_ctrl[i2s_ch].read_secs += (len/512);
			        wi += len;
				}
			}
			else
			{	// not use jmp to play
				len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
				audio_i2s_ctrl[i2s_ch].read_secs += (len/512);
	        	wi += len;
			}
			return wi;
		}
		else
		{
            msgQSend(fs_msg_q_id, msg_id, (void *)&audio_fs_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
            len = (INT32S) OSQPend(audio_fsq, 0, &err);
            if ((err != OS_NO_ERR) || len < 0) {
        	    return AUDIO_READ_FAIL;
            }
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            err = (INT32S) xQueueReceive(audio_i2s_fsq[i2s_ch], &len, portMAX_DELAY);
            if((err != pdPASS) || (len < 0)) {
                return AUDIO_READ_FAIL;
            }
        #endif

			if (audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
				audio_i2s_ctrl[i2s_ch].data_offset += len;
            audio_i2s_ctrl[i2s_ch].read_secs += (len/512);
            wi += len;
            return wi;
        }
    }

    len = ri - wi;
    if (len <= 0) {
    	len += audio_i2s_ctrl[i2s_ch].ring_size;
    }
    if(len < audio_i2s_ctrl[i2s_ch].ring_size/2) {
    	return wi;
    }

	t = wi;
	wi += audio_i2s_ctrl[i2s_ch].ring_size/2;
	if(wi == audio_i2s_ctrl[i2s_ch].ring_size) {
		wi = 0;
	}
	if(wi == ri) {
		return t;
	}

	audio_fs_para.buf_addr = (INT32U)ring_buf+t;
	audio_fs_para.buf_size = audio_i2s_ctrl[i2s_ch].ring_size/2;

	audio_fs_para.spi_para.sec_cnt = audio_i2s_ctrl[i2s_ch].ring_size/1024;

	if(msg_id == MSG_FILESRV_MAX) {
		len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_fsq, (void *) len);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        DBG_PRINT("send len=%d\r\n",len);
        xQueueSend(audio_i2s_fsq[i2s_ch], &len, portMAX_DELAY);
    #endif
	} else {
		msgQSend(fs_msg_q_id, msg_id, (void *)&audio_fs_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);
	}
	return AUDIO_READ_PEND;
}

INT32S audio_i2s_check_wi(INT32S wi_in, INT32U *wi_out, INT8U wait, INT32U i2s_ch)
{
	INT32S len;
	//INT8U  err;
	INT32S err;
	INT32S t;

	if (wi_in >= 0) {
		*wi_out = wi_in;
		return AUDIO_ERR_NONE;
	}
	if (wi_in == AUDIO_READ_FAIL) {
		return AUDIO_READ_FAIL;
	}
	/* AUDIO_READ_PEND */
	t = audio_i2s_ctrl[i2s_ch].wi;

	if (wait) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
            len = (INT32S) OSQPend(audio_fsq, 0, &err);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            err = (INT32S) xQueueReceive(audio_i2s_fsq[i2s_ch], &len, portMAX_DELAY);
            //DBG_PRINT("len aaaa=%d\r\n",len);
            #endif
	}
	else {
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
                len = (INT32U)OSQAccept(audio_fsq, &err);
		if (err == OS_Q_EMPTY) {
			*wi_out = *wi_out;
			return 	AUDIO_ERR_NONE;
		}
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                err = (INT32S) xQueueReceive(audio_i2s_fsq[i2s_ch], &len, 0);
                if(err == pdPASS) {
			*wi_out = *wi_out;
			return 	AUDIO_ERR_NONE;
                }
                #endif
	}

	audio_i2s_ctrl[i2s_ch].reading = 0;
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	if ((err != OS_NO_ERR) || len < 0) {
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        if ((err != pdPASS) || len < 0) {
        #endif
            return AUDIO_READ_FAIL;
    }
	if (audio_i2s_context_p[i2s_ch]->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
		audio_i2s_ctrl[i2s_ch].data_offset += len;
    audio_i2s_ctrl[i2s_ch].read_secs += (len/512);
	t += len;
	if(t == audio_i2s_ctrl[i2s_ch].ring_size) {
		t = 0;
	}

	*wi_out = t;
	return 	AUDIO_ERR_NONE;
}


static INT32S audio_i2s_send_to_dma(INT32U i2s_ch)
{
	INT32U count;

	if (stopped_i2s[i2s_ch] && (audio_i2s_q_check(i2s_ch)==AUDIO_SEND_FAIL)) { /* queuq full */
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
        stopped_i2s[i2s_ch] = 0;
		OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_DMA_DBF_START);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        INT32U message = MSG_AUD_DMA_DBF_START;
        stopped_i2s[i2s_ch] = 0;
        xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);

        xQueueSend(aud_i2s_profile_q, &p_ogg_i2s_profile[i2s_ch], portMAX_DELAY);
    #endif
    }
	else if (drv_l1_i2s_tx_dma_status_get(i2s_ch) == 0){
		if (!stopped_i2s[i2s_ch]) {
		#if 1 /* i2s underrun, wait queue full again and start DMA*/
			stopped_i2s[i2s_ch] = 1;
			drv_l1_i2s_tx_dbf_free(i2s_ch);

			DBG_PRINT("ch %d i2s underrun !\r\n",i2s_ch);

			audio_i2s_queue_clear(i2s_ch);
			for (count=0;count<MAX_I2S_BUFFERS;count++) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(audio_wq, (void *) count);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                xQueueSend(audio_i2s_wq[i2s_ch], &count, portMAX_DELAY);
            #endif
			}
		#else /* if any buffer into queue, start DMA again */
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_DMA_DBF_RESTART);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                INT32U message = MSG_AUD_DMA_DBF_RESTART;
                xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
                xQueueSend(aud_i2s_profile_q, &p_ogg_i2s_profile[i2s_ch], portMAX_DELAY);
            #endif
        #endif
		}
	}
	return STATUS_OK;
}

static INT32S audio_i2s_q_check(INT32U i2s_ch)
{
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OS_Q      *pq;
	pq = (OS_Q *)aud_send_q->OSEventPtr;
	if (pq->OSQEntries >= 2) {//for fast 090531
		return AUDIO_SEND_FAIL;
	}
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    INT32U Number = *(INT32U *)((INT32U)aud_i2s_send_q[i2s_ch] + 4*4 + sizeof(xList) * 2);
    if(Number >= MAX_I2S_BUFFERS) {
    	return AUDIO_SEND_FAIL;
    }
#endif
	return AUDIO_SEND_OK;
}

//===============================================================================================================
//  OGG Playback
//===============================================================================================================
#if APP_OGG_DECODE_FG_EN == 1

INT32S audio_i2s_ogg_play_init(INT32U i2s_ch)
{
	INT32U  in_length;
	INT32S  ret;
	INT32U  count;
	INT32S  t_wi;
    INT32U message;

	audio_i2s_ctrl[i2s_ch].file_cnt = 0;
	audio_i2s_ctrl[i2s_ch].f_last = 0;
	audio_i2s_ctrl[i2s_ch].try_cnt = 20;
	audio_i2s_ctrl[i2s_ch].read_secs = 0;
	ogg_i2s_profile[i2s_ch].i2s_channel = i2s_ch;

	gp_memset((INT8S*)audio_i2s_ctrl[i2s_ch].ring_buf, 0, audio_i2s_ctrl[i2s_ch].ring_size);

	// aac init
	ret = oggvorbis_dec_init((void *)audio_i2s_ctrl[i2s_ch].work_mem, (const char*)audio_i2s_ctrl[i2s_ch].ring_buf, audio_i2s_ctrl[i2s_ch].ring_size, 0);
	oggvorbis_dec_set_ring_buffer((void *)audio_i2s_ctrl[i2s_ch].work_mem, (const char*)audio_i2s_ctrl[i2s_ch].ring_buf, audio_i2s_ctrl[i2s_ch].ring_size, 0);
	audio_i2s_ctrl[i2s_ch].wi = 0;
	audio_i2s_ctrl[i2s_ch].ri = oggvorbis_dec_get_ri((void *)audio_i2s_ctrl[i2s_ch].work_mem);

	t_wi = audio_i2s_write_with_file_srv(audio_i2s_ctrl[i2s_ch].ring_buf, audio_i2s_ctrl[i2s_ch].wi, audio_i2s_ctrl[i2s_ch].ri, i2s_ch);

	/* check reading data */
	if (audio_i2s_check_wi(t_wi, &audio_i2s_ctrl[i2s_ch].wi, 1, i2s_ch) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_i2s_ctrl[i2s_ch].ri;
		ret = oggvorbis_dec_parsing((void *)audio_i2s_ctrl[i2s_ch].work_mem , audio_i2s_ctrl[i2s_ch].wi);

		audio_i2s_ctrl[i2s_ch].ri = oggvorbis_dec_get_ri((void *)audio_i2s_ctrl[i2s_ch].work_mem);
		audio_i2s_ctrl[i2s_ch].file_cnt += (audio_i2s_ctrl[i2s_ch].ri - in_length);
		if(audio_i2s_ctrl[i2s_ch].ri < in_length) {
			audio_i2s_ctrl[i2s_ch].file_cnt += audio_i2s_ctrl[i2s_ch].ring_size;
		}

		if(audio_i2s_ctrl[i2s_ch].file_cnt >= audio_i2s_ctrl[i2s_ch].file_len) {
			return AUDIO_ERR_FAILED;
		}

		switch(ret)
		{
			case OGGVORBIS_PARSING_OK:
				break;

			case OGGVORBIS_MORE_DATA:
				//feed in DecodeInBuffer;
				audio_i2s_ctrl[i2s_ch].ri = oggvorbis_dec_get_ri((void *)audio_i2s_ctrl[i2s_ch].work_mem);
				t_wi = audio_i2s_write_with_file_srv(audio_i2s_ctrl[i2s_ch].ring_buf, audio_i2s_ctrl[i2s_ch].wi, audio_i2s_ctrl[i2s_ch].ri, i2s_ch);

				/* check reading data */
				if (audio_i2s_check_wi(t_wi, &audio_i2s_ctrl[i2s_ch].wi, 1, i2s_ch) != AUDIO_ERR_NONE) {
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

	in_length = oggvorbis_dec_get_samplerate((void *)audio_i2s_ctrl[i2s_ch].work_mem);
	ret = oggvorbis_dec_get_channel((void *)audio_i2s_ctrl[i2s_ch].work_mem);
	hSrc = audio_i2s_ctrl[i2s_ch].work_mem;
	//pfnGetOutput = audio_ogg_get_output;

#if APP_VOICE_CHANGER_EN
	if(hVC && G_snd_info.VoiceChangerEn)
	{
		VoiceChanger_Link(hVC, hSrc, pfnGetOutput, in_length, ret, 0);
		VoiceChanger_SetParam(hVC, G_snd_info.Speed, G_snd_info.Pitch);
		hSrc = hVC;
		in_length = VoiceChanger_GetSampleRate(hVC);
		ret = VoiceChanger_GetChannel(hVC);
		pfnGetOutput = &VoiceChanger_GetOutput;
	}
#endif


	channel = ret;
	g_audio_sample_rate = oggvorbis_dec_get_samplerate((void *)audio_i2s_ctrl[i2s_ch].work_mem);

    ogg_i2s_profile[i2s_ch].sample_rate = oggvorbis_dec_get_samplerate((void *)audio_i2s_ctrl[i2s_ch].work_mem);
    ogg_i2s_profile[i2s_ch].channel_num = oggvorbis_dec_get_channel((void *)audio_i2s_ctrl[i2s_ch].work_mem);

	DBG_PRINT("bps: %d\r\n", oggvorbis_dec_get_bitrate((void *)audio_i2s_ctrl[i2s_ch].work_mem));
	DBG_PRINT("channel: %d\r\n", oggvorbis_dec_get_channel((void *)audio_i2s_ctrl[i2s_ch].work_mem));
	DBG_PRINT("sample rate: %d\r\n", oggvorbis_dec_get_samplerate((void *)audio_i2s_ctrl[i2s_ch].work_mem));

#if 0
	drv_l1_dac_sample_rate_set(in_length);
	if (ret == 1) {
		drv_l1_dac_mono_set();
	} else {
		drv_l1_dac_stereo_set();
	}
#else
    message = MSG_AUD_I2S_PREPARE;
    xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);

    //i2s_ch_id = 1;
    //xQueueSend(aud_i2s_ch_q, &i2s_ch, portMAX_DELAY);
    xQueueSend(aud_i2s_profile_q, &p_ogg_i2s_profile[i2s_ch], portMAX_DELAY);
#endif



	for (count=0;count<MAX_I2S_BUFFERS;count++) {
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPost(audio_wq, (void *) count);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_i2s_wq[i2s_ch], &count, portMAX_DELAY);
        #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S audio_i2s_ogg_process(INT32U i2s_ch)
{
	INT32S  pcm_point;
	//INT8U   err;
	INT32S  err;
	INT32U  wb_idx;
    INT8U   wait;
    INT32S  t_wi;
    INT32U  in_length;

    #if 1
    	wb_idx = 0;
        err = xQueueReceive(audio_i2s_wq[i2s_ch], &wb_idx, 1);
        if(err!= pdPASS){
            goto _EndOggProc;
        }
    #endif

	audio_i2s_ctrl[i2s_ch].ri = oggvorbis_dec_get_ri((CHAR *)audio_i2s_ctrl[i2s_ch].work_mem);
	t_wi = audio_i2s_write_with_file_srv(audio_i2s_ctrl[i2s_ch].ring_buf, audio_i2s_ctrl[i2s_ch].wi, audio_i2s_ctrl[i2s_ch].ri, i2s_ch);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    #if 0

    	wb_idx = 0;
    	xQueueReceive(audio_i2s_wq[i2s_ch], &wb_idx, portMAX_DELAY);
    #endif
#endif
        wait = 1;
		if (audio_i2s_check_wi(t_wi, &audio_i2s_ctrl[i2s_ch].wi, wait, i2s_ch) != AUDIO_ERR_NONE) {
			return AUDIO_ERR_DEC_FAIL;
		}

		if(audio_i2s_ctrl[i2s_ch].file_cnt >= audio_i2s_ctrl[i2s_ch].file_len) {
			return AUDIO_ERR_DEC_FINISH;
		}

		in_length = audio_i2s_ctrl[i2s_ch].ri;

		pcm_point = oggvorbis_dec_run((CHAR *)audio_i2s_ctrl[i2s_ch].work_mem, pcm_i2s_out[i2s_ch][wb_idx], audio_i2s_ctrl[i2s_ch].wi);
		//DBG_PRINT("ch%d, pcm %d\r\n", i2s_ch, pcm_point);

		audio_i2s_ctrl[i2s_ch].ri = oggvorbis_dec_get_ri((CHAR *)audio_i2s_ctrl[i2s_ch].work_mem);
		audio_i2s_ctrl[i2s_ch].file_cnt += (audio_i2s_ctrl[i2s_ch].ri - in_length);
		if(in_length >= audio_i2s_ctrl[i2s_ch].ri) {
			audio_i2s_ctrl[i2s_ch].file_cnt += audio_i2s_ctrl[i2s_ch].ring_size;

		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_i2s_ctrl[i2s_ch].try_cnt == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
			}

			#if (_OPERATING_SYSTEM == _OS_UCOS2)
                    OSQPostFront(audio_wq, (void *)wb_idx);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                    xQueueSendToFront(audio_i2s_wq[i2s_ch], &wb_idx, portMAX_DELAY);
            #endif
                    audio_i2s_send_next_frame_q(i2s_ch);
                    return 0;

		}
	pcm_i2s_len[i2s_ch][wb_idx] = pcm_point * 2;//oggvorbis_dec_get_channel((CHAR *)audio_ctrl.work_mem);

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_i2s_send_q[i2s_ch], &wb_idx, portMAX_DELAY);
    #endif
	audio_i2s_send_to_dma(i2s_ch);

_EndOggProc:
	audio_i2s_send_next_frame_q(i2s_ch);

	return 0;
}

#endif // #if APP_OGG_DECODE_FG_EN == 1
