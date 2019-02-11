/*
* Purpose: Audio task for sending data to DAC.
*
* Author: Allen Lin
*
* Date: 2008/5/22
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.00
* History :
*/
#include "turnkey_audio_dac_task.h"
#include "drv_l1_dma.h"
#if BOARD_TYPE != BOARD_GPM41XXA_EMU_V1_0
#include "gplib_spu_driver.h"
#endif

#define AUDIO_DAC_QUEUE_MAX     5
#define AUDIO_DAC_SENDQ_SIZE    MAX_DAC_BUFFERS
#define AUDIO_AVI_Q_SIZE        1
#define AUDIO_RIGHT_Q_SIZE      1

#define SPU_LEFT_CH_BIT         (1 << SPU_LEFT_CH)
#define SPU_RIGHT_CH_BIT        (1 << SPU_RIGHT_CH)
#define SPU_COMPRESS_EN			0

#define STEP_AUDIO_STOP         4

// Variables defined in this file
/* Task Q declare */
#if (_OPERATING_SYSTEM == _OS_UCOS2)
OS_EVENT	*hAudioDacTaskQ;
OS_EVENT    *aud_send_q;
OS_EVENT    *aud_bg_send_q;
OS_EVENT    *aud_avi_q;
OS_EVENT    *aud_right_q;

void        *AudioDacTaskQ_area[AUDIO_DAC_QUEUE_MAX];
void		*send_q[AUDIO_DAC_SENDQ_SIZE];
void		*send_bg_q[AUDIO_DAC_SENDQ_SIZE];
void		*avi_q[AUDIO_AVI_Q_SIZE];
void		*aud_right[AUDIO_RIGHT_Q_SIZE];

FP64		g_avi_synchroframe;
FP64		g_avi_synchrocnt;
INT32S		g_avi_datalength;

#elif (_OPERATING_SYSTEM == _OS_FREERTOS)

static xTimerHandle aud_ramp_timer;
extern INT16S   *pcm_out[MAX_DAC_BUFFERS];
extern INT32U    pcm_len[MAX_DAC_BUFFERS];
extern INT16S   *pcm_bg_out[MAX_DAC_BUFFERS];
extern INT32U    pcm_bg_len[MAX_DAC_BUFFERS];
extern xQueueHandle audio_wq;
extern xQueueHandle audio_bg_wq;

xQueueHandle hAudioDacTaskQ;
xQueueHandle aud_send_q;
xQueueHandle aud_bg_send_q;
xQueueHandle aud_right_q;
#endif
extern MSG_Q_ID AudioBGTaskQ;

INT32U last_send_idx;
ST_AUDIO_PLAY_TIME aud_time;

/* Proto types */
void audio_dac_task_init(void);
void audio_dac_task_entry(void const *p_arg);
#if (APP_AUDIO_BG_EN == 1) || ( APP_G_MIDI_DECODE_EN == 1) //(defined MCU_VERSION) && (MCU_VERSION < GPL327XX)
static void audio_spu_softch_start(void);
static void audio_spu_loopaddr_set(INT8U r_idx);
static void audio_spu_fiq_isr(void);
static void audio_dac_ramp_isr(xTimerHandle pxTimer);
#endif

void audio_dac_task_init(void)
{
    /* Create MsgQueue/MsgBox for TASK */
#if (_OPERATING_SYSTEM == _OS_UCOS2)
    hAudioDacTaskQ = OSQCreate(AudioDacTaskQ_area, AUDIO_DAC_QUEUE_MAX);
    aud_send_q = OSQCreate(send_q, AUDIO_DAC_SENDQ_SIZE);
    aud_bg_send_q = OSQCreate(send_bg_q, AUDIO_DAC_SENDQ_SIZE);
	aud_avi_q = OSQCreate(avi_q, AUDIO_AVI_Q_SIZE);
	aud_right_q = OSQCreate(aud_right, AUDIO_RIGHT_Q_SIZE);

	g_avi_synchroframe = 0;
	g_avi_synchrocnt = 0;
	g_avi_datalength = 4000;

#elif (_OPERATING_SYSTEM == _OS_FREERTOS)

    hAudioDacTaskQ = xQueueCreate(AUDIO_DAC_QUEUE_MAX, sizeof(INT32U));
    aud_send_q = xQueueCreate(AUDIO_DAC_SENDQ_SIZE, sizeof(INT32U));
    aud_bg_send_q = xQueueCreate(AUDIO_DAC_SENDQ_SIZE, sizeof(INT32U));
    aud_right_q = xQueueCreate(AUDIO_RIGHT_Q_SIZE, sizeof(INT32U));
#endif
}

void audio_dac_task_entry(void const *p_arg)
{
	INT8U			err = 0;
	INT8U           pause = 0;
	INT32S    	    audio_dac_msg;
	INT32U          r_idx = 0;
	INT32U          w_idx = 0;
#if (APP_AUDIO_BG_EN == 1) || ( APP_G_MIDI_DECODE_EN == 1) //
	INT32U          r_idx_bg = 0;
	INT32U          w_idx_bg = 0;
#endif
	INT32U          total_samples = 0;
	INT32U          pcmlen;
	INT32U          times;
    audio_dac_task_init();

    /* Task Star */
	while (1)
	{
	    /* Pend task message */
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
        audio_dac_msg = (INT32S) OSQPend(hAudioDacTaskQ, 0, &err);
        if(err != OS_NO_ERR) {
            continue;
        }
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        err = (INT8U) xQueueReceive(hAudioDacTaskQ, &audio_dac_msg, portMAX_DELAY);
        if(err != pdPASS) {
            DBG_PRINT("%x ",err);
        }
    #endif
        switch(audio_dac_msg) {
			case C_DMA_STATUS_DONE:
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
			    if (pause != 1) {
                    r_idx = (INT32U)OSQAccept(aud_send_q, &err);
				    if (err == OS_NO_ERR) {
					    dac_cha_dbf_set(pcm_out[r_idx], pcm_len[r_idx]);
					    last_send_idx = r_idx;
				    }
                }
                OSQPost(audio_wq, (void *)w_idx);
				g_avi_datalength -= (INT32S)pcm_len[w_idx];
				if(g_avi_datalength <= 0)
				{
					g_avi_synchroframe +=g_avi_synchrocnt;
					g_avi_datalength += 4000;
					OSQPost(aud_avi_q, NULL);
				}
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				if (pause != 1)
				{
					err = (INT8U) xQueueReceive(aud_send_q, &r_idx, 0);
					if (err == pdPASS) 
					{
						drv_l1_dac_cha_dbf_set(pcm_out[r_idx], pcm_len[r_idx]);
						last_send_idx = r_idx;
						pcmlen = pcm_len[r_idx];
						if (channel == 2)
						{
							pcmlen >>= 1;
						}
						total_samples += pcmlen;
						times = (total_samples*10)/(sample_rate/100);
						if ( times > aud_time.curr_play_time)
						{
							aud_time.curr_play_time = (INT32S) times;
							//DBG_PRINT("time=%d\r\n",aud_time.curr_play_time);
						}
						else
						{
							//DBG_PRINT("total_samples=%d,sample_rate=%d,times=%d\r\n",total_samples,sample_rate,times);
						}
				    }
                }
                xQueueSend(audio_wq, &w_idx, portMAX_DELAY);
            #endif

				w_idx = (w_idx+1) % MAX_DAC_BUFFERS;
				break;
			case MSG_AUD_DMA_DBF_START:
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				r_idx = (INT32U)OSQAccept (aud_send_q, &err);
                dac_cha_dbf_put(pcm_out[r_idx], pcm_len[r_idx], hAudioDacTaskQ);
				r_idx = (INT32U)OSQAccept (aud_send_q, &err);
				dac_cha_dbf_set(pcm_out[r_idx], pcm_len[r_idx]);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                err = (INT8U)xQueueReceive(aud_send_q, &r_idx, 10);
                if(err == pdPASS) {
                    drv_l1_dac_cha_dbf_put(pcm_out[r_idx], pcm_len[r_idx], hAudioDacTaskQ);
                }

                err = (INT8U)xQueueReceive(aud_send_q, &r_idx, 10);
                if(err == pdPASS) {
                    drv_l1_dac_cha_dbf_set(pcm_out[r_idx], pcm_len[r_idx]);
                }
            #endif
				pause = 0;
				DBG_PRINT("dac start\r\n");
				break;
			case MSG_AUD_DMA_DBF_RESTART:
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				r_idx = (INT32U)OSQAccept(aud_send_q, &err);
				if (err == OS_NO_ERR) {
					drv_l1_dac_cha_dbf_set(pcm_out[r_idx], pcm_len[r_idx]);
				}
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                err = (INT8U) xQueueReceive(aud_send_q, &r_idx, 0);
				if (err == pdPASS) {
					drv_l1_dac_cha_dbf_set(pcm_out[r_idx], pcm_len[r_idx]);
				}
            #endif
				DBG_PRINT("D..\r\n");
				break;
			case MSG_AUD_DMA_WIDX_CLEAR:
				w_idx = 0;
				break;
			case MSG_AUD_DMA_PAUSE:
			    pause = 1;
			    DBG_PRINT("dac pause\r\n");
			    break;
#if (APP_AUDIO_BG_EN == 1) || ( APP_G_MIDI_DECODE_EN == 1)
			case MSG_AUD_SPU_SOFTCH_START:
				audio_spu_softch_start();
				break;
			case MSG_AUD_SPU_LEFT_DONE:
				if ((SPU_GetChannelEnableStatus() & SPU_LEFT_CH_BIT) == 0) {
					break;
				}
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				r_idx_bg = (INT32U)OSQAccept(aud_bg_send_q, &err);
				if (err == OS_NO_ERR) {
					audio_spu_loopaddr_set(r_idx_bg);
				}
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                err = (INT8U)xQueueReceive(aud_bg_send_q, &r_idx_bg, 0);
				if (err == pdPASS) {
					audio_spu_loopaddr_set(r_idx_bg);
				}
            #endif
				else {
					DBG_PRINT("SPU underrun\r\n");
					SPU_StopChannel(SPU_LEFT_CH);
					SPU_StopChannel(SPU_RIGHT_CH);
					SPU_Disable_FIQ_Channel(SPU_LEFT_CH);
					SPU_Disable_FIQ_Channel(SPU_RIGHT_CH);
					msgQSend(AudioBGTaskQ, MSG_AUD_SPU_RESTART, NULL, 0, MSG_PRI_URGENT);
				}

            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(audio_bg_wq, (void *)w_idx_bg);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                xQueueSend(audio_bg_wq, &w_idx_bg, 1);
            #endif
				w_idx_bg = (w_idx_bg+1) % MAX_DAC_BUFFERS;
				break;
			case MSG_AUD_SPU_WIDX_CLEAR:
				w_idx_bg = 0;
				break;
			case MSG_AUD_SPU_WIDX_SET:
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				w_idx_bg = (INT32U)OSQAccept(aud_bg_send_q, &err);
				if (err == OS_NO_ERR) {
					OSQPostFront(aud_bg_send_q, (void *)w_idx_bg);
				}
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                err = (INT8U)xQueueReceive(aud_bg_send_q, &w_idx_bg, 0);
                if(err == pdPASS) {
                    xQueueSendToFront(aud_bg_send_q, &w_idx_bg, 1);
                }
            #endif
				else {
					w_idx_bg = 0;
				}
				break;

			case MSG_AUD_RAMP_DOWN_START:
				//sys_registe_timer_isr(audio_dac_ramp_isr);
				#if _OPERATING_SYSTEM == _OS_FREERTOS
                aud_ramp_timer = xTimerCreate("RAMP_TIMER", 10/portTICK_RATE_MS, pdTRUE, NULL, audio_dac_ramp_isr);
                if (aud_ramp_timer != NULL)
                    xTimerStart(aud_ramp_timer, 0);
                #endif
				break;
#endif
			default:
				break;
		}
	}
}

#if (APP_AUDIO_BG_EN == 1) || ( APP_G_MIDI_DECODE_EN == 1)
extern STAudioTaskPara Aud_BG_Para;

static void audio_spu_softch_start(void)
{
	INT8U   pitch,velocity,err;
	INT16S  *buff_addr;
	INT32U  l_addr_a;
	INT32U  r_addr_a;
	INT32U  l_addr_b;
	INT32U  r_addr_b;

	INT8U   EDD;
	INT8U   r_idx;
	INT32U  pcmlen;
	INT32U  phase;

	pitch = 60;
	velocity = Aud_BG_Para.volume;//127;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	r_idx = (INT32U)OSQAccept (aud_bg_send_q, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        err = (INT8U) xQueueReceive(aud_bg_send_q, &r_idx, 0);
#endif
	buff_addr = pcm_bg_out[r_idx];
	pcmlen = pcm_bg_len[r_idx];

	l_addr_a = (INT32U)buff_addr;

	if (channel == 2) {
		pcmlen >>= 1;
	}
	r_addr_a = (INT32U)(buff_addr + pcmlen + 1);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	r_idx = (INT32U)OSQAccept (aud_bg_send_q, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        err = (INT8U) xQueueReceive(aud_bg_send_q, &r_idx, 0);
#endif

	buff_addr = pcm_bg_out[r_idx];
	pcmlen = pcm_bg_len[r_idx];

	l_addr_b = (INT32U)buff_addr;

	if (channel == 2) {
		pcmlen >>= 1;
	}
	r_addr_b = (INT32U)(buff_addr + pcmlen + 1);


	SPU_PlayTone1(pitch, 0, velocity, SPU_LEFT_CH);
	SPU_PlayTone1(pitch, 127, velocity, SPU_RIGHT_CH);

	l_addr_a = l_addr_a / 2;
	SPU_SetStartAddress(l_addr_a, SPU_LEFT_CH);

	r_addr_a = r_addr_a / 2;
	SPU_SetStartAddress(r_addr_a, SPU_RIGHT_CH);
	//DBG_PRINT("r_idx = %d, l_addr = 0x%x, r_addr = 0x%x\r\n",0,l_addr_a,r_addr_a);

	l_addr_b = l_addr_b / 2;
	SPU_SetLoopAddress(l_addr_b, SPU_LEFT_CH);

	r_addr_b = r_addr_b / 2;
	SPU_SetLoopAddress(r_addr_b, SPU_RIGHT_CH);
	//DBG_PRINT("r_idx = %d, l_addr = 0x%x, r_addr = 0x%x\r\n",1,l_addr_b,r_addr_b);

	phase = ((sample_rate << 13)/281250) << 6;

	SPU_SetPhase(phase, SPU_LEFT_CH);
	SPU_SetPhase(phase, SPU_RIGHT_CH);

	SPU_EnvelopeManualMode(SPU_LEFT_CH);
	SPU_EnvelopeManualMode(SPU_RIGHT_CH);

	EDD = 0x7F;
	SPU_SetEnvelopeData(EDD, SPU_LEFT_CH);
	SPU_SetEnvelopeData(EDD, SPU_RIGHT_CH);

	SPU_SetToneColorMode(0x02, SPU_LEFT_CH);
	SPU_SetToneColorMode(0x02, SPU_RIGHT_CH);

	SPU_SingleChVolumeSelect(0x03);
	SPU_Set_MainVolume(0x7F);

#if	SPU_COMPRESS_EN == 1
	SPU_CompressorON();
	SPU_EnableCompZeroCrossing();
	SPU_SelectPeakMode();
	//SPU_SelectRMS_Mode();

	SPU_SingleChVolumeSelect(0x03);
	SPU_SetCompressThreshold(0x01);
	SPU_SetCompressorRatio(7);
	SPU_SetReleaseTimeScale(0);
	SPU_SetAttackTimeScale(0);
#endif

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQFlush(aud_right_q);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	xQueueReset(aud_right_q);
#endif

	SPU_AttachISR(C_SPU_FIQ, audio_spu_fiq_isr);
	SPU_Enable_Channel(SPU_LEFT_CH);
	SPU_Enable_Channel(SPU_RIGHT_CH);

	SPU_Clear_Ch_StopFlag(SPU_LEFT_CH);
	SPU_Clear_Ch_StopFlag(SPU_RIGHT_CH);

	SPU_Enable_FIQ_Channel(SPU_LEFT_CH);
	SPU_Enable_FIQ_Channel(SPU_RIGHT_CH);
}

static void audio_spu_loopaddr_set(INT8U r_idx)
{
	INT16S *buff_addr;
	INT32U pcmlen;
	INT32U l_addr,r_addr;
	INT8U  err;
	INT32U  loopaddr;
	INT32S  i;

	buff_addr = pcm_bg_out[r_idx];
	pcmlen = pcm_bg_len[r_idx];

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPend(aud_right_q, 0, &err); /* wait right channel done */
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        err = (INT8U) xQueueReceive(aud_right_q, &l_addr, portMAX_DELAY);
#endif

	l_addr = (INT32U)buff_addr;

	if (channel == 2) {
		pcmlen >>= 1;
	}
	r_addr = (INT32U)(buff_addr + pcmlen +1);

	l_addr = l_addr / 2;
	SPU_SetLoopAddress(l_addr, SPU_LEFT_CH);

	for (i=0;i<10;i++) {
		SPU_GetLoopAddress(&loopaddr,SPU_LEFT_CH);
		if (loopaddr != l_addr) { /* set agian */
			//DBG_PRINT("set left agian,%d\r\n",i);
			SPU_SetLoopAddress(l_addr, SPU_LEFT_CH);
        }
		else {
			break;
	    }
	}

	r_addr = r_addr/2;
	SPU_SetLoopAddress(r_addr, SPU_RIGHT_CH);

	for (i=0;i<10;i++) {
		SPU_GetLoopAddress(&loopaddr,SPU_RIGHT_CH);
		if (loopaddr != r_addr) { /* set agian */
			//DBG_PRINT("set right agian,%d\r\n",i);
			SPU_SetLoopAddress(r_addr, SPU_RIGHT_CH);
		}
		else {
			break;
		}
	 }

	//if (cnt < 4) {
	//	DBG_PRINT("r_idx = %d, l_addr = 0x%x, r_addr = 0x%x\r\n",r_idx,l_addr,r_addr);
	//	cnt++;
	//}

}

static void audio_spu_fiq_isr(void)
{
	INT32U status;

	status = SPU_Get_FIQ_Status();
	if (status & SPU_LEFT_CH_BIT) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
        SPU_Clear_FIQ_Status(SPU_LEFT_CH_BIT);
		OSQPost(hAudioDacTaskQ, (void *)MSG_AUD_SPU_LEFT_DONE);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        INT32U message = MSG_AUD_SPU_LEFT_DONE;

        SPU_Clear_FIQ_Status(SPU_LEFT_CH_BIT);
        xQueueSendFromISR(hAudioDacTaskQ, &message, NULL);
    #endif
    }

	if (status & SPU_RIGHT_CH_BIT) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		SPU_Clear_FIQ_Status(SPU_RIGHT_CH_BIT);
		OSQPost(aud_right_q, (void *)MSG_AUD_SPU_RIGHT_DONE);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        INT32U message = MSG_AUD_SPU_RIGHT_DONE;

        SPU_Clear_FIQ_Status(SPU_RIGHT_CH_BIT);
        xQueueSendFromISR(aud_right_q, &message, NULL);
    #endif
	}
}

static void audio_dac_ramp_isr(xTimerHandle pxTimer)
{
	INT8S edd1 = 0;
	INT8S edd2 = 0;

	edd1 = SPU_ReadEnvelopeData(SPU_LEFT_CH);
	edd1 -= edd_step;
	if (edd1 < 0) {
		edd1 = 0;
	}

	SPU_SetEnvelopeData(edd1, SPU_LEFT_CH);

	edd2 = SPU_ReadEnvelopeData(SPU_RIGHT_CH);
	edd2 -= edd_step;
	if (edd2 < 0) {
		edd2 = 0;
	}

	SPU_SetEnvelopeData(edd2, SPU_RIGHT_CH);

	if (edd1 == 0 && edd2 == 0) {
		//sys_release_timer_isr(audio_dac_ramp_isr);
    #if _OPERATING_SYSTEM == _OS_FREERTOS
        if (aud_ramp_timer != NULL)
            xTimerStop(aud_ramp_timer, 0);
        xTimerDelete(aud_ramp_timer, 0);
    #endif
		DBG_PRINT("edd to 0\r\n");
	}

}
#endif
