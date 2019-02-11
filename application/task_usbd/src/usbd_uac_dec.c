/******************************************************
* usbd_uac_dec.c
*
* Purpose: USBD UAC decode
*
* Author: Eugene Hsu
*
* Date: 2013/03/13
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/
#include "project.h"

#include "drv_l1_sfr.h"
#include "drv_l1_dma.h"
#include "drv_l1_dac.h"
#include "drv_l1_timer.h"
#include "drv_l1_usbd.h"
#include "drv_l1_i2s_rx.h"
#include "drv_l2_usbd.h"
#include "usbd_uac.h"

#define EPC_DMA_MODE	0
#define EPC_PIO_MODE	1
#define EPC_MODE		EPC_DMA_MODE

#if (USBD_UAC_MICROPHONE_SAMPLE_RATE == 44100)
#define AUDIO_ENCODE_SIZE USBD_UAC_MIC_PKT_SIZE*10+2   //88*9+90
#elif (USBD_UAC_MICROPHONE_SAMPLE_RATE == 22050)
#define AUDIO_ENCODE_SIZE USBD_UAC_MIC_PKT_SIZE*20+2   //44*9+45
#else
#define AUDIO_ENCODE_SIZE USBD_UAC_MIC_PKT_SIZE*10
#endif
/******************************************************
    Definition and variable declaration
*******************************************************/
#pragma data_alignment = 4

INT16S *mic_data_buffer[MAX_BUFFER_NUM];
static INT32U mic_data_size = 0;
static INT32U mic_data_end = 0;
static INT8U* mic_data_ptr = NULL;
INT32S interval = 0;
uac_aud_dec_t usb_uac_aud_ctlblk;
INT8S  zerobuf[USBD_UAC_SPEAKER_PKT_SIZE] = {0};
/**********************************************
*	Extern variables and functions
**********************************************/
extern INT32U drv_l1_usbd_get_alt_interface(void);
extern void drv_l1_usbd_uac_reset(void);
/*****************************************************
    USBD UAC functions
******************************************************/
INT8U uac_get_gain_level(void)
{
	if(usb_uac_aud_ctlblk.uac_mute)
		return usb_uac_aud_ctlblk.uac_level;
	else
		return 0;
}

void hid_set_idle_cbk(INT32U id)
{
	DBG_PRINT("HID cbk, report id = %d\r\n", id);
	usb_uac_aud_ctlblk.hidstatus = 1;
	usb_uac_aud_ctlblk.hidbuf[0] = 0;

	if(usb_uac_aud_ctlblk.hidstatus)
	{
		//drv_l1_usbd_send_int_in(usb_uac_aud_ctlblk.hidbuf, UACHIDBUFLEN);
	}
}

void uac_set_gain_cbk(INT32U cs, INT32S gain)
{
	INT32U msg;
	osStatus status;

    if(cs==2){
        //usb_uac_aud_ctlblk.uac_level = ((INT32S)gain + USBD_DAC_MIN_GAIN_OFFSET) >> USBD_DAC_AMP_LEVEL_INTERVAL;
        usb_uac_aud_ctlblk.uac_level = UAC_GAIN_TO_DAC_VOL(gain);

        if(usb_uac_aud_ctlblk.uac_level >= USBD_DAC_AMP_LEVEL)
            usb_uac_aud_ctlblk.uac_level = USBD_DAC_AMP_LEVEL;
    }
    else if(cs==1){
		if(gain & 0xFF)
			usb_uac_aud_ctlblk.uac_mute = 1;
		else
			usb_uac_aud_ctlblk.uac_mute = 0;
    }
	//DBG_PRINT("gain = 0x%x, level = %d, cs = %d!!\r\n", gain, usb_uac_aud_ctlblk.uac_level, cs);
	#if 0
	if(cs == 1)
	{
		if(gain==0x8101)
		//if(usb_uac_aud_ctlblk.uac_level==0)
			usb_uac_aud_ctlblk.uac_mute = 1;
		else
			usb_uac_aud_ctlblk.uac_mute = 0;
	}
    #endif
	msg = MSG_UAC_SET_DAC_VOLUME;
	status = osMessagePut(usb_uac_aud_ctlblk.uac_dec_aud_q, (INT32U)&msg, osWaitForever);
	if(status != osOK)
	{
		DBG_PRINT("uac_set_gain_cbk: send msg failed!!!\r\n");
	}
}
/*******************************************************
Sample control for decode read/write ring buffer access
----------------------------------------
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
----------------------------------------
                              ^
                            write ptr
          ^
 read ptr start

 (1) if write index - read index = 7
 	sample rate = 48000 + 150;
 (2) if write index - read index = 5
 	sample rate = 48000 - 150;
 (3) if write index - read index = 3
 	sample rate = 48000 - 300;

*******************************************************/
void uac_aud_dec_check_buf_rate(void)
{
	usb_uac_aud_ctlblk.spkinterval = usb_uac_aud_ctlblk.write_index - usb_uac_aud_ctlblk.read_index;
	if(usb_uac_aud_ctlblk.read_index > usb_uac_aud_ctlblk.write_index)
	{
		usb_uac_aud_ctlblk.spkinterval += USBD_UAC_SPEAKER_RING_BUFF_LEN;
	}

	if(usb_uac_aud_ctlblk.spkinterval >= USBD_UAC_DEC_BUF_MAX_INTERVAL)
	{
		drv_l1_dac_sample_rate_set(usb_uac_aud_ctlblk.speaker_sample_rate + USBD_UAC_SAMPLE_RATE_MIN_OFFSET);
		//DBG_PRINT("A%d %d%d\r\n", interval, usb_uac_aud_ctlblk.write_index, usb_uac_aud_ctlblk.read_index);
	}
	else if(usb_uac_aud_ctlblk.spkinterval == USBD_UAC_DEC_BUF_MID_INTERVAL)
	{
		drv_l1_dac_sample_rate_set(usb_uac_aud_ctlblk.speaker_sample_rate - USBD_UAC_SAMPLE_RATE_MID_OFFSET);
		//DBG_PRINT("D%d %d%d\r\n", interval, usb_uac_aud_ctlblk.write_index, usb_uac_aud_ctlblk.read_index);
	}
	else if(usb_uac_aud_ctlblk.spkinterval == USBD_UAC_DEC_BUF_MIN_INTERVAL)
	{
		drv_l1_dac_sample_rate_set(usb_uac_aud_ctlblk.speaker_sample_rate - USBD_UAC_SAMPLE_RATE_MAX_OFFSET);
		//DBG_PRINT("N%d %d%d\r\n", interval, usb_uac_aud_ctlblk.write_index, usb_uac_aud_ctlblk.read_index);
	}
	else if(usb_uac_aud_ctlblk.spkinterval == 0)
	{
		//drv_l1_dac_sample_rate_set(usb_uac_aud_ctlblk.speaker_sample_rate - 300);
	}
}

void uac_aud_dec_task_entry(void const *para)
{
    INT32U  msg_id;
    INT32U 	addr;
	uac_aud_dec_t *audpara = &usb_uac_aud_ctlblk;
	INT32U	outcnt;
	osEvent result;

#if (EPC_MODE == EPC_DMA_MODE)
	DBG_PRINT("Enter uac_aud_dec_task_entry in DMA mode...\r\n");
#elif (EPC_MODE == EPC_PIO_MODE)
	DBG_PRINT("Enter uac_aud_dec_task_entry in PIO mode...\r\n");
#endif
	DBG_PRINT("Speaker sample rate %d, %d bits, %d valid byte(s), %d bytes per packet\r\n", USBD_UAC_SPEAKER_SAMPLE_RATE, USBD_UAC_SPK_SUBFRAME_VALID_BIT, USBD_UAC_SPK_SUBFRAME_BYTE, USBD_UAC_SPEAKER_PKT_SIZE);
	usb_uac_aud_ctlblk.uac_mute = 0;
	//usb_uac_aud_ctlblk.uac_mute = 10;

	while(1)
    {
		result = osMessageGet(audpara->uac_dec_aud_q, osWaitForever);
		msg_id = result.value.v;
    	if(result.status != osEventMessage)
		{
			continue;
		}
		//if(msg_id!= C_DMA_STATUS_DONE && msg_id!= MSG_UAC_REC_SPK_DATA_DONE)
        //    DBG_PRINT("%x\r\n",msg_id);
    	switch(msg_id)
    	{
	    	case C_DMA_STATUS_DONE:
	    		/* Check speaker read/write pointer to adjust DAC sample rate */
	    		uac_aud_dec_check_buf_rate();

	    		if(usb_uac_aud_ctlblk.spkinterval == 0)
	    			usb_uac_aud_ctlblk.spkzero = 1;
	    		else if(usb_uac_aud_ctlblk.spkinterval >= 2)
	    			usb_uac_aud_ctlblk.spkzero = 0;

	    		if(!usb_uac_aud_ctlblk.spkzero)
	    		{
	    			usb_uac_aud_ctlblk.read_index++;
	    			if(usb_uac_aud_ctlblk.read_index >= USBD_UAC_SPEAKER_RING_BUFF_LEN)
	    			{
	    				usb_uac_aud_ctlblk.read_index = 0;
	    			}
	    			addr = (INT32U)(usb_uac_aud_ctlblk.ppcmbuf + (usb_uac_aud_ctlblk.pcmframesize * usb_uac_aud_ctlblk.read_index));
					drv_l1_dac_cha_dbf_set((INT16S*)addr, (usb_uac_aud_ctlblk.pcmframesize >> 1));
					//DBG_PRINT("r[%d]w[%d]\r\n", usb_uac_aud_ctlblk.read_index, usb_uac_aud_ctlblk.write_index);
	    		}
	    		else
	    		{
					drv_l1_dac_cha_dbf_set((INT16S*)&zerobuf[0], (usb_uac_aud_ctlblk.pcmframesize >> 1));
					//DBG_PRINT("L:r%d w%d\r\n", usb_uac_aud_ctlblk.read_index, usb_uac_aud_ctlblk.write_index);
	    		}
	    		break;

			case MSG_UAC_REC_SPK_DATA_DONE:
	    		/* DMA mode for audio ISO OUT */
	     		usb_uac_aud_ctlblk.write_index++;
	    		if(usb_uac_aud_ctlblk.write_index == USBD_UAC_SPEAKER_RING_BUFF_LEN)
	    		{
	    			usb_uac_aud_ctlblk.write_index = 0;
	    		}

	     		if(usb_uac_aud_ctlblk.altinterface1)
	     		{
					addr = (INT32U)(usb_uac_aud_ctlblk.ppcmbuf + (usb_uac_aud_ctlblk.pcmframesize * usb_uac_aud_ctlblk.write_index));
					drv_l1_usbd_epc_dma_iso_out((void*)addr, USBD_UAC_SPEAKER_PKT_SIZE);
					if(usb_uac_aud_ctlblk.outsendstart == UAC_ISO_OUT_IDLE && usb_uac_aud_ctlblk.write_index == USBD_UAC_DEC_BUF_MAX_INTERVAL)
					{
						INT32U msg = MSG_AUDIO_SPK_START;
						osStatus status;

						status = osMessagePut(usb_uac_aud_ctlblk.uac_dec_aud_q, (INT32U)&msg, osWaitForever);
						if(status != osOK)
						{
							DBG_PRINT("Send event to uac_dec_aud_q failed\r\n");
						}
		    			usb_uac_aud_ctlblk.outsendstart = UAC_ISO_OUT_SEND_START;
					}
				}
	    		break;

	    	case MSG_UAC_REC_SPK_PKT_DONE:
	    		/* PIO mode for audio ISO OUT */
	    		outcnt = drv_l1_usbd_get_epc_write_cnt();
	    		addr = (INT32U)(usb_uac_aud_ctlblk.ppcmbuf + (usb_uac_aud_ctlblk.pcmframesize * usb_uac_aud_ctlblk.write_index));
	    		drv_l1_usbd_get_epc_data((void*)addr, outcnt);

	    		if(usb_uac_aud_ctlblk.altinterface1)
	     		{
					drv_l1_usbd_enable_epc_iso_out();
					usb_uac_aud_ctlblk.write_index++;
	    			if(usb_uac_aud_ctlblk.write_index == USBD_UAC_SPEAKER_RING_BUFF_LEN)
	    			{
	    				usb_uac_aud_ctlblk.write_index = 0;
	    			}
					if(usb_uac_aud_ctlblk.outsendstart == UAC_ISO_OUT_IDLE && usb_uac_aud_ctlblk.write_index == USBD_UAC_DEC_BUF_MAX_INTERVAL)
					{
		    			INT32U msg = MSG_AUDIO_SPK_START;
		    			osStatus status;

						status = osMessagePut(usb_uac_aud_ctlblk.uac_dec_aud_q, (INT32U)&msg, osWaitForever);
						if(status != osOK)
						{
							DBG_PRINT("Send event to uac_dec_aud_q failed\r\n");
						}
		    			usb_uac_aud_ctlblk.outsendstart = UAC_ISO_OUT_SEND_START;
					}
				}
	    		//DBG_PRINT("cnt %d\r\n", outcnt);
	    		break;

	    	case MSG_AUDIO_SPK_START:
	    		usb_uac_aud_ctlblk.read_index = 1;
	    		drv_l1_dac_cha_dbf_put((INT16S*)(usb_uac_aud_ctlblk.ppcmbuf), (usb_uac_aud_ctlblk.pcmframesize >> 1), usb_uac_aud_ctlblk.uac_dec_aud_q);
				drv_l1_dac_cha_dbf_set((INT16S*)(usb_uac_aud_ctlblk.ppcmbuf + usb_uac_aud_ctlblk.pcmframesize), (usb_uac_aud_ctlblk.pcmframesize >> 1));
				drv_l1_dac_sample_rate_set(usb_uac_aud_ctlblk.speaker_sample_rate);
	    		break;

	    	case MSG_AUDIO_SPK_STOP:
	    		DBG_PRINT("SPK stop\r\n");
	    		drv_l1_dac_timer_stop();

	    		//while(drv_l1_dac_dma_status_get() || drv_l1_dac_dbf_status_get())
	    		//{
					//OSTimeDly(1);
				//}
				drv_l1_dac_dbf_free();
				usb_uac_aud_ctlblk.outsendstart = UAC_ISO_OUT_STOP;

				drv_l1_disable_epc_iso_out_nack_event();
				drv_l1_disable_epc_iso_out_fail_event();
	    		break;

			case MSG_AUDIO_PAUSE:
	    		break;

	    	case MSG_AUDIO_RESUME:
	    		break;

	    	case MSG_AUDIO_EXIT:
	    		break;

	    	case MSG_UAC_SPK_START:
	    		DBG_PRINT("SPK start\r\n");
	    		usb_uac_aud_ctlblk.read_index = 0;
				usb_uac_aud_ctlblk.write_index = 0;
				usb_uac_aud_ctlblk.spkzero = 0;
#if (EPC_MODE == EPC_DMA_MODE)
				drv_l1_usbd_epc_dma_iso_out((void*)usb_uac_aud_ctlblk.ppcmbuf, USBD_UAC_SPEAKER_PKT_SIZE);
#elif(EPC_MODE == EPC_PIO_MODE)
				drv_l1_usbd_enable_epc_iso_out();
#endif
				usb_uac_aud_ctlblk.outsendstart = UAC_ISO_OUT_IDLE;
	    		break;

	    	/* Following messages are from USB deivce */
	    	case MSG_UAC_SET_DAC_VOLUME:
	    		//DBG_PRINT("MSG_UAC_SET_DAC_VOLUME, mute %d, level %d\r\n", usb_uac_aud_ctlblk.uac_mute, usb_uac_aud_ctlblk.uac_level);
#if 1
	    		if(usb_uac_aud_ctlblk.uac_mute)
	    			drv_l1_dac_pga_set(0);
	    		else
	    			drv_l1_dac_pga_set(usb_uac_aud_ctlblk.uac_level);
#endif
	    		break;
	    	default:
	    		DBG_PRINT("Unknow Message\r\n");
	    		break;
    	}
	}
}

void uac_reset_ctl(void)
{
	usb_uac_aud_ctlblk.read_index = 0;
	usb_uac_aud_ctlblk.write_index = 0;
	usb_uac_aud_ctlblk.mic_read_index = 0;
	usb_uac_aud_ctlblk.mic_write_index = 0;
	usb_uac_aud_ctlblk.outsendstart = UAC_ISO_OUT_IDLE;
	usb_uac_aud_ctlblk.altinterface1 = 0;
	usb_uac_aud_ctlblk.altinterface2 = 0;
}

void uac_dec_stream_control(void)
{
	INT8U prenum;
	INT32U msg;
	osStatus status;


	prenum = usb_uac_aud_ctlblk.altinterface1;
    usb_uac_aud_ctlblk.altinterface1 = drv_l1_usbd_get_altif_num(INTERFACE_1);
	if(prenum == usb_uac_aud_ctlblk.altinterface1)
		return;

	switch(usb_uac_aud_ctlblk.altinterface1)
	{
		case 0:
            /* Send MSG_AUDIO_SPK_STOP to decode task */
            msg = MSG_AUDIO_SPK_STOP;
            status = osMessagePut(usb_uac_aud_ctlblk.uac_dec_aud_q, (INT32U)&msg, osWaitForever);
			if(status != osOK)
			{
				DBG_PRINT("uac_dec_stream_control to uac_dec_aud_q failed\r\n");
			}
			break;

		case 1:
            /* Enable ISO out */
            msg = MSG_UAC_SPK_START;
            status = osMessagePut(usb_uac_aud_ctlblk.uac_dec_aud_q, (INT32U)&msg, osWaitForever);
			if(status != osOK)
			{
				DBG_PRINT("uac_dec_stream_control to uac_dec_aud_q failed\r\n");
			}

			drv_l1_enable_epc_iso_out_nack_event();
			drv_l1_enable_epc_iso_out_fail_event();
			break;

		default:
			/* nothing to do and return */
			return;
	}
}

#define CLEAR(ptr, cblen)		gp_memset((INT8S *)ptr, 0x00, cblen)
#define APPL_ENABLE				(R_SYSTEM_CKGEN_CTRL |= 0x10)
static DMA_STRUCT i2s_rx_dma_dbf;


INT32S apll_fine_tune(INT32S apll_ftune_lv)
{
    INT32U temp;

    if(apll_ftune_lv>63)
        apll_ftune_lv = 63;
    else if(apll_ftune_lv < 0)
        apll_ftune_lv = 0;
    else{
        temp = R_SYSTEM_APLL_FREQ;
        temp &= ~0x3F;
        temp |= (INT32U)apll_ftune_lv;
        R_SYSTEM_APLL_FREQ = temp;

        return apll_ftune_lv;
    }
}

void uac_aud_enc_task_entry(void const *para)
{
    INT32U  msg_id;
	uac_aud_dec_t *audpara = &usb_uac_aud_ctlblk;
	osEvent result;
    INT8U uac_isoin_widx=0;
    INT8U uac_isoin_ridx = 0;
    INT8U i;
    INT8U pkt_cnt=0;
    INT8U uac_mic_start=1;
    INT32S apll_fine_tune_lv;
    INT32S diff;

    mic_data_size = sizeof(INT16S)*AUDIO_ENCODE_SIZE;
    DBG_PRINT("mic_data_size=%d\r\n",mic_data_size);

    for(i=0;i<MAX_BUFFER_NUM;i++)
    {
        mic_data_buffer[i] = (INT16S *)gp_malloc_align(sizeof(INT16S)*AUDIO_ENCODE_SIZE,4);
        gp_memset((INT8S *)mic_data_buffer[i], 0, sizeof(INT16S)*AUDIO_ENCODE_SIZE);
        DBG_PRINT("PCM data_buffer[%d] = 0x%x\r\n",i,mic_data_buffer[i]);
    }
	//DBG_PRINT("Enter uac_aud_enc_task_entry, size %d, data 0x%x, end 0x%x\r\n", mic_data_size, mic_data_ptr, mic_data_end);
	while(1)
    {
       	result = osMessageGet(audpara->uac_enc_aud_q, osWaitForever);
		msg_id = result.value.v;
    	if(result.status != osEventMessage){
			continue;
		}

    	switch(msg_id){
	    	case C_DMA_STATUS_DONE:
                uac_isoin_widx++;
                if(uac_isoin_widx>=MAX_BUFFER_NUM)
                {
                    uac_isoin_widx = 0;
                }
                drv_l1_i2s_rx_adc_dbf_set((INT16S*)mic_data_buffer[uac_isoin_widx], AUDIO_ENCODE_SIZE);

                if((uac_mic_start == 1)&&(uac_isoin_widx==3)){
                    uac_mic_start = 0;

                    drv_l1_usbd_iso_ep7_in(mic_data_ptr, USBD_UAC_MIC_PKT_SIZE);
                    mic_data_ptr += USBD_UAC_MIC_PKT_SIZE;
                    #if (USBD_UAC_MICROPHONE_SAMPLE_RATE == 44100)||(USBD_UAC_MICROPHONE_SAMPLE_RATE == 22050)
                    pkt_cnt++;
                    #endif
                }
	    		break;

			case MSG_UAC_SEND_MIC_DATA_DONE:
                //apll freq control
                diff = uac_isoin_widx - uac_isoin_ridx;
                if(diff<0)
                    diff+=MAX_BUFFER_NUM;

                if(diff<3){
                    apll_fine_tune_lv += 1;
                    apll_fine_tune_lv = apll_fine_tune(apll_fine_tune_lv);

                }
                else if(diff>3){
                    apll_fine_tune_lv -= 1;
                    apll_fine_tune_lv = apll_fine_tune(apll_fine_tune_lv);

                }
            #if (USBD_UAC_MICROPHONE_SAMPLE_RATE == 44100)
				if(pkt_cnt<9){
                    drv_l1_usbd_iso_ep7_in(mic_data_ptr, USBD_UAC_MIC_PKT_SIZE);
				mic_data_ptr += USBD_UAC_MIC_PKT_SIZE;
                    if((INT32U)mic_data_ptr >= mic_data_end){
                        uac_isoin_ridx++;
                        if(uac_isoin_ridx >=MAX_BUFFER_NUM)
                            uac_isoin_ridx = 0;
                        mic_data_ptr = (INT8U*)mic_data_buffer[uac_isoin_ridx];
                        mic_data_end = (INT32U)(mic_data_ptr + mic_data_size);
                    }
                    pkt_cnt++;
				}
                else{
                    drv_l1_usbd_iso_ep7_in(mic_data_ptr, (USBD_UAC_MIC_PKT_SIZE+2));

                    mic_data_ptr += (USBD_UAC_MIC_PKT_SIZE+2);
                    if((INT32U)mic_data_ptr >= mic_data_end){
                        uac_isoin_ridx++;
                        if(uac_isoin_ridx >=MAX_BUFFER_NUM)
                            uac_isoin_ridx = 0;
                        mic_data_ptr = (INT8U*)mic_data_buffer[uac_isoin_ridx];
                        mic_data_end = (INT32U)(mic_data_ptr + mic_data_size);
                    }

                    pkt_cnt=0;
                }
            #elif (USBD_UAC_MICROPHONE_SAMPLE_RATE == 22050)
				if(pkt_cnt<9){
                    drv_l1_usbd_iso_ep7_in(mic_data_ptr, USBD_UAC_MIC_PKT_SIZE);

                    mic_data_ptr += USBD_UAC_MIC_PKT_SIZE;  //get next data ptr
                    if((INT32U)mic_data_ptr >= mic_data_end){
                        uac_isoin_ridx++;
                        if(uac_isoin_ridx >=MAX_BUFFER_NUM)
                            uac_isoin_ridx = 0;
                        mic_data_ptr = (INT8U*)mic_data_buffer[uac_isoin_ridx];
                        mic_data_end = (INT32U)(mic_data_ptr + mic_data_size);
                    }

                    pkt_cnt++;
				}
                else{
                    drv_l1_usbd_iso_ep7_in(mic_data_ptr, (USBD_UAC_MIC_PKT_SIZE+1));

                    mic_data_ptr += (USBD_UAC_MIC_PKT_SIZE+1);
				if((INT32U)mic_data_ptr >= mic_data_end)
				{
                        uac_isoin_ridx++;
                        if(uac_isoin_ridx >=MAX_BUFFER_NUM)
                            uac_isoin_ridx = 0;
                        mic_data_ptr = (INT8U*)mic_data_buffer[uac_isoin_ridx];
                        mic_data_end = (INT32U)(mic_data_ptr + mic_data_size);
                    }
                    pkt_cnt=0;
				}
            #else
                drv_l1_usbd_iso_ep7_in(mic_data_ptr, USBD_UAC_MIC_PKT_SIZE);
                mic_data_ptr += USBD_UAC_MIC_PKT_SIZE;
                if((INT32U)mic_data_ptr >= mic_data_end){
                    uac_isoin_ridx++;
                    if(uac_isoin_ridx >=MAX_BUFFER_NUM)
                        uac_isoin_ridx = 0;
                    mic_data_ptr = (INT8U*)mic_data_buffer[uac_isoin_ridx];
                    mic_data_end = (INT32U)(mic_data_ptr + mic_data_size);
                }


            #endif
				break;

	    	case MSG_AUDIO_MIC_START:
	    		DBG_PRINT("MIC start\r\n");
	    		//configure adc double buffer
	    		uac_isoin_widx = 0;
	    		uac_isoin_ridx = 0;
	    		pkt_cnt = 0;
                apll_fine_tune_lv = 0x25;

                drv_l1_i2s_rx_adc_dbf_put((INT16S*)mic_data_buffer[uac_isoin_widx], AUDIO_ENCODE_SIZE, audpara->uac_enc_aud_q);
                uac_isoin_widx++;
                drv_l1_i2s_rx_adc_dbf_set((INT16S*)mic_data_buffer[uac_isoin_widx], AUDIO_ENCODE_SIZE);
                //uac_isoin_widx++;
                drv_l1_i2s_rx_adc_set_input_path(MIC_INPUT);//set MIC_INPUT
                drv_l1_adc_high_pass(0);
                drv_l1_i2s_rx_adc_init();
                drv_l1_i2s_rx_adc_sample_rate_set(USBD_UAC_MICROPHONE_SAMPLE_RATE);
                apll_fine_tune(apll_fine_tune_lv);
                drv_l1_i2s_rx_adc_mono_ch_set();
                //dagc
                drv_l1_adc_dagc_mode(0,0);
                drv_l1_adc_dagc_scale(0,0);
                drv_l1_adc_dagc_att_rel_time(1,0xf8);
                drv_l1_adc_dagc_threshold(0x20);
                drv_l1_adc_dagc_enable(1);
                drv_l1_i2s_rx_adc_start();

				mic_data_ptr = (INT8U*)mic_data_buffer[0];
				mic_data_end = (INT32U)(mic_data_ptr + mic_data_size);
				uac_mic_start = 1;

	    		break;

	    	case MSG_AUDIO_MIC_STOP:
                drv_l1_i2s_rx_adc_stop();
                drv_l1_i2s_rx_adc_dbf_free();
	    		DBG_PRINT("MIC stop\r\n");
	    		break;

	    	case MSG_AUDIO_PAUSE:
	    		break;

	    	case MSG_AUDIO_RESUME:
	    		break;

	    	case MSG_AUDIO_EXIT:
	    		break;

	    	default:
	    		DBG_PRINT("Unknow Message\r\n");
	    		break;
    	}
	}
}

void uac_enc_stream_control(void)
{
	INT8U prenum;
	INT32U msg;
	osStatus status;

	prenum = usb_uac_aud_ctlblk.altinterface2;
    usb_uac_aud_ctlblk.altinterface2 = drv_l1_usbd_get_altif_num(INTERFACE_2);

	if(prenum == usb_uac_aud_ctlblk.altinterface2)
		return;

	switch(usb_uac_aud_ctlblk.altinterface2)
	{
		case 0:
			/* Send MSG_AUDIO_MIC_STOP to mic task */
			DBG_PRINT("Send MSG_AUDIO_MIC_STOP to mic task\r\n");
			msg = MSG_AUDIO_MIC_STOP;
			status = osMessagePut(usb_uac_aud_ctlblk.uac_enc_aud_q, (INT32U)&msg, osWaitForever);
			if(status != osOK)
			{
				DBG_PRINT("uac_enc_stream_control uac_enc_aud_q failed\r\n");
			}
			break;

		case 5:
            /* Send MSG_AUDIO_MIC_START to mic task */
            DBG_PRINT("Send MSG_AUDIO_MIC_START to mic task\r\n");
            msg = MSG_AUDIO_MIC_START;
			status = osMessagePut(usb_uac_aud_ctlblk.uac_enc_aud_q, (INT32U)&msg, osWaitForever);
			if(status != osOK)
			{
				DBG_PRINT("uac_enc_stream_control uac_enc_aud_q failed\r\n");
			}
			break;

		default:
			/* nothing to do */
			DBG_PRINT("nothing to do\r\n");
			break;
	}
}

/******************************************************
*       Interrupt Transfer
*       1. usbd_state_uac_int_in()
*
*       Note. These are used for handling ISO transfer
*******************************************************/
void usbd_state_uac_int_in(INT32U event)
{
    switch(event)
    {
    	case USBD_INT_IN_PACKET_CLEAR_EVENT:
    		if(usb_uac_aud_ctlblk.hidstatus)
    		{
	    		/* Send interrupt IN data */
	    	}
    		break;

        default:
            break;
    }
}

/******************************************************
*       ISO Transfer
*       1. usbd_state_uac_iso_in()
*		2. usbd_state_uac_iso_out()
*
*       Note. These are used for handling ISO transfer
*			event from driver layer 1.
*******************************************************/
void usbd_state_uac_iso_in(INT32U event)
{
	osStatus status;
	INT32U msg;

    switch(event)
    {
    	case USBD_ISO_IN_EP7_PKT_CLEAR_EVENT:
    		drv_l1_usbd_disable_iso_ep7_in();

			msg = MSG_UAC_SEND_MIC_DATA_DONE;
			status = osMessagePut(usb_uac_aud_ctlblk.uac_enc_aud_q, (INT32U)&msg, osWaitForever);
			if(status != osOK)
			{
				DBG_PRINT("Send MSG_UAC_SEND_MIC_DATA_DONE faled!!!\r\n");
			}
    		break;

    	case USBD_ISO_VIDEO_IN_PKT_CLEAR_EVENT:
    		break;
        default:
            break;
    }
}

void usbd_state_uac_iso_out(INT32U event)
{
	osStatus status;
	INT32U msg;

    switch(event)
    {
     	case USBD_ISO_OUT_DMA_DONE_EVENT:
			msg = MSG_UAC_REC_SPK_DATA_DONE;
			status = osMessagePut(usb_uac_aud_ctlblk.uac_dec_aud_q, (INT32U)&msg, osWaitForever);
			if(status != osOK)
			{
				DBG_PRINT("Send MSG_UAC_REC_SPK_DATA_DONE faled!!!\r\n");
			}
    		break;

    	case USBD_ISO_OUT_DONE_EVENT:
			msg = MSG_UAC_REC_SPK_PKT_DONE;
			status = osMessagePut(usb_uac_aud_ctlblk.uac_dec_aud_q, (INT32U)&msg, osWaitForever);
			if(status != osOK)
			{
				DBG_PRINT("Send MSG_UAC_REC_SPK_DATA_DONE faled!!!\r\n");
			}
    		break;

    	case USBD_ISO_OUT_NACK_EVENT:
    		drv_l1_enable_epc_iso_out_nack_event();
    		DBG_PRINT("ISO OUT NAK!!\r\n");
    		break;

    	case USBD_ISO_OUT_FAIL_EVENT:
    		drv_l1_enable_epc_iso_out_fail_event();
    		DBG_PRINT("ISO OUT FAIL!!\r\n");
    		break;

        default:
            return;
    }
}

/******************************************************
*       Miscellaneous event
*
*       Note. These are used for handling suspend, resume,
*			reset, set interface, set config event from l1
*******************************************************/
void usbd_uac_misc_handle(INT32U event)
{
    switch(event)
    {
        case USBD_MISC_RESET_EVENT:
            DBG_PRINT("UAC: Received RST!\r\n");
            drv_l1_usbd_reset();
            drv_l2_usbd_ctl_reset();
            uac_reset_ctl();
            break;
        case USBD_MISC_SET_CONF_EVENT:
            //DBG_PRINT("UAC: Get set config!\r\n");
            break;
        case USBD_MISC_SET_INTERFACE_EVENT:
        DBG_PRINT("UAC: Get set interface!\r\n");
        	uac_dec_stream_control();
        	uac_enc_stream_control();
            break;
        case USBD_MISC_GET_SUSPEND_EVENT:
            DBG_PRINT("UAC: Received suspend!\r\n");
            break;
        case USBD_MISC_GET_RESUM_EVENT:
            //DBG_PRINT("USBDL2UAC: drv_l2_usbd_uac_misc_handle, Received resume!\r\n");
            break;
        case USBD_MISC_SET_ADDRESS_EVENT:
        	//temp = (rUDC_ADDR & 0x7F);
        	DBG_PRINT("UAC: Set address = %d\r\n", (rUDC_ADDR & 0x7F));
        	break;
        default:
            break;
    }
}
