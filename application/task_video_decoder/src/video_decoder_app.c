#include <string.h>
#include "task_video_decoder.h"
#include "gplib_jpeg.h"
#include "gplib_jpeg_decode.h"
#include "drv_l1_dma.h"
#include "drv_l1_timer.h"
#include "drv_l1_scaler.h"
#include "drv_l1_jpeg.h"
#include "drv_l1_dac.h"
//#include "drv_l1_wrap.h"
//#include "drv_l1_conv422to420.h"
#include "drv_l2_scaler.h"
#include "drv_l1_pscaler.h"
#include "drv_l2_display.h"

#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECODER_EN == 1)
/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
static INT32S mjpeg_seek_to_jpeg_header(INT32U raw_data_addr);
static INT32S mjpeg_memory_alloc(mjpeg_info *pMjpeg);
static void mjpeg_memory_free(void);
static void mjpeg_decode_set_scaler(mjpeg_info *pMjpeg, ScalerFormat_t *pScale);

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static vid_dec_buf_struct g_audio;
static vid_dec_buf_struct  g_video;
static vid_dec_buf_struct  g_deblock;
static vid_dec_buf_struct  g_scaler;
static volatile INT32U g_lock_vid_addr;
static volatile INT32U g_jpeg_y_addr0;
static volatile INT32U g_jpeg_y_addr;
static volatile INT32U g_jpeg_cb_addr;
static volatile INT32U g_jpeg_cr_addr;
static volatile INT32U g_fifo_size;

// timer and sync
void vid_dec_timer_isr(void)
{
	if(p_vid_dec_para->audio_flag) {
		if(p_vid_dec_para->tv - p_vid_dec_para->ta < p_vid_dec_para->time_range) {
			p_vid_dec_para->tv += p_vid_dec_para->tick_time2;
		}

		if(p_vid_dec_para->ta - p_vid_dec_para->tv > p_vid_dec_para->time_range) {
			// Make video faster if ta is greater tv more than 32ms
			// This branch will only occur when ICE stop on debug point
			p_vid_dec_para->tv += p_vid_dec_para->tick_time2;
		}
	} else {
		p_vid_dec_para->tv += p_vid_dec_para->tick_time2;
	}

	if(p_vid_dec_para->tv - p_vid_dec_para->Tv >= 0) {
		if(p_vid_dec_para->post_cnt == p_vid_dec_para->pend_cnt) {
			if(video_decode_task_one_frame() >= 0) {
				p_vid_dec_para->post_cnt++;
			}
		}
	}
}

void vid_dec_stop_timer(void)
{
	timer_stop(VIDEO_DECODE_TIMER);
}

void vid_dec_start_timer(void)
{
	INT32U temp, freq_hz;


	if(p_vid_dec_para->audio_flag) {
		//temp = 0x10000 -((0x10000 - (R_TIMERE_PRELOAD & 0xFFFF)) * p_vid_dec_para->n);
		temp = (0x10000 - (R_TIMERE_PRELOAD & 0xFFFF)) * p_vid_dec_para->n;
		freq_hz = MCLK/2/temp;
		if(MCLK%(2*temp)) {
			freq_hz++;
		}
	} else {
		freq_hz = TIME_BASE_TICK_RATE;
	}
	timer_freq_setup(VIDEO_DECODE_TIMER, freq_hz, 0, vid_dec_timer_isr);
}

//memory
void audio_buffer_set_mute(void)
{
	INT32S i;

	for(i=0; i<AUDIO_FRAME_NO; i++) {
		if(p_vid_dec_para->audio_decode_addr[i]) {
			gp_memset((INT8S*)p_vid_dec_para->audio_decode_addr[i], 0x00, p_vid_dec_para->aud_frame_size);
		}
	}
}

static void vid_dec_buffer_number_init(void)
{
	g_lock_vid_addr = 0xFFFFFFFF;
	gp_memset((INT8S*)&g_audio, 0x00, sizeof(vid_dec_buf_struct));
	gp_memset((INT8S*)&g_video, 0x00, sizeof(vid_dec_buf_struct));
	gp_memset((INT8S*)&g_deblock, 0x00, sizeof(vid_dec_buf_struct));
	gp_memset((INT8S*)&g_scaler, 0x00, sizeof(vid_dec_buf_struct));

	//audio
	g_audio.total_number = AUDIO_FRAME_NO;

	//video
	if(p_vid_dec_para->video_format == C_MJPG_FORMAT) {
		//mjpg
		g_video.total_number = VIDEO_FRAME_NO;
	} else if(p_vid_dec_para->video_format == C_H264_FORMAT)
	{
        g_video.total_number = 0;
        g_deblock.total_number = 0;
        g_scaler.total_number = SCALER_FRAME_NO;
	}
	else {
		//mpeg4
		if(p_vid_dec_para->scaler_flag && p_vid_dec_para->deblock_flag) {
			g_video.total_number = 2;
			g_deblock.total_number = 2;
			g_scaler.total_number = SCALER_FRAME_NO;
		} else if(p_vid_dec_para->scaler_flag) {
			g_video.total_number = 2;
			g_deblock.total_number = 0;
			g_scaler.total_number = SCALER_FRAME_NO;
		} else if(p_vid_dec_para->deblock_flag) {
			g_video.total_number = 2;
			g_deblock.total_number = DEBLOCK_FRAME_NO;
			g_scaler.total_number = 0;
		} else {
			g_video.total_number = VIDEO_FRAME_NO;
			g_deblock.total_number = 0;
			g_scaler.total_number = 0;
		}
	}
}

static INT32S vid_dec_audio_memory_alloc(void)
{
	INT32S i, nRet;

	if(p_vid_dec_para->audio_flag == 0) {
		RETURN(0);
	}

	for(i=0; i<g_audio.total_number; i++) {
		if(p_vid_dec_para->audio_decode_addr[i] == 0) {
			p_vid_dec_para->audio_decode_addr[i] = (INT32U)gp_malloc_align(p_vid_dec_para->aud_frame_size, 4);
			if(p_vid_dec_para->audio_decode_addr[i] == 0) {
				RETURN(-1);
			}
			gp_memset((INT8S*)p_vid_dec_para->audio_decode_addr[i], 0x00, p_vid_dec_para->aud_frame_size);
		}
	}

	nRet = STATUS_OK;
Return:
	for(i=0; i<g_audio.total_number; i++) {
		DEBUG_MSG("AudioFrame = 0x%x\r\n", p_vid_dec_para->audio_decode_addr[i]);
	}
	return nRet;
}

static void vid_dec_audio_memory_free(void)
{
	INT32S i;

	for(i=0; i<g_audio.total_number; i++) {
		if(p_vid_dec_para->audio_decode_addr[i]) {
			gp_free((void*)p_vid_dec_para->audio_decode_addr[i]);
			p_vid_dec_para->audio_decode_addr[i] = 0;
		}
	}
}

static INT32S vid_dec_video_memory_alloc(void)
{
	INT16U *pdata;
	INT32S i, j, temp, nRet;

	if(p_vid_dec_para->video_flag == 0) {
		RETURN(0);
	}

	if(p_vid_dec_para->video_format == C_MJPG_FORMAT) {
		//mjpeg video
		if(p_vid_dec_para->scaler_flag) {
			temp = p_vid_dec_para->buffer_output_width * p_vid_dec_para->buffer_output_height * 2;
		} else {
			//height must be 16-align
			if(p_bitmap_info->biHeight % 16) {
				pdata = (INT16U*)& p_bitmap_info->biHeight;
				*pdata = (*pdata + 16) & 0xFFF0;
			}
			temp = p_bitmap_info->biWidth * p_bitmap_info->biHeight * 2;
		}

		for(i=0; i<g_video.total_number; i++) {
			if(p_vid_dec_para->video_decode_addr[i] == 0) {
				p_vid_dec_para->video_decode_addr[i] = (INT32U)gp_malloc_align(temp, 32);
				if(p_vid_dec_para->video_decode_addr[i] == 0) {
					RETURN(-1);
				}
				pdata = (INT16U*)p_vid_dec_para->video_decode_addr[i];
				for(j=0; j<(temp>>1); j++)
					*pdata++ = 0x0080;
			}
		}
	} else {
		//mpeg4 video
		temp = p_bitmap_info->biWidth * p_bitmap_info->biHeight * 2;
		for(i=0; i<g_video.total_number; i++) {
			if(p_vid_dec_para->video_decode_addr[i] == 0) {
				p_vid_dec_para->video_decode_addr[i] = (INT32U)gp_malloc_align(temp, 32);
				if(p_vid_dec_para->video_decode_addr[i] == 0) {
					RETURN(-1);
				}
				pdata = (INT16U*)p_vid_dec_para->video_decode_addr[i];
				for(j=0; j<(temp>>1); j++) {
					*pdata++ = 0x0080;
				}
			}
		}

		//deblock
		temp = p_bitmap_info->biWidth * p_bitmap_info->biHeight * 2;
		for(i=0; i<g_deblock.total_number; i++) {
			if(p_vid_dec_para->deblock_addr[i] == 0) {
				p_vid_dec_para->deblock_addr[i] = (INT32U)gp_malloc_align(temp, 32);
				if(p_vid_dec_para->deblock_addr[i] == 0) {
					RETURN(-1);
				}
				pdata = (INT16U*)p_vid_dec_para->deblock_addr[i];
				for(j=0; j<(temp>>1); j++) {
					*pdata++ = 0x0080;
				}
			}
		}

		//scaler
		temp = p_vid_dec_para->buffer_output_width * p_vid_dec_para->buffer_output_height * 2;
		for(i=0; i<g_scaler.total_number; i++) {
			if(p_vid_dec_para->scaler_output_addr[i] == 0) {
				p_vid_dec_para->scaler_output_addr[i] = (INT32U)gp_malloc_align(temp, 32);
				if(p_vid_dec_para->scaler_output_addr[i] == 0) {
					RETURN(-1);
				}
				pdata = (INT16U*)p_vid_dec_para->scaler_output_addr[i];
				for(j=0; j<(temp>>1); j++) {
					*pdata++ = 0x0080;
				}
			}
		}
	}

	nRet = STATUS_OK;
Return:
	for(i=0; i<g_video.total_number; i++) {
		DEBUG_MSG("VideoFrame = 0x%x\r\n", p_vid_dec_para->video_decode_addr[i]);
	}

	for(i=0; i<g_deblock.total_number; i++) {
		DEBUG_MSG("DeblockFrame = 0x%x\r\n", p_vid_dec_para->deblock_addr[i]);
	}

	for(i=0; i<g_scaler.total_number; i++) {
		DEBUG_MSG("ScalerFrame = 0x%x\r\n", p_vid_dec_para->scaler_output_addr[i]);
	}

	return nRet;
}

static void vid_dec_video_memory_free(void)
{
	INT8U  video_n, scaler_n, deblock_n;
	INT32U i;

	if(p_vid_dec_para->video_format == C_MJPG_FORMAT) {
		if(p_vid_dec_para->user_define_flag) {
			video_n = 2;
			scaler_n = deblock_n = 0;
		} else {
			video_n = scaler_n = deblock_n = 0;
		}
	} else {
		if(p_vid_dec_para->user_define_flag) {
			if(p_vid_dec_para->deblock_flag && p_vid_dec_para->scaler_flag) {
				scaler_n = 2;
				video_n = deblock_n = 0;
			} else if(p_vid_dec_para->deblock_flag) {
				deblock_n = 2;
				video_n = scaler_n = 0;
			} else if(p_vid_dec_para->scaler_flag) {
				scaler_n = 2;
				video_n = deblock_n = 0;
			} else {
				//no deblock and scaler
				video_n = 2;
				scaler_n = deblock_n = 0;
			}
		} else {
			video_n = scaler_n = deblock_n = 0;
		}
	}

	for(i=video_n; i<g_video.total_number; i++) {
		if(p_vid_dec_para->video_decode_addr[i]) {
			gp_free((void*)p_vid_dec_para->video_decode_addr[i]);
			p_vid_dec_para->video_decode_addr[i] = 0;
		}
	}

	for(i=deblock_n; i<g_deblock.total_number; i++) {
		if(p_vid_dec_para->deblock_addr[i]) {
			gp_free((void*)p_vid_dec_para->deblock_addr[i]);
			p_vid_dec_para->deblock_addr[i] = 0;
		}
	}

	for(i=scaler_n; i<g_scaler.total_number; i++) {
		if(p_vid_dec_para->scaler_output_addr[i]) {
			gp_free((void*)p_vid_dec_para->scaler_output_addr[i]);
			p_vid_dec_para->scaler_output_addr[i] = 0;
		}
	}
}

INT32S vid_dec_memory_alloc(void)
{
	//init index
	vid_dec_buffer_number_init();
	if(vid_dec_audio_memory_alloc() < 0) {
		return -1;
	}

	if(vid_dec_video_memory_alloc() < 0) {
		return -1;
	}

	return 0;
}

void vid_dec_memory_free(void)
{
	vid_dec_audio_memory_free();
	vid_dec_video_memory_free();
}

INT32S vid_dec_memory_realloc(void)
{
	INT32U addr[2];

	//free memory
	vid_dec_video_memory_free();

	//set scaler flag
	vid_dec_set_scaler(p_vid_dec_para->user_scaler_flag,
					p_vid_dec_para->video_decode_out_format,
					p_vid_dec_para->image_output_width,
					p_vid_dec_para->image_output_height,
					p_vid_dec_para->buffer_output_width,
					p_vid_dec_para->buffer_output_height);

	//user define buffer
	if(p_vid_dec_para->user_define_flag) {
		if(p_vid_dec_para->deblock_addr[0] && p_vid_dec_para->deblock_addr[1]) {
			addr[0] = p_vid_dec_para->deblock_addr[0];
			addr[1] = p_vid_dec_para->deblock_addr[1];
			p_vid_dec_para->deblock_addr[0] = 0;
			p_vid_dec_para->deblock_addr[1] = 0;
		} else if(p_vid_dec_para->scaler_output_addr[0] && p_vid_dec_para->scaler_output_addr[1]) {
			addr[0] = p_vid_dec_para->scaler_output_addr[0];
			addr[1] = p_vid_dec_para->scaler_output_addr[1];
			p_vid_dec_para->scaler_output_addr[0] = 0;
			p_vid_dec_para->scaler_output_addr[1] = 0;
		} else {
			addr[0] = p_vid_dec_para->video_decode_addr[0];
			addr[1] = p_vid_dec_para->video_decode_addr[1];
			p_vid_dec_para->video_decode_addr[0] = 0;
			p_vid_dec_para->video_decode_addr[1] = 0;
		}
		vid_dec_set_user_define_buffer(p_vid_dec_para->user_define_flag, addr[0], addr[1]);
	}

	//init index
	vid_dec_buffer_number_init();

	//alloc memory
	if(vid_dec_video_memory_alloc() < 0) {
		return -1;
	}

	return 0;
}

//buffer
void vid_dec_buffer_lock(INT32U addr)
{
	g_lock_vid_addr = addr;
}

void vid_dec_buffer_unlock(void)
{
	g_lock_vid_addr = 0xFFFFFFFF;
}

INT32U vid_dec_get_next_vid_buffer(void)
{
	INT32U addr;

	do {
		addr = p_vid_dec_para->video_decode_addr[g_video.current_index++];
		if(g_video.current_index >= g_video.total_number)
			g_video.current_index = 0;
	} while(addr == g_lock_vid_addr);
	return addr;
}

INT32U vid_dec_get_next_deblock_buffer(void)
{
	INT32U addr;

	do {
		addr = p_vid_dec_para->deblock_addr[g_deblock.current_index++];
		if(g_deblock.current_index >= g_deblock.total_number)
			g_deblock.current_index = 0;
	}while(addr == g_lock_vid_addr);
	return addr;
}

INT32U vid_dec_get_next_aud_buffer(void)
{
	INT32U addr;

	do {
		addr = p_vid_dec_para->audio_decode_addr[g_audio.current_index++];
		if(g_audio.current_index >= g_audio.total_number)
			g_audio.current_index = 0;
	} while(addr == g_lock_vid_addr);
	return addr;
}

INT32U vid_dec_get_next_scaler_buffer(void)
{
	INT32U addr;

	do {
		addr = p_vid_dec_para->scaler_output_addr[g_scaler.current_index++];
		if(g_scaler.current_index >= g_scaler.total_number)
			g_scaler.current_index = 0;
	} while(addr == g_lock_vid_addr);
	return addr;
}

//status
void vid_dec_set_status(INT32S flag)
{
	p_vid_dec_para->status |= flag;
}

void vid_dec_clear_status(INT32S flag)
{
	p_vid_dec_para->status &= ~flag;
}

INT32S vid_dec_get_status(void)
{
	return p_vid_dec_para->status;
}

//video info
void vid_dec_set_video_flag(INT8S video_flag)
{
	if(video_flag) {
		p_vid_dec_para->video_flag = TRUE;
	} else {
		p_vid_dec_para->video_flag = FALSE;
	}
}

INT8S vid_dec_get_video_flag(void)
{
	return p_vid_dec_para->video_flag;
}

INT32S vid_dec_get_video_format(INT32U biCompression)
{
	INT8U data[4];

	data[0] = (biCompression >> 0) & 0xFF; //X
	data[1] = (biCompression >> 8) & 0xFF; //X
	data[2] = (biCompression >> 16) & 0xFF; //X
	data[3] = (biCompression >> 24) & 0xFF; //X

	if( (data[0] == 'X' || data[0] == 'x') &&
		(data[1] == 'V' || data[1] == 'v') &&
		(data[2] == 'I' || data[2] == 'i') &&
		(data[3] == 'D' || data[3] == 'd')) {
		DEBUG_MSG("VidFormat = C_XVID_FORMAT\r\n");
		return C_XVID_FORMAT;
	} else if((data[0] == 'M' || data[0] == 'm') &&
			(data[1] == '4') &&
			(data[2] == 'S' || data[2] == 's') &&
			(data[3] == '2')) {
		DEBUG_MSG("VidFormat = C_M4S2_FORMAT\r\n");
		return C_M4S2_FORMAT;
	} else if((data[0] == 'F' || data[0] == 'f') &&
			(data[1] == 'M' || data[1] == 'm') &&
			(data[2] == 'P' || data[2] == 'p') &&
			(data[3] == '4')) {
		DEBUG_MSG("VidFormat = C_FMP4_FORMAT\r\n");
		return C_M4S2_FORMAT;
	} else if((data[0] == 'H' || data[0] == 'h') &&
			(data[1] == '2') &&
			(data[2] == '6') &&
			(data[3] == '3')) {
		DEBUG_MSG("VidFormat = C_H263_FORMAT\r\n");
		return C_H263_FORMAT;
	} else if((data[0] == 'M' || data[0] == 'm') &&
			(data[1] == 'J' || data[1] == 'j') &&
			(data[2] == 'P' || data[2] == 'p') &&
			(data[3] == 'G' || data[3] == 'g')) {
		DEBUG_MSG("VidFormat = C_MJPG_FORMAT\r\n");
		return C_MJPG_FORMAT;
	} else if((data[0] == 'H' || data[0] == 'h') &&
			(data[1] == '2') &&
			(data[2] == '6') &&
			(data[3] == '4')) {
		DEBUG_MSG("VidFormat = C_H264_FORMAT\r\n");
		return C_H264_FORMAT;
	}

	DEBUG_MSG("NotSupportVidFormat = 0x%x\r\n", biCompression);
	return STATUS_FAIL;
}

INT32S vid_dec_get_file_format(INT8S *pdata)
{
	if( (*(pdata+0) == 'A' || *(pdata+0) == 'a') &&
		(*(pdata+1) == 'V' || *(pdata+1) == 'v') &&
		(*(pdata+2) == 'I' || *(pdata+2) == 'i')) {
		return FILE_TYPE_AVI;
	} else if((*(pdata+0) == 'M' || *(pdata+0) == 'm') &&
			(*(pdata+1) == 'O' || *(pdata+1) == 'o') &&
			(*(pdata+2) == 'V' || *(pdata+2) == 'v')) {
		return FILE_TYPE_MOV;
	} else if((*(pdata+0) == 'M' || *(pdata+0) == 'm') &&
			(*(pdata+1) == 'P' || *(pdata+1) == 'p') &&
			(*(pdata+2) == '4' || *(pdata+2) == '4')) {
		return FILE_TYPE_MOV;
	} else if((*(pdata+0) == '3' || *(pdata+0) == '3') &&
			(*(pdata+1) == 'G' || *(pdata+1) == 'g') &&
			(*(pdata+2) == 'P' || *(pdata+2) == 'p')) {
		return FILE_TYPE_MOV;
	} else if((*(pdata+0) == 'M' || *(pdata+0) == 'm') &&
			(*(pdata+1) == '4' || *(pdata+1) == '4') &&
			(*(pdata+2) == 'A' || *(pdata+2) == 'a')) {
		return FILE_TYPE_MOV;
	}

	return STATUS_FAIL;
}

void vid_dec_get_size(INT16U *width, INT16U *height)
{
	*width = p_bitmap_info->biWidth;
	*height = p_bitmap_info->biHeight;
}

//deblock
void vid_dec_set_deblock_flag(INT8S deblock_flag)
{
	if(deblock_flag) {
		p_vid_dec_para->deblock_flag = TRUE;
	} else {
		p_vid_dec_para->deblock_flag = FALSE;
	}
}

INT8S vid_dec_get_deblock_flag(void)
{
	return p_vid_dec_para->deblock_flag;
}

//audio info
void vid_dec_set_audio_flag(INT8S audio_flag)
{
	if(audio_flag) {
		p_vid_dec_para->audio_flag = TRUE;
	} else {
		p_vid_dec_para->audio_flag = FALSE;
	}
}

INT8S vid_dec_get_audio_flag(void)
{
	return p_vid_dec_para->audio_flag;
}

//audio decoder
INT32S vid_dec_set_aud_dec_work_mem(INT32U work_mem_size)
{
	p_vid_dec_para->work_mem_size = work_mem_size;
	p_vid_dec_para->work_mem_addr = (INT8U *)gp_malloc_align(work_mem_size, 4);
	if(!p_vid_dec_para->work_mem_addr) {
		return -1;
	}

	gp_memset((INT8S*)p_vid_dec_para->work_mem_addr, 0, work_mem_size);
	return 0;
}

INT32S vid_dec_set_aud_dec_ring_buffer(void)
{
	p_vid_dec_para->ring_buffer_size = MultiMediaParser_GetAudRingLen(p_vid_dec_para->media_handle);
	p_vid_dec_para->ring_buffer_addr = (INT8U *)MultiMediaParser_GetAudRing(p_vid_dec_para->media_handle);
	if(!p_vid_dec_para->ring_buffer_addr) {
		return -1;
	}

	return 0;
}

void vid_dec_aud_dec_memory_free(void)
{
	if(p_vid_dec_para->work_mem_addr) {
		gp_free((void*)p_vid_dec_para->work_mem_addr);
	}

	p_vid_dec_para->work_mem_size = 0;
	p_vid_dec_para->work_mem_addr = 0;
	p_vid_dec_para->ring_buffer_size = 0;
	p_vid_dec_para->ring_buffer_addr = 0;
}

//dac init
void aud_dec_dac_start(INT8U channel_no, INT32U sample_rate)
{

	if(channel_no == 1) {
		drv_l1_dac_mono_set();
	} else if(channel_no == 2){
		drv_l1_dac_stereo_set();
	} else {
		return;
	}
	drv_l1_dac_sample_rate_set(sample_rate);
}

void aud_dec_dac_stop(void)
{
	drv_l1_dac_timer_stop();
}

void aud_dec_ramp_down_handle(INT8U channel_no)
{
	INT16S  last_ldata,last_rdata;
	INT16S  i, temp;

	temp = 0 - DAC_RAMP_DOWN_STEP;
	last_ldata = R_DAC_CHA_DATA;
	last_rdata = R_DAC_CHB_DATA;
	//unsigned to signed
	last_ldata ^= 0x8000;
	last_rdata ^= 0x8000;

	if ((last_ldata == 0x0) && (last_rdata == 0x0)) {
		return;
	}

	//change timer to 44100
	drv_l1_dac_sample_rate_set(44100);
	while(1)
	{
		if (last_ldata > 0x0) {
			last_ldata -= DAC_RAMP_DOWN_STEP;
		} else if (last_ldata < 0x0)  {
			last_ldata += DAC_RAMP_DOWN_STEP;
		}

		if ((last_ldata < DAC_RAMP_DOWN_STEP)&&(last_ldata > temp)) {
			last_ldata = 0;
		}

		if (channel_no == 2) {
			if (last_rdata > 0x0) {
				last_rdata -= DAC_RAMP_DOWN_STEP;
		    } else if (last_rdata < 0x0) {
				last_rdata += DAC_RAMP_DOWN_STEP;
		    }

		    if ((last_rdata < DAC_RAMP_DOWN_STEP)&&(last_rdata > temp)) {
				last_rdata = 0;
			}
		}

		for(i=0;i<DAC_RAMP_DOWN_STEP_HOLD;i++) {
			if (channel_no == 2) {
				while(R_DAC_CHA_FIFO & 0x8000);
				R_DAC_CHA_DATA = last_ldata;
				while(R_DAC_CHB_FIFO & 0x8000);
				R_DAC_CHB_DATA = last_rdata;
			} else {
				while(R_DAC_CHA_FIFO & 0x8000);
				R_DAC_CHA_DATA = last_ldata;
			}
		}

		if (channel_no == 2) {
			if ((last_ldata == 0x0) && (last_rdata == 0x0)) {
				break;
			}
		} else {
			if (last_ldata == 0x0) {
				break;
			}
		}
	}
	drv_l1_dac_timer_stop();
}

//scaler
INT32S vid_dec_set_scaler(INT32U scaler_flag, INT32U out_fmt, INT16U out_w, INT16U out_h, INT16U out_buf_w, INT16U out_buf_h)
{
	INT8U  enable;
	INT32S nRet;

	if(p_vid_dec_para->video_format == C_MJPG_FORMAT) {
		switch(out_fmt)
		{
		case C_SCALER_CTRL_OUT_RGB565:
			enable = 1;
			p_vid_dec_para->video_decode_out_format = C_SCALER_CTRL_OUT_RGB565;
			break;

		case C_SCALER_CTRL_OUT_YUYV:
		case C_SCALER_CTRL_OUT_UYVY:
			enable = 0;
			p_vid_dec_para->video_decode_out_format = out_fmt;
			break;

		default:
			DEBUG_MSG("OutputFormatNoSupport!!!\r\n");
			RETURN(STATUS_FAIL);
		}
	} else {
		//mpeg4
		switch(out_fmt)
		{
		case C_SCALER_CTRL_OUT_RGB565:
			enable = 1;
			p_vid_dec_para->mpeg4_decode_out_format = C_MP4_OUTPUT_Y1UY0V;
			p_vid_dec_para->video_decode_in_format = C_SCALER_CTRL_IN_YUYV;
			p_vid_dec_para->video_decode_out_format = out_fmt;
			break;

		case C_SCALER_CTRL_OUT_YUYV:
			enable = 0;
			p_vid_dec_para->mpeg4_decode_out_format = C_MP4_OUTPUT_Y1UY0V;
			p_vid_dec_para->video_decode_in_format = C_SCALER_CTRL_IN_YUYV;
			p_vid_dec_para->video_decode_out_format = C_SCALER_CTRL_OUT_YUYV;
			break;

		case C_SCALER_CTRL_OUT_UYVY:
			enable = 0;
			p_vid_dec_para->mpeg4_decode_out_format = C_MP4_OUTPUT_UY1VY0;
			p_vid_dec_para->video_decode_in_format = C_SCALER_CTRL_IN_UYVY;
			p_vid_dec_para->video_decode_out_format = C_SCALER_CTRL_OUT_UYVY;
			break;

		default:
			DEBUG_MSG("OutputFormatNoSupport!!!\r\n");
			RETURN(STATUS_FAIL);
		}
	}

	if((out_w != out_buf_w) || (out_h != out_buf_h)) {
		enable = 1;
	}

	if((p_bitmap_info->biWidth != out_w) || (p_bitmap_info->biHeight != out_h)) {
		enable = 1;
	}

	if(enable == 0) {
		// not need use scaler
		scaler_flag = SCALE_MAX;
	}

	switch(scaler_flag)
	{
	case SCALE_FULL_SCREEN:
	case SCALE_FIT_BUF:
		if(scaler_flag == SCALE_FULL_SCREEN) {
			p_vid_dec_para->scaler_flag = C_SCALER_FULL_SCREEN;
		} else {
			p_vid_dec_para->scaler_flag = C_SCALER_BY_RATIO;
		}
		p_vid_dec_para->image_output_width = out_w;
		p_vid_dec_para->image_output_height = out_h;
		p_vid_dec_para->buffer_output_width = out_buf_w;
		p_vid_dec_para->buffer_output_height = out_buf_h;
		p_vid_dec_para->xoffset = p_vid_dec_para->yoffset = 0;
		break;

	case NO_SCALE:
	case NO_SCALE_AT_CENTER:
		p_vid_dec_para->scaler_flag = C_SCALER_BY_RATIO;
		p_vid_dec_para->image_output_width = p_bitmap_info->biWidth;
		p_vid_dec_para->image_output_height = p_bitmap_info->biHeight;
		p_vid_dec_para->buffer_output_width = out_buf_w;
		p_vid_dec_para->buffer_output_height = out_buf_h;

		if(scaler_flag == NO_SCALE) {
			p_vid_dec_para->xoffset = p_vid_dec_para->yoffset = 0;
		} else {
			p_vid_dec_para->xoffset = ((out_buf_w - p_bitmap_info->biWidth) / 2) & (~0x0F);
			p_vid_dec_para->yoffset = ((out_buf_h - p_bitmap_info->biHeight) / 2) & (~0x0F);
		}
		break;

	case SCALE_MAX:
		p_vid_dec_para->scaler_flag = 0;
		p_vid_dec_para->image_output_width = p_bitmap_info->biWidth;
		p_vid_dec_para->image_output_height = p_bitmap_info->biHeight;
		p_vid_dec_para->buffer_output_width = p_bitmap_info->biWidth;
		p_vid_dec_para->buffer_output_height = p_bitmap_info->biHeight;
		p_vid_dec_para->xoffset = p_vid_dec_para->yoffset = 0;
		break;

	default:
		p_vid_dec_para->scaler_flag = C_SCALER_FULL_SCREEN;
		p_vid_dec_para->image_output_width = out_w;
		p_vid_dec_para->image_output_height = out_h;
		p_vid_dec_para->buffer_output_width = out_buf_w;
		p_vid_dec_para->buffer_output_height = out_buf_h;
		p_vid_dec_para->xoffset = p_vid_dec_para->yoffset = 0;
		break;
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

void PScaler_dec_callback(INT32U PScaler_Event)
{
    INT32U display_addr = 0;
    if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
    {
        display_addr = drv_l1_pscaler_output_A_buffer_get(PSCALER_A);
        drv_l1_pscaler_output_A_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
    }
    else if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
    {
        display_addr = drv_l1_pscaler_output_B_buffer_get(PSCALER_A);
        drv_l1_pscaler_output_B_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
    }

    if (display_addr && viddec_display)
        viddec_display(p_vid_dec_para->buffer_output_width, p_vid_dec_para->buffer_output_height, display_addr);
}
INT32S vid_dec_set_pscaler(INT32U scaler_flag, INT32U video_output_format,
						INT16U image_output_width, INT16U image_output_height,
						INT16U buffer_output_width, INT16U buffer_output_height)
{
	INT8U  enable;
	INT32S nRet;
	INT32U xFactor, yFactor;

	enable = 1;

	if(enable && scaler_flag) {
		p_vid_dec_para->scaler_flag = scaler_flag;
		p_vid_dec_para->image_output_width = image_output_width;
		p_vid_dec_para->image_output_height = image_output_height;
		p_vid_dec_para->buffer_output_width = buffer_output_width;
		p_vid_dec_para->buffer_output_height = buffer_output_height;
	} else if(enable && (scaler_flag == NO_SCALE)) {
		p_vid_dec_para->scaler_flag = SCALE_FULL_SCREEN;
		p_vid_dec_para->image_output_width = p_bitmap_info->biWidth;
		p_vid_dec_para->image_output_height = p_bitmap_info->biHeight;
		p_vid_dec_para->buffer_output_width = p_bitmap_info->biWidth;
		p_vid_dec_para->buffer_output_height = p_bitmap_info->biHeight;
	} else {
		p_vid_dec_para->scaler_flag = NO_SCALE;
		p_vid_dec_para->image_output_width = p_bitmap_info->biWidth;
		p_vid_dec_para->image_output_height = p_bitmap_info->biHeight;
		p_vid_dec_para->buffer_output_width = p_bitmap_info->biWidth;
		p_vid_dec_para->buffer_output_height = p_bitmap_info->biHeight;
	}

/*	switch(video_output_format)
	{
	case C_SCALER_CTRL_OUT_RGB1555:
	case C_SCALER_CTRL_OUT_RGB565:
	case C_SCALER_CTRL_OUT_RGBG:
	case C_SCALER_CTRL_OUT_GRGB:
	case C_SCALER_CTRL_OUT_YUYV8X32:
	case C_SCALER_CTRL_OUT_YUYV8X64:
	case C_SCALER_CTRL_OUT_YUYV16X32:
	case C_SCALER_CTRL_OUT_YUYV16X64:
	case C_SCALER_CTRL_OUT_YUYV32X32:
	case C_SCALER_CTRL_OUT_YUYV64X64:
		p_vid_dec_para->video_decode_in_format = C_SCALER_CTRL_IN_YUYV;
		p_vid_dec_para->video_decode_out_format = video_output_format;
		break;
	case C_SCALER_CTRL_OUT_YUYV:
		p_vid_dec_para->video_decode_in_format = C_SCALER_CTRL_IN_YUYV;
		p_vid_dec_para->video_decode_out_format = C_SCALER_CTRL_OUT_YUYV;
		break;
	case C_SCALER_CTRL_OUT_UYVY:
		p_vid_dec_para->video_decode_in_format = C_SCALER_CTRL_IN_UYVY;
		p_vid_dec_para->video_decode_out_format = C_SCALER_CTRL_OUT_UYVY;
		break;
	default:
		DEBUG_MSG(DBG_PRINT("OutputFormatNoSupport!!!\r\n"));
		RETURN(STATUS_FAIL);
	}*/

	p_vid_dec_para->video_decode_in_format = PIPELINE_SCALER_INPUT_FORMAT_YUYV;
    p_vid_dec_para->video_decode_out_format = video_output_format;

	drv_l1_pscaler_clk_ctrl(PSCALER_A,1);
    drv_l1_pscaler_init(PSCALER_A);
    drv_l1_pscaler_callback_register(0, PScaler_dec_callback);

    xFactor = (p_bitmap_info->biWidth*0x10000)/p_vid_dec_para->image_output_width;
    yFactor = (p_bitmap_info->biHeight*0x10000)/p_vid_dec_para->image_output_height;

    drv_l1_pscaler_input_pixels_set(PSCALER_A, p_bitmap_info->biWidth, p_bitmap_info->biHeight);
    drv_l1_pscaler_output_pixels_set(PSCALER_A, xFactor, p_vid_dec_para->image_output_width, yFactor, p_vid_dec_para->image_output_height);
    drv_l1_pscaler_input_X_start_set(PSCALER_A, 0);
    drv_l1_pscaler_input_Y_start_set(PSCALER_A, 0);
    drv_l1_pscaler_output_fifo_line_set(PSCALER_A, p_vid_dec_para->image_output_height, 0);
    drv_l1_pscaler_interrupt_set(PSCALER_A, PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_A | PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_B);
    //drv_l1_pscaler_interrupt_set(PSCALER_A, PIPELINE_SCALER_INT_ENABLE_ALL);
    drv_l1_pscaler_input_format_set(PSCALER_A, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
    drv_l1_pscaler_output_format_set(PSCALER_A, video_output_format);
    drv_l1_pscaler_input_source_set(PSCALER_A, PIPELINE_SCALER_INPUT_SOURCE_MB420);
    //drv_l1_pscaler_input_buffer_set(PSCALER_A);
    //drv_l1_pscaler_H264_output_A_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
    //drv_l1_pscaler_H264_output_B_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
    drv_l1_pscaler_start(PSCALER_A);

	nRet = STATUS_OK;
Return:
	return nRet;
}
void vid_dec_set_user_define_buffer(INT8U user_flag, INT32U addr0, INT32U addr1)
{
	if(user_flag) {
		//user use define buffer
		p_vid_dec_para->user_define_flag = TRUE;
		if(p_vid_dec_para->video_format == C_MJPG_FORMAT) {
			p_vid_dec_para->video_decode_addr[0] = addr0;
			p_vid_dec_para->video_decode_addr[1] = addr1;
			p_vid_dec_para->scaler_output_addr[0] = 0;
			p_vid_dec_para->scaler_output_addr[1] = 0;
			p_vid_dec_para->deblock_addr[0] = 0;
			p_vid_dec_para->deblock_addr[1] = 0;
		} else {
			if(p_vid_dec_para->deblock_flag && p_vid_dec_para->scaler_flag) {
				p_vid_dec_para->scaler_output_addr[0] = addr0;
				p_vid_dec_para->scaler_output_addr[1] = addr1;
			} else if(p_vid_dec_para->deblock_flag) {
				p_vid_dec_para->deblock_addr[0] = addr0;
				p_vid_dec_para->deblock_addr[1] = addr1;
			} else if(p_vid_dec_para->scaler_flag) {
				p_vid_dec_para->scaler_output_addr[0] = addr0;
				p_vid_dec_para->scaler_output_addr[1] = addr1;
			} else {
				p_vid_dec_para->video_decode_addr[0] = addr0;
				p_vid_dec_para->video_decode_addr[1] = addr1;
			}
		}
	} else {
		p_vid_dec_para->user_define_flag = FALSE;
		p_vid_dec_para->video_decode_addr[0] = 0;
		p_vid_dec_para->video_decode_addr[1] = 0;
		p_vid_dec_para->scaler_output_addr[0] = 0;
		p_vid_dec_para->scaler_output_addr[1] = 0;
		p_vid_dec_para->deblock_addr[0] = 0;
		p_vid_dec_para->deblock_addr[1] = 0;
	}
}

INT32S pscaler_start(INT32U in_width, INT32U in_height, INT32U buf_width, INT32U buf_height)
{
    INT32U xFactor, yFactor;
    INT32U video_output_format;

	drv_l1_pscaler_clk_ctrl(PSCALER_A,1);
    drv_l1_pscaler_init(PSCALER_A);
    drv_l1_pscaler_callback_register(0, PScaler_dec_callback);

    switch(p_vid_dec_para->video_decode_out_format)
    {
    case C_SCALER_CTRL_OUT_RGB565:  video_output_format = PIPELINE_SCALER_OUTPUT_FORMAT_RGB565; break;
    case C_SCALER_CTRL_OUT_YUYV:    video_output_format = PIPELINE_SCALER_OUTPUT_FORMAT_YUYV; break;
    }

    xFactor = (in_width*0x10000)/p_vid_dec_para->image_output_width;
    yFactor = (in_height*0x10000)/p_vid_dec_para->image_output_height;

    drv_l1_pscaler_input_pixels_set(PSCALER_A, buf_width, buf_height);
    drv_l1_pscaler_output_pixels_set(PSCALER_A, xFactor, p_vid_dec_para->image_output_width, yFactor, p_vid_dec_para->image_output_height);
    drv_l1_pscaler_input_X_start_set(PSCALER_A, 0);
    drv_l1_pscaler_input_Y_start_set(PSCALER_A, 0);
    drv_l1_pscaler_output_fifo_line_set(PSCALER_A, p_vid_dec_para->image_output_height, 0);
    drv_l1_pscaler_interrupt_set(PSCALER_A, PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_A | PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_B);
    //drv_l1_pscaler_interrupt_set(PSCALER_A, PIPELINE_SCALER_INT_ENABLE_ALL);
    drv_l1_pscaler_input_format_set(PSCALER_A, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
    drv_l1_pscaler_output_format_set(PSCALER_A, video_output_format);
    drv_l1_pscaler_input_source_set(PSCALER_A, PIPELINE_SCALER_INPUT_SOURCE_MB420);
    //drv_l1_pscaler_input_buffer_set(PSCALER_A);
    //drv_l1_pscaler_H264_output_A_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
    //drv_l1_pscaler_H264_output_B_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
	if (g_scaler.total_number)
	{
		drv_l1_pscaler_output_A_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
		drv_l1_pscaler_output_B_buffer_set(PSCALER_A, vid_dec_get_next_scaler_buffer());
	}
    drv_l1_pscaler_start(PSCALER_A);
}

INT32S pscaler_stop()
{
    drv_l1_pscaler_stop(PSCALER_A);
    drv_l1_pscaler_clk_ctrl(PSCALER_A,0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//display frame buffer scaler
INT32S scaler_size_start(scaler_info *pScaler)
{
	INT32U temp_x;
	INT32S scaler_status;

	if(pScaler->output_x > pScaler->output_buffer_x) {
		pScaler->output_x = pScaler->output_buffer_x;
	}

	if(pScaler->output_y > pScaler->output_buffer_y) {
		pScaler->output_y = pScaler->output_buffer_y;
	}
/*
  	// Initiate Scaler
  	drv_l1_scaler_init();
  	drv_l1_scaler_input_pixels_set(pScaler->input_x, pScaler->input_y);
	drv_l1_scaler_input_visible_pixels_set(pScaler->input_visible_x, pScaler->input_visible_y);

	switch(pScaler->scaler_mode)
	{
    case C_SCALER_FULL_SCREEN:
		drv_l1_scaler_output_pixels_set((pScaler->input_x<<16)/pScaler->output_x,
								(pScaler->input_y<<16)/pScaler->output_y,
								pScaler->output_buffer_x,
								pScaler->output_buffer_y);
		break;

	case C_SCALER_FIT_BUFFER:
      	if (pScaler->output_y * pScaler->input_x > pScaler->output_x * pScaler->input_y) {
      		temp_x = (pScaler->input_x<<16) / pScaler->output_x;
      	} else {
      		temp_x = (pScaler->input_y<<16) / pScaler->output_y;
      	}

     	drv_l1_scaler_output_pixels_set(temp_x, temp_x, pScaler->output_buffer_x, pScaler->output_buffer_y);
     	break;

    case C_NO_SCALER_FIT_BUFFER:
    	if((pScaler->input_x <= pScaler->output_x) && (pScaler->input_y <= pScaler->output_y)) {
			drv_l1_scaler_output_pixels_set(1<<16, 1<<16, pScaler->output_buffer_x, pScaler->output_buffer_y);
    	}
    	break;

    case C_NO_SCALER_CENTER:
    	if((pScaler->input_x <= pScaler->output_x) && (pScaler->input_y <= pScaler->output_y)) {
			drv_l1_scaler_output_pixels_set(1<<16, 1<<16, pScaler->output_buffer_x, pScaler->output_y);
    	}
     	break;

    default:
    	return -1;
	}

#if 0
	drv_l1_scaler_input_offset_set(0, 0);
#else
	// set interpolation enable.
	drv_l1_scaler_input_offset_set(0x8000, 0x8000);
#endif
	drv_l1_scaler_input_addr_set(pScaler->input_addr, 0, 0);
   	drv_l1_scaler_output_addr_set(pScaler->output_addr, 0, 0);
   	drv_l1_scaler_fifo_line_set(C_SCALER_CTRL_FIFO_DISABLE);
	drv_l1_scaler_input_format_set(pScaler->input_format);
	drv_l1_scaler_output_format_set(pScaler->output_format);
	drv_l1_scaler_out_of_boundary_color_set(pScaler->boundary_color);

	scaler_status = scaler_wait_idle();
	if (scaler_status == C_SCALER_STATUS_STOP) {
		drv_l1_scaler_start();
	}
*/
	return scaler_status;
}

INT32S scaler_size_wait_done(void)
{
	INT32S scaler_status;
/*
	scaler_status = scaler_wait_idle();

	if(scaler_status & C_SCALER_STATUS_DONE) {
		scaler_stop();
	} else if (scaler_status & C_SCALER_STATUS_TIMEOUT) {
		DEBUG_MSG(DBG_PRINT("Scaler Timeout failed to finish its job\r\n"));
	} else if(scaler_status & C_SCALER_STATUS_INIT_ERR) {
		DEBUG_MSG(DBG_PRINT("Scaler INIT ERR failed to finish its job\r\n"));
	} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
		scaler_restart();
	} else if(scaler_status & C_SCALER_STATUS_STOP) {
		DEBUG_MSG(DBG_PRINT("ScalerStop.\r\n"));
	} else {
  		DEBUG_MSG(DBG_PRINT("Un-handled Scaler status!\r\n"));
  	}
*/
	return scaler_status;
}

#if MJPEG_DECODE_ENABLE == 1
////////////////////////////////////////////////////////////////////////////////////////////////////////
//mjpeg decode api
static INT32S mjpeg_seek_to_jpeg_header(INT32U raw_data_addr)
{
	INT8U *pdata;
	INT8S cnt;
	INT16U wdata;
	INT32U i;

	cnt = 0;
	//seek to data
	pdata = (INT8U*)raw_data_addr;
	wdata = *pdata++;
	wdata <<= 8;
	wdata |= *pdata++;

	for(i=0; i<100; i++) {
		if(wdata == 0xFFD8) {
			break;
		}

		wdata <<= 8;
		wdata |= *pdata++;
		cnt++;
	}

	if(i == 100) {
		return -1;
	}

	return cnt;
}

INT32S mjpeg_decode_get_size(INT32U raw_data_addr, INT32U size, INT16U *width, INT16U *height, INT16U *img_yuv_mode)
{
	INT32S nRet;

	nRet = mjpeg_seek_to_jpeg_header(raw_data_addr);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	size -= nRet;
	raw_data_addr += nRet;
	jpeg_decode_init();
	gplib_jpeg_default_huffman_table_load();  //set defualt table after jpeg init
	nRet = jpeg_decode_parse_header((INT8U *) raw_data_addr, size);
	if(nRet != JPEG_PARSE_OK) {
		RETURN(STATUS_FAIL);
	}

	*width = jpeg_decode_image_width_get();
	*height = jpeg_decode_image_height_get();
	*img_yuv_mode = jpeg_decode_image_yuv_mode_get();
	nRet = STATUS_OK;
Return:
	return nRet;
}

static INT32S mjpeg_memory_alloc(mjpeg_info *pMjpeg)
{
	INT16U cbcr_shift;
	INT32U y_size=0, cb_cr_size=0;

	if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV420) {
		cbcr_shift = 2;
		//factor = 15;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV411) {
		cbcr_shift = 2;
		//factor = 15;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV422) {
		cbcr_shift = 1;
		//factor = 20;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV422V) {
		cbcr_shift = 1;
		//factor = 20;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV444) {
		cbcr_shift = 0;
		//factor = 30;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_GRAYSCALE) {
		cbcr_shift = 32;
		//factor = 10;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV411V) {
		cbcr_shift = 2;
		//factor = 15;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV420H2) {
		cbcr_shift = 1;
		//factor = 15;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV420V2) {
		cbcr_shift = 1;
		//factor = 15;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV411H2) {
		cbcr_shift = 1;
		//factor = 15;
	} else if (pMjpeg->jpeg_yuv_mode == C_JPG_CTRL_YUV411V2) {
		cbcr_shift = 1;
		//factor = 15;
	} else {
		return STATUS_FAIL;
	}

	if (pMjpeg->fifo_line) {
		switch(pMjpeg->jpeg_yuv_mode)
		{
		case C_JPG_CTRL_YUV444:
		case C_JPG_CTRL_YUV422:
		case C_JPG_CTRL_YUV422V:
		case C_JPG_CTRL_YUV420:
		case C_JPG_CTRL_YUV411:
			y_size = pMjpeg->jpeg_extend_w*pMjpeg->fifo_line*2*2; //YUYV
			cb_cr_size = 0;
			break;

		case C_JPG_CTRL_GRAYSCALE:
		case C_JPG_CTRL_YUV411V:
		case C_JPG_CTRL_YUV420H2:
		case C_JPG_CTRL_YUV420V2:
		case C_JPG_CTRL_YUV411H2:
		case C_JPG_CTRL_YUV411V2:
			y_size = pMjpeg->jpeg_extend_w*pMjpeg->fifo_line*2;
			cb_cr_size = y_size >> cbcr_shift;
		}
	} else {
		y_size = pMjpeg->jpeg_extend_w * pMjpeg->jpeg_extend_h;
		cb_cr_size = y_size >> cbcr_shift;
	}

	// allocate internal first
	g_fifo_size = y_size + cb_cr_size + cb_cr_size;
	g_jpeg_y_addr0 = (INT32U) gp_iram_malloc_align(g_fifo_size + 0x3f, 64);
	if (g_jpeg_y_addr0 == 0) {
		g_jpeg_y_addr0 = (INT32U) gp_malloc_align(g_fifo_size + 0x3f, 64);
		if (g_jpeg_y_addr0 == 0) {
  		return STATUS_FAIL;
        }
    }

    g_jpeg_y_addr = (g_jpeg_y_addr0 + 0x3f) >> 6 << 6;

	if(cb_cr_size) {
		g_jpeg_cb_addr = g_jpeg_y_addr + y_size;
		g_jpeg_cr_addr = g_jpeg_cb_addr + cb_cr_size;
  		}

  	return STATUS_OK;
}

static void mjpeg_memory_free(void)
{
	if(g_jpeg_y_addr0) {
		gp_free((void*)g_jpeg_y_addr0);
	}

	g_jpeg_y_addr0 = 0;
	g_jpeg_cb_addr = 0;
	g_jpeg_cr_addr = 0;
}

static void mjpeg_decode_set_scaler(mjpeg_info *pMjpeg, ScalerFormat_t *pScale)
{
    gp_memset((INT8S *)pScale, 0, sizeof(ScalerFormat_t));
	if (pMjpeg->jpeg_output_w == 0) {
	    pMjpeg->jpeg_output_w = pMjpeg->jpeg_output_buffer_w;
	}

	if (pMjpeg->jpeg_output_h == 0) {
	    pMjpeg->jpeg_output_h = pMjpeg->jpeg_output_buffer_h;
	}

	switch(pMjpeg->jpeg_yuv_mode)
	{
	case C_JPG_CTRL_YUV444:
	case C_JPG_CTRL_YUV422:
	case C_JPG_CTRL_YUV422V:
	case C_JPG_CTRL_YUV420:
	case C_JPG_CTRL_YUV411:
		pScale->input_format = C_SCALER_CTRL_IN_YUYV;
		break;

	default:
		while(1);
	}

	pScale->input_width = pMjpeg->jpeg_valid_w;
	pScale->input_height = pMjpeg->jpeg_valid_h;
	pScale->input_visible_width = pMjpeg->jpeg_valid_w;
	pScale->input_visible_height = pMjpeg->jpeg_valid_h;
	pScale->input_x_offset = 0;
	pScale->input_y_offset = 0;

	pScale->input_y_addr = g_jpeg_y_addr;
	pScale->input_u_addr = g_jpeg_cb_addr;
	pScale->input_v_addr = g_jpeg_cr_addr;

	pScale->output_format = pMjpeg->output_format;
	pScale->output_width = pMjpeg->jpeg_output_w;
	pScale->output_height = pMjpeg->jpeg_output_h;
	pScale->output_buf_width = pMjpeg->jpeg_output_buffer_w;
	pScale->output_buf_height = pMjpeg->jpeg_output_buffer_h;
	pScale->output_x_offset = 0;

	pScale->output_y_addr = pMjpeg->output_addr;
	pScale->output_u_addr = 0;
	pScale->output_v_addr = 0;

	pScale->fifo_mode = pMjpeg->scaler_fifo;
	pScale->scale_mode = pMjpeg->scaler_mode;
	pScale->digizoom_m = 0;
	pScale->digizoom_n = 0;
}

INT32S mjpeg_decode_and_scaler(mjpeg_info *pMjpeg)
{
	#define	C_READ_SIZE			40*1024

	INT8U  *p_vlc;
	INT8U  scaler_done;
	INT32S jpeg_status, scaler_status;
	INT32S fly_len, header_len, nRet;
	ScalerFormat_t scale;
	ScalerPara_t para;
	INT8U N = 0;

	nRet = mjpeg_seek_to_jpeg_header(pMjpeg->raw_data_addr);
	if(nRet < 0) {
		return -1;
	}
	pMjpeg->raw_data_size -= nRet;
	pMjpeg->raw_data_addr += nRet;

	jpeg_decode_init();
	gplib_jpeg_default_huffman_table_load();  //set defualt table after jpeg init
	jpeg_status = jpeg_decode_parse_header((INT8U *) pMjpeg->raw_data_addr, pMjpeg->raw_data_size);
	if(jpeg_status != JPEG_PARSE_OK) {
		RETURN(STATUS_FAIL);
}

	pMjpeg->jpeg_fifo = C_JPG_FIFO_32LINE;
	pMjpeg->scaler_fifo = C_SCALER_CTRL_FIFO_32LINE;
	pMjpeg->fifo_line = 32;
	pMjpeg->jpeg_yuv_mode = jpeg_decode_image_yuv_mode_get();
	pMjpeg->jpeg_valid_w = jpeg_decode_image_width_get();
	pMjpeg->jpeg_valid_h = jpeg_decode_image_height_get();
	pMjpeg->jpeg_extend_w = jpeg_decode_image_extended_width_get();
	pMjpeg->jpeg_extend_h = jpeg_decode_image_extended_height_get();

	switch(pMjpeg->jpeg_yuv_mode)
{
	case C_JPG_CTRL_YUV444:
	case C_JPG_CTRL_YUV422:
	case C_JPG_CTRL_YUV422V:
	case C_JPG_CTRL_YUV420:
	case C_JPG_CTRL_YUV411:
        drv_l1_jpeg_using_union_mode_enable();
	#if 1
		if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>2)) &&
			(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>2))) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV4);	// Use 1/4 scale-down mode
			pMjpeg->jpeg_valid_w >>= 2;
			pMjpeg->jpeg_valid_h >>= 2;
			pMjpeg->jpeg_extend_w >>= 2;
			pMjpeg->jpeg_extend_h >>= 2;
		} else if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>1)) &&
				(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>1))) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV2);	// Use 1/2 scale-down mode
			pMjpeg->jpeg_valid_w >>= 1;
			pMjpeg->jpeg_valid_h >>= 1;
			pMjpeg->jpeg_extend_w >>= 1;
			pMjpeg->jpeg_extend_h >>= 1;
		} else {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
		}
	#else
		drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
	#endif
		break;

	case C_JPG_CTRL_GRAYSCALE:
	case C_JPG_CTRL_YUV411V:
	case C_JPG_CTRL_YUV420H2:
	case C_JPG_CTRL_YUV420V2:
	case C_JPG_CTRL_YUV411H2:
	case C_JPG_CTRL_YUV411V2:
        drv_l1_jpeg_using_union_mode_disable();
	#if 1
		if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>1)) &&
			(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>1))) {
			jpeg_decode_level2_scaledown_enable();	// Use 1/2 scale-down mode
			pMjpeg->jpeg_valid_w >>= 1;
			pMjpeg->jpeg_valid_h >>= 1;
			pMjpeg->jpeg_extend_w >>= 1;
			pMjpeg->jpeg_extend_h >>= 1;
		}
    #endif
		break;
    }

	if(mjpeg_memory_alloc(pMjpeg) < 0) {
		RETURN(STATUS_FAIL);
    }

	if(jpeg_decode_output_set(g_jpeg_y_addr, g_jpeg_cb_addr, g_jpeg_cr_addr, pMjpeg->jpeg_fifo) < 0) {
		RETURN(STATUS_FAIL);
	}

	p_vlc = jpeg_decode_image_vlc_addr_get();
	header_len = ((INT32U) p_vlc) - pMjpeg->raw_data_addr;
	fly_len = pMjpeg->raw_data_size - header_len;

	// Set maximum VLC length to prevent JPEG from hangging too long
	if(jpeg_decode_vlc_maximum_length_set(fly_len) < 0) {
		RETURN(STATUS_FAIL);
}

	// Now start JPEG decoding on the fly
	nRet = ((INT32U)p_vlc) + C_READ_SIZE;
	nRet &= ~0x0F;
	fly_len = nRet - ((INT32U)p_vlc);	//16 align
	if(jpeg_decode_on_the_fly_start(p_vlc, fly_len) < 0) {
		RETURN(STATUS_FAIL);
	}

	// Setup Scaler
	scaler_done = 0;
	scaler_status = C_SCALER_STATUS_STOP;
	mjpeg_decode_set_scaler(pMjpeg, &scale);

	memset((void *)&para, 0x00, sizeof(para));
    para.boundary_color = pMjpeg->boundary_color;

  	while(1) {
  		jpeg_status = jpeg_decode_status_query(1);

		if(jpeg_status & C_JPG_STATUS_DECODE_DONE) {
			// Wait until scaler finish its job
			while(1) {
                if (scaler_status == C_SCALER_STATUS_BUSY){
                    scaler_status = drv_l2_scaler_wait_done(SCALER_0, &scale);
				}
				if(scaler_status == C_SCALER_STATUS_STOP) {
					scaler_status = drv_l2_scaler_trigger(SCALER_0, ENABLE, &scale, &para);
				} else if(scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
					scaler_status = drv_l2_scaler_retrigger(SCALER_0, &scale);
				} else if(scaler_status & C_SCALER_STATUS_TIMEOUT) {
					DEBUG_MSG("Scale1Timeout.\r\n");
					break;
				} else if(scaler_status & C_SCALER_STATUS_INIT_ERR) {
					DEBUG_MSG("Scale1InitErr.\r\n");
					break;
                } else {
				 	DEBUG_MSG("Scale1StatusErr = 0x%x\r\n", scaler_status);
				  	break;
                }

				if(scaler_status == C_SCALER_STATUS_STOP) {
					break;
				}
			}
			break;
		}

  		if(jpeg_status & C_JPG_STATUS_INPUT_EMPTY) {
			p_vlc += fly_len;
			nRet = ((INT32U)p_vlc) + C_READ_SIZE;
			nRet &= ~0x0F;
			fly_len = nRet - ((INT32U)p_vlc);

			// Now restart JPEG decoding on the fly
		  	if(jpeg_decode_on_the_fly_start(p_vlc, fly_len) < 0) {
		  		RETURN(STATUS_FAIL);
			}
		}

		if(jpeg_status & C_JPG_STATUS_OUTPUT_FULL) {
			// Start scaler to handle the full output FIFO now
		  	if(scaler_done == 0) {
				// scale1 run
				if (scaler_status == C_SCALER_STATUS_BUSY){
                    scaler_status = drv_l2_scaler_wait_done(SCALER_0, &scale);
				}
		  		if(scaler_status == C_SCALER_STATUS_STOP) {
					scaler_status = drv_l2_scaler_trigger(SCALER_0, ENABLE, &scale, &para);
				} else if(scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
					scaler_status = drv_l2_scaler_retrigger(SCALER_0, &scale);
				} else if(scaler_status & C_SCALER_STATUS_TIMEOUT) {
					DEBUG_MSG("Scale1Timeout\r\n");
					break;
				} else if(scaler_status & C_SCALER_STATUS_INIT_ERR) {
					DEBUG_MSG("Scale1InitErr\r\n");
					break;
                } else {
			  		DEBUG_MSG("Scale1StatusErr = 0x%x\r\n", scaler_status);
			  		break;
			  	}

				if(scaler_status == C_SCALER_STATUS_STOP) {
					scaler_done = 1;
				}
			}

	  		// Now restart JPEG to output to next FIFO
	  		if(jpeg_decode_output_restart())
	  		{
	  			DEBUG_MSG("Failed to call jpeg_decode_output_restart()\r\n");
	  			break;
            }
        }

		if(jpeg_status & C_JPG_STATUS_STOP) {
			DEBUG_MSG("JPEG is not started!\r\n");
			drv_l2_scaler_stop(SCALER_0);
			break;
		}

		if(jpeg_status & C_JPG_STATUS_TIMEOUT) {
			DEBUG_MSG("JPEG execution timeout!\r\n");
			drv_l2_scaler_stop(SCALER_0);
			break;
		}

		if(jpeg_status & C_JPG_STATUS_INIT_ERR) {
			DEBUG_MSG("JPEG init error!\r\n");
			drv_l2_scaler_stop(SCALER_0);
				break;
			}

		if(jpeg_status & C_JPG_STATUS_RST_VLC_DONE) {
			DEBUG_MSG("JPEG Restart marker number is incorrect!\r\n");
			drv_l2_scaler_stop(SCALER_0);
	  		break;
		}

		if(jpeg_status & C_JPG_STATUS_RST_MARKER_ERR) {
			DEBUG_MSG("JPEG Restart marker sequence error!\r\n");
			drv_l2_scaler_stop(SCALER_0);
	  		break;
	  	}
  	}

  	nRet = STATUS_OK;
Return:
	jpeg_decode_stop();
  	mjpeg_memory_free();
	return nRet;
}

//static OS_EVENT *dma_q;
static osMessageQId dma_q = 0;
static void *dma_q_buf[1];
static osMessageQDef_t dma_q_def = {1, sizeof(INT32U), dma_q_buf};

static INT32S move_data_start(INT32U source_addr, INT32U dst_addr, INT32U cblen)
{
	DMA_STRUCT dma_struct;

	if(dma_q == 0) {
		dma_q = osMessageCreate(&dma_q_def, NULL);
	}

	dma_struct.s_addr = (INT32U) source_addr;
	dma_struct.t_addr = (INT32U) dst_addr;
	dma_struct.width = DMA_DATA_WIDTH_4BYTE;		// DMA_DATA_WIDTH_1BYTE or DMA_DATA_WIDTH_2BYTE or DMA_DATA_WIDTH_4BYTE
	dma_struct.count = (INT32U) cblen >> 2;
	dma_struct.notify = NULL;
	dma_struct.timeout = 0;
	dma_struct.aes = 0;
	dma_struct.trigger = 0;
	OSQFlush(dma_q);
	return drv_l1_dma_transfer_with_queue(&dma_struct, dma_q);
}

static INT32S move_data_wait_done(void)
{
	INT8U err;
	INT32S status;
	osEvent event;

	event = osMessageGet(dma_q, osWaitForever);
    if (event.status != osEventMessage) {
        while(1);
    }
    if (event.value.v != C_DMA_STATUS_DONE) {
        while(1);
    }
    OSQFlush(dma_q);
    vQueueDelete(dma_q);
    dma_q = 0;

	return 0;
}

INT32S mjpeg_decode_and_scaler_to_dual_buf(mjpeg_info *pMjpeg, INT32U dest_addr)
{
	#define	C_READ_SIZE			40*1024

	INT8U  *p_vlc;
	INT8U  scaler_done;
	INT32S jpeg_status, scaler_status;
	INT32S fly_len, header_len, nRet;
	ScalerFormat_t scale;
	ScalerPara_t para;

	INT32U jpeg_cnt;
	INT32U jpeg_size;
	INT32U src_addr;

	nRet = mjpeg_seek_to_jpeg_header(pMjpeg->raw_data_addr);
	if(nRet < 0) {
		return -1;
	}
	pMjpeg->raw_data_size -= nRet;
	pMjpeg->raw_data_addr += nRet;

	jpeg_decode_init();
	gplib_jpeg_default_huffman_table_load();  //set defualt table after jpeg init
	jpeg_status = jpeg_decode_parse_header((INT8U *) pMjpeg->raw_data_addr, pMjpeg->raw_data_size);
	if(jpeg_status != JPEG_PARSE_OK) {
		RETURN(STATUS_FAIL);
	}

	pMjpeg->jpeg_fifo = C_JPG_FIFO_32LINE;
	pMjpeg->scaler_fifo = C_SCALER_CTRL_FIFO_32LINE;
	pMjpeg->fifo_line = 32;
	pMjpeg->jpeg_yuv_mode = jpeg_decode_image_yuv_mode_get();
	pMjpeg->jpeg_valid_w = jpeg_decode_image_width_get();
	pMjpeg->jpeg_valid_h = jpeg_decode_image_height_get();
	pMjpeg->jpeg_extend_w = jpeg_decode_image_extended_width_get();
	pMjpeg->jpeg_extend_h = jpeg_decode_image_extended_height_get();

	switch(pMjpeg->jpeg_yuv_mode)
	{
	case C_JPG_CTRL_YUV444:
	case C_JPG_CTRL_YUV422:
	case C_JPG_CTRL_YUV422V:
	case C_JPG_CTRL_YUV420:
	case C_JPG_CTRL_YUV411:
	#if 0
		if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>2)) &&
			(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>2))) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV4);	// Use 1/4 scale-down mode
			pMjpeg->jpeg_valid_w >>= 2;
			pMjpeg->jpeg_valid_h >>= 2;
			pMjpeg->jpeg_extend_w >>= 2;
			pMjpeg->jpeg_extend_h >>= 2;
		} else if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>1)) &&
				(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>1))) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV2);	// Use 1/2 scale-down mode
			pMjpeg->jpeg_valid_w >>= 1;
			pMjpeg->jpeg_valid_h >>= 1;
			pMjpeg->jpeg_extend_w >>= 1;
			pMjpeg->jpeg_extend_h >>= 1;
		} else {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
		}
	#else
		drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
	#endif
		break;

	case C_JPG_CTRL_GRAYSCALE:
	case C_JPG_CTRL_YUV411V:
	case C_JPG_CTRL_YUV420H2:
	case C_JPG_CTRL_YUV420V2:
	case C_JPG_CTRL_YUV411H2:
	case C_JPG_CTRL_YUV411V2:
		if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>1)) &&
			(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>1))) {
			jpeg_decode_level2_scaledown_enable();	// Use 1/2 scale-down mode
			pMjpeg->jpeg_valid_w >>= 1;
			pMjpeg->jpeg_valid_h >>= 1;
			pMjpeg->jpeg_extend_w >>= 1;
			pMjpeg->jpeg_extend_h >>= 1;
		}
		break;
	}

	if(mjpeg_memory_alloc(pMjpeg) < 0) {
		RETURN(STATUS_FAIL);
	}

	if(jpeg_decode_output_set(g_jpeg_y_addr, g_jpeg_cb_addr, g_jpeg_cr_addr, pMjpeg->jpeg_fifo) < 0) {
		RETURN(STATUS_FAIL);
	}

	p_vlc = jpeg_decode_image_vlc_addr_get();
	header_len = ((INT32U) p_vlc) - pMjpeg->raw_data_addr;
	fly_len = pMjpeg->raw_data_size - header_len;

	// Set maximum VLC length to prevent JPEG from hangging too long
	if(jpeg_decode_vlc_maximum_length_set(fly_len) < 0) {
		RETURN(STATUS_FAIL);
	}

	// Now start JPEG decoding on the fly
	nRet = ((INT32U)p_vlc) + C_READ_SIZE;
	nRet &= ~0x0F;
	fly_len = nRet - ((INT32U)p_vlc);	//16 align
	if(jpeg_decode_on_the_fly_start(p_vlc, fly_len) < 0) {
		RETURN(STATUS_FAIL);
	}

	// Setup Scaler
	g_fifo_size >>= 1;
	jpeg_cnt = 0;
	jpeg_size = pMjpeg->jpeg_extend_w * pMjpeg->jpeg_extend_h * 2;
	scaler_done = 0;
	scaler_status = C_SCALER_STATUS_STOP;
	mjpeg_decode_set_scaler(pMjpeg, &scale);

	memset((void *)&para, 0x00, sizeof(para));
    para.boundary_color = pMjpeg->boundary_color;

  	while(1) {
  		jpeg_status = jpeg_decode_status_query(1);

		if(jpeg_status & C_JPG_STATUS_DECODE_DONE) {
			// Wait until scaler finish its job
		  	while (1) {
		  		if(scaler_status == C_SCALER_STATUS_STOP) {
					if(jpeg_cnt & 0x01) {
						src_addr = g_jpeg_y_addr + g_fifo_size;
					}
					else {
						src_addr = g_jpeg_y_addr;
					}

					if(jpeg_size > g_fifo_size) {
						nRet = move_data_start(src_addr, dest_addr, g_fifo_size);
						jpeg_size -= g_fifo_size;
					}
					else {
						nRet = move_data_start(src_addr, dest_addr, jpeg_size);
						jpeg_size = 0;
					}

					scaler_status = drv_l2_scaler_trigger(SCALER_0, ENABLE, &scale, &para);
					if(nRet >= 0) {
						move_data_wait_done();
						dest_addr += g_fifo_size;
						jpeg_cnt++;
					}
				} else if(scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
					if(jpeg_cnt & 0x01) {
						src_addr = g_jpeg_y_addr + g_fifo_size;
					}
					else {
						src_addr = g_jpeg_y_addr;
					}

					if(jpeg_size > g_fifo_size) {
						nRet = move_data_start(src_addr, dest_addr, g_fifo_size);
						jpeg_size -= g_fifo_size;
					}
					else {
						nRet = move_data_start(src_addr, dest_addr, jpeg_size);
						jpeg_size = 0;
					}

					scaler_status = drv_l2_scaler_retrigger(SCALER_0, &scale);
					if(nRet >= 0) {
						move_data_wait_done();
						dest_addr += g_fifo_size;
						jpeg_cnt++;
					}
				} else if(scaler_status & C_SCALER_STATUS_TIMEOUT) {
					DEBUG_MSG("Scale1Timeout.\r\n");
					break;
				} else if(scaler_status & C_SCALER_STATUS_INIT_ERR) {
					DEBUG_MSG("Scale1InitErr.\r\n");
					break;
		  		} else {
				 	DEBUG_MSG("Scale1StatusErr = 0x%x\r\n", scaler_status);
			  		break;
			  	}

				if(scaler_status == C_SCALER_STATUS_STOP) {
					break;
		  	}
			}
			break;
		}

  		if(jpeg_status & C_JPG_STATUS_INPUT_EMPTY) {
			p_vlc += fly_len;
			nRet = ((INT32U)p_vlc) + C_READ_SIZE;
			nRet &= ~0x0F;
			fly_len = nRet - ((INT32U)p_vlc);

			// Now restart JPEG decoding on the fly
		  	if(jpeg_decode_on_the_fly_start(p_vlc, fly_len) < 0) {
		  		RETURN(STATUS_FAIL);
		  	}
		}

		if(jpeg_status & C_JPG_STATUS_OUTPUT_FULL) {
			// Start scaler to handle the full output FIFO now
		  	if(scaler_done == 0) {
				// scale1 run
			  	if(scaler_status == C_SCALER_STATUS_STOP) {
		  			if(jpeg_cnt & 0x01) {
						src_addr = g_jpeg_y_addr + g_fifo_size;
					}
					else {
						src_addr = g_jpeg_y_addr;
					}

					if(jpeg_size > g_fifo_size) {
						nRet = move_data_start(src_addr, dest_addr, g_fifo_size);
						jpeg_size -= g_fifo_size;
					}
					else {
						nRet = move_data_start(src_addr, dest_addr, jpeg_size);
						jpeg_size = 0;
					}

					scaler_status = drv_l2_scaler_trigger(SCALER_0, ENABLE, &scale, &para);
					if(nRet >= 0) {
						move_data_wait_done();
						dest_addr += g_fifo_size;
						jpeg_cnt++;
					}
				} else if(scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
					if(jpeg_cnt & 0x01) {
						src_addr = g_jpeg_y_addr + g_fifo_size;
					}
					else {
						src_addr = g_jpeg_y_addr;
					}

					if(jpeg_size > g_fifo_size) {
						nRet = move_data_start(src_addr, dest_addr, g_fifo_size);
						jpeg_size -= g_fifo_size;
					}
					else {
						nRet = move_data_start(src_addr, dest_addr, jpeg_size);
						jpeg_size = 0;
					}

					scaler_status = drv_l2_scaler_retrigger(SCALER_0, &scale);
					if(nRet >= 0) {
						move_data_wait_done();
						dest_addr += g_fifo_size;
						jpeg_cnt++;
					}
				} else if(scaler_status & C_SCALER_STATUS_TIMEOUT) {
					DEBUG_MSG("Scale1Timeout\r\n");
					break;
				} else if(scaler_status & C_SCALER_STATUS_INIT_ERR) {
					DEBUG_MSG("Scale1InitErr\r\n");
					break;
			  	} else {
			  		DEBUG_MSG("Scale1StatusErr = 0x%x\r\n", scaler_status);
			  		break;
			  	}

				if(scaler_status == C_SCALER_STATUS_STOP) {
					scaler_done = 1;
			}
			}

	  		// Now restart JPEG to output to next FIFO
	  		if(jpeg_decode_output_restart())
	  		{
	  			DEBUG_MSG("Failed to call jpeg_decode_output_restart()\r\n");
	  			break;
	  		}
		}

		if(jpeg_status & C_JPG_STATUS_STOP) {
			DEBUG_MSG("JPEG is not started!\r\n");
			break;
		}

		if(jpeg_status & C_JPG_STATUS_TIMEOUT) {
			DEBUG_MSG("JPEG execution timeout!\r\n");
			break;
		}

		if(jpeg_status & C_JPG_STATUS_INIT_ERR) {
			DEBUG_MSG("JPEG init error!\r\n");
			break;
		}

		if(jpeg_status & C_JPG_STATUS_RST_VLC_DONE) {
			DEBUG_MSG("JPEG Restart marker number is incorrect!\r\n");
			break;
		}

		if(jpeg_status & C_JPG_STATUS_RST_MARKER_ERR) {
			DEBUG_MSG("JPEG Restart marker sequence error!\r\n");
			break;
		}
  	}

  	nRet = STATUS_OK;
Return:
	jpeg_decode_stop();
  	mjpeg_memory_free();
	return nRet;
}

#if 0
static void wrap_set_parameter(void)
{
	drv_l1_wrap_filter_enable(WRAP_CSIMUX, 0);
	drv_l1_wrap_path_set(WRAP_CSIMUX, 0, 0);

	drv_l1_wrap_addr_set(WRAP_CSI2SCA, DUMMY_BUFFER_ADDRS);
	drv_l1_wrap_path_set(WRAP_CSI2SCA, 1, 1);	//path-sr enable, path-o enable
	drv_l1_wrap_filter_enable(WRAP_CSI2SCA, 1);
	drv_l1_wrap_filter_flush(WRAP_CSI2SCA);
}

static void conv422to420_isr_handle(INT32U event)
{
	if(event & CONV422_CHANGE_FIFO) {
		if(event & CONV422_IRQ_BUF_A) {
			//DEBUG_MSG("CONV422_IRQ_BUF_A\r\n");
		}
		else {
			//DEBUG_MSG("CONV422_IRQ_BUF_B\r\n");
		}

		if(event & CONV422_FRAME_END) {
			//DEBUG_MSG("CONV422_FRAME_END\r\n");
		}
	}
}

static void conv422to420_set_parameter(INT16U w, INT16U h, INT32U out_fmt, INT32U output_buf)
{
	drv_l1_conv422_register_callback(conv422to420_isr_handle);
	drv_l1_conv422_init();
	drv_l1_conv422_input_pixels_set(w, h);
	drv_l1_conv422_input_fmt_set(CONV422_FMT_YUYV);
	drv_l1_conv422_fifo_line_set(h);
	drv_l1_conv422_output_A_addr_set(output_buf);
	drv_l1_conv422_output_B_addr_set(output_buf);

	if(out_fmt == C_JPG_CTRL_GP420) {
		drv_l1_conv422_output_format_set(CONV422_FMT_420);
	}
	else {
		drv_l1_conv422_output_format_set(CONV422_FMT_422);
	}

	drv_l1_conv422_mode_set(CONV422_FIFO_MODE);
	drv_l1_conv422_bypass_enable(DISABLE);
	drv_l1_conv422_irq_enable(ENABLE);
	drv_l1_conv422_reset();
}

static void scaler_isr_handle(INT32U scaler0_event, INT32U scaler1_event)
{
	//DEBUG_MSG("Scaler[1,0] 0x%x, 0x%x\r\n", scaler1_event, scaler0_event);
}

static INT32S scaler_set_parameter(mjpeg_info *pMjpeg)
{
	INT32U xfactor, yfactor;
	SCALER_MAS scaler1_mas;
	SCALER_MAS scaler0_mas;

	gp_memset((INT8S*)&scaler1_mas, 0x00, sizeof(SCALER_MAS));
	gp_memset((INT8S*)&scaler0_mas, 0x00, sizeof(SCALER_MAS));

	drv_l1_scaler_isr_callback_set(scaler_isr_handle);

	// scaler 1 for 720p size, fifo mode
	drv_l1_scaler_init(SCALER_1);

	scaler1_mas.mas_0 = MAS_EN_READ;
	scaler1_mas.mas_3 = MAS_EN_WRITE;
	drv_l1_scaler_mas_set(SCALER_1, &scaler1_mas);

	drv_l1_scaler_input_pixels_set(SCALER_1, pMjpeg->jpeg_extend_w, pMjpeg->jpeg_extend_h);
	drv_l1_scaler_input_visible_pixels_set(SCALER_1, pMjpeg->jpeg_valid_w, pMjpeg->jpeg_valid_h);
	drv_l1_scaler_input_offset_set(SCALER_1, 0, 0);

	xfactor = yfactor = 1 << 16;
	drv_l1_scaler_output_pixels_set(SCALER_1, xfactor, yfactor, pMjpeg->jpeg_valid_w, pMjpeg->jpeg_valid_h);
	drv_l1_scaler_output_offset_set(SCALER_1, 0);

	drv_l1_scaler_input_A_addr_set(SCALER_1, g_jpeg_y_addr, g_jpeg_cb_addr, g_jpeg_cr_addr);
	drv_l1_scaler_output_addr_set(SCALER_1, DUMMY_BUFFER_ADDRS, 0, 0);
	drv_l1_scaler_input_format_set(SCALER_1, C_SCALER_CTRL_IN_YUYV);
	drv_l1_scaler_output_format_set(SCALER_1, C_SCALER_CTRL_OUT_YUYV);
	drv_l1_scaler_fifo_line_set(SCALER_1, pMjpeg->scaler_fifo);

	drv_l1_scaler_out_of_boundary_mode_set(SCALER_1, 1);
	drv_l1_scaler_out_of_boundary_color_set(SCALER_1, 0x008080);

	// scaler 0 for tft size, frame mode
	drv_l1_scaler_init(SCALER_0);

	scaler0_mas.mas_0 = MAS_EN_WRITE;
	scaler0_mas.mas_2 = MAS_EN_READ;
	drv_l1_scaler_mas_set(SCALER_0, &scaler0_mas);

	drv_l1_scaler_input_pixels_set(SCALER_0, pMjpeg->jpeg_valid_w, pMjpeg->jpeg_valid_h);
	drv_l1_scaler_input_visible_pixels_set(SCALER_0, pMjpeg->jpeg_valid_w, pMjpeg->jpeg_valid_h);
	drv_l1_scaler_input_offset_set(SCALER_0, 0, 0);

	xfactor = (pMjpeg->jpeg_valid_w << 16) / pMjpeg->jpeg_output_w;
	yfactor = (pMjpeg->jpeg_valid_h << 16) / pMjpeg->jpeg_output_h;
	drv_l1_scaler_output_pixels_set(SCALER_0, xfactor, yfactor, pMjpeg->jpeg_output_w, pMjpeg->jpeg_output_h);
	drv_l1_scaler_output_offset_set(SCALER_0, 0);

	drv_l1_scaler_input_A_addr_set(SCALER_0, DUMMY_BUFFER_ADDRS, 0, 0);
	drv_l1_scaler_output_addr_set(SCALER_0, pMjpeg->output_addr, 0, 0);
	drv_l1_scaler_input_format_set(SCALER_0, C_SCALER_CTRL_IN_YUYV);
	drv_l1_scaler_output_format_set(SCALER_0, C_SCALER_CTRL_OUT_YUYV); //C_SCALER_CTRL_OUT_RGB565
	drv_l1_scaler_fifo_line_set(SCALER_0, C_SCALER_CTRL_IN_FIFO_DISABLE);

	drv_l1_scaler_out_of_boundary_mode_set(SCALER_0, 1);
	drv_l1_scaler_out_of_boundary_color_set(SCALER_0, 0x008080);

	return 0;
}

INT32S mjpeg_decode_and_scaler_to_dual_buf_by_wrapper(mjpeg_info *pMjpeg, INT32U GP420_fmt_en, INT32U dst_buf)
{
	#define	C_READ_SIZE			40*1024

	INT8U *p_vlc;
	INT8U  scaler_done;
	INT8U  scaler0_start;
	INT32S jpeg_status;
	INT32S scaler0_status;
	INT32S scaler1_status;
	INT32S fly_len, header_len, nRet;

	nRet = mjpeg_seek_to_jpeg_header(pMjpeg->raw_data_addr);
	if(nRet < 0) {
		return -1;
	}
	pMjpeg->raw_data_size -= nRet;
	pMjpeg->raw_data_addr += nRet;

	jpeg_decode_init();
	gplib_jpeg_default_huffman_table_load();  //set defualt table after jpeg init
	jpeg_status = jpeg_decode_parse_header((INT8U *) pMjpeg->raw_data_addr, pMjpeg->raw_data_size);
	if(jpeg_status != JPEG_PARSE_OK) {
		RETURN(STATUS_FAIL);
	}

	pMjpeg->jpeg_fifo = C_JPG_FIFO_32LINE;
	pMjpeg->scaler_fifo = C_SCALER_CTRL_FIFO_32LINE;
	pMjpeg->fifo_line = 32;
	pMjpeg->jpeg_yuv_mode = jpeg_decode_image_yuv_mode_get();
	pMjpeg->jpeg_valid_w = jpeg_decode_image_width_get();
	pMjpeg->jpeg_valid_h = jpeg_decode_image_height_get();
	pMjpeg->jpeg_extend_w = jpeg_decode_image_extended_width_get();
	pMjpeg->jpeg_extend_h = jpeg_decode_image_extended_height_get();

	switch(pMjpeg->jpeg_yuv_mode)
	{
	case C_JPG_CTRL_YUV444:
	case C_JPG_CTRL_YUV422:
	case C_JPG_CTRL_YUV422V:
	case C_JPG_CTRL_YUV420:
	case C_JPG_CTRL_YUV411:
	#if 0
		if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>2)) &&
			(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>2))) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV4);	// Use 1/4 scale-down mode
			pMjpeg->jpeg_valid_w >>= 2;
			pMjpeg->jpeg_valid_h >>= 2;
			pMjpeg->jpeg_extend_w >>= 2;
			pMjpeg->jpeg_extend_h >>= 2;
		} else if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>1)) &&
				(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>1))) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV2);	// Use 1/2 scale-down mode
			pMjpeg->jpeg_valid_w >>= 1;
			pMjpeg->jpeg_valid_h >>= 1;
			pMjpeg->jpeg_extend_w >>= 1;
			pMjpeg->jpeg_extend_h >>= 1;
		} else {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
	}
	#else
		drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
	#endif
		break;

	case C_JPG_CTRL_GP420:
	#if 0
		if((pMjpeg->jpeg_output_w <= ((pMjpeg->jpeg_valid_w << 1) / 3)) &&
			(pMjpeg->jpeg_output_h <= ((pMjpeg->jpeg_valid_h << 1) / 3))) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_HV_DIV23);	// Use 2/3 scale-down mode
			pMjpeg->jpeg_valid_w = (pMjpeg->jpeg_valid_w << 1) / 3;
			pMjpeg->jpeg_valid_h = (pMjpeg->jpeg_valid_h << 1) / 3;
			pMjpeg->jpeg_extend_w = (pMjpeg->jpeg_extend_w << 1) / 3;
			pMjpeg->jpeg_extend_h = (pMjpeg->jpeg_extend_h << 1) / 3;
		} else if(pMjpeg->jpeg_output_w <= ((pMjpeg->jpeg_valid_w << 1) / 3)) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_H_DIV23);	// Use 2/3 scale-down mode
			pMjpeg->jpeg_valid_w = (pMjpeg->jpeg_valid_w << 1) / 3;
			pMjpeg->jpeg_extend_w = (pMjpeg->jpeg_extend_w << 1) / 3;
		} else if(pMjpeg->jpeg_output_h <= ((pMjpeg->jpeg_valid_h << 1) / 3)) {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_V_DIV23);	// Use 2/3 scale-down mode
			pMjpeg->jpeg_valid_h = (pMjpeg->jpeg_valid_h << 1) / 3;
			pMjpeg->jpeg_extend_h = (pMjpeg->jpeg_extend_h << 1) / 3;
		} else {
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
	}
	#else
		drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
	#endif
		break;

	case C_JPG_CTRL_GRAYSCALE:
	case C_JPG_CTRL_YUV411V:
	case C_JPG_CTRL_YUV420H2:
	case C_JPG_CTRL_YUV420V2:
	case C_JPG_CTRL_YUV411H2:
	case C_JPG_CTRL_YUV411V2:
		if((pMjpeg->jpeg_output_w <= (pMjpeg->jpeg_valid_w>>1)) &&
			(pMjpeg->jpeg_output_h <= (pMjpeg->jpeg_valid_h>>1))) {
			jpeg_decode_level2_scaledown_enable();	// Use 1/2 scale-down mode
			pMjpeg->jpeg_valid_w >>= 1;
			pMjpeg->jpeg_valid_h >>= 1;
			pMjpeg->jpeg_extend_w >>= 1;
			pMjpeg->jpeg_extend_h >>= 1;
		}
			break;
		}

	if(mjpeg_memory_alloc(pMjpeg) < 0) {
			RETURN(STATUS_FAIL);
		}

	if(jpeg_decode_output_set(g_jpeg_y_addr, g_jpeg_cb_addr, g_jpeg_cr_addr, pMjpeg->jpeg_fifo) < 0) {
		RETURN(STATUS_FAIL);
	}

	p_vlc = jpeg_decode_image_vlc_addr_get();
	header_len = ((INT32U) p_vlc) - pMjpeg->raw_data_addr;
	fly_len = pMjpeg->raw_data_size - header_len;

	// Set maximum VLC length to prevent JPEG from hangging too long
	if(jpeg_decode_vlc_maximum_length_set(fly_len) < 0) {
		RETURN(STATUS_FAIL);
	}

	// Now start JPEG decoding on the fly
	nRet = ((INT32U)p_vlc) + C_READ_SIZE;
	nRet &= ~0x0F;
	fly_len = nRet - ((INT32U)p_vlc);	//16 align
	if(jpeg_decode_on_the_fly_start(p_vlc, fly_len) < 0) {
		RETURN(STATUS_FAIL);
	}

	// Setup Scaler
	scaler_done = 0;
	scaler0_start = 0;
	scaler0_status = C_SCALER_STATUS_DONE;
	scaler1_status = C_SCALER_STATUS_DONE;

	// scaler lock
	drv_l1_scaler_lock(SCALER_1);
	drv_l1_scaler_lock(SCALER_0);

	wrap_set_parameter();

	if(GP420_fmt_en) {
		conv422to420_set_parameter(pMjpeg->jpeg_valid_w, pMjpeg->jpeg_valid_h, C_JPG_CTRL_GP420, dst_buf);
	}
	else {
		conv422to420_set_parameter(pMjpeg->jpeg_valid_w, pMjpeg->jpeg_valid_h, C_JPG_CTRL_YUV422, dst_buf);
	}

	scaler_set_parameter(pMjpeg);

  	while(1) {
  		jpeg_status = jpeg_decode_status_query(1);

		if(jpeg_status & C_JPG_STATUS_DECODE_DONE) {
			// Wait until scaler finish its job
	while (1) {
				// scaler1 start
				scaler1_status = drv_l1_scaler_wait_idle(SCALER_1);
		  		if(scaler1_status == C_SCALER_STATUS_STOP) {
		  			drv_l1_scaler_start(SCALER_1, ENABLE);
		  		}
		  		else if(scaler1_status & C_SCALER_STATUS_INPUT_EMPTY) {
			  		drv_l1_scaler_restart(SCALER_1);
			  	}
		  		else if(scaler1_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
		  			DEBUG_MSG("Scaler1 failed to finish its job\r\n");
					break;
		  		}

			  	// scaler0 start
			  	if(scaler0_start == 0) {
			  		scaler0_status = drv_l1_scaler_wait_idle(SCALER_0);
			  		if(scaler0_status == C_SCALER_STATUS_STOP) {
			  			drv_l1_scaler_start(SCALER_0, ENABLE);
			  			scaler0_start = 1;
			  		}
			  		else {
			  			DEBUG_MSG("Scaler0 failed to finish its job\r\n");
						break;
			  		}
		  		}

				if(scaler1_status == C_SCALER_STATUS_DONE) {
					drv_l1_scaler_stop(SCALER_1);

					scaler0_status = drv_l1_scaler_wait_idle(SCALER_0);
					if(scaler0_status == C_SCALER_STATUS_DONE) {
						drv_l1_scaler_stop(SCALER_0);
					}
					break;
				}
			}
			break;
		}

  		if(jpeg_status & C_JPG_STATUS_INPUT_EMPTY) {
			p_vlc += fly_len;
			nRet = ((INT32U)p_vlc) + C_READ_SIZE;
			nRet &= ~0x0F;
			fly_len = nRet - ((INT32U)p_vlc);

			// Now restart JPEG decoding on the fly
		  	if(jpeg_decode_on_the_fly_start(p_vlc, fly_len) < 0) {
			RETURN(STATUS_FAIL);
		}
	}

		if(jpeg_status & C_JPG_STATUS_OUTPUT_FULL) {
			// Start scaler to handle the full output FIFO now
		  	if(scaler_done == 0) {
		  		// scaler1 start
		  		scaler1_status = drv_l1_scaler_wait_idle(SCALER_1);
		  		if(scaler1_status == C_SCALER_STATUS_STOP) {
		  			drv_l1_scaler_start(SCALER_1, ENABLE);
		  		}
		  		else if(scaler1_status & C_SCALER_STATUS_INPUT_EMPTY) {
			  		drv_l1_scaler_restart(SCALER_1);
			  	}
		  		else if(scaler1_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
		  			DEBUG_MSG("Scaler1 failed to finish its job\r\n");
					break;
		  		}

			  	// scaler0 start
			  	if(scaler0_start == 0) {
			  		scaler0_status = drv_l1_scaler_wait_idle(SCALER_0);
			  		if(scaler0_status == C_SCALER_STATUS_STOP) {
			  			drv_l1_scaler_start(SCALER_0, ENABLE);
			  			scaler0_start = 1;
			  		}
			  		else {
			  			DEBUG_MSG("Scaler0 failed to finish its job\r\n");
						break;
			  		}
		  		}

				if(scaler1_status == C_SCALER_STATUS_DONE) {
					scaler_done = 1;
				}
			}

	  		// Now restart JPEG to output to next FIFO
	  		if(jpeg_decode_output_restart())
	  		{
	  			DEBUG_MSG("Failed to call jpeg_decode_output_restart()\r\n");
	  			break;
	  		}
		}

		if(jpeg_status & C_JPG_STATUS_STOP) {
			DEBUG_MSG("JPEG is not started!\r\n");
			break;
	}

		if(jpeg_status & C_JPG_STATUS_TIMEOUT) {
			DEBUG_MSG("JPEG execution timeout!\r\n");
			break;
	}

		if(jpeg_status & C_JPG_STATUS_INIT_ERR) {
			DEBUG_MSG("JPEG init error!\r\n");
			break;
		}

		if(jpeg_status & C_JPG_STATUS_RST_VLC_DONE) {
			DEBUG_MSG("JPEG Restart marker number is incorrect!\r\n");
			break;
		}

		if(jpeg_status & C_JPG_STATUS_RST_MARKER_ERR) {
			DEBUG_MSG("JPEG Restart marker sequence error!\r\n");
			break;
		}
  	}

  	nRet = STATUS_OK;
Return:
	jpeg_decode_stop();
  	mjpeg_memory_free();

	drv_l1_scaler_stop(SCALER_1);
	drv_l2_scaler_stop(SCALER_0);
	drv_l1_scaler_unlock(SCALER_1);
	drv_l1_scaler_unlock(SCALER_0);

	drv_l1_wrap_filter_enable(WRAP_CSI2SCA, 0);
	drv_l1_conv422_init();
	return nRet;
}
#endif
INT32S mjpeg_decode_without_scaler(mjpeg_info *pMjpeg)
{
	INT8U *p_vlc;
	INT32U fly_len, header_len;
	INT32S nRet;

	nRet = mjpeg_seek_to_jpeg_header(pMjpeg->raw_data_addr);
	if(nRet < 0) return -1;
	pMjpeg->raw_data_size -= nRet;
	pMjpeg->raw_data_addr += nRet;

	jpeg_decode_init();
	gplib_jpeg_default_huffman_table_load();  //set defualt table after jpeg init
	nRet = jpeg_decode_parse_header((INT8U *) pMjpeg->raw_data_addr, pMjpeg->raw_data_size);
	if(nRet != JPEG_PARSE_OK) {
		RETURN(STATUS_FAIL);
	}

	if(jpeg_decode_output_set(pMjpeg->output_addr, 0, 0, C_JPG_FIFO_DISABLE) < 0) {
		DEBUG_MSG("Calling to jpeg_vlc_addr_set() failed\r\n");
		RETURN(STATUS_FAIL);
	}
    drv_l1_jpeg_using_union_mode_enable();

	drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_NO_SCALE_DOWN);
	p_vlc = jpeg_decode_image_vlc_addr_get();
	header_len = ((INT32U) p_vlc) - pMjpeg->raw_data_addr;
	fly_len = pMjpeg->raw_data_size - header_len;

	if (drv_l1_jpeg_vlc_addr_set((INT32U) p_vlc)) {
		DEBUG_MSG("Calling to jpeg_vlc_addr_set() failed\r\n");
		RETURN(STATUS_FAIL);
	}

	if (drv_l1_jpeg_vlc_maximum_length_set(fly_len)) {
		DEBUG_MSG("Calling to jpeg_vlc_maximum_length_set() failed\r\n");
		RETURN(STATUS_FAIL);
	}

	if (drv_l1_jpeg_decompression_start(NULL)) {
		DEBUG_MSG("Calling to jpeg_decompression_start() failed\r\n");
		RETURN(STATUS_FAIL);
	}

	while (1) {
		nRet = drv_l1_jpeg_status_polling(TRUE);
		if(nRet & C_JPG_STATUS_DECODE_DONE) {
			break;
		}

		if(nRet & (C_JPG_STATUS_STOP|C_JPG_STATUS_TIMEOUT|C_JPG_STATUS_INIT_ERR)) {
			RETURN(STATUS_FAIL);
		}
	}

	nRet = STATUS_OK;
Return:
	drv_l1_jpeg_stop();
	return nRet;
}

INT32S mjpeg_decode_stop_all(INT32U flag)
{
	jpeg_decode_stop();
	drv_l1_scaler_stop(SCALER_0);

	if(flag) {
		mjpeg_memory_free();
	}

	return STATUS_OK;
}
#endif //MJPEG_DECODE_ENABLE
///////////////////////////////////////////////////////////////////////////////////
#if MPEG4_DECODE_ENABLE == 1
//paser mpeg4 bit stream infomation
typedef struct
{
	INT8U is_visual_object_start_code;
	INT8U is_vo_start_code;
    INT8U profile_and_level_indication;
    INT8U visual_object_verid;

    INT8U video_object_type_indication;
    INT8U video_object_layer_verid;
   	INT8U visual_object_type;
    INT8U is_object_layer_identifier;

	INT8U aspect_ratio_info;
	INT8U vol_control_par;
	INT8U chroma_format;
	INT8U low_delay;

	INT8U vbv_par;
	INT8U fixed_vop_rate;
	INT8U video_object_layer_shape;
    INT8U vopTimeIncResLen;

    INT32U vop_time_increment_resolution;
    INT16U par_width;
	INT16U par_height;
    INT16U video_object_layer_width;
    INT16U video_object_layer_height;

    INT8U interlaced;
    INT8U obmc_disable;
    INT8U sprite_enable;
    INT8U not_8_bit;

    INT8U quant_bits;
    INT8U quant_type;
    INT8U quarter_sample;
    INT8U complexity_estimation_disable;

	INT8U resync_marker_disable;
	INT8U data_partitioned;
	INT8U reversible_vlc;
	INT8U newpred_enable;

	INT8U reduced_resolution_enable;
	INT8U scalability;
	INT8U vop_quant;
	INT8U time_increment;

	INT32U vop_id;
	INT32U vop_id_for_prediction;
	CHAR   user_data[256];
} Mpeg4ParaStruct;

static INT8U  g_bs_pos;
static INT8U  *g_bs_ptr;
static INT32U g_bs_buffer[2];
static Mpeg4ParaStruct *p_mp4_para, mp4_para;

static void bs_skip(INT8U bits);
static void bs_byte_align(void);
static INT32U bs_show_bits(INT8U bits);
static INT32U bs_get_bits(INT8U bits);

static void bs_skip(INT8U bits)
{
	g_bs_pos += bits;
	if(g_bs_pos >= 32) {
		g_bs_buffer[0] = g_bs_buffer[1];
		g_bs_buffer[1] = (INT32U)(*g_bs_ptr++ << 24);
		g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 16);
		g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 8);
		g_bs_buffer[1] |= *g_bs_ptr++;
		g_bs_pos -= 32;
	}
}

static void bs_byte_align(void)
{
	INT32U remainder = g_bs_pos % 8;
	if (remainder)
		bs_skip(8 - remainder);
}

static INT32U bs_show_bits(INT8U bits)
{
	INT32S nbit = (bits + g_bs_pos) - 32;
	if (nbit > 0)
		return ((g_bs_buffer[0] & (0xffffffff >> g_bs_pos)) << nbit) | (g_bs_buffer[1] >> (32 - nbit));
	else
		return (g_bs_buffer[0] & (0xffffffff >> g_bs_pos)) >> (32 - g_bs_pos - bits);
}

static INT32U bs_get_bits(INT8U bits)
{
	INT32U ret = bs_show_bits(bits);
	bs_skip(bits);
	return ret;
}

void read_vol_complexity_estimation_header(void)
{
	INT8U method;
	method = bs_get_bits(2);	/* estimation_method */
	if (method == 0 ||method == 1)
	{
		if(!bs_get_bits(1))		/* shape_complexity_estimation_disable */
		{
			bs_skip(1);		/* opaque */
			bs_skip(1);		/* transparent */
			bs_skip(1);		/* intra_cae */
			bs_skip(1);		/* inter_cae */
			bs_skip(1);		/* no_update */
			bs_skip(1);		/* upsampling */
		}

		if (!bs_get_bits(1))	/* texture_complexity_estimation_set_1_disable */
		{
			bs_skip(1);		/* intra_blocks */
			bs_skip(1);		/* inter_blocks */
			bs_skip(1);		/* inter4v_blocks */
			bs_skip(1);		/* not_coded_blocks */
		}
	}

	bs_skip(1);//READ_MARKER();
	if (!bs_get_bits(1))		/* texture_complexity_estimation_set_2_disable */
	{
		bs_skip(1);		/* dct_coefs */
		bs_skip(1);		/* dct_lines */
		bs_skip(1);		/* vlc_symbols */
		bs_skip(1);		/* vlc_bits */
	}

	if (!bs_get_bits(1))		/* motion_compensation_complexity_disable */
	{
		bs_skip(1);		/* apm */
		bs_skip(1);		/* npm */
		bs_skip(1);		/* interpolate_mc_q */
		bs_skip(1);		/* forw_back_mc_q */
		bs_skip(1);		/* halfpel2 */
		bs_skip(1);		/* halfpel4 */
	}

	bs_skip(1);//READ_MARKER();
	if (method == 1)
	{
		if (!bs_get_bits(1))	/* version2_complexity_estimation_disable */
		{
			bs_skip(1);		/* sadct */
			bs_skip(1);		/* quarterpel */
		}
	}
}

void vid_dec_parser_bit_stream_init(void)
{
	p_mp4_para = &mp4_para;
	gp_memset((INT8S*)p_mp4_para, 0, sizeof(Mpeg4ParaStruct));
	p_mp4_para->vop_time_increment_resolution = 1;
	p_mp4_para->visual_object_verid = 1;
	p_mp4_para->obmc_disable = 1;
	p_mp4_para->resync_marker_disable = 1;
	p_mp4_para->complexity_estimation_disable = 1;
}

INT32S vid_dec_paser_bit_stream(INT8U *pdata, INT32U bs_size, INT16U *width, INT16U *height, INT8U *time_inc_bits, INT8U *quant)
{
	INT32S nRet, coding_type;
	INT32U temp, start_code;

	//init
	g_bs_pos = 0;
	g_bs_ptr = pdata;
	pdata += bs_size + 8;
	g_bs_buffer[0] = (INT32U)(*g_bs_ptr++ << 24);
	g_bs_buffer[0] |= (INT32U)(*g_bs_ptr++ << 16);
	g_bs_buffer[0] |= (INT32U)(*g_bs_ptr++ << 8);
	g_bs_buffer[0] |= *g_bs_ptr++;
	g_bs_buffer[1] = (INT32U)(*g_bs_ptr++ << 24);
	g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 16);
	g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 8);
	g_bs_buffer[1] |= *g_bs_ptr++;

	// init coding type when m4s2 format
	if(p_vid_dec_para->video_format == C_M4S2_FORMAT)
		coding_type = C_I_VOP;
	else
		coding_type = C_UNKNOW_VOP;

	while(1)
	{
		bs_byte_align();
		start_code = bs_show_bits(32);
		if(start_code == VISOBJSEQ_START_CODE) //visual_object_sequence_start_code
		{
			bs_skip(32); //visual_object_sequence_start_code
			p_mp4_para->profile_and_level_indication = bs_get_bits(8); //profile_and_level_indication
			if(p_mp4_para->profile_and_level_indication == 0)
			{
				DEBUG_MSG("profile_and_level_indication = 0\r\n");
				//RETURN(ERROR_00); //don't care
			}
		}
		else if(start_code == VISOBJSEQ_STOP_CODE) //visual_object_sequence_stop_code
		{
			bs_skip(32); //visual_object_sequence_stop_code
		}
		else if(start_code == VISOBJ_START_CODE) //visual_object_start_code
		{
			// vidMP4StartCodeFind
			p_mp4_para->is_visual_object_start_code = 1;
			bs_skip(32); //visual_object_start_code

			// vidMP4VisualObjectVeridGet
			if(bs_get_bits(1))	//is_visual_object_identified
			{
				p_mp4_para->visual_object_verid = bs_get_bits(4); //visual_object_ver_id
				bs_skip(3); //visual_object_priority
			}
			else
			{
				p_mp4_para->visual_object_verid = 1;
			}

			p_mp4_para->visual_object_type = bs_show_bits(4);
			if(p_mp4_para->visual_object_type != VISOBJ_TYPE_VIDEO) //visual_object_type
			{
				DEBUG_MSG("visual_object_type = %d, not support!\r\n", p_mp4_para->visual_object_type);
				RETURN(ERROR_01);
			}
			bs_skip(4);

			if(bs_get_bits(1))	//video_signal_type
			{
				//DEBUG_MSG("+ video_signal_type!\r\n");
				bs_skip(3); //video_format
				bs_skip(1); //video_range
				if(bs_get_bits(1))	//color_description
				{
					bs_skip(8);	//color_primaries
					bs_skip(8);	//transfer_characteristics
					bs_skip(8);	//matrix_coefficients
				}
			}
		}
		else if((start_code & ~VIDOBJ_START_CODE_MASK) == VIDOBJ_START_CODE)
		{
			// vidMP4StartCodeFind
			p_mp4_para->is_vo_start_code = 1;
			bs_skip(32); //video_object_start_code
		}
		else if((start_code & ~VIDOBJLAY_START_CODE_MASK)== VIDOBJLAY_START_CODE) //video_object_layer_start_code
		{
			// vidMP4StartCodeFind
			p_mp4_para->is_vo_start_code = 1;
			bs_skip(32); //video_object_layer_start_code

			// vidMP4VideoObjectTypeIndicationGet
			bs_skip(1); //random_accessible_vol
			p_mp4_para->video_object_type_indication = bs_get_bits(8); //video_object_type_indication
			if(p_mp4_para->video_object_type_indication != 1) //simple profile
			{
				DEBUG_MSG("video_object_type_indication = %d, not support!!!\r\n", p_mp4_para->video_object_type_indication);
			#if ASP_ENABLE == 0
				RETURN(ERROR_02);
			#endif
			}

			// vidMP4VideoObjectLayerVeridGet
			p_mp4_para->is_object_layer_identifier = bs_get_bits(1); //is_object_layer_identifier
			if(p_mp4_para->is_object_layer_identifier == 1)
			{
				p_mp4_para->video_object_layer_verid = bs_get_bits(4); //video_object_layer_verid
				bs_skip(3);	//video_object_layer_priority
			}
			else
			{
				p_mp4_para->video_object_layer_verid = p_mp4_para->visual_object_verid;
			}

			// vidMP4VolControlParGet
			p_mp4_para->aspect_ratio_info = bs_get_bits(4);
			if(p_mp4_para->aspect_ratio_info == VIDOBJLAY_AR_EXTPAR)
			{
				p_mp4_para->par_width = bs_get_bits(8);
				p_mp4_para->par_height = bs_get_bits(8);
			}

			p_mp4_para->vol_control_par = bs_get_bits(1); //vol_control_parameters
			if(p_mp4_para->vol_control_par != 0)
			{
				p_mp4_para->chroma_format = bs_get_bits(2); //chroma_format
				if(p_mp4_para->chroma_format != 1)
				{
					DEBUG_MSG("vol_control_par = %d\r\n", p_mp4_para->vol_control_par);
					DEBUG_MSG("chroma_format=%d, not 420 format\r\n", p_mp4_para->chroma_format);
					RETURN(ERROR_03);
				}

				p_mp4_para->low_delay = bs_get_bits(1); //low_delay
				if(p_mp4_para->low_delay == 0)
				{
					DEBUG_MSG("low_delay=%d, VOL contains B-VOPs\r\n", p_mp4_para->low_delay);
					//RETURN(ERROR_04);	//same as 2900
				}

				p_mp4_para->vbv_par = bs_get_bits(1); //vbv_parameters
				if(p_mp4_para->vbv_par == 1)
				{
					INT32U bitrate, buffer_size, occupancy;
					//DEBUG_MSG("vbv_par=%d\n", p_mp4_para->vbv_par);
					bitrate = bs_get_bits(15) << 15; //first_half_bit_rate
					bs_skip(1);	//READ_MARKER
					bitrate |= bs_get_bits(15); //latter_half_bit_rate
					bs_skip(1);	//READ_MARKER

					buffer_size = bs_get_bits(15) << 3;	//first_half_vbv_buffer_size
					bs_skip(1);	//READ_MARKER
					buffer_size |= bs_get_bits(3); //latter_half_vbv_buffer_size

					occupancy = bs_get_bits(11) << 15; //first_half_vbv_occupancy
					bs_skip(1);	//READ_MARKER
					occupancy |= bs_get_bits(15); //latter_half_vbv_occupancy
					bs_skip(1);	//READ_MARKER
				}
			}

			// vidMP4VideoObjectLayerShapeGet
			p_mp4_para->video_object_layer_shape = bs_get_bits(2); //video_object_layer_shape
			if(p_mp4_para->video_object_layer_shape != VIDOBJLAY_SHAPE_RECTANGULAR)
			{
				DEBUG_MSG("video_object_layer_shape=%d, not support\r\n", p_mp4_para->video_object_layer_shape);
				RETURN(ERROR_05);
			}

			// vidMP4VopTimeIncrementResolutionGet
			bs_skip(1); //READ_MARKER
			p_mp4_para->vop_time_increment_resolution = bs_get_bits(16); //vop_time_increment_resolution
			temp = p_mp4_para->vop_time_increment_resolution;
			if(temp > 0)
			{
				p_mp4_para->vopTimeIncResLen = 0;
				do
				{
					temp >>= 1;
					p_mp4_para->vopTimeIncResLen++;
				}while(temp);
			}
			else
			{
				p_mp4_para->vopTimeIncResLen = 1;
			}

			*time_inc_bits = p_mp4_para->vopTimeIncResLen;
			bs_skip(1); //READ_MARKER
			p_mp4_para->fixed_vop_rate = bs_get_bits(1); //fixed_vop_rate
			if(p_mp4_para->fixed_vop_rate == 1)
			{
				bs_skip(p_mp4_para->vopTimeIncResLen); //fixed_vop_time_increment
			}

			// vidMP4WidthHeightGet
			bs_skip(1); //READ_MARKER
			p_mp4_para->video_object_layer_width = bs_get_bits(13); //video_object_layer_width
			bs_skip(1); //READ_MARKER
			p_mp4_para->video_object_layer_height = bs_get_bits(13); //video_object_layer_height
			bs_skip(1); //READ_MARKER

			*width = p_mp4_para->video_object_layer_width;
			*height = p_mp4_para->video_object_layer_height;
			if(p_mp4_para->video_object_layer_width > 768)
			{
				DEBUG_MSG("video_object_layer_width > 768, not support\r\n", p_mp4_para->video_object_layer_width);
				RETURN(ERROR_06);
			}

			// vidMP4InterlacedGet
			p_mp4_para->interlaced = bs_get_bits(1);
			if(p_mp4_para->interlaced != 0)
			{
				DEBUG_MSG("interlaced = %d, not support\r\n", p_mp4_para->interlaced);
				RETURN(ERROR_07);
			}

			// vidMP4ObmcDisableGet
			p_mp4_para->obmc_disable = bs_get_bits(1); //obmc_disable
			if(p_mp4_para->obmc_disable != 1)
			{
				DEBUG_MSG("obmc_disable = %d, not support\r\n", p_mp4_para->obmc_disable);
				RETURN(ERROR_08); //not support
			}

			// vidMP4SpriteEnableGet
			p_mp4_para->sprite_enable = bs_get_bits(p_mp4_para->video_object_layer_verid == 1 ? 1 : 2); //sprite_enable
			if(p_mp4_para->sprite_enable != 0)
			{
				DEBUG_MSG("sprite_enable = %d, not support\r\n", p_mp4_para->sprite_enable);
				RETURN(ERROR_09);
			}

			// vidMP4Not8BitGet
			p_mp4_para->not_8_bit = bs_get_bits(1);	//not_8_bit
			if(p_mp4_para->not_8_bit != 0)
			{
				DEBUG_MSG("not_8_bit = %d, not support\r\n", p_mp4_para->not_8_bit);
				RETURN(ERROR_0A);
			}

			if(p_mp4_para->not_8_bit)
			{
				p_mp4_para->quant_bits = bs_get_bits(4); //quant_precision
				bs_skip(4);	//bits_per_pixel
			}
			else
			{
				p_mp4_para->quant_bits = 5;
			}

			// vidMP4QuantTypeGet
			p_mp4_para->quant_type = bs_get_bits(1); //quant_type
			if(p_mp4_para->quant_type != 0)
			{
				DEBUG_MSG("quant_type = %d, It should be 0\r\n", p_mp4_para->quant_type);
			#if ASP_ENABLE == 0
				RETURN(ERROR_0B);	//not support
			#endif
			}

			// vidMP4QuarterSampleGet
			if(p_mp4_para->video_object_layer_verid != 1)
			{
				p_mp4_para->quarter_sample = bs_get_bits(1); //quarter_sample
				if(p_mp4_para->quarter_sample == 1)
				{
					DEBUG_MSG("quarter_sample = %d, It should be 0\r\n", p_mp4_para->quarter_sample);
				#if ASP_ENABLE == 0
					RETURN(ERROR_0C); //not support
				#endif
				}
			}

			// vidMP4CompEstDisableGet
			p_mp4_para->complexity_estimation_disable = bs_get_bits(1);
			if(p_mp4_para->complexity_estimation_disable != 1) //complexity estimation disable
			{
				DEBUG_MSG("complexity_estimation_disable = %d, It should be 1\r\n", p_mp4_para->complexity_estimation_disable);
				read_vol_complexity_estimation_header();
			#if ASP_ENABLE == 0
				RETURN(ERROR_0D); //not support
			#endif
			}

			// vidMP4DPRVLCGet
			p_mp4_para->resync_marker_disable = bs_get_bits(1);	//resync_marker_disable
			if(p_mp4_para->resync_marker_disable == 0)
			{
				DEBUG_MSG("resync_marker_disable = %d\r\n", p_mp4_para->resync_marker_disable);
				//RETURN(ERROR_0E); //support
			}

			p_mp4_para->data_partitioned = bs_get_bits(1);	//data_partitioned
			if(p_mp4_para->data_partitioned == 1)
			{
				p_mp4_para->reversible_vlc = bs_get_bits(1); //reversible_vlc
				DEBUG_MSG("data_partitioned = %d, not support\r\n", p_mp4_para->data_partitioned);
				DEBUG_MSG("reversible_vlc = %d\r\n", p_mp4_para->reversible_vlc);
				RETURN(ERROR_0F);
			}

			if(p_mp4_para->video_object_layer_verid != 1)
			{
				p_mp4_para->newpred_enable = bs_get_bits(1); //newpred_enable
				if(p_mp4_para->newpred_enable)
				{
					bs_skip(2);	//requested_upstream_message_type
					bs_skip(1);	//newpred_segment_type
				}
				p_mp4_para->reduced_resolution_enable = bs_get_bits(1); //reduced_resolution_vop_enable
			}

			// vidMP4ScalabilityGet
			p_mp4_para->scalability = bs_get_bits(1); //scalability
			if(p_mp4_para->scalability != 0)
			{
				bs_skip(1);	//hierarchy_type
				bs_skip(4);	//ref_layer_id
				bs_skip(1);	//ref_layer_sampling_direc
				bs_skip(5); //hor_sampling_factor_n
				bs_skip(5);	//hor_sampling_factor_m
				bs_skip(5);	//vert_sampling_factor_n
				bs_skip(5);	//vert_sampling_factor_m
				bs_skip(1);	//enhancement_type
				if(p_mp4_para->video_object_layer_shape == VIDOBJLAY_SHAPE_BINARY)
				{
					bs_skip(1);	//use_ref_shape
					bs_skip(1);	//use_ref_texture
					bs_skip(5);	//shape_hor_sampling_factor_n
					bs_skip(5);	//shape_hor_sampling_factor_m
					bs_skip(5);	//shape_vert_sampling_factor_n
					bs_skip(5);	//shape_vert_sampling_factor_m
				}
				DEBUG_MSG("scalability = %d, not support\r\n", p_mp4_para->scalability);
				RETURN(ERROR_10);
			}
		}
		else if (start_code == GRPOFVOP_START_CODE)
		{
			bs_skip(32);
			{
				INT16U hours, minutes, seconds;
				hours = bs_get_bits(5);
				minutes = bs_get_bits(6);
				bs_skip(1); //READ_MARKE
				seconds = bs_get_bits(6);
			}
			bs_skip(1);	//closed_gov
			bs_skip(1);	//broken_link
		}
		else if(start_code == VOP_START_CODE)
		{
			bs_skip(32); //vop_start_code
			coding_type = bs_get_bits(2); //vop_coding_type
			while (bs_get_bits(1) != 0); //time_base

			bs_skip(1); //READ_MARKER
			if(p_mp4_para->vopTimeIncResLen)
				p_mp4_para->time_increment = bs_get_bits(p_mp4_para->vopTimeIncResLen);	//vop_time_increment

			bs_skip(1); //READ_MARKER
			if(!bs_get_bits(1))	//vop_coded
				RETURN(C_N_VOP);

			if(p_mp4_para->newpred_enable)
			{
				if((p_mp4_para->vopTimeIncResLen + 3) >= 15)
					temp = 15;

				p_mp4_para->vop_id = bs_get_bits(temp);
				if(bs_get_bits(1)) //vop_id_for_prediction_indication
				{
					p_mp4_para->vop_id_for_prediction = bs_get_bits(temp);
				}
				bs_skip(1); //READ_MARKER
			}

			if ((p_mp4_para->video_object_layer_shape != VIDOBJLAY_SHAPE_BINARY_ONLY) && (coding_type == C_P_VOP))
				bs_skip(1);	//rounding_type

			if (p_mp4_para->reduced_resolution_enable &&
				(p_mp4_para->video_object_layer_shape == VIDOBJLAY_SHAPE_RECTANGULAR) &&
				((coding_type == C_P_VOP) || (coding_type == C_I_VOP)))
			{
				if (bs_get_bits(1))	//vop reduced resolution
				{
					//return -10;	//RRV not supported
				}
			}

			if (p_mp4_para->video_object_layer_shape != VIDOBJLAY_SHAPE_RECTANGULAR)
			{
				bs_skip(1);	//change_conv_ratio_disable
				if (bs_get_bits(1))	//vop_constant_alpha
				{
					bs_skip(8); //vop_constant_alpha_value
				}
			}

			if (p_mp4_para->video_object_layer_shape != VIDOBJLAY_SHAPE_BINARY_ONLY)
			{
				//if (!dec->complexity_estimation_disable)
				//{
				//	read_vop_complexity_estimation_header(bs, dec, coding_type);
				//}
				bs_skip(3); //intra_dc_threshold = intra_dc_threshold_table[BitstreamGetBits(bs, 3)]; // intra_dc_vlc_threshold
			}

			*quant = p_mp4_para->vop_quant = bs_get_bits(p_mp4_para->quant_bits); //vop_quant
			if(p_mp4_para->vop_quant < 1)
				*quant = p_mp4_para->vop_quant = 1;

			RETURN(coding_type);
		}
		else if (start_code == USERDATA_START_CODE)
		{
			bs_skip(32); //user_data_start_code
			gp_memset((INT8S*)p_mp4_para->user_data, 0, 256);
			for(temp=0; temp < 256; temp++)
			{
				if(temp == 0)
					p_mp4_para->user_data[temp] = bs_show_bits(8);
				else
					p_mp4_para->user_data[temp] = (bs_show_bits(16) & 0xFF);

				if(p_mp4_para->user_data[temp] == 0)
					break;
				bs_skip(8);
			}
		}
		else
		{
			bs_skip(8);	//start_code == ?
		}

		if(g_bs_ptr > pdata)
			RETURN(coding_type);
	}

Return:
	return nRet;
}

///////////////////////////////////////////////////////////////////////////////////
// h263 bit stream parser
INT32S vid_dec_paser_h263_bit_stream(INT8U *pdata, INT16U *width, INT16U *height, INT32U *key_word)
{
	INT8U  *psrc, source_format;
	INT16U num_mb_in_gob;
	INT32S nRet, i;
	INT32U h263_header;

	//get key word
	psrc = pdata;
	*key_word = *psrc++;
	*key_word <<= 8;
	*key_word |= *psrc++;
	*key_word <<= 8;
	*key_word |= *psrc++;

	for(i=0; i<50; i++)
	{
		if((*key_word & 0x000080) == 0x000080)
		{
			*key_word <<= 8;
			*key_word |= *psrc++;
			break;
		}

		pdata++;
		*key_word <<= 8;
		*key_word |= *psrc++;
		*key_word &= 0x00FFFFFF;
	}
	if(i == 50)
		RETURN(ERROR_00);

	//parser bs
	g_bs_pos = 0;
	g_bs_ptr = pdata;
	g_bs_buffer[0] = (INT32U)(*g_bs_ptr++ << 24);
	g_bs_buffer[0] |= (INT32U)(*g_bs_ptr++ << 16);
	g_bs_buffer[0] |= (INT32U)(*g_bs_ptr++ << 8);
	g_bs_buffer[0] |= *g_bs_ptr++;
	g_bs_buffer[1] = (INT32U)(*g_bs_ptr++ << 24);
	g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 16);
	g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 8);
	g_bs_buffer[1] |= *g_bs_ptr++;

	h263_header = bs_get_bits(22);
	if(h263_header != 0x0020)
		RETURN(ERROR_01);

	bs_skip(8); //temporal_reference
	bs_skip(1); //marker_bit
	bs_skip(1); //zero_bit
	bs_skip(1); //split_screen_indicator
	bs_skip(1); //document_camera_indicator
	bs_skip(1); //full_picture_freeze_release

	source_format = bs_get_bits(3);
	switch(source_format)
	{
	case 1:
		*width = 128; *height = 96; num_mb_in_gob = 8;
		break;
	case 2:
		*width = 176; *height = 144; num_mb_in_gob = 11;
		break;
	case 3:
		*width = 352; *height = 288;  num_mb_in_gob = 22;
		break;
	case 4:
		*width = 704; *height = 576;  num_mb_in_gob = 88;
		break;
	default:
		RETURN(ERROR_02);
	}

	nRet = bs_get_bits(1); //h263_picture_coding_type
Return:
	return nRet;
}

///////////////////////////////////////////////////////////////////////////////////
#if S263_DECODE_ENABLE == 1
INT32S vid_dec_paser_sorenson263_bit_stream(INT8U *pdata, INT16U *width, INT16U *height, INT32U *key_word)
{
	INT8U  *psrc;
	INT32S  nRet, i, temp;

	//get key word
	psrc = pdata;
	*key_word = *psrc++;
	*key_word <<= 8;
	*key_word |= *psrc++;
	*key_word <<= 8;
	*key_word |= *psrc++;

	for(i=0; i<50; i++)
	{
		if((*key_word & 0x000080) == 0x000080)
		{
			*key_word <<= 8;
			*key_word |= *psrc++;
			break;
		}

		pdata++;
		*key_word <<= 8;
		*key_word |= *psrc++;
		*key_word &= 0x00FFFFFF;
	}
	if(i == 50)
		RETURN(ERROR_00);

	g_bs_pos = 0;
	g_bs_ptr = pdata;
	g_bs_buffer[0] = (INT32U)(*g_bs_ptr++ << 24);
	g_bs_buffer[0] |= (INT32U)(*g_bs_ptr++ << 16);
	g_bs_buffer[0] |= (INT32U)(*g_bs_ptr++ << 8);
	g_bs_buffer[0] |= *g_bs_ptr++;
	g_bs_buffer[1] = (INT32U)(*g_bs_ptr++ << 24);
	g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 16);
	g_bs_buffer[1] |= (INT32U)(*g_bs_ptr++ << 8);
	g_bs_buffer[1] |= *g_bs_ptr++;

	//StartCode
	if(bs_get_bits(17) != 1)
	{
		DEBUG_MSG("StartCodeMismatch !!!\r\n");
		RETURN(ERROR_01);
	}

	//Version
	nRet = bs_get_bits(5); //h263_version

	//TemporalReference
	bs_get_bits(8);

	//PictureSize
	temp = bs_show_bits(3);
	switch(temp)
	{
	case 0:
		bs_skip(3);
	  	*width = bs_get_bits(8);
	  	*height = bs_show_bits(8);
	  	bs_skip(7);
	  	*key_word = *psrc++;
		*key_word <<= 8;
		*key_word |= *psrc++;
		*key_word <<= 8;
		*key_word |= *psrc++;
	  	*key_word <<= 8;
		*key_word |= *psrc++;
		break;

	case 1:
		bs_skip(3);
	  	*width = bs_get_bits(16);
	  	*height = bs_show_bits(16);
	  	bs_skip(15);
	  	*key_word = *psrc++;
		*key_word <<= 8;
		*key_word |= *psrc++;
		*key_word <<= 8;
		*key_word |= *psrc++;
	  	*key_word <<= 8;
		*key_word |= *psrc++;
		break;

	case 2:
		bs_skip(2);
	  	*width = 352;
	  	*height = 288;
		break;

	case 3:
		bs_skip(2);
	  	*width = 176;
	  	*height = 144;
		break;

	case 4:
		bs_skip(2);
	    *width = 128;
	    *height = 96;
		break;

	case 5:
		bs_skip(2);
	  	*width = 320;
	  	*height = 240;
		break;

	case 6:
		bs_skip(2);
	  	*width =160;
	  	*height =120;
		break;

	default:
		DEBUG_MSG("InvalidWidth/Height!!!\r\n");
		RETURN(ERROR_02);
	}

	*width = (*width + 15) & ~0x0F;
	*height = (*height + 15) & ~0x0F;

Return:
	return nRet;
}
#endif
#endif

const unsigned char * m_pStart;
unsigned short m_nLength;
int m_nCurrentBit;

static unsigned int ReadBit()
{
    //assert(m_nCurrentBit <= m_nLength * 8);
    int nIndex = m_nCurrentBit / 8;
    int nOffset = m_nCurrentBit % 8 + 1;

    m_nCurrentBit ++;
    return (m_pStart[nIndex] >> (8-nOffset)) & 0x01;
}

static unsigned int ReadBits(int n)
{
    int r = 0;
    int i;
    for (i = 0; i < n; i++)
    {
        r |= ( ReadBit() << ( n - i - 1 ) );
    }
    return r;
}

static unsigned int ReadExponentialGolombCode()
{
    int r = 0;
    int i = 0;

    while( (ReadBit() == 0) && (i < 32) )
    {
        i++;
    }

    r = ReadBits(i);
    r += (1 << i) - 1;
    return r;
}

static unsigned int ReadSE()
{
    int r = ReadExponentialGolombCode();
    if (r & 0x01)
    {
        r = (r+1)/2;
    }
    else
    {
        r = -(r/2);
    }
    return r;
}

static void Parse(const unsigned char * pStart, unsigned short nLen, INT16U* w, INT16U* h)
{
    m_pStart = pStart;
    m_nLength = nLen;
    m_nCurrentBit = 0;

    int frame_crop_left_offset=0;
    int frame_crop_right_offset=0;
    int frame_crop_top_offset=0;
    int frame_crop_bottom_offset=0;

    int profile_idc = ReadBits(8);
    int constraint_set0_flag = ReadBit();
    int constraint_set1_flag = ReadBit();
    int constraint_set2_flag = ReadBit();
    int constraint_set3_flag = ReadBit();
    int constraint_set4_flag = ReadBit();
    int constraint_set5_flag = ReadBit();
    int reserved_zero_2bits  = ReadBits(2);
    int level_idc = ReadBits(8);
    int seq_parameter_set_id = ReadExponentialGolombCode();


    if( profile_idc == 100 || profile_idc == 110 ||
        profile_idc == 122 || profile_idc == 244 ||
        profile_idc == 44 || profile_idc == 83 ||
        profile_idc == 86 || profile_idc == 118 )
    {
        int chroma_format_idc = ReadExponentialGolombCode();

        if( chroma_format_idc == 3 )
        {
            int residual_colour_transform_flag = ReadBit();
        }
        int bit_depth_luma_minus8 = ReadExponentialGolombCode();
        int bit_depth_chroma_minus8 = ReadExponentialGolombCode();
        int qpprime_y_zero_transform_bypass_flag = ReadBit();
        int seq_scaling_matrix_present_flag = ReadBit();

        if (seq_scaling_matrix_present_flag)
        {
            int i=0;
            for ( i = 0; i < 8; i++)
            {
                int seq_scaling_list_present_flag = ReadBit();
                if (seq_scaling_list_present_flag)
                {
                    int sizeOfScalingList = (i < 6) ? 16 : 64;
                    int lastScale = 8;
                    int nextScale = 8;
                    int j=0;
                    for ( j = 0; j < sizeOfScalingList; j++)
                    {
                        if (nextScale != 0)
                        {
                            int delta_scale = ReadSE();
                            nextScale = (lastScale + delta_scale + 256) % 256;
                        }
                        lastScale = (nextScale == 0) ? lastScale : nextScale;
                    }
                }
            }
        }
    }

    int log2_max_frame_num_minus4 = ReadExponentialGolombCode();
    int pic_order_cnt_type = ReadExponentialGolombCode();
    if( pic_order_cnt_type == 0 )
    {
        int log2_max_pic_order_cnt_lsb_minus4 = ReadExponentialGolombCode();
    }
    else if( pic_order_cnt_type == 1 )
    {
        int delta_pic_order_always_zero_flag = ReadBit();
        int offset_for_non_ref_pic = ReadSE();
        int offset_for_top_to_bottom_field = ReadSE();
        int num_ref_frames_in_pic_order_cnt_cycle = ReadExponentialGolombCode();
        int i;
        for( i = 0; i < num_ref_frames_in_pic_order_cnt_cycle; i++ )
        {
            ReadSE();
            //sps->offset_for_ref_frame[ i ] = ReadSE();
        }
    }
    int max_num_ref_frames = ReadExponentialGolombCode();
    int gaps_in_frame_num_value_allowed_flag = ReadBit();
    int pic_width_in_mbs_minus1 = ReadExponentialGolombCode();
    int pic_height_in_map_units_minus1 = ReadExponentialGolombCode();
    int frame_mbs_only_flag = ReadBit();
    if( !frame_mbs_only_flag )
    {
        int mb_adaptive_frame_field_flag = ReadBit();
    }
    int direct_8x8_inference_flag = ReadBit();
    int frame_cropping_flag = ReadBit();
    if( frame_cropping_flag )
    {
        frame_crop_left_offset = ReadExponentialGolombCode();
        frame_crop_right_offset = ReadExponentialGolombCode();
        frame_crop_top_offset = ReadExponentialGolombCode();
        frame_crop_bottom_offset = ReadExponentialGolombCode();
    }
    int vui_parameters_present_flag = ReadBit();
    pStart++;

    int Width = ((pic_width_in_mbs_minus1 +1)*16) - frame_crop_left_offset*2 - frame_crop_right_offset*2;
    int Height = ((2 - frame_mbs_only_flag)* (pic_height_in_map_units_minus1 +1) * 16) - (frame_crop_top_offset * 2) - (frame_crop_bottom_offset * 2);

    DBG_PRINT("Parsed H.264 stream for width and height: WxH = %dx%d\r\n",Width,Height);

    *w = Width;
    *h = Height;

}

INT32S vid_dec_parser_h264_bit_stream(INT8U *pdata, INT32U bs_size, INT16U *width, INT16U *height)
{
	INT8U* p = pdata;
	INT8U* end = p+bs_size;
	while(p < end)
	{
		INT32U* pdw = (INT32U*)p;

		if (*pdw == 0x01000000)
		{
			p+= 3;

			if (((*(p+1)) & 0x1f) == 7) // sps
			{
				p+=2;
				break;
			}
		}

		p++;
	}

	if (p >= end)
		return -1;

	Parse(p, (INT32U)end - (INT32U)p, width, height);

	return 0;
}
#endif//#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECDOER_EN == 1)
