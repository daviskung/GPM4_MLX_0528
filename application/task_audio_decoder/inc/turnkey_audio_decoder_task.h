#ifndef __AUDIO_DECODER_TASK_H__
#define __AUDIO_DECODER_TASK_H__

#include "application.h"

#define RING_BUF_SIZE     	4096
#define BG_RING_BUF_SIZE  	4096
#define MP3_PCM_BUF_SIZE  	MP3_DEC_FRAMESIZE*2 /* 1152*2,two channel*/
#define WMA_PCM_BUF_SIZE  	WMA_DEC_FRAMESIZE*2 /* 2048*2 */
#define WAV_PCM_BUF_SIZE  	WAV_DEC_FRAMESIZE*2
#define A1800_PCM_BUF_SIZE	A18_DEC_FRAMESIZE*2
#define A1600_PCM_BUF_SIZE	A16_DEC_FRAMESIZE*2
#define A6400_PCM_BUF_SIZE	A6400_DEC_FRAMESIZE*2
#define S880_PCM_BUF_SIZE	S880_DEC_FRAMESIZE*2
#define AAC_PCM_BUF_SIZE  	AAC_DEC_FRAMESIZE*2
#define OGG_PCM_BUF_SIZE  	OGGVORBIS_DEC_FRAMESIZE*2

#define MP3_FRAME_ERROR_CNT (5000/26)

#define AUDIO_SEND_FAIL   -1
#define AUDIO_SEND_OK     0

#define AUDIO_IDLE        0
#define AUDIO_PLAYING     1
#define AUDIO_PLAY_PAUSE  2
#define AUDIO_PLAY_STOP   3

#define AUDIO_READ_FAIL   -2
#define AUDIO_READ_PEND   -3
#define AUDIO_READ_WAIT   -4

#define ID3_TAG_NONE      0
#define ID3_TAG_V1        1
#define ID3_TAG_V2        2
#define ID3_TAG_V2_FOOTER 3
#define ID3_TAG_FLAG_FOOTER 0x10

#define AUDIO_PARSE_FAIL 		-1
#define AUDIO_PARSE_SUCCS 		0
#define AUDIO_MAX_TYPE			4
#define AUDIO_PASER_BUFFER_SIZE  (	2048*10)

typedef struct
{
	AUDIO_TYPE audio_file_real_type;
	INT8S (*parse_audio_file_head)(INT8S *p_audio_file_head);
}AUDIO_FILE_PARSE;

typedef struct
{
	INT8S  mpeg_version;
	INT8S  layer;
	INT8S  sample_rate;
}MP3_FILE_HEAD;

typedef struct
{
	INT8U emphasis		:	 2;
	INT8U original		:	 1;
	INT8U copyright		:	 1;
	INT8U ext			:	 2;
	INT8U channel		:    2;
	INT8U priv			:	 1;
	INT8U padding		:	 1;
	INT8U samplerate	:    2;
	INT8U bitrate		:	 4;
	INT8U protection	:	 1;
	INT8U layer			:    2;
	INT8U mpeg			:    2;
	INT16U sync			:    11;
}MP3_SYNC_WORD;

typedef union//struct
{
	MP3_SYNC_WORD sync_word;
	INT32U  word;
	INT8U   ch[4];
} MP3_SYNC_UNION;


typedef struct
{
	INT8U    state;
	INT8U    source_type;
	INT8U    curr_vol;
	INT32U   audio_format;
	INT32S	(*fp_deocde_init)();
	INT32S	(*fp_deocde)();
}AUDIO_CONTEXT,*AUDIO_CONTEXT_P;

typedef struct
{
	INT8U *ptr;
	INT32U cnt;
}RES_HD;

typedef struct
{
    INT32U sample_rate;
    INT32U channel_num;
    INT32U i2s_channel;
}AUDIO_I2S_PROFILE;
/*
typedef struct
{
	INT8U	*ring_buf;
	INT8S   *work_mem;
	INT16S  file_handle;
	INT32U  ring_size;
	INT32U  ri;
	INT32U  wi;
	INT32U   f_last;
	INT32U  file_len;
	INT32U  file_cnt;
	INT32U  try_cnt;
	INT32U  read_secs;
}AUDIO_CTRL;
*/

typedef struct
{
	INT8U	RIFF_ID[4];	//= {'R','I','F','F'};
	INT32U 	RIFF_len;	//file size -8
	INT8U 	type_ID[4];	// = {'W','A','V','E'};
	INT8U	fmt_ID[4];	// = {'f', 'm','t',' '};
	INT32U  fmt_len;	//16 + extern format byte
	INT16U  format;		// = 1; 	//pcm
	INT16U	channel;	// = 1;	// mono
	INT32U  sample_rate;// = 8000;
	INT32U  avg_byte_per_sec;// = 8000*2;	//AvgBytesPerSec = SampleRate * BlockAlign
	INT16U	Block_align;// = (16 / 8*1) ;				//BlockAlign = SignificantBitsPerSample / 8 * NumChannels
	INT16U	Sign_bit_per_sample;// = 16;		//8, 16, 24 or 32
	INT8U	data_ID[4];// = {'d','a','t','a'};
	INT32U	data_len; //extern format byte
}AUD_PCM_WAVE_HEADER;


extern void    audio_send_result(INT32S res_type,INT32S result);
extern void    audio_BG_send_result(INT32S res_type,INT32S result);
extern void    MIDI_send_result(INT32S res_type,INT32S result);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
extern INT32S  audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, OS_EVENT *ack_fsq);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
extern INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, xQueueHandle ack_fsq);
extern INT32S audio_i2s_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, xQueueHandle ack_fsq, INT32U i2s_ch);
#else
#error "Not Support No OS system!"
#endif
extern INT32S  audio_mem_alloc(INT32U audio_format,AUDIO_CTRL *aud_ctrl, INT16S *pcm[]);

extern void audio_task_entry(void *p_arg);
extern void audio_i2s_decode_task_entry(void *p_arg);
extern void audio_bg_task_entry(void *p_arg);
extern INT32U g_MIDI_index;		// added by Bruce, 2008/10/01
extern INT16U A1800_Bitrate;	// added by Bruce, 2008/10/03
//extern void	(*decode_end)();	// added by Bruce, 2008/10/03
extern void (*decode_end)(INT32U audio_decoder_num);	// modified by Bruce, 2008/11/20
extern void (*i2s_decode_end)(INT32U audio_decoder_num, INT32U i2s_ch);
extern INT32S mp3_get_duration(INT16S fd,INT32U len);
extern int wma_dec_seek(char *wmadec_workmem, int msTime);

#endif 		/* __AUDIO_DECODER_TASK_H__ */

