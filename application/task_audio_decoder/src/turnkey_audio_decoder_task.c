#include <string.h>
#include "turnkey_audio_decoder_task.h"
#include "turnkey_audio_dac_task.h"
#include "drv_l1_dac.h"
#include "drv_l1_hdmi.h"
// Constant definitions used in this file only go here
#define AUDIO_QUEUE_MAX  64
#define AUDIO_FS_Q_SIZE  1
#define AUDIO_WRITE_Q_SIZE MAX_DAC_BUFFERS
#define AUDIO_PARA_MAX_LEN  sizeof(STAudioTaskPara)
#define AUDIO_AVI_RINGBUF_LEN	4000
#define RAMP_DOWN_STEP 4
#define RAMP_DOWN_STEP_HOLD 4
#define RAMP_DOWN_STEP_LOW_SR	4*16
#define USE_RAMP_DOWN_RAPID		0
#define SKIP_ID3_TAG            0
#define Save_Voice_Buffer_Size  		1024*128//1024*32//20110517

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
extern xQueueHandle hAudioDacTaskQ;
extern SACM_CTRL G_SACM_Ctrl;
extern MSG_Q_ID fs_msg_q_id;
extern MSG_Q_ID Audio_FG_status_Q;
extern INT32S Snd_GetData(INT32U buf_adr,INT32U buf_size);
extern INT32U Snd_GetLen(void);
extern INT32S Snd_Lseek(INT32S offset,INT16S fromwhere);
#endif

extern INT32U nand_ogg_read_index;

/* Task Q declare */
MSG_Q_ID AudioTaskQ;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
OS_EVENT	*audio_wq;
void		*write_q[AUDIO_WRITE_Q_SIZE];
OS_EVENT	*audio_fsq;
void		*fs_q[AUDIO_FS_Q_SIZE];

#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
xQueueHandle audio_wq;
xQueueHandle audio_fsq;

#else
#error "Audio Decode Not Support In Non OS System!"
#endif

INT8U       audio_para[AUDIO_PARA_MAX_LEN];

INT16S		*pcm_out[MAX_DAC_BUFFERS] = {NULL};
INT32U		pcm_len[MAX_DAC_BUFFERS];

INT8S		avi_work_mem[WAV_DEC_MEMORY_SIZE];
AUDIO_CONTEXT   audio_context;
AUDIO_CONTEXT_P audio_context_p = &audio_context;
AUDIO_CTRL      audio_ctrl;
STAudioConfirm  aud_con;
#if (_OPERATING_SYSTEM == _OS_UCOS2)
volatile INT8U  *avi_state_mon;
#endif

static struct sfn_info aud_sfn_file;
INT8U	stopped;
INT8U	audio_rampdown_disable_flag;
void	(*decode_end)(INT32U audio_decoder_num);	// added by Bruce, 2008/10/03
INT32S	(*audio_move_data)(INT32U buf_addr, INT32U buf_size, INT8U *data_start_addr, INT32U data_offset);	// added by Bruce, 2008/10/27
INT32S	(*audio_bg_move_data)(INT32U buf_addr, INT32U buf_size, INT8U *data_start_addr, INT32U data_offset);
INT32S  g_audio_data_offset;
//static  INT8U   channel;
static  INT16U   g_audio_sample_rate;
static  INT8U    g_audio_main_channel;
#if 1//for new parser
INT32U mp3_VBR_flag = 0;//1:VBR  0:CBR
INT32S mp3_total_frame = 0;
INT32S audio_total_time = 0;
INT32U  mp3_ID3V2_length = 0;
INT32S  audio_play_start_time = 0;
INT32S  audio_decode_cnt = 0;
INT32U	audio_samplerate = 0;
INT32U  audio_bitrate = 0;
INT32U  audio_data_size = 0;
INT32S	audio_decode_sample_number = 0;
INT8U	audio_channel_number = 0;


static const long table_BitRate_V1L1[15]	= {0,32,64,96,128,160,192,224,256,288,320,352,384,416,448};
static const long table_BitRate_V1L2[15]	= {0,32,48,56,64,80,96,112,128,160,192,224,256,320,384};
static const long table_BitRate_V1L3[15]	= {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320};
static const long table_BitRate_V2L1[15]	= {0,32,48,56,64,80,96,112,128,144,160,176,192,224,256};
static const long table_BitRate_V2L2L3[15]	= {0,8,16,24,32,40,48,56,64,80,96,112,128,144,160};
static const long *table_BitRate_V1[4] = {0, table_BitRate_V1L3, table_BitRate_V1L2, table_BitRate_V1L1};
static const long *table_BitRate_V2[4] = {0, table_BitRate_V2L2L3, table_BitRate_V2L2L3, table_BitRate_V2L1};
static const long **table_BitRate[4] = {table_BitRate_V2, 0, table_BitRate_V2, table_BitRate_V1};


static const long table_SampleRate_V1[3]	= {44100,48000,32000};
static const long table_SampleRate_V2[3]	= {22050,24000,16000};
static const long table_SampleRate_V25[3]	= {11025,12000,8000};
static const long *table_SampleRate[4] = {table_SampleRate_V25, 0, table_SampleRate_V2, table_SampleRate_V1};

static const INT32U table_MpegVersion[4]	= {25,0,2,1};

#define EQUAL_MASK	0xFFFE0CC0
#define LAYER_III	1
#define LAYER_II	2
#define LAYER_I		3

#define MPEG_V25	0
#define MPEG_V2		2
#define MPEG_V1		3

#else//old
INT32U mp3_VBR_flag = 0;//1:VBR  0:CBR
INT32S mp3_total_frame = 0;
INT32S audio_total_time = 0;
INT32U  mp3_ID3V2_length = 0;
INT32S  audio_play_start_time = 0;
INT32S  audio_decode_cnt = 0;
INT32U	audio_samplerate = 0;
INT32U  audio_bitrate = 0;
INT32U  audio_data_size = 0;
INT32S	audio_decode_sample_number = 0;
INT8U	audio_channel_number = 0;
#endif
INT16S   gEncode_Filehandle = -1;
static INT8U *Save_Voice_Buffer;//20110517
static INT32U Write_Index = 0;//20110517
static AUD_PCM_WAVE_HEADER PCM_Header;
static INT32S	(*pfnGetOutput)(void*, short*, int);
static void     *hSrc;

/* Proto types */
void audio_task_init(void);
void audio_task_entry(void *p_arg);

INT32S Save_Last_PCM_File(void);

static void    audio_init(void);
#if APP_WAV_CODEC_FG_EN == 1
static INT32S  audio_wav_dec_play_init(void);
static INT32S  audio_wav_dec_process(void);
#endif

#if APP_MP3_DECODE_FG_EN == 1
static INT32S  audio_mp3_play_init(void);
static INT32S  audio_mp3_process(void);
int MP3Parser_Init(INT16S fin);
int MP3Parser_Seek( INT32U *p_msec,INT32U vbr);
extern xQueueHandle Mbox_MP3_Play_Seek_Flag, Mbox_MP3_Play_Seek_Offset;    //mp3 seek
#endif

#if APP_A1800_DECODE_FG_EN == 1
static INT32S  audio_a1800_play_init(void);
static INT32S  audio_a1800_process(void);
#endif

#if APP_WMA_DECODE_FG_EN == 1
static INT32S fg_error_cnt;
static INT32S  audio_wma_play_init(void);
static INT32S  audio_wma_process(void);
#endif

#if APP_A1600_DECODE_FG_EN == 1
static INT32S  audio_a16_play_init(void);
static INT32S  audio_a16_process(void);
#endif

#if APP_A6400_DECODE_FG_EN == 1
static INT32S  audio_a64_play_init(void);
static INT32S  audio_a64_process(void);
#endif

#if APP_S880_DECODE_FG_EN == 1
static INT32S  audio_s880_play_init(void);
static INT32S  audio_s880_process(void);
#endif

#if APP_AAC_DECODE_FG_EN == 1
static INT32S  audio_aac_play_init(void);
static INT32S  audio_aac_process(void);
#endif

#if APP_OGG_DECODE_FG_EN == 1
static INT32S  audio_ogg_play_init(void);
static INT32S  audio_ogg_process(void);
#endif

static INT32S  audio_send_to_dma(void);

#if APP_MP3_DECODE_FG_EN == 1 || APP_WMA_DECODE_FG_EN == 1 || APP_WAV_CODEC_FG_EN == 1 || APP_A1600_DECODE_FG_EN == 1 || APP_A1800_DECODE_FG_EN == 1 || APP_A6400_DECODE_FG_EN == 1 || APP_S880_DECODE_FG_EN == 1
static INT32S  audio_write_with_file_srv(INT8U *ring_buf, INT32U wi, INT32U ri);
static INT32S audio_check_wi(INT32S wi_in, INT32U *wi_out, INT8U wait);
#endif

static void    audio_queue_clear(void);
static void    audio_stop_unfinished(void);
static void    audio_send_next_frame_q(void);
static void    audio_ramp_down(void);
static void    audio_start(STAudioTaskPara *pAudioTaskPara);
static void    audio_pause(STAudioTaskPara *pAudioTaskPara);
static void    audio_resume(STAudioTaskPara *pAudioTaskPara);
static void    audio_stop(STAudioTaskPara *pAudioTaskPara);

static void    audio_decode_next_frame(STAudioTaskPara *pAudioTaskPara);
static void    audio_mute_set(STAudioTaskPara *pAudioTaskPara);
static void    audio_volume_set(STAudioTaskPara *pAudioTaskPara);

static void audio_start_parse(STAudioTaskPara *pAudioTaskPara);

#if (defined AUDIO_FORMAT_JUDGE_AUTO) && (AUDIO_FORMAT_JUDGE_AUTO == 1)
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static INT32S audio_get_type(INT16S fd,INT8S* file_name, OS_EVENT *ack_fsq);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
static INT32S audio_get_type(INT16S fd,INT8S* file_name, xQueueHandle ack_fsq);
#endif
#else
static INT32S  audio_get_type(INT8S* file_name);
#endif

static INT32S  audio_q_check(void);

#if USE_RAMP_DOWN_RAPID == 1
static void audio_ramp_down_rapid(void);
#endif

#if (defined SKIP_ID3_TAG) && (SKIP_ID3_TAG == 1)
static INT8U audio_get_id3_type(INT8U *data, INT32U length);
static void audio_parse_id3_header(INT8U *header, INT32U *version, INT32S *flags, INT32U *size);
static INT32U audio_id3_get_size(INT8U *ptr);
INT32S audio_id3_get_tag_len(INT8U *data, INT32U length);
#endif

void audio_task_init(void)
{
    /* Create MsgQueue/MsgBox for TASK */

    AudioTaskQ = msgQCreate(AUDIO_QUEUE_MAX, AUDIO_QUEUE_MAX, AUDIO_PARA_MAX_LEN);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
    audio_wq = OSQCreate(write_q, AUDIO_WRITE_Q_SIZE);
	audio_fsq = OSQCreate(fs_q, AUDIO_FS_Q_SIZE);

	avi_state_mon = &audio_context.state;
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    audio_wq = xQueueCreate(AUDIO_WRITE_Q_SIZE, sizeof(INT32U));
    audio_fsq = xQueueCreate(AUDIO_FS_Q_SIZE, sizeof(INT32U));
#endif

}

void audio_task_entry(void *p_arg)
{
    INT32S  ret;
    INT32U  msg_id;
	STAudioTaskPara     *pstAudioTaskPara;

	audio_task_init();
	audio_init();

	while (1)
	{
	    /* Pend task message */
	    ret = msgQReceive(AudioTaskQ, &msg_id, (void*)audio_para, AUDIO_PARA_MAX_LEN);
        if(ret < 0) {
            continue;
        }

        pstAudioTaskPara = (STAudioTaskPara*) audio_para;
		switch(msg_id) {
			case MSG_AUD_PLAY: /* by file handle */
			case MSG_AUD_PLAY_BY_SPI:
                audio_start(pstAudioTaskPara);
				break;
			case MSG_AUD_STOP:
				audio_stop(pstAudioTaskPara);
				break;
			case MSG_AUD_PAUSE:
				audio_pause(pstAudioTaskPara);
				break;
			case MSG_AUD_RESUME:
				audio_resume(pstAudioTaskPara);
				break;
			case MSG_AUD_SET_MUTE:
				audio_mute_set(pstAudioTaskPara);
				break;
			case MSG_AUD_VOLUME_SET:
				audio_volume_set(pstAudioTaskPara);
				break;
			case MSG_AUD_DECODE_NEXT_FRAME:
				audio_decode_next_frame(pstAudioTaskPara);
				break;
			case MSG_AUD_PARSE:
                audio_start_parse(pstAudioTaskPara);
				break;
			default:
				break;
		}
	}
}

static void audio_init(void)
{
	audio_ctrl.ring_buf = (INT8U*) gp_iram_malloc(RING_BUF_SIZE);
	if (audio_ctrl.ring_buf == NULL) {
		audio_ctrl.ring_buf = (INT8U*) gp_malloc(RING_BUF_SIZE);
	}

	audio_ctrl.ring_size = RING_BUF_SIZE;
	audio_ctrl.wi = 0;
	audio_ctrl.ri = 0;
	g_audio_main_channel = 0;
	audio_context_p->state = AUDIO_PLAY_STOP;
	stopped = 1;
	audio_rampdown_disable_flag = 0;
	decode_end = NULL;
}

static void audio_start_parse(STAudioTaskPara *pAudioTaskPara)
{
	INT32S ret;

	if (audio_context_p->state != AUDIO_PLAY_STOP) {
		audio_stop_unfinished();
	}

    audio_ctrl.file_handle = -1;
	stopped = 1;
	audio_queue_clear();

	audio_context_p->source_type = pAudioTaskPara->src_type;
    g_audio_main_channel = pAudioTaskPara->Main_Channel;
	if (audio_context_p->source_type == AUDIO_SRC_TYPE_FS) {
		ret = audio_play_file_set(pAudioTaskPara->fd, audio_context_p, &audio_ctrl, audio_fsq);
	}
	else if (audio_context_p->source_type == AUDIO_SRC_TYPE_FS_OGGMIX) {
        audio_ctrl.file_len = pAudioTaskPara->file_len;
		audio_ctrl.file_handle = pAudioTaskPara->fd;
		audio_context_p->audio_format = AUDIO_TYPE_OGG;
		ret = AUDIO_ERR_NONE;
	}
	else if (audio_context_p->source_type == AUDIO_SRC_TYPE_NAND) {
        audio_ctrl.file_len = pAudioTaskPara->file_len;
        audio_context_p->audio_format = pAudioTaskPara->audio_format;
        ret = AUDIO_ERR_NONE;
	}
	else if (audio_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
	{
		audio_context_p->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_context_p->audio_format == AUDIO_TYPE_NONE) {
		   DBG_PRINT("audio_play_file_set find not support audio.\r\n");
	   	}
	   	audio_ctrl.file_handle = pAudioTaskPara->fd;
		audio_ctrl.file_len = pAudioTaskPara->file_len;
		ret = AUDIO_ERR_NONE;
	}
	else if(audio_context_p->source_type == AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE)	// added by Bruce, 2010/01/22
	{
		audio_context_p->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_context_p->audio_format == AUDIO_TYPE_NONE) {
		   DBG_PRINT("audio_play_file_set find not support audio.\r\n");
	   	}
		audio_ctrl.file_handle = pAudioTaskPara->fd;
		audio_ctrl.file_len = pAudioTaskPara->file_len;
		ret = AUDIO_ERR_NONE;
	}
	else {
		ret = AUDIO_ERR_NONE;
		audio_context_p->audio_format = pAudioTaskPara->audio_format;
		audio_ctrl.file_len = pAudioTaskPara->file_len;
		audio_ctrl.file_handle = pAudioTaskPara ->fd;
	}

	if (ret == AUDIO_ERR_NONE) {
		if (audio_mem_alloc(audio_context_p->audio_format,&audio_ctrl,pcm_out) != AUDIO_ERR_NONE) {
			DBG_PRINT("audio memory allocate fail\r\n");
			audio_send_result(MSG_AUD_PLAY_RES,AUDIO_ERR_MEM_ALLOC_FAIL);
			if ((audio_ctrl.file_handle >= 0)&&(audio_context_p->source_type == AUDIO_SRC_TYPE_FS))
			{
                fs_close(audio_ctrl.file_handle);
                audio_ctrl.file_handle = -1;
            }
			return;
		}

		audio_context_p->state = AUDIO_PLAYING;

		switch(audio_context_p->audio_format) {
		#if APP_MP3_DECODE_FG_EN == 1
			case AUDIO_TYPE_MP3:
				audio_context_p->fp_deocde_init = audio_mp3_play_init;
				audio_context_p->fp_deocde = audio_mp3_process;
				audio_ctrl.ring_size = RING_BUF_SIZE;
				audio_ctrl.frame_size = MP3_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_WAV_CODEC_FG_EN == 1
			case AUDIO_TYPE_WAV:
				audio_context_p->fp_deocde_init = audio_wav_dec_play_init;
				audio_context_p->fp_deocde = audio_wav_dec_process;
				audio_ctrl.ring_size = WAV_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = WAV_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_WMA_DECODE_FG_EN == 1
			case AUDIO_TYPE_WMA:
				audio_context_p->fp_deocde_init = audio_wma_play_init;
				audio_context_p->fp_deocde = audio_wma_process;
				audio_ctrl.ring_size = RING_BUF_SIZE;
				audio_ctrl.frame_size = WMA_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A1800_DECODE_FG_EN == 1
			case AUDIO_TYPE_A1800:
				audio_context_p->fp_deocde_init = audio_a1800_play_init;
				audio_context_p->fp_deocde = audio_a1800_process;
				audio_ctrl.ring_size = A18_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = A1800_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A1600_DECODE_FG_EN == 1
			case AUDIO_TYPE_A1600:
				audio_context_p->fp_deocde_init = audio_a16_play_init;
				audio_context_p->fp_deocde = audio_a16_process;
				audio_ctrl.ring_size = A16_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = A1600_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A6400_DECODE_FG_EN == 1
			case AUDIO_TYPE_A6400:
				audio_context_p->fp_deocde_init = audio_a64_play_init;
				audio_context_p->fp_deocde = audio_a64_process;
				audio_ctrl.ring_size = RING_BUF_SIZE;
				audio_ctrl.frame_size = A6400_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_S880_DECODE_FG_EN == 1
			case AUDIO_TYPE_S880:
				audio_context_p->fp_deocde_init = audio_s880_play_init;
				audio_context_p->fp_deocde = audio_s880_process;
				audio_ctrl.ring_size = S880_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = S880_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_AAC_DECODE_FG_EN == 1
			case AUDIO_TYPE_AAC:
				audio_context_p->fp_deocde_init = audio_aac_play_init;
				audio_context_p->fp_deocde = audio_aac_process;
				audio_ctrl.ring_size = AAC_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = AAC_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_OGG_DECODE_FG_EN == 1
			case AUDIO_TYPE_OGG:
				audio_context_p->fp_deocde_init = audio_ogg_play_init;
				audio_context_p->fp_deocde = audio_ogg_process;
				audio_ctrl.ring_size = OGGVORBIS_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = OGG_PCM_BUF_SIZE;
				break;
		#endif
			default:
				audio_send_result(MSG_AUD_BG_PLAY_RES,AUDIO_ERR_INVALID_FORMAT);
				if ((audio_ctrl.file_handle >= 0)&&(audio_context_p->source_type == AUDIO_SRC_TYPE_FS))
				{
                    fs_close(audio_ctrl.file_handle);
                    audio_ctrl.file_handle = -1;
                }
				return;
		}

		ret = audio_context_p->fp_deocde_init();
		if (ret != AUDIO_ERR_NONE) {
			audio_stop_unfinished();
			DBG_PRINT("audio play init failed\r\n");
        } else {
        	//audio_send_next_frame_q();
		}
	} else {
   		ret = AUDIO_ERR_INVALID_FORMAT;
   		if ((pAudioTaskPara->fd >= 0)&&(audio_context_p->source_type == AUDIO_SRC_TYPE_FS))
   		{
            fs_close(pAudioTaskPara->fd);
            pAudioTaskPara->fd = -1;
        }
	}

	audio_send_result(MSG_AUD_PLAY_RES,ret);
}


static void audio_start(STAudioTaskPara *pAudioTaskPara)
{
	INT32S ret;

	if (audio_context_p->state != AUDIO_PLAY_STOP) {
		audio_stop_unfinished();
	}

    audio_ctrl.file_handle = -1;
	stopped = 1;
	audio_queue_clear();

	audio_context_p->source_type = pAudioTaskPara->src_type;
    g_audio_main_channel = pAudioTaskPara->Main_Channel;
	if (audio_context_p->source_type == AUDIO_SRC_TYPE_FS) {
		ret = audio_play_file_set(pAudioTaskPara->fd, audio_context_p, &audio_ctrl, audio_fsq);
	}
	else if (audio_context_p->source_type == AUDIO_SRC_TYPE_FS_OGGMIX) {
        audio_ctrl.file_len = pAudioTaskPara->file_len;
		audio_ctrl.file_handle = pAudioTaskPara->fd;
		audio_context_p->audio_format = AUDIO_TYPE_OGG;
		ret = AUDIO_ERR_NONE;
	}
	else if (audio_context_p->source_type == AUDIO_SRC_TYPE_NAND) {
        audio_ctrl.file_len = pAudioTaskPara->file_len;
        audio_context_p->audio_format = pAudioTaskPara->audio_format;
        ret = AUDIO_ERR_NONE;
	}
	else if (audio_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
	{
		audio_context_p->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_context_p->audio_format == AUDIO_TYPE_NONE) {
		   DBG_PRINT("audio_play_file_set find not support audio.\r\n");
	   	}
	   	audio_ctrl.file_handle = pAudioTaskPara->fd;
		audio_ctrl.file_len = pAudioTaskPara->file_len;
		ret = AUDIO_ERR_NONE;
	}
	else if(audio_context_p->source_type == AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE)	// added by Bruce, 2010/01/22
	{
		audio_context_p->audio_format = pAudioTaskPara->audio_format;
	   	if (audio_context_p->audio_format == AUDIO_TYPE_NONE) {
		   DBG_PRINT("audio_play_file_set find not support audio.\r\n");
	   	}
		audio_ctrl.file_handle = pAudioTaskPara->fd;
		audio_ctrl.file_len = pAudioTaskPara->file_len;
		ret = AUDIO_ERR_NONE;
	}
	else {
		ret = AUDIO_ERR_NONE;
		audio_context_p->audio_format = pAudioTaskPara->audio_format;
		audio_ctrl.file_len = pAudioTaskPara->file_len;
		audio_ctrl.file_handle = pAudioTaskPara ->fd;
	}

	if (ret == AUDIO_ERR_NONE) {
		if (audio_mem_alloc(audio_context_p->audio_format,&audio_ctrl,pcm_out) != AUDIO_ERR_NONE) {
			DBG_PRINT("audio memory allocate fail\r\n");
			audio_send_result(MSG_AUD_PLAY_RES,AUDIO_ERR_MEM_ALLOC_FAIL);
			if ((audio_ctrl.file_handle >= 0)&&(audio_context_p->source_type == AUDIO_SRC_TYPE_FS))
			{
                fs_close(audio_ctrl.file_handle);
                audio_ctrl.file_handle = -1;
            }
			return;
		}

		audio_context_p->state = AUDIO_PLAYING;

		switch(audio_context_p->audio_format) {
		#if APP_MP3_DECODE_FG_EN == 1
			case AUDIO_TYPE_MP3:
				audio_context_p->fp_deocde_init = audio_mp3_play_init;
				audio_context_p->fp_deocde = audio_mp3_process;
				audio_ctrl.ring_size = RING_BUF_SIZE;
				audio_ctrl.frame_size = MP3_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_WAV_CODEC_FG_EN == 1
			case AUDIO_TYPE_WAV:
				audio_context_p->fp_deocde_init = audio_wav_dec_play_init;
				audio_context_p->fp_deocde = audio_wav_dec_process;
				audio_ctrl.ring_size = WAV_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = WAV_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_WMA_DECODE_FG_EN == 1
			case AUDIO_TYPE_WMA:
				audio_context_p->fp_deocde_init = audio_wma_play_init;
				audio_context_p->fp_deocde = audio_wma_process;
				audio_ctrl.ring_size = RING_BUF_SIZE;
				audio_ctrl.frame_size = WMA_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A1800_DECODE_FG_EN == 1
			case AUDIO_TYPE_A1800:
				audio_context_p->fp_deocde_init = audio_a1800_play_init;
				audio_context_p->fp_deocde = audio_a1800_process;
				audio_ctrl.ring_size = A18_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = A1800_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A1600_DECODE_FG_EN == 1
			case AUDIO_TYPE_A1600:
				audio_context_p->fp_deocde_init = audio_a16_play_init;
				audio_context_p->fp_deocde = audio_a16_process;
				audio_ctrl.ring_size = A16_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = A1600_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_A6400_DECODE_FG_EN == 1
			case AUDIO_TYPE_A6400:
				audio_context_p->fp_deocde_init = audio_a64_play_init;
				audio_context_p->fp_deocde = audio_a64_process;
				audio_ctrl.ring_size = RING_BUF_SIZE;
				audio_ctrl.frame_size = A6400_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_S880_DECODE_FG_EN == 1
			case AUDIO_TYPE_S880:
				audio_context_p->fp_deocde_init = audio_s880_play_init;
				audio_context_p->fp_deocde = audio_s880_process;
				audio_ctrl.ring_size = S880_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = S880_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_AAC_DECODE_FG_EN == 1
			case AUDIO_TYPE_AAC:
				audio_context_p->fp_deocde_init = audio_aac_play_init;
				audio_context_p->fp_deocde = audio_aac_process;
				audio_ctrl.ring_size = AAC_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = AAC_PCM_BUF_SIZE;
				break;
		#endif
		#if APP_OGG_DECODE_FG_EN == 1
			case AUDIO_TYPE_OGG:
				audio_context_p->fp_deocde_init = audio_ogg_play_init;
				audio_context_p->fp_deocde = audio_ogg_process;
				audio_ctrl.ring_size = OGGVORBIS_DEC_BITSTREAM_BUFFER_SIZE;
				audio_ctrl.frame_size = OGG_PCM_BUF_SIZE;
				break;
		#endif
			default:
				audio_send_result(MSG_AUD_BG_PLAY_RES,AUDIO_ERR_INVALID_FORMAT);
				if ((audio_ctrl.file_handle >= 0)&&(audio_context_p->source_type == AUDIO_SRC_TYPE_FS))
				{
                    fs_close(audio_ctrl.file_handle);
                    audio_ctrl.file_handle = -1;
                }
				return;
		}

		ret = audio_context_p->fp_deocde_init();
		if (ret != AUDIO_ERR_NONE) {
			audio_stop_unfinished();
			DBG_PRINT("audio play init failed\r\n");
        } else {
        	audio_send_next_frame_q();
		}
	} else {
   		ret = AUDIO_ERR_INVALID_FORMAT;
   		if ((pAudioTaskPara->fd >= 0)&&(audio_context_p->source_type == AUDIO_SRC_TYPE_FS))
   		{
            fs_close(pAudioTaskPara->fd);
            pAudioTaskPara->fd = -1;
        }
	}

	audio_send_result(MSG_AUD_PLAY_RES,ret);
}

static void audio_stop(STAudioTaskPara *pAudioTaskPara)
{
	if ((audio_context_p->state == AUDIO_PLAY_STOP)||(audio_context_p->state == AUDIO_IDLE)) {
		audio_send_result(MSG_AUD_STOP_RES,AUDIO_ERR_NONE);
		return;
	}
#if 0//TBD: ramp down hangup issue
	if(audio_rampdown_disable_flag == 0) {
		if (audio_context_p->state == AUDIO_PLAYING) {
		#if (USE_RAMP_DOWN_RAPID == 0)
			audio_ramp_down();		//ramp down speed normal
		#else
			audio_ramp_down_rapid();
		#endif
		}
	}
#endif
	audio_stop_unfinished();
	audio_send_result(MSG_AUD_STOP_RES,AUDIO_ERR_NONE);
}

static void audio_ramp_down(void)
{
	INT8U   wb_idx;
	INT8U   err, ramp_down_step;
	INT16S  *ptr;
	INT16S  last_ldata,last_rdata;
	INT32U  i, j, buf_len = audio_ctrl.frame_size >> 1;

	if(g_audio_sample_rate > 16000) {
		ramp_down_step = RAMP_DOWN_STEP;
	} else {
		ramp_down_step = RAMP_DOWN_STEP_LOW_SR;
	}

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQFlush(aud_send_q);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueReset(aud_send_q);
#endif
    last_ldata = *(pcm_out[last_send_idx] + pcm_len[last_send_idx]-2);
	last_rdata = *(pcm_out[last_send_idx] + pcm_len[last_send_idx]-1);

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
        err = (INT8U) xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
        if(err != pdPASS) {
            continue;
        }
    #endif
		ptr = (INT16S*) pcm_out[wb_idx];
		audio_send_to_dma();

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

		pcm_len[wb_idx] = buf_len * channel;

    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(aud_send_q, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
    #endif
        if (drv_l1_dac_dma_status_get() == 0) {
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_DMA_DBF_RESTART);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            INT32U message = MSG_AUD_DMA_DBF_RESTART;
            xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
        #endif
        }

		if ((last_ldata == 0x0) && (last_rdata == 0x0)) {
			break;
		}
	}
}

#if USE_RAMP_DOWN_RAPID == 1
static void audio_ramp_down_rapid(void)
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


static void audio_pause(STAudioTaskPara *pAudioTaskPara)
{
#if (_OPERATING_SYSTEM == _OS_FREERTOS)
    INT32U message;
#endif
	if (audio_context_p->state != AUDIO_PLAYING) {
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
    while(drv_l1_dac_dma_status_get() || drv_l1_dac_dbf_status_get()) {
		vTaskDelay(20);
	}
#endif


	stopped = 1;
	drv_l1_dac_dbf_free();
	//dac_enable_set(FALSE);

	audio_context_p->state = AUDIO_PLAY_PAUSE;
	audio_send_result(MSG_AUD_PAUSE_RES, AUDIO_ERR_NONE);
}

static void audio_resume(STAudioTaskPara *pAudioTaskPara)
{
	if (audio_context_p->state != AUDIO_PLAY_PAUSE) {
		audio_send_result(MSG_AUD_RESUME_RES, AUDIO_ERR_NONE);
		return;
	}
	//dac_enable_set(TRUE);
	audio_context_p->state = AUDIO_PLAYING;
	audio_send_next_frame_q();
	audio_send_result(MSG_AUD_RESUME_RES,AUDIO_ERR_NONE);
}

static void audio_mute_set(STAudioTaskPara *pAudioTaskPara)
{
	if (pAudioTaskPara->mute == TRUE) {
		drv_l1_dac_pga_set(0);
		drv_l1_dac_vref_set(FALSE);
	}
	else {
		drv_l1_dac_pga_set(pAudioTaskPara->volume);
		drv_l1_dac_vref_set(TRUE);
	}
	audio_send_result(MSG_AUD_MUTE_SET_RES,AUDIO_ERR_NONE);
}

static void audio_volume_set(STAudioTaskPara *pAudioTaskPara)
{
	if (pAudioTaskPara->volume < 64) {
		drv_l1_dac_pga_set(pAudioTaskPara->volume);
	}
	audio_send_result(MSG_AUD_VOLUME_SET_RES,AUDIO_ERR_NONE);
}

static void audio_decode_next_frame(STAudioTaskPara *pAudioTaskPara)
{
	INT32S ret;
	if (audio_context_p->state != AUDIO_PLAYING) {
		return;
	}
	ret = audio_context_p->fp_deocde();
	if (ret != 0) {
		audio_stop_unfinished();
		if (decode_end != NULL) {
			decode_end(1);	// added by Bruce, 2008/10/03
		}
	}
}

static void audio_send_next_frame_q(void)
{
	//DBG_PRINT(".");
	msgQSend(AudioTaskQ, MSG_AUD_DECODE_NEXT_FRAME, NULL, 0, MSG_PRI_NORMAL);
}

void audio_send_result(INT32S res_type,INT32S result)
{
	aud_con.result_type = res_type;
	aud_con.result = result;
	DBG_PRINT("audio_send_result :res_type =  %x,result = %d\r\n",res_type,result);
	msgQSend(Audio_FG_status_Q, EVENT_APQ_ERR_MSG, (void *)&aud_con, sizeof(STAudioConfirm), MSG_PRI_NORMAL);
}

static void audio_stop_unfinished(void)
{
	INT32S i;

    if(0 <= gEncode_Filehandle)
    {
        Save_Last_PCM_File();
    }
	/* wait until dma finish */
	while(drv_l1_dac_dma_status_get() == 1) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(2);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        vTaskDelay(20);
    #endif
	}

	if (audio_context_p->source_type == AUDIO_SRC_TYPE_FS
	    ||audio_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE) {
		if (audio_ctrl.file_handle >= 0) {
			fs_close(audio_ctrl.file_handle); //who open ,who close
			audio_ctrl.file_handle = -1;
		}
	}
#if (MCU_VERSION != GPM41XXA) && (_DRV_L1_HDMI == 1)
	if(g_audio_main_channel==3)
	{
        drvl1_hdmi_audio_ctrl(0);
	}
	else
#endif
	{
        drv_l1_dac_timer_stop();
        drv_l1_dac_dbf_free(); /* release dma channel */
        drv_l1_dac_hw_up_sample_uninit();
	}

	//drv_l1_dac_enable_set(FALSE);

	/* free memory */
	if((audio_context_p->audio_format != AUDIO_TYPE_AVI) && (audio_context_p->audio_format != AUDIO_TYPE_MP3) )
	{
		gp_free(audio_ctrl.work_mem);
	}
	audio_ctrl.work_mem = NULL;

	for (i=0;i<MAX_DAC_BUFFERS;i++) {
		gp_free(*(pcm_out+i));
		*(pcm_out+i) = NULL;
	}

	audio_context_p->state = AUDIO_PLAY_STOP;
	audio_play_start_time = 0;
}

INT32S audio_mem_alloc(INT32U audio_format,AUDIO_CTRL *aud_ctrl, INT16S *pcm[])
{
	INT32S i;
	INT32U pcm_size;
	INT32U wm_size;

	switch(audio_format) {
	#if APP_MP3_DECODE_FG_EN == 1 || APP_MP3_DECODE_BG_EN == 1
		case AUDIO_TYPE_MP3:
			wm_size = MP3_DEC_MEMORY_SIZE + MP3_DECODE_RAM;
#if APP_UP_SAMPLE_EN == 1
		if(hUpSample == NULL)
			pcm_size = MP3_PCM_BUF_SIZE;
        else
            pcm_size = MP3_PCM_BUF_SIZE*2;
#else
            pcm_size = MP3_PCM_BUF_SIZE;
#endif

			break;
	#endif
	#if APP_WMA_DECODE_FG_EN == 1 || APP_WMA_DECODE_BG_EN == 1
		case AUDIO_TYPE_WMA:
			wm_size = WMA_DEC_MEMORY_SIZE;
			pcm_size = WMA_PCM_BUF_SIZE;
			break;
	#endif
	#if APP_WAV_CODEC_FG_EN == 1 || APP_WAV_CODEC_BG_EN == 1
		case AUDIO_TYPE_WAV:
			wm_size = WAV_DEC_MEMORY_SIZE;
			pcm_size = WAV_PCM_BUF_SIZE;
			break;
	#endif
		case AUDIO_TYPE_AVI:
			wm_size = WAV_DEC_MEMORY_SIZE;
			pcm_size = WAV_PCM_BUF_SIZE;
			break;
		case AUDIO_TYPE_AVI_MP3:
			wm_size = MP3_DEC_MEMORY_SIZE;
			pcm_size = MP3_PCM_BUF_SIZE;
			break;
	#if APP_A1800_DECODE_FG_EN == 1 || APP_A1800_DECODE_BG_EN == 1
		case AUDIO_TYPE_A1800:
			wm_size = A1800DEC_MEMORY_BLOCK_SIZE;
			pcm_size = A1800_PCM_BUF_SIZE;
			break;
	#endif
	#if APP_A1600_DECODE_FG_EN == 1 || APP_A1600_DECODE_BG_EN == 1
 	  	case AUDIO_TYPE_A1600:			// added by Bruce, 2008/09/23
  	  		wm_size = A16_DEC_MEMORY_SIZE;
  	  		pcm_size = A1600_PCM_BUF_SIZE;
  	  		break;
	#endif
	#if APP_A6400_DECODE_FG_EN == 1 || APP_A6400_DECODE_BG_EN == 1
 	  	case AUDIO_TYPE_A6400:			// added by Bruce, 2008/09/25
			wm_size = A6400_DEC_MEMORY_SIZE + A6400_DECODE_RAM;
  	  		pcm_size = A6400_PCM_BUF_SIZE;
  	  		break;
	#endif
	#if APP_S880_DECODE_FG_EN == 1 || APP_S880_DECODE_BG_EN == 1
 	  	case AUDIO_TYPE_S880:			// added by Bruce, 2008/09/25
  	  		wm_size = S880_DEC_MEMORY_SIZE;
  	  		pcm_size = S880_PCM_BUF_SIZE;
  	  		break;
	#endif
	#if APP_AAC_DECODE_FG_EN == 1 || APP_AAC_DECODE_BG_EN == 1
 	  	case AUDIO_TYPE_AAC:			// added by George, 2009/8/13
  	  		wm_size = AAC_DEC_MEMORY_BLOCK_SIZE;
  	  		pcm_size = AAC_PCM_BUF_SIZE;
  	  		break;
	#endif
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
	} else if(audio_format == AUDIO_TYPE_MP3) {
        aud_ctrl->work_mem = (INT8S*) MP3_WORKING_MEM;
	} else {
		aud_ctrl->work_mem = (INT8S*) gp_malloc(wm_size);
		if (aud_ctrl->work_mem == NULL) {
			return AUDIO_ERR_FAILED;
		}
	}
	gp_memset(aud_ctrl->work_mem,0,wm_size);
	DBG_PRINT("decode memory = 0x%x (%d)\r\n",aud_ctrl->work_mem,wm_size);

	pcm_size += 2 /* add for SPU end data */;
	for (i=0;i<MAX_DAC_BUFFERS;i++) {
		pcm[i] = (INT16S*) gp_malloc(pcm_size*2);
		DBG_PRINT("pcm buffer[%d] = 0x%x (%d)\r\n",i,pcm[i],pcm_size*2);
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

#if (_OPERATING_SYSTEM == _OS_UCOS2)
INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, OS_EVENT *ack_fsq)
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
INT32S audio_play_file_set(INT16S fd, AUDIO_CONTEXT *aud_context, AUDIO_CTRL *aud_ctrl, xQueueHandle ack_fsq)
#endif
{
#if (defined SKIP_ID3_TAG) && (SKIP_ID3_TAG == 1)
	INT8U id3[10];
	INT32U len;
#endif

	sfn_stat(fd,&aud_sfn_file);
#if (defined AUDIO_FORMAT_JUDGE_AUTO) && (AUDIO_FORMAT_JUDGE_AUTO == 1)
	aud_context->audio_format = audio_get_type(fd, aud_sfn_file.f_extname, ack_fsq);
#else
	aud_context->audio_format = audio_get_type(aud_sfn_file.f_extname);
#endif
   	if (aud_context->audio_format == AUDIO_TYPE_NONE) {
	   DBG_PRINT("audio_play_file_set find not support audio.\r\n");
   	   return AUDIO_ERR_INVALID_FORMAT;
   	}
	aud_ctrl->file_handle = fd;

#if (defined SKIP_ID3_TAG) && (SKIP_ID3_TAG == 1) /* skip id3 */
    if (aud_context->audio_format == AUDIO_TYPE_MP3) {
    	read(fd,(INT32U)id3,10);
    	len = audio_id3_get_tag_len(id3, 10);
    	aud_ctrl->file_len = aud_sfn_file.f_size-(len&~3);
   	   	lseek(fd,0,SEEK_SET);
   	   	lseek(fd,(len&~3),SEEK_SET); /* 4 byte alignment */
   	}
   	else {
    	aud_ctrl->file_len = aud_sfn_file.f_size;
    }
#else

	aud_ctrl->file_len = aud_sfn_file.f_size;

#endif

	return AUDIO_ERR_NONE;
}

#if (defined AUDIO_FORMAT_JUDGE_AUTO) && (AUDIO_FORMAT_JUDGE_AUTO == 1)
#if APP_WAV_CODEC_FG_EN == 1 || APP_WAV_CODEC_BG_EN == 1 || APP_MP3_DECODE_FG_EN == 1 || APP_MP3_DECODE_BG_EN == 1 || APP_WMA_DECODE_FG_EN == 1 || APP_WMA_DECODE_BG_EN == 1
static  INT8S parse_mp3_file_head(INT8S *p_audio_file_head_ptr)
{
	INT8S  mpeg_version, layer,sample_rate;
	INT8S j =0;
	INT32U ID3V2_length = 0;
	INT32U cnt  = AUDIO_PASER_BUFFER_SIZE;
#if 1// for new parser
    MP3_SYNC_UNION  mp3_file_head[2];
	INT8U *p_audio_file_head;
	INT32U BitRate;
	INT32U SampleRate;
	INT32U Padding;
	INT32U FrameLengthInBytes;
#else//old
	MP3_FILE_HEAD  mp3_file_head[2];
	INT8U *p_audio_file_head;
#endif

	p_audio_file_head = (INT8U *)p_audio_file_head_ptr;

	if(*(p_audio_file_head) == (INT8S)'I' && *(p_audio_file_head + 1) == (INT8S)'D' && *(p_audio_file_head + 2) == (INT8S)'3' )
	{
		ID3V2_length = 10 + ((*(p_audio_file_head + 9)& 0x7f)|((*(p_audio_file_head + 8) & 0x7f)<<7)|((*(p_audio_file_head + 7) & 0x7f)<<14)|((*(p_audio_file_head + 6) & 0x7f)<<21));
		//ID3V2_length += 10 + ((*(p_audio_file_head + 6) & 0x7f)<<21);
		//ID3V2_length += ((*(p_audio_file_head + 7) & 0x7f)<<14);
		//ID3V2_length += ((*(p_audio_file_head + 8) & 0x7f)<<7);
		//ID3V2_length +=  (*(p_audio_file_head + 9)& 0x7f);

		/*if(ID3V2_length > 1024)
		{
			DBG_PRINT("audio file ID3V2 too long \r\n");
			return AUDIO_PARSE_SUCCS ;
		}
		else
		{
			p_audio_file_head +=  ID3V2_length;
			cnt -= ID3V2_length;
		}
		*/
		DBG_PRINT("audio file parse succes \r\n");
		return AUDIO_PARSE_SUCCS ;
	}

	while (cnt > 4)
	{
#if 1//for new parser
        if(	mp3_file_head[0].sync_word.sync		    == 0x07FF	&&
        		mp3_file_head[0].sync_word.mpeg		!= 1		&&
        		mp3_file_head[0].sync_word.layer	!= 0		&&
        		mp3_file_head[0].sync_word.layer	!= 3		&&
        		mp3_file_head[0].sync_word.bitrate	!= 0		&&
        		mp3_file_head[0].sync_word.bitrate	!= 15		&&
        		mp3_file_head[0].sync_word.samplerate	!= 3)
		{
				BitRate = table_BitRate[mp3_file_head[0].sync_word.mpeg][mp3_file_head[0].sync_word.layer][mp3_file_head[0].sync_word.bitrate] * 1000;
				SampleRate = table_SampleRate[mp3_file_head[0].sync_word.mpeg][mp3_file_head[0].sync_word.samplerate];
				Padding = mp3_file_head[0].sync_word.padding;
				if(mp3_file_head[0].sync_word.mpeg == MPEG_V1)
					FrameLengthInBytes = 144 * BitRate / SampleRate + Padding; // 144 = 1152/8
				else
					FrameLengthInBytes = 72 * BitRate / SampleRate + Padding;
				mp3_file_head[1].ch[0] = *(p_audio_file_head + FrameLengthInBytes);
		        mp3_file_head[1].ch[1] = *(p_audio_file_head + FrameLengthInBytes - 1);
		        mp3_file_head[1].ch[2] = *(p_audio_file_head + FrameLengthInBytes - 2);
		        mp3_file_head[1].ch[3] = *(p_audio_file_head + FrameLengthInBytes - 3);
				if((mp3_file_head[0].word & EQUAL_MASK) == (mp3_file_head[1].word & EQUAL_MASK))
				{
				    DBG_PRINT("MP3 parse succes \r\n");
				    DBG_PRINT("MPEG%d,Layer%d,BitRate : %d,SampleRate : %d \r\n",table_MpegVersion[mp3_file_head[0].sync_word.mpeg]
				                 ,(4 - mp3_file_head[0].sync_word.layer),BitRate,SampleRate);
					return AUDIO_PARSE_SUCCS ;
				}
		}
		p_audio_file_head ++;
		cnt -- ;
		mp3_file_head[0].word <<= 8;
		mp3_file_head[0].ch[0] = *(p_audio_file_head);
#else//old
		if((*(p_audio_file_head) == 0xFF ) && ((*(p_audio_file_head + 1)&0xE0) == 0xE0 )  && ((*(p_audio_file_head + 2)&0xF0 )!= 0xF0 ))  // first 11 bits should be 1
		{
			mpeg_version =( *(p_audio_file_head + 1)&0x18)>>3;
			layer = (*(p_audio_file_head + 1)&0x06)>>1;
			sample_rate = (*(p_audio_file_head + 2)&0x0c)>>2;

			if((mpeg_version != 0x01) && (layer != 0x00) && (layer != 0x03) && (sample_rate != 0x03))   // != RESERVERD
			{
				if(j<2)
				{
					mp3_file_head[j].mpeg_version = mpeg_version;
					mp3_file_head[j].layer = layer;
					mp3_file_head[j].sample_rate = sample_rate;

					j++;
				}
				else if ((mp3_file_head[0].mpeg_version == mp3_file_head[1].mpeg_version) && (mp3_file_head[0].layer == mp3_file_head[1].layer) && (mp3_file_head[0].sample_rate == mp3_file_head[1].sample_rate))
				{
					DBG_PRINT("audio file parse succes \r\n");
					return  AUDIO_PARSE_SUCCS ;
				}
			}
			else
			{
				p_audio_file_head +=  4;
				cnt -= 4 ;
			}
		}
		else
		{
			p_audio_file_head +=  4;
			cnt -= 4 ;
		}
#endif
	}
    DBG_PRINT("MP3 parse fail \r\n");
	return AUDIO_PARSE_FAIL;
}

static  INT8S parse_wma_file_head(INT8S *p_audio_file_head)
{
	INT32U *pData = (INT32U *)p_audio_file_head;

	if (*pData++ != 0x75B22630) {
		return AUDIO_PARSE_FAIL;
	} else if (*pData++ != 0x11CF668E) {
		return AUDIO_PARSE_FAIL;
	} else if (*pData++ != 0xAA00D9A6) {
		return AUDIO_PARSE_FAIL;
	} else if(*pData++ != 0x6CCE6200) {
		return AUDIO_PARSE_FAIL;
	}

	return AUDIO_PARSE_SUCCS;
}

static  INT8S parse_wav_file_head(INT8S *p_audio_file_head)
{
	INT16U wave_format;
	INT32U temp;

	// Chunk ID
	temp = READSTRING(p_audio_file_head);
	if (temp != MAKEFOURCC('R', 'I', 'F', 'F')) {
		return AUDIO_PARSE_FAIL;
	}

	// RIFF Type
	p_audio_file_head += 8;
	temp = READSTRING(p_audio_file_head);
	if (temp != MAKEFOURCC('W', 'A', 'V', 'E')) {
		return AUDIO_PARSE_FAIL;
	}

	p_audio_file_head += 4;
	temp = READSTRING(p_audio_file_head);

	// Broadcast Audio Extension Chunk
	if (temp == MAKEFOURCC('b', 'e', 'x', 't')) {
		INT32S size;

		p_audio_file_head += 4;		//chunk ID
		size = *p_audio_file_head | (*(p_audio_file_head + 1) << 8) |
				(*(p_audio_file_head + 2) << 16) | (*(p_audio_file_head + 3) << 24);

		p_audio_file_head += 4;		//size of extension chunk
		p_audio_file_head += size;
		temp = READSTRING(p_audio_file_head);
	}

	// WAVE Format Chunk
	if (temp == MAKEFOURCC('f', 'm', 't', ' ')) {
		p_audio_file_head += 4;		//chunk ID
		p_audio_file_head += 4;		//chunk data size
		wave_format = *p_audio_file_head | (*(p_audio_file_head+1) << 8); //compression code
		switch(wave_format) {
			case WAVE_FORMAT_PCM:
			case WAVE_FORMAT_ADPCM:
			case WAVE_FORMAT_ALAW:
			case WAVE_FORMAT_MULAW:
			case WAVE_FORMAT_IMA_ADPCM:
				return AUDIO_PARSE_SUCCS;

			default:
				return AUDIO_PARSE_FAIL;
		}
	}

	return AUDIO_PARSE_FAIL;
}

#if (_OPERATING_SYSTEM == _OS_UCOS2)
static INT32S audio_real_type_get(INT16S fd, INT8S type_index, OS_EVENT *ack_fsq)
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
static INT32S audio_real_type_get(INT16S fd, INT8S type_index, xQueueHandle ack_fsq)
#endif
{
	INT8U err;
	INT16U i;
	INT32U audio_file_head;
	INT32S len, ret;
	TK_FILE_SERVICE_STRUCT audio_file_head_para;
	AUDIO_FILE_PARSE audio_file_parse_head[3];

	switch (type_index)
	{
		case AUDIO_TYPE_MP3:
			audio_file_parse_head[0].audio_file_real_type = AUDIO_TYPE_MP3;
			audio_file_parse_head[0].parse_audio_file_head = parse_mp3_file_head;

			audio_file_parse_head[1].audio_file_real_type = AUDIO_TYPE_WMA;
			audio_file_parse_head[1].parse_audio_file_head = parse_wma_file_head;

			audio_file_parse_head[2].audio_file_real_type = AUDIO_TYPE_WAV;
			audio_file_parse_head[2].parse_audio_file_head = parse_wav_file_head;
			break;

		case AUDIO_TYPE_WMA:
			audio_file_parse_head[0].audio_file_real_type = AUDIO_TYPE_WMA;
			audio_file_parse_head[0].parse_audio_file_head = parse_wma_file_head;

			audio_file_parse_head[1].audio_file_real_type = AUDIO_TYPE_WAV;
			audio_file_parse_head[1].parse_audio_file_head = parse_wav_file_head;

			audio_file_parse_head[2].audio_file_real_type = AUDIO_TYPE_MP3;
			audio_file_parse_head[2].parse_audio_file_head = parse_mp3_file_head;
			break;

		case AUDIO_TYPE_WAV:
			audio_file_parse_head[0].audio_file_real_type = AUDIO_TYPE_WAV;
			audio_file_parse_head[0].parse_audio_file_head = parse_wav_file_head;

			audio_file_parse_head[1].audio_file_real_type = AUDIO_TYPE_WMA;
			audio_file_parse_head[1].parse_audio_file_head = parse_wma_file_head;

			audio_file_parse_head[2].audio_file_real_type = AUDIO_TYPE_MP3;
			audio_file_parse_head[2].parse_audio_file_head = parse_mp3_file_head;
			break;

		default:
			return type_index;
	}

	audio_file_head_para.fd = fd;
	audio_file_head_para.result_queue = ack_fsq;
	audio_file_head_para.buf_size = AUDIO_PASER_BUFFER_SIZE;

	audio_file_head = (INT32U) gp_malloc_align((audio_file_head_para.buf_size), 4);
	if (audio_file_head == 0) {
		DBG_PRINT("audio file parse failed to allocate memory\r\n");
		return type_index;
	}

	audio_file_head_para.buf_addr = (INT32U) audio_file_head;

	lseek((INT16S)audio_file_head_para.fd, 0, SEEK_SET);

	msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *)&audio_file_head_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);


#if (_OPERATING_SYSTEM == _OS_UCOS2)
    len = (INT32S)OSQPend(ack_fsq, 200, &err);
	if ((err != OS_NO_ERR) || len < 0) {
		gp_free((void *) audio_file_head);
		return type_index;
	}
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    err = (INT8U)xQueueReceive(ack_fsq, &len, 2000);
    if((err != pdPASS) || (len < 0)) {
		gp_free((void *) audio_file_head);
		return type_index;
    }
#endif
	lseek((INT16S)audio_file_head_para.fd, 0, SEEK_SET);

	// check it is real the type_index,according the file extension to judge the file real type.
	ret = audio_file_parse_head[0].parse_audio_file_head((INT8S *)audio_file_head);
	if (ret == AUDIO_PARSE_SUCCS) {
		gp_free((void *) audio_file_head);
		return audio_file_parse_head[0].audio_file_real_type;
	}

	// check other support file type except current extension type
	for (i=1; i<sizeof(audio_file_parse_head)/sizeof(AUDIO_FILE_PARSE); i++) {
		ret = audio_file_parse_head[i].parse_audio_file_head((INT8S *)audio_file_head);
		if (ret == AUDIO_PARSE_SUCCS) {
			gp_free((void *) audio_file_head);
			return audio_file_parse_head[i].audio_file_real_type;
		}
	}

	gp_free((void *) audio_file_head);
	return AUDIO_TYPE_NONE;
}
#endif

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

#if APP_WAV_CODEC_FG_EN == 1 || APP_WAV_CODEC_BG_EN == 1
   	if(gp_strcmp(temp, (INT8S *)"WAV")==0) {
		//return AUDIO_TYPE_WAV;
		return audio_real_type_get(fd,AUDIO_TYPE_WAV, ack_fsq);
   	}
#endif

#if APP_MP3_DECODE_FG_EN == 1 || APP_MP3_DECODE_BG_EN == 1
   	if(gp_strcmp(temp, (INT8S *)"MP3")==0) {
		//return AUDIO_TYPE_MP3;
		return audio_real_type_get(fd,AUDIO_TYPE_MP3, ack_fsq);
	}
#endif

#if APP_WMA_DECODE_FG_EN == 1 || APP_WMA_DECODE_BG_EN == 1
	if(gp_strcmp(temp, (INT8S *)"WMA")==0) {
		//return AUDIO_TYPE_WMA;
		return audio_real_type_get(fd,AUDIO_TYPE_WMA, ack_fsq);
	}
#endif

#if APP_A1800_DECODE_FG_EN == 1 || APP_A1800_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"A18")==0) {
    	return AUDIO_TYPE_A1800;
    }
#endif

    if(gp_strcmp(temp, (INT8S *)"IDI")==0) {
    	return AUDIO_TYPE_MIDI;
    }

    if(gp_strcmp(temp, (INT8S *)"GMD")==0) {
    	return AUDIO_TYPE_MIDI;
    }

#if APP_A1600_DECODE_FG_EN == 1 || APP_A1600_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"A16")==0) {// added by Bruce, 2008/09/23
    	return AUDIO_TYPE_A1600;
    }
#endif

#if APP_A6400_DECODE_FG_EN == 1 || APP_A6400_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"A64")==0) {// added by Bruce, 2008/09/25
    	return AUDIO_TYPE_A6400;
    }
#endif

#if APP_S880_DECODE_FG_EN == 1 || APP_S880_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"S88")==0) {// added by Bruce, 2008/09/25
    	return AUDIO_TYPE_S880;
    }
#endif
#if APP_AAC_DECODE_FG_EN == 1 || APP_AAC_DECODE_BG_EN == 1
	if(gp_strcmp(temp, (INT8S *)"AAC")==0) {
		return AUDIO_TYPE_AAC;
	}
#endif

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

#if APP_WAV_CODEC_FG_EN == 1 || APP_WAV_CODEC_BG_EN == 1
   	if(gp_strcmp(temp, (INT8S *)"WAV")==0) {
      return AUDIO_TYPE_WAV;
    }
#endif

#if APP_MP3_DECODE_FG_EN == 1 || APP_MP3_DECODE_BG_EN == 1
   	if(gp_strcmp(temp, (INT8S *)"MP3")==0) {
    	return AUDIO_TYPE_MP3;
    }
#endif

#if APP_WMA_DECODE_FG_EN == 1 || APP_WMA_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"WMA")==0) {
    	return AUDIO_TYPE_WMA;
    }
#endif

#if APP_A1800_DECODE_FG_EN == 1 || APP_A1800_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"A18")==0) {
    	return AUDIO_TYPE_A1800;
    }
#endif

    if(gp_strcmp(temp, (INT8S *)"IDI")==0) {
    	return AUDIO_TYPE_MIDI;
    }

    if(gp_strcmp(temp, (INT8S *)"GMD")==0) {
    	return AUDIO_TYPE_MIDI;
    }

#if APP_A1600_DECODE_FG_EN == 1 || APP_A1600_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"A16")==0) { // added by Bruce, 2008/09/23
    	return AUDIO_TYPE_A1600;
    }
#endif

#if APP_A6400_DECODE_FG_EN == 1 || APP_A6400_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"A64")==0) { // added by Bruce, 2008/09/25
    	return AUDIO_TYPE_A6400;
    }
#endif

#if APP_S880_DECODE_FG_EN == 1 || APP_S880_DECODE_BG_EN == 1
    if(gp_strcmp(temp, (INT8S *)"S88")==0) { // added by Bruce, 2008/09/25
    	return AUDIO_TYPE_S880;
    }
#endif

#if APP_AAC_DECODE_FG_EN == 1 || APP_AAC_DECODE_BG_EN == 1
	if(gp_strcmp(temp, (INT8S *)"AAC")==0) {
		return AUDIO_TYPE_AAC;
	}
#endif

#if APP_OGG_DECODE_FG_EN == 1 || APP_OGG_DECODE_BG_EN == 1
	if(gp_strcmp(temp, (INT8S *)"OGG")==0) {
		return AUDIO_TYPE_OGG;
	}
#endif


   	return AUDIO_TYPE_NONE;
}
#endif

static void audio_queue_clear(void)
{
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	//OSQFlush(hAudioDacTaskQ);
	OSQFlush(aud_send_q);
	OSQFlush(audio_wq);
	OSQFlush(audio_fsq);
	OSQPost(hAudioDacTaskQ, (void *)MSG_AUD_DMA_WIDX_CLEAR);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    INT32U message = MSG_AUD_DMA_WIDX_CLEAR;

    //xQueueReset(hAudioDacTaskQ);
    xQueueReset(aud_send_q);
    xQueueReset(audio_wq);
    xQueueReset(audio_fsq);
    xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
#endif
}

#if APP_MP3_DECODE_FG_EN == 1 || APP_WMA_DECODE_FG_EN == 1 || APP_WAV_CODEC_FG_EN == 1 || APP_A1600_DECODE_FG_EN == 1 || APP_A1800_DECODE_FG_EN == 1 || APP_A6400_DECODE_FG_EN == 1 || APP_S880_DECODE_FG_EN == 1
INT32S audio_write_with_file_srv(INT8U *ring_buf, INT32U wi, INT32U ri)
{
#if 1
	INT32S t;
	INT32S len;
	INT8U  err;
	INT32U msg_id;
	TK_FILE_SERVICE_STRUCT audio_fs_para;

	INT8U  WAV_Header_Size = 0;
	INT8U  *ptr;
	INT32U SeekUnit;
	INT32U BytesPerSec;

	switch(audio_context_p->source_type) {
		case AUDIO_SRC_TYPE_FS:
			msg_id = MSG_FILESRV_FS_READ;
			break;
		case AUDIO_SRC_TYPE_FS_OGGMIX:
			msg_id = MSG_FILESRV_FS_READ;
			break;
		case AUDIO_SRC_TYPE_NAND:
			msg_id = MSG_FILESRV_NAND_READ;
			audio_fs_para.data_offset = audio_ctrl.data_offset;
			break;
		case AUDIO_SRC_TYPE_GPRS:
			msg_id = MSG_FILESRV_NVRAM_AUDIO_GPRS_READ;
			audio_fs_para.spi_para.sec_offset = audio_ctrl.read_secs;
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
			audio_fs_para.data_start_addr = audio_ctrl.data_start_addr;
			audio_fs_para.data_offset = audio_ctrl.data_offset;
			break;
		case AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE:	// added by Bruce, 2010/01/22
			msg_id = MSG_FILESRV_FS_READ;
			break;
		default:
			break;
	}

	audio_fs_para.fd = audio_ctrl.file_handle;
	audio_fs_para.result_queue = audio_fsq;
	audio_fs_para.main_channel = AUDIO_CHANNEL_DAC;

	if(wi == 0 && ri == 0) {
		audio_fs_para.buf_addr = (INT32U)ring_buf;
		audio_fs_para.buf_size = audio_ctrl.ring_size/2;
		audio_fs_para.spi_para.sec_cnt = audio_ctrl.ring_size/1024;

		if(msg_id == MSG_FILESRV_MAX)
		{
			if((G_SACM_Ctrl.Offsettype == SND_OFFSET_TYPE_SIZE)||(G_SACM_Ctrl.Offsettype == SND_OFFSET_TYPE_TIME))
			{
				if(G_SACM_Ctrl.AudioFormat == MP3)
				{
					if(G_SACM_Ctrl.Offset > audio_ctrl.file_len)
						return AUDIO_READ_FAIL;

					if(Snd_Lseek(G_SACM_Ctrl.Offset,SEEK_CUR) >= 0)
						audio_ctrl.file_cnt += G_SACM_Ctrl.Offset;
					else
						return AUDIO_READ_FAIL;

					len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
					audio_ctrl.read_secs += (len/512);
			        wi += len;
				}
				else if(G_SACM_Ctrl.AudioFormat == WMA)                  //add WMA_Seek
				{
				    len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
					audio_ctrl.read_secs += (len/512);
			        wi += len;
				}
				else if(G_SACM_Ctrl.AudioFormat == WAV)
				{
                    // use jmp to play
                    len = Snd_GetData(audio_fs_para.buf_addr,0x800);

                    //wav_dec_parsing((INT8U *)audio_ctrl.work_mem , audio_ctrl.wi);
                    ptr = (INT8U*)audio_fs_para.buf_addr;
                    ptr += 12;
                    while(1)
                    {
                        if((*ptr == 'f')&&(*(ptr + 1) == 'm')&&(*(ptr + 2) == 't')&&(*(ptr + 3) == 0x20))
                        {
                            //ptr = ptr + 20;
                            break;
                        }
                        else
                        {
                            ptr = ptr + 2;
                            WAV_Header_Size += 2;
                        }
                        if(WAV_Header_Size > 0x80)
                            return AUDIO_READ_FAIL;
                    }
                    SeekUnit = *(ptr +20) + *(ptr + 21) <<8;//wav_dec_get_nBlockAlign((INT8U*)audio_ctrl.work_mem);
                    BytesPerSec = *(ptr +16) + (*(ptr + 17)<<8) + (*(ptr +18)<<16) + (*(ptr + 19)<<24);

                    ptr = (INT8U*)audio_fs_para.buf_addr;
                    ptr += 36;

                    WAV_Header_Size = 36;
                    while(1)
                    {
                        if((*ptr == 'd')&&(*(ptr + 1) == 'a')&&(*(ptr + 2) == 't')&&(*(ptr + 3) == 'a'))
                        {
                            WAV_Header_Size  += 8;
                            break;
                        }
                        else
                        {
                            ptr = ptr + 2;
                            WAV_Header_Size += 2;
                        }
                        if(WAV_Header_Size > 0x80)
                            return AUDIO_READ_FAIL;
                    }


                    //if(G_SACM_Ctrl.Offsettype == SND_OFFSET_TYPE_SIZE)
                    //{
                    //	G_SACM_Ctrl.Offset = G_SACM_Ctrl.Offset/ SeekUnit;
                    //	G_SACM_Ctrl.Offset = G_SACM_Ctrl.Offset * SeekUnit;
                    //}
                    //else
                    if(G_SACM_Ctrl.Offsettype == SND_OFFSET_TYPE_TIME)
                    {
                        G_SACM_Ctrl.Offset = G_SACM_Ctrl.Offset*BytesPerSec;
                    }
                    G_SACM_Ctrl.Offset = G_SACM_Ctrl.Offset/ SeekUnit;
                    G_SACM_Ctrl.Offset = G_SACM_Ctrl.Offset * SeekUnit;
                    audio_fs_para.buf_addr += WAV_Header_Size;
                    audio_fs_para.buf_size -= WAV_Header_Size;


                    if(G_SACM_Ctrl.Offset > audio_ctrl.file_len)
                        return AUDIO_READ_FAIL;

                    audio_ctrl.file_cnt += len;
                    if(Snd_Lseek((G_SACM_Ctrl.Offset+WAV_Header_Size),SEEK_SET) >= 0)
                    {
                        audio_ctrl.file_cnt += G_SACM_Ctrl.Offset/* + WAV_Header_Size*/;
                    }
                    else
                        return AUDIO_READ_FAIL;

                    audio_ctrl.data_offset = audio_ctrl.file_cnt;

                    len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
                    len += WAV_Header_Size;
                    audio_ctrl.data_offset += len;
                    audio_ctrl.read_secs += (len/512);
                    wi += len;
				}
				else
				{	// use jmp to play
					len = Snd_GetData(audio_fs_para.buf_addr,6);
					audio_fs_para.buf_addr += 6;
					audio_fs_para.buf_size -= 6;
					if(G_SACM_Ctrl.Offset > audio_ctrl.file_len)
						return AUDIO_READ_FAIL;

					if(Snd_Lseek(G_SACM_Ctrl.Offset,SEEK_CUR) >= 0)
						audio_ctrl.file_cnt += G_SACM_Ctrl.Offset + 6;
					else
						return AUDIO_READ_FAIL;

					len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
					len += 6;
					audio_ctrl.read_secs += (len/512);
			        wi += len;
				}
			}
			else
			{	// not use jmp to play
				len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
				audio_ctrl.read_secs += (len/512);
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
            err = (INT8U) xQueueReceive(audio_fsq, &len, portMAX_DELAY);
            if((err != pdPASS) || (len < 0)) {
                return AUDIO_READ_FAIL;
            }
        #endif

			if (audio_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
				audio_ctrl.data_offset += len;
            audio_ctrl.read_secs += (len/512);
            wi += len;
            return wi;
        }
    }

    len = ri - wi;
    if (len <= 0) {
    	len += audio_ctrl.ring_size;
    }
    if(len < audio_ctrl.ring_size/2) {
    	return wi;
    }

	t = wi;
	wi += audio_ctrl.ring_size/2;
	if(wi == audio_ctrl.ring_size) {
		wi = 0;
	}
	if(wi == ri) {
		return t;
	}

	audio_fs_para.buf_addr = (INT32U)ring_buf+t;
	audio_fs_para.buf_size = audio_ctrl.ring_size/2;

	audio_fs_para.spi_para.sec_cnt = audio_ctrl.ring_size/1024;

	if(msg_id == MSG_FILESRV_MAX) {
		len = Snd_GetData(audio_fs_para.buf_addr,audio_fs_para.buf_size);
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_fsq, (void *) len);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_fsq, &len, portMAX_DELAY);
    #endif
	} else {
		msgQSend(fs_msg_q_id, msg_id, (void *)&audio_fs_para, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_URGENT);
	}
	return AUDIO_READ_PEND;

#else //audio_write_buffer
	INT16S fd;
	INT32S t;
	INT32S len;

	fd = audio_ctrl.file_handle;
	if(wi == 0 && ri == 0) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		len = read(fd,(INT32U)ring_buf, audio_ctrl.ring_size/2);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        len = fs_read(fd,(INT32U)ring_buf, audio_ctrl.ring_size/2);
    #endif
        wi += len;
        return wi;
    }

    len = ri - wi;
    if (len < 0) {
    	len += audio_ctrl.ring_size;
    }
    if(len < audio_ctrl.ring_size/2) {
    	return wi;
    }

	t = wi;
	wi += audio_ctrl.ring_size/2;
	if(wi == audio_ctrl.ring_size) {
		wi = 0;
	}
	if(wi == ri) {
		return t;
	}

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	len = read(fd,(INT32U)ring_buf+t, audio_ctrl.ring_size/2);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    len = fs_read(fd,(INT32U)ring_buf+t, audio_ctrl.ring_size/2);
#endif
	if (len<0) {
		DBG_PRINT("error read file \r\n");
	}
	wi = t + len;
	if(wi == audio_ctrl.ring_size) {
		wi = 0;
	}
	return wi;
#endif
}

INT32S audio_check_wi(INT32S wi_in, INT32U *wi_out, INT8U wait)
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
	/* AUDIO_READ_PEND */
	t = audio_ctrl.wi;

	if (wait) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
	    len = (INT32S) OSQPend(audio_fsq, 0, &err);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            err = (INT8U) xQueueReceive(audio_fsq, &len, portMAX_DELAY);
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
                err = (INT8U) xQueueReceive(audio_fsq, &len, 0);
                if(err == pdPASS) {
			*wi_out = *wi_out;
			return 	AUDIO_ERR_NONE;
                }
                #endif
	}

	audio_ctrl.reading = 0;
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	if ((err != OS_NO_ERR) || len < 0) {
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        if ((err != pdPASS) || len < 0) {
        #endif
            return AUDIO_READ_FAIL;
    }
	if (audio_context_p->source_type == AUDIO_SRC_TYPE_USER_DEFINE)
		audio_ctrl.data_offset += len;
    audio_ctrl.read_secs += (len/512);
	t += len;
	if(t == audio_ctrl.ring_size) {
		t = 0;
	}

	*wi_out = t;
	return 	AUDIO_ERR_NONE;
}
#endif

static INT32S audio_send_to_dma(void)
{
	INT32U count;

	if (stopped && (audio_q_check()==AUDIO_SEND_FAIL)) { /* queuq full */
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
        stopped = 0;
		OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_DMA_DBF_START);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        INT32U message = MSG_AUD_DMA_DBF_START;
        stopped = 0;
        xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
    #endif
    }
	else if (drv_l1_dac_dma_status_get() == 0) {
		if (!stopped) {
		#if 1 /* dac underrun, wait queue full again and start DMA*/
			stopped = 1;
			drv_l1_dac_dbf_free();
			DBG_PRINT("dac underrun !\r\n");

			audio_queue_clear();
			for (count=0;count<MAX_DAC_BUFFERS;count++) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(audio_wq, (void *) count);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                xQueueSend(audio_wq, &count, portMAX_DELAY);
            #endif
			}
		#else /* if any buffer into queue, start DMA again */
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(hAudioDacTaskQ,(void *) MSG_AUD_DMA_DBF_RESTART);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                INT32U message = MSG_AUD_DMA_DBF_RESTART;
                xQueueSend(hAudioDacTaskQ, &message, portMAX_DELAY);
            #endif
        #endif
		}
	}
	return STATUS_OK;
}

static INT32S audio_q_check(void)
{
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OS_Q      *pq;
	pq = (OS_Q *)aud_send_q->OSEventPtr;
	if (pq->OSQEntries >= 2) {//for fast 090531
		return AUDIO_SEND_FAIL;
	}
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    INT32U Number = *(INT32U *)((INT32U)aud_send_q + 4*4 + sizeof(xList) * 2);
    if(Number >= MAX_DAC_BUFFERS) {
    	return AUDIO_SEND_FAIL;
    }
#endif
	return AUDIO_SEND_OK;
}

//===============================================================================================================
//   MP3 Playback
//===============================================================================================================
#if APP_MP3_DECODE_FG_EN == 1

//#define NOT_SUPPORT_LAYER12

void Set_mp3_play_start_time(INT32U play_start_time)
{
	audio_play_start_time = play_start_time;
}

INT32S audio_mp3_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT32S  t_wi;
	INT8U   *mp3_play_seek_flag;
	INT32U  *mp3_play_seek_offset;
	INT8U   err;

    osEvent result;
	while(1)
	{

		//add seek_time
		if(G_SACM_Ctrl.Offsettype != SND_OFFSET_TYPE_NONE) {
			result = osMessageGet(Mbox_MP3_Play_Seek_Flag,200);
			mp3_play_seek_flag = (INT8U*)result.value.v;
			if ((result.status!=osEventMessage))
                continue;
			if((mp3_play_seek_flag) == (INT8U*)1) {
				result = osMessageGet(Mbox_MP3_Play_Seek_Offset,200);
				mp3_play_seek_offset = (INT32U*)result.value.v;
				if ((result.status!=osEventMessage))
                    continue;
				MP3Parser_Seek((INT32U *)&mp3_play_seek_offset,mp3_VBR_flag);

				audio_ctrl.wi = audio_ctrl.ri = 0;
				mp3_dec_set_ri((char *)audio_ctrl.work_mem, audio_ctrl.ri);
				audio_ctrl.file_cnt = lseek(audio_ctrl.file_handle,0,SEEK_CUR);
				G_SACM_Ctrl.Offsettype = SND_OFFSET_TYPE_NONE;
			}
		}

		audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

		/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			if(audio_ctrl.f_last) {
				return 0; //AUDIO_ERR_DEC_FINISH;
			} else {
				audio_ctrl.f_last = 1;
			}
		}
		in_length = audio_ctrl.ri;
		pcm_point = mp3_dec_run(work_mem, Buffer, audio_ctrl.wi);
		audio_decode_sample_number += pcm_point;

		audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
		if((audio_ctrl.ri == audio_ctrl.wi) && (audio_ctrl.ri == audio_ctrl.ring_size/2)) {
			// fixed ri for audio_write_with_file_srv
			audio_ctrl.ri = in_length + audio_ctrl.ring_size/2;
			if(audio_ctrl.ri >= audio_ctrl.ring_size) {
				audio_ctrl.ri -= audio_ctrl.ring_size;
			}

			mp3_dec_set_ri((void*)audio_ctrl.work_mem, audio_ctrl.ri);
			audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
		}

		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			//if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FAIL);
				}
			//}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
		} else {
			break;
		}
	}

	return (pcm_point * mp3_dec_get_channel((CHAR*)work_mem));
}

#define	_ADD_VBR_TYPE_

INT32S audio_mp3_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;
	INT8U *U8_ptr;
	INT8U  char_temp0,char_temp1,char_temp2,char_temp3;
	INT32S i;
	INT32U temp;

#ifdef	_ADD_VBR_TYPE_
    INT32U VBR_offset=0;
    INT32U j;
    INT32U VBR_start[]={36, 21, 13};
    INT32U VBRI_start[]={36, 38};
#endif

	audio_ctrl.file_cnt = 0;
	audio_ctrl.f_last = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;
	//fg_error_cnt = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	// mp3 parser
	MP3Parser_Init(audio_ctrl.file_handle);

	// mp3 init
	ret = mp3_dec_init((char*)audio_ctrl.work_mem, (unsigned char*)audio_ctrl.ring_buf, (char*)(audio_ctrl.work_mem + MP3_DEC_MEMORY_SIZE));
	mp3_dec_set_bs_buf_size((void*)audio_ctrl.work_mem, audio_ctrl.ring_size);

	audio_ctrl.wi = 0;
	audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	U8_ptr = (INT8U*)audio_ctrl.ring_buf;
	char_temp0 = *U8_ptr++;
	char_temp1 = *U8_ptr++;
	char_temp2 = *U8_ptr++;
	if((char_temp0 == 'I') && (char_temp1 == 'D') && (char_temp2 == '3'))//ID3 header
	{
		U8_ptr += 3;
		char_temp3 = *U8_ptr++;
		char_temp2 = *U8_ptr++;
		char_temp1 = *U8_ptr++;
		char_temp0 = *U8_ptr++;
		mp3_ID3V2_length = 10 + ((char_temp0 & 0x7f)|((char_temp1 & 0x7f)<<7)|((char_temp2 & 0x7f)<<14)|((char_temp3 & 0x7f)<<21));
		switch(audio_context_p->source_type)
		{
			case AUDIO_SRC_TYPE_FS:
				audio_ctrl.file_cnt = mp3_ID3V2_length;
				ret = (INT32S)lseek(audio_ctrl.file_handle, audio_ctrl.file_cnt, SEEK_SET);
				audio_ctrl.wi = 0;
				audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_DEC_FAIL;
				}
				break;

			case AUDIO_SRC_TYPE_SPI:
				break;

			case AUDIO_SRC_TYPE_USER_DEFINE:
				audio_ctrl.file_cnt = mp3_ID3V2_length;
				audio_ctrl.data_offset = audio_ctrl.file_cnt;
				audio_ctrl.wi = 0;
				audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_DEC_FAIL;
				}
				break;

			case AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE:	// added by Bruce, 2010/01/22
				audio_ctrl.file_cnt = mp3_ID3V2_length;
				ret = (INT32S)lseek(audio_ctrl.file_handle, audio_ctrl.file_cnt+audio_ctrl.data_offset, SEEK_SET);
				audio_ctrl.wi = 0;
				audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
					return AUDIO_ERR_DEC_FAIL;
				}
				break;

			default:
				break;
		}
	}
	else
	{
		mp3_ID3V2_length = 0;
	}

	U8_ptr = (INT8U*)audio_ctrl.ring_buf;

#ifdef	_ADD_VBR_TYPE_
		mp3_VBR_flag = 0;//CBR
		for(j=0; j<3; j++)
		{
			VBR_offset = VBR_start[j];
			if( (*(U8_ptr+VBR_offset)=='X') && (*(U8_ptr+VBR_offset+1)=='i') && (*(U8_ptr+VBR_offset+2)=='n') && (*(U8_ptr+VBR_offset+3)=='g') )//Xing flag
			{
				mp3_VBR_flag = 1;//Xing
				break;
			}
		}
		if(mp3_VBR_flag == 0)  //20130717
		{
			for(j=0; j < 2; j++)
			{
				VBR_offset = VBRI_start[j];
				if( (*(U8_ptr+VBR_offset)=='V') && (*(U8_ptr+VBR_offset+1)=='B') && (*(U8_ptr+VBR_offset+2)=='R') && (*(U8_ptr+VBR_offset+3)=='I') )//VBRI flag
				{
					mp3_VBR_flag = 2;//VBRI
					break;
				}
			}
		}
    	if(mp3_VBR_flag == 1)//Xing
    	{
    		if((*(U8_ptr + VBR_offset + 7)) | 0x01)//total frame number exist
    		{
    			mp3_total_frame = (INT32U)(*(U8_ptr+VBR_offset+8)<<24) | (*(U8_ptr+VBR_offset+9)<<16) | (*(U8_ptr+VBR_offset+10)<<8) | (*(U8_ptr+VBR_offset+11));
    			if(mp3_total_frame == 0)//090819
    			{
    				mp3_VBR_flag = 0;
    			}
    		}
    		else
    		{
    			mp3_VBR_flag = 0;
    		}
    	}
    	else if(mp3_VBR_flag == 2)//VBRI
    	{
			mp3_total_frame = (INT32U)(*(U8_ptr+VBR_offset+14)<<24) | (*(U8_ptr+VBR_offset+15)<<16) | (*(U8_ptr+VBR_offset+16)<<8) | (*(U8_ptr+VBR_offset+17));
			if(mp3_total_frame == 0)//090819
			{
				mp3_VBR_flag = 0;
			}
    	}
#else

	if( (*(U8_ptr+36)=='X') && (*(U8_ptr+37)=='i') && (*(U8_ptr+38)=='n') && (*(U8_ptr+39)=='g') ) {
		mp3_VBR_flag = 1;//VBR
	} else {
		mp3_VBR_flag = 0;//CBR
	}

	if(mp3_VBR_flag) { //VBR
		if((*(U8_ptr+43)) | 0x01) { //total frame number exist
			mp3_total_frame = (INT32U)(*(U8_ptr+44)<<24) | (*(U8_ptr+45)<<16) | (*(U8_ptr+46)<<8) | (*(U8_ptr+47));
			if(mp3_total_frame == 0) {
				mp3_VBR_flag = 0;
			}
		} else {
			mp3_VBR_flag = 0;
		}
	}
#endif
	count = 400;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = mp3_dec_parsing((void*)audio_ctrl.work_mem , audio_ctrl.wi);
#ifdef NOT_SUPPORT_LAYER12
        {
            int layer;

            layer = (INT8U)mp3_dec_get_layer((void*)audio_ctrl.work_mem);
            if(layer != 3){
                DBG_PRINT("Not Layer 3 file \r\n",layer);
                return AUDIO_ERR_DEC_FAIL;
            }
        }
#endif
		audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
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
				audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
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

	in_length = mp3_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);

	audio_samplerate = in_length;
	audio_bitrate = mp3_dec_get_bitrate((CHAR*)audio_ctrl.work_mem);
	if((audio_bitrate == 0) || (audio_samplerate == 0)) {
		return AUDIO_ERR_DEC_FAIL;
	}

	if(mp3_VBR_flag == 0) { //CBR
        INT64U file_len = audio_ctrl.file_len;
		audio_total_time = ((file_len - mp3_ID3V2_length)*(8*10))/audio_bitrate;		// modified by Bruce, 2010/01/08
	} else { //VBR
		for(i=0;i<2;i++)
		{
			audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
			t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

			/* check reading data */
			if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE){
				return AUDIO_ERR_DEC_FAIL;
			}

			if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
				if(audio_ctrl.f_last) {
					return 0; //AUDIO_ERR_DEC_FINISH;
				} else {
					audio_ctrl.f_last = 1;
				}
			}

			in_length = audio_ctrl.ri;
			temp = mp3_dec_run((CHAR*)audio_ctrl.work_mem, pcm_out[0], audio_ctrl.wi);

			audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
			audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
			if(in_length > audio_ctrl.ri) {
				audio_ctrl.file_cnt += audio_ctrl.ring_size;
			}
		}

		audio_total_time = (temp * mp3_total_frame * 10 / audio_samplerate);	// modified by Bruce, 2010/01/08

	}

	if(audio_play_start_time >= audio_total_time/10) {
		audio_play_start_time = audio_total_time/10 -1;
	}

	if(audio_play_start_time < 0) {
		audio_play_start_time = 0;
	}

	if(audio_play_start_time == 0) {
		audio_ctrl.file_cnt = mp3_ID3V2_length;
	} else {
		audio_ctrl.file_cnt = mp3_ID3V2_length + (INT32U)((INT64U)(audio_ctrl.file_len - mp3_ID3V2_length)*audio_play_start_time *10 /audio_total_time); // modified by Bruce, 2010/01/08
	}

	audio_decode_sample_number = audio_samplerate * audio_play_start_time;
	switch(audio_context_p->source_type)//100528
	{
		case AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE:// modified by Bruce, 2010/01/22
			ret = (INT32S)lseek(audio_ctrl.file_handle, audio_ctrl.file_cnt+audio_ctrl.data_offset, SEEK_SET);
			break;

		case AUDIO_SRC_TYPE_FS:
			ret = (INT32S)lseek(audio_ctrl.file_handle, audio_ctrl.file_cnt, SEEK_SET);
			break;

		case AUDIO_SRC_TYPE_USER_DEFINE:
			audio_ctrl.data_offset = audio_ctrl.file_cnt;
			break;

		default:
			break;
	}

	audio_ctrl.wi = 0;
	audio_ctrl.ri = mp3_dec_get_ri((void*)audio_ctrl.work_mem);
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	in_length = audio_samplerate;
	ret = mp3_dec_get_channel((CHAR*)audio_ctrl.work_mem);

	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_mp3_get_output;

#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ret == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ret, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ret = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ret == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ret, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ret = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
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
#if APP_UP_SAMPLE_EN
	count = 44100/in_length;
	if(count > 4)	count = 4;
	if(hUpSample && (count >= 2))
	{
		if(ret == 1)
			UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 1);
		else
			UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ret = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif //APP_UP_SAMPLE_EN

	channel = ret;
	g_audio_sample_rate = in_length;
#if (MCU_VERSION != GPM41XXA) && (_DRV_L1_HDMI == 1)
	if(g_audio_main_channel == 3)
	{
        switch(in_length)
        {
            case 44100:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_44K);
            break;
            case 48000:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_48K);
            break;
            default:break;
        }
        drvl1_hdmi_audio_ctrl(1);
	}
	else
#endif
	{
        drv_l1_dac_sample_rate_set(in_length);
        drv_l1_dac_hw_up_sample_set(in_length);
    }
	if(ret == 1) {
		drv_l1_dac_mono_set();
	} else {
		drv_l1_dac_stereo_set();
	}

	sample_rate = mp3_dec_get_samplerate((void*)audio_ctrl.work_mem);
	channel = mp3_dec_get_channel((void*)audio_ctrl.work_mem);
	DBG_PRINT("version: %s\r\n",mp3_dec_get_version());
	DBG_PRINT("bps: %d\r\n",mp3_dec_get_bitrate((void*)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",mp3_dec_get_channel((void*)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n",mp3_dec_get_samplerate((void*)audio_ctrl.work_mem));
	DBG_PRINT("block size: %d\r\n",mp3_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
   #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_mp3_process(void)
{
	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], MP3_DEC_FRAMESIZE);	//pcm_point*2
	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();
	return 0;
}
#endif // #if APP_MP3_DECODE_FG_EN == 1

//===============================================================================================================
//   WMA Playback
//===============================================================================================================
#if APP_WMA_DECODE_FG_EN == 1
INT32S audio_wma_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S	pcm_point;
	INT32U  in_length;
	INT8U   err;
	INT32S  t_wi;
	INT32S  offset;
	INT32S  len;
	INT8U   *wma_play_seek_flag;
	INT32U  *wma_play_seek_offset;
	INT32U  seek_offset;

	while(1)
	{
		audio_ctrl.ri = wma_dec_get_ri((char *)audio_ctrl.work_mem);

		offset = wma_dec_get_offset((char *)audio_ctrl.work_mem);

		if(offset > 0) {
			len = audio_ctrl.wi - audio_ctrl.ri;
			if(len < 0) {
				len += audio_ctrl.ring_size;
			}

			audio_ctrl.file_cnt += offset;
			if(len > offset) {
				audio_ctrl.ri += offset;
				if(audio_ctrl.ri >= audio_ctrl.ring_size) {
					audio_ctrl.ri -= audio_ctrl.ring_size;
				}

				wma_dec_set_ri((char *)audio_ctrl.work_mem, audio_ctrl.ri);
			} else {
				offset -= len;
				if(lseek(audio_ctrl.file_handle, offset, SEEK_CUR) < 0) {
					return (0-AUDIO_ERR_DEC_FAIL);
				}

				audio_ctrl.wi = audio_ctrl.ri = 0;
				wma_dec_set_ri((char *)audio_ctrl.work_mem, audio_ctrl.ri);
			}

			wma_dec_reset_offset((char *)audio_ctrl.work_mem);
		}
#if 0//TBD: fix seek mbox on FreeRTOS
		//add seek_time
		if(G_SACM_Ctrl.Offsettype != SND_OFFSET_TYPE_NONE) {
			wma_play_seek_flag = OSMboxAccept(Mbox_WMA_Play_Seek_Flag);
			if((*wma_play_seek_flag) == 1) {
				wma_play_seek_offset = OSMboxPend(Mbox_WMA_Play_Seek_Offset , 0 , &err);
				seek_offset = wma_dec_seek((char *)audio_ctrl.work_mem, *wma_play_seek_offset);
				if(lseek(audio_ctrl.file_handle, seek_offset, SEEK_SET)) {
			  		audio_ctrl.file_cnt = seek_offset;
			 	}

				audio_ctrl.wi = audio_ctrl.ri = 0;
				wma_dec_set_ri((char *)audio_ctrl.work_mem, audio_ctrl.ri);
			}
		}
#endif
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

		/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
			return (0-AUDIO_ERR_READ_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			if(audio_ctrl.f_last){
				return 0; //AUDIO_ERR_DEC_FINISH;
			} else {
				audio_ctrl.f_last = 1;
			}
		}

		in_length = audio_ctrl.ri;
		pcm_point = wma_dec_run((char *)audio_ctrl.work_mem,Buffer,audio_ctrl.wi);

		audio_ctrl.ri = wma_dec_get_ri((char *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FAIL);
				}
			}
		} else {
			break;
		}
	}

	return (pcm_point * wma_dec_get_channel((char *)audio_ctrl.work_mem));
}

int audio_wma_play_init()
{
	INT32S count;
	INT32U in_length;
	INT32S ret;
	INT8U  ch;
	INT32S  t_wi;

	audio_ctrl.file_cnt = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;
    fg_error_cnt = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf,0,audio_ctrl.ring_size);

	//wma init
	ret = wma_dec_init((char *)audio_ctrl.work_mem, audio_ctrl.ring_buf,(char *)audio_ctrl.work_mem + 8192,audio_ctrl.ring_size);
	audio_ctrl.wi = wma_dec_get_ri((char *)audio_ctrl.work_mem);
	audio_ctrl.ri = wma_dec_get_ri((char *)audio_ctrl.work_mem);

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 50;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = wma_dec_parsing((char *)audio_ctrl.work_mem , audio_ctrl.wi);
		audio_ctrl.ri = wma_dec_get_ri((char *)audio_ctrl.work_mem);

		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
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
				audio_ctrl.ri = wma_dec_get_ri((char *)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
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

	in_length =wma_dec_get_samplerate((char *)audio_ctrl.work_mem);
	ch = wma_dec_get_channel((char *)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	pfnGetOutput=audio_wma_get_output;

#if APP_VOICE_CHANGER_EN
	if(hVC && G_snd_info.VoiceChangerEn)
	{
		VoiceChanger_Link(hVC, hSrc, pfnGetOutput, in_length, ch, 0);
		VoiceChanger_SetParam(hVC, G_snd_info.Speed, G_snd_info.Pitch);
		hSrc = hVC;
		in_length = VoiceChanger_GetSampleRate(hVC);
		ret = VoiceChanger_GetChannel(hVC);
		pfnGetOutput = &VoiceChanger_GetOutput;
	}
#endif

	channel = ch;
	g_audio_sample_rate = in_length;
	audio_total_time = wma_dec_get_playtime((char*)audio_ctrl.work_mem);
	dac_sample_rate_set(in_length);
	if (ch == 1) {
		dac_mono_set();
	} else {
		dac_stereo_set();
	}

	DBG_PRINT("bps: %d\r\n",wma_dec_get_bitrate((char*)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",ch);
	DBG_PRINT("sample rate: %d\r\n",wma_dec_get_samplerate((char*)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
   #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_wma_process()
{
	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], WMA_DEC_FRAMESIZE);
	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();

	return 0;
}
#endif // #if APP_WMA_DECODE_FG_EN == 1

//===============================================================================================================
//   WAVE Playback
//===============================================================================================================
#if APP_WAV_CODEC_FG_EN == 1
#if 1
INT32S audio_wav_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S	pcm_point;
	INT32U  in_length;
	INT32S  tmp_len;
	INT32S  t_wi;

	while(1)
	{
		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return 0; //AUDIO_ERR_DEC_FINISH;
		}
		audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

		/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return 0; //AUDIO_ERR_DEC_FINISH;
		}

		in_length = audio_ctrl.wi - audio_ctrl.ri;
		if(audio_ctrl.wi < audio_ctrl.ri) {
			in_length += audio_ctrl.ring_size;
		}

		if(in_length > (audio_ctrl.file_len-audio_ctrl.file_cnt)) {
			tmp_len = audio_ctrl.wi;
			tmp_len -= (in_length - audio_ctrl.file_len + audio_ctrl.file_cnt);
			if(tmp_len < 0) {
				tmp_len += audio_ctrl.ring_size;
			}
			audio_ctrl.wi = tmp_len;
		}

		in_length = audio_ctrl.ri;
		pcm_point = wav_dec_run((INT8U *)audio_ctrl.work_mem, Buffer, audio_ctrl.wi);
		audio_decode_sample_number += pcm_point;
		if (pcm_point == -1) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (--audio_ctrl.try_cnt == 0) {
				return (0 - AUDIO_ERR_DEC_FAIL);
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
		} else {
			break;
		}
	}

	return (pcm_point * wav_dec_get_nChannels((INT8U *)audio_ctrl.work_mem));
}

INT32S  audio_wav_dec_process(void)
{
	INT32S pcm_point;
	INT8U   err;
	INT32U  wb_idx;

//	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], WAV_DEC_FRAMESIZE);
	if (pcm_point <= 0) {
        if(0 <= gEncode_Filehandle)
		{
			Save_Last_PCM_File();
		}
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;

	if(0 <= gEncode_Filehandle)
	{
		if((Write_Index + (pcm_len[wb_idx] << 1))>Save_Voice_Buffer_Size)
		{
            INT32S ret;
			ret = fs_write(gEncode_Filehandle , (INT32U)Save_Voice_Buffer , Write_Index);
			if(0 > ret)
			{
                return (0 - AUDIO_ERR_WRITE_FAIL);
			}
			Write_Index = 0;
		}

		gp_memcpy((INT8S*)(Save_Voice_Buffer +Write_Index),(INT8S*)pcm_out[wb_idx], (pcm_len[wb_idx] << 1));//20110517
		Write_Index += pcm_len[wb_idx] << 1;

		PCM_Header.data_len += pcm_len[wb_idx]<<1;
	}

    if(0 <= gEncode_Filehandle)
	{
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPost(audio_wq, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &wb_idx, portMAX_DELAY);
    #endif
	}
	else
	{
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPost(aud_send_q, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
    #endif
		audio_send_to_dma();
	}
	audio_send_next_frame_q();
	return 0;
}

INT32S audio_wav_dec_play_init()
{
	INT32U in_length;
	INT8U  ch;
	INT32S count;
	INT32S t_wi;

	audio_decode_sample_number = 0;
	audio_ctrl.file_cnt = 0;
	audio_ctrl.try_cnt = 10;
	audio_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	//wave init
	wav_dec_init((INT8U *)audio_ctrl.work_mem, audio_ctrl.ring_buf);
	wav_dec_set_ring_buf_size((INT8U *)audio_ctrl.work_mem, audio_ctrl.ring_size);
	audio_ctrl.wi = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
	audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);

	in_length = audio_ctrl.ri;
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	audio_ctrl.file_len = wav_dec_parsing((INT8U *)audio_ctrl.work_mem , audio_ctrl.wi);
	if((int)(audio_ctrl.file_len) <= 0) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
	audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
	if(in_length > audio_ctrl.ri) {
		audio_ctrl.file_cnt += audio_ctrl.ring_size;
	}

	in_length = wav_dec_get_SampleRate((INT8U *)audio_ctrl.work_mem);
	ch = wav_dec_get_nChannels((INT8U *)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_wav_get_output;

    if(0 <= gEncode_Filehandle)
	{
        INT32S ret;
		PCM_Header.RIFF_ID[0] ='R';
		PCM_Header.RIFF_ID[1] ='I';
		PCM_Header.RIFF_ID[2] ='F';
		PCM_Header.RIFF_ID[3] ='F';
		PCM_Header.RIFF_len = 0;

		PCM_Header.type_ID[0] = 'W';
		PCM_Header.type_ID[1] = 'A';
		PCM_Header.type_ID[2] = 'V';
		PCM_Header.type_ID[3] = 'E';

		PCM_Header.fmt_ID[0] = 'f';
		PCM_Header.fmt_ID[1] = 'm';
		PCM_Header.fmt_ID[2] = 't';
		PCM_Header.fmt_ID[3] = ' ';

		PCM_Header.fmt_len = 16;
		PCM_Header.format = 1;
		PCM_Header.channel = ch;
		PCM_Header.sample_rate = in_length;

		//8, 16, 24 or 32
		PCM_Header.Sign_bit_per_sample = 16;
		//BlockAlign = SignificantBitsPerSample / 8 * NumChannels
		PCM_Header.Block_align = 16/8*ch;
		//AvgBytesPerSec = SampleRate * BlockAlign
		PCM_Header.avg_byte_per_sec = in_length*2*ch;

		PCM_Header.data_ID[0] = 'd';
		PCM_Header.data_ID[1] = 'a';
		PCM_Header.data_ID[2] = 't';
		PCM_Header.data_ID[3] = 'a';

		PCM_Header.data_len = 0;
        ret = fs_write(gEncode_Filehandle,0,44);
        if(0 > ret)
        {
            return AUDIO_ERR_DEC_FAIL;
        }
		Save_Voice_Buffer = (INT8U *)gp_malloc_align(Save_Voice_Buffer_Size, 4);//20110517
		if(0 == Save_Voice_Buffer)
		{
            return AUDIO_ERR_MEM_ALLOC_FAIL;
		}
		Write_Index = 0;
	}


#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ch == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ch, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ch = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ch == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ch, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ch = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
#if APP_VOICE_CHANGER_EN
	if(hVC && G_snd_info.VoiceChangerEn)
	{
		VoiceChanger_Link(hVC, hSrc, pfnGetOutput, in_length, ch, 0);
		VoiceChanger_SetParam(hVC, G_snd_info.Speed, G_snd_info.Pitch);
		hSrc = hVC;
		in_length = VoiceChanger_GetSampleRate(hVC);
		ch = VoiceChanger_GetChannel(hVC);
		pfnGetOutput = &VoiceChanger_GetOutput;
	}
#endif
#if APP_UP_SAMPLE_EN
    if(0 > gEncode_Filehandle)
	{
	count = 44100 / in_length;
	if(count > 4)	count = 4;
	if(hUpSample && count>=2)
	{
		UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ch, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ch = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
        }
	}
#endif

	channel = ch;
	g_audio_sample_rate = in_length;
#if (MCU_VERSION != GPM41XXA) && (_DRV_L1_HDMI == 1)
	if(g_audio_main_channel == 3)
	{
        switch(in_length)
        {
            case 44100:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_44K);
            break;
            case 48000:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_48K);
            break;
            default:break;
        }
        drvl1_hdmi_audio_ctrl(1);
	}
	else
#endif
	{
        drv_l1_dac_sample_rate_set(in_length);
        drv_l1_dac_hw_up_sample_set(in_length);
    }
    if (ch == 1) {
        drv_l1_dac_mono_set();
    } else {
        drv_l1_dac_stereo_set();
    }
	sample_rate = wav_dec_get_SampleRate((INT8U*)audio_ctrl.work_mem);
	channel = wav_dec_get_nChannels((INT8U *)audio_ctrl.work_mem);
	DBG_PRINT("channel: %d\r\n", wav_dec_get_nChannels((INT8U *)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", wav_dec_get_SampleRate((INT8U*)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
    }

	return AUDIO_ERR_NONE;
}

#else

INT32S audio_wav_dec_play_init()
{
	INT32U in_length;
	INT8U  ch;
	INT32S count;
	INT32S t_wi;

	audio_decode_sample_number = 0;
	audio_ctrl.file_cnt = 0;
	audio_ctrl.try_cnt = 10;
	audio_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	//wave init
	wav_dec_init((INT8U *)audio_ctrl.work_mem, audio_ctrl.ring_buf);
	wav_dec_set_ring_buf_size((INT8U *)audio_ctrl.work_mem, audio_ctrl.ring_size);
	audio_ctrl.wi = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
	audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);

	in_length = audio_ctrl.ri;
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	audio_ctrl.file_len = wav_dec_parsing((INT8U *)audio_ctrl.work_mem , audio_ctrl.wi);
	if((int)(audio_ctrl.file_len) <= 0) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
	audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
	if(in_length > audio_ctrl.ri) {
		audio_ctrl.file_cnt += audio_ctrl.ring_size;
	}

	in_length = wav_dec_get_SampleRate((INT8U *)audio_ctrl.work_mem);
	ch = wav_dec_get_nChannels((INT8U *)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	//pfnGetOutput = audio_wav_get_output;

#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ch == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ch, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ch = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ch == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ch, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ch = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
#if APP_VOICE_CHANGER_EN
	if(hVC && G_snd_info.VoiceChangerEn)
	{
		VoiceChanger_Link(hVC, hSrc, pfnGetOutput, in_length, ch, 0);
		VoiceChanger_SetParam(hVC, G_snd_info.Speed, G_snd_info.Pitch);
		hSrc = hVC;
		in_length = VoiceChanger_GetSampleRate(hVC);
		ch = VoiceChanger_GetChannel(hVC);
		pfnGetOutput = &VoiceChanger_GetOutput;
	}
#endif
#if APP_UP_SAMPLE_EN
	count = 44100 / in_length;
	if(count > 4)	count = 4;
	if(hUpSample && count>=2)
	{
		UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ch, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ch = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif

	channel = ch;
	g_audio_sample_rate = in_length;
	if(g_audio_main_channel == 3)
	{
        switch(in_length)
        {
            case 44100:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_44K);
            break;
            case 48000:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_48K);
            break;
            default:break;
        }
        drvl1_hdmi_audio_ctrl(1);
	}
	else
	{
        drv_l1_dac_sample_rate_set(in_length);
    }
    if (ch == 1) {
        drv_l1_dac_mono_set();
    } else {
        drv_l1_dac_stereo_set();
    }
	DBG_PRINT("channel: %d\r\n", wav_dec_get_nChannels((INT8U *)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", wav_dec_get_SampleRate((INT8U*)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
    }

	return AUDIO_ERR_NONE;
}

INT32S  audio_wav_dec_process()
{
	INT32S pcm_point;
	INT32U  in_length;
	INT32S  tmp_len;
	INT32U  wb_idx;
	INT8U   err;
	INT8U   wait;
	INT32S  t_wi;

	audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif

	wait = 1; /* pend until read finish */

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
		return AUDIO_ERR_DEC_FINISH;
	}

	in_length = audio_ctrl.wi - audio_ctrl.ri;
	if(audio_ctrl.wi < audio_ctrl.ri) {
		in_length += audio_ctrl.ring_size;
	}

	if(in_length > (audio_ctrl.file_len-audio_ctrl.file_cnt)) {
		tmp_len = audio_ctrl.wi;
		tmp_len -= (in_length - audio_ctrl.file_len + audio_ctrl.file_cnt);
		if(tmp_len < 0) {
			tmp_len += audio_ctrl.ring_size;
		}
		audio_ctrl.wi = tmp_len;
	}

	in_length = audio_ctrl.ri;
	pcm_point = wav_dec_run((INT8U *)audio_ctrl.work_mem,pcm_out[wb_idx],audio_ctrl.wi);
	if (pcm_point == -1) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_ctrl.ri = wav_dec_get_ri((INT8U *)audio_ctrl.work_mem);
	audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
	if(in_length > audio_ctrl.ri) {
		audio_ctrl.file_cnt += WAV_DEC_BITSTREAM_BUFFER_SIZE;
	}

	if (pcm_point <= 0) {
		if (--audio_ctrl.try_cnt == 0) {
			return AUDIO_ERR_DEC_FAIL;
		}
#if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPostFront(audio_wq, (void *)wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		xQueueSendToFront(audio_wq, &wb_idx, portMAX_DELAY);
#endif
		audio_send_next_frame_q();
		return 0;
	}

	pcm_len[wb_idx] = pcm_point * wav_dec_get_nChannels((INT8U *)audio_ctrl.work_mem);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
    audio_send_to_dma();
	audio_send_next_frame_q();
	return 0;
}
#endif
#endif // #if APP_WAV_CODEC_FG_EN == 1

//===============================================================================================================
//   AVI WAVE Playback
//===============================================================================================================
#if APP_WAV_CODEC_FG_EN == 1
#endif

//===============================================================================================================
//   AVI MP3 Playback
//===============================================================================================================
#if APP_MP3_DECODE_FG_EN == 1
#endif

//===============================================================================================================
//   A1800 Playback
//===============================================================================================================
#if 1
#if APP_A1800_DECODE_FG_EN == 1
INT32S audio_a1800_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT32S  t_wi;

	while(1)
	{
		audio_ctrl.ri = a1800dec_read_index((INT8U *)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

		/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi,1) != AUDIO_ERR_NONE) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len){
			return 0; //AUDIO_ERR_DEC_FINISH;
		}

		in_length = audio_ctrl.ri;
		pcm_point = a1800dec_run((CHAR*)audio_ctrl.work_mem, audio_ctrl.wi, Buffer);
		audio_decode_sample_number += pcm_point;
		audio_ctrl.ri = a1800dec_read_index((CHAR*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);

		if(in_length >= audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FAIL);
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
                    OSQPostFront(audio_wq, (void *)wb_idx);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                    xQueueSendToFront(audio_wq, &audio_ctrl.wi, portMAX_DELAY);
            #endif
                    audio_send_next_frame_q();
                    return 0;
		} else {
			break;
		}
	}

	return pcm_point;
}

INT32S audio_a1800_play_init(void)
{
	INT32U  count;
	INT32S  ret;
	INT32U  in_length;
	INT32S  t_wi;
    INT8U   wait;

    wait = 1;
	audio_decode_sample_number = 0;
	audio_ctrl.file_cnt = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;//080724
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	//a1800 init
	ret = a1800dec_GetMemoryBlockSize();
	if(ret != A1800DEC_MEMORY_BLOCK_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = a1800dec_init((void *)audio_ctrl.work_mem, (const unsigned char *)audio_ctrl.ring_buf);//080723
	A18_dec_SetRingBufferSize((void *)audio_ctrl.work_mem, audio_ctrl.ring_size);
	audio_ctrl.wi = a1800dec_read_index((void *)audio_ctrl.work_mem);//after initial the value is 0
	audio_ctrl.ri = a1800dec_read_index((void *)audio_ctrl.work_mem);
	in_length = (audio_ctrl.ri);

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, wait) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	ret = a1800dec_parsing((void *)audio_ctrl.work_mem, audio_ctrl.wi);
	if(ret != 1) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_ctrl.ri = a1800dec_read_index((INT8U *)audio_ctrl.work_mem);
	audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
	if(in_length > audio_ctrl.ri) {
		audio_ctrl.file_cnt += audio_ctrl.ring_size;
	}

	in_length = A18_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);
	ret = A18_dec_get_channel((CHAR*)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_a1800_get_output;

#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ret == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ret, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ret = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ret == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ret, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ret = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
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
#if APP_UP_SAMPLE_EN
	count = 32000 / in_length;
	if(count > 4)	count = 4;
	if(hUpSample && count>=2)
	{
		UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ret = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif

	channel = ret;
	g_audio_sample_rate = in_length;
	drv_l1_dac_sample_rate_set(in_length);	//a1800 is 16khz sample rate
	drv_l1_dac_hw_up_sample_set(in_length);
	drv_l1_dac_mono_set();					//a1800 is mono
	sample_rate = A18_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);
	channel = A18_dec_get_channel((CHAR*)audio_ctrl.work_mem);
	DBG_PRINT("bps: %d\r\n", A18_dec_get_bitrate((void *)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", A18_dec_get_channel((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", A18_dec_get_samplerate((CHAR*)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
//		OSQPost(audio_wq, (void *) count);
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}


INT32S  audio_a1800_process(void)
{
	INT32S pcm_point;
	INT8U   err;
	INT32U  wb_idx;

//	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], A18_DEC_FRAMESIZE);
	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;
//	OSQPost(aud_send_q, (void *) wb_idx);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();
	return 0;
}
#endif // #if APP_A1800_DECODE_FG_EN == 1

#else
INT32S audio_a1800_play_init(void)
{
	INT32U  in_length;
	INT32S  ret;
	INT32U  count;
	INT32S  t_wi;

	audio_decode_sample_number = 0;
	audio_ctrl.file_cnt = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;//080724
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	//a1800 init
	ret = a1800dec_GetMemoryBlockSize();
	if(ret != A1800DEC_MEMORY_BLOCK_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = a1800dec_init((void *)audio_ctrl.work_mem, (const unsigned char *)audio_ctrl.ring_buf);//080723
	A18_dec_SetRingBufferSize((void *)audio_ctrl.work_mem, audio_ctrl.ring_size);
	audio_ctrl.wi = a1800dec_read_index((void *)audio_ctrl.work_mem);//after initial the value is 0
	audio_ctrl.ri = a1800dec_read_index((void *)audio_ctrl.work_mem);
	in_length = (audio_ctrl.ri);

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	ret = a1800dec_parsing((void *)audio_ctrl.work_mem, audio_ctrl.wi);
	if(ret != 1) {
		return AUDIO_ERR_DEC_FAIL;
	}

	audio_ctrl.ri = a1800dec_read_index((INT8U *)audio_ctrl.work_mem);
	audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
	if(in_length > audio_ctrl.ri) {
		audio_ctrl.file_cnt += audio_ctrl.ring_size;
	}

	in_length = A18_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);
	ret = A18_dec_get_channel((CHAR*)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	//pfnGetOutput = audio_a1800_get_output;

#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ret == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ret, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ret = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ret == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ret, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ret = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
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
#if APP_UP_SAMPLE_EN
	count = 48000 / in_length;
	DBG_PRINT("count = %d \r\n",count);
	if(count > 4)	count = 4;
	if(hUpSample && count>=2)
	{
		UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ret = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif

	channel = ret;
	g_audio_sample_rate = in_length;

	if(g_audio_main_channel == 3)
	{
        switch(in_length)
        {
            case 44100:
            drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_44K);
            break;
            case 48000:
            drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_48K);
            break;
            default:break;
        }
        drvl1_hdmi_audio_ctrl(1);
	}
	else
	{
        drv_l1_dac_sample_rate_set(in_length);	//a1800 is 16khz sample rate
    }
    drv_l1_dac_mono_set();					//a1800 is mono

	DBG_PRINT("bps: %d\r\n", A18_dec_get_bitrate((void *)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", A18_dec_get_channel((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", A18_dec_get_samplerate((CHAR*)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

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

	audio_ctrl.ri = a1800dec_read_index((INT8U *)audio_ctrl.work_mem);
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif

	wait = 1; /* pend until read finish */
	/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi,wait) != AUDIO_ERR_NONE) {
			return AUDIO_ERR_DEC_FAIL;
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len){
			return AUDIO_ERR_DEC_FINISH;
		}

		in_length = audio_ctrl.ri;
		pcm_point = a1800dec_run((CHAR*)audio_ctrl.work_mem, audio_ctrl.wi, pcm_out[wb_idx]);
		audio_decode_sample_number += pcm_point;
		audio_ctrl.ri = a1800dec_read_index((CHAR*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);

		if(in_length >= audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
                    OSQPostFront(audio_wq, (void *)wb_idx);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		xQueueSendToFront(audio_wq, &wb_idx, portMAX_DELAY);
            #endif
                    audio_send_next_frame_q();
                    return 0;
	}

	pcm_len[wb_idx] = pcm_point;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();
	return 0;
}
#endif



//===============================================================================================================
//   A1600 Playback
//===============================================================================================================
#if APP_A1600_DECODE_FG_EN == 1
INT32S audio_a16_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT32S  t_wi;

	while(1)
	{
		audio_ctrl.ri = A16_dec_get_ri((void *)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

		/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return 0;	//AUDIO_ERR_DEC_FINISH;
		}

		in_length = audio_ctrl.ri;
		pcm_point = A16_dec_run((CHAR*)audio_ctrl.work_mem, Buffer, audio_ctrl.wi);

		audio_ctrl.ri = A16_dec_get_ri((CHAR*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FAIL);
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
		} else {
			break;
		}
	}

	return pcm_point;
}

INT32S audio_a16_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_ctrl.file_cnt = 0;
	audio_ctrl.f_last = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	//a1600 init
	ret = A16_dec_get_mem_block_size();
	if(ret != A16_DEC_MEMORY_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = A16_dec_init((char *)audio_ctrl.work_mem, (unsigned char *)audio_ctrl.ring_buf);//080723
	audio_ctrl.wi = A16_dec_get_ri((void *)audio_ctrl.work_mem);//after initial the value is 0
	audio_ctrl.ri = A16_dec_get_ri((void *)audio_ctrl.work_mem);
	in_length = audio_ctrl.ri;
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = A16_dec_parsing((CHAR*)audio_ctrl.work_mem , audio_ctrl.wi);

		audio_ctrl.ri = A16_dec_get_ri((CHAR*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		switch(ret)
		{
			case A16_OK:
				break;

			case A16_E_NO_MORE_SRCDATA:		//not found sync word
			case A16_E_READ_IN_BUFFER:		//reserved sample frequency value
			case A16_CODE_FILE_FORMAT_ERR:	//forbidden bitrate value
			case A16_E_FILE_END:
			case A16_E_MODE_ERR:
				audio_ctrl.ri = A16_dec_get_ri((CHAR*)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
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

	in_length = A16_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);
	ret = A16_dec_get_channel((CHAR*)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_a16_get_output;

#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ret == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ret, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ret = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ret == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ret, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ret = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
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
#if APP_UP_SAMPLE_EN
	count = 48000 / in_length;
	if(count > 4)	count = 4;
	if(hUpSample && count >= 2)
	{
		UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ret = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif

	channel = ret;
	g_audio_sample_rate = in_length;
	dac_sample_rate_set(in_length);
	dac_mono_set();

	DBG_PRINT("bps: %d\r\n", A16_dec_get_bitspersample((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", A16_dec_get_channel((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", A16_dec_get_samplerate((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("block size: %d\r\n", A16_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_a16_process(void)
{

	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], audio_ctrl.ring_size);
	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();
	return 0;
}
#endif	// #if APP_A1600_DECODE_FG_EN == 1

//===============================================================================================================
//   A6400 Playback
//===============================================================================================================
#if APP_A6400_DECODE_FG_EN == 1
INT32S audio_a64_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT32S  t_wi;

	while(1)
	{
		audio_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

		 /* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		in_length = audio_ctrl.file_len - 1;
		if(audio_ctrl.file_cnt >= in_length) {
			if(audio_ctrl.f_last) {
				return 0; //AUDIO_ERR_DEC_FINISH;
			} else {
				audio_ctrl.f_last = 1;
			}
		}

		in_length = audio_ctrl.ri;
		pcm_point = a6400_dec_run((CHAR*)audio_ctrl.work_mem, Buffer, audio_ctrl.wi);

		audio_decode_sample_number += pcm_point;
		audio_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FINISH);
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
		} else {
			break;
		}
	}

	return (pcm_point * a6400_dec_get_channel((CHAR*)work_mem));
}

INT32S audio_a64_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_decode_sample_number = 0;
	audio_ctrl.file_cnt = 0;
	audio_ctrl.f_last = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	//a6400 init
	ret = a6400_dec_get_mem_block_size();
	if(ret != A6400_DEC_MEMORY_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = a6400_dec_init((char*)audio_ctrl.work_mem, (unsigned char*)audio_ctrl.ring_buf, (char*)(audio_ctrl.work_mem + A6400_DEC_MEMORY_SIZE));
	a6400_dec_set_bs_buf_size((char*)audio_ctrl.work_mem, audio_ctrl.ring_size);

	audio_ctrl.wi = a6400_dec_get_ri((void *)audio_ctrl.work_mem);//after initial the value is 0
	audio_ctrl.ri = a6400_dec_get_ri((void *)audio_ctrl.work_mem);
	in_length = audio_ctrl.ri;

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = a6400_dec_parsing((CHAR*)audio_ctrl.work_mem , audio_ctrl.wi);

		audio_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
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
				audio_ctrl.ri = a6400_dec_get_ri((CHAR*)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
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

	in_length = a6400_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);
	ret = a6400_dec_get_channel((CHAR*)audio_ctrl.work_mem);

	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_a64_get_output;

#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ret == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ret, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ret = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ret == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ret, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ret = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
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
#if APP_UP_SAMPLE_EN
	count = 48000 / in_length;
	if(count > 4)  count = 4;
	if(hUpSample && count >= 2)
	{
		if(ret == 1)
			UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 1);
		else
			UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ret = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif //APP_UP_SAMPLE_EN

	channel = ret;
	g_audio_sample_rate = in_length;
	drv_l1_dac_sample_rate_set(in_length);
	if(ret == 1) {
		drv_l1_dac_mono_set();
	} else {
		drv_l1_dac_stereo_set();
	}
	sample_rate = a6400_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);
	DBG_PRINT("bps: %d\r\n",a6400_dec_get_bitrate((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",a6400_dec_get_channel((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n",a6400_dec_get_samplerate((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("block size: %d\r\n",a6400_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_a64_process(void)
{
	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], A6400_DEC_FRAMESIZE);
	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();
	return 0;
}
#endif // #if APP_A6400_DECODE_FG_EN == 1

//===============================================================================================================
//   S880 Playback
//===============================================================================================================
#if APP_S880_DECODE_FG_EN == 1
INT32S audio_s880_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT32S  t_wi;

	while(1)
	{
		audio_ctrl.ri = S880_dec_get_ri((void *)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);
		if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) { /* check reading data */
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return 0; //AUDIO_ERR_DEC_FINISH;
		}

		in_length = audio_ctrl.ri;
		pcm_point = S880_dec_run((CHAR*)audio_ctrl.work_mem, Buffer, audio_ctrl.wi);

		audio_ctrl.ri = S880_dec_get_ri((CHAR*)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FAIL);
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
		} else {
			break;
		}
	}

	return pcm_point;
}

INT32S audio_s880_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_ctrl.file_cnt = 0;
	audio_ctrl.f_last = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;//080724
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	//s880 init
	ret = S880_dec_get_mem_block_size();
	if(ret != S880_DEC_MEMORY_SIZE) {
		return AUDIO_ERR_DEC_FAIL;
	}

	ret = S880_dec_init((char *)audio_ctrl.work_mem, (unsigned char *)audio_ctrl.ring_buf);//080723
	audio_ctrl.wi = S880_dec_get_ri((void *)audio_ctrl.work_mem);//after initial the value is 0
	audio_ctrl.ri = S880_dec_get_ri((void *)audio_ctrl.work_mem);
	in_length = audio_ctrl.ri;

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = S880_dec_parsing((CHAR*)audio_ctrl.work_mem , audio_ctrl.wi);

		audio_ctrl.ri = S880_dec_get_ri((CHAR*)audio_ctrl.work_mem);

		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		switch(ret)
		{
			case S880_OK:
				break;

			case S880_E_NO_MORE_SRCDATA:		//not found sync word
			case S880_E_READ_IN_BUFFER:			//reserved sample frequency value
			case S880_CODE_FILE_FORMAT_ERR:		//forbidden bitrate value
			case S880_E_FILE_END:
		//	case S880_E_MODE_ERR:
				//feed in DecodeInBuffer;
				audio_ctrl.ri = S880_dec_get_ri((CHAR*)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi) != AUDIO_ERR_NONE) {
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

	in_length = S880_dec_get_samplerate((CHAR*)audio_ctrl.work_mem);
	ret = S880_dec_get_channel((CHAR*)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_s880_get_output;

#if APP_CONST_PITCH_EN
	if(hConstPitch && G_snd_info.ConstPitchEn && ret == 1)
	{
		if(in_length == 8000 || in_length == 16000 || in_length == 11025 || in_length == 22050)
		{
			ConstantPitch_Link(hConstPitch, hSrc, pfnGetOutput, in_length, ret, 0);
			ConstantPitch_SetParam(hConstPitch, G_snd_info.Pitch_idx, G_snd_info.Vox_thr);
			hSrc = hConstPitch;
			in_length = ConstantPitch_GetSampleRate(hConstPitch);
			ret = ConstantPitch_GetChannel(hConstPitch);
			pfnGetOutput = &ConstantPitch_GetOutput;
		}
	}
#endif
#if APP_ECHO_EN
	if(hEcho && G_snd_info.EchoEn && ret == 1)
	{
		Echo_Link(hEcho, hSrc, pfnGetOutput, in_length, ret, 0);
		Echo_SetParam(hEcho, G_snd_info.delay_len, G_snd_info.weight_idx);
		hSrc = hEcho;
		in_length = Echo_GetSampleRate(hEcho);
		ret = Echo_GetChannel(hEcho);
		pfnGetOutput = &Echo_GetOutput;
	}
#endif
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
#if APP_UP_SAMPLE_EN
	count = 48000 / in_length;
	if(count > 4)	count = 4;
	if(hUpSample && count >= 2)
	{
		UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ret = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif

	channel = ret;
	g_audio_sample_rate = in_length;
	dac_sample_rate_set(in_length);
	dac_mono_set();

	DBG_PRINT("bps: %d\r\n",S880_dec_get_bitspersample((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n",S880_dec_get_channel((CHAR*)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", ret);
	DBG_PRINT("block size: %d\r\n",S880_dec_get_mem_block_size());

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S  audio_s880_process(void)
{
	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], S880_DEC_FRAMESIZE);
	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();
	return 0;
}
#endif // #if APP_S880_DECODE_FG_EN == 1

//===============================================================================================================
//  AAC Playback
//===============================================================================================================
#if APP_AAC_DECODE_FG_EN == 1
INT32S audio_aac_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT32S  t_wi;
    INT32S  ret = 0;

	while(1)
	{

		audio_ctrl.ri = aac_dec_get_read_index((void *)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

		/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return -1;
		}

		in_length = audio_ctrl.ri;
		pcm_point = aac_dec_run((void *)audio_ctrl.work_mem, audio_ctrl.wi, Buffer);
		ret = aac_dec_get_channel((void *)audio_ctrl.work_mem);
        if(ret != channel){
            if (ret == 1) {
                drv_l1_dac_mono_set();
            } else {
                drv_l1_dac_stereo_set();
            }
            channel = ret;
        }


		audio_ctrl.ri = aac_dec_get_read_index((void *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FAIL);
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
		} else {
			break;
		}
	}

	return (pcm_point * aac_dec_get_channel((void *)audio_ctrl.work_mem));
}

INT32S audio_aac_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;
	INT32S downMatrix = 0;	// 1: input 5.1 channels and output 2 channels

	audio_ctrl.file_cnt = 0;
	audio_ctrl.f_last = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	// aac init
	ret = aac_dec_init((void *)audio_ctrl.work_mem, downMatrix, (INT8U *)audio_ctrl.ring_buf);
	aac_dec_SetRingBufferSize((void *)audio_ctrl.work_mem,audio_ctrl.ring_size);
	audio_ctrl.wi = 0;
	audio_ctrl.ri = aac_dec_get_read_index((void *)audio_ctrl.work_mem);

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = aac_dec_parsing((void *)audio_ctrl.work_mem, audio_ctrl.wi);

		audio_ctrl.ri = aac_dec_get_read_index((void *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return AUDIO_ERR_FAILED;
		}

		switch(ret)
		{
			case AAC_OK:
				break;

			case  UNABLE_TO_FIND_ADTS_SYNCWORD:
				//feed in DecodeInBuffer;
				audio_ctrl.ri = aac_dec_get_read_index((void *)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
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

	in_length = aac_dec_get_samplerate((void *)audio_ctrl.work_mem);
	ret = aac_dec_get_channel((void *)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_aac_get_output;

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
	g_audio_sample_rate = aac_dec_get_samplerate((void *)audio_ctrl.work_mem);
	drv_l1_dac_sample_rate_set(in_length);

	if (ret == 1) {
		drv_l1_dac_mono_set();
	} else {
		drv_l1_dac_stereo_set();
	}

	DBG_PRINT("bps: %d\r\n", aac_dec_get_bitspersample((void *)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", aac_dec_get_channel((void *)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", aac_dec_get_samplerate((void *)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(audio_wq, (void *) count);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
    #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S audio_aac_process(void)
{
	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif
	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], AAC_DEC_FRAMESIZE);
	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
#endif
	audio_send_to_dma();
	audio_send_next_frame_q();

	return 0;
}
#endif // #if APP_AAC_DECODE_FG_EN == 1

//===============================================================================================================
//  OGG Playback
//===============================================================================================================
#if APP_OGG_DECODE_FG_EN == 1
#if 0
INT32S audio_ogg_play_init(void)
{


	INT32U  in_length;
	INT32S  ret;
	INT32U  count;
	INT32S  t_wi;

	audio_ctrl.file_cnt = 0;
	audio_ctrl.f_last = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;
	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	// aac init
	ret = oggvorbis_dec_init((void *)audio_ctrl.work_mem, (const char*)audio_ctrl.ring_buf, audio_ctrl.ring_size, 0);
	oggvorbis_dec_set_ring_buffer((void *)audio_ctrl.work_mem, (const char*)audio_ctrl.ring_buf, audio_ctrl.ring_size, 0);
	audio_ctrl.wi = 0;
	audio_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_ctrl.work_mem);

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = oggvorbis_dec_parsing((void *)audio_ctrl.work_mem , audio_ctrl.wi);

		audio_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return AUDIO_ERR_FAILED;
		}

		switch(ret)
		{
			case OGGVORBIS_PARSING_OK:
				break;

			case OGGVORBIS_MORE_DATA:
				//feed in DecodeInBuffer;
				audio_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
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

	in_length = oggvorbis_dec_get_samplerate((void *)audio_ctrl.work_mem);
	ret = oggvorbis_dec_get_channel((void *)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
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
	g_audio_sample_rate = oggvorbis_dec_get_samplerate((void *)audio_ctrl.work_mem);
	if(g_audio_main_channel == 3)
	{
        switch(in_length)
        {
            case 44100:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_44K);
            break;
            case 48000:
                drv_l1_hdmi_audio_sample_rate_set(HDMI_AUD_48K);
            break;
            default:break;
        }
        drvl1_hdmi_audio_ctrl(1);
	}
	else
	{
        drv_l1_dac_sample_rate_set(in_length);
    }
    if (ret == 1) {
        drv_l1_dac_mono_set();
    } else {
        drv_l1_dac_stereo_set();
    }
	DBG_PRINT("bps: %d\r\n", oggvorbis_dec_get_bitrate((void *)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", oggvorbis_dec_get_channel((void *)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", oggvorbis_dec_get_samplerate((void *)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPost(audio_wq, (void *) count);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
        #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S audio_ogg_process(void)
{
	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;
    INT8U   wait;
    INT32S  t_wi;
    INT32U  in_length;

	audio_ctrl.ri = oggvorbis_dec_get_ri((CHAR *)audio_ctrl.work_mem);
	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    	wb_idx = 0;
    	xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
#endif

        wait = 1;
		if (audio_check_wi(t_wi, &audio_ctrl.wi, wait) != AUDIO_ERR_NONE) {
			return AUDIO_ERR_DEC_FAIL;
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return AUDIO_ERR_DEC_FINISH;
		}

		in_length = audio_ctrl.ri;

		pcm_point = oggvorbis_dec_run((CHAR *)audio_ctrl.work_mem, pcm_out[wb_idx], audio_ctrl.wi);
		audio_ctrl.ri = oggvorbis_dec_get_ri((CHAR *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length >= audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return AUDIO_ERR_DEC_FAIL;
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
                    OSQPostFront(audio_wq, (void *)wb_idx);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                    xQueueSendToFront(audio_wq, &wb_idx, portMAX_DELAY);
            #endif
                    audio_send_next_frame_q();
                    return 0;

		}
	pcm_len[wb_idx] = pcm_point * 2;//oggvorbis_dec_get_channel((CHAR *)audio_ctrl.work_mem);


	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
    #endif
	audio_send_to_dma();
	audio_send_next_frame_q();

	return 0;
}
#else
INT32S audio_ogg_get_output(void *work_mem, short *Buffer, INT32S MaxLen)
{
	INT32S  pcm_point;
	INT32U  in_length;
	INT32S  t_wi;
    INT8U   wait;
	while(1)
	{
		audio_ctrl.ri = oggvorbis_dec_get_ri((CHAR *)audio_ctrl.work_mem);
		t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);
        wait = 1;

		/* check reading data */
		if (audio_check_wi(t_wi, &audio_ctrl.wi, wait) != AUDIO_ERR_NONE) {
			return (0 - AUDIO_ERR_DEC_FAIL);
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return -1;
		}

		in_length = audio_ctrl.ri;
		pcm_point = oggvorbis_dec_run((CHAR *)audio_ctrl.work_mem, Buffer, audio_ctrl.wi);

		audio_ctrl.ri = oggvorbis_dec_get_ri((CHAR *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(in_length > audio_ctrl.ri) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if (pcm_point <= 0) {
			if (pcm_point < 0) {
				if (--audio_ctrl.try_cnt == 0) {
					return (0 - AUDIO_ERR_DEC_FAIL);
				}
			}
			//OSQPostFront(audio_wq, (void *)wb_idx);
			//audio_send_next_frame_q();
			//return 0;
		} else {
			break;
		}
	}

	return (pcm_point * oggvorbis_dec_get_channel((CHAR *)audio_ctrl.work_mem));
}

INT32S audio_ogg_play_init(void)
{
	INT32U  count;
	INT32S  ret = 0;
	INT32U  in_length;
	INT32S  t_wi;

	audio_ctrl.file_cnt = 0;
	audio_ctrl.f_last = 0;
	audio_ctrl.try_cnt = 20;
	audio_ctrl.read_secs = 0;

    nand_ogg_read_index = 0;

	gp_memset((INT8S*)audio_ctrl.ring_buf, 0, audio_ctrl.ring_size);

	// aac init
	ret = oggvorbis_dec_init((void *)audio_ctrl.work_mem, (const char*)audio_ctrl.ring_buf, audio_ctrl.ring_size, 0);
	oggvorbis_dec_set_ring_buffer((void *)audio_ctrl.work_mem, (const char*)audio_ctrl.ring_buf, audio_ctrl.ring_size, 0);
	audio_ctrl.wi = 0;
	audio_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_ctrl.work_mem);

	t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

	/* check reading data */
	if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
		return AUDIO_ERR_READ_FAIL;
	}

	count = 500;
	while(1)
	{
		in_length = audio_ctrl.ri;
		ret = oggvorbis_dec_parsing((void *)audio_ctrl.work_mem , audio_ctrl.wi);

		audio_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_ctrl.work_mem);
		audio_ctrl.file_cnt += (audio_ctrl.ri - in_length);
		if(audio_ctrl.ri < in_length) {
			audio_ctrl.file_cnt += audio_ctrl.ring_size;
		}

		if(audio_ctrl.file_cnt >= audio_ctrl.file_len) {
			return AUDIO_ERR_FAILED;
		}

		switch(ret)
		{
			case OGGVORBIS_PARSING_OK:
				break;

			case OGGVORBIS_MORE_DATA:
				//feed in DecodeInBuffer;
				audio_ctrl.ri = oggvorbis_dec_get_ri((void *)audio_ctrl.work_mem);
				t_wi = audio_write_with_file_srv(audio_ctrl.ring_buf, audio_ctrl.wi, audio_ctrl.ri);

				/* check reading data */
				if (audio_check_wi(t_wi, &audio_ctrl.wi, 1) != AUDIO_ERR_NONE) {
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

	in_length = oggvorbis_dec_get_samplerate((void *)audio_ctrl.work_mem);
	ret = oggvorbis_dec_get_channel((void *)audio_ctrl.work_mem);
	hSrc = audio_ctrl.work_mem;
	pfnGetOutput = audio_ogg_get_output;

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
#if APP_UP_SAMPLE_EN
	count = 44100/in_length;
	if(count > 4)	count = 4;
	if(hUpSample && (count >= 2))
	{
		if(ret == 1)
			UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 1);
		else
			UpSample_Link(hUpSample, hSrc, pfnGetOutput, in_length, ret, count, 0);
		hSrc = hUpSample;
		in_length = UpSample_GetSampleRate(hUpSample);
		ret = UpSample_GetChannel(hUpSample);
		pfnGetOutput = &UpSample_GetOutput;
	}
#endif //APP_UP_SAMPLE_EN

	channel = ret;
	g_audio_sample_rate = oggvorbis_dec_get_samplerate((void *)audio_ctrl.work_mem);
	drv_l1_dac_sample_rate_set(in_length);
	drv_l1_dac_hw_up_sample_set(in_length);
	if (ret == 1) {
		drv_l1_dac_mono_set();
	} else {
		drv_l1_dac_stereo_set();
	}
	sample_rate = g_audio_sample_rate;
	DBG_PRINT("bps: %d\r\n", oggvorbis_dec_get_bitrate((void *)audio_ctrl.work_mem));
	DBG_PRINT("channel: %d\r\n", oggvorbis_dec_get_channel((void *)audio_ctrl.work_mem));
	DBG_PRINT("sample rate: %d\r\n", oggvorbis_dec_get_samplerate((void *)audio_ctrl.work_mem));

	for (count=0;count<MAX_DAC_BUFFERS;count++) {
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
        OSQPost(audio_wq, (void *) count);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        xQueueSend(audio_wq, &count, portMAX_DELAY);
        #endif
	}

	return AUDIO_ERR_NONE;
}

INT32S audio_ogg_process(void)
{
	INT32S  pcm_point;
	INT8U   err;
	INT32U  wb_idx;
    INT8U   wait;

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	wb_idx = (INT32U) OSQPend(audio_wq, 0, &err);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    wb_idx = 0;
    xQueueReceive(audio_wq, &wb_idx, portMAX_DELAY);
    #endif


	pcm_point = pfnGetOutput(hSrc, pcm_out[wb_idx], OGGVORBIS_DEC_FRAMESIZE);


	if (pcm_point <= 0) {
		return AUDIO_ERR_DEC_FINISH;
	}

	pcm_len[wb_idx] = pcm_point;


	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(aud_send_q, (void *) wb_idx);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueSend(aud_send_q, &wb_idx, portMAX_DELAY);
    #endif
	audio_send_to_dma();
	audio_send_next_frame_q();

	return 0;
}
#endif

#endif // #if APP_OGG_DECODE_FG_EN == 1

#if (defined SKIP_ID3_TAG) && (SKIP_ID3_TAG == 1)
static INT8U audio_get_id3_type(INT8U *data, INT32U length)
{
	if (length >= 3 && data[0] == 'T' && data[1] == 'A' && data[2] == 'G') {
    	return ID3_TAG_V1;
	}

  	if (length >= 10 && ((data[0] == 'I' && data[1] == 'D' && data[2] == '3') ||
      	(data[0] == '3' && data[1] == 'D' && data[2] == 'I')) && data[3] < 0xff && data[4] < 0xff)
   	{
   		if (data[0] == 'I')
   			return ID3_TAG_V2;
   		else
   			return ID3_TAG_V2_FOOTER;
	}
  	return ID3_TAG_NONE;
}

static void audio_parse_id3_header(INT8U *header, INT32U *version, INT32S *flags, INT32U *size)
{
	INT8U *ptr;
	INT32U ver = 0;
	ptr = header;
  	ptr += 3;

  	ver = *ptr++ << 8;
  	*version = ver | *ptr++;
  	*flags   = *ptr++;
  	*size    = audio_id3_get_size(ptr);

}

static INT32U audio_id3_get_size(INT8U *ptr)
{
	INT32U value = 0;

	value = (value << 7) | (*ptr++ & 0x7f);
    value = (value << 7) | (*ptr++ & 0x7f);
	value = (value << 7) | (*ptr++ & 0x7f);
	value = (value << 7) | (*ptr++ & 0x7f);

  	return value;
}

INT32S audio_id3_get_tag_len(INT8U *data, INT32U length)
{
  INT32U version;
  INT32S flags;
  INT32U len;

  switch (audio_get_id3_type(data, length)) {
 	case ID3_TAG_V1:
    	return 128;
  	case ID3_TAG_V2:
    	audio_parse_id3_header(data, &version, &flags, &len);
    	if (flags & ID3_TAG_FLAG_FOOTER) {
      		len += 10;
		}
    	return 10 + len;
  	case ID3_TAG_NONE:
    	break;
  }

  return 0;
}
#endif

// MP3 parser for seek
/*========================================================================
 MP3 parser for seek
=========================================================================*/
#if APP_MP3_DECODE_FG_EN==1

#define		CODEC_ID_MP2				0x15000
#define		CODEC_ID_MP3				0x15001


#define ID3_V1_TAG_LEN


#define MAD_FLAG_LSF_EXT			0x1000	// lower sampling freq. extension
//#define MAD_FLAG_MC_EXT				0x2000	// multichannel audio extension
#define MAD_FLAG_MPEG_2_5_EXT		0x4000	// MPEG 2.5 (unofficial) extension


#define MAD_LAYER_I		1			// Layer I
//#define MAD_LAYER_II 	2			// Layer II/
#define MAD_LAYER_III	3			// Layer III/

#define streamRead fs_read//read
#define streamSeek lseek

typedef struct
{
	INT16S  fin;

	int data_start;
	int data_size;
	int data_cur;

	int prev_time;
	int next_time;
	int dec_sample_num;

	unsigned int first_bitrate;
	unsigned int bitrate;
	unsigned int samplerate;
	unsigned int SamplePerFrame;
	unsigned int channels;
	unsigned int layer;
} MP3_PARSER;

MP3_PARSER g_mp3_parser;

static unsigned short const bitrate_table[5][15] = {
  /* MPEG-1, LSF = 0*/
  { 0,  32,  64,  96, 128, 160, 192, 224,  /* Layer I   */
       256, 288, 320, 352, 384, 416, 448 },
  { 0,  32,  48,  56,  64,  80,  96, 112,  /* Layer II  */
       128, 160, 192, 224, 256, 320, 384 },
  { 0,  32,  40,  48,  56,  64,  80,  96,  /* Layer III */
       112, 128, 160, 192, 224, 256, 320 },

  /* MPEG-2, LSF = 1 */
  { 0,  32,  48,  56,  64,  80,  96, 112,  /* Layer I   */
       128, 144, 160, 176, 192, 224, 256 },
  { 0,   8,  16,  24,  32,  40,  48,  56,  /* Layers II & III */
        64,  80,  96, 112, 128, 144, 160 }
};

static unsigned int const SamplePerFrame_table[3] = { 384, 1152, 576 };
static unsigned int const samplerate_table[3] = { 44100, 48000, 32000 };

static unsigned int udiv_32by32(unsigned int d, unsigned int n)
{
	unsigned q = 0, r= n, N = 32;
	do{
		N--;
		if((r>>N)>=d){
			r -= (d<<N);
			q += (1<<N);
		}
	}while(N);
	return q;
}

static unsigned int find_syncword(MP3_PARSER *Parser, int flag, int *frame_size)
{
	unsigned char byte0,byte1,byte2,byte3;
	int index;
	unsigned int ri;
//begin add by zgq on 20090220 for parsing
	unsigned int pre_ri = 0;
	unsigned short pre_samplerate = 0;
	short pre_channel = 0;
	short pre_layer = 0;
	short mpeg25;
	short bitrate_index;
	short pad_slot;

	//struct mad_header *header;
	unsigned char layer;
	int flags, resync = 0;
	unsigned int samplerate;
	unsigned short bitrate;


	int N = 0;
	short stereo;

	ri = streamSeek(Parser->fin,0,SEEK_CUR);

	while (1)
	{
		flags = 0;		//add by zgq on 20090402
		if (streamRead(Parser->fin,(INT32U)&byte0, sizeof(char)) != 1)
			break;
		if (streamRead(Parser->fin,(INT32U)&byte1, sizeof(char)) != 1)
			break;

		if (!((byte0 == 0xff) && ((byte1 & 0xe0) == 0xe0)))
		{
			ri++;
			streamSeek(Parser->fin, -1, SEEK_CUR);
			resync = 1;
			continue;
		}
		else
		{
			//mpegid
			if (streamRead(Parser->fin,(INT32U)&byte2, sizeof(char)) != 1)
				break;
			if (streamRead(Parser->fin,(INT32U)&byte3, sizeof(char)) != 1)
				break;


			if (byte1 & 0x10)
			{
				mpeg25 = 0;
				if((byte1 & 0x8) == 0)
					flags |= MAD_FLAG_LSF_EXT;

				flags &= ~MAD_FLAG_MPEG_2_5_EXT;
			}
			else
			{
				mpeg25 = 1;
				flags |= MAD_FLAG_LSF_EXT;
				flags |= MAD_FLAG_MPEG_2_5_EXT;
				if(byte1 & 0x08)
				{
					ri++;
					streamSeek(Parser->fin, -3, SEEK_CUR);
					resync = 1;
					continue;
				}
			}


			//layer description
			index = (byte1 & 0x06) >> 1;
			if(index == 0x00)
			{
				ri++;
				streamSeek(Parser->fin, -3, SEEK_CUR);
				resync = 1;
				continue;
			}
			else
			{
				layer = 4 - index;
				Parser->layer = layer;
			}

			//bitrate
			index = (byte2 & 0xF0)>>4;
			if(index == 0x0F)
			{
				ri++;
				streamSeek(Parser->fin, -3, SEEK_CUR);
				resync = 1;
				continue;
			}
			else
			{
				bitrate_index = index;

				if (flags & MAD_FLAG_LSF_EXT)
					bitrate = bitrate_table[3 + (layer >> 1)][index];
				else
					bitrate = bitrate_table[layer - 1][index];

				Parser->bitrate = bitrate * 1000;//add by zgq on 20090303

				if (flag && (pre_samplerate == 0 || pre_channel == 0 || pre_layer == 0))
				{
					Parser->first_bitrate = Parser->bitrate;
					Parser->SamplePerFrame = SamplePerFrame_table[layer - 1];

					if ((!(flags & MAD_FLAG_LSF_EXT)) && (layer == 3))
						Parser->SamplePerFrame = 1152;
				}
			}

			//sample rate
			index = (byte2 & 0x0C)>>2;
			if(index == 0x03)
			{
			    ri++;
				streamSeek(Parser->fin, -3, SEEK_CUR);
				resync = 1;
				continue;
			}
			else
			{
				samplerate =  samplerate_table[index];

				index = byte1 & 0x18;
				if(index == 0x00)						//MPEG2.5
					samplerate = samplerate >> 2;
				else if(index == 0x10)					//MPEG2
					samplerate = samplerate >> 1;

				Parser->samplerate = samplerate;
			}

			//channel mode
			index = (byte3 >> 6) & 0x03;
			stereo = (index == 0x03 ? 1 : 2);
			Parser->channels = stereo;
			//header->mode = 3 - index;

			// check next sample rate, layer, channel ...
			if(pre_samplerate == 0 || pre_channel == 0 || pre_layer == 0)
			{
				int ri_tmp2;

				/* calculate beginning of next frame */
				pad_slot = (byte2 >> 1) & 0x01;

				if(bitrate_index != 0)
				{
					if (layer == MAD_LAYER_I)
					{
						//N = ((12 * header->bitrate / header->samplerate) + pad_slot) * 4;
						unsigned int temp = 12 * Parser->bitrate;
						N = (udiv_32by32(samplerate, temp) + pad_slot) * 4;
					}
					else
					{
						unsigned int slots_per_frame;
						unsigned int temp;

						slots_per_frame = ((layer == MAD_LAYER_III) &&
							   (flags & MAD_FLAG_LSF_EXT)) ? 72 : 144;

						temp = slots_per_frame * Parser->bitrate;
						N = udiv_32by32(samplerate, temp) + pad_slot;
					}
				}
				else
				{
					int header0 = (byte0 << 24) | (byte1 << 16) | (byte2 << 8) | byte3;
					unsigned int newhead=0;
					int j = 0;

					streamSeek(Parser->fin, ri, SEEK_SET);

					while(1)
					{

						unsigned char b;
						j += 1;
						if (streamRead(Parser->fin,(INT32U)&b, sizeof(char)) != 1)
							break;
						newhead <<= 8;
						newhead |= b;
						newhead &= 0xffffffff;

						if(((header0 & 0xfffefc00) == (newhead & 0xfffefc00)))
						{
							if(layer == 1)
								N = j - 4 - pad_slot *4; // header and padding not included
							else if((layer ==2) || (layer == 3))
								N = j - 4 - pad_slot;

							//mp3_dec_set_ri(mp3dec, ri);
							break;
						}

					} // while(!feof(Parser->fin));

					N = N + pad_slot;
				}

				if (N == 0)
				{
					ri++;
					streamSeek(Parser->fin, -3, SEEK_CUR);
					resync = 1;
					continue;
				}


				if (!flag && !resync)
				{
					*frame_size = N;
					streamSeek(Parser->fin, -4, SEEK_CUR);
					break;
				}

				// pass bitstream
				ri_tmp2 = ri + N;

				streamSeek(Parser->fin, N - 4, SEEK_CUR);

				if (streamRead(Parser->fin,(INT32U)&byte0, sizeof(char)) != 1)
					break;
				if (streamRead(Parser->fin,(INT32U)&byte1, sizeof(char)) != 1)
					break;

				// check next header...
				if ((byte0 == 0xff) && ((byte1 & 0xe0) == 0xe0))
				{
					pre_ri = ri;
					pre_samplerate = samplerate;
					pre_channel = stereo;
					pre_layer = layer;
					ri = ri_tmp2;
				}
				else
				{
					ri++;
					resync = 1;
				}

				streamSeek(Parser->fin, ri, SEEK_SET);
				continue;
			}
			else
			{
				if(pre_samplerate != samplerate || pre_channel != stereo || pre_layer != layer)
				{
					pre_samplerate = 0;
					pre_channel = 0;
					pre_layer = 0;
					ri = pre_ri + 1;
					resync = 1;
					continue;
				}
				else
					ri = pre_ri;
			}

			break;
		}
	}

	if (!flag)
		*frame_size = N;

	return ri;
}

int MP3Parser_Init(INT16S fin)
{
	MP3_PARSER *Parser = &g_mp3_parser;
	int ret = 0;

	char ape_tag[21];
	unsigned char id3_tag1, id3_tag2, id3_tag3;
	int pos;

	gp_memset((INT8S *)&g_mp3_parser,0,sizeof(MP3_PARSER));
	Parser->fin = fin;
	if(Parser->fin < 0) {
		return -1;
	}

	streamSeek(Parser->fin, 0, SEEK_SET) ;

	Parser->prev_time = 0;
	Parser->next_time = 0;
	Parser->dec_sample_num = 0;

	// ========================= remove ID3v2 @ file start =========================

	streamRead(Parser->fin,(INT32U)&id3_tag1, sizeof(char));
	streamRead(Parser->fin,(INT32U)&id3_tag2, sizeof(char));
	streamRead(Parser->fin,(INT32U)&id3_tag3, sizeof(char));

	if ((id3_tag1=='I') && (id3_tag2=='D') && (id3_tag3=='3'))
	{
		unsigned int id3_length = 0;
		unsigned char x;

		streamSeek(Parser->fin, 3, SEEK_CUR);

		streamRead(Parser->fin,(INT32U)&x, sizeof(char));
		id3_length |= (x & 0x7F) << 21;

		streamRead(Parser->fin,(INT32U)&x, sizeof(char));
		id3_length |= (x & 0x7F) << 14;

		streamRead(Parser->fin,(INT32U)&x, sizeof(char));
		id3_length |= (x & 0x7F) << 7;

		streamRead(Parser->fin,(INT32U)&x, sizeof(char));
		id3_length |= (x & 0x7F);

		id3_length += 0x0A;

		streamSeek(Parser->fin, id3_length - 10, SEEK_CUR);
	}
	else
		streamSeek(Parser->fin, 0, SEEK_SET);


	// ========================= get bitrate =========================
	Parser->data_start = find_syncword(Parser, 1, NULL);

	// ========================= remove APETAG @ file end =========================
	streamSeek(Parser->fin, -500, SEEK_END);

	while (1)
	{
		pos = streamRead(Parser->fin,(INT32U)ape_tag, 20);

		if (pos != 20)
			break;
		else
		{
			pos = strcspn(ape_tag, "APETAGEX");

			if (strncmp(ape_tag + pos, "APETAGEX", 8))
				streamSeek(Parser->fin, -7, SEEK_CUR);
			else
			{
				streamSeek(Parser->fin, -(20-pos) + 12, SEEK_CUR);
				pos = 0xFF;
				break;
			}
		}
	}

	if (pos == 0xFF)	// APETAGEX exists
		Parser->data_size = lseek(Parser->fin,0,SEEK_CUR) - 12 - Parser->data_start;
	else	// ========================= remove ID3v1 @ file end =========================
	{
		streamSeek(Parser->fin, -128, SEEK_END);

		streamRead(Parser->fin,(INT32U)&id3_tag1, sizeof(char));
		streamRead(Parser->fin,(INT32U)&id3_tag2, sizeof(char));
		streamRead(Parser->fin,(INT32U)&id3_tag3, sizeof(char));

		if ((id3_tag1=='T') && (id3_tag2=='A') && (id3_tag3=='G'))
			Parser->data_size = lseek(Parser->fin,0,SEEK_CUR) - 3 - Parser->data_start;
		else
			Parser->data_size = lseek(Parser->fin,0,SEEK_CUR)+ 125 - Parser->data_start;
	}

	streamSeek(Parser->fin, 0, SEEK_SET); // set position to 0.
	Parser->data_cur = 0;

	return ret;
}

#define VBR_SUPPORTED
int MP3Parser_Seek( INT32U *p_msec,INT32U vbr)
{
	MP3_PARSER *Parser = &g_mp3_parser;
	long msec = *p_msec;
	int ret;

#ifdef VBR_SUPPORTED

	int frame = 0, frame_size;
	int target_frame;

	//DBG_PRINT("MP3Parser_seek entry \r\n");
	if(Parser->fin < 0) {
		return -1;
	}

	target_frame = (long)(((INT64S)msec * Parser->samplerate) / (Parser->SamplePerFrame * 1000));

	streamSeek(Parser->fin, Parser->data_start, SEEK_SET);
	if(vbr==0)
	{
		find_syncword(Parser, 0, &frame_size);
		streamSeek(Parser->fin,frame_size*target_frame,SEEK_CUR);
	}
	else
	{
		Parser->data_cur = 0;
		while ((frame < target_frame) && (Parser->data_cur < Parser->data_size))
		{
			//frame_size = get_frame_size(Parser);
			find_syncword(Parser, 0, &frame_size);
			streamSeek(Parser->fin, frame_size, SEEK_CUR);
			Parser->data_cur += frame_size;
			frame++;
		}

		msec = (long)(((INT64S)frame * Parser->SamplePerFrame * 1000) / Parser->samplerate);
		//DBG_PRINT("MP3Parser_seek exit \r\n");
	}
#else

	long t32;
	unsigned int ri;

	if(Parser->fin < 0) {
		return -1;
	}

	t32 = (long)(((INT64S)msec * Parser->first_bitrate) / (8 * 1000));

	if (t32 >= Parser->data_size)
	{
		t32 = Parser->data_start + Parser->data_size;
		streamSeek(Parser->fin, t32, SEEK_SET);
		Parser->data_cur = Parser->data_size;
	}
	else
	{
		// ===== find syncword =====
		t32 += Parser->data_start;
		streamSeek(Parser->fin, t32, SEEK_SET);
		ri = find_syncword(Parser, 0);


		if(streamSeek(Parser->fin, ri, SEEK_SET)!=0) ERROR("File seek failed");
		Parser->data_cur = ri - Parser->data_start;
	}

	msec = (long)(((INT64S)Parser->data_cur * 8 * 1000) / Parser->first_bitrate);

#endif

	*p_msec = msec;

	Parser->prev_time = Parser->next_time = msec;
	Parser->dec_sample_num = 0;

	ret = 0;
	return ret;
}

int MP3Parser_total_time()
{
	MP3_PARSER *Parser = &g_mp3_parser;
	return (int)((INT64U)Parser->data_size * 8 * 1000 / Parser->bitrate);
}
#endif

INT32S Save_Last_PCM_File(void)
{
	INT32U PCM_Header_Addr;

	fs_write(gEncode_Filehandle , (INT32U)Save_Voice_Buffer , Write_Index);//20110517
	Write_Index = 0;
	lseek(gEncode_Filehandle,0,SEEK_SET);
	PCM_Header.RIFF_len = PCM_Header.data_len +36;//44-8
	PCM_Header_Addr = (INT32U)&PCM_Header;
	fs_write(gEncode_Filehandle , (INT32U)PCM_Header_Addr , 44);
	fs_close(gEncode_Filehandle);
	gEncode_Filehandle = -1;
	if(Save_Voice_Buffer)
	{
		gp_free((void *)Save_Voice_Buffer);//20110517
		Save_Voice_Buffer = 0;
    }
	return 0;
}




