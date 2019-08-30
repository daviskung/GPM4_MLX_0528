#ifndef __AVI_ENCODER_APP_H__
#define __AVI_ENCODER_APP_H__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "application.h"
#include "avi_encoder_scaler_jpeg.h"
#include "AviPackerV3.h"
//#include "wav_dec.h"
#include "portable.h"
#ifdef VFRAME_MANAGER
#include "gp_vfm.h"
#endif

#include "defs_MLX.h"


/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#ifndef ACK_FAIL
#define ACK_FAIL		(-1)
#endif

#ifndef ACK_OK
#define ACK_OK			0
#endif

#ifndef C_CDSP_DMA_CH
#define	C_CDSP_DMA_CH		0
#endif

// avi encode status
#define C_AVI_ENCODE_IDLE       0x00000000
#define C_AVI_ENCODE_PACKER0	0x00000001
#define C_AVI_ENCODE_AUDIO      0x00000002
#define C_AVI_ENCODE_VIDEO      0x00000004
#define C_AVI_ENCODE_SCALER     0x00000008
#define C_AVI_ENCODE_SENSOR     0x00000010
#define C_AVI_ENCODE_PACKER1	0x00000020
#define C_AVI_ENCODE_MD			0x00000040

#define C_AVI_ENCODE_START      0x00000100
#define C_AVI_ENCODE_PRE_START	0x00000200
#define C_AVI_ENCODE_JPEG		0x00000400
#define C_AVI_ENCODE_PAUSE		0x00000800
#define C_AVI_ENCODE_STREAM		0x00001000
#define C_AVI_ENCODE_ERR		0x20000000
#define C_AVI_ENCODE_FRAME_END	0x40000000
#define C_AVI_ENCODE_FIFO_ERR	0x80000000

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
#if VIDEO_TIMESTAMP
typedef struct VidRawFrame_s
{
	INT32U frame_addrs;
	INT32U pts;
} VidRawFrame_t;
#endif

typedef struct AviEncAudPara_s
{
	//audio encode
	INT32U  audio_format;			// audio encode format
	INT16U  audio_sample_rate;		// sample rate
	INT16U  channel_no;				// channel no, 1:mono, 2:stereo
	INT8U   *work_mem;				// wave encode work memory
	INT32U  pack_size;				// wave encode package size in byte
	INT32U  pack_buffer_addr;
	INT32U  pcm_input_size;			// wave encode pcm input size in short
	INT32U  pcm_input_addr[AVI_ENCODE_PCM_BUFFER_NO];
#if AVI_PACKER_LIB_EN == 0
	INT8U 	buf_idx;
	INT32U  pack_buffer[AVI_ENCODE_PCM_BUFFER_NO];
#endif
} AviEncAudPara_t;

typedef struct AviEncVidPara_s
{
	//sensor input
    INT32U  sensor_output_format;   // sensor output format
    INT16U  sensor_capture_width;   // sensor width
    INT16U  sensor_capture_height;  // sensor height
    INT32U  csi_frame_addr0;
    INT32U  csi_frame_addr[AVI_ENCODE_CSI_BUFFER_NO];	// sensor input buffer addr
    INT32U  csi_fifo_addr0;
    INT32U  csi_fifo_addr[AVI_ENCODE_CSI_FIFO_NO];		// sensor fifo mode use

    //scaler output
    INT16U 	scaler_flag;
    INT16U 	dispaly_scaler_flag;  	// 0: not scaler, 1: scaler again for display
    INT32U    scaler_zoom_ratio;      // for zoom scaler
    INT32U  scaler_output_addr0;
    INT32U  scaler_output_addr[AVI_ENCODE_SCALER_BUFFER_NO];	// scaler output buffer addr
#if VIDEO_TIMESTAMP
	VidRawFrame_t	frame_ts[AVI_ENCODE_SCALER_BUFFER_NO];
#endif

    //display scaler
    INT32U  display_output_format;  // for display use
    INT16U  display_width;          // display width
    INT16U  display_height;         // display height
    INT16U  display_buffer_width;   // display width
    INT16U  display_buffer_height;  // display height
    INT32U  display_output_addr0;
    INT32U  display_output_addr[AVI_ENCODE_DISPALY_BUFFER_NO];	// display scaler buffer addr
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    INT32U  rotate_input_addr0;
    INT32U  rotate_input_addr [AVI_ENCODE_ROTATE_BUFFER_NO];    // rotate buffer addr
#endif
    //video encode
    INT32U  video_format;			// video encode format
    INT8U   dwScale;				// dwScale
    INT8U   dwRate;					// dwRate
    INT16U  quality_value;			// video encode quality
    INT16U  encode_width;           // video encode width
    INT16U  encode_height;          // videoe ncode height
    INT32U  video_encode_addr0;
    INT32U  video_encode_addr[AVI_ENCODE_VIDEO_BUFFER_NO]; //video encode buffer
    INT32U	h264_bitrate;
    INT32U	h264_gop_len;
#ifdef VFRAME_MANAGER
    vframe_manager_t vfm;
#endif
    // jpeg encode
    INT8U 	encode_different_flag;
    INT8U 	jpeg_use_addr0_flag;
    INT8U   jpeg_encode_enable_flag;
    INT8U 	jpeg_encode_start_flag;
    INT16U 	jpeg_encode_skip_frame;
    INT16U 	jpeg_encode_in_format;
    INT16U 	jpeg_encode_timeout_exit;
    INT16U  jpeg_encode_width;           // jpeg encode width
    INT16U  jpeg_encode_height;          // jpeg encode height
    INT32U  JPEGEncodeInBuf;             // video JPEG encode use
    INT32U  JPEGEncodeOutBuf;            // video JPEG encode use
    INT32U  CSIOrgBufA;                  // CSI BUFA
    INT32U  CSIOrgBufB;                  // CSI BUFB
    INT32U  encode_header_size;          // jpeg / encode header size
} AviEncVidPara_t;

typedef struct AviEncPacker_s
{
	void 	*avi_workmem;
	INT16S  file_handle;
    INT16S  index_handle;
    INT8U   index_path[16];

    //avi video info
    GP_AVI_AVISTREAMHEADER *p_avi_vid_stream_header;
    INT32U  bitmap_info_cblen;		// bitmap header length
    GP_AVI_BITMAPINFO *p_avi_bitmap_info;

    //avi audio info
    GP_AVI_AVISTREAMHEADER *p_avi_aud_stream_header;
    INT32U  wave_info_cblen;		// wave header length
    GP_AVI_PCMWAVEFORMAT *p_avi_wave_info;

    //avi packer
	INT32U  task_prio;
	void    *file_write_buffer;
	INT32U  file_buffer_size;		// AviPacker file buffer size in byte
	void    *index_write_buffer;
	INT32U	index_buffer_size;		// AviPacker index buffer size in byte
} AviEncPacker_t;

typedef struct VidEncFrame_s
{
	INT32U ready_frame;
	INT32U encode_size;
#if VIDEO_TIMESTAMP
	INT32U pts;
#endif
	INT32U key_flag;
} VidEncFrame_t;

typedef struct AviEncPara_s
{
	INT8U  source_type;				// SOURCE_TYPE_FS or SOURCE_TYPE_USER_DEFINE
    INT8U  skip_cnt;
    INT8U  fifo_enc_err_flag;
    INT8U  fifo_irq_cnt;
    INT8U  *memptr;					// capture jpeg file for SOURCE_TYPE_USER_DEFINE

    INT8U sensor_interface;
    INT8U abBufFlag;
    INT8U RSD0;
    INT8U RSD1;

    //avi file info
    INT32U  avi_encode_status;      // 0:IDLE
    AviEncPacker_t *AviPackerCur;

	//allow record size
	INT64U  disk_free_size;			// disk free size
	INT32U  record_total_size;		// AviPacker storage to disk total size

	//aud & vid sync
	INT64S  delta_ta, tick;
    INT64S  delta_Tv;
    INT32S  freq_div;
    INT64S  ta, tv, Tv;
    INT32U  pend_cnt, post_cnt;
    INT32U  encoded_cnt;
	INT32U  vid_pend_cnt, vid_post_cnt;
	INT32U  dummy_cnt;
	VidEncFrame_t video[AVI_ENCODE_VIDEO_BUFFER_NO];
} AviEncPara_t;

typedef struct JpegPara_s
{
	INT32U jpeg_status;
	INT32U wait_done;
	INT32U quality_value;

	INT32U input_format;
	INT32U width;
	INT32U height;
	INT32U input_buffer_y;
	INT32U input_buffer_u;
	INT32U input_buffer_v;
	INT32U input_y_len;
	INT32U input_uv_len;
	INT32U output_buffer;
} JpegPara_t;

typedef struct capture_s
{
	INT16U width;
	INT16U height;
	INT32U quality;
	INT32U csi_buf;
	INT32U jpeg_buf;
	INT32U jpeg_len;
	osMessageQId Sensor_m;
	osMessageQId Ack_m;
} catpure_t;



typedef struct MLX_TH32x24Para_s
{
	//sensor input
    INT32U  MLX_TH32x24_output_format;   // MLX_TH32x24 output format
    INT16U  MLX_TH32x24_width;   // MLX_TH32x24 width
    INT16U  MLX_TH32x24_height;  // MLX_TH32x24 height

    INT32U  MLX_TH32x24_ColorOutputFrame_addr;	// MLX_TH32x24 color table output buffer addr
    INT32U  MLX_TH32x24_TmpOutput_format_addr[MLX_TH32x24_SCALERUP_BUFFER_NO];   // MLX_TH32x24 Temperature output
    INT8U   MLX_TH32x24_PPU_frame_ON; // 0: none , 1:keep sensor_frame ;

	// data read

	INT32U  MLX_TH32x24_display_frame;
	INT32U  MLX_TH32x24_PPU_frame;
    INT32U  MLX_TH32x24_ScalerUp_status;      // 0: read E. offset
	INT16U  MLX_TH32x24_sampleCnt;      // for MLX_TH32x24_start_timer_isr

	//INT16U  MLX_TH32x24_sampleHz;	//  5.7~ 732 Hz
	INT16U  MLX_TH32x24_sampleHz;	//  5.7~ 732 Hz

	INT8U  MLX_TH32x24_ReadElecOffset_TA_startON;
	INT8U  MLX_TH32x24_readout_block_startON;

	INT32U  MLX_TH32x24_BadPixAdr_buf;
	INT32U  MLX_TH32x24_BadPixMask_buf;

	INT8U   MLX_TH32x24_move_dect;	// 0 - 2
	//INT8U   mlx_TH32x24_MaxInd_fun;	// 高/低指示 
	INT8U   MLX_TH32x24_OVR_RoomTemp;
	INT8U   MLX_TH32x24_NOISE_CUTOFF_OVR_RTemp;
	INT8U   MLX_TH32x24_CMOS_OFF;
	INT8U   MLX_TH32x24_TABLE_SCALER_FACTOR;

	float  MLX_TH32x24_ta;
	INT16U  MLX_TH32x24_ImgValAry[IMG_VAL_buf_len]; // over zero max/min , under zero max/min
	INT16S  MLX_TH32x24_ImgTempAry[IMG_VAL_buf_len]; // over zero max/min , under zero max/min
	INT16U  MLX_TH32x24_Tmin;
	
	INT32U  MLX_TH32x24_avg_buf_addr[AVG_buf_len];	// MLX_TH32x24 temperature buffer addr
	INT32U  MLX_TH32x24_display_background_frame;
	INT32U  MLX32x24_EE_READ_8bitBUF;
	INT32U  MLX32x24_EE_READ_16bitBUF;
	//INT32U  MLX32x24_READ_frameData_BUF; -> 直接以 frameData[834]
	INT16U	frameData[MLX90640_frameDataSize];

	float  MLX_TH32x24_vdd;

	INT16S	result[MLX_Pixel];
	float	result_image[MLX_Pixel];

	INT32U  MLX_TH32x24_ImgAvg_buf_addr[IMG_AVG_buf_len];	// MLX_TH32x24 image buffer addr
	INT32U  MLX_TH32x24_GrayOutputFrame_addr;
	INT32U 	MLX_TH32x24_GrayScaleUpFrame_addr;
	INT32U  MLX_TH32x24_ScaleUpFrame_addr;
} MLX_TH32x24Para_t;



/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

#define RETURN(x)			{nRet = x; goto Return;}

#define POST_MESSAGE(queue, message, ack_mbox, msc_time)\
{\
	{\
		INT32U send_msg;\
		osEvent result;\
		osStatus status;\
		result = osMessageGet(ack_mbox, 1);\
		send_msg = message;\
		status = osMessagePut(queue, (INT32U)&send_msg, msc_time);\
		if(status != osOK)\
		{\
			DBG_PRINT("PutMsg Fail!!!\r\n");\
			RETURN(STATUS_FAIL);\
		}\
		result = osMessageGet(ack_mbox, msc_time);\
		if((result.status != osEventMessage) || (result.value.v == ACK_FAIL))\
		{\
			DBG_PRINT("GetMsg ack Fail!!! message %x\r\n",message);\
			RETURN(STATUS_FAIL);\
		}\
	}\
}

#define OSQFlush(x)\
{\
    while(1) {\
        osEvent result;\
        result = osMessageGet(x, 1);\
        if(result.status != osEventMessage) {\
            break;\
        }\
    }\
}
/*
#define gp_malloc_align(x, n)		    pvPortMalloc(x)
#define gp_malloc(x)				    pvPortMalloc(x)
#define gp_iram_malloc_align(x, n)		pvPortMalloc(x)
#define gp_iram_malloc(x)				pvPortMalloc(x)
#define gp_free(x)			    		vPortFree((void *)x)
*/
/**************************************************************************
 *                              E X T E R N A L                           *
 **************************************************************************/
extern osMessageQId AVIEncodeApQ;
extern osMessageQId scaler_task_q;
extern osMessageQId scaler_frame_q;
extern osMessageQId display_frame_q;
extern osMessageQId cmos_frame_q;
extern osMessageQId vid_enc_task_q;
extern osMessageQId vid_enc_frame_q;
extern osMessageQId video_frame_q;
extern osMessageQId avi_aud_q;
extern osMessageQId jpeg_fifo_q;
#if VIDEO_TIMESTAMP
extern osMessageQId frame_ts_q;

extern volatile INT32U vid_global_tick;
#endif

extern osMessageQId MLX_TH32x24_task_q;
extern osMessageQId MLX_TH32x24_readout_task_q;
extern osMessageQId MLX_TH32x24_readout_buf_q;

extern osMessageQId MLX_TH32x24_SCALERUP_task_q;
extern osMessageQId MLX_TH32x24_SCALERUP_buf_q;


extern AviEncPara_t *pAviEncPara;
extern AviEncAudPara_t *pAviEncAudPara;
extern AviEncVidPara_t *pAviEncVidPara;
extern AviEncPacker_t *pAviEncPacker0, *pAviEncPacker1;
extern catpure_t *pCap;
extern volatile INT32S pscaler_exit_0, pscaler_exit_1;

extern MLX_TH32x24Para_t *pMLX_TH32x24_Para;	//2019.03.28 davis
extern paramsMLX90640_t *pMLX32x24_Para;	//2019.05.28 davis


// callback function
extern INT32S (*pfn_avi_encode_put_data)(void* workmem, unsigned long fourcc, long cbLen, const void *ptr, int nSamples, int ChunkFlag);
extern INT32U (*videnc_display)(INT16U w, INT16U h, INT32U frame_addr);
extern INT32U (*videnc_buferr_post)(INT32U frame_addr);

//avi encode state
extern INT32U avi_enc_packer_start(AviEncPacker_t *pAviEncPacker);
extern INT32U avi_enc_packer_stop(AviEncPacker_t *pAviEncPacker);
extern INT32S vid_enc_preview_start(void);
extern INT32S vid_enc_preview_stop(void);
extern INT32S avi_enc_start(void);
extern INT32S avi_enc_stop(void);
extern INT32S avi_enc_pause(void);
extern INT32S avi_enc_resume(void);
extern INT32S avi_enc_save_jpeg(void);
extern INT32S avi_enc_capture(INT16S fd, INT8U quality, INT16U width, INT16U height);
extern INT32S avi_enc_one_frame(void);
extern INT32S avi_enc_storage_full(void);
extern INT32S avi_packer_err_handle(INT32S ErrCode);
extern INT32S avi_encode_state_task_create(INT8U pori);
extern INT32S avi_encode_state_task_del(void);
extern INT32S avi_enc_cut_lastframe(void);

//scaler task
extern INT32S scaler_task_create(INT8U pori);
extern INT32S scaler_task_del(void);
extern INT32S scaler_task_start(void);
extern INT32S scaler_task_stop(void);

//video_encode task
extern INT32S video_encode_task_create(INT8U pori);
extern INT32S video_encode_task_del(void);
extern INT32S video_encode_task_start(void);
extern INT32S video_encode_task_stop(void);
extern INT32S video_encode_task_post_q(INT32U mode);
extern INT32S video_encode_task_empty_q(void);

//audio task
extern INT32S avi_adc_record_task_create(INT8U pori);
extern INT32S avi_adc_record_task_del(void);
extern INT32S avi_audio_record_start(void);
extern INT32S avi_audio_record_restart(void);
extern INT32S avi_audio_record_stop(void);
extern INT32S avi_audio_record_cut_lastframe(void);

//avi encode api
extern void   avi_encode_init(void);

extern void avi_encode_video_timer_start(void);
extern void avi_encode_video_timer_stop(void);

extern INT32S avi_encode_set_file_handle_and_caculate_free_size(AviEncPacker_t *pAviEncPacker, INT16S FileHandle);
extern INT16S avi_encode_close_file(AviEncPacker_t *pAviEncPacker);
extern void avi_encode_fail_handle_close_file(AviEncPacker_t *pAviEncPacker);
extern INT32S avi_encode_set_avi_header(AviEncPacker_t *pAviEncPacker);
extern void avi_encode_set_curworkmem(void *pAviEncPacker);
//status
extern void   avi_encode_set_status(INT32U bit);
extern void   avi_encode_clear_status(INT32U bit);
extern INT32S avi_encode_get_status(void);
//memory
extern INT32U scaler_disp_post_empty(INT32U buf);
extern INT32U avi_encode_post_empty(osMessageQId event, INT32U frame_addr);
extern INT32U avi_encode_get_empty(osMessageQId event);
extern INT32U avi_encode_get_csi_frame(void);
extern INT32S avi_encode_memory_alloc(void);
extern INT32S avi_packer_memory_alloc(void);
extern void avi_encode_memory_free(void);
extern void avi_packer_memory_free(void);

//format
extern void avi_encode_set_sensor_format(INT32U format);
extern void avi_encode_set_display_format(INT32U format);
//other
extern void avi_encode_set_display_scaler(void);
extern INT32S avi_encode_set_jpeg_quality(INT8U quality_value);
extern BOOLEAN avi_encode_disk_size_is_enough(INT32S cb_write_size);

// isr handle
extern void csi_eof_isr(INT32U event);
extern void csi_capture_isr(INT32U event);
extern void csi_fifo_isr(INT32U event);
extern void cdsp_eof_isr(void);
extern void cdsp_capture_isr(void);
extern void cdsp_fifo_isr(void);
extern void pscaler_eof_isr(INT32U fifoMsg, INT32U fifoAddrs);

extern INT32S jpeg_encode_once(JpegPara_t *pJpegEnc);

// jpeg fifo encode
extern INT32S jpeg_encode_fifo_start(JpegPara_t *pJpegEnc);
extern INT32S jpeg_encode_fifo_once(JpegPara_t *pJpegEnc);
extern INT32S jpeg_encode_fifo_stop(JpegPara_t *pJpegEnc);

extern INT32S save_jpeg_file(INT16S fd, INT32U encode_frame, INT32U encode_size);
extern INT32S avi_audio_memory_allocate(INT32U	cblen);
extern void avi_audio_memory_free(void);
extern INT32U avi_audio_get_next_buffer(void);

extern void avi_adc_hw_start(INT32U sample_rate);
extern void avi_adc_hw_stop(void);


#if	AVI_ENCODE_SHOW_TIME == 1
void cpu_draw_osd(const INT8U *source_addr, INT32U target_addr, INT16U offset, INT16U res);
void cpu_draw_advalue_osd(INT32S value, INT32U target_buffer, INT16U resolution,INT8U st_val, INT8U shift_val, INT8U spec_val);

//void cpu_draw_time_osd(TIME_T current_time, INT32U target_buffer, INT16U resolution);

//Time and data reminder
typedef struct
{
    INT32S tm_sec;  /* 0-59 */
    INT32S tm_min;  /* 0-59 */
    INT32S tm_hour; /* 0-23 */
    INT32S tm_mday; /* 1-31 */
    INT32S tm_mon;  /* 1-12 */
    INT32S tm_year;
    INT32S tm_wday; /* 0-6 Sunday,Monday,Tuesday,Wednesday,Thursday,Friday,Saturday */
}TIME_T;


#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
// MLX_TH32x24 I2C calc data task
///////////////////////////////////////////////////////////////////////////////////////////////////////

extern INT32S MLX_TH32x24_task_create(INT8U pori);
//extern INT32S MLX_TH32x24_task_del(void);
extern INT32S MLX_TH32x24_task_start(void);
extern INT32S MLX_TH32x24_task_stop(void);
extern void   TH80x64_start_timer_isr(void);


INT32S h264_encode_start(INT32S width, INT32S height);
INT32S h264_encode_stop(void);
INT32S h264_encode_frame(INT32U frameAddr, INT32U outAddr, INT32U PictureType, INT32S insertHdr, INT32U NullFrame);
#endif // __AVI_ENCODER_APP_H__
