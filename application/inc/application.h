#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include "drv_l1.h"
#include "project.h"
#include "gplib.h"
#include "gp_stdlib.h"
#include "gplib_mm_gplus.h"
#include "gplib_print_string.h"
#include "msg_manager.h"
#include "turnkey_drv_msg.h"
#include "turnkey_sys_msg.h"

/**************************************************************************
 *                          task poriority use	                          *
 **************************************************************************/
 //The higher the number, the higher the priority
#define SPU_PRIORITY			configMAX_PRIORITIES - 0
#define DAC_PRIORITY			configMAX_PRIORITIES - 1
#define I2S_PRIORITY			configMAX_PRIORITIES - 2
#define MIDI_PRIORITY			configMAX_PRIORITIES - 3
#define AUD_DEC_PRIORITY0		configMAX_PRIORITIES - 4
#define AUD_DEC_PRIORITY1		configMAX_PRIORITIES - 5
#define AUD_DEC_PRIORITY2		configMAX_PRIORITIES - 6
#define MIDI_DEC_PRIORITY		configMAX_PRIORITIES - 7
#define FS_PRIORITY				configMAX_PRIORITIES - 8
#define USBDL2TASKPRIORITY		9
#define USBDAPPTASKPRIORITY		13
#define UVCTASKPRIORITY			11
#define USBDDETTASKPRIORITY		17
#define IMAGE_PRIORITY			10
#define AVI_DEC_PRIORITY  		12

//usb web cam
#define USB_AUD_ENC_PRIORITY	11
#define USB_AUDIO_PRIORITY      13
#define USB_CAM_PRIORITY        15
#define USB_CAM_GPL325XX        19

//video codec
#define JPEG_ENC_PRIORITY		14
#define AUD_ENC_PRIORITY		16
#define SCALER_PRIORITY			18
#define AVI_ENC_PRIORITY		20
#define AVI_PACKER0_PRIORITY	21
#define AVI_PACKER1_PRIORITY	22
#define AVI_PACKER_PRIORITY		23
#define CAPTURE_PRIORITY		24
#define AE_AWB_PRIORITY			25

#define VIDEO_PASER_PRIORITY	16
#define AUDIO_DECODE_PRIORITY   18
#define VIDEO_DECODE_PRIORITY	20
#define VID_DEC_STATE_PRIORITY	22
#define CONSOLE_TASK_PRIORITY	26
#define MAIN_TASK_PRIORITY		32

#define TH32x32_TASK_PRIORITY	33
#define TH32x32_SCALARUP_PRIORITY	34


#define AP_QUEUE_MSG_MAX_LEN    60

/*== Turn Key File Server Task ==*/
#define C_FILE_SERVICE_FLUSH_DONE		0xFF010263

#ifndef DEBUG_MSG
	#define DEBUG_MSG		DBG_PRINT
#else
	#define DEBUG_MSG(...)
#endif

typedef struct {
	INT32U  src_id;
	INT8U   src_name[12];
	INT32U  sec_offset;
	INT32U  sec_cnt;
} TK_FILE_SERVICE_SPI_STRUCT, *P_TK_FILE_SERVICE_SPI_STRUCT;

typedef struct {
	INT32S fd;
	INT32U buf_addr;
	INT32U buf_size;
	TK_FILE_SERVICE_SPI_STRUCT spi_para;
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OS_EVENT *result_queue;
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueHandle result_queue;
#endif
	INT32U data_offset;
	INT32U main_channel;
	INT8U  *data_start_addr;
} TK_FILE_SERVICE_STRUCT, *P_TK_FILE_SERVICE_STRUCT;

typedef struct
{
	INT32S disk;
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OS_EVENT *result_queue;
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueHandle result_queue;
#endif
} STMountPara;

typedef struct
{
	struct STFileNodeInfo *pstFNodeInfo;
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OS_EVENT *result_queue;
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    xQueueHandle result_queue;
#endif
} STScanFilePara;

typedef struct
{
	INT32U	src_storage_id;
	INT32U	file_type;
	INT32U	file_idx;
	INT32U	dest_storage_id;
	INT8S	dest_name[32];
	INT32S	status;
} TK_FILE_COPY_STRUCT, *P_TK_FILE_COPY_STRUCT;

typedef struct
{
	f_pos	fpos;
	INT32U	file_idx;
	INT32S	status;
} TK_FILE_DELETE_STRUCT, *P_TK_FILE_DELETE_STRUCT;

typedef struct
{
	INT32U	*p_buffer;
	INT32U	size;
	INT8S	ext_name[4];
	INT32U	dest_storage_id;
	INT8S	dest_name[32];
	INT32S	status;
} TK_FILE_SAVE_STRUCT, *P_TK_FILE_SAVE_STRUCT;

/* audio */
#define SPU_LEFT_CH             0
#define SPU_RIGHT_CH            1
#if MCU_VERSION == GPM41XXA
#define MAX_DAC_BUFFERS			3
#else
#define MAX_DAC_BUFFERS			6
#endif
#define MAX_I2S_BUFFERS		    20
#define C_DMA_DBF_START   2
#define C_DMA_WIDX_CLEAR  3
#define AUDIO_FORMAT_JUDGE_AUTO  1

#define AUDIO_IDLE        0
#define AUDIO_PLAYING     1
#define AUDIO_PLAY_PAUSE  2
#define AUDIO_PLAY_STOP   3
typedef struct
{
	INT8U	*ring_buf;
	INT8S   *work_mem;
	INT16U  reading;
	INT16S  file_handle;
	INT32U  frame_size;
	INT32U  ring_size;
	INT32U  ri;
	INT32U  wi;
	INT32U  f_last;
	INT32U  file_len;
	INT32U  file_cnt;
	INT32U  try_cnt;
	INT32U  read_secs;
	INT32U	data_offset;
	INT8U	*data_start_addr;
} AUDIO_CTRL;
typedef struct
{
    INT32S curr_play_time;
    INT32S total_play_time;
} ST_AUDIO_PLAY_TIME;
typedef struct
{
    BOOLEAN   mute;
    INT8U     volume;
    INT16S    fd;
    INT32U    src_id;
    INT32U    src_type;
    INT32U    audio_format;
    INT32U    file_len;
    INT8U     Main_Channel;
    INT32U    i2s_channel;
} STAudioTaskPara;

typedef struct
{
    MSG_AUD_ENUM result_type;
    INT32S result;
} STAudioConfirm;

typedef enum
{
	AUDIO_SRC_TYPE_FS=0,
	AUDIO_SRC_TYPE_SDRAM,
	AUDIO_SRC_TYPE_SPI,
	AUDIO_SRC_TYPE_RS,			// added by Bruce, 2009/02/19
	AUDIO_SRC_TYPE_GPRS,
	AUDIO_SRC_TYPE_APP_RS,
	AUDIO_SRC_TYPE_APP_PACKED_RS,
	AUDIO_SRC_TYPE_USER_DEFINE,	// added by Bruce, 2008/10/27
	AUDIO_SRC_TYPE_FS_RESOURCE_IN_FILE, // added by Bruce, 2010/01/22
	AUDIO_SRC_TYPE_NAND,
	AUDIO_SRC_TYPE_FS_OGGMIX,
	AUDIO_SRC_TYPE_MAX
} AUDIO_SOURCE_TYPE;

typedef enum
{
	AUDIO_TYPE_NONE = -1,
	AUDIO_TYPE_MIDI,//080903
	AUDIO_TYPE_MP3,
	AUDIO_TYPE_WMA,
	AUDIO_TYPE_WAV,
	AUDIO_TYPE_A1800,//080722
	AUDIO_TYPE_S880,	// added by Bruce, 2008/09/25
	AUDIO_TYPE_AVI,
	AUDIO_TYPE_A1600,	// added by Bruce, 2008/09/23
	AUDIO_TYPE_A6400,	// added by Bruce, 2008/09/25
	AUDIO_TYPE_AVI_MP3,
	AUDIO_TYPE_AAC,
	AUDIO_TYPE_OGG,
	AUDIO_STREAM_PCM,
	AUDIO_TYPE_MAX
} AUDIO_TYPE;

typedef enum
{
	AUDIO_CHANNEL_SPU = 0,
	AUDIO_CHANNEL_DAC,
	AUDIO_CHANNEL_I2S,
	AUDIO_CHANNEL_MIDI,
	AUDIO_CHANNEL_MAX
} AUDIO_CHANNEL_TYPE;

typedef enum
{
	AUDIO_ERR_NONE,
	AUDIO_ERR_FAILED,
	AUDIO_ERR_INVALID_FORMAT,
	AUDIO_ERR_OPEN_FILE_FAIL,
	AUDIO_ERR_GET_FILE_FAIL,
	AUDIO_ERR_DEC_FINISH,
	AUDIO_ERR_DEC_FAIL,
	AUDIO_ERR_READ_FAIL,
	AUDIO_ERR_WRITE_FAIL,
	AUDIO_ERR_AVI_READ_FAIL,
	AUDIO_ERR_MEM_ALLOC_FAIL
} AUDIO_STATUS;

//========================= Media Format Defintion ============================
// including Audio, Image, Video
//=============================================================================
typedef enum
{
    AUD_AUTOFORMAT=0,
    MIDI,
    WMA,
    MP3,
    WAV,
    A1800,
    S880,
    A6400,
    A1600,
    IMA_ADPCM,
    MICROSOFT_ADPCM,
    AAC,
    OGG,
    ALAW,
    MULAW,
    VR_PCM
} AUDIO_FORMAT;

typedef enum
{
    IMG_AUTOFORMAT=0,
    JPEG,
    JPEG_P,                         // JPEG Progressive
    MJPEG_S,                        // Single JPEG from M-JPEG video
    BMP
} IMAGE_FORMAT;

typedef enum
{
    VID_AUTOFORMAT=0,
    MJPEG,
    MPEG4,
    H_264
} VIDEO_FORMAT;
typedef struct
{
    AUDIO_FORMAT Format;
    CHAR   *SubFormat;

    INT32U DataRate;                // unit: bit-per-second
    INT32U SampleRate;              // unit: Hz
    INT32U Channel;                 // if 1, Mono, if 2 Stereo
    INT32U Duration;                // unit: second
    INT32U FrameSize;               // unit: sample per single frame

    INT32U FileSize;                // unit: byte
    CHAR   *FileDate;               // string pointer
    CHAR   *FileName;               // file name
} AUDIO_INFO;

/*
 *                           Image Codec	                              *
 **************************************************************************/
#define IMAGE_CMD_STATE_MASK					0xFC000000
#define IMAGE_CMD_STATE_SHIFT_BITS				26

typedef enum {
	TK_IMAGE_SOURCE_TYPE_FILE = 0x0,
	TK_IMAGE_SOURCE_TYPE_BUFFER,
	TK_IMAGE_SOURCE_TYPE_MAX
} TK_IMAGE_SOURCE_TYPE_ENUM;

typedef enum {
	TK_IMAGE_TYPE_JPEG = 0x1,
	TK_IMAGE_TYPE_PROGRESSIVE_JPEG,
	TK_IMAGE_TYPE_MOTION_JPEG,
	TK_IMAGE_TYPE_BMP,
	TK_IMAGE_TYPE_GIF,
	TK_IMAGE_TYPE_PNG,
	TK_IMAGE_TYPE_GPZP,
	TK_IMAGE_TYPE_MAX
} TK_IMAGE_TYPE_ENUM;
typedef enum
{
    SOURCE_TYPE_FS=0,
    SOURCE_TYPE_SDRAM,
    SOURCE_TYPE_NVRAM,
    SOURCE_TYPE_USER_DEFINE,
    SOURCE_TYPE_FS_RESOURCE_IN_FILE,	// added by Bruce, 2010/01/22
    SOURCE_TYPE_NAND,
	SOURCE_TYPE_FS_OGGMIX,
    SOURCE_TYPE_MAX
} SOURCE_TYPE;
typedef struct
{
    SOURCE_TYPE type;                   //0: GP FAT16/32 File System by File SYSTEM
                                        //1: Directly from Memory Mapping (SDRAM)
                                        //2: Directly from Memory Mapping (NVRAM)
                                        //3: User Defined defined by call out function:audio_encoded_data_read
    struct User                         //Source File handle and memory address
    {
        INT32S FileHandle;              //File Number by File System or user Define
        INT32S temp;                    //Reserve for special use
        INT8U *memptr;                  //Memory start address
    } type_ID;

    union SourceFormat                  //Source File Format
    {
        AUDIO_FORMAT AudioFormat;       //if NULL,auto detect
        IMAGE_FORMAT ImageFormat;       //if NULL,auto detect
        VIDEO_FORMAT VideoFormat;       //if NULL,auto detect
    } Format;
} MEDIA_SOURCE;

//=============================================================================
//=============================================================================
typedef enum
{
		SND_OFFSET_TYPE_NONE = 0,
		SND_OFFSET_TYPE_HEAD,
		SND_OFFSET_TYPE_SIZE,
		SND_OFFSET_TYPE_TIME,
		SND_OFFSET_TYPE_MAX
} SndOffset_TYPE;

typedef struct {
		INT32U		DataRate;				// unit: bit-per-second
		INT32U		SampleRate;				// unit: Hz

		INT8U		Channel;				// if 1, Mono, if 2 Stereo
		INT8U		ConstPitchEn;			// constant pitch enable
		INT8U		EchoEn;					// echo efffect enable
		INT8U		VoiceChangerEn;			// voice changer enable

		INT32U		Pitch_idx;				// constant pitch index 0 ~ 7
		INT32U      Vox_thr;                // constant pitch threshold
		INT32U 		weight_idx;				// echo weight, 0 ~ 2
		INT32U		delay_len;				// echo delay

		INT16U		Speed;					// voice changer speed, 0 ~ 24
		INT16U		Pitch;					// voice changer pitch, 0 ~ 24

		INT32U		Offset;					// play offset
		SndOffset_TYPE Offsettype;

		AUDIO_FORMAT AudioFormat;

		SOURCE_TYPE Srctype;				//0: GP FAT16/32 File System by File SYSTEM
											//1: Directly from Memory Mapping (SDRAM)
											//2: Directly from Memory Mapping (NVRAM)
											//3: User Defined defined by call out function:audio_decode_data_fetch
} SND_INFO;

typedef struct {
		INT32U		Offset;					// play offset
		SndOffset_TYPE Offsettype;

		AUDIO_FORMAT AudioFormat;

} SACM_CTRL;

typedef struct {
	INT32U cmd_id;
	INT32S image_source;          	// File handle/resource handle/pointer
	INT32U source_size;             // File size or buffer size
	INT8U source_type;              // TK_IMAGE_SOURCE_TYPE_FILE/TK_IMAGE_SOURCE_TYPE_BUFFER

	INT8S parse_status;				// 0=ok, others=fail
	INT8U image_type;				// TK_IMAGE_TYPE_ENUM
	INT8U orientation;
	INT8U date_time_ptr[20];		// 20 bytes including NULL terminator. Format="YYYY:MM:DD HH:MM:SS"
	INT16U width;
	INT16U height;
} IMAGE_HEADER_PARSE_STRUCT;

typedef struct {
	INT32U cmd_id;
	INT8S status;
	INT8U special_effect;
	INT8U scaler_input_format;
	INT8U scaler_output_format;

	INT16U input_width;
	INT16U input_height;
	INT16U input_visible_width;
	INT16U input_visible_height;
	INT32U input_offset_x;
	INT32U input_offset_y;
	void *input_buf1;
//	void *input_buf2;					// This parameter is valid only when separate YUV input format is used
//	void *input_buf3;					// This parameter is valid only when separate YUV input format is used

	INT16U output_buffer_width;
	INT16U output_buffer_height;
	INT32U output_width_factor;			// factor = (input_size<<16)/output_size
	INT32U output_height_factor;
	INT32U out_of_boundary_color;
	void *output_buf1;
//	void *output_buf2;					// This parameter is valid only when separate YUV output format is used
//	void *output_buf3;					// This parameter is valid only when separate YUV output format is used
} IMAGE_SCALE_STRUCT, *P_IMAGE_SCALE_STRUCT;

typedef struct {
	INT32U cmd_id;
	INT32S image_source;          	// File handle/resource handle/pointer
	INT32U source_size;             // File size or buffer size
	INT8U source_type;              // 0=File System, 1=SDRAM, 2=NVRAM
	INT8S decode_status;            // 0=ok, others=fail
	INT8U reserved;
	INT8U output_ratio;             // 0=Fit to output_buffer_width and output_buffer_height, 1=Maintain ratio and fit to output_buffer_width or output_buffer_height, 2=Same as 1 but without scale up
	INT32U output_format;
	INT16U output_buffer_width;
	INT16U output_buffer_height;
	INT16U output_image_width;
	INT16U output_image_height;
	INT32U out_of_boundary_color;
	INT32U output_buffer_pointer;
} IMAGE_DECODE_STRUCT, *P_IMAGE_DECODE_STRUCT;

typedef struct {
	INT8U	EncodeType;
	INT8U 	QuantizationQuality;		// only 50/70/80/85/90/93/95/97/100.
	INT8U 	InputFormat;				// only C_JPEG_FORMAT_YUYV.
	INT8U 	OutputFormat;				// only C_JPG_CTRL_YUV422.
	INT8S	EncodeState;				// 0 is ok, -N is fail.
	INT16U 	InputWidth;					// must width/16=0.
	INT16U 	InputHeight;				// must height/8=0.
	INT32U 	OutputBuf;					// must align 16-byte, size=encode_size+header_size.
	INT32U 	InputBuf_Y;					// must align 16-byte, size=encode_size+header_size.
	INT32U 	InputBuf_U;					// normal is null.
	INT32U 	InputBuf_V;					// normal is null.
	INT32U	EncodeSize;					// unit is byte.
} IMAGE_ENCODE_STRUCT;

typedef struct {
	IMAGE_DECODE_STRUCT basic;
	INT16U clipping_start_x;
	INT16U clipping_start_y;
	INT16U clipping_width;
	INT16U clipping_height;
} IMAGE_DECODE_EXT_STRUCT, *P_IMAGE_DECODE_EXT_STRUCT;

/**************************************************************************
 *                           MutilMedia Codec                             *
 **************************************************************************/

typedef struct {
		IMAGE_FORMAT	Format;
		char*		SubFormat;

		INT32U		Width;					// unit: pixel
		INT32U		Height;					// unit: pixel
		INT8U		Color;					// unit: color number

		INT32U		FileSize;				// unit: byte
		char		*FileDate;				// string pointer
		char		*FileName;				// file name pointer

		INT8U		Orientation;			// Portrait or Landscape
		char		*PhotoDate;				// string pointer
		INT8U		ExposureTime;			// Exporsure Time
		INT8U		FNumber;				// F Number
		INT16U		ISOSpeed;				// ISO Speed Ratings
} IMAGE_INFO;

typedef struct {
		AUDIO_FORMAT	AudFormat;
		char		AudSubFormat[6];

		INT32U		AudBitRate;				// unit: bit-per-second
		INT32U		AudSampleRate;			// unit: Hz
		INT8U		AudChannel;				// if 1, Mono, if 2 Stereo
		INT16U		AudFrameSize;			// unit: sample per single frame

		VIDEO_FORMAT	VidFormat;
		char		VidSubFormat[4];
		INT8U		VidFrameRate;			// unit: FPS
		INT32U		Width;					// unit: pixel
		INT32U		Height;					// unit: pixel

		INT32U		TotalDuration;			// unit: second
} VIDEO_INFO;

//======================== Media Argument Defintion =======================
typedef enum
{
		FIT_OUTPUT_SIZE=0,		//Fit to output_buffer_width and output_buffer_height
		MAINTAIN_IMAGE_RATIO_1,	//Maintain ratio and fit to output_buffer_width or output_buffer_height
		MAINTAIN_IMAGE_RATIO_2	//Same as MAINTAIN_IMAGE_RATIO_1 but without scale up
}SCALER_OUTPUT_RATIO;

typedef enum
{
		SCALER_INPUT_FORMAT_RGB565 = 0,
		SCALER_INPUT_FORMAT_RGBG,
		SCALER_INPUT_FORMAT_GRGB,
		SCALER_INPUT_FORMAT_YUYV,
     	SCALER_INPUT_FORMAT_UYVY
} IMAGE_INPUT_FORMAT;

typedef enum
{
		IMAGE_OUTPUT_FORMAT_RGB565 = 0,
		IMAGE_OUTPUT_FORMAT_RGBG,
		IMAGE_OUTPUT_FORMAT_GRGB,
		IMAGE_OUTPUT_FORMAT_YUYV,
		IMAGE_OUTPUT_FORMAT_YVYU,
		IMAGE_OUTPUT_FORMAT_UYVY,
		IMAGE_OUTPUT_FORMAT_VYUY
} IMAGE_OUTPUT_FORMAT;

typedef struct {
		INT8U		Main_Channel;			//0: SPU Channels
											//1: DAC Channel A+B (can't assign MIDI)
											//   only 0 and 1 are available now
		INT8U		L_R_Channel;			//0: Invalid Setting
											//1: Left Channel only
											//2: Right Channel only
											//3: Left + Right Channel
											//   only Left + Right Channel are avialable now
	    BOOLEAN   	mute;
	    INT8U     	volume;
	    INT8U		midi_index;				// MIDI index in *.idi

		INT8U		*data_start_addr;		// *data_start_addr is used only in one file mode
											// 1: SOURCE_TYPE_USER_DEFINE

	    INT32U		data_len;				// data_len is used only in two file mode
	    									// 1: SOURCE_TYPE_USER_DEFINE
	    									// 2: SOURCE_TYPE_FS_RESOURCE_IN_FILE
	    INT32U		data_offset;
        INT8U       i2s_channel;            //0~3
} AUDIO_ARGUMENT;

typedef struct {
		INT8U       *OutputBufPtr;
		INT16U		OutputBufWidth;
		INT16U		OutputBufHeight;
		INT16U		OutputWidth;
		INT16U		OutputHeight;
		INT32U      OutBoundaryColor;
		SCALER_OUTPUT_RATIO ScalerOutputRatio;
		IMAGE_OUTPUT_FORMAT	OutputFormat;
} IMAGE_ARGUMENT;

typedef struct
{
		INT8U *yaddr;
		INT8U *uaddr;
		INT8U *vaddr;
} IMAGE_ENCODE_PTR;

typedef enum
{
		IMAGE_ENCODE_INPUT_FORMAT_YUYV=0,
        IMAGE_ENCODE_INPUT_FORMAT_YUV_SEPARATE,
        IMAGE_ENCODE_INPUT_FORMAT_YUYV_SCALE
} IMAGE_ENCODE_INPUT_FORMAT;

typedef enum
{
		IMAGE_ENCODE_OUTPUT_FORMAT_YUV422=0,
		IMAGE_ENCODE_OUTPUT_FORMAT_YUV420
} IMAGE_ENCODE_OUTPUT_FORMAT;

typedef enum
{
		IMAGE_ENCODE_ONCE_READ=0,
		IMAGE_ENCODE_BLOCK_READ,
		IMAGE_ENCODE_BLOCK_READ_WRITE
} IMAGE_ENCODE_MODE;

typedef struct {
        INT8U        *InputBufPtr;
        INT8U        *OutputBufPtr;
		INT16U		 InputWidth;
		INT16U		 InputHeight;
	    INT16U       InputVisibleWidth;
	    INT16U       InputVisibleHeight;
	    INT32U       OutputWidthFactor;
	    INT32U       OutputHeightFactor;
		INT16U		 OutputWidth;
		INT16U		 OutputHeight;
	    INT32U       InputOffsetX;
	    INT32U       InputOffsetY;
		INT16U		 OutputBufWidth;
		INT16U		 OutputBufHeight;
		INT32U       OutBoundaryColor;
		IMAGE_INPUT_FORMAT	InputFormat;
		IMAGE_OUTPUT_FORMAT	OutputFormat;
} IMAGE_SCALE_ARGUMENT;

typedef struct {
        INT8U        *OutputBufPtr;
        INT8U        *QuantLuminanceTable;
        INT8U        *QuantChrominanceTable;
        INT8U        BlockLength;             //BlockLength=16,32..
        INT8U        UseDisk;
		INT16U		 InputWidth;
		INT16U		 InputHeight;
		INT16S       FileHandle;
		INT32U       QuantizationQuality;    //quality=50,70,80,85,90,95,100
		IMAGE_ENCODE_MODE EncodeMode;        // 0=read once, 1=block read
		IMAGE_ENCODE_PTR InputBufPtr;
		IMAGE_ENCODE_INPUT_FORMAT	InputFormat;
		IMAGE_ENCODE_OUTPUT_FORMAT	OutputFormat;
		IMAGE_SCALE_ARGUMENT        *ScalerInfoPtr;
} IMAGE_ENCODE_ARGUMENT;

typedef struct {
		INT8U		bScaler;
		INT8U		bUseDefBuf;			//video decode use user define buffer or not
		INT8U		*AviDecodeBuf1;		//video decode user define buffer address
		INT8U		*AviDecodeBuf2;		//video decode user define buffer address
		INT8U       bUseEncodeDiffSize; //video/Jpeg encode use different size
		INT16U		JPEGTargetWidth;	//video JPEG encode use
		INT16U		JPEGTargetHeight;	//video JPEG encode use
		INT16U		TargetWidth;		//video encode use
		INT16U		TargetHeight;		//video encode use
		INT16U      SensorWidth;        //video encode use
		INT16U      SensorHeight;       //video encode use
		INT16U      DisplayWidth;
		INT16U      DisplayHeight;
		INT16U		DisplayBufferWidth;
		INT16U 		DisplayBufferHeight;
		INT32U      VidFrameRate;       //for avi encode only
		INT32U      AudSampleRate;      //for avi encode only
		IMAGE_OUTPUT_FORMAT	OutputFormat;
} VIDEO_ARGUMENT;

//======================== Media Status Defintion =========================
typedef enum
{
		START_OK=0,
		RESOURCE_NO_FOUND_ERROR,
		RESOURCE_READ_ERROR,
		RESOURCE_WRITE_ERROR,
		CHANNEL_ASSIGN_ERROR,
		REENTRY_ERROR,
		AUDIO_ALGORITHM_NO_FOUND_ERROR,
		CODEC_START_STATUS_ERROR_MAX
} CODEC_START_STATUS;

typedef enum
{
		AUDIO_CODEC_PROCESSING=0,					// Decoding or Encoding
		AUDIO_CODEC_PROCESS_END,					// Decoded or Encoded End
		AUDIO_CODEC_BREAK_OFF,						// Due to unexpended card-plug-in-out
		AUDIO_CODEC_PROCESS_PAUSED,
		AUDIO_CODEC_STATUS_MAX
} AUDIO_CODEC_STATUS;

typedef enum
{
		IMAGE_CODEC_DECODE_END=0,                // Decoded or Encoded End
		IMAGE_CODEC_DECODE_FAIL,
		IMAGE_CODEC_DECODING,					  // Decoding or Encoding
		IMAGE_CODEC_BREAK_OFF,					// Due to unexpended card-plug-in-out
		IMAGE_CODEC_STATUS_MAX
} IMAGE_CODEC_STATUS;

typedef enum
{
		VIDEO_CODEC_PROCESSING=0,					// Decoding or Encoding
		VIDEO_CODEC_PROCESS_PAUSE,
		VIDEO_CODEC_PROCESS_END,					// Decoded or Encoded End
		VIDEO_CODEC_BREAK_OFF,						// Due to unexpended card-plug-in-out
		VIDEO_CODEC_RESOURCE_NO_FOUND,
		VIDEO_CODEC_CHANNEL_ASSIGN_ERROR,
		VIDEO_CODEC_RESOURCE_READ_ERROR,
		VIDEO_CODEC_RESOURCE_WRITE_ERROR,
		VIDEO_CODEC_PASER_HEADER_FAIL,
		VIDEO_CODEC_STATUS_ERR,
		VIDEO_CODEC_STATUS_OK,
		VIDEO_CODEC_STATUS_MAX
} VIDEO_CODEC_STATUS;

extern SND_INFO G_snd_info;

extern void *hVC;		//for voice change
extern void *hUpSample;	//for upsample
extern void *hConstPitch;
extern void *hEcho;

#endif 		// __APPLICATION_H__

