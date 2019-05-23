#ifndef AVI_ENCODER_SCALER_JPEG_H_
#define AVI_ENCODER_SCALER_JPEG_H_

#include "drv_l1.h"
#include "drv_l1_timer.h"
#include "drv_l1_adc.h"
#if (PALM_DEMO_EN==1)||(PATTERN_DEMO_EN==1)||(WAFD_DEMO_EN==1)
//#define VFRAME_MANAGER
#else
#define VFRAME_MANAGER
#endif
#define VFM_BUF_SIZE 0x30000
#define VFM_TOTAL_SIZE 0x200000

//video encode mode
#define C_VIDEO_ENCODE_FRAME_MODE		0
#define C_VIDEO_ENCODE_FIFO_MODE		1
#define VIDEO_ENCODE_MODE				C_VIDEO_ENCODE_FRAME_MODE
#define JPEG_X1_5_SCALE					0
#if PATTERN_DEMO_EN == 1
#define SCALER_CROP_CENTER              0
#define CENTER_WIDTH                    960
#endif
//video
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
	#define AVI_ENCODE_DIGITAL_ZOOM_EN		1	//0: disable digital zoom in/out 1: enable digital zoom in/out
	#define AVI_ENCODE_PREVIEW_DISPLAY_EN	1	//0: disable display preview	 1: enable display preview
#elif VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	#define AVI_ENCODE_DIGITAL_ZOOM_EN		0	//fix to 0 when use fifo mode
	#define AVI_ENCODE_PREVIEW_DISPLAY_EN	1	//fix to 0 when use fifo mode
#endif
#define AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN	0
#define AVI_PACKER_LIB_EN				0		//0: disable, 1:enable avi packer
#define AVI_ENCODE_PRE_ENCODE_EN		0		//0: disable, 1:enable audio and video pre-encode
#define AVI_ENCODE_VIDEO_ENCODE_EN		1		//0: disable, 1:enable; video record enable
#define AVI_ENCODE_FAST_SWITCH_EN		0		//0: disable, 1:enable; video encode fast stop and start
#define AVI_ENCODE_VIDEO_TIMER			TIMER_C //timer, A,B,C
#define AVI_ENCODE_TIME_BASE			50		//timer frequency must >= frame rate
#define AVI_ENCODE_SHOW_TIME			0		//0: disable, 1:enable

#define VIDEO_PACKER_NONE               0
#define VIDEO_PACKER_MOV                1
#define VIDEO_PACKER_AVI                2
#define NEW_VIDEO_PACKER_LIB            VIDEO_PACKER_AVI

//buffer number
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
	#define AVI_ENCODE_CSI_BUFFER_NO		3	//sensor use buffer number
	#define AVI_ENCODE_SCALER_BUFFER_NO		6	//scaler use buffer number
	//#define TH32x32_SCALERUP_BUFFER_NO 		1	//scaler use buffer number
	
	#define AVI_ENCODE_CSI_FIFO_NO			34	//sensor fifo mode use buffer number
#elif VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	#define AVI_ENCODE_CSI_BUFFER_NO	3  	//fix to 2 when use fifo mode
	#define AVI_ENCODE_SCALER_BUFFER_NO		6	//scaler use buffer number
	#define AVI_ENCODE_CSI_FIFO_NO			30	//sensor fifo mode use buffer number
#endif
//#define AVI_ENCODE_SCALER_BUFFER_NO		6	//scaler use buffer number
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
	#define AVI_ENCODE_ROTATE_BUFFER_NO     3   //rotate use buffer number
	#define AVI_ENCODE_VIDEO_BUFFER_NO		8	//jpeg encode use buffer number
#else
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE || defined(VFRAME_MANAGER)
#define AVI_ENCODE_VIDEO_BUFFER_NO		60	//jpeg encode use buffer number
#else
#define AVI_ENCODE_VIDEO_BUFFER_NO		8//20	//jpeg encode use buffer number
#endif
#endif
#define AVI_ENCODE_DISPALY_BUFFER_NO    3   //display use buffer number

#define AVI_ENCODE_PCM_BUFFER_NO		3	//audio record pcm use buffer number

//audio format
#define AVI_AUDIO_ENCODE_EN				1	//0: disable, 1:enable audio encode
#define AVI_ENCODE_AUDIO_FORMAT			WAV //0: no audio, IMA_ADPCM, MICROSOFT_ADPCM

//audio encode buffer size
#define C_WAVE_ENCODE_TIMES				15	//audio encode times

//mic phone input source
#define C_ADC_LINE_IN					0	//use adc hardware
#define C_BUILDIN_MIC_IN				1	//use build-in mic
#define C_LINE_IN_LR					2	//use line in lr
#define MIC_INPUT_SRC					C_BUILDIN_MIC_IN//C_ADC_LINE_IN

#if AVI_AUDIO_ENCODE_EN == 1
#define AVI_AUDIO_RECORD_TIMER			ADC_AS_TIMER_F  //adc use timer, C ~ F
#define AVI_AUDIO_RECORD_ADC_CH			ADC_LINE_1		//adc use channel, 0 ~ 3
#else
#define AVI_AUDIO_RECORD_TIMER			0				//adc use timer, C ~ F
#define AVI_AUDIO_RECORD_ADC_CH			0				//adc use channel, 0 ~ 3
#endif

//avi file max size
#define AVI_ENCODE_CAL_DISK_SIZE_EN		0				//0: disable, 1: enable
#define AVI_FILE_MAX_RECORD_SIZE		2000000000		//2GB
#define C_MIN_DISK_FREE_SIZE			512*1024		//512K

//avi packer buffer
#define FileWriteBuffer_Size			16*32*1024		//avi pack buffer size, must multiple 32k
#define IndexBuffer_Size				64*1024			//fix to 64k

//video encode fifo mode, fifo line set
#define SENSOR_FIFO_8_LINE				(1<<3)
#define SENSOR_FIFO_16_LINE				(1<<4)
#define SENSOR_FIFO_32_LINE				(1<<5)
#define SENSOR_FIFO_LINE				SENSOR_FIFO_32_LINE

//video format
#define C_XVID_FORMAT					0x44495658
#define	C_MJPG_FORMAT					0x47504A4D
#define C_H264_FORMAT                   0x34363248

#if (PALM_DEMO_EN==1)||(PATTERN_DEMO_EN==1)||(WAFD_DEMO_EN==1)
#define AVI_ENCODE_VIDEO_FORMAT			C_MJPG_FORMAT
#else
#define AVI_ENCODE_VIDEO_FORMAT			C_H264_FORMAT //only support mjpeg
#endif

//pause/resume method
#define AUDIO_RESTART_EN				1

// function configure
#define AUDIO_SFX_HANDLE				0
#define APP_QRCODE_BARCODE_EN			0

#if NEW_VIDEO_PACKER_LIB == VIDEO_PACKER_MOV
#define VIDEO_TIMESTAMP					1
#else
#define VIDEO_TIMESTAMP					0
#endif

#define AVG_buf_len			4
#define TH32x32_ReadoutBlockBuf_max	1
#define TH32x32_SCALERUP_BUFFER_NO	1

#define PRE_READ_DIS		0
#define PRE_READ_ON			1
#define PRE_READ_OFF		2


//=====================================================================================================
// marco
//

#define TOBigEndian(y)	{y=(y >>8) | (y <<8);}



#endif //AVI_ENCODER_SCALER_JPEG_H_
