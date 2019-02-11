#ifndef __AUDIO_RECORD_H__
#define __AUDIO_RECORD_H__

#include "application.h"
#include "drv_l1_adc.h"
#include "drv_l1_dma.h"
#include "drv_l1_gpio.h"
#include "drv_l1_i2s_rx.h"

#include "gplib.h"
//=============================================================
// audio encode configure set
//=============================================================
#define ADC_LINE_IN_EN		1
#define BUILD_IN_MIC_EN		1
#define GPY0050_IN_EN		0
#define BUILD_LINEIN_EN		1

// input device
#define ADC_LINE_IN			(1 << 0)
#define BUILD_IN_MIC		(1 << 1)
#define GPY0050_IN			(1 << 2)
#define DOUBLE_LINEIN		(1 << 3)
#define BUILD_LINEIN		(1 << 4)

// for adc and mic use
#define C_ADC_USE_TIMER					ADC_AS_TIMER_C			//adc:ADC_AS_TIMER_C ~ F, mic: ADC_AS_TIMER_C ~ F
#define C_ADC_USE_CHANNEL				ADC_LINE_1				//adc channel: 0 ~ 3

// for GPY0050 use
#define C_AUDIO_RECORD_TIMER			TIMER_B		 			//timer, A,B,C
#define C_GPY0050_SPI_CS_PIN			IO_F5					//gpy0500 spi interface cs pin


#define C_A1800_RECORD_BUFFER_SIZE		A18_ENC_FRAMESIZE		//PCM buffer size, fix 320

#if MCU_VERSION == GPM41XXA
#define C_AUD_PCM_BUFFER_NO				2						//pcm buffer number
#define C_WAVE_RECORD_BUFFER_SIZE		1024					//PCM buffer size, depend on SR=16KHz
#define C_BS_BUFFER_SIZE				1024*4					//file buffer size, fix 4Kbyte
#define A1800_TIMES						3						//a1800 encode times
#define ADPCM_TIMES						4						//adpcm encode times, 500*16, 256*16, depend on SR=16KHz
#else
#define C_AUD_PCM_BUFFER_NO				3						//pcm buffer number
#define C_WAVE_RECORD_BUFFER_SIZE		1024*16					//PCM buffer size, depend on SR=16KHz
#define C_BS_BUFFER_SIZE				1024*256					//file buffer size, fix 64Kbyte
#define A1800_TIMES						30						//a1800 encode times, 320*30, <80*30, depend on SR=16KHz
#define ADPCM_TIMES						16						//adpcm encode times, 500*16, 256*16, depend on SR=16KHz
#endif
#define C_VR_RECORD_BUFFER_SIZE			512*2
#define C_AUDIO_STREAM_BUFFER_SIZE		800




#define MP3_TIME						15						//MP3 encode times, 1152*15

// pcm energy detect threshold
#define PCM_GLOBAL_THR 					0x800					//Frame Energy Threshold to judge as active
#define PCM_LOCAL_THR  					0x800 					//Local Energy Threshold of each sample

// adc record and dac play at same time
#define RECORD_WITH_DAC_OUT_EN			0						//1: enable, 0: disable

//=============================================================
// audio encode status
//=============================================================
#define C_GP_FS				0
#define C_USER_DEFINE		1

#define C_MONO_RECORD		1
#define C_STEREO_RECORD 	2

#define C_ENVDET_NO_DETECT	    0x00
#define C_ENVDET_DETECTING	    0x01
#define C_ENVDET_DETECTED	    0x02
#define C_ENVDET_DETECT_END	    0x03
#define C_ENVDET_DETECT_PAUSE	0x04
#define C_ENVDET_DETECT_RESUME	0x05

#define C_STOP_RECORD		0x00000000
#define C_STOP_RECORDING	0x00000001
#define C_START_RECORD		0x00000002
#define C_START_FAIL		0x80000001

#define AUD_WRITE_ACK_OK                0x00000001
#define AUD_WRITE_ACK_FAIL              0x00000002

#define AUD_RECORD_STATUS_OK			0x00000000
#define AUD_RECORD_STATUS_ERR			0x80000001
#define AUD_RECORD_INIT_ERR				0x80000002
#define AUD_RECORD_DMA_ERR				0x80000003
#define AUD_RECORD_RUN_ERR				0x80000004
#define AUD_RECORD_FILE_WRITE_ERR		0x80000005
#define AUD_RECORD_MEMORY_ALLOC_ERR		0x80000006

//gpy0500 command
#define C_CMD_RESET_IN1				0x83
#define C_CMD_RESET_IN4				0x89
#define C_CMD_ENABLE_ADC			0x98
#define C_CMD_ENABLE_MIC_AGC_ADC	0x9B
#define C_CMD_ENABLE_MIC_ADC		0x9B
#define C_CMD_ENABLE_MIC_AGC		0x93
#define C_CMD_DUMMY_COM				0xC0
#define C_CMD_ADC_IN1				0x82
#define C_CMD_ADC_IN4				0x88
#define C_CMD_ZERO_COM				0x00
#define C_CMD_POWER_DOWN			0x90
#define C_CMD_TEST_MODE				0xF0

typedef enum
{
	MSG_ADC_DMA_DONE = C_DMA_STATUS_DONE,
	MSG_AUDIO_ENCODE_START = 0x10000000,
    MSG_AUDIO_ENVELOP_DET_START,
	MSG_AUDIO_ENCODE_STOPING,
	MSG_AUDIO_ENCODE_STOP,
	MSG_AUDIO_ENCODE_WRITE,
	MSG_AUDIO_ENCODE_WRITE_STOP,
	MSG_AUDIO_ENCODE_ERR,
	MSG_AUDIO_ENCODE_EXIT
} AUDIO_RECORD_ENUM;

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
} AUD_ENC_WAVE_HEADER;

typedef struct
{
	INT32U  Status;
	INT32U  SourceType;
	INT16S  FileHandle;
	INT8U	InputDevice;		//signal source
	INT8U	Channel;			//1,mono or 2,stereo
	INT8U   CutLastFrame;
	INT32U  AudioFormat;
	INT32U  SampleRate;			//sample rate
	INT32U  BitRate;            //bite rate
	INT32U  FileLenth;			//byte
	INT32U  NumSamples;			//sample

#if	APP_DOWN_SAMPLE_EN
	INT8U   bEnableDownSample;
	INT8U 	*DownSampleWorkMem;
	INT8U   DownsampleFactor;
#endif

	INT8U 	*EncodeWorkMem;		//wrok memory
	INT8U   *Bit_Stream_Buffer;	//encode bit stream buffer
	INT32U  read_index;			//bit stream buffer index
    INT32U  write_index;		//bit stream buffer index
	INT32U	PackSize;			//for file write size, byte
	INT8U   *pack_buffer;		//a1800 and wav lib use
	INT32U	PCMInFrameSize;		//pcm input buffer, short
	INT32U  OnePCMFrameSize; 	//short
	INT16U	MsgTx, MsgRx;
	//allow record size
	INT64U  disk_free_size;

	INT32U  ring_buffer;
	INT32U  aud_pcm_buffer[C_AUD_PCM_BUFFER_NO];
	INT32U  mic_pcm_buffer[C_AUD_PCM_BUFFER_NO];
	DMA_STRUCT adc_dma_dbf;
	DMA_STRUCT mic_dma_dbf;

#if GPY0050_IN_EN == 1
	INT16U  buffer_index;
	INT16U  pre_value;
	INT32U  buffer_cnt;
	INT32U  ready_buffer;
#endif
} Audio_Encode_Para;

//task api
INT32S adc_record_task_create(INT8U priority);
INT32S adc_record_task_del(INT8U priority);
INT32S adc_record_task_start(void);
INT32S adc_record_task_stop(void);
INT8U audio_record_set_CutLastFrame(INT8U flag);
INT32S adc_envelop_detect_start(void);

//api
void audio_record_set_status(INT32U status);
INT32U audio_record_get_status(void);
void audio_record_set_source_type(INT32U type);
INT32U audio_record_get_source_type(void);
INT64U audio_record_set_file_handle_free_size(INT16S file_handle);
void audio_record_set_info(INT32U audio_format, INT32U sample_rate, INT32U bit_rate);
void audio_record_set_channel(INT8U device, INT8U channel);
INT32S audio_record_set_down_sample(BOOLEAN b_down_sample, INT8U ratio);
INT32S pcm_energy_detect(INT16S* buffer_addr, INT32U pcm_size);
void mic_linein_bias_enable(void);
void mic_linein_bias_disable(void);
#endif
