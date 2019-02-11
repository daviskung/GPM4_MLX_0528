#ifndef __CP_CAMERA_PROCESSOR_H__
#define __CP_CAMERA_PROCESSOR_H__

#include "cmsis_os.h"
#include "drv_l2_sensor.h"
#include "camera_processor.h"
#include "gp_vfm.h"
//#include "gp_mux.h"

extern volatile INT32U cp_global_tick;
extern INT32U cp_status;
extern INT32U cp_error;

typedef struct CP_SENSOR_INFO_s
{
	drv_l2_sensor_ops_t* sensor;
	INT32U width;
	INT32U height;
	INT32U interface;
	INT32U output_width;
	INT32U output_height;
	INT32U output_format;
	INT32U next_buf[2];
	INT32U *mem;
	INT8U enable;
	CAMERA_CALLBACK callback;

} CP_SENSOR_INFO;

#define MSG_SCALER_EXIT		0x10000000
#define MSG_ENC_TASK_EXIT	0x10000001
#define MSG_SCALE_TASK_EXIT 0x10000002
#define MSG_MUX_TASK_EXIT 	0x10000003
#define MSG_MUX_START 		0x10000004
#define MSG_MUX_STOP 		0x10000005
#define MSG_MUX_VID 		0x10000006
#define MSG_MUX_AUD 		0x10000007

typedef struct frame_info_s
{
	INT8U* data;
	INT8U* out;
	INT8U* enc_out;
	INT32U time;
	INT32U size;
	INT32U type;
	INT32U ref;
} frame_info_t;

typedef struct CP_ENCODER_INFO_s
{
	ENCCFG cfg;
	frame_info_t out_buffer[MAX_ENCODER_OUT_NUM];
	vframe_manager_t vfm;
	osMutexId mtx;
	INT32U header_size;
	INT8U encode_start;
} CP_ENCODER_INFO;

typedef struct PACKER_MSG_s
{
	INT32U msg;
	INT32S handle;
	void* data;
} packer_msg_t;

#define VARIABLE_FRAME_RATE	0

//cp_audio_record.c
extern INT32S cp_adc_record_task_create(void);
extern INT32S cp_adc_record_task_del(void);
#if AVI_AUDIO_ENCODE_EN == 1
extern INT32S cp_audio_record_start(void);
extern INT32S cp_audio_record_restart(void);
extern INT32S cp_audio_record_stop(void);
extern INT32S cp_audio_record_pause(void);
extern INT32U cp_get_audio_recording_time(void);
extern INT32U cp_get_audio_frame_duration(void);
#endif
//cp_encoder.c
extern INT32S cp_encode_init(ENCCFG* cfg);
extern void cp_encode_close(void);
extern INT32S cp_encode_task_create(void);
extern INT32S cp_encode_task_del(void);
extern INT32S cp_encode_out(INT32U in_addrs, INT32U time, INT32U scale_out_addrs);
extern INT32S cp_encode_muxer_stop(void);
extern void cp_frame_addref(frame_info_t* frame);
extern void cp_free_frame(frame_info_t* frame);
//cp_muxer.c
extern INT32S cp_muxer_task_create(void);
extern INT32S cp_muxer_task_del(void);
extern INT32S cp_muxer_start(INT16S File);
extern INT32S cp_muxer_stop(INT8U wait_stop);
extern inline INT32S cp_muxer_update_disk_free_size(INT32S size);
extern INT32S cp_muxer_video(frame_info_t* frame);
extern INT32S cp_muxer_audio(frame_info_t* frame);
extern INT32S cp_muxer_get_error(void);
extern INT32U cp_muxer_get_recording_time(void);
extern INT32U cp_muxer_get_estimate_frame_delay();
//cp_sensor.c
extern INT32S cp_sensor_init(void);
extern INT32S cp_sensor_getinfo(INT32U idx, CAMINFO* info);
extern INT32S cp_sensor_close(void);
extern INT32S cp_sensor_set(INT32U idx, CAMCFG* cfg);
extern INT32S cp_sensor_start(INT32U idx, INT32U bufA, INT32U bufB);
extern INT32S cp_sensor_stop(INT32U idx);
extern INT32S cp_sensor_set_next_frame(INT32U idx, INT32U addr);

#endif
