#ifndef __CAMERA_PROCESSOR_H__
#define __CAMERA_PROCESSOR_H__

#include "typedef.h"

#define MAX_CAMERA_NUM 2
#define MAX_ENCODER_OUT_NUM 60

typedef void (*CAMERA_CALLBACK)(INT32U idx, INT32U addr, INT32U time);
typedef void (*ENCODER_CALLBACK)(INT32U addr, INT32U scale_out_addr);
typedef void (*MUXER_CALLBACK)(INT16S file);

typedef struct CAMCFG_s
{
	INT8U enable;
	INT32U input_width;
	INT32U input_height;
	INT32U output_width;
	INT32U output_height;
	INT32U output_format;
	CAMERA_CALLBACK callback;
} CAMCFG;

typedef struct CAMINFO_s
{
	CHAR* name;
	INT32U interface;
} CAMINFO;

typedef struct CAM_SYSTEM_INFO_s
{
	INT32U camera_num;
	CAMINFO camera_info[MAX_CAMERA_NUM];
}CAM_SYSTEM_INFO;

extern INT32S camera_system_init(CAM_SYSTEM_INFO* system_info);
extern void camera_system_close();
extern INT32S camera_init(INT32U cam_idx, CAMCFG* cfg);
extern INT32S camera_start(INT32U cam_idx, INT32U bufA, INT32U bufB);
extern INT32S camera_stop(INT32U cam_idx);
extern INT32S camera_set_next_frame(INT32U cam_idx, INT32U addr);

typedef struct ENCCFG_s
{
	INT32U vFormat;
	INT32U inFormat;
	INT32U buf_width;
	INT32U buf_height;
	INT32U enc_width;
	INT32U enc_height;
	INT32U quality;
	INT32U bitrate;
	INT32U fps;
	INT8U enable_audio;
	INT32U aFormat;
	INT32U sampleRate;
	INT8U scale_out;
	INT32U scale_out_width;
	INT32U scale_out_height;
	INT32U scale_out_format;
	ENCODER_CALLBACK callback;
} ENCCFG;

extern INT32S camera_encode_init(ENCCFG* cfg);
extern INT32S camera_encode_close();
extern INT32S camera_encode_out(INT32U addr, INT32U time, INT32U scale_out_addr);

typedef struct MUXCFG_s
{
	INT32U format;
	INT16S FileHandle;
} MUXCFG;

extern INT32S camera_video_muxer_start(INT16S File);
extern INT32S camera_video_muxer_stop();
extern INT32S camera_video_muxer_get_error();
extern INT32U camera_video_muxer_get_recording_time();
extern INT32U camera_video_muxer_get_dummy_frame_count();

extern INT32U camera_system_status();
extern INT32U camera_system_error();

#define CP_STATUS_ENCODE_START		0x00000001
#define CP_STATUS_MUX_START			0x00000002

#define CP_ERROR_NONE				0x00000000
#define CP_ERROR_ENCODE_ERROR		0x00000001
#define CP_ERROR_DISK_FULL			0x00000002
#define CP_ERROR_FILE_SIZE_LIMIT	0x00000004
#define CP_ERROR_MUX_ERROR			0x00000008
#endif
