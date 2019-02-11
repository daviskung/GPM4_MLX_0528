#ifndef __VIDEO_ENCODER_H__
#define __VIDEO_ENCODER_H__

#include "application.h"

extern void video_encode_entrance(void);
extern void video_encode_exit(void);
extern INT32S video_stream_encode_start(void);
extern INT32S video_stream_encode_stop(void);
extern void video_encode_capture_disable_difference_size(void);
extern void video_encode_capture_enable_difference_size(void);

extern CODEC_START_STATUS video_encode_preview_start(VIDEO_ARGUMENT arg);
extern CODEC_START_STATUS video_encode_preview_stop(void);
//extern CODEC_START_STATUS video_encode_start(MEDIA_SOURCE src);
extern CODEC_START_STATUS video_encode_start(MEDIA_SOURCE src, INT32U write_file);
extern CODEC_START_STATUS video_encode_stop(void);
extern CODEC_START_STATUS video_encode_Info(VIDEO_INFO *info);
extern VIDEO_CODEC_STATUS video_encode_status(void);
extern CODEC_START_STATUS video_encode_set_zoom_scaler(INT32U zoom_ratio);
extern CODEC_START_STATUS video_encode_pause(void);
extern CODEC_START_STATUS video_encode_resume(void);
extern CODEC_START_STATUS video_encode_set_jpeg_quality(INT8U quality_value);
extern CODEC_START_STATUS video_encode_set_h264_rc(INT32U bitrate, INT32U gop_len);
extern CODEC_START_STATUS video_encode_capture_picture(MEDIA_SOURCE src);
extern CODEC_START_STATUS video_encode_capture_size(MEDIA_SOURCE src, INT8U quality, INT16U width, INT16U height);
extern CODEC_START_STATUS video_encode_fast_switch_stop_and_start(MEDIA_SOURCE src);
extern CODEC_START_STATUS video_encode_cut_lastframe(void);

extern void video_encode_register_display_callback(INT32U (*disp)(INT16U, INT16U, INT32U));
extern void video_encode_register_user_write_callback(INT32S (*put_data)(void* , unsigned long , long , const void *, int , int));
extern void video_encode_buffer_post_callback(INT32U (*disp_post)(INT32U));

extern INT32S video_encode_get_recording_time();
extern INT32S video_encode_get_dummy_frame_count();
#endif
