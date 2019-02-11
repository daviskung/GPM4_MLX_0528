#ifndef __drv_l2_H264ENC_H__
#define __drv_l2_H264ENC_H__

#include "drv_l2.h"

typedef struct {
  int width;
  int height;
  int level;
  int crop_left;
  int crop_right;
  int crop_top;
  int crop_bottom;
}gp_h264_cfg;

typedef struct {
  int qpHdr;
  int qpMin;
  int qpMax;
  int bitPerSeconds;
  int gopLen;
  //int fixedIntraQp;
  int frameRate;
  int ae_night;
} gp_h264enc_rate_ctrl;

typedef struct {
  INT32U RawFrameAddr;
  INT32U BitstreamOutAddr;
  int PictureType;
  int InsertHdr;
  int bitrate_level; //-5~5, 0:original bitrate
  int ae_night;
} gp_h264enc_in;

void h264_drv_init();
void h264_reset_wait_status();
INT32S h264_wait_finish(INT32U time_out);
void* gp_h264enc_init(gp_h264_cfg* cfg);
int gp_h264enc_encode(void* handle, gp_h264enc_in* in);
int gp_h264enc_encode_wait(void* handle, INT32U time_out);
int gp_h264enc_insert_skip(void* handle, gp_h264enc_in* in);
void gp_h264enc_release(void* handle);
void gp_h264enc_set_callback(void (*notify)(void* handle));
#endif
