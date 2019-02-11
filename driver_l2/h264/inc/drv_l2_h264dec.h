#ifndef __drv_l2_H264DEC_H__
#define __drv_l2_H264DEC_H__

#include "drv_l2.h"

#define H264Dec_OUT_PSCALER     0
#define H264Dec_OUT_PPU         1

typedef struct {
  //INT32U RawFrameYAddr;
  //INT32U RawFrameUVAddr;
  INT32U BitstreamAddr;
  INT32U BitstreamBaseAddr;
  INT32U BitstreamMaxAddr;
} gp_h264dec_in;

typedef struct {
    INT32U width;
    INT32U height;
    INT32U out_div;
    INT32U HDMI;
} gp_h264dec_cfg;

void* gp_h264dec_init(gp_h264dec_cfg* cfg);
int gp_h264dec_decode(void* handle, gp_h264dec_in* in);
int gp_h264dec_decode_wait(void* handle, INT32U* frame_type, INT32U time_out);
void gp_h264dec_release(void* handle);
#endif
