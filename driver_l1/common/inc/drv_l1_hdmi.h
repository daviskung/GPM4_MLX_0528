#ifndef __drv_l1_HDMI_H__
#define __drv_l1_HDMI_H__

#include "drv_l1.h"
/*
#ifdef __cplusplus
extern "C" {
#endif
*/
enum {
	HDMI_OK = 0,
	HDMI_FAIL = -1
};

enum {
	HDMI_VID_480P = 0,
	HDMI_VID_720P,
	HDMI_VID_1080P
};

enum {
	HDMI_AUD_32K = 0,
	HDMI_AUD_44K,
	HDMI_AUD_48K
};

extern INT32S  drvl1_hdmi_audio_ctrl(INT32U status);
extern INT32S  drvl1_hdmi_dac_mute(INT32U status);
extern INT32S drvl1_hdmi_init(INT32U vid_mode, INT32U aud_mode);
extern INT32S drvl1_hdmi_exit(void);
extern INT32S drvl1_hdmi_h264_scaling_enable(INT32U img_width, INT32U img_height, INT32U img_offset);
extern INT32S drvl1_hdmi_h264_scaling_disable(void);
extern void drv_l1_hdmi_audio_sample_rate_set(INT32U aud_mode);
/*
#ifdef __cplusplus
	}
#endif
*/
#endif 		/* __drv_l1_HDMI_H__ */

