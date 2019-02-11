#ifndef __DRV_L2_HDMI_H__
#define __DRV_L2_HDMI_H__

extern int hdmi_read_EDID(INT8U *p);
extern int hdmi_read_block(INT8U index, INT8U *p, INT16U length);
extern INT32S ddc_sccb_open(void);
#ifdef __cplusplus
extern "C" {
#endif

int drvl2_hdmi_init(unsigned int DISPLAY_MODE, unsigned int AUD_FREQ);


#ifdef __cplusplus
}
#endif

#endif		// __DRV_L2_HDMI_H__

