#ifndef __DRV_L1_ROTATOR_H__
#define __DRV_L1_ROTATOR_H__

#include "drv_l1.h"
#include "drv_l1_sfr.h"

// control register
#define C_RPTATOR_START			                0x00000001
#define C_RPTATOR_INT_END			            C_RPTATOR_START
#define C_RPTATOR_INT_EN			            C_RPTATOR_START
#define	MASK_IMG_RANGE			                0xFFF

#define	B_ROT_MODE				                0
#define	MASK_ROT_MODE			                (0x7 << B_ROT_MODE)
#define B_IMAGE_RGB565                          3
#define	IMAGE_RGB565_ENABLE                     (((INT32U) 0)<<B_IMAGE_RGB565)
#define	IMAGE_YUV422_ENABLE                     (((INT32U) 1)<<B_IMAGE_RGB565)
#define B_IMAGE_YUV422                          4
#define	MASK_YUYV_ENABLE                        (((INT32U) 0x3)<<B_IMAGE_YUV422)

// color mode
#define IMAGE_YUYV                              0
#define IMAGE_YVYU                              1
#define IMAGE_UYVY                              2
#define IMAGE_VYUY                              3
#define IMAGE_RGB565                            4

// rotator mode
#define ROTATOR_90                              0
#define ROTATOR_180                             1
#define ROTATOR_270                             2
#define ROTATOR_HORIZONTAL_MIRROR               3
#define ROTATOR_VERTICAL_MIRROR                 4
#define ROTATOR_270_VERTICAL_MIRROR             5
#define ROTATOR_90_HORIZONTAL_MIRROR            6

extern void rotator_init(void);
extern void rotator_uninit(void);
extern INT32S rotator_src_img_info(INT8U color, INT16U w, INT16U h, INT32U in_buf);
extern INT32S rotator_tar_img_addr(INT32U out_buf);
extern INT32S rotator_mode_set(INT32U mode);
extern INT32S rotator_start(void);
extern INT32S rotator_end_wait(INT32U wait);

#endif		// __DRV_L1_PPU_H__
