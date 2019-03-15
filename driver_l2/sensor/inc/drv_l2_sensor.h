/******************************************************
* drv_l2_sensor.h
*
* Purpose: Sensor layer 2 header file
*
* Author: Eugene Hsu
*
* Date: 2014/08/25
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/
#ifndef DRV_L2_SENSOR_H
#define DRV_L2_SENSOR_H
#include "project.h"


/*********************************************************************
        Structure, enumeration and definition
**********************************************************************/

#define DUMMY_BUFFER_ADDRS2	0xF8500000
#define _DRV_L2_SENSOR      _DRV_L1_CSI

// sensor interface
#define CSI_INTERFACE	0
#define CDSP_INTERFACE	1

// sccb interface
#define SCCB_GPIO		0
#define SCCB_HW_I2C		1
#define SCCB_MODE		SCCB_GPIO//SCCB_HW_I2C//

#define MAX_SENSOR_NUM	2	/* Support 1 sensor in platform */
#define MAX_INFO_NUM	4

#define SENSOR_FLIP             0
#define TVP5150_SLAVE_ID_SEL    0

#define CP_DEMO_EN  0

/* Define the sensor name we support now */
#define SENSOR_OV7670_CSI_NAME			"ov_7670_csi"
#define SENSOR_GC0308_CSI_NAME			"gc_0308_csi"
#define SENSOR_OV3640_CSI_NAME			"ov_3640_csi"
#define SENSOR_OV7670_CDSP_NAME			"ov_7670_cdsp"
#define SENSOR_OV9712_CDSP_NAME			"ov_9712_cdsp"
#define SENSOR_OV3640_CSI_MIPI_NAME		"ov_3640_csi_mipi"
#define SENSOR_OV3640_CDSP_MIPI_NAME	"ov_3640_cdsp_mipi"
#define SENSOR_AR0330_CDSP_MIPI_NAME	"ar_0330_cdsp_mipi"
#define SENSOR_JXF02_CDSP_MIPI_NAME	    "jx_f02_cdsp_mipi"
#define SENSOR_OV5650_CDSP_MIPI_NAME	"ov_5650_cdsp_mipi"
#define SENSOR_SOI_H22_CDSP_MIPI_NAME	"soi_h22_cdsp_mipi"
#define TVIN_TVP5150_CSI_NAME			"tvp_5150_csi"
#define SENSOR_GC1004_CDSP_MIPI_NAME	"gc_1004_cdsp_mipi"
#define SENSOR_GC1004_CDSP_DVP_NAME		"gc_1004_cdsp_dvp"
#define SENSOR_JXH22_CDSP_DVP_NAME		"jx_h22_cdsp_dvp"
#define SENSOR_H42_CDSP_MIPI_NAME       "h42_cdsp_mipi"
#define SENSOR_JXH42_CDSP_DVP_NAME		"jx_h42_cdsp_dvp"
#define SENSOR_GC1014_CDSP_DVP_NAME		"gc_1014_cdsp_dvp"
#define SENSOR_GC1014_CDSP_MIPI_NAME	"gc_1014_cdsp_mipi"
#define SENSOR_OV5658_CDSP_MIPI_NAME   "ov5658_cdsp_mipi"
#define SENSOR_SP5506_CDSP_MIPI_NAME   "sp5506_cdsp_mipi"
#define SENSOR_GC1064_CDSP_MIPI_NAME	"gc_1064_cdsp_mipi"
#define SENSOR_GC5025_CDSP_MIPI_NAME   "gc_5025_cdsp_mipi"
#define SENSOR_H62_CDSP_MIPI_NAME		"h62_cdsp_mipi"


// sensor device enable
#if CP_DEMO_EN == 1
    #define _SENSOR_GC0308_CSI				1
    #define _SENSOR_H42_CDSP_MIPI           1
#else
    #define _SENSOR_OV7670_CSI				0
    #define _SENSOR_GC0308_CSI				1
    #define _SENSOR_OV3640_CSI				0
    #define _SENSOR_OV7670_CDSP				0
    #define _SENSOR_OV9712_CDSP				0
    #define _SENSOR_OV3640_CSI_MIPI		    0
    #define _SENSOR_OV3640_CDSP_MIPI	    0
    #define _SENSOR_OV5650_CDSP_MIPI	    0
    #define _SENSOR_JXF02_CDSP_MIPI	        0
    #define _SENSOR_AR0330_CDSP_MIPI	    0
    #define _SENSOR_SOI_H22_CDSP_MIPI	    0
    #define _TVIN_TVP5150_CSI				0
    #define _SENSOR_GC1004_CDSP_DVP			0
    #define _SENSOR_GC1004_CDSP_MIPI		0
    #define _SENSOR_JXH22_CDSP_DVP		    0
    #define _SENSOR_H42_CDSP_MIPI           0
    #define _SENSOR_JXH42_CDSP_DVP          0
    #define _SENSOR_GC1014_CDSP_DVP			0
    #define _SENSOR_GC1014_CDSP_MIPI        0
    #define _SENSOR_OV5658_CDSP_MIPI        0
    #define _SENSOR_SP5506_CDSP_MIPI        0
    #define _SENSOR_GC1064_CDSP_MIPI		0
    #define _SENSOR_GC5025_CDSP_MIPI        0
    #define _SENSOR_H62_CDSP_MIPI			0
#endif
/*ISP Pin Position Configuration*/
#define ISP_CLKO__IOC9								0x00006661
#define ISP_CLKO__IOD7								0x00006662
#define ISP_CLKO__IOD12								0x00006663
#define CSI_CLKO__IOC9								0x00016661
#define CSI_CLKO__IOD7								0x00016662
#define CSI_CLKO__IOD12								0x00016663
#define ISP_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11		0x00006664
#define ISP_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9		0x00006665
#define CSI_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11		0x00016664
#define CSI_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9		0x00016665
#define ISP_DATA2_9__IOC0_7							0x00006666
#define ISP_DATA2_9__IOB8_15						0x00006667
#define ISP_DATA2_9__IOE0_7							0x00006668
#define CSI_DATA2_9__IOC0_7							0x00016666
#define CSI_DATA2_9__IOB8_15						0x00016667
#define CSI_DATA2_9__IOE0_7							0x00016668
#define ISP_DATA0_1__IOB7_6                         0x00006669
#define ISP_DATA0_1__IOB5_4                         0x0000666a
#define ISP_DATA0_1__IOC13_12                       0x0000666b

//#define DBG_PRINT	print_string
//#define MEMCPY gp_memcp
//#define memset gp_memset
//#define gp_free(...)

typedef enum
{
	MCLK_NONE = 0,
	MCLK_12M,
	MCLK_13_5M,
	MCLK_24M,
	MCLK_27M,
	MCLK_48M,
	MCLK_SYSTEM_DIV2,
	MCLK_SYSTEM_DIV4
} CSI_CLK_DEF;

typedef enum
{
	WIDTH_320 = 320,
	WIDTH_640 = 640,
	WIDTH_800 = 800,
	WIDTH_1024 = 1024,
	WIDTH_1280 = 1280,
	WIDTH_1920 = 1920
} WIDTH_MODE;

typedef enum
{
	HEIGHT_240 = 240,
	HEIGHT_480 = 480,
	HEIGHT_600 = 600,
	HEIGHT_768 = 768,
	HEIGHT_720 = 720,
	HEIGHT_1080 = 1080
} HEIGHT_MODE;

typedef enum
{
	MODE_CCIR_601 = 0,
	MODE_CCIR_656,
	MODE_CCIR_HREF
} TIMING_MODE;

typedef enum
{
	MODE_NONE_INTERLACE = 0,
	MODE_INTERLACE
} INTERLACE_MODE;

typedef enum
{
	MODE_POSITIVE_EDGE = 0,
	MODE_NEGATIVE_EDGE
} PCLK_MODE;

typedef enum
{
	MODE_ACTIVE_LOW = 0,
	MODE_ACTIVE_HIGH
} HVSYNC_MODE;

typedef enum
{
	MODE_PSCALER_DISABLE = 0,
	MODE_PSCALER_SRC_CDSP,
	MODE_PSCALER_SRC_CSI
} PSCALER_MODE;

// sensor format is LSB to MSB
typedef enum
{
	V4L2_PIX_FMT_RGB565 = 0,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_YVYU,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_VYUY,
	V4L2_PIX_FMT_SBGGR8, //5
	V4L2_PIX_FMT_SGBRG8,
	V4L2_PIX_FMT_SGRBG8,
	V4L2_PIX_FMT_SRGGB8,
	V4L2_PIX_FMT_SGRBG10,
	V4L2_PIX_FMT_SRGGB10, //10
	V4L2_PIX_FMT_SBGGR10,
	V4L2_PIX_FMT_SGBRG10,
	V4L2_PIX_FMT_MAX
} SENSOR_FORMAT;

typedef enum
{
	CDSP = 0,
	CSI
} SENSOR_PATH;

typedef enum
{
	DVP_INTERFACE = 0,    //parallel
	MIPI_INTERFACE
} SENSOR_SOURCE;

typedef enum
{
	MD_NONE = 0,
	MD_16_SIZE,
	MD_8_SIZE
} SENSOR_MD;

typedef struct drv_l2_sensor_info_s
{
	INT32U mclk;				/* master clock */
	INT32U input_format;		/* input format */
	INT32U output_format;		/* output format */
	INT32U sensor_fps;			/* FPS in sensor */
	INT16U target_w; 			/* sensor width */
	INT16U target_h;			/* sensor height */
	INT16U sensor_w; 			/* sensor width */
	INT16U sensor_h;			/* sensor height */
	INT16U hoffset; 			/* sensor h offset */
	INT16U voffset;				/* sensor v offset */
	INT8U interface_mode;		/* input interface, HREF CCIR601 and CCIR656 */
	INT8U interlace_mode;		/* interlace mode */
	INT8U hsync_active;			/* hsync pin active level */
	INT8U vsync_active;			/* vsync pin active level */
} drv_l2_sensor_info_t;

typedef struct drv_l2_sensor_ops_s
{
	char*	name;
	void   (*init)(void);
	void   (*uninit)(void);
	void   (*stream_start)(INT32U index, INT32U bufA, INT32U bufB);
	void   (*stream_stop)(void);
	drv_l2_sensor_info_t*   (*get_info)(INT32U index);
	drv_l2_sensor_info_t	info[MAX_INFO_NUM];
} drv_l2_sensor_ops_t;

typedef struct drv_l2_sensor_para_s
{
    INT8U md_threshold;        /* sensor modition threshold */
    INT32U pscaler_src_mode;   /* sensor output selection: PSCALER_MODE */
    INT32U md_mode;            /* sensor modition enable, 0:disable, 1:enable*/
    INT32U md_block_size;      /* sensor modition block size, 0: 16x16, 1: 8x8*/
    INT32U md_buf;             /* sensor modition workmem */
    void (*md_csi_callback)(INT32U event);/* sensor modition callback function for CSI */
    void (*md_callback)(void); /* sensor modition callback function for CDSP */
} drv_l2_sensor_para_t;

/*********************************************************************
        External ops declaration for sensors we spport now
**********************************************************************/
#if (defined _SENSOR_OV7670_CSI) && (_SENSOR_OV7670_CSI == 1)
extern const drv_l2_sensor_ops_t ov7670_sensor_csi_ops;
#endif
#if (defined _SENSOR_GC0308_CSI) && (_SENSOR_GC0308_CSI == 1)
extern const drv_l2_sensor_ops_t gc0308_sensor_csi_ops;
#endif
#if (defined _SENSOR_OV3640_CSI) && (_SENSOR_OV3640_CSI == 1)
extern const drv_l2_sensor_ops_t ov3640_sensor_csi_ops;
#endif
#if (defined _SENSOR_OV7670_CDSP) && (_SENSOR_OV7670_CDSP == 1)
extern const drv_l2_sensor_ops_t ov7670_cdsp_ops;
#endif
#if (defined _SENSOR_OV9712_CDSP) && (_SENSOR_OV9712_CDSP == 1)
extern const drv_l2_sensor_ops_t ov9712_cdsp_ops;
#endif
#if (defined _SENSOR_OV3640_CSI_MIPI) && (_SENSOR_OV3640_CSI_MIPI == 1)
extern const drv_l2_sensor_ops_t ov3640_csi_mipi_ops;
#endif
#if (defined _SENSOR_OV3640_CDSP_MIPI) && (_SENSOR_OV3640_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t ov3640_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_OV5650_CDSP_MIPI) && (_SENSOR_OV5650_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t ov5650_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_AR0330_CDSP_MIPI) && (_SENSOR_AR0330_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t ar0330_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_JXF02_CDSP_MIPI) && (_SENSOR_JXF02_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t jxf02_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_SOI_H22_CDSP_MIPI) && (_SENSOR_SOI_H22_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t soi_h22_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_GC1004_CDSP_MIPI) && (_SENSOR_GC1004_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t gc1004_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_GC1004_CDSP_DVP) && (_SENSOR_GC1004_CDSP_DVP == 1)
extern const drv_l2_sensor_ops_t gc1004_cdsp_dvp_ops;
#endif
#if (defined _SENSOR_JXH22_CDSP_DVP) && (_SENSOR_JXH22_CDSP_DVP == 1)
extern const drv_l2_sensor_ops_t jxh22_cdsp_dvp_ops;
#endif
#if (defined _SENSOR_H42_CDSP_MIPI) && (_SENSOR_H42_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t h42_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_H62_CDSP_MIPI) && (_SENSOR_H62_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t h62_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_JXH42_CDSP_DVP) && (_SENSOR_JXH42_CDSP_DVP == 1)
extern const drv_l2_sensor_ops_t jxh42_cdsp_dvp_ops;
#endif
#if (defined _SENSOR_GC1014_CDSP_DVP) && (_SENSOR_GC1014_CDSP_DVP == 1)
extern const drv_l2_sensor_ops_t gc1014_cdsp_dvp_ops;
#endif
#if (defined _SENSOR_GC1014_CDSP_MIPI) && (_SENSOR_GC1014_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t gc1014_cdsp_mipi_ops;
#endif
#if (defined _TVIN_TVP5150_CSI) && (_TVIN_TVP5150_CSI == 1)
 extern const drv_l2_sensor_ops_t tvp5150_csi_ops;
#endif
#if (defined _SENSOR_OV5658_CDSP_MIPI) && (_SENSOR_OV5658_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t ov5658_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_SP5506_CDSP_MIPI) && (_SENSOR_SP5506_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t sp5506_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_GC1064_CDSP_MIPI) && (_SENSOR_GC1064_CDSP_MIPI == 1)
	extern const drv_l2_sensor_ops_t GC1064_cdsp_mipi_ops;
#endif
#if (defined _SENSOR_GC5025_CDSP_MIPI) && (_SENSOR_GC5025_CDSP_MIPI == 1)
extern const drv_l2_sensor_ops_t gc5025_cdsp_mipi_ops;
#endif

/*********************************************************************
        External function declaration
**********************************************************************/
extern void drv_l2_sensor_register_ops(drv_l2_sensor_ops_t* sensor);
extern void drv_l2_sensor_init(void);
extern drv_l2_sensor_ops_t* drv_l2_sensor_get_ops(INT32U index);
extern drv_l2_sensor_para_t* drv_l2_sensor_get_para(void);

extern void drv_l2_sensor_set_path(INT32U sen_path);
extern void drv_l2_sensor_set_csi_source(INT32U from_mipi_en);
extern INT32S drv_l2_sensor_set_mclkout(INT32U mclk_sel);
extern void function_position_sel(INT32U mclk_pos, INT32U ctrl_pos, INT32U data_pos1, INT32U data_pos0);
extern void drv_l2_sensor_clkout_position_set(INT32U mclk_pos);
#endif  //DRV_L2_SENSOR_H
