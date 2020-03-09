#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_l1_cdsp.h"
#include "drv_l1_gpio.h"
#include "drv_l1_scaler.h"
#include "drv_l1_pscaler.h"

#include "drv_l2_scaler.h"
#include "drv_l2_sensor.h"
#include "drv_l2_display.h"
#include "drv_l2_ad_key_scan.h"
#include "FaceDetectAP.h"

#include "32X32RGB565NEW.h" 	//  davis 2019.10.23
#include "defs_MLX.h"
#include "drv_l1_timer.h"
#include "drv_l2_sccb.h"
#include "drv_l1_i2c.h"
#include <math.h>		// for pow()

/* PPU draw use */

#include "gplib_ppu.h"
#include "gplib_ppu_dma.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "SPRITE_object_HDR.h"


#include "sprite_dataSP_HDR.h"
//#include "SPRITE_Number0_9_HDR.h" //  自己產生 先不用 

#include "TEXT_Platform_verification_1_HDR.h"




#if  GPLIB_PPU_EN

typedef struct {
	INT32U ppu_frame_workmem;
	INT32U ppu_narray_workmem;
    INT32U prcess_post_workmem;
    INT32U disp_prcess_workmem;
} prcess_mem_t;

#define C_PPU_DRV_FRAME_NUM		            3

static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static INT16U h_size,v_size;
#define EXSP_ENABLE                         1
#define CDM_MODE_ENABLE                     1
#define Photo_Disable                       1
static prcess_mem_t *prcess_mem_set;

#endif


#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define FRAME_BUF_ALIGN16                   0xF
#define FRAME_BUF_ALIGN8                    0x7
#define FRAME_BUF_ALIGN4                    0x3

#define CSI_SD_EN                           0
#define DISP_QUEUE_MAX		                3
#define C_DEVICE_FRAME_NUM		            3
#define DISP_TH_QUEUE_MAX		            3
#define C_DEVICE_TH_FRAME_NUM		        3

#define DUMMY_BUFFER_ADDRESS                0x50000000


#define SENSOR_SRC_WIDTH		            32
#define SENSOR_SRC_HEIGHT		            24
#define PRCESS_SRC_WIDTH		            SENSOR_SRC_WIDTH
#define PRCESS_SRC_HEIGHT		            SENSOR_SRC_HEIGHT


/*
#define SENSOR_SRC_WIDTH		            640
#define SENSOR_SRC_HEIGHT		            480
#define PRCESS_SRC_WIDTH		            SENSOR_SRC_WIDTH
#define PRCESS_SRC_HEIGHT		            SENSOR_SRC_HEIGHT
*/

#define PRCESS_STATE_OK                     0x80
#define DISP_USE_PSCALE_EN                  1
#define CSI_PSCALE_USE                      PSCALER_A
#define DISP_PSCALE_USE                     PSCALER_B


#define TRANSPARENT_COLOR 	0x00	// 無色 
//#define COLOR_FRAME_OUT		1
#define DBG_COLOR_TABLE		0
#define GRAY_GRAYMAX_COLOR 	0xff	// 無色 
#define GRAY_START_COLOR 	125		// 全亮 255 , 全暗 0

#define CORE_AREA_limit		1
#define SENSOR_AREA_WIDTH	32
#define SENSOR_AREA_HIGH	24

#define TMPZONE1_HOR_AREA_limit		8
#define TMPZONE1_VER_AREA_limit		4




#define	DEBUG_IO_OUT		1
#define TH_STATUS_A15_PAD     	IO_A15
#define TH_ALERT_A14_PAD    IO_A14
#define TH_TEST1_A12_PAD    	IO_A12



#define COLOR_SET_0		0	// gray out
#define COLOR_SET_1		1	// SPRITE out
/*
#define COLOR_SET_2		2
#define COLOR_SET_3		3
#define COLOR_SET_4		4
#define COLOR_SET_5		5
#define COLOR_SET_6		6
*/

#define COLOR_SET_MAX		COLOR_SET_1

#define ALTERTmpSet_MAX		500
#define ALTERTmpSet_MIN		150


#define GRAY_AMP_START	20
#define GRAY_AMP_SCALE	2


#define SP_RGBA_TEST_EN	1

#define ENABLE_PPU_run	1


#define SP_H_SIZE                           64//64//16 //8
#define SP_V_SIZE                           64//64//16 //8
#define SP_SIZE_CH                          4
#define SP_CHR_SIZE                         (SP_H_SIZE*SP_V_SIZE*SP_SIZE_CH)
#define SP_MAX_A_LEVEL                      30		// 256 sprite 可用 

#define SP_sm_H_SIZE                           32//64//16 //8
#define SP_sm_V_SIZE                           32//64//16 //8
#define SP_SIZE_CH                          4
#define SP_sm_CHR_SIZE                         (SP_sm_H_SIZE*SP_sm_V_SIZE*SP_SIZE_CH)
//define SP_MAX_A_LEVEL                      30		// 256 sprite 可用 

#define SP_unit_H_SIZE                           32//64//16 //8
#define SP_unit_V_SIZE                           64//64//16 //8
#define SP_SIZE_CH                          4
#define SP_unit_CHR_SIZE                         (SP_unit_H_SIZE*SP_unit_V_SIZE*SP_SIZE_CH)


#define	G_area		255
#define	B_area		0
#define	A_area		0

#define BIG_POINT	1



//  |1 1 1|
//  |1 1 1| Low Pass filiter 3*3
//  |1 1 1|

  const INT16S _LowPassFiliter_Factor[3][3]={
  	{1,1,1},
  	{1,1,1},
  	{1,1,1}
  	};
#define LowPassFiliter_ON	1
#define NO_SIGNAL_FACTOR	9	// for LP filiter
#define TMP_MAX_AVG_ON 		0


#if(LowPassFiliter_ON == 1)

const INT8U MLX_GrayOutputFactor_Ary[20]={8 , 8, 8, 8,  7,  5,  5,  5,  4,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3};
const INT8U MLX_Gray_MAX_val_Ary[20]   = {30,30,30,20, 20,110,110,110,220,220,220,220,220,220,220,220,220,220,240,240};	// 255 會有計算 error
const INT8U MLX_Gray_START_val_Ary[20] = {20,20,20,10, 10,10 ,10 , 10, 10, 10, 10,  5,  5,  5,  5,  5,  5,  5,  5,  5};

#endif

#if(LowPassFiliter_ON == 0)

#if  (FOV_BAB_55 == 1) && (FOV_BAA_110 == 0)
const INT8U MLX_GrayOutputFactor_Ary[20]={8 , 8, 8, 8,  7,  7,  4,  4,  4,  4,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3};
//const INT8U MLX_Gray_MAX_val_Ary[20]   = {40,40,40,50, 60, 60, 70, 80,100,110,135,170,195,210,210,210,210,210,220,220};
//const INT8U MLX_Gray_START_val_Ary[20] = {30,30,20,10, 10,10 ,10 , 10, 10,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5};
  const INT8U MLX_Gray_MAX_val_Ary[20]   = {30,30,30,20, 20, 20, 20, 20, 20, 20, 20,220,220,220,220,220,220,220,240,240};	// 255 會有計算 error
  const INT8U MLX_Gray_START_val_Ary[20] = {20,20,20,10, 10,10 ,10 , 10, 10, 10, 10,  5,  5,  5,  5,  5,  5,  5,  5,  5};
#else  // if FOV_BAA_110
const INT8U MLX_GrayOutputFactor_Ary[20]={8 , 8, 8, 8,  7,  7,  4,  4,  4,  4,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3};
  const INT8U MLX_Gray_MAX_val_Ary[20]   = {30,30,30,20, 20, 20, 20, 20, 20,220,220,220,220,220,220,220,220,220,240,240};	// 255 會有計算 error
  const INT8U MLX_Gray_START_val_Ary[20] = {20,20,20,10, 10,10 ,10 , 10, 10,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5};
//const INT8U MLX_Gray_MAX_val_Ary[20]   = {40,40,40,50, 60, 60, 70, 80,100,110,135,170,195,210,210,210,210,210,220,220};
//const INT8U MLX_Gray_START_val_Ary[20] = {30,30,20,10, 10,10 ,10 , 10, 10,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5};

#endif

#endif







typedef enum
{
	MSG_MLX_TH32x24_TASK_INIT = 0x2200,
	MSG_MLX_TH32x24_TASK_STOP,
	MSG_MLX_TH32x24_TASK_EXIT
} MLX_TH32x24_SENSOR_ENUM;


typedef struct
{
	gpImage ScaleIn;
	gpImage ScaleOut;
} ObjFrame_t;

typedef struct {
	INT32U inWidth;
	INT32U inHeight;
	INT32U inFormat;
	INT32U inSource;
	INT32U inBuffer;
	INT32U outWidth;
	INT32U outHeight;
	INT32U outFormat;
	INT32U outBuffer_A;
	INT32U outBuffer_B;
	INT32U intEnableFlag;
	INT32U outLineCount;
	INT32U delayHsyncCnt;
	FILTER_1D_PARAM Filter1DParam;
	TEXT_STAMP_PARAM textStampParam0;
	TEXT_STAMP_PARAM textStampParam1;
	TEXT_STAMP_COLOR_PARAM textColorParam;
	void (*callbackFunc)(INT32U PScaler_Event);
	INT8U  hFirstEnable;
	INT8U  pScalerNum;
	INT8U  textStamp0Enable;
	INT8U  textStamp1Enable;
}PSCALER_PARAM_STRUCT;



/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static osThreadId prcess_id;
static osThreadId csi_id;
static osThreadId disp_id;
static xQueueHandle free_frame_buffer_queue = NULL;
static xQueueHandle csi_frame_buffer_queue = NULL;
static xQueueHandle prcess_frame_buffer_queue = NULL;
static xQueueHandle prcess_state_queue = NULL;
static xQueueHandle pscaler_frame_buffer_queue = NULL;
static xQueueHandle disp_frame_buffer_queue = NULL;
static xSemaphoreHandle sem_disp_engine = NULL;
static PSCALER_PARAM_STRUCT PScalerParam = {0};
static INT32U device_h_size, device_v_size;


MLX_TH32x24Para_t *pMLX_TH32x24_Para;
paramsMLX90640_t *pMLX90640_Para;

const INT16U DELAYTIME_at_REFRESH_RATE3[8]={2000,1000,500,250,125,63,32,15};
const INT8U MLX_REFRESH_RATE_HZ3[8]={0,1,2,4,8,16,32,64};

INT32U UnderZeroDiff_value,OverZeroDiff_value;
INT8U	firstRun;
//INT16U	*pMLX_TH32x24_INT16U_avg_buf[AVG_buf_len];
INT8U	avg_buf_Enable;
INT32U	Gcnt_lp,Gcnt_lp2,GlobalTimeCnt1,GlobalTimeCnt2;

//T32S sprite_GreenPos_addr;
//INT32S sprite_RedPos_addr;
//T32S sprite_BluePos_addr;
//INT32S sprite_GrayPos_addr;


int	OvAlertVal_SUM;
//INT16S TmpDispBuf[TMP_MAX_DISP_buf_len];
INT8U  TmpMaxDispBuf_cnt;
INT16S TmpDispBuf_Pre,Tmp1pointDisp_Pre;



static void drv_disp_lock(void)
{
    if(sem_disp_engine){
        osSemaphoreWait(sem_disp_engine, osWaitForever);
    }
}

static void drv_disp_unlock(void)
{
    if(sem_disp_engine){
        osSemaphoreRelease(sem_disp_engine);
    }
}

// prcess ready buffer queue
static INT32S prcess_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(prcess_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(prcess_frame_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}

// disp ready buffer queue
static INT32S disp_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(disp_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(disp_frame_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}

// csi ready buffer queue
static INT32S csi_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(csi_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(csi_frame_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}

// prcess state queue
static INT32S prcess_state_post(INT32U state)
{
    INT32U event,temp;

    event = (INT32U)state;
    temp = osMessagePut(prcess_state_queue, (uint32_t)&event, osWaitForever);

	return temp;
}
static INT32S prcess_state_get(void)
{
    osEvent result;
	INT32S state = 0;

    result = osMessageGet(prcess_state_queue, 10);
    state = result.value.v;
    if((result.status != osEventMessage) || (state!=PRCESS_STATE_OK)) {
        state = 0;
	}

	return state;
}

// pscaler free buffer queue
static INT32S pscaler_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(pscaler_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(pscaler_frame_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}
static INT32S pscaler_frame_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(pscaler_frame_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(pscaler_frame_buffer_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}

// csi free buffer queue
static INT32S free_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    drv_disp_lock();
    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(free_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(free_frame_buffer_queue, (uint32_t)&event, 10);
    }
    drv_disp_unlock();

	return temp;
}
static INT32S free_frame_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(free_frame_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(free_frame_buffer_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}

static void draw_rect(gpImage *image, gpRect *r, INT32U color)
{
	INT32S i;
	INT32U *p_img;
	INT32S len;
	INT32S addr;
	INT32S x, y, w, h;
	INT16U *img = (INT16U *) image->ptr;

	w = r->width;
	h = r->height;

	if(w !=0 && h !=0) {
		x = r->x;
		y = r->y;

		// top line
		len = w >> 1;
		addr = (INT32U) (img + (y * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;

		for(i=0; i<len; i++) {
			*p_img++ = color;
		}

		addr = (INT32S) (img + ((y+1) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0; i<len; i++) {
			*p_img++ = color;
		}

		addr = (INT32S) (img + ((y+2) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0; i<len; i++) {
			*p_img++ = color;
		}

		addr = (INT32S) (img + ((y+3) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0; i<len; i++) {
			*p_img++ = color;
		}

		// bottom line
		addr = (INT32S) (img + ((y +h) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0;i<len;i++) {
			*p_img++ = color;
		}

		addr = (INT32S) (img + ((y - 1 + h) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0;i<len;i++) {
			*p_img++ = color;
		}

		addr = (INT32S) (img + ((y - 2 + h) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0;i<len;i++) {
			*p_img++ = color;
		}

		addr = (INT32S) (img + ((y - 3 + h) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0;i<len;i++) {
			*p_img++ = color;
		}

		// left line
		addr = (INT32S) (img + ((y+4) * image->width) + x);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		len = h - 8;
		for(i=0;i<len;i++) {
			*p_img++ = color;
			*p_img++ = color;
			p_img += (image->width - 4) >> 1;
		}

		// right line
		addr = (INT32S) (img + ((y+4) * image->width) + x + w - 2);
		addr >>= 2;
		addr <<= 2;
		p_img = (INT32U *) addr;
		for(i=0;i<len;i++) {
			*p_img++ = color;
			*p_img++ = color;
			p_img += (image->width - 4) >> 1;
		}
	}
}

static void draw_obj(INT32U img_adr, INT32U bufw, INT32U bufh, INT32S total_obj_cnt, gpRect *obj_result_d)
{
	INT32S i;
	INT32U color;
	gpImage Image;

	Image.width = bufw;					// depends on display buffer width
	Image.height = bufh;				// depends on display buffer height
	Image.widthStep = bufw*2; 			// depends on display format
	Image.ch = 2; 						// depends on display format
	Image.format = IMG_FMT_UYVY;  		// depends on display format
	Image.ptr = (INT8U *)img_adr;

	//color = 0xff4c554c; // green
	//color = 0x4cff4c55; // blue
	color = 0xffffffff; // red
	for(i=0; i<total_obj_cnt; i++) {
		draw_rect(&Image, &obj_result_d[i], color);
	}
}

static INT32S fd_display_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
#if DISP_USE_PSCALE_EN == 1
	INT32S retStatus;

	if(frame_buffer == 0 || in_buffer == 0) {
		return -1;
	}

    drv_l1_pscaler_init(DISP_PSCALE_USE);
    drv_l1_pscaler_clk_ctrl(DISP_PSCALE_USE, 1);
    drv_l1_pscaler_input_source_set(DISP_PSCALE_USE, PIPELINE_SCALER_INPUT_SOURCE_DRAM);
    drv_l1_pscaler_input_pixels_set(DISP_PSCALE_USE, in_w, in_h);
    drv_l1_pscaler_output_pixels_set(DISP_PSCALE_USE,((in_w*65536)/out_w), out_w, ((in_h*65536)/out_h), out_h);
    drv_l1_pscaler_output_fifo_line_set(DISP_PSCALE_USE, out_w, 0);
    drv_l1_pscaler_interrupt_set(DISP_PSCALE_USE, PIPELINE_SCALER_INT_ENABLE_422GP420_FRAME_END);
    drv_l1_pscaler_input_format_set(DISP_PSCALE_USE, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
    drv_l1_pscaler_input_buffer_set(DISP_PSCALE_USE, in_buffer);
    drv_l1_pscaler_output_A_buffer_set(DISP_PSCALE_USE, frame_buffer);
    drv_l1_pscaler_output_format_set(DISP_PSCALE_USE, PIPELINE_SCALER_OUTPUT_FORMAT_YUYV);
    drv_l1_pscaler_start(DISP_PSCALE_USE);
    do
    {
        retStatus = drv_l1_pscaler_status_get(DISP_PSCALE_USE);
        if(retStatus & PIPELINE_SCALER_STATUS_FRAME_DONE)
            break;
        osDelay(1);
    }
    while(1);
    drv_l1_pscaler_stop(DISP_PSCALE_USE);
#else
    ObjFrame_t *pInput,Input;

 	if(frame_buffer == 0 || in_buffer == 0) {
		return -1;
	}

    pInput = (ObjFrame_t *)&Input;
	if(pInput == 0) {
		return -1;
	}

	pInput->ScaleIn.width = in_w;
	pInput->ScaleIn.height = in_h;
    pInput->ScaleIn.ch  = 2;
    pInput->ScaleIn.widthStep = in_w * 2;
	pInput->ScaleIn.format = IMG_FMT_UYVY;
	pInput->ScaleIn.ptr = (INT8U *)in_buffer;

	pInput->ScaleOut.width = out_w;
	pInput->ScaleOut.height = out_h;
    pInput->ScaleOut.ch  = 2;
    pInput->ScaleOut.widthStep = out_w * 2;
	pInput->ScaleOut.format = IMG_FMT_UYVY;
	pInput->ScaleOut.ptr = (INT8U *)frame_buffer;
    drv_l2_scaler_full_screen(0, 1, (gpImage *)&pInput->ScaleIn, (gpImage *)&pInput->ScaleOut);
#endif

    return 0;
 }

static void mazePscalerSet(PSCALER_PARAM_STRUCT* pPScalerParam)
{
	INT32U widthFactor,heightFactor;

	widthFactor = ((pPScalerParam->inWidth*65536)/pPScalerParam->outWidth);
	heightFactor = ((pPScalerParam->inHeight*65536)/pPScalerParam->outHeight);

	drv_l1_pscaler_init(pPScalerParam->pScalerNum);
	drv_l1_pscaler_clk_ctrl(pPScalerParam->pScalerNum,1);

	drv_l1_pscaler_input_pixels_set(pPScalerParam->pScalerNum,pPScalerParam->inWidth,pPScalerParam->inHeight);
	drv_l1_pscaler_input_source_set(pPScalerParam->pScalerNum,pPScalerParam->inSource);
	drv_l1_pscaler_input_buffer_set(pPScalerParam->pScalerNum,pPScalerParam->inBuffer);

	drv_l1_pscaler_input_format_set(pPScalerParam->pScalerNum,pPScalerParam->inFormat);

	//+++ Frame Mode: FIFO Line = Output Height
	drv_l1_pscaler_output_fifo_line_set(pPScalerParam->pScalerNum,pPScalerParam->outLineCount,0);
	drv_l1_pscaler_output_pixels_set(pPScalerParam->pScalerNum,widthFactor,pPScalerParam->outWidth,heightFactor,pPScalerParam->outHeight);

	if(pPScalerParam->outFormat == PIPELINE_SCALER_OUTPUT_FORMAT_MB420)
	{
		drv_l1_pscaler_H264_output_A_buffer_set(pPScalerParam->pScalerNum,pPScalerParam->outBuffer_A);
		drv_l1_pscaler_H264_output_B_buffer_set(pPScalerParam->pScalerNum,pPScalerParam->outBuffer_B);
	}
	else
	{
		drv_l1_pscaler_output_A_buffer_set(pPScalerParam->pScalerNum,pPScalerParam->outBuffer_A);
		drv_l1_pscaler_output_B_buffer_set(pPScalerParam->pScalerNum,pPScalerParam->outBuffer_B);
	}

	drv_l1_pscaler_output_format_set(pPScalerParam->pScalerNum,pPScalerParam->outFormat);

	drv_l1_pscaler_interrupt_set(pPScalerParam->pScalerNum,pPScalerParam->intEnableFlag);
	drv_l1_pscaler_callback_register(pPScalerParam->pScalerNum,pPScalerParam->callbackFunc);

	drv_l1_pscaler_start(pPScalerParam->pScalerNum);
}

static void PScaler_Callback_ISR_AutoZoom(INT32U PScaler_Event)
{
	INT32U prcessBuf,csiBuf;

	if(PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
	{
        csiBuf = free_frame_buffer_get(0);
        if(csiBuf)
        {
            prcessBuf = drv_l1_pscaler_output_A_buffer_get(PScalerParam.pScalerNum);
            csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            drv_l1_pscaler_output_A_buffer_set(PScalerParam.pScalerNum,(INT32U)csiBuf);
        }
        else
        {
            prcessBuf = drv_l1_pscaler_output_A_buffer_get(PScalerParam.pScalerNum);
            csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            //DBG_PRINT("A");
        }
	}
	else if(PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
	{
        csiBuf = free_frame_buffer_get(0);
        if(csiBuf)
        {
            prcessBuf = drv_l1_pscaler_output_B_buffer_get(PScalerParam.pScalerNum);
            csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            drv_l1_pscaler_output_B_buffer_set(PScalerParam.pScalerNum,(INT32U)csiBuf);
        }
        else
        {
            prcessBuf = drv_l1_pscaler_output_B_buffer_get(PScalerParam.pScalerNum);
            csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            //DBG_PRINT("B");
        }
	}
}

static void mazeTest_Preview_PScaler(void)
{
    CHAR *p;
	INT32U i,PrcessBuffer,csi_mode,temp;
	INT32U csiBufferSize,csiBuffer;
	drv_l2_sensor_ops_t *pSencor;
	drv_l2_sensor_info_t *pInfo;

	csiBufferSize = PRCESS_SRC_WIDTH*PRCESS_SRC_HEIGHT*2;
    csiBuffer = DUMMY_BUFFER_ADDRESS;

	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*DISP_QUEUE_MAX)+64), 32);
    if(PrcessBuffer == 0)
    {
        DBG_PRINT("PrcessBuffer fail\r\n");
        while(1);
    }
	PrcessBuffer = (INT32U)((PrcessBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<DISP_QUEUE_MAX; i++)
	{
		temp = (PrcessBuffer+i*csiBufferSize);
		free_frame_buffer_add((INT32U *)temp,1);
		DBG_PRINT("CSIBuffer:0x%X \r\n",temp);
	}

    // sensor init
	drv_l2_sensor_init();
    pSencor = drv_l2_sensor_get_ops(0);

 	// get csi or cdsp
	p = (CHAR *)strrchr((CHAR *)pSencor->name, 'c');
	if(p == 0) {
        DBG_PRINT("get csi or cdsp fail\r\n");
        while(1);
	}

	if(strncmp((CHAR *)p, "csi", 3) == 0) {
		csi_mode = CSI_INTERFACE;
	} else if(strncmp((CHAR *)p, "cdsp", 4) == 0) {
		csi_mode = CDSP_INTERFACE;
	} else {
        DBG_PRINT("csi mode fail\r\n");
        while(1);
	}

	for(i=0; i<3; i++) {
		pInfo = pSencor->get_info(i);
		if(pInfo->target_w == SENSOR_SRC_WIDTH && pInfo->target_h == SENSOR_SRC_HEIGHT) {
			temp = i;
			break;
		}
	}

	if(i == 3) {
        DBG_PRINT("get csi width and height fail\r\n");
        while(1);
	}
	DBG_PRINT("sensor get_info(%d)_davis\r\n",temp);
	pSencor->init();
	pSencor->stream_start(temp, csiBuffer, csiBuffer);

	// PScaler
	PScalerParam.pScalerNum = CSI_PSCALE_USE;
	PScalerParam.inBuffer = csiBuffer;
	PScalerParam.outBuffer_A = free_frame_buffer_get(1);
	PScalerParam.outBuffer_B = free_frame_buffer_get(1);

	PScalerParam.inWidth = SENSOR_SRC_WIDTH;
	PScalerParam.inHeight = SENSOR_SRC_HEIGHT;

	PScalerParam.outWidth = PRCESS_SRC_WIDTH;
	PScalerParam.outHeight = PRCESS_SRC_HEIGHT;

	PScalerParam.outLineCount = PRCESS_SRC_HEIGHT;

	PScalerParam.inFormat = PIPELINE_SCALER_INPUT_FORMAT_YUYV;
	PScalerParam.outFormat = PIPELINE_SCALER_OUTPUT_FORMAT_YUYV;

    if (csi_mode == CSI_INTERFACE)
    {
        // CSI
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CSI;
        // Open CSI data path
        drvl1_csi_input_pscaler_set(1);
    }
    else
    {
        // CDSP
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CDSP;
        // Open CDSP data path
        drv_l1_CdspSetYuvPscalePath(1);
    }

	PScalerParam.intEnableFlag = PIPELINE_SCALER_INT_ENABLE_ALL;
    PScalerParam.callbackFunc = PScaler_Callback_ISR_AutoZoom;

	mazePscalerSet(&PScalerParam);
}




 //------------------------------------------------------------------------------
 // example1:
 // Calculate  the	object	temperatures  for  all	the  pixels  in  a	frame,
 // object	emissivity	is	0.95  and  the reflected temperature is 23.15°C (measured by the user):
 // example2:
 // Calculate  the	object	temperatures  for  all	the  pixels  in  a	frame,
 // object	emissivity	is	0.95  and  the reflected temperature is the sensor ambient temperature:

 void MLX90640_CalculateTo(INT16U* pMLX32x32_frameData_prcess_INT16U_buf,float emissivity, float tr)
 {
	 float vdd;
	 float ta;
	 float ta4;
	 float tr4;
	 float taTr;
	 float gain;
	 float irDataCP[2];
	 float irData;
	 float alphaCompensated;
	 uint8_t mode;
	 int8_t ilPattern;
	 int8_t chessPattern;
	 int8_t pattern;
	 int8_t conversionPattern;
	 float Sx;
	 float To;
	 float alphaCorrR[4];
	 int8_t range;
	 uint16_t subPage;
	 int i,pixelNumber;

	 float image;


	 //subPage = pMLX_TH32x24_Para->frameData[833];

	 subPage =*(pMLX32x32_frameData_prcess_INT16U_buf+833);
	 vdd = pMLX_TH32x24_Para->MLX_TH32x24_vdd;
	 ta = pMLX_TH32x24_Para->MLX_TH32x24_ta;


	 //DBG_PRINT("frame = %d (%f)/[%f]",subPage,vdd,ta);

	 ta4 = pow((ta + 273.15), (double)4);
	 tr4 = pow((tr + 273.15), (double)4);
	 taTr = tr4 - (tr4-ta4)/emissivity;

	 alphaCorrR[0] = 1 / (1 + pMLX90640_Para->ksTo[0] * 40);
	 alphaCorrR[1] = 1 ;
	 alphaCorrR[2] = (1 + pMLX90640_Para->ksTo[2] * pMLX90640_Para->ct[2]);
	 alphaCorrR[3] = alphaCorrR[2] * (1 + pMLX90640_Para->ksTo[3] * (pMLX90640_Para->ct[3] - pMLX90640_Para->ct[2]));

 //------------------------- Gain calculation -----------------------------------
	 //gain = pMLX_TH32x24_Para->frameData[778];
	 gain =*(pMLX32x32_frameData_prcess_INT16U_buf+778);

	 if(gain > 32767)
	 {
		 gain = gain - 65536;
	 }

	 gain = pMLX90640_Para->gainEE / gain;


	// DBG_PRINT("subPage = %d,vdd = %f,ta = %f \r\n",subPage,pMLX_TH32x24_Para->MLX_TH32x24_vdd,ta);

 //------------------------- To calculation -------------------------------------
	 //mode = (pMLX_TH32x24_Para->frameData[832] & 0x1000) >> 5;
	 mode = (*(pMLX32x32_frameData_prcess_INT16U_buf+832) & 0x1000) >> 5;

	 //irDataCP[0] = pMLX_TH32x24_Para->frameData[776];
	 //irDataCP[1] = pMLX_TH32x24_Para->frameData[808];

	 irDataCP[0] = *(pMLX32x32_frameData_prcess_INT16U_buf+776);
	 irDataCP[1] = *(pMLX32x32_frameData_prcess_INT16U_buf+808);
	 for( i = 0; i < 2; i++)
	 {
		 if(irDataCP[i] > 32767)
		 {
			 irDataCP[i] = irDataCP[i] - 65536;
		 }
		 irDataCP[i] = irDataCP[i] * gain;
	 }
	 irDataCP[0] = irDataCP[0] - pMLX90640_Para->cpOffset[0] * (1 + pMLX90640_Para->cpKta * (ta - 25)) * (1 + pMLX90640_Para->cpKv * (vdd - 3.3));
	 if( mode ==  pMLX90640_Para->calibrationModeEE)
	 {
		 irDataCP[1] = irDataCP[1] - pMLX90640_Para->cpOffset[1] * (1 + pMLX90640_Para->cpKta * (ta - 25)) * (1 + pMLX90640_Para->cpKv * (vdd - 3.3));
	 }
	 else
	 {
	   irDataCP[1] = irDataCP[1] - (pMLX90640_Para->cpOffset[1] + pMLX90640_Para->ilChessC[0]) * (1 + pMLX90640_Para->cpKta * (ta - 25)) * (1 + pMLX90640_Para->cpKv * (vdd - 3.3));
	 }

	 for( pixelNumber = 0; pixelNumber < 768; pixelNumber++)
	 {
		 ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
		 chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2);
		 conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);

		 if(mode == 0)
		 {
		   pattern = ilPattern;
		 }
		 else
		 {
		   pattern = chessPattern;
		 }

		 //if(pattern == pMLX_TH32x24_Para->frameData[833])
		 if(pattern == *(pMLX32x32_frameData_prcess_INT16U_buf+833)) // frameData[833] 定義 subpage 0 or 1
		 {
			 //irData = pMLX_TH32x24_Para->frameData[pixelNumber];
			 irData =*(pMLX32x32_frameData_prcess_INT16U_buf+pixelNumber);
			 if(irData > 32767)
			 {
				 irData = irData - 65536;
			 }
			 irData = irData * gain;

			 irData = irData - pMLX90640_Para->offset[pixelNumber]*(1 + pMLX90640_Para->kta[pixelNumber]*(ta - 25))*(1 + pMLX90640_Para->kv[pixelNumber]*(vdd - 3.3));
			 if(mode !=  pMLX90640_Para->calibrationModeEE)
			 {
			   irData = irData + pMLX90640_Para->ilChessC[2] * (2 * ilPattern - 1) - pMLX90640_Para->ilChessC[1] * conversionPattern;
			 }

			 irData = irData / emissivity;

			 irData = irData - pMLX90640_Para->tgc * irDataCP[subPage];

			 alphaCompensated = (pMLX90640_Para->alpha[pixelNumber] - pMLX90640_Para->tgc * pMLX90640_Para->cpAlpha[subPage])*(1 + pMLX90640_Para->KsTa * (ta - 25));


			 image = irData/alphaCompensated;

			 pMLX_TH32x24_Para->result_image[pixelNumber] =image;

		if (pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag == 1)	// 開始計算溫度 
			{
			// ZONE 1 only
			if (((pixelNumber/rowNumEnd_32) >= TMPZONE1_VER_AREA_limit)&&((pixelNumber/rowNumEnd_32)<(SENSOR_AREA_HIGH -TMPZONE1_VER_AREA_limit))
			 &&((pixelNumber%rowNumEnd_32) >= TMPZONE1_HOR_AREA_limit)&&((pixelNumber%rowNumEnd_32)<(SENSOR_AREA_WIDTH -TMPZONE1_HOR_AREA_limit)))
				{
			 Sx = pow((double)alphaCompensated, (double)3) * (irData + alphaCompensated * taTr);
			 Sx = sqrt(sqrt(Sx)) * pMLX90640_Para->ksTo[1];

			 To = sqrt(sqrt(irData/(alphaCompensated * (1 - pMLX90640_Para->ksTo[1] * 273.15) + Sx) + taTr)) - 273.15;

			 if(To < pMLX90640_Para->ct[1])
			 {
				 range = 0;
			 }
			 else if(To < pMLX90640_Para->ct[2])
			 {
				 range = 1;
			 }
			 else if(To < pMLX90640_Para->ct[3])
			 {
				 range = 2;
			 }
			 else
			 {
				 range = 3;
			 }

			 To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + pMLX90640_Para->ksTo[range] * (To - pMLX90640_Para->ct[range]))) + taTr)) - 273.15;

			 pMLX_TH32x24_Para->result[pixelNumber] =(INT16S)(To*10);
			 	}

			else
			 	pMLX_TH32x24_Para->result[pixelNumber] = -1; // NOT ZONE1 set -1
			}
		 }
	 }
 }

 void MLX90640_GetImage(INT16U* pMLX32x32_frameData_prcess_INT16U_buf)
 {
	 float vdd;
	 float ta;
	 float gain;
	 float irDataCP[2];
	 float irData;
	 float alphaCompensated;
	 uint8_t mode;
	 int8_t ilPattern;
	 int8_t chessPattern;
	 int8_t pattern;
	 int8_t conversionPattern;
	 float image;
	 uint16_t subPage;
	 int i,pixelNumber;

	 //subPage = pMLX_TH32x24_Para->frameData[833];
	 subPage =*(pMLX32x32_frameData_prcess_INT16U_buf+833);

	 vdd = pMLX_TH32x24_Para->MLX_TH32x24_vdd;
	 ta = pMLX_TH32x24_Para->MLX_TH32x24_ta;

 //------------------------- Gain calculation -----------------------------------
	 //gain = pMLX_TH32x24_Para->frameData[778];
	 gain =*(pMLX32x32_frameData_prcess_INT16U_buf+778);

	 if(gain > 32767)
	 {
		 gain = gain - 65536;
	 }

	 gain = pMLX90640_Para->gainEE / gain; // K-gain

 //------------------------- Image calculation -------------------------------------
	// mode = (pMLX_TH32x24_Para->frameData[832] & 0x1000) >> 5;
	 mode = (*(pMLX32x32_frameData_prcess_INT16U_buf+832) & 0x1000) >> 5;

	 //DBG_PRINT(" GetImage mode = 0x%04x \r\n",mode);

 //
 // mode 決定 是 0 Interleaved (TV) mode / 1 Chess pattern (default)
 //
 // NOTE:  In  order  to  limit  the  noise  in  the  final  To  calculation  it  is  advisable
 // to	filter	the  CP  readings  at  this  point	of calculation.
 // A good practice would be to apply a Moving Average Filter with length of 16 or higher.
 //
	 //irDataCP[0] = pMLX_TH32x24_Para->frameData[776];
	 //irDataCP[1] = pMLX_TH32x24_Para->frameData[808];

	 irDataCP[0] = *(pMLX32x32_frameData_prcess_INT16U_buf+776);
	 irDataCP[1] = *(pMLX32x32_frameData_prcess_INT16U_buf+808);

	 for( i = 0; i < 2; i++)
	 {
		 if(irDataCP[i] > 32767)
		 {
			 irDataCP[i] = irDataCP[i] - 65536;
		 }
		 irDataCP[i] = irDataCP[i] * gain;
	 }
	 irDataCP[0] = irDataCP[0] - pMLX90640_Para->cpOffset[0] * (1 + pMLX90640_Para->cpKta * (ta - 25)) * (1 + pMLX90640_Para->cpKv * (vdd - 3.3));


	 // The value of the offset for compensating pixel for the subpage 1 depends on
	 // the reading pattern

	 if( mode ==  pMLX90640_Para->calibrationModeEE)
	 {

		 //DBG_PRINT(" GetImage mode == calibrationModeEE \r\n");
		 irDataCP[1] = irDataCP[1] - pMLX90640_Para->cpOffset[1] * (1 + pMLX90640_Para->cpKta * (ta - 25)) * (1 + pMLX90640_Para->cpKv * (vdd - 3.3));
	 }
	 else
	 {
	   irDataCP[1] = irDataCP[1] - (pMLX90640_Para->cpOffset[1] + pMLX90640_Para->ilChessC[0]) * (1 + pMLX90640_Para->cpKta * (ta - 25)) * (1 + pMLX90640_Para->cpKv * (vdd - 3.3));
	 }

	 for( pixelNumber = 0; pixelNumber < 768; pixelNumber++)
	 {
		 ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
		 chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2);
		 conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);

		 if(mode == 0)
		 {
		   pattern = ilPattern;
		 }
		 else
		 {
		   pattern = chessPattern; // use -> Chess patter
		 }

		 if(pattern == *(pMLX32x32_frameData_prcess_INT16U_buf+833)) // frameData[833] 定義 subpage 0 or 1
		 {
			// irData = pMLX_TH32x24_Para->frameData[pixelNumber];
			 irData =*(pMLX32x32_frameData_prcess_INT16U_buf+pixelNumber);
			 if(irData > 32767)
			 {
				 irData = irData - 65536;
			 }
			 irData = irData * gain;

			 irData = irData - pMLX90640_Para->offset[pixelNumber]*(1 + pMLX90640_Para->kta[pixelNumber]*(ta - 25))*(1 + pMLX90640_Para->kv[pixelNumber]*(vdd - 3.3));
			 if(mode !=  pMLX90640_Para->calibrationModeEE)
			 {
			   irData = irData + pMLX90640_Para->ilChessC[2] * (2 * ilPattern - 1) - pMLX90640_Para->ilChessC[1] * conversionPattern;
			 }

			 irData = irData - pMLX90640_Para->tgc * irDataCP[subPage];

			 alphaCompensated = (pMLX90640_Para->alpha[pixelNumber] - pMLX90640_Para->tgc * pMLX90640_Para->cpAlpha[subPage])*(1 + pMLX90640_Para->KsTa * (ta - 25));

			 image = irData/alphaCompensated;

			 pMLX_TH32x24_Para->result_image[pixelNumber] =image;

			// pMLX_TH32x24_Para->result[pixelNumber] = (INT16S)(image*10); // 放大10倍 
		 }
	 }
 }

 #if 0
 void CalulateData(INT8U format ,INT16U SETcontrolRegister1,INT32U csi_buf)
 {


	 INT16U  cnt,frameData_cnt;
	 INT16U  dataReady,statusRegister,controlRegister1;
	INT16U	EEaddress16;
	INT8U 	EEaddr[2],*pEEaddr;
	INT8U  *pMLX32x32_READ_INT8U_buf;
	INT16U  *pMLX32x32_frameData_INT16U_buf;
	INT16U  *pMLX32x32_frameData_csi_INT16U_buf;

	 drv_l1_i2c_bus_handle_t MXL_handle;

	 int	 error;

	 int i,pixelNumber;

	 INT32U  TimeCnt1,TimeCnt2,TimeCnt3;
	 INT32U  TimeCnt1a,TimeCnt2a;
	 INT32U  TimeCnt1b,TimeCnt2b;
	//INT8U	 frameNo;


	 MXL_handle.devNumber = I2C_1;
	 MXL_handle.slaveAddr = MLX90640_SLAVE_ADDR<<1;
	 MXL_handle.clkRate = 950;

		 pEEaddr = EEaddr;
		 pMLX32x32_READ_INT8U_buf = (INT8U*)pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF;

		pMLX32x32_frameData_INT16U_buf = (INT16U*)pMLX_TH32x24_Para->frameData;
		pMLX32x32_frameData_csi_INT16U_buf = (INT16U*)csi_buf;

	 TimeCnt2a = xTaskGetTickCount();

	 //DBG_PRINT("************ GetFrameData frame 0 ************************************** \r\n");
	 dataReady = 0;
	 frameData_cnt=0;
	 while(dataReady == 0)
	 {
		 do{
			 error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrStatus,&statusRegister);
			 //DBG_PRINT("read return-1  = %d \r\n",error);  // return data length , if error = -1
			 //  需要 重新讀取 !! 改成 副程式 檢查 
			 if( error == -1){
				 DBG_PRINT("frame0 read status error !! \r\n");
				 osDelay(10);
			 }
		 }while(error == -1);

		 dataReady = statusRegister & 0x0008; // 1 : A new data is available in RAM ?

		 //DBG_PRINT(" 2.statusRegister = 0x%04x, dataReady = 0x%04x,frameData_cnt = %d \r\n",
		 //  statusRegister,dataReady,frameData_cnt);
		 if(dataReady == 0){
				 //osDelay(DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_64HZ]);
				 osDelay(CONVERT_WAIT_TIME/5);
				 //DBG_PRINT("\r\n 1-frame0 delay  = %d ms > %d\r\n",
				//	 DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_64HZ],frameData_cnt);

			 frameData_cnt++;
		 }
		 if(frameData_cnt >4){	 // reset MLX

			 error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
			 //DBG_PRINT("(reset)read controlRegister1 set frame0= 0x%04x \r\n",controlRegister1);
			 controlRegister1 = controlRegister1 & MLX90640_SetModeClear ;
			 controlRegister1 = controlRegister1 | MLX90640_SetStepModeSubpageRep ;

			 drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrControlRegister1,controlRegister1);
			 //DBG_PRINT("(subpage = 0)write controlRegister1 = 0x%04x \r\n",controlRegister1);
			 //osDelay(DELAYTIME_at_REFRESH_RATE[MLX90640_REFRESH_RATE_32HZ]);
			 //  DBG_PRINT("\r\n frameData_cnt> frame0 delay  = %d ms \r\n",
			 // 	 DELAYTIME_at_REFRESH_RATE[MLX90640_REFRESH_RATE_32HZ]);

			 error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);

			 frameData_cnt = 0;
			 }
	 }

	  TimeCnt2b = xTaskGetTickCount();

	  if(dataReady != 0)
	  {

		 //DBG_PRINT("drv_l1_i2c_multi_read MLX90640_RAMAddrstart \r\n");
		 EEaddress16 = MLX90640_RAMAddrstart;
		 EEaddr[0]=(INT8U)(EEaddress16 >> 8);
		 EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
		 error = drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pMLX32x32_READ_INT8U_buf
			 ,MLX90640_RAM_AddrRead*2,MXL_I2C_RESTART_MODE); // 多筆讀取 RAM

		 //DBG_PRINT("read return-2  = %d \r\n",error);  // return data length , if error = -1

     //
	 // in Status register - 0x8000
	 // Bit3:b
	 //  0:No new data is available in RAM (must be reset be the customer)
	 // 0x0030 :
	 // Bit4:
	 // 1: Data in RAM overwrite is enabled
	 // Bit5:
	 // 1: In step mode - start of measurement
	 // 	 (set by the customer and cleared once the measurement is done)
	 //
		 error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);

		 for(cnt=0; cnt < MLX90640_RAM_AddrRead; cnt++)
		 {
			 i = cnt << 1;
			 *(pMLX32x32_frameData_INT16U_buf+cnt) = (INT16U)*(pMLX32x32_READ_INT8U_buf+i) *256
				 + (INT16U)*(pMLX32x32_READ_INT8U_buf+i+1);
		 }

		  for(cnt=0; cnt < MLX90640_RAM_AddrRead; cnt++)
		 {
			 i = cnt << 1;
			 *(pMLX32x32_frameData_csi_INT16U_buf+cnt) = (INT16U)*(pMLX32x32_READ_INT8U_buf+i) *256
				 + (INT16U)*(pMLX32x32_READ_INT8U_buf+i+1);
		 }




		 pMLX_TH32x24_Para->frameData[833] = statusRegister & 0x0001;	 // 紀錄 目前是 subpage ?
		 //DBG_PRINT(" 3a. [subpage = %d] \r\n",pMLX_TH32x24_Para->frameData[833]);

		 pMLX_TH32x24_Para->frameData[832] = SETcontrolRegister1;

		// error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);
	 }


	 TimeCnt1a = xTaskGetTickCount();


#if 0

	 //DBG_PRINT("************ GetFrameData frame 1 ************************************** \r\n");


	 dataReady = 0;
	 frameData_cnt=0;

	 osDelay(CONVERT_WAIT_TIME);

	 while (dataReady == 0)
	 {

		 do{
			 error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrStatus,&statusRegister);
			 //DBG_PRINT("read return  = %d \r\n",error);	 // return data length , if error = -1
			 //  需要 重新讀取 !! 改成 副程式 檢查 
			 if( error == -1){
				 DBG_PRINT("frame1 vdd/ta error !!(0x%04x)\r\n",statusRegister);
				 osDelay(10);
				 }
		 }while(error == -1);

		 dataReady = statusRegister & 0x0008;

		 //DBG_PRINT("\r\n 2. statusRegister = 0x%04x, dataReady = 0x%04x,frameData_cnt = %d [t=%d] \r\n",
		 //  statusRegister,dataReady,frameData_cnt,xTaskGetTickCount());

		 if(dataReady == 0){
				 //osDelay(DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_64HZ]);
				 //DBG_PRINT("\r\n 2-frame1 delay  = %d ms > %d\r\n",
				 //  DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_64HZ],frameData_cnt);
				 osDelay(CONVERT_WAIT_TIME/5);
				 //DBG_PRINT("\r\n 2-frame1 delay  = %d ms > %d\r\n",CONVERT_WAIT_TIME/5,frameData_cnt);
		 }

		frameData_cnt++;
	 }

	 TimeCnt1 = xTaskGetTickCount();

	 if(dataReady != 0)
	 {
		 EEaddress16 = MLX90640_RAMAddrstart;
		 EEaddr[0]=(INT8U)(EEaddress16 >> 8);
		 EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
		 error = drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pMLX32x32_READ_INT8U_buf
			 ,MLX90640_RAM_AddrRead*2,MXL_I2C_RESTART_MODE); // 多筆讀取 RAM

		 //DBG_PRINT("multi_read return = %d \r\n",error);// return data length , if error = -1

		 for(cnt=0; cnt < MLX90640_RAM_AddrRead; cnt++)
		 {
			 i = cnt << 1;
			 *(pMLX32x32_frameData_INT16U_buf+cnt) = (INT16U)*(pMLX32x32_READ_INT8U_buf+i) *256
				 + (INT16U)*(pMLX32x32_READ_INT8U_buf+i+1);
		 }

		 pMLX_TH32x24_Para->frameData[833] = statusRegister & 0x0001;	 // 紀錄 目前是 subpage ?
		 //DBG_PRINT(" 3b. [subpage = %d] \r\n",pMLX_TH32x24_Para->frameData[833]);

		 pMLX_TH32x24_Para->frameData[832] = SETcontrolRegister1;

	 //
	 // in Status register - 0x8000
	 // Bit3:b
	 //  0:No new data is available in RAM (must be reset be the customer)
	 // 0x0030 :
	 // Bit4:
	 // 1: Data in RAM overwrite is enabled
	 // Bit5:
	 // 1: In step mode - start of measurement
	 // 	 (set by the customer and cleared once the measurement is done)
	 //

		 error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);

	 }


	TimeCnt2 = xTaskGetTickCount();


	 MLX90640_GetVdd();

#if DEBUG_MLX_MSG_OUT
	 DBG_PRINT(" pMLX_TH32x24_Para->MLX_TH32x24_vdd =  %f (t=%d) \r\n"
		 ,pMLX_TH32x24_Para->MLX_TH32x24_vdd,xTaskGetTickCount());
#endif

	 MLX90640_GetTa();

#if DEBUG_MLX_MSG_OUT
	 DBG_PRINT(" pMLX_TH32x24_Para->MLX_TH32x24_ta = 0x%04x (%f) \r\n"
		 ,pMLX_TH32x24_Para->MLX_TH32x24_ta,pMLX_TH32x24_Para->MLX_TH32x24_ta);
#endif

	if(format == TEMPout){
	 MLX90640_CalculateTo(emissivity_byUser,(pMLX_TH32x24_Para->MLX_TH32x24_ta - TA_SHIFT));
	 //DBG_PRINT(" frame %d MLX90640_CalculateTo->2  \r\n", pMLX_TH32x24_Para->frameData[833]);
#if DEBUG_TMP_READ_OUT2
	 for(pixelNumber=0 ; pixelNumber<MLX_Pixel ; pixelNumber++){

	 if((pixelNumber%32 == 0) && (pixelNumber != 0)) DBG_PRINT("\r\n");
		 DBG_PRINT("(%d)",pMLX_TH32x24_Para->result[pixelNumber]);

			 }
#endif
	}
	if(format == IMGout){
	 MLX90640_GetImage();
	 //DBG_PRINT(" frame %d MLX90640_GetImage->2 End[t=%d] \r\n",
	 //  pMLX_TH32x24_Para->frameData[833],xTaskGetTickCount()-TimeCnt1);
#if DEBUG_TMP_READ_OUT2
	 for(pixelNumber=0 ; pixelNumber<MLX_Pixel ; pixelNumber++){

	 if((pixelNumber%32 == 0) && (pixelNumber != 0)) DBG_PRINT("\r\n");
		 //DBG_PRINT("(%d)",pMLX_TH32x24_Para->result[pixelNumber]);

		DBG_PRINT("[%f]",pMLX_TH32x24_Para->result_image[pixelNumber]);

	}
#endif
	}

#endif

	TimeCnt3 = xTaskGetTickCount();

#if 0
	//if (pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt % 10 == 0){

		DBG_PRINT("[CalulateData-> frame%d W = %d ms , frame0 R = %d ms , frame0 C = %d ms ] \r\n"
			,pMLX_TH32x24_Para->frameData[833],TimeCnt2b-TimeCnt2a,TimeCnt1a-TimeCnt2b,TimeCnt1b-TimeCnt1a);
		//DBG_PRINT("[ frame1 W = %d ms , frame1 R = %d ms , frame1 C = %d ms ] \r\n"
		//	,TimeCnt1-TimeCnt1b,TimeCnt2-TimeCnt1,TimeCnt3-TimeCnt2);
	//}

#endif

 }

#endif

/*
void ScaleUpForH_Mark(void)

{
	//
			// scale function for HighTMark (x 3) / ScaleUp_3
			//
			tmp_i2 = 0;
			tmp_start =0;
			RGB888TmpValue = 0xffff0000;
			for(cellNum=0;cellNum<MLX_Pixel;cellNum++){
					//RGB888TmpValue = *(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0 + cellNum);
					tmp_i2 = cellNum % 32;

				if((( cellNum % 32) == 0 ) && ( cellNum != 0 ))
					{
						tmp_start = tmp_i;
						//DBG_PRINT("tmp_start = %d,tmp_i2 = %d , cellNum = %d	\r\n",tmp_start,tmp_i2,cellNum);
					}

				for (tmp_i = tmp_start + tmp_i2 *ScaleUp_3 ; tmp_i < tmp_start + (tmp_i2 *ScaleUp_3)+ ScaleUp_3; ++tmp_i)
					{
					*(pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0 + tmp_i )
							=RGB888TmpValue;
					}
				for(tmp_i = tmp_start + ((tmp_i2+rowNumEnd_32)*ScaleUp_3) ; tmp_i < tmp_start +((tmp_i2+rowNumEnd_32)*ScaleUp_3)+ScaleUp_3 ; tmp_i++ )
					{

					*(pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0 + tmp_i )
							=RGB888TmpValue;
					}
				for(tmp_i = tmp_start + ((tmp_i2+rowNumEnd_32*2)*ScaleUp_3) ; tmp_i < tmp_start +((tmp_i2+rowNumEnd_32*2)*ScaleUp_3)+ScaleUp_3 ; tmp_i++ )
					{

					*(pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0 + tmp_i )
							=RGB888TmpValue;
					}

			}




}

*/



void FindMax_ColorAssign(void){

	INT16U  cellNum;
	float	 ImgObject;
 	float	*pMLX_TH32x24_ImgOutput_INT32S_buf0;

 	float	*pMLX_TH32x24_LowPassImgOutput_INT32S_buf0;

	INT8U  *pMLX_TH32x24_Grayframe_INT8U_buf0;

	INT32U  *pMLX_TH32x24_RGB888Colorframe_INT32U_buf0,*pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0,
			*pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0;

	INT16U  *pMLX_TH32x24_Colorframe_INT16U_buf0;
	INT16U	TminTable,TmaxTable;

	float	TmaxOverZero,TminOverZero,TmaxUnderZero,TminUnderZero;
	float	TmaxOverZeroTable,TminOverZeroTable,TmaxUnderZeroTable,TminUnderZeroTable;
	float	Tpoint3;


	INT16U	 OverZ_Tmin_number,OverZ_Tmax_number,OverZ_TminTable_number; //,OverZ_TmaxTable_number;
	INT16U	 UnderZ_Tmin_number,UnderZ_Tmax_number,UnderZ_TminTable_number; //,UnderZ_TmaxTable_number;
	INT32U	tmp_i,tmp_i2;

	INT16U	tmp_start;


	INT8U	GrayTmpValue; // 	INT16U	TminTable,TmaxTable;

	INT32U 	RGB888TmpValue; //
	INT16S	GrayValueAmp;

	INT8U	TmpTbInd;

	INT16U OverZ_TmaxTable_number,UnderZ_TmaxTable_number;

	//INT32U UnderZeroDiff_value,OverZeroDiff_value;


	INT16S	 TmpOutObject;
	INT16S	 TmpMax,TmpMin,RedMarkTmpMax;

	INT16U	 TmpMinTable_number,TmpMax_Mark_Table_number,TmpMaxTable_number;
	INT16S	 TmpMax_avg;

	//INT16U	 TmpMaxTable_Mark_number[5];

	INT16U  *pMLX_TH32x24_TmpOutput_INT16U_buf0;

	INT16U	SingleColorTmpValue,SingleColorShiftValue;



	pMLX_TH32x24_ImgOutput_INT32S_buf0 = pMLX_TH32x24_Para->result_image; // image format ?
		pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 = pMLX_TH32x24_Para->result_LOWPASS_image;

	pMLX_TH32x24_TmpOutput_INT16U_buf0 = pMLX_TH32x24_Para->result;

	pMLX_TH32x24_Grayframe_INT8U_buf0 =
				(INT8U*)pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFrame_addr;

	pMLX_TH32x24_RGB888Colorframe_INT32U_buf0=
				(INT32U*)pMLX_TH32x24_Para->MLX_TH32x24_RGB888_OutputFrame_addr;

	pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0=
				(INT32U*)pMLX_TH32x24_Para->MLX_TH32x24_RGB888_HighTMarkScaleUp_OutputFrame_addr;


	pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0=
					(INT32U*)pMLX_TH32x24_Para->MLX_TH32x24_RGB888_HighTMark_OutputFrame_addr;


	pMLX_TH32x24_Colorframe_INT16U_buf0 =
				(INT16U*)pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr;


	Tpoint3 = pMLX_TH32x24_Para->MLX_TH32x24_GRAY_Tpoint3;

	TmaxUnderZeroTable = pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmaxUnderZeroTable;

	//DBG_PRINT("cp Tpoint3=[%f] ",Tpoint3);

	#if LowPassFiliter_ON

	if (pMLX_TH32x24_Para->MLX_TH32x24_LowPass_SET == 1)
		{


	//	|1 1 1|
	//	|1 1 1|	Low Pass filiter 3*3
	//	|1 1 1|

	gp_memcpy((INT8S *)(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0),		// 先 copy 原來資料 
			(INT8S *)pMLX_TH32x24_ImgOutput_INT32S_buf0,MLX_Pixel*IMAGE_DATA_INT32S_SIZE);
	//
	//	cellNum (33 -> 734) [main body]
	//

	for(cellNum=33;cellNum<735;cellNum++){

		if( (*(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0) &&
			( cellNum%rowNumEnd_32 != 0 ) &&
			( cellNum%rowNumEnd_32 != 31 )	)// >0 強信號不被平均, 邊緣不處理 
			{
				*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
				 (
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 -1) * _LowPassFiliter_Factor[0][0] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 +1) * _LowPassFiliter_Factor[0][2] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -1) * _LowPassFiliter_Factor[1][0] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +1) * _LowPassFiliter_Factor[1][2] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 -1) * _LowPassFiliter_Factor[2][0] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32   ) * _LowPassFiliter_Factor[2][1] +
				 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 +1) * _LowPassFiliter_Factor[2][2]
				 )/9;

			}

		}

	//
	//	(colum=0) [LP filiter at edge]
	//
	cellNum = 0;
	if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0)
		{
			*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
		 	(
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 +1) * _LowPassFiliter_Factor[0][2] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +1) * _LowPassFiliter_Factor[1][2]
			 )/4;
		}

	cellNum = 736;
	if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0)
		{
			*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
		 	(
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 +1) * _LowPassFiliter_Factor[0][2] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +1) * _LowPassFiliter_Factor[1][2]
			 )/4;
		}

	for(cellNum = 32;cellNum < 705;cellNum = cellNum + rowNumEnd_32){

		if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0) 	// >0 disable high energy
			{
			*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
			(
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 +1) * _LowPassFiliter_Factor[0][2] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +1) * _LowPassFiliter_Factor[1][2] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 +1) * _LowPassFiliter_Factor[0][2]
			 )/6;
			}

		}


	//
	//	(colum=31) [LP filiter at edge]
	//

	cellNum = 31;
	if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0)	// >0 disable high energy
		{
		*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
		(
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 -1) * _LowPassFiliter_Factor[0][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -1) * _LowPassFiliter_Factor[1][2]
		)/4;
		}

	cellNum = 767;
	if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0)	// >0 disable high energy
		{
		*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
	 	(
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 -1) * _LowPassFiliter_Factor[0][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -1) * _LowPassFiliter_Factor[1][2]
		 )/4;
		}

	for(cellNum = 63;cellNum < 736;cellNum = cellNum + rowNumEnd_32){

		if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0) 	// >0 disable high energy
			{

			*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
		 	(
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 -1) * _LowPassFiliter_Factor[0][2] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -1) * _LowPassFiliter_Factor[1][2] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32   ) * _LowPassFiliter_Factor[0][1] +
			 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 -1) * _LowPassFiliter_Factor[0][2]
			 )/6;

			}

		}



	//
	//	(row=0) [LP filiter at edge]
	//
	for(cellNum = 1;cellNum < 31;cellNum = cellNum + 1)
	{
		if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0)	// >0 disable high energy
		{
		*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
		(
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -1) * _LowPassFiliter_Factor[1][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +1) * _LowPassFiliter_Factor[1][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32	) * _LowPassFiliter_Factor[0][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 +1) * _LowPassFiliter_Factor[0][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +rowNumEnd_32 -1) * _LowPassFiliter_Factor[0][2]
		 )/6;
		}
	}

	//
	//	(row=23) [LP filiter at edge]
	//
	for(cellNum = 737;cellNum < 767;cellNum = cellNum + 1)
	{
		if( *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum) < 0)	// >0 disable high energy
		{
		*(pMLX_TH32x24_LowPassImgOutput_INT32S_buf0 + cellNum) =
		(
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -1) * _LowPassFiliter_Factor[1][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum   ) * _LowPassFiliter_Factor[1][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum +1) * _LowPassFiliter_Factor[1][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32	) * _LowPassFiliter_Factor[0][1] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 +1) * _LowPassFiliter_Factor[0][2] +
		 *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum -rowNumEnd_32 -1) * _LowPassFiliter_Factor[0][2]
		 )/6;
		}
	}

	gp_memcpy((INT8S *)(pMLX_TH32x24_ImgOutput_INT32S_buf0),		//  copy LowPassFiliter to ImgOutput
			(INT8S *)pMLX_TH32x24_LowPassImgOutput_INT32S_buf0,MLX_Pixel*IMAGE_DATA_INT32S_SIZE);

		}
	#endif

	//gp_memset((INT8S *)pMLX_TH32x24_RGB888Colorframe_INT32U_buf0,
	//	0xffffffff,MLX_Pixel*IMAGE_DATA_INT32S_SIZE); // 白色 

	//0xffff0099,MLX_Pixel*IMAGE_DATA_INT32S_SIZE); // 白色 

	TmpMax = 0;
	TmpMin = 0;
	RedMarkTmpMax = 0;

	if (pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag == 1){
			TmpMax = *(pMLX_TH32x24_TmpOutput_INT16U_buf0 + MLX_ZONE1_1stPixel);
			TmpMin = *(pMLX_TH32x24_TmpOutput_INT16U_buf0 + MLX_ZONE1_1stPixel);
			RedMarkTmpMax = *(pMLX_TH32x24_TmpOutput_INT16U_buf0 + MLX_ZONE1_1stPixel);

			//DBG_PRINT("M=%d m=%d\r\n ",TmpMax,TmpMin);
			}
	for(cellNum=0;cellNum<MLX_Pixel;cellNum++){
			 ImgObject = *(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum);

			 TmpOutObject = *(pMLX_TH32x24_TmpOutput_INT16U_buf0 + cellNum);


	 //
	 // find Tmax Tmin at CORE area
	 //
	 if(cellNum == (CORE_AREA_limit*32+CORE_AREA_limit)){
		 if(ImgObject < 0 ){
			 TmaxOverZero = 0;
			 TminOverZero = 388888888;
			 TmaxUnderZero = ImgObject;
			 TminUnderZero = ImgObject;
			 }
		 else{
			 TmaxOverZero = ImgObject;
			 TminOverZero = ImgObject;
			 TmaxUnderZero = -388888888;
			 TminUnderZero = 0;
			 }
		 }

	 if(((cellNum/32) >CORE_AREA_limit)&&((cellNum/32)<(SENSOR_AREA_HIGH -CORE_AREA_limit))
			 &&((cellNum%32)>CORE_AREA_limit)&&((cellNum%32)<(SENSOR_AREA_WIDTH -CORE_AREA_limit))){
		 if (ImgObject >= 0)
			 {
			 if(TminOverZero > ImgObject){
				 TminOverZero = ImgObject;
				 tmp_i = cellNum/rowNumEnd_32;
				 tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				 OverZ_Tmin_number=tmp_i2;} //(左右 對調 後 正確位置) 

			 if(TmaxOverZero < ImgObject){
				 TmaxOverZero = ImgObject;
				 tmp_i = cellNum/rowNumEnd_32;
				 tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				 OverZ_Tmax_number=tmp_i2;} //(左右 對調 後 正確位置) 
			 }

		 else{
			 if(TminUnderZero > ImgObject){
				 TminUnderZero = ImgObject;
				 tmp_i = cellNum/rowNumEnd_32;
				 tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				 UnderZ_Tmin_number=tmp_i2;} //(左右 對調 後 正確位置) 
			 if(TmaxUnderZero < ImgObject){
				 TmaxUnderZero = ImgObject;
				 tmp_i = cellNum/rowNumEnd_32;
				 tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				 UnderZ_Tmax_number=tmp_i2;} //(左右 對調 後 正確位置) 
			 }

		if (pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag == 1)
			{
			if((TmpMin > TmpOutObject) && (TmpOutObject != -1 ) ) { // 因 沒計算 設為 -1
				TmpMin = TmpOutObject;
				 tmp_i = cellNum/rowNumEnd_32;
				 tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				 TmpMinTable_number=tmp_i2; //(左右 對調 後 正確位置) 
				}
			if(TmpOutObject > (RedMarkTmpMax+20)){	// over 2 C for Red Mark
			//if(TmpOutObject > TmpMax){
				RedMarkTmpMax = TmpOutObject;
				 tmp_i = cellNum/rowNumEnd_32;
				 tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				 TmpMax_Mark_Table_number=tmp_i2; //(左右 對調 後 正確位置) 

				}
			if(TmpOutObject > TmpMax){	// for display
				TmpMax = TmpOutObject;
				tmp_i = cellNum/rowNumEnd_32;
				tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				TmpMaxTable_number=tmp_i2; //(左右 對調 後 正確位置) 
				}
			}


	 }


	 //GrayTmpValue = 0x00; // 0x00 ~ 0x7F low temp ,0x80 ~ 0xff high temp
	 if(firstRun == 0){		 // initial setting
		 if(ImgObject > 0)
			 GrayTmpValue =0x1f;
		 else
			 GrayTmpValue = 0x00;	 // TRANSPARENT_COLOR;

		//DBG_PRINT("firstRun == 0\r\n");
	}

	else{
	 if(ImgObject >= 0)  GrayTmpValue =0xff; // 最熱 全亮 
	 else{

		 if (ImgObject <= Tpoint3)
			 {
			 GrayTmpValue =pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL;
			 }
		 else {
		 	// auto Autoscale
			 //TmpTbInd =(INT8U)(((ImgObject - TminUnderZeroTable)*255)/(TmaxUnderZeroTable - TminUnderZeroTable)) ;
			 TmpTbInd =(INT8U)(  ((ImgObject - Tpoint3)*(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL -pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL))/
					 (TmaxUnderZeroTable - Tpoint3));
			 TmpTbInd = TmpTbInd + pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL ;

			 if(TmpTbInd> 250 )  TmpTbInd=255;

			 else if(TmpTbInd <= pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL)
					 TmpTbInd = pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL;

			 GrayTmpValue = TmpTbInd ;
		 }
		 }
	 }



		//
		// mirror function (左右 對調) 
		//
		tmp_i = cellNum/rowNumEnd_32;
		tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));



			//
			//	gray sacle modify ?
			//

			if ((GrayTmpValue > pMLX_TH32x24_Para->MLX_TH32x24_GRAY_AMP_START )
				&& (pMLX_TH32x24_Para->MLX_TH32x24_GRAY_AMP_SCALE != 0))
			{
				GrayValueAmp = GrayTmpValue;

				//DBG_PRINT("[Gray=%d",GrayTmpValue);
				GrayValueAmp = (GrayValueAmp - pMLX_TH32x24_Para->MLX_TH32x24_GRAY_AMP_START)*
				pMLX_TH32x24_Para->MLX_TH32x24_GRAY_AMP_SCALE +	pMLX_TH32x24_Para->MLX_TH32x24_GRAY_AMP_START ;
				//DBG_PRINT("-Amp=%d]",GrayValueAmp);
				if(GrayValueAmp > 255) GrayTmpValue = 255;
				else GrayTmpValue =(INT8U)GrayValueAmp;

				//DBG_PRINT("Gray=%d  ",GrayTmpValue);
			}
			*(pMLX_TH32x24_Grayframe_INT8U_buf0+ tmp_i2)
			= GrayTmpValue;

			#if 0	// 背景藍色 
			RGB888TmpValue =(INT32U)GrayTmpValue<<24 | (INT32U)GrayTmpValue<<8 | (INT32U)GrayTmpValue<<16 | 0x000000ff; // 籃底色 
			// FF’ (255) 表示 alpha 係數。‘00’ 表示紅色的數量，‘FF’ 表示綠色的數量 ‘00’ 表示藍色的數量. 
			#else	// 灰階 
			RGB888TmpValue =(INT32U)GrayTmpValue | (INT32U)GrayTmpValue<<8 | (INT32U)GrayTmpValue<<16 | 0xff000000;	// 灰階 
			#endif

			*(pMLX_TH32x24_RGB888Colorframe_INT32U_buf0+ tmp_i2)
			= RGB888TmpValue;


	}

	if ((pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag == 1)
		&& (pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor < NO_SIGNAL_FACTOR))
	{


		if((TmpMin != 0) && (TmpMax != 0))
			{

			//DBG_PRINT(" Max - 1p = %d\r\n",TmpMax);
			pMLX_TH32x24_Para->MLX_TH32x24_TmpMin = TmpMin;
			pMLX_TH32x24_Para->MLX_TH32x24_TmpMax = TmpMax;
			pMLX_TH32x24_Para->MLX_TH32x24_RedMarkTmpMax=RedMarkTmpMax;

			OvAlertVal_SUM = 0;

			for (tmp_i2 = 0; tmp_i2 < (ALERT_BUF_LEN - 1); ++tmp_i2)
				{
				pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[tmp_i2] =
					pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[tmp_i2 + 1];
				OvAlertVal_SUM = OvAlertVal_SUM +
					pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[tmp_i2 + 1];
				}

			pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[ALERT_BUF_LEN-1] =
				pMLX_TH32x24_Para->MLX_TH32x24_TmpMax - pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet 	;

			OvAlertVal_SUM = OvAlertVal_SUM + pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[ALERT_BUF_LEN-1];

			if (OvAlertVal_SUM > pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET)
				{
				pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_ON_FLAG = 1;
				pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_cnt = 0;
				gpio_write_io(TH_ALERT_A14_PAD, DATA_HIGH);
				}
			else
				{
				if (pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_cnt > 250)
					{
					gpio_write_io(TH_ALERT_A14_PAD, DATA_LOW);
					pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_ON_FLAG = 0;
					}
				}


			/*
			DBG_PRINT("\r\n SUM = %d \r\n ",OvAlertVal_SUM);
			for (tmp_i2 = 0; tmp_i2 < ALERT_BUF_LEN; ++tmp_i2)
				{
				DBG_PRINT(" %d ",
				pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[tmp_i2]);
				}
			DBG_PRINT("\r\n SUM = %d \r\n ",OvAlertVal_SUM);
			*/

			}

		pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag = 0;	//
		pMLX_TH32x24_Para->MLX_TH32x24_disp_Time_Flag = 1;

		//DBG_PRINT("M_=%d \r\n ",TmpMaxTable_number);
		//DBG_PRINT("TmpMin=%d [%d] , TmpMax=%d [%d] \r\n",TmpMin,TmpMinTable_number,TmpMax,TmpMaxTable_number);

		gp_memset((INT8S *)pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0,
		0x00,MLX_Pixel*IMAGE_DATA_INT32S_SIZE); // 透明度0 ,RGB 白色 
		//0xffff0000,MLX_Pixel*IMAGE_DATA_INT32S_SIZE); // ARGB 紅色 

		/* HighTMark is Cross Red
		*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number)
			= 0xffff0000;
		if(((TmpMax_Mark_Table_number+1) < MLX_Pixel) && ((TmpMax_Mark_Table_number+1) %rowNumEnd_32 != 0))
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number+1)
				= 0xffff0000;
		if(((TmpMax_Mark_Table_number-1) >0) && ((TmpMax_Mark_Table_number-1) %rowNumEnd_32 != 31))
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number-1)
				= 0xffff0000;
		if((TmpMax_Mark_Table_number+rowNumEnd_32) < MLX_Pixel)
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number+rowNumEnd_32)
				= 0xffff0000;
		if((TmpMax_Mark_Table_number-rowNumEnd_32) > 0)
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number-rowNumEnd_32)
			= 0xffff0000;
		*/


		for (tmp_i2 = 103; tmp_i2 < 121; ++tmp_i2)	// upper line
			{
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ tmp_i2)
			= 0x20008f00;
			}
		for (tmp_i2 = 647; tmp_i2 < 665; ++tmp_i2)	// lower line
			{
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ tmp_i2)
			= 0x20008f00;
			}
		for (tmp_i2 = 135; tmp_i2 < 616; tmp_i2=tmp_i2+SENSOR_AREA_WIDTH)	// left line
			{
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ tmp_i2)
			= 0x20008f00;
			}
		for (tmp_i2 = 152; tmp_i2 < 633; tmp_i2=tmp_i2+SENSOR_AREA_WIDTH)	// right line
			{
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ tmp_i2)
			= 0x20008f00;
			}



		// HighTMark is Squre Red
		// ZONE 1 only
		//*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number)
		//	= 0xffff0000;

		if((TmpMax_Mark_Table_number+1)%rowNumEnd_32 < (SENSOR_AREA_WIDTH -TMPZONE1_HOR_AREA_limit) ) // to right < 24
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number+1)
				= 0xffff0000;
		if((TmpMax_Mark_Table_number-1)%rowNumEnd_32 >= TMPZONE1_HOR_AREA_limit ) // to left >= 8
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number-1)
				= 0xffff0000;

		if((TmpMax_Mark_Table_number-rowNumEnd_32)/rowNumEnd_32 >= TMPZONE1_VER_AREA_limit) // to upper >=4
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number-rowNumEnd_32)
			= 0xffff0000;
		if( ((TmpMax_Mark_Table_number-rowNumEnd_32+1)/rowNumEnd_32 >= TMPZONE1_VER_AREA_limit) // to upper >=4
			&& ((TmpMax_Mark_Table_number-rowNumEnd_32+1)%rowNumEnd_32 < (SENSOR_AREA_WIDTH -TMPZONE1_HOR_AREA_limit)) ) // to right < 24
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number-rowNumEnd_32+1)
			= 0xffff0000;
		if(((TmpMax_Mark_Table_number-rowNumEnd_32-1)/rowNumEnd_32 >= TMPZONE1_VER_AREA_limit) // to upper >=4
			&& ((TmpMax_Mark_Table_number-rowNumEnd_32-1)%rowNumEnd_32 >= TMPZONE1_HOR_AREA_limit)) // to left >= 8
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number-rowNumEnd_32-1)
			= 0xffff0000;

		if((TmpMax_Mark_Table_number+rowNumEnd_32)/rowNumEnd_32 < (SENSOR_AREA_HIGH -TMPZONE1_VER_AREA_limit) ) // to lower <=20
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number+rowNumEnd_32)
				= 0xffff0000;
		if(((TmpMax_Mark_Table_number+rowNumEnd_32+1)/rowNumEnd_32 < (SENSOR_AREA_HIGH -TMPZONE1_VER_AREA_limit)) // to lower <=20
			&& ((TmpMax_Mark_Table_number+rowNumEnd_32+1)%rowNumEnd_32 < (SENSOR_AREA_WIDTH -TMPZONE1_HOR_AREA_limit) )) // to right < 24
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number+rowNumEnd_32+1)
				= 0xffff0000;
		if(((TmpMax_Mark_Table_number+rowNumEnd_32-1)/rowNumEnd_32 < (SENSOR_AREA_HIGH -TMPZONE1_VER_AREA_limit)) // to lower <=20
			&& ((TmpMax_Mark_Table_number+rowNumEnd_32-1)%rowNumEnd_32 >= TMPZONE1_HOR_AREA_limit)) // to left >= 8
			*(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0+ TmpMax_Mark_Table_number+rowNumEnd_32-1)
				= 0xffff0000;

		//ScaleUpForH_Mark();
		//
		// scale function for HighTMark (x 3) / ScaleUp_3
		//
		tmp_i2 = 0;
		tmp_start =0;
		for(cellNum=0;cellNum<MLX_Pixel;cellNum++){
				RGB888TmpValue = *(pMLX_TH32x24_RGB888_HighTMark_INT32U_buf0 + cellNum);
				tmp_i2 = cellNum % 32;

			if((( cellNum % 32) == 0 ) && ( cellNum != 0 ))
				{
					tmp_start = tmp_i;
					//DBG_PRINT("tmp_start = %d,tmp_i2 = %d , cellNum = %d  \r\n",tmp_start,tmp_i2,cellNum);
				}

			for (tmp_i = tmp_start + tmp_i2 *ScaleUp_3 ; tmp_i < tmp_start + (tmp_i2 *ScaleUp_3)+ ScaleUp_3; ++tmp_i)
				{
				*(pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0 + tmp_i )
						=RGB888TmpValue;
				}
			for(tmp_i = tmp_start + ((tmp_i2+rowNumEnd_32)*ScaleUp_3) ; tmp_i < tmp_start +((tmp_i2+rowNumEnd_32)*ScaleUp_3)+ScaleUp_3 ; tmp_i++ )
				{

				*(pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0 + tmp_i )
						=RGB888TmpValue;
				}
			for(tmp_i = tmp_start + ((tmp_i2+rowNumEnd_32*2)*ScaleUp_3) ; tmp_i < tmp_start +((tmp_i2+rowNumEnd_32*2)*ScaleUp_3)+ScaleUp_3 ; tmp_i++ )
				{

				*(pMLX_TH32x24_RGB888_HighTMarkScaleUp_INT32U_buf0 + tmp_i )
						=RGB888TmpValue;
				}

		}




		}


	//
	// set Tmax/Tmin for next frame, NOT this frame
	//
	TminOverZeroTable=TminOverZero; OverZ_TminTable_number=OverZ_Tmin_number;
	TmaxOverZeroTable=TmaxOverZero; OverZ_TmaxTable_number=OverZ_Tmax_number;
	TminUnderZeroTable = TminUnderZero; UnderZ_TminTable_number=UnderZ_Tmin_number;
	TmaxUnderZeroTable = TmaxUnderZero; UnderZ_TmaxTable_number=UnderZ_Tmax_number;
	pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmaxUnderZeroTable = TmaxUnderZeroTable;

		UnderZeroDiff_value = (INT32U)((TmaxUnderZeroTable-TminUnderZeroTable)/1000000);
		if( UnderZeroDiff_value < 0 ) UnderZeroDiff_value = 0; // too small & too large ?

		OverZeroDiff_value = (INT32U)((TmaxOverZeroTable-TminOverZeroTable)/1000000);
		if( OverZeroDiff_value < 0 ) OverZeroDiff_value = 0;

	//if(pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt%20 == 0)
	//	DBG_PRINT("OverZeroDiff=[%f]/%d *10^6 , UnderZeroDiff=[%f]/%d *10^6 \r\n",
	//			(TmaxOverZeroTable-TminOverZeroTable),OverZeroDiff_value,
	//			(TmaxUnderZeroTable-TminUnderZeroTable),UnderZeroDiff_value);




	 // MLX_GrayOutputFactor_Ary[10]={8,8,7,7,6,5,4,3,3,3,3};
				if (OverZeroDiff_value > 0)
				{
					pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor = MLX_GrayOutputFactor_Ary[19];
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL = MLX_Gray_MAX_val_Ary[19]; // 255 會有計算 error
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL = MLX_Gray_START_val_Ary[19];
					pMLX_TH32x24_Para->TmpTbInd_buf_Enable = 0;

					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmpTbInd = 19;
					TmpTbInd = 0;
					//DBG_PRINT("OverZeroDiff_value > 0  \r\n");
				}
				else if (OverZeroDiff_value <= 0)
				{

					TmpTbInd =(INT8U)(UnderZeroDiff_value/50);
					if (TmpTbInd > 19){
						TmpTbInd = 19;
						//DBG_PRINT("TmpTbInd = %d \r\n",TmpTbInd);
						}
		#if TmpTbInd_NO_BUF

					pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor = MLX_GrayOutputFactor_Ary[TmpTbInd];
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL = MLX_Gray_MAX_val_Ary[TmpTbInd];
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL = MLX_Gray_START_val_Ary[TmpTbInd];

					DBG_PRINT("No buff OverZeroDiff_value < 0 , TmpTbInd = %d \r\n",TmpTbInd);


		#else

					if (pMLX_TH32x24_Para->TmpTbInd_buf_Enable == 0)
					{
						for (tmp_i = 0; tmp_i < IMG_GRAY_IND_buf_len; ++tmp_i)
						{
							pMLX_TH32x24_Para->TmpTbInd_buf[tmp_i] = TmpTbInd;
						}
						pMLX_TH32x24_Para->TmpTbInd_buf_Enable = 1;
					}
					else
					{
						for (tmp_i = 0; tmp_i < (IMG_GRAY_IND_buf_len - 1); ++tmp_i)
						{
							pMLX_TH32x24_Para->TmpTbInd_buf[tmp_i] = pMLX_TH32x24_Para->TmpTbInd_buf[tmp_i + 1];
						}
						pMLX_TH32x24_Para->TmpTbInd_buf[IMG_GRAY_IND_buf_len - 1] = TmpTbInd;
						for (tmp_i= 0; tmp_i < (IMG_GRAY_IND_buf_len - 1); ++tmp_i)
						{
							TmpTbInd = TmpTbInd + pMLX_TH32x24_Para->TmpTbInd_buf[tmp_i];
						}
						TmpTbInd = TmpTbInd / IMG_GRAY_IND_buf_len;
					}

					if (TmpTbInd > 19) TmpTbInd = 19;
					//DBG_PRINT("Ind=%d",TmpTbInd);
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmpTbInd = TmpTbInd;
					pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor = MLX_GrayOutputFactor_Ary[TmpTbInd];
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL = MLX_Gray_MAX_val_Ary[TmpTbInd];
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL = MLX_Gray_START_val_Ary[TmpTbInd];
#endif

				}





#if 0
		if(pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt%100 == 0){
			DBG_PRINT("OverZeroDiff=[%f]/%d *10^6 , UnderZeroDiff=[%f]/%d *10^6 , Ta = %f , Vdd = %f \r\n",
					(TmaxOverZeroTable-TminOverZeroTable),OverZeroDiff_value,
					(TmaxUnderZeroTable-TminUnderZeroTable),UnderZeroDiff_value,
					pMLX_TH32x24_Para->MLX_TH32x24_ta,pMLX_TH32x24_Para->MLX_TH32x24_vdd );

			DBG_PRINT("GrayOutputFactor=%d , GRAY_MAX_VAL=%d ,  GRAY_START_VAL=%d \r\n",
					pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor,
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL,
					pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL);
			}
#endif



		pMLX_TH32x24_Para->MLX_TH32x24_GRAY_Tpoint3 =
			((pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor * TmaxUnderZeroTable )
						+ (10-pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor) * TminUnderZeroTable)/10; // 30% 以下 0值 
		//DBG_PRINT("Tpoint3=[%f] ",pMLX_TH32x24_Para->MLX_TH32x24_GRAY_Tpoint3);

}




 void MLX_TH32x24_start_timer_isr(void)
{
	INT8U err;
	INT32U frame;

	pMLX_TH32x24_Para->MLX_TH32x24_Time_cnt++;

	if(pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_ON_FLAG == 1)
		pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_cnt++;


	if (pMLX_TH32x24_Para->MLX_TH32x24_Time_cnt == 75)
	//if (pMLX_TH32x24_Para->MLX_TH32x24_Time_cnt == 1725 )
		{
			if (pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag == 0)
				{
				pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag = 1;	// 開始計算溫度 
				}
			pMLX_TH32x24_Para->MLX_TH32x24_Time_cnt = 0;
		}


	if( pMLX_TH32x24_Para->MLX_TH32x24_InitReadEE_startON == 1 )
		pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt ++;

	//if ( pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt > 10 ){	// per sec
	//if (( pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt > 5 )&&(pMLX_TH32x24_Para->MLX_TH32x24_sample_startON == 0)) {	// per 500ms
	//if (( pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt > 2 )&&(pMLX_TH32x24_Para->MLX_TH32x24_sample_startON == 0)) {	// per 200ms

	//if ( ( pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt%2 == 0 )
	//	&& ( pMLX_TH32x24_Para->MLX_TH32x24_readout_block_startON == 0 ) ) {	// per 20 msec

	if ( pMLX_TH32x24_Para->MLX_TH32x24_readout_block_startON == 0 ) {	// per 2 msec

		//DBG_PRINT("timer->MLX_TH32x24");

		//frame = avi_encode_get_empty(MLX_TH32x24_SCALERUP_buf_q);

        frame = free_frame_buffer_get(0);
		if(frame == 0)	DBG_PRINT("L->MLX_TH32x24");
		else{

			//DEBUG_MSG("davis -->frame = 0x%x in csi_eof_isr \r\n",frame );
			//avi_encode_post_empty(MLX_TH32x24_task_q,frame);
            csi_frame_buffer_add((INT32U *)frame, 0);
		}

		pMLX_TH32x24_Para->MLX_TH32x24_readout_block_startON = 1;
	}
}

static void Thermal_Preview_PScaler(void)
{
    CHAR *p;
	INT32U i,PrcessBuffer,csi_mode,temp;
	INT32U csiBufferSize,csiBuffer;
	drv_l2_sensor_ops_t *pSencor;
	//drv_l2_thermal_sensor_ops_s *pSencor;
	drv_l2_sensor_info_t *pInfo;

	//csiBufferSize = PRCESS_SRC_WIDTH*PRCESS_SRC_HEIGHT*2;
	csiBufferSize = MLX90640_frameDataSize*2;
    csiBuffer = DUMMY_BUFFER_ADDRESS; // dummy addr ro thermal

	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*DISP_QUEUE_MAX)+64), 32);
    if(PrcessBuffer == 0)
    {
        DBG_PRINT("PrcessBuffer fail\r\n");
        while(1);
    }
	PrcessBuffer = (INT32U)((PrcessBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<DISP_TH_QUEUE_MAX; i++)
	{
		temp = (PrcessBuffer+i*csiBufferSize);
		free_frame_buffer_add((INT32U *)temp,1);
		DBG_PRINT("CSIBuffer:0x%X ->MLX90640_frameDataSize = %d \r\n",temp,MLX90640_frameDataSize);
	}

    // sensor init
	drv_l2_sensor_init();
    pSencor = drv_l2_sensor_get_ops(0);
    DBG_PRINT("SensorName = %s _davis\r\n", pSencor->name);

	for(i=0; i<3; i++) {
		pInfo = pSencor->get_info(i);
		if(pInfo->target_w == SENSOR_SRC_WIDTH && pInfo->target_h == SENSOR_SRC_HEIGHT) {
			temp = i;
			break;
		}
	}

	if(i == 3) {
        DBG_PRINT("get thermal sensor width and height fail\r\n");
		temp = 0;
        while(1);
	}

	DBG_PRINT("sensor target_w = %d , target_h = %d ->davis\r\n",pInfo->target_w ,pInfo->target_h);
	pSencor->init();
	pSencor->stream_start(temp, pInfo->target_w, pInfo->target_h);

}



static void csi_task_entry(void const *parm)
{
    INT32U csi_buf,PscalerBuffer,PscalerBufferSize;
    INT32U event;
    osEvent result;


	INT32S  nRet;
	INT8U	tmp_i;


	INT32U	TimeCnt1,TimeCnt2,TimeCnt3,TimeCnt4;

	INT16U	EEaddress16;
	INT8U 	EEaddr[2],*pEEaddr;

    INT16U 	dataReady,statusRegister,controlRegister1,TmpControlRegister ;
	INT16U	cnt,frameData_cnt;
	INT8U  *pMLX32x32_READ_INT8U_buf;
	INT16U  *pMLX32x32_READ_INT16U_buf,*pMLX32x32_frameData_INT16U_buf;

	INT16U  *pMLX32x32_frameData_csi_INT16U_buf;

	//INT16U  *pMLX_TH32x24_INT16U_avg_buf[AVG_buf_len];


	float  *pMLX_TH32x24_ImgOutput_INT32S_buf0;

	//INT16U pixelNumber;

    float ta,vdd;

	int		error;

    int i;

	drv_l1_i2c_bus_handle_t MXL_handle;



	INT8U	sampleCnt;
	DISP_DEV MLX_disp_dev;

    DBG_PRINT("csi_task_entry start \r\n");
    // csi init
    //mazeTest_Preview_PScaler();
	Thermal_Preview_PScaler();

    // disp size
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&device_h_size, (INT16U *)&device_v_size);

	MLX_disp_dev = DISPLAY_DEVICE;
	if (MLX_disp_dev == DISDEV_TFT)
		{
		DBG_PRINT("\r\n***MLX_disp_dev IS DISDEV_TFT***\r\n");
		}
	else
		DBG_PRINT("\r\n*** MLX_disp_dev IS DISDEV_HDMI ***\r\n");

	PscalerBufferSize = (device_h_size * device_v_size * 2);
	PscalerBuffer = (INT32U) gp_malloc_align(((PscalerBufferSize*C_DEVICE_FRAME_NUM)+64), 32);
    if(PscalerBuffer == 0)
    {
        DBG_PRINT("PscalerBuffer fail\r\n");
        while(1);
    }
	PscalerBuffer = (INT32U)((PscalerBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<C_DEVICE_TH_FRAME_NUM; i++)
	{
		csi_buf = (PscalerBuffer+i*PscalerBufferSize);
		pscaler_frame_buffer_add((INT32U *)csi_buf,1);

		DBG_PRINT("PscalerBuffer:0x%X ->width = %d / high = %d\r\n",csi_buf,device_h_size,device_v_size);
	}
    prcess_state_post(PRCESS_STATE_OK);


	PscalerBufferSize = (device_h_size * device_v_size * 2);
	DBG_PRINT("PscalerBufferSize for HighTMark = %d\r\n",PscalerBufferSize);
	PscalerBuffer = (INT32U) gp_malloc_align(PscalerBufferSize , 32);
	if(PscalerBuffer == 0)
	{
			DBG_PRINT("MLX_TH32x24_HighTMark_display_frame fail\r\n");
        while(1);
	}
	pMLX_TH32x24_Para->MLX_TH32x24_HighTMark_display_frame = PscalerBuffer;
	DBG_PRINT("davis --> MLX_TH32x24_HighTMark_display_frame addr = 0x%x ->width = %d / high = %d\r\n",
		pMLX_TH32x24_Para->MLX_TH32x24_HighTMark_display_frame,device_h_size,device_v_size);




	MXL_handle.devNumber = I2C_1;
	MXL_handle.slaveAddr = MLX90640_SLAVE_ADDR<<1;
	//MXL_handle.clkRate = 1200; // 1Mhz check from OSC. and can run
	MXL_handle.clkRate = 1000; // 900kHz check from OSC.

	pEEaddr = EEaddr;
	pMLX32x32_READ_INT8U_buf = (INT8U*)pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF;

	pMLX32x32_frameData_INT16U_buf = (INT16U*)pMLX_TH32x24_Para->frameData;

	avg_buf_Enable=0;
	firstRun = 0;

	DBG_PRINT("MLX32x24_Para->KvPTAT=%f, MLX32x24_Para->KtPTAT=%f ,MLX32x24_Para->vPTAT25= %d ,MLX32x24_Para->alphaPTAT=%f \r\n",
			pMLX90640_Para->KvPTAT,pMLX90640_Para->KtPTAT,pMLX90640_Para->vPTAT25,pMLX90640_Para->alphaPTAT);

	DBG_PRINT("CalculateTo(emissivity_byUser->%f,tr_byUser = ta - TA_SHIFT->%d ) \r\n",
	emissivity_byUser,TA_SHIFT);


	// controlRegister1 設定成 自動 subpage 0/1
	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
	DBG_PRINT("read controlRegister1 = 0x%04x \r\n",controlRegister1);

	DBG_PRINT("initial setting controlRegister1 = 0x%04x \r\n",pMLX_TH32x24_Para->MLX_TH32x24_InitSet_controlRegister1);

	while(controlRegister1 != pMLX_TH32x24_Para->MLX_TH32x24_InitSet_controlRegister1 ){

	do{
		drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrControlRegister1,pMLX_TH32x24_Para->MLX_TH32x24_InitSet_controlRegister1);
		osDelay(80);
		error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
		DBG_PRINT("*Re - write/read controlRegister1 = 0x%04x \r\n",controlRegister1);
		}while(controlRegister1 != pMLX_TH32x24_Para->MLX_TH32x24_InitSet_controlRegister1 );

	}

	//controlRegister1 = pMLX_TH32x24_Para->MLX_TH32x24_InitSet_controlRegister1;


	// *************************
	#if (FOV_BAB_55 == 1) && (FOV_BAA_110 == 0)
			DBG_PRINT("for [FOV_BAB_55] \r\n");
	#else
			DBG_PRINT("for [FOV_BAA_110 ] \r\n");
	#endif

		pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL = GRAY_GRAYMAX_COLOR;
		DBG_PRINT("MLX_TH32x24_GRAY_MAX_VAL =%d \r\n",pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL);

		pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor = 3;
			DBG_PRINT("MLX_TH32x24_GrayOutputFactor =%d \r\n",pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor);

		pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL = GRAY_START_COLOR;
			DBG_PRINT("MLX_TH32x24_GRAY_START_VAL =%d \r\n",pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL);

		//for(tmp_i=0;tmp_i<IMG_AVG_buf_len;tmp_i++){
		//	pMLX_TH32x24_INT16U_avg_buf[tmp_i]= (INT16U*)pMLX_TH32x24_Para->MLX_TH32x24_avg_buf_addr[tmp_i];
		//}


	Gcnt_lp = 0;

	GlobalTimeCnt1 = xTaskGetTickCount();

    while(1)
    {
        result = osMessageGet(csi_frame_buffer_queue, osWaitForever);
        csi_buf = result.value.v;
        if((result.status != osEventMessage) || !csi_buf) {
            continue;
        }
        //DBG_PRINT("csi_buffer = 0x%x\r\n", csi_buf);
        //DBG_PRINT(".");


	TimeCnt1 = xTaskGetTickCount();
	TimeCnt4 = xTaskGetTickCount()- GlobalTimeCnt1 ;
	GlobalTimeCnt1 = xTaskGetTickCount();

	//DBG_PRINT("c %d- 0x%x\r\n",xTaskGetTickCount(), csi_buf);

	pMLX32x32_frameData_csi_INT16U_buf = (INT16U*)csi_buf;

	// 接收資料 

	//DBG_PRINT("************ GetFrameData frame 0 ************************************** \r\n");

	 dataReady = 0;
	 frameData_cnt=0;
	 while(dataReady == 0)
	 {
		 do{
			 error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrStatus,&statusRegister);
			 //DBG_PRINT("read return-1  = %d \r\n",error);  // return data length , if error = -1
			 //  需要 重新讀取 !! 改成 副程式 檢查 
			 if( error == -1){
				 DBG_PRINT("frame0 can NOT read status error-1 !! delay %d ms - %d\r\n",
				 	DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_SET]/3,frameData_cnt);
				 osDelay(DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_SET]/3);
			 }
		 }while(error == -1);

		 dataReady = statusRegister & 0x0008; // 1 : A new data is available in RAM ?

		 //DBG_PRINT(" 2.statusRegister = 0x%04x, dataReady = 0x%04x,frameData_cnt = %d \r\n",
		 //  statusRegister,dataReady,frameData_cnt);


		 if(dataReady == 0){

			 //osDelay(DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_64HZ]);

			 /*
			 osDelay(CONVERT_WAIT_TIME);
			 DBG_PRINT(" frame0 read NOT RDY = %d ms > %d\r\n",CONVERT_WAIT_TIME,frameData_cnt);
			 */


			 osDelay(DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_SET]/3); // 依據 Hz 決定 osDelay()
			 //DBG_PRINT(" frame0 read NOT RDY = %d ms > %d\r\n",
		     //DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_SET]/3,frameData_cnt);


			 frameData_cnt++;
		 }


		 if(frameData_cnt > DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_SET] ){	 // reset MLX


			 DBG_PRINT("****wait over %d,re-send start of measurement  \r\n",frameData_cnt*DELAYTIME_at_REFRESH_RATE3[MLX90640_REFRESH_RATE_SET]);
			/*
			 error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
			 controlRegister1 = controlRegister1 & MLX90640_SetModeClear ;
			 controlRegister1 = controlRegister1 | MLX90640_SetStepModeSubpageRep ;

			 drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrControlRegister1,controlRegister1);
			 */
			 //DBG_PRINT("(subpage = 0)write controlRegister1 = 0x%04x \r\n",controlRegister1);
			 //osDelay(DELAYTIME_at_REFRESH_RATE[MLX90640_REFRESH_RATE_32HZ]);
			 //  DBG_PRINT("\r\n frameData_cnt> frame0 delay  = %d ms \r\n",
			 // 	 DELAYTIME_at_REFRESH_RATE[MLX90640_REFRESH_RATE_32HZ]);

			 error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);

			 frameData_cnt = 0;
			 }
	 }

	#if DEBUG_IO_OUT
		   gpio_write_io(TH_STATUS_A15_PAD, DATA_HIGH);
	#endif


	  if(dataReady != 0)
	  {

		 //DBG_PRINT("drv_l1_i2c_multi_read MLX90640_RAMAddrstart \r\n");
		 EEaddress16 = MLX90640_RAMAddrstart;
		 EEaddr[0]=(INT8U)(EEaddress16 >> 8);
		 EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
		 error = drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pMLX32x32_READ_INT8U_buf
			 ,MLX90640_RAM_AddrRead*2,MXL_I2C_RESTART_MODE); // 多筆讀取 RAM

		 //DBG_PRINT("read return-2  = %d \r\n",error);  // return data length , if error = -1

     //
	 // in Status register - 0x8000
	 // Bit3:b
	 //  0:No new data is available in RAM (must be reset be the customer)
	 // 0x0030 :
	 // Bit4:
	 // 1: Data in RAM overwrite is enabled
	 // Bit5:
	 // 1: In step mode - start of measurement
	 // 	 (set by the customer and cleared once the measurement is done)
	 //



		 error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);

		  for(cnt=0; cnt < MLX90640_RAM_AddrRead; cnt++)
		 {
			 i = cnt << 1;
			 *(pMLX32x32_frameData_csi_INT16U_buf+cnt) = (INT16U)*(pMLX32x32_READ_INT8U_buf+i) *256
				 + (INT16U)*(pMLX32x32_READ_INT8U_buf+i+1);
		 }

		 //pMLX_TH32x24_Para->frameData[833] = statusRegister & 0x0001;	 // 紀錄 目前是 subpage ?
		 //DBG_PRINT(" 3a. [subpage = %d] \r\n",pMLX_TH32x24_Para->frameData[833]);

		  *(pMLX32x32_frameData_csi_INT16U_buf+FrameData_Subpage_NO)= statusRegister & 0x0001;	 // 紀錄 目前是 subpage ?
		 //DBG_PRINT(" 3a. [subpage = %d] \r\n",*(pMLX32x32_frameData_csi_INT16U_buf+FrameData_Subpage_NO));

		 //pMLX_TH32x24_Para->frameData[832] = controlRegister1;
		  *(pMLX32x32_frameData_csi_INT16U_buf+FrameData_ControlRegister1_NO)=controlRegister1;

		// error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);
	 }

	#if DEBUG_IO_OUT
		   gpio_write_io(TH_STATUS_A15_PAD, DATA_LOW);
	#endif

	 if( pMLX_TH32x24_Para->MLX_TH32x24_readout_block_startON == 1 )
	 	pMLX_TH32x24_Para->MLX_TH32x24_readout_block_startON = 0;

 	 event = prcess_state_get();

     if(event == PRCESS_STATE_OK)
     {
            //**************************************//
                //DBG_PRINT("user result get \r\n");

            //**************************************//
            prcess_frame_buffer_add((INT32U *)csi_buf, 1);


			//DBG_PRINT("prcess_frame_buffer_add = 0x%x\r\n", csi_buf);
     }


     if(event != PRCESS_STATE_OK)
	 	{
            free_frame_buffer_add((INT32U *)csi_buf, 1);
     	}


		// pMLX_TH32x24_Para->MLX_TH32x24_readout_block_startON = 0;

		TimeCnt2 = xTaskGetTickCount();
#if 0
		if (pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt % 50 == 0){

			DBG_PRINT("[csi_task_entry run = %d ms , startInterval = %d ] \r\n",
				TimeCnt2-TimeCnt1,TimeCnt4);

			DBG_PRINT("----- \r\n");

		}

#endif
    }
}

static void disp_task_entry(void const *parm)
{
    INT32U display_buf;
    osEvent result;
	INT32S  nRet;

	INT32U buffer_ptr,frame_size,PPU_buffer_ptr;
	PPU_REGISTER_SETS ppu_register_structure;
	PPU_REGISTER_SETS *ppu_register_set;
	INT16U	i,lp2;

    INT32U display_PPU_buf,display_PPU_forScaler_buf;

    INT16S x_pos,y_pos;
	INT32S sprite_characterNum_pos_addr;
	INT32U sp_num,sp_num_addr;
    SpN_ptr sp_ptr;

	INT8U *src_ptr;
	INT32U sprite_base_addr;
	INT32U pos_x,pos_y;
	INT16U	userDefine_spNum;

	INT32U 	lpcnt;

	INT16S ColorLp;
	INT16U OvAlertVal_SUMtmp;

	ScalerFormat_t scale;
	ScalerPara_t para;
	INT16S TmpDispBuf;

    DBG_PRINT("disp_task_entry start \r\n");
    // Initialize display device
    drv_l2_display_init();
	// drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);

	if(DISPLAY_DEVICE == DISDEV_HDMI_720P)
    {
        h_size = 640;
        v_size = 480;
        drvl1_hdmi_h264_scaling_enable(h_size,v_size,0);
    }
    else if(DISPLAY_DEVICE != DISDEV_HDMI_720P)
    {
        h_size = 720;
        v_size = 480;
    }
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);


    if(DISPLAY_DEVICE == DISDEV_TFT)
        drv_l2_display_get_size(DISPLAY_DEVICE,(INT16U *)&h_size,(INT16U *)&v_size);

	DBG_PRINT("h_size = %d v_size = %d\r\n",h_size,v_size);

	osDelay(100);

	DBG_PRINT("PPU init*** delay 100 ms \r\n");

	//PPU init start ***

	// initial ppu register parameter set structure /
	ppu_register_set = (PPU_REGISTER_SETS *) &ppu_register_structure;

	//Initiate PPU hardware engine and PPU register set structure
	gplib_ppu_init(ppu_register_set);

	//Now configure PPU software structure
	gplib_ppu_enable_set(ppu_register_set, 1);            // Enable PPU

	//TV frame mode
	gplib_ppu_non_interlace_set(ppu_register_set, 0);            // Set non-interlace mode
	gplib_ppu_frame_buffer_mode_set(ppu_register_set, 1, 0);        // Enable TV/TFT frame buffer mode

	//PPU setting
	gplib_ppu_fb_format_set(ppu_register_set, 1, 1);            // Set PPU output frame buffer format to YUYV
	gplib_ppu_vga_mode_set(ppu_register_set, 0); // Disable VGA mode
	gplib_ppu_resolution_set(ppu_register_set, C_TFT_RESOLUTION_320X240);
	gplib_ppu_free_size_set(ppu_register_set, 0, h_size, v_size);
	gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
	gplib_ppu_long_burst_set(ppu_register_set, 1);
	gplib_ppu_yuv_type_set(ppu_register_set, 3);// value[1:0]:0=BGRG/VYUY 1=GBGR/YVYU 2=RGBG/UYVY 3=GRGB/YUYV, value[2]:0=UV is unsigned(YCbCr) 1=UV is signed(YUV)
	gplib_ppu_long_burst_set(ppu_register_set, 1);


	//Frame buffer malloc
	//frame_size = (PPU_TEXT_SIZE_HPIXEL * PPU_TEXT_SIZE_VPIXEL * 2);
	frame_size = (h_size * v_size * 2);
	PPU_buffer_ptr = (INT32U) gp_malloc_align(frame_size*C_PPU_DRV_FRAME_NUM, 64); // from 320B
	   //PPU_buffer_ptr = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
	if(!PPU_buffer_ptr)
	{
		DBG_PRINT("failed to allocate frame buffer memory =>  PPU_buffer_ptr \r\n");
		while(1);
	}
	   //PPU_buffer_ptr = (PPU_buffer_ptr + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64;


	//prcess_mem_set->ppu_frame_workmem = PPU_buffer_ptr;
	for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
		gplib_ppu_frame_buffer_add(ppu_register_set, PPU_buffer_ptr + (i*frame_size));
		DBG_PRINT("PPU_buffer_ptr[%d] -> 0x%x\r\n",i,PPU_buffer_ptr + (i*frame_size));
	}

	DBG_PRINT("PPU_buffer_ptr -> 0x%x\r\n",PPU_buffer_ptr);

	// PPU init End ***


#if 1


		// PPU setting start
		DBG_PRINT("PPU setting start \r\n");

		// Now configure TEXT relative elements
		gplib_ppu_text_compress_disable_set(ppu_register_set, 1) ;	  // Disable TEXT1/TEXT2 horizontal/vertical compress function
		gplib_ppu_text_direct_mode_set(ppu_register_set, 0);				//Disable TEXT direct address mode

		//text 1 2D CSI
		gplib_ppu_text_init(ppu_register_set, C_PPU_TEXT1);
		buffer_ptr = (INT32U)gp_malloc_align(4096+64,4);
		if(!buffer_ptr)
		{
			DBG_PRINT("gplib_ppu_text_number_array_ptr_set [ C_PPU_TEXT1 ] fail \r\n");
			while(1);
		}
		buffer_ptr = (INT32U)((buffer_ptr + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
		prcess_mem_set->ppu_narray_workmem = buffer_ptr;
		gplib_ppu_text_number_array_ptr_set(ppu_register_set, C_PPU_TEXT1, (INT32U)buffer_ptr); // Set TEXT number array address
		gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT1, 1); // Enable VALUE-display TEXT
		gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT1, 1);	  // Enable bitmap mode
		gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT1, 1);	 // Get TEXT attribute from register
		gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT1, 1, 3);	   // Set TEXT color to  < 1:RGB / 3:YUYV >
		//gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 0);			 // Set TEXT size to 512x256
		gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 2);			   // Set TEXT size to 1024x512

		gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT1, 0);	 // Set TEXT segment address
		//gplib_ppu_rgb565_transparent_color_set(ppu_register_set, 1, TRANSPARENT_COLOR);	// TRANSPARENT_COLOR = 0x00CD

		gplib_ppu_palette_type_set(ppu_register_set, 0,1);
		gplib_ppu_sprite_enable_set(ppu_register_set, 1);										// Disable Sprite
		gplib_ppu_sprite_coordinate_set(ppu_register_set, 0);									// set sprite coordinate
		gplib_ppu_sprite_direct_mode_set(ppu_register_set, 1);									// Set sprite address mode
		gplib_ppu_sprite_number_set(ppu_register_set, 256); 									// Set sprite number
		gplib_ppu_sprite_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteRAM);			// set sprite ram buffer
		gplib_ppu_sprite_extend_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteExRAM);	// value: 32-bit pointer to sprite extend attribute ram
		gplib_ppu_sprite_segment_set(ppu_register_set, 0);										// sprite cell data
		gplib_ppu_sprite_zoom_enable_set(ppu_register_set, 1);							 // Enable sprite zoom mode
		gplib_ppu_sprite_rgba_mode_set(ppu_register_set, 1);

		gplib_ppu_sprite_blend_mode_set(ppu_register_set, 1);							 // Enable sprite blend mode

	#if 1
		//text 2 2D UI
		gplib_ppu_text_init(ppu_register_set, C_PPU_TEXT2);
		buffer_ptr = (INT32U)gp_malloc_align(4096+64,4);
		if(!buffer_ptr)
		{
			DBG_PRINT("gplib_ppu_text_number_array_ptr_set [ C_PPU_TEXT2 ] fail \r\n");
			while(1);
		}
		buffer_ptr = (INT32U)((buffer_ptr + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
		prcess_mem_set->ppu_narray_workmem = buffer_ptr;
		gplib_ppu_text_number_array_ptr_set(ppu_register_set, C_PPU_TEXT2, (INT32U)buffer_ptr); // Set TEXT number array address
		gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT2, 1);		//Enable TH TEXT
		gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT2, 1);	  // Enable bitmap mode
		gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT2, 1);	 // Get TEXT attribute from register
		gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT2, 1, 3);	   // Set TEXT color to < 1:RGB / 3:YUYV >
		//gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT2, 0);			 // Set TEXT size to 512x256
		gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT2, 2);			   // Set TEXT size to 1024x512

		gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT2, 0);	 // Set TEXT segment address
		gplib_ppu_text_blend_set(ppu_register_set, C_PPU_TEXT2, 1, 1, 32);	// Set Blend




		//gplib_ppu_text_depth_set(ppu_register_set, C_PPU_TEXT2, 2);	   // Set TEXT Depth 1

	#endif

		DBG_PRINT("PPU setting End \r\n");

		// PPU setting End

		frame_size = (h_size * v_size * 2);
		display_PPU_forScaler_buf = (INT32U) gp_malloc_align(frame_size, 64); // create buffer for scaler use

		userDefine_spNum = 0;
		lpcnt = 0;
		//TmpDispBuf = 250;
#endif

		userDefine_spNum = 0;
		for(lpcnt = 0 ; lpcnt < 4 ; lpcnt++) // 64x64 Big Font(line1)
        {
			set_sprite_init(userDefine_spNum,(INT32U)&Sprite001_N1_SP);

			if(DISPLAY_DEVICE == DISDEV_TFT)
				{
				x_pos=(lpcnt%5)*(SP_H_SIZE-26) + 3*SP_H_SIZE;
				y_pos=(lpcnt/5)*SP_V_SIZE;
				}

			if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
				{
			x_pos=(lpcnt%5)*(SP_H_SIZE-26) +4* SP_H_SIZE + 32;
			y_pos=(lpcnt/5)*SP_V_SIZE -2* SP_V_SIZE;
				}

			set_sprite_display_init(userDefine_spNum,x_pos,y_pos,(INT32U)_Img0001_N1_CellIdx); // 放在 HDMI 上位置 
				userDefine_spNum++;

		}

			lpcnt = 4; // unit C/F
			set_sprite_init(userDefine_spNum,(INT32U)&Sprite023_half_SP);

			if(DISPLAY_DEVICE == DISDEV_TFT)
				{
				x_pos=(lpcnt%5)*(SP_H_SIZE-20) + 3*SP_H_SIZE;
				y_pos=(lpcnt/5)*SP_V_SIZE;
				}

			if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
				{
			x_pos=(lpcnt%5)*(SP_H_SIZE-24) +4* SP_H_SIZE + 32;
			y_pos=(lpcnt/5)*SP_V_SIZE -2* SP_V_SIZE;
				}

			set_sprite_display_init(userDefine_spNum,x_pos,y_pos,(INT32U)_Img0001_N2_CellIdx); // 放在 HDMI 上位置 
				userDefine_spNum++;




		for(lpcnt = 5 ; lpcnt < 8 ; lpcnt++)	// 32x32 small Font  (line2)
		{
			set_sprite_init(userDefine_spNum,(INT32U)&Sprite001_N3_SP);
			if(DISPLAY_DEVICE == DISDEV_TFT)
				{
				x_pos=lpcnt*(SP_sm_H_SIZE-8) + 5*SP_sm_H_SIZE;
				y_pos=3*SP_sm_V_SIZE;
				}

			if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
				{
			x_pos=lpcnt*(SP_sm_H_SIZE-8) +10* SP_sm_H_SIZE;
			y_pos=-2*SP_sm_V_SIZE;
				}
			set_sprite_display_init(userDefine_spNum,x_pos,y_pos,(INT32U)_Img0001_N3_CellIdx); // 放在 HDMI 上位置 
				userDefine_spNum++;

		}

			// +/- sign userDefine_spNum = 6]
			/*
			lpcnt = 2;
			set_sprite_init(userDefine_spNum,(INT32U)&Sprite001_N3_SP);

			if(DISPLAY_DEVICE == DISDEV_TFT)
				{
				x_pos=lpcnt*(SP_sm_H_SIZE-8) + 5*SP_sm_H_SIZE;
				y_pos=3*SP_sm_V_SIZE;
				}

			if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
				{
			x_pos=lpcnt*(SP_sm_H_SIZE-8) +11* SP_sm_H_SIZE;
			y_pos=-2*SP_sm_V_SIZE;
				}

			set_sprite_display_init(userDefine_spNum,x_pos,y_pos,(INT32U)_Img0001_N3_CellIdx); // 放在 HDMI 上位置 
				userDefine_spNum++;
			*/

			// alert/No alert sign [userDefine_spNum = 10] // line3
			lpcnt = 2;
			set_sprite_init(userDefine_spNum,(INT32U)&Sprite001_N3_SP);

			if(DISPLAY_DEVICE == DISDEV_TFT)
				{
				x_pos=lpcnt*(SP_sm_H_SIZE-8) + 5*SP_sm_H_SIZE;
				y_pos=3*SP_sm_V_SIZE;
				}

			if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
				{
			x_pos=lpcnt*(SP_sm_H_SIZE-8) +11* SP_sm_H_SIZE;
			y_pos=0*SP_sm_V_SIZE;
				}

			set_sprite_display_init(userDefine_spNum,x_pos,y_pos,(INT32U)_Img0001_N3_CellIdx); // 放在 HDMI 上位置 
				userDefine_spNum++;


		#if 0 // line3
		for(lpcnt = 7 ; lpcnt < 10 ; lpcnt++)	// 32x32 small Font (line 3)
		{
			set_sprite_init(userDefine_spNum,(INT32U)&Sprite001_N3_SP);

			if(DISPLAY_DEVICE == DISDEV_TFT)
				{
				x_pos=(lpcnt-4)*(SP_sm_H_SIZE-8) + 5*SP_sm_H_SIZE;
				y_pos=4*SP_sm_V_SIZE;
				}

			if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
				{
			x_pos=(lpcnt-4)*(SP_sm_H_SIZE-8) +11* SP_sm_H_SIZE;
			y_pos=(-1)*SP_sm_V_SIZE;
				}
			set_sprite_display_init(userDefine_spNum,x_pos,y_pos,(INT32U)_Img0001_N3_CellIdx); // 放在 HDMI 上位置 
				userDefine_spNum++;

		}
	#endif // line3


	//paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,1024);
		if(userDefine_spNum > 0){
			DBG_PRINT("userDefine_spNum  = %d \r\n ",userDefine_spNum );
		paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,userDefine_spNum);
			}



    while(1)
    {
            result = osMessageGet(disp_frame_buffer_queue, osWaitForever);
            display_buf = result.value.v;
            if((result.status != osEventMessage) || !display_buf) {
                continue;
            }
            //DBG_PRINT("display_buf-1 = 0x%x\r\n", display_buf);
            //DBG_PRINT("D");

			//gp_memcpy((INT8S *)(display_buf),
			//	(INT8S *)&(sensor32X32_RGB565),32*32*2);

			if(pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor > NO_SIGNAL_FACTOR)	// no signal ONE color
				{


					//DBG_PRINT("device_h_size = %d device_v_size = %d\r\n",device_h_size,device_v_size);
					cpu_draw_line_osd(UnderZeroDiff_value,display_buf,40,
						device_h_size,110,0,16);
					cpu_draw_line_osd(OverZeroDiff_value,display_buf,40,
						device_h_size,160,0,17);
					if(DISPLAY_DEVICE == DISDEV_TFT)
						{
					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmpTbInd,display_buf,80,
						device_h_size,90,0,10);
					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor,display_buf,80,
						device_h_size,110,0,10);
						}
					if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
						{
					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmpTbInd,display_buf,80,
						device_h_size,50,0,10);
					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor,display_buf,80,
						device_h_size,110,0,10);
						}

					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL,display_buf,80,
						device_h_size,160,0,11);

					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL,display_buf,80,
						device_h_size,204,0,0);

					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_TmpMax,display_buf,120,
						device_h_size,110,0,6);
					cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_TmpMin,display_buf,120,
						device_h_size,180,0,7);

					for (lpcnt = 0; lpcnt < ALERT_BUF_LEN; ++lpcnt)	// clear buf
					{
						pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[lpcnt] = 0;
					}
					OvAlertVal_SUM = 0 ;
					TmpMaxDispBuf_cnt = 0;
					Tmp1pointDisp_Pre = 0;

				}
			else
				{
					cpu_draw_advalue_osd(UnderZeroDiff_value,display_buf,
						device_h_size,16,0,16);
					cpu_draw_advalue_osd(OverZeroDiff_value,display_buf,
						device_h_size,60,0,17);
					cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor,display_buf,
						device_h_size,110,0,10);

					cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_MAX_VAL,display_buf,
						device_h_size,160,0,11);

					cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_START_VAL,display_buf,
						device_h_size,204,0,0);

					if(DISPLAY_DEVICE == DISDEV_HDMI_480P)
						{
						cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_TmpMin,display_buf,
							device_h_size,244,0,7);

						cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_TmpMax,display_buf,
							device_h_size,288,0,6);
						cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet,display_buf,
							device_h_size,320,100,8);
						cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmpTbInd,display_buf,
							device_h_size,360,110,17);

						if (pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET >= 0)
							{
							cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET,display_buf,
							device_h_size,400,120,16);
							}
						else
							cpu_draw_advalue_osd(abs(pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET),display_buf,
								device_h_size,400,120,1);
						}

					if(DISPLAY_DEVICE == DISDEV_TFT)
						{
						cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_TmpMin,display_buf,
							device_h_size,248,0,7);

						cpu_draw_advalue_osd(pMLX_TH32x24_Para->MLX_TH32x24_TmpMax,display_buf,
							device_h_size,288,0,6);
						if (pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET >= 0)
							{
							cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET,display_buf,180,
							device_h_size,220,100,16);
							}
						else
							cpu_draw_line_osd(abs(pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET),display_buf,180,
								device_h_size,220,100,1);

						cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet,display_buf,200,
							device_h_size,220,0,8);
						cpu_draw_line_osd(pMLX_TH32x24_Para->MLX_TH32x24_GRAY_TmpTbInd,display_buf,200,
							device_h_size,220,100,17);
					}


				}


		//
		//	COLOR_SET_6 --> sprite display mode
		//

		if(( pMLX_TH32x24_Para->MLX_TH32x24_disp_Time_Flag == 1 ) &&
			(pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFactor < NO_SIGNAL_FACTOR))	// no signal ONE color
			{
			// TmpDispBuf = 250; ini val
			// 華氏 = 攝氏*(9/5)+32

			if (TmpMaxDispBuf_cnt == 0)
				{
				TmpMaxDispBuf_cnt++;
				TmpDispBuf_Pre = pMLX_TH32x24_Para->MLX_TH32x24_TmpMax;
				}

			if ( abs(TmpDispBuf_Pre - pMLX_TH32x24_Para->MLX_TH32x24_TmpMax) < 5 )
				{
				TmpDispBuf = TmpDispBuf_Pre;
				}
			else
				{
				TmpDispBuf_Pre = pMLX_TH32x24_Para->MLX_TH32x24_TmpMax;
				TmpDispBuf = pMLX_TH32x24_Para->MLX_TH32x24_TmpMax;
				}
			if (pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET == 1) // unit F
				{
				TmpDispBuf = (TmpDispBuf*9)/5 +320;
				//DBG_PRINT("TmpDispBuf= %d\r\n",TmpDispBuf);
				}

			paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,userDefine_spNum);

			//
			//	64x64 Big Font  (line1)
			//
			if ((TmpDispBuf/1000) != 0)
				{

				sprite_base_addr = (INT32U)_SPRITE_Number0_9_V0_CellData;
				sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +		// 千位數 
				( ((TmpDispBuf/1000)+1) * SP_CHR_SIZE));
				}
			else
				{
				sprite_base_addr = (INT32U)_SPRITE_Number0_9_V0_CellData;
				sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +	0);	// 千位數  = Null
				
				}


            Get_sprite_image_info(0,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
            	// Blending is 64-level when sprite window function is disabled.
				// This function should be used to set blending level when sprite window function is disabled.
	        	gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,5);
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			sprite_base_addr = (INT32U)_SPRITE_Number0_9_V0_CellData;
			sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +		// 百位數 
				( ((TmpDispBuf/100)%10 + 1) * SP_CHR_SIZE));
				//( (pMLX_TH32x24_Para->MLX_TH32x24_TmpMax%100)/10 * SP_CHR_SIZE));

            Get_sprite_image_info(1,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {

	        	gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,5);
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			 sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +  // 十位數 
			 	(((TmpDispBuf/10)%10 + 1 ) * SP_CHR_SIZE));
			 	//((pMLX_TH32x24_Para->MLX_TH32x24_TmpMax%10) * SP_CHR_SIZE));
			 Get_sprite_image_info(2,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {

	        	gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,5);
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			sprite_base_addr = (INT32U)_SPRITE_Number0_9pv1_CellData;
			 sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +  // 個位數 
			 	((TmpDispBuf%10 + 1) * SP_CHR_SIZE));
			 	//((pMLX_TH32x24_Para->MLX_TH32x24_TmpMax%10) * SP_CHR_SIZE));
			 Get_sprite_image_info(3,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {

	        	gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,5);
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }
			//set_sprite_disable(4);

			sprite_base_addr = (INT32U)_SPRITE_UnitHalf_CellData;
			if (pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET == 0)
				{
				sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +  // unit half C
			 	(1* SP_unit_CHR_SIZE));
				}
			else
				{
			 	sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +  // unit half F
			 	(2* SP_unit_CHR_SIZE));
				}
			 	//((pMLX_TH32x24_Para->MLX_TH32x24_TmpMax%10) * SP_CHR_SIZE));
			 Get_sprite_image_info(4,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {

	        	gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,5);
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			//
			//	32x32 small Font (line2)
			//
			#if 0 // +/-
			// +/- sign

			sprite_base_addr = (INT32U)_SPRITE_SmFnt_signOnly_CellData;
			if (OvAlertVal_SUM >= 0)
				{
				sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +
			 	(0 * SP_sm_CHR_SIZE)); // +
				}
			else
				{
				sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +
			 	(1 * SP_sm_CHR_SIZE)); // -
				}


			 //DBG_PRINT("32x32-2 = 0x%x \r\n",sprite_characterNum_pos_addr);
			 Get_sprite_image_info(6,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			#endif // +/-

			sprite_base_addr = (INT32U)_SPRITE_NumFntSmall_N3_CellData;


			//OvAlertVal_SUMtmp = abs(OvAlertVal_SUM); // abs() 沒有 +/- 才可以 

			//DBG_PRINT(" [%d]",OvAlertVal_SUMtmp);


			OvAlertVal_SUMtmp = pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet; //

			sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +	// 百位數 
			//	( (pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet/100) * SP_sm_CHR_SIZE));
			//((INT16U)OvAlertVal_SUM/100) * SP_sm_CHR_SIZE);
			OvAlertVal_SUMtmp/100 * SP_sm_CHR_SIZE);

			//DBG_PRINT(" \r\n");
			//DBG_PRINT("32x32-0 = 0x%x /",sprite_characterNum_pos_addr);

            Get_sprite_image_info(5,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +	// 十位數 
				( ( OvAlertVal_SUMtmp%100)/10* SP_sm_CHR_SIZE)); // for test

			//DBG_PRINT("32x32-1 = 0x%x /",sprite_characterNum_pos_addr);
            Get_sprite_image_info(6,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			 sprite_base_addr = (INT32U)_SPRITE_SmFnt_N3p_CellData;
			 sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +  // 個位數 
			 	(( OvAlertVal_SUMtmp%10) * SP_sm_CHR_SIZE)); // fortest

			 //DBG_PRINT("32x32-2 = 0x%x \r\n",sprite_characterNum_pos_addr);
			 Get_sprite_image_info(7,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			// alert sign (line3)
			if (pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_ON_FLAG == 1)
				{
				sprite_base_addr = (INT32U)_SPRITE_SmFnt_alert_CellData;
				sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr + (0* SP_sm_CHR_SIZE)); // -
				 Get_sprite_image_info(8,(SpN_ptr *)&sp_ptr);
				gplib_ppu_sprite_attribute_zoom_set((SpN_RAM *)sp_ptr.nSPNum_ptr, 36); // SP_ZOOM36
            	sp_num_addr=sp_ptr.nSPNum_ptr;
           	 	for(i=0;i<sp_ptr.nSP_CharNum;i++)
           			 {
		                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
        		        sp_num_addr+=sizeof(SpN_RAM);
            		}
				}
			else
				{
				sprite_base_addr = (INT32U)_SPRITE_SmFnt_NoAlert_CellData;
				sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr + (0* SP_sm_CHR_SIZE)); // -
				 Get_sprite_image_info(8,(SpN_ptr *)&sp_ptr);
				gplib_ppu_sprite_attribute_zoom_set((SpN_RAM *)sp_ptr.nSPNum_ptr, 0); // SP_ZOOM0
            	sp_num_addr=sp_ptr.nSPNum_ptr;
           	 	for(i=0;i<sp_ptr.nSP_CharNum;i++)
           			 {
		                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
        		        sp_num_addr+=sizeof(SpN_RAM);
            		}
				}




		#if 0	// line3
			//	32x32 small Font (line 3)
			//

			//
			if (Tmp1pointMaxDisp_cnt == 0)
				{
				Tmp1pointDisp_Pre = pMLX_TH32x24_Para->MLX_TH32x24_1pointTmpMax;
				Tmp1pointMaxDisp_cnt++;
				}
			if ( abs(Tmp1pointDisp_Pre - pMLX_TH32x24_Para->MLX_TH32x24_1pointTmpMax) < 5 )
				{
				TmpDispBuf_sum = Tmp1pointDisp_Pre;
				}
			else
				{
				TmpDispBuf_sum = pMLX_TH32x24_Para->MLX_TH32x24_1pointTmpMax;
				Tmp1pointDisp_Pre = TmpDispBuf_sum;
				}

			sprite_base_addr = (INT32U)_SPRITE_NumFntSmall_N3_CellData;
			sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +	// 百位數 
			//	( (pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet/100) * SP_sm_CHR_SIZE));
			//((INT16U)OvAlertVal_SUM/100) * SP_sm_CHR_SIZE);
			TmpDispBuf_sum/100 * SP_sm_CHR_SIZE);

			//DBG_PRINT(" \r\n");
			//DBG_PRINT("32x32-0 = 0x%x /",sprite_characterNum_pos_addr);

            Get_sprite_image_info(7,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +	// 十位數 
				( ( TmpDispBuf_sum%100)/10* SP_sm_CHR_SIZE)); // for test

			//DBG_PRINT("32x32-1 = 0x%x /",sprite_characterNum_pos_addr);
            Get_sprite_image_info(8,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }

			 sprite_base_addr = (INT32U)_SPRITE_SmFnt_N3p_CellData;
			 sprite_characterNum_pos_addr = (INT32U)(sprite_base_addr +  // 個位數 
			 	(( TmpDispBuf_sum%10) * SP_sm_CHR_SIZE)); // fortest

			 //DBG_PRINT("32x32-2 = 0x%x \r\n",sprite_characterNum_pos_addr);
			 Get_sprite_image_info(9,(SpN_ptr *)&sp_ptr);
            sp_num_addr=sp_ptr.nSPNum_ptr;
            for(i=0;i<sp_ptr.nSP_CharNum;i++)
            {
                gplib_ppu_sprite_attribute_character_number_set(ppu_register_set, (SpN_RAM *)sp_num_addr, (sprite_characterNum_pos_addr/2));
                sp_num_addr+=sizeof(SpN_RAM);
            }
		#endif // line3

			gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT2, device_h_size, device_v_size,
			//	(INT32U)_Text001_IMG0000_CellData); // Calculate Number array
			//(INT32U)_Text_BLUE_IMG0_CellData);
			(INT32U)pMLX_TH32x24_Para->MLX_TH32x24_HighTMark_display_frame);


			pMLX_TH32x24_Para->MLX_TH32x24_disp_Time_Flag = 0;

			}

			gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, h_size, h_size,
			//(INT32U)_Text_BLUE_IMG0_CellData);	// Calculate Number array // TFT & HDMI need diff TEXT !!
			//(INT32U)_Text_WHITE_IMG0_CellData);
							(INT32U)display_buf);	 // from TH sensor


			nRet =  gplib_ppu_go_and_wait_done(ppu_register_set);
			// 解決 free_frame_buffer_queue 未放回-start
			display_PPU_buf = ppu_frame_buffer_display_get();
			if(display_PPU_buf > 0)
			         gplib_ppu_frame_buffer_add(ppu_register_set, display_PPU_buf);
			// 解決 free_frame_buffer_queue 未放回-end
			//DBG_PRINT("display_PPU_buf = 0x%x \r\n",display_PPU_buf);




			drv_l2_display_update(DISPLAY_DEVICE,display_PPU_buf);
			pscaler_frame_buffer_add((INT32U *)display_buf, 1);

			}
	//else{
	//		 drv_l2_display_update(DISPLAY_DEVICE,display_buf);


     //       pscaler_frame_buffer_add((INT32U *)display_buf, 1);
	//	}



}

static void prcess_task_entry(void const *parm)
{
    INT32U prcess_buf;
    osEvent result;
	int pixelNumber;

	INT32U	TimeCnt1,TimeCnt2,TimeCnt3;

	INT32U	TimeCnt1a,TimeCnt2a;

	INT32U	TimeCnt1b,TimeCnt2b;

	INT8U	tmp_i,lp_cnt;


	INT16U  *pMLX32x32_frameData_prcess_INT16U_buf;

	float  *pMLX_TH32x24_ImgOutput_INT32S_buf0;

	float  *pMLX_TH32x24_ImgOutput_INT32S_avg_buf[IMG_AVG_buf_len];
	INT16U	cellNum; //,EOff_cellNum;

	float	ImgObject;


    INT32U PscalerBuffer;

	INT32S  nRet;

	ScalerFormat_t scale;
	ScalerPara_t para;
	float	tr_byUser;


	INT16U  *pMLX_TH32x24_TmpOutput_INT16U_buf0;

    DBG_PRINT("prcess_task_entry start \r\n");
    //**************************************//
    //DBG_PRINT("user init add \r\n");

    for(tmp_i=0;tmp_i<IMG_AVG_buf_len;tmp_i++){
			pMLX_TH32x24_ImgOutput_INT32S_avg_buf[tmp_i]= (float*)pMLX_TH32x24_Para->MLX_TH32x24_ImgAvg_buf_addr[tmp_i];
		}

	gpio_init_io(TH_STATUS_A15_PAD, GPIO_OUTPUT);
	gpio_set_port_attribute(TH_STATUS_A15_PAD, ATTRIBUTE_HIGH);
	gpio_write_io(TH_STATUS_A15_PAD, DATA_HIGH);

	gpio_init_io(TH_ALERT_A14_PAD, GPIO_OUTPUT);
	gpio_set_port_attribute(TH_ALERT_A14_PAD, ATTRIBUTE_HIGH);
	gpio_write_io(TH_ALERT_A14_PAD, DATA_LOW);


	gpio_init_io(TH_TEST1_A12_PAD, GPIO_OUTPUT);
	gpio_set_port_attribute(TH_TEST1_A12_PAD, ATTRIBUTE_HIGH);
	gpio_write_io(TH_TEST1_A12_PAD, DATA_HIGH);


	pMLX_TH32x24_TmpOutput_INT16U_buf0 = (pMLX_TH32x24_Para->result);

	for (tmp_i = 0; tmp_i < ALERT_BUF_LEN; ++tmp_i)	// clear buf
		{
		pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal[tmp_i] = 0;
		}
	OvAlertVal_SUM = 0 ;

    //**************************************//

    while(1)
    {
	    result = osMessageGet(prcess_frame_buffer_queue, osWaitForever);
        prcess_buf = result.value.v;
        if((result.status != osEventMessage) || !prcess_buf) {
                continue;
        }


		//DBG_PRINT("p %d- 0x%x\r\n",xTaskGetTickCount(), prcess_buf);
        //DBG_PRINT("P");

        //**************************************//
           //DBG_PRINT("user code add \r\n");



		//DBG_PRINT("prcess_buf = 0x%x -%d\r\n", prcess_buf,xTaskGetTickCount());
		TimeCnt1 = xTaskGetTickCount();
		pMLX32x32_frameData_prcess_INT16U_buf = (INT16U*)prcess_buf;


	#if DEBUG_IO_OUT
		gpio_write_io(TH_TEST1_A12_PAD, DATA_HIGH);
	#endif

		// 開始 計算 Tobject

		//CalulateData(CalcFormatSet,controlRegister1,csi_buf);

		 MLX90640_GetVdd(*(pMLX32x32_frameData_prcess_INT16U_buf+810),*(pMLX32x32_frameData_prcess_INT16U_buf+832));

#if DEBUG_MLX_MSG_OUT
		 DBG_PRINT(" pMLX_TH32x24_Para->MLX_TH32x24_vdd = %f (t=%d) \r\n"
			 ,pMLX_TH32x24_Para->MLX_TH32x24_vdd,xTaskGetTickCount());
#endif

		 MLX90640_GetTa(*(pMLX32x32_frameData_prcess_INT16U_buf+800),*(pMLX32x32_frameData_prcess_INT16U_buf+768));

#if DEBUG_MLX_MSG_OUT
	 DBG_PRINT(" pMLX_TH32x24_Para->MLX_TH32x24_ta = 0x%04x (%f) \r\n"
			 ,pMLX_TH32x24_Para->MLX_TH32x24_ta,pMLX_TH32x24_Para->MLX_TH32x24_ta);
#endif
		//if(*(pMLX32x32_frameData_prcess_INT16U_buf+833) == 1)
		//if(pMLX_TH32x24_Para->frameData[833] == 1)
			//DBG_PRINT(" ***frame %d at[t=%d] \r\n",	*(pMLX32x32_frameData_prcess_INT16U_buf+833),xTaskGetTickCount());




	 		tr_byUser = pMLX_TH32x24_Para->MLX_TH32x24_ta - TA_SHIFT;

			 MLX90640_CalculateTo(pMLX32x32_frameData_prcess_INT16U_buf,emissivity_byUser,tr_byUser);

		   	//	DBG_PRINT(" ***frame %d MLX90640_CalculateTo [t=%d] \r\n",
		 	//	*(pMLX32x32_frameData_prcess_INT16U_buf+833),xTaskGetTickCount());




		// MLX90640_GetImage(pMLX32x32_frameData_prcess_INT16U_buf);
		 //DBG_PRINT(" frame %d MLX90640_GetImage->1 End[t=%d] \r\n",
		 //    pMLX_TH32x24_Para->frameData[833],xTaskGetTickCount()-TimeCnt1);

#if DEBUG_TMP_READ_OUT
		 for(pixelNumber=0 ; pixelNumber<MLX_Pixel ; pixelNumber++){

			 if((pixelNumber%32 == 0) && (pixelNumber != 0)) DBG_PRINT("\r\n");
			 DBG_PRINT("(%d)",pMLX_TH32x24_Para->result[pixelNumber]);

		 }
#endif

#if DEBUG_image_READ_OUT
		for(pixelNumber=0 ; pixelNumber<MLX_Pixel ; pixelNumber++){

			 //if((pixelNumber%32 == 0) && (pixelNumber != 0)) DBG_PRINT("\r\n");
			 if(pMLX_TH32x24_Para->result_image[pixelNumber] >0)
			 	DBG_PRINT("[%f]",pMLX_TH32x24_Para->result_image[pixelNumber]);

		}
		DBG_PRINT("\r\n");

#endif




	 TimeCnt1a = xTaskGetTickCount();


	//
	// Tobject 轉換完成 
	//

	pMLX_TH32x24_ImgOutput_INT32S_buf0 = pMLX_TH32x24_Para->result_image; // image format ?

#if IMG_AVGBUF_ON

	#if DEBUG_image_READ_OUT2

		//DBG_PRINT("\r\n pMLX_TH32x24_ImgOutput_INT32S_buf0 \r\n");
		for(pixelNumber=0 ; pixelNumber<MLX_Pixel ; pixelNumber++){

		//if((pixelNumber%32 == 0) && (pixelNumber != 0)) DBG_PRINT("\r\n");
		if (*(pMLX_TH32x24_ImgOutput_INT32S_buf0 + pixelNumber ) > 0)
			{
			DBG_PRINT("(%f)-%d",*(pMLX_TH32x24_ImgOutput_INT32S_buf0 + pixelNumber ),pixelNumber);
			}

		}
		DBG_PRINT("\r\n -- \r\n");
	#endif


	//
	// image avg Tobject
	//
	if(avg_buf_Enable ==0){ 	// [0] ~ [3] <- new data
		for(tmp_i=0;tmp_i<IMG_AVG_buf_len;tmp_i++){
			gp_memcpy((INT8S *)(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[tmp_i]),
			(INT8S *)pMLX_TH32x24_ImgOutput_INT32S_buf0,MLX_Pixel*IMAGE_DATA_INT32S_SIZE);
		}

		avg_buf_Enable=1;

		}
	else{
			// move new data to avg buf
			for(tmp_i=0;tmp_i<(IMG_AVG_buf_len-1);tmp_i++){ 	// [0]<-[1] ,[1]<-[2] ,[2]<-[3]
				gp_memcpy((INT8S *)(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[tmp_i]),
				(INT8S *)(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[tmp_i+1]),MLX_Pixel*IMAGE_DATA_INT32S_SIZE);
			}
			gp_memcpy((INT8S *)(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[IMG_AVG_buf_len-1]), // [3] <- new data
					(INT8S *)pMLX_TH32x24_ImgOutput_INT32S_buf0,MLX_Pixel*IMAGE_DATA_INT32S_SIZE);

			for(cellNum=0;cellNum<MLX_Pixel;cellNum++){
				//if(*(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[0] + cellNum ) > 0)
					ImgObject = *(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[0] + cellNum );
				//else ImgObject = 0;

				for(tmp_i=1 ; tmp_i < IMG_AVG_buf_len ; tmp_i++){
					//if(*(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[tmp_i] + cellNum ) > 0)
						ImgObject = ImgObject +*(pMLX_TH32x24_ImgOutput_INT32S_avg_buf[tmp_i] + cellNum );
				}
			ImgObject = ImgObject/IMG_AVG_buf_len;
			*(pMLX_TH32x24_ImgOutput_INT32S_buf0 + cellNum)=ImgObject;

			}

	}

	#if DEBUG_image_READ_OUT2

		//DBG_PRINT("\r\n pMLX_TH32x24_ImgOutput_INT32S_buf0 \r\n");
		for(pixelNumber=0 ; pixelNumber<MLX_Pixel ; pixelNumber++){

		//if((pixelNumber%32 == 0) && (pixelNumber != 0)) DBG_PRINT("\r\n");
		if (*(pMLX_TH32x24_ImgOutput_INT32S_buf0 + pixelNumber ) > 0)
			{
			DBG_PRINT("(%f)*%d",*(pMLX_TH32x24_ImgOutput_INT32S_buf0 + pixelNumber ),pixelNumber);
			}

		}
		DBG_PRINT("\r\n -- \r\n");
	#endif

#endif



		TimeCnt1b = xTaskGetTickCount();
		FindMax_ColorAssign();

		if (pMLX_TH32x24_Para->MLX_TH32x24_disp_Time_Flag == 1)
			{
		#if DEBUG_TMP_READ_OUT2
				DBG_PRINT("\r\n pMLX_TH32x24_TmpOutput_INT16U_buf0 NO AVG \r\n");
				for(pixelNumber=0 ; pixelNumber<MLX_Pixel ; pixelNumber++){

				if((pixelNumber%32 == 0) && (pixelNumber != 0)) DBG_PRINT("\r\n");
				DBG_PRINT("(%d)",*(pMLX_TH32x24_TmpOutput_INT16U_buf0 + pixelNumber ));

				//DBG_PRINT("(%d)",pMLX_TH32x24_Para->result[pixelNumber]);

				}
		#endif

		/*
			lp_cnt++;


			if (lp_cnt%2 == 0 )
				gpio_write_io(TH_STATUS_A15_PAD, DATA_HIGH);
			else
				gpio_write_io(TH_STATUS_A15_PAD, DATA_LOW);
		*/
			DBG_PRINT(" CalculateTo [t=%d] %d\r\n",xTaskGetTickCount()-TimeCnt1,
				pMLX_TH32x24_Para->MLX_TH32x24_TmpMax);

				//pMLX_TH32x24_Para->MLX_TH32x24_Time_Flag = 0;

		}
		//else DBG_PRINT("NO  CalculateTo -t=%d- \r\n",xTaskGetTickCount()-TimeCnt1);

		TimeCnt2 = xTaskGetTickCount();

		firstRun = 1;


        PscalerBuffer = pscaler_frame_buffer_get(1);
        //if(PscalerBuffer)
        //    fd_display_set_frame(csi_buf, PscalerBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, device_h_size, device_v_size);


		if(PscalerBuffer){

		//	fd_display_set_frame(csi_buf, PscalerBuffer, 32, 32,
		//		device_h_size, device_v_size);

		//
		// MLX_TH32x24 sensor scaler
		//


		gp_memset((INT8S *)&scale, 0x00, sizeof(scale));
			//drv_l2_scaler_init();
			//scale.input_format = pAviEncVidPara->sensor_output_format;

			//if (pMLX_TH32x24_Para->MLX_TH32x24_ColorMode == COLOR_SET_0)
			//{
			//	scale.input_format = C_SCALER_CTRL_IN_Y_ONLY; 	// gray value
			//}
			//else
			//{
				scale.input_format = C_SCALER_CTRL_IN_ARGB8888;
			//}



			scale.input_width = rowNumEnd_32;
			scale.input_height = colNumEnd_24;
			scale.input_visible_width = rowNumEnd_32;
			scale.input_visible_height = colNumEnd_24;
			scale.input_x_offset = 0;
			scale.input_y_offset = 0;

			//scale.input_y_addr = sensor_frame;
			//scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr ;

			//if (pMLX_TH32x24_Para->MLX_TH32x24_ColorMode == COLOR_SET_0)
			//scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFrame_addr ;
			//else
			//{
				scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_RGB888_OutputFrame_addr;
			//}

			scale.input_u_addr = 0;
			scale.input_v_addr = 0;
			#if 1
			scale.input_b_y_addr =0;
			scale.input_b_u_addr = 0;
			scale.input_b_v_addr = 0;
			#endif

			scale.output_format = C_SCALER_CTRL_OUT_YUYV;
			scale.output_width = device_h_size;
			scale.output_height = device_v_size;
			scale.output_buf_width = device_h_size;
			scale.output_buf_height = device_v_size;
			scale.output_x_offset = 0;

			//scale.output_y_addr = scaler_frame;
			scale.output_y_addr = PscalerBuffer;
			scale.output_u_addr = 0;
			scale.output_v_addr = 0;


			scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
			//scale.scale_mode = C_SCALER_FULL_SCREEN;
			scale.scale_mode = C_SCALER_BY_RATIO;
			scale.digizoom_m = 10;
			scale.digizoom_n = 10;
			//#endif
			memset((void *)&para, 0x00, sizeof(para));

			//para.gamma_en = 1;
			//para.color_matrix_en = 1;
			para.boundary_color = 0x008080;
			//para.yuv_type = C_SCALER_CTRL_TYPE_YUV;

    		nRet = drv_l2_scaler_trigger(SCALER_0, 1, &scale, &para);
			if(nRet == C_SCALER_STATUS_DONE || nRet == C_SCALER_STATUS_STOP) {
				drv_l2_scaler_stop(SCALER_0);
			}
			else {
				DBG_PRINT("Scale1 Fail\r\n");
			while(1);
			}


		if (pMLX_TH32x24_Para->MLX_TH32x24_disp_Time_Flag == 1)
		{

		//DBG_PRINT(" HighTMark-S [t=%d] ",xTaskGetTickCount());
		//
		// MLX_TH32x24 HighTMark display scaler
		//


		gp_memset((INT8S *)&scale, 0x00, sizeof(scale));


			scale.input_format = C_SCALER_CTRL_IN_ARGB8888;

			scale.input_width = rowNumEnd_32*ScaleUp_3;
			scale.input_height = colNumEnd_24*ScaleUp_3;
			scale.input_visible_width = rowNumEnd_32*ScaleUp_3;
			scale.input_visible_height = colNumEnd_24*ScaleUp_3;
			scale.input_x_offset = 0;
			scale.input_y_offset = 0;

			//scale.input_y_addr = sensor_frame;
			//scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr ;

			//scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_RGB888_OutputFrame_addr;
			scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_RGB888_HighTMarkScaleUp_OutputFrame_addr;


			scale.input_u_addr = 0;
			scale.input_v_addr = 0;
			#if 1
			scale.input_b_y_addr =0;
			scale.input_b_u_addr = 0;
			scale.input_b_v_addr = 0;
			#endif

			scale.output_format = C_SCALER_CTRL_OUT_YUYV;
			scale.output_width = device_h_size;
			scale.output_height = device_v_size;
			scale.output_buf_width = device_h_size;
			scale.output_buf_height = device_v_size;
			scale.output_x_offset = 0;

			//scale.output_y_addr = scaler_frame;

			scale.output_y_addr = pMLX_TH32x24_Para->MLX_TH32x24_HighTMark_display_frame;
			//scale.output_y_addr = PscalerBuffer;
			scale.output_u_addr = 0;
			scale.output_v_addr = 0;


			scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
			//scale.scale_mode = C_SCALER_FULL_SCREEN;
			scale.scale_mode = C_SCALER_BY_RATIO;
			scale.digizoom_m = 10;
			scale.digizoom_n = 10;
			//#endif
			memset((void *)&para, 0x00, sizeof(para));

			//para.gamma_en = 1;
			//para.color_matrix_en = 1;
			para.boundary_color = 0x008080;
			//para.yuv_type = C_SCALER_CTRL_TYPE_YUV;

    		nRet = drv_l2_scaler_trigger(SCALER_0, 1, &scale, &para);
			if(nRet == C_SCALER_STATUS_DONE || nRet == C_SCALER_STATUS_STOP) {
				drv_l2_scaler_stop(SCALER_0);
			}
			else {
				DBG_PRINT("-HighTMark- Scale1 Fail\r\n");
			while(1);
			}



		//DBG_PRINT(" HighTMark-F [t=%d] \r\n",xTaskGetTickCount());




		}


		}

        //================================================================

	#if DEBUG_IO_OUT
		   gpio_write_io(TH_TEST1_A12_PAD, DATA_LOW);
	#endif

        //**************************************//
        //DBG_PRINT("user draw image \r\n");

        //**************************************//

            disp_frame_buffer_add((INT32U *)PscalerBuffer, 1);



       //**************************************//

        free_frame_buffer_add((INT32U *)prcess_buf, 1);


            prcess_state_post(PRCESS_STATE_OK);



			TimeCnt3 = xTaskGetTickCount();

		#if 0
			if (pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt % 50 == 0){
			DBG_PRINT("[ CalulateData %d ms ,IMG_AVGBUF run %d ms , FindMax_ColorAssign run = %d , rest = %d ] \r\n"
			,TimeCnt1a-TimeCnt1,TimeCnt1b-TimeCnt1a,TimeCnt2-TimeCnt1b,TimeCnt3-TimeCnt2);
				}
		#endif

			//pMLX_TH32x24_Para->MLX_TH32x24_readout_block_startON = 0;
    }
}

void GPM4_CSI_DISP_Demo(void)
{
    osThreadDef_t csi_task = {"csi_task", csi_task_entry, osPriorityAboveNormal, 1, 8192};
  	//  thermal_task_entry
    //osThreadDef_t thermal_task = {"thermal_task", thermal_task_entry, osPriorityAboveNormal, 1, 8192};
    osThreadDef_t disp_task = {"disp_task", disp_task_entry, osPriorityNormal, 1, 8192};
    osThreadDef_t prcess_task = {"prcess_task", prcess_task_entry, osPriorityNormal, 1, 32768};
    osMessageQDef_t disp_q = {DISP_QUEUE_MAX, sizeof(INT32U), 0};
    osSemaphoreDef_t disp_sem = {0};


	INT32S  nRet;

#if CSI_SD_EN == 1
    while(1)
    {
        INT32S ret;

        ret = _devicemount(FS_SD2);
        if(ret)
        {
            DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD2);
            #if 1
            {
                ret = _format(FS_SD2, FAT32_Type);
                if (ret)
                  DBG_PRINT("Format Disk Fail[%d]\r\n", FS_SD2);
                ret = _deviceunmount(FS_SD2);
                if (ret)
                  DBG_PRINT("UnMount Disk Fail[%d]\r\n", FS_SD2);
                ret = _devicemount(FS_SD2);
            }
            #endif
        }
        else
        {
           DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD2);
           break;
        }
        osDelay(5);
    }
#endif

	// osSemaphoreCreate
	if(sem_disp_engine == NULL)
	{
		sem_disp_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_disp_engine)
		{
            DBG_PRINT("sem_disp_engine error\r\n");
            while(1);
		}
		else
            DBG_PRINT("sem_disp_engine = 0x%x\r\n", sem_disp_engine);
	}

    // osMessageCreate
    if(free_frame_buffer_queue == NULL)
	{
        free_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!free_frame_buffer_queue)
		{
            DBG_PRINT("free_frame_buffer_queue error\r\n");
            while(1);
		}

		else
            DBG_PRINT("free_frame_buffer_queue = 0x%x\r\n", free_frame_buffer_queue);
	}
    if(csi_frame_buffer_queue == NULL)
	{
        csi_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!csi_frame_buffer_queue)
		{
            DBG_PRINT("csi_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("csi_frame_buffer_queue = 0x%x\r\n", csi_frame_buffer_queue);
	}
    if(prcess_frame_buffer_queue == NULL)
	{
        prcess_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_frame_buffer_queue)
		{
            DBG_PRINT("prcess_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_frame_buffer_queue = 0x%x\r\n", prcess_frame_buffer_queue);
	}
    if(pscaler_frame_buffer_queue == NULL)
	{
        pscaler_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!pscaler_frame_buffer_queue)
		{
            DBG_PRINT("pscaler_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("pscaler_frame_buffer_queue = 0x%x\r\n", pscaler_frame_buffer_queue);
	}
    if(disp_frame_buffer_queue == NULL)
	{
        disp_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!disp_frame_buffer_queue)
		{
            DBG_PRINT("disp_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("disp_frame_buffer_queue = 0x%x\r\n", disp_frame_buffer_queue);
	}
    if(prcess_state_queue == NULL)
	{
        prcess_state_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_state_queue)
		{
            DBG_PRINT("prcess_state_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_state_queue = 0x%x\r\n", prcess_state_queue);
	}

	// osThreadCreate
	csi_id = osThreadCreate(&csi_task, (void *)NULL);
    if(csi_id == 0)
    {
        DBG_PRINT("osThreadCreate: csi_id error\r\n");
        while(1);
    }
    else
    {
        osDelay(5);
        //DBG_PRINT("osThreadCreate: csi_id = 0x%x \r\n", csi_id);
    }

    disp_id = osThreadCreate(&disp_task, (void *)NULL);
    if(disp_id == 0)
    {
        DBG_PRINT("osThreadCreate: disp_id error\r\n");
        while(1);
    }
    else
    {
        osDelay(5);
        //DBG_PRINT("osThreadCreate: disp_id = 0x%x \r\n", disp_id);
    }

    prcess_id = osThreadCreate(&prcess_task, (void *)NULL);
    if(prcess_id == 0)
    {
        DBG_PRINT("osThreadCreate: prcess_id error\r\n");
        while(1);
    }
    else
    {
        osDelay(5);
        //DBG_PRINT("osThreadCreate: prcess_id = 0x%x \r\n", prcess_id);
    }



	// start timer_B
	pMLX_TH32x24_Para->MLX_TH32x24_sampleCnt = 0;
	pMLX_TH32x24_Para->MLX_TH32x24_Time_cnt = 0;
	pMLX_TH32x24_Para->MLX_TH32x24_disp_Time_Flag = 0;
	pMLX_TH32x24_Para->MLX_TH32x24_calc_Time_Flag = 0;

	//pMLX_TH32x24_Para->MLX_TH32x24_InitReadEE_startON = 1;
	//pMLX_TH32x24_Para->MLX_TH32x24_sampleHz = 100; // 5.7~ 732 (100ms),20(50ms),100(10 ms),500(2 ms)
	pMLX_TH32x24_Para->MLX_TH32x24_sampleHz = 500; // 5.7~ 732 (100ms),20(50ms),100(10 ms),200(5ms),500(2 ms)

	//pMLX_TH32x24_Para->MLX_TH32x24_sampleHz = 1000; // 5.7~ 732 (100ms),20(50ms),100(10 ms),200(5ms),500(2 ms),1000(1 ms)

	nRet = timer_freq_setup(TIMER_B, pMLX_TH32x24_Para->MLX_TH32x24_sampleHz, 0, MLX_TH32x24_start_timer_isr );

	DBG_PRINT("Set MLX_TH32x24_ReadElecOffset_timer_isr ret--> %d, set -%d- ms \r\n",nRet,
		1000/pMLX_TH32x24_Para->MLX_TH32x24_sampleHz) ;

	pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet = 330;
	pMLX_TH32x24_Para->MLX_TH32x24_GRAY_AMP_START = GRAY_AMP_START;
	pMLX_TH32x24_Para->MLX_TH32x24_GRAY_AMP_SCALE = GRAY_AMP_SCALE;
	pMLX_TH32x24_Para->MLX_TH32x24_LowPass_SET = 1;
	pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET = 0;
	pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_ON_FLAG = 0;
	pMLX_TH32x24_Para->MLX_TH32x24_AlertTime_cnt = 0;

	pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET = 0; // C

    // ad key init
	adc_key_scan_init();

	while(1)
	{
		adc_key_scan();

		if(ADKEY_IO1)
		{
			pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet=
				pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet+5;
			if( pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet > ALTERTmpSet_MAX )
				pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet = ALTERTmpSet_MIN;

			DBG_PRINT("ad_key-1 selection -alterTmpSet = %d\r\n",
				pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet);
		}
		else if(ADKEY_IO2)
		{
			pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet=
				pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet-5;
			if( pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet < ALTERTmpSet_MIN )
				pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet = ALTERTmpSet_MAX;
			DBG_PRINT("ad_key-2 selection -alterTmpSet = %d\r\n",
				pMLX_TH32x24_Para->MLX_TH32x24_alertTmpSet);
		}
		else if(ADKEY_IO3)
			{
			pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET =
				pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET + 1;
			
			if( pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET > 1 )
				pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET = 0;

			DBG_PRINT("ad_key-3 TMPunit_SET = %d\r\n",
				pMLX_TH32x24_Para->MLX_TH32x24_TMPunit_SET);
			}
		
		/*
		else if(ADKEY_IO4)
			{
			pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET --;
			if( pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET < -240 )
				pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET = 0;
			DBG_PRINT("ad_key-4 selection -OvAlertVal_SET = %d\r\n",
				pMLX_TH32x24_Para->MLX_TH32x24_OvAlertVal_SET);
			}
		*/
		
		//}


	}
}



