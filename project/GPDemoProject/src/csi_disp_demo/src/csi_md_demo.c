#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_l1_csi.h"
#include "drv_l1_cdsp.h"
#include "drv_l1_gpio.h"
#include "drv_l1_scaler.h"
#include "drv_l1_pscaler.h"
#include "drv_l2_scaler.h"
#include "drv_l2_sensor.h"
#include "drv_l2_display.h"
#include "drv_l2_ad_key_scan.h"
#include "drv_l2_cdsp.h"
#include "gplib_ppu.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "FaceDetectAP.h"
#include "sprite_data_16x16SP_HDR.h"

#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define FRAME_BUF_ALIGN16                   0xF
#define FRAME_BUF_ALIGN8                    0x7
#define FRAME_BUF_ALIGN4                    0x3

#define CSI_MOTION_DETECTION_EN             1
#define PPU_DRAW_EN                         1
#define CSI_SD_EN                           0
#define DISP_QUEUE_MAX		                6
#define C_DEVICE_FRAME_NUM		            3
#define DUMMY_BUFFER_ADDRESS                0x50000000
#define SENSOR_SRC_WIDTH		            640
#define SENSOR_SRC_HEIGHT		            480
#define PRCESS_SRC_WIDTH		            SENSOR_SRC_WIDTH
#define PRCESS_SRC_HEIGHT		            SENSOR_SRC_HEIGHT
#define PRCESS_STATE_OK                     0x80
#define DISP_USE_PSCALE_EN                  1
#define CSI_PSCALE_USE                      PSCALER_A
#define DISP_PSCALE_USE                     PSCALER_B
#define MD_BLOCK_SIZE                       MD_16_SIZE

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
static xSemaphoreHandle sem_prcess_engine = NULL;
static PSCALER_PARAM_STRUCT PScalerParam = {0};
static INT32U device_h_size, device_v_size, md_block_size;
#if CSI_MOTION_DETECTION_EN == 1
#define MD_SENS			                    10
#define MD_NORMAL			                35
#define	MD_SLOW			                    80
#define	SENSOR_MD_THR		                MD_NORMAL
#define MAX_SPRITE_NUMBER                   512
static xQueueHandle md_free_buffer_queue = NULL;
static xQueueHandle md_ready_buffer_queue = NULL;
INT32U md_work_iram_addr = 0x1FFF7000;
#endif
#if PPU_DRAW_EN == 1
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set = NULL;
#endif
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

static void drv_prcess_lock(void)
{
    if(sem_prcess_engine){
        osSemaphoreWait(sem_prcess_engine, osWaitForever);
    }
}

static void drv_prcess_unlock(void)
{
    if(sem_prcess_engine){
        osSemaphoreRelease(sem_prcess_engine);
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

    //drv_disp_lock();
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
    //drv_disp_unlock();

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

#if CSI_MOTION_DETECTION_EN == 1
// md free buffer queue
static INT32S md_free_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    //drv_disp_lock();
    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(md_free_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(md_free_buffer_queue, (uint32_t)&event, 10);
    }
    //drv_disp_unlock();

	return temp;
}
static INT32S md_free_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(md_free_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(md_free_buffer_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}
// md ready buffer queue
static INT32S md_ready_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    //drv_disp_lock();
    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(md_ready_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(md_ready_buffer_queue, (uint32_t)&event, 10);
    }
    //drv_disp_unlock();

	return temp;
}
static INT32S md_ready_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(md_ready_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(md_ready_buffer_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}

#endif

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

#if CSI_MOTION_DETECTION_EN == 1
static void CSI_ISR_Callback(INT32U CSI_Event)
{
    INT32U i,*src,*tar,ready_buf,md_buf;

    if(CSI_Event == CSI_MOTION_DET_EVENT)
    {
        ready_buf = md_free_buffer_get(1);
        if(ready_buf)
        {
            md_buf = drv_l1_csi_md_get_buf();
            src = (INT32U *)md_buf;
            tar = (INT32U *)ready_buf;
            for(i=0;i<md_block_size;i++)
                *tar++ = *src++;
            md_ready_buffer_add((INT32U *)md_buf, 1);
            drv_l1_csi_md_set_buf(ready_buf);
        }
    }
}

static void CDSP_ISR_Callback(void)
{
    INT32U result = drv_l1_CdspGetResult();

    if(result > 0x125)
        DBG_PRINT("M");
}
#endif
INT32U md_size;
static void mazeTest_Preview_PScaler(void)
{
    CHAR *p;
	INT32U i,PrcessBuffer,csi_mode,temp;
	INT32U csiBufferSize,csiBuffer;
	drv_l2_sensor_ops_t *pSencor;
	drv_l2_sensor_info_t *pInfo;
	drv_l2_sensor_para_t *pPara;

	csiBufferSize = PRCESS_SRC_WIDTH*PRCESS_SRC_HEIGHT*2;
    csiBuffer = DUMMY_BUFFER_ADDRESS;

	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*C_DEVICE_FRAME_NUM)+64), 32);
    if(PrcessBuffer == 0)
    {
        DBG_PRINT("CSIBuffer fail\r\n");
        while(1);
    }
	PrcessBuffer = (INT32U)((PrcessBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<C_DEVICE_FRAME_NUM; i++)
	{
		temp = (PrcessBuffer+i*csiBufferSize);
		free_frame_buffer_add((INT32U *)temp,1);
		DBG_PRINT("CSIBuffer:0x%X \r\n",temp);
	}
#if CSI_MOTION_DETECTION_EN == 1
#if MD_BLOCK_SIZE == MD_16_SIZE
    md_block_size = drv_l1_csi_md_get_block_size(SENSOR_SRC_WIDTH, SENSOR_SRC_HEIGHT, 1);
#else
    md_block_size = drv_l1_csi_md_get_block_size(SENSOR_SRC_WIDTH, SENSOR_SRC_HEIGHT, 0);
#endif
    csiBufferSize = (md_block_size * sizeof(INT32U));
	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*C_DEVICE_FRAME_NUM)+64), 32);
    if(PrcessBuffer == 0)
    {
        DBG_PRINT("MDBuffer fail\r\n");
        while(1);
    }
	PrcessBuffer = (INT32U)((PrcessBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<C_DEVICE_FRAME_NUM; i++)
	{
		temp = (PrcessBuffer+i*csiBufferSize);
		md_free_buffer_add((INT32U *)temp,1);
		DBG_PRINT("MDBuffer:0x%X \r\n",temp);
	}
#endif

    // sensor init
	drv_l2_sensor_init();
    pSencor = drv_l2_sensor_get_ops(0);
    pPara = drv_l2_sensor_get_para();
    DBG_PRINT("SensorName = %s\r\n", pSencor->name);

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

	PScalerParam.intEnableFlag = PIPELINE_SCALER_INT_ENABLE_ALL;
    PScalerParam.callbackFunc = PScaler_Callback_ISR_AutoZoom;

    if (csi_mode == CSI_INTERFACE)
    {
        // CSI
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CSI;
        // Open CSI data path
        drvl1_csi_input_pscaler_set(1);
        pPara->pscaler_src_mode = PIPELINE_SCALER_INPUT_SOURCE_CSI;
#if CSI_MOTION_DETECTION_EN == 1
        pPara->md_csi_callback = CSI_ISR_Callback;
        pPara->md_block_size = MD_BLOCK_SIZE;
        pPara->md_buf = md_free_buffer_get(1);
        pPara->md_threshold = SENSOR_MD_THR;
        pPara->md_mode = 1;
        drv_l1_register_csi_cbk(pPara->md_csi_callback);
        drv_l1_csi_md_set_parameter(pPara->md_mode, SENSOR_SRC_WIDTH, SENSOR_SRC_HEIGHT, pPara->md_block_size);
        drv_l1_csi_md_set_threshold(pPara->md_threshold);
        drv_l1_csi_irq_enable(1, MASK_CSI_MOTION_DET_FLAG);
        drv_l1_csi_md_set_buf(pPara->md_buf);
        drv_l1_csi_set_irq(1);
#endif
    }
    else
    {
        // CDSP
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CDSP;
        // Open CDSP data path
        drv_l1_CdspSetYuvPscalePath(1);
        pPara->pscaler_src_mode = PIPELINE_SCALER_INPUT_SOURCE_CDSP;
#if CSI_MOTION_DETECTION_EN == 1
        pPara->md_buf = md_work_iram_addr;
        pPara->md_callback = CDSP_ISR_Callback;
        pPara->md_threshold = SENSOR_MD_THR;
        pPara->md_mode = 1;
        drv_l1_CdspSetMD(pPara->md_mode, pPara->md_threshold, SENSOR_SRC_WIDTH, pPara->md_buf);
        drv_l2_CdspIsrRegister(C_ISR_EOF, pPara->md_callback);
#endif
    }
    mazePscalerSet(&PScalerParam);

    pSencor->init();
	pSencor->stream_start(temp, csiBuffer, csiBuffer);

    drv_l1_pscaler_start(PScalerParam.pScalerNum);
}

#if PPU_DRAW_EN == 1
static void ppu_sprite_draw(INT32U md_buf)
{
#if CSI_MOTION_DETECTION_EN == 1
	#define CHAR_SIZE               16
	INT16U i,j,x,y,sprinum;
    INT32U diff,baseaddr;

    baseaddr = md_buf;
	sprinum = 0;
	for(i=0;i<MAX_SPRITE_NUMBER;i++)
        set_sprite_disable(i);

    for (i=0;i<SENSOR_SRC_HEIGHT/CHAR_SIZE;i++) {
        for (j=0;j<SENSOR_SRC_WIDTH/CHAR_SIZE;j=j+4) {
            diff = *(volatile unsigned int*)(baseaddr);
            if ((diff & 0x1) != 0) { 	 // Hit
                x = ((j<<4)+(CHAR_SIZE/2))-(SENSOR_SRC_WIDTH/2);
                y = (SENSOR_SRC_HEIGHT/2)-((i<<4)+(CHAR_SIZE/2));
                set_sprite_display_init(sprinum,x,y,(INT32U)_r16x16_CellIdx);
                sprinum++;
            }
            if ((diff & 0x2) != 0 ) {  // Hit
                x = (((j+1)<<4)+(CHAR_SIZE/2))-(SENSOR_SRC_WIDTH/2);
                y = SENSOR_SRC_HEIGHT/2-((i<<4)+(CHAR_SIZE/2));
                set_sprite_display_init(sprinum,x,y,(INT32U)_r16x16_CellIdx);
                sprinum++;
            }
            if ((diff & 0x4) != 0 ) {  // Hit
                x = (((j+2)<<4)+(CHAR_SIZE/2))-(SENSOR_SRC_WIDTH/2);
                y = SENSOR_SRC_HEIGHT/2-((i<<4)+(CHAR_SIZE/2));
                set_sprite_display_init(sprinum,x,y,(INT32U)_r16x16_CellIdx);
                sprinum++;
            }
            if ((diff & 0x8) != 0 ) {  // Hit
                x = (((j+3)<<4)+(CHAR_SIZE/2))-(SENSOR_SRC_WIDTH/2);
                y = SENSOR_SRC_HEIGHT/2-((i<<4)+(CHAR_SIZE/2));
                set_sprite_display_init(sprinum,x,y,(INT32U)_r16x16_CellIdx);
                sprinum++;
            }
            baseaddr+=4;
            if (sprinum >= MAX_SPRITE_NUMBER)
            {
                j = (SENSOR_SRC_WIDTH/CHAR_SIZE);
                i = (SENSOR_SRC_HEIGHT/CHAR_SIZE);
            }
        }
    }
#endif
}

static void ppu_draw_init(void)
{
    INT32U i,PPU_FRAME_BUFFER_BASE,frame_size,buffer_ptr;
    FB_LOCK_STRUCT fb_lock_set;

    //drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&device_h_size, (INT16U *)&device_v_size);

    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;
    //Initiate PPU hardware engine and PPU register set structure
    gplib_ppu_init(ppu_register_set);
    if(PRCESS_SRC_WIDTH != device_h_size)
    {
        fb_lock_set.color1 = PPU_FMT_YUYV;
        fb_lock_set.h_size1 = PRCESS_SRC_WIDTH;
        fb_lock_set.v_size1 = PRCESS_SRC_HEIGHT;
        fb_lock_set.color2 = PPU_FMT_YUYV;
        fb_lock_set.h_size2 = device_h_size;
        fb_lock_set.v_size2 = device_v_size;
        gplib_ppu_fb_lock_process_enable_set(ppu_register_set,(FB_LOCK_STRUCT *)&fb_lock_set);
    }
    //Now configure PPU software structure
    gplib_ppu_enable_set(ppu_register_set, 1);					            // Enable PPU

    //TV frame mode
    gplib_ppu_non_interlace_set(ppu_register_set, 0);			            // Set non-interlace mode
    gplib_ppu_frame_buffer_mode_set(ppu_register_set, 1, 0);		        // Enable TV/TFT frame buffer mode

    //PPU setting
    gplib_ppu_fb_format_set(ppu_register_set, 1, 1);			            // Set PPU output frame buffer format to YUYV
    gplib_ppu_vga_mode_set(ppu_register_set, 0);							// Disable VGA mode
    gplib_ppu_resolution_set(ppu_register_set, C_TFT_RESOLUTION_320X240);	// Set display resolution to 640x480
    gplib_ppu_free_size_set(ppu_register_set, 0, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT);
    gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
    gplib_ppu_long_burst_set(ppu_register_set, 1);

    //Frame buffer malloc
    frame_size = (PRCESS_SRC_WIDTH * PRCESS_SRC_HEIGHT * 2);
    PPU_FRAME_BUFFER_BASE = (INT32U) gp_malloc_align(((frame_size*C_DEVICE_FRAME_NUM)+128), 64);
    if(PPU_FRAME_BUFFER_BASE == 0)
    {
        DBG_PRINT("PPU_FRAME_BUFFER_BASE fail\r\n");
        while(1);
    }
    PPU_FRAME_BUFFER_BASE = (INT32U)((PPU_FRAME_BUFFER_BASE + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    for (i=0; i<C_DEVICE_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(PPU_FRAME_BUFFER_BASE + (i*frame_size));
            gplib_ppu_frame_buffer_add(ppu_register_set, buffer_ptr);
            DBG_PRINT("PPUBuffer:0x%X \r\n",buffer_ptr);
    }
    gp_memset((INT8U *)buffer_ptr, 0, frame_size);
    drv_l2_display_update(DISPLAY_DEVICE,buffer_ptr);

    // Now configure TEXT relative elements
    gplib_ppu_text_compress_disable_set(ppu_register_set, 1);	                    // Disable TEXT1/TEXT2 horizontal/vertical compress function
    gplib_ppu_text_direct_mode_set(ppu_register_set, 0);			                // Disable TEXT direct address mode

    //text 1 2D
    gplib_ppu_text_init(ppu_register_set, C_PPU_TEXT1);
    PPU_FRAME_BUFFER_BASE = (INT32U)gp_malloc_align((8192+64),4);
    PPU_FRAME_BUFFER_BASE = (INT32U)((PPU_FRAME_BUFFER_BASE + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    gplib_ppu_text_number_array_ptr_set(ppu_register_set, C_PPU_TEXT1, (INT32U)PPU_FRAME_BUFFER_BASE);	 // Set TEXT number array address
    gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT1, 1);	                        // Enable TEXT
    gplib_ppu_yuv_type_set(ppu_register_set, 3);								     // Set 32-bit color format to Y1UY0V
    gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT1, 1);			     // Enable bitmap mode
    gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT1, 1);	    // Get TEXT attribute from register
    gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT1, 1, 3);				     // Set TEXT color to YUYV
    gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 5);			             // Set TEXT size to 1024x512
    gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT1, 0);				    // Set TEXT segment address
 	#if CSI_MOTION_DETECTION_EN == 1
		gplib_ppu_sprite_init(ppu_register_set);
		gplib_ppu_sprite_enable_set(ppu_register_set, 1);			                        // Enable Sprite
		gplib_ppu_sprite_coordinate_set(ppu_register_set, 0);                               // set sprite center coordinate
		gplib_ppu_sprite_direct_mode_set(ppu_register_set, 0);		                        // Set sprite relative address mode
		gplib_ppu_sprite_number_set(ppu_register_set, 256);                                 // Set sprite number
		gplib_ppu_sprite_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteRAM);        // set sprite ram buffer
		gplib_ppu_sprite_segment_set(ppu_register_set, (INT32U)_r16x16_CellData);           // sprite cell data
		gplib_ppu_palette_type_set(ppu_register_set, 1,0);                                  // P1024 mode
	    gplib_ppu_palette_ram_ptr_set(ppu_register_set, 0,(INT32U)_sprite_data_16X16SP_Palette0);    // sprite palette
        set_sprite_free_size(SENSOR_SRC_WIDTH, SENSOR_SRC_HEIGHT);
    #else // Disable Sprite
		gplib_ppu_sprite_init(ppu_register_set);
		gplib_ppu_sprite_enable_set(ppu_register_set, 0);
	#endif
}

static INT32U ppu_draw_go(INT32U h, INT32U v, INT32U frame_buffer)
{
#if CSI_MOTION_DETECTION_EN == 1
    INT32U md_buf;
#endif

    if(ppu_register_set)
    {
        gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, h, v, (INT32U)frame_buffer);	// Calculate Number array
    #if CSI_MOTION_DETECTION_EN == 1
        if(PScalerParam.inSource == PIPELINE_SCALER_INPUT_SOURCE_CSI)
        {
            md_buf = md_ready_buffer_get(1);
            if(md_buf)
            {
                ppu_sprite_draw(md_buf);
                md_free_buffer_add((INT32U *)md_buf, 1);
                paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_Freemode,PPU_hardware_coordinate,MAX_SPRITE_NUMBER);
            }
        }
        else
            gplib_ppu_sprite_enable_set(ppu_register_set, 0);
    #endif
        // Start PPU and wait until PPU operation is done
        gplib_ppu_go_and_wait_done(ppu_register_set);

        return  (INT32U)ppu_frame_buffer_display_get();
    }
    else
        return 0;
}
#endif

static void csi_task_entry(void const *parm)
{
    INT32U csi_buf,PscalerBuffer,PscalerBufferSize;
    INT32U i,event;
    osEvent result;
#if PPU_DRAW_EN == 1
    INT32U ppu_buf;
#endif

    DBG_PRINT("csi_task_entry start \r\n");
    // csi init
    mazeTest_Preview_PScaler();

    // disp size
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&device_h_size, (INT16U *)&device_v_size);
	PscalerBufferSize = (device_h_size * device_v_size * 2);
	PscalerBuffer = (INT32U) gp_malloc_align(((PscalerBufferSize*C_DEVICE_FRAME_NUM)+64), 32);
    if(PscalerBuffer == 0)
    {
        DBG_PRINT("PscalerBuffer fail\r\n");
        while(1);
    }
	PscalerBuffer = (INT32U)((PscalerBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<C_DEVICE_FRAME_NUM; i++)
	{
		csi_buf = (PscalerBuffer+i*PscalerBufferSize);
		pscaler_frame_buffer_add((INT32U *)csi_buf,1);
		DBG_PRINT("PscalerBuffer:0x%X \r\n",csi_buf);
	}
    prcess_state_post(PRCESS_STATE_OK);

    while(1)
    {
        result = osMessageGet(csi_frame_buffer_queue, osWaitForever);
        csi_buf = result.value.v;
        if((result.status != osEventMessage) || !csi_buf) {
            continue;
        }
        //DBG_PRINT("csi_buffer = 0x%x\r\n", csi_buf);
        //DBG_PRINT(".");

        event = prcess_state_get();
        if(event == PRCESS_STATE_OK)
        {
            //**************************************//
                //DBG_PRINT("user result get \r\n");

            //**************************************//
            prcess_frame_buffer_add((INT32U *)csi_buf, 1);
        }

#if PPU_DRAW_EN == 1
        if(ppu_register_set)
        {
            ppu_buf = ppu_draw_go(PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, csi_buf);
            if(ppu_buf)
            {
                if(event != PRCESS_STATE_OK)
                    free_frame_buffer_add((INT32U *)csi_buf, 1);
                csi_buf = ppu_buf;
            }
        }
#endif
        PscalerBuffer = pscaler_frame_buffer_get(1);
        if(PscalerBuffer)
            fd_display_set_frame(csi_buf, PscalerBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, device_h_size, device_v_size);

        if(PscalerBuffer)
        {
            if(event == PRCESS_STATE_OK)
            {
                //**************************************//
                //DBG_PRINT("user draw image \r\n");

                //**************************************//
            }

            disp_frame_buffer_add((INT32U *)PscalerBuffer, 1);
        }
        //else
            //DBG_PRINT("@");
#if PPU_DRAW_EN == 1
        if(ppu_register_set)
            gplib_ppu_frame_buffer_add(ppu_register_set, csi_buf);
#else
        if(event != PRCESS_STATE_OK)
            free_frame_buffer_add((INT32U *)csi_buf, 1);
#endif
    }
}

static void disp_task_entry(void const *parm)
{
    INT32U display_buf;
    osEvent result;

    DBG_PRINT("disp_task_entry start \r\n");
    // Initialize display device
    drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);
#if PPU_DRAW_EN == 1
    ppu_draw_init();
#endif

    while(1)
    {
        result = osMessageGet(disp_frame_buffer_queue, osWaitForever);
        display_buf = result.value.v;
        if((result.status != osEventMessage) || !display_buf) {
            continue;
        }
        //DBG_PRINT("display_buf = 0x%x\r\n", display_buf);
        //DBG_PRINT("D");

        drv_l2_display_update(DISPLAY_DEVICE,display_buf);
        pscaler_frame_buffer_add((INT32U *)display_buf, 1);
    }
}

static void prcess_task_entry(void const *parm)
{
    INT32U prcess_buf;
    osEvent result;
#if CSI_MOTION_DETECTION_EN == 1
    INT32U md_buf;
#endif

    DBG_PRINT("prcess_task_entry start \r\n");
    //**************************************//
        //DBG_PRINT("user init add \r\n");
    //**************************************//

    while(1)
    {
        result = osMessageGet(prcess_frame_buffer_queue, osWaitForever);
        prcess_buf = result.value.v;
        if((result.status != osEventMessage) || !prcess_buf) {
            continue;
        }
        //DBG_PRINT("prcess_buf = 0x%x\r\n", prcess_buf);
        //DBG_PRINT("P");

        //**************************************//
        //DBG_PRINT("user code add \r\n");

        //**************************************//
        free_frame_buffer_add((INT32U *)prcess_buf, 1);
        prcess_state_post(PRCESS_STATE_OK);
    }
}

void GPM4_CSI_MD_Demo(void)
{
    osThreadDef_t csi_task = {"csi_task", csi_task_entry, osPriorityAboveNormal, 1, 8192};
    osThreadDef_t disp_task = {"disp_task", disp_task_entry, osPriorityNormal, 1, 8192};
    osThreadDef_t prcess_task = {"prcess_task", prcess_task_entry, osPriorityNormal, 1, 32768};
    osMessageQDef_t disp_q = {DISP_QUEUE_MAX, sizeof(INT32U), 0};
    osSemaphoreDef_t disp_sem = {0};

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

	if(sem_prcess_engine == NULL)
	{
		sem_prcess_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_prcess_engine)
		{
            DBG_PRINT("sem_prcess_engine error\r\n");
            while(1);
		}
		else
            DBG_PRINT("sem_prcess_engine = 0x%x\r\n", sem_prcess_engine);
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
#if CSI_MOTION_DETECTION_EN == 1
    if(md_free_buffer_queue == NULL)
	{
        md_free_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!md_free_buffer_queue)
		{
            DBG_PRINT("md_free_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("md_free_buffer_queue = 0x%x\r\n", md_free_buffer_queue);
	}
    if(md_ready_buffer_queue == NULL)
	{
        md_ready_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!md_ready_buffer_queue)
		{
            DBG_PRINT("md_ready_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("md_ready_buffer_queue = 0x%x\r\n", md_ready_buffer_queue);
	}

#endif

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
        osDelay(10);
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

    // ad key init
	adc_key_scan_init();

	while(1)
	{
		adc_key_scan();

		if(ADKEY_IO1)
		{
            DBG_PRINT("ad_key selection\r\n");
		}
	}
}



