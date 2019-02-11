#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_l1_cdsp.h"
#include "drv_l1_gpio.h"
#include "drv_l1_tft.h"
#include "drv_l1_fft.h"
#include "drv_l1_scaler.h"
#include "drv_l1_pscaler.h"
#include "drv_l1_csi.h"
#include "drv_l2_scaler.h"
#include "drv_l2_sensor.h"
#include "drv_l2_ad_key_scan.h"
#include "drv_l2_display.h"
#include "gplib.h"
#include "gp_stdlib.h"
#include "gplib_ppu.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "define.h"
#include "FaceDetectAP.h"
#include "RFMS.h"
#include "drv_l2_sccb.h"

#define CSI_PSCALER_STOP_EN                 1
#define NEW_OBJ_Y_MODE                      0
#define GPIO_OBJ_SET                        0
#define IMAGE_BIN_NUMBER                    10
#define IMAGE_SIZE                          (320*240*2)
#define C_PPU_DRV_FRAME_NUM		            3
#define PPU_TEXT_SIZE_HPIXEL                320
#define PPU_TEXT_SIZE_VPIXEL                240
#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define FRAME_BUF_ALIGN16                   0xF
#define FRAME_BUF_ALIGN8                    0x7
#define FRAME_BUF_ALIGN4                    0x3
#define OBJDETECT_MAX_RESULT		        128
#define RETURN(x)	                        {nRet = x; goto Return;}
#define DUMMY_BUFFER_ADDRESS                0x50000000
#if _SENSOR_H42_CDSP_MIPI == 1 || _SENSOR_H62_CDSP_MIPI == 1 || _SENSOR_GC1064_CDSP_MIPI == 1
#define SENSOR_SRC_WIDTH		            1280
#define SENSOR_SRC_HEIGHT		            720
#else
#define SENSOR_SRC_WIDTH		            640
#define SENSOR_SRC_HEIGHT		            480
#endif
#define PRCESS_SRC_WIDTH		            640
#define PRCESS_SRC_HEIGHT		            480
#define OBJ_STATE_OK                        0x80
#define MSG_PRCESS_TASK_EXIT                0xFF
#define DISP_QUEUE_MAX		                (C_PPU_DRV_FRAME_NUM+1)
#define GPIO_EN_PIN1                        IO_D6
#define CES_SD_EN                           1
#define POST_SCALE_USE                      PSCALER_B
// object recognize
#define RFMSDETECT_IMG_WIDTH	            256
#define RFMSDETECT_IMG_HEIGHT	            192
#define OBJ_DELAY_TIME                      30
#define C_AE_VALUE		 		            0x48
#define ABS(x)		                        ((x) >= 0 ? (x) : -(x))
#define ACK_OK			                    0
#define ACK_FAIL		                    (-1)

#define POST_MESSAGE(queue, message, ack_mbox, msc_time)\
{\
	{\
		INT32U send_msg;\
		osEvent result;\
		osStatus status;\
		result = osMessageGet(ack_mbox, 1);\
		send_msg = message;\
		status = osMessagePut(queue, (INT32U)&send_msg, msc_time);\
		if(status != osOK)\
		{\
			DBG_PRINT("PutMsg Fail!!!\r\n");\
			RETURN(STATUS_FAIL);\
		}\
		result = osMessageGet(ack_mbox, msc_time);\
		if((result.status != osEventMessage) || (result.value.v == ACK_FAIL))\
		{\
			DBG_PRINT("GetMsg ack Fail!!! message %x\r\n",message);\
			RETURN(STATUS_FAIL);\
		}\
	}\
}

#define OSQFlush(x)\
{\
    while(1) {\
        osEvent result;\
        result = osMessageGet(x, 1);\
        if(result.status != osEventMessage) {\
            break;\
        }\
    }\
}

typedef struct {
	INT32U FrameBuf;
	INT8S ResultCnt;
	INT8U EyeFlag;
	INT8U NoseFlag;
	INT8U MouthFlag;

	INT8U SmileFlag;
	INT8U FaceTrainingFlag;
	INT8U FaceIdentifyFlag;
	INT8U HandFlag;
	INT8U FistFlag;

	gpRect Result[64];
	gpRect Eye[2];
	gpRect Eyebrow[2];
	gpRect Nose[2];
	gpRect Mouth[4];

	INT8U ObjIdentifyFlag;
	INT8U MatchingCardNum;
	INT8U reserved0;
	INT8U reserved1;
} ObjDetResult_t;

typedef enum
{
        PPUAUTOFORMAT = 0,
        BITMAP_GRAY,
        BITMAP_YUYV,
        BITMAP_RGRB,
        CHARATER64_YUYV,
        CHARATER64_RGRB,
        CHARATER32_YUYV,
        CHARATER32_RGRB,
        CHARATER16_YUYV,
        CHARATER16_RGRB,
        CHARATER8_YUYV,
        CHARATER8_RGRB
} PPU_FORMAT;

typedef struct
{
	gpImage ScaleIn;
	gpImage ScaleOut;
} ObjFrame_t;

typedef struct {
	INT32S flag;
	void *workmem;
	void (*reset)(void *para);
	int (*alloc)(void);
	void (*free)(void *para);
	void (*run)(void *para, gpImage *org_img, void *pre_result);
	void (*get_result)(void *para, void *result);
} PostProc_t;

typedef struct {
	void *obj_track_WorkMem;
	void *obj_detect_WorkMem;
	void *rfms_detect_WorkMem;
	gpImage	image;

	//classify
	ClassifyData clf;
	INT32S xstep;
	INT32S scale;
	PostProc_t post_proc;

	//result
	INT8U no_face_cnt;
	INT8U gesture_mode;	//gesture detect use
	INT8U reserved0;
	INT8U reserved1;
	gpRect result[OBJDETECT_MAX_RESULT];
	INT32S count[OBJDETECT_MAX_RESULT];
} ObjDetect_t;

typedef struct {
	INT32S *rfms_WorkMem;
	CHAR   *obj_data_buf;
	gpImage	image;
	INT32S cardsN;
	INT32U extractionThre;
	INT32U matchingThre;
	INT32U matchingThreRangeV;
	INT32U minExtractionThre;
	INT32U incExtractionThre;
	INT32U decExtractionThre;
	INT32U startMatchingPointN;
	INT32U matchingCardNum;
} RfmsIdentify_t;

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

typedef struct {
	INT32U ppu_frame_workmem;
	INT32U ppu_narray_workmem;
	INT32U ppu_pscaler_workmem;
	INT32U ppu_draw_src_workmem;
    INT32U csi_prcess_workmem;
    INT32U obj_prcess_workmem;
    INT32U obj_y_workmem;
    INT32U obj_smooth_workmem;
    INT32U obj_data_workmem;
} prcess_mem_t;

static prcess_mem_t prcess_mem_structure;
static prcess_mem_t *prcess_mem_set;
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static INT32U PPU_FRAME_BUFFER_BASE,fd_org_buf,text1_narray,disp_buffer_post,obj_display_cnt,compare_out_buffer = 0;
static INT32U scaler_int_w_step,pscaler_buf,pscaler_disp_buffer_post,YValue,csi_init_en = 0;
static INT32U disp_h_size,disp_v_size,obj_image_number,display_bin_max,csi_pscaler_stop;
static ObjDetResult_t obj_result_set = {0};
static ObjDetect_t *ObjWorkMem_ptr = NULL;

// os
static osThreadId obj_id;
static osThreadId csi_id;
static osThreadId disp_id;
static xQueueHandle free_frame_buffer_queue = NULL;
static xQueueHandle display_frame_buffer_queue = NULL;
static xQueueHandle obj_frame_buffer_queue = NULL;
static xQueueHandle obj_free_frame_buffer_queue = NULL;
static xQueueHandle ppu_frame_buffer_queue = NULL;
static xQueueHandle pscaler_free_buffer_queue = NULL;
static xQueueHandle pscaler_display_buffer_queue = NULL;
static xQueueHandle obj_free_integral_y_buffer_queue = NULL;
static xQueueHandle obj_proc_integral_y_buffer_queue = NULL;
static xQueueHandle ppu_draw_free_buffer_queue = NULL;
static xQueueHandle obj_state_queue = NULL;
static xQueueHandle obj_task_ack_m = NULL;
static xQueueHandle csi_task_ack_m = NULL;
static xQueueHandle disp_task_ack_m = NULL;
static xSemaphoreHandle sem_prcess_engine = NULL;
static xSemaphoreHandle sem_scaler_engine = NULL;
static PSCALER_PARAM_STRUCT PScalerParam = {0};

static INT32S pscaler_free_buffer_add(INT32U *frame_buf);
static INT32S pscaler_free_buffer_get(void);
static INT32S pscaler_display_buffer_add(INT32U *frame_buf);
static INT32S pscaler_display_frame_buffer_get(void);

#if _SENSOR_GC0308_CSI == 1
extern INT8U GC0308_ReadY(void);
extern INT8U GC0308_SetY(INT8U Value);
#endif

static void obj_recognize_free(ObjDetect_t *odWorkMem);

INT32U user_malloc_function(INT32U size)
{
    INT32U temp = 0;

    temp = (INT32U)gp_malloc_align((size+64), 32);

    if(temp)
    {
        temp = (INT32U)((temp + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    }

    return temp;
}

INT32S user_fftobj_hw_function(INT32U rfms_min_base, INT32U obj_num, INT32U objectDes_ptr, INT32U imageDes_ptr, INT32U *hw_min_error, INT32U *hw_min_error_id)
{
    INT32U min_error,min_id;
    INT32U minError = rfms_min_base;

    //DBG_PRINT("sw_min_base = 0x%x\r\n", rfms_min_base);

    if(!objectDes_ptr || !imageDes_ptr || !hw_min_error || !hw_min_error_id)
        return -1;

    if(compare_out_buffer == 0)
    {
        compare_out_buffer = (INT32U)drv_l1_obj_malloc(2048);
        if(compare_out_buffer == 0)
            return -1;
        compare_out_buffer = (INT32U)((compare_out_buffer + FRAME_BUF_ALIGN4) & ~FRAME_BUF_ALIGN4);
        drv_l1_obj_output_buffer_addr_set(compare_out_buffer);
    }

    drv_l1_obj_scale_set(obj_num - 1);
    drv_l1_obj_compare_buffer_addr_set((unsigned int)objectDes_ptr);
    drv_l1_obj_database_addr_set((unsigned int)imageDes_ptr);
    drv_l1_obj_start();
    drv_l1_obj_end(1);
    min_error = drv_l1_obj_minimum_error_get();
    min_id = drv_l1_obj_minimum_error_id_get();

    if( min_error < minError )
    {
        *hw_min_error = min_error;
        *hw_min_error_id = (min_id-1);
    }
    //osDelay(1);

    return 0;
}

static void gpio1_set(int mode)
{
    gpio_write_io(GPIO_EN_PIN1, mode);
}

static void obj_gpio_init(void)
{
    gpio_init_io (GPIO_EN_PIN1, GPIO_OUTPUT);
    gpio_set_port_attribute(GPIO_EN_PIN1, 1);
}

static INT32S prcess_mem_free(void)
{
    INT32U i;

    if(prcess_mem_set)
    {
        if(prcess_mem_set->csi_prcess_workmem)
        {
            gp_free((void *)prcess_mem_set->csi_prcess_workmem);
            prcess_mem_set->csi_prcess_workmem = 0;
        }
        if(prcess_mem_set->obj_prcess_workmem)
        {
            gp_free((void *)prcess_mem_set->obj_prcess_workmem);
            prcess_mem_set->obj_prcess_workmem = 0;
        }
        if(prcess_mem_set->ppu_frame_workmem)
        {
            gp_free((void *)prcess_mem_set->ppu_frame_workmem);
            prcess_mem_set->ppu_frame_workmem = 0;
        }
        if(prcess_mem_set->ppu_narray_workmem)
        {
            gp_free((void *)prcess_mem_set->ppu_narray_workmem);
            prcess_mem_set->ppu_narray_workmem = 0;
        }
        if(prcess_mem_set->ppu_draw_src_workmem)
        {
            gp_free((void *)prcess_mem_set->ppu_draw_src_workmem);
            prcess_mem_set->ppu_draw_src_workmem = 0;
        }
        if(prcess_mem_set->obj_data_workmem)
        {
            gp_free((void *)prcess_mem_set->obj_data_workmem);
            prcess_mem_set->obj_data_workmem = 0;
        }
#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
        if(prcess_mem_set->ppu_pscaler_workmem)
        {
            gp_free((void *)prcess_mem_set->ppu_pscaler_workmem);
            prcess_mem_set->ppu_pscaler_workmem = 0;
        }
#endif
#if NEW_OBJ_Y_MODE == 1
        if(prcess_mem_set->obj_y_workmem)
        {
            gp_free((void *)prcess_mem_set->obj_y_workmem);
            prcess_mem_set->obj_y_workmem = 0;
        }
        if(prcess_mem_set->obj_smooth_workmem)
        {
            gp_free((void *)prcess_mem_set->obj_smooth_workmem);
            prcess_mem_set->obj_smooth_workmem = 0;
        }
#endif
        prcess_mem_set = NULL;
    }

    if(compare_out_buffer)
    {
        gp_free((void *)compare_out_buffer);
        compare_out_buffer = 0;
    }

    if(ObjWorkMem_ptr)
    {
        obj_recognize_free(ObjWorkMem_ptr);
        gp_free((void *)ObjWorkMem_ptr);
        ObjWorkMem_ptr = 0;
    }

    return 0;
}

static INT32S obj_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    // fd task
    POST_MESSAGE(obj_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, obj_task_ack_m, 5000);
Return:
    OSQFlush(obj_frame_buffer_queue);
    OSQFlush(obj_free_frame_buffer_queue);
    OSQFlush(obj_task_ack_m);
    OSQFlush(pscaler_free_buffer_queue);
    OSQFlush(pscaler_display_buffer_queue);
    OSQFlush(obj_state_queue);

    return nRet;
}

static INT32S disp_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    // disp task
    POST_MESSAGE(ppu_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, disp_task_ack_m, 5000);
Return:
    OSQFlush(ppu_frame_buffer_queue);
    OSQFlush(disp_task_ack_m);

    return nRet;
}

static INT32S csi_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    // csi task
    POST_MESSAGE(display_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, csi_task_ack_m, 5000);
Return:
    OSQFlush(display_frame_buffer_queue);
    OSQFlush(free_frame_buffer_queue);
    OSQFlush(ppu_draw_free_buffer_queue);
    OSQFlush(csi_task_ack_m);

    return nRet;
}

static INT32S prcess_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    nRet = csi_task_exit();
    if(nRet != STATUS_OK)
        DBG_PRINT("csi_task_exit fail\r\n");

    nRet = disp_task_exit();
    if(nRet != STATUS_OK)
        DBG_PRINT("disp_task_exit fail\r\n");

    nRet = obj_task_exit();
    if(nRet != STATUS_OK)
        DBG_PRINT("fd_task_exit fail\r\n");

    prcess_mem_free();

    return nRet;
}

static void drv_scaler_lock(void)
{
    if(sem_scaler_engine){
        osSemaphoreWait(sem_scaler_engine, osWaitForever);
    }
}

static void drv_scaler_unlock(void)
{
    if(sem_scaler_engine){
        osSemaphoreRelease(sem_scaler_engine);
    }
}

static int objDetectRY( unsigned char *img_ptr, int imageWidth,  int imageHeight)
{
	int i, j;
	int MCnt, partion;
	int avgY;

	MCnt = 0;
	partion = divFun(imageWidth, 3);
	avgY = 0;

	j = imageHeight;
	do
	{
		i = imageWidth;
        do
		{
			if( (*img_ptr) != 0)
			{
				if(i > partion )
				{
				   avgY += (*img_ptr);
				   MCnt++;
				}
			}
			img_ptr += 4;

			i -= 4;
		}
		while(i != 0);

		img_ptr += imageWidth;

	  j-= 2;
	}
	while(j != 0);


	avgY = divFun( avgY, MCnt);

    return avgY;
}

static void obj_recognize_free(ObjDetect_t *odWorkMem)
{
	RfmsIdentify_t *rfms = (RfmsIdentify_t *)odWorkMem->rfms_detect_WorkMem;

	if(odWorkMem->rfms_detect_WorkMem) {
		if(rfms->rfms_WorkMem) {
			gp_free((void *)rfms->rfms_WorkMem);
			rfms->rfms_WorkMem = 0;
		}

		if(rfms->obj_data_buf) {
			gp_free((void *)rfms->obj_data_buf);
			rfms->obj_data_buf = 0;
		}

		gp_free((void *)odWorkMem->rfms_detect_WorkMem);
		odWorkMem->rfms_detect_WorkMem = 0;
	}
}

static INT32U obj_dispbin_load_data(INT32U data_buf, INT32S bin_number)
{
	INT8U path[128];
	INT16S fd;
	INT32S size, nRet = 0;
	struct sfn_info file_info;

    sprintf((char *)path, (const char *)"K:\\Data\\image\\display%d.bin", bin_number);
	fd = fs_open(path, O_RDONLY);
	if(fd < 0 || !data_buf) {
		RETURN(STATUS_FAIL);
	}

    sfn_stat(fd, &file_info);

	nRet = fs_read(fd, data_buf, file_info.f_size);
	if(nRet != file_info.f_size) {
		RETURN(STATUS_FAIL);
	}

	fs_close(fd);
Return:
	if(nRet < 0) {
		fs_close(fd);
		if(data_buf) {
			gp_free((void *)data_buf);
		}
		return 0;
	}

	return data_buf;
}

static INT32U obj_recognize_load_data(INT32S *card_number)
{
	INT16S fd;
	INT32S nRet,size;
	INT32U *ptr, data_buf = 0;
    struct sfn_info file_info;

	fd = fs_open("K:\\Data\\RF_Database\\objDatabase.bin", O_RDONLY);
	if(fd < 0) {
		RETURN(STATUS_FAIL);
	}

    sfn_stat(fd, &file_info);

	data_buf = (INT32U)gp_malloc_align((file_info.f_size+16), 16);

	if(data_buf == 0) {
		RETURN(STATUS_FAIL);
	}

    data_buf = (INT32U)((data_buf + FRAME_BUF_ALIGN16) & ~FRAME_BUF_ALIGN16);
	DBG_PRINT("obj_database_buf = 0x%x\r\n", data_buf);

	nRet = fs_read(fd, data_buf, file_info.f_size);
	if(nRet != file_info.f_size) {
		RETURN(STATUS_FAIL);
	}

	ptr = (INT32U *)data_buf;
	obj_image_number = *ptr;
	if(obj_image_number == 0)
        RETURN(STATUS_FAIL);

    DBG_PRINT("card_number = 0x%x\r\n", obj_image_number);
	fs_close(fd);

	display_bin_max = obj_image_number / IMAGE_BIN_NUMBER;
	*card_number = obj_image_number;

Return:
	if(nRet < 0) {
		fs_close(fd);
		if(data_buf) {
			gp_free((void *)data_buf);
		}
		return 0;
	}

	return data_buf;
}

static INT32S obj_recognize_init(ObjDetect_t *odWorkMem)
{
	INT32S nRet;
	INT32U mem_size;
	RfmsIdentify_t *rfms;

	// rmfs variable alloc
	mem_size = sizeof(RfmsIdentify_t);
	odWorkMem->rfms_detect_WorkMem = (void *)gp_malloc_align(mem_size, 16);
	if(odWorkMem->rfms_detect_WorkMem == 0) {
		DBG_PRINT("Fail to allocate odWorkMem->rfms_detect_WorkMem\r\n");
		return -1;
	}
	DBG_PRINT("rfms_detect_WorkMem = 0x%x\r\n", odWorkMem->rfms_detect_WorkMem);
	rfms = (RfmsIdentify_t *)odWorkMem->rfms_detect_WorkMem;
	gp_memset((INT8S*)rfms, 0x00, mem_size);

	// load data buffer
	rfms->obj_data_buf = (CHAR *)obj_recognize_load_data(&rfms->cardsN);
	if(rfms->obj_data_buf == 0) {
		RETURN(STATUS_FAIL);
	}

	// rmfs workmem alloc
	mem_size = RFMS_get_memory_size(odWorkMem->image.width, odWorkMem->image.height);
	rfms->rfms_WorkMem = (void *)gp_malloc_align(mem_size+1024, 16);
	if(rfms->rfms_WorkMem == 0) {
		DBG_PRINT("Fail to allocate rfms->rfms_WorkMem\r\n");
		RETURN(STATUS_FAIL);
	}
	DBG_PRINT("rfms_WorkMem = 0x%x\r\n", rfms->rfms_WorkMem);
	gp_memset((INT8S*)rfms->rfms_WorkMem, 0x00, mem_size);

	rfms->image.width = odWorkMem->image.width;
	rfms->image.height = odWorkMem->image.height;
	rfms->image.widthStep = odWorkMem->image.widthStep;
	rfms->image.ch = odWorkMem->image.ch;
	rfms->image.format = odWorkMem->image.format;

	rfms->extractionThre = 400;
	rfms->matchingThre = 23000;
	rfms->matchingThreRangeV = 3000;
	rfms->minExtractionThre = 100;
	rfms->incExtractionThre = 100;
	rfms->decExtractionThre = 100;
	rfms->startMatchingPointN = 70;

	// rfms init
#if GP_OBJ_HW_EN == 1
    RFMS_init_HW(rfms->image.width,
			rfms->image.height,
			rfms->rfms_WorkMem,
			odWorkMem->image.width,
			odWorkMem->image.height,
			rfms->obj_data_buf);
    drv_l1_obj_init();
    drv_l1_obj_user_malloc_set(user_malloc_function);
    rfms_fftobj_hw_set(user_fftobj_hw_function);
#else
	RFMS_init(rfms->image.width,
			rfms->image.height,
			rfms->rfms_WorkMem,
			odWorkMem->image.width,
			odWorkMem->image.height,
			rfms->cardsN,
			rfms->obj_data_buf);
#endif
	RFMS_ParamSet(rfms->rfms_WorkMem,
				rfms->extractionThre,
				rfms->matchingThre,
				rfms->matchingThreRangeV,
				rfms->minExtractionThre,
				rfms->incExtractionThre,
				rfms->decExtractionThre,
				rfms->startMatchingPointN);

	nRet = STATUS_OK;
Return:
	if(nRet < 0) {
		obj_recognize_free(odWorkMem);
	}

	return nRet;
}

static INT32S obj_recognize_alloc(void)
{
	INT32S nRet;
	ObjDetect_t *odWorkMem;

	odWorkMem = (ObjDetect_t *)gp_malloc_align(sizeof(ObjDetect_t), 16);
	if(odWorkMem == 0) {
		DBG_PRINT("Fail to allocate odWorkMem\r\n");
		return 0;
	}
	DBG_PRINT("odWorkMem = 0x%x\r\n", odWorkMem);
	gp_memset((INT8S*)odWorkMem, 0, sizeof(ObjDetect_t));

	// image info setting
	odWorkMem->image.width     = RFMSDETECT_IMG_WIDTH;
	odWorkMem->image.height    = RFMSDETECT_IMG_HEIGHT;
	odWorkMem->image.ch        = 1;
	odWorkMem->image.widthStep = RFMSDETECT_IMG_WIDTH;
	odWorkMem->image.format    = IMG_FMT_GRAY;

	//post proc setting
	odWorkMem->post_proc.flag  = 0;
	odWorkMem->post_proc.run   = 0;
	odWorkMem->post_proc.alloc = 0;
	odWorkMem->post_proc.free  = 0;
	odWorkMem->post_proc.reset = 0;
	odWorkMem->post_proc.get_result = 0;

	// setting cascade type
	odWorkMem->clf.obj_type 		 = 0xFF;
	odWorkMem->clf.data              = 0;
	odWorkMem->clf.stage_of_classify = 0;
	odWorkMem->clf.num_of_classify   = 0;

	if(obj_recognize_init(odWorkMem) < 0) {
		RETURN(-1);
	}

	nRet = STATUS_OK;
Return:
	if(nRet < 0) {
		obj_recognize_free((void *)odWorkMem);
		return 0;
	}

	return (INT32S)odWorkMem;
}

static void obj_recognize_proc(ObjDetect_t *odWorkMem)
{
	INT32S objectTotal;
	RfmsIdentify_t *rfms = (RfmsIdentify_t *)odWorkMem->rfms_detect_WorkMem;

	//image can not be mirror or flip
	objectTotal = RFMS_extractPoint(rfms->rfms_WorkMem, &odWorkMem->image);
    osDelay(1);
#if GPIO_OBJ_SET == 1
	gpio1_set(1);
#endif
    if(objectTotal < 200 && objectTotal > rfms->startMatchingPointN)
        rfms->matchingCardNum = RFMS_findPairs(rfms->rfms_WorkMem, objectTotal);
    else
        rfms->matchingCardNum = 0;
    osDelay(1);
#if GPIO_OBJ_SET == 1
	gpio1_set(0);
#endif
	DBG_PRINT("objectTotal = %d, matchingCardNum = %d \r\n", objectTotal, rfms->matchingCardNum);
}

static void obj_recognize_get_result(void *od_work_mem, void *objresult)
{
	ObjDetect_t *odWorkMem = (ObjDetect_t *)od_work_mem;
	ObjDetResult_t *result = (ObjDetResult_t *)objresult;
	RfmsIdentify_t *rfms = (RfmsIdentify_t *)odWorkMem->rfms_detect_WorkMem;

	if(rfms->matchingCardNum > 0) {
		result->ObjIdentifyFlag = 1;
		result->MatchingCardNum = rfms->matchingCardNum;
		DBG_PRINT("CardNumber = %d\r\n", result->MatchingCardNum);
	} else {
		result->ObjIdentifyFlag = 0;
		result->MatchingCardNum = 0;
	}
}

static INT32U fd_scaler_step_get(void)
{
    return (scaler_int_w_step - 1);
}

static INT32S scalerStart(INT32U mode, gpImage *src, gpImage *dst)
{
    INT32S ret;
    gpRect clip;
	gpImage temp;

    drv_scaler_lock();
	clip.x = 0;
	clip.y = 0;
	clip.width = src->width;
	clip.height = src->height;
    scaler_int_w_step = dst->width;


	if(mode)
        ret = drv_l2_FD_scaler_clip(0, 1, src, dst, &clip);
    else
        ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);
    drv_scaler_unlock();

    return ret;
}

static INT32S scalerEnd(void)
{
        return 0;
}

static INT32S object_detect_set_integral_y_frame(INT32U in_buffer, INT32U frame_buffer, INT16U w, INT16U h)
{
	INT8U err;
	INT32S nRet;
	ObjFrame_t *pInput,Input;

	if(frame_buffer == 0 || in_buffer == 0) {
		RETURN(STATUS_FAIL);
	}
    pInput = (ObjFrame_t *)&Input;
	if(pInput == 0) {
		RETURN(STATUS_FAIL);
	}


	pInput->ScaleIn.width = w;
	pInput->ScaleIn.height = h;
    pInput->ScaleIn.ch  = 1;
    pInput->ScaleIn.widthStep = w;
	pInput->ScaleIn.format = IMG_FMT_GRAY;
	pInput->ScaleIn.ptr = (INT8U *)in_buffer;

	pInput->ScaleOut.width = w;
	pInput->ScaleOut.height = h;
    pInput->ScaleOut.ch  = 1;
    pInput->ScaleOut.widthStep = w;
	pInput->ScaleOut.format = IMG_FMT_INTEGRAL_Y;
	pInput->ScaleOut.ptr = (INT8U *)frame_buffer;

	// scale
	scalerStart(1, &pInput->ScaleIn, &pInput->ScaleOut);
	scalerEnd();
	nRet = STATUS_OK;
Return:

	return nRet;
}

static INT32S object_detect_set_y_frame(INT32U in_buffer, INT32U frame_buffer, INT16U w, INT16U h, INT16U out_w, INT16U out_h, INT32U infmt)
{
	INT8U err;
	INT32S nRet;
	ObjFrame_t *pInput,Input;

	if(frame_buffer == 0 || in_buffer == 0) {
		RETURN(STATUS_FAIL);
	}
    pInput = (ObjFrame_t *)&Input;
	if(pInput == 0) {
		RETURN(STATUS_FAIL);
	}

	switch(infmt)
	{
        case BITMAP_GRAY:
                infmt = IMG_FMT_GRAY;
                break;

        case BITMAP_YUYV:
                infmt = IMG_FMT_UYVY;
                break;

        default:
                RETURN(STATUS_FAIL);
	}

	pInput->ScaleIn.width = w;
	pInput->ScaleIn.height = h;
    pInput->ScaleIn.ch  = 2;
    pInput->ScaleIn.widthStep = w * 2;
	pInput->ScaleIn.format = IMG_FMT_UYVY;
	pInput->ScaleIn.ptr = (INT8U *)in_buffer;

	pInput->ScaleOut.width = out_w;
	pInput->ScaleOut.height = out_h;
	if(infmt == IMG_FMT_GRAY) {
		pInput->ScaleOut.ch  = 1;
		pInput->ScaleOut.widthStep = out_w;
	} else {
		pInput->ScaleOut.ch  = 2;
		pInput->ScaleOut.widthStep = out_w * 2;
	}
	pInput->ScaleOut.format = infmt;
	pInput->ScaleOut.ptr = (INT8U *)frame_buffer;

	// scale
	scalerStart(0, &pInput->ScaleIn, &pInput->ScaleOut);
	scalerEnd();
	nRet = STATUS_OK;
Return:

	return nRet;
}

static INT32S obj_display_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
	INT32S retStatus;

	if(frame_buffer == 0) {
		return -1;
	}

    drv_l1_pscaler_init(POST_SCALE_USE);
    drv_l1_pscaler_input_source_set(POST_SCALE_USE,PIPELINE_SCALER_INPUT_SOURCE_PPUFB);
    drv_l1_pscaler_input_pixels_set(POST_SCALE_USE,in_w,in_h);
    drv_l1_pscaler_output_pixels_set(POST_SCALE_USE,((in_w*65536)/out_w),out_w,((in_h*65536)/out_h),out_h);
    drv_l1_pscaler_output_fifo_line_set(POST_SCALE_USE,out_w,0);
    drv_l1_pscaler_interrupt_set(POST_SCALE_USE,PIPELINE_SCALER_INT_ENABLE_422GP420_FRAME_END);
    drv_l1_pscaler_input_format_set(POST_SCALE_USE,PIPELINE_SCALER_INPUT_FORMAT_YUYV);
    drv_l1_pscaler_output_A_buffer_set(POST_SCALE_USE,frame_buffer);
    drv_l1_pscaler_output_format_set(POST_SCALE_USE,PIPELINE_SCALER_OUTPUT_FORMAT_YUYV);
    drv_l1_pscaler_start(POST_SCALE_USE);
    gplib_ppu_go_and_wait_done(ppu_register_set);
    do
    {
        retStatus = drv_l1_pscaler_status_get(POST_SCALE_USE);

        if(retStatus & PIPELINE_SCALER_STATUS_FRAME_DONE)
            break;
        osDelay(1);
    }
    while(1);
    drv_l1_pscaler_stop(POST_SCALE_USE);
    pscaler_display_buffer_add((INT32U *)frame_buffer);

    return 0;
}

static void fd_ppu_init(void)
{
    INT32U i,frame_size,buffer_ptr;
#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
    FB_LOCK_STRUCT fb_lock_set;
#endif

    // Initialize display device
    drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);

#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&disp_h_size, (INT16U *)&disp_v_size);
    frame_size = disp_h_size*disp_v_size*2;
    pscaler_buf = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(!pscaler_buf)
    {
        DBG_PRINT("pscaler_buf fail \r\n");
        while(1);
    }
    prcess_mem_set->ppu_pscaler_workmem = pscaler_buf;
    pscaler_buf = (INT32U)((pscaler_buf + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(pscaler_buf + (i*frame_size));
            pscaler_free_buffer_add((INT32U *)buffer_ptr);
            DBG_PRINT("PscalerBuffer:0x%X \r\n",buffer_ptr);
    }
#endif

    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;

    //Initiate PPU hardware engine and PPU register set structure
    gplib_ppu_init(ppu_register_set);

#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
    fb_lock_set.color1 = PPU_FMT_YUYV;
    fb_lock_set.h_size1 = PPU_TEXT_SIZE_HPIXEL;
    fb_lock_set.v_size1 = PPU_TEXT_SIZE_VPIXEL;
    fb_lock_set.color2 = PPU_FMT_YUYV;
    fb_lock_set.h_size2 = disp_h_size;
    fb_lock_set.v_size2 = disp_v_size;
    gplib_ppu_fb_lock_process_enable_set(ppu_register_set,(FB_LOCK_STRUCT *)&fb_lock_set);
    gplib_ppu_post_process_enable_set(ppu_register_set, 1);
#endif

    //Now configure PPU software structure
    gplib_ppu_enable_set(ppu_register_set, 1);					            // Enable PPU

    //TV frame mode
    gplib_ppu_non_interlace_set(ppu_register_set, 0);			            // Set non-interlace mode
    gplib_ppu_frame_buffer_mode_set(ppu_register_set, 1, 0);		        // Enable TV/TFT frame buffer mode

    //PPU setting
    gplib_ppu_fb_format_set(ppu_register_set, 1, 1);			            // Set PPU output frame buffer format to YUYV
    gplib_ppu_vga_mode_set(ppu_register_set, 0);							// Disable VGA mode
    gplib_ppu_resolution_set(ppu_register_set, C_TFT_RESOLUTION_320X240);	// Set display resolution to 640x480
#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
    gplib_ppu_free_size_set(ppu_register_set, 0, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL);
#endif
    gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
    gplib_ppu_long_burst_set(ppu_register_set, 1);
#if (DISPLAY_DEVICE == DISDEV_TFT)
    gplib_ppu_tft_long_burst_set(ppu_register_set, 1);
#endif

    //Frame buffer malloc
    frame_size = (PPU_TEXT_SIZE_HPIXEL * PPU_TEXT_SIZE_VPIXEL * 2);
    PPU_FRAME_BUFFER_BASE = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+64), 64);
    if(!PPU_FRAME_BUFFER_BASE)
    {
        DBG_PRINT("PPU_FRAME_BUFFER_BASE fail \r\n");
        while(1);
    }
    prcess_mem_set->ppu_frame_workmem = PPU_FRAME_BUFFER_BASE;
    PPU_FRAME_BUFFER_BASE = (INT32U)((PPU_FRAME_BUFFER_BASE + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    drv_l2_display_update(DISPLAY_DEVICE, PPU_FRAME_BUFFER_BASE);
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(PPU_FRAME_BUFFER_BASE + (i*frame_size));
            gplib_ppu_frame_buffer_add(ppu_register_set, buffer_ptr);
            DBG_PRINT("PPUBuffer:0x%X \r\n",buffer_ptr);
    }

    // Now configure TEXT relative elements
    gplib_ppu_text_compress_disable_set(ppu_register_set, 1);	                    // Disable TEXT1/TEXT2 horizontal/vertical compress function
    gplib_ppu_text_direct_mode_set(ppu_register_set, 0);			                // Disable TEXT direct address mode

    //text 2 2D
    gplib_ppu_text_init(ppu_register_set, C_PPU_TEXT1);
    text1_narray = (INT32U)gp_malloc_align(1024+64,4);
    if(!text1_narray)
    {
        DBG_PRINT("text1_narray fail \r\n");
        while(1);
    }
    prcess_mem_set->ppu_narray_workmem = text1_narray;
    text1_narray = (INT32U)((text1_narray + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    gplib_ppu_text_number_array_ptr_set(ppu_register_set, C_PPU_TEXT1, (INT32U)text1_narray);	 // Set TEXT number array address
    gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT1, 1);	                        // Enable TEXT
    gplib_ppu_yuv_type_set(ppu_register_set, 3);								     // Set 32-bit color format to Y1UY0V
    gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT1, 1);			     // Enable bitmap mode
    gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT1, 1);	    // Get TEXT attribute from register
    gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT1, 1, 3);				     // Set TEXT color to YUYV
    gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 0);			             // Set TEXT size to 1024x512
    gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT1, 0);				    // Set TEXT segment address
    gplib_ppu_sprite_init(ppu_register_set);
    gplib_ppu_sprite_enable_set(ppu_register_set, 0);
}

static void fd_ppu_go(INT32U x, INT32U y, INT32U frame_buffer)
{
    gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, x, y, (INT32U)frame_buffer);	// Calculate Number array
    // Start PPU and wait until PPU operation is done
#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
    pscaler_buf = pscaler_free_buffer_get();
    if(pscaler_buf)
        obj_display_set_frame(0,pscaler_buf,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,disp_h_size,disp_v_size);
#else
    gplib_ppu_go_and_wait_done(ppu_register_set);
#endif
}

static void mazePscalerSet(PSCALER_PARAM_STRUCT* pPScalerParam)
{
	INT32U widthFactor,heightFactor;

#if _SENSOR_H42_CDSP_MIPI == 1 || _SENSOR_H62_CDSP_MIPI == 1
    INT32U x_start,y_start;
    #define CENTER_WIDTH        960
#endif

	drv_l1_pscaler_init(pPScalerParam->pScalerNum);

#if _SENSOR_H42_CDSP_MIPI == 1 || _SENSOR_H62_CDSP_MIPI == 1
	x_start = (pPScalerParam->inWidth - CENTER_WIDTH)/2;
	y_start = 0;

	widthFactor = ((CENTER_WIDTH*65536)/pPScalerParam->outWidth);
	heightFactor = ((pPScalerParam->inHeight*65536)/pPScalerParam->outHeight);

	drv_l1_pscaler_input_X_start_set(pPScalerParam->pScalerNum,x_start);
	drv_l1_pscaler_input_Y_start_set(pPScalerParam->pScalerNum,y_start);
#else
	widthFactor = ((pPScalerParam->inWidth*65536)/pPScalerParam->outWidth);
	heightFactor = ((pPScalerParam->inHeight*65536)/pPScalerParam->outHeight);
#endif

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

static INT32S free_frame_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(free_frame_buffer_queue, (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S free_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame = 0;

	while (!frame) {
        result = osMessageGet(free_frame_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status == osEventMessage) {
            break;
        }
        else {
            frame = 0;
            break;
        }
		frame = 0;
	}

	return frame;
}

static INT32S free_frame_buffer_get_ISR(void)
{
    osEvent result;
	INT32S frame = 0;

    result = osMessageGet(free_frame_buffer_queue, 10);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
    }

	return frame;
}

static INT32S disp_frame_buffer_add_ISR(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(display_frame_buffer_queue, (uint32_t)&event, 10);

	return temp;
}

static INT32S disp_frame_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(display_frame_buffer_queue, (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S disp_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
	while (!frame) {
        result = osMessageGet(display_frame_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status == osEventMessage) {
            break;
        }
        else {
            frame = 0;
            break;
        }
		frame = 0;
	}

	return frame;
}

static void PScaler_Callback_ISR_AutoZoom(INT32U PScaler_Event)
{
	INT32U dispBuf,temp;

	if(csi_pscaler_stop)
	{
        drv_l1_pscaler_stop(PScalerParam.pScalerNum);
        drv_l1_pscaler_clk_ctrl(PScalerParam.pScalerNum, 0);
        csi_pscaler_stop = 0;
	}
	else
	{
        if(PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
        {
            dispBuf = free_frame_buffer_get_ISR();
            if(dispBuf)
            {
                temp = drv_l1_pscaler_output_A_buffer_get(PScalerParam.pScalerNum);
                disp_frame_buffer_add_ISR((INT32U *)temp);
                drv_l1_pscaler_output_A_buffer_set(PScalerParam.pScalerNum,(INT32U)dispBuf);
            }
            //else
                //DBG_PRINT("A");
        }
        else if(PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
        {
            dispBuf = free_frame_buffer_get_ISR();
            if(dispBuf)
            {
                temp = drv_l1_pscaler_output_B_buffer_get(PScalerParam.pScalerNum);
                disp_frame_buffer_add_ISR((INT32U *)temp);
                drv_l1_pscaler_output_B_buffer_set(PScalerParam.pScalerNum,(INT32U)dispBuf);
            }
            //else
                //DBG_PRINT("B");
        }
	}
}

static void mazeTest_Preview_PScaler(void)
{
	CHAR *p;
    INT8U  PScalerNum;
	INT32U dispBuffer,temp;
	INT32U i,csiBuffer,*ptr;
	INT32U dispBufferSize,csi_mode;
	INT32S ret;
	drv_l2_sensor_ops_t *pSencor;
	drv_l2_sensor_info_t *pInfo;
    drv_l2_sensor_para_t *pPara;

	PScalerNum = PSCALER_A;
	dispBufferSize = PRCESS_SRC_WIDTH*PRCESS_SRC_HEIGHT*2;

    csiBuffer = DUMMY_BUFFER_ADDRESS;

	dispBuffer = (INT32U) gp_malloc_align(((dispBufferSize*DISP_QUEUE_MAX)+64), 32);
    if(dispBuffer == 0)
    {
        DBG_PRINT("dispBuffer fail\r\n");
        while(1);
    }
    prcess_mem_set->csi_prcess_workmem = dispBuffer;
	dispBuffer = (INT32U)((dispBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<DISP_QUEUE_MAX; i++)
	{
		temp = (dispBuffer+i*dispBufferSize);
		free_frame_buffer_add((INT32U *)temp);
		DBG_PRINT("CSIBuffer:0x%X \r\n",temp);
	}

    // sensor init
	drv_l2_sensor_init();
    pSencor = drv_l2_sensor_get_ops(0);
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
	PScalerParam.pScalerNum = PScalerNum;
	PScalerParam.inBuffer = csiBuffer;
	PScalerParam.outBuffer_A = free_frame_buffer_get();
	PScalerParam.outBuffer_B = free_frame_buffer_get();

	PScalerParam.inWidth = SENSOR_SRC_WIDTH;
	PScalerParam.inHeight = SENSOR_SRC_HEIGHT;

	PScalerParam.outWidth = PRCESS_SRC_WIDTH;
	PScalerParam.outHeight = PRCESS_SRC_HEIGHT;

	PScalerParam.outLineCount = PRCESS_SRC_HEIGHT;

	PScalerParam.inFormat = PIPELINE_SCALER_INPUT_FORMAT_YUYV;
	PScalerParam.outFormat = PIPELINE_SCALER_OUTPUT_FORMAT_YUYV;

	PScalerParam.intEnableFlag = PIPELINE_SCALER_INT_ENABLE_ALL;
    PScalerParam.callbackFunc = PScaler_Callback_ISR_AutoZoom;

    pPara = drv_l2_sensor_get_para();
    if (csi_mode == CSI_INTERFACE)
    {
        // CSI
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CSI;
        // Open CSI data path
        drvl1_csi_input_pscaler_set(1);
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CSI;
    }
    else
    {
        // CDSP
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CDSP;
        // Open CDSP data path
        drv_l1_CdspSetYuvPscalePath(1);
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CDSP;
    }

#if CSI_PSCALER_STOP_EN == 1
	if(csi_init_en == 0)
	{
        pSencor->init();
        pSencor->stream_start(temp, csiBuffer, csiBuffer);
        csi_init_en = 1;
	}
#else
	pSencor->init();
	pSencor->stream_start(temp, csiBuffer, csiBuffer);
#endif

	mazePscalerSet(&PScalerParam);
}

static INT32S obj_frame_buffer_post(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(obj_frame_buffer_queue, (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S onj_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(obj_frame_buffer_queue, osWaitForever);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S obj_free_frame_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(obj_free_frame_buffer_queue, (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S obj_free_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(obj_free_frame_buffer_queue, 10);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S PPU_display_buffer_post(INT32U *frame_buf)
{
    INT32U event,temp;

    event = (INT32U)frame_buf;
    temp = osMessagePut(ppu_frame_buffer_queue, (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S pscaler_free_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(pscaler_free_buffer_queue , (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S pscaler_free_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(pscaler_free_buffer_queue , osWaitForever);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S pscaler_display_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(pscaler_display_buffer_queue , (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S pscaler_display_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(pscaler_display_buffer_queue , osWaitForever);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S obj_free_integral_y_free_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(obj_free_integral_y_buffer_queue , (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S obj_free_integral_y_free_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(obj_free_integral_y_buffer_queue , osWaitForever);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S obj_proc_integral_y_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(obj_proc_integral_y_buffer_queue , (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S obj_proc_integral_y_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(obj_proc_integral_y_buffer_queue , osWaitForever);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S ppu_draw_frame_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(ppu_draw_free_buffer_queue , (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S ppu_draw_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(ppu_draw_free_buffer_queue , osWaitForever);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S obj_state_post(INT32U state)
{
    INT32U event,temp;

    event = (INT32U)state;
    temp = osMessagePut(obj_state_queue, (uint32_t)&event, osWaitForever);

	return temp;
}

static INT32S obj_state_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(obj_state_queue, 1);
    frame = result.value.v;
    if((result.status != osEventMessage) || (frame!=OBJ_STATE_OK)) {
        frame = 0;
	}

	return frame;
}

static INT32S ppu_draw_src_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
	INT8U err;
	INT32S nRet;
	ObjFrame_t *pInput,Input;

	if(frame_buffer == 0 || in_buffer == 0) {
		RETURN(STATUS_FAIL);
	}

    pInput = (ObjFrame_t *)&Input;
	if(pInput == 0) {
		RETURN(STATUS_FAIL);
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

	// scale
	scalerStart(0, &pInput->ScaleIn, &pInput->ScaleOut);
	scalerEnd();

	nRet = STATUS_OK;
Return:
	return nRet;
}

static void csi_task_entry(void const *parm)
{
    osEvent result;
    INT32U i,display_buf,y_buffer,obj_data_buf,obj_update,ack_msg;
	INT32S frame,temp,obj_flag,obj_data_flag;
 #if NEW_OBJ_Y_MODE == 1
    INT32U avgY;
 #endif

    DBG_PRINT("csi_task_entry start \r\n");

    temp = (PPU_TEXT_SIZE_HPIXEL * PPU_TEXT_SIZE_VPIXEL * 2);
    y_buffer = (INT32U) gp_malloc_align(((temp*C_PPU_DRV_FRAME_NUM)+64),32);
    if(y_buffer == 0)
    {
        DBG_PRINT("y_buffer fail\r\n");
        while(1);
    }
    prcess_mem_set->obj_prcess_workmem = y_buffer;
    y_buffer = (INT32U)((y_buffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    for(i=0;i<C_PPU_DRV_FRAME_NUM;i++)
    {
        frame = (INT32U)(y_buffer+(i*temp));
        obj_free_frame_buffer_add((INT32U *)frame);
        DBG_PRINT("y_buffer%d = 0x%x\r\n",i,frame);
    }

    // ppu draw src memory
    temp = PPU_TEXT_SIZE_HPIXEL*PPU_TEXT_SIZE_VPIXEL*2;
    y_buffer = (INT32U) gp_malloc_align(((temp*C_PPU_DRV_FRAME_NUM)+64),32);
    if(y_buffer == 0)
    {
        DBG_PRINT("y_buffer fail\r\n");
        while(1);
    }
    prcess_mem_set->ppu_draw_src_workmem = y_buffer;
    y_buffer = (INT32U)((y_buffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    for(i=0;i<C_PPU_DRV_FRAME_NUM;i++)
    {
        frame = (INT32U)(y_buffer+(i*temp));
        ppu_draw_frame_buffer_add((INT32U *)frame);
        DBG_PRINT("ppu_draw_src_buf%d = 0x%x\r\n",i,frame);
    }

    mazeTest_Preview_PScaler();

#if NEW_OBJ_Y_MODE == 1
    temp = PPU_TEXT_SIZE_HPIXEL * PPU_TEXT_SIZE_VPIXEL * 2;
    y_buffer = (INT32U) gp_malloc_align(((temp*C_PPU_DRV_FRAME_NUM)+64),32);
    if(y_buffer == 0)
    {
        DBG_PRINT("integral_y_buffer fail\r\n");
        while(1);
    }
    prcess_mem_set->obj_y_workmem = y_buffer;
    y_buffer = (INT32U)((y_buffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    for(i=0;i<C_PPU_DRV_FRAME_NUM;i++)
    {
        frame = (INT32U)(y_buffer+(i*temp));
        obj_free_integral_y_free_buffer_add((INT32U *)frame);
        DBG_PRINT("integral_y_buffer%d = 0x%x\r\n",i,frame);
    }
#endif

    obj_data_buf = (INT32U)gp_malloc_align(((IMAGE_SIZE*(IMAGE_BIN_NUMBER+2))+64),32);
    if(obj_data_buf == 0)
    {
        DBG_PRINT("obj_data_buf fail\r\n");
        while(1);
    }
    prcess_mem_set->obj_data_workmem = obj_data_buf;
    obj_data_buf = (INT32U)((obj_data_buf + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    DBG_PRINT("obj_data_buf = 0x%x \r\n",obj_data_buf);
    obj_update = 0;
    obj_dispbin_load_data(obj_data_buf,obj_update);

    display_buf = 0;
    obj_flag = 0;
    osDelay(5);
#if _SENSOR_GC0308_CSI == 1
    YValue = GC0308_ReadY();
#else
    YValue = 0;
#endif
    obj_state_post(OBJ_STATE_OK);

    while(1)
    {
        frame = disp_frame_buffer_get();
        if(!frame) {
            continue;
        }

        //DBG_PRINT(".");
        //DBG_PRINT("csi_buffer = 0x%x\r\n", frame);
        switch(frame)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_CSI_TASK_EXIT]\r\n");
                csi_pscaler_stop = 1;
                while(csi_pscaler_stop)
                    osDelay(1);
                #if CSI_PSCALER_STOP_EN == 0
                    pSencor = drv_l2_sensor_get_ops(0);
                    pSencor->>stream_stop();
                    pSencor->uninit();
                #endif
                ack_msg = ACK_OK;
                osMessagePut(csi_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(csi_id);
                break;

            default:
                y_buffer = ppu_draw_frame_buffer_get();
                if(y_buffer)
                {
                    ppu_draw_src_frame(frame,y_buffer,PRCESS_SRC_WIDTH,PRCESS_SRC_HEIGHT,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL);
                    free_frame_buffer_add((INT32U *)frame);
                    frame = y_buffer;
                }

                temp = obj_state_get();
                if(temp == OBJ_STATE_OK)
                {
                    y_buffer = obj_free_frame_buffer_get();
                    if(y_buffer)
                    {
                        object_detect_set_y_frame(frame,y_buffer,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,RFMSDETECT_IMG_WIDTH,RFMSDETECT_IMG_HEIGHT,BITMAP_GRAY);
                        #if NEW_OBJ_Y_MODE == 1
                            if(ObjWorkMem_ptr)
                            {
                                temp = y_buffer;
                                y_buffer = obj_free_frame_buffer_get();
                                if(y_buffer)
                                {
                                    GPSmoothImage((unsigned char *)temp, (unsigned char *)y_buffer, RFMSDETECT_IMG_WIDTH, RFMSDETECT_IMG_HEIGHT);
                                    obj_free_frame_buffer_add((INT32U *)temp);
                                    #if GP_OBJ_HW_EN == 1
                                        temp = obj_free_integral_y_free_buffer_get();
                                        if(temp)
                                        {
                                            object_detect_set_integral_y_frame(y_buffer, temp, RFMSDETECT_IMG_WIDTH, RFMSDETECT_IMG_HEIGHT);
                                            obj_proc_integral_y_buffer_add((INT32U *)temp);
                                        }
                                    #endif
                                    if(temp)
                                    {
                                        #if _SENSOR_GC0308_CSI == 1
                                            //**** Auto AE ****
                                            avgY = objDetectRY((unsigned char *)y_buffer, RFMSDETECT_IMG_WIDTH, RFMSDETECT_IMG_HEIGHT);
                                            if(avgY < 120)
                                                YValue += 8;
                                            else if(avgY > 150)
                                                YValue -= 8;

                                            if(YValue >= 255)
                                                YValue = 255;

                                            if(YValue <= 0)
                                                YValue = 0;
                                            GC0308_SetY(YValue);
                                        #endif
                                        obj_frame_buffer_post((INT32U *)y_buffer);
                                    }
                                    else
                                        obj_free_frame_buffer_add((INT32U *)y_buffer);
                                }
                                else
                                    obj_free_frame_buffer_add((INT32U *)temp);
                            }
                            else
                                obj_free_frame_buffer_add((INT32U *)y_buffer);
                        #else
                            obj_frame_buffer_post((INT32U *)y_buffer);
                        #endif
                    }

                    drv_prcess_lock();
                    obj_flag = obj_result_set.ObjIdentifyFlag;
                    drv_prcess_unlock();
                    if(obj_flag)
                    {
                        drv_prcess_lock();
                        obj_flag = obj_result_set.MatchingCardNum;
                        drv_prcess_unlock();
                        switch(obj_flag)
                        {
                            case 10:
                            case 20:
                            case 30:
                            case 40:
                            case 50:
                            case 60:
                                obj_data_flag = (obj_flag / 10);
                                obj_data_flag = (obj_data_flag - 1);
                                obj_flag = 10;
                                break;

                            default:
                                obj_data_flag = (obj_flag / 10);
                                obj_flag = (obj_flag % 10);
                            break;
                        }
                        //DBG_PRINT("obj_update:%d \r\n",obj_update);
                        //DBG_PRINT("obj_data_flag:%d \r\n",obj_data_flag);
                        //DBG_PRINT("obj_flag:%d \r\n",obj_flag);
                        if((obj_update != obj_data_flag) && (obj_data_flag <= display_bin_max))
                        {
                            obj_dispbin_load_data(obj_data_buf, obj_data_flag);
                            obj_update = obj_data_flag;
                        }

                        fd_ppu_go(PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,(INT32U)(obj_data_buf+((obj_flag-1)*IMAGE_SIZE)));
                    }
                    else
                        fd_ppu_go(PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,frame);
                }
                else
                    fd_ppu_go(PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,frame);

                display_buf = ppu_frame_buffer_display_get();
                ppu_draw_frame_buffer_add((INT32U *)frame);
                if(display_buf > 0)
                    PPU_display_buffer_post((INT32U *)display_buf);
        }
    }
}

static void disp_task_entry(void const *parm)
{
    osEvent result;
    INT32U display_scaler_buf,display_buf,ret,ack_msg;

    DBG_PRINT("disp_task_entry start \r\n");
    disp_buffer_post = 0;
    pscaler_disp_buffer_post = 0;
    osDelay(5);
    while(1)
    {
        result = osMessageGet(ppu_frame_buffer_queue, osWaitForever);
        display_buf = result.value.v;
        if((result.status != osEventMessage) || !display_buf) {
            continue;
        }
        //DBG_PRINT("display_buf=0x%x\r\n",display_buf);
        //DBG_PRINT("D");

        switch(display_buf)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_DISP_TASK_EXIT]\r\n");
                drv_l2_display_stop(DISPLAY_DEVICE);
                drv_l2_display_uninit();
                ack_msg = ACK_OK;
                osMessagePut(disp_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(disp_id);
                break;

            default:
    #if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
                display_scaler_buf = pscaler_display_frame_buffer_get();
                if(display_scaler_buf > 0)
                    drv_l2_display_update(DISPLAY_DEVICE, display_scaler_buf);

                if(pscaler_disp_buffer_post)
                {
                    if(pscaler_disp_buffer_post > 0)
                        pscaler_free_buffer_add((INT32U *)pscaler_disp_buffer_post);
                    if(display_scaler_buf)
                        pscaler_disp_buffer_post = display_scaler_buf;
                }
                else
                {
                    if(display_scaler_buf)
                        pscaler_disp_buffer_post = display_scaler_buf;
                }
    #else
                if(display_buf > 0)
                    drv_l2_display_update(DISPLAY_DEVICE, display_buf);
    #endif
                if(disp_buffer_post)
                {
                    if(disp_buffer_post > 0)
                        gplib_ppu_frame_buffer_add(ppu_register_set, disp_buffer_post);
                    disp_buffer_post = display_buf;
                }
                else
                {
                    disp_buffer_post = display_buf;
                }

                drv_prcess_lock();
                ret = obj_result_set.ObjIdentifyFlag;
                drv_prcess_unlock();
                if(ret)
                {
                    if(obj_display_cnt++ > OBJ_DELAY_TIME)
                    {
                        drv_prcess_lock();
                        obj_result_set.ObjIdentifyFlag = 0;
                        drv_prcess_unlock();
                        obj_display_cnt = 0;
                    }
                }
        }
    }
}

static void obj_task_entry(void const *parm)
{
    globalData *gData_ptr;
    INT32U i,y_buffer,temp,ack_msg;
 #if NEW_OBJ_Y_MODE == 0
    INT32U Smooth_buffer,avgY;
 #endif
    gpImage *pgray,gray_ptr;
    osEvent result;
    RfmsIdentify_t *rfms;

    DBG_PRINT("obj_demo start \r\n");

    ObjWorkMem_ptr = (ObjDetect_t *)obj_recognize_alloc();
    if(!ObjWorkMem_ptr)
    {
        DBG_PRINT("ObjWorkMem_ptr fail \r\n");
        while(1);
    }

#if NEW_OBJ_Y_MODE == 0
    temp = (ObjWorkMem_ptr->image.width * ObjWorkMem_ptr->image.height * 4);
    Smooth_buffer = (INT32U) gp_malloc_align(((temp*2)+64),32);
    if(!Smooth_buffer)
    {
        DBG_PRINT("Smooth_buffer fail \r\n");
        while(1);
    }
    prcess_mem_set->obj_smooth_workmem = Smooth_buffer;
    Smooth_buffer = (INT32U)((Smooth_buffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    DBG_PRINT("Smooth_buffer = 0x%x\r\n",Smooth_buffer);
    y_buffer = (Smooth_buffer + temp);
    DBG_PRINT("integral_y_buffer = 0x%x\r\n",y_buffer);
#endif

    obj_gpio_init();

    // for new int mode use
    drv_l1_scaler_new_int_callback_set(fd_scaler_step_get);
    fd_org_buf = 0;
#if NEW_OBJ_Y_MODE == 0
    avgY = 0;
    osDelay(5);
#endif
    obj_result_set.ObjIdentifyFlag = 0;

    while(1)
    {
        result = osMessageGet(obj_frame_buffer_queue, osWaitForever);
        fd_org_buf = result.value.v;
        if(result.status != osEventMessage || !fd_org_buf) {
            continue;
        }

        switch(fd_org_buf)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_OBJ_TASK_EXIT]\r\n");
                ack_msg = ACK_OK;
                osMessagePut(obj_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(obj_id);
                break;

            default:
                drv_prcess_lock();
                temp = obj_result_set.ObjIdentifyFlag;
                drv_prcess_unlock();
                if(temp == 0)
                {
                    ObjWorkMem_ptr->image.ptr = (unsigned char *)fd_org_buf;
        #if NEW_OBJ_Y_MODE == 1
                    y_buffer = obj_proc_integral_y_frame_buffer_get();
                    if(y_buffer)
                    {
                        rfms = (RfmsIdentify_t *)ObjWorkMem_ptr->rfms_detect_WorkMem;
                        gData_ptr = (globalData *)rfms->rfms_WorkMem;
                        gData_ptr->sum.i = (int *)y_buffer;
                    }
        #else
                    GPSmoothImage((unsigned char*)(ObjWorkMem_ptr->image.ptr), (unsigned char*)Smooth_buffer, ObjWorkMem_ptr->image.width, ObjWorkMem_ptr->image.height);
                    ObjWorkMem_ptr->image.ptr = (unsigned char *)Smooth_buffer;
                    #if GP_OBJ_HW_EN == 1
                        object_detect_set_integral_y_frame(Smooth_buffer,y_buffer,ObjWorkMem_ptr->image.width,ObjWorkMem_ptr->image.height);
                        rfms = (RfmsIdentify_t *)ObjWorkMem_ptr->rfms_detect_WorkMem;
                        gData_ptr = (globalData *)rfms->rfms_WorkMem;
                        gData_ptr->sum.i = (int *)y_buffer;
                    #endif
                    #if _SENSOR_GC0308_CSI == 1
                        //**** Auto AE ****
                        avgY = objDetectRY((unsigned char *)Smooth_buffer, ObjWorkMem_ptr->image.width, ObjWorkMem_ptr->image.height);
                        if(avgY < 120)
                            YValue += 8;
                        else if(avgY > 150)
                            YValue -= 8;

                        if(YValue >= 255)
                            YValue = 255;

                        if(YValue <= 0)
                            YValue = 0;
                        GC0308_SetY(YValue);
                    #endif
        #endif

        #if NEW_OBJ_Y_MODE == 1
                    if(ObjWorkMem_ptr->image.ptr && y_buffer)
                        obj_recognize_proc((ObjDetect_t *)ObjWorkMem_ptr);
        #else
                    obj_recognize_proc((ObjDetect_t *)ObjWorkMem_ptr);
        #endif
                    drv_prcess_lock();
                    obj_recognize_get_result((ObjDetect_t *)ObjWorkMem_ptr,(void *)&obj_result_set);
                    drv_prcess_unlock();
                }

        #if NEW_OBJ_Y_MODE == 1
                if(y_buffer)
                    obj_free_integral_y_free_buffer_add((INT32U *)y_buffer);
        #endif
                obj_free_frame_buffer_add((INT32U *)fd_org_buf);
                obj_state_post(OBJ_STATE_OK);
        }
    }
}

void GPM4_OBJ_Demo(void)
{
    INT32S ret;
    osThreadDef_t csi_task = {"csi_task", csi_task_entry, osPriorityNormal, 1, 20000};
    osThreadDef_t disp_task = {"disp_task", disp_task_entry, osPriorityNormal, 1, 20000};
    osThreadDef_t obj_task = {"fd_task", obj_task_entry, osPriorityNormal, 1, 20000};
    osSemaphoreDef_t disp_sem = { 0 };
	osMessageQDef_t disp_q = {(DISP_QUEUE_MAX+1), sizeof(INT32U), 0};

    while(1)
    {
        ret = _devicemount(FS_SD2);
        if(ret)
        {
            DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD2);
            #if 0
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

    /* initial prcess parameter set structure */
    prcess_mem_set = (prcess_mem_t *)&prcess_mem_structure;
    gp_memset((INT8S *)prcess_mem_set, 0, sizeof(prcess_mem_t));
    csi_pscaler_stop = 0;

	if(sem_prcess_engine == NULL)
	{
		sem_prcess_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_prcess_engine)
  		{
            DBG_PRINT("sem_prcess_engine error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("sem_prcess_engine = 0x%x\r\n",sem_prcess_engine);
	}

	if(sem_scaler_engine == NULL)
	{
		sem_scaler_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_scaler_engine)
  		{
            DBG_PRINT("sem_scaler_engine error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("sem_scaler_engine = 0x%x\r\n",sem_scaler_engine);
	}

	if(free_frame_buffer_queue == NULL)
	{
        free_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!free_frame_buffer_queue)
  		{
            DBG_PRINT("free_frame_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("free_frame_buffer_queue = 0x%x\r\n",free_frame_buffer_queue);
	}

	if(display_frame_buffer_queue  == NULL)
	{
        display_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!display_frame_buffer_queue)
  		{
            DBG_PRINT("display_frame_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("display_frame_buffer_queue = 0x%x\r\n",display_frame_buffer_queue);
	}

  	if(obj_frame_buffer_queue == NULL)
	{
        obj_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!obj_frame_buffer_queue)
  		{
            DBG_PRINT("obj_frame_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("obj_frame_buffer_queue = 0x%x\r\n",obj_frame_buffer_queue);
	}

  	if(obj_free_frame_buffer_queue == NULL)
	{
        obj_free_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!obj_free_frame_buffer_queue)
  		{
            DBG_PRINT("obj_free_frame_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("obj_free_frame_buffer_queue = 0x%x\r\n",obj_free_frame_buffer_queue);
	}

  	if(ppu_frame_buffer_queue == NULL)
	{
        ppu_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
 		if(!ppu_frame_buffer_queue)
  		{
            DBG_PRINT("ppu_frame_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("ppu_frame_buffer_queue = 0x%x\r\n",ppu_frame_buffer_queue);
	}

  	if(pscaler_free_buffer_queue == NULL)
	{
        pscaler_free_buffer_queue = osMessageCreate(&disp_q, NULL);
  		if(!pscaler_free_buffer_queue)
  		{
            DBG_PRINT("pscaler_free_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("pscaler_free_buffer_queue = 0x%x\r\n",pscaler_free_buffer_queue);
	}

  	if(pscaler_display_buffer_queue == NULL)
	{
        pscaler_display_buffer_queue = osMessageCreate(&disp_q, NULL);
  		if(!pscaler_display_buffer_queue)
  		{
            DBG_PRINT("pscaler_display_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("pscaler_display_buffer_queue = 0x%x\r\n",pscaler_display_buffer_queue);
	}

	if(ppu_draw_free_buffer_queue == NULL)
	{
        ppu_draw_free_buffer_queue = osMessageCreate(&disp_q, NULL);
  		if(!ppu_draw_free_buffer_queue)
  		{
            DBG_PRINT("ppu_draw_free_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("ppu_draw_free_buffer_queue = 0x%x\r\n",ppu_draw_free_buffer_queue);
	}

  	if(obj_free_integral_y_buffer_queue == NULL)
	{
        obj_free_integral_y_buffer_queue = osMessageCreate(&disp_q, NULL);
  		if(!obj_free_integral_y_buffer_queue)
  		{
            DBG_PRINT("obj_free_integral_y_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("obj_free_integral_y_buffer_queue = 0x%x\r\n",obj_free_integral_y_buffer_queue);
	}

  	if(obj_proc_integral_y_buffer_queue == NULL)
	{
        obj_proc_integral_y_buffer_queue = osMessageCreate(&disp_q, NULL);
  		if(!obj_proc_integral_y_buffer_queue)
  		{
            DBG_PRINT("obj_proc_integral_y_buffer_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("obj_proc_integral_y_buffer_queue = 0x%x\r\n",obj_proc_integral_y_buffer_queue);
	}

  	if(obj_state_queue == NULL)
	{
        obj_state_queue = osMessageCreate(&disp_q, NULL);
  		if(!obj_state_queue)
  		{
            DBG_PRINT("obj_state_queue error\r\n");
            while(1);
  		}
		else
            DBG_PRINT("obj_state_queue = 0x%x\r\n",obj_state_queue);
	}

  	if(obj_task_ack_m == NULL)
	{
        obj_task_ack_m = osMessageCreate(&disp_q, NULL);
  		if(!obj_task_ack_m)
		{
            DBG_PRINT("obj_task_ack_m error\r\n");
            while(1);
		}
		else
            DBG_PRINT("obj_task_ack_m = 0x%x\r\n",obj_task_ack_m);
	}

  	if(csi_task_ack_m == NULL)
	{
        csi_task_ack_m = osMessageCreate(&disp_q, NULL);
  		if(!csi_task_ack_m)
		{
            DBG_PRINT("csi_task_ack_m error\r\n");
            while(1);
		}
		else
            DBG_PRINT("csi_task_ack_m = 0x%x\r\n",csi_task_ack_m);
	}

    if(disp_task_ack_m == NULL)
	{
        disp_task_ack_m = osMessageCreate(&disp_q, NULL);
  		if(!disp_task_ack_m)
		{
            DBG_PRINT("disp_task_ack_m error\r\n");
            while(1);
		}
		else
            DBG_PRINT("disp_task_ack_m = 0x%x\r\n",disp_task_ack_m);
	}

    fd_ppu_init();
    osDelay(5);

    obj_id = osThreadCreate(&obj_task, (void *)NULL);
    if(obj_id == 0) {
        DBG_PRINT("obj_task error\r\n");
        while(1);
    }
    else
        osDelay(30);

    disp_id = osThreadCreate(&disp_task, (void *)NULL);
    if(disp_id == 0) {
        DBG_PRINT("disp_task error\r\n");
        while(1);
    }
    else
        osDelay(30);

    csi_id = osThreadCreate(&csi_task, (void *)NULL);
    if(csi_id == 0) {
        DBG_PRINT("csi_task error\r\n");
        while(1);
    }
    else
        osDelay(30);

    adc_key_scan_init();
    osDelay(500);
	DBG_PRINT("\r\n***************************************************\r\n");
	DBG_PRINT("                 This is OBJ DEMO                    **\r\n");
	DBG_PRINT("KEY_1 OBJ DEMO EXIT                                  **\r\n");
	DBG_PRINT("***************************************************\r\n");
    while(1)
    {
        adc_key_scan();
        if(ADKEY_IO1)
		{
            prcess_task_exit();
            DBG_PRINT("Demo Exit\r\n");
            break;
		}
    }
    adc_key_scan_uninit();
    _deviceunmount(FS_SD2);
}
