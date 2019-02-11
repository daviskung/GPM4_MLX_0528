#include "string.h"
#include "stdio.h"
#include "project.h"
#include "application.h"
#include <stdlib.h>
#include "gplib.h"
#include "define.h"
#include "drv_l1_dma.h"
#include "drv_l1_fft.h"
#include "FaceDetectAP.h"
#include "avi_encoder_scaler_jpeg.h"
#include "avi_encoder_app.h"

#define DRONE_OBJECT_RECOGNIZE_DEMO         1
#define COLOR_ID_DEMO                       0

#if (DRONE_OBJECT_RECOGNIZE_DEMO == 1)
#include "RFMS.h"
#include "QR_Functions.h"
#endif

#if (COLOR_ID_DEMO == 1)
#include "ColorTrackerAP.h"
#endif

#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define FRAME_BUF_ALIGN16                   0xF
#define FRAME_BUF_ALIGN8                    0x7
#define FRAME_BUF_ALIGN4                    0x3
#define OBJDETECT_MAX_RESULT		        128
#define RETURN(x)	                        {nRet = x; goto Return;}
#define FD_STATE_OK                         0x80
#define FRAME_STATE_OK                      0x1F
// object recognize
#define RFMSDETECT_IMG_WIDTH	            256
#define RFMSDETECT_IMG_HEIGHT	            192
#define OBJ_DELAY_TIME                      5
#define C_AE_VALUE		 		            0x48
#define ABS(x)		                        ((x) >= 0 ? (x) : -(x))
unsigned int delayShowCnt;

#define PRC_DMA_WIDTH			640
#define PRC_DMA_HEIGHT			480

typedef struct
{
	INT32U center_x;
	INT32U center_y;
	INT32U recong_w;
	INT32U recong_h;
	INT32U recong_id;
} objRecongResult;

static osThreadId prcess_id;
static xQueueHandle prcess_frame_buffer_queue = NULL;
static xQueueHandle prcess_state_queue = NULL;
static xQueueHandle dma_frame_buffer_queue = NULL;
static xSemaphoreHandle sem_fd_engine = NULL;
xQueueHandle prcess_draw_queue = NULL;
static INT32U retry=0;

objRecongResult obj_recong_draw;

#define PRCESS_STATE_OK                     0x80
#define C_PPU_DRV_FRAME_NUM		            5
#define C_DEMO_DRV_Q_NUM		            (C_PPU_DRV_FRAME_NUM+1)
#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F

extern const unsigned int bin_objdatabase[];

unsigned char resultstream[ 1024 ];


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

//static PPU_REGISTER_SETS ppu_register_structure;
//static PPU_REGISTER_SETS *ppu_register_set;
static INT32U PPU_FRAME_BUFFER_BASE,fd_org_buf,text1_narray,disp_buffer_post,obj_display_cnt,YValue,compare_out_buffer = 0;
static INT32U integral_image_en,scaler_int_w_step,pscaler_buf,pscaler_disp_buffer_post,pscaler_init_set = 0;
static INT32U disp_h_size,disp_v_size,obj_image_number,display_bin_max;
static ObjDetResult_t obj_result_set = {0};
static ObjDetect_t *ObjWorkMem_ptr = NULL;

#if COLOR_ID_DEMO == 1
static int color_id = 0;
#endif




extern INT32U user_malloc_function(INT32U size);
extern INT32S user_fftobj_hw_function(INT32U rfms_min_base, INT32U obj_num, INT32U objectDes_ptr, INT32U imageDes_ptr, INT32U *hw_min_error, INT32U *hw_min_error_id);

/*
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

    return 0;
}
*/
static void drv_fd_lock(void)
{
    if(sem_fd_engine){
        osSemaphoreWait(sem_fd_engine, osWaitForever);
    }
}

static void drv_fd_unlock(void)
{
    if(sem_fd_engine){
        osSemaphoreRelease(sem_fd_engine);
    }
}

static INT32S prcess_dma_start(INT32U src_buf, INT32U tar_buf)
{
    INT8S done1;
    DMA_STRUCT prcess_dma;

    prcess_dma.s_addr = src_buf;
    prcess_dma.t_addr = tar_buf;
    prcess_dma.count = (PRC_DMA_WIDTH * PRC_DMA_HEIGHT * 2)/4;
    prcess_dma.width = DMA_DATA_WIDTH_4BYTE;
    prcess_dma.timeout = 0;
    prcess_dma.notify = &done1;

    return drv_l1_dma_transfer_wait_ready((DMA_STRUCT *)&prcess_dma);
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

static INT32S dma_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(dma_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(dma_frame_buffer_queue, (uint32_t)&event, 5);
    }

	return temp;
}

static void prcess_dma_init(void)
{
    INT32U i, prcess_dma_size, prcess_dma_buf, buffer_ptr;

    //Frame buffer malloc
    prcess_dma_size = (PRC_DMA_WIDTH * PRC_DMA_HEIGHT * 2);
    prcess_dma_buf = (INT32U) gp_malloc_align(((prcess_dma_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(prcess_dma_buf == 0)
    {
        DBG_PRINT("prcess_dma_buf fail\r\n");
        while(1);
    }
    prcess_dma_buf = (INT32U)((prcess_dma_buf + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(prcess_dma_buf + (i*prcess_dma_size));
            dma_frame_buffer_add((INT32U *)buffer_ptr, 1);
            DBG_PRINT("DMABuffer:0x%X \r\n",buffer_ptr);
    }
/*
    if(disp_init_buffer == 0)
        disp_init_buffer = prcess_dma_buf;
        */
}

static INT32S dma_frame_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(dma_frame_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(dma_frame_buffer_queue, 5);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
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
/*
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
*/
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
	rfms->obj_data_buf = (INT8U*)(&bin_objdatabase[0]);

	/*(CHAR *)obj_recognize_load_data(&rfms->cardsN);
	if(rfms->obj_data_buf == 0) {
		RETURN(STATUS_FAIL);
	}
	*/

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

	rfms->extractionThre = 100;
	rfms->matchingThre = 25000;
	rfms->matchingThreRangeV = 4000;
	rfms->minExtractionThre = 10;
	rfms->incExtractionThre = 50;
	rfms->decExtractionThre = 50;
	rfms->startMatchingPointN = 70;

    delayShowCnt = 0;

	// rfms init

    RFMS_init_HW(rfms->image.width,
			rfms->image.height,
			rfms->rfms_WorkMem,
			odWorkMem->image.width,
			odWorkMem->image.height,
			rfms->obj_data_buf);
    drv_l1_obj_init();
    drv_l1_obj_user_malloc_set(user_malloc_function);
    rfms_fftobj_hw_set(user_fftobj_hw_function);

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

#if (DRONE_OBJECT_RECOGNIZE_DEMO == 1)
static void obj_recognize_proc(ObjDetect_t *odWorkMem)
{
	INT32S objectTotal;
	INT32U t1, t2;
	globalData *gData_ptr;



	RfmsIdentify_t *rfms = (RfmsIdentify_t *)odWorkMem->rfms_detect_WorkMem;
	gData_ptr = (globalData *)rfms->rfms_WorkMem;

//t1 = xTaskGetTickCount();

	//image can not be mirror or flip
	objectTotal = RFMS_extractPoint(rfms->rfms_WorkMem, &odWorkMem->image);
//t2 = xTaskGetTickCount();
//DBG_PRINT("extraction: time=%d, ", (t2 - t1));



//t1 = xTaskGetTickCount();
    if(objectTotal < 200 && objectTotal > rfms->startMatchingPointN)
        rfms->matchingCardNum = RFMS_findPairs(rfms->rfms_WorkMem, objectTotal);
    else
        rfms->matchingCardNum = 0;
//t2 = xTaskGetTickCount();
//DBG_PRINT("matching: time=%d \r\n", (t2 - t1));

	DBG_PRINT("objectTotal = %d, extractionThre = %d, matchingCardNum = %d \r\n", objectTotal, gData_ptr->extractionThre ,rfms->matchingCardNum);
	DBG_PRINT("maxCnt = %d, thre = %d \r\n",gData_ptr->maxCnt, gData_ptr->matchingThre);

}

static void obj_recognize_get_result(void *od_work_mem, void *objresult)
{
	ObjDetect_t *odWorkMem = (ObjDetect_t *)od_work_mem;
	ObjDetResult_t *result = (ObjDetResult_t *)objresult;
	RfmsIdentify_t *rfms = (RfmsIdentify_t *)odWorkMem->rfms_detect_WorkMem;

	if(rfms->matchingCardNum > 0)
	 {
		result->ObjIdentifyFlag = 1;
		result->MatchingCardNum = rfms->matchingCardNum;
		//DBG_PRINT("CardNumber = %d\r\n", result->MatchingCardNum);
	} else {
		result->ObjIdentifyFlag = 0;
		result->MatchingCardNum = 0;
	}
}
#endif

static INT32S scalerStart(gpImage *src, gpImage *dst, gpRect *clip)
{
    INT32S ret;
	gpImage temp;


	if(integral_image_en)
        ret = drv_l2_FD_scaler_clip(0, 1, src, dst, clip);
    else
        ret = drv_l2_scaler_clip(0, 1, src, dst, clip);

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
	gpRect clip;

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
    integral_image_en = 1;
    clip.x = 0;
	clip.y = 0;
	clip.width = pInput->ScaleIn.width;
	clip.height = pInput->ScaleIn.height;

drv_fd_lock();
	scalerStart(&pInput->ScaleIn, &pInput->ScaleOut, &clip);
	scalerEnd();
	nRet = STATUS_OK;
Return:
    drv_fd_unlock();

    integral_image_en = 0;

	return nRet;
}
static INT32U fd_scaler_step_get(void)
{
    return (scaler_int_w_step - 1);
}

#if (DRONE_OBJECT_RECOGNIZE_DEMO == 1)
static void prcess_task_entry(void const *parm)
{
    INT32U t1, t2;
    INT32U i,prcess_buf,PscalerBuffer,PscalerBufferSize;
    osEvent result;
    globalData *gData_ptr;
    INT32U y_buffer,temp;
    INT32U Smooth_buffer,avgY;
    gpImage *pgray,gray_ptr;
    RfmsIdentify_t *rfms;
    INT32U event;



    //QRCode marker detector
    long QR_ret;
	QR_RESULT QR_result;
	unsigned char *QRCode_Working_Memory;
	unsigned char image_type;
	unsigned char res;
	QRCode_MEMORY_BLOCK* QRCode_dataPtr;


	gpImage src;
	gpImage dst;
	gpRect clip;
	CPoint p1, p2;



	res = 0;
	QRCode_Working_Memory = ( unsigned char* ) gp_malloc_align(sizeof( QRCode_MEMORY_BLOCK ),32);   //Dynamic allocate memory for working memory
	image_type = 0;
	image_type |= IMAGETYPE_Y0Y1;	//for sequential Y image
	//memset( resultstream, 0, 1024 );
	mem_set_zero_int32(&resultstream[0], 1024>>2);
	QR_ret = QR_Init( QRCode_Working_Memory, 640, 480, image_type, resultstream );

	QRCode_dataPtr = ( QRCode_MEMORY_BLOCK*)QRCode_Working_Memory;



    src.width = 640;
	src.widthStep = 640*2;
	src.height = 480;
	src.ch = 2;
	src.format = IMG_FMT_UYVY;//IMG_FMT_YUYV;IMG_FMT_RGB;//


    dst.width = 640;
    dst.widthStep = 640;
    dst.height = 480;
    dst.ch = 1;
	dst.format = IMG_FMT_GRAY;



	clip.x = 0;
	clip.y = 0;
	clip.width = 640;
	clip.height = 480;



    DBG_PRINT("obj_demo start \r\n");



    ObjWorkMem_ptr = (ObjDetect_t *)obj_recognize_alloc();
    if(!ObjWorkMem_ptr)
    {
        DBG_PRINT("ObjWorkMem_ptr fail \r\n");
        while(1);
    }

     ObjWorkMem_ptr->image.ptr = (unsigned char*) gp_malloc_align((RFMSDETECT_IMG_WIDTH*RFMSDETECT_IMG_HEIGHT),32);

    temp = (ObjWorkMem_ptr->image.width * ObjWorkMem_ptr->image.height * 4);
    Smooth_buffer = (INT32U) gp_malloc_align(((temp*2)+64),32);
    if(!Smooth_buffer)
    {
        DBG_PRINT("Smooth_buffer fail \r\n");
        while(1);
    }

    Smooth_buffer = (INT32U)((Smooth_buffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    DBG_PRINT("Smooth_buffer = 0x%x\r\n",Smooth_buffer);
    y_buffer = (Smooth_buffer + temp);
    DBG_PRINT("integral_y_buffer = 0x%x\r\n",y_buffer);



    // for new int mode use
    drv_l1_scaler_new_int_callback_set(fd_scaler_step_get);
    fd_org_buf = 0;
    integral_image_en = 0;
    avgY = 0;
    osDelay(5);
    obj_result_set.ObjIdentifyFlag = 0;


    prcess_state_post(PRCESS_STATE_OK);
    while(1)
    {
        result = osMessageGet(prcess_frame_buffer_queue, osWaitForever);
        prcess_buf = result.value.v;
        if((result.status != osEventMessage) || !prcess_buf) {
            continue;
        }
        //DBG_PRINT("prcess_buf = 0x%x\r\n", prcess_buf);

        // 1. convert YUYV to Y for QRCode
        src.width = 640;
        src.widthStep = 640*2;
        src.height = 480;
        src.ch = 2;
        src.format = IMG_FMT_UYVY;//IMG_FMT_YUYV;IMG_FMT_RGB;//


        dst.width = 640;
        dst.widthStep = 640;
        dst.height = 480;
        dst.ch = 1;
        dst.format = IMG_FMT_GRAY;



        clip.x = 0;
        clip.y = 0;
        clip.width = 640;
        clip.height = 480;

        src.ptr = (unsigned char*)prcess_buf;
        dst.ptr = &QRCode_dataPtr->bitmap[0];

        drv_fd_lock();
        scalerStart(&src, &dst, &clip);
        scalerEnd();
        drv_fd_unlock();

//t1 = xTaskGetTickCount();

        QR_result = qr_finderpattern( ( QRCode_MEMORY_BLOCK*)QRCode_Working_Memory, dst.ptr );//find center coordinate of finder-pattern

//t2 = xTaskGetTickCount();

//DBG_PRINT("QR marker detect time= %d \r\n", (t2 - t1));

        if( QR_result.ret == 0 )
        {
            //DBG_PRINT(" QRCode detected!!\r\n");

                p1.x = QR_result.QR_center_X - (QR_result.QR_recong_ROI_W>>1);
				p1.y = QR_result.QR_center_Y - (QR_result.QR_recong_ROI_H>>1);
				p2.x = QR_result.QR_center_X + (QR_result.QR_recong_ROI_W>>1);
				p2.y = QR_result.QR_center_Y + (QR_result.QR_recong_ROI_H>>1);

        }
        else
        {
          //DBG_PRINT(" QR_result.ret = %d\r\n", QR_result.ret);
        }


        //2. use QRCode to crop and scale image(256*192) to RFMS
        if( QR_result.ret == 0 )
        {


            src.width = 640;
            src.widthStep = 640*2;
            src.height = 480;
            src.ch = 2;
            src.format = IMG_FMT_UYVY;//IMG_FMT_YUYV;IMG_FMT_RGB;//


            dst.width = RFMSDETECT_IMG_WIDTH;
            dst.widthStep = RFMSDETECT_IMG_WIDTH;
            dst.height = RFMSDETECT_IMG_HEIGHT;
            dst.ch = 1;
            dst.format = IMG_FMT_GRAY;


            clip.x =  p1.x;
            if(clip.x & 1 == 1)
                clip.x+= 1;

            clip.y =  p1.y;
            if(clip.y & 1 == 1)
                clip.y+= 1;

            clip.width = QR_result.QR_recong_ROI_W;
            clip.height = QR_result.QR_recong_ROI_H;

            src.ptr = (unsigned char*)prcess_buf;//&QRCode_dataPtr->bitmap[0];
            dst.ptr = ObjWorkMem_ptr->image.ptr;


            //DBG_PRINT(" clip = %d, %d, %d, %d \r\n", clip.x, clip.y, clip.width, clip.height);


            drv_fd_lock();
           scalerStart(&src, &dst, &clip);
            scalerEnd();
            drv_fd_unlock();


        //drv_fd_lock();
        temp = obj_result_set.ObjIdentifyFlag;
        //drv_fd_unlock();


            GPSmoothImage((unsigned char*)(ObjWorkMem_ptr->image.ptr), (unsigned char*)Smooth_buffer, ObjWorkMem_ptr->image.width, ObjWorkMem_ptr->image.height);
            ObjWorkMem_ptr->image.ptr = (unsigned char *)Smooth_buffer;

            object_detect_set_integral_y_frame((INT32U)(ObjWorkMem_ptr->image.ptr), y_buffer,ObjWorkMem_ptr->image.width, ObjWorkMem_ptr->image.height);
            rfms = (RfmsIdentify_t *)ObjWorkMem_ptr->rfms_detect_WorkMem;
            gData_ptr = (globalData *)rfms->rfms_WorkMem;
            gData_ptr->sum.i = (int *)y_buffer;

            obj_recognize_proc((ObjDetect_t *)ObjWorkMem_ptr);
            obj_recognize_get_result((ObjDetect_t *)ObjWorkMem_ptr,(void *)&obj_result_set);

           // DBG_PRINT("QR_result = (%d, %d)\r\n", QR_result.QR_center_X, QR_result.QR_center_Y);



            delayShowCnt = 6;


        }
        else if( delayShowCnt == 0 )
        {
           obj_result_set.MatchingCardNum = 0;
           QR_result.QR_center_X = 0;
           QR_result.QR_center_Y = 0;
           QR_result.QR_recong_ROI_W = 0;
           QR_result.QR_recong_ROI_H = 0;
        }

            if( delayShowCnt > 0)
                delayShowCnt--;



        drv_fd_lock();
           obj_recong_draw.recong_id = obj_result_set.MatchingCardNum;
           obj_recong_draw.center_x = QR_result.QR_center_X;
           obj_recong_draw.center_y = QR_result.QR_center_Y;
           obj_recong_draw.recong_w = QR_result.QR_recong_ROI_W;
           obj_recong_draw.recong_h = QR_result.QR_recong_ROI_H;
        drv_fd_unlock();


#if 1 //enable wifi stream, need error handle
            event = (INT32U)0x80;//obj_result_set.MatchingCardNum;
            temp = osMessagePut(prcess_draw_queue, (uint32_t)&event, osWaitForever);

            //DBG_PRINT("q");
#endif


//t2 = xTaskGetTickCount();

//DBG_PRINT("total time= %d \r\n", (t2 - t1));


        //gplib_ppu_frame_buffer_add(ppu_register_set, prcess_buf);
        dma_frame_buffer_add((INT32U *)prcess_buf, 1);
        prcess_state_post(PRCESS_STATE_OK);
    }
}
#endif

#if (COLOR_ID_DEMO == 1)
static void prcess_task_entry(void const *parm)
{
    INT32U t1, t2, temp;
    INT32U i,prcess_buf,PscalerBuffer,PscalerBufferSize;
    osEvent result;
    ColorTracker_Input ctInput;
    gpImage src;
	gpImage dst;
	gpRect clip;
	INT32U event;
	gpRect posResult;
	const int COLOR_ID_WIDTH = ColorID_GetWidth();
	const int COLOR_ID_HEIGHT = ColorID_GetHeight();

	integral_image_en = 0;

    // Color ID
    ctInput.WorkMem = gp_malloc_align(ColorID_MemCalc(), 64);
    ctInput.y_array = (unsigned char*) gp_malloc_align(COLOR_ID_WIDTH*COLOR_ID_HEIGHT,32);
    ctInput.u_array = (unsigned char*) gp_malloc_align(COLOR_ID_WIDTH*COLOR_ID_HEIGHT,32);
    ctInput.v_array = (unsigned char*) gp_malloc_align(COLOR_ID_WIDTH*COLOR_ID_HEIGHT,32);

    src.width = 640;
    src.widthStep = 640*2;
    src.height = 480;
    src.ch = 2;
    src.format = IMG_FMT_UYVY;//IMG_FMT_YUYV;IMG_FMT_RGB;//

    //dst.width = 44;
    //dst.widthStep = 44*2;
    dst.width = COLOR_ID_WIDTH;
    dst.widthStep = COLOR_ID_WIDTH << 1;
    dst.height = COLOR_ID_HEIGHT;
    dst.ch = 2;
    dst.format = IMG_FMT_YUV444;
#if 1
    clip.width = COLOR_ID_WIDTH << 3;
    clip.height = COLOR_ID_HEIGHT << 3;
    clip.x = (640 - clip.width) >> 1;
    clip.y = (480 - clip.height) >> 1;

#else

    clip.x = 160;
    clip.y = 120;
    clip.width = 320;
    clip.height = 240;
#endif


    prcess_state_post(PRCESS_STATE_OK);
    while(1)
    {
        result = osMessageGet(prcess_frame_buffer_queue, osWaitForever);
        prcess_buf = result.value.v;
        if((result.status != osEventMessage) || !prcess_buf) {
            continue;
        }
        //DBG_PRINT("prcess_buf = 0x%x\r\n", prcess_buf);

        src.ptr = (unsigned char*)prcess_buf;
        dst.ptr = ctInput.y_array;
        dst.ptr_u = ctInput.u_array;
        dst.ptr_v = ctInput.v_array;

        drv_fd_lock();
        scalerStart(&src, &dst, &clip);
        scalerEnd();
        drv_fd_unlock();

t1 = xTaskGetTickCount();

        color_id = ColorID(&ctInput, &posResult);
t2 = xTaskGetTickCount();

        //DBG_PRINT("time = %d\r\n", (t2 - t1));

#if 1
        drv_fd_lock();
        obj_recong_draw.recong_id = color_id;

        if (color_id == 0)
        {
            obj_recong_draw.center_x = 0;
            obj_recong_draw.center_y = 0;
            obj_recong_draw.recong_w = 0;
            obj_recong_draw.recong_h = 0;
        }
        else
        {
            obj_recong_draw.center_x = posResult.x;
            obj_recong_draw.center_y = posResult.y;
            obj_recong_draw.recong_w = 40;
            obj_recong_draw.recong_h = 40;
        }

        drv_fd_unlock();

#else
        switch(color_id)
        {
        case 1:
            DBG_PRINT("Green soldier, time = %d\r\n", (t2 - t1));
            break;
        case 2:
            DBG_PRINT("Red soldier, time = %d\r\n", (t2 - t1));
            break;
        case 3:
            DBG_PRINT("Blue soldier, time = %d\r\n", (t2 - t1));
            break;
        case 4:
            DBG_PRINT("Deadpool, time = %d\r\n", (t2 - t1));
            break;
        case 5:
            DBG_PRINT("Mystique, time = %d\r\n", (t2 - t1));
            break;
        case 6:
            DBG_PRINT("Hulk, time = %d\r\n", (t2 - t1));
            break;
        default:
           // DBG_PRINT("Empty, time = %d\r\n", (t2 - t1));
        }
#endif

#if 1 //enable wifi stream, need error handle
            event = (INT32U)0x80;//obj_result_set.MatchingCardNum;
            temp = osMessagePut(prcess_draw_queue, (uint32_t)&event, osWaitForever);
            //DBG_PRINT("P");
#endif
        //osDelay(300);
        //gplib_ppu_frame_buffer_add(ppu_register_set, prcess_buf);
        dma_frame_buffer_add((INT32U *)prcess_buf, 1);
        prcess_state_post(PRCESS_STATE_OK);
    }
}
#endif

static INT32U Prcess_Callback(INT16U w, INT16U h, INT32U addr)
{
    INT32U disp_buf,event;
    INT32S ret;
/*
    disp_buf = disp_frame_buffer_get(1);
    if(disp_buf)
        fd_display_set_frame(addr,disp_buf,w,h,disp_h_size,disp_v_size);
*/
    event = prcess_state_get();
    if(event == PRCESS_STATE_OK || retry)
    {
        event = dma_frame_buffer_get(1);
        //ret = prcess_frame_buffer_add((INT32U *)prcess_ppu_go(w,h,addr),1);
        if(event < 0)
            retry = 1;
        else
        {
            ret = prcess_dma_start(addr, event);
            //DBG_PRINT("dma %x %x\r\n",addr,event);
            if(ret < 0)
            {
                retry = 1;
                DBG_PRINT("r");
            }
            else
            {

                prcess_frame_buffer_add((INT32U *)event,1);
                DBG_PRINT("a");
            }
        }
    }

    //drv_l2_display_buffer_update_state_get(DISPLAY_DEVICE,1);
    //drv_l2_display_buffer_set(DISPLAY_DEVICE,disp_buf);
    //disp_frame_buffer_add((INT32U *)disp_buf,1);
}

void prcess_task_init(void)
{
    osThreadDef_t       prcess_task = {"prcess_task", prcess_task_entry, osPriorityNormal, 1, 65536};
    osThreadId          id;
    osMessageQDef_t     disp_q = {C_DEMO_DRV_Q_NUM, sizeof(INT32U), 0};
    osSemaphoreDef_t    disp_sem = {0};


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

	if(prcess_draw_queue == NULL)
	{
        prcess_draw_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_draw_queue)
		{
            DBG_PRINT("prcess_draw_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_draw_queue = 0x%x\r\n", prcess_draw_queue);
	}
    // dma
    if(dma_frame_buffer_queue == NULL)
	{
        dma_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!dma_frame_buffer_queue)
		{
            DBG_PRINT("dma_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("dma_frame_buffer_queue = 0x%x\r\n", dma_frame_buffer_queue);
	}

	//fd
	if(sem_fd_engine == NULL)
	{
		sem_fd_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_fd_engine)
		{
            DBG_PRINT("sem_fd_engine error\r\n");
            while(1);
		}

		else
            DBG_PRINT("sem_fd_engine = 0x%x\r\n",sem_fd_engine);
	}



	prcess_dma_init();

	osDelay(5);

    prcess_id = osThreadCreate(&prcess_task, (void *)NULL);
    if(prcess_id == 0)
    {
        DBG_PRINT("osThreadCreate: prcess_id error\r\n");
        while(1);
    }
    else
    {
        osDelay(50);
        //DBG_PRINT("osThreadCreate: prcess_id = 0x%x \r\n", prcess_id);
    }

    wifi_mjpeg_display_callback(Prcess_Callback);
}
