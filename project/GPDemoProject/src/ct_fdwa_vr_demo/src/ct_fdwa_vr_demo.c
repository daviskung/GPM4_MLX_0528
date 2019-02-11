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
#include "ColorTrackerAP.h"
#include "FaceDetectAP.h"
#include "PFMST.h"
#include "vr_demo_global.h"

#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define FRAME_BUF_ALIGN16                   0xF
#define FRAME_BUF_ALIGN8                    0x7
#define FRAME_BUF_ALIGN4                    0x3

#define CT_EN                               1
#define FD_EN                               1
#define VR_EN                               1
#if FD_EN == 1
#define PFMST_EN                            1
#else
#define PFMST_EN                            0
#endif

// demo
#define DEMO_SD_EN                          0
#define GPIO_FUN_EN                         1
#define C_DEVICE_FRAME_NUM		            3
#define DISP_QUEUE_MAX		                C_DEVICE_FRAME_NUM
#define DUMMY_BUFFER_ADDRESS                0x50000000
#define SENSOR_SRC_WIDTH		            1280
#define SENSOR_SRC_HEIGHT		            720
#define PRCESS_SRC_WIDTH		            640//SENSOR_SRC_WIDTH
#define PRCESS_SRC_HEIGHT		            480//SENSOR_SRC_HEIGHT
#define DEMO_DEBUG_EN                       0
#define CSI_PSCALE_USE                      PSCALER_A
#define DISP_PSCALE_USE                     PSCALER_B

//color tracking
#define CT_WIDTH                            80
#define CT_HEIGHT                           60
#define CT_STATE_OK                         0x70
#define OBJDETECT_MAX_RESULT		        1024
#define CT_EN_PIN1                          IO_D8

// face detection
#define FACE_TRACKING_EN                    1
#define SCALER_LINEAR_EN                    0
#define CSI_AE_EN                           0
#define FD_SIZE_HPIXEL                      320
#define FD_SIZE_VPIXEL                      240
#define FD_STATE_OK                         0x80
#define FD_EN_PIN1                          IO_D6
#define RETURN(x)			                {nRet = x; goto Return;}

typedef struct {
	INT8S result_cnt;
	INT8U is_best_face;
	INT8U is_get_eye;
	INT8U is_get_nose;

	INT8U is_get_mouth;
	INT8U is_smile;
	INT8U training_ok;
	INT8U identify_ok;

	INT8U is_hand;
	INT8U is_fist;
	INT8U Reserved0;
	INT8U Reserved1;

	gpRect rect[OBJDETECT_MAX_RESULT];
	gpRect best_face;
	gpRect eye[2];
	gpRect eyebrow[2];
	gpRect nose[2];
	gpRect mouth[4];
} ObjResult_t;

typedef enum
{
        PPUAUTOFORMAT = 0,
        BITMAP_GRAY,
        BITMAP_YUYV,
        BITMAP_RGRB,
        BITMAP_YUV444,
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

static osThreadId csi_id;
static osThreadId disp_id;
static xQueueHandle free_frame_buffer_queue = NULL;
static xQueueHandle csi_frame_buffer_queue = NULL;
static xQueueHandle pscaler_frame_buffer_queue = NULL;
static xQueueHandle disp_frame_buffer_queue = NULL;
static xSemaphoreHandle sem_disp_engine = NULL;
static PSCALER_PARAM_STRUCT PScalerParam = {0};
static INT32U device_h_size, device_v_size;

#if CT_EN == 1
// color tracking
static osThreadId ct_id;
static xQueueHandle ct_frame_buffer_y_queue = NULL;
static xQueueHandle ct_frame_buffer_u_queue = NULL;
static xQueueHandle ct_frame_buffer_v_queue = NULL;
static xQueueHandle ct_free_frame_buffer_y_queue = NULL;
static xQueueHandle ct_free_frame_buffer_u_queue = NULL;
static xQueueHandle ct_free_frame_buffer_v_queue = NULL;
static xQueueHandle ct_state_queue = NULL;
static xSemaphoreHandle sem_ct_engine = NULL;
static INT8U *ct_ybuf, *ct_ubuf, *ct_vbuf;
static INT32U ct_workmem = 0;
static INT32U objNum[3] = {0};
static gpRect objRect[OBJDETECT_MAX_RESULT] = {0};
static INT32U objNum_draw[3] = {0};
static gpRect objRect_draw[OBJDETECT_MAX_RESULT] = {0};
#endif

#if FD_EN == 1
// face detection
static osThreadId fd_id;
static INT32U fd_workmem_size,fd_workmem = 0;
static INT32U fd_ymem = 0;
static INT32U scaler_int_w_step;
static INT32U fd_org_buf;
static ObjResult_t fd_result = {0};
static ObjResult_t fd_draw_result = {0};
static ObjDetect_t ObjWorkMem;
static ObjDetect_t *ObjWorkMem_ptr = NULL;
static xQueueHandle fd_frame_buffer_queue = NULL;
static xQueueHandle fd_free_frame_buffer_queue = NULL;
static xQueueHandle fd_state_queue = NULL;
static xSemaphoreHandle sem_fd_engine = NULL;
static xSemaphoreHandle sem_fd_draw_engine = NULL;
#endif

#if PFMST_EN == 1
// partical filter mean-shift tracker
static gpRect trackingSet[10];
static gpRect trackingSet_result[10];
static INT32U pfmst_workmem = 0;
static TPFMSContext context; ///< Context of PFMS tracking
static TPFMSContext context_result; ///< Context of PFMS tracking
static PFMSTWorkMem PFMSTMem;
static TPFMSParam param =
 {
    {
        6,
        { { 25, 25 }, 4, 4, 0.2 },
        6,  //interative times
        0.1
    },     // PFMS parameters
    15,   // Distance threshold for online learning (= 20)  //10
    0.95,   // Similarity threshold for online learning (0 ~ 0.9)  //0.9
    0.1,   // 0.1  ///< Learning rate (= 0.1)  // 0.6
    15,     // delta_x_thd, track win result change pixel
    15      // delta_y_thd
    ///< PFMS parameters
};
#endif

#if VR_EN == 1
static osThreadId vr_id;
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

#if CT_EN == 1
void ct_gpio1_set(int mode)
{
#if GPIO_FUN_EN == 1
    gpio_write_io(CT_EN_PIN1, mode);
#endif
}
static void ct_gpio_init(void)
{
#if GPIO_FUN_EN == 1
    gpio_init_io (CT_EN_PIN1, GPIO_OUTPUT);
    gpio_set_port_attribute(CT_EN_PIN1, 1);
#endif
}

static void drv_ct_lock(void)
{
    if(sem_ct_engine){
        osSemaphoreWait(sem_ct_engine, osWaitForever);
    }
}

static void drv_ct_unlock(void)
{
    if(sem_ct_engine){
        osSemaphoreRelease(sem_ct_engine);
    }
}
#endif

#if FD_EN == 1
#if FACE_TRACKING_EN == 1
////////////////////////////////////////////////////////////////
// tracking
////////////////////////////////////////////////////////////////
static INT32S obj_track_init(ObjDetect_t *odWorkMem)
{
	INT32U mem_size;
	gpImage *img = &odWorkMem->image;

	mem_size = track_get_memory_size();
	odWorkMem->obj_track_WorkMem = (void *)gp_malloc_align(mem_size, 16);
	if(odWorkMem->obj_track_WorkMem == 0) {
		DBG_PRINT("Fail to allocate obj_detect_WorkMem\r\n");
		return -1;
	}
	DBG_PRINT("obj_track_WorkMem = 0x%x\r\n",odWorkMem->obj_track_WorkMem);
	gp_memset((INT8S *)odWorkMem->obj_track_WorkMem, 0x00, mem_size);

	track_init(odWorkMem->obj_track_WorkMem, img->width, img->height);
	return 0;
}
#endif
void fd_gpio1_set(int mode)
{
#if GPIO_FUN_EN == 1
    gpio_write_io(FD_EN_PIN1, mode);
#endif
}

static void fd_gpio_init(void)
{
#if GPIO_FUN_EN == 1
    gpio_init_io (FD_EN_PIN1, GPIO_OUTPUT);
    gpio_set_port_attribute(FD_EN_PIN1, 1);
#endif
}

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

static void drv_fd_result_lock(void)
{
    if(sem_fd_draw_engine){
        osSemaphoreWait(sem_fd_draw_engine, osWaitForever);
    }
}

static void drv_fd_result_unlock(void)
{
    if(sem_fd_draw_engine){
        osSemaphoreRelease(sem_fd_draw_engine);
    }
}

static INT32S scalerStart_no_fd(gpImage *src, gpImage *dst)
{
    INT32S ret;
    gpRect clip;
	gpImage temp;

	clip.x = 0;
	clip.y = 0;
	clip.width = src->width;
	clip.height = src->height;
    scaler_int_w_step = dst->width;

    ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);

    return ret;
}

static INT32S scalerStart_fd(gpImage *src, gpImage *dst)
{
    INT32S ret;
    gpRect clip;
	gpImage temp;

    drv_fd_lock();
	clip.x = 0;
	clip.y = 0;
	clip.width = src->width;
	clip.height = src->height;
    scaler_int_w_step = dst->width;

    ret = drv_l2_FD_scaler_clip(0, 1, src, dst, &clip);
    drv_fd_unlock();

    return ret;
}

static INT32S scalerEnd(void)
{
    return 0;
}

static INT32S scalerClip_fd(gpImage *src, gpImage *dst, gpRect *clip)
{
    return drv_l2_FD_scaler_clip(0, 1, src, dst, clip);
}

static int face_Detect_only(const gpImage* gray, gpRect* userROI, int mode, int img_num)
{
    CHAR face_path[256];
	INT16S fd;
    // detect type setting
	ClassifyData clfface;
	// best face reye and leye
	gpRect* faceROI = &userROI[0];
	// detect result and count
	gpRect Rect,rFace,lFace;
	gpRect faceResult[OBJDETECT_MAX_RESULT] = {0};
	int faceCount[OBJDETECT_MAX_RESULT] = {0};

	// face detect workmem
	void *WorkMem = 0;
	// Initialization //
	int i, maxFaceW, maxFaceCount, best_face, offset_x, offset_y, min_eye_wnd, max_eye_wnd;
	int ret,faceN, rEyeN, lEyeN, t, int_scale_face, int_scale_eye, detectH, fd_state;
	// xstep
	int xstep_face = 2;
	// ystep
	int ystep_face = 2;
	// face nbr
	int min_face_nbr_h = 8;
	int min_face_nbr_l = 6;
	// min face window
	int min_face_wnd = 50;//50;
	// sacler max window
	int max_wnd = MIN(gray->width, gray->height);
	// memory size range
	int HEIGHT = gray->height;
	int WIDTH = gray->width;

#if FACE_TRACKING_EN == 1
	INT32S nCnt, *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	INT32S search_cnt, max_result_t;
	gpRect *p_range;
	gpRect *p_result;
    gpRect *p_obj_rect = userROI;

    Rect.x = 0;
	Rect.y = 0;
	Rect.width = gray->width;
	Rect.height = gray->height;

    t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, OBJDETECT_MAX_RESULT, xstep_face, ystep_face);
    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t, 64);
        fd_workmem_size = t;
        WorkMem = (void *)fd_workmem;
        DBG_PRINT("FD_WorkMem:0x%X \r\n",WorkMem);
    }
    else
        WorkMem = (void *)fd_workmem;

    gp_memset((INT8S *)WorkMem,0, t);
    gp_memset((INT8S *)&userROI[0],0, 3*sizeof(gpRect));

    FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, OBJDETECT_MAX_RESULT, xstep_face, ystep_face);
	//get tracking info
	p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);
	search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);
	min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
	i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);

	//set face detect object type
    clfface.obj_type = OBJ_FACE;
	FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
	p_result = faceResult;
	max_result_t = OBJDETECT_MAX_RESULT;
	p_count = faceCount;
	nCnt = 0;
	i = search_cnt;
	do {
		INT32S cnt;

		min_face_wnd = *p_min_width++;
		int_scale_face = *p_scale++;
		FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);

		xstep_face = *p_xstep++;
		FaceDetect_SetX(WorkMem, xstep_face, 0);

		cnt = FaceDetect(WorkMem, gray, p_range, max_result_t, p_result, p_count);

		max_result_t -= cnt;
		p_result += cnt;
		p_count += cnt;
		nCnt += cnt;
		p_range++;
		i--;
	} while(i > 0);

	maxFaceW = 0;
	maxFaceCount = min_face_nbr_l;
	best_face = 0;
	search_cnt = 0;
    for (i=0; i<nCnt; i++) {
        if (faceCount[i] >= min_face_nbr_l)
        {
            p_obj_rect->x = faceResult[i].x;
            p_obj_rect->y = faceResult[i].y;
            p_obj_rect->width = faceResult[i].width;
            p_obj_rect->height = faceResult[i].height;
            p_obj_rect++;
            search_cnt++;
            if ((maxFaceCount >= min_face_nbr_h) && (faceCount[i] >= min_face_nbr_h))
            {
                if ((faceResult[i].width > 0.7*maxFaceW) && (faceCount[i] > maxFaceCount))
                {
                    maxFaceW = faceResult[i].width;
                    maxFaceCount = faceCount[i];
                    best_face = i;
                }
            }
            else
            {
                if (faceCount[i] >= maxFaceCount)
                {
                    maxFaceW = faceResult[i].width;
                    maxFaceCount = faceCount[i];
                    best_face = i;
                }
            }
        }
    }

    //DBG_PRINT("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
    // set tracking
    if(search_cnt > 0) {
        track_reset_face_location(ObjWorkMem_ptr->obj_track_WorkMem);
        p_obj_rect = (gpRect *)userROI;
        for (i = 0; i <search_cnt; i++) {
            track_set_face_location(ObjWorkMem_ptr->obj_track_WorkMem, &p_obj_rect[i], i);
        }
    }

    // tracking
    track_run(ObjWorkMem_ptr->obj_track_WorkMem, search_cnt);

    #if FD_DEBUG_EN == 1
        DBG_PRINT("\r\n****************************ROI******************************\r\n");
        DBG_PRINT("best_face.count = %d\r\n",faceCount[best_face]);
        DBG_PRINT("best_face.x = %d\r\n", faceResult[best_face].x);
        DBG_PRINT("best_face.y = %d\r\n", faceResult[best_face].y);
        DBG_PRINT("best_face.width = %d\r\n", faceResult[best_face].width);
        DBG_PRINT("best_face.height = %d\r\n", faceResult[best_face].height);
    #endif

    if(!maxFaceW)
    {
        // release memory //
        return 0;
    }
    else
        fd_state = 1;
#else
    Rect.x = 0;
	Rect.y = 0;
	Rect.width = gray->width;
	Rect.height = gray->height;

    #if FD_ONE_FRAME_DEBUG_MODE_EN == 1
        int_scale_face = 74054;//74054;//72089; // 1.1
    #else
        int_scale_face = 72089;//74054;//72089; // 1.1
    #endif

	t = MIN(Rect.width, Rect.height);
	if(max_wnd>t) max_wnd = t;

	t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, OBJDETECT_MAX_RESULT, xstep_face, ystep_face);

    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t, 64);
        fd_workmem_size = t;
        WorkMem = (void *)fd_workmem;
        DBG_PRINT("FD_WorkMem:0x%X \r\n",WorkMem);
    }
    else
        WorkMem = (void *)fd_workmem;

    gp_memset((INT8S *)WorkMem,0, fd_workmem_size);
    gp_memset((INT8S *)&userROI[0],0, 3*sizeof(gpRect));
	//--------------------
	//	Face Detection
	//--------------------
	ret = FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, OBJDETECT_MAX_RESULT, xstep_face, ystep_face);

	/* setting cascade type (face) */
	clfface.obj_type = OBJ_FACE;
    FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);

	FaceDetect_SetX(WorkMem, xstep_face, 0);

	faceN = FaceDetect(WorkMem, gray, &Rect, OBJDETECT_MAX_RESULT, faceResult, faceCount);

	if (!faceN)
	{
        // release memory //
        return 0;
	}
	else
        fd_state = 1;

	maxFaceW = 0;
	maxFaceCount = min_face_nbr_l;
	best_face = 0;
	i = faceN-1;

	do
	{
        if (faceCount[i] >= min_face_nbr_l)
        {
                if ((maxFaceCount >= min_face_nbr_h) && (faceCount[i] >= min_face_nbr_h))
                {
                        if ((faceResult[i].width > 0.7*maxFaceW) && (faceCount[i] > maxFaceCount))
                        {
                                maxFaceW = faceResult[i].width;
                                maxFaceCount = faceCount[i];
                                best_face = i;
                        }
                }
                else
                {
                        if (faceCount[i] >= maxFaceCount)
                        {
                                maxFaceW = faceResult[i].width;
                                maxFaceCount = faceCount[i];
                                best_face = i;
                        }
                }
        }
	} while (i--);

	#if FD_DEBUG_EN == 1
        DBG_PRINT("\r\n****************************ROI******************************\r\n");
		DBG_PRINT("best_face.count = %d\r\n",faceCount[best_face]);
		DBG_PRINT("best_face.x = %d\r\n", faceResult[best_face].x);
		DBG_PRINT("best_face.y = %d\r\n", faceResult[best_face].y);
		DBG_PRINT("best_face.width = %d\r\n", faceResult[best_face].width);
		DBG_PRINT("best_face.height = %d\r\n", faceResult[best_face].height);
    #endif

	if(!maxFaceW)
	{
		// release memory //
        return 0;
	}
	else
        fd_state = 1;
#endif

	/* Face Position Determination */
	offset_x = faceResult[best_face].width * 0.16;
	offset_y = faceResult[best_face].height/7;

	faceROI->x = faceResult[best_face].x + offset_x;
	faceROI->y = faceResult[best_face].y + offset_y;
	faceROI->width = (short)(faceResult[best_face].width - (offset_x<<1));
	faceROI->height = (short)(faceResult[best_face].height - offset_y*1.5);

	#if FD_DEBUG_EN == 1
		DBG_PRINT("faceROI.x = %d\r\n", faceROI->x);
		DBG_PRINT("faceROI.y = %d\r\n", faceROI->y);
		DBG_PRINT("faceROI.width = %d\r\n", faceROI->width);
		DBG_PRINT("faceROI.height = %d\r\n", faceROI->height);
    #endif

    return fd_state;
}

static INT32U fd_scaler_step_get(void)
{
    return (scaler_int_w_step - 1);
}
#endif

// prcess ready buffer queue

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

#if CT_EN == 1
static INT32S ct_frame_buffer_y_post(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(ct_frame_buffer_y_queue, (uint32_t)&event, osWaitForever);

	return temp;
}
// prcess state queue
static INT32S ct_state_post(INT32U state)
{
    INT32U event,temp;

    event = (INT32U)state;
    temp = osMessagePut(ct_state_queue, (uint32_t)&event, osWaitForever);

	return temp;
}
static INT32S ct_state_get(void)
{
    osEvent result;
	INT32S state = 0;

    result = osMessageGet(ct_state_queue, 10);
    state = result.value.v;
    if((result.status != osEventMessage) || (state!=CT_STATE_OK)) {
        state = 0;
	}

	return state;
}
#endif

#if FD_EN == 1
// prcess state queue
static INT32S fd_state_post(INT32U state)
{
    INT32U event,temp;

    event = (INT32U)state;
    temp = osMessagePut(fd_state_queue, (uint32_t)&event, osWaitForever);

	return temp;
}
static INT32S fd_state_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(fd_state_queue, 1);
    frame = result.value.v;
    if((result.status != osEventMessage) || (frame!=FD_STATE_OK)) {
        frame = 0;
	}

	return frame;
}
#endif

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

static void draw_rectx4(gpImage *image, gpRect *r, INT32U color)
{
	INT32S i;
	INT32U *p_img;
	INT32S len;
	INT32S addr;
	INT32S x, y, w, h;
	INT16U *img = (INT16U *) image->ptr;

    w = r->width << 2;
	h = r->height << 2;
	x = r->x << 2;
	y = r->y << 2;

    //DBG_PRINT("image->width[%d]\r\n", image->width);
    //DBG_PRINT("image->height[%d]\r\n", image->height);
	//w = MIN(r->width, image->width - r->x - 20);
	//h = MIN(r->height, image->height - r->y - 20);


	if(w !=0 && h !=0) {
		//x = r->x;
		//y = r->y;

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

static void draw_obj(INT32U img_adr, INT32U bufw, INT32U bufh, INT32S total_obj_cnt, gpRect *obj_result_d, INT32U color)
{
	INT32S i;
	//INT32U color;
	gpImage Image;

	Image.width = bufw;					// depends on display buffer width
	Image.height = bufh;				// depends on display buffer height
	Image.widthStep = bufw*2; 			// depends on display format
	Image.ch = 2; 						// depends on display format
	Image.format = IMG_FMT_UYVY;  		// depends on display format
	Image.ptr = (INT8U *)img_adr;

    //color = 0xd211d290;// yellow;
	//color = 0xff4c554c; // green
	//color = 0x4cff4c55; // blue
	//color = 0xffffffff; // red
	for(i=0; i<total_obj_cnt; i++) {
		draw_rect(&Image, &obj_result_d[i], color);
	}
}

static void draw_objx4(INT32U img_adr, INT32U bufw, INT32U bufh, INT32S total_obj_cnt, gpRect *obj_result_d, INT32U color)
{
	INT32S i;
	//INT32U color;
	gpImage Image;

	Image.width = bufw;					// depends on display buffer width
	Image.height = bufh;				// depends on display buffer height
	Image.widthStep = bufw*2; 			// depends on display format
	Image.ch = 2; 						// depends on display format
	Image.format = IMG_FMT_UYVY;  		// depends on display format
	Image.ptr = (INT8U *)img_adr;

	//color = 0xff4c554c; // green
	//color = 0x4cff4c55; // blue
	//color = 0xffffffff; // red
	for(i=0; i<total_obj_cnt; i++) {
		draw_rectx4(&Image, &obj_result_d[i], color);
	}
}

#if CT_EN == 1
 static INT32S ct_set_frame(INT32U in_buffer, INT32U frame_buffer_y, INT32U frame_buffer_u, INT32U frame_buffer_v, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
    ObjFrame_t *pInput,Input;

 	if(frame_buffer_y == 0 || frame_buffer_u == 0 || frame_buffer_v == 0 || in_buffer == 0) {
		return -1;
	}

    pInput = (ObjFrame_t *)&Input;
	if(pInput == 0) {
		return -1;
	}

#if FD_EN == 1
    drv_fd_lock();
#endif
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
	pInput->ScaleOut.format = IMG_FMT_YUV444;
	pInput->ScaleOut.ptr = (INT8U *)frame_buffer_y;
	pInput->ScaleOut.ptr_u = (INT8U *)frame_buffer_u;
	pInput->ScaleOut.ptr_v = (INT8U *)frame_buffer_v;
    drv_l2_scaler_full_screen(0, 1, (gpImage *)&pInput->ScaleIn, (gpImage *)&pInput->ScaleOut);
#if FD_EN == 1
    drv_fd_unlock();
#endif

    return 0;
 }
 #endif

#if FD_EN == 1
static INT32S fd_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h, INT32U infmt)
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

    drv_fd_lock();
	pInput->ScaleIn.width = in_w;
	pInput->ScaleIn.height = in_h;
    pInput->ScaleIn.ch  = 2;
    pInput->ScaleIn.widthStep = in_w * 2;
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
	scalerStart_no_fd(&pInput->ScaleIn, &pInput->ScaleOut);
    drv_fd_unlock();
	scalerEnd();

	nRet = STATUS_OK;
Return:
	return nRet;
}
#endif

static INT32S fd_display_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
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
        if(csiBuf && csiBuf != 0x5000000)
        {
            prcessBuf = drv_l1_pscaler_output_A_buffer_get(PScalerParam.pScalerNum);
            csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            drv_l1_pscaler_output_A_buffer_set(PScalerParam.pScalerNum,(INT32U)csiBuf);
        }
        else
        {
            prcessBuf = drv_l1_pscaler_output_A_buffer_get(PScalerParam.pScalerNum);
            if(prcessBuf && prcessBuf != 0x50000000)
                csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            drv_l1_pscaler_output_A_buffer_set(PScalerParam.pScalerNum,(INT32U)0x5000000);
            //DBG_PRINT("A");
        }
	}
	else if(PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
	{
        csiBuf = free_frame_buffer_get(0);
        if(csiBuf && csiBuf != 0x5000000)
        {
            prcessBuf = drv_l1_pscaler_output_B_buffer_get(PScalerParam.pScalerNum);
            csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            drv_l1_pscaler_output_B_buffer_set(PScalerParam.pScalerNum,(INT32U)csiBuf);
        }
        else
        {
            prcessBuf = drv_l1_pscaler_output_B_buffer_get(PScalerParam.pScalerNum);
            if(prcessBuf && prcessBuf != 0x50000000)
                csi_frame_buffer_add((INT32U *)prcessBuf, 0);
            drv_l1_pscaler_output_B_buffer_set(PScalerParam.pScalerNum,0x50000000);
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

	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*DISP_QUEUE_MAX)+64), 64);
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

#if FD_EN == 1
static INT32S fd_frame_buffer_post(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(fd_frame_buffer_queue, (uint32_t)&event, osWaitForever);

	return temp;
}


static INT32S fd_free_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(fd_free_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(fd_free_frame_buffer_queue, (uint32_t)&event, 10);
    }


	return temp;
}

static INT32S fd_free_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(fd_free_frame_buffer_queue, 10);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}
#endif

#if CT_EN == 1
static INT32S ct_free_frame_buffer_y_add(INT32U *frame_buf_y, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf_y) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf_y;
        temp = osMessagePut(ct_free_frame_buffer_y_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf_y;
        temp = osMessagePut(ct_free_frame_buffer_y_queue, (uint32_t)&event, 10);
    }

	return temp;
}
static INT32S ct_free_frame_buffer_y_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(ct_free_frame_buffer_y_queue, 10);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}
#endif

static void csi_task_entry(void const *parm)
{
    #define CSI_SOFTWARE_CNT_EN             1
    INT32U i,csi_buf,PscalerBuffer,PscalerBufferSize, FDBuffer;
    osEvent result;
#if CT_EN == 1
    INT32S ct_event;
    INT32U ct_size,CTBuffer_y, CTBuffer_u, CTBuffer_v;
    INT32U ct_frame_y, ct_frame_u, ct_frame_v;
    INT32U ctNum_draw[3] = {0};
    gpRect ctRect_draw[16] = {0};
#endif
#if FD_EN == 1
    INT32S fd_event;
    INT32U fd_size, fd_frame;
    ObjResult_t fd_draw = {0};
#if PFMST_EN == 1
    gpRect trackingSet_draw[10] = {0};
    TPFMSContext context_draw = {0};
#endif
#endif
#if CSI_SOFTWARE_CNT_EN == 1
    INT32U t1, t2;
#endif

    DBG_PRINT("csi_task_entry start \r\n");

    // csi init
    mazeTest_Preview_PScaler();

    // disp size
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&device_h_size, (INT16U *)&device_v_size);
	PscalerBufferSize = (device_h_size * device_v_size * 2);
	PscalerBuffer = (INT32U) gp_malloc_align(((PscalerBufferSize*C_DEVICE_FRAME_NUM)+64), 64);
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

#if CT_EN == 1
	// ct buffer
	ct_size = CT_WIDTH*CT_HEIGHT;
    CTBuffer_y = (INT32U) gp_malloc_align((((ct_size*3)*C_DEVICE_FRAME_NUM)+64),64);
    if(CTBuffer_y == 0)
    {
        DBG_PRINT("CTBuffer_y fail\r\n");
        while(1);
    }
    else
        CTBuffer_y = (INT32U)((CTBuffer_y + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);

    for(i=0;i<C_DEVICE_FRAME_NUM;i++)
    {
        ct_frame_y = (INT32U)(CTBuffer_y+(i*(ct_size*3)));
        ct_frame_u = (INT32U)(ct_frame_y+ct_size);
        ct_frame_v = (INT32U)(ct_frame_u+ct_size);

        ct_free_frame_buffer_y_add((INT32U *)ct_frame_y, 1);
        DBG_PRINT("CTBuffer%d y=0x%x, u=0x%x, v=0x%x\r\n",i,ct_frame_y, ct_frame_u, ct_frame_v);
    }
#endif

#if FD_EN == 1
	// fd buffer
	fd_size = FD_SIZE_HPIXEL*FD_SIZE_VPIXEL*2;
    FDBuffer = (INT32U) gp_malloc_align(((fd_size*C_DEVICE_FRAME_NUM)+64),64);
    if(FDBuffer == 0)
    {
        DBG_PRINT("FDBuffer fail\r\n");
        while(1);
    }
    FDBuffer = (INT32U)((FDBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    for(i=0;i<C_DEVICE_FRAME_NUM;i++)
    {
        fd_frame = (INT32U)(FDBuffer+(i*fd_size));
        fd_free_frame_buffer_add((INT32U *)fd_frame, 1);
        DBG_PRINT("FDBuffer%d = 0x%x\r\n",i,fd_frame);
    }
#endif

#if CT_EN == 1
    ct_state_post(CT_STATE_OK);
#endif

#if FD_EN == 1
    fd_state_post(FD_STATE_OK);
#endif

    while(1)
    {
        result = osMessageGet(csi_frame_buffer_queue, osWaitForever);
        csi_buf = result.value.v;
        if((result.status != osEventMessage) || !csi_buf) {
            continue;
        }
        //DBG_PRINT("csi_buffer = 0x%x\r\n", csi_buf);
        //DBG_PRINT("c");

#if CSI_SOFTWARE_CNT_EN == 1
        t1 = xTaskGetTickCount();
#endif
        PscalerBuffer = pscaler_frame_buffer_get(1);
        if(PscalerBuffer)
            fd_display_set_frame(csi_buf, PscalerBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, device_h_size, device_v_size);
#if CT_EN == 1
        ct_event = ct_state_get();
        if(ct_event == CT_STATE_OK)
        {
            CTBuffer_y = ct_free_frame_buffer_y_get();
            if(CTBuffer_y)
            {
                CTBuffer_u = CTBuffer_y + ct_size;
                CTBuffer_v = CTBuffer_u + ct_size;
                ct_set_frame(csi_buf,CTBuffer_y, CTBuffer_u, CTBuffer_v,PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, CT_WIDTH, CT_HEIGHT);
                ct_frame_buffer_y_post((INT32U *)CTBuffer_y);
            }
        }
#endif

#if FD_EN == 1
        fd_event = fd_state_get();
        if(fd_event == FD_STATE_OK)
        {
            FDBuffer = fd_free_frame_buffer_get();
            if(FDBuffer)
            {
    #if PFMST_EN == 1
                fd_set_frame(csi_buf,FDBuffer,PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_YUYV);
    #else
                fd_set_frame(csi_buf,FDBuffer,PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_GRAY);
    #endif
                fd_frame_buffer_post((INT32U *)FDBuffer);
            }
        }
#endif

        if(PscalerBuffer)
        {
#if CT_EN == 1
            if(ct_event == CT_STATE_OK)
            {
                drv_ct_lock();
                ctNum_draw[0] = objNum_draw[0];
                ctNum_draw[1] = objNum_draw[1];
                ctNum_draw[2] = objNum_draw[2];
                for (i = 0; i < 9; ++i)
                    ctRect_draw[i] = objRect_draw[i];
                drv_ct_unlock();
            }
            // blue
            if (ctNum_draw[0])
                draw_objx4(PscalerBuffer, device_h_size, device_v_size, ctNum_draw[0], &ctRect_draw[0], 0x4cff4c55);
            // green
            if (ctNum_draw[1])
                draw_objx4(PscalerBuffer, device_h_size, device_v_size, ctNum_draw[1], &ctRect_draw[3], 0xff4c554c);
            // red
            if (ctNum_draw[2])
                draw_objx4(PscalerBuffer, device_h_size, device_v_size, ctNum_draw[2], &ctRect_draw[6], 0xffffffff);
#endif

#if FD_EN == 1
            if(fd_event == FD_STATE_OK)
            {
                drv_fd_result_lock();
                fd_draw.is_best_face = fd_draw_result.is_best_face;
                fd_draw.is_get_eye = fd_draw_result.is_get_eye;
                fd_draw.result_cnt = fd_draw_result.result_cnt;
                fd_draw.rect[0].x = fd_draw_result.rect[0].x;
                fd_draw.rect[0].y = fd_draw_result.rect[0].y;
                fd_draw.rect[0].width = fd_draw_result.rect[0].width;
                fd_draw.rect[0].height = fd_draw_result.rect[0].height;
#if PFMST_EN == 1
                context_draw.detect_total = context_result.detect_total;
                context_draw.trackBackCnt = context_result.trackBackCnt;
                trackingSet_draw[0].width = trackingSet_result[0].width;
                trackingSet_draw[0].height = trackingSet_result[0].height;
                trackingSet_draw[0].x = trackingSet_result[0].x;
                trackingSet_draw[0].y = trackingSet_result[0].y;
#endif
                drv_fd_result_unlock();
            }

            if(fd_draw.result_cnt)
            {
                draw_obj(PscalerBuffer, device_h_size, device_v_size, fd_draw.result_cnt, &fd_draw.rect[0], 0xd211d290);
            }
#if PFMST_EN == 1
            else if((context_draw.detect_total == 0) && (context_draw.trackBackCnt != 0))
            {
                draw_obj(PscalerBuffer, device_h_size, device_v_size, 1, &trackingSet_draw[0], 0xd211d290/*0xd211d290*//*0x10801080*/);
            }
#endif
#endif
            disp_frame_buffer_add((INT32U *)PscalerBuffer, 1);
        }
        //else
            //DBG_PRINT("@");

#if CT_EN == 1
#if FD_EN == 1
        // CT_EN == 1 & FD_EN == 1
        if((ct_event != CT_STATE_OK) && (fd_event != FD_STATE_OK))
#else
        // CT_EN == 1 & FD_EN == 0
        if(ct_event != CT_STATE_OK)
#endif
#else
#if FD_EN == 1
        // CT_EN == 0 & FD_EN == 1
        if(fd_event != FD_STATE_OK)
#endif
#endif
            free_frame_buffer_add((INT32U *)csi_buf, 1);
#if CSI_SOFTWARE_CNT_EN == 1
        t2 = xTaskGetTickCount();
        if((t2 - t1) > 20)
            DBG_PRINT("csi_task = %d \r\n", (t2 - t1));
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

#if CT_EN == 1
static void ct_task_entry(void const *parm)
{
    #define CT_SOFTWARE_CNT_EN             0
    INT32U i,ct_size;
    osEvent result;
    ColorTracker_Input ctInput;
    #if CT_SOFTWARE_CNT_EN == 1
    INT32U t1, t2;
    #endif
    #if DEMO_DEBUG_EN == 1
    INT32U t3,t4;
    #endif

    DBG_PRINT("ct prcess_task_entry start \r\n");
    //**************************************//
        //DBG_PRINT("user init add \r\n");
    //**************************************//
    ct_size = CT_WIDTH*CT_HEIGHT;
    ct_gpio_init();
    ctInput.maxWnd = MIN(CT_WIDTH, CT_HEIGHT);
    ctInput.minWnd = 4;
    ctInput.MAX_RESULT = OBJDETECT_MAX_RESULT;
    ctInput.WorkMem = gp_malloc_align(ColorTracker_MemCalc(), 64);
    if(ctInput.WorkMem == 0)
    {
        DBG_PRINT("CT_WorkMem malloc fail \r\n");
        while(1);
    }

    DBG_PRINT("CT_WorkMem:0x%X \r\n",ctInput.WorkMem);

    while(1)
    {
        result = osMessageGet(ct_frame_buffer_y_queue, osWaitForever);
        ctInput.y_array = (unsigned char*)(result.value.v);
        if((result.status != osEventMessage) || !ctInput.y_array) {
            continue;
        }
        ctInput.u_array = ctInput.y_array + ct_size;
        ctInput.v_array = ctInput.u_array + ct_size;
        //DBG_PRINT("CT");

        ct_gpio1_set(1);
        #if CT_SOFTWARE_CNT_EN == 1
        t1 = xTaskGetTickCount();
        #endif
        #if DEMO_DEBUG_EN == 1
        t3 = xTaskGetTickCount();
        #endif
        // ColorTracker prcess
        ColorTracker(&ctInput, objNum, objRect);

        #if DEMO_DEBUG_EN == 1
        t4 = xTaskGetTickCount();
        DBG_PRINT("ct: time=%d \r\n", (t4 - t3));
        #endif
        drv_ct_lock();
        // ColorTracker result
        objNum_draw[0] = objNum[0];
        objNum_draw[1] = objNum[1];
        objNum_draw[2] = objNum[2];
        for (i = 0; i < 9; ++i)
            objRect_draw[i] = objRect[i];
        drv_ct_unlock();
        ct_state_post(CT_STATE_OK);
        ct_free_frame_buffer_y_add((INT32U *)ctInput.y_array, 1);
        #if CT_SOFTWARE_CNT_EN == 1
        t2 = xTaskGetTickCount();
        DBG_PRINT("CT=%d \r\n", (t2 - t1));
        #endif
        ct_gpio1_set(0);
    }
}
#endif

#if FD_EN == 1
static void fd_task_entry(void const *parm)
{
    #define FD_SOFTWARE_CNT_EN             0
    INT32U i,temp,no_face_cnt,y_buffer;
    gpImage *pgray,gray_ptr;
    osEvent result;
    #if FD_SOFTWARE_CNT_EN == 1
    INT32U t1,t2;
    #endif
    #if DEMO_DEBUG_EN == 1
    INT32U t3,t4;
    #endif

    DBG_PRINT("fd_task_entry start \r\n");

    pgray = (gpImage *)&gray_ptr;
    fd_gpio_init();
#if FACE_TRACKING_EN == 1
    ObjWorkMem_ptr = (ObjDetect_t *)&ObjWorkMem;
 	ObjWorkMem_ptr->image.width     = FD_SIZE_HPIXEL;
	ObjWorkMem_ptr->image.height    = FD_SIZE_VPIXEL;
    ObjWorkMem_ptr->image.ch        = 1;
    ObjWorkMem_ptr->image.widthStep = FD_SIZE_HPIXEL;
    ObjWorkMem_ptr->image.format    = IMG_FMT_GRAY;
    if(obj_track_init((ObjDetect_t *)ObjWorkMem_ptr))
    {
        DBG_PRINT("obj_track_init malloc fail \r\n");
        while(1);
    }
#endif

#if PFMST_EN == 1
	//***********************************************************
	//           Partical Filter Mean Shift Tracker Init
	//***********************************************************
	unsigned int PFMST_size_byte = getPFMSTrackingMemorySize();

	if(pfmst_workmem == 0)
    {
        pfmst_workmem = (INT32U)gp_malloc_align(PFMST_size_byte,64);
        if(pfmst_workmem == 0)
        {
            DBG_PRINT("PFMST WorkMem malloc fail \r\n");
            while(1);
        }
    }
    DBG_PRINT("PFMST WorkMem:0x%X \r\n", pfmst_workmem);
	PFMSTMem.workMem = (unsigned char*)pfmst_workmem;
	initGPTrack( PFMSTMem.workMem, &context, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
	context.detectMode = GP_FACE;  //for GP face detector

	gpImage ROI_Input;

    ROI_Input.width = FD_SIZE_HPIXEL;
    ROI_Input.widthStep = FD_SIZE_HPIXEL*2;
    ROI_Input.height = FD_SIZE_VPIXEL;
    ROI_Input.ch = 2;
    ROI_Input.format = IMG_FMT_UYVY;//IMG_FMT_YUYV;IMG_FMT_RGB;//
#endif
    // for new int mode use
    drv_l1_scaler_new_int_callback_set(fd_scaler_step_get);
    no_face_cnt = 0;
    fd_org_buf = 0;
    osDelay(5);

#if PFMST_EN == 1
    if(fd_ymem == 0)
    {
        fd_ymem = (INT32U)gp_malloc_align(FD_SIZE_HPIXEL*FD_SIZE_VPIXEL, 64);
        if(fd_ymem == 0)
        {
            DBG_PRINT("FD_Y WorkMem malloc fail \r\n");
            while(1);
        }
        DBG_PRINT("FD_Y WorkMem:0x%X \r\n", fd_ymem);
    }
#endif

    while(1)
    {
        result = osMessageGet(fd_frame_buffer_queue, osWaitForever);
        fd_org_buf = result.value.v;
        if(result.status != osEventMessage || !fd_org_buf) {
            continue;
        }
        //DBG_PRINT("FD");

        #if FD_SOFTWARE_CNT_EN == 1
        t1 = xTaskGetTickCount();
        #endif
        fd_gpio1_set(1);
        pgray->width = FD_SIZE_HPIXEL;
        pgray->height = FD_SIZE_VPIXEL;
        pgray->widthStep = FD_SIZE_HPIXEL;
        pgray->ch = 1;
        pgray->format = IMG_FMT_GRAY;

#if PFMST_EN == 1
        fd_set_frame(fd_org_buf, fd_ymem, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, BITMAP_GRAY);
        pgray->ptr = (INT8U *)fd_ymem;
#else
        pgray->ptr = (INT8U *)fd_org_buf;
#endif

        #if DEMO_DEBUG_EN == 1
        t3 = xTaskGetTickCount();
        #endif
        #if SCALER_LINEAR_EN == 1
            y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_LINEAR,0);
        #else
            y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,0);
        #endif
        #if DEMO_DEBUG_EN == 1
        t4 = xTaskGetTickCount();
        DBG_PRINT("fd: time=%d \r\n", (t4 - t3));
        #endif

#if PFMST_EN == 1
        context.detect_total = y_buffer;
        #if DEMO_DEBUG_EN == 1
        t3 = xTaskGetTickCount();
        #endif
        //PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &param, &context, &fd_result.rect[0]);
        PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context, &fd_result.rect[0]);
        #if DEMO_DEBUG_EN == 1
        t4 = xTaskGetTickCount();
        if (y_buffer != 0)
            DBG_PRINT("model construction: time=%d \r\n", (t4 - t3));
        else if (context.tracking_flag != 0)
            DBG_PRINT("tracking: time=%d \r\n", (t4 - t3));
        else
            DBG_PRINT("release: time=%d \r\n", (t4 - t3));
        #endif
#endif
        drv_fd_result_lock();
        if(y_buffer)
        {
            fd_result.is_best_face = 1;
            fd_result.is_get_eye = 0;
            fd_result.result_cnt = 1;
            fd_draw_result.is_best_face = fd_result.is_best_face;
            fd_draw_result.is_get_eye = fd_result.is_get_eye;
            fd_draw_result.result_cnt = fd_result.result_cnt;
            fd_draw_result.rect[0].x = fd_result.rect[0].x;
            fd_draw_result.rect[0].y = fd_result.rect[0].y;
            fd_draw_result.rect[0].width = fd_result.rect[0].width;
            fd_draw_result.rect[0].height = fd_result.rect[0].height;
            no_face_cnt = 0;
        }
        else
        {
            fd_result.is_best_face = 0;
            fd_result.is_get_eye = 0;
            fd_result.result_cnt = 0;
            fd_draw_result.is_best_face = fd_result.is_best_face;
            fd_draw_result.is_get_eye = fd_result.is_get_eye;
            fd_draw_result.result_cnt = fd_result.result_cnt;
#if PFMST_EN == 1
            context_result.detect_total = context.detect_total;
            context_result.trackBackCnt = context.trackBackCnt;
            trackingSet_result[0].width = (unsigned int)((context.trackObj_srcIMG.width)*((float)device_h_size/(float)context.SRCIMG.width));
            trackingSet_result[0].height = (unsigned int)((context.trackObj_srcIMG.height)*((float)device_v_size/(float)context.SRCIMG.height));
            trackingSet_result[0].x = (unsigned int)((context.trackObj_srcIMG.x)*((float)device_h_size/(float)context.SRCIMG.width));
            trackingSet_result[0].y = (unsigned int)((context.trackObj_srcIMG.y)*((float)device_v_size/(float)context.SRCIMG.height));
#endif
            no_face_cnt++;
        }
        drv_fd_result_unlock();
        fd_state_post(FD_STATE_OK);
        fd_free_frame_buffer_add((INT32U *)fd_org_buf, 1);
#if CSI_AE_EN == 1
        // set sensor ae
        if(obj_result.is_best_face > 0) {
            sensor_control_ae();
        }

        if(no_face_cnt >=16) {
            no_face_cnt = 16;
            sensor_control_ae();
        }
#endif
        #if FD_SOFTWARE_CNT_EN == 1
        t2 = xTaskGetTickCount();
        DBG_PRINT("F=%d \r\n", (t2 - t1));
        #endif
        fd_gpio1_set(0);
    }
}
#endif

#if VR_EN == 1
static void vr_task_entry(void const *parm)
{
    DBG_PRINT("vr_task_entry start \r\n");

    VrDemoGlobalInit() ;
    VrTtsEnable(VR_ONLY);
	audio_encode_entrance();
    TestVR_Start() ;

    while(1)
        osDelay(10);
}
#endif

void GPM4_CT_FDWA_VR_Demo(void)
{
    osThreadDef_t csi_task = {"csi_task", csi_task_entry, osPriorityAboveNormal, 1, 16384};
    osThreadDef_t disp_task = {"disp_task", disp_task_entry, osPriorityNormal, 1, 8192};
#if CT_EN == 1
    osThreadDef_t ct_task = {"ct_task", ct_task_entry, osPriorityNormal, 1, 32768};
#endif
#if FD_EN == 1
    osThreadDef_t fd_task = {"fd_task", fd_task_entry, osPriorityNormal, 1, 65536};
#endif
#if VR_EN == 1
    osThreadDef_t vr_task = {"vr_task", vr_task_entry, osPriorityNormal, 1, 16384};
#endif
    osMessageQDef_t disp_q = {DISP_QUEUE_MAX, sizeof(INT32U), 0};
    osSemaphoreDef_t disp_sem = {0};

#if DEMO_SD_EN == 1
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

#if CT_EN == 1
	if(sem_ct_engine == NULL)
	{
		sem_ct_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_ct_engine)
            while(1);
		else
            DBG_PRINT("sem_ct_engine = 0x%x\r\n",sem_ct_engine);
	}
#endif

#if FD_EN == 1
	if(sem_fd_engine == NULL)
	{
		sem_fd_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_fd_engine)
            while(1);
		else
            DBG_PRINT("sem_fd_engine = 0x%x\r\n",sem_fd_engine);
	}
	if(sem_fd_draw_engine == NULL)
	{
		sem_fd_draw_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_fd_draw_engine)
            while(1);
		else
            DBG_PRINT("sem_fd_draw_engine = 0x%x\r\n",sem_fd_draw_engine);
	}
#endif
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
#if CT_EN == 1
    if(ct_frame_buffer_y_queue == NULL)
	{
        ct_frame_buffer_y_queue = osMessageCreate(&disp_q, NULL);
		if(!ct_frame_buffer_y_queue)
		{
            DBG_PRINT("ct_frame_buffer_y_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ct_frame_buffer_y_queue = 0x%x\r\n", ct_frame_buffer_y_queue);
	}
    if(ct_frame_buffer_u_queue == NULL)
	{
        ct_frame_buffer_u_queue = osMessageCreate(&disp_q, NULL);
		if(!ct_frame_buffer_u_queue)
		{
            DBG_PRINT("ct_frame_buffer_u_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ct_frame_buffer_u_queue = 0x%x\r\n", ct_frame_buffer_u_queue);
	}
    if(ct_frame_buffer_v_queue == NULL)
	{
        ct_frame_buffer_v_queue = osMessageCreate(&disp_q, NULL);
		if(!ct_frame_buffer_v_queue)
		{
            DBG_PRINT("ct_frame_buffer_v_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ct_frame_buffer_v_queue = 0x%x\r\n", ct_frame_buffer_v_queue);
	}
	if(ct_free_frame_buffer_y_queue == NULL)
	{
        ct_free_frame_buffer_y_queue = osMessageCreate(&disp_q, NULL);
		if(!ct_free_frame_buffer_y_queue)
		{
            DBG_PRINT("ct_free_frame_buffer_y_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ct_free_frame_buffer_y_queue = 0x%x\r\n", ct_free_frame_buffer_y_queue);
	}
	if(ct_free_frame_buffer_u_queue == NULL)
	{
        ct_free_frame_buffer_u_queue = osMessageCreate(&disp_q, NULL);
		if(!ct_free_frame_buffer_u_queue)
		{
            DBG_PRINT("ct_free_frame_buffer_u_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ct_free_frame_buffer_u_queue = 0x%x\r\n", ct_free_frame_buffer_u_queue);
	}
	if(ct_free_frame_buffer_v_queue == NULL)
	{
        ct_free_frame_buffer_v_queue = osMessageCreate(&disp_q, NULL);
		if(!ct_free_frame_buffer_v_queue)
		{
            DBG_PRINT("ct_free_frame_buffer_v_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ct_free_frame_buffer_v_queue = 0x%x\r\n", ct_free_frame_buffer_v_queue);
	}
#endif

#if FD_EN == 1
	if(fd_frame_buffer_queue == NULL)
	{
        fd_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!fd_frame_buffer_queue)
		{
            DBG_PRINT("fd_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("fd_frame_buffer_queue = 0x%x\r\n", fd_frame_buffer_queue);
	}
  	if(fd_free_frame_buffer_queue == NULL)
	{
        fd_free_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!fd_free_frame_buffer_queue)
            while(1);
		else
            DBG_PRINT("fd_free_frame_buffer_queue = 0x%x\r\n",fd_free_frame_buffer_queue);
	}
#endif

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
#if CT_EN == 1
    if(ct_state_queue == NULL)
	{
        ct_state_queue = osMessageCreate(&disp_q, NULL);
		if(!ct_state_queue)
		{
            DBG_PRINT("ct_state_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ct_state_queue = 0x%x\r\n", ct_state_queue);
	}
#endif

#if FD_EN == 1
	if(fd_state_queue == NULL)
	{
        fd_state_queue = osMessageCreate(&disp_q, NULL);
		if(!fd_state_queue)
		{
            DBG_PRINT("fd_state_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("fd_state_queue = 0x%x\r\n", fd_state_queue);
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
        osDelay(5);
        //DBG_PRINT("osThreadCreate: disp_id = 0x%x \r\n", disp_id);
    }

#if CT_EN == 1
    ct_id = osThreadCreate(&ct_task, (void *)NULL);
    if(ct_id == 0)
    {
        DBG_PRINT("osThreadCreate: ct_id error\r\n");
        while(1);
    }
    else
    {
        osDelay(5);
        //DBG_PRINT("osThreadCreate: prcess_id = 0x%x \r\n", prcess_id);
    }
#endif

#if FD_EN == 1
    fd_id = osThreadCreate(&fd_task, (void *)NULL);
    if(fd_id == 0) {
        DBG_PRINT("osThreadCreate: fd_id error\r\n");
        while(1);
    }
    else
        osDelay(5);
#endif

#if VR_EN == 1
    vr_id = osThreadCreate(&vr_task, (void *)NULL);
    if(vr_id == 0) {
        DBG_PRINT("osThreadCreate: vr_id error\r\n");
        while(1);
    }
    else
        osDelay(5);
#endif

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



