#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "drv_l1_cdsp.h"
#include "drv_l1_gpio.h"
#include "drv_l1_scaler.h"
#include "drv_l1_pscaler.h"
#include "drv_l1_rotator.h"
#include "drv_l2_scaler.h"
#include "drv_l2_sensor.h"
#include "drv_l2_ad_key_scan.h"
#include "drv_l2_display.h"
#include "gplib_ppu.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "FaceDetectAP.h"
#include "FaceIdentifyAP.h"
#include "gp_track.h"
#include "MouthDetect.h"
#include "NoseDetect.h"
#include "PFMST.h"
#include "KOT.h"
#include "FLandmark.h"
#include "SPRITE_FACE_RECOGNIZE_HDR.h"
#include "drv_l1_dma.h"
#include "avi_encoder_app.h"
//**********Demo Selection Setting**********//
//********************************************
// 1.Face Recognize Demo:
// FACE_RECOGNIZE_MEUN_EN                   1
// FACE_TRACKING_EN                         1
// GESTURE_FIST_TRACKING_EN		            0
// EYE_NOSE_MOUTH_PROC_EN                   0
// FL_EN                                    0
//********************************************
//********************************************
// 2.Face Detection Demo:
// FACE_RECOGNIZE_MEUN_EN                   0
// FACE_TRACKING_EN                         1
// GESTURE_FIST_TRACKING_EN		            0
// EYE_NOSE_MOUTH_PROC_EN                   0
// FL_EN                                    0
///*******************************************
///*******************************************
// 3.Hand/Fist Detection Demo:
// FACE_RECOGNIZE_MEUN_EN                   0
// FACE_TRACKING_EN                         0
// GESTURE_FIST_TRACKING_EN		            1
// EYE_NOSE_MOUTH_PROC_EN                   0
// FL_EN                                    0
///*******************************************
//********************************************
// 4.Face/Eye/Nose/Mouth/Smile Detection Demo:
// FACE_RECOGNIZE_MEUN_EN                   0
// FACE_TRACKING_EN                         1
// GESTURE_FIST_TRACKING_EN		            0
// EYE_NOSE_MOUTH_PROC_EN                   1
// FL_EN                                    0
//********************************************
//********************************************
// 5.FLandmark Demo:
// FACE_RECOGNIZE_MEUN_EN                   0
// FACE_TRACKING_EN                         1
// GESTURE_FIST_TRACKING_EN		            0
// EYE_NOSE_MOUTH_PROC_EN                   0
// FL_EN                                    1
//********************************************
#if PALM_DEMO_EN == 1
#define FACE_RECOGNIZE_MEUN_EN		        0
#define FACE_TRACKING_EN                    0
#define GESTURE_FIST_TRACKING_EN		    1
#define EYE_NOSE_MOUTH_PROC_EN              0
#define FL_EN                               0
#else
#define FACE_RECOGNIZE_MEUN_EN		        1
#define FACE_TRACKING_EN                    1
#define GESTURE_FIST_TRACKING_EN		    0
#define EYE_NOSE_MOUTH_PROC_EN              0
#define FL_EN                               0
#endif
// demo setting
#define FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN 1
#if FACE_TRACKING_EN == 1
#define PFMST_EN                            0
#define KOT_EN                              1
#elif GESTURE_FIST_TRACKING_EN == 1
#if PALM_DEMO_EN == 1
#define PFMST_EN                            0
#else
#define PFMST_EN                            1
#endif
#define KOT_EN                              0
#else
#define PFMST_EN                            0
#define KOT_EN                              0
#endif
#define CSI_PSCALER_STOP_EN                 1
#define TWO_HAND_MODE                       0
#define MULTIOLE_TRACKING_EN                0
#define SCALER_LINEAR_EN                    0
#define CSI_AE_EN                           0
#define FD_SD_EN                            0
#define PPU_DRAW_PRCESS_DEBUG_EN            0
#define FD_NEW_PROC_EN                      0
#define DISP_QUEUE_MAX		                3
#define C_PPU_DRV_FRAME_NUM		            DISP_QUEUE_MAX
#define FD_PROC_H_SIZE                      320
#define FD_PROC_V_SIZE                      240
#define PPU_TEXT_SIZE_HPIXEL                FD_PROC_H_SIZE
#define PPU_TEXT_SIZE_VPIXEL                FD_PROC_V_SIZE
#define FD_SIZE_HPIXEL                      FD_PROC_H_SIZE
#define FD_SIZE_VPIXEL                      FD_PROC_V_SIZE
#define FD_DEBUG_EN			                0
#if FACE_RECOGNIZE_MEUN_EN == 1
#define OBJDETECT_MAX_RESULT		        4096
#else
#define OBJDETECT_MAX_RESULT		        1024
#endif
#define OBJDETECT_POSITION_RESULT           2
#define OBJDETECT_POSITION_NO_FIND_CNT      5
#define RETURN(x)	                        {nRet = x; goto Return;}
#define DUMMY_BUFFER_ADDRESS                0x50000000
#if _SENSOR_H42_CDSP_MIPI == 1 || _SENSOR_H62_CDSP_MIPI == 1
#define SENSOR_SRC_WIDTH		            1280
#define SENSOR_SRC_HEIGHT		            720
#else
#define SENSOR_SRC_WIDTH		            640
#define SENSOR_SRC_HEIGHT		            480
#endif
#define PRCESS_SRC_WIDTH		            640
#define PRCESS_SRC_HEIGHT		            480
#define FD_STATE_OK                         0x80
#define MSG_PRCESS_TASK_EXIT                0xFF
#if PALM_DEMO_EN == 0
#define GPIO_EN_PIN1                        IO_D6
#else
#define GPIO_EN_PIN1                        IO_E0
#endif
#define C_AE_VALUE		 		            0x48
#define ACK_OK			                    0
#define ACK_FAIL		                    (-1)
#define Q_FLT                               6
/*
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
*/
// eye detection
//#define PI		                            3.1415926535897932384626433832795
#define RESIZE_FACE_IMG_WIDTH	            96/*128*/
#define RESIZE_FACE_IMG_HEIGHT	            96/*128*/
#define HAAR_X_MARGIN		                7
#define HAAR_Y_MARGIN		                7
#define EYES_WIDTH			                36	// RESIZE_IMG_WIDTH - RESIZE_IMG_WIDTH * 0.13 * 2
#define EYES_HEIGHT			                36	// RESIZE_IMG_HEIGHT - RESIZE_IMG_WIDTH * 0.4 - 2 * EYES_WIDTH * 0.15

// face recognize
#define INT_MAX				                2147483647
#define FACE_ROI			                0
#define LEYE_ROI			                1
#define REYE_ROI			                2
#define BESTFACE_ROI		                3
#define FD_TRAINING_IMAGE_NUMBER		    10
#define ABS(x)		                        ((x) >= 0 ? (x) : -(x))

// face landmark
#if FL_EN == 1

//#include "FLandmark.h"
static gpPoint flandmarks_qvga_draw[floutCount << 1];
static gpPoint glasses_qvga_draw[glassesCount << 1];
static int fl_state_draw = 0;
FLandmark_Input flandmark_in;
static int flandmarkResult[2];
static gpRect fl_rect_result[2]; // face detection result for face landmark
static xSemaphoreHandle sem_fl_engine = NULL;

#endif

// fist / gesture detection
#define OBJDETECT_MIN_RT_NBR                4
#define OBJDETECT_MIN_LT_NBR                5
#define OBJDETECT_MIN_fist_NBR              5
#define OBJDETECT_MIN_NBR			        4

// multiple track use
#define MULTIOLE_HAND1                       0
#define MULTIOLE_HAND2                       1
#define MULTIOLE_FIST                        2

#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define FRAME_BUF_ALIGN16                   0xF
#define FRAME_BUF_ALIGN8                    0x7
#define FRAME_BUF_ALIGN4                    0x3

#define DRAW_RED_EN                         0
#define DRAW_GREEN_EN                       1
#define DRAW_BLUE_EN                        2
#define DRAW_YELLOW_EN                      3
#define DRAW_BLACK_EN                       4

#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
#define DISPLAY_USE_PSCALER_EN              1
#else
#define DISPLAY_USE_PSCALER_EN              0
#endif
#define POST_SCALE_USE                      PSCALER_B
#if _SENSOR_GC1064_CDSP_MIPI == 1 && AUO_A027DTN019 == 1
#define ROTATOR_FLIP_EN                     1
#else
#define ROTATOR_FLIP_EN                     0
#endif

#if FACE_RECOGNIZE_MEUN_EN == 1
#define FD_TRAINING_STATE_GET		        1
#define FD_IDENTIFY_STATE_GET		        2
#define FD_SECURITY_LEVEL_GET               3
#define FD_TRAINING_CNT_GET			        4
#define FD_IDENTIFY_SCORE_GET		        5
#define FD_ULPB_UPDATE_STATE_GET	        6
#define FD_ULPB_UPDATE_CNT_GET		        7
// mode
#define TRAINING_MODE				        1
#define IDENTIFY_MODE				        2
#define ULBP_UPDATE_MODE			        3
#define ULPB_UPDATE_CNT_DOWN		        0xFE
#define ULPB_UPDATE_CNT_UP			        0xFF
// PPU SP NUMBER
#define C_JPG_SAVE_NUMBER_1                 0
#define C_JPG_SAVE_NUMBER_2                 1
#define C_JPG_SAVE_NUMBER_3                 2
#define C_JPG_SAVE_ICON                     3
#define C_FACE_DEMO_STATE                   4
#define C_FACE_IDENTIFY_END                 5
#define C_FACE_IDENTIFY_LEVEL               6
#define C_FACE_IDENTIFY_ICON1               7
#define C_FACE_IDENTIFY_ICON2               8
#define C_FACE_CHECK_ICON1			        (C_FACE_IDENTIFY_ICON2 + 1)
#define C_FACE_CHECK_ICON2			        (C_FACE_CHECK_ICON1 + 1)
#define C_FACE_PASSWORD_ICON1		        (C_FACE_CHECK_ICON2 + 1)
#define C_FACE_PASSWORD_ICON2		        (C_FACE_PASSWORD_ICON1 + 1)
#define C_FACE_PASSWORD_NUM1		        (C_FACE_PASSWORD_ICON2 + 1)
#define C_FACE_PASSWORD_NUM2                (C_FACE_PASSWORD_NUM1 + 1)
#define C_FACE_PASSWORD_NUM3                (C_FACE_PASSWORD_NUM2 + 1)
#define C_FACE_PASSWORD_NUM4                (C_FACE_PASSWORD_NUM3 + 1)
#define C_FACE_TRAIN_SUBJECT                (C_FACE_PASSWORD_NUM4 + 1)
#define C_FACE_IDENTIFY_RESULT              (C_FACE_TRAIN_SUBJECT + 1)

// mode
#define FACE_MODE_GET                       0
#define FACE_TRAINING_RESULT_GET            FD_TRAINING_STATE_GET
#define FACE_IDENTIFY_RESULT_GET            FD_IDENTIFY_STATE_GET
#define FACE_SECURITY_LEVEL_GET             FD_SECURITY_LEVEL_GET
#define FACE_TRAINING_COUNTER_GET           FD_TRAINING_CNT_GET

#define FACE_TRAINING_MODE                  1
#define FACE_IDENTIFY_MODE                  2
#define FACE_ULBP_UPDATE_MODE               3

static const INT32U* FACE_SP_NUMBER_POOL[]={
 	(INT32U *)_Sprite0001_IMG0000_CellIdx,
 	(INT32U *)_Sprite0001_IMG0001_CellIdx,
 	(INT32U *)_Sprite0001_IMG0002_CellIdx,
 	(INT32U *)_Sprite0001_IMG0003_CellIdx,
 	(INT32U *)_Sprite0001_IMG0004_CellIdx,
 	(INT32U *)_Sprite0001_IMG0005_CellIdx,
 	(INT32U *)_Sprite0001_IMG0006_CellIdx,
 	(INT32U *)_Sprite0001_IMG0007_CellIdx,
 	(INT32U *)_Sprite0001_IMG0008_CellIdx,
 	(INT32U *)_Sprite0001_IMG0009_CellIdx
};
#endif

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

typedef struct {
	void *identify_WorkMem;
	void *eye_detect_WorkMem;
	void *ownerULBP;
	gpImage	image;
	INT8U id_mode;
	INT8U training_cnt;
	INT8U training_subject_cnt;
	INT8U identify_cnt;
	INT8U security_level;
	INT32U training_state;
	INT32U identify_state;
	INT32U security_value;
	INT32U identify_save_en;
	INT32U identify_save_number;
	INT32U identify_save_max_number;
	INT32S recognize_score;
	INT32S (*pFuncStore)(INT32U buf, INT32U size);
} FaceIdentify_t;

typedef struct {
    void *pedestrian_detect_WorkMem[OBJDETECT_POSITION_RESULT];
	void *face_detect_WorkMem[OBJDETECT_POSITION_RESULT];
	void *hand_detect_WorkMem[OBJDETECT_POSITION_RESULT];
	gpRect PedestrianPositionTrackingSet[OBJDETECT_POSITION_RESULT];
	gpRect FacePositionTrackingSet[OBJDETECT_POSITION_RESULT];
	gpRect HandPositionTrackingSet[OBJDETECT_POSITION_RESULT];
    INT32U find_face_update[OBJDETECT_POSITION_RESULT];
    INT32U find_hand_update[OBJDETECT_POSITION_RESULT];
	INT32U find_fist_update[OBJDETECT_POSITION_RESULT];
    INT32U find_face[OBJDETECT_POSITION_RESULT];
    INT32U find_hand[OBJDETECT_POSITION_RESULT];
	INT32U find_fist[OBJDETECT_POSITION_RESULT];
    INT32U FD_find_face_num;
    INT32U FD_find_hand_num;
	INT32U FD_find_fist_num;
    INT32U FD_no_find_face_cnt;
    INT32U FD_no_find_hand_cnt;
	INT32U FD_no_find_fist_cnt;
    INT32U PFMST_find_face_num;
    INT32U PFMST_find_hand_num;
	INT32U PFMST_find_fist_num;
} PFMSTParam_t;

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
	gpPoint left_eye;
	gpPoint right_eye;
	gpRect left_eyebrow;
	gpRect right_eyebrow;
	gpRect face;

	double eye_angle;
} FaceEyeResult_t;

typedef struct {
	void *eye_detect_WorkMem;
	gpImage	face_img;
	gpImage	eq_face_img;

	ClassifyData clf;
	INT32S xstep;
	INT32S scale;
	PostProc_t post_proc;

	FaceEyeResult_t fe_result;
	INT8U is_get_eye;
	INT8U no_eye_cnt;
	INT8U reserved0;
	INT8U reserved1;

	gpRect eye_rect[OBJDETECT_MAX_RESULT];
	INT32S eye_cnt[OBJDETECT_MAX_RESULT];
} EyeDetect_t;

typedef struct {
	void *mouth_detect_WorkMem;
	gpImage	upright_face_img;
	gpFeaturePoint mouth_pt;
	gpFeaturePoint nose_pt;
	INT8U is_get_nose;
	INT8U is_get_mouth;
	INT8U reserved0;
	INT8U reserved1;
} MouthNoseDetect_t;

typedef struct {
	INT32S *rfms_WorkMem;
	CHAR   *obj_data_buf;
	gpImage	image;
	INT32S cardsN;
	INT32U matchingThre;
	INT32U matchingThreRangeV;
	INT32U minExtractionThre;
	INT32U incExtractionThre;
	INT32U decExtractionThre;
	INT32U startMatchingPointN;
	INT32U matchingCardNum;
} RfmsIdentify_t;

typedef struct {
	INT32S l[3];
	INT32S level[4];
	INT32S cnt[3];
} LumStr;

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
} PSCALER_PARAM_STRUCT;

typedef struct {
    ObjResult_t track_result[DISP_QUEUE_MAX];
    ObjDetect_t trackWorkMem[DISP_QUEUE_MAX];
} TrackResult_t;

typedef struct {
	INT32U ppu_frame_workmem;
	INT32U ppu_narray_workmem;
	INT32U ppu_pscaler_workmem;
	INT32U ppu_draw_src_workmem;
    INT32U csi_prcess_workmem;
    INT32U disp_prcess_workmem;
    INT32U fd_prcess_workmem;
    INT32U fd_y_workmem;
    INT32U pfmst_prcess_workmem;
    INT32U kot_prcess_workmem;
    INT32U kot_roi_workmem;
    INT32U jpg_encode_workmem;
#if 0//CSI_FULL_IMAGE_FLOW_EN == 1
    INT32U csi_pscaler_clip_workmem;
#endif
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
    INT32U fiMem_workmem;
    INT32U userDB_workmem;
    INT32U update_num_workmem;
    INT32U update_index_workmem;
    INT32U face_crop_workmem;
#endif
#if FL_EN == 1
    INT32U face_qqvga_workmem;
#endif
} prcess_mem_t;

static osThreadId fd_id;
static osThreadId csi_id;
static osThreadId disp_id;
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static prcess_mem_t prcess_mem_structure;
static prcess_mem_t *prcess_mem_set;
static INT32U PPU_FRAME_BUFFER_BASE,fd_org_buf,text1_narray,fd_workmem_size,fd_workmem = 0;
static INT32U scaler_int_w_step,fd_type,disp_buffer_post,hand_num_init,TrackWorkMem_num;
static INT32U hand_workmem_size,hand_workmem = 0, csi_init_en = 0, csi_pscaler_stop;
#if TWO_HAND_MODE == 1
static INT32U hand_num;
#endif
#if MULTIOLE_TRACKING_EN == 1
static TrackResult_t TrackWorkMem;
static TrackResult_t *TrackWorkMem_ptr = NULL;
#endif
#if FACE_RECOGNIZE_MEUN_EN == 1
static INT32U fd_mode;
static INT32S security_level;
static FaceIdentify_t *fdWorkMem = NULL;
#endif
#if EYE_NOSE_MOUTH_PROC_EN == 1
static EyeDetect_t *eyeWorkMem = NULL;
static MouthNoseDetect_t *mouthWorkMem = NULL;
#endif
static ObjResult_t obj_result = {0};
static ObjResult_t obj_draw_result = {0};
static ObjDetect_t ObjWorkMem;
static ObjDetect_t *ObjWorkMem_ptr = NULL;
static xQueueHandle free_frame_buffer_queue = NULL;
#if PALM_DEMO_EN == 0
static xQueueHandle display_frame_buffer_queue = NULL;
#else
xQueueHandle display_frame_buffer_queue2 = NULL;
xQueueHandle fd_dma_frame_buffer_queue = NULL;
#endif
static xQueueHandle fd_frame_buffer_queue = NULL;
static xQueueHandle fd_free_frame_buffer_queue = NULL;
static xQueueHandle ppu_frame_buffer_queue = NULL;
static xQueueHandle fd_state_queue = NULL;
static xQueueHandle pscaler_free_buffer_queue = NULL;
static xQueueHandle pscaler_display_buffer_queue = NULL;
static xQueueHandle ppu_draw_free_buffer_queue = NULL;
static xQueueHandle fd_task_ack_m = NULL;
static xQueueHandle csi_task_ack_m = NULL;
static xQueueHandle disp_task_ack_m = NULL;
static xSemaphoreHandle sem_scaler_engine = NULL;
static xSemaphoreHandle sem_disp_engine = NULL;
static xSemaphoreHandle sem_fd_engine = NULL;
static xSemaphoreHandle sem_fd_draw_engine = NULL;
static PSCALER_PARAM_STRUCT PScalerParam = {0};
#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
INT32U disp_h_size, disp_v_size;
#if DISPLAY_USE_PSCALER_EN == 1
static INT32U pscaler_buf,pscaler_disp_buffer_post,pscaler_init_set = 0;
#else
static INT32U scaler_buf1,scaler_buf2,scaler_buf_flag;
#endif
#endif
static PFMSTParam_t objparam = {0};

#if _SENSOR_GC0308_CSI == 1
extern INT8U GC0308_ReadY(void);
extern INT8U GC0308_SetY(INT8U Value);
#endif

static INT32S pscaler_free_buffer_add(INT32U *frame_buf);
static INT32S pscaler_free_buffer_get(void);
static INT32S pscaler_display_buffer_add(INT32U *frame_buf);
static INT32S pscaler_display_frame_buffer_get(void);

#if PFMST_EN == 1
// partical filter mean-shift tracker
static gpRect trackingSet[10];
static gpRect trackingSet_result[10];
static gpRect trackingSet1_result[10];
static INT32U pfmst_workmem = 0;
static TPFMSContext context; ///< Context of PFMS tracking
static TPFMSContext context_result; ///< Context of PFMS tracking
static PFMSTWorkMem PFMSTMem;
static TPFMSContext context1; ///< Context of PFMS tracking
static TPFMSContext context1_result; ///< Context of PFMS tracking
static PFMSTWorkMem PFMSTMem1;
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

#if KOT_EN == 1
//KOT
static TPFMSContext context; ///< Context of PFMS tracking
static INT32U KOT_workmem = 0;
globalData_KOT* gData_ptr;
static INT32U compare_out_buffer = 0;
#endif

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
static FID_Info fidInfo;
static INT32U train_subject;
#endif

extern unsigned char gamma_30[];
extern unsigned char gamma_34[];
extern unsigned char gamma_44[];
extern unsigned char gamma_58[];

static void face_recognize_free(void *id_work_mem);

#if CSI_AE_EN == 1
void sensor_control_ae(void)
{
#if _SENSOR_GC0308_CSI == 1
	INT32S YValue, temp;

	YValue = GC0308_ReadY();
	temp = ABS(YValue - C_AE_VALUE);
	if(temp >= 10) {
		temp = temp/10;
		if(YValue > C_AE_VALUE) {
			YValue -= 16 * temp;
		} else {
			YValue += 16 * temp;
		}

		GC0308_SetY(YValue);
	}
#endif
}
#endif

void gpio1_set(int mode)
{
    gpio_write_io(GPIO_EN_PIN1, mode);
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
        if(prcess_mem_set->fd_prcess_workmem)
        {
            gp_free((void *)prcess_mem_set->fd_prcess_workmem);
            prcess_mem_set->fd_prcess_workmem = 0;
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
        if(prcess_mem_set->fd_y_workmem)
        {
            gp_free((void *)prcess_mem_set->fd_y_workmem);
            prcess_mem_set->fd_y_workmem = 0;
        }
#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
        if(prcess_mem_set->ppu_pscaler_workmem)
        {
            gp_free((void *)prcess_mem_set->ppu_pscaler_workmem);
            prcess_mem_set->ppu_pscaler_workmem = 0;
        }
#endif

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
        if(prcess_mem_set->fiMem_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->fiMem_workmem = 0x%x\r\n",prcess_mem_set->fiMem_workmem);
            #endif
            gp_free((void *)prcess_mem_set->fiMem_workmem);
            prcess_mem_set->fiMem_workmem = 0;
        }
        if(prcess_mem_set->userDB_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->userDB_workmem = 0x%x\r\n",prcess_mem_set->userDB_workmem);
            #endif
            gp_free((void *)prcess_mem_set->userDB_workmem);
            prcess_mem_set->userDB_workmem = 0;
        }
        if(prcess_mem_set->update_num_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->update_num_workmem = 0x%x\r\n",prcess_mem_set->update_num_workmem);
            #endif
            gp_free((void *)prcess_mem_set->update_num_workmem);
            prcess_mem_set->update_num_workmem = 0;
        }
        if(prcess_mem_set->update_index_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->update_index_workmem = 0x%x\r\n",prcess_mem_set->update_index_workmem);
            #endif
            gp_free((void *)prcess_mem_set->update_index_workmem);
            prcess_mem_set->update_index_workmem = 0;
        }
        if(prcess_mem_set->face_crop_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->face_crop_workmem = 0x%x\r\n",prcess_mem_set->face_crop_workmem);
            #endif
            gp_free((void *)prcess_mem_set->face_crop_workmem);
            prcess_mem_set->face_crop_workmem = 0;
        }
#endif

#if FL_EN == 1
        if(prcess_mem_set->face_qqvga_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->face_qqvga_workmem = 0x%x\r\n",prcess_mem_set->face_qqvga_workmem);
            #endif
            gp_free((void *)prcess_mem_set->face_qqvga_workmem);
            prcess_mem_set->face_qqvga_workmem = 0;

        }
#endif
        prcess_mem_set = NULL;
    }
#if FACE_RECOGNIZE_MEUN_EN == 1
    // FaceIdentify_t
    if(fdWorkMem)
    {
        face_recognize_free((void *)fdWorkMem);
        fdWorkMem = NULL;
    }
#endif

#if EYE_NOSE_MOUTH_PROC_EN == 1
    // EyeDetect_t
    if(eyeWorkMem)
    {
        if(eyeWorkMem->eye_detect_WorkMem)
        {
            gp_free((void *)eyeWorkMem->eye_detect_WorkMem);
            eyeWorkMem->eye_detect_WorkMem = 0;
        }
        gp_free((void *)eyeWorkMem);
        eyeWorkMem = NULL;
    }
#endif

#if EYE_NOSE_MOUTH_PROC_EN == 1
    // MouthNoseDetect_t
    if(mouthWorkMem)
    {
        if(mouthWorkMem->mouth_detect_WorkMem)
        {
            gp_free((void *)mouthWorkMem->mouth_detect_WorkMem);
            mouthWorkMem->mouth_detect_WorkMem = 0;
        }
        gp_free((void *)mouthWorkMem);
        mouthWorkMem = NULL;
    }
#endif

#if MULTIOLE_TRACKING_EN == 1
    if(TrackWorkMem_ptr)
    {
        for(i=0;i<DISP_QUEUE_MAX;i++)
        {
            if(TrackWorkMem_ptr->trackWorkMem[i].obj_track_WorkMem)
            {
                gp_free((void *)TrackWorkMem_ptr->trackWorkMem[i].obj_track_WorkMem);
                TrackWorkMem_ptr->trackWorkMem[i].obj_track_WorkMem = 0;
            }

            if(TrackWorkMem_ptr->trackWorkMem[i].obj_detect_WorkMem)
            {
                gp_free((void *)TrackWorkMem_ptr->trackWorkMem[i].obj_detect_WorkMem);
                TrackWorkMem_ptr->trackWorkMem[i].obj_detect_WorkMem = 0;
            }
        }
        TrackWorkMem_ptr = NULL;
    }
#endif

#if FL_EN == 1
    if(flandmark_in.fl_stable_buf)
    {
        gp_free((void *)flandmark_in.fl_stable_buf);
        flandmark_in.fl_stable_buf = 0;
    }

    if(flandmark_in.flout_stable_buf)
    {
        gp_free((void *)flandmark_in.flout_stable_buf);
        flandmark_in.flout_stable_buf = 0;
    }

    if(flandmark_in.g_stable_buf)
    {
        gp_free((void *)flandmark_in.g_stable_buf);
        flandmark_in.g_stable_buf = 0;
    }

    if(flandmark_in.flandmark_mem)
    {
        gp_free((void *)flandmark_in.flandmark_mem);
        flandmark_in.flandmark_mem = 0;
    }
#endif

    if(ObjWorkMem_ptr)
    {
        if(ObjWorkMem_ptr->obj_track_WorkMem)
        {
            gp_free((void *)ObjWorkMem_ptr->obj_track_WorkMem);
            ObjWorkMem_ptr->obj_track_WorkMem = 0;
        }

        if(ObjWorkMem_ptr->obj_detect_WorkMem)
        {
            gp_free((void *)ObjWorkMem_ptr->obj_detect_WorkMem);
            ObjWorkMem_ptr->obj_detect_WorkMem = 0;
        }

        ObjWorkMem_ptr = NULL;
    }

    if(fd_workmem)
    {
        gp_free((void *)fd_workmem);
        fd_workmem = 0;
    }

    return 0;
}

static INT32S fd_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    // fd task
    POST_MESSAGE(fd_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, fd_task_ack_m, 5000);
Return:
    OSQFlush(fd_frame_buffer_queue);
    OSQFlush(fd_free_frame_buffer_queue);
    OSQFlush(fd_task_ack_m);
    OSQFlush(pscaler_free_buffer_queue);
    OSQFlush(pscaler_display_buffer_queue);
    OSQFlush(fd_state_queue);

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
#if PALM_DEMO_EN == 0
    POST_MESSAGE(display_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, csi_task_ack_m, 5000);
#else
    POST_MESSAGE(display_frame_buffer_queue2, MSG_PRCESS_TASK_EXIT, csi_task_ack_m, 5000);
#endif
Return:
#if PALM_DEMO_EN == 0
    OSQFlush(display_frame_buffer_queue);
#else
    OSQFlush(display_frame_buffer_queue2);
#endif
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

    nRet = fd_task_exit();
    if(nRet != STATUS_OK)
        DBG_PRINT("fd_task_exit fail\r\n");

    prcess_mem_free();

    return nRet;
}

static INT32S object_position_calculate(INT32U obj_module, INT32U pos_num, gpRect *Result)
{
    PFMSTParam_t *param_ptr = (PFMSTParam_t *)&objparam;
    gpRect *Result_ptr;
    gpRect *org_Result_ptr = (gpRect *)Result;
    INT32U i,j,num,num_ck,find_state;
    INT32U org_x_offset,org_y_offset,x_center,y_center;
    INT32U x_offset,y_offset;
    INT32U *pos_update_flag,*pos_find_flag;
    INT32S ret = -1;

    if(pos_num == 0)
    {
        if(obj_module == GP_HAND)
            gp_memset((INT8U *)&param_ptr->find_hand_update[0], 0, sizeof(param_ptr->find_hand_update));
        else if(obj_module == GP_FACE)
            gp_memset((INT8U *)&param_ptr->find_face_update[0], 0, sizeof(param_ptr->find_face_update));
        return 0;
    }


    if(pos_num == 0xFFFFFFFF)
    {
        if(obj_module == GP_HAND)
        {
            gp_memset((INT8U *)&param_ptr->HandPositionTrackingSet, 0, sizeof(param_ptr->HandPositionTrackingSet));
            gp_memset((INT8U *)&param_ptr->find_hand_update, 0, sizeof(param_ptr->find_hand_update));
            gp_memset((INT8U *)&param_ptr->find_hand, 0, sizeof(param_ptr->find_hand));
            param_ptr->FD_find_hand_num = 0;
            param_ptr->FD_no_find_hand_cnt = 0;
            DBG_PRINT("0");
            return 0;
        }
        else if(obj_module == GP_FACE)
        {
            gp_memset((INT8U *)&param_ptr->FacePositionTrackingSet, 0, sizeof(param_ptr->FacePositionTrackingSet));
            gp_memset((INT8U *)&param_ptr->find_face_update, 0, sizeof(param_ptr->find_face_update));
            gp_memset((INT8U *)&param_ptr->find_face, 0, sizeof(param_ptr->find_face));
            param_ptr->FD_find_face_num = 0;
            param_ptr->FD_no_find_face_cnt = 0;
            return 0;
        }
        else
            return -1;
    }

    if(obj_module == GP_HAND)
        gp_memset((INT8U *)&param_ptr->find_hand_update[0], 0, sizeof(param_ptr->find_hand_update));
    else if(obj_module == GP_FACE)
        gp_memset((INT8U *)&param_ptr->find_face_update[0], 0, sizeof(param_ptr->find_face_update));

    if(pos_num > OBJDETECT_POSITION_RESULT)
        num = OBJDETECT_POSITION_RESULT;
    else
        num = pos_num;

    if(obj_module == GP_HAND)
    {
#if 1
        if(param_ptr->FD_find_hand_num == 2)
        {
            num_ck = param_ptr->FD_find_hand_num;
        }
        else
        {
            num_ck = pos_num;
            param_ptr->FD_find_hand_num = pos_num;
        }
#else
        if((param_ptr->FD_find_hand_num == 2) && (pos_num == 1))
        {
            gp_memset((INT8U *)&param_ptr->HandPositionTrackingSet, 0, sizeof(param_ptr->HandPositionTrackingSet));
            gp_memset((INT8U *)&param_ptr->find_hand, 0, sizeof(param_ptr->find_hand));
            num_ck = pos_num;
            param_ptr->FD_find_hand_num = pos_num;
        }
#if 0
        else if((param_ptr->FD_find_hand_num == 1) && (pos_num == 2))
        {
            num_ck = pos_num;
            param_ptr->FD_find_hand_num = pos_num;
        }
#endif
        else
        {
            num_ck = pos_num;
            param_ptr->FD_find_hand_num = pos_num;
        }
        param_ptr->FD_no_find_hand_cnt = 0;
#endif
    }
    else if(obj_module == GP_FACE)
    {
        if((param_ptr->FD_find_face_num == 2) && (pos_num == 1))
        {
            gp_memset((INT8U *)&param_ptr->FacePositionTrackingSet, 0, sizeof(param_ptr->FacePositionTrackingSet));
            gp_memset((INT8U *)&param_ptr->find_face, 0, sizeof(param_ptr->find_face));
            num_ck = pos_num;
            param_ptr->FD_find_face_num = pos_num;
        }
#if 0
        else if((param_ptr->FD_find_face_num == 1) && (pos_num == 2))
        {
            num_ck = pos_num;
            param_ptr->FD_find_face_num = pos_num;
        }
#endif
        else
        {
            num_ck = pos_num;
            param_ptr->FD_find_face_num = pos_num;
        }
        param_ptr->FD_no_find_face_cnt = 0;
    }

    // CASE 0
    org_Result_ptr = (gpRect *)Result;
    if(obj_module == GP_HAND)
    {
        Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[0];
        pos_find_flag = (INT32U *)&param_ptr->find_hand[0];
    }
    else if(obj_module == GP_FACE)
    {
        Result_ptr = (gpRect *)&param_ptr->FacePositionTrackingSet[0];
        pos_find_flag = (INT32U *)&param_ptr->find_face[0];
    }
    else
        return -1;
    for(i=0;i<num;i++)
    {
        if(Result_ptr->x == 0)
        {
            Result_ptr->x = org_Result_ptr->x;
            Result_ptr->y = org_Result_ptr->y;
            Result_ptr->width = org_Result_ptr->width;
            Result_ptr->height = org_Result_ptr->height;
            if(*pos_find_flag == 0)
                *pos_find_flag = (i+1);
        }
        Result_ptr++;
        org_Result_ptr++;
        pos_find_flag++;
    }

    // CASE 1
    org_Result_ptr = (gpRect *)Result;
    for(i=0;i<num;i++)
    {
        org_x_offset = org_Result_ptr->width/2;
        org_y_offset = org_Result_ptr->height/2;
        if(obj_module == GP_HAND)
        {
            Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[0];
            pos_update_flag = (INT32U *)&param_ptr->find_hand_update[0];
        }
        else if(obj_module == GP_FACE)
        {
            Result_ptr = (gpRect *)&param_ptr->FacePositionTrackingSet[0];
            pos_update_flag = (INT32U *)&param_ptr->find_face_update[0];
        }
        else
            return -1;
        for(j=0;j<num_ck;j++)
        {
            if(*pos_update_flag == 0)
            {
                if(Result_ptr->x != 0)
                {
                    x_center = Result_ptr->x;
                    y_center = Result_ptr->y;
                    // Position in safe range.
                    if((x_center < (org_Result_ptr->x + org_x_offset)) && (x_center > (org_Result_ptr->x - org_x_offset))
                       && (y_center < (org_Result_ptr->y + org_y_offset)) && (y_center > (org_Result_ptr->y - org_y_offset)))
                    {
                        Result_ptr->x = org_Result_ptr->x;
                        Result_ptr->y = org_Result_ptr->y;
                        Result_ptr->width = org_Result_ptr->width;
                        Result_ptr->height = org_Result_ptr->height;
                        *pos_update_flag = 1;
                        //if(param_ptr->find_hand_update[1])
                            //DBG_PRINT(".");
                        //else if(param_ptr->find_hand_update[0])
                            //DBG_PRINT("!");
                    }
                }
            }
            Result_ptr++;
            pos_update_flag++;
        }
        org_Result_ptr++;
    }
#if 0
    if(num >= 1)
    {
        if(param_ptr->find_face_update[0] == 0)
        {
            org_Result_ptr = (gpRect *)Result;
            pos_update_flag = (INT32U *)&param_ptr->find_face_update[0];
            if(param_ptr->find_face_update[1])
            {
                Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[1];
                for(i=0;i<num;i++)
                {
                    if((Result_ptr->x != org_Result_ptr->x) && (org_Result_ptr->x != 0))
                    {
                        Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[0];
                        // New Position Set
                        Result_ptr->x = org_Result_ptr->x;
                        Result_ptr->y = org_Result_ptr->y;
                        Result_ptr->width = org_Result_ptr->width;
                        Result_ptr->height = org_Result_ptr->height;
                        *pos_update_flag = 1;
                    }
                    org_Result_ptr++;
                }
            }
            else
            {
                Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[0];
                if(Result_ptr->x > org_Result_ptr->x)
                    x_offset = (Result_ptr->x - org_Result_ptr->x);
                else
                    x_offset = (org_Result_ptr->x - Result_ptr->x);
                org_Result_ptr++;
                if(Result_ptr->x > org_Result_ptr->x)
                    org_x_offset = (Result_ptr->x - org_Result_ptr->x);
                else
                    org_x_offset = (org_Result_ptr->x - Result_ptr->x);

                if(org_x_offset > x_offset)
                {
                    org_Result_ptr = (gpRect *)Result;
                    // New Position Set
                    Result_ptr->x = org_Result_ptr->x;
                    Result_ptr->y = org_Result_ptr->y;
                    Result_ptr->width = org_Result_ptr->width;
                    Result_ptr->height = org_Result_ptr->height;
                    *pos_update_flag = 1;
                }
                else
                {
                    // New Position Set
                    Result_ptr->x = org_Result_ptr->x;
                    Result_ptr->y = org_Result_ptr->y;
                    Result_ptr->width = org_Result_ptr->width;
                    Result_ptr->height = org_Result_ptr->height;
                    *pos_update_flag = 1;
                }
            }
        }

        if(param_ptr->find_face_update[1] == 0)
        {
            org_Result_ptr = (gpRect *)Result;
            pos_update_flag = (INT32U *)&param_ptr->find_face_update[1];
            if(param_ptr->find_face_update[0])
            {
                Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[0];
                for(i=0;i<num;i++)
                {
                    if((Result_ptr->x != org_Result_ptr->x) && (org_Result_ptr->x != 0))
                    {
                        Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[1];
                        // New Position Set
                        Result_ptr->x = org_Result_ptr->x;
                        Result_ptr->y = org_Result_ptr->y;
                        Result_ptr->width = org_Result_ptr->width;
                        Result_ptr->height = org_Result_ptr->height;
                    }
                    org_Result_ptr++;
                }
            }
            else
            {
                Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[1];
                if(Result_ptr->x > org_Result_ptr->x)
                    x_offset = (Result_ptr->x - org_Result_ptr->x);
                else
                    x_offset = (org_Result_ptr->x - Result_ptr->x);
                org_Result_ptr++;
                if(Result_ptr->x > org_Result_ptr->x)
                    org_x_offset = (Result_ptr->x - org_Result_ptr->x);
                else
                    org_x_offset = (org_Result_ptr->x - Result_ptr->x);

                if(org_x_offset > x_offset)
                {
                    org_Result_ptr = (gpRect *)Result;
                    // New Position Set
                    Result_ptr->x = org_Result_ptr->x;
                    Result_ptr->y = org_Result_ptr->y;
                    Result_ptr->width = org_Result_ptr->width;
                    Result_ptr->height = org_Result_ptr->height;
                    *pos_update_flag = 1;
                }
                else
                {
                    // New Position Set
                    Result_ptr->x = org_Result_ptr->x;
                    Result_ptr->y = org_Result_ptr->y;
                    Result_ptr->width = org_Result_ptr->width;
                    Result_ptr->height = org_Result_ptr->height;
                    *pos_update_flag = 1;
                }
            }
        }
    }
#endif
    // CASE 2
#if 0
    if(num > 1)
    {
        if(obj_module == GP_HAND)
        {
            Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[0];
            org_Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[1];
            pos_update_flag = (INT32U *)&param_ptr->find_hand_update[0];
        }
        else if(obj_module == GP_FACE)
        {
            Result_ptr = (gpRect *)&param_ptr->FacePositionTrackingSet[0];
            org_Result_ptr = (gpRect *)&param_ptr->FacePositionTrackingSet[1];
            pos_update_flag = (INT32U *)&param_ptr->find_face_update[0];
        }
        else
            return -1;
        if(Result_ptr->width > org_Result_ptr->width)
            org_x_offset = Result_ptr->width;
        else
            org_x_offset = org_Result_ptr->width;
        if(Result_ptr->x > org_Result_ptr->x)
            x_center = (Result_ptr->x + Result_ptr->width) - (org_Result_ptr->x + org_Result_ptr->width);
        else
            x_center = (org_Result_ptr->x + org_Result_ptr->width) - (Result_ptr->x + Result_ptr->width);
        //if(org_x_offset < x_center)
    }
#endif

#if 0
    // sequence position
    if(num > 1)
    {
        org_Result_ptr = (gpRect *)Result;
        Result_ptr = (gpRect *)&param_ptr->HandPositionTrackingSet[0];
        for(i=0;i<num;i++)
        {
            org_Result_ptr->x = Result_ptr->x;
            org_Result_ptr->y = Result_ptr->y;
            org_Result_ptr->width = Result_ptr->width;
            org_Result_ptr->height = Result_ptr->height;
            org_Result_ptr++;
            Result_ptr++;
        }
    }
#endif

    return 0;
}

static void fd_gpio_init(void)
{
    gpio_init_io (GPIO_EN_PIN1, GPIO_OUTPUT);
    gpio_set_port_attribute(GPIO_EN_PIN1, 1);
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

#if FL_EN == 1
static void drv_fl_lock(void)
{
    if(sem_fl_engine){
        osSemaphoreWait(sem_fl_engine, osWaitForever);
    }
}

static void drv_fl_unlock(void)
{
    if(sem_fl_engine){
        osSemaphoreRelease(sem_fl_engine);
    }
}
#endif

static INT32S scalerStart_no_fd(gpImage *src, gpImage *dst)
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

    ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);
    drv_scaler_unlock();

    return ret;
}

static INT32S scalerClip_no_fd(gpImage *src, gpImage *dst, gpRect *clip)
{
    INT32S ret;

    drv_scaler_lock();
    ret =  drv_l2_scaler_clip(0, 1, src, dst, clip);
    drv_scaler_unlock();

    return ret;
}

static INT32S scalerStart_dispaly(gpImage *src, gpImage *dst)
{
    INT32S ret;

    drv_scaler_lock();
    drv_l2_scaler_full_screen(0, 1, src, dst);
    drv_scaler_unlock();

    return ret;
}

static INT32S scalerEnd(void)
{
    return 0;
}

static INT32S scalerClip_fd(gpImage *src, gpImage *dst, gpRect *clip)
{
    INT32S ret;

    drv_scaler_lock();
    ret =  drv_l2_FD_scaler_clip(0, 1, src, dst, clip);
    drv_scaler_unlock();

    return ret;
}

static INT32S scalerStart_fd(gpImage *src, gpImage *dst)
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

    ret = drv_l2_FD_scaler_clip(0, 1, src, dst, &clip);
    drv_scaler_unlock();

    return ret;
}

static gpRect GPRect_utils(int _x, int _y, int _width, int _height)
{
	gpRect rect;

	rect.x = _x;
	rect.y = _y;
	rect.width = _width;
	rect.height = _height;
	return rect;
}

static int gpRoundFLT(int value)
{
    return (value + (value >= 0 ? 32 : 31)) >> Q_FLT;
}

static void equalizeHist_new2(const gpImage* src, gpImage* dst, gpRect* roi, gpRect* applyROI)
{
	unsigned int pixNum[256];
	int i, i4;
	int roiwd4s1 = (roi->width>>2) - 1;
	int roiwd4s1_apply = (applyROI->width>>2) - 1;
	int j, j4;
	int pixNum255;
	//int roiws1 = roi->width - 1;
	unsigned char* srcptr = src->ptr + (roi->y + roi->height - 1)*src->widthStep + roi->x;
	unsigned char* dstptr = dst->ptr + (applyROI->y + applyROI->height - 1)*dst->widthStep + applyROI->x;

	gp_memset((INT8S *)pixNum, 0, 1024);

	i = roi->height;

	while ((i--) & 0x3)
	{
		// 0
		j = roi->width;

		while ((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
		} while (j--);

		srcptr -= src->widthStep;
	}

	i = (roi->height>>2) - 1;
	do
	{
		// 0
		j = roi->width;

		while ((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
		} while (j--);

		srcptr -= src->widthStep;

		// 1
		j = roi->width;

		while ((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
		} while (j--);

		srcptr -= src->widthStep;

		// 2
		j = roi->width;

		while ((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
		} while (j--);

		srcptr -= src->widthStep;

		// 3
		j = roi->width;

		while ((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
			j4++;
			(*(pixNum + *(srcptr + j4)))++;
		} while (j--);

		srcptr -= src->widthStep;
	} while (i--);

	*(pixNum + 2) += *(pixNum + 1);
	*(pixNum + 3) += *(pixNum + 2);

	i = 1;
	do
	{
		i4 = i<<2;
		*(pixNum + i4) += *(pixNum + i4 - 1);
		i4++;
		*(pixNum + i4) += *(pixNum + i4 - 1);
		i4++;
		*(pixNum + i4) += *(pixNum + i4 - 1);
		i4++;
		*(pixNum + i4) += *(pixNum + i4 - 1);
	} while ((i++) != 63);

	pixNum255 = *(pixNum + 255);

	*(pixNum + 252) = gpRoundFLT(((*(pixNum + 252)*255) << Q_FLT)/pixNum255);
	*(pixNum + 253) = gpRoundFLT(((*(pixNum + 253)*255) << Q_FLT)/pixNum255);
	*(pixNum + 254) = gpRoundFLT(((*(pixNum + 254)*255) << Q_FLT)/pixNum255);

	i = 62;
	do
	{
		i4 = i<<2;

		*(pixNum + i4) = gpRoundFLT(((*(pixNum + i4)*255) << Q_FLT)/pixNum255);
		i4++;
		*(pixNum + i4) = gpRoundFLT(((*(pixNum + i4)*255) << Q_FLT)/pixNum255);
		i4++;
		*(pixNum + i4) = gpRoundFLT(((*(pixNum + i4)*255) << Q_FLT)/pixNum255);
		i4++;
		*(pixNum + i4) = gpRoundFLT(((*(pixNum + i4)*255) << Q_FLT)/pixNum255);

	} while (i--);

	*pixNum = 0;
	*(pixNum + 255) = 255;

	// lut
	srcptr = src->ptr + (applyROI->y + applyROI->height - 1)*src->widthStep + applyROI->x;

	i = applyROI->height;

	while ((i--) & 0x3)
	{
		// 0
		j = applyROI->width;

		while ((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;

			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while (j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;
	}

	i = (applyROI->height>>2) - 1;
	do
	{
		// 0
		j = applyROI->width;

		while ((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while (j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;

		// 1
		j = applyROI->width;

		while ((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while (j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;

		// 2
		j = applyROI->width;

		while ((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while (j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;

		// 3
		j = applyROI->width;

		while ((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
			j4++;
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while (j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;
	} while (i--);
}

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
int FaceIdentify_ID_y3_MP(gpImage* gray, gpRect* userROI, FID_Info* fidInfo)
{
	int valid_num = fidInfo->TRAIN_NUM + fidInfo->update_num[fidInfo->best_subject_index]; // + update number
	int templ_index_best = fidInfo->best_templ_index[0];
	fidInfo->best_templ_index[1] = (templ_index_best == (valid_num - 1)) ? (templ_index_best - 2) : (templ_index_best + 1);
	fidInfo->best_templ_index[2] = (templ_index_best == 0) ? (templ_index_best + 2) : (templ_index_best - 1);
	int score_yn1, score_yp1, score_yn2, score_yp2, score_yn3, score_yp3;
	int facey = userROI[0].y;

	// y = -1
	userROI[0].y = facey - 1;
	int verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
	score_yn1 = fidInfo->score;

	if (!verifyResult)
	{
		// y = +1
		userROI[0].y = facey + 1;
		verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
		score_yp1 = fidInfo->score;

		if (!verifyResult)
		{
			if (score_yp1 >= score_yn1)
			{
				// y = +2
				userROI[0].y = facey + 2;
				verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
				score_yp2 = fidInfo->score;

				if (!verifyResult)
				{
					if (score_yp2 >= score_yp1)
					{
						// y = +3
						userROI[0].y = facey + 3;
						verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
						score_yp3 = fidInfo->score;
					}
					else
					{
						// y = -2
						userROI[0].y = facey - 2;
						verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
						score_yn2 = fidInfo->score;
						if (!verifyResult)
						{
							if (score_yn2 >= score_yn1)
							{
								// y = -3
								userROI[0].y = facey - 3;
								verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
								score_yn3 = fidInfo->score;
							}
						}
					}
				}
			}
			else
			{
				// y = -2
				userROI[0].y = facey - 2;
				verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
				score_yn2 = fidInfo->score;

				if (!verifyResult)
				{
					if (score_yn2 >= score_yn1)
					{
						// y = -3
						userROI[0].y = facey - 3;
						verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
						score_yn3 = fidInfo->score;
					}
					else
					{
						// y = +2
						userROI[0].y = facey + 2;
						verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
						score_yp2 = fidInfo->score;
						if (!verifyResult)
						{
							if (score_yp2 >= score_yp1)
							{
								// y = +3
								userROI[0].y = facey + 3;
								verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
								score_yp3 = fidInfo->score;
							}
						}
					}
				}
			}
		}
	}

	return verifyResult;
}

int FaceIdentify_Verify_UBT(gpImage* gray, gpRect* userROI, FID_Info* fidInfo)
{
	fidInfo->best_templ_index[0] = -1;
	int verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);
	if ((!verifyResult) && (fidInfo->subject_num == 1))
		return FaceIdentify_ID_y3_MP(gray, userROI, fidInfo);

	int isBestTwoSimilar = (fidInfo->best_subject_index != fidInfo->best_subject_index2) && (fidInfo->score2 > fidInfo->score - 20000);

#if MASTER_PRIOR == 1
	if (verifyResult && (fidInfo->best_subject_index != 0) && (fidInfo->best_subject_index2 == 0) && (fidInfo->score <= fidInfo->scoreHiTh) && (fidInfo->score2 > fidInfo->scoreLoTh) && isBestTwoSimilar)
	{
		fidInfo->best_subject_index = 0;
		fidInfo->score = fidInfo->score2;
		return verifyResult;
	}
#endif

	if (!verifyResult)
	{
		if (isBestTwoSimilar)
		{
			verifyResult = FaceIdentify_ID_y3_MP(gray, userROI, fidInfo);

			if (fidInfo->score < fidInfo->scoreHiTh)
			{
				int verifyResult1 = verifyResult;
				int best_score1 = fidInfo->score;
				int best_subject_index1 = fidInfo->best_subject_index;
				int best_subject_index2 = fidInfo->best_subject_index2;

				fidInfo->best_subject_index = fidInfo->best_subject_index2;
				fidInfo->best_templ_index[0] = fidInfo->best_templ_index2[0];

				int verifyResult2 = FaceIdentify_ID_y3_MP(gray, userROI, fidInfo);
				int best_score2 = fidInfo->score;
#if MASTER_PRIOR == 1
				if ((best_subject_index1 == 0) && (best_score1 > fidInfo->scoreLoTh) && (best_score1 > (best_score2 - 20000)))
				{
					fidInfo->best_subject_index = best_subject_index1;
					fidInfo->score = best_score1;
					verifyResult = verifyResult1;
				}
				else if ((best_subject_index2 == 0) && (best_score2 > fidInfo->scoreLoTh) && (best_score2 > (best_score1 - 20000)))
				{
					fidInfo->best_subject_index = best_subject_index2;
					fidInfo->score = best_score2;
					verifyResult = verifyResult2;
				}
				else
#endif
				{
					if (best_score1 > best_score2)
					{
						fidInfo->best_subject_index = best_subject_index1;
						fidInfo->score = best_score1;
						verifyResult = verifyResult1;
					}
					else
					{
						fidInfo->best_subject_index = best_subject_index2;
						verifyResult = verifyResult2;
					}
				}
			}
		}
		else
		{
			verifyResult = FaceIdentify_ID_y3_MP(gray, userROI, fidInfo);
		}
	}

	return verifyResult;
}

static void face_recognize_proc_new(void *id_work_mem, gpImage *org_img, void *pre_result, INT32U yuv_buf, INT32U fd_cnt)
{
	#define DRAW_IMAGE_EN           1
	INT8U path[128];
	ObjResult_t *p_obj_result = (ObjResult_t *)pre_result;
	FaceIdentify_t *idWorkMem = (FaceIdentify_t *)id_work_mem;
	INT32S ret,state,identify_score[FD_TRAINING_IMAGE_NUMBER];
	INT32U i,identify_index,temp;
	gpRect image[3];
    INT32U clip_frame_buf,save_size;

	if(p_obj_result->is_best_face && (idWorkMem->id_mode == 1 || idWorkMem->id_mode == 2))
	{
		// face ROI
		image[0] = p_obj_result->rect[FACE_ROI];
		// leye	ROI
		image[1] = p_obj_result->rect[LEYE_ROI];
		// reye ROI
		image[2] = p_obj_result->rect[REYE_ROI];
	}

	if(p_obj_result->is_best_face == 0) {
		//no face
		idWorkMem->identify_cnt = 0;
		return;
	} else if(idWorkMem->id_mode == 0) {
		// idle
		idWorkMem->training_cnt = 0;
		idWorkMem->identify_cnt = 0;
		return;
	} else if(idWorkMem->id_mode == 1) {
		// traning
        fidInfo.subject_index = train_subject;
        fidInfo.train_index = idWorkMem->training_cnt;
        FaceIdentify_Train_MP(org_img, &image[0], &fidInfo);
		idWorkMem->training_cnt++;
		DBG_PRINT("training_cnt = %d\r\n", idWorkMem->training_cnt);
		if(idWorkMem->training_cnt >= FD_TRAINING_IMAGE_NUMBER) {
			DBG_PRINT("Training Face[%d] Success\r\n", train_subject);
			drv_fd_result_lock();
			idWorkMem->training_cnt = 0;
			idWorkMem->identify_cnt = 0;
            train_subject++;
			if(train_subject >= fidInfo.MAX_SUBJECT_NUM)
			{
                obj_result.training_ok = 1;
                idWorkMem->training_state = 1;
                idWorkMem->id_mode = 2;
            #if FACE_RECOGNIZE_MEUN_EN == 1
                fd_mode = idWorkMem->id_mode;
            #endif
                idWorkMem->training_subject_cnt = train_subject;
                //fidInfo.subject_num = train_subject;
                train_subject = 0;
			}
			else
			{
                obj_result.training_ok = 0;
                idWorkMem->training_state = 0;
                idWorkMem->id_mode = 0;
                idWorkMem->training_subject_cnt = train_subject;
                //fidInfo.subject_num = train_subject;
            #if FACE_RECOGNIZE_MEUN_EN == 1
                fd_mode = idWorkMem->id_mode;
            #endif
			}
            drv_fd_result_unlock();
			if(idWorkMem->pFuncStore) {
				idWorkMem->pFuncStore((INT32U)idWorkMem->ownerULBP, SZ_ONEDATA*(fidInfo.MAX_SAMPLE_NUM + 10));
			}
		}
	} else if(idWorkMem->id_mode == 2) {
        // identify
        //if(idWorkMem->training_subject_cnt == fidInfo.MAX_SUBJECT_NUM)
        //if(idWorkMem->training_subject_cnt == fidInfo.subject_num)
        {
            if(fd_cnt == 1)
                ret = FaceIdentify_Verify_UBT(org_img, &image[0], &fidInfo);
            else
            {
                ret = 0;
                for(i=0;i<fd_cnt;i++)
                {
                    state = FaceIdentify_Verify_UBT(org_img, &p_obj_result->rect[(i*3)], &fidInfo);
                    if(state)
                        ret++;
                }
            }
        }
        //else
            //ret = 0;

		if(ret == 0) {
			DBG_PRINT("NoID\r\n");
			return;
		}
		else
		{
            DBG_PRINT("People GROUP[%d] Success, score = %d\r\n", (fidInfo.best_subject_index + 1), fidInfo.score);
		}

        DBG_PRINT("Identify Success\r\n");
        drv_fd_result_lock();
        idWorkMem->id_mode = 0; //idle mode
    #if FACE_RECOGNIZE_MEUN_EN == 1
        fd_mode = idWorkMem->id_mode;
    #endif
        drv_fd_result_unlock();
        idWorkMem->training_cnt = 0;
        idWorkMem->identify_cnt = 0;
        idWorkMem->identify_state = 1;
        obj_result.identify_ok = 1;
	} else {
		DBG_PRINT("face_recognize_proc_new unknow state\r\n");
	}
}

/**
* @brief Face Detection // 2015.05.20
* @param[in] mode: 1 for training mode, 2 for verification mode
* @return detection result is good or not
*/
static int faceRoiDetect_new(const gpImage* gray, gpImage* gpHE, gpRect* userROI, int mode)
{
	#define DETMAX_RESULT              256
	gpRect* faceROI = &userROI[0];
#if FACE_DETECT_MULTI_PEOPLE_FLOW_EN == 1
    gpRect lEyeROI_ptr;
	gpRect rEyeROI_ptr;
    gpRect* lEyeROI = &lEyeROI_ptr;
	gpRect* rEyeROI = &rEyeROI_ptr;
    INT32U id_mode_en = 1;
#else
	gpRect* lEyeROI = &userROI[1];
	gpRect* rEyeROI = &userROI[2];
	INT32U id_mode_en = 0;
#endif
    gpRect faceMultiROI[DETMAX_RESULT/4];

	int HEIGHT = gray->height;
	int WIDTH = gray->width;

	gpRect faceResult[DETMAX_RESULT];
	gpRect rEyeResult[DETMAX_RESULT];
	gpRect lEyeResult[DETMAX_RESULT];
	int faceCount[DETMAX_RESULT];
	int rEyeCount[DETMAX_RESULT];
	int lEyeCount[DETMAX_RESULT];

	// Initialization //
	int ret, faceN, rEyeN, lEyeN, t, int_scale_face, int_scale_eye, int_scale_eye_retry;
    int fd_state;

	// Vtech's setting
	int xstep_face = 2;
	int ystep_face = 2;
	int xstep_eye = 2;
	int ystep_eye = 2;
	int min_face_nbr_h = 5;
	int min_face_nbr_l = 2;
	int min_eye_nbr = 1;
	int min_face_wnd = 50;
	int max_face_wnd = (int)(0.7*gray->height);
	int_scale_face = 72089;
	int_scale_eye = 72089;
	int_scale_eye_retry = 68811;
	SCALE_METHOD scale_method = SCALE_LINEAR;

	void *WorkMem = 0;
	gpRect Rect;

	fd_state = 0;

	Rect.x = 0;
	Rect.y = 0;
	Rect.width = gray->width;
	Rect.height = gray->height;

	t = MIN(Rect.width, Rect.height);
	if(max_face_wnd>t) max_face_wnd = t;

	t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, DETMAX_RESULT, xstep_face, ystep_face);

    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t,4);
        fd_workmem_size = t;
        WorkMem = (void *)fd_workmem;
        DBG_PRINT("FD_WorkMem:0x%X \r\n",WorkMem);
    }
    else
        WorkMem = (void *)fd_workmem;

    gp_memset((INT8S *)WorkMem,0, t);
    gp_memset((INT8S *)&userROI[0],0, 3*sizeof(gpRect));

	//--------------------
	//	Face Detection
	//--------------------
	ret = FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, DETMAX_RESULT, xstep_face, ystep_face);

	/* setting cascade type (face) */
	ClassifyData clfface;
	clfface.obj_type = OBJ_FACE;
	FaceDetect_set_detect_obj(WorkMem, &clfface);

#if 1//def 1//IS_HW_SCALER
	//FaceDetect_set_ScalerFn(WorkMem, gp6Scaler, Scaler_wait_end, Scaler_clip);
	FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
#else
	FaceDetect_set_ScalerFn(WorkMem, gp12Scaler, Scaler_wait_end, Scaler_clip);
#endif

	ret = FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_face_wnd, scale_method);

	FaceDetect_SetX(WorkMem, xstep_face, 0);

	faceN = FaceDetect(WorkMem, gray, &Rect, DETMAX_RESULT, faceResult, faceCount);

	int isWeakFace = 0;
	if (!faceN) // do HE
	{
		gpRect eqROI = GPRect_utils(140, 140, 83, 54);
		gpRect applyROI = GPRect_utils(57, 0, 75, 240);
		equalizeHist_new2(gray, gpHE, &eqROI, &applyROI);

		faceN = FaceDetect(WorkMem, gpHE, &Rect, DETMAX_RESULT, faceResult, faceCount);
		isWeakFace = 1;
	}
	if (!faceN)
	{
		// release memory //
		return 0;
	}
	else
        fd_state = 1;

	int maxFaceW = 0;
	int maxFaceCount = min_face_nbr_l;
	int best_face = 0;

	int i = faceN-1;
	do
	{
        if(id_mode_en && (mode == 2))
        //if(id_mode_en)
        {
            if(faceCount[i] >= min_face_nbr_l)
            {
                faceMultiROI[i].x = faceResult[i].x;
                faceMultiROI[i].y = faceResult[i].y;
                faceMultiROI[i].width = faceResult[i].width;
                faceMultiROI[i].height = faceResult[i].height;
                best_face++;
            }
        }
        else
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
        }
	} while (i--);

    if(id_mode_en && (mode == 2))
    //if(id_mode_en)
    {
        fd_state = 0;
        for(i=0;i<best_face;i++)
        {
            /* Face Position Determination */
            int offset_x = (int)(faceMultiROI[i].width*0.16);
            int offset_y = faceMultiROI[i].height/7;
            faceROI->x = faceMultiROI[i].x + offset_x;
            faceROI->y = faceMultiROI[i].y + offset_y;
            faceROI->width = faceMultiROI[i].width - (offset_x<<1);
            faceROI->height = (short)(faceMultiROI[i].height - offset_y*1.5);

            //--------------------
            //	Eyes Detection
            //--------------------
            int min_eye_wnd = MAX(faceROI->width/7, 24); // 20 is minimum
            int max_eye_wnd = (int)(faceROI->width*0.6);
            int detectH = (int)(faceMultiROI[i].height*0.6);


            //--------------------
            //	Before Eyes Detection, do Face Normalization first
            //--------------------
            // setting cascade type (right eye) //
            gp_memset(WorkMem, 0, t);
            gpRect rFace = GPRect_utils(faceMultiROI[i].x + (faceMultiROI[i].width>>1), faceMultiROI[i].y, (faceMultiROI[i].width>>1), detectH);
            ret = FaceDetect_Config(WorkMem, t, rFace.width, rFace.height, 1, DETMAX_RESULT, xstep_eye, ystep_eye);

            ClassifyData clfreye;
            clfreye.obj_type = OBJ_REYE;
            FaceDetect_set_detect_obj(WorkMem, &clfreye);

        #if 1//def IS_HW_SCALER
            //FaceDetect_set_ScalerFn(WorkMem, gp6Scaler, Scaler_wait_end, Scaler_clip);
            FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
        #else
            FaceDetect_set_ScalerFn(WorkMem, gp12Scaler, Scaler_wait_end, Scaler_clip);
        #endif
            ret = FaceDetect_SetScale(WorkMem, int_scale_eye, min_eye_wnd, max_eye_wnd, scale_method);

            FaceDetect_SetX(WorkMem, xstep_eye, 0);

            if (isWeakFace)
                rEyeN = FaceDetect(WorkMem, gpHE, &rFace, DETMAX_RESULT, rEyeResult, rEyeCount);
            else
                rEyeN = FaceDetect(WorkMem, gray, &rFace, DETMAX_RESULT, rEyeResult, rEyeCount);

            if (!rEyeN)
            {
                ret = FaceDetect_SetScale(WorkMem, int_scale_eye_retry, min_eye_wnd, max_eye_wnd, scale_method);
                equalizeHist_new2(gray, gpHE, &rFace, &rFace);
                rEyeN = FaceDetect(WorkMem, gray, &rFace, DETMAX_RESULT, rEyeResult, rEyeCount);
            }

            // setting cascade type (left eye) //
            gp_memset(WorkMem, 0, t);
            gpRect lFace = GPRect_utils(faceMultiROI[i].x, faceMultiROI[i].y, rFace.width, rFace.height);
            ret = FaceDetect_Config(WorkMem, t, lFace.width, lFace.height, 1, DETMAX_RESULT, xstep_eye, ystep_eye);

            ClassifyData clfleye;
            clfleye.obj_type = OBJ_LEYE;
            FaceDetect_set_detect_obj(WorkMem, &clfleye);

        #if 1//def IS_HW_SCALER
            //FaceDetect_set_ScalerFn(WorkMem, gp6Scaler, Scaler_wait_end, Scaler_clip);
            FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
        #else
            FaceDetect_set_ScalerFn(WorkMem, gp12Scaler, Scaler_wait_end, Scaler_clip);
        #endif
            ret = FaceDetect_SetScale(WorkMem, int_scale_eye, min_eye_wnd, max_eye_wnd, scale_method);

            FaceDetect_SetX(WorkMem, xstep_eye, 0);

            if (isWeakFace)
                lEyeN = FaceDetect(WorkMem, gpHE, &lFace, DETMAX_RESULT, lEyeResult, lEyeCount);
            else
                lEyeN = FaceDetect(WorkMem, gray, &lFace, DETMAX_RESULT, lEyeResult, lEyeCount);

            if (!lEyeN)
            {
                ret = FaceDetect_SetScale(WorkMem, int_scale_eye_retry, min_eye_wnd, max_eye_wnd, scale_method);
                equalizeHist_new2(gray, gpHE, &lFace, &lFace);
                lEyeN = FaceDetect(WorkMem, gpHE, &lFace, DETMAX_RESULT, lEyeResult, lEyeCount);
            }

            if (!rEyeN)
            {
                rEyeROI->width = 0;
                fd_state = 0;
            }
            else
            {
                int minEyeDist = INT_MAX;
                int maxEyeCount = min_eye_nbr - 1;
                int most_possible_eye = 0;

                int i = rEyeN - 1;
                do
                {
                    if (rEyeCount[i] > maxEyeCount)
                    {
                        maxEyeCount = rEyeCount[i];
                        most_possible_eye = i;
                        minEyeDist = ABS(rEyeResult[i].x + rEyeResult[i].width/2 - faceROI->x - faceROI->width*3/4) + ABS(rEyeResult[i].y + rEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
                    }
                    else if (rEyeCount[i] == maxEyeCount)
                    {
                        int evaluateDist
                            = ABS(rEyeResult[i].x + rEyeResult[i].width/2 - faceROI->x - faceROI->width*3/4) + ABS(rEyeResult[i].y + rEyeResult[i].height/2 - faceROI->y - faceROI->height/4);

                        if (evaluateDist < minEyeDist)
                        {
                            minEyeDist = evaluateDist;
                            most_possible_eye = i;
                        }
                    }
                } while (i--);

                // Face Position Determination //
                *rEyeROI = rEyeResult[most_possible_eye];
            }

            if (!lEyeN || !rEyeN)
            {
                lEyeROI->width = 0;
                fd_state = 0;
            }
            else
            {
                int minEyeDist = INT_MAX;
                int maxEyeCount = min_eye_nbr - 1;
                int most_possible_eye = 0;

                int i = lEyeN - 1;
                do
                {
                    if (lEyeCount[i] > maxEyeCount)
                    {
                        maxEyeCount = lEyeCount[i];
                        most_possible_eye = i;
                        minEyeDist = ABS(lEyeResult[i].x + lEyeResult[i].width/2 - faceROI->x - faceROI->width/4) + ABS(lEyeResult[i].y + lEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
                    }
                    else if (lEyeCount[i] == maxEyeCount)
                    {
                        int evaluateDist
                            = ABS(lEyeResult[i].x + lEyeResult[i].width/2 - faceROI->x - faceROI->width/4) + ABS(lEyeResult[i].y + lEyeResult[i].height/2 - faceROI->y - faceROI->height/4);

                        if (evaluateDist < minEyeDist)
                        {
                            minEyeDist = evaluateDist;
                            most_possible_eye = i;
                        }
                    }
                } while (i--);

                // Face Position Determination //
                *lEyeROI = lEyeResult[most_possible_eye];
                #if 1
                    #if 1
                        faceROI++;
                        *faceROI = *lEyeROI;
                        faceROI++;
                        *faceROI = *rEyeROI;
                        faceROI++;
                    #else
                        DBG_PRINT("faceROI[%d].x = %d\r\n", fd_state, faceROI->x);
                        DBG_PRINT("faceROI[%d].y = %d\r\n", fd_state, faceROI->y);
                        DBG_PRINT("faceROI[%d].width = %d\r\n", fd_state, faceROI->width);
                        DBG_PRINT("faceROIp[%d].height = %d\r\n", fd_state, faceROI->height);
                        faceROI++;
                        *faceROI = *lEyeROI;
                        DBG_PRINT("lEyeROI[%d].x = %d\r\n", fd_state, faceROI->x);
                        DBG_PRINT("lEyeROI[%d].y = %d\r\n", fd_state, faceROI->y);
                        DBG_PRINT("lEyeROI[%d].width = %d\r\n", fd_state, faceROI->width);
                        DBG_PRINT("lEyeROI[%d].height = %d\r\n", fd_state, faceROI->height);
                        faceROI++;
                        *faceROI = *rEyeROI;
                        DBG_PRINT("rEyeROI[%d].x = %d\r\n", fd_state, faceROI->x);
                        DBG_PRINT("rEyeROI[%d].y = %d\r\n", fd_state, faceROI->y);
                        DBG_PRINT("rEyeROI[%d].width = %d\r\n", fd_state, faceROI->width);
                        DBG_PRINT("rEyeROI[%d].height = %d\r\n", fd_state, faceROI->height);
                        faceROI++;
                    #endif
                #else
                    faceROI++;
                    faceROI->x = lEyeROI->x;
                    faceROI->y = lEyeROI->y;
                    faceROI->width = lEyeROI->width;
                    faceROI->height = lEyeROI->height;
                    faceROI++;
                    faceROI->x = rEyeROI->x;
                    faceROI->y = rEyeROI->y;
                    faceROI->width = rEyeROI->width;
                    faceROI->height = rEyeROI->height;
                    faceROI++;
                #endif
                fd_state++;
            }
        }
    }
    else
    {
        if (!maxFaceW)
        {
            // release memory //
            return 0;
        }
        else
            fd_state = 1;

        /* Face Position Determination */
        int offset_x = (int)(faceResult[best_face].width*0.16);
        int offset_y = faceResult[best_face].height/7;
        faceROI->x = faceResult[best_face].x + offset_x;
        faceROI->y = faceResult[best_face].y + offset_y;
        faceROI->width = faceResult[best_face].width - (offset_x<<1);
        faceROI->height = (short)(faceResult[best_face].height - offset_y*1.5);

        //--------------------
        //	Eyes Detection
        //--------------------
        int min_eye_wnd = MAX(faceROI->width/7, 24); // 20 is minimum
        int max_eye_wnd = (int)(faceROI->width*0.6);
        int detectH = (int)(faceResult[best_face].height*0.6);


        //--------------------
        //	Before Eyes Detection, do Face Normalization first
        //--------------------
        // setting cascade type (right eye) //
        memset(WorkMem, 0, t);
        gpRect rFace = GPRect_utils(faceResult[best_face].x + (faceResult[best_face].width>>1), faceResult[best_face].y, (faceResult[best_face].width>>1), detectH);
        ret = FaceDetect_Config(WorkMem, t, rFace.width, rFace.height, 1, DETMAX_RESULT, xstep_eye, ystep_eye);

        ClassifyData clfreye;
        clfreye.obj_type = OBJ_REYE;
        FaceDetect_set_detect_obj(WorkMem, &clfreye);

    #if 1//def IS_HW_SCALER
        //FaceDetect_set_ScalerFn(WorkMem, gp6Scaler, Scaler_wait_end, Scaler_clip);
        FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
    #else
        FaceDetect_set_ScalerFn(WorkMem, gp12Scaler, Scaler_wait_end, Scaler_clip);
    #endif
        ret = FaceDetect_SetScale(WorkMem, int_scale_eye, min_eye_wnd, max_eye_wnd, scale_method);

        FaceDetect_SetX(WorkMem, xstep_eye, 0);

        if (isWeakFace)
            rEyeN = FaceDetect(WorkMem, gpHE, &rFace, DETMAX_RESULT, rEyeResult, rEyeCount);
        else
            rEyeN = FaceDetect(WorkMem, gray, &rFace, DETMAX_RESULT, rEyeResult, rEyeCount);

        if (!rEyeN)
        {
            ret = FaceDetect_SetScale(WorkMem, int_scale_eye_retry, min_eye_wnd, max_eye_wnd, scale_method);
            equalizeHist_new2(gray, gpHE, &rFace, &rFace);
            rEyeN = FaceDetect(WorkMem, gray, &rFace, DETMAX_RESULT, rEyeResult, rEyeCount);
        }

        // setting cascade type (left eye) //
        memset(WorkMem, 0, t);
        gpRect lFace = GPRect_utils(faceResult[best_face].x, faceResult[best_face].y, rFace.width, rFace.height);
        ret = FaceDetect_Config(WorkMem, t, lFace.width, lFace.height, 1, DETMAX_RESULT, xstep_eye, ystep_eye);

        ClassifyData clfleye;
        clfleye.obj_type = OBJ_LEYE;
        FaceDetect_set_detect_obj(WorkMem, &clfleye);

    #if 1//def IS_HW_SCALER
        //FaceDetect_set_ScalerFn(WorkMem, gp6Scaler, Scaler_wait_end, Scaler_clip);
        FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
    #else
        FaceDetect_set_ScalerFn(WorkMem, gp12Scaler, Scaler_wait_end, Scaler_clip);
    #endif
        ret = FaceDetect_SetScale(WorkMem, int_scale_eye, min_eye_wnd, max_eye_wnd, scale_method);

        FaceDetect_SetX(WorkMem, xstep_eye, 0);

        if (isWeakFace)
            lEyeN = FaceDetect(WorkMem, gpHE, &lFace, DETMAX_RESULT, lEyeResult, lEyeCount);
        else
        lEyeN = FaceDetect(WorkMem, gray, &lFace, DETMAX_RESULT, lEyeResult, lEyeCount);

        if (!lEyeN)
        {
            ret = FaceDetect_SetScale(WorkMem, int_scale_eye_retry, min_eye_wnd, max_eye_wnd, scale_method);
            equalizeHist_new2(gray, gpHE, &lFace, &lFace);
            lEyeN = FaceDetect(WorkMem, gpHE, &lFace, DETMAX_RESULT, lEyeResult, lEyeCount);
        }

        if (!rEyeN)
        {
            rEyeROI->width = 0;
            fd_state = 0;
        }
        else
        {
            int minEyeDist = INT_MAX;
            int maxEyeCount = min_eye_nbr - 1;
            int most_possible_eye = 0;

            int i = rEyeN - 1;
            do
            {
                if (rEyeCount[i] > maxEyeCount)
                {
                    maxEyeCount = rEyeCount[i];
                    most_possible_eye = i;
                    minEyeDist = ABS(rEyeResult[i].x + rEyeResult[i].width/2 - faceROI->x - faceROI->width*3/4) + ABS(rEyeResult[i].y + rEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
                }
                else if (rEyeCount[i] == maxEyeCount)
                {
                    int evaluateDist
                        = ABS(rEyeResult[i].x + rEyeResult[i].width/2 - faceROI->x - faceROI->width*3/4) + ABS(rEyeResult[i].y + rEyeResult[i].height/2 - faceROI->y - faceROI->height/4);

                    if (evaluateDist < minEyeDist)
                    {
                        minEyeDist = evaluateDist;
                        most_possible_eye = i;
                    }
                }
            } while (i--);

            // Face Position Determination //
            *rEyeROI = rEyeResult[most_possible_eye];
        }

        if (!lEyeN)
        {
            lEyeROI->width = 0;
            fd_state = 0;
        }
        else
        {
            int minEyeDist = INT_MAX;
            int maxEyeCount = min_eye_nbr - 1;
            int most_possible_eye = 0;

            int i = lEyeN - 1;
            do
            {
                if (lEyeCount[i] > maxEyeCount)
                {
                    maxEyeCount = lEyeCount[i];
                    most_possible_eye = i;
                    minEyeDist = ABS(lEyeResult[i].x + lEyeResult[i].width/2 - faceROI->x - faceROI->width/4) + ABS(lEyeResult[i].y + lEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
                }
                else if (lEyeCount[i] == maxEyeCount)
                {
                    int evaluateDist
                        = ABS(lEyeResult[i].x + lEyeResult[i].width/2 - faceROI->x - faceROI->width/4) + ABS(lEyeResult[i].y + lEyeResult[i].height/2 - faceROI->y - faceROI->height/4);

                    if (evaluateDist < minEyeDist)
                    {
                        minEyeDist = evaluateDist;
                        most_possible_eye = i;
                    }
                }
            } while (i--);


            // Face Position Determination //
            *lEyeROI = lEyeResult[most_possible_eye];
        }
    }

#if 0
	if (((lEyeROI->y + lEyeROI->height/2) < faceROI->y) || ((rEyeROI->y + rEyeROI->height/2) < faceROI->y))
		DBG_PRINT("Error: unreasonable eye information\n");
#endif

    if(id_mode_en && (mode == 2))
    //if(id_mode_en)
    {
    #if 0
        faceROI = (gpRect *)&userROI[0];
        for(i=0;i<fd_state;i++)
        {
            faceROI++;
            lEyeROI = (gpRect *)faceROI;
            faceROI++;
            rEyeROI = (gpRect *)faceROI;
            faceROI--;
            faceROI--;
            if((rEyeROI->width) && (lEyeROI->width))
            {
                double x_ratio = 0;
                double lower_ratio = 1.85;
                double upper_ratio = 0.5;
                int eyedist, lowerbound, upperbound;

                eyedist = (rEyeROI->x + rEyeROI->width/2 - lEyeROI->x - lEyeROI->width/2);
                lowerbound = (int)MIN(((rEyeROI->y + lEyeROI->y + (rEyeROI->height + lEyeROI->height)/2)/2 + lower_ratio*eyedist), HEIGHT - 1);
                upperbound = (int)MAX(((rEyeROI->y + lEyeROI->y + (rEyeROI->height + lEyeROI->height)/2)/2 - eyedist*upper_ratio), 0);
                DBG_PRINT("lowerbound = %d\r\n", lowerbound);
                DBG_PRINT("upperbound = %d\r\n", upperbound);
                faceROI->y = upperbound;
                faceROI->height = lowerbound - upperbound;
            }
            faceROI++;
            faceROI++;
        }
    #endif
    }
    else
    {
        if ((rEyeROI->width) && (lEyeROI->width))
        {
            double x_ratio = 0;
            double lower_ratio = 1.85;
            double upper_ratio = 0.5;
            int eyedist = (rEyeROI->x + rEyeROI->width/2 - lEyeROI->x - lEyeROI->width/2);
            int lowerbound = (int)MIN(((rEyeROI->y + lEyeROI->y + (rEyeROI->height + lEyeROI->height)/2)/2 + lower_ratio*eyedist), HEIGHT - 1);
            int upperbound = (int)MAX(((rEyeROI->y + lEyeROI->y + (rEyeROI->height + lEyeROI->height)/2)/2 - eyedist*upper_ratio), 0);

            faceROI->y = upperbound;
            faceROI->height = lowerbound - upperbound;
        }
    }

    return fd_state;

#if 1 // for unlock mode
	//if ((mode == 0) && (ABS(faceROI->x + faceROI->width/2 - WIDTH/2) < WIDTH/8) && (ABS(faceROI->y + faceROI->height/2 - HEIGHT/2) < HEIGHT/8) && (faceROI->height > HEIGHT/4) && (faceROI->height < 2*HEIGHT/3))
	if (mode == 0)
		return 1;
	//else if ((mode == 1) && (ABS(faceROI->x + faceROI->width/2 - WIDTH/2) < WIDTH/5) && (ABS(faceROI->y + faceROI->height/2 - HEIGHT/2) < HEIGHT/4))
	else if (mode == 1)
		return 1;
	else
		return 0;
#endif

#if 0 // DEMO BOARD
#if 1 // Sensor 0308

	if ((mode == 0) && (ABS(faceROI->x + faceROI->width/2 - WIDTH/2) < 40) && ((faceROI->y + faceROI->height/2)  > 90) && ((faceROI->y + faceROI->height/2)  < 170) && (faceROI->height > 75) && (faceROI->height < 150))
		return 1;
	else if ((mode == 1) && (ABS(faceROI->x + faceROI->width/2 - WIDTH/2) < 60) && (ABS(faceROI->y + faceROI->height/2 - HEIGHT/2) < 60))
		return 1;
	else
		return 0;

#else // Sensor 7670

	if ((mode == 0) && (ABS(faceROI->x + faceROI->width/2 - WIDTH/2) < 40) && ((faceROI->y + faceROI->height/2)  > 90) && ((faceROI->y + faceROI->height/2)  < 170) && (faceROI->height > 85) && (faceROI->height < 145))
		return 1;
	else if ((mode == 1) && (ABS(faceROI->x + faceROI->width/2 - WIDTH/2) < 60) && (ABS(faceROI->y + faceROI->height/2 - HEIGHT/2) < 60))
		return 1;
	else
		return 0;

#endif
#endif

	return 1;
}

#endif

#if KOT_EN == 1
static INT32U user_malloc_function(INT32U size)
{
    INT32U temp = 0;

    temp = (INT32U)gp_malloc_align((size+64), 32);

    if(temp)
    {
        temp = (INT32U)((temp + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    }

    return temp;
}


static INT32S user_KOT_hw_function(INT32U rfms_min_base, INT32U obj_num, INT32U objectDes_ptr, INT32U imageDes_ptr, INT32U *hw_min_error, INT32U *hw_min_error_id)
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

INT32S KOT_scalerStart(gpImage *src, gpImage *dst)
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
	//ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);
	ret = drv_l2_FD_scaler_clip(0, 1, src, dst, &clip);
	drv_scaler_unlock();

	return ret;
}
INT32S KOT_scalerClipStart(gpImage *src, gpImage *dst, gpRect clip)
{
	INT32S ret;
	gpImage temp;

	drv_scaler_lock();
	scaler_int_w_step = dst->width;
	ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);
	drv_scaler_unlock();

	return ret;
}
INT32S KOT_scalerEnd(void)
{
	return 0;
}
#endif

#if FACE_TRACKING_EN == 1 || GESTURE_FIST_TRACKING_EN == 1
////////////////////////////////////////////////////////////////
// tracking
////////////////////////////////////////////////////////////////
INT32S obj_track_init(ObjDetect_t *odWorkMem)
{
	INT32U mem_size;
	gpImage *img = &odWorkMem->image;

	mem_size = track_get_memory_size();
	odWorkMem->obj_track_WorkMem = (void *)gp_malloc_align((mem_size+2048), 16);
	if(odWorkMem->obj_track_WorkMem == 0) {
		DBG_PRINT("Fail to allocate obj_detect_WorkMem\r\n");
		return -1;
	}
    DBG_PRINT("obj_track_WorkMem[%d] = 0x%x\r\n",TrackWorkMem_num,odWorkMem->obj_track_WorkMem);
	TrackWorkMem_num++;
	gp_memset((INT8S *)odWorkMem->obj_track_WorkMem, 0x00, mem_size);

	track_init(odWorkMem->obj_track_WorkMem, img->width, img->height);

	return 0;
}
#endif

#if GESTURE_FIST_TRACKING_EN == 1
int gesture_detect_proc(ObjDetect_t *odWorkMem)
{
	#define TWO_HAND_MAX_CNT            3
	INT32S i, j, N, nCnt, hand_mode;
	INT32S *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	gpRect *p_range;
	gpRect *p_result;
	INT32S search_cnt, min_width, max_result_t;
	INT32S xstep, scale = 72089;
	INT32S best_face = -1;
	INT32S obj_cnt = 0;
	INT32U t1,t2,maxFaceW = 0;
	INT32U maxFaceCount = OBJDETECT_MIN_NBR;
    int xstep_hand = 2;
	int ystep_hand = 2;
	gpRect *p_obj_rect = obj_result.rect;
	gpRect *p_best = &obj_result.best_face;
#if MULTIOLE_TRACKING_EN == 1
	ObjDetect_t *odWorkMem2 = (ObjDetect_t *)&TrackWorkMem_ptr->trackWorkMem[MULTIOLE_HAND2];
#endif
	gpRect *p_obj_rect2;
#if TWO_HAND_MODE == 1
    gpRect Rect;
    int min_face_wnd = 30;
    int max_wnd = MIN(odWorkMem->image.width, odWorkMem->image.height);
#endif

    N = FaceDetect_Config(0, 0, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);
    if(hand_workmem == 0)
    {
        hand_workmem = (INT32U)gp_malloc_align(N,4);
        hand_workmem_size = N;
        odWorkMem->obj_detect_WorkMem = (void *)hand_workmem;
        DBG_PRINT("Hand_WorkMem:0x%X \r\n",odWorkMem->obj_detect_WorkMem);
    }
    gp_memset((INT8S *)odWorkMem->obj_detect_WorkMem,0, N);
    //gp_memset((INT8S *)&obj_result.rect[0],0, 2*sizeof(gpRect));

    FaceDetect_Config(odWorkMem->obj_detect_WorkMem, N, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

#if TWO_HAND_MODE == 1
#if MULTIOLE_TRACKING_EN == 1
    // one hand
    //p_min_width = (int *)track_get_search_min_width(odWorkMem->obj_track_WorkMem);
    p_range = (gpRect *)track_get_search_range(odWorkMem->obj_track_WorkMem);
    //max_wnd = track_get_max_range(odWorkMem->obj_track_WorkMem);
    search_cnt = track_get_search_cnt(odWorkMem->obj_track_WorkMem);
    //p_xstep = (INT32S *)track_get_xstep(odWorkMem->obj_track_WorkMem);
    //p_scale = (INT32S *)track_get_scale(odWorkMem->obj_track_WorkMem);

	p_result = odWorkMem->result;
	max_result_t = OBJDETECT_MAX_RESULT;
	p_count = odWorkMem->count;

    /* setting cascade type (face) */
    odWorkMem->clf.obj_type = OBJ_HAND;
    hand_mode = OBJ_HAND;
    FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);

    FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
    hand_num = 0;
    obj_result.rect[MULTIOLE_HAND1].x =0;
    obj_result.rect[MULTIOLE_HAND1].y = 0;
    obj_result.rect[MULTIOLE_HAND1].width = 0;
    obj_result.rect[MULTIOLE_HAND1].height = 0;
    obj_result.rect[MULTIOLE_HAND2].x =0;
    obj_result.rect[MULTIOLE_HAND2].y = 0;
    obj_result.rect[MULTIOLE_HAND2].width = 0;
    obj_result.rect[MULTIOLE_HAND2].height = 0;

    //scale = *p_scale;
    //min_face_wnd = *p_min_width;
    //xstep_hand = *p_xstep;

    FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_face_wnd, max_wnd, SCALE_NONLINEAR);

    FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep_hand, 0);
    t1 = xTaskGetTickCount();
    obj_cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, p_range, max_result_t, p_result, p_count);
    t2 = xTaskGetTickCount();
#if FD_DEBUG_EN == 1
    DBG_PRINT("hand: time=%d \r\n", (t2 - t1));
    DBG_PRINT("FaceDetect: hand=%d \r\n", obj_cnt);
#endif

    if(obj_cnt)
    {
        N = 0;
        for(j=0;j<obj_cnt;j++)
        {
            if(odWorkMem->count[j] >= TWO_HAND_MAX_CNT) {
                if(obj_result.rect[MULTIOLE_HAND1].width < odWorkMem->result[j].width)
                {
                    obj_result.rect[MULTIOLE_HAND1].x = odWorkMem->result[j].x;
                    obj_result.rect[MULTIOLE_HAND1].y = odWorkMem->result[j].y;
                    obj_result.rect[MULTIOLE_HAND1].width = odWorkMem->result[j].width;
                    obj_result.rect[MULTIOLE_HAND1].height = odWorkMem->result[j].height;
                    N++;
                }
            }
        }

        if(N)
            hand_num++;
    }

    if(hand_num == 0)
    {
        // release memory //
        return 0;
    }

    obj_result.result_cnt = hand_num;
    if(obj_result.result_cnt > 0) {
		odWorkMem->no_face_cnt = 0;
		track_reset_face_location(odWorkMem->obj_track_WorkMem);
        track_set_face_location(odWorkMem->obj_track_WorkMem, &obj_result.rect[MULTIOLE_HAND1], 0);
	} else {
		odWorkMem->no_face_cnt++;
	}

    // tracking
	track_run(odWorkMem->obj_track_WorkMem, obj_result.result_cnt, 25);

    if((p_range->x != 0) && (p_range->y != 0) && (p_range->width != PPU_TEXT_SIZE_HPIXEL) && (p_range->height != PPU_TEXT_SIZE_VPIXEL))
    {
        // two hand
        //p_min_width = (int *)track_get_search_min_width(odWorkMem2->obj_track_WorkMem);
        p_range = (gpRect *)track_get_search_range(odWorkMem2->obj_track_WorkMem);
        //max_wnd = track_get_max_range(odWorkMem2->obj_track_WorkMem);
        //p_xstep = (INT32S *)track_get_xstep(odWorkMem2->obj_track_WorkMem);
        //p_scale = (INT32S *)track_get_scale(odWorkMem2->obj_track_WorkMem);

        p_result = odWorkMem2->result;
        max_result_t = OBJDETECT_MAX_RESULT;
        p_count = odWorkMem2->count;

        //scale = *p_scale;
        //min_face_wnd = *p_min_width;
        //xstep_hand = *p_xstep;
        FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_face_wnd, max_wnd, SCALE_NONLINEAR);

        FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep_hand, 0);
        t1 = xTaskGetTickCount();
        obj_cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, p_range, max_result_t, p_result, p_count);
        t2 = xTaskGetTickCount();
    #if FD_DEBUG_EN == 1
        DBG_PRINT("hand: time=%d \r\n", (t2 - t1));
        DBG_PRINT("FaceDetect: hand=%d \r\n", obj_cnt);
    #endif

        if(obj_cnt)
        {
            N = 0;
            for(j=0;j<obj_cnt;j++)
            {
                if(odWorkMem2->count[j] >= TWO_HAND_MAX_CNT) {
                    if((odWorkMem2->result[j].x != obj_result.rect[MULTIOLE_HAND1].x) && (odWorkMem2->result[j].y != obj_result.rect[MULTIOLE_HAND1].y))
                    if(obj_result.rect[nCnt].width < odWorkMem2->result[j].width)
                    {
                        obj_result.rect[MULTIOLE_HAND2].x = odWorkMem2->result[j].x;
                        obj_result.rect[MULTIOLE_HAND2].y = odWorkMem2->result[j].y;
                        obj_result.rect[MULTIOLE_HAND2].width = odWorkMem2->result[j].width;
                        obj_result.rect[MULTIOLE_HAND2].height = odWorkMem2->result[j].height;
                        N++;
                    }
                }
            }

            if(N)
                hand_num++;
        }

        // get result
        p_obj_rect = &obj_result.best_face;

        if(obj_result.rect[MULTIOLE_HAND2].width > obj_result.rect[MULTIOLE_HAND1].width)
        {
            p_obj_rect->x = obj_result.rect[MULTIOLE_HAND1].x;
            p_obj_rect->y = obj_result.rect[MULTIOLE_HAND1].y;
            p_obj_rect->width = obj_result.rect[MULTIOLE_HAND1].width;
            p_obj_rect->height = obj_result.rect[MULTIOLE_HAND1].height;
        }
        else
        {
            p_obj_rect->x = obj_result.rect[MULTIOLE_HAND2].x;
            p_obj_rect->y = obj_result.rect[MULTIOLE_HAND2].y;
            p_obj_rect->width = obj_result.rect[MULTIOLE_HAND2].width;
            p_obj_rect->height = obj_result.rect[MULTIOLE_HAND2].height;
        }

        if(hand_num > 1)
        {
            obj_result.result_cnt = hand_num;
            if(obj_result.result_cnt > 0) {
                odWorkMem->no_face_cnt = 0;
                track_reset_face_location(odWorkMem2->obj_track_WorkMem);
                track_set_face_location(odWorkMem2->obj_track_WorkMem, &obj_result.rect[MULTIOLE_HAND2], 0);
            } else {
                odWorkMem->no_face_cnt++;
            }

            // tracking
            track_run(odWorkMem2->obj_track_WorkMem, 1, 25);
        }
        else
            // tracking
            track_run(odWorkMem2->obj_track_WorkMem, 0, 25);

        p_obj_rect = (gpRect *)&obj_result.rect[MULTIOLE_HAND1];
        p_obj_rect2 = (gpRect *)&obj_result.rect[MULTIOLE_HAND2];
    }
#else
    Rect.x = 0;
	Rect.y = 0;
	Rect.width = odWorkMem->image.width;
	Rect.height = odWorkMem->image.height;
    scale = 72089;

	p_result = odWorkMem->result;
	max_result_t = OBJDETECT_MAX_RESULT;
	p_count = odWorkMem->count;

	/* setting cascade type (face) */
	odWorkMem->clf.obj_type = OBJ_HAND;
    FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);

    FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_face_wnd, max_wnd, SCALE_NONLINEAR);

	FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep_hand, 0);
    t1 = xTaskGetTickCount();
	obj_cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, &Rect, max_result_t, p_result, p_count);
    t2 = xTaskGetTickCount();
#if FD_DEBUG_EN == 1
    DBG_PRINT("hand: time=%d \r\n", (t2 - t1));
    DBG_PRINT("FaceDetect: hand=%d \r\n", obj_cnt);
#endif
	if (!obj_cnt)
	{
        // release memory //
        hand_num = 0;
        return 0;
	}
	else
        hand_mode = OBJ_HAND;

	// get result
	p_obj_rect = &obj_result.best_face;
#if 1
    obj_result.rect[0].x =0;
    obj_result.rect[0].y = 0;
    obj_result.rect[0].width = 0;
    obj_result.rect[0].height = 0;
    obj_result.rect[1].x =0;
    obj_result.rect[1].y = 0;
    obj_result.rect[1].width = 0;
    obj_result.rect[1].height = 0;
#endif
    hand_num = 0;
	for (i=0; i<obj_cnt; i++) {
		//if(odWorkMem->count[i] >= OBJDETECT_MIN_NBR) {
		if(odWorkMem->count[i] >= 3) {
			// set obj_result
			p_obj_rect->x = odWorkMem->result[i].x;
			p_obj_rect->y = odWorkMem->result[i].y;
			p_obj_rect->width = odWorkMem->result[i].width;
			p_obj_rect->height = odWorkMem->result[i].height;
			if(obj_result.rect[0].x == 0 || obj_result.rect[1].x == 0)
			{
                if(obj_result.rect[0].x == 0)
                {
                    obj_result.rect[0].x = p_obj_rect->x;
                    obj_result.rect[0].y = p_obj_rect->y;
                    obj_result.rect[0].width = p_obj_rect->width;
                    obj_result.rect[0].height = p_obj_rect->height;
                    hand_num = 1;
                }
                else
                {
                    obj_result.rect[1].x = p_obj_rect->x;
                    obj_result.rect[1].y = p_obj_rect->y;
                    obj_result.rect[1].width = p_obj_rect->width;
                    obj_result.rect[1].height = p_obj_rect->height;
                    hand_num = 2;
                }
			}
			else
			{
                if(obj_result.rect[0].width < p_obj_rect->width)
                {
                    obj_result.rect[0].x = p_obj_rect->x;
                    obj_result.rect[0].y = p_obj_rect->y;
                    obj_result.rect[0].width = p_obj_rect->width;
                    obj_result.rect[0].height = p_obj_rect->height;
                }
                else if(obj_result.rect[1].width < p_obj_rect->width)
                {
                    obj_result.rect[1].x = p_obj_rect->x;
                    obj_result.rect[1].y = p_obj_rect->y;
                    obj_result.rect[1].width = p_obj_rect->width;
                    obj_result.rect[1].height = p_obj_rect->height;
                }
			}
#if 0
            if(obj_result.rect[1].width > obj_result.rect[0].width)
            {
                obj_result.rect[2].x = obj_result.rect[0].x ;
                obj_result.rect[2].y = obj_result.rect[0].y;
                obj_result.rect[2].width = obj_result.rect[0].width;
                obj_result.rect[2].height = obj_result.rect[0].height;
                obj_result.rect[0].x = obj_result.rect[1].x ;
                obj_result.rect[0].y = obj_result.rect[1].y;
                obj_result.rect[0].width = obj_result.rect[1].width;
                obj_result.rect[0].height = obj_result.rect[1].height;
                obj_result.rect[1].x = obj_result.rect[2].x ;
                obj_result.rect[1].y = obj_result.rect[2].y;
                obj_result.rect[1].width = obj_result.rect[2].width;
                obj_result.rect[1].height = obj_result.rect[2].height;
            }
#endif
		}
	}
#endif
    DBG_PRINT("%d", hand_num);
    if(hand_num)
        hand_mode = OBJ_HAND;
    else
        hand_mode = 0;
#else
	//get tracking info
	p_min_width = (int *)track_get_search_min_width(odWorkMem->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(odWorkMem->obj_track_WorkMem);
	search_cnt = 1;
	min_width = track_get_min_wnd(odWorkMem->obj_track_WorkMem);
	i = track_get_max_range(odWorkMem->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(odWorkMem->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(odWorkMem->obj_track_WorkMem);

	//set face detect object type
    FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	p_result = odWorkMem->result;
	max_result_t = OBJDETECT_MAX_RESULT;
	p_count = odWorkMem->count;
	nCnt = 0;
	i = search_cnt;
	do {
		INT32S cnt;
		INT32S min_width;

		min_width = *p_min_width++;
		scale = *p_scale++;

#if SCALER_LINEAR_EN == 1
        FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_LINEAR);
#else
		FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_NONLINEAR);
#endif

		xstep = *p_xstep++;
		FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep, 0);

        hand_mode = 0;
		for(N=0; N<3; N++) {
			switch(odWorkMem->gesture_mode)
			{
			case 0: //N -> R -> L
				if(N == 0) {
					odWorkMem->clf.obj_type = OBJ_HAND;
					odWorkMem->gesture_mode = 0;
				} else if(N == 1) {
					odWorkMem->clf.obj_type = OBJ_HAND_RT45;
					odWorkMem->gesture_mode = 1;
				} else if(N == 2) {
					odWorkMem->clf.obj_type = OBJ_HAND_LT45;
					odWorkMem->gesture_mode = 2;
				}
				break;

			case 1: //R -> N -> L
				if(N == 0) {
					odWorkMem->clf.obj_type = OBJ_HAND_RT45;
					odWorkMem->gesture_mode = 1;
				} else if(N == 1) {
					odWorkMem->clf.obj_type = OBJ_HAND;
					odWorkMem->gesture_mode = 0;
				} else if(N == 2) {
					odWorkMem->clf.obj_type = OBJ_HAND_LT45;
					odWorkMem->gesture_mode = 2;
				}
				break;

			case 2: //L -> N -> R
				if(N == 0) {
					odWorkMem->clf.obj_type = OBJ_HAND_LT45;
					odWorkMem->gesture_mode = 2;
				} else if(N == 1) {
					odWorkMem->clf.obj_type = OBJ_HAND;
					odWorkMem->gesture_mode = 0;
				} else if(N == 2) {
					odWorkMem->clf.obj_type = OBJ_HAND_RT45;
					odWorkMem->gesture_mode = 1;
				}
				break;
			}

            // hand
			FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);
			cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, p_range, max_result_t, p_result, p_count);
			if(p_count[0] >= OBJDETECT_MIN_NBR) {
                #if 1
                    hand_mode = OBJ_HAND;
				#else
                    hand_mode = odWorkMem->clf.obj_type;
				#endif
				goto __Find;
			}
		}

		// fist
		odWorkMem->clf.obj_type = OBJ_FIST;
        FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);
        cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, p_range, max_result_t, p_result, p_count);
        if(p_count[0] >= OBJDETECT_MIN_NBR)
             hand_mode = OBJ_FIST;
__Find:
		max_result_t -= cnt;
		p_result += cnt;
		p_count += cnt;
		nCnt += cnt;
		p_range++;
		i--;
	} while(i != 0);

	// get result
	for (i=0; i<nCnt; i++) {
		if(odWorkMem->count[i] >= OBJDETECT_MIN_NBR) {
			// set obj_result
			p_obj_rect->x = odWorkMem->result[i].x;
			p_obj_rect->y = odWorkMem->result[i].y;
			p_obj_rect->width = odWorkMem->result[i].width;
			p_obj_rect->height = odWorkMem->result[i].height;
			obj_result.rect[0].x = p_obj_rect->x;
			obj_result.rect[0].y = p_obj_rect->y;
			obj_result.rect[0].width = p_obj_rect->width;
			obj_result.rect[0].height = p_obj_rect->height;
			p_obj_rect++;
			obj_cnt++;

			// get best face
			if(odWorkMem->count[i] >= OBJDETECT_MIN_fist_NBR) {
				if(odWorkMem->result[i].width > maxFaceW) {
					maxFaceW = odWorkMem->result[i].width;
					maxFaceCount = odWorkMem->count[i];
					best_face = i;
				} else if(odWorkMem->count[i] > maxFaceCount) {
					maxFaceW = odWorkMem->result[i].width;
					maxFaceCount = odWorkMem->count[i];
					best_face = i;
				}
			}
		}
	}

	obj_result.result_cnt = obj_cnt;
	if(best_face >= 0) {
		p_best->x = odWorkMem->result[best_face].x;
		p_best->y = odWorkMem->result[best_face].y;
		p_best->width = odWorkMem->result[best_face].width;
		p_best->height = odWorkMem->result[best_face].height;
		//obj_result.is_best_face = 1;
	} else {
		p_best->x = 0;
		p_best->y = 0;
		p_best->width = 0;
		p_best->height = 0;
		//obj_result.is_best_face = 0;
	}

	//DEBUG_MSG("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
	// set tracking
	if(obj_result.result_cnt > 0) {
		odWorkMem->no_face_cnt = 0;
		track_reset_face_location(odWorkMem->obj_track_WorkMem);
		for(i=0; i<obj_result.result_cnt; i++) {
			track_set_face_location(odWorkMem->obj_track_WorkMem, &obj_result.rect[i], i);
		}
	} else {
		odWorkMem->no_face_cnt++;
	}

	// tracking
	track_run_with_srach_cnt(odWorkMem->obj_track_WorkMem, obj_result.result_cnt, 25);
#endif
    return hand_mode;
}
#endif

#if EYE_NOSE_MOUTH_PROC_EN == 1
////////////////////////////////////////////////////////////////
// get illumination
// input: orgimg, _roi
// output: lum
////////////////////////////////////////////////////////////////
static void getIllumination(gpImage *orgimg, gpRect *_roi, LumStr *lum)
{
	INT32S i, j;
	gpRect roi;
	INT8U *org_ptr;

	if(_roi == NULL) {
		roi.x = 0;
		roi.y = 0;
		roi.width = orgimg->width;
		roi.height = orgimg->height;
	} else {
		roi.x = _roi->x;
		roi.y = _roi->y;
		roi.width = _roi->width;
		roi.height = _roi->height;
	}

	lum->cnt[0] = lum->cnt[1] = lum->cnt[2] = 0;
	lum->l[0] = lum->l[1] = lum->l[2] = 0;
	org_ptr = orgimg->ptr + roi.x + roi.y * orgimg->widthStep;
	for(j=0; j<roi.height; j++) {
		INT8U *p_org;

		p_org = org_ptr + j * orgimg->widthStep;
		for(i=0; i<roi.width; i++) {
			INT8U t = *p_org++;

			if(t >= lum->level[0]) {
				if(t < lum->level[1]) {
					lum->l[0] += t;
					lum->cnt[0] += 1;
				} else if (t < lum->level[2]) {
					lum->l[1] += t;
					lum->cnt[1] += 1;
				} else if	(t < lum->level[3]) {
					lum->l[2] += t;
					lum->cnt[2] += 1;
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////
// Gamma Correction for Gray Image
// input: orgimg
// output: outimg
////////////////////////////////////////////////////////////////
static INT32S GammaCorrection(gpImage *orgimg, gpImage *outimg, gpRect *_roi)
{
	INT32S i, j, dif;
	const unsigned char *gamma_table;
	INT8U *org_ptr, *out_ptr;
	gpRect roi;
	LumStr lum;

	if(_roi == NULL) {
		roi.x = 0;
		roi.y = 0;
		roi.width = orgimg->width;
		roi.height = orgimg->height;
	} else {
		roi.x = _roi->x;
		roi.y = _roi->y;
		roi.width = _roi->width;
		roi.height = _roi->height;
	}

	lum.level[0] = 0;
	lum.level[1] = 85;
	lum.level[2] = 170;
	lum.level[3] = 255;
	getIllumination(orgimg, &roi, &lum);

	dif = ((lum.cnt[0] + lum.cnt[1] + lum.cnt[2]) * 1638) >> 15;
	gamma_table = gamma_34;
	if((lum.cnt[0] - lum.cnt[1]) >= dif && (lum.cnt[0] - lum.cnt[2]) >= dif) {
		gamma_table = gamma_30;
	} else if((lum.cnt[1] - lum.cnt[0]) >= dif && (lum.cnt[1] - lum.cnt[2]) >= dif) {
		gamma_table = gamma_44;
	} else if((lum.cnt[2] - lum.cnt[0]) >= dif && (lum.cnt[2] - lum.cnt[1]) >= dif) {
		gamma_table = gamma_58;
	}

	org_ptr = orgimg->ptr + roi.x + roi.y * orgimg->widthStep;
	out_ptr = outimg->ptr + roi.x + roi.y * outimg->widthStep;
	for(j = 0; j < roi.height ; j++) {
		unsigned char *p_org, *p_out;

		p_org = org_ptr + j * orgimg->widthStep;
		p_out = out_ptr + j * outimg->widthStep;
		for( i = 0; i < roi.width; i ++ ) {
			unsigned char t;

			t = *p_org++;
			*p_out++ = gamma_table[t];
		}
	}

	return 0;
}

////////////////////////////////////////////////////////////////
// Set to zero
// input: addr, size
// output: nonr
////////////////////////////////////////////////////////////////
static void mem_set_zero_int32(void *addr, INT32S size)
{
	INT32S i;
	INT32U *p_addr;

	if(size > 0) {
		INT32S t0, t1, t2, t3, t4, t5, t6, t7;

		t0 = 0;
		p_addr = (unsigned int *)addr;

		if(size >= 8) {
			t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0;
			i = size >> 3;
			size = size - (i << 3);

			do {
				*p_addr++ = t0;
				*p_addr++ = t1;
				*p_addr++ = t2;
				*p_addr++ = t3;
				*p_addr++ = t4;
				*p_addr++ = t5;
				*p_addr++ = t6;
				*p_addr++ = t7;
				i--;
			} while( i != 0);
		}

		if(size > 0) {
			i = size;
			do {
				*p_addr++ = t0;
				i--;
			} while(i != 0);
		}
	}
}

////////////////////////////////////////////////////////////////
// Modified Histogram Equalization for Gray Image
// input: orgimg
// output: outimg
////////////////////////////////////////////////////////////////
static INT32S ModEqualizeHist_fix(gpImage *orgimg, gpImage *outimg, gpRect *roi)
{
	INT32S i, x;
	INT32S bins[256], *p_bins;
	INT32S width, height;
	INT32S widthStep, out_widthStep;
	INT32S sum;
	gpRect local_roi;
	INT32U scale;
	INT32S mean_bins;
	INT32S mean_bins_cnt;
	INT8U lut[256], *p_lut;
	INT8U *out_ptr, *org_ptr;

	if(orgimg->ch != 1) {
		return -1;
	}

	mem_set_zero_int32(bins, 256);
	if(roi == NULL) {
		local_roi.x = 0;
		local_roi.y = 0;
		local_roi.width = orgimg->width;
		local_roi.height = orgimg->height;
	} else {
		local_roi.x = roi->x;
		local_roi.y = roi->y;
		local_roi.width = roi->width;
		local_roi.height = roi->height;
	}


	width = local_roi.width;
	height = local_roi.height;
	widthStep = orgimg->widthStep;
	org_ptr = orgimg->ptr + local_roi.x + local_roi.y * widthStep;
	do {
		INT8U *p = org_ptr;

		for(x=0; x<= width-4; x+=4) {
			int v0 = *p++;
			int v1 = *p++;

			bins[v0]++;
			bins[v1]++;

			v0 = *p++;
			v1 = *p++;

			bins[v0]++;
			bins[v1]++;
		}

		for( ; x<width; x++) {
			int v0 = *p++;
			bins[v0]++;
		}

		org_ptr += widthStep;
		height-- ;
	} while(height != 0);

	mean_bins = 0.0;
	mean_bins_cnt = 0;
	p_bins = bins;
	i = 256;
	do {
		INT32S v = *p_bins++;

		if(v != 0) {
			mean_bins += v;
			mean_bins_cnt++;
		}

		i--;
	} while(i != 0);
	mean_bins = mean_bins / mean_bins_cnt;

	sum = 0;
	p_bins = bins;
	i = 256;
	do {
		INT32S v = *p_bins;

		v = MIN(mean_bins, v);
		sum += v;
		*p_bins++ = v;
		i--;
	} while(i != 0);

	scale = (255 << 16)/sum;
	sum = 0;
	p_bins = bins;
	p_lut = lut;
	i = 256;
	do {
		INT32S v = *p_bins++;

		sum += v;
        *p_lut++ = (sum*scale) >> 16;//(unsigned char)gpRound(sum*scale);
		i--;
	} while(i != 0);


	lut[0] = 0;
	out_widthStep = outimg->widthStep;
	org_ptr = orgimg->ptr + local_roi.x + local_roi.y * widthStep;
	out_ptr = outimg->ptr + local_roi.x + local_roi.y * out_widthStep;
	height = local_roi.height;
	//for(; height-- ; out_ptr += out_widthStep, org_ptr += widthStep)
	do {
		INT8U *p_org, *p_out;

		p_org = org_ptr;
		p_out = out_ptr;
		for(i=0; i<=width-4; i+=4) {
			INT32S p0, p1, p2, p3;
			INT8U t0, t1, t2, t3;

			p0 = *p_org++;
			p1 = *p_org++;
			p2 = *p_org++;
			p3 = *p_org++;
			t0 = lut[p0];
			t1 = lut[p1];
			t2 = lut[p2];
			t3 = lut[p3];
			*p_out++ = t0;
			*p_out++ = t1;
			*p_out++ = t2;
			*p_out++ = t3;
		}
		for( ; i<width; i++)
		{
			INT32S p0 = *p_org++;
			INT8U t0 = lut[p0];

			*p_out++ = t0;
		}

		out_ptr += out_widthStep;
		org_ptr += widthStep;
		height-- ;
	} while(height != 0);

	return 0;
}

void mouth_detect_init(MouthNoseDetect_t *mdWorkMem)
{
	MouthDetect_init(mdWorkMem->mouth_detect_WorkMem);
}

void mouth_detect_reset(void *para)
{
	MouthNoseDetect_t *mdWorkMem = (MouthNoseDetect_t *)para;

	mouth_detect_init(mdWorkMem);
}

void mouth_detect_free(void *md_work_mem)
{
	MouthNoseDetect_t *mdWorkMem = (MouthNoseDetect_t *)md_work_mem;

	if(mdWorkMem) {
		if(mdWorkMem->upright_face_img.ptr) {
			gp_free(mdWorkMem->upright_face_img.ptr);
			mdWorkMem->upright_face_img.ptr = 0;
		}

		if(mdWorkMem->mouth_detect_WorkMem) {
			gp_free(mdWorkMem->mouth_detect_WorkMem);
			mdWorkMem->mouth_detect_WorkMem = 0;
		}

		gp_free(mdWorkMem);
		mdWorkMem = 0;
	}
}

INT32S mouth_detect_alloc(void)
{
	MouthNoseDetect_t *mdWorkMem;
	INT32S nRet;
	INT32U mem_size;

	// mouth detect variable allocate
	mdWorkMem = gp_malloc_align(sizeof(MouthNoseDetect_t), 32);
	if(mdWorkMem == 0) {
		DBG_PRINT("Fail to allocate mdWorkMem\r\n");
		return 0;
	}
	DBG_PRINT("mdWorkMem = 0x%x\r\n", mdWorkMem);
	gp_memset((INT8S *)mdWorkMem, 0, sizeof(MouthNoseDetect_t));

	// mouth detect buffer allocate
	mdWorkMem->upright_face_img.width     = RESIZE_FACE_IMG_WIDTH;
	mdWorkMem->upright_face_img.height    = (int) (RESIZE_FACE_IMG_HEIGHT*1.2);
	mdWorkMem->upright_face_img.widthStep = RESIZE_FACE_IMG_WIDTH;
	mdWorkMem->upright_face_img.format    = IMG_FMT_GRAY;
	mdWorkMem->upright_face_img.ch        = 1;

	mem_size = mdWorkMem->upright_face_img.widthStep * mdWorkMem->upright_face_img.height;
	mdWorkMem->upright_face_img.ptr = gp_malloc_align(mem_size, 32);
	if(mdWorkMem->upright_face_img.ptr == 0) {
		DBG_PRINT("Fail to allocate mdWorkMem->upright_face_img.ptr\r\n");
		RETURN(STATUS_FAIL);
	}

	// mouth detect work memory allocate
	mem_size = MouthDetect_get_work_mem_size();
	mdWorkMem->mouth_detect_WorkMem = gp_malloc_align(mem_size, 32);
	if(mdWorkMem->mouth_detect_WorkMem == 0) {
		DBG_PRINT("Fail to allocate mdWork->mouth_detect_WorkMem memory\r\n");
		RETURN(STATUS_FAIL);
	}
	DBG_PRINT("mouth_detect_WorkMem = 0x%x\r\n", mdWorkMem->mouth_detect_WorkMem);
	gp_memset((INT8S *)mdWorkMem->mouth_detect_WorkMem, 0, mem_size);

	// mouth init
	mouth_detect_init(mdWorkMem);

	nRet = STATUS_OK;
Return:
	if(nRet < 0) {
		mouth_detect_free(mdWorkMem);
		return 0;
	}

	return (INT32S)mdWorkMem;
}

void mouth_detect_proc(void *md_work_mem, gpImage *face_img, void *pre_result)
{
	MouthNoseDetect_t *mdWorkMem = (MouthNoseDetect_t *)md_work_mem;
	FaceEyeResult_t *p_result = (FaceEyeResult_t *)pre_result;
	INT32S nose_ret, mouth_ret, int_angle, int_scale;
	INT32S eye_distance;
	double angle;
	gpPoint *leye, *reye;
	gpPoint rotated_reye;
    gpFeaturePoint *mouth_pt, *nose_pt;

	leye = &p_result->left_eye;
	reye = &p_result->right_eye;

	mouth_pt = &mdWorkMem->mouth_pt;
	nose_pt = &mdWorkMem->nose_pt;

	// get upright face
	angle = p_result->eye_angle;

	int_angle = (INT32S)(angle * 180 / PI + 0.5);
	int_scale = (INT32S)(1.0 * 1024 + 0.5);
	image_rotate(face_img, &mdWorkMem->upright_face_img, leye, int_angle, int_scale);

	eye_distance = sqrt((double)((reye->x - leye->x)*(reye->x - leye->x)
						+ (reye->y - leye->y)*(reye->y - leye->y)));

	rotated_reye.y = reye->y - eye_distance * sin(angle);
	rotated_reye.x = leye->x + eye_distance;

	// Nose Detection
	nose_ret = NoseDetect((unsigned char *)mdWorkMem->mouth_detect_WorkMem, mdWorkMem->upright_face_img, *leye, rotated_reye, (int *)nose_pt);

	// Mouth Detection
	mouth_ret = 0;
	if(nose_ret) {
		mouth_ret = MouthDetect(mdWorkMem->mouth_detect_WorkMem, mdWorkMem->upright_face_img,
							leye, &rotated_reye, nose_pt, eye_distance, mouth_pt);
	}

	if(mouth_ret) {
		obj_result.is_smile = MouthDetect_is_smile(mdWorkMem->mouth_detect_WorkMem);
	} else {
		obj_result.is_smile = 0;
	}

	obj_result.is_get_mouth = mdWorkMem->is_get_mouth = mouth_ret;
	obj_result.is_get_nose = mdWorkMem->is_get_nose = nose_ret;
}

void mouth_detect_get_result(void *md_work_mem, void *result)
{
	MouthNoseDetect_t *mdWorkMem = (MouthNoseDetect_t *) md_work_mem;
	ObjResult_t *p_obj_result = result;

	if(mdWorkMem->is_get_nose) {
		//left
		p_obj_result->nose[0].x = mdWorkMem->nose_pt.left_pt.x;
		p_obj_result->nose[0].y = mdWorkMem->nose_pt.left_pt.y;
		p_obj_result->nose[0].width = 4;
		p_obj_result->nose[0].height = 2;
		//right
		p_obj_result->nose[1].x = mdWorkMem->nose_pt.right_pt.x;
		p_obj_result->nose[1].y = mdWorkMem->nose_pt.right_pt.y;
		p_obj_result->nose[1].width = 4;
		p_obj_result->nose[1].height = 2;
	}

	if(mdWorkMem->is_get_mouth) {
		//top
		p_obj_result->mouth[0].x = mdWorkMem->mouth_pt.top_pt.x;
		p_obj_result->mouth[0].y = mdWorkMem->mouth_pt.top_pt.y;
		p_obj_result->mouth[0].width = 4;
		p_obj_result->mouth[0].height = 2;
		//right
		p_obj_result->mouth[1].x = mdWorkMem->mouth_pt.right_pt.x;
		p_obj_result->mouth[1].y = mdWorkMem->mouth_pt.right_pt.y;
		p_obj_result->mouth[1].width = 4;
		p_obj_result->mouth[1].height = 2;
		//bottom
		p_obj_result->mouth[2].x = mdWorkMem->mouth_pt.bottom_pt.x;
		p_obj_result->mouth[2].y = mdWorkMem->mouth_pt.bottom_pt.y;
		p_obj_result->mouth[2].width = 4;
		p_obj_result->mouth[2].height = 2;
		//left
		p_obj_result->mouth[3].x = mdWorkMem->mouth_pt.left_pt.x;
		p_obj_result->mouth[3].y = mdWorkMem->mouth_pt.left_pt.y;
		p_obj_result->mouth[3].width = 4;
		p_obj_result->mouth[3].height = 2;
	}
}

void eye_detect_init(EyeDetect_t *edWorkMem)
{
	edWorkMem->scale = (int) (1.1*65536);
	edWorkMem->xstep = 2;

	FaceDetect_set_ScalerFn(edWorkMem->eye_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
}

void eye_detect_free(void *ed_work_mem)
{
	EyeDetect_t *edWorkMem = (EyeDetect_t *)ed_work_mem;

	if(edWorkMem) {
		if(edWorkMem->face_img.ptr)  {
			gp_free(edWorkMem->face_img.ptr);
			edWorkMem->face_img.ptr = 0;
		}

		if(edWorkMem->eq_face_img.ptr) {
			gp_free(edWorkMem->eq_face_img.ptr);
			edWorkMem->eq_face_img.ptr = 0;
		}

		if(edWorkMem->eye_detect_WorkMem) {
			gp_free(edWorkMem->eye_detect_WorkMem);
			edWorkMem->eye_detect_WorkMem = 0;
		}

		gp_free(edWorkMem);
		edWorkMem = 0;
	}
}

INT32S eye_detect_alloc(void)
{
	EyeDetect_t *edWorkMem;
	INT32S nRet;
	INT32U mem_size;

	// eye detect variable allocate
	edWorkMem = gp_malloc_align(sizeof(EyeDetect_t), 32);
	if(edWorkMem == 0) {
		DBG_PRINT("Fail to allocate edWorkMem\r\n");
		return 0;
	}
	DBG_PRINT("edWorkMem = 0x%x\r\n", edWorkMem);
	gp_memset((INT8S *)edWorkMem, 0, sizeof(EyeDetect_t));

	// eye detect buffer allocate
	edWorkMem->face_img.width     = RESIZE_FACE_IMG_WIDTH;
	edWorkMem->face_img.height    = (int)(RESIZE_FACE_IMG_HEIGHT*1.2);
	edWorkMem->face_img.widthStep = RESIZE_FACE_IMG_WIDTH;
	edWorkMem->face_img.format    = IMG_FMT_GRAY;
	edWorkMem->face_img.ch        = 1;

	mem_size = edWorkMem->face_img.widthStep * edWorkMem->face_img.height;
	edWorkMem->face_img.ptr = gp_malloc_align(mem_size, 32);
	if(edWorkMem->face_img.ptr == 0) {
		DBG_PRINT("Fail to allocate edWorkMem->face_img.ptr\r\n");
		RETURN(STATUS_FAIL);
	}

	edWorkMem->eq_face_img.width     = RESIZE_FACE_IMG_WIDTH;
	edWorkMem->eq_face_img.height    = (int)(RESIZE_FACE_IMG_HEIGHT*1.2);
	edWorkMem->eq_face_img.widthStep = RESIZE_FACE_IMG_WIDTH;
	edWorkMem->eq_face_img.format    = IMG_FMT_GRAY;
	edWorkMem->eq_face_img.ch        = 1;

	mem_size = edWorkMem->eq_face_img.widthStep * edWorkMem->eq_face_img.height;
	edWorkMem->eq_face_img.ptr = gp_malloc_align(mem_size, 32);
	if(edWorkMem->eq_face_img.ptr == 0) {
		DBG_PRINT("Fail to allocate edWorkMem->eq_face_img.ptr\r\n");
		RETURN(STATUS_FAIL);
	}

	// eye detect work memory
	mem_size = FaceDetect_Config(0, 0, edWorkMem->face_img.width, edWorkMem->face_img.height ,
								edWorkMem->face_img.ch, OBJDETECT_MAX_RESULT, 2, 2);
	edWorkMem->eye_detect_WorkMem = gp_malloc_align(mem_size, 32);
	if(edWorkMem->eye_detect_WorkMem == 0) {
		DBG_PRINT("Fail to allocate edWorkMem->eye_detect_WorkMem\r\n");
		RETURN(STATUS_FAIL);
	}
	DBG_PRINT("eye_detect_WorkMem = 0x%x\r\n", edWorkMem->eye_detect_WorkMem);
	gp_memset((INT8S *)edWorkMem->eye_detect_WorkMem, 0, mem_size);

	// eye detect init
	mem_size = FaceDetect_Config(edWorkMem->eye_detect_WorkMem, mem_size, edWorkMem->face_img.width, edWorkMem->face_img.height,
								edWorkMem->face_img.ch, OBJDETECT_MAX_RESULT, 2, 2);

	eye_detect_init(edWorkMem);

	nRet = STATUS_OK;
Return:
	if(nRet < 0) {
		if(edWorkMem) {
			eye_detect_free(edWorkMem);
		}
		return 0;
	}

	return (INT32S)edWorkMem;
}

void eye_detect_proc(void *ed_work_mem, gpImage *org_img, void *pre_result)
{
	ObjResult_t *p_obj_result = (ObjResult_t *)pre_result;
	EyeDetect_t *edWorkMem = (EyeDetect_t *)ed_work_mem;
	gpPoint pos;
	gpRect RectEye;
	float scale_factor;
	INT32S max_wnd, k;
	INT32S eye_nbr;
    INT32S i, x, y;
    float angle;
    double x_dif, y_dif;
    gpPoint *leye;
    gpRect *r;

	edWorkMem->is_get_eye = 0;
	if(p_obj_result->result_cnt <= 0) {
		goto Return;
	}

	// set information and detect first face
	edWorkMem->fe_result.face.x = p_obj_result->rect[0].x;
	edWorkMem->fe_result.face.y = p_obj_result->rect[0].y;
	edWorkMem->fe_result.face.width = p_obj_result->rect[0].width;
	edWorkMem->fe_result.face.height = (INT32S)(p_obj_result->rect[0].height*1.2);
	if(edWorkMem->fe_result.face.width < (RESIZE_FACE_IMG_WIDTH/2)) {
		eye_nbr = 1;
	} else {
		eye_nbr = 2;
	}

	scalerClip_no_fd(org_img, &edWorkMem->face_img, &edWorkMem->fe_result.face);

	scale_factor = (float)p_obj_result->rect[0].width / (float)edWorkMem->face_img.width;

	RectEye.x = HAAR_X_MARGIN;
	RectEye.y = HAAR_Y_MARGIN;
	RectEye.width = RESIZE_FACE_IMG_WIDTH / 2;
	RectEye.height = RESIZE_FACE_IMG_HEIGHT / 2;

	max_wnd = MIN(MIN(RectEye.width, RectEye.height), 36);

	FaceDetect_SetScale(edWorkMem->eye_detect_WorkMem, edWorkMem->scale, 24, max_wnd,SCALE_NONLINEAR);

	// wait for scaler ending
	scalerEnd();

	for(k=0; k<2; k++) {
		INT32S N, i, max_idx, max_cnt;
		gpPoint *p_eye;
		gpRect *p_eyebrow;
		INT32S do_eq, pre_cnt;

		if(k == 0)  {
			edWorkMem->clf.obj_type = OBJ_LEYE;
			p_eye = &edWorkMem->fe_result.left_eye;
			p_eyebrow = &edWorkMem->fe_result.left_eyebrow;
		} else {
			edWorkMem->clf.obj_type = OBJ_REYE;
			p_eye = &edWorkMem->fe_result.right_eye;
			p_eyebrow = &edWorkMem->fe_result.right_eyebrow;
			RectEye.x = RESIZE_FACE_IMG_WIDTH / 2 - HAAR_X_MARGIN;
		}

		FaceDetect_set_detect_obj(edWorkMem->eye_detect_WorkMem, &edWorkMem->clf);

		pre_cnt = 0;
		do_eq = 0;
		do {
			if(do_eq == 0) {
				N = FaceDetect(edWorkMem->eye_detect_WorkMem, &edWorkMem->face_img, &RectEye,
							OBJDETECT_MAX_RESULT, edWorkMem->eye_rect, edWorkMem->eye_cnt);
			} else {
				// do EQ, and search again...
				ModEqualizeHist_fix(&edWorkMem->face_img, &edWorkMem->eq_face_img, &RectEye);
				N = FaceDetect(edWorkMem->eye_detect_WorkMem, &edWorkMem->eq_face_img, &RectEye,
							OBJDETECT_MAX_RESULT, edWorkMem->eye_rect, edWorkMem->eye_cnt);

				// do gamma
				if(N <= 0) {
					GammaCorrection(&edWorkMem->eq_face_img, &edWorkMem->eq_face_img, &RectEye);
					N = FaceDetect(edWorkMem->eye_detect_WorkMem, &edWorkMem->eq_face_img, &RectEye, OBJDETECT_MAX_RESULT, edWorkMem->eye_rect, edWorkMem->eye_cnt);
				}

				if(N > 0) {
					eye_nbr = 1;
				}
			}

			// find the best eye
			max_idx = 0;
			max_cnt = 0;
			for(i=0; i<N; i++) {
				if(edWorkMem->eye_cnt[i] > max_cnt) {
					max_cnt = edWorkMem->eye_cnt[i];
					max_idx = i;
				}
			}

			// assume that there is eye but it's not easy detected!!!
			max_cnt += pre_cnt;
			if(max_cnt < eye_nbr) {
				if(do_eq == 1) {
					goto Return;
				} else {
					do_eq = 1;
				}

				pre_cnt = max_cnt;
			} else {
				do_eq = 0;
			}
		} while(do_eq == 1);

		if(edWorkMem->eye_rect[max_idx].width > EYES_WIDTH) {
			INT32S val = (edWorkMem->eye_rect[max_idx].width - EYES_WIDTH + 1) / 2;

			edWorkMem->eye_rect[max_idx].x += val;
			edWorkMem->eye_rect[max_idx].width -= val * 2;

			val = ((edWorkMem->eye_rect[max_idx].height - EYES_WIDTH) * 5 + 7) / 8;
			edWorkMem->eye_rect[max_idx].y += val;
			edWorkMem->eye_rect[max_idx].height = edWorkMem->eye_rect[max_idx].width;
		}

		p_eye->x = edWorkMem->eye_rect[max_idx].x + (edWorkMem->eye_rect[max_idx].width >> 1);
		p_eye->y = edWorkMem->eye_rect[max_idx].y + (INT32S)(edWorkMem->eye_rect[max_idx].height * 0.6 + 0.5);

		p_eyebrow->x = edWorkMem->eye_rect[max_idx].x + 3;
		p_eyebrow->y = edWorkMem->eye_rect[max_idx].y + (INT32S)(edWorkMem->eye_rect[max_idx].height * 0.16 + 0.5);
		p_eyebrow->width = edWorkMem->eye_rect[max_idx].width - 6;
		p_eyebrow->height = 2;
	}

	edWorkMem->is_get_eye = 1;
Return:
	p_obj_result->is_get_eye = edWorkMem->is_get_eye;
	p_obj_result->is_get_nose = 0;
	p_obj_result->is_get_mouth = 0;
	if(edWorkMem->is_get_eye) {
		// push eye result to obj result
		pos.x = edWorkMem->fe_result.face.x;
		pos.y = edWorkMem->fe_result.face.y;

		// map coordinate of eye to org image
		p_obj_result->eye[0].x = (int)(edWorkMem->fe_result.left_eye.x * scale_factor + 0.5) + pos.x;
		p_obj_result->eye[0].y = (int)(edWorkMem->fe_result.left_eye.y * scale_factor + 0.5) + pos.y;
		p_obj_result->eye[0].width= 4;
		p_obj_result->eye[0].height= 4;

		p_obj_result->eye[1].x = (int)(edWorkMem->fe_result.right_eye.x * scale_factor + 0.5) + pos.x;
		p_obj_result->eye[1].y = (int)(edWorkMem->fe_result.right_eye.y * scale_factor + 0.5) + pos.y;
		p_obj_result->eye[1].width= 4;
		p_obj_result->eye[1].height= 4;

		p_obj_result->eyebrow[0].x = (int)(edWorkMem->fe_result.left_eyebrow.x * scale_factor + 0.5) + pos.x;
		p_obj_result->eyebrow[0].y = (int)(edWorkMem->fe_result.left_eyebrow.y * scale_factor + 0.5) + pos.y;
		p_obj_result->eyebrow[0].width = (int)(edWorkMem->fe_result.left_eyebrow.width * scale_factor + 0.5);
		p_obj_result->eyebrow[0].height= 2;

		p_obj_result->eyebrow[1].x = (int)(edWorkMem->fe_result.right_eyebrow.x * scale_factor + 0.5) + pos.x;
		p_obj_result->eyebrow[1].y = (int)(edWorkMem->fe_result.right_eyebrow.y * scale_factor + 0.5) + pos.y;
		p_obj_result->eyebrow[1].width= (int)(edWorkMem->fe_result.right_eyebrow.width * scale_factor + 0.5);
		p_obj_result->eyebrow[1].height= 2;
	}

	if(edWorkMem->is_get_eye == 0) {
		edWorkMem->no_eye_cnt++;
		if(edWorkMem->no_eye_cnt >= 3) {
			edWorkMem->no_eye_cnt = 3;
			obj_result.is_smile = 0;
			//edWorkMem->post_proc.reset(edWorkMem->post_proc.workmem);
			//mouth_detect_reset(mouthWorkMem);
		}
	} else {
		edWorkMem->no_eye_cnt = 0;
		if(edWorkMem->fe_result.left_eye.y == edWorkMem->fe_result.right_eye.y) {
			edWorkMem->fe_result.eye_angle = 0;
		} else {
			edWorkMem->fe_result.eye_angle = atan2((edWorkMem->fe_result.right_eye.y - edWorkMem->fe_result.left_eye.y),
												(edWorkMem->fe_result.right_eye.x - edWorkMem->fe_result.left_eye.x));
		}

		// mouth detection
        mouth_detect_proc(mouthWorkMem, &edWorkMem->face_img, &edWorkMem->fe_result);

        //get mouth, noise and smile
        mouth_detect_get_result(mouthWorkMem, p_obj_result);
        leye = &edWorkMem->fe_result.left_eye;
        angle = -edWorkMem->fe_result.eye_angle;

        //nose
        if(p_obj_result->is_get_nose) {
            for(i=0; i<2; i++) {
                r = &p_obj_result->nose[i];
                x_dif = (double)(r->x - leye->x);
                y_dif = (double)(r->y - leye->y);
                x = (INT32S)(x_dif * cos(angle) + y_dif * sin(angle) + 0.5) + leye->x;
                y = (INT32S)(y_dif * cos(angle) - x_dif * sin(angle) + 0.5) + leye->y;
                r->x = (x * scale_factor + 0.5) + pos.x;
                r->y = (y * scale_factor + 0.5) + pos.y;
            }
        }

        //mouth
        if(p_obj_result->is_get_mouth) {
            for(i=0; i<4; i++) {
                r = &p_obj_result->mouth[i];
                x_dif = (double)(r->x - leye->x);
                y_dif = (double)(r->y - leye->y);
                x = (INT32S)(x_dif * cos(angle) + y_dif * sin(angle) + 0.5) + leye->x;
                y = (INT32S)(y_dif * cos(angle) - x_dif * sin(angle) + 0.5) + leye->y;
                r->x = (x * scale_factor + 0.5) + pos.x;
                r->y = (y * scale_factor + 0.5) + pos.y;
            }
        }
	}
}
#endif

#if FACE_RECOGNIZE_MEUN_EN == 1
static INT32U face_recognize_get_mode(INT32U mode)
{
	INT32U temp;

	drv_fd_result_lock();
	switch(mode)
	{
		case FD_TRAINING_STATE_GET:
			temp = fdWorkMem->training_state;
			break;

		case FD_IDENTIFY_STATE_GET:
			temp = fdWorkMem->identify_state;
			break;

		case FD_SECURITY_LEVEL_GET:
			temp = fdWorkMem->security_level;
			break;

		case FD_TRAINING_CNT_GET:
			if(fdWorkMem->id_mode == 1 || fdWorkMem->training_state)
			{
				if(fdWorkMem->training_state)
				{
                    #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                        temp = fidInfo.MAX_SAMPLE_NUM;
                    #else
                        temp = FD_TRAINING_IMAGE_NUMBER;
                    #endif
				}
				else
					temp = fdWorkMem->training_cnt;
			}
			else
				temp = 0;
			break;

		case FD_IDENTIFY_SCORE_GET:
			temp = fdWorkMem->recognize_score;
			break;
#if 0
		case FD_ULPB_UPDATE_STATE_GET:
			temp = idWorkMem->ULBP_update_state;
			break;

		case FD_ULPB_UPDATE_CNT_GET:
			temp = idWorkMem->ULBP_update_cnt;
			break;
#endif
		default:
			temp = fdWorkMem->id_mode;
			break;
	}
    drv_fd_result_unlock();

	return temp;
}

static INT32S face_training_mode_update(INT32U en)
{
	SpN_ptr sp_ptr;
	INT32U i,sp_num_addr;

    if(face_recognize_get_mode(FACE_MODE_GET) == FACE_TRAINING_MODE)
    {
		//get sprite character number of image and sprite start ptr of sprite ram
		Get_sprite_image_info(C_JPG_SAVE_NUMBER_1,(SpN_ptr *)&sp_ptr);
		sp_num_addr=sp_ptr.nSPNum_ptr;
		for(i=0;i<sp_ptr.nSP_CharNum;i++)
		{
			gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
			sp_num_addr+=sizeof(SpN_RAM);
		}
		//get sprite character number of image and sprite start ptr of sprite ram
		Get_sprite_image_info(C_JPG_SAVE_NUMBER_2,(SpN_ptr *)&sp_ptr);
		sp_num_addr=sp_ptr.nSPNum_ptr;
		for(i=0;i<sp_ptr.nSP_CharNum;i++)
		{
			gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
			sp_num_addr+=sizeof(SpN_RAM);
		}
		//get sprite character number of image and sprite start ptr of sprite ram
		Get_sprite_image_info(C_JPG_SAVE_NUMBER_3,(SpN_ptr *)&sp_ptr);
		sp_num_addr=sp_ptr.nSPNum_ptr;
		for(i=0;i<sp_ptr.nSP_CharNum;i++)
		{
			gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
			sp_num_addr+=sizeof(SpN_RAM);
		}
#if 0
		//get sprite character number of image and sprite start ptr of sprite ram
		Get_sprite_image_info(C_JPG_SAVE_ICON,(SpN_ptr *)&sp_ptr);
		sp_num_addr=sp_ptr.nSPNum_ptr;
		for(i=0;i<sp_ptr.nSP_CharNum;i++)
		{
			gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
			sp_num_addr+=sizeof(SpN_RAM);

		}
#endif
	}

	return STATUS_OK;
}

static INT32S face_training_mode(INT32U image_num)
{
    INT32U temp,temp1;

    if(face_recognize_get_mode(FACE_MODE_GET) == FACE_TRAINING_MODE)
    {
	    // image and position init
	    set_sprite_init(C_JPG_SAVE_NUMBER_1,(INT32U)&Sprite00001_SP);
	    set_sprite_init(C_JPG_SAVE_NUMBER_2,(INT32U)&Sprite00001_SP);
	    set_sprite_init(C_JPG_SAVE_NUMBER_3,(INT32U)&Sprite00001_SP);
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
	    set_sprite_init(C_FACE_TRAIN_SUBJECT,(INT32U)&Sprite00001_SP);
#endif
	    set_sprite_init(C_JPG_SAVE_ICON ,(INT32U)&Sprite0030_SP);
	 	//set_sprite_display_init(C_JPG_SAVE_ICON,184,0,(INT32U)_Sprite0003_IMG0000_CellIdx);

        if(fd_mode == 0)
        {
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
            set_sprite_display_init(C_FACE_TRAIN_SUBJECT,180,16,0);
#endif
            set_sprite_display_init(C_JPG_SAVE_NUMBER_3,240,16,0);
            set_sprite_display_init(C_JPG_SAVE_NUMBER_2,264,16,0);
            set_sprite_display_init(C_JPG_SAVE_NUMBER_1,288,16,0);
        }
        else
        {
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
            //temp = (fdWorkMem->training_subject_cnt + 1);
            temp = fdWorkMem->training_subject_cnt;
            set_sprite_display_init(C_FACE_TRAIN_SUBJECT,180,16,(INT32U)FACE_SP_NUMBER_POOL[temp]);
#endif
            temp = (image_num / 100);
            set_sprite_display_init(C_JPG_SAVE_NUMBER_3,240,16,(INT32U)FACE_SP_NUMBER_POOL[temp]);
            temp1 = temp * 100;
            temp = (image_num-temp1);
            temp1 = (temp / 10);
            set_sprite_display_init(C_JPG_SAVE_NUMBER_2,264,16,(INT32U)FACE_SP_NUMBER_POOL[temp1]);
            temp1 = (temp % 10);
            set_sprite_display_init(C_JPG_SAVE_NUMBER_1,288,16,(INT32U)FACE_SP_NUMBER_POOL[temp1]);
		}
    }
    else
    {
	    // image and position init
	    set_sprite_init(C_JPG_SAVE_NUMBER_1,(INT32U)&Sprite00001_SP);
	    set_sprite_init(C_JPG_SAVE_NUMBER_2,(INT32U)&Sprite00001_SP);
	    set_sprite_init(C_JPG_SAVE_NUMBER_3,(INT32U)&Sprite00001_SP);
	    set_sprite_init(C_JPG_SAVE_ICON ,(INT32U)&Sprite0030_SP);
	 	//set_sprite_display_init(C_JPG_SAVE_ICON,184,0,0);
		set_sprite_display_init(C_JPG_SAVE_NUMBER_3,240,16,0);
		set_sprite_display_init(C_JPG_SAVE_NUMBER_2,264,16,0);
		set_sprite_display_init(C_JPG_SAVE_NUMBER_1,288,16,0);
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
        if(fdWorkMem->training_subject_cnt > 0)
        {
            temp = fdWorkMem->training_subject_cnt;
            set_sprite_display_init(C_FACE_TRAIN_SUBJECT,180,16,(INT32U)FACE_SP_NUMBER_POOL[temp]);
        }
#endif
    }

    return STATUS_OK;
}

static INT32S face_demo_icon_mode_update(INT32U en)
{
	SpN_ptr sp_ptr;
	INT32U i,sp_num_addr,state;

	//get sprite character number of image and sprite start ptr of sprite ram
	Get_sprite_image_info(C_FACE_DEMO_STATE,(SpN_ptr *)&sp_ptr);
	sp_num_addr=sp_ptr.nSPNum_ptr;
	for(i=0;i<sp_ptr.nSP_CharNum;i++)
	{
		gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
		sp_num_addr+=sizeof(SpN_RAM);
	}

	state = face_recognize_get_mode(FACE_MODE_GET);
	if(state != FACE_TRAINING_MODE)
	{
		state = face_recognize_get_mode(FACE_IDENTIFY_RESULT_GET);
		if(state)
		{
			//if(face_recognize_en)
			if(1)
			{
                //get sprite character number of image and sprite start ptr of sprite ram
                Get_sprite_image_info(C_FACE_IDENTIFY_END,(SpN_ptr *)&sp_ptr);
                sp_num_addr=sp_ptr.nSPNum_ptr;
                for(i=0;i<sp_ptr.nSP_CharNum;i++)
                {
                    gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
                    sp_num_addr+=sizeof(SpN_RAM);
                }
			}

			//get sprite character number of image and sprite start ptr of sprite ram
			Get_sprite_image_info(C_JPG_SAVE_NUMBER_2,(SpN_ptr *)&sp_ptr);
			sp_num_addr=sp_ptr.nSPNum_ptr;
			for(i=0;i<sp_ptr.nSP_CharNum;i++)
			{
				gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
				sp_num_addr+=sizeof(SpN_RAM);
			}
		}
	}

#if 0
	state = face_recognize_get_mode(FACE_MODE_GET);
	if(state == FACE_ULBP_UPDATE_MODE)
	{
		if(face_recognize_get_mode(FD_ULPB_UPDATE_STATE_GET) == 0)
		{
			//get sprite character number of image and sprite start ptr of sprite ram
			Get_sprite_image_info(C_FACE_CHECK_ICON2,(SpN_ptr *)&sp_ptr);
			sp_num_addr=sp_ptr.nSPNum_ptr;
			for(i=0;i<sp_ptr.nSP_CharNum;i++)
			{
				if(face_recognize_get_mode(FD_ULPB_UPDATE_CNT_GET))
				{
					gplib_ppu_sprite_attribute_palette_set((SpN_RAM *)sp_num_addr,0,6);
				}
				else
				{
					gplib_ppu_sprite_attribute_palette_set((SpN_RAM *)sp_num_addr,0,5);
				}
				sp_num_addr+=sizeof(SpN_RAM);
			}
		}
	}
#endif

	//get sprite character number of image and sprite start ptr of sprite ram
	Get_sprite_image_info(C_FACE_IDENTIFY_LEVEL,(SpN_ptr *)&sp_ptr);
	sp_num_addr=sp_ptr.nSPNum_ptr;
	for(i=0;i<sp_ptr.nSP_CharNum;i++)
	{
		gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
		sp_num_addr+=sizeof(SpN_RAM);
	}

	//get sprite character number of image and sprite start ptr of sprite ram
	Get_sprite_image_info(C_FACE_IDENTIFY_ICON1,(SpN_ptr *)&sp_ptr);
	sp_num_addr=sp_ptr.nSPNum_ptr;
	for(i=0;i<sp_ptr.nSP_CharNum;i++)
	{
		gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
		sp_num_addr+=sizeof(SpN_RAM);
	}

	//get sprite character number of image and sprite start ptr of sprite ram
	Get_sprite_image_info(C_FACE_IDENTIFY_ICON2,(SpN_ptr *)&sp_ptr);
	sp_num_addr=sp_ptr.nSPNum_ptr;
	for(i=0;i<sp_ptr.nSP_CharNum;i++)
	{
		gplib_ppu_sprite_attribute_blend64_set((SpN_RAM *)sp_num_addr,1,en);
		sp_num_addr+=sizeof(SpN_RAM);
	}

	return STATUS_OK;
}

static INT32S face_demo_icon_mode(INT32U mode)
{
	INT32U state, temp;

	if(mode == FACE_TRAINING_MODE)
	{
		set_sprite_init(C_FACE_DEMO_STATE ,(INT32U)&Sprite005_SP);
	    if(fd_mode == 0)
            set_sprite_display_init(C_FACE_DEMO_STATE,0,0,(INT32U)_Sprite005_IMG0000_CellIdx);
	    else
            set_sprite_display_init(C_FACE_DEMO_STATE,0,0,(INT32U)_Sprite005_IMG0002_CellIdx);
	}
	else if(mode == FACE_IDENTIFY_MODE)
	{
		set_sprite_init(C_FACE_DEMO_STATE ,(INT32U)&Sprite005_SP);
	    if(fd_mode == 0)
            set_sprite_display_init(C_FACE_DEMO_STATE,0,0,(INT32U)_Sprite005_IMG0000_CellIdx);
	    else
            set_sprite_display_init(C_FACE_DEMO_STATE,0,0,(INT32U)_Sprite005_IMG0001_CellIdx);
	}
	else
	{
		set_sprite_init(C_FACE_DEMO_STATE ,(INT32U)&Sprite005_SP);
	    if(fd_mode == 1)
            set_sprite_display_init(C_FACE_DEMO_STATE,0,0,(INT32U)_Sprite005_IMG0002_CellIdx);
	    else if (fd_mode == 2)
            set_sprite_display_init(C_FACE_DEMO_STATE,0,0,(INT32U)_Sprite005_IMG0001_CellIdx);
	    else
            set_sprite_display_init(C_FACE_DEMO_STATE,0,0,(INT32U)_Sprite005_IMG0000_CellIdx);
	}

#if 0
	if(mode == ULBP_UPDATE_MODE)
	{
		state = face_recognize_get_mode(FD_ULPB_UPDATE_STATE_GET);
		if(state == 0)
		{
			set_sprite_display_init(C_FACE_PASSWORD_ICON1,32,120,(INT32U)(INT32U)0);
			set_sprite_display_init(C_FACE_PASSWORD_NUM1,48,137,(INT32U)0);
			set_sprite_display_init(C_FACE_PASSWORD_NUM2,112,137,(INT32U)0);
			set_sprite_display_init(C_FACE_PASSWORD_NUM3,176,137,(INT32U)0);
			set_sprite_display_init(C_FACE_PASSWORD_NUM4,240,137,(INT32U)0);
			set_sprite_init(C_FACE_CHECK_ICON1 ,(INT32U)&Sprite017_SP);
			set_sprite_display_init(C_FACE_CHECK_ICON1,96,24,(INT32U)(INT32U)_Sprite017_IMG0000_CellIdx);
			set_sprite_init(C_FACE_CHECK_ICON2 ,(INT32U)&Sprite015_SP);
			set_sprite_display_init(C_FACE_CHECK_ICON2,32,88,(INT32U)(INT32U)_Sprite015_IMG0000_CellIdx);
		}
		else
		{
			set_sprite_display_init(C_FACE_CHECK_ICON1,96,24,(INT32U)(INT32U)0);
			set_sprite_display_init(C_FACE_CHECK_ICON2,32,88,(INT32U)(INT32U)0);
			set_sprite_init(C_FACE_PASSWORD_ICON1 ,(INT32U)&Sprite014_SP);
			set_sprite_display_init(C_FACE_PASSWORD_ICON1,32,88,(INT32U)(INT32U)_Sprite014_IMG0000_CellIdx);
			if(state == 1)
			{
				set_sprite_init(C_FACE_PASSWORD_NUM1 ,(INT32U)&Sprite00001_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM1,48,105,(INT32U)FACE_SP_NUMBER_POOL[face_recognize_get_mode(FD_ULPB_UPDATE_CNT_GET)]);
			}
			else if(state == 2)
			{
				set_sprite_init(C_FACE_PASSWORD_NUM1 ,(INT32U)&Sprite016_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM1,48,105,(INT32U)_Sprite016_IMG0000_CellIdx);
				set_sprite_init(C_FACE_PASSWORD_NUM2 ,(INT32U)&Sprite00001_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM2,112,105,(INT32U)FACE_SP_NUMBER_POOL[face_recognize_get_mode(FD_ULPB_UPDATE_CNT_GET)]);
			}
			else if(state == 3)
			{
				set_sprite_init(C_FACE_PASSWORD_NUM1 ,(INT32U)&Sprite016_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM1,48,105,(INT32U)_Sprite016_IMG0000_CellIdx);
				set_sprite_init(C_FACE_PASSWORD_NUM2 ,(INT32U)&Sprite016_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM2,112,105,(INT32U)_Sprite016_IMG0000_CellIdx);
				set_sprite_init(C_FACE_PASSWORD_NUM3 ,(INT32U)&Sprite00001_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM3,176,105,(INT32U)FACE_SP_NUMBER_POOL[face_recognize_get_mode(FD_ULPB_UPDATE_CNT_GET)]);
			}
			else if(state == 4)
			{
				set_sprite_init(C_FACE_PASSWORD_NUM1 ,(INT32U)&Sprite016_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM1,48,105,(INT32U)_Sprite016_IMG0000_CellIdx);
				set_sprite_init(C_FACE_PASSWORD_NUM2 ,(INT32U)&Sprite016_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM2,112,105,(INT32U)_Sprite016_IMG0000_CellIdx);
				set_sprite_init(C_FACE_PASSWORD_NUM3 ,(INT32U)&Sprite016_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM3,176,105,(INT32U)_Sprite016_IMG0000_CellIdx);
				set_sprite_init(C_FACE_PASSWORD_NUM3 ,(INT32U)&Sprite00001_SP);
				set_sprite_display_init(C_FACE_PASSWORD_NUM4,240,105,(INT32U)FACE_SP_NUMBER_POOL[face_recognize_get_mode(FD_ULPB_UPDATE_CNT_GET)]);
			}
		}
	}
	else
#endif
	//{
		set_sprite_display_init(C_FACE_CHECK_ICON1,96,24,(INT32U)(INT32U)0);
		set_sprite_display_init(C_FACE_CHECK_ICON2,32,88,(INT32U)(INT32U)0);
		set_sprite_display_init(C_FACE_PASSWORD_ICON1,32,120,(INT32U)(INT32U)0);
		set_sprite_display_init(C_FACE_PASSWORD_NUM1,48,137,(INT32U)0);
		set_sprite_display_init(C_FACE_PASSWORD_NUM2,112,137,(INT32U)0);
		set_sprite_display_init(C_FACE_PASSWORD_NUM3,176,137,(INT32U)0);
		set_sprite_display_init(C_FACE_PASSWORD_NUM4,240,137,(INT32U)0);
	//}

	if(mode != FACE_TRAINING_MODE)
	{
		state = face_recognize_get_mode(FACE_IDENTIFY_RESULT_GET);
		//if(state && face_recognize_en)
		if(state)
		{
			set_sprite_init(C_FACE_IDENTIFY_END ,(INT32U)&Sprite006_SP);
			set_sprite_display_init(C_FACE_IDENTIFY_END,96,56,(INT32U)_Sprite006_IMG0000_CellIdx);
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
            set_sprite_display_init(C_FACE_TRAIN_SUBJECT,180,16,(INT32U)0);
            temp = (fidInfo.best_subject_index + 1);
            if(temp >= 6)
                set_sprite_display_init(C_FACE_IDENTIFY_RESULT,180,16, 0);
            else
                set_sprite_display_init(C_FACE_IDENTIFY_RESULT,180,16,(INT32U)FACE_SP_NUMBER_POOL[temp]);
#endif
		}
		else
		{
		    set_sprite_init(C_FACE_IDENTIFY_END ,(INT32U)&Sprite006_SP);
		 	set_sprite_display_init(C_FACE_IDENTIFY_END,96,56,(INT32U)0);
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
            set_sprite_display_init(C_FACE_IDENTIFY_RESULT,180,16,(INT32U)0);
#endif
		}
    }
    else
    {
	    set_sprite_init(C_FACE_IDENTIFY_END ,(INT32U)&Sprite006_SP);
	 	set_sprite_display_init(C_FACE_IDENTIFY_END,96,56,(INT32U)0);
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
	 	set_sprite_display_init(C_FACE_IDENTIFY_RESULT,180,16,(INT32U)0);
#endif
    }

	set_sprite_init(C_FACE_IDENTIFY_LEVEL ,(INT32U)&Sprite00001_SP);
	set_sprite_init(C_FACE_IDENTIFY_ICON1 ,(INT32U)&Sprite005_SP);
	set_sprite_init(C_FACE_IDENTIFY_ICON2 ,(INT32U)&Sprite005_SP);
    set_sprite_display_init(C_FACE_IDENTIFY_LEVEL,120,206,(INT32U)FACE_SP_NUMBER_POOL[face_recognize_get_mode(FACE_SECURITY_LEVEL_GET)]);
    set_sprite_display_init(C_FACE_IDENTIFY_ICON1,0,198,(INT32U)(INT32U)_Sprite005_IMG0004_CellIdx);
    set_sprite_display_init(C_FACE_IDENTIFY_ICON2,192,198,(INT32U)(INT32U)_Sprite005_IMG0005_CellIdx);

	return 0;
}

static INT32S face_recognize_sprite_update(void)
{
	face_training_mode(face_recognize_get_mode(FACE_TRAINING_COUNTER_GET));
	face_demo_icon_mode(face_recognize_get_mode(FACE_MODE_GET));
	paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_320X240,LeftTop2Center_coordinate,20);
	face_training_mode_update(32);
	face_demo_icon_mode_update(32);

	return 0;
}
#endif

static INT32U fd_scaler_step_get(void)
{
    return (scaler_int_w_step - 1);
}

static INT32S fd_detect_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U w, INT16U h, INT32U infmt)
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

	pInput->ScaleOut.width = w;
	pInput->ScaleOut.height = h;
	if(infmt == IMG_FMT_GRAY) {
		pInput->ScaleOut.ch  = 1;
		pInput->ScaleOut.widthStep = w;
	} else {
		pInput->ScaleOut.ch  = 2;
		pInput->ScaleOut.widthStep = w * 2;
	}
	pInput->ScaleOut.format = infmt;
	pInput->ScaleOut.ptr = (INT8U *)frame_buffer;

	// scale
	scalerStart_no_fd(&pInput->ScaleIn, &pInput->ScaleOut);
	scalerEnd();

	nRet = STATUS_OK;
Return:
	return nRet;
}

static INT32S fd_display_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
#if DISPLAY_USE_PSCALER_EN == 1
	INT32S retStatus;

	if(frame_buffer == 0) {
		return -1;
	}

    drv_l1_pscaler_init(POST_SCALE_USE);
    drv_l1_pscaler_clk_ctrl(POST_SCALE_USE,1);
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
#else
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
	scalerStart_dispaly(&pInput->ScaleIn, &pInput->ScaleOut);
	scalerEnd();

	nRet = STATUS_OK;
Return:
	return nRet;
#endif
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
	scalerStart_dispaly(&pInput->ScaleIn, &pInput->ScaleOut);
	scalerEnd();

	nRet = STATUS_OK;
Return:
	return nRet;
}

#if FACE_RECOGNIZE_MEUN_EN == 1
static int faceRoiDetect(const gpImage* gray, gpRect* userROI, int mode, int img_num)
{
	#define FD_MAX_RESULT 		OBJDETECT_MAX_RESULT
    CHAR face_path[256];
	INT16S fd;
    // detect type setting
	ClassifyData clfface;
	ClassifyData clfreye;
	ClassifyData clfleye;
	// best face reye and leye
	gpRect* faceROI = &userROI[0];
	gpRect* lEyeROI = &userROI[1];
	gpRect* rEyeROI = &userROI[2];
	// detect result and count
	gpRect Rect,rFace,lFace;
	gpRect faceResult[FD_MAX_RESULT] = {0};
	gpRect rEyeResult[FD_MAX_RESULT] = {0};
	gpRect lEyeResult[FD_MAX_RESULT] = {0};
	int faceCount[FD_MAX_RESULT] = {0};
	int rEyeCount[FD_MAX_RESULT] = {0};
	int lEyeCount[FD_MAX_RESULT] = {0};
	// face detect workmem
	void *WorkMem = 0;
	// Initialization //
	int i, maxFaceW, maxFaceCount, best_face, offset_x, offset_y, min_eye_wnd, max_eye_wnd;
	int ret,faceN, rEyeN, lEyeN, t, int_scale_face, int_scale_eye, detectH, fd_state;
	// xstep
	int xstep_face = 2;
	int xstep_eye = 2;
	// ystep
	int ystep_face = 2;
	int ystep_eye = 2;
	// face nbr
	int min_face_nbr_h = 5;
	int min_face_nbr_l = 2;
	// eye nbr
	int min_eye_nbr = 1;
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

    #if FD_ONE_FRAME_DEBUG_MODE_EN == 1
        int_scale_eye = 68811;//68811;//72089; // 1.1, 65535 = 1
    #else
        int_scale_eye = 72089;//68811;//72089; // 1.1, 65535 = 1
    #endif

    t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, FD_MAX_RESULT, xstep_face, ystep_face);
    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t,4);
        fd_workmem_size = t;
        WorkMem = (void *)fd_workmem;
        DBG_PRINT("FD_WorkMem:0x%X \r\n",WorkMem);
    }
    else
        WorkMem = (void *)fd_workmem;

    gp_memset((INT8S *)WorkMem,0, t);
    gp_memset((INT8S *)&userROI[0],0, 3*sizeof(gpRect));

    FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, FD_MAX_RESULT, xstep_face, ystep_face);
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
	fd_type = OBJ_FACE;
	FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
	p_result = faceResult;
	max_result_t = FD_MAX_RESULT;
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

    #if FD_NEW_PROC_EN == 1
        int_scale_face = 74054;//74054;//72089; // 1.1
        int_scale_eye = 68811;//68811;//72089; // 1.1, 65535 = 1
    #else
        int_scale_face = 72089;//74054;//72089; // 1.1
        int_scale_eye = 72089;//68811;//72089; // 1.1, 65535 = 1
    #endif

	t = MIN(Rect.width, Rect.height);
	if(max_wnd>t) max_wnd = t;

	t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, FD_MAX_RESULT, xstep_face, ystep_face);

    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t,4);
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
	ret = FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, FD_MAX_RESULT, xstep_face, ystep_face);

	/* setting cascade type (face) */
	clfface.obj_type = OBJ_FACE;
	fd_type = OBJ_FACE;
    FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);

	FaceDetect_SetX(WorkMem, xstep_face, 0);
	faceN = FaceDetect(WorkMem, gray, &Rect, FD_MAX_RESULT, faceResult, faceCount);

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

	//--------------------
	//	Eyes Detection
	//--------------------
	min_eye_wnd = MAX(faceROI->width/7, 20); // 24 is minimum
	//Eyes detect range
	detectH = (int)(faceResult[best_face].height*0.6);
    max_eye_wnd = MIN(detectH, (faceResult[best_face].width>>1));

	ret = FaceDetect_Config(WorkMem, t, (faceResult[best_face].width>>1), detectH, 1, FD_MAX_RESULT, xstep_eye, ystep_eye);

	// setting cascade type (right eye) //
	clfreye.obj_type = OBJ_REYE;
    fd_type = OBJ_REYE;
	FaceDetect_set_detect_obj(WorkMem, &clfreye);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_eye, min_eye_wnd, max_eye_wnd, mode);

	rFace = GPRect_utils(faceResult[best_face].x + (faceResult[best_face].width>>1), faceResult[best_face].y, (faceResult[best_face].width>>1), detectH);
	FaceDetect_SetX(WorkMem, xstep_eye, 0);

	rEyeN = FaceDetect(WorkMem, gray, &rFace, FD_MAX_RESULT, rEyeResult, rEyeCount);

	if (!rEyeN)
	{
        // release memory //
        return 0;
	}
	else
        fd_state = 1;

	// setting cascade type (left eye) //
	clfleye.obj_type = OBJ_LEYE;
    fd_type = OBJ_LEYE;
	FaceDetect_set_detect_obj(WorkMem, &clfleye);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_eye, min_eye_wnd, max_eye_wnd, mode);

	lFace = GPRect_utils(faceResult[best_face].x, faceResult[best_face].y, (faceResult[best_face].width>>1), detectH);
	FaceDetect_SetX(WorkMem, xstep_eye, 0);

	lEyeN = FaceDetect(WorkMem, gray, &lFace, FD_MAX_RESULT, lEyeResult, lEyeCount);

	if (!lEyeN)
	{
        // release memory //
        return 0;
	}
	else
        fd_state = 1;

	if (!rEyeN)
		rEyeROI->width = 0;
	else
	{
		int minEyeDist = INT_MAX;
		int maxEyeCount = min_eye_nbr - 1;
		int most_possible_eye = 0;

		int i = rEyeN - 1;
		do
		{
			if (rEyeCount[i] > maxEyeCount)
			{
				maxEyeCount = rEyeCount[i];
				most_possible_eye = i;
				minEyeDist = ABS(rEyeResult[i].x + rEyeResult[i].width/2 - faceROI->x - faceROI->width*3/4) + ABS(rEyeResult[i].y + rEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
			}
			else if (rEyeCount[i] == maxEyeCount)
			{
				int evaluateDist
					= ABS(rEyeResult[i].x + rEyeResult[i].width/2 - faceROI->x - faceROI->width*3/4) + ABS(rEyeResult[i].y + rEyeResult[i].height/2 - faceROI->y - faceROI->height/4);

				if (evaluateDist < minEyeDist)
				{
					minEyeDist = evaluateDist;
					most_possible_eye = i;
				}
			}
		} while (i--);

		// Face Position Determination //
		*rEyeROI = rEyeResult[most_possible_eye];
	}

	if (!lEyeN)
		lEyeROI->width = 0;
	else
	{
		int minEyeDist = INT_MAX;
		int maxEyeCount = min_eye_nbr - 1;
		int most_possible_eye = 0;

		int i = lEyeN - 1;
		do
		{
			if (lEyeCount[i] > maxEyeCount)
			{
				maxEyeCount = lEyeCount[i];
				most_possible_eye = i;
				minEyeDist = ABS(lEyeResult[i].x + lEyeResult[i].width/2 - faceROI->x - faceROI->width/4) + ABS(lEyeResult[i].y + lEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
			}
			else if (lEyeCount[i] == maxEyeCount)
			{
				int evaluateDist
					= ABS(lEyeResult[i].x + lEyeResult[i].width/2 - faceROI->x - faceROI->width/4) + ABS(lEyeResult[i].y + lEyeResult[i].height/2 - faceROI->y - faceROI->height/4);

				if (evaluateDist < minEyeDist)
				{
					minEyeDist = evaluateDist;
					most_possible_eye = i;
				}
			}
		} while (i--);


		// Face Position Determination //
		*lEyeROI = lEyeResult[most_possible_eye];
	}

	if(!lEyeROI->width || !rEyeROI->width)
	{
        // release memory //
        return 0;
    }
	else
        fd_state = 1;

	#if FD_DEBUG_EN == 1
        #if 1
            DBG_PRINT("face: x=%d, y=%d, w=%d, h=%d\r\n",faceROI->x,faceROI->y,faceROI->width,faceROI->height);
            DBG_PRINT("reye: x=%d, y=%d, w=%d, h=%d\r\n",rEyeROI->x,rEyeROI->y,rEyeROI->width,rEyeROI->height);
            DBG_PRINT("leye: x=%d, y=%d, w=%d, h=%d\r\n",lEyeROI->x,lEyeROI->y,lEyeROI->width,lEyeROI->height);
            DBG_PRINT("===========================\r\n");
        #elif 0
            DBG_PRINT("lEyeROI.x = %d\r\n", lEyeROI->x);
            DBG_PRINT("lEyeROI.y = %d\r\n", lEyeROI->y);
            DBG_PRINT("lEyeROI.width = %d\r\n", lEyeROI->width);
            DBG_PRINT("lEyeROI.height = %d\r\n", lEyeROI->height);
            DBG_PRINT("rEyeROI.x = %d\r\n", rEyeROI->x);
            DBG_PRINT("rEyeROI.y = %d\r\n", rEyeROI->y);
            DBG_PRINT("rEyeROI.width = %d\r\n", rEyeROI->width);
            DBG_PRINT("rEyeROI.height = %d\r\n", rEyeROI->height);
            DBG_PRINT("*****************************************************************\r\n");
        #endif
    #endif
#if 0
	for(i=0; i<2; i++) {
		if(i == 0)
		{
			obj_result.eye[i].x = rEyeROI->x;
			obj_result.eye[i].y = rEyeROI->y;
			obj_result.eye[i].width = rEyeROI->width;
			obj_result.eye[i].height = rEyeROI->height;
		}
		else
		{
			obj_result.eye[i].x = lEyeROI->x;
			obj_result.eye[i].y = lEyeROI->y;
			obj_result.eye[i].width = lEyeROI->width;
			obj_result.eye[i].height = lEyeROI->height;
		}
	}
#endif

	if (((lEyeROI->y + lEyeROI->height/2) < faceROI->y) || ((rEyeROI->y + rEyeROI->height/2) < faceROI->y))
		DBG_PRINT("Error: unreasonable eye information\r\n");

	if ((rEyeROI->width) && (lEyeROI->width))
	{
		double x_ratio = 0;
		double lower_ratio = 1.85;
		double upper_ratio = 0.5;
		int eyedist = (rEyeROI->x + rEyeROI->width/2 - lEyeROI->x - lEyeROI->width/2);
		int lowerbound = (int)MIN(((rEyeROI->y + lEyeROI->y + (rEyeROI->height + lEyeROI->height)/2)/2 + lower_ratio*eyedist), HEIGHT - 1);
		int upperbound = (int)MAX(((rEyeROI->y + lEyeROI->y + (rEyeROI->height + lEyeROI->height)/2)/2 - eyedist*upper_ratio), 0);

		faceROI->y = upperbound;
		faceROI->height = lowerbound - upperbound;
	}

    return fd_state;
}
#else
static int face_Detect_only(const gpImage* gray, gpRect* userROI, int mode, int img_num)
{
	#define FD_MAX_RESULT 		    OBJDETECT_MAX_RESULT
    CHAR face_path[256];
	INT16S fd;
    // detect type setting
	ClassifyData clfface;
	// best face reye and leye
	gpRect* faceROI = &userROI[0];
	// detect result and count
	gpRect Rect,rFace,lFace;
	gpRect faceResult[FD_MAX_RESULT] = {0};
	int faceCount[FD_MAX_RESULT] = {0};

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
	int min_face_nbr_h = 12;//5;
	int min_face_nbr_l = 6;//2;
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

    t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, FD_MAX_RESULT, xstep_face, ystep_face);
    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t,4);
        fd_workmem_size = t;
        WorkMem = (void *)fd_workmem;
        DBG_PRINT("FD_WorkMem:0x%X \r\n",WorkMem);
    }
    else
        WorkMem = (void *)fd_workmem;

    gp_memset((INT8S *)WorkMem,0, fd_workmem_size);
    gp_memset((INT8S *)&userROI[0],0, 3*sizeof(gpRect));

    FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, FD_MAX_RESULT, xstep_face, ystep_face);
	//get tracking info
	p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);
	search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);

#if FL_EN == 1
    min_face_wnd = MAX(min_face_wnd, (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem));
#else
	min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
#endif
	i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);

	//set face detect object type
    clfface.obj_type = OBJ_FACE;
	fd_type = OBJ_FACE;
	FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
	p_result = faceResult;
	max_result_t = FD_MAX_RESULT;
	p_count = faceCount;
	nCnt = 0;
	i = search_cnt;
	do {
		INT32S cnt;

#if FL_EN == 0
		min_face_wnd = *p_min_width++;
#endif
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
            if ((maxFaceCount >=  min_face_nbr_h) && (faceCount[i] >= min_face_nbr_h))
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
    track_run_with_srach_cnt(ObjWorkMem_ptr->obj_track_WorkMem, search_cnt, 25);

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

	t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, FD_MAX_RESULT, xstep_face, ystep_face);

    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t,4);
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
	ret = FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, FD_MAX_RESULT, xstep_face, ystep_face);

	/* setting cascade type (face) */
	clfface.obj_type = OBJ_FACE;
	fd_type = OBJ_FACE;
    FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);

	FaceDetect_SetX(WorkMem, xstep_face, 0);

	faceN = FaceDetect(WorkMem, gray, &Rect, FD_MAX_RESULT, faceResult, faceCount);

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
#if FL_EN == 1
    //DBG_PRINT("face roi for landmark\r\n");
    faceROI->x = faceResult[best_face].x;
	faceROI->y = faceResult[best_face].y;
	faceROI->width = faceResult[best_face].width;
	faceROI->height = faceResult[best_face].height;
#else
	offset_x = faceResult[best_face].width * 0.16;
	offset_y = faceResult[best_face].height/7;

	faceROI->x = faceResult[best_face].x + offset_x;
	faceROI->y = faceResult[best_face].y + offset_y;
	faceROI->width = (short)(faceResult[best_face].width - (offset_x<<1));
	faceROI->height = (short)(faceResult[best_face].height - offset_y*1.5);
#endif

	#if FD_DEBUG_EN == 1
		DBG_PRINT("faceROI.x = %d\r\n", faceROI->x);
		DBG_PRINT("faceROI.y = %d\r\n", faceROI->y);
		DBG_PRINT("faceROI.width = %d\r\n", faceROI->width);
		DBG_PRINT("faceROI.height = %d\r\n", faceROI->height);
    #endif

    return fd_state;
}

#if FL_EN == 1
static int face_Detect_Two(const gpImage* gray, gpRect* userROI, int mode, int img_num)
{
	#define FD_MAX_RESULT 		    OBJDETECT_MAX_RESULT
    CHAR face_path[256];
	INT16S fd;
    // detect type setting
	ClassifyData clfface;
	// best face reye and leye
	gpRect* faceROI = userROI;
	// detect result and count
	gpRect Rect,rFace,lFace;
	gpRect faceResult[FD_MAX_RESULT] = {0};
	int faceCount[FD_MAX_RESULT] = {0};

	// face detect workmem
	void *WorkMem = 0;
	// Initialization //
	int i, maxFaceW, maxFaceW2, best_face, best_face2, offset_x, offset_y, min_eye_wnd, max_eye_wnd;
	int ret,faceN, rEyeN, lEyeN, t, int_scale_face, int_scale_eye, detectH, fd_state;
	// xstep
	int xstep_face = 2;
	// ystep
	int ystep_face = 2;
	// face nbr
	int min_face_nbr_h = 12;//5;
	int min_face_nbr_l = 6; //2
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

    t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, FD_MAX_RESULT, xstep_face, ystep_face);
    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t,4);
        fd_workmem_size = t;
        WorkMem = (void *)fd_workmem;
        DBG_PRINT("FD_WorkMem:0x%X \r\n",WorkMem);
    }
    else
        WorkMem = (void *)fd_workmem;

    gp_memset((INT8S *)WorkMem,0, fd_workmem_size);
    gp_memset((INT8S *)&userROI[0],0, 3*sizeof(gpRect));

    FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, FD_MAX_RESULT, xstep_face, ystep_face);
	//get tracking info
	p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);
	search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);

    //::FL_EN::min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
    min_face_wnd = MAX(min_face_wnd, (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem));

	i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);

	//set face detect object type
    clfface.obj_type = OBJ_FACE;
	fd_type = OBJ_FACE;
	FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
	p_result = faceResult;
	max_result_t = FD_MAX_RESULT;
	p_count = faceCount;
	nCnt = 0;
	i = search_cnt;
	do {
		INT32S cnt;

		//::FL_EN::min_face_wnd = *p_min_width++;

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

	maxFaceW = maxFaceW2 = 0;
	best_face = best_face2 = 0;
	search_cnt = fd_state = 0;
    for (i=0; i<nCnt; i++) {
        if (faceCount[i] >= min_face_nbr_l)
        {
            p_obj_rect->x = faceResult[i].x;
            p_obj_rect->y = faceResult[i].y;
            p_obj_rect->width = faceResult[i].width;
            p_obj_rect->height = faceResult[i].height;
            p_obj_rect++;
            search_cnt++;
            fd_state++;

            if (faceResult[i].width > maxFaceW)
            {
                best_face = i;
                maxFaceW = faceResult[i].width;
            }
            else if (faceResult[i].width > maxFaceW2)
            {
                best_face2 = i;
                maxFaceW2 = faceResult[i].width;
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
    track_run_with_srach_cnt(ObjWorkMem_ptr->obj_track_WorkMem, search_cnt, 10);

    #if FD_DEBUG_EN == 1
        DBG_PRINT("\r\n****************************ROI******************************\r\n");
        DBG_PRINT("best_face.count = %d\r\n",faceCount[best_face]);
        DBG_PRINT("best_face.x = %d\r\n", faceResult[best_face].x);
        DBG_PRINT("best_face.y = %d\r\n", faceResult[best_face].y);
        DBG_PRINT("best_face.width = %d\r\n", faceResult[best_face].width);
        DBG_PRINT("best_face.height = %d\r\n", faceResult[best_face].height);
    #endif
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

	t = FaceDetect_Config(0, 0, WIDTH, HEIGHT, 1, FD_MAX_RESULT, xstep_face, ystep_face);

    if(fd_workmem == 0)
    {
        fd_workmem = (INT32U)gp_malloc_align(t,4);
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
	ret = FaceDetect_Config(WorkMem, t, Rect.width, Rect.height, 1, FD_MAX_RESULT, xstep_face, ystep_face);

	/* setting cascade type (face) */
	clfface.obj_type = OBJ_FACE;
	fd_type = OBJ_FACE;
    FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);

	FaceDetect_SetX(WorkMem, xstep_face, 0);

	faceN = FaceDetect(WorkMem, gray, &Rect, FD_MAX_RESULT, faceResult, faceCount);

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
#endif

	/* Face Position Determination */
	if (fd_state == 0)
	{
        return 0;
	}
    else if (fd_state == 1)
    {
        faceROI[0] = faceResult[best_face];
    }
    else if (fd_state > 1)
    {
        gpRect faceTmp0 = faceResult[best_face];
        gpRect faceTmp1 = faceResult[best_face2];

        faceROI[0] = faceTmp0;
        faceROI[1] = faceTmp1;

        DBG_PRINT("0x=%d, 1x=%d\n", faceROI[0].x, faceROI[1].x);
    }

    return fd_state;
}
#endif

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

static void draw_obj(INT32U img_adr, INT32U bufw, INT32U bufh, INT32S total_obj_cnt, gpRect *obj_result_d, INT32U color_mode)
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

	switch(color_mode)
	{
        case DRAW_RED_EN:
            color = 0xffffffff; // red
            break;

        case DRAW_GREEN_EN:
            color = 0xff4c554c; // green
            break;

        case DRAW_BLUE_EN:
            color = 0x4cff4c55; // blue
            break;

        case DRAW_YELLOW_EN:
            color = 0xd211d290;// yellow;
            break;

        case DRAW_BLACK_EN:
            color = 0x10801080;// black;
            break;

        default:
            color = color_mode;
            break;
	}

	for(i=0; i<total_obj_cnt; i++) {
		if((obj_result_d[i].x+obj_result_d[i].width < Image.width) && (obj_result_d[i].x >= 0)
		&& (obj_result_d[i].y+obj_result_d[i].height < Image.height) && (obj_result_d[i].y >= 0))
            draw_rect(&Image, &obj_result_d[i], color);
	}
}

#if FL_EN == 1

static void draw_fl(INT32U img_adr, INT32U bufw, INT32U bufh, INT32U landmark_color, INT32U glasses_color, gpPoint* flandmarks_qvga_ptr, gpPoint* glasses_qvga_ptr)
{
	INT32S i, pi;
	gpImage image;

    image.width = bufw;					// depends on display buffer width
	image.height = bufh;				// depends on display buffer height
	image.widthStep = bufw*2; 			// depends on display format
	image.ch = 2; 						// depends on display format
	image.format = IMG_FMT_UYVY;  		// depends on display format
	image.ptr = (INT8U *)img_adr;

    // drawing
    INT32U ptSize = 2;
    INT32S xp, yp;
    INT16U *img = (INT16U *) image.ptr;
    INT32U *p_img;
	INT32S addr, addr_align;
	INT32S x, y, w, h;


	//draw_pt(&Image, fl_draw, color);
	//draw_flpt(&Image, landmark_color, glasses_color);

    for (pi = 0; pi < landmarksCount; ++pi)
    {
        x = flandmarks_qvga_ptr[pi].x;
        y = flandmarks_qvga_ptr[pi].y;
        for (yp = y - ptSize; yp <= y + ptSize; ++yp)
        {
            for (xp = x - ptSize; xp <= x + ptSize; ++xp)
            {
                addr = (INT32S) (img + ((yp * bufw) + xp));
                addr >>= 2;
                addr <<= 2;
                p_img = (INT32U *) addr;
                *p_img = landmark_color;
            }
        }
    }
    for (pi = 0; pi < glassesCount; ++pi)
    {
        x = glasses_qvga_ptr[pi].x;
        y = glasses_qvga_ptr[pi].y;
        for (yp = y - ptSize; yp <= y + ptSize; ++yp)
        {
            for (xp = x - ptSize; xp <= x + ptSize; ++xp)
            {
                addr = (INT32S) (img + ((yp * bufw) + xp));
                addr >>= 2;
                addr <<= 2;
                p_img = (INT32U *) addr;
                *p_img = glasses_color;
            }
        }
    }
}
/*
static void draw_flpt(gpImage *image, INT32U landmark_color, INT32U glasses_color, gpPoint* flandmarks_qvga_ptr, gpPoint* glasses_qvga_ptr)
{
	INT32S i;
	INT32S pi;
	INT32U *p_img;
	INT32S len;
	INT32S addr, addr_align;
	INT32S x, y, w, h;
	INT16U *img = (INT16U *) image->ptr;
	INT32U ptSize = 2;
	INT32S xp, yp;

    for (pi = 0; pi < landmarksCount; ++pi)
    {
        x = flandmarks_qvga_ptr[pi].x;
        y = flandmarks_qvga_ptr[pi].y;

        for (yp = y - ptSize; yp <= y + ptSize; ++yp)
        {
            for (xp = x - ptSize; xp <= x + ptSize; ++xp)
            {
                addr = (INT32S) (img + ((yp * image->width) + xp));
                addr >>= 2;
                addr <<= 2;
                p_img = (INT32U *) addr;
                *p_img = landmark_color;
            }
        }
    }
    for (pi = 0; pi < glassesCount; ++pi)
    {
        x = glasses_qvga_ptr[pi].x;
        y = glasses_qvga_ptr[pi].y;
        for (yp = y - ptSize; yp <= y + ptSize; ++yp)
        {
            for (xp = x - ptSize; xp <= x + ptSize; ++xp)
            {
                addr = (INT32S) (img + ((yp * image->width) + xp));
                addr >>= 2;
                addr <<= 2;
                p_img = (INT32U *) addr;
                *p_img = glasses_color;
            }
        }
    }
}

static void draw_fl(INT32U img_adr, INT32U bufw, INT32U bufh, INT32U landmark_color, INT32U glasses_color, gpPoint* flandmarks_qvga_ptr, gpPoint* glasses_qvga_ptr)
{
	INT32S i;
	gpImage Image;

	Image.width = bufw;					// depends on display buffer width
	Image.height = bufh;				// depends on display buffer height
	Image.widthStep = bufw*2; 			// depends on display format
	Image.ch = 2; 						// depends on display format
	Image.format = IMG_FMT_UYVY;  		// depends on display format
	Image.ptr = (INT8U *)img_adr;

	//draw_pt(&Image, fl_draw, color);
	draw_flpt(&Image, landmark_color, glasses_color, flandmarks_qvga_ptr, glasses_qvga_ptr);
}
*/
#endif

static void obj_detect_show_result(INT32U img_adr, INT32U bufw, INT32U bufh, ObjResult_t *result)
{

#if KOT_EN == 0
    if(result->is_best_face) {
        draw_obj(img_adr, bufw, bufh, result->result_cnt, (gpRect *)&result->rect[0], DRAW_RED_EN);
#if FL_EN == 1
        draw_fl(img_adr, bufw, bufh, 0xff4c554c, 0x4cff4c55);
#endif
    }
#endif

	if(result->is_get_eye) {
        draw_obj(img_adr, bufw, bufh, 2, (gpRect *)&result->eye[0], DRAW_BLUE_EN);
        draw_obj(img_adr, bufw, bufh, 2, (gpRect *)&result->eyebrow[0], DRAW_BLUE_EN);
	}

	if(result->is_get_nose) {
		draw_obj(img_adr, bufw, bufh, 2, (gpRect *)&result->nose[0], DRAW_BLUE_EN);
	}

	if(result->is_get_mouth) {
		draw_obj(img_adr, bufw, bufh, 4, (gpRect *)&result->mouth[0], DRAW_BLUE_EN);
	}

	if(result->is_smile) {
		//DBG_PRINT("Smile = %d\r\n", result->is_smile);
	}

	if(result->training_ok) {
		//DBG_PRINT("Traning = %d\r\n", result->training_ok);
	}

	if(result->identify_ok) {
		//DBG_PRINT("Identify = %d\r\n", result->identify_ok);
	}
#if TWO_HAND_MODE == 1
	if(result->is_hand) {
#if 0
		if(result->result_cnt == 1)
		{
            if(result->rect[0].x != 0)
                draw_obj(img_adr, bufw, bufh, result->result_cnt, (gpRect *)&result->rect[0], DRAW_YELLOW_EN);
            else if(result->rect[1].x != 0)
                draw_obj(img_adr, bufw, bufh, result->result_cnt, (gpRect *)&result->rect[1], DRAW_YELLOW_EN);
		}
		else
		{
            draw_obj(img_adr, bufw, bufh, 1, (gpRect *)&result->rect[0], DRAW_YELLOW_EN);
            draw_obj(img_adr, bufw, bufh, 1, (gpRect *)&result->rect[1], DRAW_RED_EN);
		}
#else
		if(result->result_cnt == 1)
            draw_obj(img_adr, bufw, bufh, result->result_cnt, (gpRect *)&result->rect[0], DRAW_YELLOW_EN);
		else
		{
            draw_obj(img_adr, bufw, bufh, 1, (gpRect *)&result->rect[0], DRAW_YELLOW_EN);
            draw_obj(img_adr, bufw, bufh, 1, (gpRect *)&result->rect[1], DRAW_RED_EN);
		}
#endif
	}
#else
	if(result->is_hand) {
		draw_obj(img_adr, bufw, bufh, result->result_cnt, (gpRect *)&result->rect[0], DRAW_YELLOW_EN);
	}
#endif

	if(result->is_fist) {
		draw_obj(img_adr, bufw, bufh, result->result_cnt, (gpRect *)&result->rect[0], DRAW_GREEN_EN);
	}
}

#if FACE_RECOGNIZE_MEUN_EN == 1
static void face_recognize_free(void *id_work_mem)
{
	FaceIdentify_t *idWorkMem = (FaceIdentify_t *)id_work_mem;

	if(idWorkMem) {
		if(idWorkMem->identify_WorkMem) {
			gp_free((void *)idWorkMem->identify_WorkMem);
			idWorkMem->identify_WorkMem = 0;
		}

		if(idWorkMem->ownerULBP) {
			gp_free((void*)idWorkMem->ownerULBP);
			idWorkMem->ownerULBP = 0;
		}

		gp_free((void *)idWorkMem);
		idWorkMem = 0;
	}
}

static void face_recognize_set_security_level(void *id_work_mem, INT8U value)
{
	INT32U temp;
	FaceIdentify_t *idWorkMem = (FaceIdentify_t *)id_work_mem;

	idWorkMem->security_level = value;

	switch(idWorkMem->security_level)
	{
		case 1:
			temp = 75112;
			break;

		case 2:
			temp = 90112;
			break;

		case 4:
			temp = 106112;
			break;

		case 5:
			temp = 128612;
			break;

		default:
			temp = 93112;
			break;
	}

	idWorkMem->security_value = temp;
}

static INT32S face_recognize_alloc(void)
{
	FaceIdentify_t *idWorkMem;
	INT32U mem_size;
	INT32S nRet;

	idWorkMem = (FaceIdentify_t *)gp_malloc_align(sizeof(FaceIdentify_t), 32);
	if(idWorkMem == 0) {
		DBG_PRINT("Fail to allocate edWorkMem\r\n");
		return 0;
	}
	DBG_PRINT("idWorkMem = 0x%x\r\n", idWorkMem);
	gp_memset((INT8S *)idWorkMem, 0, sizeof(FaceIdentify_t));

	idWorkMem->image.width     = PPU_TEXT_SIZE_HPIXEL;
	idWorkMem->image.height    = PPU_TEXT_SIZE_VPIXEL;
	idWorkMem->image.ch        = 2;
	idWorkMem->image.widthStep = PPU_TEXT_SIZE_HPIXEL;
	idWorkMem->image.format    = IMG_FMT_GRAY;

	mem_size = FaceIdentify_MemCalc();
	idWorkMem->identify_WorkMem = (INT32U *)gp_malloc_align((mem_size+1024), 4);
	DBG_PRINT("identify_WorkMem:0x%X \r\n",idWorkMem->identify_WorkMem);
	if(idWorkMem->identify_WorkMem == 0) {
		DBG_PRINT("Fail to allocate idWorkMem->identify_WorkMem\r\n");
		RETURN(STATUS_FAIL);
	}
	gp_memset((INT8S *)idWorkMem->identify_WorkMem, 0, mem_size);

	mem_size = SZ_ULBP * (FD_TRAINING_IMAGE_NUMBER + 2);
	idWorkMem->ownerULBP = (INT32U *)gp_malloc_align(mem_size, 4);
    DBG_PRINT("ownerULBP:0x%X \r\n",idWorkMem->ownerULBP);
	if(idWorkMem->ownerULBP == 0) {
		DBG_PRINT("Fail to allocate idWorkMem->ownerULBP\r\n");
		RETURN(STATUS_FAIL);
	}
	gp_memset((INT8S *)idWorkMem->ownerULBP, 0, mem_size);

	//initial
#if 1
    idWorkMem->id_mode = 0;
    face_recognize_set_security_level((void *)idWorkMem, 3);
#else
	face_recognize_init(idWorkMem);
	if(face_recognize_load(idWorkMem) < 0) {
		idWorkMem->id_mode = 1; //training mode
	} else {
		idWorkMem->id_mode = 2; //identify mode
	}
#endif

	nRet = STATUS_OK;
Return:
	if(nRet < 0) {
		face_recognize_free(idWorkMem);
		return 0;
	}
	return (INT32S)idWorkMem;
}

static void face_recognize_proc(void *id_work_mem, gpImage *org_img, void *pre_result)
{
	ObjResult_t *p_obj_result = (ObjResult_t *)pre_result;
	FaceIdentify_t *idWorkMem = (FaceIdentify_t *)id_work_mem;
	INT32S ret,identify_score[FD_TRAINING_IMAGE_NUMBER];
	INT32U identify_index,temp;
	gpRect image[3];

	if(p_obj_result->is_best_face && (idWorkMem->id_mode == 1 || idWorkMem->id_mode == 2))
	{
		// face ROI
		image[0] = p_obj_result->rect[FACE_ROI];
		// leye	ROI
		image[1] = p_obj_result->rect[LEYE_ROI];
		// reye ROI
		image[2] = p_obj_result->rect[REYE_ROI];
	}

	if(p_obj_result->is_best_face == 0) {
		//no face
		idWorkMem->identify_cnt = 0;
		return;
	} else if(idWorkMem->id_mode == 0) {
		// idle
		idWorkMem->training_cnt = 0;
		idWorkMem->identify_cnt = 0;
		return;
	} else if(idWorkMem->id_mode == 1) {
		// traning
		FaceIdentify_Train(org_img,
						&image[0],
						idWorkMem->ownerULBP,
						idWorkMem->training_cnt,
						idWorkMem->identify_WorkMem);
		idWorkMem->training_cnt++;
		DBG_PRINT("training_cnt = %d\r\n", idWorkMem->training_cnt);
		if(idWorkMem->training_cnt >= FD_TRAINING_IMAGE_NUMBER) {
			DBG_PRINT("Training Success\r\n");
			drv_fd_lock();
			idWorkMem->id_mode = 2;
			fd_mode = idWorkMem->id_mode;
			drv_fd_unlock();
			idWorkMem->training_cnt = 0;
			idWorkMem->identify_cnt = 0;
			idWorkMem->training_state = 1;
			obj_result.training_ok = 1;
			if(idWorkMem->pFuncStore) {
				idWorkMem->pFuncStore((INT32U)idWorkMem->ownerULBP, SZ_ULBP*(FD_TRAINING_IMAGE_NUMBER + 1));
			}
		}
	} else if(idWorkMem->id_mode == 2) {
        // identify
        drv_fd_lock();
        temp = idWorkMem->security_value;
        drv_fd_unlock();
        //gpio1_set(1);
        ret = FaceIdentify_Verify_Debug(org_img,
                            &image[0],
                            idWorkMem->ownerULBP,
                            temp,
                            idWorkMem->identify_WorkMem,
                            FD_TRAINING_IMAGE_NUMBER,
                            (int *)&identify_index,
                            (int *)&identify_score[0]);
        //gpio1_set(0);
		idWorkMem->recognize_score = identify_score[0];
		#if FD_DEBUG_EN == 1
			DBG_PRINT("Face_Identify_Index = %d\r\n", (FD_TRAINING_IMAGE_NUMBER - identify_index));
			DBG_PRINT("Face_Identify_Score = %d\r\n", identify_score[0]);
		#endif

		if(ret == 0) {
			DBG_PRINT("NoID\r\n");
			return;
		}

		idWorkMem->identify_cnt++;
		DBG_PRINT("identify_cnt = %d\r\n", idWorkMem->identify_cnt);
		if(idWorkMem->identify_cnt >= 2) {
			DBG_PRINT("Identify Success\r\n");
			drv_fd_lock();
			idWorkMem->id_mode = 0; //idle mode
			fd_mode = idWorkMem->id_mode;
			drv_fd_unlock();
			idWorkMem->training_cnt = 0;
			idWorkMem->identify_cnt = 0;
			idWorkMem->identify_state = 1;
			obj_result.identify_ok = 1;
		}
	} else {
		while(1);
	}
}
#endif

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
#if DISPLAY_USE_PSCALER_EN == 1
    frame_size = disp_h_size*disp_v_size*2;
    pscaler_buf = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(pscaler_buf == 0)
    {
        DBG_PRINT("pscaler_buf fail\r\n");
        while(1);
    }
    prcess_mem_set->ppu_pscaler_workmem = pscaler_buf;
    pscaler_buf = (INT32U)((pscaler_buf + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(pscaler_buf + (i*frame_size));
            pscaler_free_buffer_add((INT32U *)buffer_ptr);
            DBG_PRINT("PscalerBuffer:0x%X \r\n",buffer_ptr);
    }
#else
    scaler_buf1 = (INT32U) gp_malloc_align(((disp_h_size*disp_v_size*2*2)+128), 64);
    if(scaler_buf1 == 0)
    {
        DBG_PRINT("scaler_buf1 fail\r\n");
        while(1);
    }
    prcess_mem_set->ppu_pscaler_workmem = scaler_buf1;
    scaler_buf1 = (INT32U)((scaler_buf1 + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    scaler_buf2 = (scaler_buf1 + (disp_h_size*disp_v_size*2));
    scaler_buf_flag = 0;
    DBG_PRINT("disp_scaler_buf1:0x%X \r\n",scaler_buf1);
    DBG_PRINT("disp_scaler_buf2:0x%X \r\n",scaler_buf2);
#endif
#endif
    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;
    gp_memset((INT8S *)ppu_register_set, 0, sizeof(PPU_REGISTER_SETS));

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
#if DISPLAY_USE_PSCALER_EN == 1
    gplib_ppu_post_process_enable_set(ppu_register_set, 1);
#endif
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

    //Frame buffer malloc
    frame_size = (PPU_TEXT_SIZE_HPIXEL * PPU_TEXT_SIZE_VPIXEL * 2);
    PPU_FRAME_BUFFER_BASE = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(PPU_FRAME_BUFFER_BASE == 0)
    {
        DBG_PRINT("PPU_FRAME_BUFFER_BASE fail\r\n");
        while(1);
    }
    prcess_mem_set->ppu_frame_workmem = PPU_FRAME_BUFFER_BASE;
    PPU_FRAME_BUFFER_BASE = (INT32U)((PPU_FRAME_BUFFER_BASE + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(PPU_FRAME_BUFFER_BASE + (i*frame_size));
            gplib_ppu_frame_buffer_add(ppu_register_set, buffer_ptr);
            DBG_PRINT("PPUBuffer:0x%X \r\n",buffer_ptr);
    }
    drv_l2_display_update(DISPLAY_DEVICE,buffer_ptr);


    // Now configure TEXT relative elements
    gplib_ppu_text_compress_disable_set(ppu_register_set, 1);	                    // Disable TEXT1/TEXT2 horizontal/vertical compress function
    gplib_ppu_text_direct_mode_set(ppu_register_set, 0);			                // Disable TEXT direct address mode

    //text 2 2D
    gplib_ppu_text_init(ppu_register_set, C_PPU_TEXT1);
    text1_narray = (INT32U)gp_malloc_align(1024+64,4);
    if(text1_narray == 0)
    {
        DBG_PRINT("text1_narray fail\r\n");
        while(1);
    }
    DBG_PRINT("text1_narray:0x%X \r\n",text1_narray);
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
 	#if FACE_RECOGNIZE_MEUN_EN == 1
		gplib_ppu_palette_type_set(ppu_register_set, 0,1);
		gplib_ppu_palette_ram_ptr_set(ppu_register_set, 1, (INT32U)_SPRITE_FACE_RECOGNIZE_Palette1);
		gplib_ppu_sprite_enable_set(ppu_register_set, 1);			                     	// Disable Sprite
		gplib_ppu_sprite_coordinate_set(ppu_register_set, 0);                          // set sprite coordinate
		gplib_ppu_sprite_direct_mode_set(ppu_register_set, 0);		                 // Set sprite address mode
		gplib_ppu_sprite_number_set(ppu_register_set, 256);                             // Set sprite number
		gplib_ppu_sprite_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteRAM);   // set sprite ram buffer
		gplib_ppu_sprite_extend_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteExRAM); // value: 32-bit pointer to sprite extend attribute ram
	    gplib_ppu_sprite_segment_set(ppu_register_set, (INT32U)_SPRITE_FACE_RECOGNIZE_CellData);      // sprite cell data
		gplib_ppu_sprite_blend_mode_set(ppu_register_set, 1);
    #else // Disable Sprite
		gplib_ppu_sprite_init(ppu_register_set);
		gplib_ppu_sprite_enable_set(ppu_register_set, 0);
	#endif
}

static void fd_ppu_go(INT32U x, INT32U y, INT32U frame_buffer)
{
    #define DISP_L2_START_EN                    1

    #if DISP_L2_START_EN == 1
        gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, x, y, (INT32U)frame_buffer);	// Calculate Number array
        #if FACE_RECOGNIZE_MEUN_EN == 1
            face_recognize_sprite_update();
        #endif
        // Start PPU and wait until PPU operation is done
        #if DISPLAY_USE_PSCALER_EN == 1
            pscaler_buf = pscaler_free_buffer_get();
            if(pscaler_buf)
                fd_display_set_frame(0,pscaler_buf,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,disp_h_size,disp_v_size);
        #else
            gplib_ppu_go_and_wait_done(ppu_register_set);
        #endif
    #else
        R_PPU_ENABLE = 0x300580;
        R_TFT_FBI_ADDR = frame_buffer;
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

static INT32S free_frame_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    drv_disp_lock();
    event = (INT32U)frame_buf;
    temp = osMessagePut(free_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    drv_disp_unlock();

	return temp;
}

static INT32S free_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame = 0;

    drv_disp_lock();
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
    drv_disp_unlock();

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
#if PALM_DEMO_EN == 0
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
#endif
static INT32S disp_frame_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    drv_disp_lock();
    event = (INT32U)frame_buf;
#if PALM_DEMO_EN == 0
    temp = osMessagePut(display_frame_buffer_queue, (uint32_t)&event, osWaitForever);
#else
    temp = osMessagePut(display_frame_buffer_queue2, (uint32_t)&event, osWaitForever);
#endif
    drv_disp_unlock();

	return temp;
}

static INT32S disp_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
	drv_disp_lock();
	while (!frame) {
#if PALM_DEMO_EN == 0
        result = osMessageGet(display_frame_buffer_queue, osWaitForever);
#else
        result = osMessageGet(display_frame_buffer_queue2, osWaitForever);
#endif

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
    drv_disp_unlock();

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
    INT8U PScalerNum;
	INT32U i,dispBuffer,csi_mode,temp;
	INT32U csiBufferSize,csiBuffer;
	drv_l2_sensor_ops_t *pSencor;
	drv_l2_sensor_info_t *pInfo;
	drv_l2_sensor_para_t *pPara;

	PScalerNum = PSCALER_A;
	csiBufferSize = PRCESS_SRC_WIDTH*PRCESS_SRC_HEIGHT*2;

    csiBuffer = DUMMY_BUFFER_ADDRESS;

	dispBuffer = (INT32U) gp_malloc_align(((csiBufferSize*DISP_QUEUE_MAX)+64), 32);
    if(dispBuffer == 0)
    {
        DBG_PRINT("dispBuffer fail\r\n");
        while(1);
    }
	dispBuffer = (INT32U)((dispBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	prcess_mem_set->csi_prcess_workmem = dispBuffer;
	for(i=0; i<DISP_QUEUE_MAX; i++)
	{
		temp = (dispBuffer+i*csiBufferSize);
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

    pPara = drv_l2_sensor_get_para();
    if (csi_mode == CSI_INTERFACE)
    {
        // CSI
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CSI;
        // Open CSI data path
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CSI;
    }
    else
    {
        R_SYSTEM_CLK_EN0 &= ~(1 << 14);
        // CDSP
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CDSP;
        // Open CDSP data path
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CDSP;
        R_SYSTEM_CLK_EN0 |= (1 << 14);
        drv_l1_CdspSetYuvPscalePath(1);
        R_SYSTEM_CLK_EN0 &= ~(1 << 14);
    }

	PScalerParam.intEnableFlag = PIPELINE_SCALER_INT_ENABLE_ALL;
    PScalerParam.callbackFunc = PScaler_Callback_ISR_AutoZoom;

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

static INT32S fd_frame_buffer_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(fd_frame_buffer_queue, osWaitForever);
    frame = result.value.v;
    if(result.status != osEventMessage) {
        frame = 0;
	}

	return frame;
}

static INT32S fd_free_frame_buffer_add(INT32U *frame_buf)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    event = (INT32U)frame_buf;
    temp = osMessagePut(fd_free_frame_buffer_queue, (uint32_t)&event, osWaitForever);

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

static INT32S PPU_display_buffer_post(INT32U *frame_buf)
{
    INT32U event,temp;

    event = (INT32U)frame_buf;
    temp = osMessagePut(ppu_frame_buffer_queue, (uint32_t)&event, osWaitForever);

	return temp;
}

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
#if PALM_DEMO_EN == 1
// dma ready buffer queue
static INT32S fd_dma_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(fd_dma_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(fd_dma_frame_buffer_queue, (uint32_t)&event, 5);
    }

	return temp;
}

static INT32S fd_dma_frame_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(fd_dma_frame_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(fd_dma_frame_buffer_queue, 5);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}

static void fd_prcess_dma_init(void)
{
    INT32U i, prcess_dma_size, prcess_dma_buf, buffer_ptr;

    //Frame buffer malloc
    prcess_dma_size = 320*240*2;//(FD_PROC_H_SIZE * FD_PROC_V_SIZE * 2);
    //prcess_dma_size = (FD_PROC_H_SIZE * FD_PROC_V_SIZE * 2);
    prcess_dma_buf = (INT32U) gp_malloc_align(((prcess_dma_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(prcess_dma_buf == 0)
    {
        DBG_PRINT("prcess_dma_buf fail\r\n");
        while(1);
    }
    prcess_dma_buf = (INT32U)((prcess_dma_buf + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(prcess_dma_buf + (i*prcess_dma_size));
            fd_dma_frame_buffer_add((INT32U *)buffer_ptr, 1);
            DBG_PRINT("DMABuffer:0x%X \r\n",buffer_ptr);
    }

    //if(disp_init_buffer == 0)
    //    disp_init_buffer = prcess_dma_buf;
}

static INT32S fd_prcess_dma_start(INT32U src_buf, INT32U tar_buf)
{
    INT8S done1;
    DMA_STRUCT prcess_dma;

    prcess_dma.s_addr = src_buf;
    prcess_dma.t_addr = tar_buf;
    prcess_dma.count = (320*240*2)/4;//(FD_PROC_H_SIZE * FD_PROC_V_SIZE * 2)/4;
    //prcess_dma.count = (FD_PROC_H_SIZE * FD_PROC_V_SIZE * 2)/4;
    prcess_dma.width = DMA_DATA_WIDTH_4BYTE;
    prcess_dma.timeout = 0;
    prcess_dma.notify = &done1;

    return drv_l1_dma_transfer_wait_ready((DMA_STRUCT *)&prcess_dma);
}
#endif

static void csi_task_entry(void const *parm)
{
    osEvent result;
    INT32U i,display_buf,y_buffer,ack_msg;
	INT32S frame,temp;
	INT32S ret;
    ObjResult_t fd_draw_result = {0};
#if PFMST_EN == 1
    gpRect trackingSet_draw[10] = {0};
    TPFMSContext context_draw = {0};
    gpRect trackingSet1_draw[10] = {0};
    TPFMSContext context1_draw = {0};
#endif

    DBG_PRINT("csi_task_entry start \r\n");

    // fd prcess memory
    temp = PPU_TEXT_SIZE_HPIXEL*PPU_TEXT_SIZE_VPIXEL*2;
    y_buffer = (INT32U) gp_malloc_align(((temp*C_PPU_DRV_FRAME_NUM)+64),32);
    if(y_buffer == 0)
    {
        DBG_PRINT("y_buffer fail\r\n");
        while(1);
    }
    prcess_mem_set->fd_prcess_workmem = y_buffer;
    y_buffer = (INT32U)((y_buffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);

    for(i=0;i<C_PPU_DRV_FRAME_NUM;i++)
    {
        frame = (INT32U)(y_buffer+(i*temp));
        fd_free_frame_buffer_add((INT32U *)frame);
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
#if PALM_DEMO_EN == 0
    mazeTest_Preview_PScaler();
#endif
    display_buf = 0;
    fd_state_post(FD_STATE_OK);
    osDelay(5);

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
                #if PALM_DEMO_EN == 0
                y_buffer = ppu_draw_frame_buffer_get();
                if(y_buffer)
                {
                    ppu_draw_src_frame(frame,y_buffer,PRCESS_SRC_WIDTH,PRCESS_SRC_HEIGHT,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL);
                    free_frame_buffer_add((INT32U *)frame);
                    frame = y_buffer;
                }
                #endif

                temp = fd_state_get();
                if(temp == FD_STATE_OK)
                {
                    y_buffer = fd_free_frame_buffer_get();
                    if(y_buffer)
                    {
                        #if PFMST_EN == 1
                            fd_detect_set_frame(frame,y_buffer,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,BITMAP_YUYV);
                        #else
                            fd_detect_set_frame(frame,y_buffer,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,BITMAP_GRAY);
                        #endif
                        fd_frame_buffer_post((INT32U *)y_buffer);
                    }
                }

                if(temp == FD_STATE_OK)
                {
                    drv_fd_lock();
                    fd_draw_result.is_best_face = obj_draw_result.is_best_face;
                    fd_draw_result.is_get_eye = obj_draw_result.is_get_eye;
                    fd_draw_result.is_hand = obj_draw_result.is_hand;
                    fd_draw_result.is_fist = obj_draw_result.is_fist;
                    fd_draw_result.is_get_nose = obj_draw_result.is_get_nose;
                    fd_draw_result.is_get_mouth = obj_draw_result.is_get_mouth;
                    fd_draw_result.is_smile = obj_draw_result.is_smile;
                    fd_draw_result.result_cnt = obj_draw_result.result_cnt;
                    fd_draw_result.identify_ok = obj_draw_result.identify_ok;
                    fd_draw_result.training_ok = obj_draw_result.training_ok;

                    if(fd_draw_result.is_best_face || fd_draw_result.is_hand || fd_draw_result.is_fist)
                    {
                        fd_draw_result.rect[0].x = obj_draw_result.rect[0].x;
                        fd_draw_result.rect[0].y = obj_draw_result.rect[0].y;
                        fd_draw_result.rect[0].width = obj_draw_result.rect[0].width;
                        fd_draw_result.rect[0].height = obj_draw_result.rect[0].height;
        #if TWO_HAND_MODE == 1
                        fd_draw_result.rect[1].x = obj_draw_result.rect[1].x;
                        fd_draw_result.rect[1].y = obj_draw_result.rect[1].y;
                        fd_draw_result.rect[1].width = obj_draw_result.rect[1].width;
                        fd_draw_result.rect[1].height = obj_draw_result.rect[1].height;
        #endif
                    }
        #if PFMST_EN == 1
                    context_draw.detect_total = context_result.detect_total;
                    context_draw.trackBackCnt = context_result.trackBackCnt;
                    trackingSet_draw[0].width = trackingSet_result[0].width;
                    trackingSet_draw[0].height = trackingSet_result[0].height;
                    trackingSet_draw[0].x = trackingSet_result[0].x;
                    trackingSet_draw[0].y = trackingSet_result[0].y;
        #if TWO_HAND_MODE == 1
                    context1_draw.detect_total = context1_result.detect_total;
                    context1_draw.trackBackCnt = context1_result.trackBackCnt;
                    trackingSet1_draw[0].width = trackingSet1_result[0].width;
                    trackingSet1_draw[0].height = trackingSet1_result[0].height;
                    trackingSet1_draw[0].x = trackingSet1_result[0].x;
                    trackingSet1_draw[0].y = trackingSet1_result[0].y;
        #endif
        #endif
                    if(fd_draw_result.is_get_eye)
                    {
                        for(i=0;i<2;i++)
                        {
                            fd_draw_result.eye[i].x = obj_draw_result.eye[i].x;
                            fd_draw_result.eye[i].y = obj_draw_result.eye[i].y;
                            fd_draw_result.eye[i].width = obj_draw_result.eye[i].width;
                            fd_draw_result.eye[i].height = obj_draw_result.eye[i].height;
                        }

                        for(i=0;i<2;i++)
                        {
                            fd_draw_result.eyebrow[i].x = obj_draw_result.eyebrow[i].x;
                            fd_draw_result.eyebrow[i].y = obj_draw_result.eyebrow[i].y;
                            fd_draw_result.eyebrow[i].width = obj_draw_result.eyebrow[i].width;
                            fd_draw_result.eyebrow[i].height = obj_draw_result.eyebrow[i].height;
                        }
                    }

                    if(fd_draw_result.is_get_nose)
                    {
                        for(i=0;i<2;i++)
                        {
                            fd_draw_result.nose[i].x = obj_draw_result.nose[i].x;
                            fd_draw_result.nose[i].y = obj_draw_result.nose[i].y;
                            fd_draw_result.nose[i].width = obj_draw_result.nose[i].width;
                            fd_draw_result.nose[i].height = obj_draw_result.nose[i].height;
                        }
                    }

                    if(fd_draw_result.is_get_mouth)
                    {
                        for(i=0;i<4;i++)
                        {
                            fd_draw_result.mouth[i].x = obj_draw_result.mouth[i].x;
                            fd_draw_result.mouth[i].y = obj_draw_result.mouth[i].y;
                            fd_draw_result.mouth[i].width = obj_draw_result.mouth[i].width;
                            fd_draw_result.mouth[i].height = obj_draw_result.mouth[i].height;
                        }
                    }
                    drv_fd_unlock();
                }

                obj_detect_show_result(frame,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,(ObjResult_t *)&fd_draw_result);
        #if PFMST_EN == 1
                if((context_draw.detect_total == 0) && (context_draw.trackBackCnt != 0))
                {
                    #if PPU_DRAW_PRCESS_DEBUG_EN == 1
                        draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &trackingSet_draw[0], 0x10801080);
                    #else
                        #if GESTURE_FIST_TRACKING_EN == 1
                            draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &trackingSet_draw[0], 0xd211d290);
                        #else
                            draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &trackingSet_draw[0], 0xffffffff);
                        #endif
                    #endif
                }
            #if TWO_HAND_MODE == 1
                if((context1_draw.detect_total == 0) && (context1_draw.trackBackCnt != 0))
                {
                    #if PPU_DRAW_PRCESS_DEBUG_EN == 1
                        draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &trackingSet_draw[1], 0x4cff4c55);
                    #else
                        draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &trackingSet_draw[1], 0xffffffff);
                    #endif
                }
            #endif
        #elif KOT_EN == 1
                if(gData_ptr->KOT_initFlg == 1 && gData_ptr->trackTerminateFlg == 0)
                {
                    #if FL_EN == 0
                    #if PPU_DRAW_PRCESS_DEBUG_EN == 1
                        draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &gData_ptr->updateObject, 0xff4c554c);
                    #else
                        draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &gData_ptr->updateObject, 0xffffffff);
                    #endif
                    #endif
                }
                else
                {
                    if(fd_draw_result.is_best_face)
                        draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, &fd_draw_result.rect[0], 0xffffffff);
                }
        #if FL_EN == 1
                //DBG_PRINT("flandmark_in.fl_state=%d\r\n", flandmark_in.fl_state);
                for (i= 0; i < fl_state_draw; ++i)
                {
                    if (flandmarkResult[i] != -1)
                    {
                        //DBG_PRINT("num=%d, i=%d, x=%d, y=%d, w=%d, h=%d\r\n", fl_state_draw, i, faceroi.x, faceroi.y, faceroi.width, faceroi.height);

                        draw_obj(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 1, fl_rect_result + i, 0xff4c554c);

                        gpPoint* flandmarks_qvga_ptr = flandmarks_qvga_draw + i*floutCount;
                        gpPoint* glasses_qvga_ptr = glasses_qvga_draw + i*glassesCount;

                        //DBG_PRINT("draw_i=%d, x=%d, y=%d\r\n", i, flandmarks_qvga_ptr[0].x, flandmarks_qvga_ptr[0].y);
                        draw_fl(frame, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, 0xff4c554c, 0x4cff4c55, flandmarks_qvga_ptr, glasses_qvga_ptr);
                    }
                }
        #endif
        #endif

                #if PALM_DEMO_EN == 0
                fd_ppu_go(PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,frame);
                display_buf = ppu_frame_buffer_display_get();
                ppu_draw_frame_buffer_add((INT32U *)frame);
                if(display_buf > 0)
                    PPU_display_buffer_post((INT32U *)display_buf);
                #else

                display_buf = fd_dma_frame_buffer_get(1);
                ret = fd_prcess_dma_start(frame,display_buf);
                if(ret < 0)
                {
                    //retry = 1;
                    DBG_PRINT("r");
                }
                else
                {
                    if(display_buf > 0)
                    {
                        if(vid_enc_task_q)
                            osMessagePut(vid_enc_task_q, (INT32U)&display_buf, osWaitForever);
                    }
                }

                //osMessagePut(display_frame_q, (uint32_t)&frame, osWaitForever);
                if(scaler_frame_q)
                    osMessagePut(scaler_frame_q, (uint32_t)&frame, osWaitForever);
                #endif

                break;
        }
    }
}

static void disp_task_entry(void const *parm)
{
    osEvent result;
    INT32U display_buf,ack_msg;
#if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
    INT32U display_scaler_buf;
#endif
    DBG_PRINT("disp_task_entry start \r\n");
    disp_buffer_post = 0;
#if DISPLAY_USE_PSCALER_EN == 1
    pscaler_disp_buffer_post = 0;
#endif
#if ROTATOR_FLIP_EN == 1
    INT32U rotator_buf,rotator_addr,rotator_flag;
#endif

    DBG_PRINT("disp_task_entry start \r\n");

    #if ROTATOR_FLIP_EN == 1
        rotator_init();
        rotator_buf = (INT32U)gp_malloc_align((FD_SIZE_HPIXEL*FD_SIZE_VPIXEL*2)*2, 64);
        if(rotator_buf == 0)
        {
            DBG_PRINT("rotator_buf malloc fail \r\n");
            while(1);
        }
    #endif

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
                #if ROTATOR_FLIP_EN == 1
                    gp_free((void *)rotator_buf);
                #endif
                ack_msg = ACK_OK;
                osMessagePut(disp_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(disp_id);
                break;

            default:
                if(display_buf > 0)
                {
                    #if (TPO_TD025THD1 == 0) && (AUO_A027DTN019 == 0) && (ILI9325 == 0)
                        #if DISPLAY_USE_PSCALER_EN == 1
                            display_scaler_buf = pscaler_display_frame_buffer_get();
                            if(display_scaler_buf > 0)
                                drv_l2_display_update(DISPLAY_DEVICE,display_scaler_buf);
                        #else
                            if(scaler_buf_flag)
                            {
                                display_scaler_buf = scaler_buf2;
                                scaler_buf_flag = 0;
                            }
                            else
                            {
                                display_scaler_buf = scaler_buf1;
                                scaler_buf_flag = 1;
                            }
                            fd_display_set_frame(display_buf,display_scaler_buf,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,disp_h_size,disp_v_size);
                            drv_l2_display_update(DISPLAY_DEVICE,display_scaler_buf);
                        #endif
                    #else
                         #if ROTATOR_FLIP_EN == 1
                            if(rotator_flag)
                            {
                                rotator_addr = (INT32U)(rotator_buf+(FD_SIZE_HPIXEL*FD_SIZE_VPIXEL*2));
                                rotator_flag = 0;
                            }
                            else
                            {
                                rotator_addr = (INT32U)rotator_buf;
                                rotator_flag = 1;
                            }
                            rotator_src_img_info(IMAGE_YUYV,FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,display_buf);
                            rotator_tar_img_addr(rotator_addr);
                            //rotator_mode_set(ROTATOR_180);
                            rotator_mode_set(ROTATOR_VERTICAL_MIRROR);
                            rotator_start();
                            drv_l2_display_update(DISPLAY_DEVICE,rotator_addr);
                            rotator_end_wait(1);
                        #else
                            drv_l2_display_update(DISPLAY_DEVICE,display_buf);
                        #endif
                    #endif
                }

                #if DISPLAY_USE_PSCALER_EN == 1
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
                break;
        }
    }
}

static void fd_task_entry(void const *parm)
{
    INT32U i,temp,no_face_cnt,y_buffer,hand_buffer,ack_msg,fd_ymem = 0;
    INT32U t1, t2, t3, t4;
    gpImage *pgray,gray_ptr;
    osEvent result;
#if PFMST_EN == 1 || KOT_EN == 1
    unsigned int size;
	gpImage ROI_Input;
#endif
#if KOT_EN == 1
	unsigned int matchingThre;
	unsigned int matchingThreRangeV;
	unsigned int minExtractionThre;
	unsigned int incExtractionThre;
	unsigned int decExtractionThre;
	unsigned int startMatchingPointN;
	unsigned int extractionThre;
	gpRect objClip;
	gpImage objROI, ROISmoothImage;
	unsigned int KOTMapW, KOTMapH;
#endif
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
    void* _fiMem;
    void* _userDB;
    int* _update_num;
    int* _update_index;
    gpImage *pgray_he,grayhe_ptr;
#endif

    DBG_PRINT("fd_task_entry start \r\n");

    pgray = (gpImage *)&gray_ptr;

#if FACE_RECOGNIZE_MEUN_EN == 1
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
    _fiMem = (void *)gp_malloc_align(FaceIdentify_MemCalc(),4);
    if(!_fiMem)
    {
        DBG_PRINT("_fiMem fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("_fiMem = 0x%x\r\n",_fiMem);
    prcess_mem_set->fiMem_workmem = (INT32U)_fiMem;
    gp_memset((INT8U *)&fidInfo, 0, sizeof(FID_Info));
    fidInfo.fiMem = (void *)_fiMem;
    fidInfo.TRAIN_NUM = 10;
    fidInfo.MAX_SUBJECT_NUM = 5;
    fidInfo.subject_num = 5;
    fidInfo.MAX_UPDATE_NUM = 0;
    fidInfo.SAMPLE_NUM_ONE = fidInfo.TRAIN_NUM + fidInfo.MAX_UPDATE_NUM;
    fidInfo.MAX_SAMPLE_NUM = fidInfo.SAMPLE_NUM_ONE*fidInfo.MAX_SUBJECT_NUM;

    _update_num = (int *)gp_malloc_align(sizeof(int)*fidInfo.MAX_SUBJECT_NUM,4);
    if(!_update_num)
    {
        DBG_PRINT("_update_num fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("_update_num = 0x%x\r\n",_update_num);

    fidInfo.update_num = (int *)_update_num;
    prcess_mem_set->update_num_workmem = (INT32U)fidInfo.update_num;

    _update_index = (int *)gp_malloc_align(sizeof(int)*fidInfo.MAX_SUBJECT_NUM,4);
    if(!_update_index)
    {
        DBG_PRINT("_update_index fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("_update_index = 0x%x\r\n",_update_index);

    fidInfo.update_index = (int *)_update_index;
    prcess_mem_set->update_index_workmem = (INT32U)fidInfo.update_index;

    for(i = 0; i < fidInfo.MAX_SUBJECT_NUM; ++i)
    {
        fidInfo.update_num[i] = 0;
        fidInfo.update_index[i] = fidInfo.TRAIN_NUM;
    }

    _userDB = (void *)gp_malloc_align(SZ_ONEDATA*(fidInfo.MAX_SAMPLE_NUM + 10),4);
    if(!_userDB)
    {
        DBG_PRINT("_userDB fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("_userDB = 0x%x\r\n",_userDB);

    fidInfo.userDB = (void *)_userDB;
    prcess_mem_set->userDB_workmem = (INT32U)fidInfo.userDB;

    fidInfo.scoreLoTh = 150000;
    fidInfo.scoreHiTh = (fidInfo.scoreLoTh) << 1;

    pgray_he = (gpImage *)&grayhe_ptr;
    pgray_he->width = FD_SIZE_HPIXEL;
    pgray_he->height = FD_SIZE_VPIXEL;
    pgray_he->widthStep = FD_SIZE_HPIXEL;
    pgray_he->ch = 1;
    pgray_he->format = IMG_FMT_GRAY;
    temp = (INT32U)gp_malloc_align(FD_SIZE_HPIXEL*FD_SIZE_VPIXEL,4);
    if(!temp)
    {
        DBG_PRINT("temp fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("temp = 0x%x\r\n",temp);
    pgray_he->ptr = (INT8U *)temp;
#endif
    fdWorkMem = (FaceIdentify_t *)face_recognize_alloc();
    if(!fdWorkMem)
    {
        DBG_PRINT("fdWorkMem fail \r\n");
        while(1);
    }
    fdWorkMem->id_mode = fd_mode;
#endif

    fd_gpio_init();

#if FACE_TRACKING_EN == 1 || GESTURE_FIST_TRACKING_EN == 1
    ObjWorkMem_ptr = (ObjDetect_t *)&ObjWorkMem;
    gp_memset((INT8S *)ObjWorkMem_ptr, 0, sizeof(ObjDetect_t));
 	ObjWorkMem_ptr->image.width     = PPU_TEXT_SIZE_HPIXEL;
	ObjWorkMem_ptr->image.height    = PPU_TEXT_SIZE_VPIXEL;
	ObjWorkMem_ptr->image.ch        = 1;
	ObjWorkMem_ptr->image.widthStep = PPU_TEXT_SIZE_HPIXEL;
	ObjWorkMem_ptr->image.format    = IMG_FMT_GRAY;
    obj_track_init((ObjDetect_t *)ObjWorkMem_ptr);
#if MULTIOLE_TRACKING_EN == 1
    TrackWorkMem_ptr = (TrackResult_t *)&TrackWorkMem;
 	gp_memset((INT8S *)TrackWorkMem_ptr, 0, sizeof(TrackResult_t));
 	for(i=0;i<DISP_QUEUE_MAX;i++)
 	{
        TrackWorkMem_ptr->trackWorkMem[i].image.width     = PPU_TEXT_SIZE_HPIXEL;
        TrackWorkMem_ptr->trackWorkMem[i].image.height    = PPU_TEXT_SIZE_VPIXEL;
        TrackWorkMem_ptr->trackWorkMem[i].image.ch        = 1;
        TrackWorkMem_ptr->trackWorkMem[i].image.widthStep = PPU_TEXT_SIZE_HPIXEL;
        TrackWorkMem_ptr->trackWorkMem[i].image.format    = IMG_FMT_GRAY;
        obj_track_init((ObjDetect_t *)&TrackWorkMem_ptr->trackWorkMem[i]);
 	}
#endif
#endif

#if EYE_NOSE_MOUTH_PROC_EN == 1
    eyeWorkMem = (EyeDetect_t *)eye_detect_alloc();
    if(!eyeWorkMem)
    {
        DBG_PRINT("eyeWorkMem fail \r\n");
        while(1);
    }

    mouthWorkMem = (MouthNoseDetect_t *)mouth_detect_alloc();
    if(!mouthWorkMem)
    {
        DBG_PRINT("mouthWorkMem fail \r\n");
        while(1);
    }
#endif

#if PFMST_EN == 1
	//***********************************************************
	//           Partical Filter Mean Shift Tracker Init
	//***********************************************************
	unsigned int PFMST_size_byte = getPFMSTrackingMemorySize();

#if TWO_HAND_MODE == 1
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
	initGPTrack( PFMSTMem.workMem, &context, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL);
	context.detectMode = GP_HAND;  //for GP Haar detector

    pfmst_workmem = (INT32U)gp_malloc_align(PFMST_size_byte,64);
    if(pfmst_workmem == 0)
    {
            DBG_PRINT("PFMST WorkMem1 malloc fail \r\n");
            while(1);
    }
    DBG_PRINT("PFMST WorkMem1:0x%X \r\n", pfmst_workmem);
	PFMSTMem1.workMem = (unsigned char*)pfmst_workmem;
	initGPTrack( PFMSTMem1.workMem, &context1, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL);
	context1.detectMode = GP_HAND;  //for GP Haar detector
#else
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
	initGPTrack( PFMSTMem.workMem, &context, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL);
#if GESTURE_FIST_TRACKING_EN == 1
	context.detectMode = GP_HAND;  //for GP Haar detector
#else
    context.detectMode = GP_FACE;  //for GP Haar detector
#endif
#endif

#elif KOT_EN == 1
	//***********************************************************
	//           Keyponts object tracking Init
	//***********************************************************
	KOTMapW = 60;
	KOTMapH = 60;
	extractionThre = 30;
	minExtractionThre = 10;//50;
	incExtractionThre = 5;
	decExtractionThre = 5;
	startMatchingPointN = 10;
	matchingThre = 2300;
	matchingThreRangeV = 800;

	size = KOT_get_memory_size( KOTMapW, KOTMapH); // bytes
	KOT_workmem = (INT32U)gp_malloc_align(size,64);
    if(KOT_workmem == 0)
    {
        DBG_PRINT("KOT workmem malloc fail \r\n");
        while(1);
    }
	KOT_init_HW(KOTMapW, KOTMapH, (int *)KOT_workmem);                 // robust feature matching system
	KOT_ParamSet((int *)KOT_workmem, extractionThre, matchingThre, matchingThreRangeV, minExtractionThre, incExtractionThre, decExtractionThre, startMatchingPointN);

    #ifdef KOT_HW
        drv_l1_obj_init();
        drv_l1_obj_user_malloc_set(user_malloc_function);
        KOT_fftobj_hw_set(user_KOT_hw_function);
    #endif

    gData_ptr = (globalData_KOT*)KOT_workmem;

	objROI.ptr = (unsigned char*)gp_malloc_align(KOTMapW*KOTMapH, 64);
	objROI.width = KOTMapW;
	objROI.widthStep = KOTMapW;
	objROI.height = KOTMapH;
	objROI.ch = 1;
	objROI.format = IMG_FMT_GRAY;

	ROISmoothImage.ptr = (unsigned char*)gp_malloc_align(KOTMapW*KOTMapH, 64);
	ROISmoothImage.width = KOTMapW;
	ROISmoothImage.widthStep = KOTMapW;
	ROISmoothImage.height = KOTMapH;
	ROISmoothImage.ch = 1;
	ROISmoothImage.format = IMG_FMT_GRAY;
#endif

#if PFMST_EN == 1 || KOT_EN == 1
    ROI_Input.width = PPU_TEXT_SIZE_HPIXEL;
    ROI_Input.widthStep = PPU_TEXT_SIZE_HPIXEL*2;
    ROI_Input.height = PPU_TEXT_SIZE_VPIXEL;
    ROI_Input.ch = 2;
    ROI_Input.format = IMG_FMT_UYVY;//IMG_FMT_YUYV;IMG_FMT_RGB;//

    if(fd_ymem == 0)
    {
        fd_ymem = (INT32U)gp_malloc_align(PPU_TEXT_SIZE_HPIXEL*PPU_TEXT_SIZE_VPIXEL, 64);
        if(fd_ymem == 0)
        {
            DBG_PRINT("FD_Y WorkMem malloc fail \r\n");
            while(1);
        }
        DBG_PRINT("FD_Y WorkMem:0x%X \r\n", fd_ymem);
        prcess_mem_set->fd_y_workmem = fd_ymem;
    }
#endif

#if FL_EN == 1
    INT32U t_fl1, t_fl2;
    INT32U fi;

    gpPoint* flandmarks_qvga = (gpPoint*)gp_malloc_align(sizeof(gpPoint)*(floutCount<<1), 64);
    gpPoint* glasses_qvga = (gpPoint*)gp_malloc_align(sizeof(gpPoint)*(glassesCount<<1), 64);
    gpRect fd_result_fl[2];
    gpPoint *flandmarks_qvga_draw_ptr, *flandmarks_qvga_ptr, *glasses_qvga_draw_ptr, *glasses_qvga_ptr;
    gpImage gray_qqvga;
    gray_qqvga.ch = 1;
    gray_qqvga.format = IMG_FMT_GRAY;
    gray_qqvga.width = gray_qqvga.widthStep = FL_SIZE_HPIXEL;
    gray_qqvga.height = FL_SIZE_VPIXEL;
    gray_qqvga.ptr = (unsigned char*)gp_malloc_align(FL_SIZE_HPIXEL*FL_SIZE_VPIXEL, 64);
    if(gray_qqvga.ptr == 0)
    {
        DBG_PRINT("gray_qqvga.ptr WorkMem malloc fail \r\n");
        while(1);
    }
    DBG_PRINT("gray_qqvga.ptr WorkMem:0x%X \r\n", gray_qqvga.ptr);
    prcess_mem_set->face_qqvga_workmem = gray_qqvga.ptr;

    flandmark_in.gray_qqvga = &gray_qqvga;

	flandmark_in.fl_stable_buflen = 1;//3;
	flandmark_in.flout_stable_buflen = 1;//2;
	flandmark_in.g_stable_buflen = 1;//2;
	flandmark_in.fl_stable_buf = (gpPoint*)gp_malloc_align(sizeof(gpPoint)*landmarksCount*flandmark_in.fl_stable_buflen, 64);
    flandmark_in.flout_stable_buf = (gpPoint*)gp_malloc_align(sizeof(gpPoint)*floutCount*flandmark_in.flout_stable_buflen, 64);
	flandmark_in.g_stable_buf = (gpPoint*)gp_malloc_align(sizeof(gpPoint)*glassesCount*flandmark_in.g_stable_buflen, 64);
    flandmark_in.flandmark_mem = (void*)gp_malloc_align(FLandmark_MemCalc(), 64);

    if(!flandmark_in.fl_stable_buf || !flandmark_in.flout_stable_buf
    || !flandmark_in.g_stable_buf || !flandmark_in.flandmark_mem )
    {
        DBG_PRINT("flandmark_in WorkMem malloc fail \r\n");
        while(1);
    }
    else
    {
        DBG_PRINT("fl_stable_buf WorkMem:0x%X \r\n", flandmark_in.fl_stable_buf);
        DBG_PRINT("flout_stable_buf WorkMem:0x%X \r\n", flandmark_in.flout_stable_buf);
        DBG_PRINT("g_stable_buf WorkMem:0x%X \r\n", flandmark_in.g_stable_buf);
        DBG_PRINT("flandmark_mem WorkMem:0x%X \r\n", flandmark_in.flandmark_mem);
    }
    FLandmark_Init(&flandmark_in);
#endif
    // for new int mode use
    drv_l1_scaler_new_int_callback_set(fd_scaler_step_get);
    no_face_cnt = 0;
    fd_org_buf = 0;
    osDelay(5);
    hand_num_init = 0;
    while(1)
    {
        result = osMessageGet(fd_frame_buffer_queue, osWaitForever);
        fd_org_buf = result.value.v;
        if(result.status != osEventMessage || !fd_org_buf) {
            continue;
        }

        switch(fd_org_buf)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_FD_TASK_EXIT]\r\n");
                ack_msg = ACK_OK;
                osMessagePut(fd_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(fd_id);
                break;

            default:
        #if FACE_RECOGNIZE_MEUN_EN == 1
                drv_fd_lock();
                temp = fd_mode;
                drv_fd_unlock();
                if(temp && fd_org_buf)
        #else
                if(1)
        #endif
                {
                    pgray->width = PPU_TEXT_SIZE_HPIXEL;
                    pgray->height = PPU_TEXT_SIZE_VPIXEL;
                    pgray->widthStep = PPU_TEXT_SIZE_HPIXEL;
                    pgray->ch = 1;
                    pgray->format = IMG_FMT_GRAY;
        #if PFMST_EN == 1
                    fd_detect_set_frame(fd_org_buf, fd_ymem, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, BITMAP_GRAY);
                    pgray->ptr = (INT8U *)fd_ymem;
        #else
                    pgray->ptr = (INT8U *)fd_org_buf;
        #endif
                    t1 = xTaskGetTickCount();
        #if TWO_HAND_MODE == 1  // new prcess
                    ObjWorkMem_ptr->image.width = pgray->width;
                    ObjWorkMem_ptr->image.height = pgray->height;
                    ObjWorkMem_ptr->image.widthStep = pgray->widthStep;
                    ObjWorkMem_ptr->image.ch = pgray->ch;
                    ObjWorkMem_ptr->image.format = pgray->format;
                    ObjWorkMem_ptr->image.ptr = pgray->ptr;
            #if MULTIOLE_TRACKING_EN == 1
                    TrackWorkMem_ptr->trackWorkMem[0].image.width = pgray->width;
                    TrackWorkMem_ptr->trackWorkMem[0].image.height = pgray->height;
                    TrackWorkMem_ptr->trackWorkMem[0].image.widthStep = pgray->widthStep;
                    TrackWorkMem_ptr->trackWorkMem[0].image.ch = pgray->ch;
                    TrackWorkMem_ptr->trackWorkMem[0].image.format = pgray->format;
                    TrackWorkMem_ptr->trackWorkMem[0].image.ptr = pgray->ptr;
                    hand_buffer = gesture_detect_proc((ObjDetect_t *)&TrackWorkMem_ptr->trackWorkMem[MULTIOLE_HAND1]);
            #else
                    hand_buffer = gesture_detect_proc((ObjDetect_t *)ObjWorkMem_ptr);
                    object_position_calculate(GP_HAND, hand_num, &obj_result.rect[0]);
                #if PFMST_EN == 1
                    if(objparam.FD_find_hand_num >= 1)
                    {
                        if((hand_num >= 1) && (objparam.find_hand[0]) && (objparam.find_hand_update[0]))
                            context.detect_total = 1;
                        else
                        {
                            context.detect_total = 0;
                            DBG_PRINT("1");
                        }
                        t3 = xTaskGetTickCount();
                        PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context, &objparam.HandPositionTrackingSet[0]);
                        object_position_calculate(GP_HAND, 1, (gpRect *)&context.trackObj_srcIMG);
                        t4 = xTaskGetTickCount();
                        #if FD_DEBUG_EN == 1
                            if (y_buffer != 0)
                                DBG_PRINT("model construction: time=%d \r\n", (t4 - t3));
                            else if (context.tracking_flag != 0)
                                DBG_PRINT("tracking: time=%d \r\n", (t4 - t3));
                            else
                                DBG_PRINT("release: time=%d \r\n", (t4 - t3));
                        #endif
                    }
                #endif

                #if PFMST_EN == 1
                    if(objparam.FD_find_hand_num >= 2)
                    {
                        if((hand_num >= 1) && (objparam.find_hand[1]) && (objparam.find_hand_update[1]))
                            context1.detect_total = 1;
                        else
                        {
                            context1.detect_total = 0;
                            DBG_PRINT("2");
                        }
                        t3 = xTaskGetTickCount();
                        PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &param, &context1, &objparam.HandPositionTrackingSet[1]);
                        object_position_calculate(GP_HAND, 1, (gpRect *)&context1.trackObj_srcIMG);
                        t4 = xTaskGetTickCount();
                        #if FD_DEBUG_EN == 1
                            if (y_buffer != 0)
                                DBG_PRINT("model construction: time=%d \r\n", (t4 - t3));
                            else if (context.tracking_flag != 0)
                                DBG_PRINT("tracking: time=%d \r\n", (t4 - t3));
                            else
                                DBG_PRINT("release: time=%d \r\n", (t4 - t3));
                        #endif
                    }
                #endif

                #if PFMST_EN == 1
                    if((context.terminationCnt > 3) && (objparam.find_hand[0]) || (context1.terminationCnt > 3) && (objparam.find_hand[1]))
                    {
                        //if(objparam.FD_no_find_hand_cnt > 10)
                        object_position_calculate(GP_HAND, 0xFFFFFFFF, &obj_result.rect[0]);
                        hand_num_init = 0;
                        //objparam.FD_no_find_hand_cnt++;
                    }
                #endif
            #endif
                    if(hand_buffer)
                    {
                        drv_fd_lock();
                        if(hand_buffer == OBJ_FIST)
                        {
                            obj_result.is_fist = 1;
                            obj_result.is_hand = 0;
                        }
                        else
                        {
                            obj_result.is_fist = 0;
                            obj_result.is_hand = 1;
                        }
                        obj_result.is_best_face = 0;
                        obj_result.is_get_eye = 0;
                        obj_result.result_cnt = hand_num;
                        obj_result.is_get_nose = 0;
                        obj_result.is_get_mouth = 0;
                        obj_result.is_smile = 0;
                        obj_result.identify_ok = 0;
                        obj_result.training_ok = 0;
                        obj_draw_result.is_best_face = obj_result.is_best_face;
                        obj_draw_result.is_get_eye = obj_result.is_get_eye;
                        obj_draw_result.is_get_nose = obj_result.is_get_nose;
                        obj_draw_result.is_hand = obj_result.is_hand;
                        obj_draw_result.is_fist = obj_result.is_fist;
                        obj_draw_result.is_smile = obj_result.is_smile;
                        obj_draw_result.result_cnt = obj_result.result_cnt;
                        obj_draw_result.identify_ok = obj_result.identify_ok;
                        obj_draw_result.training_ok = obj_result.training_ok;
                        obj_draw_result.rect[0].x = obj_result.rect[0].x;
                        obj_draw_result.rect[0].y = obj_result.rect[0].y;
                        obj_draw_result.rect[0].width = obj_result.rect[0].width;
                        obj_draw_result.rect[0].height = obj_result.rect[0].height;
                        obj_draw_result.rect[1].x = obj_result.rect[1].x;
                        obj_draw_result.rect[1].y = obj_result.rect[1].y;
                        obj_draw_result.rect[1].width = obj_result.rect[1].width;
                        obj_draw_result.rect[1].height = obj_result.rect[1].height;
            #if PFMST_EN == 1
                        context_result.detect_total = 0;
                        context_result.trackBackCnt = 0;
                        context1_result.detect_total = 0;
                        context1_result.trackBackCnt = 0;
            #endif
                        drv_fd_unlock();
                        no_face_cnt = 0;
                        t2 = xTaskGetTickCount();
            #if FD_DEBUG_EN == 1
                        DBG_PRINT("fd: time=%d \r\n", (t2 - t1));
            #endif
                    }
                    else
                    {
                        drv_fd_lock();
                        obj_result.is_best_face = 0;
                        obj_result.is_get_eye = 0;
                        obj_result.is_hand = 0;
                        obj_result.is_fist = 0;
                        obj_result.result_cnt = 0;
                        obj_result.is_get_nose = 0;
                        obj_result.is_get_mouth = 0;
                        obj_result.is_smile = 0;
                        obj_result.identify_ok = 0;
                        obj_result.training_ok = 0;
                        obj_draw_result.is_best_face = obj_result.is_best_face;
                        obj_draw_result.is_hand = obj_result.is_hand;
                        obj_draw_result.is_fist = obj_result.is_fist;
                        obj_draw_result.is_get_eye = obj_result.is_get_eye;
                        obj_draw_result.is_get_nose = obj_result.is_get_nose;
                        obj_draw_result.is_get_mouth = obj_result.is_get_mouth;
                        obj_draw_result.is_smile = obj_result.is_smile;
                        obj_draw_result.result_cnt = obj_result.result_cnt;
                        obj_draw_result.identify_ok = obj_result.identify_ok;
                        obj_draw_result.training_ok = obj_result.training_ok;
            #if PFMST_EN == 1
                        context_result.detect_total = context.detect_total;
                        context_result.trackBackCnt = context.trackBackCnt;
                        trackingSet_result[0].width = (unsigned int)((context.trackObj_srcIMG.width)*((float)PPU_TEXT_SIZE_HPIXEL/(float)context.SRCIMG.width));
                        trackingSet_result[0].height = (unsigned int)((context.trackObj_srcIMG.height)*((float)PPU_TEXT_SIZE_VPIXEL/(float)context.SRCIMG.height));
                        trackingSet_result[0].x = (unsigned int)((context.trackObj_srcIMG.x)*((float)PPU_TEXT_SIZE_HPIXEL/(float)context.SRCIMG.width));
                        trackingSet_result[0].y = (unsigned int)((context.trackObj_srcIMG.y)*((float)PPU_TEXT_SIZE_VPIXEL/(float)context.SRCIMG.height));
                        context1_result.detect_total = context1.detect_total;
                        context1_result.trackBackCnt = context1.trackBackCnt;
                        trackingSet1_result[0].width = (unsigned int)((context1.trackObj_srcIMG.width)*((float)PPU_TEXT_SIZE_HPIXEL/(float)context1.SRCIMG.width));
                        trackingSet1_result[0].height = (unsigned int)((context1.trackObj_srcIMG.height)*((float)PPU_TEXT_SIZE_VPIXEL/(float)context1.SRCIMG.height));
                        trackingSet1_result[0].x = (unsigned int)((context1.trackObj_srcIMG.x)*((float)PPU_TEXT_SIZE_HPIXEL/(float)context1.SRCIMG.width));
                        trackingSet1_result[0].y = (unsigned int)((context1.trackObj_srcIMG.y)*((float)PPU_TEXT_SIZE_VPIXEL/(float)context1.SRCIMG.height));
            #endif
                        drv_fd_unlock();
                        no_face_cnt++;
                    }
        #else
            #if FACE_RECOGNIZE_MEUN_EN == 1
                #if SCALER_LINEAR_EN == 1
                        #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                            y_buffer = faceRoiDetect_new(pgray, pgray_he,(gpRect *)&obj_result.rect[0], fd_mode);
                        #else
                            y_buffer = faceRoiDetect(pgray, (gpRect *)&obj_result.rect[0],SCALE_LINEAR,0);
                        #endif
                #else
                        #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                            y_buffer = faceRoiDetect_new(pgray, pgray_he,(gpRect *)&obj_result.rect[0], fd_mode);
                        #else
                            y_buffer = faceRoiDetect(pgray, (gpRect *)&obj_result.rect[0],SCALE_NONLINEAR,0);
                        #endif
                #endif
            #else
                #if FACE_TRACKING_EN == 1
#if FL_EN == 1
                    //gpio1_set(1);
                    #if SCALER_LINEAR_EN == 1
                        y_buffer = face_Detect_Two(pgray, (gpRect *)&obj_result.rect[0],SCALE_LINEAR,0);
                    #else
                        y_buffer = face_Detect_Two(pgray, (gpRect *)&obj_result.rect[0],SCALE_NONLINEAR,0);
                    #endif
#else
                    //gpio1_set(1);
                    #if SCALER_LINEAR_EN == 1
                        y_buffer = face_Detect_only(pgray, (gpRect *)&obj_result.rect[0],SCALE_LINEAR,0);
                    #else
                        y_buffer = face_Detect_only(pgray, (gpRect *)&obj_result.rect[0],SCALE_NONLINEAR,0);
                    #endif
                #endif
                #endif
                    //gpio1_set(0);
            #if GESTURE_FIST_TRACKING_EN == 1
                    ObjWorkMem_ptr->image.width = pgray->width;
                    ObjWorkMem_ptr->image.height = pgray->height;
                    ObjWorkMem_ptr->image.widthStep = pgray->widthStep;
                    ObjWorkMem_ptr->image.ch = pgray->ch;
                    ObjWorkMem_ptr->image.format = pgray->format;
                    ObjWorkMem_ptr->image.ptr = pgray->ptr;
                    y_buffer = gesture_detect_proc((ObjDetect_t *)ObjWorkMem_ptr);
            #endif

            #if PFMST_EN == 1
                context.detect_total = y_buffer;
                t3 = xTaskGetTickCount();
                //PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &param, &context, &obj_result.rect[0]);
                PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context, &obj_result.rect[0]);
                t4 = xTaskGetTickCount();
                #if FD_DEBUG_EN == 1
                    if (y_buffer != 0)
                        DBG_PRINT("model construction: time=%d \r\n", (t4 - t3));
                    else if (context.tracking_flag != 0)
                        DBG_PRINT("tracking: time=%d \r\n", (t4 - t3));
                    else
                        DBG_PRINT("release: time=%d \r\n", (t4 - t3));
                #endif

            #elif KOT_EN == 1

#if FL_EN == 1
                if (y_buffer == 1)
                context.detect_total = y_buffer;
                else
                    context.detect_total = 0;
#else
                context.detect_total = y_buffer;
#endif
                if(context.detect_total != 0)
                {
                    gData_ptr->objSet = (gpRect*)&obj_result.rect[0];
                    //gData_ptr->objCount = (unsigned short*)&fd_result.result_cnt[0];
                }

                if( (context.detect_total != 0 && gData_ptr->modelAlreadyFlg == 0 ) || (context.detect_total != 0 )  )
                {

                    gData_ptr->InitObject.width = obj_result.rect[0].width - (obj_result.rect[0].width>>2);
                    gData_ptr->InitObject.height = obj_result.rect[0].height - (obj_result.rect[0].height>>2);
                    gData_ptr->InitObject.x = obj_result.rect[0].x + (obj_result.rect[0].width>>3);
                    gData_ptr->InitObject.y = obj_result.rect[0].y + (obj_result.rect[0].height>>3);
                    gData_ptr->updateObject = gData_ptr->InitObject;

                    gData_ptr->KOT_initFlg = 1;
                }

                if(gData_ptr->KOT_initFlg == 1)
                {
                    t3 = xTaskGetTickCount();
                    objClip.x = gData_ptr->updateObject.x;
                    objClip.y = gData_ptr->updateObject.y;
                    objClip.width = gData_ptr->updateObject.width;
                    objClip.height = gData_ptr->updateObject.height;
                    KOT_scalerClipStart( pgray, &objROI, objClip);
                    KOT_scalerEnd();
                    GPSmoothImage(objROI.ptr, ROISmoothImage.ptr, objROI.width, objROI.height);
                    keypointsObjectTracker(KOT_workmem, &ROISmoothImage, context.detect_total);
                    t4 = xTaskGetTickCount();
                #if FD_DEBUG_EN == 1
                    DBG_PRINT("KOT tracking: time=%d \r\n", (t4 - t3));
                #endif
                }
            #endif
        #endif


#if FL_EN == 1

#if KOT_EN == 1
        if ((y_buffer <= 1) && (gData_ptr->KOT_initFlg == 1 && gData_ptr->trackTerminateFlg == 0))
        {
            flandmark_in.fl_state = 1;

            // scale
            drv_scaler_lock();
            drv_l2_scaler_full_screen(0, 1, pgray, flandmark_in.gray_qqvga);
            drv_scaler_unlock();

            fd_result_fl[0].width = gData_ptr->updateObject.width*0.66667;
            fd_result_fl[0].height = gData_ptr->updateObject.height*0.66667;
            fd_result_fl[0].x = (gData_ptr->updateObject.x >> 1)- (fd_result_fl[0].width >> 3);
            fd_result_fl[0].y = (gData_ptr->updateObject.y >> 1) - (fd_result_fl[0].height >> 3);
        }
        else if (y_buffer > 1)
        {
            flandmark_in.fl_state = 2;
            flandmark_in.fl_init = 1;

            // scale
            drv_scaler_lock();
            drv_l2_scaler_full_screen(0, 1, pgray, flandmark_in.gray_qqvga);
            drv_scaler_unlock();

            fd_result_fl[0].x = obj_result.rect[0].x >> 1;
            fd_result_fl[0].y = obj_result.rect[0].y >> 1;
            fd_result_fl[0].width = obj_result.rect[0].width >> 1;
            fd_result_fl[0].height = obj_result.rect[0].height >> 1;

            fd_result_fl[1].x = obj_result.rect[1].x >> 1;
            fd_result_fl[1].y = obj_result.rect[1].y >> 1;
            fd_result_fl[1].width = obj_result.rect[1].width >> 1;
            fd_result_fl[1].height = obj_result.rect[1].height >> 1;
        }
        else
            flandmark_in.fl_state = 0;

        //t_fl1 = xTaskGetTickCount();

        for (fi = 0; fi < flandmark_in.fl_state; ++fi)
		{
            flandmark_in.face_roi = fd_result_fl + fi;

            //DBG_PRINT("flandmark_in.faceroi: fi=%d, x=%d\r\n", fi, flandmark_in.face_roi->x);
            flandmarkResult[fi] = FLandmark_Detect(&flandmark_in, flandmarks_qvga + fi*floutCount, glasses_qvga + fi*glassesCount);
        }

        t_fl2 = xTaskGetTickCount();
        DBG_PRINT("landmark: time=%d\r\n", (t_fl2 - t_fl1));

        t_fl1 = t_fl2;

        drv_fl_lock();

        fl_state_draw = flandmark_in.fl_state;

        if (fl_state_draw == 1)
        {
            fl_rect_result[0].x = fd_result_fl[0].x << 1;
            fl_rect_result[0].y = fd_result_fl[0].y << 1;
            fl_rect_result[0].width = fd_result_fl[0].width << 1;
            fl_rect_result[0].height = fd_result_fl[0].height << 1;

            flandmarks_qvga_draw_ptr = flandmarks_qvga_draw;
            flandmarks_qvga_ptr = flandmarks_qvga;
            glasses_qvga_draw_ptr = glasses_qvga_draw;
            glasses_qvga_ptr = glasses_qvga;

            for (i = 0; i < floutCount; ++i)
                *(flandmarks_qvga_draw_ptr++) = *(flandmarks_qvga_ptr++);
            for (i = 0; i < glassesCount; ++i)
                *(glasses_qvga_draw_ptr++) = *(glasses_qvga_ptr++);
        }
        else if (fl_state_draw == 2)
        {
            fl_rect_result[0] = obj_result.rect[0];
            fl_rect_result[1] = obj_result.rect[1];

            flandmarks_qvga_draw_ptr = flandmarks_qvga_draw;
            flandmarks_qvga_ptr = flandmarks_qvga;
            glasses_qvga_draw_ptr = glasses_qvga_draw;
            glasses_qvga_ptr = glasses_qvga;

            for (i = 0; i < (floutCount << 1); ++i)
                *(flandmarks_qvga_draw_ptr++) = *(flandmarks_qvga_ptr++);
            for (i = 0; i < (glassesCount << 1); ++i)
                *(glasses_qvga_draw_ptr++) = *(glasses_qvga_ptr++);
        }

        drv_fl_unlock();

        if (flandmark_in.fl_state != 1)
			flandmark_in.fl_init = 1;
#else

        if (y_buffer)
        {
            // scale
            drv_scaler_lock();
            drv_l2_scaler_full_screen(0, 1, pgray, flandmark_in.gray_qqvga);
            drv_scaler_unlock();

            fd_result_fl[0].x = obj_result.rect[0].x >> 1;
            fd_result_fl[0].y = obj_result.rect[0].y >> 1;
            fd_result_fl[0].width = obj_result.rect[0].width >> 1;
            fd_result_fl[0].height = obj_result.rect[0].height >> 1;
        }

        t_fl1 = xTaskGetTickCount();
        flandmarkResult[0] = FLandmark_Detect(&flandmark_in, flandmarks_qvga, glasses_qvga);
        t_fl2 = xTaskGetTickCount();
        DBG_PRINT("landmark: time=%d\r\n", (t_fl2 - t_fl1));
        //t_fl1 = t_fl2;

#endif

#endif

                    if(y_buffer)
                    {
                        drv_fd_lock();
        #if GESTURE_FIST_TRACKING_EN == 1
                        if(y_buffer == OBJ_FIST)
                        {
                            obj_result.is_fist = 1;
                            obj_result.is_hand = 0;
                        }
                        else
                        {
                            obj_result.is_fist = 0;
                            obj_result.is_hand = 1;
                        }
                        obj_result.is_best_face = 0;
        #else
                        obj_result.is_fist = 0;
                        obj_result.is_hand = 0;
                        obj_result.is_best_face = 1;
        #endif
                        obj_result.is_get_eye = 0;
        #if GESTURE_FIST_TRACKING_EN == 1
            #if TWO_HAND_MODE == 1
                        obj_result.result_cnt = hand_num;
            #endif
        #else
                        obj_result.result_cnt = 1;
        #endif
                        obj_result.is_get_nose = 0;
                        obj_result.is_get_mouth = 0;
                        obj_result.is_smile = 0;
                        obj_result.identify_ok = 0;
                        obj_result.training_ok = 0;
                        obj_draw_result.is_best_face = obj_result.is_best_face;
                        obj_draw_result.is_get_eye = obj_result.is_get_eye;
                        obj_draw_result.is_get_nose = obj_result.is_get_nose;
                        obj_draw_result.is_hand = obj_result.is_hand;
                        obj_draw_result.is_fist = obj_result.is_fist;
                        obj_draw_result.is_smile = obj_result.is_smile;
                        obj_draw_result.result_cnt = obj_result.result_cnt;
                        obj_draw_result.identify_ok = obj_result.identify_ok;
                        obj_draw_result.training_ok = obj_result.training_ok;
                        obj_draw_result.rect[0].x = obj_result.rect[0].x;
                        obj_draw_result.rect[0].y = obj_result.rect[0].y;
                        obj_draw_result.rect[0].width = obj_result.rect[0].width;
                        obj_draw_result.rect[0].height = obj_result.rect[0].height;
        #if TWO_HAND_MODE == 1
                        obj_draw_result.rect[1].x = obj_result.rect[1].x;
                        obj_draw_result.rect[1].y = obj_result.rect[1].y;
                        obj_draw_result.rect[1].width = obj_result.rect[1].width;
                        obj_draw_result.rect[1].height = obj_result.rect[1].height;
        #endif
        #if 0
                        context_result.detect_total = 0;
                        context_result.trackBackCnt = 0;
                        trackingSet_result[0].width = 0;
                        trackingSet_result[0].height = 0;
                        trackingSet_result[0].x = 0;
                        trackingSet_result[0].y = 0;
        #endif
                        drv_fd_unlock();
                        no_face_cnt = 0;
        #if FACE_RECOGNIZE_MEUN_EN == 1
                        drv_fd_lock();
                        fdWorkMem->id_mode = fd_mode;
                        obj_result.is_get_eye = 1;
                        drv_fd_unlock();
                        #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                            face_recognize_proc_new((void *)fdWorkMem, (gpImage *)pgray, (void *)&obj_result, 0, 1);
                        #else
                            face_recognize_proc((void *)fdWorkMem, (gpImage *)pgray, (void *)&obj_result);
                        #endif
                        drv_fd_lock();
                        obj_result.is_get_eye = 0;
                        obj_draw_result.is_get_eye = obj_result.is_get_eye;
                        obj_draw_result.identify_ok = obj_result.identify_ok;
                        obj_draw_result.training_ok = obj_result.training_ok;
                        drv_fd_unlock();
        #elif EYE_NOSE_MOUTH_PROC_EN == 1
            #if FACE_RECOGNIZE_MEUN_EN == 0
                        eye_detect_proc((void *)eyeWorkMem, (gpImage *)pgray, (void *)&obj_result);
                        t2 = xTaskGetTickCount();
                #if FD_DEBUG_EN == 1
                        DBG_PRINT("fd: time=%d \r\n", (t2 - t1));
                #endif
                        drv_fd_lock();
                        obj_draw_result.is_smile = obj_result.is_smile;
                        if(obj_result.is_get_eye && obj_result.is_get_nose && obj_result.is_get_mouth)
                        {
                            obj_result.is_best_face = 0;
                            obj_result.result_cnt = 0;
                            obj_draw_result.is_best_face = obj_result.is_best_face;
                            obj_draw_result.result_cnt = obj_result.result_cnt;
                            obj_draw_result.is_get_eye = obj_result.is_get_eye;
                            obj_draw_result.is_get_nose = obj_result.is_get_nose;
                            obj_draw_result.is_get_mouth = obj_result.is_get_mouth;
                            // eye
                            for(i=0;i<2;i++)
                            {
                                obj_draw_result.eye[i].x = obj_result.eye[i].x;
                                obj_draw_result.eye[i].y = obj_result.eye[i].y;
                                obj_draw_result.eye[i].width = obj_result.eye[i].width;
                                obj_draw_result.eye[i].height = obj_result.eye[i].height;
                            }
                            // eyebrow
                            for(i=0;i<2;i++)
                            {
                                obj_draw_result.eyebrow[i].x = obj_result.eyebrow[i].x;
                                obj_draw_result.eyebrow[i].y = obj_result.eyebrow[i].y;
                                obj_draw_result.eyebrow[i].width = obj_result.eyebrow[i].width;
                                obj_draw_result.eyebrow[i].height = obj_result.eyebrow[i].height;
                            }
                            // nose
                            for(i=0;i<2;i++)
                            {
                                obj_draw_result.nose[i].x = obj_result.nose[i].x;
                                obj_draw_result.nose[i].y = obj_result.nose[i].y;
                                obj_draw_result.nose[i].width = obj_result.nose[i].width;
                                obj_draw_result.nose[i].height = obj_result.nose[i].height;
                            }
                            // mouth
                            for(i=0;i<4;i++)
                            {
                                obj_draw_result.mouth[i].x = obj_result.mouth[i].x;
                                obj_draw_result.mouth[i].y = obj_result.mouth[i].y;
                                obj_draw_result.mouth[i].width = obj_result.mouth[i].width;
                                obj_draw_result.mouth[i].height = obj_result.mouth[i].height;
                            }
                        }
                        else
                        {
                            obj_result.is_get_eye = 0;
                            obj_result.is_get_nose = 0;
                            obj_result.is_get_mouth = 0;
                            obj_draw_result.is_get_eye = obj_result.is_get_eye;
                            obj_draw_result.is_get_nose = obj_result.is_get_nose;
                            obj_draw_result.is_get_mouth = obj_result.is_get_mouth;
                        }
                #if PFMST_EN == 1
                        context_result.detect_total = 0;
                        context_result.trackBackCnt = 0;
                #endif
                        drv_fd_unlock();
            #endif
        #endif
                    }
                    else
                    {
                        drv_fd_lock();
                        obj_result.is_best_face = 0;
                        obj_result.is_get_eye = 0;
                        obj_result.is_hand = 0;
                        obj_result.is_fist = 0;
                        obj_result.result_cnt = 0;
                        obj_result.is_get_nose = 0;
                        obj_result.is_get_mouth = 0;
                        obj_result.is_smile = 0;
                        obj_result.identify_ok = 0;
                        obj_result.training_ok = 0;
                        obj_draw_result.is_best_face = obj_result.is_best_face;
                        obj_draw_result.is_hand = obj_result.is_hand;
                        obj_draw_result.is_fist = obj_result.is_fist;
                        obj_draw_result.is_get_eye = obj_result.is_get_eye;
                        obj_draw_result.is_get_nose = obj_result.is_get_nose;
                        obj_draw_result.is_get_mouth = obj_result.is_get_mouth;
                        obj_draw_result.is_smile = obj_result.is_smile;
                        obj_draw_result.result_cnt = obj_result.result_cnt;
                        obj_draw_result.identify_ok = obj_result.identify_ok;
                        obj_draw_result.training_ok = obj_result.training_ok;
            #if PFMST_EN == 1
                        context_result.detect_total = context.detect_total;
                        context_result.trackBackCnt = context.trackBackCnt;
                        trackingSet_result[0].width = (unsigned int)((context.trackObj_srcIMG.width)*((float)PPU_TEXT_SIZE_HPIXEL/(float)context.SRCIMG.width));
                        trackingSet_result[0].height = (unsigned int)((context.trackObj_srcIMG.height)*((float)PPU_TEXT_SIZE_VPIXEL/(float)context.SRCIMG.height));
                        trackingSet_result[0].x = (unsigned int)((context.trackObj_srcIMG.x)*((float)PPU_TEXT_SIZE_HPIXEL/(float)context.SRCIMG.width));
                        trackingSet_result[0].y = (unsigned int)((context.trackObj_srcIMG.y)*((float)PPU_TEXT_SIZE_VPIXEL/(float)context.SRCIMG.height));
            #endif
                        drv_fd_unlock();
                        no_face_cnt++;
                    }
        #endif
                }
        #if FACE_RECOGNIZE_MEUN_EN == 1
                else
                {
                    drv_fd_lock();
                    if(obj_draw_result.is_best_face)
                        obj_draw_result.is_best_face = 0;
                    drv_fd_unlock();
                }
        #endif
                fd_free_frame_buffer_add((INT32U *)fd_org_buf);
                fd_state_post(FD_STATE_OK);

                // set sensor ae
                if(obj_result.is_best_face > 0) {
        #if CSI_AE_EN == 1
                    sensor_control_ae();
        #endif
                }

                if(no_face_cnt >=16) {
                    no_face_cnt = 16;
        #if CSI_AE_EN == 1
                    sensor_control_ae();
        #endif
                }
                break;
        }
    }
}

void GPM4_FD_Demo(void)
{
    INT32S ret;
    osThreadDef_t csi_task = { "csi_task", csi_task_entry, osPriorityAboveNormal, 1, 65536};
    osThreadDef_t disp_task = { "disp_task", disp_task_entry, osPriorityNormal, 1, 65536};
    osThreadDef_t fd_task = { "fd_task", fd_task_entry, osPriorityNormal, 1, 200000};
    osSemaphoreDef_t disp_sem = { 0 };
	osMessageQDef_t disp_q = { (DISP_QUEUE_MAX+1), sizeof(INT32U), 0 };

#if FD_SD_EN == 1
    while(1)
    {
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

    /* initial prcess parameter set structure */
    prcess_mem_set = (prcess_mem_t *)&prcess_mem_structure;
    gp_memset((INT8S *)prcess_mem_set, 0, sizeof(prcess_mem_t));
    csi_pscaler_stop = 0;
    TrackWorkMem_num = 0;

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

	if(sem_disp_engine == NULL)
	{
		sem_disp_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_disp_engine)
		{
            DBG_PRINT("sem_disp_engine error\r\n");
            while(1);
		}
		else
            DBG_PRINT("sem_disp_engine = 0x%x\r\n",sem_disp_engine);
	}

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

    if(sem_fd_draw_engine == NULL)
	{
		sem_fd_draw_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_fd_draw_engine)
		{
            DBG_PRINT("sem_fd_draw_engine error\r\n");
            while(1);
		}
		else
            DBG_PRINT("sem_fd_draw_engine = 0x%x\r\n",sem_fd_draw_engine);
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
#if PALM_DEMO_EN == 0
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
#else
	if(display_frame_buffer_queue2  == NULL)
	{
        display_frame_buffer_queue2 = osMessageCreate(&disp_q, NULL);
		if(!display_frame_buffer_queue2)
		{
            DBG_PRINT("display_frame_buffer_queue2 error\r\n");
            while(1);
		}
		else
            DBG_PRINT("display_frame_buffer_queue2 = 0x%x\r\n",display_frame_buffer_queue2);
	}

	// dma
    if(fd_dma_frame_buffer_queue == NULL)
	{
        fd_dma_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!fd_dma_frame_buffer_queue)
		{
            DBG_PRINT("fd_dma_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("fd_dma_frame_buffer_queue = 0x%x\r\n", fd_dma_frame_buffer_queue);
	}

	fd_prcess_dma_init();
#endif
  	if(fd_frame_buffer_queue == NULL)
	{
        fd_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!fd_frame_buffer_queue)
		{
            DBG_PRINT("fd_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("fd_frame_buffer_queue = 0x%x\r\n",fd_frame_buffer_queue);
	}

  	if(fd_free_frame_buffer_queue == NULL)
	{
        fd_free_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!fd_free_frame_buffer_queue)
		{
            DBG_PRINT("fd_free_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("fd_free_frame_buffer_queue = 0x%x\r\n",fd_free_frame_buffer_queue);
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

  	if(fd_state_queue == NULL)
	{
        fd_state_queue = osMessageCreate(&disp_q, NULL);
  		if(!fd_state_queue)
		{
            DBG_PRINT("fd_state_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("fd_state_queue = 0x%x\r\n",fd_state_queue);
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

  	if(fd_task_ack_m == NULL)
	{
        fd_task_ack_m = osMessageCreate(&disp_q, NULL);
  		if(!fd_task_ack_m)
		{
            DBG_PRINT("fd_task_ack_m error\r\n");
            while(1);
		}
		else
            DBG_PRINT("fd_task_ack_m = 0x%x\r\n",fd_task_ack_m);
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

    csi_id = osThreadCreate(&csi_task, (void *)NULL);
    if(csi_id == 0) {
        DBG_PRINT("csi_task error\r\n");
        while(1);
    }
    else
        osDelay(5);
#if PALM_DEMO_EN == 0
    disp_id = osThreadCreate(&disp_task, (void *)NULL);
    if(disp_id == 0) {
        DBG_PRINT("disp_task error\r\n");
        while(1);
    }
    else
        osDelay(5);
#endif
    fd_id = osThreadCreate(&fd_task, (void *)NULL);
    if(fd_id == 0) {
        DBG_PRINT("fd_task error\r\n");
        while(1);
    }
    else
        osDelay(5);
#if PALM_DEMO_EN == 0
    adc_key_scan_init();
#endif
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
	train_subject = 0;
#endif
    osDelay(500);
#if FACE_RECOGNIZE_MEUN_EN == 1
	fd_mode = 0;
	security_level = 3;
	DBG_PRINT("\r\n***************************************************\r\n");
	DBG_PRINT("              This is FACE_RECOGNIZE DEMO        **\r\n");
    DBG_PRINT("KEY_1 FACE_RECOGNIZE DEMO EXIT                   **\r\n");
	DBG_PRINT("KEY_2 idle mode of FACE_RECOGNIZE                **\r\n");
	DBG_PRINT("KEY_3 training mode of FACE_RECOGNIZE            **\r\n");
	DBG_PRINT("KEY_4 recognize mode of FACE_RECOGNIZE           **\r\n");
	DBG_PRINT("KEY_5 security level up of FACE_RECOGNIZE        **\r\n");
	DBG_PRINT("KEY_6 security level down of FACE_RECOGNIZE      **\r\n");
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
		if(ADKEY_IO2)
		{
            drv_fd_lock();
            fd_mode = 0;
            fdWorkMem->identify_state = 0;
            fdWorkMem->training_state = 0;
            drv_fd_unlock();
            DBG_PRINT("idle mode\r\n");
		}
		if(ADKEY_IO3)
		{
            drv_fd_lock();
            fd_mode = 1;
            fdWorkMem->identify_state = 0;
            fdWorkMem->training_state = 0;
            #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                if(train_subject >= fidInfo.MAX_SUBJECT_NUM)
                    train_subject = 0;
            #endif
            drv_fd_unlock();
            DBG_PRINT("training mode\r\n");
		}
		if(ADKEY_IO4)
		{
			drv_fd_lock();
			fd_mode = 2;
			fdWorkMem->identify_state = 0;
			drv_fd_unlock();
			DBG_PRINT("recognize mode\r\n");
		}
		if(ADKEY_IO5)
		{
            drv_fd_lock();
            if(security_level >= 5)
				security_level = 5;
			else
				security_level++;
            if(fdWorkMem)
                face_recognize_set_security_level((void *)fdWorkMem,security_level);
			drv_fd_unlock();
			DBG_PRINT("security_level = %d\r\n",security_level);

		}
		if(ADKEY_IO6)
		{
            drv_fd_lock();
            if(security_level <= 1)
				security_level = 1;
			else
				security_level--;
			if(fdWorkMem)
                face_recognize_set_security_level((void *)fdWorkMem,security_level);
			drv_fd_unlock();
            DBG_PRINT("security_level = %d\r\n",security_level);
		}
	}
#else
#if PALM_DEMO_EN == 0
	DBG_PRINT("\r\n***************************************************\r\n");
#if FL_EN == 1
	DBG_PRINT("      This is FLandmark DEMO                     **\r\n");
	DBG_PRINT("KEY_1 FLandmark EXIT                             **\r\n");
#else
	DBG_PRINT("      This is FD/EYE/NOSE/MOUTH/HAND DEMO        **\r\n");
	DBG_PRINT("KEY_1 FD/EYE/NOSE/MOUTH/HAND DEMO EXIT           **\r\n");
#endif
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
#endif
#endif
#if PALM_DEMO_EN == 0
    adc_key_scan_uninit();
#endif
#if FD_SD_EN == 1
    _deviceunmount(FS_SD2);
#endif
}



