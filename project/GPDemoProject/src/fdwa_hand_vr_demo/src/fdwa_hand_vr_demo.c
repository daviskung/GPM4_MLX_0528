#include <stdlib.h>
#include <string.h>
#include "project.h"
#include "drv_l1_dma.h"
#include "drv_l1_cdsp.h"
#include "drv_l1_gpio.h"
#include "drv_l1_scaler.h"
#include "drv_l1_pscaler.h"
#include "drv_l1_rotator.h"
#include "drv_l2_scaler.h"
#include "drv_l2_sensor.h"
#include "drv_l2_display.h"
#include "drv_l2_ad_key_scan.h"
#include "drv_l2_cdsp.h"
#include "gplib.h"
#include "ColorTrackerAP.h"
#include "FaceDetectAP.h"
#include "PFMST.h"
#include "KOT.h"
#include "avi_encoder_app.h"

#include "vr_demo_global.h"
#include "FaceIdentifyAP.h"
#include "image_encoder.h"
#include "gplib_ppu.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "SPRITE_FACE_RECOGNIZE_HDR.h"
#include "SPRITE_GPM4_HDR.h"

#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define FRAME_BUF_ALIGN16                   0xF
#define FRAME_BUF_ALIGN8                    0x7
#define FRAME_BUF_ALIGN4                    0x3
#define FRAME_BUF_ALIGN2                    0x1

#define	DISKUSED		 				    FS_SD2//FS_SD //FS_NAND1
#define USER_DETECT_RANGE_EN 	            0
#define EQUALIZEHIST_EN			            1
#define SCALER_INIT_EN		                1

#define FD_EN                               1
#define HAND_EN		                        1
#if WAFD_DEMO_EN == 0
#define VR_EN                               1
#else
#define VR_EN                               0
#endif
#if FD_EN == 1
#define KOT_EN                              1
#else
#define KOT_EN                              0
#endif
#if HAND_EN == 1
#define PFMST_EN                            1
#else
#define PFMST_EN                            0
#endif

#if WAFD_DEMO_EN == 0
#define FACE_RECOGNIZE_MEUN_EN		        1
#define MOTION_DETECTION_EN                 1
#define FUNCTION_DRAW_DEBUG_EN              1
#define CSI_FULL_IMAGE_FLOW_EN              1
#define CSI_FIX_TAR_OUTPUT_FLOW_EN          1
#define FD_NEW_FLOW_EN                      1
#define FD_COMMAND_FLOW_EN                  1
#define FD_COMMAND_FLOW_DEBUG_DRAW_EN       1
#else
#define FACE_RECOGNIZE_MEUN_EN		        0
#define MOTION_DETECTION_EN                 0
#define FUNCTION_DRAW_DEBUG_EN              0
#define CSI_FULL_IMAGE_FLOW_EN              0
#define CSI_FIX_TAR_OUTPUT_FLOW_EN          0
#define FD_NEW_FLOW_EN                      1
#define FD_COMMAND_FLOW_EN                  0
#define FD_COMMAND_FLOW_DEBUG_DRAW_EN       0
#endif
#define COMMAND_DEBUG_EN                    0
#define DMA_SP_DRAW_EN                      1
#define FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN 1
#define FACE_DETECT_MULTI_PEOPLE_FLOW_EN    0
#define IMG_H_SIZE                          32
#define FACEID_MAX_SAVE_IMAGE_NUMBER        20

#if FACE_RECOGNIZE_MEUN_EN == 1
#define PPU_DRAW_EN                         1
#define GP_NEW_FACE_RECOGNIZE_FLOW_EN       1
#if _SENSOR_GC1064_CDSP_MIPI == 1 && AUO_A027DTN019 == 1
#define ROTATOR_FLIP_EN                     1
#else
#define ROTATOR_FLIP_EN                     0
#endif
#else
#define ROTATOR_DRAW_EN                     1
#define GP_NEW_FACE_RECOGNIZE_FLOW_EN       0
#define ROTATOR_FLIP_EN                     0
#endif

#if MOTION_DETECTION_EN == 1
#define MD_SENS			                    10
#define MD_NORMAL			                35
#define	MD_SLOW			                    80
#define	SENSOR_MD_THR		                MD_NORMAL
static INT32U md_work_iram_addr = 0x1FFF6000;
#endif

// demo
#define CSI_PSCALER_STOP_EN                 0
#if WAFD_DEMO_EN == 0
#define DEMO_SD_EN                          1
#else
#define DEMO_SD_EN                          0
#endif
#define GPIO_FUN_EN                         1
#define C_DEVICE_FRAME_NUM		            3
#define DISP_QUEUE_MAX		                C_DEVICE_FRAME_NUM
#define MSG_QUEUE_MAX                       3
#define DUMMY_BUFFER_ADDRESS                0x50000000
#if _SENSOR_H42_CDSP_MIPI == 1 || _SENSOR_GC1064_CDSP_MIPI == 1 || _SENSOR_H62_CDSP_MIPI == 1
#define SENSOR_SRC_WIDTH		            1280
#define SENSOR_SRC_HEIGHT		            720
#else
#define SENSOR_SRC_WIDTH		            640
#define SENSOR_SRC_HEIGHT		            480
#endif
#define SENSOR_TAR_WIDTH                    960
#define SENSOR_TAR_HEIGHT		            720
#if WAFD_DEMO_EN == 0
#define PRCESS_SRC_WIDTH		            640
#define PRCESS_SRC_HEIGHT		            480
#define DEMO_DEBUG_EN                       0
#define CSI_PSCALE_USE                      PSCALER_A
#define DISP_PSCALE_USE                     PSCALER_B
#else
#define PRCESS_SRC_WIDTH		            320
#define PRCESS_SRC_HEIGHT		            240
#define DEMO_DEBUG_EN                       0
#define CSI_PSCALE_USE                      PSCALER_B
#define DISP_PSCALE_USE                     PSCALER_A
#endif
#define FACE_NUM_SET                        0
#define HAND_NUM_SET                        3
#define FIST_NUM_SET                        4
#define MOVINGROI_RIGHT_SET                 5
#define MOVINGROI_LEFT_SET                  6
#define FIST_NUM_SET2                       7
#define FIST_NUM_SET3                       8
#define HAND_NUM_SET2                       9
#define HAND_NUM_SET3                       10
#define DET_MAX_NUM_SET                     12
#define RETURN(x)	                        {nRet = x; goto Return;}
#define ACK_OK			                    0
#define ACK_FAIL		                    (-1)
#define OBJDETECT_MAX_RESULT		        1024
#define MSG_PRCESS_TASK_EXIT                0xFF
#define MSG_PRCESS_FD                       0x10
#define MSG_PRCESS_HAND                     0x20
#define MSG_PRCESS_FIST                     0x30
#define MSG_PRCESS_END                      0x40
#define Q_FLT                               6

#define API_COMMAND_RESULT_DET              0
#define API_COMMAND_FACE_DET                1
#define API_COMMAND_HAND_FIST_DET           2
#define API_COMMAND_DET_OFF                 0xA0
#define API_FIND_FACE                       0xF1
#define API_FIND_HAND                       0xF2
#define API_FIND_FIST                       0xF3

#define COMMAND_STATE_0                     0x80
#define COMMAND_STATE_1                     0x81
#define COMMAND_STATE_2                     0x82
#define COMMAND_STATE_3                     0x83
#define COMMAND_STATE_4                     0x84
#define COMMAND_STATE_HAND                  0x8F

#define COMMAND_HAND_FLIP_RIGHT             2
#define COMMAND_HAND_FLIP_LEFT              1
#define COMMAND_HAND_FLIP_NORESULT          0

#define FIND_FIST_STATE_EN                  1
#define FIND_FLIP_STATE_EN                  0xFF

#define COMMAND_FIND_HAND_FAR               0
#define COMMAND_FIND_HAND_NEAR              1
#define COMMAND_FIND_HAND_X_MAX             2
#define COMMAND_FIND_HAND_X_MIN             3
#define COMMAND_FIND_HAND_Y_MAX             4
#define COMMAND_FIND_HAND_Y_MIN             5
#define COMMAND_FIND_RANG_FLAG              6
#define FD_HAND_UPDATE_STATE_GET	        7
#define OMMAND_FIND_HAND_MAX                8
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
// face detection
#define FACE_TRACKING_EN                    1
#define SCALER_LINEAR_EN                    0
#define CSI_AE_EN                           0
#define FD_SIZE_HPIXEL                      320
#define FD_SIZE_VPIXEL                      240
#define FACE_CLIP_H_SIZE                    152//150
#define FACE_CLIP_V_SIZE                    150
#define PPU_TEXT_SIZE_HPIXEL                FD_SIZE_HPIXEL
#define PPU_TEXT_SIZE_VPIXEL                FD_SIZE_VPIXEL
#define FD_STATE_OK                         0x80
#if WAFD_DEMO_EN == 0
#define FD_EN_PIN1                          IO_D6
#else
#define FD_EN_PIN1                          IO_E2
#endif
#define RETURN(x)			                {nRet = x; goto Return;}

// face recognize
#define C_PPU_DRV_FRAME_NUM		            C_DEVICE_FRAME_NUM
#define INT_MAX				                2147483647
#define FACE_ROI			                0
#define LEYE_ROI			                1
#define REYE_ROI			                2
#define BESTFACE_ROI		                3
#define FD_TRAINING_IMAGE_NUMBER		    10
#define ABS(x)		                        ((x) >= 0 ? (x) : -(x))


#if TPO_TD025THD1 == 0 && ILI9325 == 0 && AUO_A027DTN019 == 0
#if WAFD_DEMO_EN == 0
#define DISPLAY_USE_PSCALER_EN              1
#else
#define DISPLAY_USE_PSCALER_EN              0
#endif
#else
#define DISPLAY_USE_PSCALER_EN              0
#endif

#if FACE_RECOGNIZE_MEUN_EN == 1
#define FD_TRAINING_STATE_GET		        1
#define FD_IDENTIFY_STATE_GET		        2
#define FD_SECURITY_LEVEL_GET               3
#define FD_TRAINING_CNT_GET			        4
#define FD_IDENTIFY_SCORE_GET		        5
#define FD_ULPB_UPDATE_STATE_GET	        6
#define FD_ULPB_UPDATE_CNT_GET		        8
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
	void *equalizeHist_WorkMem;
	void *EyeequalizeHist_WorkMem;
	gpImage	image;
	INT32U WorkMemSize;

	//classify
	ClassifyData clf;
	INT32S xstep;
	INT32S ystep;
	INT32S scale;
	PostProc_t post_proc;

	//result
	INT8U no_face_cnt;
	INT8U gesture_mode;	//gesture detect use
	INT8U fist_mode;
	INT8U reserved1;
	gpRect result[OBJDETECT_MAX_RESULT];
	INT32S count[OBJDETECT_MAX_RESULT];
} ObjDetect_t;

typedef struct {
	INT32S l[3];
	INT32S level[4];
	INT32S cnt[3];
} LumStr;

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
    INT32U csi_prcess_workmem;
    INT32U disp_prcess_workmem;
    INT32U fd_prcess_workmem;
    INT32U fd_y_workmem;
    INT32U pfmst_prcess_workmem;
    INT32U kot_prcess_workmem;
    INT32U kot_roi_workmem;
    INT32U jpg_encode_workmem;
#if CSI_FULL_IMAGE_FLOW_EN == 1
    INT32U csi_pscaler_clip_workmem;
#endif
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
    INT32U fiMem_workmem;
    INT32U userDB_workmem;
    INT32U update_num_workmem;
    INT32U update_index_workmem;
    INT32U face_crop_workmem;
#endif
    INT32U face_he_workmem;
} prcess_mem_t;

typedef enum
{
    COMMAND_FACE_HAND_FIST_ZERO = 0,
    COMMAND_FIND_HAND_NO_FIST,
    COMMAND_FIND_HAND_WITH_FIST,
    COMMAND_TURN_UP,
    COMMAND_TURN_DOWN,
    COMMAND_FUNCTION_UP,
    COMMAND_FUNCTION_DOWN,
    COMMAND_MODULE_STOP,
    COMMAND_MODULE_START,
    COMMAND_MODULE_PAUSE,
    COMMAND_MODULE_RESUME,
    COMMAND_FIND_FACE_AND_TRACK,
    COMMAND_FIND_FIST_AND_TRACK,
    COMMAND_STATE_PRCESS,
    COMMAND_STATE_MAX
} module_command_state;

typedef struct {
    void *hand_gptrackmem_ptr;
    void *fist_gptrackmem_ptr;
    void* KOTData_ptr;
    INT32U audio_en;
    INT32U audio_pause;
    INT32U mode;
    INT32U next_mode;
    INT32U command_prcess_flag;
    INT32U prcess_busy;
    INT32U prcess_find_fist_cnt;
    INT32U prcess_result_cnt;
    INT32U prcess_pos_cnt;
    INT32U prcess_no_find_cnt;
    INT32U prcess_hand_no_fist_cnt;
    INT32U hand_turn_page_on;
    INT32U hand_turn_page_on_cnt;
    INT32U find_fist_width_cnt;
    INT32U find_face;
    INT32U find_hand;
    INT32U find_hand_mode;
	INT32U find_fist;
    INT32U fist_cnt;
    INT32U fist_face_cnt;
    INT32U hand_with_stable_cnt;
    INT32U KOT_workmem;
    INT32U KOTMemSize;
    INT32U command_state;
    INT32U command_ok;
    INT32U command_check_cnt;
    INT32U command_check_enable;
    INT32U hand_find_with_fist_start;
    INT32U ack_msg_flag;
    gpImage ImageModule_ptr;
    INT32U find_hand_range_cnt[OMMAND_FIND_HAND_MAX];
    gpRect result[DET_MAX_NUM_SET];
    gpRect prcess_result[DET_MAX_NUM_SET];
	gpRect objClip;
	gpImage objROI;
	gpImage ROISmoothImage;
} module_command_t;

typedef struct {
	int		width;		/* image width in pixels*/
	int		height;		/* image height in pixels*/
}IMAGESIZE;

static osThreadId csi_id;
static osThreadId disp_id;
static osThreadId prcess_result_id;
static xQueueHandle free_frame_buffer_queue = NULL;
#if WAFD_DEMO_EN == 0
static xQueueHandle csi_frame_buffer_queue = NULL;
static xQueueHandle pscaler_frame_buffer_queue = NULL;
#else
xQueueHandle csi_frame_buffer_queue = NULL;
xQueueHandle pscaler_frame_buffer_queue = NULL;
#endif

static xQueueHandle disp_frame_buffer_queue = NULL;
static xQueueHandle prcess_selection_queue = NULL;
static xQueueHandle prcess_result_queue = NULL;
static xQueueHandle prcess_result_end = NULL;
static xSemaphoreHandle sem_pscaler_engine = NULL;
#if CSI_FULL_IMAGE_FLOW_EN == 1
static xQueueHandle pscaler_clip_buffer_queue = NULL;
#endif
#if DISPLAY_USE_PSCALER_EN == 1
static xQueueHandle ppu_pscaler_buffer_queue = NULL;
#endif
static PSCALER_PARAM_STRUCT PScalerParam = {0};
static INT32U device_h_size, device_v_size;
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static INT32U PPU_FRAME_BUFFER_BASE,fd_org_buf,text1_narray,csi_mode,command_mode;
static INT32U fd_mode,csi_pscaler_stop,csi_md_stop,csi_init_en = 0;
static INT32S security_level,jpeg_buf_size;
static ObjDetect_t *FACEodWorkMem = NULL;
static FaceIdentify_t *fdWorkMem = NULL;
static INT32U scaler_buf1,scaler_buf2,scaler_buf_flag,image_HE_ptr;
static INT32S gMax_score[20];
static INT32S gFDscore;
//INT32U cv_aeawb_open_flag = 1;
static gpRect fist_he_pos;
//
static prcess_mem_t prcess_mem_structure;
static prcess_mem_t *prcess_mem_set;
static module_command_t module_mem_structure;
static module_command_t *module_mem_set;
static xQueueHandle csi_task_ack_m = NULL;
static xQueueHandle disp_task_ack_m = NULL;
static xQueueHandle prcess_result_task_ack_m = NULL;

#if FD_EN == 1
// face detection
static INT32U face_detect_en = 0, face_recognize_en = 0;
static osThreadId fd_id;
static INT32U fd_workmem_size,fd_workmem = 0;
static INT32U fd_ymem;
static INT32U scaler_int_w_step;
static INT32U fd_org_buf;
static ObjResult_t fd_result = {0};
static ObjResult_t fd_draw_result = {0};
static ObjDetect_t ObjWorkMem;
static ObjDetect_t *ObjWorkMem_ptr = NULL;
static xQueueHandle fd_frame_buffer_queue = NULL;
static xQueueHandle fd_free_frame_buffer_queue = NULL;
static xQueueHandle fd_state_queue = NULL;
static xQueueHandle fd_task_ack_m = NULL;
static xSemaphoreHandle sem_fd_engine = NULL;
static xSemaphoreHandle sem_fd_draw_engine = NULL;
static xSemaphoreHandle sem_track_engine = NULL;
#endif

#if HAND_EN == 1
static INT32U hand_detect_en = 0,fist_detect_en = 0;
static ObjResult_t hand_result = {0};
static ObjDetect_t handWorkMem = {0};
static ObjDetect_t *handWorkMem_ptr = NULL;
static INT32U hand_workmem_size, fist_workmem_size = 0;
static ObjResult_t fist_result = {0};
static ObjDetect_t fistWorkMem = {0};
static ObjDetect_t *fistWorkMem_ptr = NULL;
#endif

#if PFMST_EN == 1
// partical filter mean-shift tracker
static gpRect trackingSet_hand_result[10];
static gpRect trackingSet_fist_result[10];
static INT32U pfmst_workmem;
static TPFMSContext context_hand; ///< Context of PFMS tracking
static TPFMSContext context_fist; ///< Context of PFMS tracking
static TPFMSContext context_hand_result; ///< Context of PFMS tracking
static TPFMSContext context_fist_result; ///< Context of PFMS tracking
static PFMSTWorkMem PFMSTMem_hand;
static PFMSTWorkMem PFMSTMem_fist;
//pfunc_PFMST_CROP_hw GPCrop_hw;
static PFMSTWorkMem PFMSTMem_hand_left;
static PFMSTWorkMem PFMSTMem_hand_right;
static TPFMSContext pContext_left;
static TPFMSContext pContext_right;
static gpRect movingROI_right;
static gpRect movingROI_left;
#endif

#if VR_EN == 1
static osThreadId vr_id;
static xQueueHandle vr_state_queue = NULL;
static xQueueHandle vr_task_ack_m = NULL;
extern void VrDemo(void);
#endif


INT32U  checktimesCnt = 0;
#if KOT_EN == 1
//KOT
static TPFMSContext context; ///< Context of PFMS tracking
static INT32U KOT_workmem = 0;
static globalData_KOT* gData_ptr;
static globalData_KOT* gData_ptr_result;
static INT32U compare_out_buffer = 0;
#endif

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
static FID_Info fidInfo;
static INT32S train_subject,train_max_flag,subject_temp;
#endif

static INT32S disp_frame_buffer_add(INT32U *frame_buf, INT32U wait);
static gpRect GPRect_utils(int _x, int _y, int _width, int _height);
static INT32S face_recognize_alloc(void);
static void gp_face_detect_free(ObjDetect_t *odWorkMem);
static INT32S command_result_check(void);
INT32U face_hand_fist_command_result_get(module_command_t *info_ptr, INT32U mode, gpRect *track_result);
static void equalizeHist_new2(const gpImage* src, gpImage* dst, gpRect* roi, gpRect* applyROI);
static void draw_obj(INT32U img_adr, INT32U bufw, INT32U bufh, INT32S total_obj_cnt, gpRect *obj_result_d, INT32U color);
static INT32S pscaler_clip_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_x, INT16U in_y, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h);
static INT32S fd_display_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h);
static INT32S fd_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h, INT32U infmt);
static INT32S prcess_result_state_add(INT32U state, INT32U wait);
static INT32S prcess_result_state_get(INT32U wait);

extern const unsigned char gamma_34[];
extern const unsigned char gamma_30[];
extern const unsigned char gamma_44[];
extern const unsigned char gamma_58[];

static void drv_pscaler_lock(void)
{
    if(sem_pscaler_engine){
        osSemaphoreWait(sem_pscaler_engine, osWaitForever);
    }
}

static void drv_pscaler_unlock(void)
{
    if(sem_pscaler_engine){
        osSemaphoreRelease(sem_pscaler_engine);
    }
}

#if FD_EN == 1
static void fd_gpio1_set(int mode)
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

static void drv_track_lock(void)
{
    if(sem_track_engine){
        osSemaphoreWait(sem_track_engine, osWaitForever);
    }
}

static void drv_track_unlock(void)
{
    if(sem_track_engine){
        osSemaphoreRelease(sem_track_engine);
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
    INT32S ret;

    drv_fd_lock();
    ret = drv_l2_FD_scaler_clip(0, 1, src, dst, clip);
    drv_fd_unlock();

    return ret;
}

static INT32S disp_scaler_clip_Start(gpImage *src, gpImage *dst, gpRect *clip_info)
{
	INT32S ret;
	gpRect clip;

	clip.x = clip_info->x;
	clip.y = clip_info->y;
	clip.width = clip_info->width;
	clip.height = clip_info->height;
	scaler_int_w_step = dst->width;
#if 1
	if((src->width == clip.width) && (src->height == clip.height))
        ret = drv_l2_scaler_full_screen(0, 1, src, dst);
	else
        ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);
#else
	ret = drv_l2_FD_scaler_clip(0, 1, src, dst, &clip);
#endif

	return ret;
}

static INT32S prcess_mem_free(void)
{
    #define MEM_DEBUG_PRINT_EN          1
    INT32U i;

    if(prcess_mem_set)
    {
        if(prcess_mem_set->csi_prcess_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->csi_prcess_workmem = 0x%x\r\n",prcess_mem_set->csi_prcess_workmem);
            #endif
            gp_free((void *)prcess_mem_set->csi_prcess_workmem);
            prcess_mem_set->csi_prcess_workmem = 0;
        }
        if(prcess_mem_set->fd_prcess_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->fd_prcess_workmem = 0x%x\r\n",prcess_mem_set->fd_prcess_workmem);
            #endif
            gp_free((void *)prcess_mem_set->fd_prcess_workmem);
            prcess_mem_set->fd_prcess_workmem = 0;
        }
        if(prcess_mem_set->ppu_frame_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->ppu_frame_workmem = 0x%x\r\n",prcess_mem_set->ppu_frame_workmem);
            #endif
            gp_free((void *)prcess_mem_set->ppu_frame_workmem);
            prcess_mem_set->ppu_frame_workmem = 0;
        }
        if(prcess_mem_set->ppu_narray_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->ppu_narray_workmem = 0x%x\r\n",prcess_mem_set->ppu_narray_workmem);
            #endif
            gp_free((void *)prcess_mem_set->ppu_narray_workmem);
            prcess_mem_set->ppu_narray_workmem = 0;
        }
        if(prcess_mem_set->ppu_pscaler_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->ppu_pscaler_workmem = 0x%x\r\n",prcess_mem_set->ppu_pscaler_workmem);
            #endif
            gp_free((void *)prcess_mem_set->ppu_pscaler_workmem);
            prcess_mem_set->ppu_pscaler_workmem = 0;
        }
        if(prcess_mem_set->fd_y_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->fd_y_workmem = 0x%x\r\n",prcess_mem_set->fd_y_workmem);
            #endif
            gp_free((void *)prcess_mem_set->fd_y_workmem);
            prcess_mem_set->fd_y_workmem = 0;
        }
        if(prcess_mem_set->disp_prcess_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->disp_prcess_workmem = 0x%x\r\n",prcess_mem_set->disp_prcess_workmem);
            #endif
            gp_free((void *)prcess_mem_set->disp_prcess_workmem);
            prcess_mem_set->disp_prcess_workmem = 0;
        }
        if(prcess_mem_set->pfmst_prcess_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->pfmst_prcess_workmem = 0x%x\r\n",prcess_mem_set->pfmst_prcess_workmem);
            #endif
            gp_free((void *)prcess_mem_set->pfmst_prcess_workmem);
            prcess_mem_set->pfmst_prcess_workmem = 0;
        }
        if(prcess_mem_set->kot_prcess_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->kot_prcess_workmem = 0x%x\r\n",prcess_mem_set->kot_prcess_workmem);
            #endif
            gp_free((void *)prcess_mem_set->kot_prcess_workmem);
            prcess_mem_set->kot_prcess_workmem = 0;
        }
        if(prcess_mem_set->kot_roi_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->kot_roi_workmem = 0x%x\r\n",prcess_mem_set->kot_roi_workmem);
            #endif
            gp_free((void *)prcess_mem_set->kot_roi_workmem);
            prcess_mem_set->kot_roi_workmem = 0;
        }
        if(prcess_mem_set->jpg_encode_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->jpg_encode_workmem = 0x%x\r\n",prcess_mem_set->jpg_encode_workmem);
            #endif
            gp_free((void *)prcess_mem_set->jpg_encode_workmem);
            prcess_mem_set->jpg_encode_workmem = 0;
        }
#if CSI_FULL_IMAGE_FLOW_EN == 1
        if(prcess_mem_set->csi_pscaler_clip_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->csi_pscaler_clip_workmem = 0x%x\r\n",prcess_mem_set->csi_pscaler_clip_workmem);
            #endif
            gp_free((void *)prcess_mem_set->csi_pscaler_clip_workmem);
            prcess_mem_set->csi_pscaler_clip_workmem = 0;
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
        if(prcess_mem_set->face_he_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("prcess_mem_set->face_he_workmem = 0x%x\r\n",prcess_mem_set->face_he_workmem);
            #endif
            gp_free((void *)prcess_mem_set->face_he_workmem);
            prcess_mem_set->face_he_workmem = 0;
        }
        prcess_mem_set = NULL;
    }

    // FaceIdentify_t
    if(fdWorkMem)
    {
        #if MEM_DEBUG_PRINT_EN == 1
        DBG_PRINT("fdWorkMem = 0x%x\r\n",fdWorkMem);
        #endif
        if(fdWorkMem->identify_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("fdWorkMem->identify_WorkMem = 0x%x\r\n",fdWorkMem->identify_WorkMem);
            #endif
            gp_free((void *)fdWorkMem->identify_WorkMem);
            fdWorkMem->identify_WorkMem = 0;
        }
        if(fdWorkMem->eye_detect_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("fdWorkMem->eye_detect_WorkMem = 0x%x\r\n",fdWorkMem->eye_detect_WorkMem);
            #endif
            gp_free((void *)fdWorkMem->eye_detect_WorkMem);
            fdWorkMem->eye_detect_WorkMem = 0;
        }
        if(fdWorkMem->ownerULBP)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("fdWorkMem->ownerULBP = 0x%x\r\n",fdWorkMem->ownerULBP);
            #endif
            gp_free((void *)fdWorkMem->ownerULBP);
            fdWorkMem->ownerULBP = 0;
        }
        gp_free((void *)fdWorkMem);
        fdWorkMem = NULL;
    }

#if FD_EN == 1
    if(ObjWorkMem_ptr)
    {
        if(ObjWorkMem_ptr->obj_track_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("ObjWorkMem_ptr->obj_track_WorkMem = 0x%x\r\n",ObjWorkMem_ptr->obj_track_WorkMem);
            #endif
            gp_free((void *)ObjWorkMem_ptr->obj_track_WorkMem);
            ObjWorkMem_ptr->obj_track_WorkMem = 0;
        }

        if(ObjWorkMem_ptr->obj_detect_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("ObjWorkMem_ptr->obj_detect_WorkMem = 0x%x\r\n",ObjWorkMem_ptr->obj_detect_WorkMem);
            #endif
            gp_free((void *)ObjWorkMem_ptr->obj_detect_WorkMem);
            ObjWorkMem_ptr->obj_detect_WorkMem = 0;
        }

        ObjWorkMem_ptr = NULL;
#if KOT_EN == 1
        if(KOT_workmem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("KOT_workmem = 0x%x\r\n",KOT_workmem);
            #endif
            gp_free((void *)KOT_workmem);
            KOT_workmem = 0;
        }

        if(compare_out_buffer)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("compare_out_buffer = 0x%x\r\n",compare_out_buffer);
            #endif
            gp_free((void *)compare_out_buffer);
            compare_out_buffer = 0;
        }
#endif
    }
#endif

#if HAND_EN == 1
    if(handWorkMem_ptr)
    {
        if(handWorkMem_ptr->obj_track_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("handWorkMem_ptr->obj_track_WorkMem = 0x%x\r\n",handWorkMem_ptr->obj_track_WorkMem);
            #endif
            gp_free((void *)handWorkMem_ptr->obj_track_WorkMem);
            handWorkMem_ptr->obj_track_WorkMem = 0;
        }

        if(handWorkMem_ptr->obj_detect_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("handWorkMem_ptr->obj_detect_WorkMem = 0x%x\r\n",handWorkMem_ptr->obj_detect_WorkMem);
            #endif
            gp_free((void *)handWorkMem_ptr->obj_detect_WorkMem);
            handWorkMem_ptr->obj_detect_WorkMem = 0;
        }

        handWorkMem_ptr = NULL;
    }

    if(fistWorkMem_ptr)
    {
        if(fistWorkMem_ptr->obj_track_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("fistWorkMem_ptr->obj_track_WorkMem = 0x%x\r\n",fistWorkMem_ptr->obj_track_WorkMem);
            #endif
            gp_free((void *)fistWorkMem_ptr->obj_track_WorkMem);
            fistWorkMem_ptr->obj_track_WorkMem = 0;
        }

        if(fistWorkMem_ptr->obj_detect_WorkMem)
        {
            #if MEM_DEBUG_PRINT_EN == 1
            DBG_PRINT("fistWorkMem_ptr->obj_detect_WorkMem = 0x%x\r\n",fistWorkMem_ptr->obj_detect_WorkMem);
            #endif
            gp_free((void *)fistWorkMem_ptr->obj_detect_WorkMem);
            fistWorkMem_ptr->obj_detect_WorkMem = 0;
        }

        fistWorkMem_ptr = NULL;
    }
#endif

    if(fd_workmem)
    {
        #if MEM_DEBUG_PRINT_EN == 1
        DBG_PRINT("fd_workmem = 0x%x\r\n",fd_workmem);
        #endif
        gp_free((void *)fd_workmem);
        fd_workmem = 0;
    }

#if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
    if(FACEodWorkMem)
        gp_face_detect_free((ObjDetect_t *)FACEodWorkMem);
#endif

    return 0;
}

static INT32S vr_task_exit(void)
{
    INT32S nRet = STATUS_OK;
#if VR_EN == 1
    // fd task
    POST_MESSAGE(vr_state_queue, MSG_PRCESS_TASK_EXIT, vr_task_ack_m, 5000);
Return:
    OSQFlush(vr_state_queue);
    vQueueDelete(vr_state_queue);
    vr_state_queue = 0;

    OSQFlush(vr_task_ack_m);
    vQueueDelete(vr_task_ack_m);
    vr_task_ack_m = 0;
#endif

    return nRet;
}

static INT32S fd_task_exit(void)
{
    INT32S nRet = STATUS_OK;

#if (FD_EN == 1) || (HAND_EN == 1) || (PFMST_EN == 1)
    // fd task
    POST_MESSAGE(fd_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, fd_task_ack_m, 5000);
Return:
    OSQFlush(fd_frame_buffer_queue);
    vQueueDelete(fd_frame_buffer_queue);
    fd_frame_buffer_queue = 0;

    OSQFlush(fd_free_frame_buffer_queue);
    vQueueDelete(fd_free_frame_buffer_queue);
    fd_free_frame_buffer_queue = 0;

    OSQFlush(fd_task_ack_m);
    vQueueDelete(fd_task_ack_m);
    fd_task_ack_m = 0;

    OSQFlush(fd_state_queue);
    vQueueDelete(fd_state_queue);
    fd_state_queue = 0;

    OSQFlush(sem_pscaler_engine);
    vQueueDelete(sem_pscaler_engine);
    sem_pscaler_engine = 0;

    OSQFlush(prcess_selection_queue);
    vQueueDelete(prcess_selection_queue);
    prcess_selection_queue = 0;

#if DISPLAY_USE_PSCALER_EN == 1
    OSQFlush(ppu_pscaler_buffer_queue);
    vQueueDelete(ppu_pscaler_buffer_queue);
    ppu_pscaler_buffer_queue = 0;
#endif
#endif

    return nRet;
}

static INT32S disp_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    // disp task
    POST_MESSAGE(disp_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, disp_task_ack_m, 5000);
Return:
    OSQFlush(disp_frame_buffer_queue);
    vQueueDelete(disp_frame_buffer_queue);
    disp_frame_buffer_queue = 0;

    OSQFlush(disp_task_ack_m);
    vQueueDelete(disp_task_ack_m);
    disp_task_ack_m = 0;

    return nRet;
}

static INT32S csi_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    // csi task
    POST_MESSAGE(csi_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, csi_task_ack_m, 5000);
Return:
    OSQFlush(csi_frame_buffer_queue);
    vQueueDelete(csi_frame_buffer_queue);
    csi_frame_buffer_queue = 0;

    OSQFlush(free_frame_buffer_queue);
    vQueueDelete(free_frame_buffer_queue);
    free_frame_buffer_queue = 0;

    OSQFlush(pscaler_frame_buffer_queue);
    vQueueDelete(pscaler_frame_buffer_queue);
    pscaler_frame_buffer_queue = 0;
#if CSI_FULL_IMAGE_FLOW_EN == 1
    OSQFlush(pscaler_clip_buffer_queue);
    vQueueDelete(pscaler_clip_buffer_queue);
    pscaler_clip_buffer_queue = 0;
#endif
    OSQFlush(csi_task_ack_m);
    vQueueDelete(csi_task_ack_m);
    csi_task_ack_m = 0;

    return nRet;
}

static INT32S prcess_result_task_exit(void)
{
    INT32S nRet = STATUS_OK;

    // disp task
    POST_MESSAGE(prcess_result_queue, MSG_PRCESS_TASK_EXIT, prcess_result_task_ack_m, 5000);
Return:
    OSQFlush(prcess_result_queue);
    vQueueDelete(prcess_result_queue);
    prcess_result_queue = 0;

    OSQFlush(prcess_result_end);
    vQueueDelete(prcess_result_end);
    prcess_result_end = 0;

    OSQFlush(prcess_result_task_ack_m);
    vQueueDelete(prcess_result_task_ack_m);
    prcess_result_task_ack_m = 0;

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

    nRet = prcess_result_task_exit();
    if(nRet != STATUS_OK)
        DBG_PRINT("prcess_result_task_exit fail\r\n");

    nRet = fd_task_exit();
    if(nRet != STATUS_OK)
        DBG_PRINT("fd_task_exit fail\r\n");

    nRet = vr_task_exit();
    if(nRet != STATUS_OK)
        DBG_PRINT("vr_task_exit fail\r\n");

    prcess_mem_free();

    return nRet;
}

INT32S Find_JPG_File_Index(INT32U max_number)
{
	CHAR  *pdata;
	struct f_info   file_info;
	INT32S nRet,temp,jpg_index;

	jpg_index = -1;

	nRet = _findfirst("*.jpg", &file_info, D_ALL);
	if(nRet < 0)
		goto Return;

	while(1)
	{
		pdata = (CHAR*)file_info.f_name;
		if(gp_strncmp((INT8S*)pdata, (INT8S*)"UBT", 3) == 0)
		{
			temp = (*(pdata + 3) - 0x30)*1000;
			temp += (*(pdata + 4) - 0x30)*100;
			temp += (*(pdata + 5) - 0x30)*10;
			temp += (*(pdata + 6) - 0x30)*1;
			if(temp > jpg_index)
				jpg_index = temp;
		}

		nRet = _findnext(&file_info);
		if(nRet < 0)
			break;
	}

Return:
	jpg_index++;
	if(jpg_index >= max_number || jpg_index < 0)
	   jpg_index = 0;

	return jpg_index;
}

static INT32U image_encdoe_end(INT32U encode_buf,INT32U encode_size)
{

	DBG_PRINT("encode size = %d \r\n",encode_size);

	return encode_buf;
}

static INT32S jpeg_once_encode(INT8U *path, INT32U input_buf, INT32U mode, IMAGESIZE *image)
{
	//char path[64];
	INT16S fd;
	IMAGE_ENCODE_ARGUMENT encode_info;
	INT32U encode_output_ptr,encode_state;

	if(prcess_mem_set->jpg_encode_workmem == 0)
	{
        jpeg_buf_size = (FD_SIZE_HPIXEL*FD_SIZE_VPIXEL);
        prcess_mem_set->jpg_encode_workmem = (INT32U) gp_malloc_align(jpeg_buf_size, 64);//malloc decode frame buffer
        if(prcess_mem_set->jpg_encode_workmem == 0)
        {
            DBG_PRINT("prcess_mem_set->jpg_encode_workmem malloc fail\r\n");
            jpeg_buf_size = 0;
            return -1;
        }
        else
        {
            DBG_PRINT("prcess_mem_set->jpg_encode_workmem = 0x%x \r\n", prcess_mem_set->jpg_encode_workmem);
        }
	}
	else if(mode && ((image->width * image->height) > jpeg_buf_size))
	{
        if(prcess_mem_set->jpg_encode_workmem)
            gp_free((void *)prcess_mem_set->jpg_encode_workmem);
        jpeg_buf_size = (image->width * image->height);
        prcess_mem_set->jpg_encode_workmem = (INT32U) gp_malloc_align(jpeg_buf_size, 64);//malloc decode frame buffer
        if(prcess_mem_set->jpg_encode_workmem == 0)
        {
            DBG_PRINT("prcess_mem_set->jpg_encode_workmem malloc fail\r\n");
            jpeg_buf_size = 0;
            return -1;
        }
        else
        {
            DBG_PRINT("prcess_mem_set->jpg_encode_workmem = 0x%x \r\n", prcess_mem_set->jpg_encode_workmem);
        }
	}
	encode_output_ptr = prcess_mem_set->jpg_encode_workmem;

	fd = fs_open((CHAR *)path, O_CREAT|O_RDWR);
	if (fd >= 0)
	{
	    //encode mode setting
	    encode_info.OutputBufPtr = (INT8U *)encode_output_ptr;
	    encode_info.FileHandle = fd;
	    encode_info.UseDisk = DISKUSED;
	  	encode_info.EncodeMode = IMAGE_ENCODE_ONCE_READ;
	    if(mode)
	    {
            encode_info.InputWidth = image->width;                       					//width of input image
            encode_info.InputHeight = image->height;                        				//Heigh of input image
	    }
	    else
	    {
            encode_info.InputWidth = FD_SIZE_HPIXEL;                       					//width of input image
            encode_info.InputHeight = FD_SIZE_VPIXEL;                        				//Heigh of input image
	    }
	    encode_info.InputBufPtr.yaddr = (INT8U *)input_buf;       		    	        //encode input buffer
	    encode_info.QuantizationQuality = 50;                               	        //encode quality
	    encode_info.InputFormat = IMAGE_ENCODE_INPUT_FORMAT_YUYV;   			        //encode input format
	    encode_info.OutputFormat = IMAGE_ENCODE_OUTPUT_FORMAT_YUV422;       	        //encode input format

		image_encode_entrance();
		image_encode_end_func_register(image_encdoe_end);
		image_encode_start(encode_info);
		while (1) {
			  encode_state = image_encode_status();
	          if (encode_state == IMAGE_CODEC_DECODE_END) {
				DBG_PRINT("image encode ok\r\n");
				break;
			  }else if(encode_state == IMAGE_CODEC_DECODE_FAIL) {
				DBG_PRINT("image encode failed\r\n");
				break;
			  }
		}
		image_encode_stop();                                                     //image encode stop
	}
	else
		DBG_PRINT("image_encode_error\r\n");

	return 0;
}

static INT32S save_file_data_set(INT8U *file_path, INT32U data, INT32U size)
{
    INT16S fd;
    INT32S state;

    fd = fs_open(file_path, O_WRONLY|O_CREAT|O_TRUNC);
    if(fd < 0)
    {
        state = 0;
    }
    else
    {
        state = fs_write(fd, data, size);
        fs_close(fd);
    }

    return state;
}

static INT32U save_file_data_get(INT8U *file_path)
{
    INT16S fd;
    INT32S nRet;
    INT32U data_buf;
    struct sfn_info file_info;

	fd = fs_open(file_path, O_RDONLY);
	if(fd < 0) {
		return 0;
	}

    sfn_stat(fd, &file_info);

	data_buf = (INT32U)gp_malloc_align((file_info.f_size+16), 16);

	if(data_buf == 0) {
		return 0;
	}

    data_buf = (INT32U)((data_buf + FRAME_BUF_ALIGN16) & ~FRAME_BUF_ALIGN16);
	//DBG_PRINT("data_buf = 0x%x\r\n", data_buf);

	nRet = fs_read(fd, data_buf, file_info.f_size);
	if(nRet != file_info.f_size) {
		return 0;
	}

    return data_buf;
}

static INT32S save_file_data_update(INT32U update_number)
{
    INT8U path[128];
    INT32S nRet;
    INT16S fd,fd_next;
    INT32U data_buf;
    struct sfn_info file_info;


    sprintf(path,"ULBP[%04d].jpg",update_number);
    fd = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
    if(fd < 0)
    {
        return -1;
    }
    else
    {
        sprintf(path,"ULBP[%04d].jpg",(update_number+1));
        fd_next = fs_open(path, O_RDONLY);
        if(fd_next < 0) {
            return -1;
        }

        sfn_stat(fd_next, &file_info);

        data_buf = (INT32U)gp_malloc_align((file_info.f_size+16), 16);

        if(data_buf == 0) {
            return -1;
        }

        data_buf = (INT32U)((data_buf + FRAME_BUF_ALIGN16) & ~FRAME_BUF_ALIGN16);
        //DBG_PRINT("data_buf = 0x%x\r\n", data_buf);

        nRet = fs_read(fd_next, data_buf, file_info.f_size);
        if(nRet != file_info.f_size) {
            return -1;
        }

        nRet = fs_write(fd, data_buf, file_info.f_size);
        fs_close(fd);
        fs_close(fd_next);
        gp_free((void *)data_buf);
        if(nRet != file_info.f_size) {
            return -1;
        }
    }

    sprintf(path,"ULBP[%04d]_face.jpg",update_number);
    fd = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
    if(fd < 0)
    {
        return -1;
    }
    else
    {
        sprintf(path,"ULBP[%04d]_face.jpg",(update_number+1));
        fd_next = fs_open(path, O_RDONLY);
        if(fd_next < 0) {
            return -1;
        }

        sfn_stat(fd_next, &file_info);

        data_buf = (INT32U)gp_malloc_align((file_info.f_size+16), 16);

        if(data_buf == 0) {
            return -1;
        }

        data_buf = (INT32U)((data_buf + FRAME_BUF_ALIGN16) & ~FRAME_BUF_ALIGN16);
        //DBG_PRINT("data_buf = 0x%x\r\n", data_buf);

        nRet = fs_read(fd_next, data_buf, file_info.f_size);
        if(nRet != file_info.f_size) {
            return -1;
        }

        nRet = fs_write(fd, data_buf, file_info.f_size);
        fs_close(fd);
        fs_close(fd_next);
        gp_free((void *)data_buf);
        if(nRet != file_info.f_size) {
            return -1;
        }
    }

    return 0;
}

static INT32S save_file_data_unlink(INT32U unlink_number)
{
    INT8U path[128];
    INT32S nRet;

    sprintf(path,"ULBP[%04d]_face.jpg",unlink_number);
    nRet = unlink(path);
    if(nRet < 0)
        return -1;
    sprintf(path,"ULBP[%04d].jpg",unlink_number);
    nRet = unlink(path);
    if(nRet < 0)
        return -1;

    return 0;
}

static INT32S Delete_FDData_InfoUpdate(INT32U max_number, INT32U del_number)
{
    INT8U path[128];
    INT32U i;
    INT32S nRet;

    for(i=del_number;i<max_number;i++)
    {
        nRet = save_file_data_update(i);
        if(nRet < 0)
           return -1;
    }

    save_file_data_unlink(max_number);
    //sprintf(path,"ULBP_FACE[%d]_INFO.bin",train_subject);
    //save_file_data_set((INT8U *)path, (INT32U)&image[0], sizeof(gpRect)*3);
    sprintf(path,"ULBP_INFO[%d].bin",0);
    save_file_data_set((INT8U *)path, (INT32U)&fidInfo, sizeof(FID_Info));
    sprintf(path,"ULBP_DATA[%d].bin",0);
    save_file_data_set((INT8U *)path, (INT32U)fidInfo.userDB, SZ_ONEDATA*(fidInfo.MAX_SAMPLE_NUM + 10));

    return 0;
}

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
static INT32S face_clip_image_end(INT32U frame_buffer, INT16U in_w, INT16U in_h)
{
    DBG_PRINT("face_clip_image = 0x%x, width[%d], height[%d]\r\n", frame_buffer, in_w, in_h);

    return 0;
}

static INT32S scaler_clip_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_x, INT16U in_y, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
    ObjFrame_t *pInput,Input;
	gpRect clip;

	if((in_buffer == 0) || (frame_buffer == 0))
        return -1;

    drv_fd_lock();
    pInput = (ObjFrame_t *)&Input;
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

    clip.x = in_x;
    clip.y = in_y;
    clip.width = out_w;
    clip.height = out_h;

    // scale
    disp_scaler_clip_Start(&pInput->ScaleIn, &pInput->ScaleOut, &clip);
    drv_fd_unlock();
    scalerEnd();

    return 0;
}

static int FaceIdentify_ID_y3_MP(gpImage* gray, gpRect* userROI, FID_Info* fidInfo)
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

static int FaceIdentify_Verify_UBT(gpImage* gray, gpRect* userROI, FID_Info* fidInfo)
{
	fidInfo->best_templ_index[0] = -1;
	int verifyResult = FaceIdentify_Verify_Shift_MP(gray, userROI, fidInfo);

    //DBG_PRINT("best_subject_index = %d \r\n", fidInfo->best_subject_index);
    //DBG_PRINT("best_subject_index2 = %d \r\n", fidInfo->best_subject_index2);

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

    //DBG_PRINT("fidInfo->best_subject_index = %d \r\n", fidInfo->best_subject_index);
    //DBG_PRINT("fidInfo->best_subject_index2 = %d \r\n", fidInfo->best_subject_index2);
    //DBG_PRINT("isBestTwoSimilar = %d \r\n", isBestTwoSimilar);

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
    IMAGESIZE imagerect;
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
        subject_temp = fidInfo.subject_num;
        if((train_subject + 1) > fidInfo.MAX_SUBJECT_NUM)
            train_subject = 0;
        fidInfo.subject_num = (train_subject + 1);
        fidInfo.subject_index = train_subject;
        fidInfo.train_index = idWorkMem->training_cnt;
        FaceIdentify_Train_MP(org_img, &image[0], &fidInfo);
		idWorkMem->training_cnt++;
        if(subject_temp == fidInfo.MAX_SUBJECT_NUM)
        {
            fidInfo.subject_num = subject_temp;
            fidInfo.subject_index = (subject_temp - 1);
            DBG_PRINT("MAX_SUBJECT_NUM = %d\r\n", fidInfo.subject_num);
        }
		//DBG_PRINT("training_cnt = %d\r\n", idWorkMem->training_cnt);
		if(idWorkMem->training_cnt >= FD_TRAINING_IMAGE_NUMBER) {
            train_subject++;
			DBG_PRINT("Training Face[%d] Success\r\n", train_subject);
			drv_fd_result_lock();
			idWorkMem->training_cnt = 0;
			idWorkMem->identify_cnt = 0;
			if(train_subject >= fidInfo.MAX_SUBJECT_NUM)
			{
                fd_result.training_ok = 1;
                idWorkMem->training_state = 1;
                idWorkMem->id_mode = 2;
                fd_mode = idWorkMem->id_mode;
                idWorkMem->training_subject_cnt = train_subject;
                train_max_flag = 1;
			}
			else
			{
                fd_result.training_ok = 0;
                idWorkMem->training_state = 0;
                idWorkMem->id_mode = 0;
                idWorkMem->training_subject_cnt = train_subject;
                fd_mode = idWorkMem->id_mode;
			}
            drv_fd_result_unlock();
            if(idWorkMem->identify_save_en)
            {
                // face result
                // x
                image[0].x = image[0].x - (image[0].width >> 1);  // /2
                if(image[0].x < 0)
                    image[0].x = 0;
                image[0].x &= ~FRAME_BUF_ALIGN2;

                // y
                image[0].y = image[0].y - (image[0].width >> 1);
                if(image[0].y < 0)
                    image[0].y = 0;
                image[0].y &= ~FRAME_BUF_ALIGN2;

                // width
                image[0].width = (image[0].width << 1);          // *2
                if((image[0].x + image[0].width) > 320)
                    image[0].width = 320 - image[0].x;
                image[0].width = ((image[0].width << 1) >> 4 << 3); //align8 for yuyv

                // height
                image[0].height = image[0].width;//image[0].height + ((image[0].height / 5) * 3);
                if((image[0].y + image[0].height) > 240)
                    image[0].height = 240 - image[0].y;
                image[0].height &= ~FRAME_BUF_ALIGN2;

                save_size = (FACE_CLIP_H_SIZE + FRAME_BUF_ALIGN16) & ~FRAME_BUF_ALIGN16;//align16 for yuv420/422

                // crop image
                if(prcess_mem_set->face_crop_workmem == 0)
                {
                    prcess_mem_set->face_crop_workmem = (INT32U)gp_malloc_align(((FD_SIZE_HPIXEL*FD_SIZE_VPIXEL*2)*2),64);
                    if(prcess_mem_set->face_crop_workmem == 0)
                    {
                        DBG_PRINT("face_crop_workmem workmem malloc fail \r\n");
                        while(1);
                    }
                    DBG_PRINT("prcess_mem_set->face_crop_workmem = 0x%x \r\n", prcess_mem_set->face_crop_workmem);
                }
                scaler_clip_frame(yuv_buf, prcess_mem_set->face_crop_workmem, image[0].x, image[0].y, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, image[0].width, image[0].height);
                face_clip_image_end(prcess_mem_set->face_crop_workmem, image[0].width, image[0].height);
                if((image[0].width != save_size) || (image[0].height != FACE_CLIP_V_SIZE))
                {
                    clip_frame_buf = (prcess_mem_set->face_crop_workmem + (FD_SIZE_HPIXEL*FD_SIZE_VPIXEL*2));
                    fd_set_frame(prcess_mem_set->face_crop_workmem, clip_frame_buf, image[0].width, image[0].height, save_size, FACE_CLIP_V_SIZE, BITMAP_YUYV);
                }
                drv_fd_lock();
                sprintf(path,"ULBP[%04d]_face.jpg",train_subject);
                imagerect.width = save_size;
                imagerect.height = FACE_CLIP_V_SIZE;
                jpeg_once_encode((INT8U *)path, clip_frame_buf, 1 , (IMAGESIZE *)&imagerect);
                //drv_fd_unlock();
                #if DRAW_IMAGE_EN == 1
                    // draw image
                    draw_obj(yuv_buf, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &image[0], 0xd211d290);
                #endif
                // save image
                sprintf(path,"ULBP[%04d].jpg",train_subject);
                imagerect.width = FD_SIZE_HPIXEL;
                imagerect.height = FD_SIZE_VPIXEL;
                //drv_fd_lock();
                jpeg_once_encode((INT8U *)path, yuv_buf, 0, (IMAGESIZE *)&imagerect);
                drv_fd_unlock();
                if(train_subject > fidInfo.MAX_SUBJECT_NUM)
                    train_subject = 0;
                #if 1
                //sprintf(path,"ULBP_FACE[%d]_INFO.bin",train_subject);
                //save_file_data_set((INT8U *)path, (INT32U)&image[0], sizeof(gpRect)*3);
                sprintf(path,"ULBP_INFO[%d].bin",0);
                //if(train_subject == 0)
                    //fidInfo.subject_num = 1;
                //else
                    //fidInfo.subject_num = train_subject;
                save_file_data_set((INT8U *)path, (INT32U)&fidInfo, sizeof(FID_Info));
                sprintf(path,"ULBP_DATA[%d].bin",0);
                save_file_data_set((INT8U *)path, (INT32U)fidInfo.userDB, SZ_ONEDATA*(fidInfo.MAX_SAMPLE_NUM + 10));
                #endif
			}
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
            {
                //fidInfo.subject_num = 6;
                ret = FaceIdentify_Verify_UBT(org_img, &image[0], &fidInfo);
            }
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
            if(ret < 0)
            {
                DBG_PRINT("FidInfo Setting Error\r\n");
                return;
            }
            else if((fidInfo.best_subject_index + 1) > fidInfo.subject_num)
            {
                DBG_PRINT("Index NoID\r\n");
                return;
            }

            DBG_PRINT("People GROUP[%d] Success, score = %d  subject_num = %d\r\n", (fidInfo.best_subject_index + 1), fidInfo.score, fidInfo.subject_num);

            if(0)//(idWorkMem->identify_save_en)
            {
                draw_obj(yuv_buf, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &image[0], 0xd211d290);
                sprintf(path,"UBT%04d.jpg",idWorkMem->identify_save_number++);
                imagerect.width = FD_SIZE_HPIXEL;
                imagerect.height = FD_SIZE_VPIXEL;
                jpeg_once_encode((INT8U *)path, yuv_buf, 0, (IMAGESIZE *)&imagerect);
                if(idWorkMem->identify_save_number >= idWorkMem->identify_save_max_number)
                    idWorkMem->identify_save_number = 0;
            }
		}

        DBG_PRINT("Identify Success\r\n");
        drv_fd_result_lock();
        idWorkMem->id_mode = 0; //idle mode
        fd_mode = idWorkMem->id_mode;
        drv_fd_result_unlock();
        idWorkMem->training_cnt = 0;
        idWorkMem->identify_cnt = 0;
        idWorkMem->identify_state = 1;
        fd_result.identify_ok = 1;
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

#if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
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

static int gpRound( double value )
{
	return (int)(value + (value >= 0 ? 0.5 : -0.5));
}

static void equalizeHist_new(const gpImage* src, gpImage* dst, gpRect* roi, gpRect* applyROI)
{
	unsigned int pixNum[256];
	int i, i4;
	int roiwd4s1 = (roi->width >> 2) - 1;
	int roiwd4s1_apply = (applyROI->width >> 2) - 1;
	int wd4s1 = (src->width >> 2) - 1;
	int j, j4;
	int pixNum255;
	//int roiws1 = roi->width - 1;
	unsigned char *srcptr = src->ptr + (roi->y + roi->height - 1) * src->widthStep + roi->x;
	unsigned char *dstptr = dst->ptr + (applyROI->y + applyROI->height - 1) * dst->widthStep + applyROI->x;

	gp_memset((INT8S *)pixNum, 0, 1024);

	i = roi->height;

	while((i--) & 0x3)
	{
		// 0
		j = roi->width;

		while((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4)))++;
		} while(j--);

		srcptr -= src->widthStep;
	}

	i = (roi->height >> 2) - 1;
	do
	{
		// 0
		j = roi->width;

		while((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4)))++;
		} while(j--);

		srcptr -= src->widthStep;

		// 1
		j = roi->width;

		while((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4)))++;
		} while(j--);

		srcptr -= src->widthStep;

		// 2
		j = roi->width;

		while((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4)))++;
		} while(j--);

		srcptr -= src->widthStep;

		// 3
		j = roi->width;

		while((j--) & 0x3)
			(*(pixNum + *(srcptr + j)))++;

		j = roiwd4s1;
		do
		{
			j4 = j << 2;

			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4++)))++;
			(*(pixNum + *(srcptr + j4)))++;
		} while(j--);

		srcptr -= src->widthStep;
	} while(i--);

	*(pixNum + 2) += *(pixNum + 1);
	*(pixNum + 3) += *(pixNum + 2);

	i = 1;
	do
	{
		i4 = i << 2;
		*(pixNum + i4++) += *(pixNum + i4 - 1);
		*(pixNum + i4++) += *(pixNum + i4 - 1);
		*(pixNum + i4++) += *(pixNum + i4 - 1);
		*(pixNum + i4) += *(pixNum + i4 - 1);
	} while((i++) != 63);

	pixNum255 = *(pixNum + 255);

	*(pixNum + 252) = gpRoundFLT(((*(pixNum + 252) * 255) << Q_FLT) / pixNum255);
	*(pixNum + 253) = gpRoundFLT(((*(pixNum + 253) * 255) << Q_FLT) / pixNum255);
	*(pixNum + 254) = gpRoundFLT(((*(pixNum + 254) * 255) << Q_FLT) / pixNum255);

	i = 62;
	do
	{
		i4 = i << 2;

		*(pixNum + i4++) = gpRoundFLT(((*(pixNum + i4) * 255) << Q_FLT) / pixNum255);
		*(pixNum + i4++) = gpRoundFLT(((*(pixNum + i4) * 255) << Q_FLT) / pixNum255);
		*(pixNum + i4++) = gpRoundFLT(((*(pixNum + i4) * 255) << Q_FLT) / pixNum255);
		*(pixNum + i4) = gpRoundFLT(((*(pixNum + i4) * 255) << Q_FLT) / pixNum255);

	} while(i--);

	*pixNum = 0;
	*(pixNum + 255) = 255;

	// lut
	srcptr = src->ptr + (applyROI->y + applyROI->height - 1) * src->widthStep + applyROI->x;

	i = applyROI->height;

	while((i--) & 0x3)
	{
		// 0
		j = applyROI->width;

		while((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;

			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while(j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;
	}

	i = (applyROI->height >> 2) - 1;
	do
	{
		// 0
		j = applyROI->width;

		while((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while(j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;

		// 1
		j = applyROI->width;

		while((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while(j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;

		// 2
		j = applyROI->width;

		while((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while(j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;

		// 3
		j = applyROI->width;

		while((j--) & 0x3)
			//(*(pixNum + *(srcptr + j)))++;
			*(dstptr + j) = *(pixNum + *(srcptr + j));

		j = roiwd4s1_apply;
		do
		{
			j4 = j << 2;
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4++) = *(pixNum + *(srcptr + j4));
			*(dstptr + j4) = *(pixNum + *(srcptr + j4));
		} while(j--);

		srcptr -= src->widthStep;
		dstptr -= dst->widthStep;
	} while(i--);
}

static void equalizeHist(gpImage* src, gpImage* dst, gpRect* roi)
{
	int i,j;
	unsigned int pixNum[256];
	INT8U temp;
#if DEMO_DEBUG_EN == 1
	INT32U t1,t2;
#endif

	gp_memset((INT8S *)pixNum, 0, sizeof(unsigned int)*256);
#if DEMO_DEBUG_EN == 1
	t1 = xTaskGetTickCount();
#endif

	for (i = roi->y; i < roi->y + roi->height; ++i)
	{
		int index = i*src->widthStep;

		for (j = roi->x; j < roi->x + roi->width; ++j)
			pixNum[src->ptr[index + j]]++;
	}

#if DEMO_DEBUG_EN == 1
	t2 = xTaskGetTickCount();
	DBG_PRINT("equalizeHist 1 = %d\r\n", (t2-t1));
	t1 = xTaskGetTickCount();
#endif

	for (i = 2; i < 256; ++i)
		pixNum[i] = pixNum[i] + pixNum[i - 1];

	pixNum[0] = 0;

	for (i = 1; i < 255; ++i)
		pixNum[i] = gpRound(pixNum[i]*255.0/pixNum[255]);

	pixNum[255] = 255;
#if DEMO_DEBUG_EN == 1
	t2 = xTaskGetTickCount();
	DBG_PRINT("equalizeHist 2 = %d\r\n", (t2-t1));
	t1 = xTaskGetTickCount();
#endif

#if 0
	for (i = 0; i < src->height; ++i)
	{
		int index = i*src->widthStep;

		for (j = 0; j < src->width; ++j)
		{
			dst->ptr[index] = pixNum[src->ptr[index]];
			index++;
		}
	}
#else
	j = src->height * src->width;
	for (i = 0; i < j; i++)
	{
		temp  = src->ptr[i];
		dst->ptr[i] = pixNum[temp];
	}
#endif

#if DEMO_DEBUG_EN == 1
	t2 = xTaskGetTickCount();
	DBG_PRINT("equalizeHist 3 = %d\r\n", (t2-t1));
#endif
}

static INT32S FD_Verify_retry(gpImage* gray, const gpRect* userROI, void* ownerULBP, int scoreThreshold, void* fiMem, const int templ_num, int valid_num, int* templ_index, int* score)
{
	int j;
	int temp = 0;
	gpRect image[3];
	INT32S y_offset = 0;
	INT32S verify_result[6] = {0};
	INT32S max_result = -1000000;
	INT32S identify_index,identify_templ[3];
	int cnt=0;
	int cnt_bak=0;
	//int id_index=0;
	//int templ_index_new;
	INT32U t1,t2;
	// face ROI
	image[0] = userROI[0];
	// leye	ROI
	image[1] = userROI[1];
	// reye ROI
	image[2] = userROI[2];

	for(j = 0;j < 6;j++)
	{
		DBG_PRINT("\r\n");
		if(1 == j)
		{
		     image[0].y = userROI[0].y + 1;
		     if((image[0].y + image[0].height) >= gray->height)
		     {
		     	    image[0].height = gray->height - image[0].y;
		     }
		     y_offset = 1;
		     DBG_PRINT("+1\r\n");
		}
		else if(2 == j)
		{
		     image[0].y = userROI[0].y - 1;
		     if(image[0].y < 0)
		     {
		         image[0].y = 0;
		     }
		     y_offset = -1;
		     DBG_PRINT("-1\r\n");
		}
		else if(3 == j)
		{
		     if(verify_result[1] >= verify_result[2])
		     {
		         image[0].y = userROI[0].y + 2;
		         if((image[0].y + image[0].height) >= gray->height)
				     {
				     	    image[0].height = gray->height - image[0].y;
				     }
		         y_offset = 2;
		         DBG_PRINT("+2\r\n");
		     }
		     else
		     {
		         image[0].y = userROI[0].y - 2;
		         if(image[0].y < 0)
			     {
			         image[0].y = 0;
			     }
			     y_offset = -2;
			     DBG_PRINT("-2\r\n");
		     }
		}
		else if(4 == j)
		{
		     if(verify_result[1] >= verify_result[2])
		     {
		         if(verify_result[1] >= verify_result[3])
		         {
		             image[0].y = userROI[0].y - 2;
			         if(image[0].y < 0)
				     {
				         image[0].y = 0;
				     }
				     y_offset = -2;
				     DBG_PRINT("-2\r\n");
		         }
		         else
		         {
		             image[0].y = userROI[0].y + 3;
		             if((image[0].y + image[0].height) >= gray->height)
						     {
						     	    image[0].height = gray->height - image[0].y;
						     }
		             y_offset = 3;
		             DBG_PRINT("+3\r\n");
		         }
		     }
		     else
		     {
		         if(verify_result[2] >= verify_result[3])
		         {
		             image[0].y = userROI[0].y + 2;
		             if((image[0].y + image[0].height) >= gray->height)
						     {
						     	    image[0].height = gray->height - image[0].y;
						     }
		             y_offset = 2;
		             DBG_PRINT("+2\r\n");

		         }
		         else
		         {
		             image[0].y = userROI[0].y - 3;
		             y_offset = 3;
		             if(image[0].y < 0)
				     {
				         image[0].y = 0;
				     }
				     DBG_PRINT("-3\r\n");
		         }
		     }
		}
		else if(5 == j)
		{
		     if((image[0].y == userROI[0].y) || (y_offset > 2) || (y_offset < (-2))
		     	            || ((verify_result[4] < verify_result[1]) && (y_offset == 2) )
			                || ((verify_result[4] < verify_result[2]) && (y_offset == -2)))
		     {
		         break;
		     }
		     else if(image[0].y > userROI[0].y)
		     {
		         image[0].y ++;
		         y_offset ++;
		         DBG_PRINT("+3\r\n");
		     }
		     else
		     {
		         image[0].y --;
		         if(image[0].y < 0)
			     {
			         image[0].y = 0;
			     }
			     y_offset --;
			     DBG_PRINT("-3\r\n");
		     }
		}

		/*
            DBG_PRINT("faceROI.x = %d\r\n", image[0].x);
		    DBG_PRINT("faceROI.y = %d\r\n", image[0].y);
		    DBG_PRINT("faceROI.width = %d\r\n", image[0].width);
		    DBG_PRINT("faceROI.height = %d\r\n", image[0].height);
		*/
        if(0 == j)
        {

            identify_templ[0] = -1;
            DBG_PRINT("identify_templ[0] = %d\r\n", identify_templ[0]);
        }
        else
        {
            DBG_PRINT("identify_templ[0] = %d\r\n", identify_templ[0]);
            DBG_PRINT("identify_templ[1] = %d\r\n", identify_templ[1]);
            DBG_PRINT("identify_templ[2] = %d\r\n", identify_templ[2]);
        }

#if DEMO_DEBUG_EN == 1
        t1 = xTaskGetTickCount();
#endif
        #if 1
            temp = 0;
            cnt = FaceIdentify_Verify_Shift(gray,
                            &image[0],
                            ownerULBP,
                            scoreThreshold,
                            fiMem,
                            templ_num,
                            valid_num,
                            (int *)&(identify_templ[0]),
                            (int *)&temp);
        #else
            cnt = FaceIdentify_Verify_Debug(gray,
                            &image[0],
                            ownerULBP,
                            scoreThreshold,
                            fiMem,
                            templ_num,
                            (int *)&(identify_templ[0]),
                            (int *)&temp);

        #endif

        if(0 == j)
        {
            identify_index = identify_templ[0];
            if(0 == identify_index)
            {
                identify_templ[0] = 0;
                identify_templ[1] = 1;
                identify_templ[2] = 2;
            }
            else if((valid_num - 1) == identify_index)
            {
                identify_templ[0] = identify_index - 2;
                identify_templ[1] = identify_index - 1;
                identify_templ[2] = identify_index;
            }
            else
            {
                identify_templ[0] = identify_index - 1;
                identify_templ[1] = identify_index;
                identify_templ[2] = identify_index + 1;
            }
        }

#if DEMO_DEBUG_EN == 1
        t2 = xTaskGetTickCount();
        DBG_PRINT("FaceIdentify_Verify_Shift = %d,recognize_score = %d\r\n", (t2 - t1),temp);
#endif
		if(temp > max_result)
		{
			max_result = temp;
			cnt_bak = cnt;
			*templ_index = identify_templ[0];
		}

		if(temp  > scoreThreshold)
		{
			break;
		}
		else
		{
			verify_result[j] = temp;
		}
	}

	*score = max_result;

	return cnt_bak;
}

void gp_face_recognize_proc(void *id_work_mem, gpImage *org_img, void *pre_result)
{
	ObjResult_t *p_obj_result = (ObjResult_t *)pre_result;
	FaceIdentify_t *idWorkMem = (FaceIdentify_t *)id_work_mem;
	INT32S ret,identify_score[FD_TRAINING_IMAGE_NUMBER];
	//INT32U i,j;
	gpRect image[3];
	INT32S identify_index;
	//INT32S y_offset = 0;
	INT32U temp;
    static INT32U face_training_cnt = 0;
    //INT32U t1,t2;

	if(p_obj_result->is_best_face && (idWorkMem->id_mode == 1 || idWorkMem->id_mode == 2))
	{
		// face ROI
		image[0] = p_obj_result->rect[FACE_ROI];
		// leye	ROI
		image[1] = p_obj_result->rect[LEYE_ROI];
		// reye ROI
		image[2] = p_obj_result->rect[REYE_ROI];

		/*
		DBG_PRINT("lEyeROI.x = %d\r\n", image[1].x);
		DBG_PRINT("lEyeROI.y = %d\r\n", image[1].y);
		DBG_PRINT("lEyeROI.width = %d\r\n", image[1].width);
		DBG_PRINT("lEyeROI.height = %d\r\n", image[1].height);
		DBG_PRINT("rEyeROI.x = %d\r\n", image[2].x);
		DBG_PRINT("rEyeROI.y = %d\r\n", image[2].y);
		DBG_PRINT("rEyeROI.width = %d\r\n", image[2].width);
		DBG_PRINT("rEyeROI.height = %d\r\n", image[2].height);
		*/
	}

	if(p_obj_result->is_best_face == 0) {
		//no face
		idWorkMem->identify_cnt = 0;
		return;
	} else if(idWorkMem->id_mode == 0) {
		// idle
		idWorkMem->training_cnt = 0;
		//face_training_cnt = idWorkMem->training_cnt;
		idWorkMem->identify_cnt = 0;
		return;
	} else if(idWorkMem->id_mode == 1) {
		// traning

		DBG_PRINT("faceROI.x = %d\r\n", image[0].x);
	    DBG_PRINT("faceROI.y = %d\r\n", image[0].y);
	    DBG_PRINT("faceROI.width = %d\r\n", image[0].width);
	    DBG_PRINT("faceROI.height = %d\r\n", image[0].height);

		FaceIdentify_Train(org_img,
						&image[0],
						idWorkMem->ownerULBP,
						idWorkMem->training_cnt,
						idWorkMem->identify_WorkMem);
		idWorkMem->training_cnt++;
		face_training_cnt = idWorkMem->training_cnt;
		DBG_PRINT("training_cnt = %d\r\n", idWorkMem->training_cnt);
		if(idWorkMem->training_cnt >= FD_TRAINING_IMAGE_NUMBER) {
			DBG_PRINT("Training Success\r\n");
			drv_fd_result_lock();
			idWorkMem->id_mode = 2;
			fd_mode = idWorkMem->id_mode;
			drv_fd_result_unlock();
			idWorkMem->training_cnt = 0;
			idWorkMem->identify_cnt = 0;
			idWorkMem->training_state = 1;
			fd_result.training_ok = 1;
			if(idWorkMem->pFuncStore) {
				idWorkMem->pFuncStore((INT32U)idWorkMem->ownerULBP, SZ_ONEDATA*(FD_TRAINING_IMAGE_NUMBER + 1));
			}
		}
	} else if(idWorkMem->id_mode == 2) {
		// identify
       /*
           0 : 0
           1 : 1
           2 : -1
           3 : (1 > 2)?     2   :  -2
           4 : (1 > 3)? -2 : 3  / (2 > 3)? 2 : -3
           5 :          -3 & 4  /          3 : -4
       */
        drv_fd_result_lock();
        temp = idWorkMem->security_value;
        drv_fd_result_unlock();
#if 1
		ret = FD_Verify_retry(org_img,
							&image[0],
							idWorkMem->ownerULBP,
							temp,
							idWorkMem->identify_WorkMem,
							FD_TRAINING_IMAGE_NUMBER,
							FD_TRAINING_IMAGE_NUMBER,
							(int *)&identify_index,
						    (int *)&identify_score[0]);
#else
		ret = FaceIdentify_Verify(org_img,
							&image[0],
							idWorkMem->ownerULBP,
							temp,
							idWorkMem->identify_WorkMem,
							FD_TRAINING_IMAGE_NUMBER,
							FD_TRAINING_IMAGE_NUMBER,
							(int *)&identify_index,
						    (int *)&identify_score[0]);
#endif
		idWorkMem->recognize_score = identify_score[0];

		if(identify_score[0] > gFDscore)
        {
             gFDscore = identify_score[0];
        }
		DBG_PRINT("max_score = %d\r\n",identify_score[0]);

		#if DEMO_DEBUG_EN == 1
			DBG_PRINT("Face_Identify_Index = %d\r\n", (identify_index));
			DBG_PRINT("Face_Identify_Score = %d\r\n", identify_score[0]);
		#endif

		if(ret == 0) {
			DBG_PRINT("NoID\r\n");
			return;
		}

		idWorkMem->identify_cnt++;
        #if DEMO_DEBUG_EN == 1
            DBG_PRINT("identify_cnt = %d\r\n", idWorkMem->identify_cnt);
		#endif
		if(idWorkMem->identify_cnt >= 2) {
			DBG_PRINT("Identify Success\r\n");
			drv_fd_result_lock();
			idWorkMem->id_mode = 0; //idle mode
			fd_mode = idWorkMem->id_mode;
			drv_fd_result_unlock();
			idWorkMem->training_cnt = 0;
			face_training_cnt = idWorkMem->training_cnt;
			idWorkMem->identify_cnt = 0;
			idWorkMem->identify_state = 1;
			fd_result.identify_ok = 1;
		}
	} else {
		while(1);
	}
}

static INT32S gp_face_detect_init(ObjDetect_t *odWorkMem)
{
	INT32U mem_size;
	gpImage *image;

	image = &odWorkMem->image;
	mem_size = FaceDetect_Config(0, 0, image->width, image->height,
								image->ch, OBJDETECT_MAX_RESULT, odWorkMem->xstep, odWorkMem->ystep);

	odWorkMem->obj_detect_WorkMem = gp_malloc_align(mem_size, 16);
	if(odWorkMem->obj_detect_WorkMem == 0) {
		DBG_PRINT("Fail to allocate obj_detect_WorkMem\r\n");
		return -1;
	}
	gp_memset((INT8S*)odWorkMem->obj_detect_WorkMem, 0x00, mem_size);
	odWorkMem->WorkMemSize = mem_size;
	DBG_PRINT("WorkMemSize = %d\r\n",(odWorkMem->WorkMemSize));
	mem_size = FaceDetect_Config(odWorkMem->obj_detect_WorkMem, mem_size, image->width, image->height,
								image->ch, OBJDETECT_MAX_RESULT, odWorkMem->xstep, odWorkMem->ystep);

	odWorkMem->equalizeHist_WorkMem = gp_malloc_align(320*240, 16);
	if(odWorkMem->equalizeHist_WorkMem == 0) {
		DBG_PRINT("Fail to allocate equalizeHist_WorkMem\r\n");
		return -1;
	}

	odWorkMem->EyeequalizeHist_WorkMem = gp_malloc_align(320*240, 16);
	if(odWorkMem->EyeequalizeHist_WorkMem == 0) {
		DBG_PRINT("Fail to allocate EyeequalizeHist_WorkMem\r\n");
		return -1;
	}

	gp_memset((INT8S*)odWorkMem->equalizeHist_WorkMem, 0x00, 320*240);

	FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	// setting cascade type
	FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);

	return 0;
}

static void gp_face_detect_free(ObjDetect_t *odWorkMem)
{
	// free memory
	if(odWorkMem) {
	    if(odWorkMem->obj_detect_WorkMem) {
			gp_free(odWorkMem->obj_detect_WorkMem);
			odWorkMem->obj_detect_WorkMem = 0;
		}

	    if(odWorkMem->equalizeHist_WorkMem) {
			gp_free(odWorkMem->equalizeHist_WorkMem);
			odWorkMem->equalizeHist_WorkMem = 0;
		}

	    if(odWorkMem->EyeequalizeHist_WorkMem) {
			gp_free(odWorkMem->EyeequalizeHist_WorkMem);
			odWorkMem->EyeequalizeHist_WorkMem = 0;
		}
        DBG_PRINT("odWorkMem = 0x%x\r\n",odWorkMem);
		gp_free(odWorkMem);
		odWorkMem = 0;
	}
}

static INT32S gp_face_detect_alloc(void)
{
	INT32S nRet;
	ObjDetect_t *odWorkMem;

	odWorkMem = gp_malloc_align(sizeof(ObjDetect_t), 16);
	if(odWorkMem == 0) {
		DBG_PRINT("Fail to allocate odWorkMem\r\n");
		return 0;
	}
	DBG_PRINT("odWorkMem = 0x%x\r\n", odWorkMem);
	gp_memset((INT8S*)odWorkMem, 0, sizeof(ObjDetect_t));

	// image info setting
	odWorkMem->image.width     = FD_SIZE_HPIXEL;
	odWorkMem->image.height    = FD_SIZE_VPIXEL;
	odWorkMem->image.ch        = 1;
	odWorkMem->image.widthStep = FD_SIZE_HPIXEL;
	odWorkMem->image.format    = IMG_FMT_GRAY;

	// setting cascade type
	odWorkMem->clf.obj_type 		 = OBJ_FACE;
	odWorkMem->clf.data              = 0;
	odWorkMem->clf.stage_of_classify = 0;
	odWorkMem->clf.num_of_classify   = 0;

	//scan step and scale size
	odWorkMem->xstep        		 = 2;
	odWorkMem->ystep        		 = 2;
	odWorkMem->scale                 = (INT32S)(1.1*65536);

	// allocate memory & initial
	if(gp_face_detect_init(odWorkMem) < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
	if(nRet < 0) {
		gp_face_detect_free((void *)odWorkMem);
		return 0;
	}

	return (INT32S)odWorkMem;
}

static int gp_faceRoiDetect(ObjDetect_t *odWorkMem,gpImage* gray, gpRect* userROI, int mode)
{
	#define MAX_RESULT 		OBJDETECT_MAX_RESULT
	FaceIdentify_t *idWorkMem = odWorkMem->post_proc.workmem;
	// detect type setting
	ClassifyData clfface;
	ClassifyData clfreye;
	ClassifyData clfleye;
#if DEMO_DEBUG_EN == 1
	INT32U  t1,t2;
#endif
	// best face reye and leye
	gpRect* faceROI = &userROI[FACE_ROI];
	gpRect* lEyeROI = &userROI[LEYE_ROI];
	gpRect* rEyeROI = &userROI[REYE_ROI];
	gpRect* best_faceROI = &userROI[BESTFACE_ROI];
	gpImage equalizeHist_image;

	// detect result and count
	gpRect Rect,rFace,lFace;
	gpRect faceResult[MAX_RESULT];
	gpRect rEyeResult[MAX_RESULT];
	gpRect lEyeResult[MAX_RESULT];
	int faceCount[MAX_RESULT];
	int rEyeCount[MAX_RESULT];
	int lEyeCount[MAX_RESULT];
	INT32U T1,T2;
	INT8U  FaceDetectCnt;
	INT8U  FaceDetectFlag;
	//new ROI 20150506
	//////////////////////////////////////////////////////////
	int lowerbound = 0;
	////////////////////////////
	// Initialization //
	int i, maxFaceW, maxFaceCount, best_face, offset_x, offset_y, min_eye_wnd;
	int ret, faceN, rEyeN, lEyeN, int_scale_face, int_scale_reye,int_scale_leye, detectH;
	// xstep
	int xstep_face = 2;
	int xstep_reye = 1;
	int xstep_leye = 1;
	// ystep
	int ystep_face = 2;
	int ystep_reye = 2;
	int ystep_leye = 2;
	// face nbr
	int min_face_nbr_h = 5;
	int min_face_nbr_l = 4;//2;
	// eye nbr
	int min_eye_nbr = 1;
	// min face window
	int min_face_wnd = 50;
	int maxEyeCount;
	// sacler max window
	int max_wnd = MIN(gray->width, gray->height);
	gpImage *pFDImage;
	INT8U do_eq,do_gamma;

	static INT8U first_flag = 1;
	pFDImage = gray;

#if EQUALIZEHIST_EN == 1
	equalizeHist_image.width = odWorkMem->image.width;
	equalizeHist_image.height = odWorkMem->image.height;
	equalizeHist_image.widthStep = odWorkMem->image.widthStep;
	equalizeHist_image.ch = odWorkMem->image.ch;
	equalizeHist_image.format = odWorkMem->image.format;
	equalizeHist_image.ptr = (unsigned char *)odWorkMem->equalizeHist_WorkMem;

	/*Rect.x = gray->width/4;
	Rect.y = gray->height/4;
	Rect.width = gray->width/2;
	Rect.height = gray->height/2;*/
	Rect.width = 140;
    Rect.height = 140;
    Rect.x = 83;
    Rect.y = 54;

    if(1 == first_flag)
	{

	    DBG_PRINT("equalizeHist Rect.x = %d\r\n", Rect.x);
	    DBG_PRINT("equalizeHist Rect.y = %d\r\n", Rect.y);
	    DBG_PRINT("equalizeHist Rect.width = %d\r\n", Rect.width);
	    DBG_PRINT("equalizeHist Rect.height = %d\r\n\r\n", Rect.height);
    }
#if DEMO_DEBUG_EN == 1
	t1 = xTaskGetTickCount();
#endif
	equalizeHist((gpImage *)gray, (gpImage *)&equalizeHist_image,(gpRect *)&Rect);
#if DEMO_DEBUG_EN == 1
	t2 = xTaskGetTickCount();
	DBG_PRINT("equalizeHist = %d\r\n", t2-t1);
#endif
	pFDImage = &equalizeHist_image;
#endif

#if USER_DETECT_RANGE_EN == 1
		// face
		xstep_face = 2;
		ystep_face = 2;
		// eye
		xstep_reye = 1;
		ystep_reye = 1;

		xstep_leye = 1;
		ystep_leye = 1;
#if 1
		Rect.x = 0+(gray->width)*0.075f;
		Rect.y = 0;
		Rect.width = gray->width-(gray->width)*0.15f;
		Rect.height = gray->height;
#else
        Rect.x = 0;
		Rect.y = 0;
		Rect.width = gray->width;
		Rect.height = gray->height;
#endif
		int_scale_face = 74054;
		int_scale_reye = 68811;
		int_scale_leye = 68811;
		min_face_wnd = 90;
#else
		Rect.x = 0;
		Rect.y = 0;
		Rect.width = gray->width;
		Rect.height = gray->height;

	    // face
		xstep_face = 2;
		ystep_face = 2;
		// eye
		xstep_reye = 2;
		ystep_reye = 2;

        xstep_leye = 2;
		ystep_leye = 2;

		int_scale_face = 78643;//72089;//74054;//72089; // 1.1  78643:1.2
		int_scale_reye = 72089;//72089; // 1.1, 65535 = 1
		int_scale_leye = 72089;//72089; // 1.1, 65535 = 1
		min_face_wnd = 50;
#endif

	max_wnd = MIN(Rect.width, Rect.height);

	//--------------------
	//	Face Detection
	//--------------------
	if(1 == first_flag)
	{
	    first_flag = 0;
		DBG_PRINT("\r\nxstep_face = %d\r\n", xstep_face);
	    DBG_PRINT("ystep_face = %d\r\n", ystep_face);
	    DBG_PRINT("xstep_reye = %d\r\n", xstep_reye);
	    DBG_PRINT("ystep_reye = %d\r\n", ystep_reye);
	    DBG_PRINT("xstep_leye = %d\r\n", xstep_leye);
	    DBG_PRINT("ystep_leye = %d\r\n", ystep_leye);
		DBG_PRINT("int_scale_face = %d\r\n", int_scale_face);
	    DBG_PRINT("int_scale_reye = %d\r\n", int_scale_reye);
	    DBG_PRINT("int_scale_leye = %d\r\n", int_scale_leye);
	    DBG_PRINT("min_face_wnd = %d\r\n", min_face_wnd);

	    DBG_PRINT("Rect.x = %d\r\n", Rect.x);
	    DBG_PRINT("Rect.y = %d\r\n", Rect.y);
	    DBG_PRINT("Rect.width = %d\r\n", Rect.width);
	    DBG_PRINT("Rect.height = %d\r\n\r\n", Rect.height);

	    DBG_PRINT("MAX_RESULT = %d \r\n",MAX_RESULT);
	    DBG_PRINT("USER_DETECT_RANGE_EN = %d \r\n",USER_DETECT_RANGE_EN);
	    DBG_PRINT("EQUALIZEHIST_EN = %d \r\n",EQUALIZEHIST_EN);
	    DBG_PRINT("SCALER_INIT_EN = %d \r\n",SCALER_INIT_EN);
	    DBG_PRINT("SCALER_LINEAR_EN = %d \r\n\r\n",SCALER_LINEAR_EN);
    }

#if 1
	gp_memset((INT8S*)odWorkMem->obj_detect_WorkMem, 0x00, odWorkMem->WorkMemSize);
//	ret = FaceDetect_Config(odWorkMem->obj_detect_WorkMem, odWorkMem->WorkMemSize, Rect.width, Rect.height, 1, MAX_RESULT, xstep_face, ystep_face);
	ret = FaceDetect_Config(odWorkMem->obj_detect_WorkMem, odWorkMem->WorkMemSize, gray->width/*Rect.width*/, gray->height/*Rect.height*/, 1, MAX_RESULT, xstep_face, ystep_face);
	/* setting cascade type (face) */
	clfface.obj_type = OBJ_FACE;
	FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &clfface);
    ret = FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);
	FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep_face, 0);

	FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	FaceDetectCnt = 0;
	FaceDetectFlag = 0;
	do
	{
	#if DEMO_DEBUG_EN == 1
		t1 = xTaskGetTickCount();
	#endif

    #if EQUALIZEHIST_EN == 1
		faceN = FaceDetect(odWorkMem->obj_detect_WorkMem, &equalizeHist_image, &Rect, MAX_RESULT, faceResult, faceCount);
	#else
        faceN = FaceDetect(odWorkMem->obj_detect_WorkMem, pFDImage, &Rect, MAX_RESULT, faceResult, faceCount);
	#endif

	#if DEMO_DEBUG_EN == 1
	    t2 = xTaskGetTickCount();
	    DBG_PRINT("OBJ_FACE = %d,faceN = %d\r\n",(t2 - t1),faceN);
	#endif
	    FaceDetectCnt ++;

	    if(0 < faceN)
	    {
        	maxFaceW = 0;
			maxFaceCount = min_face_nbr_l;
			best_face = 0;
			i = faceN-1;

			do
			{
            #if DEMO_DEBUG_EN == 1
			    DBG_PRINT("faceCount[%d] = %d\r\n",i,faceCount[i]);
			#endif
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

			if(maxFaceW)
			{
				FaceDetectFlag = 1;
			}
			else
                faceROI->x = 0;
	    }

		if (!FaceDetectFlag)
		{
		    if(2 > FaceDetectCnt)
		    {
		    #if 1//EQUALIZEHIST_EN == 1
		        gpRect EqRect;
		        gpRect applyRect;
				equalizeHist_image.width = odWorkMem->image.width;
				equalizeHist_image.height = odWorkMem->image.height;
				equalizeHist_image.widthStep = odWorkMem->image.widthStep;
				equalizeHist_image.ch = odWorkMem->image.ch;
				equalizeHist_image.format = odWorkMem->image.format;
				equalizeHist_image.ptr = (unsigned char *)odWorkMem->equalizeHist_WorkMem;

				/*Rect.x = gray->width/4;
				Rect.y = gray->height/4;
				Rect.width = gray->width/2;
				Rect.height = gray->height/2;*/
				EqRect.width = 140;
			    EqRect.height = 140;
			    EqRect.x = 83;
			    EqRect.y = 54;

                applyRect.width = 320;//200;
			    applyRect.height = 240;
			    applyRect.x = 0;//60;
			    applyRect.y = 0;
			    if(1 == first_flag)
				{

				    DBG_PRINT("equalizeHist Rect.x = %d\r\n", EqRect.x);
				    DBG_PRINT("equalizeHist Rect.y = %d\r\n", EqRect.y);
				    DBG_PRINT("equalizeHist Rect.width = %d\r\n", EqRect.width);
				    DBG_PRINT("equalizeHist Rect.height = %d\r\n\r\n", EqRect.height);
			    }
            #if DEMO_DEBUG_EN == 1
				T1 = xTaskGetTickCount();
			#endif
				//equalizeHist_new((gpImage *)gray, (gpImage *)&equalizeHist_image,(gpRect *)&EqRect,(gpRect *)&applyRect);
				equalizeHist((gpImage *)gray, (gpImage *)&equalizeHist_image,(gpRect *)&EqRect);
				//ModEqualizeHist_fix((gpImage *)gray, (gpImage *)&equalizeHist_image, &EqRect);
			#if DEMO_DEBUG_EN == 1
				T2 = xTaskGetTickCount();
				DBG_PRINT("equalizeHist = %d\r\n", t2-t1);
			#endif
            #endif
		        pFDImage = &equalizeHist_image;
		     }
		     else
		     {
			    return 0;
             }
		}
		else
		{
		    FaceDetectCnt ++;
		}
	}while(2 > FaceDetectCnt);
#endif

	offset_x = faceResult[best_face].width * 0.4;
	offset_y = faceResult[best_face].height *0.4;

#if 1
    Rect.x = faceResult[best_face].x - offset_x;
	Rect.y = faceResult[best_face].y - offset_y;
    Rect.width = faceResult[best_face].width + offset_x * 2;
    Rect.height = faceResult[best_face].height + offset_y * 2;

    if(Rect.x < 0)
    {
        Rect.x = 0;
    }

    if(Rect.y < 0)
    {
        Rect.y = 0;
    }
    if((Rect.width + Rect.x) > (gray->width + 0))
    {
        Rect.width = gray->width - Rect.x;
    }
    if((Rect.height  + Rect.y) > (gray->height + 0))
    {
        Rect.height = gray->height - Rect.y;
    }


    if((Rect.width + Rect.x) >= (gray->width + 0))
    {
        Rect.width = gray->width;
        Rect.x = 0;
    }
    else if(Rect.x < 0)
    {
        Rect.width -= Rect.x;
        Rect.x = 0;

    }

    if((Rect.height  + Rect.y) >= (gray->height + 0))
    {
        Rect.height = gray->height;
        Rect.y = 0;
    }
    else if(Rect.y < 0)
    {
        Rect.height -= Rect.y;
        Rect.y = 0;

    }
    min_face_wnd = faceResult[best_face].width / 1.6;
	if(min_face_wnd < 50) min_face_wnd = 50;
#else
    Rect.x = 0;
	Rect.y = 0;
    Rect.width = 320;
    Rect.height = 240;

    min_face_wnd = 50;
#endif

    int_scale_face = 72089;

    max_wnd = MIN(Rect.width, Rect.height);

#if DEMO_DEBUG_EN == 1
	DBG_PRINT("faceResult[best_face].x = %d\r\n",faceResult[best_face].x);
	DBG_PRINT("faceResult[best_face].y = %d\r\n",faceResult[best_face].y);
	DBG_PRINT("faceResult[best_face].width = %d\r\n",faceResult[best_face].width);
	DBG_PRINT("faceResult[best_face].height = %d\r\n",faceResult[best_face].height);
	DBG_PRINT("Rect.x = %d\r\n",Rect.x);
	DBG_PRINT("Rect.y = %d\r\n",Rect.y);
	DBG_PRINT("Rect.width = %d\r\n",Rect.width);
	DBG_PRINT("Rect.height = %d\r\n",Rect.height);
	DBG_PRINT("max_wnd = %d\r\n",max_wnd);
	DBG_PRINT("min_face_wnd = %d\r\n",min_face_wnd);
	DBG_PRINT("int_scale_face = %d\r\n",int_scale_face);
#endif

	gp_memset((INT8S*)odWorkMem->obj_detect_WorkMem, 0x00, odWorkMem->WorkMemSize);
	ret = FaceDetect_Config(odWorkMem->obj_detect_WorkMem, odWorkMem->WorkMemSize, gray->width/*Rect.width*/, gray->height/*Rect.height*/, 1, MAX_RESULT, xstep_face, ystep_face);
	/* setting cascade type (face) */
	clfface.obj_type = OBJ_FACE;
	FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &clfface);
    ret = FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);
	FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep_face, 0);

	FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	FaceDetectCnt = 0;
	FaceDetectFlag = 0;
	do
	{
	#if DEMO_DEBUG_EN == 1
		t1 = xTaskGetTickCount();
	#endif

#if EQUALIZEHIST_EN == 1
		faceN = FaceDetect(odWorkMem->obj_detect_WorkMem, &equalizeHist_image, &Rect, MAX_RESULT, faceResult, faceCount);
#else
        faceN = FaceDetect(odWorkMem->obj_detect_WorkMem, pFDImage, &Rect, MAX_RESULT, faceResult, faceCount);
#endif

#if DEMO_DEBUG_EN == 1
	    t2 = xTaskGetTickCount();
	    DBG_PRINT("OBJ_FACE = %d,faceN = %d\r\n",(t2 - t1),faceN);
#endif
	    FaceDetectCnt ++;

	    if(0 < faceN)
	    {
        	maxFaceW = 0;
			maxFaceCount = min_face_nbr_l;
			best_face = 0;
			i = faceN-1;

			do
			{
            #if DEMO_DEBUG_EN == 1
			    DBG_PRINT("faceCount[%d] = %d\r\n",i,faceCount[i]);
			#endif
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

			if(maxFaceW)
			{
				FaceDetectFlag = 1;
			}
            else
                faceROI->x = 0;
	    }

		if (!FaceDetectFlag)
		{
        #if 0
		    if(2 > FaceDetectCnt)
		    {
		    #if 1
		        gpRect EqRect;
				equalizeHist_image.width = odWorkMem->image.width;
				equalizeHist_image.height = odWorkMem->image.height;
				equalizeHist_image.widthStep = odWorkMem->image.widthStep;
				equalizeHist_image.ch = odWorkMem->image.ch;
				equalizeHist_image.format = odWorkMem->image.format;
				equalizeHist_image.ptr = (unsigned char *)odWorkMem->equalizeHist_WorkMem;

				EqRect.width = 140;
			    EqRect.height = 140;
			    EqRect.x = 83;
			    EqRect.y = 54;

			    if(1 == first_flag)
				{

				    DBG_PRINT("equalizeHist Rect.x = %d\r\n", EqRect.x);
				    DBG_PRINT("equalizeHist Rect.y = %d\r\n", EqRect.y);
				    DBG_PRINT("equalizeHist Rect.width = %d\r\n", EqRect.width);
				    DBG_PRINT("equalizeHist Rect.height = %d\r\n\r\n", EqRect.height);
			    }

				t1 = xTaskGetTickCount();
				//ModEqualizeHist_fix((gpImage *)gray, (gpImage *)&equalizeHist_image, &EqRect);
				equalizeHist((gpImage *)gray, (gpImage *)&equalizeHist_image,(gpRect *)&EqRect);
				//equalizeHist_new((gpImage *)gray, (gpImage *)&equalizeHist_image,(gpRect *)&EqRect);
				t2 = xTaskGetTickCount();
//				DBG_PRINT("equalizeHist = %d\r\n", T2-T1);
			 #endif
		        pFDImage = &equalizeHist_image;
		     }
		     else
        #endif
		     {
			    faceROI->x = 0;
			    return 0;
             }
		}
		else
		{
		    FaceDetectCnt ++;
		}
	}while(2 > FaceDetectCnt);

	best_faceROI->x = faceResult[best_face].x;
	best_faceROI->y = faceResult[best_face].y;
	best_faceROI->width = faceResult[best_face].width;
	best_faceROI->height = faceResult[best_face].height;


	/* Face Position Determination */
	offset_x = faceResult[best_face].width * 0.16;
	offset_y = faceResult[best_face].height/7;

	faceROI->x = faceResult[best_face].x + offset_x;
	faceROI->y = faceResult[best_face].y + offset_y;
	faceROI->width = (short)(faceResult[best_face].width - (offset_x<<1));
	faceROI->height = (short)(faceResult[best_face].height - offset_y*1.5);

	//--------------------
	//	Eyes Detection
	//--------------------
#if USER_DETECT_RANGE_EN == 1
	min_eye_wnd = MAX(faceROI->width/7, 20); // 20 is minimum
#else
	min_eye_wnd = MAX(faceROI->width/7, 24); // 24 is minimum
#endif
	//Eyes detect range
	detectH = (int)(faceResult[best_face].height*0.6);

	FaceDetectCnt = 0;
	do
	{
		FaceDetectCnt ++;
		gp_memset((INT8S*)odWorkMem->obj_detect_WorkMem, 0x00, odWorkMem->WorkMemSize);
		ret = FaceDetect_Config(odWorkMem->obj_detect_WorkMem, odWorkMem->WorkMemSize, (faceResult[best_face].width>>1), detectH, 1, MAX_RESULT, xstep_reye, ystep_reye);
        if(ret < 0)
        {
			return 0;
        }
		// setting cascade type (right eye) //
		clfreye.obj_type = OBJ_REYE;
		FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &clfreye);

		FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	    //DBG_PRINT("int_scale_reye = %d,min_eye_wnd = %d,max_wnd = %d\r\n",int_scale_reye,min_eye_wnd,max_wnd);
		ret = FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, int_scale_reye, min_eye_wnd, max_wnd, mode);

		rFace = GPRect_utils(faceResult[best_face].x + (faceResult[best_face].width>>1), faceResult[best_face].y, (faceResult[best_face].width>>1), detectH);

		FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep_reye, 0);
    #if DEMO_DEBUG_EN == 1
		t1 = xTaskGetTickCount();
	#endif

	#if 0//EQUALIZEHIST_EN == 1
		rEyeN = FaceDetect(odWorkMem->obj_detect_WorkMem, &equalizeHist_image, &rFace, MAX_RESULT, rEyeResult, rEyeCount);
	#else
		rEyeN = FaceDetect(odWorkMem->obj_detect_WorkMem, pFDImage, &rFace, MAX_RESULT, rEyeResult, rEyeCount);
	#endif

	#if DEMO_DEBUG_EN == 1
	    t2 = xTaskGetTickCount();
	    DBG_PRINT("OBJ_REYE = %d,rEyeN = %d\r\n",(t2 - t1),rEyeN);
	#endif

	    maxEyeCount = min_eye_nbr;
	    if(0 >= rEyeN)
		{
			rEyeROI->width = 0;
		}
		else
		{
			int i = rEyeN - 1;
			do
			{
			#if DEMO_DEBUG_EN == 1
			    DBG_PRINT("rEyeCount[%d] = %d\r\n",i,rEyeCount[i]);
			#endif
				if (rEyeCount[i] > maxEyeCount)
				{
					maxEyeCount = rEyeCount[i];
				}
			} while (i--);
		}

	    if(2 > FaceDetectCnt)
	    {
		    if(maxEyeCount <= min_eye_nbr)
		    {
		        INT32U x,y;
		        gpRect applyROI;
		        INT8U *p_old,*p_new;

		        applyROI.x = 0;
		        applyROI.y = 0;
		        applyROI.width = 320;
		        applyROI.height = 240;

				equalizeHist_image.width = odWorkMem->image.width;
				equalizeHist_image.height = odWorkMem->image.height;
				equalizeHist_image.widthStep = odWorkMem->image.widthStep;
				equalizeHist_image.ch = odWorkMem->image.ch;
				equalizeHist_image.format = odWorkMem->image.format;
				equalizeHist_image.ptr = (unsigned char *)odWorkMem->equalizeHist_WorkMem;

            #if DEMO_DEBUG_EN == 1
                t1 = xTaskGetTickCount();
            #endif

	            //ModEqualizeHist_fix((gpImage *)gray, (gpImage *)&equalizeHist_image, &rFace);
	            //equalizeHist((gpImage *)gray, (gpImage *)&equalizeHist_image, &rFace);
	            equalizeHist_new2((gpImage *)gray, (gpImage *)&equalizeHist_image, &rFace, &applyROI);

	            equalizeHist_image.width = odWorkMem->image.width;
				equalizeHist_image.height = odWorkMem->image.height;
				equalizeHist_image.widthStep = odWorkMem->image.widthStep;
				equalizeHist_image.ch = odWorkMem->image.ch;
				equalizeHist_image.format = odWorkMem->image.format;
				equalizeHist_image.ptr = (unsigned char *)odWorkMem->EyeequalizeHist_WorkMem;

	            equalizeHist_new((gpImage *)gray, (gpImage *)&equalizeHist_image, &rFace, &applyROI);

            #if DEMO_DEBUG_EN == 1
	            t2 = xTaskGetTickCount();
	            DBG_PRINT("rEyeN EQ = %d\r\n",(t2 - t1));
            #endif

            #if DEMO_DEBUG_EN == 1
	            t1 = xTaskGetTickCount();
	        #endif
	            //GammaCorrection((gpImage *)&equalizeHist_image, (gpImage *)&equalizeHist_image, &rFace);
	            pFDImage = &equalizeHist_image;

	        #if DEMO_DEBUG_EN == 1
	            t2 = xTaskGetTickCount();
	            DBG_PRINT("rEyeN Gamma = %d\r\n",(t2 - t1));
	        #endif

		        /*xstep_reye = 1;
				ystep_reye = 1;*/
				int_scale_reye = 68811;
		        //DBG_PRINT("rEyeN <= 50, xstep_reye %d,ystep_reye= %d,int_scale_reye = %d\r\n",xstep_reye,ystep_reye,int_scale_reye);
	        }
		    else if(maxEyeCount <= 50)
		    {
		        /*xstep_reye = 1;
				ystep_reye = 1;*/
				int_scale_reye = 68811;
            #if DEMO_DEBUG_EN == 1
		        DBG_PRINT("rEyeN <= 50, xstep_reye %d,ystep_reye= %d,int_scale_reye = %d\r\n",xstep_reye,ystep_reye,int_scale_reye);
            #endif
			}
			else
			{
			    FaceDetectCnt ++;
			}
		}
	}while(2 > FaceDetectCnt);


	FaceDetectCnt = 0;
	do
	{
		FaceDetectCnt ++;
		gp_memset((INT8S*)odWorkMem->obj_detect_WorkMem, 0x00, odWorkMem->WorkMemSize);
		ret = FaceDetect_Config(odWorkMem->obj_detect_WorkMem, odWorkMem->WorkMemSize, (faceResult[best_face].width>>1), detectH, 1, MAX_RESULT, xstep_leye, ystep_leye);
		if(ret < 0)
        {
			return 0;
        }

		// setting cascade type (left eye) //
		clfleye.obj_type = OBJ_LEYE;
		FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &clfleye);
		FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

#if DEMO_DEBUG_EN == 1
	    DBG_PRINT("int_scale_leye = %d,min_eye_wnd = %d,max_wnd = %d\r\n",int_scale_leye,min_eye_wnd,max_wnd);
#endif
		ret = FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, int_scale_leye, min_eye_wnd, max_wnd, mode);

        lFace = GPRect_utils(faceResult[best_face].x, faceResult[best_face].y, (faceResult[best_face].width>>1), detectH);
		//lFace = GPRect_utils(faceResult[best_face].x + (faceResult[best_face].width>>1), faceResult[best_face].y, (faceResult[best_face].width>>1), detectH);
		FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep_leye, 0);
#if DEMO_DEBUG_EN == 1
	    t1 = xTaskGetTickCount();
#endif

	#if 0//EQUALIZEHIST_EN == 1
		lEyeN = FaceDetect(odWorkMem->obj_detect_WorkMem, &equalizeHist_image, &lFace, MAX_RESULT, lEyeResult, lEyeCount);
	#else
		lEyeN = FaceDetect(odWorkMem->obj_detect_WorkMem, pFDImage, &lFace, MAX_RESULT, lEyeResult, lEyeCount);
	#endif

#if DEMO_DEBUG_EN == 1
	    t2 = xTaskGetTickCount();
	    DBG_PRINT("OBJ_LEYE = %d,lEyeN = %d\r\n",(t2 - t1),lEyeN);
#endif

	    maxEyeCount = min_eye_nbr;
	    if(0 >= lEyeN)
		{
			lEyeROI->width = 0;
		}
		else
		{
			int i = lEyeN - 1;
			do
			{
            #if DEMO_DEBUG_EN == 1
			    DBG_PRINT("lEyeCount[%d] = %d\r\n",i,lEyeCount[i]);
            #endif
				if (lEyeCount[i] > maxEyeCount)
				{
					maxEyeCount = lEyeCount[i];
				}
			} while (i--);
		}

	    if(2 > FaceDetectCnt)
	    {

		    if(maxEyeCount <= min_eye_nbr)
		    {

		        {
		            //pFDImage = gray;

					equalizeHist_image.width = odWorkMem->image.width;
					equalizeHist_image.height = odWorkMem->image.height;
					equalizeHist_image.widthStep = odWorkMem->image.widthStep;
					equalizeHist_image.ch = odWorkMem->image.ch;
					equalizeHist_image.format = odWorkMem->image.format;
					equalizeHist_image.ptr = (unsigned char *)odWorkMem->equalizeHist_WorkMem;
                #if DEMO_DEBUG_EN == 1
                    t1 = xTaskGetTickCount();
                #endif
		            //ModEqualizeHist_fix((gpImage *)gray, (gpImage *)&equalizeHist_image, &lFace);
		            equalizeHist((gpImage *)gray, (gpImage *)&equalizeHist_image, &lFace);
		            //equalizeHist_new((gpImage *)gray, (gpImage *)&equalizeHist_image, &lFace, &lFace);
		        #if DEMO_DEBUG_EN == 1
		            t2 = xTaskGetTickCount();
		            DBG_PRINT("lEyeN EQ = %d\r\n",(t2 - t1));
		        #endif
		            pFDImage = &equalizeHist_image;
		            do_eq = 1;
		        }
		        //else if(0 == do_gamma)
		        {
                #if DEMO_DEBUG_EN == 1
		            t1 = xTaskGetTickCount();
		        #endif

		            GammaCorrection((gpImage *)&equalizeHist_image, (gpImage *)&equalizeHist_image, &lFace);

		        #if DEMO_DEBUG_EN == 1
		            t2 = xTaskGetTickCount();
		            DBG_PRINT("lEyeN Gamma = %d\r\n",(t2 - t1));
		        #endif
		            do_gamma = 1;
		        }
		        //else
		        {
			        /*xstep_reye = 1;
					ystep_reye = 1;*/
					int_scale_leye = 68811;
			        //DBG_PRINT("lEyeN <= 50, xstep_leye %d,ystep_leye= %d,int_scale_leye = %d\r\n",xstep_leye,ystep_leye,int_scale_leye);
			    }
		    }
		    else if(maxEyeCount <= 50)
		    {
				int_scale_leye = 68811;
		        //DBG_PRINT("lEyeN <= 50, xstep_leye %d,ystep_leye= %d,int_scale_leye = %d\r\n",xstep_leye,ystep_leye,int_scale_leye);
			}
			else
			{
			    FaceDetectCnt ++;
			}
		}
	}while(2 > FaceDetectCnt);


	if(0 >= rEyeN)
	{
		rEyeROI->width = 0;
	}
	else
	{
		int minEyeDist = INT_MAX;
		int maxEyeCount = min_eye_nbr;
		int most_possible_eye = 0;

		int i = rEyeN - 1;
		do
		{
		    //DBG_PRINT("rEyeCount[%d] = %d\r\n",i,rEyeCount[i]);
			if (rEyeCount[i] > maxEyeCount)
			{
				maxEyeCount = rEyeCount[i];
				most_possible_eye = i;
				minEyeDist = ABS(rEyeResult[i].x + rEyeResult[i].width/2 - faceROI->x - faceROI->width*3/4) + ABS(rEyeResult[i].y + rEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
			}
		} while (i--);

         if (maxEyeCount == min_eye_nbr)
		{
			rEyeROI->width = 0;
		}
		else
		{
			i = most_possible_eye;

			// Face Position Determination //
			*rEyeROI = rEyeResult[i];
		}
	}

	if(0 >= lEyeN)
	{
		lEyeROI->width = 0;
	}
	else
	{
		int minEyeDist = INT_MAX;
		int maxEyeCount = min_eye_nbr;
		int most_possible_eye = 0;

		int i = lEyeN - 1;
		do
		{
		    //DBG_PRINT("lEyeCount[%d] = %d\r\n",i,lEyeCount[i]);
			if (lEyeCount[i] > maxEyeCount)
			{
				maxEyeCount = lEyeCount[i];
				most_possible_eye = i;
				minEyeDist = ABS(lEyeResult[i].x + lEyeResult[i].width/2 - faceROI->x - faceROI->width/4) + ABS(lEyeResult[i].y + lEyeResult[i].height/2 - faceROI->y - faceROI->height/4);
			}
		} while (i--);
        if (maxEyeCount == min_eye_nbr)
		{
			lEyeROI->width = 0;
		}
		else
		{
			i = most_possible_eye;

			// Face Position Determination //
			*lEyeROI = lEyeResult[i];
		}
	}

#if 1
		//new ROI 20150506
		//////////////////////////////////////////////////////////
		if((lEyeROI->width != 0) && (rEyeROI->width != 0))
		{
			int centerEyeX,centerEyeY;
			int distance_between_eye;

			centerEyeX=((rEyeROI->x + (rEyeROI->width>>1) + lEyeROI->x + (lEyeROI->width>>1))>>1);
			centerEyeY=((rEyeROI->y + (rEyeROI->height>>1) + lEyeROI->y + (lEyeROI->height>>1))>>1);
			distance_between_eye=(rEyeROI->x + (rEyeROI->width>>1) - lEyeROI->x - (lEyeROI->width>>1));
			lowerbound = (int)(centerEyeY+(float)(distance_between_eye)*1.85);

			if(lowerbound>gray->height - 1)
			{
				lowerbound=gray->height - 1;
			}

			fd_result.rect[0].height = lowerbound - fd_result.rect[0].y;
#if 1
			//revise the face area
			fd_result.rect[0].y = centerEyeY - distance_between_eye*0.5;
			if(fd_result.rect[0].y<0)
			{
				fd_result.rect[0].y=0;
			}
			if(fd_result.rect[0].y+fd_result.rect[0].height>=240)
			{
				fd_result.rect[0].height=240-fd_result.rect[0].y;
			}
#endif
		}
		///////////////////////////////////////////////////////////
#endif

	if(idWorkMem->id_mode)
	{
	#if DEMO_DEBUG_EN == 1
		DBG_PRINT("\r\n****************************ROI******************************\r\n");
		DBG_PRINT("best_face.count = %d\r\n", faceCount[best_face]);
		DBG_PRINT("best_face.x = %d\r\n", faceResult[best_face].x);
		DBG_PRINT("best_face.y = %d\r\n", faceResult[best_face].y);
		DBG_PRINT("best_face.width = %d\r\n", faceResult[best_face].width);
		DBG_PRINT("best_face.height = %d\r\n\r\n", faceResult[best_face].height);
		DBG_PRINT("faceROI.x = %d\r\n", faceROI->x);
		DBG_PRINT("faceROI.y = %d\r\n", faceROI->y);
		DBG_PRINT("faceROI.width = %d\r\n", faceROI->width);
		DBG_PRINT("faceROI.height = %d\r\n", faceROI->height);
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
	}

	if(((lEyeROI->y + lEyeROI->height/2) < faceROI->y) || ((rEyeROI->y + rEyeROI->height/2) < faceROI->y))
		return 1;
	else
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
        DBG_PRINT("compare_out_buffer = 0x%x\r\n",compare_out_buffer);
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

static INT32S KOT_scalerStart(gpImage *src, gpImage *dst)
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
#if 0
	ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);
#else
	ret = drv_l2_FD_scaler_clip(0, 1, src, dst, &clip);
#endif
	drv_fd_unlock();

	return ret;
}

static INT32S KOT_scalerClipStart(gpImage *src, gpImage *dst, gpRect clip)
{
	INT32S ret;
	gpImage temp;

	drv_fd_lock();
	scaler_int_w_step = dst->width;
	ret = drv_l2_scaler_clip(0, 1, src, dst, &clip);
	drv_fd_unlock();

	return ret;
}

static INT32S KOT_scalerEnd(void)
{
	return 0;
}
#endif

#if FACE_TRACKING_EN == 1
////////////////////////////////////////////////////////////////
// tracking
////////////////////////////////////////////////////////////////
static INT32S obj_track_init(ObjDetect_t *odWorkMem)
{
	INT32U mem_size;
	gpImage *img = &odWorkMem->image;

    drv_track_lock();
	mem_size = track_get_memory_size();
	odWorkMem->obj_track_WorkMem = (void *)gp_malloc_align(mem_size, 16);
	if(odWorkMem->obj_track_WorkMem == 0) {
		DBG_PRINT("Fail to allocate obj_detect_WorkMem\r\n");
		return -1;
	}
	//DBG_PRINT("obj_track_WorkMem = 0x%x\r\n",odWorkMem->obj_track_WorkMem);
	gp_memset((INT8S *)odWorkMem->obj_track_WorkMem, 0x00, mem_size);

	track_init(odWorkMem->obj_track_WorkMem, img->width, img->height);
	drv_track_unlock();

	return 0;
}
#endif

#if DISPLAY_USE_PSCALER_EN == 1
// pscaler free buffer queue
static INT32S ppu_pscaler_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(ppu_pscaler_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(ppu_pscaler_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}

static INT32S ppu_pscaler_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(ppu_pscaler_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(ppu_pscaler_buffer_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}

static INT32S fd_ppu_draw_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
	INT32S retStatus;

	if(frame_buffer == 0) {
		return -1;
	}

    drv_pscaler_lock();
    drv_l1_pscaler_init(DISP_PSCALE_USE);
    drv_l1_pscaler_input_source_set(DISP_PSCALE_USE,PIPELINE_SCALER_INPUT_SOURCE_PPUFB);
    drv_l1_pscaler_input_pixels_set(DISP_PSCALE_USE,in_w,in_h);
    drv_l1_pscaler_output_pixels_set(DISP_PSCALE_USE,((in_w*65536)/out_w),out_w,((in_h*65536)/out_h),out_h);
    drv_l1_pscaler_output_fifo_line_set(DISP_PSCALE_USE,out_w,0);
    drv_l1_pscaler_interrupt_set(DISP_PSCALE_USE,PIPELINE_SCALER_INT_ENABLE_422GP420_FRAME_END);
    drv_l1_pscaler_input_format_set(DISP_PSCALE_USE,PIPELINE_SCALER_INPUT_FORMAT_YUYV);
    drv_l1_pscaler_output_A_buffer_set(DISP_PSCALE_USE,frame_buffer);
    drv_l1_pscaler_output_format_set(DISP_PSCALE_USE,PIPELINE_SCALER_OUTPUT_FORMAT_YUYV);
    drv_l1_pscaler_start(DISP_PSCALE_USE);
    gplib_ppu_go_and_wait_done(ppu_register_set);
    do
    {
        retStatus = drv_l1_pscaler_status_get(DISP_PSCALE_USE);

        if(retStatus & PIPELINE_SCALER_STATUS_FRAME_DONE)
            break;
        osDelay(1);
    }
    while(1);
    drv_l1_pscaler_stop(DISP_PSCALE_USE);
    drv_pscaler_unlock();
    disp_frame_buffer_add((INT32U *)frame_buffer,1);

    return 0;
}
#endif

static int face_Detect_With_pfmt(const gpImage* gray, gpRect* userROI, int mode, gpRect* kot_range, int fun_en)
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
	INT32S pos_temp;
	// xstep
	int xstep_face = 2;
	// ystep
	int ystep_face = 2;
	// face nbr
	int min_face_nbr_h = 6;
	int min_face_nbr_l = 5;
	// min face window
	int min_face_wnd = 40;//50;
	// sacler max window
	int max_wnd = MIN(gray->width, gray->height);
	// memory size range
	int HEIGHT = gray->height;
	int WIDTH = gray->width;

	INT32S nCnt, *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	INT32S search_cnt, max_result_t;
	gpRect *p_range, kot_roi;
	gpRect *p_result;
    gpRect *p_obj_rect = userROI;

#if FACE_TRACKING_EN == 1
    Rect.x = 0;
	Rect.y = 0;
	Rect.width = gray->width;
	Rect.height = gray->height;
    if(fun_en)
    {
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

        drv_track_lock();
        //get tracking info
        p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
        if(kot_range)
        {
            p_range = (gpRect *)&kot_roi;
            // x
            pos_temp = (kot_range->x - (kot_range->width>>1));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->x = pos_temp;
            // y
            pos_temp = (kot_range->y - (kot_range->height>>1));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->y = pos_temp;
            // width
            pos_temp = (kot_range->width<<1);
            if(pos_temp > gray->width)
                pos_temp = gray->width;
            p_range->width = pos_temp;
            // height
            pos_temp = (kot_range->height<<1);
            if(pos_temp > gray->height)
                pos_temp = gray->height;
            p_range->height = pos_temp;
        #if DEMO_DEBUG_EN == 1
            DBG_PRINT("kot_range->x = %d \r\n",kot_range->x);
            DBG_PRINT("kot_range->y = %d \r\n",kot_range->y);
            DBG_PRINT("kot_range->width = %d \r\n",kot_range->width);
            DBG_PRINT("kot_range->height = %d \r\n",kot_range->height);
            DBG_PRINT("p_range->x = %d \r\n",p_range->x);
            DBG_PRINT("p_range->y = %d \r\n",p_range->y);
            DBG_PRINT("p_range->width = %d \r\n",p_range->width);
            DBG_PRINT("p_range->height = %d \r\n",p_range->height);
        #endif
        }
        else
            p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);

        search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);
        min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
        i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
        p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
        p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);
        drv_track_unlock();

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
    }
    else
    {
        search_cnt = 1;
        maxFaceW = 1;
        p_obj_rect->x = kot_range->x;
        p_obj_rect->y = kot_range->y;
        p_obj_rect->width = kot_range->width;
        p_obj_rect->height = kot_range->height;
    }

    //DBG_PRINT("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
    // set tracking
    drv_track_lock();
    if(search_cnt > 0) {
        track_reset_face_location(ObjWorkMem_ptr->obj_track_WorkMem);
        p_obj_rect = (gpRect *)userROI;
        for (i = 0; i <search_cnt; i++) {
            track_set_face_location(ObjWorkMem_ptr->obj_track_WorkMem, &p_obj_rect[i], i);
        }
    }

    // tracking
    track_run(ObjWorkMem_ptr->obj_track_WorkMem, search_cnt);
    drv_track_unlock();

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

    if(fun_en)
    {
        /* Face Position Determination */
        offset_x = faceResult[best_face].width * 0.16;
        offset_y = faceResult[best_face].height/7;

        faceROI->x = faceResult[best_face].x + offset_x;
        faceROI->y = faceResult[best_face].y + offset_y;
        faceROI->width = (short)(faceResult[best_face].width - (offset_x<<1));
        faceROI->height = (short)(faceResult[best_face].height - offset_y*1.5);
    }
    else
    {
        faceROI->x = kot_range->x;
        faceROI->y = kot_range->y;
        faceROI->width = kot_range->width;
        faceROI->height = kot_range->height;
    }
    #if FD_DEBUG_EN == 1
        DBG_PRINT("faceROI.x = %d\r\n", faceROI->x);
        DBG_PRINT("faceROI.y = %d\r\n", faceROI->y);
        DBG_PRINT("faceROI.width = %d\r\n", faceROI->width);
        DBG_PRINT("faceROI.height = %d\r\n", faceROI->height);
    #endif
#else
    fd_state = 0;
#endif

    return fd_state;
}

static int face_Detect_With_Kot(const gpImage* gray, gpRect* userROI, int mode, gpRect* kot_range, int fun_en)
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
	INT32S pos_temp;
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

	INT32S nCnt, *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	INT32S search_cnt, max_result_t;
	gpRect *p_range, kot_roi;
	gpRect *p_result;
    gpRect *p_obj_rect = userROI;

#if FACE_TRACKING_EN == 1
    Rect.x = 0;
	Rect.y = 0;
	Rect.width = gray->width;
	Rect.height = gray->height;
    if(fun_en)
    {
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

        drv_track_lock();
        //get tracking info
        p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
        if(kot_range)
        {
            p_range = (gpRect *)&kot_roi;
            // x
            pos_temp = (kot_range->x - (kot_range->width/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->x = pos_temp;
            // y
            pos_temp = (kot_range->y - (kot_range->height/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->y = pos_temp;
            // width
            pos_temp = (kot_range->width*2);
            if(pos_temp > gray->width)
                pos_temp = gray->width;
            p_range->width = pos_temp;
            // height
            pos_temp = (kot_range->height*2);
            if(pos_temp > gray->height)
                pos_temp = gray->height;
            p_range->height = pos_temp;
        #if DEMO_DEBUG_EN == 1
            DBG_PRINT("kot_range->x = %d \r\n",kot_range->x);
            DBG_PRINT("kot_range->y = %d \r\n",kot_range->y);
            DBG_PRINT("kot_range->width = %d \r\n",kot_range->width);
            DBG_PRINT("kot_range->height = %d \r\n",kot_range->height);
            DBG_PRINT("p_range->x = %d \r\n",p_range->x);
            DBG_PRINT("p_range->y = %d \r\n",p_range->y);
            DBG_PRINT("p_range->width = %d \r\n",p_range->width);
            DBG_PRINT("p_range->height = %d \r\n",p_range->height);
        #endif
        }
        else
            p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);

        search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);
        min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
        i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
        p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
        p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);
        drv_track_unlock();

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
    }
    else
    {
        search_cnt = 1;
        maxFaceW = 1;
        p_obj_rect->x = kot_range->x;
        p_obj_rect->y = kot_range->y;
        p_obj_rect->width = kot_range->width;
        p_obj_rect->height = kot_range->height;
    }

    //DBG_PRINT("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
    // set tracking
    drv_track_lock();
    if(search_cnt > 0) {
        track_reset_face_location(ObjWorkMem_ptr->obj_track_WorkMem);
        p_obj_rect = (gpRect *)userROI;
        for (i = 0; i <search_cnt; i++) {
            track_set_face_location(ObjWorkMem_ptr->obj_track_WorkMem, &p_obj_rect[i], i);
        }
    }

    // tracking
    track_run(ObjWorkMem_ptr->obj_track_WorkMem, search_cnt);
    drv_track_unlock();

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

    if(fun_en)
    {
        /* Face Position Determination */
        offset_x = faceResult[best_face].width * 0.16;
        offset_y = faceResult[best_face].height/7;

        faceROI->x = faceResult[best_face].x + offset_x;
        faceROI->y = faceResult[best_face].y + offset_y;
        faceROI->width = (short)(faceResult[best_face].width - (offset_x<<1));
        faceROI->height = (short)(faceResult[best_face].height - offset_y*1.5);
    }
    else
    {
        faceROI->x = kot_range->x;
        faceROI->y = kot_range->y;
        faceROI->width = kot_range->width;
        faceROI->height = kot_range->height;
    }
    #if FD_DEBUG_EN == 1
        DBG_PRINT("faceROI.x = %d\r\n", faceROI->x);
        DBG_PRINT("faceROI.y = %d\r\n", faceROI->y);
        DBG_PRINT("faceROI.width = %d\r\n", faceROI->width);
        DBG_PRINT("faceROI.height = %d\r\n", faceROI->height);
    #endif
#else
    fd_state = 0;
#endif

    return fd_state;
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
	int min_face_nbr_l = 8;
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

	drv_track_lock();
	//get tracking info
	p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);
	search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);
	min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
	i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);
    drv_track_unlock();

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

		//min_face_wnd = *p_min_width++;
		min_face_wnd = 50;
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
    drv_track_lock();
    if(search_cnt > 0) {
        track_reset_face_location(ObjWorkMem_ptr->obj_track_WorkMem);
        p_obj_rect = (gpRect *)userROI;
        for (i = 0; i <search_cnt; i++) {
            track_set_face_location(ObjWorkMem_ptr->obj_track_WorkMem, &p_obj_rect[i], i);
        }
    }

    // tracking
    track_run(ObjWorkMem_ptr->obj_track_WorkMem, search_cnt);
    drv_track_unlock();

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
	//offset_x = faceResult[best_face].width * 0.16;
	//offset_y = faceResult[best_face].height/7;

	faceROI->x = faceResult[best_face].x;// + offset_x;
	faceROI->y = faceResult[best_face].y;// + offset_y;
	faceROI->width = (short)(faceResult[best_face].width);// - (offset_x<<1));
	faceROI->height = (short)(faceResult[best_face].height);// - offset_y*1.5);

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
			if(face_recognize_en)
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
		if(state && face_recognize_en)
		{
			set_sprite_init(C_FACE_IDENTIFY_END ,(INT32U)&Sprite006_SP);
			set_sprite_display_init(C_FACE_IDENTIFY_END,96,56,(INT32U)_Sprite006_IMG0000_CellIdx);
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
            set_sprite_display_init(C_FACE_TRAIN_SUBJECT,180,16,(INT32U)0);
            temp = (fidInfo.best_subject_index + 1);
            if(temp >= (fidInfo.MAX_SUBJECT_NUM + 1))
            {
                set_sprite_display_init(C_FACE_IDENTIFY_END,96,56,(INT32U)0);
                set_sprite_display_init(C_FACE_IDENTIFY_RESULT,180,16, 0);
            }
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

static gpRect GPRect_utils(int _x, int _y, int _width, int _height)
{
	gpRect rect;

	rect.x = _x;
	rect.y = _y;
	rect.width = _width;
	rect.height = _height;

	return rect;
}

static int faceRoiDetect_with_kot(const gpImage* gray, gpRect* userROI, int mode, gpRect* kot_ROI, int kot_en)
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
	int ret,faceN, rEyeN, lEyeN, t, int_scale_face, int_scale_eye, detectH, fd_state, temp;
	// xstep
	int xstep_face = 2;
	int xstep_eye = 2;
	// ystep
	int ystep_face = 2;
	int ystep_eye = 2;
	// face nbr
	int min_face_nbr_h = 8;
	int min_face_nbr_l = 5;
	// eye nbr
	int min_eye_nbr = 1;
	// min face window
	int min_face_wnd = 50;//50;
	// sacler max window
	int max_wnd = MIN(gray->width, gray->height);
	// memory size range
	int HEIGHT = gray->height;
	int WIDTH = gray->width;
	INT32S pos_temp;

#if FACE_TRACKING_EN == 1
	INT32S nCnt, *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	INT32S search_cnt, max_result_t;
	gpRect *p_range,kot_range;
	gpRect *p_result;
    gpRect *p_obj_rect = userROI;

    Rect.x = 0;
	Rect.y = 0;
	Rect.width = gray->width;
	Rect.height = gray->height;

    if(kot_en)
    {
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
        drv_track_lock();
        p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
        if(kot_ROI)
        {
            p_range = (gpRect *)&kot_range;
            // x
            pos_temp = (kot_ROI->x - (kot_ROI->width/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->x = pos_temp;
            // y
            pos_temp = (kot_ROI->y - (kot_ROI->height/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->y = pos_temp;
            // width
            pos_temp = (kot_ROI->width * 2);
            if(pos_temp > gray->width)
                pos_temp = gray->width;
            p_range->width = pos_temp;
            // height
            pos_temp = (kot_ROI->height * 2);
            if(pos_temp > gray->height)
                pos_temp = gray->height;
            p_range->height = pos_temp;
        #if DEMO_DEBUG_EN
            DBG_PRINT("kot_ROI->x = %d \r\n",kot_ROI->x);
            DBG_PRINT("kot_ROI->y = %d \r\n",kot_ROI->y);
            DBG_PRINT("kot_ROI->width = %d \r\n",kot_ROI->width);
            DBG_PRINT("kot_ROI->height = %d \r\n",kot_ROI->height);
            DBG_PRINT("p_range->x = %d \r\n",p_range->x);
            DBG_PRINT("p_range->y = %d \r\n",p_range->y);
            DBG_PRINT("p_range->width = %d \r\n",p_range->width);
            DBG_PRINT("p_range->height = %d \r\n",p_range->height);
        #endif
        }
        else
            p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);

        search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);
        min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
        i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
        p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
        p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);
        drv_track_unlock();

        //set face detect object type
        clfface.obj_type = OBJ_FACE;
        //fd_type = OBJ_FACE;
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
    }
    else
    {
        search_cnt = 1;
        maxFaceW = 1;
        p_obj_rect->x = kot_ROI->x;
        p_obj_rect->y = kot_ROI->y;
        p_obj_rect->width = kot_ROI->width;
        p_obj_rect->height = kot_ROI->height;
    }

    //DBG_PRINT("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
    // set tracking
    if(search_cnt > 0) {
        track_reset_face_location(ObjWorkMem_ptr->obj_track_WorkMem);
        //p_obj_rect = (gpRect *)userROI;
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
	//fd_type = OBJ_FACE;
    FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);

	FaceDetect_SetX(WorkMem, xstep_face, 0);
	faceN = FaceDetect(WorkMem, gray, &Rect, MAX_RESULT, faceResult, faceCount);

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

    drv_fd_result_lock();
    temp = face_recognize_en;
    drv_fd_result_unlock();
    if(temp)
    {
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
    }

    return fd_state;
}

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
	int ret,faceN, rEyeN, lEyeN, t, int_scale_face, int_scale_eye, detectH, fd_state, temp;
	// xstep
	int xstep_face = 2;
	int xstep_eye = 2;
	// ystep
	int ystep_face = 2;
	int ystep_eye = 2;
	// face nbr
	int min_face_nbr_h = 8;
	int min_face_nbr_l = 6;
	// eye nbr
	int min_eye_nbr = 1;
	// min face window
	int min_face_wnd = 32;//50;
	// sacler max window
	int max_wnd = 100;//MIN(gray->width, gray->height);
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
	drv_track_lock();
	p_min_width = (int *)track_get_search_min_width(ObjWorkMem_ptr->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(ObjWorkMem_ptr->obj_track_WorkMem);
	search_cnt = (INT32S)track_get_search_cnt(ObjWorkMem_ptr->obj_track_WorkMem);
	min_face_wnd = (int)track_get_min_wnd(ObjWorkMem_ptr->obj_track_WorkMem);
	i = (int)track_get_max_range(ObjWorkMem_ptr->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(ObjWorkMem_ptr->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(ObjWorkMem_ptr->obj_track_WorkMem);
    drv_track_unlock();

	//set face detect object type
    clfface.obj_type = OBJ_FACE;
	//fd_type = OBJ_FACE;
	FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);
	p_result = faceResult;
	max_result_t = FD_MAX_RESULT;
	p_count = faceCount;
	nCnt = 0;
	i = search_cnt;
	do {
		INT32S cnt;

    if(face_recognize_en)
    {
		min_face_wnd = *p_min_width++;
    }
    else
    {
		min_face_wnd = 32;//*p_min_width++;
	}


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
        if (faceCount[i] >= min_face_nbr_l && (faceResult[i].width < max_wnd) )
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
	//fd_type = OBJ_FACE;
    FaceDetect_set_detect_obj(WorkMem, &clfface);

    FaceDetect_set_ScalerFn(WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	ret = FaceDetect_SetScale(WorkMem, int_scale_face, min_face_wnd, max_wnd, mode);

	FaceDetect_SetX(WorkMem, xstep_face, 0);
	faceN = FaceDetect(WorkMem, gray, &Rect, MAX_RESULT, faceResult, faceCount);

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

    drv_fd_result_lock();
    temp = face_recognize_en;
    drv_fd_result_unlock();
    if(temp)
    {
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
    }

    return fd_state;
}

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
#if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
	switch(idWorkMem->security_level)
	{
		case 1:
			temp = 99000;
			break;

		case 2:
			temp = 117000;
			break;

		case 4:
			temp = 136000;
			break;

		case 5:
			temp = 155000;
			break;

		default:
			temp = 120000;
			break;
	}
#else
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
#endif
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

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 0
	mem_size = FaceIdentify_MemCalc();
	idWorkMem->identify_WorkMem = (INT32U *)gp_malloc_align((mem_size+1024), 4);
	DBG_PRINT("identify_WorkMem:0x%X \r\n",idWorkMem->identify_WorkMem);
	if(idWorkMem->identify_WorkMem == 0) {
		DBG_PRINT("Fail to allocate idWorkMem->identify_WorkMem\r\n");
		RETURN(STATUS_FAIL);
	}
	gp_memset((INT8S *)idWorkMem->identify_WorkMem, 0, mem_size);

	mem_size = SZ_ONEDATA * (FD_TRAINING_IMAGE_NUMBER + 2);
	idWorkMem->ownerULBP = (INT32U *)gp_malloc_align(mem_size, 4);
    DBG_PRINT("ownerULBP:0x%X \r\n",idWorkMem->ownerULBP);
	if(idWorkMem->ownerULBP == 0) {
		DBG_PRINT("Fail to allocate idWorkMem->ownerULBP\r\n");
		RETURN(STATUS_FAIL);
	}
	gp_memset((INT8S *)idWorkMem->ownerULBP, 0, mem_size);
#endif

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
			drv_fd_result_lock();
			idWorkMem->id_mode = 2;
			fd_mode = idWorkMem->id_mode;
			drv_fd_result_unlock();
			idWorkMem->training_cnt = 0;
			idWorkMem->identify_cnt = 0;
			idWorkMem->training_state = 1;
			fd_result.training_ok = 1;
			if(idWorkMem->pFuncStore) {
				idWorkMem->pFuncStore((INT32U)idWorkMem->ownerULBP, SZ_ONEDATA*(FD_TRAINING_IMAGE_NUMBER + 1));
			}
		}
	} else if(idWorkMem->id_mode == 2) {
        // identify
        drv_fd_result_lock();
        temp = idWorkMem->security_value;
        drv_fd_result_unlock();
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
			drv_fd_result_lock();
			idWorkMem->id_mode = 0; //idle mode
			fd_mode = idWorkMem->id_mode;
			drv_fd_result_unlock();
			idWorkMem->training_cnt = 0;
			idWorkMem->identify_cnt = 0;
			idWorkMem->identify_state = 1;
			fd_result.identify_ok = 1;
		}
	} else {
		while(1);
	}
}
#endif

static void fd_ppu_init(void)
{
    #define PPU_TEXT_H_SIZE         512
    #define PPU_TEXT_V_SIZE         256
    INT32U i,frame_size,buffer_ptr;
#if DISPLAY_USE_PSCALER_EN == 1
    INT32U pscaler_buf;
    FB_LOCK_STRUCT fb_lock_set;
#endif

    // Initialize display device
    drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&device_h_size, (INT16U *)&device_v_size);
#if DISPLAY_USE_PSCALER_EN == 1
    if((device_h_size != PPU_TEXT_SIZE_HPIXEL) && (device_v_size != PPU_TEXT_SIZE_VPIXEL))
    {
        frame_size = device_h_size*device_v_size*2;
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
                ppu_pscaler_buffer_add((INT32U *)buffer_ptr,1);
                DBG_PRINT("PscalerBuffer:0x%X \r\n",buffer_ptr);
        }
    }
#endif
    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;

    //Initiate PPU hardware engine and PPU register set structure
    gplib_ppu_init(ppu_register_set);
#if DISPLAY_USE_PSCALER_EN == 1
    fb_lock_set.color1 = PPU_FMT_YUYV;
    fb_lock_set.h_size1 = PPU_TEXT_SIZE_HPIXEL;
    fb_lock_set.v_size1 = PPU_TEXT_SIZE_VPIXEL;
    fb_lock_set.color2 = PPU_FMT_YUYV;
    fb_lock_set.h_size2 = device_h_size;
    fb_lock_set.v_size2 = device_v_size;
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
    gplib_ppu_free_size_set(ppu_register_set, 0, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL);
    gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
    gplib_ppu_long_burst_set(ppu_register_set, 1);
    gplib_ppu_tft_long_burst_set(ppu_register_set, 1);

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
    prcess_mem_set->ppu_narray_workmem = text1_narray;
    text1_narray = (INT32U)((text1_narray + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    DBG_PRINT("text1_narray:0x%X \r\n",text1_narray);

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
    #if DISPLAY_USE_PSCALER_EN == 1
        INT32U pscaler_buf;
    #endif

    #if 1
        gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, x, y, (INT32U)frame_buffer);	// Calculate Number array
        #if FACE_RECOGNIZE_MEUN_EN == 1
            face_recognize_sprite_update();
        #endif
        // Start PPU and wait until PPU operation is done
        #if DISPLAY_USE_PSCALER_EN == 1
            pscaler_buf = ppu_pscaler_buffer_get(1);
            if(pscaler_buf)
                fd_ppu_draw_set_frame(0,pscaler_buf,PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,device_h_size,device_v_size);
        #else
            gplib_ppu_go_and_wait_done(ppu_register_set);
        #endif
    #else
        R_PPU_ENABLE = 0x300580;
        R_TFT_FBI_ADDR = frame_buffer;
    #endif
}

#if HAND_EN == 1
static int gesture_detect_proc_with_pfmst(ObjDetect_t *odWorkMem, gpRect *pfmst_rang, INT32U fun_en)
{
	#define OBJDETECT_MIN_NBR       3
    INT32S i, N, nCnt, hand_mode;
	INT32S *p_count,pos_temp,pos_diff;
	INT32S *p_min_width, *p_xstep, *p_scale;
	gpRect *p_range,gesture_range;
	gpRect *p_result;
	INT32S search_cnt, min_width, max_result_t;
	INT32S xstep, scale;
	INT32U best_obj = 0;
	INT32S obj_cnt = 0;
	INT32U maxFaceW = 0;
	INT32U maxFaceCount = OBJDETECT_MIN_NBR;
    int xstep_hand = 2;
	int ystep_hand = 2;
	gpRect *p_obj_rect = hand_result.rect;
	gpRect *p_best = &hand_result.best_face;

    if(fun_en)
    {
        N = FaceDetect_Config(0, 0, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

        if(odWorkMem->obj_detect_WorkMem == 0)
        {
            odWorkMem->obj_detect_WorkMem = (void *)gp_malloc_align(N,4);
            hand_workmem_size = N;
            DBG_PRINT("Hand_WorkMem:0x%X \r\n",odWorkMem->obj_detect_WorkMem);
        }
        else
            gp_memset((INT8S *)odWorkMem->obj_detect_WorkMem,0, N);

        FaceDetect_Config(odWorkMem->obj_detect_WorkMem, N, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

        //get tracking info
        drv_track_lock();
        p_min_width = (int *)track_get_search_min_width(odWorkMem->obj_track_WorkMem);
        if(pfmst_rang)
        {
            p_range = (gpRect *)&gesture_range;

            // x
            pos_temp = (pfmst_rang->x - (pfmst_rang->width/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->x = pos_temp;
            // y
            pos_temp = (pfmst_rang->y - (pfmst_rang->height/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->y = pos_temp;
            // width
            pos_temp = (p_range->x + pfmst_rang->width + (pfmst_rang->width));
            if(pos_temp > odWorkMem->image.width)
            {
                pos_temp = (odWorkMem->image.width - p_range->x);
                p_range->width = pos_temp;
            }
            else
            {
                p_range->width = pfmst_rang->width + (pfmst_rang->width);
            }

            // height
            pos_temp = (p_range->y + pfmst_rang->height + (pfmst_rang->height));
            if(pos_temp > odWorkMem->image.height)
            {
                pos_temp = (odWorkMem->image.height - p_range->y);
                p_range->height = pos_temp;
            }
            else
            {
               p_range->height = pfmst_rang->height + (pfmst_rang->height);
            }



           //pfmst_rang->y += (pfmst_rang->height>>2);

           //p_range =  pfmst_rang;

            //DBG_PRINT("pfmst_rang.x = %d\r\n", pfmst_rang->x);
            //DBG_PRINT("pfmst_rang.y = %d\r\n", pfmst_rang->y);
            //DBG_PRINT("pfmst_rang.width = %d\r\n", pfmst_rang->width);
            //DBG_PRINT("pfmst_rang.height = %d\r\n", pfmst_rang->height);



            //DBG_PRINT("FIST_rang.x = %d\r\n", p_range->x);
            //DBG_PRINT("FIST_rang.y = %d\r\n", p_range->y);
            //DBG_PRINT("FIST_rang.width = %d\r\n", p_range->width);
            //DBG_PRINT("FIST_rang.height = %d\r\n", p_range->height);


        }
        else
            p_range = (gpRect *)track_get_search_range(odWorkMem->obj_track_WorkMem);
        search_cnt = 1;
        min_width = track_get_min_wnd(odWorkMem->obj_track_WorkMem);
        i = track_get_max_range(odWorkMem->obj_track_WorkMem);
        p_xstep = (INT32S *)track_get_xstep(odWorkMem->obj_track_WorkMem);
        p_scale = (INT32S *)track_get_scale(odWorkMem->obj_track_WorkMem);
        drv_track_unlock();

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

             if( pfmst_rang->width > 30 && pfmst_rang->width < 240)
            {
                min_width = pfmst_rang->width; //*p_min_width++;
            }
            else
            {
                min_width = *p_min_width++;

                 if( min_width == 30 )
                    min_width = 40;
            }
            scale = *p_scale++;

    #if SCALER_LINEAR_EN == 1
            FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_LINEAR);
    #else
            FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_NONLINEAR);
    #endif

            xstep = *p_xstep++;
            FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep, 0);

            hand_mode = 0;

    #if 0//PFMST_EN == 1
            odWorkMem->clf.obj_type = OBJ_HAND;
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
    #else
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
    #endif
    __Find:
            max_result_t -= cnt;
            p_result += cnt;
            p_count += cnt;
            nCnt += cnt;
            p_range++;
            i--;
        } while(i != 0);

        // get result
        obj_cnt = 0;
        for (i=0; i<nCnt; i++) {
            if(odWorkMem->count[i] >= OBJDETECT_MIN_NBR) {
            #if 1
                    if ((odWorkMem->result[i].width > maxFaceW) && (odWorkMem->count[i] > maxFaceCount))
                    {
                        maxFaceW = odWorkMem->result[i].width;
                        maxFaceCount = odWorkMem->count[i];
                        best_obj = i;
                    }
                    else
                    {
                        if (odWorkMem->count[i] >= maxFaceCount)
                        {
                            maxFaceW = odWorkMem->result[i].width;
                            maxFaceCount = odWorkMem->count[i];
                            best_obj = i;
                        }
                    }
            #endif
                // set obj_result
                p_obj_rect->x = odWorkMem->result[i].x;
                p_obj_rect->y = odWorkMem->result[i].y;
                p_obj_rect->width = odWorkMem->result[i].width;
                p_obj_rect->height = odWorkMem->result[i].height;
                obj_cnt++;
                if(obj_cnt > 1)
                {
                    hand_result.rect[0].x = p_obj_rect->x;
                    hand_result.rect[0].y = p_obj_rect->y;
                    hand_result.rect[0].width = p_obj_rect->width;
                    hand_result.rect[0].height = p_obj_rect->height;
                }
                p_obj_rect++;
            }
        }
        hand_result.result_cnt = obj_cnt;
    }
    else
    {
        hand_mode = 1;
        hand_result.result_cnt = 1;
        hand_result.rect[0].x = pfmst_rang->x;
        hand_result.rect[0].y = pfmst_rang->y;
        hand_result.rect[0].width = pfmst_rang->width;
        hand_result.rect[0].height = pfmst_rang->height;
    }

	//DEBUG_MSG("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
	// set tracking
	drv_track_lock();
	if(hand_result.result_cnt > 0) {
		odWorkMem->no_face_cnt = 0;
		track_reset_face_location(odWorkMem->obj_track_WorkMem);
		for(i=0; i<hand_result.result_cnt; i++) {
			track_set_face_location(odWorkMem->obj_track_WorkMem, &hand_result.rect[i], i);
		}
	} else {
		odWorkMem->no_face_cnt++;
	}

	// tracking
	track_run(odWorkMem->obj_track_WorkMem, hand_result.result_cnt);
    drv_track_unlock();

    if(!fun_en)
        hand_mode = hand_result.result_cnt;

    if(fun_en && hand_result.result_cnt)
    {
        hand_result.rect[0].x = odWorkMem->result[best_obj].x;
        hand_result.rect[0].y = odWorkMem->result[best_obj].y;
        hand_result.rect[0].width = odWorkMem->result[best_obj].width;
        hand_result.rect[0].height = odWorkMem->result[best_obj].height;
    }

    return hand_mode;
}

static int gesture_detect_proc(ObjDetect_t *odWorkMem)
{
	#define OBJDETECT_MIN_NBR       3
    INT32S i, N, nCnt, hand_mode;
	INT32S *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	gpRect *p_range;
	gpRect *p_result;
	INT32S search_cnt, min_width, max_result_t;
	INT32S xstep, scale;
	INT32S obj_cnt = 0;
	INT32U maxFaceW = 0;
	INT32U maxFaceCount = OBJDETECT_MIN_NBR;
    int xstep_hand = 2;
	int ystep_hand = 2;
	INT32U best_obj;
	gpRect *p_obj_rect = hand_result.rect;
	gpRect *p_best = &hand_result.best_face;

    N = FaceDetect_Config(0, 0, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

    if(odWorkMem->obj_detect_WorkMem == 0)
    {
        odWorkMem->obj_detect_WorkMem = (void *)gp_malloc_align(N,4);
        hand_workmem_size = N;
        DBG_PRINT("Hand_WorkMem:0x%X \r\n",odWorkMem->obj_detect_WorkMem);
    }
    else
        gp_memset((INT8S *)odWorkMem->obj_detect_WorkMem,0, N);

    FaceDetect_Config(odWorkMem->obj_detect_WorkMem, N, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

	//get tracking info
	drv_track_lock();
	p_min_width = (int *)track_get_search_min_width(odWorkMem->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(odWorkMem->obj_track_WorkMem);
	search_cnt = 1;
	min_width = track_get_min_wnd(odWorkMem->obj_track_WorkMem);
	i = track_get_max_range(odWorkMem->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(odWorkMem->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(odWorkMem->obj_track_WorkMem);
    drv_track_unlock();

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

		//min_width = *p_min_width++;
		min_width = 50;
		scale = *p_scale++;

#if SCALER_LINEAR_EN == 1
        FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_LINEAR);
#else
		FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_NONLINEAR);
#endif

		xstep = *p_xstep++;
		FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep, 0);

        hand_mode = 0;

#if 0//PFMST_EN == 1
        odWorkMem->clf.obj_type = OBJ_HAND;
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
#else
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
#endif
__Find:
		max_result_t -= cnt;
		p_result += cnt;
		p_count += cnt;
		nCnt += cnt;
		p_range++;
		i--;
	} while(i != 0);

	// get result
	best_obj = 0;
	for (i=0; i<nCnt; i++) {
		if(odWorkMem->count[i] >= OBJDETECT_MIN_NBR) {
#if 1
            if ((odWorkMem->result[i].width > maxFaceW) && (odWorkMem->count[i] > maxFaceCount))
            {
                maxFaceW = odWorkMem->result[i].width;
                maxFaceCount = odWorkMem->count[i];
                best_obj = i;
            }
            else
            {
                if (odWorkMem->count[i] >= maxFaceCount)
                {
                    maxFaceW = odWorkMem->result[i].width;
                    maxFaceCount = odWorkMem->count[i];
                    best_obj = i;
                }
            }
#endif
			// set obj_result
			p_obj_rect->x = odWorkMem->result[i].x;
			p_obj_rect->y = odWorkMem->result[i].y;
			p_obj_rect->width = odWorkMem->result[i].width;
			p_obj_rect->height = odWorkMem->result[i].height;
			hand_result.rect[0].x = p_obj_rect->x;
			hand_result.rect[0].y = p_obj_rect->y;
			hand_result.rect[0].width = p_obj_rect->width;
			//DBG_PRINT("hand_result (W, H): (%d, %d) \r\n",hand_result.rect[0].width, hand_result.rect[0].height);
			hand_result.rect[0].height = p_obj_rect->height;
			p_obj_rect++;
			obj_cnt++;
		}
	}

	hand_result.result_cnt = obj_cnt;

	//if(obj_cnt != 0)
	//DBG_PRINT("obj_cnt = %d \r\n", obj_cnt);

	//DEBUG_MSG("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
	// set tracking
	drv_track_lock();
	if(hand_result.result_cnt > 0) {
		odWorkMem->no_face_cnt = 0;
		track_reset_face_location(odWorkMem->obj_track_WorkMem);
		for(i=0; i<hand_result.result_cnt; i++) {
			track_set_face_location(odWorkMem->obj_track_WorkMem, &hand_result.rect[i], i);
		}
	} else {
		odWorkMem->no_face_cnt++;
	}

	// tracking
	track_run(odWorkMem->obj_track_WorkMem, hand_result.result_cnt);
    drv_track_unlock();

    if(hand_result.result_cnt)
    {
        hand_result.rect[0].x = odWorkMem->result[best_obj].x;
        hand_result.rect[0].y = odWorkMem->result[best_obj].y;
        hand_result.rect[0].width = odWorkMem->result[best_obj].width;
        hand_result.rect[0].height = odWorkMem->result[best_obj].height;
    }

    return hand_mode;
}

static int gesture_detect_mode_proc(ObjDetect_t *odWorkMem, INT32U mode)
{
	#define OBJDETECT_MIN_NBR       3
    INT32S i, N, nCnt, hand_mode;
	INT32S *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	gpRect *p_range;
	gpRect *p_result;
	INT32S search_cnt, min_width, max_result_t;
	INT32S xstep, scale;
	INT32S obj_cnt = 0;
	INT32U maxFaceW = 0;
	INT32U maxFaceCount = OBJDETECT_MIN_NBR;
    int xstep_hand = 2;
	int ystep_hand = 2;
	INT32U best_obj;
	gpRect *p_obj_rect = hand_result.rect;
	gpRect *p_best = &hand_result.best_face;

    N = FaceDetect_Config(0, 0, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

    if(odWorkMem->obj_detect_WorkMem == 0)
    {
        odWorkMem->obj_detect_WorkMem = (void *)gp_malloc_align(N,4);
        hand_workmem_size = N;
        DBG_PRINT("Hand_WorkMem:0x%X \r\n",odWorkMem->obj_detect_WorkMem);
    }
    else
        gp_memset((INT8S *)odWorkMem->obj_detect_WorkMem,0, N);

    FaceDetect_Config(odWorkMem->obj_detect_WorkMem, N, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

	//get tracking info
	drv_track_lock();
	p_min_width = (int *)track_get_search_min_width(odWorkMem->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(odWorkMem->obj_track_WorkMem);
	search_cnt = 1;
	min_width = track_get_min_wnd(odWorkMem->obj_track_WorkMem);
	i = track_get_max_range(odWorkMem->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(odWorkMem->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(odWorkMem->obj_track_WorkMem);
    drv_track_unlock();

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

		//min_width = *p_min_width++;
		min_width = 50;
		scale = *p_scale++;

#if SCALER_LINEAR_EN == 1
        FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_LINEAR);
#else
		FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_NONLINEAR);
#endif

		xstep = *p_xstep++;
		FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep, 0);

        hand_mode = 0;

        if(mode)
        {
            DBG_PRINT("#1");
            odWorkMem->clf.obj_type = OBJ_HAND;
            // hand
            FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);
            cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, p_range, max_result_t, p_result, p_count);
            //DBG_PRINT("#3");
            if(p_count[0] >= OBJDETECT_MIN_NBR) {
                #if 1
                    hand_mode = OBJ_HAND;
                #else
                    hand_mode = odWorkMem->clf.obj_type;
                #endif
                goto __Find;
            }
        }
        else
        {
            DBG_PRINT("#2");
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
        }
__Find:
		max_result_t -= cnt;
		p_result += cnt;
		p_count += cnt;
		nCnt += cnt;
		p_range++;
		i--;
	} while(i != 0);

	// get result
	best_obj = 0;
	for (i=0; i<nCnt; i++) {
		if(odWorkMem->count[i] >= OBJDETECT_MIN_NBR) {
#if 1
            if ((odWorkMem->result[i].width > maxFaceW) && (odWorkMem->count[i] > maxFaceCount))
            {
                maxFaceW = odWorkMem->result[i].width;
                maxFaceCount = odWorkMem->count[i];
                best_obj = i;
            }
            else
            {
                if (odWorkMem->count[i] >= maxFaceCount)
                {
                    maxFaceW = odWorkMem->result[i].width;
                    maxFaceCount = odWorkMem->count[i];
                    best_obj = i;
                }
            }
#endif
			// set obj_result
			p_obj_rect->x = odWorkMem->result[i].x;
			p_obj_rect->y = odWorkMem->result[i].y;
			p_obj_rect->width = odWorkMem->result[i].width;
			p_obj_rect->height = odWorkMem->result[i].height;
			hand_result.rect[0].x = p_obj_rect->x;
			hand_result.rect[0].y = p_obj_rect->y;
			hand_result.rect[0].width = p_obj_rect->width;
			//DBG_PRINT("hand_result (W, H): (%d, %d) \r\n",hand_result.rect[0].width, hand_result.rect[0].height);
			hand_result.rect[0].height = p_obj_rect->height;
			p_obj_rect++;
			obj_cnt++;
		}
	}

	hand_result.result_cnt = obj_cnt;

	//if(obj_cnt != 0)
	//DBG_PRINT("obj_cnt = %d \r\n", obj_cnt);

	//DEBUG_MSG("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
	// set tracking
	drv_track_lock();
	if(hand_result.result_cnt > 0) {
		odWorkMem->no_face_cnt = 0;
		track_reset_face_location(odWorkMem->obj_track_WorkMem);
		for(i=0; i<hand_result.result_cnt; i++) {
			track_set_face_location(odWorkMem->obj_track_WorkMem, &hand_result.rect[i], i);
		}
	} else {
		odWorkMem->no_face_cnt++;
	}

	// tracking
	track_run(odWorkMem->obj_track_WorkMem, hand_result.result_cnt);
    drv_track_unlock();

    if(hand_result.result_cnt)
    {
        hand_result.rect[0].x = odWorkMem->result[best_obj].x;
        hand_result.rect[0].y = odWorkMem->result[best_obj].y;
        hand_result.rect[0].width = odWorkMem->result[best_obj].width;
        hand_result.rect[0].height = odWorkMem->result[best_obj].height;
    }

    return hand_mode;
}

static int fist_detect_proc_with_pfmst(ObjDetect_t *odWorkMem, gpRect *pfmst_rang, INT32U fun_en)
{
	#define OBJDETECT_MIN_NBR       3
    INT32S i, N, nCnt, hand_mode;
	INT32S *p_count,pos_temp,pos_diff;
	INT32S *p_min_width, *p_xstep, *p_scale;
	gpRect *p_range,gesture_range;
	gpRect *p_result;
	INT32S min_width, max_result_t;
	INT32S xstep, scale;
	INT32U best_obj = 0;
	INT32S obj_cnt = 0;
	INT32U maxFaceW = 0;
	INT32U maxFaceCount = OBJDETECT_MIN_NBR;
    int xstep_hand = 2;
	int ystep_hand = 2;
	gpRect *p_obj_rect = fist_result.rect;
	gpRect *p_best = &fist_result.best_face;

    if(fun_en)
    {
        N = FaceDetect_Config(0, 0, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

        if(odWorkMem->obj_detect_WorkMem == 0)
        {
            odWorkMem->obj_detect_WorkMem = (void *)gp_malloc_align(N,4);
            fist_workmem_size = N;
            DBG_PRINT("Fist_WorkMem:0x%X \r\n",odWorkMem->obj_detect_WorkMem);
        }
        else
            gp_memset((INT8S *)odWorkMem->obj_detect_WorkMem,0, N);

        FaceDetect_Config(odWorkMem->obj_detect_WorkMem, N, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

        //get tracking info
        drv_track_lock();
        p_min_width = (int *)track_get_search_min_width(odWorkMem->obj_track_WorkMem);
        if(pfmst_rang)
        {
            p_range = (gpRect *)&gesture_range;

            // x
            pos_temp = (pfmst_rang->x - (pfmst_rang->width/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->x = pos_temp;
            // y
            pos_temp = (pfmst_rang->y - (pfmst_rang->height/2));
            if(pos_temp < 0)
                pos_temp = 0;
            p_range->y = pos_temp;
            // width
            pos_temp = (p_range->x + pfmst_rang->width + (pfmst_rang->width));
            if(pos_temp > odWorkMem->image.width)
            {
                pos_temp = (odWorkMem->image.width - p_range->x);
                p_range->width = pos_temp;
            }
            else
            {
                p_range->width = pfmst_rang->width + (pfmst_rang->width);
            }

            // height
            pos_temp = (p_range->y + pfmst_rang->height + (pfmst_rang->height));
            if(pos_temp > odWorkMem->image.height)
            {
                pos_temp = (odWorkMem->image.height - p_range->y);
                p_range->height = pos_temp;
            }
            else
            {
               p_range->height = pfmst_rang->height + (pfmst_rang->height);
            }



           //pfmst_rang->y += (pfmst_rang->height>>2);

           //p_range =  pfmst_rang;

            //DBG_PRINT("pfmst_rang.x = %d\r\n", pfmst_rang->x);
            //DBG_PRINT("pfmst_rang.y = %d\r\n", pfmst_rang->y);
            //DBG_PRINT("pfmst_rang.width = %d\r\n", pfmst_rang->width);
            //DBG_PRINT("pfmst_rang.height = %d\r\n", pfmst_rang->height);



            //DBG_PRINT("FIST_rang.x = %d\r\n", p_range->x);
            //DBG_PRINT("FIST_rang.y = %d\r\n", p_range->y);
            //DBG_PRINT("FIST_rang.width = %d\r\n", p_range->width);
            //DBG_PRINT("FIST_rang.height = %d\r\n", p_range->height);


        }
        else
            p_range = (gpRect *)track_get_search_range(odWorkMem->obj_track_WorkMem);
        min_width = track_get_min_wnd(odWorkMem->obj_track_WorkMem);
        i = track_get_max_range(odWorkMem->obj_track_WorkMem);
        p_xstep = (INT32S *)track_get_xstep(odWorkMem->obj_track_WorkMem);
        p_scale = (INT32S *)track_get_scale(odWorkMem->obj_track_WorkMem);
        drv_track_unlock();

        //set face detect object type
        FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

        p_result = odWorkMem->result;
        max_result_t = OBJDETECT_MAX_RESULT;
        p_count = odWorkMem->count;
        nCnt = 0;
        //min_width = *p_min_width++;
        min_width = 40;
        scale = *p_scale++;

    #if SCALER_LINEAR_EN == 1
        FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_LINEAR);
    #else
        FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_NONLINEAR);
    #endif

        xstep = *p_xstep++;
        FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep, 0);
        hand_mode = 0;

        // fist
        odWorkMem->clf.obj_type = OBJ_FIST;
        FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);
        obj_cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, p_range, max_result_t, p_result, p_count);
#if 0
        if(!obj_cnt && fun_en)
        {
            DBG_PRINT("FIST_rang.x = %d\r\n", p_range->x);
            DBG_PRINT("FIST_rang.y = %d\r\n", p_range->y);
            DBG_PRINT("FIST_rang.width = %d\r\n", p_range->width);
            DBG_PRINT("FIST_rang.height = %d\r\n", p_range->height);
        }
#endif
        nCnt += obj_cnt;
        if(p_count[0] >= OBJDETECT_MIN_NBR)
        {
            hand_mode = OBJ_FIST;
        }

        // get result
        obj_cnt = 0;
        for (i=0; i<nCnt; i++) {
            if(odWorkMem->count[i] >= OBJDETECT_MIN_NBR) {
#if 1
                if ((odWorkMem->result[i].width > maxFaceW) && (odWorkMem->count[i] > maxFaceCount))
                {
                    maxFaceW = odWorkMem->result[i].width;
                    maxFaceCount = odWorkMem->count[i];
                    best_obj = i;
                }
                else
                {
                    if (odWorkMem->count[i] >= maxFaceCount)
                    {
                        maxFaceW = odWorkMem->result[i].width;
                        maxFaceCount = odWorkMem->count[i];
                        best_obj = i;
                    }
                }
#endif
                // set obj_result
                p_obj_rect->x = odWorkMem->result[i].x;
                p_obj_rect->y = odWorkMem->result[i].y;
                p_obj_rect->width = odWorkMem->result[i].width;
                p_obj_rect->height = odWorkMem->result[i].height;
                obj_cnt++;
                if(obj_cnt > 1)
                {
                    fist_result.rect[0].x = p_obj_rect->x;
                    fist_result.rect[0].y = p_obj_rect->y;
                    fist_result.rect[0].width = p_obj_rect->width;
                    fist_result.rect[0].height = p_obj_rect->height;
                }
                p_obj_rect++;
            }
        }
        fist_result.result_cnt = obj_cnt;
    }
    else
    {
        //DBG_PRINT("123");
        hand_mode = 1;//OBJ_FIST;
        fist_result.result_cnt = 1;
        fist_result.rect[0].x = pfmst_rang->x;
        fist_result.rect[0].y = pfmst_rang->y;
        fist_result.rect[0].width = pfmst_rang->width;
        fist_result.rect[0].height = pfmst_rang->height;
    }

	//DEBUG_MSG("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
	// set tracking
	drv_track_lock();
	if(fist_result.result_cnt > 0) {
		odWorkMem->no_face_cnt = 0;
		track_reset_face_location(odWorkMem->obj_track_WorkMem);
		for(i=0; i<fist_result.result_cnt; i++) {
			track_set_face_location(odWorkMem->obj_track_WorkMem, &fist_result.rect[i], i);
		}
	} else {
		odWorkMem->no_face_cnt++;
	}

	// tracking
	track_run(odWorkMem->obj_track_WorkMem, fist_result.result_cnt);
    drv_track_unlock();

    if(!fun_en)
        hand_mode = fist_result.result_cnt;

    if(fist_result.result_cnt && fun_en)
    {
        fist_result.rect[0].x = odWorkMem->result[best_obj].x;
        fist_result.rect[0].y = odWorkMem->result[best_obj].y;
        fist_result.rect[0].width = odWorkMem->result[best_obj].width;
        fist_result.rect[0].height = odWorkMem->result[best_obj].height;
        //DBG_PRINT("fist_result: %d \r\n",fist_result.rect[0].width);
    }

    return hand_mode;
}

static int fist_detect_proc(ObjDetect_t *odWorkMem)
{
	#define OBJDETECT_MIN_NBR       3
    INT32S i, N, nCnt, hand_mode;
	INT32S *p_count;
	INT32S *p_min_width, *p_xstep, *p_scale;
	gpRect *p_range;
	gpRect *p_result;
	INT32S min_width, max_result_t;
	INT32S xstep, scale;
	INT32U best_obj = 0;
	INT32S obj_cnt = 0;
	INT32U maxFaceW = 0;
	INT32U maxFaceCount = OBJDETECT_MIN_NBR;
    int xstep_hand = 2;
	int ystep_hand = 2;
	gpRect *p_obj_rect = fist_result.rect;
	gpRect *p_best = &fist_result.best_face;
    gpImage gpHE;

    N = FaceDetect_Config(0, 0, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

    if(odWorkMem->obj_detect_WorkMem == 0)
    {
        odWorkMem->obj_detect_WorkMem = (void *)gp_malloc_align(N,4);
        fist_workmem_size = N;
        DBG_PRINT("Fist_WorkMem:0x%X \r\n",odWorkMem->obj_detect_WorkMem);
    }
    else
        gp_memset((INT8S *)odWorkMem->obj_detect_WorkMem,0, N);

    FaceDetect_Config(odWorkMem->obj_detect_WorkMem, N, odWorkMem->image.width, odWorkMem->image.height, 1, OBJDETECT_MAX_RESULT, xstep_hand, ystep_hand);

	//get tracking info
	drv_track_lock();
	p_min_width = (int *)track_get_search_min_width(odWorkMem->obj_track_WorkMem);
	p_range = (gpRect *)track_get_search_range(odWorkMem->obj_track_WorkMem);
	min_width = track_get_min_wnd(odWorkMem->obj_track_WorkMem);
	i = track_get_max_range(odWorkMem->obj_track_WorkMem);
	p_xstep = (INT32S *)track_get_xstep(odWorkMem->obj_track_WorkMem);
	p_scale = (INT32S *)track_get_scale(odWorkMem->obj_track_WorkMem);
    drv_track_unlock();

	//set face detect object type
    FaceDetect_set_ScalerFn(odWorkMem->obj_detect_WorkMem, scalerStart_fd, scalerEnd, scalerClip_fd);

	p_result = odWorkMem->result;
	max_result_t = OBJDETECT_MAX_RESULT;
	p_count = odWorkMem->count;
	nCnt = 0;
    //min_width = *p_min_width++;
    min_width = 40;
    scale = *p_scale++;

#if SCALER_LINEAR_EN == 1
    FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_LINEAR);
#else
    FaceDetect_SetScale(odWorkMem->obj_detect_WorkMem, scale, min_width, odWorkMem->image.height,SCALE_NONLINEAR);
#endif

    xstep = *p_xstep++;
    FaceDetect_SetX(odWorkMem->obj_detect_WorkMem, xstep, 0);
    hand_mode = 0;

    // fist
    odWorkMem->clf.obj_type = OBJ_FIST;
    FaceDetect_set_detect_obj(odWorkMem->obj_detect_WorkMem, &odWorkMem->clf);
    obj_cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, &odWorkMem->image, p_range, max_result_t, p_result, p_count);

	if (0)//!obj_cnt && fist_he_pos.x != 0) // do HE
	{
		gpRect eqROI = GPRect_utils(140, 140, 83, 54);
		gpRect applyROI = GPRect_utils(57, 0, 75, 240);

        gpHE.width = odWorkMem->image.width;
        gpHE.height = odWorkMem->image.height;
        gpHE.widthStep = odWorkMem->image.widthStep;
        gpHE.ch = odWorkMem->image.ch;
        gpHE.format = odWorkMem->image.format;
        gpHE.ptr = (INT8U *)image_HE_ptr;
        p_result = odWorkMem->result;
        max_result_t = OBJDETECT_MAX_RESULT;
        p_count = odWorkMem->count;
		equalizeHist_new2(&odWorkMem->image, (gpImage*)&gpHE, &fist_he_pos, &fist_he_pos);
        #if 0
            p_range->x = 0;
            p_range->y = 0;
            p_range->width =  odWorkMem->image.height;
            p_range->height = odWorkMem->image.height;
		#endif
		obj_cnt = FaceDetect(odWorkMem->obj_detect_WorkMem, (gpImage*)&gpHE, p_range, max_result_t, p_result, p_count);
	}

    nCnt += obj_cnt;
    if(p_count[0] >= OBJDETECT_MIN_NBR)
         hand_mode = OBJ_FIST;

	// get result
	for (i=0; i<nCnt; i++) {
		if(odWorkMem->count[i] >= OBJDETECT_MIN_NBR) {
#if 1
            if ((odWorkMem->result[i].width > maxFaceW) && (odWorkMem->count[i] > maxFaceCount))
            {
                maxFaceW = odWorkMem->result[i].width;
                maxFaceCount = odWorkMem->count[i];
                best_obj = i;
            }
            else
            {
                if (odWorkMem->count[i] >= maxFaceCount)
                {
                    maxFaceW = odWorkMem->result[i].width;
                    maxFaceCount = odWorkMem->count[i];
                    best_obj = i;
                }
            }
#endif
			// set obj_result
			p_obj_rect->x = odWorkMem->result[i].x;
			p_obj_rect->y = odWorkMem->result[i].y;
			p_obj_rect->width = odWorkMem->result[i].width;
			p_obj_rect->height = odWorkMem->result[i].height;
			fist_result.rect[0].x = p_obj_rect->x;
			fist_result.rect[0].y = p_obj_rect->y;
			fist_result.rect[0].width = p_obj_rect->width;
			//DBG_PRINT("fist_result: %d \r\n",fist_result.rect[0].width);
			fist_result.rect[0].height = p_obj_rect->height;
			p_obj_rect++;
		}
	}

	fist_result.result_cnt = obj_cnt;

	//DEBUG_MSG("Face = %d, best_no = %d\r\n", obj_result.result_cnt, best_face);
	// set tracking
	drv_track_lock();
	if(fist_result.result_cnt > 0) {
		odWorkMem->no_face_cnt = 0;
		track_reset_face_location(odWorkMem->obj_track_WorkMem);
		for(i=0; i<fist_result.result_cnt; i++) {
			track_set_face_location(odWorkMem->obj_track_WorkMem, &fist_result.rect[i], i);
		}
	} else {
		odWorkMem->no_face_cnt++;
	}

	// tracking
	track_run(odWorkMem->obj_track_WorkMem, fist_result.result_cnt);
    drv_track_unlock();

    if(fist_result.result_cnt)
    {
        fist_result.rect[0].x = odWorkMem->result[best_obj].x;
        fist_result.rect[0].y = odWorkMem->result[best_obj].y;
        fist_result.rect[0].width = odWorkMem->result[best_obj].width;
        fist_result.rect[0].height = odWorkMem->result[best_obj].height;
        //DBG_PRINT("fist_result: %d \r\n",fist_result.rect[0].width);
    }

    return hand_mode;
}
#endif

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

#if CSI_FULL_IMAGE_FLOW_EN == 1
static INT32S pscaler_clip_free_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(pscaler_clip_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(pscaler_clip_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}

static INT32S pscaler_clip_free_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(pscaler_clip_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(pscaler_clip_buffer_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}
#endif

// csi free buffer queue
static INT32S free_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

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

// prcess selection state queue
static INT32S prcess_selection_state_post(INT32U state)
{
    INT32U event,temp;

    event = (INT32U)state;
    temp = osMessagePut(prcess_selection_queue, (uint32_t)&event, osWaitForever);

	return temp;
}
static INT32S prcess_selection_state_get(void)
{
    osEvent result;
	INT32S frame;

	frame = 0;
    result = osMessageGet(prcess_selection_queue, 1);
    frame = result.value.v;
    if(result.status != osEventMessage) {
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

#if CSI_FULL_IMAGE_FLOW_EN == 1
static INT32S pscaler_clip_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_x, INT16U in_y, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
	#define DISP_USE_SCALER_EN          1
	#define FIX_OUTPUT_SIZE_EN          1
	INT32S retStatus;
    gpRect clip_ptr;
	ObjFrame_t *pInput,Input;

	if(frame_buffer == 0 || in_buffer == 0) {
		return -1;
	}

#if DISP_USE_SCALER_EN == 1
    pInput = (ObjFrame_t *)&Input;
	if(pInput == 0) {
		return -1;
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
    pInput->ScaleOut.ch  = 2;
    pInput->ScaleOut.widthStep = out_w * 2;
	pInput->ScaleOut.format = IMG_FMT_UYVY;
	pInput->ScaleOut.ptr = (INT8U *)frame_buffer;

#if CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
    clip_ptr.x = 0;
    clip_ptr.y = 0;
    clip_ptr.width = in_w;
    clip_ptr.height = in_h;
#else
    clip_ptr.x = in_x;
    clip_ptr.y = in_y;
    clip_ptr.width = out_w;
    clip_ptr.height = out_h;
#endif

	// scale
    disp_scaler_clip_Start(&pInput->ScaleIn, &pInput->ScaleOut, &clip_ptr);
    drv_fd_unlock();
	scalerEnd();
#else
    drv_pscaler_lock();
    drv_l1_pscaler_init(DISP_PSCALE_USE);
    drv_l1_pscaler_input_source_set(DISP_PSCALE_USE, PIPELINE_SCALER_INPUT_SOURCE_DRAM);
#if CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
    drv_l1_pscaler_input_X_start_set(DISP_PSCALE_USE,0);
    drv_l1_pscaler_input_Y_start_set(DISP_PSCALE_USE,0);
    drv_l1_pscaler_input_pixels_set(DISP_PSCALE_USE, in_w, in_h);
    drv_l1_pscaler_output_pixels_set(DISP_PSCALE_USE,((in_w*65536)/out_w), out_w, ((in_h*65536)/out_h), out_h);
    drv_l1_pscaler_output_fifo_line_set(DISP_PSCALE_USE, out_w, 0);
#else
    drv_l1_pscaler_input_X_start_set(DISP_PSCALE_USE,in_x);
    drv_l1_pscaler_input_Y_start_set(DISP_PSCALE_USE,in_y);
#if FIX_OUTPUT_SIZE_EN == 1
    drv_l1_pscaler_input_pixels_set(DISP_PSCALE_USE, in_w, in_h);
    drv_l1_pscaler_output_pixels_set(DISP_PSCALE_USE,((out_w*65536)/out_w), out_w, ((out_h*65536)/out_h), out_h);
    drv_l1_pscaler_output_fifo_line_set(DISP_PSCALE_USE, out_w, 0);
#else
    drv_l1_pscaler_input_pixels_set(DISP_PSCALE_USE, in_w, in_h);
    drv_l1_pscaler_output_pixels_set(DISP_PSCALE_USE,((in_w*65536)/out_w), out_w, ((in_h*65536)/out_h), out_h);
    drv_l1_pscaler_output_fifo_line_set(DISP_PSCALE_USE, out_w, 0);
#endif
#endif
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
    drv_pscaler_unlock();
#endif

    return 0;
}
#endif

static INT32S fd_display_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
	INT32S retStatus;

	if(frame_buffer == 0 || in_buffer == 0) {
		return -1;
	}

    drv_pscaler_lock();
    drv_l1_pscaler_init(DISP_PSCALE_USE);
    drv_l1_pscaler_input_source_set(DISP_PSCALE_USE, PIPELINE_SCALER_INPUT_SOURCE_DRAM);
    drv_l1_pscaler_input_X_start_set(DISP_PSCALE_USE,0);
    drv_l1_pscaler_input_Y_start_set(DISP_PSCALE_USE,0);
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
    drv_pscaler_unlock();

    return 0;
}

static void mazePscalerSet(PSCALER_PARAM_STRUCT* pPScalerParam)
{
	INT32U widthFactor,heightFactor;
#if CSI_FULL_IMAGE_FLOW_EN == 0 || CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
    INT32U x_start,y_start;
    #define CENTER_WIDTH_720P        960

	drv_l1_pscaler_init(pPScalerParam->pScalerNum);

    if(pPScalerParam->inWidth == 1280 && pPScalerParam->inHeight == 720)
    {
        x_start = (pPScalerParam->inWidth - CENTER_WIDTH_720P)/2;
        y_start =0;

        widthFactor = ((CENTER_WIDTH_720P*65536)/pPScalerParam->outWidth);
        heightFactor = ((pPScalerParam->inHeight*65536)/pPScalerParam->outHeight);

        drv_l1_pscaler_input_X_start_set(pPScalerParam->pScalerNum,x_start);
        drv_l1_pscaler_input_Y_start_set(pPScalerParam->pScalerNum,y_start);
    }
    else
    {
        widthFactor = ((pPScalerParam->inWidth*65536)/pPScalerParam->outWidth);
        heightFactor = ((pPScalerParam->inHeight*65536)/pPScalerParam->outHeight);
    }
#else
	drv_l1_pscaler_init(pPScalerParam->pScalerNum);

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

}

static void PScaler_Callback_ISR_AutoZoom(INT32U PScaler_Event)
{
	INT32U prcessBuf,csiBuf;

	if(csi_pscaler_stop)
	{
        csi_pscaler_stop = 0;
        drv_l1_pscaler_stop(PScalerParam.pScalerNum);
        drv_l1_pscaler_clk_ctrl(PScalerParam.pScalerNum, 0);
	}
	else
	{
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
}

#if MOTION_DETECTION_EN == 1
static void CDSP_ISR_Callback(void)
{
    INT32U result = drv_l1_CdspGetResult();

    if(csi_md_stop)
    {
        drv_l1_CdspSetMDEn(0);
        drv_l2_CdspIsrRegister(C_ISR_EOF, 0);
        csi_md_stop = 0;
    }
    else
    {
        if(result > 0x125)
            DBG_PRINT("M");
    }
}
#endif

static void mazeTest_Preview_PScaler(void)
{
    CHAR *p;
	INT32U i,PrcessBuffer,temp;
	INT32U csiBufferSize,csiBuffer;
	drv_l2_sensor_ops_t *pSencor;
	drv_l2_sensor_info_t *pInfo;
	drv_l2_sensor_para_t *pPara;

    csiBuffer = DUMMY_BUFFER_ADDRESS;
#if CSI_FULL_IMAGE_FLOW_EN == 1
    // csi pscaler buffer
#if CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
    csiBufferSize = SENSOR_TAR_WIDTH*SENSOR_TAR_HEIGHT*2;
#else
    csiBufferSize = SENSOR_SRC_WIDTH*SENSOR_SRC_HEIGHT*2;
#endif
	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*DISP_QUEUE_MAX)+64), 64);
    if(PrcessBuffer == 0)
    {
        DBG_PRINT("CSIBuffer fail\r\n");
        while(1);
    }
	prcess_mem_set->csi_prcess_workmem = PrcessBuffer;
	PrcessBuffer = (INT32U)((PrcessBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<MSG_QUEUE_MAX; i++)
	{
		temp = (PrcessBuffer+i*csiBufferSize);
		free_frame_buffer_add((INT32U *)temp,1);
		DBG_PRINT("CSIBuffer:0x%X \r\n",temp);
	}

    // clip pscaler buffer
    csiBufferSize = PRCESS_SRC_WIDTH*PRCESS_SRC_HEIGHT*2;
	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*DISP_QUEUE_MAX)+64), 64);
    if(PrcessBuffer == 0)
    {
        DBG_PRINT("ClipBuffer fail\r\n");
        while(1);
    }
	prcess_mem_set->csi_pscaler_clip_workmem = PrcessBuffer;
	PrcessBuffer = (INT32U)((PrcessBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<DISP_QUEUE_MAX; i++)
	{
		temp = (PrcessBuffer+i*csiBufferSize);
		pscaler_clip_free_buffer_add((INT32U *)temp,1);
		DBG_PRINT("ClipBuffer:0x%X \r\n",temp);
	}
#else
    csiBufferSize = PRCESS_SRC_WIDTH*PRCESS_SRC_HEIGHT*2;
	PrcessBuffer = (INT32U) gp_malloc_align(((csiBufferSize*DISP_QUEUE_MAX)+64), 64);
    if(PrcessBuffer == 0)
    {
        DBG_PRINT("PrcessBuffer fail\r\n");
        while(1);
    }
	prcess_mem_set->csi_prcess_workmem = PrcessBuffer;
	PrcessBuffer = (INT32U)((PrcessBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<DISP_QUEUE_MAX; i++)
	{
		temp = (PrcessBuffer+i*csiBufferSize);
		free_frame_buffer_add((INT32U *)temp,1);
		DBG_PRINT("CSIBuffer:0x%X \r\n",temp);
	}
#endif

#if (BOARD_TYPE <= BOARD_GPM47XXA_EMU_V1_0)
	// Enable Sensor Clock
	R_SYSTEM_CTRL |= (1 << 11);
	// Change Sensor control pin to IOD6~9
	R_FUNPOS1 |= (1<<21)|(1<<24);
	R_FUNPOS1 |= (1<<7);
#endif

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

	// PScaler
	PScalerParam.pScalerNum = CSI_PSCALE_USE;
	PScalerParam.inBuffer = csiBuffer;
	PScalerParam.outBuffer_A = free_frame_buffer_get(1);
	PScalerParam.outBuffer_B = free_frame_buffer_get(1);

	PScalerParam.inWidth = SENSOR_SRC_WIDTH;
	PScalerParam.inHeight = SENSOR_SRC_HEIGHT;

#if CSI_FULL_IMAGE_FLOW_EN == 1
#if CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
	PScalerParam.outWidth = SENSOR_TAR_WIDTH;
	PScalerParam.outHeight = SENSOR_TAR_HEIGHT;
	PScalerParam.outLineCount = SENSOR_TAR_HEIGHT;
#else
	PScalerParam.outWidth = SENSOR_SRC_WIDTH;
	PScalerParam.outHeight = SENSOR_SRC_HEIGHT;
	PScalerParam.outLineCount = SENSOR_SRC_HEIGHT;
#endif
#else
	PScalerParam.outWidth = PRCESS_SRC_WIDTH;
	PScalerParam.outHeight = PRCESS_SRC_HEIGHT;
	PScalerParam.outLineCount = PRCESS_SRC_HEIGHT;
#endif

	PScalerParam.inFormat = PIPELINE_SCALER_INPUT_FORMAT_YUYV;
	PScalerParam.outFormat = PIPELINE_SCALER_OUTPUT_FORMAT_YUYV;

    pPara = drv_l2_sensor_get_para();
    if (csi_mode == CSI_INTERFACE)
    {
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CSI;
        // CSI
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CSI;
        // Open CSI data path
        drvl1_csi_input_pscaler_set(1);
    }
    else
    {
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CDSP;
        // CDSP
        PScalerParam.inSource = PIPELINE_SCALER_INPUT_SOURCE_CDSP;
        // Open CDSP data path
        drv_l1_CdspSetYuvPscalePath(1);
        #if MOTION_DETECTION_EN == 1
            pPara->md_callback = (void *)&CDSP_ISR_Callback;
            pPara->md_buf = md_work_iram_addr;
            pPara->md_mode = 1;
            pPara->md_threshold = SENSOR_MD_THR;
            drv_l1_CdspSetMD(1, SENSOR_MD_THR, PRCESS_SRC_WIDTH, md_work_iram_addr);
            drv_l2_CdspIsrRegister(C_ISR_EOF, CDSP_ISR_Callback);
        #endif
    }

	PScalerParam.intEnableFlag = PIPELINE_SCALER_INT_ENABLE_ALL;
    PScalerParam.callbackFunc = PScaler_Callback_ISR_AutoZoom;

	mazePscalerSet(&PScalerParam);

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

	drv_l1_pscaler_start(PScalerParam.pScalerNum);
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

static void csi_task_entry(void const *parm)
{
    #define COMMAND_CHECK_CNT               5
    #define CSI_SOFTWARE_CNT_EN             1
    #define API_COMMAND_DRAW                1
    INT32U i,csi_buf,PscalerBuffer,PscalerBufferSize,FDBuffer,ack_msg;
    osEvent result;
    drv_l2_sensor_ops_t *pSencor;
    gpRect FaceFistResult;
#if FD_EN == 1
    INT32S fd_event;
    INT32U fd_size, fd_frame;
    ObjResult_t fd_draw = {0};
#endif
#if KOT_EN == 1
    globalData_KOT gData_ptr_draw = {0};
#endif
#if PFMST_EN == 1
    gpRect trackingSet_hand_draw[10] = {0};
    TPFMSContext context_hand_draw = {0};
    gpRect trackingSet_fist_draw[10] = {0};
    TPFMSContext context_fist_draw = {0};
#endif
#if CSI_SOFTWARE_CNT_EN == 1
    INT32U t1, t2;
#endif
#if CSI_FULL_IMAGE_FLOW_EN == 1
    INT32U pscaler_clip_buffer;
#endif

    DBG_PRINT("csi_task_entry start \r\n");

    // csi init
#if WAFD_DEMO_EN == 0
    mazeTest_Preview_PScaler();
#endif
    // disp size
#if PPU_DRAW_EN == 0
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&device_h_size, (INT16U *)&device_v_size);
#endif
	PscalerBufferSize = (device_h_size * device_v_size * 2);
	PscalerBuffer = (INT32U) gp_malloc_align(((PscalerBufferSize*C_DEVICE_FRAME_NUM)+64), 64);
    if(PscalerBuffer == 0)
    {
        DBG_PRINT("PscalerBuffer fail\r\n");
        while(1);
    }
    prcess_mem_set->disp_prcess_workmem = PscalerBuffer;
	PscalerBuffer = (INT32U)((PscalerBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	for(i=0; i<C_DEVICE_FRAME_NUM; i++)
	{
		csi_buf = (PscalerBuffer+i*PscalerBufferSize);
		pscaler_frame_buffer_add((INT32U *)csi_buf,1);
		DBG_PRINT("PscalerBuffer:0x%X \r\n",csi_buf);
	}

#if FD_EN == 1
	// fd buffer
	fd_size = FD_SIZE_HPIXEL*FD_SIZE_VPIXEL*2;
    FDBuffer = (INT32U)gp_malloc_align(((fd_size*C_DEVICE_FRAME_NUM)+64),64);
    if(FDBuffer == 0)
    {
        DBG_PRINT("FDBuffer fail\r\n");
        while(1);
    }
    prcess_mem_set->fd_prcess_workmem = FDBuffer;
    FDBuffer = (INT32U)((FDBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    for(i=0;i<C_DEVICE_FRAME_NUM;i++)
    {
        fd_frame = (INT32U)(FDBuffer+(i*fd_size));
        fd_free_frame_buffer_add((INT32U *)fd_frame, 1);
        DBG_PRINT("FDBuffer%d = 0x%x\r\n",i,fd_frame);
    }
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
        //DBG_PRINT("C");

        switch(csi_buf)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_CSI_TASK_EXIT]\r\n");
                #if MOTION_DETECTION_EN == 1
                    if(csi_mode == CDSP_INTERFACE)
                    {
                        csi_md_stop = 1;
                        while(csi_md_stop)
                            osDelay(1);
                    }
                #endif
                csi_pscaler_stop = 1;
                while(csi_pscaler_stop)
                    osDelay(1);
                #if CSI_PSCALER_STOP_EN == 0
                    pSencor = drv_l2_sensor_get_ops(0);
                    pSencor->stream_stop();
                    pSencor->uninit();
                #endif
                ack_msg = ACK_OK;
                osMessagePut(csi_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(csi_id);
                break;

            default:

        #if CSI_SOFTWARE_CNT_EN == 1
                t1 = xTaskGetTickCount();
        #endif

        #if CSI_FULL_IMAGE_FLOW_EN == 1
                pscaler_clip_buffer = pscaler_clip_free_buffer_get(1);
            #if FD_COMMAND_FLOW_EN == 1
                if(pscaler_clip_buffer)
                {
                    if((device_h_size == FD_SIZE_HPIXEL) && (device_v_size == FD_SIZE_VPIXEL))
                    {
                        PscalerBuffer = pscaler_frame_buffer_get(1);
                        if(PscalerBuffer)
                        #if CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
                            fd_display_set_frame(csi_buf, PscalerBuffer, SENSOR_TAR_WIDTH, SENSOR_TAR_HEIGHT, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
                        #else
                            fd_display_set_frame(csi_buf, PscalerBuffer, SENSOR_SRC_WIDTH, SENSOR_SRC_HEIGHT, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
                        #endif
                        else
                            DBG_PRINT("+");
                    }
                    else
                    {
                        #if CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
                            pscaler_clip_set_frame(csi_buf, pscaler_clip_buffer, 0, 0, SENSOR_TAR_WIDTH, SENSOR_TAR_HEIGHT, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT);
                        #else
                            pscaler_clip_set_frame(csi_buf, pscaler_clip_buffer, (SENSOR_SRC_WIDTH - PRCESS_SRC_WIDTH)/2, (SENSOR_SRC_HEIGHT - PRCESS_SRC_HEIGHT)/2, SENSOR_SRC_WIDTH, SENSOR_SRC_HEIGHT, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT);
                        #endif

                        PscalerBuffer = pscaler_frame_buffer_get(1);
                        if(PscalerBuffer && pscaler_clip_buffer)
                            fd_display_set_frame(pscaler_clip_buffer, PscalerBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
                    }
                }
                else
                    DBG_PRINT("-");
            #else
                if(pscaler_clip_buffer)
                #if CSI_FIX_TAR_OUTPUT_FLOW_EN == 1
                    pscaler_clip_set_frame(csi_buf, pscaler_clip_buffer, 0, 0, SENSOR_TAR_WIDTH, SENSOR_TAR_HEIGHT, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT);
                #else
                    pscaler_clip_set_frame(csi_buf, pscaler_clip_buffer, (SENSOR_SRC_WIDTH - PRCESS_SRC_WIDTH)/2, (SENSOR_SRC_HEIGHT - PRCESS_SRC_HEIGHT)/2, SENSOR_SRC_WIDTH, SENSOR_SRC_HEIGHT, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT);
                #endif

                PscalerBuffer = pscaler_frame_buffer_get(1);
                if(PscalerBuffer && pscaler_clip_buffer)
                    fd_display_set_frame(pscaler_clip_buffer, PscalerBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
            #endif
        #else
                PscalerBuffer = pscaler_frame_buffer_get(1);
                if(PscalerBuffer)
                    fd_display_set_frame(csi_buf, PscalerBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
        #endif

        #if FD_EN == 1
                fd_event = fd_state_get();
                if(fd_event == FD_STATE_OK)
                {
                    FDBuffer = fd_free_frame_buffer_get();
                    if(FDBuffer)
                    {
            #if CSI_FULL_IMAGE_FLOW_EN == 1
                #if PFMST_EN == 1
                        if((device_h_size == FD_SIZE_HPIXEL) && (device_v_size == FD_SIZE_VPIXEL))
                            fd_set_frame(PscalerBuffer,FDBuffer,FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_YUYV);
                        else
                            fd_set_frame(pscaler_clip_buffer, FDBuffer,PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_YUYV);
                #else
                        if((device_h_size == FD_SIZE_HPIXEL) && (device_v_size == FD_SIZE_VPIXEL))
                            fd_set_frame(PscalerBuffer,FDBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_GRAY);
                        else
                            fd_set_frame(pscaler_clip_buffer, FDBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_GRAY);
                #endif
            #else
                #if PFMST_EN == 1
                        if((device_h_size == FD_SIZE_HPIXEL) && (device_v_size == FD_SIZE_VPIXEL))
                            fd_set_frame(PscalerBuffer, FDBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_YUYV);
                        else
                            fd_set_frame(csi_buf, FDBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_YUYV);
                #else
                        if((device_h_size == FD_SIZE_HPIXEL) && (device_v_size == FD_SIZE_VPIXEL))
                            fd_set_frame(PscalerBuffer, FDBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_GRAY);
                        else
                            fd_set_frame(csi_buf, FDBuffer, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT, FD_SIZE_HPIXEL,FD_SIZE_VPIXEL,BITMAP_GRAY);
                #endif
            #endif
                        fd_frame_buffer_post((INT32U *)FDBuffer);
                    }
                }
        #endif

        #if FD_COMMAND_FLOW_EN == 1
            if(module_mem_set)
                command_result_check();
        #endif

                if(PscalerBuffer)
                {
        #if FD_EN == 1
                    if(fd_event == FD_STATE_OK)
                    {
#if FD_COMMAND_FLOW_EN == 1
                        drv_fd_result_lock();
                        if(module_mem_set->mode == API_COMMAND_HAND_FIST_DET)
                        {
                            //DBG_PRINT("\r\nis_hand\r\n");
                            fd_draw.result_cnt = fd_draw_result.result_cnt;
                            fd_draw.is_hand = fd_draw_result.is_hand;
                            fd_draw.rect[HAND_NUM_SET].x = fd_draw_result.rect[HAND_NUM_SET].x;
                            fd_draw.rect[HAND_NUM_SET].y = fd_draw_result.rect[HAND_NUM_SET].y;
                            fd_draw.rect[HAND_NUM_SET].width = fd_draw_result.rect[HAND_NUM_SET].width;
                            fd_draw.rect[HAND_NUM_SET].height = fd_draw_result.rect[HAND_NUM_SET].height;
                        }
                        drv_fd_result_unlock();

                        if(command_mode != API_COMMAND_RESULT_DET)
                        {
                            fd_event = face_hand_fist_command_result_get((module_command_t *)module_mem_set, command_mode,(gpRect *)&FaceFistResult);
                        }
                        else
                        {
                            drv_fd_result_lock();
                            fd_event = module_mem_set->command_ok;
                            drv_fd_result_unlock();
                            if((fd_event == COMMAND_MODULE_STOP) || (fd_event == COMMAND_MODULE_START))
                            {
                                fd_event = face_hand_fist_command_result_get((module_command_t *)module_mem_set, command_mode,(gpRect *)&FaceFistResult);
                            }
                            else
                            {
                                drv_fd_result_lock();
                                fd_event = module_mem_set->command_check_enable;
                                drv_fd_result_unlock();
                                if(fd_event)
                                {
                                    drv_fd_result_lock();
                                    module_mem_set->command_check_cnt++;
                                    fd_event = module_mem_set->command_check_cnt;
                                    drv_fd_result_unlock();
                                    if((fd_event % COMMAND_CHECK_CNT) == 0)
                                        fd_event = face_hand_fist_command_result_get((module_command_t *)module_mem_set, command_mode,(gpRect *)&FaceFistResult);
                                    else
                                    {
                                        fd_event = COMMAND_STATE_MAX;
                                    }
                                }
                                else
                                {
                                    fd_event = COMMAND_STATE_MAX;
                                }
                            }
                        }

                        fd_draw.is_best_face = 0;
                        fd_draw.is_get_eye = 0;
                        fd_draw.result_cnt = 0;
                        #if 1
                            fd_draw.is_hand = 0;
                        #else
                            drv_fd_result_lock();
                            if(module_mem_set->mode != API_COMMAND_HAND_FIST_DET)
                                fd_draw.is_hand = 0;
                            drv_fd_result_unlock();
                        #endif
                        fd_draw.is_fist = 0;
                        switch(fd_event)
                        {
                            case COMMAND_FIND_HAND_WITH_FIST:
                                //DBG_PRINT("Result:COMMAND_FIND_HAND_WITH_FIST\r\n");
                                break;

                            case COMMAND_TURN_UP:
                                DBG_PRINT("\r\nResult:COMMAND_TURN_UP\r\n");
                                break;

                            case COMMAND_TURN_DOWN:
                                DBG_PRINT("\r\nResult:COMMAND_TURN_DOWN\r\n");
                                //checktimesCnt++;
                                //DBG_PRINT("\r\checktimesCnt = %d\r\n", checktimesCnt);
                                break;

                            case COMMAND_FUNCTION_UP:
                                DBG_PRINT("\r\nResult:COMMAND_FUNCTION_UP\r\n");
                                break;

                            case COMMAND_FUNCTION_DOWN:
                                DBG_PRINT("\r\nResult:COMMAND_FUNCTION_DOWN\r\n");
                                break;

                            case COMMAND_MODULE_STOP:
                                DBG_PRINT("\r\nResult:COMMAND_MODULE_STOP\r\n");
                                break;

                            case COMMAND_MODULE_START:
                                DBG_PRINT("\r\nResult:COMMAND_MODULE_START\r\n");
                                break;

                             case COMMAND_MODULE_PAUSE:
                                DBG_PRINT("\r\nResult:COMMAND_MODULE_PAUSE\r\n");
                                break;

                            case COMMAND_MODULE_RESUME:
                                DBG_PRINT("\r\nResult:COMMAND_MODULE_RESUME\r\n");
                                break;

                            case COMMAND_FIND_FACE_AND_TRACK:
                                #if API_COMMAND_DRAW == 1
                                    fd_draw.is_best_face = 1;
                                    fd_draw.is_hand = 0;
                                    fd_draw.is_fist = 0;
                                    fd_draw.result_cnt = 1;
                                    fd_draw.rect[0].x = FaceFistResult.x;
                                    fd_draw.rect[0].y = FaceFistResult.y;
                                    fd_draw.rect[0].width = FaceFistResult.width;
                                    fd_draw.rect[0].height = FaceFistResult.height;
                                #else
                                    DBG_PRINT("Result:COMMAND_FIND_FACE_AND_TRACK\r\n");
                                    DBG_PRINT("FaceResult->x = %d \r\n",FaceFistResult.x);
                                    DBG_PRINT("FaceResult->y = %d \r\n",FaceFistResult.y);
                                    DBG_PRINT("FaceResult->width = %d \r\n",FaceFistResult.width);
                                    DBG_PRINT("FaceResult->height = %d \r\n",FaceFistResult.height);
                                #endif
                                break;

                            case COMMAND_FIND_FIST_AND_TRACK:
                                #if API_COMMAND_DRAW == 1
                                    fd_draw.is_best_face = 0;
                                    fd_draw.is_hand = 0;
                                    fd_draw.is_fist = 1;
                                    fd_draw.result_cnt = 1;
                                    fd_draw.rect[FIST_NUM_SET].x = FaceFistResult.x;
                                    fd_draw.rect[FIST_NUM_SET].y = FaceFistResult.y;
                                    fd_draw.rect[FIST_NUM_SET].width = FaceFistResult.width;
                                    fd_draw.rect[FIST_NUM_SET].height = FaceFistResult.height;
                                #else
                                    DBG_PRINT("Result:COMMAND_FIND_FIST_AND_TRACK,\r\n");
                                    DBG_PRINT("FistResult->x = %d \r\n",FaceFistResult.x);
                                    DBG_PRINT("FistResult->y = %d \r\n",FaceFistResult.y);
                                    DBG_PRINT("FistResult->width = %d \r\n",FaceFistResult.width);
                                    DBG_PRINT("FistResult->height = %d \r\n",FaceFistResult.height);
                                #endif
                                break;
                        }

                        //if(face_recognize_en && (command_mode == API_COMMAND_DET_OFF))
                        if(face_recognize_en)
                        {
                            drv_fd_result_lock();
                            fd_draw.is_best_face = fd_draw_result.is_best_face;
                            fd_draw.is_get_eye = fd_draw_result.is_get_eye;
                            fd_draw.result_cnt = fd_draw_result.result_cnt;
                        #if FACE_DETECT_MULTI_PEOPLE_FLOW_EN == 1
                            for(i=0;i<fd_draw.result_cnt;i++)
                            {
                                if(i < HAND_NUM_SET)
                                {
                                    fd_draw.rect[i].x = fd_draw_result.rect[i].x;
                                    fd_draw.rect[i].y = fd_draw_result.rect[i].y;
                                    fd_draw.rect[i].width = fd_draw_result.rect[i].width;
                                    fd_draw.rect[i].height = fd_draw_result.rect[i].height;
                                }
                            }
                        #else
                            fd_draw.rect[0].x = fd_draw_result.rect[FACE_NUM_SET].x;
                            fd_draw.rect[0].y = fd_draw_result.rect[FACE_NUM_SET].y;
                            fd_draw.rect[0].width = fd_draw_result.rect[FACE_NUM_SET].width;
                            fd_draw.rect[0].height = fd_draw_result.rect[FACE_NUM_SET].height;
                        #endif
                            drv_fd_result_unlock();
                        }
#if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
                        drv_fd_result_lock();
                        if(module_mem_set->mode != API_COMMAND_DET_OFF)
                        {
                            if(module_mem_set->mode == API_COMMAND_RESULT_DET)
                            {
                                fd_draw.is_hand = fd_draw_result.is_hand;
                                fd_draw.rect[HAND_NUM_SET].x = fd_draw_result.rect[HAND_NUM_SET].x;
                                fd_draw.rect[HAND_NUM_SET].y = fd_draw_result.rect[HAND_NUM_SET].y;
                                fd_draw.rect[HAND_NUM_SET].width = fd_draw_result.rect[HAND_NUM_SET].width;
                                fd_draw.rect[HAND_NUM_SET].height = fd_draw_result.rect[HAND_NUM_SET].height;

                                fd_draw.rect[MOVINGROI_RIGHT_SET].x = fd_draw_result.rect[MOVINGROI_RIGHT_SET].x;
                                fd_draw.rect[MOVINGROI_RIGHT_SET].y = fd_draw_result.rect[MOVINGROI_RIGHT_SET].y;
                                fd_draw.rect[MOVINGROI_RIGHT_SET].width = fd_draw_result.rect[MOVINGROI_RIGHT_SET].width;
                                fd_draw.rect[MOVINGROI_RIGHT_SET].height = fd_draw_result.rect[MOVINGROI_RIGHT_SET].height;

                                fd_draw.rect[MOVINGROI_LEFT_SET].x = fd_draw_result.rect[MOVINGROI_LEFT_SET].x;
                                fd_draw.rect[MOVINGROI_LEFT_SET].y = fd_draw_result.rect[MOVINGROI_LEFT_SET].y;
                                fd_draw.rect[MOVINGROI_LEFT_SET].width = fd_draw_result.rect[MOVINGROI_LEFT_SET].width;
                                fd_draw.rect[MOVINGROI_LEFT_SET].height = fd_draw_result.rect[MOVINGROI_LEFT_SET].height;

                                fd_draw.is_fist = fd_draw_result.is_fist;
                                fd_draw.rect[FIST_NUM_SET].x = fd_draw_result.rect[FIST_NUM_SET].x;
                                fd_draw.rect[FIST_NUM_SET].y = fd_draw_result.rect[FIST_NUM_SET].y;
                                fd_draw.rect[FIST_NUM_SET].width = fd_draw_result.rect[FIST_NUM_SET].width;
                                fd_draw.rect[FIST_NUM_SET].height = fd_draw_result.rect[FIST_NUM_SET].height;
                            }
                        }
                        drv_fd_result_unlock();
#endif
#else
                        drv_fd_result_lock();
                        fd_draw.is_best_face = fd_draw_result.is_best_face;
                        fd_draw.is_get_eye = fd_draw_result.is_get_eye;
                        fd_draw.result_cnt = fd_draw_result.result_cnt;
                        fd_draw.rect[0].x = fd_draw_result.rect[0].x;
                        fd_draw.rect[0].y = fd_draw_result.rect[0].y;
                        fd_draw.rect[0].width = fd_draw_result.rect[0].width;
                        fd_draw.rect[0].height = fd_draw_result.rect[0].height;
        #if KOT_EN == 1
                        gData_ptr_draw.InitObject.width = gData_ptr_result->InitObject.width;
                        gData_ptr_draw.InitObject.height = gData_ptr_result->InitObject.height;
                        gData_ptr_draw.InitObject.x = gData_ptr_result->InitObject.x;
                        gData_ptr_draw.InitObject.y = gData_ptr_result->InitObject.y;
                        gData_ptr_draw.updateObject = gData_ptr_result->updateObject;
                        if(face_detect_en)
                            gData_ptr_draw.KOT_initFlg = gData_ptr_result->KOT_initFlg;
                        else
                            gData_ptr_draw.KOT_initFlg = 0;
                        gData_ptr_draw.trackTerminateFlg = gData_ptr_result->trackTerminateFlg;
        #endif
        #if HAND_EN == 1
                        fd_draw.is_hand = fd_draw_result.is_hand;
                        fd_draw.rect[HAND_NUM_SET].x = fd_draw_result.rect[HAND_NUM_SET].x;
                        fd_draw.rect[HAND_NUM_SET].y = fd_draw_result.rect[HAND_NUM_SET].y;
                        fd_draw.rect[HAND_NUM_SET].width = fd_draw_result.rect[HAND_NUM_SET].width;
                        fd_draw.rect[HAND_NUM_SET].height = fd_draw_result.rect[HAND_NUM_SET].height;
        #if PFMST_EN == 1
                        context_hand_draw.detect_total = context_hand_result.detect_total;
                        if(hand_detect_en)
                            context_hand_draw.tracking_flag = context_hand_result.tracking_flag;
                        else
                            context_hand_draw.tracking_flag = 0;
                        trackingSet_hand_draw[0].width = trackingSet_hand_result[0].width;
                        trackingSet_hand_draw[0].height = trackingSet_hand_result[0].height;
                        trackingSet_hand_draw[0].x = trackingSet_hand_result[0].x;
                        trackingSet_hand_draw[0].y = trackingSet_hand_result[0].y;
        #endif
                        fd_draw.is_fist = fd_draw_result.is_fist;
                        fd_draw.rect[FIST_NUM_SET].x = fd_draw_result.rect[FIST_NUM_SET].x;
                        fd_draw.rect[FIST_NUM_SET].y = fd_draw_result.rect[FIST_NUM_SET].y;
                        fd_draw.rect[FIST_NUM_SET].width = fd_draw_result.rect[FIST_NUM_SET].width;
                        fd_draw.rect[FIST_NUM_SET].height = fd_draw_result.rect[FIST_NUM_SET].height;
        #if PFMST_EN == 1
                        context_fist_draw.detect_total = context_fist_result.detect_total;
                        if(fist_detect_en)
                            context_fist_draw.tracking_flag = context_fist_result.tracking_flag;
                        else
                            context_fist_draw.tracking_flag = 0;
                        trackingSet_fist_draw[0].width = trackingSet_fist_result[0].width;
                        trackingSet_fist_draw[0].height = trackingSet_fist_result[0].height;
                        trackingSet_fist_draw[0].x = trackingSet_fist_result[0].x;
                        trackingSet_fist_draw[0].y = trackingSet_fist_result[0].y;
        #endif
        #endif
                        drv_fd_result_unlock();
#endif
                    }

                    if(fd_draw.result_cnt && fd_draw.is_best_face)
                    {
                        draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, fd_draw.result_cnt, &fd_draw.rect[0], 0xd211d290);
                    }
        #if KOT_EN == 1
                    else if(gData_ptr_draw.KOT_initFlg == 1 && gData_ptr_draw.trackTerminateFlg == 0)
                    {
                        #if FUNCTION_DRAW_DEBUG_EN == 1
                            draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &gData_ptr_draw.updateObject, 0x10801080);
                        #else
                            draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &gData_ptr_draw.updateObject, 0xd211d290);
                        #endif
                    }
        #endif
        #if HAND_EN == 1
                    if(fd_draw.is_hand)
                    {
                        draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &fd_draw.rect[HAND_NUM_SET], 0xd211d290);
                        #if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1 && FD_COMMAND_FLOW_EN == 1
                            draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 2, &fd_draw.rect[MOVINGROI_RIGHT_SET], 0xd211d290);
                        #endif
                    }
        #if PFMST_EN == 1
                    else if((context_hand_draw.detect_total == 0) && (context_hand_draw.tracking_flag != 0))
                    {
                        #if FUNCTION_DRAW_DEBUG_EN == 1
                        draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &trackingSet_hand_draw[0], 0x22222222/*0xd211d290*//*0x10801080*/);
                        #else
                        draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &trackingSet_hand_draw[0], 0xd211d290/*0xd211d290*//*0x10801080*/);
                        #endif
                    }
        #endif
                    if(fd_draw.is_fist)
                    {
                        draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &fd_draw.rect[FIST_NUM_SET], 0xd211d290);
                    }
        #if PFMST_EN == 1
                    else if((context_fist_draw.detect_total == 0) && (context_fist_draw.tracking_flag != 0))
                    {
                        #if FUNCTION_DRAW_DEBUG_EN == 1
                        draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &trackingSet_fist_draw[0], 0x10801080/*0xd211d290*//*0x10801080*/);
                        #else
                        draw_obj(PscalerBuffer, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, 1, &trackingSet_fist_draw[0], 0xd211d290/*0xd211d290*//*0x10801080*/);
                        #endif
                    }
        #endif
        #endif
        #endif

        #if DMA_SP_DRAW_EN == 1
                if(face_detect_en)
                    drv_l1_dma_buffer_copy((INT32U)&_SPRITE_GPM4_face_CellData[0], (PscalerBuffer+((PPU_TEXT_SIZE_VPIXEL/2)*(PPU_TEXT_SIZE_HPIXEL*2))), (IMG_H_SIZE*IMG_H_SIZE*2), (IMG_H_SIZE*2), (PPU_TEXT_SIZE_HPIXEL*2));
                if(hand_detect_en)
                    drv_l1_dma_buffer_copy((INT32U)&_SPRITE_GPM4_hand_CellData[0], (PscalerBuffer+(((PPU_TEXT_SIZE_VPIXEL/2)-36)*(PPU_TEXT_SIZE_HPIXEL*2))), (IMG_H_SIZE*IMG_H_SIZE*2), (IMG_H_SIZE*2), (PPU_TEXT_SIZE_HPIXEL*2));
                if(fist_detect_en)
                    drv_l1_dma_buffer_copy((INT32U)&_SPRITE_GPM4_fist_CellData[0], (PscalerBuffer+(((PPU_TEXT_SIZE_VPIXEL/2)+36)*(PPU_TEXT_SIZE_HPIXEL*2))), (IMG_H_SIZE*IMG_H_SIZE*2), (IMG_H_SIZE*2), (PPU_TEXT_SIZE_HPIXEL*2));
        #endif

        #if PPU_DRAW_EN == 1
                    //fd_ppu_go(PPU_TEXT_SIZE_HPIXEL,PPU_TEXT_SIZE_VPIXEL,PscalerBuffer);
                    //pscaler_frame_buffer_add((INT32U *)PscalerBuffer, 1);
                   //PscalerBuffer = ppu_frame_buffer_display_get();
        #endif

                    //DBG_PRINT("X ");
                    if(PscalerBuffer)
        #if DISPLAY_USE_PSCALER_EN == 1
                        gplib_ppu_frame_buffer_add(ppu_register_set, PscalerBuffer);
        #else
					#if WAFD_DEMO_EN == 0
                        disp_frame_buffer_add((INT32U *)PscalerBuffer, 1);
					#else
					{
                        //DBG_PRINT("V");
						osMessagePut(vid_enc_task_q, (INT32U)&PscalerBuffer, osWaitForever);
                    }
					#endif
        #endif
                }
                //else
                    //DBG_PRINT("@");
		#if WAFD_DEMO_EN == 1
		//DBG_PRINT("R");
		osMessagePut(scaler_frame_q, (uint32_t)&csi_buf, osWaitForever);

		#else
        #if FD_EN == 1
                if(fd_event != FD_STATE_OK)
        #endif
                    free_frame_buffer_add((INT32U *)csi_buf, 1);
        #if CSI_FULL_IMAGE_FLOW_EN == 1
                if(pscaler_clip_buffer)
                    pscaler_clip_free_buffer_add((INT32U *)pscaler_clip_buffer,1);
        #endif
        #if CSI_SOFTWARE_CNT_EN == 1
                t2 = xTaskGetTickCount();
                if((t2 - t1) > 30)
                    DBG_PRINT("csi_task = %d \r\n", (t2 - t1));
        #endif
		#endif
                break;
        }
    }
}

static void disp_task_entry(void const *parm)
{
    INT32U display_buf,ack_msg;
#if ROTATOR_FLIP_EN == 1
    INT32U rotator_buf,rotator_addr,rotator_flag;
#endif
    osEvent result;

    DBG_PRINT("disp_task_entry start \r\n");
    // Initialize display device

    #if PPU_DRAW_EN == 0
        drv_l2_display_init();
        drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);
    #endif

    #if ROTATOR_FLIP_EN == 1
        rotator_init();
        rotator_buf = (INT32U)gp_malloc_align((FD_SIZE_HPIXEL*FD_SIZE_VPIXEL*2)*2, 64);
        if(rotator_buf == 0)
        {
            DBG_PRINT("rotator_buf malloc fail \r\n");
            while(1);
        }
        else
            DBG_PRINT("rotator_buf = 0x%x \r\n", rotator_buf);
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

        switch(display_buf)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_DISP_TASK_EXIT]\r\n");
                #if ROTATOR_FLIP_EN == 1
                    if(rotator_buf)
                    {
                        gp_free((void *)rotator_buf);
                        rotator_buf = 0;
                    }
                #endif
                drv_l2_display_stop(DISPLAY_DEVICE);
                drv_l2_display_uninit();
                ack_msg = ACK_OK;
                osMessagePut(disp_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(disp_id);
                break;

            default:
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

        #if PPU_DRAW_EN == 1
            #if DISPLAY_USE_PSCALER_EN == 1
                    ppu_pscaler_buffer_add((INT32U *)display_buf, 1);
            #else
                    gplib_ppu_frame_buffer_add(ppu_register_set, display_buf);
            #endif
        #else
                pscaler_frame_buffer_add((INT32U *)display_buf, 1);
        #endif
                break;
        }
    }
}

static INT32S user_PFMST_CROP_hw_function(unsigned int in_buffer, unsigned short in_w, unsigned short in_h, unsigned int kernel_buffer, unsigned short out_w, unsigned short out_h, unsigned short crop_x, unsigned short crop_y, unsigned short crop_w, unsigned short crop_h)
{
    INT32S ret;
	INT32S x, y, xstep, ystep, i, j, t, xstart, ystart, dxMin, dxMax, dyMin, dyMax, jStep, iStep;
    gpRect clip;
    gpImage src;
    gpImage dst;
	unsigned char *pPixel;
	unsigned char *image;
	unsigned char *kernel;

	unsigned int img_index_cnt, idx, SAMPLE_STEP, YUV_index;
	unsigned int startX;
	signed int Center_x, Center_y;

	SAMPLE_STEP = 4;

//	print_string("1- (%d, %d, %d, %d)\r\n", crop_x, crop_y, crop_w, crop_h );

    if(in_buffer == 0 || kernel_buffer == 0) {
		return STATUS_FAIL;
	}
//		DBG_PRINT("( %d, %d, %d, %d, %d, %d, %d, %d )\r\n", in_w, in_h, out_w, out_h, crop_x, crop_y, crop_w, crop_h);


	//drv_fd_lock();

    src.width = in_w;
    src.height = in_h;
    src.ch  = 2;
    src.widthStep = in_w*2;
    src.format = IMG_FMT_UYVY;
    src.ptr = (INT8U *)in_buffer;

    dst.width = out_w;
    dst.height = out_h;
    dst.ch = 2;
    dst.widthStep = (out_w)*2;
    dst.format = IMG_FMT_UYVY;
    dst.ptr = (INT8U *)kernel_buffer;

    clip.x = crop_x;
    clip.y = crop_y;
    clip.width = crop_w;
    clip.height = crop_h;

    // default to be 16X16
//    xstep = (int)(crop_w >> 4);
//    ystep = (int)(crop_h >> 4);


    i = 0;
    j = 0;
    t = 0;

    iStep = (int)(crop_w>>(SAMPLE_STEP));

	if( iStep <= 1 )
	{
		iStep =1;
	}

	jStep = (int)(crop_h>>(SAMPLE_STEP));

	if( jStep <= 1 )
	{
		jStep =1;
	}

	dxMin = -( iStep << ( SAMPLE_STEP - 1 ) );//+iStep;
	dxMax = ( iStep << ( SAMPLE_STEP - 1 ) );//-iStep;
	dyMin = -( jStep << ( SAMPLE_STEP - 1 ) );//+jStep;
	dyMax = ( jStep << ( SAMPLE_STEP - 1 ) );//-jStep;

    // protection
    if( crop_x < 0 ) {
        crop_x = 0;
    }
    if( crop_y < 0 ) {
        crop_y = 0;
    }
    if( ( crop_x + crop_w ) > in_w ) {
        crop_x = in_w - crop_w;
    }
    if( ( crop_y + crop_h ) > in_h ) {
        crop_y = in_h - crop_h;
    }


   // print_string("( %d, %d, %d, %d, %d, %d )\r\n", crop_x, crop_y, crop_w, crop_h, iStep, jStep );

    // 計算影像中心點 
    Center_x = crop_x + (crop_w>>1);
	image = src.ptr +( ( ( ((crop_y+(crop_h>>1)) * src.width) + Center_x) ) << 1 );

	kernel = (INT8U *)kernel_buffer;

//    print_string("%x\r\n", kernel );
//	print_string("2 - (%d, %d, %d, %d)\r\n", crop_x, crop_y, crop_w, crop_h );
    img_index_cnt = 0;
    YUV_index = 0;
    for(j = dyMin; j < dyMax; j += jStep)
    {
        for( i = dxMin; i < dxMax; i += iStep )
        {
            idx = (unsigned int)ABS( i + Center_x );
            if( YUV_index == 1  ) {
                if( idx & 0x01)
                {
                    kernel[ img_index_cnt] = image[ ( ( j * src.width + i ) << 1 ) ];   //v
            img_index_cnt++;
                    kernel[ img_index_cnt ] = image[( ( j * src.width + i ) << 1) +1];          //y
            img_index_cnt++;


        }
                else
                {
                    kernel[ img_index_cnt] = image[ ( ( j * src.width + i ) << 1 ) + 2 ];   //v
                    img_index_cnt++;
                    kernel[ img_index_cnt ] = image[( ( j * src.width + i ) << 1)+1 ];          //y
                    img_index_cnt++;
    }

                //print_string( "1, idx = %d, img_index_cnt = %d\r\n", idx, img_index_cnt);
                YUV_index = 0;


            }
            else {

                if( idx & 0x01)
                {
                    kernel[ img_index_cnt] = image[ ( ( j * src.width + i ) << 1 )-2 ];   //u
                    img_index_cnt++;
                    kernel[ img_index_cnt ] = image[( ( j * src.width + i ) << 1) +1];          //y
                    img_index_cnt++;
                }
                else
                {
                    kernel[ img_index_cnt] = image[ ( ( j * src.width + i ) << 1 )  ];   //u
                    img_index_cnt++;
                    kernel[ img_index_cnt ] = image[( ( j * src.width + i ) << 1)+1 ];          //y
                    img_index_cnt++;
                }
    //                print_string( "1, idx = %d\r\n", idx, i, Center_x );

                YUV_index = 1;
            }

        }
    }

//    print_string("img_index_cnt = %d, idx = %d, %x\r\n", img_index_cnt, idx, kernel );
//    print_string("dxdy( %d, %d, %d, %d, %d, %d )\r\n", dxMin, dxMax, dyMin, dyMax, iStep, jStep );
//	ret = drv_l2_scaler_clip(0, 1, &src, &dst, &clip);

    ret = STATUS_OK;

Return:
	//drv_fd_unlock();

    return ret;
}

#if FD_EN == 1 || HAND_EN == 1
#if FD_COMMAND_FLOW_EN == 1
static INT32S face_hand_fist_detect_result_get(module_command_t *info_ptr, INT32U find_mode,INT32U yuv_buf, INT32U y_buf)
{
    #define FIND_USER_RANGE_EN        0xFF
    #define FIND_FIRT_CNT             10//9999//10
    INT32U mode,fist_cnt,fist_hand_cnt,find_hand_state;
    INT32S ret = 0;
    gpRect find_result;

    if(yuv_buf == 0 || y_buf == 0)
        return -1;

    drv_fd_result_lock();
    mode = info_ptr->mode;
    info_ptr->ImageModule_ptr.ptr = (INT8U *)y_buf;
    drv_fd_result_unlock();

    if(find_mode == API_FIND_FACE) // find face
    {
        fist_he_pos.x = 0;
        fist_he_pos.y = 0;
        fist_he_pos.width = 0;
        fist_he_pos.height = 0;
    #if 0
        if(info_ptr->find_hand)
        {
            info_ptr->result[HAND_NUM_SET].x = 0;
            info_ptr->result[HAND_NUM_SET].y = 0;
            info_ptr->result[HAND_NUM_SET].width = 0;
            info_ptr->result[HAND_NUM_SET].height = 0;
            info_ptr->find_hand = 0;
        }

        if(info_ptr->find_fist)
        {
            info_ptr->result[FIST_NUM_SET].x = 0;
            info_ptr->result[FIST_NUM_SET].y = 0;
            info_ptr->result[FIST_NUM_SET].width = 0;
            info_ptr->result[FIST_NUM_SET].height = 0;
            info_ptr->prcess_result[FIST_NUM_SET].x = 0;
            info_ptr->prcess_result[FIST_NUM_SET].y = 0;
            info_ptr->prcess_result[FIST_NUM_SET].width = 0;
            info_ptr->prcess_result[FIST_NUM_SET].height = 0;
            info_ptr->find_fist = 0;
        }
    #endif
        ret = face_Detect_only((gpImage *)&info_ptr->ImageModule_ptr, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,0);
        #if KOT_EN == 1
            context.detect_total = ret;
            if(context.detect_total != 0)
            {
                gData_ptr->objSet = (gpRect*)&fd_result.rect[0];
            }

            if((context.detect_total != 0 && gData_ptr->modelAlreadyFlg == 0) || (context.detect_total != 0))
            {
                gData_ptr->InitObject.width = fd_result.rect[0].width - (fd_result.rect[0].width>>2);
                gData_ptr->InitObject.height = fd_result.rect[0].height - (fd_result.rect[0].height>>2);
                gData_ptr->InitObject.x = fd_result.rect[0].x + (fd_result.rect[0].width>>3);
                gData_ptr->InitObject.y = fd_result.rect[0].y + (fd_result.rect[0].height>>3);
                gData_ptr->updateObject = gData_ptr->InitObject;

                gData_ptr->KOT_initFlg = 1;
            }

            if(gData_ptr->KOT_initFlg == 1)
            {
                info_ptr->objClip.x = gData_ptr->updateObject.x;
                info_ptr->objClip.y = gData_ptr->updateObject.y;
                info_ptr->objClip.width = gData_ptr->updateObject.width;
                info_ptr->objClip.height = gData_ptr->updateObject.height;
                KOT_scalerClipStart((gpImage *)&info_ptr->ImageModule_ptr, &info_ptr->objROI, info_ptr->objClip);
                KOT_scalerEnd();
                GPSmoothImage(info_ptr->objROI.ptr, info_ptr->ROISmoothImage.ptr, info_ptr->objROI.width, info_ptr->objROI.height);
                keypointsObjectTracker((void *)KOT_workmem, &info_ptr->ROISmoothImage, context.detect_total);
            }
        #endif

        drv_fd_result_lock();
        info_ptr->find_face = ret;
        if(ret)
        {
            info_ptr->result[FACE_NUM_SET].width = fd_result.rect[0].width- (fd_result.rect[0].width>>2);
            info_ptr->result[FACE_NUM_SET].height = fd_result.rect[0].height - (fd_result.rect[0].height>>2);
            info_ptr->result[FACE_NUM_SET].x = fd_result.rect[0].x+ (fd_result.rect[0].width>>3);
            info_ptr->result[FACE_NUM_SET].y = fd_result.rect[0].y+ (fd_result.rect[0].height>>3);
        }
        else
        {
            #if KOT_EN == 1
                if(gData_ptr->KOT_initFlg == 1 && gData_ptr->trackTerminateFlg == 0)
                {
                    info_ptr->result[FACE_NUM_SET].x = gData_ptr->InitObject.x;
                    info_ptr->result[FACE_NUM_SET].y = gData_ptr->InitObject.y;
                    info_ptr->result[FACE_NUM_SET].width = gData_ptr->InitObject.width;
                    info_ptr->result[FACE_NUM_SET].height = gData_ptr->InitObject.height;
                    info_ptr->find_face = ret = 1;
                    //DBG_PRINT("k");
                }
            #endif
        }
        drv_fd_result_unlock();
    }
    else if(find_mode == API_FIND_HAND)
    {
        fist_he_pos.x = 0;
        fist_he_pos.y = 0;
        fist_he_pos.width = 0;
        fist_he_pos.height = 0;
    #if 0
        if(info_ptr->find_face)
        {
            info_ptr->result[FACE_NUM_SET].x = 0;
            info_ptr->result[FACE_NUM_SET].y = 0;
            info_ptr->result[FACE_NUM_SET].width = 0;
            info_ptr->result[FACE_NUM_SET].height = 0;
            info_ptr->find_face = 0;
        }
    #endif
        drv_fd_result_lock();
        fist_hand_cnt = info_ptr->hand_turn_page_on;
        drv_fd_result_unlock();
        handWorkMem_ptr->image.ptr = (INT8U *)y_buf;

        if(fist_hand_cnt && (mode == API_COMMAND_RESULT_DET))
        {
            DBG_PRINT("*0\r\n");
            drv_fd_result_lock();
            find_result.x = info_ptr->result[HAND_NUM_SET2].x;
            find_result.y = info_ptr->result[HAND_NUM_SET2].y;
            find_result.width = info_ptr->result[HAND_NUM_SET2].width;
            find_result.height = info_ptr->result[HAND_NUM_SET2].height;
            drv_fd_result_unlock();
            //ret = gesture_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr, (gpRect *)&find_result, 1);
            ret = 0;
        }
        else
        {
            //DBG_PRINT("*00\r\n");
            drv_fd_result_lock();
            find_hand_state = info_ptr->find_hand_mode;
            drv_fd_result_unlock();

            if((mode == API_COMMAND_RESULT_DET))
                ret = gesture_detect_mode_proc((ObjDetect_t *)handWorkMem_ptr, find_hand_state);
            else
                ret = gesture_detect_mode_proc((ObjDetect_t *)handWorkMem_ptr, 1);

            drv_fd_result_lock();
            if(ret)
            {
                //DBG_PRINT("aeawb lock\r\n");
                //cv_aeawb_open_flag = 0;
                //cv_aeawb_flag_lock();

                info_ptr->result[HAND_NUM_SET2].x = hand_result.rect[0].x;
                info_ptr->result[HAND_NUM_SET2].y = hand_result.rect[0].y;
                info_ptr->result[HAND_NUM_SET2].width = hand_result.rect[0].width;
                info_ptr->result[HAND_NUM_SET2].height = hand_result.rect[0].height;
            }
            drv_fd_result_unlock();
            //if(ret)
                //DBG_PRINT("(%d, %d)\r\n", info_ptr->result[HAND_NUM_SET2].x, info_ptr->result[HAND_NUM_SET2].y);
        }

        #if PFMST_EN == 1
            context_fist.terminationCnt = 100;
            PFMSTracker_framework( (INT8U*)yuv_buf, fistWorkMem_ptr->image.width, fistWorkMem_ptr->image.height, &context_fist, &fist_result.rect[0]);
        #else
            context_hand.detect_total = ret;
            PFMSTracker_framework((INT8U*)yuv_buf, handWorkMem_ptr->image.width, handWorkMem_ptr->image.height, &context_hand, &hand_result.rect[0]);
        #endif
        drv_fd_result_lock();
        info_ptr->find_hand = ret;
        if(ret)
        {
            info_ptr->result[HAND_NUM_SET].x = hand_result.rect[0].x;
            info_ptr->result[HAND_NUM_SET].y = hand_result.rect[0].y;
            info_ptr->result[HAND_NUM_SET].width = hand_result.rect[0].width;
            info_ptr->result[HAND_NUM_SET].height = hand_result.rect[0].height;
            info_ptr->result[FIST_NUM_SET2].x = info_ptr->result[HAND_NUM_SET].x;
            info_ptr->result[FIST_NUM_SET2].y = info_ptr->result[HAND_NUM_SET].y;
            info_ptr->result[FIST_NUM_SET2].width = info_ptr->result[HAND_NUM_SET].width;
            info_ptr->result[FIST_NUM_SET2].height = info_ptr->result[HAND_NUM_SET].height;
        }
        else
        {
            #if 0//PFMST_EN == 1
                if(context_hand.tracking_flag != 0)
                {
                    info_ptr->result[HAND_NUM_SET].x = (unsigned int)(context_hand.tracked_obj.width);
                    info_ptr->result[HAND_NUM_SET].y = (unsigned int)(context_hand.tracked_obj.height);
                    info_ptr->result[HAND_NUM_SET].width = (unsigned int)(context_hand.tracked_obj.center.x - context_hand.tracked_obj.width/2);
                    info_ptr->result[HAND_NUM_SET].height = (unsigned int)(context_hand.tracked_obj.center.y - context_hand.tracked_obj.height/2);
                    info_ptr->find_hand = ret = 1;
                }
            #endif
        }
        drv_fd_result_unlock();

    }
    else if(find_mode == API_FIND_FIST)
    {
        drv_fd_result_lock();
        if(fist_he_pos.x == 0)
        {
            fist_he_pos.x = info_ptr->result[HAND_NUM_SET].x;
            fist_he_pos.y = info_ptr->result[HAND_NUM_SET].y;
            fist_he_pos.width = info_ptr->result[HAND_NUM_SET].width;
            fist_he_pos.height = info_ptr->result[HAND_NUM_SET].height;
        }
        else
        {
            fist_he_pos.x = info_ptr->result[FIST_NUM_SET].x;
            fist_he_pos.y = info_ptr->result[FIST_NUM_SET].y;
            fist_he_pos.width = info_ptr->result[FIST_NUM_SET].width;
            fist_he_pos.height = info_ptr->result[FIST_NUM_SET].height;
        }
        drv_fd_result_unlock();
    #if 0
        if(info_ptr->find_face)
        {
            info_ptr->result[FACE_NUM_SET].x = 0;
            info_ptr->result[FACE_NUM_SET].y = 0;
            info_ptr->result[FACE_NUM_SET].width = 0;
            info_ptr->result[FACE_NUM_SET].height = 0;
            info_ptr->find_face = 0;
        }
    #endif
        fistWorkMem_ptr->image.ptr = (INT8U *)y_buf;
        drv_fd_result_lock();
        fist_cnt = info_ptr->fist_cnt;
        fist_hand_cnt = info_ptr->find_hand;
        drv_fd_result_unlock();
        if(fist_hand_cnt && (mode == API_COMMAND_RESULT_DET))
        {
            drv_fd_result_lock();
            find_result.x = info_ptr->result[HAND_NUM_SET].x;
            find_result.y = info_ptr->result[HAND_NUM_SET].y;
            find_result.width = info_ptr->result[HAND_NUM_SET].width;
            find_result.height = info_ptr->result[HAND_NUM_SET].height;
            info_ptr->result[FIST_NUM_SET2].x = info_ptr->result[HAND_NUM_SET].x;
            info_ptr->result[FIST_NUM_SET2].y = info_ptr->result[HAND_NUM_SET].y;
            info_ptr->result[FIST_NUM_SET2].width = info_ptr->result[HAND_NUM_SET].width;
            info_ptr->result[FIST_NUM_SET2].height = info_ptr->result[HAND_NUM_SET].height;
            drv_fd_result_unlock();
            #if 0
                if(find_result.x == 0)
                    ret = fist_detect_proc((ObjDetect_t *)fistWorkMem_ptr);
                else
                    ret = fist_detect_proc_with_pfmst((ObjDetect_t *)fistWorkMem_ptr,(gpRect *)&find_result, 1);
            #endif
        }
        else if((fist_cnt < FIND_FIRT_CNT) && (mode == API_COMMAND_RESULT_DET))
        {
            drv_fd_result_lock();
            info_ptr->fist_cnt++;
            if(info_ptr->result[FIST_NUM_SET2].x == 0)
            {
                info_ptr->result[FIST_NUM_SET2].x = info_ptr->result[HAND_NUM_SET].x;
                info_ptr->result[FIST_NUM_SET2].y = info_ptr->result[HAND_NUM_SET].y;
                info_ptr->result[FIST_NUM_SET2].width = info_ptr->result[HAND_NUM_SET].width;
                info_ptr->result[FIST_NUM_SET2].height = info_ptr->result[HAND_NUM_SET].height;
            }
            find_result.x = info_ptr->result[FIST_NUM_SET2].x;
            find_result.y = info_ptr->result[FIST_NUM_SET2].y;
            find_result.width = info_ptr->result[FIST_NUM_SET2].width;
            find_result.height = info_ptr->result[FIST_NUM_SET2].height;
            drv_fd_result_unlock();
            if(find_result.x == 0)
            {
                //DBG_PRINT("#");
                ret = fist_detect_proc((ObjDetect_t *)fistWorkMem_ptr);
            }
            else
            {
                //DBG_PRINT("^");
                ret = fist_detect_proc_with_pfmst((ObjDetect_t *)fistWorkMem_ptr,(gpRect *)&find_result,1);
                //ret = fist_detect_proc((ObjDetect_t *)fistWorkMem_ptr);
            }
        }
        else
        {
            fist_cnt = FIND_USER_RANGE_EN;//0;
            if(fist_cnt == FIND_USER_RANGE_EN)
            {
                //DBG_PRINT("--");
                drv_fd_result_lock();
                find_result.x = info_ptr->result[FIST_NUM_SET2].x;
                find_result.y = info_ptr->result[FIST_NUM_SET2].y;
                find_result.width = info_ptr->result[FIST_NUM_SET2].width;
                find_result.height = info_ptr->result[FIST_NUM_SET2].height;
                drv_fd_result_unlock();
                ret = fist_detect_proc_with_pfmst((ObjDetect_t *)fistWorkMem_ptr,(gpRect *)&find_result,1);
                if(ret)
                {
                    drv_fd_result_lock();
                    info_ptr->result[FIST_NUM_SET2].x = fist_result.rect[0].x;
                    info_ptr->result[FIST_NUM_SET2].y = fist_result.rect[0].y;
                    info_ptr->result[FIST_NUM_SET2].width = fist_result.rect[0].width;
                    info_ptr->result[FIST_NUM_SET2].height = fist_result.rect[0].height;
                    drv_fd_result_unlock();
                }
            }
            else
            {
               ret = fist_detect_proc((ObjDetect_t *)fistWorkMem_ptr);
            }
        }

        if(ret && (info_ptr->result[FIST_NUM_SET3].x == 0))
        {
            //DBG_PRINT("+");
            info_ptr->result[FIST_NUM_SET3].x = fist_result.rect[0].x;
            info_ptr->result[FIST_NUM_SET3].y = fist_result.rect[0].y;
            info_ptr->result[FIST_NUM_SET3].width = fist_result.rect[0].width;
            info_ptr->result[FIST_NUM_SET3].height = fist_result.rect[0].height;

        }

        #if PFMST_EN == 1
            //if(mode == API_COMMAND_HAND_FIST_DET)
            {
                if(info_ptr->result[FIST_NUM_SET3].x != 0)
                {
                    context_fist.detect_total = ret;
                    PFMSTracker_framework( (INT8U*)yuv_buf, fistWorkMem_ptr->image.width, fistWorkMem_ptr->image.height, &context_fist, &fist_result.rect[0]);
                    //DBG_PRINT("z0 dist_square = %f\r\n", context_fist.dist_square);
                    //DBG_PRINT("PFMST_F\r\n");
                }
            }
        #endif

        drv_fd_result_lock();
        info_ptr->find_fist = ret;
        drv_fd_result_unlock();

#if 0   // for debug
        // find hand for error state
        drv_fd_result_lock();
        find_hand_state = info_ptr->hand_find_with_fist_start;
        drv_fd_result_unlock();
        if(find_hand_state)
        {
            drv_fd_result_lock();
            find_result.x = (info_ptr->result[FIST_NUM_SET].x - (info_ptr->result[FIST_NUM_SET].width / 2));
            find_result.y = (info_ptr->result[FIST_NUM_SET].y - (info_ptr->result[FIST_NUM_SET].height / 2));
            find_result.width = (info_ptr->result[FIST_NUM_SET].width + (info_ptr->result[FIST_NUM_SET].width / 2));
            find_result.height = (info_ptr->result[FIST_NUM_SET].height + (info_ptr->result[FIST_NUM_SET].height / 2));
            drv_fd_result_unlock();

            find_hand_state = gesture_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr, (gpRect *)&find_result, 1);

            drv_fd_result_lock();
            if(find_hand_state)
            {
                DBG_PRINT("&1");
                ret = API_FIND_HAND;
                info_ptr->hand_find_with_fist_start = ret;
            }
            DBG_PRINT("&2");
            drv_fd_result_unlock();
        }
#endif

        if(ret)
        {
            drv_fd_result_lock();
            info_ptr->result[FIST_NUM_SET].x = fist_result.rect[0].x;
            info_ptr->result[FIST_NUM_SET].y = fist_result.rect[0].y;
            info_ptr->result[FIST_NUM_SET].width = fist_result.rect[0].width;
            info_ptr->result[FIST_NUM_SET].height = fist_result.rect[0].height;
            drv_fd_result_unlock();
#if 0
            // find hand for error state
            //DBG_PRINT("@@1");
            drv_fd_result_lock();
            find_hand_state = info_ptr->hand_find_with_fist_start;
            drv_fd_result_unlock();
            if(find_hand_state)
            {
                drv_fd_result_lock();
                #if 1
                    find_result.x = info_ptr->result[FIST_NUM_SET].x;
                    find_result.y = info_ptr->result[FIST_NUM_SET].y;
                    find_result.width = info_ptr->result[FIST_NUM_SET].width;
                    find_result.height = info_ptr->result[FIST_NUM_SET].height;
                #else
                    find_result.x = (info_ptr->result[FIST_NUM_SET].x - (info_ptr->result[FIST_NUM_SET].width / 1));
                    find_result.y = (info_ptr->result[FIST_NUM_SET].y - (info_ptr->result[FIST_NUM_SET].height / 1));
                    find_result.width = (info_ptr->result[FIST_NUM_SET].width + (info_ptr->result[FIST_NUM_SET].width / 1));
                    find_result.height = (info_ptr->result[FIST_NUM_SET].height + (info_ptr->result[FIST_NUM_SET].height / 1));
                #endif
                drv_fd_result_unlock();

                find_hand_state = gesture_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr, (gpRect *)&find_result, 1);

                drv_fd_result_lock();
                if(find_hand_state)
                {
                    DBG_PRINT("@@2");
                    ret = API_FIND_HAND;
                    info_ptr->hand_find_with_fist_start = ret;
                }
                DBG_PRINT("@@3");
                drv_fd_result_unlock();
            }
#endif
        }
        else
        {
            #if PFMST_EN == 1
                //if(mode == API_COMMAND_HAND_FIST_DET)
                {
                    drv_fd_result_lock();
                    fist_hand_cnt = info_ptr->result[FIST_NUM_SET3].x;
                    drv_fd_result_unlock();
                    //DBG_PRINT("%1");
                    if((context_fist.tracking_flag != 0) && (fist_hand_cnt != 0))
                    {
                        //DBG_PRINT("##1");
                        drv_fd_result_lock();
                        info_ptr->result[FIST_NUM_SET].width = (unsigned int)(context_fist.tracked_obj.width);
                        info_ptr->result[FIST_NUM_SET].height = (unsigned int)(context_fist.tracked_obj.height);
                        info_ptr->result[FIST_NUM_SET].x = (unsigned int)(context_fist.tracked_obj.center.x - context_fist.tracked_obj.width/2);
                        info_ptr->result[FIST_NUM_SET].y = (unsigned int)(context_fist.tracked_obj.center.y - context_fist.tracked_obj.height/2);
                        if(fist_cnt == FIND_USER_RANGE_EN)
                        {
                            info_ptr->result[FIST_NUM_SET2].x = info_ptr->result[FIST_NUM_SET].x;
                            info_ptr->result[FIST_NUM_SET2].y = info_ptr->result[FIST_NUM_SET].y;
                            info_ptr->result[FIST_NUM_SET2].width = info_ptr->result[FIST_NUM_SET].width;
                            info_ptr->result[FIST_NUM_SET2].height = info_ptr->result[FIST_NUM_SET].height;
                        }
                        info_ptr->find_fist = ret = 1;
                        drv_fd_result_unlock();
#if 0
                        //DBG_PRINT("##1");
                        // find hand for error state
                        drv_fd_result_lock();
                        find_hand_state = info_ptr->hand_find_with_fist_start;
                        drv_fd_result_unlock();
                        if(find_hand_state)
                        {
                            drv_fd_result_lock();
                            #if 1
                                find_result.x = info_ptr->result[FIST_NUM_SET].x;
                                find_result.y = info_ptr->result[FIST_NUM_SET].y;
                                find_result.width = info_ptr->result[FIST_NUM_SET].width;
                                find_result.height = info_ptr->result[FIST_NUM_SET].height;
                            #else
                                find_result.x = (info_ptr->result[FIST_NUM_SET].x - (info_ptr->result[FIST_NUM_SET].width / 1));
                                find_result.y = (info_ptr->result[FIST_NUM_SET].y - (info_ptr->result[FIST_NUM_SET].height / 1));
                                find_result.width = (info_ptr->result[FIST_NUM_SET].width + (info_ptr->result[FIST_NUM_SET].width / 1));
                                find_result.height = (info_ptr->result[FIST_NUM_SET].height + (info_ptr->result[FIST_NUM_SET].height / 1));
                            #endif
                            drv_fd_result_unlock();

                            find_hand_state = gesture_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr, (gpRect *)&find_result, 1);

                            drv_fd_result_lock();
                            if(find_hand_state)
                            {
                                DBG_PRINT("##2");
                                ret = API_FIND_HAND;
                                info_ptr->hand_find_with_fist_start = ret;
                            }
                            drv_fd_result_unlock();
                            //DBG_PRINT("##3");
                        }
#endif
                        //DBG_PRINT("PFMST_F\r\n");
                    }
                    //DBG_PRINT("context_fist.tracking_flag = %d \r\n", context_fist.tracking_flag);
                    //DBG_PRINT("context_fist.dist_square = %f\r\n",context_fist.dist_square);
                }
            #endif
        }
    }
    else
        ret = -1;

    return ret;
}

static INT32S find_fist_command_prcess(module_command_t *info_ptr, INT32U yuv_buf, INT32U y_buf)
{
    #define POS_X_OFFST     0
    float dist_square_thre;
    INT32S pos_temp,pos_offset;

    dist_square_thre = 13;

    if(info_ptr->find_hand == 0 && pContext_right.initial_flag == 1 && pContext_left.initial_flag == 1)
    {
#if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
        drv_fd_result_lock();
        fd_draw_result.is_hand = 1;

        fd_draw_result.rect[MOVINGROI_RIGHT_SET].x = movingROI_right.x;
        fd_draw_result.rect[MOVINGROI_RIGHT_SET].y = movingROI_right.y;
        fd_draw_result.rect[MOVINGROI_RIGHT_SET].width = movingROI_right.width;
        fd_draw_result.rect[MOVINGROI_RIGHT_SET].height = movingROI_right.height;

        fd_draw_result.rect[MOVINGROI_LEFT_SET].x = movingROI_left.x;
        fd_draw_result.rect[MOVINGROI_LEFT_SET].y = movingROI_left.y;
        fd_draw_result.rect[MOVINGROI_LEFT_SET].width = movingROI_left.width;
        fd_draw_result.rect[MOVINGROI_LEFT_SET].height = movingROI_left.height;
        drv_fd_result_unlock();
#endif
        //matching check
        gesture_moving_control((unsigned char *)yuv_buf, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL , &pContext_right, &movingROI_right, 1);
        gesture_moving_control((unsigned char *)yuv_buf, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL , &pContext_left, &movingROI_left, 1);
#if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
        //DBG_PRINT("pContext_right.dist_square = %f\r\n",pContext_right.dist_square);
        //DBG_PRINT("pContext_left.dist_square = %f\r\n",pContext_left.dist_square);
#endif
        if((pContext_right.dist_square > (dist_square_thre)) && (pContext_left.dist_square > (dist_square_thre)))
        {
            return COMMAND_HAND_FLIP_NORESULT;
        }
        else if( pContext_right.dist_square >  pContext_left.dist_square )
        {
            if(pContext_right.dist_square < dist_square_thre)
            {
                return COMMAND_HAND_FLIP_NORESULT;
            }
            return COMMAND_HAND_FLIP_RIGHT;
        }
        else if(pContext_left.dist_square >  pContext_right.dist_square)
        {
            if(pContext_left.dist_square < dist_square_thre)
            {
                return COMMAND_HAND_FLIP_NORESULT;
            }
            return COMMAND_HAND_FLIP_LEFT;
        }
        else
            return COMMAND_HAND_FLIP_NORESULT;
    }
    else
    {
        pos_offset = (info_ptr->result[HAND_NUM_SET].width>>2);
        movingROI_right.x = (info_ptr->result[HAND_NUM_SET].x + info_ptr->result[HAND_NUM_SET].width - pos_offset);
        //movingROI_right.x = (info_ptr->result[HAND_NUM_SET].x + info_ptr->result[HAND_NUM_SET].width + POS_X_OFFST);
        movingROI_right.y = info_ptr->result[HAND_NUM_SET].y + (info_ptr->result[HAND_NUM_SET].height>>1);
        movingROI_right.width = info_ptr->result[HAND_NUM_SET].width-(info_ptr->result[HAND_NUM_SET].width>>2);
        movingROI_right.height = info_ptr->result[HAND_NUM_SET].height-(info_ptr->result[HAND_NUM_SET].height>>2);
        if((movingROI_right.x + movingROI_right.width) > FD_SIZE_HPIXEL)
        {
            pos_temp = ((FD_SIZE_HPIXEL - 1) - movingROI_right.x);
            movingROI_right.width = pos_temp;
        }

        pos_temp = (info_ptr->result[HAND_NUM_SET].x - ((info_ptr->result[HAND_NUM_SET].width>>2)*2));
        //pos_temp = (info_ptr->result[HAND_NUM_SET].x - ((info_ptr->result[HAND_NUM_SET].width>>2)*3) - POS_X_OFFST);
        if(pos_temp < 0)
            movingROI_left.x = 1;
        else
            movingROI_left.x = pos_temp;
        movingROI_left.y = info_ptr->result[HAND_NUM_SET].y + (info_ptr->result[HAND_NUM_SET].height>>1);
        //movingROI_left.width = info_ptr->result[HAND_NUM_SET].width-(info_ptr->result[HAND_NUM_SET].width>>2);
        movingROI_left.width = info_ptr->result[HAND_NUM_SET].width;//-(info_ptr->result[HAND_NUM_SET].width>>2) + pos_offset;
        movingROI_left.height = info_ptr->result[HAND_NUM_SET].height-(info_ptr->result[HAND_NUM_SET].height>>2);
        if((movingROI_left.x + movingROI_left.width) > (info_ptr->result[HAND_NUM_SET].x + pos_offset))
        {
            pos_temp = ((info_ptr->result[HAND_NUM_SET].x + pos_offset) - movingROI_left.x - POS_X_OFFST);
            movingROI_left.width = pos_temp;
        }
        /*
        if((movingROI_left.x + movingROI_left.width) > info_ptr->result[HAND_NUM_SET].x)
        {
            pos_temp = (info_ptr->result[HAND_NUM_SET].x - movingROI_left.x - POS_X_OFFST);
            movingROI_left.width = pos_temp;
        }
        */
#if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
        drv_fd_result_lock();
        fd_draw_result.is_hand = 1;
        fd_draw_result.rect[HAND_NUM_SET].x = info_ptr->result[HAND_NUM_SET].x;
        fd_draw_result.rect[HAND_NUM_SET].y = info_ptr->result[HAND_NUM_SET].y;
        fd_draw_result.rect[HAND_NUM_SET].width = info_ptr->result[HAND_NUM_SET].width;
        fd_draw_result.rect[HAND_NUM_SET].height = info_ptr->result[HAND_NUM_SET].height;

        fd_draw_result.rect[MOVINGROI_RIGHT_SET].x = movingROI_right.x;
        fd_draw_result.rect[MOVINGROI_RIGHT_SET].y = movingROI_right.y;
        fd_draw_result.rect[MOVINGROI_RIGHT_SET].width = movingROI_right.width;
        fd_draw_result.rect[MOVINGROI_RIGHT_SET].height = movingROI_right.height;

        fd_draw_result.rect[MOVINGROI_LEFT_SET].x = movingROI_left.x;
        fd_draw_result.rect[MOVINGROI_LEFT_SET].y = movingROI_left.y;
        fd_draw_result.rect[MOVINGROI_LEFT_SET].width = movingROI_left.width;
        fd_draw_result.rect[MOVINGROI_LEFT_SET].height = movingROI_left.height;
        drv_fd_result_unlock();
#endif
        //model
        gesture_moving_control( (unsigned char *)yuv_buf, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL , &pContext_right, &movingROI_right, 0);
        gesture_moving_control( (unsigned char *)yuv_buf, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL , &pContext_left, &movingROI_left, 0);

        return COMMAND_HAND_FLIP_NORESULT;
    }
}

static INT32S face_hand_fist_command_prcess(module_command_t *info_ptr, INT32U yuv_buf, INT32U y_buf)
{
    #define FIND_HAND_DET_EN        1
    #define FIND_HAND_DET_CNT       4
    #define FIND_HAND_DET_HOFFSET   4
    #define FIND_HAND_DET_VOFFSET   4
    #define PRCESS_DEBUG_EN         0
    #define FIND_TURN_PAGE          0//1
    #define FIND_TURN_PAGE_CNT      1
    #define POS_OFFSET              20
    #define POS_MOVE_CNT            1//10
    #define WIDTH_OFFSET            10
    #define WIDTH_OFFSET_CNT        2//1
    #define FIST_MAX_SIZE           140
    #define HAND_MAX_SIZE           140
    #define HAND_MIN_SIZE           60
    #define HAND_POS_X_MAX_SIZE     280
    #define HAND_POS_X_MIN_SIZE     40
    #define HAND_POS_Y_MAX_SIZE     220
    #define HAND_POS_Y_MIN_SIZE     20
    #define HAND_RANGE_DET_CNT      3
    #define TURN_PAGE_CNT           20
    #define H_TEMP_OFFSET           8
    #define V_TEMP_OFFSET           8
    INT32S mode,prcess_state,temp_x,temp_y,result;
    INT32S pos_offset,pos_y_offset,width_offset,height_offset;
    INT32S center_x,center_y,ack_msg,*rang_ptr;
    INT32S width_temp_offset,height_temp_offset,temp_h,temp_v;

    drv_fd_result_lock();
    mode = info_ptr->mode;
    prcess_state = info_ptr->command_state;
    pos_offset = info_ptr->command_prcess_flag;
    ack_msg = info_ptr->ack_msg_flag;
    drv_fd_result_unlock();

    temp_x = 0;
    switch(prcess_state)
    {
        case COMMAND_TURN_UP:
        case COMMAND_TURN_DOWN:
        case COMMAND_MODULE_STOP:
        case COMMAND_MODULE_START:
        case COMMAND_MODULE_PAUSE:
        case COMMAND_MODULE_RESUME:
        case COMMAND_FUNCTION_UP:
        case COMMAND_FUNCTION_DOWN:
            temp_x = 1;
            drv_fd_result_lock();
            info_ptr->command_check_enable = 1;
#if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
            fd_draw_result.is_hand = 0;
            fd_draw_result.is_fist = 0;
#endif
            drv_fd_result_unlock();
            break;
    }

    if(temp_x)
        return -2;

    if(mode == API_COMMAND_DET_OFF)
    {
_COMMAND_OFF_EN:
        fd_draw_result.is_hand = 0;
        fd_draw_result.is_fist = 0;
        drv_fd_result_lock();
        if(info_ptr->find_face || info_ptr->find_hand || info_ptr->find_fist)
        {
            info_ptr->find_face = 0;
            info_ptr->find_hand = 0;
            info_ptr->find_fist = 0;
            info_ptr->prcess_busy = 0;
            info_ptr->prcess_find_fist_cnt = 0;
            gp_memset((INT8S *)&info_ptr->result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
            gp_memset((INT8S *)&info_ptr->prcess_result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
        }

        if(info_ptr->prcess_busy || info_ptr->prcess_find_fist_cnt)
        {
            info_ptr->prcess_busy = 0;
            info_ptr->prcess_find_fist_cnt = 0;
        }

        if(info_ptr->command_state != COMMAND_FACE_HAND_FIST_ZERO)
            info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;

        if(info_ptr->prcess_pos_cnt || info_ptr->fist_cnt || info_ptr->fist_face_cnt)
        {
            info_ptr->prcess_pos_cnt = 0;
            info_ptr->fist_cnt = 0;
            info_ptr->fist_face_cnt = 0;
        }

        if(info_ptr->hand_turn_page_on)
            info_ptr->hand_turn_page_on = 0;

        if(info_ptr->command_check_cnt)
            info_ptr->command_check_cnt = 0;

        if(info_ptr->command_check_enable)
            info_ptr->command_check_enable = 0;

        if(info_ptr->hand_find_with_fist_start)
            info_ptr->hand_find_with_fist_start = 0;

        if(info_ptr->find_fist_width_cnt)
            info_ptr->find_fist_width_cnt = 0;

        if(info_ptr->hand_with_stable_cnt)
            info_ptr->hand_with_stable_cnt = 0;

        if(info_ptr->hand_turn_page_on_cnt)
            info_ptr->hand_turn_page_on_cnt = 0;

        if(info_ptr->find_hand_mode)
            info_ptr->find_hand_mode = 0;

        gp_memset((INT8S *)&info_ptr->find_hand_range_cnt[0], 0, sizeof(INT32U)*OMMAND_FIND_HAND_MAX);

        drv_fd_result_unlock();

        if(result == API_FIND_HAND && mode == API_COMMAND_RESULT_DET)
            return -3;
        else
        return 0;
    }
    else
    {
        if(mode == API_COMMAND_FACE_DET)
            prcess_state = API_FIND_FACE;
        else if((mode == API_COMMAND_RESULT_DET) || (mode == API_COMMAND_HAND_FIST_DET))
        {
            if(info_ptr->prcess_busy == 0 || info_ptr->prcess_busy == FIND_FLIP_STATE_EN)
            {
                prcess_state = API_FIND_HAND;
                //DBG_PRINT("!1");
            }
            else
            {
                prcess_state = API_FIND_FIST;
                //DBG_PRINT("!2");
            }
        }
        else
            return -1;
    }

    if(mode == API_COMMAND_RESULT_DET)
    {
        if(prcess_state == API_FIND_HAND)
        {
            if(pos_offset == COMMAND_STATE_1)
                result = 0;
            else
                result = face_hand_fist_detect_result_get(info_ptr, prcess_state, yuv_buf, y_buf);
        }
        else
            result = face_hand_fist_detect_result_get(info_ptr, prcess_state, yuv_buf, y_buf);
    }
    else
        result = face_hand_fist_detect_result_get(info_ptr, prcess_state, yuv_buf, y_buf);

    if(result == API_FIND_HAND && mode == API_COMMAND_RESULT_DET)
        goto _COMMAND_OFF_EN;

    // check command range
    if(result && (mode == API_COMMAND_RESULT_DET) && (prcess_state == API_FIND_HAND))
    {
        drv_fd_result_lock();
        rang_ptr = (INT32S *)&info_ptr->find_hand_range_cnt[FD_HAND_UPDATE_STATE_GET];
        center_x = *rang_ptr;
        drv_fd_result_unlock();

        if(center_x)
        {
            cv_aeawb_flag_lock();
            drv_fd_result_lock();
            info_ptr->ack_msg_flag = 0;
            info_ptr->find_hand_range_cnt[FD_HAND_UPDATE_STATE_GET] = 0;
            drv_fd_result_unlock();
            #if 1//PRCESS_DEBUG_EN == 1
            DBG_PRINT("!03");
            #endif
        }
        else
        {
            drv_fd_result_lock();
            temp_x = info_ptr->result[HAND_NUM_SET].width;
            rang_ptr = (INT32S *)&info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_NEAR];
            center_x = *rang_ptr;
            drv_fd_result_unlock();
            // near
            if((temp_x > HAND_MAX_SIZE))
            {
                if(center_x > HAND_RANGE_DET_CNT)
                {
                    info_ptr->hand_with_stable_cnt = 0;
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_NEAR] = 0;
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG] = 1;
                    drv_fd_result_unlock();
                    DBG_PRINT("\r\nHand too near!\r\n");
                    goto _COMMAND_OFF_EN;
                }
                else
                {
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_NEAR] = center_x;
                    drv_fd_result_unlock();
                }
            }
            else  // far
            {
                drv_fd_result_lock();
                rang_ptr = (INT32S *)&info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_FAR];
                center_x = *rang_ptr;
                drv_fd_result_unlock();
                if((temp_x < HAND_MIN_SIZE))
                {
                    center_x++;
                    if(center_x > HAND_RANGE_DET_CNT)
                    {
                        info_ptr->hand_with_stable_cnt = 0;
                        drv_fd_result_lock();
                        info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_FAR] = 0;
                        info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG] = 1;
                        drv_fd_result_unlock();
                        DBG_PRINT("\r\nHand too far!\r\n");
                        goto _COMMAND_OFF_EN;
                    }
                    else
                    {
                        drv_fd_result_lock();
                        info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_FAR] = center_x;
                        drv_fd_result_unlock();
                    }
                }
            }

            // find left and up
            drv_fd_result_lock();
            temp_x = info_ptr->result[HAND_NUM_SET].x;
            temp_y = info_ptr->result[HAND_NUM_SET].y;
            rang_ptr = (INT32S *)&info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_X_MIN];
            center_x = *rang_ptr;
            rang_ptr = (INT32S *)&info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_Y_MIN];
            center_y = *rang_ptr;
            drv_fd_result_unlock();
            if(temp_x < HAND_POS_X_MIN_SIZE)
            {
                center_x++;
                if(center_x > HAND_RANGE_DET_CNT)
                {
                    info_ptr->hand_with_stable_cnt = 0;
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_X_MIN] = 0;
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG] = 1;
                    drv_fd_result_unlock();
                    DBG_PRINT("\r\nHand too left!\r\n");
                    goto _COMMAND_OFF_EN;
                }
                else
                {
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_X_MIN] = center_x;
                    drv_fd_result_unlock();
                }
            }
            else if(temp_y < HAND_POS_Y_MIN_SIZE)
            {
                center_y++;
                if(center_y > HAND_RANGE_DET_CNT)
                {
                    info_ptr->hand_with_stable_cnt = 0;
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_Y_MIN] = 0;
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG] = 1;
                    drv_fd_result_unlock();
                    DBG_PRINT("\r\nHand too up!\r\n");
                    goto _COMMAND_OFF_EN;
                }
                else
                {
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_Y_MIN] = center_y;
                    drv_fd_result_unlock();
                }
            }

            // find right and down
            drv_fd_result_lock();
            temp_x = (info_ptr->result[HAND_NUM_SET].x + info_ptr->result[HAND_NUM_SET].width);
            temp_y = (info_ptr->result[HAND_NUM_SET].y + info_ptr->result[HAND_NUM_SET].height);
            rang_ptr = (INT32S *)&info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_X_MAX];
            center_x = *rang_ptr;
            rang_ptr = (INT32S *)&info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_Y_MAX];
            center_y = *rang_ptr;
            drv_fd_result_unlock();
            if(temp_x > HAND_POS_X_MAX_SIZE)
            {
                center_x++;
                if(center_x > HAND_RANGE_DET_CNT)
                {
                    info_ptr->hand_with_stable_cnt = 0;
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_X_MAX] = 0;
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG] = 1;
                    drv_fd_result_unlock();
                    DBG_PRINT("\r\nHand too right!\r\n");
                    goto _COMMAND_OFF_EN;
                }
                else
                {
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_X_MAX] = center_x;
                    drv_fd_result_unlock();
                }
            }
            else if(temp_y > HAND_POS_Y_MAX_SIZE)
            {
                center_y++;
                if(center_y > HAND_RANGE_DET_CNT)
                {
                    info_ptr->hand_with_stable_cnt = 0;
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_Y_MAX] = 0;
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG] = 1;
                    drv_fd_result_unlock();
                    DBG_PRINT("\r\nHand too down!\r\n");
                    goto _COMMAND_OFF_EN;
                }
                else
                {
                    drv_fd_result_lock();
                    info_ptr->find_hand_range_cnt[COMMAND_FIND_HAND_Y_MAX] = center_y;
                    drv_fd_result_unlock();
                }
            }
        }
    }

    drv_fd_result_lock();
    prcess_state = info_ptr->prcess_busy;
    drv_fd_result_unlock();

#if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
    drv_fd_result_lock();
    if(mode != API_COMMAND_HAND_FIST_DET)
       fd_draw_result.is_hand = 0;
    fd_draw_result.is_fist = 0;
    drv_fd_result_unlock();
#endif

    if((prcess_state == 0) && (mode == API_COMMAND_RESULT_DET) || prcess_state == FIND_FLIP_STATE_EN)
    {
        if(result)
        {
            #if PRCESS_DEBUG_EN == 1
                DBG_PRINT("#");
            #endif
            result = find_fist_command_prcess(info_ptr,yuv_buf,y_buf);
        #if FIND_HAND_DET_EN == 1
            drv_fd_result_lock();
            if(info_ptr->prcess_result[HAND_NUM_SET3].x == 0)
            {
                info_ptr->prcess_result[HAND_NUM_SET3].x = info_ptr->result[HAND_NUM_SET].x;
                info_ptr->prcess_result[HAND_NUM_SET3].y = info_ptr->result[HAND_NUM_SET].y;
                info_ptr->prcess_result[HAND_NUM_SET3].width = info_ptr->result[HAND_NUM_SET].width;
                info_ptr->prcess_result[HAND_NUM_SET3].height = info_ptr->result[HAND_NUM_SET].height;
                info_ptr->command_state = COMMAND_STATE_PRCESS;
            }
            else
            {
                center_x = (info_ptr->result[HAND_NUM_SET].x + (info_ptr->result[HAND_NUM_SET].width/2));
                center_y = (info_ptr->result[HAND_NUM_SET].y + (info_ptr->result[HAND_NUM_SET].height/2));
                temp_x = (info_ptr->prcess_result[HAND_NUM_SET3].x + (info_ptr->prcess_result[HAND_NUM_SET3].width/2));
                temp_y = (info_ptr->prcess_result[HAND_NUM_SET3].y + (info_ptr->prcess_result[HAND_NUM_SET3].height/2));
                if(center_x >= temp_x)
                    pos_offset = (center_x - temp_x);
                else
                    pos_offset = (temp_x - center_x);

                if(center_y >= temp_y)
                    pos_y_offset = (center_y - temp_y);
                else
                    pos_y_offset = (temp_y - center_y);

                width_offset = (info_ptr->result[HAND_NUM_SET].width / 8);
                if(width_offset > FIND_HAND_DET_HOFFSET)
                    width_offset = FIND_HAND_DET_HOFFSET;

                height_offset = (info_ptr->result[HAND_NUM_SET].height / 8);
                if(height_offset > FIND_HAND_DET_VOFFSET)
                    height_offset = FIND_HAND_DET_VOFFSET;

                if((pos_offset < width_offset) && (pos_y_offset < height_offset))
                {
                    #if PRCESS_DEBUG_EN == 1
                        DBG_PRINT("pos_x_offset = %d\r\n",pos_offset);
                        DBG_PRINT("pos_y_offset = %d\r\n",pos_y_offset);
                        DBG_PRINT("pos_w_offset = %d\r\n",width_offset);
                        DBG_PRINT("pos_h_offset = %d\r\n",height_offset);
                    #endif
                    if(info_ptr->hand_with_stable_cnt >=  FIND_HAND_DET_CNT)
                    {
                        if(info_ptr->prcess_busy != FIND_FLIP_STATE_EN)
                        {
                            width_temp_offset = info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG];
                            if(width_temp_offset == 0)
                            {
                               info_ptr->prcess_no_find_cnt = 0;
                               info_ptr->prcess_hand_no_fist_cnt = 0;
                               info_ptr->prcess_busy = FIND_FLIP_STATE_EN;
                               cv_aeawb_flag_lock();
                               prcess_result_state_add(MSG_PRCESS_HAND, 1);
                            }
                            else
                            {
                               info_ptr->prcess_busy = 0;
                               info_ptr->hand_with_stable_cnt = 0;
                               info_ptr->find_hand_mode = 0;
                               info_ptr->find_hand_range_cnt[COMMAND_FIND_RANG_FLAG] = 0;
                               #if PRCESS_DEBUG_EN == 1
                                   DBG_PRINT("!01");
                               #endif
                            }
                        }
                    }
                    else
                    {
                        info_ptr->hand_with_stable_cnt++;
                    }
                    info_ptr->command_state = COMMAND_STATE_PRCESS;
                    #if PRCESS_DEBUG_EN == 1
                        DBG_PRINT("!1");
                    #endif
                }
                else
                {
                    info_ptr->command_state = COMMAND_STATE_PRCESS;
                    #if PRCESS_DEBUG_EN == 1
                        DBG_PRINT("!2");
                    #endif
                }
                info_ptr->prcess_result[HAND_NUM_SET3].x = info_ptr->result[HAND_NUM_SET].x;
                info_ptr->prcess_result[HAND_NUM_SET3].y = info_ptr->result[HAND_NUM_SET].y;
                info_ptr->prcess_result[HAND_NUM_SET3].width = info_ptr->result[HAND_NUM_SET].width;
                info_ptr->prcess_result[HAND_NUM_SET3].height = info_ptr->result[HAND_NUM_SET].height;
            }
            drv_fd_result_unlock();
        #else
            drv_fd_result_lock();
            if(info_ptr->prcess_busy != FIND_FLIP_STATE_EN)
                info_ptr->prcess_busy = FIND_FLIP_STATE_EN;
            //info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;
            //info_ptr->command_state = COMMAND_FIND_HAND_NO_FIST;
            if(info_ptr->fist_face_cnt < TURN_PAGE_CNT)
                info_ptr->command_state = COMMAND_STATE_PRCESS;
            else
                info_ptr->command_state = COMMAND_FIND_HAND_NO_FIST;
            info_ptr->fist_face_cnt++;
            info_ptr->hand_turn_page_on = 0;
            drv_fd_result_unlock();
        #endif
        }
        else if(prcess_state == FIND_FLIP_STATE_EN)
        {
            drv_fd_result_lock();
            if(result == 0 && (info_ptr->hand_turn_page_on_cnt <= FIND_TURN_PAGE_CNT))
            {
                #if PRCESS_DEBUG_EN == 1
                    DBG_PRINT("#0");
                #endif
                info_ptr->hand_turn_page_on = 1;
                info_ptr->hand_with_stable_cnt = 0;
            }
            else
                info_ptr->hand_turn_page_on_cnt++;
            drv_fd_result_unlock();

            #if PRCESS_DEBUG_EN == 1
                DBG_PRINT("#0");
            #endif

            drv_fd_result_lock();
            ack_msg = info_ptr->command_prcess_flag;
            drv_fd_result_unlock();

            if(ack_msg == COMMAND_STATE_0)
            {
                ack_msg = prcess_result_state_get(0);
                drv_fd_result_lock();
                if(ack_msg == MSG_PRCESS_END)
                {
                    //DBG_PRINT("!3");
                    info_ptr->prcess_no_find_cnt = 0;
                    info_ptr->prcess_hand_no_fist_cnt = 0;
                    info_ptr->command_prcess_flag = COMMAND_STATE_1;
                }
                drv_fd_result_unlock();
            }

            drv_fd_result_lock();
            result = info_ptr->hand_turn_page_on;
            ack_msg = info_ptr->command_prcess_flag;
            drv_fd_result_unlock();

            if(result && (ack_msg != COMMAND_STATE_0))
            {
                #if PRCESS_DEBUG_EN == 1
                    DBG_PRINT("#1");
                #endif
                result = find_fist_command_prcess(info_ptr,yuv_buf,y_buf);
                if(result)
                {
                    drv_fd_result_lock();
                    info_ptr->fist_face_cnt = 0;
                    info_ptr->prcess_find_fist_cnt = 0;
                    if(result == COMMAND_HAND_FLIP_RIGHT)
                        info_ptr->command_state = COMMAND_TURN_UP;
                    else
                        info_ptr->command_state = COMMAND_TURN_DOWN;
                    info_ptr->hand_turn_page_on = 0;
                    drv_fd_result_unlock();
                }
                //else if(mode == 0)
                else
                {
                    result = face_hand_fist_detect_result_get(info_ptr, API_FIND_FIST, yuv_buf, y_buf);
                    drv_fd_result_lock();
                    if(result)
                    {
                        info_ptr->prcess_find_fist_cnt++;
                        if(info_ptr->prcess_find_fist_cnt > FIND_TURN_PAGE)
                        {
                            info_ptr->prcess_busy = FIND_FIST_STATE_EN;
                            info_ptr->hand_turn_page_on = 0;
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("Find Fist Enable 1\r\n");
                        #endif
                        }
                        info_ptr->command_state = COMMAND_STATE_PRCESS;
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("#2");
                        #endif
                    }
                    else
                    {
                        info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("#1");
                        #endif
                    }
                    drv_fd_result_unlock();
                }
            }
            else
            {
                drv_fd_result_lock();
                ack_msg = info_ptr->command_prcess_flag;
                drv_fd_result_unlock();
                if(ack_msg == COMMAND_STATE_0)
                    info_ptr->command_state = COMMAND_STATE_PRCESS;
                else
                    info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;
            }
        }
        else
        {
            drv_fd_result_lock();
            info_ptr->hand_with_stable_cnt = 0;
            if(info_ptr->command_prcess_flag == COMMAND_STATE_0)
            {
                info_ptr->command_state = COMMAND_STATE_PRCESS;
                #if PRCESS_DEBUG_EN == 1
                    DBG_PRINT("#2");
                #endif
            }
            else
                info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;
            #if PRCESS_DEBUG_EN == 1
                DBG_PRINT("!3");
            #endif
            drv_fd_result_unlock();
        }

        //reset fist tracking inital flag
        if(prcess_state != FIND_FLIP_STATE_EN)
        {
              pContext_right.initial_flag = 0;
              pContext_left.initial_flag = 0;
        }
    }
    else if(result && (mode == API_COMMAND_HAND_FIST_DET) && (prcess_state == 0))
    {
        drv_fd_result_lock();
        info_ptr->fist_face_cnt = 0;
        info_ptr->hand_turn_page_on = 0;
        info_ptr->prcess_find_fist_cnt++;
        #if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
            fd_draw_result.is_hand = result;
            fd_draw_result.rect[HAND_NUM_SET].x = info_ptr->result[HAND_NUM_SET].x;
            fd_draw_result.rect[HAND_NUM_SET].y = info_ptr->result[HAND_NUM_SET].y;
            fd_draw_result.rect[HAND_NUM_SET].width = info_ptr->result[HAND_NUM_SET].width;
            fd_draw_result.rect[HAND_NUM_SET].height = info_ptr->result[HAND_NUM_SET].height;
        #endif
        if(info_ptr->prcess_find_fist_cnt > FIND_TURN_PAGE)
        {
            info_ptr->prcess_busy = FIND_FIST_STATE_EN;
            info_ptr->command_prcess_flag = COMMAND_STATE_1;
        #if PRCESS_DEBUG_EN == 1
            DBG_PRINT("Find Fist Enable 2\r\n");
        #endif
        }
        info_ptr->command_state = COMMAND_STATE_PRCESS;
        drv_fd_result_unlock();

        drv_fd_result_lock();
        ack_msg = info_ptr->prcess_busy;
        drv_fd_result_unlock();
        if(ack_msg == FIND_FIST_STATE_EN)
            prcess_result_state_add(MSG_PRCESS_HAND, 1);
    }
    else
    {
        drv_fd_result_lock();
        info_ptr->fist_face_cnt = 0;
        info_ptr->hand_turn_page_on = 0;
        drv_fd_result_unlock();
        if(result)
        {
            if(mode == API_COMMAND_FACE_DET)
            {
                drv_fd_result_lock();
                info_ptr->command_prcess_flag = COMMAND_STATE_1;
                if(info_ptr->find_face)
                    info_ptr->command_state = COMMAND_FIND_FACE_AND_TRACK;
                else
                    info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;
                drv_fd_result_unlock();
            }
            else
            {
            #if FD_COMMAND_FLOW_DEBUG_DRAW_EN == 1
                drv_fd_result_lock();
                fd_draw_result.is_hand = 0;
                fd_draw_result.is_fist = 1;
                fd_draw_result.rect[FIST_NUM_SET].x = info_ptr->result[FIST_NUM_SET].x;
                fd_draw_result.rect[FIST_NUM_SET].y = info_ptr->result[FIST_NUM_SET].y;
                fd_draw_result.rect[FIST_NUM_SET].width = info_ptr->result[FIST_NUM_SET].width;
                fd_draw_result.rect[FIST_NUM_SET].height = info_ptr->result[FIST_NUM_SET].height;
                drv_fd_result_unlock();
            #endif

                drv_fd_result_lock();
                ack_msg = info_ptr->command_prcess_flag;
                drv_fd_result_unlock();
                if(ack_msg == COMMAND_STATE_1 && (mode == API_COMMAND_HAND_FIST_DET))
                {
                    ack_msg = prcess_result_state_get(0);
                    drv_fd_result_lock();
                    if(ack_msg == MSG_PRCESS_END)
                    {
                        //DBG_PRINT("!3");
                        info_ptr->prcess_no_find_cnt = 0;
                        info_ptr->prcess_hand_no_fist_cnt = 0;
                        info_ptr->command_prcess_flag = COMMAND_STATE_2;
                        ack_msg = info_ptr->command_prcess_flag;
                    }
                    drv_fd_result_unlock();
                }

                if(prcess_state == FIND_FIST_STATE_EN)
                {
                    drv_fd_result_lock();
                    if(mode == API_COMMAND_HAND_FIST_DET)
                    {
                        if(ack_msg == COMMAND_STATE_2)
                            info_ptr->command_state = COMMAND_FIND_FIST_AND_TRACK;
                        else
                            info_ptr->command_state = COMMAND_STATE_PRCESS;
                    }
                    else if(info_ptr->result[FIST_NUM_SET].width > FIST_MAX_SIZE)
                    {
                        DBG_PRINT("too near !! no cammand, FIST.width = %d\r\n",info_ptr->result[FIST_NUM_SET].width);
                        info_ptr->command_state = COMMAND_FIND_HAND_NO_FIST;
                    }
                    else
                    {
                        if((info_ptr->prcess_result[FIST_NUM_SET].width == 0))
                        {
                            // save prcess_result for find first fist.
                            //DBG_PRINT("kobe562021");
                            info_ptr->prcess_result[FIST_NUM_SET].x = info_ptr->result[FIST_NUM_SET].x;
                            info_ptr->prcess_result[FIST_NUM_SET].y = info_ptr->result[FIST_NUM_SET].y;
                            info_ptr->prcess_result[FIST_NUM_SET].width = info_ptr->result[FIST_NUM_SET].width;
                            info_ptr->prcess_result[FIST_NUM_SET].height = info_ptr->result[FIST_NUM_SET].height;
                        }

                        // find fist x position > WIDTH_OFFSET
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("WIDTH_OFFSET 1 \r\n");
                        #endif

                        if(info_ptr->prcess_result[FIST_NUM_SET].x >= info_ptr->result[FIST_NUM_SET].x)
                            temp_x = (info_ptr->prcess_result[FIST_NUM_SET].x - info_ptr->result[FIST_NUM_SET].x);
                        else
                            temp_x = (info_ptr->result[FIST_NUM_SET].x - info_ptr->prcess_result[FIST_NUM_SET].x);

                        if(temp_x < 0)
                            temp_x = 0;
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("temp_x 1= %d \r\n",temp_x);
                        #endif

                        if(info_ptr->prcess_result[FIST_NUM_SET].y >= info_ptr->result[FIST_NUM_SET].y)
                            temp_y = (info_ptr->prcess_result[FIST_NUM_SET].y - info_ptr->result[FIST_NUM_SET].y);
                        else
                            temp_y = (info_ptr->result[FIST_NUM_SET].y - info_ptr->prcess_result[FIST_NUM_SET].y);

                        if(temp_y < 0)
                            temp_y = 0;
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("temp_y 1= %d \r\n",temp_y);
                        #endif

                        pos_offset = (info_ptr->prcess_result[FIST_NUM_SET].width / 2);
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("pos_offset 1= %d \r\n",pos_offset);
                        #endif
                        if(pos_offset == 0)
                            pos_offset = POS_OFFSET;

                        pos_y_offset = (info_ptr->prcess_result[FIST_NUM_SET].height / 2);
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("pos_y_offset = %d \r\n",pos_y_offset);
                        #endif
                        if(pos_y_offset == 0)
                            pos_y_offset = POS_OFFSET;

                        if(info_ptr->prcess_result[FIST_NUM_SET].width >= info_ptr->result[FIST_NUM_SET].width)
                            width_temp_offset = (info_ptr->prcess_result[FIST_NUM_SET].width - info_ptr->result[FIST_NUM_SET].width);
                        else
                            width_temp_offset = (info_ptr->result[FIST_NUM_SET].width - info_ptr->prcess_result[FIST_NUM_SET].width);

                        if(width_temp_offset < 0)
                            width_temp_offset = 0;
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("width_temp_offset 1= %d \r\n",width_temp_offset);
                        #endif

                        if(info_ptr->prcess_result[FIST_NUM_SET].height >= info_ptr->result[FIST_NUM_SET].height)
                            height_temp_offset = (info_ptr->prcess_result[FIST_NUM_SET].height - info_ptr->result[FIST_NUM_SET].height);
                        else
                            height_temp_offset = (info_ptr->result[FIST_NUM_SET].height - info_ptr->prcess_result[FIST_NUM_SET].height);

                        if(height_temp_offset < 0)
                            height_temp_offset = 0;
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("height_temp_offset 1= %d \r\n",height_temp_offset);
                        #endif

                        temp_h = (info_ptr->prcess_result[FIST_NUM_SET].width / 4);
                        if(temp_h > H_TEMP_OFFSET)
                            temp_h = H_TEMP_OFFSET;
                        else if(temp_h < 0)
                            temp_h = (H_TEMP_OFFSET / 2);
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("temp_h = %d \r\n",temp_h);
                        #endif

                        temp_v = (info_ptr->prcess_result[FIST_NUM_SET].height / 4);
                        if(temp_v > V_TEMP_OFFSET)
                            temp_v = V_TEMP_OFFSET;
                        else if(temp_v < 0)
                            temp_v = (V_TEMP_OFFSET / 2);
                        #if PRCESS_DEBUG_EN == 1
                            DBG_PRINT("temp_v = %d \r\n",temp_v);
                        #endif

                        // find x and y pos in the define range
                        if((temp_x < pos_offset) && (temp_y < pos_y_offset) && (info_ptr->prcess_pos_cnt < POS_MOVE_CNT) && (info_ptr->command_prcess_flag != COMMAND_STATE_4))
                        {
                            #if PRCESS_DEBUG_EN == 1
                                DBG_PRINT("WIDTH_OFFSET 1.1 \r\n");
                            #endif
                            info_ptr->prcess_pos_cnt = 0;
                            info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;

                            // find fist width > WIDTH_OFFSET
                            if(info_ptr->prcess_result[FIST_NUM_SET].width < info_ptr->result[FIST_NUM_SET].width)
                            {
                                // COMMAND_MODULE_RESUME
                                info_ptr->command_prcess_flag = COMMAND_STATE_2;
                                temp_x = (info_ptr->result[FIST_NUM_SET].width - info_ptr->prcess_result[FIST_NUM_SET].width);
                                if(temp_x < 0)
                                    temp_x = 0;

                                width_offset = (info_ptr->prcess_result[FIST_NUM_SET].width / 5);
                                if(width_offset == 0)
                                    width_offset = WIDTH_OFFSET;

                                #if PRCESS_DEBUG_EN == 1
                                    DBG_PRINT("width_offset 1.1 = %d \r\n",width_offset);
                                    DBG_PRINT("temp_x 1.1 = %d \r\n",temp_x);
                                #endif

                                if(temp_x > width_offset)
                                {
                                    #if PRCESS_DEBUG_EN == 1
                                        DBG_PRINT("WIDTH_OFFSET 1.1.1 \r\n");
                                    #endif
                                    info_ptr->command_prcess_flag = COMMAND_STATE_3;
                                    info_ptr->find_fist_width_cnt++;
                                    if(info_ptr->find_fist_width_cnt <= WIDTH_OFFSET_CNT)
                                    {
                                        info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                                    }
                                    else
                                    {
                                        if(info_ptr->audio_pause)
                                        {
                                            info_ptr->audio_pause = 0;
                                            info_ptr->command_state = COMMAND_MODULE_RESUME;
                                        }
                                        else
                                        {
                                            info_ptr->audio_pause = 1;
                                            info_ptr->command_state = COMMAND_MODULE_PAUSE;
                                        }
                                    }
                                }
                            }
                            else
                                info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                        }
                        // find y pos over in the define range
                        else if((temp_x < pos_offset) && (temp_y > pos_y_offset) && (width_temp_offset < temp_h))
                        {
                            #if PRCESS_DEBUG_EN == 1
                                DBG_PRINT("temp_y 2\r\n");
                            #endif
                            info_ptr->command_state = COMMAND_FIND_HAND_NO_FIST;
                        }
                        // find x and y pos with width over in the define range
                        else if((temp_x > pos_offset) && (temp_y > pos_y_offset) && (width_temp_offset > temp_h))
                        {
                            #if PRCESS_DEBUG_EN == 1
                                DBG_PRINT("temp_y 3\r\n");
                            #endif
                            info_ptr->prcess_pos_cnt = 0;
                            info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                            // find fist width > WIDTH_OFFSET
                            //if(info_ptr->prcess_result[FIST_NUM_SET].width < info_ptr->result[FIST_NUM_SET].width)
                            //{
                                // COMMAND_MODULE_RESUME
                                info_ptr->command_prcess_flag = COMMAND_STATE_2;

                                #if 0
                                    if(info_ptr->prcess_result[FIST_NUM_SET].width >= info_ptr->result[FIST_NUM_SET].width)
                                        temp_x = (info_ptr->prcess_result[FIST_NUM_SET].width - info_ptr->result[FIST_NUM_SET].width);
                                    else
                                        temp_x = (info_ptr->result[FIST_NUM_SET].width - info_ptr->prcess_result[FIST_NUM_SET].width);
                                #else
                                    temp_x = (info_ptr->result[FIST_NUM_SET].width - info_ptr->prcess_result[FIST_NUM_SET].width);
                                #endif

                                if(temp_x < 0)
                                    temp_x = 0;

                                width_offset = (info_ptr->prcess_result[FIST_NUM_SET].width / 5);
                                if(width_offset == 0)
                                    width_offset = WIDTH_OFFSET;

                                #if PRCESS_DEBUG_EN == 1
                                    DBG_PRINT("width_offset 1.1 = %d \r\n",width_offset);
                                    DBG_PRINT("temp_x 1.1 = %d \r\n",temp_x);
                                #endif

                                if(temp_x > width_offset)
                                {
                                    #if PRCESS_DEBUG_EN == 1
                                        DBG_PRINT("WIDTH_OFFSET 1.1.1 \r\n");
                                    #endif
                                    info_ptr->command_prcess_flag = COMMAND_STATE_3;
                                    info_ptr->find_fist_width_cnt++;
                                    if(info_ptr->find_fist_width_cnt <= WIDTH_OFFSET_CNT)
                                    {
                                        info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                                    }
                                    else
                                    {
                                        if(info_ptr->audio_pause)
                                        {
                                            info_ptr->audio_pause = 0;
                                            info_ptr->command_state = COMMAND_MODULE_RESUME;
                                        }
                                        else
                                        {
                                            info_ptr->audio_pause = 1;
                                            info_ptr->command_state = COMMAND_MODULE_PAUSE;
                                        }
                                    }
                                }
                            //}
                            //else
                                //info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                        }
                        else
                        {
                            if((temp_x < pos_offset) && (info_ptr->prcess_pos_cnt < POS_MOVE_CNT) && (info_ptr->command_prcess_flag != COMMAND_STATE_4))
                            {
                                #if PRCESS_DEBUG_EN == 1
                                    DBG_PRINT("WIDTH_OFFSET 1.2 \r\n");
                                #endif
                                info_ptr->prcess_pos_cnt = 0;
                                info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;

                                // find fist width > WIDTH_OFFSET
                                if(info_ptr->prcess_result[FIST_NUM_SET].width < info_ptr->result[FIST_NUM_SET].width)
                                {
                                    info_ptr->command_prcess_flag = COMMAND_STATE_2;
                                    // COMMAND_MODULE_RESUME
                                    temp_x = (info_ptr->result[FIST_NUM_SET].width - info_ptr->prcess_result[FIST_NUM_SET].width);
                                    if(temp_x < 0)
                                        temp_x = 0;
                                    width_offset = (info_ptr->prcess_result[FIST_NUM_SET].width / 5);
                                    if(width_offset == 0)
                                        width_offset = WIDTH_OFFSET;

                                    #if PRCESS_DEBUG_EN == 1
                                        DBG_PRINT("width_offset 1.2 = %d \r\n",width_offset);
                                        DBG_PRINT("temp_x 1.2 = %d \r\n",temp_x);
                                    #endif

                                    if(temp_x > width_offset)
                                    {
                                        #if PRCESS_DEBUG_EN == 1
                                            DBG_PRINT("WIDTH_OFFSET 1.2.1 \r\n");
                                        #endif
                                        info_ptr->command_prcess_flag = COMMAND_STATE_3;
                                        info_ptr->find_fist_width_cnt++;
                                        if(info_ptr->find_fist_width_cnt <= WIDTH_OFFSET_CNT)
                                        {
                                            info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                                        }
                                        else
                                        {
                                            if(info_ptr->audio_pause)
                                            {
                                                info_ptr->audio_pause = 0;
                                                info_ptr->command_state = COMMAND_MODULE_RESUME;
                                            }
                                            else
                                            {
                                                info_ptr->audio_pause = 1;
                                                info_ptr->command_state = COMMAND_MODULE_PAUSE;
                                            }
                                        }
                                    }
                                }
                                else
                                    info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                            }
                            else if((temp_x > pos_offset) && (info_ptr->prcess_pos_cnt < POS_MOVE_CNT) && (info_ptr->command_prcess_flag != COMMAND_STATE_4))
                            {
                                // check fist x position > POS_OFFSET cnt
                                // COMMAND_FUNCTION_UP
                                #if PRCESS_DEBUG_EN == 1
                                    DBG_PRINT("WIDTH_OFFSET 1.3 \r\n");
                                #endif
                                info_ptr->prcess_pos_cnt++;
                                if(info_ptr->prcess_pos_cnt >= POS_MOVE_CNT)
                                    info_ptr->command_prcess_flag = COMMAND_STATE_4;
                                info_ptr->command_state = COMMAND_FIND_HAND_WITH_FIST;
                            }
                            else
                            {
                                #if PRCESS_DEBUG_EN == 1
                                    DBG_PRINT("info_ptr->prcess_result[FIST_NUM_SET].x = %d \r\n", info_ptr->prcess_result[FIST_NUM_SET].x);
                                    DBG_PRINT("info_ptr->result[FIST_NUM_SET].x = %d \r\n", info_ptr->result[FIST_NUM_SET].x);
                                #endif
                                if(info_ptr->prcess_result[FIST_NUM_SET].x >= info_ptr->result[FIST_NUM_SET].x)
                                {
                                    info_ptr->command_state = COMMAND_FUNCTION_UP;
                                    #if PRCESS_DEBUG_EN == 1
                                        DBG_PRINT("WIDTH_OFFSET 1.3.1 \r\n");
                                    #endif
                                }
                                else if(info_ptr->prcess_result[FIST_NUM_SET].x < info_ptr->result[FIST_NUM_SET].x)
                                {
                                    info_ptr->command_state = COMMAND_FUNCTION_DOWN;
                                    #if PRCESS_DEBUG_EN == 1
                                        DBG_PRINT("WIDTH_OFFSET 1.3.2 \r\n");
                                    #endif
                                }
                                else
                                {
                                    info_ptr->command_state = COMMAND_FIND_HAND_NO_FIST;
                                    #if PRCESS_DEBUG_EN == 1
                                        DBG_PRINT("WIDTH_OFFSET 1.3.3 \r\n");
                                    #endif
                                }
                            }
                        }
                    }
                    drv_fd_result_unlock();
                }
                else
                {
                    drv_fd_result_lock();
                    info_ptr->command_state = COMMAND_FIND_HAND_NO_FIST;
                    drv_fd_result_unlock();
                }
            }
        }
        else
        {
            drv_fd_result_lock();
            if(mode == API_COMMAND_FACE_DET)
                info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;
            else
                info_ptr->command_state = COMMAND_FIND_HAND_NO_FIST;
            drv_fd_result_unlock();
        }
    }
    //DBG_PRINT("!4");
    return info_ptr->command_state;
}

static INT32S command_result_check(void)
{
    #define COMMAND_FIND_HAND_FIST_CNT               50//60
    #define COMMAND_FIND_HAND_NO_FIST_CNT            30//60
    #define COMMAND_NO_FIND_FACE_HAND_FIST_CNT       20//60
    INT32U result_state, time_out_state;

    drv_fd_result_lock();

    // 1.time out flow
    if((module_mem_set->mode != API_COMMAND_DET_OFF) && (module_mem_set->command_state == COMMAND_FACE_HAND_FIST_ZERO))
    {
        // have command but not detect face/hand/fist.
        time_out_state = 1;
        //DBG_PRINT("@");
    }
    else if((module_mem_set->mode == API_COMMAND_HAND_FIST_DET) && (module_mem_set->command_state == COMMAND_FIND_HAND_NO_FIST))
    {
        // API_COMMAND_HAND_FIST_DET have hand but not detect fist.
        time_out_state = 2;
        //DBG_PRINT("&");
    }
    else if((module_mem_set->mode == API_COMMAND_RESULT_DET) && (module_mem_set->command_state == COMMAND_FIND_HAND_NO_FIST))
    {
        // API_COMMAND_RESULT_DET have hand but not detect fist.
        time_out_state = 3;
        //DBG_PRINT("*");
    }
    else
        time_out_state = 0;

    if(module_mem_set->command_prcess_flag == COMMAND_STATE_0)
    {
        if(module_mem_set->command_state != COMMAND_STATE_PRCESS)
        {
            module_mem_set->command_state = COMMAND_STATE_PRCESS;
            //DBG_PRINT("*");
        }
    }

    switch(time_out_state)
    {
        case 0:
            module_mem_set->prcess_no_find_cnt = 0;
            module_mem_set->prcess_hand_no_fist_cnt = 0;
            break;

        case 1:
        case 2:
            if(module_mem_set->prcess_no_find_cnt > COMMAND_NO_FIND_FACE_HAND_FIST_CNT)
            {
            #if COMMAND_DEBUG_EN == 1
                module_mem_set->mode = API_COMMAND_DET_OFF;
                module_mem_set->prcess_no_find_cnt = 0;
                module_mem_set->command_state = COMMAND_FACE_HAND_FIST_ZERO;
                DBG_PRINT("command_result_check: API_COMMAND_DET_OFF\r\n");
            #else
                module_mem_set->command_state = COMMAND_FACE_HAND_FIST_ZERO;
                module_mem_set->prcess_hand_no_fist_cnt = 0;
                module_mem_set->prcess_no_find_cnt = 0;
                module_mem_set->prcess_result_cnt = 0;
                module_mem_set->find_face = 0;
                module_mem_set->find_hand = 0;
                module_mem_set->find_fist = 0;
                module_mem_set->prcess_busy = 0;
                module_mem_set->prcess_find_fist_cnt = 0;
                module_mem_set->prcess_pos_cnt = 0;
                module_mem_set->fist_cnt = 0;
                module_mem_set->fist_face_cnt = 0;
                module_mem_set->hand_turn_page_on = 0;
                module_mem_set->command_check_cnt = 0;
                module_mem_set->command_check_enable = 0;
                module_mem_set->hand_find_with_fist_start = 0;
                module_mem_set->hand_with_stable_cnt = 0;
                module_mem_set->hand_turn_page_on_cnt = 0;
                module_mem_set->find_hand_mode = 0;
                gp_memset((INT8S *)&module_mem_set->result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
                gp_memset((INT8S *)&module_mem_set->prcess_result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
                gp_memset((INT8S *)&module_mem_set->find_hand_range_cnt[0], 0, sizeof(INT32U)*OMMAND_FIND_HAND_MAX);
                fd_draw_result.is_hand = 0;
                if(time_out_state == 1)
                {
                    if(module_mem_set->command_prcess_flag != COMMAND_STATE_0)
                    {
                        module_mem_set->command_prcess_flag = COMMAND_STATE_0;
                    }
                }
            #endif
                //DBG_PRINT("aeawb open 2\r\n");
                //cv_aeawb_open_flag = 1;
                cv_aeawb_flag_unlock();
                OSQFlush(prcess_result_end);
                //DBG_PRINT("command_result_check!!! \r\n");
                //DBG_PRINT("Time Out: Not Find/Hand/Fist Face \r\n");
                //DBG_PRINT("E11");
            }
            else
            {
                module_mem_set->prcess_no_find_cnt++;
                if(module_mem_set->prcess_no_find_cnt > (COMMAND_NO_FIND_FACE_HAND_FIST_CNT / 8))
                {
                    if(module_mem_set->hand_find_with_fist_start == 0)
                    {
                        //DBG_PRINT("T12");
                        module_mem_set->hand_find_with_fist_start = 1;
                    }
                }
                //else
                    //module_mem_set->hand_find_with_fist_start = 0;
            }
            break;

        case 3:
            if(module_mem_set->prcess_hand_no_fist_cnt > COMMAND_FIND_HAND_NO_FIST_CNT)
            {
            #if COMMAND_DEBUG_EN == 1
                module_mem_set->mode = API_COMMAND_DET_OFF;
                module_mem_set->prcess_hand_no_fist_cnt = 0;
                module_mem_set->command_state = COMMAND_FACE_HAND_FIST_ZERO;
                DBG_PRINT("command_result_check: API_COMMAND_DET_OFF\r\n");
            #else
                module_mem_set->command_state = COMMAND_FACE_HAND_FIST_ZERO;
                module_mem_set->prcess_hand_no_fist_cnt = 0;
                module_mem_set->prcess_no_find_cnt = 0;
                module_mem_set->prcess_result_cnt = 0;
                module_mem_set->find_face = 0;
                module_mem_set->find_hand = 0;
                module_mem_set->find_fist = 0;
                module_mem_set->prcess_busy = 0;
                module_mem_set->prcess_find_fist_cnt = 0;
                module_mem_set->prcess_pos_cnt = 0;
                module_mem_set->fist_cnt = 0;
                module_mem_set->fist_face_cnt = 0;
                module_mem_set->hand_turn_page_on = 0;
                module_mem_set->command_check_cnt = 0;
                module_mem_set->command_check_enable = 0;
                module_mem_set->hand_find_with_fist_start = 0;
                module_mem_set->hand_with_stable_cnt = 0;
                module_mem_set->hand_turn_page_on_cnt = 0;
                module_mem_set->find_hand_mode = 0;
                //hand_tts_flag = 0;
                gp_memset((INT8S *)&module_mem_set->result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
                gp_memset((INT8S *)&module_mem_set->prcess_result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
                gp_memset((INT8S *)&module_mem_set->find_hand_range_cnt[0], 0, sizeof(INT32U)*OMMAND_FIND_HAND_MAX);
            #endif
                cv_aeawb_flag_unlock();
                OSQFlush(prcess_result_end);
                if(module_mem_set->command_prcess_flag != COMMAND_STATE_0)
                {
                    module_mem_set->command_prcess_flag = COMMAND_STATE_0;
                    //DBG_PRINT("E12.1");
                }
                //DBG_PRINT("command_result_check!!! \r\n");
                //DBG_PRINT("Time Out: Not Find/Fist Face \r\n");
               //DBG_PRINT("E12");
            }
            else
            {
                module_mem_set->prcess_hand_no_fist_cnt++;
                if(module_mem_set->prcess_hand_no_fist_cnt > (COMMAND_FIND_HAND_NO_FIST_CNT / 8))
                {
                    if(module_mem_set->hand_find_with_fist_start == 0)
                    {
                        //DBG_PRINT("T3");
                        module_mem_set->hand_find_with_fist_start = 1;
                    }
                }
                //else
                    //module_mem_set->hand_find_with_fist_start = 0;
            }

            break;
    }

    //2. result detect
    if((module_mem_set->mode == API_COMMAND_RESULT_DET) && (module_mem_set->command_state == COMMAND_FIND_HAND_WITH_FIST))
    {
        // audio on/off check
        if(module_mem_set->prcess_result_cnt > COMMAND_FIND_HAND_FIST_CNT)
        {
            if(module_mem_set->audio_en)
            {
                module_mem_set->command_state = COMMAND_MODULE_STOP;
                module_mem_set->audio_en = 0;
            }
            else
            {
                module_mem_set->command_state = COMMAND_MODULE_START;
                module_mem_set->audio_en = 1;
            }
            module_mem_set->command_ok = module_mem_set->command_state;
        #if COMMAND_DEBUG_EN == 1
            module_mem_set->prcess_no_find_cnt = 0;
            module_mem_set->prcess_result_cnt = 0;
        #else
            module_mem_set->prcess_hand_no_fist_cnt = 0;
            module_mem_set->prcess_no_find_cnt = 0;
            module_mem_set->prcess_result_cnt = 0;
            module_mem_set->find_face = 0;
            module_mem_set->find_hand = 0;
            module_mem_set->find_fist = 0;
            module_mem_set->prcess_busy = 0;
            module_mem_set->prcess_find_fist_cnt = 0;
            module_mem_set->prcess_pos_cnt = 0;
            module_mem_set->fist_cnt = 0;
            module_mem_set->hand_turn_page_on = 0;
            module_mem_set->command_check_cnt = 0;
            module_mem_set->command_check_enable = 0;
            module_mem_set->hand_find_with_fist_start = 0;
            module_mem_set->hand_with_stable_cnt = 0;
            module_mem_set->hand_turn_page_on_cnt = 0;
            module_mem_set->find_hand_mode = 0;
            //hand_tts_flag = 0;
            gp_memset((INT8S *)&module_mem_set->result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
            gp_memset((INT8S *)&module_mem_set->prcess_result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
            gp_memset((INT8S *)&module_mem_set->find_hand_range_cnt[0], 0, sizeof(INT32U)*OMMAND_FIND_HAND_MAX);
        #endif
            cv_aeawb_flag_unlock();
            OSQFlush(prcess_result_end);
            //DBG_PRINT("command_result_check!!! \r\n");
            //DBG_PRINT("API_COMMAND_RESULT_DET OK \r\n");
            //DBG_PRINT("E2");
        }
        else
        {
            module_mem_set->prcess_result_cnt++;
            if(module_mem_set->prcess_result_cnt > (COMMAND_FIND_HAND_FIST_CNT / 4))
            {
                if(module_mem_set->hand_find_with_fist_start == 0)
                {
                    //DBG_PRINT("G1");
                    module_mem_set->hand_find_with_fist_start = 1;
                }
            }
            //else
                //module_mem_set->hand_find_with_fist_start = 0;
        }
    }
    else
        module_mem_set->prcess_result_cnt = 0;

    drv_fd_result_unlock();

    return 0;
}

INT32U face_hand_fist_command_result_set(module_command_t *info_ptr, INT32U mode)
{
    drv_fd_result_lock();
    info_ptr->mode = mode;
    drv_fd_result_unlock();

    return 0;
}

INT32U face_hand_fist_command_result_get(module_command_t *info_ptr, INT32U mode, gpRect *track_result)
{
    INT32U state;

    drv_fd_result_lock();
    if(mode == API_COMMAND_FACE_DET)
    {
        state = info_ptr->command_state;
        if(state == COMMAND_FIND_FACE_AND_TRACK)
        {
            if(track_result)
            {
                track_result->x = info_ptr->result[FACE_NUM_SET].x;
                track_result->y = info_ptr->result[FACE_NUM_SET].y;
                track_result->width = info_ptr->result[FACE_NUM_SET].width;
                track_result->height = info_ptr->result[FACE_NUM_SET].height;
            }
        }
    }
    else if(mode == API_COMMAND_HAND_FIST_DET)
    {
        state = info_ptr->command_state;
        //if((state == COMMAND_FIND_FIST_AND_TRACK) || (state == COMMAND_FUNCTION_UP) || (state == COMMAND_FUNCTION_DOWN))
        if((state == COMMAND_FIND_FIST_AND_TRACK))
        {
            if(track_result)
            {
                track_result->x = info_ptr->result[FIST_NUM_SET].x;
                track_result->y = info_ptr->result[FIST_NUM_SET].y;
                track_result->width = info_ptr->result[FIST_NUM_SET].width;
                track_result->height = info_ptr->result[FIST_NUM_SET].height;
            }
        }
    }
    else
    {
        state = info_ptr->command_ok;
        if((state != COMMAND_MODULE_STOP) && (state != COMMAND_MODULE_START))
            state = info_ptr->command_state;
    }
    switch(state)
    {
        case COMMAND_TURN_UP:
        case COMMAND_TURN_DOWN:
        case COMMAND_MODULE_STOP:
        case COMMAND_MODULE_START:
        case COMMAND_MODULE_PAUSE:
        case COMMAND_MODULE_RESUME:
        case COMMAND_FUNCTION_UP:
        case COMMAND_FUNCTION_DOWN:
#if COMMAND_DEBUG_EN == 1
            info_ptr->mode = API_COMMAND_DET_OFF;
            DBG_PRINT("face_hand_fist_command_result_get: API_COMMAND_DET_OFF\r\n");
#else
            info_ptr->find_face = 0;
            info_ptr->find_hand = 0;
            info_ptr->find_fist = 0;
            info_ptr->prcess_busy = 0;
            info_ptr->prcess_find_fist_cnt = 0;
            info_ptr->prcess_pos_cnt = 0;
            info_ptr->fist_cnt = 0;
            info_ptr->command_check_cnt = 0;
            info_ptr->command_check_enable = 0;
            info_ptr->hand_find_with_fist_start = 0;
            info_ptr->hand_with_stable_cnt = 0;
            info_ptr->hand_turn_page_on_cnt = 0;
            info_ptr->command_prcess_flag = COMMAND_STATE_0;
            info_ptr->find_hand_mode = 0;
            gp_memset((INT8S *)&info_ptr->result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
            gp_memset((INT8S *)&info_ptr->prcess_result[0], 0, sizeof(gpRect)*DET_MAX_NUM_SET);
            gp_memset((INT8S *)&info_ptr->find_hand_range_cnt[0], 0, sizeof(INT32U)*OMMAND_FIND_HAND_MAX);
            cv_aeawb_flag_unlock();
        #if PFMST_EN == 1
            context_fist.terminationCnt = 100;
            PFMSTracker_framework(0, fistWorkMem_ptr->image.width, fistWorkMem_ptr->image.height, &context_fist, &fist_result.rect[0]);
        #endif
#endif
            info_ptr->command_state = COMMAND_FACE_HAND_FIST_ZERO;
            info_ptr->command_ok = info_ptr->command_state;
            break;
    }
    drv_fd_result_unlock();

    return state;
}
#endif
#endif

static void fd_task_entry(void const *parm)
{
    #define FD_SOFTWARE_CNT_EN             0
    #define FD_HW_SEARCH_EN                1
    #define FD_CNT_SEL                     1
    #define HAND_CNT_SEL                   1000
    #define HAND_FD_CET_SEL                30
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
    INT8U path[128];
    FID_Info *fidInfo_ptr;
#endif
    INT32U i,temp,no_face_cnt,y_buffer,prcess_selection_flag;
    INT32U no_hand_cnt,no_fist_cnt,ack_msg,face_cnt,hand_cnt,fist_cnt;
    INT32U frame_flag,hand_update,fist_update;
    INT32S face_hand_cnt,fist_hand_cnt;
    gpImage *pgray,gray_ptr;
    osEvent result;
#if PFMST_EN == 1 || KOT_EN == 1
    gpImage ROI_Input;
#endif
#if KOT_EN == 1
    INT32U size;
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
#if FD_SOFTWARE_CNT_EN == 1
    INT32U t1,t2,t3,t4;
#endif
#if DEMO_DEBUG_EN == 1
    INT32U t5,t6;
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

#if FD_COMMAND_FLOW_EN == 1
    module_mem_set = (module_command_t *)&module_mem_structure;
    gp_memset((INT8S *)module_mem_set, 0, sizeof(module_command_t));
    module_mem_set->command_prcess_flag = COMMAND_STATE_0;
    command_mode = API_COMMAND_DET_OFF;
    face_hand_fist_command_result_set((module_command_t *)module_mem_set, command_mode);
    DBG_PRINT("COMMAND init: API_COMMAND_DET_OFF\r\n");
    module_mem_set->ImageModule_ptr.width = FD_SIZE_HPIXEL;
    module_mem_set->ImageModule_ptr.height = FD_SIZE_VPIXEL;
    module_mem_set->ImageModule_ptr.widthStep = FD_SIZE_HPIXEL;
    module_mem_set->ImageModule_ptr.ch = 1;
    module_mem_set->ImageModule_ptr.format = IMG_FMT_GRAY;
    module_mem_set->audio_en = 1;
#endif

    image_HE_ptr = (INT32U)gp_malloc_align((FD_SIZE_HPIXEL*FD_SIZE_VPIXEL),4);
    if(image_HE_ptr == 0)
    {
        DBG_PRINT("image_HE_ptr Fail\r\n");
         while(1);
    }
    else
        DBG_PRINT("image_HE_ptr = 0x%x \r\n", image_HE_ptr);
    prcess_mem_set->face_he_workmem = (INT32U)image_HE_ptr;

#if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
    FACEodWorkMem = (ObjDetect_t *)gp_face_detect_alloc();
#endif

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
    fidInfo.subject_num = 0;//5;
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
    gp_memset((INT8S *)_userDB, 0, SZ_ONEDATA*(fidInfo.MAX_SAMPLE_NUM + 10));
    prcess_mem_set->userDB_workmem = (INT32U)fidInfo.userDB;

    fidInfo.scoreLoTh = 150000;
    fidInfo.scoreHiTh = (fidInfo.scoreLoTh) << 1;

    pgray_he = (gpImage *)&grayhe_ptr;
    pgray_he->width = FD_SIZE_HPIXEL;
    pgray_he->height = FD_SIZE_VPIXEL;
    pgray_he->widthStep = FD_SIZE_HPIXEL;
    pgray_he->ch = 1;
    pgray_he->format = IMG_FMT_GRAY;
    pgray_he->ptr = (INT8U *)image_HE_ptr;
    train_max_flag = 0;
#endif

    fdWorkMem = (FaceIdentify_t *)face_recognize_alloc();
    if(!fdWorkMem)
    {
        DBG_PRINT("fdWorkMem fail \r\n");
        while(1);
    }
    fdWorkMem->id_mode = fd_mode;
    fdWorkMem->identify_save_max_number = FACEID_MAX_SAVE_IMAGE_NUMBER;
    fdWorkMem->identify_save_number = Find_JPG_File_Index(fdWorkMem->identify_save_max_number);
    fdWorkMem->training_subject_cnt = 0;
    fdWorkMem->identify_save_en = 1;

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
    if(fdWorkMem->identify_save_en)
    {
        sprintf(path,"ULBP_INFO[%d].bin",0);
        fidInfo_ptr = (FID_Info *)save_file_data_get((INT8U *)path);
        if(fidInfo_ptr == 0)
        {
            train_subject = 0;
            DBG_PRINT("ULBP_INFO fail \r\n");
        }
        else
        {
            fidInfo.TRAIN_NUM = fidInfo_ptr->TRAIN_NUM;
            fidInfo.MAX_SUBJECT_NUM = fidInfo_ptr->MAX_SUBJECT_NUM;
            fidInfo.subject_num = fidInfo_ptr->subject_num;
            if(fidInfo.MAX_SUBJECT_NUM != fidInfo.subject_num)
                train_max_flag = 0;
            else
                train_max_flag = 1;
            fidInfo.subject_index = fidInfo_ptr->subject_index;
            fidInfo.MAX_UPDATE_NUM = fidInfo_ptr->MAX_UPDATE_NUM;
            fidInfo.SAMPLE_NUM_ONE = fidInfo.TRAIN_NUM + fidInfo.MAX_UPDATE_NUM;
            fidInfo.MAX_SAMPLE_NUM = fidInfo.SAMPLE_NUM_ONE*fidInfo.MAX_SUBJECT_NUM;
            gp_free((void *)fidInfo_ptr);
            fidInfo_ptr = 0;
        }

        if(fidInfo.userDB)
        {
            sprintf(path,"ULBP_DATA[%d].bin",0);
            temp = (INT32U)save_file_data_get((INT8U *)path);
            if(temp == 0)
            {
                DBG_PRINT("ULBP_DATA fail \r\n");
            }
            else
            {
                gp_free((void *)fidInfo.userDB);
                fidInfo.userDB = (void *)temp;
                if(fidInfo.subject_num == fidInfo.MAX_SUBJECT_NUM)
                    fdWorkMem->training_subject_cnt = fidInfo.MAX_SUBJECT_NUM;
                else
                    fdWorkMem->training_subject_cnt = fidInfo.subject_index;
                if(fidInfo.subject_index == (fidInfo.MAX_SUBJECT_NUM - 1))
                    train_subject = fidInfo.MAX_SUBJECT_NUM;
                else
                    train_subject = fidInfo.subject_index;
                prcess_mem_set->userDB_workmem = (INT32U)fidInfo.userDB;
            }
        }
    }
#endif
#endif

    fd_gpio_init();

#if FACE_TRACKING_EN == 1
    ObjWorkMem_ptr = (ObjDetect_t *)&ObjWorkMem;
    gp_memset((INT8S *)ObjWorkMem_ptr,0, sizeof(ObjDetect_t));
 	ObjWorkMem_ptr->image.width     = FD_SIZE_HPIXEL;
	ObjWorkMem_ptr->image.height    = FD_SIZE_VPIXEL;
    ObjWorkMem_ptr->image.ch        = 1;
    ObjWorkMem_ptr->image.widthStep = FD_SIZE_HPIXEL;
    ObjWorkMem_ptr->image.format    = IMG_FMT_GRAY;
    if(obj_track_init((ObjDetect_t *)ObjWorkMem_ptr))
    {
        DBG_PRINT("ObjWorkMem_ptr malloc fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("ObjWorkMem_ptr = 0x%x\r\n",ObjWorkMem_ptr->obj_track_WorkMem);

#if HAND_EN == 1
    handWorkMem_ptr = (ObjDetect_t *)&handWorkMem;
    gp_memset((INT8S *)handWorkMem_ptr,0, sizeof(ObjDetect_t));
 	handWorkMem_ptr->image.width     = FD_SIZE_HPIXEL;
	handWorkMem_ptr->image.height    = FD_SIZE_VPIXEL;
    handWorkMem_ptr->image.ch        = 1;
    handWorkMem_ptr->image.widthStep = FD_SIZE_HPIXEL;
    handWorkMem_ptr->image.format    = IMG_FMT_GRAY;
    if(obj_track_init((ObjDetect_t *)handWorkMem_ptr))
    {
        DBG_PRINT("handWorkMem_ptr malloc fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("handWorkMem_ptr = 0x%x\r\n",handWorkMem_ptr->obj_track_WorkMem);

    fistWorkMem_ptr = (ObjDetect_t *)&fistWorkMem;
    gp_memset((INT8S *)fistWorkMem_ptr,0, sizeof(ObjDetect_t));
 	fistWorkMem_ptr->image.width     = FD_SIZE_HPIXEL;
	fistWorkMem_ptr->image.height    = FD_SIZE_VPIXEL;
    fistWorkMem_ptr->image.ch        = 1;
    fistWorkMem_ptr->image.widthStep = FD_SIZE_HPIXEL;
    fistWorkMem_ptr->image.format    = IMG_FMT_GRAY;
    if(obj_track_init((ObjDetect_t *)fistWorkMem_ptr))
    {
        DBG_PRINT("fistWorkMem_ptr malloc fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("fistWorkMem_ptr = 0x%x\r\n",fistWorkMem_ptr->obj_track_WorkMem);
#endif
#endif

    PFMST_CROP_hw_set( user_PFMST_CROP_hw_function );

#if KOT_EN == 1
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
	KOT_workmem = (INT32U)gp_malloc_align((size*2),64);
    if(KOT_workmem == 0)
    {
        DBG_PRINT("KOT workmem malloc fail \r\n");
        while(1);
    }
    else
        DBG_PRINT("KOT WorkMem:0x%X \r\n", KOT_workmem);

	KOT_init_HW(KOTMapW, KOTMapH, (int *)KOT_workmem);                 // robust feature matching system
	KOT_ParamSet((int *)KOT_workmem, extractionThre, matchingThre, matchingThreRangeV, minExtractionThre, incExtractionThre, decExtractionThre, startMatchingPointN);

    #ifdef KOT_HW
        drv_l1_obj_init();
        drv_l1_obj_user_malloc_set(user_malloc_function);
        KOT_fftobj_hw_set(user_KOT_hw_function);
    #endif

    gData_ptr = (globalData_KOT*)KOT_workmem;
    #if FD_COMMAND_FLOW_EN == 1
        module_mem_set->KOT_workmem = (INT32U)KOT_workmem;
        module_mem_set->KOTData_ptr = (void *)&KOT_workmem;
        module_mem_set->KOTMemSize = size;
    #endif
    gData_ptr_result = (globalData_KOT*)(KOT_workmem + size);

	objROI.ptr = (unsigned char*)gp_malloc_align(KOTMapW*KOTMapH, 64);
	if(objROI.ptr == 0)
	{
        DBG_PRINT("KOT objROI malloc fail \r\n");
        while(1);
	}
	objROI.width = KOTMapW;
	objROI.widthStep = KOTMapW;
	objROI.height = KOTMapH;
	objROI.ch = 1;
	objROI.format = IMG_FMT_GRAY;
	#if FD_COMMAND_FLOW_EN == 1
        module_mem_set->objROI.ptr = objROI.ptr;
        module_mem_set->objROI.width = objROI.width;
        module_mem_set->objROI.widthStep = objROI.widthStep;
        module_mem_set->objROI.height = objROI.height;
        module_mem_set->objROI.ch = objROI.ch;
        module_mem_set->objROI.format = objROI.format;
	#endif
	prcess_mem_set->kot_roi_workmem = (INT32U)objROI.ptr;
	DBG_PRINT("objROI = 0x%x\r\n",objROI.ptr);

	ROISmoothImage.ptr = (unsigned char*)gp_malloc_align(KOTMapW*KOTMapH, 64);
	if(ROISmoothImage.ptr == 0)
	{
        DBG_PRINT("KOT ROISmoothImage malloc fail \r\n");
        while(1);
	}
	ROISmoothImage.width = KOTMapW;
	ROISmoothImage.widthStep = KOTMapW;
	ROISmoothImage.height = KOTMapH;
	ROISmoothImage.ch = 1;
	ROISmoothImage.format = IMG_FMT_GRAY;
	#if FD_COMMAND_FLOW_EN == 1
        module_mem_set->ROISmoothImage.width = ROISmoothImage.width;
        module_mem_set->ROISmoothImage.height = ROISmoothImage.height;
        module_mem_set->ROISmoothImage.ch = ROISmoothImage.ch;
        module_mem_set->ROISmoothImage.format = ROISmoothImage.format;
        module_mem_set->ROISmoothImage.ptr = ROISmoothImage.ptr;
    #endif
	prcess_mem_set->kot_prcess_workmem = (INT32U)ROISmoothImage.ptr;
	DBG_PRINT("ROISmoothImage = 0x%x\r\n",ROISmoothImage.ptr);
#endif

#if PFMST_EN == 1
	//***********************************************************
	//           Partical Filter Mean Shift Tracker Init
	//***********************************************************
	unsigned int PFMST_size_byte = getPFMSTrackingMemorySize();

    temp = (PFMST_size_byte*2);
    pfmst_workmem = (INT32U)gp_malloc_align((temp*4),64);
    if(pfmst_workmem == 0)
    {
        DBG_PRINT("PFMST WorkMem malloc fail \r\n");
        while(1);
    }
    //else
        //DBG_PRINT("PFMST WorkMem:0x%X \r\n", pfmst_workmem);
    prcess_mem_set->pfmst_prcess_workmem = pfmst_workmem;

	PFMSTMem_hand.workMem = (unsigned char*)pfmst_workmem;
	DBG_PRINT("PFMSTMem_hand WorkMem:0x%X \r\n", PFMSTMem_hand.workMem);
	initGPTrack(PFMSTMem_hand.workMem, &context_hand, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
	context_hand.detectMode = GP_HAND;  //for GP Haar detector
	//context_hand.HIST_THRE = 0.01;
	//context_hand.SAMPLE_STEP = 4;
	//context_hand.ZOOMINOUT_FLG = 1;
#if FD_COMMAND_FLOW_EN == 0
	context_hand.terminatedThre = 17;
#else
	context_hand.terminatedThre = 28;
#endif

	PFMSTMem_fist.workMem = (unsigned char*)(PFMSTMem_hand.workMem + temp);
	DBG_PRINT("PFMSTMem_fist WorkMem:0x%X \r\n", PFMSTMem_fist.workMem);
	initGPTrack(PFMSTMem_fist.workMem, &context_fist, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
	context_fist.detectMode = GP_FIST;  //for GP Haar detector
	//context_fist.zoomInOutActive = 0;
	context_fist.terminatedThre = 17;

	PFMSTMem_hand_left.workMem = (unsigned char*)(PFMSTMem_fist.workMem + temp);
	DBG_PRINT("PFMSTMem_hand_left WorkMem:0x%X \r\n", PFMSTMem_hand_left.workMem);
	initGPTrack(PFMSTMem_hand_left.workMem, &pContext_left, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
	pContext_left.detectMode = GP_HAND;  //for GP Haar detector
	pContext_left.terminatedThre = 1;
	/*
	pContext_left.UV_SIZE = 8;
	pContext_left.Y_SIZE = 16;
	pContext_left.UV_SHIFT_SIZE = 3;   //2^x = UV_SIZE
	pContext_left.Y_SHIFT_SIZE = 4;    //2^x = Y_SIZE
*/
	PFMSTMem_hand_right.workMem = (unsigned char*)(PFMSTMem_hand_left.workMem + temp);
	DBG_PRINT("PFMSTMem_hand_right WorkMem:0x%X \r\n", PFMSTMem_hand_right.workMem);
	initGPTrack(PFMSTMem_hand_right.workMem, &pContext_right, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL);
	pContext_right.detectMode = GP_HAND;  //for GP Haar detector
	pContext_right.terminatedThre = 1;
    /*
    pContext_right.UV_SIZE = 8;
	pContext_right.Y_SIZE = 16;
	pContext_right.UV_SHIFT_SIZE = 3;   //2^x = UV_SIZE
	pContext_right.Y_SHIFT_SIZE = 4;    //2^x = Y_SIZE
	*/
#endif

#if PFMST_EN == 1 || KOT_EN == 1
    ROI_Input.width = FD_SIZE_HPIXEL;
    ROI_Input.widthStep = FD_SIZE_HPIXEL*2;
    ROI_Input.height = FD_SIZE_VPIXEL;
    ROI_Input.ch = 2;
    ROI_Input.format = IMG_FMT_UYVY;//IMG_FMT_YUYV;IMG_FMT_RGB;//

    fd_ymem = (INT32U)gp_malloc_align(FD_SIZE_HPIXEL*FD_SIZE_VPIXEL, 64);
    if(fd_ymem == 0)
    {
        DBG_PRINT("FD_Y WorkMem malloc fail \r\n");
        while(1);
    }
    prcess_mem_set->fd_y_workmem = fd_ymem;
    DBG_PRINT("FD_Y WorkMem:0x%X \r\n", fd_ymem);
#endif

    // for new int mode use
    drv_l1_scaler_new_int_callback_set(fd_scaler_step_get);
    prcess_selection_state_post(MSG_PRCESS_FD);

    no_face_cnt = 0;
    no_hand_cnt = 0;
    no_fist_cnt = 0;
    fd_org_buf = 0;
    frame_flag = 0;
    hand_update = 1;
    fist_update = 1;
    prcess_selection_flag = 0;
    face_cnt = 0;
    hand_cnt = 0;
    fist_cnt = 0;
    face_hand_cnt = 0;
    fist_hand_cnt = 0;
    osDelay(5);

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
                //DBG_PRINT("FD_buffer = 0x%x\r\n", fd_org_buf);
                //DBG_PRINT("F");
            #if FD_SOFTWARE_CNT_EN == 1
                t1 = xTaskGetTickCount();
            #endif
                fd_gpio1_set(1);
                pgray->width = FD_SIZE_HPIXEL;
                pgray->height = FD_SIZE_VPIXEL;
                pgray->widthStep = FD_SIZE_HPIXEL;
                pgray->ch = 1;
                pgray->format = IMG_FMT_GRAY;
            #if PFMST_EN == 1 && KOT_EN == 1
                fd_set_frame(fd_org_buf, fd_ymem, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, FD_SIZE_HPIXEL, FD_SIZE_VPIXEL, BITMAP_GRAY);
                pgray->ptr = (INT8U *)fd_ymem;
            #else
                pgray->ptr = (INT8U *)fd_org_buf;
            #endif
#if FD_COMMAND_FLOW_EN == 1
                fist_hand_cnt = face_hand_fist_command_prcess((module_command_t *)module_mem_set, fd_org_buf, fd_ymem);
                #if 0
                    if(fist_hand_cnt < 0)
                        DBG_PRINT("face_hand_fist_command_prcess = %d\r\n", fist_hand_cnt);
                #endif
                #if FACE_RECOGNIZE_MEUN_EN == 1
                    drv_fd_result_lock();
                    temp = face_recognize_en;
                    drv_fd_result_unlock();
                    #if 0//FD_SOFTWARE_CNT_EN == 1
                        t3 = xTaskGetTickCount();
                    #endif
                    if(face_recognize_en)
                    {
                        #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                            y_buffer = faceRoiDetect_new(pgray, pgray_he,(gpRect *)&fd_result.rect[0], fd_mode);
                        #else
                            y_buffer = gp_faceRoiDetect((ObjDetect_t *)FACEodWorkMem, pgray, (gpRect *)&fd_result.rect[0], SCALE_LINEAR);
                        #endif
                    }
                    else
                        y_buffer = 0;
                    #if 0//FD_SOFTWARE_CNT_EN == 1
                        t4 = xTaskGetTickCount();
                        DBG_PRINT("faceRoiDetect_new: time=%d \r\n", (t4 - t3));
                    #endif
                    if(y_buffer)
                    {
                        drv_fd_result_lock();
                        if(fd_result.rect[0].x == 0)
                            fd_result.is_best_face = 0;
                        else
                            fd_result.is_best_face = 1;
                    #if FACE_DETECT_MULTI_PEOPLE_FLOW_EN == 1
                        fd_result.result_cnt = y_buffer;
                        fd_draw_result.is_best_face = fd_result.is_best_face;
                        fd_draw_result.result_cnt = fd_result.result_cnt;
                        for(i=0;i<fd_result.result_cnt;i++)
                        {
                            if(i < HAND_NUM_SET)
                            {
                                fd_draw_result.rect[i].x = fd_result.rect[(i*3)].x;
                                fd_draw_result.rect[i].y = fd_result.rect[(i*3)].y;
                                fd_draw_result.rect[i].width = fd_result.rect[(i*3)].width;
                                fd_draw_result.rect[i].height = fd_result.rect[(i*3)].height;
                            }
                        }
                    #else
                        fd_result.result_cnt = 1;
                        fd_draw_result.is_best_face = fd_result.is_best_face;
                        fd_draw_result.result_cnt = fd_result.result_cnt;
                        fd_draw_result.rect[FACE_NUM_SET].x = fd_result.rect[0].x;
                        fd_draw_result.rect[FACE_NUM_SET].y = fd_result.rect[0].y;
                        fd_draw_result.rect[FACE_NUM_SET].width = fd_result.rect[0].width;
                        fd_draw_result.rect[FACE_NUM_SET].height = fd_result.rect[0].height;
                    #endif
                        fdWorkMem->id_mode = fd_mode;
                        if(fd_result.is_best_face)
                        {
                            if((fd_result.rect[1].x == 0) || (fd_result.rect[2].x == 0))
                                fd_result.is_get_eye = 0;
                            else
                                fd_result.is_get_eye = 1;
                        }
                        else
                            fd_result.is_get_eye = 0;
                        drv_fd_result_unlock();
                        #if 0//FD_SOFTWARE_CNT_EN == 1
                            t3 = xTaskGetTickCount();
                        #endif
                        #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                            face_recognize_proc_new((void *)fdWorkMem, (gpImage *)pgray, (void *)&fd_result, fd_org_buf, fd_result.result_cnt);
                        #else
                            face_recognize_proc((void *)fdWorkMem, (gpImage *)pgray, (void *)&fd_result);
                        #endif
                        #if 0//FD_SOFTWARE_CNT_EN == 1
                            t4 = xTaskGetTickCount();
                            DBG_PRINT("face_recognize_proc_new: time=%d \r\n", (t4 - t3));
                        #endif
                        drv_fd_result_lock();
                        fd_result.is_get_eye = 0;
                        fd_draw_result.is_get_eye = fd_result.is_get_eye;
                        fd_draw_result.identify_ok = fd_result.identify_ok;
                        fd_draw_result.training_ok = fd_result.training_ok;
                        drv_fd_result_unlock();
                    }
                    else
                    {
                        fd_result.is_best_face = 0;
                        fd_result.result_cnt = 0;
                        fd_draw_result.is_best_face = fd_result.is_best_face;
                        fd_draw_result.result_cnt = fd_result.result_cnt;
                    }
                #endif
#else
                // prcess selection state
                prcess_selection_flag = prcess_selection_state_get();

                drv_fd_result_lock();
                temp = face_detect_en;
                drv_fd_result_unlock();
            #if FD_NEW_FLOW_EN == 1
                if(temp)
            #else
                if(temp && (prcess_selection_flag == MSG_PRCESS_FD))
            #endif
                {
                    #if DEMO_DEBUG_EN == 1
                        t3 = xTaskGetTickCount();
                    #endif
                    drv_fd_result_lock();
                    temp = face_recognize_en;
                    drv_fd_result_unlock();
                    #if SCALER_LINEAR_EN == 1
                        #if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
                            if(temp)
                                y_buffer = gp_faceRoiDetect((ObjDetect_t *)FACEodWorkMem, pgray, (gpRect *)&fd_result.rect[0], SCALE_LINEAR);
                            else
                                y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_LINEAR,0);
                        #elif FACE_RECOGNIZE_MEUN_EN == 1 && GP_NEW_FACE_RECOGNIZE_FLOW_EN == 0
                            if(temp)
                                y_buffer = faceRoiDetect(pgray, (gpRect *)&fd_result.rect[0],SCALE_LINEAR,0);
                            else
                                y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_LINEAR,0);
                        #else
                            y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_LINEAR,0);
                        #endif
                    #else
                        #if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
                            if(temp)
                                y_buffer = gp_faceRoiDetect((ObjDetect_t *)FACEodWorkMem, pgray, (gpRect *)&fd_result.rect[0], SCALE_NONLINEAR);
                            else
                                y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,0);
                        #elif FACE_RECOGNIZE_MEUN_EN == 1 && GP_NEW_FACE_RECOGNIZE_FLOW_EN == 0
                            if(temp)
                                y_buffer = faceRoiDetect(pgray, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,0);
                            else
                                y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,0);
                        #else
                            y_buffer = face_Detect_only(pgray, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,0);
                        #endif
                    #endif
                    #if DEMO_DEBUG_EN == 1
                        t4 = xTaskGetTickCount();
                        DBG_PRINT("fd: time=%d \r\n", (t4 - t3));
                    #endif

                    #if KOT_EN == 1
                        context.detect_total = y_buffer;
                        if(context.detect_total != 0)
                        {
                            gData_ptr->objSet = (gpRect*)&fd_result.rect[0];
                        }

                        if( (context.detect_total != 0 && gData_ptr->modelAlreadyFlg == 0 ) || (context.detect_total != 0 )  )
                        {
                            gData_ptr->InitObject.width = fd_result.rect[0].width - (fd_result.rect[0].width>>2);
                            gData_ptr->InitObject.height = fd_result.rect[0].height - (fd_result.rect[0].height>>2);
                            gData_ptr->InitObject.x = fd_result.rect[0].x + (fd_result.rect[0].width>>3);
                            gData_ptr->InitObject.y = fd_result.rect[0].y + (fd_result.rect[0].height>>3);
                            gData_ptr->updateObject = gData_ptr->InitObject;

                            gData_ptr->KOT_initFlg = 1;
                        }

                        if(gData_ptr->KOT_initFlg == 1)
                        {
                        #if FD_DEBUG_EN == 1
                            t3 = xTaskGetTickCount();
                        #endif
                            objClip.x = gData_ptr->updateObject.x;
                            objClip.y = gData_ptr->updateObject.y;
                            objClip.width = gData_ptr->updateObject.width;
                            objClip.height = gData_ptr->updateObject.height;
                            KOT_scalerClipStart( pgray, &objROI, objClip);
                            KOT_scalerEnd();
                            GPSmoothImage(objROI.ptr, ROISmoothImage.ptr, objROI.width, objROI.height);
                            keypointsObjectTracker((void *)KOT_workmem, &ROISmoothImage, context.detect_total);
                        #if FD_DEBUG_EN == 1
                            t4 = xTaskGetTickCount();
                            DBG_PRINT("KOT tracking: time=%d \r\n", (t4 - t3));
                        #endif
                        }
                    #endif

                        if(y_buffer)
                        {
                            drv_fd_result_lock();
                        #if FACE_RECOGNIZE_MEUN_EN == 1 && GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
                            if(fd_result.rect[0].x == 0)
                                fd_result.is_best_face = 0;
                            else
                                fd_result.is_best_face = 1;
                        #else
                            fd_result.is_best_face = 1;
                        #endif
                            fd_result.is_get_eye = 0;
                            fd_result.result_cnt = 1;
                            fd_draw_result.is_best_face = fd_result.is_best_face;
                            fd_draw_result.is_get_eye = fd_result.is_get_eye;
                            fd_draw_result.result_cnt = fd_result.result_cnt;
                            fd_draw_result.rect[FACE_NUM_SET].x = fd_result.rect[0].x;
                            fd_draw_result.rect[FACE_NUM_SET].y = fd_result.rect[0].y;
                            fd_draw_result.rect[FACE_NUM_SET].width = fd_result.rect[0].width;
                            fd_draw_result.rect[FACE_NUM_SET].height = fd_result.rect[0].height;
                            no_face_cnt = 0;
                            drv_fd_result_unlock();
                            #if KOT_EN == 1
                                gData_ptr_result->KOT_initFlg = 0;
                            #endif
                    #if FACE_RECOGNIZE_MEUN_EN == 1
                            drv_fd_result_lock();
                            fdWorkMem->id_mode = fd_mode;
                        #if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
                            if(fd_result.is_best_face)
                            {
                                if((fd_result.rect[1].x == 0) || (fd_result.rect[2].x == 0))
                                    fd_result.is_get_eye = 0;
                                else
                                    fd_result.is_get_eye = 1;
                            }
                            else
                                fd_result.is_get_eye = 0;
                        #else
                                fd_result.is_get_eye = 1;
                        #endif
                            #if DEMO_DEBUG_EN == 1
                                t5 = xTaskGetTickCount();
                            #endif
                            drv_fd_result_unlock();
                            #if GP_NEW_FACE_RECOGNIZE_FLOW_EN == 1
                                gp_face_recognize_proc((void *)fdWorkMem, (gpImage *)pgray, (void *)&fd_result);
                            #else
                                face_recognize_proc((void *)fdWorkMem, (gpImage *)pgray, (void *)&fd_result);
                            #endif
                            drv_fd_result_lock();
                            #if DEMO_DEBUG_EN == 1
                                t6 = xTaskGetTickCount();
                                DBG_PRINT("recognize=%d \r\n", (t5 - t6));
                            #endif
                            fd_result.is_get_eye = 0;
                            fd_draw_result.is_get_eye = fd_result.is_get_eye;
                            fd_draw_result.identify_ok = fd_result.identify_ok;
                            fd_draw_result.training_ok = fd_result.training_ok;
                            drv_fd_result_unlock();
                    #endif
                        }
                        else
                        {
                            drv_fd_result_lock();
                            fd_result.is_best_face = 0;
                            fd_result.is_get_eye = 0;
                            fd_result.result_cnt = 0;
                            fd_draw_result.is_best_face = fd_result.is_best_face;
                            fd_draw_result.is_get_eye = fd_result.is_get_eye;
                            fd_draw_result.result_cnt = fd_result.result_cnt;
                    #if KOT_EN == 1
                            gData_ptr_result->InitObject.width = gData_ptr->InitObject.width;
                            gData_ptr_result->InitObject.height = gData_ptr->InitObject.height;
                            gData_ptr_result->InitObject.x = gData_ptr->InitObject.x;
                            gData_ptr_result->InitObject.y = gData_ptr->InitObject.y;
                            gData_ptr_result->updateObject = gData_ptr->updateObject;
                            gData_ptr_result->KOT_initFlg = gData_ptr->KOT_initFlg;
                            gData_ptr_result->trackTerminateFlg = gData_ptr->trackTerminateFlg;
                    #endif
                            no_face_cnt++;
                            drv_fd_result_unlock();
                        }
                }
                else
                {
                    drv_fd_result_lock();
                    fd_result.is_best_face = 0;
                    fd_result.is_get_eye = 0;
                    fd_result.result_cnt = 0;
                    fd_draw_result.is_best_face = fd_result.is_best_face;
                    fd_draw_result.is_get_eye = fd_result.is_get_eye;
                    fd_draw_result.result_cnt = fd_result.result_cnt;
                    drv_fd_result_unlock();
                }
#if HAND_EN == 1
                // hand detect
                handWorkMem_ptr->image.width = pgray->width;
                handWorkMem_ptr->image.height = pgray->height;
                handWorkMem_ptr->image.widthStep = pgray->widthStep;
                handWorkMem_ptr->image.ch = pgray->ch;
                handWorkMem_ptr->image.format = pgray->format;
                handWorkMem_ptr->image.ptr = pgray->ptr;
                drv_fd_result_lock();
                temp = hand_detect_en;
                drv_fd_result_unlock();
            #if FD_NEW_FLOW_EN == 1
                if(temp)
            #else
                if(temp && (prcess_selection_flag == MSG_PRCESS_HAND))
            #endif
                {
                    if((context_hand.detect_total == 0) && (context_hand.tracking_flag != 0))
                    {
                        y_buffer = 0;
                    #if PFMST_EN == 1
                        drv_fd_result_lock();
                        hand_result.rect[0].x = trackingSet_hand_result[0].x;
                        hand_result.rect[0].y = trackingSet_hand_result[0].y;
                        hand_result.rect[0].width = trackingSet_hand_result[0].width;
                        hand_result.rect[0].height = trackingSet_hand_result[0].height;
                        drv_fd_result_unlock();
                    #endif
                    #if FD_HW_SEARCH_EN == 1
                        hand_cnt++;
                        if((hand_cnt % HAND_CNT_SEL) == 0)
                        {
                            y_buffer = gesture_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr,(gpRect *)&hand_result.rect[0],1);
                            hand_cnt = 0;
                            #if PFMST_EN == 1
                                context_hand.detect_total = y_buffer;
                                if(context_hand.detect_total)
                                    context_hand.terminationCnt = 100;

                                #if DEMO_DEBUG_EN == 1
                                    t3 = xTaskGetTickCount();
                                #endif
                                PFMSTracker_framework((INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_hand, &hand_result.rect[0]);
                                if(y_buffer)
                                    context_hand.detect_total = 0;
                                #if DEMO_DEBUG_EN == 1
                                    t4 = xTaskGetTickCount();
                                    if (y_buffer != 0)
                                        DBG_PRINT("hand model construction: time=%d \r\n", (t4 - t3));
                                    else if (context_hand.tracking_flag != 0)
                                        DBG_PRINT("hand tracking: time=%d \r\n", (t4 - t3));
                                    else
                                        DBG_PRINT("hand release: time=%d \r\n", (t4 - t3));
                                #endif
                            #endif
                        }

                        face_hand_cnt++;
                        if((face_hand_cnt % HAND_FD_CET_SEL) == 0 && y_buffer == 0)
                        {

                            y_buffer = face_Detect_With_pfmt(pgray, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,(gpRect *)&hand_result.rect[0],1);
                            face_hand_cnt = 0;
                            hand_result.rect[0].x = trackingSet_hand_result[0].x;
                            hand_result.rect[0].y = trackingSet_hand_result[0].y;
                            hand_result.rect[0].width = trackingSet_hand_result[0].width;
                            hand_result.rect[0].height = trackingSet_hand_result[0].height;
                            #if PFMST_EN == 1
                                if(y_buffer)
                                {
                                    context_hand.tracking_flag = 0;
                                    context_hand.terminationCnt = 100;
                                    y_buffer = 0;
									//print_string("==============   kick face!!  =================\r\n");
                                }
                                context_hand.detect_total = 0;
                                #if DEMO_DEBUG_EN == 1
                                    t3 = xTaskGetTickCount();
                                #endif
                                PFMSTracker_framework((INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_hand, &hand_result.rect[0]);
                                #if DEMO_DEBUG_EN == 1
                                    t4 = xTaskGetTickCount();
                                    if (y_buffer != 0)
                                        DBG_PRINT("hand model construction: time=%d \r\n", (t4 - t3));
                                    else if (context_hand.tracking_flag != 0)
                                        DBG_PRINT("hand tracking: time=%d \r\n", (t4 - t3));
                                    else
                                        DBG_PRINT("hand release: time=%d \r\n", (t4 - t3));
                                #endif
                            #endif
                        }
                        else
                    #endif
                        {
                            #if PFMST_EN == 1
                                context_hand.detect_total = y_buffer;
                                #if DEMO_DEBUG_EN == 1
                                    t3 = xTaskGetTickCount();
                                #endif
                                PFMSTracker_framework((INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_hand, &hand_result.rect[0]);
                                #if DEMO_DEBUG_EN == 1
                                    t4 = xTaskGetTickCount();
                                    if (y_buffer != 0)
                                        DBG_PRINT("hand model construction: time=%d \r\n", (t4 - t3));
                                    else if (context_hand.tracking_flag != 0)
                                        DBG_PRINT("hand tracking: time=%d \r\n", (t4 - t3));
                                    else
                                        DBG_PRINT("hand release: time=%d \r\n", (t4 - t3));
                                #endif
                                gesture_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr,(gpRect *)&hand_result.rect[0],0);
                            #endif
                        }
                    }
                    else
                    {
                        y_buffer = gesture_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr,0,1);
                        #if PFMST_EN == 1
                                context_hand.detect_total = y_buffer;
                            #if DEMO_DEBUG_EN == 1
                                t3 = xTaskGetTickCount();
                            #endif
                                PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_hand, &hand_result.rect[0]);
                                if(y_buffer)
                                    context_hand.detect_total = 0;
                            #if DEMO_DEBUG_EN == 1
                                t4 = xTaskGetTickCount();
                                if (y_buffer != 0)
                                    DBG_PRINT("hand model construction: time=%d \r\n", (t4 - t3));
                                else if (context_hand.tracking_flag != 0)
                                    DBG_PRINT("hand tracking: time=%d \r\n", (t4 - t3));
                                else
                                    DBG_PRINT("hand release: time=%d \r\n", (t4 - t3));
                            #endif
                        #endif
                    }
                    // get result
                    drv_fd_result_lock();
                    if(y_buffer)
                    {
                        hand_result.is_hand = 1;
                        fd_draw_result.is_hand = hand_result.is_hand;
                        fd_draw_result.rect[HAND_NUM_SET].x = hand_result.rect[0].x;
                        fd_draw_result.rect[HAND_NUM_SET].y = hand_result.rect[0].y;
                        fd_draw_result.rect[HAND_NUM_SET].width = hand_result.rect[0].width;
                        fd_draw_result.rect[HAND_NUM_SET].height = hand_result.rect[0].height;
                        #if PFMST_EN == 1
                            context_hand_result.detect_total = 0;
                            context_hand_result.tracking_flag = context_hand.tracking_flag;
                            trackingSet_hand_result[0].width = (unsigned int)(context_hand.tracked_obj.width);
                            trackingSet_hand_result[0].height = (unsigned int)(context_hand.tracked_obj.height);
                            trackingSet_hand_result[0].x = (unsigned int)(context_hand.tracked_obj.center.x - context_hand.tracked_obj.width/2);
                            trackingSet_hand_result[0].y = (unsigned int)(context_hand.tracked_obj.center.y - context_hand.tracked_obj.height/2);
                        #endif
                        no_hand_cnt = 0;
                    }
                    else
                    {
                        hand_result.is_hand = 0;
                        fd_draw_result.is_hand = hand_result.is_hand;
                        #if PFMST_EN == 1
                            context_hand_result.detect_total = context_hand.detect_total;
                            context_hand_result.tracking_flag = context_hand.tracking_flag;
                            trackingSet_hand_result[0].width = (unsigned int)(context_hand.tracked_obj.width);
                            trackingSet_hand_result[0].height = (unsigned int)(context_hand.tracked_obj.height);
                            trackingSet_hand_result[0].x = (unsigned int)(context_hand.tracked_obj.center.x - context_hand.tracked_obj.width/2);
                            trackingSet_hand_result[0].y = (unsigned int)(context_hand.tracked_obj.center.y - context_hand.tracked_obj.height/2);
                        #endif
                        no_hand_cnt++;
                    }
                    drv_fd_result_unlock();
                }
                else
                {
                    fd_draw_result.is_hand = 0;
                    #if PFMST_EN == 1
                            context_hand.detect_total = fd_draw_result.is_hand;
                        #if DEMO_DEBUG_EN == 1
                            t3 = xTaskGetTickCount();
                        #endif
                            PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_hand, &hand_result.rect[0]);
                        #if DEMO_DEBUG_EN == 1
                            t4 = xTaskGetTickCount();
                            if (context_hand.detect_total != 0)
                                DBG_PRINT("hand model construction: time=%d \r\n", (t4 - t3));
                            else if (context_hand.tracking_flag != 0)
                                DBG_PRINT("hand tracking: time=%d \r\n", (t4 - t3));
                            else
                                DBG_PRINT("hand release: time=%d \r\n", (t4 - t3));
                        #endif
                        drv_fd_result_lock();
                        if((context_hand.detect_total == 0) && (context_hand.tracking_flag != 0))
                        {
                            context_hand_result.detect_total = context_hand.detect_total;
                            context_hand_result.tracking_flag = context_hand.tracking_flag;
                            trackingSet_hand_result[0].width = (unsigned int)(context_hand.tracked_obj.width);
                            trackingSet_hand_result[0].height = (unsigned int)(context_hand.tracked_obj.height);
                            trackingSet_hand_result[0].x = (unsigned int)(context_hand.tracked_obj.center.x - context_hand.tracked_obj.width/2);
                            trackingSet_hand_result[0].y = (unsigned int)(context_hand.tracked_obj.center.y - context_hand.tracked_obj.height/2);
                            no_hand_cnt = 0;
                        }
                        else
                        {
                            context_hand_result.detect_total = context_hand.detect_total;
                            context_hand_result.tracking_flag = context_hand.tracking_flag;
                            no_hand_cnt++;
                        }
                        drv_fd_result_unlock();
                    #else
                        no_hand_cnt++;
                    #endif
                }

                // fist detect
                fistWorkMem_ptr->image.width = pgray->width;
                fistWorkMem_ptr->image.height = pgray->height;
                fistWorkMem_ptr->image.widthStep = pgray->widthStep;
                fistWorkMem_ptr->image.ch = pgray->ch;
                fistWorkMem_ptr->image.format = pgray->format;
                fistWorkMem_ptr->image.ptr = pgray->ptr;
                drv_fd_result_lock();
                temp = fist_detect_en;
                drv_fd_result_unlock();
            #if FD_NEW_FLOW_EN == 1
                if(temp)
            #else
                if(temp && (prcess_selection_flag == MSG_PRCESS_FIST))
            #endif
                {
                    if((context_fist.detect_total == 0) && (context_fist.tracking_flag != 0))
                    {
                        y_buffer = 0;
                    #if PFMST_EN == 1
                        drv_fd_result_lock();
                        fist_result.rect[0].x = trackingSet_fist_result[0].x;
                        fist_result.rect[0].y = trackingSet_fist_result[0].y;
                        fist_result.rect[0].width = trackingSet_fist_result[0].width;
                        fist_result.rect[0].height = trackingSet_fist_result[0].height;
                        drv_fd_result_unlock();
                    #endif
                    #if FD_HW_SEARCH_EN == 1
                        fist_cnt++;
                        if((fist_cnt % HAND_CNT_SEL) == 0)
                        {
                            y_buffer = fist_detect_proc_with_pfmst((ObjDetect_t *)handWorkMem_ptr,(gpRect *)&fist_result.rect[0],1);
                            fist_cnt = 0;
                            #if PFMST_EN == 1
                                context_fist.detect_total = y_buffer;
                                if(context_fist.detect_total)
                                    context_fist.initial_flag = 0;
                                #if DEMO_DEBUG_EN == 1
                                    t3 = xTaskGetTickCount();
                                #endif
                                PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_fist, &fist_result.rect[0]);
                                if(y_buffer)
                                    context_fist.detect_total = 0;
                                #if DEMO_DEBUG_EN == 1
                                    t4 = xTaskGetTickCount();
                                    if (y_buffer != 0)
                                        DBG_PRINT("fist model construction: time=%d \r\n", (t4 - t3));
                                    else if (context_fist.tracking_flag != 0)
                                        DBG_PRINT("fist tracking: time=%d \r\n", (t4 - t3));
                                    else
                                        DBG_PRINT("fist release: time=%d \r\n", (t4 - t3));
                                #endif
                            #endif
                        }
                        fist_hand_cnt++;
                        if((fist_hand_cnt % HAND_FD_CET_SEL) == 0 && y_buffer == 0)
                        {
                            y_buffer = face_Detect_With_pfmt(pgray, (gpRect *)&fd_result.rect[0],SCALE_NONLINEAR,(gpRect *)&fist_result.rect[0],1);
                            fist_hand_cnt = 0;
                            fist_result.rect[0].x = trackingSet_fist_result[0].x;
                            fist_result.rect[0].y = trackingSet_fist_result[0].y;
                            fist_result.rect[0].width = trackingSet_fist_result[0].width;
                            fist_result.rect[0].height = trackingSet_fist_result[0].height;
                            #if PFMST_EN == 1
                                if(y_buffer)
                                {
                                    context_fist.tracking_flag = 0;
                                    context_fist.terminationCnt = 100;
                                    y_buffer = 0;
									//print_string("==============   kick face!!  =================\r\n");
                                }
                                context_fist.detect_total = 0;
                                #if DEMO_DEBUG_EN == 1
                                    t3 = xTaskGetTickCount();
                                #endif
                                PFMSTracker_framework((INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_fist, &hand_result.rect[0]);
                                #if DEMO_DEBUG_EN == 1
                                    t4 = xTaskGetTickCount();
                                    if (y_buffer != 0)
                                        DBG_PRINT("hand model construction: time=%d \r\n", (t4 - t3));
                                    else if (context_fist.tracking_flag != 0)
                                        DBG_PRINT("hand tracking: time=%d \r\n", (t4 - t3));
                                    else
                                        DBG_PRINT("hand release: time=%d \r\n", (t4 - t3));
                                #endif
                            #endif
                        }
                        else
                    #endif
                        {
                            #if PFMST_EN == 1
                                context_fist.detect_total = y_buffer;
                                #if DEMO_DEBUG_EN == 1
                                    t3 = xTaskGetTickCount();
                                #endif
                                PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_fist, &fist_result.rect[0]);
                                #if DEMO_DEBUG_EN == 1
                                    t4 = xTaskGetTickCount();
                                    if (y_buffer != 0)
                                        DBG_PRINT("fist model construction: time=%d \r\n", (t4 - t3));
                                    else if (context_fist.tracking_flag != 0)
                                        DBG_PRINT("fist tracking: time=%d \r\n", (t4 - t3));
                                    else
                                        DBG_PRINT("fist release: time=%d \r\n", (t4 - t3));
                                #endif
                                fist_detect_proc_with_pfmst((ObjDetect_t *)fistWorkMem_ptr,(gpRect *)&fist_result.rect[0],0);
                            #endif
                        }
                    }
                    else
                    {
                        y_buffer = fist_detect_proc_with_pfmst((ObjDetect_t *)fistWorkMem_ptr,0,1);
                        #if PFMST_EN == 1
                                context_fist.detect_total = y_buffer;
                            #if DEMO_DEBUG_EN == 1
                                t3 = xTaskGetTickCount();
                            #endif
                                PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_fist, &fist_result.rect[0]);
                                if(y_buffer)
                                    context_fist.detect_total = 0;
                            #if DEMO_DEBUG_EN == 1
                                t4 = xTaskGetTickCount();
                                if (y_buffer != 0)
                                    DBG_PRINT("fist model construction: time=%d \r\n", (t4 - t3));
                                else if (context_fist.tracking_flag != 0)
                                    DBG_PRINT("fist tracking: time=%d \r\n", (t4 - t3));
                                else
                                    DBG_PRINT("fist release: time=%d \r\n", (t4 - t3));
                            #endif
                        #endif
                    }
                    drv_fd_result_lock();
                    if(y_buffer)
                    {
                        fist_result.is_fist = 1;
                        fd_draw_result.is_fist = fist_result.is_fist;
                        fd_draw_result.rect[FIST_NUM_SET].x = fist_result.rect[0].x;
                        fd_draw_result.rect[FIST_NUM_SET].y = fist_result.rect[0].y;
                        fd_draw_result.rect[FIST_NUM_SET].width = fist_result.rect[0].width;
                        fd_draw_result.rect[FIST_NUM_SET].height = fist_result.rect[0].height;
                        #if PFMST_EN == 1
                            context_fist_result.detect_total = 0;
                            context_fist_result.tracking_flag = context_fist.tracking_flag;
                            trackingSet_fist_result[0].width = (unsigned int)(context_fist.tracked_obj.width);
                            trackingSet_fist_result[0].height = (unsigned int)(context_fist.tracked_obj.height);
                            trackingSet_fist_result[0].x = (unsigned int)(context_fist.tracked_obj.center.x - context_fist.tracked_obj.width/2);
                            trackingSet_fist_result[0].y = (unsigned int)(context_fist.tracked_obj.center.y - context_fist.tracked_obj.height/2);
                        #endif
                        no_fist_cnt = 0;
                    }
                    else
                    {
                        fist_result.is_fist = 0;
                        fd_draw_result.is_fist = fist_result.is_fist;
                        #if PFMST_EN == 1
                            context_fist_result.detect_total = context_fist.detect_total;
                            context_fist_result.tracking_flag = context_fist.tracking_flag;
                            trackingSet_fist_result[0].width = (unsigned int)(context_fist.tracked_obj.width);
                            trackingSet_fist_result[0].height = (unsigned int)(context_fist.tracked_obj.height);
                            trackingSet_fist_result[0].x = (unsigned int)(context_fist.tracked_obj.center.x - context_fist.tracked_obj.width/2);
                            trackingSet_fist_result[0].y = (unsigned int)(context_fist.tracked_obj.center.y - context_fist.tracked_obj.height/2);
                        #endif
                        if((context_fist.detect_total == 0) && (context_fist.tracking_flag != 0))
                            no_fist_cnt = 0;
                        else
                            no_fist_cnt++;
                    }
                    drv_fd_result_unlock();
                }
                else
                {
                    fist_result.is_fist = 0;
                    #if PFMST_EN == 1
                            context_fist.detect_total = fist_result.is_fist;
                        #if DEMO_DEBUG_EN == 1
                            t3 = xTaskGetTickCount();
                        #endif
                            PFMSTracker_framework( (INT8U*)fd_org_buf, ROI_Input.width, ROI_Input.height, &context_fist, &fist_result.rect[0]);
                        #if DEMO_DEBUG_EN == 1
                            t4 = xTaskGetTickCount();
                            if (context_fist.detect_total != 0)
                                DBG_PRINT("fist model construction: time=%d \r\n", (t4 - t3));
                            else if (context_fist.tracking_flag != 0)
                                DBG_PRINT("fist tracking: time=%d \r\n", (t4 - t3));
                            else
                                DBG_PRINT("fist release: time=%d \r\n", (t4 - t3));
                        #endif
                        drv_fd_result_lock();
                        if((context_fist.detect_total == 0) && (context_fist.tracking_flag != 0))
                        {
                            context_fist_result.detect_total = context_fist.detect_total;
                            context_fist_result.tracking_flag = context_fist.tracking_flag;
                            trackingSet_fist_result[0].width = (unsigned int)(context_fist.tracked_obj.width);
                            trackingSet_fist_result[0].height = (unsigned int)(context_fist.tracked_obj.height);
                            trackingSet_fist_result[0].x = (unsigned int)(context_fist.tracked_obj.center.x - context_fist.tracked_obj.width/2);
                            trackingSet_fist_result[0].y = (unsigned int)(context_fist.tracked_obj.center.y - context_fist.tracked_obj.height/2);
                            no_fist_cnt = 0;
                        }
                        else
                        {
                            context_fist_result.detect_total = context_fist.detect_total;
                            context_fist_result.tracking_flag = context_fist.tracking_flag;
                            no_fist_cnt++;
                        }
                        drv_fd_result_unlock();
                    #else
                        no_fist_cnt++;
                    #endif
                }
#endif
                switch(prcess_selection_flag)
                {
                    case MSG_PRCESS_FD:
                        if(hand_detect_en)
                            prcess_selection_state_post(MSG_PRCESS_HAND);
                        else if(fist_detect_en)
                            prcess_selection_state_post(MSG_PRCESS_FIST);
                        else
                            prcess_selection_state_post(MSG_PRCESS_FD);
                        break;
                    case MSG_PRCESS_HAND:
                        if(fist_detect_en)
                            prcess_selection_state_post(MSG_PRCESS_FIST);
                        else
                            prcess_selection_state_post(MSG_PRCESS_FD);
                        break;
                    case MSG_PRCESS_FIST:
                        prcess_selection_state_post(MSG_PRCESS_FD);
                        break;

                    default:
                        prcess_selection_state_post(MSG_PRCESS_FD);
                }
#endif
                fd_state_post(FD_STATE_OK);
                fd_free_frame_buffer_add((INT32U *)fd_org_buf, 1);

        #if CSI_AE_EN == 1
                // set sensor ae
                if(obj_result.is_best_face > 0) {
                    sensor_control_ae();
                }

                if(no_face_cnt >= 16 || no_hand_cnt >= 16 || no_fist_cnt >= 16) {
                    no_face_cnt = 16;
                    no_hand_cnt = 16;
                    no_fist_cnt = 16;
                    sensor_control_ae();
                }
        #endif

                #if FD_SOFTWARE_CNT_EN == 1
                t2 = xTaskGetTickCount();
                DBG_PRINT("F=%d \r\n", (t2 - t1));
                #endif
                fd_gpio1_set(0);
                break;
        }
    }
}

#if VR_EN == 1
static void vr_task_entry(void const *parm)
{
    INT32U state,ack_msg;
    osEvent result;

    DBG_PRINT("vr_task_entry start \r\n");
#if 0
    VrDemo();
#else
    VrDemoGlobalInit() ;
	audio_encode_entrance();
    TestVR_Start() ;
#endif
    while(1)
    {
        result = osMessageGet(vr_state_queue, osWaitForever);
        state = result.value.v;
        if((result.status != osEventMessage) || !state) {
            continue;
        }
        //DBG_PRINT("V");

        switch(state)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_VR_TASK_EXIT]\r\n");
                TestVR_Stop();
                HardwareRelease();
                VrDemoGlobalRelease();
                ack_msg = ACK_OK;
                osMessagePut(vr_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(vr_id);
                break;
        }
    }
}
#endif

static INT32S prcess_result_state_add(INT32U state, INT32U wait)
{
    INT32U event,temp;

    if(wait)
    {
        event = (INT32U)state;
        temp = osMessagePut(prcess_result_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)state;
        temp = osMessagePut(prcess_result_queue, (uint32_t)&event, 10);
    }

	return temp;
}

static INT32S prcess_result_state_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(prcess_result_end, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage)
            frame = 0;
    }
    else
    {
        result = osMessageGet(prcess_result_end, 10);
        frame = result.value.v;
        if(result.status != osEventMessage)
            frame = 0;
    }

	return frame;
}

static void prcess_result_task_entry(void const *parm)
{
    INT32U state,ack_msg;
    osEvent result;

    DBG_PRINT("prcess_result_task_entry start \r\n");

    while(1)
    {
        result = osMessageGet(prcess_result_queue, osWaitForever);
        state = result.value.v;
        if(result.status != osEventMessage) {
            continue;
        }
        //DBG_PRINT("P");

        switch(state)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_PRCESS_RESULT_TASK_EXIT]\r\n");
                ack_msg = ACK_OK;
                osMessagePut(prcess_result_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(prcess_result_id);
                break;

            case MSG_PRCESS_HAND:
                osDelay(500);
                DBG_PRINT("\r\n[Find Hand]\r\n");
                break;
        }
        ack_msg = MSG_PRCESS_END;
        osMessagePut(prcess_result_end, (INT32U)&ack_msg, osWaitForever);
    }
}

void GPM4_FDWA_HAND_VR_Demo(void)
{
#if DEMO_SD_EN == 1
    INT32S ret,temp;
    INT64U disk_free;
#endif
    INT32U exit_cnt,mode_set;
    osThreadDef_t csi_task = {"csi_task", csi_task_entry, osPriorityAboveNormal, 1, 32768};
    osThreadDef_t disp_task = {"disp_task", disp_task_entry, osPriorityNormal, 1, 8192};
    osThreadDef_t prcess_result_task = {"prcess_result_task", prcess_result_task_entry, osPriorityNormal, 1, 32768};
#if FD_EN == 1
    osThreadDef_t fd_task = {"fd_task", fd_task_entry, osPriorityNormal, 1, 65536};
#endif
#if VR_EN == 1
    osThreadDef_t vr_task = {"vr_task", vr_task_entry, osPriorityNormal, 1, 16384};
#endif
    osMessageQDef_t disp_q = {MSG_QUEUE_MAX, sizeof(INT32U), 0};
    osSemaphoreDef_t disp_sem = {0};

#if DEMO_SD_EN == 1
    while(1)
    {
        ret = _devicemount(DISKUSED);
        if(ret)
        {
            DBG_PRINT("Mount Disk Fail[%d]\r\n", DISKUSED);
        #if 0
            ret = _format(DISKUSED, FAT32_Type);
            if(ret)
              DBG_PRINT("Format Disk Fail[%d]\r\n", DISKUSED);
            ret = _deviceunmount(DISKUSED);
            if(ret)
              DBG_PRINT("UnMount Disk Fail[%d]\r\n", DISKUSED);
            ret = _devicemount(DISKUSED);
        #endif
        }
        else
        {
            DBG_PRINT("Mount Disk success[%d]\r\n", DISKUSED);
            disk_free = vfsFreeSpace(DISKUSED);
            DBG_PRINT("DISK FREE SIZE = %lld MByte\r\n", disk_free/1024/1024);
            break;
        }
        osDelay(5);
    }

#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
	if(DISKUSED == FS_SD1){
		ret = chdir("F://");
	}
	else if(DISKUSED == FS_SD){
		ret = chdir("C://");
	}
	else if(DISKUSED == FS_SD2){
		ret = chdir("K://");
	}

	if(ret < 0)
	{
		while(1);
	}

	ret = chdir("ID_data");
	if(ret < 0)
	{
		ret = mkdir("ID_data");
		if(ret < 0)
		{
			while(1);
		}
		ret = chdir("ID_data");
		if(ret < 0)
		{
			while(1);
		}
	}
	DBG_PRINT("Mount ID_data success\r\n");
#endif
#endif

    /* initial prcess parameter set structure */
    prcess_mem_set = (prcess_mem_t *)&prcess_mem_structure;
    gp_memset((INT8S *)prcess_mem_set, 0, sizeof(prcess_mem_t));
    csi_pscaler_stop = 0;
    csi_md_stop = 0;

	// osSemaphoreCreate
	if(sem_pscaler_engine == NULL)
	{
		sem_pscaler_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_pscaler_engine)
		{
            DBG_PRINT("sem_pscaler_engine error\r\n");
            while(1);
		}
		else
            DBG_PRINT("sem_pscaler_engine = 0x%x\r\n", sem_pscaler_engine);
	}

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

	if(sem_track_engine == NULL)
	{
		sem_track_engine = osSemaphoreCreate(&disp_sem, 1);
		if(!sem_track_engine)
            while(1);
		else
            DBG_PRINT("sem_track_engine = 0x%x\r\n",sem_track_engine);
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

#if CSI_FULL_IMAGE_FLOW_EN == 1
    if(pscaler_clip_buffer_queue == NULL)
	{
        pscaler_clip_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!pscaler_clip_buffer_queue)
		{
            DBG_PRINT("pscaler_clip_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("pscaler_clip_buffer_queue = 0x%x\r\n", pscaler_clip_buffer_queue);
	}
#endif

#if DISPLAY_USE_PSCALER_EN == 1
    if(ppu_pscaler_buffer_queue == NULL)
	{
        ppu_pscaler_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!ppu_pscaler_buffer_queue)
		{
            DBG_PRINT("ppu_pscaler_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("ppu_pscaler_buffer_queue = 0x%x\r\n", ppu_pscaler_buffer_queue);
	}
#endif

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

    if(csi_task_ack_m == NULL)
	{
        csi_task_ack_m = osMessageCreate(&disp_q, NULL);
		if(!csi_task_ack_m)
		{
            DBG_PRINT("csi_task_ack_m error\r\n");
            while(1);
		}

		else
            DBG_PRINT("csi_task_ack_m = 0x%x\r\n", csi_task_ack_m);
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
            DBG_PRINT("disp_task_ack_m = 0x%x\r\n", disp_task_ack_m);
	}

    if(prcess_result_task_ack_m == NULL)
	{
        prcess_result_task_ack_m = osMessageCreate(&disp_q, NULL);
		if(!prcess_result_task_ack_m)
		{
            DBG_PRINT("prcess_result_task_ack_m error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_result_task_ack_m = 0x%x\r\n", prcess_result_task_ack_m);
	}

	if(prcess_selection_queue == NULL)
	{
        prcess_selection_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_selection_queue)
		{
            DBG_PRINT("prcess_selection_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_selection_queue = 0x%x\r\n", prcess_selection_queue);
	}

    if(prcess_result_queue == NULL)
	{
        prcess_result_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_result_queue)
		{
            DBG_PRINT("prcess_result_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_result_queue = 0x%x\r\n", prcess_result_queue);
	}

    if(prcess_result_end == NULL)
	{
        prcess_result_end = osMessageCreate(&disp_q, NULL);
		if(!prcess_result_end)
		{
            DBG_PRINT("prcess_result_end error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_result_end = 0x%x\r\n", prcess_result_end);
	}

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
		{
             DBG_PRINT("fd_free_frame_buffer_queue error\r\n");
             while(1);
		}
		else
            DBG_PRINT("fd_free_frame_buffer_queue = 0x%x\r\n",fd_free_frame_buffer_queue);
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
            DBG_PRINT("fd_task_ack_m = 0x%x\r\n", fd_task_ack_m);
	}
#endif

#if VR_EN == 1
	if(vr_state_queue == NULL)
	{
        vr_state_queue = osMessageCreate(&disp_q, NULL);
		if(!vr_state_queue)
		{
            DBG_PRINT("vr_state_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("vr_state_queue = 0x%x\r\n", vr_state_queue);
	}
  	if(vr_task_ack_m == NULL)
	{
        vr_task_ack_m = osMessageCreate(&disp_q, NULL);
		if(!vr_task_ack_m)
		{
            DBG_PRINT("vr_task_ack_m error\r\n");
            while(1);
		}
		else
            DBG_PRINT("vr_task_ack_m = 0x%x\r\n",vr_task_ack_m);
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

#if PPU_DRAW_EN == 1
    fd_ppu_init();
    osDelay(5);
#endif

#if WAFD_DEMO_EN == 0
	// osThreadCreate
    disp_id = osThreadCreate(&disp_task, (void *)NULL);
    if(disp_id == 0)
    {
        DBG_PRINT("osThreadCreate: disp_id error\r\n");
        while(1);
    }
    else
        osDelay(50);
#endif

    csi_id = osThreadCreate(&csi_task, (void *)NULL);
    if(csi_id == 0)
    {
        DBG_PRINT("osThreadCreate: csi_id error\r\n");
        while(1);
    }
    else
        osDelay(50);

    prcess_result_id = osThreadCreate(&prcess_result_task, (void *)NULL);
    if(prcess_result_id == 0)
    {
        DBG_PRINT("osThreadCreate: prcess_result_id error\r\n");
        while(1);
    }
    else
        osDelay(50);

#if FD_EN == 1
    fd_id = osThreadCreate(&fd_task, (void *)NULL);
    if(fd_id == 0) {
        DBG_PRINT("osThreadCreate: fd_id error\r\n");
        while(1);
    }
    else
        osDelay(50);
#endif

#if VR_EN == 1
    vr_id = osThreadCreate(&vr_task, (void *)NULL);
    if(vr_id == 0) {
        DBG_PRINT("osThreadCreate: vr_id error\r\n");
        while(1);
    }
    else
        osDelay(50);
#endif

    // ad key init
	adc_key_scan_init();
    face_recognize_en = 0;
	fd_mode = 0;
	security_level = 3;
	exit_cnt = 0;
	mode_set = 0;
#if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
	train_subject = 0;
#endif

#if FD_EN == 1
    face_detect_en = 0;
#endif

#if HAND_EN == 1
    hand_detect_en = 0;
    fist_detect_en = 0;
#endif

#if WAFD_DEMO_EN == 0

osDelay(500);
#if FD_COMMAND_FLOW_EN == 1
 	DBG_PRINT("\r\n******************************************************\r\n");
	DBG_PRINT(" This is FDWA_RECOGNIZE_HAND_FIST_VR DEMO                **\r\n");
	DBG_PRINT("KEY_1 API_COMMAND_DET_OFF                                **\r\n");
	DBG_PRINT("KEY_2 API_COMMAND_RESULT_DET                             **\r\n");
	DBG_PRINT("KEY_3 API_COMMAND_FACE_DET                               **\r\n");
	DBG_PRINT("KEY_4 API_COMMAND_HAND_FIST_DET WHEN FACE_RECOGNIZE STOP **\r\n");
    #if FACE_RECOGNIZE_MEUN_EN == 1
    DBG_PRINT("KEY_5 FACE_RECOGNIZE START/STOP                          **\r\n");
    DBG_PRINT("KEY_1 FACE_RECOGNIZE START FOR idle mode                 **\r\n");
	DBG_PRINT("KEY_2 FACE_RECOGNIZE START FOR training mode             **\r\n");
	DBG_PRINT("KEY_3 FACE_RECOGNIZE START FOR recognize mode            **\r\n");
    #endif
    DBG_PRINT("*********************************************************\r\n");
#else
	DBG_PRINT("\r\n***************************************************\r\n");
	DBG_PRINT(" This is FDWA_RECOGNIZE_HAND_FIST_VR DEMO         **\r\n");
	DBG_PRINT("KEY_1 FACE_RECOGNIZE START/STOP                   **\r\n");
	DBG_PRINT("KEY_2 HAND DETECTION START/STOP                   **\r\n");
	DBG_PRINT("KEY_3 FIST DETECTION START/STOP                   **\r\n");
	DBG_PRINT("KEY_4 idle mode of FACE_RECOGNIZE or demo exit    **\r\n");
	DBG_PRINT("KEY_5 training mode of FACE_RECOGNIZE             **\r\n");
	DBG_PRINT("KEY_6 recognize mode of FACE_RECOGNIZE            **\r\n");
	DBG_PRINT("KEY_7 security level up of FACE_RECOGNIZE         **\r\n");
	DBG_PRINT("KEY_8 security level down of FACE_RECOGNIZE       **\r\n");
	DBG_PRINT("***************************************************\r\n");
#endif
#endif
#if WAFD_DEMO_EN == 1
    face_detect_en = 1;
#elif 0 // for debug
while(1)
{
    osDelay(5000);
    prcess_task_exit();
    DBG_PRINT("Demo Prcess Exit\r\n");
    break;
}
#else
	while(1)
	{
		adc_key_scan();

		if(ADKEY_IO1)
		{
#if FD_COMMAND_FLOW_EN == 1
        #if FACE_RECOGNIZE_MEUN_EN == 1
            drv_fd_result_lock();
            mode_set = face_recognize_en;
            drv_fd_result_unlock();
            if(mode_set)
            {
                if(fd_mode)
                {
                    drv_fd_result_lock();
                    exit_cnt = 0;
                    fd_mode = 0;
                    fdWorkMem->identify_state = 0;
                    fdWorkMem->training_state = 0;
                    drv_fd_result_unlock();
                    DBG_PRINT("idle mode\r\n");
                }
                else
                {
                    if(++exit_cnt >= 2)
                    {
                        prcess_task_exit();
                        DBG_PRINT("Demo Prcess Exit\r\n");
                        break;
                    }
                }
            }
            else
        #endif
            {
                command_mode = API_COMMAND_DET_OFF;
                face_hand_fist_command_result_set((module_command_t *)module_mem_set, command_mode);
                DBG_PRINT("COMMAND: API_COMMAND_DET_OFF\r\n");
            }
#else
            drv_fd_result_lock();
#if FD_EN == 1
            if(face_detect_en)
            {
                DBG_PRINT("Face Detction Stop\r\n");
                face_detect_en = 0;
            }
            else
            {
                DBG_PRINT("Face Detction Start\r\n");
                face_detect_en = 1;
            }
#endif
            drv_fd_result_unlock();
#endif
            ADKEY_IO1 = 0;
		}

		if(ADKEY_IO2)
		{
#if FD_COMMAND_FLOW_EN == 1
        #if FACE_RECOGNIZE_MEUN_EN == 1
            drv_fd_result_lock();
            mode_set = face_recognize_en;
            drv_fd_result_unlock();
            if(mode_set)
            {
                drv_fd_result_lock();
                fd_mode = 1;
                fdWorkMem->identify_state = 0;
                fdWorkMem->training_cnt = 0;
                #if FACE_RECOGNIZE_MULTI_PEOPLE_FLOW_EN == 1
                    if(train_subject > (fidInfo.MAX_SUBJECT_NUM - 1))
                        train_subject = 0;
                #endif
                drv_fd_result_unlock();
                DBG_PRINT("training mode\r\n");
            }
            else
        #endif
            {
                command_mode = API_COMMAND_RESULT_DET;
                face_hand_fist_command_result_set((module_command_t *)module_mem_set, command_mode);
                DBG_PRINT("COMMAND: API_COMMAND_RESULT_DET\r\n");
            }
#else
            drv_fd_result_lock();
#if HAND_EN == 1
            if(hand_detect_en)
            {
                DBG_PRINT("Hand Detction Stop\r\n");
                hand_detect_en = 0;
            }
            else
            {
                DBG_PRINT("Hand Detction Start\r\n");
                hand_detect_en = 1;
            }
#endif
            drv_fd_result_unlock();
#endif
            ADKEY_IO2 = 0;
		}

		if(ADKEY_IO3)
		{
#if FD_COMMAND_FLOW_EN == 1
        #if FACE_RECOGNIZE_MEUN_EN == 1
            drv_fd_result_lock();
            mode_set = face_recognize_en;
            drv_fd_result_unlock();
            if(mode_set)
            {
                drv_fd_result_lock();
                fd_mode = 2;
                fdWorkMem->identify_state = 0;
                if(train_max_flag)
                    fidInfo.subject_num = fidInfo.MAX_SUBJECT_NUM;
                drv_fd_result_unlock();
                DBG_PRINT("recognize mode\r\n");
            }
            else
        #endif
            {
                command_mode = API_COMMAND_FACE_DET;
                face_hand_fist_command_result_set((module_command_t *)module_mem_set, command_mode);
                DBG_PRINT("COMMAND: API_COMMAND_FACE_DET\r\n");

            }
#else
            drv_fd_result_lock();
#if HAND_EN == 1
            if(fist_detect_en)
            {
                DBG_PRINT("Fist Detction Stop\r\n");
                fist_detect_en = 0;
            }
            else
            {
                DBG_PRINT("Fist Detction Start\r\n");
                fist_detect_en = 1;
            }
#endif
            drv_fd_result_unlock();
#endif
            ADKEY_IO3 = 0;
		}

        if(ADKEY_IO4)
        {
#if FD_COMMAND_FLOW_EN == 1
        #if FACE_RECOGNIZE_MEUN_EN == 1
            drv_fd_result_lock();
            mode_set = face_recognize_en;
            drv_fd_result_unlock();
            if(mode_set == 0)
        #endif
            {
                command_mode = API_COMMAND_HAND_FIST_DET;
                face_hand_fist_command_result_set((module_command_t *)module_mem_set, command_mode);
                DBG_PRINT("COMMAND: API_COMMAND_HAND_FIST_DET\r\n");
            }
#endif
            ADKEY_IO4 = 0;
        }

#if FD_COMMAND_FLOW_EN == 1
        if(ADKEY_IO5)
        {
            drv_fd_result_lock();
            if(face_recognize_en)
                face_recognize_en = 0;
            else
                face_recognize_en = 1;
            drv_fd_result_unlock();
            if(face_recognize_en)
                DBG_PRINT("Face Recognize Start\r\n");
            else
                DBG_PRINT("Face Recognize Stop\r\n");
            ADKEY_IO5 = 0;
        }

        if(ADKEY_IO6)
        {
            drv_fd_result_lock();
            mode_set = face_recognize_en;
            drv_fd_result_unlock();
            if(mode_set)
            {
                if(fidInfo.subject_num)
                {
                    FaceIdentify_Delete((FID_Info *)&fidInfo, train_subject);
                    Delete_FDData_InfoUpdate((fidInfo.subject_num+1), (train_subject+1));
                    if(fidInfo.subject_num != fidInfo.MAX_SUBJECT_NUM)
                        train_max_flag = 0;
                    else
                        train_max_flag = 1;
                    DBG_PRINT("Delete %d pepole data\r\n", (train_subject+1));
                    train_subject = fidInfo.subject_num;
                    DBG_PRINT("Total %d pepole data inside\r\n", train_subject);
                    drv_fd_result_lock();
                    fdWorkMem->training_subject_cnt = train_subject;
                    drv_fd_result_unlock();
                }
                else
                    DBG_PRINT("No people to complete face training flow\r\n");
            }
            ADKEY_IO6 = 0;
        }

        if(ADKEY_IO7)
        {
            drv_fd_result_lock();
            mode_set = face_recognize_en;
            drv_fd_result_unlock();
            if(mode_set)
            {
                if(train_subject >= 0)
                {
                    train_subject--;
                    if(train_subject < 0)
                        train_subject = (fidInfo.subject_num - 1);

                    if(train_subject < 0)
                    {
                        DBG_PRINT("No people to complete face training flow\r\n");
                        train_subject = 0;
                    }
                    else
                        DBG_PRINT("Master Will Delete %d pepole data\r\n", (train_subject+1));
                }
                else
                    DBG_PRINT("No people to complete face training flow\r\n");
            }
            ADKEY_IO7 = 0;
        }
#else

        if(ADKEY_IO4)
        {
            drv_fd_result_lock();
            if(face_recognize_en)
                face_recognize_en = 0;
            else
                face_recognize_en = 1;
            drv_fd_result_unlock();
            if(face_recognize_en)
                DBG_PRINT("Face Recognize Stop\r\n");
            else
                DBG_PRINT("Face Recognize Start\r\n");
            ADKEY_IO4 = 0;
        }

        if(face_recognize_en)
        {
            if(ADKEY_IO5)
            {
                if(fd_mode)
                {
                    drv_fd_result_lock();
                    exit_cnt = 0;
                    fd_mode = 0;
                    fdWorkMem->identify_state = 0;
                    fdWorkMem->training_state = 0;
                    drv_fd_result_unlock();
                    DBG_PRINT("idle mode\r\n");
                }
                else
                {
                    if(++exit_cnt >= 2)
                    {
                        prcess_task_exit();
                        DBG_PRINT("Demo Prcess Exit\r\n");
                        break;
                    }
                }
                ADKEY_IO5 = 0;
            }

            if(ADKEY_IO6)
            {
                drv_fd_result_lock();
                fd_mode = 1;
                fdWorkMem->identify_state = 0;
                fdWorkMem->training_state = 0;
                drv_fd_result_unlock();
                DBG_PRINT("training mode\r\n");
                ADKEY_IO6 = 0;
            }

            if(ADKEY_IO7)
            {
                drv_fd_result_lock();
                fd_mode = 2;
                fdWorkMem->identify_state = 0;
                drv_fd_result_unlock();
                DBG_PRINT("recognize mode\r\n");
                ADKEY_IO7 = 0;
            }

            if(ADKEY_IO8)
            {
                drv_fd_result_lock();
                if(security_level > 5)
                    security_level = 0;
                else
                    security_level++;
                if(fdWorkMem)
                    face_recognize_set_security_level((void *)fdWorkMem,security_level);
                drv_fd_result_unlock();
                DBG_PRINT("security_level = %d\r\n",security_level);
                ADKEY_IO8 = 0;
            }
        }
        else
        {
            if(ADKEY_IO4 || ADKEY_IO5 || ADKEY_IO6 || ADKEY_IO7 || ADKEY_IO8)
                DBG_PRINT("Face Recognize Prcess Stop\r\n");
        }
#endif
	}
#endif
	adc_key_scan_uninit();
#if DEMO_SD_EN == 1
    _deviceunmount(FS_SD2);
#endif
}



