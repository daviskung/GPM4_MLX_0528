#include <string.h>
#include "avi_encoder_app.h"
#include "drv_l1_gpio.h"
#include "drv_l1_dma.h"
#include "drv_l1_scaler.h"
#include "drv_l2_scaler.h"
#include "drv_l1_pscaler.h"
#include "define.h"
#include "drv_l1_cdsp.h"
#include "drv_l2_cdsp.h"
#include "gplib_jpeg_encode.h"
#include "drv_l1_rotator.h"
#include "drv_l2_display.h"
#include "drv_l2_sensor.h"
/* wifi demo use */
#include "gspi_master_drv.h"
#include "wifi_demo.h"
/* PPU draw use */
#include "gplib_ppu.h"
#include "gplib_ppu_dma.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "SPRITE_object_HDR.h"
/* Thermopile Array */
#include "defs_th32x32.h"
#include "drv_l1_i2c.h"
#include <math.h>		// for pow()
//#include "TABLE113.h"	// for FOV = 33
//#include "TABLE114.h"	// for FOV = 90
#ifdef HTPA32x32dR1L1k8_0k7HiGe_Bodo
#include "TABLE115.h"	// for FOV = 105
#endif

#ifdef HTPA32x32dR1L5_0HiGeF7_7_Gain3k3
#include "TABLE113.h"	// for FOV = 33
#endif

#ifdef HTPA32x32dL2_1HiSiF5_0_Gain3k3
#include "TABLE114.h"	// for FOV = 90
#endif




#include "32X32RGB565NEW.h" 	//  davis 2019.04.23




#if (defined APP_VIDEO_ENCODER_EN) && (APP_VIDEO_ENCODER_EN == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#ifndef APP_QRCODE_BARCODE_EN
#define APP_QRCODE_BARCODE_EN   0
#endif

#if APP_QRCODE_BARCODE_EN == 1
#include "task_code_decoder.h"
#endif

#define C_SCALER_STACK_SIZE			2048
#define	C_JPEG_STACK_SIZE			2048
#define C_SCALER_QUEUE_MAX			6
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
#define C_JPEG_QUEUE_MAX			16
#else
#define C_JPEG_QUEUE_MAX			6
#endif
#define C_ACK_QUEUE_MAX				1
#define C_ENCODE_5M_CNTOUT          200

#if (PALM_DEMO_EN==1)||(PATTERN_DEMO_EN==1)||(WAFD_DEMO_EN==1)
/* PPU  draw use */
#define C_PPU_DRAW_EN               1
#define C_PPU_SPRITE_EN             1
#define C_PPU_UI_EN                 1
#endif
#define C_PPU_DRV_FRAME_NUM		    3
#define FRAME_BUF_ALIGN64           0x3F
#define FRAME_BUF_ALIGN32           0x1F
#define FRAME_BUF_ALIGN16           0xF
#define FRAME_BUF_ALIGN8            0x7
#define FRAME_BUF_ALIGN4            0x3

const INT16U Blk_start_ary[4]={992,960,928,896}; // 改成 TH32X32

#define TRANSPARENT_COLOR 	0x00	// 無色 
#define DBG_TOBJ			0
#define DBG_COLOR_TABLE		0

#define TH32x32_ReadStatus_WaitTime	2
#define CHECK_ReadStatus_WAITTIME	0
#define SHOWTEMP_OFFSET				0
#define TH32x32IMAGE				1
#define TH32x32_FUN					1

#define CORE_AREA_limit		4
#define SENSOR_AREA_WIDTH	32
#define SENSOR_AREA_HIGH	32


typedef struct {
	INT32U ppu_frame_workmem;
	INT32U ppu_narray_workmem;
    INT32U prcess_post_workmem;
    INT32U disp_prcess_workmem;
} prcess_mem_t;

static prcess_mem_t *prcess_mem_set;


const INT16U ColorTable_HOT[10]={
0xFFFF,	// 白 
//0xffe0,0xffea,0xffec,0xffee,0xfff0,   //黃 
0xFFFF,	// 白 
0xffea,0xffec,0xffee,0xfff0,   //黃 
0x5ff0, // 淡綠 
0x1fea,//淺 綠 
0xfa08,	//淡 紅 
0xf8e3,// 淺紅 
};


const INT16U ColorTable_COLD[10]={
0xc61f,0xb5bf,0xad7f,0xa53f,0x8c7f,	// 淡藍 
0xb59f,
0x421f,
0x295f,
0x10bf,
0x001f};		//30-39


typedef struct
{
	INT32U center_x;
	INT32U center_y;
	INT32U recong_w;
	INT32U recong_h;
	INT32U recong_id;
} objRecongResult;

extern objRecongResult obj_recong_draw;

INT32U obj_id,pos_x,pos_y;

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef enum
{
	MSG_SCALER_TASK_INIT = 0x2000,
	MSG_SCALER_TASK_STOP,
	MSG_SCALER_TASK_EXIT,
//	MSG_SCALER_TASK_TH32x32Ready,
	MSG_ROTATE_TASK_STOP = 0x2100,
	MSG_ROTATE_TASK_EXIT
} AVI_ENCODE_SCALER_ENUM;

typedef enum
{
	MSG_TH32x32_TASK_INIT = 0x2200,
	MSG_TH32x32_TASK_STOP,
	MSG_TH32x32_TASK_EXIT
} TH32x32_SENSOR_ENUM;

typedef enum
{
	MSG_TH32x32_SCALERUP_TASK_INIT = 0x2300,
	MSG_TH32x32_SCALERUP_TASK_STOP,
	MSG_TH32x32_SCALERUP_TASK_EXIT
} TH32x32_SCALERUP_ENUM;


typedef enum
{
	MSG_VIDEO_ENCODE_TASK_INIT = 0x3000,
	MSG_VIDEO_ENCODE_TASK_STOP,
	MSG_VIDEO_ENCODE_TASK_EXIT,
	MSG_VIDEO_ENCODE_TASK_RESET
} AVI_ENCODE_VIDEO_ENUM;

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
static void scaler_task_entry(void const *parm);
static void video_encode_task_entry(void const *parm);
static void rotate_task_entry(void const* param);
void scaler_video_init();
static void TH32x32_task_entry(void const *parm);
static void TH32x32_SCALERUP_task_entry(void const *parm);


/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
osMessageQId scaler_task_q = NULL;
osMessageQId scaler_task_ack_m = NULL;
osMessageQId scaler_frame_q = NULL;
osMessageQId rotate_msg_q = NULL;
osMessageQId rotate_ack_m = NULL;
osMessageQId rotate_frame_q = NULL;
osMessageQId display_frame_q = NULL;
osMessageQId uvc_frame_q = NULL;
osMessageQId cmos_frame_q = NULL;
osMessageQId vid_enc_task_q = NULL;
osMessageQId vid_enc_frame_q = NULL;
osMessageQId video_frame_q = NULL;
osMessageQId vid_enc_task_ack_m = NULL;
osMessageQId video_stream_q = NULL;
osMessageQId jpeg_fifo_q = NULL;

osMessageQId TH32x32_task_q = NULL;
osMessageQId TH32x32_task_ack_m = NULL;

osMessageQId TH32x32_SCALERUP_task_q = NULL;
osMessageQId TH32x32_SCALERUP_task_ack_m = NULL;
osMessageQId TH32x32_SCALERUP_buf_q = NULL;


#if VIDEO_TIMESTAMP
osMessageQId frame_ts_q = NULL;
#endif
#if (PALM_DEMO_EN==1)||(PATTERN_DEMO_EN==1)||(WAFD_DEMO_EN==1)
extern xQueueHandle disp_frame_buffer_queue;
extern xQueueHandle csi_frame_buffer_queue;//
extern xQueueHandle display_frame_buffer_queue2;
extern xQueueHandle fd_dma_frame_buffer_queue;
extern xQueueHandle prcess_draw_queue;
extern xQueueHandle pscaler_frame_buffer_queue;
#endif
volatile INT32S pscaler_exit_0, pscaler_exit_1, pscaler_start_1 = 0, encode_5M_flag = 0;
INT32U pscaler_zoom[2] = { 10, 10};
INT32U (*videnc_display)(INT16U w, INT16U h, INT32U frame_addr);
INT32U (*videnc_buferr_post)(INT32U frame_addr);
INT32U encode_time;
#if C_PPU_DRAW_EN == 1
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static INT32U disp_h_size, disp_v_size;
static PPU_FRAME_BUFFER_BASE, text1_narray;
const INT32U* OBJ_SP_NUMBER_POOL[]={
 	(INT32U *)_Sprite001_IMG0000_CellIdx,
 	(INT32U *)_Sprite001_IMG0001_CellIdx,
 	(INT32U *)_Sprite001_IMG0002_CellIdx,
 	(INT32U *)_Sprite001_IMG0003_CellIdx,
 	(INT32U *)_Sprite001_IMG0004_CellIdx,
 	(INT32U *)_Sprite001_IMG0005_CellIdx,
 	(INT32U *)_Sprite001_IMG0006_CellIdx,
 	(INT32U *)_Sprite001_IMG0007_CellIdx,
 	(INT32U *)_Sprite001_IMG0008_CellIdx,
 	(INT32U *)_Sprite001_IMG0009_CellIdx
};
#if C_PPU_UI_EN == 1
    ppu_buffer_info_t buffer_info;
#endif
#endif
/* wifi demo use start*/
UPDATE_CUR_JPEG update_cur_jpeg = NULL;
void register_update_cur_jpeg_cbk(INT32U cbk)
{
	update_cur_jpeg = (UPDATE_CUR_JPEG)cbk;
}
/* wifi demo use end */
#if C_PPU_DRAW_EN == 1
/* ppu draw use start*/
static void avi_ppu_draw_init(INT32U avi_h_size, INT32U avi_v_size)
{
    #define DUAL_SIZE_USE                  0
    #define C_PPU_COLOR_MODE1              PPU_FMT_YUYV
    #define C_PPU_COLOR_MODE2              PPU_FMT_YUYV
    INT32U i,frame_size,buffer_ptr;
#if DUAL_SIZE_USE == 1
    FB_LOCK_STRUCT fb_lock_set;
#endif

    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;
    gp_memset((INT8S *)ppu_register_set, 0 , sizeof(PPU_REGISTER_SETS));

#if C_PPU_UI_EN == 1
    frame_buffer_ppu_update_init(ppu_register_set);
#else
    // var init
    PPU_FRAME_BUFFER_BASE = 0;
    text1_narray = 0;

    //Initiate PPU hardware engine and PPU register set structure
    gplib_ppu_init(ppu_register_set);
#if DUAL_SIZE_USE == 1
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&disp_h_size, (INT16U *)&disp_v_size);
    fb_lock_set.color1 = C_PPU_COLOR_MODE1;
    fb_lock_set.h_size1 = avi_h_size;
    fb_lock_set.v_size1 = avi_v_size;
    fb_lock_set.color2 = C_PPU_COLOR_MODE2;
    fb_lock_set.h_size2 = disp_h_size;
    fb_lock_set.v_size2 = disp_v_size;
    gplib_ppu_fb_lock_process_enable_set(ppu_register_set,(FB_LOCK_STRUCT *)&fb_lock_set);
#if 0
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
    gplib_ppu_free_size_set(ppu_register_set, 0, avi_h_size, avi_v_size);
    gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
    gplib_ppu_long_burst_set(ppu_register_set, 1);

    //Frame buffer malloc
    frame_size = (avi_h_size * avi_v_size * 2);
    PPU_buffer_ptr = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(PPU_buffer_ptr == 0)
    {
        DBG_PRINT("PPU_buffer_ptr fail\r\n");
        while(1);
    }
    PPU_buffer_ptr = (INT32U)((PPU_buffer_ptr + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(PPU_buffer_ptr + (i*frame_size));
            gplib_ppu_frame_buffer_add(ppu_register_set, buffer_ptr);
            DBG_PRINT("PPUBuffer:0x%X \r\n",buffer_ptr);
    }

    // Now configure TEXT relative elements
    gplib_ppu_text_compress_disable_set(ppu_register_set, 1);	                    // Disable TEXT1/TEXT2 horizontal/vertical compress function
    gplib_ppu_text_direct_mode_set(ppu_register_set, 0);			                // Disable TEXT direct address mode

    //text 2 2D
    gplib_ppu_text_init(ppu_register_set, C_PPU_TEXT1);
    text1_narray = (INT32U)gp_malloc_align(4096+64,4);
    text1_narray = (INT32U)((text1_narray + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    gplib_ppu_text_number_array_ptr_set(ppu_register_set, C_PPU_TEXT1, (INT32U)text1_narray);	 // Set TEXT number array address
    gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT1, 1);	                        // Enable TEXT
    gplib_ppu_yuv_type_set(ppu_register_set, 3);								     // Set 32-bit color format to Y1UY0V
    gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT1, 1);			     // Enable bitmap mode
    gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT1, 1);	    // Get TEXT attribute from register
    gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT1, 1, 3);				     // Set TEXT color to YUYV
    gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 0);			             // Set TEXT size to 1024x512
    gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT1, 0);				    // Set TEXT segment address
 	#if C_PPU_SPRITE_EN == 1
		gplib_ppu_sprite_enable_set(ppu_register_set, 1);			                     	// Disable Sprite
		gplib_ppu_sprite_coordinate_set(ppu_register_set, 0);                          // set sprite coordinate
		gplib_ppu_sprite_direct_mode_set(ppu_register_set, 0);		                 // Set sprite address mode
		gplib_ppu_sprite_number_set(ppu_register_set, 256);                             // Set sprite number
		gplib_ppu_sprite_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteRAM);   // set sprite ram buffer
		gplib_ppu_sprite_extend_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteExRAM); // value: 32-bit pointer to sprite extend attribute ram
	    gplib_ppu_sprite_segment_set(ppu_register_set, (INT32U)_SPRITE_object_CellData);      // sprite cell data
    	set_sprite_init(0 ,(INT32U)&Sprite001_SP);
        set_sprite_init(1 ,(INT32U)&Sprite001_SP);
        set_sprite_init(2 ,(INT32U)&Sprite001_SP);
    #else // Disable Sprite
		gplib_ppu_sprite_init(ppu_register_set);
		gplib_ppu_sprite_enable_set(ppu_register_set, 0);
	#endif
#endif
}

static void avi_ppu_draw_uninit(void)
{
    if(ppu_register_set)
        gp_memset((INT8S *)ppu_register_set, 0 , sizeof(PPU_REGISTER_SETS));

#if C_PPU_UI_EN == 1
    frame_buffer_ppu_update_uninit();
#else
    if(PPU_FRAME_BUFFER_BASE)
    {
        gp_free((void *)PPU_FRAME_BUFFER_BASE);
        PPU_FRAME_BUFFER_BASE = 0;
    }

    if(text1_narray)
    {
        gp_free((void *)text1_narray);
        text1_narray = 0;
    }
#endif
}

static INT32S avi_ppu_ui_draw_object_mode(INT32U enable, INT32U image_num, INT32U h_size, INT32U v_size, INT32U tar_buf)
{
    #define SPRITE_64_EN                1
    INT32U temp,temp1,src_buffer,tar_buffer;
#if SPRITE_64_EN == 1
    INT32U bsae_buf = (INT32U)_SPRITE_object_CellData+(64*64*2);
#else
    INT32U bsae_buf = (INT32U)_SPRITE_object_CellData+(32*32*2);
#endif
    if(enable)
    {
        buffer_info.buffer_color_mode = COLOR_YUYV;
        buffer_info.flip_enable = 0;
        buffer_info.transparent_enable = 0;
        buffer_info.transparent_mode = 0;
        buffer_info.transparent_color = 0xfffd;
        buffer_info.blend_enable = 0;
        buffer_info.blend_value = 32;
        buffer_info.t_width = h_size;
        buffer_info.t_height = v_size;
#if SPRITE_64_EN == 1
        buffer_info.s_width = 64;
        buffer_info.s_height = 64;
#else
        buffer_info.s_width = 32;
        buffer_info.s_height = 32;
#endif
#if 0
        temp = (image_num / 100);
        //set_sprite_display_init(0,240,16,(INT32U)OBJ_SP_NUMBER_POOL[temp]);
        //src_buffer = (INT32U)OBJ_SP_NUMBER_POOL[temp];
        src_buffer = bsae_buf+(temp*32*32*2);
        buffer_info.t_addr = tar_buf;
        buffer_info.s_addr = src_buffer;
        frame_buffer_ppu_update_info(ppu_register_set,(ppu_buffer_info_t *)&buffer_info);
        frame_buffer_ppu_update_go(ppu_register_set, tar_buf, 544, 16);
        frame_buffer_ppu_update_state_get();
        temp1 = temp * 100;
        temp = (image_num-temp1);
        temp1 = (temp / 10);
        //src_buffer = (INT32U)OBJ_SP_NUMBER_POOL[temp1];
        src_buffer = bsae_buf+(temp1*32*32*2);
        buffer_info.t_addr = tar_buf;
        buffer_info.s_addr = src_buffer;
        frame_buffer_ppu_update_info(ppu_register_set,(ppu_buffer_info_t *)&buffer_info);
        frame_buffer_ppu_update_go(ppu_register_set, tar_buf, 576, 16);
        frame_buffer_ppu_update_state_get();
        //set_sprite_display_init(1,264,16,(INT32U)OBJ_SP_NUMBER_POOL[temp1]);
#endif
        //temp1 = (temp % 10);
        temp1 = (image_num % 10);
        //set_sprite_display_init(2,288,16,(INT32U)OBJ_SP_NUMBER_POOL[temp1]);
        //src_buffer = (INT32U)OBJ_SP_NUMBER_POOL[temp1];
#if SPRITE_64_EN == 1
        src_buffer = bsae_buf+(temp1*64*64*2);
#else
        src_buffer = bsae_buf+(temp1*32*32*2);
#endif
#if 0
        //tar_buffer = (tar_buf + ((h_size * pos_y * 2) + (pos_x * 2)));
        tar_buffer = tar_buf;
        drv_l1_dma_buffer_copy(src_buffer, tar_buffer, (32*32*2), 32*2, h_size*2);
#else

        buffer_info.t_addr = tar_buf;
        buffer_info.s_addr = src_buffer;
        frame_buffer_ppu_update_info(ppu_register_set,(ppu_buffer_info_t *)&buffer_info);
        if(pos_x && pos_y)
        {
            temp = ((pos_x + FRAME_BUF_ALIGN4) & ~FRAME_BUF_ALIGN4);
            temp1 = ((pos_y + FRAME_BUF_ALIGN4) & ~FRAME_BUF_ALIGN4);
            //DBG_PRINT("x,y %d %d %x\r\n",pos_x,pos_y,tar_buf);
            frame_buffer_ppu_update_go(ppu_register_set, tar_buf, temp, temp1);
            frame_buffer_ppu_update_state_get();
        }
#endif
    }
}

static INT32S avi_ppu_sprite_draw_object_mode(INT32U enable, INT32U image_num)
{
    INT32U temp,temp1;

    if(enable == 0)
    {
        set_sprite_display_init(0,540,16,0);
        set_sprite_display_init(1,564,16,0);
        set_sprite_display_init(2,588,16,0);
    }
    else
    {
        temp = (image_num / 100);
        set_sprite_display_init(0,540,16,(INT32U)OBJ_SP_NUMBER_POOL[temp]);
        temp1 = temp * 100;
        temp = (image_num-temp1);
        temp1 = (temp / 10);
        set_sprite_display_init(1,564,16,(INT32U)OBJ_SP_NUMBER_POOL[temp1]);
        temp1 = (temp % 10);
        set_sprite_display_init(2,588,16,(INT32U)OBJ_SP_NUMBER_POOL[temp1]);
    }

    paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_640X480,LeftTop2Center_coordinate,10);
}

static void avi_ppu_draw_go(INT32U x, INT32U y, INT32U frame_buffer)
{
    gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, x, y, (INT32U)frame_buffer);	// Calculate Number array
    // Start PPU and wait until PPU operation is done
    gplib_ppu_go_and_wait_done(ppu_register_set);
}
#endif
/* ppu draw use end */


///////////////////////////////////////////////////////////////////////////////////////////////////////
// TH32x32 I2C calc data task
///////////////////////////////////////////////////////////////////////////////////////////////////////


static void TH32x32_start_timer_isr(void)
{
	INT8U err;
	INT32U frame;


	pTH32x32_Para->TH32x32_sampleCnt ++;



	//if ( pTH32x32_Para->TH32x32_sampleCnt > 10 ){	// per sec
	//if (( pTH32x32_Para->TH32x32_sampleCnt > 5 )&&(pTH32x32_Para->TH32x32_sample_startON == 0)) {	// per 500ms
	//if (( pTH32x32_Para->TH32x32_sampleCnt > 2 )&&(pTH32x32_Para->TH32x32_sample_startON == 0)) {	// per 200ms

	if ( pTH32x32_Para->TH32x32_sampleCnt > 500 ){	// per 5 sec
		pTH32x32_Para->TH32x32_ReadElecOffset_TA_startON = 1;
		pTH32x32_Para->TH32x32_sampleCnt = 0;


	}


	if( pTH32x32_Para->TH32x32_readout_block_startON == 0 ){


		frame = avi_encode_get_empty(TH32x32_SCALERUP_buf_q);
		if(frame == 0)
				DEBUG_MSG("L->TH32x32");
		else{

			//DEBUG_MSG("davis -->frame = 0x%x in csi_eof_isr \r\n",frame );
			avi_encode_post_empty(TH32x32_task_q,frame);

			pTH32x32_Para->TH32x32_readout_block_startON = 1;
		}

	}
}




// TH32x32_SCALERUP task
INT32S TH32x32_SCALERUP_Task_create(INT8U pori)
{
	INT32S nRet;
	osThreadId id;
	osThreadDef_t TH32x32_SCALERUP_Task = { "TH32x32_SCALERUP_task", TH32x32_SCALERUP_task_entry, osPriorityNormal, 1, C_SCALER_STACK_SIZE };

	if(TH32x32_SCALERUP_task_q == 0) {
		osMessageQDef_t TH32x32_scalar_q = {C_SCALER_QUEUE_MAX*2, sizeof(INT32U), 0}; //queue size double for possible null frame

		TH32x32_SCALERUP_task_q = osMessageCreate(&TH32x32_scalar_q, NULL);
		if(TH32x32_SCALERUP_task_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(TH32x32_SCALERUP_task_ack_m == 0) {
		osMessageQDef_t TH32x32_scalar_ack_q = {C_ACK_QUEUE_MAX, sizeof(INT32U), 0};

		TH32x32_SCALERUP_task_ack_m = osMessageCreate(&TH32x32_scalar_ack_q, NULL);
		if(TH32x32_SCALERUP_task_ack_m == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(TH32x32_SCALERUP_buf_q == 0) {
		osMessageQDef_t TH32x32_scalar_f_q = {TH32x32_SCALERUP_BUFFER_NO, sizeof(INT32U), 0};

		TH32x32_SCALERUP_buf_q = osMessageCreate(&TH32x32_scalar_f_q, NULL);
		if(TH32x32_SCALERUP_buf_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	id = osThreadCreate(&TH32x32_SCALERUP_Task, (void *)NULL);
    if(id == 0) {
        RETURN(STATUS_FAIL);
    }

	nRet = STATUS_OK;
Return:
	return nRet;
}

static void TH32x32_SCALERUP_task_entry(void const *parm)
{
	INT32U msg_id,ready_buf, ack_msg;
	//INT32U sensor_frame, scaler_frame, display_frame;
	ScalerFormat_t scale;
	ScalerPara_t para;
	osEvent result;
	osThreadId id;
	INT32S  nRet;


    DEBUG_MSG("<<%s>>\r\n", __func__);

	while(1)
	{
		result = osMessageGet(TH32x32_SCALERUP_task_q, osWaitForever);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (	msg_id == 0)) {
			continue;
		}

		switch(msg_id)
		{
		case MSG_TH32x32_SCALERUP_TASK_INIT:
			DEBUG_MSG("[MSG_TH32x32_SCALERUP_TASK_INIT]\r\n");

			OSQFlush(TH32x32_SCALERUP_task_q);

			ack_msg = ACK_OK;
			osMessagePut(TH32x32_SCALERUP_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;

		case MSG_TH32x32_SCALERUP_TASK_STOP:
			DEBUG_MSG("[MSG_TH32x32_SCALERUP_TASK_STOP]\r\n");
			OSQFlush(TH32x32_SCALERUP_task_q);
			ack_msg = ACK_OK;
			osMessagePut(TH32x32_SCALERUP_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;

		case MSG_TH32x32_SCALERUP_TASK_EXIT:
			DEBUG_MSG("[MSG_TH32x32_SCALERUP_TASK_EXIT]\r\n");

			ack_msg = ACK_OK;
			osMessagePut(TH32x32_SCALERUP_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			id = osThreadGetId();
    		osThreadTerminate(id);
			break;

		default:
			ready_buf = msg_id;

			if( ready_buf != pTH32x32_Para->TH32x32_ColorOutputFrame_addr ){
				DEBUG_MSG("SCALERUP get wrong frame Fail!!!\r\n");
				goto DEFAULT_END ;
			}
			if(	pTH32x32_Para->TH32x32_ScalerUp_status == 1 ) {
				DEBUG_MSG("SCALERUP too fast !!!\r\n");
				goto DEFAULT_END ;
			}

		#if 1
		//
		// TH32x32 sensor scaler -[ first time]
		//

			gp_memset((INT8S *)&scale, 0x00, sizeof(scale));
			//drv_l2_scaler_init();
			//scale.input_format = pAviEncVidPara->sensor_output_format;
			scale.input_format = C_SCALER_CTRL_IN_RGB565;
			scale.input_width = pTH32x32_Para->TH32x32_width;
			scale.input_height = pTH32x32_Para->TH32x32_height;
			scale.input_visible_width = pTH32x32_Para->TH32x32_width;
			scale.input_visible_height = pTH32x32_Para->TH32x32_height;
			scale.input_x_offset = 0;
			scale.input_y_offset = 0;

			//scale.input_y_addr = sensor_frame;
			scale.input_y_addr =pTH32x32_Para->TH32x32_ColorOutputFrame_addr ;
			scale.input_u_addr = 0;
			scale.input_v_addr = 0;
			#if 1
			scale.input_b_y_addr =0;
			scale.input_b_u_addr = 0;
			scale.input_b_v_addr = 0;
			#endif

			scale.output_format = C_SCALER_CTRL_OUT_YUYV;
			scale.output_width = pAviEncVidPara->display_width;
			scale.output_height = pAviEncVidPara->display_height;
			scale.output_buf_width = pAviEncVidPara->display_width;
			scale.output_buf_height = pAviEncVidPara->display_height;
			scale.output_x_offset = 0;

			//scale.output_y_addr = scaler_frame;
			scale.output_y_addr = pTH32x32_Para->TH32x32_display_frame;
			scale.output_u_addr = 0;
			scale.output_v_addr = 0;


			scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
			//scale.scale_mode = C_SCALER_FULL_SCREEN;
			scale.scale_mode = C_SCALER_BY_RATIO;
			scale.digizoom_m = 10;
			scale.digizoom_n = 10;
			//#endif
			memset((void *)&para, 0x00, sizeof(para));
			para.boundary_color = 0x008080;

    		nRet = drv_l2_scaler_trigger(SCALER_0, 1, &scale, &para);
			if(nRet == C_SCALER_STATUS_DONE || nRet == C_SCALER_STATUS_STOP) {
				drv_l2_scaler_stop(SCALER_0);
			}
			else {
				DBG_PRINT("Scale1 Fail\r\n");
			while(1);
			}
		#endif

			pTH32x32_Para->TH32x32_ScalerUp_status = 1;

		DEFAULT_END:

			break;
		}
	}
}


INT32S TH32x32_SCALERUP_task_start(void)
{
	INT32S i, nRet;

	//OSQFlush(scaler_frame_q);

	nRet = STATUS_OK;
	POST_MESSAGE(TH32x32_SCALERUP_task_q, MSG_TH32x32_SCALERUP_TASK_INIT, TH32x32_SCALERUP_task_ack_m, 5000);
Return:
	return nRet;
}


INT32S TH32x32_SCALERUP_task_stop(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(TH32x32_SCALERUP_task_q, MSG_TH32x32_SCALERUP_TASK_STOP, TH32x32_SCALERUP_task_ack_m, 5000);

Return:
	return nRet;
}



/********************************************************************
 * Function:        void calcTO(unsigned int TAmb, signed int dig, unsigned long PiC)
 *
 * Description:     calculate the object temperature via look-up table
 *
 * Dependencies:    ambient temperature (TAmb), pixel voltage (dig), pixel sensitivity coefficients (PiC)
 *******************************************************************/

unsigned int TH32x32_calcTO(unsigned int TAmb, signed int dig, signed long PiC){
	unsigned int Tobject,y;
	signed int val;
	signed long vx,vy,ydist;
	static unsigned int CurTACol;
	static signed int dTA;
	signed long long scale;


	//first check the position in x-axis of table

	for(CurTACol=0;CurTACol<NROFTAELEMENTS;CurTACol++){
		if((XTATemps[CurTACol]<=TAmb)&&(XTATemps[CurTACol+1]>TAmb))
				break;
			}
	dTA=TAmb-XTATemps[CurTACol];


	if((CurTACol>=NROFTAELEMENTS-1))
			return 0;


	//now scale dig to PCSCALEVAL
	scale=(signed long long)PCSCALEVAL*(signed long long)dig;
	vx=(signed long)(scale/((signed long long)PiC));
	//vx=((signed long long)dig * (signed long long)PiC)/10 ;	// 放大 10 後 再 縮小 10

	val=vx+TABLEOFFSET;
	//now determine row
	y=val>>ADEXPBITS;
	ydist=(signed long)ADEQUIDISTANCE;
	if(y<(NROFADELEMENTS-1)){
		if(TempTable[y][CurTACol]){
			vx=((((signed long)TempTable[y][CurTACol+1]-(signed long)TempTable[y][CurTACol])*(signed long)dTA)/(signed long)TAEQUIDISTANCE)+(signed long)TempTable[y][CurTACol];
			vy=((((signed long)TempTable[y+1][CurTACol+1]-(signed long)TempTable[y+1][CurTACol])*(signed long)dTA)/(signed long)TAEQUIDISTANCE)+(signed long)TempTable[y+1][CurTACol];
			Tobject=(unsigned int)((vy-vx)*((signed long)val-(signed long)YADValues[y])/ydist+(signed long)vx);
			}
		else
			return 0;
		}
	else
		return 0;

#ifdef AdjustOffsetGain
	val=(signed int)Tobject;
	val+=(signed int)GlobalOffset;
	Tobject=(unsigned short)val;
#endif

	return (unsigned short)Tobject;
}




INT32S TH32x32_task_create(INT8U pori)
{
	INT32S nRet;
	osThreadId id;
	osThreadDef_t TH32x32_task = { "TH32x32_task", TH32x32_task_entry, osPriorityNormal, 1, C_SCALER_STACK_SIZE };


	if(TH32x32_task_q == 0) {
		osMessageQDef_t TH32x32_q = {C_SCALER_QUEUE_MAX*2, sizeof(INT32U), 0}; //queue size double for possible null frame

		TH32x32_task_q = osMessageCreate(&TH32x32_q, NULL);
		if(TH32x32_task_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(TH32x32_task_ack_m == 0) {
		osMessageQDef_t TH32x32_ack_q = {C_ACK_QUEUE_MAX, sizeof(INT32U), 0};

		TH32x32_task_ack_m = osMessageCreate(&TH32x32_ack_q, NULL);
		if(TH32x32_task_ack_m == 0) {
			RETURN(STATUS_FAIL);
		}
	}


	id = osThreadCreate(&TH32x32_task, (void *)NULL);
    if(id == 0) {
        RETURN(STATUS_FAIL);
    }

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S TH32x32_task_start(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
	POST_MESSAGE(TH32x32_task_q, MSG_TH32x32_TASK_INIT, TH32x32_task_ack_m, 5000);
Return:
	return nRet;
}

INT32S TH32x32_task_stop(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(TH32x32_task_q, MSG_TH32x32_TASK_STOP, TH32x32_task_ack_m, 5000);

Return:
	return nRet;
}
//
// 不使用此code
//	以 TH32x32_SCALERUP_task_entry(void const *parm) 執行完成 
//	直接 設定 pTH32x32_Para->TH32x32_ScalerUp_status = 1;
//

/*
INT32S TH32x32_task_FramrReady(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(scaler_task_q, MSG_SCALER_TASK_TH32x32Ready, scaler_task_ack_m, 5000);
Return:
	return nRet;
}
*/


static void TH32x32_task_entry(void const *parm)
{
	INT32U msg_id,ready_buf, ack_msg;
	//INT32U sensor_frame, scaler_frame, display_frame;
	//ScalerFormat_t scale;
	//ScalerPara_t para;
	osEvent result;
	osThreadId id;
	INT8U   i;

	drv_l1_i2c_bus_handle_t th32x32_handle;

	sensor32x32_cmdCode sensorCmd;
	INT8U value;
	INT16U value2byte;

	float	PTATGrad,PTATOff,PTATLong;
	float 	common[2];
	INT8U 	EEcopy[8],*pEEcopy;
	INT8U 	EEaddr[2],*pEEaddr;
	INT16U	EEaddress16,EEcopy16BIT,*pEEcopy16BIT;
	INT16U	tmp_i,tmp_i2,tmp_start;
	INT8U	SetMBITUser;
	unsigned short	Resolution;
	INT8U	avg_buf_Enable,check_MAX_pos;
	INT16U	TA,gradScale,TableNumberSensor,epsilon;
	unsigned short GlobalGain;	// INT16U
	char 	GlobalOffset;			// INT8U
	INT32U	PTATsum;
	float	PixCMin,PixCMax;
	INT16U	SetMBITCalib,SetBIASCalib,SetCLKCalib,SetBPACalib,SetPUCalib,VddCalib;
	INT16S	*pTH32x32_VddCompOff_buffer,*pTH32x32_VddCompGrad_buffer;
	INT16S	*pTH32x32_ThGrad_buffer,*pTH32x32_ThOff_buffer;
	INT8U 	*pTH32x32_tmp8B_to16B;
	INT8U 	NrOfDefPix,VddScaling,VddScalingOff;

	unsigned long	*pTH32x32_PixC_buffer;


	INT16U  *pTH32x32_BadPixAdr_buf;
	INT8U  	*pTH32x32_BadPixMaskAdr_buf;
	unsigned long	PixC;		// INT32U
	INT16U  *pTH32x32_frame_INT16U_buf0;
	float	tmpPixC;
	unsigned long long DividerVdd,DividerVdd2;
	signed long	VoltageLong,PowerGradScale;

	INT16U	read_EValue,offset_EValue,TmpValue;

	INT16U	TimeCnt1,TimeCnt2,tmpSec;

	INT8U	firstRun,SampleTempCnt;
	//INT8U   data_buf[1];

	INT8U   TH32x32_ReadBlockNum;

	INT16U  *pBlock_data_buf,*pBlock_EoffsetTop_buf,*pBlock_EoffsetBtm_buf;

	INT8U	retValue;
	INT16U  *pTH32x32_INT16U_avg_buf[AVG_buf_len];
	INT16U  *pTH32x32_TmpOutput_INT16U_buf0;
	INT8U	sampleCnt;
	INT16U	 ReadTempValue,Tmin_number,Tmax_number,TminTable_number,TmaxTable_number;
	signed int	TobjectRange,Tmax,Tmin,TminTable,TmaxTable;	// INT32S
	INT8U   TH32x32_BlockNum , Blk_startIdex;
	signed long VDD_MEAS_sum,VDD_MEAS_TOP,VDD_MEAS_BTM;
	unsigned long Vddlong;	// signed long	Vddlong;
	signed short	DeltaVdd;
	INT16U	cellNum,rowNum,Th_cellNum,Vdd_cellNum; //,EOff_cellNum;
	signed long long VoltageLongLong;
	signed int	Tobject;	// INT32S
	signed int 		dig;
	signed long 	PiC;
	INT8U	Tobject_err;
	INT8U	PreReadBlockFlag;
	INT8U	TmpTbInd;

    DEBUG_MSG("<<%s>>\r\n", __func__);

	while(1)
	{
		result = osMessageGet(TH32x32_task_q, osWaitForever);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (	msg_id == 0)) {
			continue;
		}

		switch(msg_id)
		{
		case MSG_TH32x32_TASK_INIT:
			DEBUG_MSG("[MSG_TH32x32_TASK_INIT]\r\n");

			OSQFlush(TH32x32_task_q);

			for(i=0; i<TH32x32_SCALERUP_BUFFER_NO; i++) {
		        avi_encode_post_empty(TH32x32_SCALERUP_buf_q, pTH32x32_Para->TH32x32_TmpOutput_format_addr[i]);
		    }

			pEEcopy = EEcopy;
			pEEaddr = EEaddr;

			//TH32x32_TEST_HIGH();
			TimeCnt1=xTaskGetTickCount();
			DBG_PRINT("StartTime = %d\r\n", xTaskGetTickCount());	// xTaskGetTickCount() timebase=1ms
			// =====================================================================
#if TH32x32_FUN
			th32x32_handle.devNumber = I2C_1;
		    th32x32_handle.slaveAddr = TH32x32_EEPROM_ID << 1 ;
		    th32x32_handle.clkRate = 800;


			pTH32x32_frame_INT16U_buf0 = (INT16U*)pTH32x32_Para->TH32x32_ColorOutputFrame_addr;
			gp_memset((INT8S *)pTH32x32_Para->TH32x32_ColorOutputFrame_addr,0x00,32*32*2);	// clear 值 
			gp_memcpy((INT8S *)(pTH32x32_Para->TH32x32_ColorOutputFrame_addr),
				(INT8S *)&(sensor32X32_RGB565),32*32*2);
			//
			// EEPROM data [ PTATGrad / PTATOff / GlobalGain / GlobalOffset ]
			//
			EEaddress16 = AdrPTATGrad;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,sizeof(float)*2,TH32x32_I2C_RESTART_MODE);
			gp_memcpy((INT8S*)&common,(INT8S*)pEEcopy,sizeof(float)*2);
			PTATGrad=common[0];
			PTATOff=common[1];
			DBG_PRINT("PTATGrad [0x%x] /PTATOff [0x%x] \r\n",(INT32U)PTATGrad,(INT32U)PTATOff);

			EEaddress16 = AdrGlobalGain;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,sizeof(char)*2,TH32x32_I2C_RESTART_MODE);
			gp_memcpy((INT8S*)&GlobalGain,(INT8S*)pEEcopy,sizeof(short));

			EEaddress16 = AdrGlobalOffset;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,sizeof(char),TH32x32_I2C_RESTART_MODE);
			gp_memcpy((INT8S*)&GlobalOffset,(INT8S*)pEEcopy,sizeof(char));
			DBG_PRINT("GlobalGain [%d] GlobalOffset [%d] \r\n",GlobalGain,GlobalOffset);

			//
			// EEPROM data [ PixCMin / PixCMax ]
			//

			EEaddress16 = AdrPixCMin;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);

			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,sizeof(float)*2,TH32x32_I2C_RESTART_MODE);
			gp_memcpy((INT8S*)&common,(INT8S*)pEEcopy,sizeof(float)*2);
			PixCMin=common[0];
			PixCMax=common[1];
			DBG_PRINT("PixCMin/PixCMax = [0x%x]/ [0x%x] \r\n", PixCMin,PixCMax);


			drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,AdrGradScale,pEEcopy);
			gradScale =(INT16U) EEcopy[0];
			DBG_PRINT("gradScale = [0x%x] \r\n", gradScale);

			EEaddress16 = AdrTableNumber;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,sizeof(char)*3,TH32x32_I2C_RESTART_MODE);
			TableNumberSensor=(INT16U)EEcopy[1]<<8;
			TableNumberSensor +=(INT16U)EEcopy[0];
			epsilon=(INT16U)EEcopy[2];
			DBG_PRINT("TableNumberSensor = [%d]/ epsilon = [%d] \r\n", TableNumberSensor,epsilon);

			EEaddress16 = AdrMBITPixC;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,sizeof(char)*5,TH32x32_I2C_RESTART_MODE);
			SetMBITCalib=(INT16U)EEcopy[0];
			SetBIASCalib=(INT16U)EEcopy[1];
			SetCLKCalib=(INT16U)EEcopy[2];
			SetBPACalib=(INT16U)EEcopy[3];
			SetPUCalib=(INT16U)EEcopy[4];
			DBG_PRINT("MBITPixC = [0x%x] , [0x%x], [0x%x], [0x%x], [0x%x] \r\n",SetMBITCalib,SetBIASCalib,SetCLKCalib,SetBPACalib,SetPUCalib);

			//
			// The corresponding order of ThGrad,ThOffset and P to the Pixelnumber is given by the
			// following overview:
			//讀入順序      block0(top) -> block1(top) -> block2(top) -> block3(top)
			//				-> block0(bottom) -> block1(bottom) -> block2(bottom) -> block3(bottom)
			//

			gp_memset((INT8S *)pTH32x32_Para->TH32x32_ThOff_buffer,0x00,32*32*2);	// clear 值 
			gp_memset((INT8S *)pTH32x32_Para->TH32x32_ThGrad_buffer,0x00,32*32*2);	// clear 值 
			//
			// 8bit 讀取 pTH32x32_tmp8B_to16B
			// 16 bit 計算 
			//
			pTH32x32_ThOff_buffer = (INT16S*)pTH32x32_Para->TH32x32_ThOff_buffer;
			pTH32x32_ThGrad_buffer = (INT16S*)pTH32x32_Para->TH32x32_ThGrad_buffer;

			pTH32x32_tmp8B_to16B = (INT8U*)pTH32x32_Para->TH32x32_ThGrad_buffer;
			EEaddress16 = AdrTh1;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pTH32x32_tmp8B_to16B,sizeof(INT16S)*Pixel,TH32x32_I2C_RESTART_MODE);
			DBG_PRINT("TH32x32_ThGrad_buffer addr= 0x%x\r\n", pTH32x32_Para->TH32x32_ThGrad_buffer);


			pTH32x32_tmp8B_to16B = (INT8U*)pTH32x32_Para->TH32x32_ThOff_buffer;
			EEaddress16 = AdrTh2;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pTH32x32_tmp8B_to16B,sizeof(INT16S)*Pixel,TH32x32_I2C_RESTART_MODE);
			DBG_PRINT("TH32x32_ThOff_buffer addr= 0x%x\r\n", pTH32x32_Para->TH32x32_ThOff_buffer);


			drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,AdrNrOfDefPix,pEEcopy);//read number of defective pixel
			NrOfDefPix = EEcopy[0];
			DBG_PRINT("NrOfDefPix = %d \r\n",NrOfDefPix);

			if(NrOfDefPix == 0)
				DBG_PRINT("NrOfDefPix is zero %d \r\n",NrOfDefPix);
			else if(NrOfDefPix > MAXNROFDEFECTS)
				DBG_PRINT("NrOfDefPix is over MAXNROFDEFECTS %d \r\n",NrOfDefPix);
			else if(( NrOfDefPix != 0 )&&( NrOfDefPix <= MAXNROFDEFECTS )) {
				gp_memset((INT8S *)pTH32x32_Para->TH32x32_BadPixAdr_buf,0x00,sizeof(INT16U)*MAXNROFDEFECTS);	// clear 值 

				pTH32x32_tmp8B_to16B = (INT8U*)pTH32x32_Para->TH32x32_BadPixAdr_buf;
				EEaddress16 = AdrDeadPix;
				EEaddr[0]=(INT8U)(EEaddress16 >> 8);
				EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
				drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pTH32x32_tmp8B_to16B,sizeof(INT16S)*NrOfDefPix,TH32x32_I2C_RESTART_MODE);
				pTH32x32_BadPixAdr_buf = (INT16U*) pTH32x32_Para->TH32x32_BadPixAdr_buf;
				DBG_PRINT("Dead Pix Adr addr = ");
				for (tmp_i2 = 0 ; tmp_i2<NrOfDefPix ; tmp_i2++){
					DBG_PRINT(" %d,",*(pTH32x32_BadPixAdr_buf + tmp_i2) );
				}
				DBG_PRINT("\r\n");
				gp_memset((INT8S *)pTH32x32_Para->TH32x32_BadPixMask_buf,0x00,sizeof(INT8U)*MAXNROFDEFECTS);	// clear 值 
				EEaddress16 = AdrDeadPixMask;
				EEaddr[0]=(INT8U)(EEaddress16 >> 8);
				EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
				drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,(INT8U*)pTH32x32_Para->TH32x32_BadPixMask_buf,sizeof(INT8U)*NrOfDefPix,TH32x32_I2C_RESTART_MODE);
				pTH32x32_BadPixMaskAdr_buf = (INT8U*) pTH32x32_Para->TH32x32_BadPixMask_buf;
				DBG_PRINT("Dead Pix mask = ");
				for (tmp_i2 = 0 ; tmp_i2<NrOfDefPix ; tmp_i2++){
					DBG_PRINT(" %d,",*(pTH32x32_BadPixMaskAdr_buf + tmp_i2));
				}
				DBG_PRINT("\r\n");
			}

			//
			// pTH32x32_Para->TH32x32_ColorOutputFrame_addr is tmp buf for AdrPixC
			//

			pTH32x32_tmp8B_to16B = (INT8U*)pTH32x32_Para->TH32x32_ColorOutputFrame_addr;
			EEaddress16 = AdrPixC;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pTH32x32_tmp8B_to16B,sizeof(INT16U)*Pixel,TH32x32_I2C_RESTART_MODE);

			DBG_PRINT("Read Pix to pTH32x32_Para->TH32x32_ColorOutputFrame_addr = 0x%x\r\n",
				pTH32x32_Para->TH32x32_ColorOutputFrame_addr);

			pTH32x32_PixC_buffer =(unsigned long*) pTH32x32_Para->TH32x32_PixC_buffer;
			for( tmp_i2 = 0 ; tmp_i2<Pixel ; tmp_i2++){
				PixC = (unsigned long)((((float)*(pTH32x32_frame_INT16U_buf0 + tmp_i2)/65535.0)*(PixCMax-PixCMin)+(float)PixCMin)*(float)epsilon/100.0*(float)GlobalGain/10000.0);	//
				*(pTH32x32_PixC_buffer+tmp_i2) = PixC;
			}


			pTH32x32_VddCompOff_buffer = (INT16S*)pTH32x32_Para->TH32x32_VddCompOff_buffer;
			pTH32x32_VddCompGrad_buffer = (INT16S*)pTH32x32_Para->TH32x32_VddCompGrad_buffer;

			gp_memset((INT8S *)pTH32x32_Para->TH32x32_VddCompGrad_buffer,0x00,sizeof(INT16U)*ELAMOUNT); // clear 值 

			pTH32x32_tmp8B_to16B = (INT8U*)pTH32x32_Para->TH32x32_VddCompGrad_buffer;
			EEaddress16 = AdrVddCompGrad;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pTH32x32_tmp8B_to16B,sizeof(INT16U)*ELAMOUNT,TH32x32_I2C_RESTART_MODE);
			DBG_PRINT("TH32x32_VddCompGrad_buffer addr= 0x%x\r\n", pTH32x32_Para->TH32x32_VddCompGrad_buffer);

			gp_memset((INT8S *)pTH32x32_Para->TH32x32_VddCompOff_buffer,0x00,sizeof(INT16U)*ELAMOUNT);	// clear 值 

			pTH32x32_tmp8B_to16B = (INT8U*)pTH32x32_Para->TH32x32_VddCompOff_buffer;
			EEaddress16 = AdrVddCompOff;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pTH32x32_tmp8B_to16B,sizeof(INT16U)*ELAMOUNT,TH32x32_I2C_RESTART_MODE);
			DBG_PRINT("TH32x32_VddCompOff_buffer addr= 0x%x\r\n", pTH32x32_Para->TH32x32_VddCompOff_buffer);

			drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,AdrVddScaling,pEEcopy);
			VddScaling = EEcopy[0];
			drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,AdrVddScalingOff,pEEcopy);
			VddScalingOff = EEcopy[0];
			DBG_PRINT("VddScaling = [0x%x] , VddScalingOff = [0x%x]\r\n",VddScaling,VddScalingOff);
			tmpPixC = pow(2.0,(float)VddScaling);	// Add #include <math.h>
			DividerVdd=(unsigned long long)tmpPixC;
			tmpPixC = pow(2.0,(float)VddScalingOff);
			DividerVdd2=(unsigned long long)tmpPixC;

			EEaddress16 = AdrVddCalib;
			EEaddr[0]=(INT8U)(EEaddress16 >> 8);
			EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,sizeof(INT16U),TH32x32_I2C_RESTART_MODE);
			VddCalib =  (INT16U)EEcopy[1]<<8;
			VddCalib += (INT16U)EEcopy[0];
			DBG_PRINT("VddCalib = [0x%x]\r\n",VddCalib);

			PowerGradScale=(float)pow(2.0,(float)gradScale);	// #include <math.h>
			DBG_PRINT("PowerGradScale = 0x%x\r\n",PowerGradScale);
#endif
			TimeCnt2 = xTaskGetTickCount();
			DBG_PRINT("EndTime = %d\r\n", xTaskGetTickCount());
			DBG_PRINT("TotalTime = %d ms\r\n",TimeCnt2 - TimeCnt1);

			TH32x32_TEST_LOW();
			//TimeCnt1 = OSTimeGet() ;

			// =====================================================================
			th32x32_handle.devNumber = I2C_1;
		    th32x32_handle.clkRate = 1000;	// 必須 再設定 


			//
			//	sensor registers
			//

			sensorCmd.DeviceAdd =(TH32x32_REG_ID << 1);
		    th32x32_handle.slaveAddr = sensorCmd.DeviceAdd;

			SetMBITUser = CONFIG_REG_WAKEUP;
			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.trimValue =SetMBITUser;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);
#if TH32x32_FUN

			DBG_PRINT("Wake up >RegAdd=0x%x trimValue=0x%x\r\n",sensorCmd.RegAdd,sensorCmd.trimValue);
#endif
			osDelay(10);
			//SetMBITUser = MBIT_TRIM_10BIT_DEFAULT;
			SetMBITUser = MBIT_TRIM_12BIT_DEFAULT;
			Resolution=(SetMBITUser&0x0F)+4; //暫不調整 
			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.trimValue =SetMBITUser;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);
#if TH32x32_FUN
			DBG_PRINT(" **** InitMBITTRIM (%d bit) [0x%x] \r\n",Resolution,SetMBITUser);
#endif
			osDelay(10);
			sensorCmd.RegAdd =TH32x32_BIAS_TRIM_TOP;
			sensorCmd.trimValue =BIAS_TRIM_TOP_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

            osDelay(10);
			sensorCmd.RegAdd =TH32x32_BIAS_TRIM_BOT;
			sensorCmd.trimValue =BIAS_TRIM_BOM_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

            osDelay(10);
           	sensorCmd.RegAdd =TH32x32_CLK_TRIM;
			sensorCmd.trimValue =CLKTRIM_13MHz;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);
#if TH32x32_FUN

				DBG_PRINT(" **** CLKTRIM_13MHz [0x%x] \r\n",sensorCmd.trimValue);
#endif
			osDelay(10);
           	sensorCmd.RegAdd =TH32x32_BPA_TRIM_TOP;
			sensorCmd.trimValue =BPA_TRIM_TOP_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

			osDelay(10);
           	sensorCmd.RegAdd =TH32x32_BPA_TRIM_BOT;
			sensorCmd.trimValue =BPA_TRIM_BOM_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

			osDelay(10);
           	sensorCmd.RegAdd =TH32x32_PU_SDA_TRIM;
			sensorCmd.trimValue =PU_SDA_TRIM_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);
#if TH32x32_FUN

			if( TableNumberSensor != TABLENUMBER )
				DBG_PRINT(" Error  !! TableNumberSensor[%d] is NOT  TABLENUMBER[%d] \r\n",TableNumberSensor,TABLENUMBER);
			else
				DBG_PRINT(" TableNumberSensor[%d] is TABLENUMBER[%d] \r\n",TableNumberSensor,TABLENUMBER);
#endif
			SampleTempCnt = 0;
			while(SampleTempCnt == 0)
			{
			osDelay(10);
			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.trimValue =READ_ELEC_OFFSET;	// start / blink /wake 產生 ReadElecOffset & TA
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

			tmp_i=0;
			do{
				osDelay(5);
				EEaddr[0]=TH32x32_STATUS_REG;
				drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
				tmp_i++;
			}while(((EEcopy[0]&0x03)!=0x03) || (tmp_i > 10) );
			if (tmp_i > 10)
				DBG_PRINT("ReadElecOffset fail\r\n "); // 曾經 出現 EEcopy[0]=0x05
			else{
				//DBG_PRINT("ReadElecOffset 0x%x wait=%d \r\n ",EEcopy[0],tmp_i);
				SampleTempCnt = 1;
				}
			}
			//
			// read TH32x32 ReadElecOffset raw data 129*2byte
			//

			gp_memset((INT8S *)pTH32x32_Para->TH32x32_readout_EOffTop_buf0_addr,0x00,sizeof(INT16U)*(ELAMOUNTHALF+1));	// clear 值 
			gp_memset((INT8S *)pTH32x32_Para->TH32x32_readout_EOffBtm_buf0_addr,0x00,sizeof(INT16U)*(ELAMOUNTHALF+1));	// clear 值 

			retValue = I2C_sensor32x32_readDataTop(&th32x32_handle,pTH32x32_Para->TH32x32_readout_EOffTop_buf0_addr);
			if(retValue<0)
				DBG_PRINT("ReadElecOffset Top read fail 0x%x \r\n",retValue);
			retValue = I2C_sensor32x32_readDataBtm(&th32x32_handle,pTH32x32_Para->TH32x32_readout_EOffBtm_buf0_addr);
			if(retValue<0)
				DBG_PRINT("ReadElecOffset Btm read fail 0x%x \r\n",retValue);


			pBlock_EoffsetTop_buf =(INT16U*)pTH32x32_Para->TH32x32_readout_EOffTop_buf0_addr;
			pBlock_EoffsetBtm_buf =(INT16U*)pTH32x32_Para->TH32x32_readout_EOffBtm_buf0_addr;

			read_EValue= (INT16U) *(pBlock_EoffsetTop_buf);
			TOBigEndian(read_EValue);
			PTATsum = (INT32U) read_EValue;
			//DBG_PRINT("[PTAT-T %x ,", PTATsum);

			read_EValue= (INT16U) *(pBlock_EoffsetBtm_buf);
			TOBigEndian(read_EValue);
			//DBG_PRINT("-B0x%x ," read_EValue);
			PTATsum = PTATsum + (INT32U) read_EValue;
			//DBG_PRINT("= 0x%x] ,", PTATsum/2);

			PTATLong =(float) ((PTATsum/2)<<(16-Resolution));

			TA=(unsigned short)((float)(PTATLong*PTATGrad+PTATOff+0.5));
			DBG_PRINT("[TA=%dC] \r\n",TA-2730);

			// 預先執行 read block cmd -0
			TH32x32_ReadBlockNum=0;

			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.trimValue =(Read_VDD_MEAS_Block0 | (TH32x32_ReadBlockNum << 4)); // start / VDD_MEAS /wake
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

			tmp_i=0;
			do{
				osDelay(TH32x32_ReadStatus_WaitTime+5);
				EEaddr[0]=TH32x32_STATUS_REG;
				drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
				tmp_i++;
			}while( ( ((EEcopy[0]&0x35) != ((sensorCmd.trimValue & 0x35) | 0x01)) )|| (tmp_i > 10) );
			if (tmp_i > 10)
				DBG_PRINT("Read_VDD_MEAS_Block0 fail\r\n "); // 曾經 出現 EEcopy[0]=0x05
			else{
				//DBG_PRINT("Read_VDD_MEAS_Block %d ack= 0x%x wait=%d \r\n ",
				//	TH32x32_ReadBlockNum,EEcopy[0],tmp_i);
				SampleTempCnt = 1;
			}

			DBG_PRINT("Read_VDD_MEAS_Block0 -start \r\n");
			//
			// read TH32x32 Read_VDD_MEAS_Block raw data 129*2byte
			//

			// 讀取 read block[0]
			retValue = I2C_sensor32x32_readDataTop(&th32x32_handle,
				pTH32x32_Para->TH32x32_readout_top_block_buf_addr[0][TH32x32_ReadBlockNum]);
			if(retValue<0)
				DBG_PRINT("Read_VDD_MEAS_Block Top read fail 0x%x \r\n",retValue);
			retValue = I2C_sensor32x32_readDataBtm(&th32x32_handle,
				pTH32x32_Para->TH32x32_readout_btm_block_buf_addr[0][TH32x32_ReadBlockNum]);
			if(retValue<0)
				DBG_PRINT("Read_VDD_MEAS_Block Btm read fail 0x%x \r\n",retValue);

			DBG_PRINT("Read_VDD_MEAS_Block0 -End \r\n");
			avg_buf_Enable=0;
			check_MAX_pos=0;
			pTH32x32_frame_INT16U_buf0 = (INT16U*)pTH32x32_Para->TH32x32_ColorOutputFrame_addr;
			pTH32x32_Para->TH32x32_OVR_RoomTemp = COLOR_TABLE_OVR_RoomTemp;
			pTH32x32_Para->TH32x32_TABLE_SCALER_FACTOR = 18;
			pTH32x32_Para->TH32x32_NOISE_CUTOFF_OVR_RTemp = NOISE_OVR_RoomTemp;
			DBG_PRINT("Noise filter =%d \r\n",pTH32x32_Para->TH32x32_NOISE_CUTOFF_OVR_RTemp);


			for(tmp_i=0;tmp_i<AVG_buf_len;tmp_i++){
				pTH32x32_INT16U_avg_buf[tmp_i]= (INT16U*)pTH32x32_Para->TH32x32_avg_buf_addr[tmp_i];
			}


			// start timer_B
			pTH32x32_Para->TH32x32_sampleCnt = 0;
			pTH32x32_Para->TH32x32_ReadElecOffset_TA_startON = 1;
			pTH32x32_Para->TH32x32_sampleHz = 100; // 5.7~ 732 (100ms),20(50ms),100(10 ms),500(2 ms)
			TmaxTable = 250;
			TminTable = 250;
			ReadTempValue = 250; // 先設定 標準溫度 
			tmpSec=0;
			retValue = timer_freq_setup(TIMER_B, pTH32x32_Para->TH32x32_sampleHz, 0, TH32x32_start_timer_isr );
			DBG_PRINT("Set TH32x32_ReadElecOffset_timer_isr ret--> %d \r\n",retValue) ;


			firstRun = 0;
			SampleTempCnt = 0;

			ack_msg = ACK_OK;
			osMessagePut(TH32x32_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;

		case MSG_TH32x32_TASK_STOP:
			DEBUG_MSG("[MSG_TH32x32_TASK_STOP]\r\n");
			OSQFlush(TH32x32_task_q);
			ack_msg = ACK_OK;
			osMessagePut(TH32x32_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;

		case MSG_TH32x32_TASK_EXIT:
			DEBUG_MSG("[MSG_TH32x32_TASK_EXIT]\r\n");

			ack_msg = ACK_OK;
			osMessagePut(TH32x32_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			id = osThreadGetId();
    		osThreadTerminate(id);
			break;

		default:

			//sensor_frame = msg_id & 0x7fffffff;
			ready_buf = msg_id;

			pTH32x32_TmpOutput_INT16U_buf0 = (INT16U*)ready_buf;

			TimeCnt1 = xTaskGetTickCount();

			if(firstRun == 1){
			// read block[0]
			TH32x32_ReadBlockNum=0;

			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.trimValue =(Read_VDD_MEAS_Block0 | (TH32x32_ReadBlockNum << 4)); // start / VDD_MEAS /wake
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

			tmp_i=0;
			do{
				osDelay(TH32x32_ReadStatus_WaitTime);
				EEaddr[0]=TH32x32_STATUS_REG;
				drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
				tmp_i++;
			}while( ( ((EEcopy[0]&0x35) != ((sensorCmd.trimValue & 0x35) | 0x01)) )|| (tmp_i > 10) );
			if (tmp_i > 10)
				DBG_PRINT("Read_VDD_MEAS_Block %d fail\r\n ",TH32x32_ReadBlockNum); // 曾經 出現 EEcopy[0]=0x05
		#if CHECK_ReadStatus_WAITTIME
			else{
				DBG_PRINT("firstRun = 1,Read_VDD_MEAS_Block %d ack= 0x%x wait=%d \r\n ",
					TH32x32_ReadBlockNum,EEcopy[0],tmp_i);
			}
		#endif
			retValue = I2C_sensor32x32_readDataTop(&th32x32_handle,
					pTH32x32_Para->TH32x32_readout_top_block_buf_addr[0][TH32x32_ReadBlockNum]);
			if(retValue<0)
					DBG_PRINT("firstRun -> read block %d Top read fail 0x%x \r\n",TH32x32_ReadBlockNum,retValue);
			retValue = I2C_sensor32x32_readDataBtm(&th32x32_handle,
					pTH32x32_Para->TH32x32_readout_btm_block_buf_addr[0][TH32x32_ReadBlockNum]);
			if(retValue<0)
					DBG_PRINT("firstRun -> read block %d Btm read fail 0x%x \r\n",TH32x32_ReadBlockNum,retValue);

			}

			if( pTH32x32_Para->TH32x32_ReadElecOffset_TA_startON == 1 ){
			tmpSec++;
			//
			// 每5秒 一次 AD7314_readTempature(暫不執行) 
			//
			ReadTempValue = TA-2730; // 先設定 TA 為標準溫度 
			pTH32x32_Para->TH32x32_TA_AD7314 = ReadTempValue;
			pTH32x32_Para->TH32x32_TMAX = Tmax;	// Tmax Tmin initial value 250 25C
			pTH32x32_Para->TH32x32_Tmin = Tmin;
			pTH32x32_Para->TH32x32_TA = TA-2730;

			//
			// 每5秒 一次 產生 	  ReadElecOffset & TA
			// 	需要定時 更新,隨環境溫度會變化 
			//

			SampleTempCnt = 0;
			while(SampleTempCnt == 0)
			{
			osDelay(10);
			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.trimValue =READ_ELEC_OFFSET;	// start / blink /wake 產生 ReadElecOffset & TA
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

			tmp_i=0;
			do{
				osDelay(TH32x32_ReadStatus_WaitTime);
				EEaddr[0]=TH32x32_STATUS_REG;
				drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
				tmp_i++;
			}while(((EEcopy[0]&0x03)!=0x03) || (tmp_i > 10) );
			if (tmp_i > 10)
				DBG_PRINT("ReadElecOffset fail\r\n "); // 曾經 出現 EEcopy[0]=0x05
			else{
				//DBG_PRINT("ReadElecOffset 0x%x wait=%d \r\n ",EEcopy[0],tmp_i);
				SampleTempCnt = 1;
				}
			}
			//
			// read TH32x32 ReadElecOffset raw data 129*2byte
			//

			// clear for test only
			//gp_memset((INT8S *)pTH32x32_Para->TH32x32_readout_EOffTop_buf0_addr,0x00,sizeof(INT16U)*(ELAMOUNTHALF+1));	// clear 值 
			//gp_memset((INT8S *)pTH32x32_Para->TH32x32_readout_EOffBtm_buf0_addr,0x00,sizeof(INT16U)*(ELAMOUNTHALF+1));	// clear 值 

			retValue = I2C_sensor32x32_readDataTop(&th32x32_handle,pTH32x32_Para->TH32x32_readout_EOffTop_buf0_addr);
			if(retValue<0)
				DBG_PRINT("ReadElecOffset Top read fail 0x%x \r\n",retValue);
			retValue = I2C_sensor32x32_readDataBtm(&th32x32_handle,pTH32x32_Para->TH32x32_readout_EOffBtm_buf0_addr);
			if(retValue<0)
				DBG_PRINT("ReadElecOffset Btm read fail 0x%x \r\n",retValue);


			//pBlock_EoffsetTop_buf =(INT16U*)pTH32x32_Para->TH32x32_readout_EOffTop_buf0_addr;
			//pBlock_EoffsetBtm_buf =(INT16U*)pTH32x32_Para->TH32x32_readout_EOffBtm_buf0_addr;

			read_EValue= (INT16U) *(pBlock_EoffsetTop_buf);
			TOBigEndian(read_EValue);
			PTATsum = (INT32U) read_EValue;
			//DBG_PRINT("[PTAT-T %x ,", PTATsum);

			read_EValue= (INT16U) *(pBlock_EoffsetBtm_buf);
			TOBigEndian(read_EValue);
			//DBG_PRINT("-B0x%x ," read_EValue);
			PTATsum = PTATsum + (INT32U) read_EValue;
			//DBG_PRINT("= 0x%x] ,", PTATsum/2);

			PTATLong =(float) ((PTATsum/2)<<(16-Resolution));

			TA=(unsigned short)((float)(PTATLong*PTATGrad+PTATOff+0.5));
			DBG_PRINT("[TA=%dC] %d * 5sec \r\n",TA-2730,tmpSec);

			// 每5秒 一次 ReadElecOffset	& TA -> end

			pTH32x32_Para->TH32x32_ReadElecOffset_TA_startON = 0;

			}



		PreReadBlockFlag = PRE_READ_DIS;
		for( TH32x32_BlockNum = 0; TH32x32_BlockNum < 4 ; TH32x32_BlockNum++ )
		{
			//
			//************************* TOP Block ***********************************************
			//
			pBlock_data_buf = (INT16U*)pTH32x32_Para->TH32x32_readout_top_block_buf_addr[0][TH32x32_BlockNum];
			//
			// ****************** 計算 Block0 Top0 與 offset 差值 以 INT16U 格式計算 *****
			//
			read_EValue= (INT16U) *(pBlock_data_buf);
			TOBigEndian(read_EValue);
			//DBG_PRINT("block[%d] read_EValue=0x%x \r\n",TH32x32_BlockNum,read_EValue);

			VDD_MEAS_TOP=(read_EValue<<(16-Resolution));
			Vddlong = (unsigned long)VDD_MEAS_TOP;
			//DBG_PRINT("< VDD= T[%d] 0x%x ,",TH32x32_BlockNum, VDD_MEAS_TOP);

			DeltaVdd = (signed short) ((signed long)Vddlong-(signed long)VddCalib);
			//DBG_PRINT("[T%d-0x%x ,",TH32x32_BlockNum, Vddlong);

			tmp_start = 1;
			for (rowNum = rowNumEnd_32 * 1+1; rowNum < rowNumEnd_32*5+1; rowNum = rowNum+ rowNumEnd_32) // rowNum from 32*1 to 32*4
			{
				//DBG_PRINT("loop start rowNum = %d \r\n",rowNum);
			for(cellNum = tmp_start;cellNum < rowNum ; cellNum++){	// cellNum form  1 to 32 /every row

				read_EValue= (INT16U) *(pBlock_data_buf+cellNum);
				TOBigEndian(read_EValue);
				offset_EValue=(INT16U) *(pBlock_EoffsetTop_buf + cellNum);
				TOBigEndian(offset_EValue);


				//if((TH32x32_BlockNum*640 + cellNum -1) == 21) {
				//	DBG_PRINT("[cellNumber = %d",(TH32x32_BlockNum*640 + cellNum -1));
				//}

				// ****************** Block0 Top0 *****
				// tmp_start=1;
				// TH32x32 byte1/byte2 is PTAT or VDD , is not data !!
				// cell[0]	 ~ cell[31]
				//	.....
				// cell[96] ~ cell[127]

				// Top substract the thermal Offsets
				Th_cellNum = cellNum + TH32x32_BlockNum*PixelOfBlock -1 ; // PixelOfBlock = 32*4
				//chkValue = *(pTH32x32_ThGrad_buffer+Th_cellNum);
				VoltageLong=(signed long)((signed short)*(pTH32x32_ThGrad_buffer+Th_cellNum));
				VoltageLong*=(signed long)PTATLong;
				VoltageLong/=PowerGradScale;
				VoltageLong+=(signed long)*(pTH32x32_ThOff_buffer+Th_cellNum);

				// Top VDD Compensation
				Vdd_cellNum = cellNum - 1 ;
				VoltageLongLong=(signed long long)*(pTH32x32_VddCompGrad_buffer + Vdd_cellNum);
				VoltageLongLong*=(signed long long)PTATLong;
				VoltageLongLong/=(signed long long)DividerVdd;
				VoltageLongLong+=(signed long long)*(pTH32x32_VddCompOff_buffer + Vdd_cellNum);
				VoltageLongLong*=(signed long long)DeltaVdd;
				VoltageLongLong/=(signed long long)DividerVdd2;
				VoltageLong+=(signed long)VoltageLongLong;


				TmpValue = (signed short)( (read_EValue<<(16-Resolution)) - (signed short)VoltageLong );
				//  Top substract the elect. Offsets
				TmpValue = TmpValue - (signed short)(offset_EValue<<(16-Resolution));

				dig = (signed short)TmpValue;
				PiC =(signed long) *(pTH32x32_PixC_buffer+TH32x32_BlockNum*PixelOfBlock + cellNum -1); // PixelOfBlock = 32*4
				Tobject = (signed int)(TH32x32_calcTO(TA, dig ,PiC ));
#if TH32x32_FUN
				// OUT of range check
				if((Tobject == 0 )&&(firstRun==1)) {
					Tobject_err=1;
					DBG_PRINT("Tobject error at top %d/T=%d  and set 25C \r\n",cellNum + TH32x32_BlockNum*PixelOfBlock -1,(Tobject));
					Tobject = 2730 + 250;
				}
#endif
				*(pTH32x32_TmpOutput_INT16U_buf0 + TH32x32_BlockNum*PixelOfBlock + cellNum -1)
						=Tobject;
				#if 0
				if(cellNum == 95)
					DBG_PRINT("cell %d blk %d  -TH %d > Vdd %d \r\n",cellNum, TH32x32_BlockNum,Th_cellNum,Vdd_cellNum);
				if(cellNum == 96)
					DBG_PRINT("cell %d blk %d  -TH %d > Vdd %d \r\n",cellNum, TH32x32_BlockNum,Th_cellNum,Vdd_cellNum);
				if(cellNum == 97)
					DBG_PRINT("cell %d blk %d  -TH %d > Vdd %d \r\n",cellNum, TH32x32_BlockNum,Th_cellNum,Vdd_cellNum);
				#endif

			}
			tmp_start=cellNum;

			//DBG_PRINT("T %d Vdd %d, rowNum = %d \r\n", tmp_start,Vdd_cellNum,rowNum);
			}

			//
			//************************* BOTTOM Block ***********************************************
			//
			pBlock_data_buf = (INT16U*)pTH32x32_Para->TH32x32_readout_btm_block_buf_addr[0][TH32x32_BlockNum];
			read_EValue= (INT16U) *(pBlock_data_buf);
			TOBigEndian(read_EValue);
			VDD_MEAS_BTM=(read_EValue<<(16-Resolution));
			Vddlong = (unsigned long)VDD_MEAS_BTM;
			DeltaVdd = (signed short) ((signed long)Vddlong-(signed long)VddCalib);
			//DBG_PRINT(" B%d-0x%x ]",TH32x32_BlockNum, Vddlong);
			//DBG_PRINT(" B[%d] 0x%x>" ,TH32x32_BlockNum,VDD_MEAS_BTM);

			tmp_start=1;
			Blk_startIdex = 0;

			for (rowNum = rowNumEnd_32*1+1; rowNum < rowNumEnd_32*5+1; rowNum = rowNum+ rowNumEnd_32) // rowNum from 32*1 to 32*4
			{

			for(cellNum = tmp_start;cellNum < rowNum ; cellNum++){	// cellNum form  1 to 32 /every row

				read_EValue= (INT16U) *(pBlock_data_buf+cellNum);
				TOBigEndian(read_EValue);

				offset_EValue=(INT16U) *(pBlock_EoffsetBtm_buf+cellNum);
				TOBigEndian(offset_EValue);

				// ****************** Block0 Bottom 0 *****
				// tmp_start=1
				// TH32x32 byte1/byte2 is PTAT or VDD , is not data
				// cell[31*32+1]=[992+1]代表 cell 992的資料      ~ cell[32*32]=[1023+1]代表 cell 1023的資料 
				//	.....
				// cell[16*32+1]   ~ cell[17*32]

				//
				// Bottom substract the thermal Offsets
				// (ThGrad/ThOff/Pij 存放資料位置 與 pix data 讀取順序相同 )
				// PixelOfBlock = 32*4 , PixelOfBlock * 4 block = 128 * 4
				//
				Th_cellNum =PixelOfBlock*4 + TH32x32_BlockNum*PixelOfBlock + cellNum - 1 ;
				VoltageLong=(signed long)((signed short)*(pTH32x32_ThGrad_buffer+Th_cellNum));
				VoltageLong*=(signed long)PTATLong;
				VoltageLong/=PowerGradScale;
				VoltageLong+=(signed long)*(pTH32x32_ThOff_buffer+Th_cellNum);


				// Bottom VDD Compensation
				Vdd_cellNum = PixelOfBlock + cellNum - 1 ;;
				VoltageLongLong=(signed long long)*(pTH32x32_VddCompGrad_buffer + Vdd_cellNum);
				VoltageLongLong*=(signed long long)PTATLong;
				VoltageLongLong/=(signed long long)DividerVdd;
				VoltageLongLong+=(signed long long)*(pTH32x32_VddCompOff_buffer + Vdd_cellNum);
				VoltageLongLong*=(signed long long)DeltaVdd;
				VoltageLongLong/=(signed long long)DividerVdd2;
				VoltageLong+=(signed long)VoltageLongLong;


				TmpValue = (signed short)( (read_EValue<<(16-Resolution)) - (signed short)VoltageLong );
				// Bottom substract the elect. Offsets
				TmpValue = TmpValue - (signed short)(offset_EValue<<(16-Resolution));

				// ambient temperature (TAmb), pixel voltage (dig), pixel sensitivity coefficients (PiC)
				//TH32x32_calcTO(unsigned int TAmb, signed int dig, signed long PiC)
				dig = (signed short)TmpValue;
				//PiC =(signed long) *(pTH32x32_PixC_buffer+ 2560 + TH32x32_BlockNum*640 + cellNum -1);
				// 改成 
				//PiC =(signed long) *(pTH32x32_PixC_buffer+ PixelOfBlock*4 + TH32x32_BlockNum*PixelOfBlock + cellNum -1);
				//
				PiC =(signed long) *(pTH32x32_PixC_buffer+ Th_cellNum);
				Tobject = (signed int)(TH32x32_calcTO(TA, dig ,PiC ));

				tmp_i2 =Blk_start_ary[Blk_startIdex]
						+ cellNum - tmp_start - TH32x32_BlockNum*PixelOfBlock ;

				// OUT of range check
#if TH32x32_FUN

				if((Tobject == 0 ) && (tmp_i2 != 1023)) {
					Tobject_err=1;
					DBG_PRINT("Tobject error at %d/T=%d and set 25C \r\n",tmp_i2,Tobject);
					Tobject = 2730 + 250;
				}
#endif
				*(pTH32x32_TmpOutput_INT16U_buf0 + tmp_i2) = Tobject;

			}
			tmp_start = cellNum;
			Blk_startIdex++;
			//DBG_PRINT("THb %d,", Th_cellNum);
			//DBG_PRINT("B %d-THb %d > Vdd %d,", tmp_start,Th_cellNum,Vdd_cellNum);
			//DBG_PRINT("\r\n");
			}


			//************************* TH32x32_BlockNum = 0 End********************************************

			// 讀取 前一個 啟動的 read block
			if((PreReadBlockFlag == PRE_READ_ON) && (TH32x32_BlockNum < 3) ){
				if(TH32x32_ReadBlockNum == TH32x32_BlockNum+1){
					tmp_i=0;
					do{
						osDelay(TH32x32_ReadStatus_WaitTime);
						EEaddr[0]=TH32x32_STATUS_REG;
						drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
						tmp_i++;
					}while( ( ((EEcopy[0]&0x35) != ((sensorCmd.trimValue & 0x35) | 0x01)) )|| (tmp_i > 10) );
					if (tmp_i > 10)
						DBG_PRINT("Read_VDD_MEAS_Block %d fail\r\n ",TH32x32_ReadBlockNum); // 曾經 出現 EEcopy[0]=0x05
					//else{
						//DBG_PRINT("PRE_READ_ON,Read_VDD_MEAS_Block %d ack= 0x%x wait=%d \r\n ",
						//	TH32x32_ReadBlockNum,EEcopy[0],tmp_i);
					//}

					retValue = I2C_sensor32x32_readDataTop(&th32x32_handle,
						pTH32x32_Para->TH32x32_readout_top_block_buf_addr[0][TH32x32_ReadBlockNum]);
					if(retValue<0)
						DBG_PRINT("Read_VDD_MEAS_Block Top read fail 0x%x \r\n",retValue);
					retValue = I2C_sensor32x32_readDataBtm(&th32x32_handle,
						pTH32x32_Para->TH32x32_readout_btm_block_buf_addr[0][TH32x32_ReadBlockNum]);
					if(retValue<0)
						DBG_PRINT("Read_VDD_MEAS_Block Btm read fail 0x%x \r\n",retValue);
				}

				//
				// 預先執行   next block  "starts a conversion"	   -TH32x32_ReadBlockNum=3
				//

				TH32x32_ReadBlockNum++;
				if(TH32x32_ReadBlockNum < 4 ){
				sensorCmd.RegAdd =TH32x32_CONFIG_REG;
				sensorCmd.trimValue =(Read_VDD_MEAS_Block0 | (TH32x32_ReadBlockNum << 4)); // start / VDD_MEAS /wake
				drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);
				}

			}


			//
			// 讀取 read next block -1 (第一次進入 loop TH32x32_BlockNum = 0 )
			//

			if(PreReadBlockFlag == PRE_READ_DIS){
				TH32x32_ReadBlockNum=TH32x32_BlockNum + 1;	// TH32x32_ReadBlockNum = 1
				if( TH32x32_ReadBlockNum < 4 ){
					sensorCmd.RegAdd =TH32x32_CONFIG_REG;
					sensorCmd.trimValue =(Read_VDD_MEAS_Block0 | (TH32x32_ReadBlockNum << 4)); // start / VDD_MEAS /wake
					drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);

					tmp_i=0;
					do{
						osDelay(TH32x32_ReadStatus_WaitTime);
						EEaddr[0]=TH32x32_STATUS_REG;
						drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
						tmp_i++;
					}while( ( ((EEcopy[0]&0x35) != ((sensorCmd.trimValue & 0x35) | 0x01)) )|| (tmp_i > 10) );
					if (tmp_i > 10)
						DBG_PRINT("Read_VDD_MEAS_Block %d fail\r\n ",TH32x32_ReadBlockNum); // 曾經 出現 EEcopy[0]=0x05
					//else{
						//DBG_PRINT("PRE_READ_DIS,Read_VDD_MEAS_Block %d ack= 0x%x wait=%d \r\n ",
						//TH32x32_ReadBlockNum,EEcopy[0],tmp_i);
					//}

				// 讀取 read block
					retValue = I2C_sensor32x32_readDataTop(&th32x32_handle,
						pTH32x32_Para->TH32x32_readout_top_block_buf_addr[0][TH32x32_ReadBlockNum]);
					if(retValue<0)
						DBG_PRINT("Read_VDD_MEAS_Block Top read fail 0x%x \r\n",retValue);
					retValue = I2C_sensor32x32_readDataBtm(&th32x32_handle,
						pTH32x32_Para->TH32x32_readout_btm_block_buf_addr[0][TH32x32_ReadBlockNum]);
					if(retValue<0)
						DBG_PRINT("Read_VDD_MEAS_Block Btm read fail 0x%x \r\n",retValue);
				}



			// 預先 執行     next block "starts a conversion" cmd	   -2

				TH32x32_ReadBlockNum=TH32x32_BlockNum + 2;	// TH32x32_ReadBlockNum = 2
				PreReadBlockFlag = PRE_READ_ON;

				sensorCmd.RegAdd =TH32x32_CONFIG_REG;
				sensorCmd.trimValue =(Read_VDD_MEAS_Block0 | (TH32x32_ReadBlockNum << 4)); // start / VDD_MEAS /wake
				drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.trimValue);
			//
			// 讀取 read next block -1 (第一次進入 loop) 結束 
			//
			}

		}

		//
		// Tobject 轉換完成 
		//

		firstRun = 1;

		//BadPixCnt= OSTimeGet() % 1000;

		//
		// bad pix check (暫不執行) 
		//DBG_PRINT("bad");

		//
		// diff temp at the same top 8 arrary
		// 偵測 移動 (暫不執行) 
		//
		pTH32x32_Para->TH32x32_move_dect = 0;

		//
		// avg Tobject
		//
			if(avg_buf_Enable ==0){ 	// [0] ~ [3] <- new data
				for(tmp_i=0;tmp_i<AVG_buf_len;tmp_i++){
					gp_memcpy((INT8S *)(pTH32x32_INT16U_avg_buf[tmp_i]),
					(INT8S *)ready_buf,Pixel*2);
				}
				gp_memcpy((INT8S *)(pTH32x32_TmpOutput_INT16U_buf0),
					(INT8S *)ready_buf,Pixel*2);
				avg_buf_Enable=1;
				check_MAX_pos=0;

			}
			else{
			// move new data to avg buf
				for(tmp_i=0;tmp_i<(AVG_buf_len-1);tmp_i++){ 	// [0]<-[1] ,[1]<-[2] ,[2]<-[3]
					gp_memcpy((INT8S *)(pTH32x32_INT16U_avg_buf[tmp_i]),
						(INT8S *)(pTH32x32_INT16U_avg_buf[tmp_i+1]),Pixel*2);
				}
				gp_memcpy((INT8S *)(pTH32x32_INT16U_avg_buf[AVG_buf_len-1]), // [3] <- new data
						(INT8S *)ready_buf,Pixel*2);
				for(cellNum=0;cellNum<Pixel;cellNum++){
					Tobject = 0;
					for(tmp_i=0;tmp_i<AVG_buf_len;tmp_i++){
						TmpValue=*(pTH32x32_INT16U_avg_buf[tmp_i] + cellNum ); //
						Tobject = Tobject +(INT32U)TmpValue;
					}
					Tobject = Tobject/AVG_buf_len;
					*(pTH32x32_TmpOutput_INT16U_buf0 + cellNum)= Tobject;
					//if(((Tobject - 3030)>30) ||((3030- Tobject)>30))DBG_PRINT("%d,",Tobject-2730);
					//if(((Tobject - 3330)>30) ||((3330- Tobject)>30))DBG_PRINT("%d,",Tobject-2730);
				}

			}
									
		//
		// find Tmax Tmin at CORE area (暫不執行) 
		//if(cellNum == (CORE_AREA_limit*80+CORE_AREA_limit)){
		//			Tmax = Tobject;
		//			Tmin = Tobject;
		//		}

		//
		// color table
		//

		//	TableFactor = pTH32x32_Para->TH32x32_TABLE_SCALER_FACTOR;(暫不執行) 
		// 	pTH32x32_Para->TH32x32_OVR_RoomTemp
		//	pTH32x32_Para->TH32x32_NOISE_CUTOFF_OVR_RTemp
		//

		ReadTempValue = pTH32x32_Para->TH32x32_TA_AD7314 + 2730 ; // 暫定 TA 當 戶外溫度 
		//ReadTempValue = 250 + 2730	;		// 25C  當 戶外溫度 

			//DBG_PRINT("Tobject -start \r\n");

		for(cellNum=0;cellNum<Pixel;cellNum++){
			Tobject = *(pTH32x32_TmpOutput_INT16U_buf0 + cellNum);
		//
		// find Tmin/Tmax
		//
			if(cellNum == (CORE_AREA_limit*32+CORE_AREA_limit)){
					Tmax = Tobject;
					Tmin = Tobject;
				}
			if(((cellNum/32) >CORE_AREA_limit)&&((cellNum/32)<(SENSOR_AREA_HIGH -CORE_AREA_limit))
					&&((cellNum%32)>CORE_AREA_limit)&&((cellNum%32)<(SENSOR_AREA_WIDTH -CORE_AREA_limit))){
					if(Tmin > Tobject){ Tmin = Tobject;Tmin_number=cellNum;}
					if(Tmax < Tobject){	Tmax = Tobject;Tmax_number=cellNum;}
			}	

		#if DBG_TOBJ
			DBG_PRINT("- %d [%d]",Tobject-2730,cellNum);
			if((cellNum == 31) || ((cellNum -31 )%32 == 0))
					DBG_PRINT("\r\n");
		#endif
		
			TmpTbInd=10;	//
			if(Tobject > (ReadTempValue+ SHOWTEMP_OFFSET) ){
				if((TminTable==250) && (TmaxTable==250)){		// initial setting
					TmpTbInd =(Tobject - ReadTempValue)/20 ;	// every 2c
					if(TmpTbInd>9)	TmpTbInd=9;
					TmpValue = ColorTable_HOT[TmpTbInd];
				}
				else{
					// auto Autoscale
					TmpTbInd =((Tobject - ReadTempValue)*10)/(TmaxTable-ReadTempValue) ;	
					if(TmpTbInd>9)	TmpTbInd=9;
						TmpValue = ColorTable_HOT[TmpTbInd];
				}
			}
			else TmpValue = TRANSPARENT_COLOR;

		#if DBG_COLOR_TABLE
			DBG_PRINT("- %d= %d [%d]",Tobject-2730,TmpTbInd,cellNum);
			if((cellNum == 31) || ((cellNum -31 )%32 == 0))
					DBG_PRINT("\r\n");
		#endif

			*(pTH32x32_TmpOutput_INT16U_buf0 + cellNum)
							=(signed short)TmpValue;
		}
			//DBG_PRINT("Tobject -End\r\n");

		//
		// set Tmax/Tmin for next frame, NOT this frame
		//
			TminTable=Tmin; TminTable_number=Tmin_number;
			TmaxTable=Tmax; TmaxTable_number=Tmax_number;
		//
		// mirror function (左右 對調) 
		//

		for(cellNum=0;cellNum<Pixel;cellNum++){
				Tobject = *(pTH32x32_TmpOutput_INT16U_buf0 + cellNum);
				tmp_i = cellNum/rowNumEnd_32;
				tmp_i2 = tmp_i*rowNumEnd_32 + (rowNumEnd_32 - 1 - (cellNum%rowNumEnd_32));
				*(pTH32x32_frame_INT16U_buf0 + tmp_i2 ) //
							=Tobject;
		}

			//
			// end of calulate
			//
			// pTH32x32_frame_INT16U_buf0 不能放入 avi_encode_post_empty()
			//
			avi_encode_post_empty(TH32x32_SCALERUP_task_q, pTH32x32_Para->TH32x32_ColorOutputFrame_addr);	// 送出 計算好的	通知	 SCALERUP Task
			avi_encode_post_empty(TH32x32_SCALERUP_buf_q,ready_buf);						// 回收 給 TH32x32_start_timer_isr
			if( pTH32x32_Para->TH32x32_readout_block_startON == 1 ) pTH32x32_Para->TH32x32_readout_block_startON = 0;

			TimeCnt2 = xTaskGetTickCount();


			firstRun = 1;



			if (sampleCnt++ % 60 == 0)
			DBG_PRINT("[ TH32x32_task_entry t2=%d ] tempSet=%d,TminTable=%d[%d-%d],TmaxTable=%d[%d-%d]\r\n",
				TimeCnt2-TimeCnt1,(ReadTempValue+ SHOWTEMP_OFFSET-2730),
				TminTable-2730,TminTable_number/32,TminTable_number%32,
				TmaxTable-2730,TmaxTable_number/32,TmaxTable_number%32);
			//DBG_PRINT("[TA=%dC, MAX =%d[%d],min =%d[%d],t2=%d ] \r\n",TA-2730,Tmax_number,Tmax,Tmin_number,Tmin,TimeCnt2-TimeCnt1);
			//if (sampleCnt++ % 20 == 0) DBG_PRINT("TH32x32_READ = %d \r\n", sampleCnt);

ERROR_QUIT:
			check_MAX_pos=1;


			break;

		}
	}
}


// scaler task
INT32S scaler_task_create(INT8U pori)
{
	INT32S nRet;
	osThreadId id;
	osThreadDef_t scalar_task = { "scalar_task", scaler_task_entry, osPriorityNormal, 1, C_SCALER_STACK_SIZE };
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    osThreadDef_t rotate_task = { "rotate_task", rotate_task_entry, osPriorityLow   , 1, C_SCALER_STACK_SIZE };
#endif

	if(scaler_task_q == 0) {
		osMessageQDef_t scalar_q = {C_SCALER_QUEUE_MAX*2, sizeof(INT32U), 0}; //queue size double for possible null frame

		scaler_task_q = osMessageCreate(&scalar_q, NULL);
		if(scaler_task_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(scaler_task_ack_m == 0) {
		osMessageQDef_t scalar_ack_q = {C_ACK_QUEUE_MAX, sizeof(INT32U), 0};

		scaler_task_ack_m = osMessageCreate(&scalar_ack_q, NULL);
		if(scaler_task_ack_m == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(scaler_frame_q == 0) {
		osMessageQDef_t scalar_f_q = {AVI_ENCODE_SCALER_BUFFER_NO, sizeof(INT32U), 0};

		scaler_frame_q = osMessageCreate(&scalar_f_q, NULL);
		if(scaler_frame_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(display_frame_q == 0) {

		osMessageQDef_t display_f_q = {AVI_ENCODE_DISPALY_BUFFER_NO, sizeof(INT32U), 0};

		display_frame_q = osMessageCreate(&display_f_q, NULL);
		if(display_frame_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(cmos_frame_q == 0) {
		osMessageQDef_t cmos_f_q = {AVI_ENCODE_CSI_BUFFER_NO, sizeof(INT32U), 0};

		cmos_frame_q = osMessageCreate(&cmos_f_q, NULL);
		if(cmos_frame_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	if(jpeg_fifo_q == 0) {
		osMessageQDef_t jpeg_ff_q = {AVI_ENCODE_CSI_FIFO_NO, sizeof(INT32U), 0};

		jpeg_fifo_q = osMessageCreate(&jpeg_ff_q, NULL);
		if(jpeg_fifo_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}
#endif

#if VIDEO_TIMESTAMP
	if (frame_ts_q == 0) {
		osMessageQDef_t frame_ts_df = {AVI_ENCODE_SCALER_BUFFER_NO, sizeof(INT32U), 0};

		frame_ts_q = osMessageCreate(&frame_ts_df, NULL);
		if(frame_ts_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}
#endif

#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    rotator_init();

    if(rotate_msg_q == 0) {
        osMessageQDef_t rot_q_def = { 2, sizeof(INT32U), 0 };

        rotate_msg_q = osMessageCreate(&rot_q_def, NULL);
        if(rotate_msg_q == 0) {
            RETURN(STATUS_FAIL);
        }
    }

    if(rotate_ack_m == 0) {
        osMessageQDef_t rotate_ack_q = {C_ACK_QUEUE_MAX, sizeof(INT32U), 0};

        rotate_ack_m = osMessageCreate(&rotate_ack_q, NULL);
        if(rotate_ack_m == 0) {
            RETURN(STATUS_FAIL);
        }
    }

    if(rotate_frame_q == 0) {
        osMessageQDef_t rotate_f_q = {AVI_ENCODE_ROTATE_BUFFER_NO, sizeof(INT32U), 0};

        rotate_frame_q = osMessageCreate(&rotate_f_q, NULL);
        if(rotate_frame_q == 0) {
            RETURN(STATUS_FAIL);
        }
    }

    id = osThreadCreate(&rotate_task, (void *)NULL);
    if(id == 0) {
        RETURN(STATUS_FAIL);
    }
#endif

	id = osThreadCreate(&scalar_task, (void *)NULL);
    if(id == 0) {
        RETURN(STATUS_FAIL);
    }

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S scaler_task_del(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
	POST_MESSAGE(scaler_task_q, MSG_SCALER_TASK_EXIT, scaler_task_ack_m, 5000);
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    POST_MESSAGE(rotate_msg_q , MSG_ROTATE_TASK_EXIT, rotate_ack_m     , 5000);
#endif
Return:
	OSQFlush(scaler_task_q);
	vQueueDelete(scaler_task_q);
	scaler_task_q = 0;

	OSQFlush(scaler_task_ack_m);
	vQueueDelete(scaler_task_ack_m);
	scaler_task_ack_m = 0;

   	OSQFlush(scaler_frame_q);
   	vQueueDelete(scaler_frame_q);
   	scaler_frame_q = 0;

   	OSQFlush(display_frame_q);
   	vQueueDelete(display_frame_q);
   	display_frame_q = 0;

   	OSQFlush(cmos_frame_q);
   	vQueueDelete(cmos_frame_q);
   	cmos_frame_q = 0;

#if VIDEO_TIMESTAMP
	OSQFlush(frame_ts_q);
	vQueueDelete(frame_ts_q);
   	frame_ts_q = 0;
#endif
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    OSQFlush(rotate_msg_q);
	vQueueDelete(rotate_msg_q);
   	rotate_msg_q = 0;

    OSQFlush(rotate_ack_m);
    vQueueDelete(rotate_ack_m);
   	rotate_ack_m = 0;

    OSQFlush(rotate_frame_q);
    vQueueDelete(rotate_frame_q);
   	rotate_frame_q = 0;
#endif
	return nRet;
}

INT32S scaler_task_start(void)
{
	INT32S i, nRet;

	OSQFlush(scaler_frame_q);
	for(i=0; i<AVI_ENCODE_SCALER_BUFFER_NO; i++) {
		avi_encode_post_empty(scaler_frame_q, pAviEncVidPara->scaler_output_addr[i]);
	}

	OSQFlush(display_frame_q);
	for(i=0; i<AVI_ENCODE_DISPALY_BUFFER_NO; i++) {
		avi_encode_post_empty(display_frame_q, pAviEncVidPara->display_output_addr[i]);
	}

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
    OSQFlush(jpeg_fifo_q);
	for(i=0; i<AVI_ENCODE_CSI_FIFO_NO; i++) {
        avi_encode_post_empty(jpeg_fifo_q, pAviEncVidPara->csi_fifo_addr[i]);
        //DBG_PRINT("jpeg_fifo_q[%d] 0x%x\r\n",i,pAviEncVidPara->csi_fifo_addr[i]);
    }
#endif

#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    OSQFlush(rotate_frame_q);
    for(i=0; i<AVI_ENCODE_ROTATE_BUFFER_NO; i++) {
        avi_encode_post_empty(rotate_frame_q, pAviEncVidPara->rotate_input_addr[i]);
    }
#endif

#if VIDEO_TIMESTAMP
	OSQFlush(frame_ts_q);
	for(i=0; i<AVI_ENCODE_SCALER_BUFFER_NO; i++) {
		avi_encode_post_empty(frame_ts_q, (INT32U)&pAviEncVidPara->frame_ts[i]);
	}
#endif
	nRet = STATUS_OK;
	POST_MESSAGE(scaler_task_q, MSG_SCALER_TASK_INIT, scaler_task_ack_m, 5000);
Return:
	return nRet;
}

INT32S scaler_task_stop(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(scaler_task_q, MSG_SCALER_TASK_STOP, scaler_task_ack_m, 5000);
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    POST_MESSAGE(rotate_msg_q , MSG_ROTATE_TASK_STOP, rotate_ack_m     , 5000);
#endif
Return:
	return nRet;
}

static void Pscaler_Digital_Zoom(INT32U PScaler_Num, INT32U oWidth, INT32U oHeight)
{
    INT32U zoom_w, zoom_h, zoom, Fw, Fh;

    zoom = pAviEncVidPara->scaler_zoom_ratio;

    if (pscaler_zoom[PScaler_Num] == zoom)
        return;

    oWidth = (oWidth + 0xf) & ~0xf;
	oHeight = (oHeight + 0xf) & ~0xf;

    zoom_w = pAviEncVidPara->sensor_capture_width*10/zoom;
    zoom_w = (zoom_w + 1) >> 1 << 1;
    zoom_h = pAviEncVidPara->sensor_capture_height*10/zoom;
    zoom_h = (zoom_h + 1) >> 1 << 1;
    Fw = zoom_w*65536/oWidth;
    Fh = zoom_h*65536/oHeight;
    drv_l1_pscaler_input_X_start_set(PScaler_Num, (pAviEncVidPara->sensor_capture_width-zoom_w) / 2);
    drv_l1_pscaler_input_Y_start_set(PScaler_Num, (pAviEncVidPara->sensor_capture_height-zoom_h) / 2);
    drv_l1_pscaler_output_pixels_set(PScaler_Num, Fw, oWidth, Fh, oHeight);
    drv_l1_pscaler_register_refresh(PScaler_Num, REG_REFRESH_NOW);
	//DBG_PRINT("p%d zoom [%x, %x]\r\n", PScaler_Num, Fw, Fh);

    pscaler_zoom[PScaler_Num] = zoom;
}

static void Pscaler_Display_Callback(INT32U PScaler_Event)
{
    INT32U display_frame = 0, free_frame = 0;
    osStatus status = osOK;

    if(pAviEncVidPara->jpeg_encode_enable_flag &&
    pAviEncVidPara->jpeg_encode_start_flag &&
    pAviEncVidPara->jpeg_use_addr0_flag)
    {
        if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE) {
            display_frame = drv_l1_pscaler_output_A_buffer_get(PSCALER_A);
            if(display_frame != 0x50000000)
            {
                pAviEncVidPara->CSIOrgBufA = display_frame;
                display_frame = 0x50000000;
                drv_l1_pscaler_output_A_buffer_set(PSCALER_A, display_frame);
                DBG_PRINT("1a");
            }
        }
        else if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE){
            display_frame = drv_l1_pscaler_output_B_buffer_get(PSCALER_A);
            if(display_frame != 0x50000000)
            {
                pAviEncVidPara->CSIOrgBufB = display_frame;
                display_frame = 0x50000000;
                drv_l1_pscaler_output_B_buffer_set(PSCALER_A, display_frame);
                DBG_PRINT("2a");
            }
        }
        if(display_frame == 0x50000000)
            display_frame = 0;
    }
    else
    {
        if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE) {
            display_frame = drv_l1_pscaler_output_A_buffer_get(PSCALER_A); // 取得 scaler 完成的 

            if(display_frame == 0x50000000)
            {
                if(pAviEncVidPara->CSIOrgBufA)
                {

                    drv_l1_pscaler_output_A_buffer_set(PSCALER_A, pAviEncVidPara->CSIOrgBufA);
                    pAviEncVidPara->CSIOrgBufA = 0;
                }
                else
                {

                    free_frame = avi_encode_get_empty(display_frame_q);
                    if(free_frame)
                        drv_l1_pscaler_output_A_buffer_set(PSCALER_A, free_frame);
                    else
                        DBG_PRINT("d");
                }
                display_frame = 0;
            }
            else
            {
            	//DBG_PRINT("1A");
                free_frame = avi_encode_get_empty(display_frame_q);
                if(free_frame)
                    drv_l1_pscaler_output_A_buffer_set(PSCALER_A, free_frame);	// davis 放入 未使用的到 pscaler
                else
                {
                    DBG_PRINT("d");
                    drv_l1_pscaler_output_A_buffer_set(PSCALER_A,0x50000000);
                }
            }
        }
        else if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE){
            display_frame = drv_l1_pscaler_output_B_buffer_get(PSCALER_A);

			//DBG_PRINT("2A");
            if(display_frame == 0x50000000)
            {
                 if(pAviEncVidPara->CSIOrgBufB)
                 {
                    drv_l1_pscaler_output_B_buffer_set(PSCALER_A, pAviEncVidPara->CSIOrgBufB);
                    pAviEncVidPara->CSIOrgBufB = 0;
                 }
                 else
                 {
                    free_frame = avi_encode_get_empty(display_frame_q);
                    if(free_frame)
                        drv_l1_pscaler_output_B_buffer_set(PSCALER_A, free_frame);
                    else
                        DBG_PRINT("d");
                 }
                 display_frame = 0;
            }
            else
            {
                free_frame = avi_encode_get_empty(display_frame_q);
                if(free_frame)
                    drv_l1_pscaler_output_B_buffer_set(PSCALER_A, free_frame);
                else
                {
                    DBG_PRINT("d");
                    drv_l1_pscaler_output_B_buffer_set(PSCALER_A,0x50000000);
                }
            }
        }
    }

    /*if(display_frame && videnc_display) {
        videnc_display(pAviEncVidPara->display_buffer_width,
		   			   pAviEncVidPara->display_buffer_height,
		   			   display_frame);
    }*/
    if (display_frame){
        display_frame |= 0x80000000; // 設定 disp_bit 給 scaler_task_entry()
        status = osMessagePut(scaler_task_q, (INT32U)&display_frame, osWaitForever);
        if(status != osOK)
            DBG_PRINT("DF");
    }

#if AVI_ENCODE_DIGITAL_ZOOM_EN
    if (pAviEncVidPara->scaler_zoom_ratio >= 10)
    {
        Pscaler_Digital_Zoom(PSCALER_A, pAviEncVidPara->display_buffer_width, pAviEncVidPara->display_buffer_height);
    }
#endif
    if (pscaler_exit_0)
    {
        pscaler_exit_0 = 0;
        drv_l1_pscaler_stop(PSCALER_A);
        drv_l1_pscaler_clk_ctrl(PSCALER_A, 0);
    }
}

static void Pscaler_H264_Callback(INT32U PScaler_Event)
{
    INT32U OutBuffer = 0;
    INT32U scaler_frame;
    osStatus status = osOK;
#if VIDEO_TIMESTAMP
	INT32U ts = vid_global_tick;
#endif
    if (pAviEncVidPara->scaler_zoom_ratio >= 10)
    {
        Pscaler_Digital_Zoom(PSCALER_B, pAviEncVidPara->encode_width, pAviEncVidPara->encode_height);
    }

    if (pscaler_exit_1)
    {

        OutBuffer = drv_l1_pscaler_H264_output_A_buffer_get(PSCALER_B);
        avi_encode_post_empty(scaler_frame_q, OutBuffer);
        OutBuffer = drv_l1_pscaler_H264_output_B_buffer_get(PSCALER_B);
        avi_encode_post_empty(scaler_frame_q, OutBuffer);
        drv_l1_pscaler_stop(PSCALER_B);
        //drv_l1_pscaler_clk_ctrl(PSCALER_B, 0);
        pscaler_exit_1 = 0;
    }

    if(PScaler_Event & PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR){
        DBG_PRINT("Pscaler-H264 Overflow\r\n");
        drv_l1_pscaler_stop(PSCALER_B);
        drv_l1_pscaler_interrupt_set(PSCALER_B, PIPELINE_SCALER_INT_ENABLE_ALL);
        pscaler_start_1 = 1;
        return;
    }

    if(!(PScaler_Event & (PIPELINE_SCALER_STATUS_FRAME_DONE |PIPELINE_SCALER_STATUS_MB420_FRAME_DONE)))
        return;

    if (PScaler_Event & PIPELINE_SCALER_STATUS_MB420_BUF_A_DONE)
    {
        OutBuffer = drv_l1_pscaler_H264_output_A_buffer_get(PSCALER_B);
        scaler_frame = avi_encode_get_empty(scaler_frame_q);
        if (scaler_frame)
            drv_l1_pscaler_H264_output_A_buffer_set(PSCALER_B,scaler_frame);
        else
        {
            DBG_PRINT("s");
            drv_l1_pscaler_H264_output_A_buffer_set(PSCALER_B,0x50000000);
        }
    }
    if (PScaler_Event & PIPELINE_SCALER_STATUS_MB420_BUF_B_DONE)
    {
        OutBuffer = drv_l1_pscaler_H264_output_B_buffer_get(PSCALER_B);
        scaler_frame = avi_encode_get_empty(scaler_frame_q);
        if (scaler_frame)
            drv_l1_pscaler_H264_output_B_buffer_set(PSCALER_B,scaler_frame);
        else
        {
            DBG_PRINT("s");
            drv_l1_pscaler_H264_output_B_buffer_set(PSCALER_B,0x50000000);
        }
    }

    if (OutBuffer)
    {
    #if VIDEO_TIMESTAMP
		VidRawFrame_t* fts;

		fts = (VidRawFrame_t*)avi_encode_get_empty(frame_ts_q);

		status = osErrorResource;
		if (fts)
		{
			fts->frame_addrs = OutBuffer;
			fts->pts = ts;
			status = osMessagePut(scaler_task_q, (INT32U)&fts, osWaitForever);
		}
    #else
		status = osMessagePut(scaler_task_q, (INT32U)&OutBuffer, osWaitForever);
	#endif
        if(status != osOK)
        {
            DBG_PRINT("PF");
            if (OutBuffer != 0x50000000)
				avi_encode_post_empty(scaler_frame_q, OutBuffer);
	#if VIDEO_TIMESTAMP
			if (fts)
				avi_encode_post_empty(frame_ts_q, (INT32U)fts);
	#endif
        }
    }

    if (pAviEncVidPara->scaler_zoom_ratio >= 10)
    {
        Pscaler_Digital_Zoom(PSCALER_B, pAviEncVidPara->encode_width, pAviEncVidPara->encode_height);
    }

    if (pscaler_exit_1)
    {
        pscaler_exit_1 = 0;
        OutBuffer = drv_l1_pscaler_H264_output_A_buffer_get(PSCALER_B);
        avi_encode_post_empty(scaler_frame_q, OutBuffer);
        OutBuffer = drv_l1_pscaler_H264_output_B_buffer_get(PSCALER_B);
        avi_encode_post_empty(scaler_frame_q, OutBuffer);
        drv_l1_pscaler_stop(PSCALER_B);
        drv_l1_pscaler_clk_ctrl(PSCALER_B, 0);
    }
}

static void Pscaler_YUYV_Callback(INT32U PScaler_Event)
{
    INT32U OutBuffer = 0;
    INT32U scaler_frame;
    osStatus status = osOK;
#if VIDEO_TIMESTAMP
	INT32U ts = vid_global_tick;
	static INT32U ts1 = 0;
#endif

    if(PScaler_Event & PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR)
    {
        if(pAviEncVidPara->jpeg_encode_enable_flag)
        {
            if(pAviEncVidPara->jpeg_encode_skip_frame == 0)
                DBG_PRINT("O");
            pAviEncVidPara->jpeg_encode_skip_frame = 1;
            encode_5M_flag++;
        }
        else
            DBG_PRINT("Pscaler Overflow\r\n");
    }

    if(pAviEncVidPara->jpeg_encode_enable_flag)
    {
        if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
        {
            OutBuffer = drv_l1_pscaler_output_A_buffer_get(PSCALER_B);
            //drv_l1_pscaler_H_first_enable(PSCALER_B, 1);
            //DBG_PRINT("A");
            if(pAviEncVidPara->jpeg_encode_skip_frame)
                pAviEncVidPara->jpeg_encode_skip_frame = 0;
        }
        else if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
        {
            OutBuffer = drv_l1_pscaler_output_B_buffer_get(PSCALER_B);
            //DBG_PRINT("B");
            if((OutBuffer != 0x50000000 && pAviEncVidPara->jpeg_encode_skip_frame == 0)
            || (OutBuffer != 0x50000000 && encode_5M_flag > C_ENCODE_5M_CNTOUT))
                drv_l1_pscaler_output_B_buffer_set(PSCALER_B, 0x50000000);
            else
            {
                OutBuffer = 0;
                pAviEncVidPara->jpeg_encode_skip_frame = 0;
            }
        }

        if(OutBuffer == 0x50000000)
            OutBuffer = 0;

        if(encode_5M_flag > C_ENCODE_5M_CNTOUT)
        {
            DBG_PRINT("D");
            pAviEncVidPara->jpeg_encode_start_flag = 0;
            OutBuffer = 0;
            encode_5M_flag = 0;
            pAviEncVidPara->jpeg_encode_timeout_exit = 1;
            pscaler_exit_1 = 1;
        }
    }
    else
    {
        if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
        {
            OutBuffer = drv_l1_pscaler_output_A_buffer_get(PSCALER_B);
            scaler_frame = avi_encode_get_empty(scaler_frame_q);
            if (scaler_frame)
                drv_l1_pscaler_output_A_buffer_set(PSCALER_B,scaler_frame);
            else
            {
                DBG_PRINT("s");
                drv_l1_pscaler_output_A_buffer_set(PSCALER_B,0x50000000);
            }
        }

        if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
        {
            OutBuffer = drv_l1_pscaler_output_B_buffer_get(PSCALER_B);
            scaler_frame = avi_encode_get_empty(scaler_frame_q);
            if (scaler_frame)
                drv_l1_pscaler_output_B_buffer_set(PSCALER_B,scaler_frame);
            else
            {
                DBG_PRINT("s");
                drv_l1_pscaler_output_B_buffer_set(PSCALER_B,0x50000000);
            }
        }
    }
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
    if (OutBuffer && (PScaler_Event & PIPELINE_SCALER_STATUS_FRAME_DONE))
    {
        OutBuffer |= C_AVI_ENCODE_FRAME_END;
    }
#endif

    if (OutBuffer)
    {
    #if VIDEO_TIMESTAMP
		VidRawFrame_t* fts;

		fts = (VidRawFrame_t*)avi_encode_get_empty(frame_ts_q);

		status = osErrorResource;
		if (fts)
		{
			fts->frame_addrs = OutBuffer;
			fts->pts = ts;
			status = osMessagePut(scaler_task_q, (INT32U)&fts, osWaitForever);
		}
    #else
		#if PALM_DEMO_EN == 1
        status = osMessagePut(display_frame_buffer_queue2, (INT32U)&OutBuffer, osWaitForever);
		#elif WAFD_DEMO_EN == 1
        status = osMessagePut(csi_frame_buffer_queue, (INT32U)&OutBuffer, osWaitForever);
        #else
		status = osMessagePut(scaler_task_q, (INT32U)&OutBuffer, osWaitForever);
		#endif
	#endif
        if(status != osOK) {
            DBG_PRINT("PF");
            if (OutBuffer != 0x50000000)
				avi_encode_post_empty(scaler_frame_q, OutBuffer);
	#if VIDEO_TIMESTAMP
			if (fts)
				avi_encode_post_empty(frame_ts_q, (INT32U)fts);
	#endif
        }

        if(pAviEncVidPara->jpeg_encode_enable_flag)
        {
            DBG_PRINT("C");
            pAviEncVidPara->jpeg_encode_start_flag = 0;
            encode_5M_flag = 0;
            pscaler_exit_1 = 1;
        }
    }

    if (pAviEncVidPara->scaler_zoom_ratio >= 10)
    {
        Pscaler_Digital_Zoom(PSCALER_B, pAviEncVidPara->encode_width, pAviEncVidPara->encode_height);
    }

    if (pscaler_exit_1)
    {
        pscaler_exit_1 = 0;
        if(pAviEncVidPara->jpeg_encode_enable_flag == 0)
        {
            OutBuffer = drv_l1_pscaler_output_A_buffer_get(PSCALER_B);
            avi_encode_post_empty(scaler_frame_q, OutBuffer);
            OutBuffer = drv_l1_pscaler_output_B_buffer_get(PSCALER_B);
            avi_encode_post_empty(scaler_frame_q, OutBuffer);
            drv_l1_pscaler_stop(PSCALER_B);
            //drv_l1_pscaler_clk_ctrl(PSCALER_B, 0);
        }
        else
        {
            drv_l1_pscaler_stop(PSCALER_B);
            drv_l1_pscaler_clk_ctrl(PSCALER_B, 0);
        }
#if VIDEO_TIMESTAMP
        ts1 = 0;
#endif
    }
}

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
static void Pscaler_FIFO_GP420_Callback(INT32U PScaler_Event)
{
    INT32U OutBuffer = 0;
    INT32U scaler_frame;
    osStatus status = osOK;

    if(PScaler_Event & PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR)
        DBG_PRINT("Ll\r\n");

    if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
    {
        OutBuffer = drv_l1_pscaler_output_A_buffer_get(PSCALER_B);
        scaler_frame = avi_encode_get_empty(jpeg_fifo_q);
        if (scaler_frame)
            drv_l1_pscaler_output_A_buffer_set(PSCALER_B,scaler_frame);
        else
        {
            DBG_PRINT("s");
            drv_l1_pscaler_output_A_buffer_set(PSCALER_B,0x50000000);
        }
    }
    if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
    {
        OutBuffer = drv_l1_pscaler_output_B_buffer_get(PSCALER_B);
        scaler_frame = avi_encode_get_empty(jpeg_fifo_q);
        if (scaler_frame)
            drv_l1_pscaler_output_B_buffer_set(PSCALER_B,scaler_frame);
        else
        {
            DBG_PRINT("s");
            drv_l1_pscaler_output_B_buffer_set(PSCALER_B,0x50000000);
        }
    }

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
    if (OutBuffer && (PScaler_Event & PIPELINE_SCALER_STATUS_FRAME_DONE))
    {
        OutBuffer |= C_AVI_ENCODE_FRAME_END;
        //DBG_PRINT("F");
    }
#endif

    if (OutBuffer)
    {
        status = osMessagePut(vid_enc_task_q, (INT32U)&OutBuffer, osWaitForever);
        if(status != osOK) {
        //DBG_PRINT("n ");
            if (OutBuffer != 0x50000000)
            {
            //DBG_PRINT("g ");
				avi_encode_post_empty(jpeg_fifo_q, OutBuffer);
            }
        }
    }

    if (pscaler_exit_1)
    {
        DBG_PRINT("E ");
        pscaler_exit_1 = 0;
        OutBuffer = drv_l1_pscaler_output_A_buffer_get(PSCALER_B);
        avi_encode_post_empty(jpeg_fifo_q, OutBuffer);
        OutBuffer = drv_l1_pscaler_output_B_buffer_get(PSCALER_B);
        avi_encode_post_empty(jpeg_fifo_q, OutBuffer);
        drv_l1_pscaler_stop(PSCALER_B);
        drv_l1_pscaler_clk_ctrl(PSCALER_B, 0);
    }
}
#endif

#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
#define TFT_DISPLAY_ROTATE_MODE ROTATOR_90
#define DISPLAY_DEVICE DISDEV_TFT
INT8U rotate_lock = 0;

static void rotate_task_entry(void const* param)
{
    osEvent event;
    osStatus status = osOK;
    INT32S drv_l2_ret = STATUS_OK, Ret = 0;
    INT32U Msg = 0, ack_msg, Buf = 0;
    INT32U display_buf, display_buf_update;
    portTickType xLastTickCount;

    rotator_mode_set(TFT_DISPLAY_ROTATE_MODE);
    xLastTickCount = xTaskGetTickCount();

    while(1)
    {
        event = osMessageGet(rotate_msg_q, osWaitForever);
        if((event.status != osEventMessage) || (event.value.v == 0)) {
            continue;
        }

        status = osOK;
        Msg = event.value.v;
        switch(Msg)
        {
        case MSG_ROTATE_TASK_STOP:
            DEBUG_MSG("[MSG_ROTATE_TASK_STOP]\r\n");
            OSQFlush(rotate_msg_q);
            ack_msg = ACK_OK;
            osMessagePut(rotate_ack_m, (INT32U)&ack_msg, osWaitForever);
            break;
        case MSG_ROTATE_TASK_EXIT:
            DEBUG_MSG("[MSG_ROTATE_TASK_EXIT]\r\n");
            ack_msg = ACK_OK;
            osMessagePut(rotate_ack_m, (INT32U)&ack_msg, osWaitForever);
            osThreadTerminate(osThreadGetId());
            break;
        default:
            Buf = Msg;
            rotator_end_wait(1);
            display_buf        = R_TFT_FBI_ADDR;
            display_buf_update = R_ROTATOR_BUF_O_ADDR;

            avi_encode_post_empty(rotate_frame_q,Buf);

            rotate_lock = 0;

            if(display_buf) avi_encode_post_empty(display_frame_q,display_buf);
            drv_l2_ret = drv_l2_display_update(DISPLAY_DEVICE, display_buf_update);
            if(drv_l2_ret != STATUS_OK)
            {
                avi_encode_post_empty(display_frame_q,display_buf_update);
                DBG_PRINT("Rotate task, update disp fail ,R 0x%x D 0x%x\r\n",R_ROTATOR_BUF_O_ADDR,R_TFT_FBI_ADDR);
            }

            Buf = 0;
            break;
        }
    }
}

void Pscaler_Display_Rotate_Callback(INT32U PScaler_Event)
{
    osStatus osRet = osOK;
    INT32S   Ret = SUCCESS;
    INT32U pscaleBuf_A, pscaleBuf_B, pscaleBuf = 0, dispBuf = 0;
    INT8U idx;
    BOOLEAN bDrop = 0;

    if(!(PScaler_Event & PIPELINE_SCALER_STATUS_FRAME_DONE)) return;

    if(PScaler_Event & PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR)
    {
        DBG_PRINT("Over(Rotate):PScaler_Event %x\r\n",PScaler_Event);
    }
    else if(PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
    {
        pscaleBuf_A = drv_l1_pscaler_output_A_buffer_get(PSCALER_A);

        //get empty Buf update to PScale
        pscaleBuf = avi_encode_get_empty(rotate_frame_q);

        if(pscaleBuf)
        {
            drv_l1_pscaler_output_A_buffer_set(PSCALER_A, pscaleBuf);
            dispBuf = avi_encode_get_empty(display_frame_q);
            if(dispBuf == 0) bDrop = 1;
        }

        if(pscaleBuf && !bDrop && rotate_lock == 0)
        {
            INT32U Buf = pscaleBuf_A;

            osRet = osMessagePut(rotate_msg_q, (uint32_t)&Buf, osWaitForever);
            if(osRet != osOK)
            {
                DBG_PRINT("PScaler rotate CB A, put Buf fail osRet 0x%x, unlock Buf 0x%x\r\n",osRet,Buf);
                bDrop = 1;
            }
            else
            {
                rotate_lock = 1;
                rotator_src_img_info(IMAGE_RGB565,pAviEncVidPara->display_buffer_width,pAviEncVidPara->display_buffer_height, Buf);
                rotator_tar_img_addr(dispBuf);
                rotator_start();
            }
        }

        if(bDrop || !rotate_lock)
        {
            DBG_PRINT("PScaler rotate CB A drop buf 0x%x, rotate_locked %d\r\n",pscaleBuf_A,rotate_lock);
            avi_encode_post_empty(rotate_frame_q,pscaleBuf_A);
        }
    }
    else if(PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
    {
        pscaleBuf_B = drv_l1_pscaler_output_B_buffer_get(PSCALER_A);

        //get empty Buf update to PScale
        pscaleBuf = avi_encode_get_empty(rotate_frame_q);

        if(pscaleBuf)
        {
            drv_l1_pscaler_output_B_buffer_set(PSCALER_A, pscaleBuf);
            dispBuf = avi_encode_get_empty(display_frame_q);
            if(dispBuf == 0) bDrop = 1;
        }

        if(!bDrop && rotate_lock == 0)
        {

            INT32U Buf = pscaleBuf_B;
            osRet = osMessagePut(rotate_msg_q, (uint32_t)&Buf, osWaitForever);
            if(osRet != osOK)
            {
                DBG_PRINT("PScaler rotate CB B put Buf fail, unlock Buf 0x%x osRet 0x%x\r\n",Buf,osRet);
                bDrop = 1;
            }
            else
            {
                rotate_lock = 1;
                rotator_src_img_info(IMAGE_RGB565,pAviEncVidPara->display_buffer_width,pAviEncVidPara->display_buffer_height, Buf);
                rotator_tar_img_addr(dispBuf);
                rotator_start();
            }
        }

        if(bDrop || !rotate_lock)
        {
            DBG_PRINT("PScaler rotate CB B drop buf 0x%x, rotate_locked %d\r\n",pscaleBuf_B,rotate_lock);
            avi_encode_post_empty(rotate_frame_q,pscaleBuf_B);
        }
    }

    if (pscaler_exit_0)
    {
        pscaler_exit_0 = 0;
        drv_l1_pscaler_stop(0);
    }
}
#endif

void scaler_disp_init()
{
    INT32U widthFactor,heightFactor;
    INT32U scaler_frame;
    INT32U out_width, out_height;
    INT32U x_start, y_start;
    drv_l2_sensor_para_t *pPara;

    drv_l1_pscaler_clk_ctrl(PSCALER_A,1);
	drv_l1_pscaler_init(PSCALER_A);

#if SCALER_CROP_CENTER == 1
    x_start = (pAviEncVidPara->sensor_capture_width - CENTER_WIDTH)/2;
	y_start = 0;

	widthFactor = ((CENTER_WIDTH*65536)/pAviEncVidPara->display_width);
	heightFactor = ((pAviEncVidPara->sensor_capture_height*65536)/pAviEncVidPara->display_height);

	drv_l1_pscaler_input_X_start_set(PSCALER_A, x_start);
	drv_l1_pscaler_input_Y_start_set(PSCALER_A, y_start);
	DBG_PRINT("cap(%d,%d), dis(%d,%d)\r\n",pAviEncVidPara->sensor_capture_height,pAviEncVidPara->sensor_capture_width,pAviEncVidPara->display_height,pAviEncVidPara->display_width);
#else
    widthFactor = ((pAviEncVidPara->sensor_capture_width*65536)/pAviEncVidPara->display_width);
	heightFactor = ((pAviEncVidPara->sensor_capture_height*65536)/pAviEncVidPara->display_height);
#endif
	//R_CDSP_DO |= (0x01 << 7);// Open CDSP data path




	pPara = drv_l2_sensor_get_para();
    if(pAviEncPara->sensor_interface == CSI_INTERFACE)
    {
        drvl1_csi_input_pscaler_set(1);
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CSI;
        drv_l1_pscaler_input_source_set(PSCALER_A, PIPELINE_SCALER_INPUT_SOURCE_CSI);
    }
    else
    {
        drv_l1_CdspSetYuvPscalePath(1);
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CDSP;
        drv_l1_pscaler_input_source_set(PSCALER_A, PIPELINE_SCALER_INPUT_SOURCE_CDSP);
    }

	drv_l1_pscaler_input_pixels_set(PSCALER_A, pAviEncVidPara->sensor_capture_width, pAviEncVidPara->sensor_capture_height);
	//drv_l1_pscaler_input_source_set(PSCALER_A, PIPELINE_SCALER_INPUT_SOURCE_CDSP);
	drv_l1_pscaler_input_buffer_set(PSCALER_A, 0x50000000);
	drv_l1_pscaler_input_format_set(PSCALER_A, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
	drv_l1_pscaler_output_fifo_line_set(PSCALER_A, pAviEncVidPara->display_height,0);
	drv_l1_pscaler_output_pixels_set(PSCALER_A, widthFactor, pAviEncVidPara->display_width, heightFactor, pAviEncVidPara->display_height);
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    scaler_frame = avi_encode_get_empty(rotate_frame_q);
    drv_l1_pscaler_output_A_buffer_set(PSCALER_A,scaler_frame);
    scaler_frame = avi_encode_get_empty(rotate_frame_q);
    drv_l1_pscaler_output_B_buffer_set(PSCALER_A,scaler_frame);
#else
    scaler_frame = avi_encode_get_empty(display_frame_q);
    if (scaler_frame)
        drv_l1_pscaler_output_A_buffer_set(0,scaler_frame);
    scaler_frame = avi_encode_get_empty(display_frame_q);
    if (scaler_frame)
        drv_l1_pscaler_output_B_buffer_set(0,scaler_frame);
#endif
    if(pAviEncVidPara->display_output_format == C_SCALER_CTRL_OUT_RGB565)
    drv_l1_pscaler_output_format_set(PSCALER_A, PIPELINE_SCALER_OUTPUT_FORMAT_RGB565);
    else if(pAviEncVidPara->display_output_format == C_SCALER_CTRL_OUT_UYVY)
        drv_l1_pscaler_output_format_set(PSCALER_A, PIPELINE_SCALER_OUTPUT_FORMAT_UYVY);
    else
        drv_l1_pscaler_output_format_set(PSCALER_A, PIPELINE_SCALER_OUTPUT_FORMAT_YUYV);

    drv_l1_pscaler_interrupt_set(PSCALER_A, PIPELINE_SCALER_INT_ENABLE_ALL);
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    drv_l1_pscaler_callback_register(PSCALER_A, Pscaler_Display_Rotate_Callback);
#else
    drv_l1_pscaler_callback_register(PSCALER_A, Pscaler_Display_Callback);
#endif
    pscaler_exit_0 = 0;

}

void scaler_video_init()
{
    INT32U i,widthFactor,heightFactor;
    INT32U scaler_frame;
    INT32U out_width, out_height;
	INT32U t_out_width, t_out_height;
    drv_l2_sensor_para_t *pPara;

	#if ((JPEG_X1_5_SCALE == 1)&&(VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE))
	t_out_width = (pAviEncVidPara->encode_width*2)/3;
	t_out_height = (pAviEncVidPara->encode_height*2)/3;
	#else
    t_out_width = pAviEncVidPara->sensor_capture_width;
    t_out_height = pAviEncVidPara->sensor_capture_height;
	#endif

	DBG_PRINT("t_in W %d H %d \r\n",t_out_width,t_out_height);

	if(pAviEncVidPara->jpeg_encode_enable_flag)
    {
        if(pAviEncVidPara->encode_different_flag)
        {
            out_width = pAviEncVidPara->jpeg_encode_width;
            out_height = pAviEncVidPara->jpeg_encode_height;
        }
        else
        {
        #if 0
            out_width = (t_out_width+ 0xf) & ~0xf;
            out_height = (t_out_height + 0xf) & ~0xf;
        #else
            out_width = pAviEncVidPara->encode_width;
            out_height = pAviEncVidPara->encode_height;
        #endif
        }
    }
	else
	{
        #if 0
            out_width = (t_out_width+ 0xf) & ~0xf;
            out_height = (t_out_height + 0xf) & ~0xf;
        #else
            out_width = pAviEncVidPara->encode_width;
            out_height = pAviEncVidPara->encode_height;
            out_height = (out_height + 0xf) >> 4 << 4;
        #endif
	}
    DBG_PRINT("t_out W %d H %d \r\n",out_width,out_height);

	widthFactor = ((t_out_width*65536)/out_width);
	heightFactor = ((t_out_height*65536)/out_height);

	drv_l1_pscaler_clk_ctrl(PSCALER_B,1);
	drv_l1_pscaler_init(PSCALER_B);

	pPara = drv_l2_sensor_get_para();
    if(pAviEncPara->sensor_interface == CSI_INTERFACE)
    {
        drvl1_csi_input_pscaler_set(1);
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CSI;
        drv_l1_pscaler_input_source_set(PSCALER_B, PIPELINE_SCALER_INPUT_SOURCE_CSI);
    }
    else
    {
        drv_l1_CdspSetYuvPscalePath(1);
        pPara->pscaler_src_mode = MODE_PSCALER_SRC_CDSP;
        drv_l1_pscaler_input_source_set(PSCALER_B, PIPELINE_SCALER_INPUT_SOURCE_CDSP);
    }

	drv_l1_pscaler_input_pixels_set(PSCALER_B, t_out_width, t_out_height);
//	drv_l1_pscaler_input_source_set(PSCALER_B, PIPELINE_SCALER_INPUT_SOURCE_CDSP);
	drv_l1_pscaler_input_buffer_set(PSCALER_B, 0x50000000);
	drv_l1_pscaler_input_format_set(PSCALER_B, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
	drv_l1_pscaler_output_fifo_line_set(PSCALER_B, t_out_height,0);
#elif VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
    drv_l1_pscaler_output_fifo_line_set(PSCALER_B, SENSOR_FIFO_LINE, 0);
#endif
	drv_l1_pscaler_output_pixels_set(PSCALER_B, widthFactor, out_width, heightFactor, out_height);

    if (pAviEncVidPara->video_format == C_H264_FORMAT)
    {
        scaler_frame = avi_encode_get_empty(scaler_frame_q);
        if (scaler_frame)
            drv_l1_pscaler_H264_output_A_buffer_set(PSCALER_B,scaler_frame);
        scaler_frame = avi_encode_get_empty(scaler_frame_q);
        if (scaler_frame)
            drv_l1_pscaler_H264_output_B_buffer_set(PSCALER_B,scaler_frame);

        drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_MB420);

        drv_l1_pscaler_interrupt_set(PSCALER_B, PIPELINE_SCALER_INT_ENABLE_ALL);
        drv_l1_pscaler_callback_register(PSCALER_B, Pscaler_H264_Callback);
    }
    else
    {
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE

		scaler_frame = avi_encode_get_empty(jpeg_fifo_q);
		if(scaler_frame)
			drv_l1_pscaler_output_A_buffer_set(PSCALER_B,scaler_frame);
		scaler_frame = avi_encode_get_empty(jpeg_fifo_q);
		if(scaler_frame)
			drv_l1_pscaler_output_B_buffer_set(PSCALER_B,scaler_frame);
		drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_GP420);
		drv_l1_pscaler_interrupt_set(PSCALER_B, PIPELINE_SCALER_INT_ENABLE_ALL);
		drv_l1_pscaler_callback_register(PSCALER_B, Pscaler_FIFO_GP420_Callback);

#else
        if(pAviEncVidPara->jpeg_encode_enable_flag)
        {
            if(pAviEncVidPara->jpeg_use_addr0_flag)
            {
                drv_l1_pscaler_output_A_buffer_set(PSCALER_B,0x50000000);
                drv_l1_pscaler_output_B_buffer_set(PSCALER_B,pAviEncVidPara->JPEGEncodeInBuf);
                if(pAviEncVidPara->jpeg_encode_in_format)
                    drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_GP420);
                else
                    drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_YUYV);

                for(i=0;i<50;i++)
                {
                    if(pAviEncVidPara->CSIOrgBufA != 0 && pAviEncVidPara->CSIOrgBufB != 0)
                    {
                        //DBG_PRINT("T");
                        break;
                    }
                    else
                        osDelay(10);
                }
            }
            else
            {
                scaler_frame = avi_encode_get_empty(scaler_frame_q);
                if (scaler_frame)
                    drv_l1_pscaler_output_A_buffer_set(PSCALER_B,scaler_frame);
                scaler_frame = avi_encode_get_empty(scaler_frame_q);
                if (scaler_frame)
                    drv_l1_pscaler_output_B_buffer_set(PSCALER_B,scaler_frame);
                drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_YUYV);
            }
        }
        else
        {
            scaler_frame = avi_encode_get_empty(scaler_frame_q);
            if (scaler_frame)
                drv_l1_pscaler_output_A_buffer_set(PSCALER_B,scaler_frame);
            scaler_frame = avi_encode_get_empty(scaler_frame_q);
            if (scaler_frame)
                drv_l1_pscaler_output_B_buffer_set(PSCALER_B,scaler_frame);
            drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_YUYV);
        }
        drv_l1_pscaler_interrupt_set(PSCALER_B, PIPELINE_SCALER_INT_ENABLE_ALL);
		drv_l1_pscaler_callback_register(PSCALER_B, Pscaler_YUYV_Callback);

#endif

    }

    if (pAviEncVidPara->video_format == C_H264_FORMAT)
    {
        R_SYSTEM_MISC_CTRL4 &= ~0x3F;
        R_SYSTEM_MISC_CTRL4 |= (0x0F|(0x1<<4));
    }


	pscaler_exit_1 = 0;
	if(pAviEncVidPara->jpeg_encode_enable_flag == 0)
	{
        //if (pAviEncVidPara->video_format == C_H264_FORMAT)
        pscaler_start_1 = 1;
		while(pscaler_start_1)
			osDelay(1);
    }
}

INT32U scaler_disp_post_empty(INT32U buf)
{
    return avi_encode_post_empty(display_frame_q, buf);
}

static void scaler_task_entry(void const *parm)
{
	INT32U msg_id, ack_msg;
	INT32U sensor_frame, scaler_frame, display_frame;
	ScalerFormat_t scale;
	ScalerPara_t para;
	osEvent result;
	osThreadId id;

	INT8U	i;
	INT32U buffer_ptr,frame_size,PPU_buffer_ptr;
	PPU_REGISTER_SETS ppu_register_structure;
	PPU_REGISTER_SETS *ppu_register_set;
	INT32S	nRet;

#if	AVI_ENCODE_SHOW_TIME == 1
	TIME_T osd_time;
#endif
    INT32U disp_bit = 0;
    INT32U first_frames = 0, state;
#if VIDEO_TIMESTAMP
	VidRawFrame_t* frame_ts;
#endif

    DEBUG_MSG("<<%s>>\r\n", __func__);

	drv_l2_scaler_init();
	while(1)
	{
		result = osMessageGet(scaler_task_q, osWaitForever);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (	msg_id == 0)) {
			continue;
		}

		switch(msg_id)
		{
		case MSG_SCALER_TASK_INIT:
			DEBUG_MSG("[MSG_SCALER_TASK_INIT]\r\n");
			display_frame = 0;
			pAviEncPara->fifo_enc_err_flag = 0;
			pAviEncPara->fifo_irq_cnt = 0;
			pAviEncPara->vid_pend_cnt = 0;
			pAviEncPara->vid_post_cnt = 0;
			first_frames = 2;
            #if C_PPU_DRAW_EN == 1
                avi_ppu_draw_init(pAviEncVidPara->encode_width, pAviEncVidPara->encode_height);
            #endif

			pTH32x32_Para->TH32x32_CMOS_OFF = 0;

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
			gplib_ppu_free_size_set(ppu_register_set, 1, 320, 240);
			gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
			gplib_ppu_long_burst_set(ppu_register_set, 1);
			gplib_ppu_yuv_type_set(ppu_register_set, 3);// value[1:0]:0=BGRG/VYUY 1=GBGR/YVYU 2=RGBG/UYVY 3=GRGB/YUYV, value[2]:0=UV is unsigned(YCbCr) 1=UV is signed(YUV)
			gplib_ppu_long_burst_set(ppu_register_set, 1);
			

			//Frame buffer malloc
			//frame_size = (PPU_TEXT_SIZE_HPIXEL * PPU_TEXT_SIZE_VPIXEL * 2);
			frame_size = (320 * 240 * 2);
			//PPU_buffer_ptr = (INT32U) gp_malloc_align(frame_size*C_PPU_DRV_FRAME_NUM, 64); // from 320B	
		    PPU_buffer_ptr = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
			if(!PPU_buffer_ptr)
			{
				DBG_PRINT("failed to allocate frame buffer memory =>  PPU_buffer_ptr \r\n");
				while(1);
			}
		    PPU_buffer_ptr = (PPU_buffer_ptr + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64;
			
			
			prcess_mem_set->ppu_frame_workmem = PPU_buffer_ptr;
			for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
				buffer_ptr = (INT32U)(PPU_buffer_ptr + (i*frame_size));
				gplib_ppu_frame_buffer_add(ppu_register_set, buffer_ptr);
				DBG_PRINT("PPU_buffer_ptr[%d] -> 0x%x\r\n",i,buffer_ptr);
			}
			
			drv_l2_display_buffer_set(DISPLAY_DEVICE, buffer_ptr);

			// PPU init End ***

			// PPU setting start
			DBG_PRINT("PPU setting start \r\n");

			// Now configure TEXT relative elements
			gplib_ppu_text_compress_disable_set(ppu_register_set, 1) ;    // Disable TEXT1/TEXT2 horizontal/vertical compress function
			gplib_ppu_text_direct_mode_set(ppu_register_set, 0);                //Disable TEXT direct address mode

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
			gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT1, 1); // Enable TH TEXT
			gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT1, 1);     // Enable bitmap mode
			gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT1, 1);    // Get TEXT attribute from register
			gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT1, 1, 3);     // Set TEXT color to  < 1:RGB / 3:YUYV >
			//gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 0);             // Set TEXT size to 512x256
			gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 2);             // Set TEXT size to 1024x512

			gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT1, 0);    // Set TEXT segment address
			//gplib_ppu_rgb565_transparent_color_set(ppu_register_set, 1, TRANSPARENT_COLOR);	// TRANSPARENT_COLOR = 0x00CD
			gplib_ppu_rgb565_transparent_color_set(ppu_register_set, 1, TRANSPARENT_COLOR);

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
			gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT2, 1);                        // Enable CMOS TEXT
			gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT2, 1);     // Enable bitmap mode
			gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT2, 1);    // Get TEXT attribute from register
			gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT2, 1, 3);     // Set TEXT color to < 1:RGB / 3:YUYV >
			//gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT2, 0);             // Set TEXT size to 512x256
			gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT2, 2);             // Set TEXT size to 1024x512

			gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT2, 0);    // Set TEXT segment address
			gplib_ppu_text_blend_set(ppu_register_set, C_PPU_TEXT2, 1, 1, 32);  // Set Blend

			//gplib_ppu_text_depth_set(ppu_register_set, C_PPU_TEXT2, 2);      // Set TEXT Depth 1



			DBG_PRINT("PPU setting End \r\n");

			// PPU setting End

			//pTH32x32_Para->TH32x32_ScalerUp_status = 1;
			gp_memcpy((INT8S *)(pTH32x32_Para->TH32x32_ColorOutputFrame_addr),
				(INT8S *)&(sensor32X32_RGB565),32*32*2);

			//picCNT=0;

			ack_msg = ACK_OK;
			osMessagePut(scaler_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;

		case MSG_SCALER_TASK_STOP:
			DEBUG_MSG("[MSG_SCALER_TASK_STOP]\r\n");
			OSQFlush(scaler_task_q);
			ack_msg = ACK_OK;
			osMessagePut(scaler_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;

		case MSG_SCALER_TASK_EXIT:
			DEBUG_MSG("[MSG_SCALER_TASK_EXIT]\r\n");
			#if C_PPU_DRAW_EN == 1
                avi_ppu_draw_uninit();
			#endif
			ack_msg = ACK_OK;
			osMessagePut(scaler_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			id = osThreadGetId();
    		osThreadTerminate(id);
			break;

			/*
		case MSG_SCALER_TASK_TH32x32Ready:
			//TH32x32_TEST_HIGH();
			pTH32x32_Para->TH32x32_ScalerUp_status = 1;
			ack_msg = ACK_OK;
			osMessagePut(scaler_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			//OSMboxPost(scaler_task_ack_m, (void*)C_ACK_SUCCESS);
			//DBG_PRINT("[ MSG_SCALER_TASK_TH32x32Ready] ");
			//osd_time.tm_sec= (INT32S) picCNT++;	// 計時用 
			//TH32x32_TEST_LOW();
			break;
			*/

		default:
            disp_bit = msg_id & 0x80000000;
            #if VIDEO_TIMESTAMP
            if (disp_bit)
				sensor_frame = msg_id & 0x7fffffff;
			else {
				frame_ts = (VidRawFrame_t*)msg_id;
				sensor_frame = frame_ts->frame_addrs;
			}
            #else
			sensor_frame = msg_id & 0x7fffffff;
			#endif
		 	if(/*AVI_ENCODE_DIGITAL_ZOOM_EN == 1 || */pAviEncVidPara->scaler_flag > 0) {
		 		scaler_frame = avi_encode_get_empty(scaler_frame_q);
    			if(scaler_frame == 0) {
					avi_encode_post_empty(cmos_frame_q, sensor_frame);
    				DEBUG_MSG("scaler_frame Fail!!!\r\n");
					goto DEFAULT_END;
				}

				scale.input_format = pAviEncVidPara->sensor_output_format;
				scale.input_width = pAviEncVidPara->sensor_capture_width;
				scale.input_height = pAviEncVidPara->sensor_capture_height;
				scale.input_visible_width = pAviEncVidPara->sensor_capture_width;
				scale.input_visible_height = pAviEncVidPara->sensor_capture_height;
				scale.input_x_offset = 0;
				scale.input_y_offset = 0;

				scale.input_y_addr = sensor_frame;
				scale.input_u_addr = 0;
				scale.input_v_addr = 0;

				scale.output_format = C_SCALER_CTRL_OUT_YUYV;
				scale.output_width = pAviEncVidPara->encode_width;
				scale.output_height = pAviEncVidPara->encode_height;
				scale.output_buf_width = pAviEncVidPara->encode_width;
				scale.output_buf_height = pAviEncVidPara->encode_height;
				scale.output_x_offset = 0;

				scale.output_y_addr = scaler_frame;
				scale.output_u_addr = 0;
				scale.output_v_addr = 0;

			/*#if AVI_ENCODE_DIGITAL_ZOOM_EN == 1
				scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
				scale.scale_mode = C_SCALER_FULL_SCREEN_BY_DIGI_ZOOM;
				scale.digizoom_m = (INT8U)(pAviEncVidPara->scaler_zoom_ratio * 10);
				scale.digizoom_n = 10;
			#else*/
				scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
				scale.scale_mode = C_SCALER_FULL_SCREEN_BY_RATIO;
				scale.digizoom_m = 10;
				scale.digizoom_n = 10;
			//#endif
				memset((void *)&para, 0x00, sizeof(para));
				para.boundary_color = 0x008080;

    			drv_l2_scaler_trigger(SCALER_0, 1, &scale, &para);

				avi_encode_post_empty(cmos_frame_q, sensor_frame);
			} else {
				scaler_frame = sensor_frame;
			}

		#if APP_QRCODE_BARCODE_EN == 1
	    	code_decoder_set_frame(scaler_frame,
								pAviEncVidPara->encode_width,
								pAviEncVidPara->encode_height,
								BITMAP_YUYV);
		#endif

            //skip first frames
			if (first_frames) {
                first_frames --;
                if (!disp_bit) {
                #if VIDEO_TIMESTAMP
					avi_encode_post_empty(frame_ts_q, (INT32U)frame_ts);
                #endif
                    avi_encode_post_empty(scaler_frame_q, scaler_frame);
				}
				else{
                    avi_encode_post_empty(display_frame_q, scaler_frame); // davis run ?
                    DBG_PRINT("skip first frames, %x\r\n",first_frames);
					}
                break;
            }

            if(uvc_frame_q && pAviEncVidPara->video_format == 0){	//pAviEncVidPara->video_format==0 means uncompressed?
                INT32U status;
                status = uvc_post_cur_frame(uvc_frame_q, scaler_frame);

                if(status)
                    DBG_PRINT("post using fail, %x\r\n",status);

            }

		#if AVI_ENCODE_PREVIEW_DISPLAY_EN == 1
			//if(pAviEncVidPara->dispaly_scaler_flag == 0) {
			if (disp_bit) {
				display_frame = scaler_frame;
			#if	AVI_ENCODE_SHOW_TIME == 1
				cal_time_get(&osd_time);
				cpu_draw_time_osd(osd_time, display_frame, pAviEncVidPara->display_width);
			#endif

			#if TH32x32IMAGE

				if(videnc_display) { // davis run this //
		    		videnc_display(pAviEncVidPara->display_buffer_width,
		    					   pAviEncVidPara->display_buffer_height,
		    					   pTH32x32_Para->TH32x32_display_frame);
					if(pTH32x32_Para->TH32x32_ScalerUp_status == 1)
						pTH32x32_Para->TH32x32_ScalerUp_status = 0;
		    	}
			#else
		    	if(videnc_display) { // davis run this

			#if 1

					// PPU 處理
				
					if(pTH32x32_Para->TH32x32_ScalerUp_status == 1){
						gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, 320, 240, (INT32U)pTH32x32_Para->TH32x32_display_frame); // from TH32x32 sensor 
						pTH32x32_Para->TH32x32_ScalerUp_status = 0;
						//DEBUG_MSG(" C \r\n");
					}
					//DEBUG_MSG("display_frame=0x%x \r\n",display_frame);
					gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT2, 320, 240, (INT32U)display_frame);	 // from CMOS sensor
					//DEBUG_MSG("PPU-s\r\n");
					nRet =  gplib_ppu_go(ppu_register_set);	// 2018.07.16 不等待執行結束
					//nRet =  gplib_ppu_go_and_wait_done(ppu_register_set);
					
					
					
					//video_encode_display_frame_ready(PPU_buffer_ptr);
					DEBUG_MSG("PPU-nRet 0x%x \r\n",nRet);
					//}

					videnc_display(pAviEncVidPara->display_buffer_width,
		    					   pAviEncVidPara->display_buffer_height,
		    					   PPU_buffer_ptr);
			#else
			
		    		videnc_display(pAviEncVidPara->display_buffer_width,
		    					   pAviEncVidPara->display_buffer_height,
		    					   display_frame);
			#endif
		    	}

			#endif

		    	if(videnc_buferr_post)
		    	{
                    scaler_frame = videnc_buferr_post(display_frame);
                    if(scaler_frame)
                        display_frame = scaler_frame;
					
		    	}
				
				//DEBUG_MSG("post display_frame=0x%x \r\n",display_frame);
		    	avi_encode_post_empty(display_frame_q, display_frame);
			} /*else {
				display_frame = avi_encode_get_empty(display_frame_q);
				if(display_frame == 0) {
					DEBUG_MSG("display_frame = 0\r\n");
					goto DEFAULT_END;
				}

				scale.input_format = C_SCALER_CTRL_IN_YUYV;
				scale.input_width = pAviEncVidPara->encode_width;
				scale.input_height = pAviEncVidPara->encode_height;
				scale.input_visible_width = pAviEncVidPara->encode_width;
				scale.input_visible_height = pAviEncVidPara->encode_height;
				scale.input_x_offset = 0;
				scale.input_y_offset = 0;

				scale.input_y_addr = scaler_frame;
				scale.input_u_addr = 0;
				scale.input_v_addr = 0;

				scale.output_format = pAviEncVidPara->display_output_format;
				scale.output_width = pAviEncVidPara->display_width;
				scale.output_height = pAviEncVidPara->display_height;
				scale.output_buf_width = pAviEncVidPara->display_buffer_width;
				scale.output_buf_height = pAviEncVidPara->display_buffer_height;
				scale.output_x_offset = 0;

				scale.output_y_addr = display_frame;
				scale.output_u_addr = 0;
				scale.output_v_addr = 0;

				scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
				scale.scale_mode = C_SCALER_FULL_SCREEN;
				scale.digizoom_m = 10;
				scale.digizoom_n = 10;

				memset((void *)&para, 0x00, sizeof(para));
				para.boundary_color = 0x008080;

    			drv_l2_scaler_trigger(SCALER_0, 1, &scale, &para);

				//post ready frame to video encode task
				if(scaler_frame) {
					osMessagePut(vid_enc_task_q, (INT32U)&scaler_frame, osWaitForever);
					scaler_frame = 0;
				}

			#if	AVI_ENCODE_SHOW_TIME == 1
				cal_time_get(&osd_time);
				cpu_draw_time_osd(osd_time, display_frame, pAviEncVidPara->display_width);
			#endif
		    	if(videnc_display) {
		    		videnc_display(pAviEncVidPara->display_buffer_width,
		    					   pAviEncVidPara->display_buffer_height,
		    					   display_frame);
		    	}

				avi_encode_post_empty(display_frame_q, display_frame);
        	}*/
		#endif
		DEFAULT_END:
			//post ready frame to video encode task
			if(scaler_frame && !disp_bit) {
			#if VIDEO_TIMESTAMP
				osMessagePut(vid_enc_task_q, (INT32U)&frame_ts, osWaitForever);
				frame_ts = 0;
			#else
#if C_PPU_DRAW_EN == 1
                result = osMessageGet(prcess_draw_queue, 5);
                state = result.value.v;
                if((result.status != osEventMessage) || (state!=0x80)) {
                   state = 0;
                }

                if(state)
                {
                    obj_id = obj_recong_draw.recong_id;
                    pos_x = obj_recong_draw.center_x;
                    pos_y = obj_recong_draw.center_y;
                }

                //if(state)
                {
                    #if C_PPU_DRAW_EN == 1
                        #if C_PPU_UI_EN == 1
                            //obj_recong_draw->recong_id
                            if(obj_id)
                            avi_ppu_ui_draw_object_mode(1,obj_id,640,480,scaler_frame);
                        #else
                            #if C_PPU_SPRITE_EN == 1
                                avi_ppu_sprite_draw_object_mode(1,10);
                            #endif
                            avi_ppu_draw_go(pAviEncVidPara->encode_width, pAviEncVidPara->encode_height, scaler_frame);
                            avi_encode_post_empty(scaler_frame_q, scaler_frame);
                            scaler_frame = ppu_frame_buffer_display_get();
                        #endif
                    #endif

                }
#endif
                osMessagePut(vid_enc_task_q, (INT32U)&scaler_frame, osWaitForever);
			#endif
                scaler_frame = 0;
			}

		#if 0
			// PPU 處理 ??

			if(pTH32x32_Para->TH32x32_ScalerUp_status == 1){
				//gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, PPU_TEXT_SIZE_HPIXEL, PPU_TEXT_SIZE_VPIXEL, (INT32U)pTH32x32_Para->TH32x32_display_frame); // from TH32x32 sensor
				gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, 640, 480, (INT32U)pTH32x32_Para->TH32x32_display_frame); // from TH32x32 sensor
				//osd_time.tm_sec= (INT32S) picCNT++;	// 計時用 
				pTH32x32_Para->TH32x32_ScalerUp_status = 0;
			}
		#endif

			//pTH32x32_Para->TH32x32_ScalerUp_status = 0; // 暫時 set 0 ?
			break;
		}
	}
}

//	video encode task
INT32S video_encode_task_create(INT8U pori)
{
	INT32S nRet;
	osThreadId id;
	osThreadDef_t video_task = { "video_task", video_encode_task_entry, osPriorityNormal, 1, C_JPEG_STACK_SIZE };

	if(vid_enc_task_q == 0) {
		osMessageQDef_t videnc_q = {C_JPEG_QUEUE_MAX*2, sizeof(INT32U), 0};//queue size double for possible null frame

		vid_enc_task_q = osMessageCreate(&videnc_q, NULL);
		if(vid_enc_task_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(vid_enc_task_ack_m == 0) {
		osMessageQDef_t vidack_q = {C_ACK_QUEUE_MAX, sizeof(INT32U), 0};

		vid_enc_task_ack_m = osMessageCreate(&vidack_q, NULL);
		if(vid_enc_task_ack_m == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(vid_enc_frame_q == 0) {
		osMessageQDef_t videnc_f_q = {AVI_ENCODE_VIDEO_BUFFER_NO, sizeof(INT32U), 0};

		vid_enc_frame_q = osMessageCreate(&videnc_f_q, NULL);
		if(vid_enc_frame_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(video_frame_q == 0) {
		osMessageQDef_t vid_f_q = {AVI_ENCODE_VIDEO_BUFFER_NO, sizeof(INT32U), 0};

		video_frame_q = osMessageCreate(&vid_f_q, NULL);
		if(video_frame_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}

	if(video_stream_q == 0) {
		osMessageQDef_t vid_f_q = {AVI_ENCODE_VIDEO_BUFFER_NO, sizeof(INT32U), 0};

		video_stream_q = osMessageCreate(&vid_f_q, NULL);
		if(video_stream_q == 0) {
			RETURN(STATUS_FAIL);
		}
	}


	id = osThreadCreate(&video_task, (void *)NULL);
	if(id == 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S video_encode_task_del(void)
{
	INT32S nRet;

	nRet = STATUS_OK;
	POST_MESSAGE(vid_enc_task_q, MSG_VIDEO_ENCODE_TASK_EXIT, vid_enc_task_ack_m, 5000);
Return:
	OSQFlush(vid_enc_task_q);
	vQueueDelete(vid_enc_task_q);
	vid_enc_task_q = 0;

	OSQFlush(vid_enc_task_ack_m);
	vQueueDelete(vid_enc_task_ack_m);
	vid_enc_task_ack_m = 0;

   	OSQFlush(vid_enc_frame_q);
   	vQueueDelete(vid_enc_frame_q);
	vid_enc_frame_q = 0;

   	OSQFlush(video_frame_q);
   	vQueueDelete(video_frame_q);
	video_frame_q = 0;

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	OSQFlush(jpeg_fifo_q);
	vQueueDelete(jpeg_fifo_q);
	jpeg_fifo_q = 0;
#endif

	return nRet;
}

INT32S video_encode_task_start(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(vid_enc_task_q, MSG_VIDEO_ENCODE_TASK_INIT, vid_enc_task_ack_m, 5000);
Return:
	return nRet;
}

INT32S video_encode_task_stop(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(vid_enc_task_q, MSG_VIDEO_ENCODE_TASK_STOP, vid_enc_task_ack_m, 5000);
Return:
    return nRet;
}

INT32S video_encode_task_reset(void)
{
	INT32S nRet = STATUS_OK;

	POST_MESSAGE(vid_enc_task_q, MSG_VIDEO_ENCODE_TASK_RESET, vid_enc_task_ack_m, 5000);
Return:
    return nRet;
}

INT32S video_encode_task_post_q(INT32U mode)
{
	INT32U i;

	if(mode == 0) {
		//avi queue
		OSQFlush(vid_enc_frame_q);
    	OSQFlush(video_frame_q);
#ifdef VFRAME_MANAGER
    	vfm_reset(&pAviEncVidPara->vfm);
#else
    	for(i=0; i<AVI_ENCODE_VIDEO_BUFFER_NO; i++) {
			osMessagePut(video_frame_q, (INT32U)&pAviEncVidPara->video_encode_addr[i], osWaitForever);
		}
#endif
	} else {
		OSQFlush(video_stream_q);
	    for(i=0; i<AVI_ENCODE_VIDEO_BUFFER_NO; i++) {
			osMessagePut (video_stream_q, (INT32U)&pAviEncVidPara->video_encode_addr[i], osWaitForever);
		}

	}
	return STATUS_OK;
}

INT32S video_encode_task_empty_q(void)
{
	INT32U video_frame;
	VidEncFrame_t * pVideo;

	do {
		video_frame = avi_encode_get_empty(vid_enc_frame_q);
#ifndef VFRAME_MANAGER
		if(video_frame) {
			pVideo = (VidEncFrame_t *)video_frame;
			avi_encode_post_empty(video_frame_q, pVideo->ready_frame);
		}
#endif
	} while(video_frame);
#ifdef VFRAME_MANAGER
	vfm_reset(&pAviEncVidPara->vfm);
#endif
	return STATUS_OK;
}

static void video_encode_task_entry(void const *parm)
{
	INT8U   rCnt=0;
	INT32S  header_size=0, encode_size;
	INT32U  msg_id, ack_msg;
	INT32U	video_frame, scaler_frame;
	JpegPara_t jpeg;
	osEvent result;
	osThreadId id;
#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	INT8U  EncMode, jpeg_start_flag;
	INT16U scaler_height;
	INT32U input_y_len, input_uv_len, uv_length;
	INT32U y_frame, u_frame, v_frame;
	INT32S status;
	ScalerFormat_t scale;
	ScalerPara_t para;
#endif
#if	AVI_ENCODE_SHOW_TIME == 1
	TIME_T osd_time;
#endif
    INT32S FrmCnt, FirstFrm, NullFrm = 0;
    INT64S dwtemp;
    INT32U t1, t2;
    INT32U key_flag;
#if VIDEO_TIMESTAMP
	VidRawFrame_t* frame_ts;
#endif

    DEBUG_MSG("<<%s>>\r\n", __func__);

	//R_IOC_DIR |= 0x0C; R_IOC_ATT |= 0x0C; R_IOC_O_DATA = 0x0;
	while(1)
	{
		result = osMessageGet(vid_enc_task_q, osWaitForever);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (msg_id == 0)) {
		    continue;
		}

		switch(msg_id)
		{
		case MSG_VIDEO_ENCODE_TASK_INIT:
			DEBUG_MSG("[MSG_VIDEO_ENCODE_TASK_INIT]\r\n");
			rCnt = 0;
		#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
			#if JPEG_X1_5_SCALE == 1
			pAviEncPara->vid_post_cnt = (pAviEncVidPara->sensor_capture_height + (SENSOR_FIFO_LINE - 1))/ SENSOR_FIFO_LINE;
			DBG_PRINT("pAviEncPara->vid_post_cnt %d \r\n",pAviEncPara->vid_post_cnt);
			DBG_PRINT("pAviEncVidPara->sensor_capture_height %d \r\n",pAviEncVidPara->sensor_capture_height);
			#else
			pAviEncPara->vid_post_cnt = pAviEncVidPara->sensor_capture_height / SENSOR_FIFO_LINE;
			#endif

			//if(pAviEncVidPara->sensor_capture_height % SENSOR_FIFO_LINE) {
			//	while(1);//this must be no remainder
			//}
/*
			if(pAviEncPara->vid_post_cnt > AVI_ENCODE_CSI_FIFO_NO) {
				while(1);//fifo buffer is not enough
			}
*/
			jpeg_start_flag = 0;
			//scaler_height = pAviEncVidPara->encode_height / pAviEncPara->vid_post_cnt;
			scaler_height = SENSOR_FIFO_LINE;
			#if JPEG_X1_5_SCALE == 1
			uv_length = ((pAviEncVidPara->encode_width * 2)/3) * scaler_height;
			input_y_len = ((pAviEncVidPara->encode_width * 2)/3) * scaler_height;
			DBG_PRINT("uv_length %d, %d \r\n",uv_length, ((pAviEncVidPara->encode_width * 2)/3));
			input_uv_len = input_y_len >> 2; //YUV420
			#else
			uv_length = pAviEncVidPara->encode_width * (scaler_height + 2);
			input_y_len = pAviEncVidPara->encode_width * scaler_height ;
			input_uv_len = input_y_len >> 1; //YUV422
			#endif
		#endif
		#if AVI_ENCODE_VIDEO_ENCODE_EN == 1
            header_size = avi_encode_set_jpeg_quality(pAviEncVidPara->quality_value);

            FrmCnt = 0;
            FirstFrm = 1;
            //header_size = 0;
            #if VIDEO_ENCODE_MODE != C_VIDEO_ENCODE_FIFO_MODE
            if (0 > h264_encode_start(pAviEncVidPara->encode_width, pAviEncVidPara->encode_height))
            {
                DEBUG_MSG("H264 Start Fail\r\n");
                while(1);
            }
            #endif
            encode_time = 0;
            pAviEncPara->encoded_cnt = 0;
		#endif
			ack_msg = ACK_OK;
			osMessagePut(vid_enc_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;
        case MSG_VIDEO_ENCODE_TASK_RESET:
            if (pAviEncVidPara->video_format == C_H264_FORMAT)
            {
                FrmCnt = 0;
                FirstFrm = 1;
                header_size = 0;
                h264_encode_reset_rc();
            }
            else {
				if (!header_size)
                	header_size = avi_encode_set_jpeg_quality(pAviEncVidPara->quality_value);
            }
            encode_time = 0;
            pAviEncPara->encoded_cnt = 0;
            ack_msg = ACK_OK;
			osMessagePut(vid_enc_task_ack_m, (INT32U)&ack_msg, osWaitForever);
            break;
		case MSG_VIDEO_ENCODE_TASK_STOP:
			DEBUG_MSG("[MSG_VIDEO_ENCODE_TASK_STOP]\r\n");
            h264_encode_stop();
			OSQFlush(vid_enc_task_q);
			ack_msg = ACK_OK;
			osMessagePut(vid_enc_task_ack_m, (INT32U)&ack_msg, osWaitForever);
			break;

		case MSG_VIDEO_ENCODE_TASK_EXIT:
			DEBUG_MSG("[MSG_VIDEO_ENCODE_TASK_EXIT]\r\n");
			ack_msg = ACK_OK;
			osMessagePut(vid_enc_task_ack_m, (INT32U)&ack_msg, osWaitForever);

			id = osThreadGetId();
    		osThreadTerminate(id);
			break;

		default:

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
		#if VIDEO_TIMESTAMP
			frame_ts = (VidRawFrame_t*)msg_id;
			scaler_frame = frame_ts->frame_addrs;
		#else
			scaler_frame = msg_id;
		#endif
			if (scaler_frame == 0x50000000)
				NullFrm = 1;
			else
				NullFrm = 0;

			if (NullFrm && pAviEncVidPara->video_format == C_MJPG_FORMAT) {
				video_frame = DUMMY_BUFFER_ADDRS;
			} else if (avi_encode_get_status()&C_AVI_ENCODE_STREAM) {
                    //if(pAviEncVidPara->video_format != C_H264_FORMAT)
                    header_size = pAviEncVidPara->encode_header_size;
#ifdef VFRAME_MANAGER
					if(pAviEncVidPara->jpeg_encode_enable_flag &&
						pAviEncVidPara->encode_different_flag)
						video_frame = vfm_get_empty(&pAviEncVidPara->vfm, pAviEncVidPara->jpeg_encode_width * pAviEncVidPara->jpeg_encode_height);
					else
						video_frame = vfm_get_empty(&pAviEncVidPara->vfm, 0);
#else
                    if(pAviEncVidPara->jpeg_encode_enable_flag &&
                    pAviEncVidPara->encode_different_flag)
                    {
                        if(pAviEncVidPara->jpeg_use_addr0_flag)
                            video_frame = pAviEncVidPara->JPEGEncodeOutBuf;
                        else
                            video_frame = avi_encode_get_empty(video_stream_q);
                    }
                    else
                        video_frame = avi_encode_get_empty(video_stream_q);
#endif
			}
			else if(avi_encode_get_status()&C_AVI_ENCODE_PRE_START) {
#ifdef VFRAME_MANAGER
				video_frame = vfm_get_empty(&pAviEncVidPara->vfm, 0);
#else
				video_frame = avi_encode_get_empty(video_frame_q);
#endif
			} else {
				goto VIDEO_ENCODE_FRAME_MODE_END;
			}

			if(video_frame == 0) {
				DBG_PRINT("c");
				//DEBUG_MSG(DBG_PRINT("video_frame = 0x%x\r\n", video_frame));
				goto VIDEO_ENCODE_FRAME_MODE_END;
			}

			//if(pAviEncVidPara->video_format == C_MJPG_FORMAT) {
			#if	AVI_ENCODE_SHOW_TIME == 1
				if(pAviEncVidPara->dispaly_scaler_flag == 1) {
					cal_time_get(&osd_time);
					cpu_draw_time_osd(osd_time, scaler_frame, pAviEncVidPara->encode_width);
				}
			#endif

            t1 = xTaskGetTickCount();
			if(pAviEncVidPara->video_format == C_MJPG_FORMAT) {
				jpeg.quality_value = pAviEncVidPara->quality_value;
				jpeg.input_format = C_JPEG_FORMAT_YUYV;
				if(pAviEncVidPara->jpeg_encode_enable_flag)
				{
                    if(pAviEncVidPara->encode_different_flag)
                    {
                        jpeg.width = pAviEncVidPara->jpeg_encode_width;
                        jpeg.height = pAviEncVidPara->jpeg_encode_height;
                    }
                    else
                    {
                        jpeg.width = pAviEncVidPara->encode_width;
                        jpeg.height = pAviEncVidPara->encode_height;
                    }
				}
                else
                {
                    jpeg.width = pAviEncVidPara->encode_width;
                    jpeg.height = pAviEncVidPara->encode_height;
                }
				jpeg.input_buffer_y = scaler_frame;
				jpeg.input_buffer_u = 0;
				jpeg.input_buffer_v = 0;
				jpeg.output_buffer = video_frame + header_size;
				key_flag = 0;
				if (NullFrm) {
                    encode_size = 0;
                } else {
                    encode_size = jpeg_encode_once(&jpeg);
                }
				if(encode_size < 0) {
					DEBUG_MSG("encode_size = 0\r\n");
					goto VIDEO_ENCODE_FRAME_MODE_END;
				}
				if (encode_size) {
#ifdef VFRAME_MANAGER
					gp_memcpy((void *)video_frame, (void *)jpeg_header_get_addr(), header_size);
#endif
					encode_size += header_size;
				}
            }
            else if(pAviEncVidPara->video_format == C_H264_FORMAT) {
                INT32S type, hdr, skip = 0;

                type = hdr = 0;
                if (FrmCnt > 0)
                    type = 1;

				if (NullFrm)
				{
					if (!(FrmCnt & 0x1) || FrmCnt >= (pAviEncVidPara->h264_gop_len-1))
					{
						encode_size = -1;
						goto VIDEO_ENCODE_FRAME_MODE_END;
					}
				}

                key_flag = type;
                if (type == 0)
					hdr = 1;

				__disable_irq();
				if (pAviEncPara->tv <= pAviEncPara->Tv) //for frame rate control
					skip = 1;
				__enable_irq();

				if (skip) {
				#ifdef VFRAME_MANAGER
					vfm_post_empty(&pAviEncVidPara->vfm, video_frame);
				#else
					avi_encode_post_empty(video_frame_q, video_frame);
				#endif
					goto VIDEO_ENCODE_FRAME_MODE_END;
				}

                encode_size = h264_encode_frame(scaler_frame, video_frame, type, hdr, NullFrm );
                if(encode_size < 0) {
					DEBUG_MSG("encode_size = 0\r\n");
					goto VIDEO_ENCODE_FRAME_MODE_END;
				}

				if (!NullFrm)
					FrmCnt ++;

                if (FrmCnt >= pAviEncVidPara->h264_gop_len)
                    FrmCnt = 0;

				__disable_irq();
				pAviEncPara->Tv += pAviEncPara->delta_Tv;
				__enable_irq();
            }
#ifdef VFRAME_MANAGER
			if (encode_size && (0 > vfm_report_size(&pAviEncVidPara->vfm, video_frame, encode_size))) {
				encode_size = -1;
				goto VIDEO_ENCODE_FRAME_MODE_END;
			}
#endif
            t2 = xTaskGetTickCount();
            encode_time += (t2-t1);

				if(pAviEncPara->avi_encode_status & C_AVI_ENCODE_START &&
					pAviEncPara->video[rCnt].ready_frame == 0) {
					INT8U skip = 0;

					#if AVI_PACKER_LIB_EN == 0
					if (pAviEncVidPara->video_format == C_MJPG_FORMAT)
					{
						__disable_irq();
						if (pAviEncPara->tv <= pAviEncPara->Tv) //for frame rate control
							skip = 1;
						__enable_irq();
					}
					#endif

					if (skip) {
						DBG_PRINT("S");
						if (video_frame != 0x50000000)
#ifdef VFRAME_MANAGER
							vfm_post_empty(&pAviEncVidPara->vfm, video_frame);
#else
							avi_encode_post_empty(video_frame_q, video_frame);
#endif
					}
					else {
						pAviEncPara->video[rCnt].ready_frame = video_frame;
						pAviEncPara->video[rCnt].encode_size = encode_size;
						pAviEncPara->video[rCnt].key_flag = key_flag;
						#if VIDEO_TIMESTAMP
						pAviEncPara->video[rCnt].pts = frame_ts->pts;
						#endif
						//DBG_PRINT("==%d==%x==\r\n", frame_ts->pts, scaler_frame);
						avi_encode_post_empty(vid_enc_frame_q, (INT32U)&pAviEncPara->video[rCnt]);

						rCnt++;
						if(rCnt >= AVI_ENCODE_VIDEO_BUFFER_NO)
							rCnt = 0;

						#if AVI_PACKER_LIB_EN == 0
						pAviEncPara->encoded_cnt ++;
						if (pAviEncVidPara->video_format == C_MJPG_FORMAT)
						{
							__disable_irq();
							pAviEncPara->Tv += pAviEncPara->delta_Tv;
							__enable_irq();
						}
						//DBG_PRINT("t=%lld, T=%lld\r\n", pAviEncPara->tv, pAviEncPara->Tv);
						#endif
					}
				} else if(pAviEncPara->avi_encode_status & C_AVI_ENCODE_JPEG) {
					pAviEncPara->video[rCnt].ready_frame = video_frame;
					pAviEncPara->video[rCnt].encode_size = encode_size;
					pAviEncPara->video[rCnt].key_flag = AVIIF_KEYFRAME;
					avi_encode_post_empty(vid_enc_frame_q, (INT32U)&pAviEncPara->video[rCnt]);

					//clear status
					pAviEncPara->avi_encode_status &= ~C_AVI_ENCODE_JPEG;
										/* wifi stream */

				}else if((avi_encode_get_status()&C_AVI_ENCODE_STREAM)){

					//DBG_PRINT("Addr 0x%x, size 0x%x\r\n", video_frame, (encode_size + header_size));
					if(update_cur_jpeg)
					{
						update_cur_jpeg((INT8U*)video_frame, encode_size);
					}

				} else {
					//post back to queue
					DBG_PRINT("C");
					if (video_frame != 0x50000000)
#ifdef VFRAME_MANAGER
						vfm_post_empty(&pAviEncVidPara->vfm, video_frame);
#else
						avi_encode_post_empty(video_frame_q, video_frame);
#endif
				}

			VIDEO_ENCODE_FRAME_MODE_END:
				//post empty buffer to scale or sensor
				//if((AVI_ENCODE_DIGITAL_ZOOM_EN == 0) && (pAviEncVidPara->scaler_flag == 0)) {
				if (0) {
					avi_encode_post_empty(cmos_frame_q, scaler_frame);
				} else {
					//DBG_PRINT("SSSS[%d]\r\n", frame_ts->pts);
					if (scaler_frame != 0x50000000)
                        if(pAviEncVidPara->jpeg_encode_enable_flag)
                        {
                            if(pAviEncVidPara->encode_different_flag == 0)
                            {
                                avi_encode_post_empty(scaler_frame_q, scaler_frame);
                            }
                            #if VIDEO_TIMESTAMP
                            else
                                frame_ts->frame_addrs = 0;
                            #endif
                        }
                        else
                        {
                            #if PALM_DEMO_EN == 1
                            avi_encode_post_empty(fd_dma_frame_buffer_queue, scaler_frame);
							#elif WAFD_DEMO_EN == 1
							avi_encode_post_empty(pscaler_frame_buffer_queue, scaler_frame);
                            #else
                            avi_encode_post_empty(scaler_frame_q, scaler_frame);
                            #endif
                        }
                        #if VIDEO_TIMESTAMP
                        avi_encode_post_empty(frame_ts_q, (INT32U)frame_ts);
                        #endif
				}
				#ifdef VFRAME_MANAGER
				if (encode_size < 0 && video_frame != 0x50000000)
					vfm_post_empty(&pAviEncVidPara->vfm, video_frame);
				#endif

//			}
			break;

#elif VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
			if(msg_id & C_AVI_ENCODE_FIFO_ERR) {
				DEBUG_MSG("F1");
				goto VIDEO_ENCODE_FIFO_MODE_FAIL;
			}

			if(pAviEncVidPara->scaler_flag) {
				//R_IOC_O_DATA ^= 0x08;
				scaler_frame = msg_id & (~C_AVI_ENCODE_FRAME_END);
				y_frame = avi_encode_get_empty(scaler_frame_q);
				if(y_frame == 0) {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

				u_frame = y_frame + uv_length;
				v_frame = u_frame + (uv_length >> 1);

				scale.input_format = pAviEncVidPara->sensor_output_format;
				scale.input_width = pAviEncVidPara->encode_width;//pAviEncVidPara->sensor_capture_width;
				scale.input_height = SENSOR_FIFO_LINE;
				scale.input_visible_width = 0;
				scale.input_visible_height = 0;
				scale.input_x_offset = 0;
				scale.input_y_offset = 0;
				scale.input_y_addr = scaler_frame;
				scale.input_u_addr = 0;
				scale.input_v_addr = 0;

				scale.output_format = C_SCALER_CTRL_OUT_YUV422;
				scale.output_width = pAviEncVidPara->encode_width;
				scale.output_height = scaler_height + 2; //+2 is to fix block line
				scale.output_buf_width = pAviEncVidPara->encode_width;
				scale.output_buf_height = scaler_height + 2; //+2 is to fix block line
				scale.output_x_offset = 0;
				scale.output_y_addr = y_frame;
				scale.output_u_addr = u_frame;
				scale.output_v_addr = v_frame;

				scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
				scale.scale_mode = C_SCALER_FULL_SCREEN;
				scale.digizoom_m = 10;
				scale.digizoom_n = 10;

				memset((void *)&para, 0x00, sizeof(para));
				para.boundary_color = 0x008080;

    			drv_l2_scaler_trigger(SCALER_0, 1, &scale, &para);
				//R_IOC_O_DATA ^= 0x08;
			} else {
				y_frame = msg_id & (~C_AVI_ENCODE_FRAME_END);
				u_frame = v_frame = 0;
			}

			pAviEncPara->vid_pend_cnt++;
			if(msg_id & C_AVI_ENCODE_FRAME_END)	{
				if(pAviEncPara->vid_pend_cnt != pAviEncPara->vid_post_cnt) {
					DEBUG_MSG("F2:%d\r\n", pAviEncPara->vid_pend_cnt);
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

			//if(pAviEncPara->vid_pend_cnt == pAviEncPara->vid_post_cnt) {
				if(jpeg_start_flag == 0) {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

				EncMode = 3; //jpeg encode end
				pAviEncPara->vid_pend_cnt = 0;
			} else if(pAviEncPara->vid_pend_cnt == 1) {
				if(jpeg_start_flag == 1) {
					jpeg_start_flag = 0;
					jpeg_encode_stop();
				}
				EncMode = 1; //jpeg encode start
			} else if(pAviEncPara->vid_pend_cnt < pAviEncPara->vid_post_cnt) {
				if(jpeg_start_flag == 0) {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

				EncMode = 2; //jpeg encode once
			} else {
				// error happen
                DBG_PRINT("d");
				goto VIDEO_ENCODE_FIFO_MODE_FAIL;//while(1);
			}

			switch(EncMode)
			{
			case 1:
				//DEBUG_MSG("J");
				if(avi_encode_get_status()&C_AVI_ENCODE_PRE_START) {
					video_frame = avi_encode_get_empty(video_frame_q);
				} else {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

				if(video_frame == 0) {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

				jpeg.jpeg_status = 0;
				jpeg.wait_done = 1;
				jpeg.quality_value = pAviEncVidPara->quality_value;
				//if(pAviEncVidPara->scaler_flag) {
				//	jpeg.input_format = C_JPEG_FORMAT_YUV_SEPARATE;
				//} else {
					jpeg.input_format = C_JPEG_FORMAT_YUYV;
				//}
				#if JPEG_X1_5_SCALE == 1
				jpeg.width = (pAviEncVidPara->encode_width*2)/3;
				jpeg.height = (pAviEncVidPara->encode_height*2)/3;
				#else
				jpeg.width = pAviEncVidPara->encode_width;
				jpeg.height = pAviEncVidPara->encode_height;
				#endif
				//DBG_PRINT("JPG W %d H %d \r\n",jpeg.width,jpeg.height);
				jpeg.input_buffer_y = y_frame;
				jpeg.input_buffer_u = u_frame;
				jpeg.input_buffer_v = v_frame;
				jpeg.input_y_len = input_y_len;
				jpeg.input_uv_len = input_uv_len;
				jpeg.output_buffer = video_frame + header_size;
				status = jpeg_encode_fifo_start(&jpeg);
				if(status < 0) {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

				jpeg_start_flag = 1;
				break;

			case 2:
				//DEBUG_MSG("*");
				jpeg.jpeg_status = status;
				jpeg.wait_done = 1;
				jpeg.input_buffer_y = y_frame;
				jpeg.input_buffer_u = u_frame;
				jpeg.input_buffer_v = v_frame;
				jpeg.input_y_len = input_y_len;
				jpeg.input_uv_len = input_uv_len;
				status = jpeg_encode_fifo_once(&jpeg);
				if(status < 0) {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}
				break;

			case 3:
				//DEBUG_MSG("G\r\n");
				jpeg.jpeg_status = status;
				jpeg.wait_done = 1;
				jpeg.input_buffer_y = y_frame;
				jpeg.input_buffer_u = u_frame;
				jpeg.input_buffer_v = v_frame;
				jpeg.input_y_len = input_y_len;
				jpeg.input_uv_len = input_uv_len;
				encode_size = jpeg_encode_fifo_stop(&jpeg);
				if(encode_size < 0) {
					goto VIDEO_ENCODE_FIFO_MODE_FAIL;
				}

				jpeg_start_flag = 0;
				if(pAviEncPara->avi_encode_status & C_AVI_ENCODE_START &&
					pAviEncPara->video[rCnt].ready_frame == 0) {
					pAviEncPara->video[rCnt].ready_frame = video_frame;
					pAviEncPara->video[rCnt].encode_size = encode_size + header_size;
					if(pAviEncPara->video[rCnt].encode_size > (pAviEncVidPara->encode_width*pAviEncVidPara->encode_height/4))
					{
                        DBG_PRINT("szie too large %d\r\n",pAviEncPara->video[rCnt].encode_size);
					}
					pAviEncPara->video[rCnt].key_flag = 0;
					avi_encode_post_empty(vid_enc_frame_q, (INT32U)&pAviEncPara->video[rCnt]);
					rCnt++;
					if(rCnt >= AVI_ENCODE_VIDEO_BUFFER_NO) {
						rCnt = 0;
					}
				} else if(pAviEncPara->avi_encode_status & C_AVI_ENCODE_JPEG) {
					pAviEncPara->video[rCnt].ready_frame = video_frame;
					pAviEncPara->video[rCnt].encode_size = encode_size + header_size;
					pAviEncPara->video[rCnt].key_flag = AVIIF_KEYFRAME;
					avi_encode_post_empty(vid_enc_frame_q, (INT32U)&pAviEncPara->video[rCnt]);
					//clear status
					pAviEncPara->avi_encode_status &= ~C_AVI_ENCODE_JPEG;
				} else {
					//post back to queue
					avi_encode_post_empty(video_frame_q, video_frame);
				}
				//DEBUG_MSG("*");
				break;
			}

			//if(pAviEncVidPara->scaler_flag) {
			if (1) {
				avi_encode_post_empty(jpeg_fifo_q, y_frame);
			}
			break;

VIDEO_ENCODE_FIFO_MODE_FAIL:
			pAviEncPara->vid_pend_cnt = 0;
			if(jpeg_start_flag) {
				jpeg_start_flag = 0;
				jpeg_encode_stop();
			}
			//post back to queue
			if (video_frame)
			{
				 avi_encode_post_empty(video_frame_q, video_frame);
			}
			avi_encode_post_empty(jpeg_fifo_q, y_frame);

#endif
		break;
//WIFI_STREAMING_JPEG_EN:


		}
	}
}
#endif//#if (defined APP_VIDEO_ENCODER_EN) && (APP_VIDEO_ENCDOER_EN == 1)
