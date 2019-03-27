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
    PPU_FRAME_BUFFER_BASE = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(PPU_FRAME_BUFFER_BASE == 0)
    {
        DBG_PRINT("PPU_FRAME_BUFFER_BASE fail\r\n");
        while(1);
    }
    PPU_FRAME_BUFFER_BASE = (INT32U)((PPU_FRAME_BUFFER_BASE + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
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

static void TH32x32_task_entry(void const *parm)
{
	INT32U msg_id, ack_msg;
	//INT32U sensor_frame, scaler_frame, display_frame;
	//ScalerFormat_t scale;
	//ScalerPara_t para;
	osEvent result;
	osThreadId id;

	drv_l1_i2c_bus_handle_t th32x32_handle;

	sensor32x32_cmdCode sensorCmd;
	INT8U value;
	INT16U value2byte;

	float	PTATGrad,PTATOff,PTATLong;
	float 	common[2];
	INT8U 	EEcopy[8],*pEEcopy;
	INT8U 	EEaddr[2],*pEEaddr;
	INT16U	EEaddress16,EEcopy16BIT,*pEEcopy16BIT;
	INT16U	TA,gradScale,TableNumberSensor,epsilon;
	unsigned short GlobalGain;	// INT16U
	char 	GlobalOffset;			// INT8U
	INT32U	PTATsum;
	float	PixCMin,PixCMax;
	INT16U	SetMBITCalib,SetBIASCalib,SetCLKCalib,SetBPACalib,SetPUCalib,VddCalib;

	//pEEcopy16BIT = EEcopy16BIT;

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


			pEEcopy = EEcopy;
			pEEaddr = EEaddr;


			// =====================================================================

			th32x32_handle.devNumber = I2C_1;
		    th32x32_handle.slaveAddr = TH32x32_EEPROM_ID << 1 ;
		    th32x32_handle.clkRate = 800;
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



			// =====================================================================
			th32x32_handle.devNumber = I2C_1;
		    th32x32_handle.clkRate = 800;	// 必須 再設定 


			//
			//	sensor registers
			//

			sensorCmd.DeviceAdd =(TH32x32_REG_ID << 1);
		    th32x32_handle.slaveAddr = sensorCmd.DeviceAdd;

			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.RegCmd =CONFIG_REG_WAKEUP;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

			osDelay(10);
			sensorCmd.RegAdd =TH32x32_MBIT_TRIM;
			sensorCmd.RegCmd =MBIT_TRIM_12BIT_DEFAULT;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

			osDelay(10);
			sensorCmd.RegAdd =TH32x32_BIAS_TRIM_TOP;
			sensorCmd.RegCmd =BIAS_TRIM_TOP_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

            osDelay(10);
			sensorCmd.RegAdd =TH32x32_BIAS_TRIM_BOT;
			sensorCmd.RegCmd =BIAS_TRIM_BOM_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

            osDelay(10);
           	sensorCmd.RegAdd =TH32x32_CLK_TRIM;
			sensorCmd.RegCmd =CLK_TRIM_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

			osDelay(10);
           	sensorCmd.RegAdd =TH32x32_BPA_TRIM_TOP;
			sensorCmd.RegCmd =BPA_TRIM_TOP_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

			osDelay(10);
           	sensorCmd.RegAdd =TH32x32_BPA_TRIM_BOT;
			sensorCmd.RegCmd =BPA_TRIM_BOM_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

			osDelay(10);
           	sensorCmd.RegAdd =TH32x32_PU_SDA_TRIM;
			sensorCmd.RegCmd =PU_SDA_TRIM_SAMPLE_VAL;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);

			osDelay(10);
           	sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.RegCmd =CONFIG_REG_START_B0_WAKEUP;
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);


			osDelay(30);
			EEaddr[0]=TH32x32_STATUS_REG;
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
			
			DBG_PRINT("EEcopy B0 = [0x%x] [0x%x]\r\n", EEcopy[0],EEcopy[1]);
			EEcopy[0]=0xf1;	// why can not change? need use this change !!
			EEcopy[1]=0xf2;
			
			DBG_PRINT("EEcopy check = [0x%x] [0x%x]\r\n", EEcopy[0],EEcopy[1]);
			EEcopy[2]=0xf3;
			EEcopy[3]=0;
			EEcopy[4]=0;
			EEcopy[5]=0;
			EEcopy[6]=0;
			EEcopy[7]=0;
			
			EEaddr[0]=TH32x32_READ_DATA_TOP;
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,8,TH32x32_I2C_RESTART_MODE);
			
			
			

			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.RegCmd = CONFIG_REG_START_B1_WAKEUP; // block1
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);


			osDelay(10);
			EEaddr[0]=TH32x32_STATUS_REG;
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
			
			DBG_PRINT("EEcopy B1 = [0x%x] [0x%x]\r\n", EEcopy[0],EEcopy[1]);

			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.RegCmd = CONFIG_REG_START_B2_WAKEUP; // block2
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);


			osDelay(10);
			EEaddr[0]=TH32x32_STATUS_REG;
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
			
			DBG_PRINT("EEcopy B2 = [0x%x] [0x%x]\r\n", EEcopy[0],EEcopy[1]);

			sensorCmd.RegAdd =TH32x32_CONFIG_REG;
			sensorCmd.RegCmd = CONFIG_REG_START_B3_WAKEUP; // block3
			drv_l1_reg_1byte_data_1byte_write(&th32x32_handle,sensorCmd.RegAdd,sensorCmd.RegCmd);


			osDelay(10);
			EEaddr[0]=TH32x32_STATUS_REG;
			drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,1,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
			
			DBG_PRINT("EEcopy B3 = [0x%x] [0x%x]\r\n", EEcopy[0],EEcopy[1]);
			
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
                //DBG_PRINT("1");
            }
        }
        else if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE){
            display_frame = drv_l1_pscaler_output_B_buffer_get(PSCALER_A);
            if(display_frame != 0x50000000)
            {
                pAviEncVidPara->CSIOrgBufB = display_frame;
                display_frame = 0x50000000;
                drv_l1_pscaler_output_B_buffer_set(PSCALER_A, display_frame);
                //DBG_PRINT("2");
            }
        }
        if(display_frame == 0x50000000)
            display_frame = 0;
    }
    else
    {
        if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_A_DONE) {
            display_frame = drv_l1_pscaler_output_A_buffer_get(PSCALER_A);
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
                free_frame = avi_encode_get_empty(display_frame_q);
                if(free_frame)
                    drv_l1_pscaler_output_A_buffer_set(PSCALER_A, free_frame);
                else
                {
                    DBG_PRINT("d");
                    drv_l1_pscaler_output_A_buffer_set(PSCALER_A,0x50000000);
                }
            }
        }
        else if (PScaler_Event & PIPELINE_SCALER_STATUS_BUF_B_DONE){
            display_frame = drv_l1_pscaler_output_B_buffer_get(PSCALER_A);
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
        display_frame |= 0x80000000;
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
				else
                    avi_encode_post_empty(display_frame_q, scaler_frame);
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
		    	if(videnc_display) {
		    		videnc_display(pAviEncVidPara->display_buffer_width,
		    					   pAviEncVidPara->display_buffer_height,
		    					   display_frame);
		    	}

		    	if(videnc_buferr_post)
		    	{
                    scaler_frame = videnc_buferr_post(display_frame);
                    if(scaler_frame)
                        display_frame = scaler_frame;
		    	}

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
