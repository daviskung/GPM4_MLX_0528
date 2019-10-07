#include "application.h"
#include "drv_l1_ppu.h"
#include "drv_l2_display.h"
#include "gplib_ppu.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "sprite_dataSP_HDR.h"
#include "TEXT_Platform_verification_1_HDR.h"
#include "TEXT_Platform_verification_2_HDR.h"
#include "TEXT_Platform_verification_3_HDR.h"

#include "drv_l1_scaler.h"
#include "drv_l2_scaler.h"
#include "32X32RGB565NEW.h" 	//  davis 2019.04.23


#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F

#define COLOR_FRAME_OUT		1


#if  GPLIB_PPU_EN
static FP32	Scale_Factor[240];
static INT32U	Tx3CosSineBuf[240];
#define C_PPU_DRV_FRAME_NUM		            3
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static INT16U h_size,v_size;
#define EXSP_ENABLE                         1
#define CDM_MODE_ENABLE                     1
#define Photo_Disable                       1
#endif

void GPM4_PPU_Demo(void)
{
#if  GPLIB_PPU_EN
    INT16S x_pos,y_pos;
    INT32U frame_size,display_buf;
    INT32U buffer_ptr,frame_count;
    INT32U i,Angle,flag,flag1,flag2,sp_rotate,sp_zoom,sp_mosaic;
    INT32S blend_level,sp_blend_level;
    INT32U sp_num,sp_num_addr;
    SpN_ptr sp_ptr;
    V3D_POS_STRUCT Sprite_pos;
    CDM_STRUCT Sprite_cdm;
    nTX_image_info image_info;
	ScalerFormat_t scale;
	ScalerPara_t para;

	INT32S  nRet;

    // Initialize display device
    drv_l2_display_init();
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



    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *) &ppu_register_structure;

    //Initiate PPU hardware engine and PPU register set structure
	gplib_ppu_init(ppu_register_set);

	//Now configure PPU software structure
	gplib_ppu_enable_set(ppu_register_set, 1);					            // Enable PPU

    //TV frame mode
  	gplib_ppu_non_interlace_set(ppu_register_set, 0);			            // Set non-interlace mode
  	gplib_ppu_frame_buffer_mode_set(ppu_register_set, 1, 0);		        // Enable TV/TFT frame buffer mode

    //PPU setting
	gplib_ppu_fb_format_set(ppu_register_set, 1, 1);			            // Set PPU output frame buffer format to YUYV
    gplib_ppu_vga_mode_set(ppu_register_set, 0);							// Disable VGA mode
    gplib_ppu_resolution_set(ppu_register_set, C_TFT_RESOLUTION_320X240);	// Set display resolution to 640x480
    gplib_ppu_free_size_set(ppu_register_set, 0, h_size, v_size);
    gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
    gplib_ppu_long_burst_set(ppu_register_set, 1);

    //Frame buffer malloc
	frame_size = (h_size * v_size * 2);
    buffer_ptr = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    buffer_ptr = (buffer_ptr + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64;
	for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
		//buffer_ptr = (INT32U) gp_malloc_align(frame_size, 64);
		//gplib_ppu_frame_buffer_add(ppu_register_set, buffer_ptr);
        gplib_ppu_frame_buffer_add(ppu_register_set, (buffer_ptr + (i*frame_size)));
		DBG_PRINT("gplib_ppu_frame_buffer_add[%d] -> 0x%x\r\n",i,(buffer_ptr + (i*frame_size)));
	}

	

	// 4K character number array is enough for char mode(Character 32x32) and bitmap mode(TEXT 1024x1024)
	buffer_ptr = (INT32U) gp_malloc_align(1024*4,4);
	if (!buffer_ptr) {
		DBG_PRINT("Photo display task failed to allocate character number array memory\r\n");
	}

   
#if 1
	// 4K character number array is enough for char mode(Character 32x32) and bitmap mode(TEXT 1024x1024)
	buffer_ptr = (INT32U) gp_malloc_align(1024*4,4);
	if (!buffer_ptr) {
		DBG_PRINT("Photo display task failed to allocate character number array memory\r\n");
	}

   



	// Start PPU and wait until PPU operation is done
	//gplib_ppu_go_and_wait_done(ppu_register_set);

    display_buf = ppu_frame_buffer_display_get();
    if(display_buf > 0)
        drv_l2_display_update(DISPLAY_DEVICE,display_buf);

	frame_count=0;


	

	while(1)
	{
	     //Start PPU and wait until PPU operation is done
	     if(display_buf > 0)
            gplib_ppu_frame_buffer_add(ppu_register_set, display_buf);

	     gplib_ppu_go_and_wait_done(ppu_register_set);
	     display_buf = ppu_frame_buffer_display_get();
	     if(display_buf > 0){
            drv_l2_display_update(DISPLAY_DEVICE,display_buf);
			DBG_PRINT("frame_count = 0x%x\r\n",frame_count);  //davisppu-1
			DBG_PRINT("display_buf = 0x%x\r\n",display_buf);	 //davisppu-1
	     	}

		// DBG_PRINT("display_buf = 0x%x\r\n",display_buf);	 //davisppu-1


		//
	// MLX_TH32x24 sensor scaler -[ first time]
	//

		gp_memset((INT8S *)&scale, 0x00, sizeof(scale));
		drv_l2_scaler_init();
		//scale.input_format = pAviEncVidPara->sensor_output_format;
	#if COLOR_FRAME_OUT
		scale.input_format = C_SCALER_CTRL_IN_RGB565;
	#else
		scale.input_format = C_SCALER_CTRL_IN_Y_ONLY; 	// gray value
	#endif
		scale.input_width = 32;		//h_size/10;
		scale.input_height = 32;	//v_size/10;
		scale.input_visible_width = 32;		//h_size/10;
		scale.input_visible_height = 32;	// v_size/10;
		scale.input_x_offset = 0;
		scale.input_y_offset = 0;

		//scale.input_y_addr = sensor_frame;
		//scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr ;

	#if COLOR_FRAME_OUT
		if(frame_count%2 == 0)
		scale.input_y_addr = &(sensor32X32_RGB565_2);
		else
		scale.input_y_addr = &(sensor32X32_RGB565);
	#else
		scale.input_y_addr =pMLX_TH32x24_Para->MLX_TH32x24_GrayScaleUpFrame_addr ;
	#endif

		scale.input_u_addr = 0;
		scale.input_v_addr = 0;
		

		scale.output_format = C_SCALER_CTRL_OUT_VYUY;
		scale.output_width = h_size;
		scale.output_height = v_size;
		scale.output_buf_width = h_size;
		scale.output_buf_height = v_size;
		scale.output_x_offset = 0;

		//scale.output_y_addr = scaler_frame;
		scale.output_y_addr = display_buf;
		scale.output_u_addr = 0;
		scale.output_v_addr = 0;


		scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
		//scale.scale_mode = C_SCALER_FULL_SCREEN;
		scale.scale_mode = C_SCALER_BY_RATIO;
		scale.digizoom_m = 10;
		scale.digizoom_n = 10;
		//#endif
		gp_memset((void *)&para, 0x00, sizeof(para));

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

	     frame_count++;
	}
#endif
#endif
}

