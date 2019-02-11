#include "application.h"
#include "drv_l1_dma.h"
#include "drv_l1_ppu.h"
#include "drv_l2_display.h"
#include "drv_l2_ad_key_scan.h"
#include "gplib_ppu.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "gplib_ppu_dma.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "SPRITE_driver_HDR.h"

#define VTECH_BUF_ALIGN                     0xFF
#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
#define C_PPU_DRV_FRAME_NUM		            3

typedef enum
{
    ppu_deiver_sprite_mask_mode = 0,

    // ppu vtech
    ppu_vtech_sprite_3d_mode,
    //end
    ppu_deiver_end
} ppu_driver_Mode;

typedef struct {
	INT32U ppu_frame_buffer_workmem;
	INT32U ppu_text1_narray_workmem;
	INT32U ppu_text2_narray_workmem;
	INT32U ppu_text3_narray_workmem;
	INT32U ppu_text4_narray_workmem;
    INT32U ppu_disp_init_flag;
} prcess_mem_t;

static osThreadId adkey_id;
static FP32	Scale_Factor[240];
static INT32U Tx3CosSineBuf[240];
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static prcess_mem_t prcess_mem_structure;
static prcess_mem_t *prcess_mem_set;
static INT16U h_size,v_size;
static INT32S ppu_driver_demo_mode, ppu_driver_exit_mode;
static INT32S (*ppu_driver_callback)(PPU_REGISTER_SETS *ppu_register_set);

// driver demo
INT32S ppu_srite_mask_mode(PPU_REGISTER_SETS *ppu_register_set);
INT32S ppu_vtech_srite_3d_mode(PPU_REGISTER_SETS *ppu_register_set);

static INT32S ppu_disp_init(INT16 *hs, INT16 *vs, DISP_FMT color_mode)
{
    INT32S ret;

    if((hs == 0) ||(vs == 0))
        return -1;

    ret = drv_l2_display_init();
    if(ret < 0)
        return ret;

    ret = drv_l2_display_start(DISPLAY_DEVICE, color_mode);
    if(ret < 0)
        return ret;

    drv_l2_display_get_size(DISPLAY_DEVICE,(INT16U *)hs,(INT16U *)vs);

    return 0;
}

static INT32U ppu_text_narray_workmem_get(INT32U text)
{
    INT32U ptr;

    if((text > C_PPU_TEXT4) || (prcess_mem_set->ppu_text1_narray_workmem == 0))
        return 0;

    switch(text)
    {
        case C_PPU_TEXT1:
            ptr = (INT32U)prcess_mem_set->ppu_text1_narray_workmem;
            break;

        case C_PPU_TEXT2:
            ptr = (INT32U)prcess_mem_set->ppu_text2_narray_workmem;
            break;

        case C_PPU_TEXT3:
            ptr = (INT32U)prcess_mem_set->ppu_text3_narray_workmem;
            break;

        case C_PPU_TEXT4:
            ptr = (INT32U)prcess_mem_set->ppu_text4_narray_workmem;
            break;
    }

    return ptr;
}

static INT32S ppu_driver_init(PPU_REGISTER_SETS *ppu_register_set, INT32U ppu_draw_mode, DISP_FMT color_mode)
{
    INT32U i, frame_size, buffer_ptr;
    INT32S ret;

    // device init
    if(ppu_draw_mode == 0)
    {
        if(prcess_mem_set->ppu_disp_init_flag == 0)
        {
            ret = ppu_disp_init(&h_size, &v_size, color_mode);
            if(ret < 0)
            {
                prcess_mem_set->ppu_disp_init_flag = 0;
                DBG_PRINT("ppu_disp_init error\r\n");

                return -1;
            }
            else
                prcess_mem_set->ppu_disp_init_flag = 1;
        }
    }

    //Initiate PPU hardware engine and PPU register set structure
	gplib_ppu_init(ppu_register_set);

	// Enable PPU
	gplib_ppu_enable_set(ppu_register_set, 1);

    // Set non-interlace mode
  	gplib_ppu_non_interlace_set(ppu_register_set, 0);

  	// Enable PPU frame buffer mode
  	gplib_ppu_frame_buffer_mode_set(ppu_register_set, 1, 0);

    //PPU setting
    // Disable VGA mode
    gplib_ppu_vga_mode_set(ppu_register_set, 0);

    gplib_ppu_resolution_set(ppu_register_set, C_TFT_RESOLUTION_320X240);
    if(ppu_draw_mode)
    {


    }
    else
        gplib_ppu_free_size_set(ppu_register_set, 0, h_size, v_size);

    // sprite coordinate range
    set_sprite_free_size(h_size, v_size);

    // bottom to top
    gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);
    gplib_ppu_long_burst_set(ppu_register_set, 1);

    switch(color_mode)
    {
        case DISP_FMT_YUYV:
            // Set PPU output frame buffer format to YUYV
            gplib_ppu_yuv_type_set(ppu_register_set, 3);
            gplib_ppu_fb_format_set(ppu_register_set, 1, 1);
            break;

        default:
            // Set PPU output frame buffer format to RGB565
            gplib_ppu_fb_format_set(ppu_register_set, 0, 0);
            break;
    }

    //Frame buffer malloc
    if(ppu_draw_mode)
    {


    }
    else
        frame_size = (h_size * v_size * 2);

    if(prcess_mem_set->ppu_frame_buffer_workmem == 0)
    {
        buffer_ptr = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
        if(buffer_ptr == 0)
        {
            DBG_PRINT("ppu_frame_buffer malloc error\r\n");
            return -1;
        }
        buffer_ptr = (buffer_ptr + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64;
        prcess_mem_set->ppu_frame_buffer_workmem = buffer_ptr;
    }
    else
        buffer_ptr = prcess_mem_set->ppu_frame_buffer_workmem;

	for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
        gplib_ppu_frame_buffer_add(ppu_register_set, (buffer_ptr + (i*frame_size)));
        DBG_PRINT("ppu_frame_buffer[%d] = 0x%x\r\n",i, (buffer_ptr + (i*frame_size)));
	}

	if(prcess_mem_set->ppu_text1_narray_workmem == 0)
	{
        frame_size = 8192;
        buffer_ptr = (INT32U) gp_malloc_align(((frame_size*4)+128), 64);
        if(buffer_ptr == 0)
        {
            DBG_PRINT("ppu_narray_buffer malloc error\r\n");
            return -1;
        }
        buffer_ptr = (buffer_ptr + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64;
        prcess_mem_set->ppu_text1_narray_workmem = buffer_ptr;
        prcess_mem_set->ppu_text2_narray_workmem = (prcess_mem_set->ppu_text1_narray_workmem + frame_size);
        prcess_mem_set->ppu_text3_narray_workmem = (prcess_mem_set->ppu_text2_narray_workmem + frame_size);
        prcess_mem_set->ppu_text4_narray_workmem = (prcess_mem_set->ppu_text3_narray_workmem + frame_size);
	}

    return 0;
}

static INT32S ppu_disp_buffer_set(PPU_REGISTER_SETS *ppu_register_set, INT32U buf)
{
    INT32S ret;

    if(!ppu_register_set || !buf)
        return -1;

    ret = drv_l2_display_update(DISPLAY_DEVICE, buf);
    if(ret < 0)
        return ret;
    gplib_ppu_frame_buffer_add(ppu_register_set, buf);

    return 0;
}

static INT32S ppu_driver_selection(INT32U enable, INT32U mode)
{
    INT32S ret = 0;

    switch(mode)
    {
        case ppu_deiver_sprite_mask_mode:
            if(enable)
                ppu_driver_callback = ppu_srite_mask_mode;
            else
                DBG_PRINT("ppu_deiver_sprite_mask_mode\r\n");
            break;

        case ppu_vtech_sprite_3d_mode:
            if(enable)
                ppu_driver_callback = ppu_vtech_srite_3d_mode;
            else
                DBG_PRINT("ppu_vtech_sprite_3d_mode\r\n");
            break;

        default :
            if(enable)
                ppu_driver_callback = 0;
            else
                DBG_PRINT("no function selection\r\n");
            ret = 1;
            break;
    }

    if(ret)
        return -1;
    else
        return 0;
}

static void adkey_task_entry(void const *parm)
{
    DBG_PRINT("[adkey_task_entry]\r\n");

    adc_key_scan_init();
    while(1)
    {
        adc_key_scan();
        if(ADKEY_IO1)
		{
            DBG_PRINT("UP\r\n");
            ppu_driver_demo_mode++;
            if(ppu_driver_demo_mode >= ppu_deiver_end)
                ppu_driver_demo_mode--;
        }

        if(ADKEY_IO2)
		{
            DBG_PRINT("DOWN\r\n");
            ppu_driver_demo_mode--;
            if(ppu_driver_demo_mode < 0)
                ppu_driver_demo_mode = 1;
        }

        if(ADKEY_IO1 || ADKEY_IO2)
        {
            ppu_driver_selection(0, ppu_driver_demo_mode);

            if(ADKEY_IO1)
                ADKEY_IO1 = 0;
            if(ADKEY_IO2)
                ADKEY_IO2 = 0;
        }

        if(ADKEY_IO3)
		{
            DBG_PRINT("Demo Start\r\n");
            ppu_driver_exit_mode = 0;
        }

        if(ADKEY_IO4)
		{
            DBG_PRINT("Demo Exit\r\n");
            ppu_driver_exit_mode = 1;
        }
    }
}

void GPM4_PPU_Driver_Demo(void)
{
#if GPLIB_PPU_EN
    osThreadDef_t adkey_task = {"adkey_task", adkey_task_entry, osPriorityNormal, 1, 4096};

	DBG_PRINT("\r\n***************************************************\r\n");
	DBG_PRINT("      This is GPM4_PPU_Driver DEMO                **\r\n");
	DBG_PRINT("KEY_1 PPU Driver Mode Selection up                **\r\n");
	DBG_PRINT("KEY_2 PPU Driver Mode Selection down              **\r\n");
	DBG_PRINT("KEY_3 PPU Driver Mode Enable                      **\r\n");
	DBG_PRINT("KEY_4 PPU Driver Mode Exit                        **\r\n");
	DBG_PRINT("***************************************************\r\n");

    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;
    prcess_mem_set = (prcess_mem_t *)&prcess_mem_structure;
    gp_memset((INT8S *)prcess_mem_set, 0, sizeof(prcess_mem_t));

    adkey_id = osThreadCreate(&adkey_task, (void *)NULL);
    if(adkey_id == 0) {
        DBG_PRINT("adkey_task error\r\n");
        while(1);
    }
    else
        osDelay(30);

_DEMO_START:
    // driver mode selection
    //ppu_vtech_sprite_3d_mode;//ppu_deiver_sprite_mask_mode;
    ppu_driver_demo_mode = ppu_deiver_sprite_mask_mode;
    //ppu_driver_demo_mode = -1;
    ppu_driver_exit_mode = 0;

    if(ppu_driver_demo_mode < 0)
    {
        while(1)
        {
            if(ppu_driver_exit_mode)
            {
                if(ppu_driver_demo_mode < 0)
                {
                    DBG_PRINT("PPU Driver Mode Selection error\r\n");
                    ppu_driver_exit_mode = 0;
                }
                else
                    break;
            }
            else
                osDelay(1);
        }
    }

    ppu_driver_selection(1, ppu_driver_demo_mode);

    if(ppu_driver_callback)
    {
        ppu_driver_callback(ppu_register_set);
        ppu_driver_demo_mode = -1;
        DBG_PRINT("GPM4_PPU_Driver_Demo End\r\n");
    }
    goto _DEMO_START;
#else
    DBG_PRINT("GPM4_PPU_Driver_Demo End\r\n");
#endif
}

INT32S ppu_srite_mask_mode(PPU_REGISTER_SETS *ppu_register_set)
{
    #define EXSP_MASK_EN            1
    INT32U buffer_ptr,sp_num_addr,flag_mode;
    SpN_ptr sp_ptr;
    V3D_POS_STRUCT Sprite_pos;

    ppu_driver_init(ppu_register_set, 0, DISP_FMT_YUYV);

	// Now configure Sprite
	gplib_ppu_sprite_init(ppu_register_set);
	gplib_ppu_sprite_enable_set(ppu_register_set, 1);			                            // Enable Sprite
	gplib_ppu_sprite_coordinate_set(ppu_register_set, 0);                                   // set sprite coordinate
	gplib_ppu_sprite_direct_mode_set(ppu_register_set, 1);		                            // Set sprite address mode
	gplib_ppu_sprite_number_set(ppu_register_set, 256);                                     // Set sprite number
	gplib_ppu_sprite_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteRAM);            // set sprite ram buffer
    gplib_ppu_sprite_extend_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteExRAM);   // value: 32-bit pointer to sprite extend attribute ram
    gplib_ppu_sprite_segment_set(ppu_register_set, (INT32U)0);                              // sprite cell data
    gplib_ppu_sprite_25d_mode_set(ppu_register_set, 1);
    gplib_ppu_sprite_window_enable_set(ppu_register_set, 1);
    #if EXSP_MASK_EN == 0
        gplib_ppu_sprite_large_size_set(ppu_register_set, 1);
    #endif

    gplib_ppu_mask_window_range_set(ppu_register_set, 1, 1, 0, 320, 0, 240);
    gplib_ppu_mask_window_range_set(ppu_register_set, 0, 0, 0, 160, 0, 120);

    // image and position init
    #if EXSP_MASK_EN == 1
        set_sprite_init(0,(INT32U)&Sprite01B_SP);
        set_sprite_display_init(0,0,0,(INT32U)_Sprite01B_IMG0000_CellIdx);
    #else
        set_sprite_init(0,(INT32U)&Sprite01A_SP);
        set_sprite_display_init(0,0,0,(INT32U)_Sprite01A_IMG0000_CellIdx);
        set_sprite_init(1,(INT32U)&Sprite01A_SP);
        set_sprite_display_init(1,0,0,(INT32U)_Sprite01A_IMG0000_CellIdx);
    #endif

    gplib_ppu_exsprite_enable_set(ppu_register_set, 1);
    gplib_ppu_exsprite_number_set(ppu_register_set, 1);
    #if EXSP_MASK_EN == 1
        gplib_ppu_exsprite_large_size_set(ppu_register_set, 1);
    #endif

	buffer_ptr = extend_sprite_malloc_memory(16);
	gplib_ppu_exsprite_start_address_set(ppu_register_set, (INT32U)buffer_ptr);

	// extend sprite set
	#if EXSP_MASK_EN == 1
        set_exsprite_init(0,(INT32U)&Sprite01A_SP);
        set_exsprite_display_init(0,0,0,(INT32U)_Sprite01A_IMG0000_CellIdx);
	#else
        set_exsprite_init(0,(INT32U)&Sprite01B_SP);
        set_exsprite_display_init(0,0,0,(INT32U)_Sprite01B_IMG0000_CellIdx);
    #endif

    flag_mode = 0;
    while(1)
    {
        paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,2);
        paint_ppu_exspriteram(Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,2);
        Get_sprite_image_info(0,(SpN_ptr *)&sp_ptr);
        sp_num_addr = sp_ptr.nSPNum_ptr;
        gplib_ppu_sprite_attribute_character_number_set((SpN_RAM *)sp_num_addr, (INT32U)0);
        Get_sprite_image_info(1,(SpN_ptr *)&sp_ptr);
        sp_num_addr = sp_ptr.nSPNum_ptr;
        #if EXSP_MASK_EN == 1
            Sprite_pos.V3D_POS1.x0 = 0;
            Sprite_pos.V3D_POS1.y0 = 0;
            Sprite_pos.V3D_POS1.x1 = (260/2);
            Sprite_pos.V3D_POS1.y1 = 0;
            Sprite_pos.V3D_POS1.x2 = (320/2);
            Sprite_pos.V3D_POS1.y2 = (240/2);
            Sprite_pos.V3D_POS1.x3 = (120/2);
            Sprite_pos.V3D_POS1.y3 = (150/2);
            Sprite_pos.group_id = 0;
        #else
            Sprite_pos.V3D_POS1.x0 = 0;
            Sprite_pos.V3D_POS1.y0 = 0;
            Sprite_pos.V3D_POS1.x1 = 260;
            Sprite_pos.V3D_POS1.y1 = 0;
            Sprite_pos.V3D_POS1.x2 = 320;
            Sprite_pos.V3D_POS1.y2 = 240;
            Sprite_pos.V3D_POS1.x3 = 120;
            Sprite_pos.V3D_POS1.y3 = 150;
            Sprite_pos.group_id = 0;
        #endif
        SpriteV3D_set(1,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,&sp_ptr,&Sprite_pos);
        #if EXSP_MASK_EN == 1
            buffer_ptr = (INT32U)_SPRITE_64_yuyv3_CellData;
        #else
            buffer_ptr = (INT32U)_SPRITE_256_yuyv3_CellData;
        #endif
        gplib_ppu_sprite_attribute_character_number_set((SpN_RAM *)sp_num_addr, (INT32U)(buffer_ptr/2));
        gplib_ppu_sprite_attribute_depth_set((SpN_RAM *)sp_num_addr, 3);
        gplib_ppu_sprite_attribute_window_set((SpN_RAM *)sp_num_addr, 1);

        //get exsprite character number of image and sprite start ptr of sprite ram
        Get_exsprite_image_info(0,(SpN_ptr *)&sp_ptr);
        sp_num_addr = sp_ptr.nSPNum_ptr;
        #if EXSP_MASK_EN == 1
            Sprite_pos.V3D_POS1.x0 = 0;
            Sprite_pos.V3D_POS1.y0 = 0;
            Sprite_pos.V3D_POS1.x1 = 260;
            Sprite_pos.V3D_POS1.y1 = 0;
            Sprite_pos.V3D_POS1.x2 = 320;
            Sprite_pos.V3D_POS1.y2 = 240;
            Sprite_pos.V3D_POS1.x3 = 120;
            Sprite_pos.V3D_POS1.y3 = 150;
            Sprite_pos.group_id = 0;
        #else
            Sprite_pos.V3D_POS1.x0 = 0;
            Sprite_pos.V3D_POS1.y0 = 0;
            Sprite_pos.V3D_POS1.x1 = (260/2);
            Sprite_pos.V3D_POS1.y1 = 0;
            Sprite_pos.V3D_POS1.x2 = (320/2);
            Sprite_pos.V3D_POS1.y2 = (240/2);
            Sprite_pos.V3D_POS1.x3 = (120/2);
            Sprite_pos.V3D_POS1.y3 = (150/2);
            Sprite_pos.group_id = 0;
        #endif
        EXSpriteV3D_set(0,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,&sp_ptr,&Sprite_pos);
        #if EXSP_MASK_EN == 1
            buffer_ptr = (INT32U)_SPRITE_256_yuyv3_CellData;
        #else
            buffer_ptr = (INT32U)_SPRITE_64_yuyv3_CellData;
        #endif
        gplib_ppu_sprite_attribute_character_number_set((SpN_RAM *)sp_num_addr, (INT32U)(buffer_ptr/2));
        #if EXSP_MASK_EN == 1
            if(flag_mode)
            {
                gplib_ppu_sprite_attribute_window_set((SpN_RAM *)sp_num_addr, 0);
                flag_mode = 0;
            }
            else
            {
                gplib_ppu_sprite_attribute_window_set((SpN_RAM *)sp_num_addr, 1);
                flag_mode = 1;
            }
            osDelay(1000);
        #endif
        // Start PPU and wait until PPU operation is done
        gplib_ppu_go_and_wait_done(ppu_register_set);
        ppu_disp_buffer_set(ppu_register_set, ppu_frame_buffer_display_get());

        if(ppu_driver_exit_mode)
            break;
    }

    return 0;
}

INT32S ppu_vtech_srite_3d_mode(PPU_REGISTER_SETS *ppu_register_set)
{
    INT16U *src_ptr,*tar_ptr;
    INT32U i,buffer_ptr,sp_num_addr,flag1;
    SpN_ptr sp_ptr;
    V3D_POS_STRUCT Sprite_pos;

    ppu_driver_init(ppu_register_set, 0, DISP_FMT_YUYV);

	// Now configure Sprite
	gplib_ppu_sprite_init(ppu_register_set);
	gplib_ppu_sprite_enable_set(ppu_register_set, 1);			                            // Enable Sprite
	gplib_ppu_sprite_coordinate_set(ppu_register_set, 0);                                   // set sprite coordinate
	gplib_ppu_sprite_direct_mode_set(ppu_register_set, 1);		                            // Set sprite address mode
	gplib_ppu_sprite_number_set(ppu_register_set, 256);                                     // Set sprite number
	gplib_ppu_sprite_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteRAM);            // set sprite ram buffer
    gplib_ppu_sprite_extend_attribute_ram_ptr_set(ppu_register_set, (INT32U)SpriteExRAM);   // value: 32-bit pointer to sprite extend attribute ram
    gplib_ppu_sprite_segment_set(ppu_register_set, (INT32U)0);                              // sprite cell data
    gplib_ppu_sprite_25d_mode_set(ppu_register_set, 1);

    // image and position init
    set_sprite_init(0,(INT32U)&Sprite01B_SP);
    set_sprite_display_init(0,0,0,(INT32U)_Sprite01B_IMG0000_CellIdx);

    buffer_ptr = (INT32U)gp_malloc_align(((64*64*2)+256), 64);
    if(buffer_ptr == 0)
    {
        DBG_PRINT("ppu_frame_buffer malloc error\r\n");
        return -1;
    }
    buffer_ptr = (buffer_ptr + VTECH_BUF_ALIGN) & ~VTECH_BUF_ALIGN;
    src_ptr = (INT16U *)_SPRITE_64_yuyv3_CellData;
    tar_ptr = (INT16U *)buffer_ptr;
    for(i=0;i<(64*64*2);i++)
       *tar_ptr++ = *src_ptr++;

    flag1 = 0;
    while(1)
    {
        paint_ppu_spriteram(ppu_register_set,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,2);
        Get_sprite_image_info(0,(SpN_ptr *)&sp_ptr);
        sp_num_addr = sp_ptr.nSPNum_ptr;
        if(flag1)
        {
            Sprite_pos.V3D_POS1.x0=64;
            Sprite_pos.V3D_POS1.y0=91;
            Sprite_pos.V3D_POS1.x1=140;
            Sprite_pos.V3D_POS1.y1=91;
            Sprite_pos.V3D_POS1.x2=140;
            Sprite_pos.V3D_POS1.y2=181;
            Sprite_pos.V3D_POS1.x3=64;
            Sprite_pos.V3D_POS1.y3=181;
            Sprite_pos.group_id=2;
            flag1 = 0;
        }
        else
        {
            Sprite_pos.V3D_POS1.x0=64;
            Sprite_pos.V3D_POS1.y0=91;
            Sprite_pos.V3D_POS1.x1=78;
            Sprite_pos.V3D_POS1.y1=91;
            Sprite_pos.V3D_POS1.x2=78;
            Sprite_pos.V3D_POS1.y2=121;
            Sprite_pos.V3D_POS1.x3=64;
            Sprite_pos.V3D_POS1.y3=121;
            Sprite_pos.group_id=2;
            flag1 = 1;
        }
        SpriteV3D_set(0,Sprite_Coordinate_Freemode,LeftTop2Center_coordinate,&sp_ptr,&Sprite_pos);
        gplib_ppu_sprite_attribute_character_number_set((SpN_RAM *)sp_num_addr, (INT32U)(buffer_ptr/2));

        // Start PPU and wait until PPU operation is done
        gplib_ppu_go_and_wait_done(ppu_register_set);
        ppu_disp_buffer_set(ppu_register_set, ppu_frame_buffer_display_get());
        osDelay(1000);

        if(ppu_driver_exit_mode)
            break;
    }

    return 0;
}


