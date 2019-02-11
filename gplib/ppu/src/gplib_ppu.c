/*
* Description: This library provides PPU APIs to init, control and start PPU hardware engine.
*
* Author: Tristan Yang
*
* Date: 2008/03/03
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.00
*/

#include "gplib_ppu.h"

#if (defined GPLIB_PPU_EN) && (GPLIB_PPU_EN == 1)

#define	PPU_YUYV_TYPE3					(3<<20)
#define	PPU_YUYV_TYPE2					(2<<20)
#define	PPU_YUYV_TYPE1					(1<<20)
#define	PPU_YUYV_TYPE0					(0<<20)

#define	PPU_RGBG_TYPE3					PPU_YUYV_TYPE3
#define	PPU_RGBG_TYPE2					PPU_YUYV_TYPE2
#define	PPU_RGBG_TYPE1					PPU_YUYV_TYPE1
#define	PPU_RGBG_TYPE0					PPU_YUYV_TYPE0

#define	PPU_YUYV_MODE					(1<<10)
#define	PPU_RGBG_MODE			        (0<<10)

#define	PPU_YUYV_RGBG_FORMAT_MODE		(1<<8)
#define	PPU_RGB565_MODE			        (0<<8)

INT32S gplib_ppu_init(PPU_REGISTER_SETS *p_register_set)
{
	// Initiate PPU hardware and driver
	ppu_init();

	// Initiate PPU register sets
	gp_memset((INT8S *) p_register_set, 0x0, (INT32U) sizeof(PPU_REGISTER_SETS));
	p_register_set->update_register_flag = C_UPDATE_REG_SET_PPU | C_UPDATE_REG_SET_TEXT1 | C_UPDATE_REG_SET_TEXT2 | C_UPDATE_REG_SET_TEXT3 | C_UPDATE_REG_SET_TEXT4 | C_UPDATE_REG_SET_SPRITE;

	// window max size
	p_register_set->ppu_window1_x = 0x7FF;
	p_register_set->ppu_window1_y = 0x7FF;
	p_register_set->ppu_window2_x = 0x7FF;
	p_register_set->ppu_window2_y = 0x7FF;
	p_register_set->ppu_window3_x = 0x7FF;
	p_register_set->ppu_window3_y = 0x7FF;
	p_register_set->ppu_window4_x = 0x7FF;
	p_register_set->ppu_window4_y = 0x7FF;

	return 0;
}

INT32S gplib_ppu_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~PPU_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_char0_transparent_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= CHAR0_TRANSPARENT_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~CHAR0_TRANSPARENT_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_bottom_up_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= TX_BOT2UP;
	} else {
		p_register_set->ppu_enable &= ~TX_BOT2UP;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_vga_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_VGA;
	} else {
		p_register_set->ppu_enable &= ~PPU_VGA;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_non_interlace_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_VGA_NONINTL;
	} else {
		p_register_set->ppu_enable &= ~PPU_VGA_NONINTL;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_frame_buffer_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U enable, INT32U select)
{
	if (!p_register_set) {
		return -1;
	}

	if (enable) {
		p_register_set->ppu_enable |= PPU_FRAME_BASE;
	} else {
		p_register_set->ppu_enable &= ~PPU_FRAME_BASE;
	}

	if (select) {
		p_register_set->ppu_enable |= FB_SEL1;
	} else {
		p_register_set->ppu_enable &= ~FB_SEL1;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

// PPU output format
INT32S gplib_ppu_fb_format_set(PPU_REGISTER_SETS *p_register_set, INT32U format, INT32U mono)
{
	if (!p_register_set || mono>3) {
		return -1;
	}

	if (format) {
		p_register_set->ppu_enable |= PPU_RGBG;
	} else {
		p_register_set->ppu_enable &= ~PPU_RGBG;
	}
	p_register_set->ppu_enable &= ~MASK_FB_MONO;
	p_register_set->ppu_enable |= (mono<<B_FB_MONO) & MASK_FB_MONO;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_save_rom_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= SAVE_ROM_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~SAVE_ROM_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_resolution_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || value>7) {
		return -1;
	}

	p_register_set->ppu_enable &= ~MASK_TFT_SIZE;
	p_register_set->ppu_enable |= (value<<B_TFT_SIZE) & MASK_TFT_SIZE;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_yuv_type_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || value>7) {
		return -1;
	}

	p_register_set->ppu_enable &= ~MASK_YUV_TYPE;
	p_register_set->ppu_enable |= (value<<B_YUV_TYPE) & MASK_YUV_TYPE;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

// This function takes effect in TFT frame mode only
INT32S gplib_ppu_tft_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_TFT_LONG_BURST;
	} else {
		p_register_set->ppu_enable &= ~PPU_TFT_LONG_BURST;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

// This function takes effect in both frame mode and line mode
// It is not suggested to enable this function when TFT long burst is also enabled
INT32S gplib_ppu_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_LONG_BURST;
        // long burst use must be set 1 with 0xD0500334 bit[1] or bit[2] or bit[3] or bit[4] or bit[6] or bit[16] or bit[17] or bit[18].
        p_register_set->extend_sprite_control |= 0x4;
	} else {
		p_register_set->ppu_enable &= ~PPU_LONG_BURST;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_blend4_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || value>3) {
		return -1;
	}

	p_register_set->ppu_blending_level = (INT16U) value;

	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_rgb565_transparent_color_set(PPU_REGISTER_SETS *p_register_set, INT32U enable, INT32U value)
{
	if (!p_register_set || value>0xFFFFFF) {
		return -1;
	}
	if (enable) {
		p_register_set->ppu_rgb565_transparent_color = (1<<24) | value;
	} else {
		p_register_set->ppu_rgb565_transparent_color = value;
	}

	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_fade_effect_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || value > 0xFF) {
		return -1;
	}

	p_register_set->ppu_fade_control = (INT16U)(value & 0xFF);

	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_window_mask_set(PPU_REGISTER_SETS *p_register_set, INT32U window_index, INT32U window_mask)
{
	#define WIN_BIT_31        0x80000000
	if (!p_register_set || window_index > 2) {
		return -1;
	}

	switch (window_index) {
        case 0:			// Window 1
            if(window_mask)
                p_register_set->ppu_window1_x |= WIN_BIT_31;
            else
                p_register_set->ppu_window1_x &= ~WIN_BIT_31;
            break;
        case 1:			// Window 2
            if(window_mask)
                p_register_set->ppu_window2_x |= WIN_BIT_31;
            else
                p_register_set->ppu_window2_x &= WIN_BIT_31;
            break;
        case 2:			// Window 3
            if(window_mask)
                p_register_set->ppu_window3_x |= WIN_BIT_31;
            else
                p_register_set->ppu_window3_x &= WIN_BIT_31;
            break;
	}

	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_window_set(PPU_REGISTER_SETS *p_register_set, INT32U window_index, INT32U window_x, INT32U window_y)
{
	#define WIN_SIZE        0x7FFFFFFF
	INT32U win_x,win_y;

	if (!p_register_set || window_index > 3) {
		return -1;
	}

	switch (window_index) {
        case 0:			// Window 1
            win_x = p_register_set->ppu_window1_x;
            win_x &= ~WIN_SIZE;
            win_x |= window_x;
            win_y = p_register_set->ppu_window1_y;
            win_y &= ~WIN_SIZE;
            win_y |= window_y;
            p_register_set->ppu_window1_x = win_x;
            p_register_set->ppu_window1_y = win_y;
            break;
        case 1:			// Window 2
            win_x = p_register_set->ppu_window2_x;
            win_x &= ~WIN_SIZE;
            win_x |= window_x;
            win_y = p_register_set->ppu_window2_y;
            win_y &= ~WIN_SIZE;
            win_y |= window_y;
            p_register_set->ppu_window2_x = win_x;
            p_register_set->ppu_window2_y = win_y;
            break;
        case 2:			// Window 3
            win_x = p_register_set->ppu_window3_x;
            win_x &= ~WIN_SIZE;
            win_x |= window_x;
            win_y = p_register_set->ppu_window3_y;
            win_y &= ~WIN_SIZE;
            win_y |= window_y;
            p_register_set->ppu_window3_x = win_x;
            p_register_set->ppu_window3_y = win_y;
            break;
        case 3:			// Window 4
            p_register_set->ppu_window4_x = window_x;
            p_register_set->ppu_window4_y = window_y;
            break;
	}

	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_mask_window_range_set(PPU_REGISTER_SETS *p_register_set, INT32U window_index, INT32U window_mask, INT32U window_x_start, INT32U window_x_stop, INT32U window_y_start, INT32U window_y_stop)
{
    #define WIN_MASK_SIZE           0x3FF
    #define WIN_SHIFT_BIT           16
    INT32U win_x,win_y;
    INT32S ret;

	if (!p_register_set || window_index > 3) {
		return -1;
	}

    win_x = ((window_x_start & WIN_MASK_SIZE) << WIN_SHIFT_BIT) | ((window_x_stop & WIN_MASK_SIZE));
    win_y = ((window_y_start & WIN_MASK_SIZE) << WIN_SHIFT_BIT) | ((window_y_stop & WIN_MASK_SIZE));

    ret = gplib_ppu_window_set(p_register_set, window_index, win_x, win_y);
    if(ret < 0)
        return ret;

    ret = gplib_ppu_window_mask_set(p_register_set, window_index, window_mask);

	return ret;
}

INT32S gplib_ppu_palette_type_set(PPU_REGISTER_SETS *p_register_set, INT32U p1024, INT32U type)
{
	if (!p_register_set || type>0x3) {
		return -1;
	}

	if (p1024) {
		p_register_set->ppu_palette_control = (1<<4) | type;
	} else {
		p_register_set->ppu_palette_control = type;
	}

	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_palette_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U bank, INT32U value)
{
#if NEW_PAL_RAM_16_EN == 1
	if (!p_register_set || bank > 15) {
#else
    if (!p_register_set || bank > 7) {
#endif
		return -1;
	}

	if(bank > 3)
	{
		p_register_set->ppu_new_palette_ptr[bank-4] = value;
		switch(bank)
		{
			case 4:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE4;
				break;
			case 8:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE8;
				break;
			case 12:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE12;
				break;

			case 5:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE5;
				break;
			case 9:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE9;
				break;
			case 13:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE13;
				break;

			case 6:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE6;
				break;
			case 10:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE10;
				break;
			case 14:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE14;
				break;

		    case 7:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE7;
				break;
			case 11:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE11;
				break;
			case 15:
				p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE15;
				break;
		}
	}
	else
	{
		if (bank == 0) {
			p_register_set->ppu_palette0_ptr = value;
			p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE0;
		} else if (bank == 1) {
			p_register_set->ppu_palette1_ptr = value;
			p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE1;
		} else if (bank == 2) {
			p_register_set->ppu_palette2_ptr = value;
			p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE2;
		} else if (bank == 3) {
			p_register_set->ppu_palette3_ptr = value;
			p_register_set->update_register_flag |= C_UPDATE_REG_SET_PALETTE3;
		}
	}
	return 0;
}

INT32S gplib_ppu_frame_buffer_add(PPU_REGISTER_SETS *p_register_set, INT32U buffer)
{
	if (!p_register_set || !buffer) {
		return -1;
	}

	if (ppu_frame_buffer_add((INT32U *) buffer)) {		// Add frame buffer to PPU driver layer
		return -1;
	}

	if ((buffer & 0x3F) && (p_register_set->ppu_enable & PPU_TFT_LONG_BURST)) {
		// Disable long burst mode when frame buffer is not 64-byte alignment
		p_register_set->ppu_enable &= ~PPU_TFT_LONG_BURST;
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;
	}

	return 0;
}

// This function is only valid under VGA/Interlace/Frame mode
INT32S gplib_ppu_deflicker_para_set(PPU_REGISTER_SETS *p_register_set, INT8U para1, INT8U para2, INT8U para3)
{
    INT32U para;

    if (!p_register_set) {
		return -1;
	}

    para = (((para3 & 0xF) << 8) | ((para2 & 0xF) << 4) | (para1 & 0xF));
    p_register_set->ppu_deflicker_para = para;

    return 0;
}

INT32S gplib_ppu_go_without_wait(PPU_REGISTER_SETS *p_register_set)
{
	INT32S result;

	result = ppu_go(p_register_set, 0, 0);
	if (result == 0) {
		gplib_ppu_text_number_array_update_flag_clear();
	}
	return result;
}

INT32S gplib_ppu_go(PPU_REGISTER_SETS *p_register_set)
{
	INT32S result;

	result = ppu_go(p_register_set, 1, 0);
	if (result == 0) {
		gplib_ppu_text_number_array_update_flag_clear();
	}
	return result;
}

INT32S gplib_ppu_go_and_wait_done(PPU_REGISTER_SETS *p_register_set)
{
	INT32S result;

	result = ppu_go(p_register_set, 1, 1);
	if (result == 0) {
		gplib_ppu_text_number_array_update_flag_clear();
	}
	return result;
}

INT32S gplib_ppu_go_hblank_first(PPU_REGISTER_SETS *p_register_set)
{
	INT32S result;

	result = set_ppu_go(p_register_set);

	return result;
}

INT32S gplib_ppu_hblnk_set(INT32U value)
{
    INT32S result;

    result=Hblenk_Enable_Set(value);

	return result;
}

INT32S gplib_ppu_hblnk_line_offset_set(INT32U line_offset)
{
    Hblenk_Line_Offset_Set(line_offset);

	return 0;
}

INT32S gplib_ppu_hblnk_irq_wait(void)
{
    INT32S result;

    result=Hblenk_Wait();

	return result;
}

INT32S gplib_ppu_hblnk_go_now(PPU_REGISTER_SETS *p_register_set)
{
    INT32S result;

    result=ppu_setting_update(p_register_set);
    if(result==0)
      Hblenk_Go();

	return result;
}

//----------------------------------------------------------------------------------
#if 1//((MCU_VERSION >= GPL326XX)&&(MCU_VERSION < GPL327XX))
// PPU new Function

INT32S gplib_ppu_dual_blend_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{

	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= DUAL_BLEND_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~DUAL_BLEND_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;

}

INT32S gplib_ppu_deflicker_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{

	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_DEFEN_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~PPU_DEFEN_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;

}

INT32S gplib_ppu_free_size_set(PPU_REGISTER_SETS *p_register_set, INT16U INTL,INT16U H_size,INT16U V_size)
{

	if (!p_register_set) {
		return -1;
	}

        p_register_set->ppu_free_mode = (((INTL << B_FREE_INIT) & MASK_FREE_INIT_SIZE)|((H_size << B_FREE_H_SIZE) & MASK_FREE_H_SIZE )|(V_size & MASK_FREE_V_SIZE));

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;

}

INT32S gplib_ppu_text_rgba_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{

	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TEXT_RGBA_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TEXT_RGBA_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;

}

INT32S gplib_ppu_text_alpha_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{

	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TXT_ALPHA_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TXT_ALPHA_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;

}

INT32S gplib_ppu_sprite_rgba_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{

	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= SPRITE_RGBA_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~SPRITE_RGBA_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_sprite_addrx2_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{

	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= SP_ADDR_X2_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~SP_ADDR_X2_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}


INT32S gplib_ppu_free_mode_size_rigister_set(INT16U INTL,INT16U H_size,INT16U V_size)
{
        set_ppu_free_control(INTL,H_size,V_size);
	return 0;

}

INT32S gplib_ppu_text_new_specialbmp_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TXT_NEWSPECBMP_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TXT_NEWSPECBMP_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_text_new_compression_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TXT_NEWCMP_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TXT_NEWCMP_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_go_delay_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TXT_DELGO_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TXT_DELGO_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_tft_vga2qvga_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TXT_TFTVTQ_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TXT_TFTVTQ_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}


INT32S gplib_ppu_tv_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TXT_TVLB_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TXT_TVLB_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_tv_upscaling_set(PPU_REGISTER_SETS *p_register_set, INT32U value, INT32U set_value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value)
	{
	  if (value == 0) {
		  p_register_set->ppu_misc |= TXT_INTPMODE0;
	  } else if(value == 1){
		  p_register_set->ppu_misc |= TXT_INTPMODE1;
	  } else if(value == 2){
	      p_register_set->ppu_misc |= TXT_INTPMODE2;
	  } else if(value == 3){
	      p_register_set->ppu_misc |= TXT_INTPMODE3;
	  }
    }
    else
    {
        p_register_set->ppu_misc &= ~TXT_INTPMODE3;
    }

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}
#endif
#if 1//((MCU_VERSION == GPL326XXB))
INT32S gplib_ppu_argb888_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= PPU_ARGB888_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~PPU_ARGB888_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_text_bitmap_flip_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= TXT_BITMAP_FLIP_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~TXT_BITMAP_FLIP_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_ui_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_ui_enable |= PPU_UI_ENABLE;
	} else {
		p_register_set->ppu_ui_enable &= ~PPU_UI_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_ui_disp_selection_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_ui_enable |= B_PPU_UI_DISP_HDMI;
	} else {
		p_register_set->ppu_ui_enable &= ~B_PPU_UI_DISP_HDMI;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_ui_color_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{

    if (!p_register_set || value > 3) {
		return -1;
	}

  	p_register_set->ppu_ui_enable &= ~MASK_PPU_UI_COLOR_TYPE;
	p_register_set->ppu_ui_enable |= (value<<B_PPU_UI_COLOR_TYPE) & MASK_PPU_UI_COLOR_TYPE;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_ui_blend_level_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
    if (!p_register_set || value > 15) {
		return -1;
	}

  	p_register_set->ppu_ui_enable &= ~MASK_PPU_UI_BLEND_LEVEL;
	p_register_set->ppu_ui_enable |= (value<<B_PPU_UI_BLEND_LEVEL) & MASK_PPU_UI_BLEND_LEVEL;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_ui_alpha_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_ui_enable |= PPU_UI_ALPHA_ENABLE;
	} else {
		p_register_set->ppu_ui_enable &= ~PPU_UI_ALPHA_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_ui_addr_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || !value) {
		return -1;
	}

	p_register_set->ppu_ui_fbi_addr = value;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_post_process_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_POST_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~PPU_POST_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_fb_lock_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_misc |= FB_LOCK_ENABLE;
	} else {
		p_register_set->ppu_misc &= ~FB_LOCK_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_fb_lock_process_enable_set(PPU_REGISTER_SETS *p_register_set, FB_LOCK_STRUCT *fb_ptr)
{
	INT32U fb_lock_color1,fb_lock_color2,fb_lock_size1,fb_lock_size2;

	if (!p_register_set || !fb_ptr) {
		return -1;
	}

    switch(fb_ptr->color1)
    {
        case PPU_FMT_RGB565:
            fb_lock_color1=(PPU_RGB565_MODE);
            break;

        case PPU_FMT_BGRG:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE0);
            break;

        case PPU_FMT_GBGR:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE1);
            break;

        case PPU_FMT_RGBG:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE2);
            break;

         case PPU_FMT_GRGB:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE3);
            break;

        case PPU_FMT_VYUV:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE0);
            break;

        case PPU_FMT_YVYU:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE1);
            break;

        case PPU_FMT_UYVY:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE2);
            break;

         case PPU_FMT_YUYV:
            fb_lock_color1=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE3);
            break;
    }
    fb_lock_size1 = ((fb_ptr->h_size1 & 0x7FF) << 16) | (fb_ptr->v_size1 & 0x7FF);

    switch(fb_ptr->color2)
    {
        case PPU_FMT_RGB565:
            fb_lock_color2=(PPU_RGB565_MODE);
            break;

        case PPU_FMT_BGRG:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE0);
            break;

        case PPU_FMT_GBGR:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE1);
            break;

        case PPU_FMT_RGBG:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE2);
            break;

         case PPU_FMT_GRGB:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_RGBG_MODE|PPU_RGBG_TYPE3);
            break;

        case PPU_FMT_VYUV:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE0);
            break;

        case PPU_FMT_YVYU:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE1);
            break;

        case PPU_FMT_UYVY:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE2);
            break;

         case PPU_FMT_YUYV:
            fb_lock_color2=(PPU_YUYV_RGBG_FORMAT_MODE|PPU_YUYV_MODE|PPU_YUYV_TYPE3);
            break;
    }
    fb_lock_size2 = ((fb_ptr->h_size2 & 0x7FF) << 16) | (fb_ptr->v_size2 & 0x7FF);

    fd_lock_set(fb_lock_color1, fb_lock_size1, fb_lock_color2, fb_lock_size2);

    gplib_ppu_fb_lock_enable_set(p_register_set, 1);

	return 0;
}
#endif
#endif

INT32S gplib_ppu_rgb_offset_set(PPU_REGISTER_SETS *p_register_set, INT8U r_value, INT8U g_value, INT8U b_value)
{
    INT32U rgb_offset;

	if (!p_register_set) {
		return -1;
	}

    rgb_offset = ( ((r_value & 0xF) << 8)| ((g_value & 0xF) << 4) | (b_value & 0xF));
    p_register_set->ppu_special_effect_rgb = rgb_offset;

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

/**
* @brief 		PPU module enable function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	value [in]: value:0=disable 1=enable.
* @return 	SUCCESS/ERROR_ID.
*/
INT32S gplib_ppu_dma_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~PPU_ENABLE;
	}

	p_register_set->ppu_enable &= ~PPU_VGA;
	p_register_set->ppu_enable &= ~PPU_VGA_NONINTL;
	p_register_set->ppu_enable &= ~SAVE_ROM_ENABLE;
	p_register_set->ppu_enable &= ~MASK_TFT_SIZE;
	//p_register_set->ppu_enable &= ~PPU_CM_ENABLE;
	p_register_set->ppu_enable &= ~PPU_DEFEN_ENABLE;
	//p_register_set->ppu_enable &= ~PPU_LONG_BURST;
	//p_register_set->ppu_enable &= ~FB_SEL1;

	p_register_set->ppu_misc &= ~TXT_NEWCMP_ENABLE;
	//p_register_set->ppu_misc &= ~TV_FLIP_ENABLE;
	//p_register_set->ppu_misc &= ~TFT_FLIP_ENABLE;
	p_register_set->ppu_misc &= ~TXT_DELGO_ENABLE;
	p_register_set->ppu_misc &= ~TXT_TFTVTQ_ENABLE;
	p_register_set->ppu_misc &= ~TXT_TVLB_ENABLE;

	p_register_set->ppu_enable |= TX_BOT2UP;
	p_register_set->ppu_enable |= PPU_FRAME_BASE;
	p_register_set->ppu_enable |= FB_SEL1;
	p_register_set->ppu_enable |= PPU_LONG_BURST;
	p_register_set->ppu_enable |= PPU_TFT_LONG_BURST;
	//p_register_set->ppu_enable |= PPU_VGA_NONINTL;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_dma_go(PPU_REGISTER_SETS *p_register_set)
{
	INT32S result;

	result = ppu_simple_go(p_register_set);
	if (result == 0) {
		gplib_ppu_text_number_array_update_flag_clear();
	}
	return result;
}

INT32S gblib_ppu_wait_poll(INT32U wait)
{

	return ppu_simple_frame_end_get(wait);
}

