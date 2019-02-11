#ifndef __GPLIB_PPU_H__
#define __GPLIB_PPU_H__


#include "drv_l1_ppu.h"

typedef enum
{
	PPU_FMT_RGB565 = 0,
	PPU_FMT_BGRG,
	PPU_FMT_GBGR,
	PPU_FMT_RGBG,
	PPU_FMT_GRGB,
	PPU_FMT_VYUV,
	PPU_FMT_YVYU,
	PPU_FMT_UYVY,
	PPU_FMT_YUYV,
	PPU_FMT_MAX
} PPU_FMT;

typedef struct {
  INT32U color1;
  INT32U h_size1;
  INT32U v_size1;
  INT32U color2;
  INT32U h_size2;
  INT32U v_size2;
} FB_LOCK_STRUCT, *FB_LOCK_STRUCT_PTR;

#define	B_POST_EN						27
#define	PPU_POST_ENABLE			        (((INT32U) 1)<<B_POST_EN)
#define	B_DEFEN_EN						25
#define	PPU_DEFEN_ENABLE			    (((INT32U) 1)<<B_DEFEN_EN)
#define	B_TFT_LB						24
#define	PPU_TFT_LONG_BURST				(((INT32U) 1)<<B_TFT_LB)
#define	B_YUV_TYPE						20
#define	MASK_YUV_TYPE					(((INT32U) 0x7)<<B_YUV_TYPE)
#define	B_PPU_LB						19
#define	PPU_LONG_BURST					(((INT32U) 1)<<B_PPU_LB)
#define	B_TFT_SIZE						16
#define	MASK_TFT_SIZE					(((INT32U) 0x7)<<B_TFT_SIZE)
#define	B_SAVE_ROM						15
#define	SAVE_ROM_ENABLE					(((INT32U) 1)<<B_SAVE_ROM)
#define	B_FB_SEL						14
#define	FB_SEL0							(((INT32U) 0)<<B_FB_SEL)
#define	FB_SEL1							(((INT32U) 1)<<B_FB_SEL)
#define	B_SPR_WIN						13
#define	SPR_WIN_DISABLE					(((INT32U) 0)<<B_SPR_WIN)
#define	SPR_WIN_ENABLE					(((INT32U) 1)<<B_SPR_WIN)
#define	B_HVCMP_DIS						12
#define	HVCMP_ENABLE					(((INT32U) 0)<<B_HVCMP_DIS)
#define	HVCMP_DISABLE					(((INT32U) 1)<<B_HVCMP_DIS)
#define	B_FB_MONO						10
#define	MASK_FB_MONO					(((INT32U) 0x3)<<B_FB_MONO)
#define	B_SPR25D						9
#define	PPU_SPR25D_DISABLE				(((INT32U) 0)<<B_SPR25D)
#define	PPU_SPR25D_ENABLE				(((INT32U) 1)<<B_SPR25D)
#define	B_FB_FORMAT						8
#define	PPU_RGB565						(((INT32U) 0)<<B_FB_FORMAT)
#define	PPU_RGBG						(((INT32U) 1)<<B_FB_FORMAT)
#define	B_FB_EN							7
#define	PPU_LINE_BASE					(((INT32U) 0)<<B_FB_EN)
#define	PPU_FRAME_BASE					(((INT32U) 1)<<B_FB_EN)
#define	B_VGA_NONINTL					5
#define	PPU_VGA_INTL					(((INT32U) 0)<<B_VGA_NONINTL)
#define	PPU_VGA_NONINTL					(((INT32U) 1)<<B_VGA_NONINTL)
#define	B_VGA_EN						4
#define	PPU_QVGA						(((INT32U) 0)<<B_VGA_EN)
#define	PPU_VGA							(((INT32U) 1)<<B_VGA_EN)
#define	B_TX_BOTUP						3
#define	TX_UP2BOT						(((INT32U) 0)<<B_TX_BOTUP)
#define	TX_BOT2UP						(((INT32U) 1)<<B_TX_BOTUP)
#define	B_TX_DIRECT						2
#define	TX_RELATIVE_ADDRESS				(((INT32U) 0)<<B_TX_DIRECT)
#define	TX_DIRECT_ADDRESS				(((INT32U) 1)<<B_TX_DIRECT)
#define	B_CHAR0_TRANSPARENT				1
#define	CHAR0_TRANSPARENT_ENABLE		(((INT32U) 1)<<B_CHAR0_TRANSPARENT)
#define	B_PPU_EN						0
#define	PPU_DISABLE						(((INT32U) 0)<<B_PPU_EN)
#define	PPU_ENABLE                      (((INT32U) 1)<<B_PPU_EN)

#define	B_PPU_UI_ALPHA_EN				8
#define	PPU_UI_ALPHA_DISABLE		    (((INT32U) 0)<<B_PPU_UI_ALPHA_EN)
#define	PPU_UI_ALPHA_ENABLE				(((INT32U) 1)<<B_PPU_UI_ALPHA_EN)
#define	B_PPU_UI_BLEND_LEVEL		    4
#define	MASK_PPU_UI_BLEND_LEVEL			(((INT32U) 0xF)<<B_PPU_UI_BLEND_LEVEL)
#define	B_PPU_UI_COLOR_TYPE				2
#define	MASK_PPU_UI_COLOR_TYPE			(((INT32U) 0x3)<<B_PPU_UI_COLOR_TYPE)
#define	B_PPU_UI_DISP_SEL				1
#define	B_PPU_UI_DISP_TFT				(((INT32U) 0)<<B_PPU_UI_DISP_SEL)
#define	B_PPU_UI_DISP_HDMI				(((INT32U) 1)<<B_PPU_UI_DISP_SEL)
#define	B_PPU_UI_EN						0
#define	PPU_UI_DISABLE					(((INT32U) 0)<<B_PPU_UI_EN)
#define	PPU_UI_ENABLE					(((INT32U) 1)<<B_PPU_UI_EN)

#define	B_TXT_BITMAP_FLIP				25
#define	TXT_BITMAP_FLIP_DISABLE		    (((INT32U) 0)<<B_TXT_BITMAP_FLIP)
#define	TXT_BITMAP_FLIP_ENABLE			(((INT32U) 1)<<B_TXT_BITMAP_FLIP)
#define	B_PPU_ARGB888				    24
#define	PPU_ARGB888_DISABLE		        (((INT32U) 0)<<B_PPU_ARGB888)
#define	PPU_ARGB888_ENABLE			    (((INT32U) 1)<<B_PPU_ARGB888)
#define	B_TXT_NEWSPECBMP				16
#define	TXT_NEWSPECBMP_DISABLE		    (((INT32U) 0)<<B_TXT_NEWSPECBMP)
#define	TXT_NEWSPECBMP_ENABLE			(((INT32U) 1)<<B_TXT_NEWSPECBMP)
#define	B_TXT_TVLB					    15
#define	TXT_TVLB_DISABLE				(((INT32U) 0)<<B_TXT_TVLB)
#define	TXT_TVLB_ENABLE				    (((INT32U) 1)<<B_TXT_TVLB)
#define	B_TXT_TFTVTQ					13
#define	TXT_TFTVTQ_DISABLE				(((INT32U) 0)<<B_TXT_TFTVTQ)
#define	TXT_TFTVTQ_ENABLE				(((INT32U) 1)<<B_TXT_TFTVTQ)
#define	B_TXT_DELGO					    12
#define	TXT_DELGO_DISABLE				(((INT32U) 0)<<B_TXT_DELGO)
#define	TXT_DELGO_ENABLE				(((INT32U) 1)<<B_TXT_DELGO)
#define	B_TXT_INTPMODE					10
#define	TXT_INTPMODE0				    (((INT32U) 0)<<B_TXT_INTPMODE)
#define	TXT_INTPMODE1				    (((INT32U) 1)<<B_TXT_INTPMODE)
#define	TXT_INTPMODE2				    (((INT32U) 2)<<B_TXT_INTPMODE)
#define	TXT_INTPMODE3				    (((INT32U) 3)<<B_TXT_INTPMODE)
#define	B_TXT_NEWCMP					9
#define	TXT_NEWCMP_DISABLE				(((INT32U) 0)<<B_TXT_NEWCMP)
#define	TXT_NEWCMP_ENABLE				(((INT32U) 1)<<B_TXT_NEWCMP)
#define	B_TXT_ALPHA					    8
#define	TXT_ALPHA_DISABLE				(((INT32U) 0)<<B_TXT_ALPHA)
#define	TXT_ALPHA_ENABLE				(((INT32U) 1)<<B_TXT_ALPHA)
#define	B_SP_ADDR_X2					7
#define	SP_ADDR_X2_DISABLE				(((INT32U) 0)<<B_SP_ADDR_X2)
#define	SP_ADDR_X2_ENABLE				(((INT32U) 1)<<B_SP_ADDR_X2)
#define	B_DUAL_BLEND					6
#define	DUAL_BLEND_DISABLE				(((INT32U) 0)<<B_DUAL_BLEND)
#define	DUAL_BLEND_ENABLE				(((INT32U) 1)<<B_DUAL_BLEND)
#define	B_SPRITE_RGBA					3
#define	SPRITE_RGBA_DISABLE				(((INT32U) 0)<<B_SPRITE_RGBA)
#define	SPRITE_RGBA_ENABLE				(((INT32U) 1)<<B_SPRITE_RGBA)
#define	B_TEXT_RGBA				        2
#define	TEXT_RGBA_DISABLE		        (((INT32U) 0)<<B_TEXT_RGBA)
#define	TEXT_RGBA_ENABLE		        (((INT32U) 1)<<B_TEXT_RGBA)
#define	B_FB_LOCK_EN					1
#define	FB_LOCK_DISABLE					(((INT32U) 0)<<B_FB_LOCK_EN)
#define	FB_LOCK_ENABLE					(((INT32U) 1)<<B_FB_LOCK_EN)
#define	B_FREE_INIT					    31
#define	MASK_FREE_INIT_SIZE			    (((INT32U) 0x1)<<B_FREE_INIT)
#define	B_FREE_H_SIZE					16
#define	MASK_FREE_H_SIZE			    (((INT32U) 0x7FF)<<B_FREE_H_SIZE)
#define	B_FREE_V_SIZE					0
#define	MASK_FREE_V_SIZE			    (((INT32U) 0x7FF)<<B_FREE_V_SIZE)
/*
* Function Name :  gplib_ppu_init
*
* Syntax : INT32S gplib_ppu_init(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to initiate PPU software structure
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_init(PPU_REGISTER_SETS *p_register_set);

/*
* Function Name :  gplib_ppu_enable_set
*
* Syntax : INT32S gplib_ppu_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable PPU operation
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value : 0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_char0_transparent_set
*
* Syntax : INT32S gplib_ppu_char0_transparent_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable text character 0 auto transparent mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value : 0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_char0_transparent_set(PPU_REGISTER_SETS *p_register_set, INT32U value);	// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_bottom_up_mode_set
*
* Syntax : INT32S gplib_ppu_bottom_up_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set PPU depth calculation order
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value : 0=from top to bottom 1=from bottom to top
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_bottom_up_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);	// value:0=from top to bottom 1=from bottom to top

/*
* Function Name :  gplib_ppu_vga_mode_set
*
* Syntax : INT32S gplib_ppu_vga_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable VGA mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value : 0=QVGA mode 1=VGA mode
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_vga_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// value:0=QVGA mode 1=VGA mode

/*
* Function Name :  gplib_ppu_non_interlace_set
*
* Syntax : INT32S gplib_ppu_non_interlace_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable interlace mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value : 0=interlace mode 1=non-interlace mode
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_non_interlace_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// vlaue:0=interlace mode 1=non-interlace mode

/*
* Function Name :  gplib_ppu_frame_buffer_mode_set
*
* Syntax : INT32S gplib_ppu_frame_buffer_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U enable, INT32U select);
*
* Purpose :  User can call this function to enable/disable frame buffer mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  enable:
*										select:
*													enable:0 , select:0=TV is line mode and TFT is frame mode
*													enable:1=both TV and TFT are frame buffer mode
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_frame_buffer_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U enable, INT32U select);	// enable:0(select:0=TV is line mode and TFT is frame mode) 1=both TV and TFT are frame buffer mode

/*
* Function Name :  gplib_ppu_fb_format_set
*
* Syntax : INT32S gplib_ppu_fb_format_set(PPU_REGISTER_SETS *p_register_set, INT32U format, INT32U mono);
*
* Purpose :  User can call this function to frame buffer data format
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  format:
*										mono:
*													format: 0, mono:0=RGB565 1=Mono 2=4-color 3=16-color
* 												format: 1, mono:0=RGBG 1=YUYV 2=RGBG 3=YUYV
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_fb_format_set(PPU_REGISTER_SETS *p_register_set, INT32U format, INT32U mono);	// format:0(mono:0=RGB565 1=Mono 2=4-color 3=16-color) 1(mono:0=RGBG 1=YUYV 2=RGBG 3=YUYV)

/*
* Function Name :  gplib_ppu_save_rom_set
*
* Syntax : INT32S gplib_ppu_save_rom_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable rom saving mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 0=disable save rom mode 1=enable save rom mode
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_save_rom_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// vlaue:0=disable save rom mode 1=enable save rom mode

/*
* Function Name :  gplib_ppu_resolution_set
*
* Syntax : INT32S gplib_ppu_resolution_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set resolution of display device (TFT or TV)
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:
*													#define	C_TFT_RESOLUTION_320X240		0
*													#define	C_TFT_RESOLUTION_640X480		1
*													#define	C_TFT_RESOLUTION_480X234		2
*													#define	C_TFT_RESOLUTION_480X272		3
*													#define	C_TFT_RESOLUTION_720X480		4
*													#define	C_TFT_RESOLUTION_800X480		5
*													#define	C_TFT_RESOLUTION_800X600		6
*													#define	C_TFT_RESOLUTION_1024X768		7
*													#define	C_TV_RESOLUTION_320X240			0
*													#define	C_TV_RESOLUTION_640X480			0
*													#define	C_TV_RESOLUTION_720X480			4
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_resolution_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:see the constants defined below
#define	C_TFT_RESOLUTION_320X240		0
#define	C_TFT_RESOLUTION_640X480		1
#define	C_TFT_RESOLUTION_480X234		2
#define	C_TFT_RESOLUTION_480X272		3
#define	C_TFT_RESOLUTION_720X480		4
#define	C_TFT_RESOLUTION_800X480		5
#define	C_TFT_RESOLUTION_800X600		6
#define	C_TFT_RESOLUTION_1024X768		7
#define	C_TV_RESOLUTION_320X240			0
#define	C_TV_RESOLUTION_640X480			0
#define	C_TV_RESOLUTION_720X480			4

/*
* Function Name :  gplib_ppu_yuv_type_set
*
* Syntax : INT32S gplib_ppu_yuv_type_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set data format when color is 16-bit RGB or YUV
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										value:		[1:0]: 0=BGRG/VYUY 1=GBGR/YVYU 2=RGBG/UYVY 3=GRGB/YUYV
*															[2]: 0=UV is unsigned(YCbCr) 1=UV is signed(YUV)
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_yuv_type_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// value[1:0]:0=BGRG/VYUY 1=GBGR/YVYU 2=RGBG/UYVY 3=GRGB/YUYV, value[2]:0=UV is unsigned(YCbCr) 1=UV is signed(YUV)

/*
* Function Name :  gplib_ppu_tft_long_burst_set
*
* Syntax : INT32S gplib_ppu_tft_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/dislabe TFT long burst mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable TFT long burst 1=enable TFT long burst
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_tft_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value);	// value:0=disable TFT long burst 1=enable TFT long burst

/*
* Function Name :  gplib_ppu_long_burst_set
*
* Syntax : INT32S gplib_ppu_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable PPU long burst mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable PPU long burst 1=enable PPU long burst
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:0=disable PPU long burst 1=enable PPU long burst

/*
* Function Name :  gplib_ppu_blend4_set
*
* Syntax : INT32S gplib_ppu_blend4_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set the blending level when 4-level blending is used
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0~3
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_blend4_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// value:0~3

/*
* Function Name :  gplib_ppu_rgb565_transparent_color_set
*
* Syntax : INT32S gplib_ppu_rgb565_transparent_color_set(PPU_REGISTER_SETS *p_register_set, INT32U enable, INT32U value);
*
* Purpose :  User can call this function to set transparent color when RGB565 color mode is used
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										enable:0=disable 1=enable
*									  value:0~0xFFFF
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_rgb565_transparent_color_set(PPU_REGISTER_SETS *p_register_set, INT32U enable, INT32U value);	// enable:0=disable 1=enable, value:0~0xFFFF

/*
* Function Name :  gplib_ppu_fade_effect_set
*
* Syntax : INT32S gplib_ppu_fade_effect_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set fade effect
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0~255
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_fade_effect_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:0~255

/*
* Function Name :  gplib_ppu_window_set
*
* Syntax : INT32S gplib_ppu_window_set(PPU_REGISTER_SETS *p_register_set, INT32U window_index, INT32U window_x, INT32U window_y);
*
* Purpose :  User can call this function to set display region of window
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										windows_index:0(window 1)~3(window 4)
*									  window_x:mask + start_x + end_x
*										window_y:start_y + end_y
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_window_set(PPU_REGISTER_SETS *p_register_set, INT32U window_index, INT32U window_x, INT32U window_y);	// windows_index:0(window 1)~3(window 4), window_x:mask + start_x + end_x, window_y:start_y + end_y

/*
* Function Name :  gplib_ppu_palette_type_set
*
* Syntax : INT32S gplib_ppu_palette_type_set(PPU_REGISTER_SETS *p_register_set, INT32U p1024, INT32U type);
*
* Purpose :  User can call this function to determine whether the same palette ram is used on TEXT and SPRITE character
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										p1024:0~1
*									  type:0~3
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_palette_type_set(PPU_REGISTER_SETS *p_register_set, INT32U p1024, INT32U type);		// p1024:0~1, type:0~3

/*
* Function Name :  gplib_ppu_palette_ram_ptr_set
*
* Syntax : INT32S gplib_ppu_palette_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U bank, INT32U value);
*
* Purpose :  User can call this function to specify address of new palette ram data
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										bank:0(palette0)~3(palette3)
*									  value: 32-bit address of palette ram buffer
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_palette_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U bank, INT32U value);	// bank:0(palette0)~3(palette3), value: 32-bit address of palette ram buffer

/*
* Function Name :  gplib_ppu_frame_buffer_add
*
* Syntax : INT32S gplib_ppu_frame_buffer_add(PPU_REGISTER_SETS *p_register_set, INT32U buffer);
*
* Purpose :  User can call this function to add a new frame buffer for PPU driver usage
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										buffer: 32-bit address of frame buffer
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_frame_buffer_add(PPU_REGISTER_SETS *p_register_set, INT32U buffer);		// buffer: 32-bit address of frame buffer

/*
* Function Name :  gplib_ppu_deflicker_para_set
*
* Syntax : INT32S gplib_ppu_deflicker_para_set(PPU_REGISTER_SETS *p_register_set, INT8U para1, INT8U para2, INT8U para3);
*
* Purpose :  User can call this function to enable/disable deflicker mode
*
* Parameters : <IN> value:0=disable deflicker function 1=enable deflicker function after PPU is done
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_deflicker_para_set(PPU_REGISTER_SETS *p_register_set, INT8U para1, INT8U para2, INT8U para3);

/*
* Function Name :  gplib_ppu_go_without_wait
*
* Syntax : INT32S gplib_ppu_go_without_wait(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to apply new settings to PPU engine and start PPU,This function will return immediately
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note : This function returns immediately if frame buffer is not available or PPU is still busy
*
*/
extern INT32S gplib_ppu_go_without_wait(PPU_REGISTER_SETS *p_register_set);		// This function returns immediately if frame buffer is not available or PPU is still busy

/*
* Function Name :  gplib_ppu_go
*
* Syntax : INT32S gplib_ppu_go(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to apply new settings to PPU engine and start PPU
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note : This function returns when PPU registers are updated, it will not wait for PPU frame buffer output to complete
*				 It is suggested to call this function when line base mode is used.
*				 If user can make sure that image data won¡¯t be modified after this function is called, it can also be used in frame base mode.
*
*/
extern INT32S gplib_ppu_go(PPU_REGISTER_SETS *p_register_set);					// This function returns when PPU registers are updated, it will not wait for PPU frame buffer output to complete

/*
* Function Name :  gplib_ppu_go_and_wait_done
*
* Syntax : INT32S gplib_ppu_go_and_wait_done(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to apply new settings to PPU engine and start PPU operation.
*					   This function will return when operation is done
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note : This function returns when PPU registers are updated and operation is done.
*				 It is suggested to call this function when frame base mode is used.
*				 After this function returns, user can modify or free image data safely.
*
*/
extern INT32S gplib_ppu_go_and_wait_done(PPU_REGISTER_SETS *p_register_set);	// This function returns when PPU registers are updated and operation is done

/*
* Function Name :  gplib_ppu_dual_blend_set
*
* Syntax : INT32S gplib_ppu_dual_blend_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable dual blend mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_dual_blend_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_deflicker_set
*
* Syntax : INT32S gplib_ppu_deflicker_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable ppu deflicker mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_deflicker_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_text_rgba_set
*
* Syntax : INT32S gplib_ppu_text_rgba_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable text rgba mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_text_rgba_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_text_alpha_set
*
* Syntax : INT32S gplib_ppu_text_alpha_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable text alpha blending mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_text_alpha_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_rgba_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_rgba_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable sprite alpha blending mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_rgba_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_rgba_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_rgba_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable and set PPU free mode
*
* Parameters : <IN> INTL:		0 => Non-interlace free mode. (For TFT type application)
*														1 => Interlace free mode. (For TV type application)
*										H_size:	PPU output resolution Horizontal length(This value must be a multiply of 16)
*										V_size: PPU output resolution Vertical length
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_free_mode_size_rigister_set(INT16U INTL,INT16U H_size,INT16U V_size);

/*
* Function Name :  gplib_ppu_text_new_specialbmp_set
*
* Syntax : INT32S gplib_ppu_text_new_specialbmp_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable text new special bitmap mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_text_new_specialbmp_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_text_new_compression_set
*
* Syntax : INT32S gplib_ppu_text_new_compression_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable text new compression mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_text_new_compression_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_rgba_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_rgba_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable and set PPU free mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										INTL:		0 => Non-interlace free mode. (For TFT type application)
*														1 => Interlace free mode. (For TV type application)
*										H_size:	PPU output resolution Horizontal length(This value must be a multiply of 16)
*										V_size: PPU output resolution Vertical length
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_free_size_set(PPU_REGISTER_SETS *p_register_set, INT16U INTL,INT16U H_size,INT16U V_size);

/*
* Function Name :  gplib_ppu_text_bitmap_flip_set
*
* Syntax : INT32S gplib_ppu_text_bitmap_flip_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set text flip  mode when text is bitmap mode
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_text_bitmap_flip_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_go_hblank_first
*
* Syntax : INT32S gplib_ppu_go_hblank_first(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to start ppu engine with Horizontal blank function
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_go_hblank_first(PPU_REGISTER_SETS *p_register_set);

/*
* Function Name :  gplib_ppu_hblnk_set
*
* Syntax : INT32S gplib_ppu_hblnk_set(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to Enable/disable Horizontal blank mode
*
* Parameters : <IN> value: 1:Enable 0:Disable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_hblnk_set(INT32U value);

/*
* Function Name :  gplib_ppu_hblnk_line_offset_set
*
* Syntax : INT32S gplib_ppu_hblnk_line_offset_set(INT32U line_offset);
*
* Purpose :  User can call this function to set Horizontal blank stop line
*
* Parameters : <IN>  line_offset: Horizontal blank stop line
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_hblnk_line_offset_set(INT32U line_offset);

/*
* Function Name :  gplib_ppu_hblnk_irq_wait
*
* Syntax : INT32S gplib_ppu_hblnk_irq_wait(void);
*
* Purpose :  User can call this function to Wait Horizontal blank irq
*
* Parameters : <IN>  none
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_hblnk_irq_wait(void);

/*
* Function Name :  gplib_ppu_hblnk_go_now
*
* Syntax : INT32S gplib_ppu_hblnk_go_now(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to start Horizontal blank and ppu go
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_hblnk_go_now(PPU_REGISTER_SETS *p_register_set);

/*
* Function Name :  gplib_ppu_tv_upscaling_set
*
* Syntax : INT32S gplib_ppu_tv_upscaling_set(PPU_REGISTER_SETS *p_register_set, INT32U value, INT32U set_value);
*
* Purpose :  User can call this function to set TV upscaling mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:	0~3
*										 set_value:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_tv_upscaling_set(PPU_REGISTER_SETS *p_register_set, INT32U value, INT32U set_value);

/*
* Function Name :  gplib_ppu_tv_long_burst_set
*
* Syntax : INT32S gplib_ppu_tv_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set TV burst mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: TV frame buffer use 8x32 burst
*														1: TV frame buffer use 64x32 burst
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_tv_long_burst_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_tft_vga2qvga_set
*
* Syntax : INT32S gplib_ppu_tft_vga2qvga_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to change tft display region from VGA to QVGA
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: 	TFT's frame buffer and display size is the same.
*														1:  TFT's frame buffer must be set to 1(PPU_SIZE is 1)and the display size is QVGA
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_tft_vga2qvga_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_go_delay_set
*
* Syntax : INT32S gplib_ppu_go_delay_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable ppu go delay mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: The PPU_GO will not be blocked by and condition
*														1: THE PPU_GO will be delayed until both TVFBI_UPD and TFTTBI_UPD is 1.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_go_delay_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_ui_enable_set
*
* Syntax : INT32S gplib_ppu_ui_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable ppu display ui mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: disable
*														1: enable.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_ui_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_ui_disp_selection_set
*
* Syntax : INT32S gplib_ppu_ui_disp_selection_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to selection display device
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: TFT
*														1: HDMI.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_ui_disp_selection_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_ui_color_set
*
* Syntax : INT32S gplib_ppu_ui_color_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to selection ppu display ui color mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: RGB1555
*														1: 4 palette.
*														2: 16 palette
*														3: 256 palette.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_ui_color_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_ui_blend_level_set
*
* Syntax : INT32S gplib_ppu_ui_blend_level_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set ppu blend level
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0 ~ 15
*														blend level.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_ui_blend_level_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_ui_alpha_enable_set
*
* Syntax : INT32S gplib_ppu_ui_alpha_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable ppu ui alpha mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: disable
*														1: enable.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_ui_alpha_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_ui_addr_set
*
* Syntax : INT32S gplib_ppu_ui_addr_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set ppu ui data address
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														data address
*
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_ui_addr_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_post_process_enable_set
*
* Syntax : INT32S gplib_ppu_post_process_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable ppu process mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: disable
*														1: enable.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_post_process_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_fb_lock_process_enable_set
*
* Syntax : INT32S gplib_ppu_fb_lock_process_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable ppu fb lock mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 FB_LOCK_STRUCT:
*														fb lock
*														setting.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_fb_lock_process_enable_set(PPU_REGISTER_SETS *p_register_set, FB_LOCK_STRUCT *fb_ptr);

/*
* Function Name :  gplib_ppu_sprite_addrx2_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_addrx2_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable ppu sprite position*2 mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*										 value:
*														0: disable
*														1: enable.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_addrx2_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

#endif 		// __GPLIB_PPU_H__
