#ifndef __GPLIB_PPU_SPRITE_H__
#define __GPLIB_PPU_SPRITE_H__


#include "drv_l1_ppu.h"

#define EXSP_STRUCTURE_ALLCMD_EN        1
#define SP_STRUCTURE_CMDDIS_EN          1
#define EXSP_STRUCTURE_SPEN_EN          1
#define EXSP_STRUCTURE_SPGROUP_EN       1
#define SP_STRUCTURE_FRACTION_EN        0
#define SP_SHARED_SETTING_EN            1

#define C_PPU_SPRITE_NUMBER				1024

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
#define	PPU_ENABLE						(((INT32U) 1)<<B_PPU_EN)


// Sprite Control Register Constant Definitions
#define	B_SP_NEW_BLEND_EN				28
#define	SP_NEW_BLEND_ENABLE				(((INT32U) 1)<<B_SP_NEW_BLEND_EN)
#define	B_SP_PAL4096_EN					27
#define	SP_PAL4096_ENABLE				(((INT32U) 1)<<B_SP_PAL4096_EN)
#define	B_SP_PALF_EN					26
#define	SP_PALF_ENABLE					(((INT32U) 1)<<B_SP_PALF_EN)
#define	B_SP_FRAC_EN					22
#define	SP_FRACTION_COORDINATE_ENABLE	(((INT32U) 1)<<B_SP_FRAC_EN)
#define	B_SP_GPR_EN					    21
#define	SP_GROUP_ENABLE			        (((INT32U) 1)<<B_SP_GPR_EN)
#define	B_SP_LS_EN					    20
#define	SP_LARGE_SIZE_ENABLE			(((INT32U) 1)<<B_SP_LS_EN)
#define	B_SP_INTP_EN					19
#define	SP_INTERPOLATION_ENABLE			(((INT32U) 1)<<B_SP_INTP_EN)
#define	B_SP_FAR_EN						18
#define	SP_FAR_ADDRESS_ENABLE			(((INT32U) 1)<<B_SP_FAR_EN)
#define	B_SP_CDM_EN						17
#define	SP_COLOR_DITHER_ENABLE			(((INT32U) 1)<<B_SP_CDM_EN)
#define	B_SP_EFFECT_EN					16
#define	SP_SPECIAL_EFFECT_ENABLE		(((INT32U) 1)<<B_SP_EFFECT_EN)
#define	B_SP_NUMBER						8
#define	MASK_SP_NUMBER					(((INT32U) 0xFF)<<B_SP_NUMBER)
#define	B_SP_ZOOMEN						7
#define	SP_ZOOM_ENABLE					(((INT32U) 1)<<B_SP_ZOOMEN)
#define	B_SP_ROTEN						6
#define	SP_ROTATE_ENABLE				(((INT32U) 1)<<B_SP_ROTEN)
#define	B_SP_MOSEN						5
#define	SP_MOSAIC_ENABLE				(((INT32U) 1)<<B_SP_MOSEN)
#define	B_SP_DIRECT						4
#define	SP_DIRECT_ADDRESS_MODE			(((INT32U) 1)<<B_SP_DIRECT)
#define	B_SP_BLDMODE					2
#define	SP_BLDMODE64_ENABLE				(((INT32U) 1)<<B_SP_BLDMODE)
#define	B_COORD_SEL						1
#define	SP_COORD1						(((INT32U) 1)<<B_COORD_SEL)
#define	SP_COORD0						(((INT32U) 0)<<B_COORD_SEL)
#define	B_SP_EN							0
#define	SP_ENABLE						(((INT32U) 1)<<B_SP_EN)
#define	SP_DISABLE						(((INT32U) 0)<<B_SP_EN)

// Extended Sprite Control Register Constant Definitions
#define	B_EXSP_NEW_BLD_EN			    18
#define	EXSP_NEW_BLD_ENABLE	            (((INT32U) 1)<<B_EXSP_NEW_BLD_EN)
#define	B_EXSP_NEW_PALF4096_EN			17
#define	EXSP_NEW_PALF4096_ENABLE	    (((INT32U) 1)<<B_EXSP_NEW_PALF4096_EN)
#define	B_EXSP_NEW_PALF_EN			    16
#define	EXSP_NEW_PALF_ENABLE	        (((INT32U) 1)<<B_EXSP_NEW_PALF_EN)
#define	B_EXSP_CMASK_EN					6
#define	EXSP_CMASK_ENABLE	            (((INT32U) 1)<<B_EXSP_CMASK_EN)
#define	B_EXSP_FRAC_EN					5
#define	EXSP_FRACTION_COORDINATE_ENABLE	(((INT32U) 1)<<B_EXSP_FRAC_EN)
#define	B_EXSP_GPR_EN					4
#define	EXSP_GROUP_ENABLE			    (((INT32U) 1)<<B_EXSP_GPR_EN)
#define	B_EXSP_LS_EN					3
#define	EXSP_LARGE_SIZE_ENABLE			(((INT32U) 1)<<B_EXSP_LS_EN)
#define	B_EXSP_INTP_EN					2
#define	EXSP_INTERPOLATION_ENABLE	    (((INT32U) 1)<<B_EXSP_INTP_EN)
#define	B_EXSP_CDM_EN					1
#define	EXSP_CDM_ENABLE	                (((INT32U) 1)<<B_EXSP_CDM_EN)

// Sprite RAM Constant Definitions
#define	B_SPN_TYPE                      11
#define MASK_SPN_TYPE                	(0x7 << B_SPN_TYPE)

#define	B_SPN_PALF                 		10
#define MASK_SPN_PALF                	(0x1 << B_SPN_PALF)
#define	B_SPN_PB                        8
#define MASK_SPN_PB               		(0x3 << B_SPN_PB)

#define	B_ESPN_INTERPOLATION            11
#define MASK_ESPN_INTERPOLATION         (0x1 << B_ESPN_INTERPOLATION)
#define	B_ESPN_LG_SIZE                  14
#define MASK_ESPN_LG_SIZE               (0x1 << B_ESPN_LG_SIZE)
#define	B_ESPN_GROUP_ID                 12
#define MASK_ESPN_GROUP_ID              (0x3 << B_ESPN_GROUP_ID)
#define	B_ESPN_CMASK_SIZE               8
#define MASK_ESPN_CMASK_SIZE            (0x7 << B_ESPN_CMASK_SIZE)

#define	B_SPN_CMASK_SIZE                4
#define MASK_SPN_CMASK_SIZE             (0x7 << B_SPN_CMASK_SIZE)
#define	B_SPN_INTERPOLATION             3
#define MASK_SPN_INTERPOLATION          (0x1 << B_SPN_INTERPOLATION)
#define	B_SPN_LG_SIZE                   2
#define MASK_SPN_LG_SIZE                (0x1 << B_SPN_LG_SIZE)
#define	B_SPN_GROUP_ID                  0
#define MASK_SPN_GROUP_ID               (0x3 << B_SPN_GROUP_ID)

#define	B_ESPN_FRAC_1                   8
#define MASK_ESPN_FRAC_1                (0x3 << B_ESPN_FRAC_1)
#define	B_ESPN_FRAC_2                   10
#define MASK_ESPN_FRAC_2                (0x3 << B_ESPN_FRAC_2)
#define	B_ESPN_FRAC_3                   12
#define MASK_ESPN_FRAC_3                (0x3 << B_ESPN_FRAC_3)
#define	B_ESPN_FRAC_4                   14
#define MASK_ESPN_FRAC_4                (0x3 << B_ESPN_FRAC_4)

#define	B_ESPN_FRAC_X0                  0
#define MASK_ESPN_FRAC_X0               (0x3 << B_ESPN_FRAC_X0)
#define	B_ESPN_FRAC_Y0                  2
#define MASK_ESPN_FRAC_Y0               (0x3 << B_ESPN_FRAC_Y0)
#define	B_ESPN_FRAC_X1                  4
#define MASK_ESPN_FRAC_X1               (0x3 << B_ESPN_FRAC_X1)
#define	B_ESPN_FRAC_Y1                  6
#define MASK_ESPN_FRAC_Y1               (0x3 << B_ESPN_FRAC_Y1)
#define	B_ESPN_FRAC_X2                  8
#define MASK_ESPN_FRAC_X2               (0x3 << B_ESPN_FRAC_X2)
#define	B_ESPN_FRAC_Y2                  10
#define MASK_ESPN_FRAC_Y2               (0x3 << B_ESPN_FRAC_Y2)
#define	B_ESPN_FRAC_X3                  12
#define MASK_ESPN_FRAC_X3               (0x3 << B_ESPN_FRAC_X3)
#define	B_ESPN_FRAC_Y3                  14
#define MASK_ESPN_FRAC_Y3               (0x3 << B_ESPN_FRAC_Y3)

// Sprite RAM Constant Definitions
#define	B_SPN_CHARNUM_LO				0
#define	MASK_SPN_CHARNUM_LO				0xFFFF

#define	B_SPN_CDM                       16
#define MASK_CDM_MODE                   0xFFFF
#define	B_SPN_CDM_8                     8
#define MASK_CDM_8_MODE                 0xFF
#define	B_SPN_POSX						0
#define	MASK_SPN_POSX					0x3FF
#define	B_SPN_ROTATE					10
#define	MASK_SPN_ROTATE					(0x3F << 10)
#define	B_SPN_Y1_LO						10
#define	MASK_SPN_Y1_LO					(0x3F << 10)
#define	B_SPN_POSY						0
#define	MASK_SPN_POSY					0x3FF
#define	B_SPN_ZOOM						10
#define	MASK_SPN_ZOOM					(0x3F << 10)
#define	B_SPN_Y2_LO						10
#define	MASK_SPN_Y2_LO					(0x3F << 10)

#define	B_SPN_COLOR						0
#define	MASK_SPN_COLOR					(0x3 << B_SPN_COLOR)
#define	B_SPN_FLIP						2
#define	MASK_SPN_FLIP					(0x3 << B_SPN_FLIP)
#define	B_SPN_HS						4
#define	MASK_SPN_HS						(0x3 << B_SPN_HS)
#define	B_SPN_VS						6
#define	MASK_SPN_VS						(0x3 << B_SPN_VS)
#define	B_SPN_PALETTE					8
#define	MASK_SPN_PALETTE				(0xF << B_SPN_PALETTE)
#define	B_SPN_DEPTH						12
#define	MASK_SPN_DEPTH					(0x3 << B_SPN_DEPTH)
#define	B_SPN_BLD						14
#define	SPN_BLD_ENABLE					(0x1 << B_SPN_BLD)
#define	B_SPN_PB_HIGH					15
#define	MASK_SPN_PB_HIGH				(0x1 << B_SPN_PB_HIGH)

#define P1024_PAL_PB                    4
#define B_PAL1024_BANK                  (0x1 << P1024_PAL_PB)
#define	B_SPN_CHARNUM_HI				16
#define	MASK_SPN_CHARNUM_HI				0x7F
#define	MASK_SPN_CHARNUM_PB				0xFF
#define	B_SPN_PB_LOW					7
#define	MASK_SPN_PB_LOW					(0x1 << B_SPN_PB_LOW)
#define	B_SPN_WIN						8
#define	MASK_SPN_WIN					(0x3 << B_SPN_WIN)
#define	B_SPN_BLD_64_LVL				8
#define	MASK_SPN_BLD_64_LVL				(0x3F << B_SPN_BLD_64_LVL)
#define	B_SPN_BLD_16_LVL				10
#define	MASK_SPN_BLD_16_LVL				(0xF << B_SPN_BLD_16_LVL)
#define	B_SPN_MOSAIC					14
#define	MASK_SPN_MOSAIC					(0x3 << B_SPN_MOSAIC)

#define	B_SPN_X1						0
#define	MASK_SPN_X1						MASK_10_BITS
#define	B_SPN_Y3_LO						10
#define	MASK_SPN_Y3_LO					MASK_6_BITS

#define	B_SPN_X2						0
#define	MASK_SPN_X2						MASK_10_BITS
#define	B_SPN_Y1_HI						10
#define	MASK_SPN_Y1_HI					MASK_4_BITS
#define	B_SPN_Y3_HI1					14
#define	MASK_SPN_Y3_HI1					MASK_2_BITS

#define	B_SPN_X3						0
#define	MASK_SPN_X3						MASK_10_BITS
#define	B_SPN_Y2_HI						10
#define	MASK_SPN_Y2_HI					MASK_4_BITS
#define	B_SPN_Y3_HI2					14
#define	MASK_SPN_Y3_HI2					MASK_2_BITS

// PPU sprite-relative APIs: global control functions
/*
* Function Name :  gplib_ppu_sprite_init
*
* Syntax : INT32S gplib_ppu_sprite_init(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to initiate SPRITE software structure
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_init(PPU_REGISTER_SETS *p_register_set);

/*
* Function Name :  gplib_ppu_sprite_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable sprite function
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value : 0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);				// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_sprite_coordinate_set
*
* Syntax : INT32S gplib_ppu_sprite_coordinate_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set coordinate mode of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=center coordinate 1=top-left coordinate
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_coordinate_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// value:0=center coordinate 1=top-left coordinate

/*
* Function Name :  gplib_ppu_sprite_blend_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_blend_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set blending mode of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=4 level blending mode 1=16 or 64 level blending mode
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_blend_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// value:0=4 level blending mode 1=16 or 64 level blending mode

/*
* Function Name :  gplib_ppu_sprite_direct_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_direct_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set address mode of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=relative address mode 1=direct address mode
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_direct_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:0=relative address mode 1=direct address mode

/*
* Function Name :  gplib_ppu_sprite_zoom_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_zoom_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable zoom function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_zoom_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_sprite_rotate_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_rotate_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable rotate function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_rotate_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_sprite_mosaic_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_mosaic_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable mosaic function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_mosaic_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_sprite_number_set
*
* Syntax : INT32S gplib_ppu_sprite_number_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set valid number of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0(1024 sprites) 1(4 sprites) 2(8 sprites) ... 255(1020 sprites)
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	If gplib_ppu_sprite_image_charnum_calculate() is used, sprite number will be set automatically.
*
*/
extern INT32S gplib_ppu_sprite_number_set(PPU_REGISTER_SETS *p_register_set, INT32U value);				// value:0(1024 sprites) 1(4 sprites) 2(8 sprites) ... 255(1020 sprites)

/*
* Function Name :  gplib_ppu_sprite_special_effect_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_special_effect_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable special effect mode of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	When sprite special effect is enabled, sprite flip mode is used to select special effect:
*						0: None		1: Negative color effect		2: Gray Scale Effect		3: Mono Color Effect
*
*/
extern INT32S gplib_ppu_sprite_special_effect_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);	// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_sprite_color_dither_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_color_dither_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable color dither mode of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	When color dither mode is enabled, the maximum number of valid sprite is 512. And sprite 2.5 mode is force enabled.
*
*/
extern INT32S gplib_ppu_sprite_color_dither_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);	// value:0=disable 1=enable

/*
* Function Name :  gplib_ppu_sprite_25d_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_25d_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable 2.5D mode of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=sprite 2D mode 1= sprite 2.5D mode
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	Sprite 2.5D mode is not reset when gplib_ppu_sprite_init() is called, it is initiated when gplib_ppu_init() is called.
*
*/
extern INT32S gplib_ppu_sprite_25d_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value);	// value:0=sprite 2D mode 1= sprite 2.5D mode

/*
* Function Name :  gplib_ppu_sprite_window_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_window_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable window function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable sprite window function 1=enable sprite window function
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	Blend level down grade to 16-level when sprite window function is enabled.
*					Sprite window enable mode is not reset when gplib_ppu_sprite_init() is called, it is initiated when gplib_ppu_init() is called.
*
*/
extern INT32S gplib_ppu_sprite_window_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);		// value:0=disable sprite window function 1=enable sprite window function

/*
* Function Name :  gplib_ppu_sprite_segment_set
*
* Syntax : INT32S gplib_ppu_sprite_segment_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set segment address of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 32-bit segment address
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_segment_set(PPU_REGISTER_SETS *p_register_set, INT32U value);			// value: 32-bit segment address

/*
* Function Name :  gplib_ppu_sprite_attribute_ram_ptr_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set address of new attribute ram of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value: 32-bit pointer to sprite attribute ram
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value);	// value: 32-bit pointer to sprite attribute ram

/*
* Function Name :  gplib_ppu_sprite_extend_attribute_ram_ptr_set
*
* Syntax : INT32S gplib_ppu_sprite_extend_attribute_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set address of extend attribute ram of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:sprite date in sdram address.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_extend_attribute_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value); // value:sprite date in sdram address.

/*
* Function Name :  gplib_ppu_sprite_sfr_set
*
* Syntax : INT32S gplib_ppu_sprite_sfr_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable far address function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_sfr_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_interpolation_set
*
* Syntax : INT32S gplib_ppu_sprite_interpolation_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable Bi-linear interpolation mode of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_interpolation_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_group_set
*
* Syntax : INT32S gplib_ppu_sprite_group_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable group function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_group_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_large_size_set
*
* Syntax : INT32S gplib_ppu_sprite_large_size_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable large function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_large_size_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_fraction_set
*
* Syntax : INT32S gplib_ppu_sprite_fraction_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable fraction function of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_fraction_set(PPU_REGISTER_SETS *p_register_set, INT32U value);


/*
* Function Name :  gplib_ppu_sprite_pal4096_set
*
* Syntax : INT32S gplib_ppu_sprite_pal4096_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable new pal4096 of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_pal4096_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_new_blend_mode_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_new_blend_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable new blend of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_new_blend_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_new_palf_mode_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_new_palf_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable new palf of SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_new_palf_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);


// PPU sprite-relative APIs: sprite attribute management functions
/*
* Function Name :  gplib_ppu_sprite_attribute_2d_position_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_2d_position_set(SpN_RAM *sprite_attr, INT16S x0, INT16S y0);
*
* Purpose :  User can call this function to set  2D position of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  x0 and y0: represent sprite top/left or center position. Only 10-bits are valid.
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_2d_position_set(SpN_RAM *sprite_attr, INT16S x0, INT16S y0);	// x0 and y0 represent sprite top/left or center position. Only 10-bits are valid.

/*
* Function Name :  gplib_ppu_sprite_attribute_25d_position_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_25d_position_set(SpN_RAM *sprite_attr, POS_STRUCT_PTR position);
*
* Purpose :  User can call this function to 2.5D position of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  position: defines four coordinates of the sprite
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_25d_position_set(SpN_RAM *sprite_attr, POS_STRUCT_PTR position);// position defines four coordinates of the sprite

/*
* Function Name :  gplib_ppu_sprite_attribute_rotate_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_rotate_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to rotate mode of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0~63
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	This function is valid only when sprite 2.5D mode is disabled.
*
*/
extern INT32S gplib_ppu_sprite_attribute_rotate_set(SpN_RAM *sprite_attr, INT32U value);				// value:0~63

/*
* Function Name :  gplib_ppu_sprite_attribute_zoom_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_zoom_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to zoom mode of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0~63
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_zoom_set(SpN_RAM *sprite_attr, INT32U value);					// value:0~63

/*
* Function Name :  gplib_ppu_sprite_attribute_color_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_color_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to set color mode of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0(2-bit) 1(4-bit) 2(6-bit) 3(8-bit/5-bit/16-bit/RGBG/YUYV/8+6 blending)
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_color_set(SpN_RAM *sprite_attr, INT32U value);					// value:0(2-bit) 1(4-bit) 2(6-bit) 3(8-bit/5-bit/16-bit/RGBG/YUYV/8+6 blending)

/*
* Function Name :  gplib_ppu_sprite_attribute_flip_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_flip_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to set flip mode of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0(No flip) 1(H-flip) 2(V-flip) 3(H+V-flip)
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_flip_set(SpN_RAM *sprite_attr, INT32U value);					// value:0(No flip) 1(H-flip) 2(V-flip) 3(H+V-flip)

/*
* Function Name :  gplib_ppu_sprite_attribute_character_size_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_character_size_set(SpN_RAM *sprite_attr, INT32U hs, INT32U vs);
*
* Purpose :  User can call this function to set size of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  hs and vs:0(8) 1(16) 2(32) 3(64)
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_character_size_set(SpN_RAM *sprite_attr, INT32U hs, INT32U vs);// hs and vs:0(8) 1(16) 2(32) 3(64)

/*
* Function Name :  gplib_ppu_sprite_attribute_palette_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_palette_set(SpN_RAM *sprite_attr, INT32U bank, INT32U palette_idx);
*
* Purpose :  User can call this function to set palette bank and index of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  bank:0~3
*										palette_idx:0~15
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	Sprite character number must be calculated again after this function is called
*
*/
extern INT32S gplib_ppu_sprite_attribute_palette_set(SpN_RAM *sprite_attr, INT32U bank, INT32U palette_idx);// bank:0~3, palette_idx:0~15

/*
* Function Name :  gplib_ppu_sprite_attribute_depth_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_depth_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to set depth mode of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0~3
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_depth_set(SpN_RAM *sprite_attr, INT32U value);					// value:0~3

/*
* Function Name :  gplib_ppu_sprite_attribute_blend64_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_blend64_set(SpN_RAM *sprite_attr, INT32U enable, INT32U value);
*
* Purpose :  User can call this function to set blending value of a SPRITE character when 64-level blending mode is used
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*										enable:0=disable 1=enable
*									  value:0~63
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	Blending is 64-level when sprite window function is disabled.
*					This function should be used to set blending level when sprite window function is disabled.
*
*/
extern INT32S gplib_ppu_sprite_attribute_blend64_set(SpN_RAM *sprite_attr, INT32U enable, INT32U value);// enable:0=disable 1=enable, value:0~63

/*
* Function Name :  gplib_ppu_sprite_attribute_blend16_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_blend16_set(SpN_RAM *sprite_attr, INT32U enable, INT32U value);
*
* Purpose :  User can call this function to set blending value of a SPRITE character when 16-level blending mode is used
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*										enable:0=disable 1=enable
*									  value:0~15
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	Blending level down grade to 16-level when sprite window function is enabled.
*					This function should be used to set blending level when sprite window function is enabled.
*
*/
extern INT32S gplib_ppu_sprite_attribute_blend16_set(SpN_RAM *sprite_attr, INT32U enable, INT32U value);// enable:0=disable 1=enable, value:0~15

/*
* Function Name :  gplib_ppu_sprite_attribute_window_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_window_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to set window index of a SPRITE character when window mode is used
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0~3
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	64-level blending will down grade to 16-level when sprite window function is enabled
*
*/
extern INT32S gplib_ppu_sprite_attribute_window_set(SpN_RAM *sprite_attr, INT32U value);				// value:0~3

/*
* Function Name :  gplib_ppu_sprite_attribute_mosaic_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_mosaic_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to set mosaic mode of a SPRITE character
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0(no effect) 1(2x2 pixels) 2(4x4 pixels) 3(8x8 pixels)
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_mosaic_set(SpN_RAM *sprite_attr, INT32U value);				// value:0(no effect) 1(2x2 pixels) 2(4x4 pixels) 3(8x8 pixels)

/*
* Function Name :  gplib_ppu_sprite_attribute_charnum_calculate
*
* Syntax : INT32S gplib_ppu_sprite_attribute_charnum_calculate(PPU_REGISTER_SETS *p_register_set, SpN_RAM *sprite_attr, INT32U data_ptr);
*
* Purpose :  User can call this function to calculate character number of a SPRITE character
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*										*sprite_attr:	sprite releated parameters
*									  data_ptr:	points to image data
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :	This function should be called after one of the setting is modified:
*					Sprite segment, Direct/relative address mode, Sprite size, Sprite color, Photo size, Palette control
*
*/
extern INT32S gplib_ppu_sprite_attribute_charnum_calculate(PPU_REGISTER_SETS *p_register_set, SpN_RAM *sprite_attr, INT32U data_ptr);	// data_ptr points to photo data of the sprite

/*
* Function Name :  gplib_ppu_sprite_large_size_attribute_set
*
* Syntax : INT32S gplib_ppu_sprite_large_size_attribute_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable large size mode of a SPRITE
*
* Parameters : <IN> *sprite_attr:	external sprite releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_large_size_attribute_set(SpN_EX_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_interpolation_attribute_set
*
* Syntax : INT32S gplib_ppu_sprite_interpolation_attribute_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable Bi-linear interpolation mode of a SPRITE
*
* Parameters : <IN> *sprite_attr:	external sprite releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_interpolation_attribute_set(SpN_EX_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_group_attribute_set
*
* Syntax : INT32S gplib_ppu_sprite_group_attribute_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set group ID of SPRITE
*
* Parameters : <IN> *sprite_attr:	external sprite releated parameters
*									  value:0~3
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_group_attribute_set(SpN_EX_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_cdm_attribute_set
*
* Syntax : gplib_ppu_sprite_cdm_attribute_set(SpN_RAM *sprite_attr, INT32U value ,CDM_STRUCT_PTR in);
*
* Purpose :  User can call this function to set color dither attribute of a SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_cdm_attribute_set(SpN_RAM *sprite_attr, INT32U value ,CDM_STRUCT_PTR in);

/*
* Function Name :  gplib_ppu_sprite_cdm_attribute_enable_set
*
* Syntax : INT32S gplib_ppu_sprite_cdm_attribute_enable_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to enable/disable color dither of a SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0=disable 1=enable
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_cdm_attribute_enable_set(SpN_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_attribute_new_palette_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_new_palette_set(SpN_RAM *sprite_attr, SpN_EX_RAM *sprite_ex_attr, INT32U palf, INT32U palette_bank);
*
* Purpose :  User can call this function to set new palette of a SPRITE
*
* Parameters : <IN> *sprite_attr:	    sprite releated parameters
* Parameters : <IN> *sprite_ex_attr:	sprite releated parameters
*									    palf:0=disable, 1=enable
*									    palette_bank: 0~7 bank
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_new_palette_set(SpN_RAM *sprite_attr, SpN_EX_RAM *sprite_ex_attr, INT32U palf, INT32U palette_bank);

/*
* Function Name :  gplib_ppu_sprite_attribute_new_blend_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_new_blend_mode_set(SpN_EX_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to enable/disable new blend of a SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0=disable 1=enable
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_new_blend_mode_set(SpN_EX_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_sprite_attribute_new_palf_mode_set
*
* Syntax : INT32S gplib_ppu_sprite_attribute_new_palf_mode_set(SpN_EX_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to enable/disable new palf of a SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0=disable 1=enable
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_sprite_attribute_new_palf_mode_set(SpN_EX_RAM *sprite_attr, INT32U value);




// PPU exsprite-relative APIs: exsprite functions
/*
* Function Name :  gplib_ppu_exsprite_interpolation_set
*
* Syntax : INT32S gplib_ppu_exsprite_interpolation_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable Bi-linear interpolation mode of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_interpolation_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_large_size_set
*
* Syntax : INT32S gplib_ppu_exsprite_large_size_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable large size mode of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_large_size_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_group_set
*
* Syntax : INT32S gplib_ppu_exsprite_group_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable group function of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_group_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_fraction_set
*
* Syntax : INT32S gplib_ppu_exsprite_fraction_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable fraction function of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_fraction_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_enable_set
*
* Syntax : INT32S gplib_ppu_exsprite_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable external sprite function
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value : 0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_cdm_enable_set
*
* Syntax : INT32S gplib_ppu_exsprite_cdm_enable_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to enable/disable color dither of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_cdm_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_number_set
*
* Syntax : INT32S gplib_ppu_exsprite_number_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set valid number of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0(1024 sprites) 1(4 sprites) 2(8 sprites) ... 255(1020 sprites)
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_number_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_start_address_set
*
* Syntax : INT32S gplib_ppu_exsprite_start_address_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set start address of external SPRITE in SDRAM
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:start address of external SPRITE
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_start_address_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_new_palf_mode_enable_set
*
* Syntax : INT32S gplib_ppu_exsprite_new_palf_mode_enable_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to enable/disable new palf of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_new_palf_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_new_blend_mode_enable_set
*
* Syntax : INT32S gplib_ppu_exsprite_new_blend_mode_enable_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to enable/disable new blend of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_new_blend_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_pal4096_set
*
* Syntax : INT32S gplib_ppu_exsprite_pal4096_set(SpN_RAM *sprite_attr, INT32U value);
*
* Purpose :  User can call this function to enable/disable new pal4096 of external SPRITE
*
* Parameters : <IN> *p_register_set:	PPU releated parameters
*									  value:0=disable 1=enable
*										in:
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_pal4096_set(PPU_REGISTER_SETS *p_register_set, INT32U value);


// PPU exsprite-relative APIs: exsprite attribute management functions
/*
* Function Name :  gplib_ppu_exsprite_interpolation_attribute_set
*
* Syntax : INT32S gplib_ppu_exsprite_interpolation_attribute_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable Bi-linear interpolation mode of a external SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_interpolation_attribute_set(SpN_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_large_size_attribute_set
*
* Syntax : INT32S gplib_ppu_exsprite_large_size_attribute_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to enable/disable large size mode of a external SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0=disable 1=enable
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_large_size_attribute_set(SpN_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_group_attribute_set
*
* Syntax : INT32S gplib_ppu_exsprite_group_attribute_set(PPU_REGISTER_SETS *p_register_set, INT32U value);
*
* Purpose :  User can call this function to set group ID of external SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:0~3
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_group_attribute_set(SpN_RAM *sprite_attr, INT32U value);

/*
* Function Name :  gplib_ppu_exsprite_fraction_attribute_set
*
* Syntax : INT32S gplib_ppu_exsprite_fraction_attribute_set(SpN_RAM *sprite_attr, POS_STRUCT_PTR_GP32XXX value);
*
* Purpose :  User can call this function to set fraction position of a external SPRITE
*
* Parameters : <IN> *sprite_attr:	sprite releated parameters
*									  value:fraction position
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_exsprite_fraction_attribute_set(SpN_RAM *sprite_attr, POS_STRUCT_PTR_GP32XXX value);

#endif 		// __GPLIB_PPU_SPRITE_H__
