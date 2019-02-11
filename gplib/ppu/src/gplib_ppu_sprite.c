/*
* Description: This library provides PPU APIs to init, control and start PPU hardware Sprite engine.
*
* Author: Tristan Yang
*
* Date: 2008/03/03
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.02
*/

#include "gplib_ppu_sprite.h"

#if (defined GPLIB_PPU_EN) && (GPLIB_PPU_EN == 1)

void sprite_position_offset_get(INT16U sprite_num, INT16S start_point, INT16S end_point, INT16S *offset, INT16U *accumulate);

INT32S gplib_ppu_sprite_init(PPU_REGISTER_SETS *p_register_set)
{
	#define GPM4_LONG_BURST_EN      1

	if (!p_register_set) {
		return -1;
	}

	p_register_set->sprite_control = 0;
	p_register_set->sprite_segment = 0;
	p_register_set->extend_sprite_control = 0x4;
#if GPM4_LONG_BURST_EN == 1
	// long burst use must be set 1 with 0xD0500334 bit[1] or bit[2] or bit[3] or bit[4] or bit[6] or bit[16] or bit[17] or bit[18].
	p_register_set->extend_sprite_addr = 0;
#endif
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->sprite_control |= SP_ENABLE;
	} else {
		p_register_set->sprite_control &= ~SP_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_coordinate_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Top-Left coordinate
		p_register_set->sprite_control |= SP_COORD1;
	} else {			// Center coordinate
		p_register_set->sprite_control &= ~SP_COORD1;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

// Blending enable/disable and 64-level blending value is controlled by sprite attribute ram
INT32S gplib_ppu_sprite_blend_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Use 64 level blending in sprite attribute ram
		p_register_set->sprite_control |= SP_BLDMODE64_ENABLE;
	} else {			// Use a global 4 level blending value in P_Blending
		p_register_set->sprite_control &= ~SP_BLDMODE64_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_direct_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Direct address mode
		p_register_set->sprite_control |= SP_DIRECT_ADDRESS_MODE;
	} else {			// Relative address mode
		p_register_set->sprite_control &= ~SP_DIRECT_ADDRESS_MODE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_zoom_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable zoom function
		p_register_set->sprite_control |= SP_ZOOM_ENABLE;
	} else {			// Disable zoom function
		p_register_set->sprite_control &= ~SP_ZOOM_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_rotate_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable rotate function
		p_register_set->sprite_control |= SP_ROTATE_ENABLE;
	} else {			// Disable rotate function
		p_register_set->sprite_control &= ~SP_ROTATE_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_mosaic_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable mosaic function
		p_register_set->sprite_control |= SP_MOSAIC_ENABLE;
	} else {			// Disable mosaic function
		p_register_set->sprite_control &= ~SP_MOSAIC_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_number_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || value > 256) {
		return -1;
	}

	p_register_set->sprite_control &= ~MASK_SP_NUMBER;
	if(value == 256)
        p_register_set->sprite_control |= (0 << B_SP_NUMBER) & MASK_SP_NUMBER;
	else
        p_register_set->sprite_control |= (value << B_SP_NUMBER) & MASK_SP_NUMBER;

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

// When special effect is enabled, flip mode in sprite attribute will control effect type.
// Flip: 0=No effect, 1=Negative, 2=GrayScale, 3=Mono color
INT32S gplib_ppu_sprite_special_effect_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable special effect function
		p_register_set->sprite_control |= SP_SPECIAL_EFFECT_ENABLE;
	} else {			// Disable special effect function
		p_register_set->sprite_control &= ~SP_SPECIAL_EFFECT_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

// When color dither mode is enabled, the maximum number of valid sprite is 512. And sprite 2.5 mode is force enabled.
INT32S gplib_ppu_sprite_color_dither_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable color dither function
		p_register_set->sprite_control |= SP_COLOR_DITHER_ENABLE;
	} else {			// Disable color dither function
		p_register_set->sprite_control &= ~SP_COLOR_DITHER_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

// Note: This function is initiated only when gplib_ppu_init() is called.
INT32S gplib_ppu_sprite_25d_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= PPU_SPR25D_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~PPU_SPR25D_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

// Blend level down grade to 16-level when sprite window function is enabled.
// Note: This function is initiated only when gplib_ppu_init() is called.
INT32S gplib_ppu_sprite_window_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= SPR_WIN_ENABLE;
	} else {
		p_register_set->ppu_enable &= ~SPR_WIN_ENABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_sprite_segment_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || (value&0x1)) {
		return -1;
	}

	p_register_set->sprite_segment = value;

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	p_register_set->ppu_sprite_attribute_ptr = value;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE_ATTRIBUTE;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_2d_position_set(SpN_RAM *sprite_attr, INT16S x0, INT16S y0)
{
	if (!sprite_attr) {
		return -1;
	}

	sprite_attr->uPosX_16 &= ~MASK_SPN_POSX;
	sprite_attr->uPosX_16 |= (x0 & MASK_SPN_POSX);
	sprite_attr->uPosY_16 &= ~MASK_SPN_POSY;
	sprite_attr->uPosY_16 |= (y0 & MASK_SPN_POSY);

	return 0;
}

INT32S gplib_ppu_sprite_attribute_25d_position_set(SpN_RAM *out, POS_STRUCT_PTR in)
{
	if (!out || !in) {
		return -1;
	}

	PPU_SPRITE_25D_POSITION_CONVERT(in->x0,in->y0,in->x1,in->y1,in->x2,in->y2,in->x3,in->y3,out->uPosX_16,out->uPosY_16,out->uX1_16,out->uX2_16,out->uX3_16);

	return 0;
}

// This function is valid only when sprite 2.5D mode is disabled
INT32S gplib_ppu_sprite_attribute_rotate_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>63) {
		return -1;
	}

	sprite_attr->uPosX_16 &= ~MASK_SPN_ROTATE;
	sprite_attr->uPosX_16 |= (value << B_SPN_ROTATE) & MASK_SPN_ROTATE;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_zoom_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>63) {
		return -1;
	}

	sprite_attr->uPosY_16 &= ~MASK_SPN_ZOOM;
	sprite_attr->uPosY_16 |= (value << B_SPN_ZOOM) & MASK_SPN_ZOOM;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_color_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>3) {
		return -1;
	}

	sprite_attr->attr0 &= ~MASK_SPN_COLOR;
	sprite_attr->attr0 |= (value << B_SPN_COLOR) & MASK_SPN_COLOR;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_flip_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>3) {
		return -1;
	}

	sprite_attr->attr0 &= ~MASK_SPN_FLIP;
	sprite_attr->attr0 |= (value << B_SPN_FLIP) & MASK_SPN_FLIP;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_character_size_set(SpN_RAM *sprite_attr, INT32U hs, INT32U vs)
{
	if (!sprite_attr || hs>3 || vs>3) {
		return -1;
	}

	// Sprite character width
	sprite_attr->attr0 &= ~MASK_SPN_HS;
	sprite_attr->attr0 |= (hs << B_SPN_HS) & MASK_SPN_HS;
	// Sprite character height
	sprite_attr->attr0 &= ~MASK_SPN_VS;
	sprite_attr->attr0 |= (vs << B_SPN_VS) & MASK_SPN_VS;

	return 0;
}

// Character number must be calculated after this function is called
#if SP_SHARED_SETTING_EN == 1
INT32S gplib_ppu_sprite_attribute_character_number_set(PPU_REGISTER_SETS *p_register_set, SpN_RAM *sprite_attr, INT32U spnum)
{
    INT16U mask = MASK_SPN_CHARNUM_LO;

	if (!sprite_attr || !p_register_set) {
		return -1;
	}

	// Sprite character width
	sprite_attr->nCharNumLo_16 &= ~mask;
	sprite_attr->nCharNumLo_16 = (spnum & mask);
	// Sprite character height
	if(p_register_set->ppu_palette_control & B_PAL1024_BANK)
	{
        sprite_attr->attr1 &= ~MASK_SPN_CHARNUM_HI;
        sprite_attr->attr1 |= ((spnum >> B_SPN_CHARNUM_HI) & MASK_SPN_CHARNUM_HI);
	}
	else
	{
        sprite_attr->attr1 &= ~MASK_SPN_CHARNUM_PB;
        sprite_attr->attr1 |= (spnum >> B_SPN_CHARNUM_HI);
	}

	return 0;
}
#else
INT32S gplib_ppu_sprite_attribute_character_number_set(SpN_RAM *sprite_attr, INT32U spnum)
{
    INT16U mask = MASK_SPN_CHARNUM_LO;

	if (!sprite_attr ) {
		return -1;
	}

	// Sprite character width
	sprite_attr->nCharNumLo_16 &= ~mask;
	sprite_attr->nCharNumLo_16 = (spnum & mask);
	// Sprite character height
	sprite_attr->attr1 &= ~MASK_SPN_CHARNUM_HI;
	sprite_attr->attr1 |= (spnum >> B_SPN_CHARNUM_HI);

	return 0;
}
#endif


// Palette_set must be calculated after this function is called
INT32S gplib_ppu_sprite_attribute_palette_set(SpN_RAM *sprite_attr, INT32U bank, INT32U palette_idx)
{
	if (!sprite_attr || bank>3 || palette_idx>0xF) {
		return -1;
	}

	// Palette index
	sprite_attr->attr0 &= ~MASK_SPN_PALETTE;
	sprite_attr->attr0 |= (palette_idx << B_SPN_PALETTE) & MASK_SPN_PALETTE;
	// Palette bank high bit
	sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
	sprite_attr->attr0 |= ((bank>>1) << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
	// Palette bank low bit
	sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
	sprite_attr->attr1 |= (bank << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_depth_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>3) {
		return -1;
	}

	sprite_attr->attr0 &= ~MASK_SPN_DEPTH;
	sprite_attr->attr0 |= (value << B_SPN_DEPTH) & MASK_SPN_DEPTH;

	return 0;
}

// Blending is 64-level when sprite window function is disabled.
// This function should be used to set blending level when sprite window function is disabled.
INT32S gplib_ppu_sprite_attribute_blend64_set(SpN_RAM *sprite_attr, INT32U enable, INT32U value)
{
	if (!sprite_attr || value>0x3F) {
		return -1;
	}

	// Blending enable
	if (enable) {
		sprite_attr->attr0 |= SPN_BLD_ENABLE;
	} else {
		sprite_attr->attr0 &= ~SPN_BLD_ENABLE;
	}
	// Blending level
	sprite_attr->attr1 &= ~MASK_SPN_BLD_64_LVL;
	sprite_attr->attr1 |= (value << B_SPN_BLD_64_LVL) & MASK_SPN_BLD_64_LVL;

	return 0;
}

// Blending level down grade to 16-level when sprite window function is enabled.
// This function should be used to set blending level when sprite window function is enabled.
INT32S gplib_ppu_sprite_attribute_blend16_set(SpN_RAM *sprite_attr, INT32U enable, INT32U value)
{
	if (!sprite_attr || value>0xF) {
		return -1;
	}

	// Blending enable
	if (enable) {
		sprite_attr->attr0 |= SPN_BLD_ENABLE;
	} else {
		sprite_attr->attr0 &= ~SPN_BLD_ENABLE;
	}
	// Blending level
	sprite_attr->attr1 &= ~MASK_SPN_BLD_16_LVL;
	sprite_attr->attr1 |= (value << B_SPN_BLD_16_LVL) & MASK_SPN_BLD_16_LVL;

	return 0;
}

// 64-level blending will down grade to 16-level when sprite window function is enabled.
INT32S gplib_ppu_sprite_attribute_window_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>3) {
		return -1;
	}

	sprite_attr->attr1 &= ~MASK_SPN_WIN;
	sprite_attr->attr1 |= (value << B_SPN_WIN) & MASK_SPN_WIN;

	return 0;
}

INT32S gplib_ppu_sprite_attribute_mosaic_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>3) {
		return -1;
	}

	sprite_attr->attr1 &= ~MASK_SPN_MOSAIC;
	sprite_attr->attr1 |= (value << B_SPN_MOSAIC) & MASK_SPN_MOSAIC;

	return 0;
}

// This function should be called after one of the setting is modified:
// Sprite segment, Direct/relative address mode, Sprite size, Sprite color, Photo size, Palette control
INT32S gplib_ppu_sprite_attribute_charnum_calculate(PPU_REGISTER_SETS *p_register_set, SpN_RAM *sprite_attr, INT32U data_ptr)
{
	INT32U char_num;
	INT16U shift_cnt;

	if (!p_register_set || !sprite_attr || p_register_set->sprite_segment>data_ptr) {
		return -1;
	}

	char_num = data_ptr - p_register_set->sprite_segment;
	if (p_register_set->sprite_control & SP_DIRECT_ADDRESS_MODE) {		// Direct mode
		char_num >>= 1;

		if (p_register_set->ppu_palette_control & (1<<4)) {		// P1024 mode is enabled, 23-bit character number
			if (char_num > 0x7FFFFF) {
				return -1;
			}
			sprite_attr->attr1 &= ~0x7F;
			sprite_attr->attr1 |= (char_num >> 16) & 0x7F;
		} else {			// 24-bit character number
			if (char_num > 0xFFFFFF) {
				return -1;
			}
			sprite_attr->attr1 &= ~0xFF;
			sprite_attr->attr1 |= (char_num >> 16) & 0xFF;
		}
	} else {		// Relative mode
		// Decide shift number by sprite size, color
		shift_cnt = 3 + ((sprite_attr->attr0 & MASK_SPN_HS)>>B_SPN_HS) + 3 +  ((sprite_attr->attr0 & MASK_SPN_VS)>>B_SPN_VS);
		switch ((sprite_attr->attr0 & MASK_SPN_COLOR) >> B_SPN_COLOR) {
		case 0:			// 2-bit color
			shift_cnt -= 2;
			break;
		case 1:			// 4-bit color
			shift_cnt -= 1;
			break;
		case 2:			// 6-bit color
			char_num -= (char_num>>2);
			break;
		case 3:
			if (sprite_attr->attr0 & (1<<B_SPN_PALETTE)) {		// 16-bit color
				shift_cnt += 1;
			}
			// else, 8-bit color
			break;
		}

		char_num >>= shift_cnt;
		if (char_num > 0xFFFF) {
			return -1;
		}
	}
	sprite_attr->nCharNumLo_16 = (INT16U) (char_num & 0xFFFF);

	return 0;
}


INT32S gplib_ppu_sprite_cdm_attribute_set(SpN_RAM *sprite_attr, INT32U value ,CDM_STRUCT_PTR in)
{
	INT16U mask = MASK_CDM_MODE;
	INT32U cmd_value;

	if (!sprite_attr || !in) {
		return -1;
	}

    // CMD H/W setting change
#if SP_STRUCTURE_CMDDIS_EN == 1
    if(value)
        cmd_value = 0;
    else
        cmd_value = 1;
#else
    if(value)
        cmd_value = 1;
    else
        cmd_value = 0;
#endif

    sprite_attr++;
	sprite_attr->nCharNumLo_16 &= ~mask;
	sprite_attr->nCharNumLo_16 = (in->cdm0 & mask);
	sprite_attr->uPosX_16 &= ~mask;
	sprite_attr->uPosX_16 = (cmd_value << (B_SPN_CDM-1)|in->cdm0 >> B_SPN_CDM);
	sprite_attr->uPosY_16 &= ~mask;
	sprite_attr->uPosY_16 = (in->cdm1 & mask);
	sprite_attr->attr0 &= ~mask;
	sprite_attr->attr0 = (in->cdm1 >> B_SPN_CDM);
	sprite_attr->attr1 &= ~mask;
	sprite_attr->attr1 = (in->cdm2 & mask);
	sprite_attr->uX1_16 &= ~mask;
	sprite_attr->uX1_16 = (in->cdm2 >> B_SPN_CDM);
	sprite_attr->uX2_16 &= ~mask;
	sprite_attr->uX2_16 = (in->cdm3 & mask);
	sprite_attr->uX3_16 &= ~mask;
	sprite_attr->uX3_16 = (in->cdm3 >> B_SPN_CDM);

	return 0;
}

INT32S gplib_ppu_exsprite_cdm_attribute_set(SpN_RAM *sprite_attr, INT32U value ,CDM_STRUCT_PTR in)
{
	INT8U  mask_8 = MASK_CDM_8_MODE;
	INT16U mask = MASK_CDM_MODE;
    INT32U cmd_value;

	if (!sprite_attr || !in) {
		return -1;
	}

    // CMD H/W setting change
#if SP_STRUCTURE_CMDDIS_EN == 1
    if(value)
        cmd_value = 0;
    else
        cmd_value = 1;
#else
    if(value)
        cmd_value = 1;
    else
        cmd_value = 0;
#endif

    sprite_attr++;
	// cdm0
	sprite_attr->nCharNumLo_16 &= ~mask;
	sprite_attr->nCharNumLo_16 = (in->cdm0 & mask);
	sprite_attr->uPosX_16 &= ~(1 << (B_SPN_CDM-1));
	sprite_attr->uPosX_16 |= (cmd_value << (B_SPN_CDM-1));
	sprite_attr->uPosX_16 &= ~mask_8;
	sprite_attr->uPosX_16 |= (in->cdm0 >> B_SPN_CDM);
	// cdm1
	sprite_attr->uPosY_16 &= ~mask;
	sprite_attr->uPosY_16 |= (in->cdm1 & mask);
	sprite_attr->attr0 &= ~mask_8;
	sprite_attr->attr0 = (in->cdm1 >> B_SPN_CDM);
	// cdm2
	sprite_attr->attr1 &= ~mask;
	sprite_attr->attr1 = (in->cdm2 & mask);
	sprite_attr->uX1_16 &= ~mask_8;
	sprite_attr->uX1_16 = (in->cdm2 >> B_SPN_CDM);
	// cdm3
	sprite_attr->uX2_16 &= ~mask;
	sprite_attr->uX2_16 = (in->cdm3 & mask);
	sprite_attr->uX3_16 &= ~mask_8;
	sprite_attr->uX3_16 = (in->cdm3 >> B_SPN_CDM);

	return 0;
}

INT32S gplib_ppu_sprite_cdm_attribute_enable_set(SpN_RAM *sprite_attr, INT32U value)
{
	INT32U cmd_value;

	if (!sprite_attr) {
		return -1;
	}

    // CMD H/W setting change
#if SP_STRUCTURE_CMDDIS_EN == 1
    if(value)
        cmd_value = 0;
    else
        cmd_value = 1;
#else
    if(value)
        cmd_value = 1;
    else
        cmd_value = 0;
#endif

    sprite_attr++;
    sprite_attr->uPosX_16 &= ~(1 << (B_SPN_CDM-1));
	sprite_attr->uPosX_16 |= (cmd_value << (B_SPN_CDM-1));

	return 0;
}

INT32S gplib_ppu_exsprite_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->extend_sprite_control |= SP_ENABLE;
        #if EXSP_STRUCTURE_SPEN_EN == 1
            p_register_set->sprite_control |= SP_ENABLE;
        #endif
	} else {
		p_register_set->extend_sprite_control &= ~SP_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_cdm_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->extend_sprite_control |= EXSP_CDM_ENABLE;
	} else {
		p_register_set->extend_sprite_control &= ~EXSP_CDM_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_number_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || value > 256) {
		return -1;
	}

	p_register_set->extend_sprite_control &= ~MASK_SP_NUMBER;
	if(value == 256)
        p_register_set->extend_sprite_control |= (0 << B_SP_NUMBER) & MASK_SP_NUMBER;
	else
        p_register_set->extend_sprite_control |= (value << B_SP_NUMBER) & MASK_SP_NUMBER;

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_start_address_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || (value&0x1)) {
		return -1;
	}

	p_register_set->extend_sprite_addr = value;

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}


#if 1//((MCU_VERSION >= GPL326XX)&&(MCU_VERSION < GPL327XX))

INT32S gplib_ppu_sprite_sfr_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Sfr mode enable
		p_register_set->sprite_control |= SP_FAR_ADDRESS_ENABLE;
	} else {			// Sfr mode disable
		p_register_set->sprite_control &= ~SP_FAR_ADDRESS_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_interpolation_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable zoom function
		p_register_set->sprite_control |= SP_INTERPOLATION_ENABLE;
	} else {			// Disable zoom function
		p_register_set->sprite_control &= ~SP_INTERPOLATION_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_group_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable zoom function
		p_register_set->sprite_control |= SP_GROUP_ENABLE;
	} else {			// Disable zoom function
		p_register_set->sprite_control &= ~SP_GROUP_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_extend_attribute_ram_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	p_register_set->ppu_sprite_ex_attribute_ptr = value;

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE_EX_ATTRIBUTE;

	return 0;
}

INT32S gplib_ppu_exsprite_interpolation_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->extend_sprite_control |= EXSP_INTERPOLATION_ENABLE;
	} else {
		p_register_set->extend_sprite_control &= ~EXSP_INTERPOLATION_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_interpolation_attribute_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>1) {
		return -1;
	}
    sprite_attr++;
#if EXSP_STRUCTURE_ALLCMD_EN == 1
    sprite_attr->uPosX_16 &= ~MASK_ESPN_INTERPOLATION;
    sprite_attr->uPosX_16 |= (value << B_ESPN_INTERPOLATION) & MASK_ESPN_INTERPOLATION;
#else
	if(GPEXSP_CMD_COMPARE())
	{
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_INTERPOLATION;
	   sprite_attr->uPosX_16 |= (value << B_ESPN_INTERPOLATION) & MASK_ESPN_INTERPOLATION;
	}
	else
	{
	   sprite_attr->nCharNumLo_16 &= ~MASK_SPN_INTERPOLATION;
	   sprite_attr->nCharNumLo_16 |= (value << B_SPN_INTERPOLATION) & MASK_SPN_INTERPOLATION;
	}
#endif

	return 0;
}

INT32S gplib_ppu_sprite_interpolation_attribute_set(SpN_EX_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>1) {
		return -1;
	}

	sprite_attr->ex_attr0 &= ~MASK_SPN_INTERPOLATION;
	sprite_attr->ex_attr0 |= (value << B_SPN_INTERPOLATION) & MASK_SPN_INTERPOLATION;

	return 0;
}

INT32S gplib_ppu_exsprite_large_size_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->extend_sprite_control |= EXSP_LARGE_SIZE_ENABLE;
	} else {
		p_register_set->extend_sprite_control &= ~EXSP_LARGE_SIZE_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_large_size_attribute_set(SpN_RAM *sprite_attr, INT32U value)
{

	if (!sprite_attr || value>1) {
		return -1;
	}

    sprite_attr++;
#if EXSP_STRUCTURE_ALLCMD_EN == 1
    sprite_attr->uPosX_16 &= ~MASK_ESPN_LG_SIZE;
    sprite_attr->uPosX_16 |= (value << B_ESPN_LG_SIZE) & MASK_ESPN_LG_SIZE;
#else
	if(GPEXSP_CMD_COMPARE())
	{
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_LG_SIZE;
	   sprite_attr->uPosX_16 |= (value << B_ESPN_LG_SIZE) & MASK_ESPN_LG_SIZE;
	}
	else
	{
	   sprite_attr->nCharNumLo_16 &= ~MASK_SPN_LG_SIZE;
	   sprite_attr->nCharNumLo_16 |= (value << B_SPN_LG_SIZE) & MASK_SPN_LG_SIZE;
	}
#endif

	return 0;
}

INT32S gplib_ppu_sprite_large_size_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable zoom function
		p_register_set->sprite_control |= SP_LARGE_SIZE_ENABLE;
	} else {			// Disable zoom function
		p_register_set->sprite_control &= ~SP_LARGE_SIZE_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_sprite_large_size_attribute_set(SpN_EX_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>1) {
		return -1;
	}

	sprite_attr->ex_attr0 &= ~MASK_SPN_LG_SIZE;
	sprite_attr->ex_attr0 |= (value << B_SPN_LG_SIZE) & MASK_SPN_LG_SIZE;

	return 0;
}

INT32S gplib_ppu_exsprite_group_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->extend_sprite_control |= EXSP_GROUP_ENABLE;
		#if EXSP_STRUCTURE_SPGROUP_EN == 1
            p_register_set->sprite_control |= SP_GROUP_ENABLE;
		#endif
	} else {
		p_register_set->extend_sprite_control &= ~EXSP_GROUP_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_group_attribute_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>3) {
		return -1;
	}

    sprite_attr++;
#if EXSP_STRUCTURE_ALLCMD_EN == 1
    sprite_attr->uPosX_16 &=~ MASK_ESPN_GROUP_ID;
    sprite_attr->uPosX_16 |=(value << B_ESPN_GROUP_ID) & MASK_ESPN_GROUP_ID;
#else
	if(GPEXSP_CMD_COMPARE())
	{
	   sprite_attr->uPosX_16 &=~ MASK_ESPN_GROUP_ID;
	   sprite_attr->uPosX_16 |=(value << B_ESPN_GROUP_ID) & MASK_ESPN_GROUP_ID;
	}
	else
	{
	   sprite_attr->nCharNumLo_16 &= ~MASK_SPN_GROUP_ID;
	   sprite_attr->nCharNumLo_16|=(value << B_SPN_GROUP_ID) & MASK_SPN_GROUP_ID;
	}
#endif
	return 0;
}

INT32S gplib_ppu_sprite_group_attribute_set(SpN_EX_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value>3) {
		return -1;
	}

	sprite_attr->ex_attr0 &= ~MASK_SPN_GROUP_ID;
	sprite_attr->ex_attr0 |= (value << B_SPN_GROUP_ID) & MASK_SPN_GROUP_ID;

	return 0;
}

INT32S gplib_ppu_sprite_fraction_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
#if SP_STRUCTURE_FRACTION_EN == 0
    return -1;
#else
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Enable zoom function
		p_register_set->sprite_control |= SP_FRACTION_COORDINATE_ENABLE;
	} else {			// Disable zoom function
		p_register_set->sprite_control &= ~SP_FRACTION_COORDINATE_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
#endif
}

INT32S gplib_ppu_exsprite_fraction_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
#if SP_STRUCTURE_FRACTION_EN == 0
    return -1;
#else
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->extend_sprite_control |= EXSP_FRACTION_COORDINATE_ENABLE;
	} else {
		p_register_set->extend_sprite_control &= ~EXSP_FRACTION_COORDINATE_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
#endif
}

INT32S gplib_ppu_exsprite_fraction_attribute_set(SpN_RAM *sprite_attr, POS_STRUCT_PTR_GP32XXX value)
{
#if SP_STRUCTURE_FRACTION_EN == 0
    return -1;
#else
	if (!sprite_attr || !value) {
		return -1;
	}

    sprite_attr++;
	if(GPEXSP_CMD_COMPARE())
	{
	   // X0
	   sprite_attr->uX1_16 &= ~MASK_ESPN_FRAC_1;
	   if((value->x0 >= 0)&&(value->x0 < 0.25))
	     sprite_attr->uX1_16 |= (0 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   else if((value->x0 >= 0.25)&&(value->x0 < 0.5))
	     sprite_attr->uX1_16 |= (1 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   else if((value->x0 >= 0.5)&&(value->x0 < 0.75))
	     sprite_attr->uX1_16 |= (2 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   else if((value->x0 >= 0.75)&&(value->x0 < 1))
	     sprite_attr->uX1_16 |= (3 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   // Y0
	   sprite_attr->uX1_16 &= ~MASK_ESPN_FRAC_2;
	   if((value->y0 >= 0)&&(value->y0 < 0.25))
	     sprite_attr->uX1_16 |= (0 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   else if((value->y0 >= 0.25)&&(value->y0 < 0.5))
	     sprite_attr->uX1_16 |= (1 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   else if((value->y0 >= 0.5)&&(value->y0 < 0.75))
	     sprite_attr->uX1_16 |= (2 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   else if((value->y0 >= 0.75)&&(value->y0 < 1))
	     sprite_attr->uX1_16 |= (3 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   // X1
	   sprite_attr->uX1_16 &= ~MASK_ESPN_FRAC_3;
	   if((value->x1 >= 0)&&(value->x1 < 0.25))
	     sprite_attr->uX1_16 |= (0 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   else if((value->x1 >= 0.25)&&(value->x1 < 0.5))
	     sprite_attr->uX1_16 |= (1 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   else if((value->x1 >= 0.5)&&(value->x1 < 0.75))
	     sprite_attr->uX1_16 |= (2 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   else if((value->x1 >= 0.75)&&(value->x1 < 1))
	     sprite_attr->uX1_16 |= (3 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   // Y1
	   sprite_attr->uX1_16 &= ~MASK_ESPN_FRAC_4;
	   if((value->y1 >= 0)&&(value->y1 < 0.25))
	     sprite_attr->attr0 |= (0 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	   else if((value->y1 >= 0.25)&&(value->y1 < 0.5))
	     sprite_attr->attr0 |= (1 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	   else if((value->y1 >= 0.5)&&(value->y1 < 0.75))
	     sprite_attr->attr0 |= (2 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	   else if((value->y1 >= 0.75)&&(value->y1 < 1))
	     sprite_attr->attr0 |= (3 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	   // X2
	   sprite_attr->attr0 &= ~MASK_ESPN_FRAC_1;
	   if((value->x2 >= 0)&&(value->x2 < 0.25))
	     sprite_attr->attr0 |= (0 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   else if((value->x2 >= 0.25)&&(value->x2 < 0.5))
	     sprite_attr->attr0 |= (1 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   else if((value->x2 >= 0.5)&&(value->x2 < 0.75))
	     sprite_attr->attr0 |= (2 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   else if((value->x2 >= 0.75)&&(value->x2 < 1))
	     sprite_attr->attr0 |= (3 << B_ESPN_FRAC_1) & MASK_ESPN_FRAC_1;
	   // Y2
	   sprite_attr->attr0 &= ~MASK_ESPN_FRAC_2;
	   if((value->y2 >= 0)&&(value->y2 < 0.25))
	     sprite_attr->attr0 |= (0 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   else if((value->y2 >= 0.25)&&(value->y2 < 0.5))
	     sprite_attr->attr0 |= (1 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   else if((value->y2 >= 0.5)&&(value->y2 < 0.75))
	     sprite_attr->attr0 |= (2 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   else if((value->y2 >= 0.75)&&(value->y2 < 1))
	     sprite_attr->attr0 |= (3 << B_ESPN_FRAC_2) & MASK_ESPN_FRAC_2;
	   // X3
	   sprite_attr->attr0 &= ~MASK_ESPN_FRAC_3;
	   if((value->x3 >= 0)&&(value->x3 < 0.25))
	     sprite_attr->attr0 |= (0 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   else if((value->x3 >= 0.25)&&(value->x3 < 0.5))
	     sprite_attr->attr0 |= (1 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   else if((value->x3 >= 0.5)&&(value->x3 < 0.75))
	     sprite_attr->attr0 |= (2 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   else if((value->x3 >= 0.75)&&(value->x3 < 1))
	     sprite_attr->attr0 |= (3 << B_ESPN_FRAC_3) & MASK_ESPN_FRAC_3;
	   // Y3
	   sprite_attr->attr0 &= ~MASK_ESPN_FRAC_4;
	   if((value->y3 >= 0)&&(value->y3 < 0.25))
	     sprite_attr->attr0 |= (0 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	   else if((value->y3 >= 0.25)&&(value->y3 < 0.5))
	     sprite_attr->attr0 |= (1 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	   else if((value->y3 >= 0.5)&&(value->y3 < 0.75))
	     sprite_attr->attr0 |= (2 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	   else if((value->y3 >= 0.75)&&(value->y3 < 1))
	     sprite_attr->attr0 |= (3 << B_ESPN_FRAC_4) & MASK_ESPN_FRAC_4;
	}
	else
	{
	   // X0
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_X0;
	   if((value->x0 >= 0)&&(value->x0 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_X0) & MASK_ESPN_FRAC_X0;
	   else if((value->x0 >= 0.25)&&(value->x0 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_X0) & MASK_ESPN_FRAC_X0;
	   else if((value->x0 >= 0.5)&&(value->x0 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_X0) & MASK_ESPN_FRAC_X0;
	   else if((value->x0 >= 0.75)&&(value->x0 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_X0) & MASK_ESPN_FRAC_X0;
	   // Y0
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_Y0;
	   if((value->y0 >= 0)&&(value->y0 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_Y0) & MASK_ESPN_FRAC_Y0;
	   else if((value->y0 >= 0.25)&&(value->y0 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_Y0) & MASK_ESPN_FRAC_Y0;
	   else if((value->y0 >= 0.5)&&(value->y0 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_Y0) & MASK_ESPN_FRAC_Y0;
	   else if((value->y0 >= 0.75)&&(value->y0 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_Y0) & MASK_ESPN_FRAC_Y0;
	   // X1
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_X1;
	   if((value->x1 >= 0)&&(value->x1 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_X1) & MASK_ESPN_FRAC_X1;
	   else if((value->x1 >= 0.25)&&(value->x1 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_X1) & MASK_ESPN_FRAC_X1;
	   else if((value->x1 >= 0.5)&&(value->x1 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_X1) & MASK_ESPN_FRAC_X1;
	   else if((value->x1 >= 0.75)&&(value->x1 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_X1) & MASK_ESPN_FRAC_X1;
	   // Y1
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_Y1;
	   if((value->y1 >= 0)&&(value->y1 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_Y1) & MASK_ESPN_FRAC_Y1;
	   else if((value->y1 >= 0.25)&&(value->y1 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_Y1) & MASK_ESPN_FRAC_Y1;
	   else if((value->y1 >= 0.5)&&(value->y1 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_Y1) & MASK_ESPN_FRAC_Y1;
	   else if((value->y1 >= 0.75)&&(value->y1 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_Y1) & MASK_ESPN_FRAC_Y1;
	   // X2
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_X2;
	   if((value->x2 >= 0)&&(value->x2 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_X2) & MASK_ESPN_FRAC_X2;
	   else if((value->x2 >= 0.25)&&(value->x2 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_X2) & MASK_ESPN_FRAC_X2;
	   else if((value->x2 >= 0.5)&&(value->x2 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_X2) & MASK_ESPN_FRAC_X2;
	   else if((value->x2 >= 0.75)&&(value->x2 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_X2) & MASK_ESPN_FRAC_X2;
	   // Y2
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_Y2;
	   if((value->y2 >= 0)&&(value->y2 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_Y2) & MASK_ESPN_FRAC_Y2;
	   else if((value->y2 >= 0.25)&&(value->y2 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_Y2) & MASK_ESPN_FRAC_Y2;
	   else if((value->y2 >= 0.5)&&(value->y2 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_Y2) & MASK_ESPN_FRAC_Y2;
	   else if((value->y2 >= 0.75)&&(value->y2 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_Y2) & MASK_ESPN_FRAC_Y2;
	   // X3
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_X3;
	   if((value->x3 >= 0)&&(value->x3 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_X3) & MASK_ESPN_FRAC_X3;
	   else if((value->x3 >= 0.25)&&(value->x3 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_X3) & MASK_ESPN_FRAC_X3;
	   else if((value->x3 >= 0.5)&&(value->x3 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_X3) & MASK_ESPN_FRAC_X3;
	   else if((value->x3 >= 0.75)&&(value->x3 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_X3) & MASK_ESPN_FRAC_X3;
	   // Y3
	   sprite_attr->uPosX_16 &= ~MASK_ESPN_FRAC_Y3;
	   if((value->y3 >= 0)&&(value->y3 < 0.25))
	     sprite_attr->uPosX_16 |= (0 << B_ESPN_FRAC_Y3) & MASK_ESPN_FRAC_Y3;
	   else if((value->y3 >= 0.25)&&(value->y3 < 0.5))
	     sprite_attr->uPosX_16 |= (1 << B_ESPN_FRAC_Y3) & MASK_ESPN_FRAC_Y3;
	   else if((value->y3 >= 0.5)&&(value->y3 < 0.75))
	     sprite_attr->uPosX_16 |= (2 << B_ESPN_FRAC_Y3) & MASK_ESPN_FRAC_Y3;
	   else if((value->y3 >= 0.75)&&(value->y3 < 1))
	     sprite_attr->uPosX_16 |= (3 << B_ESPN_FRAC_Y3) & MASK_ESPN_FRAC_Y3;
	}

	return 0;
#endif
}

INT32S gplib_ppu_sprite_pal4096_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->sprite_control |= SP_PAL4096_ENABLE;
	} else {
		p_register_set->sprite_control &= ~SP_PAL4096_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_pal4096_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->extend_sprite_control |= EXSP_NEW_PALF4096_ENABLE;
	} else {
		p_register_set->extend_sprite_control &= ~EXSP_NEW_PALF4096_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

// Character number must be calculated after this function is called
INT32S gplib_ppu_sprite_attribute_new_palette_set(SpN_RAM *sprite_attr, SpN_EX_RAM *sprite_ex_attr, INT32U palf, INT32U palette_bank)
{
#if NEW_PAL_RAM_16_EN == 1
	if (!sprite_attr || !sprite_ex_attr || palf > 1 || palette_bank > 15) {
#else
    if (!sprite_attr || !sprite_ex_attr || palf > 1 || palette_bank > 7) {
#endif
		return -1;
	}


	if(palf)
	{
		sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PALF;
		sprite_ex_attr->ex_attr1 |= (1 << B_SPN_PALF) & MASK_SPN_PALF;
	}
	else
		sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PALF;

	if(palette_bank > 3)
	{
		switch(palette_bank)
		{

			case 4:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (1 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 5:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_attr->attr1 |= (1 << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (1 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 6:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                sprite_attr->attr0 |= (1 << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (1 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 7:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                sprite_attr->attr0 |= (1 << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_attr->attr1 |= (1 << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (1 << B_SPN_PB) & MASK_SPN_PB;
				break;
#if NEW_PAL_RAM_16_EN == 1
			case 8:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (2 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 9:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_attr->attr1 |= (1 << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (2 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 10:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                sprite_attr->attr0 |= (1 << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (2 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 11:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                sprite_attr->attr0 |= (1 << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_attr->attr1 |= (1 << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (2 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 12:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (3 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 13:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_attr->attr1 |= (1 << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (3 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 14:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                sprite_attr->attr0 |= (1 << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (3 << B_SPN_PB) & MASK_SPN_PB;
				break;

			case 15:
                // Palette bank high bit
                sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
                sprite_attr->attr0 |= (1 << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
                // Palette bank low bit
                sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
                sprite_attr->attr1 |= (1 << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;
                sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
                sprite_ex_attr->ex_attr1 |= (3 << B_SPN_PB) & MASK_SPN_PB;
				break;
#endif
		}
	}
	else
	{
		// Palette bank high bit
		sprite_attr->attr0 &= ~MASK_SPN_PB_HIGH;
		sprite_attr->attr0 |= ((palette_bank>>1) << B_SPN_PB_HIGH) & MASK_SPN_PB_HIGH;
		// Palette bank low bit
		sprite_attr->attr1 &= ~MASK_SPN_PB_LOW;
		sprite_attr->attr1 |= (palette_bank << B_SPN_PB_LOW) & MASK_SPN_PB_LOW;
		sprite_ex_attr->ex_attr1 &= ~MASK_SPN_PB;
	}

	return 0;
}

// Blending enable/disable and 64-level blending value is controlled by sprite attribute ram
INT32S gplib_ppu_sprite_new_blend_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Use 64 level blending in sprite attribute ram
		p_register_set->sprite_control |= SP_NEW_BLEND_ENABLE;
	} else {			// Use a global 4 level blending value in P_Blending
		p_register_set->sprite_control &= ~SP_NEW_BLEND_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_new_blend_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Use 64 level blending in sprite attribute ram
		p_register_set->extend_sprite_control |= EXSP_NEW_BLD_ENABLE;
	} else {			// Use a global 4 level blending value in P_Blending
		p_register_set->extend_sprite_control &= ~EXSP_NEW_BLD_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

// Blending enable/disable and 64-level blending value is controlled by sprite attribute ram
INT32S gplib_ppu_sprite_new_palf_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Use 64 level blending in sprite attribute ram
		p_register_set->sprite_control |= SP_PALF_ENABLE;
	} else {			// Use a global 4 level blending value in P_Blending
		p_register_set->sprite_control &= ~SP_PALF_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

INT32S gplib_ppu_exsprite_new_palf_mode_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {		// Use 64 level blending in sprite attribute ram
		p_register_set->extend_sprite_control |= EXSP_NEW_PALF_ENABLE;
	} else {			// Use a global 4 level blending value in P_Blending
		p_register_set->extend_sprite_control &= ~EXSP_NEW_PALF_ENABLE;
	}

	// Notify PPU driver to update sprite registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_SPRITE;

	return 0;
}

// Blending enable/disable and 64-level blending value is controlled by sprite attribute ram
INT32S gplib_ppu_sprite_attribute_new_blend_mode_set(SpN_EX_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value > 7) {
		return -1;
	}

	sprite_attr->ex_attr1 &= ~MASK_SPN_TYPE;
	sprite_attr->ex_attr1 |= (value << B_SPN_TYPE) & MASK_SPN_TYPE;

	return 0;
}

INT32S gplib_ppu_exsprite_attribute_new_blend_mode_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr) {
		return -1;
	}
    sprite_attr++;
	sprite_attr->attr0 &= ~MASK_SPN_TYPE;
	sprite_attr->attr0 |= (value << B_SPN_TYPE) & MASK_SPN_TYPE;

	return 0;
}

// Blending enable/disable and 64-level blending value is controlled by sprite attribute ram
INT32S gplib_ppu_sprite_attribute_new_palf_mode_set(SpN_EX_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr || value > 7) {
		return -1;
	}

	sprite_attr->ex_attr1 &= ~MASK_SPN_PALF;
	sprite_attr->ex_attr1 |= (value << B_SPN_PALF) & MASK_SPN_PALF;

	return 0;
}

INT32S gplib_ppu_exsprite_attribute_new_palf_mode_set(SpN_RAM *sprite_attr, INT32U value)
{
	if (!sprite_attr) {
		return -1;
	}
    sprite_attr++;
	sprite_attr->attr0 &= ~MASK_SPN_PALF;
	sprite_attr->attr0 |= (value << B_SPN_PALF) & MASK_SPN_PALF;

	return 0;
}

#endif
#endif		// GPLIB_PPU_EN
