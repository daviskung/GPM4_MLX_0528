/*
* Description: This library provides PPU APIs to init, control and start PPU hardware TEXT engine.
*
* Author: Tristan Yang
*
* Date: 2008/03/03
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.01
*/

#include "gplib_ppu_text.h"

#if (defined GPLIB_PPU_EN) && (GPLIB_PPU_EN == 1)

static INT8U text_charnum_update_flag[4];

void gplib_ppu_text_set_update_reg_flag(PPU_REGISTER_SETS *p_register_set, INT32U text_index);

void gplib_ppu_text_set_update_reg_flag(PPU_REGISTER_SETS *p_register_set, INT32U text_index)
{
	// Notify PPU driver to update text registers
	if (text_index == C_PPU_TEXT1) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT1;
	} else if (text_index == C_PPU_TEXT2) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT2;
	} else if (text_index == C_PPU_TEXT3) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT3;
	} else if (text_index == C_PPU_TEXT4) {
		p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT4;
	}
}

INT32S gplib_ppu_text_init(PPU_REGISTER_SETS *p_register_set, INT32U text_index)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	text_charnum_update_flag[text_index] = 1;

	gp_memset((void *) &p_register_set->text[text_index], 0x0, sizeof(struct ppu_text_register_sets));

	if (text_index == C_PPU_TEXT3) {
		p_register_set->text3_25d_y_compress = 0x10;
	}
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	if (value) {
		p_register_set->text[text_index].control |= TXN_ENABLE;
	} else {
		p_register_set->text[text_index].control &= ~TXN_ENABLE;
	}

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

// Note: This function is initiated only when gplib_ppu_init() is called.
INT32S gplib_ppu_text_compress_disable_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= HVCMP_DISABLE;
	} else {
		p_register_set->ppu_enable &= ~HVCMP_DISABLE;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_text_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || value>3) {
		return -1;
	}

	switch (text_index) {
	case C_PPU_TEXT2:
	case C_PPU_TEXT3:
		if (value > 2) {
			return -1;
		}
		break;
	case C_PPU_TEXT4:
		if (value > 1) {
			return -1;
		}
	}
	p_register_set->text[text_index].control &= ~MASK_TXN_MODE;
	p_register_set->text[text_index].control |= (value<<B_TXN_MODE) & MASK_TXN_MODE;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

// Note: This function is initiated only when gplib_ppu_init() is called.
INT32S gplib_ppu_text_direct_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	if (value) {
		p_register_set->ppu_enable |= TX_DIRECT_ADDRESS;
	} else {
		p_register_set->ppu_enable &= ~TX_DIRECT_ADDRESS;
	}

	// Notify PPU driver to update PPU registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_PPU;

	return 0;
}

INT32S gplib_ppu_text_wallpaper_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	if (value) {
		p_register_set->text[text_index].control |= TXN_WALL_ENABLE;
	} else {
		p_register_set->text[text_index].control &= ~TXN_WALL_ENABLE;
	}

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_attribute_source_select(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	if (value) {		// Get TEXT attributes from register
		p_register_set->text[text_index].control |= TXN_REGMODE;
	} else {			// Get TEXT attributes from TXN_A_PTR
		p_register_set->text[text_index].control &= ~TXN_REGMODE;
	}

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

// TEXT horizontal move can only be used in 2D and HCMP mode
INT32S gplib_ppu_text_horizontal_move_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	if (value) {
		p_register_set->text[text_index].control |= TXN_MVE_ENABLE;
	} else {
		p_register_set->text[text_index].control &= ~TXN_MVE_ENABLE;
	}

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_size_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || value>7) {
		return -1;
	}

	p_register_set->text[text_index].attribute &= ~MASK_TXN_SIZE;
	p_register_set->text[text_index].attribute |= (value<<B_TXN_SIZE) & MASK_TXN_SIZE;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_character_size_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U txn_hs, INT32U txn_vs)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || txn_hs>3 || txn_vs>3) {
		return -1;
	}

	// Character width
	p_register_set->text[text_index].attribute &= ~MASK_TXN_HS;
	p_register_set->text[text_index].attribute |= (txn_hs<<B_TXN_HS) & MASK_TXN_HS;
	// Character height
	p_register_set->text[text_index].attribute &= ~MASK_TXN_VS;
	p_register_set->text[text_index].attribute |= (txn_vs<<B_TXN_VS) & MASK_TXN_VS;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_bitmap_mode_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	if (value) {
		p_register_set->text[text_index].control |= TXN_BMP;
	} else {
		p_register_set->text[text_index].control &= ~TXN_BMP;
	}

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_color_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U rgb_mode, INT32U color)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || rgb_mode>1 || color>3) {
		return -1;
	}

	if (rgb_mode) {
		p_register_set->text[text_index].control |= TXN_RGB15P;
	} else {
		p_register_set->text[text_index].control &= ~TXN_RGB15P;
	}
	p_register_set->text[text_index].attribute &= ~MASK_TXN_COLOR;
	p_register_set->text[text_index].attribute |= (color<<B_TXN_COLOR) & MASK_TXN_COLOR;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_palette_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U bank, INT32U palette_idx)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || bank>15 || palette_idx>15) {
		return -1;
	}
	if(bank>3)
	{
		switch(bank)
		{
			case 4:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (1<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 5:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (1<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (1<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 6:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (2<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (1<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 7:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (3<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (1<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 8:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (2<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 9:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (1<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (2<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 10:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (2<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (2<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 11:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (3<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (2<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 12:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (3<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 13:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (1<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (3<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 14:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (2<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (3<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;

			case 15:
					// Set TEXT palette bank
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
					p_register_set->text[text_index].attribute |= (3<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
					p_register_set->text[text_index].attribute |= (3<<B_TXN_PB_NEW) & MASK_TXN_PALETTE_BANK_NEW;
				break;
		}
	}
	else
	{
		// Set TEXT palette bank
		p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK;
		p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE_BANK_NEW;
		p_register_set->text[text_index].attribute |= (bank<<B_TXN_PB) & MASK_TXN_PALETTE_BANK;
	}
	// Set TEXT palette index
	p_register_set->text[text_index].attribute &= ~MASK_TXN_PALETTE;
	p_register_set->text[text_index].attribute |= (palette_idx<<B_TXN_PALETTE) & MASK_TXN_PALETTE;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_segment_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || (value&0x1)) {
		return -1;
	}

	p_register_set->text[text_index].segment = value;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_attribute_array_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	p_register_set->text[text_index].a_ptr = value;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_number_array_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	p_register_set->text[text_index].n_ptr = value;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT8U gplib_ppu_text_number_array_update_flag_get(INT32U text_index)
{
	return text_charnum_update_flag[text_index];
}

void gplib_ppu_text_number_array_update_flag_clear(void)
{
	text_charnum_update_flag[C_PPU_TEXT1] = 0;
	text_charnum_update_flag[C_PPU_TEXT2] = 0;
	text_charnum_update_flag[C_PPU_TEXT3] = 0;
	text_charnum_update_flag[C_PPU_TEXT4] = 0;
}

// This function should be called after one of the following settings is modified:
// TEXT segment, Direct/relative address mode, Character/bitmap mode, TEXT size, Character size,
// TEXT(Photo) color, Photo data address, Photo size,
// Wallpaper mode, Character 0 Auto-transparent mode
INT32S gplib_ppu_text_calculate_number_array(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U photo_width, INT32U photo_height, INT32U data_ptr)
{
	INT8U text_size_idx, char_hs, char_vs;
	INT16U text_width, text_height;
	INT8U char_width, char_height;
	INT16U text_char_count_x, text_char_count_y;
	INT16U photo_char_count_x, photo_char_count_y;
	INT16U *p16;
	INT32U *p32;
	INT32U number_value;
	INT32U char_number_offset, bitmap_offset;
	INT16U i, j;

	if (!p_register_set) {
		return -1;
	}

	text_size_idx = (p_register_set->text[text_index].attribute & MASK_TXN_SIZE) >> B_TXN_SIZE;		// TX_SIZE(0~7)
	char_hs = (p_register_set->text[text_index].attribute & MASK_TXN_HS) >> B_TXN_HS;		// TX_HS(0~3)
	char_vs = (p_register_set->text[text_index].attribute & MASK_TXN_VS) >> B_TXN_VS;		// TX_VS(0~3)
	if (text_size_idx < 0x4) {			// 512x256 ~ 1024x1024
		text_width = 1 << (9+(text_size_idx>>1));					// 512*(1<<(text_size_idx>>1))
		text_char_count_x = 1 << (6+(text_size_idx>>1)-char_hs);	// 512*(1<<(text_size_idx>>1))/(8<<TX_HS)
		if (text_size_idx & 0x1) {		// Text height is equal to text width
			text_height = text_width;
			text_char_count_y = 1 << (6+(text_size_idx>>1)-char_vs);		// 512*(1<<(text_size_idx>>1)) / (8<<TX_VS)
		} else {						// Text height is half of text width
			text_height = text_width >> 1;
			text_char_count_y = 1 << (5+(text_size_idx>>1)-char_vs);		// 256*(1<<(text_size_idx>>1)) / (8<<TX_VS)
		}

		char_width = 1 << (3+char_hs);
		char_height = 1 << (3+char_vs);

		photo_char_count_x = (photo_width+char_width-1) >> (3+char_hs);		// count = photo width / char width
		photo_char_count_y = (photo_height+char_height-1) >> (3+char_vs);	// count = photo height / char height

		char_number_offset = 1 << (3+3+char_hs+char_vs+1);		// Assume character size is char_width*char_height*2

	} else {		// 2048x1024 ~ 4096x4096
		text_width = 1 << (9+(text_size_idx>>1));					// 2048*(1<<(text_size_idx>>1))
		text_char_count_x = 1 << (4+(text_size_idx>>1)-char_hs);	// 2048*(1<<(text_size_idx>>1))/(32<<TX_HS)
		if (text_size_idx & 0x1) {		// Text height is equal to text width
			text_height = text_width;
			text_char_count_y = 1 << (4+(text_size_idx>>1)-char_vs);		// 2048*(1<<(text_size_idx>>1)) / (32<<TX_VS)
		} else {						// Text height is half of text width
			text_height = text_width >> 1;
			text_char_count_y = 1 << (3+(text_size_idx>>1)-char_vs);		// 1024*(1<<(text_size_idx>>1)) / (32<<TX_VS)
		}

		char_width = 1 << (5+char_hs);
		char_height = 1 << (5+char_vs);

		photo_char_count_x = (photo_width+char_width-1) >> (5+char_hs);		// count = photo width / char width
		photo_char_count_y = (photo_height+char_height-1) >> (5+char_vs);	// count = photo height / char height

		char_number_offset = 1 << (5+5+char_hs+char_vs+1);		// Assume character size is char_width*char_height*2
	}

	// First, check whether special bitmap mode is used
	if ((p_register_set->text[text_index].control&0x3)==0x3 && (p_register_set->text[text_index].attribute&MASK_TXN_HS)==MASK_TXN_HS) {
		// Special bitmap uses segment address as start address of pixel data. Character number array is not used.
		p_register_set->text[text_index].segment = data_ptr;

		return 0;
	} else if (p_register_set->text[text_index].control & TXN_BMP) {	// Second, handle bitmap mode
		p32 = (INT32U *) p_register_set->text[text_index].n_ptr;
		if (!p32) {
			return -1;
		}

		if (p_register_set->text[text_index].control & TXN_WALL_ENABLE) {		// Wallpaper mode
			*p32 = data_ptr;

			return 0;
		} else {		// Normal bitmap mode
			bitmap_offset = photo_width;
			if (p_register_set->text[text_index].control & TXN_RGB15P) {		// YUYV or RGBG
				bitmap_offset <<= 1;		// Each pixel contains 2-byte data
			} else {
				switch ((p_register_set->text[text_index].attribute & MASK_TXN_COLOR) >> B_TXN_COLOR) {
				case 0:			// 2-bit color mode
					bitmap_offset >>= 2;
					break;
				case 1:			// 4-bit color mode
					bitmap_offset >>= 1;
					break;
				case 2:			// 6-bit color mode
					bitmap_offset += (bitmap_offset>>1);	// +0.5
					bitmap_offset >>= 1;					// divided by 2
					break;
				case 3:			// 8-bit color mode
					break;
				}
			}

			if (((p_register_set->text[text_index].attribute & MASK_TXN_FLIP)>>B_TXN_FLIP)== 0x2) {		// Vertical flip
				number_value = data_ptr + (photo_height-1)*bitmap_offset;
				for (i=0; i<text_height; i++) {
					*p32 = number_value;
					p32++;
					number_value -= bitmap_offset;
				}
			} else {
				number_value = data_ptr;
				for (i=0; i<text_height; i++) {
					*p32 = number_value;
					p32++;
					number_value += bitmap_offset;
				}
			}
			text_charnum_update_flag[text_index] = 1;
		}

		return 0;
	} else {		// Third, handle character mode
		// Direct address mode or RGB mode is enabled, character number is a 32-bit array
		if ((p_register_set->ppu_enable & TX_DIRECT_ADDRESS) || (p_register_set->text[text_index].control & TXN_RGB15P)) {
			INT32U *p32_x_start, *p32_y_start;
			INT8U flip;

			p32 = (INT32U *) p_register_set->text[text_index].n_ptr;
			if (!p32) {
				return -1;
			}

			if (p_register_set->text[text_index].control & TXN_WALL_ENABLE) {		// Wallpaper mode
				if (data_ptr >= p_register_set->text[text_index].segment) {
					*p32 = data_ptr - p_register_set->text[text_index].segment;

					return 0;
				}

				return -1;			// Segment value is incorrect
			} else {		// Normal character mode
				if (data_ptr < p_register_set->text[text_index].segment) {
					return -1;
				}

				if (!(p_register_set->text[text_index].control & TXN_RGB15P)) {
					// In the begining of this function, we assume color is 2-byte. Now we have to shrink char_number_offset here.
					switch ((p_register_set->text[text_index].attribute & MASK_TXN_COLOR) >> B_TXN_COLOR) {
					case 0:			// 2-bit color mode
						char_number_offset >>= 3;
						break;
					case 1:			// 4-bit color mode
						char_number_offset >>= 2;
						break;
					case 2:			// 6-bit color mode
						char_number_offset *= 3;
						char_number_offset >>= 3;
						break;
					case 3:			// 8-bit color mode
						char_number_offset >>= 1;
						break;
					}
				}

				number_value = data_ptr - p_register_set->text[text_index].segment;
				p32_y_start = p32;
				flip = (p_register_set->text[text_index].attribute & MASK_TXN_FLIP) >> B_TXN_FLIP;
				for (j=0; j<text_char_count_y; j++) {
					if (flip==0x2 && j<photo_char_count_y) {
						p32_x_start = p32_y_start + (photo_char_count_y-1-j)*text_char_count_x;
					} else {
						p32_x_start = p32;
					}
					for (i=0; i<text_char_count_x; i++) {
						if (i<photo_char_count_x && j<photo_char_count_y) {
							if (flip == 0x1) {		// Horizontal flip
								*(p32_x_start+(photo_char_count_x-1-i)) = number_value;
							} else if (flip == 0x2) {		// Vertical flip
								*(p32_x_start + i) = number_value;
							} else {
								*p32 = number_value;
							}

							number_value += char_number_offset;
						} else {
							if (p_register_set->ppu_enable & CHAR0_TRANSPARENT_ENABLE) {	// Character 0 transparent mode
								*p32 = 0x0;
							} else {	// Use next character
								*p32 = number_value;
							}
						}
						p32++;
					}

					// If photo width is larger than TEXT width, we have to skip some photo character
					while (i < photo_char_count_x) {
						number_value += char_number_offset;
						i++;
					}
				}
				text_charnum_update_flag[text_index] = 1;
			}
			return 0;
		} else {	// Releative address mode and color is 2-bit or 4-bit or 6-bit, character number is a 16-bit array
			INT16U *p16_x_start, *p16_y_start;
			INT8U flip;

			if (data_ptr < p_register_set->text[text_index].segment) {
				return -1;
			}

			number_value = 	data_ptr - p_register_set->text[text_index].segment;
			p16 = (INT16U *) p_register_set->text[text_index].n_ptr;
			if (!p16) {
				return -1;
			}

			if (text_size_idx < 0x4) {
				i = 3 + 3 + char_hs + char_vs + 1;	// assume each pixel contains 2-bytes
			} else {
				i = 5 + 5 + char_hs + char_vs + 1;	// assume each pixel contains 2-bytes
			}
			// In the begining of this function, we assume color is 2-byte. Now we have to shrink char_number_offset here.
			switch ((p_register_set->text[text_index].attribute & MASK_TXN_COLOR) >> B_TXN_COLOR) {
			case 0:			// 2-bit color mode
				i -= 3;
				number_value >>= i;
				break;
			case 1:			// 4-bit color mode
				i -= 2;
				number_value >>= i;
				break;
			case 2:			// 6-bit color mode

				char_number_offset += (char_number_offset>>1);	// +0.5
				char_number_offset >>= 2;						// divided by 2 then divided by 2 again
				number_value /= char_number_offset;
				break;
			case 3:			// 8-bit color mode
				i -= 1;
				number_value >>= i;
				break;
			}

			p16_y_start = p16;
			flip = (p_register_set->text[text_index].attribute & MASK_TXN_FLIP) >> B_TXN_FLIP;
			for (j=0; j<text_char_count_y; j++) {
				if (flip==0x2 && j<photo_char_count_y) {
					p16_x_start = p16_y_start + (photo_char_count_y-1-j)*text_char_count_x;
				} else {
					p16_x_start = p16;
				}
				for (i=0; i<text_char_count_x; i++) {
					if (i<photo_char_count_x && j<photo_char_count_y) {
						if (flip == 0x1) {		// Horizontal flip
							*(p16_x_start+(photo_char_count_x-1-i)) = number_value;
						} else if (flip == 0x2) {		// Vertical flip
							*(p16_x_start + i) = number_value;
						} else {
							*p16 = number_value;
						}

						number_value++;
					} else {
						if (p_register_set->ppu_enable & CHAR0_TRANSPARENT_ENABLE) {	// Character 0 transparent mode
							*p16 = 0x0;
						} else {	// Use next character
							*p16 = number_value;
						}
					}
					p16++;
				}

				// If photo width is larger than TEXT width, we have to skip some photo character
				while (i < photo_char_count_x) {
					number_value++;
					i++;
				}
			}
			text_charnum_update_flag[text_index] = 1;
		}
	}

	return 0;
}

INT32S gplib_ppu_text_position_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U pos_x, INT32U pos_y)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || pos_x>0xFFF || pos_y>0xFFF) {
		return -1;
	}

	p_register_set->text[text_index].position_x = (INT16U) pos_x;
	p_register_set->text[text_index].position_y = (INT16U) pos_y;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_offset_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U offset_x, INT32U offset_y)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || offset_x>0x3FF || offset_y>0x3FF) {
		return -1;
	}

	p_register_set->text[text_index].offset_x = (INT16U) offset_x;
	p_register_set->text[text_index].offset_y = (INT16U) offset_y;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}


INT32S gplib_ppu_text_depth_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || value>3) {
		return -1;
	}

	p_register_set->text[text_index].attribute &= ~MASK_TXN_DEPTH;
	p_register_set->text[text_index].attribute |= (value<<B_TXN_DEPTH) & MASK_TXN_DEPTH;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_blend_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U enable, INT32U mode, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || mode>1 || value>63) {
		return -1;
	}

	// Set blend enable bit
	if (enable) {		// enable blending function
		p_register_set->text[text_index].control |= TXN_BLD_ENABLE;
	} else {			// disable blending function
		p_register_set->text[text_index].control &= ~TXN_BLD_ENABLE;
	}
	// Set blend mode bit
	if (mode) {			// Use 64 level blending in TEXT control register
		p_register_set->text[text_index].control |= TXN_BLDMODE64_ENABLE;
	} else {			// Use a global 4 level blending value in P_Blending register
		p_register_set->text[text_index].control &= ~TXN_BLDMODE64_ENABLE;
	}
	// Set blend value bits
	p_register_set->text[text_index].control &= ~MASK_TXN_BLDLVL;
	p_register_set->text[text_index].control |= (value<<B_TXN_BLDLVL) & MASK_TXN_BLDLVL;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_flip_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || value>3) {
		return -1;
	}

	p_register_set->text[text_index].attribute &= ~MASK_TXN_FLIP;
	p_register_set->text[text_index].attribute |= (value<<B_TXN_FLIP) & MASK_TXN_FLIP;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_sine_cosine_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT16U r_sine, INT16U r_cosine)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	p_register_set->text[text_index].sine = (INT16U) (r_sine & 0x1FFF);
	p_register_set->text[text_index].cosine = (INT16U) (r_cosine & 0x1FFF);

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}


INT32S gplib_ppu_text_window_select(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || value>3) {
		return -1;
	}

	p_register_set->text[text_index].attribute &= ~MASK_TXN_WINDOW;
	p_register_set->text[text_index].attribute |= (value<<B_TXN_WINDOW) & MASK_TXN_WINDOW;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

// TEXT special effect can only be used in YUV type color mode(TXN_RGBM=1 and TXN_COLOR=3)
INT32S gplib_ppu_text_special_effect_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4 || value>3) {
		return -1;
	}

	p_register_set->text[text_index].attribute &= ~MASK_TXN_SPECIAL_EFFECT;
	p_register_set->text[text_index].attribute |= (value<<B_TXN_SPECIAL_EFFECT) & MASK_TXN_SPECIAL_EFFECT;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

// Vertical compress applies to TEXT1 and TEXT2 only
// Note: This function is initiated only when gplib_ppu_init() is called.
INT32S gplib_ppu_text_vertical_compress_set(PPU_REGISTER_SETS *p_register_set, INT32U value, INT32U offset, INT32U step)
{
	if (!p_register_set || value>0x1FF || offset>0x1FF || step>0x1FF) {
		return -1;
	}

	p_register_set->ppu_vcompress_value = (INT16U) value;
	p_register_set->ppu_vcompress_offset = (INT16U) offset;
	p_register_set->ppu_vcompress_step = (INT16U) step;

	// Notify PPU driver to update text registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT1;

	return 0;
}

INT32S gplib_ppu_text_horizontal_move_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	p_register_set->ppu_horizontal_move_ptr = value;

	// Notify PPU driver to update text registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_HORIZONTAL_MOVE;

	return 0;
}

INT32S gplib_ppu_text1_horizontal_compress_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set) {
		return -1;
	}

	p_register_set->ppu_text1_hcompress_ptr = value;

	// Notify PPU driver to update text registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT1_HCOMPRESS;

	return 0;
}

const INT16S C_SIN_TABLE[] = {		//sine data table (0, 1,..., 359 degreed)
     0,     18,     36,     54,     71,     89,    107,    125,    143,    160,    178,    195,    213,    230,    248,
   265,    282,    299,    316,    333,    350,    367,    384,    400,    416,    433,    449,    465,    481,    496,
   512,    527,    543,    558,    573,    587,    602,    616,    630,    644,    658,    672,    685,    698,    711,
   724,    737,    749,    761,    773,    784,    796,    807,    818,    828,    839,    849,    859,    868,    878,
   887,    896,    904,    912,    920,    928,    935,    943,    949,    956,    962,    968,    974,    979,    984,
   989,    994,    998,   1002,   1005,   1008,   1011,   1014,   1016,   1018,   1020,   1022,   1023,   1023,   1024,
  1024,   1024,   1023,   1023,   1022,   1020,   1018,   1016,   1014,   1011,   1008,   1005,   1002,    998,    994,
   989,    984,    979,    974,    968,    962,    956,    949,    943,    935,    928,    920,    912,    904,    896,
   887,    878,    868,    859,    849,    839,    828,    818,    807,    796,    784,    773,    761,    749,    737,
   724,    711,    698,    685,    672,    658,    644,    630,    616,    602,    587,    573,    558,    543,    527,
   512,    496,    481,    465,    449,    433,    416,    400,    384,    367,    350,    333,    316,    299,    282,
   265,    248,    230,    213,    195,    178,    160,    143,    125,    107,     89,     71,     54,     36,     18,
     0,    -18,    -36,    -54,    -71,    -89,   -107,   -125,   -143,   -160,   -178,   -195,   -213,   -230,   -248,
  -265,   -282,   -299,   -316,   -333,   -350,   -367,   -384,   -400,   -416,   -433,   -449,   -465,   -481,   -496,
  -512,   -527,   -543,   -558,   -573,   -587,   -602,   -616,   -630,   -644,   -658,   -672,   -685,   -698,   -711,
  -724,   -737,   -749,   -761,   -773,   -784,   -796,   -807,   -818,   -828,   -839,   -849,   -859,   -868,   -878,
  -887,   -896,   -904,   -912,   -920,   -928,   -935,   -943,   -949,   -956,   -962,   -968,   -974,   -979,   -984,
  -989,   -994,   -998,  -1002,  -1005,  -1008,  -1011,  -1014,  -1016,  -1018,  -1020,  -1022,  -1023,  -1023,  -1024,
 -1024,  -1024,  -1023,  -1023,  -1022,  -1020,  -1018,  -1016,  -1014,  -1011,  -1008,  -1005,  -1002,   -998,   -994,
  -989,   -984,   -979,   -974,   -968,   -962,   -956,   -949,   -943,   -935,   -928,   -920,   -912,   -904,   -896,
  -887,   -878,   -868,   -859,   -849,   -839,   -828,   -818,   -807,   -796,   -784,   -773,   -761,   -749,   -737,
  -724,   -711,   -698,   -685,   -672,   -658,   -644,   -630,   -616,   -602,   -587,   -573,   -558,   -543,   -527,
  -512,   -496,   -481,   -465,   -449,   -433,   -416,   -400,   -384,   -367,   -350,   -333,   -316,   -299,   -282,
  -265,   -248,   -230,   -213,   -195,   -178,   -160,   -143,   -125,   -107,    -89,    -71,    -54,    -36,    -18
};

const INT16S C_COS_TABLE[] = {		//cosine data table (0, 1,..., 359 degreed)
	  1024,   1024,   1023,   1023,   1022,   1020,   1018,   1016,   1014,   1011,   1008,   1005,   1002,    998,    994,
	   989,    984,    979,    974,    968,    962,    956,    949,    943,    935,    928,    920,    912,    904,    896,
	   887,    878,    868,    859,    849,    839,    828,    818,    807,    796,    784,    773,    761,    749,    737,
	   724,    711,    698,    685,    672,    658,    644,    630,    616,    602,    587,    573,    558,    543,    527,
	   512,    496,    481,    465,    449,    433,    416,    400,    384,    367,    350,    333,    316,    299,    282,
	   265,    248,    230,    213,    195,    178,    160,    143,    125,    107,     89,     71,     54,     36,     18,
	     0,    -18,    -36,    -54,    -71,    -89,   -107,   -125,   -143,   -160,   -178,   -195,   -213,   -230,   -248,
	  -265,   -282,   -299,   -316,   -333,   -350,   -367,   -384,   -400,   -416,   -433,   -449,   -465,   -481,   -496,
	  -512,   -527,   -543,   -558,   -573,   -587,   -602,   -616,   -630,   -644,   -658,   -672,   -685,   -698,   -711,
	  -724,   -737,   -749,   -761,   -773,   -784,   -796,   -807,   -818,   -828,   -839,   -849,   -859,   -868,   -878,
	  -887,   -896,   -904,   -912,   -920,   -928,   -935,   -943,   -949,   -956,   -962,   -968,   -974,   -979,   -984,
	  -989,   -994,   -998,  -1002,  -1005,  -1008,  -1011,  -1014,  -1016,  -1018,  -1020,  -1022,  -1023,  -1023,  -1024,
	 -1024,  -1024,  -1023,  -1023,  -1022,  -1020,  -1018,  -1016,  -1014,  -1011,  -1008,  -1005,  -1002,   -998,   -994,
	  -989,   -984,   -979,   -974,   -968,   -962,   -956,   -949,   -943,   -935,   -928,   -920,   -912,   -904,   -896,
	  -887,   -878,   -868,   -859,   -849,   -839,   -828,   -818,   -807,   -796,   -784,   -773,   -761,   -749,   -737,
	  -724,   -711,   -698,   -685,   -672,   -658,   -644,   -630,   -616,   -602,   -587,   -573,   -558,   -543,   -527,
	  -512,   -496,   -481,   -465,   -449,   -433,   -416,   -400,   -384,   -367,   -350,   -333,   -316,   -299,   -282,
	  -265,   -248,   -230,   -213,   -195,   -178,   -160,   -143,   -125,   -107,    -89,    -71,    -54,    -36,    -18,
	     0,     18,     36,     54,     71,     89,    107,    125,    143,    160,    178,    195,    213,    230,    248,
	   265,    282,    299,    316,    333,    350,    367,    384,    400,    416,    433,    449,    465,    481,    496,
	   512,    527,    543,    558,    573,    587,    602,    616,    630,    644,    658,    672,    685,    698,    711,
	   724,    737,    749,    761,    773,    784,    796,    807,    818,    828,    839,    849,    859,    868,    878,
	   887,    896,    904,    912,    920,    928,    935,    943,    949,    956,    962,    968,    974,    979,    984,
	   989,    994,    998,   1002,   1005,   1008,   1011,   1014,   1016,   1018,   1020,   1022,   1023,   1023,   1024
};

INT32S gplib_ppu_text_rotate_zoom_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT16S angle, FP32 factor_k)
{
	INT16U r_sine, r_cosine;
	INT32S value;

	if (!p_register_set || text_index>C_PPU_TEXT4 || factor_k<0) {
		return -1;
	}

	while (angle >= 3600) {
		angle -= 3600;
	}
	while (angle >= 360) {
		angle -= 360;
	}
	while (angle <= -3600) {
		angle += 3600;
	}
	while (angle < 0) {
		angle += 360;
	}

	if (factor_k == 1) {
		value = (INT32S) C_SIN_TABLE[angle];
	} else {
		value = (INT32S) ((FP32) C_SIN_TABLE[angle] / factor_k);
	}
	if (value > 0x7FF*64) {					// Out of maximum positive value
		r_sine = 0x17FF;
	} else if (value > 0x7FF) {				// 64 times boost must be used for positive value
		r_sine = (INT16U) (value >> 6);
		r_sine |= 0x1000;					// enable 64 times burst
	} else if (value >= 0) {				// 1.999 >= r*cosine >= 0
		r_sine = (INT16U) value;
	} else if (value >= -(0x7FF)) {			// 0 > r*cosine >= -1.999
		r_sine = (INT16U) value;
		r_sine &= 0xFFF;
	} else if (value >= -(0x7FF*64)) {		// 64 times boost must be used for negative value
		r_sine = (INT16U) (value >> 6);
		r_sine &= 0xFFF;
		r_sine |= 0x1000;
	} else {								// Out of maximum negative value
		r_sine = 0x1800;
	}

	if (factor_k == 1) {
		value = (INT32S) C_COS_TABLE[angle];
	} else {
		value = (INT32S) ((FP32) C_COS_TABLE[angle] / factor_k);
	}
	if (value > 0x7FF*64) {					// Out of maximum positive value
		r_cosine = 0x17FF;
	} else if (value > 0x7FF) {				// 64 times boost must be used for positive value
		r_cosine = (INT16U) (value >> 6);
		r_cosine |= 0x1000;					// enable 64 times burst
	} else if (value >= 0) {				// 1.999 >= r*cosine >= 0
		r_cosine = (INT16U) value;
	} else if (value >= -(0x7FF)) {			// 0 > r*cosine >= -1.999
		r_cosine = (INT16U) value;
		r_cosine &= 0xFFF;
	} else if (value >= -(0x7FF*64)) {		// 64 times boost must be used for negative value
		r_cosine = (INT16U) (value >> 6);
		r_cosine &= 0xFFF;
		r_cosine |= 0x1000;
	} else {								// Out of maximum negative value
		r_cosine = 0x1800;
	}

	gplib_ppu_text_sine_cosine_set(p_register_set, text_index, (INT16U) r_sine, (INT16U) r_cosine);

	return 0;
}

INT32S gplib_ppu_text3_25d_set(PPU_REGISTER_SETS *p_register_set, INT16S angle, FP32 *factor_ptr)
{
	INT16U r_sine, r_cosine;
	INT32S value;
	INT16U *ptr;
	INT16U i;

	if (!p_register_set || !factor_ptr) {
		return -1;
	}

	while (angle >= 3600) {
		angle -= 3600;
	}
	while (angle >= 360) {
		angle -= 360;
	}
	while (angle <= -3600) {
		angle += 3600;
	}
	while (angle < 0) {
		angle += 360;
	}

	ptr = (INT16U *) p_register_set->ppu_text3_25d_ptr;
	if (!ptr) {
		return -1;
	}
	for (i=0; i<240; i++) {
		value = (INT32S) ((FP32) C_SIN_TABLE[angle] / *(factor_ptr+i));
		if (value > 0x7FF*64) {					// Out of maximum positive value
			r_sine = 0x17FF;
		} else if (value > 0x7FF) {				// 64 times boost must be used for positive value
			r_sine = (INT16U) (value >> 6);
			r_sine |= 0x1000;					// enable 64 times burst
		} else if (value >= 0) {				// 1.999 >= r*cosine >= 0
			r_sine = (INT16U) value;
		} else if (value >= -(0x7FF)) {			// 0 > r*cosine >= -1.999
			r_sine = (INT16U) value;
			r_sine &= 0xFFF;
		} else if (value >= -(0x7FF*64)) {		// 64 times boost must be used for negative value
			r_sine = (INT16U) (value >> 6);
			r_sine &= 0xFFF;
			r_sine |= 0x1000;
		} else {								// Out of maximum negative value
			r_sine = 0x1800;
		}

		value = (INT32S) ((FP32) C_COS_TABLE[angle] / *(factor_ptr+i));
		if (value > 0x7FF*64) {					// Out of maximum positive value
			r_cosine = 0x17FF;
		} else if (value > 0x7FF) {				// 64 times boost must be used for positive value
			r_cosine = (INT16U) (value >> 6);
			r_cosine |= 0x1000;					// enable 64 times burst
		} else if (value >= 0) {				// 1.999 >= r*cosine >= 0
			r_cosine = (INT16U) value;
		} else if (value >= -(0x7FF)) {			// 0 > r*cosine >= -1.999
			r_cosine = (INT16U) value;
			r_cosine &= 0xFFF;
		} else if (value >= -(0x7FF*64)) {		// 64 times boost must be used for negative value
			r_cosine = (INT16U) (value >> 6);
			r_cosine &= 0xFFF;
			r_cosine |= 0x1000;
		} else {								// Out of maximum negative value
			r_cosine = 0x1800;
		}

		*ptr++ = r_cosine;
		*ptr++ = r_sine;
	}

	// Notify PPU driver to update text registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT3_25D;

	return STATUS_OK;
}

// Y compress under 2.5D mode applies to TEXT3 only
INT32S gplib_ppu_text3_25d_y_compress_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || value>0x3F) {
		return -1;
	}

	p_register_set->text3_25d_y_compress = (INT16U) value;

	// Notify PPU driver to update text registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT3;

	return 0;
}

INT32S gplib_ppu_text3_25d_cossine_ptr_set(PPU_REGISTER_SETS *p_register_set, INT32U value)
{
	if (!p_register_set || !value) {
		return -1;
	}

    Text25D_CosSineBuf_set(p_register_set, value);

	return 0;
}

INT32S Text25D_CosSineBuf_set(PPU_REGISTER_SETS *p_register_set, INT32U CosSineBuf)
{
    if (!p_register_set || !CosSineBuf)
		return -1;

    p_register_set->ppu_text3_25d_ptr=CosSineBuf;

	// Notify PPU driver to update text registers
	p_register_set->update_register_flag |= C_UPDATE_REG_SET_TEXT3;

    return 0;
}

INT32S gplib_ppu_text_blend_type_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index > C_PPU_TEXT4) {
		return -1;
	}

	p_register_set->text[text_index].control &= MASK_TXN_BLDTYPE;
	p_register_set->text[text_index].control |= (value << B_TXN_BLDTYPE) & MASK_TXN_BLDTYPE;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

INT32S gplib_ppu_text_palf_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index > C_PPU_TEXT4) {
		return -1;
	}

	p_register_set->text[text_index].attribute &= MASK_B_TXN_PALF_BANK;
	p_register_set->text[text_index].attribute |= (value << B_TXN_PALF) & MASK_B_TXN_PALF_BANK;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

/**
 * @brief 		PPU text enable function.
* @param 	p_register_set [in]: PPU struct value set.
* @param 	text_index [in]: text_index:0:TEXT0,1: TEXT1,2: TEXT2,3: TEXT3.
* @param 	value [in]: value:0=disable 1=enable.
* @return 	SUCCESS/ERROR_ID.
*/
INT32S  gplib_ppu_dma_text_enable_set(PPU_REGISTER_SETS *p_register_set, INT32U text_index, INT32U value)
{
	if (!p_register_set || text_index>C_PPU_TEXT4) {
		return -1;
	}

	if (value) {
		p_register_set->text[text_index].control |= TXN_ENABLE;
	} else {
		p_register_set->text[text_index].control &= ~TXN_ENABLE;
	}

	p_register_set->ppu_enable &= ~HVCMP_DISABLE;
	p_register_set->ppu_enable &= ~TX_DIRECT_ADDRESS;
	p_register_set->text[text_index].control &= ~TXN_WALL_ENABLE;
	p_register_set->text[text_index].control &= ~TXN_MVE_ENABLE;
	p_register_set->text[text_index].control &= ~MASK_TXN_MODE;
	p_register_set->text[text_index].attribute &= ~MASK_TXN_HS;
	p_register_set->text[text_index].attribute &= ~MASK_TXN_VS;
	p_register_set->text[text_index].attribute &= ~MASK_TXN_DEPTH;

	p_register_set->text[text_index].control |= TXN_BMP;
	p_register_set->text[text_index].control |= TXN_REGMODE;
	p_register_set->text[text_index].segment = 0;
	p_register_set->text[text_index].position_x = 0;
	p_register_set->text[text_index].position_y = 0;

	// Notify PPU driver to update text registers
	gplib_ppu_text_set_update_reg_flag(p_register_set, text_index);

	return 0;
}

#endif		// GPLIB_PPU_EN
