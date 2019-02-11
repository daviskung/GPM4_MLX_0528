#include "gplib_ppu.h"
#include "gplib_ppu_text.h"
#include "gplib_ppu_dma.h"


//  TxNSize Definitions
#define TXN_512X256                     0
#define TXN_512X512                     1
#define TXN_1024X512                    2
#define TXN_1024X1024                   3
#define TXN_2048X1024                   4
#define TXN_2048X2048                   5
#define TXN_4096X2048                   6
#define TXN_4096X4096                   7
#define	TRANSPARENT_ENABLE 				(1 << 24)
#define	TRANSPARENT_MODE_ENABLE 		(1 << 25)
#define PPU_BUF_ALIGN_32BYTE      		0x1F
#define PPU_H_SIZE_MULTIPLY_16     		0x10
#define COLOR_ARGB						10
#define TEXT_MAX_NUMBER					2
#define DEBUG_MESSAGE_ENABLE			0

static signed int ppu_men_get = 0;
static signed int ppu_go_flag,text_h_size,text_v_size;
static ppu_buffer_info_t ppu_buffer_info_structure;
static ppu_buffer_info_t *ppu_buffer_info_set;
static unsigned int text_number_array_ptr[TEXT_MAX_NUMBER];

#if (defined GPLIB_PPU_EN) && (GPLIB_PPU_EN == 1)
static signed int gplib_ppu_buffer_get(PPU_REGISTER_SETS *p_register_set)
{
	unsigned int temp;

	if(ppu_men_get == 0)
	{
		temp = (unsigned int)gp_malloc_align(4096,32);
		if(!temp)
		{
			DBG_PRINT ("text_number_array = NULL.\n");
			return -1;
		}
		gplib_ppu_text_number_array_ptr_set(p_register_set, C_PPU_TEXT1, temp);
		text_number_array_ptr[0] = temp;
		temp = (unsigned int)gp_malloc_align(4096,32);
		if(!temp)
		{
			DBG_PRINT ("text_number_array = NULL.\n");
			return -1;
		}
		gplib_ppu_text_number_array_ptr_set(p_register_set, C_PPU_TEXT2, temp);
		text_number_array_ptr[1] = temp;
		ppu_men_get = 1;

		return 0;
	}

	return -1;
}

static signed int gplib_ppu_buffer_free(void)
{
	unsigned int temp = -1;

	if(ppu_men_get)
	{
		if(text_number_array_ptr[0])
			gp_free((void *)text_number_array_ptr[0]);
		text_number_array_ptr[0] = 0;
		if(text_number_array_ptr[1])
			gp_free((void *)text_number_array_ptr[1]);
		text_number_array_ptr[1] = 0;
		ppu_men_get = 0;
		temp = 0;
	}

	return temp;
}

INT32S frame_buffer_ppu_update_init(PPU_REGISTER_SETS *p_register_set)
{
	ppu_buffer_info_set = (ppu_buffer_info_t *)&ppu_buffer_info_structure;
	gplib_ppu_init(p_register_set);
	gplib_ppu_text_init(p_register_set, C_PPU_TEXT1);
	gplib_ppu_text_init(p_register_set, C_PPU_TEXT2);
	ppu_go_flag = 0;
	gplib_ppu_buffer_get(p_register_set);

	return 0;
}

INT32S gplib_ppu_dma_init(PPU_REGISTER_SETS *p_register_set)
{
    return frame_buffer_ppu_update_init(p_register_set);
}


INT32S frame_buffer_ppu_update_uninit(void)
{

	gplib_ppu_buffer_free();

	if(ppu_buffer_info_set)
		ppu_buffer_info_set = 0;

	return 0;
}

INT32S gplib_ppu_dma_uninit(void)
{
    return frame_buffer_ppu_update_uninit();
}


INT32S frame_buffer_ppu_update_info(PPU_REGISTER_SETS *p_register_set, ppu_buffer_info_t *ppu_info)
{
	INT32U temp = -1;

	if(ppu_info == 0)
	{
		DBG_PRINT ("ppu_info = NULL.\n");
		return -1;
	}

	gplib_ppu_dma_enable_set(p_register_set,1);
	gplib_ppu_dma_text_enable_set(p_register_set, C_PPU_TEXT1, 1);
	gplib_ppu_dma_text_enable_set(p_register_set, C_PPU_TEXT2, 1);

	switch(ppu_info->buffer_color_mode)
	{
		case COLOR_VYUY:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 3);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 3);
			gplib_ppu_fb_format_set(p_register_set, 1, 1);
			gplib_ppu_yuv_type_set(p_register_set,0);
			break;

		case COLOR_YVYU:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 3);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 3);
			gplib_ppu_fb_format_set(p_register_set, 1, 1);
			gplib_ppu_yuv_type_set(p_register_set,1);
			break;

		case COLOR_UYVY:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 3);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 3);
			gplib_ppu_fb_format_set(p_register_set, 1, 1);
			gplib_ppu_yuv_type_set(p_register_set,2);
			break;

		case COLOR_YUYV:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 3);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 3);
			gplib_ppu_fb_format_set(p_register_set, 1, 1);
			gplib_ppu_yuv_type_set(p_register_set,3);
			break;

		case COLOR_BGRG:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 2);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 2);
			gplib_ppu_fb_format_set(p_register_set, 1, 0);
			gplib_ppu_yuv_type_set(p_register_set,0);
			break;

		case COLOR_GBGR:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 2);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 2);
			gplib_ppu_fb_format_set(p_register_set, 1, 0);
			gplib_ppu_yuv_type_set(p_register_set,1);
			break;

		case COLOR_RGBG:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 2);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 2);
			gplib_ppu_fb_format_set(p_register_set, 1, 0);
			gplib_ppu_yuv_type_set(p_register_set,2);
			break;

		case COLOR_GRGB:
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 2);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 2);
			gplib_ppu_fb_format_set(p_register_set, 1, 0);
			gplib_ppu_yuv_type_set(p_register_set,3);
			break;

		case COLOR_ARGB:
			gplib_ppu_argb888_enable_set(p_register_set,1);
		case COLOR_RGBA:
			gplib_ppu_text_rgba_set(p_register_set,1);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 2);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 2);
			gplib_ppu_fb_format_set(p_register_set, 1, 2);
			break;

		default :
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT1, 1, 1);
			gplib_ppu_text_color_set(p_register_set, C_PPU_TEXT2, 1, 1);
			gplib_ppu_fb_format_set(p_register_set, 0, 0);
			gplib_ppu_yuv_type_set(p_register_set,0);
			break;
	}
	// text 1
	if(ppu_info->t_width < ppu_info->t_height)
	{
        if(ppu_info->t_height <= 512){
           text_h_size = 512;
           if(ppu_info->t_height <= 256){
                //512*256
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_512X256);
                text_v_size = 256;
           }else{
                //512*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_512X512);
                text_v_size = 512;
           }
        }else if(ppu_info->t_height <= 1024){
           text_h_size = 1024;
           if(ppu_info->t_height <= 512){
                //1024*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_1024X512);
                text_v_size = 512;
           }else{
                //1024*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_1024X1024);
                text_v_size = 1024;
           }
        }else if(ppu_info->t_height <= 2048){
           text_h_size = 2048;
           if(ppu_info->t_height <= 1024){
                //2048*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_2048X1024);
                text_v_size = 1024;
           }else{
                //2048*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_2048X2048);
                text_v_size = 2048;
           }
        }else{
           text_h_size = 4096;
           if(ppu_info->t_height <= 2048){
                //4096*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_4096X2048);
                text_v_size = 2048;
           }else{
                //4096*4096
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_4096X4096);
                text_v_size = 4096;
           }
        }
	}
	else
	{
        if(ppu_info->t_width <= 512){
           text_h_size = 512;
           if(ppu_info->t_height <= 256){
                //512*256
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_512X256);
                text_v_size = 256;
           }else{
                //512*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_512X512);
                text_v_size = 512;
           }
        }else if(ppu_info->t_width <= 1024){
           text_h_size = 1024;
           if(ppu_info->t_height <= 512){
                //1024*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_1024X512);
                text_v_size = 512;
           }else{
                //1024*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_1024X1024);
                text_v_size = 1024;
           }
        }else if(ppu_info->t_width <= 2048){
           text_h_size = 2048;
           if(ppu_info->t_height <= 1024){
                //2048*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_2048X1024);
                text_v_size = 1024;
           }else{
                //2048*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_2048X2048);
                text_v_size = 2048;
           }
        }else{
           text_h_size = 4096;
           if(ppu_info->t_height <= 2048){
                //4096*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_4096X2048);
                text_v_size = 2048;
           }else{
                //4096*4096
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT1,TXN_4096X4096);
                text_v_size = 4096;
           }
        }
	}
    // text 2
    if(ppu_info->s_width < ppu_info->s_height)
    {
        if(ppu_info->s_height <= 512){
           if(ppu_info->s_height <= 256){
                //512*256
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_512X256);
           }else{
                //512*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_512X512);
           }
        }else if(ppu_info->s_height <= 1024){
           if(ppu_info->s_height <= 512){
                //1024*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_1024X512);
           }else{
                //1024*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_1024X1024);
           }
        }else if(ppu_info->s_height <= 2048){
           if(ppu_info->s_height <= 1024){
                //2048*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_2048X1024);
           }else{
                //2048*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_2048X2048);
           }
        }else{
           if(ppu_info->s_height <= 2048){
                //4096*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_4096X2048);
           }else{
                //4096*4096
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_4096X4096);
           }
        }
    }
    else
    {
        if(ppu_info->s_width <= 512){
           if(ppu_info->s_height <= 256){
                //512*256
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_512X256);
           }else{
                //512*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_512X512);
           }
        }else if(ppu_info->s_width <= 1024){
           if(ppu_info->s_height <= 512){
                //1024*512
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_1024X512);
           }else{
                //1024*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_1024X1024);
           }
        }else if(ppu_info->s_width <= 2048){
           if(ppu_info->s_height <= 1024){
                //2048*1024
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_2048X1024);
           }else{
                //2048*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_2048X2048);
           }
        }else{
           if(ppu_info->s_height <= 2048){
                //4096*2048
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_4096X2048);
           }else{
                //4096*4096
                gplib_ppu_text_size_set(p_register_set, C_PPU_TEXT2,TXN_4096X4096);
           }
        }
    }

	if(ppu_info->s_width < 16)
	{
		DBG_PRINT ("s_width:Range 16~1920.\n");
		ppu_buffer_info_set->s_width = ppu_info->s_width;
		return -1;
	}
	else if (ppu_info->s_width % PPU_H_SIZE_MULTIPLY_16)
	{
		DBG_PRINT ("s_width:This value must be a multiply of 16.\n");
		temp = ppu_info->s_width;
		ppu_info->s_width = (temp + PPU_H_SIZE_MULTIPLY_16) & ~PPU_H_SIZE_MULTIPLY_16;
	}

	if(!ppu_info->t_width || !ppu_info->t_height || !ppu_info->t_addr || !ppu_info->s_addr)
	{
		DBG_PRINT ("PPU_TEXT1 or PPU_TEXT2 are null.\n");
		return -1;
	}

	ppu_buffer_info_set->t_width = ppu_info->t_width;
	ppu_buffer_info_set->t_height = ppu_info->t_height;
	ppu_buffer_info_set->s_width = ppu_info->s_width;
	ppu_buffer_info_set->s_height = ppu_info->s_height;

	if(ppu_info->flip_enable)
	{
		gplib_ppu_text_enable_set(p_register_set, C_PPU_TEXT2, 0);
		gplib_ppu_text_bitmap_flip_set(p_register_set, 1);
		gplib_ppu_text_flip_set(p_register_set, C_PPU_TEXT1, ppu_info->flip_mode);
		ppu_buffer_info_set->flip_enable = ppu_info->flip_enable;
		ppu_buffer_info_set->flip_mode = ppu_info->flip_mode;
	    gplib_ppu_text_calculate_number_array(p_register_set, C_PPU_TEXT1, ppu_info->s_width, ppu_info->s_height, ppu_info->s_addr);
		p_register_set->ppu_frame_buffer_fifo = 0;
		ppu_buffer_info_set->t_addr = ppu_info->t_addr;
	    temp = gplib_ppu_free_size_set(p_register_set, 0, ppu_info->t_width, ppu_info->t_height);
	}
	else
	{
		if(ppu_info->transparent_enable)
		{
			switch(ppu_info->buffer_color_mode)
			{
				case COLOR_VYUY:
				case COLOR_YVYU:
				case COLOR_UYVY:
				case COLOR_YUYV:
					temp = (ppu_info->transparent_color & 0xFFFFFF);
					if(ppu_info->transparent_mode)
						temp |= TRANSPARENT_MODE_ENABLE;
					break;

				case COLOR_BGRG:
				case COLOR_GBGR:
				case COLOR_RGBG:
				case COLOR_GRGB:
					temp = (ppu_info->transparent_color & 0xFFFFFF);
					break;

				default:
					temp = (ppu_info->transparent_color & 0xFFFF);
					break;
			}
			temp |= TRANSPARENT_ENABLE;
			p_register_set->ppu_rgb565_transparent_color = temp;
		}

		if(ppu_info->blend_enable)
            gplib_ppu_text_blend_set(p_register_set, C_PPU_TEXT2, 1, 1,ppu_info->blend_value);

		gplib_ppu_text_bitmap_flip_set(p_register_set, 0);
		gplib_ppu_text_flip_set(p_register_set, C_PPU_TEXT1, 0);
		ppu_buffer_info_set->flip_enable = 0;
		p_register_set->ppu_frame_buffer_fifo = (ppu_info->t_width - ppu_info->s_width)*2;
	    gplib_ppu_text_calculate_number_array(p_register_set, C_PPU_TEXT1, ppu_info->t_width, ppu_info->t_height, ppu_info->t_addr);
	    gplib_ppu_text_calculate_number_array(p_register_set, C_PPU_TEXT2, ppu_info->s_width, ppu_info->s_height, ppu_info->s_addr);
	    temp = gplib_ppu_free_size_set(p_register_set, 0, ppu_info->s_width, ppu_info->s_height);
    }

	return temp;
}

INT32S gplib_ppu_dma_update_info(PPU_REGISTER_SETS *p_register_set, ppu_buffer_info_t *ppu_info)
{
    return frame_buffer_ppu_update_info(p_register_set, ppu_info);
}

INT32S frame_buffer_ppu_update_go(PPU_REGISTER_SETS *p_register_set, INT32U buffer_start, INT32U buffer_x_offset, INT32U buffer_y_offset)
{
	INT32S state,temp,temp1,new_buffer_x_offset;

	if(ppu_buffer_info_set->flip_enable)
	{
		p_register_set->buffer_user_define = ppu_buffer_info_set->t_addr;
		if(ppu_buffer_info_set->flip_mode == NO_FLIP)
			gplib_ppu_text_position_set(p_register_set, C_PPU_TEXT1, 0, 0);
		else if(ppu_buffer_info_set->flip_mode == H_FLIP)
			gplib_ppu_text_position_set(p_register_set, C_PPU_TEXT1, (text_h_size - ppu_buffer_info_set->t_width), 0);
		else if(ppu_buffer_info_set->flip_mode == V_FLIP)
			gplib_ppu_text_position_set(p_register_set, C_PPU_TEXT1, 0, (text_v_size - ppu_buffer_info_set->t_height));
		else if(ppu_buffer_info_set->flip_mode == HV_FLIP)
			gplib_ppu_text_position_set(p_register_set, C_PPU_TEXT1, (text_h_size - ppu_buffer_info_set->t_width), (text_v_size - ppu_buffer_info_set->t_height));
	}
	else
	{
		if(ppu_buffer_info_set->s_width < 16)
		{
			DBG_PRINT ("s_width:Range 16~1920.\n");
			return -1;
		}

		if(buffer_start == 0)
		{
			DBG_PRINT ("buffer_start = NULL.\n");
			return -1;
		}
		new_buffer_x_offset = buffer_x_offset;
		gplib_ppu_text_position_set(p_register_set, C_PPU_TEXT1, new_buffer_x_offset, buffer_y_offset);
		temp = (buffer_start + ((ppu_buffer_info_set->t_width * buffer_y_offset * 2) + (new_buffer_x_offset * 2)));
		if (temp & PPU_BUF_ALIGN_32BYTE)
		{
			DBG_PRINT ("ppu_buffer[31:0] must be 32 bytes alignment.\n");
			temp1 = ((new_buffer_x_offset + PPU_BUF_ALIGN_32BYTE) & ~PPU_BUF_ALIGN_32BYTE);
		#if DEBUG_MESSAGE_ENABLE == 1
			DBG_PRINT ("new_buffer_y_offset[%d]\n",temp1);
		#endif
			temp = (buffer_start + ((ppu_buffer_info_set->t_width * buffer_y_offset * 2) + (temp1 * 2)));
			gplib_ppu_text_position_set(p_register_set, C_PPU_TEXT1, temp1, buffer_y_offset);
			new_buffer_x_offset = temp1;
		}
		p_register_set->buffer_user_define = temp;

		state = 0;
		// s_width out of t_width range
		if((new_buffer_x_offset + ppu_buffer_info_set->s_width) > ppu_buffer_info_set->t_width)
		{
			DBG_PRINT ("draw out of memory[x].\n");
			state |= 0x2;
		}
		// s_height out of t_height range
		if((buffer_y_offset + ppu_buffer_info_set->s_height) > ppu_buffer_info_set->t_height)
		{
			DBG_PRINT ("draw out of boundary[y].\n");
			state |= 0x1;
		}

		if(state == 3)
		{
			temp = (new_buffer_x_offset + ppu_buffer_info_set->s_width);
			temp1 = temp - ppu_buffer_info_set->t_width;
			temp = ppu_buffer_info_set->s_width - temp1;
			if(temp < 16)
			{
				DBG_PRINT ("s_width:Range 16~1920.\n");
				return -1;
			}
			else if (temp % PPU_H_SIZE_MULTIPLY_16)
			{
				DBG_PRINT ("s_width:This value must be a multiply of 16.\n");
				temp1 = (temp / PPU_H_SIZE_MULTIPLY_16);
				temp = (PPU_H_SIZE_MULTIPLY_16 * temp1);
			}
			ppu_buffer_info_set->s_height = temp;
			temp = (buffer_y_offset + ppu_buffer_info_set->s_height);
			temp1 = temp - ppu_buffer_info_set->t_height;
			temp = ppu_buffer_info_set->s_height - temp1;
			if(temp <= 0)
			{
				DBG_PRINT ("s_heigh is NULL.\n");
				return -1;
			}
			ppu_buffer_info_set->s_width = temp;
			gplib_ppu_free_size_set(p_register_set, 0, ppu_buffer_info_set->s_width, ppu_buffer_info_set->s_height);
			p_register_set->ppu_frame_buffer_fifo = (ppu_buffer_info_set->t_width - ppu_buffer_info_set->s_width)*2;
		#if DEBUG_MESSAGE_ENABLE == 1
			DBG_PRINT ("new_s_width:0x%x.\n",ppu_buffer_info_set->s_width);
			DBG_PRINT ("new_s_height:0x%x.\n",ppu_buffer_info_set->s_height);
		#endif
		}
		else if(state == 2)
		{
			temp = (new_buffer_x_offset + ppu_buffer_info_set->s_width);
			temp1 = temp - ppu_buffer_info_set->t_width;
			temp = ppu_buffer_info_set->s_width - temp1;
			if(temp < 16)
			{
				DBG_PRINT ("s_width:Range 16~1920.\n");
				return -1;
			}
			else if (temp % PPU_H_SIZE_MULTIPLY_16)
			{
				DBG_PRINT ("s_width:This value must be a multiply of 16.\n");
				temp1 = (temp / PPU_H_SIZE_MULTIPLY_16);
				temp = (PPU_H_SIZE_MULTIPLY_16 * temp1);
			}
			gplib_ppu_free_size_set(p_register_set, 0, temp,  ppu_buffer_info_set->s_height);
		#if DEBUG_MESSAGE_ENABLE == 1
			DBG_PRINT ("new_s_width:0x%x.\n",temp);
		#endif
			p_register_set->ppu_frame_buffer_fifo = (ppu_buffer_info_set->t_width - temp)*2;
		}
		else if(state == 1)
		{
			temp = (buffer_y_offset + ppu_buffer_info_set->s_height);
			temp1 = temp - ppu_buffer_info_set->t_height;
			temp = ppu_buffer_info_set->s_height - temp1;
			if(temp <= 0)
			{
				DBG_PRINT ("s_heigh is NULL.\n");
				return -1;
			}
			gplib_ppu_free_size_set(p_register_set, 0, ppu_buffer_info_set->s_width, temp);
		#if DEBUG_MESSAGE_ENABLE == 1
			DBG_PRINT ("new_s_height:0x%x.\n",temp);
		#endif
		}
	}

	temp = gplib_ppu_dma_go(p_register_set);

	if(temp == 0)
		ppu_go_flag = 1;
	else
		ppu_go_flag = 0;

	return temp;
}

INT32S gplib_ppu_dma_update_go(PPU_REGISTER_SETS *p_register_set, INT32U buffer_start, INT32U buffer_x_offset, INT32U buffer_y_offset)
{
    return frame_buffer_ppu_update_go(p_register_set, buffer_start, buffer_x_offset, buffer_y_offset);
}

INT32S frame_buffer_ppu_update_state_get(void)
{
	INT32U temp = -1;

	if(ppu_go_flag)
	{
		temp = gblib_ppu_wait_poll(1);
		ppu_go_flag = 0;
	}

	return temp;
}

INT32S gplib_ppu_dma_update_state_get(void)
{
    return frame_buffer_ppu_update_state_get();
}
#endif
