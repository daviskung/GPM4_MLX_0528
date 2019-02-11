/*
* Description: This file parse/decode/scale images
*
* Author: Tristan Yang
*
* Date: 2008/07/19
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*/
#include "ap_image_task.h"
#include "turnkey_filesrv_task.h"
#include "drv_l1_scaler.h"
#include "drv_l1_cache.h"
#include "drv_l2_scaler.h"

#if (defined APP_IMAGE_CODEC_EN ) && (APP_IMAGE_CODEC_EN == 1)

#define C_USE_SCALER_NUMBER				SCALER_0
// Control variables
#define C_IMAGE_READ_FILE_BUFFER_SIZE		512

static TK_FILE_SERVICE_STRUCT fs_cmd;
extern MSG_Q_ID	fs_msg_q_id;

#define C_SCALER_L2_EN                  1

INT32S image_decode_with_clip_and_scale(IMAGE_DECODE_STRUCT *img_decode_struct, INT32U clip_xy, INT32U clip_size);

TK_IMAGE_TYPE_ENUM image_buffer_judge_type(INT8U *buffer_ptr)
{
	if (*buffer_ptr==0xFF && *(buffer_ptr+1)==0xD8) {	    // JPEG
		return 	TK_IMAGE_TYPE_JPEG;
	} else if (*buffer_ptr=='R' && *(buffer_ptr+1)=='I' && *(buffer_ptr+2)=='F' && *(buffer_ptr+3)=='F') {		// Motion-JPEG
		return TK_IMAGE_TYPE_MOTION_JPEG;
	} else if (*buffer_ptr=='G' && *(buffer_ptr+1)=='P' && *(buffer_ptr+2)=='Z' && *(buffer_ptr+3)=='P') {		// GPZP
		return TK_IMAGE_TYPE_GPZP;
	} else if (*buffer_ptr=='B' && *(buffer_ptr+1)=='M') {		// BMP
		return TK_IMAGE_TYPE_BMP;
	} else if (*buffer_ptr=='G' && *(buffer_ptr+1)=='I' && *(buffer_ptr+2)=='F' ) {		// GIF
		return TK_IMAGE_TYPE_GIF;
  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
	} else {
		INT8U tmp[12];
		INT8U i;

		for (i=0; i<12; i++) {
			tmp[i] = (*(buffer_ptr+i)) ^ 0xFF;
		}

		if (tmp[0]==0xFF && tmp[1]==0xD8) {	    // JPEG
			return 	TK_IMAGE_TYPE_JPEG;
		} else if (tmp[0]=='R' && tmp[1]=='I' && tmp[2]=='F' && tmp[3]=='F') {		// Motion-JPEG
			return TK_IMAGE_TYPE_MOTION_JPEG;
		} else if (tmp[0]=='G' && tmp[1]=='P' && tmp[2]=='Z' && tmp[3]=='P') {		// GPZP
			return TK_IMAGE_TYPE_GPZP;
		} else if (tmp[0]=='B' && tmp[1]=='M') {		// BMP
			return TK_IMAGE_TYPE_BMP;
		} else if (tmp[0]=='G' && tmp[1]=='I' && tmp[2]=='F' ) {		// GIF
			return TK_IMAGE_TYPE_GIF;
		} else if ((tmp[4]=='p' && tmp[5]=='n' && tmp[6]=='o' && tmp[7]=='t') ||
		           (tmp[4]=='s' && tmp[5]=='k' && tmp[6]=='i' && tmp[7]=='p') ||
		           (tmp[4]=='w' && tmp[5]=='i' && tmp[6]=='d' && tmp[7]=='e') ||
		           (tmp[4]=='f' && tmp[5]=='r' && tmp[6]=='e' && tmp[7]=='e') ||
		           (tmp[4]=='m' && tmp[5]=='o' && tmp[6]=='o' && tmp[7]=='v') ||
		           (tmp[4]=='m' && tmp[5]=='d' && tmp[6]=='a' && tmp[7]=='t')) {		//MOV
			return TK_IMAGE_TYPE_MOV_JPEG;
		}
  #endif
	}

	return TK_IMAGE_TYPE_MAX;
}

TK_IMAGE_TYPE_ENUM image_file_judge_type(INT16S fd)
{
	INT8U *buffer_ptr;
	INT32S len;
	TK_IMAGE_TYPE_ENUM result;
	osEvent msg;
	INT8U  err;

	buffer_ptr = (INT8U *) gp_iram_malloc_align(C_IMAGE_READ_FILE_BUFFER_SIZE, 16);
	if (!buffer_ptr) {
		buffer_ptr = (INT8U *) gp_malloc_align(C_IMAGE_READ_FILE_BUFFER_SIZE, 16);
		if (!buffer_ptr) {
			DBG_PRINT("image_file_judge_type() failed to allocate buffer_ptr\r\n");

			return TK_IMAGE_TYPE_MAX;
		}
	}

	lseek((INT16S)fd, 0, SEEK_SET);
	fs_cmd.fd = fd;
	fs_cmd.result_queue = image_task_fs_queue_a;
	fs_cmd.buf_addr = (INT32U) buffer_ptr;
	fs_cmd.buf_size = C_IMAGE_READ_FILE_BUFFER_SIZE;
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	while (OSQAccept(image_task_fs_queue_a, &err)) ;
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	//while (osMessageGet(image_task_fs_queue_a, 0).value.v) ;
	while(1){
        msg = osMessageGet(image_task_fs_queue_a, 0);
        //if(msg.value.v==0)
        if(msg.status==osOK)
            break;
	}
	#endif
	msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	len = (INT32S) OSQPend(image_task_fs_queue_a, 2000, &err);
	if (err!=OS_NO_ERR || len<4) {
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	msg = osMessageGet(image_task_fs_queue_a, 2000);
	if((msg.status != osEventMessage) || (msg.value.v <4)) {
	#endif
		DBG_PRINT("image_file_judge_type() failed to read data from file\r\n");
		gp_free((void *) buffer_ptr);

		return TK_IMAGE_TYPE_MAX;
	}

	result = image_buffer_judge_type(buffer_ptr);
	gp_free((void *) buffer_ptr);

	return result;
}

INT32S image_parse_header(IMAGE_HEADER_PARSE_STRUCT *parser)
{
	if (!parser) {
		return -1;
	}

	parser->orientation = 1;
	parser->date_time_ptr[0] = 0x0;
	switch (parser->source_type) {
	case TK_IMAGE_SOURCE_TYPE_FILE:				// File system
		parser->image_type = (INT8U) image_file_judge_type((INT16S) parser->image_source);
		switch (parser->image_type) {
		case TK_IMAGE_TYPE_JPEG:
			jpeg_file_parse_header(parser);
			break;

	  #if GPLIB_BMP_EN == 1
		case TK_IMAGE_TYPE_BMP:
			bmp_file_parse_header(parser);
			break;
	  #endif

	  #if GPLIB_GIF_EN == 1
		case TK_IMAGE_TYPE_GIF:
			gif_parse_header(parser);
			break;
	  #endif

		default:
            parser->parse_status = STATUS_FAIL;

			return STATUS_FAIL;
		}
	    break;

	case TK_IMAGE_SOURCE_TYPE_BUFFER:			// SDRAM buffer
		parser->image_type = (INT8U) image_buffer_judge_type((INT8U *) parser->image_source);
		switch (parser->image_type) {
		case TK_IMAGE_TYPE_JPEG:
			jpeg_buffer_parse_header(parser);
			break;

		case TK_IMAGE_TYPE_MOTION_JPEG:
			break;

	  #if GPLIB_BMP_EN == 1
		case TK_IMAGE_TYPE_BMP:
			bmp_buffer_parse_header(parser);
			break;
	  #endif

	  #if GPLIB_GIF_EN == 1
		case TK_IMAGE_TYPE_GIF:
			gif_parse_header(parser);
			break;
	  #endif

		default:
            parser->parse_status = STATUS_FAIL;

			return STATUS_FAIL;
		}
	    break;

	default:
	    DBG_PRINT("Image decode task get an unknow image source type!\r\n");
	    parser->parse_status = STATUS_FAIL;

	    return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S image_decode_and_scale(IMAGE_DECODE_STRUCT *img_decode_struct)
{
	return image_decode_with_clip_and_scale(img_decode_struct, 0, 0);
}

INT32S image_decode_ext_and_scale(IMAGE_DECODE_EXT_STRUCT *img_decode_ext_struct)
{
	INT32U clip_xy, clip_size;

	clip_xy = ((INT32U) img_decode_ext_struct->clipping_start_x<<16) | img_decode_ext_struct->clipping_start_y;
	clip_size = ((INT32U) img_decode_ext_struct->clipping_width<<16) | img_decode_ext_struct->clipping_height;

	return image_decode_with_clip_and_scale((IMAGE_DECODE_STRUCT *) img_decode_ext_struct, clip_xy, clip_size);
}

INT32S image_decode_with_clip_and_scale(IMAGE_DECODE_STRUCT *img_decode_struct, INT32U clip_xy, INT32U clip_size)
{
	TK_FILE_SERVICE_STRUCT flush_fs_cmd;
	INT8U error;
    osEvent result;

	if (!img_decode_struct || !img_decode_struct->output_buffer_width || !img_decode_struct->output_buffer_height) {
		return -1;
	}
	switch (img_decode_struct->source_type) {
	case TK_IMAGE_SOURCE_TYPE_FILE:         // File system
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		while (OSQAccept(image_task_fs_queue_a, &error)) ;
		while (OSQAccept(image_task_fs_queue_b, &error)) ;
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		/*
        while(osMessageGet(image_task_fs_queue_a, 0).value.v);
        while(osMessageGet(image_task_fs_queue_b, 0).value.v);
        */
        while(1){
            result = osMessageGet(image_task_fs_queue_a, 0);
            if(result.status == osOK)
                break;
        }
        while(1){
            result = osMessageGet(image_task_fs_queue_b, 0);
            if(result.status == osOK)
                break;
        }
        #endif
		switch (image_file_judge_type((INT16S) img_decode_struct->image_source)) {
		case TK_IMAGE_TYPE_JPEG:
			jpeg_file_decode_and_scale(img_decode_struct, clip_xy, clip_size);

			// Flush file reading FIFO A
			if (image_task_fs_queue_a) {
				flush_fs_cmd.result_queue = image_task_fs_queue_a;
				msgQSend(fs_msg_q_id, MSG_FILESRV_FLUSH, (void *) &flush_fs_cmd, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);

                #if (_OPERATING_SYSTEM == _OS_UCOS2)
                while (((INT32U) OSQPend(image_task_fs_queue_a, 2000, &error)) != C_FILE_SERVICE_FLUSH_DONE) {
					if (error != OS_NO_ERR) {

                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                //while ( osMessageGet(image_task_fs_queue_a, 2000).value.v != C_FILE_SERVICE_FLUSH_DONE) {
                while(1){
                    result = osMessageGet(image_task_fs_queue_a, 2000);
                    if(result.value.v == C_FILE_SERVICE_FLUSH_DONE)
                        break;
					if (result.status != osEventMessage) {
                #endif
						DBG_PRINT("image_decode_and_scale() failed to flush image_task_fs_queue_a");
						break;
					}
				}
			}

			// Flush file reading FIFO B
			if (image_task_fs_queue_b) {
				flush_fs_cmd.result_queue = image_task_fs_queue_b;
				msgQSend(fs_msg_q_id, MSG_FILESRV_FLUSH, (void *) &flush_fs_cmd, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);

                #if (_OPERATING_SYSTEM == _OS_UCOS2)
                while (((INT32U) OSQPend(image_task_fs_queue_b, 2000, &error)) != C_FILE_SERVICE_FLUSH_DONE) {
					if (error != OS_NO_ERR) {
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                //while ( osMessageGet(image_task_fs_queue_b, 2000).value.v != C_FILE_SERVICE_FLUSH_DONE) {
                while(1){
                    result = osMessageGet(image_task_fs_queue_b, 2000);
                    if(result.value.v == C_FILE_SERVICE_FLUSH_DONE)
                        break;
					if (result.status != osEventMessage) {
                #endif
						DBG_PRINT("image_decode_and_scale() failed to flush image_task_fs_queue_b");
						break;
					}
				}
			}
			break;

	  #if GPLIB_BMP_EN == 1
		case TK_IMAGE_TYPE_BMP:
			bmp_file_decode_and_scale(img_decode_struct);
			break;
	  #endif

	  #if GPLIB_GIF_EN == 1
		case TK_IMAGE_TYPE_GIF:
			gif_decode_and_scale(img_decode_struct);
			break;
	  #endif

		default:
			img_decode_struct->decode_status = STATUS_FAIL;

			return STATUS_FAIL;
		}
	    break;

	case TK_IMAGE_SOURCE_TYPE_BUFFER:         // SDRAM buffer
		switch (image_buffer_judge_type((INT8U *) img_decode_struct->image_source)) {
		case TK_IMAGE_TYPE_JPEG:
			jpeg_buffer_decode_and_scale(img_decode_struct, clip_xy, clip_size);
			break;

		case TK_IMAGE_TYPE_MOTION_JPEG:
			break;

	  #if GPLIB_BMP_EN == 1
		case TK_IMAGE_TYPE_BMP:
			//bmp_buffer_decode_and_scale(img_decode_struct);
			img_decode_struct->decode_status = STATUS_FAIL;
			break;
	  #endif

	  #if GPLIB_GIF_EN == 1
		case TK_IMAGE_TYPE_GIF:
			gif_decode_and_scale(img_decode_struct);
			break;
	  #endif

		default:
			img_decode_struct->decode_status = STATUS_FAIL;

			return STATUS_FAIL;
		}
	    break;

	default:
	    DBG_PRINT("Image decode task get an unknow image source type!\r\n");
	    img_decode_struct->decode_status = STATUS_FAIL;

	    return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S image_encode(IMAGE_ENCODE_STRUCT *img_encode_struct)
{
  #if GPLIB_JPEG_ENCODE_EN == 1
	if (!img_encode_struct ||
		!img_encode_struct->OutputBuf ||
		!img_encode_struct->InputBuf_Y ||
		!img_encode_struct->InputWidth ||
		!img_encode_struct->InputHeight) {
		img_encode_struct->EncodeState = -1;

		return STATUS_FAIL;
	}

	switch (img_encode_struct->EncodeType) {
	case TK_IMAGE_TYPE_JPEG:
		return jpeg_buffer_encode(img_encode_struct);
	}
  #endif

	img_encode_struct->EncodeState = -1;

	return STATUS_FAIL;
}

INT32S image_scale(IMAGE_SCALE_STRUCT *img_scale_struct)
{
	INT32S	scaler_status;
	INT32U  external_line_addr = 0;
#if C_SCALER_L2_EN == 1
    ScalerFormat_t scale;
    ScalerPara_t para;
#endif
	if (!img_scale_struct || !img_scale_struct->output_buffer_width || !img_scale_struct->output_buffer_height) {
		return -1;
	}
#if C_SCALER_L2_EN == 1
    gp_memset((INT8S *)&scale, 0x00, sizeof(scale));
    drv_l2_scaler_init();
    scaler_status = C_SCALER_STATUS_STOP;

    scale.input_format = img_scale_struct->scaler_input_format;
 	scale.input_width = img_scale_struct->input_width;
	scale.input_height = img_scale_struct->input_height;
	scale.input_visible_width = img_scale_struct->input_visible_width;
	scale.input_visible_height = img_scale_struct->input_visible_height;
	scale.input_x_offset = img_scale_struct->input_offset_x;
	scale.input_y_offset = img_scale_struct->input_offset_y;

    scale.input_y_addr = (INT32U)img_scale_struct->input_buf1;
    scale.input_u_addr = 0;
    scale.input_v_addr = 0;
    scale.input_b_y_addr = 0;
    scale.input_b_u_addr = 0;
    scale.input_b_v_addr = 0;


	scale.output_format = img_scale_struct->scaler_output_format;
	scale.output_width = (img_scale_struct->output_buffer_width<<16)/img_scale_struct->output_width_factor;
	scale.output_height = (img_scale_struct->output_buffer_height<<16)/img_scale_struct->output_height_factor;
	scale.output_buf_width = img_scale_struct->output_buffer_width;
	scale.output_buf_height = img_scale_struct->output_buffer_height;
	scale.output_x_offset = 0;
    scale.output_y_addr = (INT32U)img_scale_struct->output_buf1;
    scale.output_u_addr = 0;
    scale.output_v_addr = 0;

	gp_memset((INT8S *)&para, 0x00, sizeof(para));
	para.boundary_mode = 1;
	para.boundary_color = img_scale_struct->out_of_boundary_color;
    para.yuv_type = C_SCALER_CTRL_TYPE_YCBCR;

    scale.fifo_mode = 0;
    scale.scale_mode = C_SCALER_RATIO_USER;

	while (1) {
  		scaler_status = drv_l2_scaler_wait_done(C_USE_SCALER_NUMBER, &scale);
  		if (scaler_status == C_SCALER_STATUS_STOP) {
			scaler_status = drv_l2_scaler_trigger(C_USE_SCALER_NUMBER, ENABLE, &scale, &para);
/*		} else if (scaler_status & C_SCALER_STATUS_DONE) {
			img_scale_struct->status = STATUS_OK;
			break;
*/		} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
			DBG_PRINT("Scaler failed to finish its job\r\n");
			img_scale_struct->status = STATUS_FAIL;
			break;
  		} else {
	  		DBG_PRINT("Un-handled Scaler status!\r\n");
	  		img_scale_struct->status = STATUS_FAIL;
	  		break;
	  	}

        if(scaler_status == C_SCALER_STATUS_STOP) {
            scaler_status = C_SCALER_STATUS_DONE;
            break;
        }

  	}
  	drv_l2_scaler_stop(C_USE_SCALER_NUMBER);


#else
	drv_l1_scaler_init(C_USE_SCALER_NUMBER);

	drv_l1_scaler_input_pixels_set(C_USE_SCALER_NUMBER,img_scale_struct->input_width, img_scale_struct->input_height);
	drv_l1_scaler_input_visible_pixels_set(C_USE_SCALER_NUMBER,img_scale_struct->input_visible_width, img_scale_struct->input_visible_height);
	//scaler_input_addr_set((INT32U) img_scale_struct->input_buf1, NULL, NULL);
	drv_l1_scaler_input_A_addr_set(C_USE_SCALER_NUMBER,(INT32U)img_scale_struct->input_buf1, (INT32U)NULL, (INT32U)NULL);
	drv_l1_scaler_input_offset_set(C_USE_SCALER_NUMBER,img_scale_struct->input_offset_x, img_scale_struct->input_offset_y);
	drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,img_scale_struct->scaler_input_format);

	drv_l1_scaler_output_pixels_set(C_USE_SCALER_NUMBER,img_scale_struct->output_width_factor, img_scale_struct->output_height_factor, img_scale_struct->output_buffer_width, img_scale_struct->output_buffer_height);
	drv_l1_scaler_output_addr_set(C_USE_SCALER_NUMBER,(INT32U) img_scale_struct->output_buf1, (INT32U)NULL, (INT32U)NULL);
	drv_l1_scaler_output_format_set(C_USE_SCALER_NUMBER,img_scale_struct->scaler_output_format);

	drv_l1_scaler_fifo_line_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_FIFO_DISABLE);
	drv_l1_scaler_out_of_boundary_mode_set(C_USE_SCALER_NUMBER,1);			// Use Use color defined in scaler_out_of_boundary_color_set()
	drv_l1_scaler_out_of_boundary_color_set(C_USE_SCALER_NUMBER,img_scale_struct->out_of_boundary_color);
//#if (defined MCU_VERSION) && (MCU_VERSION >= GPL326XX)
	if(img_scale_struct->output_buffer_width > 1024) {
		external_line_addr = (INT32U)gp_malloc_align((img_scale_struct->output_buffer_width - 1024) << 1, 16);
		if(external_line_addr) {
			drv_l1_scaler_external_line_buffer_set(C_USE_SCALER_NUMBER,external_line_addr);
			drv_l1_scaler_line_buffer_mode_set(C_USE_SCALER_NUMBER,C_SCALER_HYBRID_LINE_BUFFER);
		} else {
			drv_l1_scaler_external_line_buffer_set(C_USE_SCALER_NUMBER,external_line_addr);
			drv_l1_scaler_line_buffer_mode_set(C_USE_SCALER_NUMBER,C_SCALER_INTERNAL_LINE_BUFFER);
		}
	}
//#endif
	while (1) {
  		scaler_status = drv_l1_scaler_wait_idle(C_USE_SCALER_NUMBER);
  		if (scaler_status == C_SCALER_STATUS_STOP) {
			drv_l1_scaler_start(C_USE_SCALER_NUMBER,ENABLE);
		} else if (scaler_status & C_SCALER_STATUS_DONE) {
			img_scale_struct->status = STATUS_OK;
			break;
		} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
			DBG_PRINT("Scaler failed to finish its job\r\n");
			img_scale_struct->status = STATUS_FAIL;
			break;
  		} else {
	  		DBG_PRINT("Un-handled Scaler status!\r\n");
	  		img_scale_struct->status = STATUS_FAIL;
	  		break;
	  	}
  	}
  	drv_l1_scaler_stop(C_USE_SCALER_NUMBER);

	if(external_line_addr) {
		gp_free((void *)external_line_addr);
	}
#endif
  	if (scaler_status & C_SCALER_STATUS_DONE) {
  		cache_invalid_range((INT32U) img_scale_struct->output_buf1, (img_scale_struct->output_buffer_width*img_scale_struct->output_buffer_height)<<1);

  		return STATUS_OK;
  	}
  	return STATUS_FAIL;
}
#endif//#if (defined APP_IMAGE_CODEC_EN ) && (APP_IMAGE_CODEC_EN == 1)
