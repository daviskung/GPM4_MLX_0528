#include "task_video_decoder.h"
#include "gplib_jpeg.h"
#include "drv_l1_jpeg.h"
#include "drv_l2_h264dec.h"

#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECODER_EN == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define C_VIDEO_DECODE_STACK_SIZE		512
#define C_VIDEO_Q_ACCEPT_MAX			5

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef enum
{
	MSG_VID_DEC_ONE_FRAME = 0x3000,
	MSG_VID_DEC_START,
	MSG_VID_DEC_RESTART,
	MSG_VID_DEC_STOP,
	MSG_VID_DEC_NTH,
	MSG_VID_DEC_EXIT
} MSG_VID_DEC_ENUM;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
INT32U	video_decode_stack[C_VIDEO_DECODE_STACK_SIZE];
osMessageQId vid_dec_q;
osMessageQId vid_dec_ack_m;
void *vid_dec_q_buffer[C_VIDEO_Q_ACCEPT_MAX];

INT32U (*viddec_display)(INT16U w, INT16U h, INT32U disp_buf);

INT32S video_decode_task_start(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(vid_dec_q, MSG_VID_DEC_START, vid_dec_ack_m, 0, msg, err);
Return:
	return nRet;
}

INT32S video_decode_task_restart(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(vid_dec_q, MSG_VID_DEC_RESTART, vid_dec_ack_m, 0, msg, err);
Return:
	return nRet;
}

INT32S video_decode_task_stop(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(vid_dec_q, MSG_VID_DEC_STOP, vid_dec_ack_m, 0, msg, err);
Return:
	return nRet;
}

INT32S video_decode_task_nth_frame(void)
{
	INT8U  err;
	INT32S nRet, msg;

	nRet = STATUS_OK;
	SEND_MESSAGE(vid_dec_q, MSG_VID_DEC_NTH, vid_dec_ack_m, 0, msg, err);
Return:
	return nRet;
}

INT32S video_decode_task_one_frame(void)
{
	osStatus err;
  INT32U msg_id;

  msg_id = MSG_VID_DEC_ONE_FRAME;
	err = osMessagePut(vid_dec_q, (INT32U)&msg_id, 0);
	if(err != osOK) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S  video_decode_task_create(INT8U proi)
{
	INT8U  err;
	INT32S nRet;
	osThreadId id;
	osMessageQDef(vid_dec_q, C_VIDEO_Q_ACCEPT_MAX, void*);
	osMessageQDef(vid_dec_ack_m, 1, void*);
	osThreadDef(video_decode_task_entry, proi, 0, C_VIDEO_DECODE_STACK_SIZE*4);

	vid_dec_q = osMessageCreate(osMessageQ(vid_dec_q), NULL);
	if(!vid_dec_q) {
		RETURN(STATUS_FAIL);
	}

	vid_dec_ack_m = osMessageCreate(osMessageQ(vid_dec_ack_m), NULL);
	if(!vid_dec_ack_m) {
		RETURN(STATUS_FAIL);
	}

	id = osThreadCreate(osThread(video_decode_task_entry), NULL);
	if(id == 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S video_decode_task_del(INT8U proi)
{
	INT8U  err;
	INT32S msg,nRet;

	nRet = STATUS_OK;
	SEND_MESSAGE(vid_dec_q, MSG_VID_DEC_EXIT, vid_dec_ack_m, 0, msg, err);
Return:
	OSQFlush(vid_dec_q);
	vQueueDelete(vid_dec_q);
    vQueueDelete(vid_dec_ack_m);
	//OSQDel(vid_dec_q, OS_DEL_ALWAYS, &err);
	//OSMboxDel(vid_dec_ack_m, OS_DEL_ALWAYS, &err);
	return STATUS_OK;
}

static void video_decode_task_end(INT32U param)
{
#if MJPEG_DECODE_ENABLE == 1
	if(p_vid_dec_para->video_format == C_MJPG_FORMAT) {
		mjpeg_decode_stop_all(p_vid_dec_para->scaler_flag);
	} else
#endif
    if (p_vid_dec_para->video_format == C_H264_FORMAT)
    {
        if (param)
            gp_h264dec_release((void*)param);
        pscaler_stop();
    }
#if MPEG4_DECODE_ENABLE == 1
	else {
        INT32U deblock_iram_addr = (INT32U)param;
		if(p_vid_dec_para->scaler_flag) {
			scaler_size_wait_done();
		}

		mpeg4_decode_stop();
		if(p_vid_dec_para->deblock_flag) {
			drvl1_mp4_deblock_stop();
			gp_free((void*)deblock_iram_addr);
		}
	}
#endif
}

void video_decode_task_entry(void const *parm)
{
	INT8U  success_flag, init_flag, nth_flag, end_flag, skip_flag;
	INT8U  err;
	INT16U *pwdata, width, height, jpeg_yuv_mode;
	INT32S coding_type=0, status, error_id;
	INT32U raw_data_addr=0, decode_addr, display_addr;
#if MPEG4_DECODE_ENABLE == 1
	INT8U time_inc_len, quant;
	INT32U deblock_addr, deblock_iram_addr, scaler_in_addr, refer_addr, old_scaler_out_addr, scaler_out_addr;
#endif
	INT32U msg_id, offset=0;
	INT32U decode_start, decode_end, decode_count;
	INT32U fps;

	INT64S delta_Tv=0, old_delta_Tv=0, temp;
	long Duration, size;
	//OS_CPU_SR cpu_sr;
	mjpeg_info mjpeg;
#if MPEG4_DECODE_ENABLE == 1
	INT8U  mpeg4_type;
	INT32U key_word;
	ScalerFormat_t scale;
#endif
    void* h264_dec = NULL;
    INT32U decode_time, t1, t2;

	nth_flag = 0;
	while(1)
	{
		osEvent event;
    	event = osMessageGet(vid_dec_q, osWaitForever);
    	if (event.status != osEventMessage) {
        	continue;
        }

		switch(event.value.v)
		{
		case MSG_VID_DEC_ONE_FRAME:
			success_flag = display_addr = 0;
			//check video data ready
			if(!raw_data_addr || size <= 0) {
				//check end or error
				if(vid_dec_get_status() & C_VIDEO_DECODE_ERR) {
					end_flag = 1;
				} else if(MultiMediaParser_IsEOV(p_vid_dec_para->media_handle) == 1) {
					end_flag = 1;
				} else {
					end_flag = 0;
				}

				if(end_flag) {
					DEBUG_MSG("\r\nVidDecEnd.\r\n");
					if (p_vid_dec_para->video_format == C_H264_FORMAT)
					{
                        video_decode_task_end((INT32U)h264_dec);
                        h264_dec = 0;
                    }
					else
					#if MPEG4_DECODE_ENABLE == 1
							video_decode_task_end(deblock_iram_addr);
					#else
					video_decode_task_end(0);
					#endif
					OSQFlush(vid_dec_q);
					vid_dec_end_callback(C_VIDEO_DECODE_PLAYING);
					continue;
				} else {
					goto __VidDecOneFrameEnd;
				}
			}

			//check sync
			taskENTER_CRITICAL();
			temp = (INT64S)(p_vid_dec_para->tv - p_vid_dec_para->Tv);
			taskEXIT_CRITICAL();
			if(temp > p_vid_dec_para->time_range) {
			#if DROP_MENTHOD == PARSER_DROP
				if(p_vid_dec_para->fail_cnt < 2) {
					p_vid_dec_para->fail_cnt++;
					MultiMediaParser_SetFrameDropLevel(p_vid_dec_para->media_handle, p_vid_dec_para->fail_cnt);
					DEBUG_MSG("FailCnt = %d\r\n", p_vid_dec_para->fail_cnt);
				}
			#elif DROP_MENTHOD == DECODE_DROP
				p_vid_dec_para->fail_cnt = 1;
				DEBUG_MSG("FailCnt = 1\r\n");
			#endif
			} else {
			#if DROP_MENTHOD == PARSER_DROP
				if(p_vid_dec_para->fail_cnt > 0) {
					p_vid_dec_para->fail_cnt--;
					MultiMediaParser_SetFrameDropLevel(p_vid_dec_para->media_handle, p_vid_dec_para->fail_cnt);
					DEBUG_MSG("FailCnt = %d\r\n", p_vid_dec_para->fail_cnt);
				}
			#elif DROP_MENTHOD == DECODE_DROP
				p_vid_dec_para->fail_cnt = 0;
			#endif
			}

			switch(p_vid_dec_para->video_format)
			{
#if MJPEG_DECODE_ENABLE== 1
			case C_MJPG_FORMAT:
			#if DROP_MENTHOD == DECODE_DROP
				if(p_vid_dec_para->fail_cnt) {
					goto __VidDecOneFrameEnd;
				}
			#endif
				//mjpeg decode one frame
				coding_type = C_I_VOP;
				display_addr = decode_addr = vid_dec_get_next_vid_buffer();
				mjpeg.scaler_mode = p_vid_dec_para->scaler_flag;
				mjpeg.raw_data_addr = raw_data_addr;
				mjpeg.raw_data_size = size;
				mjpeg.jpeg_yuv_mode = jpeg_yuv_mode;
				mjpeg.jpeg_valid_w = width;
				mjpeg.jpeg_valid_h = height;
				mjpeg.output_addr = decode_addr + offset;
				mjpeg.output_format = p_vid_dec_para->video_decode_out_format;
				mjpeg.jpeg_output_w = p_vid_dec_para->image_output_width;
				mjpeg.jpeg_output_h = p_vid_dec_para->image_output_height;
				mjpeg.jpeg_output_buffer_w = p_vid_dec_para->buffer_output_width;
				mjpeg.jpeg_output_buffer_h = p_vid_dec_para->buffer_output_height;
				mjpeg.boundary_color = 0x008080;
				t1 = xTaskGetTickCount();
				if(p_vid_dec_para->scaler_flag) {
					status = mjpeg_decode_and_scaler(&mjpeg);
				} else {
					status = mjpeg_decode_without_scaler(&mjpeg);
				}
				t2 = xTaskGetTickCount();
				decode_time += (t2-t1);

				if(status < 0) {
					DEBUG_MSG("JpegDecFail = %d\r\n", status);
					display_addr = 0;
				}
				decode_count++;

				goto __VidDecOneFrameEnd;
				break;
#endif
			#if MPEG4_DECODE_ENABLE == 1
			case C_XVID_FORMAT:
			case C_M4S2_FORMAT:
				//get mpeg4 bitstream coding type
				coding_type = vid_dec_paser_bit_stream((INT8U*)raw_data_addr, size, &width, &height, &time_inc_len, &quant);
				if(init_flag) {
					break;
				}

				if(((coding_type == C_I_VOP)||(coding_type == C_P_VOP)) && time_inc_len) {
					init_flag = 1;
					mpeg4_decode_config(mpeg4_type, p_vid_dec_para->mpeg4_decode_out_format, width, height, time_inc_len - 1);
				} else if(coding_type & ERROR_00) {
					video_decode_task_end(deblock_iram_addr);
					vid_dec_end_callback(C_VIDEO_DECODE_PLAYING);
					p_vid_dec_para->pend_cnt = p_vid_dec_para->post_cnt;
					OSQFlush(vid_dec_q);
					continue;
				} else {
					goto __VidDecOneFrameEnd;
				}
				break;

			case C_H263_FORMAT:
				//get h263 bitstream coding type
				coding_type = vid_dec_paser_h263_bit_stream((INT8U*)raw_data_addr, &width, &height, &key_word);
				drvl1_mp4_decode_match_code_set(0x03, key_word);
				if(init_flag) {
					break;
				}

				if((coding_type & ERROR_00) == 0) {
					init_flag = 1;
					mpeg4_decode_config(mpeg4_type, p_vid_dec_para->mpeg4_decode_out_format, width, height, 0);
				} else {
					goto __VidDecOneFrameEnd;
				}
				break;
			#endif

			#if S263_DECODE_ENABLE == 1
			case C_SOREN_H263_FORMAT:
				//get soren h263 bitstream coding type
				coding_type = vid_dec_paser_sorenson263_bit_stream((INT8U*)raw_data_addr, &width, &height, &key_word);
				drvl1_mp4_decode_match_code_set(0x03, key_word);
				if(init_flag) {
					break;
				}

				if((coding_type & ERROR_00) == 0) {
					init_flag = 1;
					mpeg4_decode_config(mpeg4_type, p_vid_dec_para->mpeg4_decode_out_format, width, height, coding_type);
				} else {
					goto __VidDecOneFrameEnd;
				}
				break;
			#endif
			case C_H264_FORMAT:
                {
                    gp_h264dec_in param;
                    param.BitstreamAddr = raw_data_addr;
                    param.BitstreamBaseAddr = raw_data_addr;
                    param.BitstreamMaxAddr = raw_data_addr + size;
                    param.BitstreamMaxAddr = (param.BitstreamMaxAddr + 0x3F) >> 6 << 6; //64 byte alignment for GP22 H.264
                    t1 = xTaskGetTickCount();
                    gp_h264dec_decode(h264_dec, &param);
                    if (0 > gp_h264dec_decode_wait(h264_dec, (INT32U*)&coding_type, 2000))
                    {
                    DBG_PRINT("decode time out\r\n");
                        //while(1);
                    }
                    t2 = xTaskGetTickCount();
                    decode_time += (t2-t1);

                    decode_count++;
                }
                break;
			}

			#if MPEG4_DECODE_ENABLE == 1
			switch(coding_type)
			{
			case C_I_VOP:
			#if DROP_MENTHOD == DECODE_DROP
				skip_flag = 0;
			#endif

			case C_P_VOP:
			#if DROP_MENTHOD == DECODE_DROP
				if(skip_flag || (p_vid_dec_para->fail_cnt && (coding_type == C_P_VOP))) {
					skip_flag = 1;
					break;
				}
			#endif
				//decode mpeg4 I/P frame
				refer_addr = decode_addr;
				old_scaler_out_addr = scaler_out_addr;
				if(p_vid_dec_para->deblock_flag) {
					deblock_addr = vid_dec_get_next_deblock_buffer();
					drvl1_mp4_deblock_set(quant, 1);
					drvl1_mp4_deblock_start(deblock_addr);
				}

				decode_addr = vid_dec_get_next_vid_buffer();
				mpeg4_decode_start(raw_data_addr, decode_addr, refer_addr);
				status = mpeg4_wait_idle(TRUE);
 				if(status != C_MP4_STATUS_END_OF_FRAME) {
 					//re-init
					DEBUG_MSG("Mpeg4DecFail = 0x%x\r\n", status);
					mpeg4_decode_init();
					mpeg4_decode_config(mpeg4_type, p_vid_dec_para->mpeg4_decode_out_format, width, height, time_inc_len - 1);
					display_addr = 0;
					goto __VidDecOneFrameEnd;
				} else {
					mpeg4_decode_stop();
				}

				//check deblock set
				if(p_vid_dec_para->scaler_flag && p_vid_dec_para->deblock_flag) {
					scaler_in_addr = deblock_addr;
					display_addr = old_scaler_out_addr;
				} else if(p_vid_dec_para->scaler_flag) {
					scaler_in_addr = decode_addr;
					display_addr = old_scaler_out_addr;
				} else if(p_vid_dec_para->deblock_flag) {
					display_addr = deblock_addr;
				} else {
					display_addr = decode_addr;
				}

				//scale mpeg4 decode out frame
				if(p_vid_dec_para->scaler_flag) {
					// wait scale done
					drv_l2_scaler_wait_done(SCALER_0, &scale);

					scaler_out_addr = vid_dec_get_next_scaler_buffer();
					scale.input_format = p_vid_dec_para->video_decode_in_format;
					scale.input_width = width;
					scale.input_height = height;
					scale.input_visible_width = 0;
					scale.input_visible_height = 0;
					scale.input_x_offset = 0;
					scale.input_y_offset = 0;

					scale.input_y_addr = scaler_in_addr;
					scale.input_u_addr = 0;
					scale.input_v_addr = 0;

					scale.output_format = p_vid_dec_para->video_decode_out_format;
					scale.output_width = p_vid_dec_para->image_output_width;
					scale.output_height = p_vid_dec_para->image_output_height;
					scale.output_buf_width = p_vid_dec_para->buffer_output_width;
					scale.output_buf_height = p_vid_dec_para->buffer_output_height;
					scale.output_x_offset = 0;

					scale.output_y_addr = scaler_out_addr + offset;
					scale.output_u_addr = 0;
					scale.output_v_addr = 0;

					scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
					scale.scale_mode = p_vid_dec_para->scaler_flag;
					scale.digizoom_m = 10;
					scale.digizoom_n = 10;

					drv_l2_scaler_trigger(SCALER_0, DISABLE, &scale, 0x008080);
				}
				break;

			case C_N_VOP:
				//skip mpeg4 N frame
				goto __VidDecOneFrameEnd;
				break;

			case C_UNKNOW_VOP:
				//skip mpeg4 unknow frame
				break;

			default:
				//skip B frame or others
				goto __VidDecOneFrameEnd;
				break;
			}
			#endif

		__VidDecOneFrameEnd:
			DEBUG_MSG("%d", coding_type);
			success_flag = 1;
			if(display_addr && !nth_flag) {
				if(viddec_display) {
					viddec_display(p_vid_dec_para->buffer_output_width, p_vid_dec_para->buffer_output_height, display_addr);
				}
			}

			if(success_flag) {
				//add video decode time
				if(p_vid_dec_para->scaler_flag && (p_vid_dec_para->video_format != C_MJPG_FORMAT)) {
					taskENTER_CRITICAL();
					p_vid_dec_para->Tv += old_delta_Tv;
					taskEXIT_CRITICAL();
					old_delta_Tv = delta_Tv;
				} else {
					taskENTER_CRITICAL();
					p_vid_dec_para->Tv += delta_Tv;
					taskEXIT_CRITICAL();
				}
			}

			//allow next frame input
			p_vid_dec_para->pend_cnt++;
			taskENTER_CRITICAL();
			temp = p_vid_dec_para->tv - p_vid_dec_para->Tv;
			if((temp >= 0) && (p_vid_dec_para->post_cnt == p_vid_dec_para->pend_cnt) && (raw_data_addr)) {
                msg_id = MSG_VID_DEC_ONE_FRAME;
				if (osMessagePut(vid_dec_q, (INT32U)&msg_id, 0) == osOK) {
					p_vid_dec_para->post_cnt++;
				}
			}
			taskEXIT_CRITICAL();

			// get video raw data
			raw_data_addr = (INT32U)MultiMediaParser_GetVidBuf(p_vid_dec_para->media_handle, &size, &Duration, &error_id);
			if (error_id < 0)
				break;
			size += raw_data_addr & 3;	//align and size increase
			raw_data_addr -= raw_data_addr & 3;	//align addr
			//while(error_id<0);

			//update current time & frame
			p_vid_dec_para->video_cur_time = MultiMediaParser_GetVidCurTime(p_vid_dec_para->media_handle) >> 8;
			p_vid_dec_para->video_cur_frame = MultiMediaParser_GetVidCurNumberOfVideo(p_vid_dec_para->media_handle);

			//caculate video frame delta time
			if(p_wave_info) {
           		delta_Tv = (INT64S)Duration * p_wave_info->nSamplesPerSec*2;
       		} else {
       			p_vid_dec_para->n = 1;
       			delta_Tv = (INT64S)Duration * TIME_BASE_TICK_RATE;
       		}
       		//if (delta_Tv)
                //DBG_PRINT("parse one frame [%lld] (tv = %lld, TV = %lld)\r\n", delta_Tv, p_vid_dec_para->tv, p_vid_dec_para->Tv);
			break;

		case MSG_VID_DEC_START:
			decode_addr = 0;
			#if MPEG4_DECODE_ENABLE == 1
			refer_addr = scaler_in_addr = deblock_addr = deblock_iram_addr = old_scaler_out_addr = scaler_out_addr = 0;
			#endif
			offset = p_vid_dec_para->xoffset * 2 + p_vid_dec_para->yoffset * p_vid_dec_para->buffer_output_width * 2;
			decode_start = xTaskGetTickCount();
			decode_time = 0;
		case MSG_VID_DEC_RESTART:
			DEBUG_MSG("MSG_VID_DEC_RESTART\r\n");
			#if MPEG4_DECODE_ENABLE == 1
			time_inc_len = 0;
			quant = 0;
			#endif
			init_flag = skip_flag = 0;
			delta_Tv = old_delta_Tv = 0;
			p_vid_dec_para->fail_cnt = 0;
			p_vid_dec_para->pend_cnt = p_vid_dec_para->post_cnt = 0;
			decode_count = 0;

			//fre-latch video raw data
			while(1) {
				raw_data_addr = (INT32U)MultiMediaParser_GetVidBuf(p_vid_dec_para->media_handle, &size, &Duration, &error_id);
				if (error_id <0)
					goto __VidDecStartFail;
				size += raw_data_addr & 3;	//align and size increase
				raw_data_addr -= raw_data_addr & 3;	//align addr
				//while(error_id<0);

				//update current time & frame
				p_vid_dec_para->video_cur_time = MultiMediaParser_GetVidCurTime(p_vid_dec_para->media_handle) >> 8;
				p_vid_dec_para->video_cur_frame = MultiMediaParser_GetVidCurNumberOfVideo(p_vid_dec_para->media_handle);

				if(nth_flag) {
					if(raw_data_addr && (size >= 10)) {
						break;
					}
				} else {
					if(raw_data_addr) {
						break;
					}
				}

				//check end or error
				if(vid_dec_get_status() & C_VIDEO_DECODE_ERR) {
					goto __VidDecStartFail;
				} else if(MultiMediaParser_IsEOV(p_vid_dec_para->media_handle) == 1) {
					goto __VidDecStartFail;
				} else {
					osDelay(1);
				}
			}

			//caculate video frame delta time
			if(p_wave_info) {
           		delta_Tv = (INT64S)Duration * p_wave_info->nSamplesPerSec*2;
       		} else {
       			p_vid_dec_para->n = 1;
       			delta_Tv = (INT64S)Duration * TIME_BASE_TICK_RATE;
       		}

			switch(p_vid_dec_para->video_format)
			{
			#if MJPEG_DECODE_ENABLE == 1
			case C_MJPG_FORMAT:
				 //Fix if jpeg file have no huffman table.
				//gplib_jpeg_default_huffman_table_load(); //move into mjpeg_decode_get_size()
				coding_type = mjpeg_decode_get_size(raw_data_addr, size, &width, &height, &jpeg_yuv_mode);
				if(coding_type >= 0) {
					init_flag = 1;
				}
				break;
			#endif
			#if MPEG4_DECODE_ENABLE == 1
			case C_XVID_FORMAT:
			case C_M4S2_FORMAT:
				mpeg4_decode_init();
				mpeg4_type = C_MPEG4_CODEC;
				vid_dec_parser_bit_stream_init();
				if(p_vid_dec_para->video_format == C_XVID_FORMAT) {
					coding_type = vid_dec_paser_bit_stream((INT8U*)raw_data_addr, size, &width, &height, &time_inc_len, &quant);
				} else if(p_vid_dec_para->video_format == C_M4S2_FORMAT) {
					coding_type = vid_dec_paser_bit_stream((INT8U*)((INT32U)p_bitmap_info + sizeof(GP_BITMAPINFOHEADER)),
														g_bitmap_info_len - sizeof(GP_BITMAPINFOHEADER),
														&width, &height, &time_inc_len, &quant);
				}

				if(((coding_type == C_I_VOP)||(coding_type == C_P_VOP)) && time_inc_len) {
					init_flag = 1;
					DEBUG_MSG("TIL=%d\r\n", time_inc_len);
					mpeg4_decode_config(mpeg4_type, p_vid_dec_para->mpeg4_decode_out_format, width, height, time_inc_len - 1);
				} else if(coding_type & ERROR_00) {
					goto __VidDecStartFail;
				} else if(nth_flag && !time_inc_len) {
					goto __VidDecStartFail;
				}
				break;

			case C_H263_FORMAT:
				mpeg4_decode_init();
				mpeg4_type = C_H263_CODEC;
				coding_type = vid_dec_paser_h263_bit_stream((INT8U*)raw_data_addr, &width, &height, &key_word);
				if((coding_type & ERROR_00) == 0) {
					init_flag = 1;
					mpeg4_decode_config(mpeg4_type, p_vid_dec_para->mpeg4_decode_out_format, width, height, 0);
				}
				break;
			#endif

			#if S263_DECODE_ENABLE == 1
			case C_SOREN_H263_FORMAT:
				mpeg4_decode_init();
				mpeg4_type = C_SORENSON_H263_CODEC;
				coding_type = vid_dec_paser_sorenson263_bit_stream((INT8U*)raw_data_addr, &width, &height, &key_word);
				if((coding_type & ERROR_00) == 0) {
					init_flag = 1;
					mpeg4_decode_config(mpeg4_type, p_vid_dec_para->mpeg4_decode_out_format, width, height, coding_type);
				}
				break;
			#endif
            case C_H264_FORMAT:
                if (!vid_dec_parser_h264_bit_stream((INT8U*)raw_data_addr, size, &width, &height))
                {
                    INT32U out_div = 0, out_width, out_height, crop_width, crop_height;
                    gp_h264dec_cfg cfg;
                    init_flag = 1;
                    cfg.width = width;
                    cfg.height = height;

                    out_width = (width + 0xf) & ~0xf;
                    out_height = (height + 0xf) & ~0xf;
                    out_div = out_width/p_vid_dec_para->image_output_width;
                    if (out_div > out_height/p_vid_dec_para->image_output_height)
                        out_div = out_height/p_vid_dec_para->image_output_height;

                    if (out_div >= 4)
                        out_div = 4;
                    else if (out_div >= 2)
                        out_div = 2;
                    else
                        out_div = 0;

                    cfg.out_div = out_div;
                    cfg.HDMI = 0;
                    h264_dec = gp_h264dec_init(&cfg);

                    if (out_div){
                        out_width = out_width/out_div;
                        out_height = out_height/out_div;
                        out_width = (out_width + 0xf) & ~0xf;
						out_height = (out_height + 0xf) & ~0xf;
						crop_width = width/out_div;
						crop_height = height/out_div;
                    }
                    else
                    {
                        crop_width = width;
                        crop_height = height;
                    }
                     pscaler_start(crop_width, crop_height, out_width, out_height);
                }
                break;
			default:
				while(1);
			}

			//check header resolution is equal raw data size or not.
			if(init_flag && (p_bitmap_info->biWidth != width || p_bitmap_info->biHeight != height)) {
				pwdata = (INT16U*)&p_bitmap_info->biWidth;
				*pwdata = width;
				pwdata = (INT16U*)&p_bitmap_info->biHeight;
				*pwdata = height;
				if(vid_dec_memory_realloc() < 0) {
					goto __VidDecStartFail;
				}
			}

		#if MPEG4_DECODE_ENABLE == 1
			if(p_vid_dec_para->deblock_flag && (p_vid_dec_para->video_format != C_MJPG_FORMAT)) {
				deblock_iram_addr = (INT32U) gp_iram_malloc_align(width*2*2, 4);
				if(!deblock_iram_addr) {
					p_vid_dec_para->deblock_flag = 0;
				} else {
					drvl1_mp4_deblock_iram_set(deblock_iram_addr);
				}
			}
		#endif
			DEBUG_MSG("VidDecDeblock = 0x%x\r\n", p_vid_dec_para->deblock_flag);
			DEBUG_MSG("VidDecScaler = 0x%x\r\n", p_vid_dec_para->scaler_flag);
			if(nth_flag) {
				break;
			}
			msg_id = ACK_OK;
            osMessagePut(vid_dec_ack_m, (INT32U)&msg_id, 0);
			//OSMboxPost(vid_dec_ack_m, (void*)ACK_OK);
			break;

		__VidDecStartFail:
			DEBUG_MSG("VideoDecodeStartFail!!!\r\n");
			nth_flag = 0;
			OSQFlush(vid_dec_q);
			msg_id = ACK_FAIL;
            osMessagePut(vid_dec_ack_m, (INT32U)&msg_id, 0);
			//OSMboxPost(vid_dec_ack_m, (void*)ACK_FAIL);
			break;

		case MSG_VID_DEC_STOP:
            decode_end = xTaskGetTickCount();
			DEBUG_MSG("MSG_VID_DEC_STOP\r\n");
			nth_flag = 0;
			if (p_vid_dec_para->video_format == C_H264_FORMAT)
                video_decode_task_end((INT32U)h264_dec);
            else
			#if MPEG4_DECODE_ENABLE == 1
                video_decode_task_end(deblock_iram_addr);
			#else
			video_decode_task_end(0);
			#endif
			OSQFlush(vid_dec_q);
			msg_id = ACK_OK;
            osMessagePut(vid_dec_ack_m, (INT32U)&msg_id, 0);
			//OSMboxPost(vid_dec_ack_m, (void*)ACK_OK);
			//fps = (float)((decode_count)*1000)/(float)(decode_end - decode_start);
			//DBG_PRINT("FPS = %f\r\n", fps);
			fps = (decode_count*1000*100) / (decode_end - decode_start);
            DBG_PRINT("FPS = %d.%02d \r\n", fps/100, fps - 100*(fps/100));
			fps = (decode_count*1000*100) / decode_time;
            DBG_PRINT("decode_time = %d fps = %d.%02d \r\n", decode_time, fps/100, fps - 100*(fps/100));
			break;

		case MSG_VID_DEC_NTH:
            msg_id = MSG_VID_DEC_START;
			if(osMessagePut(vid_dec_q, (INT32U)&msg_id, 0) != osOK) {
				goto __VidDecNthFail;
			}

            msg_id = MSG_VID_DEC_ONE_FRAME;
			if(osMessagePut(vid_dec_q, (INT32U)&msg_id, 0) != osOK) {
				goto __VidDecNthFail;
			}

            msg_id = MSG_VID_DEC_STOP;
			if(osMessagePut(vid_dec_q, (INT32U)&msg_id, 0) != osOK) {
				goto __VidDecNthFail;
			}
			nth_flag = 1;
			break;

		__VidDecNthFail:
			nth_flag = 0;
			OSQFlush(vid_dec_q);
			msg_id = ACK_FAIL;
			osMessagePut(vid_dec_ack_m, (INT32U)&msg_id, 0);
			//OSMboxPost(vid_dec_ack_m, (void*)ACK_FAIL);
			break;

		case MSG_VID_DEC_EXIT:
			DEBUG_MSG("MSG_VID_DEC_EXIT\r\n");
			msg_id = ACK_OK;
			osMessagePut(vid_dec_ack_m, (INT32U)&msg_id, 0);
			//OSMboxPost(vid_dec_ack_m, (void*)ACK_OK);
			osThreadTerminate(NULL);
			break;
		}
	}
}

#endif//#if (defined APP_VIDEO_DECODER_EN) && (APP_VIDEO_DECDOER_EN == 1)

