#include "cmsis_os.h"
#include "cp_camera_processor.h"
#include "drv_l1_pscaler.h"
#include "gplib_jpeg_encode.h"
#include "drv_l1_jpeg.h"
#include "jpeg_header.h"

#define ENC_INPUT_QUEUE_MAX 6
#define ENC_VID_CACHE_SIZE	0x300000
#define ENC_VID_MAX_BLK_SIZE 0x80000

osMessageQId enc_task_q = NULL;
osMessageQId enc_task_ack_m = NULL;
osMessageQId scale_task_q = NULL;
osMessageQId scale_task_ack_m = NULL;
CP_ENCODER_INFO encinfo;
INT32U pscaler_running = 0;

static void cp_encode_task_entry(void const *para);
static void cp_scale_task_entry(void const *para);
static INT32S cp_pscaler_set();

INT32S cp_encode_init(ENCCFG* cfg)
{
	gp_memset(&encinfo, 0, sizeof(CP_ENCODER_INFO));

	encinfo.cfg = *cfg;

	if (0 > vfm_init(&encinfo.vfm, 0, ENC_VID_CACHE_SIZE, ENC_VID_MAX_BLK_SIZE))
		return STATUS_FAIL;

	if (encinfo.cfg.scale_out)
		cp_pscaler_set();

	encinfo.mtx = osMutexCreate(NULL);

	jpeg_header_generate(JPEG_IMG_FORMAT_422, cfg->quality, cfg->enc_width, cfg->enc_height);

	encinfo.header_size = jpeg_header_get_size();

	cp_error &= ~CP_ERROR_ENCODE_ERROR;

	return STATUS_OK;
}

void cp_encode_close()
{
	vfm_close(&encinfo.vfm);
	vQueueDelete(encinfo.mtx);

	gp_memset(&encinfo, 0, sizeof(CP_ENCODER_INFO));
}

INT32S cp_encode_task_create()
{
	osThreadId id1 = NULL, id2 = NULL;
	osThreadDef_t encode_task = { "encode_task", cp_encode_task_entry, osPriorityRealtime, 1, 2048 };
	osThreadDef_t scale_task = { "scale_task", cp_scale_task_entry, osPriorityNormal, 1, 2048 };

	osMessageQDef_t videnc_q = {ENC_INPUT_QUEUE_MAX, sizeof(INT32U), 0};

	enc_task_q = osMessageCreate(&videnc_q, NULL);
	if(enc_task_q == 0) {
		goto _Exit;
	}

	osMessageQDef_t vidack_q = {1, sizeof(INT32U), 0};

	enc_task_ack_m = osMessageCreate(&vidack_q, NULL);
	if(enc_task_ack_m == 0) {
		goto _Exit;
	}

	osMessageQDef_t vidscale_q = {ENC_INPUT_QUEUE_MAX, sizeof(INT32U), 0};

	scale_task_q = osMessageCreate(&vidscale_q, NULL);
	if(scale_task_q == 0) {
		goto _Exit;
	}

	osMessageQDef_t scaleack_q = {1, sizeof(INT32U), 0};

	scale_task_ack_m = osMessageCreate(&scaleack_q, NULL);
	if(scale_task_ack_m == 0) {
		goto _Exit;
	}

	id1 = osThreadCreate(&encode_task, (void *)NULL);
	if (id1 == 0)
		goto _Exit;

	id2 = osThreadCreate(&scale_task, (void *)NULL);
	if (id2 == 0)
		goto _Exit;

	return STATUS_OK;
_Exit:
	if (enc_task_q) vQueueDelete(enc_task_q);
	if (enc_task_ack_m) vQueueDelete(enc_task_ack_m);
	if (scale_task_q) vQueueDelete(scale_task_q);
	if (scale_task_ack_m) vQueueDelete(scale_task_ack_m);
	enc_task_q = enc_task_ack_m = scale_task_q = scale_task_ack_m = 0;

	if (id1) osThreadTerminate(id1);
	if (id2) osThreadTerminate(id2);

	return STATUS_FAIL;
}

INT32S cp_encode_task_del()
{
	INT32U msg_id;
	osEvent event;

	event = osMessageGet(enc_task_ack_m, 0);
	msg_id = MSG_ENC_TASK_EXIT;
	osMessagePut(enc_task_q, (INT32U)&msg_id, osWaitForever);
	event = osMessageGet(enc_task_ack_m, osWaitForever);

	event = osMessageGet(scale_task_ack_m, 0);
	msg_id = MSG_SCALE_TASK_EXIT;
	osMessagePut(scale_task_q, (INT32U)&msg_id, osWaitForever);
	event = osMessageGet(scale_task_ack_m, osWaitForever);

	vQueueDelete(enc_task_q);
	vQueueDelete(enc_task_ack_m);
	vQueueDelete(scale_task_q);
	vQueueDelete(scale_task_ack_m);
	enc_task_q = enc_task_ack_m = scale_task_q = scale_task_ack_m = 0;

	return STATUS_OK;
}

static void pscaler_out_isr(INT32U event)
{
	if (event & PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR)
		DBG_PRINT("PSCALER_B overflow\r\n");
	if (event & PIPELINE_SCALER_STATUS_FRAME_DONE)
	{
		drv_l1_pscaler_stop(PSCALER_B);
		pscaler_running = 0;
	}
	//DBG_PRINT("event=%x\r\n", event);
}

static INT32S cp_pscaler_set()
{
	INT32U widthFactor,heightFactor;

	widthFactor = ((encinfo.cfg.buf_width*65536)/encinfo.cfg.scale_out_width);
	heightFactor = ((encinfo.cfg.buf_height*65536)/encinfo.cfg.scale_out_height);

	drv_l1_pscaler_clk_ctrl(PSCALER_B,1);
	drv_l1_pscaler_init(PSCALER_B);

	drv_l1_pscaler_input_pixels_set(PSCALER_B, encinfo.cfg.buf_width, encinfo.cfg.buf_height);
	drv_l1_pscaler_input_format_set(PSCALER_B, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
	drv_l1_pscaler_output_fifo_line_set(PSCALER_B, encinfo.cfg.scale_out_height,0);
	drv_l1_pscaler_output_pixels_set(PSCALER_B, widthFactor, encinfo.cfg.scale_out_width, heightFactor, encinfo.cfg.scale_out_height);
	drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_RGB565);
	drv_l1_pscaler_interrupt_set(PSCALER_B, PIPELINE_SCALER_INT_ENABLE_ALL);
	drv_l1_pscaler_callback_register(PSCALER_B, pscaler_out_isr);

}

static INT32S cp_pscaler_out(INT32U in, INT32U out)
{
	INT32U widthFactor,heightFactor;

	pscaler_running = 1;

	widthFactor = ((encinfo.cfg.buf_width*65536)/encinfo.cfg.scale_out_width);
	heightFactor = ((encinfo.cfg.buf_height*65536)/encinfo.cfg.scale_out_height);

	drv_l1_pscaler_clk_ctrl(PSCALER_B,1);
	drv_l1_pscaler_init(PSCALER_B);

	drv_l1_pscaler_input_pixels_set(PSCALER_B, encinfo.cfg.buf_width, encinfo.cfg.buf_height);
	drv_l1_pscaler_input_source_set(PSCALER_B, PIPELINE_SCALER_INPUT_SOURCE_DRAM);
	drv_l1_pscaler_input_format_set(PSCALER_B, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
	drv_l1_pscaler_output_fifo_line_set(PSCALER_B, encinfo.cfg.scale_out_height,0);
	drv_l1_pscaler_output_pixels_set(PSCALER_B, widthFactor, encinfo.cfg.scale_out_width, heightFactor, encinfo.cfg.scale_out_height);
	drv_l1_pscaler_output_format_set(PSCALER_B, PIPELINE_SCALER_OUTPUT_FORMAT_RGB565);
	drv_l1_pscaler_interrupt_set(PSCALER_B, PIPELINE_SCALER_INT_ENABLE_ALL);
	drv_l1_pscaler_callback_register(PSCALER_B, pscaler_out_isr);

	drv_l1_pscaler_input_buffer_set(PSCALER_B, (INT32U)in);
	drv_l1_pscaler_output_A_buffer_set(PSCALER_B, (INT32U)out);
	drv_l1_pscaler_output_B_buffer_set(PSCALER_B, (INT32U)out);
	drv_l1_pscaler_start(PSCALER_B);

	while(pscaler_running)
    {
		/*INT32U ret;
        ret = drv_l1_pscaler_status_get(PSCALER_B);
		DBG_PRINT("ret = %x\r\n", ret);
        if(ret & PIPELINE_SCALER_STATUS_FRAME_DONE)
            break;*/
        osDelay(1);
    }

    return STATUS_OK;
}

INT32S cp_encode_out(INT32U in_addrs, INT32U time, INT32U scale_out_addrs)
{
	INT32U i;
	osStatus status = osOK;
	frame_info_t* pFrame;

	osMutexWait(encinfo.mtx, osWaitForever);
	for (i = 0; i < MAX_ENCODER_OUT_NUM; i++)
	{
		if (encinfo.out_buffer[i].data == 0)
		{
			encinfo.out_buffer[i].data = (INT8U*)in_addrs;
			encinfo.out_buffer[i].time = time;
			encinfo.out_buffer[i].out = (INT8U*)scale_out_addrs;
			encinfo.out_buffer[i].size = 0;
			break;
		}
	}
	osMutexRelease(encinfo.mtx);

	if (i == MAX_ENCODER_OUT_NUM)
	{
		DBG_PRINT("F"); // drop frame.
		return STATUS_FAIL;
	}

	pFrame = &encinfo.out_buffer[i];
	status = osMessagePut(enc_task_q, (INT32U)&pFrame, osWaitForever);
	if (status != osOK)
	{
		DBG_PRINT("QF"); // drop frame.
		encinfo.out_buffer[i].data = 0;
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S cp_encode_muxer_stop()
{
	INT32U msg_id;
	osEvent event;

	event = osMessageGet(enc_task_ack_m, 0);

	msg_id = MSG_MUX_STOP;
	osMessagePut(enc_task_q, (INT32U)&msg_id, osWaitForever);
	event = osMessageGet(enc_task_ack_m, osWaitForever);

	return STATUS_OK;
}
void cp_frame_addref(frame_info_t* frame)
{
	osMutexWait(encinfo.mtx, osWaitForever);
	frame->ref++;
	osMutexRelease(encinfo.mtx);
}

void cp_free_frame(frame_info_t* frame)
{
	osMutexWait(encinfo.mtx, osWaitForever);
	frame->ref--;
	if (frame->ref == 0)
		gp_memset(frame, 0, sizeof(frame_info_t));
	osMutexRelease(encinfo.mtx);
}

static INT32S cp_jpeg_encode(INT32U addrs, INT32U out_addrs)
{
	INT32S  nRet = STATUS_FAIL;
	INT32U status;
	INT32U input_format;

	gp_memcpy((INT8S*)out_addrs, (void *)jpeg_header_get_addr(), encinfo.header_size);
	out_addrs += encinfo.header_size;
	jpeg_encode_init();
	gplib_jpeg_default_quantization_table_load(encinfo.cfg.quality);	// Load default qunatization table(quality)
	gplib_jpeg_default_huffman_table_load();							// Load default huffman table

	while(1)
	{
		if (0 > jpeg_encode_input_size_set(encinfo.cfg.buf_width, encinfo.cfg.buf_height))
			break;

		if (0 > jpeg_encode_input_format_set(C_JPEG_FORMAT_YUYV))
			break;

		if (0 > jpeg_encode_yuv_sampling_mode_set(C_JPG_CTRL_YUV422))
			break;

		if (0 > jpeg_encode_output_addr_set(out_addrs))
			break;

		if (0 > jpeg_encode_once_start(addrs, 0, 0))
			break;

		nRet = STATUS_OK;
		break;
	}

	while(nRet == STATUS_OK) {
		status = jpeg_encode_status_query(TRUE);
		if(status & C_JPG_STATUS_ENCODE_DONE) {
			// Get encode length
			nRet = jpeg_encode_vlc_cnt_get() + encinfo.header_size;
			cache_invalid_range(out_addrs, nRet);
			break;
		} else if(status & C_JPG_STATUS_ENCODING) {
			continue;
		} else {
			DBG_PRINT("JPEG encode error!\r\n");
			nRet = STATUS_FAIL;
			break;
		}
	}

Return:
	jpeg_encode_stop();
	return nRet;
}

static void cp_encode_task_entry(void const *para)
{
	osEvent result;
	INT32U msg_id;
	INT32U out_addrs;
	INT32S encode_size;
	frame_info_t* frame;
	frame_info_t* frame_out;
	osStatus status = osOK;
	INT32U encode_time = 0, start_time = 0;
	INT32S av_offset,  audio_frame_time = 0;

	while(1)
	{
		result = osMessageGet(enc_task_q, 1000);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (msg_id == 0)) {
			continue;
		}

		switch(msg_id)
		{
		case MSG_ENC_TASK_EXIT:
			osMessagePut(enc_task_ack_m, (INT32U)&msg_id, osWaitForever);
			osThreadTerminate(NULL);
			break;
		case MSG_MUX_STOP:
			cp_muxer_stop(1);
			osMessagePut(enc_task_ack_m, (INT32U)&msg_id, 0);
			encode_time = 0;
			start_time = 0;
			audio_frame_time = 0;
			break;
		default:
			encinfo.encode_start = 1;
			frame = (frame_info_t*)msg_id;
			if (!frame)
				break;
			//DBG_PRINT("encode[%x, %x]\r\n", frame->data, frame->out);
			cp_frame_addref(frame);

			if (cp_status & CP_STATUS_MUX_START)
			{
				if (!start_time)
				{
					start_time = frame->time;
					encode_time = 0;
					audio_frame_time = cp_get_audio_frame_duration();
				}
				else
					encode_time = (frame->time - start_time);

				//check a/v sync
				av_offset = encode_time - cp_get_audio_recording_time();

				if (audio_frame_time && (av_offset >= audio_frame_time))
				{
					//drop frame to wait audio.
					DBG_PRINT("d");
					out_addrs = 0;
					cp_global_tick -= 1000/encinfo.cfg.fps; //global tick back one frame
				}
				out_addrs = vfm_get_empty(&encinfo.vfm, 0);
			}
			else
				out_addrs = 0;

			if (out_addrs)
			{
				encode_size = cp_jpeg_encode((INT32U)frame->data, out_addrs);

				if (encode_size > 0)
				{
					cp_error &= ~CP_ERROR_ENCODE_ERROR;
					frame->enc_out = (INT8U*)out_addrs;
					frame->size = encode_size;
					vfm_report_size(&encinfo.vfm, out_addrs, encode_size);
					cp_frame_addref(frame);
					if (0 > cp_muxer_video(frame))
					{
						vfm_post_empty(&encinfo.vfm, out_addrs);
						cp_free_frame(frame);
						if (cp_muxer_get_error())
							cp_muxer_stop(0);
					}
				}
				else
				{
					cp_error |= CP_ERROR_ENCODE_ERROR;
					vfm_post_empty(&encinfo.vfm, out_addrs);
				}
			}

			status = osMessagePut(scale_task_q, (INT32U)&frame, osWaitForever);
			if (status != osOK)
				cp_free_frame(frame);

			break;
		}
	}
}

static void cp_scale_task_entry(void const *para)
{
	osEvent result;
	INT32U msg_id;
	frame_info_t* frame;
	INT32U frame_in, frame_out;

	while(1)
	{
		result = osMessageGet(scale_task_q, 1000);
		msg_id = result.value.v;
		if((result.status != osEventMessage) || (msg_id == 0)) {
			continue;
		}

		switch(msg_id)
		{
		case MSG_SCALE_TASK_EXIT:
			osMessagePut(scale_task_ack_m, (INT32U)&msg_id, osWaitForever);
			osThreadTerminate(NULL);
			break;
		default:
			frame = (frame_info_t*)msg_id;
			if (!frame)
				break;
			//DBG_PRINT("scale[%x, %x]\r\n", frame->data, frame->out);

			if (encinfo.cfg.scale_out)
				cp_pscaler_out((INT32U)frame->data, (INT32U)frame->out);

			frame_in = (INT32U)frame->data;
			frame_out = (INT32U)frame->out;
			cp_free_frame(frame);

		    if (encinfo.cfg.callback)
				(*encinfo.cfg.callback)(frame_in, frame_out);

			break;
		}
	}
}

