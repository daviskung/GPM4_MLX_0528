#include <string.h>
#include "cp_camera_processor.h"
#include "drv_l1_csi.h"
#include "drv_l1_pscaler.h"
#include "drv_l1_scaler.h"
#include "drv_l2_scaler.h"

static CP_SENSOR_INFO _sensor[MAX_SENSOR_NUM];
static INT32U _cdsp_idx, _csi_idx;
static ScalerFormat_t scale;
static ScalerPara_t para;
static osMessageQId sclr_task_q = NULL;
static osMessageQId sclr_task_ack = NULL;
static _scaler_enable = 0;
static _scaler_busy = 0;

static void pscaler_isr(INT32U event)
{
	INT32U out_buf = 0, next_buf = 0;

	if (event & PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR)
		DBG_PRINT("PscalerA overflow!\r\n");

	if (event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
	{
		out_buf = drv_l1_pscaler_output_A_buffer_get(PSCALER_A);
	}
	else if (event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
	{
		out_buf = drv_l1_pscaler_output_B_buffer_get(PSCALER_A);
	}

	if (!out_buf)
		return;

	if (_sensor[_cdsp_idx].callback)
	{
		(*_sensor[_cdsp_idx].callback)(_cdsp_idx, out_buf, cp_global_tick);
	}

	next_buf = _sensor[_cdsp_idx].next_buf[1];
	_sensor[_cdsp_idx].next_buf[1] = 0;

	if (!next_buf)
		next_buf = DUMMY_BUFFER_ADDRS;

	if (event & PIPELINE_SCALER_STATUS_BUF_A_DONE)
	{
		drv_l1_pscaler_output_A_buffer_set(PSCALER_A, next_buf);
	}
	else if (event & PIPELINE_SCALER_STATUS_BUF_B_DONE)
	{
		drv_l1_pscaler_output_B_buffer_set(PSCALER_A, next_buf);
	}

}

static INT32S pscaler_set(CP_SENSOR_INFO* info)
{
	INT32U widthFactor,heightFactor;
	INT32U out_width, out_height;
	INT32U out_format;

	if (!info)
		return STATUS_FAIL;

	out_width = (info->output_width + 0xf) & ~0xf;
	out_height = (info->output_height + 0xf) & ~0xf;

	widthFactor = ((info->width*65536)/out_width);
	heightFactor = ((info->height*65536)/out_height);

	switch(info->output_format)
	{
	case V4L2_PIX_FMT_RGB565:
		out_format = PIPELINE_SCALER_OUTPUT_FORMAT_RGB565;	break;
	case V4L2_PIX_FMT_YUYV:
		out_format = PIPELINE_SCALER_OUTPUT_FORMAT_YUYV; break;
	default:
		return STATUS_FAIL;
	}

	drv_l1_pscaler_clk_ctrl(PSCALER_A,1);
	drv_l1_pscaler_init(PSCALER_A);
	drv_l1_CdspSetYuvPscalePath(1);
	drv_l1_pscaler_input_source_set(PSCALER_A, PIPELINE_SCALER_INPUT_SOURCE_CDSP);
	drv_l1_pscaler_input_pixels_set(PSCALER_A, info->width, info->height);
	drv_l1_pscaler_input_buffer_set(PSCALER_A, DUMMY_BUFFER_ADDRS);
	drv_l1_pscaler_input_format_set(PSCALER_A, PIPELINE_SCALER_INPUT_FORMAT_YUYV);
	drv_l1_pscaler_output_format_set(PSCALER_A, out_format);
	drv_l1_pscaler_output_fifo_line_set(PSCALER_A, info->output_height,0);
	drv_l1_pscaler_output_pixels_set(PSCALER_A, widthFactor, out_width, heightFactor, out_height);
	drv_l1_pscaler_output_A_buffer_set(PSCALER_A, DUMMY_BUFFER_ADDRS);
	drv_l1_pscaler_output_B_buffer_set(PSCALER_A, DUMMY_BUFFER_ADDRS);
	drv_l1_pscaler_callback_register(PSCALER_A, &pscaler_isr);
	drv_l1_pscaler_interrupt_set(PSCALER_A, PIPELINE_SCALER_INT_ENABLE_ALL);

	info->next_buf[0] = info->next_buf[1] = 0;

	return STATUS_OK;
}

static void csi_isr(INT32U event)
{
	INT32U out_buf;

	if (event == CSI_SENSOR_FRAME_END_EVENT)
	{
		out_buf = drv_l1_csi_get_buf();

		if (_scaler_enable)
		{
			if (osOK != osMessagePut(sclr_task_q, (INT32U)&out_buf, 0))
				return;

			if (out_buf == (INT32U)_sensor[_csi_idx].mem)
				out_buf += _sensor[_csi_idx].width*_sensor[_csi_idx].height*2;
			else
				out_buf = (INT32U)_sensor[_csi_idx].mem;
		}
		else
		{
			if (_sensor[_csi_idx].callback)
			{
				(*_sensor[_csi_idx].callback)(_csi_idx, out_buf, cp_global_tick);
			}
			out_buf = _sensor[_csi_idx].next_buf[0];

			if (!out_buf)
				out_buf = DUMMY_BUFFER_ADDRS;

			_sensor[_csi_idx].next_buf[0] = _sensor[_csi_idx].next_buf[1];
			_sensor[_csi_idx].next_buf[1] = 0;
		}

		drv_l1_csi_set_buf(out_buf);
	}
}

static void scaler_isr(INT32U event0, INT32U event1)
{
	INT32U out_buf, next_buf;

	if(event0 & C_SCALER_STATUS_DONE)
	{
		out_buf = scale.output_y_addr;

		if (_sensor[_csi_idx].callback)
		{
			(*_sensor[_csi_idx].callback)(_csi_idx, out_buf, cp_global_tick);
		}
		drv_l2_scaler_stop(SCALER_0);
		_scaler_busy = 0;
	}
}

static INT32S scaler_set(CP_SENSOR_INFO* info)
{
	INT32U out_format;

	if (!info)
		return STATUS_FAIL;

	switch(info->output_format)
	{
	case V4L2_PIX_FMT_RGB565:
		out_format = C_SCALER_CTRL_OUT_RGB565;	break;
	case V4L2_PIX_FMT_YUYV:
		out_format = C_SCALER_CTRL_OUT_YUYV; break;
	default:
		return STATUS_FAIL;
	}

	scale.input_format = C_SCALER_CTRL_IN_YUYV;
	scale.input_width = info->width;
	scale.input_height = info->height;
	scale.input_y_addr = DUMMY_BUFFER_ADDRS;
	scale.output_format = out_format;
	scale.output_width = info->output_width;
	scale.output_height = info->output_height;
	scale.output_buf_width = info->output_width;
	scale.output_buf_height = info->output_height;
	scale.output_y_addr = DUMMY_BUFFER_ADDRS;
	scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
	scale.scale_mode = C_SCALER_FULL_SCREEN;
	scale.digizoom_m = 10;
	scale.digizoom_n = 10;
	scale.callback = scaler_isr;
	para.boundary_color = 0x008080;

	info->next_buf[0] = info->next_buf[1] = 0;
}

static void scaler_task(void const *parm)
{
	INT32U i,msg, frame, out;
	osEvent result;
	osThreadId id;

	while(1)
	{
		result = osMessageGet(sclr_task_q, osWaitForever);
		msg = result.value.v;

		if((result.status != osEventMessage) || (msg == 0))
			continue;

		switch(msg)
		{
		case MSG_SCALER_EXIT:
			_scaler_busy = 0;
			osMessagePut(sclr_task_ack, (INT32U)&msg, osWaitForever);
			vQueueDelete(sclr_task_q);
			sclr_task_q = NULL;
			id = osThreadGetId();
    		osThreadTerminate(id);
    		break;
		default:
			//if (C_SCALER_STATUS_STOP != drv_l1_scaler_status_polling(SCALER_0))
			if (_scaler_busy)
				break;

			frame = msg;

			out = _sensor[_csi_idx].next_buf[0];

			if (!out)
				out = DUMMY_BUFFER_ADDRS;

			_sensor[_csi_idx].next_buf[0] = _sensor[_csi_idx].next_buf[1];
			_sensor[_csi_idx].next_buf[1] = 0;

			scale.input_y_addr = frame;
			scale.output_y_addr = out;
			_scaler_busy = 1;
			if (0 > drv_l2_scaler_trigger(SCALER_0, 0, &scale, &para))
				_scaler_busy = 0;

			frame = 0;

			break;
		}
	}
}

INT32S cp_sensor_init()
{
	INT32U i;
	CHAR* p;

	// Enable Sensor Clock
	R_SYSTEM_CTRL |= (1 << 11);
	// Change Sensor control pin to IOD6~9
	R_FUNPOS1 |= (1<<21)|(1<<24);
	R_FUNPOS1 |= (1<<7);
	// sensor init
	drv_l2_sensor_init();

	for (i = 0; i < MAX_SENSOR_NUM; i++)
	{
		gp_memset(&_sensor[i], 0, sizeof(CP_SENSOR_INFO));
		_sensor[i].sensor = drv_l2_sensor_get_ops(i);
		_sensor[i].callback = NULL;
	}

	for(i = 0; i < MAX_SENSOR_NUM; i++) {
		if (!_sensor[i].sensor)
			continue;
		// get csi or cdsp
		p = (CHAR *)strrchr((CHAR *)_sensor[i].sensor->name, 'c');
		if(p == 0) {
			break;
		}

		if(strncmp((CHAR *)p, "csi", 3) == 0) {
			_sensor[i].interface = CSI_INTERFACE;
			_csi_idx = i;
		} else if(strncmp((CHAR *)p, "cdsp", 4) == 0) {
			_sensor[i].interface = CDSP_INTERFACE;
			_cdsp_idx = i;
		} else {
			break;
		}
	}

	if (i != MAX_SENSOR_NUM)
		return STATUS_FAIL;

	return STATUS_OK;
}

INT32S cp_sensor_getinfo(INT32U idx, CAMINFO* info)
{
	if (info && _sensor[idx].sensor)
	{
		info->name = strdup(_sensor[idx].sensor->name);
		info->interface = _sensor[idx].interface;
		return STATUS_OK;
	}

	return STATUS_FAIL;
}

INT32S cp_sensor_close()
{
	INT32U i;
	INT32U msg;
	osEvent result;

	if (sclr_task_q)
	{
		msg = MSG_SCALER_EXIT;
		osMessagePut(sclr_task_q, (INT32U)&msg, osWaitForever);
		result = osMessageGet(sclr_task_ack, 3000);
		vQueueDelete(sclr_task_ack);
		sclr_task_ack = 0;
	}
	for(i = 0; i < MAX_SENSOR_NUM; i++) {
		if (_sensor[i].mem)
			gp_free(_sensor[i].mem);
	}
	return STATUS_OK;
}

INT32S cp_sensor_set(INT32U idx, CAMCFG* cfg)
{
	if (idx > MAX_SENSOR_NUM || !_sensor[idx].sensor)
		return STATUS_FAIL;

	_sensor[idx].enable = cfg->enable;

	if (!cfg->enable)
		return STATUS_OK;

	_sensor[idx].width = cfg->input_width;
	_sensor[idx].height = cfg->input_height;
	_sensor[idx].output_width = cfg->output_width;
	_sensor[idx].output_height = cfg->output_height;
	_sensor[idx].output_format = cfg->output_format;
	_sensor[idx].callback = cfg->callback;

	if(_sensor[idx].interface == CDSP_INTERFACE)
	{
		if (0 > pscaler_set(&_sensor[idx]))
			return STATUS_FAIL;
	}
	else
	{
		osThreadId id;
		osThreadDef_t scalar_task_def = { "scalar_task", scaler_task, osPriorityHigh, 1, 2048 };
		osMessageQDef_t scalar_q = {6, sizeof(INT32U), 0};
		osMessageQDef_t scalar_ack = {1, sizeof(INT32U), 0};

		if (_sensor[idx].width != _sensor[idx].output_width ||
			_sensor[idx].height != _sensor[idx].output_height)
		{
			_scaler_enable = 1;
			sclr_task_q = osMessageCreate(&scalar_q, NULL);
			if (!sclr_task_q)
				return STATUS_FAIL;

			sclr_task_ack = osMessageCreate(&scalar_ack, NULL);

			if (_sensor[idx].mem)
				gp_free(_sensor[idx].mem);
			_sensor[idx].mem = (INT32U*)gp_malloc(_sensor[idx].width*_sensor[idx].height*2*2);
			if (!_sensor[idx].mem)
				return STATUS_FAIL;

			id = osThreadCreate(&scalar_task_def, (void *)NULL);
			if (!id)
			{
				gp_free(_sensor[idx].mem);
				_sensor[idx].mem = NULL;
				return STATUS_FAIL;
			}

			drv_l1_csi_set_buf((INT32U)_sensor[idx].mem);
			drv_l1_register_csi_cbk((void*)&csi_isr);
			scaler_set(&_sensor[idx]);
		}
		else
		{
			_scaler_enable = 0;
			drv_l1_csi_set_buf(DUMMY_BUFFER_ADDRS);
			drv_l1_register_csi_cbk((void*)&csi_isr);
		}
	}

	return STATUS_OK;
}

INT32S cp_sensor_start(INT32U idx, INT32U bufA, INT32U bufB)
{
	INT32U i, info_idx;
	drv_l2_sensor_info_t *pInfo;
	drv_l2_sensor_ops_t* p;

	if (idx > MAX_SENSOR_NUM || !_sensor[idx].sensor || !_sensor[idx].enable)
		return STATUS_FAIL;

	p = _sensor[idx].sensor;

	for(i=0; i<MAX_INFO_NUM; i++) {
		pInfo = p->get_info(i);
		if(pInfo->target_w == _sensor[idx].width && pInfo->target_h == _sensor[idx].height) {
			break;
		}
	}

	if (i == MAX_INFO_NUM)
		return STATUS_FAIL;

	if(_sensor[idx].interface == CDSP_INTERFACE)
	{
		drv_l1_pscaler_output_A_buffer_set(PSCALER_A, bufA);
		drv_l1_pscaler_output_B_buffer_set(PSCALER_A, bufB);
		bufA = bufB = DUMMY_BUFFER_ADDRS;
	}
	else
	{
		_sensor[idx].next_buf[0] = bufB;
		_sensor[idx].next_buf[1] = 0;
	}

	info_idx = i;

	if(_sensor[idx].interface == CDSP_INTERFACE)
		drv_l1_pscaler_start(0);

	p->init();
	p->stream_start(info_idx, bufA, DUMMY_BUFFER_ADDRS);

	return STATUS_OK;
}

INT32S cp_sensor_stop(INT32U idx)
{
	drv_l2_sensor_ops_t* p;

	if (idx > MAX_SENSOR_NUM || !_sensor[idx].sensor || !_sensor[idx].enable)
		return STATUS_FAIL;

	p = _sensor[idx].sensor;

	p->stream_stop();
	p->uninit();

	return STATUS_OK;
}

INT32S cp_sensor_set_next_frame(INT32U idx, INT32U addr)
{
	if (idx > MAX_SENSOR_NUM || !_sensor[idx].sensor || !_sensor[idx].enable)
		return STATUS_FAIL;

	_sensor[idx].next_buf[1] = addr;

	return STATUS_OK;
}
