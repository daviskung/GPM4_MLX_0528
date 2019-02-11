#include "application.h"
#include "camera_processor.h"
#include "cp_camera_processor.h"
#include "drv_l1_timer.h"

extern CP_ENCODER_INFO encinfo;

volatile INT32U cp_global_tick = 0;
INT32U cp_status = 0;
INT32U cp_error = 0;

void cp_timer_tick(void)
{
	cp_global_tick++;
}
INT32S camera_system_init(CAM_SYSTEM_INFO* system_info)
{
	if (!system_info)
		return STATUS_FAIL;

	timer_freq_setup(0, 1000,0, cp_timer_tick);

	if (STATUS_OK == cp_sensor_init())
	{
		INT32U i;

		gp_memset((INT8S*)system_info, 0, sizeof(CAM_SYSTEM_INFO));

		for(i = 0; i < MAX_CAMERA_NUM; i++)
		{
			if (STATUS_FAIL == cp_sensor_getinfo(i, &system_info->camera_info[i]))
				break;

			system_info->camera_num++;
		}
		return STATUS_OK;
	}

	return STATUS_FAIL;
}

void camera_system_close()
{
	cp_sensor_close();
	timer_stop(0);
}

INT32S camera_init(INT32U cam_idx, CAMCFG* cfg)
{
	return cp_sensor_set(cam_idx, cfg);
}

INT32S camera_start(INT32U cam_idx, INT32U bufA, INT32U bufB)
{
	return cp_sensor_start(cam_idx, bufA, bufB);
}

INT32S camera_stop(INT32U cam_idx)
{
	return cp_sensor_stop(cam_idx);
}

INT32S camera_set_next_frame(INT32U cam_idx, INT32U addr)
{
	return cp_sensor_set_next_frame(cam_idx, addr);
}

INT32S camera_encode_init(ENCCFG* cfg)
{
	INT32S nRet;

	if (cp_encode_init(cfg) < 0)
		return STATUS_FAIL;

	if (cp_muxer_task_create() <0)
		return STATUS_FAIL;

	if (cp_encode_task_create() < 0)
	{
		cp_muxer_task_del();
		return STATUS_FAIL;
	}

	if (cfg->enable_audio)
	{
		cp_adc_record_task_create();
	}

	cp_status |= CP_STATUS_ENCODE_START;

	return STATUS_OK;
}

INT32S camera_encode_close()
{

	cp_status &= ~CP_STATUS_ENCODE_START;

	cp_encode_task_del();

	if (encinfo.cfg.enable_audio)
	{
		cp_adc_record_task_del();
	}

	cp_muxer_task_del();
	cp_encode_close();

	return STATUS_OK;
}

INT32S camera_video_muxer_start(INT16S File)
{
	cp_audio_record_start();
	return cp_muxer_start(File);
}

INT32S camera_video_muxer_stop()
{
	cp_audio_record_stop();
	return cp_encode_muxer_stop();
}

INT32S camera_encode_out(INT32U addr, INT32U time, INT32U scale_out_addr)
{
	return cp_encode_out(addr, time, scale_out_addr);
}

INT32S camera_audio_drop_frame()
{
	cp_audio_record_pause();
	return STATUS_OK;
}
INT32U camera_video_muxer_get_recording_time()
{
	INT32U ret = 0;

	ret = cp_muxer_get_recording_time();
	return ret/1000;
}

INT32U camera_video_muxer_get_dummy_frame_count()
{
	return cp_muxer_get_estimate_frame_delay();
}

INT32U camera_system_status()
{
	if (cp_status & CP_STATUS_MUX_START)
		return VIDEO_CODEC_PROCESSING;
	else
		return VIDEO_CODEC_PROCESS_END;
}

INT32U camera_system_error()
{
	return cp_error;
}
