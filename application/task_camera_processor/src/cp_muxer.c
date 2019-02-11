#include <string.h>
#include <stdio.h>
#include "application.h"
#include "cp_camera_processor.h"
#include "gp_mux.h"
#include "gp_vfm.h"

#define MUX_INPUT_QUEUE_MAX 30
#define MUX_MIN_DISK_FREE_SIZE 0x200000 //2MB
#define MUX_MAX_FILE_SIZE	0x80000000 //2GB

extern struct gpMux_s AVI_packer;
extern struct gpMux_s MOV_packer;
static struct gpMux_s* avipacker = &AVI_packer;
static INT32S hAVIPacker = 0;
static INT16S hFile;

extern CP_ENCODER_INFO encinfo;

osMessageQId mux_task_q = NULL;
osMessageQId mux_task_ack_m = NULL;

static void cp_mux_task_entry(void const *para);

static INT32U recording_time = 0;
static INT64U disk_free_size = 0;
static INT32U file_size = 0;
static INT32U current_frame_time = 0;

static void avi_encode_end(INT32S hd)
{
	muxAviAddInfoStr(hd, "ISRC", "Generplus");
	muxAviAddInfoStr(hd, "IART", "Generplus");
	muxAviAddInfoStr(hd, "ICOP", "Generplus");
	muxAviAddInfoStr(hd, "ICRD", "2017-01-01");
}

INT32S cp_muxer_task_create()
{
	INT32S nRet = STATUS_OK;
	osThreadId id = NULL;
	osThreadDef_t mux_task = { "mux_task", cp_mux_task_entry, osPriorityHigh, 1, 2048 };
	osMessageQDef_t mux_q_def = {MUX_INPUT_QUEUE_MAX, sizeof(INT32U), 0};
	osMessageQDef_t mux_ack_def = {1, sizeof(INT32U), 0};

	mux_task_q = osMessageCreate(&mux_q_def, NULL);
	mux_task_ack_m = osMessageCreate(&mux_ack_def, NULL);

	if (mux_task_q == NULL || mux_task_ack_m == NULL)
	{
		nRet = STATUS_FAIL;
	}

	if (nRet == STATUS_OK)
	{
		id = osThreadCreate(&mux_task, (void *)NULL);
		if (!id)
			nRet = STATUS_FAIL;
	}

	if (nRet == STATUS_FAIL)
	{
		if (mux_task_q) vQueueDelete(mux_task_q);
		if (mux_task_ack_m) vQueueDelete(mux_task_ack_m);

		mux_task_q = mux_task_ack_m = 0;
	}
	return nRet;
}

INT32S cp_muxer_task_del()
{
	packer_msg_t* m;
	osEvent event;

	event = osMessageGet(mux_task_ack_m, 0);
	m = (packer_msg_t*)gp_malloc(sizeof(packer_msg_t));
	m->msg = MSG_MUX_TASK_EXIT;
	osMessagePut(mux_task_q, (INT32U)&m, osWaitForever);
	event = osMessageGet(mux_task_ack_m, osWaitForever);

	vQueueDelete(mux_task_q);
	vQueueDelete(mux_task_ack_m);

	return STATUS_OK;
}

INT32S cp_muxer_start(INT16S File)
{
	packer_msg_t* m;
	INT32S tmp;
	INT16S disk_no;
	char DiskLetter[] = "ABCDEFGHIJK";
	char temp_path[5];

	cp_error &= ~(CP_ERROR_DISK_FULL | CP_ERROR_MUX_ERROR | CP_ERROR_FILE_SIZE_LIMIT);

	disk_no = GetDiskOfFile(File);
	if (disk_no < 0)
		disk_no = 10; //K

	disk_free_size = vfsFreeSpace(disk_no);

	if(disk_free_size < MUX_MIN_DISK_FREE_SIZE)
	{
		DBG_PRINT("cp_muxer_start fail: disk free size not enough. %lld KBytes\r\n", disk_free_size/1024);
		cp_error |= CP_ERROR_DISK_FULL;
		return STATUS_FAIL;
	}
	disk_free_size -= MUX_MIN_DISK_FREE_SIZE;

	hFile = File;
    hAVIPacker = avipacker->open(File);
    avipacker->set(hAVIPacker, MUX_MAX_SIZE, 0x40000000);
    avipacker->set(hAVIPacker, MUX_VID_TYPE, VIDEO_TYPE_MJPEG);
    avipacker->set(hAVIPacker, MUX_HEIGHT, encinfo.cfg.enc_height);
    avipacker->set(hAVIPacker, MUX_WIDTH, encinfo.cfg.enc_width);
    avipacker->set(hAVIPacker, MUX_FRMRATE, encinfo.cfg.fps);
    if (encinfo.cfg.enable_audio)
    {
		if (encinfo.cfg.aFormat == WAV)
			avipacker->set(hAVIPacker, MUX_AUD_TYPE, AUD_FORMAT_PCM);
		else if (encinfo.cfg.aFormat == IMA_ADPCM)
			avipacker->set(hAVIPacker, MUX_AUD_TYPE, AUD_FORMAT_IMA_ADPCM);
		avipacker->set(hAVIPacker, MUX_AUD_CHANNEL, 1);
		avipacker->set(hAVIPacker, MUX_AUDSR, encinfo.cfg.sampleRate);
		avipacker->set(hAVIPacker, MUX_AUD_BIT, 16);
	}
	sprintf((char*)temp_path, "%c:\\", DiskLetter[disk_no]);

    avipacker->set(hAVIPacker, MUX_PATH_TEMP_FILE, (INT32U)temp_path);

	DBG_PRINT("cp_muxer_start: DISK FREE SIZE = %lld MBytes\r\n", disk_free_size/1024/1024);

	cp_status |= CP_STATUS_MUX_START;

	m = (packer_msg_t*)gp_malloc(sizeof(packer_msg_t));
	m->msg = MSG_MUX_START;
	m->handle = hAVIPacker;
	tmp = (INT32S)File;
	m->data = (void*)tmp;
	osMessagePut(mux_task_q, (INT32U)&m, osWaitForever);

    return STATUS_OK;
}

INT32S cp_muxer_stop(INT8U wait_stop)
{
	packer_msg_t* m;
	osEvent event;

	event = osMessageGet(mux_task_ack_m, 0);

	m = (packer_msg_t*)gp_malloc(sizeof(packer_msg_t));
	m->msg = MSG_MUX_STOP;
	m->handle = hAVIPacker;
	m->data = NULL;
	hAVIPacker = 0;
	osMessagePut(mux_task_q, (INT32U)&m, osWaitForever);

	if (wait_stop)
		event = osMessageGet(mux_task_ack_m, osWaitForever);

	return STATUS_OK;
}

inline INT32S cp_muxer_update_disk_free_size(INT32S size)
{
	if (size > 0 && size > disk_free_size)
		return STATUS_FAIL;

	disk_free_size -= size;

	return STATUS_OK;
}

INT32S cp_muxer_video(frame_info_t* frame)
{
	packer_msg_t* m;
	gpMuxPkt_t pkt;

	if (!hAVIPacker)
		return STATUS_FAIL;

	cp_error &= ~(CP_ERROR_DISK_FULL | CP_ERROR_MUX_ERROR | CP_ERROR_FILE_SIZE_LIMIT);

	if ((file_size + frame->size) >= MUX_MAX_FILE_SIZE)
	{
		cp_error |= CP_ERROR_FILE_SIZE_LIMIT;
		return STATUS_FAIL;
	}

	if (0 > cp_muxer_update_disk_free_size(frame->size))
	{
		cp_error |= CP_ERROR_DISK_FULL;
		return STATUS_FAIL;
	}

	m = (packer_msg_t*)gp_malloc(sizeof(packer_msg_t));
	m->msg = MSG_MUX_VID;
	m->handle = hAVIPacker;
	m->data = (void*)frame;
	if (osOK == osMessagePut(mux_task_q, (INT32U)&m, 1000))
		return STATUS_OK;

	cp_muxer_update_disk_free_size(-frame->size);
	gp_free(m);
	return STATUS_FAIL;
}

INT32S cp_muxer_audio(frame_info_t* frame)
{
	packer_msg_t* m;
	gpMuxPkt_t pkt;

	if (!hAVIPacker)
		return STATUS_FAIL;

	cp_error &= ~(CP_ERROR_DISK_FULL | CP_ERROR_MUX_ERROR | CP_ERROR_FILE_SIZE_LIMIT);

	if ((file_size + frame->size) >= MUX_MAX_FILE_SIZE)
	{
		cp_error |= CP_ERROR_FILE_SIZE_LIMIT;
		return STATUS_FAIL;
	}

	if (0 > cp_muxer_update_disk_free_size(frame->size))
	{
		cp_error |= CP_ERROR_DISK_FULL;
		return STATUS_FAIL;
	}
	m = (packer_msg_t*)gp_malloc(sizeof(packer_msg_t));
	m->msg = MSG_MUX_AUD;
	m->handle = hAVIPacker;
	m->data = (void*)frame;
	if (osOK == osMessagePut(mux_task_q, (INT32U)&m, 1000))
		return STATUS_OK;

	cp_muxer_update_disk_free_size(-frame->size);
	gp_free(m);
	return STATUS_FAIL;
}

INT32S cp_muxer_get_error()
{
	if (cp_error & (CP_ERROR_DISK_FULL | CP_ERROR_FILE_SIZE_LIMIT | CP_ERROR_MUX_ERROR))
		return 1;

	return 0;
}

INT32U cp_muxer_get_recording_time()
{
	return recording_time;
}

INT32U cp_muxer_get_estimate_frame_delay()
{
	INT32U delay_frame = 0;
	if (current_frame_time && (cp_global_tick > current_frame_time))
		delay_frame = ((cp_global_tick - current_frame_time) * encinfo.cfg.fps)/ 1000;

	return delay_frame;
}
static void cp_mux_task_entry(void const *para)
{
	osEvent result;
	INT32U msg_id;
	frame_info_t* frame;
	osStatus status = osOK;
	gpMuxPkt_t pkt, np = {0, 0, 0, 0};
	INT32U last_ts = 0, last_ats = 0;
	INT32S handle;
	INT16S file_handle = -1;
	packer_msg_t* m;
	INT32S tmp;
	INT32U Null_Frm = 0, Drop_Frame, Frame_Cnt;

	while(1)
	{
		result = osMessageGet(mux_task_q, 1000);
		m = (packer_msg_t*)result.value.v;
		if((result.status != osEventMessage) || (m == 0)) {
			continue;
		}

		msg_id = m->msg;

		switch(msg_id)
		{
		case MSG_MUX_TASK_EXIT:
			DBG_PRINT("[MSG_MUX_TASK_EXIT]\r\n");
			gp_free(m);
			osMessagePut(mux_task_ack_m, (INT32U)&msg_id, osWaitForever);
			osThreadTerminate(NULL);
			break;
		case MSG_MUX_START:
			DBG_PRINT("[MSG_MUX_START][%x]\r\n", m->handle);
			current_frame_time = 0;
			recording_time = 0;
			Frame_Cnt = 0;
			Drop_Frame = 0;
			file_size = 0;
			last_ts = 0;
			last_ats = 0;
			tmp = (INT32S)m->data;
			file_handle = (INT16S)tmp;
			gp_free(m);
			break;
		case MSG_MUX_STOP:
			DBG_PRINT("[MSG_MUX_STOP][%x]\r\n", m->handle);
			handle = m->handle;
			gp_free(m);
			if (cp_status & CP_STATUS_MUX_START)
			{
				cp_status &= ~CP_STATUS_MUX_START;
				if (avipacker == &AVI_packer)
					avi_encode_end(handle);
				avipacker->close(handle);
				fs_close(file_handle);
			}
			osMessagePut(mux_task_ack_m, (INT32U)&msg_id, 0);
			file_handle = -1;
			current_frame_time = 0;
			break;
		case MSG_MUX_VID:
			frame = (frame_info_t*)m->data;
			handle = m->handle;
			gp_free(m);
			if (!frame) break;
			if (!handle || cp_muxer_get_error()){
				vfm_post_empty(&encinfo.vfm, (INT32U)frame->enc_out);
				cp_free_frame(frame);
				break;
			}
			//DBG_PRINT("[MSG_MUX_VID] %x (%d)\r\n", frame->size, xTaskGetTickCount());
			pkt.data = frame->enc_out;
			pkt.size = frame->size;
			pkt.frameType = 0;

			if (last_ts)
			{
				if (last_ts > frame->time)
				{
					DBG_PRINT("o"); //out of order
					vfm_post_empty(&encinfo.vfm, (INT32U)frame->enc_out);
					cp_free_frame(frame);
					break;
				}
				pkt.pts = frame->time - last_ts;
			}
			else
				pkt.pts = 1000/encinfo.cfg.fps;

			last_ts = frame->time;

			#if VARIABLE_FRAME_RATE == 0
			Null_Frm = 0;
			tmp = (((recording_time + pkt.pts) * encinfo.cfg.fps)/ 1000) - (Frame_Cnt+1);
			if (tmp > 0)
				Null_Frm = tmp;
			else if (tmp < 0)
				Drop_Frame -= tmp;

			while(Null_Frm--)
			{
				if (SUCCESS != avipacker->pack(handle, &np, GP_ES_TYPE_VIDEO))
					cp_error |= CP_ERROR_MUX_ERROR;
				else
					Frame_Cnt ++;
				DBG_PRINT("n");
			}

			if (Drop_Frame)
			{
				DBG_PRINT("D");
				Drop_Frame--;
				recording_time += pkt.pts;
			}
			else
			#endif
			if (SUCCESS != avipacker->pack(handle, &pkt, GP_ES_TYPE_VIDEO))
			{
				DBG_PRINT("MSG_MUX_VID mux error\r\n");
				cp_error |= CP_ERROR_MUX_ERROR;
			}
			else
			{
				Frame_Cnt ++;
				recording_time += pkt.pts;
				file_size += pkt.size;
				current_frame_time = frame->time;
				DBG_PRINT(".");
			}
			vfm_post_empty(&encinfo.vfm, (INT32U)frame->enc_out);
			cp_free_frame(frame);
			break;
		case MSG_MUX_AUD:
			frame = (frame_info_t*)m->data;
			handle = m->handle;
			gp_free(m);
			if (!frame) break;
			if (!handle|| cp_muxer_get_error()){
				gp_free(frame);
				break;
			}
			//DBG_PRINT("[MSG_MUX_VID] %x (%d)\r\n", frame->size, xTaskGetTickCount());
			pkt.data = frame->enc_out;
			pkt.size = frame->size;
			pkt.frameType = 0;
			#if VARIABLE_FRAME_RATE
			if (last_ats)
				pkt.pts = frame->time - last_ats;
			else
				pkt.pts = (pkt.size >> 1)*1000/encinfo.cfg.sampleRate;

			last_ats = frame->time;
			#else
			pkt.pts = frame->time;
			#endif

			if (SUCCESS != avipacker->pack(handle, &pkt, GP_ES_TYPE_AUDIO))
			{
				DBG_PRINT("MSG_MUX_AUD mux error\r\n");
				cp_error |= CP_ERROR_MUX_ERROR;
			}
			else
			{
				file_size += pkt.size;
			}
			gp_free(frame);
			DBG_PRINT("a");
			break;
		}
	}

}
