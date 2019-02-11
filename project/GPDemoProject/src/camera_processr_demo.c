#include <stdio.h>
#include <string.h>
#include "application.h"
#include "gplib.h"
#include "camera_processor.h"
#include "drv_l2_sensor.h"
#include "drv_l2_display.h"
#include "drv_l1_scaler.h"
#include "drv_l2_scaler.h"

//cdsp in width & height must match sensor driver config
#define CDSP_IN_WIDTH		1280
#define CDSP_IN_HEIGHT		720
//cdsp sensor out put size, if different with input, it will be scaled down with pscaler.
#define CDSP_OUT_WIDTH		640
#define CDSP_OUT_HEIGHT		720
//csi in width & height must match sensor driver config
#define CSI_IN_WIDTH		640
#define CSI_IN_HEIGHT		480
//csi sensor out put size, if different with input, it will be scaled down with scaler.
#define CSI_OUT_WIDTH		640
#define CSI_OUT_HEIGHT		480
//final buffer dimension to encode to video
#define ENCODE_WIDTH		1280
#define ENCODE_HEIGHT		720

#define VIDEO_FPS	25

INT32U cam_bufA[3];
INT32U time_bufA[3];
INT32U cam_bufB[3];
INT32U time_bufB[3];
INT32U disp_buf[3];
INT32U dst_buf[3];
INT32U cdsp_idx;
INT32U csi_idx;
osMessageQId task_q;
INT32U muxer_stop = 0;
INT32U record_start = 0;

void CAM_callback(INT32U idx, INT32U addr, INT32U time)
{
	INT32U next_addrs;
	INT32U offset;

	//get sensor frame with frame address and timestamp (in ISR mode)
	if (idx == cdsp_idx)
	{
		if (addr != DUMMY_BUFFER_ADDRS)
		{
			offset = (addr - cam_bufA[0])/(CDSP_OUT_WIDTH*CDSP_OUT_HEIGHT*2);
			next_addrs = cam_bufA[(offset+2)%3];
			time_bufA[offset] = time;
			if (record_start && osOK != osMessagePut(task_q, (INT32U)&offset, 0))
				DBG_PRINT("M1");
		}
		else
		{
			offset = 0;
			next_addrs = cam_bufA[(offset+2)%3];
		}

		//set next frame buffer address for sensor.
		camera_set_next_frame(idx, next_addrs);
	}
	else
	{
		if (addr != DUMMY_BUFFER_ADDRS)
		{
			offset = (addr - cam_bufB[0])/(CSI_OUT_WIDTH*CSI_OUT_HEIGHT*2);
			next_addrs = cam_bufB[(offset+2)%3];
			time_bufB[offset] = time;
			offset |= 0x100;
			if (record_start && osOK != osMessagePut(task_q, (INT32U)&offset, 0))
				DBG_PRINT("M2");
		}
		else
		{
			offset = 0;
			next_addrs = cam_bufB[(offset+2)%3];
		}

		camera_set_next_frame(idx, next_addrs);
	}
}

//if enable scale out, get scaled frame address and set to display
void ENC_callback(INT32U addr, INT32U scale_out_addr)
{
	if (scale_out_addr)
		R_TFT_FBI_ADDR = scale_out_addr;
}

//this demo use scaler to copy buffers
//don't use scaler if csi sensor using scaler for output.
void do_scaler(INT32U src, INT32U src_w, INT32U src_h, INT32U dst, INT32U dst_w, INT32U dst_h, INT32U offset_x)
{
	ScalerFormat_t scale = {0};
	ScalerPara_t para = {0};

	scale.input_format = C_SCALER_CTRL_IN_YUYV;
	scale.input_width = src_w;
	scale.input_height = src_h;
	scale.input_visible_width = 0;
	scale.input_visible_height = 0;
	scale.input_x_offset = 0;
	scale.input_y_offset = 0;

	scale.input_y_addr = src;
	scale.input_u_addr = 0;
	scale.input_v_addr = 0;

	scale.output_format = C_SCALER_CTRL_OUT_YUYV;
	scale.output_width = dst_w;
	scale.output_height = dst_h;
	scale.output_buf_width = dst_w;
	scale.output_buf_height = dst_h;
	scale.output_x_offset = offset_x;

	scale.output_y_addr = dst;
	scale.output_u_addr = 0;
	scale.output_v_addr = 0;

	scale.fifo_mode = C_SCALER_CTRL_FIFO_DISABLE;
	scale.scale_mode = C_SCALER_FULL_SCREEN;
	scale.digizoom_m = 10;
	scale.digizoom_n = 10;
	para.boundary_color = 0x008080;

	drv_l2_scaler_trigger(SCALER_0, 1, &scale, &para);
}

void Camera_Process_Demo(void)
{
	CAM_SYSTEM_INFO system;
	ENCCFG enc_cfg;
	INT32U i,size;
	osMessageQDef_t task_q_def = {7, sizeof(INT32U), 0};
	INT32U BufferA = 0, BufferB = 0, BufferCopy = 0;
	INT32U out_idx = 0;
	INT32U tickcount = 0;
	INT32U starttime = 0, timestamp;
	INT16S File;
	INT32U bDisk = 0;
	INT32U rec_time =0;
	INT8U filename[128];
	INT32U fidx = 0;


    DBG_PRINT("/*************************************************/\r\n");
    DBG_PRINT("/* Camera Process Demo Guide                     */\r\n");
    DBG_PRINT("/* This demo needs to run on GP DVP board with H42 in MIPI i/f and GC0308 in CCIR i/f*/\r\n");
    DBG_PRINT("/* in board_config.h                             */\r\n");
    DBG_PRINT("/* CSI_MIPI_CLKO_POS to CSI_MIPI_CLKO_MUX1       */\r\n");
    DBG_PRINT("/* CSI_DATA_2_9_POS to CSI_DATA2_9_MUX1          */\r\n");
    DBG_PRINT("/* CSI_CTRL_POS to CSI_CTRL_MUX1                 */\r\n");
    DBG_PRINT("/* SCL & SDA for H42 is IOB4/IOB5                */\r\n");
    DBG_PRINT("/* SCL & SDA for GC0308 is IOE2/IOE3             */\r\n");
    DBG_PRINT("/* configure CP_DEMO_EN as 1 in drv_l2_sensor.h  */\r\n");
        DBG_PRINT("/**********************************************/\r\n");



	//initla file system
	while(1)
	{
		if( _devicemount(FS_SD2))
		{
			DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD2);
			_deviceunmount(FS_SD2);
			break;
		}
		else
		{
			DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD2);
			bDisk = 1;
			break;
		}
	}

	/*call camera_system_init to get sensor information.
	  make sure sensor drvier is correctly setup.*/

	if (STATUS_FAIL == camera_system_init(&system))
		return;

	DBG_PRINT("camera number = %d\r\n", system.camera_num);

	//maximum camera number is 2
	for (i = 0; i < system.camera_num; i++)
	{
		CAMCFG cfg;
		DBG_PRINT("camera %d : %s interface=%d\r\n", i, system.camera_info[i].name, system.camera_info[i].interface);

		//setup sensor input & output resolution.
		if (system.camera_info[i].interface == CDSP_INTERFACE)
		{
			//cdsp sensor will use pscaler if input and output resolution are didfferent.
			cfg.enable = 1;
			cfg.input_width = CDSP_IN_WIDTH;
			cfg.input_height = CDSP_IN_HEIGHT;	//width & height must 16 pixel alignment
			cfg.output_width = CDSP_OUT_WIDTH;
			cfg.output_height = CDSP_OUT_HEIGHT;
			cfg.output_format = V4L2_PIX_FMT_YUYV;
			cdsp_idx = i;
		}
		else //CSI_INTERFACE
		{
			//csi sensor will use scaler if input and output resolution are different.
			cfg.enable = 1;
			cfg.input_width = CSI_IN_WIDTH;
			cfg.input_height = CSI_IN_HEIGHT;
			cfg.output_width = CSI_OUT_WIDTH;
			cfg.output_height = CSI_OUT_HEIGHT;
			cfg.output_format = V4L2_PIX_FMT_YUYV;
			csi_idx = i;
		}
		//sensor frames will be send through callback function.
		cfg.callback = CAM_callback;
		//init sensor
		camera_init(i, &cfg);
	}

	//setup encoder (video & audio)
	enc_cfg.enable_audio = 1;
	enc_cfg.aFormat = WAV; //IMA_ADPCM
	enc_cfg.inFormat = V4L2_PIX_FMT_YUYV;
	enc_cfg.buf_width = ENCODE_WIDTH;
	enc_cfg.buf_height = ENCODE_HEIGHT;
	enc_cfg.enc_width = ENCODE_WIDTH;
	enc_cfg.enc_height = ENCODE_HEIGHT;
	enc_cfg.quality = 20; //for mjpeg quality, 10~100
	enc_cfg.fps = VIDEO_FPS;
	enc_cfg.sampleRate = 44100; //audio sample rate
	enc_cfg.scale_out = 1; //enable scaler out
	enc_cfg.scale_out_format = V4L2_PIX_FMT_RGB565;
	enc_cfg.scale_out_width = 320;
	enc_cfg.scale_out_height = 240;
	enc_cfg.callback = ENC_callback; //callback for scaler out

	camera_encode_init(&enc_cfg);

	//initial sensor buffers
	size = CDSP_OUT_WIDTH*CDSP_OUT_HEIGHT*2;
	cam_bufA[0] = (INT32U)gp_malloc(size*3);
	cam_bufA[1] = cam_bufA[0] + size;
	cam_bufA[2] = cam_bufA[1] + size;
	size = CSI_OUT_WIDTH*CSI_OUT_HEIGHT*2;
	cam_bufB[0] = (INT32U)gp_malloc(size*3);
	cam_bufB[1] = cam_bufB[0] + size;
	cam_bufB[2] = cam_bufB[1] + size;
	//initial display buffers
	size = 320*240*2;
	disp_buf[0] = (INT32U)gp_malloc(size*3);
	disp_buf[1] = disp_buf[0] + size;
	disp_buf[2] = disp_buf[1] + size;
	//initial video out buffers
	size = ENCODE_WIDTH*ENCODE_HEIGHT*2;
	dst_buf[0] = (INT32U)gp_malloc(size*3);
	dst_buf[1] = dst_buf[0] + size;
	dst_buf[2] = dst_buf[1] + size;
	gp_memset((void*)dst_buf[0], 0, size*3);

	drv_l2_display_init();
    drv_l2_display_start(DISDEV_TFT, DISP_FMT_RGB565);

	task_q = osMessageCreate(&task_q_def, NULL);

	//start sensors with initial buffers
	camera_start(csi_idx, cam_bufB[0], cam_bufB[1]);
	camera_start(cdsp_idx, cam_bufA[0], cam_bufA[1]);

	//initial video muxer with file handle
	if (bDisk)
	{
		sprintf(filename, "K:\\test%d.avi", fidx++);
		File = fs_open(filename, O_WRONLY|O_CREAT|O_TRUNC);
		if (0 > camera_video_muxer_start(File))
			goto _Exit;
		DBG_PRINT("start record %s\r\n", filename);
	}

	record_start = 1;
	starttime = xTaskGetTickCount();

	while(1)
	{
		osEvent event;
		INT32U offset;
		INT32U tick;
		INT32U dummy_count;

		//wait for sensor frames
		event = osMessageGet(task_q, osWaitForever);
		if(event.status != osEventMessage)
			continue;

		//get recording time demo, in seconds
		tick = camera_video_muxer_get_recording_time();
		if (rec_time != tick)
		{
			rec_time = tick;
			DBG_PRINT("[%d]", rec_time);
		}
		//get estimated recording dummy frame count (possible dummy frame insert to video file due to unknown reason frame drop)
		dummy_count = camera_video_muxer_get_dummy_frame_count();
		if (dummy_count > VIDEO_FPS)
			DBG_PRINT("(%d)", dummy_count);

		offset = event.value.v;

		if (offset & 0x100) //csi
		{
			BufferB = cam_bufB[offset&0xF];
			timestamp = time_bufB[offset&0xF];
		}
		else
		{
			BufferA = cam_bufA[offset&0xF];
			timestamp = time_bufA[offset&0xF];
		}

		//for this demo, just copy frame form both sensors to one buffer.

		if (BufferA)
		{
			//tick = xTaskGetTickCount();
			//drv_l1_dma_buffer_copy(BufferA, dst_buf[out_idx], CDSP_OUT_WIDTH*CDSP_OUT_HEIGHT*2, CDSP_OUT_WIDTH*2, ENCODE_WIDTH*2);
			do_scaler(BufferA, CDSP_OUT_WIDTH, CDSP_OUT_HEIGHT, dst_buf[out_idx], CDSP_OUT_WIDTH, CDSP_OUT_HEIGHT, CSI_OUT_WIDTH);
			//tick = (xTaskGetTickCount() - tick);
			BufferA = 0;
			BufferCopy |= 0x1;
		}
		if (BufferB)
		{
			//tick = xTaskGetTickCount();
			//drv_l1_dma_buffer_copy(BufferB, dst_buf[out_idx]+(CDSP_OUT_WIDTH*2), CSI_OUT_WIDTH*CSI_OUT_HEIGHT*2, CSI_OUT_WIDTH*2, ENCODE_WIDTH*2);
			do_scaler(BufferB, CSI_OUT_WIDTH, CSI_OUT_HEIGHT, dst_buf[out_idx]+(CDSP_OUT_WIDTH*2), CSI_OUT_WIDTH, CSI_OUT_HEIGHT, CDSP_OUT_WIDTH);
			//tick = (xTaskGetTickCount() - tick);
			BufferB = 0;
			BufferCopy |= 0x2;
		}
		//tickcount += tick;
		//tick = 0;

		//if both buffer copied, send to encoder with timestamp, set both destination buffer and scale out buffer (for display)
		//timestamp is in millisecond counted from camera_system_init()
		if (BufferCopy == 0x3)
		{
			//DBG_PRINT("copy tick = %d\r\n", tickcount);
			if (STATUS_OK == camera_encode_out(dst_buf[out_idx], timestamp, disp_buf[out_idx]))
			{
				out_idx++;
				if (out_idx > 2)
					out_idx = 0;
			}
			BufferCopy = 0;
			tickcount = 0;
		}

		//reocording to 15 seconds then stop muxer, set callback function for notification of muxer stopped.
		if (bDisk && (xTaskGetTickCount() - starttime > 10*1000) ||
			(camera_system_status() == VIDEO_CODEC_PROCESS_END))
		{
			//get error code
			INT32U error = camera_system_error();
			if (error)
			{
				if (error & CP_ERROR_ENCODE_ERROR)		DBG_PRINT("video encoder error\r\n");
				if (error & CP_ERROR_DISK_FULL)			DBG_PRINT("disk full\r\n");
				if (error & CP_ERROR_FILE_SIZE_LIMIT)	DBG_PRINT("recording reach max file size\r\n");
				if (error & CP_ERROR_MUX_ERROR)			DBG_PRINT("unknow packer error\r\n");
			}
			DBG_PRINT("stop muxer\r\n");
			//call audio drop frame to pause audio recording before stop
			camera_audio_drop_frame();
			camera_video_muxer_stop();

			if (fidx < 4)
			{
				sprintf(filename, "K:\\test%d.avi", fidx++);
				File = fs_open(filename, O_WRONLY|O_CREAT|O_TRUNC);
				if (0 > camera_video_muxer_start(File))
					break;
				starttime = xTaskGetTickCount();
				DBG_PRINT("start record %s\r\n", filename);
			}
			else
				break;
		}
	}

	muxer_stop = 0;
	record_start = 0;

	//wait for muxer stop then close file.
	if (bDisk)
	{
		_deviceunmount(FS_SD2);
	}

	//stop sensor after finish sd writing.
	camera_stop(csi_idx);
	camera_stop(cdsp_idx);
_Exit:
	//release resource
	camera_encode_close();
	camera_system_close();

	vQueueDelete(task_q);

	DBG_PRINT("Camera Processorr Demo Close\r\n");
}
