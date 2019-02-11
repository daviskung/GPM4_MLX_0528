#include <stdio.h>
#include <string.h>
#include "application.h"
#include "gplib.h"
#include "drv_l2_ad_key_scan.h"
#include "drv_l1_ppu.h"
#include "drv_l2_display.h"
#include "gplib_ppu.h"
#include "gplib_ppu_sprite.h"
#include "gplib_ppu_text.h"
#include "Sprite_demo.h"
#include "Text_demo.h"
#include "FaceDetectAP.h"
#include "video_encoder.h"
#include "task_video_decoder.h"
#include "drv_l1_timer.h"

#define USE_DISK	FS_SD2
//#define USE_DISK    FS_NAND1

#define DV_PLAY			0
#define DV_RECORD		1

#define ENCODE_MJPEG_X1_5      0

#if ENCODE_MJPEG_X1_5 == 1
#define TAR_WIDTH		1280
#define TAR_HEIGHT		720
#define SNR_WIDTH		1280
#define SNR_HEIGHT		720
#else
#if 0//VGA sensor
#define TAR_WIDTH		640
#define TAR_HEIGHT		480
#define SNR_WIDTH		640
#define SNR_HEIGHT		480
#else//HD sensor
#define TAR_WIDTH		1280
#define TAR_HEIGHT		720
#define SNR_WIDTH		1280
#define SNR_HEIGHT		720
#endif
#endif

#define H264_TAR_BITRATE	7200
#define H264_GOP_LENGTH		10

#define DIP_WIDTH		640/2
#define DIP_HEIGHT		480/2
#define FPS				30
#define SPR				16000
#define C_FILE_NODE_SIZE			500
static INT16U	file_node_buf[C_FILE_NODE_SIZE];
static struct sfn_info file_fs_info;
static struct STFileNodeInfo musicFNodeInfo;

//=================================================================================================
//	Digital_Video_With_Prcess_Demo code
//=================================================================================================
#define DISP_BUFFER_POST_EN                 1
#define SENSOR_SRC_WIDTH		            1280//2560//1280//640
#define SENSOR_SRC_HEIGHT		            720//1928//720//480
#define PRCESS_SRC_WIDTH		            1280//640//320//SENSOR_SRC_WIDTH
#define PRCESS_SRC_HEIGHT		            720//480//240//SENSOR_SRC_HEIGHT
#define DISP_SRC_WIDTH		                PRCESS_SRC_WIDTH
#define DISP_SRC_HEIGHT		                PRCESS_SRC_HEIGHT
#define JPEG_SRC_WIDTH		                1280//2560//1280//640
#define JPEG_SRC_HEIGHT		                720//1920//720//480
#define VIDEO_FPS				            25
#define AUDIO_SPR				            16000
#define PRCESS_STATE_OK                     0x80
#define ACK_OK			                    0
#define ACK_FAIL		                    (-1)
#define MSG_PRCESS_TASK_EXIT                0xFF
#define C_PPU_DRV_FRAME_NUM		            3
#define FRAME_BUF_ALIGN64                   0x3F
#define FRAME_BUF_ALIGN32                   0x1F
static osThreadId prcess_id;
static osThreadId disp_id;
static xQueueHandle prcess_frame_buffer_queue = NULL;
static xQueueHandle prcess_state_queue = NULL;
static xQueueHandle disp_frame_buffer_queue = NULL;
static xQueueHandle prcess_task_ack_m = NULL;
#if DISP_BUFFER_POST_EN == 1
static xQueueHandle prcess_free_post_queue = NULL;
static INT32U prcess_free_base_addr = NULL;
static INT32U prcess_free_post = NULL;
#endif
static PPU_REGISTER_SETS ppu_register_structure;
static PPU_REGISTER_SETS *ppu_register_set;
static INT16U disp_h_size,disp_v_size;
static retry = 0;
typedef struct
{
	gpImage ScaleIn;
	gpImage ScaleOut;
} ObjFrame_t;

typedef struct {
	INT32U ppu_frame_workmem;
	INT32U ppu_narray_workmem;
    INT32U prcess_post_workmem;
    INT32U disp_prcess_workmem;
} prcess_mem_t;
static prcess_mem_t prcess_mem_structure;
static prcess_mem_t *prcess_mem_set;

#define POST_MESSAGE(queue, message, ack_mbox, msc_time)\
{\
	{\
		INT32U send_msg;\
		osEvent result;\
		osStatus status;\
		result = osMessageGet(ack_mbox, 1);\
		send_msg = message;\
		status = osMessagePut(queue, (INT32U)&send_msg, msc_time);\
		if(status != osOK)\
		{\
			DBG_PRINT("PutMsg Fail!!!\r\n");\
			RETURN(STATUS_FAIL);\
		}\
		result = osMessageGet(ack_mbox, msc_time);\
		if((result.status != osEventMessage) || (result.value.v == ACK_FAIL))\
		{\
			DBG_PRINT("GetMsg ack Fail!!! message %x\r\n",message);\
			RETURN(STATUS_FAIL);\
		}\
	}\
}

#define OSQFlush(x)\
{\
    while(1) {\
        osEvent result;\
        result = osMessageGet(x, 1);\
        if(result.status != osEventMessage) {\
            break;\
        }\
    }\
}

//=================================================================================================
//	DV demo code
//=================================================================================================
static INT32U Display_Callback(INT16U w, INT16U h, INT32U addr)
{
    R_TFT_FBI_ADDR = addr;
}

void Digital_Video_Demo(void)
{
	char  path[64];
	INT8U OperationMode;
	INT8S volume, play_index, pause ;
	INT32U  zoom_ratio;
	INT32S nRet;
	INT32S index = 0;
	INT32U folder_nums, file_nums;
	VIDEO_CODEC_STATUS status;
	VIDEO_INFO	information;
	VIDEO_ARGUMENT arg;
	MEDIA_SOURCE   src;

	while(1)
	{
		if( _devicemount(USE_DISK))
		{
			DBG_PRINT("Mount Disk Fail[%d]\r\n", USE_DISK);
	#if	USE_DISK == FS_NAND1
			nRet = DrvNand_lowlevelformat();
			DBG_PRINT("NAND LOW LEVEL FORMAT = %d \r\n", nRet);
			nRet = _format(FS_NAND1, FAT32_Type);
			DBG_PRINT("Format NAND = %d\r\n", nRet);
			DrvNand_flush_allblk();
	#endif
			_deviceunmount(USE_DISK);
		}
		else
		{
			DBG_PRINT("Mount Disk success[%d]\r\n", USE_DISK);
			break;
		}
	}

	//tft_init();
	//tv_init();

	adc_key_scan_init(); //init key scan

	// Initialize display device
    drv_l2_display_init();
    drv_l2_display_start(DISDEV_TFT, DISP_FMT_YUYV);
    video_encode_register_display_callback(Display_Callback);
    video_decode_register_display_callback(Display_Callback);

	//user_defined_video_codec_entrance();

	//init as dv play mode
	volume = 0x3F;
	play_index = 0;
	zoom_ratio = 10;
	nRet = 0;
	pause = 0;
	OperationMode = DV_RECORD;

    memset((void*)&arg, 0, sizeof(VIDEO_ARGUMENT));
	video_encode_entrance();
	arg.bScaler = 1;
	arg.TargetWidth = TAR_WIDTH;
	arg.TargetHeight = TAR_HEIGHT;
	arg.SensorWidth	= SNR_WIDTH;
	arg.SensorHeight = SNR_HEIGHT;
	arg.DisplayWidth = DIP_WIDTH;
	arg.DisplayHeight = DIP_HEIGHT;
	arg.VidFrameRate = FPS;
	arg.AudSampleRate = SPR;
	arg.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV; //for display
	//R_FUNPOS1 &= ~((0x3<<8)|(0x3<<3));
	video_encode_preview_start(arg);

    /*OperationMode = DV_PLAY;
	video_decode_entrance();
	musicFNodeInfo.disk = USE_DISK;
	gp_strcpy((INT8S *) musicFNodeInfo.extname, (INT8S *) "mov");
	musicFNodeInfo.FileBuffer = file_node_buf;
	musicFNodeInfo.FileBufferSize = C_FILE_NODE_SIZE;
	musicFNodeInfo.FolderBuffer = NULL;
	musicFNodeInfo.FolderBufferSize = 0;
	GetFileNumEx(&musicFNodeInfo, &folder_nums, &file_nums);*/

    DBG_PRINT("===============================\r\n");
	DBG_PRINT("S8 -> Demo Exit\r\n");
	DBG_PRINT("S1 -> swtich RECORD/PLAY mode\r\n");
	DBG_PRINT("S2 -> start RECORD/PLAY\r\n");
	DBG_PRINT("S3 -> stop RECORD/PLAY\r\n");
	DBG_PRINT("S4 -> pause/resume RECORD/PLAY\r\n");
	DBG_PRINT("S5 -> zoom in/volume up\r\n");
	DBG_PRINT("S6 -> zoom out/volume down\r\n");
	DBG_PRINT("S7 -> capture image\r\n");
	DBG_PRINT("===============================\r\n");

	DBG_PRINT("\r\nDEMO in RECORD Mode\r\n");

	while(1)
	{
		adc_key_scan();
		if(ADKEY_IO1)
		{
			switch(OperationMode)
			{//exit the old mode
			case DV_PLAY:
				if(video_decode_status() == VIDEO_CODEC_PROCESSING)
					video_decode_stop();
				video_decode_exit();
				break;
			case DV_RECORD:
				zoom_ratio = 10;
				video_encode_preview_stop();
				osDelay(10);// wait csi stop and scaler task stop
				video_encode_exit();
				//video_codec_show_image(1, 0, 0,IMAGE_OUTPUT_FORMAT_YUYV);
				drv_l2_display_stop(DISDEV_TFT);
				break;
			}

			if(OperationMode == DV_PLAY)
				OperationMode = DV_RECORD;
			else if(OperationMode == DV_RECORD)
				OperationMode = DV_PLAY;

			switch(OperationMode)
			{//enter new mode
			case DV_PLAY:
				video_decode_entrance();
				musicFNodeInfo.disk = USE_DISK;
				gp_strcpy((INT8S *) musicFNodeInfo.extname, (INT8S *) "avi");
				musicFNodeInfo.FileBuffer = file_node_buf;
				musicFNodeInfo.FileBufferSize = C_FILE_NODE_SIZE;
				musicFNodeInfo.FolderBuffer = NULL;
				musicFNodeInfo.FolderBufferSize = 0;
				GetFileNumEx(&musicFNodeInfo, &folder_nums, &file_nums);
				DBG_PRINT("\r\nDEMO in PLAY Mode\r\n");
				break;

			case DV_RECORD:
				video_encode_entrance();
				arg.bScaler = 1;
				arg.TargetWidth = TAR_WIDTH;
				arg.TargetHeight = TAR_HEIGHT;
				arg.SensorWidth	= SNR_WIDTH;
				arg.SensorHeight = SNR_HEIGHT;
				arg.DisplayWidth = DIP_WIDTH;
				arg.DisplayHeight = DIP_HEIGHT;
				arg.VidFrameRate = FPS;
				arg.AudSampleRate = SPR;
				arg.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV; //for display
				video_encode_preview_start(arg);
				DBG_PRINT("\r\nDEMO in RECORD Mode\r\n");
				drv_l2_display_start(DISDEV_TFT, DISP_FMT_YUYV);
				break;
			}
		}
		else if(ADKEY_IO2)
		{//start play
			switch(OperationMode)
			{
			case DV_PLAY:
				if(video_decode_status() == VIDEO_CODEC_PROCESS_END)
				{
					DBG_PRINT("video_decode_start\r\n");
					GetFileNodeInfo(&musicFNodeInfo, play_index, &file_fs_info);
					DBG_PRINT("FILE = %s\r\n", (char *)file_fs_info.f_name);
					play_index++;
					if(play_index >= file_nums)
						play_index = 0;

					arg.bScaler = TRUE;
					arg.bUseDefBuf = FALSE;
					arg.TargetWidth = DIP_WIDTH;
					arg.TargetHeight = DIP_HEIGHT;
					arg.DisplayWidth = DIP_WIDTH;
                    arg.DisplayHeight = DIP_HEIGHT;
                    arg.DisplayBufferWidth = DIP_WIDTH;
                    arg.DisplayBufferHeight = DIP_HEIGHT;
					arg.OutputFormat = IMAGE_OUTPUT_FORMAT_RGB565;

					src.type = SOURCE_TYPE_FS;
					//src.Format.VideoFormat = MJPEG;//only MJPEG
					src.type_ID.FileHandle = sfn_open(&file_fs_info.f_pos);
					src.type_ID.temp = sfn_open(&file_fs_info.f_pos);

					if((src.type_ID.FileHandle < 0)||(src.type_ID.temp < 0))
					{
						DBG_PRINT("file open fail\r\n");
						fs_close(src.type_ID.FileHandle);
						fs_close(src.type_ID.temp);
						continue;
					}

					status = video_decode_paser_header(&information, arg, src);
					if(status != VIDEO_CODEC_STATUS_OK)
					{
						DBG_PRINT("parser header fail !!!\r\n");
						continue;
					}
					drv_l2_display_start(DISDEV_TFT, DISP_FMT_RGB565);
					video_decode_start(arg, src);
					audio_decode_volume(volume);
				}
				break;
			case DV_RECORD:
				if(video_encode_status() == VIDEO_CODEC_PROCESS_END)
				{
					DBG_PRINT("video_encode_start\r\n");
					if(USE_DISK == FS_SD)
						sprintf((char *)path, (const char *)"C:\\avi_rec%d.avi", nRet++);
					else if(USE_DISK == FS_NAND1)
						sprintf((char *)path, (const char *)"A:\\avi_rec%d.avi", nRet++);
                    else if(USE_DISK == FS_SD2)
						sprintf((char *)path, (const char *)"K:\\avi_rec%d.avi", nRet++);

					src.type = SOURCE_TYPE_FS;
					src.type_ID.FileHandle = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
					if(src.type_ID.FileHandle < 0)
					{
						DBG_PRINT("file open fail\r\n");
						continue;
					}
					else
						DBG_PRINT("file name = %s\r\n", path);
					#if ENCODE_MJPEG_X1_5 == 1
					src.Format.VideoFormat = MJPEG;
					video_encode_set_jpeg_quality(50);
					#else
					src.Format.VideoFormat = H_264;
					video_encode_set_h264_rc(H264_TAR_BITRATE, H264_GOP_LENGTH);
					#endif
					//if (STATUS_OK != video_encode_start(src))
					if (STATUS_OK != video_encode_start(src, 1))
                        DBG_PRINT("video encode start fail\r\n");
				}
				break;
			}
		}
		else if(ADKEY_IO3)
		{//stop
			switch(OperationMode)
			{
			case DV_PLAY:
				if(video_decode_status() == VIDEO_CODEC_PROCESSING)
				{
					DBG_PRINT("video_decode_stop\r\n");
					video_decode_stop();
					drv_l2_display_stop(DISDEV_TFT);
				}
				break;

			case DV_RECORD:
				if(video_encode_status() == VIDEO_CODEC_PROCESSING)
				{
					DBG_PRINT("video_encode_stop\r\n");
					video_encode_stop();
					fs_close(src.type_ID.FileHandle);
					src.type_ID.FileHandle = -1;
			#if USE_DISK == FS_NAND1
					DBG_PRINT("DrvNand_flush_allblk\r\n");
					DrvNand_flush_allblk();
			#endif
				}
				break;
			}
		}
		else if(ADKEY_IO4)
		{	//pause, resume
			switch(OperationMode)
			{
			case DV_PLAY:
                if (pause) {
                    if(video_decode_status() == VIDEO_CODEC_PROCESS_PAUSE)
                    {
                        DBG_PRINT("video_decode_resume\r\n");
                        video_decode_resume();
                    }
                    pause = 0;
                }
                else {
                    if(video_decode_status() == VIDEO_CODEC_PROCESSING)
                    {
                        DBG_PRINT("video_decode_pause\r\n");
                        video_decode_pause();
                        DBG_PRINT("cur time = %d, num = %d\r\n", video_decode_get_current_time(), video_decode_get_current_number());
                    }
                    pause = 1;
                }
				break;

			case DV_RECORD:
                if (pause) {
                    if(video_encode_status() == VIDEO_CODEC_PROCESS_PAUSE)
                    {
                        DBG_PRINT("video encode resume\r\n");
                        video_encode_resume();
                    }
                    pause = 0;
                }
                else {
                    if(video_encode_status() == VIDEO_CODEC_PROCESSING)
                    {
                        DBG_PRINT("video encode pause\r\n");
                        video_encode_pause();
                        DBG_PRINT("cur time = %d\r\n", video_encode_get_recording_time());
                    }
                    pause = 1;
                }
				break;
			}
		}
		else if(ADKEY_IO5)
		{//volume up / zoom in
			switch(OperationMode)
			{
			case DV_PLAY:
				volume++;
				if(volume > 0x3F)
				{
					volume = 0x3F;
					continue;
				}
				DBG_PRINT("volume +\r\n");
				audio_decode_volume(volume);
				break;

			case DV_RECORD:
				zoom_ratio += 1;
		    	if(zoom_ratio > 20)
		    	{
		    		zoom_ratio = 20;
    				continue;
        		}
        		DBG_PRINT("Zoom in (%d)\r\n", zoom_ratio);
        		video_encode_set_zoom_scaler(zoom_ratio);
				break;
			}
		}
		else if(ADKEY_IO6)
		{// volume down / zoom out
			switch(OperationMode)
			{
			case DV_PLAY:
				volume --;
				if(volume < 0)
				{
					volume = 0;
					continue;
				}
				DBG_PRINT("volume -\r\n");
				audio_decode_volume(volume);
				break;

			case DV_RECORD:
				zoom_ratio -= 1;
		    	if(zoom_ratio < 10)
		    	{
		    		zoom_ratio = 10;
		    		continue;
            	}
            	DBG_PRINT("Zoom out (%d)\r\n", zoom_ratio);
            	video_encode_set_zoom_scaler(zoom_ratio);
            	break;
			}
		}
		else if(ADKEY_IO7)
		{
            switch(OperationMode)
            {
            case DV_PLAY:
                break;
            case DV_RECORD:
                if(video_encode_status() == VIDEO_CODEC_PROCESSING){
                    DBG_PRINT("\r\nCannot Capture Image When Recording.\r\n");
                    break;
                }
                src.type = SOURCE_TYPE_FS;
                src.Format.VideoFormat = MJPEG;
                sprintf((char *)path, (const char *)"K:\\pic%03d.jpg", index++);
                DBG_PRINT("file name = %s\r\n", path);

                src.type_ID.FileHandle = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
                if(src.type_ID.FileHandle < 0) {
                    DBG_PRINT("file open fail\r\n");
                    continue;
                }
                DBG_PRINT("video_encode_capture_picture\r\n");
                video_encode_set_jpeg_quality(50);
                status = video_encode_capture_picture(src);
                if(status != START_OK) {
                    DBG_PRINT("video_encode_capture_picture fail!!!\r\n");
                    continue;
                }
                DBG_PRINT("Capture finished\r\n\r\n");
                break;
            }
		}
		else if(ADKEY_IO8)
		{
            switch(OperationMode)
			{//exit the old mode
			case DV_PLAY:
				if(video_decode_status() == VIDEO_CODEC_PROCESSING)
					video_decode_stop();
				video_decode_exit();
				break;
			case DV_RECORD:
				zoom_ratio = 10;
				video_encode_preview_stop();
				osDelay(10);// wait csi stop and scaler task stop
				video_encode_exit();
				//video_codec_show_image(1, 0, 0,IMAGE_OUTPUT_FORMAT_YUYV);
				break;
			}
			break;
		}
	}
    drv_l2_display_stop(DISDEV_TFT);
    drv_l2_display_uninit();
    adc_key_scan_uninit();
	_deviceunmount(USE_DISK);
	DBG_PRINT("\r\nDEMO EXIT\r\n");
}
void Video_Editor(void)
{
	char 	sd_path[64] = "K:\\Demo0091NBA.AVI";
	INT32S  nRet, i,ret;
	VIDEO_CODEC_STATUS status;
	VIDEO_INFO	information;
	VIDEO_ARGUMENT arg;
	MEDIA_SOURCE   src;
	VIDEO_ARGUMENT arg_encode;
	MEDIA_SOURCE   src_encode;
	avi_frame_info avi_frame;
	avi_frame_info audio_frame;
	INT16S   handle,audio_handle;
	INT32U   jpg_index,jpg_count,audio_size,write_size;
	char 	 path[64];
	INT32U   buffer_addr = 0;
	INT32U   buffer_size;
	struct sfn_info sfile_info;
	INT32U dummy_frame = 0;
	INT32U *p_dummy_frame;

	jpg_index = 0;

	while(1)
	{
		if(_devicemount(USE_DISK))					// Mount device
		{
			DBG_PRINT("Mount Disk Fail[%d]\r\n", USE_DISK);
		#if USE_DISK == FS_NAND1
			nRet = _format(FS_NAND1, FAT32_Type);
			DrvNand_flush_allblk();
			_deviceunmount(FS_NAND1);
		#endif
		}
		else
		{
			DBG_PRINT("Mount Disk success[%d]\r\n", USE_DISK);
			break;
		}
	}
	timer_start_without_isr(TIMER_F,TIMER_SOURCE_1024HZ);
	video_decode_entrance();     // Initialize and allocate the resources needed by Video decoder
//=============下面的是將avi拆分為單?jpg===================================================================

    if(video_decode_status() != VIDEO_CODEC_PROCESS_END)
        video_decode_stop();

    if(USE_DISK == FS_SD2)
        src.type_ID.FileHandle = fs_open(sd_path, O_RDONLY);	//open file handle

    if(src.type_ID.FileHandle < 0)
    {
        DBG_PRINT("file open fail,errno = %d\r\n",gFS_errno);
        fs_close(src.type_ID.FileHandle);
        while(1);
    }

    src.type = SOURCE_TYPE_FS;	//play file by file system
    src.Format.VideoFormat = MJPEG;

    DBG_PRINT("video_decode_start\r\n");
    status = video_decode_paser_header(&information, arg, src);
    if(status != VIDEO_CODEC_STATUS_OK)
    {
        DBG_PRINT("paser header fail !!!\r\n");

    }
    DBG_PRINT("Aud SampleRate = %d\r\n", information.AudSampleRate);
    DBG_PRINT("Vid FrameRate = %d\r\n", information.VidFrameRate);
    DBG_PRINT("Vid video_total_time = %d\r\n", information.TotalDuration);
    DBG_PRINT("resolution = %d x %d\r\n", information.Width, information.Height);
    DBG_PRINT("Total Time = %d seconds\r\n", information.TotalDuration);

	if(information.VidFrameRate != 0)
	{
		ret = 4*information.VidFrameRate * information.TotalDuration;
		dummy_frame = (INT32U)gp_malloc_align(ret,4);
		if(dummy_frame == 0)
		{
			while(1);
		}
		gp_memset((INT8S *)dummy_frame,0xff,ret);
		p_dummy_frame = (INT32U *)dummy_frame;
	}

	nRet = chdir("K://");
	if(nRet < 0)
	{
		while(1);
	}

	nRet = chdir("dsc2");
	if(nRet < 0)
	{
		nRet = mkdir("dsc2");
		if(nRet < 0)
		{
			while(1);
		}
		nRet = chdir("dsc2");
		if(nRet < 0)
		{
			while(1);
		}
	}
//	video_decode_set_play_time(2);
#if 0
	ret = 1;
	while(ret)
	{
		nRet = video_decode_get_one_frame(&avi_frame);
		if(nRet == STATUS_FAIL)
		{
			while(1);
		}
		if(avi_frame.FrameSize == 0xFFFFFFFF)
		{
			ret = 0;
			DBG_PRINT("decode end\r\n");
			continue;
		}
		else if(avi_frame.FrameSize == 0xFFFFFFFe)
		{
			DBG_PRINT("decode dummy_frame %d\r\n",jpg_index);
			*p_dummy_frame =  jpg_index;
			jpg_index ++;
			p_dummy_frame ++;

		}
		else
		{
			DBG_PRINT("index = %d\r\n",jpg_index);
			sprintf((char *)path, (const char *)"jpg_%04d.jpg", jpg_index++);
			handle = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
			if(handle < 0)
			{
				DBG_PRINT("file open fail\r\n");
				fs_close(handle);
				while(1);
			}
			nRet = fs_write(handle,avi_frame.AddrFrame,avi_frame.FrameSize);
			if(nRet < 0)
			{
				DBG_PRINT("file fs_write fail\r\n");
				while(1);
			}
			fs_close(handle);
			handle = -1;
		}
		for(i = 0;i < (avi_frame.Duration - 1);i ++)
		{
			DBG_PRINT("decode dummy_frame2 %d\r\n",jpg_index);
			*p_dummy_frame =  jpg_index;
			jpg_index ++;
			p_dummy_frame ++;
		}
	}

#endif
//============================下面是将声音提取出来=========================================================

	sprintf((char *)path, (const char *)"audio.bin");
#if 0
	audio_handle = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
	DBG_PRINT("open end = %d\r\n",TimerCountReturn(TIMER_F));
	if(audio_handle < 0)
	{
		DBG_PRINT("file open fail\r\n");
		fs_close(audio_handle);
		while(1);
	}
	audio_frame.AddrFrame = (INT32U)gp_malloc_align(PARSER_AUD_BITSTREAM_SIZE,4);
	if(!audio_frame.AddrFrame)
	{
		DBG_PRINT("gp_malloc_align fail\r\n");
		while(1);
	}
	audio_frame.RI = 0;
	avi_audio_init();
	ret = 1;
	while(ret)
	{
		audio_frame.FrameSize = 0;
		nRet = avi_audio_get_one_frame(&audio_frame);
		if(nRet == STATUS_FAIL)
		{
			//ret = 0;
			//continue;
		}
		if(audio_frame.FrameSize == 0xFFFFFFFF)
		{
			DBG_PRINT("decode end\r\n");
			ret = 0;
			continue;
		}
		else if(audio_frame.FrameSize == 0)
		{
			continue;
		}
		else
		{
			nRet = fs_write(audio_handle,audio_frame.AddrFrame,audio_frame.FrameSize);
			DBG_PRINT("fs_write end = %d\r\n",TimerCountReturn(TIMER_F));
			if(nRet < 0)
			{
				DBG_PRINT("file fs_write fail\r\n");
				while(1);
			}
		}
	}
	fs_close(audio_handle);
	gp_free((void *)audio_frame.AddrFrame);
	DBG_PRINT("fs_close end = %d\r\n",TimerCountReturn(TIMER_F));
	handle = -1;
#endif
	fs_close(src.type_ID.FileHandle);
	video_decode_parser_stop();
//======================下面的部份是將jpg encpde為avi的===========================================
#if 1
    //video_encode_entrance();
    avi_encode_init();
	audio_handle = fs_open(path, O_RDONLY);
	DBG_PRINT("open end = %d\r\n",TimerCountReturn(TIMER_F));
	if(audio_handle < 0)
	{
		DBG_PRINT("file open fail\r\n");
		fs_close(audio_handle);
		while(1);
	}
	nRet = sfn_stat(audio_handle,&sfile_info);
	if(nRet < 0)
	{
		while(1);
	}
	audio_size = sfile_info.f_size;

	nRet = 0;
	jpg_count = jpg_index ;
	if(0 == jpg_count)
	{
        jpg_count = 590 ;
	}
	jpg_index = 0;

	buffer_size = 64*1024;
	buffer_addr = (INT32U)gp_malloc_align(buffer_size,4);
	if(buffer_addr == 0)
	{
		DBG_PRINT("gp_malloc_align fail\r\n");
		while(1);
	}
	arg_encode.bScaler = 1;	// must be 1
	arg_encode.TargetWidth = information.Width; 	//encode width
	arg_encode.TargetHeight = information.Height;	//encode height

	arg_encode.VidFrameRate = information.VidFrameRate;	//video encode frame rate
	arg_encode.AudSampleRate = information.AudSampleRate;//audio record sample rate
	arg_encode.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV; //display output format

	sprintf((char *)path, (const char *)"K:\\avi_rec%d.avi", nRet++);

	src_encode.type = SOURCE_TYPE_FS;
	src_encode.Format.VideoFormat = MJPEG;
	src_encode.type_ID.FileHandle = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
	if(src_encode.type_ID.FileHandle < 0)
	{
		DBG_PRINT("file creat fail\r\n");
		fs_close(src_encode.type_ID.FileHandle);
		while(1);
	}
	video_encode_packer_start(src_encode,arg_encode);

	nRet = chdir("dsc2");
	if(nRet < 0)
	{
		nRet = mkdir("dsc2");
		if(nRet < 0)
		{
			while(1);
		}
		nRet = chdir("dsc2");
		if(nRet < 0)
		{
			while(1);
		}
	}
	p_dummy_frame = (INT32U *)dummy_frame;
	while(1)
	{
		DBG_PRINT("index = %d\r\n",jpg_index);

		if((*p_dummy_frame) == jpg_index)
		{
			DBG_PRINT("encode dummy_frame = %d\r\n",jpg_index);
			p_dummy_frame ++;
			jpg_index ++;
			nRet = video_encode_one_frame(1,0,0,0);
			if(nRet < 0)
			{
				while(1);
			}
		}
		else
		{
			sprintf((char *)path, (const char *)"jpg_%04d.jpg", jpg_index++);
			if(jpg_index > jpg_count)
			{
				break;
			}

			handle = fs_open(path, O_RDONLY);
			DBG_PRINT("open end = %d\r\n",TimerCountReturn(TIMER_F));
			if(handle < 0)
			{
				DBG_PRINT("file open fail\r\n");
				fs_close(handle);
				while(1);
			}
			nRet = sfn_stat(handle,&sfile_info);
			if(nRet < 0)
			{
				while(1);
			}
			if(buffer_size < sfile_info.f_size)
			{
				gp_free((void *)buffer_addr);
				buffer_size = sfile_info.f_size;
				buffer_addr = (INT32U)gp_malloc_align(buffer_size,4);
				if(buffer_addr == 0)
				{
					DBG_PRINT("gp_malloc_align fail\r\n");
					while(1);
				}
			}

			nRet = fs_read(handle,buffer_addr,sfile_info.f_size);
			if(nRet < 0)
			{
				DBG_PRINT("file fs_read fail\r\n");
				while(1);
			}
			fs_close(handle);
			handle = -1;

			nRet = video_encode_one_frame(1,1,buffer_addr,sfile_info.f_size);
			if(nRet < 0)
			{
				while(1);
			}

			if((jpg_index % information.VidFrameRate) == 0)
			{
				if(audio_size == 0)
				{
					continue;
				}
				else if(audio_size >= arg_encode.AudSampleRate * 2)
				{
					nRet = fs_read(audio_handle,buffer_addr,arg_encode.AudSampleRate * 2);
					if(nRet < 0)
					{
						DBG_PRINT("fs_read fail\r\n");
						while(1);
					}
					write_size = arg_encode.AudSampleRate * 2;
				}
				else
				{
					nRet = fs_read(audio_handle,buffer_addr,audio_size);
					if(nRet < 0)
					{
						DBG_PRINT("fs_read fail\r\n");
						while(1);
					}
					write_size = audio_size;
				}
				audio_size -= write_size;

				nRet = video_encode_one_frame(0,1,buffer_addr,write_size);
				if(nRet < 0)
				{
					while(1);
				}
			}
		}
	}
	fs_close(audio_handle);
	video_encode_packer_stop();
	if(0 != dummy_frame)
	{
        gp_free((void*)dummy_frame);
        dummy_frame = 0;
	}
	if(0 != buffer_addr)
	{
        gp_free((void*)buffer_addr);
        buffer_addr = 0;
	}
	video_decode_exit();
	DBG_PRINT("video_encode_packer_stop\r\n");
#endif
//========================================================================================================
	while(1);
}
//=================================================================================================
//	Digital_Video_With_Prcess_Demo code
//=================================================================================================
// prcess state queue
static INT32S prcess_state_post(INT32U state)
{
    INT32U event,temp;

    event = (INT32U)state;
    temp = osMessagePut(prcess_state_queue, (uint32_t)&event, osWaitForever);

	return temp;
}
static INT32S prcess_state_get(void)
{
    osEvent result;
	INT32S state = 0;

    result = osMessageGet(prcess_state_queue, 10);
    state = result.value.v;
    if((result.status != osEventMessage) || (state!=PRCESS_STATE_OK)) {
        state = 0;
	}

	return state;
}

// prcess ready buffer queue
static INT32S prcess_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(prcess_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(prcess_frame_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}

#if DISP_BUFFER_POST_EN == 1
// prcess free buffer queue
static INT32S prcess_free_post_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(prcess_free_post_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(prcess_free_post_queue, (uint32_t)&event, 10);
    }

	return temp;
}

static INT32S prcess_free_post_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(prcess_free_post_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(prcess_free_post_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}
#endif

// disp ready buffer queue
static INT32S disp_frame_buffer_add(INT32U *frame_buf, INT32U wait)
{
    INT32U event,temp;

	if (!frame_buf) {
		return -1;
	}

    if(wait)
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(disp_frame_buffer_queue, (uint32_t)&event, osWaitForever);
    }
    else
    {
        event = (INT32U)frame_buf;
        temp = osMessagePut(disp_frame_buffer_queue, (uint32_t)&event, 10);
    }

	return temp;
}

static INT32S disp_frame_buffer_get(INT32U wait)
{
    osEvent result;
	INT32S frame = 0;

    if(wait)
    {
        result = osMessageGet(disp_frame_buffer_queue, osWaitForever);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }
    else
    {
        result = osMessageGet(disp_frame_buffer_queue, 10);
        frame = result.value.v;
        if(result.status != osEventMessage || !frame)
            frame = 0;
    }

	return frame;
}

static INT32S fd_display_set_frame(INT32U in_buffer, INT32U frame_buffer, INT16U in_w, INT16U in_h, INT16U out_w, INT16U out_h)
{
    ObjFrame_t *pInput,Input;

 	if(frame_buffer == 0 || in_buffer == 0) {
		return -1;
	}

    pInput = (ObjFrame_t *)&Input;
	if(pInput == 0) {
		return -1;
	}

	pInput->ScaleIn.width = in_w;
	pInput->ScaleIn.height = in_h;
    pInput->ScaleIn.ch  = 2;
    pInput->ScaleIn.widthStep = in_w * 2;
	pInput->ScaleIn.format = IMG_FMT_UYVY;
	pInput->ScaleIn.ptr = (INT8U *)in_buffer;

	pInput->ScaleOut.width = out_w;
	pInput->ScaleOut.height = out_h;
    pInput->ScaleOut.ch  = 2;
    pInput->ScaleOut.widthStep = out_w * 2;
	pInput->ScaleOut.format = IMG_FMT_UYVY;
	pInput->ScaleOut.ptr = (INT8U *)frame_buffer;
    drv_l2_scaler_full_screen(0, 1, (gpImage *)&pInput->ScaleIn, (gpImage *)&pInput->ScaleOut);

    return 0;
 }

static void prcess_task_entry(void const *parm)
{
    INT32U i,prcess_buf,PscalerBuffer,PscalerBufferSize,ack_msg;
    osEvent result;

    DBG_PRINT("prcess_task_entry start \r\n");

    // disp size
    if((disp_h_size != DISP_SRC_WIDTH) && (disp_v_size != DISP_SRC_HEIGHT))
    {
        PscalerBufferSize = (disp_h_size * disp_v_size * 2);
        PscalerBuffer = (INT32U)gp_malloc_align(((PscalerBufferSize*C_PPU_DRV_FRAME_NUM)+64), 32);
        if(PscalerBuffer == 0)
        {
            DBG_PRINT("dispBuffer fail\r\n");
            while(1);
        }
        PscalerBuffer = (INT32U)((PscalerBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
        prcess_mem_set->disp_prcess_workmem = PscalerBuffer;
        for(i=0; i<C_PPU_DRV_FRAME_NUM; i++)
        {
            prcess_buf = (PscalerBuffer+i*PscalerBufferSize);
            disp_frame_buffer_add((INT32U *)prcess_buf,1);
            DBG_PRINT("dispBuffer[%d]:0x%X \r\n",i,prcess_buf);
        }
    }

#if DISP_BUFFER_POST_EN == 1
    // prcess free size
	PscalerBufferSize = (PRCESS_SRC_WIDTH * PRCESS_SRC_HEIGHT * 2);
	PscalerBuffer = (INT32U)gp_malloc_align(((PscalerBufferSize*C_PPU_DRV_FRAME_NUM)+64), 32);
    if(PscalerBuffer == 0)
    {
        DBG_PRINT("prcess_free_post_Buffer fail\r\n");
        while(1);
    }
	PscalerBuffer = (INT32U)((PscalerBuffer + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
	prcess_mem_set->prcess_post_workmem = PscalerBuffer;
	for(i=0; i<C_PPU_DRV_FRAME_NUM; i++)
	{
		prcess_buf = (PscalerBuffer+i*PscalerBufferSize);
		prcess_free_post_add((INT32U *)prcess_buf,1);
		DBG_PRINT("prcess_free_post_Buffer[%d]:0x%X \r\n",i,prcess_buf);
	}
    drv_l2_display_buffer_set(DISPLAY_DEVICE, prcess_buf);
#endif

    prcess_state_post(PRCESS_STATE_OK);

    while(1)
    {
        result = osMessageGet(prcess_frame_buffer_queue, osWaitForever);
        prcess_buf = result.value.v;
        if((result.status != osEventMessage) || !prcess_buf) {
            continue;
        }
        //DBG_PRINT("prcess_buf = 0x%x\r\n", prcess_buf);
        //DBG_PRINT("P");
        switch(prcess_buf)
        {
            case MSG_PRCESS_TASK_EXIT:
                DBG_PRINT("[MSG_PRCESS_TASK_EXIT]\r\n");
                ack_msg = ACK_OK;
                osMessagePut(prcess_task_ack_m, (INT32U)&ack_msg, osWaitForever);
                osThreadTerminate(prcess_id);
                break;

            default:
            //**************************************//
                //DBG_PRINT("user code add \r\n");
            //**************************************//
            osDelay(300);
        #if DISP_BUFFER_POST_EN == 1
                prcess_free_post_add((INT32U *)prcess_buf, 1);
        #else
                gplib_ppu_frame_buffer_add(ppu_register_set, prcess_buf);
        #endif
                prcess_state_post(PRCESS_STATE_OK);
        }
    }
}

#if DISP_BUFFER_POST_EN == 0
static INT32U prcess_ppu_go(INT32U x, INT32U y, INT32U frame_buffer)
{
    #if 1
        gplib_ppu_text_calculate_number_array(ppu_register_set, C_PPU_TEXT1, x, y, (INT32U)frame_buffer);	// Calculate Number array
        gplib_ppu_go_and_wait_done(ppu_register_set);
        return ppu_frame_buffer_display_get();
    #else
        R_PPU_ENABLE = 0x300580;
        R_TFT_FBI_ADDR = frame_buffer;
    #endif
}
#endif

static INT32U Frame_Post_Callback(INT32U addr)
{
    return  prcess_free_post;
}

static INT32U Prcess_Callback(INT16U w, INT16U h, INT32U addr)
{
    INT32U disp_buf,event;
    INT32S ret;

    if((disp_h_size != DISP_SRC_WIDTH) && (disp_v_size != DISP_SRC_HEIGHT))
    {
        disp_buf = disp_frame_buffer_get(1);
        if(disp_buf)
            fd_display_set_frame(addr,disp_buf,w,h,disp_h_size,disp_v_size);
    }
    else
        disp_buf = addr;

    event = prcess_state_get();
#if DISP_BUFFER_POST_EN == 1
    if(event == PRCESS_STATE_OK)
    {
        ret = prcess_free_post_get(1);
        if(ret)
        {
            prcess_frame_buffer_add((INT32U *)addr,1);
            prcess_free_post = ret;
        }
        else
            prcess_free_post = addr;
    }
    else
        prcess_free_post = addr;
#else
    if(event == PRCESS_STATE_OK || retry)
    {
        ret = prcess_frame_buffer_add((INT32U *)prcess_ppu_go(w,h,addr),1);
        if(ret < 0)
        {
            event = ppu_frame_buffer_display_get();
            if(event)
            {
                prcess_frame_buffer_add((INT32U *)event,1);
                retry = 0;
            }
            else
                retry = 1;
        }
        else
           retry = 0;
    }
#endif
    drv_l2_display_buffer_update_state_get(DISPLAY_DEVICE,1);
    drv_l2_display_buffer_set(DISPLAY_DEVICE,disp_buf);
    if((disp_h_size != DISP_SRC_WIDTH) && (disp_v_size != DISP_SRC_HEIGHT))
        disp_frame_buffer_add((INT32U *)disp_buf,1);
}

#if DISP_BUFFER_POST_EN == 0
static void prcess_ppu_init(void)
{
    INT32U i,frame_size,buffer_ptr,PPU_FRAME_BUFFER_BASE;
    FB_LOCK_STRUCT fb_lock_set;

    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;

    // Initialize display device
    drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);

    //Initiate PPU hardware engine and PPU register set structure
    gplib_ppu_init(ppu_register_set);

    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&disp_h_size, (INT16U *)&disp_v_size);
    fb_lock_set.color1 = PPU_FMT_YUYV;
    fb_lock_set.h_size1 = PRCESS_SRC_WIDTH;
    fb_lock_set.v_size1 = PRCESS_SRC_HEIGHT;
    fb_lock_set.color2 = PPU_FMT_YUYV;
    fb_lock_set.h_size2 = disp_h_size;
    fb_lock_set.v_size2 = disp_v_size;
    gplib_ppu_fb_lock_process_enable_set(ppu_register_set,(FB_LOCK_STRUCT *)&fb_lock_set);
    //Now configure PPU software structure
    gplib_ppu_enable_set(ppu_register_set, 1);					            // Enable PPU

    //TV frame mode
    gplib_ppu_non_interlace_set(ppu_register_set, 0);			            // Set non-interlace mode
    gplib_ppu_frame_buffer_mode_set(ppu_register_set, 1, 0);		        // Enable TV/TFT frame buffer mode

    //PPU setting
    gplib_ppu_fb_format_set(ppu_register_set, 1, 1);			            // Set PPU output frame buffer format to YUYV
    gplib_ppu_vga_mode_set(ppu_register_set, 0);							// Disable VGA mode
    gplib_ppu_resolution_set(ppu_register_set, C_TFT_RESOLUTION_320X240);	// Set display resolution to 640x480
    gplib_ppu_free_size_set(ppu_register_set, 0, PRCESS_SRC_WIDTH, PRCESS_SRC_HEIGHT);

    gplib_ppu_bottom_up_mode_set(ppu_register_set, 1);                      // bottom to top
    gplib_ppu_long_burst_set(ppu_register_set, 1);

    //Frame buffer malloc
    frame_size = (PRCESS_SRC_WIDTH * PRCESS_SRC_HEIGHT * 2);
    PPU_FRAME_BUFFER_BASE = (INT32U) gp_malloc_align(((frame_size*C_PPU_DRV_FRAME_NUM)+128), 64);
    if(PPU_FRAME_BUFFER_BASE == 0)
    {
        DBG_PRINT("PPU_FRAME_BUFFER_BASE fail\r\n");
        while(1);
    }
    PPU_FRAME_BUFFER_BASE = (INT32U)((PPU_FRAME_BUFFER_BASE + FRAME_BUF_ALIGN64) & ~FRAME_BUF_ALIGN64);
    prcess_mem_set->ppu_frame_workmem = PPU_FRAME_BUFFER_BASE;
    for (i=0; i<C_PPU_DRV_FRAME_NUM; i++) {
            buffer_ptr = (INT32U)(PPU_FRAME_BUFFER_BASE + (i*frame_size));
            gplib_ppu_frame_buffer_add(ppu_register_set, buffer_ptr);
            DBG_PRINT("PPUBuffer:0x%X \r\n",buffer_ptr);
    }
    drv_l2_display_buffer_set(DISPLAY_DEVICE, buffer_ptr);

    // Now configure TEXT relative elements
    gplib_ppu_text_compress_disable_set(ppu_register_set, 1);	                    // Disable TEXT1/TEXT2 horizontal/vertical compress function
    gplib_ppu_text_direct_mode_set(ppu_register_set, 0);			                // Disable TEXT direct address mode

    //text 2 2D
    gplib_ppu_text_init(ppu_register_set, C_PPU_TEXT1);
    buffer_ptr = (INT32U)gp_malloc_align(4096+64,4);
    if(buffer_ptr == 0)
    {
        DBG_PRINT("gplib_ppu_text_number_array_ptr_set fail\r\n");
        while(1);
    }
    buffer_ptr = (INT32U)((buffer_ptr + FRAME_BUF_ALIGN32) & ~FRAME_BUF_ALIGN32);
    prcess_mem_set->ppu_narray_workmem = buffer_ptr;
    gplib_ppu_text_number_array_ptr_set(ppu_register_set, C_PPU_TEXT1, (INT32U)buffer_ptr);	 // Set TEXT number array address
    gplib_ppu_text_enable_set(ppu_register_set, C_PPU_TEXT1, 1);	                        // Enable TEXT
    gplib_ppu_yuv_type_set(ppu_register_set, 3);								     // Set 32-bit color format to Y1UY0V
    gplib_ppu_text_bitmap_mode_set(ppu_register_set, C_PPU_TEXT1, 1);			     // Enable bitmap mode
    gplib_ppu_text_attribute_source_select(ppu_register_set, C_PPU_TEXT1, 1);	    // Get TEXT attribute from register
    gplib_ppu_text_color_set(ppu_register_set, C_PPU_TEXT1, 1, 3);				     // Set TEXT color to YUYV
    gplib_ppu_text_size_set(ppu_register_set, C_PPU_TEXT1, 4);			             // Set TEXT size to 1024x512
    gplib_ppu_text_segment_set(ppu_register_set, C_PPU_TEXT1, 0);				    // Set TEXT segment address
}
#endif

static INT32S demo_prcess_init(void)
{
    osThreadDef_t prcess_task = {"prcess_task", prcess_task_entry, osPriorityNormal, 1, 65536};
#if DISP_BUFFER_POST_EN == 1
    FB_LOCK_STRUCT fb_lock_set;
#endif

    /* initial prcess parameter set structure */
    prcess_mem_set = (prcess_mem_t *)&prcess_mem_structure;
    gp_memset((INT8S *)prcess_mem_set, 0, sizeof(prcess_mem_t));
#if DISP_BUFFER_POST_EN == 1
    /* initial ppu register parameter set structure */
    ppu_register_set = (PPU_REGISTER_SETS *)&ppu_register_structure;
    gp_memset((INT8S *)ppu_register_set, 0, sizeof(ppu_register_set));
    // Initialize display device
    drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);
    drv_l2_display_get_size(DISPLAY_DEVICE, (INT16U *)&disp_h_size, (INT16U *)&disp_v_size);
    fb_lock_set.color1 = PPU_FMT_YUYV;
    fb_lock_set.h_size1 = PRCESS_SRC_WIDTH;
    fb_lock_set.v_size1 = PRCESS_SRC_HEIGHT;
    fb_lock_set.color2 = PPU_FMT_YUYV;
    fb_lock_set.h_size2 = disp_h_size;
    fb_lock_set.v_size2 = disp_v_size;
    gplib_ppu_fb_lock_process_enable_set(ppu_register_set,(FB_LOCK_STRUCT *)&fb_lock_set);
#else
    prcess_ppu_init();
#endif
	osDelay(5);

    prcess_id = osThreadCreate(&prcess_task, (void *)NULL);
    if(prcess_id == 0)
    {
        DBG_PRINT("osThreadCreate: prcess_id error\r\n");
        while(1);
    }
    else
    {
        osDelay(50);
        //DBG_PRINT("osThreadCreate: prcess_id = 0x%x \r\n", prcess_id);
    }

    return 0;
}

static INT32S demo_prcess_exit(void)
{
    INT32S nRet = STATUS_OK;

    // csi task
    POST_MESSAGE(prcess_frame_buffer_queue, MSG_PRCESS_TASK_EXIT, prcess_task_ack_m, 5000);
Return:
    OSQFlush(prcess_frame_buffer_queue);
    OSQFlush(disp_frame_buffer_queue);
    OSQFlush(prcess_state_queue);
#if DISP_BUFFER_POST_EN == 1
    OSQFlush(prcess_free_post_queue);
#endif

    if(prcess_mem_set)
    {
        if(prcess_mem_set->disp_prcess_workmem)
        {
            gp_free((void *)prcess_mem_set->disp_prcess_workmem);
            prcess_mem_set->disp_prcess_workmem = 0;
        }

        if(prcess_mem_set->ppu_frame_workmem)
        {
            gp_free((void *)prcess_mem_set->ppu_frame_workmem);
            prcess_mem_set->ppu_frame_workmem = 0;
        }

        if(prcess_mem_set->ppu_narray_workmem)
        {
            gp_free((void *)prcess_mem_set->ppu_narray_workmem);
            prcess_mem_set->ppu_narray_workmem = 0;
        }

        if(prcess_mem_set->prcess_post_workmem)
        {
            gp_free((void *)prcess_mem_set->prcess_post_workmem);
            prcess_mem_set->prcess_post_workmem = 0;
        }

        prcess_mem_set = 0;
    }

    return nRet;
}

void Digital_Video_With_Prcess_Demo(void)
{
	char  path[64];
	INT8U OperationMode;
	INT8S volume, play_index, pause ;
	INT32U  zoom_ratio;
	INT32S nRet;
	INT32S index = 0;
	INT32U folder_nums, file_nums;
	VIDEO_CODEC_STATUS status;
	VIDEO_INFO	information;
	VIDEO_ARGUMENT arg;
	MEDIA_SOURCE   src;
    osMessageQDef_t disp_q = {C_PPU_DRV_FRAME_NUM, sizeof(INT32U), 0};

	while(1)
	{
		if( _devicemount(USE_DISK))
		{
			DBG_PRINT("Mount Disk Fail[%d]\r\n", USE_DISK);
	#if	USE_DISK == FS_NAND1
			nRet = DrvNand_lowlevelformat();
			DBG_PRINT("NAND LOW LEVEL FORMAT = %d \r\n", nRet);
			nRet = _format(FS_NAND1, FAT32_Type);
			DBG_PRINT("Format NAND = %d\r\n", nRet);
			DrvNand_flush_allblk();
	#endif
			_deviceunmount(USE_DISK);
		}
		else
		{
			DBG_PRINT("Mount Disk success[%d]\r\n", USE_DISK);
			break;
		}
	}

    if(disp_frame_buffer_queue == NULL)
	{
        disp_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!disp_frame_buffer_queue)
		{
            DBG_PRINT("disp_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("disp_frame_buffer_queue = 0x%x\r\n", disp_frame_buffer_queue);
	}

    if(prcess_frame_buffer_queue == NULL)
	{
        prcess_frame_buffer_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_frame_buffer_queue)
		{
            DBG_PRINT("prcess_frame_buffer_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_frame_buffer_queue = 0x%x\r\n", prcess_frame_buffer_queue);
	}

    if(prcess_task_ack_m == NULL)
	{
        prcess_task_ack_m = osMessageCreate(&disp_q, NULL);
		if(!prcess_task_ack_m)
		{
            DBG_PRINT("prcess_task_ack_m error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_task_ack_m = 0x%x\r\n", prcess_task_ack_m);
	}

#if DISP_BUFFER_POST_EN == 1
    if(prcess_free_post_queue == NULL)
	{
        prcess_free_post_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_free_post_queue)
		{
            DBG_PRINT("prcess_free_post_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_free_post_queue = 0x%x\r\n", prcess_free_post_queue);
	}
#endif

    if(prcess_state_queue == NULL)
	{
        prcess_state_queue = osMessageCreate(&disp_q, NULL);
		if(!prcess_state_queue)
		{
            DBG_PRINT("prcess_state_queue error\r\n");
            while(1);
		}
		else
            DBG_PRINT("prcess_state_queue = 0x%x\r\n", prcess_state_queue);
	}

	demo_prcess_init();

	// Initialize display device
    video_encode_register_display_callback(Prcess_Callback);
#if DISP_BUFFER_POST_EN == 1
    video_encode_buffer_post_callback(Frame_Post_Callback);
#endif

	//init as dv play mode
	volume = 0x3F;
	play_index = 0;
	zoom_ratio = 10;
	nRet = 0;
	pause = 0;
	OperationMode = DV_RECORD;

    memset((void*)&arg, 0, sizeof(VIDEO_ARGUMENT));
	video_encode_entrance();
	arg.bScaler = 1;
	arg.bUseEncodeDiffSize = 1;
	arg.JPEGTargetWidth = JPEG_SRC_WIDTH;
	arg.JPEGTargetHeight = JPEG_SRC_HEIGHT;
	arg.TargetWidth = PRCESS_SRC_WIDTH;
	arg.TargetHeight = PRCESS_SRC_HEIGHT;
	arg.SensorWidth	= SENSOR_SRC_WIDTH;
	arg.SensorHeight = SENSOR_SRC_HEIGHT;
	arg.DisplayWidth = DISP_SRC_WIDTH;
	arg.DisplayHeight = DISP_SRC_HEIGHT;
	arg.VidFrameRate = VIDEO_FPS;
	arg.AudSampleRate = AUDIO_SPR;
	arg.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV; //for display
	video_encode_preview_start(arg);

    DBG_PRINT("===============================\r\n");
	DBG_PRINT("S4 -> Demo Exit\r\n");
	DBG_PRINT("S1 -> start RECORD\r\n");
	DBG_PRINT("S2 -> stop RECORD\r\n");
	DBG_PRINT("S3 -> capture image\r\n");
	DBG_PRINT("===============================\r\n");

    adc_key_scan_init(); //init key scan

	while(1)
	{
		adc_key_scan();
        if(ADKEY_IO1)
		{//start play
			switch(OperationMode)
			{
                case DV_RECORD:
                    if(video_encode_status() == VIDEO_CODEC_PROCESS_END)
                    {
                        DBG_PRINT("video_encode_start\r\n");

                        video_encode_capture_disable_difference_size();

                        if(USE_DISK == FS_SD)
                            sprintf((char *)path, (const char *)"C:\\avi_rec%d.avi", nRet++);
                        else if(USE_DISK == FS_NAND1)
                            sprintf((char *)path, (const char *)"A:\\avi_rec%d.avi", nRet++);
                        else if(USE_DISK == FS_SD2)
                            sprintf((char *)path, (const char *)"K:\\avi_rec%d.avi", nRet++);

                        src.type = SOURCE_TYPE_FS;
                        src.type_ID.FileHandle = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
                        if(src.type_ID.FileHandle < 0)
                        {
                            DBG_PRINT("file open fail\r\n");
                            continue;
                        }
                        else
                            DBG_PRINT("file name = %s\r\n", path);

                        //src.Format.VideoFormat = H_264;
                        src.Format.VideoFormat = MJPEG;
                        video_encode_set_jpeg_quality(70);

                        if (STATUS_OK != video_encode_start(src, 1))
                            DBG_PRINT("video encode start fail\r\n");
                    }
                    break;
			}
		}
		else if(ADKEY_IO2)
		{//stop
			switch(OperationMode)
			{
                case DV_RECORD:
                    if(video_encode_status() == VIDEO_CODEC_PROCESSING)
                    {
                        DBG_PRINT("video_encode_stop\r\n");
                        video_encode_stop();
                        fs_close(src.type_ID.FileHandle);
                        src.type_ID.FileHandle = -1;
                #if USE_DISK == FS_NAND1
                        DBG_PRINT("DrvNand_flush_allblk\r\n");
                        DrvNand_flush_allblk();
                #endif
                    }
                    break;
			}
		}
		else if(ADKEY_IO3)
		{
            switch(OperationMode)
            {
                case DV_RECORD:
                    if(video_encode_status() == VIDEO_CODEC_PROCESSING){
                        DBG_PRINT("\r\nCannot Capture Image When Recording.\r\n");
                        break;
                    }
                    src.type = SOURCE_TYPE_FS;
                    src.Format.VideoFormat = MJPEG;
                    video_encode_set_jpeg_quality(95);

                    sprintf((char *)path, (const char *)"K:\\pic%03d.jpg", index++);
                    DBG_PRINT("file name = %s\r\n", path);

                    src.type_ID.FileHandle = fs_open(path, O_WRONLY|O_CREAT|O_TRUNC);
                    if(src.type_ID.FileHandle < 0) {
                        DBG_PRINT("file open fail\r\n");
                        continue;
                    }
                    DBG_PRINT("video_encode_capture_picture\r\n");
                    status = video_encode_capture_picture(src);
                    if(status != START_OK) {
                        unlink((CHAR *)path);
                        index--;
                        DBG_PRINT("video_encode_capture_picture fail!!!\r\n");
                        continue;
                    }
                    DBG_PRINT("Capture finished\r\n\r\n");
                    break;
            }
		}
		else if(ADKEY_IO4)
		{
            switch(OperationMode)
			{//exit the old mode
                case DV_RECORD:
                    zoom_ratio = 10;
                    video_encode_preview_stop();
                    osDelay(10);// wait csi stop and scaler task stop
                    video_encode_exit();
                    demo_prcess_exit();
                    break;
			}
			break;
		}
	}
    drv_l2_display_stop(DISPLAY_DEVICE);
    adc_key_scan_uninit();
	_deviceunmount(USE_DISK);
	DBG_PRINT("\r\nDEMO EXIT\r\n");
}
