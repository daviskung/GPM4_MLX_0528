#include <stdio.h>
#include <string.h>
#include "application.h"
#include "gplib.h"
//#include "drv_l2_key_scan.h"
#include "drv_l2_ad_key_scan.h"
#include "drv_l2_display.h"

#define USE_DISK	FS_SD2
//#define USE_DISK    FS_NAND1

#define DV_PLAY			0
#define DV_RECORD		1

#define TAR_WIDTH		1280
#define TAR_HEIGHT		720
#define SNR_WIDTH		1280//1920
#define SNR_HEIGHT		720//1088
#define DIP_WIDTH		640/2
#define DIP_HEIGHT		480/2
#define FPS				30
#define SPR				8000

#define C_FILE_NODE_SIZE			500

//call drv_l2_set_raw_capure_setting() set how many step need calibration,
//para 1 : current step, para 2: "0: not include LINCORR & LENCMP, 1: with LINCORR, 2: with LENCMP"
//change CAL_MODE for capture each step
#define CAL_MODE      0
#define CAL_LINCORR 0x1
#define CAL_LENCMP  0x2

//INT16U	file_node_buf[C_FILE_NODE_SIZE];
//struct sfn_info file_fs_info;
//struct STFileNodeInfo musicFNodeInfo;

extern INT32U g_flag_update;
extern INT32U g_update_value;
extern INT32U g_frame_addr1;

//extern void TFT_TD025THED1_Init(void);
//extern INT8U  Display_Device;

//=================================================================================================
//	DV demo code
//=================================================================================================
static INT32U Display_Callback(INT16U w, INT16U h, INT32U addr)
{
    R_TFT_FBI_ADDR = addr;
}

int ev_step_index = 153;//161; //161;//169;
int file_number   = 0;
void Sensor_cal(void)
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
    INT32S fd;
    INT8U file_name[24];
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
    if (USE_DISK == FS_SD1){
		DBG_PRINT("Media : SDCard \r\n");
		chdir("F:\\");
	}
	else if (USE_DISK == FS_SD){
		DBG_PRINT("Media : SDCard \r\n");
		chdir("C:\\");
	}
	else if (USE_DISK == FS_SD2){
		DBG_PRINT("Media : SDCard \r\n");
		chdir("K:\\");
	}
	else if (USE_DISK == FS_NAND1){
		DBG_PRINT("Media : NAND \r\n");
		chdir("A:\\");
	}
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

    drv_l2_set_raw_capure_setting(CAL_MODE,0);

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
	R_FUNPOS1 &= ~((0x3<<8)|(0x3<<3));

	video_encode_preview_start(arg);
    g_frame_addr1 = (INT32U)gp_malloc_align(SNR_WIDTH*SNR_HEIGHT*2,32);
	DBG_PRINT("===============================\r\n");
	DBG_PRINT("KEY1 capture, set CAL_MODE\r\n");
	DBG_PRINT("0: capture raw mode 0(RAW)\r\n");
	DBG_PRINT("1: capture raw mode 1(+LINCORR)\r\n");
	DBG_PRINT("2: capture raw mode 2(+LENCMP)\r\n");
	DBG_PRINT("3: capture raw mode 3(+WbGainEn)\r\n");
	DBG_PRINT("4: capture raw mode 4(+LutGammaE)\r\n");
	DBG_PRINT("NOTE: mode 1 & 2 only enabled when \r\n");
	DBG_PRINT("drv_l2_set_raw_capure_setting() set correponding bit\r\n");
	DBG_PRINT("and need add/remove below function after drv_l2_cdsp_enable() in sensor driver\r\n");
    DBG_PRINT("drv_l2_cdsp_set_linear_correction()\r\n");
    DBG_PRINT("drv_l2_cdsp_set_lens_compensation()\r\n");
	DBG_PRINT("===============================\r\n");

	while(1)
	{
		adc_key_scan();
		if(ADKEY_IO7)
		{
            do
            {
                sprintf((char *)file_name, (const char *)"raw_mode%d_exp%d_%d.raw", CAL_MODE, ev_step_index, file_number);

                fd = fs_open(file_name, O_CREAT|O_EXCL);
                if(fd < 0){
                    DBG_PRINT("File %s existing\r\n", file_name);
                    file_number++;
                }
                else{
                    fs_close(fd);
                    fd = fs_open(file_name, O_TRUNC|O_WRONLY);
                    DBG_PRINT("Open File %s OK %d\r\n", file_name, fd);
                }
            }while(fd < 0);

            g_update_value = CAL_MODE;

			g_flag_update = 1;
			while(1){
                osDelay(2);
                if(g_flag_update == 0){
                    break;
                }
			}
            osDelay(30);
            nRet = fs_write(fd, (INT32U)g_frame_addr1, SNR_WIDTH * SNR_HEIGHT * 2);
            DBG_PRINT("Write %d, 0x%x, %d, %d\r\n", fd, g_frame_addr1, SNR_WIDTH * SNR_HEIGHT * 2, nRet);
            osDelay(30);
            fs_close(fd);
            DBG_PRINT("capture raw mode %d done\r\n",CAL_MODE);

		}
		else if(ADKEY_IO2)
		{
            ev_step_index++;
            DBG_PRINT("ev_step_index = %d\r\n", ev_step_index);
        }
		else if(ADKEY_IO3)
		{
            ev_step_index--;
            DBG_PRINT("ev_step_index = %d\r\n", ev_step_index);
		}
		else if(ADKEY_IO4)
		{
		}
		else if(ADKEY_IO5)
		{
		}
		else if(ADKEY_IO6)
		{
		}
		else if(ADKEY_IO1)
		{
		}
		else if(ADKEY_IO8)
		{
		}
	}
    drv_l2_display_stop(DISDEV_TFT);
    adc_key_scan_uninit();
	_deviceunmount(USE_DISK);
	DBG_PRINT("\r\nDEMO EXIT\r\n");
}

