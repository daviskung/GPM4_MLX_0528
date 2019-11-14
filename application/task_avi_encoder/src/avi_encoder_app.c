#include "drv_l1_cache.h"
#include "drv_l1_i2s_rx.h"
#include "drv_l1_adc.h"
#include "drv_l1_scaler.h"
#include "drv_l2_scaler.h"
#include "drv_l1_jpeg.h"
#include "drv_l1_csi.h"
//#include "drv_l1_cdsp.h"
//#include "drv_l2_cdsp.h"
#include "drv_l2_sensor.h"
#include "gplib_jpeg.h"
#include "gplib_jpeg_encode.h"
#include "avi_encoder_app.h"
#include "jpeg_header.h"
#include "font.h"
#include "gplib.h"
#include "drv_l1_h264.h"
#include "drv_l2_h264enc.h"
//#include "defs_th32x32.h"
//#include "defs_MLX.h"

#if (defined APP_VIDEO_ENCODER_EN) && (APP_VIDEO_ENCODER_EN == 1)

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
AviEncPara_t AviEncPara, *pAviEncPara;
AviEncAudPara_t AviEncAudPara, *pAviEncAudPara;
AviEncVidPara_t AviEncVidPara, *pAviEncVidPara;
AviEncPacker_t AviEncPacker0, *pAviEncPacker0;
AviEncPacker_t AviEncPacker1, *pAviEncPacker1;
catpure_t *pCap;
void* h264_handle;

GP_AVI_AVISTREAMHEADER	avi_aud_stream_header;
GP_AVI_AVISTREAMHEADER	avi_vid_stream_header;
GP_AVI_BITMAPINFO		avi_bitmap_info;
GP_AVI_PCMWAVEFORMAT	avi_wave_info;

//MLX_TH32x24Para_t MLX_TH32x24_Para, *pMLX_TH32x24_Para;	// 2019.03.28 davis
//paramsMLX90640_t	MLX32x24_Para, *pMLX32x24_Para;


static INT8U g_csi_index;
static INT8U g_pcm_index;
static INT32U buffer_init = 0, buffer_base_addr_5M = 0, encode_buffer_size;

extern volatile INT32S pscaler_start_1;
// function
void avi_encode_init(void)
{
    pAviEncPara = &AviEncPara;
    memset((INT8S *)pAviEncPara, 0, sizeof(AviEncPara_t));

    pAviEncAudPara = &AviEncAudPara;
    memset((INT8S *)pAviEncAudPara, 0, sizeof(AviEncAudPara_t));
	pAviEncVidPara = &AviEncVidPara;
    memset((INT8S *)pAviEncVidPara, 0, sizeof(AviEncVidPara_t));

    pAviEncPacker0 = &AviEncPacker0;
    memset((INT8S *)pAviEncPacker0, 0, sizeof(AviEncPacker_t));
    pAviEncPacker1 = &AviEncPacker1;
    memset((INT8S *)pAviEncPacker1, 0, sizeof(AviEncPacker_t));

	//pMLX_TH32x24_Para = &MLX_TH32x24_Para;	// 2019.03.28 davis
    //gp_memset((INT8S *)pMLX_TH32x24_Para, 0, sizeof(MLX_TH32x24Para_t));

	//pMLX32x24_Para = &MLX32x24_Para ;	// 2019.05.28 davis
    //gp_memset((INT8S *)pMLX32x24_Para, 0, sizeof(paramsMLX90640_t));

	pAviEncPacker0->file_handle = -1;
	pAviEncPacker0->index_handle = -1;
	pAviEncPacker1->file_handle = -1;
	pAviEncPacker1->index_handle = -1;
	pAviEncVidPara->scaler_zoom_ratio = 10;
	pAviEncVidPara->h264_bitrate = 7200;
	pAviEncVidPara->h264_gop_len = 15;
}

static void avi_encode_sync_timer_isr(void)
{
	if(pAviEncPara->AviPackerCur->p_avi_wave_info) {
		if((pAviEncPara->tv - pAviEncPara->ta) < pAviEncPara->delta_ta) {
			pAviEncPara->tv += pAviEncPara->tick;
		}
	} else {
		pAviEncPara->tv += pAviEncPara->tick;
	}

	if((pAviEncPara->tv - pAviEncPara->Tv) > 0) {
		if(pAviEncPara->post_cnt == pAviEncPara->pend_cnt) {
			if(avi_enc_one_frame() >= 0) {
				pAviEncPara->post_cnt++;
			}
		}
	}
}

void avi_encode_video_timer_start(void)
{
	INT32U freq_hz;
#if AVI_AUDIO_ENCODE_EN == 1
    INT32U preload;
#endif

	//pAviEncPara->ta = 0;
	//pAviEncPara->tv = 0;
	//pAviEncPara->Tv = 0;
	pAviEncPara->pend_cnt = 0;
	pAviEncPara->post_cnt = 0;
#if AVI_AUDIO_ENCODE_EN == 1
#if MIC_INPUT_SRC == C_ADC_LINE_IN
	if(AVI_AUDIO_RECORD_TIMER == ADC_AS_TIMER_C) {
		preload = R_TIMERC_PRELOAD;
	} else if(AVI_AUDIO_RECORD_TIMER == ADC_AS_TIMER_D) {
		preload = R_TIMERD_PRELOAD;
	} else if(AVI_AUDIO_RECORD_TIMER == ADC_AS_TIMER_E) {
		preload = R_TIMERE_PRELOAD;
	} else {//timerf
		preload = R_TIMERF_PRELOAD;
	}

	if(pAviEncPara->AviPackerCur->p_avi_wave_info) {
		INT32U temp;

		//temp = 0x10000 -((0x10000 - (R_TIMERE_PRELOAD & 0xFFFF)) * p_vid_dec_para->n);
		temp = (0x10000 - (preload & 0xFFFF)) * pAviEncPara->freq_div;
		freq_hz = MCLK/2/temp;
		if(MCLK %(2*temp)) {
			freq_hz++;
		}
	} else {
		freq_hz = AVI_ENCODE_TIME_BASE;
	}

#elif MIC_INPUT_SRC == C_BUILDIN_MIC_IN || MIC_INPUT_SRC == C_LINE_IN_LR
	freq_hz = preload = AVI_ENCODE_TIME_BASE;
#endif
#else
	freq_hz = AVI_ENCODE_TIME_BASE;
#endif
	timer_freq_setup(AVI_ENCODE_VIDEO_TIMER, freq_hz, 0, avi_encode_sync_timer_isr);
}

void avi_encode_video_timer_stop(void)
{
	timer_stop(AVI_ENCODE_VIDEO_TIMER);
}

// file handle
INT32S avi_encode_set_file_handle_and_caculate_free_size(AviEncPacker_t *pAviEncPacker, INT16S FileHandle)
{
#if (AVI_PACKER_LIB_EN || NEW_VIDEO_PACKER_LIB)
	INT16S disk_no;
	INT64U disk_free;
#if AVI_ENCODE_FAST_SWITCH_EN == 1
	struct stat_t statbuf;
#endif

	//SOURCE_TYPE_USER_DEFINE
	if(FileHandle > 100) {
		pAviEncPara->disk_free_size = (INT64U)FileHandle << 48;
   		pAviEncPara->record_total_size = 2*32*1024 + 16;
		return STATUS_OK;
	}

	//create index file handle
    disk_no = GetDiskOfFile(FileHandle);
    if(disk_no == 0) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"A:\\");
    } else if(disk_no == 1) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"B:\\");
    } else if(disk_no == 2) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"C:\\");
    } else if(disk_no == 3) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"D:\\");
    } else if(disk_no == 4) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"E:\\");
    } else if(disk_no == 5) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"F:\\");
    } else if(disk_no == 6) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"G:\\");
    } else if(disk_no == 7) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"H:\\");
    } else if(disk_no == 8) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"I:\\");
    } else if(disk_no == 9) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"J:\\");
    } else if(disk_no == 10) {
    	strcpy((INT8S*)pAviEncPacker->index_path, (INT8S*)"K:\\");
    } else {
    	return STATUS_FAIL;
    }

    chdir((CHAR*)pAviEncPacker->index_path);
#if AVI_ENCODE_FAST_SWITCH_EN == 1
    if(stat("index0.tmp", &statbuf) < 0) {
    	strcat((INT8S*)pAviEncPacker->index_path, (INT8S*)"index0.tmp");
    } else {
    	strcat((INT8S*)pAviEncPacker->index_path, (INT8S*)"index1.tmp");
	}
#else
   	strcat((INT8S*)pAviEncPacker->index_path, (INT8S*)"index0.tmp");
#endif

    pAviEncPacker->file_handle = FileHandle;
#if AVI_PACKER_LIB_EN
    pAviEncPacker->index_handle = fs_open((char*)pAviEncPacker->index_path, O_RDWR|O_CREAT|O_TRUNC);
    if(pAviEncPacker->index_handle < 0) {
    	return STATUS_FAIL;
   	}
#endif

   	//caculate storage free size
    disk_free = vfsFreeSpace(disk_no);
    DBG_PRINT("DISK FREE SIZE = %lld MByte\r\n", disk_free/1024/1024);
    if(disk_free < C_MIN_DISK_FREE_SIZE) {
    	avi_encode_close_file(pAviEncPacker);
  		return STATUS_FAIL;
  	}

    pAviEncPara->disk_free_size = disk_free - C_MIN_DISK_FREE_SIZE;
   	pAviEncPara->record_total_size = 2*32*1024 + 16; //avi header + data is 32k align + index header
#endif
	return STATUS_OK;
}

INT16S avi_encode_close_file(AviEncPacker_t *pAviEncPacker)
{
	INT32S nRet = 0;
	INT16S disk_no;

#if (AVI_PACKER_LIB_EN || NEW_VIDEO_PACKER_LIB)
#if (NAND1_EN || NAND2_EN)
	disk_no = GetDiskOfFile(pAviEncPacker->file_handle);
#endif
	// final version
	nRet = fs_close(pAviEncPacker->file_handle);
#if AVI_PACKER_LIB_EN
	nRet = fs_close(pAviEncPacker->index_handle);
	nRet = unlink((CHAR*)pAviEncPacker->index_path);
#endif
#if (NAND1_EN || NAND2_EN)
	if(disk_no == FS_NAND1 || disk_no == FS_NAND2) {
		DrvNand_flush_allblk();
		DBG_PRINT("Nand_flush_allblk.\r\n");
	}
#endif
#endif
	pAviEncPacker->file_handle = -1;
	pAviEncPacker->index_handle = -1;
	return nRet;
}

void avi_encode_fail_handle_close_file(AviEncPacker_t *pAviEncPacker)
{
#if (AVI_PACKER_LIB_EN || NEW_VIDEO_PACKER_LIB)
    fs_close(pAviEncPacker->file_handle);
#if AVI_PACKER_LIB_EN
	fs_close(pAviEncPacker->index_handle);
    unlink((CHAR*)pAviEncPacker->index_path);
#endif
#endif
	pAviEncPacker->file_handle = -1;
	pAviEncPacker->index_handle = -1;
}

INT32S avi_encode_set_avi_header(AviEncPacker_t *pAviEncPacker)
{
	INT16U sample_per_block, package_size;

	pAviEncPacker->p_avi_aud_stream_header = &avi_aud_stream_header;
	pAviEncPacker->p_avi_vid_stream_header = &avi_vid_stream_header;
	pAviEncPacker->p_avi_bitmap_info = &avi_bitmap_info;
	pAviEncPacker->p_avi_wave_info = &avi_wave_info;
	memset((INT8S*)pAviEncPacker->p_avi_aud_stream_header, 0, sizeof(GP_AVI_AVISTREAMHEADER));
	memset((INT8S*)pAviEncPacker->p_avi_vid_stream_header, 0, sizeof(GP_AVI_AVISTREAMHEADER));
	memset((INT8S*)pAviEncPacker->p_avi_bitmap_info, 0, sizeof(GP_AVI_BITMAPINFO));
	memset((INT8S*)pAviEncPacker->p_avi_wave_info, 0, sizeof(GP_AVI_PCMWAVEFORMAT));

	//audio
	avi_aud_stream_header.fccType[0] = 'a';
	avi_aud_stream_header.fccType[1] = 'u';
	avi_aud_stream_header.fccType[2] = 'd';
	avi_aud_stream_header.fccType[3] = 's';

	switch(pAviEncAudPara->audio_format)
	{
	case WAV:
		pAviEncPacker->wave_info_cblen = 16;
		avi_aud_stream_header.fccHandler[0] = 1;
		avi_aud_stream_header.fccHandler[1] = 0;
		avi_aud_stream_header.fccHandler[2] = 0;
		avi_aud_stream_header.fccHandler[3] = 0;

		avi_wave_info.wFormatTag = WAVE_FORMAT_PCM;
		avi_wave_info.nChannels = pAviEncAudPara->channel_no;
		avi_wave_info.nSamplesPerSec =  pAviEncAudPara->audio_sample_rate;
		avi_wave_info.nAvgBytesPerSec =  pAviEncAudPara->channel_no * pAviEncAudPara->audio_sample_rate * 2;
		avi_wave_info.nBlockAlign = pAviEncAudPara->channel_no * 2;
		avi_wave_info.wBitsPerSample = 16; //16bit

		avi_aud_stream_header.dwScale = avi_wave_info.nBlockAlign;
		avi_aud_stream_header.dwRate = avi_wave_info.nAvgBytesPerSec;
		avi_aud_stream_header.dwSampleSize = avi_wave_info.nBlockAlign;;
		break;

	case MICROSOFT_ADPCM:
		pAviEncPacker->wave_info_cblen = 50;
		avi_aud_stream_header.fccHandler[0] = 0;
		avi_aud_stream_header.fccHandler[1] = 0;
		avi_aud_stream_header.fccHandler[2] = 0;
		avi_aud_stream_header.fccHandler[3] = 0;

		package_size = 0x100;
		if(pAviEncAudPara->channel_no == 1) {
			sample_per_block = 2 * package_size - 12;
		} else if(pAviEncAudPara->channel_no == 2) {
			sample_per_block = package_size - 12;
		} else {
			sample_per_block = 1;
		}
		avi_wave_info.wFormatTag = WAVE_FORMAT_ADPCM;
		avi_wave_info.nChannels = pAviEncAudPara->channel_no;
		avi_wave_info.nSamplesPerSec =  pAviEncAudPara->audio_sample_rate;
		avi_wave_info.nAvgBytesPerSec =  package_size * pAviEncAudPara->audio_sample_rate / sample_per_block; // = PackageSize * FrameSize / fs
		avi_wave_info.nBlockAlign = package_size; //PackageSize
		avi_wave_info.wBitsPerSample = 4; //4bit
		avi_wave_info.cbSize = 32;
		// extension ...
		avi_wave_info.ExtInfo[0] = 0x01F4;	avi_wave_info.ExtInfo[1] = 0x0007;
		avi_wave_info.ExtInfo[2] = 0x0100;	avi_wave_info.ExtInfo[3] = 0x0000;
		avi_wave_info.ExtInfo[4] = 0x0200;	avi_wave_info.ExtInfo[5] = 0xFF00;
		avi_wave_info.ExtInfo[6] = 0x0000;	avi_wave_info.ExtInfo[7] = 0x0000;

		avi_wave_info.ExtInfo[8] =  0x00C0;	avi_wave_info.ExtInfo[9] =  0x0040;
		avi_wave_info.ExtInfo[10] = 0x00F0; avi_wave_info.ExtInfo[11] = 0x0000;
		avi_wave_info.ExtInfo[12] = 0x01CC; avi_wave_info.ExtInfo[13] = 0xFF30;
		avi_wave_info.ExtInfo[14] = 0x0188; avi_wave_info.ExtInfo[15] = 0xFF18;
		break;

	case IMA_ADPCM:
		pAviEncPacker->wave_info_cblen = 20;
		avi_aud_stream_header.fccHandler[0] = 0;
		avi_aud_stream_header.fccHandler[1] = 0;
		avi_aud_stream_header.fccHandler[2] = 0;
		avi_aud_stream_header.fccHandler[3] = 0;

		package_size = 0x100;
		if(pAviEncAudPara->channel_no == 1) {
			sample_per_block = 2 * package_size - 7;
		} else if(pAviEncAudPara->channel_no == 2) {
			sample_per_block = package_size - 7;
		} else {
			sample_per_block = 1;
		}
		avi_wave_info.wFormatTag = WAVE_FORMAT_IMA_ADPCM;
		avi_wave_info.nChannels = pAviEncAudPara->channel_no;
		avi_wave_info.nSamplesPerSec =  pAviEncAudPara->audio_sample_rate;
		avi_wave_info.nAvgBytesPerSec =  package_size * pAviEncAudPara->audio_sample_rate / sample_per_block;
		avi_wave_info.nBlockAlign = package_size;	//PackageSize
		avi_wave_info.wBitsPerSample = 4; //4bit
		avi_wave_info.cbSize = 2;
		// extension ...
		avi_wave_info.ExtInfo[0] = sample_per_block;
		break;

	default:
		pAviEncPacker->wave_info_cblen = 0;
		pAviEncPacker->p_avi_aud_stream_header = NULL;
		pAviEncPacker->p_avi_wave_info = NULL;
	}

	avi_aud_stream_header.dwScale = avi_wave_info.nBlockAlign;
	avi_aud_stream_header.dwRate = avi_wave_info.nAvgBytesPerSec;
	avi_aud_stream_header.dwSampleSize = avi_wave_info.nBlockAlign;

	//video
	avi_vid_stream_header.fccType[0] = 'v';
	avi_vid_stream_header.fccType[1] = 'i';
	avi_vid_stream_header.fccType[2] = 'd';
	avi_vid_stream_header.fccType[3] = 's';
	avi_vid_stream_header.dwScale = pAviEncVidPara->dwScale;
	avi_vid_stream_header.dwRate = pAviEncVidPara->dwRate;
	avi_vid_stream_header.rcFrame.right = pAviEncVidPara->encode_width;
	avi_vid_stream_header.rcFrame.bottom = pAviEncVidPara->encode_height;
	switch(pAviEncVidPara->video_format)
	{
	case C_MJPG_FORMAT:
		avi_vid_stream_header.fccHandler[0] = 'm';
		avi_vid_stream_header.fccHandler[1] = 'j';
		avi_vid_stream_header.fccHandler[2] = 'p';
		avi_vid_stream_header.fccHandler[3] = 'g';

		avi_bitmap_info.biSize = pAviEncPacker->bitmap_info_cblen = 40;
		avi_bitmap_info.biWidth = pAviEncVidPara->encode_width;
		avi_bitmap_info.biHeight = pAviEncVidPara->encode_height;
		avi_bitmap_info.biBitCount = 24;
		avi_bitmap_info.biCompression[0] = 'M';
		avi_bitmap_info.biCompression[1] = 'J';
		avi_bitmap_info.biCompression[2] = 'P';
		avi_bitmap_info.biCompression[3] = 'G';
		avi_bitmap_info.biSizeImage = pAviEncVidPara->encode_width * pAviEncVidPara->encode_height * 3;
		break;

	case C_XVID_FORMAT:
		avi_vid_stream_header.fccHandler[0] = 'x';
		avi_vid_stream_header.fccHandler[1] = 'v';
		avi_vid_stream_header.fccHandler[2] = 'i';
		avi_vid_stream_header.fccHandler[3] = 'd';

		avi_bitmap_info.biSize = pAviEncPacker->bitmap_info_cblen = 68;
		avi_bitmap_info.biWidth = pAviEncVidPara->encode_width;
		avi_bitmap_info.biHeight = pAviEncVidPara->encode_height;
		avi_bitmap_info.biBitCount = 24;
		avi_bitmap_info.biCompression[0] = 'X';
		avi_bitmap_info.biCompression[1] = 'V';
		avi_bitmap_info.biCompression[2] = 'I';
		avi_bitmap_info.biCompression[3] = 'D';
		avi_bitmap_info.biSizeImage = pAviEncVidPara->encode_width * pAviEncVidPara->encode_height * 3;
		break;
	}

	return STATUS_OK;
}

void avi_encode_set_curworkmem(void *pAviEncPacker)
{
	 pAviEncPara->AviPackerCur = pAviEncPacker;
}

// status
void avi_encode_set_status(INT32U bit)
{
	pAviEncPara->avi_encode_status |= bit;
}

void avi_encode_clear_status(INT32U bit)
{
	pAviEncPara->avi_encode_status &= ~bit;
}

INT32S avi_encode_get_status(void)
{
    return pAviEncPara->avi_encode_status;
}

//memory
INT32U avi_encode_post_empty(osMessageQId event, INT32U frame_addr)
{
	osStatus status;

	status = osMessagePut(event, (uint32_t)&frame_addr, osWaitForever);

	return status;
}

INT32U avi_encode_get_empty(osMessageQId event)
{
	INT32U frame_addr;
	osEvent result;

	result = osMessageGet(event, 1);

	frame_addr = result.value.v;

	if((result.status != osEventMessage) || (frame_addr == 0)) {
		return 0;
	}

	return frame_addr;
}

INT32U avi_encode_get_csi_frame(void)
{
	INT32U addr;

	addr =  pAviEncVidPara->csi_frame_addr[g_csi_index++];
	if(g_csi_index >= AVI_ENCODE_CSI_BUFFER_NO) {
		g_csi_index = 0;
	}
	return addr;
}

static INT32S sensor_mem_alloc(void)
{
	INT32U buffer_addr;
	INT32S i, buffer_size, nRet;

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
	buffer_size = pAviEncVidPara->sensor_capture_width * pAviEncVidPara->sensor_capture_height << 1;
#elif VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	buffer_size = pAviEncVidPara->sensor_capture_width * SENSOR_FIFO_LINE << 1;
#endif
    buffer_size = (buffer_size+0x3f) >> 6 << 6;
	buffer_addr = (INT32U) gp_malloc_align(buffer_size * AVI_ENCODE_CSI_BUFFER_NO + 0x3f, 32);
	if(buffer_addr == 0) {
		RETURN(STATUS_FAIL);
	}
    pAviEncVidPara->csi_frame_addr0 = buffer_addr;
    buffer_addr = (buffer_addr + 0x3f) >> 6 << 6;
	for(i=0; i<AVI_ENCODE_CSI_BUFFER_NO; i++) {
		pAviEncVidPara->csi_frame_addr[i] = buffer_addr + buffer_size * i;
		DBG_PRINT("sensor_frame_addr[%d] = 0x%x\r\n", i, pAviEncVidPara->csi_frame_addr[i]);
	}
	nRet = STATUS_OK;
Return:
	return nRet;
}

#if 0
static INT32S MLX_TH32x24_mem_alloc(void)	//davis
{
	INT32U buffer_addr;
	INT32S buffer_size, nRet;
	INT8U  tmpN1,tmpN2;

	pMLX_TH32x24_Para->MLX_TH32x24_width = MLX_LINE;
    pMLX_TH32x24_Para->MLX_TH32x24_height = MLX_COLUMN;

//	MLX32x24_EE_READ_8bitBUF
	buffer_size = MLX90640_EEMemAddrRead * 2;

	buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF = buffer_addr;
		DBG_PRINT("davis --> MLX32x24_EE_READ_8bitBUF = 0x%x\r\n", pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF);

//	MLX32x24_EE_READ_16bitBUF
		buffer_size = MLX90640_EEMemAddrRead*2;

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
			if(buffer_addr == 0) {
				RETURN(STATUS_FAIL);
			}
			pMLX_TH32x24_Para->MLX32x24_EE_READ_16bitBUF = buffer_addr;
			DBG_PRINT("davis --> MLX32x24_EE_READ_16bitBUF = 0x%x\r\n", pMLX_TH32x24_Para->MLX32x24_EE_READ_16bitBUF);


	// 	1 pixel takes 2 bytes => 32*24 pixel requires 32*24*2
	buffer_size = pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height << 1;

	buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
	//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
	if(buffer_addr == 0) {
		RETURN(STATUS_FAIL);
	}
	pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr = buffer_addr;
	DBG_PRINT("davis --> MLX_TH32x24_ColorOutputFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr);

	for(tmpN1=0; tmpN1<MLX_TH32x24_SCALERUP_BUFFER_NO; tmpN1++) {
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_TmpOutput_format_addr[tmpN1] = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_TmpOutput_format_addr[%d] = 0x%x\r\n",tmpN1, pMLX_TH32x24_Para->MLX_TH32x24_TmpOutput_format_addr[tmpN1]);
	}


	for(tmpN1=0;tmpN1<AVG_buf_len;tmpN1++){
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_avg_buf_addr[tmpN1] = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_avg_buf_addr[%d] addr = 0x%x\r\n",tmpN1, pMLX_TH32x24_Para->MLX_TH32x24_avg_buf_addr[tmpN1]);
	}

	//buffer_size = pAviEncVidPara->sensor_capture_width * pAviEncVidPara->sensor_capture_height << 1;
		buffer_size = pAviEncVidPara->display_width * pAviEncVidPara->display_height << 1;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_display_frame = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_display_frame addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_display_frame);


		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_display_background_frame = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_display_background_frame addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_display_background_frame);

	buffer_size = sizeof(INT16U)*MAXNROFDEFECTS ;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_BadPixAdr_buf = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_BadPixAdr_buf(INT16U) addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_BadPixAdr_buf);

	buffer_size = sizeof(INT8U)*MAXNROFDEFECTS ;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_BadPixMask_buf = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_BadPixMask_buf(INT8U) addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_BadPixMask_buf);


	buffer_size = IMAGE_DATA_INT32S_SIZE * MLX_Pixel ; // float/INT32S = 4 byte
	for(tmpN1 = 0 ; tmpN1 < IMG_AVG_buf_len ; tmpN1++){
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_ImgAvg_buf_addr[tmpN1] = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_ImgAvg_buf_addr[%d] addr = 0x%x\r\n",tmpN1, pMLX_TH32x24_Para->MLX_TH32x24_ImgAvg_buf_addr[tmpN1]);
	}

		//	1 pixel takes 2 bytes => 32*24 pixel requires 32*24*3*3 放大3倍 
		buffer_size = (pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height << 1)*ScaleUp_3*ScaleUp_3;

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX_TH32x24_ScaleUpFrame_addr = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_ScaleUpFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_ScaleUpFrame_addr);

		//	1 pixel takes 1 bytes (Gray out) => 32*24 pixel requires 32*24*3*3 放大3倍 
		buffer_size = (pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height )*ScaleUp_3*ScaleUp_3;

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX_TH32x24_GrayScaleUpFrame_addr = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_GrayScaleUpFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_GrayScaleUpFrame_addr);



		//	1 pixel takes 1 bytes (Gray out) => 32*24 pixel requires 32*24
		buffer_size = (pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height );

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFrame_addr = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_GrayOutputFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFrame_addr);

	nRet = STATUS_OK;
Return:
	return nRet;
}

#endif

static INT32S scaler_mem_alloc(void)
{
	INT32U buffer_addr;
	INT32S i, buffer_size, nRet;

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
	//if((AVI_ENCODE_DIGITAL_ZOOM_EN == 1) || pAviEncVidPara->scaler_flag) {
	if (1) {
        INT32U enc_w, enc_h;
        enc_w = (pAviEncVidPara->encode_width + 0xf) & ~0xf;
        enc_h = (pAviEncVidPara->encode_height + 0xf) & ~0xf;
        //if (pAviEncVidPara->video_format == C_H264_FORMAT)
        //    buffer_size = (enc_w * enc_h * 3) >> 1;
        //else
            buffer_size = (enc_w * (enc_h + 16) * 2); //h add 16 to prevent MB420 overflow
		buffer_size = (buffer_size + 0x3f) >> 6 << 6;
		encode_buffer_size = buffer_size;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size * AVI_ENCODE_SCALER_BUFFER_NO + 0x3f, 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
        pAviEncVidPara->scaler_output_addr0 = buffer_addr;
        pAviEncVidPara->JPEGEncodeInBuf = buffer_addr;
        buffer_base_addr_5M = buffer_addr;
        buffer_addr = (buffer_addr + 0x3f) >> 6 << 6;
		for(i=0; i<AVI_ENCODE_SCALER_BUFFER_NO; i++) {
			pAviEncVidPara->scaler_output_addr[i] = (INT32U)buffer_addr + buffer_size * i;
			DBG_PRINT("scaler_frame_addr[%d] = 0x%x\r\n", i, pAviEncVidPara->scaler_output_addr[i]);
		}
	}
#elif VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	//if(pAviEncVidPara->scaler_flag) {
	if (1) {
        INT32U enc_w, enc_h;
        enc_w = (pAviEncVidPara->encode_width + 0xf) & ~0xf;
        enc_h = (pAviEncVidPara->encode_height + 0xf) & ~0xf;
        buffer_size = enc_w * SENSOR_FIFO_LINE << 2;
        //buffer_size = (enc_w * SENSOR_FIFO_LINE * 3) >> 1;
/*		nRet = pAviEncVidPara->encode_height /(pAviEncVidPara->sensor_capture_height / SENSOR_FIFO_LINE);
		nRet += 2; //+2 is to fix block line
		buffer_size = pAviEncVidPara->encode_width * nRet << 1;*/
		buffer_size = (buffer_size + 0x3f) >> 6 << 6;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size * AVI_ENCODE_SCALER_BUFFER_NO + 0x3f, 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
        pAviEncVidPara->scaler_output_addr0 = buffer_addr;
        buffer_addr = (buffer_addr + 0x3f) >> 6 << 6;
		for(i=0; i<AVI_ENCODE_SCALER_BUFFER_NO; i++) {
			pAviEncVidPara->scaler_output_addr[i] = buffer_addr + buffer_size * i;
			DBG_PRINT("scaler_frame_addr[%d] = 0x%x\r\n", i, pAviEncVidPara->scaler_output_addr[i]);
		}
	}
#endif
	nRet = STATUS_OK;
Return:
	return nRet;
}

static INT32S rotate_mem_alloc(void)
{
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    INT32U buffer_addr;
    INT32S i, buffer_size, nRet;

    if(pAviEncVidPara->dispaly_scaler_flag) {
        buffer_size = pAviEncVidPara->display_buffer_width * pAviEncVidPara->display_buffer_height << 1;
        buffer_size = (buffer_size + 0x3f) >> 6 << 6;
        buffer_addr = (INT32U) gp_malloc_align(buffer_size * AVI_ENCODE_ROTATE_BUFFER_NO + 0x3f, 64);
        if(buffer_addr == 0) {
            RETURN(STATUS_FAIL);
        }
        pAviEncVidPara->rotate_input_addr0 = buffer_addr;
        buffer_addr = (buffer_addr + 0x3f) >> 6 << 6;
        for(i=0; i<AVI_ENCODE_ROTATE_BUFFER_NO; i++) {
            pAviEncVidPara->rotate_input_addr[i] = buffer_addr + buffer_size * i;
            DBG_PRINT("rotate_frame_addr[%d] = 0x%x\r\n", i, pAviEncVidPara->rotate_input_addr[i]);
        }
    }

    nRet = STATUS_OK;
Return:
    return nRet;
#else
    return STATUS_OK;
#endif
}

static INT32S display_mem_alloc(void)
{
#if AVI_ENCODE_PREVIEW_DISPLAY_EN == 1
	INT32U buffer_addr;
	INT32S i, buffer_size, nRet;

	if(pAviEncVidPara->dispaly_scaler_flag) {
		buffer_size = pAviEncVidPara->display_buffer_width * pAviEncVidPara->display_buffer_height << 1;
		buffer_size = (buffer_size + 0x3f) >> 6 << 6;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size * AVI_ENCODE_DISPALY_BUFFER_NO + 0x3f, 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
        pAviEncVidPara->display_output_addr0 = buffer_addr;
        buffer_addr = (buffer_addr + 0x3f) >> 6 << 6;
		for(i=0; i<AVI_ENCODE_DISPALY_BUFFER_NO; i++) {
			pAviEncVidPara->display_output_addr[i] = buffer_addr + buffer_size * i;
			DBG_PRINT("display_frame_addr[%d]= 0x%x\r\n", i, pAviEncVidPara->display_output_addr[i]);
		}
	}

	nRet = STATUS_OK;
Return:
	return nRet;
#else
	return STATUS_OK;
#endif
}

static INT32S video_mem_alloc(void)
{
#if AVI_ENCODE_VIDEO_ENCODE_EN == 1
	INT32U buffer_addr;
	INT32S i, buffer_size, nRet;

#ifdef VFRAME_MANAGER
	buffer_size = VFM_TOTAL_SIZE;
	if ( 0 > vfm_init(&pAviEncVidPara->vfm, 0, buffer_size, VFM_BUF_SIZE)) {
		RETURN(STATUS_FAIL);
	}
#else
    //if (pAviEncVidPara->video_format == C_H264_FORMAT)
    //    buffer_size = 0x40000;
    //else if (pAviEncVidPara->video_format == C_MJPG_FORMAT)
        buffer_size = pAviEncVidPara->encode_width * pAviEncVidPara->encode_height/4;
    //if (buffer_size > 0x40000)
        //buffer_size = 0x40000;
	buffer_addr = (INT32U) gp_malloc_align(buffer_size * AVI_ENCODE_VIDEO_BUFFER_NO + 0x3f, 32);
	if(buffer_addr == 0) {
		RETURN(STATUS_FAIL);
	}
    pAviEncVidPara->video_encode_addr0 = buffer_addr;
    pAviEncVidPara->JPEGEncodeOutBuf = buffer_addr;
    buffer_addr = (buffer_addr + 0x3f) >> 6 << 6;
	for(i=0; i<AVI_ENCODE_VIDEO_BUFFER_NO; i++) {
		pAviEncVidPara->video_encode_addr[i] = buffer_addr + buffer_size * i;
		DBG_PRINT("jpeg_frame_addr[%d]   = 0x%x\r\n", i, pAviEncVidPara->video_encode_addr[i]);
	}
#endif
	nRet = STATUS_OK;
Return:
	return nRet;
#else
	return STATUS_OK;
#endif
}

static INT32S jpeg_fifo_mem_alloc(void)
{
#if AVI_ENCODE_VIDEO_ENCODE_EN == 1
	INT32U buffer_addr;
	INT32S i, buffer_size, nRet;


	buffer_size = pAviEncVidPara->encode_width * SENSOR_FIFO_LINE *1.5;//<<1

	buffer_addr = (INT32U) gp_iram_malloc_align(buffer_size * AVI_ENCODE_CSI_FIFO_NO + 0x3F, 32);
	if(buffer_addr == 0) {
		RETURN(STATUS_FAIL);
	}

	INT32U  csi_fifo_addr0;
	INT32U  csi_fifo_addr[AVI_ENCODE_CSI_FIFO_NO];		// sensor fifo mode use
	pAviEncVidPara->csi_fifo_addr0 = buffer_addr;
	buffer_addr = (buffer_addr + 0x3f) >> 6 << 6;
	for(i=0; i<AVI_ENCODE_CSI_FIFO_NO; i++) {
		pAviEncVidPara->csi_fifo_addr[i] = buffer_addr + buffer_size * i;
		//DBG_PRINT("jpeg_fifo_addr[%d]   = 0x%x\r\n", i, pAviEncVidPara->csi_fifo_addr[i]);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
#else
	return STATUS_OK;
#endif
}

static INT32S AviPacker_mem_alloc(AviEncPacker_t *pAviEncPacker)
{
#if AVI_PACKER_LIB_EN == 1
	INT32S nRet;
	INT32U buffer_addr, buffer_size;

	pAviEncPacker->file_buffer_size = FileWriteBuffer_Size;
	pAviEncPacker->index_buffer_size = IndexBuffer_Size;
	buffer_size = AviPackerV3_GetWorkMemSize() + FileWriteBuffer_Size + IndexBuffer_Size;
	buffer_addr = (INT32U)gp_malloc_align(buffer_size, 32);
	if(buffer_addr == 0) {
		 RETURN(STATUS_FAIL);
	}

	pAviEncPacker->file_write_buffer = (void *)buffer_addr;
	pAviEncPacker->index_write_buffer = (void *)(buffer_addr + FileWriteBuffer_Size);
	pAviEncPacker->avi_workmem = (void *)(buffer_addr + FileWriteBuffer_Size + IndexBuffer_Size);

	DBG_PRINT("file_write_buffer = 0x%x\r\n", pAviEncPacker->file_write_buffer);
	DBG_PRINT("index_write_buffer= 0x%x\r\n", pAviEncPacker->index_write_buffer);
	DBG_PRINT("avi_workmem_buffer= 0x%x\r\n", pAviEncPacker->avi_workmem);

	nRet = STATUS_OK;
Return:
	return nRet;
#else
	pAviEncPacker->file_write_buffer = 0;
	return STATUS_OK;
#endif
}

static void sensor_mem_free(void)
{
	INT32S i;

	if(pAviEncVidPara->csi_frame_addr0) {
		gp_free((void*)pAviEncVidPara->csi_frame_addr0);
	}

	for(i=0; i<AVI_ENCODE_CSI_BUFFER_NO; i++) {
		pAviEncVidPara->csi_frame_addr[i] = 0;
	}
}

static void scaler_mem_free(void)
{
	INT32S i;

	if(pAviEncVidPara->scaler_output_addr0) {
		gp_free((void*)pAviEncVidPara->scaler_output_addr0);
	}

	for(i=0; i<AVI_ENCODE_SCALER_BUFFER_NO; i++) {
		pAviEncVidPara->scaler_output_addr[i] = 0;
	}
}

static void rotate_mem_free(void)
{
#if AVI_ENCODE_PREVIEW_DISPLAY_ROTATE_EN
    INT32S i;

    if(pAviEncVidPara->rotate_input_addr0) {
        gp_free((void*)pAviEncVidPara->rotate_input_addr0);
    }

    for(i=0; i<AVI_ENCODE_ROTATE_BUFFER_NO; i++) {
        pAviEncVidPara->rotate_input_addr[i] = 0;
    }
#endif
}

static void display_mem_free(void)
{
	INT32S i;

	if(pAviEncVidPara->display_output_addr0) {
		gp_free((void*)pAviEncVidPara->display_output_addr0);
	}

	for(i=0; i<AVI_ENCODE_DISPALY_BUFFER_NO; i++) {
		pAviEncVidPara->display_output_addr[i] = 0;
	}
}

static void video_mem_free(void)
{
	INT32S i;

	if(pAviEncVidPara->video_encode_addr0) {
		gp_free((void*)pAviEncVidPara->video_encode_addr0);
	}

	for(i=0; i<AVI_ENCODE_VIDEO_BUFFER_NO; i++) {
		pAviEncVidPara->video_encode_addr[i] = 0;
	}
#ifdef VFRAME_MANAGER
	vfm_close(&pAviEncVidPara->vfm);
#endif
}

static void AviPacker_mem_free(AviEncPacker_t *pAviEncPacker)
{

	if(pAviEncPacker->file_write_buffer) {
		gp_free((void*)pAviEncPacker->file_write_buffer);
	}

	pAviEncPacker->file_write_buffer = 0;
	pAviEncPacker->index_write_buffer = 0;
	pAviEncPacker->avi_workmem = 0;
}

INT32S avi_encode_memory_alloc(void)
{
	INT32S i,nRet;

    osDelay(20);
	//inti index
	g_csi_index = 0;
	g_pcm_index = 0;
/*	if(sensor_mem_alloc() < 0) {
		RETURN(STATUS_FAIL);
	}*/

    if(buffer_init == 0)
    {
        if(scaler_mem_alloc() < 0) {
            RETURN(STATUS_FAIL);
        }
        buffer_init = 1;
    }
    else
    {
        if(buffer_base_addr_5M)
        {
            pAviEncVidPara->scaler_output_addr0 = buffer_base_addr_5M;
            pAviEncVidPara->JPEGEncodeInBuf = buffer_base_addr_5M;
            for(i=0; i<AVI_ENCODE_SCALER_BUFFER_NO; i++) {
                pAviEncVidPara->scaler_output_addr[i] = (INT32U)buffer_base_addr_5M + encode_buffer_size * i;
                DBG_PRINT("scaler_frame_addr[%d] = 0x%x\r\n", i, pAviEncVidPara->scaler_output_addr[i]);
            }
        }
        else
            RETURN(STATUS_FAIL);
    }

    if(rotate_mem_alloc() < 0) {
        RETURN(STATUS_FAIL);
    }

	if(display_mem_alloc() < 0) {
		RETURN(STATUS_FAIL);
	}

	if(video_mem_alloc() < 0) {
		RETURN(STATUS_FAIL);
	}

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	if(jpeg_fifo_mem_alloc()<0){
		RETURN(STATUS_FAIL);
	}
#endif

#if 1
	if(MLX_TH32x24_mem_alloc() < 0) {	// RAM 正常 
		RETURN(STATUS_FAIL);
	}
#endif

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S avi_packer_memory_alloc(void)
{
	INT32S nRet;

	if(AviPacker_mem_alloc(pAviEncPacker0) < 0) {
		RETURN(STATUS_FAIL);
	}
#if AVI_ENCODE_FAST_SWITCH_EN == 1
	if(AviPacker_mem_alloc(pAviEncPacker1) < 0) {
		RETURN(STATUS_FAIL);
	}
#endif
	nRet = STATUS_OK;
Return:
	return nRet;
}

void avi_encode_memory_free(void)
{
	sensor_mem_free();
	if(buffer_base_addr_5M == 0)
        scaler_mem_free();
	rotate_mem_free();
	display_mem_free();
	video_mem_free();
}

void avi_packer_memory_free(void)
{
	AviPacker_mem_free(pAviEncPacker0);
#if AVI_ENCODE_FAST_SWITCH_EN == 1
	AviPacker_mem_free(pAviEncPacker1);
#endif
}

// format
void avi_encode_set_sensor_format(INT32U format)
{
	switch(format)
	{
	case V4L2_PIX_FMT_YUYV:
		//pAviEncVidPara->sensor_output_format = C_SCALER_CTRL_IN_VYUY;
		pAviEncVidPara->sensor_output_format = C_SCALER_CTRL_IN_UYVY;
		break;
	case V4L2_PIX_FMT_YVYU:
		pAviEncVidPara->sensor_output_format = C_SCALER_CTRL_IN_UYVY;
		break;
	case V4L2_PIX_FMT_UYVY:
		pAviEncVidPara->sensor_output_format = C_SCALER_CTRL_IN_UYVY;
		break;
	case V4L2_PIX_FMT_VYUY:
		pAviEncVidPara->sensor_output_format = C_SCALER_CTRL_IN_YUYV;
		break;
	}
}

void avi_encode_set_display_format(INT32U format)
{
	if(format == IMAGE_OUTPUT_FORMAT_RGB565) {
		pAviEncVidPara->display_output_format = C_SCALER_CTRL_OUT_RGB565;
	} else if(format == IMAGE_OUTPUT_FORMAT_YUYV) {
    	pAviEncVidPara->display_output_format = C_SCALER_CTRL_OUT_YUYV;
    } else if(format == IMAGE_OUTPUT_FORMAT_UYVY) {
    	pAviEncVidPara->display_output_format = C_SCALER_CTRL_OUT_UYVY;
    } else {
    	pAviEncVidPara->display_output_format = C_SCALER_CTRL_OUT_YUYV;
	}
}

void avi_encode_set_display_scaler(void)
{
    pAviEncVidPara->dispaly_scaler_flag = 1;
    pAviEncVidPara->scaler_flag = 0;
	/*if((pAviEncVidPara->encode_width != pAviEncVidPara->display_buffer_width)||
		(pAviEncVidPara->encode_height != pAviEncVidPara->display_buffer_height) ||
		(pAviEncVidPara->display_width != pAviEncVidPara->display_buffer_width) ||
		(pAviEncVidPara->display_height != pAviEncVidPara->display_buffer_height) ||
		(pAviEncVidPara->display_output_format != C_SCALER_CTRL_OUT_YUYV)) {
		pAviEncVidPara->dispaly_scaler_flag = 1;
	 } else {
		pAviEncVidPara->dispaly_scaler_flag = 0;
	}

	if((pAviEncVidPara->sensor_capture_width != pAviEncVidPara->encode_width) ||
		(pAviEncVidPara->sensor_capture_height != pAviEncVidPara->encode_height)) {
		pAviEncVidPara->scaler_flag = 1;
	} else {
		pAviEncVidPara->scaler_flag = 0;
	}*/
}

INT32S avi_encode_set_jpeg_quality(INT8U quality_value)
{
	INT32U i, video_frame;

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
pAviEncVidPara->quality_value = jpeg_header_generate(JPEG_IMG_FORMAT_420, quality_value,
														pAviEncVidPara->encode_width,
														pAviEncVidPara->encode_height);
#else
    if(pAviEncVidPara->jpeg_encode_enable_flag && pAviEncVidPara->encode_different_flag)
    {
        if(pAviEncVidPara->jpeg_encode_in_format)
            pAviEncVidPara->quality_value = jpeg_header_generate(JPEG_IMG_FORMAT_420, quality_value,
                                                                pAviEncVidPara->jpeg_encode_width,
                                                                pAviEncVidPara->jpeg_encode_height);
        else
            pAviEncVidPara->quality_value = jpeg_header_generate(JPEG_IMG_FORMAT_422, quality_value,
                                                                pAviEncVidPara->jpeg_encode_width,
                                                                pAviEncVidPara->jpeg_encode_height);
    }
    else
        pAviEncVidPara->quality_value = jpeg_header_generate(JPEG_IMG_FORMAT_422, quality_value,
														pAviEncVidPara->encode_width,
														pAviEncVidPara->encode_height);
#endif

#ifndef VFRAME_MANAGER
	//copy to video buffer
	for(i = 0; i<AVI_ENCODE_VIDEO_BUFFER_NO; i++) {
		video_frame = pAviEncVidPara->video_encode_addr[i];
		memcpy((void *)video_frame, (void *)jpeg_header_get_addr(), jpeg_header_get_size());
	}
#endif
	//return pAviEncVidPara->quality_value;
	return jpeg_header_get_size();
}

// check disk free size
BOOLEAN avi_encode_disk_size_is_enough(INT32S cb_write_size)
{
	INT32S nRet;
#if AVI_ENCODE_CAL_DISK_SIZE_EN	== 1
	INT32U temp;
	INT64U disk_free_size;

	temp = pAviEncPara->record_total_size;
	disk_free_size = pAviEncPara->disk_free_size;
	temp += cb_write_size;
	if(temp >= AVI_FILE_MAX_RECORD_SIZE) {
		RETURN(FALSE);
	}

	if(temp >= disk_free_size) {
		RETURN(FALSE);
	}
	pAviEncPara->record_total_size = temp;
#endif

	nRet = TRUE;
Return:
	return nRet;
}

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FRAME_MODE
// csi frame mode switch buffer
void csi_eof_isr(INT32U event)
{
	if(event == CSI_SENSOR_FRAME_END_EVENT) {
        //DBG_PRINT("davis\r\n");
		if (pscaler_start_1)
		{
			pscaler_start_1 = 0;
			drv_l1_pscaler_start(1);//pscaler for MB420 better start at csi or cdsp frame end.
		}
		/*INT32U frame, ready_frame;
		osEvent result;
		osStatus status;

        DBG_PRINT(".");
		result = osMessageGet(cmos_frame_q, 1);
		frame = result.value.v;
		if((result.status != osEventMessage) || (frame == 0)) {
			DBG_PRINT("L");
			return;
		}

		ready_frame = *P_CSI_TG_FBSADDR;
		*P_CSI_TG_FBSADDR = frame;
		status = osMessagePut(scaler_task_q, (INT32U)&ready_frame, osWaitForever);
		if(status != osOK) {
			DBG_PRINT("PF");
		}*/
	}

	if(event == CSI_SENSOR_FIFO_OVERFLOW_EVENT) {
		DBG_PRINT("UnderRun\r\n");
	}
}

void cdsp_eof_isr(void)
{
	INT32U frame, ready_frame;
	osEvent result;
	osStatus status;

	if (pscaler_start_1)
	{
		//DBG_PRINT("davis\r\n");
		pscaler_start_1 = 0;
		drv_l1_pscaler_start(1);//pscaler for MB420 better start at csi or cdsp frame end.
	}

/*
#if C_DMA_CH == 0
	pAviEncPara->abBufFlag ^= 0x01;
#elif C_DMA_CH == 1
	pAviEncPara->abBufFlag = 1;
#elif C_DMA_CH == 2
	pAviEncPara->abBufFlag = 0;
#endif

	result = osMessageGet(cmos_frame_q, 1);
	frame = result.value.v;
	if((result.status != osEventMessage) || (frame == 0)) {
		DBG_PRINT("L");
		return;
	}

    if(pAviEncPara->abBufFlag) {
    	ready_frame = R_CDSP_DMA_YUVABUF_SADDR;
    	R_CDSP_DMA_YUVABUF_SADDR = frame;
	} else {
    	ready_frame = R_CDSP_DMA_YUVBBUF_SADDR;
    	R_CDSP_DMA_YUVBBUF_SADDR = frame;
	}
#if VIDEO_ENCODE_MODE != C_VIDEO_ENCODE_FIFO_MODE
	status = osMessagePut(scaler_task_q, (INT32U)&ready_frame, osWaitForever);
	if(status != osOK) {
		DBG_PRINT("PF");
	}
#endif*/
}
#endif
void csi_capture_isr(INT32U event)
{
	if(event == CSI_SENSOR_FRAME_END_EVENT) {
		INT32U frame;
		osStatus status;

		pAviEncPara->skip_cnt--;
		if(pAviEncPara->skip_cnt > 0) {
			return;
		}

		R_CSI_TG_CTRL0 &= ~0x01;
		frame = *P_CSI_TG_FBSADDR;
		status = osMessagePut(pCap->Sensor_m, (INT32U)&frame, osWaitForever);
		if(status != osOK) {
			DBG_PRINT("PF");
		}
	}

	if(event == CSI_SENSOR_FIFO_OVERFLOW_EVENT) {
		DBG_PRINT("UnderRun\r\n");
	}
}

void cdsp_capture_isr(void)
{
	INT32U frame;
	osStatus status;

#if C_CDSP_DMA_CH == 0
	pAviEncPara->abBufFlag ^= 0x01;
#elif C_CDSP_DMA_CH == 1
	pAviEncPara->abBufFlag = 1;
#elif C_CDSP_DMA_CH == 2
	pAviEncPara->abBufFlag = 0;
#endif

	pAviEncPara->skip_cnt--;
	if(pAviEncPara->skip_cnt > 0) {
		return;
	}

	if(pAviEncPara->abBufFlag) {
		frame = R_CDSP_DMA_YUVABUF_SADDR;
	} else {
		frame = R_CDSP_DMA_YUVBBUF_SADDR;
	}

	status = osMessagePut(pCap->Sensor_m, (INT32U)&frame, osWaitForever);
	if(status != osOK) {
		DBG_PRINT("PF");
	}
}

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
// csi fifo mode switch buffer
static void csi_fifo_end(void)
{
	INT32U ready_frame;
	osStatus status;

	//R_IOC_O_DATA ^= 0x04;
	pAviEncPara->fifo_irq_cnt++;
	if((pAviEncPara->fifo_irq_cnt - pAviEncPara->vid_pend_cnt) >= 2) {
		pAviEncPara->fifo_enc_err_flag = 1;
		return;
	}

	if(((pAviEncPara->avi_encode_status & C_AVI_ENCODE_PRE_START) == 0) || pAviEncPara->fifo_enc_err_flag) {
		return;
	}

	ready_frame = pAviEncVidPara->csi_fifo_addr[pAviEncPara->fifo_irq_cnt - 1];
	if(pAviEncPara->fifo_irq_cnt >= pAviEncPara->vid_post_cnt) {
		ready_frame |= C_AVI_ENCODE_FRAME_END;
	}

	status = osMessagePut(vid_enc_task_q, (INT32U)&ready_frame, osWaitForever);
	if(status != osOK) {
		DBG_PRINT("L");
		pAviEncPara->fifo_enc_err_flag = 1;
		return;
	}
}

// csi fifo mode frame end, take time: post 30 times 80us
static void csi_fifo_frame_end(void)
{
	INT8U i;
	INT32U message;

	if(pAviEncPara->fifo_irq_cnt != pAviEncPara->vid_post_cnt) {
		DBG_PRINT("[%x]\r\n", pAviEncPara->fifo_irq_cnt);
		pAviEncPara->fifo_enc_err_flag = 1;
	}

	if(pAviEncPara->vid_pend_cnt && pAviEncPara->fifo_enc_err_flag) {
		OSQFlush(vid_enc_task_q);
		message = C_AVI_ENCODE_FIFO_ERR;
		osMessagePut(vid_enc_task_q, (INT32U)&message, osWaitForever);
	}

	pAviEncPara->fifo_enc_err_flag = 0;
	pAviEncPara->fifo_irq_cnt = 0;
	for(i = 0; i<pAviEncPara->vid_post_cnt; i++) {
		pAviEncVidPara->csi_fifo_addr[i] = avi_encode_get_csi_frame();
	}
}

void csi_fifo_isr(INT32U event)
{
	if(event == CSI_SENSOR_OUTPUT_FIFO_EVENT) {
		csi_fifo_end();
	}

	if(event == CSI_SENSOR_FRAME_END_EVENT) {
		csi_fifo_frame_end();
	}

	if(event == CSI_SENSOR_FIFO_OVERFLOW_EVENT) {
		DBG_PRINT("UnderRun\r\n");
	}
}

void cdsp_fifo_isr(void)
{

}
#endif

// jpeg once encode
INT32S jpeg_encode_once(JpegPara_t *pJpegEnc)
{
	INT32S nRet;

	jpeg_encode_init();
	gplib_jpeg_default_quantization_table_load(pJpegEnc->quality_value);	// Load default qunatization table(quality)
	gplib_jpeg_default_huffman_table_load();								// Load default huffman table

	nRet = jpeg_encode_input_size_set(pJpegEnc->width, pJpegEnc->height);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	jpeg_encode_input_format_set(pJpegEnc->input_format);					// C_JPEG_FORMAT_YUYV / C_JPEG_FORMAT_YUV_SEPARATE

	if(pAviEncVidPara->jpeg_encode_enable_flag)
	{
        if(pAviEncVidPara->jpeg_encode_in_format)
            nRet = jpeg_encode_yuv_sampling_mode_set((C_JPG_CTRL_GP420 | C_JPG_CTRL_YUV420));
        else
	nRet = jpeg_encode_yuv_sampling_mode_set(C_JPG_CTRL_YUV422);
	}
	else
        nRet = jpeg_encode_yuv_sampling_mode_set(C_JPG_CTRL_YUV422);

	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = jpeg_encode_output_addr_set((INT32U) pJpegEnc->output_buffer);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = jpeg_encode_once_start(pJpegEnc->input_buffer_y, pJpegEnc->input_buffer_u, pJpegEnc->input_buffer_v);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	while(1) {
		pJpegEnc->jpeg_status = jpeg_encode_status_query(TRUE);
		if(pJpegEnc->jpeg_status & C_JPG_STATUS_ENCODE_DONE) {
			// Get encode length
			nRet = jpeg_encode_vlc_cnt_get();
			cache_invalid_range(pJpegEnc->output_buffer, nRet);
			break;
		} else if(pJpegEnc->jpeg_status & C_JPG_STATUS_ENCODING) {
			continue;
		} else {
			DBG_PRINT("JPEG encode error!\r\n");
			RETURN(STATUS_FAIL);
		}
	}

Return:
	jpeg_encode_stop();
	return nRet;
}

// jpeg fifo encode
INT32S jpeg_encode_fifo_start(JpegPara_t *pJpegEnc)
{
	INT32S nRet;

	jpeg_encode_init();
	gplib_jpeg_default_quantization_table_load(pJpegEnc->quality_value);		// Load default qunatization table(quality=50)
	gplib_jpeg_default_huffman_table_load();			        // Load default huffman table

#if VIDEO_ENCODE_MODE == C_VIDEO_ENCODE_FIFO_MODE
	nRet = jpeg_encode_yuv_sampling_mode_set(C_JPG_CTRL_YUV420 | C_JPG_CTRL_GP420);//C_JPG_CTRL_YUV420
#if JPEG_X1_5_SCALE == 1
	drv_l1_jpeg_scaler_up_x1_5();
#endif
#else
	nRet = jpeg_encode_yuv_sampling_mode_set(C_JPG_CTRL_YUV422);
#endif

	nRet = jpeg_encode_input_size_set(pJpegEnc->width, pJpegEnc->height);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	jpeg_encode_input_format_set(pJpegEnc->input_format);

	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = jpeg_encode_output_addr_set((INT32U)pJpegEnc->output_buffer);
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	nRet = jpeg_encode_on_the_fly_start(pJpegEnc->input_buffer_y,
										pJpegEnc->input_buffer_u,
										pJpegEnc->input_buffer_v,
										(pJpegEnc->input_y_len + pJpegEnc->input_uv_len + pJpegEnc->input_uv_len));
	if(nRet < 0) {
		RETURN(STATUS_FAIL);
	}

	if(pJpegEnc->wait_done == 0) {
		RETURN(STATUS_OK);
	}

	nRet = jpeg_encode_status_query(TRUE);	//wait jpeg block encode done
	if(nRet & (C_JPG_STATUS_TIMEOUT | C_JPG_STATUS_INIT_ERR |C_JPG_STATUS_RST_MARKER_ERR)) {
		nRet = STATUS_FAIL;
	}
Return:
	return nRet;
}

INT32S jpeg_encode_fifo_once(JpegPara_t *pJpegEnc)
{
	INT32S nRet;

	while(1) {
		if(pJpegEnc->wait_done == 0) {
			pJpegEnc->jpeg_status = jpeg_encode_status_query(TRUE);
		}

	   	if(pJpegEnc->jpeg_status & C_JPG_STATUS_ENCODE_DONE) {
	    	RETURN(C_JPG_STATUS_ENCODE_DONE);
	    } else if(pJpegEnc->jpeg_status & C_JPG_STATUS_INPUT_EMPTY) {
   	    	nRet = jpeg_encode_on_the_fly_start(pJpegEnc->input_buffer_y,
	    										pJpegEnc->input_buffer_u,
	    										pJpegEnc->input_buffer_v,
	    										(pJpegEnc->input_y_len + pJpegEnc->input_uv_len + pJpegEnc->input_uv_len));
	    	if(nRet < 0) {
	    		RETURN(nRet);
	    	}

	    	if(pJpegEnc->wait_done == 0) {
	    		RETURN(STATUS_OK);
	    	}

	    	pJpegEnc->jpeg_status = jpeg_encode_status_query(TRUE);	//wait jpeg block encode done
	    	RETURN(pJpegEnc->jpeg_status);
	    } else if(pJpegEnc->jpeg_status & C_JPG_STATUS_STOP) {
	        DBG_PRINT("\r\njpeg encode is not started!\r\n");
	    	RETURN(C_JPG_STATUS_STOP);
	    } else if(pJpegEnc->jpeg_status & C_JPG_STATUS_TIMEOUT) {
	        DBG_PRINT("\r\njpeg encode execution timeout!\r\n");
	        RETURN(C_JPG_STATUS_TIMEOUT);
	    } else if(pJpegEnc->jpeg_status & C_JPG_STATUS_INIT_ERR) {
	         DBG_PRINT("\r\njpeg encode init error!\r\n");
	         RETURN(C_JPG_STATUS_INIT_ERR);
	    } else {
	    	DBG_PRINT("\r\nJPEG status error = 0x%x!\r\n", pJpegEnc->jpeg_status);
	        RETURN(STATUS_FAIL);
	    }
    }

Return:
	return nRet;
}

INT32S jpeg_encode_fifo_stop(JpegPara_t *pJpegEnc)
{
	INT32S nRet;

	if(pJpegEnc->wait_done == 0) {
		pJpegEnc->jpeg_status = jpeg_encode_status_query(TRUE);
	}

	while(1) {
		if(pJpegEnc->jpeg_status & C_JPG_STATUS_ENCODE_DONE) {
			nRet = jpeg_encode_vlc_cnt_get();	// get jpeg encode size
			RETURN(nRet);
		} else if(pJpegEnc->jpeg_status & C_JPG_STATUS_INPUT_EMPTY) {
			nRet = jpeg_encode_on_the_fly_start(pJpegEnc->input_buffer_y,
												pJpegEnc->input_buffer_u,
												pJpegEnc->input_buffer_v,
												(pJpegEnc->input_y_len + pJpegEnc->input_uv_len + pJpegEnc->input_uv_len));
			if(nRet < 0) {
				RETURN(STATUS_FAIL);
			}
			pJpegEnc->jpeg_status = jpeg_encode_status_query(TRUE);	//wait jpeg block encode done
		} else if(pJpegEnc->jpeg_status & C_JPG_STATUS_STOP) {
	        DBG_PRINT("\r\njpeg encode is not started!\r\n");
	    	RETURN(STATUS_FAIL);
	    } else if(pJpegEnc->jpeg_status & C_JPG_STATUS_TIMEOUT) {
	        DBG_PRINT("\r\njpeg encode execution timeout!\r\n");
	        RETURN(STATUS_FAIL);
	    } else if(pJpegEnc->jpeg_status & C_JPG_STATUS_INIT_ERR) {
	         DBG_PRINT("\r\njpeg encode init error!\r\n");
	         RETURN(STATUS_FAIL);
	    } else {
	    	DBG_PRINT("\r\nJPEG status error = 0x%x!\r\n", pJpegEnc->jpeg_status);
	        RETURN(STATUS_FAIL);
	    }
	}

Return:
	jpeg_encode_stop();
	return nRet;
}

INT32S save_jpeg_file(INT16S fd, INT32U encode_frame, INT32U encode_size)
{
	INT32S nRet;

	if(pAviEncPara->source_type == SOURCE_TYPE_FS) {
		nRet = fs_write(fd, encode_frame, encode_size);
		if(nRet != encode_size) {
			RETURN(STATUS_FAIL);
		}

		nRet = fs_close(fd);
		if(nRet < 0) {
			RETURN(STATUS_FAIL);
		}

		fd = -1;
	} else {
		if(pAviEncPara->memptr == 0) {
			RETURN(STATUS_FAIL);
		}

		memcpy((void *)pAviEncPara->memptr, (void *)encode_frame, encode_size);
	}

	nRet = STATUS_OK;
Return:
	return nRet;
}

INT32S h264_encode_start(INT32S width, INT32S height)
{
    gp_h264_cfg cfg;

    cfg.width = width;
    cfg.height = height;
	if (width <= 1280 && height <= 720)
		cfg.level = 31;
	else
		cfg.level = 41;

    h264_handle = gp_h264enc_init(&cfg);

    if (!h264_handle)
        return STATUS_FAIL;

    return STATUS_OK;
}

/* please reset RC at the beginning of "every" video clip */
INT32S h264_encode_reset_rc()
{
    gp_h264enc_rate_ctrl rccfg;

    /* please set bitPerSeconds=0 if you want to disable RC and use a fixed QP (qpHdr) */
    rccfg.bitPerSeconds = pAviEncVidPara->h264_bitrate*1000;
    rccfg.frameRate = pAviEncVidPara->dwRate;
    rccfg.gopLen = pAviEncVidPara->h264_gop_len;
    rccfg.qpHdr = 32;
    rccfg.qpMax = 51;
    rccfg.qpMin = 20;
    rccfg.ae_night = drv_l2_cdsp_ae_is_night();
    gp_h264enc_set_rc(h264_handle, &rccfg);
}

INT32S h264_encode_stop(void)
{
    if (!h264_handle)
        return STATUS_FAIL;

    gp_h264enc_release(h264_handle);

    h264_handle = NULL;

    return STATUS_OK;
}

INT32S h264_encode_frame(INT32U frameAddr, INT32U outAddr, INT32U PictureType, INT32S insertHdr, INT32U NullFrame)
{
    gp_h264enc_in in;
    INT32S stream_size;

    if (!h264_handle)
        return STATUS_FAIL;

    in.RawFrameAddr = frameAddr;
    in.BitstreamOutAddr = outAddr;
    in.PictureType = PictureType;
    in.InsertHdr = insertHdr;
    in.bitrate_level = 0;
    in.ae_night = drv_l2_cdsp_ae_is_night();

    if (NullFrame)
        gp_h264enc_insert_skip(h264_handle, &in);
    else
        gp_h264enc_encode(h264_handle, &in);
    stream_size = gp_h264enc_wait(h264_handle, 0);

    return stream_size;
}

// audio function
INT32S avi_audio_memory_allocate(INT32U	cblen)
{
	INT16U *ptr;
	INT32S i, j, size, nRet;

	for(i=0; i<AVI_ENCODE_PCM_BUFFER_NO; i++) {
		pAviEncAudPara->pcm_input_addr[i] = (INT32U) gp_malloc_align(cblen, 4);
		if(!pAviEncAudPara->pcm_input_addr[i]) {
			RETURN(STATUS_FAIL);
		}

		ptr = (INT16U*)pAviEncAudPara->pcm_input_addr[i];
		for(j=0; j<(cblen/2); j++) {
			*ptr++ = 0x8000;
		}
	}

	size = pAviEncAudPara->pack_size * C_WAVE_ENCODE_TIMES;
	#if AVI_PACKER_LIB_EN == 0
	for (i = 0; i < AVI_ENCODE_PCM_BUFFER_NO; i++)
	{
		pAviEncAudPara->pack_buffer[i] = (INT32U) gp_malloc_align(size, 4);
		if(!pAviEncAudPara->pack_buffer[i])
			RETURN(STATUS_FAIL);
	}
	pAviEncAudPara->pack_buffer_addr = pAviEncAudPara->pack_buffer[0];
	#else
	pAviEncAudPara->pack_buffer_addr = (INT32U) gp_malloc_align(size, 4);
	#endif
	if(!pAviEncAudPara->pack_buffer_addr) {
		RETURN(STATUS_FAIL);
	}
	nRet = STATUS_OK;
Return:
	return nRet;
}

void avi_audio_memory_free(void)
{
	INT32U i;

	for(i=0; i<AVI_ENCODE_PCM_BUFFER_NO; i++) {
		gp_free((void *)pAviEncAudPara->pcm_input_addr[i]);
		pAviEncAudPara->pcm_input_addr[i] = 0;
	}

	#if AVI_PACKER_LIB_EN == 0
	for(i=0; i<AVI_ENCODE_PCM_BUFFER_NO; i++) {
		if (pAviEncAudPara->pack_buffer[i])
			gp_free((void *)pAviEncAudPara->pack_buffer[i]);
		pAviEncAudPara->pack_buffer[i] = 0;
	}
	#else
	gp_free((void *)pAviEncAudPara->pack_buffer_addr);
	#endif
	pAviEncAudPara->pack_buffer_addr = 0;
}

INT32U avi_audio_get_next_buffer(void)
{
	INT32U addr;

	addr = pAviEncAudPara->pcm_input_addr[g_pcm_index++];
	if(g_pcm_index >= AVI_ENCODE_PCM_BUFFER_NO) {
		g_pcm_index = 0;
	}
	return addr;
}

void avi_adc_hw_start(INT32U sample_rate)
{
#if AVI_AUDIO_ENCODE_EN == 1
#if MIC_INPUT_SRC == C_ADC_LINE_IN
	drv_l1_adc_fifo_clear();
	drv_l1_adc_user_line_in_en(AVI_AUDIO_RECORD_ADC_CH, ENABLE);
	drv_l1_adc_auto_ch_set(AVI_AUDIO_RECORD_ADC_CH);
	drv_l1_adc_fifo_level_set(4);
	drv_l1_adc_auto_sample_start();
	drv_l1_adc_sample_rate_set(AVI_AUDIO_RECORD_TIMER, sample_rate);

#elif MIC_INPUT_SRC == C_BUILDIN_MIC_IN
	drv_l1_i2s_rx_adc_set_input_path(MIC_INPUT);
	drv_l1_i2s_rx_adc_init();
	drv_l1_i2s_rx_adc_sample_rate_set(sample_rate);
	drv_l1_i2s_rx_adc_mono_ch_set();
	drv_l1_i2s_rx_adc_start();

#elif MIC_INPUT_SRC == C_LINE_IN_LR
	drv_l1_i2s_rx_adc_set_input_path(LINE_IN_LR);
	drv_l1_i2s_rx_adc_init();
	drv_l1_i2s_rx_adc_sample_rate_set(sample_rate);
	drv_l1_i2s_rx_adc_start();
#endif
#endif
}

void avi_adc_hw_stop(void)
{
#if AVI_AUDIO_ENCODE_EN == 1
#if MIC_INPUT_SRC == C_ADC_LINE_IN
	drv_l1_adc_auto_sample_stop();
#elif MIC_INPUT_SRC == C_BUILDIN_MIC_IN || MIC_INPUT_SRC == C_LINE_IN_LR
	drv_l1_i2s_rx_adc_stop();
	drv_l1_i2s_rx_adc_exit();
#endif
#endif
}

#if	AVI_ENCODE_SHOW_TIME == 1

const INT8U *number[] = {
	acFontHZArial01700030,
	acFontHZArial01700031,
	acFontHZArial01700032,
	acFontHZArial01700033,
	acFontHZArial01700034,
	acFontHZArial01700035,
	acFontHZArial01700036,
	acFontHZArial01700037,
	acFontHZArial01700038,
	acFontHZArial01700039
};

const INT8U *specMark[] = {
	acFontHZArial017Slash,	// 0
	acFontHZArial017Dot,
	acFontHZArial017Comma,
	acFontHZArialSlash,		// 3
	acFontHZArialDot,
	acFontHZArialCommon,
	acFontHZArial017_H,		// 6
	acFontHZArial017_L,
	acFontHZArial017_M,
	acFontHZArial017_E,
	acFontHZArial_d	,	// 10
	acFontHZArial017_G,
	acFontHZArial017_B,
	acFontHZArial017_Y,
	acFontHZArial017_U, // 14
	acFontHZArial017_O,
	acFontHZArial_u,
	acFontHZArial_o		// 17
};



//  Draw OSD function
void cpu_draw_osd(const INT8U *source_addr, INT32U target_addr, INT16U offset, INT16U res)
{
	const INT8U* chptr;
	INT8U i,j,tmp;
	INT8U *ptr;

	ptr = (INT8U*) target_addr;
	ptr+= offset;
	chptr = source_addr;
	for(i=0; i<19; i++) {
		tmp = *chptr++;
		for(j=0; j<8; j++) {
			if(tmp & 0x80) {
				*ptr++ =0x80;
				*ptr++ = 0xff;
			} else {
				ptr += 2;
			}
			tmp = tmp<<1;
		}
 		ptr += (res-8)*2;
 	}
}

#if 0
void cpu_draw_time_osd(TIME_T current_time, INT32U target_buffer, INT16U resolution)
{
	INT8U  data;
	INT16U offset, space, wtemp;
	INT32U line;

	if(resolution == 320) {
		line = target_buffer + 220*resolution*2;//QVGA
		offset = 160*2;
	} else {
		line = target_buffer + 440*resolution*2;//VGA
		offset = 480*2;
	}
	space = 16;
	//Arial 17
	//year
	if (current_time.tm_year > 2008) {
		wtemp = current_time.tm_year - 2000;
		cpu_draw_osd(number[2], line, offset, resolution);
		cpu_draw_osd(number[0],line,offset+space*1,resolution);
		data = wtemp/10;
		wtemp -= data*10;
		cpu_draw_osd(number[data],line,offset+space*2,resolution);
		data = wtemp;
		cpu_draw_osd(number[data],line,offset+space*3,resolution);
	} else {
		cpu_draw_osd(number[2], line, offset, resolution);
		cpu_draw_osd(number[0],line,offset+space*1,resolution);
		cpu_draw_osd(number[0],line,offset+space*2,resolution);
		cpu_draw_osd(number[8],line,offset+space*3,resolution);
	}

	// :
	cpu_draw_osd(acFontHZArial017Slash,line,offset+space*4,resolution);

	//month
	wtemp = current_time.tm_mon;
	cpu_draw_osd(number[wtemp/10],line,offset+space*5,resolution);
	cpu_draw_osd(number[wtemp%10],line,offset+space*6,resolution);

	//:
	cpu_draw_osd(acFontHZArial017Slash,line,offset+space*7,resolution);

	//day
	wtemp = current_time.tm_mday;
	cpu_draw_osd(number[wtemp/10],line,offset+space*8,resolution);
	cpu_draw_osd(number[wtemp%10],line,offset+space*9,resolution);

	//hour
	wtemp = current_time.tm_hour;
	cpu_draw_osd(number[wtemp/10],line,offset+space*11,resolution);
	cpu_draw_osd(number[wtemp%10],line,offset+space*12,resolution);

	// :
	cpu_draw_osd(acFontHZArial017Comma, line,offset+space*13,resolution);

	//minute
	wtemp = current_time.tm_min;
	cpu_draw_osd(number[wtemp/10],line,offset+space*14,resolution);
	cpu_draw_osd(number[wtemp%10],line,offset+space*15,resolution);

	// :
	cpu_draw_osd(acFontHZArial017Comma,line,offset+space*16,resolution);

	//second
	wtemp = current_time.tm_sec;
	cpu_draw_osd(number[wtemp/10], line,offset+space*17,resolution);
	cpu_draw_osd(number[wtemp%10],line,offset+space*18,resolution);

	//drain cache
	if(resolution == 320) {
		cache_drain_range((target_buffer + (resolution*440*2)), resolution*(480-440)*2);
	} else {
		cache_drain_range((target_buffer + (resolution*220*2)), resolution*(240-220)*2);
	}
}
#endif

#if 1

void cpu_draw_advalue_osd(INT32S value, INT32U target_buffer, 
		INT16U resolution,INT8U st_val, INT8U shift_val , INT8U spec_val )
{
	INT8U  data;
	INT16U offset, space, wtemp;
	INT32U line;
	if(resolution == 320){
		line = target_buffer + 220*resolution*2;//QVGA
		offset = st_val*2 + shift_val;
	} 

	if(resolution == 720){
		line = target_buffer + 460*resolution*2;// DISDEV_HDMI_480P 
		offset = st_val*2 + shift_val;
		space = 26;
	} 

#if DISDEV_HDMI_480P
	//space = 16;
	
	
	//Arial 17
	//year
	wtemp = value;
	//data = wtemp/10000;	// 減少 至 4位輸出 
	//wtemp -= data*10000;
	//cpu_draw_osd(number[data], line, offset, resolution);
	data = wtemp/1000;
	wtemp -= data*1000;
	cpu_draw_osd(number[data],line,offset,resolution);
	data = wtemp/100;
	wtemp -= data*100;
	cpu_draw_osd(number[data],line,offset+space*1,resolution);
	data = wtemp/10;
	wtemp -= data*10;
	cpu_draw_osd(number[data],line,offset+space*2,resolution);
	data = wtemp;
	cpu_draw_osd(number[data],line,offset+space*3,resolution);
	// slash dot comma *17 , slash dot comma *14 , H L M
	cpu_draw_osd(specMark[spec_val],line,offset-space,resolution); 
#else
	space = 16;
	//Arial 17
	//year
	wtemp = value;
	//data = wtemp/10000;	// 減少 至 4位輸出 
	//wtemp -= data*10000;
	//cpu_draw_osd(number[data], line, offset, resolution);
	data = wtemp/1000;
	wtemp -= data*1000;
	cpu_draw_osd(number[data],line,offset,resolution);
	data = wtemp/100;
	wtemp -= data*100;
	cpu_draw_osd(number[data],line,offset+space*1,resolution);
	data = wtemp/10;
	wtemp -= data*10;
	cpu_draw_osd(number[data],line,offset+space*2,resolution);
	data = wtemp;
	cpu_draw_osd(number[data],line,offset+space*3,resolution);
	// slash dot comma *17 , slash dot comma *14 , H L M
	cpu_draw_osd(specMark[spec_val],line,offset-space,resolution); 
#endif
	
}
#endif
#endif
#endif//#if (defined APP_VIDEO_ENCODER_EN) && (APP_VIDEO_ENCDOER_EN == 1)
