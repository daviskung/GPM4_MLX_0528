#include <stdio.h>
#include "application.h"
#include "drv_l1_clock.h"
#include "drv_l2_ad_key_scan.h"
#include "audio_decoder.h"
#include "audio_encoder.h"

extern MSG_Q_ID AudioTaskQ;
extern osMessageQId AudioTaskQ2;

INT32S spu_malloc(INT32U size)
{
    return (INT32S)gp_malloc_align(size, 4);
}

INT32S spu_free(void *ptr)
{
    gp_free(ptr);

    return 0;
}


void audio_decode_simple_demo(void)
{
	INT32S ret, key;
	struct f_info file_info;
	MEDIA_SOURCE src;
	CODEC_START_STATUS status;
	AUDIO_ARGUMENT audio_arg;
	AUDIO_CODEC_STATUS audio_status;
	AUDIO_INFO info;
    MEDIA_SOURCE	media_source_0;

    INT32U temp;

    drv_l1_clock_set_system_clk_en(CLK_EN0_SPU,1);
    drv_l1_clock_set_system_clk_en(CLK_EN0_HDMI,1);
    drv_l1_clock_set_system_clk_en(CLK_EN0_TFT,1);
    drv_l1_clock_set_system_clk_en(CLK_EN1_DISPAY_OUT,1);

	// Mount device
	while(1) {
		if(_devicemount(FS_SD)) {
			DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD);
		} else {
			DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD);
			break;
		}
	}

	chdir("C:\\");

	ret = _findfirst("*.*", &file_info, D_ALL);
	if(ret < 0) {
		DBG_PRINT("_findfirst fail...\r\n");
		goto __exit;
	}

	DBG_PRINT("File Name = %s\r\n", file_info.f_name);

    //init varaible
	audio_arg.Main_Channel = 1;
	media_source_0.type = SOURCE_TYPE_FS;
	status = AUDIO_CODEC_PROCESS_END;

	//adpcm_index = 0;
	//volume = 0;
	//fd_adpcm = -1;
	file_info.f_size = 0;
	ret = 0;

	audio_decode_entrance();

	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("*                        This is Audio decode demo                     *\r\n");
	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("KEY1    Select file\r\n");
	DBG_PRINT("KEY2    Play\r\n");
	DBG_PRINT("KEY3    Stop\r\n");
	DBG_PRINT("KEY4    Exit\r\n");
	DBG_PRINT("\r\n");
	DBG_PRINT("\r\n");

	adc_key_scan_init();
	while(1)
	{
		adc_key_scan();

		if(ADKEY_IO1) {
			ret = _findnext(&file_info);
			if(ret < 0) {
				ret = _findfirst("*.*", &file_info, D_ALL);
				if(ret < 0) {
					DBG_PRINT("_findfirst fail...\r\n");
					goto __exit;
				}
			}

			DBG_PRINT("File Name = %s\r\n", file_info.f_name);
		}
		else if(ADKEY_IO2) {
			audio_status = audio_decode_status(audio_arg);
			if(audio_status != AUDIO_CODEC_PROCESS_END) {
				audio_decode_stop(audio_arg);
				close(media_source_0.type_ID.FileHandle);
				media_source_0.type_ID.FileHandle = -1;
			}

			media_source_0.type = SOURCE_TYPE_FS;
			media_source_0.Format.AudioFormat = WAV;
			//media_source_0.Format.AudioFormat = MIDI;;
			media_source_0.type_ID.FileHandle = fs_open((char *)file_info.f_name, O_RDONLY);
			if(media_source_0.type_ID.FileHandle < 0) {
				continue;
			}
            // Main_Channel
            // 0: SPU
            // 1: DAC
            // 2: MIDI(SPU)
            // 3: HDMI(DAC)
			audio_arg.Main_Channel = 1;		// Use DAC Channel A+B
			//audio_arg.Main_Channel = 2;		// Use spu to play midi

			if(audio_arg.Main_Channel == 0)
			{
                temp = R_SYSTEM_CLK_CTRL;
                temp &= ~0x8;
                R_SYSTEM_HDMI_CTRL = 0x8D;
                R_SYSTEM_CLK_CTRL = temp;
			}
			else if(audio_arg.Main_Channel == 2)
			{
                temp = R_SYSTEM_CLK_CTRL;
                temp &= ~0x8;
                R_SYSTEM_HDMI_CTRL = 0x8D;
                R_SYSTEM_CLK_CTRL = temp;

                spu_user_malloc_set(spu_malloc, spu_free);

                SPU_Initial();
			}

			audio_arg.L_R_Channel = 3;		// Left + Right Channel
			audio_arg.mute = 0;
			audio_arg.volume = 10;			// volume level = 0~63
			audio_arg.midi_index = 0;
            G_snd_info.Speed = 12;					//for speed control(golbal var)
            G_snd_info.Pitch = 12;					//for pitch control(golbal var)

			status = audio_decode_start(audio_arg, media_source_0);
			if(status != START_OK) {
				DBG_PRINT("audio start fail !!!\r\n");
			}
		}
		else if(ADKEY_IO3) {
			audio_status = audio_decode_status(audio_arg);
			if(audio_status != AUDIO_CODEC_PROCESS_END) {
				audio_decode_stop(audio_arg);
			}

			fs_close(media_source_0.type_ID.FileHandle);
			media_source_0.type_ID.FileHandle = -1;
		}
		else if(ADKEY_IO4) {
			DBG_PRINT("audio_decode_exit\r\n");
			goto __exit;
		}
	}

__exit:
	audio_decode_stop(audio_arg);
	fs_close(src.type_ID.FileHandle);
	audio_decode_exit();
	ad_key_uninitial();
}

void audio_encode_simple_demo(void)
{
	CHAR file_name[64];
	INT32S key, index;
	MEDIA_SOURCE src;
	CODEC_START_STATUS audio_status;
    MEDIA_SOURCE	media_source_0;
    INT32S ret=0;

	// Mount device
	while(1) {
		if(_devicemount(FS_SD)) {
			DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD);
		} else {
			DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD);
			break;
		}
	}

	chdir("C:\\");

	audio_encode_entrance();

	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("*                        This is Audio record demo                     *\r\n");
	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("KEY1    Start/Stop record (.WAV)\r\n");
	DBG_PRINT("KEY2    Start/Stop record (.WAV)(MS_ADPCM)\r\n");
	DBG_PRINT("KEY3    Start/Stop record (.A18)\r\n");
	DBG_PRINT("KEY4    Exit\r\n");


	index = 0;
	adc_key_scan_init();
	while(1)
	{
		adc_key_scan();

		if(ADKEY_IO1) {
			if(audio_encode_status() == AUDIO_CODEC_PROCESSING) {
				DBG_PRINT("Stop\r\n");
				audio_encode_stop();
			} else {
				DBG_PRINT("WAV\r\n");
				media_source_0.type = SOURCE_TYPE_FS;
                media_source_0.Format.AudioFormat = WAV;
				sprintf((char *)file_name, (const char *)"test_wav_%03d.wav", index++);
				DBG_PRINT("FileName = %s\r\n", file_name);
				media_source_0.type_ID.FileHandle = fs_open(file_name, O_CREAT|O_TRUNC|O_WRONLY);
				if(media_source_0.type_ID.FileHandle < 0) {
                    DBG_PRINT("file open failed\r\n");
					continue;

				}

				audio_status = audio_encode_start(media_source_0, BUILD_IN_MIC , 16000, 0);
				if(audio_status != START_OK) {
                    DBG_PRINT("start failed\r\n");
					continue;
				}
			}
		}
		else if(ADKEY_IO2) {
			if(audio_encode_status() == AUDIO_CODEC_PROCESSING) {
				DBG_PRINT("Stop\r\n");
				audio_encode_stop();
			} else {
				DBG_PRINT("MICROSOFT_ADPCM\r\n");
				media_source_0.type = SOURCE_TYPE_FS;
                media_source_0.Format.AudioFormat = MICROSOFT_ADPCM;
				sprintf((char *)file_name, (const char *)"test_mswav_%03d.wav", index++);
				DBG_PRINT("FileName = %s\r\n", file_name);
				media_source_0.type_ID.FileHandle = fs_open(file_name, O_CREAT|O_TRUNC|O_WRONLY);
				if(media_source_0.type_ID.FileHandle < 0) {
					continue;
				}

				audio_status = audio_encode_start(media_source_0, BUILD_IN_MIC, 16000, 0);
				if(audio_status != START_OK) {
					continue;
				}
			}
		}
		else if(ADKEY_IO3) {
			if(audio_encode_status() == AUDIO_CODEC_PROCESSING) {
				DBG_PRINT("Stop\r\n");
				audio_encode_stop();
			} else {
				DBG_PRINT("A1800\r\n");
				media_source_0.type = SOURCE_TYPE_FS;
                media_source_0.Format.AudioFormat = A1800;
				sprintf((char *)file_name, (const char *)"test_a18_%03d.a18", index++);
				DBG_PRINT("FileName = %s\r\n", file_name);
				media_source_0.type_ID.FileHandle = fs_open(file_name, O_CREAT|O_TRUNC|O_WRONLY);
				if(media_source_0.type_ID.FileHandle < 0) {
					continue;
				}

				audio_status = audio_encode_start(media_source_0, BUILD_IN_MIC, 16000, 32000);
				if(audio_status != START_OK) {
					continue;
				}
			}
		}
		else if(ADKEY_IO4) {
			DBG_PRINT("audio_encode_exit\r\n");
			goto __exit;
		}
	}

__exit:
	audio_encode_stop();
	close(src.type_ID.FileHandle);
	audio_encode_exit();
	ad_key_uninitial();
}
