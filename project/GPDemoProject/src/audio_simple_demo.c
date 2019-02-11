#include <stdio.h>
#include "application.h"
#include "drv_l1_clock.h"
#include "drv_l2_ad_key_scan.h"
#include "audio_decoder.h"
#include "audio_encoder.h"

extern MSG_Q_ID AudioTaskQ;
extern osMessageQId AudioTaskQ2;
extern xQueueHandle Mbox_MP3_Play_Seek_Flag, Mbox_MP3_Play_Seek_Offset;    //mp3 seek
extern SACM_CTRL G_SACM_Ctrl;
extern INT16S   gEncode_Filehandle;
#define SEEK_STEP       5//in sec

#define VOLUME_KEY      0
#define SEEK_KEY        1
#define KEY_OPTION      VOLUME_KEY
#define NAND_MIDI  0

INT32S spu_malloc(INT32U size)
{
    if(size)
        return (INT32S)gp_malloc_align(size, 4);
    else
        return 0;
}

INT32S spu_free(void *ptr)
{
    gp_free(ptr);

    return 0;
}


void audio_decode_simple_demo(void)
{
	INT32S ret, key,index, pll;
	struct f_info file_info;
	MEDIA_SOURCE src;
	CODEC_START_STATUS status;
	AUDIO_ARGUMENT audio_arg;
	AUDIO_CODEC_STATUS audio_status;
	AUDIO_INFO info;
    MEDIA_SOURCE	media_source_0;
    INT32U temp;
    INT32U msg_id;
    INT32S total_time = 0;
    INT32U seek_time = 0;
    CHAR path[60];
    drv_l1_clock_set_system_clk_en(CLK_EN0_SPU,1);
    drv_l1_clock_set_system_clk_en(CLK_EN0_HDMI,1);
    drv_l1_clock_set_system_clk_en(CLK_EN0_TFT,1);
    drv_l1_clock_set_system_clk_en(CLK_EN1_DISPAY_OUT,1);
#if NAND_MIDI == 1
    ret = NAND_APP_Initial();
    DBG_PRINT("NAND_APP_Initial=%d\r\n",ret);
#endif
	// Mount device
	while(1) {
		if(_devicemount(FS_SD1)) {
			DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD1);
		} else {
			DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD1);
			break;
		}
	}

	chdir("F:\\");
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
    pll = 378000000;
	audio_decode_entrance();

	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("*                        This is Audio decode demo                     *\r\n");
	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("SPU      : Main_Channel = 0, AudioFormat = WAV(or support format)\r\n");
	DBG_PRINT("DAC      : Main_Channel = 1, AudioFormat = WAV(or support format)\r\n");
	DBG_PRINT("MIDI(SPU): Main_Channel = 2, AudioFormat = MIDI\r\n");
	DBG_PRINT("HDMI(DAC): Main_Channel = 3, AudioFormat = WAV(or support format)\r\n");
	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("KEY1    Select file\r\n");
	DBG_PRINT("KEY2    Play\r\n");
	DBG_PRINT("KEY3    Stop\r\n");
	DBG_PRINT("KEY4    Pause/ Resume\r\n");
	DBG_PRINT("KEY5    Mute/ unMute\r\n");
	DBG_PRINT("KEY6    Volume up\r\n");
	DBG_PRINT("KEY7    Volume down\r\n");
	DBG_PRINT("KEY8    Exit\r\n");
	DBG_PRINT("\r\n");
	DBG_PRINT("\r\n");
    index = 1;
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
			media_source_0.Format.AudioFormat = MP3;
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
                ret = SystemCoreClock*2/27000000 -1;
                R_SYSTEM_HDMI_CTRL = 0x80 | ret;
                R_SYSTEM_CLK_CTRL = temp;
			}
			else if(audio_arg.Main_Channel == 2)
			{
                temp = R_SYSTEM_CLK_CTRL;
                temp &= ~0x8;
                ret = SystemCoreClock*2/27000000 -1;
                R_SYSTEM_HDMI_CTRL = 0x80 | ret;
                R_SYSTEM_CLK_CTRL = temp;
				media_source_0.type_ID.memptr = 0x0 ;
                spu_user_malloc_set(spu_malloc, spu_free);

                SPU_Initial();
			}

			audio_arg.L_R_Channel = 3;		// Left + Right Channel
			audio_arg.mute = 0;
			audio_arg.volume = 10;			// volume level = 0~63
			audio_arg.midi_index = 0;
            audio_arg.data_offset = 0;
            G_snd_info.Speed = 12;					//for speed control(golbal var)
            G_snd_info.Pitch = 12;					//for pitch control(golbal var)
#if 0
            G_SACM_Ctrl.Offset = 10;
            G_SACM_Ctrl.Offsettype = SND_OFFSET_TYPE_TIME;
            G_SACM_Ctrl.AudioFormat = WAV;
            media_source_0.type = SOURCE_TYPE_USER_DEFINE;
#endif

#if 0
            if(index > 0)
            {
                sprintf((char *)path, (const char *)"test_mswav_%03d.wav", index--);
                gEncode_Filehandle = fs_open(path, O_CREAT|O_TRUNC|O_WRONLY);
                if(gEncode_Filehandle < 0)
                {
                    DBG_PRINT("fs_open fail\r\n");
                    while(1);
                }
            }
#endif

            total_time = audio_decode_GetTotalTime(media_source_0.type_ID.FileHandle,media_source_0.Format.AudioFormat);
            DBG_PRINT("total_time = %d sec \r\n",total_time);
            close(media_source_0.type_ID.FileHandle);

            media_source_0.type_ID.FileHandle = fs_open((char *)file_info.f_name, O_RDONLY);
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
			audio_status = audio_decode_status(audio_arg);
			if(audio_status == AUDIO_CODEC_PROCESSING) {
				audio_decode_pause(audio_arg);
			} else if(audio_status == AUDIO_CODEC_PROCESS_PAUSED) {
				audio_decode_resume(audio_arg);
			}
		}
		else if(ADKEY_IO5 || ADKEY_IO5_C) {
            if(audio_arg.mute == 0)
            {
                DBG_PRINT("audio mute\r\n");
                audio_decode_mute(&audio_arg);
            }
            else
            {
                DBG_PRINT("audio Un-mute\r\n");
                audio_decode_unmute(&audio_arg);
            }
		}
		else if(ADKEY_IO6 || ADKEY_IO6_C) {
#if KEY_OPTION == VOLUME_KEY
            audio_arg.volume++;
			if(audio_arg.volume > 63) {
				audio_arg.volume = 0;
			}

			DBG_PRINT("Volunme = %d\r\n", audio_arg.volume);
			audio_decode_volume_set(&audio_arg, audio_arg.volume);
#elif KEY_OPTION == SEEK_KEY
            if(media_source_0.Format.AudioFormat == MP3)
            {
                seek_time += SEEK_STEP;// 5sec a step
                if(seek_time >= total_time)
                    seek_time = 0;
                DBG_PRINT("seek_time = %d\r\n",seek_time);
                G_SACM_Ctrl.Offsettype =SND_OFFSET_TYPE_TIME;
                msg_id = 1;//audio_codec_seek_enable;
                osMessagePut(Mbox_MP3_Play_Seek_Flag, (INT32U)&msg_id, 0);
                msg_id = seek_time*1000;//audio_codec_seek_time;
                osMessagePut(Mbox_MP3_Play_Seek_Offset, (INT32U)&msg_id, 0);
            }
#endif
		}
		else if(ADKEY_IO7 || ADKEY_IO7_C) {
#if KEY_OPTION == VOLUME_KEY
			if(audio_arg.volume > 0)
                audio_arg.volume--;
			else
                audio_arg.volume = 63;

			DBG_PRINT("Volunme = %d\r\n", audio_arg.volume);
			audio_decode_volume_set(&audio_arg, audio_arg.volume);
#elif KEY_OPTION == SEEK_KEY
            if(media_source_0.Format.AudioFormat == MP3)
            {
                seek_time -= SEEK_STEP;
                if(seek_time <= 0)
                    seek_time = 0;
                DBG_PRINT("seek_time = %d\r\n",seek_time);
                G_SACM_Ctrl.Offsettype =SND_OFFSET_TYPE_TIME;
                msg_id = 1;//audio_codec_seek_enable;
                osMessagePut(Mbox_MP3_Play_Seek_Flag, (INT32U)&msg_id, 0);
                msg_id = seek_time*1000;//audio_codec_seek_time;
                osMessagePut(Mbox_MP3_Play_Seek_Offset, (INT32U)&msg_id, 0);
            }
#endif
		}
		else if(ADKEY_IO8) {
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

	// Mount device
	while(1) {
		if(_devicemount(FS_SD1)) {
			DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD1);
		} else {
			DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD1);
			break;
		}
	}

	chdir("F:\\");
	audio_encode_entrance();

	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("*                        This is Audio record demo                     *\r\n");
	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("KEY1    Start/Stop record (.WAV)\r\n");
	DBG_PRINT("KEY2    Start/Stop record (.WAV)(MS_ADPCM)\r\n");
	DBG_PRINT("KEY3    Start/Stop record (.WAV)(IMA_ADPCM)\r\n");
	DBG_PRINT("KEY4    Start/Stop record (.A18)\r\n");
    DBG_PRINT("KEY6    Start/Stop record (.WAV)\r\n");
	DBG_PRINT("KEY8    Exit\r\n");
	DBG_PRINT("\r\n");
	DBG_PRINT("\r\n");

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

				audio_status = audio_encode_start(media_source_0, BUILD_IN_MIC , 44100, 0);
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
				DBG_PRINT("IMA_ADPCM\r\n");
				media_source_0.type = SOURCE_TYPE_FS;
                media_source_0.Format.AudioFormat = IMA_ADPCM;
				sprintf((char *)file_name, (const char *)"test_imawav_%03d.wav", index++);
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
		else if(ADKEY_IO4) {
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
		else if(ADKEY_IO5) {
		/* //lib not ready
			if(audio_encode_status() == AUDIO_CODEC_PROCESSING) {
				DBG_PRINT("Stop\r\n");
				audio_encode_stop();
			} else {
				DBG_PRINT("MP3\r\n");
				media_source_0.type = SOURCE_TYPE_FS;
                media_source_0.Format.AudioFormat = MP3;
				sprintf((char *)file_name, (const char *)"test_mp3_%03d.mp3", index++);
				DBG_PRINT("FileName = %s\r\n", file_name);
				media_source_0.type_ID.FileHandle = fs_open(file_name, O_CREAT|O_TRUNC|O_WRONLY);
				if(media_source_0.type_ID.FileHandle < 0) {
					continue;
				}

				audio_status = audio_encode_start(media_source_0, BUILD_IN_MIC, 44100, 128000);
				if(audio_status != START_OK) {
					continue;
				}
			}
        */
		}
		else if(ADKEY_IO6) {
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

				audio_status = audio_envelop_detect_start(media_source_0, BUILD_IN_MIC , 44100, 0);
				if(audio_status != START_OK) {
                    DBG_PRINT("start failed\r\n");
					continue;
				}
			}
		}
		else if(ADKEY_IO7) {
		}
		else if(ADKEY_IO8) {
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
