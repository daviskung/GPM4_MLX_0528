#include <stdio.h>
#include "application.h"
#include "drv_l1_clock.h"
#include "drv_l2_ad_key_scan.h"
#include "audio_decoder.h"
#include "audio_encoder.h"
#include "ogg_dat.h"

#define NAND_APP_OFFSET 256 //0x20000/512, must 512 alignment
#define NAND_OGG 0

extern MSG_Q_ID AudioTaskQ;
extern osMessageQId AudioTaskQ2;

void audio_decode_oggmix_demo(void)
{
	INT32S ret, key;
	struct f_info file_info;
	MEDIA_SOURCE src;
	CODEC_START_STATUS status;
	AUDIO_ARGUMENT audio_arg;
	AUDIO_CODEC_STATUS audio_status;
	AUDIO_INFO info;
    MEDIA_SOURCE	media_source_0;
	INT32U header_addr;
    INT32U temp;
	INT32U play_index;
	INT32U temp_len;
    INT32U temp_offset;
    INT32U bin_file;
    INT32U offset;
    INT32U nand_read_buffer;

    drv_l1_clock_set_system_clk_en(CLK_EN0_SPU,1);
    drv_l1_clock_set_system_clk_en(CLK_EN0_HDMI,1);
    drv_l1_clock_set_system_clk_en(CLK_EN0_TFT,1);
    drv_l1_clock_set_system_clk_en(CLK_EN1_DISPAY_OUT,1);
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

	audio_decode_entrance();

	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("*                        This is Audio decode demo                     *\r\n");
	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("KEY1    Select file\r\n");
	DBG_PRINT("KEY2    Play\r\n");
	DBG_PRINT("KEY3    Stop\r\n");
	DBG_PRINT("KEY4    Pause/ Resume\r\n");
	DBG_PRINT("KEY5    Mute/ unMute\r\n");
	DBG_PRINT("KEY6    next\r\n");
	DBG_PRINT("KEY7    Volume down\r\n");
	DBG_PRINT("KEY8    Exit\r\n");
	DBG_PRINT("\r\n");
	DBG_PRINT("\r\n");

	adc_key_scan_init();
    #if NAND_OGG == 1
	NAND_APP_Initial();
    #endif
    //nand_read_buffer = (INT32U)gp_malloc_align(4096,4);
    //DBG_PRINT("nand buf %x\r\n",nand_read_buffer);
    //NAND_APP_ReadSector(256,4,nand_read_buffer);
	play_index = 0;
	offset = 0;//0x400;
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

			#if NAND_OGG == 1
			media_source_0.type = SOURCE_TYPE_NAND;
			#else
			media_source_0.type = SOURCE_TYPE_FS_OGGMIX;
			#endif

			//media_source_0.Format.AudioFormat = WAV;
			media_source_0.Format.AudioFormat = OGG;
			//media_source_0.Format.AudioFormat = MIDI;;
            #if NAND_OGG == 1
            nand_read_buffer = (INT32U)gp_malloc_align(512,4);
            //DBG_PRINT("nand buf %x\r\n",nand_read_buffer);
            NAND_APP_ReadSector(NAND_APP_OFFSET,1,nand_read_buffer);
            bin_file = (*((volatile INT32U *) (nand_read_buffer)));
                    //DBG_PRINT("bin_file(NAND) %d\r\n",bin_file);

            if(play_index<bin_file)
            {

                //audio_arg
                //(*((volatile INT32U *) (header_addr+(play_index<<1))))
                temp_offset = (*((volatile INT32U *) (4+nand_read_buffer+(play_index*8))));
                temp_len = (*((volatile INT32U *) (8+nand_read_buffer+(play_index*8))));
                //DBG_PRINT("%d %d \r\n",play_index ,temp_len);
                //DBG_PRINT("%d \r\n",temp_offset);
                audio_arg.data_len = temp_len;
                audio_arg.data_offset = temp_offset+(NAND_APP_OFFSET*512);
            }
            else
                DBG_PRINT("wrong idx \r\n");

            #else

            media_source_0.type_ID.FileHandle = fs_open((char *)file_info.f_name, O_RDONLY);
            if(media_source_0.type_ID.FileHandle < 0) {
                continue;
            }
            lseek(media_source_0.type_ID.FileHandle,offset,SEEK_SET);

            fs_read(media_source_0.type_ID.FileHandle,(INT32U)&bin_file,4);
            DBG_PRINT("bin_file %d\r\n",bin_file);

            header_addr = (INT32U)gp_malloc_align(bin_file*8, 4);

            fs_read(media_source_0.type_ID.FileHandle,header_addr,bin_file*8);

            if(play_index<bin_file)
            {

                //audio_arg
                //(*((volatile INT32U *) (header_addr+(play_index<<1))))
                temp_offset = (*((volatile INT32U *) (header_addr+(play_index*8))));
                temp_len = (*((volatile INT32U *) (4+header_addr+(play_index*8))));
                DBG_PRINT("%d %d \r\n",play_index ,temp_len);
                DBG_PRINT("%d \r\n",temp_offset);
                audio_arg.data_len = temp_len;
                lseek(media_source_0.type_ID.FileHandle,0,SEEK_SET);
                lseek(media_source_0.type_ID.FileHandle,offset+temp_offset,SEEK_SET);
            }
            else
                DBG_PRINT("wrong idx \r\n");
            #endif
            // Main_Channel
            // 0: SPU
            // 1: DAC
            // 2: MIDI(SPU)
            // 3: HDMI(DAC)
			audio_arg.Main_Channel = 1;		// Use DAC Channel A+B

			if(audio_arg.Main_Channel == 0)
			{
                temp = R_SYSTEM_CLK_CTRL;
                temp &= ~0x8;
                R_SYSTEM_HDMI_CTRL = 0x8D;
                R_SYSTEM_CLK_CTRL = temp;
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
            play_index++;
            if(play_index>=_C_OGG_TOTAL_)
                play_index=0;

            DBG_PRINT("play_index %d\r\n",play_index);
            /*
            audio_arg.volume++;
			if(audio_arg.volume > 63) {
				audio_arg.volume = 0;
			}

			DBG_PRINT("Volunme = %d\r\n", audio_arg.volume);
			audio_decode_volume_set(&audio_arg, audio_arg.volume);
			*/
		}
		else if(ADKEY_IO7 || ADKEY_IO7_C) {

			if(audio_arg.volume > 0)
                audio_arg.volume--;
			else
                audio_arg.volume = 63;

			DBG_PRINT("Volunme = %d\r\n", audio_arg.volume);
			audio_decode_volume_set(&audio_arg, audio_arg.volume);
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
