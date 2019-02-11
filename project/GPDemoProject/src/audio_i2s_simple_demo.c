#include <stdio.h>
#include "application.h"
//#include "drv_l1_power.h"
#include "drv_l2_ad_key_scan.h"
#include "audio_decoder.h"
//#include "audio_encoder.h"
#include "drv_l1_clock.h"

extern MSG_Q_ID AudioTaskQ;
extern osMessageQId AudioTaskQ2;

void aud_demo_i2s_arg_init(AUDIO_ARGUMENT   arg[], MEDIA_SOURCE media_source[])
{
    INT8U   i;
    for(i=0; i<4; i++)      //4: MAX_I2X_TX_NUM
    {
        arg[i].Main_Channel = 1;
        media_source[i].type = SOURCE_TYPE_FS;
        arg[i].i2s_channel = i;
    }


}

void audio_i2s_decode_simple_demo(void)
{
	INT32S ret, key;
	struct f_info file_info;
	MEDIA_SOURCE src;
	CODEC_START_STATUS status;
	AUDIO_ARGUMENT audio_arg[4];        //4: MAX_I2X_TX_NUM
	AUDIO_CODEC_STATUS audio_status;
	AUDIO_INFO info;
    MEDIA_SOURCE	media_source_0[4];  //4: MAX_I2X_TX_NUM

    INT32U temp;
    INT32U i2s_ch=0;

    drv_l1_clock_set_system_clk_en(CLK_EN0_SDC, 1);
    drv_l1_clock_set_system_clk_en(CLK_EN0_DAC_I2STX, 1);

    R_FUNPOS0 = R_FUNPOS0 & (~(3 << 21));       //pin mux to i2s channel-3

	// Mount device
	while(1) {
		if(_devicemount(FS_SD)) {
			DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD);
		} else {
			DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD);
			break;
		}
	}

	chdir("C:\\");                              //use sd0, pin mux sd1 to i2s channel-3
	ret = _findfirst("*.*", &file_info, D_ALL);
	if(ret < 0) {
		DBG_PRINT("_findfirst fail...\r\n");
		goto __exit;
	}

	DBG_PRINT("File Name = %s\r\n", file_info.f_name);

    //init varaible
    aud_demo_i2s_arg_init(audio_arg, media_source_0);

	file_info.f_size = 0;
	ret = 0;

	audio_i2s_decode_entrance();

	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("*                        This is Audio decode demo                     *\r\n");
	DBG_PRINT("**********************************************************************\r\n");
	DBG_PRINT("KEY1    Select file\r\n");
	DBG_PRINT("KEY2    Play\r\n");
	DBG_PRINT("KEY3    Stop\r\n");
	DBG_PRINT("KEY4    Pause/ Resume\r\n");
	DBG_PRINT("KEY5    Mute/Un-mute\r\n");
	DBG_PRINT("KEY6    Volume up\r\n");
	DBG_PRINT("KEY7    Volume down\r\n");
	DBG_PRINT("KEY8    Select I2S Channel\r\n");
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
            audio_status = audio_i2s_decode_status(audio_arg[i2s_ch]);
            if(i2s_ch >=4 || audio_status != AUDIO_CODEC_PROCESS_END){  //4: MAX_I2X_TX_NUM

				continue;
            }

			media_source_0[i2s_ch].type = SOURCE_TYPE_FS;
			media_source_0[i2s_ch].Format.AudioFormat = WAV;
			media_source_0[i2s_ch].type_ID.FileHandle = fs_open((char *)file_info.f_name, O_RDONLY);
			if(media_source_0[i2s_ch].type_ID.FileHandle < 0) {
				continue;
			}
            // Main_Channel
            // 0: SPU
            // 1: DAC
            // 2: MIDI(SPU)
            // 3: HDMI(DAC)
			audio_arg[i2s_ch].Main_Channel = 1;		// Use DAC Channel A+B

			if(audio_arg[i2s_ch].Main_Channel == 0)
			{
                temp = R_SYSTEM_CLK_CTRL;
                temp &= ~0x8;
                R_SYSTEM_HDMI_CTRL = 0x8D;
                R_SYSTEM_CLK_CTRL = temp;
			}
			else if(audio_arg[i2s_ch].Main_Channel == 2)
			{
                temp = R_SYSTEM_CLK_CTRL;
                temp &= ~0x8;
                R_SYSTEM_HDMI_CTRL = 0x8D;
                R_SYSTEM_CLK_CTRL = temp;
                SPU_Initial();
			}

			audio_arg[i2s_ch].L_R_Channel = 3;		// Left + Right Channel
			audio_arg[i2s_ch].mute = 0;
			audio_arg[i2s_ch].volume = 41;			// volume level = 0~63
			audio_arg[i2s_ch].midi_index = 0;
            G_snd_info.Speed = 12;					//for speed control(golbal var)
            G_snd_info.Pitch = 12;					//for pitch control(golbal var)

			status = audio_i2s_decode_start(audio_arg[i2s_ch], media_source_0[i2s_ch]);
			if(status != START_OK) {
				DBG_PRINT("audio start fail !!!\r\n");
			}
		}
		else if(ADKEY_IO3) {    //STOP
			audio_status = audio_i2s_decode_status(audio_arg[i2s_ch]);
			if(audio_status != AUDIO_CODEC_PROCESS_END) {
                DBG_PRINT("ch%d stop\r\n", i2s_ch);
				audio_i2s_decode_stop(audio_arg[i2s_ch]);
			}

			fs_close(media_source_0[i2s_ch].type_ID.FileHandle);
			media_source_0[i2s_ch].type_ID.FileHandle = -1;
		}
		else if(ADKEY_IO4) {    //PAUSE/RESUME
                audio_status = audio_i2s_decode_status(audio_arg[i2s_ch]);
                if(audio_status == AUDIO_CODEC_PROCESSING) {
                    DBG_PRINT("ch%d pause\r\n", i2s_ch);
                    audio_i2s_decode_pause(audio_arg[i2s_ch]);
                } else if(audio_status == AUDIO_CODEC_PROCESS_PAUSED) {
                    DBG_PRINT("ch%d resume\r\n", i2s_ch);
                    audio_i2s_decode_resume(audio_arg[i2s_ch]);
                }
		}
		else if(ADKEY_IO5 || ADKEY_IO5_C) { //MUTE/UN-MUTE
            audio_status = audio_i2s_decode_status(audio_arg[i2s_ch]);
            if(audio_status== AUDIO_CODEC_PROCESSING || audio_status == AUDIO_CODEC_PROCESS_PAUSED){
                if(audio_arg[i2s_ch].mute == 0)
                {
                    DBG_PRINT("audio mute\r\n");
                    audio_i2s_decode_mute(&audio_arg[i2s_ch]);
                }
                else
                {
                    DBG_PRINT("audio Un-mute\r\n");
                    audio_i2s_decode_unmute(&audio_arg[i2s_ch]);
                }
            }
		}
		else if(ADKEY_IO6 || ADKEY_IO6_C) {//VOLUME UP
            audio_status = audio_i2s_decode_status(audio_arg[i2s_ch]);
            if(audio_status== AUDIO_CODEC_PROCESSING || audio_status == AUDIO_CODEC_PROCESS_PAUSED){
                if(audio_arg[i2s_ch].mute == 0){
                    audio_arg[i2s_ch].volume++;
                    if(audio_arg[i2s_ch].volume > 79) {
                        audio_arg[i2s_ch].volume = 0;
                    }
                    DBG_PRINT("Volunme = %d\r\n", audio_arg[i2s_ch].volume);
                    audio_i2s_decode_volume_set(&audio_arg[i2s_ch], audio_arg[i2s_ch].volume);
                }
            }
		}
		else if(ADKEY_IO7 || ADKEY_IO7_C) {//VOLUME DOWN
            audio_status = audio_i2s_decode_status(audio_arg[i2s_ch]);
            if(audio_status== AUDIO_CODEC_PROCESSING || audio_status == AUDIO_CODEC_PROCESS_PAUSED){
                if(audio_arg[i2s_ch].mute == 0){
                    if(audio_arg[i2s_ch].volume > 0)
                        audio_arg[i2s_ch].volume--;
                    else
                        audio_arg[i2s_ch].volume = 79;

                    DBG_PRINT("Volunme = %d\r\n", audio_arg[i2s_ch].volume);
                    audio_i2s_decode_volume_set(&audio_arg[i2s_ch], audio_arg[i2s_ch].volume);
                }
            }
		}
		else if(ADKEY_IO8) {
			i2s_ch = audio_i2s_ch_select(i2s_ch);
			DBG_PRINT("ch %d is selected\r\n",i2s_ch);
		}
	}

__exit:
	audio_i2s_decode_stop(audio_arg[i2s_ch]);
	fs_close(src.type_ID.FileHandle);
	audio_decode_exit();
	ad_key_uninitial();
}
