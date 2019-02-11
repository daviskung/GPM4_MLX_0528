/*****************************************************************************
 *               Copyright Generalplus Corp. All Rights Reserved.
 *
 * FileName:       application_cfg.h
 * Author:         Lichuanyue
 * Description:    Created
 * Version:        1.0
 * Function List:
 *                 Null
 * History:
 *                 1>. 2008/7/15 Created
 *****************************************************************************/

#ifndef __APPLICATION_CFG_H__
#define __APPLICATION_CFG_H__

/*
#if (defined MCU_VERSION) && (MCU_VERSION < GPL327XX)
	#define APP_G_MIDI_DECODE_EN	1
#else
	#define APP_G_MIDI_DECODE_EN	0	//fixed 0
#endif
*/


#define PALM_DEMO_EN            0
#define PATTERN_DEMO_EN         0
#define WAFD_DEMO_EN            0


#define CAM_RES_VGA             0
#define CAM_RES_720P            1
#define USBD_UVC_CAM_RES        CAM_RES_720P

#define APP_G_MIDI_DECODE_EN	1
/* ---------------------------------------------------------------------------*/
#define APP_AUDIO_BG_EN         1
#define APP_VIDEO_DECODER_EN    1
#define APP_VIDEO_ENCODER_EN    1
#define APP_IMAGE_CODEC_EN      1

//define the special effect when use video encode
#define AUDIO_SFX_HANDLE	    0
#define VIDEO_SFX_HANDLE	    0

//define audio algorithm
#define APP_WAV_CODEC_EN		1	//mudt enable when use Audio Encode and AVI Decode
#define APP_MP3_DECODE_EN		1
#define APP_A1800_DECODE_EN		1
#define APP_A1800_ENCODE_EN		1
#define APP_A6400_DECODE_EN		1
#define APP_OGG_DECODE_EN		1
#define APP_VR_ENCODE_EN		1
#define APP_AAC_DECODE_EN		1

#define APP_CONST_PITCH_EN		1
#define APP_ECHO_EN             1
#define	APP_VOICE_CHANGER_EN    1
#define APP_UP_SAMPLE_EN		1
#define	APP_DOWN_SAMPLE_EN		0
#define A1800_DOWN_SAMPLE_EN    0
#define	HW_UP_SAMPLE_EN		    1

#if APP_WAV_CODEC_EN
	#define APP_WAV_CODEC_FG_EN		1 //must enable when use Audio Encode and AVI Decode
    #define APP_WAV_CODEC_BG_EN 	1 //fixed 0
#else
	#define APP_WAV_CODEC_FG_EN		1
	#define APP_WAV_CODEC_BG_EN     0
#endif

#if APP_MP3_DECODE_EN
	#define APP_MP3_DECODE_FG_EN	1
	#define APP_MP3_DECODE_BG_EN	1
#else
	#define APP_MP3_DECODE_FG_EN	0
	#define APP_MP3_DECODE_BG_EN	0
#endif

#if APP_A1800_DECODE_EN
	#define APP_A1800_DECODE_FG_EN	1
    #define APP_A1800_DECODE_BG_EN  1 //fixed 0
#else
	#define APP_A1800_DECODE_FG_EN	0
	#define APP_A1800_DECODE_BG_EN  0
#endif

#if APP_WMA_DECODE_EN
	#define APP_WMA_DECODE_FG_EN	1
	#define APP_WMA_DECODE_BG_EN	0 //fixed 0
#else
	#define APP_WMA_DECODE_FG_EN	0
	#define APP_WMA_DECODE_BG_EN	0
#endif

#if APP_A1600_DECODE_EN
	#define APP_A1600_DECODE_FG_EN	1
	#define APP_A1600_DECODE_BG_EN	0 //fixed 0
#else
	#define APP_A1600_DECODE_FG_EN	0
	#define APP_A1600_DECODE_BG_EN	0
#endif

#if APP_A6400_DECODE_EN
	#define APP_A6400_DECODE_FG_EN	1
	#define APP_A6400_DECODE_BG_EN	0 //fixed 0
#else
	#define APP_A6400_DECODE_FG_EN	0
	#define APP_A6400_DECODE_BG_EN	0
#endif

#if APP_S880_DECODE_EN
	#define APP_S880_DECODE_FG_EN	1
	#define APP_S880_DECODE_BG_EN	0 //fixed 0
#else
	#define APP_S880_DECODE_FG_EN	0
	#define APP_S880_DECODE_BG_EN	0
#endif

#if APP_AAC_DECODE_EN
	#define APP_AAC_DECODE_FG_EN    1
	#define APP_AAC_DECODE_BG_EN	0 //fixed 0
#else
	#define APP_AAC_DECODE_FG_EN 0
	#define APP_AAC_DECODE_BG_EN 0
#endif

#if APP_OGG_DECODE_EN
	#define APP_OGG_DECODE_FG_EN    1
	#define APP_OGG_DECODE_BG_EN	0 //fixed 0
#else
	#define APP_OGG_DECODE_FG_EN 0
	#define APP_OGG_DECODE_BG_EN 0
#endif


#define APP_LPF_ENABLE				0	//low pass filter, 0; disable, 1;enable

#define APP_QRCODE_BARCODE_EN		0	//QR code and Bar code decoder, 0; disable, 1;enable

#if (defined MCU_VERSION && MCU_VERSION == GPL326XXB)
	#define APP_FACE_DETECTION_EN	0   //face detect, 0; disable, 1;enable
#else
	#define APP_FACE_DETECTION_EN	0	//fixed 0; disable
#endif




#endif		// __APPLICATION_CFG_H__
