#ifndef __AUDIO_ENCODER_H__
#define __AUDIO_ENCODER_H__

#include "application.h"
#include "audio_record.h"

void audio_encode_entrance(void);
void audio_encode_exit(void);
CODEC_START_STATUS audio_encode_start(MEDIA_SOURCE src, INT32U input_type , INT16U SampleRate, INT32U BitRate);
void audio_encode_stop(void);
AUDIO_CODEC_STATUS audio_encode_status(void);
CODEC_START_STATUS audio_encode_set_downsample(INT8U bEnable, INT8U DownSampleFactor);
CODEC_START_STATUS audio_envelop_detect_start(MEDIA_SOURCE src, INT32U input_type, INT16U SampleRate, INT32U BitRate);
INT8U audio_encode_set_CutLastFrame(INT8U flag);
#endif
