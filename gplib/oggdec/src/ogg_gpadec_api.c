#include "ogg_gpadec_api.h"
#include "gp_acodec.h"
#include "assert.h"
#include "gplib.h"

#ifdef WIN32
	#define AFMT_S16_LE	0x00000010
#else
//	#include <mach/audio/soundcard.h>
#endif
#define AFMT_S16_LE	0x00000010			//add 151123

#ifdef _DEBUG
	#include "stdio.h"
	#define DEBUG_ONLY(x)	x;
#else
	#define DEBUG_ONLY(x)	;
#endif

#define RETURN(x) {ret = x; goto Return;}

typedef enum
{
	STATE_PARSE = 0,
	STATE_RUN
} STATE;

typedef struct
{
	char Kernel[OGGVORBIS_DEC_MEMORY_SIZE];
	STATE state;
} OGG_WORK_MEM;

unsigned int acodec_oggdec_instance_size(void)
{
//[DBG]
//#include "vorbis_core.h"
//	int i;
//i=sizeof(vorbis_t);
//[DBG]
	// check if version error
	if(OGGVORBIS_DEC_MEMORY_SIZE != oggvorbis_dec_get_mem_block_size()) return 0;
	return sizeof(OGG_WORK_MEM);
}

int acodec_oggdec_init(audec_param_t *adp)
{
	int ret;
	OGG_WORK_MEM *WorkMem = (OGG_WORK_MEM *)adp->audec_buf;

	ret = oggvorbis_dec_init(WorkMem->Kernel, adp->Ring, adp->RingSize, 0);	//add 120822, stereo problem
//	ret = oggvorbis_dec_init(WorkMem->Kernel, adp->Ring, adp->RingSize);	//mark 120822, stereo problem
	if(ret<0)
	{
		DEBUG_ONLY(printf("[oggdec] Error ID = 0x%08x\n", ret));
		RETURN(ret);
	}
	adp->RingRI = adp->RingWI = oggvorbis_dec_get_ri(WorkMem->Kernel);
	WorkMem->state = STATE_PARSE;
	ret = 0;

Return:
	return ret;
}

int acodec_oggdec_dec(audec_param_t *adp, unsigned char *buf, unsigned int *out_size)
{
	int ret;
	OGG_WORK_MEM *WorkMem = (OGG_WORK_MEM *)adp->audec_buf;
	int Eat = 0, t;

	assert(WorkMem);

//<<add 160104
	if (adp->FileCnt >= adp->FileLen)
		return -1;
//add 160104>>

	*out_size = 0;
	Eat = adp->RingRI = oggvorbis_dec_get_ri(WorkMem->Kernel);

	if(WorkMem->state == STATE_PARSE)
	{
		ret = oggvorbis_dec_parsing(WorkMem->Kernel, adp->RingWI);

		if(ret<0)
		{
			DEBUG_ONLY(printf("[oggdec] Error ID = 0x%08x\n", ret));
			RETURN(ret);
		}
		if(ret==OGGVORBIS_PARSING_OK)
		{
			WorkMem->state = STATE_RUN;
		    adp->out_fmt = AFMT_S16_LE;
		    adp->out_sample_rate = oggvorbis_dec_get_samplerate(WorkMem->Kernel);
    		adp->out_channels = oggvorbis_dec_get_channel(WorkMem->Kernel);
			adp->out_bit_rate = oggvorbis_dec_get_bitrate(WorkMem->Kernel);
			DEBUG_ONLY(printf("[oggdec] parser OK\n"));
		}
		adp->RingRI = oggvorbis_dec_get_ri(WorkMem->Kernel);

		t = adp->RingRI - Eat;
		if(t < 0) t += adp->RingSize;
		DEBUG_ONLY(printf("[oggdec] parser Eat = %d\n", t));
	}

	if(WorkMem->state==STATE_RUN)
	{
		ret = oggvorbis_dec_run(WorkMem->Kernel, (short*)buf, adp->RingWI);

//[modify 120719]
/*		if(adp->out_channels == 1){
			short *buf_wt_ptr = (short*)buf;
			short *buf_rd_ptr = (short*)buf;
			int i;
			for(i=0;i<ret;i++){
				*buf_wt_ptr = *buf_rd_ptr;
				buf_rd_ptr += 2;
				buf_wt_ptr += 1;
			}
		}
*/
//[modify 120719]

		if(ret<0)
		{
			DEBUG_ONLY(printf("[oggdec] Error ID = 0x%08x\n", ret));
			RETURN(ret);
		}

		DEBUG_ONLY(printf("[oggdec] ret = %d\n", ret));

	    adp->out_fmt = 0;
	    adp->out_sample_rate = oggvorbis_dec_get_samplerate(WorkMem->Kernel);
		adp->out_channels = oggvorbis_dec_get_channel(WorkMem->Kernel);
		adp->out_bit_rate = oggvorbis_dec_get_bitrate(WorkMem->Kernel);

		//*out_size = ret * adp->out_channels;			//modify 120719
		*out_size = ret * adp->out_channels * 2;		//(ori)
	}

	adp->RingRI = oggvorbis_dec_get_ri(WorkMem->Kernel);
//<<add 160104
	adp->FileCnt += (adp->RingRI - Eat);

	if (Eat > adp->RingRI)
		adp->FileCnt += OGGVORBIS_DEC_BITSTREAM_BUFFER_SIZE;
//add 160104>>
	Eat = adp->RingRI - Eat;
	if(Eat < 0) Eat += adp->RingSize;

	ret = Eat;

Return:
	return ret;
}

int acodec_oggdec_uninit(audec_param_t *adp)
{
	return 0;
}
