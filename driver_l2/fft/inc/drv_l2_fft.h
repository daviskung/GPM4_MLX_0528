#ifndef __drv_l2_FFT_H__
#define __drv_l2_FFT_H__

typedef struct FFT_PARAM_s
{
	INT32U				nfft;			//fft points
	INT32U				dir;			//fft direction
	INT32U				scale_stage;	//scale stage(0~5)
	INT32U				scale_div;		//scale div(2 or 4)
	FFT_IN_FMT_ENUM		in_fmt;			//fft input format
	FFT_OUT_FMT_ENUM	out_fmt;		//fft output format
	INT32U				*inbuf;			//fft h/w input buffer addr
	INT32U				*outbuf;		//fft h/w output buffer addr
	INT32U				ovf_handl_ena;	//fft overflow handling enable
	INT32U				ovf_sts;		//fft overflow state
	INT32U				ovf_handl_cnt;	//fft overflow cnt for an identical sequence
	void 				(*fft_cbk)(void);
}FFT_PARAM;

enum
{
	FFT_SET_POINT_ERR=1,
	FFT_SET_DIR_ERR,
	FFT_SET_IN_MODE_ERR,
	FFT_SET_OUT_MODE_ERR,
	FFT_SET_SCALE_ERR,
	FFT_TIMEOUT
};

extern INT32S drv_l2_fft_trigger(FFT_PARAM *pFftParam);
#endif

