#include "GP_HW_FFT.h"
#include "drv_l1_fft.h"
#include "drv_l1_clock.h"
#include "drv_l2_fft.h"

#if HW_FFT == 1

FFT_PARAM	fft_param={0};

void GP_HW_FFT_OVF_DET(void)
{
	fft_param.ovf_sts = drv_l1_get_fft_ovf_sts();

	if(fft_param.ovf_sts){
		//DBG_PRINT("ogg fft ovf=%x\r\n",fft_param.ovf_sts);
	}
}

void GP_HW_FFT_RUN(int *buf, int nfft)
{
	INT32U k, idx;
    INT32S *temp_buf;
	INT32U t1, t2, tPeriod;
	INT32S shift;

	temp_buf = (INT32S *)gp_malloc_align(nfft*2*sizeof(INT32S), 64);

	fft_param.nfft = nfft;
	fft_param.dir = 0;
	fft_param.scale_stage = 0;
	fft_param.scale_div   = 0;
	fft_param.fft_cbk = GP_HW_FFT_OVF_DET;
	fft_param.in_fmt = C_IN_R_I_24b;
	fft_param.out_fmt = C_OUT_R_I_24b;
	fft_param.inbuf = buf;
	fft_param.outbuf = temp_buf;
	fft_param.ovf_handl_ena = 1;

	drv_l2_fft_trigger(&fft_param);

	idx = 0;
	shift = 1+fft_param.ovf_handl_cnt;
	for (k = 0; k < nfft*2; k++) {
		buf[idx++] = temp_buf[k]<<shift;
	}

	gp_free(temp_buf);
}

#endif
