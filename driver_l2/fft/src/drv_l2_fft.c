#include "drv_l1_fft.h"
#if (defined _DRV_L2_FFT) && (_DRV_L2_FFT == 1)

#include "drv_l2_fft.h"

void drv_l2_fft_ovf_handling(FFT_PARAM *pFftParam)
{
	INT32U ovf_sts = pFftParam->ovf_sts;
	INT32U stage=0, div=0;

	if(ovf_sts & 0x01){
		stage = 0;
	}else if(ovf_sts & 0x02){
		stage = 1;
	}else if(ovf_sts & 0x04){
		stage = 2;
	}else if(ovf_sts & 0x08){
		stage = 3;
	}else if(ovf_sts & 0x10){
		stage = 4;
	}else if(ovf_sts & 0x20){
		stage = 5;
	}
	div = (pFftParam->ovf_handl_cnt+1)<<1;

	drv_l1_fft_set_scl(stage, div);

}

INT32S drv_l2_fft_trigger(FFT_PARAM *pFftParam)
{
	INT32S	ret;

	drv_l1_fft_obj_lock();
	drv_l1_fft_init();

	if(pFftParam->fft_cbk)
		drv_l1_fft_callback_set(pFftParam->fft_cbk);

	/*** set fft point ***/
    ret = drv_l1_fft_set_points(pFftParam->nfft);
    if(ret < 0){
        DBG_PRINT("fft set points err!");
        ret = FFT_SET_POINT_ERR;
        goto _EndFftTrig;
    }
	/*** set fft direction ***/
    ret = drv_l1_set_fft_dir(pFftParam->dir);
    if(ret < 0){
        DBG_PRINT("fft set dir err!");
        ret = FFT_SET_DIR_ERR;
        goto _EndFftTrig;
    }
	/*** set fft scale stage and div(optional) ***/
	if(pFftParam->scale_div){
		ret = drv_l1_fft_set_scl(pFftParam->scale_stage,pFftParam->scale_div);
		if(ret < 0){
			DBG_PRINT("fft set dir err!");
			ret = FFT_SET_SCALE_ERR;
			goto _EndFftTrig;
		}
	}

	/*** set fft input data format ***/
    ret = drv_l1_fft_set_in_fmt(pFftParam->in_fmt);
    if(ret < 0){
        DBG_PRINT("fft set in mode err!");
        ret = FFT_SET_IN_MODE_ERR;
        goto _EndFftTrig;
    }

	/*** set fft output data format ***/
    ret = drv_l1_fft_set_out_fmt(pFftParam->out_fmt);
	if(ret < 0){
        DBG_PRINT("fft set out mode err!");
        ret = FFT_SET_OUT_MODE_ERR;
        goto _EndFftTrig;
    }

	/*** set fft input data buffer start pointer ***/
	drv_l1_fft_in_addr(pFftParam->inbuf);

	/*** set fft output data buffer start pointer ***/
	drv_l1_fft_out_addr(pFftParam->outbuf);
	cache_invalid_range(pFftParam->outbuf, pFftParam->nfft*2*sizeof(INT32S));
	pFftParam->ovf_handl_cnt = 0;
	do{
	/*** start to run fft ***/
		if(pFftParam->ovf_sts){
			drv_l2_fft_ovf_handling(pFftParam);
			pFftParam->ovf_handl_cnt++;
		}
	drv_l1_fft_start();

	ret = drv_l1_fft_wait_done(0);
	if(ret<0)
	{
		ret = FFT_TIMEOUT;
	}
		if(pFftParam->ovf_handl_cnt >= 2)
			break;
	}while(pFftParam->ovf_sts);


_EndFftTrig:
	drv_l1_fft_uninit();
	drv_l1_fft_obj_unlock();

	return ret;
}
#endif //(defined _DRV_L2_FFT) && (_DRV_L2_FFT == 1)
