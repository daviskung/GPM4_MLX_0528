/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2014 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#include "drv_l2_cdsp_config.h"
#include "drv_l2_cdsp.h"
//#include "gplib_print_string.h"

extern void sensor_set_max_lum(int max_lum);


#if (defined _DRV_L2_CDSP) && (_DRV_L2_CDSP == 1)

INT32S drv_l2_cdsp_set_badpixel_ob(void)
{
	gpCdspCtrl_t ctrl;
	gpCdspBadPixOB_t badpixel_ob;

	// bad pixel
	badpixel_ob.badpixen = ENABLE;
	badpixel_ob.badpixmirr = 0x3;
	badpixel_ob.bprthr = 160;
	badpixel_ob.bpgthr =160;
	badpixel_ob.bpbthr = 160;

	// OB
	// Auto OB
	badpixel_ob.autooben = DISABLE;
	badpixel_ob.obHOffset = 4;
	badpixel_ob.obVOffset = 4;
	badpixel_ob.obtype = 4;
	badpixel_ob.Ravg = 0;
	badpixel_ob.GRavg = 0;
	badpixel_ob.Bavg = 0;
	badpixel_ob.GBavg = 0;

	// WB offset
	badpixel_ob.wboffseten = ENABLE;
	badpixel_ob.boffset = 0;
	badpixel_ob.roffset = 0;
	badpixel_ob.gboffset = 0;
	badpixel_ob.groffset = 0;

	// Manu OB
	badpixel_ob.manuoben = ENABLE;
	badpixel_ob.manuob = 42;


	ctrl.id = MSG_CDSP_BADPIX_OB;
	ctrl.value = (INT32U)&badpixel_ob;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}
	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_wb_gain(void)
{
	gpCdspCtrl_t ctrl;
	gpCdspWhtBal_t wht_bal;
	gpCdspWbGain2_t wbgain2;

	// set AWB enable
	wht_bal.wbgainen = ENABLE;
	wht_bal.bgain = 100;
	wht_bal.gbgain = 1*64;
	wht_bal.rgain = 140;
	wht_bal.grgain = 1*64;
	wht_bal.global_gain = 1*32;

	ctrl.id = MSG_CDSP_WBGAIN;
	ctrl.value = (INT32U)&wht_bal;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	// set WB gain2 disable
	wbgain2.wbgain2en =ENABLE;
	wbgain2.bgain2 = 64;
	wbgain2.rgain2 = 64;
	wbgain2.ggain2 = 64;

	ctrl.id = MSG_CDSP_WBGAIN2;
	ctrl.value = (INT32U)&wbgain2;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}
	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_ae_target(INT32S ae_target, INT32S ae_target_night)
{
	gpCdspCtrl_t ctrl;

	// set Target Lum
	ctrl.id = MSG_CDSP_TARGET_AE;
	ctrl.value = (INT32U)&ae_target;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_aeawb_window(INT16U width, INT16U height, INT8U ae_meter, float *lpAE_weight_table)
{
	int t1, t2;
	gpCdspCtrl_t ctrl;
	gpCdspAWB_t awb_win;
	gpCdspAE_t ae;
	gpCdspRawWin_t raw_win;

// set RAW Win
	raw_win.aeawb_src = 1;
	raw_win.subsample = 0;	// 0:disable, 1:1/2, 2:1/4 subsample

	t1 = width;
	while(t1 > 1024)
	{
		raw_win.subsample++;
		t1 >>= 1;
	}
	if(raw_win.subsample > 2) raw_win.subsample = 2;

    raw_win.hwdoffset = 4;
    raw_win.vwdoffset = 4;
    raw_win.hwdsize = (t1 - (raw_win.hwdoffset << 1)) >> 3;
    raw_win.vwdsize = (height - (raw_win.vwdoffset << 1)) >> 3;

	raw_win.hwdsize >>= 1;
	raw_win.vwdsize >>= 1;
	raw_win.hwdoffset >>= 1;
	raw_win.vwdoffset >>= 1;

	raw_win.hwdsize &= 0xfffE;
	raw_win.vwdsize &= 0xfffE;
	raw_win.hwdoffset &= 0xfffE;
	raw_win.vwdoffset &= 0xfffE;

	raw_win.AeWinTest = 0;
	raw_win.AfWinTest = 0;

	DBG_PRINT("RAW_Win Set: subsample = %d, size[%d, %d],  offset[%d, %d]\r\n", raw_win.subsample, raw_win.hwdsize, raw_win.vwdsize, raw_win.hwdoffset, raw_win.vwdoffset);

	ctrl.id = MSG_CDSP_RAW_WIN;
	ctrl.value = (INT32U)&raw_win;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	// set AWB win
	awb_win.awb_win_en = ENABLE;
	awb_win.awb_win_hold = DISABLE;
	awb_win.awbclamp_en = ENABLE;//ENABLE;
	awb_win.sindata = 0*64;
	awb_win.cosdata = 1*64;
	awb_win.awbwinthr = 200;
	awb_win.Ythr0 = 50;
	awb_win.Ythr1 = 100;
	awb_win.Ythr2 = 150;
	awb_win.Ythr3 = 200;

	awb_win.UL1N1 = -30;
	awb_win.UL1P1 = 30;
	awb_win.VL1N1 = -30;
	awb_win.VL1P1 = 30;

	awb_win.UL1N2 = -50;
	awb_win.UL1P2 = 50;
	awb_win.VL1N2 = -50;
	awb_win.VL1P2 = 50;

	awb_win.UL1N3 = -100;
	awb_win.UL1P3 = 100;
	awb_win.VL1N3 = -100;
	awb_win.VL1P3 = 100;

	ctrl.id = MSG_CDSP_AWB_WIN;
	ctrl.value = (INT32U)&awb_win;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	// set AE
	ae.ae_win_en = ENABLE;
	ae.ae_win_hold = 0;
	ae.ae_meter = ae_meter;

    if(lpAE_weight_table != NULL)
    {
       /* int i;

        for(i = 0; i < 64; i++)
            ae.ae_user_weight_table[i] = lpAE_weight_table[i];*/

       ae.ae_user_weight_table = lpAE_weight_table  ;
    }
    else
    {
        ae.ae_user_weight_table = NULL;
    }

	ctrl.id = MSG_CDSP_AE_WIN;
	ctrl.value = (INT32U)&ae;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}



	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_gamma(void)
{
	gpCdspCtrl_t ctrl;
	gpCdspGamma_t lut_gamma;

	// set gamma
	lut_gamma.lut_gamma_en = ENABLE;
	ctrl.id = MSG_CDSP_LUT_GAMMA;
	ctrl.value = (INT32U)&lut_gamma;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}
	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_wb_offset(INT32S wb_offset_day, INT32S wb_offset_night)
{
	gpCdspCtrl_t ctrl;
	int offset;

	offset = wb_offset_day;
	ctrl.id = MSG_CDSP_WB_OFFSET_DAY;
	ctrl.value = (INT32U)&offset;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	offset = wb_offset_night;
	ctrl.id = MSG_CDSP_WB_OFFSET_NIGHT;
	ctrl.value = (INT32U)&offset;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_intp(INT32U hi_thr, INT32U low_thr)
{
	gpCdspCtrl_t ctrl;
	gpCdspIntpl_t intpl;

	intpl.rawspecmode = MODE_NORMAL;
	intpl.int_hi_thr = hi_thr;
	intpl.int_low_thr = low_thr;

	ctrl.id = MSG_CDSP_INTERPOLATION;
	ctrl.value = (INT32U)&intpl;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_colormatrix(void)
{
	gpCdspCtrl_t ctrl;
	gpCdspCorMatrix_t ColorMatrix;

	ColorMatrix.colcorren = ENABLE;
	ctrl.id = MSG_CDSP_COLOR_MATRIX;
	ctrl.value = (INT32U)&ColorMatrix;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_histgm(void)
{
	gpCdspCtrl_t ctrl;
	gpCdspHistgm_t histgm;

	histgm.his_en = ENABLE;
	histgm.his_hold_en = DISABLE;

	ctrl.id = MSG_CDSP_HISTGM;
	ctrl.value = (INT32U)&histgm;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}


INT32S drv_l2_cdsp_set_sharpen(INT16U width, INT8U sharpen_s)
{
	gpCdspCtrl_t ctrl;
	IspSharpenData_t sharpen;

    sharpen.sharpen_en = ENABLE;
    sharpen.uvf_en = ENABLE;
    sharpen.mask = 1;
    sharpen.reduce = 0;
    sharpen.sharpen_S = sharpen_s;//24;
    sharpen.sharpen_Th = 127;
    sharpen.clip1 = 4;
    sharpen.clip2 = 120;
    sharpen.width = width;

	ctrl.id = MSG_CDSP_SHARPEN;
	ctrl.value = (INT32U)&sharpen;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_denoise(void)
{
    gpCdspCtrl_t ctrl;
	IspHbNrData_t denoise;

    denoise.NrEn = DISABLE;
    denoise.AjEn = DISABLE;

    denoise.LumaFilter = 1;
    denoise.ChromaFilter = 1;
    denoise.Clip = 0;
    denoise.EdgeThr = 16;
    denoise.AjEdgeThr = 16;

	ctrl.id = MSG_CDSP_DENOISE;
	ctrl.value = (INT32U)&denoise;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}



INT32S drv_l2_cdsp_set_wdr(void)
{
    gpCdspCtrl_t ctrl;
	IspWDRData_t wdr;

    wdr.wdr_en = ENABLE;
    wdr.uv_enhance_en = ENABLE;
    wdr.method = 1; // mirror
    wdr.win_bg = 2; // 3x9
    wdr.win_i = 1; // 3x3
    wdr.chroma = 128;

	ctrl.id = MSG_CDSP_WDR;
	ctrl.value = (INT32U)&wdr;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}


INT32S drv_l2_cdsp_set_post_lut(void)
{
    gpCdspCtrl_t ctrl;
    int postlut_en = ENABLE;

    ctrl.id = MSG_CDSP_POSTLUT;
	ctrl.value = (INT32U)&postlut_en;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_hybrid(INT16U sensor_w, INT16U sensor_h)
{
    gpCdspCtrl_t ctrl;
    gpIspHybridRaw_t hybrid;

    hybrid.ISPWidth = sensor_w;
    hybrid.IspHeight = sensor_h;

    hybrid.CrosstalkEnable = ENABLE;
    hybrid.CrosstalkGbGr = 3;
    hybrid.CrosstalkT1 = 8;
    hybrid.CrosstalkT2 = 16;
    hybrid.CrosstalkT3 = 32;
    hybrid.CrosstalkT4 = 64;
    hybrid.CrosstalkW1 = 3;
    hybrid.CrosstalkW2 = 4;
    hybrid.CrosstalkW3 = 5;

    hybrid.DefectPixelEnable = ENABLE;
    hybrid.DefectPixelSel = 0;
    hybrid.DPCDefaultMode = 1;
    hybrid.DPCn = 1;
    hybrid.DPCth1 = 55;
    hybrid.DPCth2 = 76;
    hybrid.DPCth3 = 4;

    hybrid.DenoiseEnable = DISABLE;
    hybrid.DenoiseT1 = 4;
    hybrid.DenoiseT2 = 8;
    hybrid.DenoiseT3 = 16;
    hybrid.DenoiseT4 = 32;
    hybrid.DenoiseW1 = 1;
    hybrid.DenoiseW2 = 2;
    hybrid.DenoiseW3 = 3;

	ctrl.id = MSG_CDSP_HYBRID;
	ctrl.value = (INT32U)&hybrid;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}


INT32S drv_l2_cdsp_set_suppression(void)
{
	gpCdspCtrl_t ctrl;
	gpCdspSuppression_t suppression;

	// set suppression
	suppression.suppressen = DISABLE;
	suppression.suppr_mode = 0;
	suppression.denoisen = DISABLE;
	suppression.lowyen = DISABLE;
	suppression.denoisethrl = 0x0;
	suppression.denoisethrwth= 0x0;
	suppression.yhtdiv= 0x0;
	ctrl.id = MSG_CDSP_SUPPRESSION;
	ctrl.value = (INT32U)&suppression;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_edge(INT32S sharpness)
{
	gpCdspCtrl_t ctrl;
	gpCdspEdge_t edge;

	edge.edgeen = ENABLE;
	edge.eluten = ENABLE;
	edge.edge_table = 0;
	edge.ampga = sharpness;
	edge.edgedomain = 0x0;
	edge.lhmode = 0;
	edge.lhdiv = 2;
	edge.lhtdiv = 2;
	edge.lhcoring = 4;
	edge.Qcnt = 0;
	edge.Qthr = 255;
	edge.lf00 = 0x9;
	edge.lf01 = 0;
	edge.lf02 = 0x9;
	edge.lf10 = 0;
	edge.lf11 = 0x4;
	edge.lf12 = 0;
	edge.lf20 = 0x9;
	edge.lf21 = 0;
	edge.lf22 = 0x9;

	ctrl.id = MSG_CDSP_EDGE;
	ctrl.value = (INT32U)&edge;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}


INT32S drv_l2_cdsp_set_saturation_day(INT32S y_offset, INT32S u_offset, INT32S v_offset, INT32S y_scale, INT32S u_scale, INT32S v_scale)
{
	gpCdspCtrl_t ctrl;
	gpCdspSatHue_t sat_hue;

	// set  Saturation/Constrast enhancement
	sat_hue.YbYcEn = ENABLE;
	sat_hue.u_huecosdata = 0x40;
	sat_hue.u_huesindata = 0x00;
	sat_hue.v_huecosdata = 0x40;
	sat_hue.v_huesindata = 0x00;

	// daylight
	sat_hue.y_offset = y_offset; // -128 ~ +127
	sat_hue.u_offset = u_offset; // -128 ~ +127,   +: more blue,  -: more yellow/green
	sat_hue.v_offset = v_offset; //  -128 ~ +127,   +: more red,  -: more blue/green

	sat_hue.y_scale = y_scale; // contrast
	sat_hue.u_scale = u_scale; // blud
	sat_hue.v_scale = v_scale; // red

	ctrl.id = MSG_CDSP_SAT_HUE_DAY;
	ctrl.value = (INT32U)&sat_hue;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_saturation_night(INT32S y_offset, INT32S u_offset, INT32S v_offset, INT32S y_scale, INT32S u_scale, INT32S v_scale)
{
	gpCdspCtrl_t ctrl;
	gpCdspSatHue_t sat_hue;

	// set  Saturation/Constrast enhancement
	sat_hue.YbYcEn = ENABLE;
	sat_hue.u_huecosdata = 0x40;
	sat_hue.u_huesindata = 0x00;
	sat_hue.v_huecosdata = 0x40;
	sat_hue.v_huesindata = 0x00;

	// daylight
	sat_hue.y_offset = y_offset; // -128 ~ +127
	sat_hue.u_offset = u_offset; // -128 ~ +127,   +: more blue,  -: more yellow/green
	sat_hue.v_offset = v_offset; //  -128 ~ +127,   +: more red,  -: more blue/green

	sat_hue.y_scale = y_scale; // contrast
	sat_hue.u_scale = u_scale; // blud
	sat_hue.v_scale = v_scale; // red

	ctrl.id = MSG_CDSP_SAT_HUE_NIGHT;
	ctrl.value = (INT32U)&sat_hue;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_backlight_dt(INT32U en)
{
	gpCdspCtrl_t ctrl;
	int val = en;

	ctrl.id = MSG_CDSP_BACKLIGHT_DETECT;
	ctrl.value = (INT32U)&val;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_awb_mode(INT32U mode)
{
	gpCdspCtrl_t ctrl;
	AWB_MODE wbmode = mode;

	ctrl.id = MSG_CDSP_WB_MODE;
	ctrl.value = (INT32U)&wbmode;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_ev_val(INT32S ev)
{
	gpCdspCtrl_t ctrl;
	int ev_val = ev;

	ctrl.id = MSG_CDSP_EV;
	ctrl.value = (INT32U)&ev_val;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_raw_special_mode(INT32U mode)
{
	gpCdspCtrl_t ctrl;

	ctrl.id = MSG_CDSP_RAW_SPEF;
	ctrl.value = (INT32U)&mode;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_iso(INT32U iso)
{
	gpCdspCtrl_t ctrl;
	int iso_val;

	switch(iso)
	{
		case 100:
			iso_val = 1;
			break;
		case 200:
			iso_val = 2;
			break;
		case 400:
			iso_val = 4;
			break;
		case 800:
			iso_val = 8;
			break;
		default:
			iso_val = ISO_AUTO;
			break;
	}

	ctrl.id = MSG_CDSP_ISO;
	ctrl.value = (INT32U)&iso_val;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_yuv_scale(INT16U dst_w, INT16U dst_h)
{
	gpCdspScalePara_t cdspScale;
	gpCdspCtrl_t ctrl;

	cdspScale.hscale_en = DISABLE;
	cdspScale.hscale_mode = 1;
	cdspScale.dst_hsize = dst_w;

	cdspScale.crop_en = DISABLE;
	cdspScale.crop_hoffset = 0; //Disable H/V crop, offset set 0
	cdspScale.crop_voffset = 2;
	cdspScale.crop_hsize = dst_w;
	cdspScale.crop_vsize = dst_h+2;

	cdspScale.yuvhscale_en = ENABLE;
	cdspScale.yuvvscale_en = ENABLE;
	cdspScale.yuvhscale_mode = 1; // filter
	cdspScale.yuvvscale_mode = 1; // filter
	cdspScale.yuv_dst_hsize = dst_w;
	cdspScale.yuv_dst_vsize = dst_h;

	cdspScale.img_rb_h_size = dst_w;
	cdspScale.img_rb_v_size = dst_h;

	ctrl.id = MSG_CDSP_SCALE_CROP;
	ctrl.value = (INT32U)&cdspScale;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32S drv_l2_cdsp_set_linear_correction(void)
{
	gpCdspLinCorr_t cdspLincorr;
	gpCdspCtrl_t ctrl;

	cdspLincorr.lincorren = ENABLE;

	ctrl.id = MSG_CDSP_LINCORR;
	ctrl.value = (INT32U)&cdspLincorr;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

INT32U drv_l2_cdsp_set_lens_compensation(void)
{
	gpCdspLensCmp_t cdspLensCmp;
	gpCdspCtrl_t ctrl;

	cdspLensCmp.lcen = ENABLE;

	ctrl.id = MSG_CDSP_LENS_CMP;
	ctrl.value = (INT32U)&cdspLensCmp;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}


int drv_l2_cdsp_set_edge_ampga(int edge_day, int edge_night)
{
	gpCdspCtrl_t ctrl;

	ctrl.id = MSG_CDSP_EDGE_AMPGA_DAY;
	ctrl.value = (INT32U)&edge_day;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return -1;
	}


	ctrl.id = MSG_CDSP_EDGE_AMPGA_NIGHT;
	ctrl.value = (INT32U)&edge_night;
	if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
		DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
		return -1;
	}

	return 0;
}

INT32S drv_l2_cdsp_set_cv_para(INT8U *lpCV_para)
{
        gpCdspCtrl_t ctrl;

        ctrl.id = MSG_CDSP_CV_PARA;
        ctrl.value = (INT32U)lpCV_para;

        if(drv_l2_cdsp_set_ctrl(&ctrl) < 0) {
            DBG_PRINT("VIDIOC_S_CTRL fail!!!\r\n");
            return STATUS_FAIL;
        }

    return 0;
}


void drv_l2_cdsp_enable(gpUserFav_t *pFavTable, INT16U sensor_w, INT16U sensor_h, INT16U target_w, INT16U target_h)
{
	DBG_PRINT("%s\r\n", __func__);

#if 1
    drv_l2_cdsp_set_badpixel_ob();

	drv_l2_cdsp_set_hybrid(sensor_w, sensor_h);

	drv_l2_cdsp_set_ae_target(pFavTable->ae_target, pFavTable->ae_target_night);
	drv_l2_cdsp_set_aeawb_window(sensor_w, sensor_h, pFavTable->ae_meter, pFavTable->lpAE_weight_table_f);

	drv_l2_cdsp_set_gamma();

	drv_l2_cdsp_set_wb_gain();

	//drv_l2_cdsp_set_wb_offset(0, 0);

	drv_l2_cdsp_set_intp(240, 32);

    //#if ( (USE_CV_Prefer == 0) || ((USE_CV_Prefer == 1) && (CV_OPEN_COLORMATRIX == 1)) )
	drv_l2_cdsp_set_colormatrix();
    //#endif

	drv_l2_cdsp_set_post_lut();

	drv_l2_cdsp_set_histgm();

	drv_l2_cdsp_set_sharpen(target_w, pFavTable->day_edge);
	drv_l2_cdsp_set_denoise();

    #if ( (USE_CV_Prefer == 0) || ((USE_CV_Prefer == 1) && (CV_OPEN_WDR == 1)) )
	drv_l2_cdsp_set_wdr();
    #endif

	drv_l2_cdsp_set_suppression();
	drv_l2_cdsp_set_edge(0);

	drv_l2_cdsp_set_edge_ampga(pFavTable->day_edge, pFavTable->night_edge);

	drv_l2_cdsp_set_saturation_day(pFavTable->day_hue_y_offset, pFavTable->day_hue_u_offset, pFavTable->day_hue_v_offset, pFavTable->day_sat_y_scale, pFavTable->day_sat_u_scale, pFavTable->day_sat_v_scale);		//hue & satuaration & contrast calibration for day
	drv_l2_cdsp_set_saturation_night(pFavTable->night_hue_y_offset, pFavTable->night_hue_u_offset, pFavTable->night_hue_v_offset, pFavTable->night_sat_y_scale, pFavTable->night_sat_u_scale, pFavTable->night_sat_v_scale);		//hue & satuaration & contrast calibration for day


	sensor_set_max_lum(pFavTable->max_lum);

    #if( USE_CV_Prefer == 1 )
    drv_l2_cdsp_set_cv_para(pFavTable->lpCV_table);
    #endif

#endif

	drv_l2_cdsp_set_backlight_dt(ENABLE);

	drv_l2_cdsp_set_awb_mode(pFavTable->awb_mode);

	//drv_l2_cdsp_set_ev_val(pFavTable->ev);

	//drv_l2_cdsp_raw_special_mode(g_pFavTable->color_mode);

	//drv_l2_cdsp_set_iso(pFavTable->iso);


	//drv_l2_cdsp_set_linear_correction();

	drv_l2_cdsp_set_lens_compensation();

	DBG_PRINT("cdsp set enable finish\r\n");
}

#endif


