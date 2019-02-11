#ifndef __GP_GC1064_CV_PREFERENCE_H__
#define __GP_GC1064_CV_PREFERENCE_H__

#include "..\gp_LensDef.h"

#define LOW_WEIGHT (0.05)
#define MIDDLE_WEIGHT (0.35)
#define HIGH_WEIGHT (0.9)
static float cv_demo_weighted_tbl_f[64] =
{
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
};

static float center_weighted_tbl_f[64] =
{
   0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
};

static float center_weighted_tbl_cv_f[64] =
{
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
    0.35, 0.35, 0.65, 0.65, 0.65, 0.65, 0.35, 0.35,
};
#if(CV_IQ_Prefer == CV_UBT_IQ)
static unsigned char g_cv_table[] =
{
1,//open denoise
//raw_denoise open, w1,w2,w3, yuv_denoise_open, y_mode, uv_mode, havg_y, havg_u, havg_v
0,  0,  1, 2, 0, 1, 1, 0,  0, 0, //dark_Lv1
1 , 0,  1, 2, 1, 1, 1, 0,  0, 0,//dark_Lv2
1 , 0,  1, 2, 1, 1, 2, 0,  1, 1,//dark_Lv3
1 , 0,  1, 2, 1, 2, 2, 0,  1, 1,//dark_Lv4
1 , 0,  1, 1, 1, 2, 2, 0,  1, 1,//dark_Lv5
1 , 5,               //is_open_ev_control, threshold
};

static const gpUserFav_t g_FavTable =
{
#if (CV_IS_COLORMATRIX_GAMMA_ORDER == 0)
    .ae_target = 100,
	.ae_target_night = 100,
#else

    #if (SENSOR_TYPE == 1)
	.ae_target = 47,
	.ae_target_night = 47,
    #elif (SENSOR_TYPE == 2)
    	.ae_target = 75,
        .ae_target_night = 75,
    #endif

#endif

    .lpAE_weight_table_f =cv_demo_weighted_tbl_f,
	.ae_meter = CDSP_AE_METER_USER_WEIGHT,
	.awb_mode = AWB_AUTO_FIRST_SEARCH,
	.color_mode = 0,
	.iso = 0,

	.ev = 0,

	.day_hue_y_offset = -2, //-128 ~ +127
	.day_hue_u_offset = 0,	//-128 ~ +127,   +: more blue,  -: more yellow/green
	.day_hue_v_offset = 0,	//-128 ~ +127,   +: more red,  -: more blue/green

	.day_sat_y_scale = 0x21,//0x20, //Default:0x20
	.day_sat_u_scale = 0x24,//0x24, //Default:0x20 	// blud
	.day_sat_v_scale = 0x24,//0x24, //Default:0x20	// red

	.day_edge = 7, /*	0(smooth) ~ 127(sharp)*/

	.night_hue_y_offset = 0,
	.night_hue_u_offset = 0, //-128 ~ +127,   +: more blue,  -: more yellow/green
	.night_hue_v_offset = 0, //-128 ~ +127,   +: more red,  -: more blue/green

	.night_sat_y_scale = 0x20,	  //Default:0x20
	.night_sat_u_scale = 0x24,//Default:0x20, blue
	.night_sat_v_scale = 0x24,//Default:0x20, red

	.night_edge = 5,

	.max_lum = (64 - 4),
	.lpCV_table = g_cv_table,//g_cv_table,
};
#endif//#if(CV_IQ_Prefer == CV_UBT_IQ)

#endif

