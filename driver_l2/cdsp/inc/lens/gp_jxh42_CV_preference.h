#ifndef __GP_JXH42_CV_PREFERENCE_H__
#define __GP_JXH42_CV_PREFERENCE_H__

#include "..\gp_LensDef.h"

#define LOW_WEIGHT (0.1)
#define MIDDLE_WEIGHT (0.35)
#define HIGH_WEIGHT (0.7)
static float cv_demo_weighted_tbl_f[64] =
{
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,
    HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,
    HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,HIGH_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
    LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,LOW_WEIGHT,
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

static unsigned char g_cv_table[] =
{
1,//open denoise
//raw_denoise open, w1,w2,w3, yuv_denoise_open, y_mode, uv_mode, havg_y, havg_u, havg_v
1 , 0, 1, 2, 1, 2, 3, 0,  0, 0, //dark_Lv1
1 , 0, 1, 2, 1, 3, 3, 1,  2, 2,//dark_Lv2
1 , 0, 1, 2, 1, 3, 3, 1,  2, 2,//dark_Lv3
1 , 0, 1, 2, 1, 3, 3, 1,  2, 2,//dark_Lv4
1 , 0, 1, 1, 1, 3, 3, 1,  2, 2,//dark_Lv5
0 , 5,               //is_open_ev_control, threshold
};

static const gpUserFav_t g_FavTable =
{
	.ae_target = 70,
	.ae_target_night = 70,
    .lpAE_weight_table_f =center_weighted_tbl_f,
	.ae_meter = CDSP_AE_METER_USER_WEIGHT,
	.awb_mode = AWB_AUTO_FIRST_SEARCH,
	.color_mode = 0,
	.iso = 0,

	.ev = 0,

	.day_hue_y_offset = 0, //-128 ~ +127
	.day_hue_u_offset = 0, //-128 ~ +127,   +: more blue,  -: more yellow/green
	.day_hue_v_offset = 0,	//-128 ~ +127,   +: more red,  -: more blue/green

	.day_sat_y_scale = 0x20, //Default:0x20
	.day_sat_u_scale = 0x30, //Default:0x20 	// blud
	.day_sat_v_scale = 0x30, //Default:0x20	// red

	.day_edge = 1, /*	0(smooth) ~ 127(sharp)*/

	.night_hue_y_offset = 0,
	.night_hue_u_offset = 0, //chen //-128 ~ +127,   +: more blue,  -: more yellow/green
	.night_hue_v_offset = 0, //-128 ~ +127,   +: more red,  -: more blue/green

	.night_sat_y_scale = 0x20,	  //Default:0x20
	.night_sat_u_scale = 0x30,//Default:0x20, blue
	.night_sat_v_scale = 0x30,//Default:0x20, red

	.night_edge = 1,

	.max_lum = (64 - 10),
	.lpCV_table = g_cv_table,//g_cv_table,
};


#endif

