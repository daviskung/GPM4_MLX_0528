#ifndef __GP_JXH42_PREFERENCE_H__
#define __GP_JXH42_PREFERENCE_H__

#include "..\gp_LensDef.h"


static const gpUserFav_t g_FavTable =
{
	.ae_target = 94,
	.ae_target_night = 94,

	.ae_meter = CDSP_AE_METER_CENTER_WEIGHTED_AVG_CVR,
	.awb_mode = AWB_AUTO_FIRST_SEARCH,
	.color_mode = 0,
	.iso = 0,

	.ev = 0,

	.day_hue_y_offset = 0, //-128 ~ +127
	.day_hue_u_offset = 0, //-128 ~ +127,   +: more blue,  -: more yellow/green
	.day_hue_v_offset = 0,	//-128 ~ +127,   +: more red,  -: more blue/green

	.day_sat_y_scale = 0x28, //Default:0x20
	.day_sat_u_scale = 0x40, //Default:0x20 	// blud
	.day_sat_v_scale = 0x40, //Default:0x20	// red

	.day_edge = 23, /*	0(smooth) ~ 127(sharp)*/

	.night_hue_y_offset = 0,
	.night_hue_u_offset = 0, //chen //-128 ~ +127,   +: more blue,  -: more yellow/green
	.night_hue_v_offset = 0, //-128 ~ +127,   +: more red,  -: more blue/green

	.night_sat_y_scale = 0x28,	  //Default:0x20
	.night_sat_u_scale = 0x40,//Default:0x20, blue
	.night_sat_v_scale = 0x40,//Default:0x20, red

	.night_edge = 20,

	.max_lum = (64 - 4)
};


#endif

