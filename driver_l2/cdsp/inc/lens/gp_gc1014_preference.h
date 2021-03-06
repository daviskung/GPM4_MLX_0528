#ifndef __GP_GC1014_PREFERENCE_H__
#define __GP_GC1014_PREFERENCE_H__

#include "..\gp_LensDef.h"


static const gpUserFav_t g_FavTable =
{
	.ae_target = 110,
	.ae_target_night = 20,

	.ae_meter = CDSP_AE_METER_AVG_T2,
	.awb_mode = AWB_AUTO_FIRST_SEARCH,
	.color_mode = 0,
	.iso = 0,

	.ev = 0,

	.day_hue_y_offset = -2, //-128 ~ +127
	.day_hue_u_offset = 0,	//-128 ~ +127,   +: more blue,  -: more yellow/green
	.day_hue_v_offset = 0,	//-128 ~ +127,   +: more red,  -: more blue/green

	.day_sat_y_scale = 0x21, //Default:0x20
	.day_sat_u_scale = 0x22, //Default:0x20 	// blud
	.day_sat_v_scale = 0x22, //Default:0x20	// red

	.day_edge = 10, /*	0(smooth) ~ 127(sharp)*/

	.night_hue_y_offset = 0,
	.night_hue_u_offset = 0, //-128 ~ +127,   +: more blue,  -: more yellow/green
	.night_hue_v_offset = 0, //-128 ~ +127,   +: more red,  -: more blue/green

	.night_sat_y_scale = 0x20,	  //Default:0x20
	.night_sat_u_scale = 0x20,//Default:0x20, blue
	.night_sat_v_scale = 0x20,//Default:0x20, red

	.night_edge = 6,

	.max_lum = (64 - 4)

};


#endif

