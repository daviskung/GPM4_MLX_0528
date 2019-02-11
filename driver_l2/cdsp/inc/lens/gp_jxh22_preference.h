#ifndef __GP_JXH42_PREFERENCE_H__
#define __GP_JXH42_PREFERENCE_H__

#include "..\gp_LensDef.h"


const gpUserFav_t g_FavTable =
{
	.ae_target = 50,
	.ae_target_night = 20,

	.awb_mode = 0,
	.color_mode = 0,
	.iso = 0,

	.ev = 0,

	.day_hue_y_offset = -4, //-128 ~ +127
	.day_hue_u_offset = 0,	//-128 ~ +127,   +: more blue,  -: more yellow/green
	.day_hue_v_offset = 0,	//-128 ~ +127,   +: more red,  -: more blue/green

	.day_sat_y_scale = 0x21, //Default:0x20
	.day_sat_u_scale = 0x28, //Default:0x20 	// blud
	.day_sat_v_scale = 0x28, //Default:0x20	// red

	.day_edge = 2, /*	0:soft, 1:smooth, 2:normal, 3:strong */

	.night_hue_y_offset = 0,
	.night_hue_u_offset = 0, //-128 ~ +127,   +: more blue,  -: more yellow/green
	.night_hue_v_offset = 0, //-128 ~ +127,   +: more red,  -: more blue/green

	.night_sat_y_scale = 0x20,	  //Default:0x20
	.night_sat_u_scale = 0x20,//Default:0x20, blue
	.night_sat_v_scale = 0x20,//Default:0x20, red

	.night_edge = 2,

	.max_lum = (64 - 10)  //

};


#endif

