#ifndef __GP_LENSDEF_H__
#define __GP_LENSDEF_H__

#include "gp_aeawb.h"

typedef struct gpUserFav_s
{
	unsigned char ae_target;			/* target luminace */
	unsigned char ae_target_night;		/* target luminace in night */

    unsigned short iso;

    unsigned char ae_meter;
    float         *lpAE_weight_table_f;//when ae_meter = CDSP_AE_METER_USER_WEIGHT, use this table! table size = 64; table type = float
	unsigned char awb_mode;
	unsigned char color_mode;

	signed char   ev;

	signed char	day_hue_y_offset;	/* -128 ~ +127	*/
	signed char	day_hue_u_offset;	/* -128 ~ +127,   +: more blue,  -: more yellow/green	*/
	signed char	day_hue_v_offset;	/* -128 ~ +127,   +: more red,  -: more blue/green */

	unsigned char	day_sat_y_scale;
	unsigned char	day_sat_u_scale;
	unsigned char	day_sat_v_scale;

	unsigned char	day_edge;			/*	0:soft, 1:smooth, 2:normal, 3:strong */

	//signed char	day_wb_offset;		/*	+: warm,   -: cold , +10~-10 */

	signed char	night_hue_y_offset;	/* -128 ~ +127	*/
	signed char	night_hue_u_offset;	/* -128 ~ +127,   +: more blue,  -: more yellow/green	*/
	signed char	night_hue_v_offset;	/* -128 ~ +127,   +: more red,  -: more blue/green */

	unsigned char	night_sat_y_scale;
	unsigned char	night_sat_u_scale;
	unsigned char	night_sat_v_scale;

	unsigned char	night_edge;			/*	0:soft, 1:smooth, 2:normal, 3:strong */

	//signed char	night_wb_offset;		/*	+: warm,   -: cold , +10~-10 */


	unsigned char   max_lum; // 0 ~ 64
	unsigned char   *lpCV_table;//computer vision use table
} gpUserFav_t;





typedef struct gpCsiCali_s
{
	unsigned short *ob;
	unsigned char  *linearity;
	unsigned short *radius0;
	unsigned short *radius1;
	unsigned short *clpoint;
	unsigned short *maxtan;
	unsigned short *slope;
	unsigned short step;
	unsigned short segR;
	unsigned int   *gamma;
	signed short   *awb_thr;
	unsigned short (*wb_gain)[2];
	signed short   *color_matrix;
} gpCisCali_t;



#endif
