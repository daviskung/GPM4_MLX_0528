#ifndef __DRV_L2_USER_PREFERENCE_H__
#define __DRV_L2_USER_PREFERENCE_H__

///////////////////////////////////////////////////
// Define Lens
///////////////////////////////////////////////////
#define LENS_GP_Prefer         0
#define LENS_JX_H42_Prefer     1
#define LENS_JX_H22_Prefer     2
#define LENS_GC1004_Prefer     3
#define LENS_GC1014_Prefer     4
#define LENS_SP5506_Prefer     5
#define LENS_GC5025_Prefer     6

#define USE_CV_Prefer           0

#if(USE_CV_Prefer == 1)

#define CV_UBT_IQ   1

#define CV_IQ_Prefer       CV_UBT_IQ


#if(CV_IQ_Prefer == CV_UBT_IQ)
//#define CV_OPEN_COLORMATRIX                 (1)
#define CV_IS_COLORMATRIX_GAMMA_ORDER       (1)
#define CV_OPEN_WDR                         (0)
//#define CV_USE_PRE_GAMMA                    (0)
//#define CV_USE_LINEARCORRECT                (0)
//1: L type diagonal 60 degress; 2: L type hor 60 degress
#define SENSOR_TYPE                         (2)
#endif//#if(CV_IQ_Prefer == CV_UBT_IQ)


#endif

///////////////////////////////////////////////////
// select lens
///////////////////////////////////////////////////
#define LENS_CALI_Prefer       LENS_JX_H42_Prefer

#if (defined _SENSOR_H62_CDSP_MIPI) && (_SENSOR_H62_CDSP_MIPI == 1)

#if (USE_CV_Prefer == 0)
#include "lens/gp_jxh62_preference.h"
#else
#include "lens/gp_jxh62_CV_preference.h"
#endif


#elif (defined _SENSOR_GC1064_CDSP_MIPI) && (_SENSOR_GC1064_CDSP_MIPI ==1)
#if (USE_CV_Prefer == 0)
#include "lens/gp_gc1064_preference.h"
#else
#include "lens/gp_gc1064_CV_preference.h"
#endif
#else

#if (LENS_CALI_Prefer == LENS_GP_Prefer)
#include "lens/gp_preference.h"
#endif

#if (LENS_CALI_Prefer == LENS_JX_H42_Prefer)
#if (USE_CV_Prefer == 0)
#include "lens/gp_jxh42_preference.h"
#else
#include "lens/gp_jxh42_CV_preference.h"
#endif


#endif

#if (LENS_CALI_Prefer == LENS_JX_H22_Prefer)
#include "lens/gp_jxh22_preference.h"
#endif

#if (LENS_CALI_Prefer == LENS_GC1004_Prefer)
#include "lens/gp_gc1004_preference.h"
#endif

#if (LENS_CALI_Prefer == LENS_GC1014_Prefer)
#include "lens/gp_gc1014_preference.h"
#endif

#if (LENS_CALI_Prefer == LENS_SP5506_Prefer)
#include "lens/gp_sp5506_preference.h"
#endif

#if (LENS_CALI_Prefer == LENS_GC5025_Prefer)
#include "lens/gp_gc5025_preference.h"
#endif



#endif

#endif
