#ifndef __DRV_L2_USER_CALIBRATION_H__
#define __DRV_L2_USER_CALIBRATION_H__

///////////////////////////////////////////////////
// Define Lens
///////////////////////////////////////////////////
#define LENS_GP_CALIB         0
#define LENS_JX_H42_CALIB     1
#define LENS_JX_H22_CALIB     2
#define LENS_GC1004_CALIB     3
#define LENS_GC1014_CALIB     4
#define LENS_AR0330_CALIB     5
#define LENS_SP5506_CALIB     6
#define LENS_GC5025_CALIB     7

///////////////////////////////////////////////////
// select lens
///////////////////////////////////////////////////
#define LENS_CALI_CALIB       LENS_JX_H42_CALIB

#if (defined _SENSOR_H62_CDSP_MIPI) && (_SENSOR_H62_CDSP_MIPI == 1)
#include "lens/gp_jxh62_calibration.h"
#elif (defined _SENSOR_GC1064_CDSP_MIPI) && (_SENSOR_GC1064_CDSP_MIPI ==1)
#include "lens/gp_gc1064_calibration.h"
#else

#if (LENS_CALI_CALIB == LENS_GP_CALIB)
#include "lens/gp_calibration.h"
#endif

#if (LENS_CALI_CALIB == LENS_JX_H42_CALIB)
#include "lens/gp_jxh42_calibration.h"
#endif

#if (LENS_CALI_CALIB == LENS_JX_H22_CALIB)
#include "lens/gp_jxh22_calibration.h"
#endif

#if (LENS_CALI_CALIB == LENS_GC1004_CALIB)
#include "lens/gp_gc1004_calibration.h"
#endif

#if (LENS_CALI_CALIB == LENS_GC1014_CALIB)
#include "lens/gp_gc1014_calibration.h"
#endif

#if (LENS_CALI_CALIB == LENS_AR0330_CALIB)
#include "lens/gp_ar0330_calibration.h"
#endif

#if (LENS_CALI_CALIB == LENS_SP5506_CALIB)
#include "lens/gp_sp5506_calibration.h"
#endif

#if (LENS_CALI_CALIB == LENS_GC5025_CALIB)
#include "lens/gp_gc5025_calibration.h"
#endif
#endif

#endif
