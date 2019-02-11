#ifndef __DEFINE_H__
#define __DEFINE_H__

#include "project.h"

#define IMG_FMT_GRAY	        0
#define IMG_FMT_YUYV	        1
#define IMG_FMT_YCbYCr	        2
#define IMG_FMT_RGB		        3
#define IMG_FMT_UYVY	        4
#define IMG_FMT_CbYCrY	        5
#define IMG_FMT_INTEGRAL_Y      6
#define IMG_FMT_YUV444          7

#define MAX(x, y) ((x) >= (y) ? (x) : (y))
#define MIN(x, y) ((x) <  (y) ? (x) : (y))
#define ABS(x)	  ((x) >= 0 ? (x) : -(x))

typedef struct
{
	INT16S width;
	INT16S height;
    INT16S widthStep;
	INT8S ch;
	INT8S format;
	INT8U *ptr;
	INT8U *ptr_u;
	INT8U *ptr_v;
} gpImage;

typedef struct
{
    INT16S x;
    INT16S y;
    INT16S width;
    INT16S height;
} gpRect;

typedef struct
{
    INT16S width;
    INT16S height;
} gpSize;

typedef struct
{
	INT16S x;
	INT16S y;
} gpPoint;

typedef struct {
	gpPoint	left_pt;
	gpPoint right_pt;
	gpPoint top_pt;
	gpPoint bottom_pt;
} gpFeaturePoint;

typedef struct {
	gpPoint	left_pt;
	gpPoint right_pt;
	gpPoint rtop_pt;
	gpPoint ltop_pt;
	gpPoint rbottom_pt;
	gpPoint lbottom_pt;
} gpFeaturePoint2;

typedef struct DataArr
{

    unsigned short width;
    unsigned short height;
    unsigned short step;
    unsigned int *i;

} DataArr;

#endif // __DEFINE_H__
