#ifndef __GPLIB_JPEG_H__
#define __GPLIB_JPEG_H__


#include "drv_l1.h"

#define GPLIB_JPEG_ENCODE_EN            _DRV_L1_JPEG  
#define GPLIB_JPEG_DECODE_EN            _DRV_L1_JPEG

extern INT32U gplib_jpeg_version_get(void);
extern void gplib_jpeg_default_quantization_table_load(INT32U quality);
extern void default_huffman_dc_lum_table_load(void);

// JPEG load default huffman table.
extern void gplib_jpeg_default_huffman_table_load(void);
extern const INT8U zigzag_scan[64] ;

#endif 		// __GPLIB_JPEG_H__