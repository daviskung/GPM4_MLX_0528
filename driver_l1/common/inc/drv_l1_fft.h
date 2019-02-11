#ifndef __drv_l1_FFT_H_
#define __drv_l1_FFT_H_

#include "drv_l1.h"

#define OBJ_FUNCTION_EN                 1
#if OBJ_FUNCTION_EN == 1
// start register
#define C_FFT_OBJ_START		            0x00000001
#define C_FFT_OBJ_RESET		            0x00000002
// control register
#define C_FFT_OBJ_ENABLE		        0x00000100
#define C_OBJ_ENABLE		            0x00000200
#define C_OBJ_OUTPUTDISABLE		        0x00000400
#define C_OBJ_INT_ENABLE		        0x00008000
// OBJ Scale Register
#define C_OBJ_SCALE_MASK		        0x00000FFF
// INT Status register
#define C_OBJ_INT_HIT_STATE		        0x00000002
#define C_FFT_OBJ_INT_STATE		        0x00000001
#define C_FFT_OBJ_INT_STATE_MASK		0x00000003
// OBJ Threshold register
#define C_OBJ_THRESHOLD_MASK		    0x0003FFFF
// OBJ Minimum Error Register
#define C_OBJ_MIN_MASK		            0x0003FFFF
// OBJ Minimum Error ID Register
#define C_OBJ_MIN_ID_MASK		        0x00000FFF
#endif

#define SET_MODE_FFT			0x00000000
#define SET_MODE_IFFT			0x00000008

#define SET_FFT_32P			    0x00000000
#define SET_FFT_64P			    0x00000001
#define SET_FFT_128P			0x00000002
#define SET_FFT_256P			0x00000003
#define SET_FFT_512P			0x00000004
#define SET_FFT_1024P			0x00000005
#define SET_FFT_2048P			0x00000006

#define SET_SCL_STG_0_12		0x00000001
#define SET_SCL_STG_0_14		0x00000002
#define SET_SCL_STG_1_12		0x00000004
#define SET_SCL_STG_1_14		0x00000008
#define SET_SCL_STG_2_12		0x00000010
#define SET_SCL_STG_2_14		0x00000020
#define SET_SCL_STG_3_12		0x00000040
#define SET_SCL_STG_3_14		0x00000080
#define SET_SCL_STG_4_12		0x00000100
#define SET_SCL_STG_4_14		0x00000200
#define SET_SCL_STG_5_12		0x00000400
#define SET_SCL_STG_5_14		0x00000800

#define SET_FFT_INT_ENA			0x00008000
#define SET_FFT_HW_ENA			0x00000100	//enable FFT module
#define SET_FFT_TRIG			0x00000001	//FFT start
#define SET_FFT_RST			    0x00000002	//FFT reset

#define SET_16b_FFTIN_R_AND_I	0x00000000
#define SET_16b_FFTIN_R			0x00000010
#define SET_24b_FFTIN_R_AND_I	0x00000020
#define SET_24b_FFTIN_R			0x00000030


#define SET_16b_FFTOUT_R_AND_I	0x00000000
#define SET_16b_FFTOUT_R		0x00000040
#define SET_24b_FFTOUT_R_AND_I	0x00000080
#define SET_24b_FFTOUT_R		0x000000C0

#define SET_DIR_FFT			    0x00000000
#define SET_DIR_IFFT			0x00000008

#define C_DIR_FFT               0
#define C_DIR_IFFT              1


typedef enum{
    C_IN_R_16b=1,		//input 16-bit real only
    C_IN_R_I_16b,		//input 16-bit real and image
    C_IN_R_24b,			//input 24-bit real only
    C_IN_R_I_24b		//input 24-bit real and image
}FFT_IN_FMT_ENUM;

typedef enum{
    C_OUT_R_16b=1,		//output 16-bit real only
    C_OUT_R_I_16b,		//output 16-bit real and image
    C_OUT_R_24b,		//output 24-bit real only
    C_OUT_R_I_24b		//output 24-bit real and image
}FFT_OUT_FMT_ENUM;

extern void drv_l1_fft_init(void);
extern void drv_l1_fft_callback_set( void(*fft_cbk)(void));
extern INT32S drv_l1_fft_set_points(INT32U FFT_N);
extern INT32S drv_l1_set_fft_dir(INT32U inv);
extern INT32S drv_l1_fft_set_scl(INT32U stage, INT32U div);
extern void drv_l1_fft_in_addr(INT32U *addr);
extern void drv_l1_fft_out_addr(INT32U *addr);
extern INT32S drv_l1_fft_set_in_fmt(FFT_IN_FMT_ENUM in_fmt);
extern INT32S drv_l1_fft_set_out_fmt(FFT_OUT_FMT_ENUM out_fmt);
extern void drv_l1_fft_start(void);
extern void drv_l1_fft_obj_lock(void);
extern void drv_l1_fft_obj_unlock(void);
extern void drv_l1_fft_uninit(void);
extern INT32S drv_l1_fft_wait_done(INT32U timeout);
extern INT32U drv_l1_get_fft_ovf_sts(void);
#if OBJ_FUNCTION_EN == 1
extern INT32S drv_l1_obj_start(void);
extern INT32S drv_l1_obj_scale_set(INT32U unit);
extern INT32S drv_l1_obj_database_addr_set(INT32U addr);
extern INT32S drv_l1_obj_output_buffer_addr_set(INT32U addr);
extern INT32S drv_l1_obj_compare_buffer_addr_set(INT32U addr);
extern INT32S drv_l1_obj_threshold_set(INT32U value);
extern INT32S drv_l1_obj_minimum_error_get(void);
extern INT32S drv_l1_obj_minimum_error_id_get(void);
extern INT32S drv_l1_obj_end(INT8U wait);
extern INT32S drv_l1_obj_init(void);
extern INT32U drv_l1_obj_malloc(INT32U size);
extern INT32S drv_l1_obj_user_malloc_set(INT32U (*malloc_function)(INT32U size));
#endif

#endif // __drv_l1_FFT_H_
