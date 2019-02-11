/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2014 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#ifndef __drv_l1_H264SCALER_H__
#define __drv_l1_H264SCALER_H__

/****************************************************
*		include file								*
****************************************************/
#include "drv_l1.h"

/****************************************************
*	Definition	 									*
****************************************************/
#define H264SCALER_OK			0
#define H264SCALER_FAIL			-1

#define H264_SCALE_DIV2		(0<<1)
#define H264_SCALE_DIV4		(1<<1)

#define H264_SCALE_DROP		(0<<2)
#define H264_SCALE_AVG			(1<<2)

#define H264_SCALE_3BUF		(0<<3)
#define H264_SCALE_2BUF		(1<<3)

/****************************************************
*		Data structure 								*
*****************************************************/

/****************************************************
*		external function declarations				*
****************************************************/

extern INT32S drv_l1_h264scaler_init(INT32U width, INT32U height, INT32U mode);
extern INT32S drv_l1_h264scaler_bufA_set(INT32U y_addr, INT32U uv_addr);
extern INT32S drv_l1_h264scaler_bufB_set(INT32U y_addr, INT32U uv_addr);
extern INT32S drv_l1_h264scaler_bufC_set(INT32U y_addr, INT32U uv_addr);
extern INT32S drv_l1_h264scaler_en_set(BOOLEAN status);

#endif	/* __drv_l1_H264SCALER_H__ */
