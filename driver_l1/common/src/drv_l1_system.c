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
#include "drv_l1.h"
#include "drv_l1_sfr.h"
#include "drv_l1_system.h"

void drv_l1_system_arbiter_init(void)
{
    R_BUS_ARB_PSCA        = 0x04;
    R_BUS_ARB_PSCA_B      = 0x04;
    R_BUS_ARB_H264        = 0x81;
    R_BUS_ARB_JPG         = 0x80;
    R_BUS_ARB_PSCA_MB_B   = 0x04;
    R_BUS_ARB_DMA         = 0x80;
    R_BUS_ARB_ARM         = 0x88;
    R_BUS_ARB_FD          = 0x82;
    R_BUS_ARB_PPUPPU      = 0x80;
    //R_BUS_ARB_USB20       = 0x01;
    R_SYSTEM_MISC_CTRL5  &= 0xFFFFF0DF; //bit 5 & bit 8~11 h264 encode lock
	R_BUS_ARB_HDMI = 0x2;	// 2019.11.12 ming said
}
