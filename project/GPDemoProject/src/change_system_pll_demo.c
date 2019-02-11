
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

#include <stdio.h>
#include <string.h>
#include "project.h"
#include "typedef.h"
#include "drv_l1_cfg.h"
#include "drv_l1_sfr.h"
#include "drv_l2_power_save.h"

extern void Iram_copy(void);

#define C_X12M          12    //change C12M from 12M Xtal
#define C_387MHz        387000000
#define C_SYS_32KHz     32768
#define C_SYS_MIN_CLK   843750

/************************************************************************
*
* CHANGE SYSTEM PLL: Please check all module are off before change pll
* Please reserve 0x1fff6000 for system pll change c file
************************************************************************/

void GPM4_Change_System_Pll_Demo(void)
{
    INT32U system_pll = C_387MHz;  //system pll = 108MHz

        vTaskSuspendAll();
        portENTER_CRITICAL();
        Iram_copy();
        drv_l2_change_system_speed(C_X12M);
        portEXIT_CRITICAL();
        xTaskResumeAll();
    while(1)
    {
        DBG_PRINT("HEAP SIZE = %d ; system pll = %d \r\n", xPortGetFreeHeapSize(), system_pll);
        osDelay(100);
        vTaskSuspendAll();
        portENTER_CRITICAL();
        Iram_copy();
        drv_l2_change_system_speed(system_pll);
        portEXIT_CRITICAL();
        xTaskResumeAll();
        if(system_pll<9000000)
        {
            system_pll = system_pll<<1;
            if(system_pll>9000000)
                system_pll = 9000000;
        }
        else{
            system_pll+=9000000;
            }
        if(system_pll> 387000000)
            system_pll = C_SYS_MIN_CLK;


        osDelay(1000);

    }



}
