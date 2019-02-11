#ifndef __DRIVER_L2_CFG_H__
#define __DRIVER_L2_CFG_H__

//=========================Engineer Mode ("Not" defined CUSTOMER_DEFINE)=====================//
#ifndef CUSTOMER_DEFINE  //For Engineer Development and test                                 //
//===========================================================================================//

    #define DRV_L2_ON                   1
    #define DRV_L2_OFF                  0
    #define _DRV_L2_SYS_TIMER_EN        DRV_L2_OFF
    #define _DRV_L2_KEYSCAN             DRV_L2_ON
    #define _DRV_L2_EXT_RTC             DRV_L2_OFF
    #define _DRV_L2_ITOUCH              DRV_L2_OFF
    #define _DRV_L2_USBH                DRV_L2_OFF
    #define _DRV_L2_USBH_UVC            DRV_L2_OFF
    //#define _DRV_L2_SPI                 DRV_L2_ON // williamyeo, roy move to _DRV_L1_SPI
    #define _DRV_L2_SDIO                DRV_L2_OFF
    #define _DRV_L2_CDSP				DRV_L2_ON
    #define _DRV_L2_CSI				    DRV_L2_ON
    #define _DRV_L2_SDC                 DRV_L2_ON

    #define _DRV_L2_IR                  DRV_L2_OFF
    #define _DRV_L2_NAND                DRV_L2_ON
    #define _DRV_L2_FFT                 DRV_L2_ON
//======================== Engineer Mode END ================================================//
#else                              //Engineer Difne area END                                 //
//===========================================================================================//

//=========================Customer Mode (Defined CUSTOMER_DEFINE)===========================//
//                    //For Release to customer, follow the command of "customer.h"          //
//======================== Not Modify in this Area ==========================================//


//========================= CUSTOMER DEFINE END =============================================//
#endif  // not CUSTOMER_DEFINE                                                               //
//========================= CUSTOMER DEFINE END =============================================//

#define FS_CACHE_FAT_EN				0 // when 0, disable caching FAT code and cache buffer

#endif		// __DRIVER_L2_CFG_H__
