#ifndef __CUSTOMER_H__
#define __CUSTOMER_H__

/* Leading Define Start */

#define CUSTOM_ON       1
#define CUSTOM_OFF      0

#define CUSTOMER_H_ENABLE CUSTOM_OFF  /* decision the customer define mode on/off */

//#if CUSTOMER_H_ENABLE == CUSTOM_ON
//#define CUSTOMER_DEFINE
//#endif

#if 1

/*=== IC Version definition choices ===*/
//---------------------------------------------------------------------------
#define	GPM41XXA                     0x4100                                    //
#define	GPM43XXA                     0x4300                                    //
#define	GPM45XXA                     0x4500                                    //
#define	GPM47XXA                     0x4700                                   //
#define MCU_VERSION                 GPM47XXA                              //
//---------------------------------------------------------------------------


/*=== Board ID Config ===*/
//---------------------------------------------------------------------------
#define BOARD_EMU_BASE                  0x00000000                                 //
                                                                           //
#define BOARD_GPM41XXA_EMU_V1_0         (BOARD_EMU_BASE + 0x10)                    //
#define BOARD_GPM47XXA_EMU_V1_0         (BOARD_EMU_BASE + 0x20)                    //
#define BOARD_TYPE                      BOARD_GPM47XXA_EMU_V1_0
//---------------------------------------------------------------------------

/*=== software ID Config ===*/
//---------------------------------------------------------------------------
#define SOFTWARE_VER            "v1.6.0.0000.0000"                        //
//---------------------------------------------------------------------------

/*=== Key support ===*/
//---------------------------------------------------------------------------
#define KEY_AD_NONE             0                                          //
#define KEY_AD_4_MODE           4                                          //
#define KEY_AD_5_MODE           5                                          //
#define KEY_AD_6_MODE           6                                          //
#define KEY_AD_8_MODE           8                                          //
#define KEY_IO_MODE             2                                          //
#define KEY_USER_MODE           3                                          //
#define SUPPORT_AD_KEY          KEY_AD_8_MODE                              //
//---------------------------------------------------------------------------

/*=== web cam ===*/
//---------------------------------------------------------------------------
#define C_UVC						CUSTOM_OFF
#define C_USB_AUDIO					CUSTOM_OFF
//---------------------------------------------------------------------------

#endif //CUSTOMER_DEFINE


#endif //__CUSTOMER_H__
