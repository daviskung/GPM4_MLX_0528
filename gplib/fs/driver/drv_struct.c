#include "fsystem.h"

INT8U gNumDriver = MAX_DISK_NUM;
struct Drv_FileSystem *FileSysDrv[MAX_DISK_NUM] = {
//user setup this initial
/* A:\> */
#if NAND1_EN == 1
//Domi1
	&FS_NAND_driver,
#elif (MAX_DISK_NUM>0)
    NULL,
#endif

/* B:\> */
#if NAND2_EN == 1
//#err
	&FS_NAND2_driver,
#elif (MAX_DISK_NUM>1)
    NULL,
#endif

/* C:\> */
#if SD_EN == 1
	&FS_SD0_driver,
#elif (MAX_DISK_NUM>2)
    NULL,
#endif

/* D:\> */
#if MSC_EN == 1
	&FS_MSC_driver,
#elif (MAX_DISK_NUM>3)
    NULL,
#endif

/* E:\> */
#if CFC_EN == 1
//Domi4
    &FS_CFC_driver,
#elif (MAX_DISK_NUM>4)
    NULL,
#endif

/* F:\> */
//#if((SD_DUAL_SUPPORT==1)&&(MCU_VERSION>=GPL326XX_C))
#if SD_EN == 1
	&FS_SD1_driver,
#elif XD_EN == 1
	&FS_XD_driver,
#elif (MAX_DISK_NUM>5)
    NULL,
#endif

/* G:\> */
#if USB_NUM >= 1
	&FS_USB_driver,
#elif (MAX_DISK_NUM>6)
    NULL,
#endif

/* H:\> */
#if NOR_EN == 1
	&FS_SPI_FLASH_DISK_driver,//&FS_NOR_driver,
#elif (MAX_DISK_NUM>7)
    NULL,
#endif

/* I:\> */
#if CDROM_EN == 1
	&FS_CDROM_driver,
#elif (MAX_DISK_NUM>8)
    NULL,
#endif

/* J:\> */
#if (defined NAND_APP_EN) && (NAND_APP_EN == 1)
	&FS_NAND_App_driver,
#elif (MAX_DISK_NUM>9)
    NULL,
#endif

//#if (MCU_VERSION == GPL326XXB) || (MCU_VERSION == GP326XXXA)
/* K:\> */
#if SD_EN == 1
	&FS_SD1_driver,
#elif (MAX_DISK_NUM>10)
    NULL,
#endif
//#endif

/* L:\> */
#if USB_NUM >= 4
	&FS_USB4_driver,
#elif (MAX_DISK_NUM>11)
    NULL,
#endif

/* M:\> */
#if NAND3_EN == 1
	&FS_NAND3_driver,
#elif (MAX_DISK_NUM>11)
    NULL,
#endif

/* N:\> */

#if USB_NUM >= 3
	&FS_USB3_driver,
#elif (MAX_DISK_NUM>11)
    NULL,
#endif

/* O:\> */
#if RAMDISK_EN == 1
	&FS_RAMDISK_driver
#elif (MAX_DISK_NUM>12)
    NULL,
#endif

//other device struct initial base MAX_DISK_NUM
};
