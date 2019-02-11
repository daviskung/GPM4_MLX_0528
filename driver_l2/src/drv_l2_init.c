#include "drv_l2.h"


void drv_l2_init(void)
{
  #if (defined _DRV_L2_SYS_TIMER_EN) &&  (_DRV_L2_SYS_TIMER_EN == 1)
	sys_init_timer();
  #endif

  #if (defined _DRV_L2_SDC) && (_DRV_L2_SDC == DRV_L2_ON)
    drvl2_sd_set_bit_mode(drvl2_sd_drive_to_device_id(FS_SD1), 4); // FS_SD2 or FS_SD
  #endif

  #if (defined _DRV_L2_NAND) && (_DRV_L2_NAND == DRV_L2_ON)
  {
        INT16U ret=0;

        ret = DrvNand_initial();
        #if 1
        if (ret != 0)
        {
            DBG_PRINT("NAND L2 init fail %u\r\n", ret);
            ret = DrvNand_lowlevelformat();
            if (ret == 0)
            {
                DBG_PRINT("NAND L2 low format success.\r\n");
                ret = DrvNand_initial();
                if (ret != 0)
                {
                    DBG_PRINT("NAND L2 init fail even low formated %u\r\n", ret);
                }
                else
                    DBG_PRINT("NAND L2 init OK after low formatted.\r\n");
            }
            else
                 DBG_PRINT("NAND L2 low format fail %u\r\n", ret);
        }
        else
            DBG_PRINT("NAND L2 init OK.\r\n");
        #endif
  }
  #endif

}
