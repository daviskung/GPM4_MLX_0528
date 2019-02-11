#ifndef DRV_L2_H
#define DRV_L2_H

#include "project.h"

// SD and MMC card access APIs for system
typedef struct {
	INT32U csd[4];
	INT32U scr[2];
	INT32U cid[4];
	INT32U ocr;
	INT16U rca;
} SD_CARD_INFO_STRUCT;

//#include "drv_l2_spi_flash.h"
//#include "drv_l2_sd.h"

void drv_l2_init(void);

#endif		// __DRV_L2_H__
