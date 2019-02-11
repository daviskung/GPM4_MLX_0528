
#define DBG_PRINT		print_string

// Global definitions
#define TRUE			1
#define FALSE			0
#define ENABLE					1
#define DISABLE					0
//#define NULL			0
#define STATUS_OK		0
#define STATUS_FAIL		-1

// Operating System definitions and config
#define _OS_NONE		0
#define _OS_UCOS2		1
#define _OS_FREERTOS	2
#define	_OPERATING_SYSTEM       _OS_FREERTOS

#include "ARMCM4_FP.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "typedef.h"
#include "drv_l1_cfg.h"
#include "drv_l2_cfg.h"
#include "gplib_cfg.h"
#include "customer.h"
#include "application_cfg.h"
#include "drv_l1_sfr.h"
#include <stdlib.h>

#define malloc(x)      gp_malloc(x)
//#define DRV_Reg32(addr)		        (*(volatile unsigned *)(addr))

extern INT32U MCLK;
extern INT32U MHZ;
