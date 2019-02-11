#ifndef __DRV_L1_CONV420TO422_H__
#define __DRV_L1_CONV420TO422_H__

#include "drv_l1.h"
#include "drv_l1_sfr.h"

typedef struct 
{										// Offset
	volatile INT32U	CTRL;           	// 0x0000
	volatile INT32U	LINE_WIDTH_4BYTE;  	// 0x0004
	volatile INT32U	IN_ADDR_A;      	// 0x0008
	volatile INT32U	IN_ADDR_B;      	// 0x000C
	volatile INT32U	MASTER_MASK_INT;   	// 0x0010
}CONV420_SFR;

typedef enum
{
	CONV420_TO_TV_HDMI,
	CONV420_TO_PSCALER
}CONV420_PATH;

typedef enum
{
	CONV420_CONVERT_DISABLE,
	CONV420_CONVERT_ENABLE
}CONV420_CONVERT_SETTING;


//+++ Status
#define CONV420_STATUS_BUSY					0x00000000
#define CONV420_STATUS_BUF_A_DONE			0x00000002
#define CONV420_STATUS_BUF_B_DONE			0x00000004


/******************************************************/
// Conv420To422 Extern APIs
extern INT32S conv420_init(void);
extern INT32S conv420_reset(void);
extern INT32S conv420_start(void);
extern INT32S conv420_convert_enable(INT8U ctrlValue);
extern INT32S conv420_path(INT8U pathValue);
extern INT32S conv420_input_A_addr_set(INT32U addr);
extern INT32S conv420_input_B_addr_set(INT32U addr);
extern INT32S conv420_input_pixels_set(INT32U inWidth);
extern INT32S conv420_fifo_line_set(INT32U lineCount);
extern INT32S conv420_idle_buf_get(void);
extern INT32S conv420_fifo_interrupt_enable(INT8U enableValue);
extern INT32S conv420_fifo_interrupt_status_get(void);

extern void Conv420_Callback_Register(void (*Conv420_Callback)(INT32U Conv420_Event));

#endif		// __DRV_L1_CONV420TO422_H__

