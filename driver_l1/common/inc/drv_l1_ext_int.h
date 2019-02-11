#ifndef __drv_l1_EXT_INT_H__
#define __drv_l1_EXT_INT_H__

#include "drv_l1.h"
#include "drv_l1_sfr.h"

typedef enum
{
	EXTA,
	EXTB,
	EXTC
}EXTAB_ENUM;

typedef enum
{
	FALLING,
	RISING
}EXTAB_EDGE_ENUM;

#define EXTC_INT 0x100
#define EXTB_INT 0x80
#define EXTA_INT 0x40
#define EXTC_POL 0x20
#define EXTB_POL 0x10
#define EXTA_POL 0x08
#define EXTC_IEN 0x04
#define EXTB_IEN 0x02
#define EXTA_IEN 0x01


extern void drv_l1_ext_int_init(INT8U ext_src);
extern void drv_l1_ext_int_clr(INT8U ext_src);
extern void drv_l1_ext_edge_set(INT8U ext_src, INT8U edge_type);
extern void drv_l1_ext_enable_set(INT8U ext_src, BOOLEAN status);
extern void drv_l1_ext_user_isr_set(INT8U ext_src,void (*user_isr)(void));
extern void drv_l1_ext_user_isr_clr(INT8U ext_src);

#endif		// __drv_l1_EXT_INT_H__
