#ifndef __AD_KEY_DRIVER_H__
#define __AD_KEY_DRIVER_H__

#include "drv_l2.h"
#include "application.h"

#define C_AD_KEY_CH				0

#define C_AD_VALUE_0			20
#define C_AD_VALUE_1			100
#define C_AD_VALUE_2			200
#define C_AD_VALUE_3			350
#define C_AD_VALUE_4			500
#define C_AD_VALUE_5			600
#define C_AD_VALUE_6			750
#define C_AD_VALUE_7			820
#define C_AD_VALUE_8			1020

#define C_AD_KEY_DEBOUNCE                       4//3	// 3 * 1 / 128 = 23ms

#define C_AD_KEY_START_REPEAT                   96//64	// 128 * 1 / 128 = 0.5s
#define C_AD_KEY_REPEAT_COUNT                   12//8	// 8 * 1 / 128 = 0.5s

#define C_INVALID_KEY                           0xffffffff

// ad key
#define AD_KEY_1                                0
#define AD_KEY_2                                1
#define AD_KEY_3                                2
#define AD_KEY_4                                3
#define AD_KEY_5                                4
#define AD_KEY_6                                5
#define AD_KEY_7                                6
#define AD_KEY_8                                7

typedef enum {
    RB_KEY_DOWN,
    RB_KEY_UP,
    RB_KEY_REPEAT,
    MAX_KEY_TYPES
}KEY_TYPES_ENUM;


extern void ad_key_initial(void);
extern void ad_key_uninitial(void);
extern INT32U ad_key_get_key(void);

extern void ad_key_set_repeat_time(INT8U start_repeat, INT8U repeat_count);
extern void ad_key_get_repeat_time(INT8U *start_repeat, INT8U *repeat_count);

#endif
