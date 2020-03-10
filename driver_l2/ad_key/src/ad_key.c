/*
* Purpose: ad key driver
*
* Author: zhangzha
*
* Date: 2008/04/28
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Note: AD key driver use system timer manager api, so if you want to use ad key,
* you must initial the system timer first.
* The system time tick is 128Hz
*
* Version : 1.01
* History :
          1. 2008/4/28 zhangzha create.
          2. 2008/6/10 allen lin modify.
*/

//Include files
#include "ad_key.h"

INT32U ad_key_count;
INT32U ad_key_value;
INT8U  is_key_down;
INT8U  ad_key_start_repeat;
INT8U  ad_key_repeat_count;

static xTimerHandle adkey_timer;

extern xQueueHandle KeyTaskQ;

#if (_OPERATING_SYSTEM == _OS_FREERTOS)
static osMutexId adkey_sw_sem;
#endif

// Functions defined in this file
#if 0 //m-
static void ad_key_timer_isr(void);
#else
static void ad_key_timer_isr(xTimerHandle);
#endif
static void ad_key_adc_isr(INT16U ad_val);

typedef struct
{
    INT8U  key_code;
    INT8U  key_type;
} ADkeySYSPara;

ADkeySYSPara key_para;

#if 0 // m-
fp_msg_qsend msg_QSend;
#endif

void *msgQId;
INT32U msgId;

/*
* Function Name :  ad_key_initial
* Syntax : void ad_key_initial(void)
* Purpose :  initialization for ADC channel, register callback function, and  register timer for ad_key
* Parameters : none
* Return : none
*/
void ad_key_initial(void)
{
	ad_key_count = 0;
	is_key_down = 0;
    ad_key_value = C_INVALID_KEY;

	ad_key_start_repeat = C_AD_KEY_START_REPEAT;
	ad_key_repeat_count = C_AD_KEY_REPEAT_COUNT;

	drv_l1_adc_init();
	drv_l1_adc_manual_ch_set(C_AD_KEY_CH);
	drv_l1_adc_manual_callback_set(ad_key_adc_isr);
	drv_l1_adc_manual_sample_start();

#if _OPERATING_SYSTEM == _OS_FREERTOS

    if(adkey_sw_sem == NULL)
	{
        adkey_sw_sem = osMutexCreate(NULL);
		if(adkey_sw_sem == 0)
			DBG_PRINT("Failed to allocate adkey_sw_sem memory\r\n");
	}

    adkey_timer = xTimerCreate("ADKEY_TIMER", 10/portTICK_RATE_MS, pdTRUE, NULL, ad_key_timer_isr);
    if (adkey_timer != NULL)
        xTimerStart(adkey_timer, 0);

#endif
}

/*
* Function Name :  ad_key_uninitial
* Syntax : void ad_key_uninitial(void)
* Purpose :  uninitialization for ad key (release the registered timer for ad_key)
* Parameters : none
* Return : none
*/
void ad_key_uninitial(void)
{
#if _OPERATING_SYSTEM == _OS_FREERTOS
    if (adkey_timer != NULL)
        xTimerStop(adkey_timer, 0);
    xTimerDelete(adkey_timer, 0);

    if(adkey_sw_sem)
        vSemaphoreDelete(adkey_sw_sem);
	adkey_sw_sem = NULL;
#endif
}

/*
* Function Name :  ad_key_set_repeat_time
* Syntax : void ad_key_set_repeat_time(INT8U start_repeat, INT8U repeat_count)
* Purpose : set repeat time and repeat count for key scan
* Parameters : INT8U start_repeat: key hold time
*							 INT8U repeat_count: key hold count
* Return : none
*/
void ad_key_set_repeat_time(INT8U start_repeat, INT8U repeat_count)
{
	ad_key_start_repeat = start_repeat;
	ad_key_repeat_count = repeat_count;
}
/*
* Function Name :  ad_key_get_repeat_time
* Syntax : void ad_key_get_repeat_time(INT8U *start_repeat, INT8U *repeat_count)
* Purpose : get repeat time and repeat count for key scan
* Parameters : INT8U *start_repeat: key hold time pointer
*							 INT8U *repeat_count: key hold count pointer
* Return : none
*/
void ad_key_get_repeat_time(INT8U *start_repeat, INT8U *repeat_count)
{
	*start_repeat = ad_key_start_repeat;
	*repeat_count = ad_key_repeat_count;
}

static void adkey_lock(void)
{
	if(adkey_sw_sem)
        #if (_OPERATING_SYSTEM == _OS_FREERTOS)
        osMutexWait(adkey_sw_sem, 0);
        #endif

}

static void adkey_unlock(void)
{
	if(adkey_sw_sem)
        #if (_OPERATING_SYSTEM == _OS_FREERTOS)
        osMutexRelease(adkey_sw_sem);
        #endif

}

/*
* Function Name :  ad_key_timer_isr
* Syntax : static void ad_key_timer_isr(void)
* Purpose : ad key scan timer interrupt callback function
* Parameters : none
* Return : none
*/
#if 0 //m-
static void ad_key_timer_isr(void)
#else
static void ad_key_timer_isr(xTimerHandle pxTimer)
#endif
{
    adkey_lock();
	drv_l1_adc_manual_sample_start();
	adkey_unlock();
}

/*
* Function Name :  ad_key_adc_isr
* Syntax : static void ad_key_adc_isr(INT16U ad_val)
* Purpose :  manual adc callback function and send the result to key scan queue
* Parameters : INT16U ad_val: adc value from registered adc channel
* Return : none
*/
static void ad_key_adc_isr(INT16U ad_val)
{
	INT32U	key;

	ad_val >>= 6;
    //DBG_PRINT("AD = %d ",ad_val);
	if((ad_val > C_AD_VALUE_0)&(ad_val <= C_AD_VALUE_1)) {
		key = AD_KEY_1;
	} else if((ad_val > C_AD_VALUE_1)&(ad_val <= C_AD_VALUE_2)) {
		key = AD_KEY_2;
	} else if((ad_val > C_AD_VALUE_2)&(ad_val <= C_AD_VALUE_3)) {
		key = AD_KEY_3;
	} else if((ad_val > C_AD_VALUE_3)&(ad_val <= C_AD_VALUE_4)) {
		key = AD_KEY_4;
	} else if((ad_val > C_AD_VALUE_4)&(ad_val <= C_AD_VALUE_5)) {
		key = AD_KEY_5;
	} else if((ad_val > C_AD_VALUE_5)&(ad_val <= C_AD_VALUE_6)) {
		key = AD_KEY_6;
	} else if((ad_val > C_AD_VALUE_6)&(ad_val <= C_AD_VALUE_7)) {
		key = AD_KEY_7;
 	} else if((ad_val > C_AD_VALUE_7)&(ad_val <= C_AD_VALUE_8)) {
 		key = AD_KEY_8;
	} else if((R_SYSTEM_POWER_CTRL1 & 0x01) == 1) {		// for power ON/OFF
 		key = AD_KEY_forPWR_ON0;
	} else {
		key = C_INVALID_KEY;	// no key pressed
	}

	if(key == C_INVALID_KEY) {
		if (is_key_down) {
			key_para.key_code = ad_key_value;
			key_para.key_type = RB_KEY_UP;
#if 0 // m-
			msg_QSend(msgQId,msgId,(void*)&key_para,sizeof(ADkeySYSPara),0);
#else
            if (KeyTaskQ)
                xQueueSendFromISR( (xQueueHandle) KeyTaskQ, (void *)&key_para, 0);
#endif
			is_key_down = 0;
		}
		ad_key_count = 0;
		ad_key_value = key;
		return;
	}

	if(key == ad_key_value) {
		ad_key_count += 1;
		if(ad_key_count == C_AD_KEY_DEBOUNCE) {
			is_key_down = 1;
			key_para.key_code = key;
			key_para.key_type = RB_KEY_DOWN;
#if 0 //m-
			msg_QSend(msgQId,msgId,(void*)&key_para,sizeof(ADkeySYSPara),0);
#else
            if (KeyTaskQ)
                xQueueSendFromISR( (xQueueHandle) KeyTaskQ, (void *)&key_para, 0);
#endif
        } else if(ad_key_count == ad_key_start_repeat) {
			key_para.key_code = key;
			key_para.key_type = RB_KEY_REPEAT;
#if 0 //m-
			msg_QSend(msgQId,msgId,(void*)&key_para,sizeof(ADkeySYSPara),0);
#else
                        if (KeyTaskQ)
                          xQueueSendFromISR( (xQueueHandle) KeyTaskQ, (void *)&key_para, 0);
#endif
			ad_key_count -= ad_key_repeat_count;
		}
	} else {
		ad_key_count = 0;
		ad_key_value = key;
	}
}

void turnkey_adkey_resource_register(void* msg_qsend,void* msgq_id,INT32U msg_id)
{
#if 0 //m-
	msg_QSend = (fp_msg_qsend) msg_qsend;
	msgQId = msgq_id;
	msgId = msg_id;
#endif
}

