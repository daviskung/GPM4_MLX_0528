#include "drv_l2_ad_key_scan.h"

#define MSG_AD_KEY	1
#define KEY_QUEUE_MAX	2
#define KEY_QUEUE_MAX_MSG_LEN   5

#if 0 //m-
MSG_Q_ID KeyTaskQ;
#else
xQueueHandle KeyTaskQ = NULL;
#endif

INT32U ADKEY_IO1, ADKEY_IO2, ADKEY_IO3, ADKEY_IO4;
INT32U ADKEY_IO5, ADKEY_IO6, ADKEY_IO7, ADKEY_IO8;
INT32U ADKEY_forPWR_ON0;
INT32U ADKEY_IO1_C, ADKEY_IO2_C, ADKEY_IO3_C, ADKEY_IO4_C;
INT32U ADKEY_IO5_C, ADKEY_IO6_C, ADKEY_IO7_C, ADKEY_IO8_C;
INT32U ADKEY_forPWR_ON0_C;


extern void turnkey_adkey_resource_register(void* msg_qsend,void* msgq_id,INT32U msg_id);

/*
* Function Name :  adc_key_scan_init
* Syntax : INT32U adc_key_scan_init(void)
* Purpose :  create adkey queue, register resources and do hareware initialization
*						 and update corresponding globle variables
* Parameters : none
* Return : none
*/
INT32U adc_key_scan_init(void)
{
#if 0 //m-
	KeyTaskQ = msgQCreate(KEY_QUEUE_MAX, KEY_QUEUE_MAX, KEY_QUEUE_MAX_MSG_LEN);
	turnkey_adkey_resource_register((void*)msgQSend, KeyTaskQ, MSG_AD_KEY);
#else
        KeyTaskQ = xQueueCreate(KEY_QUEUE_MAX_MSG_LEN, sizeof(INT16U));
#endif
	ad_key_initial();

	return 0;
}
/*
* Function Name :  adc_key_scan_uninit
* Syntax : INT32U adc_key_scan_uninit(void)
* Purpose :  uninit adc_key_scan
* Parameters : none
* Return : none
*/
INT32U adc_key_scan_uninit(void)
{
#if 0 //m-
	msgQDelete(KeyTaskQ);
	ad_key_uninitial();
#else
    if(KeyTaskQ) {
            vQueueDelete(KeyTaskQ);
    KeyTaskQ = 0;
	}
	ad_key_uninitial();
#endif

	return 0;
}
/*
* Function Name :  adc_key_scan
* Syntax : INT32U adc_key_scan(void)
* Purpose :  get ADKey from adkey queue if any adkey is pressed
*						 and update corresponding globle variables
* Parameters : none
* Return : none
*/
INT32U adc_key_scan(void)
{
	INT8U  KeyPara[2];
    INT32S error;

#if 0 //m-
	INT32U msg_id;
	msgQReceive(KeyTaskQ, &msg_id, KeyPara, sizeof(KeyPara));

	if(msg_id != MSG_AD_KEY)
		return -1;
#else
        error = (INT32S) xQueueReceive(KeyTaskQ, &KeyPara, portMAX_DELAY);
#endif
	if(KeyPara[1] == RB_KEY_DOWN)
	{
		switch(KeyPara[0])
		{
		case AD_KEY_1:	ADKEY_IO1 = 1;	break;
		case AD_KEY_2:	ADKEY_IO2 = 1;	break;
		case AD_KEY_3:	ADKEY_IO3 = 1;	break;
		case AD_KEY_4:	ADKEY_IO4 = 1;	break;
		case AD_KEY_5:	ADKEY_IO5 = 1;	break;
		case AD_KEY_6:	ADKEY_IO6 = 1;	break;
		case AD_KEY_7:	ADKEY_IO7 = 1;	break;
		case AD_KEY_8:	ADKEY_IO8 = 1;	break;
		
		case AD_KEY_forPWR_ON0:	ADKEY_forPWR_ON0 = 1;	break;
		}
		//DBG_PRINT("PD = %d\r\n", KeyPara[0]);
	}
	else if(KeyPara[1] == RB_KEY_REPEAT)
	{
		switch(KeyPara[0])
		{
		case AD_KEY_1:	ADKEY_IO1_C = 1;	break;
		case AD_KEY_2:	ADKEY_IO2_C = 1;	break;
		case AD_KEY_3:	ADKEY_IO3_C = 1;	break;
		case AD_KEY_4:	ADKEY_IO4_C = 1;	break;
		case AD_KEY_5:	ADKEY_IO5_C = 1;	break;
		case AD_KEY_6:	ADKEY_IO6_C = 1;	break;
		case AD_KEY_7:	ADKEY_IO7_C = 1;	break;
		case AD_KEY_8:	ADKEY_IO8_C = 1;	break;
		
		case AD_KEY_forPWR_ON0:	ADKEY_forPWR_ON0_C = 1;	break;
		}
		//DBG_PRINT("PR = %d\r\n", KeyPara[0]);
	}
	else if(KeyPara[1] == RB_KEY_UP)
	{
		ADKEY_IO1 = 0;
		ADKEY_IO2 = 0;
		ADKEY_IO3 = 0;
		ADKEY_IO4 = 0;
		ADKEY_IO5 = 0;
		ADKEY_IO6 = 0;
		ADKEY_IO7 = 0;
		ADKEY_IO8 = 0;

		ADKEY_forPWR_ON0 = 1;

		ADKEY_IO1_C = 0;
		ADKEY_IO2_C = 0;
		ADKEY_IO3_C = 0;
		ADKEY_IO4_C = 0;
		ADKEY_IO5_C = 0;
		ADKEY_IO6_C = 0;
		ADKEY_IO7_C = 0;
		ADKEY_IO8_C = 0;

		ADKEY_forPWR_ON0_C = 0;
		//DBG_PRINT("PU\r\n");
	}
	return 0;
}
