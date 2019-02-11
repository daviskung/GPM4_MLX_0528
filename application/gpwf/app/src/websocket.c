/************************************************************
* websocket.c
*
* Purpose: Websocket with SSL
*
* Author: Eugene Hsu
*
* Date: 2016/05/05
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.10
* History :
*
************************************************************/
#include "string.h"
#include "stdio.h"
#include "project.h"
#include "drv_l1_gpio.h"
#include "application.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"
#include "wifi_demo.h"

/**********************************************
*	Definitions
**********************************************/
#define WEBSOCKET_STACK_SIZE		1024
#define WEBSOCKET_Q_NUM				16
#define WEBSOCKET_TX_BUF_LEN		1500

/**********************************************
*	Extern variables and functions
**********************************************/
/*********************************************
*	Variables declaration
*********************************************/
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *websocket_q = NULL;
static OS_EVENT *websocket_rx_sem;
static void *websocket_q_stack[WEBSOCKET_Q_NUM];
static INT32U WebSocketTaskStack[WEBSOCKET_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
static osMessageQDef_t websocket_q_d = {WEBSOCKET_Q_NUM, sizeof(INT32U), 0};
static osMessageQId websocket_q;
static osSemaphoreDef_t websocket_rx_sem_d = {0};
static osSemaphoreId websocket_rx_sem;
#endif
static INT32U websocket_first_init = 0;
static char ws_tx_buf[WEBSOCKET_TX_BUF_LEN] = {0};

void websocket_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
{
	WIFI_MSG_T* msg = NULL;

	msg = (WIFI_MSG_T*)gp_malloc(sizeof(WIFI_MSG_T));
	if(msg == NULL)
	{
		DBG_PRINT("ws gspi cbk msg failed\r\n");
		return;
	}

	msg->event = event;
	msg->len = len;		/* Data size in bytes */
	msg->buf = NULL;
	if(msg->len)
	{
		msg->buf = (char*)gp_malloc(msg->len);
		if(msg->buf == NULL)
		{
			DBG_PRINT("ws allocate buf failed\r\n");
			return;
		}
		memcpy(msg->buf, buf, msg->len);
	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(websocket_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(websocket_q, (uint32_t)&msg, osWaitForever);
	DBG_PRINT("ws MessagePut\r\n");
	#endif
}

void websocket_task(void *parm)
{
	INT8U err;
	INT8S res;
	WIFI_MSG_T* msg = NULL;
    #if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif

	DBG_PRINT("Enter Websocket task...\r\n");


	osDelay(10);  // why does delay add at there? ming
	gspi_transfer_websocket((INT8U*)"hello", 5, GSPI_WEBSOCKET_TEXT_DATA);

	while(10)
	{
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg = (WIFI_MSG_T*) OSQPend(websocket_q, 0, &err);
		if(err != OS_NO_ERR || msg == NULL)
			continue;
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        result = osMessageGet(websocket_q, osWaitForever);
        msg = (WIFI_MSG_T*) result.value.v;
        if (result.status != osEventMessage)
                continue;
        #endif

		switch(msg->event)
		{
			case GSPI_WEBSOCKET_TX_TEXT_DONE_EVENT:
				//DBG_PRINT("TX text done\r\n");
				break;

			case GSPI_WEBSOCKET_TX_BINARY_DONE_EVENT:
				//DBG_PRINT("TX binary done\r\n");
				break;

			case GSPI_WEBSOCKET_RX_DATA_DONE_EVENT:
				DBG_PRINT("WS RX[%d]:\r\n 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", msg->len, msg->buf[0], msg->buf[1], msg->buf[2], msg->buf[3], msg->buf[4]);
				gspi_transfer_websocket((INT8U*)"hello", 5, GSPI_WEBSOCKET_TEXT_DATA);
				break;

			default:
				break;
		}

		/* Free message and realted buffers */
		if(msg->buf)
		{
			gp_free(msg->buf);
		}
		gp_free(msg);
	}

	/* Exit task */
	#if(_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(WEBSOCKET_PRIORITY);
	#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId  id;
	id = osThreadGetId();
	osThreadTerminate(id);
	#endif

}

void websocket_task_init(void)
{
	INT8U err;

	if(websocket_first_init)
		return;

	/* Task only init once */
	websocket_first_init = 1;

	if(websocket_q == NULL)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		websocket_q = OSQCreate(websocket_q_stack, WEBSOCKET_Q_NUM);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		websocket_q = osMessageCreate(&websocket_q_d, NULL);
		#endif
		if(websocket_q == 0)
		{
			DBG_PRINT("Create websocket_q failed\r\n");
			return;
		}
	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	websocket_rx_sem = OSSemCreate(1);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    websocket_rx_sem = osSemaphoreCreate(&websocket_rx_sem_d, 1);
    #endif

	gspi_register_app_cbk((INT32U)websocket_gspi_cbk, GSPI_WEBSOCKET_APP);

    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	err = OSTaskCreate(websocket_task, (void *)NULL, &WebSocketTaskStack[WEBSOCKET_STACK_SIZE - 1], WEBSOCKET_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Can't create Websocket task\n\r");
		return;
	}
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	osThreadDef_t websocket_task_ent = {"websocket_task_entry", (os_pthread)websocket_task, osPriorityNormal, 1, WEBSOCKET_STACK_SIZE};
	id = osThreadCreate(&websocket_task_ent, (void*)NULL);
	if (id ==0)
	{
		DBG_PRINT("Can't create Websocket task\n\r");
		return;
	}
	#endif
	else
	{
		DBG_PRINT("Create Websocket task ok\r\n");
	}
}
