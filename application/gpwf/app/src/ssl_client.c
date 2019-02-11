/************************************************************
* ssl_client.c
*
* Purpose: SSL client demo code
*
* Author: Eugene Hsu
*
* Date: 2016/08/09
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
#include "ssl_cli.h"

/**********************************************
*	Definitions
**********************************************/
#define SSLCLI_STACK_SIZE			1024
#define SSLCLI_Q_NUM				16
#define SSLCLI_TX_BUF_LEN			1500

/**********************************************
*	Extern variables and functions
**********************************************/
/*********************************************
*	Variables declaration
*********************************************/
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *sslcli_q = NULL;
static OS_EVENT *sslcli_rx_sem;
static void *sslcli_q_stack[SSLCLI_Q_NUM];
static INT32U SSLCliTaskStack[SSLCLI_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	static osSemaphoreDef_t  sslcli_rx_sem_d = {0};
	static osSemaphoreId sslcli_rx_sem;
	static osMessageQDef_t sslcli_q_d = { SSLCLI_Q_NUM, sizeof(INT32U), 0 };
	static osMessageQId sslcli_q = NULL;
#endif

static INT32U sslcli_first_init = 0;

void sslcli_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
{
	WIFI_MSG_T* msg = NULL;

	msg = (WIFI_MSG_T*)gp_malloc(sizeof(WIFI_MSG_T));
	if(msg == NULL)
	{
		DBG_PRINT("sslcli gspi cbk msg failed\r\n");
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
			DBG_PRINT("sslcli allocate buf failed\r\n");
			return;
		}
		memcpy(msg->buf, buf, msg->len);
	}
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(sslcli_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(sslcli_q, (uint32_t)&msg, osWaitForever);
	#endif
}

void sslcli_notify_connection_state(INT32U connection, INT32S state)
{
	INT32U event;

	/* Check message Q is ready */
	if(sslcli_q == NULL)
		return;

	if(connection == GSPI_SSLCLI_CONNECTION_1)
		event = GSPI_SSLCLI_1_CONNECT_STATE_CHANGED_EVENT;
	else if(connection == GSPI_SSLCLI_CONNECTION_2)
		event = GSPI_SSLCLI_2_CONNECT_STATE_CHANGED_EVENT;

	sslcli_gspi_cbk((INT8U*)&state, sizeof(INT32S), event);
}

void sslcli_task(void *parm)
{
	INT8U err;
	INT32S ssl_1_state;
	INT32S ssl_2_state;
	WIFI_MSG_T* msg = NULL;
	#if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif

	DBG_PRINT("Enter SSLCLI task...\r\n");

	while(1)
	{
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg = (WIFI_MSG_T*) OSQPend(sslcli_q, 0, &err);
		if(err != OS_NO_ERR || msg == NULL)
			continue;
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		result = osMessageGet(sslcli_q, osWaitForever);
		msg = (WIFI_MSG_T*)result.value.v;
		if (result.status != osEventMessage)
			continue;
		#endif


		switch(msg->event)
		{
			case GSPI_SSLCLI_1_TX_DATA_DONE_EVENT:
				DBG_PRINT("\r\nSSL1 TX done\r\n");
				break;

			case GSPI_SSLCLI_1_RX_DATA_DONE_EVENT:
				DBG_PRINT("\r\nSSL1 RX[%d]\r\n", msg->len);
				break;

			case GSPI_SSLCLI_1_CONNECT_STATE_CHANGED_EVENT:
				ssl_1_state = *(INT32S*)msg->buf;
				if(SSLCLI_CONNECT_STATE <= ssl_1_state && ssl_1_state <= SSLCLI_EOF_STATE)
				{
					char state_str[32];
					if(ssl_1_state == SSLCLI_CONNECT_STATE)
						strcpy(state_str, "SSL connected");
					else if(ssl_1_state == SSLCLI_DISCONNECT_STATE)
						strcpy(state_str, "SSL disconnected");
					else if(ssl_1_state == SSLCLI_CLIENT_FAILED_STATE)
						strcpy(state_str, "SSL client failed");
					else if(ssl_1_state == SSLCLI_EOF_STATE)
						strcpy(state_str, "SSL EOF");

					DBG_PRINT("SSL1 connection state changed, state [%s]\r\n", state_str);
				}
				else
				{
					/* Refer error code in sslcli.h */
					DBG_PRINT("SSL1 connection state changed, SSL err[0x%x]\r\n", -ssl_1_state);
				}
				break;

			case GSPI_SSLCLI_2_TX_DATA_DONE_EVENT:
				DBG_PRINT("\r\nSSL2 TX done\r\n");
				break;

			case GSPI_SSLCLI_2_RX_DATA_DONE_EVENT:
				DBG_PRINT("\r\nSSL2 RX[%d]\r\n", msg->len);
				break;

			case GSPI_SSLCLI_2_CONNECT_STATE_CHANGED_EVENT:
				ssl_2_state = *(INT32S*)msg->buf;
				if(SSLCLI_CONNECT_STATE <= ssl_2_state && ssl_2_state <= SSLCLI_EOF_STATE)
				{
					char state_str[32];
					if(ssl_2_state == SSLCLI_CONNECT_STATE)
						strcpy(state_str, "SSL connected");
					else if(ssl_2_state == SSLCLI_DISCONNECT_STATE)
						strcpy(state_str, "SSL disconnected");
					else if(ssl_2_state == SSLCLI_CLIENT_FAILED_STATE)
						strcpy(state_str, "SSL client failed");
					else if(ssl_2_state == SSLCLI_EOF_STATE)
						strcpy(state_str, "SSL EOF");

					DBG_PRINT("SSL2 connection state changed, state [%s]\r\n", state_str);
				}
				else
				{
					/* Refer error code in sslcli.h */
					DBG_PRINT("SSL2 connection state changed, SSL err[0x%x]\r\n", -ssl_2_state);
				}
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
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(SSLCLI_PRIORITY);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	id = osThreadGetId();
    osThreadTerminate(id);
    #endif
}

void sslcli_task_init(void)
{
	INT8U err;

	if(sslcli_first_init)
		return;

	/* Task only init once */
	sslcli_first_init = 1;

	if(sslcli_q == NULL)
	{
	    #if (_OPERATING_SYSTEM == _OS_UCOS2)
		sslcli_q = OSQCreate(sslcli_q_stack, SSLCLI_Q_NUM);
		if(sslcli_q == 0)
		{
			DBG_PRINT("Create sslcli_q failed\r\n");
			return;
		}
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        sslcli_q = osMessageCreate(&sslcli_q_d, NULL);
        if(sslcli_q == 0)
		{
			DBG_PRINT("Create sslcli_q failed\r\n");
			return;
		}
        #endif
	}
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	sslcli_rx_sem = OSSemCreate(1);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	sslcli_rx_sem = osSemaphoreCreate(&sslcli_rx_sem_d, 1);
	#endif

	gspi_register_app_cbk((INT32U)sslcli_gspi_cbk, GSPI_SSLCLI_APP);

	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	err = OSTaskCreate(sslcli_task, (void *)NULL, &SSLCliTaskStack[SSLCLI_STACK_SIZE - 1], SSLCLI_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Can't create SSLCLI task\n\r");
		return;
	}
	else
	{
		DBG_PRINT("Create SSLCLI task ok\r\n");
	}
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	osThreadDef_t sslcli_task_ent = {"sslcli_task", (os_pthread) sslcli_task, osPriorityNormal, 1, SSLCLI_STACK_SIZE};
	id = osThreadCreate(&sslcli_task_ent, (void*)NULL);
	if (id== 0)
	{
		DBG_PRINT("Cannot create SSLCLI task\n\r");
		return;
	}
	else
	{
		DBG_PRINT("Create SSLCLI task ok\r\n");
	}
        #endif
}

INT32S sslcli_1_connection_enable(void)
{
	INT32S ret = 0;
	INT32S cnt = WIFI_WAIT_TIMEOUT;

	/* Connect to SSL 1 */
	gp_net_app_state.sslcli_1_state = SSLCLI_DISCONNECT_STATE;
	gspi_send_docmd(GP_CMD_ENABLE_SSL_1);

	while((gp_net_app_state.sslcli_1_state != SSLCLI_CONNECT_STATE) && cnt)
	{
		cnt--;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(100);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay (100*10);
		#endif
	}

	if(gp_net_app_state.sslcli_1_state != SSLCLI_CONNECT_STATE)
	{
		DBG_PRINT("SSL 1 connected to server failed, state 0x%x", gp_net_app_state.sslcli_1_state);
		ret = -1;
	}

	return ret;
}

void sslcli_1_connection_disable(void)
{
	/* Release SSL1 resource in WiFi module */
	gspi_send_docmd(GP_CMD_DISABLE_SSL_1);
}

INT32S sslcli_2_connection_enable(void)
{
	INT32S ret = 0;
	INT32S cnt = WIFI_WAIT_TIMEOUT;

	/* Connect to SSL 2 */
	gp_net_app_state.sslcli_2_state = SSLCLI_DISCONNECT_STATE;
	gspi_send_docmd(GP_CMD_ENABLE_SSL_2);

	while((gp_net_app_state.sslcli_2_state != SSLCLI_CONNECT_STATE) && cnt)
	{
		cnt--;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(100);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay (100*10);
		#endif
	}

	if(gp_net_app_state.sslcli_2_state != SSLCLI_CONNECT_STATE)
	{
		DBG_PRINT("SSL 2 connected to server failed, state 0x%x", gp_net_app_state.sslcli_2_state);
		ret = -1;
	}

	return ret;
}

void sslcli_2_connection_disable(void)
{
	/* Release SSL2 resource in WiFi module */
	gspi_send_docmd(GP_CMD_DISABLE_SSL_2);
}
