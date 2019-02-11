/************************************************************
* udp_server.c
*
* Purpose: UDP server APP
*
* Author: Eugene Hsu
*
* Date: 2015/12/02
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
#include "video_encoder.h"
#include "avi_encoder_app.h"
#include "application.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"
#include "wifi_demo.h"

/**********************************************
*	Definitions
**********************************************/
#define UDP_SERVER_BROADCAST		1

#define UDP_SERVER_STACK_SIZE		1024
#define UDP_SERVER_Q_NUM			16
#define UDP_SERVER_WAIT_TIMEOUT		60		/* Wait 60 seconds to connect server */
#define UDP_SERVER_TX_BUF_SIZE		1600
#define UDP_SERVER_RX_BUF_SIZE		1600

#define UDP_SERVER_PORT				"1688"
/**********************************************
*	Extern variables and functions
**********************************************/
/*********************************************
*	Variables declaration
*********************************************/
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *udp_server_q = NULL;
static OS_EVENT *udp_server_rx_sem;
static void *udp_server_q_stack[UDP_SERVER_Q_NUM];
static INT32U UDPServerTaskStack[UDP_SERVER_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	static osSemaphoreDef_t  udp_server_rx_sem_d = {0};
	static osSemaphoreId udp_server_rx_sem;
	static osMessageQDef_t udp_server_q_d = { UDP_SERVER_Q_NUM, sizeof(INT32U), 0 };
	static osMessageQId udp_server_q = NULL;
#endif
static char enable_atcmd[64] = {0};
//static INT8U udp_server_tx_buf[UDP_SERVER_TX_BUF_SIZE+4] = {0};
static INT8U udp_server_rx_buf[UDP_SERVER_RX_BUF_SIZE+4] = {0};

static INT32U udpsrv_rx_len = 0;
static GP_UDP_SRV_CLI_INTFO_T udp_client_info = {0};
static INT32U udpsrv_first_init = 0;

void udp_server_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
{
	static INT32U remain, cur_cnt;
	INT32U msg = event;
	//INT32U i;
	INT32U copy_cnt;
	INT32U send_q = 1;
	INT8U err;

	if(event == GSPI_US_RX_CLIINFO_EVENT)
	{
		#if(_OPERATING == _OS_UCOS2)
		OSSemPend(udp_server_rx_sem, 0, &err);
		#elif(_OPERATING==_OS_FREERTOS)
		osSemaphoreWait( udp_server_rx_sem, osWaitForever);
		#endif
		memcpy((INT8U*)&udp_client_info, buf, sizeof(udp_client_info));
	}
	else if(event == GSPI_US_RX_LEN_EVENT)
	{
		#if(_OPERATING == _OS_UCOS2)
		OSSemPend(udp_server_rx_sem, 0, &err);
		#elif(_OPERATING==_OS_FREERTOS)
		osSemaphoreWait( udp_server_rx_sem, osWaitForever);
		#endif
		udpsrv_rx_len = len;
		remain = udpsrv_rx_len;
		cur_cnt = 0;
		memset((void*)udp_server_rx_buf, 0, UDP_SERVER_RX_BUF_SIZE);
	}
	else if(event == GSPI_US_RX_DATA_EVENT)
	{
		#if(_OPERATING == _OS_UCOS2)
		OSSemPend(udp_server_rx_sem, 0, &err);
		#elif(_OPERATING==_OS_FREERTOS)
		osSemaphoreWait(udp_server_rx_sem, osWaitForever);
		#endif
		if(remain)
		{
			copy_cnt = remain;
			if(copy_cnt > UDP_SERVER_RX_BUF_SIZE)
			{
				copy_cnt = UDP_SERVER_RX_BUF_SIZE;
			}
			memcpy((char*)(udp_server_rx_buf + cur_cnt), buf, copy_cnt);

			remain -= copy_cnt;
			cur_cnt += copy_cnt;

			if(remain)
			{
				send_q = 0;
			}
		}
	}
	else
	{
		/* Normal event */
	}

	if(send_q)
	{
		#if(_OPERATING==_OS_UCOS2)
		OSQPost(udp_server_q, (void*)msg);
		#elif(_OPERATING==_OS_FREERTOS)
		osMessagePut(udp_server_q, (uint32_t)&msg, osWaitForever);
		#endif
    }
}

INT32S udp_server_enable(void)
{
	INT32S ret = -1;
	INT32U retry = 10, cnt;

connect_to_udp_server:

	cnt = UDP_SERVER_WAIT_TIMEOUT;
	/* Enable TCP client to connect TCP server */
#if (UDP_SERVER_BROADCAST == 0)
	sprintf(enable_atcmd, GP_CMD_ENABLE_UDP_SERVER "1:%s", UDP_SERVER_PORT);
#else
	sprintf(enable_atcmd, GP_CMD_ENABLE_UDP_SERVER "broadcast:%s", UDP_SERVER_PORT);
#endif
	gspi_send_docmd(enable_atcmd);
	while((gp_net_app_state.udpsrv_state != UDP_SERVER_CONNECT_READY_STATE) && cnt)
	{
		cnt--;
		#if(_OPERATING==_OS_UCOS2)
		OSTimeDly(100);
		#elif(_OPERATING==_OS_FREERTOS)
		osDelay(100);
		#endif
	}

	if(gp_net_app_state.udpsrv_state == UDP_SERVER_DISCONNECT_STATE)
	{
		retry--;
		DBG_PRINT("Enable UDP server failed, retry %d\r\n", retry);
		if(retry)
		{
			cnt = UDP_SERVER_WAIT_TIMEOUT;
			gspi_send_docmd(GP_CMD_DISABLE_UDP_SERVER);
			while((gp_net_app_state.udpsrv_state == UDP_SERVER_DISABLE_STATE) && cnt)
			{
				cnt--;
				#if(_OPERATING==_OS_UCOS2)
				OSTimeDly(100);
				#elif(_OPERATING==_OS_FREERTOS)
				osDelay(100);
				#endif
			}
			goto connect_to_udp_server;
		}
	}
	else
	{
		ret = 0;
	}

	return ret;
}

INT32S udp_server_disable(void)
{
	INT32U cnt = UDP_SERVER_WAIT_TIMEOUT;
	INT32S ret = -1;

	gspi_send_docmd(GP_CMD_DISABLE_UDP_SERVER);
	while((gp_net_app_state.udpsrv_state != UDP_SERVER_DISABLE_STATE) && cnt)
	{
		cnt--;
		#if(_OPERATING==_OS_UCOS2)
		OSTimeDly(100);
		#elif(_OPERATING==_OS_FREERTOS)
		osDelay(100);
		#endif
	}
	if(gp_net_app_state.udpsrv_state == UDP_SERVER_DISABLE_STATE)
	{
		ret = 0;
	}

	return ret;
}


void udp_server_task(void *parm)
{
	INT32U msg_id;
	INT8U err;
	//char* ptr;
	//INT32U i;
	#if(_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent event;
	#endif

	DBG_PRINT("Enter UDP server task...\r\n");

	while(1)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg_id = (INT32U) OSQPend(udp_server_q, 0, &err);
		if(err != OS_NO_ERR)
			continue;
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        event = osMessageGet(udp_server_q, osWaitForever);
        msg_id = event.value.v;
        if (event.status != osEventMessage)
            continue;
        #endif

		switch(msg_id)
		{
			case GSPI_US_RX_CLIINFO_EVENT:
				DBG_PRINT("Remote IP address %d.%d.%d.%d connected\r\n", udp_client_info.sa_data[2], udp_client_info.sa_data[3], udp_client_info.sa_data[4], udp_client_info.sa_data[5]);
#if 0
				ptr = (char*)&udp_client_info;
				for(i=0; i<sizeof(udp_client_info); i++)
				{
					DBG_PRINT("0x%x\r\n", *ptr++);
				}
#endif
				#if(_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(udp_server_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(udp_server_rx_sem);
				#endif
				break;

			case GSPI_US_RX_LEN_EVENT:
				/* Nothing to do */
				#if(_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(udp_server_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(udp_server_rx_sem);
				#endif
				break;

			case GSPI_US_RX_DATA_EVENT:
				DBG_PRINT("UDPSRV RX[%d]:\r\n%s\r\n", udpsrv_rx_len, udp_server_rx_buf);
				#if(_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(udp_server_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(udp_server_rx_sem);
				#endif
#if 0
				udp_server_tx_buf[0] = 'G';
				udp_server_tx_buf[1] = 'E';
				udp_server_tx_buf[2] = 'N';
				udp_server_tx_buf[3] = 'E';
				udp_server_tx_buf[4] = 'P';
				udp_server_tx_buf[5] = 'L';
				udp_server_tx_buf[6] = 'U';
				udp_server_tx_buf[7] = 'S';
				udp_server_tx_buf[8] = '_';
				udp_server_tx_buf[9] = 'U';
				udp_server_tx_buf[10] = 'D';
				udp_server_tx_buf[11] = 'P';
				udp_server_tx_buf[12] = '_';
				udp_server_tx_buf[13] = 'S';
				udp_server_tx_buf[14] = 'E';
				udp_server_tx_buf[15] = 'R';
				udp_server_tx_buf[16] = 'V';
				udp_server_tx_buf[17] = 'E';
				udp_server_tx_buf[18] = 'R';
				udp_server_tx_buf[19] = '_';
				udp_server_tx_buf[20] = 'T';
				udp_server_tx_buf[21] = 'E';
				udp_server_tx_buf[22] = 'S';
				udp_server_tx_buf[23] = 'T';
				udp_server_tx_buf[24] = '\\';
				udp_server_tx_buf[25] = 'r';
				udp_server_tx_buf[26] = '\\';
				udp_server_tx_buf[27] = 'n';
				gspi_transfer_udpsrv_socket((INT8U*)&udp_client_info, sizeof(udp_client_info), udp_server_tx_buf, 28);
#endif
				break;

			case GSPI_US_TX_DONE_EVENT:
				DBG_PRINT("GSPI_US_TX_DONE_EVENT\r\n");
				break;

			case GSPI_US_SOCKET_ERROR_EVENT:
				DBG_PRINT("GSPI_US_SOCKET_ERROR_EVENT\r\n");
				break;

			default:
				break;
		}
	}

	if(udp_server_q){
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQDel(udp_server_q, OS_DEL_ALWAYS, &err);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		#endif
		udp_server_q = NULL;
	}
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(DOOR_BELL_PRIORITY);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	id = osThreadGetId();
	osThreadTerminate(id);
	#endif

}

void udp_server_task_init(void)
{
	INT8U err;

	if(udpsrv_first_init)
		return;

	/* Task only init once */
	udpsrv_first_init = 1;

	/* Register call back function for GSPI UDP server */
	gspi_register_app_cbk((INT32U)udp_server_gspi_cbk, GSPI_UDP_SERVER_APP);

	if(udp_server_q == NULL)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		udp_server_q = OSQCreate(udp_server_q_stack, UDP_SERVER_Q_NUM);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		udp_server_q = osMessageCreate(&udp_server_q_d, NULL);
		#endif
		if(udp_server_q == 0)
		{
			DBG_PRINT("Create udp_server_q failed\r\n");
			return;
		}

	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	udp_server_rx_sem = OSSemCreate(1);

	err = OSTaskCreate(udp_server_task, (void *)NULL, &UDPServerTaskStack[UDP_SERVER_STACK_SIZE - 1], DOOR_BELL_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Cannot create UDP server task\n\r");
		return;
	}
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	udp_server_rx_sem = osSemaphoreCreate(&udp_server_rx_sem_d, 1);
	osThreadDef_t udp_server_task_ent = {"udp_server_task_entry", (os_pthread) udp_server_task, osPriorityNormal, 1, UDP_SERVER_STACK_SIZE};
	id = osThreadCreate(&udp_server_task_ent, (void*)NULL);
	if (id = 0)
        {
		DBG_PRINT("Cannot create UDP server task\n\r");
		return;
        }

	#endif
}
