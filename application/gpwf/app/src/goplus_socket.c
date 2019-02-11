/************************************************************
* goplus_socket.c
*
* Purpose: GOPLUS socket APP
*
* Author: Eugene Hsu
*
* Date: 2016/03/07
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
#include "application.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"

/**********************************************
*	Definitions
**********************************************/
#define GOPLUS_SOCKET_Q_NUM		16
#define GOPLUSCMD_STACK_SIZE	1024
#define GOPLUSCMD_TX_BUF_SIZE	4096	/* Maximum write 4KB buffer for GOPLUS socket */
#define GOPLUSCMD_RX_BUF_SIZE	1600

#define GP_SOCK_TAG0	'G'
#define GP_SOCK_TAG1	'P'
#define GP_SOCK_TAG2	'S'
#define GP_SOCK_TAG3	'O'
#define GP_SOCK_TAG4	'C'
#define GP_SOCK_TAG5	'K'
#define GP_SOCK_TAG6	'E'
#define GP_SOCK_TAG7	'T'

typedef enum GP_SOCK_TYPE_E
{
	GP_SOCK_CMD_TYPE = 		0x0001
} GP_SOCK_TYPE_T;

typedef enum GP_SOCK_MODE_E
{
	GP_SOCK_GENERAL_MODE = 		0x00,
	GP_SOCK_RECORD_MODE = 		0x01,
	GP_SOCK_CAP_PICTURE_MODE = 	0x02,
	GP_SOCK_PLAYBACK_MODE = 	0x03,
	GP_SOCK_MENU_MODE = 		0x04,
	GP_SOCK_FIRMWARE_MODE = 	0x05,
	GP_SOCK_VENDOR_MODE = 		0xFF
} GP_SOCK_MODE_T;

typedef enum GP_SOCK_CMD_E
{
	GP_SOCK_GENERAL_SET_MODE_CMD = 				0x00,
	GP_SOCK_GENERAL_GET_DEVICE_STATUS_CMD = 	0x01,
	GP_SOCK_GENERAL_GET_PARAMETER_FILE_CMD = 	0x02,
	GP_SOCK_GENERAL_POWER_OFF_CMD = 			0x03,
	GP_SOCK_GENERAL_RESTART_STREAMING_CMD = 	0x04,
	GP_SOCK_GENERAL_AUTH_DEVICE_CMD = 			0x05
} GP_SOCK_CMD_T;

typedef enum GOPLUS_TX_CMD_STA_E
{
	GOPLUS_TX_NONE_STA = 0,
	GOPLUS_TX_GET_PARAMETER_STA
} GOPLUS_TX_CMD_STA_T;

/* Used to parse GOPLUS cam protocol header */
typedef struct GOPLUS_CAM_CMD_S
{
	INT8U tag[8];
	INT8U type[2];
	INT8U mode[1];
	INT8U cmd[1];
	INT8U payload[128];
} PACKED GOPLUS_CAM_CMD_T;

/*********************************************
*	Variables declaration
*********************************************/
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *goplus_sock_q = NULL;
static OS_EVENT *goplus_cmd_socket_tx_sem = NULL;
static OS_EVENT *goplus_cmd_socket_rx_sem = NULL;
static void *goplus_sock_q_stack[GOPLUS_SOCKET_Q_NUM];
static INT32U GOPlusCmdTaskStack[GOPLUSCMD_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	static osSemaphoreDef_t  goplus_cmd_socket_tx_sem_d = {0};
	static osSemaphoreDef_t  goplus_cmd_socket_rx_sem_d = {0};
	static osSemaphoreId goplus_cmd_socket_tx_sem;
	static osSemaphoreId goplus_cmd_socket_rx_sem;
	static osMessageQDef_t goplus_sock_q_d = { GOPLUS_SOCKET_Q_NUM, sizeof(INT32U), 0 };
	static osMessageQId goplus_sock_q = NULL;
#endif

static INT8U goplus_rx_buf[GOPLUSCMD_RX_BUF_SIZE];
static INT8U goplus_tx_buf[GOPLUSCMD_TX_BUF_SIZE];
static INT32U goplus_rx_buf_len = 0;
static INT32U goplus_tx_cmd_state = GOPLUS_TX_NONE_STA;
static INT32U goplus_tx_data_buf = 0;
static INT32U goplus_tx_data_len = 0;
static INT32U goplus_first_init = 0;

extern INT8U GOPLUS_XML_START;
extern INT8U GOPLUS_XML_END;

/*****Function declaration*******/
static void _goplus_send_get_parameter_ack(void)
{
	GOPLUS_CAM_CMD_T* goplus_rx_cmd = (GOPLUS_CAM_CMD_T*)goplus_rx_buf;
	GOPLUS_CAM_CMD_T* goplus_tx_cmd = (GOPLUS_CAM_CMD_T*)goplus_tx_buf;
	INT32U xml_len = &GOPLUS_XML_END - &GOPLUS_XML_START;

	/* Fill tag */
	goplus_tx_cmd->tag[0] = goplus_rx_cmd->tag[0];
	goplus_tx_cmd->tag[1] = goplus_rx_cmd->tag[1];
	goplus_tx_cmd->tag[2] = goplus_rx_cmd->tag[2];
	goplus_tx_cmd->tag[3] = goplus_rx_cmd->tag[3];
	goplus_tx_cmd->tag[4] = goplus_rx_cmd->tag[4];
	goplus_tx_cmd->tag[5] = goplus_rx_cmd->tag[5];
	goplus_tx_cmd->tag[6] = goplus_rx_cmd->tag[6];
	goplus_tx_cmd->tag[7] = goplus_rx_cmd->tag[7];

	/* Fill ACK/NAK(0x0002/0x0003) type */
	goplus_tx_cmd->type[0] = 0x02;
	goplus_tx_cmd->type[1] = 0x00;

	/* Fill mode and command */
	goplus_tx_cmd->mode[0] = GP_SOCK_GENERAL_MODE;
	goplus_tx_cmd->cmd[0] = GP_SOCK_GENERAL_GET_PARAMETER_FILE_CMD;

	/* Fill payload size */
	goplus_tx_cmd->payload[0] = (xml_len & 0xff);
	goplus_tx_cmd->payload[1] = (xml_len & 0xff00) >> 8;

	gspi_tx_data(goplus_tx_buf, 14, DATA_GOPLUS_CMD_TYPE);

	/* Wait for GOPLUS_SOCK_CMD_TXDONE_EVENT to send XML file */
	goplus_tx_cmd_state = GOPLUS_TX_GET_PARAMETER_STA;
	goplus_tx_data_buf = (INT32U)&GOPLUS_XML_START;
	goplus_tx_data_len = xml_len;
}

static void _goplus_send_getstatus_ack(void)
{
	GOPLUS_CAM_CMD_T* goplus_rx_cmd = (GOPLUS_CAM_CMD_T*)goplus_rx_buf;
	GOPLUS_CAM_CMD_T* goplus_tx_cmd = (GOPLUS_CAM_CMD_T*)goplus_tx_buf;

	/* Fill tag */
	goplus_tx_cmd->tag[0] = goplus_rx_cmd->tag[0];
	goplus_tx_cmd->tag[1] = goplus_rx_cmd->tag[1];
	goplus_tx_cmd->tag[2] = goplus_rx_cmd->tag[2];
	goplus_tx_cmd->tag[3] = goplus_rx_cmd->tag[3];
	goplus_tx_cmd->tag[4] = goplus_rx_cmd->tag[4];
	goplus_tx_cmd->tag[5] = goplus_rx_cmd->tag[5];
	goplus_tx_cmd->tag[6] = goplus_rx_cmd->tag[6];
	goplus_tx_cmd->tag[7] = goplus_rx_cmd->tag[7];

	/* Fill ACK/NAK(0x0002/0x0003) type */
	goplus_tx_cmd->type[0] = 0x02;
	goplus_tx_cmd->type[1] = 0x00;

	/* Fill mode and command */
	goplus_tx_cmd->mode[0] = GP_SOCK_GENERAL_MODE;
	goplus_tx_cmd->cmd[0] = GP_SOCK_GENERAL_GET_DEVICE_STATUS_CMD;

	/* Fill payload size */
	goplus_tx_cmd->payload[0] = 0x0E;
	goplus_tx_cmd->payload[1] = 0x00;

	/* Fill payload size */
	goplus_tx_cmd->payload[2] = 0x00;
	goplus_tx_cmd->payload[3] = 0x02;
	goplus_tx_cmd->payload[4] = 0x04;
	goplus_tx_cmd->payload[5] = 0x01;
	goplus_tx_cmd->payload[6] = 0x02;
	goplus_tx_cmd->payload[7] = 0x07;
	goplus_tx_cmd->payload[8] = 0x0C;
	goplus_tx_cmd->payload[9] = 0x00;
	goplus_tx_cmd->payload[10] = 0x00;
	goplus_tx_cmd->payload[11] = 0x05;
	goplus_tx_cmd->payload[12] = 0x00;
	goplus_tx_cmd->payload[13] = 0x00;
	goplus_tx_cmd->payload[14] = 0x00;
	goplus_tx_cmd->payload[15] = 0x00;

	gspi_tx_data(goplus_tx_buf, 28, DATA_GOPLUS_CMD_TYPE);
}

static void _goplus_send_general_ack(void)
{
	GOPLUS_CAM_CMD_T* goplus_rx_cmd = (GOPLUS_CAM_CMD_T*)goplus_rx_buf;
	GOPLUS_CAM_CMD_T* goplus_tx_cmd = (GOPLUS_CAM_CMD_T*)goplus_tx_buf;

	/* Fill tag */
	goplus_tx_cmd->tag[0] = goplus_rx_cmd->tag[0];
	goplus_tx_cmd->tag[1] = goplus_rx_cmd->tag[1];
	goplus_tx_cmd->tag[2] = goplus_rx_cmd->tag[2];
	goplus_tx_cmd->tag[3] = goplus_rx_cmd->tag[3];
	goplus_tx_cmd->tag[4] = goplus_rx_cmd->tag[4];
	goplus_tx_cmd->tag[5] = goplus_rx_cmd->tag[5];
	goplus_tx_cmd->tag[6] = goplus_rx_cmd->tag[6];
	goplus_tx_cmd->tag[7] = goplus_rx_cmd->tag[7];

	/* Fill ACK/NAK(0x0002/0x0003) type */
	goplus_tx_cmd->type[0] = 0x02;
	goplus_tx_cmd->type[1] = 0x00;

	/* Fill mode and command */
	goplus_tx_cmd->mode[0] = goplus_rx_cmd->mode[0];
	goplus_tx_cmd->cmd[0] = goplus_rx_cmd->cmd[0];

	/* Fill payload size */
	goplus_tx_cmd->payload[0] = 0;
	goplus_tx_cmd->payload[1] = 0;

	gspi_tx_data(goplus_tx_buf, 14, DATA_GOPLUS_CMD_TYPE);
}

static void _goplus_process_cmd(INT8U* buf, INT32U len)
{
	GOPLUS_CAM_CMD_T* goplus_cmd = (GOPLUS_CAM_CMD_T*)buf;

	if(goplus_cmd->tag[0] == GP_SOCK_TAG0 && goplus_cmd->tag[1] == GP_SOCK_TAG1 && goplus_cmd->tag[2] == GP_SOCK_TAG2 && goplus_cmd->tag[3] == GP_SOCK_TAG3
		&& goplus_cmd->tag[4] == GP_SOCK_TAG4 && goplus_cmd->tag[5] == GP_SOCK_TAG5 && goplus_cmd->tag[6] == GP_SOCK_TAG6 && goplus_cmd->tag[7] == GP_SOCK_TAG7)
	{
		/* This is a GOPLUS command */
		if((goplus_cmd->type[0] | (goplus_cmd->type[1] << 8)) == GP_SOCK_CMD_TYPE)
		{
			if(goplus_cmd->mode[0] == GP_SOCK_GENERAL_MODE)
			{
				if(goplus_cmd->cmd[0] == GP_SOCK_GENERAL_GET_PARAMETER_FILE_CMD)
				{
					//printf("Get file list\r\n");
					_goplus_send_get_parameter_ack();
				}
				else if(goplus_cmd->cmd[0] == GP_SOCK_GENERAL_GET_DEVICE_STATUS_CMD)
				{
					//printf("Get status cmd\r\n");
					_goplus_send_getstatus_ack();
				}
				else
				{
					_goplus_send_general_ack();
				}
			}
		}
	}
}

void goplus_cmd_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
{
	INT32U msg = event;
	INT32U size = len;
	INT8U err;

	if(event == GOPLUS_SOCK_CMD_RXDONE_EVENT)
	{
        #if(_OPERATING_SYSTEM==_OS_UCOS2)
		OSSemPend(goplus_cmd_socket_rx_sem, 0, &err);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        osSemaphoreWait(goplus_cmd_socket_rx_sem, osWaitForever);
        #endif

		if(len)
		{
			if(size > GOPLUSCMD_RX_BUF_SIZE)
				size = GOPLUSCMD_RX_BUF_SIZE;

			//DBG_PRINT("Got GOPLUS buf len %d\r\n", len);
			goplus_rx_buf_len = size;
			memcpy(goplus_rx_buf, buf, goplus_rx_buf_len);
		}
	}
	else if(event == GOPLUS_SOCK_CMD_TXDONE_EVENT)
	{
		//DBG_PRINT("GOPLUS TX DONE EVENT\r\n");
	}

	else if(event == GOPLUS_SOCK_CMD_TXERROR_EVENT)
	{

	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(goplus_sock_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(goplus_sock_q, (uint32_t)&msg, osWaitForever);
	#endif
}

void goplus_cmd_socket_task(void *parm)
{
	INT32U msg_id;
	INT8U err;
    #if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif

	DBG_PRINT("Enter GOPLUS command socket task...\r\n");

	while(1)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg_id = (INT32U) OSQPend(goplus_sock_q, 0, &err);
		if(err != OS_NO_ERR)
			continue;
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    	result = osMessageGet(goplus_sock_q, osWaitForever);
		msg_id = result.value.v;
		if (result.status != osEventMessage)
			continue;
        #endif

		switch(msg_id)
		{
			case GOPLUS_SOCK_CMD_CLI_CONN_EVENT:
				DBG_PRINT("GOPLUS socket Client connected\r\n");
				break;

			case GOPLUS_SOCK_CMD_CLI_DISC_EVENT:
				DBG_PRINT("GOPLUS socket client disconnected\r\n");
				break;

			case GOPLUS_SOCK_CMD_RXDONE_EVENT:
				DBG_PRINT("GOPLUS RX len[%d]\r\n", goplus_rx_buf_len);
				_goplus_process_cmd(goplus_rx_buf, goplus_rx_buf_len);
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(goplus_cmd_socket_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(goplus_cmd_socket_rx_sem);
				#endif

				break;

			case GOPLUS_SOCK_CMD_TXDONE_EVENT:
				if(goplus_tx_cmd_state == GOPLUS_TX_GET_PARAMETER_STA)
				{
					if(goplus_tx_data_len)
					{
						if(goplus_tx_data_len >= GOPLUSCMD_TX_BUF_SIZE)
						{
							gspi_tx_data((INT8U*)goplus_tx_data_buf, GOPLUSCMD_TX_BUF_SIZE, DATA_GOPLUS_CMD_TYPE);
							goplus_tx_data_buf += GOPLUSCMD_TX_BUF_SIZE;
							goplus_tx_data_len -= GOPLUSCMD_TX_BUF_SIZE;
						}
						else
						{
							gspi_tx_data((INT8U*)goplus_tx_data_buf, goplus_tx_data_len, DATA_GOPLUS_CMD_TYPE);
							goplus_tx_data_buf += goplus_tx_data_len;
							goplus_tx_data_len = 0;
						}
					}
					else
					{
						GOPLUS_CAM_CMD_T* goplus_tx_cmd = (GOPLUS_CAM_CMD_T*)goplus_tx_buf;
						/* XML file transfer completed */
						/* Fill payload size to 0 */
						goplus_tx_cmd->payload[0] = 0;
						goplus_tx_cmd->payload[1] = 0;

						/* Send 14 bytes for get file list totally */
						gspi_tx_data(goplus_tx_buf, 14, DATA_GOPLUS_CMD_TYPE);
						goplus_tx_cmd_state = GOPLUS_TX_NONE_STA;
					}
				}
				break;

			case GOPLUS_SOCK_CMD_TXERROR_EVENT:
				/* TX socket error */
				break;

			default:
				break;
		}

	}
}

void goplus_cmd_socket_task_init(void)
{
	INT8U err;

	if(goplus_first_init)
		return;

	/* Task only init once */
	goplus_first_init = 1;

	/* Register call back function for GSPI RX data */
	gspi_register_app_cbk((INT32U)goplus_cmd_gspi_cbk, GSPI_GOPLUS_CMD_APP);

	if(goplus_sock_q == NULL)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		goplus_sock_q = OSQCreate(goplus_sock_q_stack, GOPLUS_SOCKET_Q_NUM);
		if(goplus_sock_q == 0)
		{
			DBG_PRINT("Create goplus_sock_q failed\r\n");
			return;
		}
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		goplus_sock_q = osMessageCreate(&goplus_sock_q_d, NULL);
		if(goplus_sock_q == 0)
		{
			DBG_PRINT("Create goplus_sock_q failed\r\n");
			return;
		}
		#endif

	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	goplus_cmd_socket_tx_sem = OSSemCreate(1);
	goplus_cmd_socket_rx_sem = OSSemCreate(1);

	err = OSTaskCreate(goplus_cmd_socket_task, (void *)NULL, &GOPlusCmdTaskStack[GOPLUSCMD_STACK_SIZE - 1], GP_SOCKET_CMD_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Cannot create GOPLUS command socket task\n\r");
		return;
	}
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	goplus_cmd_socket_rx_sem = osSemaphoreCreate (&goplus_cmd_socket_rx_sem_d, 1);
	goplus_cmd_socket_tx_sem = osSemaphoreCreate(&goplus_cmd_socket_tx_sem_d, 1);
	osThreadId id;
	osThreadDef_t goplus_cmd_socket_task_ent = {"goplus_cmd_socket_task", (os_pthread) goplus_cmd_socket_task, osPriorityNormal, 1, GOPLUSCMD_STACK_SIZE};
	id = osThreadCreate(&goplus_cmd_socket_task_ent, (void*)NULL);
	if (id== 0)
	{
		DBG_PRINT("Cannot create GOPLUS command socket task\n\r");
		return;
	}
	#endif

}
