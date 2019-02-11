/************************************************************
* door_bell.c
*
* Purpose: Door Bell APP
*
* Author: Eugene Hsu
*
* Date: 2015/10/27
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
#include "door_bell.h"
#include "wifi_demo.h"
#include "drv_l2_display.h"

/**********************************************
*	Definitions
**********************************************/
#define DOORBELL_STACK_SIZE			1024
#define DOORBELL_KEY_STACK_SIZE		512
#define DOOR_BEEL_Q_NUM				16
#define DOOR_BELL_WAIT_TIMEOUT		60		/* Wait 60 seconds to connect server */
#define DOOR_BELL_TX_BUF_SIZE		4096
#define DOOR_BELL_RX_BUF_SIZE		2048
#define DOOR_BELL_RECONNECT_TIME	200
#define DOOR_BELL_DELAY_TIME		100


#define DB_BUF_IDLE_STATE		0
#define DB_BUF_READY_STATE		1
#define DB_BUF_GSPI_TX_STATE	2

#define DOOR_BELL_JPEG_WIDTH		320
#define DOOR_BELL_JPEG_HEIGHT		240
#define DOOR_BELL_FPS				30
#define DOOR_BELL_Q					0
#define C_DISPLAY_DEV_HPIXEL        320
#define C_DISPLAY_DEV_VPIXEL        240

typedef enum
{
	DB_TX_SEND_JPEG_HEADER_STA = 0,
	DB_TX_SEND_JPEG_DATA_STA,
	DB_TX_SEND_JPEG_END_STA,
	DB_TX_SEND_JPEG_DONE_STA
} DB_TX_SEND_STA_E;

#define DOOR_BELL_TCP_KEY		IO_A14
#define DOOR_BELL_UDP_KEY		IO_A13
/**********************************************
*	Extern variables and functions
**********************************************/
#if(_OPERATING_SYSTEM == _OS_UCOS2)
extern OS_EVENT *video_stream_q;
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
extern osMessageQId video_stream_q;
#endif
/*********************************************
*	Variables declaration
*********************************************/
#if(_OPERATING_SYSTEM == _OS_UCOS2)
OS_EVENT *door_bell_q = NULL;
OS_EVENT *door_bell_tcp_rx_sem;
OS_EVENT *door_bell_udp_rx_sem;
static void *door_bell_q_stack[DOOR_BEEL_Q_NUM];
static INT32U DoorBellTaskStack[DOORBELL_STACK_SIZE];
static INT32U DoorBellKeyTaskStack[DOORBELL_KEY_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
osMessageQDef_t door_bell_q_d = {DOOR_BEEL_Q_NUM, sizeof(INT32U), 0 };
osMessageQId  	door_bell_q = NULL;
osSemaphoreDef_t door_bell_tcp_rx_sem_d = {0};
osSemaphoreDef_t door_bell_udp_rx_sem_d = {0};
osSemaphoreId door_bell_tcp_rx_sem;
osSemaphoreId door_bell_udp_rx_sem;

#endif

static INT32U db_buf_addr = 0;
static INT32U db_buf_remain = 0;
static INT32U db_buf_tx_cnt = 0;
static INT32U db_end_header_send = 0;
static INT32U db_buf_sta = DB_BUF_IDLE_STATE;
static INT32U db_tx_sta = DB_TX_SEND_JPEG_HEADER_STA;
static INT32U db_tcp_rx_len = 0;
static INT32U db_udp_rx_len = 0;
static INT8U db_tcp_key_status = 0;
static INT8U db_udp_key_status = 0;
static INT8U db_camera_status = 0;
static volatile INT32U db_update_addr = 0;
static volatile INT32U db_update_len = 0;
static char enable_atcmd[32] = {0};
char door_bell_host_ip[24] = {0};				/* Host IP */
static INT8U door_bell_tcp_tx_buf[DOOR_BELL_TX_BUF_SIZE+4] = {0};	/* +4 for add end character */
static INT8U door_bell_tcp_rx_buf[DOOR_BELL_RX_BUF_SIZE+4] = {0};
//static INT8U door_bell_udp_tx_buf[DOOR_BELL_TX_BUF_SIZE+4] = {0};
static INT8U door_bell_udp_rx_buf[DOOR_BELL_RX_BUF_SIZE+4] = {0};

static DOOR_BELL_UDP_COMMAND_T db_udp_cmd = {0};
static INT32U db_nonce = 0;
static INT32U db_first_init = 0;

extern INT32U (*videnc_display)(INT16U w, INT16U h, INT32U frame_addr);
void gp_doll_bell_display_callback(INT32U (*disp)(INT16U, INT16U, INT32U))
{
	if(disp) {
		videnc_display = disp;
	}
}

static INT32U Display_Callback(INT16U w, INT16U h, INT32U addr)
{
    R_TFT_FBI_ADDR = addr;
}

static void door_bell_video_encode_start(void)
{
	VIDEO_ARGUMENT arg;

//	tft_init();
//	tft_start(C_DISPLAY_DEVICE);
    drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);
    gp_doll_bell_display_callback(Display_Callback);

//	user_defined_video_codec_entrance();
	video_encode_entrance();     //Initialize and allocate the resources needed by Video decoder

	arg.bScaler = 1;
	arg.TargetWidth = DOOR_BELL_JPEG_WIDTH;
	arg.TargetHeight = DOOR_BELL_JPEG_HEIGHT;
	arg.SensorWidth	= 640;
	arg.SensorHeight = 480;
	arg.DisplayWidth = C_DISPLAY_DEV_HPIXEL;		//display width
	arg.DisplayHeight = C_DISPLAY_DEV_VPIXEL;		//display height
	arg.DisplayBufferWidth = C_DISPLAY_DEV_HPIXEL;	//display buffer width
	arg.DisplayBufferHeight = C_DISPLAY_DEV_VPIXEL;	//display buffer height

	arg.VidFrameRate = DOOR_BELL_FPS;
	arg.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV; //for display
//	arg.JPEG_Q = DOOR_BELL_Q;
	video_encode_stream_start(arg);
}

static void door_bell_video_encode_stop(void)
{
//	tft_init();
    drv_l2_display_init();

	video_encode_exit();

	video_encode_stream_stop();
}

void door_bell_gspi_tcpcli_cbk(INT8U* buf, INT32U len, INT32U event)
{
	static INT32U remain, cur_cnt;
	INT32U msg = event;
	INT32U copy_cnt;
	INT32U send_q = 1;
	INT8U err;

	if(event == GSPI_TC_RX_LEN_EVENT)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSSemPend(door_bell_tcp_rx_sem, 0, &err);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osSemaphoreWait(door_bell_tcp_rx_sem, osWaitForever);
		#endif

		db_tcp_rx_len = len;
		remain = db_tcp_rx_len;
		cur_cnt = 0;
		memset((void*)door_bell_tcp_rx_buf, 0, DOOR_BELL_RX_BUF_SIZE);
	}
	else if(event == GSPI_TC_RX_DATA_EVENT)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSSemPend(door_bell_tcp_rx_sem, 0, &err);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osSemaphoreWait(door_bell_tcp_rx_sem, osWaitForever);
		#endif

		if(remain)
		{
			copy_cnt = remain;
			if(copy_cnt > DOOR_BELL_RX_BUF_SIZE)
			{
				copy_cnt = DOOR_BELL_RX_BUF_SIZE;
			}
			memcpy((char*)(door_bell_tcp_rx_buf + cur_cnt), buf, copy_cnt);

			remain -= copy_cnt;
			cur_cnt += copy_cnt;

			if(remain)
			{
				send_q = 0;
			}
		}
	}
	else if(event == GSPI_TC_TX_DONE_EVENT)
	{
		//DBG_PRINT("GSPI_TC_TX_DONE_EVENT\r\n");
	}

	if(send_q){
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(door_bell_q, (void*)msg);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut(door_bell_q, (uint32_t) &msg, osWaitForever);
		#endif
    }
}

void door_bell_gspi_udpcli_cbk(INT8U* buf, INT32U len, INT32U event)
{
	static INT32U remain, cur_cnt;
	INT32U msg = event;
	INT32U copy_cnt;
	INT32U send_q = 1;
	INT8U err;

	if(event == GSPI_UC_RX_LEN_EVENT)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSSemPend(door_bell_udp_rx_sem, 0, &err);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osSemaphoreWait(door_bell_udp_rx_sem, osWaitForever);
		#endif

		db_udp_rx_len = len;
		remain = db_udp_rx_len;
		cur_cnt = 0;
		memset((void*)door_bell_udp_rx_buf, 0, DOOR_BELL_RX_BUF_SIZE);
	}
	else if(event == GSPI_UC_RX_DATA_EVENT)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSSemPend(door_bell_udp_rx_sem, 0, &err);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osSemaphoreWait(door_bell_udp_rx_sem, osWaitForever);
		#endif

		if(remain)
		{
			copy_cnt = remain;
			if(copy_cnt > DOOR_BELL_RX_BUF_SIZE)
			{
				copy_cnt = DOOR_BELL_RX_BUF_SIZE;
			}
			memcpy((char*)(door_bell_udp_rx_buf + cur_cnt), buf, copy_cnt);

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
		//DBG_PRINT("Unknown udp event %d\r\n", event);
	}

	if(send_q){
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(door_bell_q, (void*)msg);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut(door_bell_q, (uint32_t) &msg, osWaitForever);
		#endif
    }
}

void door_bell_update_current_jpeg(INT8U* buf, INT32U len)
{
	INT32U msg;

	if(db_buf_sta == DB_BUF_READY_STATE)
	{
		db_buf_sta = DB_BUF_GSPI_TX_STATE;	/* Update JPEG buffer state */
		msg = GSPI_TC_UPDATE_FRAME_EVENT;
		db_update_addr = (INT32U)buf;
		db_update_len = len;
		DBG_PRINT("Update db JPEG len %d, addr 0x%x\r\n", len, db_update_addr);
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(door_bell_q, (void*)msg);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut(door_bell_q, (uint32_t) &msg, osWaitForever);
		#endif

	}
	else
	{
		//DBG_PRINT("busy buf 0x%x\r\n", buf);
		avi_encode_post_empty(video_stream_q, (INT32U)buf);
	}
}

void door_bell_stop_current_jpeg(void)
{
	db_buf_sta = DB_BUF_IDLE_STATE;
	//DBG_PRINT("Stop JPEG, buf 0x%x\r\n", db_buf_addr);
	db_buf_remain = 0;
	db_tx_sta = DB_TX_SEND_JPEG_DONE_STA;
	if(db_buf_addr)
	{
		avi_encode_post_empty(video_stream_q, (INT32U)db_buf_addr);
		db_buf_addr = 0;
	}
}

static void _db_send_jpeg_header(INT32U header)
{
	INT32U transfer_len;

	memset(door_bell_tcp_tx_buf, 0, DOOR_BELL_TX_BUF_SIZE);

	if(header == 1)
	{
		INT32U copy_len;
		char* start_addr = NULL;
		sprintf((char*)door_bell_tcp_tx_buf, PICTURE_HEADER1 "Content-Length: %d\r\n" "Host: %s:%s\r\n" PICTURE_HEADER2 PICTURE_HEADER3, (db_update_len + strlen(PICTURE_HEADER3) + strlen(PICTURE_END)), door_bell_host_ip, DOOR_BELL_HOST_TCP_PORT);
		transfer_len = strlen((char*)door_bell_tcp_tx_buf);

		/* Let door_bell_tcp_tx_buf full with JPEG and header data */
		copy_len = DOOR_BELL_TX_BUF_SIZE - transfer_len;
		if(db_buf_remain <= copy_len)
			copy_len = db_buf_remain;
		start_addr = (char*)(door_bell_tcp_tx_buf + transfer_len);
		memcpy(start_addr, (char*)db_buf_addr, copy_len);
		db_buf_tx_cnt += copy_len;
		db_buf_remain -= copy_len;
		//transfer_len = DOOR_BELL_TX_BUF_SIZE;
		transfer_len += copy_len;
		db_tx_sta = DB_TX_SEND_JPEG_DATA_STA;
		gspi_transfer_tcpcli_socket(door_bell_tcp_tx_buf, transfer_len);
		DBG_PRINT("_db_send_jpeg_header: len %d\r\n", transfer_len);
	}
	else if(header == 0 && db_end_header_send == 0)
	{
		transfer_len = strlen((char*)PICTURE_END);
		strncpy((char*)door_bell_tcp_tx_buf, PICTURE_END, strlen(PICTURE_END));
		gspi_transfer_tcpcli_socket(door_bell_tcp_tx_buf, transfer_len);
		DBG_PRINT("_db_send_jpeg_header end: len %d\r\n", transfer_len);
	}
}

static void _db_send_jpeg_data(void)
{
	INT32U copy_cnt;
	INT32U end_header_len, remain;

	if(db_buf_remain)
	{
		memset(door_bell_tcp_tx_buf, 0, DOOR_BELL_TX_BUF_SIZE);

		if(db_buf_remain > DOOR_BELL_TX_BUF_SIZE)
			copy_cnt = DOOR_BELL_TX_BUF_SIZE;
		else
			copy_cnt = db_buf_remain;

		memcpy((void*)door_bell_tcp_tx_buf, (void*)(db_buf_addr + db_buf_tx_cnt), copy_cnt);
		door_bell_tcp_tx_buf[copy_cnt+1] = 0;
		db_buf_tx_cnt += copy_cnt;
		db_buf_remain -= copy_cnt;

		/* Calculate if remain space is enough for end header data size */
		remain = DOOR_BELL_TX_BUF_SIZE - copy_cnt;
		end_header_len = strlen((char*)PICTURE_END);

		if(end_header_len < remain)
		{
			char* start_addr = NULL;

			start_addr = (char*)(door_bell_tcp_tx_buf + copy_cnt);
			memcpy(start_addr, (char*)PICTURE_END, end_header_len);
			db_end_header_send = 1;
			DBG_PRINT("_db_send_jpeg_data end: len %d\r\n", (copy_cnt + end_header_len));
			gspi_transfer_tcpcli_socket(door_bell_tcp_tx_buf, (copy_cnt + end_header_len));
		}
		else
		{
			DBG_PRINT("_db_send_jpeg_data: len %d\r\n", copy_cnt);
			gspi_transfer_tcpcli_socket(door_bell_tcp_tx_buf, copy_cnt);
		}
	}
}

static void _db_send_udp_command(INT32U cmd)
{
	memset((void*)&db_udp_cmd, 0, sizeof(DOOR_BELL_UDP_COMMAND_T));

	db_udp_cmd.magic[0] = DOOR_BELL_CMD_MAGIC_1;
	db_udp_cmd.magic[1] = DOOR_BELL_CMD_MAGIC_2;
	db_udp_cmd.magic[2] = DOOR_BELL_CMD_MAGIC_3;
	db_udp_cmd.magic[3] = DOOR_BELL_CMD_MAGIC_4;

	switch(cmd)
	{
		case DOOR_BELL_REGX_CMD:
			db_udp_cmd.cmd[0] = 'R';
			db_udp_cmd.cmd[1] = 'E';
			db_udp_cmd.cmd[2] = 'G';
			db_udp_cmd.cmd[3] = 'X';
			break;

		case DOOR_BELL_RING_CMD:
			db_udp_cmd.cmd[0] = 'R';
			db_udp_cmd.cmd[1] = 'I';
			db_udp_cmd.cmd[2] = 'N';
			db_udp_cmd.cmd[3] = 'G';
			break;

		case DOOR_BELL_ANSW_CMD:
			db_udp_cmd.cmd[0] = 'A';
			db_udp_cmd.cmd[1] = 'N';
			db_udp_cmd.cmd[2] = 'S';
			db_udp_cmd.cmd[3] = 'W';
			break;

		case DOOR_BELL_BYEX_CMD:
			db_udp_cmd.cmd[0] = 'B';
			db_udp_cmd.cmd[1] = 'Y';
			db_udp_cmd.cmd[2] = 'E';
			db_udp_cmd.cmd[3] = 'X';
			break;

		case DOOR_BELL_AUDX_CMD:
			db_udp_cmd.cmd[0] = 'A';
			db_udp_cmd.cmd[1] = 'U';
			db_udp_cmd.cmd[2] = 'D';
			db_udp_cmd.cmd[3] = 'X';
			break;

		default:
			DBG_PRINT("_db_send_udp_command: unknown command %d\r\n", cmd);
			return;
	}

	db_udp_cmd.nonce = db_nonce++;

	db_udp_cmd.token[0] = DEVICE_TOKEN_CH1;
	db_udp_cmd.token[1] = DEVICE_TOKEN_CH2;
	db_udp_cmd.token[2] = DEVICE_TOKEN_CH3;
	db_udp_cmd.token[3] = DEVICE_TOKEN_CH4;

	/* Send command to GSPI */
	gspi_transfer_udpcli_socket((INT8U*)&db_udp_cmd, sizeof(DOOR_BELL_UDP_COMMAND_T));
}

void door_bell_set_server_ip(INT32U ip)
{
	memset((char*)door_bell_host_ip, 0, sizeof(door_bell_host_ip));
	sprintf(door_bell_host_ip, "%d.%d.%d.%d", ip&0x000000FF, (ip&0x0000FF00) >> 8, (ip&0x00FF0000) >> 16, (ip&0xFF000000) >> 24);
	DBG_PRINT("Door Bell server IP:%s\r\n", door_bell_host_ip);
}

INT32S door_bell_connect_tcp_server(void)
{
	INT32S ret = -1;
	INT32U retry = 10, cnt;
	INT32U port;
	char buf[8];

connect_to_tcp_server:

	cnt = DOOR_BELL_WAIT_TIMEOUT;
	/* Enable TCP client to connect TCP server */
//m freertos Timer?
    #if(_OPERATING_SYSTEM == _OS_UCOS2)
	port = OSTimeGet()%(65536-DOOR_BELL_CLIENT_TCP_PORT);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	port = (xTaskGetTickCount()*10) % (65536-DOOR_BELL_CLIENT_TCP_PORT);
	#endif
	port += DOOR_BELL_CLIENT_TCP_PORT;
	sprintf(buf, "%d", port);
	DBG_PRINT("door_bell_connect_tcp_server:%d\r\n",port);
	sprintf(enable_atcmd, GP_CMD_ENABLE_TCP_CLIENT "%s:" DOOR_BELL_HOST_TCP_PORT ":%s", door_bell_host_ip, buf);

	gspi_send_docmd(enable_atcmd);

	while(((gp_net_app_state.tcpcli_state != TCP_CLIENT_CONNECT_READY_STATE) && cnt) && gp_net_app_state.tcpcli_state != TCP_CLIENT_ERROR_STATE)
	{
		cnt--;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(DOOR_BELL_DELAY_TIME);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay(DOOR_BELL_DELAY_TIME*10);
        #endif
	}

	if(gp_net_app_state.tcpcli_state == TCP_CLIENT_ERROR_STATE)
	{
		retry--;
		DBG_PRINT("Enable TCP client failed, retry %d\r\n", retry);
		if(retry)
		{
			cnt = DOOR_BELL_WAIT_TIMEOUT;
			gspi_send_docmd(GP_CMD_DISABLE_TCP_CLIENT);

			while((gp_net_app_state.tcpcli_state != TCP_CLIENT_DISABLE_STATE) && cnt)
			{
				cnt--;
                #if (_OPERATING_SYSTEM == _OS_UCOS2)
                OSTimeDly(DOOR_BELL_DELAY_TIME);
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                osDelay(DOOR_BELL_DELAY_TIME*10);
                #endif
			}
			goto connect_to_tcp_server;
		}
	}
	else
	{
		ret = 0;
	}

	return ret;
}

INT32S door_bell_disconnect_tcp_server(void)
{
	INT32U cnt = DOOR_BELL_WAIT_TIMEOUT;
	INT32S ret = -1;

	gspi_send_docmd(GP_CMD_DISABLE_TCP_CLIENT);
	while((gp_net_app_state.tcpcli_state != TCP_CLIENT_DISABLE_STATE) && cnt)
	{
		cnt--;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(DOOR_BELL_DELAY_TIME);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay(DOOR_BELL_DELAY_TIME*10);
        #endif
	}

	if(gp_net_app_state.tcpcli_state == TCP_CLIENT_DISABLE_STATE)
	{
		ret = 0;
	}

	return ret;
}

INT32S door_bell_connect_udp_server(void)
{
	INT32S ret = -1;
	INT32U retry = 10, cnt;

connect_to_udp_server:

	cnt = DOOR_BELL_WAIT_TIMEOUT;
	/* Enable TCP client to connect TCP server */
	sprintf(enable_atcmd, GP_CMD_ENABLE_UDP_CLIENT "%s:" DOOR_BELL_HOST_UDP_PORT, door_bell_host_ip);
	gspi_send_docmd(enable_atcmd);

	while(((gp_net_app_state.udpcli_state != UDP_CLIENT_CONNECT_READY_STATE) && cnt) && gp_net_app_state.udpcli_state != UDP_CLIENT_ERROR_STATE)
	{
		cnt--;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(DOOR_BELL_DELAY_TIME);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay(DOOR_BELL_DELAY_TIME*10);
        #endif
	}

	if(gp_net_app_state.udpcli_state == UDP_CLIENT_ERROR_STATE)
	{
		retry--;
		DBG_PRINT("Enable UDP client failed, retry %d\r\n", retry);
		if(retry)
		{
			cnt = DOOR_BELL_WAIT_TIMEOUT;
			gspi_send_docmd(GP_CMD_DISABLE_UDP_CLIENT);

			while((gp_net_app_state.udpcli_state == UDP_CLIENT_DISABLE_STATE) && cnt)
			{
				cnt--;
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSTimeDly(DOOR_BELL_DELAY_TIME);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osDelay(DOOR_BELL_DELAY_TIME*10);
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

INT32S door_bell_disconnect_udp_server(void)
{
	INT32U cnt = DOOR_BELL_WAIT_TIMEOUT;
	INT32S ret = -1;

	gspi_send_docmd(GP_CMD_DISABLE_UDP_CLIENT);
	while((gp_net_app_state.udpcli_state != UDP_CLIENT_DISABLE_STATE) && cnt)
	{
		cnt--;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(DOOR_BELL_DELAY_TIME);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay(DOOR_BELL_DELAY_TIME*10);
		#endif
	}

	if(gp_net_app_state.udpcli_state == UDP_CLIENT_DISABLE_STATE)
	{
		ret = 0;
	}

	return ret;
}

void door_bell_task(void *parm)
{
	INT32U msg_id;
	static INT32U cam_status = 0;
	INT8U err;
		#if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
	#endif


	DBG_PRINT("Enter door bell task...\r\n");

	register_update_cur_jpeg_cbk((INT32U)door_bell_update_current_jpeg);

	while(1)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		msg_id = (INT32U) OSQPend(door_bell_q, 0, &err);
		if(err != OS_NO_ERR)
			continue;
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        result = osMessageGet(door_bell_q, osWaitForever);
        msg_id = result.value.v;
        if (result.status != osEventMessage)
				continue;
        #endif
		switch(msg_id)
		{
			case GSPI_TC_UPDATE_FRAME_EVENT:
				if(!cam_status)
					break;
				/* release previous buffer */
				if(db_buf_addr)
				{
					avi_encode_post_empty(video_stream_q, db_buf_addr);
				}

				db_buf_addr = (INT32U)db_update_addr;
				db_buf_remain = db_update_len;
				db_buf_tx_cnt = 0;
				db_end_header_send = 0;
				db_tx_sta = DB_TX_SEND_JPEG_HEADER_STA;
				_db_send_jpeg_header(1);
				break;

			case GSPI_TC_TX_DONE_EVENT:
				DBG_PRINT("GSPI_TC_TX_DONE_EVENT\r\n");
				if(!cam_status)
					break;
				if(db_buf_remain)
				{
					db_tx_sta = DB_TX_SEND_JPEG_DATA_STA;
					_db_send_jpeg_data();
				}
				else if(db_tx_sta == DB_TX_SEND_JPEG_DATA_STA)
				{
					db_tx_sta = DB_TX_SEND_JPEG_END_STA;
					_db_send_jpeg_header(0);
				}
				else if(db_tx_sta == DB_TX_SEND_JPEG_END_STA)
				{
					/* Update next JPEG buffer */
					db_tx_sta = DB_TX_SEND_JPEG_DONE_STA;
				}
				break;

			case GSPI_TC_KEY_PRESS_EVENT:
				db_camera_status = !db_camera_status;
				if(db_camera_status)
				{
					DBG_PRINT("Start Door Bell Camera service\r\n");

					if(!door_bell_connect_tcp_server())
					{
						DBG_PRINT("Connect %s:TCP %s successfully\r\n", door_bell_host_ip, DOOR_BELL_HOST_TCP_PORT);
					}

					if(cam_status == 0)
					{
						door_bell_video_encode_start();
						db_buf_sta = DB_BUF_READY_STATE;
						cam_status = 1;
						//Test for UDP transfer
						//msg = GSPI_UC_KEY_PRESS_EVENT;
						//OSQPost(door_bell_q, (void*)msg);
					}
				}
				else
				{
					DBG_PRINT("Stop Door Bell Camera service\r\n");
					if(cam_status)
					{
						door_bell_stop_current_jpeg();
						door_bell_video_encode_stop();
						cam_status = 0;
					}
					if(door_bell_disconnect_tcp_server())
					{
						DBG_PRINT("Disconnect server failed\r\n");
					}
				}
				break;

			case GSPI_UC_TX_DONE_EVENT:
				DBG_PRINT("UDP TX packet done\r\n");
				//Test for UDP transfer
				//_db_send_udp_command(DOOR_BELL_RING_CMD);
				break;

			case GSPI_UC_SOCKET_ERROR_EVENT:
				DBG_PRINT("UDP TX packet error\r\n");
				break;

			case GSPI_UC_KEY_PRESS_EVENT:
				//Test for UDP transfer
				//_db_send_udp_command(DOOR_BELL_REGX_CMD);
				break;

			case GSPI_UC_RX_LEN_EVENT:
				/* Nothing to do */
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(door_bell_udp_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(door_bell_udp_rx_sem);
				#endif
				break;

			case GSPI_UC_RX_DATA_EVENT:
				{
					INT32U i;
					DBG_PRINT("UDP RX[%d]:\r\n", db_udp_rx_len);
					for(i=0; i<db_udp_rx_len; i++)
					{
						DBG_PRINT("data[%d]= 0x%x\r\n", i, door_bell_udp_rx_buf[i]);
					}
				}
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(door_bell_udp_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(door_bell_udp_rx_sem);
				#endif
				break;

			case GSPI_TC_RX_LEN_EVENT:
				/* Nothing to do */
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(door_bell_tcp_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(door_bell_tcp_rx_sem);
				#endif
				break;

			case GSPI_TC_RX_DATA_EVENT:
				if(strstr((char*)door_bell_tcp_rx_buf, HTTP_11_200_OK) && cam_status)
				{
					DBG_PRINT("Get 200 OK to send next JPEG\r\n");
					db_buf_sta = DB_BUF_READY_STATE;
				}
				else
				{
					DBG_PRINT("TCPCLI RX [%d]\r\n", strlen((char*)door_bell_tcp_rx_buf));
				}
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(door_bell_tcp_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(door_bell_tcp_rx_sem);
				#endif

				break;

			case GSPI_TC_SOCKET_ERROR_EVENT:
				DBG_PRINT("GSPI_TC_SOCKET_ERROR_EVENT\r\n");
				if(door_bell_disconnect_tcp_server())
				{
					DBG_PRINT("Disconnect server failed\r\n");
				}
				break;

			case GSPI_UC_START_EVENT:
				/* Enable UDP connection */
				if(!door_bell_connect_udp_server())
				{
					DBG_PRINT("Connect %s:UDP %s successfully\r\n", door_bell_host_ip, DOOR_BELL_HOST_UDP_PORT);
					//Test for UDP transfer
					//msg = GSPI_TC_KEY_PRESS_EVENT;
					//OSQPost(door_bell_q, (void*)msg);
				}
				break;

			default:
				break;
		}
	}

	if(door_bell_q)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQDel(door_bell_q, OS_DEL_ALWAYS, &err);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)

        #endif
		door_bell_q = NULL;
	}
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(DOOR_BELL_PRIORITY);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	id = osThreadGetId();
	osThreadTerminate(id);
	#endif
}

void door_bell_key_task(void *parm)
{
	static INT32U tcpcnt = 0, udpcnt = 0;
	INT32U msg;

	while(1)
	{
		db_tcp_key_status = gpio_read_io(DOOR_BELL_TCP_KEY);
		db_udp_key_status = gpio_read_io(DOOR_BELL_UDP_KEY);

		if(db_tcp_key_status)
		{
			if(tcpcnt++ == 5)
			{
				DBG_PRINT("Door bell TCP key pressed\r\n");
				msg = GSPI_TC_KEY_PRESS_EVENT;
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(door_bell_q, (void*)msg);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osMessagePut(door_bell_q, (uint32_t)&msg, osWaitForever);
				#endif
			}
		}
		else
		{
			tcpcnt = 0;
		}

		if(db_udp_key_status)
		{
			if(udpcnt++ == 5)
			{
				DBG_PRINT("Door bell UDP key pressed\r\n");
				msg = GSPI_UC_KEY_PRESS_EVENT;
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSQPost(door_bell_q, (void*)msg);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osMessagePut(door_bell_q, (uint32_t)&msg, osWaitForever);
				#endif
			}
		}
		else
		{
			udpcnt = 0;
		}
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(20);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay(20*10);
		#endif
	}
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSTaskDel(DOOR_BELL_KEY_PRIORITY);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osThreadId id;
	id = osThreadGetId();
	osThreadTerminate(id);
	#endif
}

void door_bell_init_key(void)
{
	gpio_init_io(DOOR_BELL_TCP_KEY, GPIO_INPUT);
	gpio_set_port_attribute(DOOR_BELL_TCP_KEY, 0);

	gpio_init_io(DOOR_BELL_UDP_KEY, GPIO_INPUT);
	gpio_set_port_attribute(DOOR_BELL_UDP_KEY, 0);
}

void door_bell_task_init(void)
{
	INT8U err;

	if(db_first_init)
		return;

	/* Task only init once */
	db_first_init = 1;

	/* Register call back function for GSPI TCP client */
	gspi_register_app_cbk((INT32U)door_bell_gspi_tcpcli_cbk, GSPI_TCP_CLIENT_APP);

	/* Register call back function for GSPI UDP client */
	gspi_register_app_cbk((INT32U)door_bell_gspi_udpcli_cbk, GSPI_UDP_CLIENT_APP);

	if(door_bell_q == NULL)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		door_bell_q = OSQCreate(door_bell_q_stack, DOOR_BEEL_Q_NUM);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		door_bell_q = osMessageCreate(&door_bell_q_d, NULL);
		#endif
        if(door_bell_q == 0)
		{
			DBG_PRINT("Create door_bell_q failed\r\n");
			return;
		}

	}

	door_bell_init_key();

    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	door_bell_tcp_rx_sem = OSSemCreate(1);
	door_bell_udp_rx_sem = OSSemCreate(1);

	err = OSTaskCreate(door_bell_key_task, (void *)NULL, &DoorBellKeyTaskStack[DOORBELL_KEY_STACK_SIZE - 1], DOOR_BELL_KEY_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Cannot create Door key Bell task\n\r");
		return;
	}


	err = OSTaskCreate(door_bell_task, (void *)NULL, &DoorBellTaskStack[DOORBELL_STACK_SIZE - 1], DOOR_BELL_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Cannot create Door Bell task\n\r");
		return;
	}
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    door_bell_tcp_rx_sem = osSemaphoreCreate(&door_bell_tcp_rx_sem_d, 1);
    door_bell_udp_rx_sem = osSemaphoreCreate(&door_bell_udp_rx_sem_d, 1);

    osThreadId id;
    osThreadDef_t door_bell_task_ent = {"door_bell_task_entry", (os_pthread) door_bell_task, osPriorityNormal, 1, DOORBELL_STACK_SIZE };
	id = osThreadCreate(&door_bell_task_ent, (void*) NULL);
	if (id ==0)
	{
		DBG_PRINT("Cannot create Door Bell task\n\r");
		return;
	}
	#endif
}
