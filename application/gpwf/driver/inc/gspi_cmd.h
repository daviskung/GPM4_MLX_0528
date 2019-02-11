/***************************************************************************
* gspi_cmd.h
*
* Purpose: Header file for GSPI command
*
* Author: Eugene Hsu
*
* Date: 2015/10/01
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
******************************************************************************/
#ifndef __GSPI_CMD_H__
#define __GSPI_CMD_H__

#include "drv_l1_gpio.h"
#include "drv_l1_ext_int.h"


#define GSPI_MASTER_Q_NUM	16
#define GSPI_STACK_SIZE		1024
#define GSPI_DOCMD_SIZE		128
#define GSPI_DATA_TRANSFER_LEN	(2*1024)

#define WLAN0_NAME	"wlan0"
#define WLAN1_NAME	"wlan1"

typedef void(*GSPI_APP_CBK)(INT8U* buf, INT32U len, INT32U event);

typedef enum GSPI_APP_E
{
	GSPI_GP_CMD_APP,
	GSPI_GP_SOCK_CMD_APP,
	GSPI_MJPEG_STREAM_APP,
	GSPI_WEBSOCKET_APP,
	GSPI_VIDEO_STREAM_APP,
	GSPI_AUDIO_STREAM_APP,
	GSPI_GOPLUS_CMD_APP,
	GSPI_TCP_CLIENT_APP,
	GSPI_UDP_CLIENT_APP,
	GSPI_UDP_SERVER_APP,
	GSPI_SSLCLI_APP
} GSPI_APP_T;

typedef enum GSPI_STATE_E
{
	GSPI_IDLE_STATE = 0,
	GSPI_TX_DATA_STATE,
	GSPI_RX_DATA_STATE
} GSPI_STATE_T;

typedef enum GSPI_EVENT_E
{
	GSPI_MJPEG_TX_EVENT = 1,
	GSPI_DOCMD_TX_EVENT,
	GSPI_GP_SOCK_CMD_TX_EVENT,
	GSPI_RX_EVENT,
	GSPI_GOPLUS_TX_EVENT,
	GSPI_MAX_EVENT
} GSPI_EVENT_T;

typedef enum GP_SOCK_EVENT_E
{
	GP_SOCK_TXDONE_EVENT,
	GP_SOCK_RXDONE_EVENT,
	GP_SOCK_CLI_CONN_EVENT,
	GP_SOCK_CLI_DISC_EVENT,
	GP_SOCK_RAW_DATA_DONE_EVENT,
	GP_SOCK_RAW_DATA_SEND_BACK_EVENT,
	GP_SOCK_RAW_DATA_SEND_BACK_DONE_EVENT
} GP_SOCK_EVENT_T;

typedef enum GOPLUS_CMD_SOCK_EVENT_E
{
	GOPLUS_SOCK_CMD_TXDONE_EVENT,
	GOPLUS_SOCK_CMD_TXERROR_EVENT,
	GOPLUS_SOCK_CMD_RXDONE_EVENT,
	GOPLUS_SOCK_CMD_CLI_CONN_EVENT,
	GOPLUS_SOCK_CMD_CLI_DISC_EVENT
} GOPLUS_CMD_SOCK_EVENT_T;

/* This definition must be same in GSPI master and slave */
#define GSPI_CMD_START	0x100
typedef enum
{
	/* GSPI TX */
	GSPI_TX_MJPEG_CMD = GSPI_CMD_START,
	GSPI_TX_DOCMD_CMD,
	GSPI_TX_GP_SOCKET_CMD,				/* GSPI send data to GP socket command for ACK or data */
	GSPI_GET_GP_SOCKET_RAW_CMD,			/* GSPI command to get GP socket raw data */
	GSPI_TX_MJPEG_NET_HEADER_CMD,
	GSPI_TX_MJPEG_DATA_CMD,
	GSPI_TX_TCP_CLIENT_CMD,				/* GSPI TCP client TX data */
	GSPI_TX_UDP_CLIENT_CMD,				/* GSPI UDP client TX data */
	GSPI_TX_UDP_SERVER_CLIINFO_CMD,		/* GSPI UDP server client information */
	GSPI_TX_UDP_SERVER_CMD,				/* GSPI UDP server TX data */
	GSPI_TX_GOPLUS_CMD,					/* GSPI GOPLUS TX data */
	GSPI_TX_VIDEO_DATA_CMD,				/* GSPI VIDEO TX data */
	GSPI_TX_AUDIO_DATA_CMD,				/* GSPI AUDIO TX data */
	GSPI_TX_WEBSOCKET_TEXT_CMD,			/* GSPI WebSocket TX text data */
	GSPI_TX_WEBSOCKET_BINARY_CMD,		/* GSPI Websocket TX binary data */
	GSPI_TX_SSLCLI_1_DATA_CMD,			/* GSPI SSL client 1 TX data */
	GSPI_TX_SSLCLI_2_DATA_CMD,			/* GSPI SSL client 2 TX data */

	/* GSPI Slave request */
	GSPI_GET_STREAMING_REQ = (GSPI_CMD_START + 0x100),
	GSPI_STOP_STREAMING_REQ,
	GSPI_GP_SOCKET_CMD_REQ,				/* GP socket command request */
	GSPI_GP_SOCKET_RAW_DATA_SEND_REQ,	/* GP RAW data send request */
	GSPI_MJPEG_GET_DATA_REQ,			/* Get JPEG data for streaming, each data length is A/B buffer size of MJPEG module */
	GSPI_RX_TCPCLI_SOCKET_RECEIVED_REQ,	/* RX TCP client socket data get */
	GSPI_RX_UDPSRV_SOCKET_RECEIVED_REQ,	/* RX UDP server socket data get */
	GSPI_RX_UDPSRV_SOCKET_CLIINFO_REQ,	/* RX UDP server socket client information */
	GSPI_RX_UDPCLI_SOCKET_RECEIVED_REQ,	/* RX UDP client socket data get */
	GSPI_RX_GOPLUS_SOCKET_RECEIVED_REQ,	/* RX GOPLUS socket data get */
	GSPI_RX_RTP_SOCKET_RECEIVED_REQ,	/* RX RTP socket data get */
	GSPI_GET_VIDEO_STREAMING_REQ,		/* Get Video streaming start */
	GSPI_STOP_VIDEO_STREAMING_REQ,		/* Stop Video streaming start */
	GSPI_GET_AUDIO_STREAMING_REQ,		/* Get Audio streaming start */
	GSPI_STOP_AUDIO_STREAMING_REQ,		/* Stop Audio streaming start */
	GSPI_RX_WEBSOCKET_RECEIVED_REQ,		/* WebSocket RX data done request */
	GSPI_START_AUDIO_SPEAKER_REQ,		/* Audio speaker start */
	GSPI_STOP_AUDIO_SPEAKER_REQ,		/* Audio speaker stop */
	GSPI_RX_DATA_AUDIO_SPEAKER_REQ,		/* Audio speaker RX data get */
	GSPI_RX_SSLCLI_1_RECEIVED_REQ,		/* SSL client 1 RX data done request */
	GSPI_RX_SSLCLI_2_RECEIVED_REQ,		/* SSL client 2 RX data done request */

	/* GSPI Slave response */
	GSPI_AP_MODE_RES = (GSPI_CMD_START + 0x200),
	GSPI_STATION_MODE_RES,
	GSPI_NONE_MODE_RES,
	GSPI_STATION_CONNECT_FAILED_MODE_RES,
	GSPI_FPS_RES,
	GSPI_WIFI_SETTING_RES,
	GSPI_NET_APP_STATE_RES,
	GSPI_GP_SOCK_CLIENT_CON_RES,
	GSPI_GP_SOCK_CLIENT_DISC_RES,
	GSPI_GP_SOCKET_RAW_DATA_DONE_RES,	/* GP RAW data reveived done, notify host */
	GSPI_TX_GPSOCKET_DONE_RES,			/* GP SOCK TX socket done */
	GSPI_TX_TCPCLI_SOCKET_DONE_RES,		/* TX TCP client socket done */
	GSPI_TX_TCPCLI_SOCKET_ERROR_RES,	/* TX TCP client socket error */
	GSPI_TX_UDPSRV_SOCKET_DONE_RES,		/* TX UDP server socket done */
	GSPI_TX_UDPSRV_SOCKET_ERROR_RES,	/* TX UDP server socket error */
	GSPI_TX_UDPCLI_SOCKET_DONE_RES,		/* TX UDP client socket done */
	GSPI_TX_UDPCLI_SOCKET_ERROR_RES,	/* TX UDP client socket error */
	GSPI_DNS_RES,						/* DNS response */
	GSPI_SIMPLE_CONFIG_RES,				/* Simple config response */
	GSPI_RSSI_RES,						/* RSSI response */
	GSPI_APP_REMOTE_IP_RES,				/* Get remote APP IP address for server */
	GSPI_AP_AUTOCHANNEL_NUM_RES,		/* Get auto channel number */
	GSPI_GOPLUS_SOCK_CLIENT_CON_RES,	/* GOPLUS command socket client connected */
	GSPI_GOPLUS_SOCK_CLIENT_DISC_RES,	/* GOPLUS command socket client disconnected */
	GSPI_TX_GOPLUS_SOCKET_DONE_RES,		/* GOPLUS TX socket done */
	GSPI_TX_GOPLUS_SOCKET_ERROR_RES,	/* GOPLUS TX socket error */
	GSPI_TX_VIDEO_DATA_DONE_RES,		/* Video data TX done */
	GSPI_TX_AUDIO_DATA_DONE_RES,		/* Audio data TX done */
	GSPI_TX_WEBSOCKET_TEXT_DONE_RES,	/* WebSocket text data TX done */
	GSPI_TX_WEBSOCKET_BINARY_DONE_RES,	/* WebSocket binary data TX done */
	GSPI_TX_SSLCLI_1_DONE_RES,			/* SSL client 1 TX data done response */
	GSPI_TX_SSLCLI_2_DONE_RES,			/* SSL client 2 TX data done response */
	GSPI_DHCP_LATEST_CLIENT_IP_RES,		/* Get DHCP latest client IP address */

	/* Do something command */
	GSPI_DO_ATCMD_CMD = (GSPI_CMD_START + 0x300),

	/* System event */
	GSPI_SYS_WAKEUP_EVENT = (GSPI_CMD_START + 0x400),
	GSPI_SYS_SLEEP_EVENT,

	GSPI_STOP_CMD = 0xFFFF
} GSPI_CMD_E;

typedef enum
{
	DATA_UNKNOWN_TYPE = 0,
	DATA_JPEG_TYPE,
	DATA_DOCMD_TYPE,
	DATA_GP_SOCK_CMD_TYPE,
	DATA_GOPLUS_CMD_TYPE
} DATA_TYPE_E;

typedef enum
{
	GSPI_MJPEG_STOP_EVENT = 0,
	GSPI_MJPEG_START_EVENT,
	GSPI_MJPEG_TX_DONE_EVENT,
	GSPI_MJPEG_GET_DATA_EVENT,
	GSPI_MJPEG_UPDATE_FRAME_EVENT
} GSPI_MJPEG_EVENT_E;

typedef enum
{
	GSPI_VIDEO_STREAM_STOP_EVENT = 0,
	GSPI_VIDEO_STREAM_START_EVENT,
	GSPI_VIDEO_STREAM_SEND_EVENT,
	GSPI_VIDEO_UPDATE_FRAME_EVENT,
	GSPI_VIDEO_DATA_DONE_EVENT,
	GSPI_AUDIO_STREAM_STOP_EVENT,
	GSPI_AUDIO_STREAM_START_EVENT,
	GSPI_AUDIO_STREAM_SEND_EVENT,
	GSPI_AUDIO_DATA_DONE_EVENT,
	GSPI_AUDIO_SPEAKER_START_EVENT,
	GSPI_AUDIO_SPEAKER_STOP_EVENT,
	GSPI_AUDIO_SPEAKER_DATA_EVENT
} GSPI_STREAM_EVENT_E;

typedef enum
{
	GSPI_TC_STOP_EVENT = 0x100,
	GSPI_TC_START_EVENT,
	GSPI_TC_TX_DONE_EVENT,
	GSPI_TC_UPDATE_FRAME_EVENT,
	GSPI_TC_RX_LEN_EVENT,
	GSPI_TC_RX_DATA_EVENT,
	GSPI_TC_SOCKET_ERROR_EVENT,
	GSPI_TC_KEY_PRESS_EVENT,
	GSPI_UC_STOP_EVENT,
	GSPI_UC_START_EVENT,
	GSPI_UC_TX_DONE_EVENT,
	GSPI_UC_UPDATE_FRAME_EVENT,
	GSPI_UC_RX_LEN_EVENT,
	GSPI_UC_RX_DATA_EVENT,
	GSPI_UC_SOCKET_ERROR_EVENT,
	GSPI_UC_KEY_PRESS_EVENT,
	GSPI_US_TX_DONE_EVENT,
	GSPI_US_SOCKET_ERROR_EVENT,
	GSPI_US_RX_LEN_EVENT,
	GSPI_US_RX_CLIINFO_EVENT,
	GSPI_US_RX_DATA_EVENT,
	GSPI_RTP_RX_LEN_EVENT,
	GSPI_RTP_RX_DATA_EVENT
} GSPI_TCP_UDP_CLIENT_EVENT_E;

typedef enum
{
	GSPI_WEBSOCKET_TX_TEXT_DONE_EVENT = 0,
	GSPI_WEBSOCKET_TX_BINARY_DONE_EVENT,
	GSPI_WEBSOCKET_RX_DATA_DONE_EVENT
} GSPI_WEBSOCKET_EVENT_E;

typedef enum
{
	GSPI_WEBSOCKET_TEXT_DATA,
	GSPI_WEBSOCKET_BINARY_DATA

} GSPI_WEBSOCKET_DATA_TYPE_E;

typedef enum
{
	GSPI_SSLCLI_CONNECTION_1 = 1,
	GSPI_SSLCLI_CONNECTION_2
} GSPI_SSLCLI_CONNECTION_E;

typedef enum
{
	GSPI_SSLCLI_1_TX_DATA_DONE_EVENT = 0,
	GSPI_SSLCLI_1_RX_DATA_DONE_EVENT,
	GSPI_SSLCLI_2_TX_DATA_DONE_EVENT,
	GSPI_SSLCLI_2_RX_DATA_DONE_EVENT,
	GSPI_SSLCLI_1_CONNECT_STATE_CHANGED_EVENT,
	GSPI_SSLCLI_2_CONNECT_STATE_CHANGED_EVENT

} GSPI_SSLCLI_EVENT_E;
typedef struct GSPI_CMD_S
{
	INT8U	token[8];
	INT32U 	cmd;
	INT32U  datalen;
} PACKED GSPI_CMD_T;

typedef union gspi_do_cmd_s
{
	INT8U data[GSPI_DOCMD_SIZE];
	struct cmd_blk
	{
		INT8U atcmd[4];
		INT8U equal[1];
		INT8U parameter[123];
	} cmd;
} gspi_do_cmd_t;

typedef enum
{
    GPWFM01A =1,
    GPWFM02A
}GP_WIFI_MODULE;

#define GSPI_CMD_SIZE	(sizeof(GSPI_CMD_T))

//#if B
/* Following is GP's definition */
#define GSPI_INT_STACK_SIZE		1024
#define	SPI_MASTER_NUM			0			/* Use SPI 1 */
#define GSPI_MASTER_CS_PIN		IO_D6
#if (_OPERATING_SYSTEM == _OS_UCOS2)
#define GSPI_MASTER_CLK			SYSCLK_8	/* system clock(96Mhz) / GSPI_MASTER_CLK */
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
#define GSPI_MASTER_CLK         16000000    /*GP_WIFI_MODULE == GPWFM02A please set*/
#endif
#define GSPI_MASTER_EXT_INT		EXTB
#define WIFI_RESET_PIN			IO_D15
#define WIFI_CHANGE_IMAGE_DEMO	1
#define WIFI_CHANGE_IMAGE_PIN	IO_A10
#define WIFI_SSL_SWITCH_PIN		IO_A11
#define WIFI_SSL_SEND_REQ_PIN	IO_A13
#define WIFI_CHANGE_IMAGE_CNT	3000		/* 3000ms */
#define WIFI_MODULE_RESET_CNT	10			/* 100ms */
#define WIFI_MODULE_RESET_WAIT_CNT	21		/* 210ms */
#define WIFI_DO_SSL_CHECK		1
#define WIFI_SSL_SWITCH_CNT		2000		/* 3000ms */
#define GPWFM02A                02
#define GPWFM01A                01
#define GP_WIFI_MODULE          GPWFM02A


extern INT8U gspi_do_cmd_buf[];

extern void gspi_cmd_init(void);
extern void gspi_tx_cmd(INT32U cmd, INT32U tolen);
extern void gspi_tx_data(INT8U* buf, INT32U len, INT32U type);
extern void gspi_send_docmd(const char* cmd);
extern void gspi_transfer_mjpeg(INT8U* buf, INT32U len);
extern INT8S gspi_transfer_video(INT8U* buf, INT32U len);
extern void gspi_transfer_audio(INT8U* buf, INT32U len);
extern void gspi_transfer_websocket(INT8U* buf, INT32U len, INT32U type);
extern void gspi_transfer_sslcli(INT8U* buf, INT32U len, INT32U connection);
extern void gspi_transfer_tcpcli_socket(INT8U* buf, INT32U len);
extern void gspi_transfer_udpcli_socket(INT8U* buf, INT32U len);
extern void gspi_transfer_udpsrv_socket(INT8U* cliinfo, INT32U clilen, INT8U* buf, INT32U len);
extern void gspi_register_gp_sock_cmd_cbk(INT32U cbk);
extern void gspi_register_app_cbk(INT32U cbk, INT32U app);
#endif	//__GSPI_CMD_H__
