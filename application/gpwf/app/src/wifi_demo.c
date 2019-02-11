/************************************************************
* wifi_demo.c
*
* Purpose: WiFi demo code
*
* Author: Eugene Hsu
*
* Date: 2015/10/05
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

#include "video_encoder.h"
#include "avi_encoder_app.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"
#include "gspi_master_drv.h"
#include "wifi_demo.h"
//#include "door_bell.h"
#include "ssl_cli.h"

/**********************************************
*	Definitions
**********************************************/
#define WIFI_AP_AUTO_CHANNEL_SEL	0
#define WIFI_WAIT_TIMEOUT	30		/* 30 seconds */
#define WIFI_LOOP_WAIT_CNT	100		/* 1 seconds */
#define MJ_STREAM_TCP_MODE	0x00
#define MJ_STREAM_UDP_MODE	0x01
#define WIFI_MJ_STREAM_MODE	MJ_STREAM_UDP_MODE
/**********************************************
*	Extern variables and functions
**********************************************/

extern void gp_socket_cmd_task_init(void);
extern void gp_socket_server_loop(void);
extern void goplus_cmd_socket_task_init(void);
extern void mjpeg_streamer_task_init(void);
extern void udp_server_task_init(void);
extern void audio_stream_task_init(void);
extern void tutk_avapi_task_init(void);
extern void websocket_task_init(void);
extern void sslcli_task_init(void);
extern void sslcli_notify_connection_state(INT32U connection, INT32S state);
extern INT32S sslcli_1_connection_enable(void);
extern INT32S sslcli_2_connection_enable(void);
extern void sslcli_1_connection_disable(void);
extern void sslcli_2_connection_disable(void);
extern void simple_config_task(void);
extern INT32S udp_server_enable(void);
extern INT32S udp_server_disable(void);
extern INT32U send_mjpeg;

extern INT8U FW_LOADER_START;		/* Firmware Loader start address */
extern INT8U FW_LOADER_END;			/* Firmware Loader end address */
extern INT8U WIFI_APP_START;		/* WiFi APP start address */
extern INT8U WIFI_APP_END;			/* WiFi APP end address */
extern INT8U WIFI_APP_WS_START;		/* WiFi WebSocket APP start address */
extern INT8U WIFI_APP_WS_END;		/* WiFi WebSocket APP end address */
extern INT8U WIFI_APP_SSL_START;	/* WiFi SSL APP start address */
extern INT8U WIFI_APP_SSL_END;		/* WiFi SSL APP end address */
extern INT8U WIFI_APP_TUTK_START;	/* WiFi TUTK APP start address */
extern INT8U WIFI_APP_TUTK_END;		/* WiFi TUTK APP end address */
void wifi_iint_change_image_pin(void);
void wifi_init_ssl_switch_pin(void);
void wifi_init_ssl_send_request_pin(void);
/*********************************************
*	Variables declaration
*********************************************/
INT32U wifi_demo_flag = WIFI_GP_SOCKET_DEMO;
INT32U wifi_demo_mode = WIFI_AP_MODE;
INT32U wifi_do_auto_channel_done = 0;
GP_NET_APP_T wifi_app_state = {0};
GP_AP_AUTOCHANNEL_NUM_T wifi_ap_auto_channel = {0};
WIFI_IMAGE_INFO_T wifi_iamge_info;
static INT32U wifi_ssl_enable_state = 0;	/* Record if did enable SSL or not */

static INT32U wifi_module = GPWFM02A;

static void wifi_state_info_cbk(void* data)
{
	GP_WIFI_STATE_T* stateptr = (GP_WIFI_STATE_T*)data;
	DBG_PRINT("Update WiFi state to %d\r\n", stateptr->mode);
}

static void wifi_app_info_cbk(void* data)
{
	GP_NET_APP_T* appptr = (GP_NET_APP_T*)data;
	DBG_PRINT("Update WiFi APP state\r\n");

	if(wifi_app_state.mjpeg_streamer_state != appptr->mjpeg_streamer_state)
	{
		DBG_PRINT("MJPEG    %d => %d\r\n", wifi_app_state.mjpeg_streamer_state, appptr->mjpeg_streamer_state);
	}
	if(wifi_app_state.gp_socket_state != appptr->gp_socket_state)
	{
		DBG_PRINT("GPSOCKET %d => %d\r\n", wifi_app_state.gp_socket_state, appptr->gp_socket_state);
	}
	if(wifi_app_state.tcpcli_state != appptr->tcpcli_state)
	{
		DBG_PRINT("TCPCLI   %d => %d\r\n", wifi_app_state.tcpcli_state, appptr->tcpcli_state);
	}
	if(wifi_app_state.udpsrv_state != appptr->udpsrv_state)
	{
		DBG_PRINT("UDPSRV   %d => %d\r\n", wifi_app_state.udpsrv_state, appptr->udpsrv_state);
	}
	if(wifi_app_state.udpcli_state != appptr->udpcli_state)
	{
		DBG_PRINT("UDPCLI   %d => %d\r\n", wifi_app_state.udpcli_state, appptr->udpcli_state);
	}
	if(wifi_app_state.goplus_state != appptr->goplus_state)
	{
		DBG_PRINT("GOPLUS   %d => %d\r\n", wifi_app_state.goplus_state, appptr->goplus_state);
	}
	if(wifi_app_state.rtspsrv_state != appptr->rtspsrv_state)
	{
		DBG_PRINT("RTSPSRV   %d => %d\r\n", wifi_app_state.rtspsrv_state, appptr->rtspsrv_state);
	}
	if(wifi_app_state.rtpsrv_state != appptr->rtpsrv_state)
	{
		DBG_PRINT("RTPSRV   %d => %d\r\n", wifi_app_state.rtpsrv_state, appptr->rtpsrv_state);
	}
	if(wifi_app_state.websocket_state != appptr->websocket_state)
	{
		DBG_PRINT("Websocket   %d => %d\r\n", wifi_app_state.websocket_state, appptr->websocket_state);
	}
	if(wifi_app_state.sslcli_1_state != appptr->sslcli_1_state)
	{
		sslcli_notify_connection_state(GSPI_SSLCLI_CONNECTION_1, appptr->sslcli_1_state);
		//DBG_PRINT("SSL1   %d => 0x%x\r\n", wifi_app_state.sslcli_1_state, appptr->sslcli_1_state);
	}
	if(wifi_app_state.sslcli_2_state != appptr->sslcli_2_state)
	{
		sslcli_notify_connection_state(GSPI_SSLCLI_CONNECTION_2, appptr->sslcli_2_state);
		//DBG_PRINT("SSL2   %d => 0x%x\r\n", wifi_app_state.sslcli_2_state, appptr->sslcli_2_state);
	}

	memcpy((void*)&wifi_app_state, (void*)data, sizeof(GP_NET_APP_T));
}

static void wifi_dns_info_cbk(void* data)
{
	GP_DNS_T* dnsptr = (GP_DNS_T*)data;

	if(dnsptr->result == 0)
	{
		DBG_PRINT("DNS resolution failed\r\n");
	}

	//else
	//{
	//	INT32U msg;
	//	DBG_PRINT("DNS resolution successful\r\n");
	//	door_bell_set_server_ip(dnsptr->dnsip);
		/* Door bell demo */
	//	door_bell_task_init();

	//	msg = GSPI_UC_START_EVENT;
	//	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	//	OSQPost(door_bell_q, (void*)msg);
	//	#endif
	//}

}

static void wifi_wifi0_setting_info_cbk(void* data)
{
	GP_WIFI_SETTING_T* setting = (GP_WIFI_SETTING_T*)data;

	DBG_PRINT("==========WiFi Setting==========\r\n");
	DBG_PRINT("Interface: %s\r\n", setting->ifname);
	DBG_PRINT("Mode: %s\r\n", (setting->mode == WIFI_AP_MODE)?"AP MODE":((setting->mode == WIFI_STATION_MODE)?"STATION MODE":"NONE"));
	DBG_PRINT("SSID: %s\r\n", setting->ssid);
	DBG_PRINT("Channel: %d\r\n", setting->channel);
	DBG_PRINT("Password: %s\r\n", setting->password);
	switch(setting->security_type)
	{
		case SECURITY_OPEN:
			DBG_PRINT("Security: OPEN");
			break;

		case SECURITY_WEP_PSK:
			DBG_PRINT("Security: WEP");
			DBG_PRINT("Key index: %d", setting->key_idx);
			break;

        case SECURITY_WPA_TKIP_PSK:
			DBG_PRINT("Security: TKIP");
			break;

		case SECURITY_WPA2_AES_PSK:
			DBG_PRINT("Security: AES");
			break;

		default:
			DBG_PRINT("Security: UNKNOWN");
	}
	DBG_PRINT("\r\n==========Interface %s==========\r\n", setting->ifname);
	DBG_PRINT("MAC: %x:%x:%x:%x:%x:%x\r\n", setting->mac[0], setting->mac[1], setting->mac[2], setting->mac[3]
			, setting->mac[4], setting->mac[5]);
	DBG_PRINT("IP : %d.%d.%d.%d\r\n", setting->ip[0], setting->ip[1], setting->ip[2], setting->ip[3]);
	DBG_PRINT("GW : %d.%d.%d.%d\r\n", setting->gw[0], setting->gw[1], setting->gw[2], setting->gw[3]);

	if(setting->client_info.count != 0 && wifi_demo_mode == WIFI_AP_MODE)
	{
		INT32U i;
		DBG_PRINT("=======WiFi Client Info========\r\n");
		for(i=0; i<setting->client_info.count; i++)
		{
			DBG_PRINT("Client %d\r\n", i);
			DBG_PRINT("MAC=> %02x:%02x:%02x:%02x:%02x:%02x\r\n", setting->client_info.mac_list[i].mac[0], setting->client_info.mac_list[i].mac[1],
																		setting->client_info.mac_list[i].mac[2], setting->client_info.mac_list[i].mac[3],
																		setting->client_info.mac_list[i].mac[4], setting->client_info.mac_list[i].mac[5]);
		}
	}
}

static void wifi_get_ap_clear_channel_cbk(void* data)
{
	GP_AP_AUTOCHANNEL_NUM_T* clear_ch = (GP_AP_AUTOCHANNEL_NUM_T*)data;

	wifi_ap_auto_channel.gp_ap_ac_num = clear_ch->gp_ap_ac_num;
	wifi_do_auto_channel_done = 1;
	DBG_PRINT("Get AP clear channel number %d\r\n", wifi_ap_auto_channel.gp_ap_ac_num);
}

static void wifi_dhcp_client_ip_cbk(void* data)
{
	GP_DHCP_CLIENT_IP_T* dhcp_client_ip = (GP_DHCP_CLIENT_IP_T*)data;
	DBG_PRINT("DHCP client IP %d.%d.%d.%d\r\n", (dhcp_client_ip->gp_sock_client_ip & 0xff), (dhcp_client_ip->gp_sock_client_ip & 0xff00) >> 8
											, (dhcp_client_ip->gp_sock_client_ip & 0xff0000) >> 16, (dhcp_client_ip->gp_sock_client_ip & 0xff000000) >> 24);
    DBG_PRINT("tick %d\r\n",xTaskGetTickCount());
}
///////////////////////////////////////////////////////////////////

void wifi_start_app(INT32U flag)
{
	INT32U cnt = WIFI_WAIT_TIMEOUT;
	INT32U retry = 5;

	/* Enable WiFi mode successfully */
	if(flag == WIFI_GP_SOCKET_DEMO)
	{
retry_socket:
		/* Config GP socket command port */
		gspi_send_docmd(GP_CMD_CONFIG_GP_SOCKET_PORT);
		gspi_send_docmd(GP_CMD_ENABLE_GP_SOCKET);

		while((gp_net_app_state.gp_socket_state == 0) && cnt)
		{
			cnt--;
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay (100*10);
			#endif
		}
		if(!gp_net_app_state.gp_socket_state)
		{
			DBG_PRINT("Enale GP socket service failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop GP socket */
				gspi_send_docmd(GP_CMD_DISABLE_GP_SOCKET);
				goto retry_socket;
			}
		}
		else
		{
			/* GP socket command demo */
			gp_socket_cmd_task_init();
		}
	}
	else if(flag == WIFI_MJPEG_STREAMER_DEMO)
	{
retry_mj:
#if (WIFI_MJ_STREAM_MODE == MJ_STREAM_UDP_MODE)
		/* Config GP socket command port */
		gspi_send_docmd(GP_CMD_ENABLE_RTSP_RTP_MJPEG_SOCKET);

		while((gp_net_app_state.rtspsrv_state == 0 || gp_net_app_state.rtpsrv_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}
		if(!gp_net_app_state.rtspsrv_state || !gp_net_app_state.rtpsrv_state)
		{
			DBG_PRINT("Enale RTSP RTP failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop RTP/RTSP */
				gspi_send_docmd(GP_CMD_DISABLE_RTSP_RTP_SOCKET);
				goto retry_mj;
			}
		}
		else
		{
			DBG_PRINT("Start MJPEG streamer");
			/* MJPEG streamer demo */
			mjpeg_streamer_task_init();
		}
#elif(WIFI_MJ_STREAM_MODE == MJ_STREAM_TCP_MODE)
		/* Config GP socket command port */
		gspi_send_docmd(GP_CMD_CONFIG_MJPEG_STREAMER_PORT);
		gspi_send_docmd(GP_CMD_ENABLE_MJPEG_STREAMER);

		while((gp_net_app_state.mjpeg_streamer_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif

		}
		if(!gp_net_app_state.mjpeg_streamer_state)
		{
			DBG_PRINT("Enale MJPEG streamer failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop MJPEG streamer */
				gspi_send_docmd(GP_CMD_DISABLE_MJPEG_STREAMER);
				goto retry_mj;
			}
		}
		else
		{
			/* MJPEG streamer demo */
			mjpeg_streamer_task_init();
		}
#endif
	}
	else if(flag == WIFI_AUDIO_STREAM_DEMO)
	{
retry_as:
		/* Enable RTP/RTSP service */
		gspi_send_docmd(GP_CMD_ENABLE_RTSP_RTP_AUDIO_SOCKET);

		while((gp_net_app_state.rtspsrv_state == 0 || gp_net_app_state.rtpsrv_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}
		if(!gp_net_app_state.rtspsrv_state || !gp_net_app_state.rtpsrv_state)
		{
			DBG_PRINT("Enale RTSP RTP failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop RTP/RTSP */
				gspi_send_docmd(GP_CMD_DISABLE_RTSP_RTP_SOCKET);
				goto retry_as;
			}
		}
		else
		{
			DBG_PRINT("Start audio stream");
			/* Audio streame demo */
			audio_stream_task_init();
		}
	}
	else if(flag == WIFI_SOCKET_MJPEG_DEMO)
	{
retry_wifi_socket:
		/* MJPEG + GP socket command demo */
		gspi_send_docmd(GP_CMD_CONFIG_GP_SOCKET_PORT);
		gspi_send_docmd(GP_CMD_ENABLE_GP_SOCKET);

		while((gp_net_app_state.gp_socket_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}
		if(!gp_net_app_state.gp_socket_state)
		{
			DBG_PRINT("Enable GP socket service failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop GP socket */
				gspi_send_docmd(GP_CMD_DISABLE_GP_SOCKET);
				goto retry_wifi_socket;
			}
		}
retry_wifi_mj:
		retry = 5;
		gspi_send_docmd(GP_CMD_CONFIG_MJPEG_STREAMER_PORT);
		gspi_send_docmd(GP_CMD_ENABLE_MJPEG_STREAMER);

		while((gp_net_app_state.mjpeg_streamer_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}
		if(!gp_net_app_state.mjpeg_streamer_state)
		{
			DBG_PRINT("Enable MJPEG streamer failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop MJPEG streamer */
				gspi_send_docmd(GP_CMD_DISABLE_MJPEG_STREAMER);
				goto retry_wifi_mj;
			}
		}
		else
		{
			/* MJPEG + GP socket command demo */
			gp_socket_cmd_task_init();
			mjpeg_streamer_task_init();
		}
	}
	/*
	else if(flag == WIFI_DOOR_BELL_DEMO)
	{
		char cmd[64] = {0};

		sprintf(cmd, GP_CMD_DNS_RESOLUTION "%s", DOOR_BELL_HOST_NAME);
		//Resolution door bell host name to IP address
		gspi_send_docmd(cmd);
	}
	*/
	else if(wifi_demo_flag == WIFI_UDP_SERVER_DEMO)
	{
		/* Enable UDP server */
		if(!udp_server_enable())
		{
			DBG_PRINT("Enable UDP server successfully\r\n");
		}
		/* UDP server demo */
		udp_server_task_init();
	}
	else if(wifi_demo_flag == WIFI_GOPLUS_DEMO)
	{
		/* Start MJPEG and GOPLUS APP in GPWFM01A */
retry_goplus:
		gspi_send_docmd(GP_CMD_ENABLE_GOPLUS_SOCKET);

		while((gp_net_app_state.goplus_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}
		if(!gp_net_app_state.goplus_state)
		{
			DBG_PRINT("Enable GP GOPLUS socket failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop GP socket */
				gspi_send_docmd(GP_CMD_DISABLE_GOPLUS_SOCKET);
				goto retry_goplus;
			}
		}

		retry = 5;
retry_goplus_mj:
#if (WIFI_MJ_STREAM_MODE == MJ_STREAM_UDP_MODE)
		/* Config GP socket command port */
		gspi_send_docmd(GP_CMD_ENABLE_RTSP_RTP_MJPEG_SOCKET);

		while((gp_net_app_state.rtspsrv_state == 0 || gp_net_app_state.rtpsrv_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}
		if(!gp_net_app_state.rtspsrv_state || !gp_net_app_state.rtpsrv_state)
		{
			DBG_PRINT("Enale RTSP RTP failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop MJPEG streamer */
				gspi_send_docmd(GP_CMD_DISABLE_RTSP_RTP_SOCKET);
				goto retry_goplus_mj;
			}
		}
#elif(WIFI_MJ_STREAM_MODE == MJ_STREAM_TCP_MODE)
		gspi_send_docmd(GP_CMD_CONFIG_MJPEG_STREAMER_PORT);
		gspi_send_docmd(GP_CMD_ENABLE_MJPEG_STREAMER);

		while((gp_net_app_state.mjpeg_streamer_state == 0) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}
		if(!gp_net_app_state.mjpeg_streamer_state)
		{
			DBG_PRINT("Enale MJPEG streamer failed\r\n");
			retry--;
			if(retry)
			{
				/* Stop MJPEG streamer */
				gspi_send_docmd(GP_CMD_DISABLE_MJPEG_STREAMER);
				goto retry_goplus_mj;
			}
		}
#endif
		/* Init MPEG streamer and GOPLUS command socket task */
		mjpeg_streamer_task_init();
		goplus_cmd_socket_task_init();
	}
	else if(flag == WIFI_TUTK_DEMO)
	{
		gspi_send_docmd(GP_CMD_ENABLE_TUTK);
		/* TUTK avapi demo */
        tutk_avapi_task_init();
	}
	else if(flag == WIFI_WS_SSL_DEMO)
	{
		gp_net_app_state.websocket_state = WEBSOCKET_DISCONNECT_STATE;
		gspi_send_docmd(GP_CMD_ENABLE_WEBSOCKET_SSL);
		while((gp_net_app_state.websocket_state != WEBSOCKET_OK_STATE) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}

		if(gp_net_app_state.websocket_state != WEBSOCKET_OK_STATE)
		{
			char errmsg[128];
			if(gp_net_app_state.websocket_state == WEBSOCKET_CREATE_CLIENT_FAILED_STATE)
			{
				strcpy(errmsg, "Websocket failed due to create client failed.\r\n");
			}
			else if(gp_net_app_state.websocket_state == WEBSOCKET_CONNECT_URL_FAILED_STATE)
			{
				strcpy(errmsg, "Websocket failed due to connect to URL failed.\r\n");
			}
			else if(gp_net_app_state.websocket_state == WEBSOCKET_CONNECT_URL_FAILED_STATE)
			{
				strcpy(errmsg, "Websocket failed due to connect to URL failed.\r\n");
			}
			DBG_PRINT("%s", errmsg);
		}
		else
		{
			DBG_PRINT("Websocket connect successfully\r\n");
			websocket_task_init();
		}
	}
	else if(flag == WIFI_SSL_DEMO)
	{
		INT32U result = 0;
		INT32S ret;

#if 1
		/* Enable SSL1 */
		wifi_ssl_enable_state |= SSL_1_ENABLE;
		ret = sslcli_1_connection_enable();

		if(!ret)
		{
			DBG_PRINT("SSL 1 connection enable successfully\r\n");
			result++;
		}
		else
		{
			/* Release SSL1 resource in WiFi module */
			sslcli_1_connection_disable();
			wifi_ssl_enable_state &= ~SSL_1_ENABLE;
		}

		/* Enable SSL2 */
		wifi_ssl_enable_state |= SSL_2_ENABLE;
		ret = sslcli_2_connection_enable();

		if(!ret)
		{
			DBG_PRINT("SSL 2 connection enable successfully\r\n");
			result++;
		}
		else
		{
			/* Release SSL1 resource in WiFi module */
			sslcli_2_connection_disable();
			wifi_ssl_enable_state &= ~SSL_2_ENABLE;
		}

#else
		/* Connect to SSL 1 */
		gp_net_app_state.sslcli_1_state = SSLCLI_DISCONNECT_STATE;

		gspi_send_docmd(GP_CMD_ENABLE_SSL_1);
		wifi_ssl_enable_state |= SSL_1_ENABLE;

		while((gp_net_app_state.sslcli_1_state != SSLCLI_CONNECT_STATE) && cnt)
		{
			cnt--;
			OSTimeDly(100);
		}

		if(gp_net_app_state.sslcli_1_state != SSLCLI_CONNECT_STATE)
		{
			DBG_PRINT("SSL 1 connected to server failed, state 0x%x", gp_net_app_state.sslcli_1_state);
			/* Release SSL1 resource in WiFi module */
			gspi_send_docmd(GP_CMD_DISABLE_SSL_1);
			wifi_ssl_enable_state &= ~SSL_1_ENABLE;
		}
		else
		{

		}
		/* Connect to SSL 2 */
		gp_net_app_state.sslcli_2_state = SSLCLI_DISCONNECT_STATE;

		gspi_send_docmd(GP_CMD_ENABLE_SSL_2);
		wifi_ssl_enable_state |= SSL_2_ENABLE;

		while((gp_net_app_state.sslcli_2_state != SSLCLI_CONNECT_STATE) && cnt)
		{
			cnt--;
			OSTimeDly(100);
		}

		if(gp_net_app_state.sslcli_2_state != SSLCLI_CONNECT_STATE)
		{
			DBG_PRINT("SSL 2 connected to server failed, state 0x%x", gp_net_app_state.sslcli_2_state);
			/* Release SSL1 resource in WiFi module */
			gspi_send_docmd(GP_CMD_DISABLE_SSL_2);
			wifi_ssl_enable_state &= ~SSL_2_ENABLE;
		}
		else
		{
			DBG_PRINT("SSL 2 connected to server successfully\r\n");
			result++;
		}
#endif

		if(result)
		{
			/* Run SSL connection handle task */
			sslcli_task_init();
		}
	}
}

void wifi_start_network(INT32U flag, INT32U mode)
{
	INT32U cnt = WIFI_WAIT_TIMEOUT;
	INT32U wifi_mode_done = 0;

	gp_wifi_state.mode = 0;

	if(mode == WIFI_AP_MODE)
	{
        /* Set WIFI APIP address */
        gspi_send_docmd(GP_CMD_SET_AP_IP_ADDR);
#if (WIFI_AP_AUTO_CHANNEL_SEL == 1)
		char apchbuf[16] = {0};
		wifi_do_auto_channel_done = 0;
		/* Send AP auto channel number command */
		gspi_send_docmd(GP_CMD_GET_CLEAR_AP_CHANNEL_NUM);

		while(wifi_do_auto_channel_done != 1)
		{
			/* Wait for do auto channel done */
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(10);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(10*10);
			#endif
		}

		/* Set AP SSID */
		gspi_send_docmd(AT_CMD_SET_AP_SSID);
		/* Set AP security key */
		gspi_send_docmd(AT_CMD_SET_AP_SECURE_KEY);
		/* Set AP channel */
		sprintf(apchbuf, "ATW5=%d", wifi_ap_auto_channel.gp_ap_ac_num);
		gspi_send_docmd(apchbuf);
		/* For AP mode, wait wifi module stable */
		#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif

		/* Enable AP */
		gp_wifi_state.mode = WIFI_WAIT_RES_MODE;
		gspi_send_docmd(AT_CMD_ACTIVATE_AP);

		while((gp_wifi_state.mode == WIFI_WAIT_RES_MODE) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}

		if(gp_wifi_state.mode == WIFI_AP_MODE)
		{
			DBG_PRINT("Start service, WIFI_AP_MODE\r\n");
			wifi_mode_done = 1;
		}
#else
		/* Set AP SSID */
		gspi_send_docmd(AT_CMD_SET_AP_SSID);
		/* Set AP security key */
		gspi_send_docmd(AT_CMD_SET_AP_SECURE_KEY);
		/* Set AP channel */
		gspi_send_docmd(AT_CMD_SET_AP_CHANNEL);
		/* For AP mode, wait wifi module stable */
		#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif

		/* Enable AP */
		gp_wifi_state.mode = WIFI_WAIT_RES_MODE;
		gspi_send_docmd(AT_CMD_ACTIVATE_AP);

		while((gp_wifi_state.mode == WIFI_WAIT_RES_MODE) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}

		if(gp_wifi_state.mode == WIFI_AP_MODE)
		{
			DBG_PRINT("Start service, WIFI_AP_MODE\r\n");
			wifi_mode_done = 1;
		}
#endif
	}
	else if(mode == WIFI_STATION_MODE)
	{
		/* Set station SSID */
		gspi_send_docmd(AT_CMD_SET_NETWORK_SSID);
		/* Set password */
		gspi_send_docmd(AT_CMD_SET_NETWORK_PASSPHRASE);

		/* For station mode, wait wifi module stable */
		#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		gp_wifi_state.mode = WIFI_WAIT_RES_MODE;
		gspi_send_docmd(AT_CMD_JOIN_NEWORK);
		while((gp_wifi_state.mode == WIFI_WAIT_RES_MODE) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}

		if(gp_wifi_state.mode == WIFI_STATION_MODE)
		{
			DBG_PRINT("Start service, WIFI_STATION_MODE\r\n");
			wifi_mode_done = 1;
		}
	}
	else if(mode == WIFI_TUTK_MODE)
	{
		/* Set station SSID */
		gspi_send_docmd(AT_CMD_SET_NETWORK_SSID);
		/* Set password */
		gspi_send_docmd(AT_CMD_SET_NETWORK_PASSPHRASE);

		/* For station mode, wait wifi module stable */
		#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		gp_wifi_state.mode = WIFI_WAIT_RES_MODE;
		gspi_send_docmd(AT_CMD_JOIN_NEWORK);
		while((gp_wifi_state.mode == WIFI_WAIT_RES_MODE) && cnt)
		{
			cnt--;
			#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif
		}

		if(gp_wifi_state.mode == WIFI_STATION_MODE)
		{
			DBG_PRINT("Start service, WIFI_STATION_MODE\r\n");
			wifi_mode_done = 1;
		}
	}

	if(wifi_mode_done)
	{
		/* Start APP here */
		wifi_start_app(flag);
		gspi_send_docmd(GP_CMD_GET_WIFI0_SETTING);
	}
	else if(mode != WIFI_SC_MODE)
	{
		/* Enable WiFi mode failed */
		DBG_PRINT("Enable %s mode failed\r\n", (mode == WIFI_AP_MODE)?"AP":"STATION");
	}
}

void wifi_stop_app(INT32U flag)
{
	if(flag == WIFI_GP_SOCKET_DEMO)
	{
		/* Stop GP socket */
		gspi_send_docmd(GP_CMD_DISABLE_GP_SOCKET);
	}
	else if(flag == WIFI_MJPEG_STREAMER_DEMO)
	{
		/* Stop MJPEG streamer */
		gspi_send_docmd(GP_CMD_DISABLE_MJPEG_STREAMER);
	}
	else if(flag == WIFI_SOCKET_MJPEG_DEMO)
	{
		/* Stop GP socket + MJPEG streamer */
		gspi_send_docmd(GP_CMD_DISABLE_GP_SOCKET);
		gspi_send_docmd(GP_CMD_DISABLE_MJPEG_STREAMER);
	}
/*
	else if(flag == WIFI_DOOR_BELL_DEMO)
	{
		if(door_bell_disconnect_tcp_server())
		{
			DBG_PRINT("Disconnect DB TCP server failed\r\n");
		}
		if(door_bell_disconnect_udp_server())
		{
			DBG_PRINT("Disconnect DB UDP server failed\r\n");
		}
	}
*/
	else if(flag == WIFI_UDP_SERVER_DEMO)
	{
		/* Stop UDP server */
		udp_server_disable();
	}

	else if(flag == WIFI_GOPLUS_DEMO)
	{
		/* Stop GOPLUS and MJPEG*/
		gspi_send_docmd(GP_CMD_DISABLE_GOPLUS_SOCKET);
		gspi_send_docmd(GP_CMD_DISABLE_MJPEG_STREAMER);
	}
	else if(flag == WIFI_WS_SSL_DEMO)
	{
		/* Stop websocket */
		gspi_send_docmd(GP_CMD_DISABLE_WEBSOCKET_SSL);
	}
	else if(flag == WIFI_SSL_DEMO)
	{
		/* Disable SSL 1 connection */
		if(wifi_ssl_enable_state & SSL_1_ENABLE)
		{
			sslcli_1_connection_disable();
			wifi_ssl_enable_state &= ~SSL_1_ENABLE;
		}

		/* Disable SSL 1 connection */
		if(wifi_ssl_enable_state & SSL_2_ENABLE)
		{
			sslcli_2_connection_disable();
			wifi_ssl_enable_state &= ~SSL_2_ENABLE;
		}
	}
}

void wifi_stop_network(void)
{
	INT32U cnt = 5;

	/* Disconnect WiFI */
	gspi_send_docmd(AT_CMD_WIFI_DISCONNECT);
	/* Get WiFi module status */
	gspi_send_docmd(GP_CMD_GET_WIFISTATUS);
	while(cnt--)
	{
		/* Wait for 1 seconds */
		#if( _OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100*10);
			#endif

		if(gp_wifi_state.mode == WIFI_NONE_MODE)
		{
			DBG_PRINT("Stop network...\r\n");
			break;
		}
		gspi_send_docmd(GP_CMD_GET_WIFISTATUS);
	}
}

////////////////////////////////////////////

void wifi_init_reset_pin(void)
{
	//gpio_set_memcs(1, 0); //reset pin set as IO
	gpio_init_io(WIFI_RESET_PIN, GPIO_OUTPUT);
	gpio_set_port_attribute(WIFI_RESET_PIN, 1);
	gpio_write_io(WIFI_RESET_PIN, 1);
}

void wifi_init_gpio(void)
{
	/* Init SPI interface first for WiFi demo */
	drv_l1_spi_init(SPI_MASTER_NUM);
	drv_l1_spi_clk_set(SPI_MASTER_NUM, GSPI_MASTER_CLK);
	drv_l1_spi_cs_by_gpio_set(SPI_MASTER_NUM, DISABLE, GSPI_MASTER_CS_PIN, 1);
	//spi_init(SPI_MASTER_NUM);

	//spi_clk_set(SPI_MASTER_NUM, GSPI_MASTER_CLK);

	//spi_cs_by_external_set(SPI_MASTER_NUM);

	gspi_set_cs_io(GSPI_MASTER_CS_PIN);

	gspi_set_spi_num(SPI_MASTER_NUM);

	gspi_set_ext_int(GSPI_MASTER_EXT_INT);

	wifi_init_reset_pin();
#if (WIFI_CHANGE_IMAGE_DEMO == 1)
	wifi_iint_change_image_pin();
#endif
	//DBG_PRINT("Use SPI %d for WiFi Demo\r\n", SPI_MASTER_NUM);
}

void wifi_reset_module(void)
{
	gpio_write_io(WIFI_RESET_PIN, 1);
	gpio_write_io(WIFI_RESET_PIN, 0);
	#if (_OPERATING_SYSTEM ==_OS_UCOS2)
	OSTimeDly(WIFI_MODULE_RESET_CNT);	//wait for 100 ms for pull low
	#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
	osDelay(WIFI_MODULE_RESET_CNT*10);
	#endif
	gpio_write_io(WIFI_RESET_PIN, 1);
}

#if (WIFI_CHANGE_IMAGE_DEMO == 1)
void wifi_iint_change_image_pin(void)
{
	//gpio_set_memcs(1, 0); //reset pin set as IO
	gpio_init_io(WIFI_CHANGE_IMAGE_PIN, GPIO_OUTPUT);
	gpio_set_port_attribute(WIFI_CHANGE_IMAGE_PIN, 1);
	gpio_write_io(WIFI_CHANGE_IMAGE_PIN, 0);
}

void wifi_check_change_image(INT32U cnt)
{
	static INT32U image_cnt = 0;
	static INT8U image_key_status = 0;

	image_key_status = gpio_read_io(WIFI_CHANGE_IMAGE_PIN);

	if(image_key_status)
	{
		if(image_cnt++ == (WIFI_CHANGE_IMAGE_CNT/cnt))
		{
			/* Wait for 3 seconds */
			DBG_PRINT("Restart FW download\r\n");
			wifi_reset_module();

			/* Download WiFi module firmware */
			if(gspi_fw_download_start(&wifi_iamge_info,wifi_module) != STATUS_OK)
			{
				DBG_PRINT("wifi_demo_init: download WiFi firmware failed\r\n");
				while(1);
			}
			wifi_start_network(wifi_demo_flag, wifi_demo_mode);
		}
	}
	else
	{
		image_cnt = 0;
	}
}
#endif

#if (WIFI_DO_SSL_CHECK == 1)
void wifi_init_ssl_switch_pin(void)
{
	gpio_set_memcs(1, 0); //reset pin set as IO
	gpio_init_io(WIFI_SSL_SWITCH_PIN, GPIO_OUTPUT);
	gpio_set_port_attribute(WIFI_SSL_SWITCH_PIN, 1);
	gpio_write_io(WIFI_SSL_SWITCH_PIN, 0);
}

void wifi_switch_ssl_app(INT32U cnt)
{
	static INT32U switch_cnt = 0;
	static INT8U switch_key_status = 0;

	switch_key_status = gpio_read_io(WIFI_SSL_SWITCH_PIN);

	if(switch_key_status)
	{
		if(switch_cnt++ == (WIFI_SSL_SWITCH_CNT/cnt))
		{
			/* Wait for 3 seconds */
			DBG_PRINT("Change SSL status from %d to %d\r\n", wifi_ssl_enable_state, !wifi_ssl_enable_state);

			if(wifi_ssl_enable_state)
			{
				/* Stop SSL APP */
				wifi_stop_app(WIFI_SSL_DEMO);
			}
			else
			{
				/* Start SSL APP */
				wifi_start_app(WIFI_SSL_DEMO);
			}
		}
	}
	else
	{
		switch_cnt = 0;
	}
}

void wifi_init_ssl_send_request_pin(void)
{
	gpio_set_memcs(1, 0); //reset pin set as IO
	gpio_init_io(WIFI_SSL_SEND_REQ_PIN, GPIO_OUTPUT);
	gpio_set_port_attribute(WIFI_SSL_SEND_REQ_PIN, 1);
	gpio_write_io(WIFI_SSL_SEND_REQ_PIN, 0);
}

void wifi_check_ssl_send_request(INT32U cnt)
{
	static INT32U send_req_cnt = 0;
	static INT8U send_request_key_status = 0;

	send_request_key_status = gpio_read_io(WIFI_SSL_SEND_REQ_PIN);

	if(send_request_key_status)
	{
		if(send_req_cnt++ == (WIFI_SSL_SWITCH_CNT/cnt))
		{
			if(gp_net_app_state.sslcli_1_state == SSLCLI_CONNECT_STATE)
			{
				/* Send test request to SSL 1 if SSL 1 is connected */
				gspi_transfer_sslcli((INT8U*)TEST_GET_REQUEST, strlen(TEST_GET_REQUEST), GSPI_SSLCLI_CONNECTION_1);
			}
			else
			{
				DBG_PRINT("\r\nDue to SSL1 state = 0x%x, failed to send TEST_GET_REQUEST\r\n", gp_net_app_state.sslcli_1_state);
			}

			if(gp_net_app_state.sslcli_2_state == SSLCLI_CONNECT_STATE)
			{
				/* Send test request to SSL 2 if SSL 2 is connected */
				gspi_transfer_sslcli((INT8U*)TEST_GET_REQUEST, strlen(TEST_GET_REQUEST), GSPI_SSLCLI_CONNECTION_2);
			}
			else
			{
				DBG_PRINT("\r\nDue to SSL2 state = 0x%x, failed to send TEST_GET_REQUEST\r\n", gp_net_app_state.sslcli_2_state);
			}
		}
	}
	else
	{
		send_req_cnt = 0;
	}
}
#endif

void wifi_demo_init(INT32U flag, INT32U wifi_mode)
{

    DBG_PRINT("/**********************************************/\r\n");
    DBG_PRINT("/* please select GPWFM01A or GPWFM02A for wifi_module\r\n");
    DBG_PRINT("/* EXT_IRQ_POS = EXT_IRQ_MUX3( for EXTB -->IOD14) in board_config.h      */\r\n");
    DBG_PRINT("/* IF use MIPI sensor, please change MCLK to IOD12 (CDSP_CLKO_POS = CDSP_CLKO_MUX2)*/\r\n");
    DBG_PRINT("/* SPI pin mux IOD6~9                          */\r\n");
    DBG_PRINT("/* SPI CS change to S/W control (IOD6)         */\r\n");
    DBG_PRINT("/* WIFI RESET set to IOD15                     */\r\n");
    DBG_PRINT("/* CSI Reset change to IOD11                   */\r\n");
    DBG_PRINT("/* modify drv_l1_cfg.h-->_DRV_L1_SPI_SW_CS to 1*/\r\n");
    DBG_PRINT("/* select encode format select to MJPEG in avi_encoder_scaler_jpeg.h     /\r\n");
    DBG_PRINT("/* Comment out   #define VFRAME_MANAGER in avi_encoder_scaler_jpeg.h    /\r\n");
    DBG_PRINT("/**********************************************/\r\n");
	wifi_demo_flag = flag;
	wifi_demo_mode = wifi_mode;

	wifi_init_gpio();

	wifi_reset_module();
    if (wifi_module == GPWFM01A)
    {
	wifi_iamge_info.loader_start = (INT32U)&FW_LOADER_START;
	wifi_iamge_info.loader_end = (INT32U)&FW_LOADER_END;

	if(flag == WIFI_WS_SSL_DEMO)
	{
		wifi_iamge_info.app_start = (INT32U)&WIFI_APP_WS_START;
		wifi_iamge_info.app_end = (INT32U)&WIFI_APP_WS_END;
	}
	else if(flag == WIFI_SSL_DEMO)
	{
		wifi_iamge_info.app_start = (INT32U)&WIFI_APP_SSL_START;
		wifi_iamge_info.app_end = (INT32U)&WIFI_APP_SSL_END;
	}
	else if(flag == WIFI_TUTK_DEMO)
	{
		wifi_iamge_info.app_start = (INT32U)&WIFI_APP_TUTK_START;
		wifi_iamge_info.app_end = (INT32U)&WIFI_APP_TUTK_END;
	}
	else
	{
		wifi_iamge_info.app_start = (INT32U)&WIFI_APP_START;
		wifi_iamge_info.app_end = (INT32U)&WIFI_APP_END;
	}
    }
	/* Download WiFi module firmware */
	osDelay(1);
	if(gspi_fw_download_start(&wifi_iamge_info, wifi_module) != STATUS_OK)
	{
		DBG_PRINT("wifi_demo_init: download WiFi firmware failed\r\n");
		while(1);
	}

	/* Register WiFi state information call back */
	gp_cmd_regsiter_info_cbk((void*)wifi_state_info_cbk, GPCMD_WIFI_STATE_INFO);
	/* Register WiFi APP call back */
	gp_cmd_regsiter_info_cbk((void*)wifi_app_info_cbk, GPCMD_WIFI_APP_INFO);
	/* Register WiFi interface 0 setting information call back */
	gp_cmd_regsiter_info_cbk((void*)wifi_wifi0_setting_info_cbk, GPCMD_WIFI0_SETTING_INFO);
	/* Register WiFi DNS information call back */
	gp_cmd_regsiter_info_cbk((void*)wifi_dns_info_cbk, GPCMD_DNS_INFO);
	/* Register get AP clear channel number call back */
	gp_cmd_regsiter_info_cbk((void*)wifi_get_ap_clear_channel_cbk, GPCMD_GET_AP_CLEAR_CHANNEL_INFO);
	/* Register DHCP latest client IP response call back */
	gp_cmd_regsiter_info_cbk((void*)wifi_dhcp_client_ip_cbk, GPCMD_DHCP_LATEST_CLIENT_IP);

	wifi_start_network(wifi_demo_flag, wifi_demo_mode);

#if WAFD_DEMO_EN == 1
    GPM4_FDWA_HAND_VR_Demo();
#endif

#if PALM_DEMO_EN == 1
    GPM4_FD_Demo();
#endif

	if(wifi_demo_mode == WIFI_SC_MODE)
	{
		/* Running simple config demo */
		simple_config_task();
	}
	else
	{
		while(1)
		{
			if(wifi_demo_flag == WIFI_GP_SOCKET_DEMO || wifi_demo_flag == WIFI_SOCKET_MJPEG_DEMO)
			{
				/* process GP socket server loop */
				gp_socket_server_loop();
			}

			if(wifi_demo_flag == WIFI_SSL_DEMO)
			{
				/* Check IOA11 to do GPSSL enable or disable command */
				wifi_switch_ssl_app(WIFI_LOOP_WAIT_CNT);
				/* Check IOA12 to send TEST request */
				wifi_check_ssl_send_request(WIFI_LOOP_WAIT_CNT);
			}
			/* Sleep 1 sencond */
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSTimeDly(WIFI_LOOP_WAIT_CNT);
			#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osDelay(WIFI_LOOP_WAIT_CNT);
			#endif

#if (WIFI_CHANGE_IMAGE_DEMO == 1)
			wifi_check_change_image((WIFI_LOOP_WAIT_CNT));
#endif
		}
	}
}
