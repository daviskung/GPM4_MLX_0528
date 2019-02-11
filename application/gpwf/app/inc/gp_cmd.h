/***************************************************************************
* gp_cmd.h
*
* Purpose: For GP command to record the status in Ameba and send to GP MCU.
*
* Author: Eugene Hsu
*
* Date: 2015/09/24
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
******************************************************************************/
#ifndef __GP_CMD_H__
#define __GP_CMD_H__

typedef void(*GPCMD_INFO_CBK)(void* data);
#define GP_MAX_STATION_CONNECTION_NUM	3

/* FPS structure(NET & GSPI) */
 typedef struct GP_FPS_S
{
	INT32U net_fps;			/* FPS in wifi path */
	INT32S rssi;			/* RSSI */
} PACKED GP_FPS_T;

typedef struct GP_MAC_S
{
    char mac[6]; /*6 byte MAC address */
} PACKED GP_MAC_T;

typedef struct WIFI_CLIENT_INFO_S
{
	INT32U count;
	GP_MAC_T mac_list[GP_MAX_STATION_CONNECTION_NUM];
} PACKED WIFI_CLIENT_INFO_T;

/* WiFi setting structure */
typedef struct GP_WIFI_SETTING_S
{
	char	ifname[12];
	INT32U	mode;
	char 	ssid[33];
	char	channel;
	INT32U	security_type;
	char 	password[65];
	char	key_idx;
	INT8U	mac[6];
	INT8U	ip[4];
	INT8U	gw[4];
	WIFI_CLIENT_INFO_T client_info;
} PACKED GP_WIFI_SETTING_T;

/* WiFi status structure */
typedef struct GP_WIFI_STATE_S
{
	INT32U mode;
} PACKED GP_WIFI_STATE_T;

/* NET App service information structure */
typedef struct GP_NET_APP_S
{
	INT32U mjpeg_streamer_state;
	INT32U gp_socket_state;
	INT32U tcpcli_state;
	INT32U udpsrv_state;
	INT32U udpcli_state;
	INT32U goplus_state;
	INT32U rtspsrv_state;
	INT32U rtpsrv_state;
	INT32U websocket_state;
    INT32S sslcli_1_state;
	INT32S sslcli_2_state;
} PACKED GP_NET_APP_T;

/* DNS structure */
typedef struct GP_DNS_S
{
	INT32U dnsip;
	INT32U result;
} PACKED GP_DNS_T;

/* The client information in the transaction to UDP server */
typedef struct GP_UDP_SRV_CLI_INTFO_S
{
	INT8U sa_len;
	INT8U sa_family;
	char sa_data[14];	/* element 2,3,4,5 = IP address information */
} PACKED GP_UDP_SRV_CLI_INTFO_T;

/* Simple config structure */
typedef struct GP_SIMPLE_CONFIG_S
{
	/* API exposed to user */
	char ssid[32];
	char password[66];
	int  ip_addr;
	int  success;
} PACKED GP_SIMPLE_CONFIG_T;

/* RSSI structure */
typedef struct GP_RSSI_S
{
	INT32S rssi;
} PACKED GP_RSSI_T;

typedef struct GP_APP_REMOTE_IP_S
{
	INT32U gp_sock_client_ip;
} PACKED GP_APP_REMOTE_IP_T;

typedef struct GP_AP_AUTOCHANNEL_NUM_S
{
	int gp_ap_ac_num;
} PACKED GP_AP_AUTOCHANNEL_NUM_T;
typedef struct GP_DHCP_CLIENT_IP_S
{
	INT32U gp_sock_client_ip;
} PACKED GP_DHCP_CLIENT_IP_T;

typedef enum
{
	GPCMD_FPS_INFO,
	GPCMD_WIFI_STATE_INFO,
	GPCMD_WIFI_APP_INFO,
	GPCMD_WIFI0_SETTING_INFO,
	GPCMD_WIFI1_SETTING_INFO,
	GPCMD_SIMPLE_CONFIG_INFO,
	GPCMD_DNS_INFO,
	GPCMD_RSSI_INFO,
	GPCMD_APP_REMOTE_IP_INFO,
	GPCMD_GET_AP_CLEAR_CHANNEL_INFO,
	GPCMD_DHCP_LATEST_CLIENT_IP
} GPCMD_INFO_TYPE_E;

typedef enum
{
	WIFI_NONE_MODE = 0,
	WIFI_STATION_MODE,
	WIFI_AP_MODE,
	WIFI_STA_AP_MODE,
	WIFI_PROMISC_MODE,
	WIFI_P2P_MODE,
	WIFI_SC_MODE,						/* GP added for simple config mode */
	WIFI_TUTK_MODE,						/* GP added for TUTK station mode */
	WIFI_STATION_CONNECT_FAILE_MODE = 0x30,
	WIFI_WAIT_RES_MODE = 0xFF			/* Wait for response from GPWFM01A */

} WIFI_MODE_E;

typedef enum
{
	TCP_CLIENT_DISABLE_STATE = 1,
	TCP_CLIENT_DISCONNECT_STATE,
	TCP_CLIENT_CONNECT_READY_STATE,
	TCP_CLIENT_CONNECT_TX_STATE,
	TCP_CLIENT_ERROR_STATE
} TCP_CLIENT_STATE_E;

typedef enum
{
	UDP_CLIENT_DISABLE_STATE =1,
	UDP_CLIENT_DISCONNECT_STATE,
	UDP_CLIENT_CONNECT_READY_STATE,
	UDP_CLIENT_CONNECT_TX_STATE,
	UDP_CLIENT_ERROR_STATE
} UDP_CLIENT_STATE_E;

typedef enum
{
	UDP_SERVER_DISABLE_STATE = 1,
	UDP_SERVER_DISCONNECT_STATE,
	UDP_SERVER_CONNECT_READY_STATE,
	UDP_SERVER_CONNECT_TX_STATE
} UDP_SERVER_STATE_E;

typedef enum
{
	WEBSOCKET_OK_STATE = 1,
	WEBSOCKET_DISCONNECT_STATE,
	WEBSOCKET_CREATE_CLIENT_FAILED_STATE,
	WEBSOCKET_CONNECT_URL_FAILED_STATE
} WEBSOCKET_STATE_E;

typedef enum
{
	SSLCLI_CONNECT_STATE = 1,
	SSLCLI_DISCONNECT_STATE,
	SSLCLI_CLIENT_FAILED_STATE,
	SSLCLI_EOF_STATE
} SSLCLI_STATE_E;
#define WEP_ENABLED	  0x0001
#define TKIP_ENABLED  0x0002
#define AES_ENABLED   0x0004
#define WSEC_SWFLAG   0x0008

#define SHARED_ENABLED  0x00008000
#define WPA_SECURITY    0x00200000
#define WPA2_SECURITY   0x00400000
#define WPS_ENABLED     0x10000000

#define AT_CMD_SET_NETWORK_SSID				"ATW0=K220WiFi"
#define AT_CMD_SET_NETWORK_PASSPHRASE		"ATW1=1234567890"	/* WPA & WEP mode */
//#define AT_CMD_SET_NETWORK_SSID				"ATW0=Wifi-SDP"
//#define AT_CMD_SET_NETWORK_PASSPHRASE		"ATW1=P@$$vv0rd"	/* WPA & WEP mode */

#define AT_CMD_SET_KEY_ID					"ATW2="				/* For WEP mode */
#define AT_CMD_JOIN_NEWORK					"ATWC"

#define AT_CMD_SET_AP_SSID					"ATW3=GPLUSWIFI"
#define AT_CMD_SET_AP_SECURE_KEY			"ATW4=0987654321"
#define AT_CMD_SET_AP_CHANNEL				"ATW5=6"
#define AT_CMD_ACTIVATE_AP					"ATWA"	/* 1: ssid = NUll, 2: ssid = ......, other: ssid = ssid */
#define AT_CMD_WIFI_DISCONNECT				"ATWD"
#define AT_CMD_SIMPLE_CONFIG				"ATWQ"

#define GP_CMD_SET_AP_IP_ADDR				"GPAA=192.168.25.1"
#define GP_CMD_GET_WIFI0_SETTING			"GPGS=WIFI0SETTING"
#define GP_CMD_GET_WIFISTATUS				"GPGS=WIFISTATUS"
#define GP_CMD_GET_GP_SOCKET_REMOTE_IP		"GPGS=GPSOCKETCLIENTIP"
#define GP_CMD_GET_CLEAR_AP_CHANNEL_NUM		"GPGS=GETCLEARAPCHANNEL:1,2,3,4,5,6,7,8,9,10,11:11"	/* :ch1,ch2,ch3:total ch number */
#define GP_CMD_DNS_RESOLUTION				"GPDS="
#define GP_CMD_ENABLE_MJPEG_STREAMER		"GPMJ=1"
#define GP_CMD_DISABLE_MJPEG_STREAMER		"GPMJ=0"
#define GP_CMD_CONFIG_MJPEG_STREAMER_PORT	"GPMP=8080"
#define GP_CMD_ENABLE_GP_SOCKET				"GPSR=1"
#define GP_CMD_DISABLE_GP_SOCKET			"GPSR=0"
#define GP_CMD_CONFIG_GP_SOCKET_PORT		"GPSP=8090"
#define GP_CMD_ENABLE_TCP_CLIENT			"GPTC="
#define GP_CMD_DISABLE_TCP_CLIENT			"GPTC=0"
#define GP_CMD_ENABLE_UDP_CLIENT			"GPUC="
#define GP_CMD_DISABLE_UDP_CLIENT			"GPUC=0"
#define GP_CMD_ENABLE_UDP_SERVER			"GPUS="
#define GP_CMD_DISABLE_UDP_SERVER			"GPUS=0"
#define GP_CMD_GET_RSSI						"GPRS"
#define GP_CMD_ENABLE_GOPLUS_SOCKET			"GPGO=1:8081"
#define GP_CMD_DISABLE_GOPLUS_SOCKET		"GPGO=0"
#define GP_CMD_ENABLE_RTSP_RTP_MJPEG_SOCKET	"GPRP=MJ:320:240"
#define GP_CMD_ENABLE_RTSP_RTP_AUDIO_SOCKET	"GPRP=PCMA:8000:100"
#define GP_CMD_DISABLE_RTSP_RTP_SOCKET		"GPRP=0"
#define GP_CMD_ENABLE_TUTK					"GPTK=F3YUA14MY1RCBH6GYHF1"
#define GP_CMD_ENABLE_WEBSOCKET_SSL			"GPWS=wss://echo.websocket.org,443"
#define GP_CMD_DISABLE_WEBSOCKET_SSL		"GPWS=0"
#define GP_CMD_ENABLE_SSL_1					"GPS1=tw.yahoo.com:443"
#define GP_CMD_DISABLE_SSL_1				"GPS1=0"
#define GP_CMD_ENABLE_SSL_2					"GPS2=www.google.com:443"
#define GP_CMD_DISABLE_SSL_2				"GPS2=0"


typedef enum
{
    SECURITY_OPEN           = 0,                                                /**< Open security                           */
    SECURITY_WEP_PSK        = WEP_ENABLED,                                      /**< WEP Security with open authentication   */
    SECURITY_WEP_SHARED     = ( WEP_ENABLED | SHARED_ENABLED ),                 /**< WEP Security with shared authentication */
    SECURITY_WPA_TKIP_PSK   = ( WPA_SECURITY  | TKIP_ENABLED ),                 /**< WPA Security with TKIP                  */
    SECURITY_WPA_AES_PSK    = ( WPA_SECURITY  | AES_ENABLED ),                  /**< WPA Security with AES                   */
    SECURITY_WPA2_AES_PSK   = ( WPA2_SECURITY | AES_ENABLED ),                  /**< WPA2 Security with AES                  */
    SECURITY_WPA2_TKIP_PSK  = ( WPA2_SECURITY | TKIP_ENABLED ),                 /**< WPA2 Security with TKIP                 */
    SECURITY_WPA2_MIXED_PSK = ( WPA2_SECURITY | AES_ENABLED | TKIP_ENABLED ),   /**< WPA2 Security with AES & TKIP           */
    SECURITY_WPA_WPA2_MIXED = ( WPA_SECURITY  | WPA2_SECURITY ),                /**< WPA/WPA2 Security                       */

    SECURITY_WPS_OPEN       = WPS_ENABLED,                                      /**< WPS with open security                  */
    SECURITY_WPS_SECURE     = (WPS_ENABLED | AES_ENABLED),                      /**< WPS with AES security                   */

    SECURITY_UNKNOWN        = -1,                                               /**< May be returned by scan function if security is unknown. Do not pass this to the join function! */

     SECURITY_FORCE_32_BIT   = 0x7fffffff                                        /**< Exists only to force rtw_security_t type to 32 bits */
} SRCURITY_T;

/*	Extern variables and functions*/
void gp_cmd_regsiter_info_cbk(void* cbk, INT32U type);

extern GP_FPS_T	gp_fps;
extern GP_WIFI_STATE_T gp_wifi_state;
extern GP_WIFI_SETTING_T gp_wifi0_setting;
extern GP_WIFI_SETTING_T gp_wifi1_setting;
extern GP_SIMPLE_CONFIG_T gp_simple_config;
extern GP_NET_APP_T gp_net_app_state;
extern GP_DNS_T gp_dns;
extern GP_RSSI_T gp_rssi;
extern GP_APP_REMOTE_IP_T gp_remote_ip;
#endif	//__GP_CMD_H__
