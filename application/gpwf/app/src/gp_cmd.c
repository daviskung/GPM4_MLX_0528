/************************************************************
* gp_cmd.c
*
* Purpose: GP command to get or set the Ameba status
*
* Author: Eugene Hsu
*
* Date: 2015/09/24
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
#include "gspi_master_drv.h"
#include "gspi_cmd.h"
#include "gp_cmd.h"

/**********************************************
*	Definitions
**********************************************/

/**********************************************
*	Extern variables and functions
**********************************************/
/*********************************************
*	Variables declaration
*********************************************/
#if (_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *gp_cmd_sem;
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
static osSemaphoreDef_t gpcmd_sem ={0};
static osSemaphoreId gp_cmd_sem;
#endif

static GPCMD_INFO_CBK gp_fps_info_cbk = NULL;
static GPCMD_INFO_CBK gp_rssi_info_cbk = NULL;
static GPCMD_INFO_CBK gp_dns_info_cbk = NULL;
static GPCMD_INFO_CBK gp_wifi_state_info_cbk = NULL;
static GPCMD_INFO_CBK gp_wifi_app_info_cbk = NULL;
static GPCMD_INFO_CBK gp_simple_config_info_cbk = NULL;
static GPCMD_INFO_CBK gp_wifi0_setting_info_cbk = NULL;
static GPCMD_INFO_CBK gp_wifi1_setting_info_cbk = NULL;
static GPCMD_INFO_CBK gp_app_remote_ip_cbk = NULL;
static GPCMD_INFO_CBK gp_get_ap_clear_channel_cbk = NULL;
static GPCMD_INFO_CBK gp_dhcp_latest_client_ip_cbk = NULL;

GP_WIFI_STATE_T		gp_wifi_state = {0};
GP_WIFI_SETTING_T	gp_wifi0_setting = {0};
GP_WIFI_SETTING_T	gp_wifi1_setting = {0};
GP_NET_APP_T gp_net_app_state = {0};
GP_SIMPLE_CONFIG_T gp_simple_config = {0};
GP_DNS_T gp_dns = {0};
GP_FPS_T gp_fps = {0};
GP_RSSI_T gp_rssi = {0};
GP_APP_REMOTE_IP_T gp_remote_ip = {0};
GP_AP_AUTOCHANNEL_NUM_T gp_ap_auto_channel = {0};
GP_DHCP_CLIENT_IP_T gp_dhcp_latest_client_ip = {0};

/**********GP command API**************/
void gp_cmd_regsiter_info_cbk(void* cbk, INT32U type)
{
	switch(type)
	{
		case GPCMD_FPS_INFO:
			gp_fps_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_DNS_INFO:
			gp_dns_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_WIFI_STATE_INFO:
			gp_wifi_state_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_RSSI_INFO:
			gp_rssi_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_SIMPLE_CONFIG_INFO:
			gp_simple_config_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_WIFI0_SETTING_INFO:
			gp_wifi0_setting_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_WIFI1_SETTING_INFO:
			gp_wifi1_setting_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_WIFI_APP_INFO:
			gp_wifi_app_info_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_APP_REMOTE_IP_INFO:
			gp_app_remote_ip_cbk = (GPCMD_INFO_CBK)cbk;
			break;

		case GPCMD_GET_AP_CLEAR_CHANNEL_INFO:
			gp_get_ap_clear_channel_cbk = (GPCMD_INFO_CBK)cbk;
			break;
		case GPCMD_DHCP_LATEST_CLIENT_IP:
			gp_dhcp_latest_client_ip_cbk = (GPCMD_INFO_CBK)cbk;
			break;
	}
}

void gp_cmd_data_lock(void)
{
#if(_OPERATING_SYSTEM == _OS_UCOS2)
	INT8U err;
	OSSemPend(gp_cmd_sem, 0, &err);
#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
	osSemaphoreWait(gp_cmd_sem, osWaitForever);
	#endif
}

void gp_cmd_data_unlock(void)
{
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSSemPost(gp_cmd_sem);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osSemaphoreRelease(gp_cmd_sem);
	#endif
}

void gp_cmd_gspi_cbk(INT32U buf, INT32U len, INT32U event)
{
	int* rssiptr;
	GP_FPS_T* fpsptr;
	GP_WIFI_SETTING_T* wifi_setting_ptr;
	GP_APP_REMOTE_IP_T* app_ip_ptr;
	GP_AP_AUTOCHANNEL_NUM_T* clear_ch_ptr;
	GP_DHCP_CLIENT_IP_T* dhcp_client_ip_ptr;

	gp_cmd_data_lock();

	switch(event)
	{
		case GSPI_FPS_RES:
			fpsptr = (GP_FPS_T*)buf;
			gp_fps.net_fps = fpsptr->net_fps;
			gp_fps.rssi = fpsptr->rssi;
			if(gp_fps_info_cbk)
			{
				gp_fps_info_cbk((void*)&gp_fps);
			}
			break;

		case GSPI_RSSI_RES:
			rssiptr = (int*)buf;
			gp_rssi.rssi = *rssiptr;
			if(gp_rssi_info_cbk)
			{
				gp_rssi_info_cbk((void*)&gp_rssi);
			}
			break;

		case GSPI_APP_REMOTE_IP_RES:
			app_ip_ptr = (GP_APP_REMOTE_IP_T*)buf;
			memcpy((void*)&gp_remote_ip, (void*)app_ip_ptr, sizeof(GP_APP_REMOTE_IP_T));
			if(gp_app_remote_ip_cbk)
			{
				gp_app_remote_ip_cbk((void*)&gp_remote_ip);
			}
			break;
		case GSPI_DHCP_LATEST_CLIENT_IP_RES:
			dhcp_client_ip_ptr = (GP_DHCP_CLIENT_IP_T*)buf;
			memcpy((void*)&gp_dhcp_latest_client_ip, (void*)dhcp_client_ip_ptr, sizeof(GP_DHCP_CLIENT_IP_T));
			if(gp_dhcp_latest_client_ip_cbk)
			{
				gp_dhcp_latest_client_ip_cbk((void*)&gp_dhcp_latest_client_ip);
			}
			break;

		case GSPI_AP_AUTOCHANNEL_NUM_RES:
			clear_ch_ptr = (GP_AP_AUTOCHANNEL_NUM_T*)buf;
			memcpy((void*)&gp_ap_auto_channel, (void*)clear_ch_ptr, sizeof(GP_AP_AUTOCHANNEL_NUM_T));
			if(gp_get_ap_clear_channel_cbk)
			{
				gp_get_ap_clear_channel_cbk((void*)&gp_ap_auto_channel);
			}
			break;

		case GSPI_SIMPLE_CONFIG_RES:
			memcpy((void*)&gp_simple_config, (void*)buf, sizeof(GP_SIMPLE_CONFIG_T));
			if(gp_simple_config_info_cbk)
			{
				gp_simple_config_info_cbk((void*)&gp_simple_config);
			}
			break;

		case GSPI_DNS_RES:
			memcpy((void*)&gp_dns, (void*)buf, sizeof(GP_DNS_T));
			if(gp_dns_info_cbk)
			{
				gp_dns_info_cbk((void*)&gp_dns);
			}
			break;

		case GSPI_WIFI_SETTING_RES:
			wifi_setting_ptr = (GP_WIFI_SETTING_T*)buf;
			if(!strcmp(wifi_setting_ptr->ifname, WLAN0_NAME))
			{
				/* Copy WiFi setting from WiFi module */
				memcpy((void*)&gp_wifi0_setting, (void*)buf, sizeof(GP_WIFI_SETTING_T));

				if(gp_wifi0_setting_info_cbk)
				{
					gp_wifi0_setting_info_cbk((void*)&gp_wifi0_setting);
				}
			}
			else if(!strcmp(wifi_setting_ptr->ifname, WLAN1_NAME))
			{
				/* Copy WiFi setting from WiFi module */
				memcpy((void*)&gp_wifi1_setting, (void*)buf, sizeof(GP_WIFI_SETTING_T));
				if(gp_wifi1_setting_info_cbk)
				{
					gp_wifi1_setting_info_cbk((void*)&gp_wifi1_setting);
				}
			}
			break;

		case GSPI_AP_MODE_RES:
			gp_wifi_state.mode = WIFI_AP_MODE;
			if(gp_wifi_state_info_cbk)
			{
				gp_wifi_state_info_cbk((void*)&gp_wifi_state);
			}
			break;

		case GSPI_NONE_MODE_RES:
			gp_wifi_state.mode = WIFI_NONE_MODE;
			if(gp_wifi_state_info_cbk)
			{
				gp_wifi_state_info_cbk((void*)&gp_wifi_state);
			}
			break;

		case GSPI_STATION_MODE_RES:
			gp_wifi_state.mode = WIFI_STATION_MODE;
			if(gp_wifi_state_info_cbk)
			{
				gp_wifi_state_info_cbk((void*)&gp_wifi_state);
			}
			break;

		case GSPI_STATION_CONNECT_FAILED_MODE_RES:
			gp_wifi_state.mode = WIFI_STATION_CONNECT_FAILE_MODE;
			if(gp_wifi_state_info_cbk)
			{
				gp_wifi_state_info_cbk((void*)&gp_wifi_state);
			}
			break;

		case GSPI_NET_APP_STATE_RES:
			memcpy((void*)&gp_net_app_state, (void*)buf, sizeof(GP_NET_APP_T));
			if(gp_wifi_app_info_cbk)
			{
				gp_wifi_app_info_cbk((void*)&gp_net_app_state);
			}
			break;

		default:
			break;
	}
	gp_cmd_data_unlock();
}


void gp_cmd_init(void)
{
#if (_OPERATING_SYSTEM == _OS_UCOS2)
	gp_cmd_sem = OSSemCreate(1);
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	gp_cmd_sem = osSemaphoreCreate(&gpcmd_sem, 1);
#endif
	/* Register GP command call back into GSPI */
	gspi_register_app_cbk((INT32U)gp_cmd_gspi_cbk, GSPI_GP_CMD_APP);
}
