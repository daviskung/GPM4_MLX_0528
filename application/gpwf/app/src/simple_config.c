/************************************************************
* simple_config.c
*
* Purpose: Use simple config to enable station mode then initialize network APP
*
* Author: Eugene Hsu
*
* Date: 2015/12/03
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
#define USE_PIN_CODE					0
#define SIMPLE_CONFIG_RSSI_TIME_CNT		25			/* 25*20ms */
#define SIMPLE_CONFIG_PIN				"12345678"	/* If pin code length < 8, it will be padded to 8. ie: when ping code is "123", actually working pin code is "12300000" */
#define SIMPLE_CONFIG_KEY				IO_A10
#define SIMPLE_CONFIG_LOOP_CNT			20			/* 200ms */
/**********************************************
*	Extern variables and functions
**********************************************/
extern INT32U wifi_demo_flag;
extern INT32U wifi_demo_mode;
extern void wifi_check_change_image(INT32U cnt);
/*********************************************
*	Variables declaration
*********************************************/
static char simple_cmd[32] = {0};
static INT32U sc_wifi_app_init = 0;
static INT32U do_simple_config = 0;
static INT8U sc_key_status = 0;

static void simple_config_init_key(void)
{
	gpio_init_io(SIMPLE_CONFIG_KEY, GPIO_INPUT);
	gpio_set_port_attribute(SIMPLE_CONFIG_KEY, 1);
}

static void simple_config_rssi_info_cbk(void* data)
{
	//GP_RSSI_T* rssiptr = (GP_RSSI_T*)data;
	//DBG_PRINT("SC RSSI of AP = %d\r\n", rssiptr->rssi);
}

static void simple_config_wifi0_setting_info_cbk(void* data)
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
}

static void simple_config_info_cbk(void* data)
{
	GP_SIMPLE_CONFIG_T* sptr = (GP_SIMPLE_CONFIG_T*)data;
	if(sptr->success)
	{
		/* Simple config success */
		DBG_PRINT("Simple config succuess\r\n");
		//DBG_PRINT("SSID:%s\r\n", gp_simple_config.ssid);
		//DBG_PRINT("Password:%s\r\n", gp_simple_config.password);
		DBG_PRINT("Config device IP:%d.%d.%d.%d\r\n", (sptr->ip_addr & 0x000000FF), (sptr->ip_addr & 0x0000FF00) >> 8,
												(sptr->ip_addr & 0x00FF0000) >> 16, (sptr->ip_addr & 0xFF000000) >> 24);
	}
	else
	{
		DBG_PRINT("Simple config failed\r\n");
	}
}

static void simple_config_key_process(void)
{
	static INT32U sckeycnt = 0;

	sc_key_status = gpio_read_io(SIMPLE_CONFIG_KEY);

	if(sc_key_status)
	{
		if(sckeycnt++ == 10)
		{
			DBG_PRINT("SC key pressed\r\n");
			do_simple_config = 1;
		}
	}
	else
	{
		sckeycnt = 0;
	}

	if(do_simple_config)
	{

		memset((void*)&gp_simple_config, 0, sizeof(gp_simple_config));
		/* Stop APP and wifi service in WiFi module */
		wifi_stop_app(wifi_demo_flag);
		wifi_stop_network();
		do_simple_config = 0;
		sc_wifi_app_init = 0;
#if (USE_PIN_CODE == 1)
		sprintf(simple_cmd, AT_CMD_SIMPLE_CONFIG "=%s", SIMPLE_CONFIG_PIN);
#else
		sprintf(simple_cmd, AT_CMD_SIMPLE_CONFIG);
#endif
		gspi_send_docmd(simple_cmd);
		DBG_PRINT("Do simple config\r\n");
	}
}

void simple_config_task(void)
{
	static INT32U cnt = 0;

	simple_config_init_key();

	gp_cmd_regsiter_info_cbk((void*)simple_config_info_cbk, GPCMD_SIMPLE_CONFIG_INFO);
	gp_cmd_regsiter_info_cbk((void*)simple_config_wifi0_setting_info_cbk, GPCMD_WIFI0_SETTING_INFO);
	gp_cmd_regsiter_info_cbk((void*)simple_config_rssi_info_cbk, GPCMD_RSSI_INFO);

	DBG_PRINT("Enter simple_config_task and wait for key pressed...\r\n");
	while(1)
	{
		/* Process Key */
		simple_config_key_process();

		/* Check AP RSSI if simple config is successful */
		if(gp_simple_config.success)
		{
			if(!sc_wifi_app_init)
			{
				sc_wifi_app_init = 1;
				/* Send WiFi interface 0 setting command */
				gspi_send_docmd(GP_CMD_GET_WIFI0_SETTING);
				/* Start WiFi APP according to the demo flag */
				wifi_start_app(wifi_demo_flag);
			}

			if(cnt++ == SIMPLE_CONFIG_RSSI_TIME_CNT)
			{
				gspi_send_docmd(GP_CMD_GET_RSSI);
				cnt = 0;
			}
		}
		/* Sleep 200 milisecond */
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(SIMPLE_CONFIG_LOOP_CNT);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay(SIMPLE_CONFIG_LOOP_CNT*10);
		#endif
#if (WIFI_CHANGE_IMAGE_DEMO == 1)
		wifi_check_change_image((SIMPLE_CONFIG_LOOP_CNT*10));
#endif
	}
}
