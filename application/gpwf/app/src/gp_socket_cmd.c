/************************************************************
* gp_socket_cmd.c
*
* Purpose: GP socket command APP
*
* Author: Eugene Hsu
*
* Date: 2015/10/01
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
#include "gspi_cmd.h"
#include "gp_cmd.h"
#include "drv_l2_display.h"

/**********************************************
*	Definitions
**********************************************/
typedef enum
{
	GP_SOCK_CMD_GENERAL_GETDEVICESTATUS 	= 0x0000,
	GP_SOCK_CMD_GENERAL_POWEROFF			= 0x0001,
	GP_SOCK_CMD_GENERAL_AUTHDEVICE			= 0x0002,
	GP_SOCK_CMD_GENERAL_GETINDEXLIST		= 0x0003,

	GP_SOCK_CMD_PLAY_START	 	  			= 0x0100,
	GP_SOCK_CMD_PLAY_PAUSERESUME  			= 0x0101,
	GP_SOCK_CMD_PLAY_STOP	 	  			= 0x0102,

	GP_SOCK_CMD_RECORD_START 	  			= 0x0200,
	GP_SOCK_CMD_RECORD_STOP 	  			= 0x0201,

	GP_SOCK_CMD_DATATRANSFER_START 			= 0x0300,
	GP_SOCK_CMD_DATATRANSFER_STOP 			= 0x0301,
	GP_SOCK_CMD_DATATRANSFER_RAWDATA 		= 0x0302,

	GP_SOCK_CMD_LOOPBACKTEST 				= 0xF000,

	GP_SOCK_CMD_VENDOR						= 0xFF00
} GP_SOCK_CMD_E;

#define GP_SOCK_CMD_TYPE	0x0001
#define GP_SOCK_ACK_TYPE	0x0002
#define GP_SOCK_NAK_TYPE	0x0003

#define	GP_CMD_TAG_SIZE		8
#define	GP_CMD_TYPE_SIZE	2
#define	GP_CMD_MODE_SIZE	1
#define	GP_CMD_ID_SIZE		1
#define	GP_CMD_PAYLAOD_SIZE	2
#define GP_CMD_HEADER_SIZE			(GP_CMD_TAG_SIZE+GP_CMD_TYPE_SIZE+GP_CMD_MODE_SIZE+GP_CMD_ID_SIZE+GP_CMD_PAYLAOD_SIZE)
#define GP_CMD_PAYLOAD_BUF_SIZE		256
#define GP_CMD_CMD_BUF_SIZE			(GP_CMD_HEADER_SIZE + GP_CMD_PAYLOAD_BUF_SIZE)

#define GP_SOCK_TAG0	'G'
#define GP_SOCK_TAG1	'P'
#define GP_SOCK_TAG2	'S'
#define GP_SOCK_TAG3	'O'
#define GP_SOCK_TAG4	'C'
#define GP_SOCK_TAG5	'K'
#define GP_SOCK_TAG6	'E'
#define GP_SOCK_TAG7	'T'

#define GP_SOCK_IP_TOKEN0	0xF0
#define GP_SOCK_IP_TOKEN1	0x01

#define GP_SOCK_CMD_START_DIR_M2S	0x00
#define GP_SOCK_CMD_START_DIR_S2M	0x01
#define GP_SOCK_CMD_START_DIR_NONE	0xFF

#define GP_SOCKET_GET_REMOTE_IP_EVENT	0x300

typedef struct GP_CMD_DATA_S
{
	char tag[GP_CMD_TAG_SIZE];
	char type[GP_CMD_TYPE_SIZE];
	char mode;
	char cmdid;
	char payloadsize[GP_CMD_PAYLAOD_SIZE];
	char payload[GP_CMD_PAYLOAD_BUF_SIZE];
} PACKED GP_CMD_DATA_T;

#define GPSOCKCMD_STACK_SIZE		1024
#define GP_SOCK_RAW_DATA_BUF_SIZE	(100*1024)
#define GP_SOCK_RAW_DATA_TRANSFER_SIZE	(10*1024)
#define DISPLAY_DEV_HPIXEL 320
#define DISPLAY_DEV_VPIXEL 240
#define GP_SOCKET_Q_NUM		16
#define GP_SOCKET_RX_BUF_SIZE		2048
#define GP_SOCKET_UDP_SERVER_TIMEOUT	60
#define GP_SOCKET_UDP_SERVER_PORT		"8090"

#define GP_SOCKET_UDP_NO_ACTION				0
#define GP_SOCKET_UDP_CONNECT_TO_SERVER		1
#define GP_SOCKET_UDP_DISCONNECT_TO_SERVER	2
/**********************************************
*	Extern variables and functions
**********************************************/
/*********************************************
*	Variables declaration
*********************************************/
#if(_OPERATING_SYSTEM == _OS_UCOS2)
static OS_EVENT *gp_sock_q = NULL;
static OS_EVENT *gp_socket_udp_rx_sem = NULL;
static void *gp_sock_q_stack[GP_SOCKET_Q_NUM];
static OS_EVENT *gp_socket_cmd_tx_sem;
static INT32U GPSocketCmdTaskStack[GPSOCKCMD_STACK_SIZE];
#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
static osMessageQDef_t gp_sock_q_d = {GP_SOCKET_Q_NUM, sizeof(INT32U),0};
static osMessageQId gp_sock_q = NULL;
static osSemaphoreDef_t gp_socket_udp_rx_sem_d = {0};
static osSemaphoreId gp_socket_udp_rx_sem;
static osSemaphoreDef_t gp_socket_cmd_tx_sem_d = {0};
static osSemaphoreId gp_socket_cmd_tx_sem;
#endif
static INT8U gp_sock_rx_buf[GP_CMD_CMD_BUF_SIZE];
static INT8U gp_sock_tx_buf[GP_CMD_CMD_BUF_SIZE];
static INT8U gp_sock_raw_data_buf[GP_SOCK_RAW_DATA_BUF_SIZE];
static INT8U* gp_raw_data_ptr = NULL;
static INT32U gp_sock_rx_buf_len = 0;
static INT8U *gp_sock_read_ptr = 0;
static INT32U gpsock_first_init = 0;
//INT32U gp_sock_decode_output_ptr = NULL;
INT8U *gp_sock_decode_output_ptr = NULL;
INT32U gp_sock_udp_rx_len = 0;
INT32U gp_sock_connect_state = 0;
INT32U gp_sock_file_dir = GP_SOCK_CMD_START_DIR_NONE;
INT32U gp_sock_file_len = 0x400;
INT32U gp_socket_connect_to_upd_server = GP_SOCKET_UDP_NO_ACTION;
static INT8U gp_sock_udp_rx_buf[GP_SOCKET_RX_BUF_SIZE] = {0};
static INT8U gp_sock_test_buf[2048] = {0};
char gp_socket_client_ip[24] = {0};
static INT32U send_udpcli = 0;

INT32U (*videnc_display2)(INT16U w, INT16U h, INT32U frame_addr);
void gp_socket_cmd_display_callback(INT32U (*disp)(INT16U, INT16U, INT32U))
{
	if(disp) {
		videnc_display2 = disp;
	}
}

static INT32U Display_Callback(INT16U w, INT16U h, INT32U addr)
{
    R_TFT_FBI_ADDR = addr;
}

static void _send_upload_file_to_gp_socket(INT32U buf, INT32U len)
{
	INT32U total;
	GP_CMD_DATA_T* gp_sock_cmd_tx_ptr = (GP_CMD_DATA_T*)gp_sock_tx_buf;

	/* Send ACK back to GP socket */
	memset(gp_sock_tx_buf, 0 ,sizeof(gp_sock_tx_buf));
	gp_sock_cmd_tx_ptr->tag[0] = GP_SOCK_TAG0;
	gp_sock_cmd_tx_ptr->tag[1] = GP_SOCK_TAG1;
	gp_sock_cmd_tx_ptr->tag[2] = GP_SOCK_TAG2;
	gp_sock_cmd_tx_ptr->tag[3] = GP_SOCK_TAG3;
	gp_sock_cmd_tx_ptr->tag[4] = GP_SOCK_TAG4;
	gp_sock_cmd_tx_ptr->tag[5] = GP_SOCK_TAG5;
	gp_sock_cmd_tx_ptr->tag[6] = GP_SOCK_TAG6;
	gp_sock_cmd_tx_ptr->tag[7] = GP_SOCK_TAG7;

	/* ACK packet */
	gp_sock_cmd_tx_ptr->type[0] = 0x02;
	gp_sock_cmd_tx_ptr->type[1] = 0x00;

	/* command RAW data */
	gp_sock_cmd_tx_ptr->mode = 0x03;
	gp_sock_cmd_tx_ptr->cmdid = 0x02;


	/* For file upload, fill file size */
	gp_sock_cmd_tx_ptr->payloadsize[0] = (len&0xFF);
	gp_sock_cmd_tx_ptr->payloadsize[1] = (len&0xFF00) >> 8;

	total = GP_CMD_HEADER_SIZE + len;

	memcpy((gp_sock_tx_buf+GP_CMD_HEADER_SIZE), (void*)buf, len);

	gspi_tx_data(gp_sock_tx_buf, total, DATA_GP_SOCK_CMD_TYPE);
}

static void _send_handshake_back_to_gp_socket(INT32U type)
{
	GP_CMD_DATA_T* gp_sock_cmd_tx_ptr = (GP_CMD_DATA_T*)gp_sock_tx_buf;
	GP_CMD_DATA_T* gp_sock_cmd_rx_ptr = (GP_CMD_DATA_T*)gp_sock_rx_buf;
	INT8U err;
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSSemPend(gp_socket_cmd_tx_sem, 0, &err);
    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    osSemaphoreWait(gp_socket_cmd_tx_sem, osWaitForever);
    #endif
	/* Send ACK back to GP socket */
	memset(gp_sock_tx_buf, 0 ,sizeof(gp_sock_tx_buf));
	gp_sock_cmd_tx_ptr->tag[0] = GP_SOCK_TAG0;
	gp_sock_cmd_tx_ptr->tag[1] = GP_SOCK_TAG1;
	gp_sock_cmd_tx_ptr->tag[2] = GP_SOCK_TAG2;
	gp_sock_cmd_tx_ptr->tag[3] = GP_SOCK_TAG3;
	gp_sock_cmd_tx_ptr->tag[4] = GP_SOCK_TAG4;
	gp_sock_cmd_tx_ptr->tag[5] = GP_SOCK_TAG5;
	gp_sock_cmd_tx_ptr->tag[6] = GP_SOCK_TAG6;
	gp_sock_cmd_tx_ptr->tag[7] = GP_SOCK_TAG7;

	if(type == GP_SOCK_ACK_TYPE)
	{
		gp_sock_cmd_tx_ptr->type[0] = 0x02;
		gp_sock_cmd_tx_ptr->type[1] = 0x00;
	}
	else if(type == GP_SOCK_NAK_TYPE)
	{
		gp_sock_cmd_tx_ptr->type[0] = 0x03;
		gp_sock_cmd_tx_ptr->type[1] = 0x00;
	}

	gp_sock_cmd_tx_ptr->mode = gp_sock_cmd_rx_ptr->mode;
	gp_sock_cmd_tx_ptr->cmdid = gp_sock_cmd_rx_ptr->cmdid;

	if(gp_sock_file_dir == GP_SOCK_CMD_START_DIR_S2M)
	{
		/* For file upload, fill file size */
		gp_sock_cmd_tx_ptr->payloadsize[0] = 0x04;
		gp_sock_cmd_tx_ptr->payloadsize[1] = 0x00;
		gp_sock_cmd_tx_ptr->payload[0] = gp_sock_file_len & 0xFF;
		gp_sock_cmd_tx_ptr->payload[1] = (gp_sock_file_len & 0x0000FF00) >> 8;
		gp_sock_cmd_tx_ptr->payload[2] = (gp_sock_file_len & 0x00FF0000) >> 16;
		gp_sock_cmd_tx_ptr->payload[3] = (gp_sock_file_len & 0xFF000000) >> 24;
		gspi_tx_data(gp_sock_tx_buf, GP_CMD_HEADER_SIZE + 4, DATA_GP_SOCK_CMD_TYPE);
	}
	else
	{
		/* Set payload size = 0 */
		gp_sock_cmd_tx_ptr->payloadsize[0] = 0x00;
		gp_sock_cmd_tx_ptr->payloadsize[1] = 0x00;
		gspi_tx_data(gp_sock_tx_buf, GP_CMD_HEADER_SIZE, DATA_GP_SOCK_CMD_TYPE);
	}
}

static void start_decode_image(void)
{
	if(gp_sock_decode_output_ptr == NULL)
	{
		//gp_sock_decode_output_ptr = (INT32U) gp_malloc_align((DISPLAY_DEV_HPIXEL*DISPLAY_DEV_VPIXEL)*2, 64);//malloc decode frame buffer
		gp_sock_decode_output_ptr = (INT8U*) gp_malloc_align((DISPLAY_DEV_HPIXEL*DISPLAY_DEV_VPIXEL)*2, 64);//malloc decode frame buffer
		if(gp_sock_decode_output_ptr == NULL)
		{
			DBG_PRINT("gp_sock_decode_output_ptr malloc fail\r\n");
		}
	}
//	tft_init();
//	tft_start(C_DISPLAY_DEVICE);
// ming test LCD
    drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);
    gp_socket_cmd_display_callback(Display_Callback);
}

static void stop_decode_image(void)
{
	if(gp_sock_decode_output_ptr)
	{
		memset((INT8U*)gp_sock_decode_output_ptr, 0x0, (DISPLAY_DEV_HPIXEL*DISPLAY_DEV_VPIXEL)*2);
//		//video_codec_show_image(C_DISPLAY_DEVICE, (INT32U)gp_sock_decode_output_ptr, C_DISPLAY_DEVICE, IMAGE_OUTPUT_FORMAT_RGB565);
//        if(videnc_display) {
//		  		videnc_display(pAviEncVidPara->display_buffer_width,
//                               pAviEncVidPara->display_buffer_height,
//                               display_frame);
//         }
        #if(_OPERATING_SYSTEM == _OS_UCOS2)
		OSTimeDly(5);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osDelay(5*10);
		#endif
//		tft_init();
        drv_l2_display_init();

		gp_free((void*)gp_sock_decode_output_ptr);
		gp_sock_decode_output_ptr = NULL;
	}
}

static void decode_jpg_buffer_to_tft(INT32U buf_address, INT32U h_pixel, INT32U v_pixel)
{
	IMAGE_ARGUMENT image_decode;
	MEDIA_SOURCE image_source;
	INT32U decode_state;
	DBG_PRINT("\r\nGP socket Decode ");

	image_source.type = SOURCE_TYPE_SDRAM;
	image_source.type_ID.memptr = (INT8U*)buf_address;

	image_decode.ScalerOutputRatio=FIT_OUTPUT_SIZE;
	image_decode.OutputFormat=IMAGE_OUTPUT_FORMAT_YUYV;                  //scaler out format
	//image output information
	image_decode.OutputBufPtr=(INT8U *)gp_sock_decode_output_ptr;            //decode output buffer
	image_decode.OutputBufWidth=h_pixel;                  //width of output buffer
	image_decode.OutputBufHeight=v_pixel;                 //Heigh of output buffer
	image_decode.OutputWidth=h_pixel;                     //scaler width of output image
	image_decode.OutputHeight=v_pixel;                    //scaler Heigh of output image

	//image decode function
	image_decode_entrance();     //global variable initial for image decode
	image_decode_start(image_decode,image_source);    //image decode start
	while (1) {
		 decode_state = image_decode_status();
		 if (decode_state == IMAGE_CODEC_DECODE_END) {
//			 video_codec_show_image(C_DISPLAY_DEVICE, (INT32U)gp_sock_decode_output_ptr, C_DISPLAY_DEVICE, IMAGE_OUTPUT_FORMAT_YUYV);
             if(videnc_display2) {
                               videnc_display2(image_decode.OutputWidth,
                               image_decode.OutputHeight,
                               (INT32U)gp_sock_decode_output_ptr);
             }
			 break;
		  }else if(decode_state == IMAGE_CODEC_DECODE_FAIL) {
			 DBG_PRINT("image decode failed\r\n");
			 break;
		  }
	}

	image_decode_stop();
}

void process_gp_socket_cmd(INT8U* buf)
{
	INT32U loopback_len, payloadsize;
	INT16U gp_cmd;
	INT8U err;
	GP_CMD_DATA_T* gp_sock_cmd_ptr = (GP_CMD_DATA_T*)buf;

	if(gp_sock_cmd_ptr->tag[0] == GP_SOCK_TAG0 && gp_sock_cmd_ptr->tag[1] == GP_SOCK_TAG1 && gp_sock_cmd_ptr->tag[2] == GP_SOCK_TAG2
					&& gp_sock_cmd_ptr->tag[3] == GP_SOCK_TAG3 && gp_sock_cmd_ptr->tag[4] == GP_SOCK_TAG4 && gp_sock_cmd_ptr->tag[5] == GP_SOCK_TAG5
					&& gp_sock_cmd_ptr->tag[6] == GP_SOCK_TAG6 && gp_sock_cmd_ptr->tag[7] == GP_SOCK_TAG7)
	{
		gp_cmd = ((gp_sock_cmd_ptr->mode) << 8) | gp_sock_cmd_ptr->cmdid;
		DBG_PRINT("GP socket cmd 0x%x got\r\n", gp_cmd);

		if(gp_cmd == GP_SOCK_CMD_LOOPBACKTEST)
		{
			loopback_len = (gp_sock_cmd_ptr->payloadsize[1] << 8) | gp_sock_cmd_ptr->payloadsize[0];
			//DBG_PRINT("GPSOCKET cmd got: 0x%x, loopback len %d, buf len %d\r\n", gp_cmd, loopback_len, gp_sock_rx_buf_len);

			/* Write back to GP socket client */
			memset(gp_sock_tx_buf, 0 ,sizeof(gp_sock_tx_buf));
			memcpy((char*)gp_sock_tx_buf, (char*)gp_sock_rx_buf, gp_sock_rx_buf_len);
			#if(_OPERATING_SYSTEM == _OS_UCOS2)
			OSSemPend(gp_socket_cmd_tx_sem, 0, &err);
			#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
			osSemaphoreWait(gp_socket_cmd_tx_sem, osWaitForever);
			#endif
			/* Send back for loopback test */
			gspi_tx_data(gp_sock_tx_buf, gp_sock_rx_buf_len, DATA_GP_SOCK_CMD_TYPE);
		}
		else
		{
			switch(gp_cmd)
			{
				case GP_SOCK_CMD_PLAY_START:
					break;

				case GP_SOCK_CMD_PLAY_PAUSERESUME:
					break;

				case GP_SOCK_CMD_PLAY_STOP:
					gp_sock_read_ptr = 0;
					/* reset file direction */
					gp_sock_file_dir = GP_SOCK_CMD_START_DIR_NONE;
					break;

				case GP_SOCK_CMD_DATATRANSFER_START:
					gp_sock_file_dir = gp_sock_cmd_ptr->payload[0];
					/* Reset RX raw buffer */
					memset(gp_sock_raw_data_buf, 0, GP_SOCK_RAW_DATA_BUF_SIZE);

					if(gp_sock_file_dir == GP_SOCK_CMD_START_DIR_M2S)
					{
						gp_raw_data_ptr = (INT8U*)gp_sock_raw_data_buf;
						gp_sock_read_ptr = gp_sock_raw_data_buf;
						DBG_PRINT("DATA start prt 0x%x\r\n", gp_raw_data_ptr);
					}
					else if(gp_sock_file_dir == GP_SOCK_CMD_START_DIR_S2M)
					{
						DBG_PRINT("Upload start...\r\n");
					}
					break;

				case GP_SOCK_CMD_DATATRANSFER_RAWDATA:
					if(gp_sock_file_dir == GP_SOCK_CMD_START_DIR_M2S)
					{
						payloadsize = (gp_sock_cmd_ptr->payloadsize[1] << 8) | gp_sock_cmd_ptr->payloadsize[0];
						//DBG_PRINT("Payloadsize = 0x%x\r\n", payloadsize);
						/* Copy raw data to buffer */
						memcpy(gp_raw_data_ptr, (INT8U*)(gp_sock_cmd_ptr->payload), payloadsize);
						gp_raw_data_ptr += payloadsize;
					}
					else if(gp_sock_file_dir == GP_SOCK_CMD_START_DIR_S2M)
					{
						DBG_PRINT("Upload data transfer phase\r\n");
						/* No normal ACK phase */
						_send_upload_file_to_gp_socket((INT32U)gp_sock_test_buf, 0x400);
						return;
					}
					break;

				case GP_SOCK_CMD_DATATRANSFER_STOP:
					start_decode_image();
					decode_jpg_buffer_to_tft((INT32U)gp_sock_raw_data_buf, DISPLAY_DEV_HPIXEL, DISPLAY_DEV_VPIXEL);
					//DBG_PRINT("Raw data stop: total length 0x%x\r\n", (gp_raw_data_ptr - gp_sock_raw_data_buf));
					break;

				case GP_SOCK_CMD_VENDOR:
#if 0
					payloadsize = (gp_sock_cmd_ptr->payloadsize[1] << 8) | gp_sock_cmd_ptr->payloadsize[0];
					if(gp_sock_cmd_ptr->payload[0] == GP_SOCK_IP_TOKEN0 && gp_sock_cmd_ptr->payload[1] == GP_SOCK_IP_TOKEN1)
					{
						/* This is set IP address command */
						memset((char*)gp_socket_client_ip, 0, sizeof(gp_socket_client_ip));
						sprintf(gp_socket_client_ip, "%d.%d.%d.%d", (gp_sock_cmd_ptr->payload[2] & 0xFF), (gp_sock_cmd_ptr->payload[3] & 0xFF),
																	 (gp_sock_cmd_ptr->payload[4] & 0xFF), (gp_sock_cmd_ptr->payload[5] & 0xFF));
						DBG_PRINT("GP sokcet client IP address = %s\r\n", gp_socket_client_ip);
						gp_socket_connect_to_upd_server = GP_SOCKET_UDP_CONNECT_TO_SERVER;
					}
#endif
					break;

				default:
					break;
			}

			/* Send ACK back to GP socket */
			_send_handshake_back_to_gp_socket(GP_SOCK_ACK_TYPE);
		}
	}
}

void gp_socket_cmd_gspi_cbk(INT8U* buf, INT32U len, INT32U event)
{
	INT32U msg = event;
	INT32U size = len;

	//DBG_PRINT("gp_socket_cmd_gspi_cbk len %d, event 0x%x\r\n", len , event);

	if(event == GP_SOCK_RXDONE_EVENT)
	{
		if(len)
		{
			if(size > GP_CMD_CMD_BUF_SIZE)
				size = GP_CMD_CMD_BUF_SIZE;

			//DBG_PRINT("Got GP socket cmd buf len %d\r\n", len);
			gp_sock_rx_buf_len = size;
			memcpy(gp_sock_rx_buf, buf, gp_sock_rx_buf_len);
		}
	}
	else if(event == GP_SOCK_RAW_DATA_DONE_EVENT)
	{
		if(len)
		{
			gp_sock_rx_buf_len = size;
		}
	}
	else if(event == GP_SOCK_RAW_DATA_SEND_BACK_EVENT)
	{
		//DBG_PRINT("GP_SOCK_RAW_DATA_SEND_BACK_EVENT len %d, reamin %d\r\n", len, gp_sock_rx_buf_len);
		memcpy(gp_sock_read_ptr, buf, len);
		gp_sock_read_ptr += len;
		gp_sock_rx_buf_len -= len;

		if(gp_sock_rx_buf_len == 0)
		{
			msg = GP_SOCK_RAW_DATA_SEND_BACK_DONE_EVENT;
		}
	}
	//DBG_PRINT("gp_socket_cmd_gspi_cbk event %d\r\n", event);
	#if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(gp_sock_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut(gp_sock_q, (uint32_t) &msg, osWaitForever);
	#endif
}

void gp_sock_cmd_task(void *parm)
{
	INT32U msg_id, i;
	INT8U err;

	/* For test only */
	for(i=0;i<2048;i++)
	{
		gp_sock_test_buf[i] = i & 0xFF;
	}

	while(1)
	{
        #if(_OPERATING_SYSTEM ==_OS_UCOS2)
		msg_id = (INT32U) OSQPend(gp_sock_q, 0, &err);
		if(err != OS_NO_ERR)
			continue;
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osEvent event;
		event = osMessageGet(gp_sock_q, osWaitForever);
		msg_id = event.value.v;
		if (event.status != osEventMessage){
            continue;
		}
		#endif
		switch(msg_id)
		{
			case GP_SOCK_TXDONE_EVENT:
                #if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(gp_socket_cmd_tx_sem);
				#elif(_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(gp_socket_cmd_tx_sem);
				#endif
				break;

			case GP_SOCK_RXDONE_EVENT:
				DBG_PRINT("GP_SOCK_RXDONE_EVENT\r\n");
				process_gp_socket_cmd(gp_sock_rx_buf);
				memset(gp_sock_rx_buf, 0, sizeof(gp_sock_rx_buf));
				gp_sock_rx_buf_len = 0;
				break;

			case GP_SOCK_CLI_DISC_EVENT:
				/* Client disconnected from GP socket */
				DBG_PRINT("Client disconnected from GP socket\r\n");
				stop_decode_image();
				send_udpcli = 0;
				gp_socket_connect_to_upd_server = GP_SOCKET_UDP_DISCONNECT_TO_SERVER;
				break;

			case GP_SOCK_CLI_CONN_EVENT:
				/* Get remote GP sokcet client IP address */
				gspi_send_docmd(GP_CMD_GET_GP_SOCKET_REMOTE_IP);
				/* Client connected from GP socket */
				DBG_PRINT("Client connected to GP socket\r\n");
				break;

			case GP_SOCK_RAW_DATA_DONE_EVENT:
				/* Send GSPI_GET_GP_SOCKET_RAW_CMD to get 1k bytes data */
				if(gp_sock_rx_buf_len >= GP_SOCK_RAW_DATA_TRANSFER_SIZE)
				{
					gspi_tx_cmd(GSPI_GET_GP_SOCKET_RAW_CMD, GP_SOCK_RAW_DATA_TRANSFER_SIZE);
				}
				else
				{
					gspi_tx_cmd(GSPI_GET_GP_SOCKET_RAW_CMD, gp_sock_rx_buf_len);
				}
				break;

			case GP_SOCK_RAW_DATA_SEND_BACK_EVENT:
				if(gp_sock_rx_buf_len)
				{
					if(gp_sock_rx_buf_len >= GP_SOCK_RAW_DATA_TRANSFER_SIZE)
					{
						gspi_tx_cmd(GSPI_GET_GP_SOCKET_RAW_CMD, GP_SOCK_RAW_DATA_TRANSFER_SIZE);
					}
					else
					{
						gspi_tx_cmd(GSPI_GET_GP_SOCKET_RAW_CMD, gp_sock_rx_buf_len);
					}
				}
				break;

			case GP_SOCK_RAW_DATA_SEND_BACK_DONE_EVENT:
					gp_sock_rx_buf[10] = 0x03;
					gp_sock_rx_buf[11] = 0x02;
					//DBG_PRINT("send ack back to ameba\r\n");
					/* Send ACK back to GP socket */
					_send_handshake_back_to_gp_socket(GP_SOCK_ACK_TYPE);
				break;

			case GSPI_UC_RX_LEN_EVENT:
				/* UDP RX get buffer length */
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(gp_socket_udp_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(gp_socket_udp_rx_sem);
				#endif
				break;

			case GSPI_UC_RX_DATA_EVENT:
				DBG_PRINT("\r\nGPSOCK UDP RX[%d]:\r\n", gp_sock_udp_rx_len);
				DBG_PRINT("%s\r\n", gp_sock_udp_rx_buf);
				#if (_OPERATING_SYSTEM == _OS_UCOS2)
				OSSemPost(gp_socket_udp_rx_sem);
				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
				osSemaphoreRelease(gp_socket_udp_rx_sem);
				#endif
				break;

			case GSPI_UC_TX_DONE_EVENT:
				//DBG_PRINT("GPSOCK UDP TX packet done\r\n");
				//Test for UDP transfer
				//_db_send_udp_command(DOOR_BELL_RING_CMD);
				break;

			case GSPI_UC_SOCKET_ERROR_EVENT:
				break;

			case GP_SOCKET_GET_REMOTE_IP_EVENT:
#if 1
				memset((char*)gp_socket_client_ip, 0, sizeof(gp_socket_client_ip));
				sprintf(gp_socket_client_ip, "%d.%d.%d.%d", (gp_remote_ip.gp_sock_client_ip & 0xFF), (gp_remote_ip.gp_sock_client_ip & 0x0000FF00) >> 8,
																	 (gp_remote_ip.gp_sock_client_ip & 0x00FF0000) >> 16, (gp_remote_ip.gp_sock_client_ip & 0xFF000000) >> 24);
				DBG_PRINT("GP sokcet client IP address = %s\r\n", gp_socket_client_ip);
				gp_socket_connect_to_upd_server = GP_SOCKET_UDP_CONNECT_TO_SERVER;
#endif
				break;

			default:
				break;
		}
	}
}

void gp_socket_gspi_udpcli_cbk(INT8U* buf, INT32U len, INT32U event)
{
	static INT32U remain, cur_cnt;
	INT32U msg = event;
	INT32U copy_cnt;
	INT32U send_q = 1;
	INT8U err;

	if(event == GSPI_UC_RX_LEN_EVENT)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSSemPend(gp_socket_udp_rx_sem, 0, &err);
		#elif (_OPERATING == _OS_FREERTOS)
		osSemaphoreWait(gp_socket_udp_rx_sem, osWaitForever)
		#endif
		gp_sock_udp_rx_len = len;
		remain = gp_sock_udp_rx_len;
		cur_cnt = 0;
		memset((void*)gp_sock_udp_rx_buf, 0, GP_SOCKET_RX_BUF_SIZE);
	}
	else if(event == GSPI_UC_RX_DATA_EVENT)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSSemPend(gp_socket_udp_rx_sem, 0, &err);
		#elif (_OPERATING == _OS_FREERTOS)
		osSemaphoreWait(gp_socket_udp_rx_sem, osWaitForever)
		#endif
		if(remain)
		{
			copy_cnt = remain;
			if(copy_cnt > GP_SOCKET_RX_BUF_SIZE)
			{
				copy_cnt = GP_SOCKET_RX_BUF_SIZE;
			}
			memcpy((char*)(gp_sock_udp_rx_buf + cur_cnt), buf, copy_cnt);

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

	if(send_q)
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPost(gp_sock_q, (void*)msg);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		osMessagePut (gp_sock_q, (uint32_t)&msg, osWaitForever);
		#endif
}

static void gp_socket_remote_ip_info_cbk(void* data)
{
	INT32U msg;

	msg = GP_SOCKET_GET_REMOTE_IP_EVENT;
    #if (_OPERATING_SYSTEM == _OS_UCOS2)
	OSQPost(gp_sock_q, (void*)msg);
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	osMessagePut (gp_sock_q, (uint32_t)&msg, osWaitForever);
	#endif
}

void gp_socket_cmd_task_init(void)
{
	INT8U err;

	if(gpsock_first_init)
		return;

	/* Task only init once */
	gpsock_first_init = 1;

	/* Register remote IP address of GP socket server call back */
	gp_cmd_regsiter_info_cbk((void*)gp_socket_remote_ip_info_cbk, GPCMD_APP_REMOTE_IP_INFO);

	/* Register call back function for GSPI RX data */
	gspi_register_app_cbk((INT32U)gp_socket_cmd_gspi_cbk, GSPI_GP_SOCK_CMD_APP);

	/* Register call back function for GSPI UDP client */
	gspi_register_app_cbk((INT32U)gp_socket_gspi_udpcli_cbk, GSPI_UDP_CLIENT_APP);

	if(gp_sock_q == NULL)
	{
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		gp_sock_q = OSQCreate(gp_sock_q_stack, GP_SOCKET_Q_NUM);
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		gp_sock_q = osMessageCreate(&gp_sock_q_d, NULL);
		#endif
		if(gp_sock_q == 0)
		{
			DBG_PRINT("Create gp_sock_q failed\r\n");
			return;
		}
	}
	#if(_OPREATING_SYSTEM == _OS_UCOS2)
	gp_socket_cmd_tx_sem = OSSemCreate(1);
	gp_socket_udp_rx_sem = OSSemCreate(1);
	err = OSTaskCreate(gp_sock_cmd_task, (void *)NULL, &GPSocketCmdTaskStack[GPSOCKCMD_STACK_SIZE - 1], GP_SOCKET_CMD_PRIORITY);
	if(err != OS_NO_ERR)
	{
		DBG_PRINT("Cannot create GSPI socket command task\n\r");
		return;
	}
	#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
	gp_socket_cmd_tx_sem = osSemaphoreCreate(&gp_socket_cmd_tx_sem_d, 1);
	gp_socket_udp_rx_sem = osSemaphoreCreate(&gp_socket_udp_rx_sem_d, 1);
	osThreadId id;
	osThreadDef_t gp_sock_cmd_task_ent = {"gp_sock_cmd_task_entry", (os_pthread) gp_sock_cmd_task, osPriorityNormal, 1, GPSOCKCMD_STACK_SIZE};
    id = osThreadCreate(&gp_sock_cmd_task_ent, (void*) NULL);
    if (id == 0)
    {
        DBG_PRINT ("Cannot create GSPI socket command task\n\r");
        return;
    }
    #endif
}

void gp_socket_server_loop(void)
{
	if(gp_socket_connect_to_upd_server == GP_SOCKET_UDP_CONNECT_TO_SERVER)
	{
		INT32U retry = 10, cnt;
		char enable_atcmd[64] = {0};

connect_to_udp_server:

		cnt = GP_SOCKET_UDP_SERVER_TIMEOUT;
		/* Enable TCP client to connect TCP server */
		sprintf(enable_atcmd, GP_CMD_ENABLE_UDP_CLIENT "%s:" GP_SOCKET_UDP_SERVER_PORT, gp_socket_client_ip);
		gspi_send_docmd(enable_atcmd);

		while(((gp_net_app_state.udpcli_state != UDP_CLIENT_CONNECT_READY_STATE) && cnt) && gp_net_app_state.udpcli_state != UDP_CLIENT_ERROR_STATE)
		{
			cnt--;
			/* Sleep 1 second */
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100);
			#endif
		}

		if(gp_net_app_state.udpcli_state == UDP_CLIENT_ERROR_STATE)
		{
			retry--;
			DBG_PRINT("GP socket server enable UDP client failed, retry %d\r\n", retry);
			if(retry)
			{
				cnt = GP_SOCKET_UDP_SERVER_TIMEOUT;
				gspi_send_docmd(GP_CMD_DISABLE_UDP_CLIENT);

				while((gp_net_app_state.udpcli_state == UDP_CLIENT_DISABLE_STATE) && cnt)
				{
					cnt--;
					/* Sleep 1 second */
                    #if (_OPERATING_SYSTEM == _OS_UCOS2)
                    OSTimeDly(100);
                    #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                    osDelay(100);
                    #endif
				}
				goto connect_to_udp_server;
			}
		}

		gp_socket_connect_to_upd_server = GP_SOCKET_UDP_NO_ACTION;
		send_udpcli = 1;
	}
	else if(gp_socket_connect_to_upd_server == GP_SOCKET_UDP_DISCONNECT_TO_SERVER)
	{
		INT32U cnt = GP_SOCKET_UDP_SERVER_TIMEOUT;
		gspi_send_docmd(GP_CMD_DISABLE_UDP_CLIENT);
		while((gp_net_app_state.udpcli_state != UDP_CLIENT_DISABLE_STATE) && cnt)
		{
			cnt--;
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSTimeDly(100);
			#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
			osDelay(100);
			#endif
		}

		if(gp_net_app_state.udpcli_state == UDP_CLIENT_DISABLE_STATE)
		{
			DBG_PRINT("Disable UDP client\r\n");
		}
		gp_socket_connect_to_upd_server = GP_SOCKET_UDP_NO_ACTION;
	}

	if(send_udpcli)
	{
		char testbuf[] = "Hello UDP server!";
		gspi_transfer_udpcli_socket((INT8U*)testbuf, sizeof(testbuf));
		//DBG_PRINT("Send data to UDP server\r\n");
	}

}
