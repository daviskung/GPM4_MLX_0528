/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2016 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park,                      *
 *  Hsinchu City 30077, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
/**
 * @file		ehci_usbh.c
 * @brief		USB enhanced host controller uvc driver demo.
 * @author		Jason Chang
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "gplib.h"
#include "errno.h"
#include "cmsis_os.h"
#include "drv_l1_usbh.h"
#include "drv_l2_usbh.h"
#include "drv_l2_usbh_msc_class.h"
#include "gplib_mm_gplus.h"
#include "usbh_uvc.h"

/**************************************************************************
 *                           C O N S T A N T S                            *
**************************************************************************/
#define STACKSIZE                       32768
#define	QVGA_MODE			      		0
#define VGA_MODE						1
#define FRAME_NUM						100

#define DISP_TYPE	QVGA_MODE

#if DISP_TYPE == QVGA_MODE
	//#define RES_WIDTH	320
	//#define RES_HIGH	240
	#define RES_WIDTH	160
	#define RES_HIGH	120
#elif DISP_TYPE == VGA_MODE
	#define RES_WIDTH	640
	#define RES_HIGH	480
#endif

/**************************************************************************
 *                              M A C R O S                               *
**************************************************************************/

#define free_buffer(x)	{\
		if(x)\
		{\
			gp_free(x);\
			x = NULL;\
		}\
	}\

#define USB_HOST_UVC_TASK_QUEUE_MAX     32
/**************************************************************************
 *                          D A T A    T Y P E S                          *
**************************************************************************/

/**************************************************************************
 *                 E X T E R N A L    R E F E R E N C E S                 *
**************************************************************************/

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
**************************************************************************/

/**************************************************************************
 *                         G L O B A L    D A T A                         *
**************************************************************************/
extern USBH_DEV_INFO *g_stDev;
extern INT8U *gUVC_setup_buf;
extern INT8U *gUVC_setup_buf_m;
extern INT8U uvc_last_fid;
extern UVC_DECBUF_INFO_T uvc_decbuf_info;
extern INT8U usbh_init;
xQueueHandle hUSBHostUVCTaskQ;
xQueueHandle hUSBHostIntrQ;
xQueueHandle hUSBHostUVCdecQ;
xQueueHandle hUVCDecBufQ;
xQueueHandle hUVCDispCtrlQ;
INT32U MaxPacket;
INT32U ehci_dev_connect_status, uvc_disp_halted, udev_ins_debcnt=0, udev_rmv_debcnt=0;
INT8U *uvc_deco_bufptr;
/**************************************************************************
 *             F U N C T I O N    I M P L E M E N T A T I O N S           *
**************************************************************************/
extern INT32U display_VGA_to_QVGA_task(void);
extern void wait_ehc_halt(void);

#ifdef EHCI_UVC_EN
//--------------------------------------------------------------------
//    ehci uvc demo
//--------------------------------------------------------------------
void usbh_uvc_mem_free(void)
{
    INT32U i;
    if(gUVC_setup_buf_m != NULL)
    {
		gp_free(gUVC_setup_buf_m);
		gUVC_setup_buf = NULL;
		gUVC_setup_buf_m = NULL;
    }
/*
    if(g_stDev[0].para != 0)
    {
        gp_free(g_stDev[0].para);
        g_stDev[0].para = NULL;
    }
*/
    if(g_buf_m != NULL)
    {
		gp_free(g_buf_m);
		g_buf = NULL;
		g_buf_m = NULL;
    }
#if 0
	if(gframelist_m != NULL)
	{
		gp_free(gframelist_m);
		gframelist = NULL;
		gframelist_m = NULL;
    }
#else
    for(i=0; i<HOST_UVC_ISO_Buff_CNT; i++)
    {
        if(gframelist_m[i] != NULL)
        {
            gp_free(gframelist_m[i]);
            gframelist[i] = NULL;
            gframelist_m[i] = NULL;
        }
    }
#endif

	if(giTD_Table_Addr_m != NULL)
	{
		gp_free(giTD_Table_Addr_m);
		giTD_Table_Addr = NULL;
		giTD_Table_Addr_m = NULL;
    }

    if(uvc_deco_bufptr != NULL)
    {
        gp_free(uvc_deco_bufptr);
        uvc_deco_bufptr = 0;
    }

}

void queueflush(void)
{
    xQueueReset(hUSBHostUVCTaskQ);
    xQueueReset(hUVCDecBufQ);
    xQueueReset(hUVCDispCtrlQ);
    xQueueReset(hUSBHostUVCdecQ);
    xQueueReset(hUSBHostIntrQ);
}

static void usb_host_uvc_entry(void const *parm)
{
    INT32U msg_id, ret, y;
    INT16U msg_idd;
    INT8U n;

	while (1)
	{
        if(hUSBHostUVCTaskQ && (xQueueReceive(hUSBHostUVCTaskQ, &msg_id, 0) == pdTRUE))
        {
            cache_invalid_range(&msg_id, sizeof(INT32U));
            switch (msg_id & 0x000FFF)
            {
                case MSG_USB_HOST_UVC_PLUG_IN:
                    drv_l1_usbh_init();
                    while(1)
                    {
                        if(drv_l1_usbh_detect())
                        {
                            if(hUSBHostUVCTaskQ)
                            {
                                msg_id = MSG_USB_HOST_UVC_INITIAL;
                                xQueueSend(hUSBHostUVCTaskQ, &msg_id, 0);
                            }
                            if(hUVCDispCtrlQ)
                            {
                                msg_idd = MSG_UVC_DISP_INIT;
                                xQueueSend(hUVCDispCtrlQ, &msg_idd, 0);
                            }
                            DBG_PRINT("USB HOST UVC Connect OK!\r\n");
                            break;
                        }
                        else
                        {
                            DBG_PRINT("USB HOST UVC Connect fail!\r\n");
                        }
                    }
				break;

                case MSG_USB_HOST_UVC_INITIAL:
                    if(drv_l2_usbh_enumeration()==FALSE)
                    {
                        drv_l2_usbh_release();
                        drv_l1_usbh_un_init();
                        usbh_uvc_mem_free();
                        //usb_host_phy_clk_off();
                        DBG_PRINT("USB HOST UVC initial fail a!\r\n");

                    }
                    else
                    {
                        ret = usbh_interface_set(&MaxPacket);
                        if(ret == FALSE)
                        {
                            //drv_l2_usbh_uninit_device_infor();
                            drv_l2_usbh_release();
                            {
                                drv_l1_usbh_un_init();
                                usbh_uvc_mem_free();
                                //usb_host_phy_clk_off();
                                DBG_PRINT("USB HOST UVC initial fail b!\r\n");
                            }
                        }
                        else
                        {
                            uvc_last_fid = -1;
                            uvc_decbuf_info.buflength = UVC_JPGDEC_BUF_SIZE;
                            uvc_decbuf_info.state = BUF_STATE_IDLE;
                            uvc_decbuf_info.jpegdecbuf_idx = 0;
                            for(n=0; n<UVC_JPGDEC_BUF_MQUEUE_MAX; n++)
                                uvc_decbuf_info.bufkey[n] = 0;
                            uvc_deco_bufptr = gp_malloc_align(UVC_JPGDEC_BUF_MQUEUE_MAX*UVC_JPGDEC_BUF_SIZE, 64);
                            if(uvc_deco_bufptr == NULL)
                            {
                                DBG_PRINT("uvc deco buf alloc fail\r\n");
                                return;
                            }
                            for(y=0; y<UVC_JPGDEC_BUF_MQUEUE_MAX; y++)
                            {
                                uvc_decbuf_info.jpegdecbufptr[y] = uvc_deco_bufptr+UVC_JPGDEC_BUF_SIZE*y;
                            }
                            if(hUSBHostUVCTaskQ)
                            {
                                msg_id = MSG_USB_HOST_UVC_STARTGETISODATA;
                                xQueueSend(hUSBHostUVCTaskQ, &msg_id, 0);
                            }
                            DBG_PRINT("uvc init ok \r\n");
                        }
                    }
                break;

                case MSG_USB_HOST_UVC_STARTGETISODATA:
                    DBG_PRINT("uvc start get data!\r\n");
                    //vTaskDelay(300);//(JC)~300ms
                    vTaskDelay(30);//(JC)~30ms
                    drv_l2_usbh_iso_in_trans();
                break;

                case MSG_USB_HOST_UVC_GETNEXTISODATA:
//                    DBG_PRINT("uvc get next data!\r\n");
                    drv_l2_usbh_get_next_iso_data(msg_id & 0x1F000);
				break;

                case MSG_USB_HOST_UVC_PLUG_OUT:
                    //DBG_PRINT("UVC PLUG OUT!\r\n");
                    msg_idd = MSG_UVC_DISP_HALT;
                    xQueueSend(hUVCDispCtrlQ, &msg_idd, 0);
				break;

				case MSG_USB_HOST_UVC_RLS:
                    DBG_PRINT("uvc plugout!\r\n");
                    drv_l1_usbh_un_init();
                    wait_ehc_halt();
                    drv_l2_usbh_release();
                    usbh_uvc_mem_free();
                    queueflush();
                    drv_l1_usbh_init();
                break;

                default:
                break;
            }
        }
    }
}

void usb_host_uvc_disp_task(void const *param)
{
    platform_entrance(0);
    display_VGA_to_QVGA_task();
}

INT32S ehci_uvc_demo(void)
{
    INT32U msg_id;

    osThreadDef(usb_host_uvc_entry, osPriorityNormal, 0, STACKSIZE);
    osThreadCreate(&os_thread_def_usb_host_uvc_entry, NULL);
    osThreadDef(usb_host_uvc_disp_task, osPriorityNormal, 0, STACKSIZE);
    osThreadCreate(&os_thread_def_usb_host_uvc_disp_task, NULL);

    hUSBHostUVCTaskQ = xQueueCreate(USB_HOST_UVC_TASK_QUEUE_MAX, sizeof(INT32U));
    hUVCDecBufQ = xQueueCreate(UVC_JPGDEC_BUF_MQUEUE_MAX, sizeof(INT8U));
    hUVCDispCtrlQ = xQueueCreate(3, sizeof(INT16U));
    hUSBHostUVCdecQ = xQueueCreate(UVC_JPGDEC_BUF_MQUEUE_MAX, sizeof(INT32U));
    hUSBHostIntrQ = xQueueCreate(3, sizeof(INT32U));
    ehci_dev_connect_status = 0;
    if(usbh_init == 0)
        drv_l1_usbh_init();
    while(1)
	{
        if(hUSBHostUVCdecQ && (xQueueReceive(hUSBHostUVCdecQ, &msg_id, 0) == pdTRUE))
        {
            cache_invalid_range(&msg_id, sizeof(INT32U));
            switch(msg_id & 0xFFF00000)
            {
                case MSG_BACK_GET_DATA_N_DECODE:
                    uvc_get_data_n_decode(msg_id & 0x0001F000);
                break;

                default:
                break;
            }
        }

        if(ehci_dev_connect_status == 0)
        {
            if(drv_l2_usbh_chk_port_connect_status()==DEV_CONNECT)
            {
                if(++udev_ins_debcnt>10000)
                {
                    DBG_PRINT("usb dev insert+\r\n");
                    udev_ins_debcnt = 0;
                    ehci_dev_connect_status = 1;
                    msg_id = MSG_USB_HOST_UVC_PLUG_IN;
                    xQueueSend(hUSBHostUVCTaskQ, &msg_id, 0);
                }
            }
            else
            {
                udev_ins_debcnt = 0;
            }
        }
        else //if(ehci_dev_connect_status)
        {
            if(drv_l2_usbh_chk_port_connect_status()==DEV_DISCONNECT)
            {
                if(++udev_rmv_debcnt>10000)
                {
                    DBG_PRINT("usb dev removed-\r\n");
                    udev_rmv_debcnt = 0;
                    ehci_dev_connect_status = 0;
                    msg_id = MSG_USB_HOST_UVC_PLUG_OUT;
                    xQueueSend(hUSBHostUVCTaskQ, &msg_id, 0);
                }
            }
            else
            {
                udev_rmv_debcnt = 0;
            }
        }
	}

	drv_l2_usbh_release();
	/* ----- Uninitial USB hardware ----- */
	drv_l1_usbh_un_init();

	return 0;
}
#endif
