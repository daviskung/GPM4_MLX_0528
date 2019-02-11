
#include "project.h"
#include "drv_l1_gpio.h"//(JC)for usb dev detect
#include "drv_l2_usbd.h"

#define C_USBD_TASK_QHDL_MSDC   1
#define C_USBD_TASK_QHDL_UAC    2
#define C_USBD_TASK_QHDL_UVC    3

INT8U _usbd_detpin = 0, usbd_deb_flag = 0;
INT32U usbd_det_debcnt = 0;
INT8U connect_2_host = 0;
INT32U usbd_flag_period = 0;
extern USBD_USER_PARA_T usbd_userpara;
extern xQueueHandle hUSBD_MSDC_TaskQ;
extern xQueueHandle hUSBD_UAC_TaskQ;
extern xQueueHandle hUSBD_UVC_TaskQ;

ALIGN4 const INT8U USBD_USER_PARA[] =
{
	 IO_A12,		            //detect pin
	 0x05,		                //debounce count(unit 5ms)
	 C_USBD_TASK_QHDL_MSDC,     //USBD Q handle Type
	 100,                       //timer flag time delay(unit 5ms)
	 0                          //timer flag enable/disable initialization
};


void drv_l2_usbd_user_tmrisr(void)
{
    static INT8U usbd_detpin = 0;
    INT32U msg_idx;

    //detect host connecting
    //usbd_detpin = gpio_read_io(C_USBDEVICE_DETPIN);
    usbd_detpin = gpio_read_io(usbd_userpara.detpin);
    if(_usbd_detpin^usbd_detpin)
    {
        _usbd_detpin = usbd_detpin;
        usbd_det_debcnt = 0;
		usbd_deb_flag = 1;
    }
    else if(usbd_deb_flag)
    {
        if(++usbd_det_debcnt > usbd_userpara.debcnt)
        {
            _usbd_detpin = usbd_detpin;
			usbd_det_debcnt = 0;
			usbd_deb_flag = 0;
            if(usbd_detpin && connect_2_host == 0)
            {
                msg_idx = MSG_USB_DEVICE_INSERT;
            }
            else if(usbd_detpin == 0 && connect_2_host)
            {
                msg_idx = MSG_USB_DEVICE_REMOVE;
            }
            else
            {
                return;
            }
            cache_invalid_range(usbd_userpara.qhandletype, 1);
            if(usbd_userpara.qhandletype == 1)
            {
                if(hUSBD_MSDC_TaskQ)
                    xQueueSendFromISR(hUSBD_MSDC_TaskQ, &msg_idx, 0);
            }
            else if(usbd_userpara.qhandletype == 2)
            {
                if(hUSBD_UAC_TaskQ)
                    xQueueSendFromISR(hUSBD_UAC_TaskQ, &msg_idx, 0);
            }
            else
            {
                if(hUSBD_UVC_TaskQ)
                    xQueueSendFromISR(hUSBD_UVC_TaskQ, &msg_idx, 0);
            }
        }
    }
    if(++usbd_flag_period > usbd_userpara.tmrflag_period)
    {
        usbd_flag_period = 0;
        if(usbd_userpara.tmrflagOnOff)
        {
            msg_idx = MSG_USB_DEVICE_TMR_TIMEOUT;
            if(usbd_userpara.qhandletype == 1)
            {
                if(hUSBD_MSDC_TaskQ)
                {
                    xQueueSendFromISR(hUSBD_MSDC_TaskQ, &msg_idx, 0);
                }
            }
            else if(usbd_userpara.qhandletype == 2)
            {
                if(hUSBD_UAC_TaskQ)
                {
                    xQueueSendFromISR(hUSBD_UAC_TaskQ, &msg_idx, 0);
                }
            }
            else
            {
                if(hUSBD_UVC_TaskQ)
                {
                    xQueueSendFromISR(hUSBD_UVC_TaskQ, &msg_idx, 0);
                }
            }
        }
    }
}
