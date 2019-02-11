#ifndef __DRV_L2_SCCB_H__
#define __DRV_L2_SCCB_H__

#include "drv_l1.h"
#include "drv_l1_gpio.h"

#define _DRV_L2_SCCB    _DRV_L1_GPIO


typedef struct
{
    GPIO_ENUM       scl_port;
    IO_DRV_LEVEL    scl_drv;
    GPIO_ENUM       sda_port;
    IO_DRV_LEVEL    sda_drv;
    GPIO_ENUM       pwdn_port;
    IO_DRV_LEVEL    pwdn_drv;   // not yet implemented
    INT8U           have_pwdn;
    INT8U           RegBits;
    INT8U           DataBits;
    INT8U           slaveAddr;
    INT32U          timeout;    // in ms unit, not yet implemented
    INT32U          clock_rate; // in kHz unit, not yet implemented
}sccb_config_t;

extern void *drv_l2_sccb_open(INT8U ID, INT8U RegBits, INT8U DataBits);
extern void *drv_l2_sccb_open_ext(sccb_config_t *pSccbConfig);
extern void drv_l2_sccb_close(void *handle);
extern INT32S drv_l2_sccb_write(void *handle, INT16U addr, INT16U data);
extern INT32S drv_l2_sccb_read(void *handle, INT16U addr, INT16U *data);
extern INT32S drv_l2_sccb_continue_write(void *handle, INT16U addr, INT8U* data, INT32U len);
extern INT32S drv_l2_sccb_continue_read(void *handle, INT16U addr, INT8U *data, INT32U len);

#endif  //DRV_L2_SCCB_H
