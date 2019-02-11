/*
* Purpose: PPU driver/interface
*
* Author: Tristan Yang
*
* Date: 2008/03/03
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.02
*/
#include "drv_l1_clock.h"
#include "drv_l1_gpio.h"
#include "drv_l1_rotator.h"

#if (defined _DRV_L1_ROTATOR) && (_DRV_L1_ROTATOR == 1)
#define C_ROTATOR_QUEUE_SIZE		    5
#define GPIO_DEBUG_EN                   0
#define C_ROTATOR_END                   1
#define ROTATOR_STRUCTURE_EN            1
#if _OPERATING_SYSTEM == _OS_FREERTOS
#define SYSTEM_OS_ENABLE                1
#else
#define SYSTEM_OS_ENABLE                0
#endif
#define ROTATOR_CLOCK_SOURCE            16
#define ROTATOR_CLOCK_ENABLE            1
#define ROTATOR_CLOCK_DISABLE           0

typedef struct RotatorDev_s
{
	INT32U	CTRL;
	INT32U	IMG_WIDTH;
	INT32U	IMG_HEIGHT;
	INT32U	BUF_I_ADDR;
	INT32U	BUF_O_ADDR;
} RotatorDev_t;

#if ROTATOR_STRUCTURE_EN == 1
static RotatorDev_t rotator_structure;
#endif

#if SYSTEM_OS_ENABLE == 1
osMessageQId rotator_q = NULL;
osSemaphoreId rotator_hw_sem = NULL;
#else
static INT32U rotator_end;
#endif

#if GPIO_DEBUG_EN == 1
static INT32U rotator_end_counter = 0;
#endif

static INT32S drv_l1_rotator_memset(INT8S *dest,INT8U byte, INT32U Len)
{
	INT32S i = 0;

	for ( i = 0; i < Len; i++ ) {
		dest[i] = byte;
	}

	return(Len);
}

static INT16S drv_l1_rotator_clock_set(INT16U enable)
{
    return drv_l1_clock_set_system_clk_en(ROTATOR_CLOCK_SOURCE, enable);
}

static void drv_l1_rotator_lock(void)
{
#if SYSTEM_OS_ENABLE == 1
    if(rotator_hw_sem)
        osSemaphoreWait(rotator_hw_sem, osWaitForever);
#endif
}

static void drv_l1_rotator_unlock(void)
{
#if SYSTEM_OS_ENABLE == 1
	if(rotator_hw_sem)
        osSemaphoreRelease(rotator_hw_sem);
#endif
}

static INT32U drv_l1_rotator_structure_get(void)
{
#if ROTATOR_STRUCTURE_EN == 1
    INT32U ptr;

    drv_l1_rotator_lock();
    ptr = (INT32U)&rotator_structure;
    drv_l1_rotator_unlock();

    return ptr;
#else
    return 0;
#endif
}

static void rotator_isr(void)
{
    R_ROTATOR_IRQ_STATUS = C_RPTATOR_INT_END;
#if GPIO_DEBUG_EN == 1
    gpio_write_io(IO_D7, (rotator_end_counter++ & 0x1));
#endif

#if SYSTEM_OS_ENABLE == 1
    if(rotator_q) {
        INT32U event;

        event = C_ROTATOR_END;
        osMessagePut(rotator_q, (uint32_t)&event, osWaitForever);
    }
#else
    rotator_end = 0;
#endif
}

void ROTATOR_IRQHandler(void)
{
    rotator_isr();
}

void rotator_init(void)
{
#if ROTATOR_STRUCTURE_EN == 1
    RotatorDev_t *rot_structure = (RotatorDev_t *)drv_l1_rotator_structure_get();
#endif

    drv_l1_rotator_clock_set(ROTATOR_CLOCK_ENABLE);

    R_ROTATOR_CTRL = 0;
    R_ROTATOR_IRQ_STATUS = C_RPTATOR_INT_END;	// Clear Rotator pending interrupts
    R_ROTATOR_IRQ_EN = C_RPTATOR_INT_EN;

#if GPIO_DEBUG_EN == 1
    gpio_init_io (IO_D7, GPIO_OUTPUT);
    gpio_set_port_attribute(IO_D7, 1);
#endif

#if SYSTEM_OS_ENABLE == 1
    if(rotator_q == NULL) {
        osMessageQDef_t rot_q = { C_ROTATOR_QUEUE_SIZE, sizeof(INT32U), 0 };

        rotator_q = osMessageCreate(&rot_q, NULL);
    }

    if(rotator_hw_sem == NULL) {
        osSemaphoreDef_t rotator_sem = { 0 };

        rotator_hw_sem = osSemaphoreCreate(&rotator_sem, 1);
    }
#else
    rotator_end = 0;
#endif

#if ROTATOR_STRUCTURE_EN == 1
    drv_l1_rotator_memset((INT8S *)rot_structure,0 ,sizeof(RotatorDev_t));
#endif

    NVIC_SetPriority(ROTATOR_IRQn, 5);
    NVIC_EnableIRQ(ROTATOR_IRQn);
}

INT32S rotator_src_img_info(INT8U color, INT16U w, INT16U h, INT32U in_buf)
{
#if ROTATOR_STRUCTURE_EN == 1
    RotatorDev_t *rot_structure = (RotatorDev_t *)drv_l1_rotator_structure_get();
    INT32U temp = rot_structure->CTRL;
#else
    INT32U temp = R_ROTATOR_CTRL;
#endif


    if(!in_buf || !w || !h)
        return -1;

    if(w < 66 || h < 66)
    {
        if(w != 32 || h != 32 )
            return -1;
    }
    else
    {
        if((w % 2) != 0 || (h % 2) != 0)
            return -1;
    }

    drv_l1_rotator_lock();
    if(color == IMAGE_RGB565)
    {
         temp &= ~IMAGE_RGB565_ENABLE;
#if ROTATOR_STRUCTURE_EN == 1
         rot_structure->CTRL = temp;
#else
         R_ROTATOR_CTRL = temp;
#endif
    }
    else
    {
         temp |= IMAGE_YUV422_ENABLE;
         temp &= ~MASK_YUYV_ENABLE;
         temp |= (color << B_IMAGE_YUV422) & MASK_YUYV_ENABLE;
#if ROTATOR_STRUCTURE_EN == 1
         rot_structure->CTRL = temp;
#else
         R_ROTATOR_CTRL = temp;
#endif
    }
#if ROTATOR_STRUCTURE_EN == 1
    rot_structure->IMG_WIDTH = (w & MASK_IMG_RANGE);
    rot_structure->IMG_HEIGHT = (h & MASK_IMG_RANGE);
    rot_structure->BUF_I_ADDR = in_buf;
#else
    R_ROTATOR_IMG_WIDTH = (w & MASK_IMG_RANGE);
    R_ROTATOR_IMG_HEIGHT = (h & MASK_IMG_RANGE);
    R_ROTATOR_BUF_I_ADDR = in_buf;
#endif
    drv_l1_rotator_unlock();

    return 0;
}

INT32S rotator_tar_img_addr(INT32U out_buf)
{
#if ROTATOR_STRUCTURE_EN == 1
    RotatorDev_t *rot_structure = (RotatorDev_t *)drv_l1_rotator_structure_get();
#endif

    if(!out_buf)
        return -1;
    drv_l1_rotator_lock();
#if ROTATOR_STRUCTURE_EN == 1
    rot_structure->BUF_O_ADDR = out_buf;
#else
    R_ROTATOR_BUF_O_ADDR = out_buf;
#endif
    drv_l1_rotator_unlock();

    return 0;
}

INT32S rotator_mode_set(INT32U mode)
{
#if ROTATOR_STRUCTURE_EN == 1
    RotatorDev_t *rot_structure = (RotatorDev_t *)drv_l1_rotator_structure_get();
    INT32U temp = rot_structure->CTRL;
#else
    INT32U temp = R_ROTATOR_CTRL;
#endif
    drv_l1_rotator_lock();
    temp &= ~MASK_ROT_MODE;
    temp |= (mode << B_ROT_MODE) & MASK_ROT_MODE;
#if ROTATOR_STRUCTURE_EN == 1
    rot_structure->CTRL = temp;
#else
    R_ROTATOR_CTRL = temp;
#endif
    drv_l1_rotator_unlock();

    return 0;
}

INT32S rotator_start(void)
{
#if ROTATOR_STRUCTURE_EN == 1
    RotatorDev_t *rot_structure = (RotatorDev_t *)drv_l1_rotator_structure_get();
#endif

#if SYSTEM_OS_ENABLE == 0
    rotator_end = 1;
#endif
    drv_l1_rotator_lock();
#if ROTATOR_STRUCTURE_EN == 1
    R_ROTATOR_IMG_WIDTH = rot_structure->IMG_WIDTH;
    R_ROTATOR_IMG_HEIGHT = rot_structure->IMG_HEIGHT;
    R_ROTATOR_BUF_I_ADDR = rot_structure->BUF_I_ADDR;
    R_ROTATOR_BUF_O_ADDR = rot_structure->BUF_O_ADDR;
    R_ROTATOR_CTRL = rot_structure->CTRL;
#endif
    R_ROTATOR_START = 0x1;
    drv_l1_rotator_unlock();

    return 0;
}

INT32S rotator_end_wait(INT32U wait)
{
    INT32S state = -1;

    drv_l1_rotator_lock();
    if(wait)
    {
#if SYSTEM_OS_ENABLE == 1
        osEvent result;

        result = osMessageGet(rotator_q, 5000);
        state = result.value.v;
#else
        while(rotator_end);
#endif
    }
    drv_l1_rotator_unlock();

#if SYSTEM_OS_ENABLE == 0
    state = rotator_end;
#endif
    return state;
}

void rotator_uninit(void)
{
    R_ROTATOR_CTRL = 0;
    R_ROTATOR_IRQ_STATUS = C_RPTATOR_INT_END;	// Clear Rotator pending interrupts
    R_ROTATOR_IRQ_EN = 0;

    drv_l1_rotator_clock_set(ROTATOR_CLOCK_DISABLE);

    NVIC_DisableIRQ(ROTATOR_IRQn);
}

#endif 		// _DRV_L1_ROTATOR
