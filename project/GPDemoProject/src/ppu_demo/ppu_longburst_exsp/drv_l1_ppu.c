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

#include "drv_l1_ppu.h"

#if (defined _DRV_L1_PPU) && (_DRV_L1_PPU == 1)
#define PPU_USE_OS                  _OS_NONE//_OS_NONE//_OPERATING_SYSTEM
#define PPU_NULL                    0
#define PPU_TV_BUFFER_WAIT          0x8000
#define PPU_TFT_BUFFER_WAIT         0x2000
#define PUU_IRQ_TV_BUFFER_WAIT      0
#define PPU_CLOCK_SOURCE            19
#define FRAME_BUFFER_CLOCK_SOURCE   22
#define DISPLAY_REG_CLOCK_SOURCE    27
#define PPU_CLOCK_ENABLE            1
#define PPU_CLOCK_DISABLE           0

#define PPUOSQFlush(x)\
{\
    while(1) {\
        osEvent result;\
        result = osMessageGet(x, 1);\
        if(result.status != osEventMessage) {\
            break;\
        }\
    }\
}

static INT16U ppu_control_flag = 0;
static INT16U ppu_status_flag = 0;

#if PPU_USE_OS == _OS_UCOS2
static OS_EVENT *sem_ppu_engine = PPU_NULL;
static OS_EVENT *sem_update_register_done = PPU_NULL;
static OS_EVENT *sem_ppu_frame_output_done = PPU_NULL;

static void *free_queue_buffer[C_PPU_DRV_MAX_FRAME_NUM];
static void *display_queue_buffer[C_PPU_DRV_MAX_FRAME_NUM];
static OS_EVENT *free_frame_buffer_queue = PPU_NULL;
static OS_EVENT *display_frame_buffer_queue = PPU_NULL;
static void *Hblank_task_q_stack[C_HBLANK_QUEUE_MAX];
static OS_EVENT *Hblank_task_q = PPU_NULL;
#elif PPU_USE_OS == _OS_FREERTOS
xQueueHandle sem_update_register_done = PPU_NULL;
xQueueHandle sem_ppu_frame_output_done = PPU_NULL;
xQueueHandle free_frame_buffer_queue = PPU_NULL;
xQueueHandle display_frame_buffer_queue = PPU_NULL;
xQueueHandle Hblank_task_q = PPU_NULL;
xSemaphoreHandle sem_ppu_engine = PPU_NULL;
#else
static INT32U free_frame_buf[C_PPU_DRV_MAX_FRAME_NUM] = {0};
static INT32U prcess_frame_buf[C_PPU_DRV_MAX_FRAME_NUM] = {0};
static INT32S free_frame_index = 0, prcess_frame_index = 0;
#endif

static INT32U Hblank_wait_state = 0;
static PPU_REGISTER_SETS *new_register_sets_ptr;
static INT32U dma_current_update_type = 0;
static INT32U ppu_current_output_frame = PPU_NULL;
static INT32U tft_static_display_frame = PPU_NULL;
static INT32U tft_current_display_frame = PPU_NULL;
static INT32U tv_static_display_frame = PPU_NULL;
static INT32U tv_current_display_frame = PPU_NULL;

static INT32U enable_deflicker = 0;
static INT32U ppu_dma_enable = 0,ppu_dma_state = 0;
static INT32U ppu_dma_done, ppu_frame_done;

static void (*ppu_done_notifier)(void);

void start_dma_transfer(void);
INT32S update_ppu_registers(void);
void ppu_dma_handler(void);
void ppu_vblank_handler(void);
void ppu_tft_vblank_handler(void);
void ppu_tv_vblank_handler(void);
INT32S ppu_device_protect(void);
void ppu_device_unprotect(INT32S mask);
void ppu_isr(void);
//static void ppu_isr_event(INT32U event);
void ppu_isr_event(INT32U event);

static INT32U (*pfunc_ppu_end)(INT32U frame_buffer);

static INT16S drv_l1_PPU_clock_set(INT16U enable)
{
    INT16S state,error = 0;

    state = drv_l1_clock_set_system_clk_en(DISPLAY_REG_CLOCK_SOURCE, enable);
    if(state)
       error = state;

    state = drv_l1_clock_set_system_clk_en(FRAME_BUFFER_CLOCK_SOURCE, enable);
    if(state)
       error = state;

    state = drv_l1_clock_set_system_clk_en(PPU_CLOCK_SOURCE, enable);
    if(state)
       error = state;

    return error;
}

static INT32U ppu_frame_end(INT32U buf)
{
    ppu_frame_done = 0;
    ppu_frame_buffer_display((INT32U *)buf);
    //R_TFT_FBI_ADDR = buf;

    return 0;
}

// register encode end function.
void ppu_frame_end_register_callback(INT32U (*func)(INT32U frame_buffer))
{
	if(func) {
		pfunc_ppu_end = func;
	}
}

static void drv_l1_ppu_lock(void)
{
#if PPU_USE_OS == _OS_FREERTOS
    if(sem_ppu_engine){
        osSemaphoreWait(sem_ppu_engine, osWaitForever);
    }
#endif
}

static void drv_l1_ppu_unlock(void)
{
#if PPU_USE_OS == _OS_FREERTOS
    if(sem_ppu_engine){
        osSemaphoreRelease(sem_ppu_engine);
    }
#endif
}

static INT32S ppu_dma_state_get(INT32U wait)
{
    INT32S i,state = 0;

    if(wait)
    {
        for(i=0;i<0x20000;i++)
        {
            if(ppu_dma_done == 0)
                break;
        }
        state = -1;
    }
    else
    {
        state = ppu_dma_done;
    }

    return state;
}

static INT32S ppu_frame_end_state_get(INT32U wait)
{
    INT32S i,state = 0;

    if(wait)
    {
        for(i=0;i<0x200000;i++)
        {
            if(ppu_frame_done == 0)
                break;
        }
        state = -1;
    }
    else
    {
        state = ppu_frame_done;
    }

    return state;
}

void ppu_unint(void)
{
    drv_l1_PPU_clock_set(PPU_CLOCK_DISABLE);
}

void ppu_init(void)
{
#if PPU_USE_OS == _OS_FREERTOS
    osMessageQDef_t ppu_q = { C_PPU_DRV_MAX_FRAME_NUM, sizeof(INT32U), 0 };
#endif
	INT32U i,disp_en = R_PPU_ENABLE;

    //drv_l1_PPU_clock_set(PPU_CLOCK_ENABLE);
    //NVIC_DisableIRQ(PPU_IRQn);
	//R_PPU_ENABLE = 0;
	R_TFT_FBI_ADDR = 0;
  	R_TV_FBI_ADDR = 0;
	//ppu_display_resolution_unlock();

	ppu_control_flag = 0;
	ppu_status_flag = 0;
#if PPU_USE_OS == _OS_UCOS2
	if (sem_ppu_engine) {
		OSSemSet(sem_ppu_engine, 1, (INT8U *) &i);
	} else {
		sem_ppu_engine = OSSemCreate(1);
	}

	if (free_frame_buffer_queue) {
		OSQDel(free_frame_buffer_queue, OS_DEL_ALWAYS, (INT8U *) &i);
	}
	free_frame_buffer_queue = OSQCreate(&free_queue_buffer[0], C_PPU_DRV_MAX_FRAME_NUM);
	if (display_frame_buffer_queue) {
		OSQDel(display_frame_buffer_queue, OS_DEL_ALWAYS, (INT8U *) &i);
	}
	display_frame_buffer_queue = OSQCreate(&display_queue_buffer[0], C_PPU_DRV_MAX_FRAME_NUM);

	if (sem_update_register_done) {
		OSSemSet(sem_update_register_done, 0, (INT8U *) &i);
	} else {
		sem_update_register_done = OSSemCreate(0);
	}
	if (sem_ppu_frame_output_done) {
		OSSemSet(sem_ppu_frame_output_done, 0, (INT8U *) &i);
	} else {
		sem_ppu_frame_output_done = OSSemCreate(0);
	}
	if (Hblank_task_q) {
		OSQDel(Hblank_task_q, OS_DEL_ALWAYS, (INT8U *) &i);
	}
	Hblank_task_q = OSQCreate(&Hblank_task_q_stack[0], C_HBLANK_QUEUE_MAX);
#elif PPU_USE_OS == _OS_FREERTOS
	if (sem_ppu_engine == PPU_NULL) {
        osSemaphoreDef_t ppu_engine = { 0 };

        sem_ppu_engine = osSemaphoreCreate(&ppu_engine, 1);
	}

    if(free_frame_buffer_queue == PPU_NULL){
        free_frame_buffer_queue = osMessageCreate(&ppu_q, PPU_NULL);
    }
    else
        PPUOSQFlush(free_frame_buffer_queue);

    if(display_frame_buffer_queue == PPU_NULL){
        display_frame_buffer_queue = osMessageCreate(&ppu_q, PPU_NULL);
    }
    else
        PPUOSQFlush(display_frame_buffer_queue);

    if(sem_update_register_done == PPU_NULL){
        sem_update_register_done = osMessageCreate(&ppu_q, PPU_NULL);
    }
    else
        PPUOSQFlush(sem_update_register_done);

    if(sem_ppu_frame_output_done == PPU_NULL){
        sem_ppu_frame_output_done = osMessageCreate(&ppu_q, PPU_NULL);
    }
    else
        PPUOSQFlush(sem_ppu_frame_output_done);

    if(Hblank_task_q == PPU_NULL){
        Hblank_task_q = osMessageCreate(&ppu_q, PPU_NULL);
    }
    else
        PPUOSQFlush(Hblank_task_q);
#else
    for(i=0;i<C_PPU_DRV_MAX_FRAME_NUM;i++)
    {
        free_frame_buf[i] = 0;
        prcess_frame_buf[i] = 0;
    }
    free_frame_index = -1;
    prcess_frame_index = 0;
    ppu_frame_end_register_callback(ppu_frame_end);
#endif
  	dma_current_update_type = 0;
  	ppu_current_output_frame = PPU_NULL;
  	tft_static_display_frame = PPU_NULL;
  	tft_current_display_frame = PPU_NULL;
  	tv_static_display_frame = PPU_NULL;
  	tv_current_display_frame = PPU_NULL;
    ppu_dma_done = 0;
    ppu_frame_done = 0;

  	enable_deflicker = 0;

  	ppu_done_notifier = PPU_NULL;

  	new_register_sets_ptr = PPU_NULL;

    //drv_l2_ppu_isr_callback_set(ppu_isr_event);

	// Register PPU ISR
	//R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;	// Clear all PPU pending interrupts
	R_PPU_IRQ_EN |= (C_PPU_INT_PEND_PPU_VBLANK | C_PPU_INT_PEND_DMA_COMPLETE | C_PPU_INT_PEND_TFT_VBLANK);
#if PPU_USE_OS == _OS_FREERTOS
    if((disp_en & C_PPU_CTRL_FRAME_MODE) == 0)
    {
        NVIC_SetPriority(PPU_IRQn, 5);
        NVIC_EnableIRQ(PPU_IRQn);
    }
#endif
}


void ppu_display_resolution_lock(void)
{
#if PPU_USE_OS == _OS_UCOS2
	INT8U err;

	if (sem_ppu_engine) {
		OSSemPend(sem_ppu_engine, 0, &err);
	}
	R_PPU_MISC = 0x2;		// Enable frame buffer lock function, so that TV and TFT can have different resolution
	if (sem_ppu_engine) {
		OSSemPost(sem_ppu_engine);
	}
#elif PPU_USE_OS == _OS_FREERTOS
    drv_l1_ppu_lock();
    R_PPU_MISC = 0x2;		// Enable frame buffer lock function, so that TV and TFT can have different resolution
    drv_l1_ppu_unlock();
#endif
}


void ppu_display_resolution_unlock(void)
{
#if PPU_USE_OS == _OS_UCOS2
    INT8U err;

	if (sem_ppu_engine) {
		OSSemPend(sem_ppu_engine, 0, &err);
	}
	R_PPU_MISC = 0x0;
	if (sem_ppu_engine) {
		OSSemPost(sem_ppu_engine);
	}
#elif PPU_USE_OS == _OS_FREERTOS
    drv_l1_ppu_lock();
	R_PPU_MISC = 0x0;
    drv_l1_ppu_unlock();
#endif
}

INT32S ppu_frame_buffer_add(INT32U *frame_buf)
{
#if PPU_USE_OS == _OS_FREERTOS
    INT32U event;
#endif

	if (!frame_buf) {
		return -1;
	}
#if PPU_USE_OS == _OS_UCOS2
  	OSQPost(free_frame_buffer_queue, (void *) frame_buf);
#elif PPU_USE_OS == _OS_FREERTOS
    //xQueueSend (free_frame_buffer_queue, (void *) &frame_buf, portMAX_DELAY);
    event = (INT32U)frame_buf;
    osMessagePut(free_frame_buffer_queue, (uint32_t)&event, osWaitForever);
#else
    if(free_frame_index > C_PPU_DRV_MAX_FRAME_NUM)
        return -1;
    free_frame_index++;
    free_frame_buf[free_frame_index] = (INT32U)frame_buf;
#endif

  	if (tft_static_display_frame == PPU_NULL && tft_current_display_frame == PPU_NULL) {
  		R_TFT_FBI_ADDR = (INT32U) frame_buf;
  		tft_static_display_frame = 1;
  		tft_current_display_frame = 1;
  	}

#if 0
  	if (tv_static_display_frame == PPU_NULL && tv_current_display_frame == PPU_NULL) {
  		R_TV_FBI_ADDR = (INT32U) frame_buf;
  		tv_static_display_frame = 1;
  		tv_current_display_frame = 1;
  	}
#endif

	return 0;
}

INT32S ppu_frame_buffer_get(void)
{
#if PPU_USE_OS == _OS_FREERTOS
    osEvent result;
#endif
	INT32S i,j,frame;
	INT8U error;

	frame = PPU_NULL;
	while (!frame) {
    #if PPU_USE_OS == _OS_UCOS2
        frame = (INT32U) OSQPend(free_frame_buffer_queue, 1, &error);
		if (frame && error==OS_NO_ERR) {
			break;
		}
		frame = (INT32U) OSQPend(display_frame_buffer_queue, 1, &error);
		if (frame && error==OS_NO_ERR) {
			break;
		}
    #elif PPU_USE_OS == _OS_FREERTOS
        //error = xQueueReceive(free_frame_buffer_queue, &frame, configTICK_RATE_HZ);
		//if (frame && error == pdPASS) {
			//break;
		//}
        result = osMessageGet(free_frame_buffer_queue, 5000);
        frame = result.value.v;
        if(result.status == osEventMessage) {
            break;
        }
        else {
            error = 1;
            frame = PPU_NULL;
            break;
        }
		#if 0
		error = xQueueReceive(display_frame_buffer_queue, &frame, configTICK_RATE_HZ);
		if (frame && error == pdPASS) {
			break;
		}
		#endif
        frame = PPU_NULL;
    #else
        for(i=0;i<C_PPU_DRV_MAX_FRAME_NUM;i++)
        {
            if(free_frame_buf[free_frame_index])
            {
                frame = free_frame_buf[free_frame_index];
                free_frame_buf[free_frame_index] = 0;
                free_frame_index--;
                if(free_frame_index < 0)
                    free_frame_index = -1;
                break;
            }
            free_frame_index--;
        }
    #endif
	}

	return frame;
}

INT32S ppu_frame_buffer_display(INT32U *frame_buf)
{
#if PPU_USE_OS == _OS_FREERTOS
    INT32U event;
#endif
	if (!frame_buf) {
		return -1;
	}
#if PPU_USE_OS == _OS_UCOS2
  	OSQPost(display_frame_buffer_queue, (void *) frame_buf);
#elif PPU_USE_OS == _OS_FREERTOS
    //xQueueSend (display_frame_buffer_queue, (void *) frame_buf, portMAX_DELAY);
    event = (INT32U)frame_buf;
    osMessagePut(display_frame_buffer_queue, (uint32_t)&event, osWaitForever);
#else
    if(prcess_frame_index > C_PPU_DRV_MAX_FRAME_NUM)
        return -1;

    prcess_frame_buf[prcess_frame_index] = (INT32U)frame_buf;
    prcess_frame_index++;
#endif

	return 0;
}

INT32S ppu_frame_buffer_display_get(void)
{
#if PPU_USE_OS == _OS_FREERTOS
    osEvent result;
#endif
	INT32S i,j,frame;
	INT8U error;

#if PPU_USE_OS == _OS_UCOS2
 	frame = 0;
#elif PPU_USE_OS == _OS_FREERTOS
    error = 0;
    for(i=0;i<C_PPU_DRV_MAX_FRAME_NUM;i++)
    {
        //error = xQueueReceive(display_frame_buffer_queue, &frame, configTICK_RATE_HZ);
        //if (frame && error == pdPASS) {
            //break;
        //}
        result = osMessageGet(display_frame_buffer_queue, 5);
        frame = result.value.v;
        if(result.status == osEventMessage) {
            break;
        }
        else {
            error = 1;
        }
        frame = 0;
    }
#else
    for(i=0;i<C_PPU_DRV_MAX_FRAME_NUM;i++)
    {
        if(prcess_frame_buf[i])
        {
            frame = prcess_frame_buf[i];
            for(j=0;j<(prcess_frame_index-1);j++)
                prcess_frame_buf[j] = prcess_frame_buf[(j+1)];
            prcess_frame_index--;
            if(prcess_frame_index < 0)
                prcess_frame_index = 0;
            break;
        }
        frame = 0;
    }
#endif
	return frame;
}


INT32S ppu_tft_static_frame_set(INT32U *frame_buf)
{
	tft_static_display_frame = (INT32U) frame_buf;

	return 0;
}

INT32S ppu_tv_static_frame_set(INT32U *frame_buf)
{
	tv_static_display_frame = (INT32U) frame_buf;

	return 0;
}

INT32S ppu_deflicker_mode_set(INT32U value)
{
	enable_deflicker = value;

	return 0;
}

void start_dma_transfer(void)
{
	INT32U flag,temp,org_value;

	if (!new_register_sets_ptr) {
		return;
	}
	flag = new_register_sets_ptr->update_register_flag;
    ppu_dma_done = 1;

	while (flag & C_UPDATE_REG_SET_DMA_MASK) {
		if (flag & C_UPDATE_REG_SET_SPRITE_ATTRIBUTE) {
			dma_current_update_type = C_UPDATE_REG_SET_SPRITE_ATTRIBUTE;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_SPRITE_ATTRIBUTE_BASE & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_sprite_attribute_ptr;
			if (R_PPU_SPRITE_DMA_TARGET != R_PPU_SPRITE_DMA_SOURCE) {
				if ((new_register_sets_ptr->sprite_control & 0xFF00) == 0) {		// 1024 sprites are used
					R_PPU_SPRITE_DMA_NUMBER = 0xFFF;
				} else if (new_register_sets_ptr->sprite_control & C_PPU_SPRITE_COLOR_DITHER_MODE) {
					// If color dithering mode is used, only 512 sprites are supported and sprite size is doubled
					R_PPU_SPRITE_DMA_NUMBER = ((new_register_sets_ptr->sprite_control & 0x7F00)>>3) - 1;		// dma number = (sprite number * 4) * 32 / 4 - 1
				} else {
					R_PPU_SPRITE_DMA_NUMBER = ((new_register_sets_ptr->sprite_control & 0xFF00)>>4) - 1;		// dma number = (sprite number * 4) * 16 / 4 - 1
				}
				break;
			}
		}
    #if ((MCU_VERSION >= GPL326XX))
		if (flag & C_UPDATE_REG_SET_SPRITE_EX_ATTRIBUTE) {
			// 0xD0506000
			dma_current_update_type = C_UPDATE_REG_SET_SPRITE_EX_ATTRIBUTE;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_SPRITE_EXTERN_ATTRIBUTE_BASE & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_sprite_ex_attribute_ptr;
			if (R_PPU_SPRITE_DMA_TARGET != R_PPU_SPRITE_DMA_SOURCE) {
				if ((new_register_sets_ptr->sprite_control & 0xFF00) == 0)
					#if ((MCU_VERSION >= GPL326XX))
					  R_PPU_SPRITE_DMA_NUMBER = 0x1FF;            // 512 sprites are used
					#else
					  R_PPU_SPRITE_DMA_NUMBER = 0x3FF;            // 1024 sprites are used
					#endif
			    else
			        R_PPU_SPRITE_DMA_NUMBER = ((new_register_sets_ptr->sprite_control & 0xFF00)>>6) - 1;		// dma number = (sprite number * 4) * 16 / 4 - 1
			}
			break;
		}
    #endif
		if (flag & C_UPDATE_REG_SET_HORIZONTAL_MOVE) {		// 0xD0500400
			dma_current_update_type = C_UPDATE_REG_SET_HORIZONTAL_MOVE;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_TEXT_H_OFFSET0 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_horizontal_move_ptr;
			R_PPU_SPRITE_DMA_NUMBER = 0xF0 - 1;
			break;
		}
		if (flag & C_UPDATE_REG_SET_TEXT1_HCOMPRESS) {		// 0xD0500800
			dma_current_update_type = C_UPDATE_REG_SET_TEXT1_HCOMPRESS;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_TEXT_HCMP_VALUE0 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_text1_hcompress_ptr;
			R_PPU_SPRITE_DMA_NUMBER = 0xF0 - 1;
			break;
		}
		if (flag & C_UPDATE_REG_SET_TEXT3_25D) {		// 0xD0500800
			dma_current_update_type = C_UPDATE_REG_SET_TEXT3_25D;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_TEXT3_COS0 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_text3_25d_ptr;
			R_PPU_SPRITE_DMA_NUMBER = 0x1E0 - 1;
			break;
		}
		org_value = R_PPU_PALETTE_CTRL;
		temp = R_PPU_PALETTE_CTRL;
		temp &= ~0xC;
		R_PPU_PALETTE_CTRL = temp;
		if (flag & C_UPDATE_REG_SET_PALETTE0) {			// 0xD0501000
			dma_current_update_type = C_UPDATE_REG_SET_PALETTE0;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM0 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_palette0_ptr;
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE1) {			// 0xD0501400
			dma_current_update_type = C_UPDATE_REG_SET_PALETTE1;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM1 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_palette1_ptr;
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE2) {			// 0xD0501800
			dma_current_update_type = C_UPDATE_REG_SET_PALETTE2;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM2 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_palette2_ptr;
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE3) {			// 0xD0501C00
			dma_current_update_type = C_UPDATE_REG_SET_PALETTE3;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM3 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_palette3_ptr;
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		temp = R_PPU_PALETTE_CTRL;
		temp &= ~0xC;
		temp |= 0x4;
		R_PPU_PALETTE_CTRL = temp;
		// new palette group1
		if (flag & C_UPDATE_REG_SET_PALETTE4) {			// 0xD0501000
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE4;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM0 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[0];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE5) {			// 0xD0501400
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE5;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM1 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[1];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
            while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE6) {			// 0xD0501800
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE6;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM2 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[2];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE7) {			// 0xD0501C00
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE7;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM3 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[3];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}

		temp = R_PPU_PALETTE_CTRL;
		temp &= ~0xC;
		temp |= 0x8;
		R_PPU_PALETTE_CTRL = temp;
		// new palette group2
		if (flag & C_UPDATE_REG_SET_PALETTE8) {			// 0xD0501000
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE8;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM0 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[4];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE9) {			// 0xD0501400
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE9;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM1 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[5];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE10) {			// 0xD0501800
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE10;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM2 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[6];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE11) {			// 0xD0501C00
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE11;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM3 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[7];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
            while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
#if NEW_PAL_RAM_16_EN == 1
		temp = R_PPU_PALETTE_CTRL;
		temp |= 0xC;
		R_PPU_PALETTE_CTRL = temp;
		// new palette group2
		if (flag & C_UPDATE_REG_SET_PALETTE12) {			// 0xD0501000
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE12;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM0 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[8];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE13) {			// 0xD0501400
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE13;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM1 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[9];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE14) {			// 0xD0501800
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE14;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM2 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[10];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
		if (flag & C_UPDATE_REG_SET_PALETTE15) {			// 0xD0501C00
            dma_current_update_type = C_UPDATE_REG_SET_PALETTE15;
			R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM3 & 0x7FFF;
			R_PPU_SPRITE_DMA_SOURCE = new_register_sets_ptr->ppu_new_palette_ptr[11];
			R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
			while(R_PPU_SPRITE_DMA_NUMBER!=0);
			R_PPU_PALETTE_CTRL = org_value;
			break;
		}
#endif
	}
}

INT32S update_ppu_registers(void)
{
	INT32U flag;

	if (!new_register_sets_ptr) {
		return -1;
	}

	// Registers moved by PPU DMA engine
	start_dma_transfer();

	flag = new_register_sets_ptr->update_register_flag;

	// PPU relative registers
	if (flag & C_UPDATE_REG_SET_PPU) {
		R_PPU_BLENDING = new_register_sets_ptr->ppu_blending_level;
		R_PPU_FADE_CTRL = new_register_sets_ptr->ppu_fade_control;
		R_PPU_PALETTE_CTRL = new_register_sets_ptr->ppu_palette_control;
		R_PPU_BLD_COLOR = new_register_sets_ptr->ppu_rgb565_transparent_color;
		R_PPU_ENABLE = new_register_sets_ptr->ppu_enable;
		R_FREE_SIZE = new_register_sets_ptr->ppu_free_mode;
		R_PPU_MISC = new_register_sets_ptr->ppu_misc;
		R_PPU_FIFO_STEUP = new_register_sets_ptr->ppu_frame_buffer_fifo;
        R_PPU_UI_CTRL = new_register_sets_ptr->ppu_ui_enable;
        R_PPU_UI_ADDR = new_register_sets_ptr->ppu_ui_fbi_addr;
		R_PPU_WINDOW0_X = new_register_sets_ptr->ppu_window1_x;
		R_PPU_WINDOW0_Y = new_register_sets_ptr->ppu_window1_y;
		R_PPU_WINDOW1_X = new_register_sets_ptr->ppu_window2_x;
		R_PPU_WINDOW1_Y = new_register_sets_ptr->ppu_window2_y;
		R_PPU_WINDOW2_X = new_register_sets_ptr->ppu_window3_x;
		R_PPU_WINDOW2_Y = new_register_sets_ptr->ppu_window3_y;
		R_PPU_WINDOW3_X = new_register_sets_ptr->ppu_window4_x;
		R_PPU_WINDOW3_Y = new_register_sets_ptr->ppu_window4_y;
		R_PPU_RGB_OFFSET = new_register_sets_ptr->ppu_special_effect_rgb;
	}

	// TEXT relative registers
	if (flag & C_UPDATE_REG_SET_TEXT1) {
		R_PPU_TEXT1_X_POSITION = new_register_sets_ptr->text[C_PPU_TEXT1].position_x;
		R_PPU_TEXT1_Y_POSITION = new_register_sets_ptr->text[C_PPU_TEXT1].position_y;
		R_PPU_TEXT1_X_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT1].offset_x;
		R_PPU_TEXT1_Y_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT1].offset_y;
		R_PPU_TEXT1_ATTRIBUTE = new_register_sets_ptr->text[C_PPU_TEXT1].attribute;
		R_PPU_TEXT1_CTRL = new_register_sets_ptr->text[C_PPU_TEXT1].control;
		R_PPU_TEXT1_N_PTR = new_register_sets_ptr->text[C_PPU_TEXT1].n_ptr;
		R_PPU_TEXT1_A_PTR = new_register_sets_ptr->text[C_PPU_TEXT1].a_ptr;
		R_PPU_TEXT1_SINE = new_register_sets_ptr->text[C_PPU_TEXT1].sine;
		R_PPU_TEXT1_COSINE = new_register_sets_ptr->text[C_PPU_TEXT1].cosine;
		R_PPU_TEXT1_SEGMENT = new_register_sets_ptr->text[C_PPU_TEXT1].segment;

		R_PPU_VCOMP_VALUE = new_register_sets_ptr->ppu_vcompress_value;
		R_PPU_VCOMP_OFFSET = new_register_sets_ptr->ppu_vcompress_offset;
		R_PPU_VCOMP_STEP = new_register_sets_ptr->ppu_vcompress_step;
	}

	if (flag & C_UPDATE_REG_SET_TEXT2) {
		R_PPU_TEXT2_X_POSITION = new_register_sets_ptr->text[C_PPU_TEXT2].position_x;
		R_PPU_TEXT2_Y_POSITION = new_register_sets_ptr->text[C_PPU_TEXT2].position_y;
		R_PPU_TEXT2_X_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT2].offset_x;
		R_PPU_TEXT2_Y_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT2].offset_y;
		R_PPU_TEXT2_ATTRIBUTE = new_register_sets_ptr->text[C_PPU_TEXT2].attribute;
		R_PPU_TEXT2_CTRL = new_register_sets_ptr->text[C_PPU_TEXT2].control;
		R_PPU_TEXT2_N_PTR = new_register_sets_ptr->text[C_PPU_TEXT2].n_ptr;
		R_PPU_TEXT2_A_PTR = new_register_sets_ptr->text[C_PPU_TEXT2].a_ptr;
		R_PPU_TEXT2_SINE = new_register_sets_ptr->text[C_PPU_TEXT2].sine;
		R_PPU_TEXT2_COSINE = new_register_sets_ptr->text[C_PPU_TEXT2].cosine;
		R_PPU_TEXT2_SEGMENT = new_register_sets_ptr->text[C_PPU_TEXT2].segment;
	}

	if (flag & C_UPDATE_REG_SET_TEXT3) {
		R_PPU_TEXT3_X_POSITION = new_register_sets_ptr->text[C_PPU_TEXT3].position_x;
		R_PPU_TEXT3_Y_POSITION = new_register_sets_ptr->text[C_PPU_TEXT3].position_y;
		R_PPU_TEXT3_X_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT3].offset_x;
		R_PPU_TEXT3_Y_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT3].offset_y;
		R_PPU_TEXT3_ATTRIBUTE = new_register_sets_ptr->text[C_PPU_TEXT3].attribute;
		R_PPU_TEXT3_CTRL = new_register_sets_ptr->text[C_PPU_TEXT3].control;
		R_PPU_TEXT3_N_PTR = new_register_sets_ptr->text[C_PPU_TEXT3].n_ptr;
		R_PPU_TEXT3_A_PTR = new_register_sets_ptr->text[C_PPU_TEXT3].a_ptr;
		R_PPU_TEXT3_SIN0 = new_register_sets_ptr->text[C_PPU_TEXT3].sine;
		R_PPU_TEXT3_COS0 = new_register_sets_ptr->text[C_PPU_TEXT3].cosine;
		R_PPU_TEXT3_SEGMENT = new_register_sets_ptr->text[C_PPU_TEXT3].segment;

		R_PPU_Y25D_COMPRESS = new_register_sets_ptr->text3_25d_y_compress;
	}

	if (flag & C_UPDATE_REG_SET_TEXT4) {
		R_PPU_TEXT4_X_POSITION = new_register_sets_ptr->text[C_PPU_TEXT4].position_x;
		R_PPU_TEXT4_Y_POSITION = new_register_sets_ptr->text[C_PPU_TEXT4].position_y;
		R_PPU_TEXT4_X_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT4].offset_x;
		R_PPU_TEXT4_Y_OFFSET = new_register_sets_ptr->text[C_PPU_TEXT4].offset_y;
		R_PPU_TEXT4_ATTRIBUTE = new_register_sets_ptr->text[C_PPU_TEXT4].attribute;
		R_PPU_TEXT4_CTRL = new_register_sets_ptr->text[C_PPU_TEXT4].control;
		R_PPU_TEXT4_N_PTR = new_register_sets_ptr->text[C_PPU_TEXT4].n_ptr;
		R_PPU_TEXT4_A_PTR = new_register_sets_ptr->text[C_PPU_TEXT4].a_ptr;
		R_PPU_TEXT4_SINE = new_register_sets_ptr->text[C_PPU_TEXT4].sine;
		R_PPU_TEXT4_COSINE = new_register_sets_ptr->text[C_PPU_TEXT4].cosine;
		R_PPU_TEXT4_SEGMENT = new_register_sets_ptr->text[C_PPU_TEXT4].segment;
	}

	// Sprite relative registers
	if (flag & C_UPDATE_REG_SET_SPRITE) {
		R_PPU_SPRITE_CTRL = new_register_sets_ptr->sprite_control;
		R_PPU_SPRITE_SEGMENT = new_register_sets_ptr->sprite_segment;
		R_PPU_EXTENDSPRITE_CONTROL = new_register_sets_ptr->extend_sprite_control;
		R_PPU_EXTENDSPRITE_ADDR = new_register_sets_ptr->extend_sprite_addr;
	}

  #if 0//MCU_VERSION < GPL326XX
	// Check patch here
	{
		INT32U patch_id;

		patch_id = *((volatile INT32U *) 0xC0120008);
		if ((patch_id & 0x00000028) != 0x00000028) {
			if (patch_id && (R_PPU_ENABLE & 0x70000)!=0x20000) {
				R_PPU_ENABLE = (R_PPU_ENABLE & ~0x70000) | 0x20000;
			}
		} else if (!(patch_id & 0x00000010)) {
			if (patch_id && (R_PPU_ENABLE & 0x70000)) {
				R_PPU_ENABLE &= ~0x70000;
			}
		}
	}
  #endif

	return 0;
}

void ppu_dma_handler(void)
{
#if PPU_USE_OS == _OS_FREERTOS
	INT32S message;
#endif

    if (!new_register_sets_ptr || !dma_current_update_type) {
		return;
	}

	new_register_sets_ptr->update_register_flag &= ~dma_current_update_type;	// Clear DMA flag that is done
	dma_current_update_type = 0;
	if (!(new_register_sets_ptr->update_register_flag)) {
		// If PPU registers are all updated, notify task that we are done here.
    #if PPU_USE_OS == _OS_UCOS2
        OSSemPost(sem_update_register_done);
    #elif PPU_USE_OS == _OS_FREERTOS
        //message = SP_DMA_END;
        //xQueueSendFromISR( (xQueueHandle) sem_update_register_done, &message, 0);
        message = SP_DMA_END;
        osMessagePut(sem_update_register_done, (uint32_t)&message, 1);
    #else
        ppu_dma_done = 0;
    #endif
	} else if (new_register_sets_ptr->update_register_flag & C_UPDATE_REG_SET_DMA_MASK) {
		// If we still have DMA flag set, start next DMA transfer
		start_dma_transfer();
	}
	// Else, CPU updating registers is not done yet. Do nothing.
}

void ppu_vblank_handler(void)
{
    #if PPU_USE_OS == _OS_FREERTOS
        INT32S message;
    #endif
        if (R_PPU_ENABLE & C_PPU_CTRL_FRAME_MODE) {		// Frame base mode
		if ((R_PPU_ENABLE & C_PPU_CTRL_VGA_MODE) && !(R_PPU_ENABLE & C_PPU_CTRL_NON_INTERLACE_MODE)) {	// VGA Interlace mode
			if (R_PPU_FB_GO & C_PPU_FRAME_OUTPUT_FIELD) {		// If FBO_F bit is 1, it means only even line is done.
				R_PPU_FB_GO = 0x1;			// PPU will start calculating odd line after PPU_GO bit is set.

				return;
			}
            if(pfunc_ppu_end) {
                  if(ppu_current_output_frame)
                    pfunc_ppu_end(ppu_current_output_frame);
            }
			// Check whether de-flicker is needed
		  #if 0//(defined _DRV_L1_DEFLICKER) && (_DRV_L1_DEFLICKER == 1)
			if (enable_deflicker) {
				if (deflicker_start(1, 1, (INT32U *) ppu_current_output_frame, (INT32U *) ppu_current_output_frame, (OS_EVENT *) display_frame_buffer_queue)) {
					// If failed to call deflicker, display frame buffer directly
                    #if PPU_USE_OS == _OS_UCOS2
                    OSQPost(display_frame_buffer_queue, (void *) ppu_current_output_frame);
                    #elif PPU_USE_OS == _OS_FREERTOS
                    //xQueueSendFromISR( (xQueueHandle) display_frame_buffer_queue, &ppu_current_output_frame, 0);
                    message = ppu_current_output_frame;
                    osMessagePut(display_frame_buffer_queue, (uint32_t)&message, 0);
                    #endif
				}
			} else
		  #endif
			{
				// Post new frame to display_frame_buffer_queue
				#if PPU_USE_OS == _OS_UCOS2
                    OSQPost(display_frame_buffer_queue, (void *) ppu_current_output_frame);
                #elif PPU_USE_OS == _OS_FREERTOS
                //xQueueSendFromISR( (xQueueHandle) display_frame_buffer_queue, &ppu_current_output_frame, 0);
                    message = ppu_current_output_frame;
                    osMessagePut(display_frame_buffer_queue, (uint32_t)&message, 0);
                #endif
			}
		} else {
            if(pfunc_ppu_end) {
                  if(ppu_current_output_frame)
                    pfunc_ppu_end(ppu_current_output_frame);
            }
			// Post new frame to display_frame_buffer_queue
			#if PPU_USE_OS == _OS_UCOS2
            OSQPost(display_frame_buffer_queue, (void *) ppu_current_output_frame);
            #elif PPU_USE_OS == _OS_FREERTOS
            //xQueueSendFromISR( (xQueueHandle) display_frame_buffer_queue, &ppu_current_output_frame, 0);
            message = ppu_current_output_frame;
            osMessagePut(display_frame_buffer_queue, (uint32_t)&message, 1);
            #endif
		}

		ppu_current_output_frame = PPU_NULL;
		ppu_status_flag &= ~C_PPU_STATUS_FRAME_MODE_BUSY;
		// If current task is waiting for PPU operation to be done, notify it by setting this semaphore
		if (ppu_control_flag & C_PPU_CONTROL_WAIT_FRAME_DONE) {
			ppu_control_flag &= ~C_PPU_CONTROL_WAIT_FRAME_DONE;
			#if PPU_USE_OS == _OS_UCOS2
            OSSemPost(sem_ppu_frame_output_done);
            #elif PPU_USE_OS == _OS_FREERTOS
            //message = PPU_FRAME_END;
            //xQueueSendFromISR( (xQueueHandle) sem_ppu_frame_output_done, &message, 0);
            message = PPU_FRAME_END;
            osMessagePut(sem_ppu_frame_output_done, (uint32_t)&message, 1);
            #endif
		} else {
			// Post sem_ppu_engine semaphore to release PPU engine here
			#if PPU_USE_OS == _OS_UCOS2
            OSSemPost(sem_ppu_engine);
            #endif
		}
		if (ppu_done_notifier) {
			ppu_done_notifier();
		}
	} else {			// Line base mode
		if (ppu_control_flag & C_PPU_CONTROL_LINE_MODE_UPDATE) {
			if ((R_PPU_ENABLE & C_PPU_CTRL_VGA_MODE) && !(R_PPU_ENABLE & C_PPU_CTRL_NON_INTERLACE_MODE)) {	// VGA Interlace mode
				if (R_PPU_FB_GO & C_PPU_FRAME_OUTPUT_FIELD) {		// If FBO_F bit is 1, it means only even line is done.
					// We must wait until odd line is also done before updating registers.
					return;
				}
			}
			ppu_control_flag &= ~C_PPU_CONTROL_LINE_MODE_UPDATE;

			// Now update register sets
			if (update_ppu_registers()) {
				// Do nothing if something wrong
				return;
			}

			new_register_sets_ptr->update_register_flag &= C_UPDATE_REG_SET_DMA_MASK;	// Clear all flags except DMA
			if (!(new_register_sets_ptr->update_register_flag)) {
				// If sprite DMA copy is done or not needed, notify task that we are done here.
				#if PPU_USE_OS == _OS_UCOS2
                OSSemPost(sem_update_register_done);
                #elif PPU_USE_OS == _OS_FREERTOS
                //message = SP_DMA_END;
                //xQueueSendFromISR( (xQueueHandle) sem_update_register_done, &message, 0);
                message = PPU_FRAME_END;
                osMessagePut(sem_ppu_frame_output_done, (uint32_t)&message, 0);
                #endif
			}
			// else, leave the notification job to ppu_dma_handler
		}
        if(pfunc_ppu_end) {
              if(ppu_current_output_frame)
                pfunc_ppu_end(ppu_current_output_frame);
        }
	}
}

void ppu_tft_vblank_handler(void)
{
#if PPU_USE_OS == _OS_FREERTOS
    osEvent result;
#endif
  	INT32U frame;
  	INT8U err;

	if (tft_static_display_frame) {
		// If we want TFT to display user-defined frame only, return here
		R_TFT_FBI_ADDR = tft_static_display_frame;

		return;
	}

    #if PPU_USE_OS == _OS_UCOS2
  	frame = (INT32U) OSQAccept(display_frame_buffer_queue, &err);
	if (err==OS_NO_ERR && frame) {		// Check whether new frame buffer is available for display
		// Display new frame buffer
		R_TFT_FBI_ADDR = (INT32U) frame;

		// Post previous frame to free_frame_buffer_queue
		if (tft_current_display_frame) {
                    OSQPost(free_frame_buffer_queue, (void *) tft_current_display_frame);
		}

		// Update global variable
		tft_current_display_frame = frame;
	}
    #elif PPU_USE_OS == _OS_FREERTOS
	//err = (INT32S) xQueueReceiveFromISR(display_frame_buffer_queue, &frame, 0);
    result = osMessageGet(display_frame_buffer_queue, 1);
    frame = result.value.v;
    if(result.status == osEventMessage) {
            err = pdPASS;
    }
    else
            err = pdFAIL;
    if (err == pdPASS && frame) {		// Check whether new frame buffer is available for display
		// Display new frame buffer
		R_TFT_FBI_ADDR = (INT32U) frame;

		// Post previous frame to free_frame_buffer_queue
		if (tft_current_display_frame) {
            //xQueueSendFromISR( (xQueueHandle) free_frame_buffer_queue, &tft_current_display_frame, 0);
            frame = tft_current_display_frame;
            osMessagePut(free_frame_buffer_queue, (uint32_t)&frame, 1);
		}

		// Update global variable
		tft_current_display_frame = frame;
	}
    #endif
}

void ppu_tv_vblank_handler(void)
{
  	INT32U frame;
  	INT8U err;

	if (tv_static_display_frame) {
		// If we want TV to display user-defined frame only, return here
		R_TV_FBI_ADDR = tv_static_display_frame;

		return;
	}

    #if PUU_IRQ_TV_BUFFER_WAIT == 1
		if (!(R_PPU_FB_GO & C_PPU_FRAME_TV_UPDATED)) {
			// If previous frame has not been loaded into TV frame buffer register, just return
			return;
		}
	#endif

	#if PPU_USE_OS == _OS_UCOS2
  	frame = (INT32U) OSQAccept(display_frame_buffer_queue, &err);
	if (err==OS_NO_ERR && frame) {		// Check whether new frame buffer is available for display
		// Display new frame buffer
		R_TV_FBI_ADDR = (INT32U) frame;

		// Post previous frame to free_frame_buffer_queue
		if (tv_current_display_frame) {
                      OSQPost(free_frame_buffer_queue, (void *) tv_current_display_frame);
		}

		// Update global variable
		tv_current_display_frame = frame;
	}
        #elif PPU_USE_OS == _OS_FREERTOS
        err = (INT32S) xQueueReceiveFromISR(display_frame_buffer_queue, &frame, 0);
        if (err == pdPASS && frame) {		// Check whether new frame buffer is available for display
		// Display new frame buffer
		R_TV_FBI_ADDR = (INT32U) frame;

		// Post previous frame to free_frame_buffer_queue
		if (tv_current_display_frame) {
                    xQueueSendFromISR( (xQueueHandle) free_frame_buffer_queue, &tv_current_display_frame, 0);
		}

		// Update global variable
		tv_current_display_frame = frame;
	}
        #endif
}

INT32S ppu_device_protect(void)
{
#if PPU_USE_OS == _OS_UCOS2
        return vic_irq_disable(VIC_PPU);
#elif PPU_USE_OS == _OS_FREERTOS
        //NVIC_DisableIRQ(PPU_IRQn);

        return 0;
#endif
}

void ppu_device_unprotect(INT32S mask)
{
#if PPU_USE_OS == _OS_UCOS2
	if (mask == 0) {						// Clear device interrupt mask
		vic_irq_enable(VIC_PPU);
	} else if (mask == 1) {
		vic_irq_disable(VIC_PPU);
	} else {								// Something is wrong, do nothing
		return;
	}
#elif 0//PPU_USE_OS == _OS_FREERTOS
    if (mask == 0) {						// Clear device interrupt mask
		NVIC_EnableIRQ(PPU_IRQn);
	} else if (mask == 1) {
		NVIC_DisableIRQ(PPU_IRQn);
	} else {								// Something is wrong, do nothing
		return;
	}
#endif
}

//static void ppu_isr_event(INT32U event)
void ppu_isr_event(INT32U event)
{
#if PPU_USE_OS == _OS_FREERTOS
    INT32S state;
#endif
    if(event & C_PPU_INT_PEND_PPU_VBLANK)
    {
	    if(R_PPU_HB_CTRL & PPU_EN_HBLANK)
	    {
              #if PPU_USE_OS == _OS_UCOS2
                OSQPost(Hblank_task_q, (void *)HBLANK_FRAME_END);
              #elif PPU_USE_OS == _OS_FREERTOS
                //xQueueSendFromISR( (xQueueHandle) Hblank_task_q, (void *)HBLANK_FRAME_END, 0);
                state = HBLANK_FRAME_END;
                osMessagePut(Hblank_task_q, (uint32_t)&state, 1);
              #endif
        }
#if 1
	  	if(ppu_dma_enable == 0)
	  		ppu_vblank_handler();
	  	else
	  	    ppu_dma_state = 1;
#else
        ppu_vblank_handler();
#endif
    }

    if(event & C_PPU_INT_EN_DMA_COMPLETE)
    {
        ppu_dma_handler();
    }
}

void ppu_isr(void)
{
    // TBD: handle TFT+TV double display devices issue here

	// TFT Vertical-Blanking interrupt
	if ((R_PPU_IRQ_EN & C_PPU_INT_EN_TFT_VBLANK) && (R_PPU_IRQ_STATUS & C_PPU_INT_PEND_TFT_VBLANK)) {
		R_PPU_IRQ_STATUS = C_PPU_INT_PEND_TFT_VBLANK;
		ppu_tft_vblank_handler();
	}

	// TV Vertical-Blanking interrupt
	if ((R_PPU_IRQ_EN & C_PPU_INT_EN_TV_VBLANK) && (R_PPU_IRQ_STATUS & C_PPU_INT_PEND_TV_VBLANK)) {
		R_PPU_IRQ_STATUS = C_PPU_INT_PEND_TV_VBLANK;
		ppu_tv_vblank_handler();
	}

	// PPU Vertical-Blanking interrupt
	if ((R_PPU_IRQ_EN & C_PPU_INT_EN_PPU_VBLANK) && (R_PPU_IRQ_STATUS & C_PPU_INT_PEND_PPU_VBLANK)) {
		R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_VBLANK;
	    if(R_PPU_HB_CTRL & PPU_EN_HBLANK)
              #if PPU_USE_OS == _OS_UCOS2
	  	   OSQPost(Hblank_task_q, (void *)HBLANK_FRAME_END);
              #elif PPU_USE_OS == _OS_FREERTOS
                   xQueueSendFromISR( (xQueueHandle) Hblank_task_q, (void *)HBLANK_FRAME_END, 0);
              #endif
	  	ppu_vblank_handler();
	}

	// PPU Horizontal-Blanking interrupt
	if ((R_PPU_IRQ_EN & C_PPU_INT_EN_PPU_HBLANK) && (R_PPU_IRQ_STATUS & C_PPU_INT_PEND_PPU_HBLANK)) {
		R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_HBLANK;
		#if PPU_USE_OS == _OS_UCOS2
                OSQPost(Hblank_task_q, (void *)HBLANK_IEQ_END);
                #elif PPU_USE_OS == _OS_FREERTOS
                xQueueSendFromISR( (xQueueHandle) Hblank_task_q, (void *)HBLANK_IEQ_END, 0);
                #endif
	}
	// PPU DMA transfer complete interrupt
	if ((R_PPU_IRQ_EN & C_PPU_INT_EN_DMA_COMPLETE) && (R_PPU_IRQ_STATUS & C_PPU_INT_PEND_DMA_COMPLETE)) {
		R_PPU_IRQ_STATUS = C_PPU_INT_PEND_DMA_COMPLETE;
	  	ppu_dma_handler();
	}

	// sensor frame end interrupt
	if ((R_PPU_IRQ_EN & C_PPU_INT_EN_SENSOR_FRAME_END) && (R_PPU_IRQ_STATUS & C_PPU_INT_EN_SENSOR_FRAME_END)) {
		//extern avi_encode_switch_csi_frame_buffer(void);
		R_PPU_IRQ_STATUS |= C_PPU_INT_EN_SENSOR_FRAME_END;
	  	//avi_encode_switch_csi_frame_buffer();
	}
}

INT32U debug_en = 1;
static INT32S drv_l1_ppu_go(PPU_REGISTER_SETS *p_register_set, INT32U wait_start, INT32U wait_done)
{
#define PPU_STATE_MODE          1
#if PPU_USE_OS == _OS_FREERTOS
    osEvent result;
#endif
    INT32S mask;
	INT32U frame;
	INT8U err;

	if (!p_register_set) {
		return -1;
	}
#if 1
    frame = ppu_frame_buffer_get();
    if (!frame)
        return -1;

	// Update local pointer only when we have control of PPU engine
	new_register_sets_ptr = p_register_set;

    ppu_status_flag |= C_PPU_STATUS_FRAME_MODE_BUSY;

	ppu_current_output_frame = frame;
	R_PPU_FBO_ADDR = frame;

	if(update_ppu_registers())
		return -1;
	ppu_dma_state_get(1);

    // Start PPU engine
    R_PPU_FB_GO = 0x1;

    ppu_frame_end_state_get(1);
#else
  	// Check whether TFT and TV are both disabled
  	if (!(R_TFT_CTRL&0x1) && !(R_TV_CTRL&0x1) && !debug_en) {	// No TFT/TV interrupt source can be used, just udpate PPU registers and return
  		// Obtain sem_ppu_engine semaphore
		#if PPU_USE_OS == _OS_UCOS2
        if (wait_start) {
			OSSemPend(sem_ppu_engine, 0, &err);
			if (err != OS_NO_ERR) {
				return -1;
			}
		} else {
			if (!OSSemAccept(sem_ppu_engine)) {
				return -1;
			}
		}
        #endif
		// Update local pointer only when we have control of PPU engine
		new_register_sets_ptr = p_register_set;

  		R_PPU_IRQ_EN |= C_PPU_INT_EN_DMA_COMPLETE;

		// Set registers
		if (update_ppu_registers()) {
			// Reset new_register_sets_ptr before PPU engine semaphore is released
			new_register_sets_ptr = PPU_NULL;
			#if PPU_USE_OS == _OS_UCOS2
            OSSemPost(sem_ppu_engine);
            #endif

			return -1;
		}

		mask = ppu_device_protect();		// update_register_flag may be updated by DMA handler, so device protect is needed
		p_register_set->update_register_flag &= C_UPDATE_REG_SET_DMA_MASK;	// Clear all flags except DMA
		if (p_register_set->update_register_flag) {			// DMA transfer is not done yet
			ppu_device_unprotect(mask);
			// Now wait until DMA register update is done
			#if PPU_USE_OS == _OS_UCOS2
            OSSemPend(sem_update_register_done, 0, &err);
            #elif PPU_USE_OS == _OS_FREERTOS
            //xQueueSend (sem_update_register_done, (void *) SP_DMA_END, portMAX_DELAY);
            mask = SP_DMA_END;
            osMessagePut(sem_update_register_done, (uint32_t)&mask, osWaitForever);
            #endif
		} else {
			#if PPU_USE_OS == _OS_UCOS2
            while (OSSemAccept(sem_update_register_done)) ;		// Clear all pending semaphore
            #endif
			ppu_device_unprotect(mask);
		}

		// Reset new_register_sets_ptr before PPU engine semaphore is released
		new_register_sets_ptr = PPU_NULL;

		// TFT/TV vblank interrupt won't happen when TFT/TV is turned off. Handle queued frame by task.
		R_PPU_IRQ_EN &= ~(C_PPU_INT_EN_TFT_VBLANK);
		R_PPU_IRQ_STATUS |= C_PPU_INT_PEND_TFT_VBLANK;

		// Post previous frame to free_frame_buffer_queue
		if (tft_current_display_frame) {
			#if PPU_USE_OS == _OS_UCOS2
            OSQPost(free_frame_buffer_queue, (void *) tft_current_display_frame);
            #elif PPU_USE_OS == _OS_FREERTOS
            //xQueueSend (free_frame_buffer_queue, (void *) &tft_current_display_frame, portMAX_DELAY);
            mask = tft_current_display_frame;
            osMessagePut(free_frame_buffer_queue, (uint32_t)&mask, osWaitForever);
            #endif
			tft_current_display_frame = PPU_NULL;
		}
		if (tv_current_display_frame) {
			#if PPU_USE_OS == _OS_UCOS2
            OSQPost(free_frame_buffer_queue, (void *) tv_current_display_frame);
            #elif PPU_USE_OS == _OS_FREERTOS
            //xQueueSend (free_frame_buffer_queue, (void *) &tv_current_display_frame, portMAX_DELAY);
            mask = tv_current_display_frame;
            osMessagePut(free_frame_buffer_queue, (uint32_t)&mask, osWaitForever);
            #endif
			tv_current_display_frame = PPU_NULL;
		}

		while (1) {
			#if PPU_USE_OS == _OS_UCOS2
            frame = (INT32U) OSQAccept(display_frame_buffer_queue, &err);
			if (err==OS_NO_ERR && frame) {		// Check whether new frame buffer is available for display
				OSQPost(free_frame_buffer_queue, (void *) frame);
			} else {
				break;
			}
            #elif PPU_USE_OS == _OS_FREERTOS
            //err = (INT32S) xQueueReceive(display_frame_buffer_queue, &frame, 0);
            result = osMessageGet(display_frame_buffer_queue, 1);
            frame = result.value.v;
            if(result.status == osEventMessage) {
                err = pdPASS;
            }
            else
                err = pdFAIL;
			if (err == pdPASS && frame) {		// Check whether new frame buffer is available for display
                //xQueueSend (free_frame_buffer_queue, (void *) &frame, portMAX_DELAY);
                osMessagePut(free_frame_buffer_queue, (uint32_t)&frame, osWaitForever);
			} else {
				break;
			}
            #endif
		}

		// Disable interrupt if PPU is not enabled
		if (!(p_register_set->ppu_enable & C_PPU_CTRL_ENABLE)) {
			R_PPU_IRQ_EN &= ~C_PPU_INT_EN_PPU_MASK;			// Dislabe all PPU relative interrupts
			//R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;		// Clear all PPU pending interrupts
		} else {
			if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {
				R_PPU_IRQ_EN |= C_PPU_INT_PEND_TFT_VBLANK | C_PPU_INT_PEND_TV_VBLANK | C_PPU_INT_EN_DMA_COMPLETE | C_PPU_INT_EN_PPU_VBLANK;
			} else {
				R_PPU_IRQ_EN &= ~(C_PPU_INT_EN_TFT_VBLANK | C_PPU_INT_EN_TV_VBLANK);
				R_PPU_IRQ_EN |= C_PPU_INT_EN_DMA_COMPLETE | C_PPU_INT_EN_PPU_VBLANK;
			}
		}
        #if PPU_USE_OS == _OS_UCOS2
        OSSemPost(sem_ppu_engine);
        #endif

		return 0;
  	}

  	//if (R_PPU_ENABLE & C_PPU_CTRL_FRAME_MODE) {		// Current mode is frame base mode
	if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {		// Frame base mode
	  	if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {		// Frame base mode
			// Obtain a frame from free_frame_buffer_queue
			#if PPU_USE_OS == _OS_UCOS2
                if (wait_start) {
                    frame = (INT32U) OSQPend(free_frame_buffer_queue, 0, &err);
                } else {
                    frame = (INT32U) OSQAccept(free_frame_buffer_queue, &err);
                }
                if (err!=OS_NO_ERR || !frame) {
                    return -1;
                }
            #elif PPU_USE_OS == _OS_FREERTOS
                if (wait_start) {
                    //err = (INT32S) xQueueReceive(free_frame_buffer_queue, &frame, portMAX_DELAY);
                    result = osMessageGet(free_frame_buffer_queue, osWaitForever);
                } else {
                    //err = (INT32S) xQueueReceive(free_frame_buffer_queue, &frame, 0);
                    result = osMessageGet(free_frame_buffer_queue, 1);
                }
                frame = result.value.v;
                if(result.status == osEventMessage) {
                    err = pdPASS;
                }
                else
                    err = pdFAIL;
                if (err!=pdPASS || !frame) {
                    return -1;
                }
			#else
                frame = ppu_frame_buffer_get();
                if (!frame)
                    return -1;
            #endif
			// Obtain sem_ppu_engine semaphore
			if (wait_start) {
				#if PPU_USE_OS == _OS_UCOS2
                    OSSemPend(sem_ppu_engine, 0, &err);
                    if (err != OS_NO_ERR) {
                        // Post obtained frame back to free_frame_buffer_queue and then return
                        OSQPost(free_frame_buffer_queue, (void *) frame);

                        return -1;
                    }
                #elif PPU_USE_OS == _OS_FREERTOS

                #else

                #endif
			} else {
				#if PPU_USE_OS == _OS_UCOS2
                    if (!OSSemAccept(sem_ppu_engine)) {
                        // Post obtained frame back to free_frame_buffer_queue and then return
                        OSQPost(free_frame_buffer_queue, (void *) frame);

                        return -1;
                    }
                #elif PPU_USE_OS == _OS_FREERTOS

                #else

                #endif
			}

			// Update local pointer only when we have control of PPU engine
			new_register_sets_ptr = p_register_set;

			ppu_current_output_frame = frame;
			R_PPU_FBO_ADDR = frame;

			// Enable interrupt before updating registers
			if (!(R_PPU_ENABLE & C_PPU_CTRL_ENABLE)) {
				//R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;		// Clear all PPU pending interrupts
				if (!(p_register_set->ppu_enable & C_PPU_CTRL_ENABLE)) {
					R_PPU_IRQ_EN |= C_PPU_INT_EN_DMA_COMPLETE;
				} else {
					R_PPU_IRQ_EN |= C_PPU_INT_EN_DMA_COMPLETE | C_PPU_INT_EN_PPU_VBLANK;
				}
			}

			// Set registers
			if (update_ppu_registers()) {
				// Reset new_register_sets_ptr before PPU engine semaphore is released
				new_register_sets_ptr = PPU_NULL;
				ppu_current_output_frame = PPU_NULL;
				#if PPU_USE_OS == _OS_UCOS2
                    OSQPost(free_frame_buffer_queue, (void *) frame);
                    OSSemPost(sem_ppu_engine);
                #elif PPU_USE_OS == _OS_FREERTOS
                    //xQueueSend (free_frame_buffer_queue, (void *) &frame, portMAX_DELAY);
                    osMessagePut(free_frame_buffer_queue, (uint32_t)&frame, osWaitForever);
                #else
                    ppu_frame_buffer_add((INT32U *)frame);
                #endif

				return -1;
			}
#if 1
            p_register_set->update_register_flag &= C_UPDATE_REG_SET_DMA_MASK;	// Clear all flags except DMA
			if (p_register_set->update_register_flag) {			// DMA transfer is not done yet
            	// Registers moved by PPU DMA engine
                start_dma_transfer();
                #if PPU_USE_OS == _OS_UCOS2
                    OSSemPend(sem_update_register_done, 0, &err);
                #elif PPU_USE_OS == _OS_FREERTOS
                //err = (INT32S) xQueueReceive(sem_update_register_done, &frame, portMAX_DELAY*2);
                    result = osMessageGet(sem_update_register_done, 1000);
                    frame = result.value.v;
                #else
                    #if PPU_STATE_MODE == 1
                        while(1)
                        {
                            if(ppu_frame_end_state_get(0))
                            {
                                while((R_PPU_IRQ_STATUS & C_PPU_INT_EN_DMA_COMPLETE) == 0);
                                R_PPU_IRQ_STATUS = C_PPU_INT_EN_DMA_COMPLETE;
                                ppu_dma_handler();
                            }
                            else
                                break;
                        }
                    #else
                        ppu_dma_state_get(1);
                    #endif
                #endif
            }
#else
			mask = ppu_device_protect();		// update_register_flag may be updated by DMA handler, so device protect is needed
			p_register_set->update_register_flag &= C_UPDATE_REG_SET_DMA_MASK;	// Clear all flags except DMA
			if (p_register_set->update_register_flag) {			// DMA transfer is not done yet
				ppu_device_unprotect(mask);
				// Now wait until DMA register update is done
				#if PPU_USE_OS == _OS_UCOS2
                OSSemPend(sem_update_register_done, 0, &err);
                #elif PPU_USE_OS == _OS_FREERTOS
                //err = (INT32S) xQueueReceive(sem_update_register_done, &frame, portMAX_DELAY*2);
                result = osMessageGet(sem_update_register_done, 1000);
                frame = result.value.v;
                #endif
			} else {
				ppu_device_unprotect(mask);
				#if PPU_USE_OS == _OS_UCOS2
                while (OSSemAccept(sem_update_register_done)) ;		// Clear all pending semaphore
                #endif
			}
#endif
			// Reset new_register_sets_ptr before PPU engine semaphore is released
			new_register_sets_ptr = PPU_NULL;

			// If PPU is disabled after register updating, disable interrupt
			if (!(p_register_set->ppu_enable & C_PPU_CTRL_ENABLE)) {
				R_PPU_IRQ_EN &= ~C_PPU_INT_EN_PPU_MASK;			// Dislabe all PPU relative interrupts
				//R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;		// Clear all PPU pending interrupts
				R_PPU_IRQ_STATUS = (C_PPU_INT_PEND_DMA_COMPLETE | C_PPU_INT_PEND_PPU_VBLANK);		// Clear all PPU pending interrupts

				ppu_current_output_frame = PPU_NULL;
				#if PPU_USE_OS == _OS_UCOS2
                    // Post obtained frame back to free_frame_buffer_queue and then return
                    OSQPost(free_frame_buffer_queue, (void *) frame);
                    // Post sem_ppu_engine semaphore to release PPU engine
                    OSSemPost(sem_ppu_engine);
                #elif PPU_USE_OS == _OS_FREERTOS
                    //xQueueSend (free_frame_buffer_queue, (void *) &frame, portMAX_DELAY);
                    osMessagePut(free_frame_buffer_queue, (uint32_t)&frame, osWaitForever);
                #else
                    ppu_frame_buffer_add((INT32U *)frame);
                #endif
			} else {
				// If we want to wait until PPU frame output is done, set this flag
				if (wait_done) {
					ppu_control_flag |= C_PPU_CONTROL_WAIT_FRAME_DONE;
				} else {
					ppu_control_flag &= ~C_PPU_CONTROL_WAIT_FRAME_DONE;
				}
				ppu_status_flag |= C_PPU_STATUS_FRAME_MODE_BUSY;

				#if PPU_FRAME_REGISTER_WAIT == 1
				   while(1)
				   {
				      if(R_PPU_IRQ_EN & C_PPU_INT_EN_TV_VBLANK)
				      {
				         if(R_PPU_FB_GO & PPU_TV_BUFFER_WAIT)
				            break;
				      }
				      else
				      {

				         if(R_PPU_FB_GO & PPU_TFT_BUFFER_WAIT)
				            break;
				      }
                      #if PPU_USE_OS == _OS_UCOS2
				      OSTimeDly(1);
				      #elif PPU_USE_OS == _OS_FREERTOS
				      osDelay(1);
                      #endif
				   }
				#endif

                ppu_frame_done = 1;
				// Start PPU engine
				R_PPU_FB_GO = 0x1;

				// Wait until PPU operation is done
				if (wait_done) {
					#if PPU_USE_OS == _OS_UCOS2
                        OSSemPend(sem_ppu_frame_output_done, 0, &err);
                        OSSemPost(sem_ppu_engine);
                    #elif PPU_USE_OS == _OS_FREERTOS
                        //xQueueReceive(sem_ppu_frame_output_done, &err, portMAX_DELAY);
                        result = osMessageGet(sem_ppu_frame_output_done, osWaitForever);
                        frame = result.value.v;
                        if(result.status == osEventMessage) {
                            err = pdPASS;
                        }
                        else
                            err = pdFAIL;
                    #else
                        #if PPU_STATE_MODE == 1
                            while((R_PPU_IRQ_STATUS & C_PPU_INT_EN_PPU_VBLANK) == 0);
                            R_PPU_IRQ_STATUS = C_PPU_INT_EN_PPU_VBLANK;
                            ppu_frame_end(ppu_current_output_frame);
                        #else
                            ppu_frame_end_state_get(1);
                        #endif
                    #endif
				}
			}
		} else {		// Change from frame base mode to line base mode
			// Obtain sem_ppu_engine semaphore
			if (wait_start) {
				#if PPU_USE_OS == _OS_UCOS2
                    OSSemPend(sem_ppu_engine, 0, &err);
                    if (err != OS_NO_ERR) {
                        return -1;
                    }
				#elif PPU_USE_OS == _OS_FREERTOS

				#else

                #endif
			} else {
				#if PPU_USE_OS == _OS_UCOS2
                    if (!OSSemAccept(sem_ppu_engine)) {
                        return -1;
                    }
				#elif PPU_USE_OS == _OS_FREERTOS

				#else

                #endif
			}
			// Disable TFT/TV vblank interrupt
			//R_PPU_IRQ_EN &= ~(C_PPU_INT_EN_TFT_VBLANK | C_PPU_INT_EN_TV_VBLANK);
			//R_PPU_IRQ_STATUS |= C_PPU_INT_PEND_TFT_VBLANK | C_PPU_INT_PEND_TV_VBLANK;

			// Post previous frame to free_frame_buffer_queue
			if (tft_current_display_frame) {
				#if PPU_USE_OS == _OS_UCOS2
                    OSQPost(free_frame_buffer_queue, (void *) tft_current_display_frame);
                #elif PPU_USE_OS == _OS_FREERTOS
                    //xQueueSend (free_frame_buffer_queue, (void *) &tft_current_display_frame, portMAX_DELAY);
                    osMessagePut(free_frame_buffer_queue, (uint32_t)&tft_current_display_frame, osWaitForever);
                #else
                    ppu_frame_buffer_add((INT32U *)tft_current_display_frame);
                #endif
                tft_current_display_frame = PPU_NULL;
			}
			if (tv_current_display_frame) {
                #if PPU_USE_OS == _OS_UCOS2
                    OSQPost(free_frame_buffer_queue, (void *) tv_current_display_frame);
                #elif PPU_USE_OS == _OS_FREERTOS
                    //xQueueSend (free_frame_buffer_queue, (void *) &tv_current_display_frame, portMAX_DELAY);
                    osMessagePut(free_frame_buffer_queue, (uint32_t)&tv_current_display_frame, osWaitForever);
                #else
                    ppu_frame_buffer_add((INT32U *)tv_current_display_frame);
                #endif
                tv_current_display_frame = PPU_NULL;
			}
			// Free display frame buffer queue
			while (1) {
				#if PPU_USE_OS == _OS_UCOS2
                    frame = (INT32U) OSQAccept(display_frame_buffer_queue, &err);
                    if (err==OS_NO_ERR && frame) {
                        OSQPost(free_frame_buffer_queue, (void *) frame);
                    } else {
                        break;
                    }
                #elif PPU_USE_OS == _OS_FREERTOS
                    //err = (INT32S) xQueueReceive(display_frame_buffer_queue, &frame, 0);
                    result = osMessageGet(display_frame_buffer_queue, 1000);
                    frame = result.value.v;
                    if(result.status == osEventMessage) {
                        err = pdPASS;
                    }
                    else
                        err = pdFAIL;
                    if (err==pdPASS && frame) {
                        //xQueueSend (free_frame_buffer_queue, (void *) &frame, portMAX_DELAY);
                        osMessagePut(free_frame_buffer_queue, (uint32_t)&frame, osWaitForever);
                    } else {
                        break;
                    }
                #else

                #endif
			}

			// Update local pointer only when we have control of PPU engine
			new_register_sets_ptr = p_register_set;

			// Set register update flag
			ppu_control_flag |= C_PPU_CONTROL_LINE_MODE_UPDATE;

			// Update R_PPU_ENABLE to change to line mode. This will trigger PPU vblank interrupt
			R_PPU_ENABLE &= ~C_PPU_CTRL_FRAME_MODE;

			// Enable interrupt before updating registers
			if (!(R_PPU_ENABLE & C_PPU_CTRL_ENABLE)) {
				R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;		// Clear all PPU pending interrupts
				R_PPU_IRQ_EN |= C_PPU_INT_EN_DMA_COMPLETE | C_PPU_INT_EN_PPU_VBLANK;
			}

			// Wait until register update is done
            #if PPU_USE_OS == _OS_UCOS2
                OSSemPend(sem_update_register_done, 0, &err);
            #elif PPU_USE_OS == _OS_FREERTOS
                //err = (INT32S) xQueueReceive(sem_update_register_done, &frame, portMAX_DELAY);
                result = osMessageGet(sem_update_register_done, 1000);
                frame = result.value.v;
                if(result.status == osEventMessage) {
                    err = pdPASS;
                }
                else
                    err = pdFAIL;
            #else
                ppu_frame_end_state_get(1);
            #endif

			// If PPU is disabled after register updating, disable interrupt
			if (!(p_register_set->ppu_enable & C_PPU_CTRL_ENABLE)) {
				R_PPU_IRQ_EN &= ~C_PPU_INT_EN_PPU_MASK;			// Dislabe all PPU relative interrupts
				R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;		// Clear all PPU pending interrupts
			}

			// Reset new_register_sets_ptr before PPU engine semaphore is released
			new_register_sets_ptr = PPU_NULL;
            #if PPU_USE_OS == _OS_UCOS2
                // Post sem_ppu_engine semaphore to release PPU engine
                OSSemPost(sem_ppu_engine);
            #elif PPU_USE_OS == _OS_FREERTOS

            #else

            #endif
		}
	} else {			// Line mode
		if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {	// Change from line base mode to frame base mode
			// Obtain a frame from free_frame_buffer_queue
			#if PPU_USE_OS == _OS_UCOS2
                if (wait_start) {
                    frame = (INT32U) OSQPend(free_frame_buffer_queue, 0, &err);
                } else {
                    frame = (INT32U) OSQAccept(free_frame_buffer_queue, &err);
                }
                if (err!=OS_NO_ERR || !frame) {
                    return -1;
                }
            #elif PPU_USE_OS == _OS_FREERTOS
                if (wait_start) {
                    //err = (INT32S) xQueueReceive(free_frame_buffer_queue, &frame, portMAX_DELAY);
                    result = osMessageGet(free_frame_buffer_queue, osWaitForever);
                } else {
                    //err = (INT32S) xQueueReceive(free_frame_buffer_queue, &frame, 0);
                    result = osMessageGet(free_frame_buffer_queue, 1);
                }
                frame = result.value.v;
                if(result.status == osEventMessage) {
                    err = pdPASS;
                }
                else
                    err = pdFAIL;
                if (err!=pdPASS || !frame) {
                    return -1;
                }
            #else
                frame = ppu_frame_buffer_get();
            #endif
			ppu_current_output_frame = frame;
			R_PPU_FBO_ADDR = frame;
		}

		// Obtain sem_ppu_engine semaphore
		if (wait_start) {
			#if PPU_USE_OS == _OS_UCOS2
                OSSemPend(sem_ppu_engine, 0, &err);
                if (err != OS_NO_ERR) {
                    if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {
                        ppu_current_output_frame = PPU_NULL;
                        // Post obtained frame back to free_frame_buffer_queue and then return
                        OSQPost(free_frame_buffer_queue, (void *) frame);
                    }

                    return -1;
                }
            #elif PPU_USE_OS == _OS_FREERTOS

            #else

            #endif
		} else {
            #if PPU_USE_OS == _OS_UCOS2
                if (!OSSemAccept(sem_ppu_engine)) {
                    if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {
                        ppu_current_output_frame = PPU_NULL;
                        // Post obtained frame back to free_frame_buffer_queue and then return
                        OSQPost(free_frame_buffer_queue, (void *) frame);
                    }

                    return -1;
                }
            #elif PPU_USE_OS == _OS_FREERTOS

            #else

            #endif
		}

		// Update local pointer only when we have control of PPU engine
		new_register_sets_ptr = p_register_set;

		// Set register update flag
		ppu_control_flag |= C_PPU_CONTROL_LINE_MODE_UPDATE;

		// Enable interrupt to update registers
		if (!(R_PPU_ENABLE & C_PPU_CTRL_ENABLE)) {
			//R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;		// Clear all PPU pending interrupts
			R_PPU_IRQ_EN |= C_PPU_INT_EN_DMA_COMPLETE | C_PPU_INT_EN_PPU_VBLANK;
		}

        #if PPU_USE_OS == _OS_UCOS2
            // Wait until register update is done
            OSSemPend(sem_update_register_done, 0, &err);
        #elif PPU_USE_OS == _OS_FREERTOS
            //err = (INT32S) xQueueReceive(sem_update_register_done, &frame, portMAX_DELAY*2);
            result = osMessageGet(sem_update_register_done, 1000);
        #else
            ppu_frame_end_state_get(1);
        #endif

		// Reset new_register_sets_ptr before PPU engine semaphore is released
		new_register_sets_ptr = PPU_NULL;

		// If PPU is disabled after register updating, disable interrupt
		if (!(p_register_set->ppu_enable & C_PPU_CTRL_ENABLE)) {
			R_PPU_IRQ_EN &= ~C_PPU_INT_EN_PPU_MASK;			// Dislabe all PPU relative interrupts
			R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_MASK;		// Clear all PPU pending interrupts

			if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {	// Release frame buffer that will never be used
				ppu_current_output_frame = PPU_NULL;
				// Post obtained frame back to free_frame_buffer_queue and then return
				#if PPU_USE_OS == _OS_UCOS2
                    OSQPost(free_frame_buffer_queue, (void *) frame);
                #elif PPU_USE_OS == _OS_FREERTOS
                    //xQueueSend (free_frame_buffer_queue, (void *) &frame, portMAX_DELAY);
                    osMessagePut(free_frame_buffer_queue, (uint32_t)&frame, osWaitForever);
                #else
                    ppu_frame_buffer_add((INT32U *)frame);
                #endif
			}
            #if PPU_USE_OS == _OS_UCOS2
                // Post sem_ppu_engine semaphore to release PPU engine
                OSSemPost(sem_ppu_engine);
            #elif PPU_USE_OS == _OS_FREERTOS

            #else

            #endif
		} else if (p_register_set->ppu_enable & C_PPU_CTRL_FRAME_MODE) {	// Change from line base mode to frame base mode
			// If we want to wait until PPU frame output is done, set this flag
			if (wait_done) {
				ppu_control_flag |= C_PPU_CONTROL_WAIT_FRAME_DONE;
			} else {
				ppu_control_flag &= ~C_PPU_CONTROL_WAIT_FRAME_DONE;
			}
			ppu_status_flag |= C_PPU_STATUS_FRAME_MODE_BUSY;

			// Start PPU engine
			R_PPU_FB_GO = 0x1;

			//R_PPU_IRQ_STATUS |= C_PPU_INT_PEND_TFT_VBLANK;
			//R_PPU_IRQ_EN |= C_PPU_INT_EN_TFT_VBLANK;

			// Wait until PPU operation is done
			if (wait_done) {
                #if PPU_USE_OS == _OS_UCOS2
                    OSSemPend(sem_ppu_frame_output_done, 0, &err);
                    OSSemPost(sem_ppu_engine);
                #elif PPU_USE_OS == _OS_FREERTOS
                    //err = (INT32S) xQueueReceive(sem_ppu_frame_output_done, &frame, portMAX_DELAY);
                    result = osMessageGet(sem_ppu_frame_output_done, osWaitForever);
                #else
                    ppu_frame_end_state_get(1);
                #endif
			}
		} else {		// Line mode with PPU enabled
			#if PPU_USE_OS == _OS_UCOS2
                // Post sem_ppu_engine semaphore to release PPU engine
                OSSemPost(sem_ppu_engine);
			#elif PPU_USE_OS == _OS_FREERTOS

			#else

            #endif
		}
	}
#endif
	return 0;
}

INT32S ppu_go(PPU_REGISTER_SETS *p_register_set, INT32U wait_start, INT32U wait_done)
{
	INT32S temp;

    drv_l1_ppu_lock();
	temp = drv_l1_ppu_go(p_register_set,wait_start,wait_done);
	drv_l1_ppu_unlock();

	return temp;
}

INT8U ppu_frame_mode_busy_get(void)
{
	return (ppu_status_flag & C_PPU_STATUS_FRAME_MODE_BUSY);
}

INT32S ppu_notifier_register(void (*notifier)(void))
{
	if (!notifier) {
		return -1;
	}
	ppu_done_notifier = notifier;

	return 0;
}

INT32S ppu_setting_update(PPU_REGISTER_SETS *p_register_set)
{
     new_register_sets_ptr = p_register_set;
	 if(update_ppu_registers())
		return -1;

     return 0;
}

INT32S set_ppu_go(PPU_REGISTER_SETS *p_register_set)
{
#if PPU_USE_OS == _OS_FREERTOS
	osEvent result;
#endif
	INT32U frame;
	INT8U err;

    #if PPU_USE_OS == _OS_UCOS2
	frame = (INT32U) OSQPend(free_frame_buffer_queue, 100, &err);
    if(err!=OS_NO_ERR || !frame)
		return -1;
    #elif PPU_USE_OS == _OS_FREERTOS
    //err = (INT32S) xQueueReceive(free_frame_buffer_queue, &frame, portMAX_DELAY);
    result = osMessageGet(free_frame_buffer_queue, 1000);
    frame = result.value.v;
    if(result.status == osEventMessage) {
        err = pdPASS;
    }
    else
        err = pdFAIL;
    if(err!=pdPASS || !frame)
		return -1;
    #endif

	// Update local pointer only when we have control of PPU engine
	new_register_sets_ptr = p_register_set;

    ppu_status_flag |= C_PPU_STATUS_FRAME_MODE_BUSY;

	ppu_current_output_frame = frame;
	R_PPU_FBO_ADDR = frame;

	if(update_ppu_registers())
		return -1;

	if(R_PPU_HB_CTRL & PPU_EN_HBLANK)
	   Hblank_wait_state=1;

	#if PPU_FRAME_REGISTER_WAIT == 1
		while(1)
		{
		  if(R_PPU_IRQ_EN & C_PPU_INT_EN_TV_VBLANK)
		  {
		     if(R_PPU_FB_GO & PPU_TV_BUFFER_WAIT)
		        break;
		  }
		  else
		  {

		     if(R_PPU_FB_GO & PPU_TFT_BUFFER_WAIT)
		        break;
		  }
          #if PPU_USE_OS == _OS_UCOS2
		  OSTimeDly(1);
		  #elif PPU_USE_OS == _OS_FREERTOS
		  osDelay(1);
          #endif
		}
        #endif

	// Start PPU engine
        R_PPU_FB_GO = 0x1;

        return 0;
}

void ppu_notifier_unregister(void)
{
	ppu_done_notifier = PPU_NULL;
}

void set_ppu_free_control(INT8U INTL,INT16U H_size,INT16U V_size)
{
      R_FREE_SIZE = (((INTL & 0x1)<<31)|((H_size & 0x7ff)<<16)|(V_size & 0x7ff));

}

INT32S Hblenk_Enable_Set(INT32U value)
{

   if(!value)
   {
      R_PPU_IRQ_EN &=~C_PPU_INT_EN_PPU_HBLANK;
      R_PPU_HB_CTRL &=~PPU_EN_HBLANK;
   }
   else
   {
      R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_HBLANK;
      R_PPU_IRQ_EN |=C_PPU_INT_EN_PPU_HBLANK;
      R_PPU_HB_CTRL =PPU_EN_HBLANK;
      R_PPU_IRQ_STATUS = C_PPU_INT_PEND_PPU_HBLANK;
   }
   return 0;
}

void Hblenk_Line_Offset_Set(INT32U line_offset)
{

   R_PPU_HB_CTRL &=~PPU_HBLANK_OFFSET_MASK;
   R_PPU_HB_CTRL |=(line_offset & PPU_HBLANK_OFFSET_MASK);
}

void Hblenk_Go(void)
{
   R_PPU_HB_GO=0x1;
   Hblank_wait_state=1;
}

INT32S Hblenk_Wait(void)
{
	INT8U err;
	INT32U value;

    if(Hblank_wait_state)
    {
        #if PPU_USE_OS == _OS_UCOS2
        value = (INT32U) OSQPend(Hblank_task_q, 100, &err);
        if (err != OS_NO_ERR) {
                value=-1;              // Hblank irq time out.
        }
        #elif PPU_USE_OS == _OS_FREERTOS
        err = (INT32S) xQueueReceive(Hblank_task_q, &value, portMAX_DELAY);
        if (err != pdPASS) {
                value=-1;              // Hblank irq time out.
        }
        #endif
        Hblank_wait_state=0;
    }
    else
    {
        value=-2;                 // Hblank GO have not used.
    }

	return value;                 //HBLANK_IEQ_END=0x8001,HBLANK_FRAME_END=0x8002.

}

INT32S fd_lock_set(INT32U color1, INT32U size1, INT32U color2, INT32U size2)
{
        INT32U i, ppu_en,free_size;

        //ppu_display_resolution_lock();
        ppu_en = R_PPU_ENABLE;
        free_size = R_FREE_SIZE;
        R_PPU_MISC = 0x2;
        R_TV_CTRL = 0x1;
        R_PPU_ENABLE = color1;
        R_FREE_SIZE = size1;
        for(i=0;i<1000;i++);
        R_TFT_CTRL |= 0x1;
        R_PPU_ENABLE = color2;
        R_FREE_SIZE = size2;
        for(i=0;i<1000;i++);
        R_PPU_ENABLE = 0;
        R_FREE_SIZE = 0;
        R_TV_CTRL = 0x0;
        for(i=0;i<1000;i++);
        R_PPU_ENABLE = ppu_en;
        R_FREE_SIZE = free_size;
        //ppu_display_resolution_unlock();

        return 0;
}

/**
* @brief	       ppu module register set
* @return 	success=0,fail=-1.
*/
INT32S ppu_simple_go(PPU_REGISTER_SETS *p_register_set)
{
	if (!p_register_set) {
		return STATUS_FAIL;
	}

	ppu_dma_enable = 1;
	ppu_dma_state = 0;
	new_register_sets_ptr = p_register_set;

	R_PPU_IRQ_EN |= C_PPU_INT_EN_DMA_COMPLETE | C_PPU_INT_EN_PPU_VBLANK;

	R_PPU_FBO_ADDR = new_register_sets_ptr->buffer_user_define;

	// Set registers
	if (update_ppu_registers())
	{
		new_register_sets_ptr = NULL;
		return STATUS_FAIL;
	}

	// Start PPU engine
    R_PPU_FB_GO = 0x1;

	return 0;
}

INT32S ppu_simple_frame_end_get(INT32U wait)
{
	INT32U i,temp;

	if(wait)
	{
		for(i=0;i<100;i++)
		{
			if(ppu_dma_state)
			{
				ppu_dma_state = 0;
				temp = 1;
				break;
			}
			else
				osDelay(1);
		}
		temp = -1;
	}
	else
		temp = ppu_dma_state;

	return temp;
}
#endif 		// _DRV_L1_PPU
