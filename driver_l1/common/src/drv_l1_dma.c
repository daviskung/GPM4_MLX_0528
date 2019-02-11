/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2014 by Generalplus Inc.                         *
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
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#include "drv_l1_sfr.h"
#include "drv_l1_dma.h"
#include "drv_l1_cache.h"

#if (defined _DRV_L1_DMA) && (_DRV_L1_DMA == 1)
#include "drv_l1_gpio.h"
#include "drv_l1_timer.h"

#define K_DMA_TIMEOUT_PATCH_APPLY			1 // patch that fix timeout then next dma may have garbage bytes

#define K_SD_DMA_TEST_CODE					0 // must off when release code

extern void drv_usec_wait(INT32U u_sec);

#if (K_SD_DMA_TEST_CODE != 0)
static volatile int sd_dma_test = 0;

void dma_test_set(INT32U value)
{
    sd_dma_test = value;
}
#endif

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/


/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#define OSQFlush(x)\
{\
    while(1) {\
        osEvent result;\
        result = osMessageGet(x, 1);\
        if(result.status != osEventMessage) {\
            break;\
        }\
    }\
}

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
static INT32U dma_get_channel(INT8U usage);
///static INT32S dma_set_channel(INT8U usage, INT8U channel);
static void dma_free_channel(INT32U channel);

static INT32U dma_get_queue(void);
static void dma_free_queue(INT32U q_index);

void dma_set_notify(INT32U channel, INT32U notify, INT32U os_q);
static INT32S dma_device_protect(void);
static void dma_device_unprotect(INT32S mask);

static void dma_set_control(INT32U channel, INT32U ctrl);
static void dma_set_source(INT32U channel, INT32U addr);
static void dma_set_target(INT32U channel, INT32U addr);
static INT32S dma_set_tx_count(INT32U channel, INT32U count);
static INT32S dma_set_device(INT32U channel, INT16U device);
static INT32S dma_set_timeout(INT32U channel, INT32U timeout);
static INT32S dma_set_line_length(INT32U length);
static INT32S dma_set_sprite_size(INT32U channel, INT32U size);

#if 0
static INT32S dma_set_transparent_enable(INT32U channel);
static INT32S dma_set_transparent_disable(INT32U channel);
static INT32S dma_set_transparent_pattern(INT32U channel, INT16U pattern);
#endif

static INT32S dma_transfer_extend(DMA_STRUCT *dma_struct, INT8U usage, INT32U os_q, void (*dma_user_isr)(INT8U, INT8U));

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
static INT8U dma_init_done = FALSE;
static volatile INT8U dma_usage[C_DMA_CHANNEL_NUM];
static volatile INT32U dma_notify_variable[C_DMA_CHANNEL_NUM];
#if (K_DMA_TIMEOUT_PATCH_APPLY == 1)
static volatile INT8S dma_previous_done_state[C_DMA_CHANNEL_NUM] = {0};
#endif
static volatile INT32U dma_checksum[C_DMA_CHANNEL_NUM];

static volatile osMessageQId dma_notify_queue[C_DMA_CHANNEL_NUM];
static osMessageQId dma_driver_queue[C_DMA_Q_NUM] = {NULL};
static INT8U dma_q_usage[C_DMA_Q_NUM];
static INT8U dma_is_sd[C_DMA_CHANNEL_NUM];

/**
 * @brief   get dma idle channel
 * @param   usage: use method
 * @return 	channel
 */
static INT32U dma_get_channel(INT8U usage)
{
	INT32U i;

	for (i=0; i<C_DMA_CHANNEL_NUM; i++) {
        taskENTER_CRITICAL();
		if (dma_usage[i] == C_DMA_NOT_UESED) {
			dma_usage[i] = usage;

            taskEXIT_CRITICAL();

			// Reset DMA controller
			dma_set_control(i, C_DMA_CTRL_RESET);
			return i;
		}
		taskEXIT_CRITICAL();
	}

	return i;
}

/**
 * @brief   get dma assign channel
 * @param   usage: use method
 * @param   channel: assign channel
 * @return 	result: >=0 is success, <0 is fail.
 */
static INT32S dma_set_channel(INT8U usage, INT8U channel)
{
	INT32S ret;

	if(channel >= C_DMA_CHANNEL_NUM) {
		return STATUS_FAIL;
	}

    #if _OPERATING_SYSTEM != _OS_NONE				// Soft Protect for critical section
        #if _OPERATING_SYSTEM == _OS_UCOS2
        OSSchedLock();
        #elif _OPERATING_SYSTEM == _OS_FREERTOS
        taskENTER_CRITICAL();
        #endif
    #endif

	if (dma_usage[channel] == C_DMA_NOT_UESED) {
		dma_usage[channel] = usage;
		ret = STATUS_OK;
	} else {
		ret = STATUS_FAIL;
	}

    #if _OPERATING_SYSTEM != _OS_NONE
        #if _OPERATING_SYSTEM == _OS_UCOS2
        OSSchedUnlock();
        #elif _OPERATING_SYSTEM == _OS_FREERTOS
        taskEXIT_CRITICAL();
        #endif
    #endif

    if (ret == STATUS_OK)
        dma_set_control(channel, C_DMA_CTRL_RESET); 	// Reset DMA controller

	return ret;
}

static void (*dma_user_isr[C_DMA_CHANNEL_NUM])(INT8U, INT8U);

static INT32S drv_l1_dma_callback_set(INT32U channel, void (*user_isr)(INT8U, INT8U))
{
    INT32S ret = STATUS_OK;
    if(channel >7)
        ret = STATUS_FAIL;

    dma_user_isr[channel] = user_isr;

    return ret;
}

static INT32S drv_l1_dma_callback_clear(INT32U channel)
{
    INT32S ret = STATUS_OK;
    if(channel >7)
        ret = STATUS_FAIL;

    dma_user_isr[channel] = 0;

    return ret;
}

void drv_l1_dma_free_channel(INT32U channel)
{
	dma_free_channel(channel);
	dma_set_target(channel, 0);
	dma_set_source(channel, 0);
}

INT32U drv_l1_dma_get_channel_target_address(INT32U channel)
{
	INT32U addr = 0;

	// Clear pending bit
	if (channel == C_DMA_CH0) {
		addr = R_DMA0_TAR_ADDR;
	} else if (channel == C_DMA_CH1) {
		addr = R_DMA1_TAR_ADDR;
	} else if (channel == C_DMA_CH2) {
		addr = R_DMA2_TAR_ADDR;
	} else if (channel == C_DMA_CH3) {
		addr = R_DMA3_TAR_ADDR;
	} else if (channel == C_DMA_CH4) {
		addr = R_DMA4_TAR_ADDR;
	} else if (channel == C_DMA_CH5) {
		addr = R_DMA5_TAR_ADDR;
	} else if (channel == C_DMA_CH6) {
		addr = R_DMA6_TAR_ADDR;
	} else if (channel == C_DMA_CH7) {
		addr = R_DMA7_TAR_ADDR;
	}

	return addr;
}

static INT32U dma_get_timeout_flag(INT32U channel)
{
	INT32U flag = 0;

	// Clear pending bit
	if (channel == C_DMA_CH0) {
		flag = C_DMA0_TIMEOUT;
	} else if (channel == C_DMA_CH1) {
		flag = C_DMA1_TIMEOUT;
	} else if (channel == C_DMA_CH2) {
		flag = C_DMA2_TIMEOUT;
	} else if (channel == C_DMA_CH3) {
		flag = C_DMA3_TIMEOUT;
	} else if (channel == C_DMA_CH4) {
		flag = C_DMA4_TIMEOUT;
	} else if (channel == C_DMA_CH5) {
		flag = C_DMA5_TIMEOUT;
	} else if (channel == C_DMA_CH6) {
		flag = C_DMA6_TIMEOUT;
	} else if (channel == C_DMA_CH7) {
		flag = C_DMA7_TIMEOUT;
	}
	return flag;
}

static INT32U dma_get_int_flag(INT32U channel)
{
	INT32U flag = 0;

	// Clear pending bit
	if (channel == C_DMA_CH0) {
		flag = C_DMA0_INT_PEND;
	} else if (channel == C_DMA_CH1) {
		flag = C_DMA1_INT_PEND;
	} else if (channel == C_DMA_CH2) {
		flag = C_DMA2_INT_PEND;
	} else if (channel == C_DMA_CH3) {
		flag = C_DMA3_INT_PEND;
	} else if (channel == C_DMA_CH4) {
		flag = C_DMA4_INT_PEND;
	} else if (channel == C_DMA_CH5) {
		flag = C_DMA5_INT_PEND;
	} else if (channel == C_DMA_CH6) {
		flag = C_DMA6_INT_PEND;
	} else if (channel == C_DMA_CH7) {
		flag = C_DMA7_INT_PEND;
	}
	return flag;
}

static void dma_reset_channel(INT32U channel)
{
	if (channel >= C_DMA_CHANNEL_NUM) {
		return;
	}

	R_DMA_INT = dma_get_int_flag(channel) | dma_get_timeout_flag(channel);

	// Reset DMA controller
	dma_set_control(channel, C_DMA_CTRL_RESET);
}

/**
 * @brief   free dma unsued channel
 * @param   channel: channel
 * @return 	none
 */
static void dma_free_channel(INT32U channel)
{
	if (channel >= C_DMA_CHANNEL_NUM) {
		return;
	}

	dma_reset_channel(channel);

	dma_notify_variable[channel] = (INT32U) NULL;

  	dma_notify_queue[channel] = (osMessageQId) NULL;

	dma_usage[channel] = (INT8U) C_DMA_NOT_UESED;
}

/**
 * @brief   get and set dma channel queue
 * @param   none
 * @return 	queue number
 */
static INT32U dma_get_queue(void)
{
	INT32U i;

	for (i=0; i<C_DMA_Q_NUM; i++) {
        taskENTER_CRITICAL();
		if (!dma_q_usage[i]) {
			dma_q_usage[i] = (INT8U) TRUE;
            taskEXIT_CRITICAL();
            break;
		}
		taskEXIT_CRITICAL();
	}

	if (i < C_DMA_Q_NUM)
    {
        if (dma_driver_queue[i]) {
            OSQFlush(dma_driver_queue[i]);
        } else {
            osMessageQDef_t dma_q = {C_DMA_Q_BUF_SIZE, sizeof(INT32U), 0};

            dma_driver_queue[i] = osMessageCreate(&dma_q, NULL);
        }
    }

	return i;
}

/**
 * @brief   free dma channel queue
 * @param   q_index: queue number
 * @return 	none
 */
static void dma_free_queue(INT32U q_index)
{
	if (q_index >= C_DMA_Q_NUM) {
		return;
	}

	dma_q_usage[q_index] = (INT8U) FALSE;
}

/**
 * @brief   set dma done notify
 * @param   channel: channel number
 * @param   notify: notify point
 * @param   os_q: notify queue
 * @return 	none
 */
void dma_set_notify(INT32U channel, INT32U notify, INT32U os_q)
{
	dma_notify_variable[channel] = notify;
  	dma_notify_queue[channel] = (osMessageQId)os_q;
}

/**
 * @brief   disable dma irq
 * @param   none
 * @return 	result: >=0 is success, <0 is fail.
 */
static INT32S dma_device_protect(void)
{
    NVIC_DisableIRQ(DMA_IRQn);
	return 0;
}

/**
 * @brief   enable/disable dma irq
 * @param   mask: mask value
 * @return 	none
 */
static void dma_device_unprotect(INT32S mask)
{
    NVIC_EnableIRQ(DMA_IRQn);
}

INT32U dma_checksum_get(INT32S channel)
{
	INT32U checksum;

	if (channel == C_DMA_CH0) {
		checksum = R_DMA0_CHECKSUM;
	} else if (channel == C_DMA_CH1) {
		checksum = R_DMA1_CHECKSUM;
	} else if (channel == C_DMA_CH2) {
		checksum = R_DMA2_CHECKSUM;
	} else if (channel == C_DMA_CH3) {
		checksum = R_DMA3_CHECKSUM;
	} else if (channel == C_DMA_CH4) {
		checksum = R_DMA4_CHECKSUM;
	} else if (channel == C_DMA_CH5) {
		checksum = R_DMA5_CHECKSUM;
	} else if (channel == C_DMA_CH6) {
		checksum = R_DMA6_CHECKSUM;
	} else if (channel == C_DMA_CH7) {
		checksum = R_DMA7_CHECKSUM;
	}

	return checksum;

}


static void handle_dma_done(INT32S channel)
{
	#if (K_DMA_TIMEOUT_PATCH_APPLY == 1)
	dma_previous_done_state[channel] = C_DMA_STATUS_DONE;
	#endif
    dma_checksum[channel] = dma_checksum_get(channel);

	if (dma_user_isr[channel])
	{
        (*dma_user_isr[channel])((INT8U)channel, C_DMA_STATUS_DONE);
	}

	if (dma_notify_variable[channel]) {
		*((INT8S *) dma_notify_variable[channel]) = (INT8S) C_DMA_STATUS_DONE;
	}

	if (dma_notify_queue[channel]) {
        INT32U msg = C_DMA_STATUS_DONE;
		osMessagePut(dma_notify_queue[channel], (uint32_t)&msg, osWaitForever);
	}
}

#if (K_DMA_TIMEOUT_PATCH_APPLY == 1)

#define K_DMA_DBG_TIMEOUT_DUMMY_BYTE	0 // Just for DEBUG, must off when release code

// A dummy copy 64 bytes memory to memory to patch dma when dma abnormal complete
// it should be called when dma timeout by hardware or software.
// If the original dma involve IO device, should first stop the IO device in registered
// timeout isr, then call this patch function.
// If didnt doing this way, it will have chance that the next dma will fill with 16 garbage
// bytes. This patch working by just doing a dummy copy so that the garbage byte be flushed.
static ALIGN32 INT32U dma_dummy_src_buf[16];
static ALIGN32 INT32U dma_dummy_dst_buf[16];

void dma_patch_dummy_copy(INT32U channel)
{
#if (K_DMA_DBG_TIMEOUT_DUMMY_BYTE == 1)
static INT8U dma_dummy_src_inited = 0;
#endif
	INT32U len, ctrl;
	INT16U src_type, target_type;
	INT32U count = 16;
	INT32U s_addr = (INT32U)(&dma_dummy_src_buf[0]);
	INT32U t_addr = (INT32U)(&dma_dummy_dst_buf[0]);

	ctrl = C_DMA_CTRL_32BIT | C_DMA_CTRL_INT | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_ENABLE;
	len = count << 2;
	src_type = C_DMA_MEMORY;
	target_type = C_DMA_MEMORY;
	dma_set_source(channel, s_addr);
	dma_set_target(channel, t_addr);
	dma_set_tx_count(channel, count);
	dma_set_timeout(channel, 16);
	dma_set_sprite_size(channel, 0);
	ctrl |= C_DMA_CTRL_SINGLE_TRANS | C_DMA_CTRL_SRC_INCREASE | C_DMA_CTRL_DEST_INCREASE | C_DMA_CTRL_SOFTWARE;
	#if (K_DMA_DBG_TIMEOUT_DUMMY_BYTE == 1)
	if (!dma_dummy_src_inited)
	{
		INT8U *pbyte = (INT8U *)(&dma_dummy_src_buf[0]);
		INT8U k;

		for (k=0; k< 64; k++)
			pbyte[k] = k;
		// Drain source memory and invalid target memory from cache
		#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
		cache_drain_range(s_addr, len);
		cache_invalid_range(t_addr, len);
		#endif
		dma_dummy_src_inited = 1;
	}
	#endif
	ctrl |= C_DMA_CTRL_BURST8_ACCESS;
	dma_set_control(channel, ctrl);
}

static void handle_dma_timout_first_round(INT32S channel)
{
	dma_previous_done_state[channel] = C_DMA_STATUS_TIMEOUT;
	if (dma_user_isr[channel])
	{
        (*dma_user_isr[channel])((INT8U)channel, C_DMA_STATUS_TIMEOUT);
	}
	dma_reset_channel(channel);
	dma_patch_dummy_copy(channel);
}

static void handle_dma_timout_second_round(INT32S channel)
{
	dma_previous_done_state[channel] = C_DMA_STATUS_DONE;

	if (dma_notify_variable[channel]) {
		*((INT8S *) dma_notify_variable[channel]) = (INT8S) C_DMA_STATUS_TIMEOUT;
	}

	if (dma_notify_queue[channel]) {
        INT32U msg = C_DMA_STATUS_TIMEOUT;
		osMessagePut(dma_notify_queue[channel], (uint32_t)&msg, osWaitForever);
	}

	#if (K_DMA_DBG_TIMEOUT_DUMMY_BYTE == 1)
	{
		INT8U *pbyte = (INT8U *)(&dma_dummy_dst_buf[0]);
		INT8U match = 1;
        INT8U k;

		for (k=0; k< 64; k++)
		{
			if (pbyte[k] != k)
			{
				match = 0;
				DBG_PRINT("\r\ndma dummy buffer become dirty! k[%u]=0x%02x\r\n", k, pbyte[k]);
				//break;
			}
		}
		#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
		cache_invalid_range((INT32U)pbyte, 64);
		#endif
	}
	#endif
}

static void dma_irq_handler(INT32U pending)							// Device ISR
{
	// DMA0
	if (pending & C_DMA0_INT_PEND) {		// DMA0 interrupt is pending
		if (dma_usage[C_DMA_CH0] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA0_INT_PEND;		// Clear pending bit
			if (pending & C_DMA0_TIMEOUT) {
				R_DMA_INT = C_DMA0_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH0);
			} else {
				if (dma_previous_done_state[C_DMA_CH0] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH0);
				else
					handle_dma_done(C_DMA_CH0);
				if (dma_usage[C_DMA_CH0] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH0);
				}
			}
		}
	}

	// DMA1
	if (pending & C_DMA1_INT_PEND) {		// DMA1 interrupt is pending
		if (dma_usage[C_DMA_CH1] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA1_INT_PEND;		// Clear pending bit
			if (pending & C_DMA1_TIMEOUT) {
				R_DMA_INT = C_DMA1_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH1);
			} else {
				if (dma_previous_done_state[C_DMA_CH1] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH1);
				else
					handle_dma_done(C_DMA_CH1);

				if (dma_usage[C_DMA_CH1] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH1);
				}
			}
		}
	}

	// DMA2
	if (pending & C_DMA2_INT_PEND) {		// DMA2 interrupt is pending
		if (dma_usage[C_DMA_CH2] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA2_INT_PEND;		// Clear pending bit
			if (pending & C_DMA2_TIMEOUT) {
				R_DMA_INT = C_DMA2_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH2);
			} else {
				if (dma_previous_done_state[C_DMA_CH2] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH2);
				else
					handle_dma_done(C_DMA_CH2);
				if (dma_usage[C_DMA_CH2] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH2);
				}
			}
		}
	}

	// DMA3
	if (pending & C_DMA3_INT_PEND) {		// DMA3 interrupt is pending
		if (dma_usage[C_DMA_CH3] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA3_INT_PEND;		// Clear pending bit
			if (pending & C_DMA3_TIMEOUT) {
				R_DMA_INT = C_DMA3_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH3);
			} else {
				if (dma_previous_done_state[C_DMA_CH3] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH3);
				else
					handle_dma_done(C_DMA_CH3);

				if (dma_usage[C_DMA_CH3] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH3);
				}
			}
		}
	}

	// DMA4
	if (pending & C_DMA4_INT_PEND) {		// DMA4 interrupt is pending
		if (dma_usage[C_DMA_CH4] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA4_INT_PEND;		// Clear pending bit
			if (pending & C_DMA4_TIMEOUT) {
				R_DMA_INT = C_DMA4_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH4);
			} else {
				if (dma_previous_done_state[C_DMA_CH4] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH4);
				else
					handle_dma_done(C_DMA_CH4);

				if (dma_usage[C_DMA_CH4] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH4);
				}
			}
		}
	}

	// DMA5
	if (pending & C_DMA5_INT_PEND) {		// DMA5 interrupt is pending
		if (dma_usage[C_DMA_CH5] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA5_INT_PEND;		// Clear pending bit
			if (pending & C_DMA5_TIMEOUT) {
				R_DMA_INT = C_DMA5_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH5);
			} else {
				if (dma_previous_done_state[C_DMA_CH5] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH5);
				else
					handle_dma_done(C_DMA_CH5);

				if (dma_usage[C_DMA_CH5] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH5);
				}
			}
		}
	}

	// DMA6
	if (pending & C_DMA6_INT_PEND) {		// DMA6 interrupt is pending
		if (dma_usage[C_DMA_CH6] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA6_INT_PEND;		// Clear pending bit
			if (pending & C_DMA6_TIMEOUT) {
				R_DMA_INT = C_DMA6_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH6);
			} else {
				if (dma_previous_done_state[C_DMA_CH6] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH6);
				else
					handle_dma_done(C_DMA_CH6);

				if (dma_usage[C_DMA_CH6] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH6);
				}
			}
		}
	}

	// DMA7
	if (pending & C_DMA7_INT_PEND) {		// DMA7 interrupt is pending
		if (dma_usage[C_DMA_CH7] != C_DMA_NOT_UESED) {
			R_DMA_INT = C_DMA7_INT_PEND;		// Clear pending bit
			if (pending & C_DMA7_TIMEOUT) {
				R_DMA_INT = C_DMA7_TIMEOUT;		// Clear timeout bit
				handle_dma_timout_first_round(C_DMA_CH7);
			} else {
				if (dma_previous_done_state[C_DMA_CH7] == C_DMA_STATUS_TIMEOUT)
					handle_dma_timout_second_round(C_DMA_CH7);
				else
					handle_dma_done(C_DMA_CH7);

				if (dma_usage[C_DMA_CH7] != C_DMA_OCCUPIED) {
					dma_free_channel(C_DMA_CH7);
				}
			}
		}
	}
}

#else

static void handle_dma_timout(INT32S channel)
{
	if (dma_user_isr[channel])
	{
        (*dma_user_isr[channel])((INT8U)channel, C_DMA_STATUS_TIMEOUT);
	}

	//dma_reset_channel(channel);

	if (dma_notify_variable[channel]) {
		*((INT8S *) dma_notify_variable[channel]) = (INT8S) C_DMA_STATUS_TIMEOUT;
	}

	if (dma_notify_queue[channel]) {
        INT32U msg = C_DMA_STATUS_TIMEOUT;
		osMessagePut(dma_notify_queue[channel], (uint32_t)&msg, osWaitForever);
	}
}

static void dma_irq_handler(INT32U pending)							// Device ISR
{
	// DMA0
	if (pending & C_DMA0_INT_PEND) {		// DMA0 interrupt is pending
		if (dma_usage[C_DMA_CH0] != C_DMA_NOT_UESED) {
			if (pending & C_DMA0_TIMEOUT) {
				handle_dma_timout(C_DMA_CH0);
			} else {
				handle_dma_done(C_DMA_CH0);
			}

			if (dma_usage[C_DMA_CH0] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH0);
			}
		}
		R_DMA_INT = C_DMA0_INT_PEND;		// Clear pending bit
	}

	// DMA1
	if (pending & C_DMA1_INT_PEND) {		// DMA1 interrupt is pending
		if (dma_usage[C_DMA_CH1] != C_DMA_NOT_UESED) {
			if (pending & C_DMA1_TIMEOUT) {
				handle_dma_timout(C_DMA_CH1);
			} else {
				handle_dma_done(C_DMA_CH1);
			}

			if (dma_usage[C_DMA_CH1] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH1);
			}
		}
		R_DMA_INT = C_DMA1_INT_PEND;		// Clear pending bit
	}

	// DMA2
	if (pending & C_DMA2_INT_PEND) {		// DMA2 interrupt is pending
		if (dma_usage[C_DMA_CH2] != C_DMA_NOT_UESED) {
			if (pending & C_DMA2_TIMEOUT) {
				handle_dma_timout(C_DMA_CH2);
			} else {
				handle_dma_done(C_DMA_CH2);
			}

			if (dma_usage[C_DMA_CH2] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH2);
			}
		}
		R_DMA_INT = C_DMA2_INT_PEND;		// Clear pending bit
	}

	// DMA3
	if (pending & C_DMA3_INT_PEND) {		// DMA3 interrupt is pending
		if (dma_usage[C_DMA_CH3] != C_DMA_NOT_UESED) {
			if (pending & C_DMA3_TIMEOUT) {
				handle_dma_timout(C_DMA_CH3);
			} else {
				handle_dma_done(C_DMA_CH3);
			}

			if (dma_usage[C_DMA_CH3] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH3);
			}
		}
		R_DMA_INT = C_DMA3_INT_PEND;		// Clear pending bit
	}

	// DMA4
	if (pending & C_DMA4_INT_PEND) {		// DMA4 interrupt is pending
		if (dma_usage[C_DMA_CH4] != C_DMA_NOT_UESED) {
			if (pending & C_DMA4_TIMEOUT) {
				handle_dma_timout(C_DMA_CH4);
			} else {
				handle_dma_done(C_DMA_CH4);
			}

			if (dma_usage[C_DMA_CH4] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH4);
			}
		}
		R_DMA_INT = C_DMA4_INT_PEND;		// Clear pending bit
	}

	// DMA5
	if (pending & C_DMA5_INT_PEND) {		// DMA5 interrupt is pending
		if (dma_usage[C_DMA_CH5] != C_DMA_NOT_UESED) {
			if (pending & C_DMA5_TIMEOUT) {
				handle_dma_timout(C_DMA_CH5);
			} else {
				handle_dma_done(C_DMA_CH5);
			}

			if (dma_usage[C_DMA_CH5] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH5);
			}
		}
		R_DMA_INT = C_DMA5_INT_PEND;		// Clear pending bit
	}

	// DMA6
	if (pending & C_DMA6_INT_PEND) {		// DMA6 interrupt is pending
		if (dma_usage[C_DMA_CH6] != C_DMA_NOT_UESED) {
			if (pending & C_DMA6_TIMEOUT) {
				handle_dma_timout(C_DMA_CH6);
			} else {
				handle_dma_done(C_DMA_CH6);
			}

			if (dma_usage[C_DMA_CH6] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH6);
			}
		}
		R_DMA_INT = C_DMA6_INT_PEND;		// Clear pending bit
	}

	// DMA7
	if (pending & C_DMA7_INT_PEND) {		// DMA7 interrupt is pending
		if (dma_usage[C_DMA_CH7] != C_DMA_NOT_UESED) {
			if (pending & C_DMA7_TIMEOUT) {
				handle_dma_timout(C_DMA_CH7);
			} else {
				handle_dma_done(C_DMA_CH7);
			}

			if (dma_usage[C_DMA_CH7] != C_DMA_OCCUPIED) {
				dma_free_channel(C_DMA_CH7);
			}
		}
		R_DMA_INT = C_DMA7_INT_PEND;		// Clear pending bit
	}
}
#endif

void DMA_IRQHandler(void)							// Device ISR
{
	dma_irq_handler(R_DMA_INT);
}

/**
 * @brief   dma initialize
 * @param   none
 * @return 	none
 */
void drv_l1_dma_init(void)
{
	INT32U i;

	R_DMA0_CTRL	= C_DMA_CTRL_RESET;			// Software reset, this bit will auto clear after reset complete
	R_DMA0_TX_COUNT	= 0x0;
	R_DMA0_SPRITE_SIZE = 0x0;
	R_DMA0_TRANSPARENT = 0x0;
	R_DMA0_MISC = 0x0;

	R_DMA1_CTRL	= C_DMA_CTRL_RESET;
	R_DMA1_TX_COUNT	= 0x0;
	R_DMA1_SPRITE_SIZE = 0x0;
	R_DMA1_TRANSPARENT = 0x0;
	R_DMA1_MISC = 0x0;

	R_DMA2_CTRL	= C_DMA_CTRL_RESET;
	R_DMA2_TX_COUNT	= 0x0;
	R_DMA2_SPRITE_SIZE = 0x0;
	R_DMA2_TRANSPARENT = 0x0;
	R_DMA2_MISC = 0x0;

	R_DMA3_CTRL	= C_DMA_CTRL_RESET;
	R_DMA3_TX_COUNT	= 0x0;
	R_DMA3_SPRITE_SIZE = 0x0;
	R_DMA3_TRANSPARENT = 0x0;
	R_DMA3_MISC = 0x0;

	R_DMA4_CTRL	= C_DMA_CTRL_RESET;
	R_DMA4_TX_COUNT	= 0x0;
	R_DMA4_SPRITE_SIZE = 0x0;
	R_DMA4_TRANSPARENT = 0x0;
	R_DMA4_MISC = 0x0;

	R_DMA5_CTRL	= C_DMA_CTRL_RESET;
	R_DMA5_TX_COUNT	= 0x0;
	R_DMA5_SPRITE_SIZE = 0x0;
	R_DMA5_TRANSPARENT = 0x0;
	R_DMA5_MISC = 0x0;

	R_DMA6_CTRL	= C_DMA_CTRL_RESET;
	R_DMA6_TX_COUNT	= 0x0;
	R_DMA6_SPRITE_SIZE = 0x0;
	R_DMA6_TRANSPARENT = 0x0;
	R_DMA6_MISC = 0x0;

	R_DMA7_CTRL	= C_DMA_CTRL_RESET;
	R_DMA7_TX_COUNT	= 0x0;
	R_DMA7_SPRITE_SIZE = 0x0;
	R_DMA7_TRANSPARENT = 0x0;
	R_DMA7_MISC = 0x0;

	R_DMA_LINE_LEN = 0x0;
	R_DMA_DEVICE = 0x76543210;
	R_DMA_CEMODE = C_DMA_CE_DONT_RESET;
	R_DMA_INT = 0xFFFFFFFF;						// Clear all pending bits

	for (i=0; i<C_DMA_CHANNEL_NUM; i++) {
		dma_usage[i] = (INT8U) C_DMA_NOT_UESED;
		dma_notify_variable[i] = (INT32U) NULL;
		dma_notify_queue[i] = (osMessageQId) NULL;
		dma_checksum[i] = (INT32U) NULL;
	}

	for (i=0; i<C_DMA_Q_NUM; i++) {
		dma_q_usage[i] = (INT8U) FALSE;
		if (dma_driver_queue[i]) {
            OSQFlush(dma_driver_queue[i]);
		} else {
			osMessageQDef_t dma_q = {C_DMA_Q_BUF_SIZE, sizeof(INT32U), 0};

			dma_driver_queue[i] = osMessageCreate(&dma_q, NULL);
		}
	}

    NVIC_SetPriority(DMA_IRQn, 5);
    NVIC_EnableIRQ(DMA_IRQn);

    dma_init_done = TRUE;
}

/**
 * @brief   set dma direction
 * @param   channel: dma channel number
 * @param   dir: direction, frame buffer to sprite or sprite to frame buffer
 * @return 	none
 */
static void dma_set_direction(INT32U channel, INT32U dir)
{
	switch (channel) {
	case C_DMA_CH0:
		if(dir)
			R_DMA0_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA0_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	case C_DMA_CH1:
		if(dir)
			R_DMA1_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA1_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	case C_DMA_CH2:
		if(dir)
			R_DMA2_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA2_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	case C_DMA_CH3:
		if(dir)
			R_DMA3_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA3_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	case C_DMA_CH4:
		if(dir)
			R_DMA4_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA4_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	case C_DMA_CH5:
		if(dir)
			R_DMA5_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA5_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	case C_DMA_CH6:
		if(dir)
			R_DMA6_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA6_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	case C_DMA_CH7:
		if(dir)
			R_DMA7_MISC |= C_DMA_MISC_FB_TO_SPRITE;
		else
			R_DMA7_MISC &= ~C_DMA_MISC_FB_TO_SPRITE;
		break;
	default:
		break;
	}
}

/**
 * @brief   set dma control register
 * @param   channel: dma channel number
 * @param   ctrl: dma control register value
 * @return 	none
 */
static void dma_set_control(INT32U channel, INT32U ctrl)
{
	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_CTRL = ctrl;
		break;
	case C_DMA_CH1:
		R_DMA1_CTRL = ctrl;
		break;
	case C_DMA_CH2:
		R_DMA2_CTRL = ctrl;
		break;
	case C_DMA_CH3:
		R_DMA3_CTRL = ctrl;
		break;
	case C_DMA_CH4:
		R_DMA4_CTRL = ctrl;
		break;
	case C_DMA_CH5:
		R_DMA5_CTRL = ctrl;
		break;
	case C_DMA_CH6:
		R_DMA6_CTRL = ctrl;
		break;
	case C_DMA_CH7:
		R_DMA7_CTRL = ctrl;
		break;
	default:
		break;
	}
}

/**
 * @brief   set dma source address
 * @param   channel: dma channel number
 * @param   addr: dma source address
 * @return 	none
 */
static void dma_set_source(INT32U channel, INT32U addr)
{
	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_SRC_ADDR = addr;
		break;
	case C_DMA_CH1:
		R_DMA1_SRC_ADDR = addr;
		break;
	case C_DMA_CH2:
		R_DMA2_SRC_ADDR = addr;
		break;
	case C_DMA_CH3:
		R_DMA3_SRC_ADDR = addr;
		break;
	case C_DMA_CH4:
		R_DMA4_SRC_ADDR = addr;
		break;
	case C_DMA_CH5:
		R_DMA5_SRC_ADDR = addr;
		break;
	case C_DMA_CH6:
		R_DMA6_SRC_ADDR = addr;
		break;
	case C_DMA_CH7:
		R_DMA7_SRC_ADDR = addr;
		break;
	default:
		break;
	}
}

/**
 * @brief   set dma destination address
 * @param   channel: dma channel number
 * @param   addr: dma destination address
 * @return 	none
 */
static void dma_set_target(INT32U channel, INT32U addr)
{
	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_TAR_ADDR = addr;
		break;
	case C_DMA_CH1:
		R_DMA1_TAR_ADDR = addr;
		break;
	case C_DMA_CH2:
		R_DMA2_TAR_ADDR = addr;
		break;
	case C_DMA_CH3:
		R_DMA3_TAR_ADDR = addr;
		break;
	case C_DMA_CH4:
		R_DMA4_TAR_ADDR = addr;
		break;
	case C_DMA_CH5:
		R_DMA5_TAR_ADDR = addr;
		break;
	case C_DMA_CH6:
		R_DMA6_TAR_ADDR = addr;
		break;
	case C_DMA_CH7:
		R_DMA7_TAR_ADDR = addr;
		break;
	default:
		break;
	}
}

/**
 * @brief   set dma transfer length
 * @param   channel: dma channel number
 * @param   count: transfer length
 * @return 	result: >=0 is success, <0 is fail.
 */
static INT32S dma_set_tx_count(INT32U channel, INT32U count)
{
	if (count > C_DMA_COUNT_MAX) {
		return -1;
	}

	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_TX_COUNT = count;
		break;
	case C_DMA_CH1:
		R_DMA1_TX_COUNT = count;
		break;
	case C_DMA_CH2:
		R_DMA2_TX_COUNT = count;
		break;
	case C_DMA_CH3:
		R_DMA3_TX_COUNT = count;
		break;
	case C_DMA_CH4:
		R_DMA4_TX_COUNT = count;
		break;
	case C_DMA_CH5:
		R_DMA5_TX_COUNT = count;
		break;
	case C_DMA_CH6:
		R_DMA6_TX_COUNT = count;
		break;
	case C_DMA_CH7:
		R_DMA7_TX_COUNT = count;
		break;
	default:
		return -1;
	}
	return 0;
}

/**
 * @brief   get dma transfer length
 * @param   channel: dma channel number
 * @param   pcount: return transfer length
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S dma_get_tx_count(INT32U channel, INT32U *pcount)
{
    *pcount = 0;
	switch (channel) {
	case C_DMA_CH0:
		*pcount = R_DMA0_TX_COUNT & 0x00FFFFFF;
		break;
	case C_DMA_CH1:
		*pcount = R_DMA1_TX_COUNT & 0x00FFFFFF;
		break;
	case C_DMA_CH2:
		*pcount = R_DMA2_TX_COUNT & 0x00FFFFFF;
		break;
	case C_DMA_CH3:
		*pcount = R_DMA3_TX_COUNT & 0x00FFFFFF;
		break;
	case C_DMA_CH4:
		*pcount = R_DMA4_TX_COUNT & 0x00FFFFFF;
		break;
	case C_DMA_CH5:
		*pcount = R_DMA5_TX_COUNT & 0x00FFFFFF;
		break;
	case C_DMA_CH6:
		*pcount = R_DMA6_TX_COUNT & 0x00FFFFFF;
		break;
	case C_DMA_CH7:
		*pcount = R_DMA7_TX_COUNT & 0x00FFFFFF;
		break;
	default:
		return -1;
	}
	return 0;
}

/**
 * @brief   set dma transfer device number
 * @param   channel: dma channel number
 * @param   count: device number
 * @return 	result: >=0 is success, <0 is fail.
 */
static INT32S dma_set_device(INT32U channel, INT16U device)
{
	INT32U shift = 0, bshift = 0;
	INT32U bank = device >> 4;

	if ((channel>=C_DMA_CHANNEL_NUM) || (device>C_DMA_IO_MAX)) {
		return -1;
	}

	if (channel == C_DMA_CH0) {
		shift = C_DMA0_IO_SHIFT;
		bshift = C_DMA0_BANK_SHIFT;
	} else if (channel == C_DMA_CH1) {
		shift = C_DMA1_IO_SHIFT;
		bshift = C_DMA1_BANK_SHIFT;
	} else if (channel == C_DMA_CH2) {
		shift = C_DMA2_IO_SHIFT;
		bshift = C_DMA2_BANK_SHIFT;
	} else if (channel == C_DMA_CH3) {
		shift = C_DMA3_IO_SHIFT;
		bshift = C_DMA3_BANK_SHIFT;
	} else if (channel == C_DMA_CH4) {
		shift = C_DMA4_IO_SHIFT;
		bshift = C_DMA4_BANK_SHIFT;
	} else if (channel == C_DMA_CH5) {
		shift = C_DMA5_IO_SHIFT;
		bshift = C_DMA5_BANK_SHIFT;
	} else if (channel == C_DMA_CH6) {
		shift = C_DMA6_IO_SHIFT;
		bshift = C_DMA6_BANK_SHIFT;
	} else if (channel == C_DMA_CH7) {
		shift = C_DMA7_IO_SHIFT;
		bshift = C_DMA7_BANK_SHIFT;
	}

    taskENTER_CRITICAL();
	R_DMA_CEMODE &= ~(0x01 << bshift);
	R_DMA_CEMODE |= (bank << bshift);

	R_DMA_DEVICE &= ~((C_DMA_IO_MASK) << shift);
	R_DMA_DEVICE |= ((device & C_DMA_IO_MASK) << shift);
    taskEXIT_CRITICAL();

	return 0;
}

/**
 * @brief   set dma timeout value
 * @param   channel: dma channel number
 * @param   timeout: timeout value
 * @return 	result: >=0 is success, <0 is fail.
 */
static INT32S dma_set_timeout(INT32U channel, INT32U timeout)
{
	if((channel>=C_DMA_CHANNEL_NUM) || (timeout>C_DMA_MISC_TIMEOUT_MAX)) {
		return -1;
	}

	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA0_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	case C_DMA_CH1:
		R_DMA1_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA1_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	case C_DMA_CH2:
		R_DMA2_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA2_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	case C_DMA_CH3:
		R_DMA3_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA3_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	case C_DMA_CH4:
		R_DMA4_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA4_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	case C_DMA_CH5:
		R_DMA5_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA5_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	case C_DMA_CH6:
		R_DMA6_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA6_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	case C_DMA_CH7:
		R_DMA7_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK);
		R_DMA7_MISC |= timeout << C_DMA_MISC_TIMEOUT_SHIFT;
		break;
	default:
		break;
	}

	return 0;
}

/**
 * @brief   set dma line length
 * @param   length: line length for LCD frame buffer
 * @return 	result: >=0 is success, <0 is fail.
 */
static INT32S dma_set_line_length(INT32U length)
{
	if  (length > C_DMA_LINE_MAX) {
		return -1;
	}

	R_DMA_LINE_LEN = length;
	return 0;
}

/**
 * @brief   set dma sprite size
 * @param   channel: dma channel
 * @param   size: sprite size
 * @return 	result: >=0 is success, <0 is fail.
 */
static INT32S dma_set_sprite_size(INT32U channel, INT32U size)
{
	if  (size > C_DMA_SPRITE_MAX) {
		return -1;
	}

	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_SPRITE_SIZE = size;
		break;
	case C_DMA_CH1:
		R_DMA1_SPRITE_SIZE = size;
		break;
	case C_DMA_CH2:
		R_DMA2_SPRITE_SIZE = size;
		break;
	case C_DMA_CH3:
		R_DMA3_SPRITE_SIZE = size;
		break;
	case C_DMA_CH4:
		R_DMA4_SPRITE_SIZE = size;
		break;
	case C_DMA_CH5:
		R_DMA5_SPRITE_SIZE = size;
		break;
	case C_DMA_CH6:
		R_DMA6_SPRITE_SIZE = size;
		break;
	case C_DMA_CH7:
		R_DMA7_SPRITE_SIZE = size;
		break;
	default:
		return -1;
	}

	return 0;
}

#if 0
static INT32S dma_set_transparent_enable(INT32U channel)
{
	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	case C_DMA_CH1:
		R_DMA1_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	case C_DMA_CH2:
		R_DMA2_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	case C_DMA_CH3:
		R_DMA3_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	case C_DMA_CH4:
		R_DMA4_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	case C_DMA_CH5:
		R_DMA5_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	case C_DMA_CH6:
		R_DMA6_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	case C_DMA_CH7:
		R_DMA7_MISC |= C_DMA_MISC_TRANSPARENT_EN;
		break;
	default:
		return -1;
	}

	return 0;
}

static INT32S dma_set_transparent_disable(INT32U channel)
{
	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	case C_DMA_CH1:
		R_DMA1_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	case C_DMA_CH2:
		R_DMA2_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	case C_DMA_CH3:
		R_DMA3_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	case C_DMA_CH4:
		R_DMA4_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	case C_DMA_CH5:
		R_DMA5_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	case C_DMA_CH6:
		R_DMA6_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	case C_DMA_CH7:
		R_DMA7_MISC &= ~(C_DMA_MISC_TRANSPARENT_EN);
		break;
	default:
		return -1;
	}

	return 0;
}

static INT32S dma_set_transparent_pattern(INT32U channel, INT16U pattern)
{
	if (pattern > C_DMA_TRANSPARENT_MAX) {
		return -1;
	}

	switch (channel) {
	case C_DMA_CH0:
		R_DMA0_TRANSPARENT = pattern;
		break;
	case C_DMA_CH1:
		R_DMA1_TRANSPARENT = pattern;
		break;
	case C_DMA_CH2:
		R_DMA2_TRANSPARENT = pattern;
		break;
	case C_DMA_CH3:
		R_DMA3_TRANSPARENT = pattern;
		break;
	case C_DMA_CH4:
		R_DMA4_TRANSPARENT = pattern;
		break;
	case C_DMA_CH5:
		R_DMA5_TRANSPARENT = pattern;
		break;
	case C_DMA_CH6:
		R_DMA6_TRANSPARENT = pattern;
		break;
	case C_DMA_CH7:
		R_DMA7_TRANSPARENT = pattern;
		break;
	default:
		return -1;
	}

	return 0;
}
#endif

#if AES_ENABLE
static INT32S aes_load_key_wait(INT32U time_out)
{
	INT32U i;

	for(i=0;i<time_out;i++)
	{
		if((R_AES_LOAD_KEY & 0x10) == 0) {
			return 0;
		} else {
			INT32U j;
			INT32U loopCnt = 0xf; // [200k]0x4FF;  // [100k]0x3FF;

			for(j=0; j<loopCnt; j++) {
				j = j;
			}
		}
	}

	return -1;
}
#endif

static INT32S dma_transfer_extend(DMA_STRUCT *dma_struct, INT8U usage, INT32U os_q, void (*dma_user_isr)(INT8U, INT8U))
{
	INT32U s_addr = dma_struct->s_addr;
	INT32U t_addr = dma_struct->t_addr;
	INT32U count = dma_struct->count;
	INT32U channel, len, ctrl;
	INT16U src_type, target_type;
	INT32U des_mode = 0;
	INT32U option = 0;
	INT8U is_sd_dma = 0;

	if (!count) {
		if (dma_struct->notify) {
			*(dma_struct->notify) = C_DMA_STATUS_DONE;
		}
		return 0;
	}

	if (dma_struct->width == DMA_DATA_WIDTH_1BYTE) {
		ctrl = C_DMA_CTRL_8BIT | C_DMA_CTRL_INT | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_ENABLE;
		len = count;
	} else if (dma_struct->width == DMA_DATA_WIDTH_2BYTE) {
		if ((s_addr&0x1) || (t_addr&0x1)) {
			return -1;						// Both source and target address must be 2-byte alignment
		}

		ctrl = C_DMA_CTRL_16BIT | C_DMA_CTRL_INT | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_ENABLE;
		len = count << 1;
	} else if (dma_struct->width == DMA_DATA_WIDTH_4BYTE) {
		if ((s_addr&0x3) || (t_addr&0x3)) {
			return -1;						// Both source and target address must be 4-byte alignment
		}

		ctrl = C_DMA_CTRL_32BIT | C_DMA_CTRL_INT | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_ENABLE;
		len = count << 2;
	} else {
		return -1;
	}

	if ((s_addr<C_DMA_IO_ADDR_START) || (s_addr>C_DMA_IO_ADDR_END)) {
		src_type = C_DMA_MEMORY;
	} else {
		if (s_addr == (INT32U)P_UART0_BASE) {
			src_type = C_DMA_IO_UART0_RX;
		} else if (s_addr == (INT32U)P_UART1_BASE) {
			src_type = C_DMA_IO_UART1_RX;
		} else if (s_addr == (INT32U)P_UART2_BASE) {
			src_type = C_DMA_IO_UART2_RX;
		} else if (s_addr == (INT32U) P_SPI0_RX_DATA) {
			src_type = C_DMA_IO_SPI0_RX;
		} else if (s_addr == (INT32U) P_SPI1_RX_DATA) {
			src_type = C_DMA_IO_SPI1_RX;
		} else if (s_addr == (INT32U) P_SDC0_DATA_RX) {
			src_type = C_DMA_IO_SDC0;
			is_sd_dma = 0x81; // bit 7 mean sd0
       } else if (s_addr == (INT32U) P_SDC1_DATA_RX) {
			src_type = C_DMA_IO_SDC1;
			is_sd_dma = 0x01;
		} else if (s_addr == (INT32U) P_ADC_ASADC_DATA) {
			src_type = C_DMA_IO_ADC;
		} else if (s_addr == (INT32U) P_I2SRX_DATA) {
			src_type = C_DMA_IO_I2S_RX;
                } else if (s_addr == (INT32U) P_I2S1RX_DATA) {
			src_type = C_DMA_IO_I2S1_RX;
                } else if (s_addr == (INT32U) P_I2S2RX_DATA) {
			src_type = C_DMA_IO_I2S2_RX;
                } else if (s_addr == (INT32U) P_I2S3RX_DATA) {
			src_type = C_DMA_IO_I2S3_RX;
		} else if (s_addr == (INT32U) P_FIR_RXFIFO_BASE) {
			src_type = C_DMA_IO_FIR_RX;
		} else if (s_addr == (INT32U) P_FIR_TXFIFO_BASE) {
			src_type = C_DMA_IO_FIR_RX2;
		} else {
			return -4;						// Unknow IO
		}
	}
	if ((t_addr<C_DMA_IO_ADDR_START) || (t_addr>C_DMA_IO_ADDR_END)) {
		target_type = C_DMA_MEMORY;
	} else {
		if (src_type != C_DMA_MEMORY) {
			return -1;						// IO to IO is not supported
		}

		if (t_addr == (INT32U) P_UART0_BASE) {
			target_type = C_DMA_IO_UART0_TX;
		} else if (t_addr == (INT32U) P_UART1_BASE) {
			target_type = C_DMA_IO_UART1_TX;
		} else if (t_addr == (INT32U) P_UART2_BASE) {
			target_type = C_DMA_IO_UART2_TX;
		} else if (t_addr == (INT32U) P_SPI0_TX_DATA) {
			target_type = C_DMA_IO_SPI0_TX;
		} else if (t_addr == (INT32U) P_SPI1_TX_DATA) {
			target_type = C_DMA_IO_SPI1_TX;
		} else if (t_addr == (INT32U) P_DAC_CHA_DATA) {
			target_type = C_DMA_IO_DAC_CHA;
		} else if (t_addr == (INT32U) P_DAC_CHB_DATA) {
			target_type = C_DMA_IO_DAC_CHB;
		} else if (t_addr == (INT32U) P_SDC0_DATA_TX) {
			target_type = C_DMA_IO_SDC0;
			is_sd_dma = 0x82; // bit 7 mean sd0
                } else if (t_addr == (INT32U) P_SDC1_DATA_TX) {
			target_type = C_DMA_IO_SDC1;
			is_sd_dma = 0x02;
		} else if (t_addr == (INT32U) P_I2STX_DATA) {
			target_type = C_DMA_IO_I2S_TX;
		} else if (t_addr == (INT32U) P_I2S1TX_DATA) {
			target_type = C_DMA_IO_I2S1_TX;
                } else if (t_addr == (INT32U) P_I2S2TX_DATA) {
			target_type = C_DMA_IO_I2S2_TX;
                } else if (t_addr == (INT32U) P_I2S3TX_DATA) {
			target_type = C_DMA_IO_I2S3_TX;
		} else if (t_addr == (INT32U) P_FIR_TXFIFO_BASE) {
			target_type = C_DMA_IO_FIR_TX;
		} else {
			return -5;						// Unknow IO
		}
	}

	if (!dma_init_done) {
		drv_l1_dma_init();
	}

#if AES_ENABLE
	// aes only support in dma2
	if(dma_struct->aes && dma_struct->aes->enable)
	{
		channel = 2;
		dma_struct->channel = 2;
		if(dma_set_channel(usage, 2) < 0) {
			return -1;							// DMA channel is not available
		}

		*((volatile INT32U *)0xD03000B8) = 0;

		if(dma_struct->aes->func)       //Encrypt
		{
		  //INT32U temp;
		  if(dma_struct->aes->keyrev==0)
		    option |= (1<<4);
		  else if(dma_struct->aes->keyrev==1)
		    option &= (~(1<<4));
		  else if(dma_struct->aes->keyrev==2)
		    option |= (0xF|(1<<4));

		  //temp |= ((dma_struct->aes->option)&0xFFFF);
		  option |= ((1<<9)|(1<<14));
		  if(dma_struct->aes->InRev & 0x01)
		  option &= ~(1<<9);
		  if(dma_struct->aes->InRev & 0x02)
		    option |= (0xF<<5);

		  if(dma_struct->aes->OutRev & 0x01)
		    option &= ~(1<<14);
		  if(dma_struct->aes->OutRev & 0x02)
		    option |= (0xF<<10);


		  R_AES_REV |= option;
		}
		else
		{
		  //INT32U temp;
		  if(dma_struct->aes->keyrev==0)
		    option |= (1<<20);
		  else if(dma_struct->aes->keyrev==1)
		    option &= (~(1<<20));
		  else if(dma_struct->aes->keyrev==2)
		    option |= (0xF0000|(1<<20));

		  //temp |= (((dma_struct->aes->option)&0xFFFF)<<16);
		  option |= ((1<<25)|(1<<30));
		  if(dma_struct->aes->InRev & 0x01)
		    option &= ~(1<<25);
		  if(dma_struct->aes->InRev & 0x02)
		    option |= (0xF<<21);

		  if(dma_struct->aes->OutRev & 0x01)
		    option &= ~(1<<30);
		  if(dma_struct->aes->OutRev & 0x02)
		    option |= (0xF<<26);

		  R_AES_REV |= option;
		}

		R_AES_KEY0 = dma_struct->aes->key[0];
		R_AES_KEY1 = dma_struct->aes->key[1];
		R_AES_KEY2 = dma_struct->aes->key[2];
		R_AES_KEY3 = dma_struct->aes->key[3];
		R_AES_LOAD_KEY = 0x00;
		R_AES_LOAD_KEY = 0x01;

		if(aes_load_key_wait(5))
			return -1;

		option = 0;
		if(dma_struct->aes->mode >0)
		{
		  option = (dma_struct->aes->mode<<13);
		  R_AES_IV_0 = dma_struct->aes->iv[0];
		  R_AES_IV_1 = dma_struct->aes->iv[1];
		  R_AES_IV_2 = dma_struct->aes->iv[2];
		  R_AES_IV_3 = dma_struct->aes->iv[3];

		  if(dma_struct->aes->ivrev==0)
		    option |= (1<<6);         //IV inverse
		  else if(dma_struct->aes->ivrev==1)
		    option &= (~(1<<6));         //IV inverse
		  else if(dma_struct->aes->ivrev==2)
		    option|= ((1<<6)|(1<<5)|(1<<3)|(1<<2)|(1<<1));         //IV inverse
		}

		if(dma_struct->aes->func)
			option |= 0x1000; // Encrypt
		else
			option |= 0x1100; //	Discrypt

		R_AES_LOAD_KEY = option;

	}
	else
#endif
#if DES_3DES_ENABLE
        // des, 3des only support in dma4, could not enable together
	if(dma_struct->des)
	{
        if ((dma_struct->des->enable & 0x3) != 0)
        {
            channel = 4;
            dma_struct->channel = channel;
            if(dma_set_channel(usage, channel) < 0) {
                return -1;							// DMA channel is not available
            }

            R_DES_CTRL = (1<<0);            // enable des engine
            if (dma_struct->des->func == 0) // Descrypt
                R_DES_CTRL |= (1<<1);       // enable Descrypt

            R_DES_KEY0_LSB = (INT32U)(dma_struct->des->key[0]);
            R_DES_KEY0_MSB = (INT32U)(dma_struct->des->key[0] >> 32); // bit 24~31 will be 0

            if ((dma_struct->des->enable & (1<<1)) != 0) // 3des
            {
                R_DES_KEY1_LSB = (INT32U)(dma_struct->des->key[1]);
                R_DES_KEY1_MSB = (INT32U)(dma_struct->des->key[1] >> 32);
                R_DES_KEY2_LSB = (INT32U)(dma_struct->des->key[2]);
                R_DES_KEY2_MSB = (INT32U)(dma_struct->des->key[2] >> 32);
                R_DES_CTRL |= (1<<2);   // enable 3des
            }

            des_mode = dma_struct->des->mode<<16;

            if( des_mode >0 )
            {
                R_DES_IV_LSB = ((INT32U*)(&(dma_struct->des->iv)))[0];
                R_DES_IV_MSB = ((INT32U*)(&(dma_struct->des->iv)))[1];
            }
            option = des_mode;
            if(dma_struct->des->InRev & 0x01) //In Byte Rev
                option|= (1<<4);
            if(dma_struct->des->InRev & 0x02) //In Bit Rev
                option|= (1<<5);

            if(dma_struct->des->OutRev & 0x01) //Out Byte Rev
                option|= (1<<6);
            if(dma_struct->des->OutRev & 0x02) //Out Bit Rev
                option|= (1<<7);

            if(dma_struct->des->keyrev & 0x01)
                option|= (1<<8);
            if(dma_struct->des->keyrev & 0x02)
                option|= (1<<9);

            if(dma_struct->des->ivrev & 0x01)
                option|= (1<<10);
            if(dma_struct->des->ivrev & 0x02)
                option|= (1<<11);

            //R_DES_CTRL |= dma_struct->des->option;
            R_DES_CTRL |= option;


        }
        else
            R_DES_CTRL &= (~(1<<0));

	}
	else
#endif
	{
		channel = dma_get_channel(usage);
		dma_struct->channel = channel;
		if  (channel >= C_DMA_CHANNEL_NUM) {
			return -1;							// No free DMA channel is available
		}
	}

#if 0
	// external interrupt trigger mode.
	if(dma_struct->trigger && dma_struct->trigger->enable) {
		// must use Demand Transfer mode.
		if((src_type == C_DMA_MEMORY) && (target_type == C_DMA_MEMORY)) {
			return -1;
		}

		if(dma_struct->trigger->source > 2) {
			return -1;
		}

		ctrl |= (0x01 << 19); 		//trigger enable
		ctrl &= ~(0x0F << 20); 		//trigger source, exta, eatb, extc...
		ctrl |= ((INT32U)(dma_struct->trigger->source & 0x0F) << 20);
		if(dma_struct->trigger->edge) {
			ctrl |= (0x01 << 24);	//rising edge
		} else {
			ctrl &= ~(0x01 << 24);	//falling edge
		}
	}
#endif

	dma_is_sd[channel] = is_sd_dma;

	dma_set_notify(channel, (INT32U) dma_struct->notify, os_q);

	// Set source address
	dma_set_source(channel, s_addr);

	// Set target address
	dma_set_target(channel, t_addr);

	// Set transmit counter
	if (dma_set_tx_count(channel, count)) {
		dma_free_channel(channel);
		return -1;
	}

	{
		INT32U timeout = (INT32U)(dma_struct->timeout);

//		if (timeout == 0) // williamyeo, when timeout is 0(disable timeout), set it to 255 so that have 1 second timeout
//			timeout = 255;
		if (dma_set_timeout(channel, timeout)) {
			dma_free_channel(channel);
			return -1;
		}
	}

	if (dma_set_sprite_size(channel, 0)) {
		dma_free_channel(channel);
		return -1;
	}

	// Prepare control register
	if (src_type != C_DMA_MEMORY) {			// IO to memory
		ctrl |= C_DMA_CTRL_DEMAND_TRANS | C_DMA_CTRL_IO2M | C_DMA_CTRL_SRC_FIX | C_DMA_CTRL_EXTERNAL;

		if (dma_set_device(channel, src_type)) {
			dma_free_channel(channel);
			return -1;
		}

	#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
		// Invalid target memory from cache
		cache_invalid_range(t_addr, len);
	#endif

	} else if (target_type != C_DMA_MEMORY) {	// Memory to IO
		ctrl |= C_DMA_CTRL_DEMAND_TRANS | C_DMA_CTRL_M2IO | C_DMA_CTRL_DEST_FIX | C_DMA_CTRL_EXTERNAL;

		if (dma_set_device(channel, target_type)) {
			dma_free_channel(channel);
			return -1;
		}

	#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
		// Drain source memory from cache
		cache_drain_range(s_addr, len);
	#endif

	} else {								// Memory to memory
		ctrl |= C_DMA_CTRL_SINGLE_TRANS | C_DMA_CTRL_SRC_INCREASE | C_DMA_CTRL_DEST_INCREASE | C_DMA_CTRL_SOFTWARE;

	#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
		// Drain source memory and invalid target memory from cache
		cache_drain_range(s_addr, len);
		cache_invalid_range(t_addr, len);
	#endif
	}
#if AES_ENABLE
	if(dma_struct->aes)
	  ctrl |= C_DMA_CTRL_BURST4_ACCESS;
	else
#endif
	{
	// Check whether burst mode can be used
	if (usage == C_DMA_OCCUPIED || src_type == C_DMA_IO_FIR_RX || src_type == C_DMA_IO_FIR_RX2 || src_type == C_DMA_IO_FIR_TX || target_type == C_DMA_IO_FIR_TX) {
		ctrl |= C_DMA_CTRL_SINGLE_ACCESS;
	} else if (!(count & 0x7)) {
		ctrl |= C_DMA_CTRL_BURST8_ACCESS;
	} else if (!(count & 0x3)) {
		ctrl |= C_DMA_CTRL_BURST4_ACCESS;
	} else {
		ctrl |= C_DMA_CTRL_SINGLE_ACCESS;
	}
	}

	if(dma_user_isr != NULL)
    {
        drv_l1_dma_callback_set(dma_struct->channel, dma_user_isr);
    }
    else
        drv_l1_dma_callback_clear(dma_struct->channel);

	// Start DMA now

	#if (K_SD_DMA_TEST_CODE == 2) // TEST CODE
	if (sd_dma_test)
	{
		// stop here and plug out card to sim dma timeout
        sd_dma_test = 0;
	}
	#endif

	#if (K_DMA_TIMEOUT_PATCH_APPLY == 1)
	dma_previous_done_state[channel] = C_DMA_STATUS_WAITING;
	#endif

	dma_set_control(channel, ctrl);

	#if (K_SD_DMA_TEST_CODE == 1) // TEST CODE
	if (sd_dma_test)
	{
        sd_dma_test = 0;
		// purpoesely reset dma to sim dma become dead case so that get software osEventTimeout
        //drv_usec_wait(2);
        dma_reset_channel(channel);
    }
	#endif

/*
	if(dma_struct->des && ((dma_struct->des->enable & 0x3) != 0))
	{
        DBG_PRINT("R_DMA4_CTRL[%08x]=%08x\r\n", &R_DMA4_CTRL, R_DMA4_CTRL);    // 0xD0300200
        DBG_PRINT("R_DMA4_SRC_ADDR[%08x]=%08x\r\n", &R_DMA4_SRC_ADDR, R_DMA4_SRC_ADDR);    // 0xD0300204
        DBG_PRINT("R_DMA4_TAR_ADDR[%08x]=%08x\r\n", &R_DMA4_TAR_ADDR, R_DMA4_TAR_ADDR);    // 0xD0300208
        DBG_PRINT("R_DMA4_TX_COUNT[%08x]=%08x\r\n", &R_DMA4_TX_COUNT, R_DMA4_TX_COUNT);    // 0xD030020C
        DBG_PRINT("R_DMA4_SPRITE_SIZE[%08x]=%08x\r\n", &R_DMA4_SPRITE_SIZE, R_DMA4_SPRITE_SIZE);    // 0xD0300210
        DBG_PRINT("R_DMA4_TRANSPARENT[%08x]=%08x\r\n", &R_DMA4_TRANSPARENT, R_DMA4_TRANSPARENT);    // 0xD0300214
        DBG_PRINT("R_DMA4_MISC[%08x]=%08x\r\n", &R_DMA4_MISC, R_DMA4_MISC);    // 0xD0300218
    }
*/
	return 0;
}

/**
 * @brief   transfer data by dma
 * @param   dma_struct: dma struct
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer(DMA_STRUCT *dma_struct)
{
	if (!dma_struct) {
		return -1;
	}
	return dma_transfer_extend(dma_struct, C_DMA_NORMAL_USED, (INT32U) NULL, NULL);
}

void drv_l1_dma_enforce_timeout(INT32U channel)
{
	dma_irq_handler(dma_get_int_flag(channel) | dma_get_timeout_flag(channel));
}

#define K_SD_READ_TIMEOUT_PER_SECTOR	105 // ms unit, should be 100, add more 5 for safe.
											// although sd read spec timeout per sector is 100ms, use 105ms, because 326XXB hardware is 104.5 ms
#define K_SD_WRITE_TIMEOUT_PER_SECTOR   250

/**
 * @brief   dma transfer data and wait transfer finish
 * @param   dma_struct: dma struct
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_wait_ready(DMA_STRUCT *dma_struct)
{
	volatile INT8S notify;
  	INT32U q_index;
  	INT32S mask;
	osEvent result;

	if (!dma_struct) {
		return -1;
	}
	if (!(dma_struct->notify)) {
		dma_struct->notify = &notify;
	}

	*(dma_struct->notify) = C_DMA_STATUS_WAITING;
	q_index = dma_get_queue();
	if (q_index >= C_DMA_Q_NUM) {
		return -1;
	}
  	if (dma_transfer_extend(dma_struct, C_DMA_NORMAL_USED, (INT32U) dma_driver_queue[q_index], NULL)) {
  		if (dma_struct->notify == &notify) {		// Restore dma_struct->notify
  			dma_struct->notify = NULL;
  		}
  		dma_free_queue(q_index);

		return -1;
	}

    INT32U msecwait = 10000;

	#if 0
  	result = osMessageGet(dma_driver_queue[q_index], osWaitForever);
	#else
	// williamyeo, to avoid loop forever, set timeout when using osMessageGet
  	{
		static INT16U sd_read_over_spec_sector_timeout[2] = {0, 0};
		INT8U sd_access_type = dma_is_sd[dma_struct->channel] & 0x7f; // 1 is sd read, 2 is sd write
		INT8U sd_device_id;
		INT32U sd_spec_sector_timeout; // in ms unit
        INT32U dma_first_count;
        INT32U sd_nr_sector;

		if (sd_access_type)
		{
            dma_get_tx_count(dma_struct->channel, &dma_first_count);
            if (dma_struct->width == DMA_DATA_WIDTH_2BYTE)
                dma_first_count = (dma_first_count << 1);
            else if (dma_struct->width == DMA_DATA_WIDTH_4BYTE)
                dma_first_count = (dma_first_count << 2);

			sd_device_id = ((dma_is_sd[dma_struct->channel] & 0x80) == 0x80) ? 0 : 1;
			sd_spec_sector_timeout = (sd_access_type == 1) ? K_SD_READ_TIMEOUT_PER_SECTOR : K_SD_WRITE_TIMEOUT_PER_SECTOR;
			if (sd_access_type == 1) // read
			{
				if (sd_read_over_spec_sector_timeout[sd_device_id] != 0)
					sd_spec_sector_timeout = sd_read_over_spec_sector_timeout[sd_device_id];
			}
		}

        if (dma_struct->timeout == 0)
		{
			if (sd_access_type)
			{
				// set dma timeout for sd read and sd write

				if (dma_struct->width == DMA_DATA_WIDTH_1BYTE)
					sd_nr_sector = (dma_struct->count + 511) >> 9;
				else if (dma_struct->width == DMA_DATA_WIDTH_2BYTE)
					sd_nr_sector = ((dma_struct->count << 1) + 511) >> 9;
				else
					sd_nr_sector = ((dma_struct->count << 2) + 511) >> 9;

				if (sd_access_type == 1)
				{
					// double the timeout, because discover burn out sd card read time will up to 168ms per sector
					if (sd_read_over_spec_sector_timeout[sd_device_id] != 0)
						msecwait = sd_nr_sector * sd_spec_sector_timeout;
					else
						msecwait = sd_nr_sector*K_SD_READ_TIMEOUT_PER_SECTOR*2;
				}
				else
					msecwait = (sd_nr_sector * sd_spec_sector_timeout) + K_SD_WRITE_TIMEOUT_PER_SECTOR + 615;

				// this is to adjust precision of msec cause by os tick
				msecwait = (msecwait * 1016) / 1000;
                //DBG_PRINT("sd dma timeout %u, count %u, w %u, dma_is_sd[%u]=%u\r\n", msecwait, dma_struct->count, dma_struct->width, dma_struct->channel, dma_is_sd[dma_struct->channel]);
			}
			else
				msecwait = osWaitForever;
		}
		else
            msecwait = ((((INT32U)dma_struct->timeout) << 2)*1016)/1000;

        if (sd_access_type)
        {
			// check sd read timeout 9 bit early
			INT32U sd_status = 0;
            INT32U slicemswait;
            INT32U mswait;
			INT32U accmswait = 0;
            INT32U previous_accmswait = 0;
            INT32U dmamswait;
            INT32U sdc_timeout_ms = (((INT64U)10485760000)/MCLK)+ 3;
            INT16U slice_idx = 0;

            dmamswait = (sd_access_type == 1) ? (K_SD_READ_TIMEOUT_PER_SECTOR*2) : 615;
            mswait = msecwait;
            slicemswait = sdc_timeout_ms;
            if (slicemswait > 200)
                slicemswait = 200;

            while (mswait != 0)
            {
                // check sd read bit 9 status timeout
                sd_status = (sd_device_id == 0) ? R_SDC0_STATUS : R_SDC1_STATUS;

                if ((sd_status & (1<<9)) != 0)
                {
					// trigger dummy clock to allow sd complete it action
                    drvl1_sdc_dummyclk_en(sd_device_id, 1);
                    drv_l1_dma_enforce_timeout((INT32U)dma_struct->channel);
                    result = osMessageGet(dma_driver_queue[q_index], 100);
					if (sd_spec_sector_timeout > accmswait)
					{
                        if (drvl1_sdc_check_card_present(sd_device_id))
                            drv_msec_wait(sd_spec_sector_timeout - accmswait);
                    }
					drvl1_sdc_dummyclk_en(sd_device_id, 0); // disable sd clock

					if (MCLK >= 81000000)
                        sd_read_over_spec_sector_timeout[sd_device_id] = 135;
					else
						sd_read_over_spec_sector_timeout[sd_device_id] = 0;
					//DBG_PRINT("SD STATUS BIT9 ON!!! %u, %u\r\n", sd_spec_sector_timeout, sd_read_over_spec_sector_timeout[sd_device_id]);
                    if (sd_spec_sector_timeout == K_SD_READ_TIMEOUT_PER_SECTOR)
                        DBG_PRINT("9");
                    else
                        DBG_PRINT("6");
                    break;
                }
                else
                    sd_read_over_spec_sector_timeout[sd_device_id] = 0;

                if (slicemswait > mswait)
                    slicemswait = mswait;
                result = osMessageGet(dma_driver_queue[q_index], slicemswait);
				accmswait += slicemswait;
                mswait -= slicemswait;
                if (mswait == 0)
                {
                    break;
                }
                if (result.status != osEventTimeout)
                    break;

                if ((accmswait - previous_accmswait) >= dmamswait)
                {
                    INT32U dma_second_count;

                    dma_get_tx_count(dma_struct->channel, &dma_second_count);
                     if (dma_struct->width == DMA_DATA_WIDTH_2BYTE)
                        dma_second_count = (dma_second_count << 1);
                    else if (dma_struct->width == DMA_DATA_WIDTH_4BYTE)
                        dma_second_count = (dma_second_count << 2);
                    if ((dma_second_count != 0) && (dma_second_count == dma_first_count))
                    {
                        drv_l1_dma_enforce_timeout((INT32U)dma_struct->channel);
                        DBG_PRINT("1 sd-%u dma left count is same after a while. %u, %u, %u, %u, 0x%08x, %u\r\n", sd_device_id, dma_second_count, dmamswait, sd_access_type, accmswait, sd_status, sd_nr_sector);
                        result = osMessageGet(dma_driver_queue[q_index], 100);
                        drvl1_sdc_clear_timeout_bit(sd_device_id, 10); // this is not timeout bit bug, so clear it
                        break;
                    }
                    dma_first_count = dma_second_count;
                    previous_accmswait = accmswait;
                }

                if (drvl1_sdc_check_card_present(sd_device_id) == 0)
                {
                    drv_l1_dma_enforce_timeout((INT32U)dma_struct->channel);
                    result = osMessageGet(dma_driver_queue[q_index], 100);
                    break;
                }

                slice_idx++;
                if (slice_idx == 1)
                    slicemswait = 2;
                else if (slice_idx == 2)
                    slicemswait = 2;
                else
                    slicemswait = 100;
            } // end while
        }
        else
            result = osMessageGet(dma_driver_queue[q_index], msecwait);
    }
	#endif

    if (result.status != osEventMessage) // osEventTimeout
    {
        DBG_PRINT("dma result.status = 0x%x timeout=%u msecwait=%u count=%u w=%u\r\n", result.status, dma_struct->timeout, msecwait, dma_struct->count, dma_struct->width);
    }

	if (result.status == osEventMessage) {
  		if (dma_struct->notify == &notify) {		// Restore dma_struct->notify
  			dma_struct->notify = NULL;
  		}

		if (result.value.v == C_DMA_STATUS_DONE) {
			dma_free_queue(q_index);
			return 0;
		}

		if (result.value.v == C_DMA_STATUS_TIMEOUT) {
			dma_free_queue(q_index);
			return -1;
		}
		//return -1;		// DMA timeout // williamyeo remark this line, let it go below code to reset dma when error
  	}

	#if (K_DMA_TIMEOUT_PATCH_APPLY == 1)
	drv_l1_dma_enforce_timeout((INT32U)dma_struct->channel);
	result = osMessageGet(dma_driver_queue[q_index], msecwait);
	#else
	// If we don't receive response from DMA, we have to reset DMA controller and free the channel by ourselves
	mask = dma_device_protect();
	dma_free_channel((INT32U) dma_struct->channel);
	dma_device_unprotect(mask);
	#endif

	dma_free_queue(q_index);

	if (*(dma_struct->notify) == C_DMA_STATUS_DONE) {
  		if (dma_struct->notify == &notify) {		// Restore dma_struct->notify
  			dma_struct->notify = NULL;
  		}

		return 0;
	}

	if (dma_struct->notify == &notify) {		// Restore dma_struct->notify
		dma_struct->notify = NULL;
	}

	return -1;								// DMA timeout
}

#if 1
/**
 * @brief   dma transfer data with queue
 * @param   dma_struct: dma struct
 * @param   os_q: notify queue
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_with_queue(DMA_STRUCT *dma_struct, osMessageQId os_q)
{
	if (!dma_struct || !os_q) {
		return -1;
	}
	return dma_transfer_extend(dma_struct, C_DMA_NORMAL_USED, (INT32U) os_q, NULL);
}

/**
 * @brief   dma transfer data with callback
 * @param   dma_struct: dma struct
 * @param   *dma_user_isr: function pointer of dma user callback isr
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_with_callback(DMA_STRUCT *dma_struct, void (*dma_user_isr)(INT8U, INT8U))
{
	if (!dma_struct /*|| !os_q*/) {
		return -1;
	}
	return dma_transfer_extend(dma_struct, C_DMA_NORMAL_USED, (INT32U) NULL, dma_user_isr);
}

/**
 * @brief   dma transfer data with callback
 * @param   dma_struct: dma struct
 * @param   *dma_user_isr: function pointer of dma user callback isr
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_with_queue_n_callback(DMA_STRUCT *dma_struct, osMessageQId os_q, void (*dma_user_isr)(INT8U, INT8U))
{
	if (!dma_struct || !os_q) {
		return -1;
	}
	return dma_transfer_extend(dma_struct, C_DMA_NORMAL_USED, (INT32U) os_q, dma_user_isr);
}

/**
 * @brief   dma transfer data with queue in double buffer mode.
 * @param   dma_struct: dma struct
 * @param   os_q: notify queue
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_with_double_buf(DMA_STRUCT *dma_struct, osMessageQId os_q)
{
	if (!dma_struct /*|| !os_q*/) {
		return -1;
	}
	return dma_transfer_extend(dma_struct, C_DMA_OCCUPIED, (INT32U) os_q, NULL);
}
/**
 * @brief   dma transfer data with queue and callback in double buffer mode.
 * @param   dma_struct: dma struct
 * @param   os_q: notify queue
 * @param   *dma_user_isr: function pointer of dma user callback isr
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_double_buf_with_callback(DMA_STRUCT *dma_struct, osMessageQId os_q, void (*dma_user_isr)(INT8U, INT8U))
{
	if (!dma_struct /*|| !os_q*/) {
		return -1;
	}
	return dma_transfer_extend(dma_struct, C_DMA_OCCUPIED, (INT32U) os_q, dma_user_isr);
}
#else
/**
 * @brief   dma transfer data in double buffer mode.
 * @param   dma_struct: dma struct
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_with_double_buf(DMA_STRUCT *dma_struct)
{
	if (!dma_struct) {
		return -1;
	}
	return dma_transfer_extend(dma_struct, C_DMA_OCCUPIED, NULL);
}
#endif

/**
 * @brief   dma transfer next data in double buffer mode.
 * @param   dma_struct: dma struct
 * @param   os_q: notify queue
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_double_buf_set(DMA_STRUCT *dma_struct)
{
  	INT32S mask;
    INT32U s_addr = dma_struct->s_addr;
	INT32U t_addr = dma_struct->t_addr;
	INT32U count = dma_struct->count;
    INT32U len;
    INT16U src_type, target_type;

	if (!dma_struct || dma_struct->channel>=C_DMA_Q_NUM) {
		return -1;
	}
	if (dma_usage[dma_struct->channel] != C_DMA_OCCUPIED) {
		return -1;
	}
#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
    if (dma_struct->width == DMA_DATA_WIDTH_1BYTE)
        len = count;
    else if (dma_struct->width == DMA_DATA_WIDTH_2BYTE)
        len = count << 1;
    else if (dma_struct->width == DMA_DATA_WIDTH_4BYTE)
        len = count << 2;
    else
        return -1;
    if ((s_addr<C_DMA_IO_ADDR_START) || (s_addr>C_DMA_IO_ADDR_END))
        src_type = C_DMA_MEMORY;
    else
        src_type = C_DMA_IO_I2S_RX; // here, purpose is find out memort or IO, so just set any IO is ok
	if ((t_addr<C_DMA_IO_ADDR_START) || (t_addr>C_DMA_IO_ADDR_END))
		target_type = C_DMA_MEMORY;
    else
        target_type = C_DMA_IO_I2S_TX; // here, purpose is find out memort or IO, so just set any IO is ok

    if (src_type != C_DMA_MEMORY) 		// IO to memory
        cache_invalid_range(t_addr, len);
    else if (target_type != C_DMA_MEMORY)
        cache_drain_range(s_addr, len);
    else {
        cache_drain_range(s_addr, len);
		cache_invalid_range(t_addr, len);
    }
#endif

	dma_set_source((INT32U) dma_struct->channel, dma_struct->s_addr);
	dma_set_target((INT32U) dma_struct->channel, dma_struct->t_addr);

	if (dma_set_tx_count((INT32U) dma_struct->channel, dma_struct->count)) {
		mask = dma_device_protect();

		dma_free_channel((INT32U) dma_struct->channel);

		dma_device_unprotect(mask);

		return -1;
	}

	return 0;
}

/**
 * @brief   dma free channel in double buffer mode.
 * @param   dma_struct: dma struct
 * @param   os_q: notify queue
 * @return 	result: >=0 is success, <0 is fail.
 */
INT32S drv_l1_dma_transfer_double_buf_free(DMA_STRUCT *dma_struct)
{
  	INT32S mask;

	if (!dma_struct) {
		return -1;
	}
	if (dma_usage[dma_struct->channel] != C_DMA_OCCUPIED) {
		return -1;
	}

	mask = dma_device_protect();

	dma_free_channel((INT32U) dma_struct->channel);

	dma_device_unprotect(mask);

	return 0;
}

/**
 * @brief   Check double buffer full flag.
 * @param   channel: channel number, C_DMA_CH0 ~ C_DMA_CH7
 * @return 	1: Full, 0: Not full
 */
INT32S drv_l1_dma_dbf_status_get(INT8U channel)
{
	INT32U ctrl;

	if (channel == C_DMA_CH0) {
		ctrl = R_DMA0_CTRL;
	} else if (channel == C_DMA_CH1) {
		ctrl = R_DMA1_CTRL;
	} else if (channel == C_DMA_CH2) {
		ctrl = R_DMA2_CTRL;
	} else if (channel == C_DMA_CH3) {
		ctrl = R_DMA3_CTRL;
	} else if (channel == C_DMA_CH4) {
		ctrl = R_DMA4_CTRL;
	} else if (channel == C_DMA_CH5) {
		ctrl = R_DMA5_CTRL;
	} else if (channel == C_DMA_CH6) {
		ctrl = R_DMA6_CTRL;
	} else if (channel == C_DMA_CH7) {
		ctrl = R_DMA7_CTRL;
	} else {
		ctrl = 0;
	}

	if (ctrl & C_DMA_CTRL_DBF) {
		return 1;
	}
	return 0;
}

/**
 * @brief   Check DMA channel status
 * @param   channel: channel number, C_DMA_CH0 ~ C_DMA_CH7
 * @return 	1: Busy, 0: Idle
 */
INT32S drv_l1_dma_status_get(INT8U channel)
{
	INT32U ctrl;

	if (channel == C_DMA_CH0) {
		ctrl = R_DMA0_CTRL;
	} else if (channel == C_DMA_CH1) {
		ctrl = R_DMA1_CTRL;
	} else if (channel == C_DMA_CH2) {
		ctrl = R_DMA2_CTRL;
	} else if (channel == C_DMA_CH3) {
		ctrl = R_DMA3_CTRL;
	} else if (channel == C_DMA_CH4) {
		ctrl = R_DMA4_CTRL;
	} else if (channel == C_DMA_CH5) {
		ctrl = R_DMA5_CTRL;
	} else if (channel == C_DMA_CH6) {
		ctrl = R_DMA6_CTRL;
	} else if (channel == C_DMA_CH7) {
		ctrl = R_DMA7_CTRL;
	} else {
		ctrl = 0;
	}

	if (ctrl & C_DMA_CTRL_BUSY) {
		return 1;
	}
	return 0;
}

/**
 * @brief   Fill memory by DMA
 * @param   t_addr: memory address
 * @param   value: filled value
 * @param   byte_count: data length in byte
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_dma_memory_fill(INT32U t_addr, INT8U value, INT32U byte_count)
{
	INT8U *p8;
	INT32U *p32;
	INT32U ctrl, channel;
	INT32U src_value;
	INT32S ret;
  	INT32U q_index;
	osEvent result;
  	INT32S mask;
  	volatile INT8S notify;

	if (!dma_init_done) {
		drv_l1_dma_init();
	}

	// If length is less than 16 bytes, uses CPU to set the memory directly
	if (byte_count < 16) {
		p8 = (INT8U *) t_addr;
		while (byte_count--) {
			*p8++ = value;
		}

		return 0;
	}

	// Make sure start address is 4-byte alignment
	while (t_addr & 0x3) {
		*((INT8U *) t_addr++) = value;
		byte_count--;
	}
	// Make sure end address is on 4-byte boundry
	while (byte_count & 0x3) {
		*((INT8U *) t_addr+byte_count-1) = value;
		byte_count--;
	}

	src_value = (value<<24) | (value<<16) | (value<<8) | value;

	// If left length is less than 128 bytes, uses CPU to set the memory directly
	if (byte_count < 128) {
		p32 = (INT32U *) t_addr;
		while (byte_count) {
			*p32++ = src_value;
			byte_count -= 4;
		}
		return 0;
	}

	// Create a queue to receive DMA result
	q_index = dma_get_queue();
	if (q_index >= C_DMA_Q_NUM) {
		return -1;
	}

	// Get a free DMA channel
	channel = dma_get_channel(C_DMA_NORMAL_USED);
	if  (channel >= C_DMA_CHANNEL_NUM) {
	  	dma_free_queue(q_index);
		return -1;							// No free DMA channel is available
	}

	// Set transmit counter
	ret = dma_set_tx_count(channel, byte_count>>2);

	// Disable timeout function
	if (dma_set_timeout(channel, 0)) {
		ret = -1;
	}

	// Disable skip function
	if (dma_set_sprite_size(channel, 0)) {
		ret = -1;
	}

	// Return if any error occurs
	if (ret) {
	  	dma_free_queue(q_index);
		dma_free_channel(channel);
		return -1;
	}

	// Set DMA status to waiting transmit
	notify = C_DMA_STATUS_WAITING;
	dma_set_notify(channel, (INT32U) &notify, (INT32U) dma_driver_queue[q_index]);

	// Set source address
	dma_set_source(channel, (INT32U) &src_value);

	// Set target address
	dma_set_target(channel, t_addr);

#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
	// Drain source memory and invalid target memory from cache
	cache_drain_range((INT32U) &src_value, 4);
	cache_invalid_range(t_addr, byte_count);
#endif

	// Prepare control register
	if (!((byte_count>>2) & 0x7)) {
		ctrl = C_DMA_CTRL_BURST8_ACCESS | C_DMA_CTRL_SINGLE_TRANS | C_DMA_CTRL_32BIT | C_DMA_CTRL_M2M | C_DMA_CTRL_INT | C_DMA_CTRL_SRC_FIX | C_DMA_CTRL_DEST_INCREASE | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_SOFTWARE | C_DMA_CTRL_ENABLE;
	} else if (!((byte_count>>2) & 0x3)) {
		ctrl = C_DMA_CTRL_BURST4_ACCESS | C_DMA_CTRL_SINGLE_TRANS | C_DMA_CTRL_32BIT | C_DMA_CTRL_M2M | C_DMA_CTRL_INT | C_DMA_CTRL_SRC_FIX | C_DMA_CTRL_DEST_INCREASE | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_SOFTWARE | C_DMA_CTRL_ENABLE;
	} else {
		ctrl = C_DMA_CTRL_SINGLE_ACCESS | C_DMA_CTRL_SINGLE_TRANS | C_DMA_CTRL_32BIT | C_DMA_CTRL_M2M | C_DMA_CTRL_INT | C_DMA_CTRL_SRC_FIX | C_DMA_CTRL_DEST_INCREASE | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_SOFTWARE | C_DMA_CTRL_ENABLE;
	}
	// Start DMA now
	#if (K_DMA_TIMEOUT_PATCH_APPLY == 1)
	dma_previous_done_state[channel] = C_DMA_STATUS_WAITING;
	#endif
	dma_set_control(channel, ctrl);

	// Wait until DMA finish transmitting or timeout
	result = osMessageGet(dma_driver_queue[q_index], osWaitForever);
    if((result.status != osEventMessage) || (result.value.v != C_DMA_STATUS_DONE)) {
        return -1;
    }

	dma_free_queue(q_index);

	// If we don't receive response from DMA, we have to reset DMA controller and free the channel by ourselves
	//mask = dma_device_protect();
	//dma_free_channel(channel);
	//dma_device_unprotect(mask);

	if (notify == C_DMA_STATUS_DONE) {
		return 0;
	}
	return -1;

	//while (notify == C_DMA_STATUS_WAITING) ;
	//return 0;					// DMA timeout function is not enabled, DMA transfer must be ok to reach here
}

/**
 * @brief   Copy a section of data
 * @param   s_addr: Source address
 * @param   t_addr: Destination address
 * @param   byte_count: Data number needed to be copied
 * @param   s_width: Buffer width of source buffer
 * @param   t_width: Buffer width of destination buffer
 * @param   dir:dma direction
 * @param   mode:Prepare control mod
 * @return 	0: Success, -1: Fail
 */
INT32S dma_buffer_copy_extend(INT32U s_addr, INT32U t_addr, INT32U byte_count, INT32U s_width, INT32U t_width, INT32U dir, INT32U mode)
{
	INT32U ctrl, channel;
	INT32S ret;
	INT32U q_index;
	osEvent result;
	INT32S mask;
  	volatile INT8S notify;

	if (!dma_init_done) {
		drv_l1_dma_init();
	}

	// Make sure address and size are 2-byte alignment
	if ((s_addr & 0x1) || (t_addr & 0x1) || (byte_count & 0x1) || (s_width & 0x1) || (t_width & 0x1) || (s_width>t_width)) {
		return -1;
	}

  	// Create a queue to receive DMA result
	q_index = dma_get_queue();
	if (q_index >= C_DMA_Q_NUM) {
		return -1;
	}

	// Get a free DMA channel
	channel = dma_get_channel(C_DMA_NORMAL_USED);
	if  (channel >= C_DMA_CHANNEL_NUM) {
	  	dma_free_queue(q_index);
		return -1;							// No free DMA channel is available
	}

	// Set transmit counter
	ret = dma_set_tx_count(channel, byte_count>>1);

	// Disable timeout function
	if (dma_set_timeout(channel, 0)) {
		ret = -1;
	}

	dma_set_direction(channel, dir);

	// Set destination buffer width (1 means 2 bytes)
	if (dma_set_line_length(t_width>>1)) {
		ret = -1;
	}

	// Set source buffer width (1 means 2 bytes)
	if (dma_set_sprite_size(channel, s_width>>1)) {
		ret = -1;
	}

	// Return if any error occurs
	if (ret) {
	  	dma_free_queue(q_index);
		dma_free_channel(channel);

		return -1;
	}

	// Set DMA status to waiting transmit
	notify = C_DMA_STATUS_WAITING;
	dma_set_notify(channel, (INT32U) &notify, (INT32U) dma_driver_queue[q_index]);

	// Set source address
	dma_set_source(channel, s_addr);

	// Set target address
	dma_set_target(channel, t_addr);

#if (defined _DRV_L1_CACHE) && (_DRV_L1_CACHE == 1)
	// Drain source memory and invalid target memory from cache
	cache_drain_range(s_addr, byte_count);
	cache_invalid_range(t_addr, (byte_count/s_width)*t_width);
#endif

	// Prepare control register, burst is not supported in sprite copy mode
	ctrl = mode | C_DMA_CTRL_SINGLE_TRANS | C_DMA_CTRL_16BIT | C_DMA_CTRL_M2M | C_DMA_CTRL_INT | C_DMA_CTRL_SRC_INCREASE | C_DMA_CTRL_DEST_INCREASE | C_DMA_CTRL_NORMAL_INT | C_DMA_CTRL_SOFTWARE | C_DMA_CTRL_ENABLE;
	// Start DMA now
	#if (K_DMA_TIMEOUT_PATCH_APPLY == 1)
	dma_previous_done_state[channel] = C_DMA_STATUS_WAITING;
	#endif
	dma_set_control(channel, ctrl);

	// Wait until DMA finish transmitting or timeout
	result = osMessageGet(dma_driver_queue[q_index], osWaitForever);
  	if((result.status != osEventMessage) || (result.value.v != C_DMA_STATUS_DONE)) {
        return -1;
    }

    dma_free_queue(q_index);

	// If we don't receive response from DMA, we have to reset DMA controller and free the channel by ourselves
	//mask = dma_device_protect();
	//dma_free_channel(channel);
	//dma_device_unprotect(mask);

	if (notify == C_DMA_STATUS_DONE) {
		return 0;
	}
	return -1;

	//while (notify == C_DMA_STATUS_WAITING) ;

	//return 0;					// DMA timeout function is not enabled, DMA transfer must be ok to reach here
}

/**
 * @brief   Copy a section of data
 * @param   s_addr: Source address
 * @param   t_addr: Destination address
 * @param   byte_count: Data number needed to be copied
 * @param   s_width: Buffer width of source buffer
 * @param   t_width: Buffer width of destination buffer
 			All parameters must be multiple of 2,
 			s_width(source buffer width) must <= t_width(target buffer width)
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l1_dma_buffer_copy(INT32U s_addr, INT32U t_addr, INT32U byte_count, INT32U s_width, INT32U t_width)
{
	return dma_buffer_copy_extend(s_addr, t_addr, byte_count, s_width, t_width, C_DMA_MISC_SPRITE_TO_FB, C_DMA_CTRL_SINGLE_ACCESS);
}

/**
 * @brief   Copy memory data
 * @param   dest: Destination address
 * @param   src: Source address
 * @param   len: Data number needed to be copied
 * @return 	0: Success, -1: Fail
 */
INT32S dma_transfer_data(INT32U dest, INT32U src, INT32U len)
{
#if 1
	R_DMA0_CTRL	= C_DMA_CTRL_RESET; /* reset dma */
	R_DMA_INT = R_DMA_INT; /* clear interrupt flag */

	R_DMA0_MISC &= ~(C_DMA_MISC_TIMEOUT_MASK); /* set time-out value */
	R_DMA0_MISC |= 0xFF << C_DMA_MISC_TIMEOUT_SHIFT; /* 1 sec */

	/* set DMA 0 for transfering data */
	R_DMA0_SRC_ADDR = (INT32U)src;
	R_DMA0_TAR_ADDR = (INT32U)dest;
	R_DMA0_TX_COUNT = len>>1;
	R_DMA0_CTRL     = C_DMA_CTRL_SINGLE_ACCESS|C_DMA_CTRL_SINGLE_TRANS|C_DMA_CTRL_SRC_INCREASE|C_DMA_CTRL_DEST_INCREASE
	                 |C_DMA_CTRL_SOFTWARE|C_DMA_CTRL_16BIT|C_DMA_CTRL_INT|C_DMA_CTRL_NORMAL_INT|C_DMA_CTRL_ENABLE;


	while((R_DMA_INT&C_DMA0_INT_PEND) == 0);
	if (R_DMA_INT & C_DMA0_TIMEOUT) {
		return C_DMA0_TIMEOUT;
	}
	R_DMA_INT = C_DMA0_INT_PEND;

	return STATUS_OK;
#else
	memcpy((void *)dest, (void *)src, len);
	return STATUS_OK;
#endif
}

INT32S dma_data_memcpy(INT8U* s1, const INT8U* s2, INT32U n)
{
	DMA_STRUCT dma;
	INT8U *ss, *dd;
	INT32U s_offset = 0, d_offset = 0, remain = 0;
	INT32U i, w;

	if (n < 1024)
	{
		gp_memcpy(s1, s2, n);
		return 0;
	}

	ss = (INT8U*)s2;
	dd = (INT8U*)s1;

	//4 byte alignment
	w = 4;
	s_offset = (INT32U)s2 & 0x3;
	d_offset = (INT32U)s1 & 0x3;

	//2 byte alignment
	if (s_offset != d_offset)
	{
		w = 2;
		s_offset = (INT32U)s2 & 0x1;
		d_offset = (INT32U)s1 & 0x1;
	}

	//single byte width
	if (s_offset != d_offset)
	{
		w = 1;
		s_offset = d_offset = 0;
	}

	if (s_offset)
	{
		ss += (w-s_offset);
		dd += (w-s_offset);
		for (i = 0; i <(w-s_offset); i++)
			*((INT8U*)s1+i) = *((INT8U*)s2+i);
		n -= (w-s_offset);
	}

	remain = n & (w-1);
	i = w/2;
	n = n >> i;

	if (n & 0x7) // count should be 8 alignment for max peformence
	{
		remain += (n&0x7)*w;
		n = n >> 3 << 3;
	}

	gp_memset(&dma, 0, sizeof(DMA_STRUCT));
	dma.s_addr = (INT32U)ss;
	dma.t_addr = (INT32U)dd;
	dma.count = n;
	dma.width = w;
	dma.timeout = 255;

	if (0 > drv_l1_dma_transfer_wait_ready(&dma))
	{
		DBG_PRINT("dma copy fail\r\n");
		return -1;
	}

	if (remain)
	{
		n = n << i;
		gp_memcpy(dd+n, ss+n, remain);
	}

	return 0;
}
#endif
