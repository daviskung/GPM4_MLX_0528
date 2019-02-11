#ifndef __DRV_L1_SDC_H__
#define __DRV_L1_SDC_H__

#include "drv_l1.h"
#include "drv_l1_dma.h"
#include "drv_l1_sfr.h"

typedef struct sdReg_s {
	volatile INT32U DataTx;		/* Data Transmit register */
	volatile INT32U DataRx;		/* Data Receive register */
	volatile INT32U Cmd;		/* Command register */
	volatile INT32U Arg;		/* Argument register */
	volatile INT32U Resp;		/* Response register */
	volatile INT32U Status;		/* Controller status register */
	volatile INT32U Ctrl;		/* Control register */
	volatile INT32U IntEn;		/* Interrupt enable register */
} sdReg_t;
#if 1 || (defined MCU_VERSION) && (MCU_VERSION >= GPL326XXB)	&& (MCU_VERSION < GPL327XX)
	#define SetSDBase(x) ((x==0)?P_SDC0_BASE : P_SDC1_BASE)
#else
	#define SetSDBase(x) (P_SDC0_BASE)
#endif

typedef struct sd_l1_data_s{
	INT32U *buf;				/*!< @brief Data buffer pointer. */
	INT32U blksz;				/*!< @brief Data block size. */
	INT32U blocks; 				/*!< @brief Number of blocks. */
	INT16U dir; 				/*!< @brief Data direction. */
	INT16U dmaen;				/*!< @brief DMA enable. */
	INT32U bytes_xfered;		/*!< @brief Data transfered in byte. */
	volatile INT8S dma_notify;	/*!< @brief Used for checking dam finish. */
#if _OPERATING_SYSTEM != _OS_NONE
	void* dma_q_buf;
    #if _OPERATING_SYSTEM == _OS_UCOS2
	OS_EVENT *dma_q;
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
        xQueueHandle dma_q;
    #endif
#endif
}sd_l1_data_t;

typedef struct sd_l1_ctrl_t{
	INT8U device_id;		/*!< @brief Device ID. */
	INT8U cmd;				/*!< @brief Command. */
	INT8U resp_type;		/*!< @brief Response type. */
	INT8U data_separate;	/*!< @brief Data transfer separate. If this flag is set, data would not transfer when sent command */
	INT32U arg;				/*!< @brief Argument. */
	INT32U resp[4];			/*!< @brief Response buffer. */
	sd_l1_data_t *data;		/*!< @brief Transfer data information. */
}sd_l1_ctrl_t;

typedef INT8U(*sd_card_present_check_callback)(INT32U);

// SDC control register
#define C_SDC_CTL_CLKDIV				((10*MHZ - 8) / 8)
#define	C_SDC_CTL_BUS_4BIT				0x00000100
#define	C_SDC_CTL_DMA_ENABLE	  		0x00000200
#define	C_SDC_CTL_IO_INT				0x00000400
#define	C_SDC_CTL_ENABLE				0x00000800
#define	C_SDC_CTL_DUMMYCLK				0x00001000				//This bit only work on GPL326XXB
#define	C_SDC_CTL_BLK8			  		0x00080000
#define	C_SDC_CTL_BLK256				0x01000000
#define	C_SDC_CTL_BLK512				0x02000000
#define	C_SDC_EXT_WAIT_ENABLE			0x40000000              // Extend the NCR(command then response) wait cycles from 6x to 2xx cycles

// SDC command register
#define	C_SDC_CMD_STOP					0x00000040
#define	C_SDC_CMD_RUN					0x00000080
#define C_SDC_CMD_WITH_DATA		  		0x00000100
#define C_SDC_CMD_WRITE_DATA			0x00000200
#define C_SDC_CMD_MULTI_BLOCK	  		0x00000400
#define C_SDC_CMD_INIT_CARD	 			0x00000800
#define C_SDC_CMD_RESP_R0				0x00000000
#define C_SDC_CMD_RESP_R1				0x00001000
#define C_SDC_CMD_RESP_R2				0x00002000
#define C_SDC_CMD_RESP_R3				0x00003000
#define C_SDC_CMD_RESP_R6				0x00006000
#define C_SDC_CMD_RESP_R7  				0x00001000
#define C_SDC_CMD_RESP_R1B		  		0x00007000

// SDC status register
#define C_SDC_STATUS_CONTROLLER_BUSY	0x00000001
#define C_SDC_STATUS_CARD_BUSY			0x00000002
#define C_SDC_STATUS_CMD_COMPLETE		0x00000004
#define C_SDC_STATUS_DATA_COMPLETE		0x00000008
#define C_SDC_STATUS_RESP_INDEX_ERR		0x00000010
#define C_SDC_STATUS_RESP_CRC_ERR		0x00000020
#define C_SDC_STATUS_RESP_REG_FULL		0x00000040
#define C_SDC_STATUS_DATA_BUF_FULL		0x00000080
#define C_SDC_STATUS_DATA_BUF_EMPTY		0x00000100
#define C_SDC_STATUS_TIMEOUT			0x00000200
#define C_SDC_STATUS_DATA_CRC_ERR		0x00000400
#define C_SDC_STATUS_CARD_PROTECT		0x00000800
#define C_SDC_STATUS_CARD_PRESENT	  	0x00001000
#define	C_SDC_STATUS_SDIO_INT_PEND		0x00002000
#define C_SDC_STATUS_CLEAR_ALL			0x0000FFFF

// SDC interrupt register
#define	C_SDC_INT_CMD_COMP				0x00000001
#define	C_SDC_INT_DATA_COMP				0x00000002
#define	C_SDC_INT_RBUF_FULL				0x00000004
#define	C_SDC_INT_DBUF_FULL				0x00000008
#define	C_SDC_INT_DBUF_EMPT				0x00000010
#define	C_SDC_INT_PRESENT				0x00000020
#define	C_SDC_INT_IO_INT				0x00000040

#define SDC_RESPTYPE_NONE 	0		/*!< @brief SD response r0. */
#define SDC_RESPTYPE_R1		1		/*!< @brief SD response r1. */
#define SDC_RESPTYPE_R2		2		/*!< @brief SD response r2. */
#define SDC_RESPTYPE_R3		3		/*!< @brief SD response r3. */
#define SDC_RESPTYPE_R4		4		/*!< @brief SD response r4. */
#define SDC_RESPTYPE_R5		5		/*!< @brief SD response r5. */
#define SDC_RESPTYPE_R6		6		/*!< @brief SD response r6. */
#define SDC_RESPTYPE_R7		7		/*!< @brief SD response r7. */
#define SDC_RESPTYPE_R1b	8		/*!< @brief SD response r1b. */

#define SDC_READ			0		/*!< @brief SD data direction: read. */
#define SDC_WRITE			1		/*!< @brief SD data direction: write. */

// SD Controller: APIs for SD controller
INT32S drvl1_sdc_init(INT32U device_id);
INT32S drvl1_sdc_enable(INT32U device_id);
INT32S drvl1_sdc_disable(INT32U device_id);
INT32S drvl1_sdc_controller_busy_wait(INT32U device_id, INT32U timeout);
void drvl1_sdc_running_clock_rate_get(INT32U device_id, INT32U *p_rate, INT32U *p_souce_rate, INT32U *p_div);
INT32S drvl1_sdc_clock_set(INT32U device_id, INT32U speed);
INT32S drvl1_sdc_block_len_set(INT32U device_id, INT32U size);
INT32S drvl1_sdc_bus_width_set(INT32U device_id, INT8U width);
INT32S drvl1_sdc_stop_controller(INT32U device_id, INT32U timeout);
void drvl1_sdc_dummyclk_en(INT32U device_id, INT8U en);							//This function only work on GPL326XXB
void drvl1_sdc_ioint_en(INT32U device_id, INT8U en);
void drvl1_sdc_int_en(INT32U device_id, INT32U mask, INT8U en);
INT32S drvl1_sdc_sdio_int_check(INT32U device_id);
void drvl1_sdc_sdio_int_clear(INT32U device_id);
INT32S drvl1_sdc_int_en_check(INT32U device_id, INT32U mask);

// SD Controller: APIs for SD card
extern INT32S drvl1_sdc_card_init_74_cycles(INT32U device_id);
extern INT32S drvl1_sdc_card_busy_wait(INT32U device_id, INT32U timeout);
extern INT32S drvl1_sdc_command_send(INT32U device_id, INT32U command, INT32U argument);
extern INT32S drvl1_sdc_response_get(INT32U device_id, INT32U *response, INT32U timeout);
extern INT32S drvl1_sdc_data_get(INT32U device_id, INT32U *buf, INT32U timeout);
extern INT32S drvl1_sdc_command_complete_wait(INT32U device_id, INT32U timeout);
extern void drvl1_sdc_clear_rx_data_register(INT32U device_id);
extern INT32S drvl1_sdc_read_byte_by_dma(INT32U device_id, INT32U *buffer, INT32U byte_count);
extern INT32S drvl1_sdc_write_byte_by_dma(INT32U device_id, INT32U *buffer, INT32U byte_count);
extern INT32S drvl1_sdc_read_data_by_dma(INT32U device_id, INT32U *buffer, INT32U sector_count, INT8S *poll_status);
extern INT32S drvl1_sdc_read_data_by_dma_ext(INT32U device_id, INT32U *buffer, INT32U sector_count, INT8S *poll_status, INT8U polling);
extern INT32S drvl1_sdc_write_data_by_dma(INT32U device_id, INT32U *buffer, INT32U sector_count, INT8S *poll_status);
extern INT32S drvl1_sdc_write_data_by_dma_ext(INT32U device_id, INT32U *buffer, INT32U sector_count, INT8S *poll_status, INT8U polling);
extern INT32S drvl1_sdc_data_complete_wait(INT32U device_id, INT32U timeout);
extern INT32S drvl1_sdc_data_crc_status_get(INT32U device_id);

extern INT32S drvl1_sdc_command_issue(sd_l1_ctrl_t *ctrl);

extern INT32S drvl1_sdc_write_dbf_put(INT32U device_id, INT32U *buffer, INT32U sector_count, osMessageQId os_q, volatile INT8S *notify, void (*isr_func)(INT8U, INT8U));
extern INT32S drvl1_sdc_write_dbf_set(INT32U device_id, INT32U *buffer, INT32U sector_count);

extern INT32S drvl1_sdc_read_dbf_put(INT32U device_id, INT32U *buffer, INT32U sector_count, osMessageQId os_q, volatile INT8S *notify, void (*isr_func)(INT8U, INT8U));
extern INT32S drvl1_sdc_read_dbf_set(INT32U device_id, INT32U *buffer, INT32U sector_count);

extern void drvl1_sdc_dbf_free(INT32U device_id);
extern INT32S drvl1_sdc_dbf_status_get(INT32U device_id);
extern INT32S drvl1_sdc_dma_status_get(INT32U device_id);

extern INT32U drvl1_sdc_cal_write_timeout(INT32U sector_count);
extern INT32U drvl1_sdc_cal_read_timeout(INT32U sector_count);

extern void drvl1_sdc_set_print_status_fail_flag(INT32U device_id, INT8U enable);

extern INT32S drvl1_sdc_polling_sd(INT32U device_id, INT32U polling_ms);
extern INT32S drvl1_sdc_clear_timeout_bit(INT32U device_id, INT32U poll_cnt);

extern INT32S drvl1_sdc_card_present_check_callback_set(INT32U device_id, sd_card_present_check_callback user_card_present_check_proc); // when use this api, must call before drvl2_sd_init
extern sd_card_present_check_callback drvl1_sdc_card_present_check_callback_get(INT32U device_id); // inside used
extern INT8U drvl1_sdc_check_card_present(INT32U device_id); // inside used, this api call callback set by drvl1_sdc_card_present_check_callback_get

#endif		// __DRV_L1_SDC_H__
