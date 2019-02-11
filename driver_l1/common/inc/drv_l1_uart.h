#ifndef __drv_l1_UART_H__
#define __drv_l1_UART_H__

#include "drv_l1.h"

// UART RX error status register
#define	C_UART_ERR_FRAME		0x00000001
#define	C_UART_ERR_PARITY		0x00000002
#define	C_UART_ERR_BREAK		0x00000004
#define	C_UART_ERR_OVERRUN		0x00000008

// UART control register
#define	C_UART_CTRL_RX_INT		0x00008000
#define	C_UART_CTRL_TX_INT		0x00004000
#define	C_UART_CTRL_RXTO_INT		0x00002000
#define	C_UART_CTRL_UART_ENABLE		0x00001000
#define	C_UART_CTRL_MODEM_INT		0x00000800
#define	C_UART_CTRL_LOOPBACK		0x00000400
#define	C_UART_CTRL_WORD_8BIT		0x00000060
#define	C_UART_CTRL_WORD_7BIT		0x00000040
#define	C_UART_CTRL_WORD_6BIT		0x00000020
#define	C_UART_CTRL_WORD_5BIT		0x00000000
#define	C_UART_CTRL_FIFO_ENABLE		0x00000010
#define	C_UART_CTRL_2STOP_BIT		0x00000008
#define	C_UART_CTRL_EVEN_PARITY		0x00000004
#define	C_UART_CTRL_PARITY_EN		0x00000002
#define	C_UART_CTRL_SEND_BREAK		0x00000001

// UART interrupt pending bit and status register
#define	C_UART_STATUS_RX_INT		0x00008000
#define	C_UART_STATUS_TX_INT		0x00004000
#define	C_UART_STATUS_RXTO_INT		0x00002000
#define	C_UART_STATUS_TX_EMPTY		0x00000080
#define	C_UART_STATUS_RX_FULL		0x00000040
#define	C_UART_STATUS_TX_FULL		0x00000020
#define	C_UART_STATUS_RX_EMPTY		0x00000010
#define	C_UART_STATUS_BUSY		0x00000008

// UART FIFO register
#define	C_UART_FIFO_TX_LEVEL_0		0x00000000
#define	C_UART_FIFO_TX_LEVEL_1		0x00001000
#define	C_UART_FIFO_TX_LEVEL_2		0x00002000
#define	C_UART_FIFO_TX_LEVEL_3		0x00003000
#define	C_UART_FIFO_TX_LEVEL_4		0x00004000
#define	C_UART_FIFO_TX_LEVEL_5		0x00005000
#define	C_UART_FIFO_TX_LEVEL_6		0x00006000
#define	C_UART_FIFO_TX_LEVEL_7		0x00007000
#define	C_UART_FIFO_RX_LEVEL_0		0x00000000
#define	C_UART_FIFO_RX_LEVEL_1		0x00000010
#define	C_UART_FIFO_RX_LEVEL_2		0x00000020
#define	C_UART_FIFO_RX_LEVEL_3		0x00000030
#define	C_UART_FIFO_RX_LEVEL_4		0x00000040
#define	C_UART_FIFO_RX_LEVEL_5		0x00000050
#define	C_UART_FIFO_RX_LEVEL_6		0x00000060
#define	C_UART_FIFO_RX_LEVEL_7		0x00000070

#define UART_RX			        0
#define UART_TX			        1
#define UART_DISABLE	                0
#define UART_ENABLE		        1

#define UART_FIFO_SIZE                  32

#define UART0_DEV		        0
#define UART1_DEV		        1
#define UART2_DEV		        2
#define UART_DEV_NO		        3

typedef enum
{
	WORD_LEN_5,
	WORD_LEN_6,
	WORD_LEN_7,
	WORD_LEN_8
} WORD_LEN;

typedef enum
{
	STOP_SIZE_1,
	STOP_SIZE_2
} STOP_BIT_SIZE;

typedef enum
{
	PARITY_ODD,
	PARITY_EVEN
} PARITY_SEL;

typedef enum
{
    UART_MODE,
    SIR_MODE
} UART_SIR_MODE;

typedef void(*SIR_IRQ_HANDLE)(INT32U);

// UART0 extern API
#if (defined _DRV_L1_UART0) && (_DRV_L1_UART0 == 1)
extern void drv_l1_uart0_init(void);
extern void drv_l1_uart0_buad_rate_set(INT32U bps);
extern void drv_l1_uart0_rx_enable(void);
extern void drv_l1_uart0_rx_disable(void);
extern void drv_l1_uart0_tx_enable(void);
extern void drv_l1_uart0_tx_disable(void);
extern void drv_l1_uart0_data_send(INT8U data, INT8U wait);
extern INT32S drv_l1_uart0_data_get(INT8U *data, INT8U wait);
extern INT32S drv_l1_uart0_word_len_set(INT8U word_len);
extern INT32S drv_l1_uart0_stop_bit_size_set(INT8U stop_size);
extern INT32S drv_l1_uart0_parity_chk_set(INT8U status, INT8U parity);
extern void drv_l1_uart0_rx_isr_set(void (*user_isr)(INT8U, INT8U));
#endif

// UART1 extern API
#if (defined _DRV_L1_UART1) && (_DRV_L1_UART1 == 1)
extern void drv_l1_uart1_init(void);
extern void drv_l1_uart1_buad_rate_set(INT32U bps);
extern void drv_l1_uart1_rx_enable(void);
extern void drv_l1_uart1_rx_disable(void);
extern void drv_l1_uart1_tx_enable(void);
extern void drv_l1_uart1_tx_disable(void);
extern void drv_l1_uart1_data_send(INT8U data, INT8U wait);
extern INT32S drv_l1_uart1_data_get(INT8U *data, INT8U wait);
extern INT32S drv_l1_uart1_word_len_set(INT8U word_len);
extern INT32S drv_l1_uart1_stop_bit_size_set(INT8U stop_size);
extern INT32S drv_l1_uart1_parity_chk_set(INT8U status, INT8U parity);
extern void drv_l1_uart1_rx_isr_set(void (*user_isr)(INT8U, INT8U));
#endif

// UART2 extern API
#if (defined _DRV_L1_UART2) && (_DRV_L1_UART2 == 1)
extern void drv_l1_uart2_init(void);
extern void drv_l1_uart2_buad_rate_set(INT32U bps);
extern void drv_l1_uart2_rx_enable(void);
extern void drv_l1_uart2_rx_disable(void);
extern void drv_l1_uart2_tx_enable(void);
extern void drv_l1_uart2_tx_disable(void);
extern void drv_l1_uart2_data_send(INT8U data, INT8U wait);
extern INT32S drv_l1_uart2_data_get(INT8U *data, INT8U wait);
extern INT32S drv_l1_uart2_word_len_set(INT8U word_len);
extern INT32S drv_l1_uart2_stop_bit_size_set(INT8U stop_size);
extern INT32S drv_l1_uart2_parity_chk_set(INT8U status, INT8U parity);
extern void drv_l1_uart2_rx_isr_set(void (*user_isr)(INT8U, INT8U));
#endif

extern void drv_l1_uart_set_mode(INT32U dev_idx, INT32U mode, INT32U handle);
#endif		// __drv_l1_UART_H__
