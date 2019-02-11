#ifndef __DRV_L1_I2S_RX_H__
#define __DRV_L1_I2S_RX_H__

#include "drv_l1.h"
#include "typedef.h"

typedef struct i2sRxReg_s
{
    volatile INT32U     TX_CTRL;
    volatile INT32U     TX_DATA;
    volatile INT32U     TX_STATUS;
    volatile INT32U     TX_DUMMY;
    volatile INT32U     RX_CTRL;
    volatile INT32U     RX_DATA;
    volatile INT32U     RX_STATUS;
    volatile INT32U     RX_DUMMY;
} i2sRxReg_t;

typedef enum
{
	MIC_INPUT = 0,
	LINE_IN_LR,
	I2S_RX_INPUT_MAX
} I2S_RX_INPUT_PATH;

#define K_I2S_RX_STS_EMPTY                  0  // Read only
#define K_I2S_RX_STS_FULL                   1  // Read only

#define K_I2S_RX_CTL_BIT_CLRFIFO            18 // RW
#define K_I2S_RX_CTL_BIT_IRT_PEND           17 // RWC, write 1 to clear, raised when rx fifo is half
#define K_I2S_RX_CTL_BIT_EN_IRT             16 // RW, 1 enable, 0 disable
#define K_I2S_RX_CTL_BIT_OVF                15 // RWC, write 1 to clear, raised when rx fifo is full
#define K_I2S_RX_CTL_BIT_IRT_Polarity       14 // RW, 0 int low active, 1 int high active

extern i2sRxReg_t *drv_l1_i2s_rx_get_register_base_addr(INT32U channel);
extern INT32S drv_l1_i2s_rx_clear_fifo(INT32U channel);
extern INT32S drv_l1_i2s_rx_disable(INT32U channel, INT8U wait);
extern INT32S drv_l1_i2s_rx_enable(INT32U channel, INT8U wait);
extern void drv_l1_i2s_rx_disable_interrupt(INT32U channel);
extern void drv_l1_i2s_rx_enable_interrupt(INT32U channel);
extern INT32S drv_l1_i2s_rx_dma_transfer(INT32U channel,  INT8U* dstBuf, INT32U BufLen, volatile INT8S *notify);
extern INT32S drv_l1_i2s_rx_init(INT32U channel);
extern INT32U drv_l1_i2s_rx_get_fifo_count(INT32U channel);
extern INT32S drv_l1_i2s_rx_set_bit_receive_mode(INT32U channel, INT8U mode);
extern INT32S drv_l1_i2s_rx_set_data_bit_length(INT32U channel, INT32U data_size);
extern INT32S drv_l1_i2s_rx_set_edge_mode(INT32U channel, INT8U mode);
extern INT32S drv_l1_i2s_rx_set_first_frame_left_right(INT32U channel, INT8U polar);
extern INT32S drv_l1_i2s_rx_set_frame_polar(INT32U channel, INT8U polar);
extern INT32S drv_l1_i2s_rx_set_frame_size(INT32U channel, INT32U frame_size);
extern INT32S drv_l1_i2s_rx_set_framing_mode(INT32U channel, INT8U mode);
extern INT32S drv_l1_i2s_rx_set_merge_mode(INT32U channel, INT32U merge);
extern INT32S drv_l1_i2s_rx_set_mono(INT32U channel, INT32U mono);
extern INT32S drv_l1_i2s_rx_set_normal_mode_bit_align(INT32U channel, INT8U mode);
extern INT32S drv_l1_i2s_rx_set_right_channel_at_lsb_byte(INT32U channel, INT32U r_lsb);
extern INT32S drv_l1_i2s_rx_resgiter_isr(INT32U channel, void (*isr_func)(INT32U channel, INT32U isr_global), INT32U isr_global);

extern INT32S drv_l1_i2s_rx_dbf_put(INT32U channel, INT16S *data, INT32U cwlen, osMessageQId os_q, volatile INT8S *notify, void (*isr_func)(INT8U, INT8U));
extern INT32S drv_l1_i2s_rx_dbf_set(INT32U channel, INT16S *data, INT32U cwlen);
extern void drv_l1_i2s_rx_dbf_free(INT32U channel);
extern INT32S drv_l1_i2s_rx_dbf_status_get(INT32U channel);
extern INT32S drv_l1_i2s_rx_dma_status_get(INT32U channel);

/**************************************************************************
 *                   R X     A P I    FOR    AUDIO-IN                     *
 **************************************************************************/
extern void drv_l1_i2s_rx_adc_set_input_path(I2S_RX_INPUT_PATH path);
extern void drv_l1_i2s_rx_adc_init(void);
extern void drv_l1_i2s_rx_adc_exit(void);
extern void drv_l1_i2s_rx_adc_fifo_clear(void);
extern INT32S drv_l1_i2s_rx_adc_sample_rate_set(INT32U hz);
extern INT32S drv_l1_i2s_rx_adc_mono_ch_set(void);
extern INT32S drv_l1_i2s_rx_adc_start(void);
extern INT32S drv_l1_i2s_rx_adc_stop(void);
extern INT32S drv_l1_i2s_rx_adc_dbf_put(INT16S *data, INT32U cwlen, osMessageQId os_q);
extern INT32S drv_l1_i2s_rx_adc_dbf_set(INT16S *data, INT32U cwlen);
extern INT32S drv_l1_i2s_rx_adc_dbf_status_get(void);
extern INT32S drv_l1_i2s_rx_adc_dma_status_get(void);
extern void drv_l1_i2s_rx_adc_dbf_free(void);

#endif
