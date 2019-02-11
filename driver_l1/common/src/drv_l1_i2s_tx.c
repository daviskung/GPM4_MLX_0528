#include <stdio.h>
#include <string.h>
#include "project.h"
#include "typedef.h"
#include "drv_l1_cfg.h"
#include "drv_l1_sfr.h"

#if (_DRV_L1_I2S_TX == 1)
#include "drv_l1_i2s_tx.h"
#include "drv_l1_dma.h"

static DMA_STRUCT i2s_tx_dma_dbf[4] = {0};

i2sTxReg_t *drv_l1_i2s_tx_get_register_base_addr(INT32U channel)
{
	switch (channel)
	{
		case 0:
			return (i2sTxReg_t *)I2S0_BASE;
		case 1:
			return (i2sTxReg_t *)I2S1_BASE;
		case 2:
			return (i2sTxReg_t *)I2S2_BASE;
		case 3:
			return (i2sTxReg_t *)I2S3_BASE;
		default:
			DBG_PRINT("illegal TX channel %u\r\n", channel);
			return (i2sTxReg_t *)I2S0_BASE;
	}
}

INT32S drv_l1_i2s_tx_clear_fifo(INT32U channel)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL |= 0x040000;		// clear FIFO
    i2s->TX_CTRL &= (~0x040000);	// clear FIFO

    return 0;
}

INT32S drv_l1_i2s_tx_disable(INT32U channel, INT8U wait)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    INT32U j;

    for (j = 0; j < 1000; j++)
    {
        if (j == 0)
          i2s->TX_CTRL &= (~0x1);			// Disable I2S_RX
        if (!wait || ((i2s->TX_STATUS & (1<<10)) == 0))
        {
            if (wait)
            {
              //DBG_PRINT("TX disable require %d wait. TX_CTRL=0x%08x TX_Status=0x%08x\r\n", j, i2s->TX_CTRL, i2s->TX_STATUS);
            }
            return 0;
        }
    }

    DBG_PRINT("TX disable timeout after %d wait. TX_CTRL=0x%08x TX_STATUS=0x%08x\r\n", j, i2s->TX_CTRL, i2s->TX_STATUS);

    return -1;
}

INT32S drv_l1_i2s_tx_enable(INT32U channel)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL |= 1;
    //DBG_PRINT("TX enable immediately. TX_CTRL=0x%08x TX_STATUS=0x%08x\r\n", i2s->TX_CTRL, i2s->TX_STATUS);

    return 0;
}

void drv_l1_i2s_tx_disable_interrupt(INT32U channel)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    INT32U theIRQn;

    i2s->TX_CTRL &= (~(1<< K_I2S_TX_CTL_BIT_EN_IRT)); // disable interrupt

    theIRQn = I2STX0_IRQn + channel;
    NVIC_SetPriority((IRQn_Type)theIRQn, 5);
    NVIC_DisableIRQ((IRQn_Type)theIRQn);
}

void drv_l1_i2s_tx_enable_interrupt(INT32U channel)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    INT32U theIRQn;

    theIRQn = I2STX0_IRQn + channel;
    NVIC_SetPriority((IRQn_Type)theIRQn, 5);
    NVIC_EnableIRQ((IRQn_Type)theIRQn);

    i2s->TX_CTRL |= (1<< K_I2S_TX_CTL_BIT_EN_IRT); // enable interrupt
}

INT32S drv_l1_i2s_tx_dma_transfer(INT32U channel, INT8U *srcBuf, INT32U BufLen, volatile INT8S *notify, INT8U *pchannel)
{
	volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    DMA_STRUCT dma_struct;
    INT32S status;

    //DBG_PRINT("P_I2STX_DATA=0x%08x, channel=%d\r\n", (INT32U)(&(i2s->TX_DATA)), channel);

    if (notify != NULL)
        *notify = C_DMA_STATUS_WAITING;
    if (pchannel != NULL)
        *pchannel = 0xff;

    memset(&dma_struct, 0x00, sizeof(dma_struct));

    dma_struct.s_addr = (INT32U) srcBuf;
    dma_struct.t_addr = (INT32U) &(i2s->TX_DATA);
    dma_struct.width = DMA_DATA_WIDTH_4BYTE;		// DMA_DATA_WIDTH_1BYTE or DMA_DATA_WIDTH_2BYTE or DMA_DATA_WIDTH_4BYTE
    dma_struct.count = (INT32U) BufLen;
    dma_struct.notify = notify;
    dma_struct.timeout = 0;

    //dma_struct.sig = I2S_TX_SIG;
    //dma_struct.reserve = channel;

    // status = drv_l1_dma_transfer_wait_ready(&dma_struct); // could not use wait
    status = drv_l1_dma_transfer(&dma_struct);
    if ( status != 0)
      return status;
    else if (pchannel != NULL)
      *pchannel = dma_struct.channel;

    return STATUS_OK;
}

INT32S drv_l1_i2s_tx_init(INT32U channel)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = 0x20104024;  // default
    i2s->TX_CTRL &= (~0x07000000);	// reset BCLK
    i2s->TX_CTRL |= 0x04000000;	// BCLK = MCLK / 4

    //drv_l1_i2s_tx_set_mclk_divider(channel, 4); // BCLK = MCLK / 4
    //drv_l1_i2s_tx_set_frame_size(channel, 32, 0); // set frame size to master 32 bit

    return 0;
}

INT32U drv_l1_i2s_tx_get_fifo_count(INT32U channel)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    INT32U left_fifo_count;

    left_fifo_count = ((i2s->TX_STATUS >> 3) & 0x3f);

    return left_fifo_count;
}

// when 0, MSB sending first (Default)
// when 1, LSB sending first
INT32S drv_l1_i2s_tx_set_bit_send_mode(INT32U channel, INT8U mode)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(1<<4))) | ((mode & 0x1) << 4);

    return 0;
}

static INT8U i2s_tx_data_mode_list[] = {16, 18, 20, 22, 24, 32, 8, 32};

INT32S drv_l1_i2s_tx_set_data_bit_length(INT32U channel, INT32U data_size)
{
	volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    INT8U k, data_mode_total;
    INT8U *data_mode_list;

    data_mode_total = sizeof(i2s_tx_data_mode_list);
    data_mode_list = &i2s_tx_data_mode_list[0];

    for (k=0; k < data_mode_total; k++)
    {
        if (data_size == data_mode_list[k])
        {
            i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x7) << 6))) | (((INT32U)k) << 6);

            return 0;
        }
    }

    return -1;
}

// when 0, serial clock falling edge (Default)
// when 1, serial clock rising edge
INT32S drv_l1_i2s_tx_set_edge_mode(INT32U channel, INT8U mode)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(1<<3))) | ((mode & 0x1) << 3);

    return 0;
}

// polar = 0, left frame (Default)
// polar = 1, right frame
INT32S drv_l1_i2s_tx_set_first_frame_left_right(INT32U channel, INT8U polar)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(1<<1))) | ((polar & 0x1) << 1);

    return 0;
}

// polar = 0, LRCK =0 is right frame (Default)
// polar = 1, LRCK =0 is left frame
INT32S drv_l1_i2s_tx_set_frame_polar(INT32U channel, INT8U polar)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(1<<2))) | ((polar & 0x1) << 2);

    return 0;
}

static INT8U i2s_tx_master_frame_size_list[] = {16, 24, 32, 48, 64, 96, 128, 176, 192};
static INT8U i2s_tx_slave_frame_size_list[] = {32};

INT32S drv_l1_i2s_tx_set_frame_size(INT32U channel, INT32U frame_size, INT8U slave)
{
	volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    INT8U k, frame_size_total;
    INT8U *frame_size_list;

    if (slave)
    {
       frame_size_total = sizeof(i2s_tx_slave_frame_size_list);
       frame_size_list = &i2s_tx_slave_frame_size_list[0];
    }
    else
    {
       frame_size_total = sizeof(i2s_tx_master_frame_size_list);
       frame_size_list = &i2s_tx_master_frame_size_list[0];
    }
    for (k=0; k < frame_size_total; k++)
    {
        if (frame_size == frame_size_list[k])
        {
            i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0xf) << 28))) | (((INT32U)k) << 28);

            return 0;
        }
    }

    return -1;
}

// 0 is I2S mode, 1 is Normal Mode, 2 is DSP mode, 3 is DSP mode
INT32S drv_l1_i2s_tx_set_framing_mode(INT32U channel, INT8U mode)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x3) << 11))) | (((INT32U)mode) << 11);

    return 0;
}

INT32S drv_l1_i2s_tx_set_mclk_divider(INT32U channel, INT32U mclk_div)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x7) << 24))) | (((INT32U)mclk_div) << 24);

    return 0;
}

INT32S drv_l1_i2s_tx_set_merge_mode(INT32U channel, INT32U merge)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x1) << 20))) | ((merge & 0x1) << 20);

    return 0;
}

// when 1, received Left or Right channel according to FramePolarity and FirstFrameLR polarity
// when 0, it is stereo, receive both Left and Right channel
INT32S drv_l1_i2s_tx_set_mono(INT32U channel, INT32U mono)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x1) << 22))) | ((mono & 0x1) << 22);

    return 0;
}

// when 0, Transmitted data right aligned
// when 1, Transmitted data left aligned
INT32S drv_l1_i2s_tx_set_normal_mode_bit_align(INT32U channel, INT8U mode)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(1<<5))) | ((mode & 0x1) << 5);

    return 0;
}

// when 1, right channel data is at [15:0], left is at [31:16]
// when 0, right channel data is at [31:16], left is at [15:0]
// note that, valid only when MEGE is on
INT32S drv_l1_i2s_tx_set_right_channel_at_lsb_byte(INT32U channel, INT32U r_lsb)
{
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);

    i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x1) << 21))) | ((r_lsb & 0x1) << 21);

    return 0;
}

// 0:256 FS, 1:384 FS, 2:512 FS, 3:768 FS
static INT16U i2s_tx_spu_fs_list[] = {256, 384, 512, 768};

INT32S drv_l1_i2s_tx_set_spu_fs(INT32U channel, INT32U spu_fs)
{
	volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    INT8U k, spu_fs_total;
    INT16U *spu_fs_list;

    spu_fs_total = sizeof(i2s_tx_spu_fs_list)/sizeof(INT16U);
    spu_fs_list = &i2s_tx_spu_fs_list[0];
    spu_fs = spu_fs/2 *2;

    for (k=0; k < spu_fs_total; k++)
    {
        //spu_fs = 256; // TEST
        if (spu_fs == spu_fs_list[k])
        {
            i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x3) << 9))) | ((k & 0x3) << 9);
            DBG_PRINT("spu use FS=%u\r\n", spu_fs);

            return 0;
        }
    }
    DBG_PRINT("Serious Error: Cant find suitable spu FS=%u\r\n", spu_fs);
    for (k=0; k < spu_fs_total; k++)
    {
        if (spu_fs <= spu_fs_list[k])
        {
            i2s->TX_CTRL = (i2s->TX_CTRL & (~(((INT32U)0x3) << 9))) | ((k & 0x3) << 9);
            DBG_PRINT("spu use FS=%u, near=%u\r\n", spu_fs, spu_fs_list[k]);

            return 0;
        }
    }

    return -1;
}

typedef struct i2sTxIsr_s
{
    void (*isr_func)(INT32U channel, INT32U isr_global);
    INT32U isr_global;
} i2sTxIsr_t;

static i2sTxIsr_t tx_isr[4]={{0,0}, {0,0}, {0,0}, {0,0}};

INT32S drv_l1_i2s_tx_resgiter_isr(INT32U channel, void (*isr_func)(INT32U channel, INT32U isr_global), INT32U isr_global)
{
    tx_isr[channel].isr_func = isr_func;
    tx_isr[channel].isr_global = isr_global;

    return 0;
}

void I2STX0_IRQHandler(void)
{
    if (tx_isr[0].isr_func != 0)
    {
        tx_isr[0].isr_func(0, tx_isr[0].isr_global);
    }
}

void I2STX1_IRQHandler(void)
{
    if (tx_isr[1].isr_func != 0)
    {
        tx_isr[1].isr_func(1, tx_isr[1].isr_global);
    }
}

void I2STX2_IRQHandler(void)
{
    if (tx_isr[2].isr_func != 0)
    {
        tx_isr[2].isr_func(2, tx_isr[2].isr_global);
    }
}

void I2STX3_IRQHandler(void)
{
    if (tx_isr[3].isr_func != 0)
    {
        tx_isr[3].isr_func(3, tx_isr[2].isr_global);
    }
}

/**
 * @brief   i2s tx dma double buffer put
 * @param   data[in]: buffer
 * @param   cwlen[in]: length in short unit
 * @param   os_q[in]: os queue
 * @param   notify[in]: pointer to 1 byte notify variable
 * @param   isr_func[in] a callback function to be called when dma done
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_i2s_tx_dbf_put(INT32U channel, INT16S *data, INT32U cwlen, osMessageQId os_q, volatile INT8S *notify, void (*isr_func)(INT8U, INT8U))
{
	INT32S status;
	volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    DMA_STRUCT *pi2s_tx_dma_dbf = &i2s_tx_dma_dbf[channel];

    if (notify != NULL)
        *notify = C_DMA_STATUS_WAITING;
	memset(pi2s_tx_dma_dbf, 0x00, sizeof(DMA_STRUCT));
	pi2s_tx_dma_dbf->s_addr = (INT32U) data;
	pi2s_tx_dma_dbf->t_addr = (INT32U) &(i2s->TX_DATA);
	if (cwlen & 0x1)
	{
        pi2s_tx_dma_dbf->width = DMA_DATA_WIDTH_2BYTE;
        pi2s_tx_dma_dbf->count = (INT32U)cwlen;
	}
	else
	{
        pi2s_tx_dma_dbf->width = DMA_DATA_WIDTH_4BYTE;
        pi2s_tx_dma_dbf->count = (INT32U)(cwlen >> 1);
    }
	pi2s_tx_dma_dbf->notify = notify;
	pi2s_tx_dma_dbf->timeout = 0;

	status = drv_l1_dma_transfer_double_buf_with_callback(pi2s_tx_dma_dbf, os_q, isr_func);
	if(status < 0) {
		return status;
	}

	return STATUS_OK;
}

/**
 * @brief   i2s tx dma double buffer set
 * @param   data[in]: buffer
 * @param   cwlen[in]: length in short unit
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_i2s_tx_dbf_set(INT32U channel, INT16S *data, INT32U cwlen)
{
	INT32S status;
    volatile i2sTxReg_t *i2s = drv_l1_i2s_tx_get_register_base_addr(channel);
    DMA_STRUCT *pi2s_tx_dma_dbf = &i2s_tx_dma_dbf[channel];

	pi2s_tx_dma_dbf->s_addr = (INT32U) data;

	if (pi2s_tx_dma_dbf->width == DMA_DATA_WIDTH_4BYTE)
        pi2s_tx_dma_dbf->count = (INT32U)(cwlen>>1);
	else
        pi2s_tx_dma_dbf->count = cwlen;

	status = drv_l1_dma_transfer_double_buf_set(pi2s_tx_dma_dbf);
	if(status < 0) {
		return status;
	}

	return STATUS_OK;
}

/**
 * @brief   i2s free rx dma channel
 * @param   none
 * @return 	none
 */
void drv_l1_i2s_tx_dbf_free(INT32U channel)
{
    DMA_STRUCT *pi2s_tx_dma_dbf = &i2s_tx_dma_dbf[channel];

	drv_l1_dma_transfer_double_buf_free(pi2s_tx_dma_dbf);
	pi2s_tx_dma_dbf->channel = 0xff;
}

/**
 * @brief   i2s get tx dma double buffer status
 * @param   none
 * @return 	0: idle, 1: busy, -1: fail
 */
INT32S drv_l1_i2s_tx_dbf_status_get(INT32U channel)
{
	DMA_STRUCT *pi2s_tx_dma_dbf = &i2s_tx_dma_dbf[channel];

	if (pi2s_tx_dma_dbf->channel == 0xff) {
		return -1;
	}

	return drv_l1_dma_dbf_status_get(pi2s_tx_dma_dbf->channel);
}

/**
 * @brief   i2s get tx dma status
 * @param   none
 * @return 	0: idle, 1: busy, -1: fail
 */
INT32S drv_l1_i2s_tx_dma_status_get(INT32U channel)
{
	DMA_STRUCT *pi2s_tx_dma_dbf = &i2s_tx_dma_dbf[channel];

	if (pi2s_tx_dma_dbf->channel == 0xff) {
		return -1;
	}

	return drv_l1_dma_status_get(pi2s_tx_dma_dbf->channel);
}

#endif // _DRV_L1_I2S_TX
