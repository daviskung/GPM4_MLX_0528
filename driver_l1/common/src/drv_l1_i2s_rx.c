#include <stdio.h>
#include <string.h>
#include "project.h"
#include "typedef.h"
#include "drv_l1_cfg.h"
#include "drv_l1_sfr.h"
#include "gp_stdlib.h"
#include "drv_l1_timer.h"
#if (_DRV_L1_I2S_RX == 1)
#include "drv_l1_i2s_rx.h"
#include "drv_l1_dma.h"

/* Rx Control Register */
#define I2S_RX_EN					(1 << 0)
#define I2S_RX_FIRST_FRAME_R		(1 << 1)
#define I2S_RX_FRAME_POLARITY_L		(1 << 2)
#define I2S_RX_RISING_MODE			(1 << 3)
#define I2S_RX_LSB_MODE				(1 << 4)
#define I2S_RX_LEFT_ALIGN			(1 << 5)

#define I2S_RX_VALIDDATA_16BIT		(0x00 << 6)
#define I2S_RX_VALIDDATA_18BIT		(0x01 << 6)
#define I2S_RX_VALIDDATA_20BIT		(0x02 << 6)
#define I2S_RX_VALIDDATA_22BIT		(0x03 << 6)
#define I2S_RX_VALIDDATA_24BIT		(0x04 << 6)
#define I2S_RX_VALIDDATA_32BIT		(0x05 << 6)
#define I2S_RX_VALIDDATA_08BIT		(0x06 << 6)
#define I2S_RX_VALIDDATA_MASK		(0x07 << 6)

#define I2S_RX_FRAMESIZE_16BIT		(0x00 << 9)
#define I2S_RX_FRAMESIZE_24BIT		(0x01 << 9)
#define I2S_RX_FRAMESIZE_32BIT		(0x02 << 9)
#define I2S_RX_FRAMESIZE_AUTO		(0x03 << 9)
#define I2S_RX_FRAMESIZE_MASK		(0x03 << 9)

#define I2S_RX_MODE_I2S				(0x00 << 11)
#define I2S_RX_MODE_NORMAL			(0x01 << 11)
#define I2S_RX_MODE_DSP				(0x02 << 11)
#define I2S_RX_MODE_MASK			(0x03 << 11)

#define I2S_RX_MASTER_MODE			(1 << 13)
#define I2S_RX_IRQ_POLARITY_HI		(1 << 14)
#define I2S_RX_OVWR_CLEAR			(1 << 15)
#define I2S_RX_INT_EN				(1 << 16)
#define I2S_RX_IRQ_FLAG				(1 << 17)
#define I2S_RX_FIFO_CLEAR			(1 << 18)
#define I2S_RX_MERGE				(1 << 19)
#define I2S_RX_MERGE_R_LSB			(1 << 20)
#define I2S_RX_MONO_MODE			(1 << 21)

#define CLEAR(ptr, cblen)		gp_memset((INT8S *)ptr, 0x00, cblen)

#define APPL_ENABLE				(R_SYSTEM_CKGEN_CTRL |= 0x10)
#define APPL_DISABLE			(R_SYSTEM_CKGEN_CTRL &= ~(0x10))

#define MCLK_PLL_67MHz 			{(R_SYSTEM_APLL_FREQ &= ~0x0F00); (R_SYSTEM_APLL_FREQ |= 0x0900);}
#define MCLK_PLL_73MHz 			{(R_SYSTEM_APLL_FREQ &= ~0x0F00); (R_SYSTEM_APLL_FREQ |= 0x0A00);}
//R_SYSTEM_APLL_FREQ

#define MCLK_PLL_DIV_MASK 		(R_SYSTEM_MISC_CTRL4 &= (~0xF800))
#define MCLK_PLL_DIV4 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0x1800); }
#define MCLK_PLL_DIV6 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0x2800); }
#define MCLK_PLL_DIV8 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0x3800); }
#define MCLK_PLL_DIV9 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0x4000); }
#define MCLK_PLL_DIV12 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0x5800); }
#define MCLK_PLL_DIV16 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0x7800); }
#define MCLK_PLL_DIV18          { MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0x8800); }
#define MCLK_PLL_DIV24 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0xB800); }
#define MCLK_PLL_DIV32 			{ MCLK_PLL_DIV_MASK; (R_SYSTEM_MISC_CTRL4 |= 0xF800); }

static DMA_STRUCT i2s_rx_dma_dbf[4] = {0};

i2sRxReg_t *drv_l1_i2s_rx_get_register_base_addr(INT32U channel)
{
	switch (channel)
	{
		case 0:
			return (i2sRxReg_t *)I2S0_BASE;
		case 1:
			return (i2sRxReg_t *)I2S1_BASE;
		case 2:
			return (i2sRxReg_t *)I2S2_BASE;
		case 3:
			return (i2sRxReg_t *)I2S3_BASE;
		default:
			DBG_PRINT("illegal RX channel %u\r\n", channel);
			return (i2sRxReg_t *)I2S0_BASE;
	}
}

INT32S drv_l1_i2s_rx_clear_fifo(INT32U channel)
{
	volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL |= 0x040000;           // clear FIFO
    i2s->RX_CTRL &= (~0x040000);        // clear FIFO

    return 0;
}

INT32S drv_l1_i2s_rx_disable(INT32U channel, INT8U wait)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    INT32U j;

    for (j = 0; j < 1000; j++)
    {
        if (j == 0)
          i2s->RX_CTRL &= (~0x1);			// Disable I2S_RX
        if (!wait || ((i2s->RX_CTRL & 0x1) == 0))
        {
            if (wait)
            {
                DBG_PRINT("RX disable require %d wait. RX_CTRL=0x%08x RX_STATUS=0x%08x\r\n", j, i2s->RX_CTRL, i2s->RX_STATUS);
            }
            return 0;
        }
    }
    DBG_PRINT("RX disable timeout after %d wait. RX_CTRL=0x%08x RX_STATUS=0x%08x\r\n", j, i2s->RX_CTRL, i2s->RX_STATUS);

    return -1;
}

INT32S drv_l1_i2s_rx_enable(INT32U channel, INT8U wait)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    INT32U j;

    for (j = 0; j < 1000; j++)
    {
        if (j == 0)
          i2s->RX_CTRL |= 0x1;			// Disable I2S_RX
        if (!wait || ((i2s->RX_CTRL & 0x1) == 0x1))
        {
            if (wait)
            {
                DBG_PRINT("RX enable require %d wait. RX_CTRL=0x%08x RX_STATUS=0x%08x\r\n", j, i2s->RX_CTRL, i2s->RX_STATUS);
            }
            return 0;
        }
    }
    DBG_PRINT("RX enable timeout after %d wait. RX_CTRL=0x%08x RX_STATUS=0x%08x\r\n", j, i2s->RX_CTRL, i2s->RX_STATUS);

    return -1;
}

void drv_l1_i2s_rx_disable_interrupt(INT32U channel)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    INT32U theIRQn;

    i2s->RX_CTRL &= (~(1<< K_I2S_RX_CTL_BIT_EN_IRT)); // disable interrupt

    theIRQn = I2SRX0_IRQn + channel;
    NVIC_SetPriority((IRQn_Type)theIRQn, 5);
}

void drv_l1_i2s_rx_enable_interrupt(INT32U channel)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    INT32U theIRQn;

    theIRQn = I2SRX0_IRQn + channel;
    NVIC_SetPriority((IRQn_Type)theIRQn, 5);
    NVIC_EnableIRQ((IRQn_Type)theIRQn);
    i2s->RX_CTRL |= (1<< K_I2S_RX_CTL_BIT_EN_IRT); // enable interrupt
}

INT32S drv_l1_i2s_rx_dma_transfer(INT32U channel,  INT8U* dstBuf, INT32U BufLen, volatile INT8S *notify)
{
	volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    DMA_STRUCT dma_struct = {0};
    INT32S status;

    //DBG_PRINT("P_I2SRX_DATA=0x%08x, channel=%d\r\n", (INT32U)(&(i2s->RX_DATA)), channel);

    *notify = C_DMA_STATUS_WAITING;

    //memset(&dma_struct, 0x00, sizeof(dma_struct));
    dma_struct.s_addr = (INT32U) &(i2s->RX_DATA);
    dma_struct.t_addr = (INT32U) dstBuf;
    dma_struct.width = DMA_DATA_WIDTH_4BYTE;		// DMA_DATA_WIDTH_1BYTE or DMA_DATA_WIDTH_2BYTE or DMA_DATA_WIDTH_4BYTE
    dma_struct.count = (INT32U) BufLen;
    dma_struct.notify = notify;
    dma_struct.timeout = 0;

    //dma_struct.sig = I2S_RX_SIG;
    //dma_struct.reserve = channel;

    #if 0
    status = drv_l1_dma_transfer_wait_ready(&dma_struct); // could not use wait
    *notify = C_DMA_STATUS_DONE;
    #else
    status = drv_l1_dma_transfer(&dma_struct);
    #endif
    if ( status != 0)
      return status;


    return STATUS_OK;
}

INT32S drv_l1_i2s_rx_init(INT32U channel)
{
	volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    // [19]=1 MERGE, [13]=0 SLVMOVE set RX to slave mode, [10:9]=10 Frame size 32 bit, [8:6]=000 16 bit, [2]=1 LRCK=1. LRCK=0 is left frame
    i2s->RX_CTRL = 0x84424;
    //i2s->RX_CTRL = 0x04424; // MERGE is off
    //i2s->RX_CTRL = 0x04024 | (1 << 9) | ( 4 << 6);// MERGE = 0, Frame Size[10:9]=01 24 bit, Valid FataMode [8:6]=100 24 bit

    return 0;
}

INT32U drv_l1_i2s_rx_get_fifo_count(INT32U channel)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    INT32U fifo_count;

    fifo_count = ((i2s->RX_STATUS >> 2) & 0x3f);

    return fifo_count;
}

// when 0, MSB sending first (Default)
// when 1, LSB sending first
INT32S drv_l1_i2s_rx_set_bit_receive_mode(INT32U channel, INT8U mode)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(1<<4))) | ((mode & 0x1) << 4);

    return 0;
}

static INT8U i2s_rx_data_mode_list[] = {16, 18, 20, 22, 24, 32, 32, 32};

INT32S drv_l1_i2s_rx_set_data_bit_length(INT32U channel, INT32U data_size)
{
	volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    INT8U k, data_mode_total;
    INT8U *data_mode_list;

    data_mode_total = sizeof(i2s_rx_data_mode_list);
    data_mode_list = &i2s_rx_data_mode_list[0];

    for (k=0; k < data_mode_total; k++)
    {
        if (data_size == data_mode_list[k])
        {
            i2s->RX_CTRL = (i2s->RX_CTRL & (~(((INT32U)0x7) << 6))) | (((INT32U)k) << 6);

            return 0;
        }
    }

    return -1;
}

// when 0, serial clock falling edge (Default)
// when 1, serial clock rising edge
INT32S drv_l1_i2s_rx_set_edge_mode(INT32U channel, INT8U mode)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(1<<3))) | ((mode & 0x1) << 3);

    return 0;
}

// polar = 0, left frame (Default)
// polar = 1, right frame
INT32S drv_l1_i2s_rx_set_first_frame_left_right(INT32U channel, INT8U polar)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(1<<1))) | ((polar & 0x1) << 1);

    return 0;
}

// polar = 0, LRCK =0 is right frame (Default)
// polar = 1, LRCK =0 is left frame
INT32S drv_l1_i2s_rx_set_frame_polar(INT32U channel, INT8U polar)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(1<<2))) | ((polar & 0x1) << 2);

    return 0;
}

static INT8U i2s_rx_frame_size_list[] = {16, 24, 32, 0};

INT32S drv_l1_i2s_rx_set_frame_size(INT32U channel, INT32U frame_size)
{
	volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
    INT8U k, frame_size_total;
    INT8U *frame_size_list;

    frame_size_total = sizeof(i2s_rx_frame_size_list);
    frame_size_list = &i2s_rx_frame_size_list[0];

    for (k=0; k < frame_size_total; k++)
    {
        if (frame_size == frame_size_list[k])
        {
            i2s->RX_CTRL = (i2s->RX_CTRL & (~(((INT32U)0x3) << 9))) | (((INT32U)k) << 9);

            return 0;
        }
    }

    return -1;
}

// 0 is I2S mode, 1 is Normal Mode, 2 is DSP mode, 3 is DSP mode
INT32S drv_l1_i2s_rx_set_framing_mode(INT32U channel, INT8U mode)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(((INT32U)0x3) << 11))) | (((INT32U)mode) << 11);

    return 0;
}

INT32S drv_l1_i2s_rx_set_merge_mode(INT32U channel, INT32U merge)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(((INT32U)0x1) << 19))) | ((merge & 0x1) << 19);

    return 0;
}

// when 1, received Left or Right channel according to FramePolarity and FirstFrameLR polarity
// when 0, it is stereo, receive both Left and Right channel
INT32S drv_l1_i2s_rx_set_mono(INT32U channel, INT32U mono)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(((INT32U)0x1) << 21))) | ((mono & 0x1) << 21);

    return 0;
}

// when 0, Transmitted data right aligned
// when 1, Transmitted data left aligned
INT32S drv_l1_i2s_rx_set_normal_mode_bit_align(INT32U channel, INT8U mode)
{
	volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(1<<5))) | ((mode & 0x1) << 5);

    return 0;
}

// when 1, right channel data is at [15:0], left is at [31:16]
// when 0, right channel data is at [31:16], left is at [15:0]
// note that, valid only when MEGE is on
INT32S drv_l1_i2s_rx_set_right_channel_at_lsb_byte(INT32U channel, INT32U r_lsb)
{
    volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);

    i2s->RX_CTRL = (i2s->RX_CTRL & (~(((INT32U)0x1) << 20))) | ((r_lsb & 0x1) << 20);

    return 0;
}

typedef struct i2sRxIsr_s
{
    void (*isr_func)(INT32U channel, INT32U isr_global);
    INT32U isr_global;
} i2sRxIsr_t;

static i2sRxIsr_t rx_isr[4]={{0,0}, {0,0}, {0,0}, {0,0}};

INT32S drv_l1_i2s_rx_resgiter_isr(INT32U channel, void (*isr_func)(INT32U channel, INT32U isr_global), INT32U isr_global)
{
    rx_isr[channel].isr_func = isr_func;
    rx_isr[channel].isr_global = isr_global;

    return 0;
}

void I2SRX0_IRQHandler(void)
{
    if (rx_isr[0].isr_func != 0)
    {
        rx_isr[0].isr_func(0, rx_isr[0].isr_global);
    }
}

void I2SRX1_IRQHandler(void)
{
    if (rx_isr[1].isr_func != 0)
    {
        rx_isr[1].isr_func(1, rx_isr[1].isr_global);
    }
}

void I2SRX2_IRQHandler(void)
{
    if (rx_isr[2].isr_func != 0)
    {
        rx_isr[2].isr_func(2, rx_isr[2].isr_global);
    }
}

void I2SRX3_IRQHandler(void)
{
    if (rx_isr[3].isr_func != 0)
    {
        rx_isr[3].isr_func(3, rx_isr[2].isr_global);
    }
}

/**
 * @brief   i2s rx dma double buffer put
 * @param   data[in]: buffer
 * @param   cwlen[in]: length in short unit
 * @param   os_q[in]: os queue
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_dbf_put(INT32U channel, INT16S *data, INT32U cwlen, osMessageQId os_q, volatile INT8S *notify, void (*isr_func)(INT8U, INT8U))
{
	INT32S status;
	volatile i2sRxReg_t *i2s = drv_l1_i2s_rx_get_register_base_addr(channel);
	DMA_STRUCT *pi2s_rx_dma_dbf = &i2s_rx_dma_dbf[channel];

	 if (notify != NULL)
        *notify = C_DMA_STATUS_WAITING;
	memset(pi2s_rx_dma_dbf, 0x00, sizeof(DMA_STRUCT));
	pi2s_rx_dma_dbf->s_addr = (INT32U) &(i2s->RX_DATA);
	pi2s_rx_dma_dbf->t_addr = (INT32U) data;
	if (cwlen & 0x1)
	{
        pi2s_rx_dma_dbf->width = DMA_DATA_WIDTH_2BYTE;
        pi2s_rx_dma_dbf->count = (INT32U)cwlen;
	}
	else
	{
        pi2s_rx_dma_dbf->width = DMA_DATA_WIDTH_4BYTE;
        pi2s_rx_dma_dbf->count = (INT32U)(cwlen >> 1);
    }
	pi2s_rx_dma_dbf->notify = notify;
	pi2s_rx_dma_dbf->timeout = 0;

	status = drv_l1_dma_transfer_double_buf_with_callback(pi2s_rx_dma_dbf, os_q, isr_func);
	if(status < 0) {
		return status;
	}

	return STATUS_OK;
}

/**
 * @brief   i2s rx dma double buffer sut
 * @param   data[in]: buffer
 * @param   cwlen[in]: length in short unit
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_dbf_set(INT32U channel, INT16S *data, INT32U cwlen)
{
	INT32S status;
	DMA_STRUCT *pi2s_rx_dma_dbf = &i2s_rx_dma_dbf[channel];

	pi2s_rx_dma_dbf->t_addr = (INT32U) data;

	if (pi2s_rx_dma_dbf->width == DMA_DATA_WIDTH_4BYTE)
        pi2s_rx_dma_dbf->count = (INT32U)(cwlen>>1);
	else
        pi2s_rx_dma_dbf->count = cwlen;

	status = drv_l1_dma_transfer_double_buf_set(pi2s_rx_dma_dbf);
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
void drv_l1_i2s_rx_dbf_free(INT32U channel)
{
	DMA_STRUCT *pi2s_rx_dma_dbf = &i2s_rx_dma_dbf[channel];

	drv_l1_dma_transfer_double_buf_free(pi2s_rx_dma_dbf);
	pi2s_rx_dma_dbf->channel = 0xff;
}

/**
 * @brief   i2s get rx dma double buffer status
 * @param   none
 * @return 	0: idle, 1: busy, -1: fail
 */
INT32S drv_l1_i2s_rx_dbf_status_get(INT32U channel)
{
	DMA_STRUCT *pi2s_rx_dma_dbf = &i2s_rx_dma_dbf[channel];

	if (pi2s_rx_dma_dbf->channel == 0xff) {
		return -1;
	}

	return drv_l1_dma_dbf_status_get(pi2s_rx_dma_dbf->channel);
}

/**
 * @brief   i2s get rx dma status
 * @param   none
 * @return 	0: idle, 1: busy, -1: fail
 */
INT32S drv_l1_i2s_rx_dma_status_get(INT32U channel)
{
	DMA_STRUCT *pi2s_rx_dma_dbf = &i2s_rx_dma_dbf[channel];

	if (pi2s_rx_dma_dbf->channel == 0xff) {
		return -1;
	}

	return drv_l1_dma_status_get(pi2s_rx_dma_dbf->channel);
}

/**************************************************************************
 *                   R X     A P I    FOR    AUDIO-IN                     *
 **************************************************************************/
/**
 * @brief   i2s rx form mic or line in LR path
 * @param   path[in]: MIC_INPUT or LINE_IN_LR
 * @return 	none
 */
void drv_l1_i2s_rx_adc_set_input_path(I2S_RX_INPUT_PATH path)
{
	// check CODEC LDO enable
	if((R_SYSTEM_POWER_CTRL0 & 0x100) == 0) {
		R_SYSTEM_POWER_CTRL0 |= 0x700;
	}

	// R_MIC_SETUP
	//LNINGR: 	[15:11],
	//LNINGL:	[10:6]
	//ADHP: 	[5]
	//ENMICBIAS:[4]
	//ENLNIN: 	[3]
	//MICEN: 	[2]
	//ENADR: 	[1]
	//ENADL: 	[0]

	//R_MIC_BOOST
	//MIC_PGA :	[4:0](default 0), MIC PGA
	//MIC_BOOST:[8]  (default 1), MIC boost
	//USE_SIGN: [11] (default 0)
	//USE_LR: 	[10] (default 0)

	// turn off ADC
	R_MIC_SETUP &= (~0x1F);

	if (path == MIC_INPUT) {
		R_MIC_SETUP |= 0x17;  	// MIC In, enable I2S RX clock, enable Vref
		R_MIC_BOOST |= 0x118;  	// turn on boost and set gain
	} else {
		R_MIC_SETUP |= 0x1B;  	// LineIn
		R_MIC_SETUP |= 0x1100;	// adjust volume
	}

	// wait vref ready
	drv_msec_wait(500);
}

/**
 * @brief   i2s rx initalize
 * @param   none
 * @return 	none
 */
void drv_l1_i2s_rx_adc_init(void)
{
	// APLL EN  (audio PLL)
	APPL_ENABLE;

	// set i2s for mic or line in
	R_I2SRX_CTRL |= I2S_RX_IRQ_FLAG | I2S_RX_OVWR_CLEAR;
	R_I2SRX_CTRL = 0x4820;					// Reset Value
	R_I2SRX_CTRL &= ~I2S_RX_MODE_MASK;		// I2S Mode
	R_I2SRX_CTRL |= I2S_RX_MODE_NORMAL;		// must Normal Mode because of front end design.
	R_I2SRX_CTRL |= I2S_RX_MERGE;  			// MERGE Mode
}

/**
 * @brief   i2s rx exit
 * @param   none
 * @return 	none
 */
void drv_l1_i2s_rx_adc_exit(void)
{
	R_I2SRX_CTRL = 0x4820;		// Reset Value

	// APLL disable
	APPL_DISABLE;

	// turn off MIC
	R_MIC_SETUP &= (~0xF);
	R_MIC_BOOST &= (~0x11F);
}

/**
 * @brief   i2s rx clear fifo
 * @param   none
 * @return 	none
 */
void drv_l1_i2s_rx_adc_fifo_clear(void)
{
    drv_l1_i2s_rx_clear_fifo(0);
}

/**
 * @brief   i2s rx set mono channel
 * @param   none
 * @return 	STATUS_OK or STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_adc_mono_ch_set(void)
{
	drv_l1_i2s_rx_set_mono(0, 1);

	return STATUS_OK;
}

/**
 * @brief   i2s rx set start
 * @param   none
 * @return 	STATUS_OK or STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_adc_start(void)
{
	drv_l1_i2s_rx_clear_fifo(0);
    drv_l1_i2s_rx_enable(0, 0);

	return STATUS_OK;
}

/**
 * @brief   i2s rx set stop
 * @param   none
 * @return 	STATUS_OK or STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_adc_stop(void)
{
    drv_l1_i2s_rx_disable(0, 0);
	drv_l1_i2s_rx_clear_fifo(0);

	return STATUS_OK;
}

/**
 * @brief   i2s rx dma double buffer put
 * @param   data[in]: buffer
 * @param   cwlen[in]: length in short unit
 * @param   os_q[in]: os queue
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_adc_dbf_put(INT16S *data, INT32U cwlen, osMessageQId os_q)
{
	return drv_l1_i2s_rx_dbf_put(0, data, cwlen, os_q, NULL, NULL);
}

/**
 * @brief   i2s rx dma double buffer sut
 * @param   data[in]: buffer
 * @param   cwlen[in]: length in short unit
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_adc_dbf_set(INT16S *data, INT32U cwlen)
{
	return drv_l1_i2s_rx_dbf_set(0, data, cwlen);
}

/**
 * @brief   i2s get rx dma double buffer status
 * @param   none
 * @return 	0: idle, 1: busy, -1: fail
 */
INT32S drv_l1_i2s_rx_adc_dbf_status_get(void)
{
	return drv_l1_i2s_rx_dbf_status_get(0);
}

/**
 * @brief   i2s get rx dma status
 * @param   none
 * @return 	0: idle, 1: busy, -1: fail
 */
INT32S drv_l1_i2s_rx_adc_dma_status_get(void)
{
	return drv_l1_i2s_rx_dma_status_get(0);
}

/**
 * @brief   i2s free rx dma channel
 * @param   none
 * @return 	none
 */
void drv_l1_i2s_rx_adc_dbf_free(void)
{
	drv_l1_i2s_rx_dbf_free(0);
}

/**
 * @brief   i2s tx set sample rate
 * @param   hz[in]: sample rate
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_i2s_rx_adc_sample_rate_set(INT32U hz)
{
	switch(hz)
	{
	case 48000:
		MCLK_PLL_73MHz;
		MCLK_PLL_DIV6;
		break;

	case 44100:
		MCLK_PLL_67MHz;
		MCLK_PLL_DIV6;
		break;

	case 32000:
		MCLK_PLL_73MHz;
		MCLK_PLL_DIV9;
		break;

	case 24000:
		MCLK_PLL_73MHz;
		MCLK_PLL_DIV12;
		break;

	case 22050:
		MCLK_PLL_67MHz;
		MCLK_PLL_DIV12;
		break;

	case 16000:
		MCLK_PLL_73MHz;
		MCLK_PLL_DIV18;
		break;

    case 11025:
        MCLK_PLL_67MHz;
		MCLK_PLL_DIV24;
        break;

    case 8000: //Inaccurate
        MCLK_PLL_67MHz;
		MCLK_PLL_DIV32;
        break;

	default:
		MCLK_PLL_67MHz;
		MCLK_PLL_DIV12;
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

#endif // _DRV_L1_I2S_RX
