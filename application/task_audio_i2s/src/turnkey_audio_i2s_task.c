#include "turnkey_audio_i2s_task.h"

extern xQueueHandle hAudioDacTaskQ;
extern xQueueHandle aud_send_q, aud_bg_send_q, aud_right_q;
extern xQueueHandle audio_i2s_wq[4];

static i2s_app_t i2s_apps[4];
xQueueHandle aud_i2s_profile_q;
xQueueHandle aud_i2s_send_q[4];


static sccb_config_t i2c_0_2_config =
{   // I2C0 # 2
    .scl_port = IO_A8,
    .scl_drv = IOD_DRV_4mA,
    .sda_port = IO_A9,
    .sda_drv = IOD_DRV_4mA,
    .pwdn_port = 0,
    .pwdn_drv = IOD_DRV_4mA,   // not yet implemented
    .have_pwdn = 0,
    .RegBits = 8,
    .DataBits = 8,
    .slaveAddr = 0,
    .timeout = 2000,            // in ms unit, not yet implemented
    .clock_rate = 20            // in kHz unit, not yet implemented
};

static sccb_config_t i2c_1_0_config =
{   // I2C1 # 0
    .scl_port = IO_C0,
    .scl_drv = IOD_DRV_4mA,
    .sda_port = IO_C1,
    .sda_drv = IOD_DRV_4mA,
    .pwdn_port = 0,
    .pwdn_drv = IOD_DRV_4mA,   // not yet implemented
    .have_pwdn = 0,
    .RegBits = 8,
    .DataBits = 8,
    .slaveAddr = 0,
    .timeout = 2000,            // in ms unit, not yet implemented
    .clock_rate = 20            // in kHz unit, not yet implemented
};

static sccb_config_t i2c_2_2_config =
{   // I2C2 # 2
    .scl_port = IO_A10,
    .scl_drv = IOD_DRV_4mA,
    .sda_port = IO_A11,
    .sda_drv = IOD_DRV_4mA,
    .pwdn_port = 0,
    .pwdn_drv = IOD_DRV_4mA,   // not yet implemented
    .have_pwdn = 0,
    .RegBits = 8,
    .DataBits = 8,
    .slaveAddr = 0,
    .timeout = 2000,            // in ms unit, not yet implemented
    .clock_rate = 20            // in kHz unit, not yet implemented
};

static sccb_config_t i2c_IOA15_IOA14_config =
{
    .scl_port = IO_A15,
    .scl_drv = IOD_DRV_4mA,
    .sda_port = IO_A14,
    .sda_drv = IOD_DRV_4mA,
    .pwdn_port = 0,
    .pwdn_drv = IOD_DRV_4mA,   // not yet implemented
    .have_pwdn = 0,
    .RegBits = 8,
    .DataBits = 8,
    .slaveAddr = 0,
    .timeout = 2000,            // in ms unit, not yet implemented
    .clock_rate = 20            // in kHz unit, not yet implemented
};

static INT8U mclk_divider[]     = {2, 3, 4, 6, 8};
static INT8U mclk_divider_idx[] = {2, 3, 4, 6, 7}; // old
static INT8U i2s_tx_frame_size_list[] = {16, 24, 32, 48, 64, 96, 128, 176, 192};
static volatile INT8U pause[4] = {0};
static INT32U w_idx[4] = {0};

static void i2s_tx_dma_isr(INT32U i2s_channel, INT8U dma_channel, INT8U dma_done_status)
{
    //INT8U	err = 0;
    INT32S err = 0;
    INT32U  r_idx = 0;

    //DBG_PRINT("%d, %d, %d\r\n",i2s_channel,dma_channel,dma_done_status);
    if (dma_done_status != C_DMA_STATUS_DONE)
    {
        DBG_PRINT("i2s tx dma not done. i2s%u dma%u done=%u\r\n", i2s_channel, dma_channel, dma_done_status);
    }

    if (pause[i2s_channel] != 1) {
        err = (INT32S) xQueueReceiveFromISR(aud_i2s_send_q[i2s_channel], &r_idx, 0);
        if (err == pdPASS) {
            drv_l1_i2s_tx_dbf_set(i2s_channel, pcm_i2s_out[i2s_channel][r_idx], pcm_i2s_len[i2s_channel][r_idx]);
            last_send_idx = r_idx;
	    }
     }
    xQueueSendFromISR(audio_i2s_wq[i2s_channel], &w_idx[i2s_channel], NULL);
    w_idx[i2s_channel] = (w_idx[i2s_channel]+1) % MAX_I2S_BUFFERS;
}

static void i2s_tx0_dma_isr(INT8U dma_channel, INT8U dma_done_status)
{
    i2s_tx_dma_isr(0, dma_channel, dma_done_status);
}

static void i2s_tx1_dma_isr(INT8U dma_channel, INT8U dma_done_status)
{
    i2s_tx_dma_isr(1, dma_channel, dma_done_status);
}

static void i2s_tx2_dma_isr(INT8U dma_channel, INT8U dma_done_status)
{
    i2s_tx_dma_isr(2, dma_channel, dma_done_status);
}

static void i2s_tx3_dma_isr(INT8U dma_channel, INT8U dma_done_status)
{
    i2s_tx_dma_isr(3, dma_channel, dma_done_status);
}

void (*i2s_tx_dma_user_isr[MAX_I2X_TX_NUM])(INT8U, INT8U) = {i2s_tx0_dma_isr, i2s_tx1_dma_isr, i2s_tx2_dma_isr, i2s_tx3_dma_isr};

static INT32S i2s_cal_sample_rate(i2s_profile_t *pprofile)
{
    INT8U i, j, k, mclk_div_total = sizeof(mclk_divider);
    INT8U divider;
    INT8U divider_idx;
    INT8U max_divider;
    INT8U frame_size_total = sizeof(i2s_tx_frame_size_list);
    INT16U frame_size;
    INT16U fs;
    INT32U bclk, temp;
    INT32U MCLK_clock_rate;

    fs = pprofile->MCLK_clock_rate / pprofile->sample_rate;
    divider = fs / (2*pprofile->frame_size);
    max_divider = mclk_divider[mclk_div_total - 1];
    if (divider <=  max_divider)
    {
        // check if divider available
        for (k=0; k < mclk_div_total; k++)
        {
            if (divider == mclk_divider[k])
            {
                // found the divider supported, get it index
                pprofile->MCLK_div = mclk_divider_idx[k];
                DBG_PRINT("use divider %u, frame size %u, sample %u, clock %u\r\n", divider, pprofile->frame_size, pprofile->sample_rate, pprofile->MCLK_clock_rate);
                return 0;
            }
        }
        DBG_PRINT("Serious Error : Cant find suitable MCLK_div. divider=%u\r\n", divider);
        // find nearset divider
        divider = max_divider;
        divider_idx = mclk_div_total - 1;
        pprofile->MCLK_div = mclk_divider_idx[divider_idx];
    }
    else
    {
        DBG_PRINT("Serious Error : req divider over maximal divider 8. %u\r\n", divider);
        divider = max_divider;
        divider_idx = mclk_div_total - 1;
        pprofile->MCLK_div = mclk_divider_idx[divider_idx];
    }
    // else the request divider over maximal supported divider

    // use adjust frame size strategy

    bclk = pprofile->MCLK_clock_rate / divider;
    frame_size = (bclk / pprofile->sample_rate) / 2;
    if (frame_size < pprofile->data_size)
    {
        frame_size = pprofile->data_size; // frame size must greater and equal to data size
        divider = fs / (2*frame_size);
        for (k=0; k < mclk_div_total; k++)
        {
            if (divider == mclk_divider[k])
            {
                // found the divider supported, get it index
                pprofile->MCLK_div = mclk_divider_idx[k];
            }
        }
    }

    for (k=0; k < frame_size_total; k++)
    {
        if (frame_size == i2s_tx_frame_size_list[k])
        {
            pprofile->frame_size = frame_size;
            DBG_PRINT("update frame size. divider %u, frame size %u, sample %u, clock %u\r\n", divider, pprofile->frame_size, pprofile->sample_rate, pprofile->MCLK_clock_rate);
            return 0;
        }
    }

    // frame size
    DBG_PRINT("Serious Error : Cant find frame size. divider %u, frame size %u, sample %u, clock %u\r\n", divider, frame_size, pprofile->sample_rate, pprofile->MCLK_clock_rate);

    // use max frame size srategy
    for (k=0; k < frame_size_total; k++)
    {
        frame_size = i2s_tx_frame_size_list[frame_size_total - k - 1];
        if (frame_size >= pprofile->data_size)
        {
            if ((pprofile->MCLK_clock_rate % (frame_size*2)) == 0)
            {
                temp = pprofile->MCLK_clock_rate / (frame_size*2);
                if ((temp % pprofile->sample_rate) == 0)
                {
                    divider = temp / pprofile->sample_rate;
                    for (j=0; j < mclk_div_total; j++)
                    {
                        if (divider == mclk_divider[j])
                        {
                            // found the divider supported, get it index
                            pprofile->MCLK_div = mclk_divider_idx[j];
                            pprofile->frame_size = frame_size;
                            DBG_PRINT("use divider %u, frame size %u, sample %u, clock %u\r\n", divider, pprofile->frame_size, pprofile->sample_rate, pprofile->MCLK_clock_rate);
                            return 0;
                         }
                    } // end for j
                }
            }
        }
    } // end for k

    DBG_PRINT("Serious Error : Cant find divider and frame size. divider %u, frame size %u, sample %u, clock %u\r\n", divider, frame_size, pprofile->sample_rate, pprofile->MCLK_clock_rate);

    // use change MCLK strategy
    if ((pprofile->sample_rate % 8000) == 0)
    {
        I2S_MAIN_FREQ_E main_freq_list[] = {I2S_MAIN_FREQ_12288MHZ, I2S_MAIN_FREQ_18432MHZ, I2S_MAIN_FREQ_24576MHZ, I2S_MAIN_FREQ_36864MHZ};

        // 8k multiple
        for (i=0; i < 4; i++)
        {
            drv_l1_clock_set_i2s_main_mclk_freq(main_freq_list[i]);
            MCLK_clock_rate = drv_l1_clock_get_i2s_main_mclk_freq()*1000;
            for (k=0; k < frame_size_total; k++)
            {
                frame_size = i2s_tx_frame_size_list[frame_size_total - k - 1];
                if (frame_size >= pprofile->data_size)
                {
                    if ((MCLK_clock_rate % (frame_size*2)) == 0)
                    {
                        temp = MCLK_clock_rate / (frame_size*2);
                        if ((temp % pprofile->sample_rate) == 0)
                        {
                            divider = temp / pprofile->sample_rate;
                            for (j=0; j < mclk_div_total; j++)
                            {
                                if (divider == mclk_divider[j])
                                {
                                    // found the divider supported, get it index
                                    pprofile->MCLK_clock_rate = MCLK_clock_rate;
                                    pprofile->MCLK_div = mclk_divider_idx[j];
                                    pprofile->frame_size = frame_size;
                                    DBG_PRINT("use divider %u, frame size %u, sample %u, clock %u\r\n", divider, pprofile->frame_size, pprofile->sample_rate, pprofile->MCLK_clock_rate);
                                    return 0;
                                }
                            } // end for j
                        }
                    }
                }
            } // end for k
        } // end for i
    }

    return -1;
}

static void i2s_print_TX_source(i2s_app_t *pi2s_app)
{
    if (pi2s_app->tx_data_source == K_DATA_AT_DISK)
        DBG_PRINT("TX channel %d by file\r\n", pi2s_app->channel);
    else
        DBG_PRINT("TX channel %d by buffer\r\n", pi2s_app->channel);
}

static void i2s_delay(unsigned int num)
{
    volatile int i;

    for (i=0; i<num; ++i)
    {
        // nonsense , just delay
        //R_RANDOM0 = i;
        __asm("nop");
    }
}

static int i2s_i2c_init(INT32U devNumber, INT16U slaveAddr, i2s_i2c_t *pi2s_i2c)
{
    #if 0 // just for purpose to test B4, B5 connected, because this 2 bin connected to I2C
    if (devNumber == I2C_0)
    {
        gpio_init_io(IO_B4, GPIO_OUTPUT);
        gpio_init_io(IO_B5, GPIO_OUTPUT);
        gpio_set_port_attribute(IO_B4, ATTRIBUTE_HIGH);
        gpio_set_port_attribute(IO_B5, ATTRIBUTE_HIGH);
        gpio_write_io(IO_B4, DATA_HIGH);
        gpio_write_io(IO_B5, DATA_HIGH);
    }
    #endif

    if (pi2s_i2c->use_gpio)
    {
        pi2s_i2c->config.i2c_gpio.slaveAddr = slaveAddr;
        pi2s_i2c->config.i2c_gpio.clock_rate = 20;
        pi2s_i2c->handle = drv_l2_sccb_open_ext(&pi2s_i2c->config.i2c_gpio);
    }
    else
    {
        pi2s_i2c->config.i2c_hw.slaveAddr = slaveAddr;
        pi2s_i2c->config.i2c_hw.clkRate = 20; // 20; // from diag_i2C, 100
        pi2s_i2c->config.i2c_hw.devNumber = devNumber;
        drv_l1_i2c_init(pi2s_i2c->config.i2c_hw.devNumber);
        pi2s_i2c->handle = &pi2s_i2c->config.i2c_hw;
    }

    return 0;
}

static int i2c_write(i2s_i2c_t *pi2s_i2c, INT8U reg, INT8U value)
{
    int ret;
    int i=0;

    for (i=0;i<100;++i)
    {
        if (pi2s_i2c->use_gpio)
            ret = drv_l2_sccb_write(pi2s_i2c->handle , reg, value);
        else
            ret = drv_l1_reg_1byte_data_1byte_write(pi2s_i2c->handle, reg, value);
        if (ret != -1)
        {
          break;
        }
        else
        {
          DBG_PRINT("drv_l1_reg_1byte_data_1byte_write fail. reg=%d value=%0x02x\r\n", reg, value);
          i2s_delay(0x10000); // 0x4FFFF
        }
    }
    if ( ret == -1 ) {
      while (1) R_RANDOM0 = i;
    }

    return ret;
}


static short i2c_wolfson_WM8988(int addr, int data)
{
   int hi = addr << (8+1);
   int lo = data;
   int cmd_swap = (hi|lo);
   int cmd = ( ((cmd_swap>>8) & 0x000000ff) |  ((cmd_swap<<8) & 0x0000ff00) );

   return (short)cmd;
}

static void wolfson_WM8988_tx_init(i2s_i2c_t *pi2s_i2c, i2s_profile_t *ptx_profile)
{
    WM8988 pack;
    INT16U bcm;
    INT16U WL;
    INT16U BCLKINV = 0;
    INT16U LRP = 0;
    INT16U format = 2; // I2S mode
    INT16U clkdiv2 = 0;
    INT16U mclk_44K = 0;
    INT16U higher_mclk = 0;
    INT16U SR;
    INT16U LOUT2;
    INT16U ROUT2;
    INT16U LOUT1;
    INT16U ROUT1;

    pack.info = i2c_wolfson_WM8988(15,0x0); //reset
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    i2s_delay(0x10000); // 0x1F

    pack.info = i2c_wolfson_WM8988(67,0x0);
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(24,0x0);
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(25,0xFC);	// Pwr Mgmt(1) // 0xEC not work, 0x0FC work.
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(26,0x1F8);
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

#if 0
   // LOUT2 and ROUT2 set to same
    LOUT1 = ptx_profile->amplitude;
    ROUT2 = LOUT2 = ROUT1 = LOUT1;

    pack.info = i2c_wolfson_WM8988(2,0x179);  // LOUT1
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(3,0x179);   // ROUT1
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(40,LOUT2);  // LOUT2 volumn , adjust left volumn, 0x179 is 0db, 0x17f is +6db, 0x130 is -67db, 0x13f is mute
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(41,ROUT2); // ROUT2 volumn, 0x179 is 0db, 0x17f is +6db, 0x130 is -67db, 0x13f is mute
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);
#endif
    pack.info = i2c_wolfson_WM8988(5,0x0); // ADC and DAC control, default is 0x08(DACMU=1 Digital Soft Mute=1)
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    if (ptx_profile->data_size == 32)
        WL = 3;
    else if (ptx_profile->data_size == 24)
        WL = 2;
    else if (ptx_profile->data_size == 20)
        WL = 1;
    else if (ptx_profile->data_size == 16)
        WL = 0; // data_size 16 bit
    else
        WL = 3; // data_size 32 bit

    if (ptx_profile->edge_mode)
    {
      BCLKINV = 1;
    }

    if (ptx_profile->framingmode == 1)
      format = 1; // left justified

    pack.info = i2c_wolfson_WM8988(7, format | (WL<<2) | (BCLKINV << 7) | (LRP << 4)); // Audio Interface,
                                                     //  [7]BCLKINV, 0 BLCK not inverted, 1 BLCK onverted
                                                     //  [6]MS, 1 Enable Master Mode, 0 Slave Mode
                                                     //  [5]LRSWAP 1 swap left and right, 0 no swap
                                                     //  [4]LRP 1 invert LRCLK polarity
                                                     //  [3:2] WL Audio Data word length, 00 16 bits, 01 20 bits, 10 24 bits, 11 32 bits
                                                     //  [1:0] Format Audio Data Format, 00 reserve, 01 Left justified, 10 I2S format 11 DSP Mode

    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(34,0x150);// Bit 8, LD2LO=1 enable left dac to left mixer
                                             // Bit 7, LI2LO=0, disable LMISEL Signal to LeftMixer
                                             // Bit 6:4, LI2LOVOL, 5, LMISEL Signal to LeftMixer
                                             // Bit 2:0, 0 LINPUT1, 1 LINPUT2, 3 left ADC Input(after PGA/MICNOAST), 4 Differential input
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(35,0x50); // Bit 8, RD2LO=0 disable left dac to left mixer
                                             // Bit 7, RI2LO=0, disable RMISEL Signal to LeftMixer
                                             // Bit 6:4, RI2LOVOL, 5, RMISEL Signal to LeftMixer


    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(36,0x50); // Bit 8, LD2RO=0 disable left dac to right mixer
                                             // Bit 7, LI2RO=0, disable LMISEL Signal to RightMixer
                                             // Bit 6:4, LI2ROVOL, 5, LMISEL Signal to RightMixer
                                             // Bit 2:0, 0 RINPUT1, 1 RINPUT2, 3 right ADC Input(after PGA/MICNOAST), 4 Differential input

    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(37,0x150); // Bit 8, RD2RO=1 enable right dac to right mixer
                                              // Bit 7, RI2RO=0, disable RMISEL Signal to RightMixer
                                              // Bit 6:4, LI2ROVOL, 5, RMISEL Signal to RightMixer
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    bcm = 0; // tx use default clock mode instead of bcm(bit clock mode)

          // 1, MCLK/4
          // 2, MCLK/8
          // 3, MCLK/16
    if (ptx_profile->MCLK_clock_rate == 24576000 || \
        ptx_profile->MCLK_clock_rate == 22579000 || \
        ptx_profile->MCLK_clock_rate == 36864000 || \
        ptx_profile->MCLK_clock_rate == 33869000 )
        clkdiv2 = 1;

    if (ptx_profile->MCLK_clock_rate == 18432000 || \
        ptx_profile->MCLK_clock_rate == 36864000 || \
        ptx_profile->MCLK_clock_rate == 16934000 || \
        ptx_profile->MCLK_clock_rate == 33869000)
        higher_mclk = 1; // higher than

    if (ptx_profile->MCLK_clock_rate == 11289000 || \
        ptx_profile->MCLK_clock_rate == 22579000 || \
        ptx_profile->MCLK_clock_rate == 16934000 || \
        ptx_profile->MCLK_clock_rate == 33869000 )
      mclk_44K = 1; // 44K freq series

    if (ptx_profile->sample_rate == 24000)
    {
        SR = 0x1C; // SR[4:0]=11100=0x1C
        if (mclk_44K != 0)
          DBG_PRINT("sample rate 24K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 32000)
    {
        SR = 0x0C; // SR[4:0]=01100=0x0C
        if (mclk_44K != 0)
          DBG_PRINT("sample rate 32K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 96000)
    {
        SR = 0x0E; // SR[4:0]=01110=0x0E
        if (mclk_44K != 0)
          DBG_PRINT("sample rate 96K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
       //if (clkdiv2 == 0)
       //   DBG_PRINT("sample rate 96K, mclk freq must at >= 24576000 because mclk freq must >= 4xbclk freq. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 88000)
    {
        SR = 0x0E; // SR[4:0]=01110=0x0E
        if (mclk_44K == 0)
          DBG_PRINT("sample rate 88.2K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
       if (clkdiv2 == 0)
          DBG_PRINT("sample rate 88.2K, mclk freq must at >= 22579000 because mclk freq must >= 4xbclk freq. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 16000)
    {
        SR = 0x0A; // SR[4:0]=01010=0x0A
        if (mclk_44K != 0)
          DBG_PRINT("sample rate 16K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 22000) // 22050
    {
        SR = 0x0A; // SR[4:0]=01010=0x0A
        if (mclk_44K == 0)
          DBG_PRINT("sample rate 22.05K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 8000)
    {
        SR = 0x06; // SR[4:0]=00110=0x06
        if (mclk_44K != 0)
          DBG_PRINT("sample rate 8K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 8017)
    {
        SR = 0x06; // SR[4:0]=00110=0x06
        if (mclk_44K == 0)
          DBG_PRINT("sample rate 8.0182K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 12000)
    {
        SR = 0x08; // SR[4:0]=01000=0x08
        if (mclk_44K != 0)
          DBG_PRINT("sample rate 12K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 11000) // 11025
    {
        SR = 0x08; // SR[4:0]=01000=0x08
        if (mclk_44K == 0)
          DBG_PRINT("sample rate 11.025K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else if (ptx_profile->sample_rate == 44000)
    {
        SR = 0;   // SR[4:0]=00000=0x00
        if (mclk_44K == 0)
          DBG_PRINT("sample rate 44.1K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }
    else // 48K
    {
        SR = 0;   // SR[4:0]=00000=0x00
        if (mclk_44K != 0)
          DBG_PRINT("sample rate 48K, mclk freq wrong. %u\r\n", ptx_profile->MCLK_clock_rate);
    }

    pack.info = i2c_wolfson_WM8988(8, ((SR | (mclk_44K<<4) | higher_mclk)<<1) | (bcm << 7) | (clkdiv2 << 6) );

    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);
}

void wolfson_WM8988_tx_adjust_volumn(i2s_i2c_t *pi2s_i2c, INT32U amplitude)
{
    WM8988 pack;
    INT16U LOUT2;
    INT16U ROUT2;
    INT16U LOUT1;
    INT16U ROUT1;

    if (pi2s_i2c->handle == NULL)
        return;

    LOUT1 = amplitude;                          //bit[6:0]: max: 1111111b is +6dB, min: 0110000 is -67dB, 0111111 to 0000000 is mute
    ROUT2 = LOUT2 = ROUT1 = LOUT1;

    pack.info = i2c_wolfson_WM8988(2,LOUT1);    // LOUT1
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(3,ROUT1);    // ROUT1
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(40,LOUT2);   // LOUT2
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

    pack.info = i2c_wolfson_WM8988(41,ROUT2);   // ROUT2 volumn
    i2c_write(pi2s_i2c,pack.data[0] ,pack.data[1]);

}

void wolfson_WM8988_tx_mute(INT32U channel)
{
    i2s_app_t *pi2s_app;
    i2s_profile_t *ptx_profile;

    pi2s_app = &i2s_apps[channel];
    wolfson_WM8988_tx_adjust_volumn(&pi2s_app->i2s_i2c, 0x13F);
    wolfson_WM8988_tx_adjust_volumn(&pi2s_app->i2s_i2c, 0x100);
}

void wolfson_WM8988_tx_unmute(INT32U channel)
{
    i2s_app_t *pi2s_app;
    i2s_profile_t *ptx_profile;

    pi2s_app = &i2s_apps[channel];
    INT32U amplitude;

    ptx_profile = &pi2s_app->tx_profile;
    amplitude = ptx_profile->amplitude|0x100;
    wolfson_WM8988_tx_adjust_volumn(&pi2s_app->i2s_i2c, amplitude);
}

void i2s_volume_set(INT32U channel, INT32U volume)
{
    i2s_app_t *pi2s_app;
    i2s_profile_t *ptx_profile;

    pi2s_app = &i2s_apps[channel];
    ptx_profile = &pi2s_app->tx_profile;
    ptx_profile->amplitude = volume;
    volume |= 0x100;
    wolfson_WM8988_tx_adjust_volumn(&pi2s_app->i2s_i2c, volume);
}

static INT32U i2s_tx_mclk_select(INT32U sample_rate)
{
    INT32U mclk_rate;

    switch(sample_rate)
    {
        case    44100:
            mclk_rate = I2S_MAIN_FREQ_11289MHZ;
            break;
        case    48000:
            mclk_rate = I2S_MAIN_FREQ_12288MHZ;
            break;
        case    32000:
            mclk_rate = I2S_MAIN_FREQ_12288MHZ;
            break;
    }
	return mclk_rate;
}


static int i2s_tx_app_channel_prepare(AUDIO_I2S_PROFILE *paud_i2s_profile)
{
    i2s_app_t *pi2s_app;
    i2s_dma_t *ptx_dma;
    INT32U repat_buffer_count;
    i2s_profile_t *ptx_profile;
    INT32U devNumber;
    INT16U slaveAddr;
    INT32U channel = paud_i2s_profile->i2s_channel;
    INT32U spu_mclk_rate;
    INT32U spu_frame_rate = 0;

    spu_mclk_rate = i2s_tx_mclk_select(paud_i2s_profile->sample_rate);

    pi2s_app = &i2s_apps[channel];

    memset(pi2s_app, 0, sizeof(i2s_app_t));

    pi2s_app->channel = channel;
    ptx_profile = &pi2s_app->tx_profile;
    #if (K_I2S_USE_REAL_BOARD == 1)
    drv_l1_clock_enable_apll_clock();
    if (spu_mclk_rate != 0)
        drv_l1_clock_set_i2s_main_mclk_freq((I2S_MAIN_FREQ_E)spu_mclk_rate);
    ptx_profile->MCLK_clock_rate = drv_l1_clock_get_i2s_main_mclk_freq()*1000;
    #else
    ptx_profile->MCLK_clock_rate = K_MCLK_RATE;
    #endif
    DBG_PRINT("i2s main clock initial frequency=%u\r\n", drv_l1_clock_get_i2s_main_mclk_freq());

    ptx_profile->mono = K_AUDIO_STEREO;
    ptx_profile->merge = K_I2S_MERGE_MODE;
    ptx_profile->data_size = 16;
    ptx_profile->r_lsb = 0; // 0 is most tested
    ptx_profile->normal_mode_aligned = 1; //1 is most tested, and must 1 for i2s. This flag have effect only when framingmode is Normal Mode
    ptx_profile->send_mode = 0; // 0 is most tested
    ptx_profile->edge_mode = 0;  // 0 is most tested, wolfson is ok for 0 and 1
    ptx_profile->frame_polar = 1; // 1 is most tested
    ptx_profile->first_frame_LR = 0; // 0 is most tested
    ptx_profile->framingmode = 0; // 0 is I2S mode, most tested, 1 is Normal Mode, 2 is DSP mode, 3 is DSP mode
    ptx_profile->amplitude = 0;

    ptx_profile->amplitude = 0x159;

    if(paud_i2s_profile->sample_rate==44100)
        ptx_profile->sample_rate = 44000;
    else if(paud_i2s_profile->sample_rate==22050)
        ptx_profile->sample_rate = 22000;
    else if(paud_i2s_profile->sample_rate==11025)
        ptx_profile->sample_rate = 11000;
    else
        ptx_profile->sample_rate = paud_i2s_profile->sample_rate;

    ptx_profile->frame_size = 32;
    ptx_profile->data_size = 16;


    i2s_cal_sample_rate(ptx_profile);

    // change main mclk freq have to set fs for spu
    if (spu_frame_rate != 0)
        drv_l1_i2s_tx_set_spu_fs(channel, ptx_profile->MCLK_clock_rate/spu_frame_rate);


    pi2s_app->tx_data_source = K_TX_DATA_SOURCE;
    ptx_dma = &pi2s_app->tx_dma;
    pi2s_app->i2s_i2c.use_gpio = 0;

    if (channel == 1)
    {
        #if K_I2S_USE_REAL_BOARD == 1

        pi2s_app->i2s_i2c.use_gpio = 1;
        pi2s_app->i2s_i2c.config.i2c_gpio = i2c_0_2_config;
        R_FUNPOS0 = (R_FUNPOS0 & (~(0x3 << 25))) | (2 << 25); // williamyeo, for I2S board I2C0 #2
        devNumber = I2C_0;
        slaveAddr = SLAVE_ID_1;

        #else
        devNumber = I2C_0;
        slaveAddr = SLAVE_ID_1;
        #endif
    }
    else if (channel == 2)
    {
        #if K_I2S_USE_REAL_BOARD == 1
        pi2s_app->i2s_i2c.use_gpio = 1;
        pi2s_app->i2s_i2c.config.i2c_gpio = i2c_1_0_config;
        devNumber = I2C_1;
        slaveAddr = SLAVE_ID_1;
        #else
        devNumber = I2C_1;
        slaveAddr = SLAVE_ID_0;
        #endif
    }
    else if (channel == 3)
    {
        #if K_I2S_USE_REAL_BOARD == 1
        pi2s_app->i2s_i2c.use_gpio = 1;
        pi2s_app->i2s_i2c.config.i2c_gpio = i2c_IOA15_IOA14_config;
        devNumber = I2C_1;
        slaveAddr = SLAVE_ID_1;
        #else
        devNumber = I2C_1;
        slaveAddr = SLAVE_ID_1;
        #endif
    }
    else
    {
        #if K_I2S_USE_REAL_BOARD == 1
        pi2s_app->i2s_i2c.use_gpio = 1;
        pi2s_app->i2s_i2c.config.i2c_gpio = i2c_2_2_config;
        devNumber = I2C_2;
        slaveAddr = SLAVE_ID_1;
        R_FUNPOS0 = (R_FUNPOS0 & (~(0x3 << 16))) | (2 << 16); // williamyeo, for I2S board I2C2 #2
                                                              // document wrongly write I2C2 pinmux control at
                                                              // R_FUNPOS1[16:17], should be R_FUNPOS0[16:17]
                                                              // but R_FUNPOS0[16:17] also used by FIR...
        #else
        devNumber = I2C_0;
        slaveAddr = SLAVE_ID_0;
        #endif
    }

    i2s_i2c_init(devNumber, slaveAddr, &pi2s_app->i2s_i2c);

    wolfson_WM8988_tx_init(&pi2s_app->i2s_i2c, ptx_profile);

    {
        drv_l1_i2s_tx_init(pi2s_app->channel);

        drv_l1_i2s_tx_set_framing_mode(pi2s_app->channel, ptx_profile->framingmode);
        drv_l1_i2s_tx_set_mclk_divider(pi2s_app->channel, ptx_profile->MCLK_div);
        drv_l1_i2s_tx_set_frame_size(pi2s_app->channel, ptx_profile->frame_size, 0);
        drv_l1_i2s_tx_set_data_bit_length(pi2s_app->channel, ptx_profile->data_size);
        drv_l1_i2s_tx_set_merge_mode(pi2s_app->channel, ptx_profile->merge);
        drv_l1_i2s_tx_set_mono(pi2s_app->channel, ptx_profile->mono);
        drv_l1_i2s_tx_set_right_channel_at_lsb_byte(pi2s_app->channel, ptx_profile->r_lsb);
        drv_l1_i2s_tx_set_normal_mode_bit_align(pi2s_app->channel, ptx_profile->normal_mode_aligned);
        drv_l1_i2s_tx_set_bit_send_mode(pi2s_app->channel, ptx_profile->send_mode);
        drv_l1_i2s_tx_set_edge_mode(pi2s_app->channel, ptx_profile->edge_mode);
        drv_l1_i2s_tx_set_frame_polar(pi2s_app->channel, ptx_profile->frame_polar);
        drv_l1_i2s_tx_set_first_frame_left_right(pi2s_app->channel, ptx_profile->first_frame_LR);

        drv_l1_i2s_tx_clear_fifo(pi2s_app->channel);
    }

    return 0;
}

static void i2s_tx_dma_start(INT8U ch_id)
{
    INT32S  err = 0;
    INT32U  r_idx = 0;
    i2s_app_t *pi2s_app;
    i2s_dma_t *ptx_dma;

    pi2s_app = &i2s_apps[ch_id];
    ptx_dma = &pi2s_app->tx_dma;

    err = (INT32S)xQueueReceive(aud_i2s_send_q[ch_id], &r_idx, 10);
    if(err == pdPASS) {
        drv_l1_i2s_tx_dbf_put(ch_id, pcm_i2s_out[ch_id][r_idx], pcm_i2s_len[ch_id][r_idx], &ptx_dma->notify_flag, NULL, i2s_tx_dma_user_isr[ch_id]);
    }

    err = (INT32S)xQueueReceive(aud_i2s_send_q[ch_id], &r_idx, 10);
    if(err == pdPASS) {
        drv_l1_i2s_tx_dbf_set(ch_id, pcm_i2s_out[ch_id][r_idx], pcm_i2s_len[ch_id][r_idx]);
    }

    drv_l1_i2s_tx_enable(ch_id);
}

void audio_i2s_task_init()
{
    INT8U   i;

    hAudioDacTaskQ = xQueueCreate(AUDIO_I2S_QUEUE_MAX, sizeof(INT32U));
    //aud_send_q = xQueueCreate(AUDIO_I2S_SENDQ_SIZE, sizeof(INT32U));
    //aud_bg_send_q = xQueueCreate(AUDIO_I2S_SENDQ_SIZE, sizeof(INT32U));
    //aud_right_q = xQueueCreate(AUDIO_RIGHT_Q_SIZE, sizeof(INT32U));
    aud_i2s_profile_q = xQueueCreate(AUDIO_I2S_PROFILE_Q_SIZE, sizeof(INT32U));

    for(i=0; i<4; i++)
        aud_i2s_send_q[i] = xQueueCreate(AUDIO_I2S_SENDQ_SIZE, sizeof(INT32U));
}

void audio_i2s_task_entry(void *p_arg)
{
    INT32S  err = 0;
	INT32U  r_idx = 0;
    INT32U  spu_frame_rate = 0;
    //INT32U  spu_mclk_rate = I2S_MAIN_FREQ_11289MHZ;
    INT32S  audio_i2s_msg;
    INT32U  ch_id;
    AUDIO_I2S_PROFILE *paud_i2s_profile;

    audio_i2s_task_init();

    while(1){
        err = (INT32S) xQueueReceive(hAudioDacTaskQ, &audio_i2s_msg, portMAX_DELAY);
        if(err != pdPASS) {
            DBG_PRINT("%x ",err);
        }

        switch(audio_i2s_msg) {
            case MSG_AUD_I2S_PREPARE:

                err = (INT32S) xQueueReceive(aud_i2s_profile_q, &paud_i2s_profile, portMAX_DELAY);
                if(err != pdPASS) {
                    DBG_PRINT("%x ",err);
                }
                i2s_tx_app_channel_prepare(paud_i2s_profile);
                break;

			case MSG_AUD_DMA_DBF_START:

                err = (INT32S) xQueueReceive(aud_i2s_profile_q, &paud_i2s_profile, portMAX_DELAY);
                if(err != pdPASS) {
                    DBG_PRINT("%x ",err);
                }
                ch_id = paud_i2s_profile->i2s_channel;
                i2s_tx_dma_start(ch_id);
                DBG_PRINT("i2s start ch%d\r\n",ch_id);
				pause[ch_id] = 0;
				break;
			case MSG_AUD_DMA_DBF_RESTART:

				break;
			case MSG_AUD_DMA_WIDX_CLEAR:
                err = (INT32S) xQueueReceive(aud_i2s_profile_q, &paud_i2s_profile, portMAX_DELAY);
                if(err != pdPASS) {
                    DBG_PRINT("%x ",err);
                }
                ch_id = paud_i2s_profile->i2s_channel;
                w_idx[ch_id] = 0;
				break;
			case MSG_AUD_DMA_PAUSE:
                err = (INT32S) xQueueReceive(aud_i2s_profile_q, &paud_i2s_profile, portMAX_DELAY);
                if(err != pdPASS) {
                    DBG_PRINT("%x ",err);
                }
                ch_id = paud_i2s_profile->i2s_channel;
                pause[ch_id] = 1;
                DBG_PRINT("i2s pause ch%d\r\n",ch_id);
			    break;
			case MSG_AUD_SPU_SOFTCH_START:

				break;
			case MSG_AUD_SPU_LEFT_DONE:

				break;
			case MSG_AUD_SPU_WIDX_CLEAR:

				break;
			case MSG_AUD_SPU_WIDX_SET:

                break;

		#if (defined MCU_VERSION) && (MCU_VERSION < GPL327XX) && (_DRV_L1_SPU == 1)
			case MSG_AUD_RAMP_DOWN_START:

				break;
		#endif
			default:
				break;
		}

    }

}
