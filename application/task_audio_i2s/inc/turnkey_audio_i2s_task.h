#ifndef __TURNKEY_AUDIO_I2S_TASK_H__
#define __TURNKEY_AUDIO_I2S_TASK_H__
#include <string.h>
#include "application.h"
#include "drv_l1_dma.h"
#include "drv_l1_i2c.h"
#include "drv_l2_sccb.h"
#include "drv_l1_clock.h"
#include "gplib.h"
#include "turnkey_audio_decoder_task.h"

#define AUDIO_I2S_QUEUE_MAX             20
#define AUDIO_I2S_SENDQ_SIZE            MAX_I2S_BUFFERS
//#define AUDIO_RIGHT_Q_SIZE              1
#define AUDIO_I2S_PROFILE_Q_SIZE        20

#define K_I2S_USE_REAL_BOARD            1
#define K_AUDIO_STEREO                  0
#define K_AUDIO_MONO                    1

#define K_I2S_NONMERGE_MODE              0
#define K_I2S_MERGE_MODE                 1

#define K_HAVE_FILE_SYSTEM      GPLIB_FILE_SYSTEM_EN

#define K_DATA_AT_BUFFER                0
#define K_DATA_AT_DISK                  1

#if (K_HAVE_FILE_SYSTEM)
#define K_TX_DATA_SOURCE                K_DATA_AT_DISK //K_DATA_AT_BUFFER // K_DATA_AT_DISK
#else
#define K_TX_DATA_SOURCE                K_DATA_AT_BUFFER
#endif
#define SLAVE_ID_0 0x34
#define SLAVE_ID_1 0x36

//#define AUDIO_I2S_QUEUE_MAX             5
#define MAX_I2X_TX_NUM 		            4
typedef union i2s_i2c_config_def
{
    drv_l1_i2c_bus_handle_t    i2c_hw;
    sccb_config_t              i2c_gpio;
} i2s_i2c_config_t;

typedef struct i2s_i2c_def
{
    i2s_i2c_config_t config;
    INT8U use_gpio;
    void *handle;
} i2s_i2c_t;

typedef struct i2s_profile_s
{
    INT32U frame_size; // 16(0), 24(1), 32(2), 48(3), 64(4), 96(5), 128(6), 176(7), 192(8), 32(15, slave use)
    INT32U MCLK_clock_rate; // 12300000 = 12.3MHz
    INT32U sample_rate; // ex 8K=8000,48K=48000
                        // ¥?«eFPGA MCLK=12.3MHz
                        // ©?¥H1T BCLK :12.3M/4=3.075MHz
                        // 1T LRCK: 3.075/64=48KHz
                        // MCLK/1   BCLK = 12.3MHz,   LRCK = 192KHz
                        // MCLK/2,  BCLK = 6.15MHz,   LRCK =  96KHz
                        // MCLK/3,  BCLK = 4.1MHz,    LRCK =  64KHz
                        // MCLK/4,  BCLK = 3.075MHz,  LRCK =  48KHz
                        // MCLK/6,  BCLK = 2.05MHz,   LRCK =  32KHz
                        // MCLK/8,  BCLK = 1.5375MHz, LRCK =  24KHz
    INT32 MCLK_div;
    INT32U data_size;   // 16(0), 18(1), 20(2), 22(3), 24(4), 32(5), 32(6), 32(7)


    INT8U merge;
    INT8U mono;
    INT8U r_lsb;
    INT8U normal_mode_aligned;
    INT8U send_mode;
    INT8U edge_mode;
    INT8U frame_polar;
    INT8U first_frame_LR;
    INT8U framingmode; // 0 is I2S mode, 1 is Normal Mode, 2 is DSP mode, 3 is DSP mode
    INT32U amplitude; // -2, -1, 0, 1, 2
} i2s_profile_t;

typedef struct i2s_dma_s
{
    INT32U buf_len; // word unit, buffer len
    short *buffer;
    INT32U dma_bulk_size;
    INT8S notify_flag;
    INT32U dma_done_count;
    INT8S complete_flag;
    INT32U acc_dma_size;
    INT32U dma_req_count;
    INT32U underrun_count; // tx
    INT32U half_full_count;
} i2s_dma_t;

typedef struct i2s_app_s
{
    INT32U channel;
    unsigned int TestState;
    INT32U loop_max;
    int ret;
    char *task_name;

    i2s_i2c_t i2s_i2c;

    INT8U tx_data_source;   // K_TX_DATA_SOURCE
    INT8U running;

    i2s_dma_t tx_dma;
    i2s_profile_t tx_profile;

    INT16S fd_in;
    INT32S read_size;
    INT32S read_ret;
    char *in_file_name;

} i2s_app_t;

typedef union _WM8988
{
    short info;
    char  data[2];
}WM8988;

extern INT16S   *pcm_i2s_out[4][MAX_I2S_BUFFERS];
extern INT32U    pcm_i2s_len[4][MAX_I2S_BUFFERS];
extern INT32U last_send_idx;


#endif /*__TURNKEY_AUDIO_I2S_TASK_H__*/
