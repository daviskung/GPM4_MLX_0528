#ifndef __drv_l1_DMA_H__
#define __drv_l1_DMA_H__

#include "cmsis_os.h"
#include "drv_l1.h"

// DMA Controller status
#define C_DMA_STATUS_WAITING		0
#define C_DMA_STATUS_DONE			1
#define C_DMA_STATUS_TIMEOUT		-1

#define C_DMA_CHANNEL_NUM			8

#define C_DMA_CH0					0
#define C_DMA_CH1					1
#define C_DMA_CH2					2
#define C_DMA_CH3					3
#define C_DMA_CH4					4
#define C_DMA_CH5					5
#define C_DMA_CH6					6
#define C_DMA_CH7					7

#define C_DMA_IO_ADDR_START			0xC0000000
#define C_DMA_IO_ADDR_END			0xDFFFFFFF

// DMA Control register
#define	C_DMA_CTRL_ENABLE			0x00000001
#define	C_DMA_CTRL_BUSY				0x00000002
#define	C_DMA_CTRL_SOFTWARE			0x00000000
#define	C_DMA_CTRL_EXTERNAL			0x00000004
#define	C_DMA_CTRL_NORMAL_INT		0x00000008
#define	C_DMA_CTRL_DEST_INCREASE	0x00000000
#define	C_DMA_CTRL_DEST_DECREASE	0x00000010
#define	C_DMA_CTRL_SRC_INCREASE		0x00000000
#define	C_DMA_CTRL_SRC_DECREASE		0x00000020
#define	C_DMA_CTRL_DEST_FIX			0x00000040
#define	C_DMA_CTRL_SRC_FIX			0x00000080
#define	C_DMA_CTRL_INT				0x00000100
#define	C_DMA_CTRL_RESET			0x00000200
#define	C_DMA_CTRL_M2M				0x00000000
#define	C_DMA_CTRL_M2IO				0x00000400
#define	C_DMA_CTRL_IO2M				0x00000800
#define	C_DMA_CTRL_8BIT				0x00000000
#define	C_DMA_CTRL_16BIT			0x00001000
#define	C_DMA_CTRL_32BIT			0x00002000
#define	C_DMA_CTRL_SINGLE_TRANS		0x00000000
#define	C_DMA_CTRL_DEMAND_TRANS		0x00004000
#define	C_DMA_CTRL_DBF				0x00008000
#define	C_DMA_CTRL_SINGLE_ACCESS	0x00000000
#define	C_DMA_CTRL_BURST4_ACCESS	0x00010000
#define	C_DMA_CTRL_BURST8_ACCESS	0x00020000

// DMA Transmit Count register
#define C_DMA_COUNT_MAX				0x00FFFFFF

// DMA Sprite register
#define C_DMA_SPRITE_MAX			0x000003FF

// DMA Transparent register
#define C_DMA_TRANSPARENT_MAX		0x0000FFFF

// DMA Line register
#define C_DMA_LINE_MAX				0x000007FF

// DMA Miscellaneuous register
#define C_DMA_MISC_STATE_MASK		0x00000007
#define C_DMA_MISC_TIMEOUT_SHIFT	4
#define C_DMA_MISC_TIMEOUT_MAX		255
#define C_DMA_MISC_TIMEOUT_MASK		0x00000FF0
#define C_DMA_MISC_TRANSPARENT_EN	0x00001000
#define C_DMA_MISC_SPRITE_TO_FB		0x00000000
#define C_DMA_MISC_FB_TO_SPRITE		0x00002000
#define C_DMA_MISC_ERROR_WRITE		0x00004000
#define C_DMA_MISC_DMA_REQUEST		0x00008000

// DMA External IO register
#define	C_DMA0_IO_SHIFT				0
#define	C_DMA1_IO_SHIFT				4
#define	C_DMA2_IO_SHIFT				8
#define	C_DMA3_IO_SHIFT				12
#define	C_DMA4_IO_SHIFT				16
#define	C_DMA5_IO_SHIFT				20
#define	C_DMA6_IO_SHIFT				24
#define	C_DMA7_IO_SHIFT				28
#define	C_DMA_IO_MASK				0xF

#define	C_DMA0_BANK_SHIFT			8
#define	C_DMA1_BANK_SHIFT			9
#define	C_DMA2_BANK_SHIFT			10
#define	C_DMA3_BANK_SHIFT			11
#define	C_DMA4_BANK_SHIFT			12
#define	C_DMA5_BANK_SHIFT			13
#define	C_DMA6_BANK_SHIFT			14
#define	C_DMA7_BANK_SHIFT			15

#define C_DMA_IO_0					0x00

/* Bank 0 device */
#define C_DMA_IO_UART0_TX			0x00
#define C_DMA_IO_UART0_RX			0x01
#define C_DMA_IO_UART1_TX			0x02
#define C_DMA_IO_UART1_RX			0x03
#define C_DMA_IO_UART2_TX			0x04
#define C_DMA_IO_UART2_RX			0x05
#define C_DMA_IO_SPI0_TX			0x06
#define C_DMA_IO_SPI0_RX			0x07
#define C_DMA_IO_SPI1_TX			0x08
#define C_DMA_IO_SPI1_RX			0x09
#define C_DMA_IO_DAC_CHA			0x0A		// Left channel
#define C_DMA_IO_DAC_CHB			0x0B		// Right channel
#define C_DMA_IO_SDC0				0x0C
#define C_DMA_IO_ADC				0x0D
#define C_DMA_IO_I2S_TX				0x0E
#define C_DMA_IO_I2S_RX				0x0F

/* Bank 1 device */
#define C_DMA_IO_I2S1_TX			0x10
#define C_DMA_IO_I2S1_RX			0x11
#define C_DMA_IO_I2S2_TX			0x12
#define C_DMA_IO_I2S2_RX			0x13
#define C_DMA_IO_I2S3_TX			0x14
#define C_DMA_IO_I2S3_RX			0x15
#define C_DMA_IO_SPU_CHL			0x16
#define C_DMA_IO_SPU_CHR			0x17
#define C_DMA_IO_SDC1				0x18
#define C_DMA_IO_FIR_TX				0x19
#define C_DMA_IO_FIR_RX				0x1A
#define C_DMA_IO_FIR_RX2			0x1B
#define C_DMA_IO_MAX				0x1F
#define C_DMA_MEMORY				0xFF

// DMA channel enable register
#define C_DMA_CE_DONT_RESET			0x00000001

// DMA INT register
#define	C_DMA0_INT_PEND				0x00000001
#define	C_DMA1_INT_PEND				0x00000002
#define	C_DMA2_INT_PEND				0x00000004
#define	C_DMA3_INT_PEND				0x00000008
#define	C_DMA0_TIMEOUT				0x00000010
#define	C_DMA1_TIMEOUT				0x00000020
#define	C_DMA2_TIMEOUT				0x00000040
#define	C_DMA3_TIMEOUT				0x00000080
#define	C_DMA0_BUSY					0x00000100
#define	C_DMA1_BUSY					0x00000200
#define	C_DMA2_BUSY					0x00000400
#define	C_DMA3_BUSY					0x00000800
#define	C_DMA4_INT_PEND				0x00001000
#define	C_DMA5_INT_PEND				0x00002000
#define	C_DMA6_INT_PEND				0x00004000
#define	C_DMA7_INT_PEND				0x00008000
#define	C_DMA4_TIMEOUT				0x00010000
#define	C_DMA5_TIMEOUT				0x00020000
#define	C_DMA6_TIMEOUT				0x00040000
#define	C_DMA7_TIMEOUT				0x00080000
#define	C_DMA4_BUSY					0x00100000
#define	C_DMA5_BUSY					0x00200000
#define	C_DMA6_BUSY					0x00400000
#define	C_DMA7_BUSY					0x00800000

#define C_DMA_Q_BUF_SIZE			1
#define C_DMA_Q_NUM					C_DMA_CHANNEL_NUM

#define C_DMA_NOT_UESED				0
#define C_DMA_NORMAL_USED			1
#define C_DMA_OCCUPIED				2

#define AES_ENABLE                  0
#define DES_3DES_ENABLE             0
typedef enum {
    DMA_DATA_WIDTH_1BYTE=1,
    DMA_DATA_WIDTH_2BYTE=2,
    DMA_DATA_WIDTH_4BYTE=4
} DMA_DW_ENUM;

typedef struct AesParam_s
{
	INT32U enable;	//aes enable,
	INT32U func;	//0:Discrypt,1:Encrypt
	INT32U mode;    //0: ECB; 1:CBC; 2:OFB; 3:CFB; 4:CTR
	//INT32U option;  //
    INT32U InRev;   //0: no In Rev; 1: In Byte Rev; 2: In Bit Rev;
    INT32U OutRev;  //0: no Out Rev; 1:Out Byte Rev; 2: Out Bit Rev;
	INT32U keyrev;  //0: no Key REV; 1: Key Rev
	INT32U ivrev;   //0: no IV REV;  1: IV REV
	INT32U key[4];	//aes key
	INT32U iv[4];
} AesParam_t;

typedef struct DesParam_s
{
    INT32U enable;	// bit[0] 0: des disable, 1: des enable
                    // bit[1] 0: 3des disable,1: 3des enable
                    // Note that des and 3des could not enable together
	INT32U func;    // 0:Descrypt, 1:Encrypt
	INT32U mode;    // 0: ECB; 1:CBC; 2:OFB; 3:CFB
    //INT32U option;  // [0], not yet implement
    INT32U InRev;   //0: no In Rev; 1: In Byte Rev; 2: In Bit Rev;
    INT32U OutRev;  //0: no Out Rev; 1:Out Byte Rev; 2: Out Bit Rev;
	INT32U keyrev;  // 0: no Key REV; 1: Key Rev
	INT32U ivrev;   // 0: no IV REV;  1: IV REV
	INT64U key[3];	// des or 3des key. Each key 56 bits at lsb part, bit [56:63] please fill 0
                        // when des, just use key[0], when 3des use key[0][1][2]
    INT64U iv;
} DesParam_t;

typedef struct ExtIntTrig_s
{
	INT8U enable;	//0: disable, 1: enable
	INT8U source;	//0: EXTA, 1: EXTB, 2: EXTC
	INT8U edge;		//0: falling edge, 1: rising edge
	INT8U RSD;
} ExtIntTrig_t;

typedef struct {
    INT32U s_addr;                          // Source address. Must align to data width
    INT32U t_addr;                          // Target address. Must align to data width
    INT32U count;                           // Transfer count(1~0x00FFFFFF). Total transfer length = width * count
    INT8U width;							// transfer width, DMA_DATA_WIDTH_1BYTE, DMA_DATA_WIDTH_2BYTE or DMA_DATA_WIDTH_4BYTE
    INT8U timeout;                          // 0: No timeout, 1~255: 1/256 ~ 255/256 second
    INT8U channel;                          // This member is used internally by DMA driver. Don't modify its value.
   	INT8U RSD;								// reserved
   	volatile INT8S *notify;                 // NULL: notification is not needed. DMA will set C_DMA_STATUS_DONE/C_DMA_STATUS_TIMEOUT when transfer is finish.
	AesParam_t *aes;						// aes parameter
	DesParam_t *des;			// des, 3des parameter
	ExtIntTrig_t *trigger;					// external interrupt trigger parameter
    INT32U checksum;
    INT32U sig;     // DEBUG use
    INT32U reserve; // DEBUG use
} DMA_STRUCT;

extern void drv_l1_dma_init(void);
extern INT32S drv_l1_dma_transfer(DMA_STRUCT *dma_struct);
extern INT32S drv_l1_dma_transfer_wait_ready(DMA_STRUCT *dma_struct);
extern INT32S dma_get_tx_count(INT32U channel, INT32U *pcount);

#if 1
extern INT32S drv_l1_dma_transfer_with_queue(DMA_STRUCT *dma_struct, osMessageQId os_q);
extern INT32S drv_l1_dma_transfer_with_callback(DMA_STRUCT *dma_struct, void (*dma_user_isr)(INT8U, INT8U));
extern INT32S drv_l1_dma_transfer_with_double_buf(DMA_STRUCT *dma_struct, osMessageQId os_q);
extern INT32S drv_l1_dma_transfer_double_buf_with_callback(DMA_STRUCT *dma_struct, osMessageQId os_q, void (*dma_user_isr)(INT8U, INT8U));
#else
extern INT32S drv_l1_dma_transfer_with_double_buf(DMA_STRUCT *dma_struct);
#endif
extern INT32S drv_l1_dma_transfer_double_buf_set(DMA_STRUCT *dma_struct);
extern INT32S drv_l1_dma_transfer_double_buf_free(DMA_STRUCT *dma_struct);
extern INT32S drv_l1_dma_dbf_status_get(INT8U channel);
extern INT32S drv_l1_dma_status_get(INT8U channel);
extern INT32S drv_l1_dma_memory_fill(INT32U t_addr, INT8U value, INT32U byte_count);
extern INT32S drv_l1_dma_buffer_copy(INT32U s_addr, INT32U t_addr, INT32U byte_count, INT32U s_width, INT32U t_width);
extern INT32U drv_l1_dma_get_channel_target_address(INT32U channel);

extern void drv_l1_dma_enforce_timeout(INT32U channel);

#endif		// __drv_l1_DMA_H__

