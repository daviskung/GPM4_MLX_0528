/******************************************************
* gspi_master_drv.h
*
* Purpose: GSPI master header file
*
* Author: Eugene Hsu
*
* Date: 2015/08/27
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/
#ifndef __GSPI_MASTER_DRV_H__
#define __GSPI_MASTER_DRV_H__
/*
#define BIT0	0x0001
#define BIT1	0x0002
#define BIT2	0x0004
#define BIT3	0x0008
#define BIT4	0x0010
#define BIT5	0x0020
#define BIT6	0x0040
#define BIT7	0x0080
#define BIT8	0x0100
#define BIT9	0x0200
#define BIT10	0x0400
#define BIT11	0x0800
#define BIT12	0x1000
#define BIT13	0x2000
#define BIT14	0x4000
#define BIT15	0x8000
#define BIT16	0x00010000
#define BIT17	0x00020000
#define BIT18	0x00040000
#define BIT19	0x00080000
#define BIT20	0x00100000
#define BIT21	0x00200000
#define BIT22	0x00400000
#define BIT23	0x00800000
#define BIT24	0x01000000
#define BIT25	0x02000000
#define BIT26	0x04000000
#define BIT27	0x08000000
#define BIT28	0x10000000
#define BIT29	0x20000000
#define BIT30	0x40000000
#define BIT31	0x80000000
*/
#define SPI_LOCAL_DOMAIN 				0x0
#define SPI_TXFIFO_DOMAIN 				0xc
#define SPI_RXFIFO_DOMAIN 				0x1f

//IO Bus domain address mapping
#define DEFUALT_OFFSET					0x0
#define SPI_LOCAL_OFFSET    			0x10250000
#define SPI_TX_FIFO_OFFSET	    		0x10310000
#define SPI_RX_FIFO_OFFSET	    		0x10340000

#define SPI_LOCAL_DEVICE_ID				0
#define SPI_TXQ_FIFO_DEVICE_ID			3
#define SPI_RXQ_FIFO_DEVICE_ID			7
#define SPI_UNDEFINED_DEVICE_ID			(-1)

//SPI Local registers
#define SPI_REG_INT_CTRL				0x0004 // 4 bytes, SPI INT Control
#define SPI_REG_INT_TIMEOUT		   		0x0006  // 2 bytes, SPI 32us INT timout
#define SPI_REG_HIMR					0x0014 // 4 bytes, SPI Host Interrupt Mask
#define SPI_REG_HISR					0x0018 // 4 bytes, SPI Host Interrupt Service Routine
#define SPI_REG_RX0_REQ_LEN				0x001C // 4 bytes, RXDMA Request Length
#define SPI_REG_FREE_TX_SPACE			0x0020 // 4 bytes, Free Tx Buffer Page
#define SPI_REG_TX_SEQNUM				0x0024 // 1 byte, TX Sequence Number Definition
#define SPI_REG_HCPWM					0x0038 // 1 byte, HCI Current Power Mode
#define SPI_REG_HCPWM2			 		0x003A // 2 bytes, HCI Current Power Mode 2
#define SPI_REG_AVAI_PATH_L				0x0040 // 4 bytes, SPI TX Available Low Size reg
#define SPI_REG_AVAI_PATH_H				0x0044 // 4 bytes, SPI TX Available High Size reg
#define SPI_REG_RX_AGG_CTL				0x0048 // 4 bytes, SPI RX AGG control
#define SPI_REG_H2C_MSG					0x004C // 4 bytes, SPI_REG_H2C_MSG
#define SPI_REG_C2H_MSG					0x0050  // 4 bytes, SPI_REG_C2H_MSG
#define SPI_REG_HRPWM					0x0080  // 1 byte, SPI_REG_HRPWM
#define SPI_REG_HPS_CLKR				0x0084 // 1 byte, not uesd
#define SPI_REG_CPU_IND					0x0087 // 1 byte, firmware indication to host
#define SPI_REG_32K_TRANS_CTL			0x0088 // 1 byte, 32K transparent control, BIT0 EN32K_TRANS
#define SPI_REG_32K_IDLE_TIME			0x008B // 1 byte, 32K idle time,
#define SPI_REG_DELY_LINE_SEL			0x008C // 1 byte, Delay line selection,
#define SPI_REG_SPI_CFG					0x00F0 // 1 byte, SPI configuration,

#define LOCAL_REG_FREE_TX_SPACE			(SPI_LOCAL_OFFSET | SPI_REG_FREE_TX_SPACE)

// Register SPI_REG_CPU_IND
#define SPI_CPU_RDY_IND		(BIT0)

/************************************************/
// SPI_REG_HISR: SDIO Host Interrupt Service Routine
#define SPI_HISR_RX_REQUEST				(BIT0)
#define SPI_HISR_AVAL_INT				(BIT1)
#define SPI_HISR_TXPKT_OVER_BUFF		(BIT2)
#define SPI_HISR_TX_AGG_SIZE_MISMATCH	(BIT3)
#define SPI_HISR_TXBD_OVF				(BIT4)
//BIT5~16 not used
#define SPI_HISR_C2H_MSG_INT			(BIT17)
#define SPI_HISR_CPWM1_INT				(BIT18)
#define SPI_HISR_CPWM2_INT				(BIT19)
//BIT20~31 not used
#define SPI_HISR_CPU_NOT_RDY			(BIT22)


#define MASK_SPI_HISR_CLEAR		(SPI_HISR_RX_REQUEST |\
						SPI_HISR_AVAL_INT |\
						SPI_HISR_TXPKT_OVER_BUFF |\
						SPI_HISR_TX_AGG_SIZE_MISMATCH |\
						SPI_HISR_TXBD_OVF |\
						SPI_HISR_C2H_MSG_INT |\
						SPI_HISR_CPWM1_INT |\
						SPI_HISR_CPWM2_INT)

// RTL8195A SPI Host Interrupt Mask Register
#define SPI_HIMR_RX_REQUEST_MSK				(BIT0)
#define SPI_HIMR_AVAL_MSK					(BIT1)
#define SPI_HIMR_TXPKT_SIZE_OVER_BUFF_MSK	(BIT2)
#define SPI_HIMR_AGG_SIZE_MISMATCH_MSK		(BIT3)
#define SPI_HIMR_TXBD_OVF_MSK				(BIT4)
//BIT5~16 not used
#define SPI_HIMR_C2H_MSG_INT_MSK			(BIT17)
#define SPI_HIMR_CPWM1_INT_MSK				(BIT18)
#define SPI_HIMR_CPWM2_INT_MSK				(BIT19)
//BIT20~31 not used
#define SPI_HIMR_DISABLED			0

// Register SPI_REG_HCPWM
#define SPI_HCPWM_WLAN_TRX			(BIT1)

#define GSPI_CMD_LEN		4
#define GSPI_STATUS_LEN		8

#define FILL_SPI_CMD(byte_en, addr, domain_id, fun, write_flag) 	((byte_en & 0xff) | ((addr & 0xffff) << 8) \
									| ((domain_id & 0x1f) << 24) | ((fun & 0x3) << 29) | ((write_flag & 0x1) << 31))

#define GET_STATUS_HISR(status)			((((*(INT32U*)status)) & 0x3) |((((*(INT32U*)status) >> 2) & 0x7) << 17))
#define GET_STATUS_FREE_TX(status)		((((*(INT32U*)status) >> 5) & 0x7ffffff) << 2)
#define GET_STATUS_RXQ_REQ_LEN(status)	(((*(INT32U*)((INT8U *)status + 4))) & 0xffffff)
#define GET_STATUS_TX_SEQ(status)		(((*(INT32U)((INT8U *)status + 4)) >> 24) & 0xff)

#define GSPI_CMD_TX         0x83
#define GSPI_CMD_RX			0X82

// GSPI configuration (big endian recommended)
#define GSPI_CONFIG SPI_BIG_ENDIAN_16

#define TwoByteSwap(_twobyte)	(_twobyte)

typedef struct _GSPI_RX_DESC{
	// u4Byte 0
	INT32U	pkt_len:16;     // bit[15:0], the packet size
	INT32U	offset:8;    	// bit[23:16], the offset from the packet start to the buf start, also means the size of RX Desc
	INT32U	rsvd0:6;        // bit[29:24]
	INT32U	icv:1;          // bit[30], ICV error
	INT32U	crc:1;          // bit[31], CRC error

	// u4Byte 1
	INT32U	type:8;         // bit[7:0], the type of this packet
	INT32U	rsvd1:24;       // bit[31:8]

	// u4Byte 2
	INT32U	rsvd2;

	// u4Byte 3
	INT32U	rsvd3;

	// u4Byte 4
	INT32U	rsvd4;

	// u4Byte 5
	INT32U	rsvd5;
} GSPI_RX_DESC, *PGSPI_RX_DESC;

typedef struct _GSPI_TX_DESC{
	// u4Byte 0
	INT32U	txpktsize:16;       // bit[15:0]
	INT32U	offset:8;    		// bit[23:16], store the sizeof(SDIO_TX_DESC)
	INT32U	bus_agg_num:8;		// bit[31:24], the bus aggregation number

	// u4Byte 1
    INT32U type:8;             // bit[7:0], the packet type
    INT32U rsvd0:24;

	// u4Byte 2
	INT32U	rsvd1;

	// u4Byte 3
	INT32U	rsvd2;

	// u4Byte 4
	INT32U	rsvd3;

	// u4Byte 5
	INT32U	rsvd4;
} GSPI_TX_DESC, *PGSPI_TX_DESC;

// For memory set command
typedef struct _GSPI_DESC_MS{
	// u4Byte 0
	INT32U	txpktsize:16;       // bit[15:0]
	INT32U	offset:8;    		// bit[23:16], store the sizeof(TX_DESC)
	INT32U	bus_agg_num:8;		// bit[31:24], the bus aggregation number

	// u4Byte 1
    INT32U type:8;             // bit[7:0], the packet type
    INT32U data:8;             // bit[8:15], the value to be written to the memory
    INT32U reply:1;            // bit[16], request to send a reply message
    INT32U rsvd0:15;
	// u4Byte 2
	INT32U	start_addr;         // memory write start address

	// u4Byte 3
    INT32U write_len:16;       // bit[15:0], the length to write
    INT32U rsvd2:16;           // bit[31:16]

	// u4Byte 4
	INT32U	rsvd3;

	// u4Byte 5
	INT32U	rsvd4;
} GSPI_DESC_MS, *PGSPI_DESC_MS;

// For memory write reply command
typedef struct _GSPI_DESC_MW{
	// u4Byte 0
	INT32U	txpktsize:16;       // bit[15:0]
	INT32U	offset:8;    		// bit[23:16], store the sizeof(TX_DESC)
	INT32U	bus_agg_num:8;		// bit[31:24], the bus aggregation number

	// u4Byte 1
    INT32U type:8;             // bit[7:0], the packet type
    INT32U reply:1;            // bit[8], request to send a reply message
    INT32U rsvd0:23;

	// u4Byte 2
	INT32U	start_addr;         // memory write start address

	// u4Byte 3
    INT32U write_len:16;       // bit[15:0], the length to write
    INT32U rsvd2:16;           // bit[31:16]

	// u4Byte 4
	INT32U	rsvd3;

	// u4Byte 5
	INT32U	rsvd4;
}  GSPI_DESC_MW, *PGSPI_DESC_MW;

// TX Desc for Jump to Start command
typedef struct _GSPI_DESC_JS{
	// u4Byte 0
	INT32U	txpktsize:16;       // bit[15:0]
	INT32U	offset:8;    		// bit[23:16], store the sizeof(TX_DESC)
	INT32U	bus_agg_num:8;		// bit[31:24], the bus aggregation number

	// u4Byte 1
    INT32U type:8;             // bit[7:0], the packet type
    INT32U rsvd0:24;

	// u4Byte 2
	INT32U	start_fun;         // the pointer of the startup function

	// u4Byte 3
	INT32U	rsvd2;

	// u4Byte 4
	INT32U	rsvd3;

	// u4Byte 5
	INT32U	rsvd4;
} GSPI_DESC_JS, *PGSPI_DESC_JS;

enum
{
	SPI_LITTLE_ENDIAN = 2,
	SPI_BIG_ENDIAN = 0
};

enum
{
	SPI_WORD_LEN_16 = 0,
	SPI_WORD_LEN_32 = 1
};

typedef enum
{
	SPI_LITTLE_ENDIAN_16 = SPI_LITTLE_ENDIAN|SPI_WORD_LEN_16,
	SPI_LITTLE_ENDIAN_32 = SPI_LITTLE_ENDIAN|SPI_WORD_LEN_32, // default configure
	SPI_BIG_ENDIAN_16 = SPI_BIG_ENDIAN|SPI_WORD_LEN_16,
	SPI_BIG_ENDIAN_32 = SPI_BIG_ENDIAN|SPI_WORD_LEN_32
}_gspi_conf_t;

typedef enum
{
	READ_REG = 0,
	WRITE_REG
}_reg_ops;

struct gspi_more_data
{
	INT32U more_data;
	INT32U len;
};

typedef struct WIFI_IMAGE_INFO_S
{
	INT32U loader_start;
	INT32U loader_end;
	INT32U app_start;
	INT32U app_end;
} WIFI_IMAGE_INFO_T;

// CCPWM2 bit map definition for Firmware download
#define GSPI_INIT_DONE					(BIT0)
#define GSPI_MEM_WR_DONE				(BIT1)
#define GSPI_MEM_RD_DONE				(BIT2)
#define GSPI_MEM_ST_DONE				(BIT3)
#define GSPI_CPWM2_TOGGLE				(BIT15)

// Register REG_SPDIO_CPU_IND
#define GPSI_SYSTEM_TRX_RDY_IND		(BIT0)

// define transmit packat type
#define GPSI_TX_PACKET_802_3	(0x83)
#define GSPI_TX_PACKET_802_11	(0x81)
#define GSPI_TX_H2C_CMD			(0x11)
#define GSPI_TX_MEM_READ		(0x51)
#define GSPI_TX_MEM_WRITE		(0x53)
#define GSPI_TX_MEM_SET			(0x55)
#define GSPI_TX_FM_FREETOGO		(0x61)

#define GSPI_CMD_LEN		4
#define GSPI_STATUS_LEN		8

#define SIZE_TX_DESC	(sizeof(GSPI_TX_DESC))
#define SIZE_RX_DESC	(sizeof(GSPI_RX_DESC))

#define AGG_SIZE	5000
#define PACK_SIZE	2048
#define MAX_DLFW_PAGE_SIZE		2048
#define BUFFER_LEN	(4+ 24 + PACK_SIZE + 8) // GSPI_CMD + TX_DEC + DATA + GSPI_STATUS

typedef void(*GSPI_RX_DATA_CBK)(INT8U* buf, INT32U len);

extern void print_string(CHAR *fmt, ...);

extern INT32S gspi_fw_download_start(WIFI_IMAGE_INFO_T* image_info, INT32U wifi_module);
extern INT32S gspi_master_init(void);
extern int gspi_read_rx_fifo(INT8U *buf, INT32U len, struct gspi_more_data * pmore_data,_gspi_conf_t gspi_conf);
extern int gspi_write_tx_fifo(INT8U *buf, INT32U len, _gspi_conf_t gspi_conf);
extern int gspi_write_page(INT8U *buf, INT32U len, INT8U agg_cnt);
extern int gspi_read_page(INT8U *buf, INT32U* len);
extern void gspi_tx_data(INT8U* buf, INT32U len, INT32U type);
extern INT32S gspi_write32(INT32U addr, INT32U buf, INT32S *err);
extern INT32U gspi_read32(INT32U addr, INT32S *err);
extern INT16U gspi_read16(INT32U addr, INT32S *err);
extern INT8U gspi_read8(INT32U addr, INT32S *err);
extern void gspi_master_cs_init(void);
extern void gspi_semaphore_init(void);
extern void gspi_ext_interrpt_init(void);
extern int gspi_configuration(_gspi_conf_t gspi_conf);
extern void gspi_config_slave_device(void);
extern void gspi_register_rx_cbk(void* cbk);
extern void gspi_set_cs_io(INT32U gpio);
extern void gspi_set_ext_int(INT8U extint);
extern void gspi_set_spi_num(INT8U num);
#endif	//__GSPI_MASTER_DRV_H__
