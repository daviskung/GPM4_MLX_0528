#ifndef SPI_NAND_SUPPORT_LIST_H
#define SPI_NAND_SUPPORT_LIST_H

// moreNandInfo
#define SPI_NAND_HAVE_2_PLANE			(1<<0)
#define SPI_NAND_HAVE_2_DIE				(1<<1)
#define SPI_2_DIE_SELECT_BY_C2_CMD	    (1<<2)
// bit 4~7 0 is driver strength type

// availFeature
#define SPI_NAND_DRIVER_SUPPORT_2_PLANE	(1<<0)

// enableFeature
#define SPI_NAND_ENABLE_2_PLANE			(1<<0)

// eccSpareMapId enum
#define SPI_NAND_SPARE_ECC_MAP_LINEAR_2K_PAGE		        0
#define SPI_NAND_SPARE_ECC_MAP_LINEAR_4K_PAGE		        1
#define SPI_NAND_SPARE_ECC_MAP_MICRON_8_BIT_ECC		        2
#define SPI_NAND_SPARE_ECC_MAP_MXIC_2K_PAGE			        3
#define SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC		        4
#define SPI_NAND_SPARE_ECC_MAP_ESMT_1_BIT_OLD_ECC	        5
#define SPI_NAND_SPARE_ECC_MAP_PARAGON_8_BIT_ECC	        6
#define SPI_NAND_SPARE_ECC_MAP_PARAGON_4K_PAGE_8_BIT_ECC	7
#define SPI_NAND_SPARE_ECC_MAP_ETRON_2K_PAGE			    8
#define SPI_NAND_SPARE_ECC_MAP_ETRON_4K_PAGE			    9

// eccMapId enum
#define SPI_NAND_ECC_MAP_BASIC							0
#define SPI_NAND_ECC_MAP_MICRON_8_BIT_ECC				1
#define SPI_NAND_ECC_MAP_GIGADEVICE_B_8_BIT_ECC			2
#define SPI_NAND_ECC_MAP_TOSHIBA_8_BIT_ECC				3
#define SPI_NAND_ECC_MAP_MXIC_WITH_INTERNAL_ECC_STATUS	4
#define SPI_NAND_ECC_MAP_GIGADEVICE_C_8_BIT_ECC			5
#define SPI_NAND_ECC_MAP_ETRON							6

// driver strength enum
#define SPI_NAND_DRIVER_STRENGTH_GD                     1
#define SPI_NAND_DRIVER_STRENGTH_ESMT                   2

typedef struct
{
	INT16U mainId;
	INT16U pageSize;
	INT16U spareSize;
	INT16U pageNumPerBlk;
	INT32U totalBlkNum;
	INT32U moreNandInfo;
	INT32U availFeature;
	INT32U enableFeature;
	INT8U  eccMax;				// maximal flip bits that could be corrected by spi nand
	INT8U  eccMapId;			// way of knowning number of flip bits
	INT8U  eccSpareMapId;		// way of mapping 16 spares byte to actual position of spare area
	INT8U  eccThreshold;
	INT16U moreSpareSize;
	INT8U  blockLockMask;
	INT8U  cacheReadCmdNrDummyByte; // number of dummy byte 03H, 0BH, 3BH, 6BH, GigaDevice only

}Spi_Nand_HalInfo_st;

typedef struct
{
	INT16U start;
	INT16U nr;
}Spi_Nand_Position_Range_st;

typedef struct
{
	INT8U map_id;
	INT8U nr_range;
	INT16U length;
	Spi_Nand_Position_Range_st *positions;
}Spi_Nand_Spare_Ecc_Position_st;

extern void print_string(CHAR *fmt, ...);

#endif //SPI_NAND_SUPPORT_LIST_H
