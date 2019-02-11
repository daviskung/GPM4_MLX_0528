#include "project.h"

#ifdef USE_NANDSPI_INSTEAD_NANDFLASH

#include "spi_nand_support_list.h"

extern INT32S spi_nand_get_ecc_info(INT8U *pEccMax, INT8U *pEccMapId, INT8U *pEccThreshold, INT8U *pEccSpareMapId);
extern INT32S spi_nand_read_internal_ecc_status(INT8U* pEccStatus);
extern INT32S spi_nand_read_ecc_extend(INT8U* statusPtr);
extern void spi_nand_page_addr_to_blk(INT32U page_addr, INT16U *p_blk, INT16U *p_offset);

const Spi_Nand_HalInfo_st spi_nand_info[] =
{
	// EM73C044SNC, Etron 1G, 2K page
	{
		0xD531,									// MainID
		2048,									// PageSize
		72,									    // SpareSize
		64,	  									// PageNumPerBlk
		1024,          							// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_ETRON,	                // EccMapId
		SPI_NAND_SPARE_ECC_MAP_ETRON_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// EM73D044SNB, Etron 2G, 2K page
	{
		0xD532,									// MainID
		2048,									// PageSize
		72,									    // SpareSize
		64,	  									// PageNumPerBlk
		2048,          							// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_ETRON,	                // EccMapId
		SPI_NAND_SPARE_ECC_MAP_ETRON_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// EM73E044SNB, Etron 4G, 4K page
	{
		0xD523,									// MainID
		4096,									// PageSize
		144,								    // SpareSize
		64,	  									// PageNumPerBlk
		2048,          							// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_ETRON,	                // EccMapId
		SPI_NAND_SPARE_ECC_MAP_ETRON_4K_PAGE,	// SpareMapId
		7,										// EccThreshold
		256,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// GD5F1GQ4UxB, GigaDevice 1G,
	// GD5F1GQ4xExIG-Rev1.6, GigaDevice 1G, tested.
	{
		0xC8D1,									// MainID
		2048,									// PageSize
		64,									    // SpareSize
		64,	  									// PageNumPerBlk
		1024,          							// TotalBlkNum
		0x00000010,			    				// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_GIGADEVICE_B_8_BIT_ECC,// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MXIC_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// GD5F2GQ4UxB, GigaDevice 2G, tested.
	{
		0xC8D2,									// MainID
		2048,									// PageSize
		64,									    // SpareSize
		64,	  									// PageNumPerBlk
		2048,          							// TotalBlkNum
		0x00000010,			    				// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_GIGADEVICE_B_8_BIT_ECC,	// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MXIC_2K_PAGE,		// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	//  GD5F1GQ4xFxIG-Rev1.7, GigaDevice 1G, tested. ID: b1 48 c8 b1 48 c8 b1 48
	{
		0xB148,									// MainID
		2048,									// PageSize
		64,								     	// SpareSize
		64,	  					  				// PageNumPerBlk
		1024,          							// TotalBlkNum
		0x00000010,			    				// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_GIGADEVICE_C_8_BIT_ECC,// EccMapId
		SPI_NAND_SPARE_ECC_MAP_LINEAR_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		1										// cacheReadCmdNrDummyByte
	},

	// GD5F1GQ4UxC, GigaDevice 1G,
	{
		0xC8B1,									// MainID
		2048,									// PageSize
		64,								     	// SpareSize
		64,	  					  				// PageNumPerBlk
		1024,          							// TotalBlkNum
		0x00000010,			    				// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_GIGADEVICE_C_8_BIT_ECC,	// EccMapId
		SPI_NAND_SPARE_ECC_MAP_LINEAR_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		1										// cacheReadCmdNrDummyByte
	},

	// GD5F2GQ4UxC, GigaDevice 2G,
	{
		0xC8B2,									// MainID
		2048,									// PageSize
		64,									    // SpareSize
		64,	  									// PageNumPerBlk
		2048,          							// TotalBlkNum
		0x00000010,			    				// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_GIGADEVICE_C_8_BIT_ECC,// EccMapId
		SPI_NAND_SPARE_ECC_MAP_LINEAR_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		1										// cacheReadCmdNrDummyByte
	},

	// MT29F1G01ABAFDWB-IT:F, Micron 1G
	{
		0x2C14,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_MICRON_8_BIT_ECC,		// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_8_BIT_ECC,// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x78,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// MT29F2G01ABAGDWB-IT:G, Micron 2G, 2 plane
	{
		0x2C24,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,										// PageNumPerBlk
		2048,									// TotalBlkNum
		SPI_NAND_HAVE_2_PLANE,					// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		SPI_NAND_DRIVER_SUPPORT_2_PLANE,		// AvailFeature
		0/*SPI_NAND_ENABLE_2_PLANE*/,           // EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_MICRON_8_BIT_ECC,		// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_8_BIT_ECC,// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x78,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// MT29F4G01ADAGDWB-IT:G, Micron 4G, 2 plane, 2 die
	{
		0x2C36,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		4096,           						// TotalBlkNum
												// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		SPI_NAND_HAVE_2_PLANE | SPI_NAND_HAVE_2_DIE,
		SPI_NAND_DRIVER_SUPPORT_2_PLANE,		// AvailFeature
		0/*SPI_NAND_ENABLE_2_PLANE*/,           // EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_MICRON_8_BIT_ECC,		// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_8_BIT_ECC,// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x78,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// MX35LF1GE4AB-Z4I, MXIC 1G, 1 plane, 1 die
	{
		0xC212,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		4,										// EccMax
		SPI_NAND_ECC_MAP_MXIC_WITH_INTERNAL_ECC_STATUS, // EccMapId
		SPI_NAND_SPARE_ECC_MAP_MXIC_2K_PAGE,	// SpareMapId
		3,										// EccThreshold
		64,										// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// MX35LF2GE4AC-Z4I, MXIC 2G, 2 plane, 1 die
	{
		0xC220,									// MainID, 0xC220(actual) or 0xC222(data sheet) ?
		2048,									// PageSize
		64,									    // SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
		SPI_NAND_HAVE_2_PLANE,					// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		SPI_NAND_DRIVER_SUPPORT_2_PLANE,		// AvailFeature
		0/*SPI_NAND_ENABLE_2_PLANE*/,           // EnableFeature
		4,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MXIC_2K_PAGE,	// SpareMapId
		4,										// EccThreshold
		64,									    // moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// TC58CVG0S3HRAIG, Toshiba 1G, 1 plane, 1 die
	{
		0x98C2,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_TOSHIBA_8_BIT_ECC, 	// EccMapId
		SPI_NAND_SPARE_ECC_MAP_LINEAR_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// TC58CVG1S3HRAIG, Toshiba 2G, 1 plane, 1 die
	{
		0x98CB,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_TOSHIBA_8_BIT_ECC, 	// EccMapId
		SPI_NAND_SPARE_ECC_MAP_LINEAR_2K_PAGE,	// SpareMapId
		7,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// TC58CVG2S0HRAIG, Toshiba 4G, 1 plane, 1 die, 4K page size
	{
		0x98CD,									// MainID
		4096,									// PageSize
		128,									// SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_TOSHIBA_8_BIT_ECC, 	// EccMapId
		SPI_NAND_SPARE_ECC_MAP_LINEAR_4K_PAGE,	// SpareMapId
		7,										// EccThreshold
		256,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// MT29F1G01AAADD, Micron 1G, 1 plane, 1 die, 2K page size, 4 bit ECC
	{
		0x2C12,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		4,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC,// SpareMapId
		4,										// EccThreshold
		64,									    // moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// MT29F2G01AAAED, Micron 2G, 2 plane, 1 die, 2K page size, 4 bit ECC
	{
		0x2C22,									// MainID ?
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		4,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC,// SpareMapId
		4,										// EccThreshold
		64,									    // moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// MT29F4G01AAADD, Micron 4G, 2 plane, 1 die, 2K page size, 4 bit ECC
	{
		0x2C32,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		4096,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		4,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC,// SpareMapId
		4,										// EccThreshold
		64,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

    // W25N01GVZEIG, WINBOND 1G, 1 plane, 1 die, 2K page size, 1 bit ECC
	{
		0xEFAA,									// MainID
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		1,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC,// SpareMapId
		1,										// EccThreshold
		64,										// moreSpareSize
		0x78,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

    // W25M02GVZEIG, WINBOND 2G, 1 plane, 2 die, 2K page size, 1 bit ECC
	{
		0xEFAB,									// MainID: ef ab 21 00 00 00 00 00
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
												// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		SPI_NAND_HAVE_2_DIE | SPI_2_DIE_SELECT_BY_C2_CMD,
		0,										// AvailFeature
		0,              						// EnableFeature
		1,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC,// SpareMapId
		1,										// EccThreshold
		64,										// moreSpareSize
		0x78,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

    // F50L1G41LB, ESMT 1G, 1 plane, 1 die, 2K page size, 1 bit ECC, ID: c8 01 7f 7f 7f 00 00 00
	{
		0xC801,									// MainID, c8 01 7f 7f 7f 00 00 00
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0x00000020,			    				// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		1,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC,// SpareMapId
		1,										// EccThreshold
		64,										// moreSpareSize
		0x78,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

    // F50L2G41LB, ESMT 2G, 1 plane, 2 die, 2K page size, 1 bit ECC, ID: c8 0a 7f 7f 7f 00 00 00
	{
		0xC80A,									// MainID: c8 0a 7f 7f 7f 00 00 00
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
												// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0x00000020 | SPI_NAND_HAVE_2_DIE | SPI_2_DIE_SELECT_BY_C2_CMD,
		0,										// AvailFeature
		0,              						// EnableFeature
		1,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC,// SpareMapId
		1,										// EccThreshold
		64,										// moreSpareSize
		0x78,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// F50L1G41A, ESMT 1G, 1 plane, 1 die, 2K page size, 1 bit ECC. Old, not tested
	{
		0xC821,									// MainID,
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		1,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_ESMT_1_BIT_OLD_ECC,// SpareMapId
		1,										// EccThreshold
		64,										// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// PN26G01AWSIUG, PARAGON 1G, 1 plane, 1 die, 2K page size, 8 bit ECC.
	{
		0xA1E1,									// MainID,
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		1024,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_PARAGON_8_BIT_ECC,// SpareMapId
		8,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// PN26G02AWSIUG, PARAGON 2G, 1 plane, 1 die, 2K page size, 8 bit ECC.
	{
		0xA1E2,									// MainID,
		2048,									// PageSize
		64,										// SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_BASIC,					// EccMapId
		SPI_NAND_SPARE_ECC_MAP_PARAGON_8_BIT_ECC,// SpareMapId
		8,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// XT26G02A, XTX 2G, 1 plane, 1 die, 2K page size, 8 bit ECC.
	{
		0x0BE2,									// MainID,
		2048,									// PageSize
		64,									    // SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_GIGADEVICE_C_8_BIT_ECC,// EccMapId
		SPI_NAND_SPARE_ECC_MAP_MXIC_2K_PAGE,	// SpareMapId
		8,										// EccThreshold
		128,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// XT26G04A, XTX 4G, 1 plane, 1 die, 4K page size, 8 bit ECC.
	{
		0x0BE3,									// MainID,
		4096,									// PageSize
		128,									// SpareSize
		64,	  									// PageNumPerBlk
		2048,           						// TotalBlkNum
		0,			    						// MoreNandInfo is 4 byte, bit 4~7 0 is driver strength type
		0,										// AvailFeature
		0,              						// EnableFeature
		8,										// EccMax
		SPI_NAND_ECC_MAP_GIGADEVICE_C_8_BIT_ECC,// EccMapId
		SPI_NAND_SPARE_ECC_MAP_PARAGON_4K_PAGE_8_BIT_ECC,// SpareMapId
		8,										// EccThreshold
		256,									// moreSpareSize
		0x38,									// blockLockMask
		0										// cacheReadCmdNrDummyByte
	},

	// End of id
	{
		0xFFFF,									// MainID
		0xFFFF,									// PageSize
		0xFFFF,									// SpareSize
		0xFFFF,	  								// PageNumPerBlk
		0xFFFFFFFF,           					// TotalBlkNum
		0xFFFFFFFF,								// MoreNandInfo
		0xFFFFFFFF,								// AvailFeature
		0xFFFFFFFF,           					// EnableFeature
		0xFF,									// EccMax
		0xFF,									// EccMapId
		0xFF,									// SpareMapId
		0,										// EccThreshold
		0,									    // moreSpareSize
		0,									    // blockLockMask
		0										// cacheReadCmdNrDummyByte
	}
};

Spi_Nand_Position_Range_st spare_ecc_map_liner_2K_page[] = {{1, 15}, {16, 16}, {32, 16}, {48, 16}};
Spi_Nand_Position_Range_st spare_ecc_map_liner_4K_page[] = {{1, 15}, {16, 16}, {32, 16}, {48, 16}, {64, 16}, {80, 16}, {96, 16}, {112, 16}};
Spi_Nand_Position_Range_st spare_ecc_map_micron_8_bit_ecc[] = {{32, 8}, {40, 8}, {48, 8}, {56, 8}};
Spi_Nand_Position_Range_st spare_ecc_map_mxic_2K_page [] = {{4, 12}, {20, 12}, {36, 12}, {52, 12}};
Spi_Nand_Position_Range_st spare_ecc_map_micron_4_bit_ecc [] = {{4, 4}, {20, 4}, {36, 4}, {52, 4}};
Spi_Nand_Position_Range_st spare_ecc_map_esmt_old_1_bit_ecc[] = {{8, 8}, {24, 8}, {40, 8}, {56, 8}};
Spi_Nand_Position_Range_st spare_ecc_map_paragon_8_bit_ecc [] = {{4, 2}, {19, 2}, {34, 2}, {49, 2}};
Spi_Nand_Position_Range_st spare_ecc_map_paragon_4K_page [] = {{4, 12}, {20, 12}, {36, 12}, {52, 12}, {68, 12}, {84, 12}, {100, 12}, {116, 12}};
Spi_Nand_Position_Range_st spare_ecc_map_etron_2K_page [] = {{1, 17}, {18, 18}, {36, 18}, {54, 18}};
Spi_Nand_Position_Range_st spare_ecc_map_etron_4K_page [] = {{1, 17}, {18, 18}, {36, 18}, {54, 18}, {72, 18}, {90, 18}, {108, 18}, {126, 18}};

Spi_Nand_Spare_Ecc_Position_st spi_nand_spare_ecc_position_info[] =
{
	{SPI_NAND_SPARE_ECC_MAP_LINEAR_2K_PAGE, sizeof(spare_ecc_map_liner_2K_page)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_liner_2K_page},
	{SPI_NAND_SPARE_ECC_MAP_LINEAR_4K_PAGE, sizeof(spare_ecc_map_liner_4K_page)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_liner_4K_page},
	{SPI_NAND_SPARE_ECC_MAP_MICRON_8_BIT_ECC, sizeof(spare_ecc_map_micron_8_bit_ecc)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_micron_8_bit_ecc},
	{SPI_NAND_SPARE_ECC_MAP_MXIC_2K_PAGE, sizeof(spare_ecc_map_mxic_2K_page)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_mxic_2K_page},
	{SPI_NAND_SPARE_ECC_MAP_MICRON_4_BIT_ECC, sizeof(spare_ecc_map_micron_4_bit_ecc)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_micron_4_bit_ecc},
	{SPI_NAND_SPARE_ECC_MAP_ESMT_1_BIT_OLD_ECC, sizeof(spare_ecc_map_esmt_old_1_bit_ecc)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_esmt_old_1_bit_ecc},
	{SPI_NAND_SPARE_ECC_MAP_PARAGON_8_BIT_ECC, sizeof(spare_ecc_map_paragon_8_bit_ecc)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_paragon_8_bit_ecc},
	{SPI_NAND_SPARE_ECC_MAP_PARAGON_4K_PAGE_8_BIT_ECC, sizeof(spare_ecc_map_paragon_4K_page)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_paragon_4K_page},
	{SPI_NAND_SPARE_ECC_MAP_ETRON_2K_PAGE, sizeof(spare_ecc_map_etron_2K_page)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_etron_2K_page},
	{SPI_NAND_SPARE_ECC_MAP_ETRON_4K_PAGE, sizeof(spare_ecc_map_etron_4K_page)/sizeof(Spi_Nand_Position_Range_st), 0, spare_ecc_map_etron_4K_page},
	{0xFF, 0, 0, NULL}
};

static INT8U eccInfoInited = 0;
static INT8U eccMax, eccMapId, eccSpareMapId, eccThreshold;

#define	STATUS_ECC_OFFSET			(0x4)
#define	STATUS_ECC_MASK_2_BIT		(0x30)
#define	STATUS_ECC_MASK_3_BIT		(0x70)

INT8S spi_nand_get_basic_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8U ecc_err;
	INT8S ecc_num = 0;

	ecc_err = (status & STATUS_ECC_MASK_2_BIT) >> STATUS_ECC_OFFSET;
	if (ecc_err == 0)
		ecc_num = 0;
	else if (ecc_err == 2)
		ecc_num = -1;
	else if (ecc_err == 1)
	{
		// when 1, possible flip bit number is 1 to ~ ecc_max
		ecc_num = (eccMax <= 1) ? eccMax : (eccMax >> 1);
	}
	else
		DBG_PRINT("ecc_err is reserve value\r\n");

	return ecc_num;
}

INT8S spi_nand_get_etron_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8U ecc_err;
	INT8S ecc_num = 0;

	ecc_err = (status & STATUS_ECC_MASK_2_BIT) >> STATUS_ECC_OFFSET;
	if (ecc_err == 0)
		ecc_num = 0;
	else if (ecc_err == 2)
		ecc_num = -1;
	else if (ecc_err == 1)
	{
		// when 1, possible flip bit number is 1 to ~ ecc_max
		ecc_num = (eccMax <= 1) ? eccMax : (eccMax >> 1);
	}
	else
	{
		ecc_num = eccMax;
	}

	return ecc_num;
}

INT8S spi_nand_get_gigadevice_B_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8U ecc_err;
	INT8S ecc_num = 0;

	ecc_err = (status & STATUS_ECC_MASK_2_BIT) >> STATUS_ECC_OFFSET;
	if (ecc_err == 0)
		ecc_num = 0;
	else if (ecc_err == 2)
		ecc_num = -1;
	else if (ecc_err == 3)
		ecc_num = eccMax;
	else if (ecc_err == 1)
	{
		INT8U ecc_ext;

		if (spi_nand_read_ecc_extend(&ecc_ext) != 0)
			ecc_ext = 0;
		// when 1, possible flip bit number is 1 to ~ ecc_max - 1
		ecc_num = (ecc_err << 2) | ecc_ext;
	}

	return ecc_num;
}

INT8S spi_nand_get_gigadevice_C_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8U ecc_err;
	INT8S ecc_num = 0;

	ecc_err = (status & STATUS_ECC_MASK_3_BIT) >> STATUS_ECC_OFFSET;
	if (ecc_err == 0)
		ecc_num = 0;
	else if (ecc_err == 7)
		ecc_num = -1;
	else
		ecc_num = ecc_err + 2;

	return ecc_num;
}

INT8S spi_nand_get_micron_8_bit_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8U ecc_err;
	INT8S ecc_num = 0;

	ecc_err = (status & STATUS_ECC_MASK_3_BIT) >> STATUS_ECC_OFFSET;
	if (ecc_err == 0)
		// No errors
		ecc_num = 0;
	else if (ecc_err == 2)
		// Bit errors greater than 8 bits detected and not corrected
		ecc_num = -1;
	else if (ecc_err == 3)
		// 4-6 bit errors detected and corrected. Indicates data refreshment might be taken
		ecc_num = 5;
	else if (ecc_err == 1)
		// 1-3 bit errors detected and corrected
		ecc_num = 2;
	else if (ecc_err == 5)
		// 7-8 bit errors detected and corrected. Indicates data refreshment must be taken to guarantee data retention
		ecc_num = 7;
	else
		DBG_PRINT("unknown ecc_err statu %u!\r\n", ecc_err);

	return ecc_num;
}

INT8S spi_nand_get_mxic_internal_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8U ecc_err;
	INT8S ecc_num = 0;

	ecc_err = (status & STATUS_ECC_MASK_2_BIT) >> STATUS_ECC_OFFSET;
	if (ecc_err == 0)
		ecc_num = 0;
	else if (ecc_err == 2)
		ecc_num = -1;
	else if (ecc_err == 1)
	{
		INT8U internalEccStatus;

		if (spi_nand_read_internal_ecc_status(&internalEccStatus) == 0)
		{
			ecc_num = internalEccStatus & 0x0F;
			if (ecc_num > eccMax)
			{
				DBG_PRINT("ecc_num > eccMax. %d, %d\r\n", ecc_num, eccMax);
				ecc_num = -1;
			}
		}
		else
			// when 1, possible flip bit number is 1 to ~ ecc_max
			ecc_num = (eccMax <= 1) ? eccMax : (eccMax >> 1);
	}
	else
		DBG_PRINT("ecc_err is reserve value\r\n");

	return ecc_num;
}

INT8S spi_nand_get_toshiba8_bit_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8U ecc_err;
	INT8S ecc_num = 0;

	ecc_err = (status & STATUS_ECC_MASK_2_BIT) >> STATUS_ECC_OFFSET;
	if (ecc_err == 0)
		ecc_num = 0;
	else if (ecc_err == 2)
		// Multiple bit flips were detected and not corrected
		ecc_num = -1;
	else if (ecc_err == 3)
		// Bit flips were detected and corrected. Bit flip count exceeded(equal and greater than) the bit flip detection threshold.
		ecc_num = eccThreshold;
	else if (ecc_err == 1)
	{
		// Bit flips were detected and corrected. Bit flip count did not exceed(lower than) the bit flip detection threshold
		ecc_num = (eccThreshold <= 3) ? eccThreshold : eccThreshold / 2;
	}

	return ecc_num;
}


INT8S spi_nand_get_ecc_num(INT8U status, INT32U pageAddr)
{
	INT8S ecc_num = 0;

	if (!eccInfoInited)
	{
		INT32S retval;

		eccMapId = 0xff;
		retval = spi_nand_get_ecc_info(&eccMax, &eccMapId, &eccThreshold, &eccSpareMapId);
		if (retval == 0)
			eccInfoInited = 1;
		else
			DBG_PRINT("fail to get ecc info !\r\n");
	}

	switch(eccMapId){
		case SPI_NAND_ECC_MAP_GIGADEVICE_B_8_BIT_ECC:
			ecc_num = spi_nand_get_gigadevice_B_ecc_num(status, pageAddr);
			break;
		case SPI_NAND_ECC_MAP_GIGADEVICE_C_8_BIT_ECC:
			ecc_num = spi_nand_get_gigadevice_C_ecc_num(status, pageAddr);
			break;
		case SPI_NAND_ECC_MAP_MICRON_8_BIT_ECC:
			ecc_num = spi_nand_get_micron_8_bit_ecc_num(status, pageAddr);
			break;
		case SPI_NAND_ECC_MAP_BASIC:
			ecc_num = spi_nand_get_basic_ecc_num(status, pageAddr);
			break;
		case SPI_NAND_ECC_MAP_ETRON:
			ecc_num = spi_nand_get_etron_ecc_num(status, pageAddr);
			break;
		case SPI_NAND_ECC_MAP_TOSHIBA_8_BIT_ECC:
			ecc_num = spi_nand_get_toshiba8_bit_ecc_num(status, pageAddr);
			break;
		case SPI_NAND_ECC_MAP_MXIC_WITH_INTERNAL_ECC_STATUS:
			ecc_num = spi_nand_get_mxic_internal_ecc_num(status, pageAddr);
			break;
		default:
			DBG_PRINT("unknown eccMapId %u!\r\n", eccMapId);
			break;
	}


	if (ecc_num != 0)
	{
        INT16U blk;
        INT16U poffset;

        spi_nand_page_addr_to_blk(pageAddr, &blk, &poffset);
		DBG_PRINT("pageAddr=%u, ecc_num=%d, ecc_sts=%u, eccMapId=%u, blk=%u, poffset=%u\r\n", pageAddr, ecc_num, (status & STATUS_ECC_MASK_3_BIT) >> STATUS_ECC_OFFSET, eccMapId, blk, poffset);
    }

	return ecc_num;
}

// GD:  0(50%, default) 1(25%) 2(75%) 3(100%)    user : 0=default, 1=100%, 2=75%, 3=50%, 4=25%
static INT8S driver_strength_table_GD[] = {0, 3, 2, 0, 1};

// ESMT: 0(100%) 1(75%, default) 2(50%) 3(25%)   user : 0=default, 1=100%, 2=75%, 3=50%, 4=25%
static INT8S driver_strength_table_ESMT[] = {1, 0, 1, 2, 3};

INT8S spi_nand_convert_driver_strength(INT8U driver_strength_type, INT8U user_driver_strength)
{
    INT8S device_driver_strength = -1; // when -1, mean unsupported

    switch (driver_strength_type) {
        case SPI_NAND_DRIVER_STRENGTH_GD:
            user_driver_strength = user_driver_strength % sizeof(driver_strength_table_GD);
            device_driver_strength = driver_strength_table_GD[user_driver_strength];
            break;
        case SPI_NAND_DRIVER_STRENGTH_ESMT:
            user_driver_strength = user_driver_strength % sizeof(driver_strength_table_ESMT);
            device_driver_strength = driver_strength_table_ESMT[user_driver_strength];
            break;
        default:
			DBG_PRINT("unknown driver_strength_type %u, %u!\r\n", driver_strength_type, user_driver_strength);
			break;
    }

    return device_driver_strength;
}

#endif  // USE_NANDSPI_INSTEAD_NANDFLASH
