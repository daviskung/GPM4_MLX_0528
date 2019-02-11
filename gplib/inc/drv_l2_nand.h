#ifndef __DRV_L2_NAND_H
#define __DRV_L2_NAND_H

#include "project.h"

#if !defined(_NF_APP_PART_INFO)
#define _NF_APP_PART_INFO
typedef struct
{
	INT32U part_size; //sector
	INT32U checkSum;
	INT32U startSector; //sector
	INT32U imageSize; //sector
	INT32U destAddress;
	INT8U  type;    // NF_APP_KIND
	INT8U  version;
}NF_APP_PART_INFO;
#endif

#ifndef _NF_APP_KIND
#define _NF_APP_KIND
typedef enum {
    APP_RUNTIME_CODE   	=   0,
    APP_RESOURCE     	=   1,
    APP_FASTBOOT_BIN 	=   2,
    APP_QUICK_IMAGE     =   3,
    APP_HIBRATE_IMAGE   =   4,
    APP_IMAGE_FLAG   	=   5,
    APP_SECOND_BTLDR   	=   6,
    APP_KIND_MIN		= APP_RUNTIME_CODE,
    APP_KIND_MAX		= APP_SECOND_BTLDR
}NF_APP_KIND;
#endif

#ifndef _NFIO_CFG
#define _NFIO_CFG
typedef enum {
    NAND_IO_IOB       =0x00,
    NAND_IO_IOC		  =0x01,
    NAND_IO_AUTO_SCAN =0xFF
} NFIO_CFG;
#endif

extern void Nand_OS_LOCK(void); // undocumented, advance user only
extern void Nand_OS_UNLOCK(void); // undocumented, advance user only

extern INT32U Nand_part0_size_Get(void);
extern INT32U NandAppByteSizeGet(void); // undocumented, advance user only
extern INT32U NandBootAreaByteSizeGet(void); // undocumented, advance user only
//extern INT32S CalculateFATArea(void); // obsolete
extern INT16U GetBadFlagFromNand(INT16U wPhysicBlkNum); // undocumented, advance user only
extern INT32U nand_block_checkbad(INT32U block_id, INT8U* data_buf); // undocumented, advance user only

//APP-Zone API
/*
* Function Name :  NandBootEnableWrite
*
* Syntax :   void   NandBootEnableWrite(void);
*
* Purpose :    to enable the wirte fuction of App area.
*
* Parameters : <IN> none
*                    <OUT> none
* Return :   - None
*
* Note :  After this function is called, erase/write function is allow in App area.
*
*/
extern void   NandBootEnableWrite(void);
/*
* Function Name :  NandBootDisableWrite
*
* Syntax :   void   NandBootDisableWrite(void);
*
* Purpose :    to disable the wirte fuction of App area.
*
* Parameters : <IN> none
*                    <OUT> none
* Return :   - None
*
* Note :  After this function is called, erase/write function is not allowed in App area unless NandBootEnableWrite() is called.
*
*/
extern void   NandBootDisableWrite(void);
/*
* Function Name :  NandBootFormat
*
* Syntax :  INT32U NandBootFormat(INT16U format_type);
*
* Purpose :   to  format the Nand flash.
*
* Parameters : <IN> format_type
*                           0x01: Erase the blocks except original bad blocks and user-defined bad blocks
*                           0x10: Erase the blocks except original bad blocks
*                   <OUT> none
* Return :   0: Success
*               Others: Fail
*
* Note :  This function can be used only for App area. Before calling this function,
*		user must call NandBootEnableWrite() first to set App are as writeable.
*
*/
extern INT32U NandBootFormat(INT16U format_type);
/*
* Function Name :  NandBootFlush
*
* Syntax : INT32U NandBootFlush(void);
*
* Purpose :   to make sure the related information is updated and stored in Nand.
*
* Parameters : <IN> void
*                   <OUT> none
* Return :   0: Success
*               Others: Fail
*
* Note : Before stopping writing App area, disabling Nand flash or turning off the power,
*	     user must call this function to prevent from losing some information.
*
*/
extern INT32U NandBootFlush(void);
/*
* Function Name :  NandBootInit
*
* Syntax : INT32U NandBootInit(void);
*
* Purpose :   to initialize Nand flash app area.
*
* Parameters : <IN> void
*                   <OUT> none
* Return :    0: Success. maptable is found.
*		   -1: Fail. maptable is no found.
*
* Note : This function can be used only for App area. If the maptable is not found,
*           user has to call NandBootFormat() to generate maptable and then call this function again.
*
*/
extern INT32U NandBootInit(void);
/*
* Function Name :  NandBootWriteSector
*
* Syntax : INT32U NandBootWriteSector(INT32U wWriteLBA, INT16U wLen, INT32U DataBufAddr);
*
* Purpose :   to write App area.
*
* Parameters : <IN> wWriteLBA: start address, it's present by logical sector
*                              wLen: write sector number
*                              DataBufAddr: target address in working memory
*                   <OUT> none
* Return :   0: Success
*               Others: Fail
*
* Note :
*
*/
extern INT32U NandBootWriteSector(INT32U wWriteLBA, INT16U wLen, INT32U DataBufAddr);
/*
* Function Name :  NandBootReadSector
*
* Syntax : INT32U NandBootReadSector(INT32U wReadLBA, INT16U wLen, INT32U DataBufAddr);
*
* Purpose :   to write App area.
*
* Parameters : <IN> wReadLBA: start address, it's present by logical sector
*                              wLen: read sector number
*                              DataBufAddr: target address in working memory
*                   <OUT> none
* Return :   0: Success
*               Others: Fail
*
* Note :
*
*/
extern INT32U NandBootReadSector(INT32U wReadLBA, INT16U wLen, INT32U DataBufAddr);

//Data-Zone API
/*
* Function Name :  DrvNand_initial
*
* Syntax : INT16U DrvNand_initial(void);
*
* Purpose :  build maptab & bank infomation.
*
* Parameters : <IN> void
*              <OUT> none
* Return :
*             - NAND_OK  if successful;
*             NF_UNKNOW_TYPE if nand initial fail or page size bigger than driver support
*			  NF_NO_BUFFERPTR if no buffer was allocated.
*			  NF_FORMAT_POWER_LOSE if lose power when format data area
*			  NF_BCH_MODE_N_MATCH if read bank info fail with all bch mode.
* Note :
*
*/
extern INT16U DrvNand_initial(void);
extern INT16U DrvNand_UnInitial(void);

/*
* Function Name :  DrvNand_lowlevelformat
*
* Syntax : INT16U DrvNand_lowlevelformat(void);
*
* Purpose :  to format nand data area with low level.
*
* Parameters : <IN> void
*
*              <OUT> none
* Return :     NF_OK if successful;
*              NF_UNKNOW_TYPE if nand initial fail or page size bigger than driver support
*			   NF_NO_BUFFERPTR if no buffer was allocated.
*			   NF_NAND_PROTECTED if it is set write-protected attribute when erase nand data area
* Note :
*
*/
extern INT16U DrvNand_lowlevelformat(void);
/*
* Function Name :  DrvNand_get_Size
*
* Syntax : INT32U DrvNand_get_Size(void);
*
* Purpose :  to get ths size of Data area.
*
* Parameters : <IN> void
*                     <OUT> none
*
* Return :     -Size of Data area. The unit is sector(512 bytes).
*
* Note :
*
*/
extern INT32U DrvNand_get_Size(void);
/*
* Function Name :  DrvNand_write_sector
*
* Syntax : INT16U DrvNand_write_sector(INT32U wWriteLBA, INT16U wLen, INT32U DataBufAddr)
*
* Purpose :  to write data to nand data area with sectors..
*
* Parameters : <IN> wWriteLBA:start sector address to write data area.
*                   wLen: data length to read , unit: sector
*                   DataBufAddr: buffer used to store read back data from nand.
*              <OUT> none
* Return :     NF_OK if successful;
*              0xffff if nand data area has not been initilized or current nvram type is nand rom.
*			   NF_DMA_ALIGN_ADDR_NEED if dma address is not aligned.
*              NF_BEYOND_NAND_SIZE if address to write is overlapped the nand actual size of data area.
* Note :
*
*/
extern INT16U DrvNand_write_sector(INT32U , INT16U , INT32U );
/*
* Function Name :  DrvNand_read_sector
*
* Syntax : INT16U DrvNand_read_sector(INT32U wReadLBA, INT16U wLen, INT32U DataBufAddr);
*
* Purpose :  to read nand data area with sectors..
*
* Parameters : <IN> wReadLBA:start sector address to read data area.
*                   wLen: data length to read , unit: sector
*                   DataBufAddr: buffer used to store read back data from nand.
*              <OUT> none
* Return :    -NF_OK if successful;
*                0xffff if nand data area has not been initilized or current nvram type is nand rom.
*			   NF_DMA_ALIGN_ADDR_NEED if dma address is not aligned.
*              NF_BEYOND_NAND_SIZE if address to read is overlapped the nand actual size of data area.
* Note :
*
*/
extern INT16U DrvNand_read_sector(INT32U , INT16U , INT32U );
/*
* Function Name :  DrvNand_flush_allblk
*
* Syntax : INT16U	DrvNand_flush_allblk(void);
*
* Purpose :  to flush all block after writing data area.
*
* Parameters : <IN> void
*
*              <OUT> none
* Return :     0x00 if successful;
*              0xffff if nand data area has not been initilized or current nvram type is nand rom.
*
* Note :
*
*/
extern INT16U DrvNand_flush_allblk(void);
/*
* Function Name :  nand_part0_para_set
*
* Syntax : void nand_part0_para_set(INT32U offset, INT32U size, INT16U mode);
*
* Purpose :  to set the configuration of Part0 of Data area which can be.
*
* Parameters : <IN> offset: start address
*                   		   size: size of Part0
*				  mode: 0x0: Not readable & not writeable
*						0x1: readable
*						0x2: writeable
*						0x3: readable & writeable
*                   <OUT> none
* Return :     The start address is logical address. The unit of offset and size is sector(512 bytes).
*
*
* Note :
*
*/
extern void nand_part0_para_set(INT32U offset, INT32U size, INT16U mode);
extern void nand_part1_para_set(INT32U offset, INT32U size, INT16U mode);
extern void nand_part2_para_set(INT32U offset, INT32U size, INT16U mode);

extern void Nand_part0_Offset_Set(INT32U nand_fs_sector_offset);
extern void Nand_part0_mode_Set(INT16U mode);
extern void Nand_part0_size_Set(INT32U nand_fs_sector_size);
extern void Nand_part1_Offset_Set(INT32U nand_fs_sector_offset);
extern void Nand_part1_mode_Set(INT16U mode);
extern void Nand_part1_size_Set(INT32U nand_fs_sector_size);
/*
* Function Name :  Nand_part1_Offset_Get
*
* Syntax : INT32U Nand_part1_Offset_Get(void);
*
* Purpose :  to get the start address of Part1 of Data area which can be accessed by File system
*
* Parameters : <IN> void
*                   <OUT> none
* Return :     Start address of Part0
*
*
* Note : Before calling this function, NandInfoAutoGet() must be called.
*
*/
extern INT32U Nand_part1_Offset_Get(void);
/*
* Function Name :  Nand_part1_mode_Get
*
* Syntax : INT32U Nand_part1_mode_Get(void);
*
* Purpose :  to get the read/write mode of Part1 of Data area
*
* Parameters : <IN> void
*                   <OUT> none
* Return :    0x0: Not readable & not writeable
*			0x1: readable
*			0x2: writeable
*			0x3: readable & writeable
* Note : Before calling this function, NandInfoAutoGet() must be called.
*
*/
extern INT32U Nand_part1_mode_Get(void);
/*
* Function Name :  Nand_part1_size_Get
*
* Syntax : INT32U Nand_part1_size_Get(void);
*
* Purpose :  to get the size  of Part1 of Data area
*
* Parameters : <IN> void
*                   <OUT> none
* Return :    return the real size of part1 of data area. Unit:sector.

* Note : Before calling this function, NandInfoAutoGet() must be called.
*
*/
extern INT32U Nand_part1_size_Get(void);

extern INT32U Nand_part0_Offset_Get(void);
extern INT32U Nand_part0_mode_Get(void);

extern void   Nand_Partition_Num_Set(INT16U number);
/*
* Function Name :  Nand_Partition_Num_Get
*
* Syntax : INT16U Nand_Partition_Num_Get(void);
*
* Purpose :  to get the partition numbers  of Part1 of Data area
*
* Parameters : <IN> void
*                   <OUT> none
* Return :    return the real partition numbers of data area.

* Note :
*
*/
extern INT16U Nand_Partition_Num_Get(void);
/*
* Function Name :  NandInfoAutoGet
*
* Syntax : INT32S NandInfoAutoGet(void);
*
* Purpose :   to get the configuration of Nand flash
*
* Parameters : <IN> void
*                   <OUT> none
* Return :    return the Configuration of Nand flash.

* Note :
*
*/
extern INT32S NandInfoAutoGet(void);
extern INT32S NandAppInfoAutoGet(void);
extern INT32U DrvNand_bchtable_alloc(void); //not implemented, empty function
extern void DrvNand_bchtable_free(void); //not implemented, empty function

extern void FlushWorkbuffer(void);

extern void setSize4Partition(INT8U partitionIndex, INT32U sizeInSector);
extern INT32U getSize4Partition(INT8U partitionIndex);

extern void setMode4Partition(INT8U partitionIndex, INT8U mode);
extern INT8U getMode4Partition(INT8U partitionIndex);

extern INT32U nand_sector_nums_per_page_get(void);
extern INT32U nand_page_nums_per_block_get(void);

extern INT32S NandAppInitParts(INT16U *partTotal, INT32U *partTotalSector);
extern INT32S NandAppGetPartInfo(INT16U whichPart, NF_APP_PART_INFO *partInfo);
extern INT32S NandAppFindPart(INT16U index, INT8U type, NF_APP_PART_INFO *partInfo);
extern INT32U NandAppGetTotalFormatStep(void); // undocumented, advance user only
extern void nand_iopad_sel(NFIO_CFG nand_io_sel);

/* app area abnormal power off supporting functions */
// return 0, mean no space for version stamp
INT32U NandAppCheckTheImageSize4Part(INT32U partIndex, INT32U imageSizeInSec);
// return 0, mean complete bin, 1 mean broken bin, other minus value mean error */
INT32S NandAppCheckVersionString(INT32U partIndex, INT8U* sectorBuffer);
INT32S NandAppSetUpdatingFlag(INT32U partIndex, INT8U* sectorBuffer);
INT32S NandAppClearUpdatingFlag(INT32U partIndex, INT8U* sectorBuffer, INT8U* versionString);
INT32S NandAppGetSectorAddrOfVersionFlag(INT32U partIndex, INT32U *pSectorAddr);

#endif //__DRV_L2_NAND_H
