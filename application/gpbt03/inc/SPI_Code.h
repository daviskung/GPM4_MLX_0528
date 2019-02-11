#ifndef __SPI_CODE_H__
#define __SPI_CODE_H__

//==================================================================================
// Name                  : SPI_Code.h
// Applied Body          : GPL832X Series
// Programmer            :
// Description           :
// History version       : v1.2.1
// 1.2.1     2011/10/18  Jason     Modify for new driver definition
//==================================================================================
// =======================================================================================
// Function name : F_SPI_CS1High
// Purpose       : CS = HI
// Parameter     : none
// Return        : None
// ======================================================================================
extern void F_SPI_CS1High(void);

// =======================================================================================
// Function name : F_SPI_CS1Low
// Purpose       : CS = LOW
// Parameter     : none
// Return        : None
// ======================================================================================
extern void F_SPI_CS1Low(void);

// =======================================================================================
// Function name : F_SPI_Close
// Purpose       : SPI Disable,Hardware SPI chenange to GPIO
// Parameter     : None
// Return        : None
// ======================================================================================
extern void F_SPI_Close(void);

// =======================================================================================
// Function name : F_SPI_Open
// Purpose       : SPI Enable,Setting CS ,CLK, DI ,DO, CLOCK
// Parameter     : None
// Return        : None
// Destroy       : A
// ======================================================================================
extern void F_SPI_Open(void);

// =======================================================================================
// Function name : F_SPI_ClockSetup
// Purpose       : change SPI clock
// Parameter     : unsigned char SPICLKSel
//					   Acc = 1 is D_SPICpuClkDiv2
//					   Acc = 2 is D_SPICpuClkDiv4
//					   Acc = 3 is D_SPICpuClkDiv8
//					   Acc = 4 is D_SPICpuClkDiv16
//					   Acc = 5 is D_SPICpuClkDiv32
//					   Acc = 6 is D_SPICpuClkDiv64
//					   Acc = 7 is D_SPICpuClkDiv128
// Return        : 0: Pass
//                 1: Level error
// ======================================================================================
extern unsigned char F_SPI_ClockSetup(unsigned char SPICLKSel);

// =======================================================================================
// Function name : F_SPI_ReadOneByte
// Purpose       : SPI Read one byte
// Parameter     : unsigned char TXDATA
// Return        : 0: Complete, -1: Timeout
//                 R_SPI_ReadOneByteData = Read data
// ======================================================================================
extern char F_SPI_ReadOneByte(unsigned char TXDATA);

// =======================================================================================
// Function name : F_SPI_WriteOneByte
// Purpose       : SPI Write one byte
// Parameter     : unsigned TXDATA
// Return        : 0: Complete
//                 -1: Timeout
// ======================================================================================
extern unsigned char F_SPI_WriteOneByte(unsigned TXDATA);

// =======================================================================================
// Function name : F_SPI_ReadNByte
// Purpose       : SPI N byte read(To buffer)
// Parameter     : R_SPI_DataPtr/+1 ,R_SPI_DataLength/+1
// Return        : 0: Complete
//                 -1: Timeout
// ======================================================================================
extern unsigned char F_SPI_ReadNByte(void);

// =======================================================================================
// Function name : F_SPI_WriteNByte
// Purpose       : SPI N byte write(From buffer)
// Parameter     : R_SPI_DataPtr/+1 ,R_SPI_DataLength/+1
// Return        : 0: Complete
//                 -1: Timeout
// ======================================================================================
extern unsigned char F_SPI_WriteNByte(void);

// =======================================================================================
// Function name : F_SPI_DMAStart
// Purpose       : SPIDMA Start
// Parameter     : unsigned char DataLength
//						If SPIDMABuffer Size is 64-byte Mode, Max DataLength is 64
//						If SPIDMABuffer Size is 128-byte Mode, Max DataLength is 128
// Return        : None
// ======================================================================================
extern void F_SPI_DMAStart(unsigned char DataLength);

// =======================================================================================
// Function name : F_SPI_DMADoneCheck
// Purpose       : SPIDMA Start
// Parameter     : None
// Return        : others: DMA Transfer Completed, 0: DAM is Transferring
// ======================================================================================
extern unsigned char F_SPI_DMADoneCheck(void);

// =======================================================================================
// Function name : F_SPI_FIFOINTDis
// Purpose       : SPI RX interrupt disable
// Parameter     : None
// Return        : None
// ======================================================================================
extern void F_SPI_FIFOINTDis(void);

// =======================================================================================
// Function name : F_SPI_DMAINTEn
// Purpose       : SPIRXDMA interrupt enable
// Parameter     : None
// Return        : None
// ======================================================================================
extern void F_SPI_DMAINTEn(void);

// =======================================================================================
// Function name : F_SPI_DMAINTDis
// Purpose       : SPIRXDMA interrupt disable
// Parameter     : None
// Return        : None
// ======================================================================================
extern void F_SPI_DMAINTDis(void);

// ======================================================================================
extern unsigned int R_SPI_DataLength;
extern unsigned int R_SPI_DataPtr;
extern unsigned char R_SPI_ReadOneByteData;

#endif //__SPI_CODE_H__
