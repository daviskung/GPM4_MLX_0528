/************************************************************
* gpbt03_demo.c
*
* Purpose: Command and data stream via GSPI demo code
*
* Author: Ming Lee
*
* Date: 2017/04/06
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.0
* History :
*
************************************************************/


#include "GPBT03_Drv.h"
#include "SPI_Code.h"
#include "drv_l1_gpio.h"


extern unsigned char R_GetTxEndCnt;

const unsigned char T_DeviceName[] = "GP-BLE_woRsp";
const unsigned char D_DeviceNameSize = 12;
const INT16U T_CnnIntv2[] = {0x0008, 0x0010, 0x0258};
unsigned char R_SetIntervalFlag;
unsigned char R_IntervalValue;
unsigned int R_RxPackageCnt;
unsigned int R_TxPackageCnt;
unsigned char R_BLE_TestBuffer[20];
INT32U R_RWUserE2PTest[5];
void gpbt03_app_demo(void)
{
	unsigned char ret,i;

    DBG_PRINT("/**********************************************/\r\n");
    DBG_PRINT("/* Please reference to gpbt03_iodef.h, then setting boardconfig */\r\n");
    DBG_PRINT("/* ELE is only searched by APP */\r\n");
    DBG_PRINT("/**********************************************/\r\n");
	F_SPI_Open();
	R_IntervalValue = 19;
	F_GPBT03Drv_Initial();
	R_RWUserE2PTest[0] = 0x00010203;
	R_RWUserE2PTest[1] = 0x04050607;
	R_RWUserE2PTest[2] = 0x08090a0b;
	R_RWUserE2PTest[3] = 0x0c0d0e0f;
	R_RWUserE2PTest[4] = 0x55AA55AA;
	R_SetIntervalFlag = 0;
	R_RxPackageCnt = 0;
	R_TxPackageCnt = 0;
	R_GPBT03_DataPtr = (unsigned int) &T_DeviceName;
	F_GPBT03_SetDeviceName(D_DeviceNameSize);
	F_GPBT03_AdvStart(1);
	while(1)
	{
		F_GPBT03_Service();
		if(R_GPBT03_GetDataType)
		{
			switch(R_GPBT03_GetDataType)
			{
				case D_ConnectionType:
                    if(R_GPBT03_ConnectStatus == 0)
                        R_SetIntervalFlag = 0;			//If BLE disconnect, clear Interval Setting
					break;

				case D_TxDataType:
                    if(R_GPBT03_TxStatus != D_TxFinish)
						while(1);				// Tx Data Fail, refer to Tx Data Status
					break;

				case D_IntvType:
                    if(R_GPBT03_SetIntvStatus == D_SetIntvPass)
						F_GPBT03_ReadCtrlInfo(D_Ctrl_CnnIntv);		//If setup Interval Pass, Read Final interval value
					else
						while(1);				//Set Interval Fail, refer to Interval Setting Status
					break;

				case D_CtrlInfoType:
                    // Return Ctrl OP code:	R_CtrlInfoCode
					// Ctrl Info Length:R_CtrlInfoLen (unit: byte)
					// Ctrl Info Data: R_RxDataBuffer[]
					if(R_CtrlInfoCode == D_Ctrl_CnnIntv)
					{
						//When R_GPBT03_SetIntvStatus = D_SetIntvPass
						if(R_RxDataBuffer[0] > R_IntervalValue)		//If return interval > request interval --> not final interval
							F_GPBT03_ReadCtrlInfo(D_Ctrl_CnnIntv);	//Read interval again
						else
							R_IntervalValue = R_RxDataBuffer[0];	//get final interval value
					}
					break;

				case D_RxDataType:
                    // Rx Data Length:	R_GPBT03_RxDataLen (unit: byte)
					// Rx Data:			R_RxDataBuffer[]
					R_RxPackageCnt= R_RxPackageCnt+1;
					ret = R_RxDataBuffer[0];
					break;

				case D_GetVersion:
					ret = R_RxDataBuffer[0];
					break;

				case D_CtrlStatusType:
                    // Ctrl OP Code: R_RxDataBuffer[0]
					// Ctrl Status: R_RxDataBuffer[1]
					ret = R_RxDataBuffer[0];
					ret = R_RxDataBuffer[1];
					break;

				case D_PacketFormatErr:
                    while(1);		//Packet format fail
					break;

				case D_GetUserE2PData:
					// Read Address:R_UserE2PAddr
					// Read Size:	R_GPBT03_RxDataLen (unit: long)
					// Read Data:	R_RxDataBuffer[]
					ret = R_UserE2PAddr;
					ret = R_GPBT03_RxDataLen;
					ret = R_RxDataBuffer[0];
				break;

				case D_InitialSucess:
					R_SetIntervalFlag = 0;
					R_RxPackageCnt = 0;
					R_TxPackageCnt = 0;
					R_GPBT03_DataPtr = (unsigned int) &T_DeviceName;
					F_GPBT03_SetDeviceName(D_DeviceNameSize);
					F_GPBT03_SetTxPower(D_TxPower0dB);
					F_GPBT03_SetAdvIntv(20);
					F_GPBT03_AdvStart(1);
				break;

				case D_RetError:
					ret = R_CtrlInfoCode;	//Return Data Fail
					while(1);
				break;

				case D_InitialFail:
					F_GPBT03Drv_Initial();
				break;

			}
			R_GPBT03_GetDataType = 0;
		}
		if(R_GPBT03_ConnectStatus)
		{
			if(R_SetIntervalFlag == 0)
			{
				//Set Adv interval Test
				F_GPBT03_SetCnnIntv(R_IntervalValue);
				R_SetIntervalFlag = 1;
				/*// F_GPBT03_SetCnnIntv2 example	Note: iOS only support CnnIntv2
				R_GPBT03_DataPtr = (unsigned int) &T_CnnIntv2;
				F_GPBT03_SetCnnIntv2();
				*/
			}
			if(R_SetIntervalFlag == 2)
			{
				F_GPBT03_ReadCtrlInfo(D_Ctrl_CnnIntv);
				R_SetIntervalFlag = 3;
			}

			if((R_GPBT03_TxStatus == 0) && (R_IndicateEnFlag == 1))
			{
				R_BLE_TestBuffer[13] = (char) R_RxPackageCnt;
				R_BLE_TestBuffer[12] = (char) (R_RxPackageCnt>>8);
				R_BLE_TestBuffer[19] = (char) R_TxPackageCnt;
				R_BLE_TestBuffer[18] = (char) (R_TxPackageCnt>>8);
				R_GPBT03_DataPtr = (unsigned int) &R_BLE_TestBuffer;
				F_GPBT03_TxData(20);
				R_TxPackageCnt = R_TxPackageCnt + 1;
			}
			if(R_IndicateEnFlag == 0)
			{
				if(F_IsTxIndicateEnable())
					R_IndicateEnFlag = 1;
			}

		}
        osDelay(2);
	}
}

void F_UserWriteE2P(void)
{
	R_GPBT03_DataPtr = (unsigned int) &R_RWUserE2PTest;
	R_UserE2PAddr = 1;
	F_GPBT03_UserE2PWrite(5);
}

void F_UserReadE2P(void)
{
	R_UserE2PAddr = 1;
	F_GPBT03_UserE2PRead(5);
}

void F_GPBTM03_EnterPDnMode(void)
{
	F_SPI_Close();
// Set SPICS, SPICLK, MOSI, MISO to input with RPL
/* ming map to GPM4
	P_IO_PortB_Dir = P_IO_PortB_Dir & ~0x0F;
	P_IO_PortB_Attrib = P_IO_PortB_Attrib & ~0x0F;
	P_IO_PortB_Buffer = P_IO_PortB_Buffer & ~0x0F;
*/
	F_GPBTM03_PDN();
}

void F_GetVersion(void)
{
	F_GPBT03_GetFwVersion();
}
/*
extern void F_GPBT03_ReadPhy(unsigned char Len);
void F_ReadPhyTest(void)
{
	F_GPBT03_WakeUp();
	R_GPBT03_PhyAddr = 0x0060310C;
	R_GPBT03_PhyAddr = 0x00603114;
	R_GPBT03_PhyAddr = 0x00603118;
	F_GPBT03_ReadPhy(1);
	F_GPBT03_WakeUpLow();
}
*/
