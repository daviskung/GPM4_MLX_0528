#ifndef __GPBT03_DRV_H__
#define __GPBT03_DRV_H__

//======================================================================================
//Function name : F_GPBT03Drv_Initial
//Purpose       : Initial GPBT03 Control IO configuration, Download Fw Code, Initial Driver
//Parameter     : None
//Return        : None
//======================================================================================
extern void F_GPBT03Drv_Initial(void);

// ======================================================================================
// Function name : F_GPBTM03_PDN
// Purpose       : Set GPBTM03 enter Power down mode. Users need to set all SPI IOs to
//				   input with pull-low status before call this function
// Parameter     : None
// Return        : None
// ======================================================================================
extern void F_GPBTM03_PDN(void);

// ======================================================================================
// Function name : F_GPBT03_GetFwVersion
// Purpose       : Read Driver Fw Version
// Parameter     : None
// Return        : R_GPBT03_GetDataType = D_GetVersion
//				   ((long*) &R_RxDataBuffer)[0] = Version,
//					Ex: V1.0.0.2 --> 0x01000002
// ======================================================================================
extern void F_GPBT03_GetFwVersion(void);

//======================================================================================
//Function name : F_GPBT03_Service
//Purpose       : BLE Service Loop
//Parameter     : None
//Return        : R_GPBT03_GetDataType: Refer to "Get Data Type"
//======================================================================================
extern void F_GPBT03_Service(void);

//======================================================================================
//Function name : F_GPBT03_SetDeviceName
//Purpose       : Set Device Name
//Parameter     : Len: Name Length (byte)
//				  R_GPBT03_DataPtr->DeviceName[n<=16], ASCII format
//Return        : None
//======================================================================================
extern void F_GPBT03_SetDeviceName(unsigned char Len);

//======================================================================================
//Function name : F_GPBT03_SetAdvIntv
//Purpose       : Set BLE Adv Interval Time
//Parameter     : AdvIntv: Advertising Interval
//				  Range: 20 ~ 10240 (unit: ms)
//Return        : None
//======================================================================================
extern void F_GPBT03_SetAdvIntv(short int AdvIntv);

//======================================================================================
//Function name : F_GPBT03_SetAdvData
//Purpose       : Set BLE Adv Data
//Parameter     : R_GPBT03_DataPtr->AdvData[n<=25], AdvData[0]: LSB, AdvData[n]: MSB
//Return        : 0xFF: Len>25
//				  0: Sucess
//======================================================================================
extern char F_GPBT03_SetAdvData(char Len);

// ======================================================================================
// Function name : F_GPBT03_SetTxPower
// Purpose       : Set Device RF Tx Power
// Parameter     : PWIndex= 0: 3dB, 1: 0dB, 2: -6dB, 3: -12dB, 4: -18dB, default is 0dB
// Return        : 0x00: Set Pass
//				   0xFF: Set Fail (invalid value)
// ======================================================================================
extern char F_GPBT03_SetTxPower(unsigned char TxPwIndex);

//======================================================================================
//Function name : F_GPBT03_AdvStart
//Purpose       : BLE Adv Enable/Disable
//Parameter     : Ctrl= 0:Disable, 1:Enable
//Return        : None
//======================================================================================
extern void F_GPBT03_AdvStart(unsigned char Ctrl);

// ======================================================================================
// Function name : F_GPBT03_Disconnect
// Purpose       : BLE Disconnect
// Parameter     : None
// Return        : None
// ======================================================================================
extern void F_GPBT03_Disconnect(void);

// ======================================================================================
// Function name : F_IsTxIndicateEnable
// Purpose       : Check BLE Tx Indicate
// Parameter     : None
// Return        : 1: Indicate is enable, 0: Indicate is disable
// ======================================================================================
extern char F_IsTxIndicateEnable(void);

//======================================================================================
//Function name : F_GPBT03_TxData
//Purpose       : BLE Send Data
//Parameter     : Len=Data Length<20bytes
//				  R_GPBT03_DataPtr->TxBuffer
//Return        : None
//======================================================================================
extern void F_GPBT03_TxData(unsigned char Len);

//======================================================================================
//Function name : F_GPBT03_ReadCtrlInfo
//Purpose       : Read Ctrl Information
//Parameter     : A=CtrlType, Refer to CtrlTypeList
//Return        : None
//======================================================================================
extern void F_GPBT03_ReadCtrlInfo(unsigned char CtrlType);

// ======================================================================================
// Function name : F_GPBT03_SetCnnIntv
// Purpose       : Set BLE Interval Time
// Parameter     : CnnIntv: Connect Interval
//				   Ragnge: 8 ~ 4000, Unit: ms
// Return        : R_GPBT03_SetIntvStatus
//				   0xFF: CnnIntv>4000
//				   0: Sucess
// ======================================================================================
extern void F_GPBT03_SetCnnIntv(short int CnnIntv);

// ======================================================================================
// Function name : F_GPBT03_SetCnnIntv2
// Purpose       : BLE Disconnect
// Parameter     : R_GPBT03_DataPtr->&TimeInfo
//				   int TimeInfo[3] = {MinInterval, MaxInterval, TimeOut};
//				   MinInterval: MinInterval
//				   MaxInterval: MaxInterval
//						Unit: 1.25ms, Ragnge: 0x0006 ~ 0x0C80
//				   TimeOut: Signal Lost Timeout Time
//						Unit: 10ms, Range: 0x0A ~ 0xC80
// Return        : R_GPBT03_SetIntvStatus
// ======================================================================================
extern void F_GPBT03_SetCnnIntv2(void);

// ======================================================================================
// Function name : F_GPBT03_UserE2PWrite
// Purpose       : Write data to EEPROM user area
// Parameter     : R_UserE2PAddr: 0 ~ 31
//				   R_GPBT03_DataPtr
//				   Size: Write Data size, Unit: long
// Return        : 0x00: Write Success
//				   0xFF: Address Over 32
//				   0xFE: Write Size + R_UserE2PAddr Over 32
//				   0xFD: Write Size > 5
// ======================================================================================
extern char F_GPBT03_UserE2PWrite(unsigned char Size);

// ======================================================================================
// Function name : F_GPBT03_UserE2PRead
// Purpose       : Check BLE Tx Indicate
// Parameter     : R_UserE2PAddr: 0 ~ 31
//				   R_GPBT03_DataPtr
//				   Size: Read Data size, Unit: long
// Return        : 0x00: Write Success
//				   0xFF: Address Over 32
//				   0xFE: Read Size + R_UserE2PAddr Over 32
//				   0xFD: Read Size > 5
// ======================================================================================
extern char F_GPBT03_UserE2PRead(unsigned char Size);

//====================================================================
//Variable name : R_GPBT03_DataPtr
//Purpose		: Tx Data Buffer Pointer
//Option		: Please refer each Tx/Setting function Parameter
//====================================================================
extern unsigned int R_GPBT03_DataPtr;

//====================================================================
//Variable name : R_IndicateEnFlag
//Purpose		: Tx Indicate Enable Flag
//Option		: Only host indicated enabled, Tx data can be sent
//====================================================================
extern unsigned char R_IndicateEnFlag;

//====================================================================
//Variable name : R_GPBT03_GetDataType
//Purpose		: Record get data type
//Valid			: After F_GPBT03_Service serviced
//Option		: Set by Driver, Clear by user
//====================================================================
extern unsigned char R_GPBT03_GetDataType;			//Set by Driver, Clear by User
//======================================================================================
// Get Data Type
//======================================================================================
	#define D_ConnectionType		1
	#define D_TxDataType			2
	#define D_IntvType				3
	#define D_CtrlInfoType			4
	#define D_RxDataType			6
	#define D_GetVersion			7
	#define	D_CtrlStatusType		8
	#define D_PacketFormatErr		9
	#define D_GetUserE2PData		10
	#define D_InitialSucess			11
	#define D_InitialFail			0xFE
	#define D_RetError				0xFF

//====================================================================
//Variable name : R_GPBT03_ConnectStatus
//Purpose		: Record Connection status
//Valid			: Changed when R_GPBT03_GetDataType = D_ConnectionType (1)
//Option		: Set/Clear by Driver
//====================================================================
extern unsigned char R_GPBT03_ConnectStatus;	//Set and Clear by Driver
//======================================================================================
// Connection Status
//======================================================================================
	#define D_NoConnection		0x00
	#define D_ConnectionOk		0x01

//====================================================================
//Variable name : R_GPBT03_ConnectStatus
//Purpose		: Record Tx Data status
//Valid			: Changed when R_GPBT03_GetDataType = D_TxDataType (2)
//Option		: Set by Driver, user must check this status before Tx new data
//====================================================================
extern unsigned char R_GPBT03_TxStatus;
//======================================================================================
// Tx Data Status
//======================================================================================
	#define D_TxFinish				0x00
	#define D_TxBusy				0x01
	#define D_BLENoConnect			0x01*2
	#define D_BLENotifyDisable		0x02*2
	#define D_BLEDisConnect			0x03*2
	#define D_BusyReject			0x04*2
	#define D_TxBufferFull			0x05*2

//====================================================================
//Variable name : R_GPBT03_SetIntvStatus
//Purpose		: Record Set Interval Data status
//Valid			: Changed when R_GPBT03_GetDataType = D_IntvType (3)
//Option		: Set by Driver
//====================================================================
extern unsigned char R_GPBT03_SetIntvStatus;	//Data is valid when R_GPBT03_GetDataType=3, Set by Driver
//======================================================================================
// Interval Setting Status
//======================================================================================
	#define D_SetIntvPass		0x01
	#define D_SendIntvFail		0x40
	#define D_IntvValueIllegal	0x80

//====================================================================
//Variable name : R_CtrlInfoLen
//Purpose		: Record Control information Data Size
//Valid			: When R_GPBT03_GetDataType = D_CtrlInfoType (4)
//Option		: Set by Driver
//Unit			: Byte
//====================================================================
extern unsigned char R_CtrlInfoLen;

//====================================================================
//Variable name : R_CtrlInfoCode
//Purpose		: Record Control information Type
//Valid			: When R_GPBT03_GetDataType = D_CtrlInfoType (4)
//Option		: Set by Driver
//====================================================================
extern unsigned char R_CtrlInfoCode;
//======================================================================================
// CtrlTypeList
//======================================================================================
	#define D_Ctrl_CnnIntv		0x30		//CnnIntv: Relative: F_GPBT03_SetCnnIntv & F_GPBT03_SetCnnIntv2
	#define D_Ctrl_Rename		0x31		//Rename: Relative: F_GPBT03_SetDeviceName
	#define D_Ctrl_AdvIntv		0x35		//AdvIntv: Relative: F_GPBT03_SetAdvIntv
	#define D_Ctrl_AdvData		0x36		//AdvData: Relative: F_GPBT03_SetAdvData
	#define	D_Ctrl_TxPower		0x38		//TxPower: Relative: F_GPBT03_SetTxPower
	#define	D_Ctrl_AdvState		0x3E		//AdvState: Relative: F_GPBT03_AdvStart
	#define	D_Ctrl_DisCnn		0x5F		//DisCnn: Relative: F_GPBT03_Disconnect
//====================================================================
//Variable name : R_GPBT03_RxDataLen
//Purpose		: Record Get Rx Data Size
//Valid			: When R_GPBT03_GetDataType = D_RxPhyDataType (5) or D_RxDataType(6)
//Option		: Set by Driver
//Unit			: Byte, when R_GPBT03_GetDataType = D_RxDataType (6)
//				  Long, when R_GPBT03_GetDataType = D_RxPhyDataType (5)
//====================================================================
extern unsigned char R_GPBT03_RxDataLen;

//====================================================================
//Variable name : R_RxDataBuffer[]
//Purpose		: Record Get Rx Data
//Valid			: When R_GPBT03_GetDataType = D_RxPhyDataType (5) or D_RxDataType(6)
//Option		: Set by Driver
//====================================================================
extern unsigned char R_RxDataBuffer[32];

//======================================================================================
//Variable name : R_UserE2PAddr
//Purpose		: Record Get User E2P Data Address
//Valid			: When R_GPBT03_GetDataType = D_GetUserE2PData (10)
//Option		: Set by Driver
//======================================================================================
extern unsigned char R_UserE2PAddr;

//======================================================================================
//Tx Power Define
//======================================================================================

#define D_TxPower3dB		0
#define D_TxPower0dB		1
#define D_TxPowerMinus6dB	2
#define D_TxPowerMinus12dB	3
#define D_TxPowerMinus18dB	4

//======================================================================================
extern unsigned int R_GPBT03_PhyAddr;
extern unsigned short int R_GPBT03Temp;

#endif // __GPBT03_DRV_H__
