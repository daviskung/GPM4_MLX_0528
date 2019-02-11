#ifndef __GPBT03_CDEF_H__
#define __GPBT03_CDEF_H__
//==================================================================================
//GPBT03 SPI Driver Define Area
//==================================================================================
#define D_GPBT03_WReg		0x21
#define D_GPBT03_RReg		0x01
#define D_GPBT03_WFIFO		0xA0	//F_GPBT03_WriteFIFO
#define D_GPBT03_RFIFO		0x60
#define D_RWFIFOMaxLen		32

#define P_SPI_FIFOIntLv		(0x00<<1)
	#define D_TxFIfoBitMapShift		6

#define P_SPI_Status		(0x01<<1)
	#define D_TxFIfoUnder		0x01
	#define D_TxFIfoEmpty		0x02
	#define D_RxFIfoOver		0x04
	#define D_RxFIfoOverflow	0x08
	#define D_RxFIfoNotEmpty	0x10

#define P_SPI_IntEn			(0x02<<1)
#define P_SPI_IntClear		(0x03<<1)
#define P_SPI_FIFOCnt		(0x04<<1)
#define P_GPBT03_Status		(0x05<<1)
	#define	D_ACICmdBusy			0x0001
	#define	D_ConnectFlag			0x0002
	#define	D_IndicateEnFlag		0x0004
	#define	D_RFInitialBusyFlag		0x0040
	#define	D_ClearBufferBusyFlag	0x0080
	#define	D_GPBT03SleepSetFlag		0x0400
	#define	D_GPBT03SleepFlag			0x4000
	#define	D_ResetCompeleValue		0x8040

#define P_GPBT03_Sleep		(0x07<<1)
#define P_GPBT03_WakeUp		(0x08<<1)

//==================================================================================
// ACI Tx Define
//==================================================================================
#define D_ACIData_DataPacket		0x22	//F_Mx#define D_TxData
#define D_ACITx_ReadCtrlInfo		0x20	//F_Mx#define D_ReadCtrlInfo
#define D_ACITx_CtrlCMD				0x25	//Refer to CtrlCMD Define
#define D_ACITx_WritePhy			0x55	//F_Mx#define D_WritePhy
#define D_ACITx_ReadPhy				0x56	//F_Mx#define D_ReadPhy

//CtrlCMD Define
#define D_Ctrl_CnnIntv			0x30
#define D_CnnIntv_Len			0x06
#define D_Ctrl_CnnIntv2			0x40
#define D_CnnIntv2_Len			0x08
//F_GPBT03_SetDeviceName
#define D_Ctrl_Rename			0x31
#define D_Rename_Len			0x10
//F_GPBT03_SetBtAddr
#define D_Ctrl_BtAddr			0x33
#define D_BtAddr_Len			0x06
//F_GPBT03_ResetAdv
#define D_Ctrl_ResetAdv			0x34
#define D_ResetAdv_Len			0x00
//F_GPBT03_SetAdvIntv
#define D_Ctrl_AdvIntv			0x35
#define D_AdvIntv_Len			0x04
//F_GPBT03_SetAdvData
#define D_Ctrl_AdvData			0x36
#define D_AdvData_Len			0x19
//F_GPBT03_SetTxPower
#define D_Ctrl_TxPower			0x38
#define D_TxPower_Len			0x01
//F_GPBT03_AdvStart
#define D_Ctrl_AdvStart			0x3E
#define D_AdvStart_Len			0x01
//F_GPBT03_Disconnect
#define D_Ctrl_Disconnect		0x5F
#define D_Disconnect_Len		0x00
//==================================================================================
// ACI Rx Define
//==================================================================================
#define D_ACIRx_RetStatus		0x26
	#define D_BLEStatus				0xFF
		#define D_BLE_Disconnect		0x10
		#define D_BLE_Connect			0x11
		#define D_BLE_CmdError			0x19

	#define D_TxDataCommStatus		0x22

	#define D_IntvSettingStatus		0x30
	#define D_IntvSettingStatus2	0x40
		#define D_ValueIllegal			0x12
		#define D_SetIntvStatusPass		0x00
		#define D_SetIntvStatusFail		0x01
		#define D_SetIntvStatusFail2	0x02

	#define D_DisconnectStatus		0x5F


#define D_ACIRx_RetCtrlInfo			0x21

#define D_ACIRx_RetPhyData			0x57

//R_GPBT03_GetDataType
	#define D_ConnectionType		1
	#define D_TxDataType			2
	#define D_IntvType				3
	#define D_CtrlInfoType			4
	#define D_RxPhyDataType			5
	#define D_RxDataType			6
	#define D_GetVersion			7
	#define D_CtrlStatusType		8
	#define D_PacketFormatErr		9
	#define D_GetUserE2PData		10
	#define D_InitialSucess			11
	#define D_InitialFail			0xFE
	#define D_RetError				0xFF

//R_GPBT03_ConnectStatus
	#define D_ConnectionOk			0x01

//R_GPBT03_TxStatus
	#define D_TxBusy				0x01
	#define D_BLENoConnect			0x01*2
	#define D_BLENotifyDisable		0x02*2
	#define D_BLEDisConnect			0x03*2
	#define D_BusyReject			0x04*2
	#define D_TxBufferFull			0x05*2

//R_GPBT03_SetIntvStatus
	#define D_SetIntvPass			0x01
	#define D_IntvValueIllegal		0x80
	#define D_SendIntvFail			0x40

#endif	//__GPBT03_CDEF_H__
