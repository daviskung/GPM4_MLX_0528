#ifndef __CSPOTTER_SDK_API_CONST_H__
#define __CSPOTTER_SDK_API_CONST_H__


#define CSPOTTER_SUCCESS						(     0 )
#define CSPOTTER_ERR_SDKError					( -2000 )
#define CSPOTTER_ERR_LexiconError				( -3000 )
#define CSPOTTER_ERR_EngineError				( -5000 )


/************************************************************************/
// Recognition type                                                                
/************************************************************************/
#define CSPOTTER_RecogType_Unknown				(0)
#define CSPOTTER_RecogType_Passed				(1)
#define CSPOTTER_RecogType_NotGoodEnough		(2)
#define CSPOTTER_RecogType_MissStartSyllalbe	(3)
#define CSPOTTER_RecogType_MissEndSyllalbe		(4)

/************************************************************************/
// SD
/************************************************************************/
#define CSPOTTERSD_DIFFCHK_TIGHT				(0)
#define CSPOTTERSD_DIFFCHK_NORMAL				(1)
#define CSPOTTERSD_DIFFCHK_LOOSE				(2)
#define CSPOTTERSD_DIFFCHK_IGNORE				(3)

#define CSPOTTERSD_SIMCHK_TIGHT					(0)
#define CSPOTTERSD_SIMCHK_NORMAL				(1)
#define CSPOTTERSD_SIMCHK_LOOSE					(2)
#define CSPOTTERSD_SIMCHK_IGNORE				(3)

#define CSPOTTERSD_REJECT_TIGHT					(0)
#define CSPOTTERSD_REJECT_NORMAL				(1)
#define CSPOTTERSD_REJECT_LOOSE					(2)
#define CSPOTTERSD_REJECT_IGNORE				(3)

/************************************************************************/
// Error code                                                                  
/************************************************************************/

#define CSPOTTER_ERR_IllegalHandle				( CSPOTTER_ERR_SDKError -   1 )
#define CSPOTTER_ERR_IllegalParam				( CSPOTTER_ERR_SDKError -   2 )
#define CSPOTTER_ERR_LeaveNoMemory				( CSPOTTER_ERR_SDKError -   3 )
#define CSPOTTER_ERR_LoadDLLFailed				( CSPOTTER_ERR_SDKError -   4 )
#define CSPOTTER_ERR_LoadModelFailed			( CSPOTTER_ERR_SDKError -   5 )
#define CSPOTTER_ERR_GetFunctionFailed			( CSPOTTER_ERR_SDKError -   6 )
#define CSPOTTER_ERR_ParseEINFailed				( CSPOTTER_ERR_SDKError -   7 )
#define CSPOTTER_ERR_OpenFileFailed				( CSPOTTER_ERR_SDKError -   8 )
#define CSPOTTER_ERR_NeedMoreSample				( CSPOTTER_ERR_SDKError -   9 )
#define CSPOTTER_ERR_Timeout					( CSPOTTER_ERR_SDKError -  10 )
#define CSPOTTER_ERR_InitWTFFailed				( CSPOTTER_ERR_SDKError -  11 )
#define CSPOTTER_ERR_AddSampleFailed			( CSPOTTER_ERR_SDKError -  12 )
#define CSPOTTER_ERR_BuildUserCommandFailed	    ( CSPOTTER_ERR_SDKError -  13 )
#define CSPOTTER_ERR_MergeUserCommandFailed	    ( CSPOTTER_ERR_SDKError -  14 )
#define CSPOTTER_ERR_IllegalUserCommandFile     ( CSPOTTER_ERR_SDKError -  15 )
#define CSPOTTER_ERR_IllegalWaveFile			( CSPOTTER_ERR_SDKError -  16 )
#define CSPOTTER_ERR_BuildCommandFailed			( CSPOTTER_ERR_SDKError -  17 )
#define CSPOTTER_ERR_InitFixNRFailed			( CSPOTTER_ERR_SDKError -  18 )
#define CSPOTTER_ERR_EXCEED_NR_BUFFER_SIZE		( CSPOTTER_ERR_SDKError -  19 )
#define CSPOTTER_ERR_Rejected				    ( CSPOTTER_ERR_SDKError -  20 )
#define CSPOTTER_ERR_NoVoiceDetect		        ( CSPOTTER_ERR_SDKError -  21 )
#define CSPOTTER_ERR_Expired					( CSPOTTER_ERR_SDKError - 100 )
#define CSPOTTER_ERR_LicenseFailed				( CSPOTTER_ERR_SDKError - 200 )

#define CSPOTTER_ERR_CreateModelFailed			( CSPOTTER_ERR_SDKError - 500 )
#define CSPOTTER_ERR_WriteFailed				( CSPOTTER_ERR_SDKError - 501 )
#define CSPOTTER_ERR_IDExists					( CSPOTTER_ERR_SDKError - 502 )
#define CSPOTTER_ERR_NotEnoughStorage			( CSPOTTER_ERR_SDKError - 503 )
#define CSPOTTER_ERR_TrainSpeakerModelFirst		( CSPOTTER_ERR_SDKError - 504 )

#endif //__CSPOTTER_SDK_API_CONST_H__

