
#ifndef __CSPOTTERSDK_API_H
#define __CSPOTTERSDK_API_H

#if defined(_WIN32)
	#ifdef CSpotterDll_EXPORTS
		#define CSPDLL_API __declspec(dllexport)
	#endif
#endif

#ifndef CSPDLL_API
#define CSPDLL_API
#endif

#include "base_types.h"
#include "CSpotterSDKApi_Const.h"

#ifdef __cplusplus
extern "C"{
#endif

// Purpose: Create a recognizer
// lpbyModel(IN): The command model.
// nMaxTime(IN): The maximum buffer length in number of frames for keeping the status of commands.
// lpbyMemPool(IN/OUT): Memory buffer for the recognizer.
// nMemSize(IN): Size in bytes of the memory buffer lpbyMemPool.
// lpbyState(IN/OUT): State buffer for recognizer.
// nStateSize(IN): Size in bytes of the state buffer lpbyState.
// pnErr(OUT): CSPOTTER_SUCCESS indicates success, else error code. It can be NULL.
// Return: a recognizer handle or NULL
CSPDLL_API HANDLE CSpotter_Init(BYTE *lpbyModel, INT nMaxTime, BYTE *lpbyMemPool, INT nMemSize, BYTE *lpbyState, INT nStateSize, INT *pnErr);

// lpbyCYBase(IN): The background model, contents of CYBase.mod.
// lppbyModel(IN): The command model.
// nMaxTime(IN): The maximum buffer length in number of frames for keeping the status of commands.
// lpbyMemPool(IN/OUT): Memory buffer for the recognizer.
// nMemSize(IN): Size in bytes of the memory buffer lpbyMemPool.
// lpbyState(IN/OUT): State buffer for recognizer.
// nStateSize(IN): Size in bytes of the state buffer lpbyState.
// pnErr(OUT): CSPOTTER_SUCCESS indicates success, else error code. It can be NULL.
// Return: a recognizer handle or NULL
CSPDLL_API HANDLE CSpotter_Init_Sep(BYTE *lpbyCYBase, BYTE *lpbyModel, INT nMaxTime, BYTE *lpbyMemPool, INT nMemSize, BYTE *lpbyState, INT nStateSize, INT *pnErr);
CSPDLL_API HANDLE CSpotter_Init_Multi(BYTE *lpbyCYBase, BYTE *lppbyModel[], INT nNumModel, INT nMaxTime, BYTE *lpbyMemPool, INT nMemSize, BYTE *lpbyState, INT nStateSize, INT *pnErr);

// Purpose: Destroy a recognizer (free resources)
// hCSpotter(IN): a handle of the recognizer
// Return: Success or error code
CSPDLL_API INT CSpotter_Release(HANDLE hCSpotter);

// Purpose: Reset recognizer
// hCSpotter(IN): a handle of the recognizer
// Return: Success or error code
CSPDLL_API INT CSpotter_Reset(HANDLE hCSpotter);

// Purpose: Transfer voice samples to the recognizer for recognizing.
// hCSpotter(IN): a handle of the recognizer
// lpsSample(IN): the pointer of voice data buffer
// nNumSample(IN): the number of voice data (a unit is a short, we prefer to add 160 samples per call)
// Return: "CSPOTTER_ERR_NeedMoreSample" indicates call this function again, else call CSpotter_GetResult(...)
CSPDLL_API INT CSpotter_AddSample(HANDLE hCSpotter, SHORT *lpsSample, INT nNumSample);

// Purpose: Get recognition results.
// hCSpotter(IN): a handle of the recognizer
// Return: the command ID. It is 0 based
CSPDLL_API INT CSpotter_GetResult(HANDLE hCSpotter);
CSPDLL_API INT CSpotter_GetResultEPD(HANDLE hCSpotter, INT *pnWordDura, INT *pnEndDelay);
CSPDLL_API INT CSpotter_GetResultScore(HANDLE hCSpotter);

CSPDLL_API INT CSpotter_GetSpeakerResult(HANDLE hCSpotter);

CSPDLL_API INT CSpotter_GetNumWord(BYTE *lpbyModel);

CSPDLL_API INT CSpotter_IsActiveCmd(BYTE *lpbyModel, INT nCmdIdx);

CSPDLL_API INT CSpotter_GetMemoryUsage(BYTE *lpbyModel, INT nMaxTime);
CSPDLL_API INT CSpotter_GetMemoryUsage_Sep(BYTE *lpbyCYBase, BYTE *lpbyModel, INT nMaxTime);
CSPDLL_API INT CSpotter_GetMemoryUsage_Multi(BYTE *lpbyCYBase, BYTE *lppbyModel[], INT nNumModel, INT nMaxTime);

CSPDLL_API INT CSpotter_GetStateSize(BYTE *lpbyModel);

/************************************************************************/
//  Threshold Adjust API                                                                   
/************************************************************************/
// Purpose: Set model rejection level
// hCSpotter(IN): a handle of the recognizer
// nRejectionLevel(IN): rejection level
// Return: Success or error code
CSPDLL_API INT CSpotter_SetRejectionLevel(HANDLE hCSpotter, INT nRejectionLevel);

// Purpose: Set engine response time
// hCSpotter(IN): a handle of the recognizer
// nResponseTime(IN): response time
// Return: Success or error code
CSPDLL_API INT CSpotter_SetResponseTime(HANDLE hCSpotter, INT nResponseTime);

// Purpose: Set Cmd reward
// hCSpotter(IN): a handle of the recognizer
// nCmdIdx(IN): the command ID. It is 0 based
// nReward(IN): the reward
// Return: Success or error code
CSPDLL_API INT CSpotter_SetCmdReward(HANDLE hCSpotter, INT nCmdIdx, INT nReward);

// Purpose: Set Cmd response reward
// hCSpotter(IN): a handle of the recognizer
// nCmdIdx(IN): the command ID. It is 0 based
// nReward(IN): the reward
// Return: Success or error code
CSPDLL_API INT CSpotter_SetCmdResponseReward(HANDLE hCSpotter, INT nCmdIdx, INT nReward);

CSPDLL_API INT CSpotter_SetEndStateRange(HANDLE hCSpotter, INT nEndStateRange);

CSPDLL_API INT CSpotter_SetFreqWarpFactor(HANDLE hCSpotter, INT nFactor256);

CSPDLL_API INT CSpotter_SetMinEnergyThreshd(HANDLE hCSpotter, INT nThreshold);

CSPDLL_API INT CSpotter_SetSpeakerIdentLevel(HANDLE hCSpotter, INT nIdentLevel);

CSPDLL_API INT CSpotter_SetEnableRejectedResult(HANDLE hCSpotter, INT bEnable);
CSPDLL_API INT CSpotter_SetRejectedResultEnergyThreshd(HANDLE hCSpotter, INT nThreshold);

CSPDLL_API INT CSpotter_VAD_Enable(HANDLE hCSpotter, INT bEnable);
CSPDLL_API INT CSpotter_VAD_SetThreshold(HANDLE hCSpotter, INT nThreshold);

#ifdef __cplusplus
}
#endif

#endif // __CSPOTTERSDK_API_H
