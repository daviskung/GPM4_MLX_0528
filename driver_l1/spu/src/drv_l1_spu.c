/*******************************************************************
	mcu_spu.c
	For GPL32300A,GPL32500A, MAX Channel num is 32
  	Edited and Created by Ray

	Support 16Bits/8Bits/ADPCM/ADPCM36 modes
	Expression/Pedal/PitchBend event included

	History:
2008-05-21:
    add void SPU_ClearADCPM36_Mode(INT8U ChannelIndex)
	to clear ADPCM Sel when Play ADPCM36.

2008-08-08:
	modify the register name, make register same as programing guide.

2008-08-11
	add SPU_PlaySFX_FixCH(), it is to play the sound effect with no-envelop

2008-08-18
	1. change SPU_PlaySFX_FixCH to SPU_PlayPCM_NoEnv_FixCH
	2. fix the volume different when use SPU_PlaySFX_FixCH and play midi
	3. add SPU_Get_BeatIRQ_Enable_Flag

2008-10-24
	1. modify the  SPU_Set_EnvelopeClock function
********************************************************************/
#include "drv_l1_clock.h"
#include "drv_l1_spu.h"

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined _DRV_L1_SPU) && (_DRV_L1_SPU == 1)                   //
//================================================================//


#define C_SPU_DRIVER_VERSION	0x20071025
#define SPU_CLOCK_SOURCE            12
#define SPU_CLOCK_ENABLE            1
#define SPU_CLOCK_DISABLE           0

#ifndef __CS_COMPILER__
const INT8U DRVL1_SPU[] = "GLB_GP-S2_0610L_SPU-L1-ADS_1.0.0";
#else
const INT8U DRVL1_SPU[] = "GLB_GP-S2_0610L_SPU-L1-CS_1.0.0";
#endif

static INT32U addr,cpu_mclk;
static INT32U spu_i2s_sampleRate = 48000; // williamyeo added

const INT32U T_BitEnable_Table[]={
	0x00000001, 0x00000002, 0x00000004, 0x00000008,
	0x00000010, 0x00000020, 0x00000040, 0x00000080,
	0x00000100, 0x00000200, 0x00000400, 0x00000800,
	0x00001000, 0x00002000, 0x00004000, 0x00008000,
	0x00010000, 0x00020000, 0x00040000, 0x00080000,
	0x00100000, 0x00200000, 0x00400000, 0x00800000,
	0x01000000, 0x02000000, 0x04000000, 0x08000000,
	0x10000000, 0x20000000, 0x40000000, 0x80000000
};

const INT32U T_BitDisable_Table[]={
	~0x00000001, ~0x00000002, ~0x00000004, ~0x00000008,
	~0x00000010, ~0x00000020, ~0x00000040, ~0x00000080,
	~0x00000100, ~0x00000200, ~0x00000400, ~0x00000800,
	~0x00001000, ~0x00002000, ~0x00004000, ~0x00008000,
	~0x00010000, ~0x00020000, ~0x00040000, ~0x00080000,
	~0x00100000, ~0x00200000, ~0x00400000, ~0x00800000,
	~0x01000000, ~0x02000000, ~0x04000000, ~0x08000000,
	~0x10000000, ~0x20000000, ~0x40000000, ~0x80000000
};

static INT16S SPU_module_clock_set(INT16U enable)
{
    return drv_l1_clock_set_system_clk_en(SPU_CLOCK_SOURCE, enable);
}

static INT32U SPU_System_Colok_get(void)
{
    return cpu_mclk;
}

static INT32U SPU_Colok_Init(void)
{
    INT32U sys_clock,temp;

    sys_clock = SPU_System_Colok_get();

    if(sys_clock <= 98000000)
        R_SYSTEM_CLK_CTRL &= ~0x8;
    else
        R_SYSTEM_CLK_CTRL |= 0x8;

    temp = (R_SYSTEM_PLLEN & 0x3F);
    sys_clock = (temp*9000000);
    sys_clock += 27000000;

    temp = R_SYSTEM_HDMI_CTRL;
    temp &= ~0x3F;
    temp |= ((sys_clock / 27000000) - 1);
    R_SYSTEM_HDMI_CTRL = temp;

    return 0;
}

INT32S SPU_System_Colok_Set(INT32U Mclk)
{
    cpu_mclk = MCLK;

    return 0;
}

INT32U SPU_Get_DrvVersion(void)
{
	INT32U uiVersion;
	INT32U *pAddr;

	pAddr = (INT32U *)0xD0400EF4;
	*pAddr |= 0x8000;
	uiVersion = *(INT32U *)0xD0400EC4;
	if(uiVersion == 0x827)
		return C_SPU_DRIVER_VERSION;
	else
		return -1;
}

void SPU_Clear_SRAM(void)
{
	int i;
	for( i = 0; i < 0x400; i++)
	{
		*(P_SPU_SRAM_BASE + i) = 0;
	}
}

void SPU_Clear_Register(void)
{
	int i;
	for( i = 0; i < 0x40; i++)
	{
		*(P_SPU_CH_EN + i) = 0;
	}
	*P_SPU_WAVE_IN_LEFT = 0x8000;
	*P_SPU_WAVE_IN_RIGHT = 0x8000;
}

// SPU registers and internal SRAM initial
void SPU_Uninit(void)
{
    SPU_Clear_Register();
    SPU_Clear_SRAM();
    SPU_module_clock_set(SPU_CLOCK_DISABLE);
}

void SPU_Init(void)
{
	INT8U ChIndex;

	//R_SYSTEM_CLK_CTRL |= 0x10; /* DA/AD PLL Enable */

	//SPU_Clear_Register();
	SPU_module_clock_set(SPU_CLOCK_ENABLE);
	SPU_Colok_Init();
	SPU_Set_MainVolume(0x7F);			// main volume (0x00~0x7F)
	SPU_Clear_FIQ_Status(0xFFFFFFFF);	// chear channel 0 ~ 31 FIQ status
	SPU_Set_BeatBaseCounter(0x0000);
	SPU_Set_BeatCounter(0x0000);
	SPU_Disable_BeatIRQ();
	SPU_Clear_BeatIRQ_Flag();
	for(ChIndex = 0; ChIndex < 32; ChIndex++)
	{
		SPU_Disable_Channel(ChIndex);
		SPU_Clear_Ch_StopFlag(ChIndex);
		SPU_Disable_FIQ_Channel(ChIndex);

		SPU_Set_EnvelopeClock(0x03, ChIndex);
		SPU_Set_EnvRampDown(ChIndex);
		SPU_DisableChannelRepeat(ChIndex);
		SPU_EnvelopeAutoMode(ChIndex);
		SPU_DisablePitchBend(ChIndex);
		SPU_DisableChannelZC(ChIndex);
	}
	SPU_AccumulatorInit();
	SPU_Clear_FOF();
	SPU_SingleChVolumeSelect(0x02);
	SPU_InterpolationON();
	SPU_HQ_InterpolationON();
	SPU_CompressorON();

	SPU_SetCompressorRatio(0x05);
	SPU_DisableCompZeroCrossing();
	SPU_SetReleaseTimeScale(0x03);
	SPU_SetAttackTimeScale(0x00);
	SPU_SetCompressThreshold(0x60);
	SPU_SelectPeakMode();

	SPU_SetReleaseTime(0xFF);
	SPU_SetAttackTime(0x02);

	SPU_SetBenkAddr(0x0000);

	SPU_Clear_SRAM();
	SPU_Get_DrvVersion();
}

void SPU_Enable_Channel(INT8U ChannelIndex)
{
	INT32U *pAddr;
	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_EN |= *pAddr;
	*P_SPU_CH_EN_HIGH |= (*pAddr >> 16);
}

void SPU_Disable_Channel(INT8U ChannelIndex)
{
	INT32U *pAddr;
	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitDisable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_EN &= *pAddr;
	*P_SPU_CH_EN_HIGH &= (*pAddr >> 16);
}

void SPU_Enable_MultiChannel(INT32U ChannelBit)
{
	*P_SPU_CH_EN |= ChannelBit;
	*P_SPU_CH_EN_HIGH |= (ChannelBit >> 16);
}

void SPU_Disable_MultiChannel(INT32U ChannelBit)
{
	*P_SPU_CH_EN &= ~ChannelBit;
	*P_SPU_CH_EN_HIGH &= ~(ChannelBit >> 16);
}

INT32U SPU_GetChannelEnableStatus(void)
{
	INT32U Temp;
	Temp = (*P_SPU_CH_EN_HIGH << 16) | *P_SPU_CH_EN;
	return Temp;
}

void SPU_Set_MainVolume(INT8U VolumeData)
{
	*P_SPU_MAIN_VOLUME = VolumeData & 0x007F;
}

INT8U SPU_Get_MainVolume(void)
{
	return *P_SPU_MAIN_VOLUME;
}

void SPU_Enable_FIQ_Channel(INT8U ChannelIndex)
{
	INT32U *pAddr;
	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_FIQ_EN |= *pAddr;
	*P_SPU_CH_FIQ_EN_HIGH |= (*pAddr >> 16);
}

void SPU_Disable_FIQ_Channel(INT8U ChannelIndex)
{
	INT32U *pAddr;
	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitDisable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_FIQ_EN &= *pAddr;
	*P_SPU_CH_FIQ_EN_HIGH &= (*pAddr >> 16);
}

void SPU_Enable_FIQ_MultiChannel(INT32U ChannelBit)
{
	*P_SPU_CH_FIQ_EN |= ChannelBit;
	*P_SPU_CH_FIQ_EN_HIGH |= (ChannelBit >> 16);
}

void SPU_Disable_FIQ_MultiChannel(INT32U ChannelBit)
{
	*P_SPU_CH_FIQ_EN &= ~ChannelBit;
	*P_SPU_CH_FIQ_EN_HIGH &= ~(ChannelBit >> 16);
}

void SPU_Clear_FIQ_Status(INT32U ChannelBit)
{
	*P_SPU_CH_FIQ_STATUS = ChannelBit;
	*P_SPU_CH_FIQ_STATUS_HIGH = ChannelBit >>16;
}

INT32U SPU_Get_FIQ_Status(void)
{
	INT32U FIQ_Status;

	FIQ_Status = (*P_SPU_CH_FIQ_STATUS_HIGH << 16) | (*P_SPU_CH_FIQ_STATUS);
	return FIQ_Status;
}

void SPU_Set_BeatBaseCounter(INT16U BeatBaseCounter)
{
	BeatBaseCounter &= 0x07FF;
	*P_SPU_BEAT_BASE_COUNTER = BeatBaseCounter;
}

INT16U SPU_Get_BeatBaseCounter(void)
{
	INT16U BeatBaseCounter;

	BeatBaseCounter = *P_SPU_BEAT_BASE_COUNTER;
	return BeatBaseCounter;
}

void SPU_Set_BeatCounter(INT16U BeatCounter)
{
	INT16U BeatCountReg;

	BeatCountReg = *P_SPU_BEAT_COUNTER;
	BeatCountReg &= ~0x7FFF;	// bit14 is beat count interrupt flag,
	                            // do not write '1' to this bit otherwise the interrupt flag will be cleared
	BeatCountReg |= (BeatCounter & 0x3FFF);
	*P_SPU_BEAT_COUNTER = BeatCountReg;
}

INT16U SPU_Get_BeatCounter(void)
{
	INT16U BeatCounter;

	BeatCounter = *P_SPU_BEAT_COUNTER & 0x3FFF;
	return BeatCounter;
}

void SPU_Enable_BeatIRQ(void)
{
	*P_SPU_BEAT_COUNTER |= C_BEAT_IRQ_EN;
}

void SPU_Disable_BeatIRQ(void)
{
	*P_SPU_BEAT_COUNTER &= ~C_BEAT_IRQ_EN;
}

void SPU_Clear_BeatIRQ_Flag(void)
{
	*P_SPU_BEAT_COUNTER |= C_BEAT_IRQ_STATUS;
}

INT8U SPU_Get_BeatIRQ_Flag(void)
{
	if(*P_SPU_BEAT_COUNTER & C_BEAT_IRQ_STATUS)
		return 1;
	else
		return 0;
}

INT8U SPU_Get_BeatIRQ_Enable_Flag(void)		//add by tangqt
{
	if(*P_SPU_BEAT_COUNTER & C_BEAT_IRQ_EN)
		return 1;
	else
		return 0;
}

void SPU_Set_EnvelopeClock(INT8U EnvClock, INT8U ChannelIndex)
{
	INT16U TempData;
	INT16U MaskBit, EnvClockData;
	//INT16U *pAddr;
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	MaskBit = 0x000F;
	MaskBit = MaskBit << (ChannelIndex & 0x0003) * 4;
	EnvClockData = EnvClock << (ChannelIndex & 0x0003) * 4;
	ChannelIndex = ChannelIndex >> 2;
	if(ChannelIndex >= 4)
		//pAddr = (INT16U *)(P_SPU_ENV_CLK_CH16_19 - 4 * 2);
		pAddr = (INT32U *)(P_SPU_ENV_CLK_CH16_19 - 4 );
	else
		//pAddr = (INT16U *)P_SPU_ENV_CLK_CH0_3;
		pAddr = (INT32U *)P_SPU_ENV_CLK_CH0_3;
	//TempData = *(pAddr + ChannelIndex * 2);
	TempData = *(pAddr + ChannelIndex);
	TempData &= ~MaskBit;
	TempData |= EnvClockData;
	//*(pAddr + ChannelIndex * 2) = TempData;
	*(pAddr + ChannelIndex) = TempData;
}

void SPU_Set_EnvRampDown(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_ENV_RAMP_DOWN |= *pAddr;
	*P_SPU_ENV_RAMP_DOWN_HIGH |= (*pAddr >> 16);
}

void SPU_Set_EnvRampDownMultiChannel(INT32U ChannelBit)
{
	*P_SPU_ENV_RAMP_DOWN |= ChannelBit;
	*P_SPU_ENV_RAMP_DOWN_HIGH |= (ChannelBit >> 16);
}


INT32U SPU_Get_EnvRampDown()
{
	INT32U Temp;

	Temp = (*P_SPU_ENV_RAMP_DOWN_HIGH << 16) | *P_SPU_ENV_RAMP_DOWN;
	return Temp;
}

void SPU_Clear_Ch_StopFlag(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_STOP_STATUS = *pAddr;
	*P_SPU_CH_STOP_STATUS_HIGH = (*pAddr >> 16);
}

void SPU_Clear_MultiCh_StopFlag(INT32U ChannelBit)
{
	*P_SPU_CH_STOP_STATUS = ChannelBit;
	*P_SPU_CH_STOP_STATUS_HIGH = (ChannelBit >> 16);
}

INT32U SPU_Get_Ch_StopStatus(void)
{
	INT32U Temp;

	Temp = (*P_SPU_CH_STOP_STATUS_HIGH << 16) | *P_SPU_CH_STOP_STATUS;
	return Temp;
}

void SPU_EnableChannelZC(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_ZC_ENABLE |= *pAddr;
	*P_SPU_CH_ZC_ENABLE_HIGH |= (*pAddr >> 16);
}

void SPU_DisableChannelZC(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitDisable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_ZC_ENABLE &= *pAddr;
	*P_SPU_CH_ZC_ENABLE_HIGH &= (*pAddr >> 16);
}

void SPU_AccumulatorInit(void)
{
	*P_SPU_CONTROL_FLAG |= C_INIT_ACC;
}

INT8U SPU_Read_FOF_Status(void)
{
	INT16U Temp;
	Temp = *P_SPU_CONTROL_FLAG;
	Temp &= C_FOF_FLAG;
	Temp = Temp >> 5;
	return Temp;
}

void SPU_Clear_FOF(void)
{
	*P_SPU_CONTROL_FLAG |= C_FOF_FLAG;
}

void SPU_SingleChVolumeSelect(INT8U VolumeSelect)
{
	INT16U Temp;

	Temp = *P_SPU_CONTROL_FLAG;
	Temp &= ~C_CH_VOL_SEL;
	Temp |= ((VolumeSelect & 0x0003) << 6);
	*P_SPU_CONTROL_FLAG = Temp;
}

INT8U SPU_GetSingleChVolumeSetting(void)
{
	INT8U Temp;

	Temp = *P_SPU_CONTROL_FLAG;
	Temp &= C_CH_VOL_SEL;
	Temp = Temp >> 6;
	return Temp;
}

void SPU_InterpolationON(void)
{
	*P_SPU_CONTROL_FLAG &= ~C_NO_INTER;
}

void SPU_InterpolationOFF(void)
{
	*P_SPU_CONTROL_FLAG |= C_NO_INTER;
}

void SPU_HQ_InterpolationON(void)
{
	*P_SPU_CONTROL_FLAG &= ~C_NO_HIGH_INTER;
}

void SPU_HQ_InterpolationOFF(void)
{
	*P_SPU_CONTROL_FLAG |= C_NO_HIGH_INTER;
}

void SPU_SignedPCM16ON(void)
{
	*P_SPU_CONTROL_FLAG2 |= C_SIGNED_ENABLE;
}

void SPU_SignedPCM16OFF(void)
{
	*P_SPU_CONTROL_FLAG2 &= ~C_SIGNED_ENABLE;
}

void SPU_CompressorON(void)
{
	*P_SPU_CONTROL_FLAG |= C_COMP_EN;
}

void SPU_CompressorOFF(void)
{
	*P_SPU_CONTROL_FLAG &= ~C_COMP_EN;
}

void SPU_ClearSaturateFlag(void)
{
	*P_SPU_CONTROL_FLAG |= C_SATURATE;
}

INT8U SPU_ReadSaturateFlag(void)
{
	INT8U Temp;

	Temp = (*P_SPU_CONTROL_FLAG & C_SATURATE) >> 15;
	return Temp;
}

void SPU_SetCompressorRatio(INT8U ComRatio)
{
	INT16U Ratio;

	Ratio = *P_SPU_COMPRESSOR_CTRL;
	Ratio &= ~C_COMPRESS_RATIO;
	Ratio |= (ComRatio & C_COMPRESS_RATIO);
	*P_SPU_COMPRESSOR_CTRL = Ratio;
}

INT8U SPU_GetCompressorRatio(void)
{
	INT8U Ratio;

	Ratio = (*P_SPU_COMPRESSOR_CTRL & C_COMPRESS_RATIO);
	return Ratio;
}

void SPU_EnableCompZeroCrossing(void)
{
	*P_SPU_COMPRESSOR_CTRL &= ~C_DISABLE_ZC;
}

void SPU_DisableCompZeroCrossing(void)
{
	*P_SPU_COMPRESSOR_CTRL |= C_DISABLE_ZC;
}

void SPU_SetReleaseTimeScale(INT8U ReleaseTimeScale)
{
	INT16U Scale;

	Scale = *P_SPU_COMPRESSOR_CTRL;
	Scale &= ~C_RELEASE_SCALE;
	Scale |= ((ReleaseTimeScale & 0x0003) << 4);
	*P_SPU_COMPRESSOR_CTRL = Scale;
}

INT8U SPU_ReadReleaseTimeScale(void)
{
	INT8U Temp;

	Temp = (*P_SPU_COMPRESSOR_CTRL & C_RELEASE_SCALE) >> 4;
	return Temp;
}

void SPU_SetAttackTimeScale(INT8U AttackTimeScale)
{
	INT16U Scale;

	Scale = *P_SPU_COMPRESSOR_CTRL;
	Scale &= ~C_ATTACK_SCALE;
	Scale |= ((AttackTimeScale & 0x0003) << 6);
	*P_SPU_COMPRESSOR_CTRL = Scale;
}

INT8U SPU_ReadAttackTimeScale(void)
{
	INT8U Temp;

	Temp = (*P_SPU_COMPRESSOR_CTRL & C_ATTACK_SCALE) >> 6;
	return Temp;
}

void SPU_SetCompressThreshold(INT8U CompThreshold)
{
	INT16U Threshold;

	Threshold = *P_SPU_COMPRESSOR_CTRL;
	Threshold &= ~C_COMPRESS_THRESHOLD;
	Threshold |= ((CompThreshold & 0x007F) << 8);
	*P_SPU_COMPRESSOR_CTRL = Threshold;
}

INT8U SPU_ReadCompressThreshold(void)
{
	INT8U Temp;

	Temp = (*P_SPU_COMPRESSOR_CTRL & C_COMPRESS_THRESHOLD) >> 8;
	return Temp;
}

void SPU_SelectRMS_Mode(void)
{
	*P_SPU_COMPRESSOR_CTRL &= ~C_COMPRESS_PEAK_MODE;
}

void SPU_SelectPeakMode(void)
{
	*P_SPU_COMPRESSOR_CTRL |= C_COMPRESS_PEAK_MODE;
}

INT32U SPU_GetChannelStatus(void)
{
	INT32U Status;

	Status = (*P_SPU_CH_STATUS_HIGH << 16) | *P_SPU_CH_STATUS;
	return Status;
}

void SPU_SendToSoftChannel_Left(INT16U PCM_Data)
{
	*P_SPU_WAVE_IN_LEFT = PCM_Data;
}

void SPU_SendToSoftChannel_Right(INT16U PCM_Data)
{
	*P_SPU_WAVE_IN_RIGHT = PCM_Data;
}

INT16U SPU_GetSPU_PlusSoftOutLeft(void)
{
	return *P_SPU_WAVE_OUT_LEFT;
}

INT16U SPU_GetSPU_PlusSoftOutRight(void)
{
	return *P_SPU_WAVE_OUT_RIGHT;
}

INT16U SPU_GetSPU_OutLeft(void)
{
	return *P_SPU_POST_WAVE_OUT_LEFT;
}

INT16U SPU_GetSPU_OutRight(void)
{
	return *P_SPU_POST_WAVE_OUT_RIGHT;
}

void SPU_EnableChannelRepeat(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_REPEAT_EN |= *pAddr;
	*P_SPU_CH_REPEAT_EN_HIGH |= (*pAddr >> 16);
}

void SPU_DisableChannelRepeat(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitDisable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_REPEAT_EN &= *pAddr;
	*P_SPU_CH_REPEAT_EN_HIGH &= (*pAddr >> 16);
}

void SPU_DisableMultiChannelRepeat(INT32U ChannelBit)
{
	*P_SPU_CH_REPEAT_EN &= ~ChannelBit;
	*P_SPU_CH_REPEAT_EN_HIGH &= ~(ChannelBit >> 16);
}

void SPU_EnvelopeAutoMode(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitDisable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_ENV_MODE &= *pAddr;
	*P_SPU_CH_ENV_MODE_HIGH &= (*pAddr >> 16);
}

void SPU_EnvelopeManualMode(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_ENV_MODE |= *pAddr;
	*P_SPU_CH_ENV_MODE_HIGH |= (*pAddr >> 16);
}

void SPU_SetChannelRelease(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_TONE_RELEASE |= *pAddr;
	*P_SPU_CH_TONE_RELEASE_HIGH |= (*pAddr >> 16);
}

INT32U SPU_GetEnvelopeIRQ_Status(void)
{
	INT32U Status;

	Status = (*P_SPU_CH_IRQ_STATUS_HIGH << 16) | *P_SPU_CH_IRQ_STATUS;
	return Status;
}

void SPU_ClearEnvelopeIRQ_Status(INT32U ChannelBit)
{
	*P_SPU_CH_IRQ_STATUS = (ChannelBit & 0xFFFF);
	*P_SPU_CH_IRQ_STATUS_HIGH = ChannelBit >> 16;
}

void SPU_EnablePitchBend(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitEnable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_PITCH_BEND_EN |= *pAddr;
	*P_SPU_CH_PITCH_BEND_EN_HIGH |= (*pAddr >> 16);
}

void SPU_DisablePitchBend(INT8U ChannelIndex)
{
	INT32U *pAddr;

	ChannelIndex &= 0x1F;
	pAddr = (INT32U *)T_BitDisable_Table;
	pAddr += ChannelIndex;
	*P_SPU_CH_PITCH_BEND_EN &= *pAddr;
	*P_SPU_CH_PITCH_BEND_EN_HIGH &= (*pAddr >> 16);
}

void SPU_SetReleaseTime(INT8U ReleaseTime)
{
	INT16U RelTime;

	RelTime = *P_SPU_ATTACK_RELEASE_TIME;
	RelTime &= 0xFF00;
	RelTime |= ReleaseTime;
	*P_SPU_ATTACK_RELEASE_TIME = RelTime;
}


INT8U SPU_ReadReleaseTime(void)
{
	INT8U Temp;

	Temp = *P_SPU_ATTACK_RELEASE_TIME & 0x00FF;
	return Temp;
}

void SPU_SetAttackTime(INT8U AttackTime)
{
	INT16U AttTime;

	AttTime = *P_SPU_ATTACK_RELEASE_TIME;
	AttTime &= 0x00FF;
	AttTime |= (AttackTime << 8);
	*P_SPU_ATTACK_RELEASE_TIME = AttTime;
}

INT8U SPU_ReadAttackTime(void)
{
	INT8U Temp;

	Temp = (*P_SPU_ATTACK_RELEASE_TIME & 0xFF00) >> 8;
	return Temp;
}

void SPU_SetBenkAddr(INT16U BenkAddr)
{
	*P_SPU_BENK_ADDR = BenkAddr & 0x01FF;
}

void SPU_ClearPWFOV_Flag(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_OVERFLOW;
//	*P_SPU_POST_WAVE_CTRL |= 0x0009;
}

INT8U SPU_GetPWFOV_Flag(void)
{
	return (*P_SPU_POST_WAVE_CTRL & 0x0001);
}

void SPU_EnablePW_RightChannel(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_LR;
}

void SPU_DisablePW_RightChannel(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_LR;
}

void SPU_EnablePostWaveOutputSilence(void)
{
//	*P_SPU_POST_WAVE_CTRL &= ~C_PW_TO_DAC;
	*P_SPU_POST_WAVE_CTRL |= C_PW_TO_DAC;
}

void SPU_DisablePostWaveOutputSilence(void)
{
//	*P_SPU_POST_WAVE_CTRL |= C_PW_TO_DAC;
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_TO_DAC;
}

void SPU_SetPostWave_Signed(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_SIGNED;
}

void SPU_SetPostWave_Unsigned(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_SIGNED;
}

void SPU_ClearPostWaveIRQ_Status(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_IRQ_ACTIVE;
}

INT8U SPU_GetPostWaveIRQ_Status(void)
{
	return (*P_SPU_POST_WAVE_CTRL >> 10);
}

void SPU_EnablePostWaveIRQ(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_IRQ_ENABLE;
}

void SPU_DisablePostWaveIRQ(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_IRQ_ENABLE;
}

void SPU_SetPostWaveClock_288K(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_CLOCK_SET;
}

void SPU_SetPostWaveClock_281K(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_CLOCK_SET;
}

void SPU_EnablePostWaveLPF(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_LPF_ENABLE;
}

void SPU_DisablePostWaveLPF(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_LPF_ENABLE;
}

void SPU_EnablePostWaveI2SSync(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_I2S_SYNC;
}

void SPU_DisablePostWaveI2SSync(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_I2S_SYNC;
}

#if 1 // williamyeo added
INT8U SPU_GetPostWaveI2SSync(void)
{
    return ((*P_SPU_POST_WAVE_CTRL & C_PW_I2S_SYNC) != 0) ? 1: 0;
}

void SPU_SetI2SSyncSampleRate(INT32U sampleRate)
{
    spu_i2s_sampleRate = sampleRate;
}

INT32U SPU_GetI2SSyncSampleRate(void)
{
    return spu_i2s_sampleRate;
}
#endif

void SPU_EnablePostWaveDownSample(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_DOWN_SAMPLE;
}

void SPU_DisablePostWaveDownSample(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_DOWN_SAMPLE;
}

void SPU_EnablePostWaveToDMA(void)
{
	*P_SPU_POST_WAVE_CTRL |= C_PW_DMA;
}

void SPU_DisablePostWaveToDMA(void)
{
	*P_SPU_POST_WAVE_CTRL &= ~C_PW_DMA;
}

void SPU_SetStartAddress(INT32U StartAddr, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	*(P_SPU_WAVE_ADDR_LOW + 0x10 * ChannelIndex) = StartAddr;
	Temp = *(P_SPU_MODE + 0x10 * ChannelIndex);
	Temp &= ~0x003F;
	Temp |= ((StartAddr >> 16) & 0x003F);
	*(P_SPU_MODE + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_MODE + 0x10 * ChannelIndex);

	// ycliao, for more address bits
	Temp = *(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex);
	Temp &= ~0x003F;
	Temp |= ((StartAddr >> 22) & 0x003F);
	*(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex);
}

void SPU_GetStartAddress(INT32U *StartAddr, INT8U ChannelIndex)
{
	INT32U Temp = 0;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_WAVE_ADDR_LOW + 0x10 * ChannelIndex) & 0xFFFF ;
	Temp |= (*(P_SPU_MODE + 0x10 * ChannelIndex) & 0x3F) << 16;
	Temp |= (*(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex) & 0x3F) << 22;

	*StartAddr = Temp;
}

void SPU_SetLoopAddress(INT32U LoopAddr, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	*(P_SPU_LOOP_ADDR + 0x10 * ChannelIndex) = LoopAddr;
	Temp = *(P_SPU_MODE + 0x10 * ChannelIndex);
	Temp &= ~0x0FC0;
	Temp |= ((LoopAddr >> 10) & 0x0FC0);
	*(P_SPU_MODE + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_MODE + 0x10 * ChannelIndex);

	// ycliao, for more address bits
	Temp = *(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex);
	Temp &= ~0x0FC0;
	Temp |= ((LoopAddr >> 16) & 0x0FC0);
	*(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex);
}

void SPU_GetLoopAddress(INT32U *LoopAddr, INT8U ChannelIndex)
{
	INT32U Temp = 0;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_LOOP_ADDR + 0x10 * ChannelIndex) & 0xFFFF ;
	Temp |= (*(P_SPU_MODE + 0x10 * ChannelIndex) & 0x0FC0) << 10;
	Temp |= (*(P_SPU_WL_ADDR_HIGH + 0x10 * ChannelIndex) & 0x0FC0) << 16;

	*LoopAddr = Temp;
}

//void SPU_SetToneColorMode(INT32U ToneColorMode, INT32U ChannelIndex)
void SPU_SetToneColorMode(INT8U ToneColorMode, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_MODE + 0x10 * ChannelIndex);
	Temp &= ~0x3000;
	Temp |= (ToneColorMode << 12);
	*(P_SPU_MODE + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_MODE + 0x10 * ChannelIndex);
}

void SPU_Set_16bit_Mode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_MODE + 0x10 * ChannelIndex) |= C_16BIT_DATA_MODE;
        addr = (INT32U)(P_SPU_MODE + 0x10 * ChannelIndex);
}

void SPU_Set_8bit_Mode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_MODE + 0x10 * ChannelIndex) &= ~C_16BIT_DATA_MODE;
        addr = (INT32U)(P_SPU_MODE + 0x10 * ChannelIndex);
}

void SPU_SetADPCM_Mode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_MODE + 0x10 * ChannelIndex) |= C_ADPCM_MODE;
        addr = (INT32U)(P_SPU_MODE + 0x10 * ChannelIndex);
}

void SPU_SetPCM_Mode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_MODE + 0x10 * ChannelIndex) &= ~C_ADPCM_MODE;
        addr = (INT32U)(P_SPU_MODE + 0x10 * ChannelIndex);
}

void SPU_SetVelocity(INT8U VelocityValue, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_PAN_VELOCITY + 0x10 * ChannelIndex);
	Temp &= 0xFF00;
	Temp |= (VelocityValue & 0x007F);
	*(P_SPU_PAN_VELOCITY + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_PAN_VELOCITY + 0x10 * ChannelIndex);
}

void SPU_SetPan(INT8U PanValue, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_PAN_VELOCITY + 0x10 * ChannelIndex);
	Temp &= 0x00FF;
	Temp |= ((PanValue & 0x007F) << 8);
	*(P_SPU_PAN_VELOCITY + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_PAN_VELOCITY + 0x10 * ChannelIndex);
}

void SPU_SetEnvelope_0(INT16U Envelope_0, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex) = Envelope_0;
        addr = (INT32U)(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeIncrement(INT8U EnvInc, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex);
	Temp &= ~0x007F;
	Temp |= (EnvInc & 0x007F);
	*(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex);
}

INT8U SPU_ReadEnvelopeIncrement(INT8U ChannelIndex)
{
	INT8U Temp;

	Temp = (*(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex)) & 0x7F;
	return Temp;
}

void SPU_SetEnvelopePostive(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex) &= ~C_ENVELOPE_SIGN;
        addr = (INT32U)(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeNegative(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex) |= C_ENVELOPE_SIGN;
        addr = (INT32U)(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeTarget(INT8U EnvTarget, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex);
	Temp &= ~0x7F00;
	Temp |= ((EnvTarget & 0x007F) << 8);
	*(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex);
}

INT8U SPU_ReadEnvelopeTarget(INT8U ChannelIndex)
{
	INT8U Temp;

	Temp = (*(P_SPU_ENVELOPE_0 + 0x10 * ChannelIndex)) >> 8;
	return Temp;
}

void SPU_SetEnvelopeData(INT8U EnvData, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ENVELOPE_DATA + 0x10 * ChannelIndex);
	Temp &= ~0x007F;
	Temp |= (EnvData & 0x007F);
	*(P_SPU_ENVELOPE_DATA + 0x10 * ChannelIndex) = Temp;
}

INT8U SPU_ReadEnvelopeData(INT8U ChannelIndex)
{
	INT8U Temp;

	Temp = (*(P_SPU_ENVELOPE_DATA + 0x10 * ChannelIndex)) & 0x00FF;
	return Temp;
}

void SPU_SetEnvelopeCounter(INT8U EnvCounter, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ENVELOPE_DATA + 0x10 * ChannelIndex);
	Temp &= ~0xFF00;
	Temp |= ((EnvCounter & 0x00FF) << 8);
	*(P_SPU_ENVELOPE_DATA + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ENVELOPE_DATA + 0x10 * ChannelIndex);
}

INT8U SPU_ReadEnvelopeCounter(INT8U ChannelIndex)
{
	INT8U Temp;

	Temp = (*(P_SPU_ENVELOPE_DATA + 0x10 * ChannelIndex)) >> 8;
	return Temp;
}

void SPU_SetEnvelope_1(INT16U Envelope_1, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex) = Envelope_1;
        addr = (INT32U)(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeReloadData(INT8U EnvReloadData, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex);
	Temp &= ~0x00FF;
	Temp |= (EnvReloadData & 0x007F);
	*(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex);
}

INT8U SPU_ReadEnvelopeReloadData(INT8U ChannelIndex)
{
	INT8U Temp;

	Temp = (*(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex)) & 0x007F;
	return Temp;
}

void SPU_SetEnvelopeRepeatMode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex) |= C_ENVELOPE_REPEAT;
        addr = (INT32U)(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeNormalMode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex) &= ~C_ENVELOPE_REPEAT;
        addr = (INT32U)(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeRepeatCounter(INT8U RepeatCounter, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex);
	Temp &= ~0xFE00;
	Temp |= ((RepeatCounter & 0x007F) << 9);
	*(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex);
}

INT8U SPU_ReadEnvelopeRepeatCounter(INT8U ChannelIndex)
{
	INT8U Temp;

	Temp = (*(P_SPU_ENVELOPE_1 + 0x10 * ChannelIndex)) >> 9;
	return Temp;
}

void SPU_SetEnvelopeAddress(INT32U EnvelopeAddr, INT8U ChannelIndex)
{
        INT16U Temp;

	ChannelIndex &= 0x1F;
	*(P_SPU_ENV_ADDR_LOW + 0x10 * ChannelIndex) = EnvelopeAddr;
	Temp = *(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex);
	Temp &= ~0x003F;
	Temp |= ((EnvelopeAddr >> 16) & 0x003F);
	*(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex);

	// ycliao, for more address bits
	Temp = *(P_SPU_WAVE_ADDR_HIGH + 0x10 * ChannelIndex);
	Temp &= ~0x003F;
	Temp |= ((EnvelopeAddr >> 22) & 0x003F);
	*(P_SPU_WAVE_ADDR_HIGH + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_WAVE_ADDR_HIGH + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeIRQ_Enable(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex) |= C_ENVELOPE_IRQ_ENABLE;
        addr = (INT32U)(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeIRQ_Disable(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex) &= ~C_ENVELOPE_IRQ_ENABLE;
        addr = (INT32U)(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeIRQ_FireAddress(INT16U FireAddr, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex);
	Temp &= ~0xFF80;
	Temp |= ((FireAddr & 0x01FF) << 7);
	*(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ENV_ADDR_HIGH + 0x10 * ChannelIndex);
}

void SPU_SetWaveData_0(INT16U WDD_0, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_WAVE_DATA_0 + 0x10 * ChannelIndex) = WDD_0;
        addr = (INT32U)(P_SPU_WAVE_DATA_0 + 0x10 * ChannelIndex);
}

void SPU_SetWaveData(INT16U WDD, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_WAVE_DATA + 0x10 * ChannelIndex) = WDD;
        addr = (INT32U)(P_SPU_WAVE_DATA + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeRepeatAddrOffset(INT16U EAOffset, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_LOOP_CTRL + 0x10 * ChannelIndex);
	Temp &= ~0x01FF;
	Temp |= (EAOffset & 0x01FF);
	*(P_SPU_LOOP_CTRL + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_LOOP_CTRL + 0x10 * ChannelIndex);
}

void SPU_SetEnvelopeRampDownOffset(INT8U RampDownOffset, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_LOOP_CTRL + 0x10 * ChannelIndex);
	Temp &= 0x01FF;
	Temp |= ((RampDownOffset & 0x007F) << 9);
	*(P_SPU_LOOP_CTRL + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_LOOP_CTRL + 0x10 * ChannelIndex);
}

void SPU_SelectADPCM_Mode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex) &= ~C_ADPCM36_MODE;
        addr = (INT32U)(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex);
}

void SPU_ClearADCPM36_Mode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex) = 0;
        addr = (INT32U)(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex);
}

void SPU_SelectADPCM36_Mode(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex) |= C_ADPCM36_MODE;
        addr = (INT32U)(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex);
}

void SPU_SetADPCM_PointNumber(INT8U PointNumber, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex);
	Temp &= ~0x7E00;
	Temp |= ((PointNumber & 0x001F) << 9);
	*(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_ADPCM_SEL + 0x10 * ChannelIndex);
}

void SPU_SetPhase(INT32U Phase, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
        *(P_SPU_PHASE + 0x10 * ChannelIndex) = Phase;
        addr = (INT32U)(P_SPU_PHASE + 0x10 * ChannelIndex);
        *(P_SPU_PHASE_HIGH + 0x10 * ChannelIndex) = Phase >> 16;
        addr = (INT32U)(P_SPU_PHASE_HIGH + 0x10 * ChannelIndex);
}

INT32U SPU_ReadPhase(INT8U ChannelIndex)
{
	INT32U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_PHASE + 0x10 * ChannelIndex)  | ((*(P_SPU_PHASE_HIGH + 0x10 * ChannelIndex) & 0x0007) << 16);
	return Temp;
}

void SPU_SetPhaseAccumulator(INT32U PhaseAcc, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_PHASE_ACC + 0x10 * ChannelIndex) = PhaseAcc;
        addr = (INT32U)(P_SPU_PHASE_ACC + 0x10 * ChannelIndex);
	*(P_SPU_PHASE_ACC_HIGH + 0x10 * ChannelIndex) = PhaseAcc >> 16;
        addr = (INT32U)(P_SPU_PHASE_ACC_HIGH + 0x10 * ChannelIndex);
}

INT32U SPU_ReadPhaseAccumulator(INT8U ChannelIndex)
{
	INT32U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_PHASE_ACC + 0x10 * ChannelIndex)  | ((*(P_SPU_PHASE_ACC_HIGH + 0x10 * ChannelIndex) & 0x0007) << 16);
	return Temp;
}

void SPU_SetTargetPhase(INT32U TargetPhase, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_TARGET_PHASE + 0x10 * ChannelIndex) = TargetPhase;
        addr = (INT32U)(P_SPU_TARGET_PHASE + 0x10 * ChannelIndex);
	*(P_SPU_TARGET_PHASE_HIGH + 0x10 * ChannelIndex) = TargetPhase >> 16;
        addr = (INT32U)(P_SPU_TARGET_PHASE_HIGH + 0x10 * ChannelIndex);
}

INT32U SPU_ReadTargetPhase(INT8U ChannelIndex)
{
	INT32U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_TARGET_PHASE + 0x10 * ChannelIndex)  | ((*(P_SPU_TARGET_PHASE_HIGH + 0x10 * ChannelIndex) & 0x0007) << 16);
	return Temp;
}

void SPU_SetRampDownClock(INT8U RampDownClock, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_RAMP_DOWN_CLK + 0x10 * ChannelIndex) = (RampDownClock & 0x07);
        addr = (INT32U)(P_SPU_RAMP_DOWN_CLK + 0x10 * ChannelIndex);
}

void SPU_SetPhaseOffset(INT16U PhaseOffset, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
	Temp &= 0xF000;
	Temp |= (PhaseOffset & 0x0FFF);
	*(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
}

void SPU_SetPhaseIncrease(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex) &= ~C_PHASE_SIGN;
        addr = (INT32U)(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
}

void SPU_SetPhaseDecrease(INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex) |= C_PHASE_SIGN;
        addr = (INT32U)(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
}

void SPU_SetPhaseTimeStep(INT8U PhaseTimeStep, INT8U ChannelIndex)
{
	INT16U Temp;

	ChannelIndex &= 0x1F;
	Temp = *(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
	Temp &= 0x1FFF;
	Temp |= ((PhaseTimeStep & 0x0007) << 13);
	*(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex) = Temp;
        addr = (INT32U)(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
}

void SPU_SetPhaseControl(INT16U PhaseControl, INT8U ChannelIndex)
{
        ChannelIndex &= 0x1F;
	*(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex) = PhaseControl;
        addr = (INT32U)(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
}

INT16U SPU_ReadPhaseControl(INT8U ChannelIndex)
{
	INT16U Temp;

	Temp = *(P_SPU_PHASE_CTRL + 0x10 * ChannelIndex);
	return Temp;
}

void SPU_FIQ_Register(INT8U FIQ_Index, void (*F_ISR)(void))
{
	switch(FIQ_Index)
	{
		case VIC_SPU_PW:
			//vic_fiq_register(VIC_SPU_PW, F_ISR);
			break;
		case VIC_SPU_BEAT:
			//vic_fiq_register(VIC_SPU_BEAT, F_ISR);
			break;
		case VIC_SPU_ENV:
			//vic_fiq_register(VIC_SPU_ENV, F_ISR);
			break;
		case VIC_SPU_FIQ:
			//vic_fiq_register(VIC_SPU_FIQ, F_ISR);
			break;
	}
}

void SPU_FIQ_Enable(INT8U FIQ_Index)
{
	switch(FIQ_Index)
	{
		case VIC_SPU_PW:
			//vic_fiq_enable(VIC_SPU_PW);
			NVIC_SetPriority(SPUPW_IRQn, 5);
			NVIC_EnableIRQ(SPUPW_IRQn);
			break;
		case VIC_SPU_BEAT:
			//vic_fiq_enable(VIC_SPU_BEAT);
			NVIC_SetPriority(SPUBEAT_IRQn, 5);
			NVIC_EnableIRQ(SPUBEAT_IRQn);
			break;
		case VIC_SPU_ENV:
			//vic_fiq_enable(VIC_SPU_ENV);
			NVIC_SetPriority(SPUENV_IRQn, 5);
			NVIC_EnableIRQ(SPUENV_IRQn);
			break;
		case VIC_SPU_FIQ:
			//vic_fiq_enable(VIC_SPU_FIQ);
			NVIC_SetPriority(SPUFIQ_IRQn, 5);
			NVIC_EnableIRQ(SPUFIQ_IRQn);
			break;
	}
}

void SPU_FIQ_Disable(INT8U FIQ_Index)
{
	switch(FIQ_Index)
	{
		case VIC_SPU_PW:
			//vic_fiq_disable(VIC_SPU_PW);
                        NVIC_DisableIRQ(SPUPW_IRQn);
			break;
		case VIC_SPU_BEAT:
			//vic_fiq_disable(VIC_SPU_BEAT);
                        NVIC_DisableIRQ(SPUBEAT_IRQn);
			break;
		case VIC_SPU_ENV:
			//vic_fiq_disable(VIC_SPU_ENV);
			NVIC_DisableIRQ(SPUENV_IRQn);
                        break;
		case VIC_SPU_FIQ:
			//vic_fiq_disable(VIC_SPU_FIQ);
			NVIC_DisableIRQ(SPUFIQ_IRQn);
                        break;
	}
}


void SPU_Test_IRQ_Register(INT8U IRQ_Index, void (*F_ISR)(void))
{
	switch(IRQ_Index)
	{
		case VIC_TIMER0:
			//vic_irq_register(VIC_TIMER0, F_ISR);
			break;
		case VIC_TIMER1:
			//vic_irq_register(VIC_TIMER1, F_ISR);
			break;
		case VIC_TIMER2:
			//vic_irq_register(VIC_TIMER2, F_ISR);
			break;
		case VIC_TIMER3:
			//vic_irq_register(VIC_TIMER3, F_ISR);
			break;
	}
}

void SPU_Test_IRQ_Enable(INT8U IRQ_Index)
{
	switch(IRQ_Index)
	{
		case VIC_TIMER0:
			//vic_irq_enable(VIC_TIMER0);
			break;
		case VIC_TIMER1:
			//vic_irq_enable(VIC_TIMER1);
			break;
		case VIC_TIMER2:
			//vic_irq_enable(VIC_TIMER2);
			break;
		case VIC_TIMER3:
			//vic_irq_enable(VIC_TIMER3);
			break;
	}
}

void SPU_Test_IRQ_Disable(INT8U IRQ_Index)
{
	/*switch(IRQ_Index)
	{
		case VIC_TIMER0:
			vic_irq_disable(VIC_TIMER0);
			stop_timer(TIMER_A);
			break;
		case VIC_TIMER1:
			vic_irq_disable(VIC_TIMER1);
			stop_timer(TIMER_B);
			break;
		case VIC_TIMER2:
			vic_irq_disable(VIC_TIMER2);
			stop_timer(TIMER_C);
			break;
		case VIC_TIMER3:
			vic_irq_disable(VIC_TIMER3);
			stop_timer(TIMER_D);
			break;
	}*/
}

void SPU_Test_Timer_IRQ_Setup(INT32U timer_id, INT32U freq_hz, void (*TimerIsrFunction)(void))
{
	//timer_freq_setup(timer_id, freq_hz, 0, TimerIsrFunction);
}

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#endif //(defined _DRV_L1_SPU) && (_DRV_L1_SPU == 1)                   //
//================================================================//
