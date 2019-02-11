#ifndef __ENVELOPDETECT_H__
#define __ENVELOPDETECT_H__

#include "application.h"

//***************************************************************************************
// Contant Defintion Area
//***************************************************************************************
// Blow Sensor Level
//
#define BLOW_SENSOR_THRESHOLD      1000
#define BLOW_SENSOR_STEP           500
#define BLOW_SENSOR_LVL_1          BLOW_SENSOR_THRESHOLD + (BLOW_SENSOR_STEP * 1)
#define BLOW_SENSOR_LVL_2          BLOW_SENSOR_THRESHOLD + (BLOW_SENSOR_STEP * 2)
#define BLOW_SENSOR_LVL_3          BLOW_SENSOR_THRESHOLD + (BLOW_SENSOR_STEP * 3)
#define BLOW_SENSOR_LVL_4          BLOW_SENSOR_THRESHOLD + (BLOW_SENSOR_STEP * 4)
#define BLOW_SENSOR_LVL_5          BLOW_SENSOR_THRESHOLD + (BLOW_SENSOR_STEP * 5)
#define BLOW_SENSOR_LVL_6          BLOW_SENSOR_THRESHOLD + (BLOW_SENSOR_STEP * 6)
#define BLOW_SENSOR_LVL_7          BLOW_SENSOR_THRESHOLD + (BLOW_SENSOR_STEP * 7)
//========================================================================

/*
* Function Name :  EnvDet_Initial
*
* Syntax : void EnvDet_Initial(void)
*
* Purpose :  Initial EnvelopDetect
*
* Parameters : <IN>	none
*              <OUT> none
* Return : none
*
* Note :
*
*/

extern void EnvDet_Initial(void);

/*
* Function Name :  Set_MICSensitive_Time
*
* Syntax : void Set_MICSensitive_Time(INT16U StartTime,INT16U StopTime)
*
* Purpose :  设置EnvelopDetect的启动时间/停止的时间 
*
* Parameters : <IN>	StartTime: EnvelopDetect的启动时间 
*                   StopTime : EnvelopDetect的停止时间 
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void Set_MICSensitive_Time(INT16U StartTime,INT16U StopTime);

/*
* Function Name :  Get_MICSensitive_Time
*
* Syntax : void Get_MICSensitive_Time(INT16U *StartTime,INT16U *StopTime)
*
* Purpose :  获取EnvelopDetect的启动时间/停止的时间 
*
* Parameters : <IN>	none
*              <OUT> StartTime: EnvelopDetect的启动时间 (0~0xFFFF,	Default:0xa6)
*                    StopTime : EnvelopDetect的停止时间(0~0xFFFF,	Default:0x3000)
* Return : none
*
* Note :
*
*/
extern void Get_MICSensitive_Time(INT16U *StartTime,INT16U *StopTime);
/*
* Function Name :  Set_MICSensitive_Level
*
* Syntax : void Set_MICSensitive_Level(INT16S StartLevel,INT16S LevelH,INT16S LevelL)
*
* Purpose :  设置EnvelopDetect的触发起始值，上限值及下限值 
*
* Parameters : <IN>	StartLevel : EnvelopDetect的起始值(0~0x7FFF,	Default:0x0040)
*                   LevelH     : EnvelopDetect的上限值(0~0x7FFF,	Default:0x0300)
*                   LevelL     : EnvelopDetect的下限值(0~0x7FFF,	Default:0x0200)
*              <OUT> none
* Return : none
*
* Note :  1、当在声音较小时，检测到的值较难超过LevelH 且持续StartTime的时间，故需要比较低的值就触发检测的，可以设置StartLevel较小。 
*         在检测到声音后，后面的停止检测将使用LevelH及LevelL.LevelH可以與StartLevel一致。 
*         2、在觸發的時候，需要較快反應，故需較低的觸發值及較短的時間。 
*
*/
extern void Set_MICSensitive_Level(INT16S StartLevel,INT16S LevelH,INT16S LevelL);

/*
* Function Name :  Get_MICSensitive_Level
*
* Syntax : void Get_MICSensitive_Level(INT16S *StartLevel,INT16S *LevelH,INT16S *LevelL)
*
* Purpose :  获取EnvelopDetect的触发起始值，上限值及下限值 
*
* Parameters : <IN>	none
*              <OUT> StartLevel: EnvelopDetect的起始值 
*                   LevelH     : EnvelopDetect的上限值 
*                   LevelL     : EnvelopDetect的下限值 
* Return : none
*
* Note :
*
*/
extern void Get_MICSensitive_Level(INT16S *StartLevel,INT16S *LevelH,INT16S *LevelL);
/*
* Function Name :  EnvDet_Enable
*
* Syntax : void EnvDet_Enable(BOOLEAN en)
*
* Purpose :  Enable/Disable EnvelopDetect
*
* Parameters : <IN>	en 1:enable 0:disable
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void EnvDet_Enable(BOOLEAN en);



/*
* Function Name :  SetTrackDnSpeed
*
* Syntax : void SetTrackDnSpeed(INT16U value)
*
* Purpose :  设置EnvelopDetect的TrackDn的速度 
*
* Parameters : <IN>	value: TrackDnSpeed(0~0xFFFF,	Default:0x40)
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void SetTrackDnSpeed(INT16U value);

/*
* Function Name :  GetTrackDnSpeed
*
* Syntax : INT16U GetTrackDnSpeed(void)
*
* Purpose :  获取EnvelopDetect的TrackDn的速度 
*
* Parameters : <IN>	none
*              <OUT> none
* Return : INT16U ：TrackDnSpeed
*
* Note :
*
*/
extern INT16U GetTrackDnSpeed(void);

/*
* Function Name :  GetEnvelopeData
*
* Syntax : INT16S GetEnvelopeData(void)
*
* Purpose :  获取EnvelopDetect当前的Envelope值 
*
* Parameters : <IN> none
*              <OUT> none
* Return : INT16S : 当前的Envelope值 
*
* Note :
*
*/
extern INT16S GetEnvelopeData(void);
/*
* Function Name :  Get_MicReCordStart
*
* Syntax : INT8U Get_MicReCordStart(void)
*
* Purpose :  获取当前的检测状态 
*
* Parameters : <IN>	none
*              <OUT> none
* Return : INT8U :1:检测到，0:没有检测到 
*
* Note :
*
*/
extern INT8U Get_MicReCordStart(void);

/*
* Function Name :  UpdateEnvelop
*
* Syntax : INT32S UpdateEnvelop(const INT16S* buffer_addr,INT32U cwlen)
*
* Purpose :  检测Envelop
*
* Parameters : <IN>	buffer_addr: 准备检测的数据的buff地址 
*                   cwlen      : 准备检测的数据的size
*              <OUT> none
* Return :
*
* Note :
*
*/
extern INT32S UpdateEnvelop(const INT16S* buffer_addr,INT32U cwlen);




/*
* Function Name :  BlowSensorISR
*
* Syntax : void BlowSensorISR(INT16U ad_val)
*
* Purpose :  BlowSensor的中断函数 
*
* Parameters : <IN>	ad_val:ad的采样值 
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void BlowSensorISR(INT16U ad_val);

/*
* Function Name :  GetBlowSensorData
*
* Syntax : INT16S GetBlowSensorData(void)
*
* Purpose :  获取BlowSensor当前的Envelope值 
*
* Parameters : <IN>	none
*              <OUT> none
* Return : INT16U: 当前的Envelope值 
*
* Note :
*
*/
extern INT16U GetBlowSensorData(void);

/*
* Function Name :  BlowSensor_Enable
*
* Syntax : void BlowSensor_Enable(BOOLEAN en)
*
* Purpose :  Enable/Disable BlowSensor
*
* Parameters : <IN>	none
*              <OUT> none
* Return : none
*
* Note :
*
*/

extern void BlowSensor_Enable(BOOLEAN en);

/*
* Function Name :  BlowSensorInit
*
* Syntax : void BlowSensorInit(void)
*
* Purpose :  Initial BlowSensor
*
* Parameters : <IN>	none
*              <OUT> none
* Return :
*
* Note :
*
*/
extern void BlowSensorInit(void);


/*
* Function Name :  BlowInit
*
* Syntax : void BlowInit(void)
*
* Purpose :  Initial Blow
*
* Parameters : <IN>	none
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void BlowInit(void);

/*
* Function Name :  BlowStop
*
* Syntax : void BlowStop(void)
*
* Purpose :  Stop Blow
*
* Parameters : <IN>	none
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void BlowStop(void);
extern INT8U GetBlowLevel(void);
//====================================================================
#endif //__ENVELOPDETECT_H__
