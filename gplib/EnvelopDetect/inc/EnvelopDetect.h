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
* Purpose :  ����EnvelopDetect������ʱ��/ֹͣ��ʱ�� 
*
* Parameters : <IN>	StartTime: EnvelopDetect������ʱ�� 
*                   StopTime : EnvelopDetect��ֹͣʱ�� 
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
* Purpose :  ��ȡEnvelopDetect������ʱ��/ֹͣ��ʱ�� 
*
* Parameters : <IN>	none
*              <OUT> StartTime: EnvelopDetect������ʱ�� (0~0xFFFF,	Default:0xa6)
*                    StopTime : EnvelopDetect��ֹͣʱ��(0~0xFFFF,	Default:0x3000)
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
* Purpose :  ����EnvelopDetect�Ĵ�����ʼֵ������ֵ������ֵ 
*
* Parameters : <IN>	StartLevel : EnvelopDetect����ʼֵ(0~0x7FFF,	Default:0x0040)
*                   LevelH     : EnvelopDetect������ֵ(0~0x7FFF,	Default:0x0300)
*                   LevelL     : EnvelopDetect������ֵ(0~0x7FFF,	Default:0x0200)
*              <OUT> none
* Return : none
*
* Note :  1������������Сʱ����⵽��ֵ���ѳ���LevelH �ҳ���StartTime��ʱ�䣬����Ҫ�Ƚϵ͵�ֵ�ʹ������ģ���������StartLevel��С�� 
*         �ڼ�⵽�����󣬺����ֹͣ��⽫ʹ��LevelH��LevelL.LevelH�����cStartLevelһ�¡� 
*         2�����|�l�ĕr����Ҫ�^�췴���������^�͵��|�lֵ���^�̵ĕr�g�� 
*
*/
extern void Set_MICSensitive_Level(INT16S StartLevel,INT16S LevelH,INT16S LevelL);

/*
* Function Name :  Get_MICSensitive_Level
*
* Syntax : void Get_MICSensitive_Level(INT16S *StartLevel,INT16S *LevelH,INT16S *LevelL)
*
* Purpose :  ��ȡEnvelopDetect�Ĵ�����ʼֵ������ֵ������ֵ 
*
* Parameters : <IN>	none
*              <OUT> StartLevel: EnvelopDetect����ʼֵ 
*                   LevelH     : EnvelopDetect������ֵ 
*                   LevelL     : EnvelopDetect������ֵ 
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
* Purpose :  ����EnvelopDetect��TrackDn���ٶ� 
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
* Purpose :  ��ȡEnvelopDetect��TrackDn���ٶ� 
*
* Parameters : <IN>	none
*              <OUT> none
* Return : INT16U ��TrackDnSpeed
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
* Purpose :  ��ȡEnvelopDetect��ǰ��Envelopeֵ 
*
* Parameters : <IN> none
*              <OUT> none
* Return : INT16S : ��ǰ��Envelopeֵ 
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
* Purpose :  ��ȡ��ǰ�ļ��״̬ 
*
* Parameters : <IN>	none
*              <OUT> none
* Return : INT8U :1:��⵽��0:û�м�⵽ 
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
* Purpose :  ���Envelop
*
* Parameters : <IN>	buffer_addr: ׼���������ݵ�buff��ַ 
*                   cwlen      : ׼���������ݵ�size
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
* Purpose :  BlowSensor���жϺ��� 
*
* Parameters : <IN>	ad_val:ad�Ĳ���ֵ 
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
* Purpose :  ��ȡBlowSensor��ǰ��Envelopeֵ 
*
* Parameters : <IN>	none
*              <OUT> none
* Return : INT16U: ��ǰ��Envelopeֵ 
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
