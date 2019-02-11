
#include "EnvelopDetect.h"


/*
* Function Name :  GetBlowLevel
*
* Syntax : INT8U GetBlowLevel(void)
*
* Purpose :  获取Blow的Level
*
* Parameters : <IN>	none
*              <OUT> none
* Return : INT8U: Blow的Level
*
* Note :
*
*/

INT8U GetBlowLevel(void)
{
  INT16S BlowSensorLvl;

  BlowSensorLvl = GetBlowSensorData();
  if(BlowSensorLvl <= BLOW_SENSOR_THRESHOLD) {
    return 0;
  } else if((BlowSensorLvl > BLOW_SENSOR_THRESHOLD) && (BlowSensorLvl <= BLOW_SENSOR_LVL_1)) {
    return 1;
  } else if((BlowSensorLvl > BLOW_SENSOR_LVL_1) && (BlowSensorLvl <= BLOW_SENSOR_LVL_2)) {
    return 2;
  } else if((BlowSensorLvl > BLOW_SENSOR_LVL_2) && (BlowSensorLvl <= BLOW_SENSOR_LVL_3)) {
    return 3;
  } else if((BlowSensorLvl > BLOW_SENSOR_LVL_3) && (BlowSensorLvl <= BLOW_SENSOR_LVL_4)) {
    return 4;
  } else if((BlowSensorLvl > BLOW_SENSOR_LVL_4) && (BlowSensorLvl <= BLOW_SENSOR_LVL_5)) {
    return 5;
  } else if((BlowSensorLvl > BLOW_SENSOR_LVL_5) && (BlowSensorLvl <= BLOW_SENSOR_LVL_6)) {
    return 6;
  } else if((BlowSensorLvl > BLOW_SENSOR_LVL_6) && (BlowSensorLvl <= BLOW_SENSOR_LVL_7)) {
    return 7;;
  } else {
    return 8;;
  }
}

/*
* Function Name :  BlowSensor_timer_isr
*
* Syntax : void BlowSensor_timer_isr(void)
*
* Purpose :  BlowSensor用到的timer的中断 
*
* Parameters : <IN>	none
*              <OUT> none
* Return : none
*
* Note :
*
*/
static void BlowSensor_timer_isr(void)
{
	drv_l1_adc_manual_sample_start();
}

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
void BlowInit(void)
{
    BlowSensorInit();
    //adc_manual_ch_set(ADC_LINE_1);
	drv_l1_adc_manual_callback_set(BlowSensorISR);

	timer_freq_setup(1, 8000, 0, BlowSensor_timer_isr);

}


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
void BlowStop(void)
{
    timer_stop(1);

}
