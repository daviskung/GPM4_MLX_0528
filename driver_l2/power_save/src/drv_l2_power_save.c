/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2014 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#include <stdio.h>
#include <string.h>
#include "project.h"
#include "typedef.h"
#include "drv_l1_cfg.h"
#include "drv_l1_sfr.h"
#include "drv_l1_clock.h"
#include "drv_l2_power_save.h"

#include "drv_l1_watchdog.h"

#include "drv_l1_scaler.h"
#include "drv_l1_jpeg.h"
#include "drv_l1_dma.h"
#include "drv_l1_usbd.h"
#include "drv_l1_sfr.h"




static INT32U temp0,temp1,*temp2,temp, sys_clk_div;
static INT32U temp3[4], ddr_180parm[13];


static INT32U systempll_index, system_pll;
static ddr_parameter_t ddr_parameter_temp;
static INT32U *rom_start_address, *ram_end_address, *ram_target_address;

extern uint32_t SystemCoreClock;
extern INT32U isys1_image_base, _eisys1, isys1_load_base, isys1_size, Reset_Handler, __isr_vector;


/*-----------------------------------------------------------*/
void Iram_copy(void)
{
    ram_target_address = &isys1_image_base;
    temp0 = (INT32U)&isys1_image_base + (INT32U)&isys1_size;
    ram_end_address = (INT32U*)temp0;
    rom_start_address = &isys1_load_base;

    while(!(ram_target_address> ram_end_address))
    {

    *ram_target_address++ = *rom_start_address++;

    }
}

#if 1
////////////////////////////////////////////////////////////////////////////////////////////////
#define polling_timeout           3
#define status_ok                 0

#define sd_polling_1s             12500
#define sd_polling_3s             37000

void polling_sd(INT32U mode)
{
	INT32S module_status,i;
	INT32U sd_polling_cnt;

	sd_polling_cnt = sd_polling_1s;
	if(mode)
        sd_polling_cnt = sd_polling_3s;

	//polling status for SD_CTRL_BUSY and SD_CAED_CTRL_BUSY, set timeout=10*2.67us
	if((R_SDC0_CTRL & 0x800)==0x800){
	   for(i=0;i < polling_timeout*sd_polling_cnt;i++){
	       module_status=drvl1_sdc_status_clear_wait(0, 3, 10);
 	       if(module_status==status_ok)
 	          break;
 	       #if (defined _DRV_L1_WATCHDOG) && (_DRV_L1_WATCHDOG == 1)
           watchdog_clear();
           #endif
	   }

	   if(i == polling_timeout*sd_polling_cnt){
            DBG_PRINT("sd0 polling timeout\r\n");
	   }
	}

	if((R_SDC1_CTRL & 0x800)==0x800){
	   for(i=0;i < polling_timeout*sd_polling_cnt;i++){
	       module_status=drvl1_sdc_status_clear_wait(1, 3, 10);
 	       if(module_status==status_ok)
 	          break;
 	       #if (defined _DRV_L1_WATCHDOG) && (_DRV_L1_WATCHDOG == 1)
           watchdog_clear();
           #endif
	   }

	   if(i == polling_timeout*sd_polling_cnt){
            DBG_PRINT("sd1 polling timeout\r\n");
	   }
	}
}

#define NAND_CTRL_BUSY            0x8000
#define NAND_DMA_CTRL_BUSY        0x1

INT32S nand_status_get(void)
{
    INT32U nand_status;
    if(((R_NF_CTRL & NAND_CTRL_BUSY)==0x8000)&&((R_NF_DMA_CTRL & NAND_DMA_CTRL_BUSY)==0))
      nand_status=status_ok;
    else
      nand_status=-1;

    return nand_status;
}

void polling_nand(void)
{
	INT32S module_status,i;
	//polling nand status
	if((R_NF_CTRL & 0x1000)==0x1000){
	   for(i=0;i < polling_timeout;i++){
	       module_status= nand_status_get();
	       if(module_status==status_ok)
		      break;
	   }

	   if(i == polling_timeout){
            DBG_PRINT("nand busy\r\n");
	   }
	}
}

void polling_scaler(void)
{
	INT32S module_status,i;
	//polling scaler status
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_scaler_status_polling(SCALER_0);
	   if(module_status==C_SCALER_STATUS_STOP)
		  break;
	   else if(module_status==C_SCALER_STATUS_DONE)
	      break;
	   else if(module_status==C_SCALER_STATUS_INPUT_EMPTY)
		   break;
	}

	if(i == polling_timeout){
        DBG_PRINT("scaler0 busy\r\n");
	}

#if SCALER1_EN == 1
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_scaler_status_polling(SCALER_1);
	   if(module_status==C_SCALER_STATUS_STOP)
		  break;
	   else if(module_status==C_SCALER_STATUS_DONE)
	      break;
	   else if(module_status==C_SCALER_STATUS_INPUT_EMPTY)
          break;
	}

	if(i == polling_timeout){
        DBG_PRINT("scaler1 busy\r\n");
	}

#endif
}

void polling_jpeg(void)
{
	INT32S module_status,i;
	if(((R_JPG_CTRL & 0x2)==0x2)||((R_JPG_CTRL & 0x1)==0x1)){
	   //polling jpeg status without wait done
	   for(i=0;i < polling_timeout;i++){
	       module_status=drv_l1_jpeg_status_polling(0);
	       if(module_status==C_JPG_STATUS_STOP)
		      break;
	   }

	   if(i == polling_timeout){
            DBG_PRINT("jpeg busy\r\n");
	   }
	}
}

void polling_dma(void)
{
	INT32S module_status,i;

	////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH0);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma0 busy\r\n");
	}

	////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH1);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma1 busy\r\n");
	}

	////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH2);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma2 busy\r\n");
	}

	////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH3);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma3 busy\r\n");
	}

	////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH4);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma4 busy\r\n");
	}

	////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH5);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma5 busy\r\n");
	}

    ////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH6);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma6 busy\r\n");
	}

    /////////////////////////////
	for(i=0;i < polling_timeout;i++){
	   module_status=drv_l1_dma_status_get(C_DMA_CH7);
	   if(module_status==status_ok)
		  break;
	}
	if(i == polling_timeout){
        DBG_PRINT("dma7 busy\r\n");
	}
}

void polling_dac(void)
{
	INT32S module_status,i;
	//polling adc status
	if(((R_DAC_CHA_CTRL & 0x2000)==0x2000)||((R_DAC_CHB_CTRL & 0x2000)==0x2000)){
	    for(i=0;i < polling_timeout;i++){
            if( (drv_l1_dac_dma_status_get()== 1) || (drv_l1_dac_dbf_status_get()== 1) )
            {
                continue;
            }
            else{
                break;
            }
        }

        if(i == polling_timeout){
            DBG_PRINT("adc busy\r\n");
        }
	}
}

void polling_spu(void)
{
	INT32S module_status,i;
	//polling spu status
	for(i=0;i < polling_timeout;i++){
	    module_status=SPU_GetChannelStatus();
	    if(module_status==status_ok)
		   break;
	}

	if(i == polling_timeout){
        DBG_PRINT("spu busy\r\n");
	}
}

void polling_uart(void)
{
	INT32S i;
	//polling uart status
	if((R_UART0_CTRL & 0x1000)==0x1000){
	    for(i=0;i < polling_timeout;i++){
            if( (R_UART0_STATUS&0x8) == 0 ){
                break;
            }
	    }

	    if(i == polling_timeout){
            DBG_PRINT("uart0 busy\r\n");
	    }
	}

	if((R_UART1_CTRL & 0x1000)==0x1000){
	    for(i=0;i < polling_timeout;i++){
            if( (R_UART1_STATUS&0x8) == 0 ){
                break;
            }
	    }

	    if(i == polling_timeout){
            DBG_PRINT("uart1 busy\r\n");
	    }
	}

	if((R_UART2_CTRL & 0x1000)==0x1000){
	    for(i=0;i < polling_timeout;i++){
            if( (R_UART2_STATUS&0x8) == 0 ){
                break;
            }
	    }

	    if(i == polling_timeout){
            DBG_PRINT("uart2 busy\r\n");
	    }
	}
}

#define ADC_CTRL_BUSY             0x80

INT32S adc_status_get(void)
{
   INT32U adc_status;
	  if((R_ADC_MADC_CTRL & ADC_CTRL_BUSY)==0x80)
          adc_status=status_ok;
	  else
		  adc_status=-1;

	  return adc_status;
}

void polling_adc(void)
{
	INT32S module_status,i;
	//polling adc status
	if((R_ADC_SETUP & 0x4000)==0x4000){
	    for(i=0;i < polling_timeout;i++){
	       module_status=adc_status_get();
	       if(module_status==status_ok)
		      break;
	       else
		      DBG_PRINT("adc busy\r\n");
	    }
	}
}

INT32S usb_status_get(void)
{
   INT32U usb_status;
	  if((rUDC_EP12DMA & MASK_USBD_EP12_DMA_EN)==0)
          usb_status=status_ok;
	  else
		  usb_status=-1;

	  if((rUDC_EP7DMA & MASK_USBD_EP7_DMA_EN)==0)
	  		  usb_status=status_ok;
	  else
		  usb_status=-1;

	  if((rEP89_DMA & MASK_USBD_EP89_DMA_EN)==0)
		  usb_status=status_ok;
	  else
		  usb_status=-1;

	  if((rEPAB_DMA & MASK_USBD_EPAB_DMA_EN)==0)
		  usb_status=status_ok;
	  else
		  usb_status=-1;


	  return usb_status;
}

void polling_usb(void)
{
	INT32S module_status,i;
	//polling usb status
	for(i=0;i < polling_timeout;i++){
	    module_status=usb_status_get();
	    if(module_status==status_ok)
		   break;
	    else
		   DBG_PRINT("usb busy\r\n");
	}
}


#define DEFLICKER_CTRL_BUSY       0x100

INT32S deflicker_status_get(void)
{
   INT32U deflicker_status;
	  if((R_DEFLICKER_CTRL & DEFLICKER_CTRL_BUSY)==0)
		  deflicker_status=status_ok;
	  else
		  deflicker_status=-1;

	  return deflicker_status;
}

void polling_deflicker(void)
{
	INT32S module_status,i;
	//polling usb status
	for(i=0;i < polling_timeout;i++){
	    module_status=deflicker_status_get();
	    if(module_status==status_ok)
		   break;
	    else
		   DBG_PRINT("deflicker busy\r\n");
	}
}

void module_pollingstatus(void)
{
	DBG_PRINT("Start Polling\r\n");
	polling_sd(1);
	polling_nand();
	polling_scaler();
	polling_jpeg();
	polling_dma();
	polling_dac();
	if (system_pll>36000000)
	{
        polling_spu();
    }
	polling_uart();
	polling_adc();
	polling_usb();
	polling_deflicker();
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
#endif


INT32S system_module_check(void)
{
    temp0 = R_SYSTEM_CLK_EN0;
    temp = SUCCESS;
    if ((temp0>>2) & 0x1)  //check psca0
    {
        temp1 = *PIPELINE_SCALERA_BASE;
        if(temp1 & 0x01)
        {
            DBG_PRINT("PSCALA Active\r\n");
            temp = FAIL;
        }
    }
    if ((temp0>>3) & 0x1)   //check HDMI
    {
        temp1 = R_HDMI_EN;
        if((temp1>>30) & 0x3)
        {
            DBG_PRINT("HDMI Active\r\n");
            temp = FAIL;
        }
    }
    if(0) // bypass timer
    {
    if((temp0>>4) & 0x1)  //Timer check
    {
        temp1 = R_TIMERA_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerA Active\r\n");
            temp = FAIL;
        }
        temp1 = R_TIMERB_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerB Active\r\n");
            temp = FAIL;
        }
        temp1 = R_TIMERC_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerC Active\r\n");
            temp = FAIL;
        }

        temp1 = R_TIMERD_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerD Active\r\n");
            temp = FAIL;
        }

        temp1 = R_TIMERE_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerE Active\r\n");
            temp = FAIL;
        }
        temp1 = R_TIMERF_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerF Active\r\n");
            temp = FAIL;
        }
        temp1 = R_TIMERG_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerG Active\r\n");
            temp = FAIL;
        }
        temp1 = R_TIMERH_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerH Active\r\n");
            temp = FAIL;
        }
    }
    }
    if((temp0>>6) & 0x01)  //DAC, DC_Mixer, I2S_TX
    {
        temp1 = R_DAC_CHA_CTRL;
        if((temp1>>13)&0x01)
        {
            temp = R_TIMERE_CTRL; //check timerE
            if((temp>>13) & 0x1)
            {
                DBG_PRINT("DAC CHA Active\r\n");
                temp = FAIL;
            }
        }
        temp1 = R_DAC_CHB_CTRL;
        if((temp1>>13)&0x01)
        {
            temp = R_TIMERE_CTRL; //check timerE
            if((temp>>13) & 0x1)
            {
                DBG_PRINT("DAC CHB Active\r\n");
                temp = FAIL;
            }
        }

        temp1 = R_I2S0TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S0 TX Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2S1TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S1 TX Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2S2TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S2 TX Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2S3TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S3 TX Active\r\n");
            temp = FAIL;
        }
    }
    // no check UART and IrDA
    if((temp0>>8)&0x01)   //check CEC, CAN, TimeBase, I2C
    {
        temp1 = R_CEC_CTRL;
        if(temp1 &0x01)
        {
            DBG_PRINT("CEC Active\r\n");
            temp = FAIL;
        }
        temp1 = R_CAN_MCTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("CAN Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2C0_MISC;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2C0 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2C1_MISC;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2C1 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2C2_MISC;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2C2 Active\r\n");
            temp = FAIL;
        }
    }
    if((temp0>>9) & 0x01)  //pscale B
    {
        temp1 = *PIPELINE_SCALERB_BASE;
        if(temp1 & 0x01)
        {
            DBG_PRINT("PSCALB Active\r\n");
            temp = FAIL;
        }
    }
    if((temp0>>10) &0x01)  //codec ADC, I2S_RX
    {
        temp1 = R_MIC_SETUP;
        if(temp1 & 0x07)
        {
            DBG_PRINT("MIC or Line IN Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2S0RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S0 RX Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2S1RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S1 RX Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2S2RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S2 RX Active\r\n");
            temp = FAIL;
        }
        temp1 = R_I2S3RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S3 RX Active\r\n");
            temp = FAIL;
        }
    }
    if((temp0>>11) & 0x1)  //h264 decode, h264 scalar, MB2SCAN
    {
            DBG_PRINT("H264 decode Active\r\n");
            temp = FAIL;

    }
    if((temp0>>12) &0x01)
    {
        if (system_pll>36000000)
        {
            temp1 = *P_SPU_CH_EN;     // SPU CLK = PLL/4, system clk must be high than SPU_CLK
            if(temp1)
            {
                DBG_PRINT("SPU Active\r\n");
                temp = FAIL;
            }
        }
    }
    if((temp0>>13) &0x01)
    {
            DBG_PRINT("TFT Active\r\n");
            temp = FAIL;
    }
    if((temp0>>14) &0x01)
    {
            DBG_PRINT("ISP Active\r\n");
            temp = FAIL;
    }
    if((temp0>>15) &0x01)
    {
        temp1 = R_SDC0_CTRL;
        if((temp1>>11) & 0x01)
        {
            DBG_PRINT("SDC0 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_SDC1_CTRL;
        if((temp1>>11) & 0x01)
        {
            DBG_PRINT("SDC1 Active\r\n");
            temp = FAIL;
        }
    }
    temp0 = R_SYSTEM_CLK_EN1;
    if((temp0) & 0x01) // FD and rotation (bit16)
    {
        temp1 = R_ROTATOR_START;
        if(temp1 & 0x01)
        {
        DBG_PRINT("Rotation Active\r\n");
        temp = FAIL;
        }
        temp1 = R_FD_CTRL0;
        if(temp1 & 0x01)
        {
        DBG_PRINT("FD Active\r\n");
        temp = FAIL;
        }
    }
    if((temp0>>1)&0x01)  //USB host (bit17)
    {
            DBG_PRINT("USBH Active\r\n");
            temp = FAIL;
    }
    if((temp0>>2)&0x01)  //USB device  (bit18)
    {
            DBG_PRINT("USBD Active\r\n");
            temp = FAIL;
    }
    if((temp0>>3)&0x01)  //PPU  (bit19)
    {
            DBG_PRINT("PPU Active\r\n");
            temp = FAIL;
    }
    if((temp0>>4)&0x01)  //DMA  (bit20)
    {
        temp1 = R_DMA0_CTRL;
        if((temp1) & 0x01)  //channel enable
        {
            DBG_PRINT("DMA0 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_DMA1_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA1 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_DMA2_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA2 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_DMA3_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA3 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_DMA4_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA4 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_DMA5_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA5 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_DMA6_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA6 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_DMA7_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA7 Active\r\n");
            temp = FAIL;
        }

    }
    if((temp0>>5)&0x01)  //H264 encode  (bit21)
    {
            DBG_PRINT("H264 encode Active\r\n");
            temp = FAIL;
    }
    if((temp0>>6)&0x01)  //display frame buffer and sensor  (bit22)
    {
        temp1 = R_CSI_TG_CTRL0;
        if((temp1) & 0x01)  // module enable bit
        {
            temp1 = R_CSI_TG_CTRL1;
            if((temp1>>7) & 0x01) //CLK enable
            {
                DBG_PRINT("H264 encode Active\r\n");
                temp = FAIL;
            }
        }
    }
    if((temp0>>7)&0x01)  //FFT and Object  (bit23)
    {
        temp1 = P_FFT_START;
        if(temp1 & 0x01)  //start bit
        {
            DBG_PRINT("FFT Active\r\n");
            temp = FAIL;
        }
    }
    if((temp0>>8)&0x01)  //SPIFC and SPI I/F  (bit24)
    {
        temp1 = R_SPI0_CTRL;
        if((temp1>>15) & 0x01)
        {
            DBG_PRINT("SPI0 Active\r\n");
            temp = FAIL;
        }
        temp1 = R_SPI1_CTRL;
        if((temp1>>15) & 0x01)
        {
            DBG_PRINT("SPI1 Active\r\n");
            temp = FAIL;
        }
    }
    if((temp0>>9)&0x01)  //MIPI Aand B  (bit25)
    {
        temp1 = R_MIPIA_GLB_CSR;
        if(temp1 & 0x01)
        {
            DBG_PRINT("MIPIA Active\r\n");
            temp = FAIL;
        }
        temp1 = R_MIPIB_GLB_CSR;
        if(temp1 & 0x01)
        {
            DBG_PRINT("MIPIB Active\r\n");
            temp = FAIL;
        }
    }
    if((temp0>>10)&0x01)  //jpeg, conv420 to 422  (bit26)
    {
        temp1 = R_JPG_CTRL;
        if(temp1 & 0x3)  //check JPEG codec start bit
        {
            DBG_PRINT("JPEG codec Active\r\n");
            temp = FAIL;
        }
        temp1 = (*((volatile INT32U*)0xC0180000));  //conv420 to 422
        if((temp1>>30) & 0x01)  //check busy
        {
            DBG_PRINT("conv420to422 Active\r\n");
            temp = FAIL;
        }
    }

    /* don't care
    if((temp0>>11)&0x01)  //Display Output Reg  (bit27)
    {
            DBG_PRINT("H264 encode Active\r\n");
           temp = FAIL;
    }
    */
    if((temp0>>12)&0x01)  //Scalar  (bit28)
    {
        temp1 = R_SCA_CON;
        if((temp1>>8) & 0x01)
        {
            DBG_PRINT("Scalar Active\r\n");
            temp = FAIL;
        }
    }
    if((temp0>>13)&0x01)  //NAND and BCH  (bit29)
    {
            DBG_PRINT("NAND flash Active\r\n");
            temp = FAIL;
    }
    /* don't care
    if((temp0>>14)&0x01)  //H264AVS   (bit30)
    {
            DBG_PRINT("H264 AVS Active\r\n");
            temp = FAIL;
    }
    */


    return temp;
}

/***************************************************************
* 1.if systempll >= 843750Hz, will change PLL, and only support
*   0.84375MHz,1.6875MHz,3.375MHz, 6.75MHz,9MHz,18MHz,27MHz,36MHz,.....387MHz
* 2. SPU CLK must be equal or more than system clk.
* 3. if systempll = 32768Hz, DDR will goto self refresh and CPU run 32768Hz in iRAM
* 4. if systempll = 12 meaning CLK source change to 12MHz XTAL.
* 5. if systempll = 0, system will goto power off ( DDR LDO OFF and DC2DC_EN OFF)
* 6. return SystemCoreClock*2
***************************************************************/



INT32S drv_l2_change_system_speed(int systempll)   //systempll =  xxHz
{
    volatile register int dummy = 1;


    system_pll = systempll;
    if((system_pll>C_MAX_PLL))
    {
        DBG_PRINT("systempll out of range\r\n");
        system_pll = C_MAX_PLL;
    }

    if(system_module_check())
    {
        DBG_PRINT("change system clock fail\r\n");

       // return FAIL;
    }

    module_pollingstatus();

    cache_disable();

    if(system_pll==0)
    goto _core_power_off_mode;


/*************************************************************************
*  change CPU CLK to 32KHz if systempll = C_32KHz
**************************************************************************/

    if(system_pll == C_32KHz)
        goto  _32khz_working;

/*************************************************************************
*  change C_12M from 12M Xtal
**************************************************************************/



    if(system_pll == C_12MXTAL)
        goto _12xtal_change;
/**************************************************************************/

    sys_clk_div = 0;
    if(system_pll<C_SYSX2_09M && system_pll> C_32KHz )
    {
        systempll_index = C_MIN_PLL/C_MHz/9 -3;
        temp = system_pll *2;
        for (sys_clk_div=1; temp <= C_MIN_PLL;sys_clk_div++)
        {
            temp *=2;
        }
        sys_clk_div--;
        system_pll = C_MIN_PLL >> sys_clk_div;
    }
    else if(system_pll<C_MIN_PLL )
    {
        temp = system_pll/9000000;
        system_pll = temp * C_SYSX2_09M;
        switch(system_pll)
        {
            case C_SYSX2_45M:
                sys_clk_div = 1;
                systempll_index = 0x7;   //90M
                break;
            case C_SYSX2_36M:
                sys_clk_div = 1;
                systempll_index = 0x5;   //72M
                break;
            case C_SYSX2_27M:
                sys_clk_div = 1;
                systempll_index = 0x3;   //54M
                break;
            case C_SYSX2_18M:
                sys_clk_div = 2;            //4
                systempll_index = 0x5;   //72M
                break;
            case C_SYSX2_09M:
                sys_clk_div = 3;            //8
                systempll_index = 0x5;   //72M
                break;

        }

    }else if (system_pll>= C_MIN_PLL)
    {
        systempll_index = system_pll/C_MHz/9 - 3;
        system_pll = systempll_index*9*C_MHz + C_MIN_PLL/2;
    }

    ddr_parameter_temp.ddr_timing1 = ddr_init_table[systempll_index].ddr_timing1;
    ddr_parameter_temp.ddr_timing2 = ddr_init_table[systempll_index].ddr_timing2;
    ddr_parameter_temp.ddr_tref = ddr_init_table[systempll_index].ddr_tref;
    ddr_parameter_temp.ddr_init_v = ddr_init_table[systempll_index].ddr_init_v;
    temp0 = R_SYSTEM_CLK_EN0;
    temp1 = R_SYSTEM_CLK_EN1;
    R_SYSTEM_CLK_EN0 = (1<<CLK_EN0_SYSTEM_BUS) | (1<<CLK_EN0_MEMORY_DRAM) | (1<<CLK_EN0_CPU); /*system bus + memory + cpu I/F */
    R_SYSTEM_CLK_EN1 = (1<<(CLK_EN1_SYSTEM_CTRL-16));   /* system control */
    // get calibration data
    temp2 = (INT32U*) R_AC_INIT_TADDR;
    temp3[0] = *temp2++;
    temp3[1] = *temp2++;
    temp3[2] = *temp2++;
    temp3[3] = *temp2;


/*********************************
*  ddr_to_self_refresh flow
*********************************/


    R_DDR_CTL2 |= 0x1<<4;
    temp =R_DDR_CTL2;
    while (!(temp&(0x1<<8)))
    {
        temp =R_DDR_CTL2;

    }



_change_speed:

    R_SYSTEM_CLK_EN0 = (1<<CLK_EN0_SYSTEM_BUS) | (1<<CLK_EN0_MEMORY_DRAM) | (1<<CLK_EN0_CPU); //system bus + memory + cpu I/F
    R_DDR_CTL2 &= ~(0x1<<4);
    temp = R_DDR_CTL2;
    while ((temp&(0x1<<8)))
        temp = R_DDR_CTL2;

    asm volatile("nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 );
    asm volatile("nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 );

    R_DDR_CTL = 0x0; // DDR disable

/***********************************************************************
*  DDR auto calibration flow: setting CAS/DLL/ Delay Timinig
*
*
************************************************************************/


    if(system_pll> 198000000)
    {
        R_DDR_CAS = 0x3; //cas = 3 system = 100MHz~200MHz DDR SPEC
        R_DDR_EMR &= ~0x1;  //DLL ON
        R_DDR_RD_CTRL &= ~(0xF);

    }else if(system_pll> 162000000)
    {
        R_DDR_CAS = 0x2; //cas = 2 system = 83MHz~133MHz
        R_DDR_EMR &= ~0x1;  //DLL ON
        R_DDR_RD_CTRL &= ~(0xF);

    }else
    {
        R_DDR_CAS = 0x2; //cas = 2
        R_DDR_EMR |= 0x1;  //DLL OFF --> system < 83MHz
        temp = R_DDR_RD_CTRL & (~0xF);
        temp |= (0x1<<3);
        R_DDR_RD_CTRL = temp;
//        R_DDR_EMR &= ~0x1;  //DLL ON

     }

    temp = R_SYSTEM_CLK_CTRL & (~(0x07));

    R_SYSTEM_CLK_CTRL = temp | sys_clk_div;
    temp = R_SYSTEM_PLLEN & (~(0x3F));

    R_SYSTEM_PLLEN = temp | systempll_index;
    //delay 1 ms

    for(temp=0;temp<(systempll_index*9+27);temp++)
    {
        temp2 = (INT32U*) R_AC_INIT_TADDR;
        temp2 += 2;
        asm volatile("nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 );
    }
    do{
        R_DDR_CTL = 0x0; // DDR disable
        R_DDR_TIMING2 = 0x0000AAAA;
        R_AC_CTRL = 0x00000001; //auto calibration enable


        //set DDR 0xd0200120~012c
        R_DDR_TIMING1 = ddr_parameter_temp.ddr_timing1;
        R_DDR_TIMING2 = ddr_parameter_temp.ddr_timing2;
        if (system_pll < C_MIN_PLL)
        {
            temp = 78*system_pll/10000/2;
            if (temp> (((78*system_pll/10000/2)/1000)*1000))
                temp += 1000;

        R_DDR_TREF = temp/1000 -1;
        }else{
            R_DDR_TREF = ddr_parameter_temp.ddr_tref;
        }

        R_DDR_INIT_V = ddr_parameter_temp.ddr_init_v;

        R_AC_STEP = 0x00000222;
        R_DDR_CTL = 0x3; //set DDR size & DDR enable
        asm volatile("nop \n"
                    "nop \n"
                    "nop \n"
                    "nop \n"
                    "nop \n"
                    );

        while((R_AC_CTRL & 0x1) ==0x1);
//ddr calibration end

        temp = R_AC_PASSCNT;
    }while (!(((temp & 0xff)>4)&&(((temp>>8) & 0xff)>4)));

    temp2 = (INT32U*) R_AC_INIT_TADDR;
    *temp2++ = temp3[0];
    *temp2++ = temp3[1];
    *temp2++ = temp3[2];
    *temp2 = temp3[3];
    R_SYSTEM_CLK_EN0 = temp0;
    R_SYSTEM_CLK_EN1 = temp1;

    SystemCoreClock = system_pll/2;
    MCLK = SystemCoreClock;
    MHZ = MCLK /1000000;
    drv_l1_uart1_buad_rate_set(UART0_BAUD_RATE);
    cache_enable();

    return SystemCoreClock*2;


/**************************************
**      ddr_to_self_refresh and power off (DC2DC_EN off)
***************************************/

_core_power_off_mode:

    R_SYSTEM_CLK_EN0 = (1<<CLK_EN0_SYSTEM_BUS) | (1<<CLK_EN0_MEMORY_DRAM) | (1<<CLK_EN0_CPU); //system bus + memory + cpu I/F
    R_SYSTEM_CLK_EN1 = (1<<(CLK_EN1_SYSTEM_CTRL-16));   // system control
    for(temp=0;temp<(19*9+27);temp++)
    {
        temp2 = (INT32U*) R_AC_INIT_TADDR;
        temp2 += 2;
        asm volatile("nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 );
    }

    R_DDR_CTL2 |= 0x1<<4;
    temp =R_DDR_CTL2;
    while (!(temp&(0x1<<8)))
    {
        temp =R_DDR_CTL2;

    }
    //power down mode
    R_SYSTEM_DDR_LDO = 0x0;
    R_SYSTEM_POWER_CTRL0 &= ~0x1;
    while(1);

/***********************************************************
*  Change system/cpu clk to 23KHz and ddr goto self refresh
************************************************************/
_32khz_working:

    systempll_index = R_SYSTEM_PLLEN & 0x3F;
    system_pll = systempll_index * C_SYSX2_09M + C_SYSX2_27M;
    sys_clk_div = R_SYSTEM_CLK_CTRL & 0x07;
    system_pll >>= sys_clk_div;

    ddr_parameter_temp.ddr_timing1 = ddr_init_table[systempll_index].ddr_timing1;
    ddr_parameter_temp.ddr_timing2 = ddr_init_table[systempll_index].ddr_timing2;
    ddr_parameter_temp.ddr_tref = ddr_init_table[systempll_index].ddr_tref;
    ddr_parameter_temp.ddr_init_v = ddr_init_table[systempll_index].ddr_init_v;
    temp0 = R_SYSTEM_CLK_EN0;
    temp1 = R_SYSTEM_CLK_EN1;
    R_SYSTEM_CLK_EN0 = (1<<CLK_EN0_SYSTEM_BUS) | (1<<CLK_EN0_MEMORY_DRAM) | (1<<CLK_EN0_CPU); /*system bus + memory + cpu I/F */
    R_SYSTEM_CLK_EN1 = (1<<(CLK_EN1_SYSTEM_CTRL-16)) | (1<<(CLK_EN1_DMA-16));   /* system control */
    // get calibration data
    temp2 = (INT32U*) R_AC_INIT_TADDR;
    temp3[0] = *temp2++;
    temp3[1] = *temp2++;
    temp3[2] = *temp2++;
    temp3[3] = *temp2;

    R_DDR_CTL2 |= 0x1<<4;
    temp =R_DDR_CTL2;
    while (!(temp&(0x1<<8)))
    {
        temp =R_DDR_CTL2;

    }

    R_SYSTEM_CLK_EN0 = (1<<CLK_EN0_SYSTEM_BUS) | (1<<CLK_EN0_CPU); //stop DDR CLK
    R_SYSTEM_CTRL |= 0x1;    //select 32KHz from internal 32KH
    R_SYSTEM_CLK_CTRL |= 0x4000;   //change system CLK to 32KHz

    (*((volatile INT32U *) 0xC000000C)) |= (0x1<<11); // IOA11
    (*((volatile INT32U *) 0xC0000008)) |= (0x1<<11); // IOA11

    while(1)
    {
        (*((volatile INT32U *) 0xC0000004)) ^= (0x1<<11); // IOA11
        if((*((volatile INT32U *) 0xC0000000))& (0x1<<13))
        break;
    }

    R_SYSTEM_CLK_CTRL &= ~0x4000;   //change system CLK to PLL
    R_SYSTEM_CTRL &= ~0x1;    //select 32KHz from internal 32KHz


    goto _change_speed;

/******************************************************************************
*   change C_12MHz by 12M Xtal (default  C_12MHz from XCKGEN (32KHz xtal))
********************************************************************************/


_12xtal_change:
    (*((volatile INT32U *) 0xC000000C)) |= (0x1<<11); // IOA11
    (*((volatile INT32U *) 0xC0000008)) |= (0x1<<11); // IOA11

    temp0 = R_SYSTEM_CLK_EN0;
    temp1 = R_SYSTEM_CLK_EN1;
    R_SYSTEM_CLK_EN0 = (1<<CLK_EN0_SYSTEM_BUS) | (1<<CLK_EN0_MEMORY_DRAM) | (1<<CLK_EN0_CPU); /*system bus + memory + cpu I/F */
    R_SYSTEM_CLK_EN1 = (1<<(CLK_EN1_SYSTEM_CTRL-16)) | ((1<<CLK_EN1_DMA-16));   /* system control */
    // get calibration data

    R_DDR_CTL2 |= 0x1<<4;
    temp =R_DDR_CTL2;
    while (!(temp&(0x1<<8)))
    {
        temp =R_DDR_CTL2;

    }

    R_SYSTEM_CLK_CTRL &= ~0x8000;   //change system CLK to SLOW
    R_SYSTEM_CKGEN_CTRL |= 0x120;   // enable 12M xtal and change C_12M from 12M Xtal
    (*((volatile INT32U *) 0xC0000004)) ^= (0x1<<11); // IOA11
    for(temp=0;temp<(19*9+27);temp++)
    {
        temp2 = (INT32U*) R_AC_INIT_TADDR;
        temp2 += 2;
        asm volatile("nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 );
    }

    R_SYSTEM_CLK_CTRL |= 0x8000;   //change system CLK to FAST
    (*((volatile INT32U *) 0xC0000004)) ^= (0x1<<11); // IOA11
        for(temp=0;temp<(19*9+27);temp++)
    {
        temp2 = (INT32U*) R_AC_INIT_TADDR;
        temp2 += 2;
        asm volatile("nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 "nop \n"
                 );
    }
    (*((volatile INT32U *) 0xC0000004)) ^= (0x1<<11); // IOA11
    R_DDR_CTL2 &= ~(0x1<<4);
    temp = R_DDR_CTL2;
    while ((temp&(0x1<<8)))
        temp = R_DDR_CTL2;

    R_SYSTEM_CLK_EN0 = temp0;
    R_SYSTEM_CLK_EN1 = temp1;
    cache_enable();
    return SystemCoreClock*2;
}




