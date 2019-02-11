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
#include "drv_l2_systemclk_change.h"


INT32U temp0,temp1,*temp2,temp;
INT32U temp3[4];

INT32U systempll_index;
ddr_parameter_t ddr_parameter_temp;
INT32U *rom_start_address, *ram_end_address, *ram_target_address;

extern uint32_t SystemCoreClock;
extern INT32U isys_image_base, _eisys, isys_load_base;

/*-----------------------------------------------------------*/
void Iram_copy()
{
    ram_target_address = &isys_image_base;
    ram_end_address = &_eisys;
    rom_start_address = &isys_load_base;

    while(!(ram_target_address> ram_end_address))
    {

    *ram_target_address++ = *rom_start_address++;

    }
}



void drv_l2_ddr_to_self_refresh(void)
{
      // DDR to self refresh
    R_DDR_CTL2 |= 0x1<<4;
    temp1 =R_DDR_CTL2;
    while (!(temp1&0x1<<8))
        temp1 =R_DDR_CTL2;
}

void drv_l2_ddr_calibration(int systempll_index, ddr_parameter_t* ddr_parameter)
{

    R_DDR_EMR &= ~0x1;  //DLL ON
    if(systempll_index> 0x13)
    {
        R_DDR_CAS = 0x3; //cas = 3 system = 100MHz~200MHz DDR SPEC
    }else if(systempll_index> 0xF)
    {
        R_DDR_CAS = 0x2; //cas = 2 system = 83MHz~133MHz
    }else
    {
        R_DDR_CAS = 0x2; //cas = 2 system = 83MHz~133MHz
        R_DDR_EMR |= 0x1;  //DLL OFF
    }


    R_SYSTEM_PLLEN &= ~0x3F;
    R_SYSTEM_PLLEN |= systempll_index; //change speed to 108MHz
    R_DDR_CTL = 0x0; // DDR disable
    R_DDR_TIMING2 = 0x0000AAAA;
    R_AC_CTRL = 0x00000001; //auto calibration enable
    //set DDR 0xd0200120~012c
    R_DDR_TIMING1 = ddr_parameter->ddr_timing1;
    R_DDR_TIMING2 = ddr_parameter->ddr_timing2;
    R_DDR_TREF = ddr_parameter->ddr_tref;
    R_DDR_INIT_V = ddr_parameter->ddr_init_v;

    R_AC_STEP = 0x00000222;

    R_DDR_CTL = 0x3; //set DDR size & DDR enable
}

INT32S system_module_check(void)
{
    temp0 = R_SYSTEM_CLK_EN0;
    if ((temp0>>2) & 0x1)  //check psca0
    {
        temp1 = *PIPELINE_SCALERA_BASE;
        if(temp1 & 0x01)
        {
            DBG_PRINT("PSCALA Active\r\n");
            //return FAIL;
        }
    }
    if ((temp0>>3) & 0x1)   //check HDMI
    {
        temp1 = R_HDMI_EN;
        if((temp1>>30) & 0x3)
        {
            DBG_PRINT("HDMI Active\r\n");
            //return FAIL;
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
            //return FAIL;
        }
        temp1 = R_TIMERB_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerB Active\r\n");
            //return FAIL;
        }
        temp1 = R_TIMERC_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerC Active\r\n");
            //return FAIL;
        }

        temp1 = R_TIMERD_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerD Active\r\n");
            //return FAIL;
        }

        temp1 = R_TIMERE_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerE Active\r\n");
            //return FAIL;
        }
        temp1 = R_TIMERF_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerF Active\r\n");
            //return FAIL;
        }
        temp1 = R_TIMERG_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerG Active\r\n");
            //return FAIL;
        }
        temp1 = R_TIMERH_CTRL;
        if((temp1>>13) & 0x1)
        {
            DBG_PRINT("TimerH Active\r\n");
            //return FAIL;
        }
    }
    }
    if((temp0>>6) & 0x01)  //DAC, DC_Mixer, I2S_TX
    {
        temp1 = R_DAC_CHA_CTRL;
        if((temp1>>13)&0x01)
        {
            DBG_PRINT("DAC CHA Active\r\n");
            //return FAIL;
        }
        temp1 = R_DAC_CHB_CTRL;
        if((temp1>>13)&0x01)
        {
            DBG_PRINT("DAC CHB Active\r\n");
            //return FAIL;
        }

        temp1 = R_I2S0TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S0 TX Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2S1TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S1 TX Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2S2TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S2 TX Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2S3TX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S3 TX Active\r\n");
            //return FAIL;
        }
    }
    // no check UART and IrDA
    if((temp0>>8)&0x01)   //check CEC, CAN, TimeBase, I2C
    {
        temp1 = R_CEC_CTRL;
        if(temp1 &0x01)
        {
            DBG_PRINT("CEC Active\r\n");
            //return FAIL;
        }
        temp1 = R_CAN_MCTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("CAN Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2C0_MISC;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2C0 Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2C1_MISC;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2C1 Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2C2_MISC;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2C2 Active\r\n");
            //return FAIL;
        }
    }
    if((temp0>>9) & 0x01)  //pscale B
    {
        temp1 = *PIPELINE_SCALERB_BASE;
        if(temp1 & 0x01)
        {
            DBG_PRINT("PSCALB Active\r\n");
            //return FAIL;
        }
    }
    if((temp0>>10) &0x01)  //codec ADC, I2S_RX
    {
        temp1 = R_MIC_SETUP;
        if(temp1 & 0x07)
        {
            DBG_PRINT("MIC or Line IN Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2S0RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S0 RX Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2S1RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S1 RX Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2S2RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S2 RX Active\r\n");
            //return FAIL;
        }
        temp1 = R_I2S3RX_CTRL;
        if((temp1) &0x01)
        {
            DBG_PRINT("I2S3 RX Active\r\n");
            //return FAIL;
        }
    }
    if((temp0>>11) & 0x1)  //h264 decode, h264 scalar, MB2SCAN
    {
            DBG_PRINT("H264 decode Active\r\n");
            //return FAIL;

    }
    if((temp0>>12) &0x01)
    {
            DBG_PRINT("SPU Active\r\n");
            //return FAIL;
    }
    if((temp0>>13) &0x01)
    {
            DBG_PRINT("TFT Active\r\n");
            //return FAIL;
    }
    if((temp0>>14) &0x01)
    {
            DBG_PRINT("ISP Active\r\n");
            //return FAIL;
    }
    if((temp0>>15) &0x01)
    {
        temp1 = R_SDC0_CTRL;
        if((temp1>>11) & 0x01)
        {
            DBG_PRINT("SDC0 Active\r\n");
            //return FAIL;
        }
        temp1 = R_SDC1_CTRL;
        if((temp1>>11) & 0x01)
        {
            DBG_PRINT("SDC1 Active\r\n");
            //return FAIL;
        }
    }
    temp0 = R_SYSTEM_CLK_EN1;
    if((temp0) & 0x01) // FD and rotation (bit16)
    {
        temp1 = R_ROTATOR_START;
        if(temp1 & 0x01)
        {
        DBG_PRINT("Rotation Active\r\n");
        //return FAIL;
        }
        temp1 = R_FD_CTRL0;
        if(temp1 & 0x01)
        {
        DBG_PRINT("FD Active\r\n");
        //return FAIL;
        }
    }
    if((temp0>>1)&0x01)  //USB host (bit17)
    {
            DBG_PRINT("USBH Active\r\n");
            //return FAIL;
    }
    if((temp0>>2)&0x01)  //USB device  (bit18)
    {
            DBG_PRINT("USBD Active\r\n");
            //return FAIL;
    }
    if((temp0>>3)&0x01)  //PPU  (bit19)
    {
            DBG_PRINT("PPU Active\r\n");
            //return FAIL;
    }
    if((temp0>>4)&0x01)  //DMA  (bit20)
    {
        temp1 = R_DMA0_CTRL;
        if((temp1) & 0x01)  //channel enable
        {
            DBG_PRINT("DMA0 Active\r\n");
            //return FAIL;
        }
        temp1 = R_DMA1_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA1 Active\r\n");
            //return FAIL;
        }
        temp1 = R_DMA2_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA2 Active\r\n");
            //return FAIL;
        }
        temp1 = R_DMA3_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA3 Active\r\n");
            //return FAIL;
        }
        temp1 = R_DMA4_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA4 Active\r\n");
            //return FAIL;
        }
        temp1 = R_DMA5_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA5 Active\r\n");
            //return FAIL;
        }
        temp1 = R_DMA6_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA6 Active\r\n");
            //return FAIL;
        }
        temp1 = R_DMA7_CTRL;
        if((temp1) & 0x01)
        {
            DBG_PRINT("DMA7 Active\r\n");
            //return FAIL;
        }

    }
    if((temp0>>5)&0x01)  //H264 encode  (bit21)
    {
            DBG_PRINT("H264 encode Active\r\n");
            //return FAIL;
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
                //return FAIL;
            }
        }
    }
    if((temp0>>7)&0x01)  //FFT and Object  (bit23)
    {
        temp1 = P_FFT_START;
        if(temp1 & 0x01)  //start bit
        {
            DBG_PRINT("FFT Active\r\n");
            //return FAIL;
        }
    }
    if((temp0>>8)&0x01)  //SPIFC and SPI I/F  (bit24)
    {
        temp1 = R_SPI0_CTRL;
        if((temp1>>15) & 0x01)
        {
            DBG_PRINT("SPI0 Active\r\n");
            //return FAIL;
        }
        temp1 = R_SPI1_CTRL;
        if((temp1>>15) & 0x01)
        {
            DBG_PRINT("SPI1 Active\r\n");
            //return FAIL;
        }
    }
    if((temp0>>9)&0x01)  //MIPI Aand B  (bit25)
    {
        temp1 = R_MIPIA_GLB_CSR;
        if(temp1 & 0x01)
        {
            DBG_PRINT("MIPIA Active\r\n");
            //return FAIL;
        }
        temp1 = R_MIPIB_GLB_CSR;
        if(temp1 & 0x01)
        {
            DBG_PRINT("MIPIB Active\r\n");
            //return FAIL;
        }
    }
    if((temp0>>10)&0x01)  //jpeg, conv420 to 422  (bit26)
    {
        temp1 = R_JPG_CTRL;
        if(temp1 & 0x3)  //check JPEG codec start bit
        {
            DBG_PRINT("JPEG codec Active\r\n");
            //return FAIL;
        }
        temp1 = (*((volatile INT32U*)0xC0180000));  //conv420 to 422
        if((temp1>>30) & 0x01)  //check busy
        {
            DBG_PRINT("conv420to422 Active\r\n");
            //return FAIL;
        }
    }

    /* don't care
    if((temp0>>11)&0x01)  //Display Output Reg  (bit27)
    {
            DBG_PRINT("H264 encode Active\r\n");
           // return FAIL;
    }
    */
    if((temp0>>12)&0x01)  //Scalar  (bit28)
    {
        temp1 = R_SCA_CON;
        if((temp1>>8) & 0x01)
        {
            DBG_PRINT("Scalar Active\r\n");
            //return FAIL;
        }
    }
    if((temp0>>13)&0x01)  //NAND and BCH  (bit29)
    {
            DBG_PRINT("NAND flash Active\r\n");
            //return FAIL;
    }
    /* don't care
    if((temp0>>14)&0x01)  //H264AVS   (bit30)
    {
            DBG_PRINT("H264 AVS Active\r\n");
            //return FAIL;
    }
    */


    return SUCCESS;
}

INT32S drv_l2_change_system_speed(int systempll)
{
    if(systempll<C_MIN_PLL)
    {
        DBG_PRINT("systempll below Min speed\r\n");
        systempll = C_MIN_PLL;
    }
    portENTER_CRITICAL();
    if(system_module_check())
    {
        DBG_PRINT("change system clock fail\r\n");
        portEXIT_CRITICAL();
        return FAIL;
    }
    Iram_copy();
    systempll_index = systempll/9 - 3;
    ddr_parameter_temp.ddr_timing1 = ddr_init_table[systempll_index].ddr_timing1;
    ddr_parameter_temp.ddr_timing2 = ddr_init_table[systempll_index].ddr_timing2;
    ddr_parameter_temp.ddr_tref = ddr_init_table[systempll_index].ddr_tref;
    ddr_parameter_temp.ddr_init_v = ddr_init_table[systempll_index].ddr_init_v;
    temp0 = R_SYSTEM_CLK_EN0;
    temp1 = R_SYSTEM_CLK_EN1;
    R_SYSTEM_CLK_EN0 = 0x1 + (0x1<<1)+(0x1<<5); /*system bus + memory + cpu I/F */
    R_SYSTEM_CLK_EN1 = 0x1<<15;   /* system control */
    // get calibration data
    temp2 = (INT32U*) R_AC_INIT_TADDR;
    temp3[0] = *temp2++;
    temp3[1] = *temp2++;
    temp3[2] = *temp2++;
    temp3[3] = *temp2;

//    drv_l2_ddr_to_self_refresh();
    R_DDR_CTL2 |= 0x1<<4;
    temp =R_DDR_CTL2;
    while (!(temp&0x1<<8))
        temp =R_DDR_CTL2;

    //test delay

    for(temp=0;temp<20000000;temp++)
    {
        temp2 = (INT32U*) R_AC_INIT_TADDR;
        temp2 += 2;
    }

    R_DDR_CTL2 &= ~(0x1<<4);
    temp =R_DDR_CTL2;
    while ((temp&0x1<<8))
        temp =R_DDR_CTL2;

//    drv_l2_ddr_calibration(systempll_index,&ddr_parameter_temp);
    R_DDR_EMR &= ~0x1;  //DLL ON
    if(systempll_index> 0x13)
    {
        R_DDR_CAS = 0x3; //cas = 3 system = 100MHz~200MHz DDR SPEC
    }else if(systempll_index> 0xF)
    {
        R_DDR_CAS = 0x2; //cas = 2 system = 83MHz~133MHz
    }else
    {
        R_DDR_CAS = 0x2; //cas = 2 system = 83MHz~133MHz
        R_DDR_EMR |= 0x1;  //DLL OFF
    }


    R_SYSTEM_PLLEN &= ~0x3F;
    R_SYSTEM_PLLEN |= systempll_index; //change speed to 108MHz

    R_DDR_CTL = 0x0; // DDR disable
    R_DDR_TIMING2 = 0x0000AAAA;
    R_AC_CTRL = 0x00000001; //auto calibration enable
    //set DDR 0xd0200120~012c
    R_DDR_TIMING1 = ddr_parameter_temp.ddr_timing1;
    R_DDR_TIMING2 = ddr_parameter_temp.ddr_timing2;
    R_DDR_TREF = ddr_parameter_temp.ddr_tref;
    R_DDR_INIT_V = ddr_parameter_temp.ddr_init_v;

    R_AC_STEP = 0x00000222;

    R_DDR_CTL = 0x3; //set DDR size & DDR enable
//ddr calibration end



    temp2 = (INT32U*) R_AC_INIT_TADDR;
    *temp2++ = temp3[0];
    *temp2++ = temp3[1];
    *temp2++ = temp3[2];
    *temp2 = temp3[3];
    R_SYSTEM_CLK_EN0 = temp0;
    R_SYSTEM_CLK_EN1 = temp1;
    SystemCoreClock = systempll*1000000/2;
    drv_l1_uart1_buad_rate_set(UART0_BAUD_RATE);

    portEXIT_CRITICAL();
    return SUCCESS;
}
