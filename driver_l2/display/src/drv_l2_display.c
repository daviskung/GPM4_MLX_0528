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

/*******************************************************
    Include file
*******************************************************/
#include <stdio.h>
#include <string.h>
#include "drv_l1_sfr.h"
#include "drv_l2_display.h"
#if _DRV_L1_TFT == 1
#include "drv_l1_tft.h"
#endif
#if _DRV_L1_HDMI == 1
#include "drv_l1_hdmi.h"
//#include "drv_l1_conv420to422.h"
#endif
#if _DRV_L1_TV == 1
#include "drv_l1_tv.h"
#endif


#define DISPOSQFlush(x)\
{\
    while(1) {\
        osEvent result;\
        result = osMessageGet(x, 1);\
        if(result.status != osEventMessage) {\
            break;\
        }\
    }\
}

#if (defined _DRV_L2_DISP) && (_DRV_L2_DISP == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#ifndef DISABLE
#define DISABLE                         0
#endif

#ifndef ENABLE
#define ENABLE                          1
#endif

#define DBG_DISP_ENABLE		            1
#if DBG_DISP_ENABLE == 1
#define DRV_DISP_DBG		            print_string
#else
#define DRV_DISP_DBG(...)
#endif

#define DISP_QUEUE_MAX		            1
#define DISP_UPDATE_DONE	            1
#define DISP_UNDER_RUN_EN               0
#define DISP_OS_EN                      1

#define PPU_PPU_V_BLANKING				(1 << 0)
#define PPU_DMA_COMPLETE			    (1 << 2)
#define PPU_TV_UNDER_RUN				(1 << 10)
#define PPU_TV_V_BLANKING				(1 << 11)
#define PPU_TFT_UNDER_RUN				(1 << 12)
#define PPU_TFT_V_BLANKING				(1 << 13)
#define PPU_HDMI_V_BLANKING				(1 << 19)
#define PPU_HDMI_UNDER_RUN              (1 << 20)

#define	PPU_YUYV_TYPE3					(3 << 20)
#define	PPU_YUYV_TYPE2					(2 << 20)
#define	PPU_YUYV_TYPE1					(1 << 20)
#define	PPU_YUYV_TYPE0					(0 << 20)

#define	PPU_RGBG_TYPE3					(3 << 20)
#define	PPU_RGBG_TYPE2					(2 << 20)
#define	PPU_RGBG_TYPE1					(1 << 20)
#define	PPU_RGBG_TYPE0					(0 << 20)

#define TFT_LB							(1 << 24)
#define PPU_LB							(1 << 19)
#define	PPU_YUYV_MODE					(1 << 10)
#define	PPU_RGBG_MODE			        (0 << 10)

#define TFT_SIZE_1024X768               (7 << 16)
#define TFT_SIZE_800X600                (6 << 16)
#define TFT_SIZE_800X480                (5 << 16)
#define TFT_SIZE_720X480				(4 << 16)
#define TFT_SIZE_480X272				(3 << 16)
#define TFT_SIZE_480X234				(2 << 16)
#define TFT_SIZE_640X480                (1 << 16)
#define TFT_SIZE_320X240                (0 << 16)
#define TFT_SIZE_MASK					(7 << 16)

#define	PPU_YUYV_RGBG_FORMAT_MODE		(1 << 8)
#define	PPU_RGB565_MODE			        (0 << 8)

#define	PPU_FRAME_BASE_MODE			    (1 << 7)
#define	PPU_VGA_NONINTL_MODE			(0 << 5)

#define	PPU_VGA_MODE					(1 << 4)
#define	PPU_QVGA_MODE					(0 << 4)

#define PPU_UI_ENABLE                   (1 << 0)
#define PPU_UI_HDMI_MODE                (1 << 1)
#define PPU_UI_COLOR_1555               (0 << 2)
#define PPU_UI_COLOR_PAL_4              (1 << 2)
#define PPU_UI_COLOR_PAL_16             (2 << 2)
#define PPU_UI_COLOR_PAL_256            (3 << 2)
#define PPU_UI_COLOR_MASK               (3 << 2)
#define PPU_UI_BLEND_BIT                4
#define PPU_UI_BLEND_MASK               (0xF << PPU_UI_BLEND_BIT)
#define PPU_UI_COLOR_ARGB4444           (1 << 8)

#define BYPASS_CONV420_EN				1
/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct display_s
{
	INT8U tv_update_flag;
	INT8U tft_update_flag;
	INT8U hdmi_update_flag;
	INT8U hdmi_en_flag;

    INT32U tv_under_run_flag;
    INT32U tft_under_run_flag;
    INT32U hdmi_under_run_flag;
	INT32U tv_color_fmt;
	INT32U tft_color_fmt;
	INT32U hdmi_color_fmt;
	INT32U tv_disp_buf;
	INT32U tft_disp_buf;
	INT32U hdmi_disp_buf;

    INT32U ui_color_fmt;
    INT32U ui_update_flag;
	INT32U ui_blend_level;
	INT32U ui_disp_buf;
} display_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
#if DISP_OS_EN == 1
static osMessageQId tv_q = NULL;
static osMessageQId tft_q = NULL;
static osMessageQId hdmi_q = NULL;
static osSemaphoreId tv_sem = NULL;
static osSemaphoreId tft_sem = NULL;
static osSemaphoreId hdmi_sem = NULL;
#endif

static display_t display, *pDisp;
extern DispCtrl_t TFT_Param;
static INT32U hdmi_ui_test_cnt = 0;
static INT16U disp_h,disp_v;
static void (*ppu_isr_callback)(INT32U ppu_event);
static INT32U disp_dev_init=0;

void PPU_IRQHandler(void)
{
    INT32U msg;
	INT32U enable = R_PPU_IRQ_EN;
	INT32U status = R_PPU_IRQ_STATUS;

	R_PPU_IRQ_STATUS = status;
	enable &= status;

#if _DRV_L1_PPU == 1
    if((enable & PPU_PPU_V_BLANKING) && (status & PPU_PPU_V_BLANKING)
    || (enable & PPU_DMA_COMPLETE) && (status & PPU_DMA_COMPLETE)) {
        if(ppu_isr_callback)
            ppu_isr_callback(status);
    }
#endif

#if _DRV_L1_TV == 1
	if(enable & PPU_TV_V_BLANKING) {
		if(pDisp->tv_update_flag) {
			R_TV_FBI_ADDR = pDisp->tv_disp_buf;
			pDisp->tv_update_flag = 0;
		#if DISP_OS_EN == 1
            msg = DISP_UPDATE_DONE;
            osMessagePut(tv_q, (void *)&msg, osWaitForever);
		#endif
		}
	}
    #if DISP_UNDER_RUN_EN == 1
        if(enable & PPU_TV_UNDER_RUN) {
            pDisp->tv_under_run_flag++;
            DRV_DISP_DBG("TV under run\r\n");
        }
    #endif
#endif

#if _DRV_L1_TFT == 1
	if(enable & PPU_TFT_V_BLANKING) {
		if(pDisp->tft_update_flag) {
			R_TFT_FBI_ADDR = pDisp->tft_disp_buf;
			pDisp->tft_update_flag = 0;
		#if DISP_OS_EN == 1
			msg = DISP_UPDATE_DONE;
            osMessagePut(tft_q, (uint32_t)&msg, osWaitForever);
		#endif
		}
	}
    #if DISP_UNDER_RUN_EN == 1
        if(enable & PPU_TFT_UNDER_RUN) {
            pDisp->tft_under_run_flag++;
            DRV_DISP_DBG("TFT under run\r\n");
        }
    #endif
#endif

#if _DRV_L1_HDMI == 1
	/*
	if(enable & PPU_HDMI_V_BLANKING) {
		if (R_CONV420_IN_A_ADDR !=0 )   // GP420 testing case
		{
			// 每換一個 frame 就要重新啟動 GP_YUV420  to  YUV422
			R_CONV420_CTRL |= 0x201; // reset & enable
			R_CONV420_CTRL |= 0x100;	// start
		}
		hdmi_ui_test_cnt++;
		if ( ((hdmi_ui_test_cnt&0x3FF)==0)&&(R_PPU_UI_ADDR!=0) )
		{
			R_PPU_UI_CTRL ^= 1;		//Turn on/off UI (for FPGA testing)
		}
	}
*/
	#if 1	// driver is not ready yet. // modify by josephhsieh@20150504
	if(enable & PPU_HDMI_V_BLANKING) {
		if(pDisp->hdmi_update_flag == 1) {
			//if(pDisp->hdmi_color_fmt == DISP_FMT_GP420) {
				//drv_l1_conv420_input_A_addr_set(pDisp->hdmi_disp_buf);
			//}

			R_TV_FBI_ADDR = pDisp->hdmi_disp_buf;
			pDisp->hdmi_update_flag = 0;
		#if DISP_OS_EN == 1
			msg = DISP_UPDATE_DONE;
            osMessagePut(hdmi_q, (uint32_t)&msg, osWaitForever);
		#endif
		}
            /*
		drv_l1_conv420_reset();

		if(pDisp->hdmi_en_flag == 1) {
			// hdmi start
			if(pDisp->hdmi_color_fmt == DISP_FMT_GP420) {
				// use conv420to422 module
				drv_l1_conv420_start();
			}
		} else if(pDisp->hdmi_en_flag == 0xFF) {
			// hdmi stop
			pDisp->hdmi_en_flag = 0;
			drv_l1_conv420_uninit();
		}
		*/
	}
	#endif
    #if DISP_UNDER_RUN_EN == 1
        if(enable & PPU_HDMI_UNDER_RUN) {
            pDisp->hdmi_under_run_flag++;
            DRV_DISP_DBG("HDMI under run\r\n");
        }
    #endif
#endif
}

static void ppu_set_size(INT32U disp_dev, INT16U width, INT16U height)
{
	INT32U reg = R_PPU_ENABLE;

	reg &= ~TFT_SIZE_MASK;
	if(disp_dev == DISDEV_TV_QVGA) {
		reg &= ~PPU_VGA_MODE;
	} else if(disp_dev == DISDEV_TV_VGA) {
		reg |= PPU_VGA_MODE;
	} else if(disp_dev == DISDEV_TV_D1) {
		reg &= ~PPU_VGA_MODE;
		reg |= TFT_SIZE_720X480;
	} else {
		reg &= ~PPU_VGA_MODE;
		reg |= PPU_LB;
		if(width == 320 && height == 240) {
			reg |= TFT_SIZE_320X240;
		} else if(width == 480 && height == 234) {
			reg |= TFT_SIZE_480X234;
		} else if(width == 480 && height == 272) {
			reg |= TFT_SIZE_480X272;
		} else if(width == 640 && height == 484) {
			reg |= TFT_SIZE_640X480;
		} else if(width == 720 && height == 484) {
			reg |= TFT_SIZE_720X480;
		} else if(width == 800 && height == 480) {
			reg |= TFT_SIZE_800X480;
		} else if(width == 800 && height == 600) {
			reg |= TFT_SIZE_800X600;
		} else if(width == 1024 && height == 768) {
			reg |= TFT_SIZE_1024X768;
		} else {
			R_FREE_SIZE	= ((width & 0x7FF) << 16) | (height & 0x7FF);
		}
	}

	R_PPU_ENABLE = reg;
}

static void ppu_set_color_mode(INT32U color_mode)
{
	INT32U reg = R_PPU_ENABLE;

	reg &= ~(3 << 20);
	switch(color_mode)
	{
	case DISP_FMT_RGB565:
		reg &= ~PPU_YUYV_RGBG_FORMAT_MODE;  //RGB565 mode
		reg &= ~PPU_YUYV_MODE;				//RGB565
		break;

	case DISP_FMT_BGRG:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg &= ~PPU_YUYV_MODE;				//RGBG
		reg |= PPU_RGBG_TYPE0;
		break;

	case DISP_FMT_GBGR:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg &= ~PPU_YUYV_MODE;				//RGBG
		reg |= PPU_RGBG_TYPE1;
		break;

	case DISP_FMT_RGBG:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg &= ~PPU_YUYV_MODE;				//RGBG
		reg |= PPU_RGBG_TYPE2;
		break;

	case DISP_FMT_GRGB:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg &= ~PPU_YUYV_MODE;				//RGBG
		reg |= PPU_RGBG_TYPE3;
		break;

	case DISP_FMT_VYUV:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg |= PPU_YUYV_MODE;				//YUYV
		reg |= PPU_YUYV_TYPE0;
		break;

	case DISP_FMT_YVYU:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg |= PPU_YUYV_MODE;				//YUYV
		reg |= PPU_YUYV_TYPE1;
		break;

	case DISP_FMT_UYVY:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg |= PPU_YUYV_MODE;				//YUYV
		reg |= PPU_YUYV_TYPE2;
		break;

	case DISP_FMT_YUYV:
		reg |= PPU_YUYV_RGBG_FORMAT_MODE;
		reg |= PPU_YUYV_MODE;				//YUYV
		reg |= PPU_YUYV_TYPE3;
		break;
	}

	R_PPU_ENABLE = reg;
}

static void ppu_set_enable(INT32U enable)
{
	if(enable) {
		R_PPU_ENABLE |= PPU_FRAME_BASE_MODE;
	} else {
		R_PPU_ENABLE = 0;
		R_PPU_IRQ_EN = 0;
		R_PPU_IRQ_STATUS = 0xFFFF;
	}
}

static void ppu_tft_long_burst_enable(INT32U enable)
{
	if(enable) {
		R_PPU_ENABLE |= TFT_LB;
	} else {
		R_PPU_ENABLE &= ~TFT_LB;
	}
}

static INT32U ppu_enable_get(void)
{
    return (R_PPU_ENABLE & 0x1);
}

static void ppu_ui_set_color_mode(UI_FMT color_mode)
{
	INT32U reg = R_PPU_UI_CTRL;

	switch(color_mode)
	{
	case UI_FMT_RGB1555:
        reg &= ~PPU_UI_COLOR_MASK;
        reg &= ~PPU_UI_COLOR_ARGB4444;
		break;

	case UI_FMT_PALETTE_4:
        reg &= ~PPU_UI_COLOR_MASK;
        reg &= ~PPU_UI_COLOR_ARGB4444;
		reg |= PPU_UI_COLOR_PAL_4;
		break;

	case UI_FMT_PALETTE_16:
        reg &= ~PPU_UI_COLOR_MASK;
        reg &= ~PPU_UI_COLOR_ARGB4444;
		reg |= PPU_UI_COLOR_PAL_16;
		break;

	case UI_FMT_PALETTE_256:
        reg &= ~PPU_UI_COLOR_MASK;
        reg &= ~PPU_UI_COLOR_ARGB4444;
		reg |= PPU_UI_COLOR_PAL_256;
		break;

	case UI_FMT_ARGB444:
        reg &= ~PPU_UI_COLOR_MASK;
        reg |= PPU_UI_COLOR_ARGB4444;
		break;
	}

	R_PPU_UI_CTRL = reg;

}

static void ppu_ui_set_blend(INT8U blend_level)
{
    INT32U reg = R_PPU_UI_CTRL;

    reg &= ~PPU_UI_BLEND_MASK;
    reg |= ((blend_level << PPU_UI_BLEND_BIT) & PPU_UI_BLEND_MASK);

    R_PPU_UI_CTRL = reg;
}

static void ppu_ui_set_mode(DISP_DEV disp_dev)
{
	if(disp_dev == DISDEV_TFT) {
		R_PPU_UI_CTRL &= ~PPU_UI_HDMI_MODE;
	} else {
        R_PPU_UI_CTRL |= PPU_UI_HDMI_MODE;
	}
}

static void ppu_ui_set_addr(INT32U addr)
{
	R_PPU_UI_ADDR = addr;
}

static void ppu_ui_set_enable(INT32U enable)
{
	if(enable) {
		R_PPU_UI_CTRL |= PPU_UI_ENABLE;
	} else {
		R_PPU_UI_CTRL &= ~PPU_UI_ENABLE;
	}
}

static void ppu_ui_pal_update(INT32U pal_addr)
{
    INT32U temp, org_value = R_PPU_PALETTE_CTRL;

    R_PPU_ENABLE |= 0x1;
    temp = R_PPU_PALETTE_CTRL;
    temp &= ~0xC;
    R_PPU_PALETTE_CTRL = temp;
    // 0xD0501000
    R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_PALETTE_RAM0 & 0x7FFF;
    R_PPU_SPRITE_DMA_SOURCE = (INT32U) pal_addr;
    R_PPU_SPRITE_DMA_NUMBER = 0x100 - 1;
    while(R_PPU_SPRITE_DMA_NUMBER!=0);
    R_PPU_PALETTE_CTRL = org_value;
    R_PPU_ENABLE &= ~0x1;
}

/**
 * @brief   init display
 * @param   none
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_init(void)
{
	INT32S ret;
#if DISP_OS_EN == 1
	osSemaphoreDef_t disp_sem = { 0 };
	osMessageQDef_t disp_q = { DISP_QUEUE_MAX, sizeof(INT32U), 0 };
#endif

	if(pDisp != 0) {
		ret = STATUS_OK;
		goto __exit;
	}

#if _DRV_L1_TV == 1
	tv_init();
#endif

#if _DRV_L1_TFT == 1
	drv_l1_tft_init();
#endif

	ppu_set_enable(DISABLE);

	pDisp = &display;
	memset((INT8S *)pDisp, 0x00, sizeof(display_t));

#if DISP_OS_EN == 1
#if _DRV_L1_TV == 1
	if(tv_q == 0)
	{
        tv_q = osMessageCreate(&disp_q, NULL);
        if(tv_q == 0) {
            ret = STATUS_FAIL;
            goto __exit;
        }
	}

    if(ptv_sem == 0)
    {
        ptv_sem = osSemaphoreCreate(&disp_sem, 1);
        if(ptv_sem == 0) {
            ret = STATUS_FAIL;
            goto __exit;
        }
    }
#endif

#if _DRV_L1_TFT == 1
	if(tft_q == 0)
	{
        tft_q = osMessageCreate(&disp_q, NULL);
        if(tft_q == 0) {
            ret = STATUS_FAIL;
            goto __exit;
        }
	}

    if(tft_sem == 0)
    {
        tft_sem = osSemaphoreCreate(&disp_sem, 1);
        if(tft_sem == 0) {
            ret = STATUS_FAIL;
            goto __exit;
        }
    }
#endif

#if _DRV_L1_HDMI == 1
	if(hdmi_q == 0)
	{
        hdmi_q = osMessageCreate(&disp_q, NULL);
        if(hdmi_q == 0) {
            ret = STATUS_FAIL;
            goto __exit;
        }
	}

    if(hdmi_sem == 0)
    {
        hdmi_sem = osSemaphoreCreate(&disp_sem, 1);
        if(hdmi_sem == 0) {
            ret = STATUS_FAIL;
            goto __exit;
        }
    }
#endif
#endif

	NVIC_SetPriority(PPU_IRQn, 5);
    NVIC_EnableIRQ(PPU_IRQn);

	ret = STATUS_OK;
__exit:
	if(ret < 0) {
		drv_l2_display_uninit();
	}
	return ret;
}

/**
 * @brief   uninit display
 * @param   none
 * @return 	none
 */
void drv_l2_display_uninit(void)
{
	if(pDisp == 0) {
		return;
	}

#if _DRV_L1_TV == 1
    if(disp_dev_init == DIS_TYPE_TV)
        tv_disable();
#endif

#if _DRV_L1_TFT == 1
    if(disp_dev_init == DIS_TYPE_TFT)
        drv_l1_tft_uninit();
#endif

#if _DRV_L1_HDMI == 1
    if(disp_dev_init == DIS_TYPE_HDMI)
        drvl1_hdmi_exit();
#endif
    disp_dev_init = 0;
	ppu_set_enable(DISABLE);

#if DISP_OS_EN == 1
	if(tv_q) {
		DISPOSQFlush(tv_q);
	}

	if(tft_q) {
		DISPOSQFlush(tft_q);
	}

	if(hdmi_q) {
		DISPOSQFlush(hdmi_q);
	}
#endif

    NVIC_DisableIRQ(PPU_IRQn);
	pDisp = 0;
}

/**
 * @brief   display start
 * @param   disp_dev[in]: display device, see DISP_DEV.
 * @param   color_mode[in]: display color mode, DISP_FMT
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_start(DISP_DEV disp_dev, DISP_FMT color_mode)
{
	switch(disp_dev)
	{
	#if _DRV_L1_TV == 1
	case DISDEV_TV_QVGA:
		pDisp->tv_color_fmt = color_mode;

		tv_start(TVSTD_NTSC_J, TV_QVGA, TV_NON_INTERLACE);

		ppu_set_size(DISDEV_TV_QVGA, 320, 240);
		ppu_set_color_mode(color_mode);
		ppu_set_enable(ENABLE);
		R_PPU_IRQ_STATUS = PPU_TV_V_BLANKING;
		R_PPU_IRQ_EN |= PPU_TV_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
		R_PPU_IRQ_STATUS = PPU_TV_UNDER_RUN;
		R_PPU_IRQ_EN |= PPU_TV_UNDER_RUN;
        #endif
        disp_dev_init = DIS_TYPE_TV;
		break;

	case DISDEV_TV_VGA:
		pDisp->tv_color_fmt = color_mode;

		tv_start(TVSTD_NTSC_J, TV_HVGA, TV_INTERLACE);

		ppu_set_size(DISDEV_TV_VGA, 640, 480);
		ppu_set_color_mode(color_mode);
		ppu_set_enable(ENABLE);
		R_PPU_IRQ_STATUS = PPU_TV_V_BLANKING;
		R_PPU_IRQ_EN |= PPU_TV_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
		R_PPU_IRQ_STATUS = PPU_TV_UNDER_RUN;
		R_PPU_IRQ_EN |= PPU_TV_UNDER_RUN;
        #endif
        disp_dev_init = DIS_TYPE_TV;
		break;

	case DISDEV_TV_D1:
		pDisp->tv_color_fmt = color_mode;

		tv_start(TVSTD_NTSC_J, TV_D1, TV_INTERLACE);

		ppu_set_size(DISDEV_TV_D1, 740, 480);
		ppu_set_color_mode(color_mode);
		ppu_set_enable(ENABLE);
		R_PPU_IRQ_STATUS = PPU_TV_V_BLANKING;
		R_PPU_IRQ_EN |= PPU_TV_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
		R_PPU_IRQ_STATUS = PPU_TV_UNDER_RUN;
		R_PPU_IRQ_EN |= PPU_TV_UNDER_RUN;
        #endif
        disp_dev_init = DIS_TYPE_TV;
		break;
	#endif

	#if _DRV_L1_TFT == 1
	case DISDEV_TFT:
		pDisp->tft_color_fmt = color_mode;

		TFT_Param.init();

		ppu_set_size(DISDEV_TFT, TFT_Param.width, TFT_Param.height);
		ppu_set_color_mode(color_mode);
		ppu_set_enable(ENABLE);
		ppu_tft_long_burst_enable(ENABLE);
		R_PPU_IRQ_STATUS = PPU_TFT_V_BLANKING;
		R_PPU_IRQ_EN |= PPU_TFT_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
		R_PPU_IRQ_STATUS = PPU_TFT_UNDER_RUN;
		R_PPU_IRQ_EN |= PPU_TFT_UNDER_RUN;
        #endif
        disp_dev_init = DIS_TYPE_TFT;
		break;
	#endif

	#if _DRV_L1_HDMI == 1// driver is not ready yet. // modify by josephhsieh@20150504
	case DISDEV_HDMI_480P:
		pDisp->hdmi_en_flag = 1;
		pDisp->hdmi_color_fmt = color_mode;
		if(color_mode == DISP_FMT_GP420) {
			// use ConvGP420toYUV422 module
			/*
			drv_l1_conv420_init();
			drv_l1_conv420_reset();
			drv_l1_conv420_path(CONV420_TO_TV_HDMI);
			drv_l1_conv420_output_fmt_set(CONV420_FMT_YUYV);
			drv_l1_conv420_input_pixels_set(480);
			drv_l1_conv420_convert_enable(ENABLE);
			drv_l1_conv420_start();
			*/
		}
		else {
			// bypass ConvGP420toYUV422 module
			//drv_l1_conv420_uninit();
		}

        drvl1_hdmi_init(HDMI_VID_480P,HDMI_AUD_44K);
		drvl1_hdmi_audio_ctrl(ENABLE);
		drvl1_hdmi_dac_mute(ENABLE);

		R_SYSTEM_MISC_CTRL4 |= 0x80;
		if(color_mode == DISP_FMT_GP420) {
			ppu_set_color_mode(DISP_FMT_YUYV);
		} else {
			ppu_set_color_mode(color_mode);
		}

		ppu_set_size(DISDEV_HDMI_480P, 720, 480);
		ppu_set_enable(ENABLE);
		R_PPU_IRQ_STATUS = PPU_HDMI_V_BLANKING;
        R_PPU_IRQ_EN |= PPU_HDMI_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
		R_PPU_IRQ_STATUS = PPU_HDMI_UNDER_RUN;
        R_PPU_IRQ_EN |= PPU_HDMI_UNDER_RUN;
        #endif
        disp_dev_init = DIS_TYPE_HDMI;
		break;

	case DISDEV_HDMI_720P:
		pDisp->hdmi_en_flag = 1;
		pDisp->hdmi_color_fmt = color_mode;
		if(color_mode == DISP_FMT_GP420) {
			// use ConvGP420toYUV422 module
			/*
			drv_l1_conv420_init();
			drv_l1_conv420_reset();
			drv_l1_conv420_path(CONV420_TO_TV_HDMI);
			drv_l1_conv420_output_fmt_set(CONV420_FMT_YUYV);
			drv_l1_conv420_input_pixels_set(1280);
			drv_l1_conv420_convert_enable(ENABLE);
			drv_l1_conv420_start();
			*/
		}
		else {
			// bypass Conv GP420 to YUV422
			//drv_l1_conv420_uninit();
		}
		//R_SYSTEM_MISC_CTRL4 = 0xBF;		// bit[7:6]: 2'b00=>GP422To420,  2'b01=> H264,  2'b10=> Frame mode
		R_SYSTEM_MISC_CTRL4 |= 0x80;
		if(color_mode == DISP_FMT_GP420) {
			ppu_set_color_mode(DISP_FMT_YUYV);
		} else {
			ppu_set_color_mode(color_mode);
		}
		ppu_set_size(DISDEV_HDMI_720P, 1280, 720);
		ppu_set_enable(ENABLE);
		drvl1_hdmi_init(HDMI_VID_720P,HDMI_AUD_44K);
		drvl1_hdmi_audio_ctrl(ENABLE);
		drvl1_hdmi_dac_mute(ENABLE);
		R_PPU_IRQ_STATUS = PPU_HDMI_V_BLANKING;
        R_PPU_IRQ_EN |= PPU_HDMI_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
		R_PPU_IRQ_STATUS = PPU_HDMI_UNDER_RUN;
        R_PPU_IRQ_EN |= PPU_HDMI_UNDER_RUN;
        #endif
        disp_dev_init = DIS_TYPE_HDMI;
		break;
	#endif
	default:
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief   display stop
 * @param   disp_dev[in]: display device, see DISP_DEV.
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_stop(DISP_DEV disp_dev)
{
	switch(disp_dev)
	{
	#if _DRV_L1_TV == 1
	case DISDEV_TV_QVGA:
	case DISDEV_TV_VGA:
	case DISDEV_TV_D1:
		tv_disable();
		R_PPU_IRQ_EN &= ~PPU_TV_V_BLANKING;
		R_PPU_IRQ_STATUS = PPU_TV_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
        R_PPU_IRQ_EN &= ~PPU_TV_UNDER_RUN;
		R_PPU_IRQ_STATUS = PPU_TV_UNDER_RUN;
        #endif
		break;
	#endif

	#if _DRV_L1_TFT == 1
	case DISDEV_TFT:
		drv_l1_tft_en_set(DISABLE);
		R_PPU_IRQ_EN &= ~PPU_TFT_V_BLANKING;
		R_PPU_IRQ_STATUS = PPU_TFT_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
        R_PPU_IRQ_EN &= ~PPU_TFT_UNDER_RUN;
		R_PPU_IRQ_STATUS = PPU_TFT_UNDER_RUN;
        #endif
		break;
	#endif

	#if _DRV_L1_HDMI == 1
	case DISDEV_HDMI_480P:
	case DISDEV_HDMI_720P:
        drvl1_hdmi_exit();
        R_PPU_IRQ_EN &= ~PPU_HDMI_V_BLANKING;
  		R_PPU_IRQ_STATUS = PPU_HDMI_V_BLANKING;
        #if DISP_UNDER_RUN_EN == 1
        R_PPU_IRQ_EN &= ~PPU_HDMI_UNDER_RUN;
		R_PPU_IRQ_STATUS = PPU_HDMI_UNDER_RUN;
        #endif
		break;
	#endif
	default:
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief   display get size
 * @param   disp_dev[in]: display device
 * @param   width[out]: width
 * @param   height[out]: height
 * @return 	none
 */
void drv_l2_display_get_size(DISP_DEV disp_dev, INT16U *width, INT16U *height)
{
	switch(disp_dev)
	{
	case DISDEV_TV_QVGA:
		*width = 320;
		*height = 240;
		break;

	case DISDEV_TV_VGA:
		*width = 640;
		*height = 480;
		break;

	case DISDEV_TV_D1:
		*width = 720;
		*height = 480;
		break;

#if (DISPLAY_DEVICE == DISDEV_TFT)
	case DISDEV_TFT:
		*width = TFT_Param.width;
		*height = TFT_Param.height;
		break;
#endif

	case DISDEV_HDMI_480P:
		*width = 720;
		*height = 480;
		break;

	case DISDEV_HDMI_720P:
		*width = 1280;
		*height = 720;
		break;

	default:
		*width = 0;
		*height = 0;
	}
}

/**
 * @brief   display get format
 * @param   disp_dev[in]: display device
 * @return 	display format
 */
INT32U drv_l2_display_get_fmt(DISP_DEV disp_dev)
{
	switch(disp_dev)
	{
	case DISDEV_TV_QVGA:
	case DISDEV_TV_VGA:
	case DISDEV_TV_D1:
		return pDisp->tv_color_fmt;

	case DISDEV_TFT:
		return pDisp->tft_color_fmt;

	case DISDEV_HDMI_480P:
	case DISDEV_HDMI_720P:
		return pDisp->hdmi_color_fmt;

	default:
		return DISP_FMT_MAX;
	}
}

/**
 * @brief   display update frame buffer
 * @param   buffer[in]: frame buffer address
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_update(DISP_DEV disp_dev, INT32U buffer)
{
#if DISP_OS_EN == 1
    osEvent result;
#endif

	if(buffer == 0) {
		return STATUS_FAIL;
	}

	switch(disp_dev)
	{
	#if _DRV_L1_TV == 1
	case DISDEV_TV_QVGA:
	case DISDEV_TV_VGA:
	case DISDEV_TV_D1:
		pDisp->tv_disp_buf = buffer;
		pDisp->tv_update_flag = 1;
	#if DISP_OS_EN == 1
		osSemaphoreWait(tv_sem, osWaitForever);
		result = osMessageGet(tv_q, osWaitForever);
		osSemaphoreRelease(tv_sem);

        if((result.status != osEventMessage) || (result.value.v != DISP_UPDATE_DONE)) {
            return STATUS_FAIL;
        }
	#else
		while(pDisp->tv_update_flag);
	#endif
		break;
	#endif

	#if _DRV_L1_TFT == 1
	case DISDEV_TFT:
		pDisp->tft_disp_buf = buffer;
		pDisp->tft_update_flag = 1;
	#if DISP_OS_EN == 1
		osSemaphoreWait(tft_sem, osWaitForever);
		result = osMessageGet(tft_q, osWaitForever);
		osSemaphoreRelease(tft_sem);

        if((result.status != osEventMessage) || (result.value.v != DISP_UPDATE_DONE)) {
            return STATUS_FAIL;
        }
	#else
		while(pDisp->tft_update_flag);
	#endif
		break;
	#endif

	#if _DRV_L1_HDMI == 1
	case DISDEV_HDMI_480P:
	case DISDEV_HDMI_720P:
		pDisp->hdmi_disp_buf = buffer;
		pDisp->hdmi_update_flag = 1;
	#if DISP_OS_EN == 1
		osSemaphoreWait(hdmi_sem, osWaitForever);
		result = osMessageGet(hdmi_q, osWaitForever);
		osSemaphoreRelease(hdmi_sem);

        if((result.status != osEventMessage) || (result.value.v != DISP_UPDATE_DONE)) {
            return STATUS_FAIL;
        }
	#else
		while(pDisp->hdmi_update_flag);
	#endif
		break;
	#endif
	default:
		return DISDEV_MAX;
	}

	return STATUS_OK;
}

/**
 * @brief   display update frame buffer without wait IRQ end
 * @param   buffer[in]: frame buffer address
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_buffer_set(DISP_DEV disp_dev, INT32U buffer)
{
	if(buffer == 0) {
		return STATUS_FAIL;
	}

	switch(disp_dev)
	{
	#if _DRV_L1_TV == 1
	case DISDEV_TV_QVGA:
	case DISDEV_TV_VGA:
	case DISDEV_TV_D1:
		pDisp->tv_disp_buf = buffer;
		pDisp->tv_update_flag = 1;
		break;
	#endif

	#if _DRV_L1_TFT == 1
	case DISDEV_TFT:
		pDisp->tft_disp_buf = buffer;
		pDisp->tft_update_flag = 1;
		break;
	#endif

	#if _DRV_L1_HDMI == 1
	case DISDEV_HDMI_480P:
	case DISDEV_HDMI_720P:
		pDisp->hdmi_disp_buf = buffer;
		pDisp->hdmi_update_flag = 1;
		break;
	#endif
	default:
		return DISDEV_MAX;
	}

	return STATUS_OK;
}

/**
 * @brief   display update frame buffer state
 * @param   buffer[in]: frame buffer address
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_buffer_update_state_get(DISP_DEV disp_dev, INT32U wait)
{
#if DISP_OS_EN == 1
    osEvent result;
#endif

	switch(disp_dev)
	{
	#if _DRV_L1_TV == 1
	case DISDEV_TV_QVGA:
	case DISDEV_TV_VGA:
	case DISDEV_TV_D1:
	#if DISP_OS_EN == 1
		osSemaphoreWait(tv_sem, osWaitForever);
		if(wait)
            result = osMessageGet(tv_q, osWaitForever);
        else
            result = osMessageGet(tv_q, 100);
		osSemaphoreRelease(tv_sem);

        if((result.status != osEventMessage) || (result.value.v != DISP_UPDATE_DONE)) {
            return STATUS_FAIL;
        }
	#else
		while(pDisp->tv_update_flag);
	#endif
		break;
	#endif

	#if _DRV_L1_TFT == 1
	case DISDEV_TFT:
	#if DISP_OS_EN == 1
		osSemaphoreWait(tft_sem, osWaitForever);
		if(wait)
            result = osMessageGet(tft_q, osWaitForever);
        else
            result = osMessageGet(tft_q, 100);
		osSemaphoreRelease(tft_sem);

        if((result.status != osEventMessage) || (result.value.v != DISP_UPDATE_DONE)) {
            return STATUS_FAIL;
        }
	#else
		while(pDisp->tft_update_flag);
	#endif
		break;
	#endif

	#if _DRV_L1_HDMI == 1
	case DISDEV_HDMI_480P:
	case DISDEV_HDMI_720P:
	#if DISP_OS_EN == 1
		osSemaphoreWait(hdmi_sem, osWaitForever);
		if(wait)
            result = osMessageGet(hdmi_q, osWaitForever);
        else
            result = osMessageGet(hdmi_q, 100);
		osSemaphoreRelease(hdmi_sem);

        if((result.status != osEventMessage) || (result.value.v != DISP_UPDATE_DONE)) {
            return STATUS_FAIL;
        }
	#else
		while(pDisp->hdmi_update_flag);
	#endif
		break;
	#endif
	default:
		return DISDEV_MAX;
	}

	return STATUS_OK;
}

/**
 * @brief   init display UI
 * @param   none
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_UI_init(DISP_DEV disp_dev, DISP_FMT color_mode, INT8U blend_level, INT32U ui_addr, INT32U pal_addr)
{
    if(ui_addr == 0)
        return STATUS_FAIL;

    ppu_ui_set_mode(disp_dev);
    ppu_ui_set_color_mode(color_mode);

    if(ppu_enable_get() == 0)
    {
        if(ui_addr && (color_mode != UI_FMT_RGB1555))
            ppu_ui_pal_update(pal_addr);
    }

    drv_l2_display_get_size(disp_dev,(INT16U *)&disp_h,(INT16U *)&disp_v);
    if(disp_h == 0 || disp_v == 0)
        return STATUS_FAIL;
    else
        ppu_set_size(disp_dev, disp_h, disp_v);
    ppu_ui_set_blend(blend_level);
    ppu_ui_set_addr(ui_addr);
    ppu_ui_set_enable(1);

    return 0;
}

/**
 * @brief   uninit display UI
 * @param   none
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_UI_uninit(DISP_DEV disp_dev)
{
    ppu_ui_set_addr(0);
    ppu_ui_set_enable(0);

    return 0;
}

/**
 * @brief   display update ui frame buffer and blend level
 * @param   blend_level[in]: blend level
 * @param   ui_addr[in]: frame buffer address
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_UI_update(DISP_DEV disp_dev, INT8U blend_level, INT32U ui_addr, INT32U pal_addr)
{
    if(ppu_enable_get() == 0)
    {
        if(ui_addr)
            ppu_ui_pal_update(pal_addr);
    }
    if(ui_addr)
        ppu_ui_set_addr(ui_addr);
    ppu_ui_set_blend(blend_level);

    return 0;
}

/**
 * @brief   display UI stop
 * @param   none.
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l2_display_UI_stop(void)
{
    ppu_ui_set_enable(0);

    return 0;
}

/**
 * @brief   Register ppu call back function
 * @param   callback[in]: callback function
 * @return 	0: Success, -1: Fail
 */
INT32S drv_l2_ppu_isr_callback_set(void (*callback)(INT32U ppu_event))
{
	ppu_isr_callback = callback;

	return 0;
}

#endif //(defined _DRV_L2_DISP) && (_DRV_L2_DISP == 1)
