#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_l1.h"
#include "drv_l2.h"
#include "cmsis_os.h"
#include "gplib.h"
#include "gp_cmd.h"
#include "wifi_demo.h"
#include "string_sample.h"
#include "usbh_uvc.h"
#include "drv_l1_gpio.h"

#define AUDIO_DECODE            0
#define AUDIO_DECODE_OGGMIX     0
#define AUDIO_I2S_DECODE        0
#define AUDIO_ENCODE            0
#define IMAGE_CODEC             0
#define VIDEO_DECODE            0
#define VIDEO_ENCODE            0
#define DIGITAL_VIDEO_DEMO      0
#define GPM4_PPU_DEMO           1
#define FACE_DETECTION_DEMO     0
#define USBD_UAC_DEMO           0
#define USBD_UVC_DEMO           0	//if #define UVC_H264  1 @ usbd_uvc_dec.c, please #define AVI_AUDIO_ENCODE_EN   0 @ avi_encoder_scaler_jpeg.h
#define USBD_MSDC_DEMO          0
#define IMAGE_DECODE_DEMO       0
#define IMAGE_ENCODE_DEMO       0
#define OBJECT_RECOGNIZE_DEMO   0
#define WIFI_DEMO               0
#define VR_DEMO                 0
#define CSI_DISP_DEMO           0
#define CP_DEMO					0
#define CT_FDWA_VR_DEMO         0
#define CMSIS_DSP_DEMO          0
#define EHCI_DEMO               0
#define CPLUS_STRING_DEMO       0
#define COLOR_BALL_DEMO         0
#define CHANGE_SYSTEM_PLL_DEMO  0

#if defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
#include "console.h"
void CmdTask(void const *param);
xTaskHandle cmdTaskHandle = NULL;
#endif

#define STACKSIZE       32768

extern void GPM4_PPU_Demo(void);
extern void GPM4_FD_Demo(void);
extern void GPM4_OBJ_Demo(void);
extern void GPM4_CSI_DISP_Demo(void);
extern void GPM4_CT_FDWA_VR_Demo(void);
extern void GPM4_COLOR_BALL_Demo(void);
extern INT32S ehci_uvc_demo(void);
extern int32_t arm_sin_cos_example(void);
extern INT32S usbd_msdc_demo(void);
extern void GPM4_Change_System_Pll_Demo(void);
extern void ppu_isr_event(INT32U event);
extern INT16U _SPRITE_GP22_CellData_2[];
extern INT16U _SPRITE_GP22_CellData_3[];
extern INT16U _Text00100_IMG0000_CellData[];

/*-----------------------------------------------------------*/
#if 0
void vPortSetupTimerInterrupt()
{
/* Configure SysTick to interrupt at the requested rate. */
    SysTick_Config (32768 / configTICK_RATE_HZ);
}
#endif

/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{

}

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;
	DBG_PRINT("task name %s\r\n", pcTaskName);
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/
void InitTask(void const *param)
{
    //GPM4_DISP_Demo();
    //GPM4_CSI_MD_Demo();
    //GPM4_OBJTRACKER_Demo();
#if (CMSIS_DSP_DEMO == 1)
    arm_sin_cos_example();
#endif

#if (USBD_UAC_DEMO == 1)
    //USBD_UAC_Init();
    //usbd_uac_simple_demo();
    usbd_uac_demo();
#endif

#if (USBD_UVC_DEMO == 1)
    //usbd_uvc_uc_init();
    //usbd_uvc_simple_demo();
    usbd_uvc_demo();
#endif

#if (AUDIO_DECODE == 1)
	platform_entrance(0);
    audio_decode_simple_demo();
#endif

#if (AUDIO_DECODE_OGGMIX == 1)
	platform_entrance(0);
    audio_decode_oggmix_demo();
#endif

#if (AUDIO_I2S_DECODE == 1)
	platform_entrance_i2s(0);
    audio_i2s_decode_simple_demo();
#endif

#if (AUDIO_ENCODE == 1)
	platform_entrance(0);
    audio_encode_simple_demo();
#endif

#if (IMAGE_CODEC == 1)
#endif

#if (VIDEO_DECODE == 1)
#endif

#if (VIDEO_ENCODE == 1)
#endif

#if (DIGITAL_VIDEO_DEMO == 1)
    Digital_Video_Demo();
    //Digital_Video_With_Prcess_Demo();
#endif

#if (GPM4_PPU_DEMO == 1)
    GPM4_PPU_Demo();
    //ppu_dma_demo();
#endif

#if (FACE_DETECTION_DEMO == 1)
    while(1)
        GPM4_FD_Demo();
#endif

#if (USBD_MSDC_DEMO == 1)
//    usbd_msdc_init();
    usbd_msdc_demo();
#endif

#if IMAGE_DECODE_DEMO == 1
    platform_entrance(0);
    image_decode_simple_demo();
#endif

#if IMAGE_ENCODE_DEMO == 1
    platform_entrance(0);
    image_encode_simple_demo();
#endif

#if OBJECT_RECOGNIZE_DEMO == 1
    while(1)
        GPM4_OBJ_Demo();
#endif

#if (WIFI_DEMO == 1)
//     wifi_demo_init(WIFI_MJPEG_STREAMER_DEMO, WIFI_AP_MODE);		/* WIFI_AP_MODE => set WiFi enter AP mode, WIFI_STATION_MODE => set WiFi enter station mode*/
     wifi_demo_init(WIFI_GOPLUS_DEMO, WIFI_AP_MODE);		/* WIFI_AP_MODE => set WiFi enter AP mode, WIFI_STATION_MODE => set WiFi enter station mode*/
#endif

#if (VR_DEMO == 1)
    VrDemo();
#endif

#if (CSI_DISP_DEMO == 1)
    //GPM4_CSI_DISP_Demo();
    GPM4_CSI_MD_Demo();
#endif

#if CP_DEMO
	Camera_Process_Demo();
#endif

#if (CT_FDWA_VR_DEMO == 1)
    GPM4_CT_FDWA_VR_Demo();
#endif

#if (EHCI_DEMO == 1)
    #ifdef EHCI_UVC_EN
    ehci_uvc_demo();
    #endif
#endif

#if (COLOR_BALL_DEMO == 1)
    GPM4_COLOR_BALL_Demo();
#endif

#if (CHANGE_SYSTEM_PLL_DEMO == 1)
    GPM4_Change_System_Pll_Demo();
#endif

    while(1){osDelay(1000);}
	osThreadTerminate(NULL);
}

/*-----------------------------------------------------------*/
#define PPU_REAL_EN             1
INT32U ppu_ok,dma_ok,disp_buf_ok;
void PPU_IRQHandler(void)
{
    INT32U msg;
	INT32U enable = R_PPU_IRQ_EN;
	INT32U status = R_PPU_IRQ_STATUS;

	R_PPU_IRQ_STATUS = status;
	enable &= status;
#if PPU_REAL_EN == 1
    if((enable & 0x1) || (enable & 0x4))
    {
        ppu_isr_event(status);
    }

    if(enable & 0x2000)
    {
        if(disp_buf_ok)
            R_TFT_FBI_ADDR = disp_buf_ok;
    }
#else
    if(enable & 0x1)
        ppu_ok = 1;

    if(enable & 0x4)
        dma_ok = 1;
#endif
}

int main(void)
{
#if 1
    #define SP_V3D_BUG_EN           0
    #define SP_DMA_EN               0
    INT32U i,j,cnt,temp,src_hi;
    INT16U src_lo,*sp_ptr,*src_ptr;
///*
    SystemInit();

    R_SHCSR |= 0x00070000;
    MCLK = SystemCoreClock;
    MHZ = MCLK /1000000;
    drv_l1_system_arbiter_init();
	
#if SP_DMA_EN == 1
    R_PPU_ENABLE = 0x80;
    R_TFT_FBI_ADDR = (INT32U)_Text00100_IMG0000_CellData;
    tft_tpo_td025thd1_init();
	R_DMA0_SRC_ADDR = (INT32U)_SPRITE_GP22_CellData_3;
	R_DMA0_TAR_ADDR = (INT32U)_Text00100_IMG0000_CellData;
	R_DMA0_TX_COUNT = 0x1000;
	R_DMA0_SPRITE_SIZE = 0x40;
	R_DMA0_MISC = 0x8000;
	R_DMA_LINE_LEN = 0x140;
    R_DMA0_CTRL = 0x1109; // sin

    while(1);
#endif

#if 1 // vtech extend large sprite
    R_PPU_TEXT1_CTRL = 0;
    R_PPU_TEXT2_CTRL = 0;
    R_PPU_TEXT3_CTRL = 0;
    R_PPU_TEXT4_CTRL = 0;
    R_PPU_WINDOW0_X = 0x7FF;
    R_PPU_WINDOW0_Y = 0x7FF;
    R_PPU_WINDOW1_X = 0x7FF;
    R_PPU_WINDOW1_Y = 0x7FF;
    R_PPU_WINDOW2_X = 0x7FF;
    R_PPU_WINDOW2_Y = 0x7FF;
    R_PPU_WINDOW3_X = 0x7FF;
    R_PPU_WINDOW3_Y = 0x7FF;
    R_PPU_MISC = 0x14c;

    R_PPU_ENABLE = 0x138158f;
    R_FREE_SIZE = 0x014000f0;

    R_PPU_SPRITE_SEGMENT = 0;

    cnt = (INT32U)_SPRITE_GP22_CellData_2;
    cnt = cnt/2;
    src_lo = (cnt & 0xFFFF);
    src_hi = (cnt >> 16);

    R_PPU_SPRITE_CTRL = 0x111;
    R_PPU_EXTENDSPRITE_CONTROL = 0x105;
    //R_PPU_EXTENDSPRITE_CONTROL = 0x10D;

    cnt = 0x29bd00;
    R_PPU_EXTENDSPRITE_ADDR = cnt;

    R_PPU_FBO_ADDR = 0x1f38000;
    R_TFT_FBI_ADDR = 0x1f38000;

    // tft
    //tft_tpo_td025thd1_init();

    sp_ptr = (INT16U *)P_PPU_SPRITE_ATTRIBUTE_BASE;
    for(i=0;i<0xFFF;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_SPRITE_EXTERN_ATTRIBUTE_BASE;
    for(i=0;i<0x1FF;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM0;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM1;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM2;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM3;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_TEXT_H_OFFSET0;
    for(i=0;i<239;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_TEXT_HCMP_VALUE0;
    for(i=0;i<239;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_TEXT3_COS0;
    for(i=0;i<479;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)cnt;
    for(i=0;i<512;i++)
        *sp_ptr++ = 0;

    while(1)
    {
        sp_ptr = (INT16U *)cnt;
        *sp_ptr = src_lo;
        sp_ptr++;
        *sp_ptr = 0x381;
        sp_ptr++;
        *sp_ptr = 0x57;
        sp_ptr++;
        *sp_ptr = 0x1f3;
        //*sp_ptr = 0x153;
        sp_ptr++;
        *sp_ptr = src_hi;
        sp_ptr++;
        //*sp_ptr = 0x1bae;
        sp_ptr++;
        //*sp_ptr = 0x3ae;
        sp_ptr++;
        //*sp_ptr = 0x3a0;
        sp_ptr++;
        //*sp_ptr = 0x4; // large sprite
        sp_ptr++;
        //*sp_ptr = 0x4; // large sprite

        R_PPU_FB_GO = 0x1;
        while((R_PPU_IRQ_STATUS & 0x1) == 0);
        R_PPU_IRQ_STATUS = 0x1;
    }
#elif 0 // vtech sprite V3D mode
    R_PPU_TEXT1_CTRL = 0;
    R_PPU_TEXT2_CTRL = 0;
    R_PPU_TEXT3_CTRL = 0;
    R_PPU_TEXT4_CTRL = 0;
    R_PPU_WINDOW0_X = 0x7FF;
    R_PPU_WINDOW0_Y = 0x7FF;
    R_PPU_WINDOW1_X = 0x7FF;
    R_PPU_WINDOW1_Y = 0x7FF;
    R_PPU_WINDOW2_X = 0x7FF;
    R_PPU_WINDOW2_Y = 0x7FF;
    R_PPU_WINDOW3_X = 0x7FF;
    R_PPU_WINDOW3_Y = 0x7FF;
    R_PPU_MISC = 0x14c;

    R_PPU_ENABLE = 0x138178f;
    //R_PPU_ENABLE = 0x130178f;
    R_FREE_SIZE = 0x014000f0;
#if SP_V3D_BUG_EN == 1
    R_PPU_SPRITE_SEGMENT = 0x100002;
    //R_PPU_SPRITE_SEGMENT = 0x100000;
#else
    R_PPU_SPRITE_SEGMENT = 0;
    //R_PPU_SPRITE_SEGMENT = 0x100002;
#endif
    R_PPU_SPRITE_CTRL = 0x280115;

    src_ptr = (INT16U *)_SPRITE_GP22_CellData_2;
    cnt = (INT32U)_SPRITE_GP22_CellData_2;
    cnt = cnt/2;
    src_lo = (cnt & 0xFFFF);
    src_hi = (cnt >> 16);
    temp = 0x5fb812;
    //cnt = 0x5fb802;
    sp_ptr = (INT16U *)temp;//0x5fb800;
    for(i=0;i<64*64;i++)
        *sp_ptr++ = *src_ptr++;
    cnt = temp  - 0x100002;
    cnt = cnt/2;
    src_lo = (cnt & 0xFFFF);
    src_hi = (cnt >> 16);

#if 0
    cnt = 0x6fb802;
    //cnt = 0x5fb802;
    sp_ptr = (INT16U *)cnt;//0x5fb800;
    for(i=0;i<64*64;i++)
        *sp_ptr++ = *src_ptr++;
    cnt = 0x6fb802 - 0x100002;
    cnt = cnt/2;
    src_lo = (cnt & 0xFFFF);
    src_hi = (cnt >> 16);
#endif

    R_PPU_FBO_ADDR = 0x1f38000;
    R_TFT_FBI_ADDR = 0x1f38000;

    tft_tpo_td025thd1_init();
    sp_ptr = (INT16U *)P_PPU_SPRITE_ATTRIBUTE_BASE;
    for(i=0;i<0xFFF;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_SPRITE_EXTERN_ATTRIBUTE_BASE;
    for(i=0;i<0x1FF;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM0;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM1;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM2;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_PALETTE_RAM3;
    for(i=0;i<255;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_TEXT_H_OFFSET0;
    for(i=0;i<239;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_TEXT_HCMP_VALUE0;
    for(i=0;i<239;i++)
        *sp_ptr++ = 0;

    sp_ptr = (INT16U *)P_PPU_TEXT3_COS0;
    for(i=0;i<479;i++)
        *sp_ptr++ = 0;

    while(1)
    {
        sp_ptr = (INT16U *)P_PPU_SPRITE_ATTRIBUTE_BASE;
#if 0//SP_V3D_BUG_EN == 1
        *sp_ptr = 0xdbff;
        //*sp_ptr = 0xdc00;
#else
        *sp_ptr = src_lo;
#endif
        sp_ptr++;
        *sp_ptr = 0x93a0;
        sp_ptr++;
        *sp_ptr = 0x1824;
        sp_ptr++;
        *sp_ptr = 0x1f3;
        sp_ptr++;
#if 0//SP_V3D_BUG_EN == 1
        *sp_ptr = 0x27;
#else
        *sp_ptr = src_hi;
#endif
        sp_ptr++;
        *sp_ptr = 0x1bae;
        //*sp_ptr = 0x1bb4;
        //*sp_ptr = 0x1baf;
        sp_ptr++;
        *sp_ptr = 0x3ae;
        //*sp_ptr = 0x3b4;
        //*sp_ptr = 0x3af;
        sp_ptr++;
        *sp_ptr = 0x3a0;
        sp_ptr++;

#if 0  //sprite 1
#if 0//SP_V3D_BUG_EN == 1
        *sp_ptr = 0xdbff;
        //*sp_ptr = 0xdc00;
#else
        *sp_ptr = src_lo;
#endif
        sp_ptr++;
        *sp_ptr = 0x9004;
        sp_ptr++;
        *sp_ptr = 0x1824;
        sp_ptr++;
        *sp_ptr = 0x1f3;
        sp_ptr++;
#if 0//SP_V3D_BUG_EN == 1
        *sp_ptr = 0x27;
#else
        *sp_ptr = src_hi;
#endif
        sp_ptr++;
        //*sp_ptr = 0x1bae;
        *sp_ptr = 0x1812;
        sp_ptr++;
        //*sp_ptr = 0x3ae;
        *sp_ptr = 0x12;
        sp_ptr++;
        *sp_ptr = 0x4;
        sp_ptr++;
#endif

#if 0
        sp_ptr = (INT16U *)P_PPU_SPRITE_ATTRIBUTE_BASE;
#if SP_V3D_BUG_EN == 1
        *sp_ptr = 0xdbff;
#else
        *sp_ptr = src_lo;
#endif
        sp_ptr++;
        *sp_ptr = 0x93a0;
        sp_ptr++;
        *sp_ptr = 0x2824;
        sp_ptr++;
        *sp_ptr = 0x1f3;
        sp_ptr++;
#if SP_V3D_BUG_EN == 1
        *sp_ptr = 0x27;
#else
        *sp_ptr = src_hi;
#endif
        sp_ptr++;
        *sp_ptr = 0x2bec;
        sp_ptr++;
        *sp_ptr = 0xc3ec;
        sp_ptr++;
        *sp_ptr = 0xffa0;
        sp_ptr++;
        R_PPU_FB_GO = 0x1;
        while((R_PPU_IRQ_STATUS & 0x1) == 0);
        R_PPU_IRQ_STATUS = 0x1;
    #if 0
            for(i=0;i<0x10000;i++)
                j++;
            j=0;
    #endif
#endif

        R_PPU_FB_GO = 0x1;
        while((R_PPU_IRQ_STATUS & 0x1) == 0);
        R_PPU_IRQ_STATUS = 0x1;
    #if 0
            for(i=0;i<0x10000;i++)
                j++;
            j=0;
    #endif
    }
#else
    //cache_init();
    //drv_l1_init();
//*/
    NVIC_SetPriority(PPU_IRQn, 5);
    NVIC_EnableIRQ(PPU_IRQn);

    R_PPU_ENABLE = 0x80;
    R_PPU_IRQ_EN = 0x2005;
    tft_tpo_td025thd1_init();
    //while(1);
    gpio_init_io (IO_A15, GPIO_OUTPUT);
    gpio_set_port_attribute(IO_A15, 1);
    GPM4_PPU_Demo();

    R_PPU_TEXT1_CTRL = 0;
    R_PPU_TEXT2_CTRL = 0;
    R_PPU_TEXT3_CTRL = 0;
    R_PPU_TEXT4_CTRL = 0;
    R_PPU_WINDOW0_X = 0x7FF;
    R_PPU_WINDOW0_Y = 0x7FF;
    R_PPU_WINDOW1_X = 0x7FF;
    R_PPU_WINDOW1_Y = 0x7FF;
    R_PPU_WINDOW2_X = 0x7FF;
    R_PPU_WINDOW2_Y = 0x7FF;
    R_PPU_WINDOW3_X = 0x7FF;
    R_PPU_WINDOW3_Y = 0x7FF;
    R_PPU_MISC = 0x40;
    //R_PPU_ENABLE = 0x138158f;
    R_PPU_ENABLE = 0x80789;
    R_FREE_SIZE = 0x014000f0;
    R_PPU_SPRITE_SEGMENT = 0x64e50;
    R_PPU_IRQ_EN = 0x2005;
    R_TFT_CTRL = 0x319;
    ///*
    sp_ptr = (INT16U *)0x29bd00;
#if 0
    for(i=0;i<256;i++)
        *sp_ptr++ = 0;
#elif 1
    *sp_ptr = 0x0001;
    sp_ptr++;
    *sp_ptr = 0x0177;
    sp_ptr++;
    *sp_ptr = 0x000b;
    sp_ptr++;
    *sp_ptr = 0x415d;
    sp_ptr++;
    *sp_ptr = 0x0c00;
#endif
    //*/
    R_PPU_EXTENDSPRITE_ADDR = 0x29bd00;
    R_PPU_FBO_ADDR = 0x228380;
    ///*
    sp_ptr = (INT16U *)0x1f1a608;
#if 0
    for(i=0;i<256;i++)
        *sp_ptr++ = 0;
#elif 1
    *sp_ptr = 0x0001;
    sp_ptr++;
    *sp_ptr = 0x013f;
    sp_ptr++;
    *sp_ptr = 0x003d;
    sp_ptr++;
    *sp_ptr = 0x005d;
    sp_ptr++;
    *sp_ptr = 0x4000;
#endif
    //*/
    //R_PPU_SPRITE_CTRL = 0x115;
    R_PPU_SPRITE_CTRL = 0x800e5;
    R_PPU_EXTENDSPRITE_CONTROL = 0x201;
    //R_PPU_EXTENDSPRITE_CONTROL = 0x203;
    cnt = 0;
    while(1)
    {
        dma_ok = 0;
        R_PPU_SPRITE_DMA_SOURCE = 0x1f1a608;
        R_PPU_SPRITE_DMA_TARGET = (INT32U)P_PPU_SPRITE_ATTRIBUTE_BASE & 0x7FFF;
        R_PPU_SPRITE_DMA_NUMBER = 0xFFF;
        while(1)
        {
            if(dma_ok)
                break;
        }
        dma_ok = 0;
        R_PPU_SPRITE_DMA_SOURCE = 0x1f1e890;
        R_PPU_SPRITE_DMA_TARGET = (INT32U) P_PPU_SPRITE_EXTERN_ATTRIBUTE_BASE & 0x7FFF;
        R_PPU_SPRITE_DMA_NUMBER = 0x1FF;
        while(1)
        {
            if(dma_ok)
                break;
        }
        ppu_ok = 0;
        R_PPU_FB_GO = 0x1;
        while(1)
        {
            if(ppu_ok)
                break;
        }
#if 0
        for(i=0;i<0x10000;i++)
            j++;
        j=0;
#endif
        //while((R_PPU_IRQ_STATUS & 0x1) == 0);
        //R_PPU_IRQ_STATUS = 0x1;
        cnt++;
        gpio_write_io(IO_A15, (cnt&0x1));
    }
#endif
#else

    osThreadDef(InitTask, osPriorityNormal, 0, STACKSIZE);

    SystemInit();

    board_init();

    drv_l1_init();

    drv_l2_init();

    #if (GPLIB_FILE_SYSTEM_EN == 1)
    fs_init();              // Initiate file system module
    #endif

    #if 1 && defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
	osThreadDef(CmdTask, osPriorityAboveNormal, 0, STACKSIZE);
  	cmdTaskHandle = osThreadCreate(&os_thread_def_CmdTask, NULL);
    #endif

    #if 0 || (CPLUS_STRING_DEMO == 1) // if you need to use interrupt before osKernelStart, need to turn on below code
    __asm volatile( "dsb" );
    vPortSetBASEPRI(0); // since before os start, using os memory allocation will cause basepri
                        // set to 0xa0, so restore baseprit to 0 to allow interrupt happen
    __asm volatile ("cpsie f"); // since bootloader and rom code will off interrupt, so enable it
    __asm volatile ("cpsie i"); // since bootloader and rom code will off interrupt, so enable it
    __asm volatile( "isb" );
    #endif

#if (CPLUS_STRING_DEMO == 1)
    string_sample();
#endif

    osKernelStart(&os_thread_def_InitTask, NULL);
#endif
    /* Never go here */
    return 0;
}

#if 1 && defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)

extern void OS_Cmd(void);
extern void Mem_Cmd(void);

void CmdTask(void const *param)
{
    DBG_PRINT("%s ... \r\n",__func__);

    #if (CMD_OS == 1)
    OS_Cmd();
    #endif

    #if (CMD_MEM == 1)
    Mem_Cmd();
    #endif

    Cmd_Task((void *)param);
}
#endif // GPLIB_CONSOLE_EN

