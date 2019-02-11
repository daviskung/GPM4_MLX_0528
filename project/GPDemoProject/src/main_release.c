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

#define AUDIO_DECODE            0
#define AUDIO_DECODE_OGGMIX     0
#define AUDIO_I2S_DECODE        0
#define AUDIO_ENCODE            0
#define DIGITAL_VIDEO_DEMO      0
#define GPM4_PPU_DEMO           0
#define FACE_DETECTION_DEMO     0
#define USBD_UAC_DEMO           0
#define USBD_UVC_DEMO           0	//if #define UVC_H264  1 @ usbd_uvc_dec.c, please #define AVI_AUDIO_ENCODE_EN   0 @ avi_encoder_scaler_jpeg.h
                                    //And select camera resolution USBD_UVC_CAM_RES in application_cfg.h. Default is CAM_RES_720P
#define USBD_MSDC_DEMO          0
#define IMAGE_DECODE_DEMO       0
#define IMAGE_ENCODE_DEMO       0
#define OBJECT_RECOGNIZE_DEMO   0
#define WIFI_DEMO               0
#define VR_DEMO                 0   //to run VrDemo(); please set VR_DEMO_OPTION as VR_ONLY
#define CSI_DISP_DEMO           0
#define CP_DEMO					0
#define CT_FDWA_VR_DEMO         0
#define CMSIS_DSP_DEMO          0
#define EHCI_DEMO               0   //to run ECHI_DEMO, please declare "EHCI_UVC_EN" in usbh_uvc.h
#define CPLUS_STRING_DEMO       0
#define CHANGE_SYSTEM_PLL_DEMO  0
#define TTS_DEMO                0   //In order to get input from PC terminal correctly, please disable GPLIB_CONSOLE_EN in gplib_cfg.h
#define TTS_VR_DEMO             0   //to run TTS_VR_Demo(); please remove tts_demo.c and add tts_vr_demo.c in the project and then set VR_DEMO_OPTION as VR_TTS
#define GPBTM03_DEMO            0

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
extern INT32S ehci_uvc_demo(void);
extern int32_t arm_sin_cos_example(void);
extern INT32S usbd_msdc_demo(void);
extern void GPM4_Change_System_Pll_Demo(void);
extern void gpbt03_app_demo(void);
extern void VrDemo(void);
extern void TTS_Demo(void);
extern void TTS_VR_Demo(void);
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

#if (CPLUS_STRING_DEMO == 1)
    string_sample();
#endif

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

#if (DIGITAL_VIDEO_DEMO == 1)
    Digital_Video_Demo();
#endif

#if (GPM4_PPU_DEMO == 1)
    GPM4_PPU_Demo();
#endif

#if (FACE_DETECTION_DEMO == 1)
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
    GPM4_OBJ_Demo();
#endif

#if (WIFI_DEMO == 1)
    //wifi_demo_init(WIFI_MJPEG_STREAMER_DEMO, WIFI_AP_MODE);		/* WIFI_AP_MODE => set WiFi enter AP mode, WIFI_STATION_MODE => set WiFi enter station mode*/
    //wifi_demo_init(WIFI_SSL_DEMO, WIFI_STATION_MODE);
    //wifi_demo_init(WIFI_TUTK_DEMO, WIFI_TUTK_MODE);
    wifi_demo_init(WIFI_GOPLUS_DEMO, WIFI_AP_MODE);

#endif

#if (VR_DEMO == 1)
    VrDemo();       /* to run VrDemo(); please set VR_DEMO_OPTION as VR_ONLY */
#endif

#if (CSI_DISP_DEMO == 1)
    GPM4_CSI_DISP_Demo();
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

#if (CHANGE_SYSTEM_PLL_DEMO == 1)
    GPM4_Change_System_Pll_Demo();
#endif

#if (TTS_DEMO == 1)
    TTS_Demo();
#endif
#if (TTS_VR_DEMO == 1)
    TTS_VR_Demo();      /* to run TTS_VR_Demo(); please set VR_DEMO_OPTION as VR_TTS */
#endif
#if (GPBTM03_DEMO == 1)
    gpbt03_app_demo();
#endif

    while(1){osDelay(1000);}
	osThreadTerminate(NULL);
}

/*-----------------------------------------------------------*/
int main(void)
{

    osThreadDef(InitTask, osPriorityNormal, 0, STACKSIZE);

    SystemInit();

    board_init();

    drv_l1_init();

    drv_l2_init();

    #if (GPLIB_FILE_SYSTEM_EN == 1)
    fs_init();              // Initiate file system module
    #endif

    #if (CPLUS_STRING_DEMO == 0) && defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
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

    osKernelStart(&os_thread_def_InitTask, NULL);

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

