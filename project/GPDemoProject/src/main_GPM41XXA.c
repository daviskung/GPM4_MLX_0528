#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_l1.h"
#include "drv_l2.h"
#include "cmsis_os.h"
#include "gplib.h"

#define AUDIO_DECODE            1
#define AUDIO_ENCODE            0
#define I80_DISPLAY_DEMO        0
#define USBD_UAC_DEMO           0
#define USBD_MSDC_DEMO          0
#define VR_DEMO                 0
#define CMSIS_DSP_DEMO          0

#if defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
#include "console.h"
void CmdTask(void const *param);
xTaskHandle cmdTaskHandle = NULL;
#endif

#define STACKSIZE       2048

/*-----------------------------------------------------------*/
void vPortSetupTimerInterrupt()
{
/* Configure SysTick to interrupt at the requested rate. */
    SysTick_Config (32768 / configTICK_RATE_HZ);
}


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

#if (CMSIS_DSP_DEMO == 1)
    arm_sin_cos_example();
#endif

#if (USBD_UAC_DEMO == 1)
    //USBD_UAC_Init();
    usbd_uac_simple_demo();
#endif

#if (USBD_UVC_DEMO == 1)
    //usbd_uvc_uc_init();
    usbd_uvc_simple_demo();
#endif

#if (AUDIO_DECODE == 1)
	platform_entrance(0);
    audio_decode_simple_demo();
#endif

#if (AUDIO_ENCODE == 1)
	platform_entrance(0);
    audio_encode_simple_demo();
#endif

#if (USBD_MSDC_DEMO == 1)
    usbd_msdc_init();
#endif

#if I80_DISPLAY_DEMO == 1
    display_test();
#endif

#if (VR_DEMO == 1)
    VrDemo();
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

    #if (GPLIB_FILE_SYSTEM_EN == 1)
    fs_init();              // Initiate file system module
    #endif

    #if 0 && defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
	osThreadDef(CmdTask, osPriorityAboveNormal, 0, STACKSIZE);
  	cmdTaskHandle = osThreadCreate(&os_thread_def_CmdTask, NULL);
    #endif

    #if 0 // if you need to use interrupt before osKernelStart, need to turn on below code
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

#if 0 && defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)

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

