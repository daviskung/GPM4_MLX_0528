#include "fsystem.h"


#if _OPERATING_SYSTEM != _OS_NONE

#if _OPERATING_SYSTEM == _OS_UCOS2
OS_EVENT *gFS_sem;
#elif _OPERATING_SYSTEM == _OS_FREERTOS
xSemaphoreHandle gFS_sem;
#endif

#endif

INT32S FS_OS_Init(void) 
{
#if _OPERATING_SYSTEM != _OS_NONE
  #if _OPERATING_SYSTEM == _OS_UCOS2
    gFS_sem = OSSemCreate(1);
  #elif _OPERATING_SYSTEM == _OS_FREERTOS
    gFS_sem = xSemaphoreCreateMutex();
  #endif  
    
#endif

    return (0);
}

INT32S FS_OS_Exit(void) 
{
    return (0);
}

void FS_OS_LOCK(void)
{

  INT8U err = NULL;	  
  
#if _OPERATING_SYSTEM != _OS_NONE
  #if _OPERATING_SYSTEM == _OS_UCOS2
	OSSemPend(gFS_sem, 0, &err);
  #elif _OPERATING_SYSTEM == _OS_FREERTOS
        err = xSemaphoreTake(gFS_sem, portMAX_DELAY);
  #endif        
#endif
}

void FS_OS_UNLOCK(void)
{
#if _OPERATING_SYSTEM != _OS_NONE
  #if _OPERATING_SYSTEM == _OS_UCOS2
	OSSemPost(gFS_sem);
  #elif _OPERATING_SYSTEM == _OS_FREERTOS
        xSemaphoreGive( gFS_sem );
  #endif              
#endif
}

/****************************************************************/
/*																*/
/*			       seek speedup									*/
/*																*/
/*		  seek speedup memery management function               */
/*																*/
/****************************************************************/
INT32U FS_SeekSpeedup_Malloc(INT32U len)
{
	return (INT32U)pvPortMalloc(len << 1);
}

void FS_SeekSpeedup_Free(INT32U addr)
{
	vPortFree((void*)addr);
}

/****************************************************************/
/*																*/
/*			       getdate function								*/
/*																*/
/*		  user write this file to assign the file system date   */
/*																*/
/****************************************************************/
void FS_OS_GetDate(dosdate_t *dd)
{
	dd->year = 2004;
	dd->month = 8;
	dd->monthday = 23;
}

/****************************************************************/
/*																*/
/*			       gettime function								*/
/*																*/
/*		  user write this file to assign the file system time   */
/*																*/
/****************************************************************/
void FS_OS_GetTime(dostime_t *dt)
{
	dt->hour = 16;
	dt->minute = 54;
	dt->second = 37;
	dt->hundredth = 0;
}


