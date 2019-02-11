#include "application.h"

//========================= Platform Initialization ===========================
//
// Task Create : Audio, DAC, Image, File, AP(Demo Code) Task
// Global Variable Initialization
// Component Initialization : File system, Memory Manager,
// Driver Initialization: DMA Manager, SDC, USB Host
//=============================================================================
extern INT32U platform_entrance(void* free_memory);
extern INT32U platform_entrance_i2s(void* free_memory);
extern void platform_exit(void);

