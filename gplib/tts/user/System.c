#include "cmsis_os.h"

#include "SystemDev.h"

#ifndef TTSLHEAP_SIZE
/**
 *  \brief Defines a size for the library heap in bytes.
 */
#define TTSLHEAP_SIZE 50000
#endif  // TTSLHEAP_SIZE

/**
 *  \brief Defines the size of the library heap in blocks of 32 bit.
 */
#define TTSLHEAP_LENGTH (TTSLHEAP_SIZE / 4)

/**
 *  \brief Allocates heap memory statically.
 */
static uint32_t ttslHeap[TTSLHEAP_LENGTH];

void SYSTEM_Init(void)
{
    // Add system initialization if needed.
}

void SYSTEM_Sleep(uint32_t delay)
{
    // Delay milliseconds.
    (void) osDelay (delay); //Jason 23Aug2016: use CMSIS function
}

uint32_t SYSTEM_GetTickCount(void)
{
    // Return ticks.
    return (uint32_t)xTaskGetTickCount(); //Jason 23Aug2016: unit 1ms
}

uint32_t SYSTEM_GetTTSLHeap(void ** ppHeap)
{
    // Use statically allocated heap.
    // This call can also be replaced with a malloc if desired,
    // it is however relevant to provide a valid memory address
    // as the TTS library depends on it. The provided memory will
    // be managed by the library.
    *ppHeap = ttslHeap;
    return sizeof(ttslHeap);
}
