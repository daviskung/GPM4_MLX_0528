#ifndef SYSTEMDEV_H_
#define SYSTEMDEV_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief When implemented, Initialises system specific structures that
 * have not yet been initialized.
 * This function is called by the TTS library upon its initialization.
 * If no system initialization is required at this point, the implementation of the
 * function may be empty.
 */
extern void SYSTEM_Init(void);

/**
 * \brief When implemented, instructs the system to sleep for \p msec milliseconds.
 *
 * \param msec The number of milliseconds to sleep.
 */
extern void SYSTEM_Sleep(uint32_t msec);


/**
 * \brief When implemented, Returns the current tick count in milliseconds.
 *
 * \return The current tick count in milliseconds.
 */
extern uint32_t SYSTEM_GetTickCount(void);


/**
 * \brief When implemented, returns the size, in bytes, of the TTS library internal heap.
 * And a pointer to the memory address where the memory block starts. Integrator must allocate
 * the memory in the most adequate way for the hosting platform. The start of the memory block
 * should be 32-bit aligned.
 *
 * \param ppHeap The pointer to the address where the memory block is allocated.
 * \return  The size of the memory block in bytes.
 */
extern uint32_t SYSTEM_GetTTSLHeap(void** ppHeap);


#ifdef __cplusplus
}
#endif

#endif  // SYSTEMDEV_H_
