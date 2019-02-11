#ifndef EXTFLASH_H_
#define EXTFLASH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief When implemented, initialise the external flash module to enable access to externally
 *  stored voice database. The application must call this function prior to initializing the TTS library.
 *  In case the external storage does not require initialization or if the initialisation is performed
 *  by some other system call, the function may be implemented empty.
 */
extern void EXTFLASH_Init(void);

/**
 *  \brief When implemented, releases any resources allocated in order to access the external flash module.
 *  Application should call this function when access to the external storage is no longer required.
 */
extern void EXTFLASH_DeInit(void);

/**
 *  \brief When implemented, instructs the flash module access to continue execution. It provides for a mechanism
 *  to perform "concurrent" access to the storage enabling caching mechanisms and non-blocking calls when accessing
 *  the external storage. Application should call this from within its main loop.
 *  If such mechanisms are not required by the hosting system, the implementation may be empty.
 */
extern void EXTFLASH_Process(void);

/**
 *  \brief When implemented, reads a block of data data from the external flash.
 *  This function will be called from the library during the initialization of the TTS library.
 *  In case any caching mechanisms are being used in this implementation, it is safe to skip them
 *  when this function is called, since the requested data is not going to be requested again after
 *  initialization.
 *
 *  \param data The pointer to the data buffer where the read data should be stored.
 *  \param baseAddr The address offset in the external flash of the block of data to be read.
 *  \param size The size of the block of data to read, in bytes.
 *
 *  \return A negative value indicates failure. A value of 0 or greater indicates success.
 */
extern int EXTFLASH_Read(void * data, uint32_t baseAddr, uint32_t size);

/**
 *  \brief When implemented, will return a pointer to the requested data.
 *  This function will be called from the library every time it needs data from the voice database
 *  in order to synthesize the audio.
 *  The data must be persistent until the next call to this function.
 *  In case no actual caching mechanism is being used in this implementation, it is still necessary,
 *  to pre-allocate at least one buffer so that the data can persist between calls.
 *
 *  \param baseAddr The address offset in the external flash of the data to be read.
 *  \param size The size of the block of data to read, in bytes.
 *
 *  \return A pointer to where the requested data is. A value of NULL when an error occurred.
 */
extern void * EXTFLASH_GetCashedDataLocation(uint32_t baseAddr, uint32_t size);

/**
 *  \brief When implemented returns the total size of the external flash, in bytes.
 *  The library will call this function upon its initialisation.
 *
 *  \return The external flash size, in bytes.
 */
extern int EXTFLASH_TotalFlashSize(void);

/**
 *  \brief When implemented, indicates whether memory-mapped I/O is enabled.
 *  The library will call this function upon initialization to determine if direct memory access can be used
 *  to access the voice database.
 *
 *  \return If memory-mapped I/O is not supported, 0 is returned. Otherwise a non-zero number is returned.
 */
extern int EXTFLASH_MemoryMappingEnabled(void);

/**
 *  \brief When implemented, disables the memory-mapping of the flash module. In case it is not possible, or not intended,
 *  the implementation may be empty.
 *
 *  \return A negative number indicates failure. A value of 0 or greater indicates success. In case memory-mapped I/O is not
 *  available returns 0.
 */
extern int EXTFLASH_DisableMemoryMapping(void);

/**
 *  \brief When implemented, enables memory-mapped IO on the flash module.
 *  This implementation is only relevant when memory-mapped I/O is available.
 *
 *  \return A negative number indicates failure. A value of 0 or greater indicates success. In case memory-mapped I/O is not
 *  available returns -1.
 */
extern int EXTFLASH_EnableMemoryMapping(void);

/**
 *  \brief When implemented, erases a section of flash data and optionally writes a buffer of data in the same block.
 *  The implementation of this functionality is not required for the TTS library to work properly and may be empty.
 *
 *  \param baseAddr The address in flash memory of the block to be erased.
 *  \param data If not NULL, the data will be written in the erased data block.
 *  \param size The number of bytes to erase.
 *
 *  \return A negative number indicates failure. Zero and above indicate success.
 */
extern int EXTFLASH_Erase(uint32_t baseAddr, void * data, uint32_t size);

/**
 *  \brief When implemented, writes a block of data to flash. The implementation of this functionality is not
 *  required for the TTS library to work properly and may be empty.
 *
 *  \param baseAddr The address in flash memory of where to start writing.
 *  \param data The pointer the data buffer that should be written.
 *  \param size The size of the data buffer in bytes.of data that needs to be written.
 *
 *  \return A negative number indicates failure. Zero and above indicate success.
 */
extern int EXTFLASH_Write(uint32_t baseAddr, void * data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif  // EXTFLASH_H_
