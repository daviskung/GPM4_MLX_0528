#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "gplib.h"

#include "extflash.h"

/**
 *  \brief The size of the tts.bin file in bytes.
 */
static INT32U nandFlashSize = 0u;

/**
 *  \brief Data buffer containing the database content.
 */
static INT8U* memdb = NULL;

extern int gp_EXTFLASH_Init(void)
{
    struct sfn_info fInfo;
    INT16S ttsfh = fs_open("F:\\tts_16KHz.bin", O_RDONLY);

    if(ttsfh < 0){
        DBG_PRINT("fail to open voice data base \r\n");
        return -1;
    }


    if (sfn_stat(ttsfh, &fInfo) == 0)
    {
        nandFlashSize = fInfo.f_size;
    }

    // Allocate enough memory to load the full database.
    memdb = (INT8U*)gp_malloc(nandFlashSize);

    // Read the entire content into memory.
    if (memdb != NULL)
    {
        if (fs_read(ttsfh, (INT32U) memdb, nandFlashSize) < ((INT32S)nandFlashSize))
        {
            DBG_PRINT("Failed to load the entire database into memory.\n");
            gp_free(memdb);
            memdb = NULL;
        }
    }
    else
    {
        DBG_PRINT("Failed to allocate memory to load the database into memory.\n");
        return -1;
    }

    fs_close(ttsfh);
    return 0;
}

void gp_EXTFLASH_Process(void)
{
    // Do nothing.
}

int gp_EXTFLASH_DisableMemoryMapping(void)
{
    // No memory mapping support.
    return 0;
}

int gp_EXTFLASH_EnableMemoryMapping(void)
{
    // No memory mapping support.
    return -1;
}

int gp_EXTFLASH_MemoryMappingEnabled(void)
{
    // No memory mapping support.
    return 0;
}

int gp_EXTFLASH_Write(uint32_t baseAddr, void * data, uint32_t size)
{
    // Not Implemented.
    return -1;
}

int gp_EXTFLASH_Erase(uint32_t baseAddr, void * data, uint32_t size)
{
    // Not Implemented.
    return -1;
}

int gp_EXTFLASH_Read(void * data, uint32_t baseAddr, uint32_t size)
{
    if ((data == NULL) || (memdb == NULL))
    {
        return -1;
    }

    gp_memcpy(data, memdb + baseAddr, size);

    return (int)size;
}

void * gp_EXTFLASH_GetCashedDataLocation(uint32_t baseAddr, uint32_t size)
{
    return memdb + baseAddr;
}

int gp_EXTFLASH_TotalFlashSize(void)
{
    return (nandFlashSize != 0u) ? (int)nandFlashSize : -1;
}
