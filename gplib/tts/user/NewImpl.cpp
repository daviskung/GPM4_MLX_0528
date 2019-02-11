#include <stdint.h>
#include <stddef.h>

#include "project.h"

// Externals defined in the platform for memory allocation.
extern "C"
{
    extern "C" void* gp_malloc(INT32U size);
    extern "C" void gp_free(void* ptr);
}

/**
 *  \brief Allocates a memory block in the heap.
 *
 *  \param nSize The number of bytes to allocate.
 *
 *  \return The pointer to the newly allocated memory.
 */
void* operator new(size_t nSize)
{
    return gp_malloc(nSize);
}

/**
 *  \brief Allocates a memory block in the heap.
 *
 *  \param nSize The number of bytes to allocate.
 *
 *  \return The pointer to the newly allocated memory.
 */
void* operator new[](size_t nSize)
{
    return gp_malloc(nSize);
}

/**
 *  \brief Deletes a block of memory from the heap.
 *
 *  \param p Pointer to the memory block to release.
 */
void operator delete(void* p)
{
    gp_free(p);
}

/**
 *  \brief Deletes a block of memory from the heap.
 *
 *  \param p Pointer to the memory block to release.
 */
void operator delete[](void* p)
{
    gp_free(p);
}
