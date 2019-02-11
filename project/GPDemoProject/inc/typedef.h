#ifndef __PROJECT_H__
#define __PROJECT_H__

typedef char                    CHAR;   // By default, char is unsigned. It can be changed to signed by compiler option
typedef unsigned char           BOOLEAN;
typedef unsigned char           INT8U;
typedef signed char             INT8;
typedef signed char             INT8S;
typedef unsigned short          INT16U;
typedef signed short            INT16S;
typedef signed short            INT16;
typedef unsigned int            INT32U;
typedef signed int              INT32S;
typedef float                   FP32;
typedef long long               INT64S;
typedef unsigned long long      INT64U;
typedef double                  FP64;
typedef	int                     INT32;

#ifndef __ALIGN_DEFINE__
#define __ALIGN_DEFINE__

#if defined(__CC_ARM)
    #define ALIGN64 __align(64)
    #define ALIGN32 __align(32)
    #define ALIGN16 __align(16)
    #define ALIGN8 __align(8)
    #define ALIGN4 __align(4)
#elif defined(__GNUC__)
    #define ALIGN64 __attribute__ ((aligned(64)))
    #define ALIGN32 __attribute__ ((aligned(32)))
    #define ALIGN16 __attribute__ ((aligned(16)))
    #define ALIGN8 __attribute__ ((aligned(8)))
    #define ALIGN4 __attribute__ ((aligned(4)))
#elif defined(__ICCARM__)
    #define ALIGN64 _Pragma("data_alignment=64")
    #define ALIGN32 _Pragma("data_alignment=32")
    #define ALIGN16 _Pragma("data_alignment=16")
    #define ALIGN8 _Pragma("data_alignment=8")
    #define ALIGN4 _Pragma("data_alignment=4")
#endif

#endif

#if defined(__GNUC__)
    #define ASM(asm_code)  asm(#asm_code);
    #define IRQ            __attribute__ ((interrupt))
    #define PACKED         __attribute__((__packed__))
#elif defined(__ICCARM__)
    #define ASM(asm_code)  asm(#asm_code);
    #define IRQ            __irq
    #define PACKED         __packed
#elif defined(__CC_ARM)
    #define ASM(asm_code)  __asm {asm_code}  /*ADS embeded asm*/
    #define IRQ            __irq
    #define PACKED         __packed
#endif

#endif // __PROJECT_H__
