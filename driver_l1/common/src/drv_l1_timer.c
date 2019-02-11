/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2014 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#include "drv_l1_sfr.h"
#include "drv_l1_timer.h"
#include "drv_l1_tools.h"

#if (defined _DRV_L1_TIMER) && (_DRV_L1_TIMER == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define TIMER_TRUE  	1
#define TIMER_FALSE 	0

#define TIME_BASE_FALSE	0
#define TIME_BASE_TRUE	1

/*Timer Counter Bits Mode define*/
#define TIMER_COUNTER_BIT16     16
#define TIMER_COUNTER_BIT32     32
#define TIMER_COUNTER_BIT_MODE  TIMER_COUNTER_BIT16

/*TIMER register address define*/
#define EACH_DATA_REG_OFFSET       	0x00000008
#define EACH_REG_OFFSET             0x00000004
#define TIMER_BASE_ADDR             0xC0020000
/* TimerA address define */
#define TIMERA_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x00)  /*0xC0020000*/
#define TIMERA_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x01)  /*0xC0020004*/
#define TIMERA_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x02)  /*0xC0020008*/ /*User set the value to count from*/
#define TIMERA_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x03)  /*0xC002000C*/
#define TIMERA_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x04)  /*0xC0020010*/
#define TIMERA_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x05)  /*0xC0020014*/
#define TIMERA_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x07)  /*0xC002001C*/
/* TimerB address define */
#define TIMERB_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x08)  /*0xC0020020*/
#define TIMERB_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x09)  /*0xC0020024*/
#define TIMERB_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x0A)  /*0xC0020028*/ /*User set the value to count from*/
#define TIMERB_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x0B)  /*0xC002002C*/
#define TIMERB_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x0C)  /*0xC0020030*/
#define TIMERB_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x0D)  /*0xC0020034*/
#define TIMERB_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x0F)  /*0xC002003C*/
/* TimerC address define */
#define TIMERC_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x10)  /*0xC0020040*/
#define TIMERC_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x11)  /*0xC0020044*/
#define TIMERC_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x12)  /*0xC0020048*/ /*User set the value to count from*/
#define TIMERC_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x13)  /*0xC002004C*/
#define TIMERC_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x14)  /*0xC0020050*/
#define TIMERC_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x15)  /*0xC0020054*/
#define TIMERC_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x17)  /*0xC002005C*/
/* TimerD address define */
#define TIMERD_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x18)  /*0xC0020060*/
#define TIMERD_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x19)  /*0xC0020064*/
#define TIMERD_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x1A)  /*0xC0020068*/ /*User set the value to count from*/
#define TIMERD_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x1B)  /*0xC002006C*/
#define TIMERD_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x1C)  /*0xC0020070*/
#define TIMERD_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x1D)  /*0xC0020074*/
#define TIMERD_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x1F)  /*0xC002007C*/
/* TimerE address define */
#define TIMERE_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x20)  /*0xC0020080*/
#define TIMERE_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x21)  /*0xC0020084*/
#define TIMERE_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x22)  /*0xC0020088*/ /*User set the value to count from*/
#define TIMERE_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x23)  /*0xC002008C*/
#define TIMERE_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x24)  /*0xC0020090*/
#define TIMERE_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x25)  /*0xC0020094*/
#define TIMERE_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x27)  /*0xC002009C*/
/* TimerD address define */
#define TIMERF_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x28)  /*0xC00200A0*/
#define TIMERF_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x29)  /*0xC00200A4*/
#define TIMERF_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x2A)  /*0xC00200A8*/ /*User set the value to count from*/
#define TIMERF_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x2B)  /*0xC00200AC*/
#define TIMERF_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x2C)  /*0xC00200B0*/
#define TIMERF_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x2D)  /*0xC00200B4*/
#define TIMERF_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x2F)  /*0xC00200BC*/
/* TimerG address define */
#define TIMERG_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x30)  /*0xC00200C0*/
#define TIMERG_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x31)  /*0xC00200C4*/
#define TIMERG_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x32)  /*0xC00200C8*/ /*User set the value to count from*/
#define TIMERG_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x33)  /*0xC00200CC*/
#define TIMERG_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x34)  /*0xC00200D0*/
#define TIMERG_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x35)  /*0xC00200D4*/
#define TIMERG_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x37)  /*0xC00200DC*/
/* TimerH address define */
#define TIMERH_CTRL_ADDR            (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x38)  /*0xC00200E0*/
#define TIMERH_CCP_CTRL_ADDR        (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x39)  /*0xC00200E4*/
#define TIMERH_PRELOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x3A)  /*0xC00200E8*/ /*User set the value to count from*/
#define TIMERH_CCP_REG_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x3B)  /*0xC00200EC*/
#define TIMERH_UPCOUNT_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x3C)  /*0xC00200F0*/
#define TIMERH_CMPLOAD_ADDR         (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x3D)  /*0xC00200F4*/
#define TIMERH_STATUS               (TIMER_BASE_ADDR+EACH_REG_OFFSET*0x3F)  /*0xC00200FC*/

// Time Base: 0xC0030000
#define TIMEBASE_BASE_ADDR          0xC0030000
#define TIMEBASEA_CTRL_ADDR         (TIMEBASE_BASE_ADDR+EACH_REG_OFFSET*0x00)  /*0xC0030000*/
#define TIMEBASEB_CTRL_ADDR         (TIMEBASE_BASE_ADDR+EACH_REG_OFFSET*0x01)  /*0xC0030004*/
#define TIMEBASEC_CTRL_ADDR         (TIMEBASE_BASE_ADDR+EACH_REG_OFFSET*0x02)  /*0xC0030008*/
#define TIMEBASE_RESET_ADDR         (TIMEBASE_BASE_ADDR+EACH_REG_OFFSET*0x08)  /*0xC0030020*/


/*TIMER MASK definition*/
#define TIMER_INT_FLAG_MASK     0x00008000
#define GET16BITS_MASK          0x0000FFFF
#define GET32BITS_MASK          0xFFFFFFFF
#if TIMER_COUNTER_BIT_MODE==TIMER_COUNTER_BIT32
#define GETBITS_MASK            GET32BITS_MASK
#else
#define GETBITS_MASK            GET16BITS_MASK
#endif

#define SET_TMXIE               0x00004000
#define SET_TMXEN               0x00002000
#define TIMER_PRELOAD_COUNTER_SET(TIMERID,START_COUNT) DRV_Reg32(TIMERA_PRELOAD_ADDR+EACH_REG_OFFSET*8*TIMERID) = (START_COUNT & GETBITS_MASK); //R_TIMERA_PRELOAD &= (start_count & GET16BITS_MASK)
#define TIMER_CCP_REG_SET(TIMERID,TMXUCR) DRV_Reg32(TIMERA_CCP_REG_ADDR+EACH_REG_OFFSET*8*TIMERID) = (TMXUCR & GETBITS_MASK);

//for CMPLOAD
#define TIMER_CMPLOAD_REG_SET(TIMERID,TMXUCR) DRV_Reg32(TIMERA_CMPLOAD_ADDR+EACH_REG_OFFSET*8*TIMERID) = (TMXUCR & GET32BITS_MASK);

#define TIMERA_INT_ENABLE       R_TIMERA_CTRL |= SET_TMXIE
#define TIMERA_INT_DISABLE      R_TIMERA_CTRL &= ~SET_TMXIE
#define TIMERB_INT_ENABLE       R_TIMERB_CTRL |= SET_TMXIE
#define TIMERB_INT_DISABLE      R_TIMERB_CTRL &= ~SET_TMXIE
#define TIMERC_INT_ENABLE       R_TIMERC_CTRL |= SET_TMXIE
#define TIMERC_INT_DISABLE      R_TIMERC_CTRL &= ~SET_TMXIE
#define TIMERD_INT_ENABLE       R_TIMERD_CTRL |= SET_TMXIE
#define TIMERD_INT_DISABLE      R_TIMERD_CTRL &= ~SET_TMXIE
#define TIMERE_INT_ENABLE       R_TIMERE_CTRL |= SET_TMXIE
#define TIMERE_INT_DISABLE      R_TIMERE_CTRL &= ~SET_TMXIE
#define TIMERF_INT_ENABLE       R_TIMERF_CTRL |= SET_TMXIE
#define TIMERF_INT_DISABLE      R_TIMERF_CTRL &= ~SET_TMXIE
#define TIMERG_INT_ENABLE       R_TIMERG_CTRL |= SET_TMXIE
#define TIMERG_INT_DISABLE      R_TIMERG_CTRL &= ~SET_TMXIE
#define TIMERH_INT_ENABLE       R_TIMERH_CTRL |= SET_TMXIE
#define TIMERH_INT_DISABLE      R_TIMERH_CTRL &= ~SET_TMXIE

/*TIME BASE MASK definition*/
#define SET_TMBXEN               0x00002000

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
// Each Tiny count time is 2.666... us
#define TINY_WHILE(BREAK_CONDITION,TIMEOUT_TINY_COUNT)  \
{\
   INT32U ttt1 = tiny_counter_count_get();\
   INT32U ttt2;\
   INT32U dt=0;\
   while(!((BREAK_CONDITION) || (dt>TIMEOUT_TINY_COUNT)))\
   {\
        ttt2 = tiny_counter_count_get(); \
        if (ttt2 >= ttt1) \
            dt = ttt2 - ttt1; \
        else \
            dt = ((INT32U)0x0000FFFF - ttt1) + ttt2;\
   }\
}

/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
/* prototype function define*/
static void local_timer_isr0(void);
static void local_timer_isr1(void);
static void local_timer_isr2(void);
static void local_timer_isr3(void);
static void local_timer_isr4(void);
static void local_timer_isr5(void);
static void local_timer_isr6(void);
static void local_timer_isr7(void);

static void local_timer_times_isr0(void);
static void local_timer_times_isr1(void);
static void local_timer_times_isr2(void);
static void local_timer_times_isr3(void);
static void local_timer_times_isr4(void);
static void local_timer_times_isr5(void);
static void local_timer_times_isr6(void);
static void local_timer_times_isr7(void);

static void local_time_base_isr0(void);

/* Timer static prototype */
static INT32S timer_freq_normal_setup(INT32U timer_id, INT32U freq_hz, void(* TimerIsrFunction)(void));
static INT32S timer_freq_times_setup(INT32U timer_id, INT32U freq_hz, INT32U times_counter, void(* TimerIsrFunction)(void));
static INT32S timer_msec_normal_setup(INT32U timer_id, INT32U msec, void(* TimerIsrFunction)(void));
INT32S timer_msec_times_setup(INT32U timer_id, INT32U msec, INT32U times_counter, void(* TimerIsrFunction)(void));
static INT32S timer_control(INT32U TIMER_ID, INT32U SRCASEL, INT32U SRCBSEL, INT32U EXTBSEL, INT32U EXTASEL, INT32U TMXEN, INT32U TMXIE, INT32U TMXIF);

static INT32S time_base_control(INT32U TIME_BASE_ID, TMBXS_ENUM TMBXS, INT32U TMBXEN, INT32U TMBXIE, INT32U TMBXIF);
static INT32S time_base_normal_setup(INT32U time_base_id, TMBXS_ENUM tmbxs_enum_hz, void(* TimerIsrFunction)(void));
static INT32S time_base_times_setup(INT32U time_base_id, TMBXS_ENUM tmbxs_enum_hz, INT32U times_counter, void(* TimeBaseIsrFunction)(void));

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
/* static globle valu */
static void (*gloable_timer_isr[TIMER_ID_MAX])(void);  /* -2 is mean: Timer E and F do not have isr */
static void (*gloable_time_base_isr[TIME_BASE_ID_MAX])(void);

/* limit isr times trap for timer */
static INT32U trap_A=0;
static INT32U trap_counter_A=0;
static INT32U trap_B=0;
static INT32U trap_counter_B=0;
static INT32U trap_C=0;
static INT32U trap_counter_C=0;
static INT32U trap_D=0;
static INT32U trap_counter_D=0;
static INT32U trap_E=0;
static INT32U trap_counter_E=0;
static INT32U trap_F=0;
static INT32U trap_counter_F=0;
static INT32U trap_G=0;
static INT32U trap_counter_G=0;
static INT32U trap_H=0;
static INT32U trap_counter_H=0;
/* limit isr times trap for time base */
static INT32U timebase_trap[3]={0};
static INT32U timebase_trap_counter[3]={0};
static INT32U time_base_isr_avoid[3]={TIME_BASE_FALSE};
static INT32U time_base_times_isr_avoid[3]={TIME_BASE_FALSE};
static INT8U timer_setup_lock[TIMER_ID_MAX]={0};


void timer_init(void)
{
    INT16U i;

    for (i=0; i<TIMER_ID_MAX-2; i++)
    {
        gloable_timer_isr[i]=0;
    }
    for (i=0; i<TIME_BASE_ID_MAX; i++)
    {
        gloable_time_base_isr[i]=0;
    }
}

/*******************************************************************************
* timer_freq_setup - timer frequency setup
* Author/Date: Dominant Yang, 10/04/2007
* DESCRIPTION: Set up the timer and it's timer isr
* INPUT: timer_id : 0~3 (TIMER_A, TIMER_B, TIMER_C)
*       freq_hz : each isr cycle time (Hz)
*       times_counter : 0 --> isr happen forever until timer stop
*                       1~4294967295  --> loop times
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S timer_freq_setup(INT32U timer_id, INT32U freq_hz, INT32U times_counter, void (*TimerIsrFunction)(void))
{
    if (timer_setup_lock[timer_id]==1) {return TIMER_FAIL;}

    if (times_counter == 0)
    {
        if ( timer_freq_normal_setup(timer_id, freq_hz, TimerIsrFunction) == TIMER_FAIL) {return TIMER_FAIL;}
    }
    else
    {
        if ( timer_freq_times_setup(timer_id, freq_hz, times_counter, TimerIsrFunction) == TIMER_FAIL) {return TIMER_FAIL;}
    }

    return TIMER_OK;
}

INT32S timer_freq_setup_lock(TIMER_ID_ENUM timer_id)
{
    timer_setup_lock[timer_id]=1;
	return STATUS_OK;
}

INT32S timer_freq_setup_unlock(TIMER_ID_ENUM timer_id)
{
    timer_setup_lock[timer_id]=0;
    return STATUS_OK;
}

INT32S timer_freq_setup_status_get(TIMER_ID_ENUM timer_id)
{
    return timer_setup_lock[timer_id];
}

//ggg test ext clock source

INT32S timer_ext_freq_setup(INT32U timer_id, INT32U ext_clk, INT32U freq_hz, void (*TimerIsrFunction)(void))
{
    switch (timer_id) {
    case TIMER_A:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM0_IRQn);
        //vic_irq_register(VIC_TIMER0, local_timer_isr0);
        //vic_irq_enable(VIC_TIMER0);
        break;
    case TIMER_B:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM1_IRQn);
        //vic_irq_register(VIC_TIMER1, local_timer_isr1);
        //vic_irq_enable(VIC_TIMER1);
        break;
    case TIMER_C:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM2_IRQn);
        //vic_irq_register(VIC_TIMER2, local_timer_isr2);
        //vic_irq_enable(VIC_TIMER2);
        break;
    case TIMER_D:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM3_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_E:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM4_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_F:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM5_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_G:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM6_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_H:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM7_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    default:
        return TIMER_FAIL;
        break;
    }
    #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
      TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(ext_clk/freq_hz)));
    #else
      TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(ext_clk/freq_hz)+1));
    #endif
      DRV_WriteReg32(TIMER_BASE_ADDR+EACH_REG_OFFSET*8*timer_id, 0x1000);
      timer_control(timer_id,SRCA_EXTA_WITH_PRESCALE,SRCB_256Hz_SELECT,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);

    return TIMER_OK;

}


INT32S timer_freq_normal_setup(INT32U timer_id, INT32U freq_hz, void (*TimerIsrFunction)(void))
{
    switch (timer_id) {
    case TIMER_A:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM0_IRQn, 5);
        NVIC_EnableIRQ(TIM0_IRQn);
        //vic_irq_register(VIC_TIMER0, local_timer_isr0);
        //vic_irq_enable(VIC_TIMER0);
        break;
    case TIMER_B:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM1_IRQn, 5);
        NVIC_EnableIRQ(TIM1_IRQn);
        //vic_irq_register(VIC_TIMER1, local_timer_isr1);
        //vic_irq_enable(VIC_TIMER1);
        break;
    case TIMER_C:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM2_IRQn, 5);
        NVIC_EnableIRQ(TIM2_IRQn);
        //vic_irq_register(VIC_TIMER2, local_timer_isr2);
        //vic_irq_enable(VIC_TIMER2);
        break;
    case TIMER_D:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM3_IRQn, 5);
        NVIC_EnableIRQ(TIM3_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_E:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM4_IRQn, 5);
        NVIC_EnableIRQ(TIM4_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_F:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM5_IRQn, 5);
        NVIC_EnableIRQ(TIM5_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_G:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM6_IRQn, 5);
        NVIC_EnableIRQ(TIM6_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_H:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM7_IRQn, 5);
        NVIC_EnableIRQ(TIM7_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    default:
        return TIMER_FAIL;
        break;
    }
    if (freq_hz > (SystemCoreClock/2))
	{
        return TIMER_FAIL;
    }
    else if (freq_hz > ((SystemCoreClock/2)/0x10000))
	{
        #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
	    TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(SystemCoreClock/2/freq_hz)));
        #else
        TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(SystemCoreClock/2/freq_hz)+1));
        DRV_WriteReg32(TIMER_BASE_ADDR+EACH_REG_OFFSET*8*timer_id, 0x1000);

        #endif
        //DRV_Reg32(TIMERA_PRELOAD_ADDR+EACH_REG_OFFSET*8*timer_id) = (0x10000-(SystemCoreClock/2/freq_hz));
	    timer_control(timer_id,SRCA_SYSCLK_DIV2_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);
    }
    else if(freq_hz > ((SystemCoreClock/256)/0x10000))/* freq > 2Hz */
    {
        #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
        TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(SystemCoreClock/256/freq_hz)));
        #else
        TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(SystemCoreClock/256/freq_hz)+1));
        #endif
        //DRV_Reg32(TIMERA_PRELOAD_ADDR+EACH_REG_OFFSET*8*timer_id) = (0x10000-(SystemCoreClock/2/freq_hz));
	    timer_control(timer_id,SRCA_SYSCLK_DIV256_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	            TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);
    }
    else
    {
        return TIMER_FAIL;
    }

    return TIMER_OK;

}

/* This API can setup the interrupt times */
INT32S timer_freq_times_setup(INT32U timer_id, INT32U freq_hz, INT32U times_counter,void (*TimerIsrFunction)(void))
{
    switch (timer_id) {
    case TIMER_A:
        trap_A=0;
        trap_counter_A=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM0_IRQn, 5);
        NVIC_EnableIRQ(TIM0_IRQn);
        //vic_irq_register(VIC_TIMER0, local_timer_times_isr0);
        //vic_irq_enable(VIC_TIMER0);
        break;
    case TIMER_B:
        trap_B=0;
        trap_counter_B=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM1_IRQn, 5);
        NVIC_EnableIRQ(TIM1_IRQn);
        //vic_irq_register(VIC_TIMER1, local_timer_times_isr1);
        //vic_irq_enable(VIC_TIMER1);
        break;
    case TIMER_C:
        trap_C=0;
        trap_counter_C=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM2_IRQn, 5);
        NVIC_EnableIRQ(TIM2_IRQn);
        //vic_irq_register(VIC_TIMER2, local_timer_times_isr2);
        //vic_irq_enable(VIC_TIMER2);
        break;
    case TIMER_D:
        trap_D=0;
        trap_counter_D=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM3_IRQn, 5);
        NVIC_EnableIRQ(TIM3_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_times_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_E:
        trap_E=0;
        trap_counter_E=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM4_IRQn, 5);
        NVIC_EnableIRQ(TIM4_IRQn);
        break;
    case TIMER_F:
        trap_F=0;
        trap_counter_F=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM5_IRQn, 5);
        NVIC_EnableIRQ(TIM5_IRQn);
        break;
    case TIMER_G:
        trap_G=0;
        trap_counter_G=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM6_IRQn, 5);
        NVIC_EnableIRQ(TIM6_IRQn);
        break;
    case TIMER_H:
        trap_H=0;
        trap_counter_H=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM7_IRQn, 5);
        NVIC_EnableIRQ(TIM7_IRQn);
        break;
    default:
        return TIMER_FAIL;
        break;
    }

	//count_start = (0x10000-(SystemCoreClock/2/freq_hz));
	if (freq_hz > (SystemCoreClock/2))
	{
        return TIMER_FAIL;
    }
	else if (freq_hz > ((SystemCoreClock/2)/0x10000))
	{
	  #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
	    TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(SystemCoreClock/2/freq_hz)));
      #else
        TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(SystemCoreClock/2/freq_hz)+1));
      #endif

	    timer_control(timer_id,SRCA_SYSCLK_DIV2_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	            TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);
	}
    else  if(freq_hz > ((SystemCoreClock/256)/0x10000))/* freq > 2Hz */
    {
      #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
        TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(SystemCoreClock/256/freq_hz)));
      #else
        TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(SystemCoreClock/2/freq_hz))+1);
      #endif

	    timer_control(timer_id,SRCA_SYSCLK_DIV256_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	            TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);
    }
    else
    {
        return TIMER_FAIL;
    }
    return TIMER_OK;

}

/*******************************************************************************
* timerD_counter_init - SYSTEM 2.66us Counter timer (TIMER D) initial
* Author/Date: Dominant Yang, 12/01/2007
* DESCRIPTION: Set up the timer
* INPUT:
*
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S timerD_counter_init(void)
{
	TIMER_PRELOAD_COUNTER_SET(TIMER_D,0x0000);
	timer_control(TIMER_D,SRCA_SYSCLK_DIV256_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
    return TIMER_OK;
}

INT32U tiny_counter_count_get(void)
{
    INT32U ret_counter = R_TIMERD_UPCOUNT & 0x0000FFFF;
    return ret_counter;
}

INT32U tiny_counter_get(void)   /* each tiny counter == 2.66us*/
{
    INT32U ret_counter = R_TIMERD_UPCOUNT;

    switch (MHZ)
	{
		case 24:
			ret_counter=(ret_counter<<2);
			break;
		case 48:
			ret_counter=(ret_counter<<1);
			break;
		case 78:
			ret_counter=(ret_counter*96/78);
			break;
		case 144:
			ret_counter=(ret_counter*96/144);
			break;
		default:
			ret_counter=(ret_counter*96/MHZ);
	}

    return ret_counter;
}


/*******************************************************************************
* dac_timer_freq_setup - DAC timer (TIMER E) frequency setup
* Author/Date: Dominant Yang, 10/04/2007
* DESCRIPTION: Set up the timer and it's timer isr
* INPUT:
*       freq_hz : each isr cycle time (Hz)
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S dac_timer_freq_setup(INT32U freq_hz)
{
  #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
	TIMER_PRELOAD_COUNTER_SET(TIMER_E,(0x10000-(SystemCoreClock/2/freq_hz)));
  #else
    TIMER_PRELOAD_COUNTER_SET(TIMER_E,(0xFFFFFFFF-(SystemCoreClock/2/freq_hz)+1));
  #endif
	timer_control(TIMER_E,SRCA_SYSCLK_DIV2_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
    return TIMER_OK;
}

/*******************************************************************************
* adc_timer_freq_setup - ADC timer (TIMER C,D,E,F) frequency setup
* Author/Date: Dominant Yang, 10/04/2007
* DESCRIPTION: Set up the timer and it's timer isr
* INPUT:
*       freq_hz : each isr cycle time (Hz)
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S adc_timer_freq_setup(INT32U timer_id, INT32U freq_hz)
{
  #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
	TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(SystemCoreClock/2/freq_hz)));
  #else
    TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(SystemCoreClock/2/freq_hz)+1));
  #endif
	timer_control(timer_id,SRCA_SYSCLK_DIV2_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
    return TIMER_OK;
}

/*******************************************************************************
* timer_msec_setup - timer msec setup
* Author/Date: Dominant Yang, 10/04/2007
* DESCRIPTION: Set up the timer and it's timer isr
* INPUT: timer_id : 0~3 (TIMER_A, TIMER_B, TIMER_C)
*       msec : each isr cycle time (mSec)
*       times_counter : 0 --> isr happen forever until timer stop
*                       1~4294967295  --> loop times
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S timer_msec_setup(INT32U timer_id, INT32U msec, INT32U times_counter,void (*TimerIsrFunction)(void))
{
    if (times_counter == 0)
    {
        if ( timer_msec_normal_setup(timer_id, msec, TimerIsrFunction) == TIMER_FAIL) {return TIMER_FAIL;}
    }
    else
    {
        if ( timer_msec_times_setup(timer_id, msec, times_counter, TimerIsrFunction) == TIMER_FAIL) {return TIMER_FAIL;}
    }

    return TIMER_OK;
}

INT32S timer_msec_normal_setup(INT32U timer_id, INT32U msec, void (*TimerIsrFunction)(void))
{
    switch (timer_id) {
	case TIMER_A:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM0_IRQn, 5);
        NVIC_EnableIRQ(TIM0_IRQn);
        //vic_irq_register(VIC_TIMER0, local_timer_isr0);
        //vic_irq_enable(VIC_TIMER0);
        break;
	case TIMER_B:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM1_IRQn, 5);
        NVIC_EnableIRQ(TIM1_IRQn);
        //vic_irq_register(VIC_TIMER1, local_timer_isr1);
        //vic_irq_enable(VIC_TIMER1);
        break;
	case TIMER_C:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM2_IRQn, 5);
        NVIC_EnableIRQ(TIM2_IRQn);
        //vic_irq_register(VIC_TIMER2, local_timer_isr2);
        //vic_irq_enable(VIC_TIMER2);
        break;
	case TIMER_D:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM3_IRQn, 5);
        NVIC_EnableIRQ(TIM3_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
	case TIMER_E:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM4_IRQn, 5);
        NVIC_EnableIRQ(TIM4_IRQn);
        break;
	case TIMER_F:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM5_IRQn, 5);
        NVIC_EnableIRQ(TIM5_IRQn);
        break;
	case TIMER_G:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM6_IRQn, 5);
        NVIC_EnableIRQ(TIM6_IRQn);
        break;
	case TIMER_H:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM7_IRQn, 5);
        NVIC_EnableIRQ(TIM7_IRQn);
        break;

    default:
        return TIMER_FAIL;
        break;
	}

  #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
    TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(8192*msec/1000)));  //count_start;
  #else
    TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(8192*msec/1000)+1));  //count_start;
  #endif
    //DRV_Reg32(TIMERA_PRELOAD_ADDR+EACH_REG_OFFSET*8*timer_id) = (0x10000-(32768*msec/1000)); //count_start;
    timer_control(timer_id,SRCA_8192Hz_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
                    TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);

    return TIMER_OK;
}

INT32S timer_msec_times_setup(INT32U timer_id, INT32U msec, INT32U times_counter, void (*TimerIsrFunction)(void))
{
    switch (timer_id) {
    case TIMER_A:
        trap_A=0;
        trap_counter_A=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM0_IRQn, 5);
        NVIC_EnableIRQ(TIM0_IRQn);
        //vic_irq_register(VIC_TIMER0, local_timer_times_isr0);
        //vic_irq_enable(VIC_TIMER0);
        break;
    case TIMER_B:
        trap_B=0;
        trap_counter_B=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM1_IRQn, 5);
        NVIC_EnableIRQ(TIM1_IRQn);
        //vic_irq_register(VIC_TIMER1, local_timer_times_isr1);
        //vic_irq_enable(VIC_TIMER1);
        break;
    case TIMER_C:
        trap_C=0;
        trap_counter_C=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM2_IRQn, 5);
        NVIC_EnableIRQ(TIM2_IRQn);
        //vic_irq_register(VIC_TIMER2, local_timer_times_isr2);
        //vic_irq_enable(VIC_TIMER2);
        break;
    case TIMER_D:
        trap_D=0;
        trap_counter_D=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM3_IRQn, 5);
        NVIC_EnableIRQ(TIM3_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_times_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_E:
        trap_E=0;
        trap_counter_E=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM4_IRQn, 5);
        NVIC_EnableIRQ(TIM4_IRQn);
        break;
    case TIMER_F:
        trap_F=0;
        trap_counter_F=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM5_IRQn, 5);
        NVIC_EnableIRQ(TIM5_IRQn);
        break;
    case TIMER_G:
        trap_G=0;
        trap_counter_G=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM6_IRQn, 5);
        NVIC_EnableIRQ(TIM6_IRQn);
        break;
    case TIMER_H:
        trap_H=0;
        trap_counter_H=times_counter;
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_SetPriority(TIM7_IRQn, 5);
        NVIC_EnableIRQ(TIM7_IRQn);
        break;

    default:
        return TIMER_FAIL;
        break;
    }

  #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
    TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(4096*msec/1000)));  //count_start;
  #else
    TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(4096*msec/1000)+1));  //count_start;
  #endif
    timer_control(timer_id,SRCA_4096Hz_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
                    TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);
    return TIMER_OK;

}

INT32U TimerCountReturn(INT32U Timer_Id)
{
    return DRV_Reg32(TIMERA_UPCOUNT_ADDR+EACH_REG_OFFSET*8*Timer_Id);
}

void TimerCountSet(INT32U Timer_Id, INT32U set_count)
{
    TIMER_PRELOAD_COUNTER_SET(Timer_Id, set_count);
}


void local_timer_isr0(void)
{
    R_TIMERA_CTRL = (R_TIMERA_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_timer_isr[TIMER_A] != 0) {
        (*gloable_timer_isr[TIMER_A])();
    }
}

void local_timer_isr1(void)
{
    R_TIMERB_CTRL = (R_TIMERB_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERB Interrupt flag*/
    if (gloable_timer_isr[TIMER_B] != 0) {
        (*gloable_timer_isr[TIMER_B])();
    }
}

void local_timer_isr2(void)
{
    R_TIMERC_CTRL = (R_TIMERC_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERC Interrupt flag*/
    if (gloable_timer_isr[TIMER_C] != 0) {
        (*gloable_timer_isr[TIMER_C])();
    }
}

void local_timer_isr3(void)
{
    R_TIMERD_CTRL = (R_TIMERD_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERD Interrupt flag*/
    if (gloable_timer_isr[TIMER_D] != 0) {
        (*gloable_timer_isr[TIMER_D])();
    }
}

void local_timer_isr4(void)
{
    R_TIMERE_CTRL = (R_TIMERE_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERD Interrupt flag*/
    if (gloable_timer_isr[TIMER_E] != 0) {
        (*gloable_timer_isr[TIMER_E])();
    }
}

void local_timer_isr5(void)
{
    R_TIMERF_CTRL = (R_TIMERF_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERD Interrupt flag*/
    if (gloable_timer_isr[TIMER_F] != 0) {
        (*gloable_timer_isr[TIMER_F])();
    }
}

void local_timer_isr6(void)
{
    R_TIMERG_CTRL = (R_TIMERG_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERD Interrupt flag*/
    if (gloable_timer_isr[TIMER_G] != 0) {
        (*gloable_timer_isr[TIMER_G])();
    }
}

void local_timer_isr7(void)
{
    R_TIMERH_CTRL = (R_TIMERH_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERD Interrupt flag*/
    if (gloable_timer_isr[TIMER_H] != 0) {
        (*gloable_timer_isr[TIMER_H])();
    }
}

void local_timer_times_isr0(void)
{

    R_TIMERA_CTRL = (R_TIMERA_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/

    if (gloable_timer_isr[TIMER_A] != 0) {
        if (trap_A < trap_counter_A)
        {
            (*gloable_timer_isr[TIMER_A])();
            trap_A++;
        }
        else
        {
            //vic_irq_disable(VIC_TIMER0);
            //vic_irq_unregister(VIC_TIMER0);
            NVIC_DisableIRQ(TIM0_IRQn);
            trap_A = 0;
            trap_counter_A = 0;
        }
    }
}

void local_timer_times_isr1(void)
{

    R_TIMERB_CTRL = (R_TIMERB_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/

    if (gloable_timer_isr[TIMER_B] != 0) {
        if (trap_B < trap_counter_B)
        {
            (*gloable_timer_isr[TIMER_B])();
            trap_B++;
        }
        else
        {
          //vic_irq_disable(VIC_TIMER1);
          //vic_irq_unregister(VIC_TIMER1);
          NVIC_DisableIRQ(TIM1_IRQn);
          trap_B = 0;
          trap_counter_B = 0;
        }
    }
}

void local_timer_times_isr2(void)
{

    R_TIMERC_CTRL = (R_TIMERC_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/

    if (gloable_timer_isr[TIMER_C] != 0) {
        if (trap_C < trap_counter_C)
        {
            (*gloable_timer_isr[TIMER_C])();
            trap_C++;
        }
        else
        {
          //vic_irq_disable(VIC_TIMER2);
          //vic_irq_unregister(VIC_TIMER2);
          NVIC_DisableIRQ(TIM2_IRQn);
          trap_C = 0;
          trap_counter_C = 0;
        }
    }
}

void local_timer_times_isr3(void)
{

    R_TIMERD_CTRL = (R_TIMERD_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_timer_isr[TIMER_D] != 0) {
        if (trap_D < trap_counter_D)
        {
            (*gloable_timer_isr[TIMER_D])();
            trap_D++;
        }
        else
        {
          //vic_irq_disable(VIC_TIMER3);
          //vic_irq_unregister(VIC_TIMER3);
          NVIC_DisableIRQ(TIM3_IRQn);
          trap_D = 0;
          trap_counter_D = 0;
        }
    }
}

void local_timer_times_isr4(void)
{

    R_TIMERE_CTRL = (R_TIMERE_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_timer_isr[TIMER_E] != 0) {
        if (trap_E < trap_counter_E)
        {
            (*gloable_timer_isr[TIMER_E])();
            trap_E++;
        }
        else
        {
            //vic_irq_disable(VIC_TIMER3);
            //vic_irq_unregister(VIC_TIMER3);
          NVIC_DisableIRQ(TIM4_IRQn);
          trap_E = 0;
          trap_counter_E = 0;
        }
    }
}

void local_timer_times_isr5(void)
{

    R_TIMERF_CTRL = (R_TIMERE_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_timer_isr[TIMER_F] != 0) {
        if (trap_F < trap_counter_F)
        {
            (*gloable_timer_isr[TIMER_F])();
            trap_F++;
        }
        else
        {
          //vic_irq_disable(VIC_TIMER3);
          //vic_irq_unregister(VIC_TIMER3);
          NVIC_DisableIRQ(TIM5_IRQn);
          trap_F = 0;
          trap_counter_F = 0;
        }
    }
}

void local_timer_times_isr6(void)
{

    R_TIMERG_CTRL = (R_TIMERG_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_timer_isr[TIMER_G] != 0) {
        if (trap_G < trap_counter_G)
        {
            (*gloable_timer_isr[TIMER_G])();
            trap_G++;
        }
        else
        {
          //vic_irq_disable(VIC_TIMER3);
          //vic_irq_unregister(VIC_TIMER3);
          NVIC_DisableIRQ(TIM6_IRQn);
          trap_G = 0;
          trap_counter_G = 0;
        }
    }
}

void local_timer_times_isr7(void)
{

    R_TIMERH_CTRL = (R_TIMERH_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_timer_isr[TIMER_H] != 0) {
        if (trap_F < trap_counter_H)
        {
            (*gloable_timer_isr[TIMER_H])();
            trap_H++;
        }
        else
        {
          //vic_irq_disable(VIC_TIMER3);
          //vic_irq_unregister(VIC_TIMER3);
          NVIC_DisableIRQ(TIM7_IRQn);
          trap_H = 0;
          trap_counter_H = 0;
        }
    }
}

void timer_irq_handeler(INT32U timerID)
{
  switch (timerID){
  case TIMER_A:
    local_timer_isr0();
    break;
  case TIMER_B:
    local_timer_isr1();
    break;
  case TIMER_C:
    local_timer_isr2();
    break;
  case TIMER_D:
    local_timer_isr3();
    break;
  case TIMER_E:
    local_timer_isr4();
    break;
  case TIMER_F:
    local_timer_isr5();
    break;
  case TIMER_G:
    local_timer_isr6();
    break;
  case TIMER_H:
     local_timer_isr7();
     break;
  default:
    break;
  }
}

void timer_times_irq_handeler(INT32U timerID)
{
  switch (timerID){
  case TIMER_A:
    local_timer_times_isr0();
    break;
  case TIMER_B:
    local_timer_times_isr1();
    break;
  case TIMER_C:
    local_timer_times_isr2();
    break;
  case TIMER_D:
    local_timer_times_isr3();
    break;
  case TIMER_E:
    local_timer_times_isr4();
    break;
  case TIMER_F:
    local_timer_times_isr5();
    break;
  case TIMER_G:
    local_timer_times_isr6();
    break;
  case TIMER_H:
     local_timer_times_isr7();
     break;
  default:
    break;
  }
}
/* timer_control is for internal use*/

INT32S timer_control(INT32U TIMER_ID, INT32U SRCASEL, INT32U SRCBSEL,
    INT32U EXTBSEL, INT32U EXTASEL, INT32U TMXEN, INT32U TMXIE, INT32U TMXIF)
{
    INT32U reg_value=DRV_Reg32(TIMER_BASE_ADDR+EACH_REG_OFFSET*8*TIMER_ID);
    reg_value &= ~(0xFFFF);
    reg_value |= (SRCASEL<<0 | SRCBSEL<<4 | EXTBSEL<<8 | EXTASEL<<10 | TMXEN <<13 | TMXIE<<14 | TMXIF<<15);

    if (TIMER_ID < TIMER_ID_MAX)  /* Timer id  =  {Timer A, Timer B , Timer C, Timer D} which included in Timer1 reg.*/
    {
        DRV_WriteReg32(TIMER_BASE_ADDR+EACH_REG_OFFSET*8*TIMER_ID, reg_value);
    }
    else
    {
        return TIMER_FAIL;
    }

    return TIMER_OK;
}

INT32S timer_ccp_ctrl(INT32U TIMER_ID, INT32U PWMXSEL, INT32U CMPXSEL, INT32U CAPXSEL, INT32U CCPXEN)
{
    INT32U reg_value=DRV_Reg32(TIMERA_CCP_CTRL_ADDR+EACH_REG_OFFSET*8*TIMER_ID);;

    reg_value &= ~(0xFFFF);
    if (TIMER_ID < TIMER_ID_MAX) {   /* only used for Timer A,B,C,D*/
        reg_value |= (PWMXSEL <<0 | CMPXSEL<<4 | CAPXSEL<<9 | CCPXEN<<14);
        DRV_Reg32(TIMERA_CCP_CTRL_ADDR+0x20*TIMER_ID) = reg_value;
        //DRV_WriteReg32(TIMERA_CCP_CTRL_ADDR+EACH_REG_OFFSET*5*TIMER_ID, reg_value);
    }
    else
    {
        return TIMER_FAIL;
    }

    return TIMER_OK;
}

/*******************************************************************************
* timer_input_filter_fs - Set Input Filter Sampling Period (A~H)
* DESCRIPTION: to set sampling period for an external input timer source
* INPUT:
*       timer_id : TIMER_A ~ TIMER_H (0~7)
*       sample_period : 0 ~ 255
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S timer_input_filter_fs(INT32U timer_id,INT32U sample_period)
{
    if (sample_period > 255)
        return TIMER_FAIL;

    if (timer_id < TIMER_ID_MAX) {
        DRV_Reg32(TIMERA_CTRL_ADDR+EACH_REG_OFFSET*8*timer_id) &= ~(0xFF << 24);
        DRV_Reg32(TIMERA_CTRL_ADDR+EACH_REG_OFFSET*8*timer_id) |= (sample_period << 24);
        return TIMER_TRUE;
    }
    else {
        return TIMER_FAIL;
    }
}

/*******************************************************************************
* timer_input_filter_cnt - Set Input Filter Count(A~H)
* DESCRIPTION: to set filter count for an external input timer source
* INPUT:
*       timer_id : TIMER_A ~ TIMER_H (0~7)
*       filter_cnt : 0 ~ 7 ( which  stands for 3 samples to 10 samples)
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S timer_input_filter_cnt(INT32U timer_id,INT32U filter_cnt)
{
    if (filter_cnt > TIMER_INPUT_FILTER_CNT_MAX)
        return TIMER_FAIL;

    if (timer_id < TIMER_ID_MAX) {
        DRV_Reg32(TIMERA_CTRL_ADDR+EACH_REG_OFFSET*8*timer_id) &= ~(0x7 << 21);
        DRV_Reg32(TIMERA_CTRL_ADDR+EACH_REG_OFFSET*8*timer_id) |= (filter_cnt << 21);
        return TIMER_TRUE;
    }
    else {
        return TIMER_FAIL;
    }
}


/*******************************************************************************
* timer_stop - Stop Time (A~H)
* Author/Date: Dominant Yang, 10/04/2007
* DESCRIPTION: used to stop the time
* INPUT:
*       timer_id : TIMER_A ~ TIMER_H (0~7)
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S timer_stop(INT32U timer_id)
{
    if (timer_id < TIMER_ID_MAX) {
        DRV_Reg32(TIMERA_CTRL_ADDR+EACH_REG_OFFSET*8*timer_id) &= ~(SET_TMXIE | SET_TMXEN);
        return TIMER_TRUE;
    }
    else {
        return TIMER_FAIL;
    }
}

void timer_counter_reset(INT32U timer_id)
{
    TIMER_PRELOAD_COUNTER_SET(timer_id,0);
}

/* Start the timer counter */
INT32S timer_start_without_isr(INT32U timer_id, TIMER_SOURCE_ENUM timer_source)
{
    if (timer_id < TIMER_ID_MAX) {   /* only used for Timer A,B,C,D*/
        switch (timer_source)
        {
            case TIMER_SOURCE_256HZ:
                timer_control(timer_id,SRCA_1_,SRCB_256Hz_SELECT,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            case TIMER_SOURCE_1024HZ:
                timer_control(timer_id,SRCA_1_,SRCB_1024Hz_SELECT,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            case TIMER_SOURCE_2048HZ:
                timer_control(timer_id,SRCA_1_,SRCB_2048Hz_SELECT,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            case TIMER_SOURCE_4096HZ:
                timer_control(timer_id,SRCA_4096Hz_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            case TIMER_SOURCE_8192HZ:
                timer_control(timer_id,SRCA_8192Hz_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            case TIMER_SOURCE_32768HZ:
                timer_control(timer_id,SRCA_4096Hz_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            case TIMER_SOURCE_MCLK_DIV_256:
                timer_control(timer_id,SRCA_SYSCLK_DIV256_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            case TIMER_SOURCE_MCLK_DIV_2:
                timer_control(timer_id,SRCA_SYSCLK_DIV2_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
                break;
            default:
            	return TIMER_FAIL;
        }
        return TIMER_TRUE;
    }
    else {
        return TIMER_FAIL;
    }
}




INT32S timer_pwm_setup(INT32U timer_id, INT32U freq_hz, INT8U pwm_duty_cycle_percent, PWMXSEL_ENUM NRO_NRZ)
{
    INT32U preload_value;
    INT32U ccp_reg_value;

    if (pwm_duty_cycle_percent>100)
    {
        return TIMER_FAIL;
    }

    if (freq_hz > (SystemCoreClock/2))
	{
        return TIMER_FAIL;
    }
    else if (freq_hz > ((SystemCoreClock/2)/0x10000))
	{
	  #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
	    preload_value = (0x10000-(SystemCoreClock/2/freq_hz));
        ccp_reg_value = preload_value+(pwm_duty_cycle_percent*(0x10000-preload_value)/100)-1;
      #else
        preload_value = (0xFFFFFFFF-(SystemCoreClock/2/freq_hz)+1);
        ccp_reg_value = preload_value+(pwm_duty_cycle_percent*(0xFFFFFFFF-preload_value)/100);
      #endif

	    TIMER_PRELOAD_COUNTER_SET(timer_id,preload_value);
        TIMER_CCP_REG_SET(timer_id, ccp_reg_value);
	    timer_control(timer_id,SRCA_SYSCLK_DIV2_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
        timer_ccp_ctrl(timer_id, NRO_NRZ, 0, 0, PWM_ENABLED);
    }
    else if(freq_hz > ((SystemCoreClock/256)/0x10000))/* freq > 2Hz */
    {
      #if TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
        preload_value = (0x10000-(SystemCoreClock/256/freq_hz));
        ccp_reg_value = preload_value+(pwm_duty_cycle_percent*(0x10000-preload_value)/100)-1;
      #else
        preload_value = (0xFFFFFFFF-(SystemCoreClock/256/freq_hz)+1);
        ccp_reg_value = preload_value+(pwm_duty_cycle_percent*(0xFFFFFFFF-preload_value)/100);
      #endif

        TIMER_PRELOAD_COUNTER_SET(timer_id,preload_value);
        TIMER_CCP_REG_SET(timer_id, ccp_reg_value);
	    timer_control(timer_id,SRCA_SYSCLK_DIV256_SELECT,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	            TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
        timer_ccp_ctrl(timer_id, NRO_NRZ, 0, 0, PWM_ENABLED);
    }
    else
    {
        return TIMER_FAIL;
    }

    return TIMER_OK;
}


INT32S timer_bam_setup(INT32U timer_id, INT32U SRCASEL, INT32U SRCBSEL, INT8U bam_duty_percent)
{
    INT32U preload_value;
    INT32U ccp_reg_value=0;
    INT32U bam_value;
    INT32U i,temp;

    if (bam_duty_percent>100)
    {
        return TIMER_FAIL;
    }

    if (timer_id < TIMER_ID_MAX)  /* Timer id  =  {Timer A, Timer B , Timer C, Timer D} which included in Timer1 reg.*/
	{
	    preload_value = 0xFF00;//(0x10000-(SystemCoreClock/2/freq_hz));

        bam_value = 256*bam_duty_percent/100;
        for(i=7;i>0;i--)
        {
        	temp = bam_value / (1<<i);
        	bam_value = bam_value % (1<<i);
        	if(temp)
        		ccp_reg_value |= (1<<i);
        }


        ccp_reg_value |= preload_value;

	    TIMER_PRELOAD_COUNTER_SET(timer_id,preload_value);
        TIMER_CCP_REG_SET(timer_id, ccp_reg_value);
	    timer_control(timer_id,SRCASEL,SRCBSEL,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                TIMER_ENABLE, TIMER_INT_DISABLE, TIMER_INT_FLAG_FALSE);
        timer_ccp_ctrl(timer_id, BAM_OUTPUT, 0, 0, PWM_ENABLED);
    }
    else
    {
        return TIMER_FAIL;
    }

    return TIMER_OK;
}


INT32S timer_16bitcmp_setup(INT32U timer_id, INT32U SRCASEL, INT32U SRCBSEL, INT32U cmp1_value, INT32U cmp2_value)
{
    INT32U preload_value;
    INT32U ccp_reg_value=0;
    INT32U bam_value;
    INT32U i,temp;


    if (timer_id < TIMER_ID_MAX)  /* Timer id  =  {Timer A, Timer B , Timer C, Timer D} which included in Timer1 reg.*/
    {
	NVIC_EnableIRQ(TIM0_IRQn);
      preload_value = 0x0000;//(0x10000-(SystemCoreClock/2/freq_hz));

      ccp_reg_value = cmp1_value | cmp2_value<<16;
      TIMER_PRELOAD_COUNTER_SET(timer_id,preload_value);
      //TIMER_CCP_REG_SET(timer_id, ccp_reg_value);
      DRV_WriteReg32(TIMERA_CCP_REG_ADDR+EACH_REG_OFFSET*8*timer_id, ccp_reg_value);
      ccp_reg_value = (cmp1_value | cmp2_value<<16)<<1;
      TIMER_CMPLOAD_REG_SET(timer_id, ccp_reg_value);
      timer_control(timer_id,SRCASEL,SRCBSEL,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
          TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);

      DRV_WriteReg32(TIMERA_CCP_CTRL_ADDR+EACH_REG_OFFSET*8*timer_id, 0x00a89000);
      timer_ccp_ctrl(timer_id, 0, CMP_HIGH_PULSE_ON_CCP, 0, COMPARISON_ENABLED);
    }
    else
    {
        return TIMER_FAIL;
    }

    return TIMER_OK;
}


/* Time Base */
INT32S time_base_control(INT32U TIME_BASE_ID, TMBXS_ENUM TMBXS, INT32U TMBXEN, INT32U TMBXIE, INT32U TMBXIF)
{
    INT32U reg_value=0;

    reg_value |= (TMBXS<<0 | TMBXEN<<13 | TMBXIE<<14 | TMBXIF<<15);

    if (TIME_BASE_ID < TIME_BASE_ID_MAX)  /* Timer id  =  {Timer A, Timer B , Timer C, Timer D} which included in Timer1 reg.*/
    {
        DRV_WriteReg32(TIMEBASE_BASE_ADDR+EACH_REG_OFFSET*TIME_BASE_ID, reg_value);
    }
    else
    {
        return TIME_BASE_FAIL;
    }

    return TIME_BASE_OK;
}

/*******************************************************************************
* time_base_setup - time base setup
* Author/Date: Dominant Yang, 10/04/2007
* DESCRIPTION: Time base can wake up CPU when from suspend/sleep mode, but the
* time source is fixed, so we use the enum num to hint the cycle time
* INPUT: time_base_id : 0~3 (TIME_BASE_A, TIME_BASE_B, TIME_BASE_C, TIME_BASE_D)
*       tmbxs_enum_hz : 	TMBAS_1HZ=1,
*	                        TMBAS_2HZ,
*	                        TMBAS_4HZ,
*	                        TMBBS_8HZ=0,
*	                        TMBBS_16HZ,
*	                        TMBBS_32HZ,
*	                        TMBBS_64HZ,
*	                        TMBCS_128HZ=0,
*	                        TMBCS_256HZ,
*	                        TMBCS_512HZ,
*	                        TMBCS_1024HZ
*       times_counter : 0 --> isr happen forever until timer stop
*                       1~4294967295  --> loop times
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S time_base_setup(INT32U time_base_id, TMBXS_ENUM tmbxs_enum_hz, INT32U times_counter, void (*TimeBaseIsrFunction)(void))
{
    if (times_counter == 0)
    {
        time_base_normal_setup(time_base_id, tmbxs_enum_hz, TimeBaseIsrFunction);
    }
    else
    {
        time_base_times_setup(time_base_id, tmbxs_enum_hz,times_counter, TimeBaseIsrFunction);
    }
    return TIME_BASE_OK;
}

INT32S time_base_normal_setup(INT32U time_base_id, TMBXS_ENUM tmbxs_enum_hz, void (*TimerIsrFunction)(void))
{

    if (time_base_id >= TIME_BASE_ID_MAX) {return TIME_BASE_FAIL;} /* Timer id  =  {Timer A, Timer B , Timer C, Timer D} which included in Timer1 reg.*/

#if _OPERATING_SYSTEM == 1
	OSSchedLock();
#endif
    gloable_time_base_isr[time_base_id] = TimerIsrFunction;
    time_base_isr_avoid[time_base_id] = TIME_BASE_FALSE;
    time_base_times_isr_avoid[time_base_id] = TIME_BASE_TRUE;  /*avoid times isr*/
#if _OPERATING_SYSTEM == 1
	OSSchedUnlock();
#endif

    //vic_irq_register(VIC_TMB_ABC, local_time_base_isr0);
    //vic_irq_enable(VIC_TMB_ABC);
	time_base_control(time_base_id,tmbxs_enum_hz,TMBXEN_ENABLE,TMBXIE_ENABLE,TMBXIF_ENABLE);

    return TIME_BASE_OK;

}

INT32S time_base_times_setup(INT32U time_base_id, TMBXS_ENUM tmbxs_enum_hz, INT32U times_counter, void (*TimeBaseIsrFunction)(void))
{
    if (time_base_id >= TIME_BASE_ID_MAX) {return TIME_BASE_FAIL;} /* Timer id  =  {Timer A, Timer B , Timer C, Timer D} which included in Timer1 reg.*/
#if _OPERATING_SYSTEM == 1
	OSSchedLock();
#endif
        timebase_trap[time_base_id]=0;
        timebase_trap_counter[time_base_id]=times_counter;
        gloable_time_base_isr[time_base_id] = TimeBaseIsrFunction;
        time_base_times_isr_avoid[time_base_id] = TIME_BASE_FALSE;
        time_base_isr_avoid[time_base_id] = TIME_BASE_TRUE;   /*avoid nomal isr*/
#if _OPERATING_SYSTEM == 1
	OSSchedUnlock();
#endif
        //vic_irq_register(VIC_TMB_ABC, local_time_base_isr0);
        //vic_irq_enable(VIC_TMB_ABC);


	time_base_control(time_base_id,tmbxs_enum_hz,TMBXEN_ENABLE,TMBXIE_ENABLE,TMBXIF_ENABLE);

    return TIME_BASE_OK;
}

/*******************************************************************************
* stop_time_base - Stop Time base (A~C)
* Author/Date: Dominant Yang, 10/04/2007
* DESCRIPTION: used to stop time base
* INPUT:
*       timer_id : TIME_BASE_A ~ TIME_BASE_D (0~3)
* OUTPUT: none
* RETURN: TIMER_OK/TIMER_FAIL.
*******************************************************************************/
INT32S time_base_stop(INT32U time_base_id)
{
    if (time_base_id < TIME_BASE_ID_MAX) {   /* only used for Timer A,B,C*/
        DRV_Reg32(TIMEBASE_BASE_ADDR+EACH_REG_OFFSET*time_base_id) &= ~(SET_TMBXEN);
        return TIME_BASE_OK;
    }
    else {
        return TIME_BASE_FAIL;
    }
}


/* time base isr */
void local_time_base_isr0(void)
{
  if (((R_TIMEBASEA_CTRL & TIMER_INT_FLAG_MASK) == TIMER_INT_FLAG_MASK) && (time_base_isr_avoid[TIME_BASE_A] != TIME_BASE_TRUE))
  {
    R_TIMEBASEA_CTRL = (R_TIMEBASEA_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_time_base_isr[TIME_BASE_A] != 0) {
        (*gloable_time_base_isr[TIME_BASE_A])();
    }
  }

  if (((R_TIMEBASEB_CTRL & TIMER_INT_FLAG_MASK) == TIMER_INT_FLAG_MASK) && (time_base_isr_avoid[TIME_BASE_B] != TIME_BASE_TRUE))
  {
    R_TIMEBASEB_CTRL = (R_TIMEBASEB_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERB Interrupt flag*/
    if (gloable_time_base_isr[TIME_BASE_B] != 0) {
        (*gloable_time_base_isr[TIME_BASE_B])();
    }
  }

  if (((R_TIMEBASEC_CTRL & TIMER_INT_FLAG_MASK) == TIMER_INT_FLAG_MASK) && (time_base_isr_avoid[TIME_BASE_C] != TIME_BASE_TRUE))
  {
    R_TIMEBASEC_CTRL = (R_TIMEBASEC_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERC Interrupt flag*/
    if (gloable_time_base_isr[TIME_BASE_C] != 0) {
        (*gloable_time_base_isr[TIME_BASE_C])();
    }
  }

  if (((R_TIMEBASEA_CTRL & TIMER_INT_FLAG_MASK) == TIMER_INT_FLAG_MASK) && (time_base_times_isr_avoid[TIME_BASE_A] != TIME_BASE_TRUE))
  {
    R_TIMEBASEA_CTRL = (R_TIMEBASEA_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_time_base_isr[TIME_BASE_A] != 0) {
        if (timebase_trap[TIME_BASE_A] < timebase_trap_counter[TIME_BASE_A])
        {
            (*gloable_time_base_isr[TIME_BASE_A])();
            (timebase_trap[TIME_BASE_A])++;
        }
        else
        {
            time_base_times_isr_avoid[TIME_BASE_A] = TIME_BASE_TRUE;
            timebase_trap[TIME_BASE_A] = 0;
            timebase_trap_counter[TIME_BASE_A] = 0;
        }
    }
  }

  if (((R_TIMEBASEB_CTRL & TIMER_INT_FLAG_MASK) == TIMER_INT_FLAG_MASK) && (time_base_times_isr_avoid[TIME_BASE_B] != TIME_BASE_TRUE))
  {
    R_TIMEBASEB_CTRL = (R_TIMEBASEB_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_time_base_isr[TIME_BASE_B] != 0) {
        if (timebase_trap[TIME_BASE_B] < timebase_trap_counter[TIME_BASE_B])
        {
            (*gloable_time_base_isr[TIME_BASE_B])();
            (timebase_trap[TIME_BASE_B])++;
        }
        else
        {
            time_base_times_isr_avoid[TIME_BASE_B] = TIME_BASE_TRUE;
            timebase_trap[TIME_BASE_B] = 0;
            timebase_trap_counter[TIME_BASE_B] = 0;
        }
    }
  }

  if (((R_TIMEBASEC_CTRL & TIMER_INT_FLAG_MASK) == TIMER_INT_FLAG_MASK) && (time_base_times_isr_avoid[TIME_BASE_C] != TIME_BASE_TRUE))
  {
    R_TIMEBASEC_CTRL = (R_TIMEBASEC_CTRL | TIMER_INT_FLAG_MASK);  /*clean the TIMERA Interrupt flag*/
    if (gloable_time_base_isr[TIME_BASE_C] != 0) {
        if (timebase_trap[TIME_BASE_C] < timebase_trap_counter[TIME_BASE_C])
        {
            (*gloable_time_base_isr[TIME_BASE_C])();
            (timebase_trap[TIME_BASE_C])++;
        }
        else
        {
            time_base_times_isr_avoid[TIME_BASE_C] = TIME_BASE_TRUE;
            timebase_trap[TIME_BASE_C] = 0;
            timebase_trap_counter[TIME_BASE_C] = 0;
        }
    }
  }
}

void time_base_reset(void)
{
    DRV_WriteReg32(TIMEBASE_RESET_ADDR, 0x5555);
}

void  drv_msec_wait(INT32U m_sec)
{
#if 0
  	R_TIMERB_PRELOAD = (0x10000-m_sec*1000/122);
	R_TIMERB_CTRL = 0x8063;/*8192 (122us per-count)*/
  	R_TIMERB_CTRL |= 0x2000;
  	while(1) {
  		if (R_TIMERB_CTRL & 0x8000) {
  			break;
  		}
  	}
  	R_TIMERB_CTRL &= ~0x2000;
#endif
	INT32U i, k;
	k = ((INT64U)376UL*MCLK)/96000000UL;
	for (i=0;i<m_sec;i++) {
		TINY_WHILE(0,k);
	}
}

void TIM0_IRQHandler(void)
{
  //clear interrupt & isr callback
 timer_irq_handeler(0);

}

void TIM1_IRQHandler(void)
{
  //clear interrupt & isr callback
 timer_irq_handeler(1);

}

void TIM2_IRQHandler(void)
{
  //clear interrupt & isr callback
 timer_irq_handeler(2);

}

void TIM3_IRQHandler(void)
{
  //clear interrupt & isr callback
 timer_irq_handeler(3);

}

void TIM4_IRQHandler(void)
{
  //clear interrupt & isr callback
 timer_irq_handeler(4);

}

void TIM5_IRQHandler(void)
{
  //clear interrupt & isr callback
 timer_irq_handeler(5);

}

void TIM6_IRQHandler(void)
{
  //clear interrupt & isr callback
  timer_irq_handeler(6);
  //timer_times_irq_handeler(6);
}

void TIM7_IRQHandler(void)
{
  //clear interrupt & isr callback
 timer_irq_handeler(7);

}

#if 0//ggg test only

INT32S timer_test(INT32U timer_id, INT32U ext_clk, INT32U freq_hz, void (*TimerIsrFunction)(void))
{
    switch (timer_id) {
    case TIMER_A:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM0_IRQn);
        //vic_irq_register(VIC_TIMER0, local_timer_isr0);
        //vic_irq_enable(VIC_TIMER0);
        break;
    case TIMER_B:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM1_IRQn);
        //vic_irq_register(VIC_TIMER1, local_timer_isr1);
        //vic_irq_enable(VIC_TIMER1);
        break;
    case TIMER_C:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM2_IRQn);
        //vic_irq_register(VIC_TIMER2, local_timer_isr2);
        //vic_irq_enable(VIC_TIMER2);
        break;
    case TIMER_D:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM3_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_E:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM4_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_F:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM5_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_G:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM6_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    case TIMER_H:
        gloable_timer_isr[timer_id] = TimerIsrFunction;
        NVIC_EnableIRQ(TIM7_IRQn);
        //vic_irq_register(VIC_TIMER3, local_timer_isr3);
        //vic_irq_enable(VIC_TIMER3);
        break;
    default:
        return TIMER_FAIL;
        break;
    }


  NVIC_EnableIRQ(TIM0_IRQn);
  TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(ext_clk/freq_hz)));
  #if 0//TIMER_COUNTER_BIT_MODE == TIMER_COUNTER_BIT16
    TIMER_PRELOAD_COUNTER_SET(timer_id,(0x10000-(ext_clk/freq_hz)));
//    DRV_WriteReg32(TIMER_BASE_ADDR+EACH_REG_OFFSET*8*timer_id, 0x01000000);
  #else
    TIMER_PRELOAD_COUNTER_SET(timer_id,(0xFFFFFFFF-(ext_clk/freq_hz)+1));
    DRV_WriteReg32(TIMER_BASE_ADDR+EACH_REG_OFFSET*8*timer_id, 0x00001000);
  #endif
  timer_control(timer_id,SRCA_TOUT_2,SRCB_1_,EXTBSEL_DEFAULT,EXTASEL_DEFAULT,
	                TIMER_ENABLE, TIMER_INT_ENABLE, TIMER_INT_FLAG_FALSE);

}
#endif//end ggg test only

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#endif //(defined _DRV_L1_TIMER) && (_DRV_L1_TIMER == 1)          //
//================================================================//

