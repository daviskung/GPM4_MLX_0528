#include "drv_l1_sfr.h"
#include "drv_l1_fft.h"
#include <stdio.h>
//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined _DRV_L1_FFT) && (_DRV_L1_FFT == 1)                   //
//================================================================//
void (*fft_cbk_func)(void);

#if OBJ_FUNCTION_EN == 1
INT32U (*fft_obj_malloc_func)(INT32U size);
#define C_OBJ_QUEUE_SIZE		        5
#if _OPERATING_SYSTEM == _OS_FREERTOS
#define SYSTEM_OS_ENABLE                1
#else
#define SYSTEM_OS_ENABLE                0
#endif

#define CLK_EN1_FFT				23
#define C_FFT_ISR_QUEUE_SIZE	1
#define C_FFT_ACK				1
#if SYSTEM_OS_ENABLE == 1
osMessageQId	fft_ack_q	= NULL;
osMessageQId obj_state_q = NULL;
osSemaphoreId 	fft_obj_hw_sem = NULL;
#else
static INT32U obj_hit;
#endif
static INT32U fft_obj_init_flag = 0,obj_state;
static void obj_isr(void);
#endif

//INT32U drv_l1_get_fft_ovf_sts(void);

void drv_l1_fft_obj_lock(void)
{
#if SYSTEM_OS_ENABLE == 1
    if(fft_obj_hw_sem)
        osSemaphoreWait(fft_obj_hw_sem, osWaitForever);
#endif
}

void drv_l1_fft_obj_unlock(void)
{
#if SYSTEM_OS_ENABLE == 1
	if(fft_obj_hw_sem)
        osSemaphoreRelease(fft_obj_hw_sem);
#endif
}
#if OBJ_FUNCTION_EN == 1
INT32S drv_l1_obj_start(void)
{
    drv_l1_fft_obj_lock();
#if SYSTEM_OS_ENABLE == 0
    obj_state = 1;
#endif
    P_FFT_START = C_FFT_OBJ_START;
    drv_l1_fft_obj_unlock();

    return 0;
}

INT32S drv_l1_obj_scale_set(INT32U unit)
{
    P_FFT_SCALE = (unit & C_OBJ_SCALE_MASK);

    return 0;
}

INT32S drv_l1_obj_database_addr_set(INT32U addr)
{
    if(!addr || addr & 0x3)
        return -1;

    P_OBJ_RADDR = addr;

    return 0;
}

INT32S drv_l1_obj_output_buffer_addr_set(INT32U addr)
{
    if(!addr || addr & 0x3)
        return -1;

    P_FFT_WADDR = addr;

    return 0;
}

static INT32S drv_l1_obj_interrupt_status_get(void)
{
    INT32U temp;

    temp = P_FFT_INTSTS;

    return temp;
}

static INT32S drv_l1_obj_interrupt_status_set(INT32U vaule)
{
    P_FFT_INTSTS = vaule;

    return 0;
}

static INT16S drv_l1_fft_clock_set(INT16U enable)
{
	return	drv_l1_clock_set_system_clk_en(CLK_EN1_FFT, enable);
}

INT32S drv_l1_obj_compare_buffer_addr_set(INT32U addr)
{
    if(!addr || addr & 0x3)
        return -1;

    P_FFT_RADDR = addr;

    return 0;
}

INT32S drv_l1_obj_threshold_set(INT32U value)
{
    P_OBJ_THRESHOLD = (value & C_OBJ_THRESHOLD_MASK);

    return 0;
}

INT32S drv_l1_obj_minimum_error_get(void)
{
    INT32U temp;

    drv_l1_fft_obj_lock();
    temp = P_OBJ_MIN;
    drv_l1_fft_obj_unlock();

    return temp;
}

INT32S drv_l1_obj_minimum_error_id_get(void)
{
    INT32U temp;

    drv_l1_fft_obj_lock();
    temp = P_OBJ_MIN_ID;
    drv_l1_fft_obj_unlock();

    return temp;
}

static INT32S drv_l1_obj_hit_get(void)
{
#if SYSTEM_OS_ENABLE == 1
    INT32S state;
    osEvent result;

    result = osMessageGet(obj_state_q, 5000);
    state = result.value.v;

    return (state >> 1);
#else
    return (obj_hit >> 1);
#endif
}

static void obj_isr(void)
{
    INT32S temp;

#if SYSTEM_OS_ENABLE == 1
    obj_state = 0;
#else
    obj_hit = 0;
#endif
    temp = drv_l1_obj_interrupt_status_get();
    drv_l1_obj_interrupt_status_set(temp);

    if(temp & C_OBJ_INT_HIT_STATE)
        #if SYSTEM_OS_ENABLE == 1
            obj_state |= C_OBJ_INT_HIT_STATE;
        #else
            obj_hit = C_OBJ_INT_HIT_STATE;
        #endif

    if(temp & C_FFT_OBJ_INT_STATE)
        #if SYSTEM_OS_ENABLE == 1
            obj_state |= C_FFT_OBJ_INT_STATE;
        #else
            obj_state = 0;
        #endif

    #if SYSTEM_OS_ENABLE == 1
        if(obj_state_q) {
            osMessagePut(obj_state_q, (uint32_t)&obj_state, 1);
        }
    #endif
}

INT32S drv_l1_obj_end(INT8U wait)
{
    INT32S temp = -1;

    drv_l1_fft_obj_lock();
    if(wait)
    {
#if SYSTEM_OS_ENABLE == 0
        #if 1
         while(1)
         {
            if(obj_state == 0)
                break;
            else
                osDelay(1);
         }
        #else
        while(obj_state);
        #endif
#endif
        temp = drv_l1_obj_hit_get();
    }
    drv_l1_fft_obj_unlock();

    return temp;
}

INT32S drv_l1_obj_init(void)
{
    drv_l1_obj_interrupt_status_set(C_FFT_OBJ_INT_STATE_MASK);
    P_FFT_START = C_FFT_OBJ_RESET;
    P_FFT_CTRL = (C_OBJ_INT_ENABLE | C_FFT_OBJ_ENABLE |C_OBJ_ENABLE);
    obj_state = 0;
    if(fft_obj_init_flag != C_OBJ_ENABLE)
    {
        #if SYSTEM_OS_ENABLE == 1
            if(obj_state_q == NULL) {
                osMessageQDef_t obj_q = { C_OBJ_QUEUE_SIZE, sizeof(INT32U), 0 };

                obj_state_q = osMessageCreate(&obj_q, NULL);
            }

            if(fft_obj_hw_sem == NULL) {
                osSemaphoreDef_t fft_obj_sem = { 0 };
                fft_obj_hw_sem = osSemaphoreCreate(&fft_obj_sem, 1);
            }
        #endif
        NVIC_SetPriority(FFT_IRQn, 5);
        NVIC_EnableIRQ(FFT_IRQn);
        fft_obj_init_flag = C_OBJ_ENABLE;
    }

    return 0;
}
INT32U drv_l1_obj_malloc(INT32U size)
{
    if(fft_obj_malloc_func)
        return fft_obj_malloc_func(size);
    else
        return 0;
}

INT32S drv_l1_obj_user_malloc_set(INT32U (*malloc_function)(INT32U size))
{
    if(malloc_function)
    {
        fft_obj_malloc_func = malloc_function;
        return 0;
    }
    else
        return -1;
}
#endif

static INT32U drv_l1_fft_intr_status_get(void)
{
	return (P_FFT_INTSTS&0x01);
}

static void drv_l1_fft_intr_status_clear(void)
{
	P_FFT_INTSTS |= 0x01;		//write 1 clear
}

static void fft_isr(void)
{
	if(fft_ack_q){
		INT32U	event;

		event = C_FFT_ACK;
		osMessagePut(fft_ack_q, (uint32_t)&event, osWaitForever);
	}

    if(fft_cbk_func!=0)
        (*fft_cbk_func)();

	drv_l1_fft_intr_status_clear();
}

void FFT_IRQHandler(void)
{
#if OBJ_FUNCTION_EN == 1
    if(fft_obj_init_flag == C_OBJ_ENABLE)   // obj function
    {
        obj_isr();
    }
    else
#endif

	fft_isr();

}

static void drv_l1_fft_hw_en(void)
{
    P_FFT_CTRL |= SET_FFT_HW_ENA;
}

INT32S drv_l1_fft_wait_done(INT32U timeout)
{
	osEvent	result;

	if(!fft_ack_q)
		return -1;

	if(timeout == 0)
		result = osMessageGet(fft_ack_q, osWaitForever);
	else
		result = osMessageGet(fft_ack_q, timeout);

	 if((result.status != osEventMessage) || (result.value.v == 0)) {
        DBG_PRINT("FFT Wait timeout (%d ms)\r\n", timeout);
        return -1;
    }
    return 0;
}



void drv_l1_fft_callback_set( void(*fft_cbk)(void))
{
    fft_cbk_func = fft_cbk;
}

static void drv_l1_fft_callback_clear(void)
{
    if(fft_cbk_func != 0)
        fft_cbk_func = 0;
}

static void drv_l1_fft_hw_dis(void)
{
    P_FFT_CTRL &= ~SET_FFT_HW_ENA;
}

static void drv_l1_fft_ena_intr(void)
{
    P_FFT_CTRL |= SET_FFT_INT_ENA;
}

static void drv_l1_fft_dis_intr(void)
{
    P_FFT_CTRL &= ~SET_FFT_INT_ENA;
}



void drv_l1_fft_init(void)
{
	drv_l1_fft_clock_set(ENABLE);

    if(drv_l1_fft_intr_status_get() == 1 || (drv_l1_get_fft_ovf_sts()))
		drv_l1_fft_intr_status_clear();

    if(fft_ack_q == NULL) {
        osMessageQDef_t fft_a_q = { C_FFT_ISR_QUEUE_SIZE, sizeof(INT32U), 0 };

        fft_ack_q = osMessageCreate(&fft_a_q, NULL);
    }
    else{
		xQueueReset(fft_ack_q);
    }

	if(fft_obj_hw_sem == NULL) {
        osSemaphoreDef_t fft_obj_sem = { 0 };
        fft_obj_hw_sem = osSemaphoreCreate(&fft_obj_sem, 1);
    }

    P_FFT_START = 0;
    //P_FFT_CTRL = 0;
    P_FFT_SCALE = 0;
    P_FFT_RADDR = (INT32S)NULL;
    P_FFT_WADDR = (INT32S)NULL;
    fft_cbk_func = 0;

    drv_l1_fft_hw_en();
    drv_l1_fft_ena_intr();

    NVIC_SetPriority(FFT_IRQn, 5);
    NVIC_EnableIRQ(FFT_IRQn);
#if OBJ_FUNCTION_EN == 1
    fft_obj_init_flag = C_FFT_OBJ_ENABLE;
#endif
}
void drv_l1_fft_uninit(void)
{
	NVIC_DisableIRQ(FFT_IRQn);
	drv_l1_fft_callback_clear();
	drv_l1_fft_dis_intr();
	drv_l1_fft_hw_dis();
	drv_l1_fft_clock_set(DISABLE);
	fft_obj_init_flag = 0;
}


INT32S drv_l1_fft_set_points(INT32U FFT_N)
{
    INT32S ret = 0;
    INT32S value = 0;

    if(FFT_N==32)
        value |= SET_FFT_32P;
    else if(FFT_N==64)
        value |= SET_FFT_64P;
    else if(FFT_N==128)
        value |= SET_FFT_128P;
    else if(FFT_N==256)
        value|= SET_FFT_256P;
    else if(FFT_N==512)
        value |= SET_FFT_512P;
    else if(FFT_N==1024)
        value |= SET_FFT_1024P;
    else if(FFT_N==2048)
        value |= SET_FFT_2048P;
    else
        ret = -1;

    P_FFT_CTRL &= ~0x07;
    P_FFT_CTRL |= value;

    return ret;
}

INT32S drv_l1_set_fft_dir(INT32U inv)
{
    INT32S ret=0;
    if(inv == C_DIR_IFFT)
        P_FFT_CTRL |= SET_DIR_IFFT;
    else if(inv == C_DIR_FFT)
        P_FFT_CTRL &= (~SET_DIR_IFFT);
    else
        ret = -1;

    return ret;
}

INT32S drv_l1_fft_set_scl(INT32U stage, INT32U div)
{
    INT32S ret = 0;

    if( (stage>5) ||( div!=2 && div!=4) )
        return -1;
    else{
        if(stage == 0){
            if(div == 2)
                P_FFT_SCALE = SET_SCL_STG_0_12;
            else
                P_FFT_SCALE = SET_SCL_STG_0_14;

        }else if(stage == 1){
            if(div == 2)
                P_FFT_SCALE = SET_SCL_STG_1_12;
            else
                P_FFT_SCALE = SET_SCL_STG_1_14;
        }else if(stage == 2){
            if(div == 2)
                P_FFT_SCALE = SET_SCL_STG_2_12;
            else
                P_FFT_SCALE = SET_SCL_STG_2_14;
        }else if(stage == 3){
            if(div == 2)
                P_FFT_SCALE = SET_SCL_STG_3_12;
            else
                P_FFT_SCALE = SET_SCL_STG_3_14;
        }else if(stage == 4){
            if(div == 2)
                P_FFT_SCALE = SET_SCL_STG_4_12;
            else
                P_FFT_SCALE = SET_SCL_STG_4_14;
        }else if(stage == 5){
            if(div == 2)
                P_FFT_SCALE = SET_SCL_STG_5_12;
            else
                P_FFT_SCALE = SET_SCL_STG_5_14;
        }
    }
    return ret;
}

void drv_l1_fft_in_addr(INT32U *addr)
{
    P_FFT_RADDR = (INT32S)addr;
}

void drv_l1_fft_out_addr(INT32U *addr)
{
    P_FFT_WADDR = (INT32S)addr;
}

INT32S drv_l1_fft_set_in_fmt(FFT_IN_FMT_ENUM in_fmt)
{
    INT32S ret=0;

    if(in_fmt == C_IN_R_16b)
        P_FFT_CTRL |= SET_16b_FFTIN_R;
    else if(in_fmt == C_IN_R_I_16b)
        P_FFT_CTRL |= SET_16b_FFTIN_R_AND_I;
    else if(in_fmt == C_IN_R_24b)
        P_FFT_CTRL |= SET_24b_FFTIN_R;
    else if(in_fmt == C_IN_R_I_24b)
        P_FFT_CTRL |= SET_24b_FFTIN_R_AND_I;
    else
        ret = -1;

    return ret;

}

INT32S drv_l1_fft_set_out_fmt(FFT_OUT_FMT_ENUM out_fmt)
{
    INT32S ret=0;

    if(out_fmt == C_OUT_R_16b)
        P_FFT_CTRL |= SET_16b_FFTOUT_R;
    else if(out_fmt == C_OUT_R_I_16b)
        P_FFT_CTRL |= SET_16b_FFTOUT_R_AND_I;
    else if(out_fmt == C_OUT_R_24b)
        P_FFT_CTRL |= SET_24b_FFTOUT_R;
    else if(out_fmt == C_OUT_R_I_24b)
        P_FFT_CTRL |= SET_24b_FFTOUT_R_AND_I;
    else
        ret = -1;

    return ret;

}

INT32U drv_l1_get_fft_ovf_sts(void)
{
	INT32U ovf_sts = P_FFT_OVFSTS&0x7F;
	return ovf_sts;
}
void drv_l1_fft_start(void)
{
	P_FFT_START |= SET_FFT_TRIG;
}


//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#endif //(defined _DRV_L1_FFT) && (_DRV_L1_FFT == 1)              //
//================================================================//
