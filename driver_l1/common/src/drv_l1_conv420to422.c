
#include "drv_l1_conv420to422.h"

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined _DRV_L1_CONV420TO422) && (_DRV_L1_CONV420TO422 == 1)       

/******************************************************/
CONV420_SFR* pConv420; 
static void (*Conv420_Isr_Callback)(INT32U Conv420_Event);

/******************************************************/
#define CONV420_DISABLE_FALSE		0
#define CONV420_ENABLE_TRUE		1

/*
	REG_CTRL
*/
#define CONV420_DEST_PATH_IDX(x)			(x<<1)
#define CONV420_DO_START					(1<<8)
#define CONV420_DO_RESET					(1<<9)
#define CONV420_FIFO_LINE_IDX(x)			(x<<16)
#define CONV420_INTERRUPT_SETTING_IDX(x)	(x<<20)
#define CONV420_MASTER_MASK_ENABLE_IDX(x)	(x<<21)
#define CONV420_FIFO_INT_IDX				(1<<29)
#define CONV420_FIFO_INT_STATUS_IDX			((INT32U)1<<31)


/*****************************************************
*	get_CONV420_SFR_base: 
*
*****************************************************/
CONV420_SFR* get_CONV420_SFR_base(void)
{
	return (CONV420_SFR*)P_CONV420_BASE;
}

/*****************************************************
*	conv420_init: 
*
*****************************************************/
INT32S conv420_init(void)
{
	pConv420 = get_CONV420_SFR_base();
	return STATUS_OK;	
}

/*****************************************************
*	conv420_reset: 
*
*****************************************************/
INT32S conv420_reset(void)
{
	Conv420_Isr_Callback = NULL;
	pConv420->CTRL = 0;
	pConv420->LINE_WIDTH_4BYTE = 0;

	NVIC_DisableIRQ(CONV420TO422_IRQn);    
	
	pConv420->CTRL |= CONV420_DO_RESET;

	return STATUS_OK;	
}

/*****************************************************
*	conv420_start: 
*
*****************************************************/
INT32S conv420_start(void)
{
	pConv420->CTRL |= CONV420_DO_START;

	return STATUS_OK;	
}

/*****************************************************
*	conv420_convert_enable: 
*	1:Enable GP420->YUV422  0:YUV422->YUV422
*****************************************************/
INT32S conv420_convert_enable(INT8U ctrlValue)
{
	pConv420->CTRL &= ~(CONV420_ENABLE_TRUE);	

	if(ctrlValue == CONV420_ENABLE_TRUE)
	{
		pConv420->CTRL |= CONV420_ENABLE_TRUE;
	}
	return STATUS_OK;
}

/*****************************************************
*	conv420_path: 
*
*****************************************************/
INT32S conv420_path(INT8U pathValue)
{
	pConv420->CTRL &= ~(CONV420_DEST_PATH_IDX(1));
	pConv420->CTRL |= CONV420_DEST_PATH_IDX(pathValue);

	return STATUS_OK;	
}

/*****************************************************
*	conv420_input_A_addr_set: 
*
*****************************************************/
INT32S conv420_input_A_addr_set(INT32U addr)
{
	pConv420->IN_ADDR_A = addr;

	pConv420->MASTER_MASK_INT = 0;

	return STATUS_OK;	
}

/*****************************************************
*	conv420_input_B_addr_set: 
*
*****************************************************/
INT32S conv420_input_B_addr_set(INT32U addr)
{
	pConv420->IN_ADDR_B = addr;

	pConv420->MASTER_MASK_INT = 0;

	return STATUS_OK;	
}

/*****************************************************
*	conv420_input_pixels_set: 
*
*****************************************************/
INT32S conv420_input_pixels_set(INT32U inWidth)
{
	pConv420->LINE_WIDTH_4BYTE &= ~(0x1FF);

	pConv420->LINE_WIDTH_4BYTE |= ((inWidth>>2)-1);

	return STATUS_OK;	
}

/*****************************************************
*	conv420_fifo_line_set: 
*
*****************************************************/
INT32S conv420_fifo_line_set(INT32U lineCount)
{
	pConv420->LINE_WIDTH_4BYTE &= ~(CONV420_FIFO_LINE_IDX(0xFF));

	if(lineCount > 0)
	{
		if(pConv420->CTRL & CONV420_ENABLE_TRUE) // Convert GP420 to YUYV
		{	
			pConv420->LINE_WIDTH_4BYTE |= CONV420_FIFO_LINE_IDX(((lineCount*3)>>1)); // conv420 »Ý­n1.5­¿
		}
		else
		{
			pConv420->LINE_WIDTH_4BYTE |= CONV420_FIFO_LINE_IDX(lineCount);
		}
	}
	
	return STATUS_OK;	
}

/*****************************************************
*	conv420_idle_buf_get: 
*
*****************************************************/
INT32S conv420_idle_buf_get(void)
{
	INT32U intFlag;
	INT32U Conv420_Interrupt_Flag;
	
	intFlag = pConv420->CTRL;
	pConv420->CTRL = intFlag; // Clear Bit31

	Conv420_Interrupt_Flag = CONV420_STATUS_BUSY;
	
	if(intFlag & CONV420_FIFO_INT_IDX)
	{
		Conv420_Interrupt_Flag |= CONV420_STATUS_BUF_B_DONE;
	}
	else
	{
		Conv420_Interrupt_Flag |= CONV420_STATUS_BUF_A_DONE;
	}
	
	return Conv420_Interrupt_Flag;
}

/*****************************************************
*	conv420_fifo_interrupt_enable: 
*
*****************************************************/
INT32S conv420_fifo_interrupt_enable(INT8U enableValue)
{
	pConv420->CTRL &= ~(CONV420_INTERRUPT_SETTING_IDX(1)|CONV420_MASTER_MASK_ENABLE_IDX(1)); // Clear bit

	if(enableValue == CONV420_ENABLE_TRUE)
	{
		pConv420->CTRL |= CONV420_INTERRUPT_SETTING_IDX(enableValue)|CONV420_MASTER_MASK_ENABLE_IDX(enableValue);
	}

	return STATUS_OK;	
}

/*****************************************************
*	conv420_fifo_interrupt_status_get: 
*
*****************************************************/
INT32S conv420_fifo_interrupt_status_get(void)
{
	INT32U intFlag;
	INT32U Conv420_Interrupt_Flag;
	
	intFlag = pConv420->CTRL;
	pConv420->CTRL = intFlag; // Clear Bit31

	Conv420_Interrupt_Flag = CONV420_STATUS_BUSY;
	
	if(!(intFlag & CONV420_FIFO_INT_STATUS_IDX))
	{
		return Conv420_Interrupt_Flag;
	}
	
	if(intFlag & CONV420_FIFO_INT_IDX)
	{
		Conv420_Interrupt_Flag |= CONV420_STATUS_BUF_B_DONE;
	}
	else
	{
		Conv420_Interrupt_Flag |= CONV420_STATUS_BUF_A_DONE;
	}
	
	return Conv420_Interrupt_Flag;
}

/*****************************************************
*	CONV420TO422_IRQHandler: 
*
*****************************************************/
void CONV420TO422_IRQHandler(void)
{
	INT32U Conv420_Interrupt_Flag;

	Conv420_Interrupt_Flag = conv420_fifo_interrupt_status_get();

	if(Conv420_Isr_Callback)
	{
		(*Conv420_Isr_Callback)(Conv420_Interrupt_Flag);				
	}
}

/*****************************************************
*	Conv420_Callback_Register: 
*
*****************************************************/
void Conv420_Callback_Register(void (*Conv420_Callback)(INT32U Conv420_Event))
{
    //NVIC_SetPriority(CONV420TO422_IRQn, 5);
	NVIC_EnableIRQ(CONV420TO422_IRQn);    

	Conv420_Isr_Callback = Conv420_Callback;
}

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#endif //(defined _DRV_L1_CONV420TO422) && (_DRV_L1_CONV420TO422 == 1)//

