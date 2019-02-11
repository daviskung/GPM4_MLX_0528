
#include "drv_l1_pscaler.h"
#include "drv_l1_clock.h"

/******************************************************/
static void (*PScaler_Isr_Callback[PSCALER_MAX])(INT32U PScaler_Event);
volatile INT32S PScalerInterruptFlag[PSCALER_MAX] = {0};

/*****************************************************
*	get_PSCALER_Reg_Base:
*
*****************************************************/
static PIPELINE_SCALER_SFR * get_PSCALER_Reg_Base(INT8U pscaler_num)
{
	if (pscaler_num == PSCALER_A)
	{
		return (PIPELINE_SCALER_SFR *)PIPELINE_SCALERA_BASE;
	}
	else
	{
		return (PIPELINE_SCALER_SFR *)PIPELINE_SCALERB_BASE;
	}
}

/*****************************************************
*	PSCALER_IRQHandler:
*
*****************************************************/
void PSCALER_IRQHandler(INT8U PScalerNum)
{
	PIPELINE_SCALER_SFR* pPScaler_Reg;
	INT32U intFlag;
	INT32U PScaler_Interrupt_Flag;

	PScaler_Interrupt_Flag = PIPELINE_SCALER_STATUS_BUSY;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);
    if(pPScaler_Reg->PSCA_IN_X_WIDTH > 1920)
        {
        drv_l1_pscaler_H_first_enable(PScalerNum, 1);
        }
	intFlag = pPScaler_Reg->PSCA_INT_FLAG;
	pPScaler_Reg->PSCA_INT_FLAG = intFlag;

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_422GP420_FRAME_END)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_FRAME_DONE;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_FRAME_DONE;
	}

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_A)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_BUF_A_DONE;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_BUF_A_DONE;
	}

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_B)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_BUF_B_DONE;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_BUF_B_DONE;
	}

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_MB420_FRAME_END)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_MB420_FRAME_DONE;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_MB420_FRAME_DONE;
	}

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_MB420_BUF_A)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_MB420_BUF_A_DONE;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_MB420_BUF_A_DONE;
	}

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_MB420_BUF_B)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_MB420_BUF_B_DONE;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_MB420_BUF_B_DONE;
	}

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_OVERFLOW)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR;
	}

	if(intFlag & PIPELINE_SCALER_INT_ENABLE_AHB_IN_FIFO_DONE)
	{
		PScaler_Interrupt_Flag |= PIPELINE_SCALER_STATUS_INPUT_EMPTY;
		PScalerInterruptFlag[PScalerNum] |= PIPELINE_SCALER_STATUS_INPUT_EMPTY;
	}

	// Call Back
	if(PScaler_Isr_Callback[PScalerNum])
	{
		(*PScaler_Isr_Callback[PScalerNum])(PScaler_Interrupt_Flag);
	}
}

void PSCALER0_IRQHandler(void)
{
	PSCALER_IRQHandler(PSCALER_A);
}

void PSCALER1_IRQHandler(void)
{
	PSCALER_IRQHandler(PSCALER_B);
}

/*****************************************************
*	drv_l1_pscaler_init:
*
*****************************************************/
void drv_l1_pscaler_init(INT8U PScalerNum)
{
	PIPELINE_SCALER_SFR* pPScaler_Reg;
    drv_l1_pscaler_clk_ctrl(PScalerNum,1);
	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_CTRL = 0x04; // Reset HW
	pPScaler_Reg->PSCA_INTEN_CTRL = 0;
	pPScaler_Reg->PSCA_INT_FLAG = 0xFFFF; // Clear All Interrupt Flag: Write 1 clear
	pPScaler_Reg->PSCA_CTRL = 0;
	pPScaler_Reg->PSCA_X_START = 0;
	pPScaler_Reg->PSCA_Y_START = 0;
	pPScaler_Reg->PSCA_HEIGHT_LINE = 0;
	pPScaler_Reg->PSCA_H_PERIOD_CTRL = 0;
	pPScaler_Reg->PSCA_OUTBND_COLOR = 0;
	pPScaler_Reg->PSCA_BLD0_CTRL0 = 0;
	pPScaler_Reg->PSCA_BLD1_CTRL0 = 0;
	pPScaler_Reg->PSCA_H_PERIOD_CTRL = 0;
	pPScaler_Reg->PSCA_1D_FILTER_CTRL = 0;


	PScaler_Isr_Callback[PScalerNum] = NULL;
	PScalerInterruptFlag[PScalerNum]  = 0;

	if(PScalerNum == PSCALER_A)
	{
	  NVIC_SetPriority(PSCALER_A_IRQn, 5);
		NVIC_EnableIRQ(PSCALER_A_IRQn);
	}
	else
	{
	  NVIC_SetPriority(PSCALER_B_IRQn, 5);
		NVIC_EnableIRQ(PSCALER_B_IRQn);
	}
}

/*****************************************************
*	drv_l1_pscaler_input_pixels_set:
*
*****************************************************/
INT32S drv_l1_pscaler_input_pixels_set(INT8U PScalerNum, INT32U inWidth, INT32U inHeight)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_IN_X_WIDTH = (inWidth-1);
	pPScaler_Reg->PSCA_IN_Y_WIDTH = (inHeight-1);

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_output_pixels_set:
*	MB420: outWidth & outHeight must be 16-byte alignment
*****************************************************/
INT32S drv_l1_pscaler_output_pixels_set(INT8U PScalerNum, INT32U widthFactor, INT32U outWidth, INT32U heightFactor, INT32U outHeight)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_X_FACTOR = widthFactor;

	pPScaler_Reg->PSCA_OUT_X_WIDTH &= (~0x1FFF);
	pPScaler_Reg->PSCA_OUT_X_WIDTH |= ((outWidth-1) & 0x1FFF);

	pPScaler_Reg->PSCA_Y_FACTOR = heightFactor;
	pPScaler_Reg->PSCA_OUT_Y_WIDTH = (outHeight-1);

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_input_X_start_set:
*
*****************************************************/
INT32S drv_l1_pscaler_input_X_start_set(INT8U PScalerNum, INT32U xOffset)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_X_START = (xOffset << 16);

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_input_Y_start_set:
*
*****************************************************/
INT32S drv_l1_pscaler_input_Y_start_set(INT8U PScalerNum, INT32U yOffset)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_Y_START = (yOffset << 16);

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_output_A_buffer_set:
*	bufAddr: 4-byte alignment
*****************************************************/
INT32S drv_l1_pscaler_output_A_buffer_set(INT8U PScalerNum, INT32U bufAddr)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_OUT_A_ADDR = bufAddr;

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_output_A_buffer_get:
*****************************************************/
INT32U drv_l1_pscaler_output_A_buffer_get(INT8U PScalerNum)
{
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	return (pPScaler_Reg->PSCA_OUT_A_ADDR);
}

/*****************************************************
*	drv_l1_pscaler_output_B_buffer_set:
*	bufAddr: 4-byte alignment
*****************************************************/
INT32S drv_l1_pscaler_output_B_buffer_set(INT8U PScalerNum, INT32U bufAddr)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_OUT_B_ADDR = bufAddr;

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_output_B_buffer_get:
*****************************************************/
INT32U drv_l1_pscaler_output_B_buffer_get(INT8U PScalerNum)
{
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	return (pPScaler_Reg->PSCA_OUT_B_ADDR);
}

/*****************************************************
*	drv_l1_pscaler_output_fifo_line_set:
*	Only Support 422/GP420 , Not Support MB420
*****************************************************/
INT32S drv_l1_pscaler_output_fifo_line_set(INT8U PScalerNum, INT32U fifoLine, INT8U outStopEnable)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_HEIGHT_LINE |= (fifoLine|(outStopEnable << 15)); // Bit15: Output Fifo Mode Output Full Stop

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_input_fifo_line_set:
*
*****************************************************/
INT32S drv_l1_pscaler_input_fifo_line_set(INT8U PScalerNum, INT8U addrsMode, INT8U fifoLine)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_HEIGHT_LINE |= ((addrsMode << 23)|(fifoLine << 16));

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_interrupt_set:
*
*****************************************************/
INT32S drv_l1_pscaler_interrupt_set(INT8U PScalerNum, INT32U intEnVale)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_INTEN_CTRL = 0;

	if(intEnVale > PIPELINE_SCALER_INT_ENABLE_DISABLE)
	{
		pPScaler_Reg->PSCA_INTEN_CTRL = (intEnVale|(0x01 << 16)); // Bit16:Enable Interrupt
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_input_format_set:
*	PIPELINE_SCALER_INPUT_FORMAT_GP420: Only For PSCALER_B
*
*****************************************************/
INT32S drv_l1_pscaler_input_format_set(INT8U PScalerNum, INT32U inFormat)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_CTRL &= ~((0x0F << 16)|(0x01 << 23));

	if(inFormat == PIPELINE_SCALER_INPUT_FORMAT_GP420)
	{
		pPScaler_Reg->PSCA_CTRL |= (0x01 << 23); // Bit23: Input GP420 Mode
		inFormat = PIPELINE_SCALER_INPUT_FORMAT_YUYV;
	}

	pPScaler_Reg->PSCA_CTRL |= (inFormat << 16); // Bit16~19: Input Format

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_output_format_set:
*
*****************************************************/
INT32S drv_l1_pscaler_output_format_set(INT8U PScalerNum, INT32U outFormat)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_CTRL &= ~(0x07 << 8); // Bit8~11: Output Format
	pPScaler_Reg->PSCA_CTRL |= (outFormat << 8); // Bit8~11: Output Format

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_input_source_set:
*
*****************************************************/
INT32S drv_l1_pscaler_input_source_set(INT8U PScalerNum, INT32U inSource)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_CTRL &= ~(0x07 << 4); // Bit4~6: Input Source
	pPScaler_Reg->PSCA_CTRL |= (inSource << 4); // Bit4~6: Input Source

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_input_buffer_set:
*	bufAddr: 4-byte alignment
*****************************************************/
INT32S drv_l1_pscaler_input_buffer_set(INT8U PScalerNum, INT32U bufAddr)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_IN_ADDR = bufAddr;

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_H264_output_A_buffer_set:
*	bufAddr: 4-byte alignment
*****************************************************/
INT32S drv_l1_pscaler_H264_output_A_buffer_set(INT8U PScalerNum, INT32U bufAddr)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_OUT_A_ADDR_MB420 = bufAddr;

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_H264_output_A_buffer_get:
*****************************************************/
INT32U drv_l1_pscaler_H264_output_A_buffer_get(INT8U PScalerNum)
{
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	return (pPScaler_Reg->PSCA_OUT_A_ADDR_MB420);
}

/*****************************************************
*	drv_l1_pscaler_H264_output_B_buffer_set:
*	bufAddr: 4-byte alignment
*****************************************************/
INT32S drv_l1_pscaler_H264_output_B_buffer_set(INT8U PScalerNum, INT32U bufAddr)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_OUT_B_ADDR_MB420 = bufAddr;

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_H264_output_B_buffer_get:
*****************************************************/
INT32U drv_l1_pscaler_H264_output_B_buffer_get(INT8U PScalerNum)
{
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	return (pPScaler_Reg->PSCA_OUT_B_ADDR_MB420);
}

/*****************************************************
*	drv_l1_pscaler_H264_mirror_set:
*
*****************************************************/
INT32S drv_l1_pscaler_H264_mirror_set(INT8U PScalerNum, INT32U mirrorEnable)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(mirrorEnable)
	{
		pPScaler_Reg->PSCA_CTRL |= (mirrorEnable << 25); // Bit25: Mirror Enalbe
	}
	else
	{
		pPScaler_Reg->PSCA_CTRL &= ~(0x01 << 25); // Bit25: Mirror Enalbe
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_H264_flip_set:
*
*****************************************************/
INT32S drv_l1_pscaler_H264_flip_set(INT8U PScalerNum, INT32U flipEnable)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(flipEnable)
	{
		pPScaler_Reg->PSCA_CTRL |= (flipEnable << 24); // Bit24: Flip Enalbe
	}
	else
	{
		pPScaler_Reg->PSCA_CTRL &= ~(0x01 << 24); // Bit24: Flip Enalbe
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_output_pixels_X_offset_set:
*	Only Support 422/GP420 , Not Support MB420
*****************************************************/
INT32S drv_l1_pscaler_output_pixels_X_offset_set(INT8U PScalerNum, INT32U outOffsetX)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_OUT_X_WIDTH &= (~0x1FFF0000);
	pPScaler_Reg->PSCA_OUT_X_WIDTH |= ((outOffsetX & 0x1FFF) << 16);

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_outboundary_color_set:
*	outBoundEnable: 0:Disable 1:Enable
*	outBoundColor: 0x8080: Black
*****************************************************/
INT32S drv_l1_pscaler_outboundary_color_set(INT8U PScalerNum, INT8U outBoundEnable, INT32U outBoundColor)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(outBoundEnable) // [Bit31]: Enable Outboundary color
	{
		pPScaler_Reg->PSCA_OUTBND_COLOR = (0x80000000|(outBoundColor & 0x00FFFFFF));
	}
	else
	{
		pPScaler_Reg->PSCA_OUTBND_COLOR = 0;
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_text_stamp_set:
*
*****************************************************/
INT32S drv_l1_pscaler_text_stamp_set(INT8U PScalerNum, TEXT_STAMP_PARAM* ptextStampParam)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(ptextStampParam->textStampSet == TEXT_STAMP_1ST_SETTING)
	{	 // First Setting
		pPScaler_Reg->PSCA_BLD0_ADDR = ptextStampParam->textSrcAddrs;
		pPScaler_Reg->PSCA_BLD0_CTRL1 = (((ptextStampParam->textStartX & 0x1FFF) << 16)|(ptextStampParam->textStartY & 0x1FFF));
		pPScaler_Reg->PSCA_BLD0_CTRL0 = (((ptextStampParam->textWidth & 0x1FFF) << 16)|(ptextStampParam->textHeight << 8)|(ptextStampParam->textBitMode << 1));
	}
	else
	{	// Second Setting
		pPScaler_Reg->PSCA_BLD1_ADDR = ptextStampParam->textSrcAddrs;
		pPScaler_Reg->PSCA_BLD1_CTRL1 = (((ptextStampParam->textStartX & 0x1FFF) << 16)|(ptextStampParam->textStartY & 0x1FFF));
		pPScaler_Reg->PSCA_BLD1_CTRL0 = (((ptextStampParam->textWidth & 0x1FFF) << 16)|(ptextStampParam->textHeight << 8)|(ptextStampParam->textBitMode << 1));
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_text_stamp_color_set:
*
*****************************************************/
INT32S drv_l1_pscaler_text_stamp_color_set(INT8U PScalerNum, TEXT_STAMP_COLOR_PARAM* ptextStampColorParam)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_BLD_COLOR01 = (((ptextStampColorParam->textColor1 & 0xFFFF) << 16)|(ptextStampColorParam->textColor0 & 0xFFFF));
	pPScaler_Reg->PSCA_BLD_COLOR23 = (((ptextStampColorParam->textColor3 & 0xFFFF) << 16)|(ptextStampColorParam->textColor2 & 0xFFFF));

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_text_stamp_enable:
*
*****************************************************/
INT32S drv_l1_pscaler_text_stamp_enable(INT8U PScalerNum, INT8U settingNum, INT8U enableCtrl)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(settingNum == TEXT_STAMP_1ST_SETTING)
	{
		if(enableCtrl)
		{
			pPScaler_Reg->PSCA_BLD0_CTRL0 |= 0x01;
		}
		else
		{
			pPScaler_Reg->PSCA_BLD0_CTRL0 &= ~0x01;
		}
	}
	else
	{
		if(enableCtrl)
		{
			pPScaler_Reg->PSCA_BLD1_CTRL0 |= 0x01;
		}
		else
		{
			pPScaler_Reg->PSCA_BLD1_CTRL0 &= ~0x01;
		}
	}

	return Ret_Err;

}

/*****************************************************
*	drv_l1_pscaler_start:
*
*****************************************************/
INT32S drv_l1_pscaler_start(INT8U PScalerNum)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

    //drv_l1_pscaler_H_first_enable(PScalerNum, 1);    //ming test H first
	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

   // pPScaler_Reg->PSCA_CTRL |= 0x04; // Bit0: Engine Enalbe
	pPScaler_Reg->PSCA_CTRL |= 0x01; // Bit0: Engine Enalbe


	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_input_fifo_restart:
*
*****************************************************/
INT32S drv_l1_pscaler_input_fifo_restart(INT8U PScalerNum)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_CTRL |= (0x01 << 1); // Bit1:Input Fifo Restart

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_output_fifo_restart:
*
*****************************************************/
INT32S drv_l1_pscaler_output_fifo_restart(INT8U PScalerNum)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_CTRL |= (0x01 << 3); // Bit3:Output Fifo Restart

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_stop:
*
*****************************************************/
INT32S drv_l1_pscaler_stop(INT8U PScalerNum)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_INTEN_CTRL = 0;
	pPScaler_Reg->PSCA_INT_FLAG = 0xFFFF; // Clear All Interrupt Flag: Write 1 clear

	pPScaler_Reg->PSCA_CTRL &= 0xFFFFFFFE;//= 0x00; // Bit0: Engine Enalbe
	pPScaler_Reg->PSCA_CTRL |= 0x4; //Bit2: Reset Engine status

	PScalerInterruptFlag[PScalerNum] = 0;

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_status_get:
*
*****************************************************/
INT32S drv_l1_pscaler_status_get(INT8U PScalerNum)
{
	INT32S Ret_Status;

	if(PScalerInterruptFlag[PScalerNum] == 0)
	{
		return PIPELINE_SCALER_STATUS_BUSY;
	}

	if(PScalerNum == PSCALER_A)
	{
		NVIC_DisableIRQ(PSCALER_A_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(PSCALER_B_IRQn);
	}
	Ret_Status = PScalerInterruptFlag[PScalerNum];
	PScalerInterruptFlag[PScalerNum] = 0;
	if(PScalerNum == PSCALER_A)
	{
		NVIC_EnableIRQ(PSCALER_A_IRQn);
	}
	else
	{
		NVIC_EnableIRQ(PSCALER_B_IRQn);
	}

	return Ret_Status;
}

/*****************************************************
*	drv_l1_pscaler_callback_register:
*
*****************************************************/
void drv_l1_pscaler_callback_register(INT8U PScalerNum,void (*PScaler_Callback)(INT32U PScaler_Event))
{
	PScaler_Isr_Callback[PScalerNum] = PScaler_Callback;
}

/*****************************************************
*	drv_l1_pscaler_HSYNC_latch_delay:
*	delayTCount: Zero to disable, nonzero to enable
*
*****************************************************/
INT32S drv_l1_pscaler_HSYNC_latch_delay(INT8U PScalerNum, INT32U delayTCount)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_H_PERIOD_CTRL = (delayTCount & 0x03FF);

	if(delayTCount > 0)
	{
		pPScaler_Reg->PSCA_H_PERIOD_CTRL |= (0x01 << 15); // Enable
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_1D_filter_set:
*
*****************************************************/
INT32S drv_l1_pscaler_1D_filter_set(INT8U PScalerNum, FILTER_1D_PARAM* pFilter1DParam)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_1D_FILTER_CTRL = 0;

	if(pFilter1DParam->Filter1DEnableY)
	{
		pPScaler_Reg->PSCA_1D_FILTER_CTRL |= (0x01 << 29);
	}

	if(pFilter1DParam->Filter1DEnableX)
	{
		pPScaler_Reg->PSCA_1D_FILTER_CTRL |= (pFilter1DParam->Filter1DXFactorA & 0x1F);

		pPScaler_Reg->PSCA_1D_FILTER_CTRL |= ((pFilter1DParam->Filter1DXFactorB & 0x1F) << 8);

		pPScaler_Reg->PSCA_1D_FILTER_CTRL |= ((pFilter1DParam->Filter1DXFactorC & 0x1F) << 16);

		pPScaler_Reg->PSCA_1D_FILTER_CTRL |= ((pFilter1DParam->Filter1DXFactorD & 0x07) << 24);

		pPScaler_Reg->PSCA_1D_FILTER_CTRL |= (0x01 << 28);
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_H_first_enable:
*
*****************************************************/
INT32S drv_l1_pscaler_H_first_enable(INT8U PScalerNum, INT8U hFirstEnable)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(hFirstEnable)
	{
		pPScaler_Reg->PSCA_CTRL |= (0x01 << 27);
	}
	else
	{
		pPScaler_Reg->PSCA_CTRL &= ~(0x01 << 27);
	}

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_register_refresh:
*
*****************************************************/
INT32S drv_l1_pscaler_register_refresh(INT8U PScalerNum, INT8U refreshMode)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	pPScaler_Reg->PSCA_CTRL |= (refreshMode << 30);

	return Ret_Err;
}

/*****************************************************
*	drv_l1_pscaler_register_refresh_status_get:
*
*****************************************************/
INT32S drv_l1_pscaler_register_refresh_status_get(INT8U PScalerNum)
{
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(pPScaler_Reg->PSCA_CTRL & (0x03U << 30))
	{
		return PIPELINE_SCALER_STATUS_BUSY;
	}
	else
	{
		return PIPELINE_SCALER_STATUS_REFRESH_DONE;
	}
}

/*****************************************************
*	drv_l1_pscaler_MB420_bypass_mode_set:
*
*****************************************************/
INT32S drv_l1_pscaler_MB420_bypass_mode_set(INT8U PScalerNum,INT8U bypassEnable)
{
	INT32S Ret_Err = PSCALER_RET_SUCCESS;
	PIPELINE_SCALER_SFR* pPScaler_Reg;

	pPScaler_Reg = get_PSCALER_Reg_Base(PScalerNum);

	if(bypassEnable)
	{
		pPScaler_Reg->PSCA_CTRL |= (0x01 << 15); // Enable MB420 Bypass Mode
		pPScaler_Reg->PSCA_CTRL |= (0x01 << 11); // Set Output Format [Bit 11] to 1
	}
	else
	{
		pPScaler_Reg->PSCA_CTRL &= ~(0x01 << 15);
		pPScaler_Reg->PSCA_CTRL &= ~(0x01 << 11);
	}

	return Ret_Err;
}

/*****************************************************
* drv_l1_pscaler_clk_ctrl:
*
*****************************************************/
void drv_l1_pscaler_clk_ctrl(INT8U PScalerNum,INT16U clkEnable)
{
	INT16U pscalerModule;

	if(PScalerNum == PSCALER_A)
	{
		pscalerModule = CLK_EN0_PSCA0;
	}
	else
	{
		pscalerModule = CLK_EN0_PSCA1;
	}

	drv_l1_clock_set_system_clk_en(pscalerModule,clkEnable);

}

