
#ifndef __drv_l1_PSCALER_H__
#define __drv_l1_PSCALER_H__

/******************************************************/
#include "project.h"

/******************************************************/
#define PSCALER_RET_SUCCESS		0
#define PSCALER_RET_FAIL		1

/******************************************************/
//+++ Interrupt
#define PIPELINE_SCALER_INT_ENABLE_DISABLE					0x00000000
#define PIPELINE_SCALER_INT_ENABLE_AHB_READ_DONE			0x00000001
#define PIPELINE_SCALER_INT_ENABLE_422GP420_FRAME_END		0x00000002
#define PIPELINE_SCALER_INT_ENABLE_MB420_FRAME_END			0x00000004
#define PIPELINE_SCALER_INT_ENABLE_EOF_CLEAR				0x00000008
#define PIPELINE_SCALER_INT_ENABLE_422GP420_CHG_FIFO		0x00000010
#define PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_A			0x00000020
#define PIPELINE_SCALER_INT_ENABLE_422GP420_BUF_B			0x00000040
#define PIPELINE_SCALER_INT_ENABLE_OVERFLOW					0x00000080
#define PIPELINE_SCALER_INT_ENABLE_AHB_IN_FIFO_DONE			0x00000100
#define PIPELINE_SCALER_INT_ENABLE_TEXT_STAMP_AHB_RD_DONE	0x00000200
#define PIPELINE_SCALER_INT_ENABLE_TEXT_STAMP_DONE			0x00000400
#define PIPELINE_SCALER_INT_ENABLE_MB420_BUF_A				0x00001000
#define PIPELINE_SCALER_INT_ENABLE_MB420_BUF_B				0x00002000
#define PIPELINE_SCALER_INT_ENABLE_ALL						0x000037FF

//+++ Input Format
#define PIPELINE_SCALER_INPUT_FORMAT_YUYV			0x00000000
#define PIPELINE_SCALER_INPUT_FORMAT_YVYU			0x00000001
#define PIPELINE_SCALER_INPUT_FORMAT_UYVY			0x00000002
#define PIPELINE_SCALER_INPUT_FORMAT_VYUY			0x00000003
#define PIPELINE_SCALER_INPUT_FORMAT_GP420			0x00000004  // Notice: Only For PSCALER_1

//+++ Output Format
#define PIPELINE_SCALER_OUTPUT_FORMAT_YUYV			0x00000000
#define PIPELINE_SCALER_OUTPUT_FORMAT_YVYU			0x00000001
#define PIPELINE_SCALER_OUTPUT_FORMAT_UYVY			0x00000002
#define PIPELINE_SCALER_OUTPUT_FORMAT_VYUY			0x00000003
#define PIPELINE_SCALER_OUTPUT_FORMAT_RGB565		0x00000004
#define PIPELINE_SCALER_OUTPUT_FORMAT_GP420			0x00000006
#define PIPELINE_SCALER_OUTPUT_FORMAT_MB420			0x00000007

//+++ Input Source
#define PIPELINE_SCALER_INPUT_SOURCE_CDSP			0x00000000
#define PIPELINE_SCALER_INPUT_SOURCE_CSI			0x00000001
#define PIPELINE_SCALER_INPUT_SOURCE_PPUFB			0x00000002
#define PIPELINE_SCALER_INPUT_SOURCE_MB420			0x00000003
#define PIPELINE_SCALER_INPUT_SOURCE_DRAM			0x00000004

//+++ Status
#define PIPELINE_SCALER_STATUS_BUSY							0x00000000
#define PIPELINE_SCALER_STATUS_FRAME_DONE					0x00000002
#define PIPELINE_SCALER_STATUS_MB420_FRAME_DONE				0x00000004
#define PIPELINE_SCALER_STATUS_BUF_A_DONE					0x00000008
#define PIPELINE_SCALER_STATUS_BUF_B_DONE					0x00000010
#define PIPELINE_SCALER_STATUS_MB420_BUF_A_DONE				0x00000020
#define PIPELINE_SCALER_STATUS_MB420_BUF_B_DONE				0x00000040
#define PIPELINE_SCALER_STATUS_OVERFLOW_OCCUR				0x00000080
#define PIPELINE_SCALER_STATUS_INPUT_EMPTY					0x00000100
#define PIPELINE_SCALER_STATUS_REFRESH_DONE					0x00000200

/******************************************************/
typedef enum
{
    PSCALER_A,                 
    PSCALER_B,
    PSCALER_MAX
}PIPELINE_SCALER_NUM;

typedef struct 
{												// Offset
	volatile INT32U	PSCA_CTRL;         			// 0x0000
	volatile INT32U	PSCA_OUTBND_COLOR;     		// 0x0004
	volatile INT32U	PSCA_OUT_X_WIDTH;          	// 0x0008
	volatile INT32U	PSCA_OUT_Y_WIDTH;          	// 0x000C
	volatile INT32U	PSCA_X_FACTOR;          	// 0x0010
	volatile INT32U	PSCA_Y_FACTOR;          	// 0x0014
	volatile INT32U	PSCA_X_START;	          	// 0x0018
	volatile INT32U	PSCA_Y_START;   	       	// 0x001C
	volatile INT32U	PSCA_IN_X_WIDTH;          	// 0x0020
	volatile INT32U	PSCA_IN_Y_WIDTH;   	       	// 0x0024
	volatile INT32U	PSCA_OUT_A_ADDR;          	// 0x0028
	volatile INT32U	PSCA_OUT_B_ADDR;   	       	// 0x002C
	volatile INT32U	PSCA_HEIGHT_LINE;  	       	// 0x0030
	volatile INT32U	PSCA_1D_FILTER_CTRL;       	// 0x0034
	volatile INT32U	PSCA_H_PERIOD_CTRL;       	// 0x0038
	volatile INT32U	RESERVED_1;        			// 0x003C
	volatile INT32U	PSCA_OUT_A_ADDR_MB420; 		// 0x0040
	volatile INT32U	PSCA_OUT_B_ADDR_MB420; 		// 0x0044
	volatile INT32U	PSCA_IN_ADDR;		 		// 0x0048	
	volatile INT32U	RESERVED_2; 	   			// 0x004C
	volatile INT32U	PSCA_BLD0_CTRL0; 			// 0x0050
	volatile INT32U	PSCA_BLD0_CTRL1; 			// 0x0054
	volatile INT32U	PSCA_BLD0_ADDR; 			// 0x0058
	volatile INT32U	RESERVED_3;	    			// 0x005C
	volatile INT32U	PSCA_BLD1_CTRL0; 			// 0x0060
	volatile INT32U	PSCA_BLD1_CTRL1; 			// 0x0064
	volatile INT32U	PSCA_BLD1_ADDR; 			// 0x0068
	volatile INT32U	RESERVED_4;	    			// 0x006C
	volatile INT32U	PSCA_BLD_COLOR01; 			// 0x0070
	volatile INT32U	PSCA_BLD_COLOR23; 			// 0x0074
	volatile INT32U	RESERVED_5[2];    			// 0x0078	
	volatile INT32U	PSCA_INTEN_CTRL; 			// 0x0080
	volatile INT32U	PSCA_INT_FLAG;	 			// 0x0084
	volatile INT32U	PSCA_STATUS;	 			// 0x0088
	volatile INT32U	PSCA_STATUS_LINEA_CNT;		// 0x008C
	volatile INT32U	PSCA_STATUS_LINEB_CNT;		// 0x0090
	volatile INT32U	PSCA_STATUS_LINE_CNT;		// 0x0094
	volatile INT32U	PSCA_STATUS_CNT_FRAME;		// 0x0098
}PIPELINE_SCALER_SFR;

/******************************************************/
typedef enum
{
    TEXT_STAMP_Y_LINES_16,                 
    TEXT_STAMP_Y_LINES_32,
    TEXT_STAMP_Y_LINES_64,                 
    TEXT_STAMP_Y_LINES_128    
}TEXT_STAMP_Y_LINES;

typedef enum
{
    TEXT_STAMP_DISABLE,                 
    TEXT_STAMP_ENABLE
}TEXT_STAMP_ACTION;

typedef enum
{
    TEXT_STAMP_1ST_SETTING,                 
    TEXT_STAMP_2ND_SETTING
}TEXT_STAMP_GROUP;

typedef enum
{
    TEXT_STAMP_1BIT_MODE,                 
    TEXT_STAMP_2BIT_MODE
}TEXT_STAMP_BITMODE;

typedef struct 
{
	INT32U textStartX;			// Can,t be 0
	INT32U textStartY;			// Can,t be 0
	INT32U textWidth;			// 16 alignment
	INT32U textHeight;			// Refter to TEXT_STAMP_Y_LINES
	INT32U textSrcAddrs; 
	INT8U  textBitMode; 		// 0:1bit 1:2bit
	INT8U  textStampSet;	 	// 0:First Setting  1:Second Setting  	
}TEXT_STAMP_PARAM;

typedef struct 
{
	INT16U textColor0;	
	INT16U textColor1;	
	INT16U textColor2;	
	INT16U textColor3;		
}TEXT_STAMP_COLOR_PARAM;

typedef enum
{
    PSCALER_IN_FIFO_MODE_RELOAD_ADDRS,                 
    PSCALER_IN_FIFO_MODE_CONTINUE_ADDRS,                 
}PIPELINE_SCALER_IN_FIFO_MODE;

typedef enum
{
    PSCALER_IN_FIFO_FRAME_MODE,                 
    PSCALER_IN_FIFO_8_LINES,
    PSCALER_IN_FIFO_16_LINES,
    PSCALER_IN_FIFO_32_LINES,
    PSCALER_IN_FIFO_64_LINES,
    PSCALER_IN_FIFO_128_LINES,   
    PSCALER_IN_FIFO_256_LINES   
}PIPELINE_SCALER_IN_FIFO_LINES;

typedef struct 
{
	INT8U Filter1DEnableY;  // 0:Disable 1:Enable
	INT8U Filter1DEnableX;	// 0:Disable 1:Enable
	INT8S Filter1DXFactorA;	// -15~15 
	INT8S Filter1DXFactorB;	// -15~15
	INT8S Filter1DXFactorC;	// -15~15
	INT8U Filter1DXFactorD;	// 0~5  A+B+C=2^D
}FILTER_1D_PARAM;

typedef enum
{
    REG_REFRESH_NOW = 1U,                 
    REG_REFRESH_NEXT_FRAME
}REG_REFRESH_PARAM;

/******************************************************/
extern void drv_l1_pscaler_init(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_input_pixels_set(INT8U PScalerNum, INT32U inWidth, INT32U inHeight);
extern INT32S drv_l1_pscaler_output_pixels_set(INT8U PScalerNum, INT32U widthFactor, INT32U outWidth, INT32U heightFactor, INT32U outHeight);
extern INT32S drv_l1_pscaler_input_X_start_set(INT8U PScalerNum, INT32U xOffset);
extern INT32S drv_l1_pscaler_input_Y_start_set(INT8U PScalerNum, INT32U yOffset);
extern INT32S drv_l1_pscaler_output_A_buffer_set(INT8U PScalerNum, INT32U bufAddr); // bufAddr: 4-byte alignment
extern INT32U drv_l1_pscaler_output_A_buffer_get(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_output_B_buffer_set(INT8U PScalerNum, INT32U bufAddr); // bufAddr: 4-byte alignment
extern INT32U drv_l1_pscaler_output_B_buffer_get(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_output_fifo_line_set(INT8U PScalerNum, INT32U fifoLine, INT8U outStopEnable); // Only Support 422/GP420 , Not Support MB420
extern INT32S drv_l1_pscaler_input_fifo_line_set(INT8U PScalerNum, INT8U addrsMode, INT8U fifoLine);
extern INT32S drv_l1_pscaler_interrupt_set(INT8U PScalerNum, INT32U intEnVale);
extern INT32S drv_l1_pscaler_input_fFormat_set(INT8U PScalerNum, INT32U inFormat);
extern INT32S drv_l1_pscaler_output_fFormat_set(INT8U PScalerNum, INT32U outFormat); 
extern INT32S drv_l1_pscaler_input_source_set(INT8U PScalerNum, INT32U inSource);
extern INT32S drv_l1_pscaler_input_buffer_set(INT8U PScalerNum, INT32U bufAddr);

extern INT32S drv_l1_pscaler_H264_output_A_buffer_set(INT8U PScalerNum, INT32U bufAddr); // bufAddr: 4-byte alignment
extern INT32U drv_l1_pscaler_H264_output_A_buffer_get(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_H264_output_B_buffer_set(INT8U PScalerNum, INT32U bufAddr); // bufAddr: 4-byte alignment
extern INT32U drv_l1_pscaler_H264_output_B_buffer_get(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_H264_mirror_set(INT8U PScalerNum, INT32U mirrorEnable);
extern INT32S drv_l1_pscaler_H264_flip_set(INT8U PScalerNum, INT32U flipEnable);

extern INT32S drv_l1_pscaler_outboundary_color_set(INT8U PScalerNum, INT8U outBoundEnable, INT32U outBoundColor);
extern INT32S drv_l1_pscaler_output_pixels_X_offset_set(INT8U PScalerNum, INT32U outOffsetX); // Only Support 422/GP420 , Not Support MB420

extern INT32S drv_l1_pscaler_text_stamp_set(INT8U PScalerNum, TEXT_STAMP_PARAM* ptextStampParam);
extern INT32S drv_l1_pscaler_text_stamp_color_cet(INT8U PScalerNum, TEXT_STAMP_COLOR_PARAM* ptextStampColorParam);
extern INT32S drv_l1_pscaler_text_stamp_enable(INT8U PScalerNum, INT8U settingNum, INT8U enableCtrl);

extern INT32S drv_l1_pscaler_start(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_input_fifo_restart(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_output_fifo_restart(INT8U PScalerNum);
extern INT32S drv_l1_pscaler_stop(INT8U PScalerNum);

extern INT32S drv_l1_pscaler_status_get(INT8U PScalerNum);
extern void drv_l1_pscaler_callback_register(INT8U PScalerNum,void (*PScaler_Callback)(INT32U PScaler_Event));

extern INT32S drv_l1_pscaler_HSYNC_latch_delay(INT8U PScalerNum, INT32U delayTCount);
extern INT32S drv_l1_pscaler_1D_filter_set(INT8U PScalerNum, FILTER_1D_PARAM* pFilter1DParam);
extern INT32S drv_l1_pscaler_H_first_enable(INT8U PScalerNum, INT8U hFirstEnable);

extern INT32S drv_l1_pscaler_register_refresh(INT8U PScalerNum, INT8U refreshMode);
extern INT32S drv_l1_pscaler_register_refresh_status_get(INT8U PScalerNum);

extern INT32S drv_l1_pscaler_MB420_bypass_mode_set(INT8U PScalerNum,INT8U bypassEnable);

extern void drv_l1_pscaler_clk_ctrl(INT8U PScalerNum,INT16U clkEnable);
/******************************************************/

#endif // __drv_l1_PSCALER_H__

