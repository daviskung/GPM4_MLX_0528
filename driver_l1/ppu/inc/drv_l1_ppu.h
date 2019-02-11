#ifndef __DRV_L1_PPU_H__
#define __DRV_L1_PPU_H__

#include "drv_l1.h"
#include "drv_l1_sfr.h"

#define NEW_PAL_RAM_16_EN                   0
#define	C_PPU_TEXT1							0
#define	C_PPU_TEXT2							1
#define	C_PPU_TEXT3							2
#define	C_PPU_TEXT4							3
#define R_PPU_SPRITE_X0					(*((volatile INT32U *) 0xD0500300))	// Sprite X0 register
#define R_PPU_SPRITE_Y0					(*((volatile INT32U *) 0xD0500304))	// Sprite Y0 register
#define R_PPU_SPRITE_X1					(*((volatile INT32U *) 0xD0500308))	// Sprite X1 register
#define R_PPU_SPRITE_Y1					(*((volatile INT32U *) 0xD050030C))	// Sprite Y1 register
#define R_PPU_SPRITE_X2					(*((volatile INT32U *) 0xD0500310))	// Sprite X2 register
#define R_PPU_SPRITE_Y2					(*((volatile INT32U *) 0xD0500314))	// Sprite Y2 register
#define R_PPU_SPRITE_X3					(*((volatile INT32U *) 0xD0500318))	// Sprite X3 register
#define R_PPU_SPRITE_Y3					(*((volatile INT32U *) 0xD050031C))	// Sprite Y3 register
#define R_PPU_SPRITE_W0					(*((volatile INT32U *) 0xD0500320))	// Sprite Word 1 register
#define R_PPU_SPRITE_W1					(*((volatile INT32U *) 0xD0500324))	// Sprite Word 2 register
#define R_PPU_SPRITE_W2					(*((volatile INT32U *) 0xD0500328))	// Sprite Word 5 register
#define R_PPU_SPRITE_W3					(*((volatile INT32U *) 0xD050032C))	// Sprite Word 6 register
#define R_PPU_SPRITE_W4					(*((volatile INT32U *) 0xD0500330))	// Sprite Word 7 register

#define PPU_SPRITE_25D_POSITION_CONVERT(x0,y0,x1,y1,x2,y2,x3,y3, w1,w2,w5,w6,w7) \
{\
	R_PPU_SPRITE_X0 = (INT32U) (x0);\
	R_PPU_SPRITE_Y0 = (INT32U) (y0);\
	R_PPU_SPRITE_X1 = (INT32U) (x1);\
	R_PPU_SPRITE_Y1 = (INT32U) (y1);\
	R_PPU_SPRITE_X2 = (INT32U) (x2);\
	R_PPU_SPRITE_Y2 = (INT32U) (y2);\
	R_PPU_SPRITE_X3 = (INT32U) (x3);\
	R_PPU_SPRITE_Y3 = (INT32U) (y3);\
	w1 = R_PPU_SPRITE_W0;\
	w2 = R_PPU_SPRITE_W1;\
	w5 = R_PPU_SPRITE_W2;\
	w6 = R_PPU_SPRITE_W3;\
	w7 = R_PPU_SPRITE_W4;\
}

#define	R_PPU_SPRITE_CTRL					(*((volatile INT32U *) 0xD0500108))
#define R_PPU_EXTENDSPRITE_CONTROL          (*((volatile INT32U *) 0xD0500334))
#define cdm_enable                          0x20000
#define ex_cdm_enable                       0x2
#define GPSP_CMD_COMPARE()                  (R_PPU_SPRITE_CTRL & cdm_enable)
#define GPEXSP_CMD_COMPARE()                (R_PPU_EXTENDSPRITE_CONTROL & ex_cdm_enable)

typedef struct {
	INT32U update_register_flag;			// This flag indicates which parts of the register sets should be updated
// Updated by DMA engine
#define C_UPDATE_REG_SET_PALETTE0			 0x01000000
#define C_UPDATE_REG_SET_PALETTE1			 0x02000000
#define C_UPDATE_REG_SET_PALETTE2			 0x04000000
#define C_UPDATE_REG_SET_PALETTE3			 0x08000000
#define C_UPDATE_REG_SET_HORIZONTAL_MOVE	 0x10000000
#define C_UPDATE_REG_SET_TEXT1_HCOMPRESS	 0x20000000
#define C_UPDATE_REG_SET_TEXT3_25D			 0x40000000
#define C_UPDATE_REG_SET_SPRITE_ATTRIBUTE	 0x80000000
#define C_UPDATE_REG_SET_SPRITE_EX_ATTRIBUTE 0x00800000
#define C_UPDATE_REG_SET_PALETTE15			 0x00400000
#define C_UPDATE_REG_SET_PALETTE14			 0x00200000
#define C_UPDATE_REG_SET_PALETTE13			 0x00100000
#define C_UPDATE_REG_SET_PALETTE12			 0x00080000
#define C_UPDATE_REG_SET_PALETTE11			 0x00040000
#define C_UPDATE_REG_SET_PALETTE10			 0x00020000
#define C_UPDATE_REG_SET_PALETTE9			 0x00010000
#define C_UPDATE_REG_SET_PALETTE8			 0x00008000
#define C_UPDATE_REG_SET_PALETTE7			 0x00004000
#define C_UPDATE_REG_SET_PALETTE6			 0x00002000
#define C_UPDATE_REG_SET_PALETTE5			 0x00001000
#define C_UPDATE_REG_SET_PALETTE4			 0x00000800
#define C_UPDATE_REG_SET_DMA_MASK			 0xFFFFF800
// Updated by CPU
#define C_UPDATE_REG_SET_PPU				0x00000001
#define C_UPDATE_REG_SET_TEXT1				0x00000002
#define C_UPDATE_REG_SET_TEXT2				0x00000004
#define C_UPDATE_REG_SET_TEXT3				0x00000008
#define C_UPDATE_REG_SET_TEXT4				0x00000010
#define C_UPDATE_REG_SET_SPRITE				0x00000020

	// Registers moved by PPU DMA engine
	INT32U ppu_palette0_ptr;				// Updated when C_UPDATE_REG_SET_PALETTE0 is set in update_register_flag
	INT32U ppu_palette1_ptr;				// Updated when C_UPDATE_REG_SET_PALETTE1 is set in update_register_flag
	INT32U ppu_palette2_ptr;				// Updated when C_UPDATE_REG_SET_PALETTE2 is set in update_register_flag
	INT32U ppu_palette3_ptr;				// Updated when C_UPDATE_REG_SET_PALETTE3 is set in update_register_flag
	// add
	INT32U ppu_new_palette_ptr[12];			// Updated when C_UPDATE_REG_SET_PALETTE0 is set in update_register_flag
    INT32U ppu_horizontal_move_ptr;			// Updated when C_UPDATE_REG_SET_HORIZONTAL_MOVE is set in update_register_flag
	INT32U ppu_text1_hcompress_ptr;			// Updated when C_UPDATE_REG_SET_TEXT1_HCOMPRESS is set in update_register_flag
	INT32U ppu_text3_25d_ptr;				// Updated when C_UPDATE_REG_SET_TEXT3_25D is set in update_register_flag
	INT32U ppu_sprite_attribute_ptr;		// Updated when C_UPDATE_REG_SET_SPRITE_ATTRIBUTE is set in update_register_flag
    INT32U ppu_sprite_ex_attribute_ptr;     // Updated when C_UPDATE_REG_SET_SPRITE_EX_ATTRIBUTE is set in update_register_flag

    // PPU relative registers. Updated when C_UPDATE_REG_SET_PPU is set in update_register_flag
	INT16U ppu_blending_level;				// R_PPU_BLENDING
	INT16U ppu_fade_control;				// R_PPU_FADE_CTRL
	INT32U ppu_palette_control;				// R_PPU_PALETTE_CTRL
	INT32U ppu_rgb565_transparent_color;	// R_PPU_BLD_COLOR
	INT32U ppu_window1_x;					// R_PPU_WINDOW0_X
	INT32U ppu_window1_y;					// R_PPU_WINDOW0_Y
	INT32U ppu_window2_x;					// R_PPU_WINDOW1_X
	INT32U ppu_window2_y;					// R_PPU_WINDOW1_Y
	INT32U ppu_window3_x;					// R_PPU_WINDOW2_X
	INT32U ppu_window3_y;					// R_PPU_WINDOW2_Y
	INT32U ppu_window4_x;					// R_PPU_WINDOW3_X
	INT32U ppu_window4_y;					// R_PPU_WINDOW3_Y
	INT32U ppu_enable;						// R_PPU_ENABLE
	INT32U ppu_misc;                        // R_PPU_MISC
	INT32U ppu_free_mode;                   // R_FREE_SIZE
	INT32U ppu_special_effect_rgb;			// R_PPU_RGB_OFFSET
    INT32U ppu_ui_enable;			        // R_PPU_UI_ENABLE
    INT32U ppu_ui_fbi_addr;			        // R_PPU_UI_FBI_ADDR
	INT32U ppu_frame_buffer_fifo;           // R_PPU_FBO_FIFO
    INT32U buffer_user_define;
    INT32U ppu_deflicker_para;

	// TEXT relative registers
	struct ppu_text_register_sets {
		INT16U position_x;					// R_PPU_TEXT1_X_OFFSET
		INT16U position_y;					// R_PPU_TEXT1_Y_OFFSET
		INT16U offset_x;					// R_PPU_TEXT1_X_OFFSET
		INT16U offset_y;					// R_PPU_TEXT1_Y_OFFSET
		INT32U attribute;					// R_PPU_TEXT1_ATTRIBUTE
		INT32U control;						// R_PPU_TEXT1_CTRL
		INT32U n_ptr;						// P_PPU_TEXT1_N_PTR
		INT32U a_ptr;						// P_PPU_TEXT1_A_PTR
		INT16U sine;						// R_PPU_TEXT1_SINE
		INT16U cosine;						// R_PPU_TEXT1_COSINE
		INT32U segment;						// R_PPU_TEXT1_SEGMENT
	} text[4];
	INT16U ppu_vcompress_value;				// R_PPU_VCOMP_VALUE
	INT16U ppu_vcompress_offset;			// R_PPU_VCOMP_OFFSET
	INT16U ppu_vcompress_step;				// R_PPU_VCOMP_STEP
	INT16U text3_25d_y_compress;			// R_PPU_Y25D_COMPRESS

	// Sprite relative registers. Updated when C_UPDATE_REG_SET_SPRITE is set in update_register_flag
	INT32U sprite_control;					// R_PPU_SPRITE_CTRL
	INT32U sprite_segment;					// R_PPU_SPRITE_SEGMENT
	INT32U extend_sprite_control;			// R_PPU_EXTENDSPRITE_CONTROL
	INT32U extend_sprite_addr;				// R_PPU_EXTENDSPRITE_ADDR
} PPU_REGISTER_SETS, *PPU_REGISTER_SETS_PTR;

typedef struct {
	INT16U	nCharNumLo_16;
	INT16S	uPosX_16;
	INT16S	uPosY_16;
	INT16U	attr0;
	INT16U	attr1;
	INT16U	uX1_16;
	INT16U	uX2_16;
	INT16U	uX3_16;
} SpN_RAM, *PSpN_RAM;

typedef struct {
	INT16S	ex_attr0;                           //  0.  Sprite Extend Attribute 0
	INT16U	ex_attr1;                           //  1.  Sprite Extend Attribute 1
} SpN_EX_RAM, *PSpN_EX_RAM;

typedef struct {
	INT16S x0;
	INT16S y0;
	INT16S x1;
	INT16S y1;
	INT16S x2;
	INT16S y2;
	INT16S x3;
	INT16S y3;
} POS_STRUCT, *POS_STRUCT_PTR;

typedef struct {
	FP32 x0;
	FP32 y0;
	FP32 x1;
	FP32 y1;
	FP32 x2;
	FP32 y2;
	FP32 x3;
	FP32 y3;
} POS_STRUCT_GP32XXX, *POS_STRUCT_PTR_GP32XXX;

typedef struct {
  INT8U group_id;
  POS_STRUCT_GP32XXX V3D_POS1;
  POS_STRUCT V3D_POS2;
} V3D_POS_STRUCT, *V3D_POS_STRUCT_PTR;

typedef struct {
	INT32U cdm0;
	INT32U cdm1;
	INT32U cdm2;
	INT32U cdm3;
} CDM_STRUCT, *CDM_STRUCT_PTR;

typedef struct {
	INT16S	nCharNum;                           //  0.  Character Number Set
	INT16S	nPosX;                              //  1.  Sprite 2D X Position or Sprite Virtual 3D X0 Position
	INT16S	nPosY;                              //  2.  Sprite 2D Y Position or Sprite Virtual 3D Y0 Position
	INT16S	nSizeX;                             //  3.  Width of Sprite Image
	INT16S	nSizeY;                             //  4.  Heigh of Sprite Image
	INT16U	uAttr0;                             //  5.  Sprite Attribute 0
	INT16U	uAttr1;                             //  6.  Sprite Attribute 1
	INT16S	nPosX1;                             //  7.  Sprite Virtual 3D X1 Position
	INT16S	nPosX2;                             //  8.  Sprite Virtual 3D X1 Position
	INT16S	nPosX3;                             //  9.  Sprite Virtual 3D X1 Position
	INT16S	nPosY1;                             //  10. Sprite Virtual 3D Y1 Position
	INT16S	nPosY2;                             //  11. Sprite Virtual 3D Y2 Position
	INT16S	nPosY3;                             //  12. Sprite Virtual 3D Y3 Position
	const INT32U	*SpCell;                    //  13. Sprite Attribute Start Pointer
} SPRITE, *PSPRITE;

// PPU function control
#define C_PPU_DRV_MAX_FRAME_NUM				4
#define C_HBLANK_QUEUE_MAX		            10
#define HBLANK_IEQ_END                      0x8001
#define HBLANK_FRAME_END                    0x8002
#define PPU_FRAME_END                       0x0001
#define SP_DMA_END                          0x1001
#define PPU_FRAME_REGISTER_WAIT             0

// PPU control register
#define C_PPU_CTRL_ENABLE					0x00000001
#define C_PPU_CTRL_VGA_MODE					0x00000010
#define C_PPU_CTRL_NON_INTERLACE_MODE		0x00000020
#define C_PPU_CTRL_FRAME_MODE				0x00000080

// PPU frame mode control register
#define C_PPU_FRAME_TFT_UPDATED				0x00002000
#define C_PPU_FRAME_OUTPUT_FIELD			0x00004000
#define C_PPU_FRAME_TV_UPDATED				0x00008000

// PPU sprite control register
#define C_PPU_SPRITE_COLOR_DITHER_MODE		0x00020000

// Interrupt control register
#define C_PPU_INT_EN_PPU_VBLANK				0x00000001
#define C_PPU_INT_EN_VIDEO_POSITION			0x00000002
#define C_PPU_INT_EN_DMA_COMPLETE			0x00000004
#define C_PPU_INT_EN_PALETTE_ERROR			0x00000008
#define C_PPU_INT_EN_TEXT_UNDERRUN			0x00000010
#define C_PPU_INT_EN_SPRITE_UNDERRUN		0x00000020
#define C_PPU_INT_EN_SENSOR_FRAME_END		0x00000040
#define C_PPU_INT_EN_MOTION_DETECT			0x00000080
#define C_PPU_INT_EN_SENSOR_POSITION_HIT	0x00000100
#define C_PPU_INT_EN_MOTION_UNDERRUN		0x00000200
#define C_PPU_INT_EN_TV_UNDERRUN			0x00000400
#define C_PPU_INT_EN_TV_VBLANK				0x00000800
#define C_PPU_INT_EN_TFT_UNDERRUN			0x00001000
#define C_PPU_INT_EN_TFT_VBLANK				0x00002000
#define C_PPU_INT_EN_PPU_HBLANK				0x00004000

#define C_PPU_INT_EN_PPU_MASK				0x00007C3D

// Interrupt pending register
#define C_PPU_INT_PEND_PPU_VBLANK			0x00000001
#define C_PPU_INT_PEND_VIDEO_POSITION		0x00000002
#define C_PPU_INT_PEND_DMA_COMPLETE			0x00000004
#define C_PPU_INT_PEND_PALETTE_ERROR		0x00000008
#define C_PPU_INT_PEND_TEXT_UNDERRUN		0x00000010
#define C_PPU_INT_PEND_SPRITE_UNDERRUN		0x00000020
#define C_PPU_INT_PEND_SENSOR_FRAME_END		0x00000040
#define C_PPU_INT_PEND_MOTION_DETECT		0x00000080
#define C_PPU_INT_PEND_SENSOR_POSITION_HIT	0x00000100
#define C_PPU_INT_PEND_MOTION_UNDERRUN		0x00000200
#define C_PPU_INT_PEND_TV_UNDERRUN			0x00000400
#define C_PPU_INT_PEND_TV_VBLANK			0x00000800
#define C_PPU_INT_PEND_TFT_UNDERRUN			0x00001000
#define C_PPU_INT_PEND_TFT_VBLANK			0x00002000
#define C_PPU_INT_PEND_PPU_HBLANK			0x00004000
#define C_PPU_INT_PEND_PPU_MASK				0x00007C3D

// Definitions for driver control and status
#define C_PPU_CONTROL_LINE_MODE_UPDATE		0x0001
#define C_PPU_CONTROL_WAIT_FRAME_DONE		0x0002
#define C_PPU_STATUS_FRAME_MODE_BUSY		0x0001

#define PPU_EN_HBLANK                       0x8000
#define PPU_HBLANK_OFFSET_MASK              0x3FF

/*
* Function Name :  ppu_init
*
* Syntax : void ppu_init(void);
*
* Purpose :  Initialize global variables and buffers. Register De-flicker and PPU ISR
*
* Parameters : <IN> none
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void ppu_init(void);
/*
* Function Name :  ppu_display_resolution_lock
*
* Syntax : void ppu_display_resolution_lock(void);
*
* Purpose :  Enable frame buffer lock function
*
* Parameters : <IN> none
*              <OUT> none
* Return : none
*
* Note : TV and TFT can have different resolution
*
*/
extern void ppu_display_resolution_lock(void);
/*
* Function Name :  ppu_display_resolution_unlock
*
* Syntax : void ppu_display_resolution_unlock(void);
*
* Purpose :  Disable frame buffer lock function
*
* Parameters : <IN> none
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void ppu_display_resolution_unlock(void);
/*
* Function Name :  ppu_frame_buffer_add
*
* Syntax : INT32S ppu_frame_buffer_add(INT32U *frame_buf);
*
* Purpose :  Add frame buffer by sending a message to free frame buffer queue
*
* Parameters : <IN> frame_buf: Pointer of the frame buffer
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S ppu_frame_buffer_add(INT32U *frame_buf);
/*
* Function Name :  ppu_frame_buffer_get
*
* Syntax : INT32S ppu_frame_buffer_get(void);
*
* Purpose :  Get a pointer of the frame buffer index which is available
*
* Parameters : <IN> none
*              <OUT> none
* Return : INT32S:	Pointer of the frame buffer
*
* Note :
*
*/
extern INT32S ppu_frame_buffer_get(void);
/*
* Function Name :  ppu_frame_buffer_display
*
* Syntax : INT32S ppu_frame_buffer_display(INT32U *frame_buf);
*
* Purpose :  Set the pointer of the frame buffer index which is ready to display
*
* Parameters : <IN> frame_buf: Pointer of the frame buffer
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S ppu_frame_buffer_display(INT32U *frame_buf);
/*
* Function Name :  ppu_tft_static_frame_set
*
* Syntax : INT32S ppu_tft_static_frame_set(INT32U *frame_buf);
*
* Purpose :  Set the pointer of the frame buffer index which is ready to tft display
*
* Parameters : <IN> frame_buf: Pointer of the frame buffer
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note : If this function is used,ppu_frame_buffer_add is invalid.
*
*/
extern INT32S ppu_tft_static_frame_set(INT32U *frame_buf);
/*
* Function Name :  ppu_tv_static_frame_set
*
* Syntax : INT32S ppu_tv_static_frame_set(INT32U *frame_buf);
*
* Purpose :  Set the pointer of the frame buffer index which is ready to tv display
*
* Parameters : <IN> frame_buf: Pointer of the frame buffer
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note : If this function is used,ppu_frame_buffer_add is invalid.
*
*/
extern INT32S ppu_tv_static_frame_set(INT32U *frame_buf);
/*
* Function Name :  ppu_deflicker_mode_set
*
* Syntax : INT32S ppu_deflicker_mode_set(INT32U value);
*
* Purpose :  Enable/Disable deflicker
*
* Parameters : <IN> value 1: Enable 0: Disable
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S ppu_deflicker_mode_set(INT32U value);
/*
* Function Name :  ppu_go
*
* Syntax : INT32S ppu_go(PPU_REGISTER_SETS *p_register_set, INT32U wait_start, INT32U wait_done);
*
* Purpose :  Update PPU registers. User can choose the timing of return
*
* Parameters : <IN> *p_register_set: PPU releated parameters
*										wait_available: Wait until PPU is available or not. 1: Wait, 0: Do not wait
*										wait_done: Wait until current PPU operation is done. 1: Wait, 0: Do not wait
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S ppu_go(PPU_REGISTER_SETS *p_register_set, INT32U wait_start, INT32U wait_done);
/*
* Function Name :  ppu_frame_mode_busy_get
*
* Syntax : INT8U ppu_frame_mode_busy_get(void);
*
* Purpose :  Get the status of PPU frame buffer(busy or not)
*
* Parameters : <IN> none
*              <OUT> none
* Return : INT8U: 1: Busy
*									0: Idle
*
* Note :
*
*/
extern INT8U ppu_frame_mode_busy_get(void);
/*
* Function Name :  ppu_notifier_register
*
* Syntax : INT32S ppu_notifier_register(void (*notifier)(void));
*
* Purpose :  Set ppu_done_notifier through a hock function
*
* Parameters : <IN> *notifier: The function which will be execute after PPU operation is done
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S ppu_notifier_register(void (*notifier)(void));
/*
* Function Name :  ppu_notifier_unregister
*
* Syntax : void ppu_notifier_unregister(void);
*
* Purpose :  Clear ppu_done_notifier
*
* Parameters : <IN> none
*              <OUT> none
* Return : none
*
* Note :
*
*/
extern void ppu_notifier_unregister(void);
/*
* Function Name :  set_ppu_free_control
*
* Syntax : void set_ppu_free_control(INT8U INTL,INT16U H_size,INT16U V_size);
*
* Purpose :  Enable and set PPU free mode. max resolution is 1024*1024
*
* Parameters : <IN> INTL:		0 => Non-interlace free mode. (For TFT type application)
*														1 => Interlace free mode. (For TV type application)
*										H_size:	PPU output resolution Horizontal length(This value must be a multiply of 16)
*										V_size: PPU output resolution Vertical length
*              <OUT> none
* Return : none
	*
* Note :
*
*/
extern void set_ppu_free_control(INT8U INTL,INT16U H_size,INT16U V_size);
/*
* Function Name :  Hblenk_Enable_Set
*
* Syntax : INT32S Hblenk_Enable_Set(INT32U value);
*
* Purpose :  Enable/Disable Horizontal blank
*
* Parameters : <IN> value: 1:Enable 0: Disable
*              <OUT> none
* Return : none
	*
* Note :
*
*/
extern INT32S Hblenk_Enable_Set(INT32U value);
/*
* Function Name :  Hblenk_Line_Offset_Set
*
* Syntax : void Hblenk_Line_Offset_Set(INT32U line_offset);
*
* Purpose :  set Horizontal blank stop line
*
* Parameters : <IN> line_offset: Horizontal blank stop line
*              <OUT> none
* Return : none
	*
* Note : PPU will process from start of the frame and stop before this line.
*
*/
extern void Hblenk_Line_Offset_Set(INT32U line_offset);
/*
* Function Name :  Hblenk_Go
*
* Syntax : void Hblenk_Go(void);
*
* Purpose :  start Horizontal blank
*
* Parameters : <IN> none
*              <OUT> none
* Return : none
	*
* Note :
*
*/
extern void Hblenk_Go(void);
/*
* Function Name :  Hblenk_Wait
*
* Syntax : INT32S Hblenk_Wait(void);
*
* Purpose :  Wait Horizontal blank end
*
* Parameters : <IN> none
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S Hblenk_Wait(void);
/*
* Function Name :  set_ppu_go
*
* Syntax : INT32S set_ppu_go(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  Start PPU engine
*
* Parameters : <IN> *p_register_set: PPU releated parameters
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S set_ppu_go(PPU_REGISTER_SETS *p_register_set);
/*
* Function Name :  ppu_setting_update
*
* Syntax : INT32S ppu_setting_update(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  Update PPU related register
*
* Parameters : <IN> *p_register_set: PPU releated parameters
*              <OUT> none
* Return : INT32S: 0: Success
*									-1: Fail
*
* Note :
*
*/
extern INT32S ppu_setting_update(PPU_REGISTER_SETS *p_register_set);

extern INT32S fd_lock_set(INT32U color1, INT32U size1, INT32U color2, INT32U size2);

// extern function
extern INT32S drv_l2_ppu_isr_callback_set(void (*callback)(INT32U ppu_event));
#endif		// __DRV_L1_PPU_H__
