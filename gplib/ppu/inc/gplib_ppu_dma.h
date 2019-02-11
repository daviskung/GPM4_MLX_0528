#include "gplib.h"

#define COLOR_RGB565			0
#define COLOR_VYUY				1
#define COLOR_YVYU				2
#define COLOR_UYVY				3
#define COLOR_YUYV				4
#define COLOR_BGRG				5
#define COLOR_GBGR				6
#define COLOR_RGBG				7
#define COLOR_GRGB				8
#define COLOR_RGBA				9
#define NO_FLIP					0
#define H_FLIP					1
#define V_FLIP					2
#define HV_FLIP					3

typedef struct ppu_buffer_info_s{
	unsigned int buffer_color_mode;
	unsigned int transparent_color;
	unsigned int transparent_enable;
	unsigned int transparent_mode;
	unsigned int blend_enable;
	unsigned int blend_value;
	unsigned int flip_enable;
	unsigned int flip_mode;
	unsigned int t_width;
	unsigned int t_height;
	unsigned int s_width;
	unsigned int s_height;
	unsigned int t_addr;
	unsigned int s_addr;
} ppu_buffer_info_t;

//PPU DMA Mode
/*
* Function Name :  gplib_ppu_dma_init
*
* Syntax : INT32S gplib_ppu_dma_init(PPU_REGISTER_SETS *p_register_set);
*
* Purpose :  User can call this function to initialize ppu dma mode
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*
*
*
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_dma_init(PPU_REGISTER_SETS *p_register_set);

/*
* Function Name :  gplib_ppu_dma_uninit
*
* Syntax : INT32S gplib_ppu_dma_uninit(void);
*
* Purpose :  User can call this function to uninitialize ppu dma mode
*
* Parameters : <IN>  none
*
*
*
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_dma_uninit(void);

/*
* Function Name :  gplib_ppu_dma_update_info
*
* Syntax : INT32S gplib_ppu_dma_update_info(PPU_REGISTER_SETS *p_register_set, ppu_buffer_info_t *ppu_info);
*
* Purpose :  User can call this function to set ppu dma information
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*					                    *ppu_info:
*														ppu dma releated parameters
*
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_dma_update_info(PPU_REGISTER_SETS *p_register_set, ppu_buffer_info_t *ppu_info);

/*
* Function Name :  gplib_ppu_dma_update_go
*
* Syntax : INT32S gplib_ppu_dma_update_go(PPU_REGISTER_SETS *p_register_set, INT32U buffer_start, INT32U buffer_x_offset, INT32U buffer_y_offset);
*
* Purpose :  User can call this function to set ppu dma start draw
*
* Parameters : <IN>  *p_register_set:	PPU releated parameters
*					                    buffer_start:
*														frame buffer start ptr.
*					                    buffer_x_offset:
*														x offset of frame buffer start ptr.
*					                    buffer_y_offset:
*														y offset of frame buffer start ptr.
*
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_dma_update_go(PPU_REGISTER_SETS *p_register_set, INT32U buffer_start, INT32U buffer_x_offset, INT32U buffer_y_offset);

/*
* Function Name :  gplib_ppu_dma_update_state_get
*
* Syntax : INT32S gplib_ppu_dma_update_state_get(void);
*
* Purpose :  User can call this function to get ppu dma state
*
* Parameters : <IN>  none
*
*
*
*              <OUT> none
* Return : INT32S : 0: Success
*										Others: Fail
* Note :
*
*/
extern INT32S gplib_ppu_dma_update_state_get(void);
