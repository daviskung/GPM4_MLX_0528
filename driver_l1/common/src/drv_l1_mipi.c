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
#include "drv_l1_mipi.h"
#include "drv_l1_clock.h"

#if (defined _DRV_L1_MIPI) && (_DRV_L1_MIPI == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#ifndef DISABLE
#define DISABLE     0
#endif

#ifndef ENABLE
#define ENABLE      1
#endif

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct mipiReg_s
{
	volatile INT32U	GLB_CSR;		//0x00
	volatile INT32U	PHY_RST;		//0x04
	volatile INT32U	RC_CTRL;		//0x08
	volatile INT32U	ECC_ORDER;		//0x0C

	volatile INT32U	CCIR601_TIMING;	//0x10
	volatile INT32U	IMG_SIZE;		//0x14
	volatile INT32U	RESERVED0[2];	//0x18 ~ 0x1C

	volatile INT32U	DATA_FMT;		//0x20
	volatile INT32U	PAYLOAD_HEADER;	//0x24
	volatile INT32U	CTRL_STATE;		//0x28
	volatile INT32U	VLK_CHECK;		//0x2C

	volatile INT32U	HEADER_DATA;	//0x30
	volatile INT32U	HEADER_DT_VLD;	//0x34
	volatile INT32U	RESERVED1[2];	//0x38 ~ 0x3C

	volatile INT32U INT_ENABLE;		//0x40
	volatile INT32U	RESERVED2[3];	//0x44 ~ 0x4C

	volatile INT32U	RESERVED3[12];	//0x50 ~ 0x7C

	volatile INT32U INT_SOURCE;		//0x80
	volatile INT32U	RESERVED4[3];	//0x84 ~ 0x8C

	volatile INT32U	RESERVED5[12];	//0x90 ~ 0xBC

	volatile INT32U	INT_STATUS;		//0xC0
} mipiReg_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
//extern INT32U MCLK;
static void (*mipi_isr_callback)(INT32U dev_no, INT32U event);


static mipiReg_t* mipi_select_sfr_base(INT32S DevNo)
{
    if(DevNo == MIPI_0) {
        return (mipiReg_t *)MIPI0_BASE;
    }

    if(DevNo == MIPI_1) {
        return (mipiReg_t *)MIPI1_BASE;
    }

	return 0;
}

void MIPI_IRQHandler(void)
{
	INT32U dev_no;
	INT32U IRQ_Status, IRQ_Enable;
	mipiReg_t *pDev;

	for(dev_no=0; dev_no<MIPI_MAX_NUMS; dev_no++) {
		pDev = mipi_select_sfr_base(dev_no);

		IRQ_Status = pDev->INT_STATUS;
		IRQ_Enable = pDev->INT_ENABLE;

		if(IRQ_Status) {
			// clear interrupt
			pDev->INT_SOURCE = IRQ_Status;

			IRQ_Status &= IRQ_Enable;
			if(mipi_isr_callback && IRQ_Status) {
				mipi_isr_callback(dev_no, IRQ_Status);
			}
		}
	}
}

static void drv_l1_mipi_phy_reset(INT32U dev_no)
{
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	pDev->PHY_RST |=0x3; //SOF_RST_DA & CK
}

/**
 * @brief   mipi enable
 * @param   dev_no[in]: mipi device number
 * @param   enable[in]: mipi enable or disable
 * @return 	none
 */
void drv_l1_mipi_on(INT32U dev_no, INT8U enable)
{
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

        pDev->GLB_CSR &= ~0x01;

	if(enable == 0) {
		pDev->GLB_CSR &= 0x01;
		pDev->INT_SOURCE = 0x3F;
		pDev->INT_ENABLE = 0;
		NVIC_DisableIRQ(MIPI_IRQn);
		return;
	}

	pDev->GLB_CSR |= 0x01;
}

/**
 * @brief   mipi init
 * @param   dev_no[in]: mipi device number
 * @param   enable[in]: mipi enable or disable
 * @return 	none
 */
void drv_l1_mipi_init(INT32U dev_no, INT8U enable)
{
	drv_l1_clock_set_system_clk_en(25,1);

	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	if(enable == 0) {
		pDev->GLB_CSR = 0;
		pDev->INT_SOURCE = 0x3F;
		pDev->INT_ENABLE = 0;
		NVIC_DisableIRQ(MIPI_IRQn);
		return;
	}

	pDev->GLB_CSR = 0;		//init mipi must be off then will be can set cfg
    //pDev->GLB_CSR = 0x01;
	pDev->ECC_ORDER = 0;
	pDev->CCIR601_TIMING = 0;
	pDev->IMG_SIZE = 0;
	pDev->DATA_FMT = 0;
	pDev->INT_SOURCE = 0x3F;
	pDev->INT_ENABLE = 0;

    NVIC_SetPriority(MIPI_IRQn, 5);
    NVIC_EnableIRQ(MIPI_IRQn);
}

/**
 * @brief   mipi isr function register
 * @param   user_isr[in]: isr function
 * @return 	STATUS_OK/STATUS_FAIL
 */
INT32S drv_l1_mipi_isr_register(void (*user_isr_callback)(INT32U dev_no, INT32U event))
{
	if(user_isr_callback == 0) {
		return STATUS_FAIL;
	}

	mipi_isr_callback = user_isr_callback;
	return STATUS_OK;
}

/**
 * @brief   mipi isr function unregister
 * @param   none
 * @return 	none
 */
void drv_l1_mipi_isr_unregister(void)
{
	mipi_isr_callback = 0;
}

/**
 * @brief   mipi isr function unregister
 * @param   dev_no[in]: mipi device number
 * @param   enable[in]: enable or disable
 * @param   index[in]: interrdrv_l1_mipi_initupt bit
 * @return 	STATUS_OK/STATUS_FAIL
 */
INT32S drv_l1_mipi_set_irq_enable(INT32U dev_no, INT32U enable, INT32U bit)
{
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	if(enable) {
		pDev->INT_SOURCE |= bit & 0x3F;
		pDev->INT_ENABLE |= bit & 0x3F;
	} else {
		pDev->INT_ENABLE &= ~bit & 0x3F;
		pDev->INT_SOURCE = bit & 0x3F;
	}

	return STATUS_OK;
}

/**
 * @brief   set mipi global configure
 * @param   dev_no[in]: mipi device number
 * @param   low_power_en[in]: low power enable or disable
 * @param   byte_clk_edge[in]: byt clock edge set, 0: posedge, 1: negedge
 * @return 	none
 */
void drv_l1_mipi_set_global_cfg(INT32U dev_no, INT8U low_power_en, INT8U byte_clk_edge)
{
	INT32U reg;
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	reg = pDev->GLB_CSR;
	if(low_power_en) {
		reg |= (1<<4);
	} else {
		reg &= ~(1<<4);
	}

	if(byte_clk_edge) {
		reg |= (1<<5);
	} else {
		reg &= ~(1<<5);
	}

	pDev->GLB_CSR = reg;
}

/**
 * @brief   set mipi lane number
 * @param   dev_no[in]: mipi device number
 * @param   lane_no[in]: data lane number set, 0: MIPI_1Lane, 1: MIPI_2Lane
 * @return 	none
 */
void drv_l1_mipi_set_lane_number(INT32U dev_no, INT8U lane_no)
{
	INT32U reg;
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	reg = pDev->GLB_CSR;
	if(lane_no == MIPI_2_LANE) {
		reg |= (1<<8);
	} else {
		reg &= ~(1<<8);
	}

	pDev->GLB_CSR = reg;
}

/**
 * @brief   set mipi byte clock source
 * @param   dev_no[in]: mipi device number
 * @param   byte_clk[in]: 1: use pixel clock 0: use D_PHY byte clock
 * @return 	none
 */
void drv_l1_mipi_set_pixel_clock(INT32U dev_no, INT8U pixel_clk)
{
	INT32U reg;
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	reg = pDev->GLB_CSR;
	if(pixel_clk) {
		reg |= (1<<12);
	} else {
		reg &= ~(1<<12);
	}

	pDev->GLB_CSR = reg;
}

/**
 * @brief   set mipi ecc configure
 * @param   dev_no[in]: mipi device number
 * @param   ecc_order[in]: ecc order, MIPI_ECC_ORDER0 ~ MIPI_ECC_ORDER3
 * @param   ecc_check_en[in]: ecc check enable/disable
 * @return 	none
 */
void drv_l1_mipi_set_ecc(INT32U dev_no, INT8U ecc_order, INT8U ecc_check_en)
{
	INT32U reg;
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	reg = pDev->ECC_ORDER;
	reg &= ~0x7;
	reg |= ((ecc_check_en & 0x01) << 2) | (ecc_order & 0x03);
	pDev->ECC_ORDER = reg;
}

/**
 * @brief   set mipi ecc configure
 * @param   dev_no[in]: mipi device number
 * @param   da_mask_cnt[in]: LP to HS mask count
 * @param   check_hs_seq[in]: 1: Check HS sequence. 0: just check LP. when enter HS mode
 * @return 	none
 */
void drv_l1_mipi_set_mask_cnt(INT32U dev_no, INT8U da_mask_cnt, INT8U check_hs_seq)
{
	INT32U reg;
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	reg = pDev->ECC_ORDER;
	reg &= ~(0x1FF << 8);
	reg |= (((INT32U)check_hs_seq & 0x01) << 16) | (((INT32U)da_mask_cnt & 0xFF) << 8);
	pDev->ECC_ORDER = reg;
}

/**
 * @brief   mipi set ccir601 interface
 * @param   dev_no[in]: mipi device number
 * @param   h_back_proch[in]: horizontal back proch
 * @param   h_front_proch[in]: horizontal front proch
 * @param   blanking_line_en[in]: blanking line enable, 0 mask hsync when vsync
 * @return 	none
 */
void drv_l1_mipi_set_ccir601_if(INT32U dev_no, INT8U h_back_porch, INT8U h_front_porch, INT8U blanking_line_en)
{
	INT32U reg;
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	reg = pDev->CCIR601_TIMING;
	reg &= ~(0x0F << 0);
	reg |= (h_back_porch & 0x0F);

	reg &= ~(0x0F << 8);
	reg |= ((INT32U)(h_front_porch & 0x0F) << 8);

	if(blanking_line_en) {
		reg |= (1<<16);
	} else {
		reg &= ~(1<<16);
	}

	pDev->CCIR601_TIMING = reg;
}

/**
 * @brief   mipi set image size
 * @param   dev_no[in]: mipi device number
 * @param   h_size[in]: horizontal size set
 * @param   v_size[in]: vertical size set
 * @return 	none
 */
void drv_l1_mipi_set_image_size(INT32U dev_no, INT16U h_size, INT16U v_size)
{
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	if(h_size == 0) {
		h_size = 1;
	}

	if(v_size == 0) {
		v_size = 1;
	}

	h_size &= 0xFFFF;
	v_size &= 0xFFFF;
	pDev->IMG_SIZE = h_size | ((INT32U)v_size << 16);
}

/**
 * @brief   mipi set data format
 * @param   dev_no[in]: mipi device number
 * @param   data_from_mmr[in]: 0: auto detect data type, 1: user define
 * @param   data_type_mmr[in]: user define data format. MIPI_YUV422 ~ MIPI_RAW12
 * @param   data_type_cdsp[in]: for cdsp 10bit data bus, 1:data[7:0]+2':00, 0: 2'00+data[7:0]
 * @return 	none
 */
void drv_l1_mipi_set_data_fmt(INT32U dev_no, INT8U data_from_mmr, INT8U data_type_mmr, INT8U data_type_cdsp)
{
	INT32U reg;
	mipiReg_t *pDev = mipi_select_sfr_base(dev_no);

	reg = pDev->DATA_FMT;
	if(data_from_mmr) {
		reg |= (1<<0);
	} else {
		reg &= ~(1<<0);
	}

	reg &= ~(0x07 << 4);
	reg |= (data_type_mmr & 0x07) << 4;

	if(data_type_cdsp) {
		reg |= (1 << 7);
	} else {
		reg &= ~(1 << 7);
	}

	pDev->DATA_FMT = reg;
}

/**
 * @brief   mipi set parameter
 * @param   dev_no[in]: mipi device number
 * @param   argp[in]: parameter
 * @return 	STATUS_OK / STATUS_FAIL
 */
INT32S drv_l1_mipi_set_parameter(INT32U dev_no, mipi_config_t *argp)
{
	INT32U delta_t, data_mask_cnt, remain;

	if(argp == 0) {
		return STATUS_FAIL;
	}

	drv_l1_mipi_init(dev_no, ENABLE);
	drv_l1_mipi_set_global_cfg(dev_no, argp->low_power_en, argp->byte_clk_edge);
	drv_l1_mipi_set_lane_number(dev_no, argp->mipi_lane_no);
	drv_l1_mipi_set_pixel_clock(dev_no, argp->pixel_clk_sel);

	/* change to ns */
	delta_t = 1000*1000000/ (SystemCoreClock / 1024);

	/* must > 40ns */
	if(argp->data_mask_time < 40) {
		argp->data_mask_time = 40;
	}

	/* get data mask count */
	data_mask_cnt = argp->data_mask_time*1024 / delta_t;
	//remain = argp->data_mask_time % delta_t;
	//if(remain) {
	//	data_mask_cnt++;
	//}
	data_mask_cnt-=2;
	if(data_mask_cnt <= 0) data_mask_cnt = 1;

	drv_l1_mipi_set_ecc(dev_no, argp->ecc_order, argp->ecc_check_en);
	drv_l1_mipi_set_mask_cnt(dev_no, data_mask_cnt, argp->check_hs_seq);
	drv_l1_mipi_set_ccir601_if(dev_no, argp->h_back_porch, argp->h_front_porch, argp->blanking_line_en);
	if(argp->data_from_mmr) {
		// user define format
		drv_l1_mipi_set_data_fmt(dev_no, argp->data_from_mmr, argp->data_type, argp->data_type_to_cdsp);
		if(argp->data_type == MIPI_YUV422) {
			drv_l1_mipi_set_image_size(dev_no, argp->h_size << 1, argp->v_size);
		} else {
			drv_l1_mipi_set_image_size(dev_no, argp->h_size, argp->v_size);
		}
	} else {
		// auto detect
		drv_l1_mipi_set_data_fmt(dev_no, 0, 0, argp->data_type_to_cdsp);
		drv_l1_mipi_set_image_size(dev_no, argp->h_size, argp->v_size);
	}

	drv_l1_mipi_phy_reset(dev_no);
    drv_l1_mipi_on(dev_no, ENABLE);		//cfg done, then mipi on
	return STATUS_OK;
}
#endif
