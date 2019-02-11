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
#include "board_config.h"
#include "drv_l1.h"
#include "drv_l1_sfr.h"
#include "drv_l1_gpio.h"
#include "drv_l2_nand.h"

INT8S uart0_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

	if(idx > 2)
		return MUX_ERR;

	funpos0 &= ~0x3;
	if( idx == UART0_MUX0){//UART0_TX_RX__IOB5_IOB4
		funpos0 &= ~(3<<0);
		gpio_drving_init_io(IO_B5,(IO_DRV_LEVEL)UART0_DRIVING);
		gpio_drving_init_io(IO_B4,(IO_DRV_LEVEL)UART0_DRIVING);
	}
	else if( idx == UART0_MUX1){//UART0_TX_RX__IOC12_IOC13
		funpos0 |= (1<<0);
		gpio_drving_init_io(IO_C12,(IO_DRV_LEVEL)UART0_DRIVING);
		gpio_drving_init_io(IO_C13,(IO_DRV_LEVEL)UART0_DRIVING);
	}
	else if (idx == UART0_MUX2){//UART0_TX_RX__IOD5_IOD4
		funpos0 |= (2<<0);
		gpio_drving_init_io(IO_D5,(IO_DRV_LEVEL)UART0_DRIVING);
		gpio_drving_init_io(IO_D4,(IO_DRV_LEVEL)UART0_DRIVING);
	}

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

INT8S uart1_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

	if(idx > 2)
		return MUX_ERR;

	funpos0 &= ~0xc;
    if (idx == UART1_MUX0){//UART1_TX_RX__IOB7_IOB6
        funpos0 &= ~(3<<2);
        gpio_drving_init_io(IO_B6,(IO_DRV_LEVEL)UART1_DRIVING);
        gpio_drving_init_io(IO_B7,(IO_DRV_LEVEL)UART1_DRIVING);
	}
    else if (idx == UART1_MUX1){//UART1_TX_RX__IOC15_IOC14
        funpos0 |= (1<<2);
        gpio_drving_init_io(IO_C15,(IO_DRV_LEVEL)UART1_DRIVING);
        gpio_drving_init_io(IO_C14,(IO_DRV_LEVEL)UART1_DRIVING);
    }
    else if (idx == UART1_MUX2){//UART1_TX_RX__IOD9_IOD8
        funpos0 |= (2<<2);
        gpio_drving_init_io(IO_D8,(IO_DRV_LEVEL)UART1_DRIVING);
        gpio_drving_init_io(IO_D9,(IO_DRV_LEVEL)UART1_DRIVING);
     }

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

INT8S uart2_pinmux_set(INT8U idx)
{
	INT32U funpos1 = R_FUNPOS1;

	if(idx > 3)
        return MUX_ERR;

	funpos1 &= ~(3<<13);
    if (idx == UART2_MUX0){//UART2_TX_RX__IOB12_IOB11
        funpos1 &= ~(3<<13);
        gpio_drving_init_io(IO_B6,(IO_DRV_LEVEL)UART1_DRIVING);
        gpio_drving_init_io(IO_B7,(IO_DRV_LEVEL)UART1_DRIVING);
	}
    else if (idx == UART2_MUX1){//UART2_TX_RX__IOA7_IOA6
        funpos1 |= (1<<13);
        gpio_drving_init_io(IO_A6,(IO_DRV_LEVEL)UART1_DRIVING);
        gpio_drving_init_io(IO_A7,(IO_DRV_LEVEL)UART1_DRIVING);
	}
    else if (idx == UART2_MUX2){//UART2_TX_RX__IOC3_IOC2
        funpos1 |= (2<<13);
        gpio_drving_init_io(IO_C3,(IO_DRV_LEVEL)UART1_DRIVING);
        gpio_drving_init_io(IO_C2,(IO_DRV_LEVEL)UART1_DRIVING);
    }
    else if (idx == UART2_MUX3){//UART2_TX_RX__IOC5_IOC4
        funpos1 |= (3<<13);
        gpio_drving_init_io(IO_C4,(IO_DRV_LEVEL)UART1_DRIVING);
        gpio_drving_init_io(IO_C5,(IO_DRV_LEVEL)UART1_DRIVING);
    }

	R_FUNPOS1 = funpos1;
	return MUX_OK;
}

INT8S ext_irq_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx > 3)
        return MUX_ERR;

	funpos0 &= ~0x0600;
    if (idx == EXT_IRQ_MUX0)//EXT_IRQ_A_B_C__IOB8_IOB9_IOB10
        funpos0 &= ~(3<<9);
    else if (idx == EXT_IRQ_MUX1)//EXT_IRQ_A_B_C__IOC13_IOC14_IOC15
        funpos0 |= (1<<9);
    else if (idx == EXT_IRQ_MUX2)//EXT_IRQ_A_B_C__IOD7_IOD8_IOD9
        funpos0 |= (2<<9);
    else if (idx == EXT_IRQ_MUX3 )//EXT_IRQ_A_B_C__IOD12_IOD14_IOD15
        funpos0 |= (3<<9);

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

INT8S i2c0_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx > 2)
        return MUX_ERR;

	funpos0 &= ~(3<<25);
    if (idx == I2C0_MUX0){//I2C_SCL_SDA__IOB4_IOB5
        funpos0 &= ~(3<<25);
        gpio_drving_init_io(IO_B4,(IO_DRV_LEVEL)I2C0_DRIVING);
        gpio_drving_init_io(IO_B5,(IO_DRV_LEVEL)I2C0_DRIVING);
	}
    else if (idx == I2C0_MUX1){//I2C_SCL_SDA__IOC14_IOC15
        funpos0 |= (1<<25);
        gpio_drving_init_io(IO_C14,(IO_DRV_LEVEL)I2C0_DRIVING);
        gpio_drving_init_io(IO_C15,(IO_DRV_LEVEL)I2C0_DRIVING);
	}
    else if (idx == I2C0_MUX2){//I2C_SCL_SDA__IOD4_IOD5
        funpos0 |= (2<<25);
        gpio_drving_init_io(IO_D4,(IO_DRV_LEVEL)I2C0_DRIVING);
        gpio_drving_init_io(IO_D5,(IO_DRV_LEVEL)I2C0_DRIVING);
	}

	R_FUNPOS0 = funpos0;
	return MUX_ERR;
}

INT8S i2c1_pinmux_set(INT8U idx)
{
	INT32U funpos1 = R_FUNPOS1;

    if (idx > 1)
        return MUX_ERR;

	funpos1 &= ~(1<<15);
    if (idx == I2C1_MUX0){//I2C1_SCL_SDA__IOC0_IOC1
        funpos1 &= ~(1<<15);
        gpio_drving_init_io(IO_C0,(IO_DRV_LEVEL)I2C1_DRIVING);
        gpio_drving_init_io(IO_C1,(IO_DRV_LEVEL)I2C1_DRIVING);
	}
    else if (idx == I2C1_MUX1){//I2C_SCL_SDA__IOD4_IOD5
        funpos1 |= (1<<15);
        gpio_drving_init_io(IO_D4,(IO_DRV_LEVEL)I2C1_DRIVING);
        gpio_drving_init_io(IO_D5,(IO_DRV_LEVEL)I2C1_DRIVING);
	}

	R_FUNPOS1 = funpos1;
	return MUX_OK;
}

INT8S i2c2_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx > 2)
        return MUX_ERR;

	funpos0 &= ~(3<<16);
    if (idx == I2C2_MUX0){//I2C2_SCL_SDA__IOC14_IOC15
        funpos0 &= ~(3<<16);
        gpio_drving_init_io(IO_C14,(IO_DRV_LEVEL)I2C2_DRIVING);
        gpio_drving_init_io(IO_C15,(IO_DRV_LEVEL)I2C2_DRIVING);
	}
    else if (idx == I2C2_MUX1){//I2C1_SCL_SDA__IOD10_IOD12
        funpos0 |= (1<<16);
        gpio_drving_init_io(IO_D10,(IO_DRV_LEVEL)I2C2_DRIVING);
        gpio_drving_init_io(IO_D12,(IO_DRV_LEVEL)I2C2_DRIVING);
	}
    else if (idx == I2C2_MUX2){//I2C1_SCL_SDA__IOA10_IOA11
        funpos0 |= (2<<16);
        gpio_drving_init_io(IO_A10,(IO_DRV_LEVEL)I2C2_DRIVING);
        gpio_drving_init_io(IO_A11,(IO_DRV_LEVEL)I2C2_DRIVING);
	}

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}


INT8S spi0_pinmux_set(INT8U idx, INT8U cs_gpio_en)
{
	INT32U funpos0 = R_FUNPOS0;
	INT32U gpiofunpos = R_GPIOCTRL;

    if (idx > 1)
        return MUX_ERR;

	funpos0 &= ~(1<<14);
    if (idx == SPI0_MUX0){//SPI_CS_CLK_TX_RX__IOD6_IOD7_IOD8_IOD9
        funpos0 &= ~(1<<14);
        gpio_drving_init_io(IO_D6,(IO_DRV_LEVEL)SPI0_DRIVING);
        gpio_drving_init_io(IO_D7,(IO_DRV_LEVEL)SPI0_DRIVING);
        gpio_drving_init_io(IO_D8,(IO_DRV_LEVEL)SPI0_DRIVING);
	}
    else if (idx == SPI0_MUX1){//SPI0_CS_CLK_TX_RX__IOD0_IOD1_IOD2_IOD2
        funpos0 |= (1<<14);
        gpio_drving_init_io(IO_D0,(IO_DRV_LEVEL)SPI0_DRIVING);
        gpio_drving_init_io(IO_D1,(IO_DRV_LEVEL)SPI0_DRIVING);
        gpio_drving_init_io(IO_D2,(IO_DRV_LEVEL)SPI0_DRIVING);
    }

    if(cs_gpio_en)
        gpiofunpos |= (1<<3);
    else
        gpiofunpos &= ~(1<<3);

	R_FUNPOS0 = funpos0;
	R_GPIOCTRL = gpiofunpos;
	return MUX_OK;
}

INT8S spi1_pinmux_set(INT8U idx, INT8U cs_gpio_en)
{
	INT32U funpos0 = R_FUNPOS0;
    INT32U gpiofunpos = R_GPIOCTRL;

    if (idx > 1)
        return MUX_ERR;

	funpos0 &= ~(1<<15);
    if (idx == SPI1_MUX0){//SPI1_CS_CLK_TX_RX__IOC8_IOC9_IOC10_IOC11
        funpos0 &= ~(1<<15);
        gpio_drving_init_io(IO_C8,(IO_DRV_LEVEL)SPI1_DRIVING);
        gpio_drving_init_io(IO_C9,(IO_DRV_LEVEL)SPI1_DRIVING);
        gpio_drving_init_io(IO_C10,(IO_DRV_LEVEL)SPI1_DRIVING);
    }
    else if (idx == SPI1_MUX1){//SPI1_CS_CLK_TX_RX__IOD10_IOD11_IOD12_IOD13
        funpos0 |= (1<<15);
        gpio_drving_init_io(IO_D10,(IO_DRV_LEVEL)SPI1_DRIVING);
        gpio_drving_init_io(IO_D11,(IO_DRV_LEVEL)SPI1_DRIVING);
        gpio_drving_init_io(IO_D12,(IO_DRV_LEVEL)SPI1_DRIVING);
    }

    if(cs_gpio_en)
        gpiofunpos &= ~(1<<9);
    else
        gpiofunpos |= (1<<9);

	R_FUNPOS0 = funpos0;
	R_GPIOCTRL = gpiofunpos;
    return MUX_OK;
}

INT8S spifc_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx >1)
        return MUX_ERR;
    if (idx == SPIFC_MUX0){//SPIFC_CS_CLK_RX0_3__IOD0_IOD1_IOD2_IOD3_IOD4_IOD5
        funpos0 &= ~(1<<30);
        gpio_drving_init_io(IO_D0,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_D1,(IO_DRV_LEVEL)SPIFC_CLK_DRIVING);
        gpio_drving_init_io(IO_D2,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_D3,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_D4,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_D5,(IO_DRV_LEVEL)SPIFC_DRIVING);
    }
    else if (idx == SPIFC_MUX1){//SPIFC_CS_CLK_RX0_3__IOD7_IOA14_IOA12_IOD8_IOD9_IOB3
        funpos0 |= (1<<30);
        gpio_drving_init_io(IO_D7,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_A14,(IO_DRV_LEVEL)SPIFC_CLK_DRIVING);
        gpio_drving_init_io(IO_A12,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_D8,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_D9,(IO_DRV_LEVEL)SPIFC_DRIVING);
        gpio_drving_init_io(IO_A15,(IO_DRV_LEVEL)SPIFC_DRIVING);
    }

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

INT8S sdc0_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx > 2)
        return MUX_ERR;

	funpos0 &= ~(3<<21);
    if (idx== SDC0_MUX0){//SDC0_CMD_CLK_DATA0_3__IOA2_IOA3_IOA5_IOA6_IOA7_IOA4
        funpos0 &= ~(3<<21);
        R_IOA_DIR &= ~0x00F4;
        R_IOA_ATT &= ~0x00F4;
        R_IOA_O_DATA |= 0x00F4;
        gpio_drving_init_io(IO_A2,(IO_DRV_LEVEL)SDC0_CMD_CLK_DRIVING);
        gpio_drving_init_io(IO_A3,(IO_DRV_LEVEL)SDC0_CMD_CLK_DRIVING);
	}
    else if (idx == SDC0_MUX1){//SDC0_CMD_CLK_DATA0_3__IOB14_IOB15_IOB11_IOB12_IOB13_IOB10
        funpos0 |= (1<<21);
        R_IOB_DIR &= ~0x7C00;
        R_IOB_ATT &= ~0x7C00;
        R_IOB_O_DATA |= 0x7C00;
        gpio_drving_init_io(IO_B14,(IO_DRV_LEVEL)SDC0_CMD_CLK_DRIVING);
        gpio_drving_init_io(IO_B15,(IO_DRV_LEVEL)SDC0_CMD_CLK_DRIVING);
    }
    else if (idx == SDC0_MUX2){//SDC0_CMD_CLK_DATA0_3__IOC6_IOC7_IOC9_IOC10_IOC11_IOC8
        funpos0 |= (2<<21);
        R_IOC_DIR &= ~0x0F40;
        R_IOC_ATT &= ~0x0F40;
        R_IOC_O_DATA |= 0x0F40;
        gpio_drving_init_io(IO_C6,(IO_DRV_LEVEL)SDC0_CMD_CLK_DRIVING);
        gpio_drving_init_io(IO_C7,(IO_DRV_LEVEL)SDC0_CMD_CLK_DRIVING);
    }

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

INT8S sdc1_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx > 2)
        return MUX_ERR;

	funpos0 &= ~(3<<23);
    if (idx == SDC1_MUX0){//SDC1_CMD_CLK_DATA0_3__IOD10_IOD11_IOD13_IOD14_IOD15_IOD12
        funpos0 &= ~(3<<23);
        R_IOD_DIR &= ~0xF400;
        R_IOD_ATT &= ~0xF400;
        R_IOD_O_DATA |= 0xF400;
        gpio_drving_init_io(IO_D10,(IO_DRV_LEVEL)SDC1_CMD_CLK_DRIVING);
        gpio_drving_init_io(IO_D11,(IO_DRV_LEVEL)SDC1_CMD_CLK_DRIVING);
	}
    else if (idx == SDC1_MUX1){//SDC1_CMD_CLK_DATA0_3__IODB0_IOB1_IOB3_IOB4_IOB5_IOB2
        funpos0 |= (1<<23);
        R_IOB_DIR &= ~0x003D;
        R_IOB_ATT &= ~0x003D;
        R_IOB_O_DATA |= 0x003D;
        gpio_drving_init_io(IO_B0,(IO_DRV_LEVEL)SDC1_CMD_CLK_DRIVING);
        gpio_drving_init_io(IO_B1,(IO_DRV_LEVEL)SDC1_CMD_CLK_DRIVING);
    }
    else if (idx == SDC1_MUX2){//SDC1_CMD_CLK_DATA0_3__IOD10_IOD11_IOD13_IOC14_IOC15_IOC13
        //DBG_PRINT("R_GPIOCTRL=0x%08x\r\n", R_GPIOCTRL);
        //R_GPIOCTRL &= ~0x1; // SD1#2 pins conflict with JTAG, so off the JTAG.
                            // JTAG be on no matter what boot mode and no matter JTAG ioTrap on.
        funpos0 |= (2<<23);
        R_IOD_DIR &= ~0x2400;
        R_IOD_ATT &= ~0x2400;
        R_IOD_O_DATA |= 0x2C00;
        R_IOC_DIR &= ~0xE000;
        R_IOC_ATT &= ~0xE000;
        R_IOC_O_DATA |= 0xE000;
        gpio_drving_init_io(IO_D10,(IO_DRV_LEVEL)SDC1_CMD_CLK_DRIVING);
        gpio_drving_init_io(IO_D11,(IO_DRV_LEVEL)SDC1_CMD_CLK_DRIVING);
	}

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

INT8S tft_pinmux_set(INT8U ctrl_mux, INT8U ldata_mux, INT8U hdata_mux)
{
	INT32U funpos0 = R_FUNPOS0;

    if ((ctrl_mux > 1) || (ldata_mux > 1) || (hdata_mux > 1))
        return MUX_ERR;

	funpos0 &= ~(7<<11);
    if (ldata_mux == TFT_DATA0_7_MUX0)//TFT_DATA_0_7__IOA0_IOA7
        funpos0 &= ~(1<<11);
    else if (ldata_mux == TFT_DATA0_7_MUX1)//TFT_DATA_0_7__IOE0_IOE7
        funpos0 |= (1<<11);

    if (hdata_mux == TFT_DATA8_15_MUX0)//TFT_DATA_8_15__IOA8_IOA15
        funpos0 &= ~(1<<12);
    else if (hdata_mux == TFT_DATA8_15_MUX1)//TFT_DATA_8_15__IOE0_E7
        funpos0 |= (1<<12);

    if (ctrl_mux == TFT_CTRL_MUX0){//TFT_CTRL_DE_HSYNC_VSYNC_CLK__IOB0_IOB1_IOB2_IOB3
        funpos0 &= ~(1<<13);
        gpio_drving_init_io(IO_B3,(IO_DRV_LEVEL)TFT_CLK_DRIVING);
	}
    else if (ctrl_mux == TFT_CTRL_MUX1){//TFT_CTRL_DE_HSYNC_VSYNC_CLK__IOC8_IOC9_IOC10_IOC11
        funpos0 |= (1<<13);
        gpio_drving_init_io(IO_C11,(IO_DRV_LEVEL)TFT_CLK_DRIVING);
	}

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

INT8S csi_mipi_clko_set(INT8U clko_mux)
{
	INT32U funpos1 = R_FUNPOS1;

    if (clko_mux > 2)
        return MUX_ERR;

	funpos1 &= ~(0x7<<23);
	funpos1 &= ~(0x7<<20);

    if (clko_mux == CSI_MIPI_CLKO_MUX0){//CSI_MIPI_CLKO__IOC9
        funpos1 &= ~(1<<23);
        funpos1 |= (1<<20);
        gpio_drving_init_io(IO_C9,(IO_DRV_LEVEL)CSI_MIPI_CLKO_DRIVING);
	}
    else if (clko_mux == CSI_MIPI_CLKO_MUX1 ){//CSI_MIPI_CLKO__IOD7
        funpos1 |= (1<<24) | (1<<21);
        gpio_drving_init_io(IO_D7,(IO_DRV_LEVEL)CSI_MIPI_CLKO_DRIVING);
    }
    else if (clko_mux == CSI_MIPI_CLKO_MUX2){//CSI_MIPI_CLKO__IOD12
        funpos1 |= (1<<25) | (1<<22);
        gpio_drving_init_io(IO_D12,(IO_DRV_LEVEL)CSI_MIPI_CLKO_DRIVING);
	}

	R_FUNPOS1 = funpos1;
    return MUX_OK;
}


INT8S csi_pinmux_set(INT8U ctrl_mux, INT8U data_mux)
{
	INT32U funpos1 = R_FUNPOS1;

    if ((ctrl_mux > 2) || (data_mux > 2))
        return MUX_ERR;

	funpos1 &= ~(0x1c<<0);

    if (ctrl_mux == CSI_CTRL_MUX0){//CSI_CTRL_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11
        funpos1 &= ~(1<<2);
	}
    else if (ctrl_mux == CSI_CTRL_MUX1 ){//CSI_CTRL_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9
        funpos1 |= (1<<2);
    }

    if (data_mux == CSI_DATA2_9_MUX0)//CSI_DATA_2_9__IOC0_IOC7
        funpos1 &= ~(3<<3);
    else if (data_mux == CSI_DATA2_9_MUX1)//CSI_DATA_2_9__IOB8_IOB15
        funpos1 |= (1<<3);
    else if (data_mux == CSI_DATA2_9_MUX2)//CSI_DATA_2_9__IOE0_IOE7
        funpos1 |= (2<<3);

	R_FUNPOS1 = funpos1;
    return MUX_OK;
}

INT8S cdsp_clko_set(INT8U clko_mux)
{
	INT32U funpos1 = R_FUNPOS1;

    if (clko_mux > 2)
        return MUX_ERR;

	funpos1 &= ~(0x7<<20);
	funpos1 &= ~(0x7<<23);

    if (clko_mux == CDSP_CLKO_MUX0){//CDSP_CLKO__IOC9
        funpos1 |= (1<<20);
        gpio_drving_init_io(IO_C9,(IO_DRV_LEVEL)CDSP_CLKO_DRIVING);
	}
    else if (clko_mux == CDSP_CLKO_MUX1){//CDSP_CLKO__IOD7
        funpos1 |= (1<<21);
        gpio_drving_init_io(IO_D7,(IO_DRV_LEVEL)CDSP_CLKO_DRIVING);
    }
    else if (clko_mux == CDSP_CLKO_MUX2){//CDSP_CLKO__IOD12
        funpos1 |= (1<<22);
        gpio_drving_init_io(IO_D12,(IO_DRV_LEVEL)CDSP_CLKO_DRIVING);
	}

	R_FUNPOS1 = funpos1;
    return MUX_OK;
}

INT8S cdsp_pinmux_set(INT8U ctrl_mux, INT8U hdata_mux, INT8U ldata_mux)
{
	INT32U funpos1 = R_FUNPOS1;
	INT32U gpioctrl = R_GPIOCTRL;

    if ((ctrl_mux > 2) || (hdata_mux > 2) || (ldata_mux > 2))
        return MUX_ERR;

	funpos1 &= ~(0x0F80<<0);

    if (ctrl_mux == CDSP_CTRL_MUX0){//CDSP_CTRL_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11
        funpos1 &= ~(1<<7);
	}
    else if (ctrl_mux == CDSP_CTRL_MUX1 ){//CDSP_CTRL_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9
        funpos1 |= (1<<7);
    }

    if (hdata_mux == CDSP_DATA2_9_MUX0)//CSI_DATA_2_9__IOC0_IOC7
        funpos1 &= ~(3<<8);
    else if (hdata_mux == CDSP_DATA2_9_MUX1)//CSI_DATA_2_9__IOB8_IOB15
        funpos1 |= (1<<8);
    else if (hdata_mux == CDSP_DATA2_9_MUX2)//CSI_DATA_2_9__IOE0_IOE7
        funpos1 |= (2<<8);

    //gpioctrl |= (1<<1);
    gpioctrl &= ~(1<<1);

    if (ldata_mux == CDSP_DATA0_1_MUX0)//CSI_DATA_0_1__IOB6_IOB7
        funpos1 &= ~(3<<10);
    else if (ldata_mux == CDSP_DATA0_1_MUX1)//CSI_DATA_0_1__IOB4_IOB5
        funpos1 |= (1<<10);
    else if (ldata_mux == CDSP_DATA0_1_MUX2)//CSI_DATA_0_1__IOC12_IOC13
        funpos1 |= (2<<10);


	R_FUNPOS1 = funpos1;
	R_GPIOCTRL = gpioctrl;

    return MUX_OK;
}
#if (BOARD_TYPE != BOARD_GPM41XXA_EMU_V1_0) || (_DRV_L1_NAND == 1)
INT8U nand_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx>1)
        return MUX_ERR;

    if (idx == NADN_DATA_MUX0)//NAND_DATA_0_7__IOB8_IOB15
    {
        funpos0 &= ~(1<<6);
        nand_iopad_sel(NAND_IO_IOB); // NFIO_CFG, NAND_IO_IOB
    }
    else if (idx == NADN_DATA_MUX1)//NANN_DATA_0_7__IOC4_IOC11
    {
        funpos0 |= (1<<6);
        nand_iopad_sel(NAND_IO_IOC);  // NFIO_CFG, NAND_IO_IOC
    }
	R_FUNPOS0 = funpos0;
	return MUX_OK;
}
#endif
INT8U cec_pinmux_set(INT8U idx)
{
    if (idx>MUX1)
        return MUX_ERR;

    if (idx == MUX0)
    {
        R_FUNPOS1 &= ~(0x1<<12);
        R_IOC_O_DATA &= ~(0x1);
        R_IOC_DIR &= ~(0x1);
        R_IOC_ATT |= (0x1);
    }
    else if (idx == MUX1)
    {
        R_FUNPOS1 |= (0x1<<12);
        R_GPIOCTRL &= ~(0x1);
        R_IOC_O_DATA &= ~(0x1<<12);
        R_IOC_DIR &= ~(0x1<<12);
        R_IOC_ATT |= (0x1<<12);
    }

	return MUX_OK;
}

INT8U can_pinmux_set(INT8U idx)
{
    //CAN_SEL  - 0: IOC0/IOC1, 1: IOC2/IOC3
    if (idx>MUX1)
        return MUX_ERR;
    R_GPIOCTRL |= 0x00000100;     // Enable CAN Pin Mux
    R_FUNPOS0 &= ~0x80000000;

    if (idx == MUX1)
    {
        R_FUNPOS0 |= 0x80000000;
    }

	return MUX_OK;
}

INT8S timer_ccp_pinmux_set(INT8U idx)
{
	INT32U funpos0 = R_FUNPOS0;

    if (idx > 4)
        return MUX_ERR;

	funpos0 &= ~0x0180;
    if (idx == TIMER_CCP_MUX0)      //TIMER_A_TO_H_CCP__IOB8_IOB9_IOB10_IOC12_IOC13_IOD4_IOD5_IOD6
        funpos0 &= ~(3<<7);
    else if (idx == TIMER_CCP_MUX1) //TIMER_A_TO_H_CCP__IOD7_IOD8_IOD9_IOD12_IOD14_IOD15_IOB6_IOB7
        funpos0 |= (1<<7);
    else if (idx == TIMER_CCP_MUX2) //TIMER_A_TO_H_CCP__IOA8_IOA9_IOA10_IOA11_IOA12_IOA13_IOA14_IOA15
        funpos0 |= (2<<7);
    else if (idx == TIMER_CCP_MUX3 )//TIMER_A_TO_H_CCP__IOE0_IOE1_IOE2_IOE3_IOE4_IOE5_IOE6_IOE7
        funpos0 |= (3<<7);

	R_FUNPOS0 = funpos0;
	return MUX_OK;
}

void board_init(void)
{
	gpio_init();

	tft_pinmux_set(TFT_CTRL_POS,TFT_DATA_0_7_POS,TFT_DATA_8_15_POS);

    csi_mipi_clko_set(CSI_MIPI_CLKO_POS);
    csi_pinmux_set(CSI_CTRL_POS, CSI_DATA_2_9_POS);

    cdsp_clko_set(CDSP_CLKO_POS);
	cdsp_pinmux_set(CDSP_CTRL_POS, CDSP_DATA_2_9_POS, CDSP_DATA_0_1_POS);

    uart0_pinmux_set(UART0_POS);
    uart1_pinmux_set(UART1_POS);
	uart2_pinmux_set(UART2_POS);

	ext_irq_pinmux_set(EXT_IRQ_POS);

	i2c0_pinmux_set(I2C0_POS);
	i2c1_pinmux_set(I2C1_POS);
	i2c2_pinmux_set(I2C2_POS);

	spi0_pinmux_set(SPI0_POS, SPI0_CS_GPIO_MODE);
	spi1_pinmux_set(SPI1_POS, SPI1_CS_GPIO_MODE);
#if BOARD_TYPE != BOARD_GPM41XXA_EMU_V1_0
	spifc_pinmux_set(SPIFC_POS);
#endif
	sdc0_pinmux_set(SDC0_POS);
	sdc1_pinmux_set(SDC1_POS);
#if (BOARD_TYPE != BOARD_GPM41XXA_EMU_V1_0) || (_DRV_L1_NAND == 1)
	nand_pinmux_set(NAND_DATA_POS);
#endif
	#if _DRV_L1_HDMI == 0
	gpio_IOE_switch_config_from_HDMI_to_GPIO();
	#endif

	timer_ccp_pinmux_set(TIMER_CCP_POS);

}
