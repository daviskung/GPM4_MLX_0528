#ifndef __BOARD_CONFIG__
#define __BOARD_CONFIG__

#include "project.h"

/************************************************************/
/*						define								*/
/************************************************************/
/* mux selection define */
#define MUX0        0
#define MUX1        1
#define MUX2        2
#define MUX3        3

#define MUX_OK      0
#define MUX_ERR     -1

/* analog power define */
#define VDAC_V30	0x0
#define VDAC_V31	0x1
#define VDAC_V32	0x2
#define VDAC_V33	0x3

/* GPIO PAD DRIVING */
#define IO_DRIVING_8mA				0
#define IO_DRIVING_16mA				1

/************************************************************/
/*						Pinmux	setting						*/
/************************************************************/
/* UART0 PINMUX */
#define UART0_MUX0								MUX0//UUART0_TX_RX__IOB5_IOB4
#define UART0_MUX1								MUX1//UART0_TX_RX__IOC12_IOC13
#define UART0_MUX2								MUX2// UART0_TX_RX__IOD5_IOD4
#define UART0_MUX3								MUX3// UART0_TX_RX__IOA3_IOA2

#define UART0_POS					            UART0_MUX0

#define UART0_DRIVING				            IO_DRIVING_8mA

/* UART1 PINMUX */
#define UART1_MUX0								MUX0//UART1_TX_RX__IOB7_IOB6
#define UART1_MUX1								MUX1//UART1_TX_RX__IOC15_IOC14
#define UART1_MUX2								MUX2//UART1_TX_RX__IOD9_IOD8

#define UART1_POS					            UART1_MUX0
#define UART1_DRIVING				            IO_DRIVING_8mA


/* UART1 PINMUX */
#define UART2_MUX0								MUX0//UART2_TX_RX__IOB12_IOB11
#define UART2_MUX1								MUX1//UART2_TX_RX__IOA7_IOA6
#define UART2_MUX2								MUX2//UART2_TX_RX__IOC3_IOC2
#define UART2_MUX3								MUX3//UART2_TX_RX__IOC5_IOC4

#define UART2_POS					            UART2_MUX2
#define UART2_DRIVING				            IO_DRIVING_8mA

/* EXT IRQ PINMUX */
#define EXT_IRQ_MUX0							MUX0//EXT_IRQ_A_B_C__IOB8_IOB9_IOB10
#define EXT_IRQ_MUX1							MUX1//EXT_IRQ_A_B_C__IOC12_IOC13_IOC14
#define EXT_IRQ_MUX2							MUX2//EXT_IRQ_A_B_C__IOD7_IOD8_IOD9
#define EXT_IRQ_MUX3							MUX3//EXT_IRQ_A_B_C__IOD12_IOD14_IOD15
#define EXT_IRQ_POS								EXT_IRQ_MUX3
#define EXT_IRQ_DRIVING							IO_DRIVING_8mA

/* Timer A/B/C CCP PINMUX */
#define TIMER_CCP_MUX0						    MUX0//TIMER_A_TO_H_CCP__IOB8_IOB9_IOB10_IOC12_IOC13_IOD4_IOD5_IOD6
#define TIMER_CCP_MUX1						    MUX1//TIMER_A_TO_H_CCP__IOD7_IOD8_IOD9_IOD12_IOD14_IOD15_IOB6_IOB7
#define TIMER_CCP_MUX2						    MUX2//TIMER_A_TO_H_CCP__IOA8_IOA9_IOA10_IOA11_IOA12_IOA13_IOA14_IOA15
#define TIMER_CCP_MUX3						    MUX3//TIMER_A_TO_H_CCP__IOE0_IOE1_IOE2_IOE3_IOE4_IOE5_IOE6_IOE7
#define TIMER_CCP_POS					    	TIMER_CCP_MUX0
#define TIMER_CCP_DRIVING			    		IO_DRIVING_8mA


/* I2C0 PINMUX */
#define I2C0_MUX0								MUX0//I2C0_SCL_SDA__IOB4_IOB5
#define I2C0_MUX1								MUX1//I2C0_SCL_SDA__IOC12_IOC13
#define I2C0_MUX2								MUX2//I2C0_SCL_SDA__IOA8_IOA9

#define I2C0_POS							    I2C0_MUX1
#define I2C0_DRIVING						    IO_DRIVING_8mA

/* I2C1 PINMUX */
#define I2C1_MUX0								MUX0//I2C1_SCL_SDA__IOC0_IOC1
#define I2C1_MUX1								MUX1//I2C1_SCL_SDA__IOD4_IOD5

#define I2C1_POS							    I2C1_MUX0
#define I2C1_DRIVING						    IO_DRIVING_8mA

/* I2C2 PINMUX */
#define I2C2_MUX0								MUX0//I2C2_SCL_SDA__IOD14_IOD15
#define I2C2_MUX1								MUX1//I2C2_SCL_SDA__IOD10_IOD12
#define I2C2_MUX2								MUX2//I2C2_SCL_SDA__IOA10_IOA11

#define I2C2_POS							    I2C2_MUX1
#define I2C2_DRIVING						    IO_DRIVING_8mA

/* SPI0 PINMUX */
#define SPI0_MUX0								MUX0//SPI0_CS_CLK_TX_RX__IOD6_IOD7_IOD8_IOD9
#define SPI0_MUX1								MUX1//SPI0_CS_CLK_TX_RX__IOD0_IOD1_IOD2_IOD2
#define SPI0_POS								SPI0_MUX0
#define CS_GPIO_ENABLE                          1
#define CS_GPIO_DISABLE                         0
#define SPI0_CS_GPIO_MODE                       CS_GPIO_ENABLE
#define SPI0_DRIVING							IO_DRIVING_8mA

/* SPI1 PINMUX */
#define SPI1_MUX0								MUX0//SPI1_CS_CLK_TX_RX__IOC8_IOC9_IOC10_IOC11
#define SPI1_MUX1								MUX1//SPI1_CS_CLK_TX_RX__IOD10_IOD11_IOD12_IOD13
#define SPI1_POS								SPI1_MUX1
#define SPI1_CS_GPIO_MODE                       CS_GPIO_ENABLE
#define SPI1_DRIVING							IO_DRIVING_8mA

/* SPIFC PINMUX */
#define SPIFC_MUX0								MUX0//SPIFC_CS_CLK_RX0_3__IOD0_IOD1_IOD2_IOD3_IOD4_IOD5
#define SPIFC_MUX1								MUX1//SPIFC_CS_CLK_RX0_3__IOD7_IOA14_IOA12_IOD8_IOD9_IOA15

#define SPIFC_POS								SPIFC_MUX0
#define SPIFC_CLK_DRIVING					    IO_DRIVING_8mA
#define SPIFC_DRIVING							IO_DRIVING_8mA

/* SDC0 PINMUX */
#define SDC0_MUX0								MUX0//SDC0_CMD_CLK_DATA0_3__IOA2_IOA3_IOA5_IOA6_IOA7_IOA4
#define SDC0_MUX1								MUX1//SDC0_CMD_CLK_DATA0_3__IOB14_IOB15_IOB11_IOB12_IOB13_IOB10
#define SDC0_MUX2								MUX2//SDC0_CMD_CLK_DATA0_3__IOC6_IOC7_IOC9_IOC10_IOC11_IOC8
#if (BOARD_TYPE == BOARD_GPM41XXA_EMU_V1_0)
#define SDC0_POS								SDC0_MUX1
#define SDC0_CMD_CLK_DRIVING			        IO_DRIVING_8mA
#else
#define SDC0_POS								SDC0_MUX0
#define SDC0_CMD_CLK_DRIVING			        IO_DRIVING_8mA
#endif
/* SDC1 PINMUX */
#define SDC1_MUX0								MUX0//SDC1_CMD_CLK_DATA0_3__IOD10_IOD11_IOD13_IOD14_IOD15_IOD12
#define SDC1_MUX1								MUX1//SDC1_CMD_CLK_DATA0_3__IODB0_IOB1_IOB3_IOB4_IOB5_IOB2
#define SDC1_MUX2								MUX2//SDC1_CMD_CLK_DATA0_3__IOD10_IOD11_IOD13_IOC14_IOC15_IOC13	0x2

#define SDC1_POS								SDC1_MUX0
#define SDC1_CMD_CLK_DRIVING			        IO_DRIVING_8mA

/* I2S0 PINMUX */
#define I2S0_MUX0_RX							MUX0//I2S_MCLK_LR_BCK_TX__IOB0_IOB1_IOB2_IOB3
#define I2S0_MUX1_TX							MUX1//I2S_MCLK_LR_BCK_TX__IOB4_IOB5_IOB6_IOB7

#define I2S0_POS								I2S0_MUX0_RX
#define I2S0_POS_DRIVING					    IO_DRIVING_8mA

/* I2S1 PINMUX */
#define I2S1_MUX0_RX							MUX0//I2S_MCLK_LR_BCK_TX__IOB8_IOB9_IOB10_IOB11
#define I2S1_MUX1_TX							MUX1//I2S_MCLK_LR_BCK_TX__IOB12_IOB13_IOB14_IOB15

#define I2S1_POS								I2S1_MUX0_RX
#define I2S1_POS_DRIVING					    IO_DRIVING_8mA

/* I2S2 PINMUX */
#define I2S2_MUX0_RX							MUX0//I2S_MCLK_LR_BCK_TX__IOC4_IOC5_IOC6_IOC7
#define I2S2_MUX1_TX							MUX1//I2S_MCLK_LR_BCK_TX__IOC21_IOC13_IOD14_IOD15

#define I2S2_POS								I2S2_MUX0_RX
#define I2S2_POS_DRIVING					    IO_DRIVING_8mA

/* I2S3 PINMUX */
#define I2S3_MUX0_RX							MUX0//I2S_MCLK_LR_BCK_TX__IOD6_IOD7_IOD8_IOD9
#define I2S3_MUX1_TX							MUX1//I2S_MCLK_LR_BCK_TX__IOD10_IOD11_IOD12_IOD13

#define I2S3_POS								I2S3_MUX0_RX
#define I2S3_POS_DRIVING					    IO_DRIVING_8mA

/* NAND DATA PINMUX */
#define NADN_DATA_MUX0						    MUX0//NAND_DATA_0_7__IOB8_IOB15
#define NADN_DATA_MUX1						    MUX1//NANN_DATA_0_7__IOC4_IOC11
#define NAND_DATA_POS							NADN_DATA_MUX0
#define NAND_DATA_DRIVING					    IO_DRIVING_8mA
#define NAND_CTRL_DRIVING					    IO_DRIVING_8mA

/* TFT PIN MUX	*/
#define TFT_DATA0_7_MUX0					    MUX0//TFT_DATA_0_7__IOA0_IOA7
#define TFT_DATA0_7_MUX1					    MUX1//TFT_DATA_0_7__IOE0_IOE7

#define TFT_DATA8_15_MUX0					    MUX0//TFT_DATA_8_15__IOA8_IOA15
#define TFT_DATA8_15_MUX1					    MUX1//TFT_DATA_8_15__IOE0_E7

#define TFT_DATA_0_7_POS					    TFT_DATA0_7_MUX0
#define TFT_DATA_8_15_POS					    TFT_DATA8_15_MUX0
#define TFT_DATA_DIRVING					    IO_DRIVING_8mA
#define TFT_CLK_DRIVING						    IO_DRIVING_8mA

#define TFT_CTRL_MUX0							MUX0//TFT_CTRL_DE_HSYNC_VSYNC_CLK__IOB0_IOB1_IOB2_IOB3
#define TFT_CTRL_MUX1							MUX1//TFT_CTRL_DE_HSYNC_VSYNC_CLK__IOC8_IOC9_IOC10_IOC11
#define TFT_CTRL_POS							TFT_CTRL_MUX0
#define TFT_CLK_DRIVING						    IO_DRIVING_8mA

/*	CSI PIN	MUX	*/
#define CSI_MIPI_CLKO_MUX0                      MUX0//CSI_MIPI_CLKO__IOC9
#define CSI_MIPI_CLKO_MUX1                      MUX1//CSI_MIPI_CLKO__IOD7
#define CSI_MIPI_CLKO_MUX2                      MUX2//CSI_MIPI_CLKO__IOD12
#define CSI_MIPI_CLKO_POS                       CSI_MIPI_CLKO_MUX0
#define CSI_MIPI_CLKO_DRIVING					IO_DRIVING_8mA

#define CSI_DATA2_9_MUX0					    MUX0//CSI_DATA_2_9__IOC0_IOC7
#define CSI_DATA2_9_MUX1					    MUX1//CSI_DATA_2_9__IOB8_IOB15
#define CSI_DATA2_9_MUX2					    MUX2//CSI_DATA_2_9__IOE0_IOE7
#define CSI_DATA_2_9_POS					    CSI_DATA2_9_MUX0

#define CSI_CTRL_MUX0							MUX0//CSI_CTRL_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11
#define CSI_CTRL_MUX1							MUX1//CSI_CTRL_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9
#define CSI_CTRL_POS							CSI_CTRL_MUX0



/*	CDSP PIN	MUX	*/
#define CDSP_CLKO_MUX0                          MUX0//CDSP_CLKO__IOC9
#define CDSP_CLKO_MUX1                          MUX1//CDSP_CLKO__IOD7
#define CDSP_CLKO_MUX2                          MUX2//CDSP_CLKO__IOD12
#define CDSP_CLKO_POS                           CDSP_CLKO_MUX1
#define CDSP_CLKO_DRIVING					    IO_DRIVING_8mA

#define CDSP_DATA2_9_MUX0					    MUX0//CSI_DATA_2_9__IOC0_IOC7
#define CDSP_DATA2_9_MUX1					    MUX1//CSI_DATA_2_9__IOB8_IOB15
#define CDSP_DATA2_9_MUX2					    MUX2//CSI_DATA_2_9__IOE0_IOE7
#define CDSP_DATA_2_9_POS					    CDSP_DATA2_9_MUX1

#define CDSP_DATA0_1_MUX0					    MUX0//CSI_DATA_0_1__IOB6_IOB7
#define CDSP_DATA0_1_MUX1					    MUX1//CSI_DATA_0_1__IOB4_IOB5
#define CDSP_DATA0_1_MUX2					    MUX2//CSI_DATA_0_1__IOC12_IOC13
#define CDSP_DATA_0_1_POS					    CDSP_DATA0_1_MUX0

#define CDSP_CTRL_MUX0							MUX0//CSI_CTRL_CLKI_HSYNC_VSYNC__IOC8_IOC10_IOC11
#define CDSP_CTRL_MUX1							MUX1//CSI_CTRL_CLKI_HSYNC_VSYNC__IOD6_IOD8_IOD9
#define CDSP_CTRL_POS							CDSP_CTRL_MUX1



/* CAN PINMUX */
#define CAN_MUX0							    MUX0//CAN_TX_RX__IOC0_IOC1
#define CAN_MUX1							    MUX1//CAN_TX_RX__IOC2_IOC3
#define CAN_POS						            CAN_MUX0
#define CAN_DRIVING     					    IO_DRIVING_8mA

/* SMCARD RST Pin PINMUX */
#define SM_RST_MUX0							    MUX0//SMCARD_RST__IOB5
#define SM_RST_MUX1							    MUX1//SMCARD_RST__IOB6
#define SM_RST_POS						        SM_RST_MUX0
#define SM_RST_DRIVING     					    IO_DRIVING_8mA

/* CEC Control Pin PINMUX */
#define CEC_CTRL_MUX0							MUX0//CEC_CTRL__IOC0
#define CEC_CTRL_MUX1							MUX1//CEC_CTRL__IOC12
#define CEC_CTRL_POS						    CEC_CTRL_MUX0
/************************************************************/
/*					function 								*/
/************************************************************/
extern void board_init(void);

#endif
