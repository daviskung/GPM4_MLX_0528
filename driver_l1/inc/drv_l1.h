#ifndef __DRIVER_L1_H__
#define __DRIVER_L1_H__

#include "project.h"
#include "cmsis_os.h"

#define		BIT0		0x00000001
#define		BIT1		0x00000002
#define		BIT2		0x00000004
#define		BIT3		0x00000008
#define		BIT4		0x00000010
#define		BIT5		0x00000020
#define		BIT6		0x00000040
#define		BIT7		0x00000080
#define		BIT8		0x00000100
#define		BIT9		0x00000200
#define		BIT10		0x00000400
#define		BIT11		0x00000800
#define		BIT12		0x00001000
#define		BIT13		0x00002000
#define		BIT14		0x00004000
#define		BIT15		0x00008000
#define		BIT16		0x00010000
#define		BIT17		0x00020000
#define		BIT18		0x00040000
#define		BIT19		0x00080000
#define		BIT20		0x00100000
#define		BIT21		0x00200000
#define		BIT22		0x00400000
#define		BIT23		0x00800000
#define		BIT24		0x01000000
#define		BIT25		0x02000000
#define		BIT26		0x04000000
#define		BIT27		0x08000000
#define		BIT28		0x10000000
#define		BIT29		0x20000000
#define		BIT30		0x40000000
#define		BIT31		0x80000000

extern void drv_l1_init(void);

/********************* Define R_SYSTEM_CTRL bit mask (0xD000000C, System Control Register) *****************/
#define MASK_SYSCTL_CSICLK_DIV					BIT14	/* Sensor Master Clock Division, 0:Sensor Master Clock / 1,1: Sensor Master Clock / 2 */
#define MASK_SYSCTL_SEL30K						BIT12	/* 0: disable, 1: enable */
#define MASK_SYSCTL_SEN_48M_SEL					BIT11	/* Sensor clock source, 0: system clock/2, 1: XCKGEN_48M/2 */
#define MASK_SYSCTL_CSICLK_EN					BIT8	/* CSI Clock Enable */
#define MASK_SYSCTL_HNRC						BIT1	/* 0: normal halt mode, 1: The next instruction is continued after the CPU is waked up from the Halt mode */
#define MASK_SYSCTL_SEL_REAL32K					BIT0	/* 0: from XCKGEN48M, 1: from RTC's 32768 */

/********************* Define R_SYSTEM_MISC_CTRL0 bit mask (0xD0000008, SYSTEM MISC Control Register 0) *****************/
#define MASK_SYSMISC_CTL0_SW_PHY_AFE_RESET		BIT15	/* Software Control USB PHY AFE_RESET CONTROL Singal, 1: reset */
#define MASK_SYSMISC_CTL0_SW_PHY_SIM_MODE		BIT14	/* Software Control USB PHY SIM_MODE CONTROL Singal, 1: go Sim Mode */
#define MASK_SYSMISC_CTL0_SW_PHY_SLUMBER		BIT13	/* Software Control USB PHY SLUMBER CONTROL Singal, 1: go SLUMBER Mode */
#define MASK_SYSMISC_CTL0_SW_PHY_PARTIAL		BIT12	/* Software Control USB PHY PARTIAL CONTROL Singal, 1: Go PARTIAL Mode */
#define MASK_SYSMISC_CTL0_SW_PHY_RESET			BIT11	/* Software Control USB PHY RESET CONTROL Singal, 1: reset */
#define MASK_SYSMISC_CTL0_SW_PHY_DPDM_PD		BIT10	/* Software Control USB PHY_DPDM_PD CONTROL Singal, 1: DPDM Pull Down */
#define MASK_SYSMISC_CTL0_HOST_SW				BIT8	/* PHY Connect Host or Device, 0: switch device, 1: switch host */
#define MASK_SYSMISC_CTL0_SEN2CDSP_CLKO_INV		BIT5	/* Invert CDSP Sensor Clock,1: invert */
#define MASK_SYSMISC_CTL0_SEN2CDSP_CLKO_EN		BIT4	/* Enable CDSP Sensor Clock, 0: disable, 1: enable */
#define MASK_SYSMISC_CTL0_CDSP_SOURCE_MIPI		BIT3	/* set CDSP source is from mipi interface */
#define MASK_SYSMISC_CTL0_CDSP_SOURCE_SENSOR	BIT2	/* set cdsp source is from parallel sensor */
#define MASK_SYSMISC_CTL0_CDSP_YUV_MODE			BIT1	/* CDSP Source format, 0: Bayer Raw, 1: YUV mode */
#define MASK_SYSMISC_CTL0_PPU_MIPI_SW			BIT0	/* PPU Sensor Source Swich, 0: from sensor, 1: from MIPI */

/********************* Define R_SYSTEM_MISC_CTRL4 bit mask (0xD0000084, SYSTEM MISC Control Register 4) *****************/
#define MASK_SYSMISC_CTL4_MAP_0_TO_SPIF			BIT15	/* In Internal boot ROM code and dis_bootremap is true, map address 0 to => 0: SD RAM, 1: SPI flash */
#define MASK_SYSMISC_CTL4_CDSP_SEN_SEL			BIT4	/* Internal data path selection, 0: cdsp_w->mux3m2m_cdsp_W, sen->x1_csi_mux, 1: cdsp_w->x1_csi_mux, sen->mux3m2m_cdsp_W */

/********************* R_FUNPOS1: Sensor CLKO source selection for pin location *****************/
#define MCLK_IOC9                               BIT20
#define MCLK_IOD7                               BIT21
#define MCLK_IOD12                              BIT22

#define DUMMY_BUFFER_ADDRS			0x50000000

#define BIT_BAND_ALIAS(addr, bitnum)           ((((unsigned int)(addr)) & 0xf0000000) + 0x2000000 + ((((unsigned int)(addr)) & 0x0000ffff)<<5) + ((bitnum)<<2))
#define BIT_BAND_REGISTER_ALIAS(addr, bitnum)  ((((unsigned int)(addr)) & 0x40000000) + 0x2000000 + ((((unsigned int)(addr)) & 0x0000ffff)<<5) + ((bitnum)<<2))

#endif  // __DRIVER_L1_H__

