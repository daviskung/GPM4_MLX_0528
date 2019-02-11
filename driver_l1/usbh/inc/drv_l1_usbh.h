/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2012 by Generalplus Inc.                         *
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
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park,                      *
 *  Hsinchu City 30077, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
/**
 * @file		drv_l1_usbh.h
 * @brief		Driver layer1 for usb controller header file.
 * @author		Dunker Chen
 */

#ifndef __drv_l1_USBH_H__
#define __drv_l1_USBH_H__

/**************************************************************************
 *                         H E A D E R   F I L E S                        *
**************************************************************************/

#include "project.h"
#include "gplib.h"

/**************************************************************************
*                           C O N S T A N T S                             *
**************************************************************************/

#define TEST_USBH	0

//#define USBH0_BASE	0x93004000
//#define USBH1_BASE	0x93005000

#define USBH0_BASE	0xD0C00000
#define USBH1_BASE	0xD0C00000

#if TEST_USBH == 0
	#define USBH_BASE	USBH0_BASE
#else
	#define USBH_BASE	USBH1_BASE
#endif

/* ------ USBH error code define ----- */
#define USBH_E_NoErr		0x00
#define USBH_E_RX			0x01
#define USBH_E_TX			0x02
#define USBH_E_NACK			0x03
#define USBH_E_Stall		0x04
#define USBH_E_UnPID		0x05
#define USBH_E_BitStuff		0x06
#define USBH_E_DataSeq		0x07
#define USBH_E_CRC16		0x08
#define USBH_E_TimeOut		0x09
#define USBH_E_Para			0x0A
#define USBH_E_DMA			0x10
#define USBH_E_Remove		0xFF

/* ------------------------------------*/
#define DEV_CONNECT 2
#define DEV_DISCONNECT 3

/**************************************************************************
*                          D A T A    T Y P E S                           *
**************************************************************************/

struct ehci_regs {

	/* USBCMD: offset 0x00 */
	volatile INT32U		command;
/* 23:16 is r/w intr rate, in microframes; default "8" == 1/msec */
#define CMD_PARK	(1<<11)		/* enable "park" on async qh */
#define CMD_PARK_CNT(c)	(((c)>>8)&3)	/* how many transfers to park for */
#define CMD_LRESET	(1<<7)		/* partial reset (no ports, etc) */
#define CMD_IAAD	(1<<6)		/* "doorbell" interrupt async advance */
#define CMD_ASE		(1<<5)		/* async schedule enable */
#define CMD_PSE		(1<<4)		/* periodic schedule enable */
/* 3:2 is periodic frame list size */
#define CMD_RESET	(1<<1)		/* reset HC not bus */
#define CMD_RUN		(1<<0)		/* start/stop HC */

	/* USBSTS: offset 0x04 */
	volatile INT32U		status;
#define STS_ASS		(1<<15)		/* Async Schedule Status */
#define STS_PSS		(1<<14)		/* Periodic Schedule Status */
#define STS_RECL	(1<<13)		/* Reclamation */
#define STS_HALT	(1<<12)		/* Not running (any reason) */
/* some bits reserved */
	/* these STS_* flags are also intr_enable bits (USBINTR) */
#define STS_IAA		(1<<5)		/* Interrupted on async advance */
#define STS_FATAL	(1<<4)		/* such as some PCI access errors */
#define STS_FLR		(1<<3)		/* frame list rolled over */
#define STS_PCD		(1<<2)		/* port change detect */
#define STS_ERR		(1<<1)		/* "error" completion (overflow, ...) */
#define STS_INT		(1<<0)		/* "normal" completion (short, ...) */

	/* USBINTR: offset 0x08 */
	volatile INT32U		intr_enable;

	/* FRINDEX: offset 0x0C */
	volatile INT32U		frame_index;	/* current microframe number */
	/* CTRLDSSEGMENT: offset 0x10 */
	volatile INT32U		segment;	/* address bits 63:32 if needed */
	/* PERIODICLISTBASE: offset 0x14 */
	volatile INT32U		frame_list;	/* points to periodic list */
	/* ASYNCLISTADDR: offset 0x18 */
	volatile INT32U		async_next;	/* address of next async queue head */

	volatile INT32U		reserved [9];

	/* CONFIGFLAG: offset 0x40 */
	volatile INT32U		configured_flag;
#define FLAG_CF		(1<<0)		/* true: we'll support "high speed" */

	/* PORTSC: offset 0x44 */
	volatile INT32U		port_status [1];	/* up to N_PORTS */
/* 31:23 reserved */
#define PORT_WKOC_E	(1<<22)		/* wake on overcurrent (enable) */
#define PORT_WKDISC_E	(1<<21)		/* wake on disconnect (enable) */
#define PORT_WKCONN_E	(1<<20)		/* wake on connect (enable) */
/* 19:16 for port testing */
#define PORT_TEST_PKT	(0x4<<16)	/* Port Test Control - packet test */
#define PORT_LED_OFF	(0<<14)
#define PORT_LED_AMBER	(1<<14)
#define PORT_LED_GREEN	(2<<14)
#define PORT_LED_MASK	(3<<14)
#define PORT_OWNER	(1<<13)		/* true: companion hc owns this port */
#define PORT_POWER	(1<<12)		/* true: has power (see PPC) */
#define PORT_USB11(x) (((x)&(3<<10)) == (1<<10))	/* USB 1.1 device */
/* 11:10 for detecting lowspeed devices (reset vs release ownership) */
/* 9 reserved */
#define PORT_RESET	(1<<8)		/* reset port */
#define PORT_SUSPEND	(1<<7)		/* suspend port */
#define PORT_RESUME	(1<<6)		/* resume it */
#define PORT_OCC	(1<<5)		/* over current change */
#define PORT_OC		(1<<4)		/* over current active */
#define PORT_PEC	(1<<3)		/* port enable change */
#define PORT_PE		(1<<2)		/* port enable */
#define PORT_CSC	(1<<1)		/* connect status change */
#define PORT_CONNECT	(1<<0)		/* device connected */
#define PORT_RWC_BITS   (PORT_CSC | PORT_PEC | PORT_OCC)
} /*__attribute__ ((packed))*/;

/*
 * EHCI Specification 0.95 Section 3.5
 * QTD: describe data transfer components (buffer, direction, ...)
 * See Fig 3-6 "Queue Element Transfer Descriptor Block Diagram".
 *
 * These are associated only with "QH" (Queue Head) structures,
 * used with control, bulk, and interrupt transfers.
 */
struct ehci_qtd {
	/* first part defined by EHCI spec */
	struct ehci_qtd*		hw_next;	/* see EHCI 3.5.1 */
	struct ehci_qtd*		hw_alt_next;    /* see EHCI 3.5.2 */
	INT32U			hw_token;       /* see EHCI 3.5.3 */
#define	QTD_TOGGLE	(1 << 31)	/* data toggle */
#define	QTD_LENGTH(tok)	(((tok)>>16) & 0x7fff)
#define	QTD_IOC		(1 << 15)	/* interrupt on complete */
#define	QTD_CERR(tok)	(((tok)>>10) & 0x3)
#define	QTD_PID(tok)	(((tok)>>8) & 0x3)
#define	QTD_STS_ACTIVE	(1 << 7)	/* HC may execute this */
#define	QTD_STS_HALT	(1 << 6)	/* halted on error */
#define	QTD_STS_DBE	(1 << 5)	/* data buffer error (in HC) */
#define	QTD_STS_BABBLE	(1 << 4)	/* device was babbling (qtd halted) */
#define	QTD_STS_XACT	(1 << 3)	/* device gave illegal response */
#define	QTD_STS_MMF	(1 << 2)	/* incomplete split transaction */
#define	QTD_STS_STS	(1 << 1)	/* split transaction state */
#define	QTD_STS_PING	(1 << 0)	/* issue PING? */

	INT32U*			hw_buf [5];        /* see EHCI 3.5.4 */

} /*__attribute__ ((__aligned (32)))*/;

/*
 * EHCI Specification 0.95 Section 3.6
 * QH: describes control/bulk/interrupt endpoints
 * See Fig 3-7 "Queue Head Structure Layout".
 *
 * These appear in both the async and (for interrupt) periodic schedules.
 */

/* first part defined by EHCI spec */
struct ehci_qh {
	struct ehci_qh*	next;	/* see EHCI 3.6.1 */
	INT32U			info1;       /* see EHCI 3.6.2 */
#define	QH_HEAD		0x00008000
	INT32U			info2;        /* see EHCI 3.6.2 */
#define	QH_SMASK	0x000000ff
#define	QH_CMASK	0x0000ff00
#define	QH_HUBADDR	0x007f0000
#define	QH_HUBPORT	0x3f800000
#define	QH_MULT		0xc0000000
	struct ehci_qtd*	qtd_current;	/* qtd list - see EHCI 3.6.4 */
	/* qtd overlay (hardware parts of a struct ehci_qtd) */
	struct ehci_qtd*	qtd_next;
	struct ehci_qtd*	alt_next;
	INT32U			hw_token;
	INT32U*			hw_buf [5];
}/* __attribute__ ((aligned(32)))*/;

/*
 * EHCI Specification 0.95 Section 3.3
 * Fig 3-4 "Isochronous Transaction Descriptor (iTD)"
 *
 * Schedule records for high speed iso xfers
 */
struct ehci_itd {
	/* first part defined by EHCI spec */
	INT32U			hw_next;           /* see EHCI 3.3.1 */
	INT32U			hw_transaction[8]; /* see EHCI 3.3.2 */
#define EHCI_ISOC_ACTIVE        (1<<31)        /* activate transfer this slot */
#define EHCI_ISOC_BUF_ERR       (1<<30)        /* Data buffer error */
#define EHCI_ISOC_BABBLE        (1<<29)        /* babble detected */
#define EHCI_ISOC_XACTERR       (1<<28)        /* XactErr - transaction error */
#define	EHCI_ITD_LENGTH(tok)	(((tok)>>16) & 0x0fff)
#define	EHCI_ITD_IOC		    (1 << 15)	/* interrupt on complete */

	INT32U			hw_bufptr[7];	/* see EHCI 3.3.3 */
#if 0
	/* the rest is HCD-private */
	dma_addr_t		itd_dma;	/* for this itd */
	union ehci_shadow	itd_next;	/* ptr to periodic q entry */
	struct urb		*urb;
	struct ehci_iso_stream	*stream;	/* endpoint's queue */
	struct list_head	itd_list;	/* list of stream's itds */

	/* any/all hw_transactions here may be used by that urb */
	unsigned		frame;		/* where scheduled */
	unsigned		pg;
	unsigned		index[8];	/* in urb->iso_frame_desc */
#endif
} /*__attribute__ ((aligned (32)))*/;

typedef struct
{
	INT8U			cAddr;			// USB address
	INT8U			cEndp;			// USB endpoint
	INT8U			cDMAeEn;		// DMA enable
	INT8U*			cTog;			// USB data toggle
	void *		 	pBuf;			// data buffer address
	INT32U  		uiLn;			// buffer length
	INT32U			uiMaxPkt;		// maximum packet size
	INT8U           cMulti;
	INT32U*			pAct;			// active length
	INT8U*			pErrorCode;
	INT32U* 		offset;			// Buffer offset (for DMA use)
}ST_USBH_CTRL;

/**************************************************************************
*               F U N C T I O N    D E C L A R A T I O N S                *
**************************************************************************/

extern void print_string(CHAR *fmt, ...);

/**
* @brief 		Hardware reset USB host controller.
* @return		None.
*/
extern void drv_l1_usbh_hw_reset(void);

/**
* @brief 		Get SOF number.
* @return		SOF number.
*/
extern INT32U drv_l1_usbh_get_frame_num(void);

/**
* @brief 		Wait USB host intrrrupt flag.
* @param 		uiPollBit[in]: Polling flag.
* @return		TRUE: success, FALSE: timeout or device plug out.
*/
extern INT32U drv_l1_usbh_wait_int(
	INT32U uiPollBit);

/**
* @brief 		Wait USB SOF tick. (high speed: 125us per tick, full speed 1ms per tick)
* @param 		nSOF[in]: Number of SOF.
* @return		None.
*/
extern void drv_l1_usbh_wait_sof_tick(
	INT32U nSOF);

/**
* @brief 		Wait USB ms tick.
* @param 		ms[in]: mini sec.
* @return		None.
*/
extern void drv_l1_usbh_wait_ms(
	INT32U ms);

/**
* @brief 		Get USB host status.
* @param 		pErrorCode[out]: Error code.
* @return		Error code.
*/
extern INT8U drv_l1_usbh_get_status(void);

/**
* @brief 		Wait USB host Data transaction finish.
* @param 		pid[in]: USB packet id.
* @return		0 means success, others means error code.
*/
extern INT8U drv_l1_usbh_data_trans(
	INT32U pid);

/**
* @brief 		Issue reset signal to device.
* @param 		uiTick[in]: reset duration. (unit: ms)
* @return		None.
*/
extern void drv_l1_usbh_issue_reset(
	INT32U uiTick);

/**
* @brief 		USB host initial.
* @return		None.
*/
extern void drv_l1_usbh_init(void);

/**
* @brief 		USB host un-initial.
* @return		None.
*/
extern void drv_l1_usbh_un_init(void);

/**
* @brief 		Check USB device plug in.
* @return		TRUE: plug in, FALSE: plug out.
*/
extern INT32U drv_l1_usbh_check_device_plug(void);

/**
* @brief 		Get device speed mode.
* @return		TRUE: high speed, FALSE: full speed.
*/
extern INT32U drv_l1_usbh_get_speed(void);

/**
* @brief 		Get device speed mode.
* @return		TRUE: high speed, FALSE: full speed.
*/
extern INT32U drv_l1_usbh_detect(void);

/**
* @brief 		Issue setup request.
* @param 		Ctrl[in]: Control setting.
* @return		TRUE: success, FALSE: fail.
*/
extern INT32U drv_l1_usbh_issue_setup(
	ST_USBH_CTRL *Ctrl);

/**
* @brief 		Transfer out data.
* @param 		Ctrl[in]: Control setting.
* @return		TRUE: success, FALSE: fail.
*/
extern INT32U drv_l1_usbh_out_token(
	ST_USBH_CTRL *Ctrl);

/**
* @brief 		Receive in data.
* @param 		Ctrl[in]: Control setting.
* @return		TRUE: success, FALSE: fail.
*/
extern INT32U drv_l1_usbh_in_token(
	ST_USBH_CTRL *Ctrl);

/**
* @brief 		Receive in data for interrupt endpoint (without NAK re-try).
* @param 		Ctrl[in]: Control setting.
* @return		TRUE: success, FALSE: fail.
*/
extern INT32U drv_l1_usbh_int_in_token(
	ST_USBH_CTRL *Ctrl);

/**
* @brief 		Receive in data for isochronous endpoint.
* @param 		Ctrl[in]: Control setting.
* @return		TRUE: success, FALSE: fail.
*/
extern INT32U drv_l1_usbh_iso_in_token(
	ST_USBH_CTRL *Ctrl);


extern INT32U drv_l1_usbh_ehci_chk_connect_status_change(void);

extern INT32U drv_l1_usbh_ehci_chk_current_connect_status(void);

#endif /*__drv_l1_USBH_H__*/

