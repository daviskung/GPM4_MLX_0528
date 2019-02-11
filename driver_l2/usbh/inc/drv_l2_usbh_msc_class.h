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
 * @file		drv_l2_usbh_msc_class.h
 * @brief		Driver layer2 for mess storage class header file.
 * @author		Dunker Chen
 */
 
#ifndef __drv_l2_USBH_MSC_CLASS_H__
#define __drv_l2_USBH_MSC_CLASS_H__
 
/**************************************************************************
 *                         H E A D E R   F I L E S                        *
**************************************************************************/

#include "drv_l2_usbh.h"

/**************************************************************************
*                           C O N S T A N T S                             *
**************************************************************************/

#define MSC_TEST_UNIT_READY	0x00
#define MSC_REQUEST_SENSE	0x03
#define MSC_INQUIRY			0x12
#define MSC_READ_CAPACITY	0x25
#define MSC_READ10			0x28
#define MSC_READ12			0xA8
#define MSC_WRITE10			0x2A
#define MSC_WRITE12			0xAA
#define MSC_MODE_SENSE6		0x1A
#define MSC_MODE_SENSE10	0x5A

#define MSC_SCSI_STATUS_OK			0x20
#define MSC_SCSI_STATUS_FAIL		0x21
#define MSC_SCSI_STATUS_PHASE_ERROR	0x22

extern USBH_CLASS_OPERATION	g_usbh_msc_operation;

/**************************************************************************
*                          D A T A    T Y P E S                           *
**************************************************************************/

typedef struct
{
	INT8U					bulk_in;
	INT8U					bulk_in_toggle;
	INT8U					bulk_out;
	INT8U					bulk_out_toggle;
	INT8U 					int_in;
	INT8U					int_in_toggle;
	INT16U 					bulk_in_max;
	INT16U 					bulk_out_max;
	INT16U					int_in_max;
	INT8U					int_in_interval;
}USBH_MSC_INFO;

typedef struct{
	INT8U	status;
	INT8U	lun;
	INT8U	cdb_length;
	INT8U	data_dir;
	void*	data_buffer;
	INT32U	data_transfer_length;
	INT8U	cdb[16];
	INT8U* pErrorCode;
}BULK_ONLY_CTRL;

typedef struct
{
	USBH_DEV_INFO	*dev;
	INT8U			interface;
	INT8U			lun;
	INT8U			wp;
	INT8U			present;
	INT32U			capacity;
	INT32U			sector_size;
}USBH_DISK_INFO;

/**************************************************************************
*               F U N C T I O N    D E C L A R A T I O N S                *
**************************************************************************/

/**
* @brief 		Bulk only protocol transaction.
* @param 		msc[in]: Mess storage device information.
* @param 		bulk_ctrl[in]: Bulk only control.
* @return		Tag.
*/
extern INT32U drv_l2_usbh_msc_bulk_only_trans(
	USBH_DEV_INFO *dev,
	INT8U interface_number,
	BULK_ONLY_CTRL* bulk_ctrl);

/**
* @brief 		Alloc disk.
* @return		NULL means no free disk memory, others means success.
*/
extern USBH_DISK_INFO* drv_l2_usbh_msc_alloc_disk(void);

/**
* @brief 		Free disk memory.
* @param 		disk[in]: Disk information.
* @return		None.
*/
extern void drv_l2_usbh_msc_free_disk(
	USBH_DISK_INFO* disk);

/**
* @brief 		Get all disk number.
* @return		Disk number.
*/
extern INT32U drv_l2_usbh_msc_get_disk_number(void);

/**
* @brief 		Check disk ready or not.
* @param 		disk[in]: Disk information.
* @return		FALSE means command fail or not ready, SUCCESS means disk ready.
*/
extern INT32U drv_l2_usbh_msc_check_disk_ready(
	USBH_DISK_INFO* disk);

/**
* @brief 		Get disk handle.
* @param 		diskno[in]: Disk number.
* @return		NULL means no disk, other means SUCCESS.
*/
extern USBH_DISK_INFO* drv_l2_usbh_msc_get_disk_handle(
	INT32U diskno);
	
/**
* @brief 		USB mess storage device identify.
* @param 		dev[in]: Device information.
* @param 		interface_number[in]: Interface number.
* @return		FALSE means command fail or not ready, SUCCESS means disk ready.
*/
extern INT32U drv_l2_usbh_msc_identify(
	USBH_DEV_INFO *dev,
	INT8U interface_number);

/**
* @brief 		USB host mess storage class enumeration.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_enumeration(void);

/**
* @brief 		USB host mess storage class release resource.
* @return		None.
*/
extern void drv_l2_usbh_msc_release(void);

/**
* @brief 		USB mess storage device read sector.
* @param 		disk[in]: Disk information.
* @param 		StartLBA[in]: Start LBA.
* @param 		SectorCnt[in]: Sector number, length.
* @param 		buf[in]: Data buffer pointer.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_read_sector(
	USBH_DISK_INFO *disk,
	INT32U StartLBA,
	INT32U SectorCnt,
	void* buf);

/**
* @brief 		USB mess storage device write sector.
* @param 		disk[in]: Disk information.
* @param 		StartLBA[in]: Start LBA.
* @param 		SectorCnt[in]: Sector number, length.
* @param 		buf[in]: Data buffer pointer.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_write_sector(
	USBH_DISK_INFO *disk,
	INT32U StartLBA,
	INT32U SectorCnt,
	void* buf);

/**
* @brief 		SCSI cpmmand: inquiry.
* @param 		disk[in]: Disk information.
* @param 		buf[in]: Data buffer pointer.
* @param 		ln[in]: Data buffer length.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_scsi_inquiry(
	USBH_DISK_INFO *disk,
	void * buf,
	INT32U ln,
	INT8U* pErrorCode
	);

/**
* @brief 		SCSI cpmmand: test unit ready.
* @param 		disk[in]: Disk information.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_scsi_test_unit_ready(
	USBH_DISK_INFO *disk,
	INT8U* pErrorCode
	);

/**
* @brief 		SCSI cpmmand: request sense.
* @param 		disk[in]: Disk information.
* @param 		buf[in]: Data buffer pointer.
* @param 		ln[in]: Data buffer length.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_scsi_request_sense(
	USBH_DISK_INFO *disk,
	void * buf,
	INT32U ln,
	INT8U* pErrorCode
	);
	
/**
* @brief 		SCSI cpmmand: read capacity.
* @param 		disk[in]: Disk information.
* @param 		buf[in]: Data buffer pointer.
* @param 		ln[in]: Data buffer length.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_scsi_read_capacity(
	USBH_DISK_INFO *disk,
	void * buf,
	INT32U ln,
	INT8U* pErrorCode
	);

/**
* @brief 		SCSI cpmmand: mode sense.
* @param 		disk[in]: Disk information.
* @param 		page_mode[in]: Page mode.
* @param 		buf[in]: Data buffer pointer.
* @param 		ln[in]: Data buffer length.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/	
extern INT32U drv_l2_usbh_msc_scsi_mode_sense(
	USBH_DISK_INFO *disk,
	INT8U	page_mode,
	void * buf,
	INT32U ln,
	INT8U* pErrorCode
	);
	
/**
* @brief 		SCSI cpmmand: read.
* @param 		disk[in]: Disk information.
* @param 		StartLBA[in]: Start LBA.
* @param 		SectorCnt[in]: Sector number, length.
* @param 		buf[in]: Data buffer pointer.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_scsi_read(
	USBH_DISK_INFO *disk,
	INT32U StartLBA,
	INT32U SectorCnt,
	void * buf,
	INT8U* pErrorCode
	);
	
/**
* @brief 		SCSI cpmmand: write.
* @param 		disk[in]: Disk information.
* @param 		StartLBA[in]: Start LBA.
* @param 		SectorCnt[in]: Sector number, length.
* @param 		buf[in]: Data buffer pointer.
* @param 		pErrorCode[in]: Error code.
* @return		TRUE: success, FALSE: false.
*/
extern INT32U drv_l2_usbh_msc_scsi_write(
	USBH_DISK_INFO *disk,
	INT32U StartLBA,
	INT32U SectorCnt,
	void * buf,
	INT8U* pErrorCode
	);
	
#endif /* __drv_l2_USBH_MSC_CLASS_H__ */
