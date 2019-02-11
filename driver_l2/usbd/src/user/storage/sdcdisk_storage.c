/***************************************************************************
* sdcdisk_storage.c
*
* Purpose: This is used for USB MSDC to access the storage read/write and others operations
*
* Author:
*
* Date: 2012/06/19
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
****************************************************************************/
#include "project.h"
#include "stdio.h"
#include "gplib_mm_gplus.h"
#include "drv_l1_cache.h"
#include "drv_l2_usbd_msdc.h"
#include "drv_l1_cfg.h"
#include "drv_l2_sd.h"

#define K_SD_MSDC_READ_FLUSH	0	// when 1 mean always send SD stop command when read, this ensure sd card dont keep data at its FIFO but slow down performance
#define K_SD_MSDC_WRITE_FLUSH	0	// when 1 mean always send SD stop command when write, this ensure sd card dont keep data at its FIFO but slow down performance

#define SD_MSDC_ERR_PRINT         DBG_PRINT
#define SD_MSDC_PRINT(...)
//#define SD_MSDC_PRINT           DBG_PRINT

#define C_SD_MSDC_NO_ERR                  0
#define C_SD_MSDC_GENERAL_ERR             -1
#define C_SD_MSDC_CARD_REMOVED_ERR        -2
#define C_SD_MSDC_LOGIC_ERR               -3

static INT32U sd_msdc_offset[2] = {0, 0};
static INT32U msdc_sd_lba[2];
static INT32U msdc_sd_seccount[2];
static INT32S msdc_sd_error[2] = {C_SD_MSDC_NO_ERR, C_SD_MSDC_NO_ERR};

/*****************************************************
    Functions API
*****************************************************/
/*****************************************************
    RAMDISK API
*****************************************************/
#define SDC_TAG0		            0x73644750		// string: PGds
#define SDC_TAG1		            0x64636373		// string: sccd
#define SDC_TAG2		            0x7364			// string: ds
#define APP_TAG			            0x5047			// string: GP

static void SDCARD_Set_Error(INT32U device_id, INT32S error_code)
{
    msdc_sd_error[device_id] = error_code;
}

// Note, call this to get error detail only when other msdc function return error
INT32S SDCARD_Get_Error(INT32U device_id)
{
    return msdc_sd_error[device_id];
}

static INT32U sdcard_boot_parser_header(INT32U device_id)
{
	INT32U buf[128];
	INT16U *u16_buf = (INT16U *)buf;
	INT16U boot_size;
	INT32U app_size = 0, data_start_addr = 0;
	INT32S ret;

	/* ----- Read boot header ----- */
	if( (ret = drvl2_sd_read(device_id, 0x00, buf, 1)) !=0 )
		return 0;
	/* ----- check boot header tag ----- */
	if( (buf[0] != SDC_TAG0) || (buf[1] != SDC_TAG1) || ( u16_buf[4] != SDC_TAG2))
		return 0;
	/* ----- Get boot area size ----- */
	boot_size = u16_buf[8];
	if(boot_size == 0)
		return 0;
	/* ----- Read APP header ----- */
	if( (ret = drvl2_sd_read(device_id, boot_size, buf, 1 )) !=0 )
		return 0;
	/* ----- Check APP header tag ----- */
	if( u16_buf[0] != APP_TAG )
		return 0;
	/* ----- Get APP area size ----- */
	app_size = (u16_buf[0x18]<<16) + u16_buf[0x17];
	if(app_size == 0)
		return 0;
	/* ----- Get file system area ----- */
	data_start_addr = boot_size + app_size;
	if( data_start_addr & 0xfff)
		data_start_addr += 0x1000;

	return data_start_addr & ~0xfff;
}

static INT32S SDCARD_Initial_Storage(void* priv)
{
	INT32U device_id = (INT32U)priv;
	INT32S ret = 0;

	SD_MSDC_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);
	SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
	sd_msdc_offset[device_id] = 0;

	ret = drvl2_sd_init(device_id);
	if (ret != 0)
	{
        SD_MSDC_ERR_PRINT("[%s] drvl2_sd_init fail. id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);
	}
	else
	{
		sd_msdc_offset[device_id] = sdcard_boot_parser_header(device_id);
	}

    if (ret != 0)
    {
        if (drvl1_sdc_check_card_present(device_id) == 0)
            SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        else
            SDCARD_Set_Error(device_id, C_SD_MSDC_GENERAL_ERR);
    }

	SD_MSDC_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);

	return ret;
}

static INT32S SDCARD_Uninitial_Storage(void* priv)
{
	INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

	SD_MSDC_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);
    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);

	drvl2_sd_card_remove(device_id);

    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = -1;
    }

	SD_MSDC_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);

	return ret;
}

static void SDCARD_GetDrvInfo_Storage(void* priv, STORAGE_DRV_INFO *info)
{
	INT32U device_id = (INT32U)priv;

	SD_MSDC_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);
    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);

	info->nSectors = drvl2_sd_sector_number_get(device_id);
	info->nBytesPerSector = 512;

	SD_MSDC_PRINT("[%s] id=%d, nSectors=%u\r\n", __FUNCTION__, device_id, info->nSectors);
}

static INT32S SDCARD_ReadCmdPhase(void* priv, INT32U lba,INT32U seccount)
{
	INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

    SD_MSDC_PRINT("[%s] id=%u lba=%u seccount=%u\r\n", __FUNCTION__, device_id, lba, seccount);
    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
    msdc_sd_lba[device_id] = lba;
	msdc_sd_seccount[device_id] = seccount;

    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = -1;
    }

	SD_MSDC_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);

	return ret;
}

static INT32S SDCARD_ReadSectorDMA(void* priv, INT32U *buf, INT8U ifwait, INT32U seccount)
{
	INT32U device_id = (INT32U)priv;
	INT32S ret = 0;

    ifwait = 0;

	SD_MSDC_PRINT("[%s] id=%d ifwait=%u seccount=%u\r\n", __FUNCTION__, device_id, ifwait, seccount);
	SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);

    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        return -1;
    }

    if ((seccount!= 0) && (msdc_sd_seccount[device_id]>=seccount))
	{
		#if 0
		// slower
		ret = drvl2_sd_read(device_id, msdc_sd_lba[device_id] + sd_msdc_offset[device_id], buf, seccount);
		#else
		// faster
        ret = drvl2_sd_read_start(device_id, msdc_sd_lba[device_id] + sd_msdc_offset[device_id], seccount);
		if (ret == 0)
		{
            ret = drvl2_sd_read_sector(device_id,  buf, seccount, ifwait);
            if (ret == 0)
            {
                ret = drvl2_sd_read_stop(device_id);
                if (ret != 0)
                    DBG_PRINT("drvl2_sd_read_stop err=%d\r\n", ret);
            }
            else
                DBG_PRINT("drvl2_sd_read_sector err=%d\r\n", ret);
		}
		else
            DBG_PRINT("drvl2_sd_read_start err=%d\r\n", ret);
		#endif

		#if K_SD_MSDC_READ_FLUSH
		if (ret == 0)
			ret = drvl2_sd_flush(device_id);
        #endif

		if (ret != 0)
        {
            if (drvl1_sdc_check_card_present(device_id) == 0)
                SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
            else
                SDCARD_Set_Error(device_id, C_SD_MSDC_GENERAL_ERR);
            SD_MSDC_ERR_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);
        }

        //DBG_PRINT("sd msdc read lba=%u, seccount=%u\r\n", msdc_sd_lba[device_id], seccount, msdc_sd_seccount[device_id]);
	    msdc_sd_lba[device_id] += seccount;
	    if (msdc_sd_seccount[device_id] >= seccount)
            msdc_sd_seccount[device_id] -= seccount;
        else
            msdc_sd_seccount[device_id] = 0;
	}

	SD_MSDC_PRINT("[%s] id=%d ret=%s\r\n", __FUNCTION__, device_id, ret);

	return ret;
}

static INT32S SDCARD_ReadEndCmdPhase(void* priv)
{
	INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

    SD_MSDC_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);
    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = -1;
    }

	return ret;
}

static INT32S SDCARD_WriteSectorDMA(void* priv, INT32U *buf, INT8U ifwait, INT32U seccount)
{
	INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

    ifwait = 0;

    SD_MSDC_PRINT("[%s] id=%d ifwait=%u seccount=%u\r\n", __FUNCTION__, device_id, ifwait, seccount);
    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        return -1;
    }

    if ((seccount!= 0) && (msdc_sd_seccount[device_id]>=seccount))
	{
		#if 0
		// slower
		ret = drvl2_sd_write(device_id, msdc_sd_lba[device_id] + sd_msdc_offset[device_id], buf, seccount);
		#else
		// faster
        ret = drvl2_sd_write_start(device_id, msdc_sd_lba[device_id] + sd_msdc_offset[device_id], seccount);
		if (ret == 0)
		{
            ret = drvl2_sd_write_sector(device_id,  buf,  seccount, ifwait);
            if (ret == 0)
                ret = drvl2_sd_write_stop(device_id);
		}
		#endif

		#if K_SD_MSDC_WRITE_FLUSH
		if (ret == 0)
			ret = drvl2_sd_flush(device_id);
        #endif

		if (ret != 0)
        {
            if (drvl1_sdc_check_card_present(device_id) == 0)
                SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
            else
                SDCARD_Set_Error(device_id, C_SD_MSDC_GENERAL_ERR);
            SD_MSDC_ERR_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);
        }

		//DBG_PRINT("sd msdc write lba=%u, seccount=%u\r\n", msdc_sd_lba[device_id], seccount, msdc_sd_seccount[device_id]);
	    msdc_sd_lba[device_id] += seccount;
	    if (msdc_sd_seccount[device_id] >= seccount)
            msdc_sd_seccount[device_id] -= seccount;
        else
            msdc_sd_seccount[device_id] = 0;
	}

	SD_MSDC_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);

    return ret;
}

static INT32S SDCARD_WriteCmdPhase(void* priv, INT32U lba, INT32U seccount)
{
	INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

    SD_MSDC_PRINT("[%s] id=%d lba=%u seccount=%u\r\n", __FUNCTION__, device_id, lba, seccount);
    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
    msdc_sd_lba[device_id] = lba;
	msdc_sd_seccount[device_id] = seccount;

    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = -1;
    }

	SD_MSDC_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);

	return ret;
}

static INT32S SDCARD_WriteEndCmdPhase(void* priv)
{
	INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
    SD_MSDC_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);
    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = -1;
    }
	return ret;
}

static INT32S  SDCARD_CheckDmaCheckState(void* priv)
{
	INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
    SD_MSDC_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);
    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = -1;
        ret = STORAGE_CHECK_NOT_READY;
    }
    else
        ret = STORAGE_CHECK_READY;

    return ret;
}

static INT32S SDCARD_Insertion(void* priv)
{
	INT32S ret = 0;
	INT32U device_id = (INT32U)priv;

    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);

    /* 0: inserted, 1: removed */
    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = 1;
    }
    SD_MSDC_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);

	return ret;
}

static INT32S SDCARD_StopUnit_UserFunc(void* priv)
{
    INT32U device_id = (INT32U)priv;
    INT32S ret = 0;

    SD_MSDC_PRINT("[%s]\r\n", __FUNCTION__);
    SDCARD_Set_Error(device_id, C_SD_MSDC_NO_ERR);
    drvl2_sd_card_remove(device_id);
    if (drvl1_sdc_check_card_present(device_id) == 0)
    {
        SDCARD_Set_Error(device_id, C_SD_MSDC_CARD_REMOVED_ERR);
        ret = -1;
    }

    return ret;
}

/* SDC0 disk access functions table */
MDSC_LUN_STORAGE_DRV const gp_msdc_sd0 =
{
    0,                         //private data
    SDCARD_Initial_Storage,    //init
    SDCARD_Uninitial_Storage,  //uninit
    SDCARD_Insertion,	       //insert event
    SDCARD_GetDrvInfo_Storage, //get driver info
    SDCARD_ReadCmdPhase,       //read command phase
    SDCARD_ReadSectorDMA,      //read DMA phase
    SDCARD_ReadEndCmdPhase,    //read command end phase
    SDCARD_WriteCmdPhase,      //write command phase
    SDCARD_WriteSectorDMA,     //write DMA phase
    SDCARD_WriteEndCmdPhase,   //write command end phase
    SDCARD_CheckDmaCheckState, //check DMA buffer state
    SDCARD_StopUnit_UserFunc   //stop lun unit callback
};

/* SDC1 disk access functions table */
MDSC_LUN_STORAGE_DRV const gp_msdc_sd1 =
{
    ((void*)1),                //private data
    SDCARD_Initial_Storage,    //init
    SDCARD_Uninitial_Storage,  //uninit
    SDCARD_Insertion,          //insert event
    SDCARD_GetDrvInfo_Storage, //get driver info
    SDCARD_ReadCmdPhase,       //read command phase
    SDCARD_ReadSectorDMA,      //read DMA phase
    SDCARD_ReadEndCmdPhase,    //read command end phase
    SDCARD_WriteCmdPhase,      //write command phase
    SDCARD_WriteSectorDMA,     //write DMA phase
    SDCARD_WriteEndCmdPhase,   //write command end phase
    SDCARD_CheckDmaCheckState, //check DMA buffer state
    SDCARD_StopUnit_UserFunc   //stop lun unit callback
};
