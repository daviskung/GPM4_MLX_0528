#define CREAT_DRIVERLAYER_STRUCT

#include "drv_l2_cfg.h"
#include "fsystem.h"
#include "drv_l2_sd.h"

#define SD_FS_ERR_PRINT  DBG_PRINT

#define SD_FS_PRINT(...)
//#define SD_FS_PRINT         DBG_PRINT

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined SD_EN) && (SD_EN == 1)                               //

//================================================================//
#define SD_TOTAL_DEVICE				2
#define SD_RW_RETRY_COUNT		    3
#define SDC_TAG0		            0x73644750		// string: PGds
#define SDC_TAG1		            0x64636373		// string: sccd
#define SDC_TAG2		            0x7364			// string: ds
#define APP_TAG			            0x5047			// string: GP
#define FS_CACHE_DBG_EN             0
#if MCU_VERSION == GPM41XXA
#define FS_FLUSH_CAHCE_EN           0
#else
#define FS_FLUSH_CAHCE_EN           1		// control whether want to caching the FAT1 area
											// note that the FAT1 caching just working with GP fs code that
											// write FAT1 area per 1 sector
#endif
#define FAT_MAIN_TBL_ID             0
#define FAT_SECOND_TBL_ID           1
#define CACHE_FAT_TABLE_SYNC		0x80
#define CACHE_FAT_TABLE_HIT			0x40
#define CACHE_FAT_TABLE_INIT		0x20
#define CACHE_FAT_TABLE_MISS		0x10
#define CACHE_FAT_TABLE_CNT	        4		// maximal value is 15, because just using 4 bit
#define SDC_CACHE_SIZE              8192
#define SDC_CACHE_SECTOR_NUMS       (SDC_CACHE_SIZE/512)
#define SD_RW_RETRY_COUNT           3

INT32U sd_fs_offset[SD_TOTAL_DEVICE] = {0, 0};

#if (FS_CACHE_FAT_EN == 1)

typedef struct {
	INT32U  cache_sector_idx;
	INT32U  cache_nr_sector;
	INT32U  cache_addrs;
	INT32U	cache_time;
	INT8U  	dirty_flag;
	INT8U  	is_used_flag;
}cache_info;

static cache_info cache_fat_table_list[SD_TOTAL_DEVICE][CACHE_FAT_TABLE_CNT]={0};

static ALIGN32 INT8U __attribute__((section(".uninit"))) sd_cache_ram_list[SD_TOTAL_DEVICE][SDC_CACHE_SIZE*CACHE_FAT_TABLE_CNT];
static INT8U   cache_init_flag[SD_TOTAL_DEVICE] = {0, 0};// cache has initialized, its value is set to 0xA8
static INT8U   fat_cache_L1_en[SD_TOTAL_DEVICE] = {0, 0};	// if cache is intialized sucessfully, it will be set to 1
static INT32U  fat_cluster_size[SD_TOTAL_DEVICE];	// unit: sectors
static INT32U  fat_tbl_size[SD_TOTAL_DEVICE];		// unit: sectors
static INT32U  fat_start_id_list[SD_TOTAL_DEVICE][2];
static INT32U  cache_range_start[SD_TOTAL_DEVICE];
static INT32U  cache_range_end[SD_TOTAL_DEVICE];
static INT8U   fat_type[SD_TOTAL_DEVICE];		//FAT16 if its value is 16, or FAT32 if its value is 32
static INT32U  fs_info_start[SD_TOTAL_DEVICE];//sector no of FSINFO
static INT32U  BPB_Start_id[SD_TOTAL_DEVICE];		//the first sector no of DOS partition

static INT32S fat_l1_cache_init(INT32U device_id);
static INT32S cache_sync(INT32U device_id);
static void FsCpy4(void *_dst, const void *_src, int len);

void fat_l1_cache_reinit(INT32U device_id);
void fat_l1_cahhe_uninit(INT32U device_id);

/****************************************************************************************************************************/
#if FS_FLUSH_CAHCE_EN
static INT8U find_cache_fat_table_for_read(INT32U device_id, INT32U sectorIdx)
{
	INT8U i;
#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
  	INT32U max_time_range = OSTimeGet();		// 10ms per tick
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
    INT32U max_time_range = xTaskGetTickCount();
    #endif
#endif
	INT8U select_idx = 0;
	cache_info *cache_fat_table = &cache_fat_table_list[device_id][0];

	for(i=0; i<CACHE_FAT_TABLE_CNT; i++)
	{
		// Return unused cache fat table
		if(cache_fat_table[i].is_used_flag == 0)
		{
			return (i|CACHE_FAT_TABLE_INIT);
		}

		// When all table is full, find the oldest cache time of cache fat table to will replace it later
		if(max_time_range >= cache_fat_table[i].cache_time)
		{
			max_time_range = cache_fat_table[i].cache_time;
			select_idx = i;
		}

		// BIGO! Return cache fat table	index
		if((sectorIdx >=cache_fat_table[i].cache_sector_idx) &&
		   (sectorIdx < (cache_fat_table[i].cache_sector_idx+cache_fat_table[i].cache_nr_sector))
		)
		{
			return (i|CACHE_FAT_TABLE_HIT);
		}
	}

	return (select_idx|CACHE_FAT_TABLE_MISS);
}

static INT8U find_cache_fat_table_for_write(INT32U device_id, INT32U sectorIdx)
{
	INT8U i;
#if _OPERATING_SYSTEM != _OS_NONE
    #if _OPERATING_SYSTEM == _OS_UCOS2
  	INT32U max_time_range = OSTimeGet();		// 10ms per tick
    #elif _OPERATING_SYSTEM == _OS_FREERTOS
    INT32U max_time_range = xTaskGetTickCount();
    #endif
#endif
	INT8U select_idx = 0;
	cache_info *cache_fat_table = &cache_fat_table_list[device_id][0];

	for(i=0; i<CACHE_FAT_TABLE_CNT; i++)
	{
		if(cache_fat_table[i].is_used_flag == 0)
		{
			return (i|CACHE_FAT_TABLE_INIT);
		}

		if(max_time_range >= cache_fat_table[i].cache_time)
		{
			max_time_range = cache_fat_table[i].cache_time;
			select_idx = i;
		}

		if((sectorIdx >=cache_fat_table[i].cache_sector_idx) &&
		   (sectorIdx < (cache_fat_table[i].cache_sector_idx+cache_fat_table[i].cache_nr_sector))
		)
		{
			return (i|CACHE_FAT_TABLE_HIT);
		}
	}

	return (select_idx|CACHE_FAT_TABLE_SYNC);
}
#endif

void fat_l1_cahhe_uninit(INT32U device_id)
{
	fat_cache_L1_en[device_id] = 0;
	cache_init_flag[device_id] = 0;
}

void fat_l1_cache_reinit(INT32U device_id)
{
	cache_sync(device_id);
    cache_init_flag[device_id] = 0;
    fat_l1_cache_init(device_id);
}

INT32S fat_l1_cache_init(INT32U device_id)
{
    INT32S ret;
    INT8U *byte_buf;
    INT16U *short_buf;
    INT32U *word_buf;
    INT32U Next_Free = 0xFFFFFFFF;
    INT32U FS_info_tag = 0x00000000;
    INT32U Free_Count = 0xFFFFFFFF;
	cache_info *cache_fat_table = &cache_fat_table_list[device_id][0];
	INT32U  *fat_start_id = &fat_start_id_list[device_id][0];
    INT8U *sd_cache_ram = &sd_cache_ram_list[device_id][0];

    if (cache_init_flag[device_id] != 0xA8)
    {
      #if FS_CACHE_DBG_EN == 1
        DBG_PRINT ("SD FatL1 cache Init...");
      #endif

        fs_info_start[device_id] = 0xFFFFFFFF;

        BPB_Start_id[device_id] = 0;  // NO MBR, BPB always is 0

        short_buf = (INT16U *) &sd_cache_ram[0];
        word_buf = (INT32U *) &sd_cache_ram[0];
        byte_buf = (INT8U *) &sd_cache_ram[0];


		#if FS_FLUSH_CAHCE_EN
		for(ret=0; ret<CACHE_FAT_TABLE_CNT; ret++)
		{
			cache_fat_table[ret].cache_sector_idx = 0;
			cache_fat_table[ret].cache_nr_sector = 0;
			cache_fat_table[ret].dirty_flag = 0;
			cache_fat_table[ret].cache_time = 0;
			cache_fat_table[ret].is_used_flag = 0;
			cache_fat_table[ret].cache_addrs = (INT32U)(sd_cache_ram+(ret*SDC_CACHE_SIZE));
		}
		#endif


        {
            INT32U k;
            for(k = 0; k < SD_RW_RETRY_COUNT; k++)
            {
                ret = drvl2_sd_read(device_id, BPB_Start_id[device_id], (INT32U *) &sd_cache_ram[0], 1);
                if(ret == 0)
                    break;
            }
        }
        if(ret==0 && short_buf[0x1fE/2] == 0xAA55 && byte_buf[0] != 0xEB) {
            BPB_Start_id[device_id] = ((short_buf[0x1C8/2]<<16) | short_buf[0x1C6/2]);
            DBG_PRINT ("MBR Find, first part offset :0x%x\r\n",BPB_Start_id[device_id]);
            {
                INT32U k;

                for(k = 0; k < SD_RW_RETRY_COUNT; k++)
                {
                    ret = drvl2_sd_read(device_id, BPB_Start_id[device_id], (INT32U *) &sd_cache_ram[0], 1);
                    if(ret == 0)
                        break;
                }
            }
        }

        if(ret != 0) {
            fat_cache_L1_en[device_id] = 0;

			DBG_PRINT("SD read fail 1\r\n");
            return -1;

		} else if(ret == 0) {
            fat_cache_L1_en[device_id] = 1;

            fat_cluster_size[device_id] = byte_buf[13];  // 8 sectors
            fat_start_id[0] = short_buf[7] + BPB_Start_id[device_id];   //0x24*0x200=0x4800

            fat_tbl_size[device_id] = short_buf[22/2];	//this value must be set to 0 in FAT32(sector per fat)
			cache_range_start[device_id] = fat_start_id[0];
			cache_range_end[device_id] = fat_start_id[0] + fat_tbl_size[device_id];

            if(fat_tbl_size[device_id] == 0) {
                //find FAT32
                fat_type[device_id] = 32;
                DBG_PRINT ("FAT32\r\n");

                fat_tbl_size[device_id] = word_buf[9];  //0x1d72*0x200=0x3AE400
                fs_info_start[device_id] = short_buf[48/2] + BPB_Start_id[device_id];
				cache_range_end[device_id] = fat_start_id[0] + fat_tbl_size[device_id];

                {
                    INT32U k;

                    for(k = 0; k < SD_RW_RETRY_COUNT; k++)
                    {
                        if (drvl2_sd_read(device_id, fs_info_start[device_id], (INT32U *) &sd_cache_ram[0], 1) != 0) {
                            DBG_PRINT("SD read fail 2, try %u\r\n", k);
                        }
                        else
                            break;
                    }
                }
                FS_info_tag = word_buf[0];
                Free_Count = word_buf[488/4] + BPB_Start_id[device_id];  //should not add BPB_Start_id, wwj comment@20140929
                Next_Free = word_buf[492/4] + BPB_Start_id[device_id];
              //#endif

                DBG_PRINT ("Fs info Tag: 0x%x (SectorId:%d)\r\n", FS_info_tag, fs_info_start[device_id]);

            } else {
                fat_type[device_id] = 16;
                DBG_PRINT ("FAT16\r\n");
            }
            fat_start_id[1] = fat_start_id[0] + fat_tbl_size[device_id]; // 0x3AE400+0x4800=0x3B2C00
        }

      #if 1 || FS_CACHE_DBG_EN == 1
        DBG_PRINT ("\r\nfat_cluster_size:%d\r\n",fat_cluster_size[device_id]);
        DBG_PRINT ("fat_start_id[0]:%d\r\n",fat_start_id[0]);
        DBG_PRINT ("fat_start_id[1]:%d\r\n",fat_start_id[1]);
        DBG_PRINT ("fat_tbl_size:%d\r\n",fat_tbl_size[device_id]);
      #endif
        cache_init_flag[device_id] = 0xA8;
    }
    return 0;
}

INT32S cache_sync(INT32U device_id)
{
	INT8U i, k;
    INT32S ret = 0;
	cache_info *cache_fat_table = &cache_fat_table_list[device_id][0];

#if FS_FLUSH_CAHCE_EN==1
    if (fat_cache_L1_en[device_id]) {
		for(ret = 0, i=0; i<CACHE_FAT_TABLE_CNT; i++)
		{
			if(cache_fat_table[i].dirty_flag)
			{
				cache_fat_table[i].dirty_flag = 0;

				for(k = 0; k < SD_RW_RETRY_COUNT; k++)
				{
					ret = drvl2_sd_write(device_id, cache_fat_table[i].cache_sector_idx, (INT32U *)cache_fat_table[i].cache_addrs, cache_fat_table[i].cache_nr_sector);
					if (ret == 0)
						break;
				}

			    if (ret != 0) {
					DBG_PRINT("SD write fail 6. i=%u\r\n", i);
					break;
				}
			}
		}
		return ret;
    }
    return ret;
#endif
}
#endif // FS_CACHE_FAT_EN == 1

/****************************************************************************************************************************/
static INT32U sdx_boot_parser_header(INT32U device_id)
{
	INT32U buf[128];
	INT16U *u16_buf = (INT16U *)buf;
	INT16U boot_size;
	INT32U app_size = 0, data_start_addr = 0;
	INT32S ret = -1;
    INT32U k;

	/* ----- Read boot header ----- */
	for(k = 0; k < SD_RW_RETRY_COUNT; k++)
	{
        if( (ret = drvl2_sd_read(device_id, 0x00, buf, 1)) ==0 )
            break;
    }
    if (ret != 0)
        return 0;

	/* ----- check boot header tag ----- */
	if( (buf[0] != SDC_TAG0) || (buf[1] != SDC_TAG1) || ( u16_buf[4] != SDC_TAG2))
		return 0;
	/* ----- Get boot area size ----- */
	boot_size = u16_buf[8];
	if(boot_size == 0)
		return 0;
	/* ----- Read APP header ----- */
	for(k = 0; k < SD_RW_RETRY_COUNT; k++)
	{
        if( (ret = drvl2_sd_read(device_id, boot_size, buf, 1 )) ==0 )
            break;
    }
    if (ret != 0)
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

INT32S SDX_Initial(INT32U device_id)
{
	INT32S ret;
    INT32U k;

    SD_FS_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);

	#if (FS_CACHE_FAT_EN == 1)
    cache_sync(device_id);
	#endif

    for(k = 0; k < SD_RW_RETRY_COUNT; k++)
    {
        ret = drvl2_sd_init(device_id);
        if (ret == 0)
            break;
    }

	#if (FS_CACHE_FAT_EN == 1)
    fat_l1_cahhe_uninit(device_id);
	#endif

	if (ret == 0) {
		#if (FS_CACHE_FAT_EN == 1)
		fat_l1_cache_init(device_id);
		#endif
	} else {
        SD_FS_ERR_PRINT("[%s] drvl2_sd_init fail. id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);
	}

	sd_fs_offset[device_id] = 0;
	if(ret == 0 )
		sd_fs_offset[device_id] = sdx_boot_parser_header(device_id);
	return ret;
}

INT32S SDX_Uninitial(INT32U device_id)
{
    INT32S ret;

	#if (FS_CACHE_FAT_EN == 1)
	cache_sync(device_id);
	#endif

    ret = drvl2_sd_card_remove(device_id);

	#if (FS_CACHE_FAT_EN == 1)
	fat_l1_cahhe_uninit(device_id);
	#endif

    SD_FS_PRINT("[%s] id=%d ret=%d\r\n", __FUNCTION__, device_id, ret);

    return 0;
}

void SDX_GetDrvInfo(INT32U device_id, struct DrvInfo* info)
{
    SD_FS_PRINT("[%s] id=%d\r\n", __FUNCTION__, device_id);

	#if (FS_CACHE_FAT_EN == 1)
    fat_l1_cache_init(device_id);
	#endif

	info->nSectors = drvl2_sd_sector_number_get(device_id) - sd_fs_offset[device_id];
	info->nBytesPerSector = 512;
}

//read/write speed test function
INT32S SDX_ReadSector(INT32U device_id, INT32U blkno, INT32U blkcnt, INT32U buf)
{
	INT32S	ret =0;
	INT32S	i;
#if (FS_CACHE_FAT_EN == 1)
#if FS_FLUSH_CAHCE_EN==1
	INT8U retIdx;
	INT32U cache_sector_idx;
	INT32U offset;
	INT32U cache_blkcnt = 0;
	cache_info *cache_fat_table = &cache_fat_table_list[device_id][0];
#endif
#endif

    SD_FS_PRINT("[%s] id=%d blkno=%u blkcnt=%u buf=%u\r\n", __FUNCTION__, device_id, blkno, blkcnt, buf);

	#if (FS_CACHE_FAT_EN == 1)
    fat_l1_cache_init(device_id);
	#endif

    if (fs_sd_ms_plug_out_flag_get()==1) {return 0xFFFFFFFF;}

#if (FS_CACHE_FAT_EN == 1)
#if FS_FLUSH_CAHCE_EN==1
    if (fat_cache_L1_en[device_id] && (blkcnt == 1))
	{
		if ((blkno>=cache_range_start[device_id]) && (blkno<cache_range_end[device_id]))
		{
			if ((blkno + blkcnt) > cache_range_end[device_id])
			{
				// partially go to fat cache, calculate left blkcnt to be handle
				cache_blkcnt = cache_range_end[device_id] - blkno;
				blkcnt = blkcnt - cache_blkcnt;
			}
			else
			{
				// completely go to fat cache
				cache_blkcnt = blkcnt;
				blkcnt = 0;
			}
			retIdx = find_cache_fat_table_for_read(device_id, blkno);
			if (retIdx & CACHE_FAT_TABLE_HIT)
			{
				retIdx &= ~(CACHE_FAT_TABLE_HIT);
				offset = ((blkno-cache_fat_table[retIdx].cache_sector_idx)<<9);
				FsCpy4((void *)buf, (void *)(cache_fat_table[retIdx].cache_addrs+offset), cache_blkcnt<<9);
				buf = (INT32U)((void *)buf + (cache_blkcnt<<9));
			}
			else if (retIdx & CACHE_FAT_TABLE_INIT)
			{
				retIdx &= ~(CACHE_FAT_TABLE_INIT);

				// Read data from SDC and store it into cache fat table
				cache_fat_table[retIdx].cache_sector_idx = (((blkno - cache_range_start[device_id])/SDC_CACHE_SECTOR_NUMS)*SDC_CACHE_SECTOR_NUMS) + cache_range_start[device_id];
				cache_fat_table[retIdx].cache_nr_sector = SDC_CACHE_SECTOR_NUMS;
				if ((cache_fat_table[retIdx].cache_sector_idx + cache_fat_table[retIdx].cache_nr_sector) > cache_range_end[device_id])
					cache_fat_table[retIdx].cache_nr_sector = cache_range_end[device_id] - cache_fat_table[retIdx].cache_sector_idx;

				for(i=0; i<SD_RW_RETRY_COUNT; i++)
				{
					ret = drvl2_sd_read(device_id, cache_fat_table[retIdx].cache_sector_idx, (INT32U *) cache_fat_table[retIdx].cache_addrs, cache_fat_table[retIdx].cache_nr_sector);
					if(ret == 0)
					{
						break;
					}
				}

				if (ret == 0)
				{
					// Return data
					offset = ((blkno-cache_fat_table[retIdx].cache_sector_idx)<<9);
					FsCpy4((void *)buf, (void *)(cache_fat_table[retIdx].cache_addrs+offset), cache_blkcnt<<9);
					cache_fat_table[retIdx].is_used_flag = 1;
					buf = (INT32U)((void *)buf + (cache_blkcnt<<9));
				#if _OPERATING_SYSTEM != _OS_NONE
					#if _OPERATING_SYSTEM == _OS_UCOS2
					cache_fat_table[retIdx].cache_time = OSTimeGet();		// 10ms per tick
					#elif _OPERATING_SYSTEM == _OS_FREERTOS
					cache_fat_table[retIdx].cache_time = xTaskGetTickCount();
					#endif
				#endif
				}
			}
			else
			{
				retIdx &= ~(CACHE_FAT_TABLE_MISS);

				// Write the replaced data from the cache fat table to the SDC
				if(cache_fat_table[retIdx].dirty_flag)
				{
					for(i=0; i<SD_RW_RETRY_COUNT; i++)
					{
						ret = drvl2_sd_write(device_id, cache_fat_table[retIdx].cache_sector_idx, (INT32U *)cache_fat_table[retIdx].cache_addrs, cache_fat_table[retIdx].cache_nr_sector);
						if(ret == 0)
						{
							break;
						}
					}
				}

				if (ret == 0)
				{
					// Read data from SDC and store it into cache fat table
					cache_fat_table[retIdx].cache_sector_idx = (((blkno - cache_range_start[device_id])/SDC_CACHE_SECTOR_NUMS)*SDC_CACHE_SECTOR_NUMS) + cache_range_start[device_id];
					cache_fat_table[retIdx].cache_nr_sector = SDC_CACHE_SECTOR_NUMS;
					if ((cache_fat_table[retIdx].cache_sector_idx + cache_fat_table[retIdx].cache_nr_sector) > cache_range_end[device_id])
						cache_fat_table[retIdx].cache_nr_sector = cache_range_end[device_id] - cache_fat_table[retIdx].cache_sector_idx;

					for(i=0; i<SD_RW_RETRY_COUNT; i++)
					{
						ret = drvl2_sd_read(device_id, cache_fat_table[retIdx].cache_sector_idx, (INT32U *) cache_fat_table[retIdx].cache_addrs, cache_fat_table[retIdx].cache_nr_sector);
						if(ret == 0)
						{
							break;
						}
					}

					if (ret == 0)
					{
						offset = ((blkno-cache_fat_table[retIdx].cache_sector_idx)<<9);
						FsCpy4((void *)buf,(void *)(cache_fat_table[retIdx].cache_addrs+offset),cache_blkcnt<<9);
					#if _OPERATING_SYSTEM != _OS_NONE
						#if _OPERATING_SYSTEM == _OS_UCOS2
						cache_fat_table[retIdx].cache_time = OSTimeGet();		// 10ms per tick
						#elif _OPERATING_SYSTEM == _OS_FREERTOS
						cache_fat_table[retIdx].cache_time = xTaskGetTickCount();
						#endif
					#endif
						cache_fat_table[retIdx].is_used_flag = 1;
						cache_fat_table[retIdx].dirty_flag = 0;
						buf = (INT32U)((void *)buf + (cache_blkcnt<<9));
					}
					else
					{
						cache_fat_table[retIdx].is_used_flag = 0;
						cache_fat_table[retIdx].dirty_flag = 0;
					}
				}
			}
			blkno += cache_blkcnt;
		}
    }
#endif
#endif

	if (blkcnt && (ret == 0))
	{
		for(i = 0; i < SD_RW_RETRY_COUNT; i++)
		{
			ret = drvl2_sd_read(device_id, blkno + sd_fs_offset[device_id], (INT32U *) buf, blkcnt);
			if(ret == 0)
			{
				break;
			}
			else
				SD_FS_ERR_PRINT("[%s] drvl2_sd_read fail. id=%d ret=%d try=%d\r\n", __FUNCTION__, device_id, ret, i);
		}
	}

    #if SUPPORT_STG_SUPER_PLUGOUT == 1
    if (ret!=0)
    {
        //if (drvl2_sd_read(device_id, 0, (INT32U *) buf, 1)!=0)
        {
            fs_sd_ms_plug_out_flag_en();
            DBG_PRINT ("============>SUPER PLUG OUT DETECTED<===========\r\n");
        }
    }
    #endif

	return ret;
}

INT32S SDX_WriteSector(INT32U device_id, INT32U blkno, INT32U blkcnt, INT32U buf)
{
	INT32S	ret = 0;
	INT32S	i;
#if (FS_CACHE_FAT_EN == 1)
#if FS_FLUSH_CAHCE_EN==1
	INT8U retIdx;
	INT32U cache_sector_idx;
	INT32U offset;
	INT32U cache_blkcnt = 0;
	cache_info *cache_fat_table = &cache_fat_table_list[device_id][0];
#endif
#endif

    SD_FS_PRINT("[%s] id=%d blkno=%u blkcnt=%u buf=%u\r\n", __FUNCTION__, device_id, blkno, blkcnt, buf);

    #if (FS_CACHE_FAT_EN == 1)
    fat_l1_cache_init(device_id);
    #endif

    if (fs_sd_ms_plug_out_flag_get()==1) {return 0xFFFFFFFF;}

    //if ((blkno >= fat_start_id_list[device_id][1]) && (blkno <= (fat_start_id_list[device_id][1] + fat_tbl_size[device_id])))
    //    DBG_PRINT("FAT2 was written:blkno=%u blkcnt%u\r\n", blkno, blkcnt);

	#if (FS_CACHE_FAT_EN == 1)
	#if FS_FLUSH_CAHCE_EN==1

    if (fat_cache_L1_en[device_id] && (blkcnt == 1))
	{
        if ((blkno >= cache_range_start[device_id]) && (blkno < cache_range_end[device_id]))
		{
            //DBG_PRINT("FAT1 was written:blkno=%u blkcnt%u\r\n", blkno, blkcnt);

			if ((blkno + blkcnt) > cache_range_end[device_id])
			{
				// partially go to fat cache, calculate left blkcnt to be handle
				cache_blkcnt = cache_range_end[device_id] - blkno;
				blkcnt = blkcnt - cache_blkcnt;
			}
			else
			{
				// completely go to fat cache
				cache_blkcnt = blkcnt;
				blkcnt = 0;
			}

			retIdx = find_cache_fat_table_for_write(device_id, blkno);
			if ((retIdx & CACHE_FAT_TABLE_SYNC) || (retIdx & CACHE_FAT_TABLE_INIT))
			{
				// Write the replaced data from the cache fat table to the SDC
				if(retIdx & CACHE_FAT_TABLE_SYNC)
				{
					retIdx &= ~(CACHE_FAT_TABLE_SYNC);

					if(cache_fat_table[retIdx].dirty_flag)
					{
						for(i=0; i<SD_RW_RETRY_COUNT; i++)
						{
							ret = drvl2_sd_write(device_id, cache_fat_table[retIdx].cache_sector_idx, (INT32U *)cache_fat_table[retIdx].cache_addrs, cache_fat_table[retIdx].cache_nr_sector);
							if(ret == 0)
							{
								cache_fat_table[retIdx].dirty_flag = 0;
								break;
							}
						}
					}
				}
				else
				{
					retIdx &= ~(CACHE_FAT_TABLE_INIT);
				}

				// Read data from SDC and store it into cache fat table
				if (ret == 0)
				{
					cache_fat_table[retIdx].cache_sector_idx = (((blkno - cache_range_start[device_id])/SDC_CACHE_SECTOR_NUMS)*SDC_CACHE_SECTOR_NUMS) + cache_range_start[device_id];
					cache_fat_table[retIdx].cache_nr_sector = SDC_CACHE_SECTOR_NUMS;
					if ((cache_fat_table[retIdx].cache_sector_idx + cache_fat_table[retIdx].cache_nr_sector) > cache_range_end[device_id])
						cache_fat_table[retIdx].cache_nr_sector = cache_range_end[device_id] - cache_fat_table[retIdx].cache_sector_idx;

					for(i=0; i<SD_RW_RETRY_COUNT; i++)
					{
						ret = drvl2_sd_read(device_id, cache_fat_table[retIdx].cache_sector_idx, (INT32U *) cache_fat_table[retIdx].cache_addrs, cache_fat_table[retIdx].cache_nr_sector);
						if(ret == 0)
						{
							break;
						}
					}

					if (ret == 0)
					{
						offset = ((blkno-cache_fat_table[retIdx].cache_sector_idx)<<9);
						FsCpy4((void *)(cache_fat_table[retIdx].cache_addrs+offset),(void *)buf, cache_blkcnt<<9);
						//cache_fat_table[retIdx].cache_time = OSTimeGet();
					#if _OPERATING_SYSTEM != _OS_NONE
						#if _OPERATING_SYSTEM == _OS_UCOS2
						cache_fat_table[retIdx].cache_time = OSTimeGet();
						#elif _OPERATING_SYSTEM == _OS_FREERTOS
						cache_fat_table[retIdx].cache_time = xTaskGetTickCount();
						#endif
					#endif
						cache_fat_table[retIdx].is_used_flag = 1;
						cache_fat_table[retIdx].dirty_flag = 1;
						buf = (INT32U)((void *)buf + (cache_blkcnt<<9));
					}
					else
					{
						cache_fat_table[retIdx].is_used_flag = 0;
						cache_fat_table[retIdx].dirty_flag = 0;
					}
				}
			}
			else if(retIdx & CACHE_FAT_TABLE_HIT)
			{
				retIdx &= ~(CACHE_FAT_TABLE_HIT);

				offset = ((blkno-cache_fat_table[retIdx].cache_sector_idx)<<9);
				FsCpy4((void *)(cache_fat_table[retIdx].cache_addrs+offset),(void *)buf, cache_blkcnt<<9);
				cache_fat_table[retIdx].dirty_flag = 1;
				buf = (INT32U)((void *)buf + (cache_blkcnt<<9));
			}
			blkno += cache_blkcnt;
        }
    }
	#endif
	#endif

	if (blkcnt && (ret == 0))
	{
		for(i = 0; i < SD_RW_RETRY_COUNT; i++)
		{
			ret = drvl2_sd_write(device_id, blkno + sd_fs_offset[device_id], (INT32U *) buf, blkcnt);
			if(ret == 0)
			{
				break;
			}
			else
				SD_FS_ERR_PRINT("[%s] drvl2_sd_write fail. id=%d ret=%d try=%d\r\n", __FUNCTION__, device_id, ret, i);
		}
	}
    #if SUPPORT_STG_SUPER_PLUGOUT == 1
    if (ret!=0)
    {
        if (drvl2_sd_read(device_id, 0, (INT32U *) buf, 1)!=0)
        {
            fs_sd_ms_plug_out_flag_en();
            DBG_PRINT ("============>SUPER PLUG OUT DETECTED<===========\r\n");
        }
    }
    #endif
	return ret;
}

#define K_SDX_LIMIT_BULK_SIZE	2048UL	// 1M byte

INT32S SDX_WriteSector_Limit_Bulk_Size(INT32U device_id, INT32U blkno, INT32U blkcnt, INT32U buf)
{
	INT32S ret = -1;
	INT32U bulk_cnt;

	while (blkcnt != 0)
	{
		bulk_cnt = (blkcnt < K_SDX_LIMIT_BULK_SIZE) ? blkcnt : K_SDX_LIMIT_BULK_SIZE;
		ret = SDX_WriteSector(device_id, blkno, bulk_cnt, buf);
		if (ret!=0)
			break;
		blkno += bulk_cnt;
		buf += (bulk_cnt*512UL);
		blkcnt -= bulk_cnt;
	}
	return ret;
}

INT32S SDX_Flush(INT32U device_id)
{
    INT32S ret;

    #if (FS_CACHE_FAT_EN == 1)
    cache_sync(device_id);
    #endif

    ret = drvl2_sd_flush(device_id);

    #if (FS_CACHE_FAT_EN == 1)
    if (ret != 0)
        fat_l1_cahhe_uninit(device_id);
    #endif

    SD_FS_PRINT("[%s] id=%d ret=%s\r\n", __FUNCTION__, device_id, ret);

	return 0;
}

//=== SD 0 setting ===//
INT32S SD0_Initial(void)
{
	return SDX_Initial(0);
}
INT32S SD0_Uninitial(void)
{
	return SDX_Uninitial(0);
}

void SD0_GetDrvInfo(struct DrvInfo* info)
{
	SDX_GetDrvInfo(0, info);
}

INT32S SD0_ReadSector(INT32U blkno, INT32U blkcnt, INT32U buf)
{
	return SDX_ReadSector(0, blkno, blkcnt, buf);
}

INT32S SD0_WriteSector(INT32U blkno, INT32U blkcnt, INT32U buf)
{
	//return SDX_WriteSector_Limit_Bulk_Size(0, blkno, blkcnt, buf);
	return SDX_WriteSector(0, blkno, blkcnt, buf);
}
INT32S SD0_Flush(void)
{
	return SDX_Flush(0);
}

struct Drv_FileSystem FS_SD0_driver = {
	"SD0",
	DEVICE_READ_ALLOW|DEVICE_WRITE_ALLOW,
	SD0_Initial,
	SD0_Uninitial,
	SD0_GetDrvInfo,
	SD0_ReadSector,
	SD0_WriteSector,
	SD0_Flush,
};

//=== SD 1 setting ===//
INT32S SD1_Initial(void)
{
	return SDX_Initial(1);
}
INT32S SD1_Uninitial(void)
{
	return SDX_Uninitial(1);
}

void SD1_GetDrvInfo(struct DrvInfo* info)
{
	SDX_GetDrvInfo(1, info);
}

INT32S SD1_ReadSector(INT32U blkno, INT32U blkcnt, INT32U buf)
{
	return SDX_ReadSector(1, blkno, blkcnt, buf);
}

INT32S SD1_WriteSector(INT32U blkno, INT32U blkcnt, INT32U buf)
{
	//return SDX_WriteSector_Limit_Bulk_Size(1, blkno, blkcnt, buf);
	return SDX_WriteSector(1, blkno, blkcnt, buf);
}
INT32S SD1_Flush(void)
{
	return SDX_Flush(1);
}

struct Drv_FileSystem FS_SD1_driver = {
	"SD1",
	DEVICE_READ_ALLOW|DEVICE_WRITE_ALLOW,
	SD1_Initial,
	SD1_Uninitial,
	SD1_GetDrvInfo,
	SD1_ReadSector,
	SD1_WriteSector,
	SD1_Flush,
};

#if (FS_CACHE_FAT_EN == 1)
static void FsCpy4(void *_dst, const void *_src, int len)
{

    #if 1
    memcpy((INT8S*)_dst, (INT8S*)_src, len);
    #else
    __asm(
        /*
        r0: *_dst
        r1: *_src
        r2: len
        */
        "add    r3, r0, r2      \n" /*r3: *end */
        "CONT_CPY:              \n"
        "ldmia  r1!,{r4-r11}    \n"
        "stmia  r0!,{r4-r11}    \n"
        "ldmia  r1!,{r4-r11}    \n"
        "stmia  r0!,{r4-r11}    \n"
        "subs   r4, r0, r3      \n" /*_dst - *end*/
        "bne    CONT_CPY        \n"
    );
    #endif
}
#endif

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#endif //(defined SD_EN) && (SD_EN == 1)                          //
//================================================================//
