#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "application.h"
#include "drv_l1_spi.h"
#include "drv_l2_spi_flash.h"

#define UPDATE_FILE_NAME	"F:\\update_123456.bin"
#define SPI_FLASH_SPEED	10
#define SECTOR_SIZE			4096
#define PAGE_SIZE			256
#define UPDATE_BUFF_SIZE	SECTOR_SIZE//Sector size
#define SECTOR_PAGE_No		SECTOR_SIZE/PAGE_SIZE
void sd_update_demo(void)
{
	INT16S fid;
	INT8U str[8],*p1,*p2;
	INT32U buffer_addr,pbuf,Vbuffer_addr;
	struct stat_t state_buffer;
	INT32S filesize,ret;
	INT32U flash_id,page_addr;
	INT32U i;
	// Mount device
	while(1)
	{
		if(_devicemount(FS_SD1))
		{
			DBG_PRINT("Mount Disk Fail[%d]\r\n", FS_SD1);
		}
		else
		{
			DBG_PRINT("Mount Disk success[%d]\r\n", FS_SD1);
			break;
		}
	}
	fid = fs_open(UPDATE_FILE_NAME, O_RDONLY);
	if(fid < 0)
	{
		DBG_PRINT("update file open fail...\r\n");
		while(1);
	}
	drv_l2_spiflash_init(SPI_FLASH_SPEED);//spi1
	drv_l2_spiflash_read_id(str);
	
	DBG_PRINT("SPI ID : 0x%02x,0x%02x,0x%02x\r\n",str[0],str[1],str[2]);
	buffer_addr = (INT32U) gp_malloc_align(UPDATE_BUFF_SIZE,32);
	Vbuffer_addr = (INT32U) gp_malloc_align(UPDATE_BUFF_SIZE,32);
	
	fstat(fid, &state_buffer);
	filesize = state_buffer.st_size;
	DBG_PRINT("update file size=%d\r\n",filesize);
	if(filesize%UPDATE_BUFF_SIZE)
	{
		filesize &= ~(UPDATE_BUFF_SIZE-1);
		filesize += UPDATE_BUFF_SIZE;
	}
	DBG_PRINT("update write start....\r\n");
	for(flash_id=0;flash_id<filesize;flash_id+=SECTOR_SIZE)
	{
		if(fs_read(fid,buffer_addr,SECTOR_SIZE)<0)
		{
			DBG_PRINT("update file read fail[0x%x]....\r\n",flash_id);
			while(1);
		}
		if(drv_l2_spiflash_sector_erase(flash_id)<0)
		{
			DBG_PRINT("update sector erase fail[0x%x]....\r\n",flash_id);
			while(1);
		}
		page_addr = flash_id;
		pbuf = buffer_addr;
		for(i = 0; i < SECTOR_PAGE_No; i++)
		{
			if(drv_l2_spiflash_write_page(page_addr, (INT8U *)pbuf)<0)
			{
				DBG_PRINT("update page write file[0x%x][0x%x]...\r\n",flash_id,i);
				while(1);
			}
			page_addr += 256;
			pbuf += 256;
		}
		DBG_PRINT("write sector 0x%x\r\n",flash_id);
	}
	if(lseek(fid, 0, SEEK_SET)<0)
	{
		DBG_PRINT("update file seek fail...\r\n");
		while(1);
	}
	DBG_PRINT("update verify start....\r\n");
	p1 = buffer_addr;
	p2 = Vbuffer_addr;
	for(flash_id=0;flash_id<filesize;flash_id+=SECTOR_SIZE)
	{
		if(fs_read(fid,buffer_addr,SECTOR_SIZE)<0)
		{
			DBG_PRINT("update file read fail[0x%x]....\r\n",flash_id);
			while(1);
		}
		page_addr = flash_id;
		pbuf = Vbuffer_addr;
		for(i = 0; i < SECTOR_PAGE_No; i++)
		{
			if(drv_l2_spiflash_read_page(page_addr, (INT8U *)pbuf)<0)
			{
				DBG_PRINT("update page read file[0x%x][0x%x]...\r\n",flash_id,i);
				while(1);
			}
			page_addr += 256;
			pbuf += 256;
		}
		for(i=0;i<SECTOR_SIZE;i++)
		{
			if(p1[i]!=p2[i])
			{
				DBG_PRINT("update verify fail[0x%x]....\r\n",flash_id);
				while(1);
			}
		}
		DBG_PRINT("read back sector 0x%x\r\n",flash_id);
	}
	DBG_PRINT("SDC to Spiflash update finish.\r\n",flash_id);
	while(1);
}
