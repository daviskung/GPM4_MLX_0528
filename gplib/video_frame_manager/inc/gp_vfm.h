#ifndef __GP_VFM_H__
#define __GP_VFM_H__

#include "cmsis_os.h"

typedef struct mem_blk_s
{
	INT32U tag;
	INT32U size;
	INT32U allocate;
	//INT32U pad[13]; //padding to 64 bytes
} mem_blk_t;

typedef struct vframe_manager_s
{
	INT32U start;
	INT32U end;
	INT32U current;
	INT32U blk_size;
	INT32U total_size;
	INT32U allocated_size;
	INT32U allocated_block;
	INT32U ext_addr;
	osMutexId mutex;
} vframe_manager_t;

INT32S vfm_init(vframe_manager_t* vfm, INT32U ext_addr, INT32U size, INT32U max_blk_size);
INT32S vfm_reset(vframe_manager_t* vfm);
INT32S vfm_close(vframe_manager_t* vfm);
INT32S check_vfm(vframe_manager_t* vfm);
INT32U vfm_get_empty(vframe_manager_t* vfm, INT32U blk_size);
INT32S vfm_report_size(vframe_manager_t* vfm, INT32U frame_addr, INT32U size);
INT32S vfm_post_empty(vframe_manager_t* vfm, INT32U frame_addr);
#endif
