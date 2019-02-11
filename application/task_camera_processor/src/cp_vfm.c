#include "cp_camera_processor.h"

INT32S vfm_init(vframe_manager_t* vfm, INT32U size, INT32U max_blk_size)
{
	mem_blk_t* mb;

	if (!vfm) {
		DBG_PRINT("vfm_init fail: invalid pointer\r\n");
		return -1;
	}

	if (size < 64) {
		DBG_PRINT("vfm_init fail: invalid size\r\n");
		return -1;
	}
	size = (size + 63) >> 6 << 6;
	vfm->start = (INT32U)gp_malloc_align(size, 64);
	//DBG_PRINT("free = %x\r\n", mm_free_get());

	if (!vfm->start){
		DBG_PRINT("vfm_init fail: out of memory\r\n");
		return -1;
	}

	vfm->end = vfm->start + size;
	vfm->current = vfm->start;
	vfm->mutex = osMutexCreate(NULL);
	vfm->blk_size = max_blk_size + 64;
	vfm->total_size = size;
	vfm->allocated_size = 0;
	vfm->allocated_block = 0;

	if (!vfm->mutex){
		DBG_PRINT("vfm_init fail: create mutex fail\r\n");
		gp_free(vfm->start);
		gp_memset(vfm, 0, sizeof(vframe_manager_t));
		return -1;
	}

	mb = (mem_blk_t*)vfm->start;
	mb->tag = 0xABCDEF01;
	mb->size = size - 64;
	mb->allocate = 0;

	DBG_PRINT("vfm init: start:0x%x, size = 0x%x, end = 0x%x max buf size = %x\r\n", vfm->start, size, vfm->end, vfm->blk_size);
	return 0;
}

INT32S vfm_reset(vframe_manager_t* vfm)
{
	mem_blk_t* mb;

	if (!vfm || !vfm->mutex) {
		DBG_PRINT("vfm_reset fail: invalid pointer\r\n");
		return -1;
	}

	osMutexWait(vfm->mutex, osWaitForever);

	vfm->current = vfm->start;
	vfm->allocated_size = 0;
	vfm->allocated_block = 0;

	mb = (mem_blk_t*)vfm->start;
	mb->tag = 0xABCDEF01;
	mb->size = vfm->end - vfm->start - 64;
	mb->allocate = 0;

	osMutexRelease(vfm->mutex);

	DBG_PRINT("vfm reset: start:0x%x, size = 0x%x, end = 0x%x\r\n", vfm->start, mb->size, vfm->end);

	return 0;
}

INT32S vfm_close(vframe_manager_t* vfm)
{
	DBG_PRINT("%s\r\n", __FUNCTION__);
	if (!vfm || !vfm->mutex) {
		DBG_PRINT("vfm_close fail: invalid pointer\r\n");
		return -1;
	}

	osMutexWait(vfm->mutex, osWaitForever);

	gp_free(vfm->start);
	vfm->start = 0;
	vfm->current = 0;
	vfm->end = 0;
	vfm->total_size = 0;
	vfm->allocated_size = 0;
	vfm->allocated_block = 0;

	osMutexRelease(vfm->mutex);

	DBG_PRINT("vfm close: start:0x%x, end = 0x%x\r\n", vfm->start, vfm->end);
	vQueueDelete(vfm->mutex);
	vfm->mutex = 0;

	return 0;
}

#define CHECK_MB(m)\
	if (m->tag != 0xABCDEF01 || m->allocate > 1)\
		DBG_PRINT("MBERROR[%p,%x,%x,%x,%d]", m, m->tag, m->size, m->allocate, __LINE__)

#define MERGE_MB(m,n)\
	m->size += (n->size + 64);\
	n->tag = 0

#define SPLIT_MB(m,s,n)\
	n = (mem_blk_t*)((INT32U)m + s);\
	n->tag = 0xABCDEF01;\
	n->size = m->size - s;\
	n->allocate = 0;\
	m->size = (s - 64)

#define FIT_SIZE(m,s) (m && m->tag == 0xABCDEF01 && !m->allocate && (m->size+64) >= s)
#define NEXT_MB(m) (mem_blk_t*)(((INT32U)m)+m->size+64)
#define DUMP_MB(m)	DBG_PRINT("DUMP[%p,%x,%x,%x,%d]", m, m->tag, m->size, m->allocate, __LINE__)

INT32S check_vfm(vframe_manager_t* vfm)
{
	mem_blk_t *mb;

	mb = (mem_blk_t*)vfm->start;
	while((INT32U)mb < vfm->end)
	{
		if (mb->tag != 0xABCDEF01 || mb->allocate > 1)
			while(1);
		mb = NEXT_MB(mb);
	}

	return 0;
}
INT32U vfm_get_empty(vframe_manager_t* vfm, INT32U blk_size)
{
	mem_blk_t *mb, *new_mb, *mb1, *mb2;
	INT32U frame_addr = 0;
	INT32U loop = 0, reach_end = 0, start;

	if (!vfm || !vfm->mutex) {
		DBG_PRINT("vfm_get_empty fail: invalid pointer\r\n");
		return 0;
	}

	if (!blk_size)
		blk_size = vfm->blk_size;
	else
		blk_size += 64;

	osMutexWait(vfm->mutex, osWaitForever);

	if (vfm->current >= vfm->end)
		vfm->current = vfm->start;

	start = vfm->current;

	mb1 = NULL;

	while(1) {
		mb = (mem_blk_t*)vfm->current;
		if (FIT_SIZE(mb, blk_size)) {
			frame_addr = (INT32U)mb +64;
			mb->allocate = 1;
			if ((mb->size +64) > blk_size) {
				SPLIT_MB(mb, blk_size, new_mb);
			}

			vfm->current = (INT32U)NEXT_MB(mb);
			break;
		}

		vfm->current = (INT32U)NEXT_MB(mb);

		if (vfm->current < vfm->end) {
			mb1 = NEXT_MB(mb);
			if (mb1 && !mb1->allocate && !mb->allocate) {
				MERGE_MB(mb, mb1);
				vfm->current = (INT32U)mb;
				if (start == (INT32U)mb1) {
					start = (INT32U)NEXT_MB(mb);
					if (start >= vfm->end)
						start = vfm->start;
				}
				continue;
			}
		}
		else
			vfm->current = vfm->start;

		if (vfm->current == start)
			break;
	}

	if(!frame_addr) {
		DBG_PRINT("vfm_get_empty fail [a:%x (%d), total:%x, req:%x]\r\n", vfm->allocated_size, vfm->allocated_block, vfm->total_size, blk_size);
/*		mb = (mem_blk_t*)vfm->start;
		if (vfm->total_size < 0x200000)
		while((INT32U)mb < vfm->end)
		{
			DBG_PRINT("[%p][%x][%d]\r\n", mb, mb->size, mb->allocate);
			mb = NEXT_MB(mb);
		}*/
	}
	else {
		vfm->allocated_size += (mb->size + 64);
		vfm->allocated_block ++;
	}
	//DBG_PRINT("X");
	osMutexRelease(vfm->mutex);

	return frame_addr;
}

INT32S vfm_report_size(vframe_manager_t* vfm, INT32U frame_addr, INT32U size)
{
	mem_blk_t *mb, *new_mb, *next_mb;

	if (!vfm || !frame_addr || !vfm->mutex) {
		DBG_PRINT("vfm_report_size fail: invalid pointer or addr\r\n");
		return -1;
	}

	osMutexWait(vfm->mutex, osWaitForever);

	mb = (mem_blk_t*)(frame_addr - 64);

	if (mb->tag != 0xABCDEF01) {
		DBG_PRINT("vfm_report_size fail: invalid memory [%x]\r\n", frame_addr);
		osMutexRelease(vfm->mutex);
		return -1;
	}

	size = (size + 63) >> 6 << 6;

	if (size > mb->size)
	{
		DBG_PRINT("vfm_report_size fail: size overflow mb = %x size = %x bufsize = %x\r\n", mb, size, mb->size);
		osMutexRelease(vfm->mutex);
		return -1;
	}

	vfm->allocated_size -= (mb->size + 64);

	if (size < (mb->size - 64)) {
		SPLIT_MB(mb, (size+64), new_mb);

		next_mb = NEXT_MB(new_mb);

		if ((INT32U)next_mb < vfm->end && next_mb->tag == 0xABCDEF01 && next_mb->allocate == 0)
		{
			//merge blocks
			MERGE_MB(new_mb, next_mb);
			if (vfm->current == (INT32U)next_mb)
				vfm->current = (INT32U)new_mb;
		}
	}

	//vfm->current = (INT32U)new_mb;
	vfm->allocated_size += (mb->size + 64);

	osMutexRelease(vfm->mutex);

	//DBG_PRINT("r [%x, %x][%x, %x]\r\n", mb, mb->size, (INT32U)new_mb, new_mb->size);
	return 0;
}

INT32S vfm_post_empty(vframe_manager_t* vfm, INT32U frame_addr)
{
	mem_blk_t *mb, *next_mb, *prev_mb;

	if (!vfm || !frame_addr || !vfm->mutex) {
		DBG_PRINT("vfm_post_empty fail: invalid pointer or addr\r\n");
		return -1;
	}

	osMutexWait(vfm->mutex, osWaitForever);

	mb = (mem_blk_t*)(frame_addr - 64);

	if (mb->tag != 0xABCDEF01) {
		DBG_PRINT("vfm_post_empty fail: invalid memory\r\n");
		osMutexRelease(vfm->mutex);
		return -1;
	}

	mb->allocate = 0;

	vfm->allocated_size -= (mb->size + 64);
	vfm->allocated_block --;

	//DBG_PRINT("p [%x, %x] c[%x]\r\n", mb, mb->size, vfm->current);

	next_mb = NEXT_MB(mb);

	if ((INT32U)next_mb < vfm->end && next_mb->tag == 0xABCDEF01 && next_mb->allocate == 0)
	{
		//merge blocks
		MERGE_MB(mb, next_mb);
		if (vfm->current == (INT32U)next_mb)
			vfm->current = (INT32U)mb;

		//DBG_PRINT("p p[%x, %x] c[%x]\r\n", (INT32U)mb, mb->size, vfm->current);
	}

	/*prev_mb = (mem_blk_t*)((INT32U)vfm->current);

	if (prev_mb && mb!=prev_mb){
		if ((INT32U)mb == vfm->current + prev_mb->size + 64) {
			prev_mb->size += mb->size + 64;
			mb->tag = 0;
		}
		//DBG_PRINT("p p[%x, %x] c[%x]\r\n", (INT32U)prev_mb, prev_mb->size, vfm->current);
	}
	else {
		vfm->current = (INT32U)mb;
		//DBG_PRINT("p p[%x, %x] c[%x]\r\n", (INT32U)mb, mb->size, vfm->current);
	}*/

	osMutexRelease(vfm->mutex);
	return 0;
}
