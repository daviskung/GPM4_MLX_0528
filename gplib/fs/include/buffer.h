/****************************************************************/
/*                                                              */
/*                            buffer.h                          */
/*                                                              */
/* Sector buffer structure                                      */
/*                                                              */
/*                      Copyright (c) 2001                      */
/*			Bart Oldeman				*/
/*								*/
/*			Largely taken from globals.h:		*/
/*			Copyright (c) 1995, 1996                */
/*                      Pasquale J. Villani                     */
/*                      All Rights Reserved                     */
/*                                                              */
/* This file is part of DOS-C.                                  */
/*                                                              */
/* DOS-C is free software; you can redistribute it and/or       */
/* modify it under the terms of the GNU General Public License  */
/* as published by the Free Software Foundation; either version */
/* 2, or (at your option) any later version.                    */
/*                                                              */
/* DOS-C is distributed in the hope that it will be useful, but */
/* WITHOUT ANY WARRANTY; without even the implied warranty of   */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See    */
/* the GNU General Public License for more details.             */
/*                                                              */
/* You should have received a copy of the GNU General Public    */
/* License along with DOS-C; see the file COPYING.  If not,     */
/* write to the Free Software Foundation, 675 Mass Ave,         */
/* Cambridge, MA 02139, USA.                                    */
/****************************************************************/
#ifndef __BUFFER_H__
#define __BUFFER_H__

#define BUFFERSIZE (512>>WORD_PACK_VALUE)

struct buffer {
	struct buffer *b_next;                 /* next buffer in LRU list      */
	struct buffer *b_prev;                 /* previous buffer in LRU list  */
	INT8S b_unit;                  /* disk for this buffer         */
	INT8S b_flag;                  /* buffer flags                 */
	INT32U b_blkno;                /* block for this buffer        */
	INT8U b_copies;               /* number of copies to write    */
	INT16U b_offset;               /* offset in sectors between copies
	                                 to write for FAT sectors     */
	struct dpb *b_dpbp;     		/* pointer to DPB               */
#if FS_SMALL_CAPACITY_MEMORY == 1
	INT8U b_buffer[BUFFERSIZE/* + 512*/];   /* 512 byte sectors for now     */
#else
    INT8U b_buffer[BUFFERSIZE + 512];   /* 512 byte sectors for now     */
#endif
    /*這裡應是512就可以了，但是不確定之前是為什麼修改的，暫時只在RAM小的版本先修改，
	後面測試沒有問題后，再更新到標準的版本。
    20160718 xiadangdu */
};

#define BFR_DIRTY       0x40    /* buffer modified              */
#define BFR_VALID       0x20    /* buffer contains valid data   */
#define BFR_BITMAP		0x10	/* buffer maptable*/
#define BFR_DATA        0x08    /* buffer is from data area     */
#define BFR_DIR         0x04    /* buffer is from dir area      */
#define BFR_FAT         0x02    /* buffer is from fat area      */
#define BFR_UNCACHE     0x01    /* buffer to be released ASAP   */

//------------------------------------------------------
//for blockio define
#define b_next(bp) ((struct buffer *)(bp)->b_next)
#define b_prev(bp) ((struct buffer *)(bp)->b_prev)
#define bufptr(fbp) ((struct buffer *)(fbp))

#endif



