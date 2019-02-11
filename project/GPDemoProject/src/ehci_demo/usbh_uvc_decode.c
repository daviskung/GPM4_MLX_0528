/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2016 by Generalplus Inc.                         *
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
 * @file		usbh_uvc_decode.c
 * @brief		parsing uvc isoch packet and assembling and concatenating jpeg frame
 * @author		Jason Chang
 **/

#include "drv_l1_usbh.h"
#include "drv_l2_usbh.h"
#include "usbh_uvc.h"
#include "errno.h"

#ifdef USB_DEBUG
    #define DBGMSG        print_string//printf
#else
    #define DBGMSG(...)
#endif

//extern xQueueHandle hUVCDecBufQ;
extern xQueueHandle hUVCDispCtrlQ;
UVC_DECBUF_INFO_T uvc_decbuf_info;
INT8U uvc_last_fid;
INT8U jbuf_idx;
INT8U skip_frame = 0;

static int uvc_parse_start(UVC_DECBUF_INFO_T *bufinfo, char *data, INT32U len)
{
    INT8U fid;

    if(len < 2 || data[0] < 2 || data[0] > len)
		return -EINVAL;

	/* Skip payloads marked with the error bit ("error frames"). */
	if(data[1] & UVC_STREAM_ERR)
	{
		DBGMSG("Dropping payload (error bit set)\r\n");
		return -ENODATA;
	}
	fid = data[1] & UVC_STREAM_FID;
    if(skip_frame)
    {
        if(data[1] & UVC_STREAM_EOF)
        {
            skip_frame = 0;
            bufinfo->bytesused = 0;
            bufinfo->state = BUF_STATE_IDLE;
        }
        else if(data[12] == 0xFF && data[13] == 0xD8)
        {
            skip_frame = 0;
            bufinfo->bytesused = 0;
            bufinfo->state = BUF_STATE_IDLE;
            goto L_RS1;
        }
        return -ENODATA;
    }
L_RS1:
    cache_invalid_range(&bufinfo->bufkey[bufinfo->jpegdecbuf_idx], sizeof(INT8U));
    if(bufinfo->bufkey[bufinfo->jpegdecbuf_idx] == 1)//(JC)buffer in use
    {
//        DBG_PRINT("!-skip-!\r\n");
        skip_frame = 1;
		uvc_last_fid = fid;
		return -ENODATA;
	}
	if(bufinfo->state != BUF_STATE_ACTIVE)
	{
        DBGMSG("fid %p, uvc_last_fid %p, d[1]=%p\r\n", fid, uvc_last_fid, data[1]);
		if(fid == uvc_last_fid)
		{
			DBGMSG("Dropping payload (out of sync)\r\n");
			return -ENODATA;
		}

		/* TODO: Handle PTS and SCR. */
		bufinfo->state = BUF_STATE_ACTIVE;
	}
	if ((fid != uvc_last_fid) && (bufinfo->bytesused != 0))
	{
		DBGMSG("Frame complete (FID bit toggled)\r\n");
		bufinfo->state = BUF_STATE_DONE;
        DBGMSG("*img %x\r\n", bufinfo->bytesused);
		bufinfo->bytesused = 0;
		return -EAGAIN;
	}
	uvc_last_fid = fid;

	return data[0];
}

static void uvc_parse_data(UVC_DECBUF_INFO_T *bufinfo, char *data, INT32U len)
{
    unsigned int maxlen, nbytes;
    char *tgtmem;

	if (len <= 0)
		return;
	/* Copy the video data to the buffer. */
	maxlen = bufinfo->buflength - bufinfo->bytesused;
	tgtmem = bufinfo->bufhead + bufinfo->bytesused;
	nbytes = min(len, maxlen);
	cache_invalid_range(data, ISO_Max_Packet_Size);
	gp_memcpy(tgtmem, data, nbytes);
	bufinfo->bytesused += nbytes;

	/* Complete the current frame if the buffer size was exceeded. */
	if (len > maxlen)
	{
		DBGMSG("Frame complete (overflow)\r\n");
		DBG_PRINT("#img %p\r\n", bufinfo->bytesused);
		bufinfo->total_len[bufinfo->jpegdecbuf_idx] = bufinfo->bytesused;
		bufinfo->bytesused = 0;
		bufinfo->state = BUF_STATE_DONE;
	}

}

static void uvc_parse_end(UVC_DECBUF_INFO_T *bufinfo, char *data, INT32U len)
{
	if ((data[1] & UVC_STREAM_EOF) && (bufinfo->bytesused != 0))
	{
		DBGMSG("Frame complete (EOF found)\r\n");
		if (data[0] == len)
		{
			DBGMSG("EOF in empty payload\r\n");
        }
        DBGMSG("$img %x\r\n", bufinfo->bytesused);
        bufinfo->total_len[bufinfo->jpegdecbuf_idx] = bufinfo->bytesused;
        bufinfo->bytesused = 0;
		bufinfo->state = BUF_STATE_DONE;
	}
}


void uvc_parse_isoch(UVC_DECBUF_INFO_T *buffinfo, char *data_buff, int data_len)
{
    //Application gets Isochronous-In data here.
    char *tgt_buff;
    char *src_buff = data_buff;
    static int act_len;
    int ret;
    INT16U msg;

    act_len = data_len;
    tgt_buff = (buffinfo->jpegdecbufptr[buffinfo->jpegdecbuf_idx]);
    buffinfo->bufhead = tgt_buff;
    /* Decode the payload header. */
	do
	{
		ret = uvc_parse_start(buffinfo, src_buff, act_len);
		if (ret == -EAGAIN)
		{
/*
            if(hUVCDecBufQ)
            {
                xQueueSend(hUVCDecBufQ, (const void *)&buffinfo->jpegdecbuf_idx, 0);
            }
*/

            if(hUVCDispCtrlQ)
            {
                msg = ((buffinfo->jpegdecbuf_idx<<8)&0xFF00)|MSG_UVC_DISP_DECODE_SCALE;
                xQueueSend(hUVCDispCtrlQ, (const void *)&msg, 0);
            }

            if((++buffinfo->jpegdecbuf_idx) > (UVC_JPGDEC_BUF_MQUEUE_MAX-1))
            {
                buffinfo->jpegdecbuf_idx = 0;
            }

            buffinfo->state = BUF_STATE_IDLE;
        }
	} while (ret == -EAGAIN);

	if (ret < 0)
	{
        DBGMSG("err parsing uvc! %p\r\n", abs(ret));
		return;
    }

	/* Decode the payload data. */
	uvc_parse_data(buffinfo, src_buff + ret, (act_len - ret));//(JC)offset ret length of uvc header(normally 0x0C)

	/* Process the header again. */
	uvc_parse_end(buffinfo, src_buff, act_len);

	if (buffinfo->state == BUF_STATE_DONE || buffinfo->state == BUF_STATE_ERROR)
	{
//        if(hUVCDecBufQ)
//        {
//            xQueueSend(hUVCDecBufQ, (const void *)&buffinfo->jpegdecbuf_idx, 0);
//        }
//DBG_PRINT("buf hd=%p, idx=%p\r\n", buffinfo->bufhead, buffinfo->jpegdecbuf_idx);
        if(hUVCDispCtrlQ)
        {
            msg = ((buffinfo->jpegdecbuf_idx<<8)&0xFF00)|MSG_UVC_DISP_DECODE_SCALE;
            xQueueSend(hUVCDispCtrlQ, (const void *)&msg, 0);
        }
        if((++buffinfo->jpegdecbuf_idx) > (UVC_JPGDEC_BUF_MQUEUE_MAX-1))
        {
            buffinfo->jpegdecbuf_idx = 0;
        }
    }
}


void uvc_get_data_n_decode(INT32U itd_idx)
{
    INT32U itd_dw_offset, itd_dw_extrcnt;
	INT32U temp_len = 0;
	INT32U rcv_length, rcv_offset, rcv_page;
	INT8U *start_pageaddr;
	INT8U iso_status;
	INT32U i, j;
	INT32U cnt;
    //----------------------------------------------------------------------------------------------
    cache_invalid_range(&giTD_buf_use[0], HOST_UVC_ISO_Buff_CNT);
    cache_invalid_range(&giTD_buf_not_clear[0], HOST_UVC_ISO_Buff_CNT);
    cache_invalid_range(&itd_idx, sizeof(INT32U));
    cnt = 0;
    while(cnt<HOST_UVC_ISO_Buff_CNT)
    {
        if((giTD_buf_use[cnt] == 1) && (giTD_buf_not_clear[cnt] == 1) && (itd_idx == ((cnt+1)<<12)))
        {
            itd_dw_offset = (cnt*HOST_UVC_iTD_Table_Size*HOST_UVC_iTD_Table_CNT/4);
            itd_dw_extrcnt = ((cnt+1)*HOST_UVC_iTD_Table_Size*HOST_UVC_iTD_Table_CNT/4);
            cache_invalid_range((INT32U)(giTD_Table_Addr+(cnt*HOST_UVC_iTD_Table_Size*HOST_UVC_iTD_Table_CNT/4)), (HOST_UVC_iTD_Table_Size*HOST_UVC_iTD_Table_CNT));
            cache_invalid_range((INT32U)(g_buf+(cnt*HOST_UVC_ISO_DATA_Buff*HOST_UVC_iTD_Table_CNT)), (HOST_UVC_ISO_DATA_Buff*HOST_UVC_iTD_Table_CNT));
            break;
        }
        cnt++;
    }
    if(cnt == HOST_UVC_ISO_Buff_CNT)
    {
	    DBG_PRINT("!error get iso data*");
        return;
    }
    //----------------------------------------------------------------------------------------------
    cache_invalid_range(&itd_dw_offset, 4);
    cache_invalid_range(&itd_dw_extrcnt, 4);
    cache_invalid_range(&itd_idx, 4);
//  DBG_PRINT("itd ofst=%p, itd extrcnt=%p itd idx=%p\r\n",itd_dw_offset,itd_dw_extrcnt);
    while(itd_dw_offset < itd_dw_extrcnt)
    {
        for(i=0;i<8;i++)
        {
            rcv_length = ((*(giTD_Table_Addr + itd_dw_offset + i + 1) & 0x0FFF0000)>>16);
            if(rcv_length == 0)
            {
                continue;
            }
            else
            {
                iso_status = (*(giTD_Table_Addr + itd_dw_offset + i + 1) & 0xF0000000)>>28;
                cache_invalid_range(&iso_status, sizeof(INT8U));
                if(iso_status != 0)// status error
                {
                    skip_frame = 1;
                    continue;
                }
                rcv_offset = (*(giTD_Table_Addr + itd_dw_offset + i + 1) & 0xFFF);
				rcv_page = ((*(giTD_Table_Addr + itd_dw_offset + i + 1) & 0x7000)>>12);
				temp_len = ((*(giTD_Table_Addr + itd_dw_offset + 9 + rcv_page) & 0xFFFFF000) + rcv_offset);
				start_pageaddr = (INT8U *)temp_len;
				cache_invalid_range(start_pageaddr, rcv_length);
				uvc_parse_isoch(&uvc_decbuf_info, start_pageaddr, rcv_length);
            }
        }//for(i=0;i<8;i++)
        itd_dw_offset += 16;
    }
    for(i=0;i<HOST_UVC_iTD_Table_CNT;i++)
    {
        if((giTD_buf_use[i] == 1) && (giTD_buf_not_clear[i] == 1) && (itd_idx == (i+1)*0x1000))
        {
            giTD_buf_use[i] = 0;
            giTD_buf_not_clear[i] = 0;
		    break;
        }
    }
}

