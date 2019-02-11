#include <stdio.h>
#include "application.h"
#include "image_decoder.h"
#include "image_encoder.h"
#include "drv_l1_uart.h"
#include "drv_l2_display.h"
#include "usbh_uvc.h"

#define DECODE_JPG             			0

static INT16U width, height;
INT8U img_dec_outbuf_idx;

extern xQueueHandle hUVCDecBufQ;
extern xQueueHandle hUVCDispCtrlQ;
extern UVC_DECBUF_INFO_T uvc_decbuf_info;
extern INT32U uvc_disp_halted;
extern xQueueHandle hUSBHostUVCTaskQ;

static INT32U image_decode_end(INT32U decode_buf)
{
    drv_l2_display_update(DISPLAY_DEVICE, decode_buf);

	return decode_buf;
}

void display_init(void)
{
	drv_l2_display_init();
    drv_l2_display_start(DISPLAY_DEVICE,DISP_FMT_YUYV);
	drv_l2_display_get_size(DISPLAY_DEVICE,&width,&height);
}

INT32U display_VGA_to_QVGA_task(void)
{
	INT32S ret,temp;
	INT32U decode_state, decode_output_ptr, decode_type, type_flag, msg_id;
	IMAGE_ARGUMENT image_decode;
	MEDIA_SOURCE image_source;
	INT8U bufidx;
	INT16U dispmsg, dispmesg;

    DBG_PRINT("-disp task-\r\n");
    while(1)
    {
        if(hUVCDispCtrlQ && (xQueueReceive(hUVCDispCtrlQ, &dispmsg, 0) == pdTRUE))
        {
            cache_invalid_range(&dispmsg, sizeof(INT16U));
            switch(dispmsg&0x00FF)
            {
                case MSG_UVC_DISP_INIT:
                    DBG_PRINT("disp init!\r\n");
                    //image_codec_demo_init();
                    display_init();
                    //width = 320;
                    //height = 240;
                    //malloc frame buffer
                    img_dec_outbuf_idx = 0;
                    decode_output_ptr = (INT32U)gp_malloc_align((width * height *2)*IMAGE_DECODE_OUTBUF_NUM, 64);//malloc decode frame buffer
                    if(decode_output_ptr == 0)
                    {
                        DBG_PRINT("decode_output_ptr malloc fail\r\n");
                        goto _decode_exit;
                    }
                    else
                    {
                        gp_memset((INT8U *)decode_output_ptr, 0x00, (width * height * 2)*IMAGE_DECODE_OUTBUF_NUM);
                    }
                    //image source infomation
                    //image_source.type = SOURCE_TYPE_FS;                                    //image file infomation form file system
                    image_source.type = SOURCE_TYPE_SDRAM;
                    image_decode.OutputFormat = IMAGE_OUTPUT_FORMAT_YUYV;				//scaler out format
                    //image output infomation
                    image_decode.OutputBufPtr = (INT8U *)decode_output_ptr;           	//decode output buffer
                    image_decode.OutputBufWidth = width;                  				//width of output buffer
                    image_decode.OutputBufHeight = height;                 				//Height of output buffer
                    image_decode.OutputWidth = width;                     				//scaler width of output image
                    image_decode.OutputHeight = height;                    				//scaler Heigh of output image
                    image_decode.ScalerOutputRatio = FIT_OUTPUT_SIZE; 				 	//Fit to output_buffer_width and output_buffer_height for image output size
                    image_decode.OutBoundaryColor = (INT32U)0x008080; 				 	//set the black for out of boundary color

                    decode_type = DECODE_JPG;
                    image_decode_entrance();     										//global variable initial for image decode
                    image_decode_end_register_callback(image_decode_end);
                    //display_init();
                break;

                case MSG_UVC_DISP_DECODE_SCALE:
                    cache_invalid_range(&bufidx, sizeof(INT8U));
                    bufidx = (INT8U)((dispmsg&0xFF00)>>8);
//                    DBG_PRINT("-imgdec- %p\r\n", bufidx);
                    //image decode function
                    uvc_decbuf_info.bufkey[bufidx] = 1;
#if 0
                    cache_invalid_range(&uvc_decbuf_info.jpegdecbuf[bufidx][0], UVC_JPGDEC_BUF_SIZE);
                    image_source.type_ID.memptr = &(uvc_decbuf_info.jpegdecbuf[bufidx][0]);
#else
                    cache_invalid_range(uvc_decbuf_info.jpegdecbufptr[bufidx], UVC_JPGDEC_BUF_SIZE);
                    cache_invalid_range(&uvc_decbuf_info.total_len[bufidx], sizeof(INT32U));
                    image_source.type_ID.memptr = (uvc_decbuf_info.jpegdecbufptr[bufidx]);
                    image_source.type_ID.temp = uvc_decbuf_info.total_len[bufidx];
#endif
                    image_decode_start(image_decode, image_source);//image decode start
                    while (1)
                    {
                        decode_state = image_decode_status();
                        if (decode_state == IMAGE_CODEC_DECODE_END)
                        {
                            if(++img_dec_outbuf_idx > (IMAGE_DECODE_OUTBUF_NUM-1))
                            {
                                 img_dec_outbuf_idx = 0;
                            }
                            image_decode.OutputBufPtr = (INT8U *)(decode_output_ptr+((width * height * 2) * img_dec_outbuf_idx));
                            break;
                        }
                        else if(decode_state == IMAGE_CODEC_DECODE_FAIL)
                        {
                            DBG_PRINT("image decode failed\r\n");
                            break;
                        }
                    }
                    image_decode_stop();
                    uvc_decbuf_info.bufkey[bufidx] = 0;
                break;


                case MSG_UVC_DISP_HALT:
                    if(decode_output_ptr)
                    {
                        gp_free((void *)decode_output_ptr);
                        decode_output_ptr = 0;
                    }
                    image_decode_exit();
//                    image_codec_demo_uninit();
                    drv_l2_display_uninit();
                    if(hUSBHostUVCTaskQ)
                    {
                        msg_id = MSG_USB_HOST_UVC_RLS;
                        xQueueSend(hUSBHostUVCTaskQ, &msg_id, 0);
                    }
                break;

                default:

                break;
            }
        }
    }
_decode_exit:
    if(decode_output_ptr)
    {
		gp_free((void *)decode_output_ptr);
		decode_output_ptr = 0;
    }


	image_decode_exit();
	image_codec_demo_uninit();

	return STATUS_OK;
}
