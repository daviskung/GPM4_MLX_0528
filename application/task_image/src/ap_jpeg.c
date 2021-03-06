/*
* Description: This file parse/decode/scale jpeg images
*
* Author: Tristan Yang
*
* Date: 2008/07/19
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*/
#include "ap_jpeg.h"
#include "turnkey_filesrv_task.h"
#include "gplib_jpeg.h"
#include "gplib_jpeg_decode.h"
#include "gplib_jpeg_encode.h"
#include "drv_l1_jpeg.h"
#include "drv_l1_scaler.h"
#include "drv_l1_cache.h"
#include "drv_l2_scaler.h"

#define JPG_FS_TIMEOUT		2000
#if (defined APP_IMAGE_CODEC_EN ) && (APP_IMAGE_CODEC_EN == 1)

#define C_USE_SCALER_NUMBER				SCALER_0

#define C_JPEG_READ_FILE_BUFFER_SIZE		4096        // 4K file buffer, must less than (16K-100) bytes when internal sram is used
#define C_SCALER_L2_EN                  1
#if C_SCALER_L2_EN == 1
static ScalerFormat_t	jpeg_src={0};
static ScalerPara_t		jpeg_para={0};
#endif
// Image decoding relative global definitions and variables
static INT32U jpeg_output_format;
static INT8U jpeg_output_ratio;             // 0=Fit to output_buffer_width and output_buffer_height, 1=Maintain ratio and fit to output_buffer_width or output_buffer_height, 2=Same as 1 but without scale up
static INT16U jpeg_output_buffer_width;
static INT16U jpeg_output_buffer_height;
static INT16U jpeg_output_image_width;
static INT16U jpeg_output_image_height;
static INT32U out_of_boundary_color;
static INT32U jpeg_output_buffer_pointer;

// Control variables
static INT32U jpeg_fifo_line_num = 32;
static INT32U size_temp;

// JPEG variables
static INT16U jpeg_valid_width, jpeg_valid_height, jpeg_image_yuv_mode;
static INT32U jpeg_extend_width, jpeg_extend_height;
static INT32U jpeg_fifo_register = C_JPG_FIFO_32LINE;
static INT8U thumbnail_exist;
static INT16U thumbnail_width;
static INT16U thumbnail_height;
static INT16U thumbnail_yuv_mode;

// Buffer addresses
static INT32U jpeg_out_y, jpeg_out_cb, jpeg_out_cr;
static INT32U scaler_out_buffer;

// Scaler variables
static INT32U scaler_fifo_register = C_SCALER_CTRL_FIFO_32LINE;//C_SCALER_CTRL_FIFO_DISABLE;//C_SCALER_CTRL_FIFO_32LINE;
extern MSG_Q_ID	fs_msg_q_id;
// Function declaration
void jpeg_memory_allocate(INT32U fifo);
void jpeg_scaler_set_parameters(INT32U fifo);

#if (defined _DRV_L1_JPEG) && (_DRV_L1_JPEG == 1)&& (GPLIB_JPEG_ENCODE_EN==1)
INT32S jpeg_encode_init_header(IMAGE_ENCODE_STRUCT* img_encode_struct);

static INT8U jpeg_encode_422_header[IMAGE_ENCODE_JPEG_HEADER_LENGTH] = {
	0xFF, 0xD8, 0xFF, 0xFE, 0x00, 0x0B, 0x47, 0x50, 0x45, 0x6E, 0x63, 0x6F, 0x64, 0x65, 0x72, 0xFF,
#if 0		// Quality 50
	0xDB, 0x00, 0x43, 0x00, 0x10, 0x0B, 0x0C, 0x0E, 0x0C, 0x0A,	0x10, 0x0E, 0x0D, 0x0E, 0x12, 0x11,
	0x10, 0x13, 0x18, 0x28, 0x1A, 0x18, 0x16, 0x16, 0x18, 0x31, 0x23, 0x25, 0x1D, 0x28, 0x3A, 0x33,
	0x3D, 0x3C, 0x39, 0x33, 0x38, 0x37, 0x40, 0x48, 0x5C, 0x4E, 0x40, 0x44, 0x57, 0x45, 0x37, 0x38,
	0x50, 0x6D, 0x51, 0x57, 0x5F, 0x62, 0x67, 0x68, 0x67, 0x3E, 0x4D, 0x71, 0x79, 0x70, 0x64, 0x78,
	0x5C, 0x65, 0x67, 0x63, 0xFF, 0xDB, 0x00, 0x43, 0x01, 0x11,	0x12, 0x12, 0x18, 0x15, 0x18, 0x2F,
	0x1A, 0x1A, 0x2F, 0x63, 0x42, 0x38, 0x42, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
	0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
	0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
	0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0xF0,
#else		// Quality 100
	0xDB, 0x00, 0x43, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,	0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,	0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0xFF, 0xDB, 0x00, 0x43, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,	0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0xF0,
#endif

	0x01, 0x40, 0x03, 0x01, 0x21, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xC4, 0x00, 0x1F,
	0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00,
	0xB5, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00,
	0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51,
	0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52,
	0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26,
	0x27, 0x28, 0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67,
	0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
	0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3,
	0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
	0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,
	0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xC4, 0x00, 0x1F, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
	0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00, 0xB5, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04,
	0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04,
	0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14,
	0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16,
	0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36,
	0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56,
	0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76,
	0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94,
	0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2,
	0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9,
	0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
	0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDD, 0x00, 0x04,
	0x00, 0x00, 0xFF, 0xDA, 0x00, 0x0C, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00, 0x3F, 0x00
};
static INT8U Q50_L_table[64]=
{
	0x10, 0x0B, 0x0A, 0x10, 0x18, 0x28, 0x33, 0x3D, 0x0C, 0x0C, 0x0E, 0x13, 0x1A, 0x3A, 0x3C, 0x37,
	0x0E, 0x0D, 0x10, 0x18, 0x28, 0x39, 0x45, 0x38, 0x0E, 0x11, 0x16, 0x1D, 0x33, 0x57, 0x50, 0x3E,
	0x12, 0x16, 0x25, 0x38, 0x44, 0x6D, 0x67, 0x4D, 0x18, 0x23, 0x37, 0x40, 0x51, 0x68, 0x71, 0x5C,
	0x31, 0x40, 0x4E, 0x57, 0x67, 0x79, 0x78, 0x65, 0x48, 0x5C, 0x5F, 0x62, 0x70, 0x64, 0x67, 0x63
};
static INT8U Q50_C_table[64]=
{
	0x11, 0x12, 0x18, 0x2F, 0x63, 0x63, 0x63, 0x63, 0x12, 0x15, 0x1A, 0x42, 0x63, 0x63, 0x63, 0x63,
	0x18, 0x1A, 0x38, 0x63, 0x63, 0x63, 0x63, 0x63, 0x2F, 0x42, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
	0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
	0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63
};
static INT8U Q70_L_table[64]=
{
    0x0A, 0x07, 0x06, 0x0A, 0x0E, 0x18, 0x1F, 0x25, 0x07, 0x07, 0x08, 0x0B, 0x10, 0x23, 0x24, 0x21,
    0x08, 0x08, 0x0A, 0x0E, 0x18, 0x22, 0x29, 0x22, 0x08, 0x0A, 0x0D, 0x11, 0x1F, 0x34, 0x30, 0x25,
    0x11, 0x0D, 0x16, 0x22, 0x29, 0x41, 0x3E, 0x2E, 0x0E, 0x15, 0x21, 0x26, 0x31, 0x3E, 0x44, 0x37,
    0x1D, 0x26, 0x2F, 0x34, 0x3E, 0x49, 0x48, 0x3D, 0x2B, 0x37, 0x39, 0x3B, 0x43, 0x3C, 0x3E, 0x3B
};
static INT8U Q70_C_table[64]=
{
    0x0A, 0x0B, 0x0E, 0x1C, 0x3B, 0x3B, 0x3B, 0x3B, 0x0B, 0x0D, 0x10, 0x28, 0x3B, 0x3B, 0x3B, 0x3B,
    0x0E, 0x10, 0x22, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x1C, 0x28, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B,
    0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B,
    0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B, 0x3B
};
static INT8U Q80_L_table[64]=
{
    0x06, 0x04, 0x04, 0x06, 0x0A, 0x10, 0x14, 0x18, 0x05, 0x05, 0x06, 0x08, 0x0A, 0x17, 0x18, 0x16,
    0x06, 0x05, 0x06, 0x0A, 0x10, 0x17, 0x1C, 0x16, 0x06, 0x07, 0x09, 0x0C, 0x14, 0x23, 0x20, 0x19,
    0x07, 0x09, 0x0F, 0x16, 0x1B, 0x2C, 0x29, 0x1F, 0x0A, 0x0E, 0x16, 0x1A, 0x20, 0x2A, 0x2D, 0x25,
    0x14, 0x1A, 0x1F, 0x23, 0x29, 0x30, 0x30, 0x28, 0x1D, 0x25, 0x26, 0x27, 0x2D, 0x28, 0x29, 0x28
};
static INT8U Q80_C_table[64]=
{
    0x07, 0x07, 0x0A, 0x13, 0x28, 0x28, 0x28, 0x28, 0x07, 0x08, 0x0A, 0x1A, 0x28, 0x28, 0x28, 0x28,
    0x0A, 0x0A, 0x16, 0x28, 0x28, 0x28, 0x28, 0x28, 0x13, 0x1A, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
    0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
    0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28
};
static INT8U Q85_L_table[64]=
{
	0x05, 0x03, 0x03, 0x05, 0x07, 0x12, 0x15, 0x18, 0x04, 0x04, 0x04, 0x06, 0x08, 0x17, 0x18, 0x17,
	0x04, 0x04, 0x05, 0x07, 0x12, 0x17, 0x21, 0x17, 0x04, 0x05, 0x07, 0x09, 0x15, 0x26, 0x24, 0x19,
	0x05, 0x07, 0x11, 0x17, 0x20, 0x33, 0x31, 0x23, 0x07, 0x11, 0x17, 0x19, 0x24, 0x31, 0x34, 0x28,
	0x15, 0x19, 0x23, 0x26, 0x31, 0x36, 0x36, 0x30, 0x22, 0x28, 0x29, 0x29, 0x34, 0x30, 0x31, 0x30
};
static INT8U Q85_C_table[64]=
{
	0x05, 0x05, 0x07, 0x14, 0x30, 0x30, 0x30, 0x30, 0x05, 0x06, 0x08, 0x20, 0x30, 0x30, 0x30, 0x30,
	0x07, 0x08, 0x17, 0x30, 0x30, 0x30, 0x30, 0x30, 0x14, 0x20, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
	0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
	0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30
};
static INT8U Q90_L_table[64]=
{
    0x03, 0x02, 0x02, 0x03, 0x05, 0x08, 0x0A, 0x0C, 0x02, 0x02, 0x03, 0x04, 0x05, 0x0C, 0x0C, 0x0B,
    0x03, 0x03, 0x03, 0x05, 0x08, 0x0B, 0x0E, 0x0B, 0x03, 0x03, 0x04, 0x06, 0x0A, 0x11, 0x10, 0x0C,
    0x04, 0x04, 0x07, 0x0B, 0x0E, 0x16, 0x15, 0x0F, 0x05, 0x07, 0x0B, 0x0D, 0X10, 0x15, 0x17, 0x12,
    0x0A, 0x0D, 0x10, 0x11, 0x15, 0x18, 0x18, 0x14, 0x0E, 0x12, 0x13, 0x14, 0x16, 0x14, 0x15, 0x14
};
static INT8U Q90_C_table[64]=
{
    0x03, 0x04, 0x05, 0x09, 0x14, 0x14, 0x14, 0x14, 0x04, 0x04, 0x05, 0x0D, 0x14, 0x14, 0x14, 0x14,
    0x05, 0x05, 0x0B, 0x14, 0x14, 0x14, 0x14, 0x14, 0x09, 0x0D, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
    0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
    0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14
};
static INT8U Q93_L_table[64]=
{
    0x02, 0x02, 0x01, 0x02, 0x03, 0x06, 0x07, 0x09, 0x02, 0x02, 0x02, 0x03, 0x04, 0x08, 0x08, 0x08,
    0x02, 0x02, 0x02, 0x03, 0x06, 0x08, 0x0A, 0x08, 0x02, 0x02, 0x03, 0x04, 0x07, 0x0C, 0x0B, 0x09,
    0x03, 0x03, 0x05, 0x08, 0x0A, 0x0F, 0x0E, 0x0B, 0x03, 0x05, 0x08, 0x09, 0x0B, 0x0F, 0x10, 0x0D,
    0x07, 0x09, 0x0B, 0x0C, 0x0E, 0x11, 0x11, 0x0E, 0x0A, 0x0D, 0x0D, 0x0E, 0x10, 0x0E, 0x0E, 0x0E
};
static INT8U Q93_C_table[64]=
{
    0x02, 0x03, 0x03, 0x07, 0x0E, 0x0E, 0x0E, 0x0E, 0x03, 0x03, 0x04, 0x09, 0x0E, 0x0E, 0x0E, 0x0E,
    0x03, 0x04, 0x08, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x07, 0x09, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
    0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
    0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E
};
static INT8U Q95_L_table[64]=
{
	0x02, 0x01, 0x01, 0x02, 0x02, 0x04, 0x05, 0x06, 0x01, 0x01, 0x01, 0x02, 0x03, 0x06, 0x06, 0x06,
	0x01, 0x01, 0x02, 0x02, 0x04, 0x06, 0x07, 0x06, 0x01, 0x02, 0x02, 0x03, 0x05, 0x09, 0x08, 0x06,
	0x02, 0x02, 0x04, 0x06, 0x07, 0x0B, 0x0A, 0x08, 0x02, 0x04, 0x06, 0x06, 0x08, 0x0A, 0x0B, 0x09,
	0x05, 0x06, 0x08, 0x09, 0x0A, 0x0C, 0x0C, 0x0A, 0x07, 0x09, 0x0A, 0x0A, 0x0B, 0x0A, 0x0A, 0x0A,
};
static INT8U Q95_C_table[64]=
{
	0x02, 0x02, 0x02, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x02, 0x02, 0x03, 0x07, 0x0A, 0x0A, 0x0A, 0x0A,
	0x02, 0x03, 0x06, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,	0x05, 0x07, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
	0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,	0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
	0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,	0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
};
static INT8U Q98_L_table[64]=
{
    0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02,
    0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x03, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x03, 0x03, 0x02,
    0x01, 0x01, 0x01, 0x02, 0x03, 0x04, 0x04, 0x03, 0x01, 0x01, 0x02, 0x03, 0x03, 0x04, 0x05, 0x04,
    0x02, 0x03, 0x03, 0x03, 0x04, 0x05, 0x05, 0x04, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04
};
static INT8U Q98_C_table[64]=
{
    0x01, 0x01, 0x01, 0x02, 0x04, 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x03, 0x04, 0x04, 0x04, 0x04,
    0x01, 0x01, 0x02, 0x04, 0x04, 0x04, 0x04, 0x04, 0x02, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04
};

INT32S jpeg_buffer_encode(IMAGE_ENCODE_STRUCT *img_encode_struct)
{
	INT32S encode_state;

	if (!img_encode_struct) {
		return -1;
	}

	if ((img_encode_struct->InputWidth&0xF) || (img_encode_struct->InputHeight&0x7) || (img_encode_struct->OutputBuf&0xF)) {
		img_encode_struct->EncodeState = -1;

		return -1;
	}

	jpeg_encode_init_header(img_encode_struct);

	jpeg_encode_init();
	// Load default qunatization table.
	gplib_jpeg_default_quantization_table_load(img_encode_struct->QuantizationQuality);
	// Load default huffman table.
	gplib_jpeg_default_huffman_table_load();
	jpeg_encode_input_size_set(img_encode_struct->InputWidth, img_encode_struct->InputHeight);
	jpeg_encode_input_format_set(img_encode_struct->InputFormat);
	jpeg_encode_yuv_sampling_mode_set(img_encode_struct->OutputFormat);
	jpeg_encode_output_addr_set((INT32U) (img_encode_struct->OutputBuf+IMAGE_ENCODE_JPEG_HEADER_LENGTH));
	jpeg_encode_once_start(img_encode_struct->InputBuf_Y, img_encode_struct->InputBuf_U, img_encode_struct->InputBuf_V);
	while (1) {
		encode_state = jpeg_encode_status_query(1);
        if (encode_state & C_JPG_STATUS_ENCODE_DONE) {
			// Get encode length
		  	img_encode_struct->EncodeSize = jpeg_encode_vlc_cnt_get();
          	cache_invalid_range(img_encode_struct->InputBuf_Y, img_encode_struct->EncodeSize);
		  	img_encode_struct->EncodeSize += IMAGE_ENCODE_JPEG_HEADER_LENGTH;
		  	img_encode_struct->EncodeState = 0;
          	break;
        } else if (encode_state & C_JPG_STATUS_STOP) {
		    DBG_PRINT("JPEG is not started!\r\n");
			img_encode_struct->EncodeState = -1;
		    break;
	    } else if (encode_state & C_JPG_STATUS_TIMEOUT) {
		    DBG_PRINT("JPEG execution timeout-2!\r\n");
			img_encode_struct->EncodeState = -1;
		    break;
	    } else if (encode_state & C_JPG_STATUS_INIT_ERR) {
		    DBG_PRINT("JPEG init error!\r\n");
			img_encode_struct->EncodeState = -1;
		    break;
	    } else {
	        DBG_PRINT("jpeg_status = 0x%08x!\r\n", (INT32U)encode_state);
			img_encode_struct->EncodeState = -1;
			break;
        }
	}
	jpeg_encode_stop();

	return 0;
}

INT32S jpeg_encode_init_header(IMAGE_ENCODE_STRUCT *img_encode_struct)
{
	INT8S *ptr_src_l, *ptr_src_c;
	INT8S *ptr_dec;
    INT32U i;

	if (!img_encode_struct) {
		return -1;
	}

	ptr_dec = (INT8S *) img_encode_struct->OutputBuf;
	gp_memcpy((INT8S *) ptr_dec, (INT8S *) jpeg_encode_422_header, IMAGE_ENCODE_JPEG_HEADER_LENGTH);

	switch (img_encode_struct->QuantizationQuality) {
	case 50:
		ptr_src_l = (INT8S *) Q50_L_table;
		ptr_src_c = (INT8S *) Q50_C_table;
		break;
	case 70:
		ptr_src_l = (INT8S *) Q70_L_table;
		ptr_src_c = (INT8S *) Q70_C_table;
		break;
	case 80:
		ptr_src_l = (INT8S *) Q80_L_table;
		ptr_src_c = (INT8S *) Q80_C_table;
		break;
	case 85:
		ptr_src_l = (INT8S *) Q85_L_table;
		ptr_src_c = (INT8S *) Q85_C_table;
		break;
	case 93:
		ptr_src_l = (INT8S *) Q93_L_table;
		ptr_src_c = (INT8S *) Q93_C_table;
		break;
	case 95:
		ptr_src_l = (INT8S *) Q95_L_table;
		ptr_src_c = (INT8S *) Q95_C_table;
		break;
	case 98:
		ptr_src_l = (INT8S *) Q98_L_table;
		ptr_src_c = (INT8S *) Q98_C_table;
		break;
	case 90:
	default:
		ptr_src_l = (INT8S *) Q90_L_table;
		ptr_src_c = (INT8S *) Q90_C_table;
		break;
	}

	if (100 != img_encode_struct->QuantizationQuality) {
		//Luminance
		for(i=0; i<64; i++) {
			ptr_dec[i+0x14] = (INT8U) (*(ptr_src_l + zigzag_scan[i]) & 0xFF);
		}
		//Chrominance
		for(i=0; i<64; i++) {
			ptr_dec[i+0x59] = (INT8U) (*(ptr_src_c + zigzag_scan[i]) & 0xFF);
		}
	}
	ptr_dec[IMAGE_ENCODE_JPEG_HEIGHT_H_OFFSET] = (img_encode_struct->InputHeight>>8) & 0xFF;
	ptr_dec[IMAGE_ENCODE_JPEG_HEIGHT_L_OFFSET] = (img_encode_struct->InputHeight) & 0xFF;
	ptr_dec[IMAGE_ENCODE_JPEG_WIDTH_H_OFFSET] = (img_encode_struct->InputWidth>>8) & 0xFF;
	ptr_dec[IMAGE_ENCODE_JPEG_WIDTH_L_OFFSET] = (img_encode_struct->InputWidth) & 0xFF;

	return 0;
}
#endif

INT32S jpeg_file_parse_header(IMAGE_HEADER_PARSE_STRUCT *parser)
{
	TK_FILE_SERVICE_STRUCT fs_cmd;
	INT32U cmd_id;
	INT32S parse_status;
	INT32U fs_buffer_ptr;
	INT32S len;
  #if (_OPERATING_SYSTEM == _OS_UCOS2)
	INT8U error;
  #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
    osEvent result;
  #endif

  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
  	INT8U jpeg_need_invert = 1;
  #endif

	cmd_id = parser->cmd_id;

	fs_buffer_ptr = (INT32U) gp_malloc_align(C_JPEG_READ_FILE_BUFFER_SIZE, 16);
	if (fs_buffer_ptr == (INT32U) NULL) {
		DBG_PRINT("image_parse_header() failed to allocate memory\r\n");
		parser->parse_status = -2;

		return -2;
	}

	fs_cmd.fd = (INT16S) parser->image_source;
	fs_cmd.result_queue = image_task_fs_queue_a;
	fs_cmd.buf_addr = (INT32U) fs_buffer_ptr;
	fs_cmd.buf_size = C_JPEG_READ_FILE_BUFFER_SIZE;

	lseek((INT16S) parser->image_source, 0, SEEK_SET);

	jpeg_decode_init();
	gplib_jpeg_default_huffman_table_load(); //load default huffman table before parse header
	do {
		msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		len = (INT32S) OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
		if (error!=OS_NO_ERR || len<0) {
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		result = osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
		len = (INT32S)result.value.v;
		if (result.status !=osEventMessage || len<0) {
		#endif
			parser->parse_status = STATUS_FAIL;
			gp_free((void *) fs_buffer_ptr);

			return STATUS_FAIL;
		}

		// Check whether we have to break decoding this image
		if (image_task_handle_remove_request(cmd_id) > 0) {
			parser->parse_status = STATUS_FAIL;
			gp_free((void *) fs_buffer_ptr);

			return STATUS_FAIL;
		}

	  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
		if (jpeg_need_invert) {
			INT8U *ptr8;

			ptr8 = (INT8U *) fs_buffer_ptr;
			if (*ptr8==0x00 && *(ptr8+1)==0x27) {
				INT32U *ptr32;
				INT8U i;

				ptr32 = (INT32U *) fs_buffer_ptr;
				for (i=0; i<128; i++) {
					*ptr32++ ^= 0xFFFFFFFF;
				}
			}
		}
		jpeg_need_invert = 0;
	  #endif

		parse_status = jpeg_decode_parse_header((INT8U *) fs_buffer_ptr, len);
	} while (parse_status == JPEG_PARSE_NOT_DONE) ;
	if (parse_status != JPEG_PARSE_OK) {
		DBG_PRINT("Parse header failed. Skip this file\r\n");
		parser->parse_status = STATUS_FAIL;
		gp_free((void *) fs_buffer_ptr);

		return STATUS_FAIL;
	}

	parser->width = jpeg_decode_image_width_get();
	parser->height = jpeg_decode_image_height_get();
	if (jpeg_decode_image_progressive_mode_get()) {
		parser->image_type = TK_IMAGE_TYPE_PROGRESSIVE_JPEG;
	}
	// gplib jpeg
	//parser->orientation = jpeg_decode_image_orientation_get();
	if (parser->orientation<1 || parser->orientation>8) {
		parser->orientation = 1;
	}
	jpeg_decode_date_time_get(parser->date_time_ptr);
	parser->parse_status = STATUS_OK;
	gp_free((void *) fs_buffer_ptr);

	return STATUS_OK;
}

INT32S jpeg_buffer_parse_header(IMAGE_HEADER_PARSE_STRUCT *parser)
{
	INT32S parse_status;
  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
	INT8U *ptr8;

	ptr8 = (INT8U *) parser->image_source;
	if (*ptr8==0x00 && *(ptr8+1)==0x27) {
		INT32U *ptr32;
		INT8U i;

		ptr32 = (INT32U *) parser->image_source;
		for (i=0; i<128; i++) {
			*ptr32++ ^= 0xFFFFFFFF;
		}
	}
  #endif

	parse_status = jpeg_decode_parse_header((INT8U *) parser->image_source, parser->source_size);
	if (parse_status != JPEG_PARSE_OK) {
		DBG_PRINT("Parse header failed. Skip this file\r\n");
		parser->parse_status = STATUS_FAIL;

		return STATUS_FAIL;
	}

	parser->width = jpeg_decode_image_width_get();
	parser->height = jpeg_decode_image_height_get();
	if (jpeg_decode_image_progressive_mode_get()) {
		parser->image_type = TK_IMAGE_TYPE_PROGRESSIVE_JPEG;
	}
	// gplib jpeg
	//parser->orientation = jpeg_decode_image_orientation_get();
	if (parser->orientation<1 || parser->orientation>8) {
		parser->orientation = 1;
	}
	jpeg_decode_date_time_get(parser->date_time_ptr);
	parser->parse_status = STATUS_OK;

	return STATUS_OK;
}

void jpeg_memory_allocate(INT32U fifo)
{
	INT32U jpeg_output_y_size=0;
	INT32U jpeg_output_cb_cr_size=0;
	INT16U cbcr_shift;

	if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420) {
		cbcr_shift = 2;
		//factor = 15;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411) {
		cbcr_shift = 2;
		//factor = 15;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV422) {
		cbcr_shift = 1;
		//factor = 20;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV422V) {
		cbcr_shift = 1;
		//factor = 20;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV444) {
		cbcr_shift = 0;
		//factor = 30;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_GRAYSCALE) {
		cbcr_shift = 32;
		//factor = 10;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411V) {
		cbcr_shift = 2;
		//factor = 15;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420H2) {
		cbcr_shift = 1;
		//factor = 15;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420V2) {
		cbcr_shift = 1;
		//factor = 15;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411H2) {
		cbcr_shift = 1;
		//factor = 15;
	} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411V2) {
		cbcr_shift = 1;
		//factor = 15;
	} else {
		jpeg_out_y = 0;
		return;
	}

	if (fifo) {
		#if 0
		jpeg_output_y_size = jpeg_extend_width*fifo*2;
		jpeg_output_cb_cr_size = jpeg_output_y_size >> cbcr_shift;
		#else
		switch(jpeg_image_yuv_mode)
		{
		case C_JPG_CTRL_YUV444:
		case C_JPG_CTRL_YUV422:
		case C_JPG_CTRL_YUV422V:
		case C_JPG_CTRL_YUV420:
		case C_JPG_CTRL_YUV411:
			jpeg_output_y_size = jpeg_extend_width*fifo*2*2; //YUYV
			jpeg_output_cb_cr_size = 0;
			size_temp = jpeg_extend_width*fifo*2;
			break;

		case C_JPG_CTRL_GRAYSCALE:
		case C_JPG_CTRL_YUV411V:
		case C_JPG_CTRL_YUV420H2:
		case C_JPG_CTRL_YUV420V2:
		case C_JPG_CTRL_YUV411H2:
		case C_JPG_CTRL_YUV411V2:
			jpeg_output_y_size = jpeg_extend_width*fifo*2;
			jpeg_output_cb_cr_size = jpeg_output_y_size >> cbcr_shift;
		}
		#endif
	} else {
		jpeg_output_y_size = jpeg_extend_width * jpeg_extend_height;
		jpeg_output_cb_cr_size = jpeg_output_y_size >> cbcr_shift;
	}

  	jpeg_out_y = (INT32U) gp_malloc_align(jpeg_output_y_size, 32);
  	if (!jpeg_out_y) {
  		return;
  	}

  	if(jpeg_output_cb_cr_size) {
  		jpeg_out_cb = (INT32U) gp_malloc_align(jpeg_output_cb_cr_size, 8);
  		if (!jpeg_out_cb) {
  			gp_free((void *) jpeg_out_y);
  			jpeg_out_y = (INT32U) NULL;
  			return;
  		}
  		jpeg_out_cr = (INT32U) gp_malloc_align(jpeg_output_cb_cr_size, 8);
  		if (!jpeg_out_cr) {
  			gp_free((void *) jpeg_out_cb);
  			jpeg_out_cb = (INT32U) NULL;
  			gp_free((void *) jpeg_out_y);
  			jpeg_out_y = (INT32U) NULL;
  			return;
  		}
  	} else {
  		jpeg_out_cb = 0;
  		jpeg_out_cr = 0;
  	}
}

void jpeg_scaler_set_parameters(INT32U fifo)
{
	INT32U factor;
#if C_SCALER_L2_EN == 1
	switch(jpeg_image_yuv_mode)
	{
		case C_JPG_CTRL_YUV444:
		case C_JPG_CTRL_YUV422:
		case C_JPG_CTRL_YUV422V:
		case C_JPG_CTRL_YUV420:
		case C_JPG_CTRL_YUV411:
			jpeg_src.input_format = C_SCALER_CTRL_IN_YUYV;	//union mode
			break;
		case C_JPG_CTRL_GRAYSCALE:
			jpeg_src.input_format = C_SCALER_CTRL_IN_Y_ONLY;
			break;
		case C_JPG_CTRL_YUV411V:
			jpeg_src.input_format = C_SCALER_CTRL_IN_YUV411V;
			break;
		case C_JPG_CTRL_YUV420H2:
			jpeg_src.input_format = C_SCALER_CTRL_IN_YUV422V;
			break;
		case C_JPG_CTRL_YUV420V2:
			jpeg_src.input_format = C_SCALER_CTRL_IN_YUV422;
			break;
		case C_JPG_CTRL_YUV411H2:
			jpeg_src.input_format = C_SCALER_CTRL_IN_YUV422;
			break;
		case C_JPG_CTRL_YUV411V2:
			jpeg_src.input_format = C_SCALER_CTRL_IN_YUV422V;
			break;
		default:
			return;
	}
	//jpeg_src.input_format = jpeg_image_yuv_mode;
	jpeg_src.input_width = jpeg_extend_width;
	jpeg_src.input_height = jpeg_extend_height;
	jpeg_src.input_visible_width = jpeg_valid_width;
	jpeg_src.input_visible_height = jpeg_valid_height;

	jpeg_src.input_x_offset = 0;
	jpeg_src.input_y_offset = 0;
	jpeg_src.input_y_addr = jpeg_out_y;
	jpeg_src.input_u_addr = jpeg_out_cb;
	jpeg_src.input_v_addr = jpeg_out_cr;

	jpeg_src.input_b_y_addr = jpeg_out_y+size_temp;
	jpeg_src.input_b_u_addr = jpeg_out_cb;
	jpeg_src.input_b_v_addr = jpeg_out_cr;
	jpeg_src.output_format = jpeg_output_format;

	jpeg_src.output_width = jpeg_output_image_width;
	jpeg_src.output_height = jpeg_output_image_height;
	jpeg_src.output_buf_width = jpeg_output_buffer_width;
	jpeg_src.output_buf_height = jpeg_output_buffer_height;
	jpeg_src.callback = NULL;
	jpeg_src.output_x_offset = 0;

	if (jpeg_output_buffer_pointer) {
	    scaler_out_buffer = jpeg_output_buffer_pointer;
	} else {
	    scaler_out_buffer = (INT32U) gp_malloc((jpeg_output_buffer_width*jpeg_output_buffer_height)<<1);
	    if (!scaler_out_buffer) {
		    return;
		}
	}

	jpeg_src.output_y_addr = scaler_out_buffer;
	jpeg_src.output_u_addr = 0;
	jpeg_src.output_v_addr = 0;
	jpeg_src.fifo_mode = fifo;
	jpeg_src.scale_mode = C_SCALER_BY_RATIO;
	jpeg_src.digizoom_m = 10;
	jpeg_src.digizoom_n = 10;
	jpeg_para.yuv_type = C_SCALER_CTRL_TYPE_YCBCR;
	jpeg_para.boundary_mode = 1;
	jpeg_para.boundary_color = out_of_boundary_color;

#else
	drv_l1_scaler_input_pixels_set(C_USE_SCALER_NUMBER,jpeg_extend_width, jpeg_extend_height);
	drv_l1_scaler_input_visible_pixels_set(C_USE_SCALER_NUMBER,jpeg_valid_width, jpeg_valid_height);
	if (jpeg_output_image_width && jpeg_output_image_height) {
	    drv_l1_scaler_output_pixels_set(C_USE_SCALER_NUMBER,(jpeg_valid_width<<16)/jpeg_output_image_width, (jpeg_valid_height<<16)/jpeg_output_image_height, jpeg_output_buffer_width, jpeg_output_buffer_height);
	} else {
	    if (!jpeg_output_image_width) {
	        jpeg_output_image_width = jpeg_output_buffer_width;
	    }
	    if (!jpeg_output_image_height) {
	        jpeg_output_image_height = jpeg_output_buffer_height;
	    }
    	if (jpeg_output_ratio == 0x0) {      // Fit to output buffer width and height
      		drv_l1_scaler_output_pixels_set(C_USE_SCALER_NUMBER,(jpeg_valid_width<<16)/jpeg_output_image_width, (jpeg_valid_height<<16)/jpeg_output_image_height, jpeg_output_buffer_width, jpeg_output_buffer_height);
    	} else if (jpeg_output_ratio==2 && jpeg_valid_width<=jpeg_output_image_width && jpeg_valid_height<=jpeg_output_image_height) {
    		drv_l1_scaler_output_pixels_set(C_USE_SCALER_NUMBER,1<<16, 1<<16, jpeg_output_buffer_width, jpeg_output_buffer_height);
    		jpeg_output_image_width = jpeg_valid_width;
    		jpeg_output_image_height = jpeg_output_buffer_height;
    	} else {						// Fit to output buffer width or height
      		if (jpeg_output_image_height*jpeg_valid_width > jpeg_output_image_width*jpeg_valid_height) {
      			factor = (jpeg_valid_width<<16)/jpeg_output_image_width;
      			jpeg_output_image_height = (jpeg_valid_height<<16)/factor;
      		} else {
      			factor = (jpeg_valid_height<<16)/jpeg_output_image_height;
      			jpeg_output_image_width = (jpeg_valid_width<<16)/factor;
      		}
      		drv_l1_scaler_output_pixels_set(C_USE_SCALER_NUMBER,factor, factor, jpeg_output_buffer_width, jpeg_output_buffer_height);
      	}
    }

	drv_l1_scaler_input_A_addr_set(C_USE_SCALER_NUMBER,jpeg_out_y, jpeg_out_cb, jpeg_out_cr);
	drv_l1_scaler_input_B_addr_set(C_USE_SCALER_NUMBER,(jpeg_out_y + size_temp) , jpeg_out_cb, jpeg_out_cr);

	if (jpeg_output_buffer_pointer) {
	    scaler_out_buffer = jpeg_output_buffer_pointer;
	} else {
	    scaler_out_buffer = (INT32U) gp_malloc((jpeg_output_buffer_width*jpeg_output_buffer_height)<<1);
	    if (!scaler_out_buffer) {
		    return;
		}
	}

   	drv_l1_scaler_output_addr_set(C_USE_SCALER_NUMBER,scaler_out_buffer, (INT32U)NULL, (INT32U)NULL);
   	drv_l1_scaler_fifo_line_set(C_USE_SCALER_NUMBER,fifo);
	drv_l1_scaler_YUV_type_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_TYPE_YCBCR);
	switch(jpeg_image_yuv_mode)
	{
		case C_JPG_CTRL_YUV444:
		case C_JPG_CTRL_YUV422:
		case C_JPG_CTRL_YUV422V:
		case C_JPG_CTRL_YUV420:
		case C_JPG_CTRL_YUV411:
			drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_IN_YUYV);	//union mode
			break;
		case C_JPG_CTRL_GRAYSCALE:
			drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_IN_Y_ONLY);
			break;
		case C_JPG_CTRL_YUV411V:
			drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_IN_YUV411V);
			break;
		case C_JPG_CTRL_YUV420H2:
			drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_IN_YUV422V);
			break;
		case C_JPG_CTRL_YUV420V2:
			drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_IN_YUV422);
			break;
		case C_JPG_CTRL_YUV411H2:
			drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_IN_YUV422);
			break;
		case C_JPG_CTRL_YUV411V2:
			drv_l1_scaler_input_format_set(C_USE_SCALER_NUMBER,C_SCALER_CTRL_IN_YUV422V);
			break;
		default:
			return;
	}
	drv_l1_scaler_output_format_set(C_USE_SCALER_NUMBER,jpeg_output_format);
	drv_l1_scaler_out_of_boundary_mode_set(C_USE_SCALER_NUMBER,1);			// Use Use color defined in scaler_out_of_boundary_color_set()
	drv_l1_scaler_out_of_boundary_color_set(C_USE_SCALER_NUMBER,out_of_boundary_color);
#endif
}

void jpeg_pre_scaledown_set(void)
{
	switch(jpeg_image_yuv_mode)
	{
	case C_JPG_CTRL_YUV444:
	case C_JPG_CTRL_YUV422:
	case C_JPG_CTRL_YUV422V:
	case C_JPG_CTRL_YUV420:
	case C_JPG_CTRL_YUV411:
        drv_l1_jpeg_using_union_mode_enable();
		if((jpeg_output_image_width <= (jpeg_valid_width>>2)) &&
			(jpeg_output_image_height <= (jpeg_valid_height>>2))) {
			//DBG_PRINT("Jpeg decode union + 1/4\r\n");
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV4);// Use 1/4 scale-down mode
			jpeg_valid_width >>= 2;
			jpeg_valid_height >>= 2;
			jpeg_extend_width >>= 2;
			jpeg_extend_height >>= 2;
		} else if((jpeg_output_image_width <= (jpeg_valid_width>>1)) &&
			(jpeg_output_image_height <= (jpeg_valid_height>>1))) {
			//DBG_PRINT("Jpeg decode union + 1/2\r\n");
			//drv_l1_jpeg_level2_scaledown_mode_enable();// Use 1/2 scale-down mode
			drv_l1_jpeg_decode_scale_down_set(ENUM_JPG_DIV2);// Use 1/2 scale-down mode
			jpeg_valid_width >>= 1;
			jpeg_valid_height >>= 1;
			jpeg_extend_width >>= 1;
			jpeg_extend_height >>= 1;
		}
		break;

	case C_JPG_CTRL_GRAYSCALE:
	case C_JPG_CTRL_YUV411V:
	case C_JPG_CTRL_YUV420H2:
	case C_JPG_CTRL_YUV420V2:
	case C_JPG_CTRL_YUV411H2:
	case C_JPG_CTRL_YUV411V2:
	drv_l1_jpeg_using_union_mode_disable();
		// drv jpeg
		//jpeg_using_scaler_mode_disable();
		if((jpeg_output_image_width <= (jpeg_valid_width>>1)) &&
			(jpeg_output_image_height <= (jpeg_valid_height>>1))) {
			//DBG_PRINT("Jpeg decode separate + 1/2\r\n");
			jpeg_decode_level2_scaledown_enable();	// Use 1/2 scale-down mode
			jpeg_valid_width >>= 1;
			jpeg_valid_height >>= 1;
			jpeg_extend_width >>= 1;
			jpeg_extend_height >>= 1;
		}
		break;
	}

}

INT32S jpeg_file_decode_and_scale(IMAGE_DECODE_STRUCT *img_decode_struct, INT32U clip_xy, INT32U clip_size)
{
	INT32U cmd_id;
   	INT32S parse_status;
	INT8U *p_vlc;
	INT32U header_len;
  	INT32U fly_len;
  	INT32S jpeg_status;
  	INT32S scaler_status=0;
	INT8U scaler_done;
	TK_FILE_SERVICE_STRUCT fs_cmd_a, fs_cmd_b;
	INT32S read_cnt;
	INT8U error, fifo_a;
  #if (_OPERATING_SYSTEM == _OS_FREERTOS)
	osEvent result;
  #endif
  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
  	INT8U jpeg_need_invert = 1;
  #endif

	if (!img_decode_struct) {
		return STATUS_FAIL;
	}
	if (!image_task_fs_queue_a || !image_task_fs_queue_b) {
		img_decode_struct->decode_status = STATUS_FAIL;

		return STATUS_FAIL;
	}

	cmd_id = img_decode_struct->cmd_id;
    jpeg_output_format = img_decode_struct->output_format;
    jpeg_output_ratio = img_decode_struct->output_ratio;
    jpeg_output_buffer_width = img_decode_struct->output_buffer_width;
    jpeg_output_buffer_height = img_decode_struct->output_buffer_height;
    jpeg_output_image_width = img_decode_struct->output_image_width;
    jpeg_output_image_height = img_decode_struct->output_image_height;
    out_of_boundary_color = img_decode_struct->out_of_boundary_color;
    jpeg_output_buffer_pointer = img_decode_struct->output_buffer_pointer;

	// Setup a command that will be used to read data into fifo A
	fs_cmd_a.fd = (INT16S) img_decode_struct->image_source;
	fs_cmd_a.buf_addr = (INT32U) gp_iram_malloc_align(C_JPEG_READ_FILE_BUFFER_SIZE, 16);
	if (!fs_cmd_a.buf_addr) {
	  	fs_cmd_a.buf_addr = (INT32U) gp_malloc_align(C_JPEG_READ_FILE_BUFFER_SIZE, 16);

  		if (!fs_cmd_a.buf_addr) {
        	DBG_PRINT("jpeg_file_decode_and_scale() failed to allocate fs_cmd_a.buf_addr");
  			img_decode_struct->decode_status = -2;

			return -2;
		}
  	}
	fs_cmd_a.buf_size = C_JPEG_READ_FILE_BUFFER_SIZE;
	fs_cmd_a.result_queue = image_task_fs_queue_a;

	// Setup a command that will be used to read data into fifo B
	fs_cmd_b.fd = (INT16S) img_decode_struct->image_source;
	fs_cmd_b.buf_addr = (INT32U) gp_iram_malloc_align(C_JPEG_READ_FILE_BUFFER_SIZE, 16);
	if (!fs_cmd_b.buf_addr) {
	  	fs_cmd_b.buf_addr = (INT32U) gp_malloc_align(C_JPEG_READ_FILE_BUFFER_SIZE, 16);

	  	if (!fs_cmd_b.buf_addr) {
	  		DBG_PRINT("jpeg_file_decode_and_scale() failed to allocate fs_cmd_b.buf_addr");
	  		img_decode_struct->decode_status = -2;
	  		gp_free((void *) fs_cmd_a.buf_addr);

			return -2;
	  	}
	}
	fs_cmd_b.buf_size = C_JPEG_READ_FILE_BUFFER_SIZE;
	fs_cmd_b.result_queue = image_task_fs_queue_b;

	fifo_a = 1;				// We will read data to FIFO A first and then FIFO B
	header_len = 0;

	// Seek file pointer to begin
	lseek((INT16S) img_decode_struct->image_source, 0, SEEK_SET);

	// Initiate software header parser and hardware engine
	jpeg_decode_init();
   	do {
		// Send this command to file server task
		msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_a, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		read_cnt = (INT32S) OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
		if ((error!=OS_NO_ERR) || (read_cnt<=0)) {
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		result = osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
		read_cnt = (INT32S)result.value.v;
		if ((result.status!=osEventMessage) || (read_cnt<=0)) {
		#endif
			img_decode_struct->decode_status = STATUS_FAIL;
			gp_free((void *) fs_cmd_b.buf_addr);
			gp_free((void *) fs_cmd_a.buf_addr);

			return STATUS_FAIL;
	  	}

		// Check whether we have to break decoding this image
		if (image_task_handle_remove_request(cmd_id) > 0) {
			img_decode_struct->decode_status = STATUS_FAIL;
			gp_free((void *) fs_cmd_b.buf_addr);
			gp_free((void *) fs_cmd_a.buf_addr);

			return STATUS_FAIL;
		}
	  #if 0
		  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
			if (jpeg_need_invert) {
				INT8U *ptr8;

				ptr8 = (INT8U *) fs_cmd_a.buf_addr;
				if (*ptr8==0x00 && *(ptr8+1)==0x27) {
					INT32U *ptr32;
					INT8U i;

					ptr32 = (INT32U *) fs_cmd_a.buf_addr;
					for (i=0; i<128; i++) {
						*ptr32++ ^= 0xFFFFFFFF;
					}
				}
			}
			jpeg_need_invert = 0;
		  #endif
	  #endif
	  	parse_status = jpeg_decode_parse_header((INT8U *) fs_cmd_a.buf_addr, read_cnt);

   		if (parse_status == JPEG_PARSE_NOT_DONE) {
   			header_len += read_cnt;
   		}
	} while (parse_status == JPEG_PARSE_NOT_DONE) ;
	if (parse_status != JPEG_PARSE_OK) {
		DBG_PRINT("Parse header failed. Skip this file\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
		gp_free((void *) fs_cmd_a.buf_addr);
		gp_free((void *) fs_cmd_b.buf_addr);

	  	return STATUS_FAIL;
	}

	jpeg_valid_width = jpeg_decode_image_width_get();
	jpeg_valid_height = jpeg_decode_image_height_get();
	jpeg_image_yuv_mode = jpeg_decode_image_yuv_mode_get();
	jpeg_extend_width = jpeg_decode_image_extended_width_get();
	jpeg_extend_height = jpeg_decode_image_extended_height_get();
	if (!jpeg_extend_width || !jpeg_extend_height) {
		DBG_PRINT("JPEG width or height is 0. Skip this file\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
		gp_free((void *) fs_cmd_a.buf_addr);
		gp_free((void *) fs_cmd_b.buf_addr);

	  	return STATUS_FAIL;
	}

#if (defined LIMIT_IMAGE_WIDTH_HEIGHT) && (LIMIT_IMAGE_WIDTH_HEIGHT == CUSTOM_ON)
	if(jpeg_decode_image_progressive_mode_get())
	{
		// is progressive jpeg
		if(jpeg_valid_width * jpeg_valid_height > IMAGE_WIDTH_HEIGHT_LIMITATION)
		{
			DBG_PRINT("Do not support this progressive jpeg!\r\n");
			img_decode_struct->decode_status = STATUS_FAIL;
			gp_free((void *) fs_cmd_a.buf_addr);
			gp_free((void *) fs_cmd_b.buf_addr);

		  	return STATUS_FAIL;
		}
	}
#endif

	thumbnail_exist = jpeg_decode_thumbnail_exist_get();		// 0=No thumbnail image, 1=Thumbnail image exists
	thumbnail_width = jpeg_decode_thumbnail_width_get();
	thumbnail_height = jpeg_decode_thumbnail_height_get();
	thumbnail_yuv_mode = jpeg_decode_thumbnail_yuv_mode_get();

	// Check whether thumbnail exists and satisfies our requirement
	if (thumbnail_exist &&
		(jpeg_output_buffer_width <= thumbnail_width+(thumbnail_width>>2)) &&
		(jpeg_output_buffer_height <= thumbnail_height+(thumbnail_height>>2))) {

		header_len = 0;
		lseek((INT16S) img_decode_struct->image_source, 0, SEEK_SET);  	// Get jpeg file header from file system again
	  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
		jpeg_need_invert = 1;
	  #endif

		jpeg_decode_init();
		jpeg_decode_thumbnail_image_enable();					// This time, parse and decode thumbnail image
	   	do {
			// Send this command to file server task
			msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_a, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
			#if (_OPERATING_SYSTEM == _OS_UCOS2)
			read_cnt = (INT32S) OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
			if ((error!=OS_NO_ERR) || (read_cnt<=0)) {
			#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
			result = osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
			read_cnt = (INT32S)result.value.v;
			if ((result.status!=osEventMessage) || (read_cnt<=0)) {
			#endif
				img_decode_struct->decode_status = STATUS_FAIL;
				gp_free((void *) fs_cmd_b.buf_addr);
				gp_free((void *) fs_cmd_a.buf_addr);

				return STATUS_FAIL;
		  	}

			// Check whether we have to break decoding this image
			if (image_task_handle_remove_request(cmd_id) > 0) {
				img_decode_struct->decode_status = STATUS_FAIL;
				gp_free((void *) fs_cmd_b.buf_addr);
				gp_free((void *) fs_cmd_a.buf_addr);

				return STATUS_FAIL;
			}

		  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
			if (jpeg_need_invert) {
				INT8U *ptr8;

				ptr8 = (INT8U *) fs_cmd_a.buf_addr;
				if (*ptr8==0x00 && *(ptr8+1)==0x27) {
					INT32U *ptr32;
					INT8U i;

					ptr32 = (INT32U *) fs_cmd_a.buf_addr;
					for (i=0; i<128; i++) {
						*ptr32++ ^= 0xFFFFFFFF;
					}
				}
			}
			jpeg_need_invert = 0;
		  #endif

		  	parse_status = jpeg_decode_parse_header((INT8U *) fs_cmd_a.buf_addr, read_cnt);

	   		if (parse_status == JPEG_PARSE_NOT_DONE) {
	   			header_len += read_cnt;
	   		}
		} while (parse_status == JPEG_PARSE_NOT_DONE) ;
		if (parse_status != JPEG_PARSE_OK) {
			DBG_PRINT("Parse header failed. Skip this file\r\n");
			img_decode_struct->decode_status = STATUS_FAIL;
			gp_free((void *) fs_cmd_a.buf_addr);
			gp_free((void *) fs_cmd_b.buf_addr);

		  	return STATUS_FAIL;
		}
	    jpeg_valid_width = thumbnail_width;
		jpeg_valid_height = thumbnail_height;
		jpeg_image_yuv_mode = thumbnail_yuv_mode;
		jpeg_extend_width = jpeg_decode_thumbnail_extended_width_get();
		jpeg_extend_height = jpeg_decode_thumbnail_extended_height_get();

	} else if (jpeg_decode_image_progressive_mode_get()) {
		gp_free((void *) fs_cmd_a.buf_addr);
		gp_free((void *) fs_cmd_b.buf_addr);

	  #if GPLIB_PROGRESSIVE_JPEG_EN == 1
		jpeg_status = jpeg_decode_progressive(img_decode_struct);
	  #else
	  	jpeg_status = STATUS_FAIL;
	  #endif
		img_decode_struct->decode_status = jpeg_status;

		return jpeg_status;

	} else if (clip_size) {
		INT16U clip_x, clip_y, clip_width, clip_height;
		INT16U align_x, align_y;

		// Initiate align_x and align_y according to YUV mode
		if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV422) {
			align_x = 0xF;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420) {
			align_x = 0xF;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV422V) {
			align_x = 0x7;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV444) {
			align_x = 0x7;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_GRAYSCALE) {
			align_x = 0x7;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411) {
			align_x = 0x1F;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411V) {
			align_x = 0x7;
			align_y = 0x1F;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420H2) {
			align_x = 0xF;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420V2) {
			align_x = 0xF;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411H2) {
			align_x = 0x1F;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411V2) {
			align_x = 0x7;
			align_y = 0x1F;
		} else {
			align_x = 0x7;
			align_y = 0x7;
		}

		// Get input clipping start position and width/height
		clip_x = clip_xy >> 16;
		clip_y = clip_xy & 0xFFFF;
		clip_width = clip_size >> 16;
		clip_height = clip_size & 0xFFFF;
		// Make sure start position and width/height align to JPEG YUV mode
		if (clip_x >= jpeg_valid_width) {
			clip_x = 0;
		} else {
			clip_x = (clip_x + align_x) & (~align_x);
		}
		if (clip_y >= jpeg_valid_height) {
			clip_y = 0;
		} else {
			clip_y = (clip_y + align_y) & (~align_y);
		}
		if (!clip_width) {
			clip_width = align_x + 1;
		} else {
			clip_width = (clip_width + align_x) & (~align_x);
		}
		if (!clip_height) {
			clip_height = align_y + 1;
		} else {
			clip_height = (clip_height + align_y) & (~align_y);
		}

		// Check valid clipping range
		if (clip_x+clip_width > jpeg_extend_width) {
			clip_width = jpeg_extend_width - clip_x;
		}
		if (clip_y+clip_height > jpeg_extend_height) {
			clip_height = jpeg_extend_height - clip_y;
		}
		if (clip_x+clip_width > jpeg_valid_width) {
			jpeg_valid_width -= clip_x;
		} else {
			jpeg_valid_width = clip_width;
		}
		if (clip_y+clip_height > jpeg_valid_height) {
			jpeg_valid_height -= clip_y;
		} else {
			jpeg_valid_height = clip_height;
		}
		jpeg_extend_width = clip_width;
		jpeg_extend_height = clip_height;
		jpeg_decode_clipping_range_set(clip_x, clip_y, clip_width, clip_height);
		jpeg_decode_clipping_mode_enable();

	}

	// Send a command to read fifo_b
	msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_b, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
	fifo_a ^= 0x1;

	// Set Jpeg H/W pre scaledown.
	jpeg_pre_scaledown_set();

	jpeg_memory_allocate(jpeg_fifo_line_num);
	if (!jpeg_out_y) {
		DBG_PRINT("Failed to allocate memory in jpeg_memory_allocate()\r\n");
		img_decode_struct->decode_status = -2;
		if (fifo_a) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
            #endif
		} else {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
            #endif
		}
		gp_free((void *) fs_cmd_a.buf_addr);
		gp_free((void *) fs_cmd_b.buf_addr);

		return -2;
	}

	if (jpeg_decode_output_set(jpeg_out_y, jpeg_out_cb, jpeg_out_cr, jpeg_fifo_register)) {
		DBG_PRINT("Failed to call jpeg_decode_output_set()\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
	  	if (fifo_a) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
            #endif
		} else {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
			OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
            #endif
		}
	  	gp_free((void *) fs_cmd_a.buf_addr);
		gp_free((void *) fs_cmd_b.buf_addr);
	  	gp_free((void *) jpeg_out_y);
	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
	  		gp_free((void *) jpeg_out_cb);
	  		gp_free((void *) jpeg_out_cr);
	  	}

		return STATUS_FAIL;
	}

	p_vlc = jpeg_decode_image_vlc_addr_get();

	if ((INT32U) p_vlc == (fs_cmd_a.buf_addr + read_cnt)) {
		msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_a, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
		fifo_a ^= 0x1;
		#if (_OPERATING_SYSTEM == _OS_UCOS2)
		read_cnt = (INT32S) OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
		if ((error!=OS_NO_ERR) || (read_cnt<=0)) {
		#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
		result = osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
		read_cnt = (INT32S)result.value.v;
		if ((result.status!=osEventMessage) || (read_cnt<=0)) {
		#endif
			DBG_PRINT("Failed to read JPEG file\r\n");
			img_decode_struct->decode_status = STATUS_FAIL;
		  	if (fifo_a) {
                #if (_OPERATING_SYSTEM == _OS_UCOS2)
                OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
                #endif

			} else {
                #if (_OPERATING_SYSTEM == _OS_UCOS2)
                OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
                #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
                osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
                #endif
			}
		  	gp_free((void *) fs_cmd_a.buf_addr);
    		gp_free((void *) fs_cmd_b.buf_addr);
    	  	gp_free((void *) jpeg_out_y);
    	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
    	  		gp_free((void *) jpeg_out_cb);
    	  		gp_free((void *) jpeg_out_cr);
    	  	}

	  		return STATUS_FAIL;
	  	}
		p_vlc = (INT8U *) fs_cmd_b.buf_addr;
		fly_len = (fs_cmd_b.buf_addr+read_cnt) - ((INT32U) p_vlc);
		jpeg_decode_vlc_maximum_length_set(img_decode_struct->source_size - header_len - C_JPEG_READ_FILE_BUFFER_SIZE);	// Set maximum VLC length to prevent JPEG from hangging too long
	} else {
		fly_len = (fs_cmd_a.buf_addr+read_cnt) - ((INT32U) p_vlc);
		jpeg_decode_vlc_maximum_length_set(img_decode_struct->source_size - header_len - ((INT32U) p_vlc - (INT32U) fs_cmd_a.buf_addr));	// Set maximum VLC length to prevent JPEG from hangging too long
	}

   	// Now start JPEG decoding on the fly
	if (jpeg_decode_on_the_fly_start(p_vlc, fly_len)) {
		DBG_PRINT("Failed to call jpeg_decode_on_the_fly_start()\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
	  	if (fifo_a) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
            OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
            #endif

		} else {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
            OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
            #endif
		}
	  	gp_free((void *) fs_cmd_a.buf_addr);
		gp_free((void *) fs_cmd_b.buf_addr);
	  	gp_free((void *) jpeg_out_y);
	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
	  		gp_free((void *) jpeg_out_cb);
	  		gp_free((void *) jpeg_out_cr);
	  	}

		return STATUS_FAIL;
	}

  	// Initiate Scaler
#if C_SCALER_L2_EN == 1
    drv_l2_scaler_init();
#else
  	drv_l1_scaler_init(C_USE_SCALER_NUMBER);
#endif
  	scaler_done = 0;

	// Setup Scaler
	jpeg_scaler_set_parameters(scaler_fifo_register);
  	if (!scaler_out_buffer) {
		jpeg_decode_stop();
		DBG_PRINT("Failed to allocate scaler_out_buffer\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
	  	if (fifo_a) {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
            OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
            #endif

		} else {
            #if (_OPERATING_SYSTEM == _OS_UCOS2)
            OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
            #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
            osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
            #endif
		}
	  	gp_free((void *) fs_cmd_a.buf_addr);
		gp_free((void *) fs_cmd_b.buf_addr);
	  	gp_free((void *) jpeg_out_y);
	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
	  		gp_free((void *) jpeg_out_cb);
	  		gp_free((void *) jpeg_out_cr);
	  	}

		return STATUS_FAIL;
  	}
#if C_SCALER_L2_EN == 1
	scaler_status = C_SCALER_STATUS_STOP;
  	while (1) {
  		jpeg_status = jpeg_decode_status_query(1);

		if (jpeg_status & C_JPG_STATUS_DECODE_DONE) {
		  	// Wait until scaler finish its job
		  	while (!scaler_done) {
		  		if (scaler_status == C_SCALER_STATUS_STOP) {
					DBG_PRINT("bb\r\n");
					scaler_status = drv_l2_scaler_trigger(C_USE_SCALER_NUMBER, 1, &jpeg_src, &jpeg_para);
				} else if (scaler_status & C_SCALER_STATUS_DONE) {
					break;
				} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
		  			scaler_status = drv_l2_scaler_retrigger(C_USE_SCALER_NUMBER, &jpeg_src);
		  		} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}

			  	if(scaler_status == C_SCALER_STATUS_STOP)
			  	{
					scaler_status = C_SCALER_STATUS_DONE;
			  	}
		  	}
			break;
		}		// if (jpeg_status & C_JPG_STATUS_DECODE_DONE)

  		if (jpeg_status & C_JPG_STATUS_INPUT_EMPTY) {
			if (fifo_a) {
				msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_b, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
				fifo_a ^= 0x1;
	   			#if (_OPERATING_SYSTEM == _OS_UCOS2)
	   			read_cnt = (INT32S) OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
   				if ((error!=OS_NO_ERR) || (read_cnt<0)) {
   				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
   				result = osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
   				read_cnt = (INT32S)result.value.v;
                if ((result.status!=osEventMessage) || (read_cnt<0)) {
   				#endif
	   				DBG_PRINT("Failed to read more data from JPEG file\r\n");
		  			break;
			  	}

				// Now restart JPEG decoding on the fly
		  		if (jpeg_decode_on_the_fly_start((INT8U *) fs_cmd_a.buf_addr, fs_cmd_a.buf_size)) {
		  			DBG_PRINT("Failed to call jpeg_decode_on_the_fly_start()\r\n");
		  			break;
		  		}
			} else {
				msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_a, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
				fifo_a ^= 0x1;
	   			#if (_OPERATING_SYSTEM == _OS_UCOS2)
	   			read_cnt = (INT32S) OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
   				if ((error!=OS_NO_ERR) || (read_cnt<0)) {
   				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
   				result = osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
   				read_cnt = (INT32S)result.value.v;
   				if ((result.status != osEventMessage) || (read_cnt<0)) {
   				#endif
	   				DBG_PRINT("Failed to read more data from JPEG file\r\n");
		  			break;
			  	}

				// Now restart JPEG decoding on the fly
		  		if (jpeg_decode_on_the_fly_start((INT8U *) fs_cmd_b.buf_addr, fs_cmd_b.buf_size)) {
		  			DBG_PRINT("Failed to call jpeg_decode_on_the_fly_start()\r\n");
		  			break;
		  		}
			}
		}		// if (jpeg_status & C_JPG_STATUS_INPUT_EMPTY)

		if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL) {
		  	// Start scaler to handle the full output FIFO now
		  	if (!scaler_done) {
			  	if (scaler_status == C_SCALER_STATUS_STOP) {
					scaler_status = drv_l2_scaler_trigger(C_USE_SCALER_NUMBER, 1, &jpeg_src, &jpeg_para);
			  	} else if (scaler_status & C_SCALER_STATUS_DONE) {
			  		// Scaler might finish its job before JPEG does when image is zoomed in.
			  		scaler_done = 1;
			  	} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
			  		scaler_status = drv_l2_scaler_retrigger(C_USE_SCALER_NUMBER, &jpeg_src);
			  	} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}

			  	if(scaler_status == C_SCALER_STATUS_STOP)
			  	{
					scaler_status = C_SCALER_STATUS_DONE;
			  	}
			}

	  		// Now restart JPEG to output to next FIFO
	  		if (jpeg_decode_output_restart()) {
	  			DBG_PRINT("Failed to call jpeg_decode_output_restart()\r\n");
	  			break;
	  		}
		}		// if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL)

		if (jpeg_status & C_JPG_STATUS_STOP) {
			DBG_PRINT("JPEG is not started!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_TIMEOUT) {
			DBG_PRINT("JPEG execution timeout-3!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_INIT_ERR) {
			DBG_PRINT("JPEG init error!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_VLC_DONE) {
			DBG_PRINT("JPEG Restart marker number is incorrect!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_MARKER_ERR) {
			DBG_PRINT("JPEG Restart marker sequence error!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		// Check whether we have to break decoding this image
		if (image_task_handle_remove_request(cmd_id) > 0) {
			break;
		}
  	}
	jpeg_decode_stop();
	if( (scaler_status >= 0) &&(scaler_status != C_SCALER_STATUS_STOP))
		drv_l2_scaler_stop(C_USE_SCALER_NUMBER);

#else
  	while (1) {
  		jpeg_status = jpeg_decode_status_query(1);

		if (jpeg_status & C_JPG_STATUS_DECODE_DONE) {
		  	// Wait until scaler finish its job
		  	while (!scaler_done) {
		  		scaler_status = drv_l1_scaler_wait_idle(C_USE_SCALER_NUMBER);
		  		if (scaler_status == C_SCALER_STATUS_STOP) {
					if (drv_l1_scaler_start(C_USE_SCALER_NUMBER,ENABLE)) {
						DBG_PRINT("Failed to call scaler_start\r\n");
						break;
					}
				} else if (scaler_status & C_SCALER_STATUS_DONE) {
					break;
				} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
		  			if (drv_l1_scaler_restart(C_USE_SCALER_NUMBER)) {
						DBG_PRINT("Failed to call scaler_restart\r\n");
						break;
					}
		  		} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}
		  	}
			break;
		}		// if (jpeg_status & C_JPG_STATUS_DECODE_DONE)

  		if (jpeg_status & C_JPG_STATUS_INPUT_EMPTY) {
			if (fifo_a) {
				msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_b, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
				fifo_a ^= 0x1;
	   			#if (_OPERATING_SYSTEM == _OS_UCOS2)
	   			read_cnt = (INT32S) OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
   				if ((error!=OS_NO_ERR) || (read_cnt<0)) {
   				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
   				result = osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
   				read_cnt = (INT32S)result.value.v;
                if ((result.status!=osEventMessage) || (read_cnt<0)) {
   				#endif
	   				DBG_PRINT("Failed to read more data from JPEG file\r\n");
		  			break;
			  	}

				// Now restart JPEG decoding on the fly
		  		if (jpeg_decode_on_the_fly_start((INT8U *) fs_cmd_a.buf_addr, fs_cmd_a.buf_size)) {
		  			DBG_PRINT("Failed to call jpeg_decode_on_the_fly_start()\r\n");
		  			break;
		  		}
			} else {
				msgQSend(fs_msg_q_id, MSG_FILESRV_FS_READ, (void *) &fs_cmd_a, sizeof(TK_FILE_SERVICE_STRUCT), MSG_PRI_NORMAL);
				fifo_a ^= 0x1;
	   			#if (_OPERATING_SYSTEM == _OS_UCOS2)
	   			read_cnt = (INT32S) OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
   				if ((error!=OS_NO_ERR) || (read_cnt<0)) {
   				#elif (_OPERATING_SYSTEM == _OS_FREERTOS)
   				result = osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
   				read_cnt = (INT32S)result.value.v;
   				if ((result.status != osEventMessage) || (read_cnt<0)) {
   				#endif
	   				DBG_PRINT("Failed to read more data from JPEG file\r\n");
		  			break;
			  	}

				// Now restart JPEG decoding on the fly
		  		if (jpeg_decode_on_the_fly_start((INT8U *) fs_cmd_b.buf_addr, fs_cmd_b.buf_size)) {
		  			DBG_PRINT("Failed to call jpeg_decode_on_the_fly_start()\r\n");
		  			break;
		  		}
			}
		}		// if (jpeg_status & C_JPG_STATUS_INPUT_EMPTY)

		if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL) {
		  	// Start scaler to handle the full output FIFO now
		  	if (!scaler_done) {
			  	scaler_status = drv_l1_scaler_wait_idle(C_USE_SCALER_NUMBER);
			  	if (scaler_status == C_SCALER_STATUS_STOP) {
					if (drv_l1_scaler_start(C_USE_SCALER_NUMBER,ENABLE)) {
						DBG_PRINT("Failed to call scaler_start\r\n");
						break;
					}
			  	} else if (scaler_status & C_SCALER_STATUS_DONE) {
			  		// Scaler might finish its job before JPEG does when image is zoomed in.
			  		scaler_done = 1;
			  	} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
			  		if (drv_l1_scaler_restart(C_USE_SCALER_NUMBER)) {
						DBG_PRINT("Failed to call scaler_restart\r\n");
						break;
					}
			  	} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}
			}

	  		// Now restart JPEG to output to next FIFO
	  		if (jpeg_decode_output_restart()) {
	  			DBG_PRINT("Failed to call jpeg_decode_output_restart()\r\n");
	  			break;
	  		}
		}		// if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL)

		if (jpeg_status & C_JPG_STATUS_STOP) {
			DBG_PRINT("JPEG is not started!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_TIMEOUT) {
			DBG_PRINT("JPEG execution timeout-3!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_INIT_ERR) {
			DBG_PRINT("JPEG init error!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_VLC_DONE) {
			DBG_PRINT("JPEG Restart marker number is incorrect!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_MARKER_ERR) {
			DBG_PRINT("JPEG Restart marker sequence error!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		// Check whether we have to break decoding this image
		if (image_task_handle_remove_request(cmd_id) > 0) {
			break;
		}
  	}

	jpeg_decode_stop();

	drv_l1_scaler_stop(C_USE_SCALER_NUMBER);
#endif
  	if(jpeg_out_y) gp_free((void *) jpeg_out_y);
  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
  		if(jpeg_out_cb) gp_free((void *) jpeg_out_cb);
  		if(jpeg_out_cr) gp_free((void *) jpeg_out_cr);
  	}
	if (fifo_a) {
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPend(image_task_fs_queue_a, JPG_FS_TIMEOUT, &error);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        osMessageGet(image_task_fs_queue_a, JPG_FS_TIMEOUT);
        #endif
	} else {
        #if (_OPERATING_SYSTEM == _OS_UCOS2)
		OSQPend(image_task_fs_queue_b, JPG_FS_TIMEOUT, &error);
        #elif (_OPERATING_SYSTEM == _OS_FREERTOS)
        osMessageGet(image_task_fs_queue_b, JPG_FS_TIMEOUT);
        #endif
	}
  	gp_free((void *) fs_cmd_a.buf_addr);
	gp_free((void *) fs_cmd_b.buf_addr);

	if ((jpeg_status & C_JPG_STATUS_DECODE_DONE) && (scaler_status & C_SCALER_STATUS_DONE)) {
	    if (!jpeg_output_buffer_pointer) {
	        jpeg_output_buffer_pointer = scaler_out_buffer;
	    }
	    cache_invalid_range(scaler_out_buffer, (jpeg_output_buffer_width*jpeg_output_buffer_height)<<1);
	} else {
		img_decode_struct->decode_status = STATUS_FAIL;
    	if (!jpeg_output_buffer_pointer) {
    	    gp_free((void *) scaler_out_buffer);
    	}

        return STATUS_FAIL;
	}

    img_decode_struct->decode_status = STATUS_OK;
    if (!img_decode_struct->output_buffer_pointer && jpeg_output_buffer_pointer) {
        img_decode_struct->output_buffer_pointer = jpeg_output_buffer_pointer;
    }
    img_decode_struct->output_image_width = jpeg_output_image_width;
    img_decode_struct->output_image_height = jpeg_output_image_height;

	return STATUS_OK;
}

INT32S jpeg_buffer_decode_and_scale(IMAGE_DECODE_STRUCT *img_decode_struct, INT32U clip_xy, INT32U clip_size)
{
	INT32U cmd_id;
	INT32S parse_status;
	INT8U *p_vlc;
	INT32U left_len;
  	INT32S jpeg_status;
  	INT32S scaler_status=0;
	INT8U scaler_done;
  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
	INT8U *ptr8;
  #endif

	if (!img_decode_struct) {
		return STATUS_FAIL;
	}

	cmd_id = img_decode_struct->cmd_id;
    jpeg_output_format = img_decode_struct->output_format;
    jpeg_output_ratio = img_decode_struct->output_ratio;
    jpeg_output_buffer_width = img_decode_struct->output_buffer_width;
    jpeg_output_buffer_height = img_decode_struct->output_buffer_height;
    jpeg_output_image_width = img_decode_struct->output_image_width;
    jpeg_output_image_height = img_decode_struct->output_image_height;
    out_of_boundary_color = img_decode_struct->out_of_boundary_color;
    jpeg_output_buffer_pointer = img_decode_struct->output_buffer_pointer;

	// Initiate software header parser and hardware engine
	jpeg_decode_init();

  #if (defined GP_FILE_FORMAT_SUPPORT) && (GP_FILE_FORMAT_SUPPORT==GP_FILE_FORMAT_SET_1)
	ptr8 = (INT8U *) img_decode_struct->image_source;
	if (*ptr8==0x00 && *(ptr8+1)==0x27) {
		INT32U *ptr32;
		INT8U i;

		ptr32 = (INT32U *) img_decode_struct->image_source;
		for (i=0; i<128; i++) {
			*ptr32++ ^= 0xFFFFFFFF;
		}
	}
  #endif
	parse_status = jpeg_decode_parse_header((INT8U *) img_decode_struct->image_source, img_decode_struct->source_size);
    if (parse_status != JPEG_PARSE_OK) {
		DBG_PRINT("Parse header failed. Skip this file\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;

        return STATUS_FAIL;
	}

	jpeg_valid_width = jpeg_decode_image_width_get();
	jpeg_valid_height = jpeg_decode_image_height_get();
	jpeg_image_yuv_mode = jpeg_decode_image_yuv_mode_get();
	jpeg_extend_width = jpeg_decode_image_extended_width_get();
	jpeg_extend_height = jpeg_decode_image_extended_height_get();

	thumbnail_exist = jpeg_decode_thumbnail_exist_get();		// 0=No thumbnail image, 1=Thumbnail image exists
	thumbnail_width = jpeg_decode_thumbnail_width_get();
	thumbnail_height = jpeg_decode_thumbnail_height_get();
	thumbnail_yuv_mode = jpeg_decode_thumbnail_yuv_mode_get();

	// Check whether thumbnail exists and satisfies our requirement
	if (thumbnail_exist &&
		(jpeg_output_buffer_width <= thumbnail_width+(thumbnail_width>>2)) &&
		(jpeg_output_buffer_height <= thumbnail_height+(thumbnail_height>>2))) {

		jpeg_decode_init();
		jpeg_decode_thumbnail_image_enable();
	   	parse_status = jpeg_decode_parse_header((INT8U *) img_decode_struct->image_source, img_decode_struct->source_size);
		if (parse_status != JPEG_PARSE_OK) {
			DBG_PRINT("Parse header failed. Skip this file\r\n");
			img_decode_struct->decode_status = STATUS_FAIL;

		  	return STATUS_FAIL;
		}

	    jpeg_valid_width = thumbnail_width;
		jpeg_valid_height = thumbnail_height;
		jpeg_image_yuv_mode = thumbnail_yuv_mode;
		jpeg_extend_width = jpeg_decode_thumbnail_extended_width_get();
		jpeg_extend_height = jpeg_decode_thumbnail_extended_height_get();

	} else if (jpeg_decode_image_progressive_mode_get()) {
	  #if GPLIB_PROGRESSIVE_JPEG_EN == 1
		jpeg_status = jpeg_decode_progressive(img_decode_struct);
	  #else
	  	jpeg_status = STATUS_FAIL;
	  #endif
		img_decode_struct->decode_status = jpeg_status;

		return jpeg_status;

	} else if (clip_size) {
		INT16U clip_x, clip_y, clip_width, clip_height;
		INT16U align_x, align_y;

		// Initiate align_x and align_y according to YUV mode
		if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV422) {
			align_x = 0xF;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420) {
			align_x = 0xF;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV422V) {
			align_x = 0x7;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV444) {
			align_x = 0x7;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_GRAYSCALE) {
			align_x = 0x7;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411) {
			align_x = 0x1F;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411V) {
			align_x = 0x7;
			align_y = 0x1F;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420H2) {
			align_x = 0xF;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV420V2) {
			align_x = 0xF;
			align_y = 0xF;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411H2) {
			align_x = 0x1F;
			align_y = 0x7;
		} else if (jpeg_image_yuv_mode == C_JPG_CTRL_YUV411V2) {
			align_x = 0x7;
			align_y = 0x1F;
		} else {
			align_x = 0x7;
			align_y = 0x7;
		}

		// Get input clipping start position and width/height
		clip_x = clip_xy >> 16;
		clip_y = clip_xy & 0xFFFF;
		clip_width = clip_size >> 16;
		clip_height = clip_size & 0xFFFF;
		// Make sure start position and width/height align to JPEG YUV mode
		if (clip_x >= jpeg_valid_width) {
			clip_x = 0;
		} else {
			clip_x = (clip_x + align_x) & (~align_x);
		}
		if (clip_y >= jpeg_valid_height) {
			clip_y = 0;
		} else {
			clip_y = (clip_y + align_y) & (~align_y);
		}
		if (!clip_width) {
			clip_width = align_x + 1;
		} else {
			clip_width = (clip_width + align_x) & (~align_x);
		}
		if (!clip_height) {
			clip_height = align_y + 1;
		} else {
			clip_height = (clip_height + align_y) & (~align_y);
		}

		// Check valid clipping range
		if (clip_x+clip_width > jpeg_extend_width) {
			clip_width = jpeg_extend_width - clip_x;
		}
		if (clip_y+clip_height > jpeg_extend_height) {
			clip_height = jpeg_extend_height - clip_y;
		}
		if (clip_x+clip_width > jpeg_valid_width) {
			jpeg_valid_width -= clip_x;
		} else {
			jpeg_valid_width = clip_width;
		}
		if (clip_y+clip_height > jpeg_valid_height) {
			jpeg_valid_height -= clip_y;
		} else {
			jpeg_valid_height = clip_height;
		}
		jpeg_extend_width = clip_width;
		jpeg_extend_height = clip_height;
		jpeg_decode_clipping_range_set(clip_x, clip_y, clip_width, clip_height);
		jpeg_decode_clipping_mode_enable();

	}

	// Set Jpeg H/W pre scaledown.
	jpeg_pre_scaledown_set();

	jpeg_memory_allocate(jpeg_fifo_line_num);
	if (!jpeg_out_y) {
		DBG_PRINT("Failed to allocate memory in jpeg_memory_allocate()\r\n");
		img_decode_struct->decode_status = -2;

		return -2;
	}

	if (jpeg_decode_output_set(jpeg_out_y, jpeg_out_cb, jpeg_out_cr, jpeg_fifo_register)) {
		DBG_PRINT("Failed to call jpeg_decode_output_set()\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
	  	gp_free((void *) jpeg_out_y);
	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
	  		gp_free((void *) jpeg_out_cb);
	  		gp_free((void *) jpeg_out_cr);
	  	}

		return STATUS_FAIL;
	}

	p_vlc = jpeg_decode_image_vlc_addr_get();
  	if (((INT32U) p_vlc) >= (img_decode_struct->image_source+img_decode_struct->source_size)) {
  		DBG_PRINT("VLC address exceeds file range\r\n");
  		img_decode_struct->decode_status = STATUS_FAIL;
	  	gp_free((void *) jpeg_out_y);
	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
	  		gp_free((void *) jpeg_out_cb);
	  		gp_free((void *) jpeg_out_cr);
	  	}

        return STATUS_FAIL;
  	}
	left_len = img_decode_struct->source_size - (((INT32U) p_vlc) - img_decode_struct->image_source);
	jpeg_decode_vlc_maximum_length_set(left_len);

	// Now start JPEG decoding
	if (jpeg_decode_once_start(p_vlc, left_len)) {
		DBG_PRINT("Failed to call jpeg_decode_once_start()\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
	  		gp_free((void *) jpeg_out_cb);
	  		gp_free((void *) jpeg_out_cr);
	  	}
	  	gp_free((void *) jpeg_out_y);

        return STATUS_FAIL;
  	}

  	// Initiate Scaler
#if C_SCALER_L2_EN == 0
  	drv_l1_scaler_init(C_USE_SCALER_NUMBER);
#endif
	scaler_done = 0;
	// Setup Scaler
	jpeg_scaler_set_parameters(scaler_fifo_register);

  	if (!scaler_out_buffer) {
		jpeg_decode_stop();
		DBG_PRINT("Failed to allocate scaler_out_buffer\r\n");
		img_decode_struct->decode_status = STATUS_FAIL;
	  	gp_free((void *) jpeg_out_y);
	  	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
	  		gp_free((void *) jpeg_out_cb);
	  		gp_free((void *) jpeg_out_cr);
	  	}

		return STATUS_FAIL;
  	}
#if C_SCALER_L2_EN == 1
	scaler_status = C_SCALER_STATUS_STOP;
  	while (1) {
  		jpeg_status = jpeg_decode_status_query(1);

		if (jpeg_status & C_JPG_STATUS_DECODE_DONE) {
		  	// Wait until scaler finish its job
		  	while (!scaler_done) {
		  		if (scaler_status == C_SCALER_STATUS_STOP) {
					scaler_status = drv_l2_scaler_trigger(C_USE_SCALER_NUMBER, 1, &jpeg_src, &jpeg_para);
				} else if (scaler_status & C_SCALER_STATUS_DONE) {
					break;
				} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
		  			scaler_status = drv_l2_scaler_retrigger(C_USE_SCALER_NUMBER, &jpeg_src);
		  		} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}

			  	if(scaler_status == C_SCALER_STATUS_STOP)
			  	{
					scaler_status = C_SCALER_STATUS_DONE;
			  	}
		  	}
			break;
		}		// if (jpeg_status & C_JPG_STATUS_DECODE_DONE)

		if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL) {
		  	// Start scaler to handle the full output FIFO now
		  	if (!scaler_done) {
			  	if (scaler_status == C_SCALER_STATUS_STOP) {
					scaler_status = drv_l2_scaler_trigger(C_USE_SCALER_NUMBER, 1, &jpeg_src, &jpeg_para);
			  	} else if (scaler_status & C_SCALER_STATUS_DONE) {
			  		// Scaler might finish its job before JPEG does when image is zoomed in.
			  		scaler_done = 1;
			  	} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
			  		scaler_status = drv_l2_scaler_retrigger(C_USE_SCALER_NUMBER, &jpeg_src);
			  	} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}

			  	if(scaler_status == C_SCALER_STATUS_STOP)
			  	{
					scaler_status = C_SCALER_STATUS_DONE;
			  	}
			}

	  		// Now restart JPEG to output to next FIFO
	  		if (jpeg_decode_output_restart()) {
	  			DBG_PRINT("Failed to call jpeg_decode_output_restart()\r\n");
	  			break;
	  		}
		}		// if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL)

		if (jpeg_status & C_JPG_STATUS_STOP) {
			DBG_PRINT("JPEG is not started!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_TIMEOUT) {
			DBG_PRINT("JPEG execution timeout-3!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_INIT_ERR) {
			DBG_PRINT("JPEG init error!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_VLC_DONE) {
			DBG_PRINT("JPEG Restart marker number is incorrect!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_MARKER_ERR) {
			DBG_PRINT("JPEG Restart marker sequence error!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		// Check whether we have to break decoding this image
		if (image_task_handle_remove_request(cmd_id) > 0) {
			break;
		}
  	}
	jpeg_decode_stop();
	if( (scaler_status >= 0) && (scaler_status != C_SCALER_STATUS_STOP) )
		drv_l2_scaler_stop(C_USE_SCALER_NUMBER);
#else
	while (1) {
		jpeg_status = jpeg_decode_status_query(1);

		if (jpeg_status & C_JPG_STATUS_DECODE_DONE) {
		  	// Wait until scaler finish its job
		  	while (!scaler_done) {
		  		scaler_status = drv_l1_scaler_wait_idle(C_USE_SCALER_NUMBER);
		  		if (scaler_status == C_SCALER_STATUS_STOP) {
					if (drv_l1_scaler_start(C_USE_SCALER_NUMBER,ENABLE)) {
						DBG_PRINT("Failed to call scaler_start\r\n");
						break;
					}
				} else if (scaler_status & C_SCALER_STATUS_DONE) {
					break;
				} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
		  			if (drv_l1_scaler_restart(C_USE_SCALER_NUMBER)) {
						DBG_PRINT("Failed to call scaler_restart\r\n");
						break;
					}
		  		} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}
		  	}
			break;
		}		// if (jpeg_status & C_JPG_STATUS_DECODE_DONE)

		if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL) {
		  	// Start scaler to handle the full output FIFO now
		  	if (!scaler_done) {
			  	scaler_status = drv_l1_scaler_wait_idle(C_USE_SCALER_NUMBER);
			  	if (scaler_status == C_SCALER_STATUS_STOP) {
					if (drv_l1_scaler_start(C_USE_SCALER_NUMBER,ENABLE)) {
						DBG_PRINT("Failed to call scaler_start\r\n");
						break;
					}
			  	} else if (scaler_status & C_SCALER_STATUS_DONE) {
		  			// Scaler might finish its job before JPEG does when image is zoomed in.
		  			scaler_done = 1;
				} else if (scaler_status & (C_SCALER_STATUS_TIMEOUT|C_SCALER_STATUS_INIT_ERR)) {
					DBG_PRINT("Scaler failed to finish its job\r\n");
					break;
				} else if (scaler_status & C_SCALER_STATUS_INPUT_EMPTY) {
			  		if (drv_l1_scaler_restart(C_USE_SCALER_NUMBER)) {
						DBG_PRINT("Failed to call scaler_restart\r\n");
						break;
					}
			  	} else {
			  		DBG_PRINT("Un-handled Scaler status!\r\n");
			  		break;
			  	}
			}

			// Now restart JPEG to output to next FIFO
	  		if (jpeg_decode_output_restart()) {
	  			DBG_PRINT("Failed to call jpeg_decode_output_restart()\r\n");
	  			break;
	  		}
		}		// if (jpeg_status & C_JPG_STATUS_OUTPUT_FULL)

		if (jpeg_status & C_JPG_STATUS_STOP) {
			DBG_PRINT("JPEG is not started!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_TIMEOUT) {
			DBG_PRINT("JPEG execution timeout-4!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_INIT_ERR) {
			DBG_PRINT("JPEG init error!\r\n");
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_VLC_DONE) {
			DBG_PRINT("JPEG Restart marker number is incorrect!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		if (jpeg_status & C_JPG_STATUS_RST_MARKER_ERR) {
			DBG_PRINT("JPEG Restart marker sequence error!\r\n");
			jpeg_status = C_JPG_STATUS_DECODE_DONE;
			scaler_status = C_SCALER_STATUS_DONE;
			break;
		}
		// Check whether we have to break decoding this image
		if (image_task_handle_remove_request(cmd_id) > 0) {
			break;
		}
	}

	jpeg_decode_stop();

	drv_l1_scaler_stop(C_USE_SCALER_NUMBER);
#endif
	if(jpeg_out_y) gp_free((void *) jpeg_out_y);
	if (jpeg_image_yuv_mode != C_JPG_CTRL_GRAYSCALE) {
  		if(jpeg_out_cb) gp_free((void *) jpeg_out_cb);
  		if(jpeg_out_cr) gp_free((void *) jpeg_out_cr);
  	}

	if ((jpeg_status & C_JPG_STATUS_DECODE_DONE) && (scaler_status & C_SCALER_STATUS_DONE)) {
	    if (!jpeg_output_buffer_pointer) {
	        jpeg_output_buffer_pointer = scaler_out_buffer;
	    }
	    cache_invalid_range(scaler_out_buffer, (jpeg_output_buffer_width*jpeg_output_buffer_height)<<1);
	} else {
		img_decode_struct->decode_status = STATUS_FAIL;
    	if (!jpeg_output_buffer_pointer) {
    	    gp_free((void *) scaler_out_buffer);
    	}

        return STATUS_FAIL;
	}

    img_decode_struct->decode_status = STATUS_OK;
    if (!img_decode_struct->output_buffer_pointer && jpeg_output_buffer_pointer) {
        img_decode_struct->output_buffer_pointer = jpeg_output_buffer_pointer;
    }
    img_decode_struct->output_image_width = jpeg_output_image_width;
    img_decode_struct->output_image_height = jpeg_output_image_height;

	return STATUS_OK;
}
#endif//#if (defined APP_IMAGE_CODEC_EN ) && (APP_IMAGE_CODEC_EN == 1)
