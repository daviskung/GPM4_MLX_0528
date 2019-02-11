#ifndef __USBH_UVC_H__
#define __USBH_UVC_H__

//#define USB_DEBUG
//#define EHCI_UVC_EN
//#define SPIT_5M_CAM
#define FRAME_WIDTH                     640//1280
#define FRAME_HEIGHT                    480//720
#define UVC_JPGDEC_BUF_MQUEUE_MAX 3
#define UVC_JPGDEC_BUF_SIZE 320*1024
#define HOST_UVC_ISO_Buff_CNT 30//10
#define HOST_UVC_iTD_Table_CNT 10
#define HOST_UVC_iTD_Table_Size 64
#define HOST_UVC_Framelist_Size 1024//4096
#define iTD_Trans_Time 8
#define ISO_Max_Packet_Size 3*1024    // Multi=3
#define HOST_UVC_ISO_DATA_Buff iTD_Trans_Time*ISO_Max_Packet_Size   // for usb2.0 each iTD table can max receive 3*8 times

#define BACK_SENSOR_FRAME_RATE_5     2000000
#define BACK_SENSOR_FRAME_RATE_10    1000000
#define BACK_SENSOR_FRAME_RATE_15    666666
#define BACK_SENSOR_FRAME_RATE_20    500000
#define BACK_SENSOR_FRAME_RATE_25    0x61A80//0x07A120////400000
#define BACK_SENSOR_FRAME_RATE_30    0x51615
#define BACK_SENSOR_FRAME_RATE       BACK_SENSOR_FRAME_RATE_25

#define GUID_YUV2_0 0x32595559
#define GUID_YUV2_1 0x0000
#define GUID_YUV2_2 0x0010
#define GUID_YUV2_3 0x8000
#define GUID_YUV2_4 0x00AA00
#define GUID_YUV2_5 0x389B71

#define GUID_NV12_0 0x3231564E
#define GUID_NV12_1 0x0000
#define GUID_NV12_2 0x0010
#define GUID_NV12_3 0x8000
#define GUID_NV12_4 0x00AA00
#define GUID_NV12_5 0x389B71

/* Values for bmHeaderInfo (Video and Still Image Payload Headers, 2.4.3.3) */
#define UVC_STREAM_EOH	(1 << 7)
#define UVC_STREAM_ERR	(1 << 6)
#define UVC_STREAM_STI	(1 << 5)
#define UVC_STREAM_RES	(1 << 4)
#define UVC_STREAM_SCR	(1 << 3)
#define UVC_STREAM_PTS	(1 << 2)
#define UVC_STREAM_EOF	(1 << 1)
#define UVC_STREAM_FID	(1 << 0)

/* Video Class-Specific Request Codes, (USB_Video_Class_1.1.pdf, A.8 Video Class-Specific Request Codes) */
#define RC_UNDEFINED                               0x00
#define SET_CUR                                    0x01
#define GET_CUR                                    0x81
#define GET_MIN                                    0x82
#define GET_MAX                                    0x83
#define GET_RES                                    0x84
#define GET_LEN                                    0x85
#define GET_INFO                                   0x86
#define GET_DEF                                    0x87

/* VideoStreaming Interface Control Selectors, (USB_Video_Class_1.1.pdf, A.9.7 VideoStreaming Interface Control Selectors) */
#define VS_CONTROL_UNDEFINED             	       0x00
#define VS_PROBE_CONTROL                 	       0x01
#define VS_COMMIT_CONTROL                     	   0x02
#define VS_STILL_PROBE_CONTROL               	   0x03
#define VS_STILL_COMMIT_CONTROL                    0x04
#define VS_STILL_IMAGE_TRIGGER_CONTROL      	   0x05
#define VS_STREAM_ERROR_CODE_CONTROL       	       0x06
#define VS_GENERATE_KEY_FRAME_CONTROL     	       0x07
#define VS_UPDATE_FRAME_SEGMENT_CONTROL    	       0x08

// Video Class-Specific VideoStreaming Interface Descriptor Subtypes, (USB_Video_Class_1.1.pdf, A.6 Video Class-Specific VS Interface Descriptor Subtypes) */
#define VS_UNDEFINED                               0x00
#define VS_INPUT_HEADER                            0x01
#define VS_OUTPUT_HEADER                           0x02
#define VS_STILL_IMAGE_FRAME                       0x03
#define VS_FORMAT_UNCOMPRESSED                     0x04
#define VS_FRAME_UNCOMPRESSED                      0x05
#define VS_FORMAT_MJPEG                            0x06
#define VS_FRAME_MJPEG                             0x07
#define VS_FORMAT_MPEG2TS                          0x0A
#define VS_FORMAT_DV                               0x0C
#define VS_COLORFORMAT                             0x0D
#define VS_FORMAT_FRAME_BASED                      0x10
#define VS_FRAME_FRAME_BASED                       0x11
#define VS_FORMAT_STREAM_BASED                     0x12

#define MSG_BACK_GET_DATA_N_DECODE                 0x00100000
#define IMAGE_DECODE_OUTBUF_NUM                    3  /*(JC)maximum jpeg decode output buffer number*/
//#define FRAME_RATE_TEST

extern INT32U *gframelist[];
extern INT32U *gframelist_m[];
extern INT32U *giTD_Table_Addr;
extern INT32U *giTD_Table_Addr_m;
extern INT8U giTD_buf_use[];
extern INT8U *g_buf;
extern INT8U *g_buf_m;
extern volatile INT32U g_usbh_int_queue;
extern INT8U *gUVC_setup_buf;
extern INT8U giTD_buf_not_clear[];
extern INT8U giTD_buf_use[];

#undef min
#define min(a,b) (((a)<(b))?(a):(b))
#undef max
#define max(a,b) (((a)>(b))?(a):(b))

enum
{
    MSG_USB_HOST_UVC_PLUG_IN = 0,
    MSG_USB_HOST_UVC_INITIAL,
    MSG_USB_HOST_UVC_STARTGETISODATA,
    MSG_USB_HOST_UVC_GETNEXTISODATA,
    MSG_USB_HOST_UVC_JPEG_DECODE,
    MSG_USB_HOST_UVC_PLUG_OUT,
    MSG_USB_HOST_UVC_RLS
};

enum
{
    MSG_UVC_DISP_INIT = 1,
//  MSG_UVC_DISP_RUN,
    MSG_UVC_DISP_DECODE_SCALE,
    MSG_UVC_DISP_HALT
};

//----- main uvc task control message -----
typedef enum
{
	MSG_UVC_INIT = 1,
	MSG_UVC_PREVIEW_EN,
	MSG_UVC_PREVIEW_DIS,
	MSG_UVC_RECORD_EN,
	MSG_UVC_RECORD_DIS,
	MSG_UVC_FORCE_CAPTURE_EN,
	MSG_UVC_CAPTURE_EN,
	MSG_UVC_CLOSE,
	MSG_UVC_SOF
}UVC_MSG_CMD;
//----- main uvc task status message -----
typedef enum
{
	MSG_INIT_SUCCESS=1,
	MSG_INIT_FAIL,
	MSG_USB_FAIL,
	MSG_PREVIEW_INIT_SUCCESS,
	MSG_PREVIEW_INIT_FAIL,
	MSG_PREVIEW_FRAME_END,
	MSG_CAPTURE_FAIL,
	MSG_CAPTURE_SUCCESS
}MSG_UVC_STATUS;


typedef struct
{
	unsigned char format_index;
	unsigned char frame_index;
	unsigned	buf_num;
	unsigned char	**buf;
	unsigned	ln;
}ST_UVC_PREVIEW_CTRL;

typedef struct
{
	unsigned type;
	unsigned char* buf;
	unsigned ln;
}ST_UVC_CAPTURE;

typedef struct
{
	unsigned Buf_Num;
	unsigned ln;
}ST_UVC_FRAME_PASER;

/* USB class code */
/*typedef enum
{
	UC_DEVICE 	= 0x00,
	UC_AUDIO	= 0x01,
	UC_CDC_CTRL	= 0x02,
	UC_HID		= 0x03,
	UC_PHYSICAL	= 0x05,
	UC_IMAGE	= 0x06,
	UC_PRINTER	= 0x07,
	UC_MSC		= 0x08,
	UC_HUB		= 0x09,
	UC_CDC		= 0x0A,
	UC_CCID		= 0x0B,
	UC_CS		= 0x0D,
	UC_VIDEO	= 0x0E
}USB_CLASS_CODES;*/

/* USB descriptor types */
/*typedef enum
{
	VS_UNDEFINED			= 0x00,
	VS_INPUT_HEADER			= 0x01,
	VS_OUTPUT_HEADER 		= 0x02,
	VS_STILL_IMAGE_FRAME	= 0x03,
	VS_FORMAT_UNCOMPRESSED	= 0x04,
	VS_FRAME_UNCOMPRESSED	= 0x05,
	VS_FORMAT_MJPEG			= 0x06,
	VS_FRAME_MJPEG			= 0x07,
	VS_FORMAT_MPEG2TS		= 0x0A,
	VS_FORMAT_DV			= 0x0C,
	VS_COLORFPRMAT			= 0x0D,
	VS_FORMAT_FRAME_BASED	= 0x10,
	VS_FRAME_FRAME_BASED	= 0x11,
	VS_FORMAT_STREAM_BASED	= 0x12
}UVC_VS_SUBTYPE;*/
//----- USB Video Class Specific Request Codes -----
/*typedef enum
{
	RC_UNDEFINED	= 0x00,
	SET_CUR			= 0x01,
	GET_CUR			= 0x81,
	GET_MIN			= 0x82,
	GET_MAX			= 0x83,
	GET_RES			= 0x84,
	GET_LEN			= 0x85,
	GET_INFO		= 0x86,
	GET_DEF			= 0x87
}UVC_CS_REQ;*/
//----- UVC Video Streaming Interface Control Selector -----
/*typedef enum
{
	VS_CONTROL_UNDEFINED	= 0x00,
	VS_PROBE_CONTROL,
	VS_COMMIT_CONTROL,
	VS_STILL_PROBE_CONTROL,
	VS_STILL_COMMIT_CONTROL,
	VS_STILL_IMAGE_TRIGGER_CONTROL,
	VS_STREAM_ERROR_CODE_CONTROL,
	VS_GENERATE_KEY_FRAME_CONTROL,
	VS_UPDATE_FRAME_SEGMENT_CONTROL,
	VS_SYNCH_DELAY_CONTROL
}UVC_VS_REQ;*/

typedef enum
{
	UVC_DATA_UNCOMPRESSED_YUV2 = 1,
	UVC_DATA_UNCOMPRESSED_NV12,
	UVC_DATA_MJPEG
}UVC_DATA_FORMAT;

#define UVC_MAX_FORMAT 4
#define UVC_MAX_FRAME  10

typedef struct
{
	unsigned short Width;
	unsigned short Height;
}ST_Resolution;

typedef struct
{
	unsigned short	IMG_NUM;
	ST_Resolution	res[UVC_MAX_FRAME];
}ST_UVC_STILL_INFO;

typedef struct
{
	unsigned char		FrameIndex;
	ST_Resolution		res;
	unsigned char		FrameInterType;
	unsigned 			MaxBuffer;
	unsigned			MinFrameInter;
	unsigned			MaxFrameInter;
	unsigned			FrameStep;
	unsigned			FrameInter[10];
}ST_UVC_FRAME_INFO;

typedef struct
{
	unsigned char		FormatIndex;
	unsigned char 		FormatType;
	unsigned char		FrameTypeNum;
	ST_UVC_FRAME_INFO	FRAME_INFO[UVC_MAX_FRAME];
	ST_UVC_STILL_INFO	STILL_INFO;
}ST_UVC_FORMAT_INFO;

typedef struct
{
	unsigned char		FormatTypeNum;
	unsigned char 		CaptureMode;
	ST_UVC_FORMAT_INFO	Format_INFO[UVC_MAX_FORMAT];
}ST_UVC_STREAM_INFO;
//----- USB Video Class Interface information -----
typedef struct
{
	unsigned char 	IF;				/* Interface Number */
	unsigned char	Alt_Num;		/* Alternative Interface Number */
	struct
	{
		unsigned char 	Alt;		/* Alternative Interface Index */
		unsigned char 	Endp;		/* Endpoint */
		unsigned char   Multi;
		unsigned short	MaxPkt;		/* Maximum Packet Size */
	}alt[15];
	struct							/* Interrupt Endpoint Information */
	{
		unsigned char 	Endp;		/* Endpoint */
		unsigned char 	Poll;		/* Polling Interval */
		unsigned short	MaxPkt;		/* Maximum Packet Size */
	}Int;
}ST_UVC_IF_INFO;

typedef struct
{
	ST_UVC_STREAM_INFO		STREAM_INFO;
	ST_UVC_IF_INFO			IF_INFO;
}ST_UVC_INFO;


enum jpegdec_buffer_state {
	BUF_STATE_IDLE	 = 0,
	BUF_STATE_QUEUED = 1,
	BUF_STATE_ACTIVE = 2,
	BUF_STATE_DONE	 = 3,
	BUF_STATE_ERROR	 = 4,
};

typedef struct
{
    INT8U jpegdecbuf_idx;
    char *bufhead;
    INT32U buflength;
    INT32U bytesused;
    INT32U total_len[UVC_JPGDEC_BUF_MQUEUE_MAX];
	enum jpegdec_buffer_state state;
	INT8U bufkey[UVC_JPGDEC_BUF_MQUEUE_MAX];
//    ALIGN64 char jpegdecbuf[UVC_JPGDEC_BUF_MQUEUE_MAX][UVC_JPGDEC_BUF_SIZE];
    char *jpegdecbufptr[UVC_JPGDEC_BUF_MQUEUE_MAX];
}PACKED UVC_DECBUF_INFO_T;
#endif // __USBH_UVC_H__
