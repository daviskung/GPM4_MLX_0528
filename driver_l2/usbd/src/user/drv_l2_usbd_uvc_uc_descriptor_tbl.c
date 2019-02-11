/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 *      Name:    USBDESC.C
 *      Purpose: USB Descriptors
 *      Version: V1.10
 *----------------------------------------------------------------------------
 *      This file is part of the uVision/ARM development tools.
 *      This software may only be used under the terms of a valid, current,
 *      end user licence from KEIL for a compatible version of KEIL software
 *      development tools. Nothing else gives you the right to use it.
 *
 *      Copyright (c) 2005-2007 Keil Software.
 *---------------------------------------------------------------------------*/
#include "drv_l2_usbd_uvc_uc.h"
#include "usbd_uac.h"
#include "usbd_uvc.h"

#if (AUDIO_SUPPORT == 0)
#define USB_CONFIG_DES_LEN	WBVAL( \
    0x00E7)
#elif UVC_H264 == 1
#define USB_CONFIG_DES_LEN	WBVAL( \
 	0x00EC + \
    UVC_INTERFACE_ASSOCIATION_DESC_SIZE + \
    USB_INTERFACE_DESC_SIZE + \
    AUDIO_CONTROL_INTERFACE_DESC_SZ(1) + \
    AUDIO_INPUT_TERMINAL_DESC_SIZE + \
    AUDIO_FEATURE_UNIT_DESC_SZ(0,1) + \
    AUDIO_OUTPUT_TERMINAL_DESC_SIZE + \
    USB_INTERFACE_DESC_SIZE + \
    USB_INTERFACE_DESC_SIZE + \
    AUDIO_STREAMING_INTERFACE_DESC_SIZE + \
    AUDIO_FORMAT_TYPE_I_DESC_SZ(1) + \
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE + \
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE)

#else
#define USB_CONFIG_DES_LEN	WBVAL( \
 	0x00E7 + \
    UVC_INTERFACE_ASSOCIATION_DESC_SIZE + \
    USB_INTERFACE_DESC_SIZE + \
    AUDIO_CONTROL_INTERFACE_DESC_SZ(1) + \
    AUDIO_INPUT_TERMINAL_DESC_SIZE + \
    AUDIO_FEATURE_UNIT_DESC_SZ(0,1) + \
    AUDIO_OUTPUT_TERMINAL_DESC_SIZE + \
    USB_INTERFACE_DESC_SIZE + \
    USB_INTERFACE_DESC_SIZE + \
    AUDIO_STREAMING_INTERFACE_DESC_SIZE + \
    AUDIO_FORMAT_TYPE_I_DESC_SZ(1) + \
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE + \
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE)
#endif


/* USB Standard Device Descriptor */
#pragma data_alignment = 4
ALIGN4 INT8U USB_UVC_UC_DeviceDescriptor[] =
{
	USB_DEVICE_DESC_SIZE,                      // bLength                  18
	USB_DEVICE_DESCRIPTOR_TYPE,                // bDescriptorType          1
	WBVAL(0x0200),						   	 // bcdUSB                   2.00
	0xEF,                                      // bDeviceClass             239 Miscellaneous Device
	0x02,                                      // bDeviceSubClass          2 Common Class
	0x01,                                      // bDeviceProtocol          1 Interface Association
	0x40,                           			 // bMaxPacketSize0
	WBVAL(0x1b3f),                             // idVendor				 Vendor ID
	WBVAL(0x2005),                             // idProduct                Product ID
	WBVAL(0x0100),                             // bcdDevice                1.00
	0x01,                                      // iManufacturer            1
	0x02,                                      // iProduct                 2
	0x00,                                      // iSerialNumber            3
	0x01                                       // bNumConfigurations       1
};

#pragma data_alignment = 4
ALIGN4 INT8U USB_UVC_UC_Qualifier_Descriptor_TBL[]=
{
	0x0A,                   //bLength: 0x0A byte
	0x06,                   //bDescriptorType: DEVICE_QUALIFIER
	0x00, 0x02,             //bcdUSB: version 200 // 0x00,0x02
	0x00,                   //bDeviceClass:
	0x00,                   //bDeviceSubClass:
	0x00,                   //bDeviceProtocol:
	0x40,                   //bMaxPacketSize0: maximum packet size for endpoint zero
	0x01,                   //bNumConfigurations: 1 configuration
	0x00					//bReserved
};

/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
#pragma data_alignment = 4
ALIGN4 INT8U USB_UVC_UC_ConfigDescriptor[] =
{
	/* Configuration descriptor 9 bytes */
	USB_CONFIGUARTION_DESC_SIZE,               // bLength                  9
	USB_CONFIGURATION_DESCRIPTOR_TYPE,         // bDescriptorType          2
	//WBVAL(0x00E7),							 // wTotalLength
	USB_CONFIG_DES_LEN,			    	 		// wTotalLength
#if (AUDIO_SUPPORT == 0)
	0x02,										 // bNumInterface			 support 2 interfaces
#else
	0x04,										 // bNumInterface			 support 2 interfaces
#endif
	0x01,                                      // bConfigurationValue      1 ID of this configuration
	0x00,                                      // iConfiguration           0 no description available
	USB_CONFIG_BUS_POWERED ,                   // bmAttributes           0x80 Bus Powered
	USB_CONFIG_POWER_MA(100),                  // bMaxPower              100 mA

	/* Interface Association Descriptor 8 bytes */
	UVC_INTERFACE_ASSOCIATION_DESC_SIZE,       // bLength                 0x08
	USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE, // bDescriptorType         0x0b
	0x00,									   // bFirstInterface			0x00
	0x02,                                      // bInterfaceCount         0x02
	CC_VIDEO,                                  // bFunctionClass          0x0e Video class
	SC_VIDEO_INTERFACE_COLLECTION,             // bFunctionSubClass       0x03 Video Interface Collection
	PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol      0x00 (protocol undefined)
	0x02,                                      // iFunction               0x02

	/* VideoControl Interface Descriptor */

	/* Standard VC Interface Descriptor  = interface 0 9 bytes */
	USB_INTERFACE_DESC_SIZE,                   // bLength                  9
	USB_INTERFACE_DESCRIPTOR_TYPE,             // bDescriptorType          4
	USB_UVC_VCIF_NUM,                          // bInterfaceNumber         0 index of this interface
	0x00,                                      // bAlternateSetting        0 index of this setting
	0x01,                                      // bNumEndpoints            1 one interrupt endpoint
	CC_VIDEO,                                  // bInterfaceClass          14 Video
	SC_VIDEOCONTROL,                           // bInterfaceSubClass       1 Video Control
	PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
	0x02,                                      // iFunction

	/* Class-specific VC Interface Descriptor 13 bytes */
	0x0D,									     // bLength
	CS_INTERFACE,                              // bDescriptorType          36(INTERFACE)
	0x01,                                      // bDescriptorSubtype       0x01
	WBVAL(UVC_VERSION),                        // bcdUVC                   1.00
	WBVAL(0x004D),                             // wTotalLength
	DBVAL(0x01C9C380),                         // dwClockFrequency         30000000 HZ
	0x01,                                      // bInCollection            1 one streaming interface
	0x01,                                      // baInterfaceNr( 0)        1 VS interface 1 belongs to this VC interface

	/* Input Terminal Descriptor (Camera) 18 bytes */
	0x12,								         // bLength                  size = 18 bytes
	CS_INTERFACE,                              // bDescriptorType          36 (INTERFACE)
	VC_INPUT_TERMINAL,                         // bDescriptorSubtype       0x02 (INPUT_TERMINAL)
	0x01,                                      // bTerminalID              1 ID of this Terminal
	WBVAL(ITT_CAMERA),                         // wTerminalType            0x0201 Camera Sensor
	0x00,                                      // bAssocTerminal           0 no Terminal assiciated
	0x00,                                      // iTerminal                0 no description available
	WBVAL(0x0000),                             // wObjectiveFocalLengthMin 0x00
	WBVAL(0x0000),                             // wObjectiveFocalLengthMax 0x00
	WBVAL(0x0000),                             // wOcularFocalLength       0x00
	0x03,                                      // bControlSize             0x03
	0x0E,										 // bmControls 0             D1 = D2 = D3 = 1
	0x00,                                      // bmControls 1             0x00 no controls supported
	0x00,                                      // bmControls 2             0x00 no controls supported

	/* Output Terminal Descriptor 9 bytes */
	0x09,          							 // bLength                  0x09
	CS_INTERFACE,                              // bDescriptorType          36 (INTERFACE)
	VC_OUTPUT_TERMINAL,                        // bDescriptorSubtype       0x03 (OUTPUT_TERMINAL)
	0x02,                                      // bTerminalID              2 ID of this Terminal
	WBVAL(TT_STREAMING),                       // wTerminalType            0x0101 USB streaming terminal
	0x00,                                      // bAssocTerminal           0 no Terminal assiciated
	0x04,                                      // bSourceID                The source ID is 4
	0x00,                                      // iTerminal                0 no description available

	/* Processing Unit Descriptor 11 bytes */
	0x0B,                                      // bLength                 11 bytes
	CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
	VC_PROCESSING_UNIT,                        // bDescriptorSubtype       5 (PROCESSING_UNIT)
	0x03,                                      // bUnitID                  0x03
	0x01,										 // bSourceID                0x01
	WBVAL(0x0000),                             // wMaxMultiplier           0 not used
	0x02,                                      // bControlSize             2 two control bytes
	0x7F,										 // bmControls 0
	0x14,										 // bmControls 1
	0x00,                                      // iProcessing              0 no description available

	/* VC externsion unit descriptor 26 bytes*/
	0x1A,										 // bLength
	CS_INTERFACE,                              // bDescriptorType          36 (INTERFACE)
	VC_EXTENSION_UNIT,                         // bDescriptorSubtype       0x06 (EXTENSION UNIT)
	0x04,										 // bUnitID					 0x04
	DBVAL(0xC2B1CCAD),						 // guidExtensionCode
	DBVAL(0x48B8ABF6),						 // guidExtensionCode
	DBVAL(0xD432378E),						 // guidExtensionCode
	DBVAL(0xECFEA3F3),						 // guidExtensionCode
	0x08,										 // bNumControls			 0x08
	0x01,										 // bNrInPins				 0x01
	0x03,										 // baSourceID(1)			 0x03
	0x01,										 // bControlSize			 0x01
	0x3F,										 // bmControls               0x3F
	0x00,										 // iExtension				 0x00

	/* Standard Interrupt Endpoint Descriptor 7 bytes */
	// we use an interrupt endpoint for notification
	USB_ENDPOINT_DESC_SIZE,                   // bLength                 7
	USB_ENDPOINT_DESCRIPTOR_TYPE,             // bDescriptorType         5 (ENDPOINT)
	USB_ENDPOINT_IN(1),                       // bEndpointAddress        0x81 EP 1 IN
	USB_ENDPOINT_TYPE_INTERRUPT,              // bmAttributes            3 interrupt transfer type
	WBVAL(0x0040),                            // wMaxPacketSize          0x0008 64 bytes
	0x04,                                     // bInterval               every 8 uFrames interval

	/* Class-Specific Interrupt Endpoint Descriptor 5 bytes */
	// mandatory if Standard Interrupt Endpoint is used
	UVC_VC_ENDPOINT_DESC_SIZE,                // bLength                  5
	CS_ENDPOINT,                              // bDescriptorType          0x25 (CS_ENDPOINT)
	EP_INTERRUPT,                             // bDescriptorSubtype       3 (EP_INTERRUPT)
	WBVAL(0x0400),                            // wMaxTransferSize         1024 bytes status packet

	/* Video Streaming Interface Descriptor */

	/* Standard VS Interface Descriptor  = interface 1  9 bytes*/
	// alternate setting 0 = Zero Bandwidth
	USB_INTERFACE_DESC_SIZE,                   // bLength                  9
	USB_INTERFACE_DESCRIPTOR_TYPE,             // bDescriptorType          4
	USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
	0x00,                                      // bAlternateSetting        0 index of this setting
	0x00,                                      // bNumEndpoints            0 no EP used
	CC_VIDEO,                                  // bInterfaceClass          14 Video
	SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
	PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
	0x00,                                      // iInterface               0 no description available

	/* Class-specific VS Header Descriptor (Input) 14 bytes */
	0x0E,										 // bLength                  14 bytes
	CS_INTERFACE,                              // bDescriptorType          36 (INTERFACE)
	VS_INPUT_HEADER,                           // bDescriptorSubtype       0x01 (INPUT_HEADER)
	0x01,                                      // bNumFormats              0x01 one format descriptor follows
	#if	UVC_H264 == 1
	WBVAL(0x0E + 0xE +0x1C+0x22+0x06),
	#else
	//WBVAL(0x0079),                             // wTotalLength             header+frame/format descriptors
	WBVAL(0x005B),                             // wTotalLength             header+frame/format descriptors
	#endif
	USB_ENDPOINT_IN(5),                        // bEndPointAddress         0x85 EP 5 iso-IN
	0x00,                                      // bmInfo                   0 no dynamic format change supported
	0x02,                                      // bTerminalLink            2 supplies terminal ID
	0x02,                                      // bStillCaptureMethod      2 supports still image capture method1
	0x01,                                      // bTriggerSupport          1 HW trigger supported for still image capture
	0x01,                                      // bTriggerUsage            1 HW trigger initiate a still image capture
	0x01,                                      // bControlSize             1 one byte bmaControls field size
	0x00,                                      // bmaControls(0)           0 no VS specific controls

#if	UVC_H264 == 1
	0x1C,										//bLength
	CS_INTERFACE,								//bDescriptorType
	VS_FORMAT_FRAME_BASED,						//bDescriptorSubtype = VS_FORMAT_FRAME_BASED
	0x01,										//bFormatIndex
	0x01,										//bNumFrameDescriptors
	DBVAL(0x34363248),							//guidFormat = H.264 streams GUIDs
	WBVAL(0x0000),								//guidFormat
	WBVAL(0x0010),								//guidFormat
	DBVAL(0xAA000080),							//guidFormat
	DBVAL(0x719B3800),							//guidFormat
	0x10,										//bBitsPerPixel
	0x01,										//bDefaultFrameIndex
	0x00,										//bAspectRatioX
	0x00,										//bAspectRatioY
	0x00,										//bmInterlaceFlags
	0x00,										//bCopyProtect
	0x01,										//bVariableSize

	0x22,										//bLength = 34
	CS_INTERFACE,								//bDescriptorType
	VS_FRAME_FRAME_BASED,						//bDescriptorSubtype
	0x01,										//bFrameIndex
	0x00,										//bmCapabilities
#if (USBD_UVC_CAM_RES == CAM_RES_720P)
	WBVAL(1280),								//wWidth
	WBVAL(720),									//wHeight
#elif(USBD_UVC_CAM_RES == CAM_RES_VGA)
	WBVAL(640),									//wWidth
	WBVAL(480),									//wHeigh
#else
	WBVAL(1280),								//wWidth
	WBVAL(720),									//wHeigh
#endif
	DBVAL(0x0D2F0000),							//dwMinBitRate
	DBVAL(0x1A5E0000),							//dwMaxBitRate
	DBVAL(0x00051615),							//dwDefaultFrameInterval
	0x02,										//bFrameIntervalType
	DBVAL(0x00000000),							//dwBytesPerLine
	DBVAL(0x00051615),							//adwFrameInterval[1]
	DBVAL(0x000A2C2A),							//adwFrameInterval[2]
#else
	/* Class-specific VS Format Descriptor 27 bytes */
	0x1B,                                      // bLength                  27
	CS_INTERFACE,                              // bDescriptorType          36 (INTERFACE)
	VS_FORMAT_UNCOMPRESSED,                    // bDescriptorSubtype       4 (VS_FORMAT_UNCOMPRESSED)
	0x01,                                      // bFormatIndex             1 first (and only) format descriptor
	0x01,                                      // bNumFrameDescriptors     1 one frame descriptor follows
	//0x01,                                      // bNumFrameDescriptors     1 one frame descriptor follows
	DBVAL(0x32595559),						 // guidFormat
	DBVAL(0x00100000),						 // guidFormat
	DBVAL(0xAA000080),                         // guidFormat
	DBVAL(0x719B3800),                         // guidFormat
	0x10,										 // bBitsPerPixel
	0x01,                                      // bDefaultFrameIndex       1 default frame index is 1
	0x00,                                      // bAspectRatioX            0 non-interlaced stream - not required
	0x00,                                      // bAspectRatioY            0 non-interlaced stream - not required
	0x00,                                      // bmInterlaceFlags         0 non-interlaced stream
	0x00,                                      // bCopyProtect             0 no restrictions
	/* Class specific VS Frame Descriptor 30 bytes */
	0x1E,                                      // bLength                 30
	CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
	VS_FRAME_UNCOMPRESSED,                     // bDescriptorSubtype      0x05  (VS_FRAME_UNCOMPRESSED)
	0x01,                                      // bFrameIndex             0x01 Frame Descripot
	0x00,                                      // bmCapabilities          0x01 Still images using capture method 1
	WBVAL(640),                                // wWidth                  640
	WBVAL(480),                                // wHeight                 480
	DBVAL(0x08CA0000),                         // dwMinBitRate            640x480x2x30
	DBVAL(0x08CA0000),                         // dwMaxBitRate            640x480x2x30
	DBVAL(0x00096000),                         // dwMaxVideoFrameBufferSize   38016 max video/still frame size in bytes 640*480*2
	//30 frame set
	DBVAL(0x00051615),                         // dwDefaultFrameInterval     666666 default frame interval is 333333ns (30fps)
	0x01,                                      // bFrameIntervalType         0x01 continuous frame interval
	DBVAL(0x00051615),                         // dwFreameInterval(1)        666666 min frame interval is 333333ns (30fps)
	#endif
	/* VS Still image frame descriptor 14 bytes */
	0x0E,										 // bLength                 14
	//0x0A,										 // bLength                 14
	CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
	VS_STILL_IMAGE_FRAME,						 // bDescriptorSubtype      0x03 still image frame
	0x00,										 // bEndpointAddress
	0x02,										 // bNumImageSizePatterns   0x02
	WBVAL(640),                              	  // wWidth                 640
	WBVAL(480),                               	 // wHeight                 480
	WBVAL(320),                               	 // wWidth                  320
	WBVAL(240),                               	 // wHeight                 240
	0x00,										 // bNumCompressionPatterns

	/* VS COLORFORMAT descriptor 6 bytes */
	0x06,										 // bLength					0x06
	CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
	VS_COLORFORMAT,							 // bDescriptorSubtype      0x0D (VS_COLORFORMAT)
	0x01,										 // bColorPrimaries
	0x01,                                      // bXferCharacteristics
	0x04,										 // bMatrixCoefficients

	/* Standard VS Interface Descriptor  = interface 1 9 bytes */
	// alternate setting 1 = operational setting
	USB_INTERFACE_DESC_SIZE,                   // bLength                  9
	USB_INTERFACE_DESCRIPTOR_TYPE,             // bDescriptorType          4
	USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
	0x07,                                      // bAlternateSetting        7 index of this setting
	0x01,                                      // bNumEndpoints            1 one EP used
	CC_VIDEO,                                  // bInterfaceClass         14 Video
	SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
	PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
	0x00,                                      // iInterface               0 no description available

	/* Standard VS Isochronous Video data Endpoint Descriptor 7 bytes */
	USB_ENDPOINT_DESC_SIZE,                   // bLength                  7
	USB_ENDPOINT_DESCRIPTOR_TYPE,             // bDescriptorType          5 (ENDPOINT)
	USB_ENDPOINT_IN(5),                       // bEndpointAddress         0x87 EP 7 IN usb2.0
	USB_ENDPOINT_TYPE_ISOCHRONOUS |           // bmAttributes             0x05 isochronous transfer type
	USB_ENDPOINT_SYNC_ASYNCHRONOUS,           //                          asynchronous synchronizationtype
	WBVAL(0x1400),                            // wMaxPacketSize           1024x3
	//WBVAL(0x1354),                            // wMaxPacketSize           852x3	alt = 6
	//WBVAL(0x0C00),                            // wMaxPacketSize           1024x2	alt = 5
	//WBVAL(0x0B00),                            // wMaxPacketSize           768x2	alt = 4
	//WBVAL(0x0400),                            // wMaxPacketSize           1024x1  alt = 3
	0x01,                                     // bInterval                1 one frame interval

#if (AUDIO_SUPPORT == 1)
	/* audio IAD descriptor */
	0x8,                                       // bLength                  8
	USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE, // bDescriptorType         11
	0x02,                                      // bFirstInterface          2
	0x02,                                      // bInterfaceCount          2
	0x01,                                       // bFunctionClass          audio
	0x00,                                       // bFunctionSubClass       audio stream
	PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
	0x04,                                      // iFunction

	/* interface 2 ,audio interface descriptor */
	USB_INTERFACE_DESC_SIZE,                   // bLength                  9
	USB_INTERFACE_DESCRIPTOR_TYPE,             // bDescriptorType          4
	0x02,                                      // bInterfaceNumber         2 index of this interface
	0x00,                                      // bAlternateSetting        0 index of this setting
	0x00,                                      // bNumEndpoints
	0x01,                                      // bInterfaceClass          audio
	0x01,                                      // bInterfaceSubClass       audio Control interface
	PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
	0x04,                                      // iFunction

	/* Audio Control Header Interface */
	AUDIO_CONTROL_INTERFACE_DESC_SZ(1),   /* bLength */
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
	AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
	WBVAL(0x0100), /* 1.00 */             /* bcdADC */
	WBVAL(                                /* wTotalLength */
	AUDIO_CONTROL_INTERFACE_DESC_SZ(1) +  /* 9 */
	AUDIO_INPUT_TERMINAL_DESC_SIZE     +  /* 12 */
	AUDIO_FEATURE_UNIT_DESC_SZ(0,1)    +  /* 9 */
	AUDIO_OUTPUT_TERMINAL_DESC_SIZE       /* 9 */
	),
	1,                                    /* bInCollection Number of streaming interface */
	3,                                    /* audiostreaming interface 1 belongs to this audiocontrol interface */

	/* Audio Input Terminal */
	AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
	AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
	0x03,                                 /* bTerminalID */
	WBVAL(AUDIO_TERMINAL_MICROPHONE),     /* wTerminalType */
	0x00,                                 /* bAssocTerminal */
	0x01,                                 /* bNrChannels */
	WBVAL(AUDIO_CHANNEL_M),               /* wChannelConfig,mono */
	0x00,                                 /* iChannelNames */
	0x00,                                 /* iTerminal */

	/* Audio Feature Unit */
	AUDIO_FEATURE_UNIT_DESC_SZ(0,1),      /* bLength, 8 */
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
	AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype, 6 */
	0x05,                                 /* bUnitID */
	0x03,                                 /* bSourceID */
	0x01,                                 /* bControlSize */
	AUDIO_CONTROL_MUTE |
	AUDIO_CONTROL_VOLUME,                 /* bmaControls(0) */
	0x00,                                 /* iTerminal */

	/* Audio Output Terminal */
	AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
	AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
	0x04,                                 /* bTerminalID */
	WBVAL(AUDIO_TERMINAL_USB_STREAMING),  /* wTerminalType */
	0x00,                                 /* bAssocTerminal */
	0x05,                                 /* bSourceID */
	0x00,                                 /* iTerminal */

	/* Interface 3, Alternate Setting 0, Audio Streaming - Zero Bandwith */
	USB_INTERFACE_DESC_SIZE,              /* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
	0x03,                                 /* bInterfaceNumber */
	0x00,                                 /* bAlternateSetting */
	0x00,                                 /* bNumEndpoints */
	USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
	AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
	AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
	0x00,                                 /* iInterface */

	/* Interface 3, Alternate Setting 1, Audio Streaming - Operational */
	USB_INTERFACE_DESC_SIZE,              /* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
	0x03,                                 /* bInterfaceNumber */
	0x05,                                 /* bAlternateSetting */
	0x01,                                 /* bNumEndpoints */
	USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
	AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
	AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
	0x00,                                 /* iInterface */

	/* Audio Streaming Interface */
	AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
	AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
	0x04,                                 /* bTerminalLink */
	0x01,                                 /* bDelay */
	WBVAL(AUDIO_FORMAT_PCM),              /* wFormatTag */

	/* Audio Type I Format */
	AUDIO_FORMAT_TYPE_I_DESC_SZ(1),       /* bLength */
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
	AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
	AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
	0x01,                                 /* bNrChannels */
	0x02,                                 /* bSubFrameSize */
	16,                                   /* bBitResolution */
	0x01,                                 /* bSamFreqType */
	//B3VAL(8000),                         /* tSamFreq */
	B3VAL(USBD_UAC_MICROPHONE_SAMPLE_RATE), /*tSamFreq */
	//B3VAL(11025),                         /* tSamFreq */
	//B3VAL(16000),                         /* tSamFreq */
	//B3VAL(22050),                         /* tSamFreq */
	//B3VAL(32000),                         /* tSamFreq */
	//B3VAL(44100),                         /* tSamFreq */
	//B3VAL(48000),                         /* tSamFreq */
	//B3VAL(96000),

	/* Endpoint - Standard Descriptor */
	AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
	USB_ENDPOINT_IN(7),                   /* bEndpointAddress */
	USB_ENDPOINT_TYPE_ISOCHRONOUS,        /* bmAttributes */
	WBVAL(256),                           /* wMaxPacketSize */
	0x4,                                  /* bInterval */
	0x00,                                 /* bRefresh */
	0x00,                                 /* bSynchAddress */

	/* Endpoint - Audio Streaming */
	AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
	AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
	AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
	0x00,                                 /* bmAttributes */
	0x00,                                 /* bLockDelayUnits */
	WBVAL(0x0000),                        /* wLockDelay */
#endif
/* Terminator */
  0x00                                      // bLength                  0
};

#pragma data_alignment = 4
ALIGN4 INT8U UVC_UC_String0_Descriptor[] =
{
    /* Index 0x00: LANGID Codes */
    0x04,                                   // bLength                  4
    USB_STRING_DESCRIPTOR_TYPE,             // bDescriptorType          3 (STRING)
    0x09,                      	 // wLANGID             0x0409 US English
    0x04,
};

#pragma data_alignment = 4
ALIGN4 INT8U UVC_UC_String1_Descriptor[] =
{
    /* Index 0x01: Manufacturer */
     0x10,                                   // bLength                 28
    USB_STRING_DESCRIPTOR_TYPE,             // bDescriptorType          3 (STRING)
    'G',0,
    'E',0,
    'N',0,
    'E',0,
    'R',0,
    'A',0,
    'L',0
};

#pragma data_alignment = 4
ALIGN4 INT8U UVC_UC_String2_Descriptor[] =
{
  /* Index 0x02: Product */
	0x20,                                   // bLength                  38
	USB_STRING_DESCRIPTOR_TYPE,             // bDescriptorType          3 (STRING)
    'G',0,
    'E',0,
    'N',0,
    'E',0,
    'R',0,
    'A',0,
    'L',0,
    ' ',0,
    '-',0,
    ' ',0,
    'V',0,
    'I',0,
    'D',0,
    'E',0,
    'O',0
};

#pragma data_alignment = 4
ALIGN4 INT8U UVC_UC_String3_Descriptor[] =
{
    /* Index 0x03: Serial Number */
    0x14,                                   // bLength                  20
    USB_STRING_DESCRIPTOR_TYPE,             // bDescriptorType          3 (STRING)
    'D',0,
    'e',0,
    'm',0,
    'o',0,
    ' ',0,
    '1',0,
    '.',0,
    '0',0,
    '0',0
};

#pragma data_alignment = 4
ALIGN4 INT8U UVC_UC_String4_Descriptor[] =
 {
    /* Index 0x04:  */
    0x20,                                   // bLength
    USB_STRING_DESCRIPTOR_TYPE,             // bDescriptorType          3 (STRING)
    'G',0,
    'E',0,
    'N',0,
    'E',0,
    'R',0,
    'A',0,
    'L',0,
    ' ',0,
    '-',0,
    ' ',0,
    'A',0,
    'U',0,
    'D',0,
    'I',0,
    'O',0
};

#pragma data_alignment = 4
ALIGN4 INT8U UVC_UC_String5_Descriptor[] =
{
    /* Index 0x05: endof string  */
    0x04,                                   // bLength                  4
    USB_STRING_DESCRIPTOR_TYPE,             // bDescriptorType          3 (STRING)
    0,0
};
