/******************************************************
* drv_l2_usbd_user_uac_tbl.c
*
* Purpose: UAC(+HID) Data table data for USBD layer 2
*
* Author: Eugene Hsu
*
* Date: 2013/03/07
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version :
* History :
*
*******************************************************/

#include "project.h"
#include "drv_l2_usbd.h"
#include "usbd_uac.h"


/*******************************************************
	USBD Descriptor table
********************************************************/
/* Device descriptor */
#pragma data_alignment = 4
ALIGN4 const INT8U uac_device_descriptor_tbl[] =
{
     0x12,                      //bLength: 0x12 byte
 	 0x01,				        //bDescriptorType	: Device
	 0x00, 0x02,				//bcdUSB			: version 1.01
	 0x00, 						//bDeviceClass
	 0x00, 						//bDeviceSubClass
	 0x00,						//bDeviceProtocol
     0x40, 						//bMaxPacketSize0
	 0x3F, 0x1B,				//idVendor
	 0xff, 0x20,				//idProduct
	 0x00, 0x06,				//bcdDevice
	 0x01,						//iManufacturer
	 0x02,						//iProduct
	 0x00,						//iSerialNumber
	 0x01,						//bNumConfigurations
};

/* Qualifier descriptor */
const INT8U uac_qualifier_descriptor_tbl[]=
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

/* Config descriptor */
#pragma data_alignment = 4
ALIGN4 INT8U uac_config_descriptor_tbl[] =   //Configuration (0x09 byte)
{
	/* Configuration descriptor */
	0x09,				//Descriptor size is 9 bytes
    0x02,				//CONFIGURATION Descriptor Type
    //0xC0, 0x00,
//    0xD9,0x00, 			//The total length of data for this configuration is 217.
    0xE1,0x00,
    //0x03,
    0x04,				//This configuration supports 4 interfaces
    0x01,				//The value 1 should be used to select this configuration
    0x00,				//The device doesn't have the string descriptor describing this configuration
    0x80,				//Configuration characteristics : Bit 7: Reserved (set to one) 1 Bit 6: Self-powered 0 Bit 5: Remote Wakeup 0
    0xC8,				//Maximum power consumption of the device in this configuration is 400 mA

    /* Interface descriptor */
    0x09,				// Descriptor size is 9 bytes
    0x04, 				// INTERFACE Descriptor Type
    USBD_UAC_CTRL_INTF, // The number of this interface is 0.
    0x00,				// The value used to select the alternate setting for this interface is 0
    0x00,				// The number of endpoints used by this interface is 0 (excluding endpoint zero)
    0x01,				// The interface implements the Audio Interface class
    0x01,				// The interface implements the AUDIOCONTROL Subclass
    0x00,				// The interface doesn't use any class-specific protocols
    0x00,				// The device doesn't have a string descriptor describing this iInterface

    /* Audio Class Specific INTERFACE Descriptor*/
    0x0A,				// Size of the descriptor, 10 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x01,				// HEADER descriptor subtype
    0x00,0x01,			// Audio Device compliant to the USB Audio specification version 1.00
//    0x46,0x00,			// Total number of bytes returned for the class-specific AudioControl interface descriptor. Includes the combined length of this descriptor header and all Unit and Terminal descriptors.
    0x4E,0x00,
    0x02,				// The number of AudioStreaming and MIDIStreaming interfaces in the Audio Interface Collection to which this AudioControl interface belongs
    USBD_UAC_STREAM_INTF,//interface 1
	USBD_UAC_MIC_INTF,//interface 2

    /* Audio Class Specific INTERFACE Descriptor */
    0x0C,				// Size of the descriptor, 12 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x02,				// INPUT_TERMINAL descriptor subtype
    0x01,				// bTerminalID: 1.
    0x01,0x01,			// A Terminal dealing with a signal carried over an endpoint in an AudioStreaming interface.
    0x00,				// This Input Terminal has no association
    0x02,				// This Terminal's output audio channel cluster has 2 logical output channels
    0x03,0x00,			// Spatial locations present in the cluster
               			// Bit 0: Left Front 0
               			// Bit 1: Right Front 0
               			// Bit 2: Center Front 0
               			// Bit 3: Low Freq Enh 0
               			// Bit 4: Left Surround 0
               			// Bit 5: Right Surround 0
               			// Bit 6: Left of Center 0
               			// Bit 7: Right of Center 0
               			// Bit 8: Surround 1
               			// Bit 9: ...
    0x00,				// Index of a string descriptor, describing the name of the first logical channel.
    0x00,				// Index of a string descriptor, describing the Input Terminal.

    /* Audio Class Specific INTERFACE Descriptor */
    0x0C,				// Size of the descriptor, 12 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x02,				// INPUT_TERMINAL descriptor subtype
    0x04,				// bTerminalID: 4.
    0x01,0x02,			// A generic microphone that does not fit under any of the other classifications.
    0x00,				// This Input Terminal has no association
    0x01,				// This Terminal's output audio channel cluster has 1 logical output channels
    0x01,0x00,			// Spatial locations present in the cluster
               			// Bit 0: Left Front 0
               			// Bit 1: Right Front 0
               			// Bit 2: Center Front 0
              	 		// Bit 3: Low Freq Enh 0
              	 		// Bit 4: Left Surround 0
              	 		// Bit 5: Right Surround 0
              	 		// Bit 6: Left of Center 0
               			// Bit 7: Right of Center 0
               			// Bit 8: Surround 1
               			// Bit 9: ...
    0x00,				// Index of a string descriptor, describing the name of the first logical channel.
    0x00,				// Index of a string descriptor, describing the Input Terminal.

	/* Audio Class Specific INTERFACE Descriptor */
    0x09,				// Size of the descriptor, 9 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x03,				// OUTPUT_TERMINAL descriptor subtype
    0x03,				// bTerminalID: 3.
    0x01,0x03,			// A generic speaker or set of speakers that does not fit under any of the other classifications.
    0x00,				// This Output Terminal has no association
    USBD_UAC_SPEAKER_UID,  // ID of the Unit or Terminal to which this Terminal is connected.
    0x00,  				// Index of a string descriptor, describing the Output Terminal.

	/* Audio Class Specific INTERFACE Descriptor */
    0x09,				// Size of the descriptor, 9 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x03,				// OUTPUT_TERMINAL descriptor subtype
    0x02,				// bTerminalID: 2.
    0x01,0x01,			// A Terminal dealing with a signal carried over an endpoint in an AudioStreaming interface.
    0x00,				// This Output Terminal has no association
//  USBD_UAC_MIC_UID,	// ID of the Unit or Terminal to which this Terminal is connected.
    USBD_UAC_SU_UID,
    0x00,				// Index of a string descriptor, describing the Output Terminal.


//************************************************
	0x07, //Size of the descriptor, in bytes
	0x24, //CS_INTERFACE Descriptor Type
	0x05, //SELECTOR_UNIT descriptor subtype
	0x09, //Constant uniquely identifying the Unit within the audio function. This value is used in all requests to address this Unit.
	0x01, //Number of Input Pins of this Unit
//	0x05, //ID of the Unit or Terminal to which the Selector Unit is connected
	USBD_UAC_MIC_UID,//ID of the Unit or Terminal to which the Selector Unit is connected
	0x00, // Index of a string descriptor, describing the Selector Unit
//************************************************


	/* Audio Class Specific INTERFACE Descriptor */
    0x0A, 				// Size of the descriptor, 10 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x06, 				// FEATURE_UNIT descriptor subtype
    USBD_UAC_SPEAKER_UID, // bTerminalID: 6.
    0x01,				// ID of the Unit or Terminal to which this Feature Unit is connected.
    0x01,				// Size in bytes of an element of the bmaControls() array:
    0x03, 				// master channel 0: //dennis
                        // Bit 0: Mute 1
                        // Bit 1: Volume 0
                        // Bit 2: Bass 0
                        // Bit 3: Mid 0
                        // Bit 4: Treble 0
                        // Bit 5: Graphic Equalizer 0
                        // Bit 6: Automatic Gain 0
                        // Bit 7: Delay 0
                        // Bit 8: Bass Boost 0
                        // Bit 9: Loudness 0
    0x00,				// master channel 1:
                        // Bit 0: Mute 0
                        // Bit 1: Volume 0
                        // Bit 2: Bass 0
                        // Bit 3: Mid 0
                        // Bit 4: Treble 0
                        // Bit 5: Graphic Equalizer 0
                        // Bit 6: Automatic Gain 0
                        // Bit 7: Delay 0
                        // Bit 8: Bass Boost 0
                        // Bit 9: Loudness 0
    0x00,				// master channel 2:
                        // Bit 0: Mute 0
                        // Bit 1: Volume 0
                        // Bit 2: Bass 0
                        // Bit 3: Mid 0
                        // Bit 4: Treble 0
                        // Bit 5: Graphic Equalizer 0
                        // Bit 6: Automatic Gain 0
                        // Bit 7: Delay 0
                        // Bit 8: Bass Boost 0
                        // Bit 9: Loudness 0
    0x00,               // Index of a string descriptor, describing this Feature Unit.
    /* Audio Class Specific INTERFACE Descriptor */
//    0x08,				// Size of the descriptor, 8 bytes
    0x09,				// Size of the descriptor, in bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x06,				// FEATURE_UNIT descriptor subtype
    USBD_UAC_MIC_UID,	// bTerminalID: 5.
    0x04, 				// ID of the Unit or Terminal to which this Feature Unit is connected.
    0x01,				// Size in bytes of an element of the bmaControls() array:
    0x43, 				// master channel 0: //dennis
             			// Bit 0: Mute 1
              			// Bit 1: Volume 0
              			// Bit 2: Bass 0
              			// Bit 3: Mid 0
              			// Bit 4: Treble 0
              			// Bit 5: Graphic Equalizer 0
              			// Bit 6: Automatic Gain 0
              			// Bit 7: Delay 0
              			// Bit 8: Bass Boost 0
              			// Bit 9: Loudness 0
    0x00,               // master channel 1
    0x00, 				// Index of a string descriptor, describing this Feature Unit.
//---------------------------------------------------------------------------------------

    /* Standard INTERFACE Descriptor #1 */
    0x09,				// Descriptor size is 9 bytes
    0x04,				// INTERFACE Descriptor Type
    USBD_UAC_STREAM_INTF, // The number of this interface is 1.
    0x00,				// The value used to select the alternate setting for this interface is 0
    0x00,				// The number of endpoints used by this interface is 0 (excluding endpoint zero)
    0x01,				// The interface implements the Audio Interface class
    0x02,				// The interface implements the AUDIOSTREAMING Subclass
    0x00, 				// The interface doesn't use any class-specific protocols
    0x00,				// The device doesn't have a string descriptor describing this iInterface

    /* Standard INTERFACE Descriptor #1 */
    0x09,				// Descriptor size is 9 bytes
    0x04, 				// INTERFACE Descriptor Type
    USBD_UAC_STREAM_INTF, // The number of this interface is 1.
    0x01,				// The value used to select the alternate setting for this interface is 1
    0x01,				// The number of endpoints used by this interface is 1 (excluding endpoint zero)
    0x01, 				// The interface implements the Audio Interface class
    0x02,				// The interface implements the AUDIOSTREAMING Subclass
    0x00,				// The interface doesn't use any class-specific protocols
    0x00, 				// The device doesn't have a string descriptor describing this iInterface

    /* Audio Class Specific INTERFACE Descriptor */
    0x07,				// Size of the descriptor, 7 bytes
    0x24, 				// CS_INTERFACE Descriptor Type
    0x01,				// AS_GENERAL descriptor subtype
    0x01, 				// The Terminal ID of the Terminal to which the endpoint of this interface is connected.
    0x01, 				// Delay introduced by the data path. Expressed in number of frames.
    0x01,0x00, 			// PCM

    /* Audio Class Specific INTERFACE Descriptor */
    0x0B,				// Size of the descriptor, 11 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x02,				// FORMAT_TYPE descriptor subtype
    0x01,				// FORMAT_TYPE_I
    0x02,				// Indicates the number of physical channels in the audio data stream.
    USBD_UAC_SPK_SUBFRAME_BYTE,  				// The number of bytes occupied by one audio subframe. Can be 1, 2, 3 or 4.
    USBD_UAC_SPK_SUBFRAME_VALID_BIT, 			// The number of effectively used bits from the available bits in an audio subframe.
    0x01, 				// Indicates how the sampling frequency can be programmed:
    // 0x80,0xBB,0x00,  	// Sampling frequency 1 in Hz for this isochronous data endpoint.
    B3VAL(USBD_UAC_SPEAKER_SAMPLE_RATE),

    /* Standard ENDPOINT Descriptor */
    0x09,				// Descriptor size is 9 bytes
    0x05,				// ENDPOINT Descriptor Type
    0x0C, 				// Endpoint C, ISO out
    0x01, 				// Types -
              			//   Transfer: ISOCHRONOUS
              			//   Sync: Adaptive
              			//   Usage: Data EP
    WBVAL(EPC_MAX_PKT_SIZE),
    //0x00,0x03, 			// Maximum packet size for this endpoint is 192 Bytes. If Hi-Speed, 0 additional transactions per frame
    0x04,//0x01, 		// The polling interval value is every 1 Frames. If Hi-Speed, 1 uFrames/NAK
    0x00, 				// Refresh Rate 2**n ms where n = 0
    0x00, 				// Synchronization Endpoint (if used) is endpoint 0

    /* Audio Class Specific ENDPOINT Descriptor */
    0x07,				// Size of the descriptor, 7 bytes
    0x25,				// CS_ENDPOINT Descriptor Type
    0x01, 				// AUDIO_EP_GENERAL descriptor subtype
    0x00, 				//
              			//   Bit 0: Sampling Frequency 0
              			//   Bit 1: Pitch 0
              			//   Bit 7: MaxPacketsOnly 0
    0x00, 				// Indicates the units used for the wLockDelay field: 0: Undefined
    0x00,0x00,			// Indicates the time it takes this endpoint to reliably lock its internal clock recovery circuitry. Units used depend on the value of the bLockDelayUnits field.


    /* Standard INTERFACE Descriptor #2m alt 0 */
    0x09, 				// Descriptor size is 9 bytes
    0x04,	 			// INTERFACE Descriptor Type
    USBD_UAC_MIC_INTF,	// The number of this interface is 2.
    0x00,				// The value used to select the alternate setting for this interface is 0
    0x00, 				// The number of endpoints used by this interface is 0 (excluding endpoint zero)
    0x01, 				// The interface implements the Audio Interface class
    0x02, 				// The interface implements the AUDIOSTREAMING Subclass
    0x00, 				// The interface doesn't use any class-specific protocols
    0x00, 				// The device doesn't have a string descriptor describing this iInterface

    /* INTERFACE Descriptor #2, alt 5*/
    0x09,				// Descriptor size is 9 bytes
    0x04,				// INTERFACE Descriptor Type
    USBD_UAC_MIC_INTF,  // The number of this interface is 2.
    0x05, 				// The value used to select the alternate setting for this interface is 0x05
    0x01,				// The number of endpoints used by this interface is 1 (excluding endpoint zero)
    0x01,				// The interface implements the Audio Interface class
    0x02,			    // The interface implements the AUDIOSTREAMING Subclass
    0x00,				// The interface doesn't use any class-specific protocols
    0x00,			    // The device doesn't have a string descriptor describing this iInterface

    /* Audio Class Specific INTERFACE Descriptor */
    0x07, 				// Size of the descriptor, 7 bytes
    0x24, 				// CS_INTERFACE Descriptor Type
    0x01,				// AS_GENERAL descriptor subtype
    0x02,				// The Terminal ID of the Terminal to which the endpoint of this interface is connected.
    0x01,				// Delay introduced by the data path. Expressed in number of frames.
    0x01,0x00,			// PCM

    /* Audio Class Specific INTERFACE Descriptor */
    0x0B,				// Size of the descriptor, 11 bytes
    0x24,				// CS_INTERFACE Descriptor Type
    0x02,			    // FORMAT_TYPE descriptor subtype
    0x01,			    // FORMAT_TYPE_I
    0x01,				// Indicates the number of physical channels in the audio data stream.
    0x02,  				// The number of bytes occupied by one audio subframe. Can be 1, 2, 3 or 4.
    0x10,				// The number of effectively used bits from the available bits in an audio subframe.
    0x01,				// Indicates how the sampling frequency can be programmed:
    //0x40,0x1F,0x00,		// Sampling frequency 1 in Hz for this isochronous data endpoint.
    B3VAL(USBD_UAC_MICROPHONE_SAMPLE_RATE),

    /* Standard AS Isochronous audio data ENDPOINT Descriptor */
    0x09,				// Descriptor size is 9 bytes
    0x05,				// ENDPOINT Descriptor Type
    0x87,				// Endpoint 6, ISO in
    0x01,				// Types -
              			// Transfer: ISOCHRONOUS
              			// Sync: Adaptive
              			// Usage: Data EP
    0x00,0x01,			// Maximum packet size for this endpoint is 192 Bytes. If Hi-Speed, 0 additional transactions per frame
    0x04,//0x01,				// The polling interval value is every 1 Frames. If Hi-Speed, 1 uFrames/NAK
    0x00,				// Refresh Rate 2**n ms where n = 0
    0x00,				// Synchronization Endpoint (if used) is endpoint 0


    /* Audio Class Specific ENDPOINT Descriptor */
    0x07, 				// Size of the descriptor, 7 bytes
    0x25, 				// CS_ENDPOINT Descriptor Type
    0x01, 				// AUDIO_EP_GENERAL descriptor subtype
    0x00, 				// Bit 0: Sampling Frequency 0
          				// Bit 1: Pitch 0
              			// Bit 7: MaxPacketsOnly 0

    0x00, 				// Indicates the units used for the wLockDelay field: 0: Undefined
    0x00,0x00,			// Indicates the time it takes this endpoint to reliably lock its internal clock recovery circuitry. Units used depend on the value of the bLockDelayUnits field.

#if 1
    /* Standard INTERFACE Descriptor #3 */
    0x09,				//Descriptor size is 9 bytes
    0x04,				//INTERFACE Descriptor Type
    USBD_HID_INTF,  	//The number of this interface is 3.
    0x00,				//The value used to select the alternate setting for this interface is 0
    0x01,				//The number of endpoints used by this interface is 1 (excluding endpoint zero)
    0x03,				//The interface implements the HID class
    0x00,				//The interface implements the No SubClass Subclass
    0x00,				//The interface uses the None Protocol
    0x00,				//The device doesn't have a string descriptor describing this iInterface

    /* HID Descriptor */
    0x09,				//the total size of the HID descriptor, 9 bytes
    0x21,				//HID_DESCRIPTOR Descriptor Type
    0x01,0x02,			//Numeric expression identifying the HID Class Specification release.
    0x00,				//Not Supported
    0x01,				//number of class descriptors.
    0x22,				//REPORT_DESCRIPTOR
    0x21,0x00,			//the total size of the Report descriptor.

    /* Standard ENDPOINT Descriptor */
    0x07,				//Descriptor size is 7 bytes
    0x05, 				//ENDPOINT Descriptor Type
    0x83, 				//This is an IN endpoint with endpoint number 3
    0x03, 				//Types -
              			//  Transfer: INTERRUPT
             			//  Low Power: No
              			//  Pkt Size Adjust: No
    0x08,0x00, 			//Maximum packet size for this endpoint is 8 Bytes. If Hi-Speed, 0 additional transactions per frame
    0xff  	  			//The polling interval value is every 1 Frames. If Hi-Speed, every 1 uFrames
#endif
};

/* Index 0 string descriptor */
const INT8U uac_string0_Descriptor[] =
{
	 0x04,		//bLength
	 0x03,		//bDescriptorType
	 0x09, 0x04,//bString
};

/* Index 1 string descriptor */
#pragma data_alignment = 4
ALIGN4 const INT8U uac_string1_Descriptor[] =
{
    /* Index 0x01: Manufacturer */
    0x1A,		// bLength
    0x03,   	// bDescriptorType
    'G', 0x00,
	'E', 0x00,
	'N', 0x00,
	'E', 0x00,
	'R', 0x00,
	'A', 0x00,
	'L', 0x00,
    'P', 0x00,
    'L', 0x00,
    'U', 0x00,
    'S', 0x00,
    ' ', 0x00
};

/* Index 2 string descriptor */
#pragma data_alignment = 4
ALIGN4 const INT8U uac_string2_descriptor[] =
{
	/* Index 0x02: Product */
    0x1E,		// bLength
    0x03,		// bDescriptorType
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
    'U',0,
    'A',0,
    'C',0,
    ' ',0
};

/* USB report descriptor */
#pragma data_alignment = 4
ALIGN4 const INT8U hid_report_descriptor[]=
{
	0x05,0x0C,  	  //bType: Global, bTag: Usage Page, Usage Page(Consumer )
	0x09,0x01,  	  //bType: Local, bTag: Usage, Usage(Consumer Control)
	0xA1,0x01,  	  //bType: Main, bTag: Collection, Collection(Application)
	0x15,0x00,  	  //bType: Global, bTag: Logical Minimum, Logical Minimum(0x0)
	0x25,0x01,  	  //bType: Global, bTag: Logical Maximum, Logical Maximum(0x1)
	0x09,0xE9,  	  //bType: Local, bTag: Usage, Usage(Volume Increment)
	0x09,0xEA,  	  //bType: Local, bTag: Usage, Usage(Volume Decrement)
	0x09,0xE2,  	  //bType: Local, bTag: Usage, Usage(Mute)
	0x09,0xCD,  	  //bType: Local, bTag: Usage, Usage(Play/Pause)
	0x09,0xB5,  	  //bType: Local, bTag: Usage, Usage(Scan Next Track)
	0x09,0xB6,  	  //bType: Local, bTag: Usage, Usage(Scan Previous Track)
	0x09,0xB3,  	  //bType: Local, bTag: Usage, Usage(Fast Forward)
	0x09,0xB7,  	  //bType: Local, bTag: Usage, Usage(Stop)
	0x75,0x01,  	  //bType: Global, bTag: Report Size, Report Size(0x1)
	0x95,0x08,  	  //bType: Global, bTag: Report Count, Report Count(0x8 )
	0x81,0x42,  	  //bType: Main, bTag: Input, Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, Null State, Bit Field)
	0xC0  	  		  //bType: Main, bTag: End Collection
};

/* Max volume descriptor */
#pragma data_alignment = 4
ALIGN4 const INT8U uac_vol_max_descriptor[] =
{
    0xFF,
    0x80
};

/* Min volume descriptor */
#pragma data_alignment = 4
ALIGN4 const INT8U uac_vol_min_descriptor[] =
{
	0x00,
	0x80
};

/* Volume resolution descriptor */
#pragma data_alignment = 4
ALIGN4 const INT8U uac_vol_res_descriptor[] =
{
    0x01,
    0x00,
};
