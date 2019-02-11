/*
* Purpose: Driver layer2 for controlling HDMI
*
* Author: Tristan
*
* Date: 2008/12/11
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.06
*/

#include "drv_l1.h"
#include "Customer.h"
#include "drv_l1_i2c.h"
#include "drv_l1_tft.h"
#include "drv_l2_hdmi.h"
#include "drv_l2_sccb.h"
//#include "bitslice.h"
//#include "hdcp_cipher.h"

//#if (defined _DRV_L2_HDMI) && (_DRV_L2_HDMI == 1)

#define  HDMI_640X480P60_BIT      0x00000001
#define  HDMI_720X480P60_BIT      0x00000002
#define  HDMI_1280X720P60_BIT     0x00000004

#define D1_MODE	2
#define TFT_1280x720_MODE		6

#define  HDMI_SEGMENT_ID	0x60
#define  HDMI_DDC_ID_0		0xA0
#define  HDMI_DDC_ID_1		0xA1
#define  HDMI_HDCP_ID_0		0x74
#define  HDMI_HDCP_ID_1		0x76
#define RETRY_CNT 5

#if 1 //sccb
static void *ddc_handle;
static void *hdcp_handle;
#else //hardware I2C
static drv_l1_i2c_bus_handle_t hHdmiDDC;
static drv_l1_i2c_bus_handle_t hHdmiDDCSegment;
#endif

//HDMI EDID check ----------------------------------------------------------------------------
enum {
	HDMI_640X480P60 = 0,
	HDMI_720X480P60,
	HDMI_1280X720P60,
	HDMI_VIDEO_TIMING_NUM
};

typedef struct
{
	/*	Progressive: 0, Interlaced: 1	*/
	/*	Negative: 0, Positive: 1	*/
	short	PixelClock,	HFreq,	VFreq;
} HDMI_TIMMING_TAB;

static const HDMI_TIMMING_TAB HdmiTiming[HDMI_VIDEO_TIMING_NUM] =
{
	//	PxlClk	HFreq	VFreq	HRes	VRes	Intrlac	HTotal	HBlank	HFPorch	HSWidth	HBPorch	HPol	VTotal	VBlank	VFPorch	VSWidth	VBPorch	VPol	Oversmp	ID	IsPC	PHY1		PHY2
	{	2475,	315,	60	}, 	// 640 x 480 x60
	{	2700,	315,	60	},	// 480  p60
	{	7425,	450,	60	},	// 720  p60
};

typedef struct
{
    unsigned short PixelClock;                                     // in 10khz
    unsigned char HorizontalAddressableVideo;                      // in pixels
    unsigned char HorizontalBlanking;                              // in pixels
    unsigned char HorizontalExtraInfo;
    unsigned char VerticalAddressableVideo;
    unsigned char VerticalBlanking;
    unsigned char VerticalExtraInfo;

    unsigned char HorizontalFrontPorch;                            //  in pixels
    unsigned char HorizontalSyncPulseWidth;
    unsigned char VerticalSync;
    unsigned char HV_Info;

    unsigned char HorizontalAddressableVideoImageSize;              // in mm
    unsigned char VerticalAddressableVideoImageSize;                // in mm
    unsigned char HV_AddressableVideoImageSize;						// in mm
    unsigned char RightHorizontalBorderOrLeftHorizontalBorder;      // in pixels
    unsigned char TopVerticalBorderOrBottomVerticalBorder;          // in Lines
    unsigned char Info;
} VESA_E_EDID_TIMING_MODE;


typedef struct
{
    unsigned long Indicator;
    unsigned char DisplayRangeLimitsOffsets;
    unsigned char MinimumVerticalRate;
    unsigned char MaximumVerticalRate;
    unsigned char MinimumHorizontalRate;
    unsigned char MaximumHorizontalRate;
    unsigned char MaximumPixelClock;
    unsigned char VideoTimingSupportFlags;
    unsigned char VideoTimingData[7];
} VESA_E_EDID_DISPLAY_RANGE_LIMITS;

typedef struct
{
    int Size, Blanking, FrontPorch, PluseWidth;
} TIMING;

typedef struct
{
    int PixelClock;
    int hSize, hBlanking, hFrontPorch, hPluseWidth;
    int vSize, vBlanking, vFrontPorch, vPluseWidth;
} FRAME_TIMING;

typedef struct
{
    int maxPixelClock;          // MHz
    int hMaxRate, hMinRate;     // kHz
    int vMaxRate, vMinRate;     // Hz
} TIMING_LIMITS;

static int hdmi_check_supported(unsigned char *edid, unsigned long *flag_supported, unsigned int offset)
{
    int i;
    const unsigned char *ptr;
    TIMING_LIMITS Limits;
    int flag_timing_limits = 0;

    DBG_PRINT("EDID Version = %d.%d\r\n", edid[0x12], edid[0x13]);
    ptr = edid + offset;

    for(i=0;i<4;i++)
    {

        if( ptr[0] != 0 ||
                ptr[1] != 0) // check if timing descriptor
        {
            const VESA_E_EDID_TIMING_MODE *VesaTiming = (const VESA_E_EDID_TIMING_MODE*)ptr;
            FRAME_TIMING Timing;
			unsigned char HorizontalBlanking_upper = VesaTiming->HorizontalExtraInfo & 0x0F;
			unsigned char HorizontalAddressableVideo_upper = (VesaTiming->HorizontalExtraInfo & 0xF0)>>4;
			unsigned char VerticalBlanking_upper = VesaTiming->VerticalExtraInfo & 0x0F;
			unsigned char VerticalAddressableVideo_upper = (VesaTiming->VerticalExtraInfo & 0xF0)>>4;
			unsigned char VerticalSyncPulseWidth = VesaTiming->VerticalSync & 0x0F;
			unsigned char VerticalFrontPorch = (VesaTiming->VerticalSync & 0xF0)>>4;
			unsigned char VerticalSyncPulseWidth_upper = VesaTiming->HV_Info & 0x3;
			unsigned char VerticalFrontPorch_upper = (VesaTiming->HV_Info & 0x0C)>>2;
			unsigned char HorizontalSyncPulseWidth_upper = (VesaTiming->HV_Info & 0x30)>>4;
			unsigned char HorizontalFrontPorch_upper = (VesaTiming->HV_Info & 0xC0)>>6;

            Timing.PixelClock   = VesaTiming->PixelClock;

            Timing.hSize        = VesaTiming->HorizontalAddressableVideo | (HorizontalAddressableVideo_upper << 8);
            Timing.hBlanking    = VesaTiming->HorizontalBlanking | (HorizontalBlanking_upper << 8);
            Timing.hFrontPorch  = VesaTiming->HorizontalFrontPorch | (HorizontalFrontPorch_upper << 8);
            Timing.hPluseWidth  = VesaTiming->HorizontalSyncPulseWidth | (HorizontalSyncPulseWidth_upper << 8);

            Timing.vSize        = VesaTiming->VerticalAddressableVideo | (VerticalAddressableVideo_upper << 8);
            Timing.vBlanking    = VesaTiming->VerticalBlanking | (VerticalBlanking_upper << 8);
            Timing.vFrontPorch  = VerticalFrontPorch | (VerticalFrontPorch_upper << 4);
            Timing.vPluseWidth  = VerticalSyncPulseWidth | (VerticalSyncPulseWidth_upper << 4);

            if(i==0) DBG_PRINT("========= Preffered Timing =========\r\n");
            else     DBG_PRINT("============ Timing #%d =============\r\n", i);
            DBG_PRINT("Pixel clock = 0x%x\r\n",Timing.PixelClock);
            DBG_PRINT("           Addressable Blanking FrontPorch PulseWidth \r\n");
            DBG_PRINT("Horizontal    %4d        %4d     %4d      %4d    (pixels)\r\n",Timing.hSize,  Timing.hBlanking,   Timing.hFrontPorch, Timing.hPluseWidth);
            DBG_PRINT("Vertical      %4d        %4d     %4d      %4d    (lines)\r\n",Timing.vSize,  Timing.vBlanking,   Timing.vFrontPorch,     Timing.vPluseWidth);
        }
        else if(ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 0xFD) // Display Range Limits
        {
            const VESA_E_EDID_DISPLAY_RANGE_LIMITS *VesaLimits = (const VESA_E_EDID_DISPLAY_RANGE_LIMITS*)ptr;
            int hMaxOffset = 0, hMinOffset = 0;
            int vMaxOffset = 0, vMinOffset = 0;
            if(flag_timing_limits == 0)
            {
                if(VesaLimits->DisplayRangeLimitsOffsets & 1) vMinOffset = 255;
                if(VesaLimits->DisplayRangeLimitsOffsets & 2) vMaxOffset = 255;
                if(VesaLimits->DisplayRangeLimitsOffsets & 4) hMinOffset = 255;
                if(VesaLimits->DisplayRangeLimitsOffsets & 8) hMaxOffset = 255;


                Limits.maxPixelClock    = VesaLimits->MaximumPixelClock * 10;
                Limits.hMaxRate         = VesaLimits->MaximumHorizontalRate + hMaxOffset;
                Limits.hMinRate         = VesaLimits->MinimumHorizontalRate + hMinOffset;
                Limits.vMaxRate         = VesaLimits->MaximumVerticalRate + vMaxOffset;
                Limits.vMinRate         = VesaLimits->MinimumVerticalRate + vMinOffset;

                DBG_PRINT("========= Timing Limits =========\r\n");
                DBG_PRINT("Vertical Rate       = %3d ~ %3d Hz\r\n", Limits.vMinRate, Limits.vMaxRate);
                DBG_PRINT("Horizontal Rate     = %3d ~ %3d kHz\r\n", Limits.hMinRate, Limits.hMaxRate);
                DBG_PRINT("Maximum Pixel Clock = %3d MHz\r\n", Limits.maxPixelClock);
                flag_timing_limits = 1;
            }
        }
        ptr += 18;
    }

    if(flag_timing_limits)
    {
        int val = 0;
        if(flag_supported)
        {
            for(i=0;i<sizeof(HdmiTiming)/sizeof(HdmiTiming[0]);i++)
            {
                if( HdmiTiming[i].PixelClock <= Limits.maxPixelClock * 100 &&
                        HdmiTiming[i].HFreq <= Limits.hMaxRate * 10 &&
                        HdmiTiming[i].HFreq >= Limits.hMinRate * 10 &&
                        HdmiTiming[i].VFreq <= Limits.vMaxRate &&
                        HdmiTiming[i].VFreq >= Limits.vMinRate)
                    val |= 1 << i;
            }
            *flag_supported = val;
        }

        return 0;
    }
    else
    {
        return -1;
    }
}

static INT32S generate_An(INT8U *p_An)
{
    INT8U i;
    for(i=0;i<8;i++)
    {
        *(p_An+i) = (INT8U)R_RANDOM0;
        DBG_PRINT("%02x ",*(p_An+i));
    }
    DBG_PRINT("\r\n");
}

static INT32S set_HDMI_An_reg(INT8U *p_An)
{
    INT32U temp;
    INT8U i;
    temp = 0;
    for(i=0;i<4;i++)
    {
        temp += (*(p_An+i))<<(i*8);
    }
    R_HDCP_AN_Register1 = temp;	// HDCP encrypt cipher AN value ,range bit[31:0]
    temp = 0;
    for(i=0;i<4;i++)
    {
        temp += (*(p_An+i+4))<<(i*8);
    }
    R_HDCP_AN_Register2 = temp;	// HDCP encrypt cipher AN value ,range bit[63:32]
}

static INT32S set_HDMI_km_reg(INT64U km)
{
    INT32U temp32;
    INT64U temp64;
    temp32 = (INT32U)km;
    R_HDCP_KM_Register1 = temp32;	// HDCP encrypt cipher KM value ,range bit[31:0]
    temp64 = km>>32;
    temp32 = (INT32U)temp64;
    R_HDCP_KM_Register2 = temp32;	// HDCP encrypt cipher KM value ,range bit[55:32]

}

static INT64U sum_of_key_calculation(INT8U *Bksv)
{
    INT64U temp;
    INT64U temp2;
    INT64U temp3;
    INT32U i;
    INT8U *select;
    //INT8U sel_idx;
    temp = 0;
    temp += *Bksv;
    temp += (*(Bksv+1))<<8;
    temp += (*(Bksv+2))<<16;
    temp += (*(Bksv+3))<<24;
    temp3 = (*(Bksv+4))<<24;
    temp3 = temp3<<8;
    temp += temp3;
    //DBG_PRINT("select bit 0x%llx\r\n",temp);
    //sel_idx = 0;
    temp2 =0;
    for(i=0;i<40;i++)
    {
        if(temp&0x1)
        {
            //DBG_PRINT("KEY[%d] 0x%llx\r\n",i,facsimile_Key[i]);
            //temp2 += facsimile_Key[i];
        }
        temp = temp>>1;
    }

    temp2 = temp2&0x00FFFFFFFFFFFFFF;
    //DBG_PRINT("sum of key 0x%llx\r\n",temp2);
    return temp2;
}

INT32S ddc_sccb_open(void)
{

	ddc_handle = drv_l2_sccb_open(HDMI_DDC_ID_0, 8, 8);
	if(ddc_handle == 0) {
		DBG_PRINT("ddc Sccb open fail.\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

int hdmi_read_block(INT8U index, INT8U *p,INT16U length)
{
    //ddc_sccb_open();
    //ddc_handle = drv_l2_sccb_open(HDMI_DDC_ID_0, 8, 8);
    drv_l2_sccb_continue_read(ddc_handle,index,p,length);
}

int hdmi_read_EDID(INT8U *p)
{
    INT8U reg = 0;
    INT32U i;
    //ddc_sccb_open();
    //ddc_handle = drv_l2_sccb_open(HDMI_DDC_ID_0, 8, 8);
    drv_l2_sccb_continue_read(ddc_handle,0x00,p,128);
   // for(i=0;i<128;i++)
   // {
     //   drv_l2_sccb_read(ddc_handle,reg,p+i);
     //   reg++;

   // }
    /*
	int ret = 0;
	hHdmiDDCSegment.slaveAddr = HDMI_SEGMENT_ID;
	hHdmiDDCSegment.clkRate = 80;
	hHdmiDDC.slaveAddr = HDMI_DDC_ID_0;
	hHdmiDDC.clkRate = 80;

	drv_l1_i2c_init(0);

	//ret = reg_1byte_data_1byte_write(&hHdmiDDCSegment, 0, 0);
	ret = drv_l1_reg_1byte_data_1byte_write(&hHdmiDDC, 0, 0);
	if (ret < 0) {
		DBG_PRINT("DDC Error!!\r\n");
	}
	hHdmiDDC.slaveAddr = HDMI_DDC_ID_1;
	drv_l1_reg_1byte_data_1byte_write(&hHdmiDDC, 0, 0);

	return drv_l1_i2c_bus_read(&hHdmiDDC, p, 128);
	*/
	return STATUS_OK;
}

static INT32S hdcp_sccb_open(void)
{

	hdcp_handle = drv_l2_sccb_open(HDMI_HDCP_ID_0, 8, 8);
	if(hdcp_handle == 0) {
		DBG_PRINT("hdcp Sccb open fail.\r\n");
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

static int hdmi_write_HDCP_An(INT8U* An)
{
    INT32U reg;
    INT32U i;
    DBG_PRINT("\r\nWrite An\r\n");
    //hdcp_handle = drv_l2_sccb_open(HDMI_HDCP_ID_0, 8, 8);
    drv_l2_sccb_continue_write(hdcp_handle,0x18,An,8);
    /*
    for(i=0;i<4;i++)
    {
        DBG_PRINT("%2x ",reg&0x00FF);
        drv_l2_sccb_write(hdcp_handle,0x18+i,(INT16U)(reg&0x00FF));
        reg >>=8;

    }
    reg = AnH;
    for(i=0;i<4;i++)
    {
        DBG_PRINT("%2x ",reg&0x00FF);
        drv_l2_sccb_write(hdcp_handle,0x1C+i,(INT16U)(reg&0x00FF));
        reg >>=8;
    }
    */
}

static int hdmi_write_HDCP_Aksv(INT8U* Aksv)
{
    INT32U reg;

    DBG_PRINT("\r\nWrite AKsv\r\n");
    //hdcp_handle = drv_l2_sccb_open(HDMI_HDCP_ID_0, 8, 8);
    drv_l2_sccb_continue_write(hdcp_handle,0x10,Aksv,5);
    /*
    reg = KsvH;
    drv_l2_sccb_write(hdcp_handle,0x10,(INT16U)(reg&0x00FF));
    reg = KsvL;
    drv_l2_sccb_write(hdcp_handle,0x14,(INT16U)(reg&0x00FF));
    reg = KsvL>>8;
    drv_l2_sccb_write(hdcp_handle,0x13,(INT16U)(reg&0x00FF));
    reg = KsvL>>16;
    drv_l2_sccb_write(hdcp_handle,0x12,(INT16U)(reg&0x00FF));
    reg = KsvL>>24;
    drv_l2_sccb_write(hdcp_handle,0x11,(INT16U)(reg&0x00FF));
    */
}

static int hdmi_read_HDCP_Ri(INT8U* Ri)
{
    //hdcp_handle = drv_l2_sccb_open(HDMI_HDCP_ID_0, 8, 8);
    drv_l2_sccb_continue_read(hdcp_handle,0x8,Ri,2);
    //drv_l2_sccb_read(hdcp_handle,0x9,&ri_dataL);
    //DBG_PRINT("Ri'%x%x\r\n",ri_dataH,ri_dataL);
}

static int hdmi_read_HDCP_KSV_FIFO(INT8U *p, INT32U length)
{
    drv_l2_sccb_continue_read(hdcp_handle, 0x43, p, length);
}

static int hdmi_read_HDCP_VH(INT8U *p)
{
    drv_l2_sccb_continue_read(hdcp_handle, 0x20, p, 20);
}

static int hdmi_read_HDCP_bcap(INT8U *bcap)
{
    drv_l2_sccb_continue_read(hdcp_handle,0x40,bcap,1);
}

static int hdmi_read_HDCP_bstatus(INT8U *bstatus)
{
    drv_l2_sccb_continue_read(hdcp_handle,0x41,bstatus,2);
}

static int hdmi_read_HDCP_Bksv(INT8U *Bksv)
{
    INT8U ptemp;
    drv_l2_sccb_continue_read(hdcp_handle,0x00,Bksv,5);
    #if 1
    ptemp = *Bksv;
    *Bksv = *(Bksv+4);
    *(Bksv+4) = ptemp;
    ptemp = *(Bksv+1);
    *(Bksv+1) = *(Bksv+3);
    *(Bksv+3) = ptemp;
    #endif
}

static int hdmi_read_HDCP_reg(unsigned char *p)
{
    INT8U reg = 0;
    INT32U i;
    drv_l2_sccb_continue_read(hdcp_handle,0x00,p,128);
    /*
    for(i=0;i<128;i++)
    {
        drv_l2_sccb_read(hdcp_handle,reg,p+i);
        reg++;
    }
    DBG_PRINT("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
    */
    for(i=0;i<128;i++)
    {
        if((i%16)==0)
            DBG_PRINT("\r\n%2x ",i/16);
        DBG_PRINT("%2x ",*(p+i));
    }
    DBG_PRINT("\r\n----\r\n");
}

int drvl2_hdmi_init(unsigned int DISPLAY_MODE, unsigned int AUD_FREQ)
{
	int ret = 0;
	int i;
	unsigned long flag = 0;
	INT8U IsRxRepeater = 0;
	INT8U timeout = 0;
	INT8U EDID[128];
    INT8U HDCPreg[128];
    INT8U HDCP_Ri[2];
    //INT8U HDCP_An[8]={0x0b,0x4A,0xFD,0x19,0xB8,0x43,0xA9,0x7A};
    INT8U HDCP_An[8]={0x03,0x04,0x07,0x0C,0x13,0x1C,0x27,0x34};
    INT8U HDCP_An2[8]={0xe5,0x0f,0xd1,0x3a,0xa5,0x62,0x5e,0x44};
    //INT8U HDCP_An[8]={0x34,0x27,0x1C,0x13,0x0C,0x07,0x04,0x03};
    //INT8U HDCP_An[8];
    //INT8U HDCP_Aksv[5]={0xFE,0x87,0x85,0x22,0xA3};
    INT8U HDCP_Aksv[5]={0xB7,0x03,0x61,0xf7,0x14};
    //INT8U HDCP_Aksv[5]={0x14,0xF7,0x61,0x03,0xB7};
    //INT8U HDCP_Aksv[5]={0x41,0x7F,0x16,0x30,0x7B};
    INT8U HDCP_bcap[1];
    INT8U HDCP_Bksv[5];
    INT8U HDCP_Bksv2[5]={0xcd,0x1a,0xf2,0x1e,0x51};
    INT8U HDCP_Bksv3[5]={0x01,0xf4,0x97,0x26,0xe7};
    INT8U HDCP_Bksv4[5]={0x88,0xD1,0xB1,0xEE,0xB2};
    INT8U HDCP_Bksv5[5]={0xB2,0xEE,0xB1,0xD1,0x88};
    INT8U HDCP_Bksv6[5]={0x2B,0xEE,0x1B,0x1D,0x88};
    INT8U HDCP_bstatus[2];
    INT8U *HDCP_KSV_FIFO;
    INT8U HDCP_VH[20];//={0xB2,0xEE,0xB1,0xD1,0x88};
    //bsvec_t Km[1], REPEATER[1], An[1], Ks[1], R0[1], M0[1];
    //BS_HDCPCipherState hs;
    //REPEATER[0] = 0;
    //An[0] = 0x34271c130c070403;
    //    An[0] = 0x0304070c131c2734;
    //An[0] = 0x445e62a53ad10fe5;
    //{0x88,0xD1,0xB1,0xEE,0xB2}
    INT64U km_get;
    ddc_sccb_open();
	ret = hdmi_read_EDID(EDID);
	osDelay(1);

	if (ret >= 0)
	{	// Read DDC data
		ret = hdmi_check_supported(EDID, &flag, 0x36);
		if (flag == 0) // try again（不知道是電視少送，還是硬體少讀一byte）
			ret = hdmi_check_supported(EDID, &flag, 0x35);
		if ( (DISPLAY_MODE==D1_MODE)&&(flag&HDMI_720X480P60_BIT) ) {
			DBG_PRINT("HDMI_720X480P60_BIT\r\n");
		}
		if ( (DISPLAY_MODE==TFT_1280x720_MODE)&&(flag&HDMI_1280X720P60_BIT) ) {
			DBG_PRINT("HDMI_1280X720P60\r\n");
		}
		if (flag == 0) {
			DBG_PRINT("DDC doesn't contain Monitor Range Limits Information\r\n");
		}
	}
	else
	{
		DBG_PRINT("DDC read error.\r\n");
		return -1;
	}
    osDelay(2000);
	hdcp_sccb_open();
	/*
	for(i=0;i<20;i++){
	hdmi_read_HDCP_bcap(HDCP_bcap);
	}
	*/

	//generate_An(HDCP_An);


	osDelay(2);
    hdmi_read_HDCP_bcap(HDCP_bcap);
	DBG_PRINT("bcap = %x\r\n",HDCP_bcap[0]);
	//osDelay(2);
	hdmi_read_HDCP_bcap(HDCP_bcap);
	DBG_PRINT("bcap = %x\r\n",HDCP_bcap[0]);
	//osDelay(2);
    if(HDCP_bcap[0]&0x40)
    {
        IsRxRepeater = 1;
    }
	//osDelay(5);
	hdmi_write_HDCP_An(HDCP_An);
	osDelay(1);
	hdmi_write_HDCP_Aksv(HDCP_Aksv);
    osDelay(1);
    set_HDMI_An_reg(HDCP_An);
    set_HDMI_km_reg(km_get);

    R_HDMICONFIG |= (0x1<<14)|(0x9<<4)|(0x1<<15);//

	hdmi_read_HDCP_Bksv(HDCP_Bksv);
	osDelay(1);
	DBG_PRINT("bksv = %02x%02x%02x%02x%02x\r\n",HDCP_Bksv[4],HDCP_Bksv[3],HDCP_Bksv[2],HDCP_Bksv[1],HDCP_Bksv[0]);
	//osDelay(5);

    //km_get = sum_of_key_calculation(HDCP_Bksv);
    //DBG_PRINT("km %llx \r\n",km_get);
    //calculate M0 R0 KS
/*
    km_get = sum_of_key_calculation(HDCP_Bksv3);
	DBG_PRINT("km %llx \r\n",km_get);

	km_get = sum_of_key_calculation(HDCP_Bksv2);
	DBG_PRINT("km %llx \r\n",km_get);

    km_get = sum_of_key_calculation(HDCP_Bksv4);
	DBG_PRINT("km %llx \r\n",km_get);

	km_get = sum_of_key_calculation(HDCP_Bksv5);
	DBG_PRINT("km %llx \r\n",km_get);
*/
/*
    Km[0] = sum_of_key_calculation(HDCP_Bksv);
    DBG_PRINT("km %llx \r\n",Km[0]);
    R_IOA_O_DATA |= 0x2000;
    //HDCPBlockCipher(1, Km, REPEATER, An, &hs, Ks, R0, M0); //it spend 50msec
    R_IOA_O_DATA &= ~0x2000;
    DBG_PRINT("Ks %llx \r\n",Ks[0]);
    DBG_PRINT("An %llx \r\n",An[0]);
    DBG_PRINT("R0 %llx \r\n",R0[0]);
    DBG_PRINT("M0 %llx \r\n",M0[0]);
    osDelay(150);
    */
/*
	km_get = sum_of_key_calculation(HDCP_Bksv3);
	DBG_PRINT("km %llx \r\n",km_get);

	km_get = sum_of_key_calculation(HDCP_Bksv2);
	DBG_PRINT("km %llx \r\n",km_get);

    km_get = sum_of_key_calculation(HDCP_Bksv4);
	DBG_PRINT("km %llx \r\n",km_get);

	km_get = sum_of_key_calculation(HDCP_Bksv5);
	DBG_PRINT("km %llx \r\n",km_get);
	*/

	//sum_of_key_calculation();
	//ret = hdmi_read_HDCP_reg(HDCPreg);
	//osDelay(2);
    //while(1);


    hdmi_read_HDCP_Ri(HDCP_Ri);
    DBG_PRINT("HDCP_Ri = %02x%02x\r\n",HDCP_Ri[1],HDCP_Ri[0]);
    osDelay(1);
    hdmi_read_HDCP_Ri(HDCP_Ri);
    DBG_PRINT("HDCP_Ri = %02x%02x\r\n",HDCP_Ri[1],HDCP_Ri[0]);
    osDelay(1);
    hdmi_read_HDCP_Ri(HDCP_Ri);
    DBG_PRINT("HDCP_Ri = %02x%02x\r\n",HDCP_Ri[1],HDCP_Ri[0]);
    osDelay(1);
    hdmi_read_HDCP_Ri(HDCP_Ri);
    DBG_PRINT("HDCP_Ri = %02x%02x\r\n",HDCP_Ri[1],HDCP_Ri[0]);
    osDelay(1);
    //read Bcap
    if(IsRxRepeater == 1)
    {
        HDCP_bcap[0] = 0;
        while((HDCP_bcap[0]&0x20)==0)
        {
            hdmi_read_HDCP_bcap(HDCP_bcap);
            DBG_PRINT("bcap = %x\r\n",HDCP_bcap[0]);
            osDelay(1000);
            timeout++;
            if(timeout > RETRY_CNT)
            {
                goto __Authenticated_failed;
            }
        }

        //read Bstatus
        hdmi_read_HDCP_bstatus(HDCP_bstatus);
        DBG_PRINT("device count = %x\r\n",HDCP_bstatus[0]);
        HDCP_KSV_FIFO = (INT8U*)gp_malloc_align((HDCP_bstatus[0]&0x3F)*5,4);
        hdmi_read_HDCP_KSV_FIFO(HDCP_KSV_FIFO,(HDCP_bstatus[0]&0x3F)*5);

        hdmi_read_HDCP_bstatus(HDCP_bstatus);
        //SHA-1 transfrom (KSV list || Bstatus || M0)
        hdmi_read_HDCP_VH(HDCP_VH);
        //compare V' = V
    }
	while(1)
	{
        //calculate Ri conti.
        hdmi_read_HDCP_Ri(HDCP_Ri);
        DBG_PRINT("HDCP_Ri = %02x%02x\r\n",HDCP_Ri[1],HDCP_Ri[0]);
        osDelay(1000);
	}
__Authenticated_failed:
    return 0;
	//return drvl1_hdmi_init(DISPLAY_MODE, AUD_FREQ);
}

//#endif	// _DRV_L2_HDMI

