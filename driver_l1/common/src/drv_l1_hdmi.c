/*
* Purpose: HDMI driver/interface
*
* Author: josephhsieh
*
* Date: 2013/11/21
*
* Copyright Generalplus Corp. ALL RIGHTS RESERVED.
*
* Version : 1.00
* History :
*/

//Include files
#include "string.h"
#include "drv_l1.h"
#include "drv_l1_sfr.h"
#include "drv_l1_hdmi.h"

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#if (defined _DRV_L1_HDMI) && (_DRV_L1_HDMI == 1)                   //
//================================================================//

/////////////////// TFT2 Setting ///////////////////
#define TFT2_EN                        (1<<0)
#define TFT2_CLK_SEL               (7<<1)
#define TFT2_MODE                    (0xF<<4)
#define TFT2_VSYNC_INV          (1<<8)
#define TFT2_HSYNC_INV          (1<<9)
#define TFT2_DCLK_INV             (1<<10)
#define TFT2_DE_INV                 (1<<11)
#define TFT2_H_COMPRESS        (1<<12)
#define TFT2_MEM_BYTE_EN      (1<<13)
#define TFT2_INTERLACE_MOD  (1<<14)
#define TFT2_VSYNC_UNIT         (1<<15)

#define TFT2_REG_POL             (1<<4)
#define TFT2_REG_REV             (1<<5)
#define TFT2_UD_I                   (1<<6)
#define TFT2_RL_I                    (1<<7)
#define TFT2_DITH_EN             (1<<8)
#define TFT2_DITH_MODE        (1<<9)
#define TFT2_DATA_MODE        (3<<10)
#define TFT2_SWITCH_EN         (1<<12)
#define TFT2_GAMMA_EN          (1<<13)
#define TFT2_DCLK_SEL            (3<<14)
#define TFT2_DCLK_DELAY       (7<<18)
#define TFT2_SLIDE_EN            (1<<21)

#define TFT2_MODE_UPS051         0x0
#define TFT2_MODE_UPS052         0x10
#define TFT2_MODE_CCIR656        0x20
#define TFT2_MODE_PARALLEL      0x30
#define TFT2_MODE_TCON             0x40

#define TFT2_MODE_MEM_CMD_WR     0x80
#define TFT2_MODE_MEM_CMD_RD      0x90
#define TFT2_MODE_MEM_DATA_WR   0xa0
#define TFT2_MODE_MEM_DATA_RD    0xb0
#define TFT2_MODE_MEM_CONTI         0xc0
#define TFT2_MODE_MEM_ONCE           0xd0

#define TFT2_DATA_MODE_8         0x0
#define TFT2_REG_POL_1LINE       0x0
#define TFT2_REG_POL_2LINE       0x1
#define TFT2_NEW_POL_EN           0x80

#define TFT2_DCLK_SEL_0            0x0000
#define TFT2_DCLK_SEL_90          0x4000
#define TFT2_DCLK_SEL_180        0x8000
#define TFT2_DCLK_SEL_270        0xC000

#define TFT2_DCLK_DELAY_0        0x000000
#define TFT2_DCLK_DELAY_1        0x040000
#define TFT2_DCLK_DELAY_2        0x080000
#define TFT2_DCLK_DELAY_3        0x0C0000
#define TFT2_DCLK_DELAY_4        0x100000
#define TFT2_DCLK_DELAY_5        0x140000
#define TFT2_DCLK_DELAY_6        0x180000
#define TFT2_DCLK_DELAY_7        0x1C0000

#define TFT2_CLK_DIVIDE_1        0x0
#define TFT2_CLK_DIVIDE_2        0x2
#define TFT2_CLK_DIVIDE_3        0x4
#define TFT2_CLK_DIVIDE_4        0x6
#define TFT2_CLK_DIVIDE_5        0x8
#define TFT2_CLK_DIVIDE_6        0xA
#define TFT2_CLK_DIVIDE_7        0xC
#define TFT2_CLK_DIVIDE_8        0xE
#define TFT2_CLK_DIVIDE_9        0x10
#define TFT2_CLK_DIVIDE_10       0x12
#define TFT2_CLK_DIVIDE_11       0x14
#define TFT2_CLK_DIVIDE_12       0x16
#define TFT2_CLK_DIVIDE_13       0x18
#define TFT2_CLK_DIVIDE_14       0x1A
#define TFT2_CLK_DIVIDE_15       0x1C
#define TFT2_CLK_DIVIDE_16       0x1E
#define TFT2_CLK_DIVIDE_17       0x20
#define TFT2_CLK_DIVIDE_18       0x22
#define TFT2_CLK_DIVIDE_19       0x24
#define TFT2_CLK_DIVIDE_20       0x26
#define TFT2_CLK_DIVIDE_21       0x28
#define TFT2_CLK_DIVIDE_22       0x2A
#define TFT2_CLK_DIVIDE_23       0x2C
#define TFT2_CLK_DIVIDE_24       0x2E
#define TFT2_CLK_DIVIDE_25       0x30
#define TFT2_CLK_DIVIDE_26       0x32
#define TFT2_CLK_DIVIDE_27       0x34
#define TFT2_CLK_DIVIDE_28       0x36
#define TFT2_CLK_DIVIDE_29       0x38
#define TFT2_CLK_DIVIDE_30       0x3A
#define TFT2_CLK_DIVIDE_31       0x3C
#define TFT2_CLK_DIVIDE_32       0x3E

// for H264 scaler is enabled
static INT32U	g_hdmi_img_width = 0;		// Image width after H264 scaling down
static INT32U	g_hdmi_img_height = 0;		// Image height after H264 scaling down
static INT32U	g_hdmi_img_offset = 0;		// If you don't know how to setup, set it to zero.
static INT32U  g_system_cpuclk_ctrl_bak = 0;  // backup register R_SYSTEM_CPUCLK_CTRL (0xD000007C)

static void tft2_clk_set(INT32U clk)
{
    R_TFT2_CTRL &= ~TFT2_CLK_SEL;
    R_TFT_TS_MISC &= ~0xC0;

    if (clk < TFT2_CLK_DIVIDE_9)
    {
        R_TFT2_CTRL |= clk;
    }
    else
    {
        R_TFT2_CTRL |= clk & 0xF;
        R_TFT_TS_MISC |= (clk & 0x20) << 1;
        R_TFT_TS_MISC |= (clk & 0x10) << 3;
    }
}

static void tft2_data_mode_set(INT32U mode)
{
    R_TFT_TS_MISC &= ~TFT2_DATA_MODE;
    R_TFT_TS_MISC |= mode;
}

static void tft2_mode_set(INT32U mode)
{
    R_TFT2_CTRL &= ~TFT2_MODE;
    R_TFT2_CTRL |= mode;
}

static void tft2_signal_inv_set(INT32U mask, INT32U value)
{
    /*set vsync,hsync,dclk and DE inv */
    R_TFT2_CTRL &= ~mask;
    R_TFT2_CTRL |= (mask & value);
}

static void tft2_scaling_up_size_set(INT32U hdmi_width, INT32U hdmi_height)
{
    // setup TFT2 scaler, scale up to hdmi_widthxhdmi_height
    INT32U W_factor, H_factor;

    W_factor = 65536 * g_hdmi_img_width / hdmi_width;
    if (W_factor>=65536)
    {
        W_factor = 65535;
    }

    H_factor = 65536 * g_hdmi_img_height / hdmi_height;
    if (H_factor>=65536)
    {
        H_factor = 65535;
    }
    R_HDMI_INTP_OFFSET = g_hdmi_img_offset;
    R_HDMI_INTP_FACTOR =  (H_factor<<16) | W_factor;
}


/////////////////// HDMI Setting ///////////////////
#define HAL_HDMI_PKT_HVLD				0x10000000
#define HAL_HDMI_PKT_VVLD				0x20000000
#define HAL_HDMI_PKT_AUTO				0x40000000
#define HAL_HDMI_PKT_SEND				0x80000000

#define HDMI_ACR_PACKET					0x01
#define HDMI_AUDIOSAMPLE_PACKET			0x02
#define HDMI_GENERALCTRL_PACKET			0x03
#define HDMI_ACP_PACKET					0x04
#define HDMI_ISRC1_PACKET				0x05
#define HDMI_ISRC2_PACKET				0x06
#define HDMI_1BITAUDIOSAMPLE_PACKET		0x07
#define HDMI_DSTAUDIO_PACKET			0x08
#define HDMI_HBRAUDIOSTREAM_PACKET		0x09
#define HDMI_GAMUTMETADATA_PACKET		0x0a
#define HDMI_VENDORINFOFRAME_PACKET		0x81
#define HDMI_AVIINFOFRAME_PACKET		0x82
#define HDMI_SPDINFOFRAME_PACKET		0x83
#define HDMI_AUDIOINFOFRAME_PACKET		0x84
#define HDMI_MPEGINFOFRAME_PACKET		0x85

#define HDMI_VS0	0
#define HDMI_VS1	1
#define HDMI_VS2	2
#define HDMI_VS3	3

enum Sampling
{
    Sampling_22K = 0x4000000,
    Sampling_44K = 0x0000000,
    Sampling_88K = 0x8000000,
    Sampling_176K = 0xC000000,
    Sampling_24K = 0x6000000,
    Sampling_48K = 0x2000000,
    Sampling_96K = 0xA000000,
    Sampling_192K = 0xE000000,
    Sampling_32K = 0x3000000,
    Sampling_none = 0x1000000,
    Sampling_768K = 0x9000000
};

enum Original_sampling
{
    Original_sampling_no_match = 0x00,
    Original_sampling_16K = 0x80,
    Original_sampling_Reserved1 = 0x40,
    Original_sampling_32K = 0xC0,
    Original_sampling_12K = 0x20,
    Original_sampling_11K = 0xA0,
    Original_sampling_8K = 0x60,
    Original_sampling_Reserved2 = 0xE0,
    Original_sampling_192K = 0x10,
    Original_sampling_24K = 0x90,
    Original_sampling_96K = 0x50,
    Original_sampling_48K = 0xD0,
    Original_sampling_176K = 0x30,
    Original_sampling_22K = 0xB0,
    Original_sampling_88K = 0x70,
    Original_sampling_44K = 0xF0
};


typedef struct
{
    volatile unsigned int dispHDMIPacketHD;	    /* 0x0100 ~ 0x0103 */
    volatile unsigned int dispHDMIPacketBD[7];	/* 0x0104 ~ 0x0107 */
} HDMI_PKT;

typedef struct HDMI_Reg_s
{
    volatile HDMI_PKT HdmiPkt[4];			/* 0x0100 ~ 0x017f */
    volatile unsigned int dispHDMIAudioN;	        /* 0x0180 ~ 0x0183 */
    volatile unsigned int dispHDMIAudioCTS;	    /* 0x0184 ~ 0x0187 */
    volatile unsigned int dispHDMIAudioSample;	/* 0x0188 ~ 0x018b */
    volatile unsigned int dispHDMIGCtrlPacket;	/* 0x018c ~ 0x018f */
    volatile unsigned int dispHDMITimeCycle;	    /* 0x0190 ~ 0x0193 */
    volatile unsigned int dispHDMIConfig;		    /* 0x0194 ~ 0x0197 */
} HDMI_Reg_t;

typedef struct AUD_CH_Reg_s
{
    volatile unsigned int status0;
    volatile unsigned int status1;
    volatile unsigned int status2;
    volatile unsigned int status3;
} AUD_CH_Reg_t;

typedef struct
{
    unsigned char type;
    unsigned char version;
    unsigned char length;
    unsigned char flag;
    unsigned char CheckSum;
    unsigned char Body[27];
} HDMI_PACKET;

static void hdmi_packet_checksum(void *data)
{
    HDMI_PACKET *Pkt = (HDMI_PACKET *)(data);
    INT8U *body = (INT8U *)(Pkt->Body);
    int CheckSum = 0, i;

    CheckSum -= Pkt->type + Pkt->version + Pkt->length;
    for(i=0; i<Pkt->length; i++)
        CheckSum -= *body++;
    Pkt->CheckSum = (INT8U)(CheckSum & 0xFF);
}


static void hdmi_avi_info_pkt(HDMI_PACKET *Pkt, INT8U VID)
{
    memset((void*)Pkt, 0, sizeof(HDMI_PACKET));
    Pkt->type = HDMI_AVIINFOFRAME_PACKET;
    Pkt->version = 2;
    Pkt->length = 13;
    Pkt->Body[0] = 0x12;	//
    Pkt->Body[1] = 0x08;	// Active Portion Aspect Ratio
    Pkt->Body[2] = 0x00;
    Pkt->Body[3] = VID;	// VideoIdCode
    Pkt->Body[4] = 0x00;	// PixelRep
    Pkt->Body[5] = 0x00;
    Pkt->Body[6] = 0x00;
    Pkt->Body[7] = 0x00;
    Pkt->Body[8] = 0x00;
    Pkt->Body[9] = 0x00;
    Pkt->Body[10] = 0x00;
    Pkt->Body[11] = 0x00;
    Pkt->Body[12] = 0x00;
}

static void hdmi_vendor_info_frame_pkt(HDMI_PACKET *Pkt)
{
    memset((void*)Pkt, 0, sizeof(HDMI_PACKET));
    Pkt->type = HDMI_VENDORINFOFRAME_PACKET;
    Pkt->version = 1;
    Pkt->length = 4;
    Pkt->Body[0] = 0x03;
    Pkt->Body[1] = 0x0C;
    Pkt->Body[2] = 0x00;
    Pkt->Body[3] = 0x00;
}

static void hdmi_spd_info_frame_pkt(HDMI_PACKET *Pkt)
{
    memset((void*)Pkt, 0, sizeof(HDMI_PACKET));
    Pkt->type = HDMI_SPDINFOFRAME_PACKET;
    Pkt->version = 1;
    Pkt->length = 25;
    Pkt->Body[0] = 0x47;
    Pkt->Body[1] = 0x50;
    Pkt->Body[2] = 0x6C;
    Pkt->Body[3] = 0x75;
    Pkt->Body[4] = 0x73;
    Pkt->Body[5] = 0x00;
    Pkt->Body[6] = 0x00;
    Pkt->Body[7] = 0x00;
    Pkt->Body[8] = 0x47;
    Pkt->Body[9] = 0x50;
    Pkt->Body[10] = 0x4D;
    Pkt->Body[11] = 0x50;
    Pkt->Body[12] = 0x38;
    Pkt->Body[13] = 0x33;
    Pkt->Body[14] = 0x30;
    Pkt->Body[15] = 0x30;
    Pkt->Body[16] = 0x20;
    Pkt->Body[17] = 0x50;
    Pkt->Body[18] = 0x4D;
    Pkt->Body[19] = 0x50;
    Pkt->Body[20] = 0x00;
    Pkt->Body[21] = 0x00;
    Pkt->Body[22] = 0x00;
    Pkt->Body[23] = 0x00;
    Pkt->Body[24] = 0x0D;
}

static void hdmi_audio_info_frame_pkt(HDMI_PACKET *Pkt)
{
    memset((void*)Pkt, 0, sizeof(HDMI_PACKET));
    Pkt->type = HDMI_AUDIOINFOFRAME_PACKET;
    Pkt->version = 1;
    Pkt->length = 10;
    Pkt->Body[0] = 0x00;
    Pkt->Body[1] = 0x00;
    Pkt->Body[2] = 0x00;
    Pkt->Body[3] = 0x1F;
    Pkt->Body[4] = 0x00;
    Pkt->Body[5] = 0x00;
    Pkt->Body[6] = 0x00;
    Pkt->Body[7] = 0x00;
    Pkt->Body[8] = 0x00;
    Pkt->Body[9] = 0x00;
}

static void hdmi_audio_channel_status(void *RegBase, INT32U sampling_rate, INT32U org_sampling_frequency)
{
    AUD_CH_Reg_t *AUD_ch = (AUD_CH_Reg_t*)RegBase;

    AUD_ch->status0 = 0x00120014 | sampling_rate;
    AUD_ch->status1 = 0x02 | org_sampling_frequency;
    AUD_ch->status2 = 0x00220014 | sampling_rate;
    AUD_ch->status3 = 0x02 | org_sampling_frequency;
}

void drvl1_hdmi_send_packet(void *RegBase, INT32U ch, void *data, INT32U blank, INT32U sendMode)
{
    HDMI_Reg_t *pHDMI_Reg = (HDMI_Reg_t*)RegBase;
    INT32U *tmp = (INT32U *)data;

    hdmi_packet_checksum(data);

    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketBD[0] = tmp[1];
    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketBD[1] = tmp[2];
    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketBD[2] = tmp[3];
    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketBD[3] = tmp[4];
    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketBD[4] = tmp[5];
    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketBD[5] = tmp[6];
    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketBD[6] = tmp[7];
    pHDMI_Reg->HdmiPkt[ch].dispHDMIPacketHD = (tmp[0] & 0xFFFFFF) | blank | sendMode;
}

void drvl1_hdmi_set_general_ctrl_packet(void *RegBase)
{
    HDMI_Reg_t *pHDMI_Reg = (HDMI_Reg_t*)RegBase;
    pHDMI_Reg->dispHDMIGCtrlPacket = 0x0400;	// display mode
}

void drvl1_hdmi_set_acr_packet(void *RegBase, INT32U N, INT32U CTS)
{
    HDMI_Reg_t *pHDMI_Reg = (HDMI_Reg_t*)RegBase;
    INT32S ACR_STEP = 128, ACR_EN = 1;

    // Both V-Blank and H-Blank are available to transfer ACR packets may blocking other packets.
    pHDMI_Reg->dispHDMIAudioN = HAL_HDMI_PKT_AUTO | HAL_HDMI_PKT_VVLD | (ACR_STEP << 20) | N;
    pHDMI_Reg->dispHDMIAudioCTS = (ACR_EN << 31) | CTS;
}

void drvl1_hdmi_set_audio_sample_packet(void *RegBase, INT32U ch)
{
    HDMI_Reg_t *pHDMI_Reg = (HDMI_Reg_t*)RegBase;

    pHDMI_Reg->dispHDMIAudioSample = 0x001;	//  Only support 2ch (1 subpacket)
}

void drvl1_hdmi_config_phy(INT32U phy1, INT32U phy2)
{
    R_HDMITXPHYCONFIG1 = phy1;
    R_HDMITXPHYCONFIG2 = phy2;
}

void drvl1_hdmi_set_time_cycle(void *RegBase, INT32U VBack, INT32U HBlank )
{
    HDMI_Reg_t *pHDMI_Reg = (HDMI_Reg_t*)RegBase;

    pHDMI_Reg->dispHDMITimeCycle = (VBack << 12) | HBlank;
}


/**
 * @brief   The function is called after HDMI initialization. When it is called, both DAC of GPM4xxA and HDMI device will output the same sound.
 * @param   status: on/off HDMI audio
 * @return   HDMI_OK: Function is successful. HDMI_FAIL: Function is fail.
 */
INT32S drvl1_hdmi_audio_ctrl(INT32U status)
{
    if (status)
        R_HDMI_AUD_CTRL |= 0x1;			// Audio on
    else
        R_HDMI_AUD_CTRL &= (~0x1);			// Audio off
    return HDMI_OK;
}


/**
 * @brief      It will turn off DAC of GPM4xxA¡Abut audio of HDMI device is still functioning properly.
 * @param   status: mute DAC of GPM4xxA
 * @return   HDMI_OK: Function is successful. HDMI_FAIL: Function is fail.
 */
INT32S drvl1_hdmi_dac_mute(INT32U status)
{

    if (status)
        R_HDMI_AUD_CTRL |= 0x40;			// DAC off
    else
        R_HDMI_AUD_CTRL &= (~0x40);		// DAC on
    return HDMI_OK;
}

void drv_l1_hdmi_arb_init(void)
{
	R_NF_SHARE_DELAY = 0x98; //DRAM arbiter priority
    //DMA SDRAM priority:     0x19
    (*(volatile INT32U *)(0xD02000DC)) = 0xA6; //DRAM arbiter priority
    //CPU SDRAM priority:     0x1a
    (*(volatile INT32U *)(0xD02000E0)) = 0xA7; //DRAM arbiter priority
    R_MEM_SDRAM_CTRL1 &= 0xFFFFDFFF; //DRAM CAS latency from 3 --> 2
}

/**
 * @brief   Initiate video and audio block in HDMI hardware macro.
 * @param   vid_mode: resolution of display
 * @param   aud_mode: resolution of audio
 * @return   HDMI_OK: Function is successful. HDMI_FAIL: Function is fail.
 */
INT32S drvl1_hdmi_init(INT32U vid_mode, INT32U aud_mode)
{
    HDMI_PACKET	Pkt;
    INT32U vback=0, hblank=0;		// video
    INT32U phy1=0, phy2=0, config=0;
    INT32S ret = HDMI_FAIL;

    drv_l1_hdmi_arb_init();

    if (g_system_cpuclk_ctrl_bak!=0)
    {
    		DBG_PRINT("HDMI plug out abnormally.\r\n");
	       drvl1_hdmi_exit();
    }
    g_system_cpuclk_ctrl_bak = R_SYSTEM_CPUCLK_CTRL;

    // initial HDMI
    R_SYSTEM_CLK_EN0 |= 0x8;		// HDMI clock enable
    R_TFT2_CTRL = 0;
    tft2_signal_inv_set(TFT2_VSYNC_INV|TFT2_HSYNC_INV, TFT2_VSYNC_INV|TFT2_HSYNC_INV);
    tft2_mode_set(TFT2_MODE_PARALLEL);		//  HDMI must  set "PARALLEL" mode
    tft2_data_mode_set(TFT2_DATA_MODE_8);
    tft2_clk_set(TFT2_CLK_DIVIDE_1); /* FS=66 */
    R_SYSTEM_HDMI_CTRL = 0;

    switch (vid_mode)
    {
        case HDMI_VID_480P:
            // 720 X 480
            R_TFT2_CTRL			&=  (~0xF0000);		// clear mode
            //R_TFT2_CTRL			|=  0xB0000;		// 480P
            R_TFT2_CTRL			|=  0x90000;		// 480P
            // TFT2 Registers
            R_TFT2_HS_WIDTH 		= 3;				//	1		=HPW
            R_TFT2_H_PERIOD 		= 858-1;	//	1560	=HPW+HBP+HDE+HFP
            R_TFT2_VS_WIDTH 		= 0;				//	1		=VPW				(DCLK)
            R_TFT2_V_PERIOD 		= 524; 	//	262 	=VPW+VBP+VDE+VFP	(LINE)
            //R_TFT2_LINE_RGB_ORDER	 = 0x00;
            // HDMI
            R_TFT2_V_START = 0x0009002D; 	// Vsync start of HDMI( < Vynsc end of HDMI)
            R_TFT2_V_END = 0x000F020D-1;	// Vsync end of HDMI ( < P_TFT2_V_START_END[15:0])
            R_TFT2_H_START = 0x000F0089; 	// Hsync start of HDMI( < Hsync end of HDMI)
            R_TFT2_H_END = 0x004C0358; 		// Hsync end of HDMI ( < P_TFT2_H_START_END[15:0])

	     R_SYSTEM_CPUCLK_CTRL &= ~0x7F00;   // CPUCLK reset
	     R_SYSTEM_CPUCLK_CTRL |= 0x0C00;	// 270MHz
	     R_SYSTEM_CPUCLK_CTRL |= 0x4000;	// CPUCLK enable
            R_SYSTEM_HDMI_CTRL |= 0xC900;	// HDMI_CLK = 270/10=27MHz
            // HDMI hard marco
            hdmi_avi_info_pkt(&Pkt, 0x02);      // 480P
            config &= (~0x3000);   // H sync, V sync
            config |= 0x400;
            hblank = 0x8A;
            vback = 0x96d2;
            phy1 = 0x10270201;
            phy2 = 0x00000010;

            // HDMI scaling up
            if ( (g_hdmi_img_width!=0) && (g_hdmi_img_height!=0) )
            {
                tft2_scaling_up_size_set(720, 480);
            }

            ret = HDMI_OK;
            break;

        case HDMI_VID_720P:
            // 1280 X 720
            R_TFT2_CTRL			       &=  (~0xF0000);		// clear mode
            //R_TFT2_CTRL			       |=  0x30000;		// 720P
            R_TFT2_CTRL			       |=  0x10000;		// 720P
            // TFT2 Registers
            R_TFT2_HS_WIDTH = 3;
            R_TFT2_H_PERIOD	= 1650-1;
            R_TFT2_VS_WIDTH = 2;
            R_TFT2_V_PERIOD	= 750-1;
           // R_TFT2_LINE_RGB_ORDER	 = 0x00;
            // HDMI
            R_TFT2_V_START 	= 0x0005001E;//5<<16(HDMI_VSYNC_START) + 30;
            R_TFT2_V_END	= 0x000A02ED;//10<<16(HDMI_VSYNC_END) + 749;
            R_TFT2_H_START 	= 0x006D0171;//109<<16(HDMI_HSYNC_START) + 369;
            R_TFT2_H_END	= 0x00940670;//148<<16(HDMI_HSYNC_END) + 1648;

	     R_SYSTEM_CPUCLK_CTRL &= ~0x7F00;   // CPUCLK reset
	     R_SYSTEM_CPUCLK_CTRL |= 0x1E00;	// 594MHz
	     R_SYSTEM_CPUCLK_CTRL |= 0x4000;	// CPUCLK enable
            R_SYSTEM_HDMI_CTRL |= 0xC700;	// HDMI_CLK = 297/4=74.25MHz
            // HDMI hard marco
            hdmi_avi_info_pkt(&Pkt, 0x04);      // 720P
            config |= 0x3000;	// H sync INV, V sync INV
            config |= 0x0400;
            hblank = 0x172;
            vback = 0x9600;
            phy1 = 0x10270201;
            phy2 = 0x00000010;

            // HDMI scaling up
            if ( (g_hdmi_img_width!=0) && (g_hdmi_img_height!=0) )
            {
                tft2_scaling_up_size_set(1280, 720);
            }

            ret = HDMI_OK;
            break;

	  case HDMI_VID_1080P:
            // 1920 X 1080
            R_TFT2_CTRL			       &=  (~0xF0000);		// clear mode
            R_TFT2_CTRL			       |=  0x70000;		// 1080P
            // TFT2 Registers
            R_TFT2_HS_WIDTH = 3;
            R_TFT2_H_PERIOD	= 2200-1;
            R_TFT2_VS_WIDTH = 2;
            R_TFT2_V_PERIOD	= 1125-1;
          //  R_TFT2_LINE_RGB_ORDER	 = 0x00;
            // HDMI
            R_TFT2_V_START 	= 0x0004002D; // 4<<16(HDMI_VSYNC_START) + 45;
            R_TFT2_V_END	= 0x00090465; //9<<16(HDMI_VSYNC_END) + 1125;
            R_TFT2_H_START 	= 0x00570117;//88<<16(HDMI_HSYNC_START) + 280;
            R_TFT2_H_END	= 0x00820896;//132<<16(HDMI_HSYNC_END) + 2200;

	     R_TFT2_VS_START	= 1;
	     R_TFT2_VS_END = 0x80008000; //HDMI_V_FACTOR[31:16]=540*65536/1080  HDMI_H_FACTOR[15:0]=960*65536/1920
	     R_TFT2_HS_START 	= 1;
	     R_TFT2_HS_END	= 260;


	     R_SYSTEM_CPUCLK_CTRL &= ~0x7F00;   // CPUCLK reset
	     R_SYSTEM_CPUCLK_CTRL |= 0x1E00;	// 594MHz
	     R_SYSTEM_CPUCLK_CTRL |= 0x4000;	// CPUCLK enable
            R_SYSTEM_HDMI_CTRL |= 0xC300;	// HDMI_CLK = 594M/4=148.5MHz
            // HDMI hard marco
            hdmi_avi_info_pkt(&Pkt, 0x10);      // 1080P
            config |= 0x3000;	// H sync INV, V sync INV
            config |= 0x400;
            hblank = 0x118;
            vback = 0x182B8;
            phy1 = 0x10670201;
            phy2 = 0x00000030;

            // HDMI scaling up
            if ( (g_hdmi_img_width!=0) && (g_hdmi_img_height!=0) )
            {
                tft2_scaling_up_size_set(1920, 1080);
            }

            ret = HDMI_OK;
	     break;

        default:
            ret = HDMI_FAIL;
    }

    ////////////////////////  HDMI Registers////////////////////////////
    R_SYSTEM_CKGEN_CTRL |= 0x1C;		// for HDMI video and audio clock
    //(*((volatile INT32U *) 0xD05000F0)) |= 0x1; // TV_EN will trigger TV interrupt
    //(*((volatile INT32U *) 0xC01B0000)) &= 0x01;	// dst_sel moves to another place.

    R_TFT2_CTRL |= 0x10000;		// TFT ON  (HDMI macro won't have HDMI_CLK unless TFT_ON is enabled)
    // default:0x10331

    R_HDMICONFIG = config;
    drvl1_hdmi_config_phy(phy1, phy2);
    // hdmi avi info frame
    drvl1_hdmi_send_packet((void*)P_HDMI_PKT_BASE, HDMI_VS0, (void*)(&Pkt), HAL_HDMI_PKT_VVLD, HAL_HDMI_PKT_AUTO);
    drvl1_hdmi_set_time_cycle((void*)P_HDMI_PKT_BASE, vback, hblank);
    // hdmi vendor info frame
    hdmi_vendor_info_frame_pkt(&Pkt);
    drvl1_hdmi_send_packet((void*)P_HDMI_PKT_BASE, HDMI_VS1, (void*)(&Pkt), HAL_HDMI_PKT_VVLD, HAL_HDMI_PKT_AUTO);
    // hdmi spd info frame
    hdmi_spd_info_frame_pkt(&Pkt);
    drvl1_hdmi_send_packet((void*)P_HDMI_PKT_BASE, HDMI_VS2, (void*)(&Pkt), HAL_HDMI_PKT_VVLD, HAL_HDMI_PKT_AUTO);
    // hdmi audio info frame
    hdmi_audio_info_frame_pkt(&Pkt);
    drvl1_hdmi_send_packet((void*)P_HDMI_PKT_BASE, HDMI_VS3, (void*)(&Pkt), HAL_HDMI_PKT_VVLD, HAL_HDMI_PKT_AUTO);
    // hdmi general ctrl frame
    drvl1_hdmi_set_general_ctrl_packet((void*)P_HDMI_PKT_BASE);
    // hdmi audio sample frame
    drvl1_hdmi_set_audio_sample_packet((void*)P_HDMI_PKT_BASE, 2);

    R_SYSTEM_APLL_FREQ &= (~0xF00);
    R_SYSTEM_CODEC_CTRL1 &= (~0xFFF);
	switch(aud_mode)
    {
        case HDMI_AUD_32K:
            // hdmi audio acr frame
            R_SYSTEM_APLL_FREQ |= 0xA00;			   // using 73.728M Hz
            R_SYSTEM_CODEC_CTRL1 |= 0x2FF;		//(2303/3);		   // HDMI_MCLK_DIV
            drvl1_hdmi_set_acr_packet((void*)P_HDMI_PKT_BASE, 4096, 74250);	// auto CTS
            // "Channel Status Bit"  information
            hdmi_audio_channel_status((void *)P_HDMI_AUD_CH_INFO_BASE, Sampling_32K, Original_sampling_32K);
            break;
        case HDMI_AUD_44K:
            // hdmi audio acr frame
            R_SYSTEM_APLL_FREQ |= 0x900;			   // using 67.7376M Hz
            R_SYSTEM_CODEC_CTRL1 |= 0x1FF;	// (1536/3);		   // HDMI_MCLK_DIV
            drvl1_hdmi_set_acr_packet((void*)P_HDMI_PKT_BASE, 6272, 82500);	// auto CTS
            // "Channel Status Bit"  information
            hdmi_audio_channel_status((void *)P_HDMI_AUD_CH_INFO_BASE, Sampling_44K, Original_sampling_44K);
            break;
        case HDMI_AUD_48K:
        default:
            // hdmi audio acr frame
            R_SYSTEM_APLL_FREQ = 0;
            R_SYSTEM_APLL_FREQ |= 0xA34;		// using 73.728M Hz
            R_SYSTEM_CODEC_CTRL1 |= 0x204;//0x1FF;//(1536/3);		   // HDMI_MCLK_DIV

            drvl1_hdmi_set_acr_packet((void*)P_HDMI_PKT_BASE, 6144, 74250);	// auto CTS
            // "Channel Status Bit"  information
            hdmi_audio_channel_status((void *)P_HDMI_AUD_CH_INFO_BASE, Sampling_48K, Original_sampling_48K);
    }


    //////////////////////////////////////////////////////
    R_HDMI_EN = 0x90000000;		// bit[31:30] must be 2'b10
    R_HDMI_IRQ_STATUS = 0xFFFF;	// clear HDMI interrupt

    R_HDCP_AN_Register1 = 0x0;	// HDCP encrypt cipher AN value ,range bit[31:0]
    R_HDCP_AN_Register2 = 0x0;	// HDCP encrypt cipher AN value ,range bit[63:32]
    R_HDCP_KM_Register1 = 0x0;	// HDCP encrypt cipher KM value ,range bit[31:0]
    R_HDCP_KM_Register2 = 0x0;	// HDCP encrypt cipher KM value ,range bit[55:32]

    R_TFT2_CTRL |= 0x1;		// TFT ON,  PPU enable => HDMI is enabled now.

    return ret;
}

void drv_l1_hdmi_audio_sample_rate_set(INT32U aud_mode)
{
    R_SYSTEM_APLL_FREQ &= (~0xF00);
    R_SYSTEM_CODEC_CTRL1 &= (~0xFFF);
	switch(aud_mode)
    {
        case HDMI_AUD_32K:
            // hdmi audio acr frame
            R_SYSTEM_APLL_FREQ |= 0xA00;			   // using 73.728M Hz
            R_SYSTEM_CODEC_CTRL1 |= 0x2FF;		//(2303/3);		   // HDMI_MCLK_DIV
            drvl1_hdmi_set_acr_packet((void*)P_HDMI_PKT_BASE, 4096, 74250);	// auto CTS
            // "Channel Status Bit"  information
            hdmi_audio_channel_status((void *)P_HDMI_AUD_CH_INFO_BASE, Sampling_32K, Original_sampling_32K);
            break;
        case HDMI_AUD_44K:
            // hdmi audio acr frame
            R_SYSTEM_APLL_FREQ |= 0x900;			   // using 67.7376M Hz
            R_SYSTEM_CODEC_CTRL1 |= 0x1FF;	// (1536/3);		   // HDMI_MCLK_DIV
            drvl1_hdmi_set_acr_packet((void*)P_HDMI_PKT_BASE, 6272, 82500);	// auto CTS
            // "Channel Status Bit"  information
            hdmi_audio_channel_status((void *)P_HDMI_AUD_CH_INFO_BASE, Sampling_44K, Original_sampling_44K);
            break;
        case HDMI_AUD_48K:
        default:
            // hdmi audio acr frame
            R_SYSTEM_APLL_FREQ = 0;
            R_SYSTEM_APLL_FREQ |= 0xA34;		// using 73.728M Hz
            R_SYSTEM_CODEC_CTRL1 |= 0x204;//0x1FF;//(1536/3);		   // HDMI_MCLK_DIV

            drvl1_hdmi_set_acr_packet((void*)P_HDMI_PKT_BASE, 6144, 74250);	// auto CTS
            // "Channel Status Bit"  information
            hdmi_audio_channel_status((void *)P_HDMI_AUD_CH_INFO_BASE, Sampling_48K, Original_sampling_48K);
    }
}

/**
 * @brief   Disconnect HDMI device and turn off HDMI clock.
 * @param   none
 * @return   HDMI_OK: Function is successful. HDMI_FAIL: Function is fail.
 */
INT32S drvl1_hdmi_exit(void)
{
    // disable clock source
    // 148MHz maybe used by SPI (Don't disable)
    // disable audio
    R_HDMI_AUD_CTRL &= (~0x41);
    // disable video
    R_TFT2_CTRL &= (~0x1);
    // disable PHY power
    R_HDMITXPHYCONFIG1 &= (~0x1);       // HDMI disable
    // disable HDMI clock enable
    R_SYSTEM_CLK_EN0 &= (~0x8);
    R_SYSTEM_CPUCLK_CTRL = g_system_cpuclk_ctrl_bak;
    g_system_cpuclk_ctrl_bak = 0;
    {        // waiting CPUPLL stable
		INT32U i;
		for (i=0;i<0x100;++i)
			R_RANDOM0 = i;
    }

    return HDMI_OK;
}


/**
 * @brief      If video frame size of H264 are smaller than you want to play(480P/720P/1080P). Enable this function, It will scale up H264 video frame to appropriate size.
 * @param   width of H264 frame
 * @param   height of H264 frame
 * @param   image offset (Usually set 0)
 * @return   HDMI_OK: Function is successful. HDMI_FAIL: Function is fail.
 */
INT32S drvl1_hdmi_h264_scaling_enable(INT32U img_width, INT32U img_height, INT32U img_offset)
{
    g_hdmi_img_width = img_width;
    g_hdmi_img_height = img_height;
    g_hdmi_img_offset = img_offset;

    return HDMI_OK;
}


/**
 * @brief      Disable scaling H264 frame.
 * @param   none
 * @return   HDMI_OK: Function is successful. HDMI_FAIL: Function is fail.
 */
INT32S drvl1_hdmi_h264_scaling_disable(void)
{
    g_hdmi_img_width = 0;
    g_hdmi_img_height = 0;
    g_hdmi_img_offset = 0;

    return HDMI_OK;
}

//=== This is for code configuration DON'T REMOVE or MODIFY it ===//
#endif //(defined _DRV_L1_HDMI) && (_DRV_L1_HDMI == 1)                   //
//================================================================//

