#ifndef __drv_l1_H264_H__
#define __drv_l1_H264_H__
/*#define REG_INT_MASK            (*(volatile INT32U *)(0xD1500008))
#define REG_INT_STATUS          (*(volatile INT32U *)(0xD150000C))
#define REG_H264_CONTROL        (*(volatile INT32U *)(0xD1500014))
#define REG_FRAME_DATA_SIZE     (*(volatile INT32U *)(0xD150001C))
#define REG_PICTURE_TYPE        (*(volatile INT32U *)(0xD1500020))
#define REG_QP                  (*(volatile INT32U *)(0xD1500024))
#define REG_MISC_SETTING        (*(volatile INT32U *)(0xD1500028))
#define REG_IP_VERSION          (*(volatile INT32U *)(0xD150002C))
#define REG_PROFILE_LEVEL       (*(volatile INT32U *)(0xD1500030))
#define REG_BITSTREAM_STARADDR  (*(volatile INT32U *)(0xD1500034))
#define REG_BITSTREAM_BASEADDR  (*(volatile INT32U *)(0xD1500038))
#define REG_BITSTREAM_MAXADDR   (*(volatile INT32U *)(0xD150003C))
#define REG_RAW_FRAME_ADDR      (*(volatile INT32U *)(0xD1500040))
#define REG_AR_TABLE_ADDR       (*(volatile INT32U *)(0xD150004C))

#define REG_REC_LUMA_ADDR       (*(volatile INT32U *)(0xD1500060))
#define REG_REC_CHROMA_ADDR     (*(volatile INT32U *)(0xD1500064))
#define REG_REC_MB_Y_OFFSET     (*(volatile INT32U *)(0xD1500068))
#define REG_REF_MB_Y_OFFSET     (*(volatile INT32U *)(0xD150006C))
#define REG_MAX_MB_Y_OFFSET     (*(volatile INT32U *)(0xD1500070))

#define REG_ME_WINDOW           (*(volatile INT32U *)(0xD1500090))
#define REG_FUNCTION_CONTROL    (*(volatile INT32U *)(0xD1500094))

#define REG_BITSTREAM_LEN       (*(volatile INT32U *)(0xD1500098))
#define REG_STATUS0             (*(volatile INT32U *)(0xD150009C))
#define REG_STATUS1             (*(volatile INT32U *)(0xD15000A0))
#define REG_GLOBAL_DEBUG0       (*(volatile INT32U *)(0xD15000A4))
#define REG_GLOBAL_DEBUG1       (*(volatile INT32U *)(0xD15000A8))
#define REG_FRAME_CROP_OFFSET   (*(volatile INT32U *)(0xD15000B0))
#define REG_BITSTREAM_BYPASSLEN (*(volatile INT32U *)(0xD15000B4))
#define REG_BITSTREAM_LEN_NOSEI (*(volatile INT32U *)(0xD15000B8))*/

//0x14
#define BIT_ENC_TRIGGER         0x1
#define BIT_DEC_TRIGGER         0x2
#define BIT_CODEC_ENGINE_RESET  0x10

#define REF_FRAME       0x0
#define NON_REF_FRAME   0x2
#define I_FRAME         0x1
#define P_FRAME         0x0

#define PPS_EN          0x1
#define SPS_EN          0x2
#define IDR_FLAG        0x4

#define NEW_OP_2_QUANT_TABLE    (0x1 << 26)
#define I16_COST_TWEAK          (0x1 << 25)
#define FOURCE_INTRA_AS_SCN_CHG (0x1 << 24)

#define FRAME_SKIP              (0x1 << 30)

typedef struct H264Enc_Reg_s
{
  INT32U FrameDataSize;         //0x1c
  INT32U PictureType;           //0x20
  INT32U QP;                    //0x24
  INT32U MiscSetting;           //0x28
  INT32U ProfileLevel;          //0x30
  INT32U BitstreamStartAddr;    //0x34
  INT32U RawFrameAddr;          //0x40
  INT32U RecLumaAddr;           //0x60
  INT32U RecChromaAddr;         //0x64
  INT32U RecMbYOffset;          //0x68
  INT32U RefMbYOffset;          //0x6c
  INT32U MaxMbYOffset;          //0x70
  INT32U MEDynamicSearch;       //0x90
  INT32U FunctionControl;       //0x94
  INT32U FrameCropOffset;       //0xb0
} H264Enc_Reg_t;

typedef struct h264_buffers
{
  INT32U LumaAddr0;
  INT32U ChromaAddr0;
  INT32U LumaAddr;
  INT32U ChromaAddr;
  INT32U RecMBYOffset;
  INT32U RefMBYOffset;
  INT32U MB2ScanYOffset;
  INT32U NotUsedMBYOffset;
  INT32U MaxMBYOffset;
  INT32U DivOutLumaAddr0;
  INT32U DivOutChromaAddr0;
  INT32U DivOutLumaAddr[3];
  INT32U DivOutChromaAddr[3];
  INT16U DivOutIdx;
} h264_buffers_t;

void H264Enc_EnableClock();
void H264Enc_DisableClock();
void H264Dec_EnableClock();
void H264Dec_DisableClock();
void H264_ResetEngine();
void H264Enc_Trigger();
void H264_Wait_Complete();
int H264_SetFrameSize(int width, int height);
void H264Enc_SetFrame_I(int idr_pic_id, int ref, int sps_pps);
void H264Enc_SetFrame_P(int poc, int ref);
void H264Enc_SetFrame_Skip(int poc);
int H264_GetFrameType();
int H264_SetQP(int qp);
int H264_GetQP();
void H264_SetLevel(int level);
void H264_GetLevel(int * level);
void H264_SetFrameAddr(INT32U addr);
void H264Enc_SetBitstreamAddr(INT32U addr);
void H264Dec_SetBitstreamAddr(INT32U addr, INT32U base, INT32U max);
void H264Enc_SetMEDynamicSearchWindow(int rangeX, int rangeY);
void H264_SetFrameCropOffset(int left, int right, int top, int bottom);
void H264_GetFrameCropOffset(int* left, int* right, int* top, int* bottom);
INT32U H264_GetBistreamLength();
INT32U H264Enc_InitBuffers(int width, int height);
void H264Enc_UpdateBuffers(INT32U param, int update_offset);
INT32U H264Dec_InitBuffers(int width, int height, int out_div);
void H264Dec_UpdateBuffers(INT32U param, int update_offset);
void H264_ReleaseBuffers(INT32U param);
void H264Dec_GetOutputBuffer(INT32U *LumaAddr, INT32U *ChromaAddr);
void H264_DumpRegister();
void H264Dec_MB2ScanReset();
void H264Dec_MB2Scan(int HDMI, INT32U mb_offset);
void H264Dec_MB2ScanDiv(int HDMI, INT32U LumaAddr, INT32U ChromaAddr, INT32U div);
#endif
