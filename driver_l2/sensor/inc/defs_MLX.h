
#define MLX90640_SLAVE_ADDR	    0x33 // SA = 0X33

#define C_ISR_TEST_PIN				IO_D9  //0x08			// IOA10




#define	WRITE_CMD				0x00
#define	READ_CMD				0x00



#define CONVERT_WAIT_TIME		15

//mode[in]:i2c restart without stop or not:
// 1:I2C_RESTART_WITHOUT_STOP 0:I2C_RESTART_WITH_STOP

#define MXL_I2C_RESTART_MODE	    1
#define ReadBlock0				0x09
#define Read_VDD_MEAS_Block0	0x0D

#define COLOR_TABLE_OVR_RoomTemp 		20 // 大於 室溫 3 度 
#define COLOR_TABLE_UNDER_RoomTemp 		60 // 低於 室溫 6 度 
#define NOISE_OVR_RoomTemp 				20 // 大於 室溫 3 度 


#define MLX_LINE 32
//#define MLX_rowNumEnd_32 32
#define MLX_COLUMN 24
//#define MLX_PixelEighth 128
#define MLX_Pixel 768				//=32x24
//#define ELAMOUNT 256
//#define MAXNROFDEFECTS	5




#define MLX90640_AdrDevID 				0x2407
#define MLX90640_AdrStatus				0x8000
#define MLX90640_AdrControlRegister1 	0x800D
#define MLX90640_AdrConfig 				0x800F

#define MLX90640_EEAddrRegister1 		0x240C
#define MLX90640_EEAddrRegister2		0x240D
#define MLX90640_EEAddrConfig 			0x240E
#define MLX90640_EEAddrInternal_I2C		0x240F

#define MLX90640_EEAddrstart 		0x2400
#define MLX90640_EEMemAddrRead 		832
#define MLX90640_RAM_AddrRead 		832
//
//	增加 
//	frameData[832] = controlRegister1;
//  frameData[833] = statusRegister & 0x0001;
//	配合 demo code API
//
#define MLX90640_frameDataSize		834		//
#define MLX90640_RAMAddrstart 		0x0400

#define MLX90640_RESOLUTION_16B 		0x00
#define MLX90640_RESOLUTION_17B 		0x01
#define MLX90640_RESOLUTION_18B 		0x02	//default
#define MLX90640_RESOLUTION_19B 		0x03

#define MLX90640_REFRESH_RATE_0P5HZ 	0x00
#define MLX90640_REFRESH_RATE_1HZ	 	0x01
#define MLX90640_REFRESH_RATE_2HZ	 	0x02	//default
#define MLX90640_REFRESH_RATE_4HZ	 	0x03
#define MLX90640_REFRESH_RATE_8HZ	 	0x04
#define MLX90640_REFRESH_RATE_16HZ	 	0x05
#define MLX90640_REFRESH_RATE_32HZ	 	0x06
#define MLX90640_REFRESH_RATE_64HZ	 	0x07


#ifndef _MLX640_API_H_
#define _MLX640_API_H_

  typedef struct	paramsMLX90640_s
    {
        INT16 kVdd;
        INT16 vdd25;
        float KvPTAT;
        float KtPTAT;
        INT16U vPTAT25;
        float alphaPTAT;
        INT16 gainEE;
        float tgc;
        float cpKv;
        float cpKta;
        INT8U resolutionEE;
        INT8U calibrationModeEE;
        float KsTa;
        float ksTo[4];
        INT16 ct[4];
        float alpha[768];
        INT16 offset[768];
        float kta[768];
        float kv[768];
        float cpAlpha[2];
        INT16 cpOffset[2];
        float ilChessC[3];
        INT16U brokenPixels[5];
        INT16U outlierPixels[5];
    } paramsMLX90640_t;
#endif


