//
//	sensor_MXL_Thermopile
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board_config.h"
#include "drv_l1_sfr.h"
//#include "drv_l1_csi.h"
#include "drv_l1_i2c.h"
#include "drv_l2_sensor.h"
//#include "drv_l2_csi.h"
#include "drv_l2_sccb.h"
#include "gp_aeawb.h"
#include "defs_MLX.h"
#include <math.h>		// for pow()
//#include "avi_encoder_app.h" // for MLX90640_GetVdd

//_SENSOR_MLX90640_THERMOPILE


#if (defined _SENSOR_MXL90640_THERMOPILE) && (_SENSOR_MXL90640_THERMOPILE == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/


// MXL sccb interface
#define SCCB_GPIO		0
#define SCCB_HW_I2C		1
#define MXL_SCCB_MODE		SCCB_HW_I2C //SCCB_HW_I2C//SCCB_GPIO

//#define NULL			0
#define STATUS_OK		0
#define STATUS_FAIL		-1

#define RETURN(x) {nRet = x; goto Return;}
#if 0
#define DEBUG_MSG(x)	{}
#else
#define DEBUG_MSG(x)	{x;}
#endif

const INT16U DELAYTIME_at_REFRESH_RATE2 [8]={2000,1000,500,250,125,63,32,15};
const INT8U MLX_REFRESH_RATE_HZ2 [8]={0,1,2,4,8,16,32,64};


/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/

extern MLX_TH32x24Para_t *pMLX_TH32x24_Para;
extern paramsMLX90640_t *pMLX90640_Para;

//MLX_TH32x24Para_t *pMLX_TH32x24_Para;
//paramsMLX90640_t *pMLX90640_Para;

MLX_TH32x24Para_t MLX_TH32x24_Para; // , *pMLX_TH32x24_Para;
paramsMLX90640_t MLX90640_Para;     //,*pMLX90640_Para;

/**************************************************************************
*				 F U N C T I O N	D E C L A R A T I O N S 			  *
**************************************************************************/
static INT32S MXL_sccb_open(void);
static void MXL_sccb_close(void);
static INT32S MXL_sccb_write(INT8U reg, INT8U value);
static INT32S MXL_sccb_read(INT8U reg, INT8U *value);
static INT32S MLX_TH32x24_mem_alloc(void);	//davis
INT32S MLX90640_SetResolution(INT8U resolution);
INT32S MLX90640_SetRefreshRate(INT8U refreshRate);
int CheckEEPROMValid(INT16U *pMLX32x24_READ_INT16U_buf);






#if MXL_SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t MXL_handle;
#endif

//paramsMLX90640_t	MLX_Para,*pMLX_Para;		// 2019.05.07 davis

static INT32S MXL_sccb_open(void)
{

	INT8U value;
#if MXL_SCCB_MODE == SCCB_GPIO

	//MXL_handle = drv_l2_sccb_open(MXL_EEPROM_ID, 8, 8);
	if(MXL_handle == 0) {
		DBG_PRINT(" MXL_EEPROM_ID Sccb open fail.\r\n");
		return STATUS_FAIL;
	}

#elif MXL_SCCB_MODE == SCCB_HW_I2C



	MXL_handle.devNumber = I2C_1;
    MXL_handle.slaveAddr = MLX90640_SLAVE_ADDR<<1;

    MXL_handle.clkRate = 950;
    drv_l1_i2c_init(MXL_handle.devNumber);

	DBG_PRINT(" MLX90640_SLAVE_ADDR Sccb open in HW_I2C.\r\n");




#endif
	return STATUS_OK;
}

static void MXL_sccb_close(void)
{
#if MXL_SCCB_MODE == SCCB_GPIO
	if(MXL_handle) {
		drv_l2_sccb_close(MXL_handle);
		MXL_handle = NULL;
	}
#elif MXL_SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_1);
	MXL_handle.slaveAddr = 0;
	MXL_handle.clkRate = 0;
#endif
}

static INT32S MXL_sccb_write(INT8U reg, INT8U value)
{
#if MXL_SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(MXL_handle, reg, value);

#elif MXL_SCCB_MODE == SCCB_HW_I2C
	INT8U data[2];

	data[0] = reg;
	data[1] = value;
	return drv_l1_i2c_bus_write(&MXL_handle, data, 2);
#endif
}

#if 1
static INT32S MXL_sccb_read(INT8U reg, INT8U *value)
{
#if MXL_SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(MXL_handle, reg, &data) >= 0) {
		*value = (INT8U)data;
		return STATUS_OK;
	} else {
		*value = 0xFF;
		return STATUS_FAIL;
	}
#elif MXL_SCCB_MODE == SCCB_HW_I2C
	INT8U data[1];

	data[0] = reg;
	if(drv_l1_i2c_bus_write(&MXL_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&MXL_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}
#endif


static INT32S MLX_TH32x24_mem_alloc(void)	//davis
{
	INT32U buffer_addr;
	INT32S buffer_size, nRet;
	INT8U  tmpN1,tmpN2;


	//pMLX_TH32x24_Para->MLX_TH32x24_width = bufA;
    //pMLX_TH32x24_Para->MLX_TH32x24_height = bufB;

	
	DBG_PRINT("davis --> pMLX_TH32x24_Para->MLX_TH32x24_width = %d pMLX_TH32x24_Para->MLX_TH32x24_height = %d \r\n", 
		pMLX_TH32x24_Para->MLX_TH32x24_width,pMLX_TH32x24_Para->MLX_TH32x24_height);

//	MLX32x24_EE_READ_8bitBUF
	buffer_size = MLX90640_EEMemAddrRead * 2;

	buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF = buffer_addr;
		DBG_PRINT("davis --> MLX32x24_EE_READ_8bitBUF = 0x%x\r\n", pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF);

//	MLX32x24_EE_READ_16bitBUF
		buffer_size = MLX90640_EEMemAddrRead*2;

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
			if(buffer_addr == 0) {
				RETURN(STATUS_FAIL);
			}
			pMLX_TH32x24_Para->MLX32x24_EE_READ_16bitBUF = buffer_addr;
			DBG_PRINT("davis --> MLX32x24_EE_READ_16bitBUF = 0x%x\r\n", pMLX_TH32x24_Para->MLX32x24_EE_READ_16bitBUF);


	// 	1 pixel takes 2 bytes => 32*24 pixel requires 32*24*2
	buffer_size = pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height << 1;

	buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
	//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
	if(buffer_addr == 0) {
		RETURN(STATUS_FAIL);
	}
	pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr = buffer_addr;
	DBG_PRINT("davis --> MLX_TH32x24_ColorOutputFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_ColorOutputFrame_addr);

	for(tmpN1=0; tmpN1<MLX_TH32x24_SCALERUP_BUFFER_NO; tmpN1++) {
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_TmpOutput_format_addr[tmpN1] = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_TmpOutput_format_addr[%d] = 0x%x\r\n",tmpN1, pMLX_TH32x24_Para->MLX_TH32x24_TmpOutput_format_addr[tmpN1]);
	}


	for(tmpN1=0;tmpN1<AVG_buf_len;tmpN1++){
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_avg_buf_addr[tmpN1] = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_avg_buf_addr[%d] addr = 0x%x\r\n",tmpN1, pMLX_TH32x24_Para->MLX_TH32x24_avg_buf_addr[tmpN1]);
	}

	/*
		buffer_size = pAviEncVidPara->display_width * pAviEncVidPara->display_height << 1;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_display_frame = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_display_frame addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_display_frame);


		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_display_background_frame = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_display_background_frame addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_display_background_frame);
	*/

	buffer_size = sizeof(INT16U)*MAXNROFDEFECTS ;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_BadPixAdr_buf = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_BadPixAdr_buf(INT16U) addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_BadPixAdr_buf);

	buffer_size = sizeof(INT8U)*MAXNROFDEFECTS ;
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_BadPixMask_buf = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_BadPixMask_buf(INT8U) addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_BadPixMask_buf);


	buffer_size = IMAGE_DATA_INT32S_SIZE * MLX_Pixel ; // float/INT32S = 4 byte
	for(tmpN1 = 0 ; tmpN1 < IMG_AVG_buf_len ; tmpN1++){
		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);	}
		pMLX_TH32x24_Para->MLX_TH32x24_ImgAvg_buf_addr[tmpN1] = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_ImgAvg_buf_addr[%d] addr = 0x%x\r\n",tmpN1, pMLX_TH32x24_Para->MLX_TH32x24_ImgAvg_buf_addr[tmpN1]);
	}

		//	1 pixel takes 2 bytes => 32*24 pixel requires 32*24*3*3 放大3倍 
		buffer_size = (pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height << 1)*ScaleUp_3*ScaleUp_3;

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX_TH32x24_ScaleUpFrame_addr = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_ScaleUpFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_ScaleUpFrame_addr);

		//	1 pixel takes 1 bytes (Gray out) => 32*24 pixel requires 32*24*3*3 放大3倍 
		buffer_size = (pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height )*ScaleUp_3*ScaleUp_3;

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX_TH32x24_GrayScaleUpFrame_addr = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_GrayScaleUpFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_GrayScaleUpFrame_addr);



		//	1 pixel takes 1 bytes (Gray out) => 32*24 pixel requires 32*24
		buffer_size = (pMLX_TH32x24_Para->MLX_TH32x24_width * pMLX_TH32x24_Para->MLX_TH32x24_height );

		buffer_addr = (INT32U) gp_malloc_align(buffer_size , 32);
		//buffer_addr = (INT32U) gp_malloc_align(buffer_size , 64);  // 64 ?
		if(buffer_addr == 0) {
			RETURN(STATUS_FAIL);
		}
		pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFrame_addr = buffer_addr;
		DBG_PRINT("davis --> MLX_TH32x24_GrayOutputFrame_addr = 0x%x\r\n", pMLX_TH32x24_Para->MLX_TH32x24_GrayOutputFrame_addr);

	nRet = STATUS_OK;
Return:
	return nRet;
}




INT32S MLX90640_SetResolution(INT8U resolution)
{
    INT32S error;
	INT16U value;

	INT16U 	EEcopy16BIT[1]={0},*pEEcopy16BIT;
	INT8U	resolutionRAM;

	pEEcopy16BIT = EEcopy16BIT;

	value = (resolution & 0x03) << 10;

	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,pEEcopy16BIT);

	if(error > 0)
    {
        resolutionRAM = (EEcopy16BIT[0] & 0x0C00) >> 10;
			DBG_PRINT("GetResolution value = 0x%04X \r\n",resolutionRAM);

		value = (EEcopy16BIT[0] & 0xF3FF ) | value;
		//DBG_PRINT("value = 0x%04X \r\n",value);

		error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrControlRegister1,value);
	}

    return error;
}


INT32S MLX90640_SetRefreshRate(INT8U refreshRate)
{
    INT32S error;
	INT16U value;

	INT16U 	EEcopy16BIT[1]={0},*pEEcopy16BIT;
	INT8U	refreshRateRAM;

	pEEcopy16BIT = EEcopy16BIT;

	value = (refreshRate & 0x07)<<7;

	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,pEEcopy16BIT);

	if(error > 0)
    {
        refreshRateRAM = (EEcopy16BIT[0] & 0x0380) >> 7;
			DBG_PRINT("GetRefreshRate value = 0x%04X \r\n",refreshRateRAM);

		value = (EEcopy16BIT[0] & 0xFC7F ) | value;
		//DBG_PRINT("value = 0x%04X \r\n",value);

		error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrControlRegister1,value);
	}

    return error;
}

int CheckEEPROMValid(INT16U	*pMLX32x24_READ_INT16U_buf)
{
	int deviceSelect;


	DBG_PRINT("pMLX32x24_READ_INT16U_buf addr=0x%0x \r\n", pMLX32x24_READ_INT16U_buf);
	deviceSelect = *(pMLX32x24_READ_INT16U_buf+10) & 0x0040;
	if(deviceSelect == 0)
	{
		return 0;
	}

	return -7;
}

void ExtractVDDParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    int16_t kVdd;
    int16_t vdd25;

    kVdd =*(pMLX32x24_READ_INT16U_buf+51);  //eeData[51];

    kVdd = (*(pMLX32x24_READ_INT16U_buf+51) & 0xFF00) >> 8;
    if(kVdd > 127)
    {
        kVdd = kVdd - 256;
    }
    kVdd = 32 * kVdd;
    vdd25 = *(pMLX32x24_READ_INT16U_buf+51) & 0x00FF;
    vdd25 = ((vdd25 - 256) << 5) - 8192;

	mlx90640->kVdd = kVdd;
    mlx90640->vdd25 = vdd25;

}


void ExtractPTATParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;

    KvPTAT =(*(pMLX32x24_READ_INT16U_buf+50)  & 0xFC00) >> 10;
    if(KvPTAT > 31)
    {
        KvPTAT = KvPTAT - 64;
    }
    KvPTAT = KvPTAT/4096;

    KtPTAT = *(pMLX32x24_READ_INT16U_buf+50) & 0x03FF;
    if(KtPTAT > 511)
    {
        KtPTAT = KtPTAT - 1024;
    }
    KtPTAT = KtPTAT/8;

    vPTAT25 = *(pMLX32x24_READ_INT16U_buf+49);

    alphaPTAT = (*(pMLX32x24_READ_INT16U_buf+16) & 0xF000) / pow(2, (double)14) + 8.0f;

    mlx90640->KvPTAT = KvPTAT;
    mlx90640->KtPTAT = KtPTAT;
    mlx90640->vPTAT25 = vPTAT25;
    mlx90640->alphaPTAT = alphaPTAT;

}

void ExtractGainParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    int16_t gainEE;

    gainEE = *(pMLX32x24_READ_INT16U_buf+48);
    if(gainEE > 32767)
    {
        gainEE = gainEE -65536;
    }

    mlx90640->gainEE = gainEE;
}

//------------------------------------------------------------------------------

void ExtractTgcParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    float tgc;
    tgc = *(pMLX32x24_READ_INT16U_buf+60) & 0x00FF;
    if(tgc > 127)
    {
        tgc = tgc - 256;
    }
    tgc = tgc / 32.0f;

    mlx90640->tgc = tgc;
}

//------------------------------------------------------------------------------

void ExtractResolutionParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    uint8_t resolutionEE;
    resolutionEE = (*(pMLX32x24_READ_INT16U_buf+56) & 0x3000) >> 12;

    mlx90640->resolutionEE = resolutionEE;
}

//------------------------------------------------------------------------------

void ExtractKsTaParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    float KsTa;
    KsTa = (*(pMLX32x24_READ_INT16U_buf+60) & 0xFF00) >> 8;
    if(KsTa > 127)
    {
        KsTa = KsTa -256;
    }
    KsTa = KsTa / 8192.0f;

    mlx90640->KsTa = KsTa;
}

//------------------------------------------------------------------------------

void ExtractKsToParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    int KsToScale;
    int8_t step;
	int i;

    step = ((*(pMLX32x24_READ_INT16U_buf+63) & 0x3000) >> 12) * 10;

    mlx90640->ct[0] = -40;
    mlx90640->ct[1] = 0;
    mlx90640->ct[2] = (*(pMLX32x24_READ_INT16U_buf+63)& 0x00F0) >> 4;
    mlx90640->ct[3] = (*(pMLX32x24_READ_INT16U_buf+63) & 0x0F00) >> 8;

    mlx90640->ct[2] = mlx90640->ct[2]*step;
    mlx90640->ct[3] = mlx90640->ct[2] + mlx90640->ct[3]*step;

    KsToScale = (*(pMLX32x24_READ_INT16U_buf+63) & 0x000F) + 8;

	DBG_PRINT("KsToScale = %d, step = %d \r\n",KsToScale,step);
    KsToScale = 1 << KsToScale;

    mlx90640->ksTo[0] = *(pMLX32x24_READ_INT16U_buf+61) & 0x00FF;
    mlx90640->ksTo[1] = (*(pMLX32x24_READ_INT16U_buf+61) & 0xFF00) >> 8;
    mlx90640->ksTo[2] = *(pMLX32x24_READ_INT16U_buf+62) & 0x00FF;
    mlx90640->ksTo[3] = (*(pMLX32x24_READ_INT16U_buf+62) & 0xFF00) >> 8;


    for(i = 0; i < 4; i++)
    {
        if(mlx90640->ksTo[i] > 127)
        {
            mlx90640->ksTo[i] = mlx90640->ksTo[i] -256;
        }
        mlx90640->ksTo[i] = mlx90640->ksTo[i] / KsToScale;
    }
}

//------------------------------------------------------------------------------

void ExtractAlphaParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    int accRow[24];
    int accColumn[32];
    int p = 0;
    int alphaRef;
    uint8_t alphaScale;
    uint8_t accRowScale;
    uint8_t accColumnScale;
    uint8_t accRemScale;

	int i,j;

    accRemScale =*(pMLX32x24_READ_INT16U_buf+32) & 0x000F;
    accColumnScale = (*(pMLX32x24_READ_INT16U_buf+32) & 0x00F0) >> 4;
    accRowScale = (*(pMLX32x24_READ_INT16U_buf+32) & 0x0F00) >> 8;
    alphaScale = ((*(pMLX32x24_READ_INT16U_buf+32) & 0xF000) >> 12) + 30;
    alphaRef = *(pMLX32x24_READ_INT16U_buf+33);

    for( i = 0; i < 6; i++)
    {
        p = i * 4;
        accRow[p + 0] = (*(pMLX32x24_READ_INT16U_buf+34+i) & 0x000F);
        accRow[p + 1] = (*(pMLX32x24_READ_INT16U_buf+34+i) & 0x00F0) >> 4;
        accRow[p + 2] = (*(pMLX32x24_READ_INT16U_buf+34+i) & 0x0F00) >> 8;
        accRow[p + 3] = (*(pMLX32x24_READ_INT16U_buf+34+i) & 0xF000) >> 12;
    }

    for(i = 0; i < 24; i++)
    {
        if (accRow[i] > 7)
        {
            accRow[i] = accRow[i] - 16;
        }
    }

    for( i = 0; i < 8; i++)
    {
        p = i * 4;
        accColumn[p + 0] = (*(pMLX32x24_READ_INT16U_buf+40+i) & 0x000F);
        accColumn[p + 1] = (*(pMLX32x24_READ_INT16U_buf+40+i) & 0x00F0) >> 4;
        accColumn[p + 2] = (*(pMLX32x24_READ_INT16U_buf+40+i) & 0x0F00) >> 8;
        accColumn[p + 3] = (*(pMLX32x24_READ_INT16U_buf+40+i) & 0xF000) >> 12;
    }

    for(i = 0; i < 32; i ++)
    {
        if (accColumn[i] > 7)
        {
            accColumn[i] = accColumn[i] - 16;
        }
    }

    for(i = 0; i < 24; i++)
    {
        for(j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            mlx90640->alpha[p] = (*(pMLX32x24_READ_INT16U_buf+64+p) & 0x03F0) >> 4;
            if (mlx90640->alpha[p] > 31)
            {
                mlx90640->alpha[p] = mlx90640->alpha[p] - 64;
            }
            mlx90640->alpha[p] = mlx90640->alpha[p]*(1 << accRemScale);
            mlx90640->alpha[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + mlx90640->alpha[p]);
            mlx90640->alpha[p] = mlx90640->alpha[p] / pow(2,(double)alphaScale);
        }
    }
	DBG_PRINT("alphaScale = %d, accRowScale = %d ,accColumnScale = %d,accRemScale=%d ,alphaRef=%d \r\n",
		alphaScale,accRowScale,accColumnScale,accRemScale,alphaRef);
}

//------------------------------------------------------------------------------

void ExtractOffsetParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    int occRow[24];
    int occColumn[32];
    int p = 0;
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;

	int i,j;

    occRemScale = (*(pMLX32x24_READ_INT16U_buf+16) & 0x000F);
    occColumnScale = (*(pMLX32x24_READ_INT16U_buf+16) & 0x00F0) >> 4;
    occRowScale = (*(pMLX32x24_READ_INT16U_buf+16) & 0x0F00) >> 8;
    offsetRef = *(pMLX32x24_READ_INT16U_buf+17);
    if (offsetRef > 32767)
    {
        offsetRef = offsetRef - 65536;
    }

    for( i = 0; i < 6; i++)
    {
        p = i * 4;
        occRow[p + 0] = (*(pMLX32x24_READ_INT16U_buf+18+i) & 0x000F);
        occRow[p + 1] = (*(pMLX32x24_READ_INT16U_buf+18+i) & 0x00F0) >> 4;
        occRow[p + 2] = (*(pMLX32x24_READ_INT16U_buf+18+i) & 0x0F00) >> 8;
        occRow[p + 3] = (*(pMLX32x24_READ_INT16U_buf+18+i) & 0xF000) >> 12;
    }

    for( i = 0; i < 24; i++)
    {
        if (occRow[i] > 7)
        {
            occRow[i] = occRow[i] - 16;
        }
    }

    for( i = 0; i < 8; i++)
    {
        p = i * 4;
        occColumn[p + 0] = (*(pMLX32x24_READ_INT16U_buf+24+i) & 0x000F);
        occColumn[p + 1] = (*(pMLX32x24_READ_INT16U_buf+24+i) & 0x00F0) >> 4;
        occColumn[p + 2] = (*(pMLX32x24_READ_INT16U_buf+24+i) & 0x0F00) >> 8;
        occColumn[p + 3] = (*(pMLX32x24_READ_INT16U_buf+24+i) & 0xF000) >> 12;
    }

    for( i = 0; i < 32; i ++)
    {
        if (occColumn[i] > 7)
        {
            occColumn[i] = occColumn[i] - 16;
        }
    }

    for( i = 0; i < 24; i++)
    {
        for( j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            mlx90640->offset[p] = (*(pMLX32x24_READ_INT16U_buf+64+p) & 0xFC00) >> 10;
            if (mlx90640->offset[p] > 31)
            {
                mlx90640->offset[p] = mlx90640->offset[p] - 64;
            }
            mlx90640->offset[p] = mlx90640->offset[p]*(1 << occRemScale);
            mlx90640->offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + mlx90640->offset[p]);
        }
    }
	DBG_PRINT("occRowScale = %d, occColumnScale = %d ,occRemScale = %d,offsetRef=%d \r\n",
		occRowScale,occColumnScale,occRemScale,offsetRef);
}

//------------------------------------------------------------------------------

void ExtractKtaPixelParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    int p = 0;
    int8_t KtaRC[4];
    int8_t KtaRoCo;
    int8_t KtaRoCe;
    int8_t KtaReCo;
    int8_t KtaReCe;
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;

	int i,j;

    KtaRoCo = (*(pMLX32x24_READ_INT16U_buf+54) & 0xFF00) >> 8;
    if (KtaRoCo > 127)
    {
        KtaRoCo = KtaRoCo - 256;
    }
    KtaRC[0] = KtaRoCo;

    KtaReCo = (*(pMLX32x24_READ_INT16U_buf+54) & 0x00FF);
    if (KtaReCo > 127)
    {
        KtaReCo = KtaReCo - 256;
    }
    KtaRC[2] = KtaReCo;

    KtaRoCe = (*(pMLX32x24_READ_INT16U_buf+55) & 0xFF00) >> 8;
    if (KtaRoCe > 127)
    {
        KtaRoCe = KtaRoCe - 256;
    }
    KtaRC[1] = KtaRoCe;

    KtaReCe = (*(pMLX32x24_READ_INT16U_buf+55) & 0x00FF);
    if (KtaReCe > 127)
    {
        KtaReCe = KtaReCe - 256;
    }
    KtaRC[3] = KtaReCe;

    ktaScale1 = ((*(pMLX32x24_READ_INT16U_buf+56) & 0x00F0) >> 4) + 8;
    ktaScale2 = (*(pMLX32x24_READ_INT16U_buf+56) & 0x000F);

    for( i = 0; i < 24; i++)
    {
        for( j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            split = 2*(p/32 - (p/64)*2) + p%2;
            mlx90640->kta[p] = (*(pMLX32x24_READ_INT16U_buf+64+p) & 0x000E) >> 1;
            if (mlx90640->kta[p] > 3)
            {
                mlx90640->kta[p] = mlx90640->kta[p] - 8;
            }
            mlx90640->kta[p] = mlx90640->kta[p] * (1 << ktaScale2);
            mlx90640->kta[p] = KtaRC[split] + mlx90640->kta[p];
            mlx90640->kta[p] = mlx90640->kta[p] / pow(2,(double)ktaScale1);
        }
    }
	DBG_PRINT("KtaRoCo = %d, KtaReCo = %d ,KtaRoCe = %d,KtaReCe = %d,ktaScale1 = %d,ktaScale2 = %d \r\n",
		KtaRoCo,KtaReCo,KtaRoCe,KtaReCe,ktaScale1,ktaScale2);
}

//------------------------------------------------------------------------------

void ExtractKvPixelParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    int p = 0;
    int8_t KvT[4];
    int8_t KvRoCo;
    int8_t KvRoCe;
    int8_t KvReCo;
    int8_t KvReCe;
    uint8_t kvScale;
    uint8_t split;

	int i,j;

    KvRoCo = (*(pMLX32x24_READ_INT16U_buf+52) & 0xF000) >> 12;
    if (KvRoCo > 7)
    {
        KvRoCo = KvRoCo - 16;
    }
    KvT[0] = KvRoCo;

    KvReCo = (*(pMLX32x24_READ_INT16U_buf+52) & 0x0F00) >> 8;
    if (KvReCo > 7)
    {
        KvReCo = KvReCo - 16;
    }
    KvT[2] = KvReCo;

    KvRoCe = (*(pMLX32x24_READ_INT16U_buf+52) & 0x00F0) >> 4;
    if (KvRoCe > 7)
    {
        KvRoCe = KvRoCe - 16;
    }
    KvT[1] = KvRoCe;

    KvReCe = (*(pMLX32x24_READ_INT16U_buf+52) & 0x000F);
    if (KvReCe > 7)
    {
        KvReCe = KvReCe - 16;
    }
    KvT[3] = KvReCe;

    kvScale = (*(pMLX32x24_READ_INT16U_buf+56) & 0x0F00) >> 8;


    for( i = 0; i < 24; i++)
    {
        for( j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            split = 2*(p/32 - (p/64)*2) + p%2;
            mlx90640->kv[p] = KvT[split];
            mlx90640->kv[p] = mlx90640->kv[p] / pow(2,(double)kvScale);
        }
    }
	DBG_PRINT("KvRoCo = %d, KvReCo = %d ,KvRoCe = %d,KvReCe = %d,kvScale = %d \r\n",
		KvRoCo,KvReCo,KvRoCe,KvReCe,kvScale);
}

//------------------------------------------------------------------------------

void ExtractCPParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = ((*(pMLX32x24_READ_INT16U_buf+32) & 0xF000) >> 12) + 27;

    offsetSP[0] = (*(pMLX32x24_READ_INT16U_buf+58) & 0x03FF);
    if (offsetSP[0] > 511)
    {
        offsetSP[0] = offsetSP[0] - 1024;
    }

    offsetSP[1] = (*(pMLX32x24_READ_INT16U_buf+58) & 0xFC00) >> 10;
    if (offsetSP[1] > 31)
    {
        offsetSP[1] = offsetSP[1] - 64;
    }
    offsetSP[1] = offsetSP[1] + offsetSP[0];

    alphaSP[0] = (*(pMLX32x24_READ_INT16U_buf+57) & 0x03FF);
    if (alphaSP[0] > 511)
    {
        alphaSP[0] = alphaSP[0] - 1024;
    }

    alphaSP[0] = alphaSP[0] /  pow(2,(double)alphaScale);

    alphaSP[1] = (*(pMLX32x24_READ_INT16U_buf+57) & 0xFC00) >> 10;

    if (alphaSP[1] > 31)
    {
        alphaSP[1] = alphaSP[1] - 64;
    }
    alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0];

    cpKta = (*(pMLX32x24_READ_INT16U_buf+59) & 0x00FF);
    if (cpKta > 127)
    {
        cpKta = cpKta - 256;
    }
    ktaScale1 = ((*(pMLX32x24_READ_INT16U_buf+56) & 0x00F0) >> 4) + 8;
    mlx90640->cpKta = cpKta / pow(2,(double)ktaScale1);

    cpKv = (*(pMLX32x24_READ_INT16U_buf+59) & 0xFF00) >> 8;
    if (cpKv > 127)
    {
        cpKv = cpKv - 256;
    }
    kvScale = (*(pMLX32x24_READ_INT16U_buf+56) & 0x0F00) >> 8;
    mlx90640->cpKv = cpKv / pow(2,(double)kvScale);

    mlx90640->cpAlpha[0] = alphaSP[0];
    mlx90640->cpAlpha[1] = alphaSP[1];
    mlx90640->cpOffset[0] = offsetSP[0];
    mlx90640->cpOffset[1] = offsetSP[1];
}

//------------------------------------------------------------------------------

void ExtractCILCParameters(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    float ilChessC[3];
    uint8_t calibrationModeEE;

    calibrationModeEE = (*(pMLX32x24_READ_INT16U_buf+10) & 0x0800) >> 4;
    calibrationModeEE = calibrationModeEE ^ 0x80;

    ilChessC[0] = (*(pMLX32x24_READ_INT16U_buf+53) & 0x003F);
    if (ilChessC[0] > 31)
    {
        ilChessC[0] = ilChessC[0] - 64;
    }
    ilChessC[0] = ilChessC[0] / 16.0f;

    ilChessC[1] = (*(pMLX32x24_READ_INT16U_buf+53) & 0x07C0) >> 6;
    if (ilChessC[1] > 15)
    {
        ilChessC[1] = ilChessC[1] - 32;
    }
    ilChessC[1] = ilChessC[1] / 2.0f;

    ilChessC[2] = (*(pMLX32x24_READ_INT16U_buf+53) & 0xF800) >> 11;
    if (ilChessC[2] > 15)
    {
        ilChessC[2] = ilChessC[2] - 32;
    }
    ilChessC[2] = ilChessC[2] / 8.0f;

    mlx90640->calibrationModeEE = calibrationModeEE;
    mlx90640->ilChessC[0] = ilChessC[0];
    mlx90640->ilChessC[1] = ilChessC[1];
    mlx90640->ilChessC[2] = ilChessC[2];
	DBG_PRINT("calibrationModeEE ?? = 0x%x \r\n",calibrationModeEE);
}

//------------------------------------------------------------------------------

int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2)
 {
     int pixPosDif;

     pixPosDif = pix1 - pix2;
     if(pixPosDif > -34 && pixPosDif < -30)
     {
         return -6;
     }
     if(pixPosDif > -2 && pixPosDif < 2)
     {
         return -6;
     }
     if(pixPosDif > 30 && pixPosDif < 34)
     {
         return -6;
     }

     return 0;
 }


int ExtractDeviatingPixels(INT16U	*pMLX32x24_READ_INT16U_buf, paramsMLX90640_t *mlx90640)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    int warn = 0;
    int i;

    for(pixCnt = 0; pixCnt<5; pixCnt++)
    {
        mlx90640->brokenPixels[pixCnt] = 0xFFFF;
        mlx90640->outlierPixels[pixCnt] = 0xFFFF;
    }

    pixCnt = 0;
    while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5)
    {
        if(*(pMLX32x24_READ_INT16U_buf+64+pixCnt) == 0)
        {
            mlx90640->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }
        else if((*(pMLX32x24_READ_INT16U_buf+64+pixCnt) & 0x0001) != 0)
        {
            mlx90640->outlierPixels[outlierPixCnt] = pixCnt;
            outlierPixCnt = outlierPixCnt + 1;
        }

        pixCnt = pixCnt + 1;

    }

    if(brokenPixCnt > 4)
    {
        warn = -3;
    }
    else if(outlierPixCnt > 4)
    {
        warn = -4;
    }
    else if((brokenPixCnt + outlierPixCnt) > 4)
    {
        warn = -5;
    }
    else
    {
        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<brokenPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->brokenPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }
            }
        }

        for(pixCnt=0; pixCnt<outlierPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->outlierPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }
            }
        }

        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=0; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }
            }
        }

    }


    return warn;

}



/**
 * @brief   MXL initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void MXL90640_thermopile_init(void)
{
	MXL_sccb_open();
	DBG_PRINT("MXL_sccb_open() \r\n");
}

/**
 * @brief   MLX un-initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void MXL90640_thermopile_uninit(void)
{

	// release sccb
	MXL_sccb_close();

	// Turn off LDO 2.8V for CSI sensor
	//drv_l1_power_ldo_28_ctrl(0, LDO_LDO28_2P8V);
	//gpio_write_io(GC0308_RESET,DATA_LOW);
}








/**
 * @brief   gc0308 stream start function
 * @param   info index
 *
 * @return 	none
 */
void MXL90640_thermopile_stream_start(INT32U index, INT32U bufA, INT32U bufB)
{

	INT32S  nRet;

	INT32U	TimeCnt1,TimeCnt2;

	INT16U 	EEcopy16BIT[8]={0};
	INT16U	EEaddress16,*pEEcopy16BIT;
	INT8U 	EEaddr[2],*pEEaddr;
	INT8U  *pMLX32x24_READ_INT8U_buf;

	INT16U  *pMLX32x24_READ_INT16U_buf;
	INT16U 	FromEEcontrolRegister1,controlRegister1,i;
	INT16U	cnt,frameData_cnt;

	int		error;

	DBG_PRINT("%s = %d _davis\r\n", __func__, 0);
	//while(1);

	pMLX90640_Para = &MLX90640_Para;
	gp_memset((INT8S *)&MLX90640_Para, 0, sizeof(paramsMLX90640_t));
	pMLX_TH32x24_Para = &MLX_TH32x24_Para;
    gp_memset((INT8S *)&MLX_TH32x24_Para, 0, sizeof(MLX_TH32x24Para_t));

	pMLX_TH32x24_Para->MLX_TH32x24_width = bufA;
    pMLX_TH32x24_Para->MLX_TH32x24_height = bufB;

	if(MLX_TH32x24_mem_alloc() < 0) {	// RAM 正常 
		DBG_PRINT("MLX_TH32x24_mem_alloc err!! \r\n");
		while(1);
	}


    pMLX_TH32x24_Para->MLX_TH32x24_InitReadEE_startON = 0;

	//MLX_TH32x24_TEST_HIGH();
	TimeCnt1=xTaskGetTickCount();
	DBG_PRINT("StartTime = %d\r\n", xTaskGetTickCount());	// xTaskGetTickCount() timebase=1ms


	pEEcopy16BIT = EEcopy16BIT;
	pEEaddr = EEaddr;
	pMLX32x24_READ_INT8U_buf = (INT8U*)pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF;
	pMLX32x24_READ_INT16U_buf = (INT16U*)pMLX_TH32x24_Para->MLX32x24_EE_READ_16bitBUF;
	DBG_PRINT("************************************************** \r\n");
	DBG_PRINT("pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF addr=0x%0x \r\n", pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF);
	DBG_PRINT("pMLX_TH32x24_Para->MLX32x24_EE_READ_16bitBUF addr=0x%0x \r\n", pMLX_TH32x24_Para->MLX32x24_EE_READ_16bitBUF);

	MXL_handle.devNumber = I2C_1;
	MXL_handle.slaveAddr = MLX90640_SLAVE_ADDR<<1;
	MXL_handle.clkRate = 900;

	//
	//Wait 80ms + delay determined by the refresh rate
	//
	osDelay(80);



	// =====================================================================

	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrDevID,pEEcopy16BIT);
	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrDevID+1,pEEcopy16BIT+1);
	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrDevID+2,pEEcopy16BIT+2);
		DBG_PRINT("EEPROM MLX90640_AdrDevID addr=0x%04X, data=0x%04X - 0x%04X - 0x%04X \r\n",
			MLX90640_AdrDevID, *(pEEcopy16BIT) ,*(pEEcopy16BIT+1),*(pEEcopy16BIT+2));

	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_EEAddrRegister1,pEEcopy16BIT);
	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_EEAddrRegister2,pEEcopy16BIT+1);
	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_EEAddrConfig,pEEcopy16BIT+2);
	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_EEAddrInternal_I2C,pEEcopy16BIT+3);
		DBG_PRINT("EEPROM MLX90640_EEAddrRegister1 addr=0x%04X, data=0x%04X - 0x%04X - 0x%04X - 0x%04X \r\n",
				MLX90640_EEAddrRegister1, *(pEEcopy16BIT) ,*(pEEcopy16BIT+1),*(pEEcopy16BIT+2),*(pEEcopy16BIT+3));

	FromEEcontrolRegister1 = *(pEEcopy16BIT);

	EEaddress16 = MLX90640_EEAddrstart;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pMLX32x24_READ_INT8U_buf
			,MLX90640_EEMemAddrRead*2,MXL_I2C_RESTART_MODE); // 多筆讀取 EE

	for(cnt=0; cnt < MLX90640_EEMemAddrRead; cnt++)
	{
		i = cnt << 1;
		*(pMLX32x24_READ_INT16U_buf+cnt) = (INT16U)*(pMLX32x24_READ_INT8U_buf+i) *256
			+ (INT16U)*(pMLX32x24_READ_INT8U_buf+i+1);
	}

	osDelay(80);
	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
	DBG_PRINT("read controlRegister1 = 0x%04x \r\n",controlRegister1);

	DBG_PRINT("read controlRegister1 from EE = 0x%04x \r\n",FromEEcontrolRegister1);

	while(controlRegister1 != FromEEcontrolRegister1 ){

	drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrControlRegister1,FromEEcontrolRegister1);
	osDelay(80);
	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
	DBG_PRINT("*read controlRegister1 = 0x%04x \r\n",controlRegister1);

	}




	MLX90640_SetRefreshRate(MLX90640_REFRESH_RATE_32HZ);
						DBG_PRINT("SetRefreshRate = %d Hz \r\n",
							MLX_REFRESH_RATE_HZ2[MLX90640_REFRESH_RATE_32HZ]);
	osDelay(80);
	osDelay(DELAYTIME_at_REFRESH_RATE2[MLX90640_REFRESH_RATE_32HZ]);
		DBG_PRINT("SetRefreshRate then delay  = %d ms \r\n",
			DELAYTIME_at_REFRESH_RATE2[MLX90640_REFRESH_RATE_32HZ]);

	// 設定 StepMode & Subpage0
	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
	DBG_PRINT("read controlRegister1 = 0x%04x \r\n",controlRegister1);

	controlRegister1 = controlRegister1 & MLX90640_SetModeClear ;
	controlRegister1 = controlRegister1 | MLX90640_SetStepModeSubpageRep ;

	drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrControlRegister1,controlRegister1);
	DBG_PRINT("(set initial subpage = 0)write controlRegister1 = 0x%04x \r\n",controlRegister1);

	// controlRegister1 設定成 自動 subpage 0/1
	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrControlRegister1,&controlRegister1);
	DBG_PRINT("read controlRegister1 = 0x%04x \r\n",controlRegister1);


	pMLX_TH32x24_Para->MLX_TH32x24_InitSet_controlRegister1 = controlRegister1;

		// 0x0030 :
		// 1 Data in RAM overwrite is enabled
		// 1 In step mode - start of measurement
		//		(set by the customer and cleared once the measurement is done)
		//
	error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrStatus,0x0030);

	DBG_PRINT("set MLX_TH32x24 start of measurement \r\n");

	//DBG_PRINT("clear pMLX90640_Para - 1 \r\n");
	//retVal = gp_memset((INT8S *)pMLX90640_Para,0,sizeof(paramsMLX90640_t));	// clear 值 
	//DBG_PRINT("clear pMLX90640_Para -2\r\n");

	error = CheckEEPROMValid(pMLX32x24_READ_INT16U_buf);

	DBG_PRINT("CheckEEPROMValid, ERROR=%d \r\n", error);
#if 1
	if(error == 0)
	{
		ExtractVDDParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		DBG_PRINT("MLX32x24_Para->kVdd=%d, MLX32x24_Para->vdd25=%d \r\n", pMLX90640_Para->kVdd,pMLX90640_Para->vdd25);
	#if 1
		ExtractPTATParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		//DBG_PRINT("MLX32x24_Para->KvPTAT=%f, MLX32x24_Para->KtPTAT=%f ,MLX32x24_Para->vPTAT25= %d ,MLX32x24_Para->alphaPTAT=%f \r\n",
		//	pMLX90640_Para->KvPTAT,pMLX90640_Para->KtPTAT,pMLX90640_Para->vPTAT25,pMLX90640_Para->alphaPTAT);

		ExtractGainParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		DBG_PRINT("MLX32x24_Para->gainEE=%d \r\n",pMLX90640_Para->gainEE);

		ExtractTgcParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractResolutionParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractKsTaParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractKsToParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractAlphaParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractOffsetParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractKtaPixelParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractKvPixelParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractCPParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		ExtractCILCParameters(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
		error = ExtractDeviatingPixels(pMLX32x24_READ_INT16U_buf, pMLX90640_Para);
	#endif
	}

	

	gp_memset((INT8S *)pMLX_TH32x24_Para->result,0x00,MLX_Pixel*2); // clear 值 
				DBG_PRINT("clear result \r\n");

#endif

	// 可以 與 pMLX_TH32x24_Para->MLX32x24_EE_READ_8bitBUF 共用 ??

	TimeCnt2 = xTaskGetTickCount();
	DBG_PRINT("\r\n EndTime = %d\r\n", xTaskGetTickCount());
	DBG_PRINT("TotalTime = %d ms\r\n",TimeCnt2 - TimeCnt1);





    pMLX_TH32x24_Para->MLX_TH32x24_InitReadEE_startON = 1;

}


/**
 * @brief    stream stop function
 * @param   info index
 *
 * @return 	none
 */
void MXL90640_thermopile_stream_stop(void)
{

	DBG_PRINT("%s = %d _davis\r\n", __func__, 0);

}



void MXL_TEST_LOW(void)
{
	gpio_write_io(C_ISR_TEST_PIN, 0);
}
void MXL_TEST_HIGH(void)
{
	gpio_write_io(C_ISR_TEST_PIN, 1);
}

//float MLX90640_GetVdd(uint16_t *frameData, paramsMLX90640_t *params)
void MLX90640_GetVdd(INT16U frameData_vdd,INT16U frameData_resolutionRAM)
{
    float vdd;
    float resolutionCorrection;

    int resolutionRAM;

	//vdd = pMLX_TH32x24_Para->frameData[810];
	vdd = frameData_vdd;

    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }

	//resolutionRAM = (pMLX_TH32x24_Para->frameData[832] & 0x0C00) >> 10;
	
	resolutionRAM = (frameData_resolutionRAM & 0x0C00) >> 10;

	resolutionCorrection = pow(2, (double)pMLX90640_Para->resolutionEE) / pow(2, (double)resolutionRAM);

	vdd = (resolutionCorrection * vdd - pMLX90640_Para->vdd25) / pMLX90640_Para->kVdd + 3.3;

	pMLX_TH32x24_Para->MLX_TH32x24_vdd = vdd;
    return ;
}
//------------------------------------------------------------------------------

//float MLX90640_GetTa(uint16_t *frameData, paramsMLX90640_t *params)
void MLX90640_GetTa(INT16U frameData_ptat,INT16U frameData_ptatArt)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;

    //MLX90640_GetVdd();
	vdd = pMLX_TH32x24_Para->MLX_TH32x24_vdd;

	//ptat = pMLX_TH32x24_Para->frameData[800];
	ptat = frameData_ptat;
	if(ptat > 32767)
	{
	  ptat = ptat - 65536;
	}

	//ptatArt = pMLX_TH32x24_Para->frameData[768];
	ptatArt = frameData_ptatArt;
	if(ptatArt > 32767)
	{
	    ptatArt = ptatArt - 65536;
	}
	ptatArt = (ptat / (ptat * pMLX90640_Para->alphaPTAT + ptatArt)) * pow(2, (double)18);

	ta = (ptatArt / (1 + pMLX90640_Para->KvPTAT * (vdd - 3.3)) - pMLX90640_Para->vPTAT25);
	ta = ta / pMLX90640_Para->KtPTAT + 25;
	pMLX_TH32x24_Para->MLX_TH32x24_ta = ta;

    return ;
}



/**
 * @brief   gc0308 get info function
 * @param   none
 * @return 	pointer to sensor information data
 */
drv_l2_sensor_info_t* MXL90640_thermopile_get_info(INT32U index)
{
	if(index > (MAX_INFO_NUM - 1))
		return NULL;
	else
		return (drv_l2_sensor_info_t*)&mlx90640_sensor_thermal_ops.info[index];
}

/*********************************************
*	sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t mlx90640_sensor_thermal_ops =
{
	SENSOR_MLX90640_THERMAL_NAME,	/* sensor name */
	MXL90640_thermopile_init,
	MXL90640_thermopile_uninit,
	MXL90640_thermopile_stream_start,
	MXL90640_thermopile_stream_stop,
	MXL90640_thermopile_get_info,
	{
		/* 1rd info */
		{
			MCLK_24M,					/* CSI clock */
			V4L2_PIX_FMT_VYUY,			/* input format */
			V4L2_PIX_FMT_VYUY,			/* output format */
			0,			/* FPS in sensor */
			32,					/* target width */
			24, 				/* target height */
			32,					/* sensor width */
			24, 				/* sensor height */
			0,							/* sensor h offset */
			0,							/* sensor v offset */
			MODE_CCIR_HREF,				/* input interface */
			MODE_NONE_INTERLACE,		/* interlace mode */
			MODE_ACTIVE_HIGH,			/* hsync pin active level */
			MODE_ACTIVE_LOW,			/* vsync pin active level */
		}
	}
};


#endif //(defined _SENSOR_MXL90640_THERMOPILE) && (_SENSOR_MXL90640_THERMOPILE == 1)
