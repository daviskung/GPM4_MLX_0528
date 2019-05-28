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
//#include "defs_th32x32.h"
#include	"defs_MXL.h"

//_SENSOR_MLX90640_THERMOPILE


#if (defined _SENSOR_MXL90640_THERMOPILE) && (_SENSOR_MXL90640_THERMOPILE == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/


// MXL sccb interface
#define SCCB_GPIO		0
#define SCCB_HW_I2C		1
#define MXL_SCCB_MODE		SCCB_HW_I2C //SCCB_HW_I2C//SCCB_GPIO


/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
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

    MXL_handle.clkRate = 400;
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




INT32S MLX90640_SetResolution(INT8U resolution)
{
    INT32S error;
	INT16U value;

	INT16U 	EEcopy16BIT[1]={0},*pEEcopy16BIT;
	INT8U	resolutionRAM;

	pEEcopy16BIT = EEcopy16BIT;

	value = (resolution & 0x03) << 10;

	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrRegister1,pEEcopy16BIT);

	if(error > 0)
    {
        resolutionRAM = (EEcopy16BIT[0] & 0x0C00) >> 10;
			DBG_PRINT("GetResolution value = 0x%04X \r\n",resolutionRAM);

		value = (EEcopy16BIT[0] & 0xF3FF ) | value;
		//DBG_PRINT("value = 0x%04X \r\n",value);

		error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrRegister1,value);
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

	error = drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrRegister1,pEEcopy16BIT);

	if(error > 0)
    {
        refreshRateRAM = (EEcopy16BIT[0] & 0x0380) >> 7;
			DBG_PRINT("GetRefreshRate value = 0x%04X \r\n",refreshRateRAM);

		value = (EEcopy16BIT[0] & 0xFC7F ) | value;
		//DBG_PRINT("value = 0x%04X \r\n",value);

		error = drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_AdrRegister1,value);
	}

    return error;
}

int CheckEEPROMValid(uint16_t *eeData)
{
	int deviceSelect;
	deviceSelect = eeData[10] & 0x0040;
	if(deviceSelect == 0)
	{
		return 0;
	}

	return -7;
}

void ExtractVDDParameters(INT16U *eeData) //, paramsMLX90640_t *mlx90640)
{
    int16_t kVdd;
    int16_t vdd25;

    kVdd = eeData[51];

    kVdd = (eeData[51] & 0xFF00) >> 8;
    if(kVdd > 127)
    {
        kVdd = kVdd - 256;
    }
    kVdd = 32 * kVdd;
    vdd25 = eeData[51] & 0x00FF;
    vdd25 = ((vdd25 - 256) << 5) - 8192;

	DBG_PRINT("mlx90640->kVdd=%d, mlx90640->vdd25=%d \r\n", kVdd,vdd25);

   // pMLX_Para->kVdd = kVdd;
   // pMLX_Para->vdd25 = vdd25;
}



/**
 * @brief   MXL initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void MXL90640_thermopile_init(void)
{
	int		error;
	INT8U value;
	INT16U value16bit;

	INT8U 	EEcopy[8]={0} ;
	INT16U 	EEcopy16BIT[8]={0},*pEEcopy16BIT;
	INT8U 	EEaddr[2],*pEEaddr;
	INT16U	EEaddress16;
	char 	i2cData[1664] = {0},*pEEcopy;
	INT16U	eeData[MLX90640_EEMemAddrRead],*p_eeData;
	INT16U	i,cnt;
	//paramsMLX90640_t	MLX_Para,*pMLX_Para;

	pEEcopy16BIT = EEcopy16BIT;
	pEEcopy = i2cData;
	pEEaddr = EEaddr;
	p_eeData = eeData;
	//pMLX_Para = &MLX_Para ;			// 2019.05.07 davis

	// request sccb
	MXL_sccb_open();
	DBG_PRINT("MXL_sccb_open() \r\n");
	
	MXL_handle.devNumber = I2C_1;
	MXL_handle.slaveAddr = MLX90640_SLAVE_ADDR<<1;
	MXL_handle.clkRate = 800;
	
	pEEcopy = i2cData;
	pEEaddr = EEaddr;
	p_eeData = eeData;

	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_EEAddrstart,pEEcopy16BIT);	// 單筆讀取 
	EEaddress16 = MLX90640_EEAddrstart;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pEEcopy,2,MXL_I2C_RESTART_MODE);
	EEaddress16 = MLX90640_EEAddrstart+1;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pEEcopy+2,2,MXL_I2C_RESTART_MODE);
	EEaddress16 = MLX90640_EEAddrstart+2;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pEEcopy+4,2,MXL_I2C_RESTART_MODE);


	DBG_PRINT("EEPROM MLX90640_EEAddrstart addr=0x%04X, data=0x%02X 0x%02X 0x%02X \r\n",
				MLX90640_EEAddrstart, *(pEEcopy+2),*(pEEcopy+1),*(pEEcopy));

	
	EEaddress16 = MLX90640_EEAddrstart;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&MXL_handle,pEEaddr,2,pEEcopy,MLX90640_EEMemAddrRead*2,MXL_I2C_RESTART_MODE); // 多筆讀取 EE
	for(cnt=0; cnt < MLX90640_EEMemAddrRead; cnt++)
	{
		i = cnt << 1;
		*p_eeData++ = (INT16U)i2cData[i]*256 + (INT16U)i2cData[i+1];
	}
	
	DBG_PRINT("MLX90640_DumpEE, cnt=%d \r\n", cnt);

	/*
	//MLX90640_ExtractParameters(p_eeData,pMLX_Para);
	error = CheckEEPROMValid(eeData);

	DBG_PRINT("CheckEEPROMValid, ERROR=%d \r\n", error);

	if(error == 0)
    {
        ExtractVDDParameters(p_eeData); //, pMLX_Para);

        ExtractPTATParameters(eeData, mlx90640);
        ExtractGainParameters(eeData, mlx90640);
        ExtractTgcParameters(eeData, mlx90640);
        ExtractResolutionParameters(eeData, mlx90640);
        ExtractKsTaParameters(eeData, mlx90640);
        ExtractKsToParameters(eeData, mlx90640);
        ExtractAlphaParameters(eeData, mlx90640);
        ExtractOffsetParameters(eeData, mlx90640);
        ExtractKtaPixelParameters(eeData, mlx90640);
        ExtractKvPixelParameters(eeData, mlx90640);
        ExtractCPParameters(eeData, mlx90640);
        ExtractCILCParameters(eeData, mlx90640);
        error = ExtractDeviatingPixels(eeData, mlx90640);


    }
	*/

	

#if 0
	value16bit = 0x1981;	// refresh rate = 4 Hz
	drv_l1_reg_2byte_data_2byte_write(&MXL_handle,MLX90640_EEAddrRegister1,value16bit);
	DBG_PRINT("Write EEPROM MLX90640_EEAddrRegister1 addr=0x%04X, data=0x%04X \r\n",
				MLX90640_EEAddrRegister1,value16bit );
#endif

	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrStatus,pEEcopy16BIT);
	DBG_PRINT("EEPROM MLX90640_AdrStatus addr=0x%04X, data=0x%04X \r\n",MLX90640_AdrStatus, *(pEEcopy16BIT));

	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrRegister1,pEEcopy16BIT);
		DBG_PRINT("MLX90640_AdrRegister1 addr=0x%04X, data=0x%04X \r\n",MLX90640_AdrRegister1, *(pEEcopy16BIT));

	MLX90640_SetResolution(MLX90640_RESOLUTION_17B);
		DBG_PRINT("SetResolution value = 0x%04X \r\n",MLX90640_RESOLUTION_17B);
	MLX90640_SetRefreshRate(MLX90640_REFRESH_RATE_8HZ);
		DBG_PRINT("SetRefreshRate value = 0x%04X \r\n",MLX90640_REFRESH_RATE_8HZ);

	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrRegister1,pEEcopy16BIT);
		DBG_PRINT("after MLX90640_AdrRegister1 addr=0x%04X, data=0x%04X \r\n",MLX90640_AdrRegister1, *(pEEcopy16BIT));

	drv_l1_reg_2byte_data_2byte_read(&MXL_handle,MLX90640_AdrConfig,pEEcopy16BIT);
	DBG_PRINT("EEPROM MLX90640_AdrConfig addr=0x%04X, data=0x%04X \r\n", MLX90640_AdrConfig, *(pEEcopy16BIT));

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



}


void MXL_TEST_LOW(void)
{
	gpio_write_io(C_ISR_TEST_PIN, 0);
}
void MXL_TEST_HIGH(void)
{
	gpio_write_io(C_ISR_TEST_PIN, 1);
}

#if 0
//
// read MXL ReadElecOffset raw data 129*2byte
//
INT32S I2C_sensor32x32_readDataTop(drv_l1_i2c_bus_handle_t *handle,INT8U* Id)
{
	INT32S	ret = STATUS_OK;
	INT8U   i;

	INT8U 	EEaddr[2],*pEEaddr;

	pEEaddr = EEaddr;
	//EEaddr[0]=MXL_READ_DATA_TOP;
	ret = drv_l1_i2c_multi_read(handle,pEEaddr,1,
		Id,ELAMOUNT+2,MXL_I2C_RESTART_MODE);

	return ret;
}

INT32S I2C_sensor32x32_readDataBtm(drv_l1_i2c_bus_handle_t *handle,INT8U* Id)
{
	INT32S	ret = STATUS_OK;
	INT8U   i;

	INT8U 	EEaddr[2],*pEEaddr;

	pEEaddr = EEaddr;
	//EEaddr[0]=MXL_READ_DATA_BOM;
	ret = drv_l1_i2c_multi_read(handle,pEEaddr,1,
		Id,ELAMOUNT+2,MXL_I2C_RESTART_MODE);

	return ret;
}

#endif




#endif //(defined _SENSOR_MXL90640_THERMOPILE) && (_SENSOR_MXL90640_THERMOPILE == 1)
