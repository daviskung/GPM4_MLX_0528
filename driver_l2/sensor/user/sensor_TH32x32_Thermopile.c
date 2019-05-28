//
//	sensor_TH32x32_Thermopile
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
#include "defs_th32x32.h"

#if (defined _SENSOR_TH32x32_THERMOPILE) && (_SENSOR_TH32x32_THERMOPILE == 1)
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/


// TH32x32 sccb interface
#define SCCB_GPIO		0
#define SCCB_HW_I2C		1
#define TH32x32_SCCB_MODE		SCCB_HW_I2C //SCCB_HW_I2C//SCCB_GPIO


#define MLX90640_TEST_ON		1

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
#if TH32x32_SCCB_MODE == SCCB_GPIO
	static void *th32x32_handle;
#elif TH32x32_SCCB_MODE == SCCB_HW_I2C
	static drv_l1_i2c_bus_handle_t th32x32_handle;
#endif


static INT32S th32x32_sccb_open(void)
{
	
	INT8U value;
#if TH32x32_SCCB_MODE == SCCB_GPIO

	//th32x32_handle = drv_l2_sccb_open(TH32x32_EEPROM_ID, 8, 8);
	if(th32x32_handle == 0) {
		DBG_PRINT(" TH32x32_EEPROM_ID Sccb open fail.\r\n");
		return STATUS_FAIL;
	}

#elif TH32x32_SCCB_MODE == SCCB_HW_I2C

	#if MLX90640_TEST_ON   // for MLX90640 is good 
	
	th32x32_handle.devNumber = I2C_1;
    th32x32_handle.slaveAddr = MLX90640_SLAVE_ADDR<<1;
   
    th32x32_handle.clkRate = 400;
    drv_l1_i2c_init(th32x32_handle.devNumber);

	DBG_PRINT(" MLX90640_SLAVE_ADDR Sccb open in HW_I2C.\r\n");
	
	#endif

	#if 1
	th32x32_handle.devNumber = I2C_1;
    th32x32_handle.slaveAddr = TH32x32_EEPROM_ID << 1;
    th32x32_handle.clkRate = 950;
    drv_l1_i2c_init(th32x32_handle.devNumber);

	DBG_PRINT(" TH32x32_EEPROM_ID Sccb open in HW_I2C.\r\n");
	#endif
#endif
	return STATUS_OK;
}

static void th32x32_sccb_close(void)
{
#if TH32x32_SCCB_MODE == SCCB_GPIO
	if(th32x32_handle) {
		drv_l2_sccb_close(th32x32_handle);
		th32x32_handle = NULL;
	}
#elif TH32x32_SCCB_MODE == SCCB_HW_I2C
	drv_l1_i2c_uninit(I2C_1);
	th32x32_handle.slaveAddr = 0;
	th32x32_handle.clkRate = 0;
#endif
}

static INT32S th32x32_sccb_write(INT8U reg, INT8U value)
{
#if TH32x32_SCCB_MODE == SCCB_GPIO
	return drv_l2_sccb_write(th32x32_handle, reg, value);

#elif TH32x32_SCCB_MODE == SCCB_HW_I2C
	INT8U data[2];

	data[0] = reg;
	data[1] = value;
	return drv_l1_i2c_bus_write(&th32x32_handle, data, 2);
#endif
}

#if 1
static INT32S th32x32_sccb_read(INT8U reg, INT8U *value)
{
#if TH32x32_SCCB_MODE == SCCB_GPIO
	INT16U data;

	if(drv_l2_sccb_read(th32x32_handle, reg, &data) >= 0) {
		*value = (INT8U)data;
		return STATUS_OK;
	} else {
		*value = 0xFF;
		return STATUS_FAIL;
	}
#elif TH32x32_SCCB_MODE == SCCB_HW_I2C
	INT8U data[1];

	data[0] = reg;
	if(drv_l1_i2c_bus_write(&th32x32_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	if(drv_l1_i2c_bus_read(&th32x32_handle, data, 1) < 0) {
		*value = 0xFF;
		return STATUS_FAIL;
	}

	*value = data[0];
#endif
	return STATUS_OK;
}
#endif


/**
 * @brief   th32x32 initialization function
 * @param   sensor format parameters
 * @return 	none
 */
void th32x32_thermopile_init(void)
{
#if MLX90640_TEST_ON
	INT8U value;
	INT16U value16bit;
	
	INT8U 	EEcopy[8]={0} ,*pEEcopy;
	INT16U 	EEcopy16BIT[8]={0},*pEEcopy16BIT;
	INT8U 	EEaddr[2],*pEEaddr;
	INT16U	EEaddress16;
	
	

	
	pEEcopy = EEcopy;
	pEEcopy16BIT = EEcopy16BIT;
	pEEcopy = EEcopy;
	pEEaddr = EEaddr;
#endif

	// request sccb
	th32x32_sccb_open();

#if MLX90640_TEST_ON

	EEaddress16 = MLX90640_EEAddrstart;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,8,TH32x32_I2C_RESTART_MODE);


	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_EEAddrstart,pEEcopy16BIT);	
	EEaddress16 = MLX90640_EEAddrstart;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy,2,TH32x32_I2C_RESTART_MODE);
	EEaddress16 = MLX90640_EEAddrstart+1;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy+2,2,TH32x32_I2C_RESTART_MODE);
	EEaddress16 = MLX90640_EEAddrstart+2;
	EEaddr[0]=(INT8U)(EEaddress16 >> 8);
	EEaddr[1]=(INT8U)(EEaddress16 & 0xff);
	drv_l1_i2c_multi_read(&th32x32_handle,pEEaddr,2,pEEcopy+4,2,TH32x32_I2C_RESTART_MODE);


	DBG_PRINT("EEPROM MLX90640_EEAddrstart addr=0x%04X, data=0x%02X 0x%02X 0x%02X \r\n", 
				MLX90640_EEAddrstart, *(pEEcopy+2),*(pEEcopy+1),*(pEEcopy));



	//drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,MLX90640_EEAddrRegister1,pEEcopy);
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_EEAddrRegister1,pEEcopy16BIT);	
	
	//drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,MLX90640_EEAddrRegister2,pEEcopy+1);
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_EEAddrRegister2,pEEcopy16BIT);	
	//drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,MLX90640_EEAddrConfig,pEEcopy+2);
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_EEAddrConfig,pEEcopy16BIT);	
	//drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,MLX90640_EEAddrInternal_I2C,pEEcopy+3);
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_EEAddrInternal_I2C,pEEcopy16BIT);	
	

	
	EEcopy[0]=0;
	EEcopy[1]=0;
	//drv_l1_reg_2byte_data_1byte_read(&th32x32_handle,MLX90640_AdrStatus,pEEcopy);
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_AdrStatus,pEEcopy16BIT);	
	
	EEcopy[0]=0;
	EEcopy[1]=0;
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_AdrRegister1,pEEcopy16BIT);	
	//DBG_PRINT("EEPROM MLX90640_AdrRegister1 addr=0x%04X, data=0x%04X \r\n",MLX90640_AdrRegister1, *(pEEcopy16BIT));	

	EEcopy[0]=0;
	EEcopy[1]=0;
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_AdrConfig,pEEcopy16BIT);	
	//DBG_PRINT("EEPROM MLX90640_AdrConfig addr=0x%04X, data=0x%04X \r\n", MLX90640_AdrConfig, *(pEEcopy16BIT));

	EEcopy[0]=0;
	EEcopy[1]=0;
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_AdrDevID,pEEcopy16BIT);	
	
	EEcopy[0]=0;
	EEcopy[1]=0;
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_AdrDevID+1,pEEcopy16BIT);	

	EEcopy[0]=0;
	EEcopy[1]=0;
	drv_l1_reg_2byte_data_2byte_read(&th32x32_handle,MLX90640_AdrDevID+2,pEEcopy16BIT);	
	
#endif

}


void TH32x32_TEST_LOW(void)
{
	gpio_write_io(C_ISR_TEST_PIN, 0);
}
void TH32x32_TEST_HIGH(void)
{
	gpio_write_io(C_ISR_TEST_PIN, 1);
}

//
// read TH32x32 ReadElecOffset raw data 129*2byte
//
INT32S I2C_sensor32x32_readDataTop(drv_l1_i2c_bus_handle_t *handle,INT8U* Id)
{
	INT32S	ret = STATUS_OK;
	INT8U   i;
	
	INT8U 	EEaddr[2],*pEEaddr;
	
	pEEaddr = EEaddr;
	EEaddr[0]=TH32x32_READ_DATA_TOP;
	ret = drv_l1_i2c_multi_read(handle,pEEaddr,1,
		Id,ELAMOUNT+2,TH32x32_I2C_RESTART_MODE);
	
	return ret;
}

INT32S I2C_sensor32x32_readDataBtm(drv_l1_i2c_bus_handle_t *handle,INT8U* Id)
{
	INT32S	ret = STATUS_OK;
	INT8U   i;
	
	INT8U 	EEaddr[2],*pEEaddr;

	pEEaddr = EEaddr;
	EEaddr[0]=TH32x32_READ_DATA_BOM;
	ret = drv_l1_i2c_multi_read(handle,pEEaddr,1,
		Id,ELAMOUNT+2,TH32x32_I2C_RESTART_MODE);
	
	return ret;
}






#endif //(defined _SENSOR_TH32x32_THERMOPILE) && (_SENSOR_TH32x32_THERMOPILE == 1)
