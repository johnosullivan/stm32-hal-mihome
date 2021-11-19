/*
 * ccs811.c
 *
 *  Created on: Oct 28, 2021
 *      Author: johnosullivan
 */

#include "ccs811.h"

/**
  * @Brief  initialization process for the ccs811 sensor
  * @Return None
  */
bool CCS811_Init(CCS811_HandleTypedef *dev)
{
	uint8_t startup_conf = APP_START;
	uint8_t meas_mode_conf = 0;
	meas_mode_conf = DRIVE_MODE(DRIVE_MODE_MODE1);

	HAL_I2C_Master_Transmit(dev->i2c, dev->addr<<1, &startup_conf, 1, 100);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(dev->i2c, dev->addr<<1, MEAS_MODE, I2C_MEMADD_SIZE_8BIT, &meas_mode_conf, 1, 100);
	HAL_Delay(1);
	CCS811_Temp_RH_Compensation(dev, 20.0, 20.0);

	return true;
}


/**
  * @Brief  read sensor i2c co2 and tvoc registers
  * @Return None
  */
void CCS811_Measure_CO2_TVOC(CCS811_HandleTypedef *dev, uint32_t *co2, uint32_t *tvoc)
{
	uint8_t alg_result[4] = {0};
	uint32_t aux_co2;
	uint32_t aux_tvoc;

	HAL_I2C_Mem_Read(dev->i2c, dev->addr<<1, ALG_RESULT_DATA, I2C_MEMADD_SIZE_8BIT, alg_result, 4, 100);

	aux_co2 = alg_result[1] + (alg_result[0]<<8);
	aux_tvoc = alg_result[2] + (alg_result[3]<<8);

	*co2 = aux_co2;
	*tvoc = aux_tvoc;
}


/**
  * @Brief  write temperature and humidity in i2c registers for compensation
  * @Return None
  */
void CCS811_Temp_RH_Compensation(CCS811_HandleTypedef *dev, float temp, float rh)
{
	uint8_t env_data[4] = {0};

	env_data[0] = (( (uint32_t)(rh*512) ) & 0xFF00)>>8;		//humidity high byte
	env_data[1] = (( (uint32_t)(rh*512) ) & 0x00FF);		//humidity low byte

	env_data[2] = (( (uint32_t)(rh*512)-25 ) & 0xFF00)>>8;	//temperature high byte
	env_data[3] = (( (uint32_t)(rh*512)-25 ) & 0x00FF);		//temperature low byte

	HAL_I2C_Mem_Write(dev->i2c, dev->addr<<1, ENV_DATA, I2C_MEMADD_SIZE_8BIT, env_data, 4, 100);
}



