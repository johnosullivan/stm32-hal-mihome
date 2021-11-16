
#include "lc709203f.h"

/*
   @Brief         CRC Checker
   @Description   Needed byte for Cyclic Redundancy Check.
                  LC709203, Control Reg, LSB Byte, MSB Byte, CRC Byte
   @Parameter     uint8_t *rec_values -> array name for check
                  uint8_t len         -> lengt of array
   @Return value  uint8_t
 */
uint8_t lc_check_crc(uint8_t *rec_values, uint8_t len) {
  uint8_t crc = 0x00;
  uint8_t current_byte;
  uint8_t bit;
  for (current_byte = 0 ; current_byte < len ; current_byte++)
  {
   crc ^= (rec_values[current_byte]);
    for (bit = 8 ; bit > 0 ;bit--)
    {
     if (crc & 0x80)
       {
         crc = (crc << 1) ^ 0x07;
       }
     else
       {
         crc = (crc << 1);
       }
    }
  }
  return crc;
}
/* mapping function */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
   @Brief
   @Description
   @Parameter LC709203_HandleTypedef pointer
   @Return LC709203F_STATUS_OK or LC709203F_STATUS_FAILED
 */
int write_lc709(LC709203_HandleTypedef *config, uint8_t command, uint16_t data) {
	/*
	 * Creates Write Message
	 */
	uint8_t buffer[5];
	buffer[0] = config->addr * 2;
	buffer[1] = command;
	buffer[2] = data & 0xFF;
	buffer[3] = data >> 8;
	buffer[4] = lc_check_crc(buffer, 4);

	return HAL_I2C_Master_Transmit(config->i2c, config->addr << 1, buffer + 1, 4, 200);
}

/*
   @Brief
   @Description
   @Parameter LC709203_HandleTypedef pointer
   @Return LC709203F_STATUS_OK or LC709203F_STATUS_FAILED
 */
uint16_t read_lc709(LC709203_HandleTypedef *config, uint8_t command) {
	uint16_t data = 0;
	uint8_t reply[6];
	uint8_t read[3];

	reply[0] = config->addr * 2;
	reply[1] = command;
	reply[2] = reply[0] | 0x1;
	reply[3] = 0x0;
	reply[4] = 0x0;
	reply[5] = 0x0;

	/*
	 * Sends Message
	 */
	if (HAL_I2C_Master_Transmit(config->i2c,config->addr << 1, reply, 3, 200) != HAL_OK) {
		return LC709203F_STATUS_FAILED;
	}

	/*
	 * Reads Response
	 */
	if (HAL_I2C_Mem_Read(config->i2c, (config->addr << 1), command, I2C_MEMADD_SIZE_8BIT, read, 3, 200) != HAL_OK) {
		return LC709203F_STATUS_FAILED;
	}
	/*
	 * Back Fill Bytes
	 */
	reply[3] = read[0];
	reply[4] = read[1];
	reply[5] = read[2];

	/*
	 * CRC Checker
	 */
	uint8_t crc = lc_check_crc(reply, 5);
	if (crc != reply[5]) {
	   return LC709203F_STATUS_FAILED;
	}

	/*
	 * Process Bytes
	 */
	data = reply[4];
	data <<= 8;
	data |= reply[3];

	return data;
}

/*
   @Brief LC709203_Init
   @Description Sets up the LC709203 IC
   @Parameter LC709203_HandleTypedef pointer
   @Return LC709203F_STATUS_OK or LC709203F_STATUS_FAILED
 */
LC709203_status_t LC709203_Init(LC709203_HandleTypedef *config) {
	/*
	 * Sets LC709203 Power Mode
	 */
	if(write_lc709(config, LC709203F_CMD_POWERMODE, config->power) != HAL_OK) {
		return LC709203F_STATUS_FAILED;
	}

	/* 10ms delay */
	HAL_Delay(10);

	/*
	 * Sets LC709203 Battery Size
	 */
	if(write_lc709(config, LC709203F_CMD_APA, config->adjust) != HAL_OK) {
		return LC709203F_STATUS_FAILED;
	}

	/* 10ms delay */
	HAL_Delay(10);

	/*
	 * Sets LC709203 Battery Profile
	 */
	if(write_lc709(config, LC709203F_CMD_BATTPROF, 0x1) != HAL_OK) {
		return LC709203F_STATUS_FAILED;
	}

	/* 10ms delay */
	HAL_Delay(10);

	/*
	 * Sets LC709203 Status Bit
	 */
	if(write_lc709(config, LC709203F_CMD_STATUSBIT, config->tempmode) != HAL_OK) {
		return LC709203F_STATUS_FAILED;
	}

	/* 10ms delay */
	HAL_Delay(10);

	/*
	 * If RSOC Alert Set, Sets LC709203 ALARMRSOC
	 */
	if (config->arsoc != 0) {
		if(write_lc709(config, LC709203F_CMD_ALARMRSOC, config->arsoc) != HAL_OK) {
			return LC709203F_STATUS_FAILED;
		}
	}

	/* 10ms delay */
	HAL_Delay(10);

	/*
	 * If Voltage Alert Set, Sets LC709203 ALARMVOLT
	 */
	if (config->avoltage != 0) {
		if(write_lc709(config, LC709203F_CMD_ALARMVOLT, config->avoltage) != HAL_OK) {
			return LC709203F_STATUS_FAILED;
		}
	}

	return LC709203F_STATUS_OK;
}

/*
   @Brief LC709203_Read_IC_Version
   @Description Reads IC version
   @Parameter LC709203_HandleTypedef pointer
   @Return LC709203F_STATUS_OK or LC709203F_STATUS_FAILED
 */
LC709203_status_t LC709203_Read_IC_Version(LC709203_HandleTypedef *config) {
	config->version = read_lc709(config, LC709203F_CMD_ICVERSION);
	if (config->version == -1) {
		return LC709203F_STATUS_FAILED;
	}
	return LC709203F_STATUS_OK;
}

/*
   @Brief LC709203_Read_Voltage
   @Description Reads the current cell voltage
   @Parameter LC709203_HandleTypedef pointer
   @Return LC709203F_STATUS_OK or LC709203F_STATUS_FAILED
 */
LC709203_status_t LC709203_Read_Voltage(LC709203_HandleTypedef *config) {
	config->voltage = read_lc709(config, LC709203F_CMD_CELLVOLTAGE);
	if (config->voltage == -1) {
		return LC709203F_STATUS_FAILED;
	}
	return LC709203F_STATUS_OK;
}

/*
   @Brief LC709203_Read_Cell_Percent
   @Description Reads the current cell percent
   @Parameter LC709203_HandleTypedef pointer
   @Return LC709203F_STATUS_OK or LC709203F_STATUS_FAILED
 */
LC709203_status_t LC709203_Read_Cell_Percent(LC709203_HandleTypedef *config) {
	config->celllife = read_lc709(config, LC709203F_CMD_CELLITE);
	if (config->celllife == -1) {
		return LC709203F_STATUS_FAILED;
	}
	return LC709203F_STATUS_OK;
}

/*
   @Brief LC709203_Read_Cell_Temperature
   @Description Reads the current cell temperature
   @Parameter LC709203_HandleTypedef pointer
   @Return LC709203F_STATUS_OK or LC709203F_STATUS_FAILED
 */
LC709203_status_t LC709203_Read_Cell_Temperature(LC709203_HandleTypedef *config) {
	uint16_t ctr = read_lc709(config, LC709203F_CMD_CELLTEMPERATURE);
	if (ctr == -1) {
		return LC709203F_STATUS_FAILED;
	}
	config->temperature = map(ctr, 0x9E4, 0xD04, -200, 600);
	return LC709203F_STATUS_OK;
}
