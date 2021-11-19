#ifndef _LC709203F_H
#define _LC709203F_H

#include "config.h"

#define LC709203F_I2CADDR_DEFAULT 		0x0B
#define LC709203F_CMD_THERMISTORB 		0x06
#define LC709203F_CMD_INITRSOC 			0x07
#define LC709203F_CMD_CELLTEMPERATURE 	0x08
#define LC709203F_CMD_CELLVOLTAGE		0x09
#define LC709203F_CMD_APA 				0x0B
#define LC709203F_CMD_RSOC 				0x0D
#define LC709203F_CMD_CELLITE 			0x0F
#define LC709203F_CMD_ICVERSION 		0x11
#define LC709203F_CMD_BATTPROF 			0x12
#define LC709203F_CMD_ALARMRSOC 		0x13
#define LC709203F_CMD_ALARMVOLT 		0x14
#define LC709203F_CMD_POWERMODE 		0x15
#define LC709203F_CMD_STATUSBIT 		0x16
#define LC709203F_CMD_PARAMETER 		0x1A

/*
	Example #1 - LC709203_HandleTypedef

	LC709203_HandleTypedef lc709203;
	lc709203.addr = LC709203F_I2CADDR_DEFAULT;
	lc709203.power = LC709203F_POWER_OPERATE;
	lc709203.adjust= LC709203F_APA_500MAH;
	lc709203.tempmode = LC709203F_TEMPERATURE_THERMISTOR;
	lc709203.i2c = &hi2c1;
 */

/*!  Battery temperature source */
typedef enum {
  LC709203F_TEMPERATURE_I2C 			= 0x0000,
  LC709203F_TEMPERATURE_THERMISTOR 		= 0x0001,
} LC709203_tempmode_t;

/*!  Chip power state */
typedef enum {
  LC709203F_POWER_OPERATE 				= 0x0001,
  LC709203F_POWER_SLEEP 				= 0x0002,
} LC709203_powermode_t;

/*!  Approx battery pack size */
typedef enum {
  LC709203F_APA_100MAH 					= 0x08,
  LC709203F_APA_200MAH 					= 0x0B,
  LC709203F_APA_500MAH 					= 0x10,
  LC709203F_APA_1000MAH 				= 0x19,
  LC709203F_APA_2000MAH 				= 0x2D,
  LC709203F_APA_3000MAH 				= 0x36,
} LC709203_adjustment_t;

typedef enum {
  LC709203F_STATUS_FAILED				= -1,
  LC709203F_STATUS_OK 					= 0
} LC709203_status_t;

typedef struct {
    uint16_t addr;						// ic2 address
    LC709203_adjustment_t adjust;		// battery size
    LC709203_powermode_t power;			// power setting
    LC709203_tempmode_t tempmode;		// temperature reading mode
    uint16_t arsoc;						// alert rsoc percent
    uint16_t avoltage;					// alert voltage
    uint16_t version;					// IC version
    uint16_t voltage;					// current voltage
    uint16_t celllife;					// current battery percent
    float temperature;					// current temperature
    I2C_HandleTypeDef* i2c;				// i2c stm32 handler
} LC709203_HandleTypedef;

/* supporting functions */
uint8_t lc_check_crc(uint8_t *rec_values, uint8_t len);
uint16_t read_lc709(LC709203_HandleTypedef *config, uint8_t command);
int write_lc709(LC709203_HandleTypedef *config, uint8_t command, uint16_t data);
long map(long x, long in_min, long in_max, long out_min, long out_max);

/* primary functions */
LC709203_status_t LC709203_Init(LC709203_HandleTypedef *config);
LC709203_status_t LC709203_Read_IC_Version(LC709203_HandleTypedef *config);
LC709203_status_t LC709203_Read_Voltage(LC709203_HandleTypedef *config);
LC709203_status_t LC709203_Read_Cell_Percent(LC709203_HandleTypedef *config);
LC709203_status_t LC709203_Read_Cell_Temperature(LC709203_HandleTypedef *config);

#endif
