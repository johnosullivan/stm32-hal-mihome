#ifndef _CCS811_H_
#define _CCS811_H_

#include "config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CCS811_I2C_ADDR_0 	0x5B

// I2C Registers
#define STATUS 				0x00	// APPLICATION & BOOTLOADER
#define MEAS_MODE 			0x01	// APPLICATION ONLY
#define ALG_RESULT_DATA 	0x02	// APPLICATION ONLY
#define RAW_DATA 			0x03	// APPLICATION ONLY
#define ENV_DATA 			0x05	// APPLICATION ONLY
#define NTC 				0x06	// APPLICATION ONLY
#define THRESHOLDS 			0x10	// APPLICATION ONLY
#define BASELINE 			0x11	// APPLICATION ONLY
#define HW_ID 				0x20	// APPLICATION & BOOTLOADER
#define HW_VERSION 			0x21	// APPLICATION & BOOTLOADER
#define FW_BOOT_VERSION 	0x23 	// APPLICATION & BOOTLOADER
#define FW_APP_VERSION 		0x24	// APPLICATION & BOOTLOADER
#define ERROR_ID 			0xE0	// APPLICATION & BOOTLOADER
#define APP_ERASE 			0xF1	// BOOTLOADER ONLY
#define APP_DATA 			0xF2	// BOOTLOADER ONLY
#define APP_VERIFY 			0xF3	// BOOTLOADER ONLY
#define APP_START 			0xF4	// BOOTLOADER ONLY
#define SW_RESET 			0xFF	// APPLICATION & BOOTLOADER

// STATUS Registers
#define FW_MODE(X)			((X & 0x1) << 7)
#define FW_MODE_BOOT		0x0	    // Allows new firmware to be loaded
#define FW_MODE_APPL		0x1	    // CCS811 is ready to take ADC measurements
#define APP_VALID(X)		((X & 0x1) << 4)
#define APP_VALID_NOAPP		0x0	    // No application firmware loaded
#define APP_VALID_VALID 	0x1	    // Valid application firmware loaded
#define DATA_READY(X) 		((X & 0x1) << 3)
#define DATA_READY_NO 		0x0	    // No new data samples are ready
#define DATA_READY_NEW 		0x1	    // A new data sample is ready in ALG_RESULT_DATA, this bit is cleared when ALG_RESULT_DATA is read on the I2C interface
#define ERROR 				0x00

// MEASSURE_MODE Registers
#define DRIVE_MODE(X)		((X & 0x7) << 4)
#define DRIVE_MODE_MODE0 	0x0     // Idle (Measurements are disabled in this mode) 001
#define DRIVE_MODE_MODE1 	0x1     // Constant power mode, IAQ measurement every second 010
#define DRIVE_MODE_MODE2 	0x2     // Pulse heating mode IAQ measurement every 10 seconds 011
#define DRIVE_MODE_MODE3 	0x3     // Low power pulse heating mode IAQ measurement every 60 seconds 100
#define DRIVE_MODE_MODE4 	0x4     // Constant power mode, sensor measurement every 250ms
#define INT_DATARDY 		0x03
#define INT_THRESH 			0x02

// CCS811_HandleType 
typedef struct {
    uint16_t addr;
    I2C_HandleTypeDef* i2c;
} CCS811_HandleTypedef;

bool CCS811_Init(CCS811_HandleTypedef *dev);
void CCS811_Measure_CO2_TVOC(CCS811_HandleTypedef *dev, uint32_t *co2, uint32_t *tvoc);
void CCS811_Temp_RH_Compensation(CCS811_HandleTypedef *dev, float temp, float rh);

#ifdef __cplusplus
}
#endif

#endif /* _CCS811_H_ */
