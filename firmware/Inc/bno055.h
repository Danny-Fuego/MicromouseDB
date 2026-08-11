#ifndef BNO055_H
#define BNO055_H

#include "main.h"
#include <stdint.h>

#define BNO055_I2C_ADDR     (0x28 << 1)

// Page 0 registers
#define BNO055_REG_CHIP_ID  0X00
#define BNO055_REG_PAGE_ID  0X07
#define BNO055_REG_EUL_HEADING_LSB 0x1A
#define BNO055_REG_EUL_ROLL_LSB 0x1C
#define BNO055_REG_EUL_PITCH_LSB 0x1E
#define BNO055_REG_CALIB_STAT 0x35
#define BNO055_REG_SYS_STAT 0x39
#define BNO055_REG_SYS_ERR 0x3A
#define BNO055_REG_UNIT_SEL 0X3B
#define BNO055_REG_OPR_MODE 0x3D
#define BNO055_REG_PWR_MODE 0x3E
#define BNO055_REG_SYS_TRIGGER 0X3F
#define BNO055_REG_GYR_DATA_X_LSB 0x14
#define BNO055_REG_GYR_DATA_Y_LSB 0x16
#define BNO055_REG_GYR_DATA_Z_LSB 0x18

// Chip ID
#define BNO055_CHIP_ID_VALUE 0xA0

// Operation modes
#define BNO055_MODE_CONFIG 0x00
#define BNO055_MODE_IMUPLUS 0x08
#define BNO055_MODE_NDOF 0x0C

// Power modes
#define BNO055_PWR_NORMAL 0x00

typedef enum{
	BNO055_OK = 0,
	BNO055_ERR_I2C,
	BNO055_ERR_CHIP_ID
} BNO055_Status;

typedef struct{
	float heading_deg;
	float roll_deg;
	float pitch_deg;
} BNO055_Euler;

typedef struct{
	uint8_t raw;
	uint8_t sys;
	uint8_t gyro;
	uint8_t accel;
	uint8_t mag;
} BNO055_Calib;

typedef struct {
	float x_dps;
	float y_dps;
	float z_dps;
} BNO055_Gyro;

BNO055_Status BNO055_Reset(void);
BNO055_Status BNO055_ReadChipID(I2C_HandleTypeDef *hi2c, uint8_t *chip_id);
BNO055_Status BNO055_Init(I2C_HandleTypeDef *hi2c);
BNO055_Status BNO055_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode);
BNO055_Status BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, BNO055_Euler *euler);
BNO055_Status BNO055_ReadHeading(I2C_HandleTypeDef *hi2c, float *heading_deg);
BNO055_Status BNO055_ReadCalibration(I2C_HandleTypeDef *hi2c, BNO055_Calib *calib);
BNO055_Status BNO055_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *sys_stat, uint8_t *sys_err);
BNO055_Status BNO055_ReadGyro(I2C_HandleTypeDef *hi2c, BNO055_Gyro *gyro);

float BNO055_AngleError(float target_deg, float current_deg);

#endif
