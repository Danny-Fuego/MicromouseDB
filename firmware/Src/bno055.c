#include "bno055.h"

#ifndef GYRO_RST_GPIO_Port
#error "GYRO_RST_GPIO_Port not defined. MQKE SURE PA12 is labeled GYRO_RST in CubeMX."
#endif

#ifndef GYRO_RST_Pin
#error "GYRO_RST_Pin not defined. Make sure PA12 is labeled GYRO_RST in CubeMX."
#endif

static BNO055_Status BNO055_WriteByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value){
	if(HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY) != HAL_OK){
		return BNO055_ERR_I2C;
	}
	return BNO055_OK;
}

static BNO055_Status BNO055_ReadBytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len){
	if (HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) != HAL_OK){
		return BNO055_ERR_I2C;
	}
	return BNO055_OK;
}

BNO055_Status BNO055_Reset(void){
	HAL_GPIO_WritePin(GYRO_RST_GPIO_Port, GYRO_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GYRO_RST_GPIO_Port, GYRO_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(700);

	return BNO055_OK;
}

BNO055_Status BNO055_ReadChipID(I2C_HandleTypeDef *hi2c, uint8_t *chip_id){
	return BNO055_ReadBytes(hi2c, BNO055_REG_CHIP_ID, chip_id, 1);
}

BNO055_Status BNO055_SetMode(I2C_HandleTypeDef *hi2c, uint8_t mode){
	BNO055_Status st;

	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
	if (st != BNO055_OK) return st;
	HAL_Delay(25);

	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, mode);
	if (st != BNO055_OK) return st;
	HAL_Delay(30);

	return BNO055_OK;
}

BNO055_Status BNO055_Init(I2C_HandleTypeDef *hi2c){
	BNO055_Status st;
	uint8_t chip_id = 0;

	HAL_GPIO_WritePin(GYRO_RST_GPIO_Port, GYRO_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	st = BNO055_ReadChipID(hi2c, &chip_id);
	if (st != BNO055_OK) return st;
	if (chip_id != BNO055_CHIP_ID_VALUE) return BNO055_ERR_CHIP_ID;

	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
	if (st != BNO055_OK) return st;
	HAL_Delay(25);

	st = BNO055_WriteByte(hi2c, BNO055_REG_PAGE_ID, 0X00);
	if (st != BNO055_OK) return st;

	st = BNO055_WriteByte(hi2c, BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL);
	if (st != BNO055_OK) return st;
	HAL_Delay(10);

	// Units:
	// orientation = Windows
	// temp = C
	// Euler = degrees
	// gyro = dps
	// accel = m/s^2

	st = BNO055_WriteByte(hi2c, BNO055_REG_UNIT_SEL, 0x00);
	if (st != BNO055_OK) return st;

	st = BNO055_WriteByte(hi2c, BNO055_REG_SYS_TRIGGER, 0x00);
	if (st != BNO055_OK) return st;
	HAL_Delay(10);

	// Absolute heading
	st = BNO055_WriteByte(hi2c, BNO055_REG_OPR_MODE, BNO055_MODE_NDOF);
	if (st != BNO055_OK) return st;
	HAL_Delay(30);

	return BNO055_OK;
}

BNO055_Status BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, BNO055_Euler *euler){
	uint8_t buf[6];
	int16_t raw_heading;
	int16_t raw_roll;
	int16_t raw_pitch;
	BNO055_Status st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_EUL_HEADING_LSB, buf, 6);
	if (st != BNO055_OK) return st;

	raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
	raw_roll = (int16_t)((buf[3] << 8) | buf[2]);
	raw_pitch = (int16_t)((buf[5] << 8) | buf[4]);

	// BNO055 Euler output scale: 1 degree = 16 LSB
	euler->heading_deg = raw_heading / 16.0f;
	euler->roll_deg = raw_roll / 16.0f;
	euler->pitch_deg = raw_pitch / 16.0f;

	return BNO055_OK;
}

BNO055_Status BNO055_ReadHeading(I2C_HandleTypeDef *hi2c, float *heading_deg){
	BNO055_Euler euler;
	BNO055_Status st = BNO055_ReadEuler(hi2c, &euler);

	if (st != BNO055_OK) return st;

	*heading_deg = euler.heading_deg;
	return BNO055_OK;
}

BNO055_Status BNO055_ReadCalibration(I2C_HandleTypeDef *hi2c, BNO055_Calib *calib){
	uint8_t raw = 0;
	BNO055_Status st = BNO055_ReadBytes(hi2c, BNO055_REG_CALIB_STAT, &raw, 1);

	if (st != BNO055_OK) return st;

	calib->raw = raw;
	calib->sys = (raw >> 6) & 0x03;
	calib->gyro = (raw >> 4) & 0x03;
	calib->accel = (raw >> 2) & 0x03;
	calib->mag = raw & 0x03;

	return BNO055_OK;
}

BNO055_Status BNO055_ReadGyro(I2C_HandleTypeDef *hi2c, BNO055_Gyro *gyro){
	uint8_t buf[6];
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;
	BNO055_Status st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_GYR_DATA_X_LSB, buf, 6);
	if (st != BNO055_OK) return st;

	raw_x = (int16_t)((buf[1] << 8) | buf[0]);
	raw_y = (int16_t)((buf[3] << 8) | buf[2]);
	raw_z = (int16_t)((buf[5] << 8) | buf[4]);

	gyro->x_dps = raw_x / 16.0f;
	gyro->y_dps = raw_y / 16.0f;
	gyro->z_dps = raw_z / 16.0f;

	return BNO055_OK;
}

BNO055_Status BNO055_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *sys_stat, uint8_t *sys_err){
	BNO055_Status st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_SYS_STAT, sys_stat, 1);
	if (st != BNO055_OK) return st;

	st = BNO055_ReadBytes(hi2c, BNO055_REG_SYS_ERR, sys_err, 1);
	if (st != BNO055_OK) return st;

	return BNO055_OK;
}

float BNO055_AngleError(float target_deg, float current_deg){
	float err = target_deg - current_deg;

	while (err > 180.0f) err -= 360.0f;
	while (err < -180.0f) err += 360.0f;

	return err;
}
