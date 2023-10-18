/*
 * MPL3115A2.cpp
 *
 *  Created on: Oct 3, 2023
 *      Author: adarsh
 */

#include "MPL3115A2.h"

HAL_StatusTypeDef result;

void configure_MPL3115A2(I2C_HandleTypeDef &i2c_device) {

	// Data to set the control register 0b00111000 [ALT RAW OS2 OS1 OS0 0 OST SB]
	uint8_t config_data = 0x39;

	result = HAL_I2C_Mem_Write(&i2c_device, MPL3115A2_WRITE_ADDR, CTRL_REG_1,
			I2C_MEMADD_SIZE_8BIT, &config_data, 1, HAL_MAX_DELAY);
}

float read_baromateric_pressure(I2C_HandleTypeDef &i2c_device) {
	uint8_t pressure_data[3];
	int32_t pressure_raw;

	// Read pressure data from the sensor (3 bytes)
	result = HAL_I2C_Mem_Read(&i2c_device, MPL3115A2_READ_ADDR, PRESSURE_MSB,
	                          I2C_MEMADD_SIZE_8BIT, pressure_data, 3, HAL_MAX_DELAY);

	// Combine the data bytes to get the pressure value (20 bits)
	pressure_raw = ((int32_t)pressure_data[0] << 12) | ((int32_t)pressure_data[1] << 4) | ((pressure_data[2] >> 4) & 0x0F);

	// Convert to a double value in Pascals (adjust as needed)
	float pressure_pascals = (float) pressure_raw / 4.0;

    return pressure_pascals;
}

error_code i2c_write_register(I2C_HandleTypeDef &i2c_device, uint32_t reg_addr, uint8_t* data, uint16_t num_bytes) {
	HAL_StatusTypeDef error;
	if (num_bytes == 1) {
		error = HAL_I2C_Mem_Write(&i2c_device, MPL3115A2_WRITE_ADDR << 1, reg_addr,
						          I2C_MEMADD_SIZE_8BIT, data, num_bytes, HAL_MAX_DELAY);
	} else if (num_bytes == 2) {
		error = HAL_I2C_Mem_Write(&i2c_device, MPL3115A2_WRITE_ADDR << 1, reg_addr,
						          I2C_MEMADD_SIZE_16BIT, data, num_bytes, HAL_MAX_DELAY);
	} else {
		return ERROR_INVALID_PARAMETER;
	}
	if (error != HAL_OK) {
		return ERROR_HAL;
	}
	return NO_ERROR;
}

error_code i2c_read_register(I2C_HandleTypeDef &i2c_device, uint32_t reg_addr, uint8_t* buff, uint16_t num_bytes) {

	HAL_StatusTypeDef error = HAL_I2C_Mem_Read (&i2c_device, MPL3115A2_READ_ADDR, reg_addr,
					  	  	  	  	  	  	  	I2C_MEMADD_SIZE_8BIT, buff, num_bytes, HAL_MAX_DELAY);
	if (error != HAL_OK) {
		return ERROR_HAL;
	}
	return NO_ERROR;
}

