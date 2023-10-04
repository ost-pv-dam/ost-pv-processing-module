/*
 * MPL3115A2.cpp
 *
 *  Created on: Oct 3, 2023
 *      Author: adarsh
 */

#include "MPL3115A2.h"

void configure_MPL3115A2(I2C_HandleTypeDef &i2c_device) {

	// Data to set the control register 0b00111000 [ALT RAW OS2 OS1 OS0 0 OST SB]
	uint8_t config_data = 0x38;

	HAL_I2C_Mem_Write(&i2c_device, MPL3115A2_WRITE_ADDR, CTRL_REG_1,
					  1, &config_data, 1, HAL_MAX_DELAY);
}

double read_baromateric_pressure(I2C_HandleTypeDef &i2c_device) {

    uint8_t data[3];
    uint32_t pressure;

    // Read pressure data from the sensor
    HAL_I2C_Mem_Read(&i2c_device, MPL3115A2_WRITE_ADDR, PRESSURE_MSB,
    				 1, data, 3, HAL_MAX_DELAY);

    // Combine the data bytes to get the pressure value
    pressure = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

    return (float) pressure;

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
