/*
 * MPL3115A2.h
 *
 *  Created on: Oct 3, 2023
 *      Author: adarsh
 */

#ifndef INC_MPL3115A2_H_
#define INC_MPL3115A2_H_

#include <solar_processing_structs.h>
#include "stm32f4xx_hal.h"

// MPL3115A2 I2C address
#define MPL3115A2_ADDR 0x60
#define MPL3115A2_WRITE_ADDR 	(MPL3115A2_ADDR << 1)
#define MPL3115A2_READ_ADDR 	((MPL3115A2_ADDR << 1) | 0x01)


// MPL3115A2 Control Registers
#define CTRL_REG_1		0x26
#define PRESSURE_MSB    0x01
#define PRESSURE_CSB    0x02
#define PRESSURE_LSB    0x03


void configure_MPL3115A2(I2C_HandleTypeDef &i2c_device);

float read_baromateric_pressure(I2C_HandleTypeDef &i2c_device);

/* Writes specified register via I2C. Pass in the starting ptr to a uint8_t buffer, with how many bytes you want to write */
error_code i2c_write_register(I2C_HandleTypeDef &i2c_device, uint32_t reg_addr, uint8_t *data, uint16_t num_bytes);

/* Reads specified register via I2C. Pass in the starting ptr to a uint8_t buffer, with how many bytes you want to read */
error_code i2c_read_register(I2C_HandleTypeDef &i2c_device, uint32_t reg_addr, uint8_t *buff, uint16_t num_bytes);

#endif /* INC_MPL3115A2_H_ */

