/*
 * MPL3115A2.h
 *
 *  Created on: Oct 3, 2023
 *      Author: adarsh
 */

#ifndef INC_MPL3115A2_H_
#define INC_MPL3115A2_H_

#include "stm32f4xx_hal.h"

// MPL3115A2 I2C address
#define MPL3115A2_ADDRESS 0x60

// MPL3115A2 Control Registers
#define CTRL_REG_1		0x26
#define PRESSURE_MSB    0x01
#define PRESSURE_CSB    0x02
#define PRESSURE_LSB    0x03



extern I2C_HandleTypeDef hi2c1;

void configure_MPL3115A2();

float read_baromateric_pressure();

void i2c_write_register(uint32_t reg_addr, uint16_t data);

uint16_t i2c_read_register(uint32_t reg_addr);



#endif /* INC_MPL3115A2_H_ */
