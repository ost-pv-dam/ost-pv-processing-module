#ifndef MPL3115A2_H
#define MPL3115A2_H

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

class MPL3115A2 {
public:
	MPL3115A2(I2C_HandleTypeDef &i2c_device) : i2c_device(i2c_device) {}

	void init();
	float read_baromateric_pressure();
private:
	I2C_HandleTypeDef &i2c_device;
};



#endif // MPL3115A2_H
