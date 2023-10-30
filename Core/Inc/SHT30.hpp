#ifndef SHT30_H
#define SHT30_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <assert.h>

constexpr uint16_t SHT30_I2C_ADDR = 0x44;
constexpr uint16_t  SHT30_I2C_TIMEOUT = 30;
constexpr uint16_t SHT30_COMMAND_MEASURE_HIGHREP_STRETCH = 0x2c06;
constexpr uint16_t SHT30_COMMAND_READ_STATUS = 0xf32d;

class SHT30 {
public:
	SHT30(I2C_HandleTypeDef& hi2c) : hi2c(hi2c) {}

	bool init();
	bool send_cmd(uint16_t cmd);
	bool read_temp_humidity(double& temperature, double& humidity);

private:
	I2C_HandleTypeDef& hi2c;
};

#endif // SHT30_H
