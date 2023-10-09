#ifndef SHT30_H
#define SHT30_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <assert.h>

#define SHT30_I2C_ADDR 0x44
#define SHT30_I2C_TIMEOUT 30
#define SHT30_COMMAND_MEASURE_HIGHREP_STRETCH 0x2c06
#define SHT30_COMMAND_READ_STATUS 0xf32d

typedef struct SHT30_t {
	I2C_HandleTypeDef* hi2c;
} SHT30_t;

uint8_t SHT30_init(SHT30_t*);
uint8_t SHT30_send_cmd(SHT30_t*, uint16_t cmd);
uint8_t SHT30_read_temp_humidity(SHT30_t* sht, float* temperature, float* humidity);

#endif // SHT30_H
