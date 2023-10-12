#include "SHT30.h"

static uint8_t calculate_crc(const uint8_t *data, size_t length)
{
	uint8_t crc = 0xff;
	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (size_t j = 0; j < 8; j++) {
			if ((crc & 0x80u) != 0) {
				crc = (uint8_t)((uint8_t)(crc << 1u) ^ 0x31u);
			} else {
				crc <<= 1u;
			}
		}
	}
	return crc;
}

static uint16_t uint8_to_uint16(uint8_t msb, uint8_t lsb)
{
	return (uint16_t)((uint16_t)msb << 8u) | lsb;
}

uint8_t SHT30_init(SHT30_t* sht) {
	assert(sht->hi2c->Init.NoStretchMode == I2C_NOSTRETCH_DISABLE);

	uint8_t status_reg_and_checksum[3];
	if (HAL_I2C_Mem_Read(sht->hi2c, SHT30_I2C_ADDR << 1u, SHT30_COMMAND_READ_STATUS, 2, (uint8_t*)&status_reg_and_checksum,
					  sizeof(status_reg_and_checksum), SHT30_I2C_TIMEOUT) != HAL_OK) {
		return 0;
	}

	uint8_t calculated_crc = calculate_crc(status_reg_and_checksum, 2);

	if (calculated_crc != status_reg_and_checksum[2]) {
		return 0;
	}

	return 1;
}

uint8_t SHT30_send_cmd(SHT30_t* sht, uint16_t cmd) {
	uint8_t command_buffer[2] = {(uint8_t)((cmd & 0xff00u) >> 8u), uint8_t(cmd & 0xffu)};

	if (HAL_I2C_Master_Transmit(sht->hi2c, SHT30_I2C_ADDR << 1u, command_buffer, sizeof(command_buffer),
								SHT30_I2C_TIMEOUT) != HAL_OK) {
		return 0;
	}

	return 1;

}

uint8_t SHT30_read_temp_humidity(SHT30_t* sht, float* temperature, float* humidity) {
	SHT30_send_cmd(sht, SHT30_COMMAND_MEASURE_HIGHREP_STRETCH);

	HAL_Delay(1);

	uint8_t buffer[6];
	if (HAL_I2C_Master_Receive(sht->hi2c, SHT30_I2C_ADDR << 1u, buffer, sizeof(buffer), SHT30_I2C_TIMEOUT) != HAL_OK) {
		return 0;
	}

	uint8_t temperature_crc = calculate_crc(buffer, 2);
	uint8_t humidity_crc = calculate_crc(buffer + 3, 2);
	if (temperature_crc != buffer[2] || humidity_crc != buffer[5]) {
		return 0;
	}

	uint16_t temperature_raw = uint8_to_uint16(buffer[0], buffer[1]);
	uint16_t humidity_raw = uint8_to_uint16(buffer[3], buffer[4]);

	*temperature = -45.0f + 175.0f * (float)temperature_raw / 65535.0f;
	*humidity = 100.0f * (float)humidity_raw / 65535.0f;

	return 1;
}
