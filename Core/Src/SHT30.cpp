#include <SHT30.hpp>

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

bool SHT30::init() {
	assert(hi2c.Init.NoStretchMode == I2C_NOSTRETCH_DISABLE);

	uint8_t status_reg_and_checksum[3];
	if (HAL_I2C_Mem_Read(&hi2c, SHT30_I2C_ADDR << 1u, SHT30_COMMAND_READ_STATUS, 2, (uint8_t*)&status_reg_and_checksum,
					  sizeof(status_reg_and_checksum), SHT30_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	uint8_t calculated_crc = calculate_crc(status_reg_and_checksum, 2);

	if (calculated_crc != status_reg_and_checksum[2]) {
		return false;
	}

	return true;
}

bool SHT30::send_cmd(uint16_t cmd) {
	uint8_t command_buffer[2] = {(uint8_t)((cmd & 0xff00u) >> 8u), uint8_t(cmd & 0xffu)};

	if (HAL_I2C_Master_Transmit(&hi2c, SHT30_I2C_ADDR << 1u, command_buffer, sizeof(command_buffer),
								SHT30_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	return true;

}

bool SHT30::read_temp_humidity(double& temperature, double& humidity) {
	send_cmd(SHT30_COMMAND_MEASURE_HIGHREP_STRETCH);

	HAL_Delay(1);

	uint8_t buffer[6];
	if (HAL_I2C_Master_Receive(&hi2c, SHT30_I2C_ADDR << 1u, buffer, sizeof(buffer), SHT30_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	uint8_t temperature_crc = calculate_crc(buffer, 2);
	uint8_t humidity_crc = calculate_crc(buffer + 3, 2);
	if (temperature_crc != buffer[2] || humidity_crc != buffer[5]) {
		return false;
	}

	uint16_t temperature_raw = uint8_to_uint16(buffer[0], buffer[1]);
	uint16_t humidity_raw = uint8_to_uint16(buffer[3], buffer[4]);

	temperature = -45.0 + 175.0 * (double)temperature_raw / 65535.0;
	humidity = 100.0 * (double)humidity_raw / 65535.0;

	return true;
}
