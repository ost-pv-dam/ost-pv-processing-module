#include "MPL3115A2.hpp"



HAL_StatusTypeDef MPL3115A2::init() {

	// Data to set the control register 0b00111000 [ALT RAW OS2 OS1 OS0 0 OST SB]
	uint8_t config_data = 0x39;

	HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&i2c_device, MPL3115A2_WRITE_ADDR, CTRL_REG_1,
			I2C_MEMADD_SIZE_8BIT, &config_data, 1, HAL_MAX_DELAY);

	return result;
}

double MPL3115A2::read_baromateric_pressure() {
	uint8_t pressure_data[3];
	int32_t pressure_raw;

	HAL_StatusTypeDef result;
	// Read pressure data from the sensor (3 bytes)
	result = HAL_I2C_Mem_Read(&i2c_device, MPL3115A2_READ_ADDR, PRESSURE_MSB,
	                          I2C_MEMADD_SIZE_8BIT, pressure_data, 3, HAL_MAX_DELAY);

	// Combine the data bytes to get the pressure value (20 bits)
	pressure_raw = ((int32_t)pressure_data[0] << 12) | ((int32_t)pressure_data[1] << 4) | ((pressure_data[2] >> 4) & 0x0F);

	// Convert to a double value in Pascals (adjust as needed)
	double pressure_pascals = (double) pressure_raw / 4.0;

    return pressure_pascals;
}
