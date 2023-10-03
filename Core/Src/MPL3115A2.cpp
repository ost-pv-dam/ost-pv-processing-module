/*
 * MPL3115A2.cpp
 *
 *  Created on: Oct 3, 2023
 *      Author: adarsh
 */


I2C_HandleTypeDef hi2c1;

void configure_MPL3115A2() {

	// Data to set the control register 0b00111000 [ALT RAW OS2 OS1 OS0 0 OST SB]
	uint8_t config_data = 0x38;

	HAL_I2C_Mem_Write(&hi2c1, MPL3115A2_ADDRESS << 1, CTRL_REG1, 1, &config_data, 1, HAL_MAX_DELAY);
}

float read_baromateric_pressure() {

    uint8_t data[3];
    uint32_t pressure;

    // Read pressure data from the sensor
    HAL_I2C_Mem_Read(&hi2c1, MPL3115A2_ADDRESS << 1, PRESSURE_MSB, 1, data, 3, HAL_MAX_DELAY);

    // Combine the data bytes to get the pressure value
    pressure = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

    return (float) pressure;

}

void i2c_write_register(uint32_t reg_addr, uint16_t data) {

    uint8_t data_buffer[2];
    data_buffer[0] = (data >> 8) & 0xFF; 	// MSB
    data_buffer[1] = data & 0xFF;        	// LSB

	HAL_I2C_Mem_Write(&hi2c1, MPL3115A2_ADDRESS << 1, reg_addr, 1, &data_buffer, 1, HAL_MAX_DELAY);
}

uint16_t i2c_read_register(uint32_t reg_addr) {


}
