#ifndef SMU_H
#define SMU_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "data.hpp"
#include <string>

class SMU {
public:
    SMU() = default;
	SMU(UART_HandleTypeDef* huart) : huart(huart) {}

	void config_voltage_sweep();
	void run_voltage_sweep();

	void send_scpi(std::string scpi);

	UART_HandleTypeDef* get_uart_handle() {
		return huart;
	}

private:
	UART_HandleTypeDef* huart;
};

#endif // SMU_H
