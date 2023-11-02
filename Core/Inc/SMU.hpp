#ifndef SMU_H
#define SMU_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "data.hpp"
#include <cstring>

#define SMU_MAX_RESP_LENGTH 512

class SMU {
public:
	SMU(UART_HandleTypeDef& huart) : huart(huart) {}

	void init_voltage_sweep();

	void run_voltage_sweep();

	UART_HandleTypeDef& get_uart_handle() {
		return huart;
	}

private:
	UART_HandleTypeDef& huart;
};

#endif // SMU_H
