#ifndef SMU_H
#define SMU_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "data.hpp"

class SMU {
public:
	SMU(UART_HandleTypeDef& huart) : huart(huart) {}

	void run_voltage_sweep();

	UART_HandleTypeDef& get_uart_handle() {
		return huart;
	}

private:
	UART_HandleTypeDef& huart;
	osSemaphoreId_t& data_packet_sem;
};

#endif // SMU_H
