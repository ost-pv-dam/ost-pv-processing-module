#ifndef ESP32_H
#define ESP32_H

#include "stm32f4xx_hal.h"
#include <string>

const std::string ESP_ECHO_OFF_OK = "ATE0\r\n\r\nOK\r\n";
const std::string ESP_OK = "\r\nOK\r\n";

class ESP32 {
public:
	ESP32(UART_HandleTypeDef& huart) : huart(huart) {}

	int init();
	void send_cmd(std::string cmd);
	std::string poll(int num_bytes);

private:
	UART_HandleTypeDef& huart;
};

#endif // ESP32_H
