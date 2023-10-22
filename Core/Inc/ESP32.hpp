#ifndef ESP32_H
#define ESP32_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string>
#include <queue>

const std::string ESP_ECHO_OFF_OK = "ATE0\r\n\r\nOK\r\n";
const std::string ESP_OK = "\r\nOK\r\n";

class ESP32 {
public:
	ESP32(UART_HandleTypeDef& huart, osSemaphoreId_t& messages_sem) :
		huart(huart), messages_sem(messages_sem) {}

	int init();
	void send_cmd(std::string cmd);
	std::string poll(int num_bytes);

	void process_incoming_bytes(char* buf, int num_bytes);
	std::string consume_message();

private:
	UART_HandleTypeDef& huart;

	std::string current_message = "";
	std::queue<std::string> messages;
	osSemaphoreId_t& messages_sem;
};

#endif // ESP32_H
