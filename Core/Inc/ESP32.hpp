#ifndef ESP32_H
#define ESP32_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string>
#include <queue>
#include "logger.hpp"

const std::string ESP_OK = "OK\r\n";
const std::string ESP_WIFI_OK = "WIFI GOT IP\r\n";
constexpr uint16_t ESP_RESP_LEN = 50;
const std::string ESP_API_HEADER = "x-api-key: test";


class ESP32 {
public:
	ESP32(UART_HandleTypeDef& huart, osSemaphoreId_t& messages_sem) :
		huart(huart), messages_sem(messages_sem) {}

	int init();
	void send_cmd(std::string cmd, bool crlf = true);
	std::string poll(int num_bytes, uint32_t timeout = 100);

	void process_incoming_bytes(char* buf, int num_bytes);
	std::string consume_message();

private:
	UART_HandleTypeDef& huart;

	std::string current_message = "";
	std::queue<std::string> messages;
	osSemaphoreId_t& messages_sem;
};

#endif // ESP32_H
