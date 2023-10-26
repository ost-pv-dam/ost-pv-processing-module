#ifndef ESP32_H
#define ESP32_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string>
#include <queue>
#include "data.hpp"
#include "logger.hpp"
#include <sstream>

const std::string ESP_OK = "OK\r\n";
const std::string ESP_WIFI_OK = "WIFI GOT IP\r\n";
constexpr uint16_t ESP_RESP_LEN = 50;
const std::string ESP_API_HEADER = "x-api-key: test";
const std::string ESP_READY = "\r\n>";

class ESP32 {
public:
	ESP32(UART_HandleTypeDef& huart, osMessageQueueId_t& external_queue, osSemaphoreId_t& data_ready_sem) :
		huart(huart), external_queue(external_queue), data_ready_sem(data_ready_sem) {}

	int init();
	void send_cmd(std::string cmd, bool crlf = true);
	std::string poll(int num_bytes, uint32_t timeout = 100);

	void send_data_packet(DataPacket& data);

	void process_incoming_bytes(char* buf, int num_bytes);
	std::string consume_message();
	void flush();

private:
	UART_HandleTypeDef& huart;

	std::string current_message = "";
	std::queue<std::string> messages;

	osMessageQueueId_t& external_queue;
	osSemaphoreId_t& data_ready_sem;
};

#endif // ESP32_H
