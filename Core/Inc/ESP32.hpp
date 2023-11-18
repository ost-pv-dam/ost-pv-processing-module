#ifndef ESP32_H
#define ESP32_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string>
#include <queue>
#include "data.hpp"
#include "logger.hpp"
#include <sstream>

constexpr const char* ESP_OK = "OK\r\n";
const std::string ESP_WIFI_OK = "WIFI GOT IP\r\n";
constexpr uint16_t ESP_RESP_LEN = 50;
constexpr size_t LONG_CMD_THRESHOLD = 1000;
const std::string ESP_API_HEADER = "x-api-key: test";
constexpr const char* ESP_READY = "\r\n>";

constexpr size_t ESP_MAX_RESP_LENGTH = 512; // probably good enough?
constexpr size_t ESP_PHOTO_CHUNK_LENGTH = 1024;

class ESP32 {
public:
	ESP32(UART_HandleTypeDef& huart, osMessageQueueId_t& external_queue, osSemaphoreId_t& data_ready_sem) :
		huart(huart), external_queue(external_queue), data_ready_sem(data_ready_sem) {}

	int init();
	void send_cmd(const std::string& cmd, bool crlf = true);
	void send_raw(std::unique_ptr<char[]>&& cmd);
    void send_raw(std::array<uint8_t, ESP_PHOTO_CHUNK_LENGTH>& buffer);
	std::string poll(int num_bytes, uint32_t timeout = 1000);

	void send_data_packet_start(size_t json_length,
                                const std::string& url = "https://api.umich-ost-pv-dam.org:5050/api/v1/sensorCellData",
                                const std::string& content_type = "application/json");

	void send_data_packet_start(size_t json_length,
	                                   const std::string& url,
	                                   const std::string& content_type, const time_t timestamp);

	void push_message(std::string msg);
	std::string consume_message();
	void flush();

	UART_HandleTypeDef& get_uart_handle() {
		return huart;
	}

private:
	UART_HandleTypeDef& huart;
	std::queue<std::string> messages;

	osSemaphoreId_t& external_queue;
	osSemaphoreId_t& data_ready_sem;
};

#endif // ESP32_H
