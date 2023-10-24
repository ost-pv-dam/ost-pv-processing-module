#include "ESP32.hpp"

void ESP32::send_cmd(std::string cmd, bool crlf) {
	if (crlf)
		cmd += "\r\n";

	HAL_UART_Transmit(&huart, (uint8_t*) cmd.c_str(), cmd.length(), 100);
}

int ESP32::init() {
	// AT+RESTORE restarts the module, so the timing is hard to get right
	// will not include for now

	send_cmd("ATE0"); // disable echo

	std::string resp = poll(ESP_RESP_LEN);
	if (resp.find(ESP_OK) == std::string::npos) {
		Logger::getInstance()->error("ATE0: " + resp);
		return 0; // init FAIL
	}

	HAL_Delay(100);

	// set UART parameters
	// NOTE: if you change the baud rate, you will need to change the STM baudrate too
	send_cmd("AT+UART_CUR=9600,8,1,0,0");
	resp = poll(ESP_RESP_LEN);
	if (resp.find(ESP_OK) == std::string::npos) {
		Logger::getInstance()->error("UART_CUR: " + resp);
		return 0; // init FAIL
	}

	HAL_Delay(100);

	send_cmd("AT+HTTPCHEAD=" + std::to_string(ESP_API_HEADER.length()));
	resp = poll(ESP_RESP_LEN);
	if (resp.back() != '>') {
		Logger::getInstance()->error("AT+HTTPCHEAD: " + resp);
		return 0; // init FAIL
	}

	send_cmd(ESP_API_HEADER, false);
	if (resp.find(ESP_OK) == std::string::npos) {
		Logger::getInstance()->error("Couldn't set header: " + resp);
		return 0; // init FAIL
	}

	HAL_Delay(100);

	send_cmd("AT+CWMODE=1"); // set WiFi mode
	resp = poll(ESP_RESP_LEN);
	if (resp.find(ESP_OK) == std::string::npos) {
		Logger::getInstance()->error("CWMODE: " + resp);
		return 0; // init FAIL
	}

	HAL_Delay(100);

	send_cmd("AT+CWJAP=\"MSetup\",\"\""); // connect to MSetup
	resp = poll(ESP_RESP_LEN, 5000);
	if (resp.find(ESP_WIFI_OK) == std::string::npos) {
		Logger::getInstance()->error("CWJAP: " + resp);
		return 0; // init FAIL
	}

	return 1;
}

std::string ESP32::poll(int num_bytes, uint32_t timeout) {
	char* buf = new char[num_bytes+1]();
	HAL_UART_Receive(&huart, (uint8_t*) buf, num_bytes, timeout);
	std::string ret(buf);
	delete[] buf;

	return ret;
}

void ESP32::process_incoming_bytes(char* buf, int num_bytes) {
	for (int i = 0; i < num_bytes; i++) {
		if (current_message.length() > 1 && buf[i] == '\n' && current_message.back() == '\r') {
			messages.push(current_message + buf[i]);
			osSemaphoreRelease(messages_sem);
			current_message = "";
		} else {
			current_message += buf[i];
		}
	}
}

std::string ESP32::consume_message() {
	if (!messages.empty()) {
		std::string msg = messages.front();
		messages.pop();
		return msg;
	} else {
		return "ERROR: someone tried to consume from an empty message queue";
	}
}
