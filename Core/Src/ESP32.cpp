#include "ESP32.hpp"

void ESP32::send_cmd(const std::string& cmd, bool crlf) {
	if (crlf) {
		std::string cmd_crlf = cmd + "\r\n";
		HAL_UART_Transmit(&huart, (uint8_t*) cmd_crlf.c_str(), cmd_crlf.length(), 100);
	} else {
		if (cmd.length() > LONG_CMD_THRESHOLD) {
			HAL_UART_Transmit(&huart, (uint8_t*) cmd.c_str(), cmd.length(), HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart, (uint8_t*) cmd.c_str(), cmd.length(), 100);
		}
	}
}

void ESP32::send_raw(std::unique_ptr<char[]>&& cmd) {
	HAL_UART_Transmit(&huart, (uint8_t*) cmd.get(), std::strlen(cmd.get()), 100);
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
	send_cmd("AT+UART_CUR=115200,8,1,0,0");
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


void ESP32::send_data_packet_start(size_t json_length) {
	std::ostringstream postCmd;

	postCmd << "AT+HTTPCPOST=\"https://api.umich-ost-pv-dam.org:5050/api/v1/sensorCellData\",";
	postCmd << json_length;
	postCmd << ",2,\"connection: keep-alive\",\"content-type: application/json\"";

	send_cmd(postCmd.str());
}

void ESP32::push_message(std::string msg) {
	messages.push(msg);
	osSemaphoreRelease(external_queue);
}

std::string ESP32::consume_message() {
	if (!messages.empty()) {
		std::string msg = messages.front();
		messages.pop();
		return msg;
	} else {
		return "ERROR";
	}
}

void ESP32::flush() {
	osMessageQueueReset(external_queue);
	std::queue<std::string>().swap(messages);
}
