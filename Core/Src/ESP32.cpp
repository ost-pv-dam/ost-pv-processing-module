#include "ESP32.hpp"

void ESP32::send_cmd(std::string cmd) {
	cmd += "\r\n";
	HAL_UART_Transmit(&huart, (uint8_t*) cmd.c_str(), cmd.length(), 100);
}

int ESP32::init() {
	// AT+RESTORE restarts the module, so the timing is hard to get right
	// will not include for now

	send_cmd("ATE0"); // disable echo

	std::string resp = poll(ESP_ECHO_OFF_OK.length());
	if (resp != ESP_ECHO_OFF_OK && resp != ESP_OK) {
		return 0; // init FAIL
	}

	return 1;
}

std::string ESP32::poll(int num_bytes) {
	char* buf = new char[num_bytes+1]();
	HAL_UART_Receive(&huart, (uint8_t*) buf, num_bytes, 100);
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
		return "ERROR -- This should never be seen";
	}
}
