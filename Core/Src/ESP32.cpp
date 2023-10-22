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
		return 0;
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
