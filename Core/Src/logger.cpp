#include "Logger.hpp"

Logger* Logger::instance = nullptr;

void Logger::log(LogLevel level, std::string msg) {
	if (logLevel >= level) {
		msg += newline;
		HAL_UART_Transmit(&huart, (uint8_t*) msg.c_str(), msg.length(), 2000);
		// TODO: write to SD log file
	}
}

void Logger::debug(std::string msg) {
	log(LogLevel::Debug, "[DEBUG] " + msg);
}

void Logger::info(std::string msg) {
	log(LogLevel::Debug, "[INFO] " + msg);
}
void Logger::warn(std::string msg) {
	log(LogLevel::Debug, "[WARN] " + msg);
}

void Logger::error(std::string msg) {
	log(LogLevel::Debug, "[ERROR] " + msg);
}

void Logger::direct(std::string msg) {
	HAL_UART_Transmit(&huart, (uint8_t*) msg.c_str(), msg.length(), 2000);
}
