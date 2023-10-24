#ifndef LOGGER_H
#define LOGGER_H

#include "stm32f4xx_hal.h"
#include <string>

enum LogLevel {
	Debug,
	Info,
	Warn,
	Error
};

class Logger {
public:
	Logger(UART_HandleTypeDef& huart, LogLevel level = Info, std::string newline = "\n") :
		huart(huart), logLevel(level), newline(newline) {}

	void debug(std::string msg);
	void info(std::string msg);
	void warn(std::string msg);
	void error(std::string msg);

	static void registerInstance(Logger* inst) {
		instance = inst;
	}

	static Logger* getInstance() {
		return instance;
	}

private:
	UART_HandleTypeDef& huart;
	LogLevel logLevel;
	std::string newline;

	static Logger* instance;

	void log(LogLevel level, std::string msg);
};

#endif // LOGGER_H
