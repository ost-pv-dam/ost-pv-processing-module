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

enum LogMode {
    UART,
    FILE,
    UART_AND_FILE
};

class Logger {
public:
    Logger(Logger &other) = delete;
    void operator=(const Logger &) = delete;
    ~Logger() {
        if (instance != nullptr) {
            delete instance;
        }
    }

	void debug(std::string msg);
	void info(std::string msg);
	void warn(std::string msg);
	void error(std::string msg);

	void direct(std::string msg);

	static Logger* getInstance() {
        if (instance == nullptr) {
            instance = new Logger();
        } else {
            return instance;
        }
	}

    UART_HandleTypeDef* huart;
    LogLevel logLevel = LogLevel::Info;
    std::string newline = "\n";

private:
    Logger() = default;

	static Logger* instance;
	void log(LogLevel level, std::string msg);
};

#endif // LOGGER_H
