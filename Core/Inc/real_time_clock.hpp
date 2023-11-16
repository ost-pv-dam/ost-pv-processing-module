#ifndef REAL_TIME_CLOCK_H
#define REAL_TIME_CLOCK_H

#include <string>
#include "logger.hpp"
#include <ctime>
#include <vector>

std::optional<time_t> get_timestamp_from_api(std::string time_resp);

#endif // REAL_TIME_CLOCK_H
