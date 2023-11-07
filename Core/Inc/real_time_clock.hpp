#ifndef REAL_TIME_CLOCK_H
#define REAL_TIME_CLOCK_H

#include <string>
#include "logger.hpp"
#include <time.h>
#include <vector>

class RealTimeClock {
public:
	RealTimeClock(RTC_HandleTypeDef& hrtc) : hrtc(hrtc) {}
	bool parse_and_sync(std::string time_resp);
	time_t get_current_timestamp();

private:
	RTC_HandleTypeDef& hrtc;
};

time_t get_timestamp_from_api(std::string time_resp);

#endif // REAL_TIME_CLOCK_H
