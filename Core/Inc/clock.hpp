#include <string>
#include "logger.hpp"

class Clock {
public:
	Clock(RTC_HandleTypeDef& hrtc) : hrtc(hrtc) {}
	bool parse_and_sync(std::string time_resp);

private:
	RTC_HandleTypeDef& hrtc;
};
