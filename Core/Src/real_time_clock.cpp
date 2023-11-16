#include <optional>
#include "real_time_clock.hpp"

bool RealTimeClock::parse_and_sync(std::string time_resp) {
	size_t resp_pos = time_resp.find("+HTTPCLIENT:");

	if (resp_pos == std::string::npos) {
		return false;
	}

	size_t next_delim = time_resp.find(",", resp_pos+12);
	int data_size = stoi(time_resp.substr(resp_pos+12, next_delim));
	size_t end_valid_data = next_delim + data_size;

	Logger::getInstance()->debug(time_resp.substr(next_delim+1, end_valid_data));
	size_t prev_delim = next_delim;
	std::vector<int> datetime;

	while (true) {
	  next_delim = time_resp.find(",", prev_delim+1);
	  if (next_delim == std::string::npos || next_delim > end_valid_data) {
		  break;
	  }
	  datetime.push_back(stoi(time_resp.substr(prev_delim+1, next_delim)));
	  prev_delim = next_delim;
	}

	// end of response is followed by a CRLF
	next_delim = time_resp.find("\r\n", prev_delim+1);
	datetime.push_back(stoi(time_resp.substr(prev_delim+1, next_delim)));

	if (datetime.size() != 6) {
	  Logger::getInstance()->warn("Unexpected number of components in datetime");
	}

	RTC_DateTypeDef sDate;
	sDate.Year = (uint8_t) (datetime[0] - 2000);
	sDate.Month = (uint8_t) datetime[1];
	sDate.Date = (uint8_t) datetime[2];

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
		Logger::getInstance()->error("Unable to store new RTC date");
		return false;
	}

	RTC_TimeTypeDef sTime;
	sTime.Hours = (uint8_t) datetime[3];
	sTime.Minutes = (uint8_t) datetime[4];
	sTime.Seconds = (uint8_t) datetime[5];

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Logger::getInstance()->error("Unable to store new RTC time");
		return false;
	}

//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // look into what this does
	return true;
}

std::optional<time_t> get_timestamp_from_api(std::string time_resp) {
	size_t resp_pos = time_resp.find("+SYSTIMESTAMP:");
	if (resp_pos == std::string::npos) {
		return std::nullopt;
	}

    size_t end_pos = time_resp.find('\r', resp_pos);
    if (end_pos == std::string::npos) {
        return std::nullopt;
    }

    try {
        long long timestamp = std::stoll(time_resp.substr(resp_pos+14, end_pos));
        auto result = static_cast<time_t>(timestamp);

        return std::make_optional<time_t>(result);
    } catch (const std::exception& e) {
        return std::nullopt;
    }
}

time_t RealTimeClock::get_current_timestamp() {
	RTC_DateTypeDef rtcDate;
	RTC_TimeTypeDef rtcTime;
	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BCD);

	struct tm tim = {0};
	tim.tm_year = (uint16_t) rtcDate.Year + 2000 - 1900;
	tim.tm_mon = rtcDate.Month - 1; // 0-indexed
	tim.tm_mday = rtcDate.Date;
	tim.tm_hour = rtcTime.Hours;
	tim.tm_min = rtcTime.Minutes;
	tim.tm_sec = rtcTime.Seconds;

	return mktime(&tim);
}
