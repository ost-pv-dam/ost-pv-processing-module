#include "clock.hpp"

bool Clock::parse_and_sync(std::string time_resp) {
	if (resp_pos == std::string::npos) {
		return false;
	}

	size_t next_delim = time_resp.find(",", resp_pos+12);
	int data_size = stoi(time_resp.substr(resp_pos+12, next_delim));

	size_t end_valid_data = next_delim + data_size;
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

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // look into what this does

	return true;
}
