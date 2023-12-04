#include <optional>
#include "real_time_clock.hpp"
#include "main.h"

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
