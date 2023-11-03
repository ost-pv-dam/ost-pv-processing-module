#include "data.hpp"

void DataPacket::serialize_json() {
	json << "{";
	json << get_json_key("timestamp") << timestamp << ",";
	json << get_json_key("ambient_temp") << ambient_temp << ",";
	json << get_json_key("humidity") << humidity << ",";
	json << get_json_key("barometric_pressure") << barometric_pressure << ",";
	json << get_json_key("iv_curves") << "{";

	for (auto it = iv_curves.begin(); it != iv_curves.end(); it++) {
		json << get_json_key(std::to_string(it->first)) << "["; // panel ID
		for (size_t i = 0; i < it->second.size(); i++) {
			const CurrentVoltagePair& reading = it->second[i];
			json << "{";
			json << get_json_key("voltage") << reading.voltage << ",";
			json << get_json_key("current") << reading.current;
			json << "}";

			if (i < it->second.size() - 1) { // no trailing comma
				json << ",";
			}
		}

		json << "]";
		if (std::next(it) != iv_curves.end()) { // don't include trailing comma
			json << ",";
		}
	}

	json << "},";
	json << get_json_key("cell_temperatures") << "{";

	for (auto it = cell_temperatures.begin(); it != cell_temperatures.end(); it++) {
		json << get_json_key(std::to_string(it->first)) << it->second; // panel_ID: temperature

		if (std::next(it) != cell_temperatures.end()) { // don't include trailing comma
			json << ",";
		}
	}

	json << "}"; // cell_temperatures
	json << "}"; // JSON object

	serialized_json = json.str();

	json.str("");
	json.clear();
}

void DataPacket::clear() {
	iv_curves.clear();
	cell_temperatures.clear();
}
