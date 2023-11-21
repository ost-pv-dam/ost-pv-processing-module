#include "data.hpp"

void DataPacket::serialize_json() {
	HeapStats_t stats;
	json += "{";
	json += get_json_key("timestamp") + std::to_string(timestamp) + ",";
	json += get_json_key("ambient_temp") + std::to_string(ambient_temp) + ",";
	json += get_json_key("humidity") + std::to_string(humidity) + ",";
	json += get_json_key("barometric_pressure") + std::to_string(barometric_pressure) + ",";
	json += get_json_key("iv_curves") += "{";

	for (auto it = iv_curves.begin(); it != iv_curves.end(); it++) {
		json += get_json_key(std::to_string(it->first)) += "["; // panel ID

		while (!it->second.empty()) {
			const CurrentVoltagePair& reading = it->second.front();
			json += "{" + get_json_key("v") + reading.voltage + "," + get_json_key("c") + reading.current + "}";

			if (it->second.size() > 1) { // no trailing comma
				json += ",";
			}

			it->second.pop();
		}

		// VERY IMPORTANT: will run out of heap otherwise
		it->second = std::queue<CurrentVoltagePair>();

		json += "]";
		if (std::next(it) != iv_curves.end()) { // don't include trailing comma
			json += ",";
		}
	}

	json += "},";
	json += get_json_key("cell_temperatures") += "{";

	for (auto it = cell_temperatures.begin(); it != cell_temperatures.end(); it++) {
		json += get_json_key(std::to_string(it->first)) + std::to_string(it->second); // panel_ID: temperature

		if (std::next(it) != cell_temperatures.end()) { // don't include trailing comma
			json += ",";
		}
	}

	json += "}"; // cell_temperatures
	json += "}"; // JSON object
}

void DataPacket::clear() {
	iv_curves = std::map<uint8_t, std::queue<CurrentVoltagePair>>();
	cell_temperatures = std::map<uint8_t, double>();
	json = JsonBuilder();
}
