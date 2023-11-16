#ifndef DATA_H
#define DATA_H

#include <ctime>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <deque>
#include "cmsis_os.h"
#include "logger.hpp"

/* measured cell voltage for the supplied current */
struct CurrentVoltagePair {
	std::string current;
	std::string voltage;
};

class JsonBuilder {
public:
    JsonBuilder() = default;

    JsonBuilder& operator+=(const std::string& str) {
        data.push_back(str);
        return *this;
    }

    std::deque<std::string>& chunks() {
        return data;
    }

private:
    std::deque<std::string> data;
};


/* contains an entire data record with I-V curves for each cell and
   environmental data. Will be transmitted and logged all together */
struct DataPacket {
	time_t timestamp;

	double ambient_temp;
	double humidity;
	double barometric_pressure;
	std::unordered_map<uint8_t, std::vector<CurrentVoltagePair>> iv_curves;
	std::unordered_map<uint8_t, double> cell_temperatures;

	void serialize_json();
	JsonBuilder json;

	void clear();

private:
	static std::string get_json_key(std::string field) {
		return "\"" + field + "\": ";
	}
};

#endif /* DATA_H */
