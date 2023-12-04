#ifndef DATA_H
#define DATA_H

#include <ctime>
#include <map>
#include <vector>
#include <sstream>
#include <list>
#include "cmsis_os.h"
#include "logger.hpp"
#include <queue>
#include <memory>
#include <cstring>

/* measured cell voltage for the supplied current */
struct CurrentVoltagePair {
	std::string current;
	std::string voltage;
};

class JsonBuilder {
public:
    JsonBuilder() = default;

    JsonBuilder& operator+=(const std::string& str) {
    	auto allocated = std::make_unique<char[]>(str.size()+1); // include null term.
    	m_size += str.size();
    	std::strcpy(allocated.get(), str.c_str());

        data.push_back(std::move(allocated));
        return *this;
    }

    std::list<std::unique_ptr<char[]>>& chunks() {
        return data;
    }

    size_t size() {
    	return m_size;
    }

private:
    std::list<std::unique_ptr<char[]>> data;
    size_t m_size{};
};


/* contains an entire data record with I-V curves for each cell and
   environmental data. Will be transmitted and logged all together */
struct DataPacket {
	time_t timestamp;

	double ambient_temp;
	double humidity;
	double barometric_pressure;
	std::map<uint8_t, std::queue<CurrentVoltagePair>> iv_curves;
	std::map<uint8_t, double> cell_temperatures;

	void serialize_json();
	JsonBuilder json;

	void clear();

private:
	static std::string get_json_key(std::string field) {
		return "\"" + field + "\": ";
	}
};

#endif /* DATA_H */
