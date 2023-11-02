/*
 * solar_processing_structs.h
 *
 *  Created on: Oct 4, 2023
 *      Author: adarsh
 */

#ifndef DATA_H
#define DATA_H

#include <time.h>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <string>

/* measured cell voltage for the supplied current */
struct CurrentVoltagePair {
	std::string voltage;
	std::string current;
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
	std::string serialized_json;

private:
	std::ostringstream json;
	static std::string get_json_key(std::string field) {
		return "\"" + field + "\": ";
	}
};



/*
enum ErrorCode {
    NO_ERROR = 0,
    ERROR_INVALID_PARAMETER,
    ERROR_TIMEOUT,
	ERROR_HAL,
    // Add more error codes as needed

}
*/

#endif /* DATA_H */
