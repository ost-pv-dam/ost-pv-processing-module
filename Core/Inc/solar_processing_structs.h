/*
 * solar_processing_structs.h
 *
 *  Created on: Oct 4, 2023
 *      Author: adarsh
 */

#ifndef INC_SOLAR_PROCESSING_STRUCTS_H_
#define INC_SOLAR_PROCESSING_STRUCTS_H_

#include <time.h>
#include <unordered_map>
#include <vector>

/* measured cell voltage for the supplied current */
typedef struct {
	double voltage;
	double current;
} current_voltage_pair;


/* contains an entire data record with I-V curves for each cell and
   environmental data. Will be transmitted and logged all together */
typedef struct {

	time_t timestamp;

	double ambient_temp;
	double humidity;
	double barometric_pressure;
	std::unordered_map<char, std::vector<current_voltage_pair>> iv_curves;
	std::unordered_map<char, double> cell_temperatures;


} sensor_data_packet;

typedef enum {
    NO_ERROR = 0,
    ERROR_INVALID_PARAMETER,
    ERROR_TIMEOUT,
	ERROR_HAL,
    // Add more error codes as needed

} error_code;



#endif /* INC_SOLAR_PROCESSING_STRUCTS_H_ */
