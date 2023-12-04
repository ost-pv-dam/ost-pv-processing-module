//
// Created by Joseph Maffetone on 11/15/23.
//

#ifndef OST_PV_PROCESSING_MODULE_THERMISTOR_ARRAY_HPP
#define OST_PV_PROCESSING_MODULE_THERMISTOR_ARRAY_HPP


#include <vector>
#include <cstdint>
#include <cmath>

#include "stm32f4xx_hal.h"
#include "logger.hpp"

static constexpr uint32_t MAX_ADC_VALUE = 4096;
//static constexpr double COEFF_A = -14.634;
//static constexpr double COEFF_B = 4791.842;
//static constexpr double COEFF_C = -115334.0;
//static constexpr double COEFF_D = -3730535;

static constexpr double a0=1.14061e-3;
static constexpr double a1=2.32134e-4;
static constexpr double a3=9.63666e-8;

//static constexpr double RT = 10000;

class ThermistorArray {
public:
    ThermistorArray() = default;
    ThermistorArray(ADC_HandleTypeDef* hadc, std::size_t num_channels) : hadc{hadc} {
        values.resize(num_channels);
    }

    double get_temperature_at(std::size_t cell_number) {
        uint32_t raw_value = values[cell_number];
        Logger::getInstance()->debug("Raw ADC: " + std::to_string(raw_value));
        double voltage = (static_cast<double>(raw_value) / MAX_ADC_VALUE) * 3.3;


        double resistance = (voltage * 10000) / (3.3 - voltage);
//        double x = log(resistance / RT);
		float logR = log(resistance);
		float temp = 1.0 / (a0 + logR * (a1 + a3 * logR * logR)); // see https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
		temp -= 273.15;


//        double temp = 1 / (COEFF_A + COEFF_B * x +
//        		COEFF_C * pow(x, 2) +
//        		COEFF_D * pow(x, 3));

        return temp;
    }

    void update() {
        HAL_ADC_Start_DMA(hadc, values.data(), values.size());
    }

private:
    ADC_HandleTypeDef* hadc;
    std::vector<std::uint32_t> values;
};


#endif //OST_PV_PROCESSING_MODULE_THERMISTOR_ARRAY_HPP
