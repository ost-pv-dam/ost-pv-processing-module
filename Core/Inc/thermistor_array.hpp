//
// Created by Joseph Maffetone on 11/15/23.
//

#ifndef OST_PV_PROCESSING_MODULE_THERMISTOR_ARRAY_HPP
#define OST_PV_PROCESSING_MODULE_THERMISTOR_ARRAY_HPP


#include <vector>
#include <cstdint>

#include "stm32f4xx_hal.h"
#include "logger.hpp"

static constexpr uint32_t MAX_ADC_VALUE = 4096;

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
        // TODO: convert voltage to temperature
        return voltage;
    }

    void update() {
        HAL_ADC_Start_DMA(hadc, values.data(), values.size());
    }

private:
    ADC_HandleTypeDef* hadc;
    std::vector<std::uint32_t> values;
};


#endif //OST_PV_PROCESSING_MODULE_THERMISTOR_ARRAY_HPP
