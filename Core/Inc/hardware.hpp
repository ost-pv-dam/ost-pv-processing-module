//
// Created by Joseph Maffetone on 11/14/23.
//

#ifndef OST_PV_PROCESSING_MODULE_HARDWARE_HPP
#define OST_PV_PROCESSING_MODULE_HARDWARE_HPP

#include "main.h"
#include <vector>

class ADCSensor {
public:
    ADCSensor() = default;

    ADCSensor(ADC_HandleTypeDef *hadc, uint8_t channels)
            : m_hadc(hadc), m_channels(channels) {
        m_values.resize(channels);
    }

    uint32_t get_raw_channel_value(uint8_t channel) {
        return m_values.at(channel);
    }

    void update() {
        HAL_ADC_Start_DMA(m_hadc, m_values.data(), m_channels);
    }

private:
    ADC_HandleTypeDef* m_hadc;
    uint8_t m_channels;
    std::vector<uint32_t> m_values;
};

#endif //OST_PV_PROCESSING_MODULE_HARDWARE_HPP
