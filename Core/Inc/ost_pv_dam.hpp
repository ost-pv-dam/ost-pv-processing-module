//
// Created by Joseph Maffetone on 11/14/23.
//

#ifndef OST_PV_PROCESSING_MODULE_OST_PV_DAM_HPP
#define OST_PV_PROCESSING_MODULE_OST_PV_DAM_HPP

#include <memory>
#include <utility>
#include "SHT30.hpp"
#include "selector.hpp"
#include "logger.hpp"
#include "ESP32.hpp"
#include "data.hpp"
#include "SMU.hpp"
#include "real_time_clock.hpp"
#include "MPL3115A2.hpp"

struct BufferRange {
    size_t start;
    size_t end;
};

class OstPvDam {
public:
    OstPvDam() = default;

    OstPvDam(SHT30 temp_humidity_sensor, MPL3115A2 pressure_sensor, Selector selector, ESP32 esp, SMU smu) :
            temp_humidity_sensor{temp_humidity_sensor},
            pressure_sensor{pressure_sensor},
            selector{selector},
            esp{std::move(esp)},
            smu{smu} {}

    void init();

    void update_and_upload_data() {
        std::shared_ptr<DataPacket> data = std::make_unique<DataPacket>(); // need this to be on the heap
    }


private:
    SHT30 temp_humidity_sensor;
    MPL3115A2 pressure_sensor;
    Selector selector;
    ESP32 esp;
    SMU smu;

};


#endif //OST_PV_PROCESSING_MODULE_OST_PV_DAM_HPP
