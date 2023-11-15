//
// Created by Joseph Maffetone on 11/14/23.
//

#include <array>
#include "ost_pv_dam.hpp"

extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

static constexpr size_t ESP_BUFFER_LENGTH = 512;
static constexpr size_t SMU_BUFFER_LENGTH = 512;

OstPvDam dam;

osSemaphoreId_t esp_messages_sem; // TODO: move the queue out of the esp class
osSemaphoreId_t esp_data_ready_sem;
osSemaphoreId_t smu_done_sem;

std::array<uint8_t, ESP_BUFFER_LENGTH> esp_rx_buffer;
std::array<uint8_t, SMU_BUFFER_LENGTH> smu_rx_buffer;

void OstPvDam::init() {
    if (!esp.init()) {
        Logger::getInstance()->error("ESP32 init FAIL!");
    } else {
        Logger::getInstance()->info("ESP32 init SUCCESS");
    }

    if (!temp_humidity_sensor.init()) {
        Logger::getInstance()->error("SHT30 init FAIL");
    } else {
        Logger::getInstance()->info("SHT30 init OK");
    }

    if (pressure_sensor.init() != HAL_OK) {
        Logger::getInstance()->error("Pressure sensor init FAIL");
    } else {
        Logger::getInstance()->info("Pressure sensor init OK");
    }

    selector.deselect_all();

    smu.config_voltage_sweep();
}

void init() {
    esp_data_ready_sem = osSemaphoreNew(1U, 0U, NULL);
    smu_done_sem = osSemaphoreNew(1U, 0U, NULL);

    Logger::getInstance()->huart = &huart4;
    Logger::getInstance()->logLevel = LogLevel::Debug;

    SHT30 temp_humidity_sensor = SHT30{&hi2c1};
    MPL3115A2 pressure_sensor = MPL3115A2{&hi2c2};

    auto selector_bit_0 = GPIOPortPin{GPIOD, GPIO_PIN_12};
    auto selector_bit_1 = GPIOPortPin{GPIOD, GPIO_PIN_13};
    auto selector_bit_2 = GPIOPortPin{GPIOD, GPIO_PIN_14};

    Selector selector = Selector{5U, 7U,
                                 selector_bit_0,
                                 selector_bit_1,
                                 selector_bit_2};

    esp_messages_sem = osSemaphoreNew(10U, 0U, NULL);
    ESP32 esp{&huart2, &esp_messages_sem};

    SMU smu{&huart6};

    dam = OstPvDam{temp_humidity_sensor, pressure_sensor, selector, esp, smu};
    dam.init();
}

void update_and_upload_data() {

}

void process_esp_message() {

}

void process_smu_data() {

}


void handle_esp_uart_interrupt() {

}

void handle_smu_uart_interrupt() {

}