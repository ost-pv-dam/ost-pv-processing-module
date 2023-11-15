//
// Created by Joseph Maffetone on 11/14/23.
//

#include <array>
#include <cstring>
#include "ost_pv_dam.hpp"
#include "uart_circular_buffer.hpp"

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

static constexpr size_t UART_CIRCULAR_BUFFER_LENGTH = 1024;

OstPvDam dam;

osSemaphoreId_t esp_messages_sem; // TODO: move the queue out of the esp class
osSemaphoreId_t esp_data_ready_sem;
osSemaphoreId_t smu_done_sem;

osMessageQueueId_t esp_msg_rx_queue;
osMessageQueueId_t smu_data_rx_queue;

UartCircularBuffer esp_rx_buf{};
UartCircularBuffer smu_rx_buf{};

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
    esp_msg_rx_queue = osMessageQueueNew(10U, sizeof(BufferRange), NULL);
    smu_data_rx_queue = osMessageQueueNew(100U, sizeof(BufferRange), NULL);

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

bool circular_buffer_contains(std::array<uint8_t, UART_CIRCULAR_BUFFER_LENGTH>& buf, size_t pos, std::string str) {
    if (pos >= str.length()) {

    }
}


void handle_esp_uart_interrupt() {
    static auto esp_buffer_range = BufferRange{0, 0};

    // short circuit to avoid unnecessary strncmp() calls
    if (esp_rx_buffer[esp_rx_pos] == '\n') {
        if (esp_rx_pos >= 3 && std::equal(esp_rx_buffer.begin() + esp_rx_pos - ESP_OK.size(),
                                          esp_rx_buffer.begin() + esp_rx_pos,
                                          ESP_OK.begin(), ESP_OK.end(),
                                          [](uint8_t a, char b) { return a == static_cast<uint8_t>(b); })) {
            esp_buffer_range.end = esp_rx_pos;
            osMessageQueuePut(esp_msg_rx_queue, &esp_buffer_range, 0U, 0U);

            if (esp_rx_pos == esp_rx_buffer.size()-1) {
                esp_buffer_range.start = 0;
            } else {
                esp_buffer_range.start = esp_rx_pos+1;
            }
        } else {
            // circular buffer look back
            size_t idx = esp_rx_pos;
            bool match = true;
            for (long long i = ESP_OK.size(); i >= 0; i--) {
                if (ESP_OK[i] != static_cast<char>(esp_rx_buffer[idx])) {
                    match = false;
                    break;
                }

                if (idx == 0) {
                    idx = esp_rx_buffer.size()-1;
                } else {
                    --idx;
                }
            }

            if (match) {
                esp_buffer_range.end = esp_rx_pos;
                osMessageQueuePut(esp_msg_rx_queue, &esp_buffer_range, 0U, 0U);
                esp_buffer_range.start = esp_rx_pos+1;
            }
        }
    } else if (esp_rx_buffer[esp_rx_pos] == '>') {
        osSemaphoreRelease(esp_data_ready_sem);
    }

    if (esp_rx_pos == esp_rx_buffer.size()-1) {
        esp_rx_pos = 0;
    } else {
        ++esp_rx_pos;
    }

    HAL_UART_Receive_IT(dam.esp.get_uart_handle(), esp_rx_buf.get_tail(), 1);
}

void handle_smu_uart_interrupt() {

}