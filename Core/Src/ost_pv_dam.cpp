//
// Created by Joseph Maffetone on 11/14/23.
//

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

OstPvDam dam;
osSemaphoreId_t esp_messages_sem;

void init() {
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
}

void update_and_upload_data() {

}