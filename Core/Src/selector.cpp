#include "selector.hpp"

using namespace std;

bool Selector::select(uint8_t panel_id) {
	auto it = panel_gpio.find(panel_id);

	if (it == panel_gpio.end()) {
		return false; // panel ID not registered
	}

	deselect_all();
	HAL_Delay(RELAY_SETTLE_WAIT);
	HAL_GPIO_WritePin(it->second.port, it->second.pin, GPIO_PIN_SET);
	return true;
}

void Selector::deselect_all() {
	for (const auto& panel : panel_gpio) {
		const GPIOPortPin& gpio = panel.second;
		HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_RESET);
	}
}
