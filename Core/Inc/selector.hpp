#ifndef SELECTOR_H
#define SELECTOR_H

#include "stm32f4xx_hal.h"
#include <unordered_map>

constexpr uint32_t RELAY_SETTLE_WAIT = 100; // ticks

struct GPIOPortPin {
	GPIO_TypeDef* port;
	uint16_t pin;
};

class Selector {
public:
	Selector(const std::unordered_map<uint8_t, GPIOPortPin> panel_gpio) : panel_gpio(panel_gpio) {}

	bool select(uint8_t panel_id);
	void deselect_all();

private:
	std::unordered_map<uint8_t, GPIOPortPin> panel_gpio;
};

#endif // SELECTOR_H
