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
	Selector(const std::unordered_map<uint8_t, uint8_t> panel_outputs,
			GPIOPortPin decoder_bit_0,
			GPIOPortPin decoder_bit_1,
			GPIOPortPin decoder_bit_2, uint8_t deselect_number) :
				panel_outputs(panel_outputs),
				decoder_bit_0(decoder_bit_0),
				decoder_bit_1(decoder_bit_1),
				decoder_bit_2(decoder_bit_2), deselect_output(deselect_number) {}

	bool select(uint8_t panel_id);
	void deselect_all();

private:
	std::unordered_map<uint8_t, uint8_t> panel_outputs;
	GPIOPortPin decoder_bit_0;
	GPIOPortPin decoder_bit_1;
	GPIOPortPin decoder_bit_2;
	uint8_t deselect_output;

	void set_decoder(uint8_t output);
};

#endif // SELECTOR_H
