#include "selector.hpp"

using namespace std;

bool Selector::select(uint8_t panel_id) {
	auto it = panel_outputs.find(panel_id);

	if (it == panel_outputs.end()) {
		return false; // panel ID not registered
	}

	deselect_all();
	HAL_Delay(RELAY_SETTLE_WAIT);
	set_decoder(it->second);
	return true;
}

void Selector::deselect_all() {
	set_decoder(deselect_output);
}

void Selector::set_decoder(uint8_t output) {
	if (output & 0b1) {
		HAL_GPIO_WritePin(decoder_bit_0.port, decoder_bit_0.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(decoder_bit_0.port, decoder_bit_0.pin, GPIO_PIN_RESET);
	}

	if (output & 0b10) {
		HAL_GPIO_WritePin(decoder_bit_1.port, decoder_bit_1.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(decoder_bit_1.port, decoder_bit_1.pin, GPIO_PIN_RESET);
	}

	if (output & 0b100) {
		HAL_GPIO_WritePin(decoder_bit_2.port, decoder_bit_2.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(decoder_bit_2.port, decoder_bit_2.pin, GPIO_PIN_RESET);
	}
}
