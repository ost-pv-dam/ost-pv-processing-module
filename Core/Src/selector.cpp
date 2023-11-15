#include "selector.hpp"

using namespace std;

bool Selector::select(uint8_t panel_id) {
	if (panel_id >= num_panels) {
        return false;
    }

	deselect_all();
	HAL_Delay(RELAY_SETTLE_WAIT);
	set_decoder(panel_id);
	return true;
}

void Selector::deselect_all() {
	set_decoder(deselect_output);
}

void Selector::set_decoder(uint8_t output) const {
	if (output & 0b1) {
		HAL_GPIO_WritePin(decoder_bit_0.port, decoder_bit_0.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(decoder_bit_0.port, decoder_bit_0.pin, GPIO_PIN_RESET);
	}

	if ((output & 0b10) >> 1) {
		HAL_GPIO_WritePin(decoder_bit_1.port, decoder_bit_1.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(decoder_bit_1.port, decoder_bit_1.pin, GPIO_PIN_RESET);
	}

	if ((output & 0b100) >> 2) {
		HAL_GPIO_WritePin(decoder_bit_2.port, decoder_bit_2.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(decoder_bit_2.port, decoder_bit_2.pin, GPIO_PIN_RESET);
	}
}
