#include "SMU.hpp"


void SMU::init_voltage_sweep() {
    char volt[] = ":SOUR:VOLT 0\r";
	char del[] = ":SOUR:DEL 0.1\r";
	char swe[] = ":SOUR:SWE:RANG BEST\r";
	char voltMode[] = ":SOUR:VOLT:MODE SWE\r";
	char spac[] = ":SOUR:SWE:SPAC LIN\r";
	char star[] = ":SOUR:VOLT:STAR -1\r";
	char stop[] = ":SOUR:VOLT:STOP 6\r";
	char step[] = ":SOUR:VOLT:STEP 1\r";
	char coun[] = ":TRIG:COUNT 8\r";

	HAL_UART_Transmit(&huart, (uint8_t*)volt, strlen(volt), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart, (uint8_t*)del, strlen(del), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart, (uint8_t*)swe, strlen(swe), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart, (uint8_t*)voltMode, strlen(voltMode), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart, (uint8_t*)spac, strlen(spac), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart, (uint8_t*)star, strlen(star), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart, (uint8_t*)stop, strlen(stop), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart, (uint8_t*)step, strlen(step), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart, (uint8_t*)coun, strlen(coun), HAL_MAX_DELAY);
}
void SMU::run_voltage_sweep() {
	char on[] = ":OUTP ON\r";
	char read[] = ":READ?\r";
	char off[] = ":OUTP OFF\r";

	HAL_UART_Transmit(&huart, (uint8_t*)on, strlen(on), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart, (uint8_t*)read, strlen(read), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart, (uint8_t*)off, strlen(off), HAL_MAX_DELAY);
}
