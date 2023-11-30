#include "SMU.hpp"

void SMU::send_scpi(std::string scpi) {
	scpi += "\r";
	HAL_UART_Transmit(&huart, (uint8_t*) scpi.c_str(), scpi.length(), 100);
}

void SMU::config_voltage_sweep() {
	send_scpi(":SOUR:VOLT 0");
	send_scpi(":SOUR:DEL 0.1");
	send_scpi(":SOUR:SWE:RANG BEST");
	send_scpi(":SOUR:VOLT:MODE SWE");
	send_scpi(":SOUR:SWE:SPAC LIN");
	send_scpi(":SOUR:VOLT:STAR -1");
	send_scpi(":SOUR:VOLT:STOP 6");
	send_scpi(":SOUR:VOLT:STEP 0.05");
	send_scpi(":TRIG:COUNT 141");
	send_scpi(":FORM:ELEM CURR, VOLT");
	send_scpi(":SENS:CURR:DC:PROT:LEV 100E-3");
}

void SMU::run_voltage_sweep() {
	send_scpi(":OUTP ON");
	send_scpi(":READ?");
	send_scpi(":OUTP OFF");
}
