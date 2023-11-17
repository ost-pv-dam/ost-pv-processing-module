/*
 * VC0706.cpp
 *
 *  Created on: Oct 23, 2023
 *      Author: adarsh
 */


#include "VC0706.h"

UART_HandleTypeDef huart1;

VC0706::VC0706(UART_HandleTypeDef *huart) {
    this->huart = huart;
    frame_ptr = 0;
    buffer_len = 0;
    serial_num = 0;
}

bool VC0706::begin() {
	return reset();
}

bool VC0706::reset() {
	uint8_t args[] = {0x0};
	return run_command(VC0706_RESET, args, 1, 5);
}

bool VC0706::take_picture() {
	frame_ptr = 0;
	uint8_t args[] = {0x1, VC0706_STOPCURRENTFRAME};
	bool result = run_command(VC0706_FBUF_CTRL, args, sizeof(args), 5);
	return result;
}

uint8_t* VC0706::read_picture(uint8_t n) {
	uint8_t args[] = {0x0C,
	                  0x0,
	                  0x0A, 	//MCU mode
	                  (uint8_t)((frame_ptr >> 24) & 0xFF),
	                  (uint8_t)((frame_ptr >> 16) & 0xFF),
	                  (uint8_t)((frame_ptr >> 8) & 0xFF),
	                  (uint8_t)(frame_ptr & 0xFF),
	                  0,
	                  0,
	                  0,
	                  n,
	                  CAMERADELAY >> 8,
					  CAMERADELAY & 0xFF};

	bool result = run_command(VC0706_READ_FBUF, args, sizeof(args), 5, false);
	if (!result)
		return 0;

	// read into the buffer
	if (read_response(n + 5, CAMERADELAY) == 0)
		return 0;

	frame_ptr += n;

	return camera_buff;
}

uint8_t VC0706::get_image_size() {
	uint8_t args[] = {0x4, 0x4, 0x1, 0x00, 0x19};
	if (!run_command(VC0706_READ_DATA, args, sizeof(args), 6))
		return -1;

	return camera_buff[5];
}

bool VC0706::set_image_size(uint8_t size) {
	uint8_t args[] = {0x05, 0x04, 0x01, 0x00, 0x19, size};

	if (size < VC0706_1024x768)
    // standard image resolution
		args[1] = 0x04;
	else
	// extended image resolution
		args[1] = 0x05;

	return run_command(VC0706_WRITE_DATA, args, sizeof(args), 5);
}

char* VC0706::set_baud(uint32_t baud) {
	//default to 9600 baud
	uint8_t args[4] = {0x03, 0x01, 0xAE, 0xC8};

	if (baud == 19200) {
	    args[2] = 0x56;
	    args[3] = 0xE4;
	} else if (baud == 38400) {
	    args[2] = 0x2A;
	    args[3] = 0xF2;
	} else if (baud == 57600) {
	    args[2] = 0x1C;
	    args[3] = 0x1C;
	} else if (baud == 115200) {
	    args[2] = 0x0D;
	    args[3] = 0xA6;
	}

//	//write to camera
//	send_command(VC0706_SET_PORT, args, sizeof(args));
//
//	//get response back
//	uint8_t result = read_response(CAMERABUFFSIZE, 200);
//	if (!read_response(CAMERABUFFSIZE, 200)) return 0;
//
//	camera_buff[buffer_len] = 0; 	// end it!
//	return (char *)camera_buff; 	// return it!
	run_command(VC0706_SET_PORT, args, sizeof(args), 5);

	return (char *)camera_buff;
}

uint32_t VC0706::frameLength(void) {
  uint8_t args[] = {0x01, 0x00};
  if (!run_command(VC0706_GET_FBUF_LEN, args, sizeof(args), 9))
    return 0;

  uint32_t len;
  len = camera_buff[5];
  len <<= 8;
  len |= camera_buff[6];
  len <<= 8;
  len |= camera_buff[7];
  len <<= 8;
  len |= camera_buff[8];

  return len;
}

bool VC0706::run_command(uint8_t cmd, uint8_t *args, uint8_t argn, uint8_t resplen, bool flushflag){
	// flush out anything in the buffer?
	send_command(cmd, args, argn);

	uint8_t bytes_read = read_response(resplen, 200);
	if (bytes_read != resplen)
		return false;
	if (!verify_response(cmd))
		return false;

	return true;
}

void VC0706::send_command(uint8_t cmd, uint8_t args[], uint8_t argn) {
	// Send the data through USART using HAL functions.
	uint8_t toSend[2] = {0x56, 0x0};
	HAL_UART_Transmit(this->huart, toSend, 2, HAL_MAX_DELAY);
	HAL_UART_Transmit(this->huart, &cmd, 1, HAL_MAX_DELAY);
	for (uint8_t i = 0; i < argn; i++) {
		HAL_UART_Transmit(this->huart, (args + i), 1, HAL_MAX_DELAY);
	}
}

uint8_t VC0706::read_response(uint8_t num_bytes, uint8_t timeout) {
    HAL_StatusTypeDef result;
//    while (__HAL_UART_GET_FLAG(this->huart, UART_FLAG_RXNE)) {
//        uint8_t data;
//        HAL_UART_Receive(this->huart, &data, 1, 0); // Non-blocking read to discard data
//    }
    uint8_t data;
        HAL_UART_Receive(this->huart, &data, 1, 0);

   	result = HAL_UART_Receive(this->huart, camera_buff, num_bytes, HAL_MAX_DELAY);
   	if (result == HAL_OK) {
      	return num_bytes;
    }

    return -1;
}


bool VC0706::verify_response(uint8_t cmd){
	if ((camera_buff[0] != 0x76) || (camera_buff[1] != serial_num) ||
		(camera_buff[2] != cmd) || (camera_buff[3] != 0x0))
		return false;
	return true;
}