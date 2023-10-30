/*
 * VC0706.h
 *
 *  Created on: Oct 23, 2023
 *      Author: adarsh
 */

#ifndef INC_VC0706_H_
#define INC_VC0706_H_

#include <stdint.h>
#include "stdio.h"
#include "stm32f4xx_hal.h"

#define VC0706_RESET 				0x26
#define VC0706_GEN_VERSION 			0x11
#define VC0706_SET_PORT 			0x24
#define VC0706_READ_FBUF 			0x32
#define VC0706_GET_FBUF_LEN 		0x34
#define VC0706_FBUF_CTRL 			0x36
#define VC0706_DOWNSIZE_CTRL 		0x54
#define VC0706_DOWNSIZE_STATUS 		0x55
#define VC0706_READ_DATA 			0x30
#define VC0706_WRITE_DATA 			0x31
#define VC0706_COMM_MOTION_CTRL 	0x37
#define VC0706_COMM_MOTION_STATUS 	0x38
#define VC0706_COMM_MOTION_DETECTED 0x39
#define VC0706_MOTION_CTRL 			0x42
#define VC0706_MOTION_STATUS 		0x43
#define VC0706_TVOUT_CTRL 			0x44
#define VC0706_OSD_ADD_CHAR 		0x45

#define VC0706_STOPCURRENTFRAME 	0x0
#define VC0706_STOPNEXTFRAME 		0x1
#define VC0706_RESUMEFRAME 			0x3
#define VC0706_STEPFRAME 			0x2

// standard image sizes for PTC-08 (Adafruit Product ID: 397)
#define VC0706_640x480 				0x00
#define VC0706_320x240 				0x11
#define VC0706_160x120 				0x22

// additional image sizes for compatible 2MP cameras like SC20MPB
#define VC0706_1024x768 			0x33
#define VC0706_1280x720 			0x44
#define VC0706_1280x960 			0x55
#define VC0706_1920x1080 			0x66

#define VC0706_MOTIONCONTROL 		0x0
#define VC0706_UARTMOTION 			0x01
#define VC0706_ACTIVATEMOTION 		0x01

#define VC0706_SET_ZOOM 			0x52
#define VC0706_GET_ZOOM 			0x53

#define CAMERABUFFSIZE 				100
#define CAMERADELAY 				10


class VC0706 {
public:
	VC0706(UART_HandleTypeDef *huart);

    bool begin();

    bool reset();

    bool take_picture();

    uint8_t* read_picture(uint8_t n);

    char* set_baud(uint32_t baud = 9600);

    uint8_t get_image_size();

    bool set_image_size(uint8_t size);

    uint32_t frameLength(void);

    uint8_t serial_num;
    uint8_t camera_buff[CAMERABUFFSIZE + 1];
    uint8_t buffer_len;
    uint32_t frame_ptr;
    UART_HandleTypeDef *huart;

private:

    bool run_command(uint8_t cmd, uint8_t *args, uint8_t argn, uint8_t resplen, bool flushflag = true);

    void send_command(uint8_t cmd, uint8_t args[], uint8_t argn);

    uint8_t read_response(uint8_t num_bytes, uint8_t timeout);

    bool verify_response(uint8_t cmd);
};


#endif /* INC_VC0706_H_ */
