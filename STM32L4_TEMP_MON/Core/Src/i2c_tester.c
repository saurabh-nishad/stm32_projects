/*
 * i2c_tester.c
 *
 *  Created on: Sep 29, 2023
 *      Author: saurabh Nishad
 */


#include "i2c_tester.h"
#include "logger.h"


void check_connected_i2c_dev(I2C_HandleTypeDef *hi2c) {
	printf_log("Starting i2c Scanning\n", LOG_INFO);
	uint8_t buffer[25] = {0};
	for (uint8_t i = 0; i < 128; ++i) {
		if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i << 1), 3, 5) != HAL_OK) {
			printf_log("No Device Found", LOG_WARN);
		} else {
			sprintf((char *)buffer, "Device found: 0x%X", i << 1);
			printf_log("----------------------------", LOG_INFO);
			printf_log((char *)buffer, LOG_INFO);
			printf_log("----------------------------", LOG_INFO);
			break;
		}
	}
	printf_log("Scanning Complete...\n", LOG_INFO);
}
