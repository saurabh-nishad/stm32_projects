/*
 * logger.c
 *
 *  Created on: Sep 29, 2023
 *      Author: saura
 */

#include "logger.h"
#include "usart.h"

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}


/**
 * @brief: printf_log
 */
void printf_log(char* data, log_type_t logName)
{
	printf("\033[0m");
	switch(logName) {
	case LOG_INFO:
		printf("\033[1;32m");
		printf("Info\t:%s\n", data);
		break;
	case LOG_WARN:
		printf("\033[1;33m");
		printf("Warning\t:%s\n", data);
		break;
	case LOG_ERROR:
		printf("\033[1;31m");
		printf("Error\t:%s\n", data);
		break;
	}
}


void printAssert(HAL_StatusTypeDef status, const char* message) {
	char buffer[100] = {0};
	switch (status) {
		case HAL_BUSY:
			sprintf((char *)buffer, "%s: HAL_BUSY", message);
			printf_log(buffer, LOG_ERROR);
			break;
		case HAL_OK:
			sprintf((char *)buffer, "%s: HAL_OK", message);
			printf_log(buffer, LOG_INFO);
			break;

		case HAL_ERROR:
			sprintf((char *)buffer, "%s: HAL_ERROR", message);
			printf_log(buffer, LOG_ERROR);
			break;

		case HAL_TIMEOUT:
			sprintf((char *)buffer, "%s: HAL_TIMEOUT", message);
			printf_log(buffer, LOG_ERROR);
			break;
		default:
			break;
	}
}
