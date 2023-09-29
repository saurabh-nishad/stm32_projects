/*
 * logger.h
 *
 *  Created on: Sep 29, 2023
 *      Author: saura
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "main.h"
#include "stdio.h"

typedef enum log_type {
	LOG_INFO = 0,
	LOG_WARN,
	LOG_ERROR
}log_type_t;

void printf_log(char* data, log_type_t logName);
void printAssert(HAL_StatusTypeDef status, const char* message);

#endif /* INC_LOGGER_H_ */
