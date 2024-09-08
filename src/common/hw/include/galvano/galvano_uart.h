/*
 * galvanometer_uart.h
 *
 *  Created on: 2024. 9. 8.
 *      Author: User
 */

#ifndef SRC_COMMON_HW_INCLUDE_GALVANO_GALVANO_UART_H_
#define SRC_COMMON_HW_INCLUDE_GALVANO_GALVANO_UART_H_


#include "hw_def.h"

#ifdef _USE_HW_GALVANO

#include "galvano.h"



bool galvanoUartDriver(galvano_driver_t* p_driver);

#endif


#endif /* SRC_COMMON_HW_INCLUDE_GALVANO_GALVANO_UART_H_ */
