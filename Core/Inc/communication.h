/*
 * communication.h
 *
 *  Created on: Aug 8, 2024
 *      Author: macub
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef UART_Transmit_Two_Floats_DMA(UART_HandleTypeDef *huart, float value1, float value2);

#endif /* INC_COMMUNICATION_H_ */
