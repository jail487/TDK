/*
 * communication.cpp
 *
 *  Created on: Aug 8, 2024
 *      Author: macub
 */
#include "communication.h"
#include "stdlib.h"
#include "string.h"

uint8_t buffer_TX[2 * sizeof(float)];
//uint8_t buffer_RX[2 * sizeof(float)];

HAL_StatusTypeDef UART_Transmit_Two_Floats_DMA(UART_HandleTypeDef *huart, float value1, float value2) {

    memcpy(buffer_TX, &value1, sizeof(float));
    memcpy(buffer_TX + sizeof(float), &value2, sizeof(float));

    return HAL_UART_Transmit_DMA(huart, buffer_TX, sizeof(buffer_TX));
}
