/*
 * mainpp.cpp
 *
 *  Created on: Jul 29, 2024
 *      Author: macub
 */
#include "mainpp.h"
#include "PathSensor.h"
#include "communication.h"
#include "stdlib.h"
#include "string.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

float sp[2];
int tx = 0, rx = 0, timer;
float speed1, speed2;
uint8_t buffer_RX[2 * sizeof(float)];

void main_function(){

	HAL_UART_Receive_DMA(&huart1, buffer_RX, sizeof(buffer_RX));
	HAL_TIM_Base_Start_IT(&htim6);

	path_setup();

	while(1){

		path(sp);

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	if(huart -> Instance == USART1)
        tx++;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart){

	if(huart -> Instance == USART1){

		memcpy(&speed1, buffer_RX, sizeof(float));
	    memcpy(&speed2, buffer_RX + sizeof(float), sizeof(float));

	    HAL_UART_Receive_DMA(&huart1, buffer_RX, sizeof(buffer_RX));

		rx++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM6){

		timer++;
		UART_Transmit_Two_Floats_DMA(&huart1,sp[0],sp[1]);

	}
}
