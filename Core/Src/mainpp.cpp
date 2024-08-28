/*
 * mainpp.cpp
 *
 *  Created on: Jul 29, 2024
 *      Author: macub
 */
#include "mainpp.h"
#include "PathSensor.h"
#include "communication.h"
#include "servo.h"
#include "motor.h"
#include "UART_servo.h"
#include "stdlib.h"
#include "string.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim6;
float sp[2];
int tx = 0, rx = 0;
float speed1, speed2;
uint8_t buffer_RX[2 * sizeof(float)];
bool state [5];
void main_function(){

	//HAL_UART_Receive_DMA(&huart5, buffer_RX, sizeof(buffer_RX));

	//path_setup();
	//servo_setup();
	//motor_setup();
	//while(1){
	//path(sp);
	//}
}
void setup(){
	DCmotor_setup();
	servo_setup();
	HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	if(huart -> Instance == UART5)
        tx++;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart){

	if(huart -> Instance == UART5){

		memcpy(&speed1, buffer_RX, sizeof(float));
	    memcpy(&speed2, buffer_RX + sizeof(float), sizeof(float));
	    HAL_UART_Receive_DMA(&huart5, buffer_RX, sizeof(buffer_RX));

		rx++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM6){

		//timer++;
		//UART_Transmit_Two_Floats_DMA(&huart5,sp[0],sp[1]);
		blockState(state[0],state[1],state[2],state[3],state[4],state[5]);/////git add
		DCmotor_run();//////////
		//arm_run();

	}
}
