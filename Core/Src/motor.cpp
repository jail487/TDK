/*
 * encoder.cpp
 *
 *  Created on: Jul 24, 2024
 *      Author: 88698
 */
#include "motor.h"
#include "string.h"
#include "math.h"
int16_t enc_a,enc_b,enc_c,enc_d;
const int resolution = 512;
//const int reduction_ratio = 64;
float speed[4];//encoder讀到的4顆馬達實際轉速，底盤只會用到前兩個，會透過UART回傳到任務機構STM裡
float Setpoint[4];//PID 的setpoint，即給馬達的目標轉速，由任務機構的循線模組算出速度再透過UART傳下來
float ki = 273.2793,ki_a = 10;
float kp = 7.77902,kp_a = 2;

int arr = 799;
float span = 0.001;
float v1 = -1,v2;
int timer = 0 ;
float p = 3.5, i= 10.1,turns;
int a = 1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim20;
extern UART_HandleTypeDef huart1;


void DCmotor_setup(){
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);//motor[1]
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);//motor[2]
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim20, TIM_CHANNEL_2);//motor[3]
	HAL_TIM_Encoder_Start(&htim20, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}
//typedef struct {
//    float kp;
//    float ki;
//    float integral;
//    float span;
//    int arr;
//    GPIO_TypeDef* gpioPort;
//    uint16_t gpioPin;
//    TIM_HandleTypeDef *htim;
//    uint32_t TIM_CHANNEL;
//} PID_controller;

// Define the PID_controller array
PID_controller motors[3] = {
    {kp_a, ki_a, 0, span, arr, GPIOB, GPIO_PIN_5, &htim1, TIM_CHANNEL_1},  // motor_a
    {kp, ki, 0, span, arr, GPIOB, GPIO_PIN_6, &htim1, TIM_CHANNEL_2},  // motor_b
    {2, 10, 0, span, arr, GPIOB, GPIO_PIN_7, &htim1, TIM_CHANNEL_3}    // motor_c
};



void PI_control_run(PID_controller* motor, float sp, float speed) {
    float error, u_a = 0;
    int pul = 0;
    float bound = 1 / motor->ki;

    error = sp - speed;
    motor->integral += error * motor->span;
    if (motor->integral > bound) motor->integral = bound;
    else if (motor->integral < -bound) motor->integral = -bound;

    u_a = motor->kp * error + motor->ki * motor->integral;
    if (u_a > 1) u_a = 1;
    else if (u_a < -1) u_a = -1;

    if (u_a > 0) {
        pul = (int)(u_a * motor->arr);
        HAL_GPIO_WritePin(motor->gpioPort, motor->gpioPin, GPIO_PIN_SET);
    } else if (u_a < 0) {
        pul = (int)(-u_a * motor->arr);
        HAL_GPIO_WritePin(motor->gpioPort, motor->gpioPin, GPIO_PIN_RESET);
    } else {
        pul = 0;
    }
    __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_CHANNEL, pul);
}

void getSpeed(TIM_HandleTypeDef *htim,float *speed,float reduction_ratio){
	int16_t enc ;
	enc = __HAL_TIM_GetCounter(htim);
	*speed = (float)enc /(4*resolution*reduction_ratio*span);
	__HAL_TIM_SetCounter(htim,0);
}

void DCmotor_run(){
	getSpeed(&htim8,&speed[0],20.8);//encoder1
	getSpeed(&htim2,&speed[1],20.8);//encoder2//left,- when+
	getSpeed(&htim20,&speed[2],20.8);//encoder4//right,+ when +
	PI_control_run(&motors[0], Setpoint[0], speed[0]);
	PI_control_run(&motors[1], Setpoint[1], speed[1]);
	PI_control_run(&motors[2], Setpoint[2], speed[2]);
	timer++;
	//turns -= speed_c*0.001;
//	if (a == 1){
//		if (abs(turns-6.5) <= 0.1){
//		//HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
//		v1 = 1;
//		a = 2;
//		//HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
//	}}
//	else if(a == 2){
//	 if(abs(turns-3)<=0.5){
//		 HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
//        a = 3;
//	}
//	}
		//HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
		//HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){
//	if (htim -> Instance == TIM6){
//
//
//	}
//}



