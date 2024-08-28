/*
 * encoder.cpp
 *
 *  Created on: Jul 24, 2024
 *      Author: 88698
 */
#include "motor.h"
#include "string.h"
#include "math.h"
#define epsilon 0.01
const int resolution = 512;
float ki = 273.2793,ki_a = 10;
float kp = 7.77902,kp_a = 2;
int arr = 799;
float span = 0.001;
int motor_run = 0;
float goalHeight[4] = {0};
int l = 0;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim20;

PID_controller motors[4] = {
    {kp_a, ki_a, 0, span, 0, arr,
    		GPIOB, GPIO_PIN_5, &htim1, TIM_CHANNEL_1},  // lifter_motor_a
    {kp, ki, 0, span, 0, arr,
    		GPIOB, GPIO_PIN_6, &htim1, TIM_CHANNEL_2},  // lifter_motor_b
    {7.77902, 273.2793, 0, span, 0, arr,
    		GPIOB, GPIO_PIN_7, &htim1, TIM_CHANNEL_3},    // cascade_motor_a
    {7.77902, 273.2793, 0, span, 0, arr,
    		GPIOB, GPIO_PIN_4, &htim1, TIM_CHANNEL_4}//cascade_motor_b
};
DC_motor DCmotorlifer[4] = {
	{&motors[0],&htim8,0,0,0,20.8},
	{&motors[1],&htim2,0,0,0,20.8},
	{&motors[2],&htim20,0,0,0,64},
	{&motors[3],&htim4,0,0,0,64}
};

void DCmotor_setup(){
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);//motor[1]
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);//motor[2]
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim20, TIM_CHANNEL_2);//motor[3]
	HAL_TIM_Encoder_Start(&htim20, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);//motor[3]
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}

void PI_control_run(PID_controller *motor,DC_motor *lifter ) {
    float error, u_a = 0;
    int pul = 0;
    float bound = 1 / motor->ki;

    error = motor->setpoint - lifter->speed;
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

void getState(DC_motor *lifter,int sign){
	int16_t enc ;
	enc = __HAL_TIM_GetCounter(lifter->htim);
	lifter->speed = (float)enc /(4*resolution*lifter->reduction_ratio*span);
	__HAL_TIM_SetCounter(lifter->htim,0);
	lifter->currentHeight += lifter->speed*span*sign;
}
void set_goalHeight(PID_controller *motor, DC_motor *lifter,float height_setpoint){
	lifter->goalHeight = height_setpoint;
	if (abs(lifter->currentHeight - lifter->goalHeight) >= epsilon){
		if(lifter->goalHeight >= lifter->currentHeight){
		    motor->setpoint = -0.5;
		}else if(lifter->goalHeight <= lifter->currentHeight){
			motor->setpoint = 0.5;
		}
	}else{
		motor->setpoint = 0;l++;
	}
}
void DCmotor_run(){
	getState(&DCmotorlifer[0],-1);//encoder1
	getState(&DCmotorlifer[1],1);//encoder2//left,- when+
	getState(&DCmotorlifer[2],1);//encoder4//right,+ when +
	getState(&DCmotorlifer[3],1);
	PI_control_run(&motors[0], &DCmotorlifer[0]);
	PI_control_run(&motors[1], &DCmotorlifer[1]);
	PI_control_run(&motors[2], &DCmotorlifer[2]);
	PI_control_run(&motors[2], &DCmotorlifer[3]);
	set_goalHeight(&motors[0], &DCmotorlifer[0],goalHeight[0]);
	set_goalHeight(&motors[1], &DCmotorlifer[1],goalHeight[1]);
	set_goalHeight(&motors[2], &DCmotorlifer[2],goalHeight[2]);
	set_goalHeight(&motors[3], &DCmotorlifer[3],goalHeight[3]);
	motor_run++;
	}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){
//	if (htim -> Instance == TIM6){
//
//
//	}
//}



