/*
 * motor.h
 *
 *  Created on: Aug 6, 2024
 *      Author: 88698
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_
#include "stm32g4xx_hal.h"
#include "mainpp.h"
typedef struct {
    float kp;
    float ki;
    float integral;
    float span;
    int arr;
    GPIO_TypeDef* gpioPort;
    uint16_t gpioPin;
    TIM_HandleTypeDef *htim;
    uint32_t TIM_CHANNEL;
} PID_controller;
typedef struct {
	PID_controller* motor;
	TIM_HandleTypeDef *htim;//encoder_timer
} DC_motor;
void DCmotor_setup();
void test();
void getSpeed(TIM_HandleTypeDef *htim,float *speed,float reduction_ratio);
void PI_control_run(PID_controller* motor, float sp, float speed);
void DCmotor_run();


#endif /* INC_ENCODER_H_ */
