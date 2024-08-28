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
    float setpoint;
    int arr;
    GPIO_TypeDef *gpioPort;
    uint16_t gpioPin;
    TIM_HandleTypeDef *htim;
    uint32_t TIM_CHANNEL;
} PID_controller;
typedef struct {
	PID_controller *motor;
	TIM_HandleTypeDef *htim;//encoder_timer
	float speed;
	float currentHeight;
	float goalHeight;
	float reduction_ratio;
} DC_motor;
void DCmotor_setup();
void getState(DC_motor *lifter,int sign);
void PI_control_run(PID_controller *motor,DC_motor *lifter );
void DCmotor_run();
void set_goalHeight(PID_controller *motor, DC_motor *lifter,float height_setpoint);


#endif /* INC_ENCODER_H_ */
