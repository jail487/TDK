/*
 * PathSensor.cpp
 *
 *  Created on: Jul 29, 2024
 *      Author: macub
 */
#include "PathSensor.h"
#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc1;

#define normal_Speed 1
#define w_kp 0.21
#define w_kd 0
#define boundry 1
#define spin_sp 0.8

uint16_t adcRead[7];
int   check = 0;
float weight_err;
float weight_lasttime = 0;
float weight_change = 0;
float tempSpeed[2];
/*
adcRead[0]  adc1-1   PA0  right
adcRead[1]  adc1-2   PA1    |
adcRead[2]  adc1-15  PB0    |
adcRead[3]  adc1-7   PC1    V
adcRead[4]  adc1-6   PC0  left
adcRead[5]  adc1-11  PB12  middle right
adcRead[6]  adc1-14  PB11  middle left
*/
void path_setup(){
	if(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcRead,7) != HAL_OK)
		check++;
}
//motor_speed[0]:right motor speed, motor_speed[1]:left motor speed
void path(float *motor_speed){

	//權重, P的err
	weight_err = (float)(-3*adcRead[0]-adcRead[1]+adcRead[3]+3*adcRead[4])/
					(adcRead[0]+adcRead[1]+adcRead[2]+adcRead[3]+adcRead[4]);

	//權重變化, D的err
	weight_change = weight_err - weight_lasttime;

	weight_lasttime = weight_err;

	//right
	tempSpeed[0] = normal_Speed + weight_err * w_kp + weight_change * w_kd;
	//left
	tempSpeed[1] = normal_Speed - weight_err * w_kp - weight_change * w_kd;

	//turn right
	if(adcRead[5] >= boundry && adcRead[6] < boundry && adcRead[0] < boundry && adcRead[1] < boundry
			&& adcRead[2] < boundry && adcRead[3] < boundry && adcRead[4] < boundry){

		motor_speed[0] = spin_sp * -1;
		motor_speed[1] = spin_sp;

		while(adcRead[5] >= boundry){}
		while(adcRead[2] < 3*boundry){}
/*
		motor_speed[0] = spin_sp;
		motor_speed[1] = spin_sp;

		while(adcRead[5] >= 3*boundry){}*/
	}
	//turn left
	else if(adcRead[5] < boundry && adcRead[6] >= 2 * boundry && adcRead[0] < boundry && adcRead[1] < boundry
			&& adcRead[2] < boundry && adcRead[3] < boundry && adcRead[4] < boundry){

		motor_speed[0] = spin_sp;
		motor_speed[1] = spin_sp * -1;

		while(adcRead[6] >= boundry){}
		while(adcRead[2] < 3*boundry){}
/*
		motor_speed[0] = spin_sp;
		motor_speed[1] = spin_sp;

		while(adcRead[6] >= 3*boundry){}*/
	}
	//stop
	else if(adcRead[5] >= boundry && adcRead[6] >= boundry){

		motor_speed[0] = 0;
		motor_speed[1] = 0;
	}
	//forward
	else{
		for(int j = 0; j < 2; j++){
			for(float i = 2; i >= 0; i -= 0.01){

				if(tempSpeed[j] >= i){
					motor_speed[j] = i;
					break;
				}
				else if(tempSpeed[j] > 2){
					motor_speed[j] = 2;
					break;
				}
				else if(tempSpeed[j] < 0){
					motor_speed[j] = 0;
					break;
				}
			}
		}
	}
}
