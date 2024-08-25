/*
 * PathSensor.h
 *
 *  Created on: Jul 29, 2024
 *      Author: macub
 */

#ifndef INC_PATHSENSOR_H_
#define INC_PATHSENSOR_H_

#include "stm32g4xx_hal.h"

void path_setup();
void path(float *motor_speed);

#endif /* INC_PATHSENSOR_H_ */
