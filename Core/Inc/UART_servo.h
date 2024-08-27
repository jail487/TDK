
#pragma once

#include "stm32g4xx_hal.h"

void UART_Send(uint8_t u8_data);

void UART_Send_SetMotorPosition(uint16_t motorId, uint16_t Position,uint16_t Time);
void inverseKinematics(double x, double y, double L1, double L2, double *theta1, double *theta2, bool elbowUp);
void arm_run();
void claw();
void arm_script();
