
#include "UART_servo.h"

#include <math.h>

const float PI = 3.14159265358979323846;
float x,l1 = 191 ,l2 = 174;
float pos[6];
bool elbowUp = true;
int claw_open = 1;
int arm_setup = 0;
int k ;
int script =0;
extern UART_HandleTypeDef huart3;
int Checksum_Calc, count = 0;
void UART_Send(uint8_t u8_data) {
	uint8_t *u8_pointer = &u8_data;
	HAL_UART_Transmit(&huart3, u8_pointer, 1, 100);
	Checksum_Calc += u8_data;
}




// 函数来计算逆运动学
//void inverseKinematics(float x, float y, float L1, float L2, float *pos1, float *pos2) {
//    //float distance = sqrt(x * x + y * y);
//    // 计算 theta2
//    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
//    float theta2 = acos(cosTheta2);
//    // 计算 theta1
//    float sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2);  // sin(theta2)
//    float theta1 = atan2(y, x) - atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);
//    // 将角度从弧度转换为度数
//    *pos1 = theta1 * 180.0 / PI;
//    *pos2 = theta2 * 180.0 / PI;
//}
//void inverseKinematics(float x, float y, float L1, float L2, float *pos1, float *pos2, bool elbowUp) {
//    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
//    // Ensure the value passed to acos is within valid range [-1, 1]
//    cosTheta2 = fmin(fmax(cosTheta2, -1.0), 1.0);
//
//    float theta2 = acos(cosTheta2);
//    if (!elbowUp) {
//        theta2 = -(theta2);  // Flip the solution for elbow down
//    }
//
//    float sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2);
//    float theta1 = atan2(y, x) - atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);
//
//    // Convert angles from radians to degrees
//    *pos1 = theta1 * 180.0 / PI;
//    *pos2 = theta2 * 180.0 / PI;
//}

void UART_Send_SetMotorPosition(uint16_t motorId, uint16_t Position, uint16_t Time) {
	Checksum_Calc = 0;
	UART_Send(0x80 + motorId);    //header mark & broadcast ID
	UART_Send(0x83);              //header mark & command code
	UART_Send(0x05);              //total data length
	UART_Send((Position / 256) & 0x7F);  //Servo Pos_H
	UART_Send(Position % 256);           //Servo Pos_L
	UART_Send((Time / 256) & 0x7F); //Servo Time_H
	UART_Send(Time % 256);          //Servo Time_L
	UART_Send(Checksum_Calc);     //data length (one servo with time and speed)
}
void claw(){
	if (claw_open == 0){
	UART_Send_SetMotorPosition(4,1300,2000);
	}else{
	UART_Send_SetMotorPosition(4,2200,2000);
	}
}
//void arm_run(){
//	if(arm_setup == 1){
//	inverseKinematics(x,y,l1,l2,&pos1,&pos2,elbowUp);
//	UART_Send_SetMotorPosition(1,(uint16_t)(1080+7*pos1),2000);
//	UART_Send_SetMotorPosition(3,(uint16_t)(980+7*pos2),2000);
//	claw();
//	}
//}
void arm_run(){
	//inverseKinematics(x,y,l1,l2,&pos1,&pos2,elbowUp);
	if(arm_setup == 1){
	UART_Send_SetMotorPosition(1,(uint16_t)(1270+7*pos[0]),2000);
	UART_Send_SetMotorPosition(3,(uint16_t)(1000+7*pos[1]),2000);

//    arm_setup = 0;

	k++;
	}claw();
}
void arm_script(){
	if (script  == 1){
//		pos1 = -35;
//		pos2 = -60;
	}else if (script == 2){
//		pos1 = 40;
//		pos2 = 80;
		//HAL_Delay(2000);
		//claw_open = 1;
	}else if (script == 3){
//		pos1 =60;
	}
}



