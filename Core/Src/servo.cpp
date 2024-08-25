/*
 * servo.cpp
 *
 *  Created on: 2024年8月19日
 *      Author: 88698
 */
#include "servo.h"
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim4;
int goalAngle,t;
int responseTime = 1000;
//F front, B back, R right, M middle, L left
servo servo_a{0,0,0,2000,true, &htim4, TIM_CHANNEL_1 };//FR
servo servo_b{0,0,0,2000,true, &htim4, TIM_CHANNEL_2 };//FM
servo servo_c{0,0,0,2000,true, &htim4, TIM_CHANNEL_3 };//FL
servo servo_d{0,0,0,2000,true, &htim4, TIM_CHANNEL_4 };//BR
servo servo_e{0,0,0,2000,true, &htim15, TIM_CHANNEL_1 };//BM
servo servo_f{0,0,0,2000,true, &htim15, TIM_CHANNEL_2 };//BL
//servo servo_a{0,0,0,2000,true, &htim4, TIM_CHANNEL_1 };

void servo_move(servo*servo,float goalAngle,int responseTime){
	servo -> goalAngle = goalAngle;
	servo -> responseTime = responseTime;
	servo -> move = true;
}
void servo_run(servo*servo ,int updateFreq){
	if (servo -> move == true){
		if ((int)servo -> pos == (int)servo -> goalAngle){
    	servo -> move = false;
    	servo -> lastAngle = servo -> goalAngle;
        }else{
         float distance = servo -> goalAngle - servo -> lastAngle;
         servo -> pos += distance/(servo -> responseTime * updateFreq / 1000);
         __HAL_TIM_SET_COMPARE(servo -> htim, servo -> TIM_CHANNEL,600+10*(int)servo -> pos);
         t++;
        }
	}
}

void blockState(bool state_a,bool state_b,bool state_c,bool state_d,bool state_e,bool state_f){
	//true open ,false close
	if (state_a == true){
		servo_move(&servo_a, 180,1000);
	}else{
		servo_move(&servo_a, 1,1000);
	}
	if (state_b == true){
		servo_move(&servo_b, 40,1000);
	}else{
		servo_move(&servo_b, 180,1000);
		}
	if (state_c == true){
		servo_move(&servo_c, 40,1000);
	}else{
		servo_move(&servo_c, 180,1000);
	}
	if (state_d == true){
		servo_move(&servo_d, 1,1000);
	}else{
		servo_move(&servo_d, 180,1000);
		}
	if (state_e == true){
		servo_move(&servo_e, 180,1000);
	}else{
		servo_move(&servo_e, 40,1000);
		}
	if (state_f == true){
		servo_move(&servo_f, 180,1000);
	}else{
		servo_move(&servo_f, 40,1000);
		}
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM6){
//	servo_move(&servo_a,goalAngle,responseTime);
//	servo_run(&servo_a ,1000);
//	//t++;
//	}
//}




