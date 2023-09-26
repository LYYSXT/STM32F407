#ifndef __Motor_Init_H
#define __Motor_Init_H
#include "stm32f4xx.h"                  // Device header
#include "math.h"
#include "Motor.h"  

//����Ƚ�ģʽ��������Ϊ0xFFFF
#define TIM_PERIOD                   0xFFFF

void MOTOR_GPIO_Init_T(void);
void MOTOR_GPIO_Init_S(void);
void MOTOR_TIM_Init(void);

#endif

