#ifndef __straight_tim_H
#define __straight_tim_H
#include "stm32f4xx.h"                  // Device header
#include "math.h"
//#include "Motor.h"  

//����Ƚ�ģʽ��������Ϊ0xFFFF
//#define TIM_PERIOD                   0xFFFF

void straight_GPIO_Init(void) ;
void straight_PWM_Init(uint16_t arr, uint16_t psc);
#endif

