#ifndef __straight_tim_H
#define __straight_tim_H
#include "stm32f4xx.h"                  // Device header
#include "math.h"
//#include "Motor.h"  

//输出比较模式周期设置为0xFFFF
//#define TIM_PERIOD                   0xFFFF

void straight_GPIO_Init(void) ;
void straight_PWM_Init(uint16_t arr, uint16_t psc);
#endif

