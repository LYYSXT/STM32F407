#include "stm32f4xx.h"                  // Device header
#include "straight_tim.h"  
//#include "Motor.h"

//uint16_t CCR1=10500;  //F=2400          84000 000/(2*CCR)  = 2400
//uint16_t CCR2=10500;  //F=2400          84000 000/(2*CCR)  = 2400


//GPIO�ڳ�ʼ��
void straight_GPIO_Init(void) 
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);		//CH2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);		//CH1

	/* ����������������������PB4 */	//B3	TIM2_CH2	A15	TIM2_CH1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;						//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;						//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ���������������������� PBG13*/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	 /* ���������������ʹ�ܿ���PG14 */
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}



void straight_PWM_Init(uint16_t arr, uint16_t psc)
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_InternalClockConfig(TIM2);		//�ڲ�ʱ��	APB2����ʱ��(PCLK2=SYSCLK/2)   =84MHz     
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//ʱ�ӷ�Ƶ���ӣ������˲�����Ƶ�ʵĲ���
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = arr;									//ARR	
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;				//PSC		
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; 						//�߼���ʱ������
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;	
	TIM_OCStructInit(&TIM_OCInitStructure);	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  						//��תģʽ	TIM_OCMode_Toggle
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;											//CCR �Ƚ�ֵ		CCR1
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  						//��תģʽ	TIM_OCMode_Toggle
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/3;											//CCR �Ƚ�ֵ		CCR1
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	


//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM2, ENABLE);

}

//void TIM3_IRQHandler(void)
//{
//	//T
////	speed_decision();
//	
//	//	S
////	speed_decision_s();
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC1)== SET)
//	{	
//		
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);	// �����ʱ���ж�	
//	}

//}





