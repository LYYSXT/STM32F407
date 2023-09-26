//#include "stm32f10x.h"                  // Device header
#include "stm32f4xx.h"                  // Device header

//uint16_t CCR1=10500;  //F=2400          84000 000/(2*CCR)  = 2400

void PWM_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
	/* ����������������������PB4 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;									//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* ���������������������� PBG13*/

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	 /* ���������������ʹ�ܿ���PG14 */
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM3);		//�ڲ�ʱ��	APB2����ʱ��(PCLK2=SYSCLK/2)   =84MHz     
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//ʱ�ӷ�Ƶ���ӣ������˲�����Ƶ�ʵĲ���
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;									//ARR	65536
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;				//PSC		0
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; 						//�߼���ʱ������
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;	
	TIM_OCStructInit(&TIM_OCInitStructure);	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;  						//��תģʽ	TIM_OCMode_Toggle
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;											//CCR �Ƚ�ֵ		CCR1
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Disable);    						//�ر�Ԥװ��	TIM_OCPreload_Disable
	TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Enable);
	TIM_ARRPreloadConfig(TIM3,DISABLE);		//ENABLE

	TIM_ClearFlag(TIM3,TIM_FLAG_CC1);											//�����ʱ��3��ͨ��1�ı�־λ
	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);									//������ʱ��3��ͨ��1�ж�
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM3, ENABLE);

}


//void TIM3_IRQHandler(void)
//{
//	uint16_t num = 0;
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
//	{		 
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
//		num = TIM_GetCapture1(TIM3);
//		TIM_SetCompare1(TIM3,num+CCR1);
//	}

//}




