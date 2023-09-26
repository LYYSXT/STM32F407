#include "stm32f4xx.h"                  // Device header
#include "Motor_Init.h"  
#include "Motor.h"

uint16_t CCR1=10500;  //F=2400          84000 000/(2*CCR)  = 2400
uint16_t CCR2=10500;  //F=2400          84000 000/(2*CCR)  = 2400


//GPIO�ڳ�ʼ��
void MOTOR_GPIO_Init_T(void) 
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);

	/* ����������������������PB4 */	//B4	TIM3_CH1	C7	TIM3_CH2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;						//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ���������������������� PG13*/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	 /* ���������������ʹ�ܿ���PG14 */
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void MOTOR_GPIO_Init_S(void) 
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);

	/* ����������������������PC7 */	//B4	TIM3_CH1	C7	TIM3_CH2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;						//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* ���������������������� PBG13*/	//D7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	 /* ���������������ʹ�ܿ���PG14 */	//C10
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//��ʱ����ʼ��
void MOTOR_TIM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_InternalClockConfig(TIM3);		//�ڲ�ʱ��	APB2����ʱ��(PCLK2=SYSCLK/2)   =84MHz     
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//ʱ�ӷ�Ƶ���ӣ������˲�����Ƶ�ʵĲ���
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;								//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 168-1;				//PSC		TIM_PRESCALER	168-1
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; 						//�߼���ʱ������
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
		
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;  						//��תģʽ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1;											//CCR �Ƚ�ֵ
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;  						//��תģʽ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2;											//CCR �Ƚ�ֵ
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Disable);    						//�ر�Ԥװ��
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Disable); 

	TIM_ClearFlag(TIM3,TIM_FLAG_CC1);											//�����ʱ��3��ͨ��1�ı�־λ
	TIM_ClearFlag(TIM3,TIM_FLAG_CC2);
	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);										//������ʱ��3��ͨ��1�ж�
	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);										//������ʱ��3��ͨ��1�ж�		ENABLE

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM3, ENABLE);
}
