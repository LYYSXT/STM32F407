#include "stm32f4xx.h"                  // Device header
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "bluetooth.h"

//����
extern int test_number;
int key_1=0;
uint8_t Direction=0;

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	delay_init(168);      //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					  //��ʼ��LED
 	LCD_Init();           //��ʼ��LCD FSMC�ӿ�
	
	Serial_Init();
	
	POINT_COLOR=BLACK;      //������ɫ����ɫ
	LCD_Clear(WHITE);
	LCD_ShowString(14*16,28*16,500,16,16,"TEST--LYYXXT");
	
  	while(1) 
	{		
		lanya_transmit();

		if(key_1==0)
		{
			test_number++;
			if(test_number==80)
			{
				key_1=1;
			}			
		}
		else if(key_1==1)
		{
			test_number--;
			if(test_number==10)
			{
				key_1=0;
			}	
		}
	
		if (Serial_GetRxFlag() == 1)		//�ж��Ƿ��յ�����
		{
			Direction = Serial_GetRxData();		//��ȡ����
			LCD_Clear(WHITE);
			LCD_ShowxNum(16,32,key2,2,16,1);
			LCD_ShowChar(16,16,Direction,16,1);

			printf("�ַ���%c		���֣�%d\r\n",Direction,key2);
			Serial_SendByte(Direction);		//�����ݻش�������
		}
	} 
}

