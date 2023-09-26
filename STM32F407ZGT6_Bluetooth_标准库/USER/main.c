#include "stm32f4xx.h"                  // Device header
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "bluetooth.h"

//蓝牙
extern int test_number;
int key_1=0;
uint8_t Direction=0;

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	LED_Init();					  //初始化LED
 	LCD_Init();           //初始化LCD FSMC接口
	
	Serial_Init();
	
	POINT_COLOR=BLACK;      //画笔颜色：红色
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
	
		if (Serial_GetRxFlag() == 1)		//判断是否收到数据
		{
			Direction = Serial_GetRxData();		//读取数据
			LCD_Clear(WHITE);
			LCD_ShowxNum(16,32,key2,2,16,1);
			LCD_ShowChar(16,16,Direction,16,1);

			printf("字符：%c		数字：%d\r\n",Direction,key2);
			Serial_SendByte(Direction);		//将数据回传到电脑
		}
	} 
}

