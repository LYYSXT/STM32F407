#include "bluetooth.h"

//接收
#define E_START                    0  	 //准备成功
#define E_OK                       1   	//成功
#define E_FRAME_HEADER_ERROR       2   	//包头错误
#define E_FRAME_RTAIL_ERROR        3  	 //包尾错误
#define LINE_LIN                   12  	//数据长度
#define HEADER 0xA5						//包头
#define TAIL 0x5A						//包尾

//发送
#define USART_TX_LEN 14					//数据包大小

uint8_t USART_TX_BUF[USART_TX_LEN];	//数据包缓存区

uint8_t uart_flag;                     //接收标志
vu8 RX_lanya[12];						//接收大小

int test_number=1;

uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

void Serial_Init(void)		//初始化
{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOD时钟
	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOD8复用为USART3_TX
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOC11复用为USART3_RX
	
	//GPIO端口设置		//不能发送数据可能就是这里的问题	没有配置好
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	//推挽复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		//TX	需要连接RX
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		//RX	需要连接TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//USART2 初始化设置
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;		//注意波特率要和蓝牙模块匹配	38400	JDY31-SPP	115200
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位
	USART_Init(USART2, &USART_InitStructure);
	
	//配置中断
	USART_ITConfig(USART2, USART_IT_RXNE , ENABLE);
	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
}

void Serial_SendByte(uint8_t Byte)					//发送字节
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)				//发送数组
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)				//发送字符串
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)				//发送数字
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

//int fputc(int ch, FILE *f)					//移植printf
//{
//	Serial_SendByte(ch);						//与usart文件里的重定义重复注释掉
//	return ch;
//}

void Serial_Printf(char *format, ...)			//移植printf
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;
}


//和手机相关联。

/*接收数据*/
void get_slave_data(uint8_t data)
{
    static uint8_t uart_num=0;
    RX_lanya[uart_num++]=data;		//?
    if(1==uart_num)
    {
        //接收到第一个字节不是0xA5,包头错误
        if(0XA5!=RX_lanya[0])
        {
            uart_num=0;
            uart_flag=E_FRAME_HEADER_ERROR;
        }
    }
    if(LINE_LIN==uart_num)
    {
        uart_flag=E_OK;
        //接收到最后一个字节是0X5A
        if(0X5A==RX_lanya[LINE_LIN-1])
        {
            uart_flag=E_OK;
        }
        else    //接收到的最后一个字节不为ox5A,包尾错误
        {
            uart_flag=E_FRAME_RTAIL_ERROR;
        }
        uart_num=0;
    }
 
}
 
int key1=0,key2=0,key3=0,key4=0,key5=0;
 
/*解析数据*/
/*数据包构成：包头（1字节）+字节变量（1字节）+4个段整形变量（共8个字节）+校验位（1字节）+包尾（1字节），共12个字节*/
void lanya_receive(void)
{
	key1=RX_lanya[1];
	
	key2=((int)RX_lanya[3]<<8)|RX_lanya[2];
	
	key3=((int)RX_lanya[5]<<8)|RX_lanya[4];
	
	key4=((int)RX_lanya[7]<<8)|RX_lanya[6];
	
	key5=((int)RX_lanya[9]<<8)|RX_lanya[8];
	
}

//发送数据
//计算校验位
uint8_t checksum(void)
{
	uint8_t checksum = 0;
	for (int i = 1; i <= (USART_TX_LEN-3); ++i) 
	{
		checksum += USART_TX_BUF[i];
	}
	checksum &= 0xff;
	return checksum;
}
//数据类型转变
void Int_to_Byte(int i,uint8_t *byte)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&i;          
	byte[3] = (longdata & 0xFF000000) >> 24;
	byte[2] = (longdata & 0x00FF0000) >> 16;
	byte[1] = (longdata & 0x0000FF00) >> 8;
	byte[0] = (longdata & 0x000000FF);

}
void Float_to_Byte(float f,uint8_t *byte)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;          
	byte[3] = (longdata & 0xFF000000) >> 24;
	byte[2] = (longdata & 0x00FF0000) >> 16;
	byte[1] = (longdata & 0x0000FF00) >> 8;
	byte[0] = (longdata & 0x000000FF);

}

void Short_to_Byte(short s,uint8_t *byte)
{  
	byte[1] = (s & 0xFF00) >> 8;
	byte[0] = (s & 0xFF);
}

//组合数据包
/*数据包构成：包头（1字节）+1个字节变量（1字节）+1个短整形变量（2个字节）+1个整形变量（4个字节）+1个浮点形变量（4个字节）+校验位（1字节）+包尾（1字节），共14个字节*/
void lanya_transmit(void) 
{
	char x = 0x10;							//-128~127
	short y = test_number;					//-32768~32767
	int z = 0x09;							//4字节
	float f = 20.5;							//4字节
	
	USART_TX_BUF[0] = HEADER;				//包头

	USART_TX_BUF[1] = (uint8_t)x;
	
	Short_to_Byte(y,&USART_TX_BUF[2]);
	
	Int_to_Byte(z,&USART_TX_BUF[4]);

	Float_to_Byte(f,&USART_TX_BUF[8]);

	//计算校验和
	USART_TX_BUF[12] = checksum();
		
	USART_TX_BUF[13] = TAIL;				//包尾

	//通过串口1发送
	Serial_SendArray(USART_TX_BUF,14);

}


void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
    {
	   USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	   Serial_RxData = USART_ReceiveData(USART2);		//返回 USART2 最近接收到的数据
	   Serial_RxFlag = 1;
	   get_slave_data(Serial_RxData);       			//获取数据
			if(uart_flag==1)
			{
				uart_flag=0;
				lanya_receive();   						//数据解析
			}
    }
}

