#ifndef __bluetooth_H
#define __bluetooth_H
#include "sys.h"
#include <stdio.h>
#include <stdarg.h>


extern int key1,key2,key3,key4,key5;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);


//和手机相关联	GUI
void get_slave_data(uint8_t data);
void lanya_receive(void);
uint8_t checksum(void);
void Int_to_Byte(int i,uint8_t *byte);
void Float_to_Byte(float f,uint8_t *byte);
void Short_to_Byte(short s,uint8_t *byte);
void lanya_transmit(void); 

#endif
