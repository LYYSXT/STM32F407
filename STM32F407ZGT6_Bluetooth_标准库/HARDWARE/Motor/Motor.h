#ifndef __Motor_H
#define __Motor_H
#include "stm32f4xx.h"                  // Device header

//#include "stm32f10x.h"                  // Device header
#include "math.h"


typedef struct
{
	 uint8_t run_state;    //�����ת״̬
	 uint8_t dir ;         //�����ת����
	 int step_delay;   //�¸��������ڣ�ʱ����������ʱΪ���ٶ�
	 int decel_start;  //��������λ��
	 int decel_val;    //���ٽ׶β���
	 int min_delay;    //��С�������ڣ�����ٶȣ������ٶε��ٶȣ�
	 int accel_count;  //���ٽ׶μ���ֵ
}speedRampData;



#define TRUE     1
#define FALSE    0


/*����ٶȾ����е��ĸ�״̬*/
#define STOP              0 // ֹͣ״̬
#define ACCEL             1 // ����״̬
#define DECEL             2 // ����״̬
#define RUN               3 // ����״̬


#define TIM_PRESCALER      84		//Ԥ��Ƶ	32
#define T1_FREQ            (SystemCoreClock/(TIM_PRESCALER+1))     //��ʱ��Ƶ��

/*�����Ȧ����*/
#define STEP_ANGLE			1.8									//��������Ĳ���� ��λ����
#define FSPR            	200        //���������Ȧ����
#define MICRO_STEP        8         				//ϸ����ϸ���� 	8
#define SPR               (FSPR*MICRO_STEP)  //16ϸ�ֵĲ���

//��ѧ����������MSD_MOVE�����ļ򻯼���
#define ALPHA             ((float)(2*3.14159/SPR))       // ��= 2*pi/spr  �����  
#define A_T_x10           ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148       ((float)((T1_FREQ*0.676)/10)) // 0.676Ϊ�������ֵ(������̣��ĵ�����д)
#define A_SQ              ((float)(2*100000*ALPHA)) 
#define A_x200            ((float)(200*ALPHA))


void MOTOR_Move_T(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);				
extern void speed_decision(void);	

					
#endif
             
