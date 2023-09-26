#include "stm32f4xx.h"                  // Device header

#ifndef __Motor_S_H
#define __Motor_S_H

//#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/

#define T1_FREQ_S                 (168000000/(84+1))                         /* Ƶ��ftֵ */	//(168000000/168)
#define FSPR_S                    200                                     /* ���������Ȧ���� */
#define MICRO_STEP_S              8    
#define SPR_S                     (FSPR_S * MICRO_STEP_S)                     /* ��Ȧ����Ҫ�������� */

#define ROUNDPS_2_STEPPS(rpm)   ((rpm) * SPR_S / 60)                      /* ���ݵ��ת�٣�r/min�������������٣�step/s�� */
#define MIDDLEVELOCITY(vo,vt)   ( ( (vo) + (vt) ) / 2 )                 /* S�ͼӼ��ټ��ٶε��е��ٶ�  */
#define INCACCEL(vo,v,t)        ( ( 2 * ((v) - (vo)) ) / pow((t),2) )   /* �Ӽ��ٶ�:���ٶ�������   V - V0 = 1/2 * J * t^2 */
#define INCACCELSTEP(j,t)       ( ( (j) * pow( (t) , 3 ) ) / 6.0f )     /* �Ӽ��ٶε�λ����(����)  S = 1/6 * J * t^3 */
#define ACCEL_TIME(t)           ( (t) / 2 )                             /* �Ӽ��ٶκͼ����ٶε�ʱ������ȵ� */
#define SPEED_MIN               (T1_FREQ / (65535.0f))                  /* ���Ƶ��/�ٶ� */

#ifndef TRUE
#define TRUE                    1
#endif
#ifndef FALSE
#define FALSE                   0
#endif

typedef struct {
    int32_t vo;             /*  ���ٶ� ��λ step/s */
    int32_t vt;             /*  ĩ�ٶ� ��λ step/s */
    int32_t accel_step;     /*  ���ٶεĲ�����λ step */
    int32_t decel_step;     /*  ���ٶεĲ�����λ step */
    float   *accel_tab;     /*  �ٶȱ�� ��λ step/s �������������Ƶ�� */
    float   *decel_tab;     /*  �ٶȱ�� ��λ step/s �������������Ƶ�� */
    float   *ptr;           /*  �ٶ�ָ�� */
    int32_t dec_point;      /*  ���ٵ� */
    int32_t step;
    int32_t step_pos;
} speed_calc_t;

typedef enum
{
    STATE_ACCEL = 1,        /* �������״̬ */
    STATE_AVESPEED = 2,     /* �������״̬ */
    STATE_DECEL = 3,        /* �������״̬ */
    STATE_STOP = 0,         /* ���ֹͣ״̬ */
    STATE_IDLE = 4,         /* �������״̬ */
} motor_state_typedef;

enum DIR
{
 CCW = 0,                   /*��ʱ��*/ 
 CW                         /*˳ʱ��*/
};

enum EN
{
 EN_ON = 0,                 /* ʧ���ѻ����� */
 EN_OFF                     /* ʹ���ѻ����� ʹ�ܺ���ֹͣ��ת */
};

				
/* �ⲿ�ӿں���*/
//void stepper_init(uint16_t arr, uint16_t psc);              /* ��������ӿڳ�ʼ�� */
void stepper_star(uint8_t motor_num);                       /* ����������� */
void stepper_stop(uint8_t motor_num);                       /* �رղ������ */     
void stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step);  /* S�ͼӼ����˶����ƺ��� */
uint8_t calc_speed(int32_t vo, int32_t vt, float time);     /* �����ٶȱ� */                   
#endif
