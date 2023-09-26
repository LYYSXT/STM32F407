#ifndef __straight_motor_H
#define __straight_motor_H
#include "stm32f4xx.h"                  // Device header

#include "straight_motor.h" 
//#include "straight_tim.h"  
//#include <math.h>

#include "Motor_S.h"  
#include "sys.h"

/******************************************************************************************/
/* ֱ�߲岹���� */

#define AXIS_X              0       /* X���� */
#define AXIS_Y              1       /* Y���� */
#define LINE                0

#define EN                  0   /* ʧ���ѻ��ź� */
#define OFF                 1   /* ʹ���ѻ��ź� */

#define FIRST_QUADRANT      1   /* ��һ���� */
#define SECOND_QUADRANT     2   /* �ڶ����� */
#define THIRD_QUADRANT      3   /* �������� */
#define FOURTH_QUADRANT     4   /* �������� */

#define ARC                 1   /* Բ���岹 */

//enum dir
//{
//    CCW = 0,                        /* ��ʱ����ת */
//    CW ,                            /* ˳ʱ����ת */
//};  

typedef enum                        /* ���״̬ */
{
    STATE_STOP_straight = 0,
    STATE_RUN_straight = 1,
} st_motor_status_def;

typedef struct {
    uint16_t        pulse_pin;      /* ��ʱ������������� */
    uint32_t        pulse_channel;  /* ��ʱ���������ͨ�� */
    uint16_t        en_pin;         /* ���ʹ�����ű�� */
    uint16_t        dir_pin;        /* ����������ű�� */
    GPIO_TypeDef    *dir_port;      /* ����������Ŷ˿� */
    GPIO_TypeDef    *en_port;       /* ���ʹ�����Ŷ˿� */
} st_motor_ctr_def;

/*  �岹�㷨���Ͷ��� */
typedef struct {
    __IO uint8_t    moving_mode;    /* �˶�ģʽ */
    __IO uint8_t    inter_dir;      /* �岹���� */
    __IO uint8_t    qua_points;     /* ���޵� */
    __IO uint8_t    x_dir;          /* X�᷽�� */
    __IO uint8_t    y_dir;          /* Y�᷽�� */
    __IO int32_t    end_x;          /* �յ�����X */
    __IO int32_t    end_y;          /* �յ�����Y */
    __IO uint32_t   end_pulse;      /* �յ�λ���ܵ������� */
    __IO uint32_t   act_axis;       /* ��� */
    __IO int32_t    f_e;            /* �������� */
	__IO int32_t    start_point[2]; /* ������� */
} inter_pol_def;


				
//__IO st_motor_status_def  g_motor_sta_straight = STATE_STOP_straight;     /* ��������˶�״̬ */


/* �ⲿ�ӿں���*/
//void stepper_init(uint16_t arr, uint16_t psc);                          /* ��������ӿڳ�ʼ�� */
//void stepper_star(uint8_t motor_num);                                   /* ����������� */
//void stepper_stop(uint8_t motor_num);                                   /* �رղ������ */    
//void stepper_pwmt_speed(uint16_t speed,uint32_t Channel);               /* �����ٶ� */
void line_inpolation( int32_t coordsX,int32_t coordsY, int32_t Speed);  /* ʵ����������ֱ�߲岹 */           
int run_stop(void);


void stepper_pwmt_speed(uint16_t speed,uint32_t Channel);               /* �����ٶ� */
//void line_inpolation( int32_t coordsX,int32_t coordsY, int32_t Speed);  /* ʵ����������ֱ�߲岹 */    
void arc_incmove(int32_t start_x_point,int32_t start_y_point,int32_t stop_x_point,int32_t stop_y_point,uint32_t speed,int8_t dir);  /* ʵ����������Բ���岹 */  
 

#endif
