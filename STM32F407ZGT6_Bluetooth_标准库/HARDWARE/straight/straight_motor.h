#ifndef __straight_motor_H
#define __straight_motor_H
#include "stm32f4xx.h"                  // Device header

#include "straight_motor.h" 
//#include "straight_tim.h"  
//#include <math.h>

#include "Motor_S.h"  
#include "sys.h"

/******************************************************************************************/
/* 直线插补定义 */

#define AXIS_X              0       /* X轴标号 */
#define AXIS_Y              1       /* Y轴标号 */
#define LINE                0

#define EN                  0   /* 失能脱机信号 */
#define OFF                 1   /* 使能脱机信号 */

#define FIRST_QUADRANT      1   /* 第一象限 */
#define SECOND_QUADRANT     2   /* 第二象限 */
#define THIRD_QUADRANT      3   /* 第三象限 */
#define FOURTH_QUADRANT     4   /* 第四象限 */

#define ARC                 1   /* 圆弧插补 */

//enum dir
//{
//    CCW = 0,                        /* 逆时针旋转 */
//    CW ,                            /* 顺时针旋转 */
//};  

typedef enum                        /* 电机状态 */
{
    STATE_STOP_straight = 0,
    STATE_RUN_straight = 1,
} st_motor_status_def;

typedef struct {
    uint16_t        pulse_pin;      /* 定时器脉冲输出引脚 */
    uint32_t        pulse_channel;  /* 定时器脉冲输出通道 */
    uint16_t        en_pin;         /* 电机使能引脚编号 */
    uint16_t        dir_pin;        /* 电机方向引脚编号 */
    GPIO_TypeDef    *dir_port;      /* 电机方向引脚端口 */
    GPIO_TypeDef    *en_port;       /* 电机使能引脚端口 */
} st_motor_ctr_def;

/*  插补算法类型定义 */
typedef struct {
    __IO uint8_t    moving_mode;    /* 运动模式 */
    __IO uint8_t    inter_dir;      /* 插补方向 */
    __IO uint8_t    qua_points;     /* 象限点 */
    __IO uint8_t    x_dir;          /* X轴方向 */
    __IO uint8_t    y_dir;          /* Y轴方向 */
    __IO int32_t    end_x;          /* 终点坐标X */
    __IO int32_t    end_y;          /* 终点坐标Y */
    __IO uint32_t   end_pulse;      /* 终点位置总的脉冲数 */
    __IO uint32_t   act_axis;       /* 活动轴 */
    __IO int32_t    f_e;            /* 函数方程 */
	__IO int32_t    start_point[2]; /* 起点坐标 */
} inter_pol_def;


				
//__IO st_motor_status_def  g_motor_sta_straight = STATE_STOP_straight;     /* 步进电机运动状态 */


/* 外部接口函数*/
//void stepper_init(uint16_t arr, uint16_t psc);                          /* 步进电机接口初始化 */
//void stepper_star(uint8_t motor_num);                                   /* 开启步进电机 */
//void stepper_stop(uint8_t motor_num);                                   /* 关闭步进电机 */    
//void stepper_pwmt_speed(uint16_t speed,uint32_t Channel);               /* 设置速度 */
void line_inpolation( int32_t coordsX,int32_t coordsY, int32_t Speed);  /* 实现任意象限直线插补 */           
int run_stop(void);


void stepper_pwmt_speed(uint16_t speed,uint32_t Channel);               /* 设置速度 */
//void line_inpolation( int32_t coordsX,int32_t coordsY, int32_t Speed);  /* 实现任意象限直线插补 */    
void arc_incmove(int32_t start_x_point,int32_t start_y_point,int32_t stop_x_point,int32_t stop_y_point,uint32_t speed,int8_t dir);  /* 实现任意象限圆弧插补 */  
 

#endif
