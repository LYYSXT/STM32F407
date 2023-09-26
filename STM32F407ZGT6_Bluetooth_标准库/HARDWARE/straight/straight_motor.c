#include "stm32f4xx.h"                  // Device header
#include "straight_motor.h" 
//#include "straight_tim.h"  
#include <math.h>

#include "Motor_S.h"  
#include "sys.h"
#include "stdlib.h"
/*****************************************************直线插补实验*****************************************************/
inter_pol_def g_pol_par= {0};   /* 直线插补参数值 */

//在头文件定义
__IO st_motor_status_def  g_motor_sta_straight = STATE_STOP_straight;     /* 步进电机运动状态 */

//__IO st_motor_status_def  g_motor_sta = STATE_STOP;     /* 步进电机运动状态 */
__IO int32_t  sign_dir[2] = {1,1};                      /* 偏差方程的计算公式符号位 */

int run_stop(void)
{
	while(1)
	{
		if(g_motor_sta_straight==STATE_STOP_straight)
		{
			break;
		}
		return 1;
	}
	return 0;
}

/**
 * @brief       直线增量插补函数实现直线插补功能,两个步进电机分别向X轴和Y轴步进IncX,IncY步
 * @param       IncX    ：终点X轴坐标
 * @param       IncY    ：终点Y轴坐标
 * @param       Speed   ：进给速度
 * @retval      无
 */
void line_incmove(uint32_t IncX,uint32_t IncY,uint32_t Speed)
{
    /* 偏差方程置零 */
    g_pol_par.f_e = 0;

    /* 计算起点到终点坐标对应的脉冲数位置*/
    g_pol_par.end_x = IncX;
    g_pol_par.end_y = IncY;
    g_pol_par.end_pulse = g_pol_par.end_y + g_pol_par.end_x;

    /* 根据终点判断在直线上的进给方向,减少偏差 */
    if(g_pol_par.end_y > g_pol_par.end_x)
    {
        g_pol_par.act_axis = AXIS_Y;                    /* 第一步进给Y轴 */
        g_pol_par.f_e = g_pol_par.f_e + g_pol_par.end_x;
    }
    else
    {
        g_pol_par.act_axis = AXIS_X;                    /* 第一步进给X轴 */
        g_pol_par.f_e = g_pol_par.f_e - g_pol_par.end_y;
    }
    /* 设置通道的比较值 */
	TIM_SetCompare1(TIM2, Speed);
	TIM_SetCompare2(TIM2, Speed);
	TIM_SetAutoreload(TIM2, Speed*2);
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_X].pulse_channel,Speed);   
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_Y].pulse_channel,Speed);
//    __HAL_TIM_SET_AUTORELOAD(&g_atimx_handle,Speed*2);  /* ARR设置为比较值2倍，这样输出的波形就是50%的占空比 */

//	TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);		//打开输出通道		STATE_STOP_straight
	
	if(g_pol_par.act_axis == AXIS_Y)
	{
		TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Enable);
		TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);		
	}
	else
	{
		TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Enable);
		TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
	}
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);			 			//使能定时器通道 
//	TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	
//    TIM_CCxChannelCmd(TIM8, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_ENABLE);    /* 使能通道输出 */
//    HAL_TIM_Base_Start_IT(&g_atimx_handle);             /* 使能定时器以及开启更新中断 */
    g_motor_sta_straight = STATE_RUN_straight;                            /* 标记电机正在运动 */
}
/**
 * @brief       实现任意象限直线插补
 * @param       coordsX    ：终点X轴坐标
 * @param       coordsY    ：终点Y轴坐标
 * @param       Speed      ：进给速度
 * @retval      无
*/
void line_inpolation( int32_t coordsX,int32_t coordsY, int32_t Speed)
{
    if(g_motor_sta_straight != STATE_STOP_straight)   /* 当前电机正在运转 */
       return ;
    /* 其他象限的直线跟第一象限是一样,只是电机运动方向不一样 */
	GPIO_SetBits(GPIOA,GPIO_Pin_12);	//PA12为使能引脚，输出高电平
	GPIO_SetBits(GPIOB,GPIO_Pin_5);	//PB5为使能引脚，输出高电平
    g_pol_par.moving_mode = LINE;
    if(coordsX < 0)                 /* 当x轴小于0时，电机方向设为反向*/
    {
        g_pol_par.x_dir = CCW;
        coordsX = -coordsX;         /* 取绝对值 */
		
		GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平
//		GPIO_SetBits(GPIOD,GPIO_Pin_7);		//PD7为方向引脚，输出高电平
//        ST_LINE_DIR(CCW,AXIS_X);
    }
    else
    {
        g_pol_par.x_dir = CW;
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平
//        ST_LINE_DIR(CW,AXIS_X);
    }
    if(coordsY < 0)                 /* 当y轴小于0时，电机方向设为反向*/
    {
        g_pol_par.y_dir = CCW;
        coordsY = -coordsY;         /* 取绝对值 */
		GPIO_SetBits(GPIOB,GPIO_Pin_6);
//        ST_LINE_DIR(CCW,AXIS_Y);
    }
    else
    {
        g_pol_par.y_dir = CW;
		GPIO_ResetBits(GPIOB,GPIO_Pin_6);
//        ST_LINE_DIR(CW,AXIS_Y);
    }
    line_incmove(coordsX,coordsY,Speed);
}

/**
 * @brief       定时器中断回调函数
 * @param       htim ： 定时器句柄
 * @retval      无
 */
void straight_speed_decision()
{
    __IO uint32_t axis = 0;         /* 进给轴 */
    axis = g_pol_par.act_axis;      /* 当前进给轴 */

	if (TIM_GetITStatus(TIM2, TIM_IT_Update)== SET)
	{	  
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	// 清除定时器中断	


		/* 判断是否到达终点或者还没开始运动 */
		if(g_pol_par.end_pulse == 0)
			return;
		/* 根据进给方向 更新坐标值 */
	  
		if(g_pol_par.moving_mode == LINE)
		{
			if(g_pol_par.f_e > 0)                                   /* 偏差方程 > 0 ,说明当前位置位于直线上方,应向X轴进给 */
			{
				g_pol_par.act_axis = AXIS_X;
				g_pol_par.f_e = g_pol_par.f_e - g_pol_par.end_y;    /* 第一象限的X轴进给时,偏差计算 */
			}
			else if(g_pol_par.f_e < 0)                              /* 偏差方程 < 0 ,说明当前位置位于直线下方,应向Y轴进给 */
			{
				g_pol_par.act_axis = AXIS_Y;
				g_pol_par.f_e = g_pol_par.f_e+g_pol_par.end_x;      /* 第一象限的Y轴进给时,偏差计算 */
			}
			/* 偏差为0的时候,判断x,y轴终点的大小决定进给方向 */
			else if(g_pol_par.f_e == 0)                             /* 偏差方程 = 0 ,说明当前位置位于直线,应判断终点坐标再进给 */
			{
				if(g_pol_par.end_y > g_pol_par.end_x)               /* 当Y轴更长的话，应向Y轴进给 */
				{
					g_pol_par.act_axis = AXIS_Y;
					g_pol_par.f_e = g_pol_par.f_e+g_pol_par.end_x;  /* 第一象限的Y轴进给时,偏差计算 */
				}
				else
				{
					g_pol_par.act_axis = AXIS_X;
					g_pol_par.f_e = g_pol_par.f_e - g_pol_par.end_y;
				}
			}
		}
		/* 判断是否需要跟换进给轴 */
		if(axis != g_pol_par.act_axis)
		{
			
			
				//	这里可能会出问题
//			TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
//			TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
		
			if(g_pol_par.act_axis == AXIS_Y)
			{
				TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Enable);
				TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
//				TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
			}
			else
			{
				TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
				TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Enable);
//				TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);

			}
			
			
			
//			TIM_CCxChannelCmd(TIM8, st_motor[axis].pulse_channel, TIM_CCx_DISABLE);
//			TIM_CCxChannelCmd(TIM8, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_ENABLE);
		}
		/* 终点判别:总步长 */
		g_pol_par.end_pulse--;
		if(g_pol_par.end_pulse == 0)
		{
			g_motor_sta_straight = STATE_STOP_straight;                               /* 到达终点 */
			//关闭使能
			GPIO_ResetBits(GPIOA,GPIO_Pin_12);	//PA12为使能引脚，输出低电平
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);	//PB5为使能引脚，输出低电平
			
			TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
			TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
//			TIM_IT_Update
			TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);	
//			TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);			 			//使能定时器通道 
//			TIM_ITConfig(TIM2,TIM_IT_CC2,DISABLE);
			
//			TIM_CCxChannelCmd(TIM8, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_DISABLE);/* 关闭当前轴输出 */
//			HAL_TIM_Base_Stop_IT(&g_atimx_handle);                  /* 停止定时器 */
		}
		
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	// 清除定时器中断	
		
		
	}
	
	
	
}
//TIM_IT_Update





//圆弧插补


/**
 * @brief       设置圆弧插补方向
 * @param       coordsX,coordsY:x,y坐标,dir: 圆弧的插补方向
 * @retval      无
 */
void setarcdir(int32_t coords_x,int32_t coords_y,int8_t dir)
 {
    g_pol_par.inter_dir = dir;
    if(g_pol_par.inter_dir == CCW)		/*逆时针*/ 
    {
        if(coords_x > 0)                /* 坐标x > 0 起点在X轴的正半轴,顺时针Y轴进给方向是负的 */
        {
            if(coords_y >= 0)           /* 如果y > 0,则是第一象限 */
            {
                g_pol_par.qua_points = FIRST_QUADRANT;		//第一象限
                g_pol_par.x_dir = CCW;			 /* X轴方向 		逆时针*/
                g_pol_par.y_dir = CW;			 /* Y轴方向		顺时针 */
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	反转
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		正转
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
            else										//第四象限
            {
                g_pol_par.qua_points = FOURTH_QUADRANT;
                g_pol_par.x_dir = CW;   /* y <= 0, 在第四象限 */
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	正转
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		正转
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
        else if(coords_x < 0)           /* X轴的负半轴 */
        {
            if(coords_y <= 0)           /* y < 0,则是第三象限 */
            {
                g_pol_par.qua_points = THIRD_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = -1;
//				GPIO_ResetBits
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	正转
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		反转
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
            else                        /* y >= 0,第二象限 */
            {
                g_pol_par.qua_points = SECOND_QUADRANT;
                g_pol_par.x_dir = CCW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	反转
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		反转
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
        }
        else if(coords_x == 0)          /* x = 0,在Y轴上的特殊起点 */
        {
            if(coords_y > 0)
            {
                g_pol_par.qua_points = SECOND_QUADRANT;
                g_pol_par.x_dir = CCW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	反转
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		反转
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
            else
            {
                g_pol_par.qua_points = FOURTH_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	正转
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		正转
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
    }
    else /* CW方向		顺时针 */
    {
        if(coords_x > 0)                /* 坐标x > 0 起点在X轴的正半轴,顺时针Y轴进给方向是负的 */
        {
            if(coords_y >0)             /* 如果y > 0,则是第一象限 */
            {
                g_pol_par.qua_points = FIRST_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	正转
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		反转
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
            else
            {
                g_pol_par.qua_points = FOURTH_QUADRANT;
                g_pol_par.x_dir = CCW;  /* y <= 0, 在第四象限 */
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	反转
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		反转
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
        }
        else if(coords_x < 0)           /* X轴的负半轴 */
        {
            if(coords_y < 0)            /* y < 0,则是第三象限 */
            {
                g_pol_par.qua_points = THIRD_QUADRANT;
                g_pol_par.x_dir = CCW;
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	反转
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		正转
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
            else                        /* y >= 0,第二象限 */
            {
                g_pol_par.qua_points = SECOND_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	正转
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		正转
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
        else if(coords_x == 0)          /* x = 0,在Y轴上的特殊起点 */
        {
            if(coords_y > 0)
            {
                g_pol_par.qua_points = FIRST_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	正转
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		反转
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
            else
            {
                g_pol_par.qua_points = THIRD_QUADRANT;
                g_pol_par.x_dir = CCW;
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11为方向引脚，输出高电平	X	反转
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		正转
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
    }
}


/**
 * @brief       在XOY平面内画任意圆弧
 * @param       start_x_point,start_y_point:  分别是起点坐标X,Y
 * @param       stop_x_point,stop_y_point:  分别是终点坐标X,Y
 * @param       speed:  速度值
 * @param       dir:    圆弧的方向
 * @retval      无
 */
void arc_incmove(int32_t start_x_point,int32_t start_y_point,int32_t stop_x_point,int32_t stop_y_point,uint32_t speed,int8_t dir)
{
    if(g_motor_sta_straight == STATE_RUN_straight)                            /*  当前电机正在运动 */
        return ;

	//STATE_RUN_straight
	
    /* 不符合圆的坐标方程 */
    if( (pow(start_x_point,2)+pow(start_y_point,2)) != (pow(stop_x_point,2)+pow(stop_y_point,2)))   /* 需要满足半径一致才是一个圆 */
        return ;
	GPIO_SetBits(GPIOA,GPIO_Pin_12);	//PA12为使能引脚，输出高电平
	GPIO_SetBits(GPIOB,GPIO_Pin_5);		//PB5为使能引脚，输出高电平
	
    g_pol_par.moving_mode = ARC;                            /* 圆弧标志 */
    
    g_pol_par.f_e = 0;                                      /* 偏差方程置零 */
    g_pol_par.start_point[AXIS_X] = start_x_point;
    g_pol_par.start_point[AXIS_Y] = start_y_point;
    g_pol_par.end_x = stop_x_point;
    g_pol_par.end_y = stop_y_point;
    /* 设置电机做逆时针圆弧轨迹的运动方向 */
    setarcdir(g_pol_par.start_point[AXIS_X],g_pol_par.start_point[AXIS_Y],dir);                     /* 设置圆弧插补方向 */
    /* 计算总的步数 */  
    g_pol_par.end_pulse = abs((stop_y_point-start_y_point))+ abs((stop_x_point-start_x_point));     /* 从起点到终点的脉冲数 */
    /* 起点坐标x = 0,说明起点位于y轴上,此时往X轴进给误差会减少 */
    if(g_pol_par.start_point[AXIS_X] == 0)
    {
        g_pol_par.act_axis = AXIS_X;                        /* 第一步给X轴 */
        /* 根据圆弧方向决定向X轴进给的偏差方程 */
        g_pol_par.f_e = g_pol_par.f_e + sign_dir[AXIS_X]*g_pol_par.start_point[AXIS_X]*2 + 1;       /* 偏差方程的计算 */
    }
    else
    {
        g_pol_par.act_axis = AXIS_Y;                        /* 第一步给Y轴 */
        g_pol_par.f_e = g_pol_par.f_e + sign_dir[AXIS_Y]*g_pol_par.start_point[AXIS_Y]*2 + 1;       /* 偏差方程的计算 */
    }
    
//    ST_LINE_EN(EN,AXIS_X);
//    ST_LINE_EN(EN,AXIS_Y);
	
	TIM_SetCompare1(TIM2, speed);
	TIM_SetCompare2(TIM2, speed);
	TIM_SetAutoreload(TIM2, speed*2);
	
	
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_X].pulse_channel,speed);                    /* 设置通道的比较值 */
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_Y].pulse_channel,speed);
//    __HAL_TIM_SET_AUTORELOAD(&g_atimx_handle,speed*2);      /* ARR设置为比较值2倍，这样输出的波形就是50%的占空比 */

	if(g_pol_par.act_axis == AXIS_Y)
	{
		TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Enable);
		TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);		
	}
	else
	{
		TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Enable);
		TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
	}
 
//    TIM_CCxChannelCmd(ATIM_TIMX_PWM, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_ENABLE);		STATE_STOP
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);			 			//使能定时器通道 
	TIM_Cmd(TIM2, ENABLE);
	
//    HAL_TIM_Base_Start_IT(&g_atimx_handle);                 /* 使能定时器以及开启更新中断 */
	
	
    g_motor_sta_straight = STATE_RUN_straight;                                /* 标记电机正在运动 */
}



/**
 * @brief       定时器中断回调函数
 * @param       htim ： 定时器句柄
 * @retval      无
 */
void circle_speed_decision()
{
    __IO uint32_t axis = 0;         /* 进给轴 */
    axis = g_pol_par.act_axis;      /* 当前进给轴 */

	if (TIM_GetITStatus(TIM2, TIM_IT_Update)== SET)
	{	  
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	// 清除定时器中断	

		/* 判断是否到达终点或者还没开始运动 */
		if(g_pol_par.end_pulse == 0)
			return;
		/* 计算新的动点坐标 */
		if(g_pol_par.act_axis == AXIS_X)
		{
			if(g_pol_par.x_dir == CCW)
			{
				g_pol_par.start_point[AXIS_X]--;
			}
			else
			{
				g_pol_par.start_point[AXIS_X]++;
			}
		}
		if(g_pol_par.act_axis == AXIS_Y)
		{
			if(g_pol_par.y_dir == CCW)
			{
				g_pol_par.start_point[AXIS_Y]--;
			}
			else
			{
				g_pol_par.start_point[AXIS_Y]++;
			}
		}    
		
		/* 根据进给方向 更新坐标值 */
		if(g_pol_par.moving_mode == ARC)
		{
			/* 根据上一次的偏差判断下一步进给方向,同时计算下一次的偏差 */
			if(g_pol_par.inter_dir == CCW)          /* 插补方向:逆时针圆弧 */
			{
				if(g_pol_par.f_e < 0)               /* 偏差方程 < 0 ,说明当前位置位于圆弧内侧,应向圆外进给 */
				{
					if( (g_pol_par.qua_points == SECOND_QUADRANT) || (g_pol_par.qua_points == FOURTH_QUADRANT) )/*  第二和第四象限,当偏差<0时都是向X轴进给 */
					{
						g_pol_par.act_axis = AXIS_X;
					}
					else                            /* 在第一和第三象限,偏差<0,都是向Y轴进给 */
					{
						g_pol_par.act_axis = AXIS_Y;
					}
				}
				else if(g_pol_par.f_e >= 0)         /* 偏差方程 >= 0 ,说明当前位置位于圆弧外侧,应向圆内进给 */
				{
					if( (g_pol_par.qua_points == SECOND_QUADRANT) || (g_pol_par.qua_points == FOURTH_QUADRANT) )
					{
						g_pol_par.act_axis = AXIS_Y;
					}
					else
					{
						g_pol_par.act_axis = AXIS_X;
					}
				}
			}
			else
			{
				if(g_pol_par.f_e < 0)               /* 偏差方程 < 0 ,说明当前位置位于圆弧内侧,应向圆外进给 */
				{
					if( (g_pol_par.qua_points == FIRST_QUADRANT) || (g_pol_par.qua_points == THIRD_QUADRANT) )/*  第一和第三象限,当偏差<0时都是向X轴进给 */
					{
						g_pol_par.act_axis = AXIS_X;
					}
					else                            /* 在第二和第四象限,偏差<0,都是向Y轴进给 */
					{
						g_pol_par.act_axis = AXIS_Y;
					}
				}
				else if(g_pol_par.f_e >= 0)         /* 偏差方程 >= 0 ,说明当前位置位于圆弧外侧,应向圆内进给 */
				{
					if( (g_pol_par.qua_points == FIRST_QUADRANT) || (g_pol_par.qua_points == THIRD_QUADRANT) )
					{
						g_pol_par.act_axis = AXIS_Y;
					}
					else
					{
						g_pol_par.act_axis = AXIS_X;
					}
				}
			}
			/* 计算当前坐标与目标曲线的偏差 */
			g_pol_par.f_e = g_pol_par.f_e + 2*sign_dir[g_pol_par.act_axis]*g_pol_par.start_point[g_pol_par.act_axis] + 1;   /* 偏差方程的计算 */
		}   
		
		/* 判断是否需要跟换进给轴 */
		if(axis != g_pol_par.act_axis)
		{
			
			
//			TIM_CCxChannelCmd(ATIM_TIMX_PWM, st_motor[axis].pulse_channel, TIM_CCx_DISABLE);
//			TIM_CCxChannelCmd(ATIM_TIMX_PWM, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_ENABLE);
			
//			TIM_CCxChannelCmd(TIM8, st_motor[axis].pulse_channel, TIM_CCx_DISABLE);
//			TIM_CCxChannelCmd(TIM8, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_ENABLE);
			
			if(g_pol_par.act_axis == AXIS_Y)
			{
				TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Enable);
				TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
//				TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
			}
			else
			{
				TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
				TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Enable);
//				TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);

			}
			
			
			
		}
		/* 终点判别:总步长 */
		g_pol_par.end_pulse--;
		if(g_pol_par.end_pulse == 0)
		{
			g_motor_sta_straight = STATE_STOP_straight;               /*  到达终点 */
			
			//关闭使能
			GPIO_ResetBits(GPIOA,GPIO_Pin_12);	//PA12为使能引脚，输出低电平
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);	//PB5为使能引脚，输出低电平
			TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
			TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
			TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
			
			
//			TIM_CCxChannelCmd(ATIM_TIMX_PWM, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_DISABLE);  /* 关闭当前轴输出 */
//			HAL_TIM_Base_Stop_IT(&g_atimx_handle);  /*  停止定时器 */
		}
		
	}
	
	
	
}


void TIM2_IRQHandler(void)
{
	
	if(g_pol_par.moving_mode == LINE)
	{
		straight_speed_decision();
	}
	
	else
	{
		circle_speed_decision();
	}
	
//	circle_speed_decision();
}

