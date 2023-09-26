#include "stm32f4xx.h"                  // Device header
#include "Motor.h"  
#include "Motor_Init.h"  
#include <math.h>
//梯形加减速
//


speedRampData srd= {STOP,0,0,0,0,0,0}; //加减速变量    

uint8_t  motor_sta        = 0;//电机状态


/*
step   移动步数（正数为正转，负数为逆时针）
accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
decel  减速度,实际值为decel*0.1*rad/sec^2
speed  最大速度,实际值为speed*0.1*rad/sec
 */
void MOTOR_Move_T(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
		
    uint16_t tim_count; 							 //存放中断时刻的计数值
    unsigned int max_s_lim;                          //达到最大速度时的步数    
    unsigned int accel_lim;							 //必须开始减速的步数（如果还没有加速度到最大速度时）

	if(motor_sta!= STOP)  							//只允许步进电机在停止的时候才继续
		return;			
    if(step < 0)   									//正反转	 //逆时针
	{
		GPIO_SetBits(GPIOG,GPIO_Pin_14);
		GPIO_SetBits(GPIOG,GPIO_Pin_13);		//PG13为方向引脚，输出高电平
		step = -step;      
	}		
	else   											 //顺时针
	{
		GPIO_SetBits(GPIOG,GPIO_Pin_14);
		GPIO_ResetBits(GPIOG,GPIO_Pin_13);			//PA13为方向引脚，输出低电平		
	}   
	
    if(step == 1)   								 // 如果只移动一步
    {       
		srd.accel_count = -1; 						 // 只移动一步
        
		srd.run_state = DECEL;						 // 减速状态
        
		srd.step_delay = 1000;						// 短延时

     }
    
    else if(step != 0)  						// 步数不为零才移动
    {					
		srd.min_delay = (int32_t)(A_T_x10/speed);
		// 设置最大速度极限, 计算min_delay用于定时器的计数器的值min_delay = (alpha / tt)/ w   
		srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
		// 通过计算第一个(c0) 的步进延时来设定加速度,其中accel单位为0.01rad/sec^2
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100 
		max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
		//计算多少步之后达到最大速度的限制 max_s_lim = speed^2 / (2*alpha*accel)

		if(max_s_lim == 0)							//如果达到最大速度小于0.5步，我们将四舍五入为0,但实际我们必须移动至少一步才能达到想要的速度 
		{
			max_s_lim = 1;
		}    
		accel_lim = (uint32_t)(step*decel/(accel+decel)); 
		// 计算多少步之后我们必须开始减速,n1 = (n1+n2)decel / (accel + decel)

		if(accel_lim == 0)			// 我们必须加速至少1步才能开始减速
		{
			accel_lim = 1;
		}

		if(accel_lim <= max_s_lim)										
			//加速阶段到不了最大速度就得减速。。。使用限制条件我们可以计算出减速阶段步数 
		{
			srd.decel_val = accel_lim - step;							//减速段的步数
		}
		else
		{
			srd.decel_val = -(max_s_lim*accel/decel);					//减速段的步数 
		}
   
		if(srd.decel_val == 0) 										// 不足一步 按一步处理 
		{
			srd.decel_val = -1;
		}    
		srd.decel_start = step + srd.decel_val;							//计算开始减速时的步数


		if(srd.step_delay <= srd.min_delay)	// 如果一开始c0的速度比匀速段速度还大，就不需要进行加速运动，直接进入匀速
		{
			srd.step_delay = srd.min_delay;
			srd.run_state = RUN;
		}
		else
		{
			srd.run_state = ACCEL;
		}
    
		srd.accel_count = 0;										// 复位加速度计数值

	}
	
	
	motor_sta = 1;  														// 电机为运动状态
	tim_count = TIM_GetCounter(TIM3);												//获取计数值
	TIM_SetCompare1(TIM3,tim_count+srd.step_delay/2);								//设置定时器比较值 
	TIM_SetCompare2(TIM3,tim_count+srd.step_delay/2);								//设置定时器比较值 
	
	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);											//使能定时器通道 
	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);											//使能定时器通道 
	
	TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Enable);
	TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);
	
	
	TIM_Cmd(TIM3, ENABLE);										//开启定时器
}
 


void speed_decision()                                                         //中断执行函数
{
	__IO uint32_t tim_count=0;
	__IO uint32_t tmp = 0;  
	uint16_t new_step_delay=0;                                  // 保存新（下）一个延时周期  
	__IO static uint16_t last_accel_delay=0;                    // 加速过程中最后一次延时（脉冲周期）. 
	__IO static uint32_t step_count = 0; 				 // 总移动步数计数器  
	__IO static int32_t rest = 0;		//余数			// 记录new_step_delay中的余数，提高下一步计算的精度  
	__IO static uint8_t i=0;						//定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
	 
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1)== SET)
	{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);	// 清除定时器中断		
		tim_count = TIM_GetCounter(TIM3);			//获取定时器2的计数值		//获取计数值
		tmp = tim_count+srd.step_delay/2;		//	int step_delay;   下个脉冲周期（时间间隔）启动时为加速度
		TIM_SetCompare1(TIM3,tmp);				//就是不断地更新CCR1的值						// 设置比较值
		i++; 
		if(i==2)													//中断两次为一个脉冲
		{
			i=0; 
			switch(srd.run_state)
			{
			case STOP:												//停止状态
				step_count = 0;
				rest = 0;
					
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
				TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Disable);
				TIM_Cmd(TIM3, DISABLE);					   //单个电机可以关闭定时器，多个电机只关闭通道即可
				motor_sta = 0;  
				break;
				
			case ACCEL:											//加速状态
				step_count++;
				srd.accel_count++;
				new_step_delay = srd.step_delay - (((2 *srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			//计算新(下)一步脉冲周期(时间间隔)
				rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
				
				if(step_count >= srd.decel_start) 						//检查是够应该开始减速
				{
					srd.accel_count = srd.decel_val;				//加速计数值为减速阶段计数值的初始值
					srd.run_state = DECEL;							//下个脉冲进入减速阶段 
				}
				
				else if(new_step_delay <= srd.min_delay)			//检查是否到达期望的最大速度
				{
					last_accel_delay = new_step_delay;				//保存加速过程中最后一次延时（脉冲周期）
					new_step_delay = srd.min_delay;   				// 使用min_delay（对应最大速度speed） 
					rest = 0;            						//清零余值               
					srd.run_state = RUN;						//设置为匀速运行状态 
				}
				break;
					
			case RUN:
				step_count++;  									// 步数加1				  
				new_step_delay = srd.min_delay;   				 // 使用min_delay（对应最大速度speed）				 
				if(step_count >= srd.decel_start)   			// 需要开始减速
				{
					srd.accel_count = srd.decel_val;  			// 减速步数做为加速计数值
					new_step_delay = last_accel_delay;		// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
					srd.run_state = DECEL;           	  // 状态改变为减速
				}
				break;
					
			case DECEL:
				step_count++;  									// 步数加1

				srd.accel_count++; 								// 是个负数
				new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest)/(4 * srd.accel_count + 1)); 
			//计算新(下)一步脉冲周期(时间间隔)
				rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
				if(srd.accel_count >= 0) 						//检查是否为最后一步  是个负数所以要判断 大于等于零时 应该就是减速结束
				{
					srd.run_state = STOP;
				}
				break;
			}
			srd.step_delay = new_step_delay; 				// 为下个(新的)延时(脉冲周期)赋值
		}
	}
	
	
}
	

//void TIM3_IRQHandler(void)
//{
//	speed_decision();
//}



