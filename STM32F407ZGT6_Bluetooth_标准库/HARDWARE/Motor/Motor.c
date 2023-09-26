#include "stm32f4xx.h"                  // Device header
#include "Motor.h"  
#include "Motor_Init.h"  
#include <math.h>
//���μӼ���
//


speedRampData srd= {STOP,0,0,0,0,0,0}; //�Ӽ��ٱ���    

uint8_t  motor_sta        = 0;//���״̬


/*
step   �ƶ�����������Ϊ��ת������Ϊ��ʱ�룩
accel  ���ٶ�,ʵ��ֵΪaccel*0.1*rad/sec^2  10������2��������һ������������
decel  ���ٶ�,ʵ��ֵΪdecel*0.1*rad/sec^2
speed  ����ٶ�,ʵ��ֵΪspeed*0.1*rad/sec
 */
void MOTOR_Move_T(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
		
    uint16_t tim_count; 							 //����ж�ʱ�̵ļ���ֵ
    unsigned int max_s_lim;                          //�ﵽ����ٶ�ʱ�Ĳ���    
    unsigned int accel_lim;							 //���뿪ʼ���ٵĲ����������û�м��ٶȵ�����ٶ�ʱ��

	if(motor_sta!= STOP)  							//ֻ�����������ֹͣ��ʱ��ż���
		return;			
    if(step < 0)   									//����ת	 //��ʱ��
	{
		GPIO_SetBits(GPIOG,GPIO_Pin_14);
		GPIO_SetBits(GPIOG,GPIO_Pin_13);		//PG13Ϊ�������ţ�����ߵ�ƽ
		step = -step;      
	}		
	else   											 //˳ʱ��
	{
		GPIO_SetBits(GPIOG,GPIO_Pin_14);
		GPIO_ResetBits(GPIOG,GPIO_Pin_13);			//PA13Ϊ�������ţ�����͵�ƽ		
	}   
	
    if(step == 1)   								 // ���ֻ�ƶ�һ��
    {       
		srd.accel_count = -1; 						 // ֻ�ƶ�һ��
        
		srd.run_state = DECEL;						 // ����״̬
        
		srd.step_delay = 1000;						// ����ʱ

     }
    
    else if(step != 0)  						// ������Ϊ����ƶ�
    {					
		srd.min_delay = (int32_t)(A_T_x10/speed);
		// ��������ٶȼ���, ����min_delay���ڶ�ʱ���ļ�������ֵmin_delay = (alpha / tt)/ w   
		srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
		// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶ�,����accel��λΪ0.01rad/sec^2
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100 
		max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
		//������ٲ�֮��ﵽ����ٶȵ����� max_s_lim = speed^2 / (2*alpha*accel)

		if(max_s_lim == 0)							//����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0,��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ� 
		{
			max_s_lim = 1;
		}    
		accel_lim = (uint32_t)(step*decel/(accel+decel)); 
		// ������ٲ�֮�����Ǳ��뿪ʼ����,n1 = (n1+n2)decel / (accel + decel)

		if(accel_lim == 0)			// ���Ǳ����������1�����ܿ�ʼ����
		{
			accel_lim = 1;
		}

		if(accel_lim <= max_s_lim)										
			//���ٽ׶ε���������ٶȾ͵ü��١�����ʹ�������������ǿ��Լ�������ٽ׶β��� 
		{
			srd.decel_val = accel_lim - step;							//���ٶεĲ���
		}
		else
		{
			srd.decel_val = -(max_s_lim*accel/decel);					//���ٶεĲ��� 
		}
   
		if(srd.decel_val == 0) 										// ����һ�� ��һ������ 
		{
			srd.decel_val = -1;
		}    
		srd.decel_start = step + srd.decel_val;							//���㿪ʼ����ʱ�Ĳ���


		if(srd.step_delay <= srd.min_delay)	// ���һ��ʼc0���ٶȱ����ٶ��ٶȻ��󣬾Ͳ���Ҫ���м����˶���ֱ�ӽ�������
		{
			srd.step_delay = srd.min_delay;
			srd.run_state = RUN;
		}
		else
		{
			srd.run_state = ACCEL;
		}
    
		srd.accel_count = 0;										// ��λ���ٶȼ���ֵ

	}
	
	
	motor_sta = 1;  														// ���Ϊ�˶�״̬
	tim_count = TIM_GetCounter(TIM3);												//��ȡ����ֵ
	TIM_SetCompare1(TIM3,tim_count+srd.step_delay/2);								//���ö�ʱ���Ƚ�ֵ 
	TIM_SetCompare2(TIM3,tim_count+srd.step_delay/2);								//���ö�ʱ���Ƚ�ֵ 
	
	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);											//ʹ�ܶ�ʱ��ͨ�� 
	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);											//ʹ�ܶ�ʱ��ͨ�� 
	
	TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Enable);
	TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);
	
	
	TIM_Cmd(TIM3, ENABLE);										//������ʱ��
}
 


void speed_decision()                                                         //�ж�ִ�к���
{
	__IO uint32_t tim_count=0;
	__IO uint32_t tmp = 0;  
	uint16_t new_step_delay=0;                                  // �����£��£�һ����ʱ����  
	__IO static uint16_t last_accel_delay=0;                    // ���ٹ��������һ����ʱ���������ڣ�. 
	__IO static uint32_t step_count = 0; 				 // ���ƶ�����������  
	__IO static int32_t rest = 0;		//����			// ��¼new_step_delay�е������������һ������ľ���  
	__IO static uint8_t i=0;						//��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
	 
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1)== SET)
	{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);	// �����ʱ���ж�		
		tim_count = TIM_GetCounter(TIM3);			//��ȡ��ʱ��2�ļ���ֵ		//��ȡ����ֵ
		tmp = tim_count+srd.step_delay/2;		//	int step_delay;   �¸��������ڣ�ʱ����������ʱΪ���ٶ�
		TIM_SetCompare1(TIM3,tmp);				//���ǲ��ϵظ���CCR1��ֵ						// ���ñȽ�ֵ
		i++; 
		if(i==2)													//�ж�����Ϊһ������
		{
			i=0; 
			switch(srd.run_state)
			{
			case STOP:												//ֹͣ״̬
				step_count = 0;
				rest = 0;
					
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
				TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Disable);
				TIM_Cmd(TIM3, DISABLE);					   //����������Թرն�ʱ����������ֻ�ر�ͨ������
				motor_sta = 0;  
				break;
				
			case ACCEL:											//����״̬
				step_count++;
				srd.accel_count++;
				new_step_delay = srd.step_delay - (((2 *srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			//������(��)һ����������(ʱ����)
				rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// �����������´μ��㲹���������������
				
				if(step_count >= srd.decel_start) 						//����ǹ�Ӧ�ÿ�ʼ����
				{
					srd.accel_count = srd.decel_val;				//���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
					srd.run_state = DECEL;							//�¸����������ٽ׶� 
				}
				
				else if(new_step_delay <= srd.min_delay)			//����Ƿ񵽴�����������ٶ�
				{
					last_accel_delay = new_step_delay;				//������ٹ��������һ����ʱ���������ڣ�
					new_step_delay = srd.min_delay;   				// ʹ��min_delay����Ӧ����ٶ�speed�� 
					rest = 0;            						//������ֵ               
					srd.run_state = RUN;						//����Ϊ��������״̬ 
				}
				break;
					
			case RUN:
				step_count++;  									// ������1				  
				new_step_delay = srd.min_delay;   				 // ʹ��min_delay����Ӧ����ٶ�speed��				 
				if(step_count >= srd.decel_start)   			// ��Ҫ��ʼ����
				{
					srd.accel_count = srd.decel_val;  			// ���ٲ�����Ϊ���ټ���ֵ
					new_step_delay = last_accel_delay;		// �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
					srd.run_state = DECEL;           	  // ״̬�ı�Ϊ����
				}
				break;
					
			case DECEL:
				step_count++;  									// ������1

				srd.accel_count++; 								// �Ǹ�����
				new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest)/(4 * srd.accel_count + 1)); 
			//������(��)һ����������(ʱ����)
				rest = ((2 * srd.step_delay)+rest)%(4 * srd.accel_count + 1);// �����������´μ��㲹���������������
				if(srd.accel_count >= 0) 						//����Ƿ�Ϊ���һ��  �Ǹ���������Ҫ�ж� ���ڵ�����ʱ Ӧ�þ��Ǽ��ٽ���
				{
					srd.run_state = STOP;
				}
				break;
			}
			srd.step_delay = new_step_delay; 				// Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
		}
	}
	
	
}
	

//void TIM3_IRQHandler(void)
//{
//	speed_decision();
//}



