#include "malloc.h"
#include "usart.h"
#include "Motor_S.h"  
#include "Motor_Init.h"  
#include <math.h>
#include "Motor.h"
/**
 * @brief       ��ʼ������������IO��, ��ʹ��ʱ��
 * @retval      ��
 */
//void stepper_init(uint16_t arr, uint16_t psc)
//}

/**
 * @brief       �����������
 */
//void stepper_star(uint8_t motor_num)
//}

/**
 * @brief       �رղ������
 */
//void stepper_stop(uint8_t motor_num)
//}

/****************************************S�ͼӼ����˶�*****************************************************/
volatile int32_t g_step_pos = 0;                        /* ��ǰλ�� */
volatile uint16_t g_toggle_pulse  = 0;                  /* ����Ƶ�ʿ��� */
motor_state_typedef g_motor_sta  = STATE_IDLE;          /* ���״̬ */
speed_calc_t g_calc_t = {0};

__IO  uint32_t g_add_pulse_count=0;                     /* ��������ۼ� */


/**
 * @brief       �ٶȱ���㺯��
 * @param       vo,���ٶ�;vt,ĩ�ٶ�;time,����ʱ��
 * @retval      TRUE���ɹ���FALSE��ʧ��
 */
uint8_t calc_speed(int32_t vo, int32_t vt, float time)
{
	
//	myfree(SRAMIN,g_calc_t.accel_tab);                          /* �ͷż��ٶ��ٶȱ� */
//	myfree(SRAMIN,g_calc_t.decel_tab);                          /* �ͷż��ٶ��ٶȱ� */
    uint8_t is_dec = FALSE;
    int32_t i = 0;
    int32_t vm =0;                              /* �м���ٶ� */
    int32_t inc_acc_stp = 0;                    /* �Ӽ�������Ĳ��� */
    int32_t dec_acc_stp = 0;                    /* ����������Ĳ��� */
    int32_t accel_step = 0;                     /* ���ٻ������Ҫ�Ĳ��� */
    float jerk = 0;                             /* �Ӽ��ٶ� */
    float ti = 0;                               /* ʱ���� dt */
    float sum_t = 0;                            /* ʱ���ۼ��� */
    float delta_v = 0;                          /* �ٶȵ�����dv */
    float ti_cube = 0;                          /* ʱ���������� */
    float *velocity_tab = NULL;                 /* �ٶȱ��ָ�� */
    
    if(vo > vt )                                /* ���ٶȱ�ĩ�ٶȴ�,�������˶�,��ֵ�仯�������˶���ͬ */
    {                                           /* ֻ�ǽ����ʱ��ע�⽫�ٶȵ��� */
        is_dec = TRUE;                          /* ���ٶ� */
        g_calc_t.vo = ROUNDPS_2_STEPPS(vt);     /* ת����λ ����:step/s */
        g_calc_t.vt = ROUNDPS_2_STEPPS(vo);     /* ת����λ ĩ��:step/s */
    }
    else
    {
        is_dec = FALSE;                         /* ���ٶ� */
        g_calc_t.vo = ROUNDPS_2_STEPPS(vo);
        g_calc_t.vt = ROUNDPS_2_STEPPS(vt);
    }

    time = ACCEL_TIME(time);                                                    /* �õ��Ӽ��ٶε�ʱ�� */
    printf("time=%f\r\n",time);
    vm =  (g_calc_t.vo + g_calc_t.vt) / 2 ;                                     /* �����е��ٶ� */
    
    jerk = fabs(2.0f * (vm - g_calc_t.vo) /  (time * time));                    /* �����е��ٶȼ���Ӽ��ٶ� */

    inc_acc_stp = (int32_t)(g_calc_t.vo * time + INCACCELSTEP(jerk,time));      /* �Ӽ�����Ҫ�Ĳ��� */

    dec_acc_stp = (int32_t)((g_calc_t.vt + g_calc_t.vo) * time - inc_acc_stp);  /* ��������Ҫ�Ĳ��� S = vt * time - S1 */

    /* �����ڴ�ռ����ٶȱ� */
    accel_step = dec_acc_stp + inc_acc_stp;                                     /* ������Ҫ�Ĳ��� */
    if( accel_step  % 2 != 0)                                                   /* ���ڸ���������ת�����������ݴ��������,���������1 */
        accel_step  += 1;
    /* mallo�����ڴ�ռ�,�ǵ��ͷ� */
    velocity_tab = (float*)(mymalloc(SRAMIN,((accel_step + 1) * sizeof(float))));
    if(velocity_tab == NULL)
    {
        printf("�ڴ治��!���޸Ĳ���\r\n");
        return FALSE;
    }
/*
 * Ŀ���S���ٶ������Ƕ�ʱ��ķ���,�����ڿ��Ƶ����ʱ�������Բ����ķ�ʽ����,���������V-t������ת��
 * �õ�V-S����,����õ����ٶȱ��ǹ��ڲ������ٶ�ֵ.ʹ�ò������ÿһ�����ڿ��Ƶ���
 */
/* �����һ���ٶ�,���ݵ�һ�����ٶ�ֵ�ﵽ��һ����ʱ�� */
    ti_cube  = 6.0f * 1.0f / jerk;                  /* ����λ�ƺ�ʱ��Ĺ�ʽS = 1/6 * J * ti^3 ��1����ʱ��:ti^3 = 6 * 1 / jerk */
    ti = pow(ti_cube,(1 / 3.0f));                   /* ti */
    sum_t = ti;
    delta_v = 0.5f * jerk * pow(sum_t,2);           /* ��һ�����ٶ� */
    velocity_tab[0] = g_calc_t.vo + delta_v;

/*****************************************************/
    if( velocity_tab[0] <= SPEED_MIN )              /* �Ե�ǰ��ʱ��Ƶ�����ܴﵽ������ٶ� */
        velocity_tab[0] = SPEED_MIN;
    
/*****************************************************/
    
    for(i = 1; i < accel_step; i++)
    {
        /* ����������ٶȾ��Ƕ�ʱ���������Ƶ��,���Լ����ÿһ����ʱ�� */
        /* �õ���i-1����ʱ�� */
        ti = 1.0f / velocity_tab[i-1];              /* ���ÿ��һ����ʱ�� ti = 1 / Vn-1 */
        /* �Ӽ��ٶ��ٶȼ��� */
        if( i < inc_acc_stp)
        {
            sum_t += ti;                            /* ��0��ʼ��i��ʱ���ۻ� */
            delta_v = 0.5f * jerk * pow(sum_t,2);   /* �ٶȵı仯��: dV = 1/2 * jerk * ti^2 */
            velocity_tab[i] = g_calc_t.vo + delta_v;/* �õ��Ӽ��ٶ�ÿһ����Ӧ���ٶ� */
            /* �����һ����ʱ��,ʱ�䲢���ϸ����time,��������Ҫ��������,��Ϊ�����ٶε�ʱ�� */
            if(i == inc_acc_stp - 1)
                sum_t  = fabs(sum_t - time );
        }
        /* �����ٶ��ٶȼ��� */
        else
        {
            sum_t += ti;                                        /* ʱ���ۼ� */
            delta_v = 0.5f * jerk * pow(fabs( time - sum_t),2); /* dV = 1/2 * jerk *(T-t)^2 ��������򿴼����ٵ�ͼ */
            velocity_tab[i] = g_calc_t.vt - delta_v;            /* V = vt - delta_v */
            if(velocity_tab[i] >= g_calc_t.vt)
            {
                accel_step = i;
                break;
            }
        }
    }
    if(is_dec == TRUE)                                          /* ���� */
    {
        float tmp_Speed = 0;
        /* �������� */
        for(i = 0; i< (accel_step / 2); i++)
        {
            tmp_Speed = velocity_tab[i];
            velocity_tab[i] = velocity_tab[accel_step-1 - i];   /* ͷβ�ٶȶԻ� */
            velocity_tab[accel_step-1 - i] = tmp_Speed;
        }

        g_calc_t.decel_tab = velocity_tab;                      /* ���ٶ��ٶȱ� */
        g_calc_t.decel_step = accel_step;                       /* ���ٶε��ܲ��� */

    }
    else                                                        /* ���� */
    {
        g_calc_t.accel_tab = velocity_tab;                      /* ���ٶ��ٶȱ� */
        g_calc_t.accel_step = accel_step;                       /* ���ٶε��ܲ��� */
    }
    return TRUE;
}


/**
 * @brief       S�ͼӼ����˶�
 * @param       vo:���ٶ�;vt:ĩ�ٶ�;AcTime:����ʱ��;DeTime:����ʱ��;step:����;count��У��λ����count�Լ��ӵģ����������ڴ治�㣩
 * @retval      ��
 */
void stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step)
{
	if(calc_speed(vo,vt,AcTime) == FALSE) /* ��������ٶε��ٶȺͲ��� */
		return;
	if(calc_speed(vt,vo,DeTime) == FALSE) /* ��������ٶε��ٶȺͲ��� */
		return;
		
    if(step < 0)
    {
		GPIO_SetBits(GPIOC,GPIO_Pin_10);	//PC10Ϊʹ�����ţ�����ߵ�ƽ
		GPIO_SetBits(GPIOD,GPIO_Pin_7);		//PD7Ϊ�������ţ�����ߵ�ƽ
		step = -step; 
    }
    else
    {
		GPIO_SetBits(GPIOC,GPIO_Pin_10);	//PC10Ϊʹ�����ţ�����ߵ�ƽ
		GPIO_ResetBits(GPIOD,GPIO_Pin_7);	//PD7Ϊ�������ţ�����͵�ƽ	
    }
    
    if(step >= (g_calc_t.decel_step+g_calc_t.accel_step) )          /* ���ܲ������ڵ��ڼӼ��ٶȲ������ʱ���ſ���ʵ��������S�μӼ��� */
    {
        g_calc_t.step = step;
        g_calc_t.dec_point = g_calc_t.step - g_calc_t.decel_step;   /* ��ʼ���ٵĲ��� */
    }
    else                                                            /* ���������Խ����㹻�ļӼ��� */
    {
        /* �������㲻�����˶���Ҫ��ǰ��������ٶȱ���ռ�ڴ��ͷţ��Ա�������ظ����� */
        myfree(SRAMIN,g_calc_t.accel_tab);                          /* �ͷż��ٶ��ٶȱ� */
        myfree(SRAMIN,g_calc_t.decel_tab);                          /* �ͷż��ٶ��ٶȱ� */
        printf("�������㣬�������ô���!\r\n");
        return;
    }
    g_calc_t.step_pos = 0;
    g_motor_sta = STATE_ACCEL;                                      /* ���Ϊ����״̬ */

    g_calc_t.ptr = g_calc_t.accel_tab;                              /* �Ѽ��ٶε��ٶȱ�洢��ptr��� */
    g_toggle_pulse  = (uint32_t)(T1_FREQ_S/(*g_calc_t.ptr));
    g_calc_t.ptr++;

	TIM_SetCounter(TIM3, 0);		//��ռ���ֵ
	TIM_SetCompare2(TIM3,(uint16_t)(g_toggle_pulse/2));			//�����ͨ�����ô����²�������ת
	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);			 			//ʹ�ܶ�ʱ��ͨ�� 
	TIM_Cmd(TIM3, ENABLE);

}

void speed_decision_s()
{
    volatile uint32_t Tim_Count = 0;
    volatile uint32_t tmp = 0;
    volatile float Tim_Pulse = 0;
    volatile static uint8_t i = 0;
	
	
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2)== SET)
	{	
		
	
		i++;                    /* ��ʱ���жϴ�������ֵ */
        if(i == 2)              /* 2�Σ�˵���Ѿ����һ���������� */
        {
            i = 0;              /* ���㶨ʱ���жϴ�������ֵ */
            g_step_pos ++;      /* ��ǰλ�� */
            if((g_motor_sta!=STATE_IDLE)&&(g_motor_sta != STATE_STOP))
            {
                g_calc_t.step_pos ++;
            }
            switch(g_motor_sta)
            {
                case STATE_ACCEL:		//����״̬
					TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);		//�����ͨ��
                    g_add_pulse_count++;
                    Tim_Pulse = T1_FREQ_S / (*g_calc_t.ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
                    g_calc_t.ptr++;                                 /* ȡ�ٶȱ����һλ */
                    g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);    /* ��תģʽC��Ҫ����2 */
                    if(g_calc_t.step_pos >= g_calc_t.accel_step)    /* �����ڼ��ٶβ����ͽ������� */
                    {
                        myfree(SRAMIN,g_calc_t.accel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
                        g_motor_sta = STATE_AVESPEED;
                    }
                    break;
                case STATE_DECEL:		//����״̬
//					TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);		//�����ͨ��
                    g_add_pulse_count++;
                    Tim_Pulse = T1_FREQ_S / (*g_calc_t.ptr);          /* ���ٶȱ�õ�ÿһ���Ķ�ʱ������ֵ */
                    g_calc_t.ptr++;
                    g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);
                    if(g_calc_t.step_pos >= g_calc_t.step )
                    {
                        myfree(SRAMIN,g_calc_t.decel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
                        g_motor_sta = STATE_STOP;
                    }
                    break;
                case STATE_AVESPEED:		//����״̬
//					TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);		//�����ͨ��
                    g_add_pulse_count++;
                    Tim_Pulse  = T1_FREQ_S /g_calc_t.vt;
                    g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);
                    if(g_calc_t.step_pos >= g_calc_t.dec_point )
                    {
                        g_calc_t.ptr = g_calc_t.decel_tab;          /* �����ٶε��ٶȱ�ֵ��ptr */
                        g_motor_sta = STATE_DECEL;
                    }
                    break;
                case STATE_STOP:			//ֹͣ״̬
					TIM_ITConfig(TIM3,TIM_IT_CC2,DISABLE);		//�رն�ʱ��3��ͨ��2�ж�
                    g_motor_sta = STATE_IDLE;
					TIM_ITConfig(TIM3,TIM_IT_CC2,DISABLE);
					TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Disable);		//�ر����
                    break;
                case STATE_IDLE:			//����״̬
					TIM_ITConfig(TIM3,TIM_IT_CC2,DISABLE);
//					TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Disable);		//�ر����
                    break;
            }
			
			if(g_calc_t.step_pos >= g_calc_t.step )
			{
				myfree(SRAMIN,g_calc_t.decel_tab);          /* �˶���Ҫ�ͷ��ڴ� */
				myfree(SRAMIN,g_calc_t.accel_tab);          /* �ͷż��ٶ��ٶȱ� */
				g_motor_sta = STATE_STOP;
			}
        }
		
		/*  ���ñȽ�ֵ */
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);	// �����ʱ���ж�	
		Tim_Count = TIM_GetCounter(TIM3);			//��ȡ��ʱ��2�ļ���ֵ		//��ȡ����ֵ
        tmp = 0xFFFF & (Tim_Count + g_toggle_pulse);
		TIM_SetCompare2(TIM3,tmp);
	
    } 
	
}

/**
  * @brief  ��ʱ���Ƚ��ж�
  * @param  htim����ʱ�����ָ��
  * @note   ��
  * @retval ��
*/
void TIM3_IRQHandler(void)
{
	//T
	speed_decision();
	
	//	S
	speed_decision_s();
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC1)== SET)
//	{	
//		
//		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);	// �����ʱ���ж�	
//	}

}



