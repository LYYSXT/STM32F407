#include "stm32f4xx.h"                  // Device header
#include "straight_motor.h" 
//#include "straight_tim.h"  
#include <math.h>

#include "Motor_S.h"  
#include "sys.h"
#include "stdlib.h"
/*****************************************************ֱ�߲岹ʵ��*****************************************************/
inter_pol_def g_pol_par= {0};   /* ֱ�߲岹����ֵ */

//��ͷ�ļ�����
__IO st_motor_status_def  g_motor_sta_straight = STATE_STOP_straight;     /* ��������˶�״̬ */

//__IO st_motor_status_def  g_motor_sta = STATE_STOP;     /* ��������˶�״̬ */
__IO int32_t  sign_dir[2] = {1,1};                      /* ƫ��̵ļ��㹫ʽ����λ */

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
 * @brief       ֱ�������岹����ʵ��ֱ�߲岹����,������������ֱ���X���Y�Ჽ��IncX,IncY��
 * @param       IncX    ���յ�X������
 * @param       IncY    ���յ�Y������
 * @param       Speed   �������ٶ�
 * @retval      ��
 */
void line_incmove(uint32_t IncX,uint32_t IncY,uint32_t Speed)
{
    /* ƫ������� */
    g_pol_par.f_e = 0;

    /* ������㵽�յ������Ӧ��������λ��*/
    g_pol_par.end_x = IncX;
    g_pol_par.end_y = IncY;
    g_pol_par.end_pulse = g_pol_par.end_y + g_pol_par.end_x;

    /* �����յ��ж���ֱ���ϵĽ�������,����ƫ�� */
    if(g_pol_par.end_y > g_pol_par.end_x)
    {
        g_pol_par.act_axis = AXIS_Y;                    /* ��һ������Y�� */
        g_pol_par.f_e = g_pol_par.f_e + g_pol_par.end_x;
    }
    else
    {
        g_pol_par.act_axis = AXIS_X;                    /* ��һ������X�� */
        g_pol_par.f_e = g_pol_par.f_e - g_pol_par.end_y;
    }
    /* ����ͨ���ıȽ�ֵ */
	TIM_SetCompare1(TIM2, Speed);
	TIM_SetCompare2(TIM2, Speed);
	TIM_SetAutoreload(TIM2, Speed*2);
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_X].pulse_channel,Speed);   
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_Y].pulse_channel,Speed);
//    __HAL_TIM_SET_AUTORELOAD(&g_atimx_handle,Speed*2);  /* ARR����Ϊ�Ƚ�ֵ2������������Ĳ��ξ���50%��ռ�ձ� */

//	TIM_CCxCmd(TIM3,TIM_Channel_2,TIM_CCx_Enable);		//�����ͨ��		STATE_STOP_straight
	
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
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);			 			//ʹ�ܶ�ʱ��ͨ�� 
//	TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	
//    TIM_CCxChannelCmd(TIM8, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_ENABLE);    /* ʹ��ͨ����� */
//    HAL_TIM_Base_Start_IT(&g_atimx_handle);             /* ʹ�ܶ�ʱ���Լ����������ж� */
    g_motor_sta_straight = STATE_RUN_straight;                            /* ��ǵ�������˶� */
}
/**
 * @brief       ʵ����������ֱ�߲岹
 * @param       coordsX    ���յ�X������
 * @param       coordsY    ���յ�Y������
 * @param       Speed      �������ٶ�
 * @retval      ��
*/
void line_inpolation( int32_t coordsX,int32_t coordsY, int32_t Speed)
{
    if(g_motor_sta_straight != STATE_STOP_straight)   /* ��ǰ���������ת */
       return ;
    /* �������޵�ֱ�߸���һ������һ��,ֻ�ǵ���˶�����һ�� */
	GPIO_SetBits(GPIOA,GPIO_Pin_12);	//PA12Ϊʹ�����ţ�����ߵ�ƽ
	GPIO_SetBits(GPIOB,GPIO_Pin_5);	//PB5Ϊʹ�����ţ�����ߵ�ƽ
    g_pol_par.moving_mode = LINE;
    if(coordsX < 0)                 /* ��x��С��0ʱ�����������Ϊ����*/
    {
        g_pol_par.x_dir = CCW;
        coordsX = -coordsX;         /* ȡ����ֵ */
		
		GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ
//		GPIO_SetBits(GPIOD,GPIO_Pin_7);		//PD7Ϊ�������ţ�����ߵ�ƽ
//        ST_LINE_DIR(CCW,AXIS_X);
    }
    else
    {
        g_pol_par.x_dir = CW;
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ
//        ST_LINE_DIR(CW,AXIS_X);
    }
    if(coordsY < 0)                 /* ��y��С��0ʱ�����������Ϊ����*/
    {
        g_pol_par.y_dir = CCW;
        coordsY = -coordsY;         /* ȡ����ֵ */
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
 * @brief       ��ʱ���жϻص�����
 * @param       htim �� ��ʱ�����
 * @retval      ��
 */
void straight_speed_decision()
{
    __IO uint32_t axis = 0;         /* ������ */
    axis = g_pol_par.act_axis;      /* ��ǰ������ */

	if (TIM_GetITStatus(TIM2, TIM_IT_Update)== SET)
	{	  
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	// �����ʱ���ж�	


		/* �ж��Ƿ񵽴��յ���߻�û��ʼ�˶� */
		if(g_pol_par.end_pulse == 0)
			return;
		/* ���ݽ������� ��������ֵ */
	  
		if(g_pol_par.moving_mode == LINE)
		{
			if(g_pol_par.f_e > 0)                                   /* ƫ��� > 0 ,˵����ǰλ��λ��ֱ���Ϸ�,Ӧ��X����� */
			{
				g_pol_par.act_axis = AXIS_X;
				g_pol_par.f_e = g_pol_par.f_e - g_pol_par.end_y;    /* ��һ���޵�X�����ʱ,ƫ����� */
			}
			else if(g_pol_par.f_e < 0)                              /* ƫ��� < 0 ,˵����ǰλ��λ��ֱ���·�,Ӧ��Y����� */
			{
				g_pol_par.act_axis = AXIS_Y;
				g_pol_par.f_e = g_pol_par.f_e+g_pol_par.end_x;      /* ��һ���޵�Y�����ʱ,ƫ����� */
			}
			/* ƫ��Ϊ0��ʱ��,�ж�x,y���յ�Ĵ�С������������ */
			else if(g_pol_par.f_e == 0)                             /* ƫ��� = 0 ,˵����ǰλ��λ��ֱ��,Ӧ�ж��յ������ٽ��� */
			{
				if(g_pol_par.end_y > g_pol_par.end_x)               /* ��Y������Ļ���Ӧ��Y����� */
				{
					g_pol_par.act_axis = AXIS_Y;
					g_pol_par.f_e = g_pol_par.f_e+g_pol_par.end_x;  /* ��һ���޵�Y�����ʱ,ƫ����� */
				}
				else
				{
					g_pol_par.act_axis = AXIS_X;
					g_pol_par.f_e = g_pol_par.f_e - g_pol_par.end_y;
				}
			}
		}
		/* �ж��Ƿ���Ҫ���������� */
		if(axis != g_pol_par.act_axis)
		{
			
			
				//	������ܻ������
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
		/* �յ��б�:�ܲ��� */
		g_pol_par.end_pulse--;
		if(g_pol_par.end_pulse == 0)
		{
			g_motor_sta_straight = STATE_STOP_straight;                               /* �����յ� */
			//�ر�ʹ��
			GPIO_ResetBits(GPIOA,GPIO_Pin_12);	//PA12Ϊʹ�����ţ�����͵�ƽ
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);	//PB5Ϊʹ�����ţ�����͵�ƽ
			
			TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
			TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
//			TIM_IT_Update
			TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);	
//			TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);			 			//ʹ�ܶ�ʱ��ͨ�� 
//			TIM_ITConfig(TIM2,TIM_IT_CC2,DISABLE);
			
//			TIM_CCxChannelCmd(TIM8, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_DISABLE);/* �رյ�ǰ����� */
//			HAL_TIM_Base_Stop_IT(&g_atimx_handle);                  /* ֹͣ��ʱ�� */
		}
		
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	// �����ʱ���ж�	
		
		
	}
	
	
	
}
//TIM_IT_Update





//Բ���岹


/**
 * @brief       ����Բ���岹����
 * @param       coordsX,coordsY:x,y����,dir: Բ���Ĳ岹����
 * @retval      ��
 */
void setarcdir(int32_t coords_x,int32_t coords_y,int8_t dir)
 {
    g_pol_par.inter_dir = dir;
    if(g_pol_par.inter_dir == CCW)		/*��ʱ��*/ 
    {
        if(coords_x > 0)                /* ����x > 0 �����X���������,˳ʱ��Y����������Ǹ��� */
        {
            if(coords_y >= 0)           /* ���y > 0,���ǵ�һ���� */
            {
                g_pol_par.qua_points = FIRST_QUADRANT;		//��һ����
                g_pol_par.x_dir = CCW;			 /* X�᷽�� 		��ʱ��*/
                g_pol_par.y_dir = CW;			 /* Y�᷽��		˳ʱ�� */
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
            else										//��������
            {
                g_pol_par.qua_points = FOURTH_QUADRANT;
                g_pol_par.x_dir = CW;   /* y <= 0, �ڵ������� */
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
        else if(coords_x < 0)           /* X��ĸ����� */
        {
            if(coords_y <= 0)           /* y < 0,���ǵ������� */
            {
                g_pol_par.qua_points = THIRD_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = -1;
//				GPIO_ResetBits
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
            else                        /* y >= 0,�ڶ����� */
            {
                g_pol_par.qua_points = SECOND_QUADRANT;
                g_pol_par.x_dir = CCW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
        }
        else if(coords_x == 0)          /* x = 0,��Y���ϵ�������� */
        {
            if(coords_y > 0)
            {
                g_pol_par.qua_points = SECOND_QUADRANT;
                g_pol_par.x_dir = CCW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
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
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
    }
    else /* CW����		˳ʱ�� */
    {
        if(coords_x > 0)                /* ����x > 0 �����X���������,˳ʱ��Y����������Ǹ��� */
        {
            if(coords_y >0)             /* ���y > 0,���ǵ�һ���� */
            {
                g_pol_par.qua_points = FIRST_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
            else
            {
                g_pol_par.qua_points = FOURTH_QUADRANT;
                g_pol_par.x_dir = CCW;  /* y <= 0, �ڵ������� */
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CCW,AXIS_Y);
            }
        }
        else if(coords_x < 0)           /* X��ĸ����� */
        {
            if(coords_y < 0)            /* y < 0,���ǵ������� */
            {
                g_pol_par.qua_points = THIRD_QUADRANT;
                g_pol_par.x_dir = CCW;
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = -1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
            else                        /* y >= 0,�ڶ����� */
            {
                g_pol_par.qua_points = SECOND_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = 1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
        else if(coords_x == 0)          /* x = 0,��Y���ϵ�������� */
        {
            if(coords_y > 0)
            {
                g_pol_par.qua_points = FIRST_QUADRANT;
                g_pol_par.x_dir = CW;
                g_pol_par.y_dir = CCW;
                sign_dir[AXIS_X] = 1;
                sign_dir[AXIS_Y] = -1;
				
				GPIO_ResetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_SetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
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
				
				GPIO_SetBits(GPIOB,GPIO_Pin_11);	//PB11Ϊ�������ţ�����ߵ�ƽ	X	��ת
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);		//Y		��ת
//                ST_LINE_DIR(CCW,AXIS_X);
//                ST_LINE_DIR(CW,AXIS_Y);
            }
        }
    }
}


/**
 * @brief       ��XOYƽ���ڻ�����Բ��
 * @param       start_x_point,start_y_point:  �ֱ����������X,Y
 * @param       stop_x_point,stop_y_point:  �ֱ����յ�����X,Y
 * @param       speed:  �ٶ�ֵ
 * @param       dir:    Բ���ķ���
 * @retval      ��
 */
void arc_incmove(int32_t start_x_point,int32_t start_y_point,int32_t stop_x_point,int32_t stop_y_point,uint32_t speed,int8_t dir)
{
    if(g_motor_sta_straight == STATE_RUN_straight)                            /*  ��ǰ��������˶� */
        return ;

	//STATE_RUN_straight
	
    /* ������Բ�����귽�� */
    if( (pow(start_x_point,2)+pow(start_y_point,2)) != (pow(stop_x_point,2)+pow(stop_y_point,2)))   /* ��Ҫ����뾶һ�²���һ��Բ */
        return ;
	GPIO_SetBits(GPIOA,GPIO_Pin_12);	//PA12Ϊʹ�����ţ�����ߵ�ƽ
	GPIO_SetBits(GPIOB,GPIO_Pin_5);		//PB5Ϊʹ�����ţ�����ߵ�ƽ
	
    g_pol_par.moving_mode = ARC;                            /* Բ����־ */
    
    g_pol_par.f_e = 0;                                      /* ƫ������� */
    g_pol_par.start_point[AXIS_X] = start_x_point;
    g_pol_par.start_point[AXIS_Y] = start_y_point;
    g_pol_par.end_x = stop_x_point;
    g_pol_par.end_y = stop_y_point;
    /* ���õ������ʱ��Բ���켣���˶����� */
    setarcdir(g_pol_par.start_point[AXIS_X],g_pol_par.start_point[AXIS_Y],dir);                     /* ����Բ���岹���� */
    /* �����ܵĲ��� */  
    g_pol_par.end_pulse = abs((stop_y_point-start_y_point))+ abs((stop_x_point-start_x_point));     /* ����㵽�յ�������� */
    /* �������x = 0,˵�����λ��y����,��ʱ��X������������ */
    if(g_pol_par.start_point[AXIS_X] == 0)
    {
        g_pol_par.act_axis = AXIS_X;                        /* ��һ����X�� */
        /* ����Բ�����������X�������ƫ��� */
        g_pol_par.f_e = g_pol_par.f_e + sign_dir[AXIS_X]*g_pol_par.start_point[AXIS_X]*2 + 1;       /* ƫ��̵ļ��� */
    }
    else
    {
        g_pol_par.act_axis = AXIS_Y;                        /* ��һ����Y�� */
        g_pol_par.f_e = g_pol_par.f_e + sign_dir[AXIS_Y]*g_pol_par.start_point[AXIS_Y]*2 + 1;       /* ƫ��̵ļ��� */
    }
    
//    ST_LINE_EN(EN,AXIS_X);
//    ST_LINE_EN(EN,AXIS_Y);
	
	TIM_SetCompare1(TIM2, speed);
	TIM_SetCompare2(TIM2, speed);
	TIM_SetAutoreload(TIM2, speed*2);
	
	
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_X].pulse_channel,speed);                    /* ����ͨ���ıȽ�ֵ */
//    __HAL_TIM_SET_COMPARE(&g_atimx_handle,st_motor[AXIS_Y].pulse_channel,speed);
//    __HAL_TIM_SET_AUTORELOAD(&g_atimx_handle,speed*2);      /* ARR����Ϊ�Ƚ�ֵ2������������Ĳ��ξ���50%��ռ�ձ� */

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
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);			 			//ʹ�ܶ�ʱ��ͨ�� 
	TIM_Cmd(TIM2, ENABLE);
	
//    HAL_TIM_Base_Start_IT(&g_atimx_handle);                 /* ʹ�ܶ�ʱ���Լ����������ж� */
	
	
    g_motor_sta_straight = STATE_RUN_straight;                                /* ��ǵ�������˶� */
}



/**
 * @brief       ��ʱ���жϻص�����
 * @param       htim �� ��ʱ�����
 * @retval      ��
 */
void circle_speed_decision()
{
    __IO uint32_t axis = 0;         /* ������ */
    axis = g_pol_par.act_axis;      /* ��ǰ������ */

	if (TIM_GetITStatus(TIM2, TIM_IT_Update)== SET)
	{	  
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	// �����ʱ���ж�	

		/* �ж��Ƿ񵽴��յ���߻�û��ʼ�˶� */
		if(g_pol_par.end_pulse == 0)
			return;
		/* �����µĶ������� */
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
		
		/* ���ݽ������� ��������ֵ */
		if(g_pol_par.moving_mode == ARC)
		{
			/* ������һ�ε�ƫ���ж���һ����������,ͬʱ������һ�ε�ƫ�� */
			if(g_pol_par.inter_dir == CCW)          /* �岹����:��ʱ��Բ�� */
			{
				if(g_pol_par.f_e < 0)               /* ƫ��� < 0 ,˵����ǰλ��λ��Բ���ڲ�,Ӧ��Բ����� */
				{
					if( (g_pol_par.qua_points == SECOND_QUADRANT) || (g_pol_par.qua_points == FOURTH_QUADRANT) )/*  �ڶ��͵�������,��ƫ��<0ʱ������X����� */
					{
						g_pol_par.act_axis = AXIS_X;
					}
					else                            /* �ڵ�һ�͵�������,ƫ��<0,������Y����� */
					{
						g_pol_par.act_axis = AXIS_Y;
					}
				}
				else if(g_pol_par.f_e >= 0)         /* ƫ��� >= 0 ,˵����ǰλ��λ��Բ�����,Ӧ��Բ�ڽ��� */
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
				if(g_pol_par.f_e < 0)               /* ƫ��� < 0 ,˵����ǰλ��λ��Բ���ڲ�,Ӧ��Բ����� */
				{
					if( (g_pol_par.qua_points == FIRST_QUADRANT) || (g_pol_par.qua_points == THIRD_QUADRANT) )/*  ��һ�͵�������,��ƫ��<0ʱ������X����� */
					{
						g_pol_par.act_axis = AXIS_X;
					}
					else                            /* �ڵڶ��͵�������,ƫ��<0,������Y����� */
					{
						g_pol_par.act_axis = AXIS_Y;
					}
				}
				else if(g_pol_par.f_e >= 0)         /* ƫ��� >= 0 ,˵����ǰλ��λ��Բ�����,Ӧ��Բ�ڽ��� */
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
			/* ���㵱ǰ������Ŀ�����ߵ�ƫ�� */
			g_pol_par.f_e = g_pol_par.f_e + 2*sign_dir[g_pol_par.act_axis]*g_pol_par.start_point[g_pol_par.act_axis] + 1;   /* ƫ��̵ļ��� */
		}   
		
		/* �ж��Ƿ���Ҫ���������� */
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
		/* �յ��б�:�ܲ��� */
		g_pol_par.end_pulse--;
		if(g_pol_par.end_pulse == 0)
		{
			g_motor_sta_straight = STATE_STOP_straight;               /*  �����յ� */
			
			//�ر�ʹ��
			GPIO_ResetBits(GPIOA,GPIO_Pin_12);	//PA12Ϊʹ�����ţ�����͵�ƽ
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);	//PB5Ϊʹ�����ţ�����͵�ƽ
			TIM_CCxCmd(TIM2,TIM_Channel_1,TIM_CCx_Disable);
			TIM_CCxCmd(TIM2,TIM_Channel_2,TIM_CCx_Disable);
			TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
			
			
//			TIM_CCxChannelCmd(ATIM_TIMX_PWM, st_motor[g_pol_par.act_axis].pulse_channel, TIM_CCx_DISABLE);  /* �رյ�ǰ����� */
//			HAL_TIM_Base_Stop_IT(&g_atimx_handle);  /*  ֹͣ��ʱ�� */
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

