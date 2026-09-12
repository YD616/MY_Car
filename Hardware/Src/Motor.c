/* 电机驱动: 方向由 GPIO 控制, 转速由 TIM1 PWM 控制 (10kHz) */
#include "motor.h"

void Motor_Direction(char position,int8_t direction)
{
	/*左侧电机*/
	if(position == 'L')
	{
	
	if(direction == DIR_FWD)
		{
            HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    }
	else if(direction == DIR_BWD)
		{
            HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    }
	else
		{
            /* 刹车: 两端拉低, 电机短接制动 */
            HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    }

	}
	/*右侧电机*/	
	else if(position == 'R')
  {
  
    if(direction == DIR_FWD)
    {
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
    }
    else if(direction == DIR_BWD)
    {
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);
    }
    else
    {
        /* 刹车: 两端拉低, 电机短接制动 */
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);
    }
	}
}

void Motor_PWM(char position,int16_t speed)
{
    if(position == 'L')
	{
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_L_PWM_CH, (uint32_t)speed);   /* 电机L: TIM1_CH1, PA8 */
    }
	else if(position == 'R')
	{
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_R_PWM_CH, (uint32_t)speed);   /* 电机R: TIM1_CH4, PA11 */
	}
}

/* 电机初始化: 启动 TIM1 PWM (高级定时器必须调 Start 才有输出) */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* 电机L: TIM1_CH1, PA8 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   /* 电机R: TIM1_CH4, PA11 */
	  Motor_Direction('L', DIR_STOP);
		Motor_Direction('R', DIR_STOP);
		Motor_PWM('L', 0);
		Motor_PWM('R', 0);
}

void Turn_Left(void)
{
	Motor_Direction('L',DIR_BWD);
	Motor_Direction('R',DIR_FWD);
}       

void Turn_Right(void)
{
	Motor_Direction('L',DIR_FWD);
	Motor_Direction('R',DIR_BWD);
}
