/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encode.h"
#include "oled.h"
#include "serial.h"
#include "pid.h"
#include "Delay.h"
#include "Motor.h"
#include "sys.h"
#include "kalman_filter.h"
#include "app_config.h"
#include "app_sensor.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 速度环停车死区 (cm/s): |目标速度| 小于它即锁轮制动并清 PID */
#define SPEED_STOP_EPS   0.5f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float lv_target = 0.0f;    /* 左轮目标速度 (cm/s), 当前无蓝牙命令设定, 恒为 0 */
static float rv_target = 0.0f;    /* 右轮目标速度 (cm/s), 蓝牙 "SV:xx.xx##" 设定 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 行4(y=48) 显示速度环 Kp/Ki/Kd
 * 数据源必须是 pid_speed_L: 蓝牙 KPA/KIA/KDA 只写 pid_speed_L,
 * 显示与命令写入必须是同一实例, 否则行4 恒为开机初值不更新 */
static void OLED_ShowPidRow(void)
{
    OLED_ClearArea(0, 48, 128, 12);
    OLED_ShowFloatNum(0,  48, pid_speed_L.Kp, 3, 2, OLED_6X8);
    OLED_ShowFloatNum(44, 48, pid_speed_L.Ki, 3, 2, OLED_6X8);
    OLED_ShowFloatNum(88, 48, pid_speed_L.Kd, 3, 2, OLED_6X8);
}

/* 将带符号 PID 输出转换为电机方向 + 限幅后的 PWM */
static void Motor_DriveSpeed(char wheel, float out)
{
    int8_t dir;
    float  mag;

    if (out > 0.0f) {
        dir = DIR_FWD;
        mag = out;
    } else if (out < 0.0f) {
        dir = DIR_BWD;
        mag = -out;
    } else {
        dir = DIR_STOP;
        mag = 0.0f;
    }

    if (mag > (float)PWM_MAX) mag = (float)PWM_MAX;
    Motor_Direction(wheel, dir);
    Motor_PWM(wheel, (int16_t)mag);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 禁用 JTAG(保留SWD), 释放 PB3/PA15/PB4 给 OLED 软件SPI */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  Delay_Init();
  OLED_Init();

  /* 开机显示 wait... 直到初始化完成 */
  OLED_Clear();
  OLED_ShowString(46, 28, "wait...", OLED_6X8);
  OLED_Update();

  Encoder_Init();    /* 启动 TIM3/TIM4 编码器接口 */

  /* MPU6050 初始化(含 I2C/唤醒/配置/ID 检测) */
  if (MPU_Init() != 0)
  {
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 Error", OLED_6X8);
    OLED_Update();
    while (1);
  }

  /* 卡尔曼滤波器(含 DWT 周期计数器使能) */
  Kalman_Filter_GlobalInit();
  Kalman_Filter_GlobalSetParams(Q_ANGLE, Q_GYRO, R_ANGLE);

  /* HMC5883L 磁力计(共用 PB8/PB9 I2C) */
  HMC5883L_Init();

  /* 上电校准: 采集陀螺零偏, 记录当前姿态为直立目标 */
  Sensor_Calibrate();

  /* 主界面: 行0 蓝牙设定速度 / 行1 R / 行2 VL / 行3 VR / 行4 KpKiKd */
  OLED_Clear();
  OLED_ShowString(0, 12, "R:",  OLED_6X8);
  OLED_ShowString(0, 24, "VL:", OLED_6X8);
  OLED_ShowString(0, 36, "VR:", OLED_6X8);
  OLED_ShowString(90, 24, "cm/s", OLED_6X8);
  OLED_ShowString(90, 36, "cm/s", OLED_6X8);
  OLED_ShowFloatNum(0, 0, lv_target, 3, 2, OLED_6X8);
  OLED_Update();
  Serial_Init();   /* USART3 蓝牙: 中断接收 + printf 重定向 */
	Motor_Init();
	/* 左右轮速度环 PID: 当前先使用同一组参数
	 * 蓝牙 KPA/KIA/KDA/SV 只在线修改左轮 pid_speed_L, 行4 显示的也是该实例 */
	PID_Init(&pid_speed_L, 125, 215, 0.2);
	PID_Init(&pid_speed_R, 125, 200, 0.28);
	OLED_ShowPidRow();   /* 开机画出 行4 Kp/Ki/Kd */
	OLED_Update();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t report_tick = 0;
    static uint32_t pid_tick = 0;

    /* ---- 左右轮速度环 PI: 5ms 一拍，反馈低通/抗饱和由 PID_ComputeInc 完成 ---- */
    {
        uint32_t now = HAL_GetTick();

        if (now - pid_tick >= 5U) {
            float v_l, v_r, out_l, out_r, dt_ms;

            dt_ms = (float)(now - pid_tick);
            pid_tick = now;

            /* 同一时刻各读一次编码器，左右轮 PID 使用同一控制周期 */
            v_l = Encoder_GetSpeedL();
            v_r = Encoder_GetSpeedR();

            PID_SetSetpoint(&pid_speed_L, lv_target);
            PID_SetSetpoint(&pid_speed_R, rv_target);

            if (lv_target > -SPEED_STOP_EPS && lv_target < SPEED_STOP_EPS) {
                PID_Reset(&pid_speed_L);
                Motor_Direction('L', DIR_STOP);
                Motor_PWM('L', 0);
            } else {
                out_l = PID_ComputeInc(&pid_speed_L, v_l, (float)PWM_MAX, dt_ms);
                Motor_DriveSpeed('L', out_l);
            }

            if (rv_target > -SPEED_STOP_EPS && rv_target < SPEED_STOP_EPS) {
                PID_Reset(&pid_speed_R);
                Motor_Direction('R', DIR_STOP);
                Motor_PWM('R', 0);
            } else {
                out_r = PID_ComputeInc(&pid_speed_R, v_r, (float)PWM_MAX, dt_ms);
                Motor_DriveSpeed('R', out_r);
            }
        }
    }

    /* ---- 蓝牙命令处理 ---- */
    {
        float kp, ki, kd, sv;

        /* "KPA:xx##" -> 左轮 Kp */
        if (KPA_Get(&kp)) {
            PID_SetTunings(&pid_speed_L, kp, pid_speed_L.Ki, pid_speed_L.Kd);
            OLED_ShowPidRow();
            OLED_Update();
        }

        /* "KIA:xx##" -> 左轮 Ki */
        if (KIA_Get(&ki)) {
            PID_SetTunings(&pid_speed_L, pid_speed_L.Kp, ki, pid_speed_L.Kd);
            OLED_ShowPidRow();
            OLED_Update();
        }

        /* "KDA:xx##" -> 左轮 Kd */
        if (KDA_Get(&kd)) {
            PID_SetTunings(&pid_speed_L, pid_speed_L.Kp, pid_speed_L.Ki, kd);
            OLED_ShowPidRow();
            OLED_Update();
        }

        /* "SV:xx.xx##" -> 左轮目标速度(cm/s), 下一控制拍生效 */
        if (SV_Get(&sv)) {
            lv_target = sv;
            OLED_ClearArea(0, 0, 128, 12);
            OLED_ShowFloatNum(0, 0, lv_target, 3, 2, OLED_6X8);
            OLED_Update();
        }

    }

    /* ---- 每 100ms: 蓝牙遥测上报 + OLED 速度刷新 ---- */
    if (HAL_GetTick() - report_tick >= 100) {
        float v_l, v_r;
        report_tick = HAL_GetTick();
        v_l = Encoder_GetSpeedL();
        v_r = Encoder_GetSpeedR();

        /* 遥测格式 "Y,R,VL,VR" (各1位小数) */
        {
            char tel[48];
            snprintf(tel, sizeof(tel), "%.1f,%.1f,%.1f,%.1f\r\n",
                     yaw, roll, v_l, v_r);
            USART_SendString(tel);
        }

        /* 刷新 VL(行2) / VR(行3) */
        OLED_ClearArea(24, 24, 60, 12);
        OLED_ClearArea(24, 36, 60, 12);
        OLED_ShowFloatNum(24, 24, v_l, 3, 1, OLED_6X8);
        OLED_ShowFloatNum(24, 36, v_r, 3, 1, OLED_6X8);

        OLED_Update();
    }

    /* ---- 姿态显示: pitch/roll/yaw 由 PA12 中断更新, 此处只画 R 横滚角 ---- */
    if (Display_NeedUpdate(pitch, roll, yaw))
    {
      OLED_ClearArea(24, 12, 104, 12);
      OLED_ShowFloatNum(24, 12, Display_GetR(), 3, 1, OLED_6X8);
      OLED_Update();
    }

    Delay_ms(5);   /* 主循环节流 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 外部中断回调已移至 Hardware/Src/Interrupt.c (PA12 -> Sensor_Angle_Update) */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
