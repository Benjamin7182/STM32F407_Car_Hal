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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ????:??????????
static void OLED_Show_Signed(uint8_t x, uint8_t y, int16_t val)
{
    if (val < 0) {
        OLED_ShowChar(x, y, '-', 12, 1);
        val = -val;
    } else {
        OLED_ShowChar(x, y, '+', 12, 1);
    }
    OLED_ShowNumber(x + 8, y, (uint32_t)val, 5, 12);
}
void Test_MPU6050_OLED(void *pvParameters)
{

    /* ?? MPU6050 ???? (?? WHO_AM_I ?????? 0x68) */
    if (MPU6050_initialize() == 0) {
        OLED_ShowString(0, 0, (u8*)"MPU6050: OK");
			 OLED_Refresh_Gram(); 
    } else {
        OLED_ShowString(0, 0, (u8*)"MPU6050: FAIL");
        OLED_Refresh_Gram();
        while(1); // ????????????
    }
    while (1)
    {
        /* ?????? */
        MPU_Get_Accelscope(); // ??????????? accel[3]
        MPU_Get_Gyroscope();  // ?????????? gyro[3]
        /* ????????? (X, Y, Z) */
        OLED_ShowString(0, 16, (u8*)"AX:");
        OLED_Show_Signed(24, 16, accel[0]);
        OLED_ShowString(64, 16, (u8*)"AY:");
        OLED_Show_Signed(88, 16, accel[1]);
        
        OLED_ShowString(0, 32, (u8*)"AZ:");
        OLED_Show_Signed(24, 32, accel[2]);
        /* ????????? (GX, GY, GZ) */
        OLED_ShowString(0, 48, (u8*)"GZ:");
        OLED_Show_Signed(24, 48, gyro[2]);
        OLED_Refresh_Gram(); // ????
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* PS2 手柄数据显示任务 */
void Test_PS2_OLED(void *pvParameters)
{
    while (1)
    {
        /* 显示红灯模式连接状态 */
        if (PS2_RedLight() == 0) {
            OLED_ShowString(0, 0, (u8*)"PS2: RED OK   ");
        } else {
            OLED_ShowString(0, 0, (u8*)"PS2: SEARCHING");
        }

        /* 显示按键值 (1~16) */
        OLED_ShowString(0, 16, (u8*)"KEY:");
        OLED_ShowNumber(36, 16, (uint32_t)PS2_KEY, 2, 12);

        /* 显示左摇杆 LX, LY (0~255) */
        OLED_ShowString(0, 32, (u8*)"LX:");
        OLED_ShowNumber(24, 32, (uint32_t)PS2_LX, 3, 12);
        OLED_ShowString(64, 32, (u8*)"LY:");
        OLED_ShowNumber(88, 32, (uint32_t)PS2_LY, 3, 12);

        /* 显示右摇杆 RX, RY (0~255) */
        OLED_ShowString(0, 48, (u8*)"RX:");
        OLED_ShowNumber(24, 48, (uint32_t)PS2_RX, 3, 12);
        OLED_ShowString(64, 48, (u8*)"RY:");
        OLED_ShowNumber(88, 48, (uint32_t)PS2_RY, 3, 12);

        OLED_Refresh_Gram();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


/* 编码器测试与显示任务 */
void Test_Encoder_OLED(void *pvParameters)
{
    int32_t total_a = 0;
    int32_t total_b = 0;
    
    while (1)
    {
        /* 读取周期脉冲数 (50ms 周期) */
        int pulse_a = Read_Encoder(2);   // 左轮 Encoder A (TIM2)
        int pulse_b = -Read_Encoder(3);  // 右轮 Encoder B (TIM3 镜像取反)
        
        total_a += pulse_a;
        total_b += pulse_b;
        
        OLED_ShowString(0, 0, (u8*)"ENCODER TEST");
        
        /* 瞬时脉冲增量 */
        OLED_ShowString(0, 16, (u8*)"L:");
        OLED_Show_Signed(16, 16, pulse_a);
        OLED_ShowString(68, 16, (u8*)"R:");
        OLED_Show_Signed(84, 16, pulse_b);
        
        /* 累计脉冲位置 */
        OLED_ShowString(0, 34, (u8*)"POS_L:");
        OLED_Show_Signed(48, 34, total_a);
        
        OLED_ShowString(0, 48, (u8*)"POS_R:");
        OLED_Show_Signed(48, 48, total_b);
        
        OLED_Refresh_Gram();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
void sys_init(){
	DWT_Delay_Init();
	I2C_GPIOInit();
	OLED_Init();
	OLED_Clear();
	Motor_Init();
	Set_Pwm(0, 0, 0, 0, 0);

	/* 初始化 PS2 手柄接收器并配置为红灯模拟模式 */
	PS2_SetInit();
	Encoder_Init();


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
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	sys_init();

	xTaskCreate(vKeyTask, "vKeyTask", 128, NULL, 2, NULL);
	xTaskCreate(pstwo_task, "pstwo_task", 128, NULL, 3, NULL);
	xTaskCreate(Test_Encoder_OLED, "Test_Encoder_OLED", 256, NULL, 2, NULL);

	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
