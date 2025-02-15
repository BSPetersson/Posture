/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"

#define UART_BUFFER_SIZE 256
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);  // Give time for the accelerometer to boot up
  haptic_feedback_controller_initialize();
  accelerometer_controller_initialize();
  led_controller_initialize();
  button_controller_initialize();
  state_machine_initialize();
  posture_controller_initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  led_execute_sequence(LED_SEQ_THREE_BLINKS);
  bool first_time_sleep = true;
  accel_data_t accel_data;

  char uart_buffer[UART_BUFFER_SIZE];
  int offset = 0;
  while (1)
  {
  //  state_machine_update();
    posture_controller_update();

    // if (int1_flag) {
    //   led_execute_sequence(LED_SEQ_DOUBLE_BLINK);
    //   int1_flag = false;
    // }

    // if (int2_flag) {
    //   led_execute_sequence(LED_SEQ_FADE_IN_OUT);
    //   int2_flag = false;
    // }

    // // HAL_Delay(1000);

    // // Fetch data from all relevant registers
    // uint8_t sysmod = get_sysmod();
    // uint8_t ff_mt_src = get_ff_mt_src();
    // uint8_t int_source = get_int_source();
    // uint8_t transient_src = get_transient_src();
    // accelerometer_read_mps2(&accel_data);

    // clear_accelerometer_interrupts();
    // sleep_controller_activate_sleep_mode();

    // led_execute_sequence(LED_SEQ_THREE_BLINKS);
    // led_execute_sequence(LED_SEQ_DOUBLE_BLINK);
    // led_execute_sequence(LED_SEQ_FADE_IN_OUT);
    // led_execute_sequence(LED_SEQ_THREE_BLINKS);

    // // Format SYSMOD
    // uart_buffer[0] = '\0';
    // offset = 0;
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "SYSMOD: 0x%02X\r\n", sysmod);
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Mode: %s\r\n", (sysmod & 0x03) == 0 ? "Standby" :
    //                                       (sysmod & 0x03) == 1 ? "Wake" : "Sleep");
    // HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // // Format FF_MT_SRC
    // offset = 0;
    // uart_buffer[0] = '\0';
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "FF_MT_SRC: 0x%02X\r\n", ff_mt_src);
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Motion/Freefall detected: %s\r\n", (ff_mt_src & 0x80) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - X motion: %s\r\n", (ff_mt_src & 0x02) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Y motion: %s\r\n", (ff_mt_src & 0x04) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Z motion: %s\r\n", (ff_mt_src & 0x08) ? "True" : "False");
    // HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // // Format INT_SOURCE
    // offset = 0;
    // uart_buffer[0] = '\0';
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "INT_SOURCE: 0x%02X\r\n", int_source);
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Data Ready: %s\r\n", (int_source & 0x01) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Motion/Freefall: %s\r\n", (int_source & 0x04) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Transient: %s\r\n", (int_source & 0x20) ? "True" : "False");
    // HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // // Format TRANSIENT_SRC
    // offset = 0;
    // uart_buffer[0] = '\0';
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "TRANSIENT_SRC: 0x%02X\r\n", transient_src);
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Transient detected: %s\r\n", (transient_src & 0x40) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - X transient: %s\r\n", (transient_src & 0x02) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Y transient: %s\r\n", (transient_src & 0x04) ? "True" : "False");
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                    "  - Z transient: %s\r\n", (transient_src & 0x08) ? "True" : "False");
    // HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // // Format Accelerometer Data
    // // Scale the float values by 100 to preserve two decimal places
    // int32_t x_scaled = (int32_t)(accel_data.x_mps2 * 100);
    // int32_t y_scaled = (int32_t)(accel_data.y_mps2 * 100);
    // int32_t z_scaled = (int32_t)(accel_data.z_mps2 * 100);

    // // Construct the UART message
    // offset = 0;
    // uart_buffer[0] = '\0';

    // // Header
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                     "Accelerometer Data:\r\n");

    // // X-axis
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                     "  - X: %ld.%02ld m/s^2\r\n",
    //                     x_scaled / 100,
    //                     (x_scaled < 0 ? -(x_scaled % 100) : (x_scaled % 100)));

    // // Y-axis
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                     "  - Y: %ld.%02ld m/s^2\r\n",
    //                     y_scaled / 100,
    //                     (y_scaled < 0 ? -(y_scaled % 100) : (y_scaled % 100)));

    // // Z-axis
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                     "  - Z: %ld.%02ld m/s^2\r\n",
    //                     z_scaled / 100,
    //                     (z_scaled < 0 ? -(z_scaled % 100) : (z_scaled % 100)));

    // // Delimiter
    // offset += snprintf(uart_buffer + offset, UART_BUFFER_SIZE - offset,
    //                     "---\r\n");
    
    // // Transmit the message
    // HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // HAL_Delay(500);

  // bool is_sleep = is_accelerometer_in_sleep_mode();

  // if (is_sleep && first_time_sleep) {
  //     led_execute_sequence(LED_SEQ_THREE_BLINKS);
  //     first_time_sleep = false;
  // }

  // bool is_motion = is_motion_detected();

  // if (is_motion) {
  //     led_on(100);
  // }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 1) Check if it's the Button line (PB5)
    if (GPIO_Pin == GPIO_PIN_5)
    {
        // Call the button driver's function
        button_handle_exti(GPIO_Pin);
    }

    // 2) Check if it's the Accelerometer lines
    // else if (GPIO_Pin == GPIO_PIN_0)
    // {
    //     // PA0 => accelerometer INT2

    //     accelerometer_handle_int2();
    // }
    // else if (GPIO_Pin == GPIO_PIN_1)
    // {
    //     // PA1 => accelerometer INT1
    //     accelerometer_handle_int1();
    // }

    // You can handle more EXTI pins if needed
}

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