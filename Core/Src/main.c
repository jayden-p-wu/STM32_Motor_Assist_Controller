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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fardriver.h"
#include "fardriver_sif.h"
#include "vehicle_can.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Serial debug console: lets you set throttle/regen over UART without
 * needing live vehicle CAN traffic. Set to 0 for a normal build. */
#define DEBUG_SERIAL_CONTROL 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* CAN bus timeout: if no vehicle message in 500 ms, cut motor output */
#define VEH_CAN_TIMEOUT_MS  500U
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if DEBUG_SERIAL_CONTROL
/* Commands (newline terminated): "T<0-100>" set throttle %, "R0"/"R1"
 * regen off/on, "N" release override and resume normal CAN control, "?"
 * print current state. Overrides the safety cutout while active. */
static uint8_t dbg_override_active = 0U;
static float   dbg_throttle_pct = 0.0f;
static uint8_t dbg_regen = 0U;

static void Debug_HandleLine(char *line)
{
  size_t len = strlen(line);
  if (len == 0U)
  {
    return;
  }

  if ((line[0] == 'T' || line[0] == 't') && len > 1U)
  {
    float val = strtof(&line[1], NULL);
    if (val < 0.0f) val = 0.0f;
    if (val > 100.0f) val = 100.0f;
    dbg_throttle_pct = val;
    dbg_override_active = 1U;
    printf("[DBG] throttle=%.1f%%\r\n", (double)dbg_throttle_pct);
  }
  else if ((line[0] == 'R' || line[0] == 'r') && len > 1U)
  {
    dbg_regen = (line[1] != '0') ? 1U : 0U;
    dbg_override_active = 1U;
    printf("[DBG] regen=%u\r\n", (unsigned)dbg_regen);
  }
  else if (line[0] == 'N' || line[0] == 'n')
  {
    dbg_override_active = 0U;
    dbg_throttle_pct = 0.0f;
    dbg_regen = 0U;
    printf("[DBG] override released, resuming CAN control\r\n");
  }
  else if (line[0] == '?')
  {
    printf("[DBG] override=%u throttle=%.1f%% regen=%u\r\n",
           (unsigned)dbg_override_active, (double)dbg_throttle_pct, (unsigned)dbg_regen);
  }
  else
  {
    printf("[DBG] unknown cmd '%s' (use T<0-100>, R0/R1, N, ?)\r\n", line);
  }
}

/* RX is interrupt-driven (USART2 has no hardware FIFO, so polling once per
 * ~10-20 ms loop iteration can miss/overwrite bytes). The ISR just stashes
 * each byte in a ring buffer and re-arms itself; Debug_PollUART() drains
 * the buffer and assembles/dispatches lines from the main loop. */
#define DBG_RXBUF_SIZE 64U
static volatile uint8_t  dbg_rxbuf[DBG_RXBUF_SIZE];
static volatile uint16_t dbg_rxbuf_head = 0U;
static volatile uint16_t dbg_rxbuf_tail = 0U;
static uint8_t           dbg_rx_byte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint16_t next = (uint16_t)((dbg_rxbuf_head + 1U) % DBG_RXBUF_SIZE);
    if (next != dbg_rxbuf_tail)  /* drop byte if buffer is full */
    {
      dbg_rxbuf[dbg_rxbuf_head] = dbg_rx_byte;
      dbg_rxbuf_head = next;
    }
    HAL_UART_Receive_IT(&huart2, &dbg_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
    HAL_UART_Receive_IT(&huart2, &dbg_rx_byte, 1U);
  }
}

/* Drains the ISR ring buffer and assembles/dispatches lines; call once per
 * main loop iteration. */
static void Debug_PollUART(void)
{
  static char    line[32];
  static uint8_t idx = 0U;

  while (dbg_rxbuf_tail != dbg_rxbuf_head)
  {
    uint8_t rx_byte = dbg_rxbuf[dbg_rxbuf_tail];
    dbg_rxbuf_tail = (uint16_t)((dbg_rxbuf_tail + 1U) % DBG_RXBUF_SIZE);

    if (rx_byte == '\r' || rx_byte == '\n')
    {
      if (idx > 0U)
      {
        line[idx] = '\0';
        Debug_HandleLine(line);
        idx = 0U;
      }
    }
    else if (idx < sizeof(line) - 1U)
    {
      line[idx++] = (char)rx_byte;
    }
  }
}
#endif /* DEBUG_SERIAL_CONTROL */

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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  Fardriver_Init();
  VehicleCAN_Init();
  /* FardriverSIF_Init(); */  /* start TIM2 input capture — disabled until SIF feedback is needed */

  Fardriver_SetKey(1);        /* power on the Fardriver controller */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); /* green on */

#if DEBUG_SERIAL_CONTROL
  HAL_UART_Receive_IT(&huart2, &dbg_rx_byte, 1U);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if DEBUG_SERIAL_CONTROL
    /* Non-blocking; keep polling even when CAN is dead so bench testing
     * doesn't need a live vehicle bus. */
    Debug_PollUART();
#endif

    /* --- Safety: if CAN is lost, immediately cut throttle and regen ---
     * Bypassed while a serial debug override is active. */
    if (!VehicleCAN_IsAlive(VEH_CAN_TIMEOUT_MS)
#if DEBUG_SERIAL_CONTROL
        && !dbg_override_active
#endif
       )
    {
      Fardriver_SetThrottle(0.0f);
      Fardriver_SetRegenBrake(0);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      HAL_Delay(10);
      continue;
    }

    /* --- Throttle control ---
     * Only drive motor when transmission is in Drive (gear=3).
     * Cut throttle in Park, Neutral, Reverse
     * Also cut in Sport, or Low for now
     */
    float pedal_pct = 0.0f;
    float throttle_pct = 0.0f;
#if DEBUG_SERIAL_CONTROL
    if (dbg_override_active)
    {
      throttle_pct = dbg_throttle_pct;
    }
    else
#endif
    {
      if (g_vehicle.gear == PRND_DRIVE || g_vehicle.gear == PRND_PARK /* && VEHICLE_MOVING() */)
      {
        pedal_pct = g_vehicle.accel_pedal_pct;
        if (pedal_pct > 100.0f) pedal_pct = 100.0f;
      }
      throttle_pct = pedal_pct * 10; // 0-33% pedal -> 0-100% throttle
      if (throttle_pct > 100.0f) throttle_pct = 100.0f;
    }

    /* --- Regen braking ---
     * Engage when vehicle moving forward, gas pedal released, brake pedal pressed, and in drive
     */
    uint8_t regen;
#if DEBUG_SERIAL_CONTROL
    if (dbg_override_active)
    {
      regen = dbg_regen;
    }
    else
#endif
    {
      regen = /* g_vehicle.gear == PRND_DRIVE && VEHICLE_MOVING() */ 1 /* && g_vehicle.brake_applied == 1U */ && (pedal_pct < 1.0f) ? 1U : 0U;
    }

    /* Output to Fardriver */
    float pwm_val = Fardriver_SetThrottle(throttle_pct);
    Fardriver_SetRegenBrake(regen);

    /* LED: solid = throttling, blink = regen, off = idle */
    if (pedal_pct > 1.0f) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    } else if (regen) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                        (HAL_GetTick() & 256U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }

    /* Decoded CAN summary every 500 ms */
    static uint32_t s_print_ts = 0U;
    if (HAL_GetTick() - s_print_ts >= 500U) {
      s_print_ts = HAL_GetTick();
      static const char *gear_str[] = {"Park","Reverse","Neutral","Drive","Sport","Low",
                                        "?","?","?","?","?","?","?","?","?","Unknown"};
      printf("Gear=%s  Pedal=%.1f%%  Brake=%u  Spd=%.1fkph  EngRPM=%.0f  PWM=%.2fV  MotThrottle=%.1f%%  Regen=%u\r\n",
             gear_str[g_vehicle.gear & 0xFU],
             (double)g_vehicle.accel_pedal_pct,
             (unsigned)g_vehicle.brake_applied,
             (double)g_vehicle.vehicle_speed_kph,
             (double)g_vehicle.engine_speed_rpm,
             ((double)pwm_val / 999.0) * 3.3,
             (double)throttle_pct,
             (unsigned)regen);
    }

    HAL_Delay(10);  /* 100 Hz control loop */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, REGEN_BRAKE_Pin|FARDRIVER_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : REGEN_BRAKE_Pin */
  GPIO_InitStruct.Pin = REGEN_BRAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REGEN_BRAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FARDRIVER_KEY_Pin */
  GPIO_InitStruct.Pin = FARDRIVER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FARDRIVER_KEY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
