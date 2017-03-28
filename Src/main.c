/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
#define AXIS_CNT            5 // 1..5
#define OUT_STEP_MULT       10 // 1..100
#define INP_MAX_PERIOD      3000 // us


#define CNT_TIM             &htim2 // timer to calculate periods
#define OUT_TIM             &htim1 // output timer


volatile uint32_t uwc = 0; // number of CNT_TIM's updates

volatile uint32_t aSteps[AXIS_CNT] = {0};
volatile uint32_t aAxisTimPresc[AXIS_CNT] = {0};
volatile uint32_t aAxisTimPrescPrev[AXIS_CNT] = {0};
volatile uint64_t aStepInPeriod[AXIS_CNT] = {0};
volatile uint64_t aStepInTime[AXIS_CNT] = {0};

uint32_t aOC_DMA_val[OUT_STEP_MULT*2] = {0};

struct PORT_PIN_t
{
  GPIO_TypeDef*   PORT;
  uint16_t        PIN;
};
const struct PORT_PIN_t aAxisStepInputs[] = {
  {AXIS1_STEP_INPUT_GPIO_Port, AXIS1_STEP_INPUT_Pin},
  {AXIS2_STEP_INPUT_GPIO_Port, AXIS2_STEP_INPUT_Pin},
  {AXIS3_STEP_INPUT_GPIO_Port, AXIS3_STEP_INPUT_Pin},
  {AXIS4_STEP_INPUT_GPIO_Port, AXIS4_STEP_INPUT_Pin},
  {AXIS5_STEP_INPUT_GPIO_Port, AXIS5_STEP_INPUT_Pin}
};
const struct PORT_PIN_t aAxisDirInputs[] = {
  {AXIS1_DIR_INPUT_GPIO_Port, AXIS1_DIR_INPUT_Pin},
  {AXIS2_DIR_INPUT_GPIO_Port, AXIS2_DIR_INPUT_Pin},
  {AXIS3_DIR_INPUT_GPIO_Port, AXIS3_DIR_INPUT_Pin},
  {AXIS4_DIR_INPUT_GPIO_Port, AXIS4_DIR_INPUT_Pin},
  {AXIS5_DIR_INPUT_GPIO_Port, AXIS5_DIR_INPUT_Pin}
};
const struct PORT_PIN_t aAxisStepOutputs[] = {
  {AXIS1_STEP_OUTPUT_GPIO_Port, AXIS1_STEP_OUTPUT_Pin},
  {AXIS2_STEP_OUTPUT_GPIO_Port, AXIS2_STEP_OUTPUT_Pin},
  {AXIS3_STEP_OUTPUT_GPIO_Port, AXIS3_STEP_OUTPUT_Pin},
  {AXIS4_STEP_OUTPUT_GPIO_Port, AXIS4_STEP_OUTPUT_Pin},
  {AXIS5_STEP_OUTPUT_GPIO_Port, AXIS5_STEP_OUTPUT_Pin}
};
const struct PORT_PIN_t aAxisDirOutputs[] = {
  {AXIS1_DIR_OUTPUT_GPIO_Port, AXIS1_DIR_OUTPUT_Pin},
  {AXIS2_DIR_OUTPUT_GPIO_Port, AXIS2_DIR_OUTPUT_Pin},
  {AXIS3_DIR_OUTPUT_GPIO_Port, AXIS3_DIR_OUTPUT_Pin},
  {AXIS4_DIR_OUTPUT_GPIO_Port, AXIS4_DIR_OUTPUT_Pin},
  {AXIS5_DIR_OUTPUT_GPIO_Port, AXIS5_DIR_OUTPUT_Pin}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// setup DMA array values
void static inline setup_OC_DMA_array()
{
  uint16_t uhWidth = (OUT_TIM_BASE_PERIOD + 1) / (OUT_STEP_MULT*2);
  uint16_t uhWidthDivLost = (OUT_TIM_BASE_PERIOD + 1) % (OUT_STEP_MULT*2);

  for ( uint16_t i = 0, c = 0; i < (OUT_STEP_MULT*2 - 1); ++i )
  {
    if ( uhWidthDivLost )
    {
      c += uhWidth + 1;
      --uhWidthDivLost;
    }
    else
    {
      c += uhWidth;
    }

    aOC_DMA_val[i] = c;
  }

  aOC_DMA_val[OUT_STEP_MULT*2 - 1] = OUT_TIM_BASE_PERIOD;
}
// set axis DIR output pins same as inputs
void static inline setup_out_pins()
{
  for ( uint8_t axis = AXIS_CNT; axis--; )
  {
    HAL_GPIO_WritePin(
      aAxisStepOutputs[axis].PORT, aAxisStepOutputs[axis].PIN,
      HAL_GPIO_ReadPin(aAxisStepInputs[axis].PORT, aAxisStepInputs[axis].PIN)
    );

    HAL_GPIO_WritePin(
      aAxisDirOutputs[axis].PORT, aAxisDirOutputs[axis].PIN,
      HAL_GPIO_ReadPin(aAxisDirInputs[axis].PORT, aAxisDirInputs[axis].PIN)
    );
  }
}
// enable counter timer interrupts
void static inline start_counter_timer()
{
  HAL_TIM_Base_Start_IT(CNT_TIM);
  uwc = 0;
}
// start all out timers
void static inline start_out_timer()
{
  HAL_TIM_Base_Start_IT(OUT_TIM);

  __HAL_TIM_SET_COUNTER(OUT_TIM, 0);
  __HAL_TIM_SET_COMPARE(OUT_TIM, TIM_CHANNEL_1, aOC_DMA_val[0]);
  HAL_TIM_OC_Start_DMA(OUT_TIM, TIM_CHANNEL_1, &aOC_DMA_val[1], (OUT_STEP_MULT*2 - 1));
}




uint64_t static inline time_us()
{
  return (uwc*(0xFFFFFFFF+1) + __HAL_TIM_GET_COUNTER(CNT_TIM));
}




// update prescallers for the active output timers
// every 1000us (on systick event)
void update_out_timer_presc()
{
#if 0
  static uint8_t axis = 0;
  static uint32_t presc = 0;
#endif

  // TODO - proper out timer's prescaler handling

  #if 0
  for ( axis = AXIS_CNT; axis--; )
  {
    if ( aSteps[axis] )
    {
      presc = aStepInPeriod[axis];
      if (!presc || presc > INP_MAX_PERIOD) presc = INP_MAX_PERIOD;

      aAxisTimPrescPrev[axis] = aAxisTimPresc[axis];
      aAxisTimPresc[axis] = aSteps[axis] < presc ? (presc - aSteps[axis]) : 0;

      __HAL_TIM_SET_PRESCALER(
        OUT_TIM,
        ((aAxisTimPresc[axis] + aAxisTimPrescPrev[axis]) / 2)
      );
    }
  }
#endif
}




// on EXTI 4-0 input
void process_input_step(uint8_t axis)
{
  static uint64_t ulTime = 0;

  ++aSteps[axis];

  ulTime = time_us();
  aStepInPeriod[axis] = ulTime - aStepInTime[axis];
  aStepInTime[axis] = ulTime;
}
// on EXTI 5-9 input
void process_input_dirs()
{
  static uint8_t axis = 0;

  for ( axis = AXIS_CNT; axis--; )
  {
    if ( __HAL_GPIO_EXTI_GET_IT(aAxisDirInputs[axis].PIN) )
    {
      __HAL_GPIO_EXTI_CLEAR_IT(aAxisDirInputs[axis].PIN);

      // TODO - proper DIR handling
#if 0
      HAL_GPIO_TogglePin(aAxisDirOutputs[axis].PORT, aAxisDirOutputs[axis].PIN);
#endif
    }
  }
}

// on output timer OC it
void process_output()
{
  if ( __HAL_TIM_GET_FLAG(OUT_TIM, TIM_FLAG_CC1) )
  {
    if ( __HAL_TIM_GET_IT_SOURCE(OUT_TIM, TIM_IT_CC1) )
    {
      {
        __HAL_TIM_CLEAR_IT(OUT_TIM, TIM_IT_CC1);

        // TODO - proper output handling
      }
    }
  }
}

// on counter timer update
void counter_timer_update()
{
  ++uwc;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  setup_OC_DMA_array();
  setup_out_pins();
  start_counter_timer();
  start_out_timer();
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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
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

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = OUT_TIM_BASE_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = OUT_TIM_BASE_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = CNT_TIM_PRESCALLER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = CNT_TIM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, AXIS4_DIR_OUTPUT_Pin|AXIS5_DIR_OUTPUT_Pin|AXIS1_STEP_OUTPUT_Pin|AXIS2_STEP_OUTPUT_Pin 
                          |AXIS3_STEP_OUTPUT_Pin|AXIS4_STEP_OUTPUT_Pin|AXIS5_STEP_OUTPUT_Pin|AXIS1_DIR_OUTPUT_Pin 
                          |AXIS2_DIR_OUTPUT_Pin|AXIS3_DIR_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AXIS1_STEP_INPUT_Pin AXIS2_STEP_INPUT_Pin AXIS3_STEP_INPUT_Pin AXIS4_STEP_INPUT_Pin 
                           AXIS5_STEP_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_STEP_INPUT_Pin|AXIS2_STEP_INPUT_Pin|AXIS3_STEP_INPUT_Pin|AXIS4_STEP_INPUT_Pin 
                          |AXIS5_STEP_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS1_DIR_INPUT_Pin AXIS2_DIR_INPUT_Pin AXIS3_DIR_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_DIR_INPUT_Pin|AXIS2_DIR_INPUT_Pin|AXIS3_DIR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS4_DIR_OUTPUT_Pin AXIS5_DIR_OUTPUT_Pin AXIS1_STEP_OUTPUT_Pin AXIS2_STEP_OUTPUT_Pin 
                           AXIS3_STEP_OUTPUT_Pin AXIS4_STEP_OUTPUT_Pin AXIS5_STEP_OUTPUT_Pin AXIS1_DIR_OUTPUT_Pin 
                           AXIS2_DIR_OUTPUT_Pin AXIS3_DIR_OUTPUT_Pin */
  GPIO_InitStruct.Pin = AXIS4_DIR_OUTPUT_Pin|AXIS5_DIR_OUTPUT_Pin|AXIS1_STEP_OUTPUT_Pin|AXIS2_STEP_OUTPUT_Pin 
                          |AXIS3_STEP_OUTPUT_Pin|AXIS4_STEP_OUTPUT_Pin|AXIS5_STEP_OUTPUT_Pin|AXIS1_DIR_OUTPUT_Pin 
                          |AXIS2_DIR_OUTPUT_Pin|AXIS3_DIR_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS4_DIR_INPUT_Pin AXIS5_DIR_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS4_DIR_INPUT_Pin|AXIS5_DIR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
