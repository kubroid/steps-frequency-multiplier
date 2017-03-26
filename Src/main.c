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

/* USER CODE BEGIN PV */
#define AXIS_CNT            4 // 1..4
#define OUT_STEP_MULT       10 // 1..100
#define INP_MAX_PERIOD      3000 // us


#define CNT_TIM             &htim2 // timer to calculate periods
#define OUT_TIM             &htim1 // timer for output
#define BUF_SIZE            (UINT16_MAX+1)
#define CELL_SIZE           32


volatile uint32_t uwc = 0; // number of CNT_TIM's updates

volatile uint8_t auqOut[BUF_SIZE] = {0};
volatile uint32_t auwOutSet[BUF_SIZE/CELL_SIZE] = {0};
volatile uint16_t uhOutPos = 0;
volatile uint16_t auhLastOutPos[AXIS_CNT] = {0};

volatile uint64_t aulLastInTime[AXIS_CNT] = {0};
volatile uint64_t aulLastPeriod[AXIS_CNT] = {0};

struct PORT_PIN_t
{
  GPIO_TypeDef*   PORT;
  uint16_t        PIN;
};
const struct PORT_PIN_t asAxisStepInputs[] = {
  {AXIS1_STEP_INPUT_GPIO_Port, AXIS1_STEP_INPUT_Pin},
  {AXIS2_STEP_INPUT_GPIO_Port, AXIS2_STEP_INPUT_Pin},
  {AXIS3_STEP_INPUT_GPIO_Port, AXIS3_STEP_INPUT_Pin},
  {AXIS4_STEP_INPUT_GPIO_Port, AXIS4_STEP_INPUT_Pin},
  {AXIS5_STEP_INPUT_GPIO_Port, AXIS5_STEP_INPUT_Pin}
};
const struct PORT_PIN_t asAxisDirInputs[] = {
  {AXIS1_DIR_INPUT_GPIO_Port, AXIS1_DIR_INPUT_Pin},
  {AXIS2_DIR_INPUT_GPIO_Port, AXIS2_DIR_INPUT_Pin},
  {AXIS3_DIR_INPUT_GPIO_Port, AXIS3_DIR_INPUT_Pin},
  {AXIS4_DIR_INPUT_GPIO_Port, AXIS4_DIR_INPUT_Pin},
  {AXIS5_DIR_INPUT_GPIO_Port, AXIS5_DIR_INPUT_Pin}
};
const struct PORT_PIN_t asAxisStepOutputs[] = {
  {AXIS1_STEP_OUTPUT_GPIO_Port, AXIS1_STEP_OUTPUT_Pin},
  {AXIS2_STEP_OUTPUT_GPIO_Port, AXIS2_STEP_OUTPUT_Pin},
  {AXIS3_STEP_OUTPUT_GPIO_Port, AXIS3_STEP_OUTPUT_Pin},
  {AXIS4_STEP_OUTPUT_GPIO_Port, AXIS4_STEP_OUTPUT_Pin},
  {AXIS5_STEP_OUTPUT_GPIO_Port, AXIS5_STEP_OUTPUT_Pin}
};
const struct PORT_PIN_t asAxisDirOutputs[] = {
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
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// enable counter timer interrupts
void static inline start_counter_timer_it()
{
  HAL_TIM_Base_Start_IT(CNT_TIM);
  uwc = 0;
}

// enable out timer interrupts
void static inline start_out_timer_it()
{
  HAL_TIM_Base_Start_IT(OUT_TIM);
}

// prepare out port
void static inline setup_output_port()
{
  // set out poer same as input port
  (asAxisStepOutputs[0].PORT)->ODR = (asAxisStepInputs[0].PORT)->IDR;

  for ( uint32_t o = UINT16_MAX+1; o--; ) auqOut[o] = UINT8_MAX;
}




//
uint64_t static inline time_us()
{
  return (uwc*(0xFFFFFFFF+1) + __HAL_TIM_GET_COUNTER(CNT_TIM));
}




//
uint32_t static inline out_cell_state(uint16_t cell)
{
  return auwOutSet[cell/CELL_SIZE] & (1 << (cell%CELL_SIZE));
}
//
void static inline out_cell_set(uint16_t cell)
{
  auwOutSet[cell/CELL_SIZE] |= (1 << (cell%CELL_SIZE));
}
//
void static inline out_cell_reset(uint16_t cell)
{
  auwOutSet[cell/CELL_SIZE] &= ~(1 << (cell%CELL_SIZE));
}




// on EXTI 0-4 low state
void process_input_step(uint8_t axis)
{
  static uint8_t uqM = 0;
  static uint16_t uhT = 0;
  static uint16_t uhPos = 0;
  static uint64_t ulTime = 0;

  // get timestamp
  ulTime = time_us();

  // get period
  aulLastPeriod[axis] = ulTime - aulLastInTime[axis];
  aulLastInTime[axis] = ulTime;

  // get output pos
  if ( aulLastPeriod[axis] > INP_MAX_PERIOD )
  {
    aulLastPeriod[axis] = INP_MAX_PERIOD;
    uhPos = uhOutPos;
  }
  else
  {
    uhPos = auhLastOutPos[axis];
  }
  auhLastOutPos[axis] = uhPos + aulLastPeriod[axis];

  // get pin toggle time
#if 1
  uhT = aulLastPeriod[axis] / (OUT_STEP_MULT*2);
#else
  uhT = 1;
#endif

  // add pin toggles to the out buffer
  for ( uhPos += uhT, uqM = OUT_STEP_MULT; uqM--; uhPos += uhT )
  {
    CLEAR_BIT(auqOut[uhPos], 1 << axis);
    out_cell_set(uhPos);
    uhPos += uhT;

    SET_BIT(auqOut[uhPos], 1 << axis);
    out_cell_set(uhPos);
  }
}

// on EXTI 5-9 input
void process_input_dirs()
{
  static uint8_t axis = 0;
  static uint16_t uhPos = 0;
  static uint64_t ulTime = 0;

  for ( axis = AXIS_CNT; axis--; )
  {
    if ( __HAL_GPIO_EXTI_GET_IT(asAxisDirInputs[axis].PIN) )
    {
      __HAL_GPIO_EXTI_CLEAR_IT(asAxisDirInputs[axis].PIN);

      // get timestamp
      ulTime = time_us();

      // get period
      aulLastPeriod[axis] = ulTime - aulLastInTime[axis];
      aulLastInTime[axis] = ulTime;

      // get output pos
      if ( aulLastPeriod[axis] > INP_MAX_PERIOD )
      {
        aulLastPeriod[axis] = INP_MAX_PERIOD;
        uhPos = uhOutPos + 1;
      }
      else
      {
        uhPos = auhLastOutPos[axis] + 1;
      }
      auhLastOutPos[axis] = uhPos;

      // add input value into the buffer
      if ( HAL_GPIO_ReadPin(asAxisDirInputs[axis].PORT, asAxisDirInputs[axis].PIN) )
      {
        SET_BIT(auqOut[uhPos], 1 << (axis+AXIS_CNT));
      }
      else
      {
        CLEAR_BIT(auqOut[uhPos], 1 << (axis+AXIS_CNT));
      }
      out_cell_set(uhPos);

      return;
    }
  }
}

// on out TIM update
void process_output()
{
  static uint32_t out = 0;
  static uint8_t axis = 0;

  __HAL_TIM_CLEAR_IT(OUT_TIM, TIM_IT_UPDATE);

  if ( out_cell_state(uhOutPos) )
  {
    out_cell_reset(uhOutPos);

    for ( out = 0, axis = AXIS_CNT; axis--; )
    {
      if ( READ_BIT(auqOut[uhOutPos], 1 << axis) ) {
        SET_BIT(out, (uint32_t) asAxisStepOutputs[axis].PIN);
      } else {
        SET_BIT(out, (uint32_t) (asAxisStepOutputs[axis].PIN << 16U));
      }

      if ( READ_BIT(auqOut[uhOutPos], 1 << (axis+AXIS_CNT)) ) {
        SET_BIT(out, (uint32_t) asAxisDirOutputs[axis].PIN);
      } else {
        SET_BIT(out, (uint32_t) (asAxisDirOutputs[axis].PIN << 16U));
      }
    }

    // set output port value
    AXIS1_STEP_OUTPUT_GPIO_Port->BSRR = out;
    auqOut[uhOutPos] = UINT8_MAX;
  }

  ++uhOutPos;
}

//
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
  MX_TIM2_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  start_counter_timer_it();
  start_out_timer_it();
  setup_output_port();
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
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = OUT_TIM_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = OUT_TIM_PERIOD;
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

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  /*Configure GPIO pins : AXIS1_DIR_INPUT_Pin AXIS2_DIR_INPUT_Pin AXIS3_DIR_INPUT_Pin AXIS4_DIR_INPUT_Pin 
                           AXIS5_DIR_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_DIR_INPUT_Pin|AXIS2_DIR_INPUT_Pin|AXIS3_DIR_INPUT_Pin|AXIS4_DIR_INPUT_Pin 
                          |AXIS5_DIR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS4_DIR_OUTPUT_Pin AXIS5_DIR_OUTPUT_Pin AXIS1_STEP_OUTPUT_Pin AXIS2_STEP_OUTPUT_Pin 
                           AXIS3_STEP_OUTPUT_Pin AXIS4_STEP_OUTPUT_Pin AXIS5_STEP_OUTPUT_Pin AXIS1_DIR_OUTPUT_Pin 
                           AXIS2_DIR_OUTPUT_Pin AXIS3_DIR_OUTPUT_Pin */
  GPIO_InitStruct.Pin = AXIS4_DIR_OUTPUT_Pin|AXIS5_DIR_OUTPUT_Pin|AXIS1_STEP_OUTPUT_Pin|AXIS2_STEP_OUTPUT_Pin 
                          |AXIS3_STEP_OUTPUT_Pin|AXIS4_STEP_OUTPUT_Pin|AXIS5_STEP_OUTPUT_Pin|AXIS1_DIR_OUTPUT_Pin 
                          |AXIS2_DIR_OUTPUT_Pin|AXIS3_DIR_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
