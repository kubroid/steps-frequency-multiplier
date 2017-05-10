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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim1_ch3;

/* USER CODE BEGIN PV */

// SETTINGS

#define AXIS_CNT            1     // axis count = 1..5
#define OUT_STEP_MULT       10    // steps frequency multiplier = 1..100
#define MAX_INPUT_FREQ      50    // KHz, max input steps frequency

#define DIR_HOLD_TIME       50    // us, delay before DIR change
#define DIR_SETUP_TIME      50    // us, delay after DIR change





// OUTPUT TIMERS

static TIM_HandleTypeDef* aAxisTim[] = { // links to the output timers
    &htim1, // axis 0
    &htim2, // axis 1
    &htim3, // axis 2
    &htim4, // axis 3
    &htim5  // axis 4
};
static const uint32_t auwAxisTimStepCh[] = { // list of STEP output channels
    TIM_CHANNEL_3,  // axis 0
    TIM_CHANNEL_3,  // axis 1
    TIM_CHANNEL_1,  // axis 2
    TIM_CHANNEL_1,  // axis 3
    TIM_CHANNEL_3   // axis 4
};
static const uint32_t auwAxisTimDirCh[] = { // list of DIR output channels
    TIM_CHANNEL_4,  // axis 0
    TIM_CHANNEL_4,  // axis 1
    TIM_CHANNEL_2,  // axis 2
    TIM_CHANNEL_2,  // axis 3
    TIM_CHANNEL_4   // axis 4
};
static const uint8_t auqTimSourceFreqMHz[] = { // source frequencies of the timers
    168,  // axis 0
     84,  // axis 1
     84,  // axis 2
     84,  // axis 3
     84   // axis 4
};
static const uint8_t auqAxisTimDirPresc[] = { // prescalers for the DIR outputs
    168-1,  // axis 0
     84-1,  // axis 1
     84-1,  // axis 2
     84-1,  // axis 3
     84-1   // axis 4
};





// OUTPUT BUFFER

#define OUT_BUF_SIZE 256

struct OUTBUF_t
{
  int16_t   hCount;
  uint16_t  uhTime;
};

volatile struct OUTBUF_t aBuf[AXIS_CNT][OUT_BUF_SIZE] = {{{0}}};
volatile uint8_t auqBufAddPos[AXIS_CNT] = {0};
volatile uint8_t auqBufOutPos[AXIS_CNT] = {0};




// INPUT STEPS
volatile uint16_t auhSteps[AXIS_CNT] = {0};
#if 0
volatile uint16_t auhStepsMax[AXIS_CNT] = {0};
#endif



// SERVO CYCLE
volatile uint64_t ulTime = 0;
volatile uint64_t aulTimePrev[AXIS_CNT] = {0};




// OUTPUT
volatile uint8_t auqOutputOn[AXIS_CNT] = {0};
volatile uint16_t auhTimPresc[AXIS_CNT] = {0};
volatile uint16_t auhTimPeriod[AXIS_CNT] = {0};




// SYSTICK BASED COUNTER
volatile uint32_t uwSysTickClk = 0;
volatile uint32_t uwSysTimeDivUS = 0;




// OC DMA ARRAY

#define DMA_ARRAY_SIZE (MAX_INPUT_FREQ * OUT_STEP_MULT * 2)

uint16_t auhOCDMAVal[DMA_ARRAY_SIZE] = {0};




// PINOUT

struct PORT_PIN_t
{
  GPIO_TypeDef*   PORT;
  uint16_t        PIN;
};
const struct PORT_PIN_t aAxisStepInputs[] = {
  {AXIS1_STEP_INPUT_GPIO_Port, AXIS1_STEP_INPUT_Pin},
#if 0
  {AXIS2_STEP_INPUT_GPIO_Port, AXIS2_STEP_INPUT_Pin},
  {AXIS3_STEP_INPUT_GPIO_Port, AXIS3_STEP_INPUT_Pin},
  {AXIS4_STEP_INPUT_GPIO_Port, AXIS4_STEP_INPUT_Pin},
  {AXIS5_STEP_INPUT_GPIO_Port, AXIS5_STEP_INPUT_Pin}
#endif
};
const struct PORT_PIN_t aAxisDirInputs[] = {
  {AXIS1_DIR_INPUT_GPIO_Port, AXIS1_DIR_INPUT_Pin},
#if 0
  {AXIS2_DIR_INPUT_GPIO_Port, AXIS2_DIR_INPUT_Pin},
  {AXIS3_DIR_INPUT_GPIO_Port, AXIS3_DIR_INPUT_Pin},
  {AXIS4_DIR_INPUT_GPIO_Port, AXIS4_DIR_INPUT_Pin},
  {AXIS5_DIR_INPUT_GPIO_Port, AXIS5_DIR_INPUT_Pin}
#endif
};
const struct PORT_PIN_t aAxisStepOutputs[] = {
  {AXIS1_STEP_OUTPUT_GPIO_Port, AXIS1_STEP_OUTPUT_Pin},
#if 0
  {AXIS2_STEP_OUTPUT_GPIO_Port, AXIS2_STEP_OUTPUT_Pin},
  {AXIS3_STEP_OUTPUT_GPIO_Port, AXIS3_STEP_OUTPUT_Pin},
  {AXIS4_STEP_OUTPUT_GPIO_Port, AXIS4_STEP_OUTPUT_Pin},
  {AXIS5_STEP_OUTPUT_GPIO_Port, AXIS5_STEP_OUTPUT_Pin}
#endif
};
const struct PORT_PIN_t aAxisDirOutputs[] = {
  {AXIS1_DIR_OUTPUT_GPIO_Port, AXIS1_DIR_OUTPUT_Pin},
#if 0
  {AXIS2_DIR_OUTPUT_GPIO_Port, AXIS2_DIR_OUTPUT_Pin},
  {AXIS3_DIR_OUTPUT_GPIO_Port, AXIS3_DIR_OUTPUT_Pin},
  {AXIS4_DIR_OUTPUT_GPIO_Port, AXIS4_DIR_OUTPUT_Pin},
  {AXIS5_DIR_OUTPUT_GPIO_Port, AXIS5_DIR_OUTPUT_Pin}
#endif
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_NVIC_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// setup DMA array values
void static inline setup_OC_DMA_array()
{
  // auhOCDMAVal = {0,1,2,3,4,...}
  for ( uint16_t i = 0; i < DMA_ARRAY_SIZE; ++i ) auhOCDMAVal[i] = i+1;
}

// setup output timers data
void static inline setup_out_timers()
{
  for ( uint8_t axis = AXIS_CNT; axis--; )
  {
    // enable Update event
    __HAL_TIM_ENABLE_IT(aAxisTim[axis], TIM_IT_UPDATE);
    // set output DIR setup time
    __HAL_TIM_SET_COMPARE(aAxisTim[axis], auwAxisTimDirCh[axis], DIR_SETUP_TIME);
  }
}

// setup US counter data
void static inline setup_counter()
{
  uwSysTickClk = HAL_RCC_GetHCLKFreq()/1000;
  uwSysTimeDivUS = HAL_RCC_GetHCLKFreq()/1000000;
}




// get timestamp in US since system startup
// value is overloaded after 49.71 days (256*256*256*256 ms)
uint64_t static inline time_us()
{
  return HAL_GetTick()*1000 + (uwSysTickClk - SysTick->VAL)/uwSysTimeDivUS;
}




// start output for selected axis
void static inline start_output(uint8_t axis)
{
  // if we have anything to output
  if ( aBuf[axis][auqBufOutPos[axis]].hCount )
  {
    // if we have steps to output
    if ( aBuf[axis][auqBufOutPos[axis]].hCount > 0 )
    {
      // calculate timer's new period
      auhTimPeriod[axis] = aBuf[axis][auqBufOutPos[axis]].hCount
        * (OUT_STEP_MULT * 2)
        - 1;

      // calculate timer's new prescaler
      auhTimPresc[axis] = auqTimSourceFreqMHz[axis]
        * aBuf[axis][auqBufOutPos[axis]].uhTime
        / (auhTimPeriod[axis] + 1)
        - 1;

      // update prescaler, period, compare value
      __HAL_TIM_SET_PRESCALER(aAxisTim[axis], auhTimPresc[axis]);
      __HAL_TIM_SET_AUTORELOAD(aAxisTim[axis], auhTimPeriod[axis]);
      __HAL_TIM_SET_COMPARE(aAxisTim[axis], auwAxisTimStepCh[axis], auhOCDMAVal[0]);

      // generate an update event to reload the Prescaler
      aAxisTim[axis]->Instance->EGR |= TIM_EGR_UG; // update event
      aAxisTim[axis]->Instance->SR  &= ~TIM_SR_UIF; // reset update flag

      // start timer's channel in OC+DMA mode
      HAL_TIM_OC_Start_DMA(
          aAxisTim[axis], auwAxisTimStepCh[axis],
          ((uint32_t*)&auhOCDMAVal[1]), auhTimPeriod[axis] - 1
      );

      // turn on step output flag
      auqOutputOn[axis] = 1;
    }
    // if we have DIR change
    else
    {
      // update prescaler, period
      __HAL_TIM_SET_PRESCALER( aAxisTim[axis], auqAxisTimDirPresc[axis] );
      __HAL_TIM_SET_AUTORELOAD(aAxisTim[axis], DIR_HOLD_TIME + DIR_SETUP_TIME);

      // generate an update event to reload the Prescaler
      aAxisTim[axis]->Instance->EGR |= TIM_EGR_UG; // update event
      aAxisTim[axis]->Instance->SR  &= ~TIM_SR_UIF; // reset update flag

      // start timer's channel in OC mode
      HAL_TIM_OC_Start_IT(aAxisTim[axis], auwAxisTimStepCh[axis]);

      // turn on DIR output flag
      auqOutputOn[axis] = 2;
    }
  }
}

// stop output for selected axis
void static inline stop_output(uint8_t axis)
{
  // if output is enabled
  if ( auqOutputOn[axis] )
  {
    // if it was a step output
    if ( auqOutputOn[axis] == 1 )
    {
      // stop timer's channel in OC+DMA mode
      HAL_TIM_OC_Stop_DMA(aAxisTim[axis], auwAxisTimStepCh[axis]);
    }
    // if it was a DIR output
    else
    {
      // stop timer's channel in OC mode
      HAL_TIM_OC_Stop_IT(aAxisTim[axis], auwAxisTimDirCh[axis]);
    }

    // turn off output flag
    auqOutputOn[axis] = 0;
  }
}

// get output enabled flag
uint8_t static inline output(uint8_t axis)
{
  return auqOutputOn[axis];
}




// on EXTI 4-0 input
void process_servo_cycle()
{
  static uint8_t axis = 0;
  static uint64_t ulTime = 0;

#if 0
  // TODO - it's test code
  if ( !(HAL_GetTick() % 1000) )
  {
    aBuf[0][auqBufAddPos[0]].hCount = 1;
    aBuf[0][auqBufAddPos[0]].uhTime = 5000;
    ++auqBufAddPos[0];

    if ( !output(0) ) start_output(0);
  }
#endif
#if 1
  // get current time
  ulTime = time_us();

  for ( axis = AXIS_CNT; axis--; )
  {
    // if we have any input steps
    if ( auhSteps[axis] )
    {
      // add them to the out buffer
      aBuf[axis][auqBufAddPos[axis]].hCount = auhSteps[axis];
      aBuf[axis][auqBufAddPos[axis]].uhTime = ulTime - aulTimePrev[axis];

      // goto next buff pos
      ++auqBufAddPos[axis];

      // reset steps count
      auhSteps[axis] = 0;

      if ( !output(axis) ) start_output(axis);
    }

    // save current time for all axes
    aulTimePrev[axis] = ulTime;
  }
#endif
}

// on EXTI 4-0 input
void process_input_step(uint8_t axis)
{
  // update input steps count
  ++auhSteps[axis];
}

// on EXTI 5-9 input
void process_input_dirs()
{
  static uint8_t axis = 0;
  static uint64_t ulTime = 0;

  // get current time
  ulTime = time_us();

  for ( axis = AXIS_CNT; axis--; )
  {
    if ( __HAL_GPIO_EXTI_GET_IT(aAxisDirInputs[axis].PIN) )
    {
      __HAL_GPIO_EXTI_CLEAR_IT(aAxisDirInputs[axis].PIN);

#if 0
      // if we have any input steps before DIR change
      if ( auhSteps[axis] )
      {
        // add them to the out buffer
        aBuf[axis][auqBufAddPos[axis]].hCount = auhSteps[axis];
        aBuf[axis][auqBufAddPos[axis]].uhTime = ulTime - aulTimePrev[axis];
        // goto next buff pos
        ++auqBufAddPos[axis];

        // reset steps count
        auhSteps[axis] = 0;
      }

      // save current time for this axis
      aulTimePrev[axis] = ulTime;

      // add DIR change to the out buffer
      aBuf[axis][auqBufAddPos[axis]].hCount = -1;
      // goto next buff pos
      ++auqBufAddPos[axis];

      if ( !output(axis) ) start_output(axis);
#endif
    }
  }
}

// on axis TIM update
void on_axis_tim_update(uint8_t axis)
{
  if ( __HAL_TIM_GET_FLAG(aAxisTim[axis], TIM_FLAG_UPDATE) )
  {
    if ( __HAL_TIM_GET_IT_SOURCE(aAxisTim[axis], TIM_IT_UPDATE) )
    {
      __HAL_TIM_CLEAR_IT(aAxisTim[axis], TIM_IT_UPDATE);

      // if we have output enabled
      if ( output(axis) )
      {
        // disable output
        stop_output(axis);
        // reset buf pos cell
        aBuf[axis][auqBufOutPos[axis]].hCount = 0;
        // goto next buf pos
        ++auqBufOutPos[axis];
        // if we have something else to output - start output
        if ( aBuf[axis][auqBufOutPos[axis]].hCount ) start_output(axis);
      }
    }
  }
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  setup_counter();
  setup_OC_DMA_array();
  setup_out_timers();
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
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = OUT_TIM_PRESC;
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

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = DIR_SETUP_TIME_US;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
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

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PD8 PD9 PD1 PD2 
                           PD3 PD4 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : AXIS1_STEP_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_STEP_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(AXIS1_STEP_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AXIS1_DIR_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_DIR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(AXIS1_DIR_INPUT_GPIO_Port, &GPIO_InitStruct);

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
