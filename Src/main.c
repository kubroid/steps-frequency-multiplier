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
#define _MAIN_C
#include "shared.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// TEST 1
int32_t _TEST_axis_step_pos[AXES] = {0};
uint32_t _TEST_axis_dir_state[AXES] = {0};

void init_ports_pins()
{
  // find EXTI 9:5 and 15:10 pins
  uint8_t pos[4] = {0};
  for ( uint8_t a = AXES; a--; )
  {
    // find a list of STEP input pins for EXTI 9:5
    if ( STEP_INPUTS[a].PIN >= GPIO_PIN_5 && STEP_INPUTS[a].PIN <= GPIO_PIN_9 )
    {
      exti_9_5_step_pins[ pos[0]++ ] = a;
    }
    else if ( STEP_INPUTS[a].PIN >= GPIO_PIN_10 && STEP_INPUTS[a].PIN <= GPIO_PIN_15 )
    {
      exti_15_10_step_pins[ pos[1]++ ] = a;
    }

    // find a list of DIR input pins for EXTI 15:10
    if ( DIR_INPUTS[a].PIN >= GPIO_PIN_5 && DIR_INPUTS[a].PIN <= GPIO_PIN_9 )
    {
      exti_9_5_dir_pins[ pos[2]++ ] = a;
    }
    else if ( DIR_INPUTS[a].PIN >= GPIO_PIN_10 && DIR_INPUTS[a].PIN <= GPIO_PIN_15 )
    {
      exti_15_10_dir_pins[ pos[3]++ ] = a;
    }
  }

  exti_9_5_step_pins_count    = pos[0];
  exti_15_10_step_pins_count  = pos[1];
  exti_9_5_dir_pins_count     = pos[2];
  exti_15_10_dir_pins_count   = pos[3];
}

void input_IRQ_handler
(
    uint8_t step_pins[], uint8_t *step_pins_count,
    uint8_t dir_pins[],  uint8_t *dir_pins_count
)
{
  // checking step pins for any changes
  #define a step_pins[p]
  for ( uint8_t p = *step_pins_count; p--; )
  {
    // get current state of a pin
    step_input_state_cur[a] = HAL_GPIO_ReadPin(STEP_INPUTS[a].PORT, STEP_INPUTS[a].PIN);

    // on step signal rise
    if ( step_input_state_cur[a] && !step_input_state_prev[a] )
    {
      process_input_value(&a, -1);
    }

    // save current state of a pin
    step_input_state_prev[a] = step_input_state_cur[a];
  }
  #undef a

  // checking dir pins for any changes
  #define a dir_pins[p]
  for ( uint8_t p = *dir_pins_count; p--; )
  {
    // get current state of a pin
    dir_input_state_cur[a] = HAL_GPIO_ReadPin(DIR_INPUTS[a].PORT, DIR_INPUTS[a].PIN);

    // on dir signal change
    if ( dir_input_state_cur[a] != dir_input_state_prev[a] )
    {
      process_input_value(&a, dir_input_state_cur[a] + 1);
    }

    // save current state of a pin
    dir_input_state_prev[a] = dir_input_state_cur[a];
  }
  #undef a
}

void output_generator_step()
{
  #define pos out_pos[axis]
  #define STEP_OUT STEP_OUTPUTS[axis]
  #define DIR_OUT DIR_OUTPUTS[axis]
  for ( uint8_t axis = AXES; axis--; )
  {
    if ( !out_time[axis][pos] ) continue;

    switch ( out_val[axis][pos] )
    {
      case -2: // step low
        if ( step_output_state_cur[axis] == GPIO_PIN_SET )
        {
          HAL_GPIO_WritePin(STEP_OUT.PORT, STEP_OUT.PIN, GPIO_PIN_RESET);
        }
        break;

      case -1: // step high
        if ( step_output_state_cur[axis] == GPIO_PIN_RESET )
        {
          HAL_GPIO_WritePin(STEP_OUT.PORT, STEP_OUT.PIN, GPIO_PIN_SET);
        }
        break;

      case 1: // dir low
        if ( dir_output_state_cur[axis] == GPIO_PIN_SET )
        {
          HAL_GPIO_WritePin(DIR_OUT.PORT, DIR_OUT.PIN, GPIO_PIN_RESET);
        }
        break;

      case 2: // dir high
        if ( dir_output_state_cur[axis] == GPIO_PIN_RESET )
        {
          HAL_GPIO_WritePin(DIR_OUT.PORT, DIR_OUT.PIN, GPIO_PIN_SET);
        }
        break;
    }

    if ( !out_time[axis][pos] ) ++pos;
  }
  #undef pos
  #undef STEP_OUT
  #undef DIR_OUT
}

void servo_cycle_handler()
{
  // servo cycle counter update and check
  if ( ++servo_cycle_timestamp_us >= SERVO_CYCLE_INTERVAL )
  {
    // reset servo cycle time counter (actualy not needed)
    servo_cycle_timestamp_us = 0;

    process_input_buffers();
  }

  // step/dir output from the buffer
  output_generator_step();
}

void process_input_value ( uint8_t* axis, int8_t value )
{
  // TEST 1
  if ( value < 0 ) // if we have a step
  {
    _TEST_axis_step_pos[*axis] += _TEST_axis_dir_state[*axis] ? -1 : 1;
  }
  else // if we have a direction change
  {
    _TEST_axis_dir_state[*axis] = value - 1;
  }

  // save input value
  inp_val[*axis][ inp_pos[*axis] ] = value;
  // save input time
  inp_time[*axis][ inp_pos[*axis] ] = servo_cycle_timestamp_us;
  // increase input position
  ++inp_pos[*axis];

  // process input buffers if input position is greater than input buffer size
  if ( inp_pos[*axis] >= INP_BUF_SIZE ) process_input_buffers();
}

void process_input_buffers()
{
  // reset servo cycle time counter
  servo_cycle_timestamp_us = 0;

  // process input step/dir buffer
  if ( INPUT_FILTERING )
  {
    for ( uint8_t axis = AXES; axis--; )
    {
      // filtering input buffer of timestamps
      for ( uint16_t p = inp_pos[axis] - 2; p--; )
      {
        inp_time[axis][p+1] = (inp_time[axis][p] + inp_time[axis][p+2]) / 2;
      }
    }
  }

  // add input buffer to the output buffer
  #define pos out_add_pos[axis]
  for ( uint8_t axis = AXES; axis--; )
  {
    // do nothing if no input for the current axis
    if ( !inp_pos[axis] ) continue;

    // add start delay if needed
    if ( inp_time[axis][0] )
    {
      out_val[axis][pos]  = 0;
      out_time[axis][pos] = inp_time[axis][0];
      if ( ++pos >= OUT_BUF_SIZE ) pos = 0;
    }

    for ( uint16_t p = 0, time; p < inp_pos[axis]; ++p )
    {
      // don't calculate the last period in the input buffer
      // because p+1 input position doesn't exist
      if ( (p + 1) < inp_pos[axis] )
      {
        inp_last_period[axis] = inp_time[axis][p+1] - inp_time[axis][p];
      }
      // add input value to the output buffer
      switch ( inp_val[axis][p] )
      {
        case -1: // add a STEP
          // time of the one step state
          time = inp_last_period[axis] / (MULT*2);
          // add multiplied steps to the buffer
          for ( uint8_t m = MULT; m--; )
          {
            // add STEP high state
            out_val[axis][pos] = -1;
            out_time[axis][pos] = (time * OUTPUT_STEP_DUTY_CYCLE) / 50;
            if ( !out_time[axis][pos] ) out_time[axis][pos] = 1;
            if ( ++pos >= OUT_BUF_SIZE ) pos = 0;
            // add STEP low state
            out_val[axis][pos] = -2;
            out_time[axis][pos] = (time * 50) / OUTPUT_STEP_DUTY_CYCLE;
            if ( !out_time[axis][pos] ) out_time[axis][pos] = 1;
            if ( ++pos >= OUT_BUF_SIZE ) pos = 0;
          }
          break;

        case 1: // add a DIR low
        case 2: // add a DIR high
          // time of the one dir change
          time = inp_last_period[axis] / 2;
          // add a delay
          out_val[axis][pos] = 0;
          out_time[axis][pos] = time > OUTPUT_DIR_DELAY_BEFORE ? time : OUTPUT_DIR_DELAY_BEFORE;
          if ( ++pos >= OUT_BUF_SIZE ) pos = 0;
          // add a DIR low/high
          out_val[axis][pos] = inp_val[axis][p];
          out_time[axis][pos] = time > OUTPUT_DIR_DELAY_AFTER ? time : OUTPUT_DIR_DELAY_AFTER;
          if ( ++pos >= OUT_BUF_SIZE ) pos = 0;
          break;

        default: // add delay
          out_val[axis][pos] = 0;
          out_time[axis][pos] = inp_time[axis][p];
          if ( ++pos >= OUT_BUF_SIZE ) pos = 0;
      }
    }

    // reset add position for input values
    inp_pos[axis] = 0;
  }
  #undef pos
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
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */

  // init pins
  init_ports_pins();

  // start SERVO CYCLE interrupt
  HAL_TIM_Base_Start_IT(&htim3);

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
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = SERVO_CYCLE_TIM_PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = SERVO_CYCLE_TIM_MAX_COUNT;
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

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AXIS1_STEP_OUTPUT_Pin|AXIS1_DIR_OUTPUT_Pin|AXIS2_STEP_OUTPUT_Pin|AXIS2_DIR_OUTPUT_Pin 
                          |AXIS3_STEP_OUTPUT_Pin|AXIS3_DIR_OUTPUT_Pin|AXIS4_STEP_OUTPUT_Pin|AXIS4_DIR_OUTPUT_Pin 
                          |AXIS5_STEP_OUTPUT_Pin|AXIS5_DIR_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AXIS1_STEP_OUTPUT_Pin AXIS1_DIR_OUTPUT_Pin AXIS2_STEP_OUTPUT_Pin AXIS2_DIR_OUTPUT_Pin 
                           AXIS3_STEP_OUTPUT_Pin AXIS3_DIR_OUTPUT_Pin AXIS4_STEP_OUTPUT_Pin AXIS4_DIR_OUTPUT_Pin 
                           AXIS5_STEP_OUTPUT_Pin AXIS5_DIR_OUTPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_STEP_OUTPUT_Pin|AXIS1_DIR_OUTPUT_Pin|AXIS2_STEP_OUTPUT_Pin|AXIS2_DIR_OUTPUT_Pin 
                          |AXIS3_STEP_OUTPUT_Pin|AXIS3_DIR_OUTPUT_Pin|AXIS4_STEP_OUTPUT_Pin|AXIS4_DIR_OUTPUT_Pin 
                          |AXIS5_STEP_OUTPUT_Pin|AXIS5_DIR_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS3_STEP_INPUT_Pin AXIS3_DIR_INPUT_Pin AXIS4_STEP_INPUT_Pin AXIS4_DIR_INPUT_Pin 
                           AXIS5_STEP_INPUT_Pin AXIS5_DIR_INPUT_Pin AXIS1_STEP_INPUT_Pin AXIS1_DIR_INPUT_Pin 
                           AXIS2_STEP_INPUT_Pin AXIS2_DIR_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS3_STEP_INPUT_Pin|AXIS3_DIR_INPUT_Pin|AXIS4_STEP_INPUT_Pin|AXIS4_DIR_INPUT_Pin 
                          |AXIS5_STEP_INPUT_Pin|AXIS5_DIR_INPUT_Pin|AXIS1_STEP_INPUT_Pin|AXIS1_DIR_INPUT_Pin 
                          |AXIS2_STEP_INPUT_Pin|AXIS2_DIR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
