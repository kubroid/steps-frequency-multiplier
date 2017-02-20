/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "shared.h"

  // add a step to the output buffer of current axis
void static inline out_buf_add_step ( const uint8_t axis )
{
  static uint16_t t1;
  static uint16_t t2;
  static uint8_t m;

  #define pos out_add_pos[axis]
  // TODO - TEST CODE - remove this code
  _TEST_axis_step_pos[axis] += _TEST_axis_dir_state[axis] ? -1 : 1;

  // calculate duration of the one pulse state
#if INPUT_SIMPLE_PERIOD_FILTER
  inp_last_period[axis] = ((PULSE_WAIT_TIME - inp_wait_time[axis]) + inp_last_period[axis]) / 2;
  t1 = inp_last_period[axis] / (MULT*2); // pulse time
  t2 = inp_last_period[axis] % (MULT*2); // missed time
#else
  t1 = (PULSE_WAIT_TIME - inp_wait_time[axis]) / (MULT*2); // pulse time
  t2 = (PULSE_WAIT_TIME - inp_wait_time[axis]) % (MULT*2); // missed time
#endif

  // add multiplied values to the output buffer
  for ( m = MULT; m--; )
  {
    out_val[axis][pos] = 1;
    out_time[axis][pos] = t1;
    if ( t2 ) // add a piece of missed time to the step high time
    {
      ++out_time[axis][pos];
      --t2;
    }
    ++pos;

    out_val[axis][pos] = 0;
    out_time[axis][pos] = t1;
    if ( t2 ) // add a piece of missed time to the step low time
    {
      ++out_time[axis][pos];
      --t2;
    }
    ++pos;
  }
  #undef pos

  // reset wait timer for this axis
  inp_wait_time[axis] = PULSE_WAIT_TIME;
}

// add direction changes to the output buffer of current axis
void static inline out_buf_add_dir
(
    const uint8_t       axis,
    const GPIO_PinState cur,
    const GPIO_PinState prev
)
{
  #define pos out_add_pos[axis]
  // TODO - TEST CODE - remove this code
  _TEST_axis_dir_state[axis] = cur;

#if INPUT_SIMPLE_PERIOD_FILTER
  inp_last_period[axis] = PULSE_WAIT_TIME - inp_wait_time[axis];
#endif

  // add direction change to the output buffer
  out_val[axis][pos] = prev ? 4 : 2;
  out_time[axis][pos] = DIR_DELAY_BEFORE;
  ++pos;
  out_val[axis][pos] = cur ? 4 : 2;
  out_time[axis][pos] = DIR_DELAY_AFTER;
  ++pos;
  #undef pos

  // reset wait timer for this axis
  inp_wait_time[axis] = PULSE_WAIT_TIME;
}

// uses to update any time counters
void static inline time_tick(void)
{
  static uint8_t axis;

  // update individual wait timers for each axis
  for ( axis = AXES; axis--; )
  {
    if ( inp_wait_time[axis] ) --inp_wait_time[axis];
  }
}

// uses to output steps/directions
void static inline output_tick(void)
{
  static uint8_t axis;

  #define pos         out_pos[axis]
  #define STEP_OUT    STEP_OUTPUTS[axis]
  #define DIR_OUT     DIR_OUTPUTS[axis]
  #define step_state  step_output_state_cur[axis]
  #define dir_state   dir_output_state_cur[axis]
  #define time        out_time[axis][pos]
  #define val         out_val[axis][pos]

  for ( axis = AXES; axis--; )
  {
    // if nothing to output for current axis, go to the next axis
    if ( !time ) continue;

    // ticks decrement for the current output value
    --time;

    // output
    if ( !val ) // step low
    {
      if ( step_state )
      {
        step_state = GPIO_PIN_RESET;
        write_pin(STEP_OUT.PORT, STEP_OUT.PIN, GPIO_PIN_RESET);
        // TODO - TEST CODE - remove this code
        _TEST_axis_step_out_pos[axis] += _TEST_axis_dir_out_state[axis] ? -1 : 1;
      }
    }
    else if ( val & 1 ) // step high
    {
      if ( !step_state )
      {
        step_state = GPIO_PIN_SET;
        write_pin(STEP_OUT.PORT, STEP_OUT.PIN, GPIO_PIN_SET);
      }
    }
    else if ( val & 2 ) // dir low
    {
      if ( dir_state )
      {
        dir_state = GPIO_PIN_RESET;
        write_pin(DIR_OUT.PORT, DIR_OUT.PIN, GPIO_PIN_RESET);
        // TODO - TEST CODE - remove this code
        _TEST_axis_dir_out_state[axis] = GPIO_PIN_RESET;
      }
    }
    else // dir high
    {
      if ( !dir_state )
      {
        dir_state = GPIO_PIN_SET;
        write_pin(DIR_OUT.PORT, DIR_OUT.PIN, GPIO_PIN_SET);
        // TODO - TEST CODE - remove this code
        _TEST_axis_dir_out_state[axis] = GPIO_PIN_SET;
      }
    }

    // if no more ticks for the current output value, go to the next value
    if ( !time ) ++pos;
  }

  #undef time
  #undef val
  #undef cur
  #undef pos
  #undef STEP_OUT
  #undef DIR_OUT
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  global_error = 2;
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  global_error = 4;
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  global_error = 8;
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  global_error = 16;
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  out_buf_add_step(0);
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
  out_buf_add_step(1);
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
  out_buf_add_step(2);
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
  out_buf_add_step(3);
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
  out_buf_add_step(4);
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  static uint8_t axis;
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  #define cur  dir_input_state_cur[axis]
  #define prev dir_input_state_prev[axis]
  for ( axis = AXES; axis--; )
  {
    // get current state of a pin
    cur = read_pin(DIR_INPUTS[axis].PORT, DIR_INPUTS[axis].PIN);
    // if we have any changes
    if ( cur != prev ) out_buf_add_dir(axis, cur, prev);
    // save current state of a pin
    prev = cur;
  }
  #undef cur
  #undef prev
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  time_tick();
  output_tick();
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
