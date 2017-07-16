/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;




// you can edit these
#define AXIS_CNT            4 // 1..5
#define OUT_STEP_MULT       8 // 1..84


// don't touch these
#define MAX_PERIOD_MULT     10
#define MAX_WAIT_TIME       65 // ms
#define MIN_WAIT_TIME       3 // ms
#define OUT_BUF_SIZE        256
#define HCLK_HZ             72000000
#define HCLK_MHZ            (HCLK_HZ/1000000)
#define SYSTICK_CLK         (HCLK_HZ/1000)
#define SYSTIME_DIV_US      (HCLK_HZ/1000000)


struct AXIS_TIM_t
{
  TIM_HandleTypeDef*  h;
  uint32_t            uwChannel;
  uint32_t            uwDMAID;
  uint32_t            uwDMASRC;
};
const struct AXIS_TIM_t aTim[] = {
  {&htim1, TIM_CHANNEL_1, TIM_DMA_ID_CC1, TIM_DMA_CC1},
  {&htim2, TIM_CHANNEL_4, TIM_DMA_ID_CC4, TIM_DMA_CC4},
  {&htim3, TIM_CHANNEL_1, TIM_DMA_ID_CC1, TIM_DMA_CC1},
  {&htim4, TIM_CHANNEL_1, TIM_DMA_ID_CC1, TIM_DMA_CC1},
};

volatile uint32_t auwPresc[AXIS_CNT] = {0};
volatile uint32_t auwPeriod[AXIS_CNT] = {0};
volatile uint64_t aulTime[AXIS_CNT] = {0};

volatile uint8_t auqMoving[AXIS_CNT] = {0};
volatile uint16_t auhWaiting[AXIS_CNT] = {0};
volatile uint8_t auq1stStep[AXIS_CNT] = {0};
volatile uint8_t auqOutputOn[AXIS_CNT] = {0};

volatile uint8_t auqSteps[AXIS_CNT] = {0};

uint32_t auhOCDMAVal_2 = 0;

struct OUT_BUF_t
{
  uint8_t   uqType;
  uint16_t  uhTime;
  uint16_t  uhCount;
};
volatile struct OUT_BUF_t aBuf[AXIS_CNT][OUT_BUF_SIZE] = {{{0}}};
volatile uint8_t auqBufOutPos[AXIS_CNT] = {0};
volatile uint8_t auqBufAddPos[AXIS_CNT] = {0};

struct PORT_PIN_t
{
  GPIO_TypeDef*   PORT;
  uint16_t        PIN;
};
const struct PORT_PIN_t saAxisStepInputs[] = {
  {AXIS1_STEP_INPUT_GPIO_Port, AXIS1_STEP_INPUT_Pin},
  {AXIS2_STEP_INPUT_GPIO_Port, AXIS2_STEP_INPUT_Pin},
  {AXIS3_STEP_INPUT_GPIO_Port, AXIS3_STEP_INPUT_Pin},
  {AXIS4_STEP_INPUT_GPIO_Port, AXIS4_STEP_INPUT_Pin},
};
const struct PORT_PIN_t saAxisDirInputs[] = {
  {AXIS1_DIR_INPUT_GPIO_Port, AXIS1_DIR_INPUT_Pin},
  {AXIS2_DIR_INPUT_GPIO_Port, AXIS2_DIR_INPUT_Pin},
  {AXIS3_DIR_INPUT_GPIO_Port, AXIS3_DIR_INPUT_Pin},
  {AXIS4_DIR_INPUT_GPIO_Port, AXIS4_DIR_INPUT_Pin},
};
const struct PORT_PIN_t saAxisStepOutputs[] = {
  {AXIS1_STEP_OUTPUT_GPIO_Port, AXIS1_STEP_OUTPUT_Pin},
  {AXIS2_STEP_OUTPUT_GPIO_Port, AXIS2_STEP_OUTPUT_Pin},
  {AXIS3_STEP_OUTPUT_GPIO_Port, AXIS3_STEP_OUTPUT_Pin},
  {AXIS4_STEP_OUTPUT_GPIO_Port, AXIS4_STEP_OUTPUT_Pin},
};
const struct PORT_PIN_t saAxisDirOutputs[] = {
  {AXIS1_DIR_OUTPUT_GPIO_Port, AXIS1_DIR_OUTPUT_Pin},
  {AXIS2_DIR_OUTPUT_GPIO_Port, AXIS2_DIR_OUTPUT_Pin},
  {AXIS3_DIR_OUTPUT_GPIO_Port, AXIS3_DIR_OUTPUT_Pin},
  {AXIS4_DIR_OUTPUT_GPIO_Port, AXIS4_DIR_OUTPUT_Pin},
};




#define TIM_DMA_H     aTim[axis].h->hdma[aTim[axis].uwDMAID]
#define TIM_DMA       aTim[axis].h->hdma[aTim[axis].uwDMAID]->Instance
#define TIM_H         aTim[axis].h
#define TIM_CH        aTim[axis].uwChannel
#define TIM_DMA_SRC   aTim[axis].uwDMASRC
#define TIM           aTim[axis].h->Instance
#define BUF_IN_TYPE   aBuf[axis][BUF_IN_POS].uqType
#define BUF_IN_TIME   aBuf[axis][BUF_IN_POS].uhTime
#define BUF_IN_CNT    aBuf[axis][BUF_IN_POS].uhCount
#define BUF_IN_POS    auqBufAddPos[axis]
#define BUF_OUT_TYPE  aBuf[axis][BUF_OUT_POS].uqType
#define BUF_OUT_TIME  aBuf[axis][BUF_OUT_POS].uhTime
#define BUF_OUT_CNT   aBuf[axis][BUF_OUT_POS].uhCount
#define BUF_OUT_POS   auqBufOutPos[axis]
#define DIR_OUT       saAxisDirOutputs[axis]
#define DIR_INP       saAxisDirInputs[axis]
#define STEP_OUT      saAxisStepOutputs[axis]
#define STEP_INP      saAxisStepInputs[axis]




void static inline start_output_2(uint8_t axis);
void static inline stop_output_2(uint8_t axis);
uint8_t static inline output(uint8_t axis);




// --- TOOLS ---

void static inline add2buf_step_2(uint8_t axis, uint16_t time, uint16_t count)
{
  BUF_IN_TYPE = 0; // 0 = STEP
  BUF_IN_TIME = time;
  BUF_IN_CNT = count;
  ++BUF_IN_POS;

  // start output if needed
  if ( !output(axis) ) start_output_2(axis);
}

void static inline add2buf_dir(uint8_t axis, GPIO_PinState dir)
{
  BUF_IN_TYPE = 1; // 1 = DIR
  BUF_IN_TIME = 1 + (uint16_t)dir;
  ++BUF_IN_POS;

  // start output if needed
  if ( !output(axis) ) start_output_2(axis);
}

void static inline goto_next_out_pos(uint8_t axis)
{
  BUF_OUT_TIME = 0;
  ++BUF_OUT_POS;
}

uint16_t static inline need2output(uint8_t axis)
{
  return BUF_OUT_TIME;
}

uint8_t static inline need2output_dir(uint8_t axis)
{
  return BUF_OUT_TYPE;
}

void static inline tim_update(uint8_t axis)
{
  TIM->EGR |= TIM_EGR_UG; // update event
  TIM->SR  &= ~TIM_SR_UIF; // reset update flag
}

// get timestamp in US since system startup
// value is overloaded after 49.71 days (256*256*256*256 ms)
uint64_t static inline time_us()
{
  static uint64_t t[2] = {0};

  t[0] = t[1];
  t[1] = HAL_GetTick()*1000 + (SYSTICK_CLK - SysTick->VAL)/SYSTIME_DIV_US;

  return t[0] < t[1] ?
    t[1] :
    t[1] + 1000 ;
}




// --- SETUP ---

// setup all out timers
void setup_out_timers_2()
{
  for ( uint8_t axis = AXIS_CNT; axis--; )
  {
    // update period, compare value
    __HAL_TIM_SET_AUTORELOAD(TIM_H, 1);
    __HAL_TIM_SET_COMPARE(TIM_H, TIM_CH, 1);
    // generate an update event to apply timer's data
    tim_update(axis);

    /* Disable the peripheral */
    __HAL_DMA_DISABLE(TIM_DMA_H);
    /* Configure DMA Channel data length */
    TIM_DMA->CPAR = (uint32_t)&(TIM->CNT);
    /* Configure DMA Channel source address */
    TIM_DMA->CMAR = (uint32_t)&auhOCDMAVal_2;
    /* Enable the transfer complete interrupt */
    __HAL_DMA_ENABLE_IT(TIM_DMA_H, DMA_IT_TC);

    /* Enable the TIM Capture/Compare DMA request */
    __HAL_TIM_ENABLE_DMA(TIM_H, TIM_DMA_SRC);
    /* Enable the Output compare channel */
    TIM_CCxChannelCmd(TIM, TIM_CH, TIM_CCx_ENABLE);

    /* Enable the main output */
    if(IS_TIM_BREAK_INSTANCE(TIM) != RESET)
    {
      __HAL_TIM_MOE_ENABLE(TIM_H);
    }
  }
}




// --- OUTPUT PROCESSING ---

// start output for selected axis
void static inline start_output_2(uint8_t axis)
{
  // set the prescaler
  __HAL_TIM_SET_PRESCALER(TIM_H, HCLK_MHZ * BUF_OUT_TIME / (2*OUT_STEP_MULT*BUF_OUT_CNT) - 1);
  // generate an update event to apply the prescaler
  tim_update(axis);

  /* Configure DMA Channel data length */
  TIM_DMA_H->Instance->CNDTR = (2*OUT_STEP_MULT*BUF_OUT_CNT - 1);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(TIM_H);
  __HAL_DMA_ENABLE(TIM_DMA_H);

  // output is enabled
  auqOutputOn[axis] = 1;
}

// stop output for selected axis
void static inline stop_output_2(uint8_t axis)
{
  // disable timer
  TIM->CR1 &= ~(TIM_CR1_CEN);

  // disable timer's dma
  __HAL_DMA_DISABLE(TIM_DMA_H);

  // +1 to the current out pos
  goto_next_out_pos(axis);

  // output is disabled
  auqOutputOn[axis] = 0;

  // start output if needed
  if ( need2output(axis) ) start_output_2(axis);
}

// get output enabled flag
uint8_t static inline output(uint8_t axis)
{
  // return output state
  return auqOutputOn[axis];
}




// --- INPUT PROCESSING ---

// on EXTI 4-0 input
void static inline process_input_step(uint8_t axis)
{
#if 0
  static uint64_t t = 0;

  __HAL_GPIO_EXTI_CLEAR_IT(STEP_INP.PIN);

  // more steps to output
  ++auqSteps[axis];

  // if axis isn't moving
  if ( !auqMoving[axis] )
  {
    aulTime[axis] = time_us(); // save step time
    auqMoving[axis] = 1; // axis is moving now
    auq1stStep[axis] = 1; // it's 1st step after axis move start
    auhWaiting[axis] = MAX_WAIT_TIME; // set max wait time for the next step
  }
  else // axis is moving
  {
    t = time_us(); // get step time
    auwPeriod[axis] = t - aulTime[axis]; // calculate the period between 2 input steps
    aulTime[axis] = t; // save step time

    // if now we have just 2 steps after axis move start
    if ( auq1stStep[axis] )
    {
      auq1stStep[axis] = 0; // "1st axis step" flag reset
      // add 2 steps to the buffer
      add2buf_step(axis, auwPeriod[axis]/2);
      add2buf_step(axis, auwPeriod[axis]/2);
    }
    else // we have more than 2 steps after axis move start
    {
      add2buf_step(axis, auwPeriod[axis]); // add step to the buffer
    }

    // set new waiting time for the next step
    auhWaiting[axis] = MIN_WAIT_TIME + (auwPeriod[axis] * MAX_PERIOD_MULT)/1000;
    if ( auhWaiting[axis] > MAX_WAIT_TIME ) auhWaiting[axis] = MAX_WAIT_TIME;
  }
#endif
}

// on EXTI 5-9 input
void static inline process_input_dirs()
{
#if 0
  static uint8_t axis = 0;

  for ( axis = AXIS_CNT; axis--; )
  {
    if ( __HAL_GPIO_EXTI_GET_IT(DIR_INP.PIN) )
    {
      __HAL_GPIO_EXTI_CLEAR_IT(DIR_INP.PIN);

      // if we have just 1 step while axis was moving
      if ( auq1stStep[axis] )
      {
        auq1stStep[axis] = 0; // "1st axis step" flag reset
        add2buf_step(axis, time_us() - aulTime[axis]); // add step to the buffer
      }

      auqMoving[axis] = 0; // axis isn't moving now
      auhWaiting[axis] = 0; // don't wait for the next step

      // add DIR to the output buffer
      add2buf_dir(axis, HAL_GPIO_ReadPin(DIR_INP.PORT, DIR_INP.PIN));
    }
  }
#endif
}

// on sys tick (every 1000us)
void static inline process_sys_tick()
{
#if 0
  static uint8_t axis = 0;

  for ( axis = AXIS_CNT; axis--; )
  {
    // if axis is moving
    if ( auqMoving[axis] )
    {
      // if we waiting for the next step
      if ( auhWaiting[axis] )
      {
        --auhWaiting[axis]; // decrease waiting time

        // if wait time is over
        if ( !auhWaiting[axis] )
        {
          // if we have just 1 step while axis was moving
          if ( auq1stStep[axis] )
          {
            auq1stStep[axis] = 0; // "1st axis step" flag reset
            add2buf_step(axis, time_us() - aulTime[axis]); // add step to the buffer
          }

          auqMoving[axis] = 0; // axis isn't moving now
          auhWaiting[axis] = 0; // don't wait for the next step
        }
      }
    }
    // if axis is not moving
    else
    {
      // force axis DIR input/output sync
      HAL_GPIO_WritePin(
        DIR_OUT.PORT, DIR_OUT.PIN,
        HAL_GPIO_ReadPin(DIR_INP.PORT, DIR_INP.PIN)
      );
    }
  }
#endif
}

// on sys tick (every 1000us)
void static inline process_sys_tick_2()
{
  // do it every second
  if ( !(HAL_GetTick() % 1000) )
  {
    add2buf_step_2(0, 50, 16);
  }
}




#undef TIM_DMA_H
#undef TIM_DMA
#undef TIM_H
#undef TIM_CH
#undef TIM_DMA_SRC
#undef TIM
#undef BUF_IN_TYPE
#undef BUF_IN_TIME
#undef BUF_IN_CNT
#undef BUF_IN_POS
#undef BUF_OUT_TYPE
#undef BUF_OUT_TIME
#undef BUF_OUT_CNT
#undef BUF_OUT_POS
#undef DIR_OUT
#undef DIR_INP
#undef STEP_OUT
#undef STEP_INP
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim2_ch2_ch4;
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
extern DMA_HandleTypeDef hdma_tim4_ch1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

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

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

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
  process_sys_tick_2();
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
#if 0
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
#endif
  process_input_step(0);
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
#if 0
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
#endif
  process_input_step(1);
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
#if 0
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
#endif
  process_input_step(2);
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
#if 0
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
#endif
  process_input_step(3);
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  /* Clear the transfer complete flag */
  __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch1, DMA_FLAG_TC2);

  stop_output_2(3);

#if 0
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
#endif

  UNUSED(hdma_tim4_ch1);
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel2 global interrupt.
*/
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
  /* Clear the transfer complete flag */
  __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch1, DMA_FLAG_TC2);

  stop_output_2(0);

#if 0
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_ch1);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
#endif

  UNUSED(hdma_tim1_ch1);
  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel6 global interrupt.
*/
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
  /* Clear the transfer complete flag */
  __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch1, DMA_FLAG_TC2);

  stop_output_2(2);

#if 0
  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch1_trig);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */
#endif

  UNUSED(hdma_tim3_ch1_trig);
  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
  /* Clear the transfer complete flag */
  __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch1, DMA_FLAG_TC2);

  stop_output_2(1);

#if 0
  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch2_ch4);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */
#endif

  UNUSED(hdma_tim2_ch2_ch4);
  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
#if 0
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
#endif
  process_input_dirs();
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
