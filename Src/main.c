/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim3_ch3;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim8_ch4_trig_com;

/* USER CODE BEGIN PV */
// you can edit these
#define AXIS_CNT            5 // 1..5
#define OUT_STEP_MULT       10 // 1..84


// don't touch these
#define MAX_PERIOD_MULT     10
#define MAX_WAIT_TIME       65 // ms
#define MIN_WAIT_TIME       3 // ms
#define OUT_BUF_SIZE        256


struct AXIS_TIM_t
{
  TIM_HandleTypeDef*  h;
  uint32_t            uwChannel;
  uint32_t            uwDMAID;
  uint32_t            uwDMASRC;
};
const struct AXIS_TIM_t aTim[] = {
  {&htim1, TIM_CHANNEL_1, TIM_DMA_ID_CC1, TIM_DMA_CC1},
  {&htim3, TIM_CHANNEL_3, TIM_DMA_ID_CC3, TIM_DMA_CC3},
  {&htim4, TIM_CHANNEL_1, TIM_DMA_ID_CC1, TIM_DMA_CC1},
  {&htim5, TIM_CHANNEL_1, TIM_DMA_ID_CC1, TIM_DMA_CC1},
  {&htim8, TIM_CHANNEL_4, TIM_DMA_ID_CC4, TIM_DMA_CC4}
};

volatile const uint8_t auqPrescDiv[] = {1,2,2,2,1};
volatile uint32_t auwPresc[AXIS_CNT] = {0};

volatile uint32_t auwPeriod[AXIS_CNT] = {0};
volatile uint64_t aulTime[AXIS_CNT] = {0};

volatile uint8_t auqMoving[AXIS_CNT] = {0};
volatile uint16_t auhWaiting[AXIS_CNT] = {0};
volatile uint8_t auq1stStep[AXIS_CNT] = {0};
volatile uint8_t auqOutputOn[AXIS_CNT] = {0};

volatile uint8_t auqSteps[AXIS_CNT] = {0};

extern volatile uint32_t uwTick;
volatile uint32_t uwSysTickClk = 0;
volatile uint32_t uwSysTimeDivUS = 0;

uint16_t auhOCDMAVal[2*OUT_STEP_MULT + 1] = {0};

struct OUT_BUF_t
{
  uint8_t   uqType;
  uint16_t  uhTime;
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
  {AXIS5_STEP_INPUT_GPIO_Port, AXIS5_STEP_INPUT_Pin}
};
const struct PORT_PIN_t saAxisDirInputs[] = {
  {AXIS1_DIR_INPUT_GPIO_Port, AXIS1_DIR_INPUT_Pin},
  {AXIS2_DIR_INPUT_GPIO_Port, AXIS2_DIR_INPUT_Pin},
  {AXIS3_DIR_INPUT_GPIO_Port, AXIS3_DIR_INPUT_Pin},
  {AXIS4_DIR_INPUT_GPIO_Port, AXIS4_DIR_INPUT_Pin},
  {AXIS5_DIR_INPUT_GPIO_Port, AXIS5_DIR_INPUT_Pin}
};
const struct PORT_PIN_t saAxisStepOutputs[] = {
  {AXIS1_STEP_OUTPUT_GPIO_Port, AXIS1_STEP_OUTPUT_Pin},
  {AXIS2_STEP_OUTPUT_GPIO_Port, AXIS2_STEP_OUTPUT_Pin},
  {AXIS3_STEP_OUTPUT_GPIO_Port, AXIS3_STEP_OUTPUT_Pin},
  {AXIS4_STEP_OUTPUT_GPIO_Port, AXIS4_STEP_OUTPUT_Pin},
  {AXIS5_STEP_OUTPUT_GPIO_Port, AXIS5_STEP_OUTPUT_Pin}
};
const struct PORT_PIN_t saAxisDirOutputs[] = {
  {AXIS1_DIR_OUTPUT_GPIO_Port, AXIS1_DIR_OUTPUT_Pin},
  {AXIS2_DIR_OUTPUT_GPIO_Port, AXIS2_DIR_OUTPUT_Pin},
  {AXIS3_DIR_OUTPUT_GPIO_Port, AXIS3_DIR_OUTPUT_Pin},
  {AXIS4_DIR_OUTPUT_GPIO_Port, AXIS4_DIR_OUTPUT_Pin},
  {AXIS5_DIR_OUTPUT_GPIO_Port, AXIS5_DIR_OUTPUT_Pin}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_NVIC_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void static inline start_output(uint8_t axis);
void static inline stop_output(uint8_t axis);
uint8_t static inline output(uint8_t axis);
void static inline setup_axis_timer(uint8_t axis);
void static inline start_axis_timer(uint8_t axis);
void static inline stop_axis_timer(uint8_t axis);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// useful tools
void static inline add2buf_step(uint8_t axis, uint16_t time)
{
  aBuf[axis][auqBufAddPos[axis]].uqType = 0;
  aBuf[axis][auqBufAddPos[axis]].uhTime = time;
  ++auqBufAddPos[axis];

  // start output if needed
  if ( !output(axis) ) start_output(axis);
}

void static inline add2buf_dir(uint8_t axis)
{
  aBuf[axis][auqBufAddPos[axis]].uqType = 1;
  aBuf[axis][auqBufAddPos[axis]].uhTime = 1;
  ++auqBufAddPos[axis];

  // start output if needed
  if ( !output(axis) ) start_output(axis);
}

void static inline goto_next_out_pos(uint8_t axis)
{
  aBuf[axis][auqBufOutPos[axis]].uhTime = 0;
  ++auqBufOutPos[axis];
}

uint16_t static inline need2output(uint8_t axis)
{
  return aBuf[axis][auqBufOutPos[axis]].uhTime;
}

uint8_t static inline need2output_dir(uint8_t axis)
{
  return aBuf[axis][auqBufOutPos[axis]].uqType;
}

uint16_t static inline out_presc(uint8_t axis)
{
  auwPresc[axis] = aBuf[axis][auqBufOutPos[axis]].uhTime / auqPrescDiv[axis];

  return auwPresc[axis] > auqSteps[axis] ?
    auwPresc[axis] - auqSteps[axis] :
    0 ;
}

void static inline tim_update(uint8_t axis)
{
  aTim[axis].h->Instance->EGR |= TIM_EGR_UG; // update event
  aTim[axis].h->Instance->SR  &= ~TIM_SR_UIF; // reset update flag
}




// setup DMA array values
void static inline setup_OC_DMA_array()
{
  uint8_t uqPeriod = HAL_RCC_GetHCLKFreq()/1000000;
  uint8_t uqWidth = uqPeriod / (OUT_STEP_MULT*2);
  uint8_t uqWidthDivLost = uqPeriod % (OUT_STEP_MULT*2);

  for ( uint8_t i = 0, c = 0; i < (OUT_STEP_MULT*2 - 1); ++i )
  {
    if ( uqWidthDivLost )
    {
      c += uqWidth + 1;
      --uqWidthDivLost;
    }
    else
    {
      c += uqWidth;
    }

    auhOCDMAVal[i] = c;
  }

  auhOCDMAVal[OUT_STEP_MULT*2 - 1] = uqPeriod - 1;
  auhOCDMAVal[OUT_STEP_MULT*2] = 0xFFFF;
}

// set axis DIR output pins same as inputs
void static inline setup_out_DIR_pins()
{
  for ( uint8_t axis = AXIS_CNT; axis--; )
  {
    // out pin state = input pin state
    HAL_GPIO_WritePin(
      saAxisDirOutputs[axis].PORT, saAxisDirOutputs[axis].PIN,
      HAL_GPIO_ReadPin(saAxisDirInputs[axis].PORT, saAxisDirInputs[axis].PIN)
    );
  }
}

// setup all out timers
void static inline setup_out_timers()
{
  for ( uint8_t axis = AXIS_CNT; axis--; ) setup_axis_timer(axis);
}

// setup counter data
void static inline setup_counter()
{
  uwSysTickClk = HAL_RCC_GetHCLKFreq()/1000;
  uwSysTimeDivUS = HAL_RCC_GetHCLKFreq()/1000000;
}




// get timestamp in US since system startup
// value is overloaded after 49.71 days (256*256*256*256 ms)
uint64_t static inline time_us()
{
  static uint64_t t[2] = {0};

  t[0] = t[1];
  t[1] = uwTick*1000 + (uwSysTickClk - SysTick->VAL)/uwSysTimeDivUS;

  return t[0] < t[1] ?
    t[1] :
    t[1] + 1000 ;
}




// start output for selected axis
void static inline start_output(uint8_t axis)
{
  // if we need just a DIR change
  while ( need2output(axis) && need2output_dir(axis) )
  {
    HAL_GPIO_TogglePin(saAxisDirOutputs[axis].PORT, saAxisDirOutputs[axis].PIN);
    goto_next_out_pos(axis);
  }

  // exit if nothing to output
  if ( !need2output(axis) ) return;

#if 1
  // set timer's data
  __HAL_TIM_SET_PRESCALER(aTim[axis].h, out_presc(axis));
  __HAL_TIM_SET_COMPARE(aTim[axis].h, aTim[axis].uwChannel, auhOCDMAVal[0]);
  // generate an update event to apply timer's data
  tim_update(axis);

  // start output timer in OC+DMA mode
  HAL_TIM_OC_Start_DMA(
    aTim[axis].h, aTim[axis].uwChannel,
    ((uint32_t*)&auhOCDMAVal[1]), (OUT_STEP_MULT*2 - 1)
  );
#endif
#if 0
  // start output generation
  start_axis_timer(axis);
#endif
  // output is enabled
  auqOutputOn[axis] = 1;
}

// stop output for selected axis
void static inline stop_output(uint8_t axis)
{
  if ( !auqOutputOn[axis] ) return;

#if 1
  // stop OC
  __HAL_TIM_SET_COMPARE(aTim[axis].h, aTim[axis].uwChannel, 0xFFFF);
  // stop output timer in OC+DMA mode
  HAL_TIM_OC_Stop_DMA(aTim[axis].h, aTim[axis].uwChannel);
#endif
#if 0
  // start output generation
  stop_axis_timer(axis);
#endif
  // +1 to the current out pos
  goto_next_out_pos(axis);
  // output is disabled
  auqOutputOn[axis] = 0;
  // less steps to output
  --auqSteps[axis];
  // start output if needed
  if ( need2output(axis) ) start_output(axis);
}

// get output enabled flag
uint8_t static inline output(uint8_t axis)
{
  // return output state
  return auqOutputOn[axis];
}




// on EXTI 4-0 input
void process_input_step(uint8_t axis)
{
  static uint64_t t = 0;

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
}

// on EXTI 5-9 input
void process_input_dirs()
{
  static uint8_t axis = 0;

  for ( axis = AXIS_CNT; axis--; )
  {
    if ( __HAL_GPIO_EXTI_GET_IT(saAxisDirInputs[axis].PIN) )
    {
      __HAL_GPIO_EXTI_CLEAR_IT(saAxisDirInputs[axis].PIN);

      // if we have just 1 step while axis was moving
      if ( auq1stStep[axis] )
      {
        auq1stStep[axis] = 0; // "1st axis step" flag reset
        add2buf_step(axis, time_us() - aulTime[axis]); // add step to the buffer
      }

      auqMoving[axis] = 0; // axis isn't moving now
      auhWaiting[axis] = 0; // don't wait for the next step

      // add DIR to the output buffer
      add2buf_dir(axis);
    }
  }
}

// on axis TIM update
void on_axis_tim_update(uint8_t axis)
{
  /* TIM Update event */
  if ( __HAL_TIM_GET_FLAG(aTim[axis].h, TIM_FLAG_UPDATE) )
  {
    if ( __HAL_TIM_GET_IT_SOURCE(aTim[axis].h, TIM_IT_UPDATE) )
    {
      // reset update flag
      __HAL_TIM_CLEAR_IT(aTim[axis].h, TIM_IT_UPDATE);

      // we have finished some output
      stop_output(axis);
    }
  }
}

// on sys tick (every 1000us)
void process_sys_tick()
{
  static uint8_t axis = 0;

  for ( axis = AXIS_CNT; axis--; )
  {
    // if we waiting for the next step
    if ( auqMoving[axis] && auhWaiting[axis] )
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
}




#define TIM_DMA_H   aTim[axis].h->hdma[aTim[axis].uwDMAID]
#define TIM_DMA     aTim[axis].h->hdma[aTim[axis].uwDMAID]->Instance
#define TIM_H       aTim[axis].h
#define TIM_CH      aTim[axis].uwChannel
#define TIM         aTim[axis].h->Instance

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


void static inline setup_axis_timer(uint8_t axis)
{
  // get out timer period
  uint16_t uhPeriod = HAL_RCC_GetHCLKFreq()/1000000 - 1;
  // Enable the TIM Update interrupt
  __HAL_TIM_ENABLE_IT(TIM_H, TIM_IT_UPDATE);
  // set default period
  __HAL_TIM_SET_AUTORELOAD(TIM_H, uhPeriod);
  // generate an update event to reload the Period
  tim_update(axis);

#if 0
  /* Set the DMA Period elapsed callback */
  TIM_DMA_H->XferCpltCallback = TIM_DMADelayPulseCplt;
  /* Set the DMA error callback */
  TIM_DMA_H->XferErrorCallback = TIM_DMAError ;


  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers*)TIM_DMA_H->StreamBaseAddress;
  /* Clear all interrupt flags at correct offset within the register */
  regs->IFCR = 0x3FU << TIM_DMA_H->StreamIndex;
  /* Enable Common interrupts*/
  TIM_DMA->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
  TIM_DMA->FCR |= DMA_IT_FE;


  /* Clear DBM bit */
  TIM_DMA->CR &= (uint32_t)(~DMA_SxCR_DBM);
  /* Configure DMA Stream data length */
  TIM_DMA->NDTR = (2*OUT_STEP_MULT);
  /* Configure DMA Stream source address */
  TIM_DMA->PAR = (uint32_t)&auhOCDMAVal[1];
  /* Configure DMA Stream destination address */
  switch ( TIM_CH ) {
    case TIM_CHANNEL_1: TIM_DMA->M0AR = (uint32_t)&(TIM->CCR1); break;
    case TIM_CHANNEL_2: TIM_DMA->M0AR = (uint32_t)&(TIM->CCR2); break;
    case TIM_CHANNEL_3: TIM_DMA->M0AR = (uint32_t)&(TIM->CCR3); break;
    case TIM_CHANNEL_4: TIM_DMA->M0AR = (uint32_t)&(TIM->CCR4); break;
  }


  /* Enable the Peripheral */
  __HAL_DMA_ENABLE(TIM_DMA_H);
#endif
}

void static inline start_axis_timer(uint8_t axis)
{
  // set timer's data
  __HAL_TIM_SET_PRESCALER(TIM_H, out_presc(axis));
  __HAL_TIM_SET_COMPARE(TIM_H, TIM_CH, auhOCDMAVal[0]);
  // generate an update event to apply timer's data
  tim_update(axis);


  /* Enable the TIM Capture/Compare DMA request */
  __HAL_TIM_ENABLE_DMA(TIM_H, aTim[axis].uwDMASRC);


  /* Enable the Output compare channel */
  TIM_CCxChannelCmd(TIM, TIM_CH, TIM_CCx_ENABLE);


  /* Enable the main output */
  if ( IS_TIM_ADVANCED_INSTANCE(TIM) ) __HAL_TIM_MOE_ENABLE(TIM_H);
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(TIM_H);
}

void static inline stop_axis_timer(uint8_t axis)
{
  /* Disable the TIM Capture/Compare interrupt */
  __HAL_TIM_DISABLE_DMA(TIM_H, aTim[axis].uwDMASRC);


  /* Disable the Output compare channel */
  TIM_CCxChannelCmd(TIM, TIM_CH, TIM_CCx_DISABLE);


  /* Disable the main output */
  if ( IS_TIM_ADVANCED_INSTANCE(TIM) ) __HAL_TIM_MOE_DISABLE(TIM_H);
  /* Disable the Peripheral */
  __HAL_TIM_DISABLE(TIM_H);
}


#undef TIM_DMA_H
#undef TIM_DMA
#undef TIM_H
#undef TIM_CH
#undef TIM
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  setup_counter();
  setup_OC_DMA_array();
  setup_out_DIR_pins();
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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
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
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM8_UP_TIM13_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (OUT_TIM_BASE_PRESCALER/2);
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = OUT_TIM_BASE_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  htim4.Init.Prescaler = (OUT_TIM_BASE_PRESCALER/2);
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = OUT_TIM_BASE_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  htim5.Init.Prescaler = (OUT_TIM_BASE_PRESCALER/2);
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = OUT_TIM_BASE_PERIOD;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = OUT_TIM_BASE_PRESCALER;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = OUT_TIM_BASE_PERIOD;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AXIS1_DIR_OUTPUT_Pin|AXIS2_DIR_OUTPUT_Pin|AXIS3_DIR_OUTPUT_Pin|AXIS5_DIR_OUTPUT_Pin 
                          |AXIS4_DIR_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AXIS4_STEP_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS4_STEP_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(AXIS4_STEP_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AXIS5_STEP_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS5_STEP_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(AXIS5_STEP_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS1_DIR_INPUT_Pin AXIS4_DIR_INPUT_Pin AXIS5_DIR_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_DIR_INPUT_Pin|AXIS4_DIR_INPUT_Pin|AXIS5_DIR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS1_STEP_INPUT_Pin AXIS2_STEP_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_STEP_INPUT_Pin|AXIS2_STEP_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS1_DIR_OUTPUT_Pin AXIS2_DIR_OUTPUT_Pin AXIS3_DIR_OUTPUT_Pin AXIS5_DIR_OUTPUT_Pin 
                           AXIS4_DIR_OUTPUT_Pin */
  GPIO_InitStruct.Pin = AXIS1_DIR_OUTPUT_Pin|AXIS2_DIR_OUTPUT_Pin|AXIS3_DIR_OUTPUT_Pin|AXIS5_DIR_OUTPUT_Pin 
                          |AXIS4_DIR_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AXIS2_DIR_INPUT_Pin AXIS3_DIR_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS2_DIR_INPUT_Pin|AXIS3_DIR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : AXIS3_STEP_INPUT_Pin */
  GPIO_InitStruct.Pin = AXIS3_STEP_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(AXIS3_STEP_INPUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
