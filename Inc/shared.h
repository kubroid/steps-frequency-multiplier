/*
 * shared.h
 *
 *  Created on: 24 jan, 2017
 *      Author: MX_Master
 */

#ifndef __SHARED_H
#define __SHARED_H




// THESE SETTINGS CAN BE FREELY MODIFIED

#define AXES                        5 // 1..5
#define MULT                        8 // output steps multiplier

// change this value to the maximim output frequency
// for you step/dir controller/generator
// EXAMPLE: Mach3 can generate up to 65 KHz of output pulses
//          so you must change THIS value to the 65000
#define INPUT_MAX_FREQUENCY         20000 // Hz

// direction change delays
#define OUTPUT_DIR_DELAY_BEFORE     50 // us
#define OUTPUT_DIR_DELAY_AFTER      50 // us

// max wait time between 2 input pulses
// uses for calculating input periods
#define INPUT_MAX_WAIT_TIME         1000 // us




// PLEASE, DON'T TOUCH THESE SETTINGS BELOW !!!

// values for calculations
#define OUTPUT_LOOP_FREQ  (2 * MULT * INPUT_MAX_FREQUENCY)

#define PULSE_WAIT_TIME   ((INPUT_MAX_WAIT_TIME * OUTPUT_LOOP_FREQ) / 1000000)

#define DIR_DELAY_BEFORE  ((OUTPUT_DIR_DELAY_BEFORE * OUTPUT_LOOP_FREQ) / 1000000)
#define DIR_DELAY_AFTER   ((OUTPUT_DIR_DELAY_AFTER  * OUTPUT_LOOP_FREQ) / 1000000)

#if PULSE_WAIT_TIME <= 0
  #undef  PULSE_WAIT_TIME
  #define PULSE_WAIT_TIME 1
#endif
#if DIR_DELAY_BEFORE <= 0
  #undef  DIR_DELAY_BEFORE
  #define DIR_DELAY_BEFORE 1
#endif
#if DIR_DELAY_AFTER <= 0
  #undef  DIR_DELAY_AFTER
  #define DIR_DELAY_AFTER 1
#endif




// redefining timer's max count
#ifdef TIM_COUNT
  #undef TIM_COUNT
  #define TIM_COUNT (RCC_MAX_FREQUENCY / (8 * MULT * INPUT_MAX_FREQUENCY) - 1)
#endif




// TODO - TEST CODE - remove this code
#ifdef _MAIN_C
  volatile int32_t        _TEST_axis_step_pos[AXES]       = {0};
  volatile GPIO_PinState  _TEST_axis_dir_state[AXES]      = {0};
  volatile int32_t        _TEST_axis_step_out_pos[AXES]   = {0};
  volatile GPIO_PinState  _TEST_axis_dir_out_state[AXES]  = {0};
#else
  extern volatile int32_t       _TEST_axis_step_pos[AXES];
  extern volatile GPIO_PinState _TEST_axis_dir_state[AXES];
  extern volatile int32_t       _TEST_axis_step_out_pos[AXES];
  extern volatile GPIO_PinState _TEST_axis_dir_out_state[AXES];
#endif




// var to store last error code
#ifdef _MAIN_C
  volatile uint8_t global_error = 0;
#else
  extern volatile uint8_t global_error;
#endif




// input pins states (current and previous)
#ifdef _MAIN_C
  volatile GPIO_PinState dir_input_state_cur[AXES]   = {0};
  volatile GPIO_PinState dir_input_state_prev[AXES]  = {0};
#else
  extern volatile GPIO_PinState dir_input_state_cur[AXES];
  extern volatile GPIO_PinState dir_input_state_prev[AXES];
#endif

// output pins states
#ifdef _MAIN_C
  volatile GPIO_PinState step_output_state_cur[AXES] = {0};
  volatile GPIO_PinState dir_output_state_cur[AXES]  = {0};
#else
  extern volatile GPIO_PinState step_output_state_cur[AXES];
  extern volatile GPIO_PinState dir_output_state_cur[AXES];
#endif




#ifdef _MAIN_C
  volatile uint16_t inp_last_time[AXES] = {0};
#else
  extern volatile uint16_t inp_last_time[AXES];
#endif




// input/output buffers
#define OUT_BUF_SIZE 256 // must be >= max value of out_add_pos[] or out_pos[]

#ifdef _MAIN_C
  // possible values for any cell of the out_val[axis][pos] :
  // 0 = step low, 1 = step high,
  // 2 - dir low,  4 = dir high
  volatile uint8_t  out_val[AXES][OUT_BUF_SIZE]   = {{0}};
  volatile uint16_t out_time[AXES][OUT_BUF_SIZE]  = {{0}};
  volatile uint8_t  out_add_pos[AXES]             = {0};
  volatile uint8_t  out_pos[AXES]                 = {0};
#else
  extern volatile uint8_t out_val[AXES][OUT_BUF_SIZE];
  extern volatile uint16_t out_time[AXES][OUT_BUF_SIZE];
  extern volatile uint8_t out_add_pos[AXES];
  extern volatile uint8_t out_pos[AXES];
#endif




// pinout
struct PORT_PIN_type
{
  GPIO_TypeDef*   PORT;
  uint16_t        PIN;
};

#ifdef _MAIN_C
  const struct PORT_PIN_type STEP_INPUTS[] = {
    {AXIS1_STEP_INPUT_GPIO_Port, AXIS1_STEP_INPUT_Pin},
    {AXIS2_STEP_INPUT_GPIO_Port, AXIS2_STEP_INPUT_Pin},
    {AXIS3_STEP_INPUT_GPIO_Port, AXIS3_STEP_INPUT_Pin},
    {AXIS4_STEP_INPUT_GPIO_Port, AXIS4_STEP_INPUT_Pin},
    {AXIS5_STEP_INPUT_GPIO_Port, AXIS5_STEP_INPUT_Pin}
  };
  const struct PORT_PIN_type DIR_INPUTS[] = {
    {AXIS1_DIR_INPUT_GPIO_Port, AXIS1_DIR_INPUT_Pin},
    {AXIS2_DIR_INPUT_GPIO_Port, AXIS2_DIR_INPUT_Pin},
    {AXIS3_DIR_INPUT_GPIO_Port, AXIS3_DIR_INPUT_Pin},
    {AXIS4_DIR_INPUT_GPIO_Port, AXIS4_DIR_INPUT_Pin},
    {AXIS5_DIR_INPUT_GPIO_Port, AXIS5_DIR_INPUT_Pin}
  };

  const struct PORT_PIN_type STEP_OUTPUTS[] = {
    {AXIS1_STEP_OUTPUT_GPIO_Port, AXIS1_STEP_OUTPUT_Pin},
    {AXIS2_STEP_OUTPUT_GPIO_Port, AXIS2_STEP_OUTPUT_Pin},
    {AXIS3_STEP_OUTPUT_GPIO_Port, AXIS3_STEP_OUTPUT_Pin},
    {AXIS4_STEP_OUTPUT_GPIO_Port, AXIS4_STEP_OUTPUT_Pin},
    {AXIS5_STEP_OUTPUT_GPIO_Port, AXIS5_STEP_OUTPUT_Pin}
  };
  const struct PORT_PIN_type DIR_OUTPUTS[] = {
    {AXIS1_DIR_OUTPUT_GPIO_Port, AXIS1_DIR_OUTPUT_Pin},
    {AXIS2_DIR_OUTPUT_GPIO_Port, AXIS2_DIR_OUTPUT_Pin},
    {AXIS3_DIR_OUTPUT_GPIO_Port, AXIS3_DIR_OUTPUT_Pin},
    {AXIS4_DIR_OUTPUT_GPIO_Port, AXIS4_DIR_OUTPUT_Pin},
    {AXIS5_DIR_OUTPUT_GPIO_Port, AXIS5_DIR_OUTPUT_Pin}
  };
#else
  extern const struct PORT_PIN_type STEP_INPUTS[];
  extern const struct PORT_PIN_type DIR_INPUTS[];

  extern const struct PORT_PIN_type STEP_OUTPUTS[];
  extern const struct PORT_PIN_type DIR_OUTPUTS[];
#endif




// atomic work with pins
GPIO_PinState static inline read_pin ( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
  return (GPIOx->IDR & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void static inline write_pin ( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState )
{
  if ( PinState ) GPIOx->BSRR = GPIO_Pin;
  else            GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
}
void static inline toggle_pin ( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
  GPIOx->ODR ^= GPIO_Pin;
}




#endif
