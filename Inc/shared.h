/*
 * shared.h
 *
 *  Created on: 24 џэт. 2017 у.
 *      Author: MX_Master
 */

#ifndef __SHARED_H
#define __SHARED_H




// number of using axes
#define AXES                    5 // max is 5
// output multiplier
#define MULT                    2

// more input frequency needs more SRAM
#define INPUT_MAX_FREQUENCY     20000 // Hz
// enable/disable input filtering
#define INPUT_FILTERING         1 // any boolean value (0,1,false,true)
// more time to wait for input needs more SRAM
#define SERVO_CYCLE_INTERVAL    5000 // us

// duty cycle of the output pulses
#define OUTPUT_STEP_DUTY_CYCLE  50 // %
// direction pre- and post delays
#define OUTPUT_DIR_DELAY_BEFORE 50 // us
#define OUTPUT_DIR_DELAY_AFTER  50 // us




struct PORT_PIN_type
{
  GPIO_TypeDef*   PORT;
  uint16_t        PIN;
};

#define INP_BUF_SIZE (SERVO_CYCLE_INTERVAL / (1000000 / INPUT_MAX_FREQUENCY)) // pulses
#define OUT_BUF_SIZE (INP_BUF_SIZE * 2) // pulse states




#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))




// input pins states (current and previous)
#ifdef _MAIN_C
  GPIO_PinState   step_input_state_cur[AXES]      = {0};
  GPIO_PinState   step_input_state_prev[AXES]     = {0};
  GPIO_PinState   dir_input_state_cur[AXES]       = {0};
  GPIO_PinState   dir_input_state_prev[AXES]      = {0};
#else
  extern GPIO_PinState   step_input_state_cur[AXES];
  extern GPIO_PinState   step_input_state_prev[AXES];
  extern GPIO_PinState   dir_input_state_cur[AXES];
  extern GPIO_PinState   dir_input_state_prev[AXES];
#endif

// output pins states
#ifdef _MAIN_C
  GPIO_PinState   step_output_state_cur[AXES]      = {0};
  GPIO_PinState   dir_output_state_cur[AXES]       = {0};
#else
  extern GPIO_PinState   step_output_state_cur[AXES];
  extern GPIO_PinState   dir_output_state_cur[AXES];
#endif



// input buffers
#ifdef _MAIN_C
  // "inp_val[axis][pos]" cell values legend:
  // -2 = STEP low, -1 = STEP high
  // 0 = do nothing (used as delay)
  // 1 = DIR low, 2 = DIR high
  int8_t          inp_val[AXES][INP_BUF_SIZE]   = {{0}}; // values of input steps/dirs
  uint16_t        inp_time[AXES][INP_BUF_SIZE]  = {{0}}; // timestamps of input steps/dirs
  uint16_t        inp_pos[AXES]                 = {0}; // position for the next input timestamp
  uint16_t        inp_last_period[AXES]         = {(1000000/INPUT_MAX_FREQUENCY)}; // time between last to pulses
#else
  extern int8_t   inp_val[AXES][INP_BUF_SIZE];
  extern uint16_t inp_time[AXES][INP_BUF_SIZE];
  extern uint16_t inp_pos[AXES];
  extern uint16_t inp_last_period[AXES];
#endif




// output buffers
#ifdef _MAIN_C
  // "out_val[axis][pos]" cell values legend:
  // -2 = STEP low, -1 = STEP high
  // 0 = do nothing (used as delay)
  // 1 = DIR low, 2 = DIR high
  int8_t          out_val[AXES][OUT_BUF_SIZE]         = {{0}};
  uint16_t        out_time[AXES][OUT_BUF_SIZE]        = {{0}};
  uint16_t        out_add_pos[AXES]               = {0};
  uint16_t        out_pos[AXES]                   = {0};
#else
  extern int8_t          out_val[AXES][OUT_BUF_SIZE];
  extern uint16_t        out_time[AXES][OUT_BUF_SIZE];
  extern uint16_t        out_add_pos[AXES];
  extern uint16_t        out_pos[AXES];
#endif




// servo cycle settings
#ifdef _MAIN_C
  uint32_t        servo_cycle_timestamp_us        = 0;
#else
  extern uint32_t        servo_cycle_timestamp_us;
#endif




// pinout
#ifdef _MAIN_C
  struct PORT_PIN_type STEP_INPUTS[5] = {
      {AXIS1_STEP_INPUT_GPIO_Port, AXIS1_STEP_INPUT_Pin},
      {AXIS2_STEP_INPUT_GPIO_Port, AXIS2_STEP_INPUT_Pin},
      {AXIS3_STEP_INPUT_GPIO_Port, AXIS3_STEP_INPUT_Pin},
      {AXIS4_STEP_INPUT_GPIO_Port, AXIS4_STEP_INPUT_Pin},
      {AXIS5_STEP_INPUT_GPIO_Port, AXIS5_STEP_INPUT_Pin}
  };
  uint8_t exti_9_5_step_pins[AXES]    = {0};
  uint8_t exti_15_10_step_pins[AXES]  = {0};
  uint8_t exti_9_5_step_pins_count    = 0;
  uint8_t exti_15_10_step_pins_count  = 0;

  struct PORT_PIN_type DIR_INPUTS[5] = {
      {AXIS1_DIR_INPUT_GPIO_Port, AXIS1_DIR_INPUT_Pin},
      {AXIS2_DIR_INPUT_GPIO_Port, AXIS2_DIR_INPUT_Pin},
      {AXIS3_DIR_INPUT_GPIO_Port, AXIS3_DIR_INPUT_Pin},
      {AXIS4_DIR_INPUT_GPIO_Port, AXIS4_DIR_INPUT_Pin},
      {AXIS5_DIR_INPUT_GPIO_Port, AXIS5_DIR_INPUT_Pin}
  };
  uint8_t exti_9_5_dir_pins[AXES]     = {0};
  uint8_t exti_15_10_dir_pins[AXES]   = {0};
  uint8_t exti_9_5_dir_pins_count     = 0;
  uint8_t exti_15_10_dir_pins_count   = 0;

  struct PORT_PIN_type STEP_OUTPUTS[5] = {
      {AXIS1_STEP_OUTPUT_GPIO_Port, AXIS1_STEP_OUTPUT_Pin},
      {AXIS2_STEP_OUTPUT_GPIO_Port, AXIS2_STEP_OUTPUT_Pin},
      {AXIS3_STEP_OUTPUT_GPIO_Port, AXIS3_STEP_OUTPUT_Pin},
      {AXIS4_STEP_OUTPUT_GPIO_Port, AXIS4_STEP_OUTPUT_Pin},
      {AXIS5_STEP_OUTPUT_GPIO_Port, AXIS5_STEP_OUTPUT_Pin}
  };
  struct PORT_PIN_type DIR_OUTPUTS[5] = {
      {AXIS1_DIR_OUTPUT_GPIO_Port, AXIS1_DIR_OUTPUT_Pin},
      {AXIS2_DIR_OUTPUT_GPIO_Port, AXIS2_DIR_OUTPUT_Pin},
      {AXIS3_DIR_OUTPUT_GPIO_Port, AXIS3_DIR_OUTPUT_Pin},
      {AXIS4_DIR_OUTPUT_GPIO_Port, AXIS4_DIR_OUTPUT_Pin},
      {AXIS5_DIR_OUTPUT_GPIO_Port, AXIS5_DIR_OUTPUT_Pin}
  };
#else
  extern struct PORT_PIN_type STEP_INPUTS[5];
  extern uint8_t  exti_9_5_step_pins[AXES];
  extern uint8_t  exti_15_10_step_pins[AXES];
  extern uint8_t  exti_9_5_step_pins_count;
  extern uint8_t  exti_15_10_step_pins_count;

  extern struct PORT_PIN_type DIR_INPUTS[5];
  extern uint8_t  exti_9_5_dir_pins[AXES];
  extern uint8_t  exti_15_10_dir_pins[AXES];
  extern uint8_t  exti_9_5_dir_pins_count;
  extern uint8_t  exti_15_10_dir_pins_count;

  extern struct PORT_PIN_type STEP_OUTPUTS[5];
  extern struct PORT_PIN_type DIR_OUTPUTS[5];
#endif




void init_ports_pins();
void input_IRQ_handler
(
    uint8_t step_pins[], uint8_t *step_pins_count,
    uint8_t dir_pins[],  uint8_t *dir_pins_count
);
void output_generator_step();
void servo_cycle_handler();
void process_input_value ( uint8_t *axis, int8_t value );
void process_input_buffers();




#endif
