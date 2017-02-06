# STEP / DIRECTION multiplier and filter
This is a simple step/dir multiplier based on STM32 dev boards for use with any CNC controller.

## Features
* Up to **5** input/output axes
* Up to **500 KHz** of input/output pulses frequency (for the alpha)

### Methods
* Uses **STM32CubeMX** to initialize all options of MCU
* Uses latest **HAL** drivers
* Uses **EXTI** to capture the input values
* Simple input pulse filtering by time
* Output with configurable duty cycles

### Branches:
* **MASTER**: for the STM32F407VETx dev board (_12$_)
* **STM32F103C8Tx**: for the STM32F103C8Tx dev board (_2$_)

More info will be later..
