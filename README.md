# Steps frequency multiplier
Simple steps frequency multiplier based on STM32 dev boards. 
For use with any CNC/3DPrint controller.

![2017-02-21_202058](https://cloud.githubusercontent.com/assets/16130975/23169236/29a15cdc-f875-11e6-8569-83c2fc136169.png)

## Downloads
* https://github.com/MX-Master/steps-frequency-multiplier/releases

## Features
* Up to **5** input/output axis count
* Up to **2 MHz** of output pulse frequency

### Methods
* Uses **STM32CubeMX** to initialize all options of MCU
* Uses latest **HAL** drivers
* Uses **SW4STM32** IDE to build project
* Uses **EXTI** to capture input pulses
* Uses timer's **OC** mode with DMA for the output

### Branches:
* **STM32F407VETx_OC_DMA**: New fast method of output with timer's OC+DMA for the STM32F407VETx
* **STM32F407VETx_slow**: Old and slow method of output for the STM32F407VETx
* **STM32F103C8Tx**: First (non tested) version for the STM32F103C8Tx
