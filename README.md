# Steps frequency multiplier
Simple steps frequency multiplier based on STM32 dev boards. 
For use with any CNC/3DPrint controller.

![1](https://cloud.githubusercontent.com/assets/16130975/23888395/eb75af6a-08b1-11e7-86b9-adae2919858f.png)
![1](https://cloud.githubusercontent.com/assets/16130975/23888399/eb7ce5a0-08b1-11e7-9daf-cbecc38b2927.png)

## Downloads
* https://github.com/MX-Master/steps-frequency-multiplier/releases

## Features
* Up to **5** input/output axis count
* Up to **4 MHz** of output pulse frequency

### Methods
* Uses **STM32CubeMX** to initialize all options of MCU
* Uses latest **HAL** drivers
* Uses **SW4STM32** IDE to build project
* Uses **EXTI** to capture input pulses
* Uses timer's **OC** mode with DMA for the output

### Active branches:
* **STM32F407VETx_OC_DMA**: New fast method of output with timer's OC+DMA for the STM32F407VETx
* **STM32F407VETx_OC_DMA_2**: Method based on STM32F407VETx_OC_DMA using only 2 timers for the output
* **F407VETx_BUF**: Another method of output for the STM32F407VETx using buffer and timer's OC+DMA

### Other branches:
* **STM32F407VETx_old**: Old and slow method of output for the STM32F407VETx
* **STM32F103C8Tx_OC_DMA**: New fast method of output with timer's OC+DMA for the STM32F103C8Tx
* **STM32F103C8Tx**: First (non tested) version for the STM32F103C8Tx
