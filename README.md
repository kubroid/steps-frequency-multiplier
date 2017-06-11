# Step pulses frequency multiplier
It's a very useful tool when your **CNC controller** can't output more than **20-50 KHz** of step pulses. 
With this mupltiplier you can encrease the output frequency from **20-50 KHz** up to **2-4 MHz**.
Such frequencies can be used for servo drivers or for step motor drivers (for the extra small microstep).
Steps frequency multiplier it's a good solution when you are using **Mach3** or **LinuxCNC** soft with **LPT** (parallel) port output. Or when you are using a cheap **USB** controller like **Arduino** with **GRBL** firmware.

![1](https://cloud.githubusercontent.com/assets/16130975/23888395/eb75af6a-08b1-11e7-86b9-adae2919858f.png)
![1](https://cloud.githubusercontent.com/assets/16130975/23888399/eb7ce5a0-08b1-11e7-9daf-cbecc38b2927.png)
![1](https://user-images.githubusercontent.com/16130975/26940716-94b039cc-4c9d-11e7-83ac-2319147041b3.jpg)
![1](https://user-images.githubusercontent.com/16130975/26940755-bbe32db0-4c9d-11e7-9723-13ab55ed22e3.png)

## Downloads
* https://github.com/MX-Master/steps-frequency-multiplier/releases

## Features
* Up to **5** input/output axis count
* Up to **4 MHz** of output pulse frequency

### Methods
* Uses **STM32CubeMX** to initialize all options of MCU
* Uses latest **HAL** drivers
* Uses **SW4STM32** IDE to build the project
* Uses **EXTI** to capture input pulses
* Uses timer's **OC** mode with **DMA** for the output
* Uses **SysTick** timer to catch the timestamp (in us) of each input step

### Active branches:
* **F407VETx_OC_DMA_3**: The most stable method of output using timer's OC mode + DMA (for the F407VETx)
* **F103C8Tx_OC_DMA_3**: The most stable method of output using timer's OC mode + DMA (for the F103C8Tx)

### Test branches:
* **STM32F407VETx_old**: Old and slow method of output for the STM32F407VETx
* **STM32F407VETx_OC_DMA**: New fast method of output with timer's OC+DMA for the STM32F407VETx
* **STM32F407VETx_OC_DMA_2**: Method based on STM32F407VETx_OC_DMA using only 2 timers for the output
* **F407VETx_BUF**: Another method of output for the STM32F407VETx using buffer and timer's OC+DMA
* **STM32F103C8Tx_OC_DMA**: New fast method of output with timer's OC+DMA for the STM32F103C8Tx
* **STM32F103C8Tx**: First (non tested) version for the STM32F103C8Tx
