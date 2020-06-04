# nucleo-f334r8-ten-running-leds
In this project example of running LED lights consisting of 10 LEDs and controled with [NUCLEO-F334R8](https://www.st.com/en/evaluation-tools/nucleo-f334r8.html) board are implemented. Pressing of user button B1 changes sequence direction. It is possible to adjust speed from terminal by sending __+__ or __-__ characters.

Project is built using [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) integrated development environment for STM32 (version 1.3.0). Project initialization code is generated using STM32CubeMX. 

Hardware abtraction layer (HAL) is used to enable portability between diffrent STM32 devices.
