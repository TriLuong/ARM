# Control-Position-for-Motor-using-PID-Controller

Hi everyone,

In this project, I used STM32f407, Driver LMD18200 and PID Controller to Control Position for Faulhaber Motor.
I use KeilC and STM32CubeMX to code for STM32f407.
- Pin PE10 on MCU connect to DIR pin of Driver.
- Pin PA7 on MCU connect to PWM pin of Driver.
- I use Pin PA0 and PA1 on MCU to read encoder from Motor.
