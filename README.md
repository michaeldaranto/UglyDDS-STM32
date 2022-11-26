# UglyDDS-STM32
Simple DDS with STM32F411 & Si5351

Library :
* https://github.com/afiskon/stm32-si5351 
* https://github.com/afiskon/stm32-ssd1306

WeAct Black Pill V2.0 :
* https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0

Output Pin :
* I2C1_SCL->PB6, I2C_SDA->PB7
* TIM3_CH1->PA6, TIM3_CH2->PA7
* GPIO_EXTI0->PA0

Output CLK0-->BFO-VFO, CLK2-->BFO

NOTES:
* Si5351 ADAFRUIT breakout "CLONE" will need 3,3V for Vin. NOT 3V or 5V (blame it to shift level conversion). My Discovery Board fail to recognize this Si5351 board because Discovery output pin for supply only provide 3V! (maybe this case only mine??)
* Timebase system tick timer Preemption Priority must lower value than EXTI0. We need this for HAL_Delay inside switch radix function (pin PA0) interupt. Check NVIC Interrupt table for preemption priority.
* Put HAL_Delay for anti bouncing switch (EXTI0) at stm32f4xx_it.c
* Port for others variants : F103,F407,H743 not tested! But thanks to HAL! :)
