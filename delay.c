#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

void __delay(uint32_t ms)
{
	 ms *= 1000;
    while(ms--) {
        __NOP();
    }
}