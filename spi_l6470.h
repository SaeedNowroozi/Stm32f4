#ifndef __SPI_L6470_H
#define  __SPI_L6470_H

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"

void __init_spi();
uint8_t recive_l6470();
uint8_t Send_l6470(uint8_t byte);
void __init_reset();
//const u32 LIS302DL_FLAG_TIMEOUT = 0x1000;
#endif