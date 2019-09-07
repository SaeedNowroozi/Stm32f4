/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

#include "math.h"
#include "L6470.h"
#include "spi_l6470.h"
//#include "delay.h"

const u16 LEDS = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
const u16 LED1 = GPIO_Pin_12;
const u16 LED2 = GPIO_Pin_13;
const u16 LED3 = GPIO_Pin_14;
const u16 LED4 = GPIO_Pin_15;
//const u32 LIS302DL_FLAG_TIMEOUT = 0x1000;
//const u32 PWM_PERIOD = 4000;

//int main(void);
//void init(void);
//void loop(void);
//void delay(u32 ms);
//void delay(u32 ms);
void initLeds(void);
void loopp();
//void initSpi(void);
//void LIS302DL_Init(void);
//void initTimer(void);
//void initPWM(void);

//u8 LIS302DL_SendByte(u8 byte);
//u8 LIS302DL_GetByte(void);
//void L6470_Write(u8 REG, u8 *DATA, u8 count);
//void L6470_Read(u8 REG, u8 *DATA, u8 count);
//void LIS302DL_ReadACC(int32_t* out);
//void LIS302DL_ReadACCY(int32_t* out);
double _presure_value();
uint16_t readADC(uint8_t channel);

double presure;
float presure_value_analog;
 float milibar, presure_cal;
//u32 abs(int32_t n);
//int32_t cround(int32_t x, int32_t y);

//########################################
//u8 send_l6470(u8 byte);
//u8 recive_l6470();
/*                       FPGA                  */
//const uint32_t PWM_PERIOD = 4000;

//int brightLed1 = 0;
//int brigthLed2 = 5;
//int delta = 1;
//int lastButtonStatus = RESET;

//void init_fpga();
//void loop_fpga();

void delay_fpga();

//void initButton_fpga();
//void initLeds_fpga(void);
//void initTimer_fpga(void);
//void initPWM_fpga(void);
///////////////////////
//void initSPI_fpga(void);
//void fpga_fpga(void);
//////////////////////
//uint32_t absa_fpga(uint32_t n);
//uint32_t cround_fpga(uint32_t x, uint32_t y);
//////////////////////////READ & WRITE
//uint8_t SendByte_fpga(uint8_t byte);
//uint8_t GetByte_fpga(void);

//void Write_fpga(uint8_t Reg, uint8_t *DATA, uint8_t count);
//void Read_fpga(uint8_t Reg, uint8_t *DATA, uint8_t count);

////const u32 TIMEOUT = 0x1000;

//const uint16_t USER_BUTTON = GPIO_Pin_0;

//const uint8_t byte1 = 0x89;
//uint8_t byteRecive;

#endif /* __MAIN_H */
