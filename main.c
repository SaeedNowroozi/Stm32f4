#include "main.h"
#include "delay.h"

int main(void) {
  // initLeds();
//	__init_spi();
	//__delay(100);
   init();
//	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	//recive_l6470();
	//Send_l6470(0x0a);
//   /*             ADC              */
//    // ADC Config
//	ADC_InitTypeDef ADC_init_Structure;
//	GPIO_InitTypeDef GPIO_init_Structure;
//	//Clock configure
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_AHB1ENR_GPIOCEN,ENABLE);
//	//Analog pin Configure
//	GPIO_StructInit(&GPIO_init_Structure);
//	GPIO_init_Structure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_init_Structure.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_init_Structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOC, &GPIO_init_Structure);
//	//ADC Structure configure
//	ADC_DeInit();
//	ADC_init_Structure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_init_Structure.ADC_Resolution = ADC_Resolution_10b;
//	ADC_init_Structure.ADC_ContinuousConvMode = ENABLE;
//	ADC_init_Structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
//	ADC_init_Structure.ADC_NbrOfConversion = 1;
//	ADC_init_Structure.ADC_ScanConvMode = DISABLE;
//	ADC_Init(ADC1, &ADC_init_Structure);
//	//Enable ADC convertion
//	ADC_Cmd(ADC1, ENABLE);
//	//Select the channel to read from
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);
//	presure_cal = _presure_value();
//	milibar = presure_cal / 100;
//run(10, 10);

	//goTo_DIR(10, 1000);
	
	//softFree();
//
//softFree();
//run(0x00,0x00001);
//while(1);
////{
//// init();
////	} 
//setMicroSteps(STEP_SEL_1_8);
//	 hardStop(); //engage motors
//goTo(0x300000);
//while(1)
//{
////	__delay(10);
////	run(0x01,0x00);
//}
	//setMicroSteps(STEP_SEL_1_8);
	
	setAcc(100);
	setMaxSpeed(800);
	setMinSpeed(1);
  setMicroSteps(64); //1,2,4,8,16,32,64 or 128
  setThresholdSpeed(1000);
  setOverCurrent(6000); //set overcurrent protection
  setStallCurrent(3000);
//	goTo(200);
while(1){
	loopp();
}
}	
void loopp()
{
  while(isBusy()){ 
	__delay(10);
}
	// goTo(-200);
  
  while(isBusy()){
     __delay(10); 
  }
 //run(REV, 0x0FFFF);
//	Step_Clock(REV);
	//goUntil(ACTION_RESET,REV,0xffff);
	goTo_DIR(REV, 0xfff);
//	move(0x0FFFF);

//	goHome();
  //goTo(0x0Ff);
	//hardStop();
}
void initLeds() {
/*###############BUSYN PIN.12 ####### RESET PIN.13 ###############*/
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOD, &gpio);
}

//double _presure_value()
//{
//	presure_value_analog = readADC(12);
//	presure = ((presure_value_analog / 1024.0) + 0.095 ) / 0.000009;//1024
//	return presure;
//}
//uint16_t readADC(uint8_t channel)
//{
//	ADC_SoftwareStartConv(ADC1);
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
//	return ADC_GetConversionValue(ADC1);
//}
/*                   FPGA                */
//void init_fpga() {
//    initButton_fpga();
//    initLeds();
//  //  initTimer();
//   // initPWM();
//	  initSPI_fpga();
//}

//void loop_fpga() {
//    switch(brightLed1) {
//        case 299: delta=-1; break;
//        case 1: delta=1; break;
//    }

//    brightLed1 += delta;

//    TIM_SetCompare4(TIM4, 300 - brightLed1 % 300); // set brightness

//    uint8_t currentButtonStatus = GPIO_ReadInputDataBit(GPIOA, USER_BUTTON);
//    if (lastButtonStatus != currentButtonStatus && currentButtonStatus != RESET) {
//        brigthLed2 *= 2;
//        if (brigthLed2 >= 500 ) {
//            brigthLed2 = 5;
//        }
//        TIM_SetCompare2(TIM4, brigthLed2);
//    }
//    lastButtonStatus = currentButtonStatus;

//    delay_fpga(4);
//}

void delay_fpga(uint32_t ms) {
    ms *= 3360;
    while(ms--) {
        __NOP();
    }
}

//void initButton_fpga() {
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    GPIO_InitTypeDef gpio;
//    GPIO_StructInit(&gpio);
//    gpio.GPIO_Mode = GPIO_Mode_IN;
//    gpio.GPIO_Pin = USER_BUTTON;
//    GPIO_Init(GPIOA, &gpio);
//}

//void initLeds_fpga() {
//    GPIO_InitTypeDef GPIO_InitStructure;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

//    GPIO_InitStructure.GPIO_Pin = LEDS;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOD, &GPIO_InitStructure);

////    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
////	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
////	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
////    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
//	///////////////////////////////
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//    GPIO_InitTypeDef gpio;
//    GPIO_StructInit(&gpio);
//    gpio.GPIO_Mode = GPIO_Mode_OUT;
//    gpio.GPIO_Pin = LEDS;
//    GPIO_Init(GPIOD, &gpio);

//    GPIO_SetBits(GPIOD, LEDS);
//}

//void initTimer() {
//    /* TIM4 clock enable */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

//    /* Compute the prescaler value */
//    uint32_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 21000000) - 1;

//    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
//    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//}

//void initPWM_fpga() {
//    TIM_OCInitTypeDef TIM_OCInitStructure;

//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = 0;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//    /* PWM1 Mode configuration: Channel1 (GPIOD Pin 12)*/
//    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
//    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

//    /* PWM1 Mode configuration: Channel2 (GPIOD Pin 13)*/
//    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
//    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
//    
//	 /* PWM1 Mode configuration: Channel3 (GPIOD Pin 14)*/
//    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
//    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

//    /* PWM1 Mode configuration: Channel4 (GPIOD Pin 15)*/
//    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
//    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
//		
//    TIM_Cmd(TIM4, ENABLE);
//}
//void initSPI_fpga(void)
//{
//	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//	
//	  GPIO_InitTypeDef GPIO_InitStruct;
//    GPIO_StructInit(&GPIO_InitStruct);
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//    GPIO_Init(GPIOA, &GPIO_InitStruct);
//	  
//	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
//    GPIO_Init(GPIOE, &GPIO_InitStruct);
//    GPIO_SetBits(GPIOE, GPIO_Pin_3);

//    SPI_InitTypeDef SPI_InitStructure;
//    SPI_I2S_DeInit(SPI1);
//    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;
//    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
//    SPI_Init(SPI1, &SPI_InitStructure);

//    SPI_Cmd(SPI1, ENABLE);
		//////////////////////////////////////
//		RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//	
//	  GPIO_InitTypeDef GPIO_InitStruct;
//    GPIO_StructInit(&GPIO_InitStruct);
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//    GPIO_Init(GPIOB, &GPIO_InitStruct);
//	  
//	  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
//		
//		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
//    GPIO_Init(GPIOB, &GPIO_InitStruct);
//    GPIO_SetBits(GPIOB, GPIO_Pin_12);
		
//		SPI_InitTypeDef SPI_InitStructure;
//    SPI_I2S_DeInit(SPI2);
//    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;
//    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
//    SPI_Init(SPI2, &SPI_InitStructure);

    
//    SPI_InitTypeDef SPI_InitStructure;
//    SPI_I2S_DeInit(SPI1);
//    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStructure.SPI_CRCPolynomial = 7;
//    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
//    SPI_Init(SPI1, &SPI_InitStructure);

//    SPI_Cmd(SPI2, ENABLE);

//}
//void fpga_fpga(void)
//{
//	//GPIO_SetBits(GPIOE, GPIO_PIN_3);
//	uint8_t reg = 0x47;
//	Write_fpga(0x20, &reg, 1);
//	//GPIO_ResetBits(GPIOE, GPIO_PIN_3);
////	unsigned char value;
////	switch(value){
////		case 0xff :
////			GPIO_SetBits(GPIOD, LEDS);
////			break;
////		case 0x25 :
////			GPIO_SetBits(GPIOD, LEDS);
////			break;
////		case 0x90 :
////			GPIO_SetBits(GPIOD, LEDS);
////			break;
////		default :
////      GPIO_SetBits(GPIOD, LEDS);	
////			}
//}
//uint8_t SendByte_fpga(uint8_t byte)
//{
//	 uint32_t LIS302DLTimeout;
//	 while (SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET) {
//        if((LIS302DLTimeout --) == 0) return 0x00;
//    }

//    /* Send a Byte through the SPI peripheral */
//    SPI_I2S_SendData(SPI2, byte);
//     return 0x88;
//    //return GetByte();
//}
//uint8_t GetByte_fpga(void)
//	{
//     uint32_t LIS302DLTimeout;
//    /* Wait to receive a Byte */
//     //LIS302DLTimeout = TIMEOUT;
//    while (SPI_I2S_GetFlagStatus(SPI2,  SPI_FLAG_RXNE) == RESET) {
//        if((LIS302DLTimeout --) == 0) return 0x00;
//    }

//    /* Return the Byte read from the SPI bus */
//    byteRecive = (uint8_t)SPI_I2S_ReceiveData(SPI2);
//		return SendByte_fpga(byte1);
//}
//void Write_fpga(uint8_t Reg, uint8_t *DATA, uint8_t count)
//{
//	  int i;
//     /* Write FPGA */
//		GPIO_SetBits(GPIOB, GPIO_Pin_12);
//		delay_fpga(100);
//		/* Start SPI Protecol */
//		GPIO_ToggleBits(GPIOB,GPIO_Pin_12);
//    //GPIO_WriteBit(GPIOE, GPIO_Pin_3, RESET);
////    SendByte(Reg);
////    for (i=0; i < count; i++) {
////        SendByte(*DATA);
////        DATA++;
////    }
//	  GetByte_fpga();
//		GPIO_ToggleBits(GPIOB, GPIO_Pin_12);
//    //GPIO_WriteBit(GPIOE, GPIO_Pin_3, SET);
//}
//void Read_fpga(uint8_t Reg, uint8_t *DATA, uint8_t count)
//{
//		uint8_t i;
//    //GPIO_WriteBit(GPIOE, GPIO_PIN_3, SET);
//    /* Read FPGA */
//		GPIO_SetBits(GPIOB, GPIO_Pin_12);
//		delay_fpga(100000);
//		/* Start SPI Protecol */
//		GPIO_ToggleBits(GPIOB,GPIO_Pin_12);
//	//GPIO_ReadInputData(GPIOB);
//	  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
//	  //GPIO_WriteBit(GPIOE, GPIO_Pin_3, RESET);
////    Reg |= 0x80;
////    if (count > 1) {
////        Reg |= 0x40;
////    }
////    SendByte(Reg);
////    for (i=0; i < count; i++) {
////        *DATA = SendByte((uint8_t)0x00);
////         DATA++;
////    }
//	  GetByte_fpga();
//	  GPIO_ToggleBits(GPIOB, GPIO_Pin_12);
//}




