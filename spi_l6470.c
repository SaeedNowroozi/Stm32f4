#include "spi_l6470.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
void __init_reset()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &gpio);

    GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
}
void __init_spi()
	{
 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    SPI_InitTypeDef SPI_InitStructure;
    SPI_I2S_DeInit(SPI1);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High ;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		
    SPI_Init(SPI1, &SPI_InitStructure);

    

    SPI_Cmd(SPI1, ENABLE);
		////////////////////////////////
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//    GPIO_InitTypeDef gpio;
//    GPIO_StructInit(&gpio);
//    gpio.GPIO_Mode = GPIO_Mode_OUT;
//    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
//    GPIO_Init(GPIOD, &gpio);

//    GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
}
	uint8_t recive_l6470(){
		u32 LIS302DLTimeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(SPI1,  SPI_FLAG_RXNE) == RESET) {
        if((LIS302DLTimeout--) == 0) return 0x00;
    }

//    /* Return the Byte read from the SPI bus */
      return (u8)SPI_I2S_ReceiveData(SPI1);
}
	uint8_t Send_l6470(uint8_t byte) {
    u32 LIS302DLTimeout = 0x1000;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET) {
         if((LIS302DLTimeout--) == 0) return 0x00;
    }

    /* Send a Byte through the SPI peripheral */
    SPI_I2S_SendData(SPI1, byte);

    return recive_l6470();
}


