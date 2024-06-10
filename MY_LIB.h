
#ifndef __MY_LIB_H
#define __MY_LIB_H

#include "stdio.h"
#include "ch32v20x.h"

#endif

/*set the system clock to the given number using PLL multiplication (HSI/2 * PLL factor)*/
#define SYS_CLK_Internal_8MHz   RCC_PLLMul_2
#define SYS_CLK_Internal_12MHz  RCC_PLLMul_3
#define SYS_CLK_Internal_16MHz  RCC_PLLMul_4
#define SYS_CLK_Internal_20MHz  RCC_PLLMul_5
#define SYS_CLK_Internal_24MHz  RCC_PLLMul_6
#define SYS_CLK_Internal_28MHz  RCC_PLLMul_7
#define SYS_CLK_Internal_32MHz  RCC_PLLMul_8
#define SYS_CLK_Internal_36MHz  RCC_PLLMul_9
#define SYS_CLK_Internal_40MHz  RCC_PLLMul_10
#define SYS_CLK_Internal_44MHz  RCC_PLLMul_11
#define SYS_CLK_Internal_48MHz  RCC_PLLMul_12
#define SYS_CLK_Internal_52MHz  RCC_PLLMul_13
#define SYS_CLK_Internal_56MHz  RCC_PLLMul_14
#define SYS_CLK_Internal_60MHz  RCC_PLLMul_15
#define SYS_CLK_Internal_64MHz  RCC_PLLMul_16
#define SYS_CLK_Internal_72MHz  RCC_PLLMul_18
#define SPI_flag_TXE 	2 
#define SPI_flag_RXNE 	1

void SET_SYS_CLK(uint32_t SYS_CLK_);



//-----GPIO
void My_GPIO_init (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin , uint8_t mode ,uint8_t speed);
#define output_high(GPIOx,GPIO_Pin) GPIOx->BSHR = GPIO_Pin
#define output_low(GPIOx,GPIO_Pin)  GPIOx->BCR = GPIO_Pin
#define output_toggle(GPIOx,GPIO_Pin)  (GPIOx->OUTDR & GPIO_Pin) ? (output_low(GPIOx,GPIO_Pin)): (output_high(GPIOx,GPIO_Pin))
#define input_read_Pin(GPIOx, GPIO_Pin)  GPIOx->INDR & GPIO_Pin
#define input_read_Port(GPIOx) GPIOx->INDR 	
#define output_read_Pin(GPIOx, GPIO_Pin)  GPIOx->OUTDR & GPIO_Pin 
#define output_read_Port(GPIOx) 	GPIOx->OUTDR 
#define output_write(GPIOx, GPIO_Pin , i)  (i) ? (outputhigh(GPIOx,GPIO_Pin)) : (output_low(GPIOx,GPIO_Pin)) 

//-------SPI

void My_SPI1_Master_Init(uint16_t NSS_PIN) ; //spi master clock polarity low clock phase 1st edge
void My_SPI1_Send_Frame(uint8_t* data , uint8_t len); //sends a series of data sequencially
uint8_t My_SPI1_Transmit(uint8_t data ); //sends data byte and receives a byte 



//------RCC clock BS 
#define GPIOA_EN() 					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)
#define GPIOB_EN() 					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
#define GPIOC_EN() 					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)
#define GPIOD_EN() 					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE)
#define	GPIOE_EN()  				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE) 

#define	AFIO_EN()   				RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE)                
#define	ADC1_EN()   				RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE)      
#define	TIM1_EN()   				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE)     
#define	SPI1_EN()   				RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE)     
#define	TIM8_EN()   				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE)   
#define	USART1_EN() 				RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)    
#define	TIM9_EN()   				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE)    
#define	TIM10_EN()  				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE)   
																		
#define	TIM2_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2    , ENABLE)   
#define	TIM3_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3    , ENABLE)   
#define	TIM4_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4    , ENABLE)   
#define	TIM5_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5    , ENABLE)   
#define	TIM6_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6    , ENABLE)   
#define	TIM7_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7    , ENABLE)   
#define	UART6_EN()  				RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART6   , ENABLE)   
#define	UART7_EN()  				RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7   , ENABLE)   
#define	UART8_EN()  				RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8   , ENABLE)   
#define	WWDG_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG    , ENABLE)   
#define	SPI2_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2    , ENABLE)   
#define	SPI3_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3    , ENABLE)   
#define	USART2_EN() 				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2  , ENABLE)   
#define	USART3_EN() 				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3  , ENABLE)   
#define	UART4_EN()  				RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4   , ENABLE)   
#define	UART5_EN()  				RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5   , ENABLE)   
#define	I2C1_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1    , ENABLE)   
#define	I2C2_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2    , ENABLE)   
#define	USB_EN()    				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB     , ENABLE)   
#define	CAN1_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1    , ENABLE)   
#define	CAN2_EN()   				RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2    , ENABLE)   
#define	BKP_EN()    				RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP     , ENABLE)   
#define	PWR_EN()    				RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR     , ENABLE)   
#define	DAC_EN()    				RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC     , ENABLE)   
