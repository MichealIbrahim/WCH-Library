#include "MY_LIB.h"

volatile uint16_t SPI_NSS_PIN = GPIO_Pin_4;


/*********************************************************************
 * @fn      SET_SYS_CLK
 *
 * @brief   sets up the microcontroller clock (reliably)
 *
 * @param   SYS_CLK the system clock desired the for the microcontroller to run on
 *			example SYS_CLK_Internal_32MHz
 *
 * 
 */
void SET_SYS_CLK(uint32_t SYS_CLK)
{
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    RCC_PLLCmd(DISABLE);
    if(SYS_CLK == SYS_CLK_Internal_8MHz)
    {
        SystemCoreClockUpdate();
        return;
    }
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, SYS_CLK);
    RCC_PLLCmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    SystemCoreClockUpdate();
}

void My_GPIO_init (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin , uint8_t mode ,uint8_t speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Speed = speed;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      Int_Simple_enable
 * 
 * @brief   enable Interrupt
 *
 * @param   addr - VTF interrupt service function base address.
 *          IRQn - Interrupt Numbers example TIM2_IRQn
 *
 */
void Int_Simple_enable(uint32_t addr , IRQn_Type IRQn)
{
    NVIC_EnableIRQ(IRQn);
    SetVTFIRQ((u32)addr,IRQn,0,ENABLE);
}


/*********************************************************************
 * @fn      Int_Simple_disable
 * 
 * @brief   disable Interrupt
 *
 * @param   addr - VTF interrupt service function base address.
 *          IRQn - Interrupt Numbers example TIM2_IRQn
 *
 */
void Int_Simple_disable(uint32_t addr , IRQn_Type IRQn)
{
    NVIC_DisableIRQ(IRQn);
    SetVTFIRQ((u32)addr,IRQn,0,DISABLE);
}

/*
 *this function sets up SPI1 as master mode0 and software cs select pin as argument
 *the cs pin you input will be used automatically on the rest of SPI/w5500 functions 
 */
void My_SPI1_Master_Init(uint16_t NSS_PIN) {
    // Enable clocks for SPI1 and GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	SPI_NSS_PIN = NSS_PIN;
    // Configure GPIO pins for SPI SCK and MOSI (PA5, PA7)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7; // SCK, MOSI
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // Set GPIO speed
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function push-pull mode
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure GPIO pin for SPI MISO (PA6)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // MISO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Floating input mode
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure GPIO pin for SPI NSS (PA4)
    GPIO_InitStructure.GPIO_Pin =   SPI_NSS_PIN ; // NSS
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // General-purpose output push-pull mode
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // SPI configuration
    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // Full duplex mode
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; // SPI master mode
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // 8-bit data size
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; // Clock polarity low
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; // Clock phase 1st edge
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // Software control of NSS
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // Baud rate prescaler
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // MSB transmitted first
    SPI_InitStructure.SPI_CRCPolynomial = 7; // CRC polynomial
    SPI_Init(SPI1, &SPI_InitStructure); // Initialize SPI1 with these settings

    // Enable SPI1
    SPI_Cmd(SPI1, ENABLE);

    // Set NSS high initially (deselect the slave)
    output_high(GPIOA, SPI_NSS_PIN);
    //GPIO_ResetBits(GPIOA, SS_PIN);
}



/*********************************************************************
 * @fn      My_SPI1_Send_Frame
 * 
 * @brief   sends set of bytes of data on SPI1 (can be modified to input SPIx but i am only using SPI1)
 *			
 * @param   data - the data array to be sent
 *			len  - the size of the array          
 *
 */
void My_SPI1_Send_Frame(uint8_t* data , uint8_t len)
{
	
    // Set NSS low to select the slave
    output_low(GPIOA, SPI_NSS_PIN);
	for(uint8_t i = 0 ; i < len ; i++)
	{
		// Wait until the transmit buffer is empty
		while (!(SPI1->STATR  |= SPI_flag_TXE)); 
		SPI1->DATAR = data[i];
	}
	output_high(GPIOA, SPI_NSS_PIN);
}


/*********************************************************************
 * @fn      My_SPI1_Transmit
 * 
 * @brief   sends a byte and receives a byte but can cause data flow sending to be cut(can be modified to input SPIx but i am only using SPI1)
 *			
 * @param   data - the data array to be sent
 *			len  - the size of the array 
 * 
 * @returns data received from slave 
 */
uint8_t My_SPI1_Transmit(uint8_t data ) {


	uint8_t received_data = SPI1->DATAR; //clearing RXNE 
    // Wait until the transmit buffer is empty
    while (!(SPI1->STATR  |= SPI_flag_TXE));
    // Set NSS low to select the slave
    output_low(GPIOA, SPI_NSS_PIN);
    // Send data
    SPI_I2S_SendData(SPI1, data);
    // Set NSS high to deselect the slave
    output_high(GPIOA, SPI_NSS_PIN);
	
    while (!(SPI1->STATR  |= SPI_flag_RXNE));

    // Get the received data
     received_data = SPI_I2S_ReceiveData(SPI1);

    return received_data; // Return the received data
}


//------Timer 
/********************************************
 * use TIMx_En() before 
 * sets up the TIMx as a regular timer
 *
 *
 */
void My_TIMx_TimeBase_Setup(TIM_TypeDef *TIMx,uint16_t period, uint16_t prescaler, uint16_t TIM_CKD_DIV , uint16_t TIM_CounterMode)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = period;  // period
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;  // prescaler
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
}
/********************************************
 * use TIMx_En() before 
 * sets up the TIMx as a regular timer
 * set up proper output pins as GPIO_Mode_AF_PP
 *
 */
void My_TIMx
void My_TIMx_PWM_SetUp(TIM_TypeDef *TIMx,uint16_t period, uint16_t prescaler, uint16_t TIM_CKD_DIV, uint16_t pwm_duty , uint8_t channels 
, FlagStatus Nchannel_En )
{
		    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


    // Configure TIMx for PWM mode
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = period;  
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = Nchannel_En;
	TIM_OCInitStructure.TIM_Pulse = pwm_duty;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_ARRPreloadConfig(TIMx, ENABLE);
    // PWM1 Mode configuration: Channel1
	if(channels | PWM_Channel_1)
	{
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}		
	if(channels | PWM_Channel_2)
	{
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}	
	if(channels | PWM_Channel_3)
	{
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}	
	if(channels | PWM_Channel_4)
	{
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}	
	
    
 
    // Enable TIM2
    TIM_Cmd(TIM2, ENABLE);

}