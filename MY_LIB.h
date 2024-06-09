
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
//w5500 BLOCK selection 
#define COMMON_REG                  0b00000000
#define Socket_n_Register(socket)  	0b00001000 | (socket<<5)
#define Socket_n_TX_Buffer(socket)  0b00010000 | (socket<<5)
#define Socket_n_RX_Buffer(socket)  0b00011000 | (socket<<5)

// Offset Address for Common Register 
#define W5500_MR 		0x0000	//mode register 
#define W5500_GAR 		0x0001	//default gateway address 4 bytes
#define W5500_SUBR		0x0005	//subnet mask 4 bytes
#define W5500_SHAR		0x0009	//source MAC address 6 bytes
#define W5500_SIPR		0x000F	//source IP address 4 bytes
#define W5500_IR 		0x0015	//interrupt register
#define W5500_IMR 		0x0016	//interrupt mask
#define W5500_SIR 		0x0017	
#define W5500_SIMR		0x0018
#define W5500_PHYCFGR   0x002E
#define W5500_CHIPV 	0x0039	//chip version register
//useful for MR 
#define RESET_BIT   0x80
#define LINK_UP     0x01


// Offset Address in Socket n Register Block
#define W5500_SN_MR			0x0000	//socket mode register
#define W5500_SN_CR			0x0001	//socket command register
#define W5500_SN_IR			0x0002	//socket interrupt register 
#define W5500_SN_SR			0x0003	//socket status register
#define W5500_SN_PORT0		0x0004  //source port register 2 bytes
#define W5500_SN_DHAR		0x0006  //destination MAC address 6 bytes
#define W5500_SN_DIPR		0x000C	//destination IP address 4 byes
#define W5500_SN_DPORT 		0x0010 	//destination PORT 2 bytes 
#define W5500_SN_TX_FSR 	0x0020  //Socket n TX Free Size
#define W5500_SN_TX_RD 		0x0022  //Socket n TX Read Pointer
#define W5500_SN_TX_WR 		0x0024  //Socket n TX Write Pointer
#define W5500_SN_RX_RSR		0x0026  //Socket n RX Received Size
#define W5500_SN_RX_RD 		0x0028  //Socket n RX Read Pointer
#define W5500_SN_RX_WR 		0x002A  //Socket n RX Write Pointer
#define W5500_SN_Sn_FRAG0	0x002D	//Socket n Fragment Offset in IP header
#define W5500_SN_Sn_FRAG1	0x002E

//socket commands (writen to command register)
#define W5500_CR_OPEN 		0x01
#define W5500_CR_CONNECT 	0x04
#define W5500_CR_DISCON		0x08
#define W5500_CR_CLOSE 		0x10
#define W5500_CR_SEND 		0x20
#define W5500_CR_RECV 		0x40
//useful for SR (status register)
#define W5500_SOCK_INIT 		0x13
#define W5500_SOCK_ESTABLISHED 	0x17
#define W5500_SOCK_CLOSE_WAIT   0x1C


void SET_SYS_CLK(uint32_t SYS_CLK_);

//SPI functions 
void My_SPI1_Master_Init(uint16_t NSS_PIN) ; //spi master clock polarity low clock phase 1st edge
void My_SPI1_Send(uint8_t data); //sends data and doesn't check for receive (maybe can keep the data flow if used sequencially idk)
void My_SPI1_Send_Frame(uint8_t* data , uint8_t len); //sends a series of data sequencially
uint8_t My_SPI1_Transmit(uint8_t data ); //sends data byte and receives a byte 

//w5500 module functions
void W5500_write(uint16_t address,uint8_t block , uint8_t data);
uint8_t W5500_read(uint16_t address,uint8_t block );
uint8_t W5500_Connection_check(void);
void W5500_setup(uint8_t *ip, uint8_t *mac); //sets up SPI1 with CS pin as A4 default gateway and subnet mask
uint8_t W5500_connect(uint8_t socket, uint8_t *targetIP, uint16_t targetPort);
void W5500_sendData(uint8_t socket, uint8_t *data, uint16_t len);
uint16_t W5500_Size_Received(uint8_t socket);
uint8_t W5500_receiveData(uint8_t socket);

/*	example to enable an interrupt
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //NVIC_EnableIRQ(TIM2_IRQn);
    SetVTFIRQ((u32)TIM2_IRQHandler,TIM2_IRQn,0,ENABLE);   <<------ important function
*/