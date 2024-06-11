
#include "MY_LIB.h"
#include "W5500_lib.h"


/*********************************************************************
 * @fn      W5500_write
 * 
 * @brief  	using spi1(can be modified) sends a write SPI frame to the W5500
 *			
 * @param   address - address phase the offset address in the frame 
 *			block   - control phase selects the socket or selecting common register          
 *			data    - data phase the data to be written in the specified address
 */

void W5500_write(uint16_t address,uint8_t block , uint8_t data) {
    output_low(GPIOA, SPI_NSS_PIN);
	//Sending Address
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI1->DATAR = (address & 0xFF00) >> 8) ; 
	
	while (!(SPI1->STATR  |= SPI_flag_TXE));
    SPI1->DATAR = address & 0x00FF; 
	//Sending Control byte
	while (!(SPI1->STATR  |= SPI_flag_TXE));
    SPI1->DATAR = 0x04  | block; // Write command
	//Sending Data
	while (!(SPI1->STATR  |= SPI_flag_TXE));
    SPI1->DATAR = data ;
    output_high(GPIOA, SPI_NSS_PIN);
}
/*********************************************************************
 * @fn      W5500_read
 * 
 * @brief  	using spi1(can be modified) sends a read SPI frame to the W5500
 *			
 * @param   address - address phase the offset address in the frame 
 *			block   - control phase selects the socket or selecting common register          
 *		
 * @returns data from the specified address
 *
 */
uint8_t W5500_read(uint16_t address,uint8_t block ) {
	 uint8_t received_data = SPI1->DATAR; //clearing RXNE 
    output_low(GPIOA, SPI_NSS_PIN);
	//Sending Address
	while (!(SPI1->STATR  |= SPI_flag_TXE));
	SPI1->DATAR = (address & 0xFF00) >> 8) ; 
	while (!(SPI1->STATR  |= SPI_flag_RXNE));
    received_data = SPI1->DATAR; //clearing RXNE 
	while (!(SPI1->STATR  |= SPI_flag_TXE));
	SPI1->DATAR = address & 0x00FF; 
	while (!(SPI1->STATR  |= SPI_flag_RXNE));
	received_data = SPI1->DATAR ; //clearing RXNE 
	
	//Sending Control byte
	while (!(SPI1->STATR  |= SPI_flag_TXE));
	SPI1->DATAR = 0x00  | block;
	while (!(SPI1->STATR  |= SPI_flag_RXNE));
	received_data = SPI1->DATAR ; //clearing RXNE 
	//Sending Data
	while (!(SPI1->STATR  |= SPI_flag_TXE)); 
	SPI1->DATAR = 0x00 ;
    // Get the received data
	while (!(SPI1->STATR  |= SPI_flag_RXNE));
    received_data = SPI1->DATAR;
	output_high(GPIOA, SPI_NSS_PIN);
    return received_data; // Return the received data
    
}
/*********************************************************************
 * @fn      W5500_Connection_check
 * 
 * @brief  	-it is suggested to use this as the first function to communicate with the w5500
 *			-it excutes 2 read operation, w5500 replies with 1 to the first read frame
 *			-reads version of chip and checks if it is 4
 *			
 *  @returns 0 nothing was recieved 
 * 			 1 first acknowlgement bit received (1) but chip version data is wronge
 *	         2 first acknowlgement bit not received (1) but chip version data is correct
 *			 3 first acknowlgement bit received (1) but chip version data is correct
 */
uint8_t W5500_Connection_check(void)
{	
	uint8_t x = 0;
	//the W5500 when you read from it first time it returns a 1 regardless of anything
	if(W5500_read(W5500_CHIPV, 0x04) == 1)
	{
		x = 1 ; 
	}
	//chip version is always 4
	if(W5500_read(W5500_CHIPV, 0x04) == 4)
	{
		x += 2 ;
	}
	return x;
}
/* 
 * this function resets w5500 and sets up default gateway / subnetmask
 * @param   ip  - source ip
 *          mac - source mac			
 */

void W5500_setup(uint8_t *ip, uint8_t *mac) {
	My_SPI1_Master_Init(GPIO_Pin_4);

    // Perform software reset
    W5500_write(W5500_MR, COMMON_REG, RESET_BIT);

    // Wait for the reset to complete
    while(W5500_read(W5500_MR, COMMON_REG) & RESET_BIT);

    // Check PHY status to ensure the link is up
    while(!(W5500_read(W5500_PHYCFGR, COMMON_REG) & LINK_UP));

	// Set MAC address
    for (int i = 0; i < 6; i++) {
        W5500_write(W5500_SHAR + i, COMMON_REG , mac[i]);
    }
    
    // Set IP address
    for (int i = 0; i < 4; i++) {
        W5500_write(W5500_SIPR + i, COMMON_REG, ip[i]);
    }

    // Set Subnet Mask (example 255.255.255.0)
    uint8_t subnet[4] = {255, 255, 255, 0};
    for (int i = 0; i < 4; i++) {
        W5500_write(W5500_SUBR + i, COMMON_REG , subnet[i]);
    }

    // Set Gateway (example 192.168.1.1)
    uint8_t gateway[4] = {192, 168, 1, 1};
    for (int i = 0; i < 4; i++) {
        W5500_write(W5500_GAR + i, COMMON_REG , gateway[i]);
    }
}

uint8_t W5500_connect(uint8_t socket, uint8_t *targetIP, uint16_t targetPort)
{
	
    // Step 1: Set the socket mode to TCP
    W5500_write(W5500_SN_MR, Socket_n_Register(socket) , 0x01); // 0x01 is TCP mode
	
	 // Step 2: Open the socket
    W5500_write(W5500_SN_CR, Socket_n_Register(socket) , W5500_CR_OPEN );
    while (W5500_read(W5500_SN_CR, Socket_n_Register(socket))); // Wait until the command is cleared
	delay_Us(1);
	
	// Step 3: Check if socket is in INIT state
    if (W5500_read(W5500_SN_SR, Socket_n_Register(socket)) != W5500_SOCK_INIT) {
        return 0; // Failed to open socket
    }
	
    // Step 4: Set the destination IP address
    for (int i = 0; i < 4; i++) {
        W5500_write(W5500_SN_DIPR + i, Socket_n_Register(socket) , targetIP[i]);
    }
	
    // Step 5: Set the destination port
    W5500_write(W5500_SN_DPORT, Socket_n_Register(socket) , (uint8_t)(targetPort >> 8)); // Upper byte
    W5500_write(W5500_SN_DPORT + 1, Socket_n_Register(socket) , (uint8_t)(targetPort & 0xFF)); // Lower byte
	
	// Step 6: Send CONNECT command
    W5500_write(W5500_SN_CR,  Socket_n_Register(socket) , W5500_CR_CONNECT);
    while (W5500_read(W5500_SN_CR, Socket_n_Register(socket) )); // Wait until the command is cleared
	
	// Step 7: Wait until socket is in ESTABLISHED state
    while ( W5500_read(W5500_SN_SR, Socket_n_Register(socket) ) != W5500_SOCK_ESTABLISHED) {
    // Optional: Add a timeout mechanism here
    }
	return 1;
}

void W5500_sendData(uint8_t socket, uint8_t *data, uint16_t len) {
    // Socket TX Write Pointer Address Offset

    //#define TX_BUF_BASE 0x4000

    uint8_t socket_block = Socket_n_Register(socket);

    uint16_t ptr = (W5500_read(W5500_SN_TX_WR, socket_block) << 8) + W5500_read(W5500_SN_TX_WR + 1, socket_block);
    //uint16_t offset = ptr & 0x07FF; // Mask for 2KB buffer

    // Write data to the TX buffer
    for (int i = 0; i < len; i++) {
        W5500_write(ptr, Socket_n_TX_Buffer(socket), data[i]);
    }

    // Update the TX write pointer
    ptr += len;
    W5500_write(W5500_SN_TX_WR, socket_block, (uint8_t)(ptr >> 8));
    W5500_write(W5500_SN_TX_WR + 1, socket_block, (uint8_t)(ptr & 0xFF));

    // Send the SEND command
    W5500_write(W5500_SN_CR, socket_block, W5500_CR_SEND); // SEND command
    while (W5500_read(W5500_SN_CR, socket_block)); // Wait until the command is cleared

    // Clear the SEND_OK interrupt
    W5500_write(W5500_SN_IR, socket_block, 0x10);
}

uint16_t W5500_Size_Received(uint8_t socket)
{
	uint16_t size = (W5500_read(W5500_SN_RX_RSR, Socket_n_Register(socket)) << 8 + W5500_read(W5500_SN_RX_RSR + 1, Socket_n_Register(socket)));
    if (size == 0) {
        return 0; // No data received
    }
	return size;
}
uint8_t W5500_receiveData(uint8_t socket) {
    // Socket RX Read Pointer Address Offset
    #define RX_BUF_BASE 0x6000
	
    uint8_t socket_block = Socket_n_Register(socket);


    uint16_t ptr = (W5500_read(W5500_SN_RX_WR, socket_block) << 8) + W5500_read(W5500_SN_RX_WR + 1, socket_block);

    // Read data from RX buffer
    uint8_t data = W5500_read(ptr, Socket_n_RX_Buffer(socket));

    // Update the RX read pointer
    ptr++;
    W5500_write(W5500_SN_RX_WR, socket_block, (uint8_t)(ptr >> 8));
    W5500_write(W5500_SN_RX_WR + 1, socket_block, (uint8_t)(ptr & 0xFF));

    // Send the RECV command
    W5500_write(W5500_SN_CR, socket_block, W5500_CR_RECV ); // RECV command
    while (W5500_read(W5500_SN_CR, socket_block)); // Wait until the command is cleared

    return data;
}
