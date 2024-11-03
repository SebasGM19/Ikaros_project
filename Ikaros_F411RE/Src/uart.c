/*
 * uart.c
 *
 *  Created on: Oct 26, 2024
 *      Author: sebas
 */

#include "uart.h"
#include "gpios.h"

Status_code_t USART_Clock(Enabled_Disabled_t state, usart_alternative_t USART){
	RCC_offset_t volatile RccOffset = RCC_OFFSET_APB2ENR;
	memoryMapAddress_t volatile MainAddress = USART1_ADDRESS;

	switch(USART){
	case Usart_1:
		RccOffset= RCC_OFFSET_APB2ENR;
		MainAddress = USART1_ADDRESS;
		break;
	case Usart_2:
		RccOffset= RCC_OFFSET_APB1ENR;
		MainAddress = USART2_ADDRESS;
		break;
	case Usart_6:
		RccOffset= RCC_OFFSET_APB2ENR;
		MainAddress = USART6_ADDRESS;
		break;
	default:
		return USART_alternative_not_suported;
		break;
	}


	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(MainAddress + RccOffset);

    *pClockControlReg = (state) ? (*pClockControlReg | (Enabled << USART)) : (*pClockControlReg & ~(Enabled << USART));

    return Success;

}


/*______________________________________USART_1___________________________*/
Status_code_t Init_UART1(usart_config_t USART_config){
	Status_code_t status = Success;

	status = SetPinMode(Port_B, Pin_6, Alt_func_mode); //TX
	status |= SetPinMode(Port_B, Pin_7, Alt_func_mode); //RX

	if(status!=Success){
		return status;
	}
	USART_Clock(Enabled, Usart_1);

	GpioSetAlternativeFunction(Port_B, Pin_6, SPI3_I2S3_USART1_USART2);
	GpioSetAlternativeFunction(Port_B, Pin_7, SPI3_I2S3_USART1_USART2);

	status = set_baud_rate(Usart_1, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}

	return Success;
}


/*______________________________________USART_2___________________________*/

Status_code_t Init_UART2(usart_config_t USART_config){
	Status_code_t status = Success;

	status = SetPinMode(Port_A, Pin_2, Alt_func_mode); //TX
	status |= SetPinMode(Port_A, Pin_3, Alt_func_mode); //RX

	if(status!=Success){
		return status;
	}
	USART_Clock(Enabled, Usart_2);


	GpioSetAlternativeFunction(Port_A, Pin_2, SPI3_I2S3_USART1_USART2); //alt 2
	GpioSetAlternativeFunction(Port_A, Pin_3, SPI3_I2S3_USART1_USART2); //alt 2

	status = set_baud_rate(Usart_2, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}

	return Success;
}





/*______________________________________USART_6___________________________*/
Status_code_t Init_UART6(usart_config_t USART_config){

	Status_code_t status = Success;

	status = SetPinMode(Port_C, Pin_6, Alt_func_mode); //TX
	status |= SetPinMode(Port_C, Pin_7, Alt_func_mode); //RX

	if(status!=Success){
		return status;
	}
	USART_Clock(Enabled, Usart_6);


	GpioSetAlternativeFunction(Port_C, Pin_6, USART_6); //alt 2
	GpioSetAlternativeFunction(Port_C, Pin_7, USART_6); //alt 2

	status = set_baud_rate(Usart_6, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}


	return Success;
}

/*NOTE: firts deinit uart alternative before configure the uart*/
Status_code_t set_baud_rate(usart_alternative_t USART, uint32_t baudrate){

	memoryMapAddress_t volatile MainAddress = USART1_ADDRESS;
	uint32_t total_clock_ticks =0;

	if(baudrate<300 || baudrate > 460800U ){
		return USART_baud_rate_out_of_limit;
	}
	switch(USART){
	case Usart_1:
		MainAddress = USART1_ADDRESS;
		break;
	case Usart_2:
		MainAddress = USART2_ADDRESS;
		break;
	case Usart_6:
		MainAddress = USART6_ADDRESS;
		break;
	default:
		return USART_alternative_not_suported;
		break;
	}
	uint32_t volatile *USART_BRR_Reg = (uint32_t volatile*)(MainAddress + USART_BRR);
	total_clock_ticks = (uint32_t)(BOARD_CLOCK/baudrate);

	*USART_BRR_Reg = 0;
	*USART_BRR_Reg = total_clock_ticks;

	return Success;
}


