/*
 * uart.c
 *
 *  Created on: Oct 26, 2024
 *      Author: sebas
 */

#include "uart.h"

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

	USART_Clock(Enabled, Usart_1);


	return Success;
}


/*______________________________________USART_2___________________________*/

Status_code_t Init_UART2(usart_config_t USART_config){

	USART_Clock(Enabled, Usart_1);


	return Success;
}





/*______________________________________USART_6___________________________*/
Status_code_t Init_UART6(usart_config_t USART_config){

	USART_Clock(Enabled, Usart_1);


	return Success;
}


