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
	USARTMapAddr_t volatile MainAddress = USART1_ADDRESS;

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

	status = USART_set_baud_rate(USART1_ADDRESS, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}
	USART_configDirection(USART1_ADDRESS,USART_config.data_direction);

	USART_set_data_bits(USART1_ADDRESS,USART_config.data_bits);
	USART_set_parity(USART1_ADDRESS, USART_config.parity);


	USART_enable(USART1_ADDRESS);


	return Success;
}


Status_code_t UART1_Write(uint8_t *data,uint32_t data_lenght){

	return Success;
}

Status_code_t UART1_Read(uint8_t *data_buff, uint8_t data_buff_lenght){

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

	status = USART_set_baud_rate(USART2_ADDRESS, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}
	USART_configDirection(USART2_ADDRESS,USART_config.data_direction);

	USART_set_data_bits(USART2_ADDRESS,USART_config.data_bits);
	USART_set_parity(USART2_ADDRESS, USART_config.parity);


	USART_enable(USART2_ADDRESS);


	return Success;
}

Status_code_t UART2_Write(uint8_t *data, uint32_t data_lenght){
	uint32_t volatile *USART_SR_Reg = (uint32_t volatile*)(USART2_ADDRESS + USART_SR);
	uint32_t volatile *USART_DR_Reg = (uint32_t volatile*)(USART2_ADDRESS + USART_DR);


	while(!(USART_TXE & *USART_SR_Reg)){}
	*USART_DR_Reg = ((*data) & DR_MAX_VALUE);

	return Success;
}

Status_code_t UART2_Read(uint8_t *data_buff, uint8_t data_buff_lenght){

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

	status = USART_set_baud_rate(USART6_ADDRESS, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}
	USART_configDirection(USART6_ADDRESS,USART_config.data_direction);

	USART_set_data_bits(USART6_ADDRESS,USART_config.data_bits);
	USART_set_parity(USART6_ADDRESS, USART_config.parity);



	USART_enable(USART6_ADDRESS);

	return Success;
}

Status_code_t UART6_Write(uint8_t *data,uint32_t data_lenght){

	return Success;
}

Status_code_t UART6_Read(uint8_t *data_buff, uint8_t data_buff_lenght){

	return Success;

}




/*_____________________________Others protocol functions______________________________________*/
/*NOTE: firts deinit uart alternative before configure the uart*/
Status_code_t USART_set_baud_rate(USARTMapAddr_t USART_Addr, uint32_t baudrate){

	uint32_t total_clock_ticks =0;
	uint32_t volatile *USART_BRR_Reg = (uint32_t volatile*)(USART_Addr + USART_BRR);

	if(baudrate<300 || baudrate > 460800U ){
		return USART_baud_rate_out_of_limit;
	}
	total_clock_ticks = (uint32_t)(BOARD_CLOCK/baudrate);

	*USART_BRR_Reg = 0;
	*USART_BRR_Reg = total_clock_ticks;

	return Success;
}


void USART_configDirection(USARTMapAddr_t USART_Addr,usart_data_direction_t data_direction){

	uint32_t volatile *USART_CR1_Reg = (uint32_t volatile*)(USART_Addr + USART_CR1);

	*USART_CR1_Reg &= ~(Clear_two_bits<<RX_ENABLE_BIT_POS);
	*USART_CR1_Reg |= (data_direction);

}

void USART_CR1_config_bit(USARTMapAddr_t USART_Addr, USART_CR1_t USAR_CR1_bit, Enabled_Disabled_t state){
	uint32_t volatile *USART_CR1_Reg = (uint32_t volatile*)(USART_Addr + USART_CR1);

    *USART_CR1_Reg = (state) ? (*USART_CR1_Reg | (USAR_CR1_bit)) : (*USART_CR1_Reg & ~(USAR_CR1_bit));

}

void USART_set_data_bits(USARTMapAddr_t USART_Addr, usart_data_bits_t data_lenght){
	uint32_t volatile *USART_CR1_Reg = (uint32_t volatile*)(USART_Addr + USART_CR1);
	*USART_CR1_Reg &= ~(USART_M);
	*USART_CR1_Reg = (data_lenght) ? (*USART_CR1_Reg | (USART_M)) : (*USART_CR1_Reg  & ~(USART_M));

}

void USART_set_stop_bits(USARTMapAddr_t USART_Addr, usart_stop_bits_t stop_bits){
	uint32_t volatile *USART_CR2_Reg = (uint32_t volatile*)(USART_Addr + USART_CR2);
	*USART_CR2_Reg &= ~(Clear_two_bits<<USART_STOP);
	*USART_CR2_Reg |= (stop_bits<<USART_STOP);

}

void USART_set_parity(USARTMapAddr_t USART_Addr, usart_parity_t parity){
	uint32_t volatile *USART_CR1_Reg = (uint32_t volatile*)(USART_Addr + USART_CR1);

	*USART_CR1_Reg &= ~(USART_PCE);
	*USART_CR1_Reg &= ~(USART_PS);

	if(parity){
		*USART_CR1_Reg |= (USART_PCE);
		if(parity == Odd){ //if is even is already configure to cero
			*USART_CR1_Reg |= (USART_PS);
		}
	}

}



void USART_enable(USARTMapAddr_t USART_Addr){

	uint32_t volatile *USART_CR1_Reg = (uint32_t volatile*)(USART_Addr + USART_CR1);

	*USART_CR1_Reg &= ~(USART_UE);
	*USART_CR1_Reg |= (USART_UE);

}
void USART_disabled(USARTMapAddr_t USART_Addr){

	uint32_t volatile *USART_CR1_Reg = (uint32_t volatile*)(USART_Addr + USART_CR1);
	*USART_CR1_Reg &= ~(USART_UE);

}

