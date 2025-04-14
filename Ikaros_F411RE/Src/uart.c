/*
 * uart.c
 *
 *  Created on: Oct 26, 2024
 *      Author: sebas
 */

#include "uart.h"
#include "gpios.h"
#include "timers.h"


uint8_t static string_end[STRING_END_MAX_LENGHT];
uint8_t static string_end_lenght = 2;
uint8_t static const default_str_end[STRING_END_STANDAR]={"\r\n"};

uint8_t static usart_global_buffer[MAX_LENGHT_GLOBAL_BUFFER]={0};
uint32_t static global_data_count = 0;


uint8_t static usart1_interrupt_buffer[MAX_LENGHT_GLOBAL_BUFFER]={0};
int32_t static usart1_interrupt_data_count = 0;

uint8_t static usart2_interrupt_buffer[MAX_LENGHT_GLOBAL_BUFFER]={0};
int32_t static usart2_interrupt_data_count = 0;

uint8_t static usart6_interrupt_buffer[MAX_LENGHT_GLOBAL_BUFFER]={0};
int32_t static usart6_interrupt_data_count = 0;

Status_code_t USART_Clock(Enabled_Disabled_t state, usart_alternative_t USART){
	RCC_offset_t RccOffset = RCC_OFFSET_APB2ENR;

	if(USART == Usart_2){
		RccOffset = RCC_OFFSET_APB1ENR;
	}

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RccOffset);

    *pClockControlReg = (state) ? (*pClockControlReg | (Enabled << USART)) : (*pClockControlReg & ~(Enabled << USART));

    return Success;

}


/*______________________________________USART_1___________________________*/


void USART1_HANDLER(void){
	Status_code_t status = Success;
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART1_ADDRESS + USART_DR);
	status= USART_INT_received_status(USART1_ADDRESS);

	if(status==USART_read_data_not_empty){
	//to save the data in to individual buff
		usart1_interrupt_buffer[usart1_interrupt_data_count] =(uint8_t)(*USART_DR_Reg);
		if(usart1_interrupt_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			usart1_interrupt_data_count = 0;
		}else{
			usart1_interrupt_data_count++;
		}

	//to save the data in to a global buff
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}

/*_____________________________DEVELOP_EXTRA_CODE_HERE_______________________________*/



/*_____________________________________TO_HERE_______________________________________*/

	}else{
        (void)*USART_DR_Reg;
	}
}

Status_code_t UART1_Init_RX_Interrupt(usart_config_t USART_INT_config){

		Status_code_t status = Success;

		SetPinMode(Port_B, Pin_6, Alt_func_mode); //TX
		SetPinMode(Port_B, Pin_7, Alt_func_mode); //RX

		GpioSetAlternativeFunction(Port_B, Pin_6, SPI3_I2S3_USART1_USART2);
		GpioSetAlternativeFunction(Port_B, Pin_7, SPI3_I2S3_USART1_USART2);

		USART_Clock(Enabled, Usart_1);
		USART_SET_RX_ending_string(default_str_end, STRING_END_STANDAR);

		status = USART_set_baud_rate(USART1_ADDRESS, USART_INT_config.baud_rate);
		if(status!=Success){
			return status;
		}
		USART_configDirection(USART1_ADDRESS,USART_INT_config.data_direction);
		USART_config_mode(USART1_ADDRESS, USART_INT_config.mode, USART_INT_config.synchronous_config);

		USART_set_data_bits(USART1_ADDRESS,USART_INT_config.data_bits);
		USART_set_stop_bits(USART1_ADDRESS, USART_INT_config.stop_bits);
		USART_set_parity(USART1_ADDRESS, USART_INT_config.parity);

		USART_enable_Rx_Interrupt(USART1_ADDRESS);

		USART_enable(USART1_ADDRESS);

		NVIC_EnableIRQ(USART1_IRQn);

		return Success;

}

Status_code_t UART1_Deinit_RX_Interrupt(void){

	USART_disabled(USART1_ADDRESS);
	USART_Clock(Disabled, Usart_1);
	NVIC_DisableIRQ(USART1_IRQn);

	return Success;

}

Status_code_t UART1_Init(usart_config_t USART_config){
	Status_code_t status = Success;

	SetPinMode(Port_B, Pin_6, Alt_func_mode); //TX
	SetPinMode(Port_B, Pin_7, Alt_func_mode); //RX

	GpioSetAlternativeFunction(Port_B, Pin_6, SPI3_I2S3_USART1_USART2);
	GpioSetAlternativeFunction(Port_B, Pin_7, SPI3_I2S3_USART1_USART2);
	Gpio_Output_type(Port_B, Pin_6, Push_pull); //push_pull only in TX line

	USART_Clock(Enabled, Usart_1);
	USART_SET_RX_ending_string(default_str_end, STRING_END_STANDAR);

	status = USART_set_baud_rate(USART1_ADDRESS, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}
	USART_configDirection(USART1_ADDRESS,USART_config.data_direction);
	USART_config_mode(USART1_ADDRESS, USART_config.mode, USART_config.synchronous_config);

	USART_set_data_bits(USART1_ADDRESS,USART_config.data_bits);
	USART_set_stop_bits(USART1_ADDRESS, USART_config.stop_bits);
	USART_set_parity(USART1_ADDRESS, USART_config.parity);


	USART_enable(USART1_ADDRESS);


	return Success;
}
Status_code_t UART1_Deinit(void){ //pendiente sebas
	USART_enable(USART1_ADDRESS);
	USART_Clock(Disabled, Usart_1);
	return Success;
}

Status_code_t UART1_Write(uint8_t const *data, uint32_t data_lenght, uint16_t timeout){

	Status_code_t status = Success;
	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART1_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART1_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();

	for(uint32_t i = 0; i<data_lenght; i++){

		while( (!(USART_TXE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){}

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		*USART_DR_Reg = (data[i] & DR_MAX_VALUE);
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	return Success;
}

Status_code_t UART1_Read(uint8_t *data_buff, uint32_t *data_buff_lenght, uint16_t timeout){

	Status_code_t status = Success;
	uint32_t data_arrive_count = 0;
	uint8_t exit_count = 0;

	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART1_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART1_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();
	do{

		while( (!(USART_RXNE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){} //wait until receive a char

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		status = USART_error_flags_status(USART1_ADDRESS);
        if (status != Success) {
            TIM11_Deinit();
            TIM11_clear_interrupt_flag();
            return status;
        }
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}


		if((uint8_t)(*USART_DR_Reg) == string_end[exit_count]){
			exit_count++;
		}else{
			data_buff[data_arrive_count] = (uint8_t)(*USART_DR_Reg);
			data_arrive_count++;
			exit_count=0;
		}

	}while(exit_count != string_end_lenght && status== Success);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	(*data_buff_lenght) = data_arrive_count;

	return status;

}

Status_code_t UART1_Read_bytes(uint8_t *data_buff, uint32_t expected_data_lenght, uint16_t timeout){

	Status_code_t status = Success;
	uint32_t data_arrive_count = 0;

	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART1_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART1_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();
	do{

		while( (!(USART_RXNE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){} //need to develop the timeout validation

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		status = USART_error_flags_status(USART1_ADDRESS);
        if (status != Success) {
            TIM11_Deinit();
            TIM11_clear_interrupt_flag();
            return status;
        }
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}

		data_buff[data_arrive_count] = (uint8_t)(*USART_DR_Reg);
		data_arrive_count++;

	}while(data_arrive_count != expected_data_lenght && status== Success);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();


	return status;

}


/*______________________________________USART_2___________________________*/

void USART2_HANDLER(void){
	Status_code_t status = Success;
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART2_ADDRESS + USART_DR);
	status= USART_INT_received_status(USART2_ADDRESS);

	if(status==USART_read_data_not_empty){
	//to save the data in to individual buff
		usart2_interrupt_buffer[usart2_interrupt_data_count] =(uint8_t)(*USART_DR_Reg);
		if(usart2_interrupt_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			usart2_interrupt_data_count = 0;
		}else{
			usart2_interrupt_data_count++;
		}

	//to save the data in to a global buff
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}

/*_____________________________DEVELOP_EXTRA_CODE_HERE_______________________________*/
//	print_message((uint8_t)(*USART_DR_Reg));


/*_____________________________________TO_HERE_______________________________________*/

	}else{
        (void)*USART_DR_Reg;
	}


}

///*dummy function for a test*/
//uint32_t finish_count = 0;
//uint32_t msg_count =0;
//void print_message(uint8_t character){
//
//	uint8_t data_msg_get[30]={};
//	uint32_t point_to_data =0;
//	int32_t aux_point_value =0;
//
//	if(default_str_end[finish_count]== character){
//		finish_count++;
//		if(finish_count >=2){
//
//			lcd_printXY(0, 0,"MSG received:   ", 16);
//			lcd_printXY(0, 1,"                ", 16);
//
//			for(uint32_t i =0; i<msg_count; i++){
//
//				aux_point_value = (int32_t)(usart2_interrupt_data_count - ((msg_count+2)-i));
//
//				if( aux_point_value  < 0){
//					point_to_data = MAX_LENGHT_GLOBAL_BUFFER - ((msg_count+2)-i) + usart2_interrupt_data_count;
//				}
//				if( aux_point_value >= 0){
//							point_to_data = (usart2_interrupt_data_count - ((msg_count+2)-i) );
//				}
//				data_msg_get[i] = usart2_interrupt_buffer[point_to_data];
//			}
//
//			lcd_printXY(0, 1,data_msg_get, msg_count);
//
//			TIM5_Start();
//			finish_count=0;
//			msg_count=0;
//		}
//	}else{
//		msg_count++;
//	}
//}


Status_code_t UART2_Init_RX_Interrupt(usart_config_t USART_INT_config){

		Status_code_t status = Success;

		SetPinMode(Port_A, Pin_2, Alt_func_mode); //TX
		SetPinMode(Port_A, Pin_3, Alt_func_mode); //RX

		GpioSetAlternativeFunction(Port_A, Pin_2, SPI3_I2S3_USART1_USART2);
		GpioSetAlternativeFunction(Port_A, Pin_3, SPI3_I2S3_USART1_USART2);

		USART_Clock(Enabled, Usart_2);
		USART_SET_RX_ending_string(default_str_end, STRING_END_STANDAR);

		status = USART_set_baud_rate(USART2_ADDRESS, USART_INT_config.baud_rate);
		if(status!=Success){
			return status;
		}
		USART_configDirection(USART2_ADDRESS,USART_INT_config.data_direction);

		USART_config_mode(USART2_ADDRESS, USART_INT_config.mode, USART_INT_config.synchronous_config);

		USART_set_data_bits(USART2_ADDRESS,USART_INT_config.data_bits);
		USART_set_stop_bits(USART2_ADDRESS, USART_INT_config.stop_bits);
		USART_set_parity(USART2_ADDRESS, USART_INT_config.parity);

		USART_enable_Rx_Interrupt(USART2_ADDRESS);

		USART_enable(USART2_ADDRESS);

		NVIC_EnableIRQ(USART2_IRQn);

		return Success;

}
Status_code_t UART2_Deinit_RX_Interrupt(void){
	USART_disabled(USART2_ADDRESS);
	USART_Clock(Disabled, Usart_2);
	NVIC_DisableIRQ(USART2_IRQn);
	return Success;

}

Status_code_t UART2_Init(usart_config_t USART_config){
	Status_code_t status = Success;

	SetPinMode(Port_A, Pin_2, Alt_func_mode); //TX
	SetPinMode(Port_A, Pin_3, Alt_func_mode); //RX

	GpioSetAlternativeFunction(Port_A, Pin_2, SPI3_I2S3_USART1_USART2);
	GpioSetAlternativeFunction(Port_A, Pin_3, SPI3_I2S3_USART1_USART2);
	Gpio_Output_type(Port_A, Pin_2, Push_pull); //push_pull only in TX line

	USART_Clock(Enabled, Usart_2);
	USART_SET_RX_ending_string(default_str_end, STRING_END_STANDAR);

	status = USART_set_baud_rate(USART2_ADDRESS, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}
	USART_configDirection(USART2_ADDRESS,USART_config.data_direction);

	USART_config_mode(USART2_ADDRESS, USART_config.mode, USART_config.synchronous_config);

	USART_set_data_bits(USART2_ADDRESS,USART_config.data_bits);
	USART_set_stop_bits(USART2_ADDRESS, USART_config.stop_bits);
	USART_set_parity(USART2_ADDRESS, USART_config.parity);


	USART_enable(USART2_ADDRESS);


	return Success;
}

Status_code_t UART2_Deinit(void){

	USART_enable(USART6_ADDRESS);
	USART_Clock(Disabled, Usart_6);

	return Success;
}

Status_code_t UART2_Write(uint8_t const *data, uint32_t data_lenght, uint16_t timeout){

	Status_code_t status = Success;
	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART2_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART2_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();

	for(uint32_t i = 0; i<data_lenght; i++){

		while( (!(USART_TXE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){}

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		*USART_DR_Reg = (data[i] & DR_MAX_VALUE);
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	return Success;
}

Status_code_t UART2_Read(uint8_t *data_buff, uint32_t *data_buff_lenght, uint16_t timeout){

	Status_code_t status = Success;
	uint32_t data_arrive_count = 0;
	uint8_t exit_count = 0;

	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART2_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART2_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();
	do{

		while( (!(USART_RXNE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){} //need to develop the timeout validation

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}
		status = USART_error_flags_status(USART2_ADDRESS);
        if (status != Success) {
            TIM11_Deinit();
            TIM11_clear_interrupt_flag();
            return status;
        }
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}


		if((uint8_t)(*USART_DR_Reg) == string_end[exit_count]){
			exit_count++;
		}else{
			data_buff[data_arrive_count] = (uint8_t)(*USART_DR_Reg);
			data_arrive_count++;
			exit_count=0;
		}

	}while(exit_count != string_end_lenght && status == Success);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	(*data_buff_lenght) = data_arrive_count;

	return status;

}

Status_code_t UART2_Read_bytes(uint8_t *data_buff, uint32_t expected_data_lenght, uint16_t timeout){

	Status_code_t status = Success;
	uint32_t data_arrive_count = 0;

	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART2_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART2_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();
	do{

		while( (!(USART_RXNE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){} //need to develop the timeout validation

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		status = USART_error_flags_status(USART2_ADDRESS);
        if (status != Success) {
            TIM11_Deinit();
            TIM11_clear_interrupt_flag();
            return status;
        }
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}

		data_buff[data_arrive_count] = (uint8_t)(*USART_DR_Reg);
		data_arrive_count++;

	}while(data_arrive_count != expected_data_lenght && status == Success);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();


	return status;

}



/*______________________________________USART_6___________________________*/

void USART6_HANDLER(void){
	Status_code_t status = Success;
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART6_ADDRESS + USART_DR);
	status= USART_INT_received_status(USART6_ADDRESS);

	if(status==USART_read_data_not_empty){
	//to save the data in to individual buff
		usart6_interrupt_buffer[usart6_interrupt_data_count] =(uint8_t)(*USART_DR_Reg);
		if(usart6_interrupt_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			usart6_interrupt_data_count = 0;
		}else{
			usart6_interrupt_data_count++;
		}

	//to save the data in to a global buff
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}

/*_____________________________DEVELOP_EXTRA_CODE_HERE_______________________________*/



/*_____________________________________TO_HERE_______________________________________*/

	}else{
        (void)*USART_DR_Reg;
	}
}

Status_code_t UART6_Init_RX_Interrupt(usart_config_t USART_INT_config){

	Status_code_t status = Success;

	SetPinMode(Port_C, Pin_6, Alt_func_mode); //TX
	SetPinMode(Port_C, Pin_7, Alt_func_mode); //RX

	GpioSetAlternativeFunction(Port_C, Pin_6, USART_6);
	GpioSetAlternativeFunction(Port_C, Pin_7, USART_6);

	USART_Clock(Enabled, Usart_6);
	USART_SET_RX_ending_string(default_str_end, STRING_END_STANDAR);

	status = USART_set_baud_rate(USART6_ADDRESS, USART_INT_config.baud_rate);
	if(status!=Success){
		return status;
	}
	USART_configDirection(USART6_ADDRESS,USART_INT_config.data_direction);

	USART_config_mode(USART6_ADDRESS, USART_INT_config.mode, USART_INT_config.synchronous_config);//aqui configurar internemtente los otros pines de sck y el otro


	USART_set_data_bits(USART6_ADDRESS,USART_INT_config.data_bits);
	USART_set_stop_bits(USART6_ADDRESS, USART_INT_config.stop_bits);
	USART_set_parity(USART6_ADDRESS, USART_INT_config.parity);

	USART_enable_Rx_Interrupt(USART6_ADDRESS);

	USART_enable(USART6_ADDRESS);

	NVIC_EnableIRQ(USART6_IRQn);

	return Success;
}

Status_code_t UART6_Deinit_RX_Interrupt(void){

	USART_disabled(USART6_ADDRESS);
	USART_Clock(Disabled, Usart_6);
	NVIC_DisableIRQ(USART6_IRQn);
	return Success;

}

Status_code_t UART6_Init(usart_config_t USART_config){

	Status_code_t status = Success;

	SetPinMode(Port_C, Pin_6, Alt_func_mode); //TX
	SetPinMode(Port_C, Pin_7, Alt_func_mode); //RX

	GpioSetAlternativeFunction(Port_C, Pin_6, USART_6);
	GpioSetAlternativeFunction(Port_C, Pin_7, USART_6);
	Gpio_Output_type(Port_C, Pin_6, Push_pull); //push_pull only in TX line

	USART_Clock(Enabled, Usart_6);
	USART_SET_RX_ending_string(default_str_end, STRING_END_STANDAR);

	status = USART_set_baud_rate(USART6_ADDRESS, USART_config.baud_rate);
	if(status!=Success){
		return status;
	}
	USART_configDirection(USART6_ADDRESS,USART_config.data_direction);

	USART_config_mode(USART6_ADDRESS, USART_config.mode, USART_config.synchronous_config);//aqui configurar internemtente los otros pines de sck y el otro


	USART_set_data_bits(USART6_ADDRESS,USART_config.data_bits);
	USART_set_stop_bits(USART6_ADDRESS, USART_config.stop_bits);
	USART_set_parity(USART6_ADDRESS, USART_config.parity);


	USART_enable(USART6_ADDRESS);

	return Success;
}

Status_code_t UART6_Deinit(void){
	USART_enable(USART6_ADDRESS);
	USART_Clock(Disabled, Usart_6);
	return Success;
}

Status_code_t UART6_Write(uint8_t const *data, uint32_t data_lenght, uint16_t timeout){

	Status_code_t status = Success;
	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART6_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART6_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();

	for(uint32_t i = 0; i<data_lenght; i++){

		while( (!(USART_TXE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){}

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		*USART_DR_Reg = (data[i] & DR_MAX_VALUE);
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	return Success;
}

Status_code_t UART6_Read(uint8_t *data_buff, uint32_t *data_buff_lenght, uint16_t timeout){

	Status_code_t status = Success;
	uint32_t data_arrive_count = 0;
	uint8_t exit_count = 0;

	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART6_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART6_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();
	do{

		while( (!(USART_RXNE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){}

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}
		status = USART_error_flags_status(USART6_ADDRESS);
        if (status != Success) {
            TIM11_Deinit();
            TIM11_clear_interrupt_flag();
            return status;
        }
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}


		if((uint8_t)(*USART_DR_Reg) == string_end[exit_count]){
			exit_count++;
		}else{
			data_buff[data_arrive_count] = (uint8_t)(*USART_DR_Reg);
			data_arrive_count++;
			exit_count=0;
		}

	}while(exit_count != string_end_lenght && status == Success);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	(*data_buff_lenght) = data_arrive_count;

	return status;

}

Status_code_t UART6_Read_bytes(uint8_t *data_buff, uint32_t expected_data_lenght, uint16_t timeout){

	Status_code_t status = Success;
	uint32_t data_arrive_count = 0;

	__I uint32_t * const USART_SR_Reg = (__I uint32_t * const)(USART6_ADDRESS + USART_SR);
	__IO uint32_t *USART_DR_Reg = (__IO uint32_t *)(USART6_ADDRESS + USART_DR);

	status = TIM11_Init(timeout);
	if(status != Success){
		return status;
	}
	TIM11_Start();
	do{

		while( (!(USART_RXNE & (*USART_SR_Reg))) && !TIM11_GET_interrupt_flag_status() ){}

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		status = USART_error_flags_status(USART6_ADDRESS);
        if (status != Success) {
            TIM11_Deinit();
            TIM11_clear_interrupt_flag();
            return status;
        }
		usart_global_buffer[global_data_count] =(uint8_t)(*USART_DR_Reg);
		if(global_data_count == MAX_LENGHT_GLOBAL_BUFFER-1){
			global_data_count = 0;
		}else{
			global_data_count++;
		}

		data_buff[data_arrive_count] = (uint8_t)(*USART_DR_Reg);
		data_arrive_count++;

	}while(data_arrive_count != expected_data_lenght && status == Success);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();


	return status;

}



/*_____________________________Others protocol functions______________________________________*/
/*NOTE: first deinit uart alternative before configure the uart*/

Status_code_t USART_set_baud_rate(USARTMapAddr_t USART_Addr, uint32_t baudrate){

	__IO uint32_t *USART_BRR_Reg = (__IO uint32_t *)(USART_Addr + USART_BRR);

	if(baudrate<300U || baudrate > 460800U ){
		return USART_baud_rate_out_of_limit;
	}
	*USART_BRR_Reg |= (uint32_t)(round((BOARD_CLOCK/baudrate)));

	return Success;
}


void USART_configDirection(USARTMapAddr_t USART_Addr,usart_data_direction_t data_direction){

	__IO uint32_t *USART_CR1_Reg = (__IO uint32_t *)(USART_Addr + USART_CR1);

	*USART_CR1_Reg &= ~(Clear_two_bits<<RX_ENABLE_BIT_POS);
	*USART_CR1_Reg |= (data_direction);

}

void USART_CR1_config_bit(USARTMapAddr_t USART_Addr, USART_CR1_t USAR_CR1_bit, Enabled_Disabled_t state){
	__IO uint32_t *USART_CR1_Reg = (__IO uint32_t *)(USART_Addr + USART_CR1);

    *USART_CR1_Reg = (state) ? (*USART_CR1_Reg | (USAR_CR1_bit)) : (*USART_CR1_Reg & ~(USAR_CR1_bit));

}

void USART_set_data_bits(USARTMapAddr_t USART_Addr, usart_data_bits_t data_lenght){
	__IO uint32_t *USART_CR1_Reg = (__IO uint32_t *)(USART_Addr + USART_CR1);
	*USART_CR1_Reg &= ~(USART_M);
	*USART_CR1_Reg = (data_lenght) ? (*USART_CR1_Reg | (USART_M)) : (*USART_CR1_Reg  & ~(USART_M));

}

void USART_set_stop_bits(USARTMapAddr_t USART_Addr, usart_stop_bits_t stop_bits){
	__IO uint32_t *USART_CR2_Reg = (__IO uint32_t *)(USART_Addr + USART_CR2);
	*USART_CR2_Reg &= ~(Clear_two_bits<<USART_STOP);
	*USART_CR2_Reg |= (stop_bits<<USART_STOP);

}

void USART_set_parity(USARTMapAddr_t USART_Addr, usart_parity_t parity){
	__IO uint32_t *USART_CR1_Reg = (__IO uint32_t *)(USART_Addr + USART_CR1);

	*USART_CR1_Reg &= ~(USART_PCE);
	*USART_CR1_Reg &= ~(USART_PS);

	if(parity){
		*USART_CR1_Reg |= (USART_PCE);
		if(parity == Odd){ //if is even is already configure to cero
			*USART_CR1_Reg |= (USART_PS);
		}
	}

}

void USART_config_mode(USARTMapAddr_t USART_Addr, usart_mode_t USART_mode, usart_synchronous_config_t *synchronous_config){
	__IO uint32_t *USART_CR2_Reg = (__IO uint32_t *)(USART_Addr + USART_CR2);

	*USART_CR2_Reg &= ~USART_CLKEN; //by default unable

	if(USART_mode == Synchronous && synchronous_config != NULL){
		switch(USART_Addr){
			case USART2_ADDRESS:
				SetPinMode(Port_A, Pin_4, Alt_func_mode); //CK
				GpioSetAlternativeFunction(Port_A, Pin_4, SPI3_I2S3_USART1_USART2); //CK enable
				break;
			case USART1_ADDRESS:
				SetPinMode(Port_A, Pin_8, Alt_func_mode); //CK
				GpioSetAlternativeFunction(Port_A, Pin_8, SPI3_I2S3_USART1_USART2); //CK enable
				break;
			case USART6_ADDRESS:
				SetPinMode(Port_C, Pin_8, Alt_func_mode); //CK
				GpioSetAlternativeFunction(Port_C, Pin_8, USART_6); //CK enable
				break;
			default:
				break;
		}

		*USART_CR2_Reg |= USART_CLKEN;

		*USART_CR2_Reg &= ~USART_CPOL;//by default unable
		*USART_CR2_Reg &= ~USART_CPHA;//by default unable
		*USART_CR2_Reg &= ~USART_LBCL;//by default unable

		if(synchronous_config->CPOL == CPOL_high){
			*USART_CR2_Reg |= USART_CPOL;
		}
		if(synchronous_config->CPHA == CPHA_second_edge){
			*USART_CR2_Reg |= USART_CPHA;
		}
		if(synchronous_config->LBCL == LBCL_is_output){
			*USART_CR2_Reg |= USART_LBCL;
		}

	}
}

void USART_enable_Rx_Interrupt(USARTMapAddr_t USART_Addr){

	__IO uint32_t *USART_CR1_Reg = (__IO uint32_t *)(USART_Addr + USART_CR1);
	*USART_CR1_Reg |= USART_RXNEIE;

}


Status_code_t USART_INT_received_status(USARTMapAddr_t USART_Addr){

	__IO uint32_t *USART_SR_Reg = (__IO uint32_t *)(USART_Addr + USART_SR);

	if(*USART_SR_Reg & USART_ORE){
		return USART_overrun_error;
	}else if(*USART_SR_Reg & USART_PE){ //parity error
		return USART_parity_error;
	}
	else if(*USART_SR_Reg & USART_FE){ //framing error
		return USART_framing_error;
	}
	else if(*USART_SR_Reg & USART_NF){ //Noise flag
		return USART_noise_error;
	}
	else if(*USART_SR_Reg & USART_RXNE){
		return USART_read_data_not_empty;
	}else{
		return Timeout;
	}

}


Status_code_t USART_error_flags_status(USARTMapAddr_t USART_Addr){

	__IO uint32_t *USART_SR_Reg = (__IO uint32_t *)(USART_Addr + USART_SR);

	if(*USART_SR_Reg & USART_ORE){ //overrun error
		return USART_overrun_error;
	}
	else if(*USART_SR_Reg & USART_PE){ //parity error
		return USART_parity_error;
	}
	else if(*USART_SR_Reg & USART_FE){ //framing error
		return USART_framing_error;
	}
	else if(*USART_SR_Reg & USART_NF){ //Noise flag
		return USART_noise_error;
	}else{
		return Success;
	}

}

void USART_enable(USARTMapAddr_t USART_Addr){

	__IO uint32_t *USART_CR1_Reg = (__IO uint32_t *)(USART_Addr + USART_CR1);

	*USART_CR1_Reg &= ~(USART_UE);
	*USART_CR1_Reg |= (USART_UE);

}

void USART_disabled(USARTMapAddr_t USART_Addr){

	__IO uint32_t *USART_CR1_Reg = (__IO uint32_t *)(USART_Addr + USART_CR1);
	*USART_CR1_Reg &= ~(USART_UE);

}



void USART_SET_RX_ending_string(uint8_t const* ending_string, uint32_t ending_string_lenght){

	memset(string_end,'\0',STRING_END_MAX_LENGHT);
	string_end_lenght = ending_string_lenght;
	strncpy((char *)(string_end), (const char *)(ending_string),string_end_lenght);

}


void clear_global_buff(void){
	memset(usart_global_buffer,'\0',MAX_LENGHT_GLOBAL_BUFFER);
	global_data_count=0;
}
