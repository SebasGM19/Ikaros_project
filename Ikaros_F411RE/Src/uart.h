/*
 * uart.h
 *
 *  Created on: Oct 26, 2024
 *      Author: sebas
 */

#ifndef UART_H_
#define UART_H_

#include "system_settings.h"



typedef enum{
	Usart_1 = 4U, //TX:(B6)  RX:(B7)
	Usart_6 = 5U,  //TX:(C6)  RX:(C7)
	Usart_2 = 17U, //TX:(A2)  RX:(A3) //USED BY LINKER

}usart_alternative_t;


typedef enum{
	Synchronous,
	Asynchronous,
	Hardware_flow_control,
}usart_mode_t;

typedef enum{
	None,
	Even,
	Odd,
}usart_parity_t;

typedef enum{
	Ignore = 0U,
	Hash = 35U,
	Asterisk = 42U,
	Question_mark = 63U,
	Slash_O = 248U,
}usart_parity_error_char_t;



typedef enum{
	Data_5_bits,
	Data_6_bits,
	Data_7_bits,
	Data_8_bits,
}usart_data_bits_t;

typedef enum{
	Stop_1_bits,
	Stop_2_bits,
}usart_stop_bits_t;


typedef enum{
	Usart_1_HFC, //CTS:(A11)  RTS:(A12) CK:A8
	Usart_2_HFC, //CTS:(A0)  RTS:(A1)  CK:A4
	Usart_6_HFC, //CK:C8


}usart_hardware_flow_ctrl_t;



typedef struct{
	uint32_t baud_rate;
	usart_mode_t mode;
	usart_parity_t parity;
	usart_parity_error_char_t parity_char;
	usart_data_bits_t data_bits;
	usart_stop_bits_t stop_bits;
}usart_config_t;


Status_code_t USART_Clock(Enabled_Disabled_t state, usart_alternative_t USART);

Status_code_t Init_UART1(usart_config_t USART_config);


Status_code_t Init_UART2(usart_config_t USART_config);


Status_code_t Init_UART6(usart_config_t USART_config);




#endif /* UART_H_ */
