/*
 * uart.h
 *
 *  Created on: Oct 26, 2024
 *      Author: sebas
 */

#ifndef UART_H_
#define UART_H_

#include "system_settings.h"


#define USART1_HANDLER	USART1_IRQHandler
#define USART2_HANDLER	USART2_IRQHandler
#define USART6_HANDLER	USART6_IRQHandler


#define RX_ENABLE_BIT_POS			(2U)
#define DR_MAX_VALUE				(0xFF)
#define STRING_END_MAX_LENGHT 		(10)
#define STRING_END_STANDAR	 		(2)
#define MAX_LENGHT_GLOBAL_BUFFER	(4096)



typedef enum{
	Usart_1 = 4U, //TX:(B6)  RX:(B7)
	Usart_6 = 5U,  //TX:(C6)  RX:(C7)
	Usart_2 = 17U, //TX:(A2)  RX:(A3) //USED BY LINKER

}usart_alternative_t;


typedef enum{
	Synchronous,
	Asynchronous,
}usart_mode_t;

typedef enum{
	disabled_TX_and_RX = (0U<<2U),
	only_RX = 			 (1U<<2U),
	only_TX = 			 (2U<<2U),
	enable_TX_and_RX = 	 (3U<<2U),
}usart_data_direction_t;

typedef enum{
	None,
	Even,
	Odd,
}usart_parity_t;

typedef enum{

	Data_8_bits,
	Data_9_bits,
}usart_data_bits_t;

typedef enum{
	Stop_1_bits,
	Stop_0_5_bits,
	Stop_2_bits,
}usart_stop_bits_t;


typedef enum{
	Usart_1_HFC, //CTS:(A11)  RTS:(A12) CK:A8
	Usart_2_HFC, //CTS:(A0)  RTS:(A1)  CK:A4
	Usart_6_HFC, //CK:C8


}usart_hardware_flow_ctrl_t;

typedef enum{
	CPOL_low = 0U,
	CPOL_high
}usart_clock_polarity_t;

typedef enum{
	CPHA_first_edge = 0U,
	CPHA_second_edge,
}usart_clock_phase_t;

typedef enum{
	LBCL_not_output = 0U,
	LBCL_is_output,
}usart_last_bit_clock_t;

typedef struct{
	usart_clock_polarity_t CPOL;
	usart_clock_phase_t CPHA;
	usart_last_bit_clock_t LBCL;
}usart_synchronous_config_t;

typedef struct{
	uint32_t baud_rate;
	usart_mode_t mode;
	usart_synchronous_config_t *synchronous_config;  //if is Asynchronous put NULL
	usart_data_direction_t data_direction;
	usart_parity_t parity;
	usart_data_bits_t data_bits;
	usart_stop_bits_t stop_bits;
}usart_config_t;


//typedef struct{
//	uint32_t baud_rate;
//	usart_parity_t parity;
//	usart_data_bits_t data_bits;
//	usart_stop_bits_t stop_bits;
//}usart_INT_config_t;

typedef enum{
	USART_SR =		0x00U,
	USART_DR =		0x04U,
	USART_BRR =		0x08U,
	USART_CR1 = 	0x0CU,
	USART_CR2 = 	0x10U,
	USART_CR3 = 	0x14U,
	USART_GTPR = 	0x18U,


}USART_register_offset_t;

typedef enum{
	 USART_PE= 		(1U<<0U),
	 USART_FE=		(1U<<1U),
	 USART_NF=		(1U<<2U),
	 USART_ORE=		(1U<<3U),
	 USART_IDLE=	(1U<<4U),
	 USART_RXNE=	(1U<<5U),
	 USART_TC=		(1U<<6U),
	 USART_TXE=		(1U<<7U),
	 USART_LBD=		(1U<<8U),
	 USART_CTS=		(1U<<9U),

}USART_SR_t;


typedef enum{
	 USART_DIV_Fraction= 	0U,
	 USART_DIV_Mantissa=	4U,

}USART_BRR_t;


typedef enum{
	 USART_SBK= 	(1U<<0U),
	 USART_RWU=		(1U<<1U),
	 USART_RE=		(1U<<2U),
	 USART_TE=		(1U<<3U),
	 USART_IDLEIE=	(1U<<4U),
	 USART_RXNEIE=	(1U<<5U),
	 USART_TCIE=	(1U<<6U),
	 USART_TXEIE=	(1U<<7U),
	 USART_PEIE=	(1U<<8U),

	 USART_PS=		(1U<<9U),
	 USART_PCE=		(1U<<10U),
	 USART_WAKE=	(1U<<11U),
	 USART_M=		(1U<<12U),
	 USART_UE=		(1U<<13U),
	 USART_OVER8=	(1U<<15U),

}USART_CR1_t;

typedef enum{
	 USART_ADD= 	0U,
	 USART_LBDL=	(1U<<5U),
	 USART_LBDIE=	(1U<<6U),
	 USART_LBCL=	(1U<<8U),
	 USART_CPHA=	(1U<<9U),
	 USART_CPOL=	(1U<<10U),
	 USART_CLKEN=	(1U<<11U),
	 USART_STOP=	12U,
	 USART_LINEN=	(1U<<14U),

}USART_CR2_t;

typedef enum{
	 USART_EIE=		(1U<<0U),
	 USART_IREN=	(1U<<1U),
	 USART_IRLP=	(1U<<2U),
	 USART_HDSEL=	(1U<<3U),
	 USART_NACK=	(1U<<4U),
	 USART_SCEN=	(1U<<5U),
	 USART_DMAR=	(1U<<6U),
	 USART_DMAT=	(1U<<7U),
	 USART_RTSE=	(1U<<8U),
	 USART_CTSE=	(1U<<9U),
	 USART_CTSIE=	(1U<<10U),
	 USART_ONEBIT=	(1U<<11U),

}USART_CR3_t;


typedef enum{
	 USART_PSC=	0U,
	 USART_GT=	8U,

}USART_GTPR_t;

Status_code_t USART_Clock(Enabled_Disabled_t state, usart_alternative_t USART);

void USART1_HANDLER(void);
Status_code_t UART1_Init_RX_Interrupt(usart_config_t USART_INT_config);
Status_code_t UART1_Deinit_RX_Interrupt(void);
Status_code_t UART1_Init(usart_config_t USART_config);
Status_code_t UART1_Deinit(void);
Status_code_t UART1_Write(uint8_t const *data, uint32_t data_lenght, uint16_t timeout);
Status_code_t UART1_Read(uint8_t *data_buff, uint32_t *data_buff_lenght, uint16_t timeout);
Status_code_t UART1_Read_bytes(uint8_t *data_buff, uint32_t expected_data_lenght, uint16_t timeout);

void USART2_HANDLER(void);
Status_code_t UART2_Init_RX_Interrupt(usart_config_t USART_INT_config);
Status_code_t UART2_Deinit_RX_Interrupt(void);
Status_code_t UART2_Init(usart_config_t USART_config);
Status_code_t UART2_Deinit(void);
Status_code_t UART2_Write(uint8_t const *data, uint32_t data_lenght, uint16_t timeout);
Status_code_t UART2_Read(uint8_t *data_buff, uint32_t *data_buff_lenght, uint16_t timeout);
Status_code_t UART2_Read_bytes(uint8_t *data_buff, uint32_t expected_data_lenght, uint16_t timeout);

void USART6_HANDLER(void);
Status_code_t UART6_Init_RX_Interrupt(usart_config_t USART_INT_config);
Status_code_t UART6_Deinit_RX_Interrupt(void);
Status_code_t UART6_Init(usart_config_t USART_config);
Status_code_t UART6_Deinit(void);
Status_code_t UART6_Write(uint8_t const *data, uint32_t data_lenght, uint16_t timeout);
Status_code_t UART6_Read(uint8_t *data_buff, uint32_t *data_buff_lenght, uint16_t timeout);
Status_code_t UART6_Read_bytes(uint8_t *data_buff, uint32_t expected_data_lenght, uint16_t timeout);


Status_code_t USART_set_baud_rate(USARTMapAddr_t USART_Addr, uint32_t baudrate);
void USART_configDirection(USARTMapAddr_t USART_Addr,usart_data_direction_t data_direction);
void USART_CR1_config_bit(USARTMapAddr_t USART_Addr, USART_CR1_t USAR_CR1_bit, Enabled_Disabled_t state);
void USART_set_data_bits(USARTMapAddr_t USART_Addr, usart_data_bits_t data_lenght);
void USART_set_stop_bits(USARTMapAddr_t USART_Addr, usart_stop_bits_t stop_bits);
void USART_set_parity(USARTMapAddr_t USART_Addr, usart_parity_t parity);
void USART_config_mode(USARTMapAddr_t USART_Addr, usart_mode_t USART_mode, usart_synchronous_config_t *synchronous_config);

void USART_enable_Rx_Interrupt(USARTMapAddr_t USART_Addr);
Status_code_t USART_INT_received_status(USARTMapAddr_t USART_Addr);
Status_code_t USART_error_flags_status(USARTMapAddr_t USART_Addr);

void USART_enable(USARTMapAddr_t USART_Addr);
void USART_disabled(USARTMapAddr_t USART_Addr);

void USART_SET_RX_ending_string(uint8_t const * ending_string, uint32_t ending_string_lenght);

#endif /* UART_H_ */
