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
	Ignore = 		0U,
	Hash = 			35U,
	Asterisk = 		42U,
	Question_mark = 63U,
	Slash_O = 		248U,
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

typedef enum{
	USART_SR =		0x00,
	USART_DR =		0x04,
	USART_BRR =		0x08,
	USART_CR1 = 	0x0C,
	USART_CR2 = 	0x10,
	USART_CR3 = 	0x14,
	USART_GTPR = 	0x18,


}USART_register_offset_t;

typedef enum{
	 USART_PE= 		(1u<<0u),
	 USART_FE=		(1u<<1u),
	 USART_NF=		(1u<<2u),
	 USART_ORE=		(1u<<3u),
	 USART_IDLE=	(1u<<4u),
	 USART_RXNE=	(1u<<5u),
	 USART_TC=		(1u<<6u),
	 USART_TXE=		(1u<<7u),
	 USART_LBD=		(1u<<8u),
	 USART_CTS=		(1u<<9u),

}USART_SR_t;


typedef enum{
	 USART_DIV_Fraction= 	0U,
	 USART_DIV_Mantissa=	4U,

}USART_BRR_t;


typedef enum{
	 USART_SBK= 	(1u<<0u),
	 USART_RWU=		(1u<<1u),
	 USART_RE=		(1u<<2u),
	 USART_TE=		(1u<<3u),
	 USART_IDLEIE=	(1u<<4u),
	 USART_RXNEIE=	(1u<<5u),
	 USART_TCIE=	(1u<<6u),
	 USART_TXEIE=	(1u<<7u),
	 USART_PEIE=	(1u<<8u),

	 USART_PS=		(1u<<9u),
	 USART_PCE=		(1u<<10u),
	 USART_WAKE=	(1u<<11u),
	 USART_M=		(1u<<12u),
	 USART_UE=		(1u<<13u),
	 USART_OVER8=	(1u<<15u),

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

Status_code_t Init_UART1(usart_config_t USART_config);


Status_code_t Init_UART2(usart_config_t USART_config);


Status_code_t Init_UART6(usart_config_t USART_config);

Status_code_t set_baud_rate(usart_alternative_t USART, uint32_t baudrate);


#endif /* UART_H_ */
