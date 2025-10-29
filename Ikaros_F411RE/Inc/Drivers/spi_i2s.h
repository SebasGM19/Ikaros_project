/*
 * spi.h
 *
 *  Created on: Aug 31, 2025
 *      Author: sebas
 */

#ifndef DRIVERS_SPI_I2S_H_
#define DRIVERS_SPI_I2S_H_

#include "../Src/system_settings.h"


typedef enum{
	SPI1_I2C1_Alt = 12U,	//12 means RCC APB2 bit 12
	SPI4_I2C4_Alt = 13U,	//13 means RCC APB2 bit 13

	SPI2_I2S2_Alt = 14U, 	//14 means RCC APB1 bit 14
	SPI3_I2S3_Alt = 15U, 	//15 means RCC APB1 bit 15

	SPI5_I2C5_Alt = 20U,	//20 means RCC APB2 bit 20
}SPI_I2S_alternative_t;

typedef enum{
	SPI_CR1 	= 0x00U,
	SPI_CR2 	= 0x04U,
	SPI_SR 		= 0x08U,
	SPI_DR 		= 0x0CU,
	SPI_CRCPR 	= 0x10U,
	SPI_RXCRCR 	= 0x14U,
	SPI_TXCRCR 	= 0x18U,
	SPI_I2SCFGR = 0x1CU,
	SPI_I2SPR 	= 0x20U,

}SPI_I2S_Register_Offset_t;

typedef enum{
	SPI_firts_clk_transition 	= 0U,
	SPI_second_clk_transition 	= 1U,
}SPI_CPHA_t;

typedef enum{
	SPI_high_pol_clk 	= 0U,
	SPI_Low_pol_clk 	= 1U,
}SPI_CPOL_t;

typedef enum{
	SPI_set_as_slave 	= 0U,
	SPI_set_as_master 	= 1U,
}SPI_Master_Select_t;

typedef enum{
	SPI_preescaler_2 	= 0U,
	SPI_preescaler_4 	= 1U,
	SPI_preescaler_8 	= 2U,
	SPI_preescaler_16 	= 3U,
	SPI_preescaler_32 	= 4U,
	SPI_preescaler_64 	= 5U,
	SPI_preescaler_128 	= 6U,
	SPI_preescaler_256 	= 7U,

}SPI_BaudRate_Control_t;

typedef enum{
	SPI_MSB_trans_first = 0U,
	SPI_LSB_trans_first = 1U,
}SPI_CR1_Frame_Format_t;

typedef enum{
	SPI_full_duplex  = 0U,
	SPI_receive_only = 1U,
}SPI_Receive_Mode_t;

typedef enum{
	SPI_8_bit_format  	= 0U,
	SPI_16_bit_format	= 1U,
}SPI_Data_Frame_Format_t;

typedef enum{
	SPI_no_CRC  	= 0U,
	SPI_CRC_phase 	= 1U,
}SPI_CRC_Trans_Next_t;

typedef enum{
	SPI_2_lines_unidirectional  = 0U,
	SPI_1_line_bidirectional	= 1U,
}SPI_Bidirectional_Mode_t;


typedef enum{
	SPI_CPHA		= 0U,
	SPI_CPOL		= 1U,
	SPI_MSTR		= 2U,
	SPI_BR			= 3U,
	SPI_SPE 		= (1U<<6U),
	SPI_LSBFIRST 	= 7U,
	SPI_SSI 		= (1U<<8U),
	SPI_SSM 		= (1U<<9U),
	SPI_RXONLY 		= 10U,
	SPI_DFF 		= 11U,
	SPI_CRCNEXT 	= 12U,
	SPI_CRCEN 		= (1U<<13U),
	SPI_BIDIOE 		= (1U<<14U),
	SPI_BIDIMODE	= 15U,
}SPI_CR1_t;



typedef enum{
	SPI_motorola_mode	= 0U,
	SPI_TI_mode			= 1U,
}SPI_CR2_Frame_Format_t;

typedef enum{
	SPI_RXDMAEN = (1U<<0U),
	SPI_TXDMAEN = (1U<<1U),
	SPI_SSOE	= (1U<<2U),
	SPI_FRF		= (1U<<4U),
	SPI_ERRIE 	= (1U<<5U),
	SPI_RXNEIE 	= (1U<<6U),
	SPI_TXEIE 	= (1U<<7U),

}SPI_CR2_t;


typedef enum{
	SPI_channel_left	= 0U,
	SPI_channel_right	= 1U,
}SPI_Channel_Side_t;


typedef enum{
	SPI_RXNE	= (1U<<0U),
	SPI_TXE		= (1U<<1U),
	SPI_CHSIDE	= (1U<<2U),
	SPI_UDR		= (1U<<3U),
	SPI_CRCERR 	= (1U<<4U),
	SPI_MODF 	= (1U<<5U),
	SPI_OVR 	= (1U<<6U),
	SPI_BSY 	= (1U<<7U),
	SPI_FRE 	= (1U<<8U),

}SPI_SR_t;


typedef enum{
	SPI_CRCPOLY	= 0U
}SPI_CRCPR_t;

typedef enum{
	SPI_RXCRC	= 0U
}SPI_RXCRCR_t;

typedef enum{
	SPI_TXCRC	= 0U
}SPI_TXCRCR_t;

typedef enum{
	SPI_I2S_16_bit_wide	= 0U,
	SPI_I2S_32_bit_wide	= 1U,

}SPI_Channel_length_t;

typedef enum{
	SPI_I2S_16_bit_data_lenght	= 0U,
	SPI_I2S_24_bit_data_lenght	= 1U,
	SPI_I2S_32_bit_data_lenght	= 2U,

}SPI_Data_Length_Trans_t;

typedef enum{
	SPI_I2S_clock_steady_low	= 0U,
	SPI_I2S_clock_steady_high	= 1U,

}SPI_Steady_State_Polarity_t;

typedef enum{
	SPI_I2S_Philips_standard	= 0U,
	SPI_I2S_MSB_standard		= 1U,
	SPI_I2S_LSB_standard		= 2U,
	SPI_I2S_PCM_standard		= 3U
}SPI_I2S_Standar_Select_t;

typedef enum{
	SPI_I2S_short_frame_sync	= 0U,
	SPI_I2S_long_frame_sync		= 1U,

}SPI_PCM_Frame_Sync_t;

typedef enum{
	SPI_I2S_slave_transmit	= 0U,
	SPI_I2S_slave_receive	= 1U,
	SPI_I2S_master_transmit	= 2U,
	SPI_I2S_master_receive	= 3U,

}SPI_I2S_Config_Mode_t;

typedef enum{
	SPI_mode	= 0U,
	I2S_mode	= 1U,

}SPI_I2S_Mode_Selection_t;


typedef enum{
	SPI_I2S_CHLEN	= 0U,
	SPI_I2S_DATLEN	= 1U,
	SPI_I2S_CKPOL 	= 3U,
	SPI_I2S_I2SSTD 	= 4U,
	SPI_I2S_PCMSYNC = 7U,
	SPI_I2S_I2SCFG	= 8U,
	SPI_I2S_I2SE	= (1U<<10U),
	SPI_I2S_MOD		= 11U,

}SPI_I2SCFGR_t;


typedef enum{
	SPI_I2S_real_divide			= 0U,
	SPI_I2S_real_divide_plus_1	= 1U,

}SPI_I2S_odd_factor_preescaler_t;

typedef enum{
	SPI_I2S_I2SDIV	= 0U, //not set 0 or 1
	SPI_I2S_ODD		= 1U,
	SPI_I2S_MCKOE 	= (1U<<9U),
}SPI_I2SPR_t;

//typedef struct{
//	SPI_CPHA_t baud_rate;
//	SPI_CPOL_t mode;
//	SPI_Data_Frame_Format_t *synchronous_config;  //if is Asynchronous put NULL
//	SPI_Bidirectional_Mode_t data_direction;
//
//}SPI_config_t;

#endif /* DRIVERS_SPI_I2S_H_ */
