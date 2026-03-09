/*
 * SRAM23LCV512.h
 *
 *  Created on: Dec 7, 2025
 *      Author: sebas
 */

#ifndef SRAM23LCV512_H_
#define SRAM23LCV512_H_

#include "spi_i2s.h"

typedef enum{
	SRAM23LCV_byte_mode 		= 0U,
	SRAM23LCV_Sequential_mode 	= 1U,
	SRAM23LCV_Page_mode 		= 2U,

}SRAM23LCV_operation_mode;


typedef enum{
	WRMR 	= 0x01U,
	WRITE 	= 0x02U,
	READ 	= 0x03U,
	RDMR 	= 0x05U,
	EDIO 	= 0x3BU,
	RSTIO 	= 0xFFU,
}SRAM23LCV_instruction_name;

Status_code_t SRAM23LCV_Get_Mode(SPI_I2S_alternative_t alt, SRAM23LCV_operation_mode* mode);
Status_code_t SRAM23LCV_Set_Mode(SPI_I2S_alternative_t alt, const SRAM23LCV_operation_mode mode);

Status_code_t SRAM23LCV_Write_Byte(SPI_I2S_alternative_t alt, uint16_t start_addr, const uint8_t byte_to_write);
Status_code_t SRAM23LCV_Read_Byte(SPI_I2S_alternative_t alt, uint16_t start_addr, uint8_t* data_read);

Status_code_t SRAM23LCV_Write_Sequential(SPI_I2S_alternative_t alt, uint16_t start_addr, const uint8_t* buff, uint16_t buff_lengt);
Status_code_t SRAM23LCV_Read_Sequential(SPI_I2S_alternative_t alt, uint16_t start_addr, uint8_t* buff, uint16_t buff_lengt);

Status_code_t SRAM23LCV_Write_Page(SPI_I2S_alternative_t alt, uint16_t pageX, const uint8_t* buff, uint8_t buff_lenght);
Status_code_t SRAM23LCV_Read_Page(SPI_I2S_alternative_t alt, uint16_t pageX, uint8_t* buff);

#endif /* PERIPHERALS_SRAM23LCV512_H_ */
