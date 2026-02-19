/*
 * SRAM23LCV512.c
 *
 *  Created on: Dec 7, 2025
 *      Author: sebas
 */


#include "SRAM23LCV512.h"

#define BYTES_OF_PAGE				(32U)
#define WRITE_MODE					(2U)
#define OPER_PLUS_ADDRS				(3U)
#define OPER_PLUS_ADDRS_PLUS_BYTE	(4U)
#define TOTAL_PAGES					(2048U)

// this function read the mode of reading the memory, where it could be byte, page or sequential mode
Status_code_t SRAM23LCV_Get_Mode(SPI_I2S_alternative_t alt, SRAM23LCV_operation_mode* mode){
	Status_code_t status = Success;
	uint8_t data_to_write = (uint8_t)RDMR;
	uint8_t data_buff = 0x00U;

	status = SPI_Write(alt, &data_to_write, sizeof(data_to_write));
	if(status != Success){
		return status;
	}
	status = SPI_Read(alt, &data_buff, sizeof(data_buff));

	(*mode)  = (SRAM23LCV_operation_mode)(data_buff >> 6U);

	return status;

}

// this function set the mode of reading the memory, where it could be byte, page or sequential mode
Status_code_t SRAM23LCV_Set_Mode(SPI_I2S_alternative_t alt, const SRAM23LCV_operation_mode mode){
	Status_code_t status = Success;
	uint8_t data_to_write[WRITE_MODE] = {(uint8_t)WRMR, 0x00U};

	data_to_write[1] |= (mode<<6);

	status = SPI_Write(alt, data_to_write, sizeof(data_to_write));

	return status;

}

Status_code_t SRAM23LCV_Read_Byte(SPI_I2S_alternative_t alt, uint16_t start_addr, uint8_t* data_read){
	Status_code_t status = Success;

	uint8_t data_to_write[OPER_PLUS_ADDRS] = {READ, (uint8_t)(start_addr>>8U),(uint8_t)(start_addr&0x00FF)};
	uint8_t data_buff = 0x00U;

	status = SPI_Write(alt, data_to_write, sizeof(data_to_write));
	if(status != Success){
		return status;
	}
	status = SPI_Read(alt, &data_buff, sizeof(data_buff));

	(*data_read) = data_buff;

	return status;

}


Status_code_t SRAM23LCV_Write_Byte(SPI_I2S_alternative_t alt, uint16_t start_addr, const uint8_t byte_to_write){
	Status_code_t status = Success;

	uint8_t data_to_write[OPER_PLUS_ADDRS_PLUS_BYTE] = {WRITE, (uint8_t)(start_addr>>8U),(uint8_t)(start_addr&0x00FF), byte_to_write};

	status = SPI_Write(alt, data_to_write, sizeof(data_to_write));

	return status;
}



//TO TEST
Status_code_t SRAM23LCV_Write_Sequential(SPI_I2S_alternative_t alt, uint16_t start_addr, const uint8_t* buff, uint16_t buff_lengt){
	Status_code_t status = Success;

	uint16_t total_buff = buff_lengt+OPER_PLUS_ADDRS;

	uint8_t data_to_write[total_buff];

	data_to_write[0] = (uint8_t)WRITE;
	data_to_write[1] = (uint8_t)(start_addr>>8U);
	data_to_write[2] = (uint8_t)(start_addr&0x00FF);

	for(uint16_t i = 0; i < buff_lengt; i++){
		data_to_write[i+OPER_PLUS_ADDRS] = buff[i];
	}

	status = SPI_Write(alt, data_to_write, sizeof(data_to_write));

	return status;
}

Status_code_t SRAM23LCV_Read_Sequential(SPI_I2S_alternative_t alt, uint16_t start_addr, uint8_t* buff, uint16_t buff_lengt){
	Status_code_t status = Success;


	uint8_t data_to_write[OPER_PLUS_ADDRS] = {READ, (uint8_t)(start_addr>>8U),(uint8_t)(start_addr&0x00FF)};
	uint8_t buff_data_read[buff_lengt];

	status = SPI_Write(alt, data_to_write, sizeof(data_to_write));
	if(status != Success){
		return status;
	}
	status = SPI_Read(alt, buff_data_read, buff_lengt);

	for(uint16_t i = 0; i<buff_lengt; i++){
		buff[i] = buff_data_read[i];
	}

	return status;
}


/*una pagina consta de 32 bytes, y el total de paginas que se puede tener es de 2048*/
/* starts as Page 1*/
Status_code_t SRAM23LCV_Write_Page(SPI_I2S_alternative_t alt, uint16_t pageX, const uint8_t* buff, uint8_t buff_lenght){
	Status_code_t status = Success;

	uint8_t data_to_write[BYTES_OF_PAGE + OPER_PLUS_ADDRS];
	uint16_t start_addr = 0U;
	if(pageX > TOTAL_PAGES || buff_lenght > BYTES_OF_PAGE){
		return Error;
	}
	memset(data_to_write, '\0',sizeof(data_to_write));

	start_addr = pageX*BYTES_OF_PAGE;

	data_to_write[0] = (uint8_t)WRITE;
	data_to_write[1] = (uint8_t)(start_addr>>8U);
	data_to_write[2] = (uint8_t)(start_addr&0x00FF);

	for(uint16_t i = 0; i < buff_lenght; i++){
		data_to_write[i+OPER_PLUS_ADDRS] = buff[i];
	}

	status = SPI_Write(alt, data_to_write, sizeof(data_to_write));

	return status;
}


//TO TEST
Status_code_t SRAM23LCV_Read_Page(SPI_I2S_alternative_t alt, uint16_t pageX, uint8_t* buff){
	Status_code_t status = Success;
	uint16_t start_addr = 0U;

	start_addr = pageX*BYTES_OF_PAGE;

	uint8_t data_to_write[OPER_PLUS_ADDRS] = {READ, (uint8_t)(start_addr>>8U),(uint8_t)(start_addr&0x00FF)};
	uint8_t buff_data_read[BYTES_OF_PAGE];

	status = SPI_Write(alt, data_to_write, sizeof(data_to_write));
	if(status != Success){
		return status;
	}
	status = SPI_Read(alt, buff_data_read, BYTES_OF_PAGE);

	for(uint16_t i = 0; i<BYTES_OF_PAGE; i++){
		buff[i] = buff_data_read[i];
	}

	return status;
}

