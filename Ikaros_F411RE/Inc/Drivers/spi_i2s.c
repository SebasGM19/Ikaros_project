/*
 * spi.c
 *
 *  Created on: Aug 31, 2025
 *      Author: sebas
 */


#include "spi_i2s.h"

static SPI_config_t SPI1_config = {SPI_CPHA_first_edge, SPI_CPOL_low_idle, SPI_preescaler_2, SPI_MSB_trans_first, SPI_8_bit_format, SPI_2_lines_unidirectional, SPI_motorola_mode};
static SPI_config_t SPI2_config = {SPI_CPHA_first_edge, SPI_CPOL_low_idle, SPI_preescaler_2, SPI_MSB_trans_first, SPI_8_bit_format, SPI_2_lines_unidirectional, SPI_motorola_mode};
static SPI_config_t SPI5_config = {SPI_CPHA_first_edge, SPI_CPOL_low_idle, SPI_preescaler_2, SPI_MSB_trans_first, SPI_8_bit_format, SPI_2_lines_unidirectional, SPI_motorola_mode};



static void SPI_Clock(SPI_I2S_alternative_t SPI_alt, Enabled_Disabled_t state);
static void SPI_Manual_Slave_Managment(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state);
static void SPI_Configure_As(SPI_I2SMapAddr_t SPI_addr, SPI_MSTR_Master_Select_t Select);
static void SPI_Set_CPOL(SPI_I2SMapAddr_t SPI_addr, SPI_CPOL_t CPOL);
static void SPI_Set_CPHA(SPI_I2SMapAddr_t SPI_addr, SPI_CPHA_t CPHA);
static void SPI_Set_BaudRate(SPI_I2SMapAddr_t SPI_addr, SPI_Preescaler_to_BaudRate_t divide_by);
static void SPI_Enable(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state);
static void SPI_Transmition_Format(SPI_I2SMapAddr_t SPI_addr, SPI_CR1_Frame_Format_t Format);
static void SPI_Data_Frame_Format(SPI_I2SMapAddr_t SPI_addr, SPI_DFF_t Format);
static void SPI_Receive_config(SPI_I2SMapAddr_t SPI_addr, SPI_RXONLY_t Rx);
static void SPI_1Line_Direction_Of_Transfer(SPI_I2SMapAddr_t SPI_addr, SPI_BIDIOE_Mode_t RxTx);
static void SPI_Lines_Mode(SPI_I2SMapAddr_t SPI_addr, SPI_Bidirectional_Mode_t bidirectional);
static void SPI_DMA_Mode(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state);
static void SPI_SS_Output(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state);
static void SPI_Protocol_Frame_Format(SPI_I2SMapAddr_t SPI_addr, SPI_CR2_FRF_t protocol_format);
static void SPI_Error_Interrupt(SPI_I2SMapAddr_t SPI_addr, SPI_ERR_INT_t state);
static bool SPI_Status_Flag(SPI_I2SMapAddr_t SPI_addr, SPI_SR_t flag);
static void SPI_Clear_OVR_Flag(SPI_I2SMapAddr_t SPI_addr);
static void SPI_Clear_FRE_Flag(SPI_I2SMapAddr_t SPI_addr);

//I2S
static void I2S_Clear_UDR_Flag(SPI_I2SMapAddr_t SPI_addr);

//IMPORTANT: for CS you must define your own GPIO, initialized and control it by your own

Status_code_t SPI1_Init(SPI_config_t *SPI_config){

	Status_code_t status = Success;

	status = SetPinMode(Port_A, Pin_6, Alt_func_mode); //MISO
	status |=SetPinMode(Port_A, Pin_7, Alt_func_mode); //MOSI
	status |=SetPinMode(Port_A, Pin_5, Alt_func_mode); //SCK

	GpioSetAlternativeFunction(Port_A, Pin_6, SPI1_I2S1_SPI2_I2S2_SPI3_12S3);	//MISO
	GpioSetAlternativeFunction(Port_A, Pin_7, SPI1_I2S1_SPI2_I2S2_SPI3_12S3);	//MOSI
	GpioSetAlternativeFunction(Port_A, Pin_5, SPI1_I2S1_SPI2_I2S2_SPI3_12S3);	//SCK

	SPI_Clock(SPI1_I2S1_Alt, Enabled);

	SPI_Enable(SPI1_I2S1_ADDRESS, Disabled);

	SPI_Manual_Slave_Managment(SPI1_I2S1_ADDRESS, Enabled);
	SPI_Configure_As(SPI1_I2S1_ADDRESS, SPI_set_as_master);
	SPI_Set_CPOL(SPI1_I2S1_ADDRESS, SPI_config->CPOL);
	SPI_Set_CPHA(SPI1_I2S1_ADDRESS, SPI_config->CPHA);
	SPI_Set_BaudRate(SPI1_I2S1_ADDRESS, SPI_config->preescaler);
	SPI_Transmition_Format(SPI1_I2S1_ADDRESS, SPI_config->transmitionFormat);
	SPI_Data_Frame_Format(SPI1_I2S1_ADDRESS, SPI_config->dataFrameFormat);
	SPI_Lines_Mode(SPI1_I2S1_ADDRESS, SPI_config->linesMode); //terminamos hasta el minuto 6:37 definicion del CR1

	SPI_SS_Output(SPI1_I2S1_ADDRESS, Disabled);
	SPI_Protocol_Frame_Format(SPI1_I2S1_ADDRESS, SPI_config->SPIFrameFormat);
	SPI_Error_Interrupt(SPI1_I2S1_ADDRESS, SPI_error_int_enabled);

	SPI_Enable(SPI1_I2S1_ADDRESS, Enabled); //enabled the protocol until the end

	SPI1_config = *SPI_config;
	return status;
}

Status_code_t SPI2_Init(SPI_config_t *SPI_config){

	SetPinMode(Port_B, Pin_14, Alt_func_mode); //MISO
	SetPinMode(Port_B, Pin_15, Alt_func_mode); //MOSI
	SetPinMode(Port_B, Pin_13, Alt_func_mode); //SCK

	GpioSetAlternativeFunction(Port_B, Pin_14, SPI1_I2S1_SPI2_I2S2_SPI3_12S3);	//MISO
	GpioSetAlternativeFunction(Port_B, Pin_15, SPI1_I2S1_SPI2_I2S2_SPI3_12S3);	//MOSI
	GpioSetAlternativeFunction(Port_B, Pin_13, SPI1_I2S1_SPI2_I2S2_SPI3_12S3);	//SCK

	SPI_Clock(SPI2_I2S2_Alt, Enabled);

	SPI_Enable(SPI2_I2S2_ADDRESS, Disabled);

	SPI_Manual_Slave_Managment(SPI2_I2S2_ADDRESS, Enabled);
	SPI_Configure_As(SPI2_I2S2_ADDRESS, SPI_set_as_master);
	SPI_Set_CPOL(SPI2_I2S2_ADDRESS, SPI_config->CPOL);
	SPI_Set_CPHA(SPI2_I2S2_ADDRESS, SPI_config->CPHA);
	SPI_Set_BaudRate(SPI2_I2S2_ADDRESS, SPI_config->preescaler);
	SPI_Transmition_Format(SPI2_I2S2_ADDRESS, SPI_config->transmitionFormat);
	SPI_Data_Frame_Format(SPI2_I2S2_ADDRESS, SPI_config->dataFrameFormat);
	SPI_Lines_Mode(SPI2_I2S2_ADDRESS, SPI_config->linesMode); //terminamos hasta el minuto 6:37 definicion del CR1

	SPI_SS_Output(SPI2_I2S2_ADDRESS, Disabled);
	SPI_Protocol_Frame_Format(SPI2_I2S2_ADDRESS, SPI_config->SPIFrameFormat);
	SPI_Error_Interrupt(SPI2_I2S2_ADDRESS, SPI_error_int_enabled);

	SPI_Enable(SPI2_I2S2_ADDRESS, Enabled); //enabled the protocol until the end

	SPI2_config = *SPI_config;

	return Success;
}

Status_code_t SPI5_Init(SPI_config_t *SPI_config){

	SetPinMode(Port_A, Pin_12, Alt_func_mode); //MISO
	SetPinMode(Port_A, Pin_10, Alt_func_mode); //MOSI
	SetPinMode(Port_B, Pin_0, Alt_func_mode); //SCK

	GpioSetAlternativeFunction(Port_A, Pin_12, SPI2_I2S2_SPI3_I2S3_SPI4_I2S4_SPI5_I2S5);	//MISO
	GpioSetAlternativeFunction(Port_A, Pin_10, SPI2_I2S2_SPI3_I2S3_SPI4_I2S4_SPI5_I2S5);	//MOSI
	GpioSetAlternativeFunction(Port_B, Pin_0, SPI2_I2S2_SPI3_I2S3_SPI4_I2S4_SPI5_I2S5);	//SCK

	SPI_Clock(SPI5_I2S5_Alt, Enabled);

	SPI_Enable(SPI5_I2S5_ADDRESS, Disabled);

	SPI_Manual_Slave_Managment(SPI5_I2S5_ADDRESS, Enabled);
	SPI_Configure_As(SPI5_I2S5_ADDRESS, SPI_set_as_master);
	SPI_Set_CPOL(SPI5_I2S5_ADDRESS, SPI_config->CPOL);
	SPI_Set_CPHA(SPI5_I2S5_ADDRESS, SPI_config->CPHA);
	SPI_Set_BaudRate(SPI5_I2S5_ADDRESS, SPI_config->preescaler);
	SPI_Transmition_Format(SPI5_I2S5_ADDRESS, SPI_config->transmitionFormat);
	SPI_Data_Frame_Format(SPI5_I2S5_ADDRESS, SPI_config->dataFrameFormat);
	SPI_Lines_Mode(SPI5_I2S5_ADDRESS, SPI_config->linesMode); //terminamos hasta el minuto 6:37 definicion del CR1

	SPI_SS_Output(SPI5_I2S5_ADDRESS, Disabled);
	SPI_Protocol_Frame_Format(SPI5_I2S5_ADDRESS, SPI_config->SPIFrameFormat);
	SPI_Error_Interrupt(SPI5_I2S5_ADDRESS, SPI_error_int_enabled);

	SPI_Enable(SPI5_I2S5_ADDRESS, Enabled); //enabled the protocol until the end

	SPI5_config = *SPI_config;
	return Success;
}



Status_code_t SPI_Write(SPI_I2S_alternative_t SPI_alt, const void *buff, uint32_t buff_lenght){
	SPI_I2SMapAddr_t SPI_Clock_Addr = SPI2_I2S2_ADDRESS;


	const uint8_t *buff_8_bits = NULL;
	const uint16_t *buff_16_bits = NULL;

	SPI_config_t Used_Frame_format  = {0};

	Status_code_t status = Success;

	switch(SPI_alt){
		case SPI1_I2S1_Alt:
			SPI_Clock_Addr = SPI1_I2S1_ADDRESS;
			Used_Frame_format = SPI1_config;
			break;
		case SPI2_I2S2_Alt:
			SPI_Clock_Addr = SPI2_I2S2_ADDRESS;
			Used_Frame_format = SPI2_config;
			break;
		case SPI5_I2S5_Alt:
			SPI_Clock_Addr = SPI5_I2S5_ADDRESS;
			Used_Frame_format = SPI5_config;
			break;
		default:
			return Unknown;
	}
	__IO uint8_t* SPI_DR_8_bit_reg = (__IO uint8_t *)(SPI_Clock_Addr + SPI_DR);
	__IO uint16_t* SPI_DR_16_bit_reg = (__IO uint16_t *)(SPI_Clock_Addr + SPI_DR);

	if(Used_Frame_format.dataFrameFormat == SPI_8_bit_format){
		buff_8_bits = (const uint8_t *)buff;
	}else{
		buff_16_bits = (const uint16_t *)buff;
	}

	SPI_Clear_OVR_Flag(SPI_Clock_Addr);
	SPI_Clear_FRE_Flag(SPI_Clock_Addr);

	status = TIM11_Init(SPI_MAX_TIMEOUT);
	if(status != Success){
		return status;
	}
	TIM11_Start();


	for(uint32_t i =0; i<buff_lenght; i++){

		while(!SPI_Status_Flag(SPI_Clock_Addr, SPI_TXE) && !TIM11_GET_interrupt_flag_status()){

			if(SPI_Status_Flag(SPI_Clock_Addr, SPI_OVR)){
				TIM11_Deinit();
				TIM11_clear_interrupt_flag();
				SPI_Clear_OVR_Flag(SPI_Clock_Addr);
				return SPI_overrun_error;
			}

			if(SPI_Status_Flag(SPI_Clock_Addr, SPI_FRE) && (Used_Frame_format.SPIFrameFormat == SPI_TI_mode )){
				TIM11_Deinit();
				TIM11_clear_interrupt_flag();
				SPI_Clear_FRE_Flag(SPI_Clock_Addr);
				return SPI_frame_format_error;
			}

		}

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		if(Used_Frame_format.dataFrameFormat == SPI_8_bit_format){
			*SPI_DR_8_bit_reg = (*(buff_8_bits + i));
		}else{
			*SPI_DR_16_bit_reg = (*(buff_16_bits + i));

		}
	}


	while(!SPI_Status_Flag(SPI_Clock_Addr, SPI_TXE) && !TIM11_GET_interrupt_flag_status()){

		if(SPI_Status_Flag(SPI_Clock_Addr, SPI_OVR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			SPI_Clear_OVR_Flag(SPI_Clock_Addr);
			return SPI_overrun_error;
		}

		if(SPI_Status_Flag(SPI_Clock_Addr, SPI_FRE) && (Used_Frame_format.SPIFrameFormat == SPI_TI_mode )){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			SPI_Clear_FRE_Flag(SPI_Clock_Addr);
			return SPI_frame_format_error;
		}

	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		return Timeout;
	}

	while(SPI_Status_Flag(SPI_Clock_Addr, SPI_BSY) && !TIM11_GET_interrupt_flag_status()){

		if(SPI_Status_Flag(SPI_Clock_Addr, SPI_OVR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			SPI_Clear_OVR_Flag(SPI_Clock_Addr);
			return SPI_overrun_error;
		}

		if(SPI_Status_Flag(SPI_Clock_Addr, SPI_FRE) && (Used_Frame_format.SPIFrameFormat == SPI_TI_mode )){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			SPI_Clear_FRE_Flag(SPI_Clock_Addr);
			return SPI_frame_format_error;
		}


	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		return Timeout;
	}

	SPI_Clear_OVR_Flag(SPI_Clock_Addr);


	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	return Success;

}


Status_code_t SPI_Read(SPI_I2S_alternative_t SPI_alt, void *buff, uint32_t buff_size_expected){
	SPI_I2SMapAddr_t SPI_Clock_Addr = SPI2_I2S2_ADDRESS;

	uint8_t *buff_8_bits = NULL;
	uint16_t *buff_16_bits = NULL;

	SPI_config_t Used_Frame_format  = {0};

	Status_code_t status = Success;

	switch(SPI_alt){
		case SPI1_I2S1_Alt:
			SPI_Clock_Addr = SPI1_I2S1_ADDRESS;
			Used_Frame_format = SPI1_config;
			break;
		case SPI2_I2S2_Alt:
			SPI_Clock_Addr = SPI2_I2S2_ADDRESS;
			Used_Frame_format = SPI2_config;
			break;
		case SPI5_I2S5_Alt:
			SPI_Clock_Addr = SPI5_I2S5_ADDRESS;
			Used_Frame_format = SPI5_config;
			break;
		default:
			return Unknown;
	}


	__IO uint8_t* SPI_DR_8_bit_reg = (__IO uint8_t *)(SPI_Clock_Addr + SPI_DR);
	__IO uint16_t* SPI_DR_16_bit_reg = (__IO uint16_t *)(SPI_Clock_Addr + SPI_DR);

	if(Used_Frame_format.dataFrameFormat == SPI_8_bit_format){
		buff_8_bits = (uint8_t *)buff;
	}else{
		buff_16_bits = (uint16_t *)buff;
	}

	status = TIM11_Init(SPI_MAX_TIMEOUT);
	if(status != Success){
		return status;
	}
	TIM11_Start();


	for(uint32_t i = 0; i<buff_size_expected; i++){

		if(Used_Frame_format.linesMode == SPI_2_lines_unidirectional){

			if(Used_Frame_format.dataFrameFormat == SPI_8_bit_format){
				*SPI_DR_8_bit_reg = 0x00U; //send dummy data to activate clock
			}else{
				*SPI_DR_16_bit_reg = 0x0000U; //send dummy data to activate clock
			}

		}

		while(!SPI_Status_Flag(SPI_Clock_Addr, SPI_RXNE) && !TIM11_GET_interrupt_flag_status()){

			if(SPI_Status_Flag(SPI_Clock_Addr, SPI_OVR)){
				TIM11_Deinit();
				TIM11_clear_interrupt_flag();
				SPI_Clear_OVR_Flag(SPI_Clock_Addr);
				return SPI_overrun_error;
			}

			if(SPI_Status_Flag(SPI_Clock_Addr, SPI_FRE) && (Used_Frame_format.SPIFrameFormat == SPI_TI_mode )){
				TIM11_Deinit();
				TIM11_clear_interrupt_flag();
				SPI_Clear_FRE_Flag(SPI_Clock_Addr);
				return SPI_frame_format_error;
			}

		}

		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}

		if(Used_Frame_format.dataFrameFormat == SPI_8_bit_format){
			*(buff_8_bits + i) = *SPI_DR_8_bit_reg;
		}else{
			*(buff_16_bits + i) = *SPI_DR_16_bit_reg;
		}

	}


	while(SPI_Status_Flag(SPI_Clock_Addr, SPI_BSY) && !TIM11_GET_interrupt_flag_status()){

		if(SPI_Status_Flag(SPI_Clock_Addr, SPI_OVR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			SPI_Clear_OVR_Flag(SPI_Clock_Addr);
			return SPI_overrun_error;
		}

		if(SPI_Status_Flag(SPI_Clock_Addr, SPI_FRE) && (Used_Frame_format.SPIFrameFormat == SPI_TI_mode )){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			SPI_Clear_FRE_Flag(SPI_Clock_Addr);
			return SPI_frame_format_error;
		}


	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		return Timeout;
	}

	if(SPI_Status_Flag(SPI_Clock_Addr, SPI_FRE) && (Used_Frame_format.SPIFrameFormat == SPI_TI_mode )){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		SPI_Clear_FRE_Flag(SPI_Clock_Addr);
		return SPI_frame_format_error;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	return Success;

}


Status_code_t SPI1_Deinit(void){

	Status_code_t status = Success;

	SPI_Enable(SPI1_I2S1_ADDRESS, Disabled);
	SPI_Clock(SPI1_I2S1_Alt, Disabled);

	status = SetPinMode(Port_A, Pin_6, Output); //MISO
	status |=SetPinMode(Port_A, Pin_7, Output); //MOSI
	status |=SetPinMode(Port_A, Pin_5, Output); //SCK

	return status;
}

Status_code_t SPI2_Deinit(void){

	Status_code_t status = Success;
	SPI_Enable(SPI2_I2S2_ADDRESS, Disabled);
	SPI_Clock(SPI2_I2S2_Alt, Disabled);

	status =SetPinMode(Port_B, Pin_14, Output); //MISO
	status |=SetPinMode(Port_B, Pin_15, Output); //MOSI
	status |=SetPinMode(Port_B, Pin_13, Output); //SCK

	return status;
}

Status_code_t SPI5_Deinit(void){

	Status_code_t status = Success;
	SPI_Enable(SPI5_I2S5_ADDRESS, Disabled);
	SPI_Clock(SPI5_I2S5_Alt, Disabled);

	status =SetPinMode(Port_A, Pin_12, Output); //MISO
	status |=SetPinMode(Port_A, Pin_10, Output); //MOSI
	status |=SetPinMode(Port_B, Pin_0, Output); //SCK

	return status;
}


////////////////////////// Configurations////////////////////////////////////////////////////////////////
static void SPI_Clock(SPI_I2S_alternative_t SPI_alt, Enabled_Disabled_t state){

	RCC_offset_t RCC_option = RCC_OFFSET_APB2ENR;

	if(SPI_alt == SPI2_I2S2_Alt){
		RCC_option = RCC_OFFSET_APB1ENR;
	}

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_option);

    *pClockControlReg = (state) ? (*pClockControlReg | (Enabled << SPI_alt)) : (*pClockControlReg & ~(Enabled << SPI_alt));

}

static void SPI_Manual_Slave_Managment(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state){

	__IO uint32_t *slave_managment = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *slave_managment = (state) ? (*slave_managment | (SPI_SSM)) : (*slave_managment & ~(SPI_SSM)); //

    if(state == Enabled){
    	*slave_managment = (state) ? (*slave_managment | (SPI_SSI)) : (*slave_managment & ~(SPI_SSI));
    }
}

static void SPI_Configure_As(SPI_I2SMapAddr_t SPI_addr, SPI_MSTR_Master_Select_t Select){

	__IO uint32_t *master_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *master_select = (Select) ? (*master_select | (SPI_MSTR)) : (*master_select & ~(SPI_MSTR)); //

}

static void SPI_Set_CPOL(SPI_I2SMapAddr_t SPI_addr, SPI_CPOL_t CPOL){

	__IO uint32_t *CPOL_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *CPOL_select = (CPOL) ? (*CPOL_select | (SPI_CPOL)) : (*CPOL_select & ~(SPI_CPOL)); //

}

static void SPI_Set_CPHA(SPI_I2SMapAddr_t SPI_addr, SPI_CPHA_t CPHA){

	__IO uint32_t *CPHA_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *CPHA_select = (CPHA) ? (*CPHA_select | (SPI_CPHA)) : (*CPHA_select & ~(SPI_CPHA)); //

}


static void SPI_Set_BaudRate(SPI_I2SMapAddr_t SPI_addr, SPI_Preescaler_to_BaudRate_t divide_by){

	__IO uint32_t *baudRate = (__IO uint32_t *)(SPI_addr + SPI_CR1);
//	uint32_t Frequency_clock = APB2_CLOCK;
//	uint32_t SPI_Frequency = 0;
//	if(SPI_addr == SPI2_I2S2_ADDRESS){
//		Frequency_clock = APB1_CLOCK;
//	}
//	SPI_Frequency = (uint32_t)( Frequency_clock/(divide_by)); //to show the frequency to use

	*baudRate &= ~(Clear_three_bits<<SPI_BR);
	*baudRate |= (divide_by<<SPI_BR);


}

static void SPI_Enable(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state){

	__IO uint32_t *SPE_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *SPE_select = (state) ? (*SPE_select | (SPI_SPE)) : (*SPE_select & ~(SPI_SPE)); //

}

static void SPI_Transmition_Format(SPI_I2SMapAddr_t SPI_addr, SPI_CR1_Frame_Format_t Format){

	__IO uint32_t *LSB_MSB_firts_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *LSB_MSB_firts_select = (Format) ? (*LSB_MSB_firts_select | (SPI_LSBFIRST)) : (*LSB_MSB_firts_select & ~(SPI_LSBFIRST)); //

}

static void SPI_Data_Frame_Format(SPI_I2SMapAddr_t SPI_addr, SPI_DFF_t Format){

	__IO uint32_t *SPE_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *SPE_select = (Format) ? (*SPE_select | (SPI_DFF)) : (*SPE_select & ~(SPI_DFF)); //

}

static void SPI_Receive_config(SPI_I2SMapAddr_t SPI_addr, SPI_RXONLY_t Rx){

	__IO uint32_t *SPE_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *SPE_select = (Rx) ? (*SPE_select | (SPI_RXONLY)) : (*SPE_select & ~(SPI_RXONLY)); //

}


static void SPI_1Line_Direction_Of_Transfer(SPI_I2SMapAddr_t SPI_addr, SPI_BIDIOE_Mode_t RxTx){

	__IO uint32_t *BIDIOE_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *BIDIOE_select = (RxTx) ? (*BIDIOE_select | (SPI_BIDIOE)) : (*BIDIOE_select & ~(SPI_BIDIOE)); //

}


static void SPI_Lines_Mode(SPI_I2SMapAddr_t SPI_addr, SPI_Bidirectional_Mode_t bidirectional){

	__IO uint32_t *BIDIOE_select = (__IO uint32_t *)(SPI_addr + SPI_CR1);

    *BIDIOE_select = (bidirectional) ? (*BIDIOE_select | (SPI_BIDIMODE)) : (*BIDIOE_select & ~(SPI_BIDIMODE));
	SPI_Receive_config(SPI_addr, SPI_full_duplex);

	if(bidirectional == SPI_2_lines_unidirectional){
		SPI_1Line_Direction_Of_Transfer(SPI_addr, SPI_BIDIOE_set_as_RX); //set as 0 when is 2 lines, not used
	}else{
		SPI_1Line_Direction_Of_Transfer(SPI_addr, SPI_BIDIOE_set_as_TX); //if is 1 line, this must be use to Tx or Rx data
	}


}


static void SPI_DMA_Mode(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state){

	__IO uint32_t *SPI_DMA_TX_RX_mode = (__IO uint32_t *)(SPI_addr + SPI_CR2);

    *SPI_DMA_TX_RX_mode = (state) ? (*SPI_DMA_TX_RX_mode | (SPI_RXDMAEN)) : (*SPI_DMA_TX_RX_mode & ~(SPI_RXDMAEN));
    *SPI_DMA_TX_RX_mode = (state) ? (*SPI_DMA_TX_RX_mode | (SPI_TXDMAEN)) : (*SPI_DMA_TX_RX_mode & ~(SPI_TXDMAEN));

}

static void SPI_SS_Output(SPI_I2SMapAddr_t SPI_addr, Enabled_Disabled_t state){

	__IO uint32_t *SPI_SS_output_mode = (__IO uint32_t *)(SPI_addr + SPI_CR2);

    *SPI_SS_output_mode = (state) ? (*SPI_SS_output_mode | (SPI_SSOE)) : (*SPI_SS_output_mode & ~(SPI_SSOE));

}


static void SPI_Protocol_Frame_Format(SPI_I2SMapAddr_t SPI_addr, SPI_CR2_FRF_t protocol_format){

	__IO uint32_t *Frame_Format_mode = (__IO uint32_t *)(SPI_addr + SPI_CR2);

    *Frame_Format_mode = (protocol_format) ? (*Frame_Format_mode | (SPI_FRF)) : (*Frame_Format_mode & ~(SPI_FRF));

}

static void SPI_Error_Interrupt(SPI_I2SMapAddr_t SPI_addr, SPI_ERR_INT_t state){

	__IO uint32_t *ERRINT = (__IO uint32_t *)(SPI_addr + SPI_CR2);

    *ERRINT = (state) ? (*ERRINT | (SPI_ERRIE)) : (*ERRINT & ~(SPI_ERRIE));

}



static bool SPI_Status_Flag(SPI_I2SMapAddr_t SPI_addr, SPI_SR_t flag){

	__I uint32_t *SPI_SR_Reg = (__I uint32_t *)(SPI_addr + SPI_SR);
	return ((*SPI_SR_Reg & flag) == flag); //will return the state


}


static void SPI_Clear_OVR_Flag(SPI_I2SMapAddr_t SPI_addr){

	__I uint32_t *SPI_SR_Reg = (__I uint32_t *)(SPI_addr + SPI_SR);
	__I uint32_t *SPI_DR_Reg = (__I uint32_t *)(SPI_addr + SPI_DR);

	(void)*SPI_DR_Reg;
	(void)*SPI_SR_Reg;
}

static void SPI_Clear_FRE_Flag(SPI_I2SMapAddr_t SPI_addr){

	__I uint32_t *SPI_SR_Reg = (__I uint32_t *)(SPI_addr + SPI_SR);

	(void)*SPI_SR_Reg;

}

static void I2S_Clear_UDR_Flag(SPI_I2SMapAddr_t SPI_addr){

	__I uint32_t *I2S_SR_Reg = (__I uint32_t *)(SPI_addr + SPI_SR);
	__I uint32_t *I2S_DR_Reg = (__I uint32_t *)(SPI_addr + SPI_DR);

	(void)*I2S_SR_Reg;
	(void)*I2S_DR_Reg;
}
