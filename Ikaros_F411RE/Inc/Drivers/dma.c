/*
 * dma.c
 *
 *  Created on: Mar 8, 2026
 *      Author: sebas
 */


#include "dma.h"

static void DMA_Clock(DMA_alt_t DMAx, Enabled_Disabled_t state);
static void DMA_Enable_Stream(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, Enabled_Disabled_t state);
static void DMA_Set_Peripheral_Addr(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, uint32_t Peripheral_addr);
static void DMA_Set_Mem_buff_Addr(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, uint32_t * Mem0_buff_addr);


Status_code_t DMA_to_ADC_Init(uint32_t Peripheral_addr, uint32_t* dest_buff){ //ADC es Stream 0, channel 0 or channel 4
	Status_code_t status = Success;

	/*Enable DMA clock*/
	DMA_Clock(DMA2_Alt, Enabled); // DMA2 to enable ADC
	/*Disable Stream 0*/
	DMA_Enable_Stream(DMA2_ADDRESS, DMA_Streams_0, Disabled);
	/*Clear all interrupt flags of stream 0*/
	DMA_Clear_Stream_Flag(DMA2_ADDRESS, DMA_Streams_0, DMA_FEIFx);
	DMA_Clear_Stream_Flag(DMA2_ADDRESS, DMA_Streams_0, DMA_DMEIFx);
	DMA_Clear_Stream_Flag(DMA2_ADDRESS, DMA_Streams_0, DMA_TEIFx);
	DMA_Clear_Stream_Flag(DMA2_ADDRESS, DMA_Streams_0, DMA_HTIFx);
	DMA_Clear_Stream_Flag(DMA2_ADDRESS, DMA_Streams_0, DMA_TCIFx);


	/*set the destination buffer*/
//	DMA_Set_Destination_Buffer(DMA2_ADDRESS, DMA_Streams_0, (ADC1_ADDRESS + ADC_DR)); //example
	DMA_Set_Peripheral_Addr(DMA2_ADDRESS, DMA_Streams_0, Peripheral_addr);

	/*set the source buffer*/
	DMA_Set_Mem_buff_Addr(DMA2_ADDRESS, DMA_Streams_0, dest_buff);
	/*set lenght*/
	/*select stream 0 channel 0*/
	/*enable memory increment*/
	//enable circular buffer
	/*configure transfer direction*/
	/*enable direct mode*/
	/*disable FIFO*/
	/*enable stream 6*/
	/*Enable ADC DMA (it is necesarry to call ADC_Init())*/
	/*DMA NVIC_interrupt to do something after the transfer is completed*/







//	__IO uint8_t *DMA_DR_Reg = (__IO uint8_t *)(DMA2_ADDRESS + I2C_DR);
//	__IO uint32_t *DMA_SR2_Reg = (__IO uint32_t *)(DMA2_ADDRESS + I2C_SR2);

	return status;
}

//testear
void DMA_Clear_Stream_Flag(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, DMA_flags_t flag){

	DMA_register_offset_t DMA_addrs_offset = DMA_LIFCR;
	uint32_t position = 0U;
	uint8_t add_offset =0;


	if(stream == DMA_Streams_2 || stream == DMA_Streams_3 || stream == DMA_Streams_6 || stream == DMA_Streams_7){
		add_offset = 4U;
	}

	if(stream >= DMA_Streams_4 ){
		DMA_addrs_offset = DMA_HIFCR;
		position = (uint32_t)((Enabled<<((DMA_FLAGS_OFFSET * (stream - DMA_Streams_4)) + add_offset)) << flag);
	}else{
		position = (uint32_t)((Enabled<<((DMA_FLAGS_OFFSET * stream) + add_offset)) << flag);
	}


	__IO uint32_t *DMA_flag_Reg = (__IO uint32_t *)(DMA_Addr + DMA_addrs_offset);
	(*DMA_flag_Reg) |= position; //add 1 to the flag clears their value


}

//testear
FlagStatus DMA_Get_Stream_Flag(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, DMA_flags_t flag){

	DMA_register_offset_t DMA_addrs_offset = DMA_LIFCR;
	uint32_t position = 0U;
	uint8_t add_offset = 0;


	if(stream == DMA_Streams_2 || stream == DMA_Streams_3 || stream == DMA_Streams_6 || stream == DMA_Streams_7){
		add_offset = 4U;
	}

	if(stream >= DMA_Streams_4 ){
		DMA_addrs_offset = DMA_HIFCR;
		position = (uint32_t)((Enabled<<((DMA_FLAGS_OFFSET * (stream - DMA_Streams_4)) + add_offset)) << flag);
	}else{
		position = (uint32_t)((Enabled<<((DMA_FLAGS_OFFSET * stream) + add_offset)) << flag);
	}


	__IO uint32_t *DMA_flag_Reg = (__IO uint32_t *)(DMA_Addr + DMA_addrs_offset);

	return (FlagStatus)((*DMA_flag_Reg) & position); //returns SET if is in (1&1), returns CLEAR if its different (0 & 1)

}




//////////////////////////////funciones locales////////////////////////////////
static void DMA_Clock(DMA_alt_t DMAx, Enabled_Disabled_t state){

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_AHB1ENR);

    *pClockControlReg = (state) ? (*pClockControlReg | (Enabled << DMAx)) : (*pClockControlReg & ~(Enabled << DMAx));

}

static void DMA_Enable_Stream(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, Enabled_Disabled_t state){

	__IO uint32_t *DMA_ENA_DIS_stream = (__IO uint32_t *)(DMA_Addr + (DMA_SxCR + (DMA_OFFSET_STREAM_SELECTOR*stream)));

    *DMA_ENA_DIS_stream = (state) ? (*DMA_ENA_DIS_stream | (DMA_EN)) : (*DMA_ENA_DIS_stream & ~(DMA_EN));

}

static void DMA_Set_Peripheral_Addr(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, uint32_t Peripheral_addr){

	__IO uint32_t *DMA_Peripheral_addr = (__IO uint32_t *)(DMA_Addr + (DMA_SxPAR + (DMA_OFFSET_STREAM_SELECTOR*stream)));

    *DMA_Peripheral_addr = Peripheral_addr;

}

//pasa la direccion de memoria del buffer donde se gaurdara la info
static void DMA_Set_Mem_buff_Addr(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, uint32_t * Mem0_buff_addr){

	__IO uint32_t *DMA_Mem0_addr = (__IO uint32_t *)(DMA_Addr + (DMA_SxM0AR + (DMA_OFFSET_STREAM_SELECTOR*stream)));

    *DMA_Mem0_addr = Mem0_buff_addr; //pasa como direccion de memoria

}

