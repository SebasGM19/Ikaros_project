/*
 * adc.c
 *
 *  Created on: Jul 22, 2024
 *      Author: sebas
 */
#include "adc.h"

Pin_number_t const ADC_PIN_MAP[16]={Pin_0,Pin_1,Pin_2,Pin_3,Pin_4,Pin_5,Pin_6,Pin_7,Pin_0,Pin_1,Pin_0,Pin_1,Pin_2,Pin_3,Pin_4,Pin_5};
Set_Port_t const ADC_PORT_MAP[16]={Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_B,Port_B,Port_C,Port_C,Port_C,Port_C,Port_C,Port_C};

uint8_t static ADC_channel_ocupped[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//uint8_t static conversion_lenght=0;

//para el ADC se debe habilitar el APB2 la resolucion es de 12 bits para 16 canales, 16 pines
//3v/4096 * input  = adc


void ADC_conversion_state(Enabled_Disabled_t state){
	uint32_t volatile *ADC_REG_CR2 = (uint32_t volatile*)(ADC1_ADDRESS + ADC_CR2);

	if(state){
		*ADC_REG_CR2 |= (ADC_ADON);
	}else{
		*ADC_REG_CR2 &= ~(ADC_ADON);
	}
}

uint8_t ADC_countChannelActivated(void){

	uint8_t channel_count =0;
	for(uint8_t j =0; j<16; j++){
		if(ADC_channel_ocupped[j]==1){
			channel_count++;
		}
	}
	return channel_count;
}

//void Channel_SQR_position(ADC_channel_t Channel){
//	ADC_register_offset_t used_SQR = ADC_SQR3;
//	uint32_t SQR_position_start =0;
//
// if(Channel<=5){
//	 used_SQR = ADC_SQR3;
//	 SQR_position_start = (uint32_t)ADC_SQ1+(Channel*5);
//
// }else if (Channel>5 && Channel<=11){
//	 used_SQR = ADC_SQR2;
//	 SQR_position_start = (uint32_t)ADC_SQ7+((Channel-6)*5);
//
// }else{
//	 used_SQR = ADC_SQR1;
//	 SQR_position_start = (uint32_t)ADC_SQ13+((Channel-12)*5);
//
// }
//
//	uint32_t volatile *ADC_REG_SQR = (uint32_t volatile*)(ADC1_ADDRESS + used_SQR); //agregamos el 0x14 par indicar output
//
//	*ADC_REG_SQR &=  ~(0x1F<<SQR_position_start);
//	*ADC_REG_SQR |=  (Channel<<SQR_position_start); //se asigna el numero del canal
//
//
//}

void ADC_Channel_SQR_Single_position(ADC_channel_t Channel){
	uint32_t volatile *ADC_REG_SQR = (uint32_t volatile*)(ADC1_ADDRESS + ADC_SQR3);

	*ADC_REG_SQR &=  ~(0x1F<<ADC_SQR_SINGLE_POSITION);
	*ADC_REG_SQR |=  (Channel<<ADC_SQR_SINGLE_POSITION);

}

//void SetChannelLenght(void){
//
//	conversion_lenght = countChannelActivated();
//
//	uint32_t volatile *ADC_REG_LEN = (uint32_t volatile*)(ADC1_ADDRESS + ADC_SQR1); //agregamos el 0x14 par indicar output
//	*ADC_REG_LEN &=  ~(Clear_four_bits<<ADC_L);
//	*ADC_REG_LEN |=  (conversion_lenght<<ADC_L);
//}


void ADC_SetSingleChannelLenght(ADC_single_channel_lenght_t lenght){


	uint32_t volatile *ADC_REG_LEN = (uint32_t volatile*)(ADC1_ADDRESS + ADC_SQR1);
	*ADC_REG_LEN &=  ~(Clear_four_bits<<ADC_L);
	*ADC_REG_LEN |=  (lenght<<ADC_L);
}


void ADC_Clock(Enabled_Disabled_t state){

	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(RCC_ADDRESS + RCC_OFFSET_APB2ENR);
    if(state){
        *pClockControlReg |= ADC1_ENABLE;
    }else{
        *pClockControlReg &= ~ADC1_ENABLE;
    }
}


Status_code_t ADC_Init(void){

	ADC_Clock(Enabled);
//	ADC_conversion_state(Disabled);
	return Success;

}


Status_code_t ADC_Configure_Channel(ADC_channel_t Channel){

	if(Channel == Channel_2 || Channel == Channel_3 || Channel>Channel_15){
		return ADC_channel_not_available;
	}
	if(	ADC_channel_ocupped[Channel]){
		return ADC_channel_already_initialized;
	}

	SetPinMode(ADC_PORT_MAP[Channel], ADC_PIN_MAP[Channel], Analog_mode);
	ADC_channel_ocupped[Channel] = 1;

	return Success;
}


Status_code_t ADC_start_convertion_regular_channel(void){
	uint32_t volatile *ADC_REG_CR2 = (uint32_t volatile*)(ADC1_ADDRESS + ADC_CR2);

	if( !((*ADC_REG_CR2) & ADC_ADON) ){
		return ADC_ADConverterd_off;
	}

	ADC_start_conversion(Start);
	ADC_wait_conversion_flag();

	return Success;
}

//Status_code_t start_convertion_continues_channel(void){
//	uint32_t volatile *ADC_REG_CR2 = (uint32_t volatile*)(ADC1_ADDRESS + ADC_CR2);
//
//
//	*ADC_REG_CR2 |= ADC_CONT;
//
////	if( !((*ADC_REG_CR2) & ADC_CONT) ){
////		return ADC_ADConverterd_off;
////	}
//
//	*ADC_REG_CR2 |= (ADC_SWSTART);
//
//	return Success;
//}

void ADC_cleanConversionFlag(void){
	uint32_t volatile *ADC_REG_SR = (uint32_t volatile*)(ADC1_ADDRESS + ADC_SR);

	*ADC_REG_SR &= ~(ADC_EOC);

}

void ADC_wait_conversion_flag(void){
	uint32_t volatile *ADC_REG_SR = (uint32_t volatile*)(ADC1_ADDRESS + ADC_SR);
	 while( !(ADC_EOC & (*ADC_REG_SR)) ){} //use a timeout exit
	 ADC_cleanConversionFlag();
}



uint32_t ADC_Read(ADC_channel_t Channel){


	if(!ADC_channel_ocupped[Channel]){
		return 0;
	}
	uint32_t volatile *ADC_REG_DR = (uint32_t volatile*)(ADC1_ADDRESS + ADC_DR);
	uint32_t ADC_value=0;

	ADC_conversion_state(Disabled);
	ADC_Channel_SQR_Single_position(Channel);
	ADC_SetSingleChannelLenght(Single_channel);
	ADC_conversion_state(Enabled);
	ADC_start_convertion_regular_channel();

	ADC_value= (*ADC_REG_DR);


	return ADC_value;

}

//uint32_t ADC_Read_continues(ADC_channel_t Channel){
//	uint32_t volatile *ADC_REG_DR = (uint32_t volatile*)(ADC1_ADDRESS + ADC_DR);
//	uint32_t ADC_value=0;
//
//	ADC_value= (*ADC_REG_DR);
//
//
//return ADC_value;
//
//}

Status_code_t ADC_Deinit(void){

	for(uint8_t i =0; i<16; i++){
		ADC_channel_ocupped[i] = 0;
	}
	ADC_SetSingleChannelLenght(None_channel);

	ADC_start_conversion(Stop);
	ADC_conversion_state(Disabled);

	ADC_Clock(Disabled);

	return Success;

}

void ADC_start_conversion(ADC_conversion_start_t state){

	uint32_t volatile *ADC_REG_CR2 = (uint32_t volatile*)(ADC1_ADDRESS + ADC_CR2);

	if(state){
		*ADC_REG_CR2 |= (ADC_SWSTART);
	}else{
		*ADC_REG_CR2 &= ~(ADC_SWSTART);
	}

}


void ADC_Set_Resolution(ADC_resolution_t resolution){

}


void ADC_set_Operation_Mode(ADC_operation_mode_t Mode){

}

