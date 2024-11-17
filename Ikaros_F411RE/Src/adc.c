/*
 * adc.c
 *
 *  Created on: Jul 22, 2024
 *      Author: sebas
 */
#include "adc.h"
#include "timers.h"
#include "gpios.h"



Pin_number_t const ADC_PIN_MAP[16]={Pin_0,Pin_1,Pin_2,Pin_3,Pin_4,Pin_5,Pin_6,Pin_7,Pin_0,Pin_1,Pin_0,Pin_1,Pin_2,Pin_3,Pin_4,Pin_5};
Set_Port_t const ADC_PORT_MAP[16]={Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_B,Port_B,Port_C,Port_C,Port_C,Port_C,Port_C,Port_C};

uint8_t static ADC_channel_ocupped[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

Status_code_t ADC_Init(ADC_resolution_t resolution){

	ADC_Clock(Enabled);
	ADC_conversion_state(Disabled);
	ADC_Set_Resolution(resolution);

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

uint32_t ADC_Read(ADC_channel_t Channel){


	if(!ADC_channel_ocupped[Channel]){
		return 0;
	}
	__I uint32_t * const ADC_REG_DR = (__I uint32_t * const)(ADC1_ADDRESS + ADC_DR);
	uint32_t ADC_value=0;

	ADC_conversion_state(Disabled);
	ADC_Channel_SQR_Single_position(Channel);
	ADC_SetSingleChannelLenght(Single_channel);
	ADC_conversion_state(Enabled);
	ADC_start_convertion_regular_channel();

	ADC_value= (*ADC_REG_DR);


	return ADC_value;

}

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




void ADC_conversion_state(Enabled_Disabled_t state){
	__IO uint32_t *ADC_REG_CR2 = (__IO uint32_t *)(ADC1_ADDRESS + ADC_CR2);

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


void ADC_Channel_SQR_Single_position(ADC_channel_t Channel){
	__IO uint32_t *ADC_REG_SQR = (__IO uint32_t *)(ADC1_ADDRESS + ADC_SQR3);

	*ADC_REG_SQR &=  ~(0x1F<<ADC_SQR_SINGLE_POSITION);
	*ADC_REG_SQR |=  (Channel<<ADC_SQR_SINGLE_POSITION);

}


void ADC_SetSingleChannelLenght(ADC_single_channel_lenght_t lenght){


	__IO uint32_t *ADC_REG_LEN = (__IO uint32_t *)(ADC1_ADDRESS + ADC_SQR1);
	*ADC_REG_LEN &=  ~(Clear_four_bits<<ADC_L);
	*ADC_REG_LEN |=  (lenght<<ADC_L);
}


void ADC_Clock(Enabled_Disabled_t state){

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_APB2ENR);
    if(state){
        *pClockControlReg |= ADC1_ENABLE;
    }else{
        *pClockControlReg &= ~ADC1_ENABLE;
    }
}






Status_code_t ADC_start_convertion_regular_channel(void){
	__I uint32_t * const ADC_REG_CR2 = (__I uint32_t * const)(ADC1_ADDRESS + ADC_CR2);

	if( !((*ADC_REG_CR2) & ADC_ADON) ){
		return ADC_ADConverterd_off;
	}

	ADC_start_conversion(Start);
	ADC_wait_conversion_flag();

	return Success;
}


void ADC_cleanConversionFlag(void){
	__IO uint32_t *ADC_REG_SR = (__IO uint32_t *)(ADC1_ADDRESS + ADC_SR);

	*ADC_REG_SR &= ~(ADC_EOC);

}

Status_code_t ADC_wait_conversion_flag(void){
	Status_code_t status = Success;
	__IO uint32_t *const ADC_REG_SR = (__IO uint32_t *const)(ADC1_ADDRESS + ADC_SR);

	status = TIM11_Init(MAX_ADC_TIMEOUT);
	if(status != Success){
		return status;
	}
	TIM11_Start();


	 while( (!(ADC_EOC & (*ADC_REG_SR))) && !TIM11_GET_interrupt_flag_status() ){
		if( TIM11_GET_interrupt_flag_status() ){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
			return Timeout;
		}
	 }

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	ADC_cleanConversionFlag();
	return Success;
}





void ADC_start_conversion(ADC_conversion_start_t state){

	__IO uint32_t *ADC_REG_CR2 = (__IO uint32_t *)(ADC1_ADDRESS + ADC_CR2);

	if(state){
		*ADC_REG_CR2 |= (ADC_SWSTART);
	}else{
		*ADC_REG_CR2 &= ~(ADC_SWSTART);
	}

}


void ADC_Set_Resolution(ADC_resolution_t resolution){
	__IO uint32_t *ADC_REG_CR1 = (__IO uint32_t *)(ADC1_ADDRESS + ADC_CR1);

	*ADC_REG_CR1 &= ~(Clear_two_bits<<ADC_RES);

	*ADC_REG_CR1 |= (resolution<<ADC_RES);


}



