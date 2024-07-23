/*
 * adc.c
 *
 *  Created on: Jul 22, 2024
 *      Author: sebas
 */
#include "adc.h"

Pin_number_t const ADC_PIN_MAP[16]={Pin_0,Pin_1,Pin_2,Pin_3,Pin_4,Pin_5,Pin_6,Pin_7,Pin_0,Pin_1,Pin_0,Pin_1,Pin_2,Pin_3,Pin_4,Pin_5};
Set_Port_t const ADC_PORT_MAP[16]={Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_A,Port_B,Port_B,Port_C,Port_C,Port_C,Port_C,Port_C,Port_C};
ADC_channel_t Channel_used[16];

//para el ADC se debe habilitar el APB2 la resolucion es de 12 bits para 16 canales, 16 pines
//5v/4096 * input  = adc

void Channel_SQR_position(ADC_channel_t Channel){
	ADC_register_offset_t used_SQR = ADC_SQR3;
	uint32_t SQR_position_start =0;

 if(Channel<=5){
	 used_SQR = ADC_SQR3;
	 SQR_position_start = (uint32_t)ADC_SQ1+Channel;

 }else if (Channel>5 && Channel<=11){
	 used_SQR = ADC_SQR2;
	 SQR_position_start = (uint32_t)ADC_SQ7+Channel;

 }else{
	 used_SQR = ADC_SQR1;
	 SQR_position_start = (uint32_t)ADC_SQ13+Channel;

 }

	uint32_t volatile *ADC_REG_SQR = (uint32_t volatile*)(ADC1_ADDRESS + used_SQR); //agregamos el 0x14 par indicar output

	*ADC_REG_SQR &=  ~(0x1F<<SQR_position_start);
	*ADC_REG_SQR |=  (Channel<<SQR_position_start); //se asigna el numero del canal


}


void ADC_Clock(Enabled_Disabled_t state){

	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(RCC_ADDRESS + RCC_OFFSET_APB2ENR);
    if(state){
        *pClockControlReg |= ADC1_ENABLE;
    }else{
        *pClockControlReg &= ~ADC1_ENABLE;
    }
}


Status_code_t ADC_Init(ADC_channel_t Channel){

	if(Channel == Channel_2 || Channel == Channel_3 || Channel>Channel_15){
		return ADC_channel_not_available;
	}
	ADC_Clock(Enabled);
	SetPinMode(ADC_PORT_MAP[Channel], ADC_PIN_MAP[Channel], Analog_mode);

	Channel_SQR_position(Channel); //hasta aqui vamos 16:19

	return Success;

}

uint32_t ADC_Read(ADC_channel_t Channel){

return 0;

}

void ADC_Deinit(void){
	ADC_Clock(Disabled);

}

