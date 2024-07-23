/*
 * Basic_settings.c
 *
 *  Created on: Feb 4, 2024
 *      Author: Sebastian G.M.
 */

#include "gpios.h"

Gpio_State_Control_t GPIO_DigitalRead(Set_Port_t Port_define, Pin_number_t Pin_defined){

	uint32_t const volatile * const PORT_REG_INPUT = (uint32_t const volatile *const)(Port_define + GPIOx_IDR_OFFSET); //agregamos el 10 para decirle el offset

	if(((1<<Pin_defined) & (*PORT_REG_INPUT))==(1<<Pin_defined)){
		return High;
	}
	return Low;
}


Gpio_State_Control_t GPIO_DigitalWrite(Set_Port_t Port_define, Pin_number_t Pin_defined, Gpio_State_Control_t State){

	uint32_t volatile *PORT_REG_OUTPUT = (uint32_t volatile*)(Port_define + GPIOx_ODR_OFFSET); //agregamos el 0x14 par indicar output

	if(State){
		*PORT_REG_OUTPUT |= (1<<Pin_defined);
	}else{
		*PORT_REG_OUTPUT &= ~(1<<Pin_defined);

	}

	return State;

}


Status_code_t SetPinMode(Set_Port_t Port_define, Pin_number_t Pin_defined, PinMode_t Mode){

	uint32_t volatile *pPort_ModeReg = (uint32_t volatile *)((Port_define)+ OFFSET_PORTS);
	uint16_t volatile PositionsOfPin =0;
	Status_code_t status=Success;

	/*DO NOT MODIFY THIS IF STATEMENT, IMPLEMENTED TO AVOID TROUBLES WITH THE BOARD!!!*/
	if((Port_define == Port_A && (Pin_defined ==Pin_13 || Pin_defined ==Pin_14 || Pin_defined ==Pin_2|| Pin_defined ==Pin_3))
	|| (Port_define == Port_C && (Pin_defined ==Pin_14 || Pin_defined == Pin_15))
	|| (Port_define == Port_D && Pin_defined != Pin_2)
	|| (Port_define == Port_B && (Pin_defined == Pin_11 || Pin_defined == Pin_3))){

		return PinNotAvailable;
	}

	status = ClockEnable(Port_define, Enabled);

	PositionsOfPin = (uint16_t volatile)Pin_defined*2;
	*pPort_ModeReg &= ~(Clear_two_bits<<PositionsOfPin);

	switch(Mode){

		case Output:
			*pPort_ModeReg |= (Output<<PositionsOfPin);
			break;
		case Input:
			*pPort_ModeReg &= ~(Clear_two_bits<<PositionsOfPin); //si es input simplemente limpiara esas posiciones a 00
			break;
		case Alt_func_mode:
			*pPort_ModeReg |= (Alt_func_mode<<PositionsOfPin);
			break;
		case Analog_mode:
			*pPort_ModeReg |= (Analog_mode<<PositionsOfPin);
			break;
		default:
			status = OptionNotSupported;
			break;
	}

	return status;
}


Status_code_t GpioPullUpDownState(Set_Port_t Port_define, Pin_number_t Pin_defined, GPIO_UP_DOWN_STATE_t GPIO_State){
	uint32_t volatile *REG_PULL_UPdown = (uint32_t volatile*)(Port_define + GPIOx_PUPDR_OFFSET); //add 0x0C for offset to pull up or pull down
	uint16_t RealPosition=0;

	RealPosition = Pin_defined*2;
	*REG_PULL_UPdown &= ~(Clear_two_bits<<RealPosition); //limpiamos esa posicion del bit
	*REG_PULL_UPdown |= (GPIO_State<<RealPosition); //set the state in to pull
	return Success;

}
