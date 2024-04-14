/*
 * Keypad_4x4.c
 *
 *  Created on: Apr 13, 2024
 *      Author: S4IOT
 */
#include "keypad_4x4.h"

Status_code_t init_keypad(keypad_alternatives_t keypad_alternative){

	Set_Port_t Port_option = Port_A; //set as a default
	uint16_t volatile PositionsOfPin =0;
	uint8_t input_output_count =0;

	switch(keypad_alternative){
	case keypad_PortA:
		Port_option = Port_A;
		break;
	case keypad_PortB:
		Port_option = Port_B;
		break;
	case keypad_PortC:
		Port_option = Port_C;
		break;
	default:
		return OptionNotSupported;
	}

	uint32_t volatile *pPort_ModeReg = (uint32_t volatile *)(Port_option+ OFFSET_PORTS);
	ClockEnable(Port_option, Enabled);

	PositionsOfPin = (uint16_t volatile)keypad_alternative*2;
	*pPort_ModeReg &= ~(Clear_sixteen_bits<<PositionsOfPin);

	while(input_output_count < NUM_OUTPUT_INPUT_COUNT){
		//SET_THE OUTPUT
		PositionsOfPin = (uint16_t volatile)((keypad_alternative+input_output_count)*2);
		*pPort_ModeReg |= (Output<<PositionsOfPin);
		GPIO_DigitalWrite(Port_option,(keypad_alternative+input_output_count),High); //set them as high for default

		//SET THE INPUTS
		PositionsOfPin = (uint16_t volatile)((keypad_alternative + input_output_count + INPUT_OFFSET)*2);
		*pPort_ModeReg |= (Input<<PositionsOfPin);
		GpioPullUpDownState(Port_option,(keypad_alternative + input_output_count + INPUT_OFFSET),Pull_Up);
		input_output_count++;
	}


	return Success;
}

uint32_t read_keybord(keypad_alternatives_t keypad_alternative){

	do{


	}while(1);

	return 0;

}
