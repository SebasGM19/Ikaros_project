/*
 * Keypad_4x4.c
 *
 *  Created on: Apr 13, 2024
 *      Author: Sebastian G.M.
 */
#include "keypad_4x4.h"

uint8_t KeypadMap[MAX_ROWS][MAX_COLUMNS] = {
	{'1','2','3','A'},
	{'4','5','6','B'},
	{'7','8','9','C'},
	{'*','0','#','D'}
};

Status_code_t Init_keypad(keypad_alternatives_t keypad_alternative){

	Set_Port_t Port_option = Port_A; //set as a default
	uint16_t volatile PositionsOfPin =0;
	uint8_t input_output_count =0;

	switch(keypad_alternative){
	case keypad_PortA:
		Port_option = Port_A;
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

uint32_t Read_keypad(keypad_alternatives_t keypad_alternative){

	Set_Port_t Port_option = Port_A; //set as a default
	uint8_t dataArray[MAX_INPUT_DATA];
	uint8_t positionXY =0;
	uint8_t array_position =0;
	uint32_t capture_value =0;
	memset(dataArray,'\0',MAX_INPUT_DATA);


	switch(keypad_alternative){
	case keypad_PortA:
		Port_option = Port_A;
		break;
	case keypad_PortC:
		Port_option = Port_C;
		break;
	default:
		return OptionNotSupported;
	}


	do{
		rowSequence(Port_option, keypad_alternative, (first_row +positionXY));
		columnSequence(Port_option,keypad_alternative,(first_row + positionXY),dataArray,&array_position);
		positionXY++;
		if(positionXY>3){
			positionXY=0;
		}
	}while((dataArray[(array_position)]!= HASH) && array_position < MAX_INPUT_DATA);

	if(array_position == 0){ //if # is only pressed will return 0
		return 0;
	}

	uint8_t finalDataArray[(array_position)];
	memset(finalDataArray,'\0',(array_position));
	strncat((char *)(finalDataArray), (const char *)(dataArray),(array_position));
	capture_value = (uint32_t)atoi((char *)finalDataArray);

	return capture_value;
}

void rowSequence(Set_Port_t Port_option, Pin_number_t Pin_defined, row_to_low_states_t row_to_low){

	for(uint8_t i =0; i<NUM_OUTPUT_INPUT_COUNT; i++){ //aqui se mueve en filas

		if(i == row_to_low){
			GPIO_DigitalWrite(Port_option, (Pin_defined+i),Low);
		}else{
			GPIO_DigitalWrite(Port_option,(Pin_defined+i),High);
		}
	}

}

void columnSequence(Set_Port_t Port_option, Pin_number_t Pin_defined, row_to_low_states_t row_to_low, uint8_t* pDataArray, uint8_t* data_lenght){

	uint8_t pressed_boton=0;

	for(uint8_t column = 0; column<NUM_OUTPUT_INPUT_COUNT; column++){ //aqui se mueve entre columnas

		if(!GPIO_DigitalRead(Port_option, (Pin_defined+column+INPUT_OFFSET))){

			while(!GPIO_DigitalRead(Port_option, (Pin_defined+column+INPUT_OFFSET)));//wait until the button is not pressed
			/*later put a condition to avoid the infinite button pressed*/

			Delay(100000); //small delay to avoid troubles

			if(column == fourth_column){
				pressed_boton = (ASCII_A)+column; //for A,B,C,D but will do nothing

			}else if(column == first_column && row_to_low == fourth_row){
				pressed_boton = STAR; //FOR * but will do nothing

			}else if(column == third_column && row_to_low == fourth_row){
				pressed_boton = HASH; //to exit the loop
				pDataArray[(*data_lenght)] = pressed_boton;

			}else{
				pressed_boton = KeypadMap[row_to_low][column];
				pDataArray[(*data_lenght)] = pressed_boton;
				(*data_lenght)+=1;
			}
		}

	}

}
