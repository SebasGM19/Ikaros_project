/*
 * lcd.c
 *
 *  Created on: Jun 12, 2024
 *      Author: sebas
 */

#include "lcd.h"

uint8_t volatile coordinate_X = 0;//Coordinates
uint8_t volatile coordinate_Y = 0;

void set_new_coordinates(uint8_t new_X_value, uint8_t new_Y_value){
	coordinate_X = new_X_value;
	coordinate_Y = new_Y_value;
}

void get_actual_coordinates(uint8_t X_position,uint8_t Y_position){
	X_position = coordinate_X;
	Y_position = coordinate_Y;
}

Status_code_t Init_lcd(lcd_alternative_t lcd_alternative){

	Set_Port_t Port_option = Port_B; //set as a default
	uint16_t volatile PositionsOfPin =0;


	switch(lcd_alternative){
	case lcd_PortB:
		Port_option = Port_B;
		break;
	case lcd_PortC:
		Port_option = Port_C;
		break;
	default:
		return OptionNotSupported;
	}

	uint32_t volatile *pPort_ModeReg = (uint32_t volatile *)(Port_option+ OFFSET_PORTS);
	ClockEnable(Port_option, Enabled);

	PositionsOfPin = (uint16_t volatile)(lcd_alternative * 2);
	*pPort_ModeReg &= ~(clear_fourteen_bits<<PositionsOfPin);

	//set the inputs and outputs down here ¡pending!

	return Success;
}
