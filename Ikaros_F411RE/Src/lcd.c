/*
 * lcd.c
 *
 *  Created on: Jun 12, 2024
 *      Author: sebas
 */
#include "lcd.h"


controls_gpios_t volatile control_alternative;
//uint8_t volatile coordinate_X = 0;//Coordinates
//uint8_t volatile coordinate_Y = 0;
//
//
//void set_new_coordinates(uint8_t new_X_value, uint8_t new_Y_value){
//	coordinate_X = new_X_value;
//	coordinate_Y = new_Y_value;
//}
//
//void get_actual_coordinates(uint8_t X_position,uint8_t Y_position){
//	X_position = coordinate_X;
//	Y_position = coordinate_Y;
//}


/*
 * Example: 0001 1001
 * 0001
 * PIN_10 = 0, PIN_9 =0, PIN_8 = 0, PIN_7 = 1
 *
 * 1001
 * PIN_10 = 1, PIN_9 =0, PIN_8 = 0, PIN_7 = 1
 *
 */
void set_controls_gpios(Pin_number_t RS, Pin_number_t RW, Pin_number_t E,Set_Port_t PORT){
	control_alternative.PIN_RS = RS;
	control_alternative.PIN_RW = RW;
	control_alternative.PIN_E = E;
	control_alternative.PIN_D4 = (E+1);
	control_alternative.PORT = PORT;

}

Status_code_t Send_command(uint8_t command, command_type_t type){
	uint8_t MSB = 0;
	uint8_t LSB = 0;

	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_RS, type);

	MSB = (command & 0xF0)>>4;
	LSB = (command & 0x0F); 	//keep only the first 4 bits

	for(uint8_t i =0; i<4;i++){
		GPIO_DigitalWrite(control_alternative.PORT, (control_alternative.PIN_D4 + i), (MSB&1));
		MSB = MSB>>1;
	}

	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, High);
	Delay(10);
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, Low);
	Delay(10);


	for(uint8_t i =0; i<4;i++){
		GPIO_DigitalWrite(control_alternative.PORT, (control_alternative.PIN_D4 + i), (LSB&1));
		LSB = LSB>>1;
	}
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, High);
	Delay(10);
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, Low);
	Delay(10);

	return Success;

}

/*
 * function to init the lcd configuration selected, all the commands are set_commmands, it means, RS will be 0 and
 * RS in 1 for write data
 */
Status_code_t lcd_init(lcd_alternative_t lcd_alternative){

	uint16_t volatile PositionsOfPin =0;

	switch(lcd_alternative){
	case lcd_PortB:
		set_controls_gpios(Pin_4, Pin_5, Pin_6, Port_B);
		break;
	case lcd_PortC:
		set_controls_gpios(Pin_7, Pin_8, Pin_9, Port_C);
		break;
	default:
		return OptionNotSupported;
	}

	uint32_t volatile *pPort_ModeReg = (uint32_t volatile *)(control_alternative.PORT+ OFFSET_PORTS);
	ClockEnable(control_alternative.PORT, Enabled);

	PositionsOfPin = (uint16_t volatile)(lcd_alternative * 2);
	*pPort_ModeReg &= ~(clear_fourteen_bits<<PositionsOfPin);
	*pPort_ModeReg |= (0x1555 << PositionsOfPin);// equal to : 01 0101 0101 0101 set as output

	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_RW, Low); //this stay LOW in 4bit configuration

	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_RS, Low);
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, Low);

	/*set to low all gpio*/
	for(uint8_t i =0; i<4; i++){
		GPIO_DigitalWrite(control_alternative.PORT, (control_alternative.PIN_D4+i), Low);
	}

	Delay(100000);
	Send_command(SET_8_BITS_MODE, set_command);
	Delay(5000);
	Send_command(SET_8_BITS_MODE, set_command);
	Delay(5000);
	Send_command(SET_8_BITS_MODE, set_command);
	Delay(5000);
	Send_command(SET_4_BITS_CONFIGURATION, set_command);
	Delay(5000);
	Send_command(SET_2_LINES_QUALITY_5X8, set_command);
	Delay(5000);
	Send_command(START_LCD_WITHOUT_CURSOR, set_command);
	Delay(5000);
	Send_command(CLEAN_SCREEN, set_command);
	Delay(50000);
	Send_command(RETURN_HOME, set_command);
	Delay(50000);//minimum delay
	Send_command(POSITION_CERO, set_command);
	Delay(50000);
//	coordinate_Y = 1;
//	coordinate_X = 0;


	return Success;
}

/*
 * This function will only print the data starting at position 0,1
 */
void lcd_print(uint8_t* data, uint8_t data_lenght){

	uint8_t data_left_to_send =0;

	while(data_left_to_send < data_lenght){
		Send_command((*data), write_command);
//		coordinate_X++;
		data_left_to_send++;
		data++;
	}
//	coordinate_X++;
}


/*
 * In this function you must indicate the axis to start the print
 * the printed data will not be deleted from its position
 */
Status_code_t lcd_printXY(uint8_t X_axis, uint8_t Y_axis, uint8_t* data, uint8_t data_lenght){
	Status_code_t status = Success;
	status = lcd_set_cursor_position(X_axis, Y_axis);
	if(status!= Success){
		return status;
	}
	lcd_print(data, data_lenght);
	return Success;
}

/*
 * this function cleans the screen and return the position to the beginning
 */
void lcd_clean_screen(void){
	Send_command(CLEAN_SCREEN, set_command);
	Delay(50000);//minimum delay
	lcd_return_to_home();
}

/*
 * this function moves the data in te display one position to the selected direction
 */
void lcd_move_display(direction_to_move_t direction){
	if(direction == move_left){
		Send_command(MOVE_DISPLAY_TO_LEFT, set_command);

	}else{
		Send_command(MOVE_DISPLAY_TO_RIGHT, set_command);
	}
	Delay(500);//minimum delay

}

void lcd_move_cursor(direction_to_move_t direction){
	if(direction == move_left){
		Send_command(MOVE_CURSOR_TO_LEFT, set_command);

	}else{
		Send_command(MOVE_CURSOR_TO_RIGHT, set_command);
	}
	Delay(500);//minimum delay
}

//returns to position 0,1
void lcd_return_to_home(void){
	Send_command(RETURN_HOME, set_command);
	Delay(50000);//minimum delay

}

Status_code_t lcd_set_cursor_position(uint8_t X_axis, uint8_t Y_axis){
	uint8_t Y_position;
	uint8_t real_position;
	switch(Y_axis){
	case 0:
		Y_position = POSITION_CERO | START_AT_ROW1;
		break;
	case 1:
		Y_position = POSITION_CERO | START_AT_ROW2;
		break;
#if defined(LCD_16X4)
	case 2:
		Y_position = POSITION_CERO | START_AT_ROW3;
		break;
	case 3:
		Y_position = POSITION_CERO | START_AT_ROW4;
		break;
#endif
	default:
		return lcd_Wrong_Y_Axis_Value;
		break;
	}
//	coordinate_Y = Y_axis;

	real_position = Y_position | X_axis;
	Send_command(real_position, set_command);
	Delay(5000);//minimum delay
	return Success;
}

