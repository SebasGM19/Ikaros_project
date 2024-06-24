/*
 * lcd.h
 *
 *  Created on: Jun 12, 2024
 *      Author: sebas
 */

#ifndef LCD_H_
#define LCD_H_

#include "system_settings.h"
#include "gpios.h"

//NOTE: CHANGE THE DEFINE TO USE ANOTHER LCD WITH DIFERENT DIMENTIONS
#define LCD_16X2
//#define LCD_16X4

//NOTE:
/*
 * ROW 1 start at 0x00 and end in 0x0F it meand is map from = 0 to 15 in Xaxis of the firts row(Yaxis 1)
 */

#define SET_8_BITS_MODE				(0x30)
#define SET_4_BITS_CONFIGURATION	(0x20)
#define SET_2_LINES_QUALITY_5X8		(0x28)
#define START_LCD_WITHOUT_CURSOR	(0x0C)
#define CLEAN_SCREEN		 		(0x01)

#define POSITION_CERO (0x80)
#define START_AT_ROW1	(0x00)
#define START_AT_ROW2	(0x40)

#define RETURN_HOME						(0x02) //return cursor to position (0,1)
#define MOVE_CURSOR_TO_LEFT				(0x10)
#define MOVE_CURSOR_TO_RIGHT			(0x14)
#define MOVE_DISPLAY_TO_LEFT			(0x18)
#define MOVE_DISPLAY_TO_RIGHT			(0x1C)


#if defined(LCD_16X2)

#define X_LENGTH (16)
#define Y_HEIGHT (2)

#elif defined(LCD_16X4)

#define X_LENGTH (16)
#define Y_HEIGHT (4)
#define START_AT_ROW3	(0x10)
#define START_AT_ROW4	(0x50)

#else
#error "Display Height Not Available"
#endif



typedef struct{
	Pin_number_t PIN_RS;
	Pin_number_t PIN_RW;
	Pin_number_t PIN_E;
	Pin_number_t PIN_D4;
	Set_Port_t PORT;
}controls_gpios_t;


typedef enum {

	lcd_PortB=4,
	lcd_PortC=7

}lcd_alternative_t;


typedef enum{
	set_command,
	write_command
}command_type_t;

typedef enum{
	move_left,
	move_right

}direction_to_move_t;

void set_controls_gpios(Pin_number_t RS, Pin_number_t RW, Pin_number_t E,Set_Port_t PORT);
Status_code_t Send_command(uint8_t command, command_type_t type);
Status_code_t lcd_init(lcd_alternative_t lcd_option);
void lcd_print(uint8_t* data, uint8_t data_lenght);
Status_code_t lcd_printXY(uint8_t X_axis, uint8_t Y_axis, uint8_t* data, uint8_t data_lenght);
void lcd_clean_screen(void);
void lcd_return_to_home(void);
void lcd_move_display(direction_to_move_t direction);
void lcd_move_cursor(direction_to_move_t direction);
Status_code_t lcd_set_cursor_position(uint8_t X_axis, uint8_t Y_axis);


#endif /* LCD_H_ */
