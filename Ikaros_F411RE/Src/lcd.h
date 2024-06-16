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

#define LCD_16X2
//#define LCD_16X4


#if defined(LCD_16X2)
#define X_LENGTH (16)
#define Y_HEIGHT (2)

#elif defined(LCD_16X4)

#define X_LENGTH (16)
#define Y_HEIGHT (4)

#else
#error "Display Longitude Not Available"
#endif



typedef struct{
	Pin_number_t PIN_RS;
	Pin_number_t PIN_RW;
	Pin_number_t PIN_E;
	Pin_number_t PIN_D4;
	Set_Port_t PORT;
}controls_gpios_t;


typedef enum {
	X_axis,
	Y_axis
}axis_t;


typedef enum {

	lcd_PortB=7,
	lcd_PortC=10

}lcd_alternative_t;


typedef enum{
	set_command,
	write_command
}command_type_t;

void set_new_coordinates(uint8_t new_X_value, uint8_t new_Y_value);
void get_actual_coordinates(uint8_t X_position,uint8_t Y_position);
void set_controls_gpios(Pin_number_t RS, Pin_number_t RW, Pin_number_t E,Set_Port_t PORT);
Status_code_t Send_command(uint8_t command, command_type_t type);
Status_code_t Init_lcd(lcd_alternative_t lcd_option);
void lcd_print(uint8_t* data, uint8_t *data_lenght);
void lcd_printXY(uint8_t X_axis, uint8_t Y_axis, uint8_t* data, uint8_t *data_lenght);
void lcd_clean_screen(void);
void lcd_return_to_home(void);



#endif /* LCD_H_ */
