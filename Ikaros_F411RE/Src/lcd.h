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

#define X_LENGTH (16)
#define Y_HEIGHT (2)


typedef enum {
	X_axis,
	Y_axis
}axis_t;


typedef enum {

	lcd_PortB=4,
	lcd_PortC=7

}lcd_alternative_t;

void set_new_coordinates(uint8_t new_X_value, uint8_t new_Y_value);
void get_actual_coordinates(uint8_t X_position,uint8_t Y_position);
Status_code_t Init_lcd(lcd_alternative_t lcd_option);



#endif /* LCD_H_ */
