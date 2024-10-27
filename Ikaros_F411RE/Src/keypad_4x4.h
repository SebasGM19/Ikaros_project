/*
 * Keypad_4x4.h
 *
 *  Created on: Apr 13, 2024
 *      Author: Sebastian G.M
 */

#ifndef KEYPAD_4X4_H_
#define KEYPAD_4X4_H_

#define NUM_OUTPUT_INPUT_COUNT		(4)
#define INPUT_OFFSET				(4)

#define MAX_ROWS					(4)
#define MAX_COLUMNS					(4)

#define MAX_INPUT_DATA				(9)
#define ASCII_A						(0x41)
#define HASH						(0x23)
#define STAR						(0x2A)

#include "system_settings.h"
#include "gpios.h"
#include "lcd.h"

typedef enum{
	keypad_PortA = 4,	//start with the first GPIO of the alternative corresponding to the R1
	keypad_PortC = 5,
}keypad_alternatives_t;

typedef enum{
	first_row,
	second_row,
	third_row,
	fourth_row
}row_to_low_states_t;

typedef enum{
	first_column,
	second_column,
	third_column,
	fourth_column
}columns_states_t;

typedef enum{
	print_disabled,
	print_enabled,

}print_activate_t;


/*
 * for reference R1 will be the lowest GPIO of the selected alternative
 */
Status_code_t Init_keypad(keypad_alternatives_t keypad_alternative);
/*____________________________________________________________________________________________*/
Status_code_t Deinit_keypad(keypad_alternatives_t keypad_alternative);


uint32_t Read_keypad(keypad_alternatives_t keypad_alternative);


void rowSequence(Set_Port_t Port_option, Pin_number_t Pin_defined, row_to_low_states_t row_to_low);

void columnSequence(Set_Port_t Port_option, Pin_number_t Pin_defined, row_to_low_states_t row_to_low, uint8_t* pDataArray, uint8_t* data_lenght, print_activate_t opPrint);

uint32_t print_keypad(keypad_alternatives_t keypad_alternative, uint8_t X_axis, uint8_t Y_axis);
#endif /* KEYPAD_4X4_H_ */
