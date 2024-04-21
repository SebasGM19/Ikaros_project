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
#define MAX_INPUT_DATA				(10)
#define ASCII_A						(0x41)
#define HASHTAG						(0x23)
#define STAR						(0x2A)

#include "system_settings.h"
#include "gpios.h"
#include <string.h>
#include <stdlib.h>

typedef enum{
	keypad_PortC = 0, //start with in the firts GPIO of the alternative corresponding to the R1
	keypad_PortA = 4,
	keypad_PortB = 7,
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



/*
 * for reference R1 will be the lowest GPIO of the selected alternative
 */
Status_code_t init_keypad(keypad_alternatives_t keypad_alternative);
/*____________________________________________________________________________________________*/


uint32_t read_keybord(keypad_alternatives_t keypad_alternative);


void rowSequence(Set_Port_t Port_option, Pin_number_t Pin_defined, row_to_low_states_t row_to_low);

void columnSequence(Set_Port_t Port_option, Pin_number_t Pin_defined, row_to_low_states_t row_to_low, uint8_t* pDataArray, uint8_t* data_lenght);

#endif /* KEYPAD_4X4_H_ */
