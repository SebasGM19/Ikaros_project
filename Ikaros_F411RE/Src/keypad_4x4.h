/*
 * Keypad_4x4.h
 *
 *  Created on: Apr 13, 2024
 *      Author: S4IOT
 */

#ifndef KEYPAD_4X4_H_
#define KEYPAD_4X4_H_

#define NUM_OUTPUT_INPUT_COUNT		(4)
#define INPUT_OFFSET	(4)

#include "system_settings.h"
#include "gpios.h"

typedef enum{
	keypad_PortC = 0,
	keypad_PortA = 4,
	keypad_PortB = 7,
}keypad_alternatives_t;



/*
 * for reference R1 will be the less significant GPIO of the selected alternative
 */
Status_code_t init_keypad(keypad_alternatives_t keypad_alternative);
/*____________________________________________________________________________________________*/


uint32_t read_keybord(keypad_alternatives_t keypad_alternative);

#endif /* KEYPAD_4X4_H_ */
