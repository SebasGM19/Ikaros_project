/*
 * stepper_motor.h
 *
 *  Created on: Apr 7, 2024
 *      Author: Sebastian G.M
 */

#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_


#include "system_settings.h"
#include "gpios.h"
#include "string.h"

typedef enum{
	PortC_Op1 = 0xF, //from GPIO C0 to C3
	PortA_Op1 = 0xF0, //from GPIO A4 to A7
	PortA_Op2 = 0xF00,// from GPIO A8 to A11
	PortB_Op1 = 0xF000 //from GPIO B12 to B15

}Stepper_option_t;


/*
 * Set to output mode the first 8 GPIOS of the port C
 * the purpose of this is to use is to bipolar motor which normally needs 4 GPIOS,
 * in this case it is possible to connect 2 bipolar motor at the same time or something that
 * need to use 8 bits
 */
Status_code_t Init_8bits_Stepper_Motor(void);
/*____________________________________________________________________________________________*/
void Deinit_8bits_Stepper_Motor(void);

/*_____________________________________________________________________________________________
 *this function change the logic state of the first 8 GPIO of the Port_C
 *depending of passing value of the argument
 *for example -> decimal value= 12 (00001100) will turn on the GPIO C2 and C3
 *
 */
Status_code_t Write_8bits_Stepper_Motor(uint8_t secuence);
/*____________________________________________________________________________________________*/


Status_code_t Init_4bits_Stepper_Motor(Stepper_option_t Stepper_alternative);
/*____________________________________________________________________________________________*/
Status_code_t Deinit_4bits_Stepper_Motor(Stepper_option_t Stepper_alternative);

/*_____________________________________________________________________________________________
 * this function is use to activate the sequence of 4 bit from the user request
 * this function has been develop to be use a simple sequence of 4 LSB and it automatically
 * moves the bits depending of the alternative selected
 *
 * the example of the basic sequence to use for stepper motors is:
 * uint8_t sec4bit[8]={9,8,12,4,6,2,3,1}; it can be seen as:
 * uint8_t sec4bit[8]={0b1001,0b1000,0b1100,0b0100,0b0110,0b0010,0b0011,0b0001};
 * THE LSB IN THE SECUENCE CORRESPOND TO THE LOWES GPIO FROM THAT ALTERNATIVE SHOWS IN THE
 * DOCUMENTATION
 */
Status_code_t Write_4bits_Stepper_Motor(Stepper_option_t Stepper_alternative, uint32_t secuence);
/*____________________________________________________________________________________________*/


uint8_t *search_bits(Stepper_option_t Stepper_alternative, uint8_t* found_bits);
/*____________________________________________________________________________________________*/

#endif /* STEPPER_MOTOR_H_ */
