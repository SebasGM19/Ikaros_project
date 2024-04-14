/*
 * Basic_settings.h
 *
 *  Created on: Feb 4, 2024
 *      Author: Sebastian G.M.
 */


#ifndef BASIC_SETTINGS_H_
#define BASIC_SETTINGS_H_
#define MAX_GPIOS	(15)

#include "system_settings.h"

typedef enum{
	Low,
	High
}Gpio_State_Control_t;


typedef enum{
	Pin_0,
	Pin_1,
	Pin_2,
	Pin_3,
	Pin_4,
	Pin_5,
	Pin_6,
	Pin_7,
	Pin_8,
	Pin_9,
	Pin_10,
	Pin_11,
	Pin_12,
	Pin_13,
	Pin_14,
	Pin_15
}Pin_number_t;

typedef enum{
	Input,
	Output,
	Alt_func_mode,
	Analog_mode
}PinMode_t;

typedef enum{

	GPIOx_MODER_OFFSET =0x00,
	GPIOx_OTYPER_OFFSET =0x04,
	GPIOx_OSPEEDR_OFFSET =0x08,
	GPIOx_PUPDR_OFFSET =0x0C,
	GPIOx_IDR_OFFSET =0x10,
	GPIOx_ODR_OFFSET =0x14,
	GPIOx_BSRR_OFFSET =0x18,
	GPIOx_LCKR_OFFSET =0x1C,
	GPIOx_AFRL_OFFSET =0x20,
	GPIOx_AFRH_OFFSET =0x24

}GPIO_register_offset_t;


typedef enum{
	No_pull_No_Down,
	Pull_Up,
	Pull_Down

}GPIO_UP_DOWN_STATE_t;


Gpio_State_Control_t GPIO_DigitalRead(Set_Port_t Port_define, Pin_number_t Pin_defined);
Gpio_State_Control_t GPIO_DigitalWrite(Set_Port_t Port_define, Pin_number_t Pin_defined, Gpio_State_Control_t State);
Status_code_t SetPinMode(Set_Port_t Port_define, Pin_number_t Pin_defined, PinMode_t Mode);
Status_code_t GpioPullUpDownState(Set_Port_t Port_define, Pin_number_t Pin_defined, GPIO_UP_DOWN_STATE_t GPIO_State);


#endif /* BASIC_SETTINGS_H_ */
