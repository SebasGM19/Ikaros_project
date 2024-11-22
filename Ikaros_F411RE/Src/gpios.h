/*
 * Basic_settings.h
 *
 *  Created on: Feb 4, 2024
 *      Author: Sebastian G.M.
 */


#ifndef BASIC_SETTINGS_H_
#define BASIC_SETTINGS_H_

#define EXTI0_HANDLER EXTI0_IRQHandler
#define EXTI1_HANDLER EXTI1_IRQHandler

#define EXTI2_HANDLER EXTI2_IRQHandler
#define EXTI3_HANDLER EXTI3_IRQHandler

#define EXTI4_HANDLER EXTI4_IRQHandler
#define EXTI9_5_HANDLER EXTI9_5_IRQHandler

#define EXTI15_10_HANDLER EXTI15_10_IRQHandler

#define MAX_GPIOS	(16)

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
	SYS_AF,
	TIM1_TIM2,
	TIM3_TIM4_TIM5,
	TIM9_TIM10_TIM11,
	I2C1_I2C2_I2C3,
	SPI1_I2S1_SPI2_I2S2_SPI3_12S3,
	SPI2_I2S2_SPI3_I2S3_SPI4_I2S4_SPI5_I2S5,
	SPI3_I2S3_USART1_USART2,
	USART_6,
	I2C2_I2C3,
	OTG1_F1,
	SDIO_ = 12,

}Alternate_function_map_t;

typedef enum{
	Input,
	Output,
	Alt_func_mode,
	Analog_mode
}PinMode_t;

typedef enum{

	GPIOx_MODER_OFFSET =	0x00,
	GPIOx_OTYPER_OFFSET =	0x04,
	GPIOx_OSPEEDR_OFFSET =	0x08,
	GPIOx_PUPDR_OFFSET =	0x0C,
	GPIOx_IDR_OFFSET =		0x10,
	GPIOx_ODR_OFFSET =		0x14,
	GPIOx_BSRR_OFFSET =		0x18,
	GPIOx_LCKR_OFFSET =		0x1C,
	GPIOx_AFRL_OFFSET =		0x20,
	GPIOx_AFRH_OFFSET =		0x24

}GPIO_register_offset_t;

typedef enum{
	No_pull_No_Down,
	Pull_Up,
	Pull_Down

}GPIO_UP_DOWN_STATE_t;


typedef enum{
	Falling_edge,
	Rising_edge,

}GPIO_Exti_Config_t;


typedef enum{
	EXTI_Port_A,
	EXTI_Port_B,
	EXTI_Port_C,
	EXTI_Port_D,
//	EXTI_Port_E,
//	EXTI_Port_H =7,

}GPIO_Exti_Port_t;


Status_code_t SetPinMode(Set_Port_t Port_define, Pin_number_t Pin_defined, PinMode_t Mode);
Gpio_State_Control_t GPIO_DigitalRead(Set_Port_t Port_define, Pin_number_t Pin_defined);
Gpio_State_Control_t GPIO_DigitalWrite(Set_Port_t Port_define, Pin_number_t Pin_defined, Gpio_State_Control_t State);
void GpioSetAlternativeFunction(Set_Port_t Port_define, Pin_number_t Pin_defined, Alternate_function_map_t AF);
Status_code_t GpioPullUpDownState(Set_Port_t Port_define, Pin_number_t Pin_defined, GPIO_UP_DOWN_STATE_t GPIO_State);


void EXTI0_HANDLER(void);
Status_code_t GPIO_Init_EXTI0(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode);
void GPIO_Enable_EXTI0(void);
void GPIO_Disable_EXTI0(void);
void GPIO_Deinit_EXTI0(void);


void EXTI1_HANDLER(void);
Status_code_t GPIO_Init_EXTI1(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode);
void GPIO_Enable_EXTI1(void);
void GPIO_Disable_EXTI1(void);
void GPIO_Deinit_EXTI1(void);

void EXTI2_HANDLER(void);
Status_code_t GPIO_Init_EXTI2(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode);
void GPIO_Enable_EXTI2(void);
void GPIO_Disable_EXTI2(void);
void GPIO_Deinit_EXTI2(void);

void EXTI3_HANDLER(void);
Status_code_t GPIO_Init_EXTI3(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode);
void GPIO_Enable_EXTI3(void);
void GPIO_Disable_EXTI3(void);
void GPIO_Deinit_EXTI3(void);

void EXTI4_HANDLER(void);
Status_code_t GPIO_Init_EXTI4(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode);
void GPIO_Enable_EXTI4(void);
void GPIO_Disable_EXTI4(void);
void GPIO_Deinit_EXTI4(void);

void EXTI9_5_HANDLER(void);
Status_code_t GPIO_Init_EXTI9_To_EXTI5(GPIO_Exti_Port_t EXTI_Port, Pin_number_t Pin_defined, GPIO_Exti_Config_t EXTI_mode);
void GPIO_Enable_EXTI9_To_EXTI5(Pin_number_t Pin_defined);
void GPIO_Disable_EXTI9_To_EXTI5(Pin_number_t Pin_defined);
void GPIO_Deinit_EXTI9_To_EXTI5(Pin_number_t Pin_defined);

void EXTI15_10_HANDLER(void);
Status_code_t GPIO_Init_EXTI15_To_EXTI10(GPIO_Exti_Port_t EXTI_Port, Pin_number_t Pin_defined, GPIO_Exti_Config_t EXTI_mode);
void GPIO_Enable_EXTI15_To_EXTI10(Pin_number_t Pin_defined);
void GPIO_Disable_EXTI15_To_EXTI10(Pin_number_t Pin_defined);
void GPIO_Deinit_EXTI15_To_EXTI10(Pin_number_t Pin_defined);


void GPIO_EXTI_Trigger_seleccion(Pin_number_t Pin_defined,GPIO_Exti_Config_t EXTI_mode);
Status_code_t GPIO_Save_EXTI_PIN(Pin_number_t Pin_defined);
void GPIO_Set_EXTI_Line(GPIO_Exti_Port_t EXTI_Port,Pin_number_t Pin_defined);
void GPIO_Delete_EXTI_PIN(Pin_number_t Pin_defined);
void GPIO_EXTI_Mask(Pin_number_t Pin_defined,Enabled_Disabled_t Intention);
void GPIO_EXTI_Clean_Flag(Pin_number_t Pin_defined);
void GPIO_EXTI_Clean_Group_Of_Flag(Pin_number_t Pin_defined,RegAuxClean_t bit_to_clear);


#endif /* BASIC_SETTINGS_H_ */
