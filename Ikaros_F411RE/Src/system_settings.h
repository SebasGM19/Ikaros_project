/*
 * system_settings.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Sebastian G.M.
 */

#ifndef SYSTEM_SETTINGS_H_
#define SYSTEM_SETTINGS_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "stdbool.h"
#include <string.h>
#include <stdlib.h>



#define BOARD_CLOCK 			(16000000) //set to 16MHz
#define PSC_TO_MICROSEC_DELAY	(uint32_t)(16) //minimum to microsecond count
#define PSC_TO_MILLISEC_DELAY	(uint32_t)(16000) //minimum to microsecond count
#define USEC_TO_DELAY(CLOCK_BASE,PSC,DELAY)	 (uint32_t)((((CLOCK_BASE) / (PSC)) * ((DELAY) * (0.000001))) )
#define MILLSEC_TO_DELAY(CLOCK_BASE,PSC,DELAY)	 (uint32_t)((((CLOCK_BASE) / (PSC)) * ((DELAY) * (0.001))) )



typedef enum{
	Success,
	PinNotAvailable,
	DelayTimeNotSupported,
	ClockNotSupported,
	OptionNotSupported,
	WrongParameter,
	TimeSetNotSuported,

	lcd_Wrong_X_Axis_Value,
	lcd_Wrong_Y_Axis_Value,

	ADC_channel_not_available,

}Status_code_t;

typedef enum{
	Disabled,
	Enabled
}Enabled_Disabled_t;

typedef enum{
	Clean_one_bit =			0x01, 	//to clean 1 bit
	Clear_two_bits=			0x03, 	//to clean 1 GPIOS
	Clear_four_bits = 		0xF, 	//to clean 2 GPIOS
	Clear_eight_bits =		0xFF, 	//to clean 4 GPIOS
	clear_fourteen_bits = 	0x3FFF, 	//to clean 7 GPIOS
	Clear_sixteen_bits =	0xFFFF 	//to clean 8 GPIOS
}RegAuxClean_t;


typedef enum{

	Port_A = 0x40020000,
	Port_B = 0x40020400,
	Port_C = 0x40020800,
	Port_D = 0x40020C00,
//	Port_E = 0x40021000,
//	Port_H = 0x40021C00

}Set_Port_t;

typedef enum{
	Clk_PortA,
	Clk_PortB,
	Clk_PortC,
	Clk_PortD,
//	Clk_PortE,
//	Clk_PortH=7,
}Clock_enabled_t; //BUS AHB1 todos estos

typedef enum{
//	TIM2_ADDRESS = 			0x40000000, //APB1 BUS
//	TIM3_ADDRESS =			0x40000400,
//	TIM4_ADDRESS =			0x40000800,
//	TIM5_ADDRESS =			0x40000C00,
	RTC_BKP_REG_ADDRESS = 	0x40002800,
	WWDG_ADDRESS = 			0x40002C00,
	IWDG_ADDRESS = 			0x40003000,
	I2S2ext_ADDRESS =		0x40003400,
	SPI2_I2S2_ADDRESS =		0x40003800,
	SPI3_I2S3_ADDRESS = 	0x40003C00,
	I2S3ext_ADDRESS = 		0x40004000,
	USART2_ADDRESS = 		0x40004400,
	I2C1_ADDRESS = 			0x40005400,
	I2C2_ADDRESS = 			0x40005800,
	I2C3_ADDRESS = 			0x40005C00,
	PWR_ADDRESS = 			0x40007000, //APB1

//	TIM1_ADDRESS = 			0x40010000, //APB2 BUS
	USART1_ADDRESS = 		0x40011000,
	USART6_ADDRESS = 		0x40011400,
	ADC1_ADDRESS = 			0x40011000,
	SDIO_ADDRESS = 			0x40012C00,
	SPI1_I2C1_ADDRESS =		0x40013000,
	SPI4_I2C4_ADDRESS = 	0x40013400,
	SYSCFG_ADDRESS= 		0x40013800,
	EXT1_ADDRESS =			0x40013C00,
//	TIM9_ADDRESS = 			0x40014000,
//	TIM10_ADDRESS = 		0x40014400,
//	TIM11_ADDRESS = 		0x40014800,
	SPI5_I2S5_ADDRESS = 	0x40015000,

	CRC_ADDRESS = 			0x40023000, //AHB1 BUS
	RCC_ADDRESS =			0x40023800,
	FLASH_INT_REG_ADDRESS = 0x40023C00,
	DMA1_ADDRESS = 			0x40026000,
	DMA2_ADDRESS = 			0x40026400,

	USB_OTG_FS_ADDRESS = 	0x50000000 //BUS AHB2

}memoryMapAddress_t;


typedef enum{
	TIM2_ADDRESS = 	0x40000000, //APB1 BUS
	TIM3_ADDRESS =	0x40000400,
	TIM4_ADDRESS =	0x40000800,
	TIM5_ADDRESS =	0x40000C00,

	TIM1_ADDRESS = 	0x40010000, //APB2 BUS
	TIM9_ADDRESS = 	0x40014000,
	TIM10_ADDRESS = 0x40014400,
	TIM11_ADDRESS = 0x40014800,

}TimerMapAddr_t;

typedef enum{
	RCC_OFFSET_CR =			0x00,
	RCC_OFFSET_PLLCFGR =	0x04,
	RCC_OFFSET_CFGR =		0x08,
	RCC_OFFSET_CIR = 		0x0C,
	RCC_OFFSET_AHB1RSTR =	0x10,
	RCC_OFFSET_AHB2RSTR =	0x14,
	RCC_OFFSET_APB1RSTR = 	0x20,
	RCC_OFFSET_APB2RSTR = 	0x24,
	RCC_OFFSET_AHB1ENR =	0x30,
	RCC_OFFSET_AHB2ENR =	0x34,
	RCC_OFFSET_APB1ENR =	0x40,
	RCC_OFFSET_APB2ENR =	0x44,
	RCC_OFFSET_AHB1LPENR = 	0x50,
	RCC_OFFSET_AHB2LPENR =	0x54,
	RCC_OFFSET_APB1LPENR =	0x60,
	RCC_OFFSET_APB2LPENR =	0x64,
	RCC_OFFSET_BDCR = 		0x70,
	RCC_OFFSET_CSR =		0x74,
	RCC_OFFSET_SSCGR =		0x80,
	RCC_OFFSET_PLLI2S = 	0x84,
	RCC_OFFSET_DCKCFGR =	0x8C

}RCC_offset_t;

typedef enum{
	OFFSET_PORTS =	0x00,
}peripherial_offset_t;


Status_code_t ClockEnable(Set_Port_t Port_define, Enabled_Disabled_t Intention);
Status_code_t Delay(uint32_t microseconds);
void resetDelay(TimerMapAddr_t Timer);
Status_code_t Peripherial_delay(uint16_t miliseconds);

void ftoa(float decimalData, uint8_t* cadena, uint8_t decimales);

#endif /* SYSTEM_SETTINGS_H_ */
