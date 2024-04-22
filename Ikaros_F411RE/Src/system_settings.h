/*
 * system_settings.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Sebastian G.M.
 */

#ifndef SYSTEM_SETTINGS_H_
#define SYSTEM_SETTINGS_H_

#include <stdint.h>
#include "stm32f4xx.h"

typedef enum{
	Success,
	PinNotAvailable,
	DelayTimeNotSupported,
	ClockNotSupported,
	OptionNotSupported,
	WrongParameter
}Status_code_t;

typedef enum{
	Disabled,
	Enabled
}Enabled_Disabled_t;

typedef enum{
	Clean_one_bit =0x01,
	Clear_two_bits=0x03,
	Clear_four_bits = 0xF,
	Clear_eight_bits =0xFF,
	Clear_sixteen_bits =0xFFFF
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
	TIM2_ADDRESS = 			0x40000000, //APB1 BUS
	TIM3_ADDRESS =			0x40000400,
	TIM4_ADDRESS =			0x40000800,
	TIM5_ADDRESS =			0x40000C00,
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

	TIM1_ADDRESS = 			0x40010000, //APB2 BUS
	USART1_ADDRESS = 		0x40011000,
	USART6_ADDRESS = 		0x40011400,
	ADC1_ADDRESS = 			0x40011000,
	SDIO_ADDRESS = 			0x40012C00,
	SPI1_I2C1_ADDRESS =		0x40013000,
	SPI4_I2C4_ADDRESS = 	0x40013400,
	SYSCFG_ADDRESS= 		0x40013800,
	EXT1_ADDRESS =			0x40013C00,
	TIM9_ADDRESS = 			0x40014000,
	TIM10_ADDRESS = 		0x40014400,
	TIM11_ADDRESS = 		0x40014800,
	SPI5_I2S5_ADDRESS = 	0x40015000,

	CRC_ADDRESS = 			0x40023000, //AHB1 BUS
	RCC_ADDRESS =			0x40023800,
	FLASH_INT_REG_ADDRESS = 0x40023C00,
	DMA1_ADDRESS = 			0x40026000,
	DMA2_ADDRESS = 			0x40026400,

	USB_OTG_FS_ADDRESS = 	0x50000000 //BUS AHB2

}memoryMapAddress_t;

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
	OFFSET_PORTS =0x00,
}peripherial_offset_t;


typedef enum{
	TIMER_2,
	TIMER_3,
	TIMER_4,
	TIMER_5

}timers_enb_t;

typedef enum{
	TIMx_CR1 =0x00,
	TIMx_CR2 =0x04,
	TIMx_SMCR =0x08,
	TIMx_DIER = 0x0C,
	TIMx_SR = 0x10,
	TIMx_EGR = 0x14,
	TIMx_CCMR1 = 0x18,
	TIMx_CCMR2 = 0x1C,
	TIMx_CCER = 0x20,
	TIMx_CNT = 0x24, //important
	TIMx_PSC = 0x28, //important
	TIMx_ARR = 0x2C, //important
	TIMx_CCR1 = 0x34,
	TIMx_CCR2 = 0x38,
	TIMx_CCR3 =0x3C,
	TIMx_CCR4 = 0x40,
	TIMx_DCR = 0x48,
	TIMx_DMAR = 0x4C,
	TIM2_TIM5_OR = 0x50

}TIM2_TIM5_register_offset_t;

typedef enum{
	TIMx_DIV_BY_1 = 	(1u<<1),
	TIMx_DIV_BY_2 =		(1u<<2),
	TIMx_DIV_BY_4=		(1u<<3),
	TIMx_DIV_BY_8=		(1u<<4),
	TIMx_DIV_BY_16=		(1u<<5),
	TIMx_DIV_BY_32=		(1u<<6),
	TIMx_DIV_BY_64=		(1u<<7),
	TIMx_DIV_BY_128=	(1u<<8),
	TIMx_DIV_BY_256=	(1u<<9),
	TIMx_DIV_BY_512=	(1u<<10)

}TIMx_preescaler_t;

Status_code_t ClockEnable(Set_Port_t Port_define, Enabled_Disabled_t Intention);
Status_code_t Delay(uint32_t Miliseconds);

#endif /* SYSTEM_SETTINGS_H_ */
