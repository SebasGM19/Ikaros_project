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
#include <math.h>




#define BOARD_CLOCK 			(16000000U) //set to 16MHz
#define AHB2_CLOCK				(16000000U)
#define AHB1_CLOCK				(16000000U)
#define APB2_CLOCK				(16000000U)

#define APB1_CLOCK				(16000000U)
#define PSC_TO_MICROSEC_DELAY	(uint32_t)(16U) //minimum to microsecond count
#define PSC_TO_MILLISEC_DELAY	(uint32_t)(16000U) //minimum to microsecond count
#define USEC_TO_DELAY(CLOCK_BASE,PSC,DELAY)	 (uint32_t)((((CLOCK_BASE) / (PSC)) * ((DELAY) * (0.000001))) )
#define MILLSEC_TO_DELAY(CLOCK_BASE,PSC,DELAY)	 (uint32_t)((((CLOCK_BASE) / (PSC)) * ((DELAY) * (0.001))) )

#define BINARY_BASE			(2U)
#define OCTAL_BASE			(8U)
#define DECIMAL_BASE		(10U)
#define HEXADECIMAL_BASE	(16U)

typedef enum{
	Success,
	PinNotAvailable,
	DelayTimeNotSupported,
	ClockNotSupported,
	OptionNotSupported,
	WrongParameter,
	TimeSetNotSuported,
	Unknown,
	Error,

	lcd_Wrong_X_Axis_Value,
	lcd_Wrong_Y_Axis_Value,

	ADC_channel_not_available,
	ADC_No_More_conversion_space,
	ADC_channel_already_initialized,
	ADC_channel_not_initialized,
	ADC_channel_already_deinitialized,
	ADC_ADConverterd_off,

	EXTI_Line_already_use,
	EXTI_Pin_Not_Allowed,

	TIMx_incorrect,
	PWM_frecuency_not_supported_in_this_mode,

	USART_alternative_not_supported,
	USART_baud_rate_out_of_limit,
	USART_read_data_not_empty,
	USART_overrun_error,
	USART_parity_error,
	USART_framing_error,
	USART_noise_error,
	Timeout,

	WWDG_invalid_parameter,

	I2C_frecuency_not_supported,
	I2C_clock_value_out_of_range,
	I2C_NACK,
	I2C_overrun_error,
	I2C_bus_error,

}Status_code_t;

typedef enum{
	Disabled,
	Enabled
}Enabled_Disabled_t;

typedef enum{
	Clean_one_bit =			0x01, 	//to clean 1 bit
	Clear_two_bits=			0x03, 	//to clean 1 GPIOS
	Clear_three_bits = 		0x07, 	//to clena 3 bits
	Clear_four_bits = 		0x0F, 	//to clean 2 GPIOS

	Clear_five_bits = 		0x1F,
	Clear_six_bits =		0x3F,
	Clear_seven_bits =		0x7F,
	Clear_eight_bits =		0xFF, 	//to clean 4 GPIOS

	Clear_nine_bits =		0x1FF,
	Clear_ten_bits =		0x3FF,
	Clear_eleven_bits =		0x7FF,
	Clear_twelve_bits =		0xFFF,

	Clear_thirteen_bits =   0x1FFF,
	clear_fourteen_bits = 	0x3FFF, //to clean 7 GPIOS
	Clear_fiveteen_bits =	0x7FFF,
	Clear_sixteen_bits =	0xFFFF 	//to cslean 8 GPIOS
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
//	USART2_ADDRESS = 		0x40004400,
//	I2C1_ADDRESS = 			0x40005400,
//	I2C2_ADDRESS = 			0x40005800,
//	I2C3_ADDRESS = 			0x40005C00,
	PWR_ADDRESS = 			0x40007000, //APB1

//	TIM1_ADDRESS = 			0x40010000, //APB2 BUS
//	USART1_ADDRESS = 		0x40011000,
//	USART6_ADDRESS = 		0x40011400,
	ADC1_ADDRESS = 			0x40012000,
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
	USART2_ADDRESS = 		0x40004400,
	USART1_ADDRESS = 		0x40011000,
	USART6_ADDRESS = 		0x40011400,

}USARTMapAddr_t;

typedef enum{
	I2C1_ADDRESS = 			0x40005400,
	I2C2_ADDRESS = 			0x40005800,
	I2C3_ADDRESS = 			0x40005C00,

}I2CMapAddr_t;

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
	RCC_SYSCFG_ENABLE = (1u<<14)
}RCC_SYSCFG_CLOCK_EN_t;


typedef enum{
	OFFSET_PORTS =	0x00,
}peripherial_offset_t;



typedef enum{
	SYSCFG_MEMRMP = 	0x00,
	SYSCFG_PMC = 		0x04,
	SYSCFG_EXTICR1 = 	0x08,
	SYSCFG_EXTICR2 = 	0x0C,
	SYSCFG_EXTICR3 = 	0x10,
	SYSCFG_EXTICR4 = 	0x14,
	SYSCFG_CMPCR = 		0x20

}SYSCFG_register_offset_t;

//typedef enum{
//	 SYS_EXTI0 = (0),
//	 SYS_EXTI1 = (4),
//	 SYS_EXTI2 = (8),
//	 SYS_EXTI3 = (12)
//
//}SYSCFG_EXTICR1_t;
//
//typedef enum{
//	 SYS_EXTI4 = (0),
//	 SYS_EXTI5 = (4),
//	 SYS_EXTI6 = (8),
//	 SYS_EXTI7 = (12)
//
//}SYSCFG_EXTICR2_t;
//
//typedef enum{
//	 SYS_EXTI8 = 	(0),
//	 SYS_EXTI9 = 	(4),
//	 SYS_EXTI10 = 	(8),
//	 SYS_EXTI11 = 	(12)
//
//}SYSCFG_EXTICR3_t;
//
//typedef enum{
//	 SYS_EXTI12 = (0),
//	 SYS_EXTI13 = (4),
//	 SYS_EXTI14 = (8),
//	 SYS_EXTI15 = (12)
//
//}SYSCFG_EXTICR4_t;

typedef enum{
	 SYS_MEM_MODE= 0,
}SYSCFG_MEMRMP_t;

typedef enum{
	 SYS_ADC1DC2= (1u<<16),
}SYSCFG_PMC_t;

typedef enum{
	 SYS_CMP_PD= (1u<<0),
	 SYS_READY = (1u<<8),
}SYSCFG_CMPCR_t;


typedef enum{
	EXTI_IMR = 		0x00,
	EXTI_EMR = 		0x04,
	EXTI_RTSR = 	0x08,
	EXTI_FTSR = 	0x0C,
	EXTI_SWIER = 	0x10,
	EXTI_PR = 		0x14,

}EXTI_registers_offset_t;


typedef enum{
	RCC_LSION = 	(1U<<0U),
	RCC_LSIRDY = 	(1U<<1U),
	RCC_RMVF = 		(1U<<24U), //used to clear all flags
	RCC_BORRSTF = 	(1U<<25U),
	RCC_PINRSTF = 	(1U<<26U),
	RCC_PORRSTF = 	(1U<<27U),
	RCC_SFTRSTF = 	(1U<<28U),
	RCC_IWDGRSTF = 	(1U<<29U),
	RCC_WWDGRSTF = 	(1U<<30U),
	RCC_LPWRRSTF = 	(1U<<31U),

}RCC_CSR_t;


typedef enum{
	DBGMCU_CR 		= 0xE0042004,
	DBGMCU_APB1_FZ 	= 0xE0042008,
	DBGMCU_APB2_FZ 	= 0xE004200C,

}Debug_MCU_addr_t;


typedef enum{
	DBG_SLEEP   = 	(1U<<0U),
	DBG_STOP    =	(1U<<1U),
	DBG_STANDBY = 	(1U<<2U),
	TRACE_IOEN  = 	(1U<<5U),
	TRACE_MODE  = 	(6U),

}Debug_CR_t;

typedef enum{
	DBG_TIM2_STOP  			= 	(1U<<0U),
	DBG_TIM3_STOP  			=	(1U<<1U),
	DBG_TIM4_STOP 			= 	(1U<<2U),
	DBG_TIM5_STOP  			= 	(1U<<3U),
	DBG_RTC_STOP  			= 	(1U<<10U),
	DBG_WWDG_STOP   		= 	(1U<<11U),
	DBG_IWDG_STOP    		=	(1U<<12U),
	DBG_I2C1_SMBUS_TIMEOUT  = 	(1U<<21U),
	DBG_I2C2_SMBUS_TIMEOUT  = 	(1U<<22U),
	DBG_I2C3_SMBUS_TIMEOUT  = 	(1U<<23U),

}DBGMCU_APB1_FZ_t;

typedef enum{
	DBG_TIM1_STOP  	= 	(1U<<0U),
	DBG_TIM9_STOP  	=	(1U<<16U),
	DBG_TIM10_STOP 	= 	(1U<<17U),
	DBG_TIM11_STOP 	= 	(1U<<18U),

}DBGMCU_APB2_FZ_t;





Status_code_t ClockEnable(Set_Port_t Port_define, Enabled_Disabled_t Intention);
Status_code_t Delay(uint32_t microseconds);
void resetDelay(TimerMapAddr_t Timer);
Status_code_t Peripherial_delay(uint16_t miliseconds);
void Init_Board(void);
void ftoa(float decimalData, uint8_t* cadena, uint8_t decimales);

bool RCC_get_reset_status_flag(RCC_CSR_t flag);
void RCC_clear_reset_flags(void);

void SYS_ClockEnable(Enabled_Disabled_t Intention);


//DBG functions
bool system_is_debugging(void);
void frezze_WWDG(void);
void frezze_IWDG(void);


#endif /* SYSTEM_SETTINGS_H_ */
