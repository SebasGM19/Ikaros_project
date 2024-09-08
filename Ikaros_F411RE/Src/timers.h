/*
 * timers.h
 *
 *  Created on: Jun 6, 2024
 *      Author: Sebastian G.M.
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "system_settings.h"




//#define TIM1  usado para delay periphericos
//#define TIM2  usado para delay  general

#define TIM3_HANDLER TIM3_IRQHandler
#define TIM4_HANDLER TIM4_IRQHandler
#define TIM5_HANDLER TIM5_IRQHandler

#define TIM9_HANDLER TIM1_BRK_TIM9_IRQHandler //si tiene handler como el TIM3, 4 y 5
#define TIM10_HANDLER TIM1_UP_TIM10_IRQHandler //si tiene handler como el TIM3, 4 y 5
//#define TIM11_HANDLER TIM11_IRQHandler //usar como delay para protocolos y timeout

/*Value in microseconds Equals to 2^32 bit count*/
#define MAX_TIME_TIM5_AND_TIM2	(uint32_t)(4294967290)
#define MAX_TIME_TIM3_TIM4		(uint16_t)(65530)
#define MAX_TIME_TIM1			(uint16_t)(65530)
#define MAX_TIME_TIM9_TO_TIM11	(uint16_t)(65530)

#define MAX_DATA_BUFF			(100)



typedef enum{
	TIMER_2, //APB1
	TIMER_3,
	TIMER_4,
	TIMER_5,

	TIMER_1 = 	10, //APB2 substrac 10
	TIMER_9 = 	26,
	TIMER_10 = 	27,
	TIMER_11 = 	28

}timers_enb_t;

typedef enum{
	TIMx_CR1 =		0x00,
	TIMx_CR2 =		0x04,
	TIMx_SMCR =		0x08,
	TIMx_DIER = 	0x0C, //important
	TIMx_SR = 		0x10,
	TIMx_EGR = 		0x14,
	TIMx_CCMR1 = 	0x18,
	TIMx_CCMR2 = 	0x1C,
	TIMx_CCER = 	0x20,
	TIMx_CNT = 		0x24, //important
	TIMx_PSC = 		0x28, //important
	TIMx_ARR = 		0x2C, //important
	TIMx_CCR1 = 	0x34,
	TIMx_CCR2 = 	0x38,
	TIMx_CCR3 =		0x3C,
	TIMx_CCR4 = 	0x40,
	TIMx_DCR = 		0x48,
	TIMx_DMAR = 	0x4C,
	TIM2_TIM5_OR = 	0x50

}TIMx_register_offset_t;

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

/*_____________________________________TIMx_CR1 BITS____________________________________________*/

typedef enum{
	 TIMx_CEN= 					(1u<<0),
	 TIMx_UDIS=					(1u<<1),
	 TIMx_URS=					(1u<<2),
	 TIMx_OPM=					(1u<<3),
	 TIM1_TIM2_TO_TIM5_DIR=		(1u<<4),
	 TIM1_TIM2_TO_TIM5_CMS=		(5),
	 TIMx_ARPE=					(1u<<7),
	 TIMx_CKD=					(8) //2 bits set

}global_timers_CR1_t;

/*_____________________________________TIMx_SR BITS____________________________________________*/

typedef enum{
	 TIM1_CC1F=		(1u<<1),
	 TIM1_CC2IF=	(1u<<2),
	 TIM1_CC3IF=	(1u<<3),
	 TIM1_CC4IF=	(1u<<4),
	 TIM1_COMIF=	(1u<<5),
	 TIM1_TIF=		(1u<<6),
	 TIM1_BIF=		(1u<<7),
	 TIM1_CC10F=	(1u<<9),
	 TIM1_CC20F=	(1u<<10),
	 TIM1_CC30F=	(1u<<11),
	 TIM1_CC40F=	(1u<<12)

}TIM1_SR_status_t;

typedef enum{
	 TIM2_TO_TIM5_CC1F=		(1u<<1),
	 TIM2_TO_TIM5_CC2IF=	(1u<<2),
	 TIM2_TO_TIM5_CC3IF=	(1u<<3),
	 TIM2_TO_TIM5_CC4IF=	(1u<<4),
	 TIM2_TO_TIM5_TIF=		(1u<<6),
	 TIM2_TO_TIM5_CC10F=	(1u<<9),
	 TIM2_TO_TIM5_CC20F=	(1u<<10),
	 TIM2_TO_TIM5_CC30F=	(1u<<11),
	 TIM2_TO_TIM5_CC40F=	(1u<<12)

}TIM2_TIM5_SR_status_t;


typedef enum{
	 TIM9_CC1F=		(1u<<1),
	 TIM9_CC2IF=	(1u<<2),
	 TIM9_TIF=		(1u<<6),
	 TIM9_CC10F=	(1u<<9),
	 TIM9_CC20F=	(1u<<10)

}TIM9_SR_status_t;

typedef enum{
	 TIM10_TIM11_CC1IF=		(1u<<1),
	 TIM10_TIM11_CC10F=		(1u<<9)

}TIM10_TIM11_SR_status_t;


typedef enum{
	TIMx_UIF = (1u<<0),
}Global_timers_SR_t;


typedef enum{
	TIM1_TO_TIM5_CC1S_and_CC3S  = 	(0),
	TIM1_TO_TIM5_OC1FE_and_OC3FE = 	(1u<<2),
	TIM1_TO_TIM5_OC1PE_and_OC3PE = 	(1u<<3),
	TIM1_TO_TIM5_OC1M_and_OC3M  = 	(4),
	TIM1_TO_TIM5_OC1CE_and_OC3CE = 	(1u<<7),
	TIM1_TO_TIM5_CC2S_and_CC4S  = 	(8),
	TIM1_TO_TIM5_OC2FE_and_OC4FE = 	(1u<<10),
	TIM1_TO_TIM5_OC2PE_and_OC4PE = 	(1u<<11),
	TIM1_TO_TIM5_OC2M_and_OC4M = 	(12),
	TIM1_TO_TIM5_OC2CE_and_OC4CE = 	(1u<<15)
}TIM1_to_TIM5_CCMR1_CCMR2_t;

typedef enum{
	TIM9_CC1S  = (0),
	TIM9_OC1FE = (1u<<2),
	TIM9_OC1PE = (1u<<3),
	TIM9_OC1M  = (4),
	TIM9_CC2S  = (8),
	TIM9_OC2FE = (1u<<10),
	TIM9_OC2PE = (1u<<11),
	TIM9_OC2M  = (12),
}TIM9_CCMR1_t;

typedef enum{
	TIM10_TO_TIM11_CC1S  = (0),
	TIM10_TO_TIM11_OC1FE = (1u<<2),
	TIM10_TO_TIM11_OC1PE = (1u<<3),
	TIM10_TO_TIM11_OC1M  = (4),

}TIM10_TO_TIM11_CCMR1_t;



typedef enum{
	TIM1_CC1E = (1u<<0),
	TIM1_CC1P = (1u<<1),
	TIM1_CC1NE = (1u<<2),
	TIM1_CC1NP = (1u<<3),

	TIM1_CC2E = (1u<<4),
	TIM1_CC2P = (1u<<5),
	TIM1_CC2NE = (1u<<6),
	TIM1_CC2NP = (1u<<7),

	TIM1_CC3E = (1u<<8),
	TIM1_CC3P = (1u<<9),
	TIM1_CC3NE = (1u<<10),
	TIM1_CC3NP = (1u<<11),

	TIM1_CC4E = (1u<<12),
	TIM1_CC4P = (1u<<13),
}TIM1_CCER_t;

typedef enum{
	TIM2_TO_TIM5_CC1E = (1u<<0),
	TIM2_TO_TIM5_CC1P = (1u<<1),
	TIM2_TO_TIM5_CC1NP = (1u<<3),

	TIM2_TO_TIM5_CC2E = (1u<<4),
	TIM2_TO_TIM5_CC2P = (1u<<5),
	TIM2_TO_TIM5_CC2NP = (1u<<7),

	TIM2_TO_TIM5_CC3E = (1u<<8),
	TIM2_TO_TIM5_CC3P = (1u<<9),
	TIM2_TO_TIM5_CC3NP = (1u<<11),

	TIM2_TO_TIM5_CC4E = (1u<<12),
	TIM2_TO_TIM5_CC4P = (1u<<13),
	TIM2_TO_TIM5_CC4NP = (1u<<15),

}TIM2_to_TIM5_CCER_t;

typedef enum{
	TIM9_CC1E = (1u<<0),
	TIM9_CC1P = (1u<<1),
	TIM9_CC1NP = (1u<<3),

	TIM9_CC2E = (1u<<4),
	TIM9_CC2P = (1u<<5),
	TIM9_CC2NP = (1u<<7),

}TIM9_CCER_t;

typedef enum{
	TIM10_TO_TIM11_CC1E = (1u<<0),
	TIM10_TO_TIM11_CC1P = (1u<<1),
	TIM10_TO_TIM11_CC1NP = (1u<<3),
}TIM10_to_TIM11_CCER_t;

/*_____________________________________TIMx_DIER BITS____________________________________________*/

typedef enum{
	 TIM1_CC2IE=	(1u<<2),
	 TIM1_CC3IE=	(1u<<3),
	 TIM1_CC4IE=	(1u<<4),
	 TIM1_COMIE=	(1u<<5),
	 TIM1_TIE=		(1u<<6),
	 TIM1_BIE=		(1u<<7),
	 TIM1_UDE=		(1u<<8),
	 TIM1_CC1DE=	(1u<<9),
	 TIM1_CC2DE=	(1u<<10),
	 TIM1_CC3DE=	(1u<<11),
	 TIM1_CC4DE=	(1u<<12),
	 TIM1_COMDE=	(1u<<13),
	 TIM1_TDE=		(1u<<14)
}TIM1_DIER_t;

typedef enum{
	 TIM2_TO_TIM5_CC2IE=	(1u<<2),
	 TIM2_TO_TIM5_CC3IE=	(1u<<3),
	 TIM2_TO_TIM5_CC4IE=	(1u<<4),
	 TIM2_TO_TIM5_TIE=		(1u<<6),
	 TIM2_TO_TIM5_UDE=		(1u<<8),
	 TIM2_TO_TIM5_CC1DE=	(1u<<9),
	 TIM2_TO_TIM5_CC2DE=	(1u<<10),
	 TIM2_TO_TIM5_CC3DE=	(1u<<11),
	 TIM2_TO_TIM5_CC4DE=	(1u<<12),
	 TIM2_TO_TIM5_TDE=		(1u<<14)
}TIM2_TIM5_DIER_t;

typedef enum{
	 TIM9_CC2IE=	(1u<<2),
	 TIM9_TIE=		(1u<<6),
}TIM9_DIER_t;

typedef enum{
	 TIMx_UIE= 		(1u<<0),
	 TIMx_CC1IE=	(1u<<1),
}global_timers_DIER_t;



typedef enum{
	Frozen,
	active_level,
	inactive_level,
	toggle,
	force_inactive_level,
	force_active_level,
	PWM_mode_1,
	PWM_mode_2,

}TIM_output_compare_mode_t;


typedef enum{
	Clean_pwm_channel_1_and_3 =	(0x7B),
	Clean_pwm_channel_2_and_4 = (0x7B << 8),

}Clean_PWM_channel_t;


typedef enum{//channel and PIN

	TIM3_CH3= 0,//PORTB
	TIM3_CH4= 1,//PORTB
	TIM3_CH1= 4,//PORTB
	TIM3_CH2= 5,//PORTB

}TIM3_PWM_channel_select_t;

typedef enum{//channel and PIN

	TIM4_CH1= 6,//PORTB
	TIM4_CH2= 7,//PORTB
	TIM4_CH3= 8,//PORTB
	TIM4_CH4= 9,//PORTB

}TIM4_PWM_channel_select_t;

typedef enum{//channel and PIN

	TIM5_CH1= 0,//PORT A
	TIM5_CH2= 1,//PORT A

}TIM5_PWM_channel_select_t;


volatile void set_cuenta(uint32_t cuenta_lim);

void volatile TIMER_WaitFlag(TimerMapAddr_t TIMER_addr);
void volatile TIMER_Clock(Enabled_Disabled_t state, timers_enb_t Timer);
void volatile TIMER_cleanCountFlag(TimerMapAddr_t TIMER_addr);

void TIM3_HANDLER(void);
Status_code_t TIM3_Init(uint16_t milliseconds);
void TIM3_Start(void);
void TIM3_Stop(void);
void TIM3_Deinit(void);

void TIM4_HANDLER(void);
Status_code_t TIM4_Init(uint16_t milliseconds);
void TIM4_Start(void);
void TIM4_Stop(void);
void TIM4_Deinit(void);

void TIM5_HANDLER(void);
Status_code_t TIM5_Init(uint32_t microseconds);
void TIM5_Start(void);
void TIM5_Stop(void);
void TIM5_Deinit(void);


Status_code_t TIM3_PWM_Init(TIM3_PWM_channel_select_t Channel);
Status_code_t TIM3_PWM_start_channel(uint32_t miliseconds_duty, TIM3_PWM_channel_select_t Channel);
Status_code_t TIM3_PWM_start_channel_duty_porcent(uint8_t porcent_duty, TIM3_PWM_channel_select_t Channel);
Status_code_t TIM3_PWM_stop_channel(TIM3_PWM_channel_select_t Channel);
Status_code_t TIM3_PWM_Deinit(void);



#endif /* TIMERS_H_ */
