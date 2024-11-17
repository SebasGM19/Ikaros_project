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

#define TIM9_HANDLER TIM1_BRK_TIM9_IRQHandler
#define TIM10_HANDLER TIM1_UP_TIM10_IRQHandler
#define TIM11_HANDLER TIM1_TRG_COM_TIM11_IRQHandler //usar como delay para protocolos y timeout

/*Value in microseconds Equals to 2^32 bit count*/
#define MAX_TIME_TIM5_AND_TIM2	(uint32_t)(4294967290U)
#define MAX_TIME_TIM3_TIM4		(uint16_t)(65530U)
#define MAX_TIME_TIM1			(uint16_t)(65530U)
#define MAX_TIME_TIM9_TO_TIM11	(uint16_t)(65530U)

#define MAX_DATA_BUFF			(100U)



typedef enum{
	TIMER_2  = 0U, //APB1
	TIMER_3  = 1U,
	TIMER_4  = 2U,
	TIMER_5  = 3U,

	TIMER_1  = 10U, //APB2 substrac 10
	TIMER_9  = 26U,
	TIMER_10 = 27U,
	TIMER_11 = 28U

}timers_enb_t;

typedef enum{
	TIMx_CR1 =		0x00U,
	TIMx_CR2 =		0x04U,
	TIMx_SMCR =		0x08U,
	TIMx_DIER = 	0x0CU, //important
	TIMx_SR = 		0x10U,
	TIMx_EGR = 		0x14U,
	TIMx_CCMR1 = 	0x18U,
	TIMx_CCMR2 = 	0x1CU,
	TIMx_CCER = 	0x20U,
	TIMx_CNT = 		0x24U, //important
	TIMx_PSC = 		0x28U, //important
	TIMx_ARR = 		0x2CU, //important
	TIMx_CCR1 = 	0x34U,
	TIMx_CCR2 = 	0x38U,
	TIMx_CCR3 =		0x3CU,
	TIMx_CCR4 = 	0x40U,
	TIMx_DCR = 		0x48U,
	TIMx_DMAR = 	0x4CU,
	TIM2_TIM5_OR = 	0x50U

}TIMx_register_offset_t;

typedef enum{
	TIMx_DIV_BY_1 = 	(1U<<0U),
	TIMx_DIV_BY_2 =		(1U<<1U),
	TIMx_DIV_BY_4=		(1U<<2U),
	TIMx_DIV_BY_8=		(1U<<3U),
	TIMx_DIV_BY_16=		(1U<<4U),
	TIMx_DIV_BY_32=		(1U<<5U),
	TIMx_DIV_BY_64=		(1U<<6U),
	TIMx_DIV_BY_128=	(1U<<7U),
	TIMx_DIV_BY_256=	(1U<<8U),
	TIMx_DIV_BY_512=	(1U<<9U)

}TIMx_preescaler_t;

/*_____________________________________TIMx_CR1 BITS____________________________________________*/

typedef enum{
	 TIMx_CEN= 					(1U<<0U),
	 TIMx_UDIS=					(1U<<1U),
	 TIMx_URS=					(1U<<2U),
	 TIMx_OPM=					(1U<<3U),
	 TIM1_TIM2_TO_TIM5_DIR=		(1U<<4U),
	 TIM1_TIM2_TO_TIM5_CMS=		(5U),
	 TIMx_ARPE=					(1U<<7U),
	 TIMx_CKD=					(8U) //2 bits set

}global_timers_CR1_t;

/*_____________________________________TIMx_SR BITS____________________________________________*/

typedef enum{
	 TIM1_CC1F=		(1U<<1U),
	 TIM1_CC2IF=	(1U<<2U),
	 TIM1_CC3IF=	(1U<<3U),
	 TIM1_CC4IF=	(1U<<4U),
	 TIM1_COMIF=	(1U<<5U),
	 TIM1_TIF=		(1U<<6U),
	 TIM1_BIF=		(1U<<7U),
	 TIM1_CC10F=	(1U<<9U),
	 TIM1_CC20F=	(1U<<10U),
	 TIM1_CC30F=	(1U<<11U),
	 TIM1_CC40F=	(1U<<12U)

}TIM1_SR_status_t;

typedef enum{
	 TIM2_TO_TIM5_CC1F=		(1U<<1U),
	 TIM2_TO_TIM5_CC2IF=	(1U<<2U),
	 TIM2_TO_TIM5_CC3IF=	(1U<<3U),
	 TIM2_TO_TIM5_CC4IF=	(1U<<4U),
	 TIM2_TO_TIM5_TIF=		(1U<<6U),
	 TIM2_TO_TIM5_CC10F=	(1U<<9U),
	 TIM2_TO_TIM5_CC20F=	(1U<<10U),
	 TIM2_TO_TIM5_CC30F=	(1U<<11U),
	 TIM2_TO_TIM5_CC40F=	(1U<<12U)

}TIM2_TIM5_SR_status_t;


typedef enum{
	 TIM9_CC1F=		(1U<<1U),
	 TIM9_CC2IF=	(1U<<2U),
	 TIM9_TIF=		(1U<<6U),
	 TIM9_CC10F=	(1U<<9U),
	 TIM9_CC20F=	(1U<<10U)

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


}TIMx_global_OCxM_t;


typedef enum{

	PWM_mode_1 = 6, //to use time on
	PWM_mode_2 = 7, //to set time off

}PWM_mode_OCxM_t;

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



typedef struct{
	TIM3_PWM_channel_select_t channel;
	uint8_t duty_cycle_percent;
	uint32_t frequency;

}TIM3_pwm_auto_parameters_t;


typedef struct{
	TIM3_PWM_channel_select_t channel;
	uint32_t Total_count_ARR;
	uint32_t duty_count;
	uint32_t prescaler;

}TIM3_pwm_custom_parameters_t;



typedef struct{
	TIM4_PWM_channel_select_t channel;
	uint8_t duty_cycle_percent;
	uint32_t frequency;

}TIM4_pwm_auto_parameters_t;


typedef struct{
	TIM4_PWM_channel_select_t channel;
	uint32_t Total_count_ARR;
	uint32_t duty_count;
	uint32_t prescaler;

}TIM4_pwm_custom_parameters_t;



typedef struct{
	TIM5_PWM_channel_select_t channel;
	uint8_t duty_cycle_percent;
	uint32_t frequency;

}TIM5_pwm_auto_parameters_t;


typedef struct{
	TIM5_PWM_channel_select_t channel;
	uint32_t Total_count_ARR;
	uint32_t duty_count;
	uint32_t prescaler;

}TIM5_pwm_custom_parameters_t;

volatile void set_cuenta(uint32_t cuenta_lim);

void TIMER_WaitFlag(TimerMapAddr_t TIMER_addr);
void TIMER_Clock(Enabled_Disabled_t state, timers_enb_t Timer);
void TIMER_cleanCountFlag(TimerMapAddr_t TIMER_addr);

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


void TIM11_HANDLER(void);
Status_code_t TIM11_Init(uint32_t milliseconds);
void TIM11_Start(void);
void TIM11_Stop(void);
void TIM11_Deinit(void);
bool TIM11_GET_interrupt_flag_status(void);

void TIM11_clear_interrupt_flag(void);




void TIM3_set_global_PWM_ARR(uint32_t new_arr_value);

Status_code_t TIM3_PWM_Init(TIM3_PWM_channel_select_t channel,PWM_mode_OCxM_t mode);
Status_code_t TIM3_PWM_start_custom_channel(TIM3_pwm_custom_parameters_t const PWM_Custom);
Status_code_t TIM3_PWM_start_channel(TIM3_pwm_auto_parameters_t const PWM);
Status_code_t TIM3_PWM_stop_channel(TIM3_PWM_channel_select_t channel);
Status_code_t TIM3_PWM_Deinit(void);


void TIM4_set_global_PWM_ARR(uint32_t new_arr_value);

Status_code_t TIM4_PWM_Init(TIM4_PWM_channel_select_t channel,PWM_mode_OCxM_t mode);
Status_code_t TIM4_PWM_start_custom_channel(TIM4_pwm_custom_parameters_t const PWM_Custom);
Status_code_t TIM4_PWM_start_channel(TIM4_pwm_auto_parameters_t const PWM);
Status_code_t TIM4_PWM_stop_channel(TIM4_PWM_channel_select_t channel);
Status_code_t TIM4_PWM_Deinit(void);


//void TIM5_set_global_PWM_ARR(uint32_t new_arr_value);
//
//Status_code_t TIM5_PWM_Init(TIM5_PWM_channel_select_t channel,PWM_mode_OCxM_t mode);
//Status_code_t TIM5_PWM_start_custom_channel(TIM5_pwm_custom_parameters_t const PWM_Custom);
//Status_code_t TIM5_PWM_start_channel(TIM5_pwm_auto_parameters_t const PWM);
//Status_code_t TIM5_PWM_stop_channel(TIM5_PWM_channel_select_t channel);
//Status_code_t TIM5_PWM_Deinit(void);




#endif /* TIMERS_H_ */
