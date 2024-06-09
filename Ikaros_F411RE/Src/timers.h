/*
 * timers.h
 *
 *  Created on: Jun 6, 2024
 *      Author: Sebastian G.M.
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "system_settings.h"
#include "gpios.h"


#define TIM3_HANDLER TIM3_IRQHandler
#define TIM4_HANDLER TIM4_IRQHandler
#define TIM5_HANDLER TIM5_IRQHandler

/*Value in microseconds Equals to 2^32 bit count*/
#define MAX_TIME_TIM5_AND_TIM2	(uint32_t)(4294967290)

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

typedef enum{
	 TIM2_TO_TIM5_CEN= 		(1u<<0),
	 TIM2_TO_TIM5_UDIS=		(1u<<1),
	 TIM2_TO_TIM5_URS=		(1u<<2),
	 TIM2_TO_TIM5_OPM=		(1u<<3),
	 TIM2_TO_TIM5_DIR=		(1u<<4)
 //faltan los demas bits //pendiente
}TIM2_TIM5_CR1_t;

typedef enum{
	 TIM2_TO_TIM5_UIF= 		(1u<<0),
	 TIM2_TO_TIM5_CC1F=		(1u<<1),
	 TIM2_TO_TIM5_CC2IF=	(1u<<2),
	 TIM2_TO_TIM5_CC3IF=	(1u<<3),
	 TIM2_TO_TIM5_CC4IF=	(1u<<4),
	 TIM2_TO_TIM5_TIF=		(1u<<6),
	 TIM2_TO_TIM5_CC10F=	(1u<<9),
	 TIM2_TO_TIM5_CC20F=	(1u<<10),
	 TIM2_TO_TIM5_CC30F=	(1u<<11),
	 TIM2_TO_TIM5_CC40F=	(1u<<12),

}TIM2_TIM5_SR_status_t;

typedef enum{
	 TIM2_TO_TIM5_UIE= 		(1u<<0),
	 TIM2_TO_TIM5_CC1IE=	(1u<<1),
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


void volatile TIMER_WaitFlag(TimerMapAddr_t TIMER_addr);
void volatile TIMER_Clock(Enabled_Disabled_t state, timers_enb_t Timer);
void volatile TIMER_cleanCountFlag(TimerMapAddr_t TIMER_addr);

Status_code_t TIM5_Init(uint32_t microseconds);
void TIM5_Deinit(void);
void TIM5_HANDLER(void);

#endif /* TIMERS_H_ */
