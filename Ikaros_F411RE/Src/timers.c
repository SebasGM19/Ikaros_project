/*
 * timers.c
 *
 *  Created on: Jun 6, 2024
 *      Author: Sebastian G.M.
 */

#include "timers.h"

void volatile TIMER_cleanCountFlag(TimerMapAddr_t TIMER_addr){
	uint32_t volatile *TIM_REG_SR = (uint32_t volatile*)(TIMER_addr + TIMx_SR);

	*TIM_REG_SR &= ~(TIM2_TO_TIM5_UIF);

}

void volatile TIMER_Clock( Enabled_Disabled_t state, timers_enb_t Timer){
	RCC_offset_t volatile RccOffset = RCC_OFFSET_APB1ENR;

	if(Timer >= 10){
		Timer-=10; //it means that is the timers from the APB2
		RccOffset = RCC_OFFSET_APB2ENR;
	}

	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(RCC_ADDRESS + RccOffset);
    if(state) {
        *pClockControlReg |= (Enabled << Timer);
    } else {
        *pClockControlReg &= ~(Enabled << Timer);
    }
}

void volatile TIMER_WaitFlag(TimerMapAddr_t TIMER_addr){ //use of SR and UIF?
	uint32_t volatile *TIM_REG_SR = (uint32_t volatile*)(TIMER_addr + TIMx_SR);
	while(!(TIM2_TO_TIM5_UIF & *TIM_REG_SR)){}
	TIMER_cleanCountFlag(TIMER_addr);

}



void TIM3_HANDLER(void){
	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_CNT);
	TIMER_cleanCountFlag(TIM3_ADDRESS);

	*TIM_REG_CNT = 0;

}

void TIM3_Deinit(void){

	TIMER_Clock(Disabled,TIMER_3);
	NVIC_DisableIRQ(TIM3_IRQn);

}

void TIM3_Init(uint32_t microseconds){ //init function to execute handler after the count happend

	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_PSC);
	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_ARR);
	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_CNT);
	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_CR1);

	uint32_t volatile *TIM_REG_DIER = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_DIER);
	TIMER_Clock(Enabled,TIMER_3);

	*TIM_REG_CR1 &= ~TIM2_TO_TIM5_CEN; // Disable timer before configuration
	*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY-1);
	*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,microseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

//	for(uint8_t i =0;i<100;i++){
//		__NOP();
//	}

	*TIM_REG_CR1 |= TIM2_TO_TIM5_CEN;


	*TIM_REG_DIER |= TIM2_TO_TIM5_UIE;

	NVIC_EnableIRQ(TIM3_IRQn);

}
