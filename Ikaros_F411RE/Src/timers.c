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

void volatile TIMER_WaitFlag(TimerMapAddr_t TIMER_addr){
	uint32_t volatile *TIM_REG_SR = (uint32_t volatile*)(TIMER_addr + TIMx_SR);
	while(!(TIM2_TO_TIM5_UIF & *TIM_REG_SR)){}
	TIMER_cleanCountFlag(TIMER_addr);

}


/*///////////////////TIMER 5 HANDLER AND CONTROL FUNCTIONS///////////////////////////////////*/

bool toggle_led = false;

void TIM5_HANDLER(void){
	TIMER_cleanCountFlag(TIM5_ADDRESS);
	/*Develop all the code to be executed in a second thread down here*/
//	toggle_led = !toggle_led;
//	GPIO_DigitalWrite(Port_A, Pin_5, toggle_led);

}

Status_code_t TIM5_Init(uint32_t microseconds){

	if(microseconds > MAX_TIME_TIM5_AND_TIM2){
		return TimeSetNotSuported;
	}
	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM5_ADDRESS + TIMx_PSC);
	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM5_ADDRESS + TIMx_ARR);
	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM5_ADDRESS + TIMx_CNT);
	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM5_ADDRESS + TIMx_CR1);

	uint32_t volatile *TIM_REG_DIER = (uint32_t volatile*)(TIM5_ADDRESS + TIMx_DIER);
	TIMER_Clock(Enabled,TIMER_5);

	*TIM_REG_CR1 &= ~TIM2_TO_TIM5_CEN; // Disable timer before configuration
	*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY-1);
	*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,microseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	*TIM_REG_CR1 |= TIM2_TO_TIM5_CEN;


	*TIM_REG_DIER |= TIM2_TO_TIM5_UIE;

	NVIC_EnableIRQ(TIM5_IRQn);
	return Success;

}

void TIM5_Deinit(void){

	NVIC_DisableIRQ(TIM5_IRQn);
	TIMER_Clock(Disabled,TIMER_5);

}



/*////////////////////////////////// TIMER 3 HANDLER AND CONTROL FUNCTIONS/////////////////////////////////////*/
void TIM3_HANDLER(void){
	TIMER_cleanCountFlag(TIM3_ADDRESS);
	/*Develop all the code to be executed in a second thread down here*/
	toggle_led = !toggle_led;
	GPIO_DigitalWrite(Port_A, Pin_5, toggle_led);

}

Status_code_t TIM3_Init(uint16_t milliseconds){

	if(milliseconds > MAX_TIME_TIM3_TIM4){
		return TimeSetNotSuported;
	}
	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_PSC);
	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_ARR);
	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_CNT);
	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_CR1);

	uint32_t volatile *TIM_REG_DIER = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_DIER);
	TIMER_Clock(Enabled,TIMER_3);

	*TIM_REG_CR1 &= ~TIM2_TO_TIM5_CEN; // Disable timer before configuration
	*TIM_REG_PSC = (PSC_TO_MILLISEC_DELAY-1);
	*TIM_REG_ARR = (MILLSEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MILLISEC_DELAY,milliseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	*TIM_REG_CR1 |= TIM2_TO_TIM5_CEN;


	*TIM_REG_DIER |= TIM2_TO_TIM5_UIE;

	NVIC_EnableIRQ(TIM3_IRQn);
	return Success;

}

void TIM3_Deinit(void){

	NVIC_DisableIRQ(TIM3_IRQn);
	TIMER_Clock(Disabled,TIMER_3);

}


/*////////////////////// TIMER 4 HANDLER AND CONTROL FUNCTIONS/////////////////////////////////*/

void TIM4_HANDLER(void){
	TIMER_cleanCountFlag(TIM4_ADDRESS);
	/*Develop all the code to be executed in a second thread down here*/
	toggle_led = !toggle_led;
	GPIO_DigitalWrite(Port_A, Pin_5, toggle_led);

}

Status_code_t TIM4_Init(uint16_t milliseconds){

	if(milliseconds > MAX_TIME_TIM3_TIM4){
		return TimeSetNotSuported;
	}
	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM4_ADDRESS + TIMx_PSC);
	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM4_ADDRESS + TIMx_ARR);
	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM4_ADDRESS + TIMx_CNT);
	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM4_ADDRESS + TIMx_CR1);

	uint32_t volatile *TIM_REG_DIER = (uint32_t volatile*)(TIM4_ADDRESS + TIMx_DIER);
	TIMER_Clock(Enabled,TIMER_4);

	*TIM_REG_CR1 &= ~TIM2_TO_TIM5_CEN; // Disable timer before configuration
	*TIM_REG_PSC = (PSC_TO_MILLISEC_DELAY-1);
	*TIM_REG_ARR = (MILLSEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MILLISEC_DELAY,milliseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	*TIM_REG_CR1 |= TIM2_TO_TIM5_CEN;


	*TIM_REG_DIER |= TIM2_TO_TIM5_UIE;

	NVIC_EnableIRQ(TIM4_IRQn);
	return Success;

}

void TIM4_Deinit(void){

	NVIC_DisableIRQ(TIM4_IRQn);
	TIMER_Clock(Disabled,TIMER_4);

}
