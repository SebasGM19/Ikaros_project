/*
 * system_settings.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Sebastian G.M.
 *
 */

#include "system_settings.h"
#include "timers.h"


Status_code_t ClockEnable(Set_Port_t Port_define, Enabled_Disabled_t Intention){
	Status_code_t status = Success;
	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(RCC_ADDRESS + RCC_OFFSET_AHB1ENR);

	if(Intention){
		switch(Port_define){
			case Port_A:
				*pClockControlReg |= (Enabled<<Clk_PortA);
				status = Success;
				break;
			case Port_B:
				*pClockControlReg |= (Enabled<<Clk_PortB);
				status = Success;
				break;
			case Port_C:
				*pClockControlReg |= (Enabled<<Clk_PortC);
				status = Success;
				break;
			case Port_D:
				*pClockControlReg |= (Enabled<<Clk_PortD);
				status = Success;
				break;
			default:
				status = ClockNotSupported;
				break;

		}

	}else if (Intention == Disabled){

		switch(Port_define){
			case Port_A:
				*pClockControlReg &= ~(Enabled<<Clk_PortA);
				status = Success;
				break;
			case Port_B:
				*pClockControlReg &= ~(Enabled<<Clk_PortB);
				status = Success;
				break;
			case Port_C:
				*pClockControlReg &= ~(Enabled<<Clk_PortC);
				status = Success;
				break;
			case Port_D:
				*pClockControlReg &= ~(Enabled<<Clk_PortD);
				status = Success;
				break;
			default:
				status = ClockNotSupported;
				break;
		}
	}else{
		status = WrongParameter;
	}

	return status;

}

void resetDelay(TimerMapAddr_t Timer){

	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(Timer + TIMx_CR1);
	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(Timer + TIMx_PSC);
	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(Timer + TIMx_ARR);

	if(Timer == TIM2_ADDRESS){
		TIMER_Clock(Enabled, TIMER_2);
		*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY);
		*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,2) - 1);
		*TIM_REG_CR1 |= TIMx_CEN;
		TIMER_Clock(Disabled, TIMER_2);

	}else if(Timer == TIM1_ADDRESS){
		TIMER_Clock(Enabled, TIMER_1);
		*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
		*TIM_REG_PSC = (PSC_TO_MILLISEC_DELAY-1);
		*TIM_REG_ARR = (MILLSEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MILLISEC_DELAY,2) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
		*TIM_REG_CR1 |= TIMx_CEN;
		TIMER_Clock(Disabled, TIMER_1);
	}else{

	}

}

Status_code_t Delay(uint32_t microseconds){

	if(microseconds < 50 || microseconds > MAX_TIME_TIM5_AND_TIM2){
		return DelayTimeNotSupported;
	}
		uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_PSC);
		uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_ARR);
		uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CNT);
		uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CR1);

		resetDelay(TIM2_ADDRESS);
		TIMER_Clock(Enabled,TIMER_2);

		*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration


		*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY-1);
		*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,microseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
		TIMER_cleanCountFlag(TIM2_ADDRESS);

		*TIM_REG_CNT = 0;
		*TIM_REG_CR1 |= TIMx_CEN;
		TIMER_WaitFlag(TIM2_ADDRESS);

		TIMER_Clock(Disabled,TIMER_2);

		return Success;

}

Status_code_t Peripherial_delay(uint16_t miliseconds){

	if(miliseconds < 2 || miliseconds > MAX_TIME_TIM1){
		return DelayTimeNotSupported;
	}
		uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM1_ADDRESS + TIMx_PSC);
		uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM1_ADDRESS + TIMx_ARR);
		uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM1_ADDRESS + TIMx_CNT);
		uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM1_ADDRESS + TIMx_CR1);

		resetDelay(TIM1_ADDRESS);
		TIMER_Clock(Enabled,TIMER_1);

		*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration

		*TIM_REG_PSC = (PSC_TO_MILLISEC_DELAY-1);
		*TIM_REG_ARR = (MILLSEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MILLISEC_DELAY,miliseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
		TIMER_cleanCountFlag(TIM1_ADDRESS);

		*TIM_REG_CNT = 0;
		*TIM_REG_CR1 |= TIMx_CEN;
		TIMER_WaitFlag(TIM1_ADDRESS);

		TIMER_Clock(Disabled,TIMER_1);

		return Success;

}



