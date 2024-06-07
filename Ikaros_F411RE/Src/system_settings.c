/*
 * system_settings.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Sebastian G.M.
 *
 */

#include "system_settings.h"
#include "timers.h"

void InitSystem(void){
	Delay(100);

}

Status_code_t ClockEnable(Set_Port_t Port_define, Enabled_Disabled_t Intention){
	Status_code_t status=Success;
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

Status_code_t Delay(uint32_t microseconds){

	if(microseconds < 100){
		return DelayTimeNotSupported;
	}
		uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_PSC);
		uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_ARR);
		uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CNT);
		uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CR1);

		TIMER_Clock(Enabled,TIMER_2);

		*TIM_REG_CR1 &= ~TIM2_TO_TIM5_CEN; // Disable timer before configuration
		*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY-1);
		*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,microseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
		TIMER_cleanCountFlag(TIM2_ADDRESS);


		*TIM_REG_CNT = 0;

//		for(uint8_t i =0;i<100;i++){
//			__NOP();
//		}
		*TIM_REG_CR1 |= TIM2_TO_TIM5_CEN;
		TIMER_WaitFlag(TIM2_ADDRESS);

		TIMER_Clock(Disabled,TIMER_2);

		return Success;

}



