/*
 * system_settings.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Sebastian G.M.
 *
 */

#include "system_settings.h"

#define TIM3_HANDLER TIM3_IRQHandler

//static void TIMER_cleanCountFlag(TimerMapAddr_t TIMER_addr);
//static void TIMER_Clock( Enabled_Disabled_t state, timers_enb_t Timer);
//static void TIMER_WaitFlag(TimerMapAddr_t TIMER_addr);

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


void volatile TIMER_cleanCountFlag(TimerMapAddr_t TIMER_addr){
	uint32_t volatile *TIM_REG_SR = (uint32_t volatile*)(TIMER_addr + TIMx_SR);

//	if(!((TIM2_5_UIF) & (*TIM_REG_SR))){
		*TIM_REG_SR &= ~(TIM2_5_UIF);
//	}

}

void volatile TIMER_Clock( Enabled_Disabled_t state, timers_enb_t Timer){
	RCC_offset_t volatile RccOffset = RCC_OFFSET_APB1ENR;

	if(Timer >= 10){
		Timer-=10; //it means that is the timers from the APB2
		RccOffset = RCC_OFFSET_APB2ENR;
	}

	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(RCC_ADDRESS + RccOffset);
//	*pClockControlReg = state ? (*pClockControlReg | (Enabled << Timer)) : (*pClockControlReg & ~(Enabled << Timer));
    if(state) {
        *pClockControlReg |= (Enabled << Timer);
    } else {
        *pClockControlReg &= ~(Enabled << Timer);
    }
}

void volatile TIMER_WaitFlag(TimerMapAddr_t TIMER_addr){ //use of SR and UIF?
	uint32_t volatile *TIM_REG_SR = (uint32_t volatile*)(TIMER_addr + TIMx_SR);
	while(!(TIM2_5_UIF & *TIM_REG_SR)){}
	TIMER_cleanCountFlag(TIMER_addr);

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


		*TIM_REG_CR1 &= ~TIM2_5_CEN; // Disable timer before configuration
		*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY-1);
		*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,microseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
		TIMER_cleanCountFlag(TIM2_ADDRESS);


		*TIM_REG_CNT = 0;

//		for(uint8_t i =0;i<100;i++){
//			__NOP();
//		}
		*TIM_REG_CR1 |= TIM2_5_CEN;
		TIMER_WaitFlag(TIM2_ADDRESS);

		TIMER_Clock(Disabled,TIMER_2);

		return Success;

}


void TIM3_HANDLER(void){

}

void init_timer3(uint32_t microseconds){ //init function to execute handler after the count happend

	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_PSC);
	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_ARR);
	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_CNT);
	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_CR1);

	uint32_t volatile *TIM_REG_DIER = (uint32_t volatile*)(TIM3_ADDRESS + TIMx_DIER);
	TIMER_Clock(Enabled,TIMER_3);

	*TIM_REG_CR1 &= ~TIM2_5_CEN; // Disable timer before configuration
	*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY-1);
	*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,microseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	for(uint8_t i =0;i<100;i++){
		__NOP();
	}

	*TIM_REG_CR1 |= TIM2_5_CEN;


	*TIM_REG_DIER |= TIM2_5_UIE;

	NVIC_EnableIRQ(TIM3_IRQn);

}


//void timer_1hz_1s_init(void){
//	uint32_t microseconds_timer =0;
//	uint32_t time_nedded = 1000000; //in microseconds
//	uint32_t arr_set_value =0;
//	uint32_t psc_set_value =16; //para esto es lo minimo para tener cuenta en microsegundos
//	float micro_time =0.0f;
//	micro_time = time_nedded*0.000001;
//
//	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(RCC_ADDRESS + RCC_OFFSET_APB1ENR);
//	*pClockControlReg |= (Enabled<<TIMER_2); //hasta aqui solo habilitamos el reloj del tim2
//	//la condiguracion es para 1hz entonces para el tiempo es 1/hz =s
//
//	uint32_t volatile *TIM_REG_SR = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_SR); //agregamos el 10 para decirle el offset
//	*TIM_REG_SR &= ~TIM2_5_UIF; //limpiamos la bandera
//
//
//	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_PSC);
//	*TIM_REG_PSC = (psc_set_value - 1); //clk 16 000 000 (16Mhz) = 16 000 000/1 6 = 1 000 000
//
//	microseconds_timer = (uint32_t)(BOARD_CLOCK / psc_set_value);
//
//	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_ARR);
//	arr_set_value = (microseconds_timer * micro_time);
//	*TIM_REG_ARR = (arr_set_value - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
//
//	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CNT);
//	*TIM_REG_CNT = 0;
//
//	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CR1);
//	*TIM_REG_CR1 |= TIM2_5_CEN;
//
//	NVIC_EnableIRQ(TIM2_IRQn);
//
//
//}

//void timer_wait(void){
//	uint32_t volatile *TIM_REG_SR = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_SR); //agregamos el 10 para decirle el offset
//	while(!((TIM2_5_UIF) & (*TIM_REG_SR))){} //cuando sea falso que se mantenga, ver si es normalmente en 0 o 1
//	*TIM_REG_SR &= ~TIM2_5_UIF; //cuando salga volvemos a limpiar esa bandera
//	//para el delay aqui se deberia desactivar el timer para que no se siguiera repitinedo
//	//hacer librerias de delay
//}

