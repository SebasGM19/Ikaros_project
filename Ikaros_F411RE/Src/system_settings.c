/*
 * system_settings.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Sebastian G.M.
 *
 */

#include "system_settings.h"

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

Status_code_t Delay(uint32_t Miliseconds){ //do it in microseconds later

	uint64_t count =0;
	uint64_t volatile time = 0;

	if(Miliseconds >= 1000000){
		return DelayTimeNotSupported;
	}
	count= Miliseconds*2022;

	while(time<count){
		time++;
	}
	time =0;
	count =0;

	return Success;
}



void timer_1hz_1s_init(void){
	uint32_t volatile *pClockControlReg = (uint32_t volatile *)(RCC_ADDRESS + RCC_OFFSET_APB1ENR);
	*pClockControlReg |= (Enabled<<TIMER_2); //hasta aqui solo habilitamos el reloj del tim2
	//la condiguracion es para 1hz entonces para el tiempo es 1/hz =s

	uint32_t volatile *TIM_REG_PSC = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_PSC);
	*TIM_REG_PSC = (1600 - 1); //clk nuestro 16 000 000 (16Mhz) = 16 000 000/1 600 = 10 000

	uint32_t volatile *TIM_REG_ARR = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_ARR);
	*TIM_REG_ARR = (10000 - 1); //termina en 10 000/10 000 = 1

	uint32_t volatile *TIM_REG_CNT = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CNT);
	*TIM_REG_CNT = 0;

	uint32_t volatile *TIM_REG_CR1 = (uint32_t volatile*)(TIM2_ADDRESS + TIMx_CR1);
	*TIM_REG_CR1 |= TIM2_5_CEN;

//	NVIC_EnableIRQ(TIM2_IRQn);


}

void timer_wait(void){
	uint32_t const volatile * const TIM_REG_SR = (uint32_t const volatile *const)(TIM2_ADDRESS + TIMx_SR); //agregamos el 10 para decirle el offset
	while(!((TIM2_5_UIF) & (*TIM_REG_SR))){} //cuando sea falso que se mantenga, ver si es normalmente en 0 o 1
	*TIM_REG_SR &= ~TIM2_5_UIF; //cuando salga volvemos a limpiar esa bandera
	//para el delay aqui se deberia desactivar el timer para que no se siguiera repitinedo
}

//uint8_t Delay_with_timer2(void){
//
//	//primero habilitar el reloj del timer2 APB1 con el offset del apb1
//	//set el preescaler
//	//set auto reload value
//	//clear counter
//	//enabled timer
//
//
//}
