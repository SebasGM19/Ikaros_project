/*
 * system_settings.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Sebastian G.M.
 *
 */

#include "system_settings.h"
#include "timers.h"

void Init_Board(void){
	__disable_irq();
	Delay(100000);
	__enable_irq();


}


Status_code_t ClockEnable(Set_Port_t Port_define, Enabled_Disabled_t Intention){
	Status_code_t status = Success;
	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_AHB1ENR);

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

	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(Timer + TIMx_CR1);
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(Timer + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(Timer + TIMx_ARR);

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

	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM2_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM2_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM2_ADDRESS + TIMx_CNT);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM2_ADDRESS + TIMx_CR1);

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
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM1_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM1_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM1_ADDRESS + TIMx_CNT);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM1_ADDRESS + TIMx_CR1);

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


void ftoa(float decimalData, uint8_t* cadena, uint8_t decimales){

        uint8_t DATA_BUFF_MAX_SIZE = 8;
        uint8_t string_dot[]={"."};
        uint8_t string_cero[]={"0"};
        uint8_t string_menos[]={"-"};
        uint16_t entero;
        uint32_t parte_decimal;
        uint32_t multiplicador=1;
        uint8_t cuenta_ceros=0;
        uint8_t bandera=0;

        uint8_t buff[DATA_BUFF_MAX_SIZE];
        memset(buff, '\0', DATA_BUFF_MAX_SIZE);

        if(decimalData<0){
            strcpy((char *)cadena, (const char *)(string_menos));
            decimalData=decimalData*-1;
            bandera=1;
        }

        entero=decimalData;// ejemplo es el 23.44 guardara el 23 -0.00220
        utoa(entero, (char *)(buff), 10);//pasamos a entero la parte antes del punto

        if(bandera==0){
            strcpy((char*)cadena, (const char *)buff);//guardamos el entero en cadena
        }else{
            strncat((char*)cadena, (const char *)buff, strlen((const char *)buff));
        }

        memset(buff, '\0', DATA_BUFF_MAX_SIZE);
        strcat((char *)cadena, (const char *)(string_dot)); //hasta aqui conectamos el 23.

        for(uint8_t i=0; i<decimales;i++){

            multiplicador*=10;
            parte_decimal = (decimalData-entero)*(multiplicador);

            if(parte_decimal==0){
                strcat((char*)cadena, (const char *)(string_cero)); //agrega cero
                cuenta_ceros++;
            }
        }

        if(cuenta_ceros!=decimales){
            utoa(parte_decimal, (char *)(buff), 10);
            strncat((char *)cadena, (const char *)(buff), strlen((const char *)buff));
            memset(buff, '\0', DATA_BUFF_MAX_SIZE);
        }

        cuenta_ceros=0;
        bandera=0;
}

bool RCC_reset_status_flag(RCC_CSR_t flag){
	__I uint32_t *const RCC_CSR_Reg = (__I uint32_t *const)(RCC_ADDRESS + RCC_OFFSET_CSR);
	if((*RCC_CSR_Reg) & flag){
		return true;
	}else{
		return false;
	}
}

void RCC_clear_reset_flags(void){
	__IO uint32_t *RCC_CSR_Reg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_CSR);
	*RCC_CSR_Reg |= RCC_RMVF;
}

void SYS_ClockEnable(Enabled_Disabled_t Intention){
	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_APB2ENR);

	if(Intention){
		*pClockControlReg |= RCC_SYSCFG_ENABLE;
	}else{
		*pClockControlReg &= ~(RCC_SYSCFG_ENABLE);
	}
}

