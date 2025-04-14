/*
 * watchdog.c
 *
 *  Created on: Oct 17, 2024
 *      Author: sebas
 */
#include "watchdog.h"
#include "uart.h"


WWDG_config_t static WWDG_global_params;


void Init_Ind_Watchdog(IWDG_reload_time_t reload_time){
	__O uint32_t *IWDG_KR_Reg = (__O uint32_t *)(IWDG_ADDRESS + IWDG_KR);
	__IO uint32_t *IWDG_PR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_PR);
	__IO uint32_t *IWDG_RLR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_RLR);
	if(system_is_debugging()){
		frezze_IWDG();
	}

	*IWDG_KR_Reg = IWDG_ENABLE_ACCESS;
	*IWDG_PR_Reg |= (reload_time<<IWDG_PR_DIV);
	*IWDG_RLR_Reg |= IWDG_MAX_RL_TIME;
	*IWDG_KR_Reg |= IWDG_RELOAD_COUNT;
	*IWDG_KR_Reg = IWDG_START_COUNT;

}

void Ind_Watchdog_control(watchdog_food_t food){

	if(food){
		Ind_reloadTheDog();
	}else{
		Ind_resetTheDog();
	}

}


void Ind_resetTheDog(void){

	__O uint32_t *IWDG_KR_Reg = (__O uint32_t *)(IWDG_ADDRESS + IWDG_KR);
	__IO uint32_t *IWDG_PR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_PR);
	__IO uint32_t *IWDG_RLR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_RLR);

	*IWDG_KR_Reg |= IWDG_START_COUNT;
	*IWDG_PR_Reg |= (reload_512ms<<IWDG_PR_DIV);
	*IWDG_RLR_Reg |= IWDG_MIN_RL_RESET_DOG;
	*IWDG_KR_Reg |= IWDG_RELOAD_COUNT;

}


void Ind_reloadTheDog(void){
	__O uint32_t *IWDG_KR_Reg = (__O uint32_t *)(IWDG_ADDRESS + IWDG_KR);

	*IWDG_KR_Reg |= IWDG_RELOAD_COUNT;

}




/*__________________WINDOW_WD__________________*/
/*Window watchdog is used for precision application less than 130ms*/
void WWDG_HANDLER(void){
	Win_reloadTheDog(); //keep reloading the dog until you procces is over, then  use the Win_resetTheDog function
	Win_clear_reset_flag();


//	UART2_Write("UART2 WWDG RESET!\r\n",19 , 1000);
//	Win_reloadTheDog();
//	UART1_Write("UART1 WWDG RESET!\r\n",19 , 1000);
//	Win_reloadTheDog();


	Win_resetTheDog(); //last line
}


Status_code_t Init_Win_Watchdog(WWDG_config_t config_window){ //max time 0.130 s to wd reset

	uint32_t T_var=0;
	uint32_t W_var=0;

	__IO uint32_t *WWDG_CR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CR);
	__IO uint32_t *WWDG_CFR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CFR);

	if(config_window.T_max_time >= WWDG_MAX_MS_TIME_VALUE || config_window.T_max_time < WWDG_MIN_MS_TIME_VALUE
			|| config_window.W_time >= WWDG_MAX_MS_TIME_VALUE ||config_window.W_time < WWDG_MIN_MS_TIME_VALUE
			||config_window.W_time >= config_window.T_max_time){

		return WWDG_invalid_parameter;
	}
//	*WWDG_CR_Reg |= WWDG_INIT_START_TIME<<WWDG_T;

	T_var = WWDG_MS_TIME_T(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.T_max_time);
	W_var = WWDG_MS_TIME_T(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.T_max_time) - WWDG_MS_TIME_W(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.W_time);

	if(T_var >= WWDG_MAX_COUNT_VALUE || W_var >= WWDG_MAX_COUNT_VALUE){
		return WWDG_invalid_parameter;
	}

	if(system_is_debugging()){
		frezze_WWDG();
	}

	WWDG_Clock(Enabled);
	Win_clear_reset_flag();


	WWDG_global_params = config_window;


	*WWDG_CFR_Reg &= ~(Clear_two_bits<<WWDG_WDGTB);

	*WWDG_CFR_Reg |= (WWDG_MAX_CK_DIV<<WWDG_WDGTB | (WWDG_MS_TIME_T(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.T_max_time) - WWDG_MS_TIME_W(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.W_time)) );


	if(WWDG_global_params.interrupt_access){
		*WWDG_CFR_Reg |= WWDG_EWI;

		NVIC_EnableIRQ(WWDG_IRQn);
	}

	*WWDG_CR_Reg  |=  (WWDG_WDGA | WWDG_MS_TIME_T(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.T_max_time));

	return Success;

}

void Win_Watchdog_control(watchdog_food_t food){

	if(food){
		Win_reloadTheDog();
	}else{
		Win_resetTheDog();
	}

}
void Win_resetTheDog(void){
	__IO uint32_t *WWDG_CFR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CFR);
	__IO uint32_t *WWDG_CR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CR);

	NVIC_DisableIRQ(WWDG_IRQn);

	*WWDG_CR_Reg |= WWDG_INIT_START_TIME<<WWDG_T;


	*WWDG_CFR_Reg &= ~(Clear_two_bits<<WWDG_WDGTB);
	*WWDG_CFR_Reg |= WWDG_MIN_CK_DIV<<WWDG_WDGTB;
	*WWDG_CFR_Reg |= WWDG_W_VALUE_TO_RESET;

	*WWDG_CR_Reg  |=  WWDG_T_VALUE_TO_RESET;

}

void Win_clear_reset_flag(void){
	__IO uint32_t *WWDG_CR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_SR);
	*WWDG_CR_Reg &= ~(WWDG_EWIF);
}



void Win_reloadTheDog(void){
	__IO uint32_t *WWDG_CR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CR);

	*WWDG_CR_Reg |= WWDG_MS_TIME_T(BOARD_CLOCK,WWDG_MAX_CK_DIV,WWDG_global_params.T_max_time)<<WWDG_T;

}

Status_code_t WWDG_Clock(Enabled_Disabled_t state){

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_APB1ENR);

    *pClockControlReg = (state) ? (*pClockControlReg | (WWDG_CLOCK_BIT_REG)) : (*pClockControlReg & ~(WWDG_CLOCK_BIT_REG));

    return Success;

}
