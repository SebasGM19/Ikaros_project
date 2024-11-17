/*
 * watchdog.c
 *
 *  Created on: Oct 17, 2024
 *      Author: sebas
 */
#include "watchdog.h"

WWDG_config_t static WWDG_global_params;


void Init_Ind_Watchdog(IWDG_reload_time_t reload_time){
	__O uint32_t *IWDG_KR_Reg = (__O uint32_t *)(IWDG_ADDRESS + IWDG_KEY);
	__IO uint32_t *IWDG_PR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_PR);
	__IO uint32_t *IWDG_RLR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_RL);

	*IWDG_KR_Reg = IWDG_START_COUNT;
	*IWDG_PR_Reg = (reload_time<<IWDG_PR_DIV);
	*IWDG_RLR_Reg = IWDG_MAX_RL_TIME;
	*IWDG_KR_Reg = IWDG_RELOAD_COUNT;

}

void Ind_Watchdog_control(watchdog_food_t food){

	if(food){
		Ind_reloadTheDog();
	}else{
		Ind_resetTheDog();
	}

}


void Ind_resetTheDog(void){

	__O uint32_t *IWDG_KR_Reg = (__O uint32_t *)(IWDG_ADDRESS + IWDG_KEY);
	__IO uint32_t *IWDG_PR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_PR);
	__IO uint32_t *IWDG_RLR_Reg = (__IO uint32_t *)(IWDG_ADDRESS + IWDG_RL);

	*IWDG_KR_Reg = IWDG_START_COUNT;
	*IWDG_PR_Reg = (reload_512ms<<IWDG_PR_DIV);
	*IWDG_RLR_Reg = IWDG_MIN_RL_RESET_DOG;
	*IWDG_KR_Reg = IWDG_RELOAD_COUNT;

}


void Ind_reloadTheDog(void){
	__O uint32_t *IWDG_KR_Reg = (__O uint32_t *)(IWDG_ADDRESS + IWDG_KEY);

	*IWDG_KR_Reg = IWDG_RELOAD_COUNT;

}




/*__________________WINDOW_WD__________________*/
void WWDG_HANDLER(void){


}


Status_code_t Init_Win_Watchdog(WWDG_config_t config_window){

	__IO uint32_t *IWDG_CFR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CFR);
	__IO uint32_t *WWDG_CR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CR);

	if(config_window.T_max_time >= WWDG_MAX_TIME_VALUE || config_window.T_max_time < WWDG_MIN_TIME_VALUE
			|| config_window.W_time >= WWDG_MAX_TIME_VALUE ||config_window.W_time < WWDG_MIN_TIME_VALUE
			||config_window.W_time >= config_window.T_max_time){

		return WWDG_invalid_parameter;
	}

	WWDG_global_params = config_window;

	WWDG_Clock(Enabled);
	*WWDG_CR_Reg |= WWDG_INIT_START_TIME<<WWDG_T;


	*IWDG_CFR_Reg &= ~(Clear_two_bits<<WWDG_WDGTB);
	*IWDG_CFR_Reg |= WWDG_MAX_CK_DIV<<WWDG_WDGTB;
	*IWDG_CFR_Reg |= WWDG_MS_TIME(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.W_time)<<WWDG_W ;


	*WWDG_CR_Reg  |=  WWDG_MS_TIME(BOARD_CLOCK,WWDG_MAX_CK_DIV,config_window.T_max_time)<<WWDG_T;
	*WWDG_CR_Reg |= (WWDG_WDGA);

	NVIC_EnableIRQ(WWDG_IRQn);
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
	__IO uint32_t *IWDG_CFR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CFR);
	__IO uint32_t *WWDG_CR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CR);

	*WWDG_CR_Reg |= WWDG_INIT_START_TIME<<WWDG_T;


	*IWDG_CFR_Reg &= ~(Clear_two_bits<<WWDG_WDGTB);
	*IWDG_CFR_Reg |= WWDG_MIN_CK_DIV<<WWDG_WDGTB;
	*IWDG_CFR_Reg |= WWDG_W_VALUE_TO_RESET<<WWDG_W ;

	*WWDG_CR_Reg  |=  WWDG_T_VALUE_TO_RESET<<WWDG_T;

}


void Win_reloadTheDog(void){
	__IO uint32_t *WWDG_CR_Reg = (__IO uint32_t *)(WWDG_ADDRESS + WWDG_CR);

	*WWDG_CR_Reg |= WWDG_MS_TIME(BOARD_CLOCK,WWDG_MAX_CK_DIV,WWDG_global_params.T_max_time)<<WWDG_T;

}

Status_code_t WWDG_Clock(Enabled_Disabled_t state){

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_APB1ENR);

    *pClockControlReg = (state) ? (*pClockControlReg | (WWDG_CLOCK_BIT_REG)) : (*pClockControlReg & ~(WWDG_CLOCK_BIT_REG));

    return Success;

}
