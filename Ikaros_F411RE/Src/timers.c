/*
 * timers.c
 *
 *  Created on: Jun 6, 2024
 *      Author: Sebastian G.M.
 */

#include "timers.h"
#include "gpios.h"
#include "lcd.h"
#include "keypad_4x4.h"
#include "adc.h"
#include "watchdog.h"


uint32_t static global_TIM3_ARR_count = 1024; //set as 1048 to default
uint32_t static global_TIM4_ARR_count = 1024; //set as 2048 to default
//uint32_t static global_TIM5_ARR_count = 1024; //set as 2048 to default


bool static volatile reset_tim3_flag = false;
bool static volatile reset_tim4_flag = false;
bool static volatile reset_tim5_flag = false;
bool static volatile reset_tim11_flag = false;

bool static volatile TIM11_interrupt_flag_active = false;


void TIMER_cleanCountFlag(TimerMapAddr_t TIMER_addr){
	__IO uint32_t *TIM_REG_SR = (__IO uint32_t *)(TIMER_addr + TIMx_SR);

	*TIM_REG_SR &= ~(TIMx_UIF);

}

void TIMER_Clock( Enabled_Disabled_t state, timers_enb_t Timer){
	RCC_offset_t RccOffset = RCC_OFFSET_APB1ENR;

	if(Timer >= 10){
		Timer-=10; //it means that is the timers from the APB2
		RccOffset = RCC_OFFSET_APB2ENR;
	}

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RccOffset);
    if(state) {
        *pClockControlReg |= (Enabled << Timer);
    } else {
        *pClockControlReg &= ~(Enabled << Timer);
    }
}

void TIMER_WaitFlag(TimerMapAddr_t TIMER_addr){
	__I uint32_t *const TIM_REG_SR = (__I uint32_t *const)(TIMER_addr + TIMx_SR);
	while(!(TIMx_UIF & *TIM_REG_SR)){}
	TIMER_cleanCountFlag(TIMER_addr);

}




/*////////////////////////////////// TIMER 3 HANDLER AND CONTROL FUNCTIONS/////////////////////////////////////*/
void TIM3_HANDLER(void){
	TIMER_cleanCountFlag(TIM3_ADDRESS);

	if(reset_tim3_flag){
	/*Develop all the code to be executed in a second thread down here*/

		Win_Watchdog_control(reload_food); //reload the food


	}else{reset_tim3_flag=!reset_tim3_flag;}
}

Status_code_t TIM3_Init(uint16_t milliseconds){

	if(milliseconds > MAX_TIME_TIM3_TIM4){
		return TimeSetNotSuported;
	}
	TIMER_Clock(Enabled,TIMER_3);
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CNT);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);

	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_DIER);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;

	*TIM_REG_PSC = (PSC_TO_MILLISEC_DELAY-1);
	*TIM_REG_ARR = (MILLSEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MILLISEC_DELAY,milliseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	NVIC_EnableIRQ(TIM3_IRQn);

	return Success;

}
void TIM3_Start(void){

	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CNT);

	TIMER_cleanCountFlag(TIM3_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_CR1 |= TIMx_CEN;
	*TIM_REG_DIER |= TIMx_UIE;


}

void TIM3_Stop(void){

	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CNT);

	TIMER_cleanCountFlag(TIM3_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;
}

void TIM3_Deinit(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_DIER);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;
	NVIC_DisableIRQ(TIM3_IRQn);

	TIMER_Clock(Disabled,TIMER_3);

}


/*////////////////////////////////// TIMER 4 HANDLER AND CONTROL FUNCTIONS/////////////////////////////////////*/
void TIM4_HANDLER(void){
	TIMER_cleanCountFlag(TIM4_ADDRESS);
	if(reset_tim4_flag){

	/*Develop all the code to be executed in a second thread down here*/


	}else{reset_tim4_flag=!reset_tim4_flag;}
}

Status_code_t TIM4_Init(uint16_t milliseconds){

	if(milliseconds > MAX_TIME_TIM3_TIM4){
		return TimeSetNotSuported;
	}
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CNT);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);

	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_DIER);
	TIMER_Clock(Enabled,TIMER_4);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;

	*TIM_REG_PSC = (PSC_TO_MILLISEC_DELAY-1);
	*TIM_REG_ARR = (MILLSEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MILLISEC_DELAY,milliseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	NVIC_EnableIRQ(TIM4_IRQn);

	return Success;

}

void TIM4_Start(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CNT);

	TIMER_cleanCountFlag(TIM4_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_CR1 |= TIMx_CEN;
	*TIM_REG_DIER |= TIMx_UIE;

}

void TIM4_Stop(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CNT);


	TIMER_cleanCountFlag(TIM4_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;


}

void TIM4_Deinit(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_DIER);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;
	NVIC_DisableIRQ(TIM4_IRQn);

	TIMER_Clock(Disabled,TIMER_4);

}


/*///////////////////TIMER 5 HANDLER AND CONTROL FUNCTIONS///////////////////////////////////*/
uint8_t netx_lcd_msg =0;//dummy var
void state_to_print(uint8_t state){  //dummy function
	netx_lcd_msg = state;
}

void TIM5_HANDLER(void){
	/*Develop all the code to be executed in a second thread down here*/
	TIMER_cleanCountFlag(TIM5_ADDRESS);
	if(reset_tim5_flag){

		switch(netx_lcd_msg){
		case 1:
			lcd_printXY(0, 0," Welcome to my  ", 16);
			lcd_printXY(0, 1," State Machine  ", 16);
			break;
		case 2:
			lcd_printXY(0, 0,"1-Write UART2 : ", 16);
			lcd_printXY(0, 1,"2-Write UART1 >6", 16);
			break;
		case 3:
			lcd_printXY(0, 0,"3-Read UART1  : ", 16);
			lcd_printXY(0, 1,"4-Reset WWDG  <8", 16);
			break;
		case 4:
			lcd_printXY(0, 0," Write to UART2 ", 16);
			lcd_printXY(0, 1,"                ", 16);
			break;
		case 5:
			lcd_printXY(0, 0," Write to UART1 ", 16);
			lcd_printXY(0, 1,"                ", 16);
			break;
		case 6:
			lcd_printXY(0, 0,"Received Data:  ", 16);
			lcd_printXY(0, 1,"                ", 16);
			break;
		case 7:
			lcd_printXY(0, 0,"  WWDG RESET!!  ", 16);
			lcd_printXY(0, 1,"                ", 16);
			break;
		case 8:
			lcd_printXY(0, 0," INVALID OPTION ", 16);
			lcd_printXY(0, 1,"                ", 16);
			break;


		}

	}else{reset_tim5_flag=!reset_tim5_flag;}
}



Status_code_t TIM5_Init(uint32_t microseconds){

	if(microseconds > MAX_TIME_TIM5_AND_TIM2){
		return TimeSetNotSuported;
	}
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CNT);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_DIER);

	TIMER_Clock(Enabled,TIMER_5);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;

	*TIM_REG_PSC = (PSC_TO_MICROSEC_DELAY-1);
	*TIM_REG_ARR = (USEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MICROSEC_DELAY,microseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	NVIC_EnableIRQ(TIM5_IRQn);

	return Success;

}

void TIM5_Start(void){

	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CNT);


	TIMER_cleanCountFlag(TIM5_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_DIER |= TIMx_UIE;
	*TIM_REG_CR1 |= TIMx_CEN;
}

void TIM5_Stop(void){

	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CNT);

	TIMER_cleanCountFlag(TIM5_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;

}

void TIM5_Deinit(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_DIER);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;
	NVIC_DisableIRQ(TIM5_IRQn);

	TIMER_Clock(Disabled,TIMER_5);

}


/*///////////////////TIMER 11 HANDLER FOR TIMEOUT FOR PROTOCOLS///////////////////////////////////*/
void TIM11_HANDLER(void){
	TIMER_cleanCountFlag(TIM11_ADDRESS);
	if(reset_tim11_flag){
	/*Develop all the code to be executed in a second thread down here*/

		TIM11_interrupt_flag_active = true;
		TIM11_Stop();


	}else{reset_tim11_flag=!reset_tim11_flag;}
}



Status_code_t TIM11_Init(uint32_t milliseconds){

	if(milliseconds > MAX_TIME_TIM9_TO_TIM11){
		return TimeSetNotSuported;
	}
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t*)(TIM11_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_CNT);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_DIER);

	TIMER_Clock(Enabled,TIMER_11);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;

	*TIM_REG_PSC = (PSC_TO_MILLISEC_DELAY-1);
	*TIM_REG_ARR = (MILLSEC_TO_DELAY(BOARD_CLOCK,PSC_TO_MILLISEC_DELAY,milliseconds) - 1); //real para 5s = the result from the psc it aplied in this ecuation arr/1000000= seconds
	*TIM_REG_CNT = 0;

	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

	return Success;

}

void TIM11_Start(void){

	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_CNT);


	TIMER_cleanCountFlag(TIM11_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_DIER |= TIMx_UIE;
	*TIM_REG_CR1 |= TIMx_CEN;
}

void TIM11_Stop(void){

	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_DIER);
	__IO uint32_t *TIM_REG_CNT = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_CNT);

	TIMER_cleanCountFlag(TIM11_ADDRESS);
	*TIM_REG_CNT = 0;
	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;

}

void TIM11_Deinit(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_DIER = (__IO uint32_t *)(TIM11_ADDRESS + TIMx_DIER);

	*TIM_REG_CR1 &= ~TIMx_CEN; // Disable timer before configuration
	*TIM_REG_DIER &= ~TIMx_UIE;
	NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);

	TIMER_Clock(Disabled,TIMER_11);

}


bool TIM11_GET_interrupt_flag_status(void){
	return TIM11_interrupt_flag_active;
}

void TIM11_clear_interrupt_flag(void){
	TIM11_interrupt_flag_active = false;
}



/*______________________________PWM functions for TIM3____________________________________________*/
void TIM3_set_global_PWM_ARR(uint32_t new_arr_value){
	global_TIM3_ARR_count = new_arr_value;
}

Status_code_t TIM3_PWM_Init(TIM3_PWM_channel_select_t channel,PWM_mode_OCxM_t mode){

	Status_code_t status = Success;
	TIMx_register_offset_t capture_compare_mode_reg = TIMx_CCMR1;
	Clean_PWM_channel_t clean_channel = Clean_pwm_channel_1_and_3;
	TIM2_to_TIM5_CCER_t CCx_enable = TIM2_TO_TIM5_CC1E;
	TIM2_to_TIM5_CCER_t CCx_polarity = TIM2_TO_TIM5_CC1P;
	TIM1_to_TIM5_CCMR1_CCMR2_t CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
	TIM1_to_TIM5_CCMR1_CCMR2_t CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;


	switch(channel){
	case TIM3_CH1:
		 capture_compare_mode_reg = TIMx_CCMR1;
		 clean_channel = Clean_pwm_channel_1_and_3;
		 CCx_enable = TIM2_TO_TIM5_CC1E;
		 CCx_polarity = TIM2_TO_TIM5_CC1P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
		 CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;
		break;
	case TIM3_CH2:
		 capture_compare_mode_reg = TIMx_CCMR1;
		 clean_channel = Clean_pwm_channel_2_and_4;
		 CCx_enable = TIM2_TO_TIM5_CC2E;
		 CCx_polarity = TIM2_TO_TIM5_CC2P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC2S_and_CC4S;
		 CCMRx_mode = TIM1_TO_TIM5_OC2M_and_OC4M;

		break;
	case TIM3_CH3:
		 capture_compare_mode_reg = TIMx_CCMR2;
		 clean_channel = Clean_pwm_channel_1_and_3;
		 CCx_enable = TIM2_TO_TIM5_CC3E;
		 CCx_polarity = TIM2_TO_TIM5_CC3P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
		 CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;

		break;
	case TIM3_CH4:
		 capture_compare_mode_reg = TIMx_CCMR2;
		 clean_channel = Clean_pwm_channel_2_and_4;
		 CCx_enable = TIM2_TO_TIM5_CC4E;
		 CCx_polarity = TIM2_TO_TIM5_CC4P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC2S_and_CC4S;
		 CCMRx_mode = TIM1_TO_TIM5_OC2M_and_OC4M;


		break;
	default:
		return TIMx_incorrect;
		break;
	}


	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_CCMRx = (__IO uint32_t *)(TIM3_ADDRESS + capture_compare_mode_reg);
	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CCER);



	status = SetPinMode(Port_B, channel, Alt_func_mode); //always timer for PORT B defined

	if(status!=Success){
		return status;
	}

	TIMER_Clock(Enabled,TIMER_3);
	GpioSetAlternativeFunction(Port_B, channel, TIM3_TIM4_TIM5); 		//alt 2

	*TIM_REG_CCMRx &= ~(clean_channel); 								//clean the bits for the channel

	*TIM_REG_CCMRx &= ~(Clear_two_bits<<CCMRx_selecction); 				//00 means PWM output

	*TIM_REG_CCER &= ~CCx_polarity; 									// 0 to high polarity

	*TIM_REG_CCMRx |= (mode<<CCMRx_mode); 								// configuring PWM mode

	*TIM_REG_CCER &= ~CCx_enable; 										//deinit the channel compare

	*TIM_REG_CCMRx |= TIM1_TO_TIM5_OC1PE_and_OC3PE;						//Output compare 1 pre-load enable
	*TIM_REG_CR1 |= TIMx_ARPE;


	return status;

}


Status_code_t TIM3_PWM_start_custom_channel(TIM3_pwm_custom_parameters_t const PWM_Custom){


	TIMx_register_offset_t capture_compare_reg = TIMx_CCR1;
	TIM2_to_TIM5_CCER_t capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;


	switch(PWM_Custom.channel){
	case TIM3_CH1:
		 capture_compare_reg = TIMx_CCR1;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
		break;
	case TIM3_CH2:
		 capture_compare_reg = TIMx_CCR2;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC2E;
		break;
	case TIM3_CH3:
		 capture_compare_reg = TIMx_CCR3;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC3E;
		break;
	case TIM3_CH4:
		 capture_compare_reg = TIMx_CCR4;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC4E;
		break;
	default:
		return TIMx_incorrect;
		break;
	}
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CCER);
	__IO uint32_t *TIM_REG_CCRx = (__IO uint32_t *)(TIM3_ADDRESS + capture_compare_reg);



	*TIM_REG_PSC = (PWM_Custom.prescaler);
	*TIM_REG_ARR = (PWM_Custom.Total_count_ARR - 1); 	//total count
	*TIM_REG_CCRx= PWM_Custom.duty_count;  				//start count

	*TIM_REG_CR1 |= TIMx_CEN;
	*TIM_REG_CCER |= capture_compare_enb_reg; 			// enable channel output


	return Success;

}


/*
  NOTE:
  to reach higher frequency modify the variable global_TIM3_ARR_count using the function TIM3_set_global_PWM_ARR

  or use the function TIM3_PWM_start_channel_custom to personalizes the frequency and duty cycle
 */
Status_code_t TIM3_PWM_start_channel(TIM3_pwm_auto_parameters_t const PWM){


	TIMx_register_offset_t capture_compare_reg = TIMx_CCR1;
	TIM2_to_TIM5_CCER_t capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;

	uint32_t miliseconds_duty = 0;

	int32_t preescaler=0;


	switch(PWM.channel){
	case TIM3_CH1:
		 capture_compare_reg = TIMx_CCR1;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
		break;
	case TIM3_CH2:
		 capture_compare_reg = TIMx_CCR2;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC2E;
		break;
	case TIM3_CH3:
		 capture_compare_reg = TIMx_CCR3;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC3E;
		break;
	case TIM3_CH4:
		 capture_compare_reg = TIMx_CCR4;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC4E;
		break;
	default:
		return TIMx_incorrect;
		break;
	}
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CCER);
	__IO uint32_t *TIM_REG_CCRx = (__IO uint32_t *)(TIM3_ADDRESS + capture_compare_reg);


	if(PWM.duty_cycle_percent<=0){
		miliseconds_duty=0;
	}else if(PWM.duty_cycle_percent>=100){
		miliseconds_duty = global_TIM3_ARR_count;
	}else{
		miliseconds_duty = (uint32_t)((PWM.duty_cycle_percent * global_TIM3_ARR_count) / 100.0f);
	}

	preescaler = (int32_t)round((float)((BOARD_CLOCK / ((global_TIM3_ARR_count + 1) * PWM.frequency)) - 1));
	if(preescaler<=0){
		/*NOTE: if this happend try change the global_ARR_reg to a lower value*/
		return PWM_frecuency_not_suported_in_this_mode;
	}

	*TIM_REG_PSC = preescaler;
	*TIM_REG_ARR = (global_TIM3_ARR_count-1);
	*TIM_REG_CCRx = miliseconds_duty;

	*TIM_REG_CR1 |= TIMx_CEN; 						// Disable timer before configuration
	*TIM_REG_CCER |= capture_compare_enb_reg; 		// enable channel output


	return Success;

}

Status_code_t TIM3_PWM_stop_channel(TIM3_PWM_channel_select_t channel){

	TIM2_to_TIM5_CCER_t CCx_enable = TIM2_TO_TIM5_CC1E;

	switch(channel){
	case TIM3_CH1:
		 CCx_enable = TIM2_TO_TIM5_CC1E;
		break;
	case TIM3_CH2:
		 CCx_enable = TIM2_TO_TIM5_CC2E;
		break;
	case TIM3_CH3:
		 CCx_enable = TIM2_TO_TIM5_CC3E;
		break;
	case TIM3_CH4:
		 CCx_enable = TIM2_TO_TIM5_CC4E;
		break;
	default:
		return TIMx_incorrect;
		break;
	}


	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CCER);

	*TIM_REG_CCER &= ~CCx_enable;


	    return Success;

}

Status_code_t TIM3_PWM_Deinit(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM3_ADDRESS + TIMx_CR1);

	*TIM_REG_CR1 &= ~TIMx_CEN;
	TIMER_Clock(Disabled,TIMER_3);

	return Success;
}


/*__________________________________PWM functions for TIM4____________________________________________*/
void TIM4_set_global_PWM_ARR(uint32_t new_arr_value){
	global_TIM4_ARR_count = new_arr_value;
}

Status_code_t TIM4_PWM_Init(TIM4_PWM_channel_select_t channel,PWM_mode_OCxM_t mode){

	Status_code_t status = Success;
	TIMx_register_offset_t capture_compare_mode_reg = TIMx_CCMR1;
	Clean_PWM_channel_t clean_channel = Clean_pwm_channel_1_and_3;
	TIM2_to_TIM5_CCER_t CCx_enable = TIM2_TO_TIM5_CC1E;
	TIM2_to_TIM5_CCER_t CCx_polarity = TIM2_TO_TIM5_CC1P;
	TIM1_to_TIM5_CCMR1_CCMR2_t CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
	TIM1_to_TIM5_CCMR1_CCMR2_t CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;


	switch(channel){
	case TIM4_CH1:
		 capture_compare_mode_reg = TIMx_CCMR1;
		 clean_channel = Clean_pwm_channel_1_and_3;
		 CCx_enable = TIM2_TO_TIM5_CC1E;
		 CCx_polarity = TIM2_TO_TIM5_CC1P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
		 CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;
		break;
	case TIM4_CH2:
		 capture_compare_mode_reg = TIMx_CCMR1;
		 clean_channel = Clean_pwm_channel_2_and_4;
		 CCx_enable = TIM2_TO_TIM5_CC2E;
		 CCx_polarity = TIM2_TO_TIM5_CC2P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC2S_and_CC4S;
		 CCMRx_mode = TIM1_TO_TIM5_OC2M_and_OC4M;

		break;
	case TIM4_CH3:
		 capture_compare_mode_reg = TIMx_CCMR2;
		 clean_channel = Clean_pwm_channel_1_and_3;
		 CCx_enable = TIM2_TO_TIM5_CC3E;
		 CCx_polarity = TIM2_TO_TIM5_CC3P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
		 CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;

		break;
	case TIM4_CH4:
		 capture_compare_mode_reg = TIMx_CCMR2;
		 clean_channel = Clean_pwm_channel_2_and_4;
		 CCx_enable = TIM2_TO_TIM5_CC4E;
		 CCx_polarity = TIM2_TO_TIM5_CC4P;
		 CCMRx_selecction = TIM1_TO_TIM5_CC2S_and_CC4S;
		 CCMRx_mode = TIM1_TO_TIM5_OC2M_and_OC4M;

		break;
	default:
		return TIMx_incorrect;
		break;
	}


	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_CCMRx = (__IO uint32_t *)(TIM4_ADDRESS + capture_compare_mode_reg);	//depending the channel selected
	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CCER);



	status = SetPinMode(Port_B, channel, Alt_func_mode); //always timer for PORT B for PWM TIM4

	if(status!=Success){
		return status;
	}

	TIMER_Clock(Enabled,TIMER_4);
	GpioSetAlternativeFunction(Port_B, channel, TIM3_TIM4_TIM5); //alt 2

	*TIM_REG_CCMRx &= ~(clean_channel); 					//clean the bits for the channel

	*TIM_REG_CCMRx &= ~(Clear_two_bits<<CCMRx_selecction); 	//00 means PWM output

	*TIM_REG_CCER &= ~CCx_polarity; 						// 0 to high polarity

	*TIM_REG_CCMRx |= (mode<<CCMRx_mode); 					// configuring PWM mode

	*TIM_REG_CCER &= ~CCx_enable; 							//deinit the channel compare

	*TIM_REG_CCMRx |= TIM1_TO_TIM5_OC1PE_and_OC3PE;			//Output compare 1 pre-load enable
	*TIM_REG_CR1 |= TIMx_ARPE; 								//save compare count


	return status;

}


Status_code_t TIM4_PWM_start_custom_channel(TIM4_pwm_custom_parameters_t const PWM_Custom){


	TIMx_register_offset_t capture_compare_reg = TIMx_CCR1;
	TIM2_to_TIM5_CCER_t capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;


	switch(PWM_Custom.channel){
	case TIM4_CH1:
		 capture_compare_reg = TIMx_CCR1;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
		break;
	case TIM4_CH2:
		 capture_compare_reg = TIMx_CCR2;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC2E;
		break;
	case TIM4_CH3:
		 capture_compare_reg = TIMx_CCR3;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC3E;
		break;
	case TIM4_CH4:
		 capture_compare_reg = TIMx_CCR4;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC4E;
		break;
	default:
		return TIMx_incorrect;
		break;
	}
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CCER);
	__IO uint32_t *TIM_REG_CCRx = (__IO uint32_t *)(TIM4_ADDRESS + capture_compare_reg);



	*TIM_REG_PSC = (PWM_Custom.prescaler);
	*TIM_REG_ARR = (PWM_Custom.Total_count_ARR - 1); 	//total count
	*TIM_REG_CCRx= PWM_Custom.duty_count;  				//start count

	*TIM_REG_CR1 |= TIMx_CEN;
	*TIM_REG_CCER |= capture_compare_enb_reg; 			// enable channel output


	return Success;

}


/*
  NOTE:
  to reach higher frequency modify the variable global_TIM4_ARR_count using the function TIM4_set_global_PWM_ARR

  or use the function TIM4_PWM_start_channel_custom to personalizes the frequency and duty cycle
 */
Status_code_t TIM4_PWM_start_channel(TIM4_pwm_auto_parameters_t const PWM){


	TIMx_register_offset_t capture_compare_reg = TIMx_CCR1;
	TIM2_to_TIM5_CCER_t capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;

	uint32_t miliseconds_duty = 0;

	int32_t preescaler=0;


	switch(PWM.channel){
	case TIM4_CH1:
		 capture_compare_reg = TIMx_CCR1;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
		break;
	case TIM4_CH2:
		 capture_compare_reg = TIMx_CCR2;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC2E;
		break;
	case TIM4_CH3:
		 capture_compare_reg = TIMx_CCR3;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC3E;
		break;
	case TIM4_CH4:
		 capture_compare_reg = TIMx_CCR4;
		 capture_compare_enb_reg = TIM2_TO_TIM5_CC4E;
		break;
	default:
		return TIMx_incorrect;
		break;
	}
	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_PSC);
	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_ARR);
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);
	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CCER);
	__IO uint32_t *TIM_REG_CCRx = (__IO uint32_t *)(TIM4_ADDRESS + capture_compare_reg);


	if(PWM.duty_cycle_percent<=0){
		miliseconds_duty=0;
	}else if(PWM.duty_cycle_percent>=100){
		miliseconds_duty = global_TIM4_ARR_count;
	}else{
		miliseconds_duty = (uint32_t)((PWM.duty_cycle_percent * global_TIM4_ARR_count) / 100.0f);
	}

	preescaler = (int32_t)round((float)((BOARD_CLOCK / ((global_TIM4_ARR_count + 1) * PWM.frequency)) - 1));
	if(preescaler<=0){
		/*NOTE: if this happend try change the global_ARR_reg to a lower value*/
		return PWM_frecuency_not_suported_in_this_mode;
	}

	*TIM_REG_PSC = preescaler;
	*TIM_REG_ARR = (global_TIM4_ARR_count-1);
	*TIM_REG_CCRx = miliseconds_duty;

	*TIM_REG_CR1 |= TIMx_CEN; 						// Disable timer before configuration
	*TIM_REG_CCER |= capture_compare_enb_reg; 		// enable channel output


	return Success;

}

Status_code_t TIM4_PWM_stop_channel(TIM4_PWM_channel_select_t channel){

	TIM2_to_TIM5_CCER_t CCx_enable = TIM2_TO_TIM5_CC1E;

	switch(channel){
	case TIM4_CH1:
		 CCx_enable = TIM2_TO_TIM5_CC1E;
		break;
	case TIM4_CH2:
		 CCx_enable = TIM2_TO_TIM5_CC2E;
		break;
	case TIM4_CH3:
		 CCx_enable = TIM2_TO_TIM5_CC3E;
		break;
	case TIM4_CH4:
		 CCx_enable = TIM2_TO_TIM5_CC4E;
		break;
	default:
		return TIMx_incorrect;
		break;
	}


	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CCER);

	*TIM_REG_CCER &= ~CCx_enable;


	    return Success;

}

Status_code_t TIM4_PWM_Deinit(void){
	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM4_ADDRESS + TIMx_CR1);

	*TIM_REG_CR1 &= ~TIMx_CEN;
	TIMER_Clock(Disabled,TIMER_4);

	return Success;
}


////__________________________________PWM functions for TIM5____________________________________________
//void TIM5_set_global_PWM_ARR(uint32_t new_arr_value){
//	global_TIM5_ARR_count = new_arr_value;
//}
//
//Status_code_t TIM5_PWM_Init(TIM5_PWM_channel_select_t channel,PWM_mode_OCxM_t mode){
//
//	Status_code_t status = Success;
//	TIMx_register_offset_t capture_compare_mode_reg = TIMx_CCMR1;
//	Clean_PWM_channel_t clean_channel = Clean_pwm_channel_1_and_3;
//	TIM2_to_TIM5_CCER_t CCx_enable = TIM2_TO_TIM5_CC1E;
//	TIM2_to_TIM5_CCER_t CCx_polarity = TIM2_TO_TIM5_CC1P;
//	TIM1_to_TIM5_CCMR1_CCMR2_t CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
//	TIM1_to_TIM5_CCMR1_CCMR2_t CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;
//
//
//	switch(channel){
//	case TIM5_CH1:
//		 capture_compare_mode_reg = TIMx_CCMR1;
//		 clean_channel = Clean_pwm_channel_1_and_3;
//		 CCx_enable = TIM2_TO_TIM5_CC1E;
//		 CCx_polarity = TIM2_TO_TIM5_CC1P;
//		 CCMRx_selecction = TIM1_TO_TIM5_CC1S_and_CC3S;
//		 CCMRx_mode = TIM1_TO_TIM5_OC1M_and_OC3M;
//		break;
//	case TIM5_CH2:
//		 capture_compare_mode_reg = TIMx_CCMR1;
//		 clean_channel = Clean_pwm_channel_2_and_4;
//		 CCx_enable = TIM2_TO_TIM5_CC2E;
//		 CCx_polarity = TIM2_TO_TIM5_CC2P;
//		 CCMRx_selecction = TIM1_TO_TIM5_CC2S_and_CC4S;
//		 CCMRx_mode = TIM1_TO_TIM5_OC2M_and_OC4M;
//
//		break;
//	default:
//		return TIMx_incorrect;
//		break;
//	}
//
//
//	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
//	__IO uint32_t *TIM_REG_CCMRx = (__IO uint32_t *)(TIM5_ADDRESS + capture_compare_mode_reg);	//depending the channel selected
//	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CCER);
//
//
//
//	status = SetPinMode(Port_A, channel, Alt_func_mode); 				//always timer for PORT A for PWM TIM5
//
//	if(status!=Success){
//		return status;
//	}
//
//	TIMER_Clock(Enabled,TIMER_5);
//	GpioSetAlternativeFunction(Port_A, channel, TIM3_TIM4_TIM5); 		//alt 2
//
//	*TIM_REG_CCMRx &= ~(clean_channel); 								//clean the bits for the channel
//
//	*TIM_REG_CCMRx &= ~(Clear_two_bits<<CCMRx_selecction); 				//00 means PWM output
//
//	*TIM_REG_CCER &= ~CCx_polarity; 									// 0 to high polarity
//
//	*TIM_REG_CCMRx |= (mode<<CCMRx_mode); 								// configuring PWM mode
//
//	*TIM_REG_CCER &= ~CCx_enable; 										//deinit the channel compare
//
//	*TIM_REG_CCMRx |= TIM1_TO_TIM5_OC1PE_and_OC3PE;						//Output compare 1 pre-load enable
//	*TIM_REG_CR1 |= TIMx_ARPE; 											//save compare count
//
//
//	return status;
//
//}
//
//
//Status_code_t TIM5_PWM_start_custom_channel(TIM5_pwm_custom_parameters_t const PWM_Custom){
//
//
//	TIMx_register_offset_t capture_compare_reg = TIMx_CCR1;
//	TIM2_to_TIM5_CCER_t capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
//
//
//	switch(PWM_Custom.channel){
//	case TIM5_CH1:
//		 capture_compare_reg = TIMx_CCR1;
//		 capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
//		break;
//	case TIM5_CH2:
//		 capture_compare_reg = TIMx_CCR2;
//		 capture_compare_enb_reg = TIM2_TO_TIM5_CC2E;
//		break;
//	default:
//		return TIMx_incorrect;
//		break;
//	}
//	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_PSC);
//	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_ARR);
//	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
//	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CCER);
//	__IO uint32_t *TIM_REG_CCRx = (__IO uint32_t *)(TIM5_ADDRESS + capture_compare_reg);
//
//
//
//	*TIM_REG_PSC = (PWM_Custom.prescaler);
//	*TIM_REG_ARR = (PWM_Custom.Total_count_ARR - 1); 	//total count
//	*TIM_REG_CCRx= PWM_Custom.duty_count;  				//start count
//
//	*TIM_REG_CR1 |= TIMx_CEN;
//	*TIM_REG_CCER |= capture_compare_enb_reg; 			// enable output of channel
//
//
//	return Success;
//
//}
//
//
///*
//  NOTE:
//  to reach higher frequency modify the variable global_TIM5_ARR_count using the function TIM5_set_global_PWM_ARR
//
//  or use the function TIM5_PWM_start_channel_custom to personalizes the frequency and duty cycle
// */
//Status_code_t TIM5_PWM_start_channel(TIM5_pwm_auto_parameters_t const PWM){
//
//
//	TIMx_register_offset_t capture_compare_reg = TIMx_CCR1;
//	TIM2_to_TIM5_CCER_t capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
//
//	uint32_t miliseconds_duty = 0;
//
//	int32_t preescaler=0;
//
//
//	switch(PWM.channel){
//	case TIM5_CH1:
//		 capture_compare_reg = TIMx_CCR1;
//		 capture_compare_enb_reg = TIM2_TO_TIM5_CC1E;
//		break;
//	case TIM5_CH2:
//		 capture_compare_reg = TIMx_CCR2;
//		 capture_compare_enb_reg = TIM2_TO_TIM5_CC2E;
//		break;
//	default:
//		return TIMx_incorrect;
//		break;
//	}
//	__IO uint32_t *TIM_REG_PSC = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_PSC);
//	__IO uint32_t *TIM_REG_ARR = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_ARR);
//	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
//	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CCER);
//	__IO uint32_t *TIM_REG_CCRx = (__IO uint32_t *)(TIM5_ADDRESS + capture_compare_reg);
//
//
//	if(PWM.duty_cycle_percent<=0){
//		miliseconds_duty=0;
//	}else if(PWM.duty_cycle_percent>=100){
//		miliseconds_duty = global_TIM5_ARR_count;
//	}else{
//		miliseconds_duty = (uint32_t)((PWM.duty_cycle_percent * global_TIM5_ARR_count) / 100.0f);
//	}
//
//	preescaler = (int32_t)round((float)((BOARD_CLOCK / ((global_TIM5_ARR_count + 1) * PWM.frequency)) - 1));
//	if(preescaler<=0){
//		/*NOTE: if this happend try change the global_ARR_reg to a lower value*/
//		return PWM_frecuency_not_suported_in_this_mode;
//	}
//
//	*TIM_REG_PSC = preescaler;
//	*TIM_REG_ARR = (global_TIM5_ARR_count-1);
//	*TIM_REG_CCRx = miliseconds_duty;
//
//	*TIM_REG_CR1 |= TIMx_CEN; 						// Disable timer before configuration
//	*TIM_REG_CCER |= capture_compare_enb_reg; 		// enable output of channel
//
//
//	return Success;
//
//}
//
//Status_code_t TIM5_PWM_stop_channel(TIM5_PWM_channel_select_t channel){
//
//	TIM2_to_TIM5_CCER_t CCx_enable = TIM2_TO_TIM5_CC1E;
//
//	switch(channel){
//	case TIM5_CH1:
//		 CCx_enable = TIM2_TO_TIM5_CC1E;
//		break;
//	case TIM5_CH2:
//		 CCx_enable = TIM2_TO_TIM5_CC2E;
//		break;
//	default:
//		return TIMx_incorrect;
//		break;
//	}
//
//
//	__IO uint32_t *TIM_REG_CCER = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CCER);
//
//	*TIM_REG_CCER &= ~CCx_enable;
//
//	return Success;
//
//}
//
//Status_code_t TIM5_PWM_Deinit(void){
//	__IO uint32_t *TIM_REG_CR1 = (__IO uint32_t *)(TIM5_ADDRESS + TIMx_CR1);
//
//	*TIM_REG_CR1 &= ~TIMx_CEN;
//	TIMER_Clock(Disabled,TIMER_5);
//
//	return Success;
//}
//
