/*
 * grove_ultrasonic.c
 *
 *  Created on: Dec 27, 2025
 *      Author: sebas
 */

#include "grove_ultrasonic.h"

#define HANDLER_5_SET		(1U)
const float soundSpeed = 340.0f;
#define CMTOMETERS			(100U)
#define MMTOMETERS			(1000U)
#define MICROS 				(1000000U)
#define ULTRASONIC_TIMEOUT	(2000U)

static Status_code_t Grove_Wait_Rising_Edge(void);
static Status_code_t Grove_Wait_Falling_Edge(void);
static void Grove_Send_trig(void);

//TO USE THIS PERIPHERAL IS NECESARI TO CONFIGURE THE TIM5 HANDLER WITH THE FOLLOWING CODE

//#########use this function in TIM5 handler#############

//volatile uint32_t Grove_ultrasoinic_count = 0;
//
//uint32_t Grove_Get_Count(void){
//	return Grove_ultrasoinic_count;
//}
//
//void Grove_Clear_Count(void){
//	Grove_ultrasoinic_count = 0;
//}
//
//void TIM5_HANDLER(void){
//	TIMER_cleanCountFlag(TIM5_ADDRESS);
//	if(reset_tim5_flag){
//	/*Develop all the code to be executed in a second thread down here*/
//		Grove_ultrasoinic_count++;
//
//	}else{reset_tim5_flag = !reset_tim5_flag;}
//}


//################# EXT Functions change ###############

//volatile bool Grove_EXT_flag = false;
//
//bool Grove_Get_EXT_flag_status(void){
//	return Grove_EXT_flag;
//}
//
//void Grove_Clear_EXT_flag_status(void){
//	Grove_EXT_flag = false;
//}
//
//void EXTI9_5_HANDLER(void){
//	/*Develop your interruption code from here*/
//
//	Grove_EXT_flag = true;
//
//
//	/*To here*/
//	GPIO_EXTI_Clean_Group_Of_Flag(Pin_5,Clear_five_bits);
//}



#if HANDLER_5_SET

static Set_Port_t GPIO_PORT = Port_A;
static Pin_number_t GPIO_PIN = Pin_0;
static GPIO_Exti_Port_t EXT_PORT =EXTI_Port_A;


Status_code_t Grove_Init(Set_Port_t PORT, Pin_number_t GPIO){


	if(GPIO > Pin_9 || GPIO < Pin_5){
		return EXTI_Pin_Not_Allowed;
	}

	switch(PORT){
		case Port_A:
			EXT_PORT =EXTI_Port_A;
			break;
		case Port_B:
			EXT_PORT =EXTI_Port_B;
			break;
		case Port_C:
			EXT_PORT =EXTI_Port_C;
			break;
		case Port_D:
			EXT_PORT =EXTI_Port_D;
			break;
		default:
			return WrongParameter;
	}

	GPIO_PORT = PORT;
	GPIO_PIN = GPIO;

	return Success;
}


Status_code_t Grove_Read_Distance(uint32_t* distance_mm){

	uint32_t times_count = 0;
	uint32_t echo = 0;
	Status_code_t status = Success;

	Grove_Send_trig();
	status = Grove_Wait_Rising_Edge();

	if(status != Success){
		return status;
	}

	TIM5_Init(MIN_TIME_TIM5_AND_TIM2);
	TIM5_Start();

	status = Grove_Wait_Falling_Edge();

	TIM5_Stop();

	if(status != Success){
		TIM5_Deinit();
		Grove_Clear_Count();

		return status;
	}

	times_count = Grove_Get_Count();

	echo = (uint32_t)(times_count * MIN_TIME_TIM5_AND_TIM2);

	(*distance_mm) = (uint32_t)((echo*((soundSpeed*MMTOMETERS)/MICROS))/2.0f);

	Grove_Clear_Count();

	TIM5_Deinit();

	return Success;

}

void Grove_Deinit(void){

	SetPinMode(GPIO_PORT, GPIO_PIN, Output);


}



static Status_code_t Grove_Wait_Rising_Edge(void){

	TIM11_Init(ULTRASONIC_TIMEOUT);
	TIM11_Start();

	GPIO_Init_EXTI9_To_EXTI5(EXT_PORT, GPIO_PIN, Rising_edge);
	GPIO_Enable_EXTI9_To_EXTI5(GPIO_PIN);

	while(!Grove_Get_EXT_flag_status() && !TIM11_GET_interrupt_flag_status());

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		return Timeout;
	}

	Grove_Clear_EXT_flag_status();

	GPIO_Disable_EXTI9_To_EXTI5(GPIO_PIN);
	GPIO_Deinit_EXTI9_To_EXTI5(GPIO_PIN);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	return Success;

}


static Status_code_t Grove_Wait_Falling_Edge(void){

	TIM11_Init(ULTRASONIC_TIMEOUT);
	TIM11_Start();

	GPIO_Init_EXTI9_To_EXTI5(EXT_PORT, GPIO_PIN, Falling_edge);
	GPIO_Enable_EXTI9_To_EXTI5(GPIO_PIN);

	while(!Grove_Get_EXT_flag_status() && !TIM11_GET_interrupt_flag_status());


	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		return Timeout;
	}

	Grove_Clear_EXT_flag_status();

	GPIO_Disable_EXTI9_To_EXTI5(GPIO_PIN);
	GPIO_Deinit_EXTI9_To_EXTI5(GPIO_PIN);

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	return Success;
}


static void Grove_Send_trig(void){

	SetPinMode(GPIO_PORT, GPIO_PIN, Output);

	TIM5_Init(15);

	GPIO_DigitalWrite(GPIO_PORT, GPIO_PIN, High);

	TIM5_Start();

	while(Grove_Get_Count() <= 0);

	TIM5_Stop();

	GPIO_DigitalWrite(GPIO_PORT, GPIO_PIN, Low);

	Grove_Clear_Count();

	TIM5_Deinit();


}

#endif
