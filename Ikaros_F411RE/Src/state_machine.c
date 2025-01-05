/*
 * state_machine.c
 *
 *  Created on: Dec 22, 2024
 *      Author: sebas
 */

#include "state_machine.h"
#include "adc.h"
#include "gpios.h"
#include "timers.h"
#include "uart.h"
#include "watchdog.h"
#include "stepper_motor.h"
#include "keypad_4x4.h"



const SM_functions_list_t SM_Init_Protocols_next[2]={
		welcome_msg,
		init_Protocols,
};


const SM_functions_list_t SM_Welcome_msg_next = select_action_msg_menu1;


const SM_functions_list_t SM_Select_Action_msg_menu1_next[4]={
	send_UART2,
	send_UART1,
	select_action_msg_menu2,
	select_action_msg_menu1,

};


const SM_functions_list_t SM_Select_Action_msg_menu2_next[4]={

	read_UART1,
	reset_WWDG,
	select_action_msg_menu1,
	select_action_msg_menu2,

};


const SM_functions_list_t SM_Send_UART2_next = select_action_msg_menu1;

const SM_functions_list_t SM_Send_UART1_next = select_action_msg_menu1;

const SM_functions_list_t SM_Read_UART1_next = select_action_msg_menu2;
const SM_functions_list_t SM_Reset_WWDG_next = wait_reset;
const SM_functions_list_t SM_Wait_Reset_next = wait_reset;


SM_parameters_t const State_machine_map[SM_MAX_STATES] =
{
    {SM_Init_Protocols,SM_Init_Protocols_next},
    {SM_Welcome_msg,&SM_Welcome_msg_next},
    {SM_Select_Action_msg_menu1,SM_Select_Action_msg_menu1_next},
    {SM_Select_Action_msg_menu2,SM_Select_Action_msg_menu2_next},
	{SM_Send_UART2,&SM_Send_UART2_next},
    {SM_Send_UART1,&SM_Send_UART1_next},
    {SM_Read_UART1,&SM_Read_UART1_next},
    {SM_Reset_WWDG,&SM_Reset_WWDG_next},
    {SM_Wait_Reset,&SM_Wait_Reset_next},


};

//pendiente lo del LCD mesagge con uart 2 interrupcion

SM_next_state_t SM_Init_Protocols(void){
	Status_code_t status = Success;

	usart_config_t usart_config={
			9600,
			Asynchronous, //pendiente  y las demas configuraciones
			NULL,
			enable_TX_and_RX,
			None,
			Data_8_bits,
			Stop_1_bits
	};

	WWDG_config_t WWDog ={
			120, //ms
			50,  //allow to refresh after 50 ms has passed but before 120 ms
			enabled_interrupt //eneable interrupt
	};

	status = Init_UART2_RX_Interrupt(usart_config);
	status |= Init_UART1(usart_config);

	status |= lcd_init(lcd_PortC);
	Delay(300000);
	status |= Init_keypad(keypad_PortA);

	status |= TIM3_Init(WWDog.W_time + 10); //set the window time + 10 ms
	status |= TIM5_Init(2000000); //set TIM4 to 2s


	if(status != Success){
		return Next_state_1;
	}

	status = Init_Win_Watchdog(WWDog);
	Win_Watchdog_control(reload_food);
	TIM3_Start();


	if(status!= Success){
		return Next_state_1;
	}

	return Next_state_0;

}

SM_next_state_t SM_Welcome_msg(void){
	lcd_printXY(0, 0," Welcome to my  ", 16);
	lcd_printXY(0, 1," State Machine  ", 16);
	state_to_print(welcome_msg);

	Delay(2000000);

	return Next_state_0;

}

SM_next_state_t SM_Select_Action_msg_menu1(void){

	Status_code_t status = Success;
	uint32_t answere =0;
	lcd_printXY(0, 0,"1-Write UART2 : ", 16);
	lcd_printXY(0, 1,"2-Write UART1 >6", 16);
	state_to_print(select_action_msg_menu1);
	TIM5_Stop();
	answere = print_keypad(keypad_PortA, &status,15,0);
	TIM5_Start();

	if(answere == 6){
		return Next_state_2;
	}else if(answere == 1){
		return Next_state_0;
	}
	else if(answere == 2){
		return Next_state_1;

	}else if(answere == 0 && status != Success){
		return Next_state_3;

	}else{
		lcd_printXY(0, 0," INVALID OPTION ", 16);
		lcd_printXY(0, 1,"                ", 16);
		state_to_print(8);

		Delay(1000000);
		return Next_state_3;
	}
}

SM_next_state_t SM_Select_Action_msg_menu2(void){
	uint32_t answere =0;
	Status_code_t status = Success;
	lcd_printXY(0, 0,"3-Read UART1  : ", 16);
	lcd_printXY(0, 1,"4-Reset WWDG  <8", 16);
	state_to_print(select_action_msg_menu2);

	TIM5_Stop();
	answere = print_keypad(keypad_PortA, &status,15,0);
	TIM5_Start();

	if(answere == 8){
		return Next_state_2;

	}else if(answere == 3){
		return Next_state_0;

	}
	else if(answere == 4){
		return Next_state_1;

	}else if(answere == 0 && status != Success){
		return Next_state_3;

	}else{
		lcd_printXY(0, 0," INVALID OPTION ", 16);
		lcd_printXY(0, 1,"                ", 16);
		state_to_print(8);

		Delay(1000000);
		return Next_state_3;

	}


}

SM_next_state_t SM_Send_UART2(void){
	Status_code_t status = Success;
	lcd_printXY(0, 0," Write to UART2 ", 16);
	lcd_printXY(0, 1,"                ", 16);
	state_to_print(send_UART2);

	status = UART2_Write("I am \"UART 2\" From STM32\r\n",26 , 1000);
	Delay(1000000);
	return Next_state_0;

}

SM_next_state_t SM_Send_UART1(void){
	Status_code_t status = Success;
	lcd_printXY(0, 0," Write to UART1 ", 16);
	lcd_printXY(0, 1,"                ", 16);
	state_to_print(send_UART1);

	status = UART1_Write("I am \"UART 1\" From STM32\r\n",26 , 1000);
	Delay(1000000);
	return Next_state_0;

}

SM_next_state_t SM_Read_UART1(void){
	Status_code_t status = Success;

	uint8_t data[50]={0};
	uint32_t data_lenght=0;
	lcd_printXY(0, 0,"Received Data:  ", 16);
	lcd_printXY(0, 1,"                ", 16);
	state_to_print(read_UART1);

	TIM5_Stop();

	status = UART1_Read(data, &data_lenght, 3000);

	if(status != Success){
		lcd_printXY(0, 1,"Nothing",7);
	}else{
		lcd_printXY(0, 1,data,data_lenght);
	}
	Delay(1000000);

	TIM5_Start();

	return Next_state_0;

}

SM_next_state_t SM_Reset_WWDG(void){

	lcd_printXY(0, 0,"  WWDG RESET!!  ", 16);
	lcd_printXY(0, 1,"                ", 16);
	state_to_print(reset_WWDG);

	Delay(1000000);
	TIM3_Stop();

	return Next_state_0;

}

SM_next_state_t SM_Wait_Reset(void){

	return Next_state_0;

}


static SM_functions_list_t run_funtion = init_Protocols;
static SM_next_state_t next_move= Next_state_0;


void runMachine(void){


    while(1){

    	next_move = State_machine_map[run_funtion].run_fun();
    	run_funtion = State_machine_map[run_funtion].next_state[next_move];

    }

}


