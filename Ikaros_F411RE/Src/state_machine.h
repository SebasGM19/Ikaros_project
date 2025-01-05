/*
 * state_machine.h
 *
 *  Created on: Dec 22, 2024
 *      Author: sebas
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include "system_settings.h"


#define SM_MAX_STATES (9)

typedef enum{
    Next_state_0,
    Next_state_1,
    Next_state_2,
    Next_state_3,
    Next_state_4,
    Next_state_5,
    Next_state_6,
    Next_state_7,


}SM_next_state_t;


typedef enum{
	init_Protocols,
	welcome_msg,
	select_action_msg_menu1,
	select_action_msg_menu2,
	send_UART2,
	send_UART1,
	read_UART1,
	reset_WWDG,
	wait_reset,

}SM_functions_list_t;

//
//const SM_functions_list_t SM_Init_Protocols_next[2]={
//		welcome_msg,
//		init_Protocols,
//};
//
//
//const SM_functions_list_t SM_Welcome_msg_next = select_action_msg;
//
//
//const SM_functions_list_t SM_Select_Action_msg_next[5]={
//	send_UART2,
//	send_UART1,
//	read_UART1,
//	reset_WWDG,
//	select_action_msg,
//
//};
//
//
//const SM_functions_list_t SM_Send_UART2_next = select_action_msg;
//
//const SM_functions_list_t SM_Send_UART1_next = select_action_msg;
//
//const SM_functions_list_t SM_Read_UART1_next = select_action_msg;
//const SM_functions_list_t SM_Reset_WWDG_next = wait_reset;
//const SM_functions_list_t SM_Wait_Reset_next = wait_reset;



typedef struct{
	SM_next_state_t (*run_fun)();
    const SM_functions_list_t *next_state;
}SM_parameters_t;




SM_next_state_t SM_Init_Protocols(void);
SM_next_state_t SM_Welcome_msg(void);
SM_next_state_t SM_Select_Action_msg_menu1(void);
SM_next_state_t SM_Select_Action_msg_menu2(void);
SM_next_state_t SM_Send_UART2(void);
SM_next_state_t SM_Send_UART1(void);
SM_next_state_t SM_Read_UART1(void);
SM_next_state_t SM_Reset_WWDG(void);
SM_next_state_t SM_Wait_Reset(void);

void runMachine(void);



//SM_parameters_t const State_machine_map[SM_MAX_STATES] =
//{
//    {SM_Init_Protocols,SM_Init_Protocols_next},
//    {SM_Welcome_msg,&SM_Welcome_msg_next},
//    {SM_Select_Action_msg,SM_Select_Action_msg_next},
//    {SM_Send_UART2,&SM_Send_UART2_next},
//    {SM_Send_UART1,&SM_Send_UART1_next},
//    {SM_Read_UART1,&SM_Read_UART1_next},
//    {SM_Reset_WWDG,&SM_Reset_WWDG_next},
//    {SM_Wait_Reset,&SM_Wait_Reset_next},
//
//
//};



#endif /* STATE_MACHINE_H_ */
