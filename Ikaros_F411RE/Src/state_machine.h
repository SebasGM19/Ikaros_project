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
#define MAX_MSG_LENGHT	(1500)

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
	init_watchdog,
	init_Protocols,
	config_sensors,
	read_sensors,
	data_format,
	send_msg,
	deinit_protocols,

}SM_functions_list_t;


typedef struct{
	SM_next_state_t (*run_fun)();
    const SM_functions_list_t *next_state;
}SM_parameters_t;




SM_next_state_t SM_Watchdog_Init(void);
SM_next_state_t SM_Init_Protocols(void);
SM_next_state_t SM_Config_Sensors(void);
SM_next_state_t SM_Read_Sensors(void);
SM_next_state_t SM_Data_Format(void);
SM_next_state_t SM_Send_Msg(void);
SM_next_state_t SM_Deinit_protocols(void);


void StartMachine(void);





#endif /* STATE_MACHINE_H_ */
