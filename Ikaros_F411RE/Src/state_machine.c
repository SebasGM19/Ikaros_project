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

#include "../Inc/Drivers/i2c.h"

#include "../Inc/Peripherals/SHT20.h"
#include "../Inc/Peripherals/BMI160.h"

#include "SRAM23LCV512.h"
#include "spi_i2s.h"
#include "grove_ultrasonic.h"


#define MAX_PAGES_TO_SAVE	(5U)

const SM_functions_list_t SM_Init_Watchdog_next = init_peripherals;

const SM_functions_list_t SM_Init_Peripherals_next[2]={
		config_sensors,
		init_peripherals,
};

const SM_functions_list_t SM_Config_Sensors_next[3] ={
		wait_exti,
		config_sensors,
		deinit_peripherals,
};

const SM_functions_list_t SM_Wait_EXTI_next[2] ={
		read_sensors,
		wait_exti,
};


const SM_functions_list_t SM_Read_Sensors_next[3]={
	save_data,
	read_sensors,
	deinit_peripherals
};

const SM_functions_list_t SM_Save_Data_next[3] ={
	wait_exti,
	data_format,
	save_data,
};

const SM_functions_list_t SM_Data_Format_next[3] ={
		send_msg,
		data_format,
		clean_memory,
};


const SM_functions_list_t SM_Clean_Memory_next[3]={
	wait_exti,
	clean_memory,
	deinit_peripherals
};

const SM_functions_list_t SM_Send_Msg_next[2]={
	clean_memory,
	send_msg,
};

const SM_functions_list_t SM_Deinit_peripherals_next = init_peripherals;


SM_parameters_t const State_machine_map[SM_MAX_STATES] =
{
    {SM_Watchdog_Init,&SM_Init_Watchdog_next},
    {SM_Init_Peripherals,SM_Init_Peripherals_next},
    {SM_Config_Sensors,SM_Config_Sensors_next},

    {SM_Wait_EXTI,SM_Wait_EXTI_next},
	{SM_Read_Sensors, SM_Read_Sensors_next},
    {SM_Save_Data,SM_Save_Data_next},

	{SM_Data_Format,SM_Data_Format_next},
	{SM_Clean_Memory, SM_Clean_Memory_next},

    {SM_Send_Msg,SM_Send_Msg_next},
    {SM_Deinit_Peripherals,&SM_Deinit_peripherals_next},

};




SM_next_state_t SM_Watchdog_Init(void)
{

	Init_Ind_Watchdog(reload_32768ms);
	Ind_Watchdog_control(reload_food);

	return Next_state_0;

}

SM_next_state_t SM_Init_Peripherals(void)
{
	Status_code_t status = Success;


	//GPIO DEFINDED AS EXTI
	GPIO_Init_EXTI15_To_EXTI10(EXTI_Port_C, Pin_13, Falling_edge); //user button to trig when to save data
	GPIO_Enable_EXTI15_To_EXTI10(Pin_13);

	//SPI CONFIGURATION
	//GPIO for SPI SLAVE
	SetPinMode(Port_C, Pin_8, Output);
	GPIO_DigitalWrite(Port_C, Pin_8, High);
	SPI_config_t SPI_SRAM_config = {
			SPI_CPHA_first_edge,
			SPI_CPOL_low_idle,
			SPI_preescaler_2, //defined for 4MHz clock frecuency
			SPI_MSB_trans_first, //datasheet of the sensor indicates this
			SPI_8_bit_format,
			SPI_2_lines_unidirectional,
			SPI_motorola_mode
	};
	status = SPI1_Init(&SPI_SRAM_config);


	//UART2 CONFIGURATION
	usart_config_t UART2_config =
	{
		9600,
		Asynchronous,
		NULL,
		enable_TX_and_RX,
		None,
		Data_8_bits,
		Stop_1_bits

	};
	status |= UART2_Init(UART2_config);


	if(status != Success){

		return Next_state_1;
	}

	Ind_Watchdog_control(reload_food);
	return Next_state_0;

}



uint32_t config_tries =0;

SM_next_state_t SM_Config_Sensors(void)
{
	Status_code_t status = Success;

	//configuracion del ADC de la temperatura
	status = ADC_Init_Temperature_Sensor(RES_12_bits);

	GPIO_DigitalWrite(Port_C, Pin_8, Low);
	status |= SRAM23LCV_Set_Mode(SPI1_I2S1_Alt, SRAM23LCV_Page_mode);
	GPIO_DigitalWrite(Port_C, Pin_8, High);

	status |= Grove_Init(Port_C, Pin_9);


	Ind_Watchdog_control(reload_food);

	if(config_tries > 3){
		config_tries=0;
		return Next_state_2;
	}

	if(status!= Success){

		config_tries++;
			return Next_state_1;
	}

	config_tries=0;
	return Next_state_0;

}


uint32_t saved_times = 0;
SM_next_state_t SM_Wait_EXTI(void)
{

	Ind_Watchdog_control(reload_food);

	if(get_EXTI_times() > saved_times){

		saved_times++;
		return Next_state_0;

	}

	return Next_state_1;


}

uint8_t read_sensor_tries = 0;
uint32_t distance_read = 0;
uint32_t temperature_val = 0;
SM_next_state_t SM_Read_Sensors(void)
{
		Status_code_t status = Success;

		Ind_Watchdog_control(reload_food);

		if(read_sensor_tries >= 3){
			read_sensor_tries = 0;
			return Next_state_2;
		}

		status = Grove_Read_Distance(&distance_read);

		if(status!= Success){

			read_sensor_tries++;
			return Next_state_1;
		}

		status = ADC_Read_Temperature(&temperature_val);

		if(status != Success){
			read_sensor_tries++;
			return Next_state_1;
		}

		read_sensor_tries = 0;
		return Next_state_0;
}



uint8_t msg_to_send[MAX_MSG_LENGHT] = {0};

uint8_t string_start_json[]={"{"}; //start json
uint8_t string_end_json[]={"}"}; //end json

uint8_t string_square_bracket_start[]={"["}; //end json
uint8_t string_square_bracket_end[]={"]"}; //end json

uint8_t string_colon[]={":"};
uint8_t string_coma[]={","};
uint8_t string_new_line[]={"\n"}; //end json
uint8_t string_new_line_carriage_return[]={"\r\n"}; //end json

uint8_t string_payload[]={"{\"Payload\":["};

uint8_t string_distance[]={"{\"Dist\":"};
uint8_t string_temperature[]={"\"Temp\":"};


uint16_t page_count = 1;
uint32_t save_tries = 0;

SM_next_state_t SM_Save_Data(void)
{
	Status_code_t status = Success;

	uint8_t page[32] = {0};
	uint8_t aux_data_buff[10] = {0};

	memset(aux_data_buff, '\0', sizeof(aux_data_buff));
	memset(page, '\0', sizeof(page));


    strcpy((char *)(page), (const char *)(string_distance));
	utoa(distance_read, (char *)aux_data_buff, 10);
    strncat((char *)(page), (const char *)(aux_data_buff), strlen((const char *)aux_data_buff));//guardamos el dato de float en cadena
    strcat((char *)(page), (const char *)(string_coma));
	memset(aux_data_buff, '\0', sizeof(aux_data_buff));


    strcat((char *)(page), (const char *)(string_temperature));
	utoa(temperature_val, (char *)aux_data_buff, 10);
    strncat((char *)(page), (const char *)(aux_data_buff), strlen((const char *)aux_data_buff));
	strcat((char *)(page), (const char *)(string_end_json));

	GPIO_DigitalWrite(Port_C, Pin_8, Low);
	status = SRAM23LCV_Write_Page(SPI1_I2S1_Alt, page_count, page, strlen((const char *)page));
	GPIO_DigitalWrite(Port_C, Pin_8, High);


	Ind_Watchdog_control(reload_food);
	if(save_tries >= 3){
		save_tries = 0;
		return Next_state_0;
	}

	if(status != Success){
//		Delay(100000);
		save_tries++;
		return Next_state_2;
	}

	save_tries = 0;
	if(page_count >= MAX_PAGES_TO_SAVE){
		page_count = 1;
		return Next_state_1;

	}
	page_count++;
	return Next_state_0;

}




#define DATA_BUFF_MAX_SIZE          (15)

uint8_t data_format_count = 0;
SM_next_state_t SM_Data_Format(void)
{
	Status_code_t status = Success;

    uint8_t temp_buff[DATA_BUFF_MAX_SIZE] = {0};
    uint8_t Page_buff[32] = {0};
    uint16_t count = 1;

	memset(msg_to_send, '\0', sizeof(msg_to_send));
	memset(temp_buff, '\0', sizeof(temp_buff));
	memset(Page_buff, '\0', sizeof(Page_buff));

    strcpy((char *)(msg_to_send), (const char *)(string_payload));


    while(count <= MAX_PAGES_TO_SAVE && status == Success){
    	GPIO_DigitalWrite(Port_C, Pin_8, Low);
    	status = SRAM23LCV_Read_Page(SPI1_I2S1_Alt, count ,Page_buff);
    	GPIO_DigitalWrite(Port_C, Pin_8, High);

        strncat((char *)(msg_to_send), (const char *)(Page_buff), strlen((const char *)Page_buff));//guardamos el dato de float en cadena
        if(count<MAX_PAGES_TO_SAVE){
        	strcat((char *)(msg_to_send), (const char *)(string_coma));
        }
        count++;
    }

    strcat((char *)(msg_to_send), (const char *)(string_square_bracket_end));
    strcat((char *)(msg_to_send), (const char *)(string_end_json));
    strcat((char *)(msg_to_send), (const char *)(string_new_line));
    strcat((char *)(msg_to_send), (const char *)(string_new_line));

	Ind_Watchdog_control(reload_food);

	if(data_format_count >= 3){
		data_format_count =0;
    	return Next_state_2;

	}

    if(status != Success){
    	data_format_count++;
//    	Delay(100000);
    	return Next_state_1;
    }

    data_format_count=0;
	return Next_state_0;
}


uint8_t clean_memory_count =0;
SM_next_state_t SM_Clean_Memory(void)
{
	Status_code_t status = Success;
	uint8_t page[32] = {0};
	memset(page, '\0', sizeof(page));
    uint16_t count = 1;


    while(count <= MAX_PAGES_TO_SAVE && status == Success){
    	GPIO_DigitalWrite(Port_C, Pin_8, Low);
		status = SRAM23LCV_Write_Page(SPI1_I2S1_Alt, count, page, strlen((const char *)page));
		GPIO_DigitalWrite(Port_C, Pin_8, High);

		Delay(100);
		count++;
	}


	clear_EXTI_times();
	saved_times = 0;

	Ind_Watchdog_control(reload_food);
	if(clean_memory_count >= 3){
		clean_memory_count =0;
		return Next_state_2;

	}
	if(status != Success){
		clean_memory_count++;
//		Delay(100000);
		return Next_state_1;

	}

	clean_memory_count = 0;
	return Next_state_0;

}


uint8_t send_msg_retries =0;
SM_next_state_t SM_Send_Msg(void)
{
	Status_code_t status = Success;

	status = UART2_Write(msg_to_send, strlen((const char *)msg_to_send) , 1000);
	Delay(100);


	Ind_Watchdog_control(reload_food);
	if(send_msg_retries >= 3){
		send_msg_retries = 0;
		return Next_state_0;
	}

	if( status != Success){
		send_msg_retries++;
		return Next_state_1;

	}

	send_msg_retries=0;
	return Next_state_0;

}

SM_next_state_t SM_Deinit_Peripherals(void)
{

	SPI1_Deinit();


	Ind_Watchdog_control(reload_food);
	return Next_state_0;

}




static SM_functions_list_t run_funtion = init_watchdog;
static SM_next_state_t next_move = Next_state_0;

void StartMachine(void){


    while(1){

    	next_move = State_machine_map[run_funtion].run_fun();
    	run_funtion = State_machine_map[run_funtion].next_state[next_move];

    }

}


