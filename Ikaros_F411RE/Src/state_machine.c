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

const SM_functions_list_t SM_Init_Watchdog_next = init_Protocols;

const SM_functions_list_t SM_Init_Protocols_next[2]={
		config_sensors,
		init_Protocols,
};

const SM_functions_list_t SM_Config_Sensors_next[3] ={
		read_sensors,
		config_sensors,
		deinit_protocols,
};

const SM_functions_list_t SM_Read_Sensors_next[3]={
	data_format,
	read_sensors,
	deinit_protocols
};

const SM_functions_list_t SM_Data_Format_next = send_msg;

const SM_functions_list_t SM_Send_Msg_next[3]={
	read_sensors,
	send_msg,
	deinit_protocols,
};

const SM_functions_list_t SM_Deinit_protocols_next = init_Protocols;


SM_parameters_t const State_machine_map[SM_MAX_STATES] =
{
    {SM_Watchdog_Init,&SM_Init_Watchdog_next},
    {SM_Init_Protocols,SM_Init_Protocols_next},
    {SM_Config_Sensors,SM_Config_Sensors_next},
    {SM_Read_Sensors,SM_Read_Sensors_next},
	{SM_Data_Format,&SM_Data_Format_next},
    {SM_Send_Msg,SM_Send_Msg_next},
    {SM_Deinit_protocols,&SM_Deinit_protocols_next},

};




SM_next_state_t SM_Watchdog_Init(void)
{

	Init_Ind_Watchdog(reload_32768ms);
	Ind_Watchdog_control(reload_food);

	return Next_state_0;

}

SM_next_state_t SM_Init_Protocols(void)
{
	Status_code_t status = Success;
	i2c_config_parameters_t I2C_configurations;

	usart_config_t usart_config={
			115200,
			Asynchronous,
			NULL,
			enable_TX_and_RX,
			None,
			Data_8_bits,
			Stop_1_bits
	};

	status = UART2_Init(usart_config);

	I2C_configurations.baudrate = FastMode_400Kbps;
	I2C_configurations.duty = duty_2_1;

	status |= I2C1_Init_Master(I2C_configurations);


	if(status != Success){

		Ind_Watchdog_control(reload_food);
		return Next_state_1;
	}

	Ind_Watchdog_control(reload_food);
	return Next_state_0;

}


BMI160_TotalData_t Data;

uint32_t config_tries =0;
SM_next_state_t SM_Config_Sensors(void)
{
	Status_code_t status = Success;

	BMI160_init_parameters config;
	config.Acc_power_mode = Acc_Normal;
	config.Gyr_power_mode = Gyr_Normal;
	config.Mag_power_mode = Mag_Normal;
	config.Acc_res = Acc_2g;
	config.Gyr_res = Gyr_250s;

	status = BMI160_Init(I2C1_Alt,&config);

	if(status!= Success){

		if(config_tries > 3){
			config_tries=0;
			Ind_Watchdog_control(reload_food);
			return Next_state_2;
		}
		config_tries++;
		Ind_Watchdog_control(reload_food);
		return Next_state_1;
	}

	config_tries=0;
	Ind_Watchdog_control(reload_food);
	return Next_state_0;

}

uint8_t read_sensor_tries =0;
int16_t temperature = 0;
int16_t temperaturesht20 = 0;
uint16_t humedadsht20 = 0;




SM_next_state_t SM_Read_Sensors(void)
{
		Status_code_t status = Success;
		data_status_t accelerometer = Not_ready;
		data_status_t Gyroscope 	= Not_ready;
		data_status_t magnetomer 	= Not_ready;
		data_status_t FOC		 	= Not_ready;

		status = BMI160_Data_Status(I2C1_Alt, &accelerometer, &Gyroscope, &magnetomer, &FOC);

		if(status!= Success && Gyroscope != Not_ready && accelerometer != Not_ready){

			read_sensor_tries++;
			if(read_sensor_tries >= 3){
				read_sensor_tries=0;
				Ind_Watchdog_control(reload_food);
				return Next_state_2;
			}
			Ind_Watchdog_control(reload_food);
			return Next_state_1;
		}

		status = BMI160_Get_TotalData(I2C1_Alt, &Data);

		if(status!= Success){
			read_sensor_tries++;
			if(read_sensor_tries>= 3){
				read_sensor_tries=0;
				Ind_Watchdog_control(reload_food);
				return Next_state_0;
			}
			Ind_Watchdog_control(reload_food);
			return Next_state_1;
		}
		read_sensor_tries=0;
		Ind_Watchdog_control(reload_food);
		return Next_state_0;
}


uint8_t msg_to_send[MAX_MSG_LENGHT] = {0};

uint8_t string_start_json[]={"{"}; //start json
uint8_t string_end_json[]={"}"}; //end json
uint8_t string_colon[]={":"};
uint8_t string_coma[]={","};
uint8_t string_new_line[]={"\n"}; //end json

uint8_t string_Gyro_X[]={"\"GyroX\": "}; //start json
uint8_t string_Gyro_Y[]={"\"GyroY\": "}; //start json
uint8_t string_Gyro_Z[]={"\"GyroZ\": "}; //start json

//uint8_t string_Mag_X[]={"\"MagX\": "}; //start json
//uint8_t string_Mag_Y[]={"\"MagY\": "}; //start json
//uint8_t string_Mag_Z[]={"\"MagZ\": "}; //start json

uint8_t string_Acc_X[]={"\"AccX\": "}; //start json
uint8_t string_Acc_Y[]={"\"AccY\": "}; //start json
uint8_t string_Acc_Z[]={"\"AccZ\": "}; //start json


uint8_t string_temperature[]={"\"Temperature\": "}; //start json
//uint8_t string_hall_res[]={"\"HallR\": "}; //start json



#define DATA_BUFF_MAX_SIZE          (15)

SM_next_state_t SM_Data_Format(void)
{
    uint8_t temp_buff[DATA_BUFF_MAX_SIZE] = {0};
	memset(msg_to_send, '\0', sizeof(msg_to_send));
	memset(temp_buff, '\0', sizeof(temp_buff));

    strcpy((char *)(msg_to_send), (const char *)(string_start_json));

    //___________INICIO_info del giroscopio_______________
    strcat((char *)(msg_to_send), (const char *)(string_Gyro_X));
    ftoa(Data.gyroscope_X, temp_buff, 2); //dos decimales despues
    strncat((char *)(msg_to_send), (const char *)(temp_buff), strlen((const char *)temp_buff));//guardamos el dato de float en cadena
	memset(temp_buff, '\0', sizeof(temp_buff));
    strcat((char *)(msg_to_send), (const char *)(string_coma));

    strcat((char *)(msg_to_send), (const char *)(string_Gyro_Y));
    ftoa(Data.gyroscope_Y, temp_buff, 2); //dos decimales despues
    strncat((char *)(msg_to_send), (const char *)(temp_buff), strlen((const char *)temp_buff));//guardamos el dato de float en cadena
	memset(temp_buff, '\0', sizeof(temp_buff));
    strcat((char *)(msg_to_send), (const char *)(string_coma));

    strcat((char *)(msg_to_send), (const char *)(string_Gyro_Z));
    ftoa(Data.gyroscope_Z, temp_buff, 2); //dos decimales despues
    strncat((char *)(msg_to_send), (const char *)(temp_buff), strlen((const char *)temp_buff));//guardamos el dato de float en cadena
	memset(temp_buff, '\0', sizeof(temp_buff));
    strcat((char *)(msg_to_send), (const char *)(string_coma));
    //___________FIN_info del giroscopio_______________

    //__________INICIO_info de accelerometer______________
    strcat((char *)(msg_to_send), (const char *)(string_Acc_X));
    ftoa(Data.accelerometer_X, temp_buff, 2); //dos decimales despues
    strncat((char *)(msg_to_send), (const char *)(temp_buff), strlen((const char *)temp_buff));//guardamos el dato de float en cadena
	memset(temp_buff, '\0', sizeof(temp_buff));
    strcat((char *)(msg_to_send), (const char *)(string_coma));

    strcat((char *)(msg_to_send), (const char *)(string_Acc_Y));
    ftoa(Data.accelerometer_Y, temp_buff, 2); //dos decimales despues
    strncat((char *)(msg_to_send), (const char *)(temp_buff), strlen((const char *)temp_buff));//guardamos el dato de float en cadena
	memset(temp_buff, '\0', sizeof(temp_buff));
    strcat((char *)(msg_to_send), (const char *)(string_coma));

    strcat((char *)(msg_to_send), (const char *)(string_Acc_Z));
    ftoa(Data.accelerometer_Z, temp_buff, 2); //dos decimales despues
    strncat((char *)(msg_to_send), (const char *)(temp_buff), strlen((const char *)temp_buff));//guardamos el dato de float en cadena
	memset(temp_buff, '\0', sizeof(temp_buff));
    //__________FIN info de accelerometer______________



//    strcat((char *)(msg_to_send), (const char *)(string_coma));
//    strcat((char *)(msg_to_send), (const char *)(string_temperature));
//    itoa(temperature,(char *)(temp_buff), DECIMAL_BASE); //dos decimales despues
//    strncat((char *)(msg_to_send), (const char *)(temp_buff), strlen((const char *)temp_buff));//guardamos el dato de float en cadena
//	memset(temp_buff, '\0', sizeof(temp_buff));
//
    strcat((char *)(msg_to_send), (const char *)(string_end_json));
    strcat((char *)(msg_to_send), (const char *)(string_new_line));


	Ind_Watchdog_control(reload_food);
	return Next_state_0;
}

uint8_t send_msg_retries =0;
SM_next_state_t SM_Send_Msg(void)
{
	Status_code_t status = Success;

	status = UART2_Write(msg_to_send, strlen((const char *)msg_to_send) , 1000);
	Delay(500);

	if( status != Success){

		if(send_msg_retries >= 3){
			send_msg_retries=0;
			Ind_Watchdog_control(reload_food);
			return Next_state_2;
		}
		send_msg_retries++;
		Ind_Watchdog_control(reload_food);
		return Next_state_1;

	}
	send_msg_retries=0;
	Ind_Watchdog_control(reload_food);
	return Next_state_0;

}

SM_next_state_t SM_Deinit_protocols(void)
{

	BMI160_Deinit(I2C1_Alt);
	I2C1_Deinit();

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


