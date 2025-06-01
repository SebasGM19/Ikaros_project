/*
 * BMI160.c
 *
 *  Created on: May 3, 2025
 *      Author: sebas
 */


#include "BMI160.h"
#include "../src/watchdog.h"

static const float Gyr_one_unit = 0.061f;
static const float Acc_one_unit = 0.0039f;
static const float Gravity = 9.81f; //gravity m/s2
static volatile uint32_t global_Acc_sensibility = 2048;
static volatile float global_Gyr_sensibility = 16.4f;

Status_code_t BMI160_Init(I2C_alternative_t I2C, BMI160_init_parameters *param){

	PMU_status_t accelerometer  = Normal;
	PMU_status_t Gyroscope 		= Normal;
	PMU_status_t magnetomer 	= Normal;

	Status_code_t status = Success;


	status =  BMI160_Soft_Reset(I2C);
	Delay(50000);
	status =  BMI160_Set_Acceleromete_PM(I2C, param->Acc_power_mode);
	Delay(500);
	status =  BMI160_Set_Gyroscope_PM(I2C, param->Gyr_power_mode);
	Delay(500);

	status = BMI160_PMU_Status(I2C, &accelerometer, &Gyroscope, &magnetomer);
	if(accelerometer != Normal && Gyroscope != Normal && status != Success){
		return Error;
	}

	status = BMI160_Set_Accelerometer_Resolution(I2C, param->Acc_res);//la mejor resolucion
	Delay(500);

	status = BMI160_Set_Gyroscope_Resolution(I2C, param->Gyr_res); // la mejor resolucion
	Delay(500);


	return status;

}

Status_code_t BMI160_Deinit(I2C_alternative_t I2C){

	BMI160_Soft_Reset(I2C);

	return Success;

}
//returns the CHIP ID, typically 0xD1
Status_code_t BMI160_Chip_ID(I2C_alternative_t I2C, uint8_t *chip_id)
{

    uint8_t chip_id_reg=0x00;
    Status_code_t status = Success;
    uint8_t read_data;


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &chip_id_reg, sizeof(chip_id_reg));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }

    (*chip_id) = (uint8_t)read_data;

	return Success;
}

//returns the power mode status of the sensors
Status_code_t BMI160_PMU_Status(I2C_alternative_t I2C,
				PMU_status_t *Accelerometer, PMU_status_t *Gyroscope, PMU_status_t *Magnetometer)
{

    uint8_t PMU_status = 0x03;
    Status_code_t status = Success;
    uint8_t read_data;


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &PMU_status,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

    (*Magnetometer) = (PMU_status_t)(read_data & 0x03);
    (*Gyroscope) 	= (PMU_status_t)((read_data>>2) & 0x03);
    (*Accelerometer) = (PMU_status_t)((read_data>>4) & 0x03);

	return Success;
}


Status_code_t BMI160_Get_TotalData(I2C_alternative_t I2C, BMI160_TotalData_t *parameter)
{

    uint8_t Data_arr[20] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
							0x0A, 0x0B,
							0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
							0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

    Status_code_t status = Success;
    uint8_t read_data[20] = {0};

	int16_t accelerometer_X_aux = 0;
	int16_t accelerometer_Y_aux = 0;
	int16_t accelerometer_Z_aux = 0;

	int16_t gyroscope_X_aux = 0;
	int16_t gyroscope_Y_aux = 0;
	int16_t gyroscope_Z_aux = 0;


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }

    parameter->magnetometer_X 	= (int16_t)((read_data[1]<<8) | read_data[0]);
    parameter->magnetometer_Y 	= (int16_t)((read_data[3]<<8) | read_data[2]);
    parameter->magnetometer_Z 	= (int16_t)((read_data[5]<<8) | read_data[4]);

    parameter->hall_resistance 	= (uint16_t)((read_data[7]<<8) | read_data[6]);

    gyroscope_X_aux 	   	= (int16_t)((read_data[9]<<8) | read_data[8]);
    gyroscope_Y_aux 	   	= (int16_t)((read_data[11]<<8) | read_data[10]);
    gyroscope_Z_aux 		= (int16_t)((read_data[13]<<8) | read_data[12]);

    accelerometer_X_aux 	= (int16_t)((read_data[15]<<8) | read_data[14]);
    accelerometer_Y_aux 	= (int16_t)((read_data[17]<<8) | read_data[16]);
    accelerometer_Z_aux 	= (int16_t)((read_data[19]<<8) | read_data[18]);


    parameter->gyroscope_X = (float)(gyroscope_X_aux/(float)global_Gyr_sensibility);
    parameter->gyroscope_Y = (float)(gyroscope_Y_aux/(float)global_Gyr_sensibility);
    parameter->gyroscope_Z = (float)(gyroscope_Z_aux/(float)global_Gyr_sensibility);

    parameter->accelerometer_X = (float)(accelerometer_X_aux/(float)global_Acc_sensibility)*Gravity;
    parameter->accelerometer_Y = (float)(accelerometer_Y_aux/(float)global_Acc_sensibility)*Gravity;
    parameter->accelerometer_Z = (float)(accelerometer_Z_aux/(float)global_Acc_sensibility)*Gravity;


	return Success;
}

//this function gets the data of magnetometer in the struct
Status_code_t BMI160_Get_Magnetometer_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter)
{

    uint8_t Data_arr[6] = {0x04,0x05,0x06,0x07,0x08,0x09};

    Status_code_t status = Success;
    uint8_t read_data[6] = {0};


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    parameter->magnetometer_X 	= (int16_t)((read_data[1]<<8) | read_data[0]);
    parameter->magnetometer_Y 	= (int16_t)((read_data[3]<<8) | read_data[2]);
    parameter->magnetometer_Z 	= (int16_t)((read_data[5]<<8) | read_data[4]);


	return Success;
}

Status_code_t BMI160_Get_HallR_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter)
{

    uint8_t Data_arr[2] = {0x0A,0x0B};

    Status_code_t status = Success;
    uint8_t read_data[2] = {0};


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    parameter->hall_resistance 	= (int16_t)((read_data[1]<<8) | read_data[0]);


	return Success;
}

//this function gets the data of gyroscope in the struct
Status_code_t BMI160_Get_Gyroscope_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter)
{

    uint8_t Data_arr[6] = {0x0C,0x0D,0x0E,0x0F,0x10,0x11};

    Status_code_t status = Success;
    uint8_t read_data[6] = {0};

	int16_t gyroscope_X_aux = 0;
	int16_t gyroscope_Y_aux = 0;
	int16_t gyroscope_Z_aux = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    gyroscope_X_aux	   	= (int16_t)((read_data[1]<<8) | read_data[0]);
    gyroscope_Y_aux    	= (int16_t)((read_data[3]<<8) | read_data[2]);
    gyroscope_Z_aux 	= (int16_t)((read_data[5]<<8) | read_data[4]);

    parameter->gyroscope_X = (float)(gyroscope_X_aux/(float)global_Gyr_sensibility);
    parameter->gyroscope_X = (float)(gyroscope_Y_aux/(float)global_Gyr_sensibility);
    parameter->gyroscope_X = (float)(gyroscope_Z_aux/(float)global_Gyr_sensibility);


	return Success;
}

//this function gets the data of accelerometer in the struct
Status_code_t BMI160_Get_Accelerometer_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter)
{

    uint8_t Data_arr[6] = {0x12U,0x13U,0x14U,0x15U,0x16U,0x17U};
    Status_code_t status = Success;
    uint8_t read_data[6] = {0};

	int16_t accelerometer_X_aux = 0;
	int16_t accelerometer_Y_aux = 0;
	int16_t accelerometer_Z_aux = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    accelerometer_X_aux = (int16_t)((read_data[1]<<8) | read_data[0]);
    accelerometer_Y_aux = (int16_t)((read_data[3]<<8) | read_data[2]);
    accelerometer_Z_aux = (int16_t)((read_data[5]<<8) | read_data[4]);

    parameter->accelerometer_X = (float)(accelerometer_X_aux/(float)global_Acc_sensibility)*Gravity;
    parameter->accelerometer_Y = (float)(accelerometer_Y_aux/(float)global_Acc_sensibility)*Gravity;
    parameter->accelerometer_Z = (float)(accelerometer_Z_aux/(float)global_Acc_sensibility)*Gravity;

	return Success;
}

//this function return the data status if its ready to read
Status_code_t BMI160_Data_Status(I2C_alternative_t I2C, data_status_t* accelerometer,
								data_status_t* gyroscope, data_status_t* magnetometer, data_status_t* FOC)
{

    uint8_t Data_arr = 0x1BU;
    Status_code_t status = Success;
    uint8_t read_data;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }

    (*accelerometer) = (data_status_t)(read_data>>7 & Ready);
    (*gyroscope) 	 = (data_status_t)(read_data>>6 & Ready);
    (*magnetometer)  = (data_status_t)(read_data>>5 & Ready);
    (*FOC)  		 = (data_status_t)(read_data>>3 & Ready);
	return Success;
}

//this function get the temperature
Status_code_t BMI160_Temperature(I2C_alternative_t I2C, int16_t *temperature)
{

    uint8_t Data_arr[2] = {0x20,0x21};
    Status_code_t status = Success;
    uint8_t read_data[2] ={0};
    int16_t raw_value =0;
    float temp_aux =0.0f;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data,2);
    if(Success!=status){
        return status;
    }
    raw_value= (int16_t)(read_data[1]<<8 | read_data[0]);

    temp_aux = (float)(23.0f +(raw_value/ 512.0f ));
    *temperature = (int16_t)(temp_aux * 10.0f);

	return Success;
}


Status_code_t BMI160_Get_Accelerometer_Resolution(I2C_alternative_t I2C,Acc_Resolution_t *res)
{

    uint8_t Data_arr = 0x41;
    Status_code_t status = Success;
    uint8_t read_data = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_arr,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

    *res = (Acc_Resolution_t)(read_data & 0x0F);

    switch((*res)){
		case Acc_2g:
			global_Acc_sensibility = (uint32_t)(1<<14);
			break;
		case Acc_4g:
			global_Acc_sensibility = (uint32_t)(1<<13);
			break;
		case Acc_8g:
			global_Acc_sensibility = (uint32_t)(1<<12);
			break;
		case Acc_16g:
			global_Acc_sensibility = (uint32_t)(1<<11);
			break;
		default:
			global_Acc_sensibility = (uint32_t)(1<<14);
			break;
    }

	return Success;
}


Status_code_t BMI160_Set_Accelerometer_Resolution(I2C_alternative_t I2C, Acc_Resolution_t res)
{

    uint8_t Data_arr[2] = {0x41, (uint8_t)(res)};
    Status_code_t status = Success;
    uint8_t read_data = 0;


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }



    switch(res){
		case Acc_2g:
			global_Acc_sensibility = (uint32_t)(Acc_2g_sen);
			break;
		case Acc_4g:
			global_Acc_sensibility = (uint32_t)(Acc_4g_sen);
			break;
		case Acc_8g:
			global_Acc_sensibility = (uint32_t)(Acc_8g_sen);
			break;
		case Acc_16g:
			global_Acc_sensibility = (uint32_t)(Acc_16g_sen);
			break;
		default:
			global_Acc_sensibility = (uint32_t)(Acc_2g_sen);
			break;
    }

	return Success;
}


Status_code_t BMI160_Get_Gyroscope_Resolution(I2C_alternative_t I2C, Gyr_Resolution_t *res)
{

    uint8_t Data_arr = 0x43;
    Status_code_t status = Success;
    uint8_t read_data = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_arr,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

    *res = (Gyr_Resolution_t)(read_data & 0x07);

    switch((*res)){ //taking the max sensibility in each case
		case Gyr_2000s:
			global_Gyr_sensibility = (float)(16.9f);
			break;
		case Gyr_1000s:
			global_Gyr_sensibility = (float)(33.8f);
			break;
		case Gyr_500s:
			global_Gyr_sensibility = (float)(67.6f);
			break;
		case Gyr_250s:
			global_Gyr_sensibility = (float)(135.2f);
			break;
		case Gyr_125s:
			global_Gyr_sensibility = (float)(270.3f);
			break;
		default:
			global_Gyr_sensibility = (float)(16.9f);
			break;
    }

	return Success;
}


Status_code_t BMI160_Set_Gyroscope_Resolution(I2C_alternative_t I2C, Gyr_Resolution_t res)
{

    uint8_t Data_arr[2] = {0x43, (uint8_t)(res)};
    Status_code_t status = Success;
    uint8_t read_data = 0;


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

    switch(res){ //taking the max sensibility in each case
		case Gyr_2000s:
			global_Gyr_sensibility = (float)(16.9f);
			break;
		case Gyr_1000s:
			global_Gyr_sensibility = (float)(33.8f);
			break;
		case Gyr_500s:
			global_Gyr_sensibility = (float)(67.6f);
			break;
		case Gyr_250s:
			global_Gyr_sensibility = (float)(135.2f);
			break;
		case Gyr_125s:
			global_Gyr_sensibility = (float)(270.3f);
			break;
		default:
			global_Gyr_sensibility = (float)(16.9f);
			break;
    }

	return Success;
}

Status_code_t BMI160_Set_Fast_Offset_Compensation(I2C_alternative_t I2C, FOC_gyr_values_t gyr_state,
												FOC_acc_values_t acc_X, FOC_acc_values_t acc_Y, FOC_acc_values_t acc_Z)
{

    uint8_t Data_config_FOC[2] = {0x69,0x00};
    uint8_t Data_activate_FOC[2] = {0x7E,0x03};
    Status_code_t status = Success;
    uint8_t read_data = 0;

    Data_config_FOC[1] = (uint8_t)( (gyr_state<<6) | (acc_X<<4) | (acc_Y<<2) | (acc_Z<<0) );

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_config_FOC,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }
    Delay(500);

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_activate_FOC,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }
	return Success;
}



Status_code_t BMI160_AutoSet_Data_Offset(I2C_alternative_t I2C)
{

    Status_code_t status = Success;
    uint8_t Data_arr_AccX_off[2] = {0x71,0x00};
    uint8_t Data_arr_AccY_off[2] = {0x72,0x00};
    uint8_t Data_arr_AccZ_off[2] = {0x73,0x00};

    uint8_t Data_arr_GyrX_off[2] = {0x74,0x00};
    uint8_t Data_arr_GyrY_off[2] = {0x75,0x00};
    uint8_t Data_arr_GyrZ_off[2] = {0x76,0x00};

    uint8_t Data_arr_last_off[2] = {0x77,0x00};

    uint8_t Data_arr[20] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
							0x0A, 0x0B,
							0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
							0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

    uint8_t read_data[20] = {0};

    int16_t AccX_real_off =0;
    int16_t AccY_real_off =0;
    int16_t AccZ_real_off =0;

    int16_t accelerometer_X_aux = 0;
	int16_t accelerometer_Y_aux = 0;
	int16_t accelerometer_Z_aux = 0;


    int16_t GyrX_real_off =0;
    int16_t GyrY_real_off =0;
    int16_t GyrZ_real_off =0;

	int16_t gyroscope_X_aux = 0;
	int16_t gyroscope_Y_aux = 0;
	int16_t gyroscope_Z_aux = 0;

	data_status_t accelerometer = Not_ready;
	data_status_t Gyroscope 	= Not_ready;
	data_status_t magnetomer 	= Not_ready;
	data_status_t FOC		 	= Not_ready;

	status = BMI160_Data_Status(I2C1_Alt, &accelerometer, &Gyroscope, &magnetomer,&FOC);
	if(accelerometer != Ready  && Gyroscope != Ready){
		return Unknown;
	}


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }

    gyroscope_X_aux 	   	= (int16_t)((read_data[9]<<8) | read_data[8]);
    gyroscope_Y_aux 	   	= (int16_t)((read_data[11]<<8) | read_data[10]);
    gyroscope_Z_aux 		= (int16_t)((read_data[13]<<8) | read_data[12]);

    accelerometer_X_aux 	= (int16_t)((read_data[15]<<8) | read_data[14]);
    accelerometer_Y_aux 	= (int16_t)((read_data[17]<<8) | read_data[16]);
    accelerometer_Z_aux 	= (int16_t)((read_data[19]<<8) | read_data[18]);



    //parte del accelerometro
    AccX_real_off = (int16_t)(-1 * (float)(accelerometer_X_aux/(float)Acc_one_unit));
    AccY_real_off = (int16_t)(-1 * (float)(accelerometer_Y_aux/(float)Acc_one_unit));
    AccZ_real_off = (int16_t)(-1 * (float)(accelerometer_Z_aux/(float)Acc_one_unit));

    Data_arr_AccX_off[1] = (uint8_t)(AccX_real_off & 0x00FF);
    Data_arr_AccY_off[1] = (uint8_t)(AccY_real_off & 0x00FF);
    Data_arr_AccZ_off[1] = (uint8_t)(AccZ_real_off & 0x00FF);

    //parte del gyroscopio
    GyrX_real_off = (int16_t)(-1 * (float)(gyroscope_X_aux/(float)Gyr_one_unit));
	GyrY_real_off = (int16_t)(-1 * (float)(gyroscope_Y_aux/(float)Gyr_one_unit));
	GyrZ_real_off = (int16_t)(-1 * (float)(gyroscope_Z_aux/(float)Gyr_one_unit));

	Data_arr_GyrX_off[1] = (uint8_t)(GyrX_real_off & 0x00FF);
	Data_arr_GyrY_off[1] = (uint8_t)(GyrY_real_off & 0x00FF);
	Data_arr_GyrZ_off[1] = (uint8_t)(GyrZ_real_off & 0x00FF);

	Data_arr_last_off[1] =  (uint8_t)( (((GyrX_real_off >> 8) & 0x03)<<0) |
							  	  	   (((GyrY_real_off >> 8) & 0x03)<<2) |
									   (((GyrZ_real_off >> 8) & 0x03)<<4) );


    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr_AccX_off,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr_AccY_off,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr_AccZ_off,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr_GyrX_off,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr_GyrY_off,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr_GyrZ_off,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr_last_off,2);
    if(Success != status){
        return status;
    }
    Delay(500);

	status = BMI160_Enabled_Accelerometer_Offset(I2C);
	Delay(500);
	status = BMI160_Enabled_Gyroscope_Offset(I2C);
	Delay(500);

	return status;
}


Status_code_t BMI160_Get_Data_Offset(I2C_alternative_t I2C, uint8_t* data)
{

    Status_code_t status = Success;
    uint8_t Data_arr = 0x77;

    status = I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_arr,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, data,1);
    if(Success!=status){
        return status;
    }

	return Success;
}

Status_code_t BMI160_Enabled_Accelerometer_Offset(I2C_alternative_t I2C)
{

    uint8_t Data_arr[2] = {0x77,0x00};
	uint8_t Data_ask = 0x77;
    Status_code_t status = Success;
    uint8_t read_data = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_ask,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }
    Delay(500);

    Data_arr[1] = (uint8_t)(read_data | acc_offset_bit_EN);

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }

	return Success;
}

Status_code_t BMI160_Disabled_Accelerometer_Offset(I2C_alternative_t I2C)
{

    uint8_t Data_arr[2] = {0x77, 0x00};
	uint8_t Data_ask = 0x77;
    Status_code_t status = Success;
    uint8_t read_data = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_ask,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }
    Delay(500);

    Data_arr[1] = (uint8_t)(read_data | (~acc_offset_bit_EN));

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }

	return Success;
}

Status_code_t BMI160_Enabled_Gyroscope_Offset(I2C_alternative_t I2C)
{

    uint8_t Data_arr[2] = {0x77, 0x00};
	uint8_t Data_ask = 0x77;
    Status_code_t status = Success;
    uint8_t read_data = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_ask,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }
    Delay(500);

    Data_arr[1] = (uint8_t)(read_data | gyr_offset_bit_EN);

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }

	return Success;
}

Status_code_t BMI160_Disabled_Gyroscope_Offset(I2C_alternative_t I2C)
{

    uint8_t Data_arr[2] = {0x77, 0x00};
	uint8_t Data_ask = 0x77;
    Status_code_t status = Success;
    uint8_t read_data = 0;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_ask,1);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }
    Delay(500);

    Data_arr[1] = (uint8_t)(read_data | (~gyr_offset_bit_EN));

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }

	return Success;
}

//This function change the power mode of the accelerometer
Status_code_t BMI160_Set_Acceleromete_PM(I2C_alternative_t I2C, Acc_PM_t selected_ACC_PM)
{

    uint8_t ACC_PM[2] = {0x7E, (uint8_t)selected_ACC_PM};
    Status_code_t status = Success;
    uint8_t read_data;


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, ACC_PM,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

	return Success;
}

//This function change the power mode of the Gyroscope
Status_code_t BMI160_Set_Gyroscope_PM(I2C_alternative_t I2C, Gyr_PM_t selected_GYR_PM)
{

    uint8_t GYR_PM[2]  = {0x7E, (uint8_t)selected_GYR_PM};
    Status_code_t status = Success;
    uint8_t read_data;


    status=I2C_Write(I2C, BMI160_SLAVE_ADDR, GYR_PM,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

	return Success;
}

//This function change the power mode of the magnetometer
Status_code_t BMI160_Set_Magnetometer_PM(I2C_alternative_t I2C, Mag_PM_t selected_MAG_PM)
{

    uint8_t MAG_PM[2]  = {0x7E, (uint8_t)selected_MAG_PM};
    Status_code_t status = Success;
    uint8_t read_data;


    status=I2C_Write(I2C, BMI160_SLAVE_ADDR, MAG_PM,2);
    if(Success != status){
        return status;
    }
    Delay(500);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

	return Success;
}

Status_code_t BMI160_Soft_Reset(I2C_alternative_t I2C)
{

    uint8_t Data_arr[2]  = {0x7E, 0xB6};
    Status_code_t status = Success;


    status=I2C_Write(I2C, BMI160_SLAVE_ADDR, Data_arr,2);
    if(Success != status){
        return status;
    }
    Delay(500);

	return Success;
}

