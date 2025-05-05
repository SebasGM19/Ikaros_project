/*
 * BMI160.c
 *
 *  Created on: May 3, 2025
 *      Author: sebas
 */


#include "BMI160.h"

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
    Delay(100000);

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
    Delay(100000);

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

    uint8_t Data_arr[20] = {0x04,0x05,0x06,0x07,0x08,
    						0x09,0x0A,0x0B,0x0C,0x0D,
							0x0E,0x0F,0x10,0x11,0x12,
							0x13,0x14,0x15,0x16,0x17};

    Status_code_t status = Success;
    uint8_t read_data[20] = {0};


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    parameter->magnetometer_X 	= (int16_t)((read_data[1]<<8) | read_data[0]);
    parameter->magnetometer_Y 	= (int16_t)((read_data[3]<<8) | read_data[2]);
    parameter->magnetometer_Z 	= (int16_t)((read_data[5]<<8) | read_data[4]);

    parameter->hall_resistance 	= (uint16_t)((read_data[7]<<8) | read_data[6]);

    parameter->gyroscope_X 	   	= (int16_t)((read_data[9]<<8) | read_data[8]);
    parameter->gyroscope_Y 	   	= (int16_t)((read_data[11]<<8) | read_data[10]);
    parameter->gyroscope_Z 		= (int16_t)((read_data[13]<<8) | read_data[12]);

    parameter->accelerometer_X 	= (int16_t)((read_data[15]<<8) | read_data[14]);
    parameter->accelerometer_Y 	= (int16_t)((read_data[17]<<8) | read_data[16]);
    parameter->accelerometer_Z 	= (int16_t)((read_data[19]<<8) | read_data[18]);


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
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    parameter->magnetometer_X 	= (int16_t)((read_data[1]<<8) | read_data[0]);
    parameter->magnetometer_Y 	= (int16_t)((read_data[3]<<8) | read_data[2]);
    parameter->magnetometer_Z 	= (int16_t)((read_data[5]<<8) | read_data[4]);


	return Success;
}

//this function gets the data of gyroscope in the struct
Status_code_t BMI160_Get_Gyroscope_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter)
{

    uint8_t Data_arr[6] = {0x0C,0x0D,0x0E,0x0F,0x10,0x11};

    Status_code_t status = Success;
    uint8_t read_data[6] = {0};


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    parameter->gyroscope_X 	   	= (int16_t)((read_data[1]<<8) | read_data[0]);
    parameter->gyroscope_Y 	   	= (int16_t)((read_data[3]<<8) | read_data[2]);
    parameter->gyroscope_Z 		= (int16_t)((read_data[5]<<8) | read_data[4]);


	return Success;
}

//this function gets the data of accelerometer in the struct
Status_code_t BMI160_Get_Accelerometer_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter)
{

    uint8_t Data_arr[6] = {0x12U,0x13U,0x14U,0x15U,0x16U,0x17U};
    Status_code_t status = Success;
    uint8_t read_data[6] = {0};

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }
    parameter->accelerometer_X 	= (int16_t)((read_data[1]<<8) | read_data[0]);
    parameter->accelerometer_Y  = (int16_t)((read_data[3]<<8) | read_data[2]);
    parameter->accelerometer_Z 	= (int16_t)((read_data[5]<<8) | read_data[4]);


	return Success;
}

//this function return the data status if its ready to read
Status_code_t BMI160_Data_Status(I2C_alternative_t I2C, data_status_t* accelerometer,
								data_status_t* gyroscope, data_status_t* magnetometer)
{

    uint8_t Data_arr = 0x1BU;
    Status_code_t status = Success;
    uint8_t read_data;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &Data_arr, sizeof(Data_arr));
    if(Success != status){
        return status;
    }
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data, sizeof(read_data));
    if(Success!=status){
        return status;
    }

    (*accelerometer) = (data_status_t)(read_data>>7 & Ready);
    (*gyroscope) 	 = (data_status_t)(read_data>>6 & Ready);
    (*magnetometer)  = (data_status_t)(read_data>>5 & Ready);

	return Success;
}

//this function get the temperature
Status_code_t BMI160_Temperature(I2C_alternative_t I2C, int16_t *temperature)
{

    uint8_t chip_id_reg[2] = {0x20,0x21};
    Status_code_t status = Success;
    uint8_t read_data[2] ={0};
    int16_t raw_value =0;
    float temp_aux =0.0f;

    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, chip_id_reg,2);
    if(Success != status){
        return status;
    }
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, read_data,2);
    if(Success!=status){
        return status;
    }
    raw_value= (int16_t)(read_data[1]<<8 | read_data[0]);

    temp_aux = (float)(23.0f +(raw_value/ 512.0f ));
    *temperature = (int16_t)(temp_aux * 10.0f);

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
    Delay(100000);

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
    Delay(100000);

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
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
    if(Success!=status){
        return status;
    }

	return Success;
}

