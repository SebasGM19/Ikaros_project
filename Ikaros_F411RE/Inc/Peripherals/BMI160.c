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


    status=I2C_Write(I2C,BMI160_SLAVE_ADDR, &chip_id_reg,1);
    if(Success != status){
        return status;
    }
    Delay(100000);

    status= I2C_Read(I2C, BMI160_SLAVE_ADDR, &read_data,1);
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


//this function get the temperature
Status_code_t BMI160_temperature(I2C_alternative_t I2C, int16_t *temperature)
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

