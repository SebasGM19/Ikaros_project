/*
 * SHT20.c
 *
 *  Created on: 13 abr. 2025
 *      Author: Sebas
 */
#include "SHT20.h"


Status_code_t SHT20ReadTemperature(I2C_alternative_t I2C, int16_t* temp){

    uint8_t temp_reg=0xF3;
    Status_code_t status = Success;
    uint8_t read_data_temp[2];
    float decimal_temp=0;

    status=I2C_Write(I2C,SLAVE_ADDRS, &temp_reg,1);

    Delay(50000);

    status= I2C_Read(I2C,SLAVE_ADDRS,read_data_temp,2);
    if(Success!=status){
        return status;
    }
    Delay(500000);
    decimal_temp = (175.72f * (((read_data_temp[0]<<8 | read_data_temp[1])+1.0f)/65536.0f))-46.85f;

    (*temp)=decimal_temp*10;
    return status;
}

Status_code_t SHT20ReadHumedad(I2C_alternative_t I2C, uint16_t* hum){

    uint8_t hum_reg=0xF5;
    Status_code_t status = Success;
    uint8_t read_data_hum[2];
    float decimal_hum=0;

    status = I2C_Write(I2C,SLAVE_ADDRS, &hum_reg,1);

    Delay(50000);

    status= I2C_Read(I2C,SLAVE_ADDRS,read_data_hum,2);
    if(Success != status){
        return status;
    }
    Delay(500000);
    decimal_hum = (125.0f * (((read_data_hum[0]<<8 | read_data_hum[1])+1.0f)/65536.0f))-6.0f;

    (*hum)=decimal_hum*10;

    return status;
}
