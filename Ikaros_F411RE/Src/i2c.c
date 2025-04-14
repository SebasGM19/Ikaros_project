/*
 * i2c.c
 *
 *  Created on: Apr 12, 2025
 *      Author: sebas
 */

#include "i2c.h"
#include "gpios.h"


Status_code_t I2C1_Init(i2c_config_parameters_t config){

	Status_code_t status = Success;

	SetPinMode(Port_B, Pin_8, Alt_func_mode); //SCL
	SetPinMode(Port_B, Pin_9, Alt_func_mode); //SDA

	GpioSetAlternativeFunction(Port_B, Pin_8, I2C1_I2C2_I2C3);
	GpioSetAlternativeFunction(Port_B, Pin_9, I2C1_I2C2_I2C3);

	Gpio_Output_type(Port_B, Pin_8, Open_drain);
	Gpio_Output_type(Port_B, Pin_9, Open_drain);

	GpioPullUpDownState(Port_B, Pin_8, Pull_Up);
	GpioPullUpDownState(Port_B, Pin_9, Pull_Up);

	I2C_Clock(I2C_1,Enabled);

	return status;
}

Status_code_t I2C3_Init(i2c_config_parameters_t config){

	Status_code_t status = Success;

	SetPinMode(Port_A, Pin_8, Alt_func_mode); //SCL
	SetPinMode(Port_C, Pin_9, Alt_func_mode); //SDA

	GpioSetAlternativeFunction(Port_A, Pin_8, I2C1_I2C2_I2C3);
	GpioSetAlternativeFunction(Port_C, Pin_9, I2C1_I2C2_I2C3);

	Gpio_Output_type(Port_A, Pin_8, Open_drain);
	Gpio_Output_type(Port_C, Pin_9, Open_drain);

	GpioPullUpDownState(Port_A, Pin_8, Pull_Up);
	GpioPullUpDownState(Port_C, Pin_9, Pull_Up);



	I2C_Clock(I2C_3,Enabled);

	return status;

}


Status_code_t I2C_Write(I2C_alternative_t i2c_alt,uint8_t addrs, uint8_t* data_sent, uint32_t data_size){
	//bit 0 for write
	Status_code_t status = Success;

	return status;

}

Status_code_t I2C_Read(I2C_alternative_t i2c_alt, uint8_t addrs, uint8_t* data_received, uint32_t data_size_expected){
	//bit 1 for read
	Status_code_t status = Success;

	return status;

}

Status_code_t I2C1_Deinit(void){

	Gpio_Output_type(Port_B, Pin_8, Push_pull);
	Gpio_Output_type(Port_B, Pin_9, Push_pull);

	GpioPullUpDownState(Port_B, Pin_8, No_pull_No_Down);
	GpioPullUpDownState(Port_B, Pin_9, No_pull_No_Down);

	I2C_Clock(I2C_1,Disabled);

	return Success;
}

Status_code_t I2C3_Deinit(void){

	Gpio_Output_type(Port_A, Pin_8, Push_pull);
	Gpio_Output_type(Port_C, Pin_9, Push_pull);

	GpioPullUpDownState(Port_A, Pin_8, No_pull_No_Down);
	GpioPullUpDownState(Port_C, Pin_9, No_pull_No_Down);

	I2C_Clock(I2C_3,Disabled);

	return Success;

}

//auxiliar functions


Status_code_t I2C_Clock(Enabled_Disabled_t state, I2C_alternative_t I2C){

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_APB1ENR);

    *pClockControlReg = (state) ? (*pClockControlReg | (Enabled << I2C)) : (*pClockControlReg & ~(Enabled << I2C));

    return Success;

}
