/*
 * stepper_motor.c
 *
 *  Created on: Apr 7, 2024
 *      Author: Sebastian G.M
 */

#include "stepper_motor.h"


Status_code_t Init_8bits_Stepper_Motor(void){

	Status_code_t status;

	__IO uint32_t *pPort_ModeReg = (__IO uint32_t *)(Port_C + OFFSET_PORTS);

	status = ClockEnable(Port_C, Enabled);
	*pPort_ModeReg &= ~(Clear_sixteen_bits); //clean those 16 bits only for this funtion is ok by this way
	*pPort_ModeReg |= 0x5555;// equal to : 0101 0101 0101 0101

	return status;

}

void Deinit_8bits_Stepper_Motor(void){

	__IO uint32_t *pPort_ModeReg = (__IO uint32_t *)(Port_C + OFFSET_PORTS);

	*pPort_ModeReg &= ~(Clear_sixteen_bits); //clean those 16 bits only for this funtion is ok by this way

}


Status_code_t Write_8bits_Stepper_Motor(uint8_t secuence){


	__IO uint32_t *PORT_REG_OUTPUT = (__IO uint32_t *)(Port_C + GPIOx_ODR_OFFSET);
	*PORT_REG_OUTPUT &= ~(Clear_eight_bits);
	*PORT_REG_OUTPUT |= secuence;

	return Success;

}


Status_code_t Init_4bits_Stepper_Motor(Stepper_option_t Stepper_alternative){

	uint8_t output_count=0;
	uint8_t found_bits =0;
	uint8_t starting_gpio =0;
	uint16_t PositionsOfPin =0;
	Set_Port_t Port_option = Port_A; //set as a default

	switch(Stepper_alternative){
	case PortA_Op1:
		Port_option = Port_A;
		starting_gpio =Pin_4;
		break;
	case PortA_Op2:
		starting_gpio = Pin_8;
		Port_option = Port_A;
		break;
	case PortB_Op1:
		starting_gpio =Pin_12;

		Port_option = Port_B;
		break;
	case PortC_Op1:
		starting_gpio =Pin_0;
		Port_option = Port_C;
		break;
	default:
		return OptionNotSupported;
	}

	__IO uint32_t *pPort_ModeReg = (__IO uint32_t *)(Port_option+ OFFSET_PORTS);
	ClockEnable(Port_option, Enabled);
	uint8_t  *set_to_output = search_bits(Stepper_alternative, &found_bits);


	PositionsOfPin = (uint16_t)starting_gpio*2;
	*pPort_ModeReg &= ~(Clear_eight_bits<<PositionsOfPin);

	while(output_count<=found_bits && output_count<MAX_GPIOS){ //antes <=MAXGPIOS
		PositionsOfPin=(set_to_output[output_count]*2);
		*pPort_ModeReg |= (Output<<PositionsOfPin);
		output_count++;
	}


	return Success;
}

Status_code_t Deinit_4bits_Stepper_Motor(Stepper_option_t Stepper_alternative){

	uint8_t starting_gpio =0;
	uint16_t PositionsOfPin =0;
	Set_Port_t Port_option = Port_A; //set as a default

	switch(Stepper_alternative){
	case PortA_Op1:
		Port_option = Port_A;
		starting_gpio =Pin_4;
		break;
	case PortA_Op2:
		starting_gpio = Pin_8;
		Port_option = Port_A;
		break;
	case PortB_Op1:
		starting_gpio =Pin_12;

		Port_option = Port_B;
		break;
	case PortC_Op1:
		starting_gpio =Pin_0;
		Port_option = Port_C;
		break;
	default:
		return OptionNotSupported;
	}

	__IO uint32_t *pPort_ModeReg = (__IO uint32_t *)(Port_option+ OFFSET_PORTS);

	PositionsOfPin = (uint16_t)starting_gpio*2;
	*pPort_ModeReg &= ~(Clear_eight_bits<<PositionsOfPin);

	return Success;
}


Status_code_t Write_4bits_Stepper_Motor(Stepper_option_t Stepper_alternative, uint32_t secuence){

	uint8_t gpio_offset = 0;
	Set_Port_t Port_option = Port_A;

	switch(Stepper_alternative){
	case PortA_Op1:
		Port_option = Port_A;
		gpio_offset = Pin_4;
		break;
	case PortA_Op2:
		Port_option = Port_A;
		gpio_offset = Pin_8;
		break;
	case PortB_Op1:
		Port_option = Port_B;
		gpio_offset = Pin_12;
		break;
	case PortC_Op1:
		Port_option = Port_C;
		gpio_offset = Pin_0;
		break;
	default:
		return OptionNotSupported;
	}

	__IO uint32_t *PORT_REG_OUTPUT = (__IO uint32_t *)(Port_option + GPIOx_ODR_OFFSET);
	*PORT_REG_OUTPUT &= ~(Stepper_alternative); //no need to be multiplied by 2, it from 0 to 15
	*PORT_REG_OUTPUT |= (secuence << gpio_offset);

	return Success;

}

uint8_t *search_bits(Stepper_option_t Stepper_alternative, uint8_t* found_bits){

	uint8_t high_state_positions[MAX_GPIOS];
	memset(high_state_positions,'\0',MAX_GPIOS);
	uint8_t* pHigh_position;
	uint8_t GPIO_position_count = 0;
	uint8_t position_high_state =0;

	while(GPIO_position_count < MAX_GPIOS){

		if((Stepper_alternative>>GPIO_position_count) & 1){
			high_state_positions[position_high_state] = GPIO_position_count;
			position_high_state++;
		}
		GPIO_position_count++;
	}

	*found_bits = position_high_state;
	pHigh_position = &high_state_positions[0];

	return pHigh_position;

}


