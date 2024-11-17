/*
 * Basic_settings.c
 *
 *  Created on: Feb 4, 2024
 *      Author: Sebastian G.M.
 */

#include "gpios.h"
#include "adc.h"

uint8_t static GPIO_EXT_Pin_ocupped[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


Gpio_State_Control_t GPIO_DigitalRead(Set_Port_t Port_define, Pin_number_t Pin_defined){

	__I uint32_t* const PORT_REG_INPUT = (__I uint32_t* const)(Port_define + GPIOx_IDR_OFFSET); //agregamos el 10 para decirle el offset

	if(((1<<Pin_defined) & (*PORT_REG_INPUT))==(1<<Pin_defined)){
		return High;
	}
	return Low;
}


Gpio_State_Control_t GPIO_DigitalWrite(Set_Port_t Port_define, Pin_number_t Pin_defined, Gpio_State_Control_t State){

	__IO uint32_t *PORT_REG_OUTPUT = (__IO uint32_t *)(Port_define + GPIOx_ODR_OFFSET); //agregamos el 0x14 par indicar output

	if(State){
		*PORT_REG_OUTPUT |= (1<<Pin_defined);
	}else{
		*PORT_REG_OUTPUT &= ~(1<<Pin_defined);

	}

	return State;

}


Status_code_t SetPinMode(Set_Port_t Port_define, Pin_number_t Pin_defined, PinMode_t Mode){

	__IO uint32_t *pPort_ModeReg = (__IO uint32_t *)(Port_define + OFFSET_PORTS);
	__IO uint16_t PositionsOfPin =0;
	Status_code_t status=Success;

	/*DO NOT MODIFY THIS IF STATEMENT, IMPLEMENTED TO AVOID TROUBLES WITH THE BOARD!!!*/
	if((Port_define == Port_A && (Pin_defined ==Pin_13 || Pin_defined ==Pin_14))
	|| (Port_define == Port_C && (Pin_defined ==Pin_14 || Pin_defined == Pin_15))
	|| (Port_define == Port_D && Pin_defined != Pin_2)
	|| (Port_define == Port_B && (Pin_defined == Pin_11 || Pin_defined == Pin_3))){

		return PinNotAvailable;
	}

	status = ClockEnable(Port_define, Enabled);

	PositionsOfPin = (__IO uint16_t)Pin_defined*2;
	*pPort_ModeReg &= ~(Clear_two_bits<<PositionsOfPin);

	switch(Mode){

		case Output:
			*pPort_ModeReg |= (Output<<PositionsOfPin);
			break;
		case Input:
			*pPort_ModeReg &= ~(Clear_two_bits<<PositionsOfPin); //si es input simplemente limpiara esas posiciones a 00
			break;
		case Alt_func_mode:
			*pPort_ModeReg |= (Alt_func_mode<<PositionsOfPin);
			break;
		case Analog_mode:
			*pPort_ModeReg |= (Analog_mode<<PositionsOfPin);
			break;
		default:
			status = OptionNotSupported;
			break;
	}

	return status;
}

void GpioSetAlternativeFunction(Set_Port_t Port_define, Pin_number_t Pin_defined, Alternate_function_map_t AF){

	GPIO_register_offset_t GPIO_AFRx_Offset=GPIOx_AFRL_OFFSET;
	uint8_t Pin_offset_to_substract =0;
	uint32_t real_register_position =0;

	if(Pin_defined >= 8){
		GPIO_AFRx_Offset=GPIOx_AFRH_OFFSET;
		Pin_offset_to_substract = 32;//to adjust for AFRH and PIN position
	}

	__IO uint32_t *pGpio_alt_func_reg = (__IO uint32_t *)((Port_define)+ GPIO_AFRx_Offset);

	real_register_position = (Pin_defined*4) - Pin_offset_to_substract;

	*pGpio_alt_func_reg &= ~(Clear_four_bits<<real_register_position);

	*pGpio_alt_func_reg |= (AF<<real_register_position);

}

Status_code_t GpioPullUpDownState(Set_Port_t Port_define, Pin_number_t Pin_defined, GPIO_UP_DOWN_STATE_t GPIO_State){
	__IO uint32_t *REG_PULL_UPdown = (__IO uint32_t *)(Port_define + GPIOx_PUPDR_OFFSET); //add 0x0C for offset to pull up or pull down
	uint16_t RealPosition=0;

	RealPosition = Pin_defined*2;
	*REG_PULL_UPdown &= ~(Clear_two_bits<<RealPosition); //limpiamos esa posicion del bit
	*REG_PULL_UPdown |= (GPIO_State<<RealPosition); //set the state in to pull
	return Success;

}

//EXT9 to 5 has the same line IRQ
//EXTI 10 to 15 has the same line IRQ

Status_code_t GPIO_Save_EXTI_PIN(Pin_number_t Pin_defined){

	return GPIO_EXT_Pin_ocupped[Pin_defined] ? EXTI_Line_already_use : (GPIO_EXT_Pin_ocupped[Pin_defined] = 1, Success);
}

void GPIO_Delete_EXTI_PIN(Pin_number_t Pin_defined){

	GPIO_EXT_Pin_ocupped[Pin_defined] = 0;
}

void GPIO_Set_EXTI_Line(GPIO_Exti_Port_t EXTI_Port,Pin_number_t Pin_defined){

	SYSCFG_register_offset_t Syscf_EXTI_CR_Offset = SYSCFG_EXTICR1 + (((uint8_t)(Pin_defined / 4)) * 4);
	__IO uint32_t *EXTI_LineReg = (__IO uint32_t *)(SYSCFG_ADDRESS+ Syscf_EXTI_CR_Offset);

	uint8_t reg_pin_offser = (((uint8_t)(Pin_defined / 4)) * 16);
	uint8_t star_position = (Pin_defined*4)-reg_pin_offser;

	*EXTI_LineReg &= ~(Clear_four_bits<<star_position);
	*EXTI_LineReg |= (EXTI_Port<<star_position);

}

void GPIO_EXTI_Mask(Pin_number_t Pin_defined,Enabled_Disabled_t Intention){


	__IO uint32_t *EXTI_mask_reg = (__IO uint32_t *)(EXT1_ADDRESS+ EXTI_IMR);
//	*EXTI_mask_reg = Intention ? (*EXTI_mask_reg | (1u << Pin_defined)) : (*EXTI_mask_reg & ~(1u << Pin_defined));
	if(Intention){
		*EXTI_mask_reg |= (1u<<Pin_defined);
	}else{
		*EXTI_mask_reg &= ~(1u<<Pin_defined);
	}

}



void GPIO_EXTI_Trigger_seleccion(Pin_number_t Pin_defined,GPIO_Exti_Config_t EXTI_mode){

	EXTI_registers_offset_t trigger_offset = EXTI_FTSR;

//	trigger_offset = EXTI_mode ? EXTI_RTSR : EXTI_FTSR; 	//if EXTI_mode = falling_edge(0) iquals FTSR offset
	if(EXTI_mode){
		trigger_offset = EXTI_RTSR;
	}

	__IO uint32_t *EXTI_trigger_reg = (__IO uint32_t *)(EXT1_ADDRESS + trigger_offset);
	*EXTI_trigger_reg|= (1u<<Pin_defined);

}


void GPIO_EXTI_Clean_Flag(Pin_number_t Pin_defined){

	__IO uint32_t *EXTI_REG_PR = (__IO uint32_t *)(EXT1_ADDRESS + EXTI_PR);
	*EXTI_REG_PR |= (1u<<Pin_defined);	//is cleared by programming it to ‘1’.

}

void GPIO_EXTI_Clean_Group_Of_Flag(Pin_number_t Pin_defined,RegAuxClean_t bit_to_clear){

	__IO uint32_t *EXTI_REG_PR = (__IO uint32_t *)(EXT1_ADDRESS + EXTI_PR);
	*EXTI_REG_PR |= (bit_to_clear<<Pin_defined);	//is cleared by programming it to ‘1’.

}

void GPIO_End_All_EXTIx(void){ //deinit all EXTI

	for(uint8_t i =0; i<MAX_GPIOS;i++){
		GPIO_EXTI_Mask(i,Disabled);
		GPIO_Delete_EXTI_PIN(i);
	}

	NVIC_DisableIRQ(EXTI0_IRQn);
	NVIC_DisableIRQ(EXTI1_IRQn);
	NVIC_DisableIRQ(EXTI2_IRQn);
	NVIC_DisableIRQ(EXTI3_IRQn);
	NVIC_DisableIRQ(EXTI4_IRQn);
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	NVIC_DisableIRQ(EXTI15_10_IRQn);

	SYS_ClockEnable(Disabled);
}



//////////////////////////////////////EXTI0 FOR ALL PIN_0 OF ALL PORTS/////////////////////////////
void EXTI0_HANDLER(void){
	/*Develop your interruption code from here*/




	/*To here*/
	GPIO_EXTI_Clean_Flag(Pin_0);

}

Status_code_t GPIO_Init_EXTI0(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode){//next task
	Status_code_t status =Success;
	Set_Port_t Port_define=Port_A;

	status = GPIO_Save_EXTI_PIN(Pin_0);
	if(status!= Success){
		return status;
	}

	switch(EXTI_Port){
	case EXTI_Port_A:
		 Port_define = Port_A;
		break;
	case EXTI_Port_B:
		 Port_define = Port_B;
		break;
	case EXTI_Port_C:
		 Port_define = Port_C;
		break;
	case EXTI_Port_D:
		 Port_define = Port_D;
		break;
	default:
		GPIO_Delete_EXTI_PIN(Pin_0);
		return OptionNotSupported;
		break;
	}

	status =  SetPinMode(Port_define, Pin_0, Input);
	if(status!=Success){
		GPIO_Delete_EXTI_PIN(Pin_0);
		return status;
	}
	SYS_ClockEnable(Enabled);
	GPIO_Set_EXTI_Line(EXTI_Port,Pin_0);
	GPIO_EXTI_Trigger_seleccion(Pin_0,EXTI_mode);
	GPIO_EXTI_Mask(Pin_0,Disabled);
	NVIC_EnableIRQ(EXTI0_IRQn);
	return Success;
}

void GPIO_Enable_EXTI0(void){
	GPIO_EXTI_Mask(Pin_0,Enabled);
}

void GPIO_Disable_EXTI0(void){
	GPIO_EXTI_Mask(Pin_0,Disabled);
}

void GPIO_Deinit_EXTI0(void){
	GPIO_EXTI_Mask(Pin_0, Disabled);
	GPIO_EXT_Pin_ocupped[Pin_0]=0;
	NVIC_DisableIRQ(EXTI0_IRQn);
}


/*______________________________EXTI1 FOR ALL PIN_1 OF ALL PORTS______________________________*/
void EXTI1_HANDLER(void){
	/*Develop your interruption code from here*/




	/*To here*/
	GPIO_EXTI_Clean_Flag(Pin_1);
}

Status_code_t GPIO_Init_EXTI1(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode){//next task
	Status_code_t status =Success;
	Set_Port_t Port_define=Port_A;

	status = GPIO_Save_EXTI_PIN(Pin_1);
	if(status!= Success){
		return status;
	}

	switch(EXTI_Port){
	case EXTI_Port_A:
		 Port_define = Port_A;
		break;
	case EXTI_Port_B:
		 Port_define = Port_B;
		break;
	case EXTI_Port_C:
		 Port_define = Port_C;
		break;
	case EXTI_Port_D:
		 Port_define = Port_D;
		break;
	default:
		GPIO_Delete_EXTI_PIN(Pin_1);
		return OptionNotSupported;
		break;
	}

	status =  SetPinMode(Port_define, Pin_1, Input);
	if(status!=Success){
		GPIO_Delete_EXTI_PIN(Pin_1);
		return status;
	}
	SYS_ClockEnable(Enabled);
	GPIO_Set_EXTI_Line(EXTI_Port,Pin_1);
	GPIO_EXTI_Mask(Pin_1,Disabled);
	GPIO_EXTI_Trigger_seleccion(Pin_1,EXTI_mode);
	NVIC_EnableIRQ(EXTI1_IRQn);

	return Success;
}

void GPIO_Enable_EXTI1(void){
	GPIO_EXTI_Mask(Pin_1,Enabled);
}

void GPIO_Disable_EXTI1(void){
	GPIO_EXTI_Mask(Pin_1,Disabled);
}

void GPIO_Deinit_EXTI1(void){
	GPIO_EXTI_Mask(Pin_1, Disabled);
	GPIO_Delete_EXTI_PIN(Pin_1);
	NVIC_DisableIRQ(EXTI1_IRQn);
}




//////////////////////////////////////EXTI2 FOR ALL PIN_2 OF ALL PORTS/////////////////////////////
void EXTI2_HANDLER(void){
	/*Develop your interruption code from here*/




	/*To here*/
	GPIO_EXTI_Clean_Flag(Pin_2);

}

Status_code_t GPIO_Init_EXTI2(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode){//next task
	Status_code_t status =Success;
	Set_Port_t Port_define=Port_A;

	status = GPIO_Save_EXTI_PIN(Pin_2);
	if(status!= Success){
		return status;
	}

	switch(EXTI_Port){
	case EXTI_Port_A:
		 Port_define = Port_A;
		break;
	case EXTI_Port_B:
		 Port_define = Port_B;
		break;
	case EXTI_Port_C:
		 Port_define = Port_C;
		break;
	case EXTI_Port_D:
		 Port_define = Port_D;
		break;
	default:
		GPIO_Delete_EXTI_PIN(Pin_2);
		return OptionNotSupported;
		break;
	}

	status =  SetPinMode(Port_define, Pin_2, Input);
	if(status!=Success){
		GPIO_Delete_EXTI_PIN(Pin_2);
		return status;
	}
	SYS_ClockEnable(Enabled);
	GPIO_Set_EXTI_Line(EXTI_Port,Pin_2);
	GPIO_EXTI_Mask(Pin_2,Disabled);
	GPIO_EXTI_Trigger_seleccion(Pin_2,EXTI_mode);

	NVIC_EnableIRQ(EXTI2_IRQn);

	return Success;
}

void GPIO_Enable_EXTI2(void){
	GPIO_EXTI_Mask(Pin_2,Enabled);
}

void GPIO_Disable_EXTI2(void){
	GPIO_EXTI_Mask(Pin_2,Disabled);
}

void GPIO_Deinit_EXTI2(void){
	GPIO_EXTI_Mask(Pin_2, Disabled);
	GPIO_Delete_EXTI_PIN(Pin_2);
	NVIC_DisableIRQ(EXTI2_IRQn);
}

//////////////////////////////////////EXTI3 FOR ALL PIN_3 OF ALL PORTS/////////////////////////////
void EXTI3_HANDLER(void){
	/*Develop your interruption code from here*/




	/*To here*/
	GPIO_EXTI_Clean_Flag(Pin_3);

}

Status_code_t GPIO_Init_EXTI3(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode){//next task
	Status_code_t status =Success;
	Set_Port_t Port_define=Port_A;

	status = GPIO_Save_EXTI_PIN(Pin_3);
	if(status!= Success){
		return status;
	}

	switch(EXTI_Port){
	case EXTI_Port_A:
		 Port_define = Port_A;
		break;
	case EXTI_Port_B:
		 Port_define = Port_B;
		break;
	case EXTI_Port_C:
		 Port_define = Port_C;
		break;
	case EXTI_Port_D:
		 Port_define = Port_D;
		break;
	default:
		GPIO_Delete_EXTI_PIN(Pin_3);
		return OptionNotSupported;
		break;
	}

	status =  SetPinMode(Port_define, Pin_3, Input);
	if(status!=Success){
		GPIO_Delete_EXTI_PIN(Pin_3);
		return status;
	}
	SYS_ClockEnable(Enabled);
	GPIO_Set_EXTI_Line(EXTI_Port,Pin_3);
	GPIO_EXTI_Mask(Pin_3,Disabled);
	GPIO_EXTI_Trigger_seleccion(Pin_3,EXTI_mode);

	NVIC_EnableIRQ(EXTI3_IRQn);

	return Success;
}

void GPIO_Enable_EXTI3(void){
	GPIO_EXTI_Mask(Pin_3,Enabled);
}

void GPIO_Disable_EXTI3(void){
	GPIO_EXTI_Mask(Pin_3,Disabled);
}

void GPIO_Deinit_EXTI3(void){
	GPIO_EXTI_Mask(Pin_3, Disabled);
	GPIO_Delete_EXTI_PIN(Pin_3);
	NVIC_DisableIRQ(EXTI3_IRQn);
}

//////////////////////////////////////EXTI4 FOR ALL PIN_4 OF ALL PORTS/////////////////////////////
void EXTI4_HANDLER(void){
	/*Develop your interruption code from here*/




	/*To here*/
	GPIO_EXTI_Clean_Flag(Pin_4);
}

Status_code_t GPIO_Init_EXTI4(GPIO_Exti_Port_t EXTI_Port,GPIO_Exti_Config_t EXTI_mode){//next task
	Status_code_t status =Success;
	Set_Port_t Port_define=Port_A;

	status = GPIO_Save_EXTI_PIN(Pin_4);
	if(status!= Success){
		return status;
	}

	switch(EXTI_Port){
	case EXTI_Port_A:
		 Port_define = Port_A;
		break;
	case EXTI_Port_B:
		 Port_define = Port_B;
		break;
	case EXTI_Port_C:
		 Port_define = Port_C;
		break;
	case EXTI_Port_D:
		 Port_define = Port_D;
		break;
	default:
		GPIO_Delete_EXTI_PIN(Pin_4);
		return OptionNotSupported;
		break;
	}

	status =  SetPinMode(Port_define, Pin_4, Input);
	if(status!=Success){
		GPIO_Delete_EXTI_PIN(Pin_4);
		return status;
	}
	SYS_ClockEnable(Enabled);
	GPIO_Set_EXTI_Line(EXTI_Port,Pin_4);
	GPIO_EXTI_Mask(Pin_4,Disabled);
	GPIO_EXTI_Trigger_seleccion(Pin_4,EXTI_mode);

	NVIC_EnableIRQ(EXTI4_IRQn);

	return Success;
}

void GPIO_Enable_EXTI4(void){
	GPIO_EXTI_Mask(Pin_4,Enabled);
}

void GPIO_Disable_EXTI4(void){
	GPIO_EXTI_Mask(Pin_4,Disabled);
}

void GPIO_Deinit_EXTI4(void){
	GPIO_EXTI_Mask(Pin_4, Disabled);
	GPIO_Delete_EXTI_PIN(Pin_4);
	NVIC_DisableIRQ(EXTI4_IRQn);
}


////////////////////////////////ONLY 1 PIN FROM PIN_5 TO PIN_9 OF ALL PORTS/////////////////////////////
void EXTI9_5_HANDLER(void){
	/*Develop your interruption code from here*/




	/*To here*/
	GPIO_EXTI_Clean_Group_Of_Flag(Pin_5,Clear_five_bits);
}

Status_code_t GPIO_Init_EXTI9_To_EXTI5(GPIO_Exti_Port_t EXTI_Port, Pin_number_t Pin_defined, GPIO_Exti_Config_t EXTI_mode){//next task
	Status_code_t status =Success;
	Set_Port_t Port_define=Port_A;

	if(Pin_defined>9 || Pin_defined<5){
		return EXTI_Pin_Not_Allowed;
	}

	for(uint8_t i = 5;i<10;i++){
		status = GPIO_Save_EXTI_PIN(i);
		if(status!= Success){
			return status;
		}
	}

	switch(EXTI_Port){
	case EXTI_Port_A:
		 Port_define = Port_A;
		break;
	case EXTI_Port_B:
		 Port_define = Port_B;
		break;
	case EXTI_Port_C:
		 Port_define = Port_C;
		break;
	case EXTI_Port_D:
		 Port_define = Port_D;
		break;
	default:
		for(uint8_t i = 5;i<10;i++){
			GPIO_Delete_EXTI_PIN(i);
		}
		return OptionNotSupported;
		break;
	}

	status =  SetPinMode(Port_define, Pin_defined, Input);
	if(status!=Success){
		for(uint8_t i = 5;i<10;i++){
			GPIO_Delete_EXTI_PIN(i);
		}
		return status;
	}
	SYS_ClockEnable(Enabled);
	GPIO_Set_EXTI_Line(EXTI_Port,Pin_defined);
	GPIO_EXTI_Mask(Pin_defined,Disabled);
	GPIO_EXTI_Trigger_seleccion(Pin_defined,EXTI_mode);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	return Success;
}

void GPIO_Enable_EXTI9_To_EXTI5(Pin_number_t Pin_defined){
	GPIO_EXTI_Mask(Pin_defined,Enabled);
}
void GPIO_Disable_EXTI9_To_EXTI5(Pin_number_t Pin_defined){
	GPIO_EXTI_Mask(Pin_defined,Disabled);

}
void GPIO_Deinit_EXTI9_To_EXTI5(Pin_number_t Pin_defined){
	GPIO_EXTI_Mask(Pin_defined, Disabled);
	for(uint8_t i = 5;i<10;i++){
		GPIO_Delete_EXTI_PIN(i);
	}
	NVIC_DisableIRQ(EXTI9_5_IRQn);
}


////////////////////////////////ONLY 1 PIN FROM PIN_10 TO PIN_15 OF ALL PORTS/////////////////////////////



void EXTI15_10_HANDLER(void){
	/*Develop your interruption code from here*/
	uint32_t hw_count_dummy=0;



	while(hw_count_dummy<40000){hw_count_dummy++;} //dummy count to avoid debouncing


	/*To here*/
	GPIO_EXTI_Clean_Group_Of_Flag(Pin_10,Clear_six_bits);

}

Status_code_t GPIO_Init_EXTI15_To_EXTI10(GPIO_Exti_Port_t EXTI_Port, Pin_number_t Pin_defined, GPIO_Exti_Config_t EXTI_mode){//next task
	Status_code_t status =Success;
	Set_Port_t Port_define=Port_A;

	if(Pin_defined>15 || Pin_defined<10){
		return EXTI_Pin_Not_Allowed;
	}
	for(uint8_t i = 10;i<16;i++){
		status = GPIO_Save_EXTI_PIN(i);
		if(status!= Success){
			return status;
		}
	}

	switch(EXTI_Port){
	case EXTI_Port_A:
		 Port_define = Port_A;
		break;
	case EXTI_Port_B:
		 Port_define = Port_B;
		break;
	case EXTI_Port_C:
		 Port_define = Port_C;
		break;
	case EXTI_Port_D:
		 Port_define = Port_D;
		break;
	default:
		for(uint8_t i = 10;i<16;i++){
			GPIO_Delete_EXTI_PIN(i);
		}
		return OptionNotSupported;
		break;
	}

	status =  SetPinMode(Port_define, Pin_defined, Input);

	if(status!=Success){
		for(uint8_t i = 10;i<16;i++){
			GPIO_Delete_EXTI_PIN(i);
		}
		return status;
	}

	SYS_ClockEnable(Enabled);
	GPIO_Set_EXTI_Line(EXTI_Port,Pin_defined);
	GPIO_EXTI_Mask(Pin_defined,Disabled);
	GPIO_EXTI_Trigger_seleccion(Pin_defined,EXTI_mode);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	return Success;
}

void GPIO_Enable_EXTI15_To_EXTI10(Pin_number_t Pin_defined){
	GPIO_EXTI_Mask(Pin_defined,Enabled);
}

void GPIO_Disable_EXTI15_To_EXTI10(Pin_number_t Pin_defined){
	GPIO_EXTI_Mask(Pin_defined,Disabled);
}

void GPIO_Deinit_EXTI15_To_EXTI10(Pin_number_t Pin_defined){
	GPIO_EXTI_Mask(Pin_defined, Disabled);
	for(uint8_t i = 10;i<16;i++){
		GPIO_Delete_EXTI_PIN(i);
	}
	NVIC_DisableIRQ(EXTI15_10_IRQn);
}


