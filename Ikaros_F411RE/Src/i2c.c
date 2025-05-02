/*
 * i2c.c
 *
 *  Created on: Apr 12, 2025
 *      Author: sebas
 */

#include "i2c.h"
#include "gpios.h"
#include "timers.h"


Status_code_t I2C1_Init_Master(i2c_config_parameters_t config){

	Status_code_t status = Success;

	SetPinMode(Port_B, Pin_8, Alt_func_mode); //SCL
	SetPinMode(Port_B, Pin_9, Alt_func_mode); //SDA

	GpioSetAlternativeFunction(Port_B, Pin_8, I2C1_I2C2_I2C3);
	GpioSetAlternativeFunction(Port_B, Pin_9, I2C1_I2C2_I2C3);

	Gpio_Output_type(Port_B, Pin_8, Open_drain);
	Gpio_Output_type(Port_B, Pin_9, Open_drain);

	GpioPullUpDownState(Port_B, Pin_8, Pull_Up);
	GpioPullUpDownState(Port_B, Pin_9, Pull_Up);

	I2C_Clock(I2C1_Alt,Enabled);

	status = I2C_config(I2C1_ADDRESS,&config);


	if(status != Success){
		return status;
	}

	return status;
}

//dummy pendiete
//Status_code_t I2C1_Init_Slave(uint8_t addrs){
//
//	Status_code_t status = Success;
//
//	SetPinMode(Port_B, Pin_8, Alt_func_mode); //SCL
//	SetPinMode(Port_B, Pin_9, Alt_func_mode); //SDA
//
//	GpioSetAlternativeFunction(Port_B, Pin_8, I2C1_I2C2_I2C3);
//	GpioSetAlternativeFunction(Port_B, Pin_9, I2C1_I2C2_I2C3);
//
//	Gpio_Output_type(Port_B, Pin_8, Open_drain);
//	Gpio_Output_type(Port_B, Pin_9, Open_drain);
//
//	GpioPullUpDownState(Port_B, Pin_8, Pull_Up);
//	GpioPullUpDownState(Port_B, Pin_9, Pull_Up);
//
//	I2C_Clock(I2C1_Alt,Enabled);
//
//	I2C_Reset_Protocol(I2C1_ADDRESS);
//	I2C_Peripherial_Mode(I2C1_ADDRESS, Disabled);
//
//	status = I2C_Set_Clock_frecuency(I2C1_ADDRESS, APB1_CLOCK);
//
//	if(status != Success){
//		return status;
//	}
//
//	I2C_slave_config(I2C1_ADDRESS,addrs);
//	I2C_ACK_bit(I2C1_ADDRESS, Enabled);
//
//	I2C_Peripherial_Mode(I2C1_ADDRESS, Enabled);
//
//
//
//
//	return status;
//}

Status_code_t I2C3_Init_Master(i2c_config_parameters_t config){

	Status_code_t status = Success;

	SetPinMode(Port_A, Pin_8, Alt_func_mode); //SCL
	SetPinMode(Port_C, Pin_9, Alt_func_mode); //SDA

	GpioSetAlternativeFunction(Port_A, Pin_8, I2C1_I2C2_I2C3);
	GpioSetAlternativeFunction(Port_C, Pin_9, I2C1_I2C2_I2C3);

	Gpio_Output_type(Port_A, Pin_8, Open_drain);
	Gpio_Output_type(Port_C, Pin_9, Open_drain);

	GpioPullUpDownState(Port_A, Pin_8, Pull_Up);
	GpioPullUpDownState(Port_C, Pin_9, Pull_Up);

	I2C_Clock(I2C3_Alt,Enabled);

	status = I2C_config(I2C3_ADDRESS,&config);

	if(status != Success){
		return status;
	}


	return status;

}
 //dummy pendiente
//Status_code_t I2C3_Init_Slave(uint8_t addrs){
//
//	Status_code_t status = Success;
//
//	SetPinMode(Port_A, Pin_8, Alt_func_mode); //SCL
//	SetPinMode(Port_C, Pin_9, Alt_func_mode); //SDA
//
//	GpioSetAlternativeFunction(Port_A, Pin_8, I2C1_I2C2_I2C3);
//	GpioSetAlternativeFunction(Port_C, Pin_9, I2C1_I2C2_I2C3);
//
//	Gpio_Output_type(Port_A, Pin_8, Open_drain);
//	Gpio_Output_type(Port_C, Pin_9, Open_drain);
//
//	GpioPullUpDownState(Port_A, Pin_8, Pull_Up);
//	GpioPullUpDownState(Port_C, Pin_9, Pull_Up);
//
//	I2C_Clock(I2C3_Alt,Enabled);
//
//
//	I2C_Reset_Protocol(I2C3_ADDRESS);
//	I2C_Peripherial_Mode(I2C3_ADDRESS, Disabled);
//	status = I2C_Set_Clock_frecuency(I2C3_ADDRESS, APB1_CLOCK);
//
//	if(status != Success){
//		return status;
//	}
//
//	I2C_slave_config(I2C3_ADDRESS,addrs);
//	I2C_ACK_bit(I2C3_ADDRESS, Enabled);
//
//	I2C_Peripherial_Mode(I2C3_ADDRESS, Enabled);
//
//
//	return status;
//
//}


Status_code_t I2C_Write(I2C_alternative_t i2c_alt,uint8_t addr, uint8_t* data_sent, uint32_t data_size){
	//bit 0 for write
	Status_code_t status = Success;
	I2CMapAddr_t I2C_addr = I2C1_ADDRESS;

	if(i2c_alt == I2C3_Alt){
		I2C_addr = I2C3_ADDRESS;
	}

	__IO uint8_t *I2C_DR_Reg = (__IO uint8_t *)(I2C_addr + I2C_DR);
	__IO uint32_t *I2C_SR2_Reg = (__IO uint32_t *)(I2C_addr + I2C_SR2);

	////////////////////////////////////////////// PART1 /////////////////////////////////////////////////
	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
		return status;
	}
	TIM11_Start();

	while(I2C_Busy_State(I2C_addr) && !TIM11_GET_interrupt_flag_status()){}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		return Timeout;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	////////////////////////////////////////////// PART2 /////////////////////////////////////////////////

	I2C_START_bit(I2C_addr,Enabled); //START bit

	////////////////////////////////////////////// PART3 /////////////////////////////////////////////////

	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
		I2C_STOP_bit(I2C_addr, Enabled); // STOP
		return status;
	}
	TIM11_Start();

	while(!I2C_status_flag(I2C_addr, I2C_SB) && !TIM11_GET_interrupt_flag_status()){

	    if(I2C_status_flag(I2C_addr, I2C_BERR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
	    	I2C_Reset_BERR_bit(I2C_addr);
	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
	        return I2C_bus_error;
	    }

	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		I2C_STOP_bit(I2C_addr, Enabled);// STOP
		return Timeout;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();
	////////////////////////////////////////////// PART4 /////////////////////////////////////////////////

	*I2C_DR_Reg = (uint8_t)(addr<<1); //recorremos en 1 para que al final tenga un 0 representando un write

	////////////////////////////////////////////// PART5 /////////////////////////////////////////////////

	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
        I2C_STOP_bit(I2C_addr, Enabled); // STOP
		return status;
	}
	TIM11_Start();

	while(!I2C_status_flag(I2C_addr, I2C_ADDR) && !TIM11_GET_interrupt_flag_status()){

		if(I2C_status_flag(I2C_addr, I2C_AF)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
	        I2C_Reset_ACK_bit(I2C_addr);
	        I2C_STOP_bit(I2C_addr, Enabled); // Inmediatamente mandamos STOP
			return I2C_NACK;
		}

	    if(I2C_status_flag(I2C_addr, I2C_BERR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
	    	I2C_Reset_BERR_bit(I2C_addr);
	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
	        return I2C_bus_error;
	    }
	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
        I2C_STOP_bit(I2C_addr, Enabled); // Inmediatamente mandamos STOP
		return Timeout;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();


	////////////////////////////////////////////// PART6 /////////////////////////////////////////////////
	(void)*I2C_SR2_Reg;// Dummy read para limpiar la bandera ADDR

	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
        I2C_STOP_bit(I2C_addr, Enabled); // STOP
		return status;
	}
	TIM11_Start();

	for(uint32_t reg =0; reg<data_size; reg++){

		while(!I2C_status_flag(I2C_addr, I2C_TxE) && !TIM11_GET_interrupt_flag_status()){

			if(I2C_status_flag(I2C_addr, I2C_AF)){
				TIM11_Deinit();
				TIM11_clear_interrupt_flag();
		        I2C_Reset_ACK_bit(I2C_addr);
		        I2C_STOP_bit(I2C_addr, Enabled); // STOP
				return I2C_NACK;
			}
		    if(I2C_status_flag(I2C_addr, I2C_BERR)){
				TIM11_Deinit();
				TIM11_clear_interrupt_flag();
		    	I2C_Reset_BERR_bit(I2C_addr);
		        I2C_STOP_bit(I2C_addr, Enabled); // STOP
		        return I2C_bus_error;
		    }
		}

    	if( TIM11_GET_interrupt_flag_status() ){
    		TIM11_Deinit();
    		TIM11_clear_interrupt_flag();
	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
    		return Timeout;
    	}

		*I2C_DR_Reg = (uint8_t)(*(data_sent + reg));

	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	////////////////////////////////////////////// PART7 /////////////////////////////////////////////////

	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
        I2C_STOP_bit(I2C_addr, Enabled); // STOP
		return status;
	}
	TIM11_Start();

	while(!I2C_status_flag(I2C_addr, I2C_BTF) && !TIM11_GET_interrupt_flag_status()){

	    if(I2C_status_flag(I2C_addr, I2C_BERR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
	    	I2C_Reset_BERR_bit(I2C_addr);
	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
	        return I2C_bus_error;
	    }

	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
        I2C_STOP_bit(I2C_addr, Enabled); // STOP
		return Timeout;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	I2C_STOP_bit(I2C_addr,Enabled);

	return Success;

}

Status_code_t I2C_Read(I2C_alternative_t i2c_alt, uint8_t addr, uint8_t* data_received, uint32_t data_size_expected){
	//bit 1 for read
	Status_code_t status = Success;
	I2CMapAddr_t I2C_addr = I2C1_ADDRESS;

	if(i2c_alt == I2C3_Alt){
		I2C_addr = I2C3_ADDRESS;
	}

	__IO uint8_t *I2C_DR_Reg = (__IO uint8_t *)(I2C_addr + I2C_DR);
	__IO uint32_t *I2C_SR2_Reg = (__IO uint32_t *)(I2C_addr + I2C_SR2);


	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
		return status;
	}
	TIM11_Start();

	while(I2C_Busy_State(I2C_addr) && !TIM11_GET_interrupt_flag_status()){}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
		return Timeout;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();


    I2C_ACK_bit(I2C_addr, Enabled); // Aseguramos que el ACK esté habilitado al principio

    I2C_START_bit(I2C_addr,Enabled); //START

	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
        I2C_STOP_bit(I2C_addr, Enabled); // STOP
		return status;
	}
	TIM11_Start();

	while(!I2C_status_flag(I2C_addr, I2C_SB) && !TIM11_GET_interrupt_flag_status()){

	    if(I2C_status_flag(I2C_addr, I2C_BERR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
	    	I2C_Reset_BERR_bit(I2C_addr);
	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
	        return I2C_bus_error;
	    }

	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
        I2C_STOP_bit(I2C_addr, Enabled);// STOP
		return Timeout;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();

	*I2C_DR_Reg = (uint8_t)(addr<<1 | 1); //recorremos en 1 y agregamos 1 para que sea read

	status = TIM11_Init(I2C_MAX_TIMEOUT);
	if(status != Success){
        I2C_STOP_bit(I2C_addr, Enabled); // STOP
		return status;
	}
	TIM11_Start();

	while(!I2C_status_flag(I2C_addr, I2C_ADDR) && !TIM11_GET_interrupt_flag_status()){

		if(I2C_status_flag(I2C_addr, I2C_AF)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
	        I2C_Reset_ACK_bit(I2C_addr);
	        I2C_STOP_bit(I2C_addr, Enabled); // Inmediatamente mandamos STOP
			return I2C_NACK;
		}

	    if(I2C_status_flag(I2C_addr, I2C_BERR)){
			TIM11_Deinit();
			TIM11_clear_interrupt_flag();
	    	I2C_Reset_BERR_bit(I2C_addr);
	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
	        return I2C_bus_error;
	    }
	}

	if( TIM11_GET_interrupt_flag_status() ){
		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
        I2C_STOP_bit(I2C_addr, Enabled); // Inmediatamente mandamos STOP
		return Timeout;
	}

	TIM11_Deinit();
	TIM11_clear_interrupt_flag();


	if(data_size_expected == 1){
		I2C_ACK_bit(I2C_addr, Disabled);
        (void)*I2C_SR2_Reg;
        I2C_STOP_bit(I2C_addr, Enabled);

    	status = TIM11_Init(I2C_MAX_TIMEOUT);
    	if(status != Success){
    		return status;
    	}
    	TIM11_Start();

        if(I2C_status_flag(I2C_addr, I2C_OVR)){
            I2C_Reset_OVR_bit(I2C_addr);
            I2C_STOP_bit(I2C_addr, Enabled); // STOP inmediato
            return I2C_overrun_error;
        }

        while(!I2C_status_flag(I2C_addr, I2C_RxNE) && !TIM11_GET_interrupt_flag_status()){

    	    if(I2C_status_flag(I2C_addr, I2C_BERR)){
    			TIM11_Deinit();
    			TIM11_clear_interrupt_flag();
    	    	I2C_Reset_BERR_bit(I2C_addr);
    	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
    	        return I2C_bus_error;
    	    }

        }

        if(I2C_status_flag(I2C_addr, I2C_OVR)){
            I2C_Reset_OVR_bit(I2C_addr);
            I2C_STOP_bit(I2C_addr, Enabled); // STOP inmediato
            return I2C_overrun_error;
        }

    	if( TIM11_GET_interrupt_flag_status() ){
    		TIM11_Deinit();
    		TIM11_clear_interrupt_flag();
    		return Timeout;
    	}

    	TIM11_Deinit();
    	TIM11_clear_interrupt_flag();

        data_received[0] = (uint8_t)(*I2C_DR_Reg);

	}else{
        (void)*I2C_SR2_Reg;
		status = TIM11_Init(I2C_MAX_TIMEOUT);
		if(status != Success){
	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
			return status;
		}
		TIM11_Start();

		for(uint32_t i = 0; i<data_size_expected; i++){
			if(i == (data_size_expected - 2)){
				I2C_ACK_bit(I2C_addr, Disabled);
		        I2C_STOP_bit(I2C_addr, Enabled); // STOP

			}

	        if(I2C_status_flag(I2C_addr, I2C_OVR)){
	            I2C_Reset_OVR_bit(I2C_addr);
	            I2C_STOP_bit(I2C_addr, Enabled); // STOP inmediato
	            return I2C_overrun_error;
	        }

	        while(!I2C_status_flag(I2C_addr, I2C_RxNE) && !TIM11_GET_interrupt_flag_status()){

	    	    if(I2C_status_flag(I2C_addr, I2C_BERR)){
	    			TIM11_Deinit();
	    			TIM11_clear_interrupt_flag();
	    	    	I2C_Reset_BERR_bit(I2C_addr);
	    	        I2C_STOP_bit(I2C_addr, Enabled); // STOP
	    	        return I2C_bus_error;
	    	    }

	        }

	        if(I2C_status_flag(I2C_addr, I2C_OVR)){
	            I2C_Reset_OVR_bit(I2C_addr);
	            I2C_STOP_bit(I2C_addr, Enabled); // STOP inmediato
	            return I2C_overrun_error;
	        }

	    	if( TIM11_GET_interrupt_flag_status() ){
	    		TIM11_Deinit();
	    		TIM11_clear_interrupt_flag();
		        I2C_STOP_bit(I2C_addr, Enabled); // STOP
	    		return Timeout;
	    	}


	    	data_received[i] = (uint8_t)(*I2C_DR_Reg);
		}

		TIM11_Deinit();
		TIM11_clear_interrupt_flag();
	}


	return Success;

}


Status_code_t I2C1_Deinit(void){

	Gpio_Output_type(Port_B, Pin_8, Push_pull);
	Gpio_Output_type(Port_B, Pin_9, Push_pull);

	GpioPullUpDownState(Port_B, Pin_8, No_pull_No_Down);
	GpioPullUpDownState(Port_B, Pin_9, No_pull_No_Down);

	I2C_Clock(I2C1_Alt,Disabled);

	return Success;
}

Status_code_t I2C3_Deinit(void){

	Gpio_Output_type(Port_A, Pin_8, Push_pull);
	Gpio_Output_type(Port_C, Pin_9, Push_pull);

	GpioPullUpDownState(Port_A, Pin_8, No_pull_No_Down);
	GpioPullUpDownState(Port_C, Pin_9, No_pull_No_Down);

	I2C_Clock(I2C3_Alt,Disabled);


	return Success;

}

//auxiliar functions


void I2C_Clock(I2C_alternative_t I2C, Enabled_Disabled_t state){

	__IO uint32_t *pClockControlReg = (__IO uint32_t *)(RCC_ADDRESS + RCC_OFFSET_APB1ENR);

    *pClockControlReg = (state) ? (*pClockControlReg | (Enabled << I2C)) : (*pClockControlReg & ~(Enabled << I2C));

}


Status_code_t I2C_config(I2CMapAddr_t I2C_addr,i2c_config_parameters_t *config){
	Status_code_t status = Success;

	I2C_Reset_Protocol(I2C_addr);
	I2C_Peripherial_Mode(I2C_addr, Disabled);
	status = I2C_Set_Clock_frecuency(I2C_addr, APB1_CLOCK);

	if(status != Success){
		return status;
	}

	I2C_Speed_Mode(I2C_addr,config, APB1_CLOCK);

	I2C_Peripherial_Mode(I2C_addr, Enabled);


	return status;
}


void I2C_slave_config(I2CMapAddr_t I2C_addr,uint8_t slave_addrs){
	__IO uint32_t *I2C_OAR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_OAR1);

	*I2C_OAR1_Reg &= ~(I2C_ADDMODE); //set as 7 bit dirction addrs
	*I2C_OAR1_Reg |= (I2C_OAR1_BIT14); //bit 14 always in 1

	*I2C_OAR1_Reg &= ~(Clear_seven_bits<<I2C_ADD1); //set as 7 bit dirction addrs
	*I2C_OAR1_Reg |= (slave_addrs<<I2C_ADD1); //set as 7 bit dirction addrs

}


void I2C_Peripherial_Mode(I2CMapAddr_t I2C_addr, Enabled_Disabled_t state){
	__IO uint32_t *I2C_CR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_CR1);

    *I2C_CR1_Reg = (state) ? (*I2C_CR1_Reg | (I2C_PE)) : (*I2C_CR1_Reg & ~(I2C_PE));

}



void I2C_Reset_Protocol(I2CMapAddr_t I2C_addr){
	__IO uint32_t *I2C_CR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_CR1);

	 *I2C_CR1_Reg |=  I2C_SWRST;
	 *I2C_CR1_Reg &=  ~(I2C_SWRST);

}



Status_code_t I2C_Set_Clock_frecuency(I2CMapAddr_t I2C_addr, uint32_t peripherial_clock){
	__IO uint32_t *I2C_CR2_Reg = (__IO uint32_t *)(I2C_addr + I2C_CR2);

	if(peripherial_clock < 2000000 || peripherial_clock > 50000000){
		return I2C_frecuency_not_supported;
	}

	*I2C_CR2_Reg &= ~(Clear_six_bits<<I2C_FREQ_CR2);

	*I2C_CR2_Reg = (peripherial_clock/1000000U)<<I2C_FREQ_CR2;

    return Success;
}


void I2C_Set_FM_Duty(I2CMapAddr_t I2C_addr, i2c_duty_t duty){
	__IO uint32_t *I2C_CCR_Reg = (__IO uint32_t *)(I2C_addr + I2C_CCR);

    *I2C_CCR_Reg = (duty) ? (*I2C_CCR_Reg | (I2C_DUTY)) : (*I2C_CCR_Reg & ~(I2C_DUTY));

}


Status_code_t I2C_Speed_Mode(I2CMapAddr_t I2C_addr, i2c_config_parameters_t* config, uint32_t peripherial_clock){

	__IO uint32_t *I2C_CCR_Reg = (__IO uint32_t *)(I2C_addr + I2C_CCR);
	Status_code_t status = Success;

	if(config->baudrate == StandarMode_100Kbps){
		*I2C_CCR_Reg &= ~(I2C_F_S);

	}else if(config->baudrate == FastMode_400Kbps){
		*I2C_CCR_Reg |= (I2C_F_S);

		I2C_Set_FM_Duty(I2C_addr, config->duty);
	}

	status = I2C_Set_CCR(I2C_addr, config, peripherial_clock);

	if(status != Success){
		return status;
	}

	I2C_Set_Trise(I2C_addr,config->baudrate,peripherial_clock);

	return Success;
}


Status_code_t I2C_Set_CCR(I2CMapAddr_t I2C_addr, i2c_config_parameters_t *config, uint32_t peripherial_clock){
	__IO uint32_t *I2C_CCR_Reg = (__IO uint32_t *)(I2C_addr + I2C_CCR);
	uint32_t CCR_value =0;

	*I2C_CCR_Reg &= ~(Clear_twelve_bits<<I2C_CCR_enum);

	/*example for duty_16_9
		T_SCL = T_high + T_low
	      = (9 × CCR × tPCLK1) + (16 × CCR × tPCLK1)
	      = (25 × CCR × tPCLK1)

	      then it must be in to frequency domain
	      SCL = 1 / T_SCL
    			= 1 / (25 × CCR × tPCLK1)
    			= Fpclk1 / (25 × CCR)

    	   then to find CCR
    	   CCR = Fpclk1/(25*SCL) Final equation
	*/

	if(config->baudrate == StandarMode_100Kbps){
		CCR_value = (uint32_t)(peripherial_clock / (2U*config->baudrate)); // peripherilCLK / 2U*baudrate
	}else{
		if(config->duty == duty_2_1){
			CCR_value = (uint32_t)(peripherial_clock / (3U*config->baudrate)); // peripherilCLK / 3U*baudrate
		}else{
			CCR_value = (uint32_t)(peripherial_clock / (25U*config->baudrate)); // peripherilCLK / 25U*baudrate
		}
	}

	if(CCR_value < 1 || CCR_value >= 0xFFF){ //less or more than the bits of the register
		return I2C_clock_value_out_of_range;
	}

	*I2C_CCR_Reg |= (CCR_value<<I2C_CCR_enum);

	return Success;

}

void I2C_Set_Trise(I2CMapAddr_t I2C_addr, i2c_baudrate_t baudrate, uint32_t peripherial_clock){
	__IO uint32_t * I2C_TRISE_reg = (__IO uint32_t*)(I2C_addr + I2C_TRISE);

	*I2C_TRISE_reg &= ~(Clear_six_bits<<I2C_TRISE_enum);

	uint32_t Trise_value = 0;
	if(baudrate == StandarMode_100Kbps){

		Trise_value = (uint32_t)(((double)(MAX_SM_RISE_TIME_ns / (double)(1.0f / peripherial_clock)) / DIVIDE_FOR_TRISE_ns) + 1U);

	}else if(baudrate == FastMode_400Kbps){

		Trise_value = (uint32_t)(((double)(MAX_FM_RISE_TIME_ns / (double)(1.0f / peripherial_clock)) / DIVIDE_FOR_TRISE_ns) + 1U);

	}

	*I2C_TRISE_reg |= Trise_value<<I2C_TRISE_enum;

}




bool I2C_status_flag(I2CMapAddr_t I2C_addr, I2C_SR1_t flag){

	__I uint32_t *I2C_SR1_Reg = (__I uint32_t *)(I2C_addr + I2C_SR1);
	return ((*I2C_SR1_Reg & flag) == flag); //will return the state


}

bool I2C_Busy_State(I2CMapAddr_t I2C_addr){
	__I uint32_t *I2C_SR2_Reg = (__I uint32_t *)(I2C_addr + I2C_SR2);

	return ((*I2C_SR2_Reg & I2C_BUSY) == I2C_BUSY); //will return true if is busy and 0 if is not

}

void I2C_START_bit(I2CMapAddr_t I2C_addr, Enabled_Disabled_t state){
	__IO uint32_t *I2C_CR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_CR1);

    *I2C_CR1_Reg = (state) ? (*I2C_CR1_Reg | (I2C_START)) : (*I2C_CR1_Reg & ~(I2C_START));

}

void I2C_STOP_bit(I2CMapAddr_t I2C_addr, Enabled_Disabled_t state){
	__IO uint32_t *I2C_CR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_CR1);

    *I2C_CR1_Reg = (state) ? (*I2C_CR1_Reg | (I2C_STOP)) : (*I2C_CR1_Reg & ~(I2C_STOP));

}

void I2C_ACK_bit(I2CMapAddr_t I2C_addr, Enabled_Disabled_t state){
	__IO uint32_t *I2C_CR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_CR1);

    *I2C_CR1_Reg = (state) ? (*I2C_CR1_Reg | (I2C_ACK)) : (*I2C_CR1_Reg & ~(I2C_ACK));

}


void I2C_Reset_ACK_bit(I2CMapAddr_t I2C_addr){
	__IO uint32_t *I2C_SR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_SR1);

    *I2C_SR1_Reg &= ~(I2C_AF);

}

void I2C_Reset_OVR_bit(I2CMapAddr_t I2C_addr){
	__IO uint32_t *I2C_SR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_SR1);

    *I2C_SR1_Reg &= ~(I2C_OVR);

}

void I2C_Reset_BERR_bit(I2CMapAddr_t I2C_addr){
	__IO uint32_t *I2C_SR1_Reg = (__IO uint32_t *)(I2C_addr + I2C_SR1);

    *I2C_SR1_Reg &= ~(I2C_BERR);

}



