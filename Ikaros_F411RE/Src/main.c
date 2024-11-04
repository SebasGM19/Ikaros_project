/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "system_settings.h"
#include "gpios.h"
#include "stepper_motor.h"
#include "keypad_4x4.h"
#include "timers.h"
#include "lcd.h"
#include "adc.h"
#include "uart.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif




usart_config_t usart_config={
		115200,
		Asynchronous, //pendiente  y las demas configuraciones
		enable_TX_and_RX,
		None,
		Ignore,//pendiente
		Data_8_bits,
		Stop_1_bits
};



int main(void){
	Init_Board();


	Init_UART2(usart_config);

	while(1){

	}

	return 0;
}




