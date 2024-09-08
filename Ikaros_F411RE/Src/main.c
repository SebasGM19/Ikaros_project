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
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

ADC_channel_t chan_to_read = Channel_0;
uint32_t count =0;
uint8_t str_save_data[6]={};


int main(void){
	Init_Board();
	uint32_t adc_value =0;
	float voltaje_0 = 0.0f;
	uint8_t str_save_data[6]={};


	lcd_init(lcd_PortB);
//	ADC_Init(RES_12_bits);
//	ADC_Configure_Channel(Channel_1);
//	ADC_Configure_Channel(Channel_0);
//	GPIO_Init_EXTI15_To_EXTI10(EXTI_Port_C, Pin_13, 0);
//
//	GPIO_Enable_EXTI15_To_EXTI10(Pin_13);
//    SetPinMode(Port_C, Pin_6, Output);
//    GPIO_DigitalWrite(Port_C, Pin_6, High);
//    GPIO_DigitalWrite(Port_C, Pin_6, Low);


	uint32_t count =0;
	TIM3_PWM_Init(TIM3_CH3);

bool To_on =true;

	while(1){

		TIM3_PWM_start_channel_duty_porcent(count, TIM3_CH3);
		Delay(50000);
		if(To_on){
			count++;
			if(count>=100){
				To_on=false;
			}
		}else{
			count--;
			if(count<=0){
				To_on=true;
			}
		}
		lcd_printXY(0, 0,"PWM Test:       ", strlen((const char *)"PWM Test:       "));
		memset(str_save_data,'\0',6);

		itoa(count, str_save_data, 10);
		lcd_printXY(10, 0,str_save_data, strlen((const char *)str_save_data));



//		Delay(250000);
//		memset(str_save_data,'\0',6);
//		chan_to_read = get_channel();
//		adc_value = ADC_Read(chan_to_read);
//		voltaje_0 = (float)((3.3f*adc_value)/4096.0f);
//
//		ftoa(voltaje_0, str_save_data, 2);
//		if(chan_to_read){
//			lcd_printXY(0, 0,"CHN1: ", strlen((const char *)"CHN1: "));
//		}else{
//			lcd_printXY(0, 0,"CHN0: ", strlen((const char *)"CHN0: "));
//		}
//
//		lcd_printXY(0, 1,"EXT Count: ", strlen((const char *)"EXT Count: "));
//
//		lcd_printXY(6, 0,str_save_data, strlen((const char *)str_save_data));
//
//		memset(str_save_data,'\0',6);
//		count = get_EXT_count();
//		itoa(count, str_save_data, 10);
//
//		lcd_printXY(11, 1,str_save_data, strlen((const char *)str_save_data));

	}
	return 0;
}



//uint32_t adc_value =0;
//
//int main(void){
//
//	Init_Board();
//
////	SetPinMode(Port_A, Pin_1, Output);
////	GPIO_DigitalWrite(Port_A, Pin_1, 1);
//
//	lcd_init(lcd_PortB);
//	ADC_Init(RES_12_bits);
//	TIM3_Init(500);
//	ADC_Configure_Channel(Channel_1);
//	ADC_Configure_Channel(Channel_0);
//	TIM3_Start();
//
//		uint32_t adc_value=0;
//		float voltaje_0=0.0f;
//		uint8_t str_save_data[6]={};
//
//	while(1){
//		Delay(500000);
//		adc_value = ADC_Read(Channel_0);
//		voltaje_0 = (float)((3.3f*adc_value)/4096.0f);
//
//		ftoa(voltaje_0, str_save_data, 5);
//		lcd_printXY(0, 0,"CHN0: ", strlen((const char *)"CHN0: "));
//
//		lcd_printXY(6, 0,str_save_data, strlen((const char *)str_save_data));
//
//	}
//
//
//
//	return 0;
//
//}

//example 1
/*
uint8_t seq_motor_UNIPOLAR[8]={1,3,2,6,4,12,8,9};
int8_t start=7;

int main(void) {

	uint32_t data_key =0;
	uint32_t volatile sentido_giro =0;

	lcd_init(lcd_PortB);
	Init_keypad(keypad_PortC);
	Init_4bits_Stepper_Motor(PortB_Op1);

	SetPinMode(Port_A, Pin_5, Output);
	SetPinMode(Port_C, Pin_13, Input);
	GPIO_DigitalWrite(Port_A, Pin_5, Low);


	lcd_printXY(0, 0, "COUNT TO:       ",16);
	data_key = print_keypad(keypad_PortC, 10, 0);
	set_cuenta(data_key);
	Delay(15000);
	lcd_printXY(0, 0, "ROTATION:       ",16);
	lcd_printXY(0, 1, "LEFT=0   RIGHT=1",16);
	sentido_giro = print_keypad(keypad_PortC, 10, 0);
	lcd_clean_screen();

	TIM5_Init(1000000);
	TIM3_Init(500);
	TIM5_Start();
	TIM3_Start();
	lcd_printXY(0, 0, "COUNT: 0        ",16);

	while(1){
		while(GPIO_DigitalRead(Port_C,Pin_13)){
			if(sentido_giro){					//1 to Right
				Write_4bits_Stepper_Motor(PortB_Op1,seq_motor_UNIPOLAR[start]);
				start++;
				if(start>7){
					start=0;
				}
			}else{								//0 to Left
				Write_4bits_Stepper_Motor(PortB_Op1,seq_motor_UNIPOLAR[start]);
				start--;
				if(start<0){
					start=7;
				}
			}

			Delay(2000);
		}

		TIM5_Stop();
		TIM3_Stop();
		lcd_clean_screen();
		lcd_printXY(0, 0, "COUNT TO:       ",16);
		data_key = print_keypad(keypad_PortC, 10, 0);
		set_cuenta(data_key);
		Delay(15000);
		lcd_printXY(0, 0, "ROTATION:       ",16);
		lcd_printXY(0, 1, "LEFT=0   RIGHT=1",16);

		sentido_giro = print_keypad(keypad_PortC, 10, 0);

		lcd_clean_screen();
		TIM5_Start();
		TIM3_Start();
		lcd_printXY(0, 0, "COUNT: 0        ",16);

	}

	TIM5_Deinit();
	TIM3_Deinit();

	return 0;
}

*/



//int main(void){
//	lcd_init(lcd_PortB);
//	Init_keypad(keypad_PortA);
//	uint8_t longitude = (sizeof(titulo1)/sizeof(titulo1[0]))-1;
//	uint8_t longitude2 = (sizeof(titulo2)/sizeof(titulo2[0]))-1;
////	uint8_t longitude3 = (sizeof(titulo3)/sizeof(titulo3[0]))-1;
//
//
//
//	while(1){
//		lcd_printXY(0, 0, titulo1, longitude);
//		data_key = print_keypad(keypad_PortA, 4, 0);
//		Delay(1000000);
//		lcd_printXY(0, 0, titulo2, longitude2);
//		Delay(1000000);
//
//	}
//	return 0;
//}



//int main(void){
//	lcd_init(lcd_PortB);
//	uint8_t longitude = (sizeof(titulo1)/sizeof(titulo1[0]))-1;
//	uint8_t longitude2 = (sizeof(titulo2)/sizeof(titulo2[0]))-1;
//	uint8_t longitude3 = (sizeof(titulo3)/sizeof(titulo3[0]))-1;
//	uint8_t longitude4 = (sizeof(titulo4)/sizeof(titulo4[0]))-1;
//	uint8_t longitude5 = (sizeof(titulo5)/sizeof(titulo5[0]))-1;
//	uint8_t longitude6 = (sizeof(titulo6)/sizeof(titulo6[0]))-1;
//	uint8_t longitude7 = (sizeof(titulo7)/sizeof(titulo7[0]))-1;
//
//
////	SetPinMode(Port_C, Pin_13, Input);
//
//
//
//
//	while(1){
//	//	TIM5_Init(1000000);
////	Init_keypad(keypad_PortA);
//	lcd_printXY(2, 0, titulo1, longitude);
//	lcd_clean_screen();
//	lcd_print(titulo2, longitude2);
//	Delay(1000000);
//
//
//	for(uint8_t i =0; i<4; i++){
//		lcd_move_display(move_right);
//		Delay(200000);
//	}
//	Delay(1000000);
//	for(uint8_t i =0; i<4; i++){
//		lcd_move_display(move_left);
//		Delay(200000);
//	}
//
//	lcd_clean_screen();
//	Delay(1000000);
//
//	lcd_printXY(5, 1, titulo7, longitude7);
//	Delay(1000000);
//
//	for(uint8_t i =0; i<8; i++){
//		lcd_move_cursor(move_left);
////		Delay(5000);
//	}
////	Delay(1000000);
//
//	lcd_print(titulo6, longitude6);
//	Delay(1000000);
//
//	lcd_clean_screen();
//	Delay(1000000);
//
//
//
//
//	}
//	return 0;
//}

//int main(void){
//
//	SetPinMode(Port_B, Pin_9, Input);
//	SetPinMode(Port_A, Pin_5, Output);
//	GPIO_DigitalWrite(Port_A, Pin_5, High);
//	GPIO_DigitalWrite(Port_A, Pin_5, Low);
//
//	GpioPullUpDownState(Port_B, Pin_9, Pull_Up);
//
//	while(1){
//
//		if(GPIO_DigitalRead(Port_B,Pin_9) == Low){
//
//			GPIO_DigitalWrite(Port_A, Pin_5, High);
//
//			while(GPIO_DigitalRead(Port_B,Pin_9) == Low){
//
//			}
//		}else{
//			GPIO_DigitalWrite(Port_A, Pin_5, Low);
//
//		}
//
//	}
//	return 0;
//}




//example for GPIOS and stepper_motor

//int main(void)
//{
//
//	SetPinMode(Port_B, Pin_8, Output);
//	SetPinMode(Port_B, Pin_9, Output);
//	SetPinMode(Port_C, Pin_13, Input);
//
//
////	Init_8bits_Stepper_Motor();
////	uint8_t sec_8bit_linea[16]={129,128,192,64,96,32,48,16,24,8,12,4,6,2,3,1};
////	int8_t start_8bit=15;
//
//	Init_4bits_Stepper_Motor(PortA_Op1);
//	Init_4bits_Stepper_Motor(PortA_Op2);
//
//
//	uint8_t sec_4bit_linea_UNIPOLAR[8]={1,3,2,6,4,12,8,9};
////	int8_t start_4bit=7;
//	int8_t start_4bit2=7;
//
//	uint8_t sentido =0;
//
//	while(1){
//
//		if(GPIO_DigitalRead(Port_C,Pin_13) == Low){
//
//			GPIO_DigitalWrite(Port_B, Pin_8, sentido);
//			GPIO_DigitalWrite(Port_B, Pin_9, !sentido);
//			sentido = !sentido;
//			while(GPIO_DigitalRead(Port_C,Pin_13) == Low){
//
//			}
//		}
//
//		if(sentido==0){
//			Write_4bits_Stepper_Motor(PortA_Op1,sec_4bit_linea[start_4bit]);
//			Write_4bits_Stepper_Motor(PortA_Op2,sec_4bit_linea[start_4bit2]);
//			start_4bit++;
//			if(start_4bit>7){
//				start_4bit=0;
//			}
//
//			start_4bit2--;
//			if(start_4bit2<0){
//				start_4bit2=7;
//			}
//			Delay(30);
//		}else{
//			Write_4bits_Stepper_Motor(PortA_Op1,sec_4bit_linea[start_4bit]);
//			Write_4bits_Stepper_Motor(PortA_Op2,sec_4bit_linea[start_4bit2]);
//			start_4bit--;
//			if(start_4bit<0){
//				start_4bit=7;
//			}
//
//			start_4bit2++;
//			if(start_4bit2>7){
//				start_4bit2=0;
//			}
//			Delay(30);
//		}
//
//	};
//
//}
//
//
