/*
 * i2c.h
 *
 *  Created on: Apr 12, 2025
 *      Author: sebas
 */

#ifndef DRIVERS_I2C_H_
#define DRIVERS_I2C_H_

#include "system_settings.h"

#define MAX_SM_RISE_TIME_ns  (1000)
#define MAX_FM_RISE_TIME_ns  (300)
#define DIVIDE_FOR_TRISE_ns  (1000000000)
#define I2C_MAX_TIMEOUT		 (3000)


typedef enum{
	I2C1_Alt = 21U, 		//SCL:(B8)  SDA:(B9) //21 means RCC APB1 bit 21
	I2C3_Alt = 23U, 		//SCL:(A8)  SDA:(C9) //21 means RCC APB1 bit 23

}I2C_alternative_t;

typedef enum{
	I2C_CR1   =		0x00U,
	I2C_CR2   =		0x04U,
	I2C_OAR1  =		0x08U,
	I2C_OAR2  = 	0x0CU,
	I2C_DR    = 	0x10U,
	I2C_SR1   = 	0x14U,
	I2C_SR2   = 	0x18U,
	I2C_CCR   = 	0x1CU,
	I2C_TRISE = 	0x20U,
	I2C_FLTR  = 	0x24U,

}I2C_register_offset_t;


typedef enum{
	 I2C_PE		   = (1U<<0U),
	 I2C_SMBUS     = (1U<<1U),
	 I2C_SMBTYPE   = (1U<<3U),
	 I2C_ENARP     = (1U<<4U),
	 I2C_ENPEC     = (1U<<5U),
	 I2C_ENGC      = (1U<<6U),
	 I2C_NOSTRETCH = (1U<<7U),
	 I2C_START     = (1U<<8U),
	 I2C_STOP      = (1U<<9U),
	 I2C_ACK       = (1U<<10U),
	 I2C_POS       = (1U<<11U),
	 I2C_PEC       = (1U<<12U),
	 I2C_ALERT	   = (1U<<13U),
	 I2C_SWRST     = (1U<<15U),

}I2C_CR1_t;

typedef enum{
	 I2C_FREQ_CR2    = 0U,
	 I2C_ITERREN = (1U<<8U),
	 I2C_ITEVTEN = (1U<<9U),
	 I2C_ITBUFEN = (1U<<10U),
	 I2C_DMAEN   = (1U<<11U),
	 I2C_LAST    = (1U<<12U),

}I2C_CR2_t;


typedef enum{
	 I2C_FREQ_OAR1	= (1U<<0U),
	 I2C_ADD0 	    = 1U,
	 I2C_ADD1 	 	= 8U,
	 I2C_OAR1_BIT14 = (1U<<14U),
	 I2C_ADDMODE 	= (1U<<15U),

}I2C_OAR1_t;

typedef enum{
	 I2C_FREQ_OAR2    = (1U<<0U),
	 I2C_ADD2 	 = 1U,

}I2C_OAR2_t;

//typedef enum{
//	 I2C_DR    = 0U,
//}I2C_DR_t;

typedef enum{
	 I2C_SB		  = (1U<<0U),
	 I2C_ADDR     = (1U<<1U),
	 I2C_BTF  	  = (1U<<2U),
	 I2C_ADD10    = (1U<<3U),
	 I2C_STOPF    = (1U<<4U),
	 I2C_RxNE     = (1U<<6U),
	 I2C_TxE 	  = (1U<<7U),
	 I2C_BERR     = (1U<<8U),
	 I2C_ARLO     = (1U<<9U),
	 I2C_AF       = (1U<<10U),
	 I2C_OVR      = (1U<<11U),
	 I2C_PECERR   = (1U<<12U),
	 I2C_TIMEOUT  = (1U<<14U),
	 I2C_SMBALERT = (1U<<15U),

}I2C_SR1_t;



typedef enum{
	 I2C_MSL	    = (1U<<0U),
	 I2C_BUSY       = (1U<<1U),
	 I2C_TRA  	    = (1U<<2U),
	 I2C_GENCALL    = (1U<<4U),
	 I2C_SMBDEFAULT = (1U<<5U),
	 I2C_SMBHOST    = (1U<<6U),
	 I2C_DUALF 	    = (1U<<7U),
	 I2C_PEC_SR2    = 8U,

}I2C_SR2_t;


typedef enum{
	 I2C_CCR_enum   = 0U,
	 I2C_DUTY 		= (1U<<14U),
	 I2C_F_S  		= (1U<<15U),

}I2C_CCR_t;


typedef enum{
	 I2C_TRISE_enum  = 0U,

}I2C_TRISE_t;

typedef enum{
	 I2C_DNF   = 0U,
	 I2C_ANOFF = (1U<<4U),

}I2C_FLTR_t;



typedef enum{
	set_as_slave  = 0U,
	set_as_master = 1U,

}i2c_mode_t;


typedef enum{
	StandarMode_100Kbps  = 100000U,
	FastMode_400Kbps 	 = 400000U,

}i2c_baudrate_t;



typedef enum{
	duty_2_1 	  = 0U,
	duty_16_9 = 1U,
	duny_none = 2U,

}i2c_duty_t;


//typedef enum{
//	Start_status_flag = I2C_SB,
//	Addres_sent_status_flag = I2C_ADDR,
//	Byte_transfer_status_flag = I2C_BTF,
//	Bit_10_header_sent_status_flag = I2C_ADD10,
//	Stop_status_flag = I2C_STOPF,
//	TxE_status_flag = I2C_TxE,
//	Bus_error_status_flag = I2C_BERR,
//	Arbitration_lost_status_flag = I2C_ARLO,
//	ACK_fail_status_flag = I2C_AF,
//	OVR_status_flag = I2C_OVR,
//
//}i2c_status_flags_t;


typedef struct{
	i2c_baudrate_t baudrate;
	i2c_duty_t duty; //only for FM

}i2c_config_parameters_t;

Status_code_t I2C1_Init_Master(i2c_config_parameters_t config);
Status_code_t I2C3_Init_Master(i2c_config_parameters_t config);

Status_code_t I2C1_Init_Slave(uint8_t addrs);
Status_code_t I2C3_Init_Slave(uint8_t addrs);


Status_code_t I2C_Write(I2C_alternative_t i2c_alt,uint8_t addr, uint8_t* data_sent, uint32_t data_size);
Status_code_t I2C_Read(I2C_alternative_t i2c_alt, uint8_t addr, uint8_t* data_received, uint32_t data_size_expected);
Status_code_t I2C1_Deinit(void);
Status_code_t I2C3_Deinit(void);



void I2C_Clock(I2C_alternative_t I2C, Enabled_Disabled_t state);
Status_code_t I2C_config(i2c_config_parameters_t *config);
void I2C_slave_config(I2CMapAddr_t I2C_addr,uint8_t slave_addrs);
void I2C_Peripherial_Mode(I2CMapAddr_t I2C_addr,Enabled_Disabled_t state);
void I2C_Reset_Protocol(I2CMapAddr_t I2C_addr);
Status_code_t I2C_Set_Clock_frecuency(I2CMapAddr_t I2C_addr,uint32_t Clock_frecuency_hz);
void I2C_Set_FM_Duty(I2CMapAddr_t I2C_addr, i2c_duty_t duty);
Status_code_t I2C_Speed_Mode(I2CMapAddr_t I2C_addr, i2c_config_parameters_t* config, uint32_t peripherial_clock);
Status_code_t I2C_Set_CCR(I2CMapAddr_t I2C_addr, i2c_config_parameters_t *config, uint32_t peripherial_clock);
void I2C_Set_Trise(I2CMapAddr_t I2C_addr, i2c_baudrate_t baudrate, uint32_t peripherial_clock);

bool I2C_status_flag(I2CMapAddr_t I2C_addr, I2C_SR1_t flag);
bool I2C_Busy_State(I2CMapAddr_t I2C_addr);
void I2C_START_bit(I2CMapAddr_t I2C_addr, Enabled_Disabled_t state);
void I2C_STOP_bit(I2CMapAddr_t I2C_addr, Enabled_Disabled_t state);
void I2C_ACK_bit(I2CMapAddr_t I2C_addr, Enabled_Disabled_t state);
void I2C_Reset_ACK_bit(I2CMapAddr_t I2C_addr);
void I2C_Reset_OVR_bit(I2CMapAddr_t I2C_addr);
void I2C_Reset_BERR_bit(I2CMapAddr_t I2C_addr);

#endif /* DRIVERS_I2C_H_ */
