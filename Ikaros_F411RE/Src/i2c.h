/*
 * i2c.h
 *
 *  Created on: Apr 12, 2025
 *      Author: sebas
 */

#ifndef DRIVERS_I2C_H_
#define DRIVERS_I2C_H_

#include "system_settings.h"

typedef enum{
	I2C_1 = 21U, 		//SCL:(B8)  SDA:(B9) //21 means RCC APB1 bit 21
	I2C_3 = 23U, 		//SCL:(A8)  SDA:(C9) //21 means RCC APB1 bit 23

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
	 I2C_FREQ_OAR1    = (1U<<0U),
	 I2C_ADD0 	 = 1U,
	 I2C_ADD1 	 = 8U,
	 I2C_ADDMODE = (1U<<15U),

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
	 i2c_TRISE  = 0U,

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
	frecuency_100Khz  = 100000U,
	frecuency_400Khz = 400000U,
	frecuency_1Mhz = 1000000U,


}i2c_baudrate_t;



typedef struct{
	uint32_t baudrate;
	i2c_mode_t set_as;

}i2c_config_parameters_t;

Status_code_t I2C1_Init(i2c_config_parameters_t config);
Status_code_t I2C3_Init(i2c_config_parameters_t config);
Status_code_t I2C_Write(I2C_alternative_t i2c_alt,uint8_t addrs, uint8_t* data_sent, uint32_t data_size);
Status_code_t I2C_Read(I2C_alternative_t i2c_alt, uint8_t addrs, uint8_t* data_received, uint32_t data_size_expected);
Status_code_t I2C1_Deinit(void);
Status_code_t I2C3_Deinit(void);



Status_code_t I2C_Clock(Enabled_Disabled_t state, I2C_alternative_t I2C);

#endif /* DRIVERS_I2C_H_ */
