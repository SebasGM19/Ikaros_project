/*
 * SHT20.h
 *
 *  Created on: 13 abr. 2025
 *      Author: Sebas
 */

#ifndef SHT20_H_
#define SHT20_H_

#include "system_settings.h"
#include "i2c.h"

#define SLAVE_ADDRS (0x40)
#define TEMP_REG	(0xF3)
#define HUM_REG		(0xF5)



Status_code_t SHT20_Read_Temperature(I2C_alternative_t I2C, int16_t* temp);
Status_code_t SHT20_Read_Humedad(I2C_alternative_t I2C, uint16_t* hum);

#endif /* SHT20_H_ */
