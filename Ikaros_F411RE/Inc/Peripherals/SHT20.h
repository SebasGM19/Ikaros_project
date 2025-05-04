/*
 * SHT20.h
 *
 *  Created on: 13 abr. 2025
 *      Author: Sebas
 */

#ifndef SHT20_H_
#define SHT20_H_

#include "../Src/system_settings.h"
#include "../Drivers/i2c.h"

#define SHT20_SLAVE_ADDR (0x40)




Status_code_t SHT20ReadTemperature(I2C_alternative_t I2C, int16_t* temp);
Status_code_t SHT20ReadHumedad(I2C_alternative_t I2C, uint16_t* hum);

#endif /* SHT20_H_ */
