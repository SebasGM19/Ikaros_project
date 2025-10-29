/*
 * CRC_gen.h
 *
 *  Created on: Sep 6, 2025
 *      Author: sebas
 */

#ifndef DRIVERS_CRC_GEN_H_
#define DRIVERS_CRC_GEN_H_

#include <stdint.h>
#include <stdio.h>
#include "stdbool.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>


uint16_t Modbus_CRC_reflected(uint8_t *data, uint8_t data_length);
uint16_t ModbusRTU_CRC_NoReflected(uint8_t *data, uint8_t data_length);
uint8_t calculate_crc8(const uint8_t *data, size_t data_length, uint8_t poly, uint8_t initial_value, uint8_t xor_out);

#endif /* DRIVERS_CRC_GEN_H_ */
