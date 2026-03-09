/*
 * CRC_gen.c
 *
 *  Created on: Sep 6, 2025
 *      Author: sebas
 */
#include "CRC_gen.h"

uint16_t Modbus_CRC_reflected(uint8_t *data, uint8_t data_length)
{

    uint16_t crc = 0xFFFF;
    uint8_t num_byte = 0;
    uint8_t num_bit = 0;
    while(num_byte < data_length)
    {
        crc ^= data[num_byte];
        while(num_bit < 8){
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }

            else
            {
                crc >>= 1;
            }
            num_bit++;
        }
        num_bit = 0;
        num_byte++;
    }

    return crc;

//    data[num_byte++] = (uint8_t)crc;
//    data[num_byte] = (uint8_t)(crc>>8);

}


uint16_t ModbusRTU_CRC_NoReflected(uint8_t *data, uint8_t data_length)
{
    uint16_t crc = 0xFFFF;
    uint8_t num_byte = 0;
    uint8_t num_bit = 0;

    while(num_byte < data_length)
    {
        crc ^= (uint16_t)data[num_byte] << 8; // XOR con el byte de datos

        while(num_bit < 8)
        {
            if ((crc & 0x8000) != 0) // Comprobación del bit más significativo
            {
                crc <<= 1;
                crc ^= 0x8005;
            }
            else
            {
                crc <<= 1;
            }
            num_bit++;
        }
        num_bit = 0;
        num_byte++;
    }
    return crc;
}

/**
 * @brief Calcula el CRC-8 con parámetros configurables.
 * @param data Puntero a los datos de entrada.
 * @param data_length Longitud de los datos.
 * @param poly Polinomio generador (ej. 0x2F).
 * @param initial_value Valor inicial del registro CRC.
 * @param xor_out Valor XOR final.
 * @return El valor del CRC-8 calculado.
 */
uint8_t calculate_crc8(const uint8_t *data, size_t data_length, uint8_t poly, uint8_t initial_value, uint8_t xor_out)
{
    uint8_t crc = initial_value;

    for (size_t i = 0; i < data_length; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc <<= 1;
                crc ^= poly;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc ^ xor_out;
}
