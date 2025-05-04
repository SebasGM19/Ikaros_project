/*
 * BMI160.h
 *
 *  Created on: May 3, 2025
 *      Author: sebas
 */

#ifndef BMI160_H_
#define BMI160_H_

#include "../Src/system_settings.h"
#include "../Drivers/i2c.h"

#define BMI160_SLAVE_ADDR (0x68) //addr when the pin SDO/SAO is connected to GND
//#define BMI160_SLAVE_ADDR (0x69) //addr when the pin SDO/SAO is connected to VDD

#define GYROSCOPE_PMU_OFFSET	(10)

typedef enum{
	Suspend = 0U,
	Normal = 1U,
	Low_Power = 2U,
	Fast_Start_Up = 3U,

}PMU_status_t;


typedef enum{
	Acc_Suspend = 0x10U,
	Acc_Normal = 0x11U,
	Acc_Low_Power = 0x12U,

}Acc_PM_t;

typedef enum{
	Gyr_Suspend = 0x14U,
	Gyr_Normal = 0x15U,
	Gyr_Fast_Start_Up = 0x17U,

}Gyr_PM_t;

typedef enum{
	Mag_Suspend = 0x18U,
	Mag_Normal = 0x19U,
	Mag_Low_Power = 0x1AU,

}Mag_PM_t;

//returns the CHIP ID, typically 0xD1
Status_code_t BMI160_Chip_ID(I2C_alternative_t I2C, uint8_t *chip_id);

//returns the power mode status of the sensors
Status_code_t BMI160_PMU_Status(I2C_alternative_t I2C,
				PMU_status_t *Accelerometer, PMU_status_t *Gyroscope, PMU_status_t *Magnetometer);

//this function get the temperature
Status_code_t BMI160_temperature(I2C_alternative_t I2C, int16_t *temperature);

//This function change the power mode of the accelerometer
Status_code_t BMI160_Set_Acceleromete_PM(I2C_alternative_t I2C, Acc_PM_t selected_ACC_PM);

//This function change the power mode of the Gyroscope
Status_code_t BMI160_Set_Gyroscope_PM(I2C_alternative_t I2C, Gyr_PM_t selected_GYR_PM);

//This function change the power mode of the magnetometer
Status_code_t BMI160_Set_Magnetometer_PM(I2C_alternative_t I2C, Mag_PM_t selected_MAG_PM);

#endif /* BMI160_H_ */
