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
	Not_ready = 0U,
	Ready 	  = 1U,
}data_status_t;

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


typedef enum{
	Gyr_2000s = 0U,
	Gyr_1000s = 1U,
	Gyr_500s  = 2U,
	Gyr_250s  = 3U,
	Gyr_125s  = 4U,

}Gyr_Resolution_t;


typedef enum{
	Acc_2g  = 3U,
	Acc_4g  = 5U,
	Acc_8g  = 8U,
	Acc_16g = 12U,

}Acc_Resolution_t;


//taking the max sensibility in each case
typedef enum{
	Acc_2g_sen  = (17039U),
	Acc_4g_sen  = (8520U),
	Acc_8g_sen  = (4260U),
	Acc_16g_sen = (2130U),

}Acc_Sensibility_t;

typedef enum{
	acc_offset_bit_EN = (1U<<6U),
	gyr_offset_bit_EN = (1U<<7U),

}offset_bit_t;


typedef enum{
	compensation_disabled = 0U,
	compensation_plus_1   = 1U,
	compensation_minus_1  = 2U,
	compensation_cero 	  =	3U,

}FOC_acc_values_t;

typedef enum{
	disabled_FOC_gyr = 0U,
	enabled_FOC_gyr = 1U,

}FOC_gyr_values_t;

typedef struct{
	int16_t magnetometer_X;
	int16_t magnetometer_Y;
	int16_t magnetometer_Z;

	float gyroscope_X;
	float gyroscope_Y;
	float gyroscope_Z;

	float accelerometer_X;
	float accelerometer_Y;
	float accelerometer_Z;

	uint16_t hall_resistance;

}BMI160_TotalData_t;



typedef struct{
	Acc_PM_t Acc_power_mode;
	Gyr_PM_t Gyr_power_mode;
	Mag_PM_t Mag_power_mode;
	Acc_Resolution_t Acc_res;
	Gyr_Resolution_t Gyr_res;

}BMI160_init_parameters;

Status_code_t BMI160_Init(I2C_alternative_t I2C, BMI160_init_parameters *param);

Status_code_t BMI160_Deinit(I2C_alternative_t I2C);

//returns the CHIP ID, typically 0xD1
Status_code_t BMI160_Chip_ID(I2C_alternative_t I2C, uint8_t *chip_id);

//returns the power mode status of the sensors
Status_code_t BMI160_PMU_Status(I2C_alternative_t I2C,
				PMU_status_t *Accelerometer, PMU_status_t *Gyroscope, PMU_status_t *Magnetometer);

//this function gets all the data of magnetometer, gyroscope and accelerometer in one struct
Status_code_t BMI160_Get_TotalData(I2C_alternative_t I2C, BMI160_TotalData_t *parameter);

//this function gets the data of magnetometer in the struct (ONLY Adding the BMM150 sensor)
Status_code_t BMI160_Get_Magnetometer_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter);

//this function gets the data of hall resistance in the struct (ONLY Adding the BMM150 sensor)
Status_code_t BMI160_Get_HallR_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter);

//this function gets the data of gyroscope in the struct
Status_code_t BMI160_Get_Gyroscope_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter);

//this function gets the data of accelerometer in the struct
Status_code_t BMI160_Get_Accelerometer_Data(I2C_alternative_t I2C, BMI160_TotalData_t *parameter);

//this function return the data status if its ready to read
Status_code_t BMI160_Data_Status(I2C_alternative_t I2C, data_status_t* accelerometer,
								data_status_t* gyroscope, data_status_t* magnetometer, data_status_t* FOC);

//this function get the temperature
Status_code_t BMI160_Temperature(I2C_alternative_t I2C, int16_t *temperature);

//this function Gets the currently resolution of the accelerometer
Status_code_t BMI160_Get_Accelerometer_Resolution(I2C_alternative_t I2C,Acc_Resolution_t *res);

//this function Sets the currently resolution of the accelerometer
Status_code_t BMI160_Set_Accelerometer_Resolution(I2C_alternative_t I2C, Acc_Resolution_t res);

//this function Gets the currently resolution of the accelerometer
Status_code_t BMI160_Get_Gyroscope_Resolution(I2C_alternative_t I2C, Gyr_Resolution_t *res);

//this function Sets the currently resolution of the accelerometer
Status_code_t BMI160_Set_Gyroscope_Resolution(I2C_alternative_t I2C, Gyr_Resolution_t res);

//this function enables de fast offset compensation
Status_code_t BMI160_Set_Fast_Offset_Compensation(I2C_alternative_t I2C, FOC_gyr_values_t gyr_state,
												FOC_acc_values_t acc_X, FOC_acc_values_t acc_Y, FOC_acc_values_t acc_Z);

//this function set automatically the offset with out passign values
Status_code_t BMI160_AutoSet_Data_Offset(I2C_alternative_t I2C);

//this function return the arr of the offset
Status_code_t BMI160_Get_Data_Offset(I2C_alternative_t I2C, uint8_t* data);

//this function enabled the accelerometer offset
Status_code_t BMI160_Enabled_Accelerometer_Offset(I2C_alternative_t I2C);

//this function enabled the accelerometer offset
Status_code_t BMI160_Disabled_Accelerometer_Offset(I2C_alternative_t I2C);

//this function enabled the gyroscope offset
Status_code_t BMI160_Enabled_Gyroscope_Offset(I2C_alternative_t I2C);

//this function enabled the gyroscope offset
Status_code_t BMI160_Disabled_Gyroscope_Offset(I2C_alternative_t I2C);

//This function change the power mode of the accelerometer
Status_code_t BMI160_Set_Acceleromete_PM(I2C_alternative_t I2C, Acc_PM_t selected_ACC_PM);

//This function change the power mode of the Gyroscope
Status_code_t BMI160_Set_Gyroscope_PM(I2C_alternative_t I2C, Gyr_PM_t selected_GYR_PM);

//This function change the power mode of the magnetometer
Status_code_t BMI160_Set_Magnetometer_PM(I2C_alternative_t I2C, Mag_PM_t selected_MAG_PM);

//This functions send soft reset
Status_code_t BMI160_Soft_Reset(I2C_alternative_t I2C);

#endif /* BMI160_H_ */
