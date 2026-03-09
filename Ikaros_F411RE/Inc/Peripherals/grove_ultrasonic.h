/*
 * grove_ultrasonic.h
 *
 *  Created on: Dec 27, 2025
 *      Author: sebas
 */

#ifndef GROVE_ULTRASONIC_H_
#define GROVE_ULTRASONIC_H_

#include "system_settings.h"
#include "gpios.h"
#include "timers.h"

Status_code_t Grove_Init(Set_Port_t PORT, Pin_number_t GPIO);
Status_code_t Grove_Read_Distance(uint32_t *distance_mm);

void Grove_Deinit(void);

#endif /*GROVE_ULTRASONIC_H_ */
