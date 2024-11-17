/*
 * watchdog.h
 *
 *  Created on: Oct 17, 2024
 *      Author: sebas
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "system_settings.h"

#define WWDG_HANDLER WWDG_IRQHandler

#define IWDG_MAX_RL_TIME		(0xFFFFU)
#define IWDG_MIN_RL_RESET_DOG	(0x0001U)

#define IWDG_START_COUNT 		(0xCCCCU)
#define IWDG_RELOAD_COUNT		(0xAAAAU)
#define IWDG_ENABLE_ACCESS		(0x5555U)


#define WWDG_CLOCK_BIT_REG		(1U<<11U)
#define WWDG_MAX_TIME_VALUE		(130U)
#define WWDG_MIN_TIME_VALUE		(2U)
#define WWDG_INIT_START_TIME	(0x40U)

//#define WWDG_MAX_PRESCALER		(0x08U)
//#define WWDG_MIN_PRESCALER		(0x01U)

#define WWDG_MAX_CK_DIV			(0x03U)
#define WWDG_MIN_CK_DIV			(0x00U)

#define WWDG_W_VALUE_TO_RESET	(0x01U)
#define WWDG_T_VALUE_TO_RESET	(0x02U)






#define WWDG_MS_TIME(CLOCK_BASE,div,time)	 (uint32_t)((((float)time/1000) / ((1/ ((float)CLOCK_BASE)) * (1<<div) * 4096))-1)

typedef enum{
	reset_food,
	reload_food,

}watchdog_food_t;


typedef enum{

	reload_512ms,
	reload_1024ms,
	reload_2048ms,
	reload_4096ms,
	reload_8192ms,
	reload_16384ms,
	reload_32768ms,

}IWDG_reload_time_t;

typedef enum{
	IWDG_KR = 0x00U,
	IWDG_PR = 0x04U,
	IWDG_RLR = 0x08U,
	IWDG_SR = 0x0CU
}IWDG_register_offset_t;

typedef enum{
	IWDG_KEY = 0U
}IWDG_KR_t;

typedef enum{
	IWDG_PR_DIV = 0U
}IWDG_PR_t;

typedef enum{
	IWDG_RL = 0U
}IWDG_RLR_t;

typedef enum{
	IWDG_PVU = (1U<<0U),
	IWDG_RVU = (1U<<1U),
}IWDG_SR_t;

/*__________________Independent_Watchdog__________________*/

void Init_Ind_Watchdog(IWDG_reload_time_t reload_time);
void Ind_Watchdog_control(watchdog_food_t food);
void Ind_resetTheDog(void);
void Ind_reloadTheDog(void);

/*__________________WINDOW_Watchdog__________________*/

typedef enum{
	WWDG_CR = 0x00U,
	WWDG_CFR = 0x04U,
	WWDG_SR = 0x08U,
}WWDG_register_offset_t;

typedef enum{
	WWDG_T 	= 	 0U,
	WWDG_WDGA = (1U<<7U),
}WWDG_CR_t;

typedef enum{
	WWDG_W 	= 	 0U,
	WWDG_WDGTB = 7U,
	WWDG_EWI =   (1U<<9U),
}WWDG_CFR_t;

typedef enum{
	WWDG_EWIF = (1U<<0U),
}WWDG_SR_t;




typedef struct{
	uint8_t T_max_time;
	uint8_t W_time;

}WWDG_config_t;


void WWDG_HANDLER(void);
Status_code_t Init_Win_Watchdog(WWDG_config_t config_window);
void Win_Watchdog_control(watchdog_food_t food);
void Win_resetTheDog(void);
void Win_reloadTheDog(void);

Status_code_t WWDG_Clock(Enabled_Disabled_t state);


#endif /* WATCHDOG_H_ */
