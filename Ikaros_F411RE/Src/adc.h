/*
 * adc.h
 *
 *  Created on: Jul 22, 2024
 *      Author: sebas
 */

#ifndef ADC_H_
#define ADC_H_
#include "system_settings.h"


#define MAX_ADC_TIMEOUT	(5000)

typedef enum{
	ADC_SR =		0x00U,
	ADC_CR1 =		0x04U,
	ADC_CR2 =		0x08U,
	ADC_SMPR1 =		0x0CU,
	ADC_SMPR2 =		0x10U,
	ADC_JOFRx =		0x14U,//0x14 - 0x20
	ADC_HTR =		0x24U,
	ADC_LTR =		0x28U,
	ADC_SQR1 =		0x2CU,
	ADC_SQR2 =		0x30U,
	ADC_SQR3 =		0x34U,
	ADC_JSQR =		0x38U,
	ADC_JDRx =		0x3CU, //0x3C-0x48
	ADC_DR =		0x4CU,
	ADC_CCR = 		0x304U

}ADC_register_offset_t;

typedef enum{
	 ADC_AWD= 		(1U<<0U),
	 ADC_EOC=		(1U<<1U),
	 ADC_JEOC=		(1U<<2U),
	 ADC_JSTRT=		(1U<<3U),
	 ADC_STRT=		(1U<<4U),
	 ADC_OVR=		(1U<<5U),

}ADC_SR_t;

typedef enum{
	 ADC_AWDCH= 	(0U),
	 ADC_EOCIE=		(1U<<5U),
	 ADC_AWDIE=		(1U<<6U),
	 ADC_JEOCIE=	(1U<<7U),
	 ADC_SCAN=		(1U<<8U),
	 ADC_AWDSGL=	(1U<<9U),
	 ADC_JAUTO = 	(1U<<10U),
	 ADC_DISCEN = 	(1U<<11U),
	 ADC_JDISCEN = 	(1U<<12U),
	 ADC_DISCNUM = 	(13U),
	 ADC_JAWDEN = 	(1U<<22U),
	 ADC_AWDEN = 	(1U<<23U),
	 ADC_RES = 		(24U),
	 ADC_OVRIE = 	(1U<<26U)

}ADC_CR1_t;

typedef enum{
	 ADC_ADON= 		(1U<<0U),
	 ADC_CONT =		(1U<<1U),
	 ADC_DMA =		(1U<<8U),
	 ADC_DDS =		(1U<<9U),
	 ADC_EOCS =		(1U<<10U),
	 ADC_ALIGN =	(1U<<11U),
	 ADC_JEXTSEL = 	(16U),
	 ADC_JEXTEN = 	(20U),
	 ADC_JSWSTART =	(1U<<22U),
	 ADC_EXTSEL = 	(24U),
	 ADC_EXTEN = 	(28U),
	 ADC_SWSTART = 	(1U<<30U)

}ADC_CR2_t;


typedef enum{
	 ADC_SMP10= (0U),
	 ADC_SMP11=	(3U),
	 ADC_SMP12=	(6U),
	 ADC_SMP13=	(9U),
	 ADC_SMP14=	(12U),
	 ADC_SMP15=	(15U),
	 ADC_SMP16=	(18U),
	 ADC_SMP17=	(21U),
	 ADC_SMP18=	(24U)

}ADC_SMPR1_t;

typedef enum{
	 ADC_SMP0= 	(0U),
	 ADC_SMP1=	(3U),
	 ADC_SMP2=	(6U),
	 ADC_SMP3=	(9U),
	 ADC_SMP4=	(12U),
	 ADC_SMP5=	(15U),
	 ADC_SMP6=	(18U),
	 ADC_SMP7=	(21U),
	 ADC_SMP8=	(24U),
	 ADC_SMP9=	(27U)

}ADC_SMPR2_t;


typedef enum{
	 ADC_JOFFSET = (0U)
}ADC_JOFRx_t;

typedef enum{
	 ADC_HT_0 = (0U)
}ADC_HTR_t;

typedef enum{
	 ADC_LT = (0U)
}ADC_LTR_t;

typedef enum{
	 ADC_JDATA_0 = (0U)
}ADC_JDRx_t;

typedef enum{
	 ADC_DATA_0 = (0U)
}ADC_DR_t;


typedef enum{
	 ADC_SQ13 = (0U),
	 ADC_SQ14 = (5U),
	 ADC_SQ15 = (10U),
	 ADC_SQ16 = (15U),
	 ADC_L = 	(20U)

}ADC_SQR1_t;

typedef enum{
	 ADC_SQ7 = (0U),
	 ADC_SQ8 = (5U),
	 ADC_SQ9 = (10U),
	 ADC_SQ10 =(15U),
	 ADC_SQ11 =(20U),
	 ADC_SQ12 =(25U)

}ADC_SQR2_t;

typedef enum{
	 ADC_SQ1 = (0U),
	 ADC_SQ2 = (5U),
	 ADC_SQ3 = (10U),
	 ADC_SQ4 = (15U),
	 ADC_SQ5 = (20U),
	 ADC_SQ6 = (25U)

}ADC_SQR3_t;

typedef enum{
	 ADC_SQR_SINGLE_POSITION = (0U)
}ADC_SQR_SET_SINGLE_POSITION_T;

typedef enum{
	 Stop = (0U),
	 Start = (1U)
}ADC_conversion_start_t;

typedef enum{
	 None_channel = (0U),
	 Single_channel = (1U),
	 Two_channels = (2U),
}ADC_single_channel_lenght_t;


typedef enum{
	 ADC_JSQ1= 	(0U),
	 ADC_JSQ2= 	(5U),
	 ADC_JSQ3= 	(10U),
	 ADC_JSQ4= 	(15U),
	 ADC_JL= 	(20U)

}ADC_JSQR_t;

typedef enum{
	 ADC_ADCPRE=	(16U),
	 ADC_VBATE=		(1U<<22U),
	 ADC_TSVREFE=	(1U<<23U)
}ADC_CCR_t;

typedef enum{
	 PCLK_DIV_BY_2 =	(0U),
	 PCLK_DIV_BY_4 =	(1U),
	 PCLK_DIV_BY_6 =	(2U),
	 PCLK_DIV_BY_8 =	(3U),
}ADC_CCR_Prescaler_t;

typedef enum{
	RES_12_bits = 	(0U),
	RES_10_bits = 	(1U),
	RES_8_bits = 	(2U),
	RES_6_bits = 	(3U)

}ADC_resolution_t;

typedef enum{
	Sampling_3_cycles = 	(0U),
	Sampling_15_cycles = 	(1U),
	Sampling_28_cycles = 	(2U),
	Sampling_56_cycles = 	(3U),
	Sampling_84_cycles = 	(4U),
	Sampling_112_cycles = 	(5U),
	Sampling_144_cycles = 	(6U),
	Sampling_480_cycles = 	(7U),

}ADC_cycles_t;

typedef enum{
	Channel_0 = 	(0U),
	Channel_1 = 	(1U),
	Channel_2 = 	(2U),
	Channel_3 = 	(3U),
	Channel_4 = 	(4U),
	Channel_5 = 	(5U),
	Channel_6 = 	(6U),
	Channel_7 = 	(7U),
	Channel_8 = 	(8U),
	Channel_9 = 	(9U),
	Channel_10 = 	(10U),
	Channel_11 = 	(11U),
	Channel_12 = 	(12U),
	Channel_13 = 	(13U),
	Channel_14 = 	(14U),
	Channel_15 = 	(15U),
	Channel_17_VREFINT = (17U),
	Channel_18_temp_and_Vbat_sensor = (18U),

}ADC_channel_t;

typedef enum{
	ADC1_ENABLE = (1U<<8U)

}ADC_RCC_EN_t;



/*
 * single-> Performs a single conversion of a single channel and stop after conversion complete
 * continuous -> the ADC start a new conversion as soon as finishes one for one channel
 * scan-> scan all the channels selected, making a single conversion each
 * injected use when conversion in triggered by external event or software, has priority than others
 */
typedef enum{

	single_conversion,
	continuous_conversion,
	scan,
	injected,
}ADC_operation_mode_t;

Status_code_t ADC_Init(ADC_resolution_t resolution);
Status_code_t ADC_Configure_Channel(ADC_channel_t Channel);
uint32_t ADC_Read(ADC_channel_t Channel);
Status_code_t ADC_Deinit(void);
Status_code_t ADC_Init_Temperature_Sensor(ADC_resolution_t resolution);
Status_code_t ADC_Read_Temperature(uint32_t *temperature);


void ADC_conversion_state(Enabled_Disabled_t state);
uint8_t ADC_countChannelActivated(void);
void ADC_Channel_SQR_Single_position(ADC_channel_t Channel);
void ADC_SetSingleChannelLenght(ADC_single_channel_lenght_t lenght);
void ADC_Clock(Enabled_Disabled_t state);
Status_code_t ADC_start_convertion_regular_channel(void);
void ADC_cleanConversionFlag(void);
Status_code_t ADC_wait_conversion_flag(void);

void ADC_start_conversion(ADC_conversion_start_t state);
void ADC_Set_Resolution(ADC_resolution_t resolution);
const uint32_t ADC_Get_Resolution(void);
void ADC_Set_Sample_Time_Cycles(ADC_cycles_t cycles, ADC_channel_t Channel);


void ADC_Set_temperature_sensor_state(Enabled_Disabled_t const state);
void ADC_Set_Vbat_sensor_state(Enabled_Disabled_t const state);

#endif /* ADC_H_ */
