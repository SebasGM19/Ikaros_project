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
	ADC_SR =		0x00,
	ADC_CR1 =		0x04,
	ADC_CR2 =		0x08,
	ADC_SMPR1 =		0x0C,
	ADC_JOFRx =		0x14,//0x14 - 0x20
	ADC_HTR =		0x24,
	ADC_LTR =		0x28,
	ADC_SQR1 =		0x2C,
	ADC_SQR2 =		0x30,
	ADC_SQR3 =		0x34,
	ADC_JSQR =		0x38,
	ADC_JDRx =		0x3C, //0x3C-0x48
	ADC_DR =		0x4C,

}ADC_register_offset_t;

typedef enum{
	 ADC_AWD= 		(1u<<0),
	 ADC_EOC=		(1u<<1),
	 ADC_JEOC=		(1u<<2),
	 ADC_JSTRT=		(1u<<3),
	 ADC_STRT=		(1u<<4),
	 ADC_OVR=		(1u<<5),

}ADC_SR_t;

typedef enum{
	 ADC_AWDCH= 	(0),
	 ADC_EOCIE=		(1u<<5),
	 ADC_AWDIE=		(1u<<6),
	 ADC_JEOCIE=	(1u<<7),
	 ADC_SCAN=		(1u<<8),
	 ADC_AWDSGL=	(1u<<9),
	 ADC_JAUTO = 	(1u<<10),
	 ADC_DISCEN = 	(1u<<11),
	 ADC_JDISCEN = 	(1u<<12),
	 ADC_DISCNUM = 	(13),
	 ADC_JAWDEN = 	(1u<<22),
	 ADC_AWDEN = 	(1u<<23),
	 ADC_RES = 		(24),
	 ADC_OVRIE = 	(1u<<26)

}ADC_CR1_t;

typedef enum{
	 ADC_ADON= 		(1u<<0),
	 ADC_CONT =		(1u<<1),
	 ADC_DMA =		(1u<<8),
	 ADC_DDS =		(1u<<9),
	 ADC_EOCS =		(1u<<10),
	 ADC_ALIGN =	(1u<<11),
	 ADC_JEXTSEL = 	(16),
	 ADC_JEXTEN = 	(20),
	 ADC_JSWSTART =	(1u<<22),
	 ADC_EXTSEL = 	(24),
	 ADC_EXTEN = 	(28),
	 ADC_SWSTART = 	(1u<<30)

}ADC_CR2_t;


typedef enum{
	 ADC_SMP10= (0),
	 ADC_SMP11=	(3),
	 ADC_SMP12=	(6),
	 ADC_SMP13=	(9),
	 ADC_SMP14=	(12),
	 ADC_SMP15=	(15),
	 ADC_SMP16=	(18),
	 ADC_SMP17=	(21),
	 ADC_SMP18=	(24)

}ADC_SMPR1_t;

typedef enum{
	 ADC_SMP0= 	(0),
	 ADC_SMP1=	(3),
	 ADC_SMP2=	(6),
	 ADC_SMP3=	(9),
	 ADC_SMP4=	(12),
	 ADC_SMP5=	(15),
	 ADC_SMP6=	(18),
	 ADC_SMP7=	(21),
	 ADC_SMP8=	(24),
	 ADC_SMP9=	(27)

}ADC_SMPR2_t;


typedef enum{
	 ADC_JOFFSET
}ADC_JOFRx_t;

typedef enum{
	 ADC_HT_0
}ADC_HTR_t;

typedef enum{
	 ADC_LT
}ADC_LTR_t;

typedef enum{
	 ADC_JDATA_0
}ADC_JDRx_t;

typedef enum{
	 ADC_DATA_0
}ADC_DR_t;


typedef enum{
	 ADC_SQ13 = (0),
	 ADC_SQ14 = (5),
	 ADC_SQ15 = (10),
	 ADC_SQ16 = (15),
	 ADC_L = 	(20)

}ADC_SQR1_t;

typedef enum{
	 ADC_SQ7 = (0),
	 ADC_SQ8 = (5),
	 ADC_SQ9 = (10),
	 ADC_SQ10 =(15),
	 ADC_SQ11 =(20),
	 ADC_SQ12 =(25)

}ADC_SQR2_t;

typedef enum{
	 ADC_SQ1 = (0),
	 ADC_SQ2 = (5),
	 ADC_SQ3 = (10),
	 ADC_SQ4 = (15),
	 ADC_SQ5 = (20),
	 ADC_SQ6 = (25)

}ADC_SQR3_t;

typedef enum{
	 ADC_SQR_SINGLE_POSITION
}ADC_SQR_SET_SINGLE_POSITION_T;

typedef enum{
	 Stop,
	 Start
}ADC_conversion_start_t;

typedef enum{
	 None_channel,
	 Single_channel
}ADC_single_channel_lenght_t;


typedef enum{
	 ADC_JSQ1= 	(0),
	 ADC_JSQ2= 	(5),
	 ADC_JSQ3= 	(10),
	 ADC_JSQ4= 	(15),
	 ADC_JL= 	(20)

}ADC_JSQR_t;

typedef enum{
	 ADC_ADCPRE=	(16),
	 ADC_VBATE=		(1u<<22),
	 ADC_TSVREFE=	(1u<<23)
}ADC_CCR_t;

typedef enum{
	RES_12_bits,
	RES_10_bits,
	RES_8_bits,
	RES_6_bits

}ADC_resolution_t;

typedef enum{
	Channel_0,
	Channel_1,
	Channel_2,
	Channel_3,
	Channel_4,
	Channel_5,
	Channel_6,
	Channel_7,
	Channel_8,
	Channel_9,
	Channel_10,
	Channel_11,
	Channel_12,
	Channel_13,
	Channel_14,
	Channel_15,

}ADC_channel_t;

typedef enum{
	ADC1_ENABLE = (1u<<8)

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


#endif /* ADC_H_ */
