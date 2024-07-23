/*
 * adc.h
 *
 *  Created on: Jul 22, 2024
 *      Author: sebas
 */

#ifndef ADC_H_
#define ADC_H_
#include "system_settings.h"
#include "gpios.h"


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
	 ADC_AWDCH= 	(1u<<0),
	 ADC_EOCIE=		(1u<<5),
	 ADC_AWDIE=		(1u<<6),
	 ADC_JEOCIE=	(1u<<7),
	 ADC_SCAN=		(1u<<8),
	 ADC_AWDSGL=	(1u<<9),
	 ADC_JAUTO = 	(1u<<10),
	 ADC_DISCEN = 	(1u<<11),
	 ADC_JDISCEN = 	(1u<<12),
	 ADC_DISCNUM = 	(1u<<13),
	 ADC_JAWDEN = 	(1u<<22),
	 ADC_AWDEN = 	(1u<<23),
	 ADC_RES = 		(1u<<24),
	 ADC_OVRIE = 	(1u<<26)

}ADC_CR1_t;

typedef enum{
	 ADC_ADON= 		(1u<<0),
	 ADC_CONT =		(1u<<1),
	 ADC_DMA =		(1u<<8),
	 ADC_DDS =		(1u<<9),
	 ADC_EOCS =		(1u<<10),
	 ADC_ALIGN =	(1u<<11),
	 ADC_JEXTSEL = 	(1u<<16),
	 ADC_JEXTEN = 	(1u<<20),
	 ADC_JSWSTART =	(1u<<22),
	 ADC_EXTSEL = 	(1u<<24),
	 ADC_EXTEN = 	(1u<<28),
	 ADC_SWSTART = 	(1u<<30)

}ADC_CR2_t;


typedef enum{
	 ADC_SMP10= (1u<<0),
	 ADC_SMP11=	(1u<<3),
	 ADC_SMP12=	(1u<<6),
	 ADC_SMP13=	(1u<<9),
	 ADC_SMP14=	(1u<<12),
	 ADC_SMP15=	(1u<<15),
	 ADC_SMP16=	(1u<<18),
	 ADC_SMP17=	(1u<<21),
	 ADC_SMP18=	(1u<<24)

}ADC_SMPR1_t;

typedef enum{
	 ADC_SMP0= 	(1u<<0),
	 ADC_SMP1=	(1u<<3),
	 ADC_SMP2=	(1u<<6),
	 ADC_SMP3=	(1u<<9),
	 ADC_SMP4=	(1u<<12),
	 ADC_SMP5=	(1u<<15),
	 ADC_SMP6=	(1u<<18),
	 ADC_SMP7=	(1u<<21),
	 ADC_SMP8=	(1u<<24),
	 ADC_SMP9=	(1u<<27)

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
	 ADC_SQ13 = (1u<<0),
	 ADC_SQ14 = (1u<<5),
	 ADC_SQ15 = (1u<<10),
	 ADC_SQ16 = (1u<<15),
	 ADC_L = 	(1u<<20)

}ADC_SQR1_t;

typedef enum{
	 ADC_SQ7 = (1u<<0),
	 ADC_SQ8 = (1u<<5),
	 ADC_SQ9 = (1u<<10),
	 ADC_SQ10 = (1u<<15),
	 ADC_SQ11 = (1u<<20),
	 ADC_SQ12 = (1u<<25)

}ADC_SQR2_t;

typedef enum{
	 ADC_SQ1 = (1u<<0),
	 ADC_SQ2 = (1u<<5),
	 ADC_SQ3 = (1u<<10),
	 ADC_SQ4 = (1u<<15),
	 ADC_SQ5 = (1u<<20),
	 ADC_SQ6 = (1u<<25)

}ADC_SQR3_t;

typedef enum{
	 ADC_JSQ1= 	(1u<<0),
	 ADC_JSQ2= 	(1u<<5),
	 ADC_JSQ3= 	(1u<<10),
	 ADC_JSQ4= 	(1u<<15),
	 ADC_JL= 	(1u<<20)

}ADC_JSQR_t;

typedef enum{
	 ADC_ADCPRE=	(1u<<16),
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


void Channel_SQR_position(ADC_channel_t Channel);
void ADC_Clock(Enabled_Disabled_t state);
Status_code_t ADC_Init(ADC_channel_t Channel);
uint32_t ADC_Read(ADC_channel_t Channel);
void ADC_Deinit(void);

#endif /* ADC_H_ */
