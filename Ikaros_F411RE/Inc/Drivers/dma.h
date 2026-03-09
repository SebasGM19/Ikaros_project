/*
 * dma.h
 *
 *  Created on: Mar 8, 2026
 *      Author: sebas
 */

#ifndef DRIVERS_DMA_H_
#define DRIVERS_DMA_H_

#include "../Src/system_settings.h"
#define DMA_OFFSET_STREAM_SELECTOR	((uint8_t) 0x18U)
#define DMA_FLAGS_OFFSET			((uint8_t) 6U)

typedef enum{
	DMA_LISR   	= 0x00U,
	DMA_HISR   	= 0x04U,
	DMA_LIFCR 	= 0x08U,
	DMA_HIFCR  	= 0x0CU,
	DMA_SxCR    = 0x10U,
	DMA_SxNDTR  = 0x14U,
	DMA_SxPAR   = 0x18U,
	DMA_SxM0AR  = 0x1CU,
	DMA_SxM1AR 	= 0x20U,
	DMA_SxFCR  	= 0x24U,

}DMA_register_offset_t;


typedef enum{
	DMA1_Alt = 21U, 		//21 means RCC AHB1 bit 21
	DMA2_Alt = 22U, 		//22 means RCC AHB1 bit 22

}DMA_alt_t;

typedef enum{

	DMA_Peripheral_to_Memory 	= 0U,
	DMA_Memory_to_Peripheral	= 1U,
	DMA_Memory_to_Memory		= 2U,

}DMA_src_and_dest_option_t;


typedef enum{

	DMA_Channel_0 = 0U,
	DMA_Channel_1 = 1U,
	DMA_Channel_2 = 2U,
	DMA_Channel_3 = 3U,
	DMA_Channel_4 = 4U,
	DMA_Channel_5 = 5U,
	DMA_Channel_6 = 6U,
	DMA_Channel_7 = 7U,


}DMA_channels_t;

typedef enum{

	DMA_Streams_0 = 0U,
	DMA_Streams_1 = 1U,
	DMA_Streams_2 = 2U,
	DMA_Streams_3 = 3U,
	DMA_Streams_4 = 4U,
	DMA_Streams_5 = 5U,
	DMA_Streams_6 = 6U,
	DMA_Streams_7 = 7U,

}DMA_streams_t;


typedef enum{

	DMA_Priority_Low 		= 0U,
	DMA_Priority_Medium 	= 1U,
	DMA_Priority_High 		= 2U,
	DMA_Priority_Very_High 	= 3U,

}DMA_PL_t;//priority level


typedef enum{

	DMA_DSIZE_Byte 		= 0U,
	DMA_DSIZE_Half_Word	= 1U,
	DMA_DSIZE_Word		= 2U,

}DMA_data_size_t; //memory size used for MSIZE y PSIZE


typedef enum{
	DMA_FEIF0		= (1U<<0U),
	DMA_DMEIF0		= (1U<<2U),
	DMA_TEIF0		= (1U<<3U),
	DMA_HTIF0		= (1U<<4U),
	DMA_TCIF0		= (1U<<5U),

	DMA_FEIF1		= (1U<<6U),
	DMA_DMEIF1		= (1U<<8U),
	DMA_TEIF1		= (1U<<9U),
	DMA_HTIF1		= (1U<<10U),
	DMA_TCIF1		= (1U<<11U),

	DMA_FEIF2		= (1U<<16U),
	DMA_DMEIF2		= (1U<<18U),
	DMA_TEIF2		= (1U<<19U),
	DMA_HTIF2		= (1U<<20U),
	DMA_TCIF2		= (1U<<21U),

	DMA_FEIF3		= (1U<<22U),
	DMA_DMEIF3		= (1U<<24U),
	DMA_TEIF3		= (1U<<25U),
	DMA_HTIF3		= (1U<<26U),
	DMA_TCIF3		= (1U<<27U),
}DMA_LISR_t;

typedef enum{
	DMA_FEIF4		= (1U<<0U),
	DMA_DMEIF4		= (1U<<2U),
	DMA_TEIF4		= (1U<<3U),
	DMA_HTIF4		= (1U<<4U),
	DMA_TCIF4		= (1U<<5U),

	DMA_FEIF5		= (1U<<6U),
	DMA_DMEIF5		= (1U<<8U),
	DMA_TEIF5		= (1U<<9U),
	DMA_HTIF5		= (1U<<10U),
	DMA_TCIF5		= (1U<<11U),

	DMA_FEIF6		= (1U<<16U),
	DMA_DMEIF6		= (1U<<18U),
	DMA_TEIF6		= (1U<<19U),
	DMA_HTIF6		= (1U<<20U),
	DMA_TCIF6		= (1U<<21U),

	DMA_FEIF7		= (1U<<22U),
	DMA_DMEIF7		= (1U<<24U),
	DMA_TEIF7		= (1U<<25U),
	DMA_HTIF7		= (1U<<26U),
	DMA_TCIF7		= (1U<<27U),
}DMA_HISR_t;



typedef enum{
	DMA_CFEIF0		= (1U<<0U),
	DMA_CDMEIF0		= (1U<<2U),
	DMA_CTEIF0		= (1U<<3U),
	DMA_CHTIF0		= (1U<<4U),
	DMA_CTCIF0		= (1U<<5U),

	DMA_CFEIF1		= (1U<<6U),
	DMA_CDMEIF1		= (1U<<8U),
	DMA_CTEIF1		= (1U<<9U),
	DMA_CHTIF1		= (1U<<10U),
	DMA_CTCIF1		= (1U<<11U),

	DMA_CFEIF2		= (1U<<16U),
	DMA_CDMEIF2		= (1U<<18U),
	DMA_CTEIF2		= (1U<<19U),
	DMA_CHTIF2		= (1U<<20U),
	DMA_CTCIF2		= (1U<<21U),

	DMA_CFEIF3		= (1U<<22U),
	DMA_CDMEIF3		= (1U<<24U),
	DMA_CTEIF3		= (1U<<25U),
	DMA_CHTIF3		= (1U<<26U),
	DMA_CTCIF3		= (1U<<27U),

}DMA_LIFCR_t;



typedef enum{
	DMA_CFEIF4		= (1U<<0U),
	DMA_CDMEIF4		= (1U<<2U),
	DMA_CTEIF4		= (1U<<3U),
	DMA_CHTIF4		= (1U<<4U),
	DMA_CTCIF4		= (1U<<5U),

	DMA_CFEIF5		= (1U<<6U),
	DMA_CDMEIF5		= (1U<<8U),
	DMA_CTEIF5		= (1U<<9U),
	DMA_CHTIF5		= (1U<<10U),
	DMA_CTCIF5		= (1U<<11U),

	DMA_CFEIF6		= (1U<<16U),
	DMA_CDMEIF6		= (1U<<18U),
	DMA_CTEIF6		= (1U<<19U),
	DMA_CHTIF6		= (1U<<20U),
	DMA_CTCIF6		= (1U<<21U),

	DMA_CFEIF7		= (1U<<22U),
	DMA_CDMEIF7		= (1U<<24U),
	DMA_CTEIF7		= (1U<<25U),
	DMA_CHTIF7		= (1U<<26U),
	DMA_CTCIF7		= (1U<<27U),

}DMA_HIFCR_t;



typedef enum{
	DMA_FEIFx	= (0U),
	DMA_DMEIFx	= (2U),
	DMA_TEIFx	= (3U),
	DMA_HTIFx	= (4U),
	DMA_TCIFx	= (5U),

}DMA_flags_t;

typedef enum{
	DMA_EN		= (1U<<0U),
	DMA_DMEIE	= (1U<<1U),
	DMA_TEIE	= (1U<<2U),
	DMA_HTIE	= (1U<<3U),
	DMA_TCIE	= (1U<<4U),

	DMA_PFCTRL	= (1U<<5U),
	DMA_DIR		= (6U),
	DMA_CIRC	= (1U<<8U),
	DMA_PINC	= (1U<<9U),
	DMA_MINC	= (1U<<10U),

	DMA_PSIZE	= (11U),
	DMA_MSIZE	= (13U),
	DMA_PINCOS	= (1U<<15U),
	DMA_PL		= (16U),
	DMA_DBM		= (1U<<18U),

	DMA_CT		= (1U<<19U),
	DMA_PBURST	= (21U),
	DMA_MBURST	= (23U),
	DMA_CHSEL	= (25U),

}DMA_SxCR_t;


typedef enum{
	DMA_NDT		= (0U),

}DMA_SxNDTR_t;


typedef enum{
	DMA_PAR		= (0U),

}DMA_SxPAR_t;


typedef enum{
	DMA_M0A		= (0U),

}DMA_SxM0AR_t;


typedef enum{
	DMA_M1A		= (0U),

}DMA_SxM1AR_t;


typedef enum{
	DMA_FTH		= (0U),
	DMA_DMDIS   = (1U<<2U),
	DMA_FS		= (3U),
	DMA_FEIE	= (1U<<7U),
}DMA_SxFCR_t;





Status_code_t DMA_to_ADC_Init(uint32_t Peripheral_addr, uint32_t* dest_buff); //ADC es Stream 0, channel 0 or channel 4

void DMA_Clear_Stream_Flag(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, DMA_flags_t flag);
FlagStatus DMA_Get_Stream_Flag(DMAMapAddr_t DMA_Addr, DMA_streams_t stream, DMA_flags_t flag);

#endif /* DRIVERS_DMA_H_ */
