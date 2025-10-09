#ifndef __DMA_H
#define __DMA_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6ºËÐÄ°å
//DMA Çý¶¯´úÂë	   
//°æ±¾£ºV1.0								  
////////////////////////////////////////////////////////////////////////////////// 	

extern DMA_HandleTypeDef  UART1TxDMA_Handler;      //DMA¾ä±ú

void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx);
void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
#endif
