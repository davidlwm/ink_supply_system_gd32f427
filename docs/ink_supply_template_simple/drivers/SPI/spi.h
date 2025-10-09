#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427ZVET6ºËÐÄ°å
//SPI Çý¶¯´úÂë	   
//°æ±¾£ºV1.0								  
////////////////////////////////////////////////////////////////////////////////// 	

extern SPI_HandleTypeDef SPI1_Handler;  //SPI¾ä±ú

void SPI1_Init(void);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
u8 SPI1_ReadWriteByte(u8 TxData);
#endif
