#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427ZVET6���İ�
//SPI ��������	   
//�汾��V1.0								  
////////////////////////////////////////////////////////////////////////////////// 	

extern SPI_HandleTypeDef SPI1_Handler;  //SPI���

void SPI1_Init(void);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
u8 SPI1_ReadWriteByte(u8 TxData);
#endif
