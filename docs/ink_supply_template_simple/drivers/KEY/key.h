#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6核心板
//按键输入驱动代码	   
//版本：V1.0											  
//////////////////////////////////////////////////////////////////////////////////

//下面的方式是通过位带操作方式读取IO
//#define KEY0        PEin(4) //KEY0按键PE4
//#define WK_UP       PAin(0) //WKUP按键PA0


//下面的方式是通过直接操作HAL库函数方式读取IO
#define KEY0        HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)  //KEY0按键PC0
#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP按键PA0

#define KEY0_PRES 	1
#define WKUP_PRES   2

void KEY_Init(void);     //按键初始化函数
u8 KEY_Scan(u8 mode);    //按键扫描函数
#endif
