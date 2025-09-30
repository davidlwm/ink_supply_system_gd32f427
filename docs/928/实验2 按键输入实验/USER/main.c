#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
/************************************************
 WKS GD32F427VET6核心板 
 按键输入实验-HAL库函数版
************************************************/

int main(void)
{
	  u8 key;
	
    HAL_Init();                    	//初始化HAL库    
    GD32_Clock_Init(336,25,2,7);  	//设置时钟,168Mhz
	  delay_init(168);               	//初始化延时函数
	  LED_Init();					            //初始化LED	
    KEY_Init();                     //初始化按键
	  LED0=0;					                //点亮LED0
	
    while(1)
    {
        key=KEY_Scan(1);            //按键扫描，0:不支持连续按;1:支持连续按
		switch(key)
		{				 
			case WKUP_PRES:			
				LED1=!LED1;
			  delay_ms(100);              //延时一定时间，方便观察连按时的效果
				break;
			case KEY0_PRES:			
				LED0=!LED0;
			  delay_ms(100);
				break;
		}
        delay_ms(10);
	}
}

