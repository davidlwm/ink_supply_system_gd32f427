#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6核心板
//LED驱动代码	   
//版本：V1.0						  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化PA1和PA2为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟
	
    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_2; //PA1,2
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);	//PA1置1，默认初始化后灯灭
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);	//PA2置1，默认初始化后灯灭
}
