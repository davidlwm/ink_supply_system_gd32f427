#include "exti.h"
#include "delay.h"
#include "led.h"
#include "key.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6核心板
//外部中断 驱动代码	   
//版本：V1.0								  
////////////////////////////////////////////////////////////////////////////////// 	

//外部中断初始化
void EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOC_CLK_ENABLE();               //开启GPIOC时钟

    GPIO_Initure.Pin=GPIO_PIN_0; 				        //PC0
    GPIO_Initure.Mode=GPIO_MODE_IT_RISING;      //上升沿触发
    GPIO_Initure.Pull=GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
    
    //中断线0-PA0
    HAL_NVIC_SetPriority(EXTI0_IRQn,0,2);       //抢占优先级为0，子优先级为2
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);             //使能中断线0
    
}


//中断服务函数
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);		//调用中断处理公用函数
}



//中断服务程序中需要做的事情
//在HAL库中所有的外部中断服务函数都会调用此函数
//GPIO_Pin:中断引脚号
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(100);      //消抖
    switch(GPIO_Pin)
    {
        case GPIO_PIN_0:
            if(KEY0==1)  
            {
				LED0=!LED0;//控制LED0翻转
            }
            break;
    }
}
