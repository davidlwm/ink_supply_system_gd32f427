#include "timer.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6核心板
//PWM输出 驱动代码		  
//版本：V1.0
////////////////////////////////////////////////////////////////////////////////// 

TIM_HandleTypeDef TIM2_Handler;      	//定时器句柄 
TIM_OC_InitTypeDef TIM2_CH2Handler;	//定时器2通道1句柄


//TIM14 PWM部分初始化 
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void TIM2_PWM_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                         	   //定时器2
    TIM2_Handler.Init.Prescaler=psc;                         //定时器分频系数
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //向上计数模式
    TIM2_Handler.Init.Period=arr;                            //自动重装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM2_Handler);                         //初始化PWM
    
    TIM2_CH2Handler.OCMode=TIM_OCMODE_PWM1;                  //模式选择PWM1
    TIM2_CH2Handler.Pulse=arr/2;                             //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM2_CH2Handler.OCPolarity=TIM_OCPOLARITY_LOW;           //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH2Handler,TIM_CHANNEL_2);//配置TIM2通道2
	
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_2);//开启PWM通道2
}


//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM2_CLK_ENABLE();			      //TIM2时钟使能
	__HAL_RCC_GPIOA_CLK_ENABLE();			      //开启GPIOA时钟
	
	GPIO_Initure.Pin=GPIO_PIN_1;           	//PA1
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	  //复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF1_TIM2;	//PA1复用为TIM2_CH2
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

//设置TIM2通道2的占空比
//compare:比较值
void TIM_SetTIM2Compare1(u32 compare)
{
	TIM2->CCR2=compare; 
}




