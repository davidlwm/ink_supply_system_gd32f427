#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6核心板
//按键输入驱动代码	   
//版本：V1.0									  
//////////////////////////////////////////////////////////////////////////////////

//按键初始化函数
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟

    GPIO_Initure.Pin=GPIO_PIN_0;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
    
	  GPIO_Initure.Pin=GPIO_PIN_0;            //PC0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);    
}

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY_UP按下 即WK_UP
//注意此函数有响应优先级,KEY0>KEY_UP!!
u8 KEY_Scan(u8 mode)
{
    static u8 key_up=1;     //按键松开标志
    if(mode==1)key_up=1;    //支持连按
    if(key_up&&(KEY0==1||WK_UP==1))
    {
        delay_ms(10);       //去抖动
        key_up=0;
        if(KEY0==1)       return KEY0_PRES;
        else if(WK_UP==1) return WKUP_PRES;          
    }else if(KEY0==0&&WK_UP==0)key_up=1;
    return 0;   //无按键按下
}
