/**
 * @file    main.c
 * @brief   墨水供应系统 - 基础模板工程 (基于IIC实验)
 * @author  Template based on WKS GD32F427VET6 IIC实验
 * @date    2024
 */

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "usmart.h"
#include "24cxx.h"
#include "adc.h"
#include "dac.h"
#include "timer.h"
#include "w25qxx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"

/* lwIP includes for testing */
#include "lwip/opt.h"
#if LWIP_ACD
#include "lwip/acd.h"
#endif

/* 运行时统计相关变量 */
static uint32_t ulHighFrequencyTimerTicks = 0;

/* errno变量定义 (lwIP需要) */
int errno = 0;

/**
 * @brief   系统初始化
 * @param   无
 * @retval  无
 */
void system_init(void)
{
    HAL_Init();                         /* 初始化HAL库 */
    GD32_Clock_Init(336, 25, 2, 7);    /* 设置时钟,168Mhz */
    delay_init(168);                    /* 初始化延时函数 */
    uart_init(115200);                  /* 初始化USART */
    usmart_dev.init(84);                /* 初始化USMART */
    LED_Init();                         /* 初始化LED */
    KEY_Init();                         /* 初始化KEY */
    LCD_Init();                         /* 初始化LCD */
    AT24CXX_Init();                     /* 初始化IIC和24C02 */
}

/**
 * @brief   显示系统信息
 * @param   无
 * @retval  无
 */
void display_system_info(void)
{
    POINT_COLOR = RED;
    LCD_ShowString(30, 50, 200, 16, 16, "Ink Supply System");
    LCD_ShowString(30, 70, 200, 16, 16, "GD32F427 Template");
    LCD_ShowString(30, 90, 200, 16, 16, "WKS SMART");
    LCD_ShowString(30, 110, 200, 16, 16, "KEY_UP:Test  KEY0:Demo");

    /* 检测24C02 */
    while (AT24CXX_Check())
    {
        LCD_ShowString(30, 130, 200, 16, 16, "24C02 Check Failed!");
        delay_ms(500);
        LCD_ShowString(30, 130, 200, 16, 16, "Please Check!      ");
        delay_ms(500);
        LED0 = !LED0;
    }
    LCD_ShowString(30, 130, 200, 16, 16, "24C02 Ready!");
    POINT_COLOR = BLUE;
}

/**
 * @brief   主函数
 * @param   无
 * @retval  无
 */
int main01(void)
{
    u8 key;
    u16 led_counter = 0;
    u32 system_counter = 0;

    /* 系统初始化 */
    system_init();

    /* 显示系统信息 */
    display_system_info();

    printf("Ink Supply System Template Ready!\r\n");
    printf("System Clock: 168MHz\r\n");
    printf("Hardware: GD32F427VET6\r\n");
    printf("Template: Based on IIC Experiment\r\n");

    /* 主循环 */
    while (1)
    {
        key = KEY_Scan(0);

        /* 按键处理 */
        if (key == WKUP_PRES)
        {
            LCD_Fill(0, 170, 319, 319, WHITE);
            LCD_ShowString(30, 170, 200, 16, 16, "WKUP Key Pressed!");
            LCD_ShowString(30, 190, 200, 16, 16, "System Test Mode");
            printf("WKUP Key: System test mode activated\r\n");

            /* 这里可以添加系统测试代码 */
            delay_ms(1000);
        }

        if (key == KEY0_PRES)
        {
            LCD_Fill(0, 170, 319, 319, WHITE);
            LCD_ShowString(30, 170, 200, 16, 16, "KEY0 Pressed!");
            LCD_ShowString(30, 190, 200, 16, 16, "Demo Function");
            printf("KEY0 Key: Demo function triggered\r\n");

            /* 这里可以添加演示功能代码 */
            delay_ms(1000);
        }

        /* 系统计数和状态显示 */
        system_counter++;

        /* LED闪烁指示系统运行 */
        led_counter++;
        if (led_counter >= 50)  /* 500ms闪烁一次 */
        {
            LED0 = !LED0;
            led_counter = 0;

            /* 每5秒输出一次系统状态 */
            if (system_counter % 500 == 0)
            {
                printf("System running... Counter: %lu\r\n", system_counter);
            }
        }

        delay_ms(10);  /* 10ms延时 */
    }
}

int main02(void)
{
  u16 adcx;
	float temp;
	
	HAL_Init();                   	//初始化HAL库    
	GD32_Clock_Init(336,25,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	uart_init(115200);             	//初始化USART
	usmart_dev.init(84); 		        //初始化USMART
	LED_Init();						          //初始化LED	
	KEY_Init();						          //初始化KEY
 	LCD_Init();           			    //初始化LCD
  MY_ADC_Init();                  //初始化ADC1
    
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"GD32F427 Core Board");	
	LCD_ShowString(30,70,200,16,16,"ADC TEST");	
	LCD_ShowString(30,90,200,16,16,"WKS SMART");	  
	POINT_COLOR=BLUE;//设置字体为蓝色
	LCD_ShowString(30,130,200,16,16,"ADC1_CH5_VAL:");	      
	LCD_ShowString(30,150,200,16,16,"ADC1_CH5_VOL:0.000V");	//先在固定位置显示小数点  	
	
  while(1)
	{
    adcx=Get_Adc_Average(ADC_CHANNEL_5,20);//获取通道5的转换值，20次取平均
		LCD_ShowxNum(134,130,adcx,4,16,0);    //显示ADC采样后的原始值
		temp=(float)adcx*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		adcx=temp;                            //赋值整数部分给adcx变量，因为adcx为u16整形
		LCD_ShowxNum(134,150,adcx,1,16,0);    //显示电压值的整数部分，3.1111的话，这里就是显示3
		temp-=adcx;                           //把已经显示的整数部分去掉，留下小数部分，比如3.1111-3=0.1111
		temp*=1000;                           //小数部分乘以1000，例如：0.1111就转换为111.1，相当于保留三位小数。
		LCD_ShowxNum(150,150,temp,3,16,0X80); //显示小数部分（前面转换为了整形显示），这里显示的就是111.
		LED0=!LED0;
		delay_ms(250);	
	} 
}


//要写入到W25QXX的字符串数组
const u8 TEXT_Buffer[]={"GD32F427 Core Board SPI TEST"};
#define SIZE sizeof(TEXT_Buffer)

int main(void)
{
  u8 key;
	u16 i=0;
	u8 datatemp[SIZE];
	u32 FLASH_SIZE; 
	u16 id = 0;	
	
  HAL_Init();                   	//初始化HAL库    
  GD32_Clock_Init(336,25,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	uart_init(115200);             	//初始化USART
	usmart_dev.init(84); 		        //初始化USMART
	LED_Init();						          //初始化LED	
	KEY_Init();						          //初始化KEY
 	LCD_Init();           			    //初始化LCD
	
  W25QXX_Init();				          //W25QXX初始化
  POINT_COLOR=RED;
	LCD_ShowString(30,50,200,16,16,"GD32F427 Core Board");	
	LCD_ShowString(30,70,200,16,16,"SPI TEST");	
	LCD_ShowString(30,90,200,16,16,"WKS SMART");	 		
	LCD_ShowString(30,110,200,16,16,"KEY_UP:Write  KEY0:Read");	//显示提示信息		
	while(1)								
	{
		id = W25QXX_ReadID();
		if (id == W25Q128 || id == NM25Q128)
			break;
		LCD_ShowString(30,130,200,16,16,"W25Q128 Check Failed!"); //检测不到W25Q128
		delay_ms(500);
		LCD_ShowString(30,130,200,16,16,"Please Check!        ");
		delay_ms(500);
		LED0=!LED0;		//DS0闪烁
	}
	LCD_ShowString(30,130,200,16,16,"W25Q128 Ready!"); 
	FLASH_SIZE=16*1024*1024;	//FLASH 大小为16M字节
  POINT_COLOR=BLUE;			  //设置字体为蓝色	  
	while(1)
	{
		key=KEY_Scan(0);
		if(key==WKUP_PRES)	//KEY_UP按下,写入W25Q128
		{
			LCD_Fill(0,170,319,319,WHITE);//清除半屏    
 			LCD_ShowString(30,170,200,16,16,"Start Write W25Q128....");
			W25QXX_Write((u8*)TEXT_Buffer,FLASH_SIZE-100,SIZE);		//从倒数第100个地址处开始,写入SIZE长度的数据
			LCD_ShowString(30,170,200,16,16,"W25Q128 Write Finished!");	//提示传送完成
		}
		if(key==KEY0_PRES) //KEY0按下,读取字符串并显示
		{
 			LCD_ShowString(30,170,200,16,16,"Start Read W25Q128.... ");
			W25QXX_Read(datatemp,FLASH_SIZE-100,SIZE);					//从倒数第100个地址处开始,读出SIZE个字节
			LCD_ShowString(30,170,200,16,16,"The Data Readed Is:   ");	//提示传送完成
			LCD_ShowString(30,190,290,16,16,datatemp);					//显示读到的字符串
		} 
		i++;
		delay_ms(10);
		if(i==20)
		{
			LED0=!LED0;//提示系统正在运行	
			i=0;
		}
	}
}

/**
 * @brief  FreeRTOS时钟节拍钩子函数
 * @param  无
 * @retval 无
 * @note   此函数在每个时钟节拍中断中被调用，用于维护HAL库的时钟
 */
void vApplicationTickHook(void)
{
    HAL_IncTick();  /* 维护HAL库的时钟计数 */
}

/**
 * @brief  FreeRTOS堆栈溢出钩子函数
 * @param  pxTask 溢出的任务句柄
 * @param  pcTaskName 溢出的任务名称
 * @retval 无
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    /* 堆栈溢出处理 */
    (void)pxTask;
    (void)pcTaskName;

    /* 在调试时可以在此处设置断点 */
    while(1)
    {
        /* 死循环指示堆栈溢出 */
    }
}

/**
 * @brief  FreeRTOS内存分配失败钩子函数
 * @param  无
 * @retval 无
 */
void vApplicationMallocFailedHook(void)
{
    /* 内存分配失败处理 */
    while(1)
    {
        /* 死循环指示内存分配失败 */
    }
}

/**
 * @brief  配置用于运行时统计的定时器
 * @param  无
 * @retval 无
 * @note   简单实现，使用SysTick计数
 */
void vConfigureTimerForRunTimeStats(void)
{
    /* 重置计数器 */
    ulHighFrequencyTimerTicks = 0;
}

/**
 * @brief  获取运行时计数器的值
 * @param  无
 * @retval 计数器值
 * @note   简单实现，使用SysTick * 10来模拟高频计数器
 */
uint32_t ulGetRunTimeCounterValue(void)
{
    /* 返回基于系统tick的高频计数 */
    return xTaskGetTickCount() * 10;
}