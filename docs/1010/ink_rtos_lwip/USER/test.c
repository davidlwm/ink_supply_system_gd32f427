#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "lcd.h"
//#include "ltdc.h"
//#include "sdram.h"
#include "key.h"
#include "usmart.h"
#include "malloc.h"
#include "timer.h"
//#include "adc.h"
//#include "rtc.h"
#include "pcf8574.h"
#include "lwip_comm.h"
//#include "tcp_client_demo.h"
#include "tcp_server_demo.h"
//#include "udp_demo.h"
//#include "httpd.h"
#include "test2.h"  // FreeRTOS test
//YTCE ԭҰ�������STM32F429������ ʵ��58
//����ͨ�� ʵ��
//����֧�֣�s8088.taobao.com
//ԭҰ������� 

//����UI
//mode:
//bit0:0,������;1,����ǰ�벿��UI
//bit1:0,������;1,���غ�벿��UI
void lwip_test_ui(u8 mode)
{
	u8 speed;
	u8 buf[30]; 
	POINT_COLOR=RED;
	if(mode&1<<0)
	{
		LCD_Fill(30,30,lcddev.width,110,WHITE);	//�����ʾ
		LCD_ShowString(30,30,200,16,16,"YTCE STM32F4/F7");
		LCD_ShowString(30,50,200,16,16,"Ethernet lwIP Test");
		LCD_ShowString(30,70,200,16,16,"ATOM@YTCE");
		LCD_ShowString(30,90,200,16,16,"2016/1/25"); 	
	}
	if(mode&1<<1)
	{
		LCD_Fill(30,110,lcddev.width,lcddev.height,WHITE);	//�����ʾ
		LCD_ShowString(30,110,200,16,16,"lwIP Init Successed");
		if(lwipdev.dhcpstatus==2)sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//��ӡ��̬IP��ַ
		else sprintf((char*)buf,"Static IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//��ӡ��̬IP��ַ
		LCD_ShowString(30,130,210,16,16,buf); 
		speed=LAN8720_Get_Speed();//�õ�����
		if(speed&1<<1)LCD_ShowString(30,150,200,16,16,"Ethernet Speed:100M");
		else LCD_ShowString(30,150,200,16,16,"Ethernet Speed:10M");
		LCD_ShowString(30,170,200,16,16,"KEY0:TCP Server Test");
		LCD_ShowString(30,190,200,16,16,"KEY1:TCP Client Test");
		LCD_ShowString(30,210,200,16,16,"KEY2:UDP Test");
	}
}

// Original main function - renamed to main01
int main01(void)
{   
	u8 t;
	u8 key;
	Stm32_Clock_Init(360,25,2,8);//����ʱ��,180Mhz
	delay_init(180);			//��ʼ����ʱ���� 
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200 
   	LED_Init();					//��ʼ����LED���ӵ�Ӳ���ӿ�
	//SDRAM_Init();				//��ʼ��SDRAM 
	LCD_Init();					//��ʼ��LCD
	KEY_Init();					//��ʼ������
	//RTC_Init();  				//RTC��ʼ��
	//Adc_Init();  				//ADC��ʼ�� 
	PCF8574_Init();				//��ʼ��PCF8574
	TIM3_Int_Init(100-1,9000-1);//10khz��Ƶ��,����100Ϊ10ms
	usmart_dev.init(90);		//��ʼ��USMART
 	//my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	//my_mem_init(SRAMEX);		//��ʼ���ⲿ�ڴ��
	//my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ��
	POINT_COLOR=RED;
	LED0=0;
	lwip_test_ui(1);			//����ǰ�벿��UI
	//�ȳ�ʼ��lwIP(����LAN8720��ʼ��),��ʱ�����������,�����ʼ����ʧ��!! 
	LCD_ShowString(30,110,200,16,16,"lwIP Initing...");
	while(lwip_comm_init()!=0)
	{
		LCD_ShowString(30,110,200,16,16,"lwIP Init failed!");
		delay_ms(1200);
		LCD_Fill(30,110,230,110+16,WHITE);//�����ʾ
		LCD_ShowString(30,110,200,16,16,"Retrying...");  
	}
	LCD_ShowString(30,110,200,16,16,"lwIP Init Successed");
	//�ȴ�DHCP��ȡ 
 	LCD_ShowString(30,130,200,16,16,"DHCP IP configing...");
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//�ȴ�DHCP��ȡ�ɹ�/��ʱ���
	{
		lwip_periodic_handle();
	}
	lwip_test_ui(2);//���غ�벿��UI 
	//httpd_init();	//HTTP��ʼ��(Ĭ�Ͽ���websever)
	while(1)
	{
		key=KEY_Scan(0);
		switch(key)
		{
			case KEY0_PRES://TCP Serverģʽ
				tcp_server_test();
 				lwip_test_ui(3);//���¼���UI  
				break;
			/*
			case KEY1_PRES://TCP Clientģʽ
				tcp_client_test();
				lwip_test_ui(3);//���¼���UI
				break; 
			case KEY2_PRES://UDPģʽ
				udp_demo_test();
				lwip_test_ui(3);//���¼���UI
				break; */
		} 
		lwip_periodic_handle();
		delay_ms(2);
		t++;
		if(t==100)LCD_ShowString(30,230,200,16,16,"Please choose a mode!");
		if(t==200)
		{ 
			t=0;
			LCD_Fill(30,230,230,230+16,WHITE);//�����ʾ
			LED0=!LED0;
		} 
	}
}

/**
 * @brief  New main function to test FreeRTOS integration
 * @retval None
 */
int main(void)
{
	HAL_Init();                     // 初始化HAL库
	Stm32_Clock_Init(360,25,2,8);   // 设置时钟,180Mhz
	delay_init(180);                // 初始化延时函数
	uart_init(115200);              // 初始化串口波特率为115200
	LED_Init();                     // 初始化与LED连接的硬件接口
	LCD_Init();                     // 初始化LCD
	KEY_Init();                     // 初始化按键
	PCF8574_Init();                 // 初始化PCF8574

	// 设置LCD显示颜色
	POINT_COLOR = RED;
	LED0 = 0;

	// 清屏并显示标题
	LCD_Fill(0, 0, lcddev.width, lcddev.height, WHITE);
	LCD_ShowString(30, 30, 200, 16, 16, "GD32F427 FreeRTOS Test");
	LCD_ShowString(30, 50, 200, 16, 16, "System Clock: 180MHz");
	LCD_ShowString(30, 70, 200, 16, 16, "UART: 115200 bps");

	// 显示初始化进度
	LCD_ShowString(30, 110, 200, 16, 16, "FreeRTOS Initializing...");
	delay_ms(500);

	// 测试FreeRTOS配置 - 使用LCD显示而不是printf
	Test_RuntimeStats();
	Test_HookFunctions();

	LCD_ShowString(30, 130, 200, 16, 16, "Creating Tasks...");
	delay_ms(500);

	// 初始化FreeRTOS测试任务
	if (FreeRTOS_Test_Init() == pdPASS)
	{
		LCD_ShowString(30, 150, 200, 16, 16, "Tasks Created!");
		LCD_ShowString(30, 170, 200, 16, 16, "Starting Scheduler...");
		delay_ms(1000);

		// 启动调度器 - 不会返回
		vTaskStartScheduler();
	}
	else
	{
		LCD_ShowString(30, 150, 200, 16, 16, "ERROR: Init Failed!");
	}

	// 不应该到达这里
	LCD_ShowString(30, 190, 200, 16, 16, "ERROR: Start Failed!");
	while(1)
	{
		LED0 = !LED0;
		delay_ms(500);
	}
}

















