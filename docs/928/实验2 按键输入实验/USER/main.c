#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
/************************************************
 WKS GD32F427VET6���İ� 
 ��������ʵ��-HAL�⺯����
************************************************/

int main(void)
{
	  u8 key;
	
    HAL_Init();                    	//��ʼ��HAL��    
    GD32_Clock_Init(336,25,2,7);  	//����ʱ��,168Mhz
	  delay_init(168);               	//��ʼ����ʱ����
	  LED_Init();					            //��ʼ��LED	
    KEY_Init();                     //��ʼ������
	  LED0=0;					                //����LED0
	
    while(1)
    {
        key=KEY_Scan(1);            //����ɨ�裬0:��֧��������;1:֧��������
		switch(key)
		{				 
			case WKUP_PRES:			
				LED1=!LED1;
			  delay_ms(100);              //��ʱһ��ʱ�䣬����۲�����ʱ��Ч��
				break;
			case KEY0_PRES:			
				LED0=!LED0;
			  delay_ms(100);
				break;
		}
        delay_ms(10);
	}
}

