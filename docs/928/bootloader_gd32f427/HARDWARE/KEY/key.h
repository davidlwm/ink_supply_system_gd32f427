#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6���İ�
//����������������	   
//�汾��V1.0											  
//////////////////////////////////////////////////////////////////////////////////

//����ķ�ʽ��ͨ��λ��������ʽ��ȡIO
//#define KEY0        PEin(4) //KEY0����PE4
//#define WK_UP       PAin(0) //WKUP����PA0


//����ķ�ʽ��ͨ��ֱ�Ӳ���HAL�⺯����ʽ��ȡIO
#define KEY0        HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)  //KEY0����PC0
#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP����PA0

#define KEY0_PRES 	1
#define WKUP_PRES   2

void KEY_Init(void);     //������ʼ������
u8 KEY_Scan(u8 mode);    //����ɨ�躯��
#endif
