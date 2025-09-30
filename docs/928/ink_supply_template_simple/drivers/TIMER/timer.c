#include "timer.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//WKS GD32F427VET6���İ�
//PWM��� ��������		  
//�汾��V1.0
////////////////////////////////////////////////////////////////////////////////// 

TIM_HandleTypeDef TIM2_Handler;      	//��ʱ����� 
TIM_OC_InitTypeDef TIM2_CH2Handler;	//��ʱ��2ͨ��1���


//TIM14 PWM���ֳ�ʼ�� 
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
void TIM2_PWM_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                         	   //��ʱ��2
    TIM2_Handler.Init.Prescaler=psc;                         //��ʱ����Ƶϵ��
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //���ϼ���ģʽ
    TIM2_Handler.Init.Period=arr;                            //�Զ���װ��ֵ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM2_Handler);                         //��ʼ��PWM
    
    TIM2_CH2Handler.OCMode=TIM_OCMODE_PWM1;                  //ģʽѡ��PWM1
    TIM2_CH2Handler.Pulse=arr/2;                             //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM2_CH2Handler.OCPolarity=TIM_OCPOLARITY_LOW;           //����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH2Handler,TIM_CHANNEL_2);//����TIM2ͨ��2
	
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_2);//����PWMͨ��2
}


//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM2_CLK_ENABLE();			      //TIM2ʱ��ʹ��
	__HAL_RCC_GPIOA_CLK_ENABLE();			      //����GPIOAʱ��
	
	GPIO_Initure.Pin=GPIO_PIN_1;           	//PA1
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	  //�����������
	GPIO_Initure.Pull=GPIO_PULLUP;          //����
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	GPIO_Initure.Alternate= GPIO_AF1_TIM2;	//PA1����ΪTIM2_CH2
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

//����TIM2ͨ��2��ռ�ձ�
//compare:�Ƚ�ֵ
void TIM_SetTIM2Compare1(u32 compare)
{
	TIM2->CCR2=compare; 
}




