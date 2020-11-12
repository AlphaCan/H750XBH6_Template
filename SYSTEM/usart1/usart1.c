#include "usart1.h"


void USART_Init()
{
	RCC->AHB4ENR |= 1<<0;	//��GPIOAʱ��
	RCC->APB2ENR |= 1<<4;	//��USART1ʱ��
	
	SYS_GPIO_Init(GPIOA,GPIO_PIN_10|GPIO_PIN_9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);
	SYS_GPIO_AF_Set(GPIOA,GPIO_PIN_10,7);
	SYS_GPIO_AF_Set(GPIOA,GPIO_PIN_9,7);
	
	USART1->CR1 = 0;//���üĴ��������ȱ���رմ���ʹ��
	USART1->CR1|= 0<<28; //����M1 1����ʼλ 8������λ n��ֹͣλ
	USART1->CR1|= 0<<12; //����M0�ֳ�Ϊ8
	USART1->CR1|=0<<15; 	//����OVER8=0,16�������� 
	USART1->CR1|=1<<3;  	//���ڷ���ʹ�� 
	USART1->CR1|=1<<2;  	//���ڽ���ʹ��
	USART1->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��
	SYS_NVIC_SetPriority(USART1_IRQn,1,0);
	SYS_NVIC_EnableIRQ(USART1_IRQn);
	 
	
}








