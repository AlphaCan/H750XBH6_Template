#include "usart1.h"


void USART_Init()
{
	RCC->AHB4ENR |= 1<<0;	//打开GPIOA时钟
	RCC->APB2ENR |= 1<<4;	//打开USART1时钟
	
	SYS_GPIO_Init(GPIOA,GPIO_PIN_10|GPIO_PIN_9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);
	SYS_GPIO_AF_Set(GPIOA,GPIO_PIN_10,7);
	SYS_GPIO_AF_Set(GPIOA,GPIO_PIN_9,7);
	
	USART1->CR1 = 0;//配置寄存器，首先必须关闭串口使能
	USART1->CR1|= 0<<28; //设置M1 1个起始位 8个数据位 n个停止位
	USART1->CR1|= 0<<12; //设置M0字长为8
	USART1->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
	USART1->CR1|=1<<3;  	//串口发送使能 
	USART1->CR1|=1<<2;  	//串口接收使能
	USART1->CR1|=1<<5;    	//接收缓冲区非空中断使能
	SYS_NVIC_SetPriority(USART1_IRQn,1,0);
	SYS_NVIC_EnableIRQ(USART1_IRQn);
	 
	
}








