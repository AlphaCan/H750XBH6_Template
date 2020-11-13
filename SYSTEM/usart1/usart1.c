#include "usart1.h"
#include "stdio.h"

//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART1->TDR = (uint8_t) ch;      
	return ch;
}


/*********************************************************************
//	@param		:
//		1.uint32_t bound 波特率
//		2. 
//		3.
//		4.
//		5.
//	@Description	:
//	串口1初始化
//
//	@Author		:	alpha
//	@Date		:	2020.11.13
*********************************************************************/	
void USART_Init(uint32_t bound)
{
	uint32_t boundtemp = 0;
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
	
	boundtemp = (100*100000+bound/2)/bound;//四舍五入
	USART1->BRR = boundtemp;
	USART1->CR1|=1<<0;//使能串口
	
	SYS_NVIC_SetPriority(USART1_IRQn,1,0);
	SYS_NVIC_EnableIRQ(USART1_IRQn);
	 
	
}

void USART1_IRQHandler()
{
	
}






