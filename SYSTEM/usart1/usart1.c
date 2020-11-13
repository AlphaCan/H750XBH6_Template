#include "usart1.h"
#include "stdio.h"

//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->ISR&0X40)==0);//ѭ������,ֱ���������   
	USART1->TDR = (uint8_t) ch;      
	return ch;
}


/*********************************************************************
//	@param		:
//		1.uint32_t bound ������
//		2. 
//		3.
//		4.
//		5.
//	@Description	:
//	����1��ʼ��
//
//	@Author		:	alpha
//	@Date		:	2020.11.13
*********************************************************************/	
void USART_Init(uint32_t bound)
{
	uint32_t boundtemp = 0;
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
	
	boundtemp = (100*100000+bound/2)/bound;//��������
	USART1->BRR = boundtemp;
	USART1->CR1|=1<<0;//ʹ�ܴ���
	
	SYS_NVIC_SetPriority(USART1_IRQn,1,0);
	SYS_NVIC_EnableIRQ(USART1_IRQn);
	 
	
}

void USART1_IRQHandler()
{
	
}






