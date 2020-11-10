#include "delay.h"

static uint16_t  fac_us=0;							//us��ʱ������	
//SYSTICK��ʱ������Ϊ�ں�ʱ��
//SYSCLK:ϵͳʱ��Ƶ��,��CPUƵ��(rcc_c_ck),400Mhz
void delay_init(uint16_t SYSCLK)
{
	//1MS�ж�һ��
 	SysTick->LOAD  = (uint32_t)(SYSCLK*1000 - 1UL);	 
	NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	SysTick->VAL   = 0UL;
	SysTick->CTRL  = 1 << 2 | 1 << 1 | 1; //����ʱ��Դ �ж� ��ʹ��
}


//��ʱnus
//nusΪҪ��ʱ��us��.	
//ע��:nus��ֵ,��Ҫ����41943us(���ֵ��2^24/fac_us@fac_us=400)
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//ʱ�����	  		 
	SysTick->VAL=0x00;        				//��ռ�����
	SysTick->CTRL|=1<<0 ;          			//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~(1<<0) ;       			//�ر�SYSTICK
	SysTick->VAL =0X00;       				//��ռ����� 
}


