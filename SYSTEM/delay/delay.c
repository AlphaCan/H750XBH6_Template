#include "delay.h"

static uint16_t  fac_us=0;							//us延时倍乘数	
//SYSTICK的时钟设置为内核时钟
//SYSCLK:系统时钟频率,即CPU频率(rcc_c_ck),400Mhz
void delay_init(uint16_t SYSCLK)
{
	//1MS中断一次
 	SysTick->LOAD  = (uint32_t)(SYSCLK*1000 - 1UL);	 
	NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	SysTick->VAL   = 0UL;
	SysTick->CTRL  = 1 << 2 | 1 << 1 | 1; //设置时钟源 中断 并使能
}


//延时nus
//nus为要延时的us数.	
//注意:nus的值,不要大于41943us(最大值即2^24/fac_us@fac_us=400)
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL|=1<<0 ;          			//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL&=~(1<<0) ;       			//关闭SYSTICK
	SysTick->VAL =0X00;       				//清空计数器 
}


