#include "delay.h"

volatile uint32_t systick;


//SYSTICK的时钟设置为内核时钟
//SYSCLK:系统时钟频率,即CPU频率(rcc_c_ck),400Mhz
void delay_init(uint16_t SYSCLK)
{
	//1MS中断一次
 	SysTick->LOAD  = (uint32_t)(SYSCLK*1000 - 1UL);	 
	SYS_NVIC_SetPriority (SysTick_IRQn, 0,0);
	SysTick->VAL   = 0UL;
	SYS_NVIC_EnableIRQ(SysTick_IRQn);
	SysTick->CTRL  = 1 << 2 | 1 << 1 | 1; //设置时钟源 中断 并使能
}


void delay_ms(uint32_t ms)
{
	uint32_t oldtime = systick;
	if(ms < 0xFFFFFFFF)
		ms += 1;//确保最小等待
	while(systick - oldtime < ms);
}




void SysTick_Handler(void)
{
	systick++;
}

