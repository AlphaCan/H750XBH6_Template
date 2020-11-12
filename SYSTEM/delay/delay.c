#include "delay.h"

volatile uint32_t systick;


//SYSTICK��ʱ������Ϊ�ں�ʱ��
//SYSCLK:ϵͳʱ��Ƶ��,��CPUƵ��(rcc_c_ck),400Mhz
void delay_init(uint16_t SYSCLK)
{
	//1MS�ж�һ��
 	SysTick->LOAD  = (uint32_t)(SYSCLK*1000 - 1UL);	 
	SYS_NVIC_SetPriority (SysTick_IRQn, 0,0);
	SysTick->VAL   = 0UL;
	SYS_NVIC_EnableIRQ(SysTick_IRQn);
	SysTick->CTRL  = 1 << 2 | 1 << 1 | 1; //����ʱ��Դ �ж� ��ʹ��
}


void delay_ms(uint32_t ms)
{
	uint32_t oldtime = systick;
	if(ms < 0xFFFFFFFF)
		ms += 1;//ȷ����С�ȴ�
	while(systick - oldtime < ms);
}




void SysTick_Handler(void)
{
	systick++;
}

