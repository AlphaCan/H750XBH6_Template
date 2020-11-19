#include "sys.h"
#include "delay.h"
#include "usart1.h"

int main (void)
{
	//Cache_Enable();
	SYS_NVIC_SetPriorityGroup(NVIC_PRIORITYGROUP_4);
	Stm32_Clock_Init(160,5,2,4);
	delay_init(400);
	USART_Init(115200);
	
	RCC->AHB4ENR |= 1<<1;	//´ò¿ªGPIOBÊ±ÖÓ
	SYS_GPIO_Init(GPIOB,GPIO_PIN_1|GPIO_PIN_0, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_PU);
	
	
	while(1)
	{
		SYS_GPIO_TogglePin(GPIOB,GPIO_PIN_1|GPIO_PIN_0);
		delay_ms(1000);
		printf("567\r\n");
	}
	
	
	
	return 0;
}













