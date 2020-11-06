#ifndef __SYS_H
#define __SYS_H	

#include "stm32h7xx.h"

//优先级分组
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bits for pre-emption priority
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bits for pre-emption priority
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority
                                                                 1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority
                                                                 0 bits for subpriority */


#define GPIO_PIN_0                 ((uint16_t)(1<<0))  //0引脚
#define GPIO_PIN_1                 ((uint16_t)(1<<1))  //1引脚
#define GPIO_PIN_2                 ((uint16_t)(1<<2))  //2引脚
#define GPIO_PIN_3                 ((uint16_t)(1<<3))  //3引脚
#define GPIO_PIN_4                 ((uint16_t)(1<<4))  //4引脚
#define GPIO_PIN_5                 ((uint16_t)(1<<5))  //5引脚
#define GPIO_PIN_6                 ((uint16_t)(1<<6))  //6引脚
#define GPIO_PIN_7                 ((uint16_t)(1<<7))  //7引脚
#define GPIO_PIN_8                 ((uint16_t)(1<<8))  //8引脚
#define GPIO_PIN_9                 ((uint16_t)(1<<9))  //9引脚
#define GPIO_PIN_10                 ((uint16_t)(1<<10))  //10引脚
#define GPIO_PIN_11                 ((uint16_t)(1<<11))  //11引脚
#define GPIO_PIN_12                 ((uint16_t)(1<<12))  //12引脚
#define GPIO_PIN_13                 ((uint16_t)(1<<13))  //13引脚
#define GPIO_PIN_14                 ((uint16_t)(1<<14))  //14引脚
#define GPIO_PIN_15                 ((uint16_t)(1<<15))  //15引脚

//GPIO模式
#define GPIO_MODE_IN    	0		//普通输入模式
#define GPIO_MODE_OUT		1		//普通输出模式
#define GPIO_MODE_AF		2		//AF功能模式
#define GPIO_MODE_AIN		3		//模拟输入模式

//设置速度
#define GPIO_SPEED_LOW		0		//GPIO速度(低速,12M)
#define GPIO_SPEED_MID		1		//GPIO速度(中速,60M)
#define GPIO_SPEED_FAST		2		//GPIO速度(快速,85M)
#define GPIO_SPEED_HIGH		3		//GPIO速度(高速,100M)  

//设置上/下拉
#define GPIO_PUPD_NONE		0		//不带上下拉
#define GPIO_PUPD_PU		1		//上拉
#define GPIO_PUPD_PD		2		//下拉
#define GPIO_PUPD_RES		3		//保留 

//输出模式
#define GPIO_OTYPE_PP		0		//推挽输出
#define GPIO_OTYPE_OD		1		//开漏输出 




void SYS_QSPI_Enable_Memmapmode(void);















#endif
