#ifndef __SYS_H
#define __SYS_H	

#include "stm32h7xx.h"

//���ȼ�����
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


#define GPIO_PIN_0                 ((uint16_t)(1<<0))  //0����
#define GPIO_PIN_1                 ((uint16_t)(1<<1))  //1����
#define GPIO_PIN_2                 ((uint16_t)(1<<2))  //2����
#define GPIO_PIN_3                 ((uint16_t)(1<<3))  //3����
#define GPIO_PIN_4                 ((uint16_t)(1<<4))  //4����
#define GPIO_PIN_5                 ((uint16_t)(1<<5))  //5����
#define GPIO_PIN_6                 ((uint16_t)(1<<6))  //6����
#define GPIO_PIN_7                 ((uint16_t)(1<<7))  //7����
#define GPIO_PIN_8                 ((uint16_t)(1<<8))  //8����
#define GPIO_PIN_9                 ((uint16_t)(1<<9))  //9����
#define GPIO_PIN_10                 ((uint16_t)(1<<10))  //10����
#define GPIO_PIN_11                 ((uint16_t)(1<<11))  //11����
#define GPIO_PIN_12                 ((uint16_t)(1<<12))  //12����
#define GPIO_PIN_13                 ((uint16_t)(1<<13))  //13����
#define GPIO_PIN_14                 ((uint16_t)(1<<14))  //14����
#define GPIO_PIN_15                 ((uint16_t)(1<<15))  //15����

//GPIOģʽ
#define GPIO_MODE_IN    	0		//��ͨ����ģʽ
#define GPIO_MODE_OUT		1		//��ͨ���ģʽ
#define GPIO_MODE_AF		2		//AF����ģʽ
#define GPIO_MODE_AIN		3		//ģ������ģʽ

//�����ٶ�
#define GPIO_SPEED_LOW		0		//GPIO�ٶ�(����,12M)
#define GPIO_SPEED_MID		1		//GPIO�ٶ�(����,60M)
#define GPIO_SPEED_FAST		2		//GPIO�ٶ�(����,85M)
#define GPIO_SPEED_HIGH		3		//GPIO�ٶ�(����,100M)  

//������/����
#define GPIO_PUPD_NONE		0		//����������
#define GPIO_PUPD_PU		1		//����
#define GPIO_PUPD_PD		2		//����
#define GPIO_PUPD_RES		3		//���� 

//���ģʽ
#define GPIO_OTYPE_PP		0		//�������
#define GPIO_OTYPE_OD		1		//��©��� 




void SYS_QSPI_Enable_Memmapmode(void);















#endif
