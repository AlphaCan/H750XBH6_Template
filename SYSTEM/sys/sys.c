#include "sys.h"

/*********************************************************************
//	@param		:
//		1.NVIC_VecTab ����ַ
//		2.Offset ƫ����
//		3.
//		4.
//		5.
//	@Description	:
//	�����ж�����ƫ�Ƶ�ַ
//	����NVIC��������ƫ�ƼĴ���,VTOR��9λ����,��[8:0]������
//	@Author		:	alpha
//	@Date		:	2020.11.4
*********************************************************************/	

void SYS_NVIC_SetVectorTable(uint32_t NVIC_VecTab,uint32_t Offset)
{
	SCB->VTOR = NVIC_VecTab|(Offset&(uint32_t)0xFFFFFE00);
}

/*********************************************************************
//	@param		:
//		1.PriorityGroup ���ȼ����飬ֻ����3...7����sys.h�궨��
//		2.
//		3.
//		4.
//		5.
//	@Description	:
//	����NVIC����
//
//	@Author		:	alpha
//	@Date		:	2020.11.4
*********************************************************************/	
void SYS_NVIC_SetPriorityGroup(uint32_t PriorityGroup)
{
	NVIC_SetPriorityGrouping(PriorityGroup);
}

/*********************************************************************
//	@param		:
//		1.IRQ_Number �жϺ�0~149
//		2.PreemptPriority ��ռ���ȼ�
//		3.SubPriority ��Ӧ���ȼ�
//		4.
//		5.
//	@Description	:
//	�����ж����ȼ�
//
//	@Author		:	alpha
//	@Date		:	2020.11.4
*********************************************************************/	
void SYS_NVIC_SetPriority(IRQn_Type IRQ_Number,uint32_t PreemptPriority, uint32_t SubPriority)
{
	uint32_t prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(IRQ_Number,NVIC_EncodePriority(prioritygroup,PreemptPriority,SubPriority));
}


/*********************************************************************
//	@param		:
//		1.IRQ_Number �жϺ�0~149
//		2.
//		3.
//		4.
//		5.
//	@Description	:
//	ʹ���ж�
//
//	@Author		:	alpha
//	@Date		:	2020.11.5
*********************************************************************/	
void SYS_NVIC_EnableIRQ(IRQn_Type IRQ_Number)
{
	NVIC_EnableIRQ(IRQ_Number);
}

/*********************************************************************
//	@param		:
//		1.IRQ_Number �жϺ�0~149
//		2.
//		3.
//		4.
//		5.
//	@Description	:
//	ʧ���ж�
//
//	@Author		:	
//	@Date		:
*********************************************************************/	
void SYS_NVIC_DisableIRQ(IRQn_Type IRQ_Number)
{
	NVIC_DisableIRQ(IRQ_Number);
}

/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	��ΧGPIOA~GPIOK.
//		2.uint32_t Pin			���ű��0~15
//		3.uint32_t Alternate	���ù��ܱ��
//		AF0:SYS		AF1��TIM1/2/16/17/LPTIM1/HRTIM1		AF2:SAI1/TIM3/4/5/12/HRTIM1		AF3:LPUART/TIM8/LPTIM2/3/4/5/HRTIM1/DFSDM1
//		AF4:I2C1/2/3/4/USART1/TIM15/LPTIM2/DFSDM1/CEC	AF5:SPI1/2/3/4/5/6/CEC			AF6:SPI2/3/SAI1/3/I2C4/UART4/DFSDM1
//		AF7:SPI2/3/6/USART1/2/3/6/UART7/SDMMC1			AF8:SPI6/SAI2/4/UART4/5/8/LPUART/SDMMC1/SPDIFRX1
//		AF9:SAI4/FDCAN1/2/QUADSPI/FMC/SDMMC2/LCD/SPDIFRX1								AF10:SAI2/4/TIM8/QUADSPI/SDMMC2/OTG1_HS/OTG2_FS/LCD
//		AF11:I2C4/UART7/SWPMI1/DFSDM1/SDMMC2/MDIOS/ETH									AF12:TIM1/8/FMC/SDMMC1/MDIOS/OTG1_FS/LCD
//		AF13:TIM1/DCMI/LCD/COMP							AF14:UART5/LCD 					AF15:SYS
//	@Description	:
//	���Ÿ��ù�������
//
//	@Author		:	alpha
//	@Date		:	2020.11.6
*********************************************************************/	
void SYS_GPIO_AF_Set(GPIO_TypeDef *GPIOx,uint32_t Pin, uint32_t Alternate)
{
	//GPIOx->AFR[Pin >> 3] &= ~(0x0f<<(Pin&0x07)*4);//���ԭ��������
	//GPIOx->AFR[Pin >> 3] |= (uint32_t)Alternate<<((Pin&0x07)*4);//����
	uint32_t position = 0x00;//0~15
	uint32_t iocurrent = 0x00;
	
	while((Pin >> position) != 0x00)
	{
		iocurrent = Pin & (1 << position);//�ҵ�Ҫ���õ�����
		if(iocurrent != 0x00)//��Ҫ����
		{
			GPIOx->AFR[position >> 3] &= ~(0xF << ((position & 0x07) * 4));
			GPIOx->AFR[position >> 3] |= (uint32_t)Alternate << ((position & 0x07) * 4);
		}
		position++;

	}

	
	
}


/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	��ΧGPIOA~GPIOK.
//		2.uint32_t Pin			���ű��0x0000~0xffff
//		3.uint32_t Mode  		0~3;ģʽѡ��,0,����(ϵͳ��λĬ��״̬);1,��ͨ���;2,���ù���;3,ģ������.
//		4.uint32_t OTYPER 		0/1;�������ѡ��,0,�������;1,��©���.
//		5.uint32_t OSPEED		0~3;����ٶ�����,0,����;1,����;2,����;3,����. 
//		6.uint32_t PUPDR		0~3:����������,0,����������;1,����;2,����;3,����.
//	@Description	:
//	GPIOͨ������ 
//	������ģʽ(��ͨ����/ģ������)��,OTYPE��OSPEED������Ч!!
//	@Author		:	alpha
//	@Date		:	2020.11.5
*********************************************************************/	
void SYS_GPIO_Init(GPIO_TypeDef *GPIOx,uint32_t Pin, uint32_t Mode, uint32_t OTYPER, uint32_t OSPEED, uint32_t PUPDR)
{
	uint32_t position = 0x00;//0~15
	uint32_t iocurrent = 0x00;
	
	while((Pin >> position) != 0x00)
	{
		iocurrent = Pin & (1 << position);//�ҵ�Ҫ���õ�����
		if(iocurrent != 0x00)//��Ҫ����
		{
			GPIOx->MODER &= ~(3<< (position*2));//���������ԭ�е�����
			GPIOx->MODER |= Mode << (position*2);//д��ģʽ
			if((Mode == 0x01)||(Mode == 0x02))//���ģʽ�����/����
			{
				GPIOx->OSPEEDR&=~(3<<(position*2));	//���ԭ��������
				GPIOx->OSPEEDR|=(OSPEED<<(position*2));//�����µ��ٶ�ֵ  
				GPIOx->OTYPER&=~(1<<position) ;		//���ԭ��������
				GPIOx->OTYPER|=OTYPER<<position;		//�����µ����ģʽ
			}
			GPIOx->PUPDR&=~(3<<(position*2));	//�����ԭ��������
			GPIOx->PUPDR|=PUPDR<<(position*2);	//�����µ�������
		}
		position++;
	}	
}

/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	��ΧGPIOA~GPIOK
//		2.uint32_t Pin			���ű��0x0000~0xffff
//		3.uint8_t PinState		���Ž�Ҫ���óɵ�״̬	��Χ0 or 1
//		4.
//		5.
//	@Description	:
//	����GPIOĳ�����ŵ����״̬
//
//	@Author		:	alpha
//	@Date		:	2020.11.5
*********************************************************************/	
void SYS_GPIO_SetPin(GPIO_TypeDef *GPIOx, uint32_t Pin, uint8_t PinState)
{
	if(PinState != 0)
	{
		GPIOx->BSRR = Pin;//��ʮ��λ����ODRx
	}
	else{
		GPIOx->BSRR = (uint32_t)(Pin << 16);//��ʮ��λ����ODRx
	}
}

/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	��ΧGPIOA~GPIOK
//		2.uint32_t Pin			���ű��0x0000~0xffff
//		3.
//		4.
//		5.
//	@Description	:
//	��ȡGPIO����״̬ 0 ������ 1������
//
//	@Author		:	alpha
//	@Date		:	2020.11.5
*********************************************************************/	
uint8_t SYS_GPIO_GetPin(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
	if((GPIOx->IDR)&Pin)return 1;
	else
		return 0;
}

/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	��ΧGPIOA~GPIOK
//		2.uint32_t Pin			���ű��0x0000~0xffff
//		3.
//		4.
//		5.
//	@Description	:
//	����״̬��ת
//
//	@Author		:	alpha
//	@Date		:	2020.11.5
*********************************************************************/	
void SYS_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
	if((GPIOx->ODR&Pin) == Pin)
		GPIOx->BSRR = (uint32_t)(Pin << 16);
	else
		GPIOx->BSRR = Pin;
}


//ʹ��STM32H7��L1-Cache,ͬʱ����D cache��ǿ��͸д
void Cache_Enable(void)
{
    SCB_EnableICache();	//ʹ��I-Cache,������core_cm7.h���涨��
    SCB_EnableDCache();	//ʹ��D-Cache,������core_cm7.h���涨�� 
	SCB->CACR|=1<<2;	//ǿ��D-Cache͸д,�粻����͸д,ʵ��ʹ���п���������������
}


//ʱ�����ú���
//Fvco=Fs*(plln/pllm);
//Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fq=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCOƵ��
//Fsys:ϵͳʱ��Ƶ��,Ҳ��PLL1��p��Ƶ���ʱ��Ƶ��
//Fq:PLL1��q��Ƶ���ʱ��Ƶ��
//Fs:PLL����ʱ��Ƶ��,������HSI,CSI,HSE��. 

//plln:PLL1��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:4~512.
//pllm:PLL1Ԥ��Ƶϵ��(��PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
//pllp:PLL1��p��Ƶϵ��(PLL֮��ķ�Ƶ),��Ƶ����Ϊϵͳʱ��,ȡֵ��Χ:2~128.(�ұ�����2�ı���)
//pllq:PLL1��q��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:1~128.

//CPUƵ��(rcc_c_ck)=sys_d1cpre_ck=400Mhz 
//rcc_aclk=rcc_hclk3=200Mhz
//AHB1/2/3/4(rcc_hclk1/2/3/4)=200Mhz  
//APB1/2/3/4(rcc_pclk1/2/3/4)=100Mhz  
//pll2_p_ck=(25/25)*440/2)=220Mhz
//pll2_r_ck=FMCʱ��Ƶ��=((25/25)*440/2)=110Mhz

//�ⲿ����Ϊ25M��ʱ��,�Ƽ�ֵ:plln=160,pllm=5,pllp=2,pllq=4.
//�õ�:Fvco=25*(160/5)=800Mhz
//     Fsys=pll1_p_ck=800/2=400Mhz
//     Fq=pll1_q_ck=800/4=200Mhz
//����ֵ:0,�ɹ�;1,ʧ�ܡ�
uint8_t Sys_Clock_Set(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq)
{ 
	uint16_t retry=0;
	uint8_t status=0;
	
	PWR->CR3&=~(1<<2);				//SCUEN=0,����LDOEN��BYPASSλ������
	PWR->D3CR|=3<<14;				//VOS=3,Scale1,1.15~1.26V�ں˵�ѹ,FLASH���ʿ��Եõ��������
	while((PWR->D3CR&(1<<13))==0);	//�ȴ���ѹ�ȶ� 
	RCC->CR|=1<<16;					//HSEON=1,����HSE
	while(((RCC->CR&(1<<17))==0)&&(retry<0X7FFF))retry++;//�ȴ�HSE RDY
	if(retry==0X7FFF)status=1;		//HSE�޷�����
	else   
	{
		RCC->PLLCKSELR|=2<<0;		//PLLSRC[1:0]=2,ѡ��HSE��ΪPLL������ʱ��Դ
		RCC->PLLCKSELR|=pllm<<4;	//DIVM1[5:0]=pllm,����PLL1��Ԥ��Ƶϵ��
		RCC->PLL1DIVR|=(plln-1)<<0;	//DIVN1[8:0]=plln-1,����PLL1�ı�Ƶϵ��,����ֵ���1
		RCC->PLL1DIVR|=(pllp-1)<<9;	//DIVP1[6:0]=pllp-1,����PLL1��p��Ƶϵ��,����ֵ���1
		RCC->PLL1DIVR|=(pllq-1)<<16;//DIVQ1[6:0]=pllq-1,����PLL1��q��Ƶϵ��,����ֵ���1
		RCC->PLL1DIVR|=1<<24;		//DIVR1[6:0]=pllr-1,����PLL1��r��Ƶϵ��,����ֵ���1,r��Ƶ������ʱ��û�õ�
		RCC->PLLCFGR|=2<<2;			//PLL1RGE[1:0]=2,PLL1����ʱ��Ƶ����4~8Mhz֮��(25/5=5Mhz),���޸�pllm,��ȷ�ϴ˲���
		RCC->PLLCFGR|=0<<1;			//PLL1VCOSEL=0,PLL1���VCO��Χ,192~836Mhz
		RCC->PLLCFGR|=3<<16;		//DIVP1EN=1,DIVQ1EN=1,ʹ��pll1_p_ck��pll1_q_ck
		RCC->CR|=1<<24;				//PLL1ON=1,ʹ��PLL1
		while((RCC->CR&(1<<25))==0);//PLL1RDY=1?,�ȴ�PLL1׼����  
	
		//����PLL2��R��Ƶ���,Ϊ220Mhz,������SDRAMʱ��,�ɵõ�110M��SDRAMʱ��Ƶ��
		RCC->PLLCKSELR|=25<<12;		//DIVM2[5:0]=25,����PLL2��Ԥ��Ƶϵ��
		RCC->PLL2DIVR|=(440-1)<<0;	//DIVN2[8:0]=440-1,����PLL2�ı�Ƶϵ��,����ֵ���1
		RCC->PLL2DIVR|=(2-1)<<9;	//DIVP2[6:0]=2-1,����PLL2��p��Ƶϵ��,����ֵ���1
		RCC->PLL2DIVR|=(2-1)<<24;	//DIVR2[6:0]=2-1,����PLL2��r��Ƶϵ��,����ֵ���1
		RCC->PLLCFGR|=0<<6;			//PLL2RGE[1:0]=0,PLL2����ʱ��Ƶ����1~2Mhz֮��(25/25=1Mhz)
		RCC->PLLCFGR|=0<<5;			//PLL2VCOSEL=0,PLL2���VCO��Χ,192~836Mhz
		RCC->PLLCFGR|=1<<19;		//DIVP2EN=1,ʹ��pll2_p_ck
		RCC->PLLCFGR|=1<<21;		//DIVR2EN=1,ʹ��pll2_r_ck
		RCC->D1CCIPR&=~(3<<0);		//���FMCSEL[1:0]ԭ��������
		RCC->D1CCIPR|=2<<0;			//FMCSEL[1:0]=2,FMCʱ��������pll2_r_ck		
		RCC->CR|=1<<26;				//PLL2ON=1,ʹ��PLL2
		while((RCC->CR&(1<<27))==0);//PLL2RDY=1?,�ȴ�PLL2׼����  
	
		RCC->D1CFGR|=8<<0;			//HREF[3:0]=8,rcc_hclk1/2/3/4=sys_d1cpre_ck/2=400/2=200Mhz,��AHB1/2/3/4=200Mhz
		RCC->D1CFGR|=0<<8;			//D1CPRE[2:0]=0,sys_d1cpre_ck=sys_clk/1=400/1=400Mhz,��CPUʱ��=400Mhz
		RCC->CFGR|=3<<0;			//SW[2:0]=3,ϵͳʱ��(sys_clk)ѡ������pll1_p_ck,��400Mhz
		while(1)
		{
			retry=(RCC->CFGR&(7<<3))>>3;	//��ȡSWS[2:0]��״̬,�ж��Ƿ��л��ɹ�
			if(retry==3)break;		//�ɹ���ϵͳʱ��Դ�л�Ϊpll1_p_ck
		}
			
		FLASH->ACR|=2<<0;			//LATENCY[2:0]=2,2��CPU�ȴ�����(@VOS1 Level,maxclock=210Mhz)
		FLASH->ACR|=2<<4;			//WRHIGHFREQ[1:0]=2,flash����Ƶ��<285Mhz
		
		RCC->D1CFGR|=4<<4;			//D1PPRE[2:0]=4,rcc_pclk3=rcc_hclk3/2=100Mhz,��APB3=100Mhz
		RCC->D2CFGR|=4<<4;			//D2PPRE1[2:0]=4,rcc_pclk1=rcc_hclk1/2=100Mhz,��APB1=100Mhz
		RCC->D2CFGR|=4<<8;			//D2PPRE2[2:0]=4,rcc_pclk2=rcc_hclk1/2=100Mhz,��APB2=100Mhz
		RCC->D3CFGR|=4<<4;			//D3PPRE[2:0]=4,rcc_pclk4=rcc_hclk4/2=100Mhz,��APB4=100Mhz
 
		RCC->CR|=1<<7;				//CSION=1,ʹ��CSI,ΪIO������Ԫ�ṩʱ��
		RCC->APB4ENR|=1<<1;			//SYSCFGEN=1,ʹ��SYSCFGʱ��
		SYSCFG->CCCSR|=1<<0;		//EN=1,ʹ��IO������Ԫ 
	} 
	return status;
}  
 
//ϵͳʱ�ӳ�ʼ������
//plln:PLL1��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:4~512.
//pllm:PLL1Ԥ��Ƶϵ��(��PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
//pllp:PLL1��p��Ƶϵ��(PLL֮��ķ�Ƶ),��Ƶ����Ϊϵͳʱ��,ȡֵ��Χ:2~128.(�ұ�����2�ı���)
//pllq:PLL1��q��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2~128.
void Stm32_Clock_Init(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq)
{  
	
	RCC->CR=0x00000001;				//����HISON,�����ڲ�����RC�񵴣�����λȫ����
	RCC->CFGR=0x00000000;			//CFGR���� 
	RCC->D1CFGR=0x00000000;			//D1CFGR���� 
	RCC->D2CFGR=0x00000000;			//D2CFGR���� 
	RCC->D3CFGR=0x00000000;			//D3CFGR���� 
	RCC->PLLCKSELR=0x00000000;		//PLLCKSELR���� 
	RCC->PLLCFGR=0x00000000;		//PLLCFGR���� 
	RCC->CIER=0x00000000;			//CIER����,��ֹ����RCC����ж�  
	//AXI_TARG7_FN_MOD�Ĵ���,����û����stm32h750xx.h���涨��,����,ֻ����ֱ��
	//������ַ�ķ�ʽ,���޸�,�üĴ�����<<STM32H750�ο��ֲ�>>��113ҳ,AXI_TARGx_FN_MOD
	*((volatile uint32_t*)0x51008108)=0x00000001;//����AXI SRAM�ľ����ȡ����Ϊ1 
	Sys_Clock_Set(plln,pllm,pllp,pllq);//����ʱ��  
	//SYS_QSPI_Enable_Memmapmode();		//ʹ��QSPI�ڴ�ӳ��ģʽ
	Cache_Enable();					//ʹ��L1 Cache
	//����������				  
#ifdef  VECT_TAB_RAM
	SYS_NVIC_SetVectorTable(D1_AXISRAM_BASE,0x0);
#else   
	SYS_NVIC_SetVectorTable(FLASH_BANK1_BASE,0x0);
#endif 
}

void SYS_QSPI_Enable_Memmapmode(void)
{
	uint32_t tempreg=0;
	volatile uint32_t *data_reg=&QUADSPI->DR;
	
	RCC->AHB4ENR |= 1<<1;	//��GPIOBʱ��
	RCC->AHB4ENR |= 1<<5;	//��GPIOFʱ��
	RCC->AHB3ENR |= 1<14;	//��QSPIʱ��
	
	SYS_GPIO_Init(GPIOB,GPIO_PIN_2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);//PB2���ù������
	SYS_GPIO_Init(GPIOB,GPIO_PIN_6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);//PB6���ù������
	SYS_GPIO_Init(GPIOF,GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);//PF6~9���ù������	
	
	SYS_GPIO_AF_Set(GPIOB,GPIO_PIN_2,9);//AF9
	SYS_GPIO_AF_Set(GPIOB,GPIO_PIN_6,10);//AF10
	
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_6,9);//AF9
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_7,9);//AF9
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_8,10);//AF10
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_9,10);//AF10
	
	RCC->AHB3RSTR|=1<<14;			//��λQSPI
	RCC->AHB3RSTR&=~(1<<14);		//ֹͣ��λQSPI
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ����
	
	//QSPIʱ��Ĭ������rcc_hclk3(��RCC_D1CCIPR��QSPISEL[1:0]ѡ��)
	tempreg=(2-1)<<24;		//����QSPIʱ��ΪAHBʱ�ӵ�1/2,��200M/2=100Mhz,10ns
	tempreg|=(4-1)<<8;		//����FIFO��ֵΪ4���ֽ�(���Ϊ31,��ʾ32���ֽ�)
	tempreg|=0<<7;			//ѡ��FLASH1
	tempreg|=0<<6;			//��ֹ˫����ģʽ
	tempreg|=1<<4;			//������λ�������(DDRģʽ��,��������Ϊ0)
	QUADSPI->CR=tempreg;	//����CR�Ĵ���
	tempreg=(23-1)<<16;		//����FLASH��СΪ2^23=8MB
	tempreg|=(5-1)<<8;		//Ƭѡ�ߵ�ƽʱ��Ϊ5��ʱ��(10*5=50ns),���ֲ������tSHSL����
	tempreg|=1<<0;			//Mode3,����ʱCLKΪ�ߵ�ƽ
	QUADSPI->DCR=tempreg;	//����DCR�Ĵ���
	QUADSPI->CR|=1<<0;		//ʹ��QSPI
	
	//W25Q64дʹ��
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->CCR=QUADSPI->CCR=0X00000106;		//����0X06ָ�W25QXXдʹ��
	while((QUADSPI->SR&(1<<1))==0);	//�ȴ�ָ������
	QUADSPI->FCR|=1<<1;				//���������ɱ�־λ 
	
	//ʹ��QEλ����QSPI
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->CCR=QUADSPI->CCR=0X00000131;		//����0X31ָ�W25QXXдʹ��
	QUADSPI->DLR=0;					//����һ���ֽ�
	while((QUADSPI->SR&(1<<2))==0);	//�ȴ�FTF
	*(volatile uint8_t*)data_reg = 1 << 1;//QEλ��1
	while((QUADSPI->SR&(1<<1))==0);	//�ȴ�ָ������
	QUADSPI->FCR|=1<<1;				//���������ɱ�־λ 
	
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->ABR=0;					//�����ֽ�����Ϊ0��ʵ���Ͼ���W25Q 0XEBָ���,M0~M7=0
	tempreg=0XEB;					//INSTRUCTION[7:0]=0XEB,����0XEBָ�Fast Read QUAD I/O��
	tempreg|=3<<8;					//IMODE[1:0]=3,���ߴ���ָ��
	tempreg|=3<<10;					//ADDRESS[1:0]=3,���ߴ����ַ
	tempreg|=2<<12;					//ADSIZE[1:0]=2,24λ��ַ����
	tempreg|=3<<14;					//ABMODE[1:0]=3,���ߴ��佻���ֽ�
	tempreg|=0<<16;					//ABSIZE[1:0]=0,8λ�����ֽ�(M0~M7)
	tempreg|=6<<18;					//DCYC[4:0]=6,6��dummy����
	tempreg|=3<<24;					//DMODE[1:0]=3,���ߴ�������
	tempreg|=3<<26;					//FMODE[1:0]=3,�ڴ�ӳ��ģʽ
	QUADSPI->CCR=tempreg;			//����CCR�Ĵ���
	
	//����QSPI FLASH�ռ��MPU����
	SCB->SHCSR&=~(1<<16);			//��ֹMemManage 
	MPU->CTRL&=~(1<<0);				//��ֹMPU
	MPU->RNR=0;						//���ñ���������Ϊ0(1~7���Ը������ڴ���)
	MPU->RBAR=0X90000000;			//����ַΪ0X9000 000,��QSPI����ʼ��ַ
	MPU->RASR=0X0303002D;			//������ر�������(��ֹ����,����cache,������),���MPUʵ��Ľ���
	MPU->CTRL=(1<<2)|(1<<0);		//ʹ��PRIVDEFENA,ʹ��MPU 
	SCB->SHCSR|=1<<16;				//ʹ��MemManage
}












