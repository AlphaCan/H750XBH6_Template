#include "sys.h"

/*********************************************************************
//	@param		:
//		1.NVIC_VecTab 基地址
//		2.Offset 偏移量
//		3.
//		4.
//		5.
//	@Description	:
//	设置中断向量偏移地址
//	设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留。
//	@Author		:	alpha
//	@Date		:	2020.11.4
*********************************************************************/	

void SYS_NVIC_SetVectorTable(uint32_t NVIC_VecTab,uint32_t Offset)
{
	SCB->VTOR = NVIC_VecTab|(Offset&(uint32_t)0xFFFFFE00);
}

/*********************************************************************
//	@param		:
//		1.PriorityGroup 优先级分组，只分配3...7，见sys.h宏定义
//		2.
//		3.
//		4.
//		5.
//	@Description	:
//	设置NVIC分组
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
//		1.IRQ_Number 中断号0~149
//		2.PreemptPriority 抢占优先级
//		3.SubPriority 响应优先级
//		4.
//		5.
//	@Description	:
//	设置中断优先级
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
//		1.IRQ_Number 中断号0~149
//		2.
//		3.
//		4.
//		5.
//	@Description	:
//	使能中断
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
//		1.IRQ_Number 中断号0~149
//		2.
//		3.
//		4.
//		5.
//	@Description	:
//	失能中断
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
//		1.GPIO_TypeDef *GPIOx	范围GPIOA~GPIOK.
//		2.uint32_t Pin			引脚编号0~15
//		3.uint32_t Alternate	复用功能编号
//		AF0:SYS		AF1：TIM1/2/16/17/LPTIM1/HRTIM1		AF2:SAI1/TIM3/4/5/12/HRTIM1		AF3:LPUART/TIM8/LPTIM2/3/4/5/HRTIM1/DFSDM1
//		AF4:I2C1/2/3/4/USART1/TIM15/LPTIM2/DFSDM1/CEC	AF5:SPI1/2/3/4/5/6/CEC			AF6:SPI2/3/SAI1/3/I2C4/UART4/DFSDM1
//		AF7:SPI2/3/6/USART1/2/3/6/UART7/SDMMC1			AF8:SPI6/SAI2/4/UART4/5/8/LPUART/SDMMC1/SPDIFRX1
//		AF9:SAI4/FDCAN1/2/QUADSPI/FMC/SDMMC2/LCD/SPDIFRX1								AF10:SAI2/4/TIM8/QUADSPI/SDMMC2/OTG1_HS/OTG2_FS/LCD
//		AF11:I2C4/UART7/SWPMI1/DFSDM1/SDMMC2/MDIOS/ETH									AF12:TIM1/8/FMC/SDMMC1/MDIOS/OTG1_FS/LCD
//		AF13:TIM1/DCMI/LCD/COMP							AF14:UART5/LCD 					AF15:SYS
//	@Description	:
//	引脚复用功能设置
//
//	@Author		:	alpha
//	@Date		:	2020.11.6
*********************************************************************/	
void SYS_GPIO_AF_Set(GPIO_TypeDef *GPIOx,uint32_t Pin, uint32_t Alternate)
{
	//GPIOx->AFR[Pin >> 3] &= ~(0x0f<<(Pin&0x07)*4);//清空原来的数据
	//GPIOx->AFR[Pin >> 3] |= (uint32_t)Alternate<<((Pin&0x07)*4);//设置
	uint32_t position = 0x00;//0~15
	uint32_t iocurrent = 0x00;
	
	while((Pin >> position) != 0x00)
	{
		iocurrent = Pin & (1 << position);//找到要设置的引脚
		if(iocurrent != 0x00)//需要设置
		{
			GPIOx->AFR[position >> 3] &= ~(0xF << ((position & 0x07) * 4));
			GPIOx->AFR[position >> 3] |= (uint32_t)Alternate << ((position & 0x07) * 4);
		}
		position++;

	}

	
	
}


/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	范围GPIOA~GPIOK.
//		2.uint32_t Pin			引脚编号0x0000~0xffff
//		3.uint32_t Mode  		0~3;模式选择,0,输入(系统复位默认状态);1,普通输出;2,复用功能;3,模拟输入.
//		4.uint32_t OTYPER 		0/1;输出类型选择,0,推挽输出;1,开漏输出.
//		5.uint32_t OSPEED		0~3;输出速度设置,0,低速;1,中速;2,快速;3,高速. 
//		6.uint32_t PUPDR		0~3:上下拉设置,0,不带上下拉;1,上拉;2,下拉;3,保留.
//	@Description	:
//	GPIO通用设置 
//	在输入模式(普通输入/模拟输入)下,OTYPE和OSPEED参数无效!!
//	@Author		:	alpha
//	@Date		:	2020.11.5
*********************************************************************/	
void SYS_GPIO_Init(GPIO_TypeDef *GPIOx,uint32_t Pin, uint32_t Mode, uint32_t OTYPER, uint32_t OSPEED, uint32_t PUPDR)
{
	uint32_t position = 0x00;//0~15
	uint32_t iocurrent = 0x00;
	
	while((Pin >> position) != 0x00)
	{
		iocurrent = Pin & (1 << position);//找到要设置的引脚
		if(iocurrent != 0x00)//需要设置
		{
			GPIOx->MODER &= ~(3<< (position*2));//清除该引脚原有的设置
			GPIOx->MODER |= Mode << (position*2);//写入模式
			if((Mode == 0x01)||(Mode == 0x02))//如果模式是输出/复用
			{
				GPIOx->OSPEEDR&=~(3<<(position*2));	//清除原来的设置
				GPIOx->OSPEEDR|=(OSPEED<<(position*2));//设置新的速度值  
				GPIOx->OTYPER&=~(1<<position) ;		//清除原来的设置
				GPIOx->OTYPER|=OTYPER<<position;		//设置新的输出模式
			}
			GPIOx->PUPDR&=~(3<<(position*2));	//先清除原来的设置
			GPIOx->PUPDR|=PUPDR<<(position*2);	//设置新的上下拉
		}
		position++;
	}	
}

/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	范围GPIOA~GPIOK
//		2.uint32_t Pin			引脚编号0x0000~0xffff
//		3.uint8_t PinState		引脚将要设置成的状态	范围0 or 1
//		4.
//		5.
//	@Description	:
//	设置GPIO某个引脚的输出状态
//
//	@Author		:	alpha
//	@Date		:	2020.11.5
*********************************************************************/	
void SYS_GPIO_SetPin(GPIO_TypeDef *GPIOx, uint32_t Pin, uint8_t PinState)
{
	if(PinState != 0)
	{
		GPIOx->BSRR = Pin;//低十六位设置ODRx
	}
	else{
		GPIOx->BSRR = (uint32_t)(Pin << 16);//高十六位重置ODRx
	}
}

/*********************************************************************
//	@param		:
//		1.GPIO_TypeDef *GPIOx	范围GPIOA~GPIOK
//		2.uint32_t Pin			引脚编号0x0000~0xffff
//		3.
//		4.
//		5.
//	@Description	:
//	读取GPIO引脚状态 0 无输入 1有输入
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
//		1.GPIO_TypeDef *GPIOx	范围GPIOA~GPIOK
//		2.uint32_t Pin			引脚编号0x0000~0xffff
//		3.
//		4.
//		5.
//	@Description	:
//	引脚状态翻转
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


//使能STM32H7的L1-Cache,同时开启D cache的强制透写
void Cache_Enable(void)
{
    SCB_EnableICache();	//使能I-Cache,函数在core_cm7.h里面定义
    SCB_EnableDCache();	//使能D-Cache,函数在core_cm7.h里面定义 
	SCB->CACR|=1<<2;	//强制D-Cache透写,如不开启透写,实际使用中可能遇到各种问题
}


//时钟设置函数
//Fvco=Fs*(plln/pllm);
//Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fq=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCO频率
//Fsys:系统时钟频率,也是PLL1的p分频输出时钟频率
//Fq:PLL1的q分频输出时钟频率
//Fs:PLL输入时钟频率,可以是HSI,CSI,HSE等. 

//plln:PLL1倍频系数(PLL倍频),取值范围:4~512.
//pllm:PLL1预分频系数(进PLL之前的分频),取值范围:2~63.
//pllp:PLL1的p分频系数(PLL之后的分频),分频后作为系统时钟,取值范围:2~128.(且必须是2的倍数)
//pllq:PLL1的q分频系数(PLL之后的分频),取值范围:1~128.

//CPU频率(rcc_c_ck)=sys_d1cpre_ck=400Mhz 
//rcc_aclk=rcc_hclk3=200Mhz
//AHB1/2/3/4(rcc_hclk1/2/3/4)=200Mhz  
//APB1/2/3/4(rcc_pclk1/2/3/4)=100Mhz  
//pll2_p_ck=(25/25)*440/2)=220Mhz
//pll2_r_ck=FMC时钟频率=((25/25)*440/2)=110Mhz

//外部晶振为25M的时候,推荐值:plln=160,pllm=5,pllp=2,pllq=4.
//得到:Fvco=25*(160/5)=800Mhz
//     Fsys=pll1_p_ck=800/2=400Mhz
//     Fq=pll1_q_ck=800/4=200Mhz
//返回值:0,成功;1,失败。
uint8_t Sys_Clock_Set(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq)
{ 
	uint16_t retry=0;
	uint8_t status=0;
	
	PWR->CR3&=~(1<<2);				//SCUEN=0,锁定LDOEN和BYPASS位的设置
	PWR->D3CR|=3<<14;				//VOS=3,Scale1,1.15~1.26V内核电压,FLASH访问可以得到最高性能
	while((PWR->D3CR&(1<<13))==0);	//等待电压稳定 
	RCC->CR|=1<<16;					//HSEON=1,开启HSE
	while(((RCC->CR&(1<<17))==0)&&(retry<0X7FFF))retry++;//等待HSE RDY
	if(retry==0X7FFF)status=1;		//HSE无法就绪
	else   
	{
		RCC->PLLCKSELR|=2<<0;		//PLLSRC[1:0]=2,选择HSE作为PLL的输入时钟源
		RCC->PLLCKSELR|=pllm<<4;	//DIVM1[5:0]=pllm,设置PLL1的预分频系数
		RCC->PLL1DIVR|=(plln-1)<<0;	//DIVN1[8:0]=plln-1,设置PLL1的倍频系数,设置值需减1
		RCC->PLL1DIVR|=(pllp-1)<<9;	//DIVP1[6:0]=pllp-1,设置PLL1的p分频系数,设置值需减1
		RCC->PLL1DIVR|=(pllq-1)<<16;//DIVQ1[6:0]=pllq-1,设置PLL1的q分频系数,设置值需减1
		RCC->PLL1DIVR|=1<<24;		//DIVR1[6:0]=pllr-1,设置PLL1的r分频系数,设置值需减1,r分频出来的时钟没用到
		RCC->PLLCFGR|=2<<2;			//PLL1RGE[1:0]=2,PLL1输入时钟频率在4~8Mhz之间(25/5=5Mhz),如修改pllm,请确认此参数
		RCC->PLLCFGR|=0<<1;			//PLL1VCOSEL=0,PLL1宽的VCO范围,192~836Mhz
		RCC->PLLCFGR|=3<<16;		//DIVP1EN=1,DIVQ1EN=1,使能pll1_p_ck和pll1_q_ck
		RCC->CR|=1<<24;				//PLL1ON=1,使能PLL1
		while((RCC->CR&(1<<25))==0);//PLL1RDY=1?,等待PLL1准备好  
	
		//设置PLL2的R分频输出,为220Mhz,后续做SDRAM时钟,可得到110M的SDRAM时钟频率
		RCC->PLLCKSELR|=25<<12;		//DIVM2[5:0]=25,设置PLL2的预分频系数
		RCC->PLL2DIVR|=(440-1)<<0;	//DIVN2[8:0]=440-1,设置PLL2的倍频系数,设置值需减1
		RCC->PLL2DIVR|=(2-1)<<9;	//DIVP2[6:0]=2-1,设置PLL2的p分频系数,设置值需减1
		RCC->PLL2DIVR|=(2-1)<<24;	//DIVR2[6:0]=2-1,设置PLL2的r分频系数,设置值需减1
		RCC->PLLCFGR|=0<<6;			//PLL2RGE[1:0]=0,PLL2输入时钟频率在1~2Mhz之间(25/25=1Mhz)
		RCC->PLLCFGR|=0<<5;			//PLL2VCOSEL=0,PLL2宽的VCO范围,192~836Mhz
		RCC->PLLCFGR|=1<<19;		//DIVP2EN=1,使能pll2_p_ck
		RCC->PLLCFGR|=1<<21;		//DIVR2EN=1,使能pll2_r_ck
		RCC->D1CCIPR&=~(3<<0);		//清除FMCSEL[1:0]原来的设置
		RCC->D1CCIPR|=2<<0;			//FMCSEL[1:0]=2,FMC时钟来自于pll2_r_ck		
		RCC->CR|=1<<26;				//PLL2ON=1,使能PLL2
		while((RCC->CR&(1<<27))==0);//PLL2RDY=1?,等待PLL2准备好  
	
		RCC->D1CFGR|=8<<0;			//HREF[3:0]=8,rcc_hclk1/2/3/4=sys_d1cpre_ck/2=400/2=200Mhz,即AHB1/2/3/4=200Mhz
		RCC->D1CFGR|=0<<8;			//D1CPRE[2:0]=0,sys_d1cpre_ck=sys_clk/1=400/1=400Mhz,即CPU时钟=400Mhz
		RCC->CFGR|=3<<0;			//SW[2:0]=3,系统时钟(sys_clk)选择来自pll1_p_ck,即400Mhz
		while(1)
		{
			retry=(RCC->CFGR&(7<<3))>>3;	//获取SWS[2:0]的状态,判断是否切换成功
			if(retry==3)break;		//成功将系统时钟源切换为pll1_p_ck
		}
			
		FLASH->ACR|=2<<0;			//LATENCY[2:0]=2,2个CPU等待周期(@VOS1 Level,maxclock=210Mhz)
		FLASH->ACR|=2<<4;			//WRHIGHFREQ[1:0]=2,flash访问频率<285Mhz
		
		RCC->D1CFGR|=4<<4;			//D1PPRE[2:0]=4,rcc_pclk3=rcc_hclk3/2=100Mhz,即APB3=100Mhz
		RCC->D2CFGR|=4<<4;			//D2PPRE1[2:0]=4,rcc_pclk1=rcc_hclk1/2=100Mhz,即APB1=100Mhz
		RCC->D2CFGR|=4<<8;			//D2PPRE2[2:0]=4,rcc_pclk2=rcc_hclk1/2=100Mhz,即APB2=100Mhz
		RCC->D3CFGR|=4<<4;			//D3PPRE[2:0]=4,rcc_pclk4=rcc_hclk4/2=100Mhz,即APB4=100Mhz
 
		RCC->CR|=1<<7;				//CSION=1,使能CSI,为IO补偿单元提供时钟
		RCC->APB4ENR|=1<<1;			//SYSCFGEN=1,使能SYSCFG时钟
		SYSCFG->CCCSR|=1<<0;		//EN=1,使能IO补偿单元 
	} 
	return status;
}  
 
//系统时钟初始化函数
//plln:PLL1倍频系数(PLL倍频),取值范围:4~512.
//pllm:PLL1预分频系数(进PLL之前的分频),取值范围:2~63.
//pllp:PLL1的p分频系数(PLL之后的分频),分频后作为系统时钟,取值范围:2~128.(且必须是2的倍数)
//pllq:PLL1的q分频系数(PLL之后的分频),取值范围:2~128.
void Stm32_Clock_Init(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq)
{  
	
	RCC->CR=0x00000001;				//设置HISON,开启内部高速RC振荡，其他位全清零
	RCC->CFGR=0x00000000;			//CFGR清零 
	RCC->D1CFGR=0x00000000;			//D1CFGR清零 
	RCC->D2CFGR=0x00000000;			//D2CFGR清零 
	RCC->D3CFGR=0x00000000;			//D3CFGR清零 
	RCC->PLLCKSELR=0x00000000;		//PLLCKSELR清零 
	RCC->PLLCFGR=0x00000000;		//PLLCFGR清零 
	RCC->CIER=0x00000000;			//CIER清零,禁止所有RCC相关中断  
	//AXI_TARG7_FN_MOD寄存器,由于没有在stm32h750xx.h里面定义,所以,只能用直接
	//操作地址的方式,来修改,该寄存器在<<STM32H750参考手册>>第113页,AXI_TARGx_FN_MOD
	*((volatile uint32_t*)0x51008108)=0x00000001;//设置AXI SRAM的矩阵读取能力为1 
	Sys_Clock_Set(plln,pllm,pllp,pllq);//设置时钟  
	//SYS_QSPI_Enable_Memmapmode();		//使能QSPI内存映射模式
	Cache_Enable();					//使能L1 Cache
	//配置向量表				  
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
	
	RCC->AHB4ENR |= 1<<1;	//打开GPIOB时钟
	RCC->AHB4ENR |= 1<<5;	//打开GPIOF时钟
	RCC->AHB3ENR |= 1<14;	//打开QSPI时钟
	
	SYS_GPIO_Init(GPIOB,GPIO_PIN_2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);//PB2复用功能输出
	SYS_GPIO_Init(GPIOB,GPIO_PIN_6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);//PB6复用功能输出
	SYS_GPIO_Init(GPIOF,GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);//PF6~9复用功能输出	
	
	SYS_GPIO_AF_Set(GPIOB,GPIO_PIN_2,9);//AF9
	SYS_GPIO_AF_Set(GPIOB,GPIO_PIN_6,10);//AF10
	
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_6,9);//AF9
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_7,9);//AF9
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_8,10);//AF10
	SYS_GPIO_AF_Set(GPIOF,GPIO_PIN_9,10);//AF10
	
	RCC->AHB3RSTR|=1<<14;			//复位QSPI
	RCC->AHB3RSTR&=~(1<<14);		//停止复位QSPI
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零
	
	//QSPI时钟默认来自rcc_hclk3(由RCC_D1CCIPR的QSPISEL[1:0]选择)
	tempreg=(2-1)<<24;		//设置QSPI时钟为AHB时钟的1/2,即200M/2=100Mhz,10ns
	tempreg|=(4-1)<<8;		//设置FIFO阈值为4个字节(最大为31,表示32个字节)
	tempreg|=0<<7;			//选择FLASH1
	tempreg|=0<<6;			//禁止双闪存模式
	tempreg|=1<<4;			//采样移位半个周期(DDR模式下,必须设置为0)
	QUADSPI->CR=tempreg;	//设置CR寄存器
	tempreg=(23-1)<<16;		//设置FLASH大小为2^23=8MB
	tempreg|=(5-1)<<8;		//片选高电平时间为5个时钟(10*5=50ns),即手册里面的tSHSL参数
	tempreg|=1<<0;			//Mode3,空闲时CLK为高电平
	QUADSPI->DCR=tempreg;	//设置DCR寄存器
	QUADSPI->CR|=1<<0;		//使能QSPI
	
	//W25Q64写使能
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=QUADSPI->CCR=0X00000106;		//发送0X06指令，W25QXX写使能
	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 
	
	//使能QE位进入QSPI
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=QUADSPI->CCR=0X00000131;		//发送0X31指令，W25QXX写使能
	QUADSPI->DLR=0;					//发送一个字节
	while((QUADSPI->SR&(1<<2))==0);	//等待FTF
	*(volatile uint8_t*)data_reg = 1 << 1;//QE位置1
	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 
	
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->ABR=0;					//交替字节设置为0，实际上就是W25Q 0XEB指令的,M0~M7=0
	tempreg=0XEB;					//INSTRUCTION[7:0]=0XEB,发送0XEB指令（Fast Read QUAD I/O）
	tempreg|=3<<8;					//IMODE[1:0]=3,四线传输指令
	tempreg|=3<<10;					//ADDRESS[1:0]=3,四线传输地址
	tempreg|=2<<12;					//ADSIZE[1:0]=2,24位地址长度
	tempreg|=3<<14;					//ABMODE[1:0]=3,四线传输交替字节
	tempreg|=0<<16;					//ABSIZE[1:0]=0,8位交替字节(M0~M7)
	tempreg|=6<<18;					//DCYC[4:0]=6,6个dummy周期
	tempreg|=3<<24;					//DMODE[1:0]=3,四线传输数据
	tempreg|=3<<26;					//FMODE[1:0]=3,内存映射模式
	QUADSPI->CCR=tempreg;			//设置CCR寄存器
	
	//设置QSPI FLASH空间的MPU保护
	SCB->SHCSR&=~(1<<16);			//禁止MemManage 
	MPU->CTRL&=~(1<<0);				//禁止MPU
	MPU->RNR=0;						//设置保护区域编号为0(1~7可以给其他内存用)
	MPU->RBAR=0X90000000;			//基地址为0X9000 000,即QSPI的起始地址
	MPU->RASR=0X0303002D;			//设置相关保护参数(禁止共用,允许cache,允许缓冲),详见MPU实验的解析
	MPU->CTRL=(1<<2)|(1<<0);		//使能PRIVDEFENA,使能MPU 
	SCB->SHCSR|=1<<16;				//使能MemManage
}












