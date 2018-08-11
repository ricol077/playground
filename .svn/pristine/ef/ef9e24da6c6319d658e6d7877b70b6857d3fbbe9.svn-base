#include "spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//SPI驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//以下是SPI模块的初始化代码，配置成主机模式，访问SD Card/W25Q64/NRF24L01						  
//SPI口初始化
//这里针是对SPI2的初始化
/*
void SPI2_Init(void)
{	 
	RCC->APB2ENR|=1<<3;  	//PORTB时钟使能 	 
	RCC->APB1ENR|=1<<14;   	//SPI2时钟使能 
	//这里只针对SPI口初始化
	GPIOB->CRH&=0X000FFFFF; 
	GPIOB->CRH|=0XBBB00000;	//PB13/14/15复用 	    
	GPIOB->ODR|=0X7<<13;   	//PB13/14/15上拉
	SPI2->CR1|=0<<10;		//全双工模式	
	SPI2->CR1|=1<<9; 		//软件nss管理
	SPI2->CR1|=1<<8;  

	SPI2->CR1|=1<<2; 		//SPI主机
	SPI2->CR1|=0<<11;		//8bit数据格式	
	SPI2->CR1|=1<<1; 		//空闲模式下SCK为1 CPOL=1
	SPI2->CR1|=1<<0; 		//数据采样从第二个时间边沿开始,CPHA=1  
	//对SPI2属于APB1的外设.时钟频率最大为36M.
	SPI2->CR1|=3<<3; 		//Fsck=Fpclk1/256
	SPI2->CR1|=0<<7; 		//MSBfirst   
	SPI2->CR1|=1<<6; 		//SPI设备使能
	SPI2_ReadWriteByte(0xff);//启动传输		 
}   
*/
#include "delay.h"
#include "spi.h"
#include "user_def.h"

//#define PCRCHip_Sel	PGout(2) // define the real pin as HW use
#define PCRCHip_Sel	PBout(12) 


u8 SPI_2_SendBuf[SPI_BUF_BYTE_LEN]={0};
u8 SPI_2_RcvBuf[SPI_BUF_BYTE_LEN]={0};
u8 SPI_2_CommLen=0;

u8 SPI_2_Rx_sts=0;

void SPI_2_DMA_Initialization(void);


void SPI2_Initializaion(void)
{
	RCC->APB1ENR|=1<<14;   	//SPI2时钟使能 
	RCC->APB2ENR|=1<<3;     //使能PORTB时钟 	    
 	RCC->APB2ENR|=1<<5;		//使能PORTD时钟
 	RCC->APB2ENR|=1<<8;		//使能PORTG时钟
	GPIOB->CRH&=0XFFF00FFF; 
	GPIOB->CRH|=0X00030000;	//PB12 推挽 	    
	GPIOB->ODR|=1<<12;    	//PB12上拉

 	GPIOB->CRH|=0X00003000;	//PB11 推挽 	    
	GPIOB->ODR|=1<<11;    	//PB11上拉	 as PCRChip selection

	//这里pd2和pg7拉高,是为了防止NRF24L01和SD卡影响FLASH的烧写.
	//因为他们共用一个SPI口. 
	GPIOD->CRL&=0XFFFFF0FF; 
	GPIOD->CRL|=0X00000300;	//PD2 推挽 	    
	GPIOD->ODR|=1<<2;    	//PD2上拉

	PCRCHip_Sel=1;
	//这里只针对SPI口初始化
	GPIOB->CRH&=0X000FFFFF; 
	GPIOB->CRH|=0XBBB00000;	//PB13/14/15复用 	    
	GPIOB->ODR|=0X7<<13;   	//PB13/14/15上拉
	SPI2->CR1|=0<<10;		//全双工模式	
	SPI2->CR1|=1<<9; 		//软件nss管理
	SPI2->CR1|=1<<8;  

	SPI2->CR1|=1<<2; 		//SPI主机
	SPI2->CR1|=0<<11;		//8bit数据格式	
	SPI2->CR1|=0<<1; 		//空闲模式下SCK为0 CPOL=0
	SPI2->CR1|=0<<0; 		//数据采样从第二个时间边沿开始,CPHA=0  
	//对SPI2属于APB1的外设.时钟频率最大为36M.
	SPI2->CR1|=4<<3; 		//Fsck=Fpclk1/32, 4<<3;
	SPI2->CR1|=0<<7; 		//MSBfirst   
	SPI2->CR2|=3;
	SPI2->CR1|=1<<6; 		//SPI设备使能

	SPI_2_DMA_Initialization();
		 
}   

#define SPI2_DR_Addr ( (u32)0x4000380C )
void SPI_2_DMA_Initialization(void)
{
	SPI_2_CommLen=0;
	RCC->AHBENR|=1<<0;							//开启DMA1时钟
	delay_ms(5);								//等待DMA时钟稳定
    RCC->AHBENR |= 1<<0 ;                     //DMA1时钟使能

	/*------------------配置SPI1_RX_DMA通道Channel2---------------------*/

    DMA1_Channel4->CCR &= ~( 1<<14 ) ;        //非存储器到存储器模式
	DMA1_Channel4->CCR |=    2<<12   ;        //通道优先级高
	DMA1_Channel4->CCR &= ~( 3<<10 ) ;        //存储器数据宽度8bit
	DMA1_Channel4->CCR &= ~( 3<<8  ) ;        //外设数据宽度8bit
	DMA1_Channel4->CCR |=    1<<7    ;        //存储器地址增量模式
	DMA1_Channel4->CCR &= ~( 1<<6  ) ;        //不执行外设地址增量模式
	DMA1_Channel4->CCR &= ~( 1<<5  ) ;        //执行循环操作
	DMA1_Channel4->CCR &= ~( 1<<4  ) ;        //从外设读

	DMA1_Channel4->CNDTR &= 0x0000   ;        //传输数量寄存器清零
	DMA1_Channel4->CNDTR = SPI_2_CommLen ;       //传输数量设置为buffersize个

	DMA1_Channel4->CPAR = SPI2_DR_Addr ;      //设置外设地址，注意PSIZE
	DMA1_Channel4->CMAR = (u32)SPI_2_RcvBuf ; //设置DMA存储器地址，注意MSIZE

	/*------------------配置SPI1_TX_DMA通道Channel3---------------------*/

	DMA1_Channel5->CCR &= ~( 1<<14 ) ;        //非存储器到存储器模式
	DMA1_Channel5->CCR |=    0<<12   ;        //通道优先级最低
	DMA1_Channel5->CCR &= ~( 3<<10 ) ;        //存储器数据宽度8bit
	DMA1_Channel5->CCR &= ~( 3<<8 )  ;        //外设数据宽度8bit
	DMA1_Channel5->CCR |=    1<<7    ;        //存储器地址增量模式
	DMA1_Channel5->CCR &= ~( 1<<6 )  ;        //不执行外设地址增量模式
	DMA1_Channel5->CCR &= ~( 1<<5 ) ;         //不执行循环操作
	DMA1_Channel5->CCR |=    1<<4    ;        //从存储器读

	DMA1_Channel5->CNDTR &= 0x0000   ;        //传输数量寄存器清零
	DMA1_Channel5->CNDTR = SPI_2_CommLen ;       //传输数量设置为buffersize个
	
	DMA1_Channel5->CPAR = SPI2_DR_Addr ;      //设置外设地址，注意PSIZE
	DMA1_Channel5->CMAR = (u32)SPI_2_SendBuf ; //设置DMA存储器地址，注意MSIZE	
	
	
	
	DMA1_Channel4->CCR|=1<<1;	    // transfer (recevie) complete ISR.
	MY_NVIC_Init(3,3,DMA1_Channel4_IRQChannel,2);
}


void SPI_2_Trans(u8 TransLen)
{

	while (SPI_2_Rx_sts !=SPI_IDLE);
	DMA1_Channel4->CCR &=~1;	
	DMA1_Channel5->CCR &=~1;
	SPI_2_Rx_sts=SPI_BUSY;
  	SPI_2_CommLen=TransLen;
	DMA1_Channel4->CNDTR=TransLen;
	DMA1_Channel5->CNDTR=TransLen;

	DMA1_Channel4->CPAR = SPI2_DR_Addr ;      //设置外设地址，注意PSIZE
	DMA1_Channel4->CMAR = (u32)SPI_2_RcvBuf ; //设置DMA存储器地址，注意MSIZE
	DMA1_Channel5->CPAR = SPI2_DR_Addr ;      //设置外设地址，注意PSIZE
	DMA1_Channel5->CMAR = (u32)SPI_2_SendBuf ; //设置DMA存储器地址，注意MSIZE	

	PCRCHip_Sel=0;
	DMA1_Channel4->CCR |=1;	
	DMA1_Channel5->CCR |=1;

  while (SPI_2_Rx_sts!=SPI_CMPLT);
	//txc//delay_ms(20);
	SPI_2_Rx_sts=SPI_IDLE;
}

#define  DMA1_CH4_ISR_CLR ((u32)(3<<12))
#define  DMA1_CH5_ISR_CLR ((u32)(3<<16))
void DMA1_Channel4_IRQHandler(void)
{
  DMA1->IFCR |= DMA1_CH4_ISR_CLR;
  while(SPI2->SR & (1<<7));
  PCRCHip_Sel=1;
  SPI_2_Rx_sts=SPI_CMPLT;
	DMA1_Channel4->CCR &=~1;	
	DMA1_Channel5->CCR &=~1;
//  SPI_Rev_Data_Copy(SPI_2_CommLen);
}

void DMA1_Channel5_IRQHandler(void)
{
  DMA1->IFCR |= DMA1_CH5_ISR_CLR;
  while(SPI2->SR & (1<<7));
  PCRCHip_Sel=1;
}

u8 RowData[SPI_BUF_BYTE_LEN]={0};
/*LPG:	copy the SPI data into RowData buffer*/
/*		this function will correct the ADC data neanwhile*/
/*		4 bit ovelopped will be removed	*/
void SPI_Rev_Data_Copy(u8 length)
{

	u8 i;
/*
	for(i=0;i<length;i++)
	{
		if((i & 0x1)==0)
			RowData[i]=SPI_2_RcvBuf[i];	
		else
			RowData[i]=(SPI_2_RcvBuf[i]>>4);	
	}
*/

	for(i=0;i<length;i++)
			RowData[i]=SPI_2_RcvBuf[i];	
				
}


