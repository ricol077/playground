#include "spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//SPI���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ������SD Card/W25Q64/NRF24L01						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI2�ĳ�ʼ��
/*
void SPI2_Init(void)
{	 
	RCC->APB2ENR|=1<<3;  	//PORTBʱ��ʹ�� 	 
	RCC->APB1ENR|=1<<14;   	//SPI2ʱ��ʹ�� 
	//����ֻ���SPI�ڳ�ʼ��
	GPIOB->CRH&=0X000FFFFF; 
	GPIOB->CRH|=0XBBB00000;	//PB13/14/15���� 	    
	GPIOB->ODR|=0X7<<13;   	//PB13/14/15����
	SPI2->CR1|=0<<10;		//ȫ˫��ģʽ	
	SPI2->CR1|=1<<9; 		//���nss����
	SPI2->CR1|=1<<8;  

	SPI2->CR1|=1<<2; 		//SPI����
	SPI2->CR1|=0<<11;		//8bit���ݸ�ʽ	
	SPI2->CR1|=1<<1; 		//����ģʽ��SCKΪ1 CPOL=1
	SPI2->CR1|=1<<0; 		//���ݲ����ӵڶ���ʱ����ؿ�ʼ,CPHA=1  
	//��SPI2����APB1������.ʱ��Ƶ�����Ϊ36M.
	SPI2->CR1|=3<<3; 		//Fsck=Fpclk1/256
	SPI2->CR1|=0<<7; 		//MSBfirst   
	SPI2->CR1|=1<<6; 		//SPI�豸ʹ��
	SPI2_ReadWriteByte(0xff);//��������		 
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
	RCC->APB1ENR|=1<<14;   	//SPI2ʱ��ʹ�� 
	RCC->APB2ENR|=1<<3;     //ʹ��PORTBʱ�� 	    
 	RCC->APB2ENR|=1<<5;		//ʹ��PORTDʱ��
 	RCC->APB2ENR|=1<<8;		//ʹ��PORTGʱ��
	GPIOB->CRH&=0XFFF00FFF; 
	GPIOB->CRH|=0X00030000;	//PB12 ���� 	    
	GPIOB->ODR|=1<<12;    	//PB12����

 	GPIOB->CRH|=0X00003000;	//PB11 ���� 	    
	GPIOB->ODR|=1<<11;    	//PB11����	 as PCRChip selection

	//����pd2��pg7����,��Ϊ�˷�ֹNRF24L01��SD��Ӱ��FLASH����д.
	//��Ϊ���ǹ���һ��SPI��. 
	GPIOD->CRL&=0XFFFFF0FF; 
	GPIOD->CRL|=0X00000300;	//PD2 ���� 	    
	GPIOD->ODR|=1<<2;    	//PD2����

	PCRCHip_Sel=1;
	//����ֻ���SPI�ڳ�ʼ��
	GPIOB->CRH&=0X000FFFFF; 
	GPIOB->CRH|=0XBBB00000;	//PB13/14/15���� 	    
	GPIOB->ODR|=0X7<<13;   	//PB13/14/15����
	SPI2->CR1|=0<<10;		//ȫ˫��ģʽ	
	SPI2->CR1|=1<<9; 		//���nss����
	SPI2->CR1|=1<<8;  

	SPI2->CR1|=1<<2; 		//SPI����
	SPI2->CR1|=0<<11;		//8bit���ݸ�ʽ	
	SPI2->CR1|=0<<1; 		//����ģʽ��SCKΪ0 CPOL=0
	SPI2->CR1|=0<<0; 		//���ݲ����ӵڶ���ʱ����ؿ�ʼ,CPHA=0  
	//��SPI2����APB1������.ʱ��Ƶ�����Ϊ36M.
	SPI2->CR1|=4<<3; 		//Fsck=Fpclk1/32, 4<<3;
	SPI2->CR1|=0<<7; 		//MSBfirst   
	SPI2->CR2|=3;
	SPI2->CR1|=1<<6; 		//SPI�豸ʹ��

	SPI_2_DMA_Initialization();
		 
}   

#define SPI2_DR_Addr ( (u32)0x4000380C )
void SPI_2_DMA_Initialization(void)
{
	SPI_2_CommLen=0;
	RCC->AHBENR|=1<<0;							//����DMA1ʱ��
	delay_ms(5);								//�ȴ�DMAʱ���ȶ�
    RCC->AHBENR |= 1<<0 ;                     //DMA1ʱ��ʹ��

	/*------------------����SPI1_RX_DMAͨ��Channel2---------------------*/

    DMA1_Channel4->CCR &= ~( 1<<14 ) ;        //�Ǵ洢�����洢��ģʽ
	DMA1_Channel4->CCR |=    2<<12   ;        //ͨ�����ȼ���
	DMA1_Channel4->CCR &= ~( 3<<10 ) ;        //�洢�����ݿ��8bit
	DMA1_Channel4->CCR &= ~( 3<<8  ) ;        //�������ݿ��8bit
	DMA1_Channel4->CCR |=    1<<7    ;        //�洢����ַ����ģʽ
	DMA1_Channel4->CCR &= ~( 1<<6  ) ;        //��ִ�������ַ����ģʽ
	DMA1_Channel4->CCR &= ~( 1<<5  ) ;        //ִ��ѭ������
	DMA1_Channel4->CCR &= ~( 1<<4  ) ;        //�������

	DMA1_Channel4->CNDTR &= 0x0000   ;        //���������Ĵ�������
	DMA1_Channel4->CNDTR = SPI_2_CommLen ;       //������������Ϊbuffersize��

	DMA1_Channel4->CPAR = SPI2_DR_Addr ;      //���������ַ��ע��PSIZE
	DMA1_Channel4->CMAR = (u32)SPI_2_RcvBuf ; //����DMA�洢����ַ��ע��MSIZE

	/*------------------����SPI1_TX_DMAͨ��Channel3---------------------*/

	DMA1_Channel5->CCR &= ~( 1<<14 ) ;        //�Ǵ洢�����洢��ģʽ
	DMA1_Channel5->CCR |=    0<<12   ;        //ͨ�����ȼ����
	DMA1_Channel5->CCR &= ~( 3<<10 ) ;        //�洢�����ݿ��8bit
	DMA1_Channel5->CCR &= ~( 3<<8 )  ;        //�������ݿ��8bit
	DMA1_Channel5->CCR |=    1<<7    ;        //�洢����ַ����ģʽ
	DMA1_Channel5->CCR &= ~( 1<<6 )  ;        //��ִ�������ַ����ģʽ
	DMA1_Channel5->CCR &= ~( 1<<5 ) ;         //��ִ��ѭ������
	DMA1_Channel5->CCR |=    1<<4    ;        //�Ӵ洢����

	DMA1_Channel5->CNDTR &= 0x0000   ;        //���������Ĵ�������
	DMA1_Channel5->CNDTR = SPI_2_CommLen ;       //������������Ϊbuffersize��
	
	DMA1_Channel5->CPAR = SPI2_DR_Addr ;      //���������ַ��ע��PSIZE
	DMA1_Channel5->CMAR = (u32)SPI_2_SendBuf ; //����DMA�洢����ַ��ע��MSIZE	
	
	
	
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

	DMA1_Channel4->CPAR = SPI2_DR_Addr ;      //���������ַ��ע��PSIZE
	DMA1_Channel4->CMAR = (u32)SPI_2_RcvBuf ; //����DMA�洢����ַ��ע��MSIZE
	DMA1_Channel5->CPAR = SPI2_DR_Addr ;      //���������ַ��ע��PSIZE
	DMA1_Channel5->CMAR = (u32)SPI_2_SendBuf ; //����DMA�洢����ַ��ע��MSIZE	

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


