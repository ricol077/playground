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


#ifdef __QPCR_HW
	#define PCRCHip_Sel	PBout(12) 
	#define PCRCHip_Sel_1 PBout(6)
	#define PCRCHip_Sel_2 PBout(7)
	#define PCRCHip_Sel_3 PBout(8)
#else
	#define PCRCHip_Sel	PBout(12) 
	#define PCRCHip_Sel_1 PBout(6)
	#define PCRCHip_Sel_2 PBout(7)
	#define PCRCHip_Sel_3 PBout(8)
#endif
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
	
	GPIOD->CRL&=0X000FFFFF; 
	GPIOD->CRL|=0X33300000;	//PD5-7 PP out as SS    
	GPIOD->ODR|=7<<5;    	  //����
	
	
	//����pd2��pg7����,��Ϊ�˷�ֹNRF24L01��SD��Ӱ��FLASH����д.
	//��Ϊ���ǹ���һ��SPI��. 
	GPIOD->CRL&=0XFFFFF0FF; 
	GPIOD->CRL|=0X00000300;	//PD2 ���� 	    
	GPIOD->ODR|=1<<2;    	//PD2����

	PCRCHip_Sel=1;
	PCRCHip_Sel_1=1;
	PCRCHip_Sel_2=1;
	PCRCHip_Sel_3=1;
	
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
	PCRCHip_Sel=PCRCHip_Sel_1=PCRCHip_Sel_2=PCRCHip_Sel_3=1;	 
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

	switch(SPI_Sel)
	{
		case 0:
			PCRCHip_Sel=0;
			break;
		
		case 1:
			PCRCHip_Sel_1=0;
			break;
				
		case 2:
			PCRCHip_Sel_2=0;
			break;
						
		case 3:
			PCRCHip_Sel_3=0;
			break;
								
		default:
				break;
	}

	
	
	DMA1_Channel4->CCR |=1;	
	DMA1_Channel5->CCR |=1;

  while (SPI_2_Rx_sts!=SPI_CMPLT);
	//txc//delay_ms(20);
	PCRCHip_Sel=PCRCHip_Sel_1=PCRCHip_Sel_2=PCRCHip_Sel_3=1;
	SPI_2_Rx_sts=SPI_IDLE;
}

#define  DMA1_CH4_ISR_CLR ((u32)(3<<12))
#define  DMA1_CH5_ISR_CLR ((u32)(3<<16))
void DMA1_Channel4_IRQHandler(void)
{
  DMA1->IFCR |= DMA1_CH4_ISR_CLR;
  while(SPI2->SR & (1<<7));
  //PCRCHip_Sel=PCRCHip_Sel_1=PCRCHip_Sel_2=PCRCHip_Sel_3=1;
  SPI_2_Rx_sts=SPI_CMPLT;
	DMA1_Channel4->CCR &=~1;	
	DMA1_Channel5->CCR &=~1;
//  SPI_Rev_Data_Copy(SPI_2_CommLen);
}

void DMA1_Channel5_IRQHandler(void)
{
  DMA1->IFCR |= DMA1_CH5_ISR_CLR;
  while(SPI2->SR & (1<<7));
  PCRCHip_Sel=PCRCHip_Sel_1=PCRCHip_Sel_2=PCRCHip_Sel_3=1;
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


#ifdef __USE_SPI_1_SLAVE  // version 1.8 : new feature, SPI slave interface


static u8 SPI_1_RxBuf[MAX_LIMIT]={0};
static u8 SPI_1_TxBuf[MAX_LIMIT]={0};
static u8 SPI_1_RxIdex=0;
static u8 SPI_1_TxIdex=0;
static u8 SPI_1_Send_Len=0;
static u8 SPI_1_Recv_Len=0;

static u8 SPI_1_Busy=0;


void SPI1_Slave_Initializaion(void)
{
	RCC->APB2ENR|=1<<12;   	//SPI1ʱ��ʹ�� 
	RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ�� 	    
	
	//����ֻ���SPI�ڳ�ʼ��
	GPIOA->CRL&=0X0000FFFF; 
	GPIOA->CRL|=0XBBBB0000;	//PA4~7���� 	    
	GPIOA->ODR|=0XF<<4;   	//PA4~7����
	SPI1->CR1 &=~(1<<10);		//ȫ˫��ģʽ	
	SPI1->CR1 &=~(1<<9); 		//disable���nss����
	//SPI1->CR1|=1<<8;  

	SPI1->CR1&=~(1<<2); 		//SPI slave
	SPI1->CR1&=~(1<<11);		//8bit���ݸ�ʽ	
	SPI1->CR1&=~(1<<1); 		//����ģʽ��SCKΪ0 CPOL=0
	SPI1->CR1&=~(1<<0); 		//���ݲ����ӵ�1��ʱ����ؿ�ʼ,CPHA=0  
	//��SPI2����APB1������.ʱ��Ƶ�����Ϊ36M.
	SPI1->CR1|=4<<3; 		//Fsck=Fpclk1/32, 4<<3;
	SPI1->CR1|=0<<7; 		//MSBfirst   
	//SPI1->CR2|=3;
	SPI1->CR2|=(3<<6);   // just enable TXE, RXNE, no DMA
	SPI1->CR1|=1<<6; 		//SPI�豸ʹ��
  SPI1->DR=0x00;
	//SPI_1_DMA_Initialization();
	MY_NVIC_Init(3,3,SPI1_IRQChannel,2);
}

void  SPI1_IRQHandler(void)
{
	u8 u8temp;
	u8temp=SPI1->SR;
  if(u8temp & 0x1) // RXNE
	{
		SPI_1_RxBuf[SPI_1_RxIdex]=SPI1->DR;
		SPI_1_RxIdex++;
		if(SPI_1_RxIdex >= MAX_LIMIT)
			SPI_1_RxIdex=SPI_1_Recv_Len=0;
		else
			SPI_1_Recv_Len++;	
	}
	
	if(u8temp & 0x2) // TXE
	{
			if(SPI_1_Send_Len==0) //send done
			{
				SPI1->DR=0x00;
				SPI_1_Busy=0;
				SPI_1_TxIdex=0;
			}
			else
			{
					SPI_1_TxIdex++;
					SPI1->DR=SPI_1_TxBuf[SPI_1_TxIdex];
					SPI_1_Send_Len--;
			}
	}
	// other status
}


u8 SPI_1_Send(u8 *p, u8 cnt)
{
		u8 i;
	  if(cnt > MAX_LIMIT)
				return ERR_LENGTH;
		if(SPI_1_Busy)
			return OUT_RANGE;
		else
		{
				for(i=0;i<cnt;i++)
					SPI_1_TxBuf[i]= *(p+i);
		}
		
		SPI_1_Send_Len=cnt;
		SPI_1_TxIdex=0;
		SPI1->DR=SPI_1_TxBuf[0];
		SPI_1_Busy=1;
		
		return NO_ERR;
}

#endif	
