#include "timer.h"
#include "led.h"
#include "user_def.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 

//��ʱ��3�жϷ������
extern u8 trg_count; // count the trg number
extern u8 mst_flag;  // flag record
extern u16 Time_LED_Delay; // timer
extern u16 SetTm_LED_Delay;// set

	
static u8 IntCnt=0;
void TIM3_IRQHandler(void)
{ 	
	TIM3->CR1&=~(u32)(0x01);
	/*	    		  			    
	if(TIM3->SR&0X0001)//����ж�
	{
		LED1=!LED1;			    				   				     	    	
	
	*/
	
	//LED1=!LED1;			   
	TIM3->SR&=~(1<<0);//����жϱ�־λ

	if(BaseCounter>0)
		BaseCounter--;
	else
		BaseCounter=0;	
	
	if(BaseCounter>0)
	{
		
		TIM3->ARR=PCR_Regs[SPI_Sel].InteCount;
		TIM3->CR1|=(0x01);
	}
	else
	{
		if(PixReadmMode==TYP_IMAGE||PixReadmMode==TYP_24PIXIMAG)
		{
	
			TMR_Int_Flag |=(1<<IntCnt);
			IntCnt+=1;
			if(IntCnt<PIX_TOTAL_ROW)
			{
				TIM3->CNT=TIM3->ARR=PCR_Regs[SPI_Sel].InteDelayCount;
				TIM3->CR1|=(0x01);	
			}		
			else
				IntCnt=0;
	
		}
		else
			TMR_Int_Flag=1; 
	

	}	    
}
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3ʱ��ʹ��    
 	TIM3->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM3->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM3->DIER|=1<<0;   //��������ж�	  
	//TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3

	TIM3->CR1|=(0x01<<7);
	TIM3->SR&=~(1<<0);//����жϱ�־λ
	TIM3->CR1|=(0x01<<4); // count down
	TIM3->CR1|=(0x01<<3); // opm
	TIM3->CNT=(u16)arr;
  	MY_NVIC_Init(1,3,TIM3_IRQChannel,2);//��ռ1�������ȼ�3����2									 
}

void TIM3_ARR_Update(u16 arr)
{

 	TIM3->ARR=arr;  	 
	TIM3->CNT=arr;							 
}


////////////////////////////


//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!




void TIM1_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<11;	//TIM1ʱ��ʹ��    
 	TIM1->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM1->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	//TIM1->DIER|=1<<0;   //��������ж�	  
	//TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��3

	TIM1->CR1|=(0x01<<7);
	TIM1->SR&=~(1<<0);//����жϱ�־λ
	TIM1->CR1|=(0x01<<4); // count down
	//TIM2->CR1|=(0x01<<3); // opm
	TIM1->CNT=(u16)arr;
  	//MY_NVIC_Init(1,3,TIM1_UP_IRQChannel,2);//��ռ1�������ȼ�3����2
	TIM1->CCMR1=(0x01<<3); //pre-load enable
	TIM1->CCMR1 |= (0x6<<4);
/*
#ifdef __QPCR_HW
		TIM1->CCER  |= (0x3<<0);	      // active low
#else
  	TIM1->CCER  |= (0x1<<0);	      // OCRef active high, output on CH1
#endif
*/
  TIM1->CCER  |= (0x1<<0);
 	TIM1->CCR1  = 0;				  // count down mode, inactive all cycle when initial								 
}

void TIM1_ARR_Update(u16 arr)
{

 	TIM1->ARR=arr;  	 
	TIM1->CNT=arr;							 
}

void TIM8_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<13;	//TIM8ʱ��ʹ��    
 	TIM8->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM8->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	//TIM8->DIER|=1<<0;   //��������ж�	  
	//TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��3

	TIM8->CR1|=(0x01<<7);
	TIM8->SR&=~(1<<0);//����жϱ�־λ
	TIM8->CR1|=(0x01<<4); // count down
	//TIM2->CR1|=(0x01<<3); // opm
	TIM8->CNT=(u16)arr;
  	//MY_NVIC_Init(1,3,TIM8_UP_IRQChannel,2);//��ռ1�������ȼ�3����2
	TIM8->CCMR1=(0x01<<3); //pre-load enable
	TIM8->CCMR1 |= (0x6<<4);
/*
#ifdef __QPCR_HW
		TIM8->CCER  |= (0x3<<0);	      // active low
#else
  	TIM8->CCER  |= (0x1<<0);	      // OCRef active high, output on CH1
#endif
*/
  TIM8->CCER  |= (0x1<<0);
 	TIM8->CCR1  = 0;				  // count down mode, inactive all cycle when initial									 
}

void TIM8_ARR_Update(u16 arr)
{

 	TIM8->ARR=arr;  	 
	TIM8->CNT=arr;							 
}

void TIM6_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<4;	//TIM6ʱ��ʹ��    
 	TIM6->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM6->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM6->DIER|=1<<0;   //��������ж�	  
	//TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��3

	TIM6->CR1|=(0x01<<7);
	TIM6->SR&=~(1<<0);//����жϱ�־λ
	//TIM6->CR1|=(0x01<<4); // no count down i tim6
	//TIM6->CNT=(u16)arr;
  	MY_NVIC_Init(1,3,TIM6_IRQChannel,2);//��ռ1�������ȼ�3����2									 
}

void TIM6_ARR_Update(u16 arr)
{

 	TIM6->ARR=arr;  	 
	TIM6->CNT=arr;							 
}


void TIM7_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<5;	//TIM7ʱ��ʹ��    
 	TIM7->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM7->PSC=psc;  	//Ԥ��Ƶ��	  
	TIM7->DIER|=1<<0;   //��������ж�	  

	TIM7->CR1|=(0x01<<7);
	TIM7->SR&=~(1<<0);//����жϱ�־λ
  	MY_NVIC_Init(1,3,TIM7_IRQChannel,2);//��ռ1�������ȼ�3����2									 
}

void TIM7_ARR_Update(u16 arr)
{

 	TIM7->ARR=arr;  	 
	TIM7->CNT=arr;							 
}

void TIM7_Init(void)
{
	TIM7_Int_Init(100,639);  // tick= 100k/100 = 1ms
	TIM7->CR1|=0x01;
}

void TIM7_Stop(void)
{
	TIM7->CR1&=~0x01;
}


void TIM7_IRQHandler(void)
{
   TIM7->SR&=~(1<<0);
   Time_LED_Delay++;
}





