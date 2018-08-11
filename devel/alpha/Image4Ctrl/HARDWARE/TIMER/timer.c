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
extern u8 SPI_Sel;
extern u16 tempReg;
static u8 IntCnt=0;
static u8 IntCnt_1=0;
static u8 IntCnt_2=0;
static u8 IntCnt_3=0;


void TIM3_IRQHandler(void)
{ 		   
	//TIM3->CR1&=~(u32)(0x01);
	TIM3->SR&=~(1<<0);//����жϱ�־λ  

	if(BaseCounter>0)
	{
		BaseCounter--;
		//TIM3->CNT=TIM3->ARR=PCR_Regs.InteCount;
		TIM3->CR1|=(0x01);
		IntCnt=0;
	}
	else
	{

		if(PixReadmMode==TYP_IMAGE||PixReadmMode==TYP_24PIXIMAG)
		{
			if(IntCnt==0)
				TIM3->CNT=TIM3->ARR=INTERVAL_DELAY_COUNT;//PCR_Regs[SPI_Sel].InteDelayCount;
			TMR_Int_Flag |=(1<<IntCnt);
			IntCnt++;
			if(IntCnt<PIX_TOTAL_ROW)
				TIM3->CR1|=(0x01);	
			else
				IntCnt=0;
		}
		else
			TMR_Int_Flag =1;
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

void TIM4_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<2;	//TIM4ʱ��ʹ��    
 	TIM4->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM4->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM4->DIER|=1<<0;   //��������ж�	  
	//TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3

	TIM4->CR1|=(0x01<<7);
	TIM4->SR&=~(1<<0);//����жϱ�־λ
	TIM4->CR1|=(0x01<<4); // count down
	TIM4->CR1|=(0x01<<3); // opm
	TIM4->CNT=(u16)arr;
  MY_NVIC_Init(1,3,TIM4_IRQChannel,2);//��ռ1�������ȼ�3����2									 
}

void TIM4_ARR_Update(u16 arr)
{

 	TIM4->ARR=arr;  	 
	TIM4->CNT=arr;							 
}

void TIM4_IRQHandler(void)
{ 		   
	TIM4->SR&=~(1<<0);//����жϱ�־λ  

	if(BaseCounter_2>0)
	{
		BaseCounter_2--;
		TIM4->CR1|=(0x01);
		IntCnt_2=0;
	}
	else
	{

		if(PixReadmMode==TYP_IMAGE||PixReadmMode==TYP_24PIXIMAG)
		{
			if(IntCnt_2==0)
				TIM4->CNT=TIM4->ARR=INTERVAL_DELAY_COUNT;//PCR_Regs[SPI_Sel].InteDelayCount;
			TMR_Int_Flag_2 |=(1<<IntCnt_2);
			IntCnt_2++;
			if(IntCnt_2<PIX_TOTAL_ROW)
				TIM4->CR1|=(0x01);	
			else
				IntCnt_2=0;
		}
		else
			TMR_Int_Flag_2 =1;
	}	
}




void TIM1_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<11;	//TIM1ʱ��ʹ��  
/*  
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
  	TIM1->CCER  |= (0x1<<0);	      // OCRef active high, output on CH1
 	TIM1->CCR1  = 0;				  // count down mode, inactive all cycle when initial	
*/
 	TIM1->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM1->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM1->DIER|=1<<0;   //��������ж�	  
	TIM1->CR1|=(0x01<<7);
	TIM1->SR&=~(1<<0);//����жϱ�־λ
	TIM1->CR1|=(0x01<<4); // count down
	TIM1->CR1|=(0x01<<3); // opm
	TIM1->CNT=(u16)arr;
  MY_NVIC_Init(1,3,TIM1_UP_IRQChannel,2);//��ռ1�������ȼ�3����2		
}

void TIM1_ARR_Update(u16 arr)
{

 	TIM1->ARR=arr;  	 
	TIM1->CNT=arr;							 
}

void TIM1_UP_IRQHandler(void)
{ 		   
	TIM1->SR&=~(1<<0);//����жϱ�־λ  

	if(BaseCounter_1>0)
	{
		BaseCounter_1--;
		TIM1->CR1|=(0x01);
		IntCnt_1=0;
	}
	else
	{

		if(PixReadmMode==TYP_IMAGE||PixReadmMode==TYP_24PIXIMAG)
		{
			if(IntCnt_1==0)
				TIM1->CNT=TIM1->ARR=INTERVAL_DELAY_COUNT;//PCR_Regs[SPI_Sel].InteDelayCount;
			TMR_Int_Flag_1 |=(1<<IntCnt_1);
			IntCnt_1++;
			if(IntCnt_1<PIX_TOTAL_ROW)
				TIM1->CR1|=(0x01);	
			else
				IntCnt_1=0;
		}
		else
			TMR_Int_Flag_1 =1;
	}	
}




void TIM8_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<13;	//TIM8ʱ��ʹ��    
/*
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
  	TIM8->CCER  |= (0x1<<0);	      // OCRef active high, output on CH1
 	TIM8->CCR1  = 0;				  // count down mode, inactive all cycle when initial			
*/
 	TIM8->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM8->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM8->DIER|=1<<0;   //��������ж�	  
	TIM8->CR1|=(0x01<<7);
	TIM8->SR&=~(1<<0);//����жϱ�־λ
	TIM8->CR1|=(0x01<<4); // count down
	TIM8->CR1|=(0x01<<3); // opm
	TIM8->CNT=(u16)arr;
  MY_NVIC_Init(1,3,TIM8_UP_IRQChannel,2);//��ռ1�������ȼ�3����2
	
}

void TIM8_ARR_Update(u16 arr)
{

 	TIM8->ARR=arr;  	 
	TIM8->CNT=arr;							 
}


void TIM8_UP_IRQHandler(void)
{ 		   
	TIM8->SR&=~(1<<0);//����жϱ�־λ  

	if(BaseCounter_3>0)
	{
		BaseCounter_3--;
		TIM8->CR1|=(0x01);
		IntCnt_3=0;
	}
	else
	{

		if(PixReadmMode==TYP_IMAGE||PixReadmMode==TYP_24PIXIMAG)
		{
			if(IntCnt_3==0)
				TIM8->CNT=TIM8->ARR=INTERVAL_DELAY_COUNT;//PCR_Regs[SPI_Sel].InteDelayCount;
			TMR_Int_Flag_3 |=(1<<IntCnt_3);
			IntCnt_3++;
			if(IntCnt_3<PIX_TOTAL_ROW)
				TIM8->CR1|=(0x01);	
			else
				IntCnt_3=0;
		}
		else
			TMR_Int_Flag_3 =1;
	}	
}


void TIM6_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<4;	//TIM6ʱ��ʹ��    
 	TIM6->ARR=arr-1;  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM6->CNT=0;
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



//////2015-09-20, add time out
extern u8 u8TimOut_flag;
extern u8 u8TimOut_check;
extern u8 u8Timout_Cnt;
////////////////////
void TIM6_IRQHandler(void)
{ 	
	TIM6->SR&=~(1<<0);//����жϱ�־λ
	u8Timout_Cnt++;
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

#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON))) 
		void TIM7_Init(u16 arr,u16 psc)
		{
			TIM7_Int_Init(arr,psc);  // tick= 100k/100 = 1ms
			TIM7->CR1|=0x01;
		}

#else
		void TIM7_Init(void)
		{
			TIM7_Int_Init(100,639);  // tick= 100k/100 = 1ms
			TIM7->CR1|=0x01;
		}
#endif

void TIM7_Stop(void)
{
	TIM7->CR1&=~0x01;
}

#if ((!defined(__QPCR_HW)) && (defined(__SWISS_LUCENTIX_BUTTON)))  //version 1.14: check button
		static u8 LED_sts=0;
		void TIM7_IRQHandler(void)
		{
			 TIM7->SR&=~(1<<0);
			 if(blink_count > 0)
			 {
					blink_count--;
				  if(LED_sts==0)
					{
						LED_KEY=LED_sts=1;
					}
					else
					{
						LED_KEY=LED_sts=0;
					}
			 }
			 else
			 {
				  TIM7->CR1&=~0x01;
					blink_count=LED_sts=0;
				  Lucentix_Button_sts_Bit_Set(BUTTON_DONE_MASK);
			 }
			 
		}

#else
		void TIM7_IRQHandler(void)
		{
			 TIM7->SR&=~(1<<0);
			 Time_LED_Delay++;
		}
#endif
//////////////////////////////////////////
////////////////////////////////////////////////////
void TIM5_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<3;	//TIM5ʱ��ʹ��    
 	TIM5->ARR=(arr);  	//�趨�������Զ���װֵ//�պ�1ms    
	TIM5->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
	TIM5->DIER|=1<<0;   //��������ж�	  

	TIM5->CNT=(arr);
	TIM5->CR1|=(0x01<<4);
	TIM5->SR&=~(1<<0);//����жϱ�־λ
	
	
	
  MY_NVIC_Init(1,3,TIM5_IRQChannel,2);//��ռ1�������ȼ�3����2
	TIM5->CR1|=(0x01<<0); 
							 
}

void TIM5_ARR_Update(u16 arr)
{

 	TIM5->ARR=arr;  	 
	TIM5->CNT=arr;							 
}
u8 tick_for_ADC=0;
void tick_for_ADC_initial(void)
{
	TIM5_Int_Init(10000,7199);
	tick_for_ADC=0;
}


