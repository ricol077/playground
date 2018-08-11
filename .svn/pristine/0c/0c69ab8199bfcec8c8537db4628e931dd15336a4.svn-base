#include "timer.h"
#include "led.h"
#include "user_def.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 

//定时器3中断服务程序
extern u8 trg_count; // count the trg number
extern u8 mst_flag;  // flag record
extern u16 Time_LED_Delay; // timer
extern u16 SetTm_LED_Delay;// set

	
static u8 IntCnt=0;
void TIM3_IRQHandler(void)
{ 	
	TIM3->CR1&=~(u32)(0x01);
	/*	    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{
		LED1=!LED1;			    				   				     	    	
	
	*/
	
	//LED1=!LED1;			   
	TIM3->SR&=~(1<<0);//清除中断标志位

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
//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM3->PSC=psc;  	//预分频器7200,得到10Khz的计数时钟		  
	TIM3->DIER|=1<<0;   //允许更新中断	  
	//TIM3->CR1|=0x01;    //使能定时器3

	TIM3->CR1|=(0x01<<7);
	TIM3->SR&=~(1<<0);//清除中断标志位
	TIM3->CR1|=(0x01<<4); // count down
	TIM3->CR1|=(0x01<<3); // opm
	TIM3->CNT=(u16)arr;
  	MY_NVIC_Init(1,3,TIM3_IRQChannel,2);//抢占1，子优先级3，组2									 
}

void TIM3_ARR_Update(u16 arr)
{

 	TIM3->ARR=arr;  	 
	TIM3->CNT=arr;							 
}


////////////////////////////


//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!




void TIM1_Int_Init(u16 arr,u16 psc)
{
	RCC->APB2ENR|=1<<11;	//TIM1时钟使能    
 	TIM1->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM1->PSC=psc;  	//预分频器7200,得到10Khz的计数时钟		  
	//TIM1->DIER|=1<<0;   //允许更新中断	  
	//TIM1->CR1|=0x01;    //使能定时器3

	TIM1->CR1|=(0x01<<7);
	TIM1->SR&=~(1<<0);//清除中断标志位
	TIM1->CR1|=(0x01<<4); // count down
	//TIM2->CR1|=(0x01<<3); // opm
	TIM1->CNT=(u16)arr;
  	//MY_NVIC_Init(1,3,TIM1_UP_IRQChannel,2);//抢占1，子优先级3，组2
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
	RCC->APB2ENR|=1<<13;	//TIM8时钟使能    
 	TIM8->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM8->PSC=psc;  	//预分频器7200,得到10Khz的计数时钟		  
	//TIM8->DIER|=1<<0;   //允许更新中断	  
	//TIM1->CR1|=0x01;    //使能定时器3

	TIM8->CR1|=(0x01<<7);
	TIM8->SR&=~(1<<0);//清除中断标志位
	TIM8->CR1|=(0x01<<4); // count down
	//TIM2->CR1|=(0x01<<3); // opm
	TIM8->CNT=(u16)arr;
  	//MY_NVIC_Init(1,3,TIM8_UP_IRQChannel,2);//抢占1，子优先级3，组2
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
	RCC->APB1ENR|=1<<4;	//TIM6时钟使能    
 	TIM6->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM6->PSC=psc;  	//预分频器7200,得到10Khz的计数时钟		  
	TIM6->DIER|=1<<0;   //允许更新中断	  
	//TIM1->CR1|=0x01;    //使能定时器3

	TIM6->CR1|=(0x01<<7);
	TIM6->SR&=~(1<<0);//清除中断标志位
	//TIM6->CR1|=(0x01<<4); // no count down i tim6
	//TIM6->CNT=(u16)arr;
  	MY_NVIC_Init(1,3,TIM6_IRQChannel,2);//抢占1，子优先级3，组2									 
}

void TIM6_ARR_Update(u16 arr)
{

 	TIM6->ARR=arr;  	 
	TIM6->CNT=arr;							 
}


void TIM7_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<5;	//TIM7时钟使能    
 	TIM7->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM7->PSC=psc;  	//预分频器	  
	TIM7->DIER|=1<<0;   //允许更新中断	  

	TIM7->CR1|=(0x01<<7);
	TIM7->SR&=~(1<<0);//清除中断标志位
  	MY_NVIC_Init(1,3,TIM7_IRQChannel,2);//抢占1，子优先级3，组2									 
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





