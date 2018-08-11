#include "led.h"
#include "user_def.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟	
	RCC->APB2ENR|=1<<4;    //使能PORTC时钟 
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟   	 
	RCC->APB2ENR|=1<<6;    //使能PORTE时钟	
	RCC->APB2ENR|=1<<8;     //使能PORTG时钟
	RCC->APB2ENR|=1<<7;     //使能PORTF时钟
	   	 ////////////////TXC

  GPIOG->CRH&=0XFFFFF0FF;	
	GPIOG->CRH|=0X33333300;
	GPIOG->ODR|=1<<10;      //PG10.11输出高 
//	GPIOG->ODR&=~(1<<6);    
//	GPIOG->ODR|=1<<8; 

///////////////
	GPIOB->CRL&=0XFF0FFFFF; 
	GPIOB->CRL|=0X00300000;//PB.5 推挽输出   	 
    GPIOB->ODR|=1<<5;      //PB.5 输出高
											  
	GPIOE->CRL&=0XFF0FFFFF;
	GPIOE->CRL|=0X00300000;//PE.5推挽输出
	GPIOE->ODR|=1<<5;      //PE.5输出高 


	GPIOG->CRL&=0X00FFF0FF;	
	GPIOG->CRL|=0X83000300;

   	GPIOG->CRH&=0XFFFFFFF0;	
	GPIOG->CRH|=0X3;
	GPIOG->ODR|=1<<2;      //PG.2输出高 
	GPIOG->ODR&=~(1<<6);    
	GPIOG->ODR|=1<<8; 

	////////2 brd branch----only trigger input
	/*
    GPIOG->CRH&=0XFF0FFFFF;	
	GPIOG->CRH|=(u32)(0X3<<20);
	GPIOG->ODR&= ~(u32)(1<<13);      //PG.13输出 0.LED control 
    */


  GPIOF->CRL&=0XFF00FFFF;			 // PF4, PF5 output PP, FAN, LED control
	GPIOF->CRL|=(u32)(0X33<<16);
	GPIOF->ODR&= ~(u32)(0x3<<4); 	  // default output low

	/*
	// PG14, is input of trigger to capture image
	GPIOE->CRH&=0XF0FFFFFF;
	GPIOE->CRH|=(u32)(0X8<<24); 
	GPIOE->ODR &= ~(u32)(1<<14);
 	*/
#ifdef __QPCR_HW
	RCC->APB2ENR|=1<<5;     //使能PORTD时钟
	GPIOD->CRL&=0XFFFFF0FF;		   //PD2 trigger, output
	GPIOD->CRL|=(u32)(0X3<<8); 
	GPIOD->ODR &= ~(u32)(0x1<<2);

	RCC->APB2ENR|=1<<4;     //使能PORTC时钟
	GPIOC->CRH&=0XFFF000FF;		   //PC10~12 trigger, output
	GPIOC->CRH|=(u32)(0X333<<8); 
	GPIOC->ODR &= ~(u32)(0x7<<10);
	
	//PB0 is fan ctrl
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟	
	GPIOB->CRL&=0XFFFFFFF0;		   //PB0 fan ctrl, output
	GPIOB->CRL|=(u32)(0X3); 
	GPIOB->ODR &= ~(u32)(0x1);	
	
#else
	GPIOF->CRL&=0X00FFFFFF;		   //PF6,7 trigger, output
	GPIOF->CRL|=(u32)(0X33<<24); 
	GPIOF->ODR &= ~(u32)(0xf<<6);
	
	GPIOF->CRH&=0XFFFFFF00;		   //PF8,9 trigger, output
	GPIOF->CRH|=(u32)(0X22); 

///////////////workarounf on H/W rev0.2, wrong layout to use PF 5,6,7, PA2
	GPIOF->CRL&=0XFF0FFFFF;		   //PF5 trigger, output
	GPIOF->CRL|=(u32)(0X3<<20); 
	GPIOF->ODR &= ~(u32)(0x1<<5);
	
	GPIOA->CRL&=0XFFFFF0FF;		   //PA2 trigger, output
	GPIOA->CRL|=(u32)((0X3)<<8); 
	GPIOA->ODR &= ~(u32)(0x1<<2);
//////////////////////////////////////
#endif
	//Ex_NVIC_Config(GPIO_F,6,RTIR); 
	//Ex_NVIC_Config(GPIO_E,15,RTIR);
	//MY_NVIC_Init(2,3,EXTI9_5_IRQChannel,2); 
	////////
#ifndef __USE_CAN_BUS 
	#ifdef __USE_RAM_DUMP
			RCC->APB2ENR|=1<<5;	 // port D enable
			GPIOD->CRH&=0XFFFFFFF0;	//PD8 PP output, active high
			GPIOD->CRH|=0X3;
			GPIOD->ODR&=~(1<<8);   
	#endif
#endif 
   /*
	GPIOC->CRH&=0XFFFFF0FF; //PC.10 is analog mode

    GPIOC->CRL&=0XFF0FFFFF;
	GPIOC->CRL|=0X00300000;	   //PC.5 is Fan control pin, PP output
	GPIOC->ODR&= ~(1<<5);      //PC.5 output Low as initial status
	*/

}






