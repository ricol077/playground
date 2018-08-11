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
	
//	RCC->APB2ENR |= 0x1;
	   	 ////////////////TXC
	

#ifdef __QPCR_HW    //qPCR HW
/////////////////POR on PA9,neg assert 
  GPIOA->CRH&=0XFFFFFF0F;			 // PA9
	GPIOA->CRH|=(u32)(0X3<<4);
	GPIOA->ODR|= (u32)(0x1<<9); 	  // default output hi
	
/////////////////////////
  GPIOA->CRL&=0XFFFF0000;			 // PA0-PA3 output, LED control
	GPIOA->CRL|=(u32)(0X3333);
	GPIOA->ODR&= ~(u32)(0xf); 	  // default output low

//////////////////////////////////////////////////////////////////////
  ////// Exti trigger IRQ
	GPIOC->CRH&=0xFF0000FF;		   			//PC10-PC13 input trigger, input PP
	GPIOC->CRH|=(u32)((u32)0x8888<<8); 
	GPIOC->ODR &= ~(u32)(0xF<<10);

/*
	Ex_NVIC_Config(GPIO_C,13,RTIR);
	Ex_NVIC_Config(GPIO_C,12,RTIR);
	Ex_NVIC_Config(GPIO_C,11,RTIR);
	Ex_NVIC_Config(GPIO_C,10,RTIR);
	MY_NVIC_Init(2,3,EXTI15_10_IRQChannel,2);
*/
	////////  ADC done IRQ
	
	GPIOA->CRL&=0X000FFFFF;	     //PA5-PA8 input ADC RDY, input pp
	GPIOA->CRL|=0X88800000; 
	GPIOA->CRH&=0XFFFFFFF0;
	GPIOA->CRH|=0X00000008; 
	GPIOA->ODR |= (u32)(0xF<<5);

	Ex_NVIC_Config(GPIO_A,5,RTIR);
	Ex_NVIC_Config(GPIO_A,6,RTIR);
	Ex_NVIC_Config(GPIO_A,7,RTIR);
	Ex_NVIC_Config(GPIO_A,8,RTIR);
	MY_NVIC_Init(2,3,EXTI9_5_IRQChannel,2);


	GPIOC->CRL &= 0XFFFF0000; //PC0 ,1,2,3 is analog mode

	
	
	///////////// Xc change SPI SS pin to PB6,7,8,12
	GPIOB->CRL&=0X00FFFFFF;			 // SS2--PB6
	GPIOB->CRL|=(u32)(0X33<<24);
	GPIOB->ODR|= (u32)(0x3<<6); 	  // default output hi
	
	GPIOB->CRH&=0XFFF0FFF0;			 // SS2--PB8,12
	GPIOB->CRH|=(u32)(0X3);
	GPIOB->CRH|=(u32)(0X3<<16);
	GPIOB->ODR |= (u32)(0x1<<8); 	  // default output hi
	GPIOB->ODR |= (u32)(0x1<<12); 

#else  ////////2 brd branch----only trigger input

	GPIOG->CRH&=0XFFFFFFF0;	
	GPIOG->CRH|=0X3;
	GPIOG->ODR|=1<<8; 

	GPIOG->CRH&=0X000FFFFF;	
	GPIOG->CRH|=(0X333<<20);
	GPIOG->ODR&=~(7<<13); 
	
	#ifdef __USE_MOS_GATE                 // version 1.13: CMOS power gate for Swiss project
				RCC->APB2ENR|=1<<5;
				GPIOD->CRL &= 0XFFFFF0FF;			 	// PD2, new feature, MOS gate for CMOS power
				GPIOD->CRL |= (u32)(0X3<<8);
				GPIOD->ODR |= (u32)(0x1<<2); 	// default output high
	#endif 
	
  #ifdef __SWISS_LUCENTIX_BUTTON       // version 1.14: LUCENTIX button
	
				GPIOF->CRH&=0XF0FFFFFF;			 			// PF14- output LED_EX
				GPIOF->CRH|=(u32)(0X3<<24);
				GPIOF->ODR&= ~(u32)(0x1<<14); 	  // default output low
	
				#ifdef	__LUCENTIX_DEBUG_ON_WARSHIP
						GPIOC->CRH&=0XFFFF0FFF;			 			// PC11- output LED_KEY for debug to avoid OSC pin PC15 on warship
						GPIOC->CRH|=(u32)(0X3<<12);
						GPIOC->ODR&= ~(u32)(0x1<<11); 	  // default output low	
						
						GPIOC->CRH&=0xFFFFF0FF;		   			//PC10 - input KEY1 on warship to avoid OSC pin PC14
						GPIOC->CRH|=(u32)((u32)0x8<<8); 
						GPIOC->ODR |= (u32)(1<<10);				// pull up
						
				#else
						GPIOC->CRH&=0X0FFFFFFF;			 			// PC15- output LED_KEY
						GPIOC->CRH|=(u32)(0X3<<28);
						GPIOC->ODR&= ~(u32)(0x1<<15); 	  // default output low
						
						GPIOC->CRH&=0xF0FFFFFF;		   			//PC14 - input KEY1
						GPIOC->CRH|=(u32)((u32)0x8<<24); 
						GPIOC->ODR |= (u32)(1<<14);				// pullup, button press to low
				#endif
				


	#endif


  GPIOF->CRL&=0XFF00FFFF;			 // PF4, PF5 output PP, FAN, LED control
	GPIOF->CRL|=(u32)(0X33<<16);
	GPIOF->ODR&= ~(u32)(0x3<<4); 	  // default output low

  GPIOD->CRL&=0X0FFFFFFF;			 // PD7, LED control
	GPIOD->CRL|=(u32)(0X3<<28);
	GPIOD->ODR&= ~(u32)(0x1<<7); 	  // default output low

  GPIOD->CRH&=0X00FFFFFF;			 // PD14,PD15 LED control
	GPIOD->CRH|=(u32)(0X33<<24);
	GPIOD->ODR&= ~(u32)(0x3<<14); 	  // default output low
	
																		//PD5,PD6 is debug usage LED D1,D2
	GPIOD->CRL&=0XF00FFFFF;			 
	GPIOD->CRL|=(u32)(0X33<<20);
	GPIOD->ODR|= (u32)(0x3<<5); 	  // default output high																
//////////////////////////////////////////////////////////////////////
  ////// Exti trigger IRQ
	GPIOF->CRL&=0x00FFFFFF;		   //PF6 ,PF7input trigger, input PP
	GPIOF->CRL|=(u32)((u32)0x88<<24); 
	GPIOF->ODR &= ~(u32)(3<<6);

	GPIOF->CRH&=0XFFFFFF00;		   //PF8 ,PF9 input trigger, input PP
	GPIOF->CRH|=(u32)(0X88); 
	GPIOF->ODR &= ~(u32)(3<<8);
	
	
	////////  ADC done IRQ
	
	GPIOA->CRL&=0XFFFFFFF0;	     //PA0 input ADC RDY, input pp
	GPIOA->CRL|=0X00000008; 
	GPIOA->ODR |= (u32)1;

	Ex_NVIC_Config(GPIO_A,0,RTIR);
//	EXTI->IMR |= 0x1;
//	AFIO->EXTICR[0] &= ~(u32)0xF;
	
//	Ex_NVIC_Config(GPIO_F,13,RTIR);
//	Ex_NVIC_Config(GPIO_F,14,RTIR);
//	Ex_NVIC_Config(GPIO_F,15,RTIR);
	MY_NVIC_Init(2,3,EXTI0_IRQChannel,2);
/*
	GPIOF->CRH&=0X000FFFFF;		   //PF13 - 15 input ADC RDY, input PP
	GPIOF->CRH|=(u32)((u32)0x888<<20); 
	GPIOF->ODR &= ~(u32)(7<<13);
*/
	GPIOE->CRH&=0X000FFFFF;		   //PE13 - 15 input ADC RDY, input PP
	GPIOE->CRH|=(u32)((u32)0x888<<20); 
	GPIOE->ODR &= ~(u32)(7<<13);
	Ex_NVIC_Config(GPIO_E,13,RTIR);
	Ex_NVIC_Config(GPIO_E,14,RTIR);
	Ex_NVIC_Config(GPIO_E,15,RTIR);
	MY_NVIC_Init(2,3,EXTI15_10_IRQChannel,2);
   
	GPIOC->CRL &= 0XFFFF0000; //PC0 ,1,2,3 is analog mode

	
	
	///////////// Xc change SPI SS pin to PB6,7,8,
	GPIOB->CRL&=0X00FFFFFF;			 // SS2--PB6
	GPIOB->CRL|=(u32)(0X33<<24);
	GPIOB->ODR|= (u32)(0x3<<6); 	  // default output hi
	
	GPIOB->CRH&=0XFFFFFFF0;			 // SS2--PB6
	GPIOB->CRH|=(u32)(0X3);
	GPIOB->ODR |= (u32)(0x1<<8); 	  // default output hi
#endif	

}






