#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/8
//版本：V1.3
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//********************************************************************************
//V1.1 20120904
//1,增加TIM3_PWM_Init函数。
//2,增加LED0_PWM_VAL宏定义，控制TIM3_CH2脉宽
//V1.2 20120904
//1,新增TIM5_Cap_Init函数
//2,新增TIM5_IRQHandler中断服务函数	
//V1.3 20120908
//1,新增TIM4_PWM_Init函数						  
////////////////////////////////////////////////////////////////////////////////// 


//通过改变TIM3->CCR2的值来改变占空比，从而控制LED0的亮度
#define LED0_PWM_VAL TIM3->CCR2    
//TIM4 CH1作为PWM DAC的输出通道 
#define PWM_DAC_VAL  TIM4->CCR1 

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM5_Cap_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM3_ARR_Update(u16 arr);

void TIM1_Int_Init(u16 arr,u16 psc);
void TIM1_ARR_Update(u16 arr);

void TIM6_Int_Init(u16 arr,u16 psc);
void TIM6_ARR_Update(u16 arr);

void TIM8_Int_Init(u16 arr,u16 psc);
void TIM8_ARR_Update(u16 arr);


void TIM7_Int_Init(u16 arr,u16 psc);
void TIM7_ARR_Update(u16 arr);
void TIM7_Init(void);
void TIM7_Stop(void);
void TIM7_IRQHandler(void);
#endif























