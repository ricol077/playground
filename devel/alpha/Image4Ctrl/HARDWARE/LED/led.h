#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//LED�˿ڶ���
#define LED0 PGout(10)// LED2
#define LED1 PGout(11)// LED3
#define LED2 PGout(12)// LED4
#define LED3 PGout(13)// LED5
#define LED4 PGout(14)// LED6
#define LED5 PGout(15)// LED7


//#define LED0 PBout(5)// DS0
//#define LED1 PEout(5)// DS1	
#ifndef __QPCR_HW
	#define LED_D1 	PDout(5)
	#define LED_D2 	PDout(6)
#endif
void LED_Init(void);//��ʼ��		 				    
#endif

















