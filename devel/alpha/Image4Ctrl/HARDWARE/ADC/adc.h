#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEKս��STM32������
//ADC ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/7
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved	
//********************************************************************************
//V1.1 20120907
//��Adc_Init�����������ڲ��¶Ȳ����ĳ�ʼ��������									  
////////////////////////////////////////////////////////////////////////////////// 

							  
#define ADC_CH1  		1  			//ͨ��1	
#define ADC_CH2			2
#define ADC_CH3	 	    3
#define ADC_CH_TEMP  	16 			//�¶ȴ�����ͨ��
#define ADC_CH10  		10  			//ͨ��1	
#define ADC_CH11			11
#define ADC_CH12	 	  12
#define ADC_CH13	 	  13
#define ADC_CH14  		14
#define ADC_CH15 		15
	   									   
void Adc_Init(void); 				//ADCͨ����ʼ��
u16  Get_Adc(u8 ch); 				//���ĳ��ͨ��ֵ 
u16 Get_Adc_Average(u8 ch,u8 times);//�õ�ĳ��ͨ��10�β�����ƽ��ֵ 	  
#endif 














