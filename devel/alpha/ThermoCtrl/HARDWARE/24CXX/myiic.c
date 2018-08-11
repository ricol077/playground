#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//��ʼ��IIC

#define SCL_H         GPIOB->BSRR = (1<<10)
#define SCL_L         GPIOB->BRR  = (1<<10)
   
#define SDA_H         GPIOB->BSRR = (1<<11)
#define SDA_L         GPIOB->BRR  = (1<<11)

#define SCL_read      GPIOB->IDR  & (1<<10)
#define SDA_read      GPIOB->IDR  & (1<<11)

#define SCL_read      GPIOB->IDR  & (1<<10)
#define SDA_read      GPIOB->IDR  & (1<<11)

void IIC_Init(void)
{					     
 	RCC->APB2ENR|=1<<3;//��ʹ������IO PORTBʱ�� 							 
	GPIOB->CRH&=0XFFFF00FF;//PB1/11  OD mode
	GPIOB->CRH|=0X00007700;	   
	GPIOB->ODR|=3<<10;     //PB10,11 �����
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	      SDA_H;
        SCL_H;
        delay_us(4);
        if(!(SDA_read))
						return;
        SDA_L;
        delay_us(4);
        if(SDA_read) 
						return; 
        SCL_L;
        delay_us(4);
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
        SCL_L;
        delay_us(4);
        SDA_L;
        delay_us(4);
        SCL_H;
        delay_us(4);
        SDA_H;
        delay_us(4);						   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SCL_L; 
	SDA_H;
  delay_us(2);	
	SCL_H; 
	delay_us(4);
	while(SDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			//SCL_L;
			return 1;
		}
	}
	//SCL_L;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	SCL_L;
	SDA_L;
	delay_us(2);
	SCL_H;
	delay_us(4);
	SCL_L;
	delay_us(4);
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	SCL_L;
	SDA_H;
	delay_us(2);
	SCL_H;
	delay_us(4);
	SCL_L;
	delay_us(2);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
    SCL_L;
    for(t=0;t<8;t++)
    {              
        if(txd&0x80)
						SDA_H;  
				else
						SDA_L;  
			txd<<=1; 	  
			delay_us(2);   //��TEA5767��������ʱ���Ǳ����
			SCL_H;
			delay_us(4); 
			SCL_L;
			delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_H;  
    for(i=0;i<8;i++ )
		{
        SCL_L;
        delay_us(4);
				SCL_H;
        receive<<=1;
        if(SDA_read) 
						receive|=0x01;   
			delay_us(4); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



























