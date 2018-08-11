#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//初始化IIC

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
 	RCC->APB2ENR|=1<<3;//先使能外设IO PORTB时钟 							 
	GPIOB->CRH&=0XFFFF00FF;//PB1/11  OD mode
	GPIOB->CRH|=0X00007700;	   
	GPIOB->ODR|=3<<10;     //PB10,11 输出高
}
//产生IIC起始信号
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
//产生IIC停止信号
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
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
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
	//SCL_L;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
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
			delay_us(2);   //对TEA5767这三个延时都是必须的
			SCL_H;
			delay_us(4); 
			SCL_L;
			delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}



























