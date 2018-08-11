#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//SPI驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

#include "user_def.h"				    
// SPI总线速度设置 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7


extern u8 SPI_2_SendBuf[SPI_BUF_BYTE_LEN];
extern u8 SPI_2_RcvBuf[SPI_BUF_BYTE_LEN];
extern u8 SPI_2_CommLen;
extern u8 SPI_2_Rx_sts;

void SPI_2_Trans(u8 TransLen);						  	    													  
void SPI2_Init(void);			 //初始化SPI2口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI2速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2总线读写一个字节


void SPI2_Initializaion(void);
void SPI_Rev_Data_Copy(u8 length);
extern u8 RowData[SPI_BUF_BYTE_LEN]; 


extern u8 SPI_Sel;  // work around to skip SPI prototype modification for Multi PCR
		 
#endif

