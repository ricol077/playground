#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//SPI���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

#include "user_def.h"				    
// SPI�����ٶ����� 
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
void SPI2_Init(void);			 //��ʼ��SPI2��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI2�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2���߶�дһ���ֽ�


void SPI2_Initializaion(void);
void SPI_Rev_Data_Copy(u8 length);
extern u8 RowData[SPI_BUF_BYTE_LEN]; 


extern u8 SPI_Sel;  // work around to skip SPI prototype modification for Multi PCR
		 
#endif

