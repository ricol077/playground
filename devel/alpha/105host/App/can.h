/******************** (C) COPYRIGHT 2013 www.armjishu.com  ********************
 * �ļ���  ��can.h
 * ����    ��
 * ʵ��ƽ̨��STM32���ۿ�����
 * ��׼��  ��STM32F10x_StdPeriph_Driver V3.5.0
 * ����    ��www.armjishu.com 
*******************************************************************************/
#ifndef  __CAN_BUS_H 
#define  __CAN_BUS_H 
#include "stm32f10x.h" 	

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

void CAN_Config(void);

#endif
