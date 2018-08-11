/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "ucos_ii.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "usbd_hid_core.h"
#include "usb_conf.h"
#include "user_def.h"
extern OS_EVENT* Right_time;
extern OS_EVENT * Sem_CAN1_AVL;	
extern OS_EVENT * Sem_CAN2_AVL;	
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	unsigned char Res;
	u8 data_to_send = 0;
	OS_CPU_SR  cpu_sr; 	
	OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  	OSIntNesting++; 
    OS_EXIT_CRITICAL();
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res = USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		if(Res == '1'){
			data_to_send = 0x01;
			printf("received 1 \r\n");
			OSQPost(Right_time,(void *)data_to_send);
		}
		else if(Res == '2'){
			data_to_send = 0x02;
			printf("received 2 \r\n");
			OSQPost(Right_time,(void *)data_to_send);
		}
		else{
			printf("received num is %d",(unsigned int)Res);
		}			 
		  
     }	  
	 if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)                     // 
  	{ 
     	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  	}  
	 OSIntExit(); 
}   
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

extern CanRxMsg RxMessage;
extern CanRxMsg RxMessage_IM;
extern OS_EVENT * msg_can1_rcv_length;
extern OS_EVENT * msg_can2_rcv_length;
extern OS_EVENT * M_POLL_flag;

static u8 CAN1_Next_expect=1;
u8 CAN1_Buffer[MAX_LIMIT]={0};
static u8 CAN1_Index=0;
u8 cnt_can1=0;
void CAN1_RX0_IRQHandler(void)
{
  //CanRxMsg RxMessage;
  OS_CPU_SR  cpu_sr; 
	u8 i,j,k;	
  OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  OSIntNesting++; 
  OS_EXIT_CRITICAL();
 
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	if(RxMessage.DLC)
	{
			k=((RxMessage.StdId & 0xF0)>>4);
			j=(RxMessage.StdId & 0xF);
			if(CAN1_Next_expect == j) 
			{
					CAN1_Next_expect++;
					
					for(i=0; i<RxMessage.DLC; i++)
					{
							if(CAN1_Index<MAX_LIMIT) 
							{
									CAN1_Buffer[CAN1_Index] = RxMessage.Data[i];
									CAN1_Index++;
							}
							else
							{
									CAN1_Index=0;
									j=0;
							}
					}	

					if(j>=k)
					{
						CAN1_Next_expect=1;
						
						cnt_can1=CAN1_Index;
						OSMboxPost(msg_can1_rcv_length,(void*)&cnt_can1);
						CAN1_Index=0;
					}
					else
					{
						if(CAN1_Index==2)
						{
							if(CAN1_TX_pending[1])
							{
									CAN1_Next_expect=CAN1_Send_pending();
									CAN1_Index=0;
							}
						}

					}
			}
			else
				CAN1_Next_expect=1;
		
	}		
  OSIntExit(); 
}

/**
  * @brief  This function handles CAN2 RX0 Handler.
  * @param  None
  * @retval None
  */
/*
void CAN2_RX0_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr; 	
  OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  OSIntNesting++; 
  OS_EXIT_CRITICAL()
  CAN_Receive(CAN2, CAN_FIFO0, &RxMessage_IM);
  if(RxMessage_IM.DLC) 
			OSMboxPost(msg_can2_rcv_length,(void*)&RxMessage_IM.DLC);
  OSIntExit(); 
}
*/

u8 cnt_CAN2=0;
u8 test_flag1=0;
u8 test_flag2=0;
static u8 Next_expect=1;
u8 DualBuffer[24][MAX_LIMIT];
static u8 BufferSwitcher=0;
static u8 BufferIndex=0;

u8 ArraySize[24]={0};
u8 MonoBuffer[MAX_LIMIT]={0};
u8 MonoIndex=0;
void CAN2_RX1_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr; 
	u8 k,j,i,u8temp; 
	u8 *p;	
	u8 *index;
  OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  OSIntNesting++; 
  OS_EXIT_CRITICAL()
  CAN_Receive(CAN2, CAN_FIFO1, &RxMessage_IM);

  if(RxMessage_IM.DLC) 
	{
			k=((RxMessage_IM.StdId & 0xF0)>>4);
			j=(RxMessage_IM.StdId & 0xF);
		
			if(Next_expect == j) 
			{
					Next_expect++;
					
					if(Image_Pending)
					{
							for(i=0; i<RxMessage_IM.DLC; i++)
							{
									if(BufferIndex<MAX_LIMIT) 
									{
							  
											DualBuffer[BufferSwitcher][BufferIndex] = RxMessage_IM.Data[i];
										  BufferIndex++;

									}
									else
									{
											BufferIndex=0;
											j=0;
									}
							}	
					}
					else
					{
							for(i=0; i<RxMessage_IM.DLC; i++)
							{
									if(MonoIndex<MAX_LIMIT) 
									{
											MonoBuffer[MonoIndex] = RxMessage_IM.Data[i];
										  MonoIndex++;
									}
									else
									{
											MonoIndex=0;
											j=0;
									}
							}	
					}
					if(j>=k)
					{
						Next_expect=1;
						
						if(Image_Pending)
						{
								u8temp=(Image_Pending==1)? 12:24;
							
								cnt_CAN2=(BufferIndex & 0x7F);
								ArraySize[BufferSwitcher]=cnt_CAN2;
								test_flag2++;
								BufferIndex=0;
								for(i=0;i<u8temp;i++)
								{
									if(ArraySize[i]==0)
										break;
								}
								if(i<u8temp)
									BufferSwitcher=i;
								else
								{
									BufferSwitcher=0;

									OSMboxPost(msg_can2_rcv_length,(void*)&cnt_CAN2);
									for(i=0;i<u8temp;i++)
										ArraySize[i]=0;
								}
						}	
						else
						{
							cnt_CAN2=MonoIndex;
							OSMboxPost(msg_can2_rcv_length,(void*)&cnt_CAN2);
							MonoIndex=0;
						}
					}
			}
			else
				Next_expect=1;
	}
  OSIntExit(); 
}

void CAN1_TX_IRQHandler()
{
  CanRxMsg RxMessage;
  OS_CPU_SR  cpu_sr; 	
  OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  OSIntNesting++; 
  OS_EXIT_CRITICAL()

	if((CAN1->TSR & (0x3<<24)) !=0)
			OSSemPost(Sem_CAN1_AVL);
  OSIntExit(); 
}

void CAN2_TX_IRQHandler()
{
  CanRxMsg RxMessage;
  OS_CPU_SR  cpu_sr; 	
  OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  OSIntNesting++; 
  OS_EXIT_CRITICAL()

	if((CAN2->TSR & (0x3<<24)) !=0)
			OSSemPost(Sem_CAN2_AVL);
  OSIntExit(); 
}

u8 IRQ_happen=1;
void TIM3_IRQHandler(void)   //TIM3中断
{
	OS_CPU_SR  cpu_sr; 
  OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  OSIntNesting++; 
  OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 
		OSMboxPost(M_POLL_flag,(void*)&IRQ_happen);
	}
	OSIntExit(); 	
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    OS_CPU_SR  cpu_sr;


    OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

    OSTimeTick();                                /* Call uC/OS-II's OSTimeTick()                       */

    OSIntExit();                                 /* Tell uC/OS-II that we are leaving the ISR          */

}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 
///////////////////////////////////////////////////////////////
//  USB ISR Handler
/////////////////////////////////////////////////////////////////
void OTG_FS_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr; 	
  OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断// Tell uC/OS-II that we are starting an ISR
  OSIntNesting++; 
  OS_EXIT_CRITICAL();
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
	OSIntExit(); 
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
