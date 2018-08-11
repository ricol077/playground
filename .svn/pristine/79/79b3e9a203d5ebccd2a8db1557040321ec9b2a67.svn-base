/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V2.2.1
* Date               : 09/22/2008
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_istr.h"
#include "user_def.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8 Receive_Buffer[64];//nReportCnt];
u8 Transi_Buffer[64];//nReportCnt];
u8 USB_ReceiveFlg = FALSE;
u16 USB_RxIdx=0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{
	u16 i,j;
    //USB_ReceiveFlg = TRUE;
	i=_GetEPRxCount(ENDP1);
	i &= 0x1ff;
	PMAToUserBufferCopy(Receive_Buffer, ENDP1_RXADDR,i);

	USB_ReceiveFlg = FALSE;
	USB_RxIdx=0;

	if(Receive_Buffer[0]==HEADER)
	{
//		if((RxBuffer[i-1]==TAIL) && (RxBuffer[i-2]==TAIL))
		{
			//////only for yaya USB in case 22byte
			 for(j=0;j<i;j++)
			 {
			 	if((Receive_Buffer[j]==TAIL) && (Receive_Buffer[j+1]==TAIL))
					break;
			 }
			 for(i=0;i<=(j+1);i++)
			 	RxBuffer[i]=Receive_Buffer[i];
			////////////////////
			
		 	USB_ReceiveFlg=TRUE;
			USB_RxIdx=i-1;
		}	
	}

	SetEPRxStatus(ENDP1, EP_RX_VALID);
}

void EP2_IN_Callback(void)
{
	#if defined(USB_WAIT_MODE)
	EpMsgStk(1);
	SetEPTxStatus(ENDP1, EP_TX_NAK);
	#endif	 
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

