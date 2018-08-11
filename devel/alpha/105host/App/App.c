#include <includes.h>
OS_EVENT* Right_time;
void *RightQueueTbl[20];
static  OS_STK  startup_task_stk[STARTUP_TASK_STK_SIZE];   //开辟任务堆栈
static  OS_STK  led1_task_stk[LED1_TASK_STK_SIZE ];    //开辟任务堆栈
static  OS_STK  CAN1_task_stk[CAN1_TASK_STK_SIZE ];    //开辟CAN任务堆栈
static  OS_STK  USB_task_stk[USB_TASK_STK_SIZE ];    //开辟CAN任务堆栈
static  OS_STK  CAN2_task_stk[CAN2_TASK_STK_SIZE ];

static  OS_STK  CMD_task_stk[CMD_TASK_STK_SIZE ];
static void systick_init(void); //函数声明
 
u8 cmd_buf[64];  // command received both USB / BT
u8 usb_rcv_buf[64];

OS_EVENT * M_POLL_flag;
//////////////////////////////////////////////////
u8 Buf_CAN1_DAT[CAN_MAX_LEN]={0};
OS_EVENT * msg_can1_rcv_length;			//按键邮箱事件块	 
CanRxMsg RxMessage;
OS_EVENT * Sem_CAN1_AVL;	
OS_EVENT * M_Temper_reply_len;	
//OS_EVENT * M_can1_wait_length;	
OS_EVENT * M_can1_cmd_length;	
OS_EVENT * M_USB_rsv;

//////////////////////////////////
u8 Buf_CAN2_DAT[CAN_MAX_LEN]={0};
OS_EVENT * msg_can2_rcv_length;			//按键邮箱事件块	 
CanRxMsg RxMessage_IM;
OS_EVENT * Sem_CAN2_AVL;	
OS_EVENT * M_Image_reply_len;	
//OS_EVENT * M_can2_wait_length;	
OS_EVENT * M_can2_cmd_length;	


/////////////////////////////////
u8 Temper_send_packet[64]={0};
u8 Temper_CMD_packet [64]={0};
u8 Temper_reply_packet[64]={0};

u8 Image_send_packet[64]={0};
u8 Image_CMD_packet [64]={0};
u8 Image_reply_packet[64]={0};
u8 ImageArrayBuf[MAX_PCR_CH][24][64]={0}; 
u8 ImageBuf[PIX_TOTAL_ROW][(PIX_TOTAL_COL<<1)+2]={0};

u8 can1_cmd_buf[64]={0};  //save command data for CAN1
u8 can2_cmd_buf[64]={0};  //save command data for CAN2

u32 can2_timeout=10;

u8 Image_Pending=0;
/////////////////////////////////////
static void CAN1_task(void *p_arg);
static void CAN2_task(void *p_arg);


///////////////////////////////////
static void systick_init(void) 
{ 
    RCC_ClocksTypeDef  rcc_clocks; 
    RCC_GetClocksFreq(&rcc_clocks);   //调用标准库函数，获取系统时钟。
    SysTick_Config(rcc_clocks.HCLK_Frequency / OS_TICKS_PER_SEC); //初始化并使能SysTick
									          //OS_TICKS_PER_SEC是在os_cfg.h中定义的
}

static void led1_task(void *p_arg)
{   
    p_arg=p_arg;      //防止编译器产生警告
	while(1)
	{	 
		 /*LED1以2s频率闪烁*/
		 led_on(LED4);
     	 OSTimeDlyHMSM(0,0,1,0);  //1s延时，释放CPU控制权 
		 led_off(LED4);
		 OSTimeDlyHMSM(0,0,1,0); 	//1s延时，释放CPU控制权
	}
}
#define    SET_CAN_SJW   CAN_SJW_1tq
#define    SET_CAN_BS1   CAN_BS1_4tq	// 8
#define    SET_CAN_BS2   CAN_BS2_4tq	// 7
#define    SET_CAN_PRES  8				// 波特率分频器 9-250K 18-125K 


CanTxMsg TxMsg1={0xAB,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAB,0,0,0,0,0,0,0}};
CanTxMsg TxMsg2={0xAB,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAB,0,0,0,0,0,0,0}};
void CAN1_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* 打开GPIO时钟、AFIO时钟，CAN时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	/* CAN1 RX PB8 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* CAN1 TX PB9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap

    /* CAN1 Enabling interrupt */									  
    NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);									
							  	
    /* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);   

    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW=sjw;
    CAN_InitStructure.CAN_BS1=bs1;  
    CAN_InitStructure.CAN_BS2=bs2;	
    CAN_InitStructure.CAN_Prescaler=pres;
    

    CAN_Init(CAN1,&CAN_InitStructure);	// CAN1											

    CAN_FilterInitStructure.CAN_FilterNumber=0;	 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // 标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// 过滤器标识符
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// 过滤器屏蔽标识符
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0指向过滤器
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    //CAN_ITConfig(CAN1,(CAN_IT_FMP0|CAN_IT_TME),ENABLE);  // CAN1
		CAN_ITConfig(CAN1,(CAN_IT_FMP0),ENABLE);
}






/*
void CAN2_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);//使能PORTB时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
    GPIO_Init(GPIOB, &GPIO_InitStructure);		//初始化IO
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化IO

		GPIO_PinRemapConfig(GPIO_Remap_CAN2,ENABLE);  // CAN1 remap
							
							  	
    // CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler 
		CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);   

    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW=sjw;
    CAN_InitStructure.CAN_BS1=bs1;  
    CAN_InitStructure.CAN_BS2=bs2;	
    CAN_InitStructure.CAN_Prescaler=pres;
    

    CAN_Init(CAN2,&CAN_InitStructure);	// CAN2											

		CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0

  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}
*/

void CAN2_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* 打开GPIO时钟、AFIO时钟，CAN时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	/* CAN2 RX PB12 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* CAN2 TX PB13 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);								

	/* CAN2 Enabling interrupt */								 	  
    NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX1_IRQn;	// FIFO_1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);								  	

    /* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);   

    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW=sjw;
    CAN_InitStructure.CAN_BS1=bs1;  
    CAN_InitStructure.CAN_BS2=bs2;	
    CAN_InitStructure.CAN_Prescaler=pres;

    CAN_Init(CAN2,&CAN_InitStructure);   // CAN2													

    CAN_FilterInitStructure.CAN_FilterNumber=14;	// 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // 标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// 过滤器标识符
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// 过滤器屏蔽标识符
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;	 // FIFO1指向过滤器
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);  // CAN2
}




void CAN_SendData(CAN_TypeDef* CANx,CanTxMsg* CanData)
{
    uint8_t retrys=0;
    uint8_t mailbox=0;

    do
	{
	    mailbox=CAN_Transmit(CANx,CanData);
		retrys++;
	}
	while((mailbox==CAN_TxStatus_NoMailBox)&&(retrys<0xFE));
	retrys=0;
}
u8 can_rcv=0;



#define LENGTH_CAN1_TEMPER	4  // temerature byte length expected to read back
#define MAX_TRY_ERR					10  // if comm tried > 10 times, always fail on CAN, will report CAN bus failure

static void CAN_POST_CMD_NACK(u8 can, CanTxMsg TxMessage)
{

}	
////////////////////////////////
// can bus processor, handle set command CAN bus communication
// Input: txbuffer, Tx length, Rxbuffer
// output: the length  received
/////////////////////////////
static u8 CAN1_Set_Processor(u8 * TxBuffer, u8 Txlength, u8 RxBuffer) 
{
	CanTxMsg TxMessage;
	CanRxMsg Can1_buf;	
	INT8U err, error_cnt;
	u8 Can1_rcv_len,i,j,k,index_rcv_CAN,index_tx_CAN,Last_packet_size,Send_Packet_Num;
	//u8 Temper_reply_packet[64];
	
	TxMessage.StdId = 0x100; 	//  1 = to function 0x1  
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1; 
	Can1_rcv_len=0;				// reset reply length
	
	Send_Packet_Num =  Txlength/8;  // replay packet: ACK, cmd,length, type, data[0..3]	
	Send_Packet_Num = (Txlength%8)? (Send_Packet_Num+1): Send_Packet_Num;
	Last_packet_size = (Txlength%8)? (Txlength%8): 8;
	
	TxMessage.StdId = 0x100 | (Send_Packet_Num<<4);
	
	for(j=0;j<Send_Packet_Num;j++)
	{
		if(j==(Send_Packet_Num-1))  // last packet
			k=Last_packet_size;
		else
			k=8;
		index_tx_CAN=j*8+1;
		for (i=0;i<k;i++)
			TxMessage.Data[i]=Temper_CMD_packet[i]=can1_cmd_buf[index_tx_CAN+i];  //send valid data on CAN only,remove header

		TxMessage.StdId |= (j+1) ;  //	 0xUVZ is U is F$  V=total packet, Z= current #
		TxMessage.DLC = k; 
		CAN_SendData(CAN1, &TxMessage);
		
		//index_rcv_CAN=0;
		do
		{
			Can1_rcv_len=*(u8 *)OSMboxPend(msg_can1_rcv_length,30,&err);	
															// wait reply from Temper Func
			if((Can1_rcv_len) && (err==OS_ERR_NONE))				// reply received 
			{
				error_cnt=0;
				for(i=0; i<Can1_rcv_len; i++)
					Temper_reply_packet[i] = RxMessage.Data[i];
				break; 
			}
			else										// no replay
			{
				error_cnt++;
				if(error_cnt > MAX_TRY_ERR)
				{
					Can1_rcv_len=0;
					break; 							// 
				}
			}
		}while(1);		
	
		if(error_cnt) return 0;
		else if(Temper_reply_packet[0] != NO_ERR) return Can1_rcv_len;
	}
	return Can1_rcv_len;

}
////////////////////////////////
// can bus processor, handle read command CAN bus communication
// Input: txbuffer, Tx length, Rxbuffer, length to be read
// output: the length  received
/////////////////////////////
static u8 CAN1_Read_Processor(u8 * TxBuffer, u8 Txlength, u8 RxBuffer, u8 LengthToRead) 
{
	CanTxMsg TxMessage;
	CanRxMsg Can1_buf;	
	INT8U err, error_cnt;
	u8 Can1_rcv_len,i,j,k,l,index_rcv_CAN, Read_packet_Num;

	TxMessage.StdId = 0x111; 	//  1 = to function 0x1  
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1; 
	Can1_rcv_len=0;				// reset reply length
    Read_packet_Num=(LengthToRead/8);
	Read_packet_Num=(LengthToRead%8)? (Read_packet_Num+1): Read_packet_Num;
	
	for (i=0;i<Txlength;i++)
		TxMessage.Data[i]=Temper_CMD_packet[i]=can1_cmd_buf[1+i];  //send valid data on CAN only,remove header

	TxMessage.StdId = 0x111 ;  //	 0xUVZ is U is F$  V=total packet, Z= current #
	TxMessage.DLC = Txlength; 
	CAN_SendData(CAN1, &TxMessage);
	
	index_rcv_CAN=k=j=0;
	for(l=0;l<Read_packet_Num;l++)
	{
		do
		{
			Can1_rcv_len=*(u8 *)OSMboxPend(msg_can1_rcv_length,30,&err);	
															// wait reply from Temper Func
			if((Can1_rcv_len) && (err==OS_ERR_NONE))				// reply received 
			{
				error_cnt=0;
				for(i=0; i<Can1_rcv_len; i++)
					Temper_reply_packet[index_rcv_CAN++] = RxMessage.Data[i];
				
				k=(RxMessage.StdId & 0xF0 >>4);
				j=(RxMessage.StdId & 0xF);
				
				TxMessage.StdId = 0x100 | (RxMessage.StdId & 0xFF ); //	 ACK to current packet id
				TxMessage.Data[0]= 0;  // reply ACK
				TxMessage.DLC = 1; 
				CAN_SendData(CAN1, &TxMessage);
				
				
				break; 
			}
			else										// no replay
			{
				error_cnt++;
				if(error_cnt > MAX_TRY_ERR)
				{
					Can1_rcv_len=0;
					break; 							// 
				}
			}
		}while(index_rcv_CAN< LengthToRead);		
	}
	if(error_cnt) return 0;
	else if(Temper_reply_packet[0] != NO_ERR) return index_rcv_CAN;

	return index_rcv_CAN;

}

///////////////////////////////////
// save in buffer with max limit, 
// input: *buffer, Max size limit in case overflow, index(actual size of received) for output
// return: error code- 0: NO error, 1: timeout, 2, overflow
//////////////////////////////////
#define NO_ERR	0
#define TM_OUT	1
#define OV_FLOW	2
#define NACK		3
#define PACK_MISS	4
static u8 CAN1_Receive_Function(u8 * RcvBuf, u8 maxSize, u8 * index) 
{																												
		CanTxMsg TxMessage;
		CanRxMsg Can1_buf;
		u8 Flag_CAN1_READ, Flag_CAN1_replied;
		INT8U err, error_cnt;
		u8 Can1_rcv_len,i,j,k,index_rcv_CAN, Next_expect;
		Next_expect=k=j=1;
		index_rcv_CAN=0;
		err=NO_ERR;
		*index=0;
	  
		do
		{
				Can1_rcv_len=*(u8 *)OSMboxPend(msg_can1_rcv_length,30,&err);	
																												
				if((Can1_rcv_len) && (err==OS_ERR_NONE))				
				{
						error_cnt=0;

					
						for(i=0; i<Can1_rcv_len; i++)
									RcvBuf[i] = CAN1_Buffer[i];
						break;
				}
				else										// no replay
						error_cnt++;

		}while(error_cnt < MAX_TRY_ERR);
		
		
		
		if(error_cnt)
		{
			err=TM_OUT;
		}
		else
			*index=i;
		
		return err;
}

///////////////////////////////////
// send from buffer with size set, 
// input: *buffer, size to send
// return: error code- 0: NO error, 1: timeout, 3, bus NACK
//////////////////////////////////
u8 CAN1_TX_buffer[8][8]={0};
u8 CAN1_TX_pending[2]={0};
static u8 CAN1_TX_tail=0;

static u8 CAN1_Send_Function(u8 * TxBuf, u8 Size) 
{																												
		CanTxMsg TxMessage;
		CanRxMsg Can1_buf;
		INT8U err, error_cnt;
		u8 Can1_rcv_len,i,j,k,index_txm_CAN,u8temp;
	
		if(Size==0) return NO_ERR;
	
		k=(Size/8);
		u8temp=Size%8;
		k=(u8temp>0)? (k+1):k; // packet number
		j= (k==1)? Size:(Size%8);
	
		index_txm_CAN=1;
		err=NO_ERR;

		TxMessage.StdId = 0x100; 	//  1 = to function 0x1  
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE = CAN_ID_STD;

		//Can1_rcv_len=*(u8 *)OSMboxPend(msg_can1_rcv_length,30,&err);		
	
		if(k==1)
		{
				TxMessage.StdId = 0x111;
			  CAN1_TX_pending[0]=CAN1_TX_pending[1]=0;
				for (i=1;i<Size;i++)
						TxMessage.Data[i-1]=TxBuf[i];  //cmd, len, type, sensor, float temper, uint16 time
				TxMessage.DLC = Size-1;
				CAN_SendData(CAN1, &TxMessage);
				return NO_ERR;
		}
		else
		{
				CAN1_TX_pending[0]=k;
			  for(u8temp=0;u8temp<k;u8temp++)
			  {
						if(u8temp<(k-1))
						{
								for (i=0;i<8;i++)
									CAN1_TX_buffer[u8temp][i]=TxBuf[index_txm_CAN++]; 
						}
						else
						{
								for (i=0;i<j;i++)
									CAN1_TX_buffer[u8temp][i]=TxBuf[index_txm_CAN++]; 
							  CAN1_TX_tail=j;
						}
				}
			  CAN1_TX_pending[1]=k-1;
				
				TxMessage.DLC=8;
				TxMessage.StdId = 0x100 | (k<<4)| (1);
				TxMessage.RTR = CAN_RTR_DATA;
				TxMessage.IDE = CAN_ID_STD;
				for(j=0;j<8;j++)
						TxMessage.Data[j]=CAN1_TX_buffer[0][j];
				CAN_SendData(CAN1, &TxMessage);
				
				return NO_ERR;
		}
}

u8 CAN1_Send_pending(void)
{
		CanTxMsg TxMessage;
	  u8 k,j,i;
		u8 retVal=1;

		
		i=CAN1_TX_pending[0]-CAN1_TX_pending[1];
		TxMessage.StdId = 0x100 | (CAN1_TX_pending[0]<<4)| (i+1);
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE = CAN_ID_STD;
	  if(CAN1_TX_pending[1]==1)
			k=CAN1_TX_tail;
		else
		{
			k=8;
			retVal=i+1;
		}
		for(j=0;j<k;j++)
			TxMessage.Data[j]=CAN1_TX_buffer[i][j];
		TxMessage.DLC=k;
		CAN1_TX_pending[1]--;
		CAN_SendData(CAN1, &TxMessage);
		return retVal;
}

static void CAN1_task(void *p_arg)
{   
		CanTxMsg TxMessage;
		CanRxMsg Can1_buf;
		u8 Flag_CAN1_READ, Flag_CAN1_replied;
		INT8U err, error_cnt;
		u8 Can1_rcv_len,i,j,k,index_rcv_CAN, index_send_CAN, Temper_packet_len,Temper_cmd_length,u8temp,u8temp1;
		u8 command, type;
		p_arg=p_arg;      //防止编译器产生警告

		CAN1_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  
		index_rcv_CAN=index_send_CAN=Temper_packet_len=error_cnt=0;	   			  		  				
		
		while(1)
		{				
				Temper_cmd_length =*(u8 *)OSMboxPend(M_can1_cmd_length,30,&err);  // get command length
			
				if((Temper_cmd_length) && (err==OS_ERR_NONE))		// command for termperature thread is received
				{
					
						command =	can1_cmd_buf[CMD_BYTE_NUM];
						type	= 	can1_cmd_buf[TYP_BYTE_NUM];
					
						err=CAN1_Send_Function(can1_cmd_buf,Temper_cmd_length);
						if(!err) 
									err=CAN1_Receive_Function(Temper_reply_packet, 64, &Temper_packet_len);
						if(err)
						{
								Temper_reply_packet[0]=ERR_CAN1;
								Temper_reply_packet[1]=command;
								Temper_reply_packet[2]=1;
								Temper_reply_packet[3]=type;		
								Temper_packet_len=4;
						}
						OSMboxPost(M_Temper_reply_len, (void *) &Temper_packet_len);
					} // end OSMboxPend(M_can1_cmd_length,10,&err);
			} // end while (1)
} // end CAN1_task


static void CMD_task(void *p_arg)        // command task
{

}


uint8_t temp=0x01;
uint32_t count;
static uint8_t *USBD_HID_GetPos (void)
{
  static uint8_t HID_Buffer [HID_INOUT_BYTES];
  

  count++;
  temp<<=1;
  if(temp<=0)
      temp=0x01;
  
  HID_Buffer[0] = temp;
  HID_Buffer[1] = (count&0xFF);
  HID_Buffer[2] = ((count>>8)&0xFF);
  HID_Buffer[3] = ((count>>16)&0xFF);
  HID_Buffer[4] = ((count>>24)&0xFF);
  
  return HID_Buffer;
}

u8 PacketChkSum(u8 *p, u8 length)    // packet sum check
{
	u8 i;  
	u8 temp=0;
	for	(i=1;i<(length-2);i++)
		temp+=*(p+i);
	if(temp==TAIL)
		temp++;
	if(temp==*(p+length-2))
		return TRUE;
	else
		return FALSE;
	
}

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

void USBRes(u8 response, u8 command,u8 * buffer, u8 length)
{
	u16 i, u8temp, len;
	u8 Transi_Buffer[64];
	
	Transi_Buffer[0]=HEADER;
	Transi_Buffer[1]=response;
	Transi_Buffer[2]=command;
	Transi_Buffer[3]=length;
	len=4;
	for(i=0;i<length;i++,len++)
			Transi_Buffer[len]=*(buffer+i);
	u8temp=0;
	for(i=1;i<(len-1);i++)
		u8temp+=Transi_Buffer[i];
	if(u8temp==TAIL)
		u8temp++;
   	Transi_Buffer[len]=u8temp;
	Transi_Buffer[len+1]=Transi_Buffer[len+2]=TAIL;
	USBD_HID_SendReport (&USB_OTG_dev, Transi_Buffer,HID_INOUT_BYTES);
}

void USBResFromCAN(u8 * buffer, u8 length)
{
	u16 i, u8temp, len;
	u8 Transi_Buffer[64];
	
	Transi_Buffer[0]=HEADER;
	len=1;
	for(i=0;i<length;i++,len++)
			Transi_Buffer[len]=*(buffer+i);
	u8temp=0;
	for(i=1;i<(len);i++)
		u8temp+=Transi_Buffer[i];
	if(u8temp==TAIL)
		u8temp++;
   	Transi_Buffer[len]=u8temp;
	Transi_Buffer[len+1]=Transi_Buffer[len+2]=TAIL;
	USBD_HID_SendReport (&USB_OTG_dev, Transi_Buffer,HID_INOUT_BYTES);
}

static void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	 

	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM3, ENABLE);  //使能TIMx	
}
static void TIM3_Stop(void)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_DeInit(TIM3);
	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE );
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器	
}


#define COUNT_TRESH_READ_T2	4
#define COUNT_TRESH_READ_T1	3
#define COUNT_TRESH_READ_CYCLE_STS	2
#define COUNT_TRESH_READ_NOTIFY	1

static u8 ImageArrayBuf_trigger[MAX_PCR_CH][24][64]={0};

static void USB_task(void *p_arg)				// USB thread
{
	INT8U err;
	u8 i,j,k, u8temp,u8temp1, u8temp2,Command_Len,ResCode,u8temp3, ACK_reply;
	u16 TransLen,u16temp;
	u8 cmd,type,cmd_accepted, SubCmdLenCAN1,SubCmdLenCAN2;  // length to send to CAN_BUS
	u8 WaitResLenCAN1,WaitResLenCAN2;  // length from sub device expected
	KL_union KL_temp,KP_temp,KI_temp,KD_temp,TempSet_1_temp,TempSet_2_temp,TempCurr_1_temp,TempCurr_2_temp, UnionTemp;
	float f32temp1,Fan_Gap_temp;
	CycleControlType  PCR_Cycle_Control,Buffer_Cycle_Control;
	CycleTempType	  PCR_Cycle_SetPoint[MAX_STAGE],Buffer_Cycle_SetPoint[MAX_STAGE];
	CycleTempType     *pCycleArray; 
  float integration_ms=1;
	CycleTempType     PE_Cycle_SetPoint[2]; // pre_pump & extension setpoint

	///////////////////// for polling 
	u8 cnt=0;
	u8 polling_flag=0;
	u8 IRQ_count=0;
	u8 cycleSTS=0;							// cycle status buffer
	u8 triggerSTS=0;  					// trigger IRQ buffer
	u8 trg_assert=0;
	KL_union temper_buffer[2];	// temperature buffer for two sensor
	
		
	const u8 cmd_read_temp1[]={0xAA,CMD_TEMP_SET,0x2,TEMP_READ_SENSOR,SENSOR_1};
	const u8 cmd_read_temp2[]={0xAA,CMD_TEMP_SET,0x2,TEMP_READ_SENSOR,SENSOR_2};
	const u8 cmd_read_image12[]={0xAA,CMD_GET,0x01,TYP_IMAGE}; 
	const u8 cmd_read_image24[]={0xAA,CMD_GET,0x01,TYP_24PIXIMAG}; 
	const u8 cmd_read_CycleSTS[]={0xAA,CMD_CYCLE_CTRL_READ, 0x01,CYCLE_STATE_POLL};
	const u8 cmd_read_Notify[]={0xAA,CMD_HOST_NOTIFY,0x01, TRG_VALID}; 
	
	u8 poll_type=0;
	u8 image_mode=0;
	u8 image_mask=0xf;
	u8 Image12_Pending=0;
	u8 Image24_Pending=0;
	/////////////////////
	u8 Comand_Buf[64],TxBuffer[64];  // to comply with old version, just simplify code copy only
				USBD_Init(&USB_OTG_dev,           
				USB_OTG_FS_CORE_ID,
				&USR_desc, 
				&USBD_HID_cb, 
				&USR_cb);
	while(1)
	{
		
		///////////////////////////////
		
		if(polling_flag==1)
		{
				cnt= *(u8 *)OSMboxPend(M_POLL_flag, 5, &err);
				if((cnt) && (err==OS_ERR_NONE))
				{
						IRQ_count++;
				}
#ifdef __DEBUG_ON_LED		
				led_on(LED3);
				
#endif				
		}
#ifdef __DEBUG_ON_LED
		else
		{
				led_off(LED3);
		}
#endif
		
		
		if(IRQ_count>0)
		{
				cmd_accepted = NONE_ACCEPT;
			  poll_type=trg_assert=0;
				switch(IRQ_count)
				{
					case COUNT_TRESH_READ_T2:
									poll_type=COUNT_TRESH_READ_T2;
									SubCmdLenCAN1=6; 
									cmd_accepted=CAN1_ACCEPT;       
									WaitResLenCAN1=CAN_BUS_ACK_LENTH+5;	
                  for(i=0;i<SubCmdLenCAN1;i++)
										can1_cmd_buf[i]=cmd_read_temp2[i];
						break;
					
					case COUNT_TRESH_READ_T1:
									poll_type=COUNT_TRESH_READ_T1;
									SubCmdLenCAN1=6; 
									cmd_accepted=CAN1_ACCEPT;       
									WaitResLenCAN1=CAN_BUS_ACK_LENTH+5;		
									for(i=0;i<SubCmdLenCAN1;i++)
										can1_cmd_buf[i]=cmd_read_temp1[i];
						break;
					
					case COUNT_TRESH_READ_CYCLE_STS:
									poll_type=COUNT_TRESH_READ_CYCLE_STS;
									SubCmdLenCAN1=4; 
									cmd_accepted=CAN1_ACCEPT;       
									WaitResLenCAN1=CAN_BUS_ACK_LENTH+1;
                  for(i=0;i<SubCmdLenCAN1;i++)
										can1_cmd_buf[i]=cmd_read_CycleSTS[i];					
						break;
					
					case COUNT_TRESH_READ_NOTIFY:
					/// verion 1.2: 170318, skip refresh triggerSTS if IRQ pending to be read, in case that overwrite
									if(triggerSTS==0)
									{
											poll_type=COUNT_TRESH_READ_NOTIFY;
											SubCmdLenCAN2=4; 
											cmd_accepted=CAN2_ACCEPT;       
											WaitResLenCAN2=CAN_BUS_ACK_LENTH+1;		
											for(i=0;i<SubCmdLenCAN2;i++)
												can2_cmd_buf[i]=cmd_read_Notify[i];	
									}
						break;
					
					default:
									IRQ_count=0;
						break;		
				}
				

					if(cmd_accepted & CAN1_ACCEPT)
					{
						OSMboxPost(M_can1_cmd_length, &SubCmdLenCAN1);
						//OSTimeDly(6);
						TransLen=*(u8 *)OSMboxPend(M_Temper_reply_len, 50, &err);											
						u8temp3=0;
					
						do
						{
							if((TransLen)&&(err==OS_ERR_NONE))
							{
								u8temp3=0;
								if(Temper_reply_packet[0]==0)
								{
										switch(poll_type)
										{
											case COUNT_TRESH_READ_T2:
															for(i=0;i<4;i++)
																	temper_buffer[1].tempd.byte0=Temper_reply_packet[i+CAN_BUS_ACK_LENTH+1];
												break;
											
											case COUNT_TRESH_READ_T1:
															for(i=0;i<4;i++)
																	temper_buffer[0].tempd.byte0=Temper_reply_packet[i+CAN_BUS_ACK_LENTH+1];												
												break;
											
											case COUNT_TRESH_READ_CYCLE_STS:
															cycleSTS=Temper_reply_packet[CAN_BUS_ACK_LENTH];
															if(cycleSTS==READY)
															{
																if(polling_flag)
																{
																		polling_flag=0;
																		TIM3_Stop();
																		IRQ_count=0;
																}
															}
												break;
											
											default:
												break;
										}
								}
								break;
							}
							else
							{
								u8temp3++;
								if(u8temp3< MAX_TRY_ERR) 
									TransLen=*(u8 *)OSMboxPend(M_Temper_reply_len, 50, &err);	
							}		
						}while(u8temp3< MAX_TRY_ERR);
					}
					else if(cmd_accepted & CAN2_ACCEPT)
								{
										OSMboxPost(M_can2_cmd_length, &SubCmdLenCAN2);
										//OSTimeDly(6);
										TransLen=*(u16 *)OSMboxPend(M_Image_reply_len, 50, &err);											
										u8temp3=0;
						
										do
										{
												if((TransLen)&&(err==OS_ERR_NONE))
												{
														u8temp3=0;
													  if(Image_reply_packet[0]==0)
														{
																switch(poll_type)
																{
																	case COUNT_TRESH_READ_NOTIFY:
																					triggerSTS=Image_reply_packet[CAN_BUS_ACK_LENTH];
																					trg_assert=(triggerSTS ==0)? 0:1;																	
																		break;
																	default:
																		break;
																}
														}
														break;
												}
												else
												{
													u8temp3++;
													if(u8temp3< MAX_TRY_ERR)
														TransLen=*(u16 *)OSMboxPend(M_Image_reply_len, 50, &err);
												}		
										}while(u8temp3< MAX_TRY_ERR);
								} // end   if(cmd_accepted & CAN2_ACCEPT)
								
								if(trg_assert)
								{
#ifdef __DEBUG_ON_LED		
				led_on(LED2);
				
#endif								
									
										trg_assert=0;
									
										for(k=0;k<4;k++)
										{
												u8temp=1<<k;
											// version 1.3: fix logic & instead of &&, which cause alway run 4 times if no one is 0
												if((u8temp & triggerSTS) & image_mask) // trigger on unmasked channel
												{
														SubCmdLenCAN2=4;
														can2_timeout = integration_ms+40;
														if(image_mode==0)
														{
																	u16temp=12*MAX_TRY_ERR;
															    for(i=0;i<SubCmdLenCAN2;i++)
																			can2_cmd_buf[i]=cmd_read_image12[i];
														}
														else
														{
																	u16temp=24*MAX_TRY_ERR;
																	for(i=0;i<SubCmdLenCAN2;i++)
																			can2_cmd_buf[i]=cmd_read_image24[i];
														}
														can2_cmd_buf[TYP_BYTE_NUM] |= (k<<4);
														Image_Pending=(image_mode==0)? 1:2;
														OSMboxPost(M_can2_cmd_length, &SubCmdLenCAN2);
														
														do
														{
															TransLen=*(u16 *)OSMboxPend(M_Image_reply_len, (u16) can2_timeout, &err);											
															
															if((TransLen)&&(err==OS_ERR_NONE))
															{
																	u8temp2=(image_mode==0)? 12:24;
																	
																	for(i=0;i<u8temp2;i++)
																	{
																			for(j=0;j<64;j++)
																					ImageArrayBuf_trigger[k][i][j]=ImageArrayBuf[0][i][j];
		//																  ImageArrayBuf_trigger[k][i][0] = (u8temp3 | (k<<4));
		//																	ImageArrayBuf_trigger[k][i][1] =  i;
																	}
																	if(image_mode==0)
																			Image12_Pending |= u8temp;
																	else
																			Image24_Pending |= u8temp;
																break;
															}

	
														}while(u16temp--);	
														
													}
											}
								Image_Pending=0;		
								}
		}
		////////////////////////////////
		
		
		//Command_Len=(u8 *)OSMboxPend(M_USB_rsv, 60, &err);
		Command_Len= *(u8 *)OSMboxPend(M_USB_rsv, 60, &err);
		if((Command_Len) && (err==OS_ERR_NONE))
		{
			ACK_reply=0;
			if(PacketChkSum(usb_rcv_buf, Command_Len)==TRUE)
			{
				for(i=0;i<Command_Len;i++)
				{
					Comand_Buf[i]=usb_rcv_buf[i];
					usb_rcv_buf[i]=0;
				}
					//can1_cmd_buf[i]=Comand_Buf[i]=usb_rcv_buf[i];
				ResCode=NO_ERR;
				cmd=Comand_Buf[CMD_BYTE_NUM];
				TxBuffer[0]=type=Comand_Buf[TYP_BYTE_NUM];
				cmd_accepted=NONE_ACCEPT;   // 0-- host processing, 1-- CAN1 processing, 2-- CAN2 processing
				TransLen=1;
				
				switch(Comand_Buf[CMD_BYTE_NUM])
				{
				
					case CMD_GET:
									switch(Comand_Buf[TYP_BYTE_NUM] & 0xF)
									{
											////////// CAN2 task
										
											case TYP_ROW:
											case TYP_IMAGE:
											case TYP_24PIXIMAG:
								#ifdef __OS_DELAY_DEBUG_LED
										
												led_on(LED4);
								#endif												
														u8temp2=(Comand_Buf[TYP_BYTE_NUM] & 0xF);
														u8temp=(Comand_Buf[TYP_BYTE_NUM] & 0xF0)>>4; // channel
														if(u8temp2==TYP_IMAGE)
														{
																if(Image12_Pending & (1<<u8temp))  // buffered already
																{
																		Image12_Pending &= ~(1<<u8temp);
																		triggerSTS &= ~(1<<u8temp);
																		TransLen=(PIX_TOTAL_COL<<1)+6; // reply bytes + total + row
																		for(i=0;i<12;i++)
																		{
																			//Version 1.3: fix packet format
																				USBResFromCAN(&ImageArrayBuf_trigger[u8temp][i][0],TransLen);
																				//USBRes(NO_ERR,CMD_GET, &ImageArrayBuf_trigger[u8temp][i][0],TransLen);
																				OSTimeDly(32);
																		}
																		cmd_accepted=DONE_ACCEPT;   
																		break;
																	
																}
														}
														else if(u8temp2==TYP_24PIXIMAG)
																	{
																			if(Image24_Pending & (1<<u8temp))  // buffered already
																			{
																					Image24_Pending &= ~(1<<u8temp);
																					triggerSTS &= ~(1<<u8temp);
																				  TransLen=(PIX_TOTAL_COL<<2)+6; // reply bytes + total + row
																					for(i=0;i<24;i++)
																					{
																						//Version 1.4:  go on fix on 24x24 of bug in Version 1.3: fix packet format
																							USBResFromCAN(&ImageArrayBuf_trigger[u8temp][i][0],TransLen);
																							//USBRes(NO_ERR,CMD_GET, &ImageArrayBuf_trigger[u8temp][i][0],TransLen);
																							OSTimeDly(32);
																					}
																					cmd_accepted=DONE_ACCEPT;
																					break;
																			}
																	}

																		
																								
																image_mode=((Comand_Buf[TYP_BYTE_NUM] & 0xF) == TYP_24PIXIMAG)? 1:0;
													
																SubCmdLenCAN2=4; // remove head	
																cmd_accepted |= CAN2_ACCEPT;       // CAN2_processing		
																WaitResLenCAN2= CAN_BUS_ACK_LENTH;
													
																if(u8temp2==TYP_ROW)
																		can2_timeout = integration_ms + 5;
																else if(u8temp2==TYP_IMAGE)
																				can2_timeout = integration_ms*12 + 5;
																					else
																						can2_timeout = integration_ms*24 + 5;
													
												break;
											
											////////// CAN2 task end
											
											default:
												break;
										
									}
					
						break;

					case  CMD_SET:
											
								if(Comand_Buf[LEN_BYTE_NUM]<MIN_PACKET_LENGTH)
									ResCode=ERR_LENGTH;
								else
								{
									switch (Comand_Buf[TYP_BYTE_NUM])
									{
										/*
										case INTE_TIME:		//0x20	// special case, will send to both CAN1 & CAN2					
										//case LED_PRO_TIME:
										//case LED_HOLD_TIME:
												SubCmdLenCAN1=8; // remove head	
												cmd_accepted |= CAN1_ACCEPT;       // CAN1_processing
												WaitResLenCAN1 = CAN_BUS_ACK_LENTH;
											break;
										
										//case LED_SWITCH:											 
										case TYP_PCR_TRG_MASK:    // trigger IRQ mask
												SubCmdLenCAN1=5; // remove head	
												cmd_accepted |= CAN1_ACCEPT;       // CAN1_processing		
												WaitResLenCAN1= CAN_BUS_ACK_LENTH;
											break;
										*/
										///////CAN2 task parse
										
										case RAMP_TRIM:
										case RANG_TRIM: 
										case V24_TRIM:
										case V20_TRIM:
										case V15_TRIM:
										case IPIX_TRIM:
										case SWITCH_BIT:
										case TX_PATTERN:
										case AMUX_CONTROL:
										case TEST_ADC:
										case SPI_PULSE:
										case PCR_RESET:	
										case OSC_CONTROL:
										case LED_SWITCH:
										case TYE_TRG_PIX_MODE:
										case TYP_PCR_SEL:
										case TYP_IMG_EN:				
														SubCmdLenCAN2=5; // remove head	
														cmd_accepted |= CAN2_ACCEPT;       // CAN1_processing		
														WaitResLenCAN2= CAN_BUS_ACK_LENTH;
											break;
										
										case LED_PRO_TIME:
										case LED_HOLD_TIME:
														SubCmdLenCAN2=8; // remove head	
														cmd_accepted |= CAN2_ACCEPT;       // CAN1_processing
														WaitResLenCAN2 = CAN_BUS_ACK_LENTH;
											break;
										///////CAN2 task parse end
										
										// special case, will send to both CAN1 & CAN2	
										case INTE_TIME:		
														SubCmdLenCAN1=SubCmdLenCAN2=8; 
														cmd_accepted = CAN1_ACCEPT | CAN2_ACCEPT;       	
														WaitResLenCAN1= WaitResLenCAN2 = CAN_BUS_ACK_LENTH;
														// record inte time for CAN2 timeout
										
														UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
														UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
														UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
														UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];
														if(UnionTemp.float_num!=0)
																integration_ms=UnionTemp.float_num;
											break;
										
										case TYP_PCR_TRG_MASK:    // trigger IRQ mask
											
														image_mask=Comand_Buf[TYP_BYTE_NUM+1];
										
														SubCmdLenCAN1=SubCmdLenCAN2=5; 
														cmd_accepted = CAN1_ACCEPT | CAN2_ACCEPT;       	
														WaitResLenCAN1= WaitResLenCAN2 = CAN_BUS_ACK_LENTH;
											break;
										
										default:  			// invalid type 
												ResCode=ERR_TYP;
											break;
									}
								}
						break;


					case CMD_READ:

								if(Comand_Buf[LEN_BYTE_NUM]<MIN_PACKET_LENGTH)
									ResCode=ERR_LENGTH;
								else
								{
									switch (Comand_Buf[TYP_BYTE_NUM])
									{
										case INTE_TIME:		//0x20
												TransLen=5;
												SubCmdLenCAN1=4; // remove head	
												cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
												WaitResLenCAN1=CAN_BUS_ACK_LENTH + 4; // data length is 4
											break;
										
										
										case TYP_PCR_TRG_MASK:    // trigger IRQ mask
												TransLen=2;
												SubCmdLenCAN1=4; // remove head	
												cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
												WaitResLenCAN1=CAN_BUS_ACK_LENTH + 1; // data length is 1
											break;
										case TYP_VER_INFO:   // get F/W information
										 
												ResCode=NO_ERR;
												TransLen=7;
												TxBuffer[0]=TYP_VER_INFO;
												TxBuffer[1]=FUNC_CODE;		
												TxBuffer[2]=VERSION_INFO_MSB;	
												TxBuffer[3]=VERSION_INFO_LSB;	
												TxBuffer[4]=YEAR_INFO;		//  
												TxBuffer[5]=MONTH_INFO;		//  
												TxBuffer[6]=DATE_INFO;		//  
							 
											break;
									
										///////CAN2 task
										case PCR_TEMP: 
										case LED_PRO_TIME:
										case LED_HOLD_TIME:
														TransLen=2;
														SubCmdLenCAN2=4; // remove head	
														cmd_accepted=CAN2_ACCEPT;       // CAN1_processing	
														WaitResLenCAN2=CAN_BUS_ACK_LENTH + 4; // data length is 4
													break;
										
										case LED_SWITCH:
										case RAMP_TRIM:
										case RANG_TRIM: 
										case V24_TRIM:
										case V20_TRIM:
										case V15_TRIM:
										case IPIX_TRIM:
										case SWITCH_BIT:
										case TX_PATTERN:
										case AMUX_CONTROL:
										case TEST_ADC:
										case SPI_PULSE:
										case PCR_RESET:	
										case OSC_CONTROL:
										case TYE_TRG_PIX_MODE:
										case TYP_PCR_SEL:
										case TYP_IMG_EN:											
														TransLen=2;
														SubCmdLenCAN2=4; // remove head	
														cmd_accepted=CAN2_ACCEPT;       // CAN1_processing	
														WaitResLenCAN2=CAN_BUS_ACK_LENTH + 1; // data length is 1										
											break;
										
										///////CAN2 task end
										
										default:  			// invalid type 
												ResCode=ERR_TYP;
												TransLen=1;
											break;
									}
								}
						break;

					case CMD_MEASURE:

						break;
				
					case CMD_TEMP_SET: // control temperature

							 switch (Comand_Buf[TYP_BYTE_NUM])  // modify @ 140326, add sensor number at byte 1
							 {
								case TEMP_SET_ON:	// float	

										u8temp3=Comand_Buf[TYP_BYTE_NUM+1];	 // #sensor 
										UnionTemp.tempd.byte0 = Comand_Buf[TYP_BYTE_NUM+1+1];
										UnionTemp.tempd.byte1 = Comand_Buf[TYP_BYTE_NUM+2+1];
										UnionTemp.tempd.byte2 = Comand_Buf[TYP_BYTE_NUM+3+1];
										UnionTemp.tempd.byte3 = Comand_Buf[TYP_BYTE_NUM+4+1];

										f32temp1=UnionTemp.float_num;
										if((f32temp1>0)&& (f32temp1<150))
										{
											u8temp1 = Comand_Buf[TYP_BYTE_NUM+5+1];
											u8temp2 = Comand_Buf[TYP_BYTE_NUM+6+1];
											switch (u8temp3)
											{
												case SENSOR_2:
												case SENSOR_1:        //SENSOR_PUMP:  // set the pump temperature for cycle control//txc
														TransLen=1;
														SubCmdLenCAN1=9; // remove head	
														cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
														WaitResLenCAN1=CAN_BUS_ACK_LENTH;
													break;

												default:
														ResCode=BAD_DATA;
													break;
											}

										}
										else
											ResCode=BAD_DATA;
									break;

								case TEMP_SET_OFF:	 		
								
											u8temp3=Comand_Buf[TYP_BYTE_NUM+1];	 // #sensor 
											switch (u8temp3)
											{
												case SENSOR_1:
												case SENSOR_2:
														TransLen=1;
														SubCmdLenCAN1=5; // remove head	
														cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
														WaitResLenCAN1=CAN_BUS_ACK_LENTH;
													break;
													
												default:
														ResCode=BAD_DATA;
													break;
											}
										break;

								case TEMP_READ_SENSOR:  
									
												u8temp3=Comand_Buf[TYP_BYTE_NUM+1];	 // #sensor 
						
												switch (u8temp3)
												{
													case SENSOR_1:
													case SENSOR_2:
																TransLen=5;
																SubCmdLenCAN1=5; // remove head	
																cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
																WaitResLenCAN1=CAN_BUS_ACK_LENTH+4;
														break;
													default:
																ResCode=BAD_DATA;
														break;
												}									
										break;

								case TYP_FAN_CTRL:   // fan control, feature @0512, output on PC5
								case TYP_FAN_READ:
												TransLen=2;
												SubCmdLenCAN1=4; // remove head	
												cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
												WaitResLenCAN1=CAN_BUS_ACK_LENTH+1;
										break;
						
								case TYP_FAN_GAP_SET:
							
												UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
												UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
												UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
												UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];		


												if((UnionTemp.float_num>=0)&&(UnionTemp.float_num<TEMP_GAP_SWAP))
												{
													TransLen=1;
													SubCmdLenCAN1=8; // remove head	
													cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
													WaitResLenCAN1=CAN_BUS_ACK_LENTH;
												}
												else
													ResCode=OUT_RANGE;

									break;
						 
								case TYP_FAN_GAP_READ:
												TransLen=5;
												SubCmdLenCAN1=4; // remove head	
												cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
												WaitResLenCAN1=CAN_BUS_ACK_LENTH+4;
									break;

							
								default:
										ResCode=ERR_TYP;
									break;
							}

						break;					

					case  CMD_PID_CFG: // modify @2014-12-03
								   // Comand_Buf[TYP_BYTE_NUM]	will insert segment index at bit7..bit4
								   // bit3..bit0 still be type
				
									u8temp3= (Comand_Buf[TYP_BYTE_NUM]>>4);
									if (u8temp3>=PID_SLOP_SEG_MAX)
										ResCode=OUT_RANGE;	
									else
									{
										u8temp2=(Comand_Buf[TYP_BYTE_NUM] & 0xf);
										u8temp1=Comand_Buf[LEN_BYTE_NUM];
									
										switch(u8temp2)
										{
												case TYP_PID_KTM:
															if(u8temp3 !=0)
																ResCode=OUT_RANGE; // just only support one point now.
															else
															{
																if(u8temp1 < 4)
																	ResCode=ERR_LENGTH;
																else
																{
																	TransLen=1;
																	SubCmdLenCAN1=8; // remove head	
																	cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
																	WaitResLenCAN1=CAN_BUS_ACK_LENTH;
																}

															}
													break;

												case TYP_PID_KP:
												case TYP_PID_KI:
												case TYP_PID_KD:
												case TYP_PID_KL:													
															if(u8temp1 < 4)
																ResCode=ERR_LENGTH;
															else
															{
																TransLen=1;
																SubCmdLenCAN1=8; // remove head	
																cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
																WaitResLenCAN1=CAN_BUS_ACK_LENTH;
															}

														break;
			
			
												case TYP_PID_KP_KI:
													
																if(u8temp1 < 8)
																	ResCode=ERR_LENGTH;

																else
																{
																	TransLen=1;
																	SubCmdLenCAN1=12; // remove head	
																	cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
																	WaitResLenCAN1=CAN_BUS_ACK_LENTH;
																}

														break;
			
			
												case TYP_PID_KP_KI_KD:
													
																	if(u8temp1 < 12)
																		ResCode=ERR_LENGTH;

																	else
																	{
																		TransLen=1;
																		SubCmdLenCAN1=16; // remove head	
																		cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
																		WaitResLenCAN1=CAN_BUS_ACK_LENTH;
																	}

														break;
			


												default:
																ResCode=ERR_TYP;
													break;
										}
									}
						break; 
				
					case  CMD_PID_READ:
																		
								TransLen=38;
								SubCmdLenCAN1=4; // remove head	
								cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
								WaitResLenCAN1=CAN_BUS_ACK_LENTH+37;
						
						break;
						   
					case CMD_CYCLE_CTRL:
					
								u8temp2=Comand_Buf[TYP_BYTE_NUM];
								u8temp1=Comand_Buf[LEN_BYTE_NUM];
					
								switch(u8temp2)
								{
										case TYP_CYCLE_LOAD:
								
													if((Comand_Buf[SECT_BYTE]<1) || (Comand_Buf[STAGE_BYTE]<1))
														ResCode=OUT_RANGE;
													else
													{
														u8temp=Comand_Buf[STAGE_BYTE];
														Buffer_Cycle_Control.TotalStage= u8temp-1;
													
														for(i=0;i<u8temp;i++)	
														{
															u8temp1=i*BYTE_PER_STAGE;
															Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[SET_START_BYTE+u8temp1];
															Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[SET_START_BYTE+u8temp1+1];
															Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[SET_START_BYTE+u8temp1+2];
															Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[SET_START_BYTE+u8temp1+3];
							
															Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[SET_START_BYTE+u8temp1+4];
															Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[SET_START_BYTE+u8temp1+5];
							
														}
														if(SetPoint_Check(u8temp,Buffer_Cycle_SetPoint)!= VALID)
															ResCode=OUT_RANGE;	
														else		
														{
															TransLen=1;
															SubCmdLenCAN1=4+(u8temp*BYTE_PER_STAGE)+ 3; // remove head	
															cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
															WaitResLenCAN1=CAN_BUS_ACK_LENTH;																			
														}
													}
												
											break;


										case TYP_PE_LOAD: // pump, ext, start inforamtion

													ResCode=NO_ERR;
													for(i=0;i<2;i++)
													{
														u8temp1=i*BYTE_PER_STAGE;  
														Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte0=Comand_Buf[SET_START_BYTE+u8temp1];
														Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte1=Comand_Buf[SET_START_BYTE+u8temp1+1];
														Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte2=Comand_Buf[SET_START_BYTE+u8temp1+2];
														Buffer_Cycle_SetPoint[i].SetPoint.tempd.byte3=Comand_Buf[SET_START_BYTE+u8temp1+3];
						
														Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte1=Comand_Buf[SET_START_BYTE+u8temp1+4];
														Buffer_Cycle_SetPoint[i].SetTime_Union.tempw.byte0=Comand_Buf[SET_START_BYTE+u8temp1+5];
													}

													if(SetPoint_Check(2,Buffer_Cycle_SetPoint)!= VALID)
														ResCode=OUT_RANGE;	
													else
													{
														TransLen=1;
														SubCmdLenCAN1=4+(2*BYTE_PER_STAGE)+ 3; // remove head	
														cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
														WaitResLenCAN1=CAN_BUS_ACK_LENTH;	
														
														////// if start cycle control, special handle host notify
														if(Comand_Buf[CYCLE_SET_BYTE]==1) // comand to start cycle
														{
																// post host_polling_task to check cycle status & host notify
																polling_flag=1;
																TIM3_Int_Init(150,7199);
																IRQ_count=0;
														}
														else
														{
																polling_flag=0;
																TIM3_Stop();
																IRQ_count=0;
														}
													}
														
												break;
											 
											
											case TYP_OVERSHOT_TIM:
												
														ResCode=NO_ERR;
														UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
														UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
														UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
														UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];	
												
														if(UnionTemp.float_num>=0)
														{
															TransLen=1;
															SubCmdLenCAN1=8; // remove head	
															cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
															WaitResLenCAN1=CAN_BUS_ACK_LENTH;																			
														}
														else
															ResCode=OUT_RANGE;

												break;
												
											case TYP_OVERSHOT_TEM:
												
														ResCode=NO_ERR;
														UnionTemp.tempd.byte0= Comand_Buf[TYP_BYTE_NUM+1];
														UnionTemp.tempd.byte1= Comand_Buf[TYP_BYTE_NUM+2];
														UnionTemp.tempd.byte2= Comand_Buf[TYP_BYTE_NUM+3];
														UnionTemp.tempd.byte3= Comand_Buf[TYP_BYTE_NUM+4];	
												
														if((UnionTemp.float_num>=0)&&(UnionTemp.float_num<TEMP_GAP_SWAP))
														{
															TransLen=1;
															SubCmdLenCAN1=8; // remove head	
															cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
															WaitResLenCAN1=CAN_BUS_ACK_LENTH;																			
														}
														else
															ResCode=OUT_RANGE;
						
									
												break;
											
										case TYP_PWM_LIMIT:
													
												break;
													
										default:
														ResCode=ERR_TYP;	
											break;
								}	
					 
					break;

				/////////////////////////////////////////
				case CMD_CYCLE_CTRL_READ:   // add @ 2014-11-30, read out cycle number

								ResCode=NO_ERR;
								TxBuffer[0]=u8temp2=Comand_Buf[TYP_BYTE_NUM];										
								TransLen=1;
					
								switch(u8temp2)
								{
										case TYP_CYCLE_READ:
											
												TransLen=3;
												SubCmdLenCAN1=4; // remove head	
												cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
												WaitResLenCAN1=CAN_BUS_ACK_LENTH+2;															
											break;
										
										case TYP_PWM_READ:
											
												TransLen=7;
												SubCmdLenCAN1=4; // remove head	
												cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
												WaitResLenCAN1=CAN_BUS_ACK_LENTH+6;															
											break;
										
										case TYP_OVERSHOT_TIM:		
										case TYP_OVERSHOT_TEM:
											
													TransLen=5;
													SubCmdLenCAN1=4; // remove head	
													cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
													WaitResLenCAN1=CAN_BUS_ACK_LENTH+4;	
												break;
										
										case CYCLE_STATE_POLL:
																					
													TransLen=2;
										      if(polling_flag==1)
													{		
															TxBuffer[0]=CYCLE_STATE_POLL;
															TxBuffer[1]=cycleSTS;
															USBRes(NO_ERR, cmd, TxBuffer, TransLen);
														  OSTimeDly(32);
															cmd_accepted=DONE_ACCEPT; 
													}
													else
													{
															SubCmdLenCAN1=4; // remove head	
															cmd_accepted=CAN1_ACCEPT;       // CAN1_processing	
															WaitResLenCAN1=CAN_BUS_ACK_LENTH+1;																			
													}
											break;
										
										case TYP_PWM_LIMIT:
													
												break;
												
										default:
														ResCode=ERR_TYP;
											break;
								}
						break;

#ifdef __USE_USB_BUS
#ifdef __USE_RAM_DEBUG

				case CMD_DEBUG:
					
						ResCode=NO_ERR;
						TxBuffer[0]=u8temp2=Comand_Buf[TYP_BYTE_NUM];
				
						switch(u8temp2)
						{
							case TYP_RAM_DUMP:  // dump the RAM

											if(RamDumpFlag==2)
											{
													u8temp= DUMPSIZE/SIZE_PER_PACKET;
													u8temp=(DUMPSIZE%SIZE_PER_PACKET)? (u8temp+1):u8temp;
													TransLen=SIZE_PER_PACKET;
													TxBuffer[0]=TYP_RAM_DUMP;
													TxBuffer[1]=RAM_Dump_index/SIZE_PER_PACKET;
													TxBuffer[1]=(RAM_Dump_index%SIZE_PER_PACKET)? (TxBuffer[1]+1):TxBuffer[1];
											
													u16temp=0;
													for(i=0;i<u8temp;i++)
													{
															TxBuffer[2]=i+1;
															for(j=0;j<SIZE_PER_PACKET;j++)
																	TxBuffer[j+3]=RamDumpBlock[u16temp++];

															USBRes(ResCode, CMD_DEBUG, TxBuffer, TransLen+3);
															delay_ms(32);
													}
													RamDumpFlag=TickDump=0;  // unlock ram to dump again;
													TRG_OUT=0;
											}
											else
											{
													ResCode=BAD_DATA;
												  TransLen=1;
												  USBRes(ResCode, CMD_DEBUG, TxBuffer, TransLen);
													OSTimeDly(32);
											}
								break;					
							default:
								break;
						}
					break;

#endif
#endif		

						
				case CMD_HOST_NOTIFY:
							
							ResCode=NO_ERR;
							TransLen=2;
							if(polling_flag)
							{
									TxBuffer[0]=type;
									TxBuffer[1]=triggerSTS;
									//USBRes(ResCode, cmd, TxBuffer, TransLen);
								  //OSTimeDly(32);
								//// version 1.2: 170318 -- add triggerSTS clearing after reading
									triggerSTS=0;

									break;
							}
							SubCmdLenCAN2=4; // remove head	
							cmd_accepted=CAN2_ACCEPT; //DONE_ACCEPT      
							WaitResLenCAN2=CAN_BUS_ACK_LENTH+2;		
					break;
				
				default:
							ResCode=ERR_CMD;
					break;
			
			}
			ACK_reply=0;
			
			if(cmd_accepted == NONE_ACCEPT)
			{
				USBRes(ResCode, cmd, TxBuffer, TransLen);
				OSTimeDly(32);
			}
			else
			{
				if(cmd_accepted & CAN1_ACCEPT)
				{
					for(i=0;i<SubCmdLenCAN1;i++)
								can1_cmd_buf[i]=Comand_Buf[i];
					OSMboxPost(M_can1_cmd_length, &SubCmdLenCAN1);
					// OSTimeDlyHMSM(0,0,0,100);
					TransLen=*(u8 *)OSMboxPend(M_Temper_reply_len, 30, &err);											
					u8temp3=0;
					
				//	if(WaitResLenCAN1 != TransLen) // how about length received match expect
				//	{}
					do
					{
						if((TransLen)&&(err==OS_ERR_NONE))
						{
							if(!(cmd_accepted & CAN2_ACCEPT))  // case: both CAN1 & CAN2 reply
							{
									USBResFromCAN(Temper_reply_packet,TransLen);
									OSTimeDly(32);
							}   
							u8temp3=0;
							//ACK_reply |= CAN1_ACCEPT;
							break;
						}
						else
						{
							u8temp3++;
							//if(u8temp3< MAX_TRY_ERR) 
								TransLen=*(u8 *)OSMboxPend(M_Temper_reply_len, 30, &err);	
						}		
					}while(u8temp3< MAX_TRY_ERR);
					
					if(u8temp3)  // no reply from target device
					{
						ResCode=ERR_CAN1;
						TxBuffer[0]= type;
						TransLen=1;
						USBRes(ResCode, cmd, TxBuffer, TransLen);
						OSTimeDly(32);
						continue;  // if error, ignore to post CAN2
					}
				}
				
				if(cmd_accepted & CAN2_ACCEPT)
				{
						for(i=0;i<SubCmdLenCAN2;i++)
									can2_cmd_buf[i]=Comand_Buf[i];
						u8temp3=0;
						u8temp2=(type & 0xF);
					  if((cmd==CMD_GET) && ((u8temp2==TYP_IMAGE)||(u8temp2==TYP_24PIXIMAG)))
						{
							Image_Pending=(u8temp2==TYP_IMAGE)? 1:2;
								u16temp=MAX_TRY_ERR*24;
						}
						else
						{
								Image_Pending=0;
								u16temp=MAX_TRY_ERR;
						}
						OSMboxPost(M_can2_cmd_length, &SubCmdLenCAN2);
						//OSTimeDly(10);
							
						TransLen=*(u16 *)OSMboxPend(M_Image_reply_len, (u16) can2_timeout, &err);											

						do
						{
							if((TransLen)&&(err==OS_ERR_NONE))
							{
								//if((cmd==CMD_GET) && ((type==TYP_IMAGE)||(type==TYP_24PIXIMAG)))
								if((cmd==CMD_GET) && ((u8temp2==TYP_IMAGE)||(u8temp2==TYP_24PIXIMAG)))
								{
									u8temp=(u8temp2==TYP_IMAGE)? 12:24;
									//if(TransLen>64)
									{
											for(j=0;j<u8temp;j++)
											{
													//USBResFromCAN(&ImageArrayBuf[0][i][0],TransLen);
													USBResFromCAN(&ImageArrayBuf[0][j][0],64);
													OSTimeDly(32);
											}
											u8temp3=0;
											break;
									}
									/*
									else
									{
													USBResFromCAN(&Image_reply_packet[0],TransLen);
													OSTimeDly(32);									
									}
									*/
								}	
								else
								{
									USBResFromCAN(Image_reply_packet,TransLen);
									//OSTimeDly(32);
								}
								u8temp3=0;
								//ACK_reply=1;
								break;
							}
							else
							{
								u8temp3++;
								if(u8temp3< u16temp) 
									TransLen=*(u16 *)OSMboxPend(M_Image_reply_len, (u16) can2_timeout, &err);	
							}		
						}while(u8temp3< u16temp);
						
						if(Image_Pending)   	//v1.10 @20170517 to clear flag
								Image_Pending=0;	//in case that next is IRQ count trigger to read
																	// in CAN2 interrupt still see image cmd is pended.
						
						if(u8temp3)  // no reply from target device
						{
							ResCode=ERR_CAN2;
							TxBuffer[0]= type;
							TransLen=1;
				#ifdef __OS_DELAY_DEBUG_LED
								led_off(LED4);
				#endif							
							
							USBRes(ResCode, cmd, TxBuffer, TransLen);
							OSTimeDly(32);
						}
						can2_timeout=10;
				} // end   if(cmd_accepted & CAN2_ACCEPT)
			}		// end   if(cmd_accepted == NONE_ACCEPT)
		}			// end   if(PacketChkSum(usb_rcv_buf, Command_Len)==TRUE)							
////////////////////////////////////////					
//	USBD_HID_SendReport (&USB_OTG_dev, buf,HID_INOUT_BYTES);	
		}			// end   if((Command_Len) && (err==OS_ERR_NONE))
	}       // end 	while(1)
}
static void startup_task(void *p_arg)
{ 
  systick_init();     /* Initialize the SysTick. */
	Right_time=OSQCreate((void*)&RightQueueTbl[0],20);		                 //建立消息队列
	msg_can1_rcv_length=OSMboxCreate((void*)0);
	msg_can2_rcv_length=OSMboxCreate((void*)0);
	//Sem_CAN1_AVL=OSSemCreate(3);   // creat Semphore for CAN_1 (temp control bus)
	M_Temper_reply_len=OSMboxCreate((void*)0);
	M_Image_reply_len=OSMboxCreate((void*)0);
	
//	M_can1_wait_length=OSMboxCreate((void*)0);
	M_can1_cmd_length=OSMboxCreate((void*)0);
	
//	M_can2_wait_length=OSMboxCreate((void*)0);
	M_can2_cmd_length=OSMboxCreate((void*)0);	
	
	M_USB_rsv=OSMboxCreate((void*)0);
	
	M_POLL_flag=OSMboxCreate((void*)0);
	/*创建新任务*/
	//OSTaskCreate(led1_task, 0, &led1_task_stk[LED1_TASK_STK_SIZE - 1], LED1_TASK_PRIO); 
	OSTaskCreate(CAN1_task, 0, &CAN1_task_stk[CAN1_TASK_STK_SIZE - 1], CAN1_TASK_PRIO);
	OSTaskCreate(CAN2_task, 0, &CAN2_task_stk[CAN2_TASK_STK_SIZE - 1], CAN2_TASK_PRIO);
	//OSTaskCreate(CMD_task, 	0, &CMD_task_stk [CMD_TASK_STK_SIZE - 1], CMD_TASK_PRIO);
	OSTaskCreate(USB_task, 	0, &USB_task_stk [USB_TASK_STK_SIZE - 1], USB_TASK_PRIO);
	while(1) 
	{
		/*LED2以1s频率闪烁*/
	 	led_on(LED1);
	 	OSTimeDlyHMSM(0,0,0,5);   //500ms延时，释放CPU控制权
	 	led_off(LED1);
	 	OSTimeDlyHMSM(0,0,0, 5);   //500ms延时，释放CPU控制权
	}        
}


int main(void) 
{ 
    BSP_Init();

		OSInit(); 
	// g_TxMbox=OSMboxCreate((void*)0); //创建全局信号-消息邮箱
    OSTaskCreate(startup_task, (void *)0, 
          &startup_task_stk[STARTUP_TASK_STK_SIZE - 1], 
          STARTUP_TASK_PRIO); 
     OSStart(); 
     return 0; 
}

///////////////////////////////////
// save in buffer with max limit, 
// input: *buffer, Max size limit in case overflow, index(actual size of received) for output
// 				u16 time_out  --to cover image integration time, enlong the time out
// return: error code- 0: NO error, 1: timeout, 2, overflow
//////////////////////////////////

static u8 CAN2_Receive_Function(u8 * RcvBuf, u8 maxSize, u16 * index, u16 time_out) 
{																												
		CanTxMsg TxMessage;
		CanRxMsg Can2_buf;
		u8 Flag_CAN2_READ, Flag_CAN2_replied, u8Line;
		INT8U err, error_cnt;
		u8 Can2_rcv_len,i,j,k,index_rcv_CAN, Next_expect;
		Next_expect=k=j=1;
		index_rcv_CAN=0;
		err=NO_ERR;
		*index=0;
	  error_cnt=0;
		do
		{
				Can2_rcv_len=*(u8 *)OSMboxPend(msg_can2_rcv_length,time_out,&err);	
																												
				if((Can2_rcv_len) && (err==OS_ERR_NONE))				
				{
						error_cnt=0;
					  if(Image_Pending)
						{
								u8Line=(Image_Pending==1)? 12:24;
								//Next_expect= (Can2_rcv_len & 0x80)? 1:0;
								Can2_rcv_len &=0x7F;
								for(j=0;j<u8Line;j++)
								{
									for(i=0; i<Can2_rcv_len; i++)
									{
										*(RcvBuf+i) = DualBuffer[j][i];
									}	
									RcvBuf +=64;
								}
								break;
						}
						else
						{
									for(i=0; i<Can2_rcv_len; i++)
									{
										*RcvBuf = MonoBuffer[i];
										RcvBuf++;
									}
									break;
									
						}
				}
				else										// no replay
						error_cnt++;

		}while(error_cnt < (MAX_TRY_ERR*12));
		
		
		
		if(error_cnt)
		{
			err=TM_OUT;
		}
		else
			*index=Can2_rcv_len;
		
		return err;
}



///////////////////////////////////
// send from buffer with size set, 
// input: *buffer, size to send, u16 time_out -- think for integration time 
// return: error code- 0: NO error, 1: timeout, 3, bus NACK
//////////////////////////////////
static u8 CAN2_Send_Function(u8 * TxBuf, u8 Size) 
{																												
		CanTxMsg TxMessage;
		CanRxMsg Can2_buf;
		INT8U err, error_cnt;
		u8 Can2_rcv_len,i,j,k,index_txm_CAN,u8temp;
	
		if(Size==0) return NO_ERR;
	
		k=(Size/8);
		u8temp=Size%8;
		k=(u8temp>0)? (k+1):k; // packet number
		j= (k==1)? Size:(Size%8);
	
		index_txm_CAN=1;
		err=NO_ERR;

		TxMessage.StdId = IMAGE_FUNC_ID; 	//  2 = to function 0x2  
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE = CAN_ID_STD;

//		Can2_rcv_len=*(u8 *)OSMboxPend(msg_can2_rcv_length,10,&err);		
	
		if(k==1)
		{
				TxMessage.StdId = 0x211;
				for (i=1;i<Size;i++)
						TxMessage.Data[i-1]=TxBuf[i];  //cmd, len, type, sensor, float temper, uint16 time
				TxMessage.DLC = Size-1;
				CAN_SendData(CAN2, &TxMessage);
				return NO_ERR;
		}
		else
		{
				for(u8temp=0;u8temp<k;u8temp++)
				{
						if(u8temp<(k-1))
						{
								for (i=0;i<8;i++)
									TxMessage.Data[i]=TxBuf[index_txm_CAN++];  
								TxMessage.DLC=8;
							  TxMessage.StdId = IMAGE_FUNC_ID | (k<<4)| (u8temp+1);
								TxMessage.RTR = CAN_RTR_DATA;
								TxMessage.IDE = CAN_ID_STD;
								do
								{
										CAN_SendData(CAN2, &TxMessage);
										Can2_rcv_len=* (u8 *)OSMboxPend(msg_can2_rcv_length,10,&err);	
										if((Can2_rcv_len)&&(err==OS_ERR_NONE))
										{
												//if((RxMessage.StdId==TxMessage.StdId) && (RxMessage.Data[0]== NO_ERR))
												if(RxMessage_IM.Data[0]== NO_ERR)
												{
													error_cnt=0;
													break;
												}
												error_cnt++;
										}
										else error_cnt++;
								} while(error_cnt<MAX_TRY_ERR);
							
								if(error_cnt) 
									return TM_OUT;								
						}
						else
						{
								for (i=0;i<j;i++)
									TxMessage.Data[i]=TxBuf[index_txm_CAN++]; 
								TxMessage.StdId = IMAGE_FUNC_ID | (k<<4)| (u8temp+1);
								TxMessage.RTR = CAN_RTR_DATA;
								TxMessage.IDE = CAN_ID_STD;
							  TxMessage.DLC=j;
							  CAN_SendData(CAN2, &TxMessage);
								return NO_ERR;
						}
				}
		}
}



static void CAN2_task(void *p_arg)
{   
		CanTxMsg TxMessage;
		CanRxMsg Can2_buf;
		u8 Flag_CAN2_READ, Flag_CAN2_replied,debug_flag;
		INT8U err, error_cnt;
		u8 Can2_rcv_len,i,j,k,index_rcv_CAN, index_send_CAN, Image_cmd_length, u8temp,u8temp1;
		u16 Image_packet_len;
		u8 command, type;
		float integration_time[4];  // catch of 4 channel integratio time
		u16 time_out=30;
		p_arg=p_arg;      //防止编译器产生警告

		CAN2_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  
		index_rcv_CAN=index_send_CAN=Image_packet_len=error_cnt=0;	 
debug_flag=0;
		while(1)
		{
			Image_cmd_length =*(u8 *)OSMboxPend(M_can2_cmd_length,60,&err); 
			if((Image_cmd_length) && (err==OS_ERR_NONE))
			{
						command =	can2_cmd_buf[CMD_BYTE_NUM];
						type	= 	can2_cmd_buf[TYP_BYTE_NUM];
				/////////version 1.5 fix column number error
				    u8temp1= (type & 0xF);
						err=CAN2_Send_Function(can2_cmd_buf,Image_cmd_length);
						if(!err)
						{
							/////////version 1.5 fix column number error
							//if((command==CMD_GET) && ((type==TYP_IMAGE)||(type==TYP_24PIXIMAG))) 
							if((command==CMD_GET) && ((u8temp1==TYP_IMAGE)||(u8temp1==TYP_24PIXIMAG))) 
							{
								
								#ifdef __OS_DELAY_DEBUG_LED
									led_on(LED3);
								#endif
									/////////version 1.5 fix column number error
									//u8temp=(type==TYP_IMAGE)? 12:24;
								  u8temp=(u8temp1==TYP_IMAGE)? 12:24;
								  
									//for(i=0;i<u8temp;i++)
									{
										
										err=CAN2_Receive_Function(&ImageArrayBuf[0][0][0], 64, &Image_packet_len, can2_timeout);
										if(err) 
												u8temp=0;
								#ifdef __OS_DELAY_DEBUG_LED
										else
												led_off(LED3);
								#endif
										

									}
									Image_packet_len=u8temp*56;
							}
							else
									err=CAN2_Receive_Function(Image_reply_packet, 64, &Image_packet_len, can2_timeout);
							
							if(err)
							{
								#ifdef  __DEBUG_ON_LED
									if(command==CMD_HOST_NOTIFY) 
									{
											if(debug_flag==0)
											{led_on(LED4);debug_flag=1;}
											else
											{led_off(LED4);debug_flag=0;}M_Image_reply_len
									}
								#endif 
									Image_reply_packet[0]=ERR_CAN2;
									Image_reply_packet[1]=command;
									Image_reply_packet[2]=1;
									Image_reply_packet[3]=type;		
									Image_packet_len=4;
							}
							OSMboxPost(M_Image_reply_len, (void *) &Image_packet_len);
					}
			}
		}
}
