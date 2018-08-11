#include <includes.h>
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
static  void  BSP_LED_Init(void);
void TIM3_PWM_Init(void);
void All_GPIO_Init(void);
void uart_init(u32 bound);
void NVIC_Configuration(void);
void CAN_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);
//	static  void  BSP_KEY_Init (void); 

void  BSP_Init (void)
{
	SystemInit();
	BSP_LED_Init();                 /* Initialize the LED  */
	All_GPIO_Init(); 
//	uart_init(115200);
	CAN_Config();
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); 
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
    NVIC_Configuration();
}


void NVIC_Configuration(void)
{

    //EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  	/* Configure the NVIC Preemption Priority Bits 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1	 */ 

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		//	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQí¨μàê1?ü
	NVIC_Init(&NVIC_InitStructure);	//?ù?YNVIC_InitStruct?D???¨μ?2?êy3?ê??ˉíaéèNVIC??′??÷USART2

	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01 ;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQí¨μàê1?ü
	NVIC_Init(&NVIC_InitStructure);	//?ù?YNVIC_InitStruct?D???¨μ?2?êy3?ê??ˉíaéèNVIC??′??÷USART2	

}
void All_GPIO_Init(){
	u16 i;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOA ,ENABLE);


   /* Configure CAN1 RX pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   
   /* Configure CAN2 RX pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure CAN1 TX pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   /* Configure CAN2 TX pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Remap CAN1 and CAN2 GPIOs */
   GPIO_PinRemapConfig(GPIO_Remap2_CAN1 , ENABLE);
   GPIO_PinRemapConfig(GPIO_Remap_CAN2, ENABLE);

#ifdef __QPCR_HW
		 //RST_IM, PC0  RST_TEMP,PC1
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		 GPIO_Init(GPIOC, &GPIO_InitStructure);

		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		 GPIO_Init(GPIOC, &GPIO_InitStructure);

		 GPIO_SetBits(GPIO_PORT_RST, GPIO_Pin_RST_IM);
		 GPIO_SetBits(GPIO_PORT_RST, GPIO_Pin_RST_TM);
		 for(i=0;i<200;i++);
		 GPIO_ResetBits(GPIO_PORT_RST, GPIO_Pin_RST_IM);
		 GPIO_ResetBits(GPIO_PORT_RST, GPIO_Pin_RST_TM);
		 for(i=0;i<20000;i++);
		 GPIO_SetBits(GPIO_PORT_RST, GPIO_Pin_RST_IM);
		 GPIO_SetBits(GPIO_PORT_RST, GPIO_Pin_RST_TM);
#else
		 //USART1_TX   PA.9
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		 GPIO_Init(GPIOA, &GPIO_InitStructure);
		 
			//USART1_RX	  PA.10
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		 GPIO_Init(GPIOA, &GPIO_InitStructure); 
#endif
 
}

/*void uart_init(u32 bound){
    //GPIO端口设置
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
   //USART 初始化设置
   
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
   

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
   
    USART_Cmd(USART1, ENABLE);                    //使能串口 

}
 */
static  void BSP_LED_Init(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_GPIO_PORT_LED, ENABLE);  //使能时钟 //根据自己板子情况修改
#ifdef __QPCR_HW
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_led1|GPIO_Pin_led2;
#else
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_led1|GPIO_Pin_led2|GPIO_Pin_led3|GPIO_Pin_led4; //根据自己板子情况修改
#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
	GPIO_Init(GPIO_PORT_LED, &GPIO_InitStructure); //根据自己板子情况修改
	led_off(LED1);led_off(LED2);led_off(LED3);led_off(LED4);
} 

void led_on(uint32_t n) 
{ 
	switch (n) 
	{ 
		case LED1:  GPIO_ResetBits(GPIO_PORT_LED, GPIO_Pin_led1); 	break; 
		case LED2:  GPIO_ResetBits(GPIO_PORT_LED, GPIO_Pin_led2); 	break; 
#ifndef __QPCR_HW
		case LED3:  GPIO_ResetBits(GPIO_PORT_LED, GPIO_Pin_led3); 	break; 
		case LED4:  GPIO_ResetBits(GPIO_PORT_LED, GPIO_Pin_led4); 	break; 
#endif
		default: 	break; 
	} 
} 
 
void led_off(uint32_t n)
{ 
switch (n) 
{ 
    	case LED1:  GPIO_SetBits(GPIO_PORT_LED, GPIO_Pin_led1);   break; 
    	case LED2:  GPIO_SetBits(GPIO_PORT_LED, GPIO_Pin_led2);   break;  
#ifndef __QPCR_HW
    	case LED3:  GPIO_SetBits(GPIO_PORT_LED, GPIO_Pin_led3);   break; 
    	case LED4:  GPIO_SetBits(GPIO_PORT_LED, GPIO_Pin_led4);   break;  
#endif
    	default:  break; 
} 
}
void TIM3_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
  	const uint16_t Period_Val = 1020;
	const uint16_t Period_Ori = 1;
    __IO uint16_t CCR1_Val = Period_Val/2;
    __IO uint16_t CCR2_Val = Period_Val-20;
    uint16_t PrescalerValue = 0;
     GPIO_InitTypeDef GPIO_InitStructure;    
    /* System Clocks Configuration */ /* PCLK1 = HCLK/2 = 36MHz */
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

     
 
    /* GPIOA and GPIOB clock enable */ 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  
 
    /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */ 
   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7; 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           // 复用推挽输出 
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
   GPIO_Init(GPIOA, &GPIO_InitStructure); 

    /* -----------------------------------------------------------------------
      TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
      The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
      clock at 24 MHz the Prescaler is computed as following:
       - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
      SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
      and Connectivity line devices and to 24 MHz for Low-Density Value line and
      Medium-Density Value line devices
  
      The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                    = 24 MHz / 1000 = 24 KHz
      TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
      TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    ----------------------------------------------------------------------- */
    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = Period_Val;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 200;				  //占空比 plus/period * 100%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 200;

    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* TIM IT enable */
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);

    //TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);			  
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
  //  NVIC_TIM3Configuration();
}
void CAN_Config(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOB, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);

  
  //CAN1 cell init
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStructure.CAN_Prescaler = 12;
  CAN_Init(CAN1, &CAN_InitStructure);
  CAN_Init(CAN2, &CAN_InitStructure);

  //* CAN1 filter init
  CAN_FilterInitStructure.CAN_FilterNumber = 1;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x6420;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  //* CAN2 filter init 
  CAN_FilterInitStructure.CAN_FilterNumber = 14;
  CAN_FilterInit(&CAN_FilterInitStructure);

  
}

/**
  * @brief  Initializes a Rx Message.
  * @param  CanRxMsg *RxMessage
  * @retval None
  */

void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t i = 0;

  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (i = 0; i < 8; i++)
  {
    RxMessage->Data[i] = 0x00;
  }
}


