#line 1 "Temperature.c"
#line 1 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\SYSTEM\\sys\\stm32f10x_map.h"














 

 







 
#line 1 "..\\SYSTEM\\sys\\stm32f10x_conf.h"













 

 



 
#line 1 "..\\SYSTEM\\sys\\stm32f10x_type.h"














 

 



 
 
typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long  const sc32;   
typedef signed short const sc16;   
typedef signed char  const sc8;    

typedef volatile signed long  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long  const vsc32;   
typedef volatile signed short const vsc16;   
typedef volatile signed char  const vsc8;    

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;   
typedef unsigned short const uc16;   
typedef unsigned char  const uc8;    

typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long  const vuc32;   
typedef volatile unsigned short const vuc16;   
typedef volatile unsigned char  const vuc8;    

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

#line 73 "..\\SYSTEM\\sys\\stm32f10x_type.h"

 
 
 



 
#line 22 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 
 


 
 

 
 





 


 


 


 


 


 
#line 66 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 


 



 


 


 
#line 90 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 




 


 


 


 


 


 


 





 


 
#line 133 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 
#line 141 "..\\SYSTEM\\sys\\stm32f10x_conf.h"

 



 


 
#line 167 "..\\SYSTEM\\sys\\stm32f10x_conf.h"



 
#line 27 "..\\SYSTEM\\sys\\stm32f10x_map.h"
#line 28 "..\\SYSTEM\\sys\\stm32f10x_map.h"
#line 1 "..\\SYSTEM\\sys\\cortexm3_macro.h"













 

 



 
#line 22 "..\\SYSTEM\\sys\\cortexm3_macro.h"

 
 
 
 
void __WFI(void);
void __WFE(void);
void __SEV(void);
void __ISB(void);
void __DSB(void);
void __DMB(void);
void __SVC(void);
u32 __MRS_CONTROL(void);
void __MSR_CONTROL(u32 Control);
u32 __MRS_PSP(void);
void __MSR_PSP(u32 TopOfProcessStack);
u32 __MRS_MSP(void);
void __MSR_MSP(u32 TopOfMainStack);
void __RESETPRIMASK(void);
void __SETPRIMASK(void);
u32 __READ_PRIMASK(void);
void __RESETFAULTMASK(void);
void __SETFAULTMASK(void);
u32 __READ_FAULTMASK(void);
void __BASEPRICONFIG(u32 NewPriority);
u32 __GetBASEPRI(void);
u16 __REV_HalfWord(u16 Data);
u32 __REV_Word(u32 Data);



 
#line 29 "..\\SYSTEM\\sys\\stm32f10x_map.h"

 
 
 
 

 
typedef struct
{
  vu32 SR;
  vu32 CR1;
  vu32 CR2;
  vu32 SMPR1;
  vu32 SMPR2;
  vu32 JOFR1;
  vu32 JOFR2;
  vu32 JOFR3;
  vu32 JOFR4;
  vu32 HTR;
  vu32 LTR;
  vu32 SQR1;
  vu32 SQR2;
  vu32 SQR3;
  vu32 JSQR;
  vu32 JDR1;
  vu32 JDR2;
  vu32 JDR3;
  vu32 JDR4;
  vu32 DR;
} ADC_TypeDef;

 
typedef struct
{
  u32  RESERVED0;
  vu16 DR1;
  u16  RESERVED1;
  vu16 DR2;
  u16  RESERVED2;
  vu16 DR3;
  u16  RESERVED3;
  vu16 DR4;
  u16  RESERVED4;
  vu16 DR5;
  u16  RESERVED5;
  vu16 DR6;
  u16  RESERVED6;
  vu16 DR7;
  u16  RESERVED7;
  vu16 DR8;
  u16  RESERVED8;
  vu16 DR9;
  u16  RESERVED9;
  vu16 DR10;
  u16  RESERVED10; 
  vu16 RTCCR;
  u16  RESERVED11;
  vu16 CR;
  u16  RESERVED12;
  vu16 CSR;
  u16  RESERVED13[5];
  vu16 DR11;
  u16  RESERVED14;
  vu16 DR12;
  u16  RESERVED15;
  vu16 DR13;
  u16  RESERVED16;
  vu16 DR14;
  u16  RESERVED17;
  vu16 DR15;
  u16  RESERVED18;
  vu16 DR16;
  u16  RESERVED19;
  vu16 DR17;
  u16  RESERVED20;
  vu16 DR18;
  u16  RESERVED21;
  vu16 DR19;
  u16  RESERVED22;
  vu16 DR20;
  u16  RESERVED23;
  vu16 DR21;
  u16  RESERVED24;
  vu16 DR22;
  u16  RESERVED25;
  vu16 DR23;
  u16  RESERVED26;
  vu16 DR24;
  u16  RESERVED27;
  vu16 DR25;
  u16  RESERVED28;
  vu16 DR26;
  u16  RESERVED29;
  vu16 DR27;
  u16  RESERVED30;
  vu16 DR28;
  u16  RESERVED31;
  vu16 DR29;
  u16  RESERVED32;
  vu16 DR30;
  u16  RESERVED33; 
  vu16 DR31;
  u16  RESERVED34;
  vu16 DR32;
  u16  RESERVED35;
  vu16 DR33;
  u16  RESERVED36;
  vu16 DR34;
  u16  RESERVED37;
  vu16 DR35;
  u16  RESERVED38;
  vu16 DR36;
  u16  RESERVED39;
  vu16 DR37;
  u16  RESERVED40;
  vu16 DR38;
  u16  RESERVED41;
  vu16 DR39;
  u16  RESERVED42;
  vu16 DR40;
  u16  RESERVED43;
  vu16 DR41;
  u16  RESERVED44;
  vu16 DR42;
  u16  RESERVED45;    
} BKP_TypeDef;

 
typedef struct
{
  vu32 TIR;
  vu32 TDTR;
  vu32 TDLR;
  vu32 TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct
{
  vu32 RIR;
  vu32 RDTR;
  vu32 RDLR;
  vu32 RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  vu32 FR1;
  vu32 FR2;
} CAN_FilterRegister_TypeDef;

typedef struct
{
  vu32 MCR;
  vu32 MSR;
  vu32 TSR;
  vu32 RF0R;
  vu32 RF1R;
  vu32 IER;
  vu32 ESR;
  vu32 BTR;
  u32  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  u32  RESERVED1[12];
  vu32 FMR;
  vu32 FM1R;
  u32  RESERVED2;
  vu32 FS1R;
  u32  RESERVED3;
  vu32 FFA1R;
  u32  RESERVED4;
  vu32 FA1R;
  u32  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

 
typedef struct
{
  vu32 DR;
  vu8  IDR;
  u8   RESERVED0;
  u16  RESERVED1;
  vu32 CR;
} CRC_TypeDef;


 
typedef struct
{
  vu32 CR;
  vu32 SWTRIGR;
  vu32 DHR12R1;
  vu32 DHR12L1;
  vu32 DHR8R1;
  vu32 DHR12R2;
  vu32 DHR12L2;
  vu32 DHR8R2;
  vu32 DHR12RD;
  vu32 DHR12LD;
  vu32 DHR8RD;
  vu32 DOR1;
  vu32 DOR2;
} DAC_TypeDef;

 
typedef struct
{
  vu32 IDCODE;
  vu32 CR;	
}DBGMCU_TypeDef;

 
typedef struct
{
  vu32 CCR;
  vu32 CNDTR;
  vu32 CPAR;
  vu32 CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  vu32 ISR;
  vu32 IFCR;
} DMA_TypeDef;

 
typedef struct
{
  vu32 IMR;
  vu32 EMR;
  vu32 RTSR;
  vu32 FTSR;
  vu32 SWIER;
  vu32 PR;
} EXTI_TypeDef;

 
typedef struct
{
  vu32 ACR;
  vu32 KEYR;
  vu32 OPTKEYR;
  vu32 SR;
  vu32 CR;
  vu32 AR;
  vu32 RESERVED;
  vu32 OBR;
  vu32 WRPR;
} FLASH_TypeDef;

typedef struct
{
  vu16 RDP;
  vu16 USER;
  vu16 Data0;
  vu16 Data1;
  vu16 WRP0;
  vu16 WRP1;
  vu16 WRP2;
  vu16 WRP3;
} OB_TypeDef;

 
typedef struct
{
  vu32 BTCR[8];   
} FSMC_Bank1_TypeDef; 

typedef struct
{
  vu32 BWTR[7];
} FSMC_Bank1E_TypeDef;

typedef struct
{
  vu32 PCR2;
  vu32 SR2;
  vu32 PMEM2;
  vu32 PATT2;
  u32  RESERVED0;   
  vu32 ECCR2; 
} FSMC_Bank2_TypeDef;  

typedef struct
{
  vu32 PCR3;
  vu32 SR3;
  vu32 PMEM3;
  vu32 PATT3;
  u32  RESERVED0;   
  vu32 ECCR3; 
} FSMC_Bank3_TypeDef; 

typedef struct
{
  vu32 PCR4;
  vu32 SR4;
  vu32 PMEM4;
  vu32 PATT4;
  vu32 PIO4; 
} FSMC_Bank4_TypeDef; 

 
typedef struct
{
  vu32 CRL;
  vu32 CRH;
  vu32 IDR;
  vu32 ODR;
  vu32 BSRR;
  vu32 BRR;
  vu32 LCKR;
} GPIO_TypeDef;

typedef struct
{
  vu32 EVCR;
  vu32 MAPR;
  vu32 EXTICR[4];
} AFIO_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 OAR1;
  u16  RESERVED2;
  vu16 OAR2;
  u16  RESERVED3;
  vu16 DR;
  u16  RESERVED4;
  vu16 SR1;
  u16  RESERVED5;
  vu16 SR2;
  u16  RESERVED6;
  vu16 CCR;
  u16  RESERVED7;
  vu16 TRISE;
  u16  RESERVED8;
} I2C_TypeDef;

 
typedef struct
{
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

 
typedef struct
{
  vu32 ISER[2];
  u32  RESERVED0[30];
  vu32 ICER[2];
  u32  RSERVED1[30];
  vu32 ISPR[2];
  u32  RESERVED2[30];
  vu32 ICPR[2];
  u32  RESERVED3[30];
  vu32 IABR[2];
  u32  RESERVED4[62];
  vu32 IPR[15];
} NVIC_TypeDef;

typedef struct
{
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CSR;
} PWR_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_TypeDef;

 
typedef struct
{
  vu16 CRH;
  u16  RESERVED0;
  vu16 CRL;
  u16  RESERVED1;
  vu16 PRLH;
  u16  RESERVED2;
  vu16 PRLL;
  u16  RESERVED3;
  vu16 DIVH;
  u16  RESERVED4;
  vu16 DIVL;
  u16  RESERVED5;
  vu16 CNTH;
  u16  RESERVED6;
  vu16 CNTL;
  u16  RESERVED7;
  vu16 ALRH;
  u16  RESERVED8;
  vu16 ALRL;
  u16  RESERVED9;
} RTC_TypeDef;

 
typedef struct
{
  vu32 POWER;
  vu32 CLKCR;
  vu32 ARG;
  vu32 CMD;
  vuc32 RESPCMD;
  vuc32 RESP1;
  vuc32 RESP2;
  vuc32 RESP3;
  vuc32 RESP4;
  vu32 DTIMER;
  vu32 DLEN;
  vu32 DCTRL;
  vuc32 DCOUNT;
  vuc32 STA;
  vu32 ICR;
  vu32 MASK;
  u32  RESERVED0[2];
  vuc32 FIFOCNT;
  u32  RESERVED1[13];
  vu32 FIFO;
} SDIO_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SR;
  u16  RESERVED2;
  vu16 DR;
  u16  RESERVED3;
  vu16 CRCPR;
  u16  RESERVED4;
  vu16 RXCRCR;
  u16  RESERVED5;
  vu16 TXCRCR;
  u16  RESERVED6;
  vu16 I2SCFGR;
  u16  RESERVED7;
  vu16 I2SPR;
  u16  RESERVED8;  
} SPI_TypeDef;

 
typedef struct
{
  vu32 CTRL;
  vu32 LOAD;
  vu32 VAL;
  vuc32 CALIB;
} SysTick_TypeDef;

 
typedef struct
{
  vu16 CR1;
  u16  RESERVED0;
  vu16 CR2;
  u16  RESERVED1;
  vu16 SMCR;
  u16  RESERVED2;
  vu16 DIER;
  u16  RESERVED3;
  vu16 SR;
  u16  RESERVED4;
  vu16 EGR;
  u16  RESERVED5;
  vu16 CCMR1;
  u16  RESERVED6;
  vu16 CCMR2;
  u16  RESERVED7;
  vu16 CCER;
  u16  RESERVED8;
  vu16 CNT;
  u16  RESERVED9;
  vu16 PSC;
  u16  RESERVED10;
  vu16 ARR;
  u16  RESERVED11;
  vu16 RCR;
  u16  RESERVED12;
  vu16 CCR1;
  u16  RESERVED13;
  vu16 CCR2;
  u16  RESERVED14;
  vu16 CCR3;
  u16  RESERVED15;
  vu16 CCR4;
  u16  RESERVED16;
  vu16 BDTR;
  u16  RESERVED17;
  vu16 DCR;
  u16  RESERVED18;
  vu16 DMAR;
  u16  RESERVED19;
} TIM_TypeDef;

 
typedef struct
{
  vu16 SR;
  u16  RESERVED0;
  vu16 DR;
  u16  RESERVED1;
  vu16 BRR;
  u16  RESERVED2;
  vu16 CR1;
  u16  RESERVED3;
  vu16 CR2;
  u16  RESERVED4;
  vu16 CR3;
  u16  RESERVED5;
  vu16 GTPR;
  u16  RESERVED6;
} USART_TypeDef;

 
typedef struct
{
  vu32 CR;
  vu32 CFR;
  vu32 SR;
} WWDG_TypeDef;

 
 
 
 



 



 


 




#line 634 "..\\SYSTEM\\sys\\stm32f10x_map.h"

#line 651 "..\\SYSTEM\\sys\\stm32f10x_map.h"



#line 670 "..\\SYSTEM\\sys\\stm32f10x_map.h"

 

 


 






 


 






 
 
 

 



























































































































































































































#line 924 "..\\SYSTEM\\sys\\stm32f10x_map.h"














 
#line 1180 "..\\SYSTEM\\sys\\stm32f10x_map.h"

 
 
 



 
#line 4 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"














 

 



 
#line 23 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"

 
 
typedef struct
{
  u8 NVIC_IRQChannel;
  u8 NVIC_IRQChannelPreemptionPriority;
  u8 NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

 
 
#line 96 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"


#line 158 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"


 
#line 170 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"





#line 182 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"












#line 201 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"











 






 








 
#line 239 "..\\SYSTEM\\sys\\stm32f10x_nvic.h"












 
 
void NVIC_DeInit(void);
void NVIC_SCBDeInit(void);
void NVIC_PriorityGroupConfig(u32 NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_StructInit(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SETPRIMASK(void);
void NVIC_RESETPRIMASK(void);
void NVIC_SETFAULTMASK(void);
void NVIC_RESETFAULTMASK(void);
void NVIC_BASEPRICONFIG(u32 NewPriority);
u32 NVIC_GetBASEPRI(void);
u16 NVIC_GetCurrentPendingIRQChannel(void);
ITStatus NVIC_GetIRQChannelPendingBitStatus(u8 NVIC_IRQChannel);
void NVIC_SetIRQChannelPendingBit(u8 NVIC_IRQChannel);
void NVIC_ClearIRQChannelPendingBit(u8 NVIC_IRQChannel);
u16 NVIC_GetCurrentActiveHandler(void);
ITStatus NVIC_GetIRQChannelActiveBitStatus(u8 NVIC_IRQChannel);
u32 NVIC_GetCPUID(void);
void NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void NVIC_GenerateSystemReset(void);
void NVIC_GenerateCoreReset(void);
void NVIC_SystemLPConfig(u8 LowPowerMode, FunctionalState NewState);
void NVIC_SystemHandlerConfig(u32 SystemHandler, FunctionalState NewState);
void NVIC_SystemHandlerPriorityConfig(u32 SystemHandler, u8 SystemHandlerPreemptionPriority,
                                      u8 SystemHandlerSubPriority);
ITStatus NVIC_GetSystemHandlerPendingBitStatus(u32 SystemHandler);
void NVIC_SetSystemHandlerPendingBit(u32 SystemHandler);
void NVIC_ClearSystemHandlerPendingBit(u32 SystemHandler);
ITStatus NVIC_GetSystemHandlerActiveBitStatus(u32 SystemHandler);
u32 NVIC_GetFaultHandlerSources(u32 SystemHandler);
u32 NVIC_GetFaultAddress(u32 SystemHandler);



 
#line 5 "..\\SYSTEM\\sys\\sys.h"





























																	    
	 







#line 50 "..\\SYSTEM\\sys\\sys.h"

#line 58 "..\\SYSTEM\\sys\\sys.h"
 
























#line 92 "..\\SYSTEM\\sys\\sys.h"
								   







void Stm32_Clock_Init(u8 PLL);  
void Sys_Soft_Reset(void);      
void Sys_Standby(void);         
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);
void JTAG_Set(u8 mode);


void WFI_SET(void);		
void INTX_DISABLE(void);
void INTX_ENABLE(void);	
void MSR_MSP(u32 addr);	















#line 2 "Temperature.c"
#line 1 "user_def.h"



















#line 29 "user_def.h"





















 
#line 61 "user_def.h"


















 









extern u8 RxBuffer[64];
extern u8 RxIdx;
extern u8 RxStage;

extern u16 TMR_Int_Flag; 
extern  u8 PixReadmMode;
extern u16 InteCnt;
extern u8 PixReadmMode;







#line 110 "user_def.h"
				 























	





#line 146 "user_def.h"




#line 159 "user_def.h"





#line 174 "user_def.h"

#line 183 "user_def.h"





       


   
	   







#line 209 "user_def.h"
 
typedef struct
{
  u8 RampTrim;
  u8 RangTrim;
  u8 Ipix_V24Trim;		 
  u8 V20_V15Trim; 		 
  u8 SW_TxCtrl; 		  
  u8 TsTADC_AmuxCtrl;    

  u8 SpiInsertion;		 
  u16 InteTime;			 
  u16 InteCount;		 
  u16 InteDelayCount;	

} PCR_Regs_type;
extern PCR_Regs_type PCR_Regs;




























#line 264 "user_def.h"



#line 277 "user_def.h"




									  






									   










 
#line 307 "user_def.h"
void OSC_Ctrl_Init(void);
extern u8 OSC_Status;
extern u8 OSC_mode;
extern u8 OSC_Busy;
extern u16 UserCountMS;
extern u16 BaseCounter;


                       
typedef enum
{	
    READY,
	WAIT,
	ACTIVE
}Cycle_STS_Type;

extern Cycle_STS_Type CycleSTS;






#line 345 "user_def.h"

#line 363 "user_def.h"







typedef union 
{
	struct
	{
		u8 byte0;
		u8 byte1;
		u8 byte2;
		u8 byte3;
	}tempd;
	float float_num;
}KL_union;


typedef struct
{
	u8 CycleValid;
	u8 TotalCycle;
	u8 TotalSect;
	u8 TotalStage;
}CycleControlType;

typedef struct
{
 	KL_union SetPoint;
	union
	{
		struct
		{
			u8 byte0;
			u8 byte1;
		}tempw;
		u16 SetTime;
	}SetTime_Union;	
}CycleTempType;


















typedef u8 MSG_TYP;



	void EpMsgStk(MSG_TYP u8Q);
	MSG_TYP EpMsgPop(void);
	void EpMsgClr(void);








	
#line 3 "Temperature.c"
#line 1 "temperature.h"












#line 23 "temperature.h"














typedef enum {TEMP1=1,TEMP2} TempControlNum;


void TempControl_Initial(float TempSet);
void TempControl_2_Initial(float TempSet);
float  TempSensorRead(u8 IIC_Addr);







bool Sensor_Cfg_Write(u8 idx, u16 dt);
u16 Sensor_Cfg_Read(u8 idx);


void Sensor_Res_Init(void);

void TempControl_stop(void);
void TempControl_2_stop(void);
void PWM1_Init(void);
void PWM2_Init(void);
void PWM1_FREEZE(void);
void PWM2_FREEZE(void);
void PWM1_RELEASE(void);
void PWM2_RELEASE(void);
void TempContorl_2_Init(void);
void Channel_Swap(u8 sensor_num);
void PID_Clear(u8 INDEX);

extern float TempCurrent;
extern float TempCurrent_2;
extern u16 TempCtrlTime_Sec;   
extern u16 TempTickLength;
extern u16 TempCtrlTime_Sec_2;   
extern u16 TempTickLength_2;
extern u8 TempCtrl_Active;
extern u8 TempCtrl_Active_2;
extern u8 TempValid;
extern u8 Control_2_Dir;
extern float TempSet;
extern float TempSet_2;
extern float TempPumpSet;
extern u8 TickLock;
extern u8 Peltier_Swap_Msg;

extern float Kp[2];
extern float Ki[2];
extern float Kd[2];
extern float Kl[2];
extern float Ktm[2];
typedef struct PID_Err
{
	float Current_Err;
	float Previous_Err;
	float Current_Derr;
	float Previous_Derr;
	float Inte_Err;
}PID_Err_Rec_type;






extern PID_Err_Rec_type PID_Err_Rec[2];

#line 122 "temperature.h"




	
typedef u8 MsgQ;




extern MsgQ MsgStk;

MsgQ PollMsg(void);
void MsgHandler(void);
void PopMsg(u8 msk);
void PushMsg(u8 msk);
void ClearMsg(void);
#line 4 "Temperature.c"
#line 1 "..\\HARDWARE\\24CXX\\myiic.h"
#line 4 "..\\HARDWARE\\24CXX\\myiic.h"













   	   		   










void IIC_Init(void);                
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

















#line 5 "Temperature.c"
#line 1 "..\\HARDWARE\\LED\\led.h"
#line 4 "..\\HARDWARE\\LED\\led.h"














#line 24 "..\\HARDWARE\\LED\\led.h"





void LED_Init(void);


















#line 6 "Temperature.c"
#line 1 "..\\HARDWARE\\TIMER\\timer.h"
#line 4 "..\\HARDWARE\\TIMER\\timer.h"




























void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM5_Cap_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM3_ARR_Update(u16 arr);

void TIM1_Int_Init(u16 arr,u16 psc);
void TIM1_ARR_Update(u16 arr);

void TIM6_Int_Init(u16 arr,u16 psc);
void TIM6_ARR_Update(u16 arr);

void TIM8_Int_Init(u16 arr,u16 psc);
void TIM8_ARR_Update(u16 arr);


void TIM7_Int_Init(u16 arr,u16 psc);
void TIM7_ARR_Update(u16 arr);
void TIM7_Init(void);
void TIM7_Stop(void);
void TIM7_IRQHandler(void);
























#line 7 "Temperature.c"
#line 1 "PCR_Cycle.h"



 
#line 15 "PCR_Cycle.h"
 
  











void FAN_Init(void);

extern CycleControlType  PCR_Cycle_Control,Buffer_Cycle_Control;
extern CycleTempType	 PCR_Cycle_SetPoint[10],Buffer_Cycle_SetPoint[10], *pCycleArray;
extern CycleTempType     PE_Cycle_SetPoint[2];




void FanCtrl_Force(u8 ctrl);   
void FanCtrl_Auto(u8 ctrl);	   
void FanMode_Clear(void);      

void PCR_Cycle_Init(void);
u8 SetPoint_Check(u8 Stage_Num,CycleTempType *pCycleArray);
void SetPoint_Copy(u8 Stage_Num,CycleTempType *pSource, CycleTempType *pDest);
void SetPoint_Cycle_Start(CycleTempType *p);
u8 Cycle_Check(void);
u8 Stage_Check(void);
void Cycle_Set(u8 cycle);
void Stage_Set(u8 stage);
void Cycle_INC(void);
void Stage_INC(void);
void TempCtrl_Reload_InCycle(void);
void FanCtrl_Init(void);
u8 FanCSR_Read(void);
void SetPoint_PrePump_Start(void);
void SetPoint_Extension_Start(void);
void Cycle_Control_Start(void);
void Fan_Echo(u8 dat);

#line 8 "Temperature.c"


static u8 I2C_State_Machine=0;
static u8 TermpValue[2]={0};
static u8 I2C_Rx_Cnt=0;

u8 TempCtrl_Active=0;
u8 TempCtrl_Active_2=0;
u8 TempValid=0;
u8 Control_2_Dir=0x0;
u8 TickLock=0;
void TIM4_Int_Init(u16 arr,u16 psc);
void SysTick_Init(void);

int PID_Cal(u8 sensor_index,float temperature);



u16 TempCtrlTime_Sec=10; 
u16 TempCtrlTime_Sec_2=10;
u16 TempTickLength=0;   
u16 TempTickLength_2=0;
u8 Peltier_Swap_Msg=0;

u8 Sensor_Addr[2]={0x90,0x92};

PID_Err_Rec_type PID_Err_Rec[2];
static u8 Pwm_Pos_Lock= 0x1;
static MsgQ MsgStk=0;
























 

void TIM4_Int_Init(u16 arr,u16 psc)
{
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB1ENR|=1<<2;			
 	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->ARR=((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CNT=arr;  	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->PSC=psc;  	  



	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CR1|=(0x01<<7);
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->SR&=~(1<<0);
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CR1|=(0x01<<4); 
	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCMR2=(0x01<<3); 

	
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCMR2 |= (0x6<<4);
  	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCER  |= (0x1<<8);	      
 	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCR3  = 0;				  
	


}

void P2N_Control(u8 Control)
{
	if(Control == 0x1)
		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR&=~(u32)(1<<1);
	else
		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR|= (u32)(1<<1);	
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL&=0XFFFFFF0F; 
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL|=0X00000030;

}

void P1N_Control(u8 Control)
{
	if(Control == 0x1)
		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR&=~(u32)(1<<0);
	else
		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR|= (u32)(1<<0);
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL&=0XFFFFFFF0; 
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL|=0X00000003;

}

void PWM1_FREEZE(void)  
	{((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x2C00))->CR1 &=~(u16)0x1; 
	 ((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH&=0XFFFFFFF0; 
	 ((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH|=0X00000003;
	 ((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->ODR&=~(u32)(1<<8);
	 P2N_Control(0x0);	}

void PWM2_FREEZE(void)	 
	{((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CR1 &=~(u16)0x1; 
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL&=0XF0FFFFFF; 
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL|=0X03000000;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->ODR&=~(u32)(1<<6);
	P1N_Control(0x0);	}

void PWM1_RELEASE(void) 
	{PWM2_FREEZE();
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<2;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH&=0XFFFFFFF0; 
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH|=0X0000000B;
	P2N_Control(0x1);} 


void PWM2_RELEASE(void)
	{PWM1_FREEZE();
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<4;
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL&=0XF0FFFFFF; 
    ((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x1000))->CRL|=0X0B000000;
	P1N_Control(0x1);}	  


void PWM1_Init()
{
   PWM1_RELEASE();
   TIM1_Int_Init(499,71);
}

void PWM2_Init()
{
   PWM2_RELEASE();
   TIM8_Int_Init(499,71);
}



static float TempReg=0;
float  TempSensorRead(u8 IIC_Addr)
{
 
   u8 i,TMP_NACK;
   float fTemp=0;
   vs8 s8temp;

   TMP_NACK=0;

 	do
	{
		IIC_Start();
		IIC_Send_Byte(IIC_Addr);
		if(IIC_Wait_Ack()==0)
		{
			IIC_Send_Byte(0);
			if(IIC_Wait_Ack()==0)
			{
				IIC_Start();
				IIC_Send_Byte((IIC_Addr |0x1));
				if(IIC_Wait_Ack()==0)
				{
					TermpValue[1]=IIC_Read_Byte(1);
					TermpValue[0]=IIC_Read_Byte(0);
					IIC_Stop();
					TMP_NACK=0;
				}
				else
					TMP_NACK++;	
			}
			else
				TMP_NACK++;	
		}
		else
			TMP_NACK++;

	}while((TMP_NACK != 0)&&(TMP_NACK<5));




















 
   if(TMP_NACK==0)
   {
   		fTemp=(vs8)TermpValue[1];
		



 
			fTemp+=(float)((TermpValue[0]>>4)*0.0625);			
   }
   if(TermpValue[0]>0x80)
   	TermpValue[0]--;
   return fTemp;
}

u16 TempControl_ReadTemp(void)
{
	u16 u16temp=0;
	u8  u8temp=0;
 	u16temp=(u16)TempReg;
	u16temp<<=8;
	u8temp=	(u8)(TempReg*2);
	if(u8temp & 0x1)
		u16temp |= 0x5;

	return u16temp; 
}

 





float RegE;
float RegE1;
float RegE2;
u16   DutyDelta;
u16   DutyCycle; 

float TempSet;
float TempCurrent;
float TempDelta;

float RegE_2;
float RegE1_2;
float RegE2_2;
int   DutyDelta_2;
int   DutyCycle_2; 


float TempSet_2;
float TempCurrent_2;
float TempDelta_2;

float TempPumpSet=105;
void Tick_Init(void);
void PWM3_Initial(void)
{
	((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<3;    
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRH&=0XFFFFFFF0; 
	((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRH|=0X0000000B;
	TIM4_Int_Init(499,71);	 
}

void TempControl_Initial(float Target)
{
	PID_Err_Rec[0].Current_Err=PID_Err_Rec[0].Previous_Err=PID_Err_Rec[0].Previous_Derr=PID_Err_Rec[0].Previous_Derr=0;
	PID_Err_Rec[0].Inte_Err=0;
	PWM3_Initial();

	TempSet=Target;




	TempCurrent=TempSensorRead(0x90);


	TempDelta=TempSet-TempCurrent;

	
	{
		RegE1=RegE2=0;
		DutyCycle=DutyDelta=1;
		((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CR1|=0x1;	   




			TempCurrent=TempSensorRead(0x90);


			RegE=TempDelta=TempSet-TempCurrent;
			if(TempDelta!=0)
			{
				if(TickLock==0)
					Tick_Init();
				TickLock |= 0x1;
			}
				
		    
		
	}
	
	{
	
	}
}

void Channel2_PWM_Start(u8 control)
{
	if(control==0x1)
	{
		((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x2C00))->CR1 |=0x1;
		((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x2C00))->BDTR |= 1<<15;
	}
	else if(control==0x2)
	{
		((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CR1 |=0x1;
		((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->BDTR |= 1<<15;
	}	
}

void TempContorl_2_Init(void)
{
   ((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<4;
   ((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<2;
   PWM1_FREEZE();
   ((RCC_TypeDef *) ((((u32)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<3;
   PWM2_FREEZE();
}


void TempControl_2_Initial(float Target)
{
	PID_Err_Rec[1].Current_Err=PID_Err_Rec[1].Previous_Err=PID_Err_Rec[1].Current_Derr=PID_Err_Rec[1].Previous_Derr=0;
	PID_Err_Rec[1].Inte_Err=0;
	TempSet_2=Target;
	TempCurrent_2=TempSensorRead(0x92);
	if(TempCurrent_2 > TempSet_2)  
	{
	   PWM2_Init();
	   Control_2_Dir=0x2;	
	}else{ 				   			
		   PWM1_Init();	
		   Control_2_Dir=0x1;					
		}	

	if(Control_2_Dir==0x1)
		RegE=TempDelta_2=TempSet_2-TempCurrent_2;
	else
		RegE=TempDelta_2=TempCurrent_2-TempSet_2;

	RegE1_2=RegE2_2=0;
	DutyCycle_2=DutyDelta_2=1;
    Channel2_PWM_Start(Control_2_Dir);

	if(TempDelta_2!=0)
	{
		if(TickLock==0)
			Tick_Init();
		TickLock |= 0x2;
	}
}


void Tick_Init(void)
{
	
	
	
	TIM6_Int_Init(1666,7199);  
	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->CR1|=0x01;
}


void TempControl_stop(void)
{
	((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CR1 &=~(u16)0x1;
	TickLock &= ~(1<<0);
	if(TickLock==0)       
		((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->CR1 &=~(u16)0x1;
   ((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRH&=0XFFFFFFF0; 
   ((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->CRH|=0X00000003;
   ((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0C00))->ODR&=~(u32)(1<<8);
}

void TempControl_2_stop(void)
{
	TickLock &= ~(1<<1);
	
		PWM1_FREEZE();
	
		PWM2_FREEZE();
	if(TickLock==0)       			
		((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->CR1 &=~(u16)0x1;
    
		
	FanCtrl_Auto(0);	

}

static void Channel_2_Load (u8 control_dir, u16 load)
{
	if(control_dir==0x1)
		((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x2C00))->CCR1 =load; 
	else if(control_dir==0x2)
			((TIM_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x3400))->CCR1 =load; 
}

















































 

u8 rec=0;

void TIM6_IRQHandler(void)
{ 	
	int DutyDiff=0;		   
	((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->SR&=~(1<<0);
	rec=~rec;
	*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x2000)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x2000)+12) &0xFFFFF)<<5)+(11<<2))))=rec;




  




	if((TickLock & 0x1) !=0)
	{
		if(TempCtrl_Active==3)	  
		{
			if(CycleSTS == READY) 
				TempTickLength--;
		}

		if(TempTickLength==0)
			TempCtrl_Active=4;
				
		RegE=TempDelta=TempSet-TempCurrent;

		if(TempCtrl_Active==2) 
		{
			if((TempDelta>=-2) && (TempDelta<=2))
			{
				TempCtrl_Active=3;
				if(CycleSTS == WAIT)   
					PushMsg(0x4);
			}
		}



















 	









 
		PID_Err_Rec[0].Previous_Err=PID_Err_Rec[0].Current_Err;
		PID_Err_Rec[0].Current_Err= TempSet -TempCurrent;
		
		PID_Err_Rec[0].Previous_Derr= PID_Err_Rec[0].Current_Derr;
		PID_Err_Rec[0].Current_Derr= PID_Err_Rec[0].Current_Err-PID_Err_Rec[0].Previous_Err;

		DutyDiff=PID_Cal(0,TempCurrent);
		if((DutyDiff)>500)
				DutyDiff=500;
		else if	((DutyDiff)<0)
					DutyDiff=0;	
		DutyCycle=DutyDiff;
		((TIM_TypeDef *) (((u32)0x40000000) + 0x0800))->CCR3=DutyCycle;
	} 
	
	if((TickLock & 0x2)!=0) 
	{
		
		if(TempCtrl_Active_2==3)
		{
			TempTickLength_2--;
			*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=1;
		}
		if(TempTickLength_2==0)
		{
			TempCtrl_Active_2=4;
			*((volatile unsigned long *)((((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) & 0xF0000000)+0x2000000+(((((((u32)0x40000000) + 0x10000) + 0x1C00)+12) &0xFFFFF)<<5)+(5<<2))))=0;
			if(CycleSTS != READY)
			{
				PushMsg(0x2);	  
				if((( PCR_Cycle_Control.TotalCycle==Cycle_Check())&&(PCR_Cycle_Control.TotalStage==Stage_Check()))) 
					PushMsg(0x10);
			}
		}
		if(Control_2_Dir==0x1)		
			RegE_2=TempDelta_2=TempSet_2-TempCurrent_2;
		else if(Control_2_Dir==0x2)
			{
		    	RegE_2=TempDelta_2=TempCurrent_2-TempSet_2;
				if((TempDelta_2<8)&&(TempCtrl_Active_2 !=4))  
				{
					Peltier_Swap_Msg |= 0x2;
				}
			}
		if(TempCtrl_Active_2==2) 
		{
			if((TempDelta_2>=-2) && (TempDelta_2<=2))
				TempCtrl_Active_2=3;
		}






















 








 
		PID_Err_Rec[1].Previous_Err=PID_Err_Rec[1].Current_Err;
		PID_Err_Rec[1].Current_Err= (Control_2_Dir==0x1)? (TempSet_2 -TempCurrent_2):(TempCurrent_2-TempSet_2);
		
		PID_Err_Rec[1].Previous_Derr= PID_Err_Rec[1].Current_Derr;
		PID_Err_Rec[1].Current_Derr= PID_Err_Rec[1].Current_Err-PID_Err_Rec[1].Previous_Err;
		









 

		if(!((TempDelta_2>= -0.25) && (TempDelta_2<= 0.25)))
		{
		DutyDiff=PID_Cal(1,TempCurrent_2);
		if((DutyDiff)>500)
				DutyDiff=500;
		else if	((DutyDiff)<0)
					DutyDiff=0;
		
		if(Control_2_Dir==0x2)
		{
			DutyDiff=25;
		}else if(DutyDiff==0)
			{
				if(Pwm_Pos_Lock==0x1)
				{
					Pwm_Pos_Lock=0x0;
					((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH&=0XFFFFFFF0; 
			 		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH|=0X00000003;
			 		((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->ODR&=~(u32)(1<<8);
				}
			}else if(Pwm_Pos_Lock==0x0)
				{
					Pwm_Pos_Lock=0x1;
					((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH&=0XFFFFFFF0; 
					((GPIO_TypeDef *) ((((u32)0x40000000) + 0x10000) + 0x0800))->CRH|=0X0000000B;	
				}
		
		Channel_2_Load(Control_2_Dir,DutyDiff);
		}
	}  
}













































 





 

float Kp[2]={20,20};
float Ki[2]={0.03,0.03};
float Kd[2]={1,1};
float Kl[2]={0.3,0.3};

float Ktm[2]={70,70}; 




















 

int PID_Cal(u8 sensor_index,float temperature)
{
	int Val;
	
	u8 idx=0;
	if(temperature>Ktm[sensor_index])
	 	idx=1;
	
	
	
	PID_Err_Rec[sensor_index].Inte_Err=(float)Ki[idx] * PID_Err_Rec[sensor_index].Current_Err + PID_Err_Rec[sensor_index].Inte_Err;
	if(sensor_index==0)
		Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet* Kl[idx];
	else if(sensor_index==1)
		Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx];
	return Val;
}

void Channel_Swap(u8 sensor_num)
{
 	if(sensor_num==0x2)  
	{
		
		((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->DIER&= ~0x1;
		Peltier_Swap_Msg &=  ~0x2;
		if(Control_2_Dir==0x2)
		{
			PID_Clear(1);
			Control_2_Dir=0x1;
			PWM1_Init();
			Channel2_PWM_Start(Control_2_Dir);
			FanCtrl_Auto(0);  
		}
		((TIM_TypeDef *) (((u32)0x40000000) + 0x1000))->DIER|= 0x1;	

	}
}


void PID_Clear(u8 INDEX)
{
 	PID_Err_Rec[INDEX].Current_Err=0;
	PID_Err_Rec[INDEX].Previous_Err=0;
	PID_Err_Rec[INDEX].Current_Derr=0;
	PID_Err_Rec[INDEX].Previous_Derr=0;
	PID_Err_Rec[INDEX].Inte_Err=0;
}


#line 877 "Temperature.c"


bool Sensor_Cfg_Write(u8 idx, u16 dt)
{  
   u8 TMP_NACK;
   TMP_NACK=0;

 	do
	{
		IIC_Start();
		IIC_Send_Byte(Sensor_Addr[idx]);
		if(IIC_Wait_Ack()==0)
		{
			IIC_Send_Byte(0x1);
			if(IIC_Wait_Ack()==0)
			{
				
				
				
				{
					IIC_Send_Byte((u8)(dt>>8));
					if(IIC_Wait_Ack()==0)
					{
						IIC_Send_Byte((u8)(dt));
						if(IIC_Wait_Ack()==0)
						  	IIC_Stop();
						else
						    TMP_NACK++;
					}	
					else
						TMP_NACK++;
				}
				
				
			}
			else
				TMP_NACK++;	
		}
		else
			TMP_NACK++;

	}while((TMP_NACK != 0)&&(TMP_NACK<5));

   	if(TMP_NACK==0)
		return TRUE;
	else
   		return FALSE;
} 

u16 Sensor_Cfg_Read(u8 idx)
{
   u16 dt;
   u8 TMP_NACK;

   TMP_NACK=0;
   dt=0;
 	do
	{
		IIC_Start();
		IIC_Send_Byte(Sensor_Addr[idx]);
		if(IIC_Wait_Ack()==0)
		{
			IIC_Send_Byte(0x1);
			if(IIC_Wait_Ack()==0)
			{
				IIC_Start();
				IIC_Send_Byte((Sensor_Addr[idx]|0x1));
				if(IIC_Wait_Ack()==0)
				{
					dt=IIC_Read_Byte(1);
					dt<<=8;
					dt |=IIC_Read_Byte(0);
					IIC_Stop();
					TMP_NACK=0;
				}
				else
					TMP_NACK++;	
			}
			else
				TMP_NACK++;	
		}
		else
			TMP_NACK++;

	}while((TMP_NACK != 0)&&(TMP_NACK<5));

	return 	dt;	
}


void Sensor_Res_Init(void)
{
	u8 i;




	u16 u16temp;



	for(i=0;i<2;i++)
	{
#line 988 "Temperature.c"
		u16temp=Sensor_Cfg_Read(i);
		u16temp |= (0x11<<6);
		Sensor_Cfg_Write(i,u16temp);


	}
}


MsgQ PollMsg(void)
{
 	return MsgStk;
}

void ClearMsg(void)
{
	MsgStk=0;
}

void MsgHandler(void)
{

	if(MsgStk !=0)
	{
		TickLock &= ~0x2;
		switch (PCR_Cycle_Control.CycleValid)
		{
			case 2:
					if((MsgStk & 0x4))
					{
						ClearMsg();
						SetPoint_PrePump_Start();
						PCR_Cycle_Control.CycleValid=4;			
					}
				break;
			case 4:
				 	if((MsgStk & 0x2))
					{
						ClearMsg();
						SetPoint_Cycle_Start(PCR_Cycle_SetPoint);
						PCR_Cycle_Control.CycleValid=6;
					}
				break;
			case 6:			  
									
					if((MsgStk & 0x10))
					{
						ClearMsg();
						SetPoint_Extension_Start();
						PCR_Cycle_Control.CycleValid=8;
	
					}else if((MsgStk & 0x2))
						{
							ClearMsg();
							TickLock &= ~0x2;
							if(PCR_Cycle_Control.TotalStage==Stage_Check())
							{				
								Cycle_INC();
								Stage_Set(0);
							}
							else 
							{
							 	Stage_INC();	
							}
							TempCtrl_Reload_InCycle();
							TickLock |= 0x2;
						}
				break;
				
			case 8:
					 if((MsgStk & 0x2))
					 {
					 	ClearMsg();
			 			
						TempCtrl_Active_2=0;
						TempValid =0;
						TempControl_2_stop();
						CycleSTS=READY;
						TempControl_stop();
					 }
			
					break;
			default:
	
				break;
		}

	}
}

void PopMsg(u8 msk)
{
	MsgStk &= ~msk;
}

void PushMsg(u8 msk)
{
	MsgStk |= msk;
}
