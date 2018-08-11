#include "sys.h"
#include "user_def.h"
#include "temperature.h"
#include "myiic.h"
#include "led.h"
#include "timer.h"
#include "PCR_Cycle.h"
//#define TMP_DBUG

//static u8 I2C_State_Machine=0;
static u8 TermpValue[2]={0};
//static u8 I2C_Rx_Cnt=0;

u8 TempCtrl_Active=0;
u8 TempCtrl_Active_2=0;
u8 TempValid=0;
u8 Control_2_Dir=NULL_DIR;
u8 TickLock=0;
void TIM4_Int_Init(u16 arr,u16 psc);
void SysTick_Init(void);
//u16 PID_Cal(float Delta);
int PID_Cal(u8 sensor_index,float temperature);
#define MAX_DUTY	500
#define MAX_DUTY_NEG	325
#define MIN_DUTY	0


TimType TempCtrlTime_Sec=10; // time for heating, uint is second
TimType TempCtrlTime_Sec_2=10;
TimType TempTickLength=0;   // tick for heating. second * tick unit/sec
TimType TempTickLength_2=0;


u8 Peltier_Swap_Msg=0;

u8 Sensor_Addr[SENSOR_TOTAL]={STLM75_ADDR_WR,TEMP_SENSOR_2_ADDR_WR};
//static u16 TickPerSecond=1;
PID_Err_Rec_type PID_Err_Rec[TOTAL_SENSOR_NUM];
static u8 Pwm_Pos_Lock= OPEN;
static MsgQ MsgStk = 0;
u8 WaitOverShot=0;

float integrationDur=0;
/*
void TempSensor_Initial(TempControlNum TempNum)
{
	float fRead=0;
	 ////TIM4_CH3 use pin PB8
	if(TempNum==TEMP1)
	{
	   RCC->APB2ENR|=1<<3;    //使能PORTB时钟 
	   GPIOB->CRH&=0XFFFFFFF0; //PB.8 AF_PP
	   GPIOB->CRH|=0X0000000B;
	   TIM4_Int_Init(499,63);	 // 500k clock, 500 step pwm
	}else if(TempNum==TEMP2)	 // 2nd channel is bi-dir, check sensor to determind PWM 
		{
			fRead=TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
			if(fRead > TempSet_2)  // neg control
			{
			   PWM2_Init();
			   Control_2_Dir=NEG_DIR;	
			}else{ 				   			// pos control
				   PWM1_Init();	
				   Control_2_Dir=POS_DIR;					
				}								
		}
}
*/

void TIM4_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<2;			//TIM4时钟使能    
 	TIM4->ARR=TIM4->CNT=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM4->PSC=psc;  	  //预分频器7200,得到10Khz的计数时钟		  
//	TIM8->DIER|=1<<0;   //允许更新中断	  
//	TIM3->CR1|=0x01;    //使能定时器3

	TIM4->CR1|=(0x01<<7);
	TIM4->SR&=~(1<<0);//清除中断标志位
	TIM4->CR1|=(0x01<<4); // count down
	//TIM8->CR1|=(0x01<<3); // opm
	TIM4->CCMR2=(0x01<<3); //pre-load enable
//  	MY_NVIC_Init(1,3,TIM3_IRQChannel,2);//抢占1，子优先级3，组2
	
	TIM4->CCMR2 |= (0x6<<4);
  	TIM4->CCER  |= (0x1<<8);	      // OCRef active high, output on CH3
 	TIM4->CCR3  = 0;				  // count down mode, inactive all cycle when initial
	
//	TIM4->CR1|=0x1;	
//	TIM4->CR1&=~0x1;								 
}

void P2N_Control(u8 Control)
{
#ifndef __QPCR_HW
	if(Control == OPEN)
		GPIOC->ODR&=~(u32)(1<<1);
	else
		GPIOC->ODR|= (u32)(1<<1);	
#else  // version 1.4 :2017-04-12: fix to change use NMOS
	#ifdef __USE_NMOS_DRIVE   // version 1.5 to suuport NMOS H/W 
				if(Control == OPEN)
					GPIOC->ODR&=~(u32)(1<<1);
				else
					GPIOC->ODR|= (u32)(1<<1);	
	#else
				if(Control == SHUT)
					GPIOC->ODR&=~(u32)(1<<1);
				else
					GPIOC->ODR|= (u32)(1<<1);
	#endif
#endif
		GPIOC->CRL&=0XFFFFFF0F; 
		GPIOC->CRL|=0X00000030;	
}

void P1N_Control(u8 Control)
{
#ifndef __QPCR_HW
	if(Control == OPEN)
		GPIOC->ODR&=~(u32)(1<<0);
	else
		GPIOC->ODR|= (u32)(1<<0);
#else  // version 1.4: 2017-04-12: fix to change use NMOS
	#ifdef __USE_NMOS_DRIVE   // version 1.5 to suuport NMOS H/W 
			if(Control == OPEN)
				GPIOC->ODR&=~(u32)(1<<0);
			else
				GPIOC->ODR|= (u32)(1<<0);	
	#else
			if(Control == SHUT)
				GPIOC->ODR&=~(u32)(1<<0);
			else
				GPIOC->ODR|= (u32)(1<<0);			
	#endif
#endif
		GPIOC->CRL&=0XFFFFFFF0; 
		GPIOC->CRL|=0X00000003;	
}

void PWM1_FREEZE(void)  
{
		TIM1->CR1 &=~(u16)0x1; 
		GPIOA->CRH&=0XFFFFFFF0; 
		GPIOA->CRH|=0X00000003;
/*
#ifdef __QPCR_HW
	 GPIOA->ODR |=(u32)(1<<8);
#else
	 GPIOA->ODR&=~(u32)(1<<8);
#endif
*/
		///// version 1.4: 2017-04-12: fix to change default all Low
		GPIOA->ODR&=~(u32)(1<<8);
		P2N_Control(SHUT);	
}

void PWM2_FREEZE(void)	 
{
		TIM8->CR1 &=~(u16)0x1; 
		GPIOC->CRL&=0XF0FFFFFF; 
		GPIOC->CRL|=0X03000000;
/*
#ifdef __QPCR_HW
	 GPIOC->ODR |=(u32)(1<<6);
#else
	 GPIOC->ODR&=~(u32)(1<<6);
#endif	
*/
		///// version 1.4: 2017-04-12: fix to change default all Low
		GPIOC->ODR&=~(u32)(1<<6);
		P1N_Control(SHUT);	
}

void PWM1_RELEASE(void) 
	{PWM2_FREEZE();
	RCC->APB2ENR|=1<<2;
	GPIOA->CRH&=0XFFFFFFF0; 
	GPIOA->CRH|=0X0000000B;
	P2N_Control(OPEN);} // Open P-1N


void PWM2_RELEASE(void)
	{PWM1_FREEZE();
	RCC->APB2ENR|=1<<4;
	GPIOC->CRL&=0XF0FFFFFF; //PC.6 AF_PP
    GPIOC->CRL|=0X0B000000;
	P1N_Control(OPEN);}	  // if without NOT


void PWM1_Init()
{
   PWM1_RELEASE();
   TIM1_Int_Init(499,TIM1_PSC_VAL);
}

void PWM2_Init()
{
   PWM2_RELEASE();
   TIM8_Int_Init(499,TIM8_PSC_VAL);
}



static float TempReg=0;
float  TempSensorRead(u8 IIC_Addr)
{
 //  vs16  temp=0;
// u8 i  
   u8 TMP_NACK;
   float fTemp=0;
   vs8 s8temp;

   TMP_NACK=0;

 	do
	{
		//IIC_Start();
		//IIC_Send_Byte(IIC_Addr);
		//if(IIC_Wait_Ack()==0)
		{
			//IIC_Send_Byte(0);
			//if(IIC_Wait_Ack()==0)
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
				{
					TMP_NACK++;	
					IIC_Stop();
				}
			}
			//else
			//{
			//	TMP_NACK++;	
			//	IIC_Stop();
			//}
		}
		//else
		//{
		//	TMP_NACK++;	
		//	IIC_Stop();
		//}

	}while((TMP_NACK != 0)&&(TMP_NACK<5));
/*
   if(TMP_NACK==0)
   {
	   for(i=7;i>3;i--)
	   {
	   		if(((1<<i) & TermpValue[0]) !=0)
				fTemp+=(0.5/(8-i));
	   }
	
	   s8temp=(vs8)TermpValue[1]; 
	   if(s8temp >= 0)
	   		fTemp += (float)s8temp;
		else
		{
			s8temp= (-s8temp);
			fTemp += (float)s8temp;
			fTemp=(-fTemp);	
		}
		TempReg=fTemp;
   }
*/
   if(TMP_NACK==0)
   {
   		fTemp=(vs8)TermpValue[1];
		/*
		if(fTemp<0)
			fTemp-=(float)((TermpValue[0]>>SENSOR_LSB_SHIFT)*SENSOR_RES);
		else
		*/
			fTemp+=(float)((TermpValue[0]>>SENSOR_LSB_SHIFT)*SENSOR_RES);			
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

/*temperature control, parameter is the temp set point*/
#define AT	0.5
#define BT	0.001
#define CT	0.001
#define INI_ARR	

float RegE;
float RegE1;
float RegE2;
u16   DutyDelta;
u16   DutyCycle; 

float TempSet;
float TempCurrent;
float TempDelta;
////////////////fix issue when any temp sensor is error @160829
float TempCurrent_reg=0;
u8 sensor_1_error_cnt=0;

float RegE_2;
float RegE1_2;
float RegE2_2;
int   DutyDelta_2;
int   DutyCycle_2; 


float TempSet_2;
float TempCurrent_2;
float TempDelta_2;
////////////////fix issue when any temp sensor is error @160829
float TempCurrent_2_reg=0;
u8 sensor_2_error_cnt=0;

float TempPumpSet=TOP_TEMP_DEFAULT;
void Tick_Init(void);
void PWM3_Initial(void)
{
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟 
	GPIOB->CRH&=0XFFFFFFF0; //PB.8 AF_PP
	GPIOB->CRH|=0X0000000B;
	TIM4_Int_Init(499,TIM4_PSC_VAL);	 // 500k clock, 500 step pwm
}

void TempControl_Initial(float Target)
{
	PID_Err_Rec[INDEX0].Current_Err=PID_Err_Rec[INDEX0].Previous_Err=PID_Err_Rec[INDEX0].Previous_Derr=PID_Err_Rec[INDEX0].Previous_Derr=0;
	PID_Err_Rec[INDEX0].Inte_Err=0;
	PWM3_Initial();

	TempSet=Target;

	#ifdef TMP_DBUG
	TempCurrent=0;
	#else
	TempCurrent=TempSensorRead(STLM75_ADDR_WR);
	#endif

	TempDelta=TempSet-TempCurrent;

	//if(TempSensorRead>0)  // increase
	{
		RegE1=RegE2=0;
		DutyCycle=DutyDelta=1;
		TIM4->CR1|=0x1;	   //start PWM

			#ifdef TMP_DBUG
			TempCurrent++;
			#else
			TempCurrent=TempCurrent_reg=TempSensorRead(STLM75_ADDR_WR);
			sensor_1_error_cnt=0;
			#endif

			RegE=TempDelta=TempSet-TempCurrent;
			#ifndef __DEBUG_BETA_ERROR		
			if(TempDelta!=0)
			#endif
			{
				if(TickLock==0)
					Tick_Init();
				TickLock |= 0x1;
			}
				//SysTick_Init();
		    //while(1);			
		
	}
	//else				  // decrease, to invert the current, handle this case later
	{
	
	}
}

void Channel2_PWM_Start(u8 control)
{
	if(control==POS_DIR)
	{
		TIM1->CR1 |=0x1;
		TIM1->BDTR |= 1<<15;
	}
	else if(control==NEG_DIR)
	{
		TIM8->CR1 |=0x1;
		TIM8->BDTR |= 1<<15;
	}	
}

void TempContorl_2_Init(void)
{
   RCC->APB2ENR|=1<<4;
   RCC->APB2ENR|=1<<2;
   PWM1_FREEZE();
   RCC->APB2ENR|=1<<3;
   PWM2_FREEZE();
}


void TempControl_2_Initial(float Target)
{
	PID_Err_Rec[INDEX1].Current_Err=PID_Err_Rec[INDEX1].Previous_Err=PID_Err_Rec[INDEX1].Current_Derr=PID_Err_Rec[INDEX1].Previous_Derr=0;
	PID_Err_Rec[INDEX1].Inte_Err=0;
	TempSet_2=Target;
	TempCurrent_2=TempCurrent_2_reg=TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
	sensor_2_error_cnt=0;
	if(TempCurrent_2 > TempSet_2)  // neg control
	{
	   PWM2_Init();
	   Control_2_Dir=NEG_DIR;	
	}else{ 				   			// pos control
		   PWM1_Init();	
		   Control_2_Dir=POS_DIR;					
		}	

	if(Control_2_Dir==POS_DIR)
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
	//SysTick_Init();
	//TIM6_Int_Init(4999,6399);  // 500ms tick
	//TickPerSecond=2;
	TIM6_Int_Init(SAMPLE_CLOCK,TIM6_PSC_VAL);  
	TIM6->CR1|=0x01;

/*	
    #ifdef __RPI_THERMO

	    // pending ***???here much start a timer for time control in more precise in BETA version
		TIM3_Int_Init(THERMO_TICK,THERMO_PSC_VAL);  	
         TIM3->CR1|=0x01;
    #endif
*/

}


void TempControl_stop(void)
{
	TIM4->CR1 &=~(u16)0x1;
	TickLock &= ~(1<<0);
	if(TickLock==0)       // tick shutdown only when no temp control valid
    {   
				TIM6->CR1 &=~(u16)0x1;
        
        #ifdef __RPI_THERMO
            TIM3->CR1 &=~(u16)0x1;
        #endif         
    }
   GPIOB->CRH&=0XFFFFFFF0; //PB.8
   GPIOB->CRH|=0X00000003;
   GPIOB->ODR&=~(u32)(1<<8);
}

void TempControl_2_stop(void)
{
		TickLock &= ~(1<<1);
		//if(Control_2_Dir &= POS_DIR)	// shutdown Pos Dir
		PWM1_FREEZE();
		//if(Control_2_Dir &= NEG_DIR)	// shutdown Neg dir
		PWM2_FREEZE();
		if(TickLock==0)       			// tick shutdown only when no temp control valid
    {   
				TIM6->CR1 &=~(u16)0x1;
				// Sensor_2 stop will auto shutdown fan
		
        #ifdef __RPI_THERMO
            TIM3->CR1 &=~(u16)0x1;
        #endif 
    }
		FanCtrl_Auto(0);

		//add @2018-04-20 to support ramp parameter in alpha
		RampCtrl.HasValidRamp=0;
		RampCtrl.RampRate = 0;
}

static void Channel_2_Load (u8 control_dir, u16 load)
{
	if(control_dir==POS_DIR)
		TIM1->CCR1 =load; 
	else if(control_dir==NEG_DIR)
			TIM8->CCR1 =load; 
}

/*
void TempControl_stop(void)
{
 	SysTick->CTRL &= ~(u32)0x3;
	TIM4->CR1 &=~(u16)0x1;
}


void SysTick_Init(void)
{
 	SysTick->CTRL |= 0x4;
	SysTick->LOAD = 16000000; // 1/4S
	SysTick->CTRL |= 0x1;
	SysTick->CTRL |= 0x2;

}

u8 rec=0;
void SysTick_Handler(void)
{ 
    SysTick->VAL=0;;
	rec=~rec;
	LED1=rec;

	//TempCurrent=TempSensorRead();
	RegE=TempDelta=TempSet-TempCurrent;
	if(TempDelta>0)
	{
		//DutyDelta=	(u16)(AT*RegE- BT*RegE1 + CT*RegE2);
		//DutyCycle+=DutyDelta;

		if(DutyCycle<MAX_DUTY)
			DutyCycle++;
		else
			DutyCycle=MAX_DUTY;	
	}
	else if(TempDelta<0)
		{
			if(DutyCycle>MIN_DUTY)
				DutyCycle--;
			else
				DutyCycle=MIN_DUTY;					
		
		}

	TIM4->CCR3=DutyCycle;
	
}
*/

u8 rec=0;

////////////debug ram dump
#ifdef __USE_RAM_DEBUG
		u8 RamDumpBlock[DUMPSIZE]={0}; // input temper: 4byte,  PID output, 2 byte 
		u8 RamDumpFlag=0;
		u16 RAM_Dump_index=0;
		u8 TickDump=0;
#endif
		int Diff_Reg=0;
//#define SENSOR_DEBUG  

#ifndef __RPI_THERMO

#ifdef __USE_LYSIS

	u16 PWM_Lysis_Reg=0;

#endif		
		
void TIM6_IRQHandler(void)
{ 	
#ifdef __USE_RAM_DEBUG
	KL_union	fTemper;
#endif

#ifdef __USE_LYSIS

	u16 PWM_reg_floor=0;
  u16 PWM_reg_top=0;
#endif	
	
	u8 flag_update_PWM=0;
    u8  u8temp;
//	int j=0;
	int DutyDiff=0;	
  float fTemp=0;	
	TIM6->SR&=~(1<<0);//清除中断标志位
	rec=~rec;
	LED1=rec;
/*	
 #if defined(SENSOR_DEBUG) 
 	TempCurrent_2=95; 	
 #endif 
*/ 

#ifdef DEBUG_MSG
	  msg_debug++;
#endif
	if((TickLock & 0x1) !=0)
	{
		if(TempCtrl_Active==3)	  
		{
			if(CycleSTS == READY) 
				TempTickLength--;
		}
        #ifndef __RPI_THERMO  // not control time of peltier 0 in BETA version now, as not know cycle time at all
                              // only depend on thermo step finish
            if(TempTickLength==0)
                TempCtrl_Active=4;
        #endif 
				
		RegE=TempDelta=TempSet-TempCurrent;

		if(TempCtrl_Active==2) 
		{

			//if((TempDelta>=-2) && (TempDelta<=2))
			//if((TempDelta>=-1) && (TempDelta<=1))  // change @2018-04-22
			if(TempDelta<=1)
			{
				TempCtrl_Active=3;
				if(CycleSTS == WAIT)   // add @0607, push sensor1 active3 msg
					PushMsg(SS1_ACT3);

                #ifdef __USE_LYSIS
                    if(CycleSTS == LYSIS_READY)   // add @2018-07-14, support to run lysis only
    					PushMsg(SS1_ACT3);
                #endif

			}
		}
/*
		if( (TempDelta>0.25) || (TempDelta<-0.25))
		{
				flag_update_PWM=1;
				if((TempDelta<0)||((TempDelta>0)&&(TempDelta<DELTA_THRES)))
				{
					PID_Err_Rec[INDEX0].Previous_Err=PID_Err_Rec[INDEX0].Current_Err;
					PID_Err_Rec[INDEX0].Current_Err= TempSet -TempCurrent;
					//PID_Err_Rec[INDEX0].Inte_Err+=PID_Err_Rec[INDEX0].Current_Err;
					PID_Err_Rec[INDEX0].Previous_Derr= PID_Err_Rec[INDEX0].Current_Derr;
					PID_Err_Rec[INDEX0].Current_Derr= PID_Err_Rec[INDEX0].Current_Err-PID_Err_Rec[INDEX0].Previous_Err;
					DutyDiff=PID_Cal(INDEX0,TempCurrent);
				}
				else
					DutyDiff=MAX_DUTY;
				
				if((DutyDiff)>MAX_DUTY)
						DutyDiff=MAX_DUTY;
				else if	((DutyDiff)<MIN_DUTY)
							DutyDiff=MIN_DUTY;	
				DutyCycle=DutyDiff;
		}
		*/
///////////////////////////////////////////////
		flag_update_PWM=0;
		
		if(TempDelta<DELTA_THRES)
		{
			PID_Err_Rec[INDEX0].Previous_Err=PID_Err_Rec[INDEX0].Current_Err;
			PID_Err_Rec[INDEX0].Current_Err= TempSet -TempCurrent;
			//PID_Err_Rec[INDEX0].Inte_Err+=PID_Err_Rec[INDEX0].Current_Err;
			PID_Err_Rec[INDEX0].Previous_Derr= PID_Err_Rec[INDEX0].Current_Derr;
			PID_Err_Rec[INDEX0].Current_Derr= PID_Err_Rec[INDEX0].Current_Err-PID_Err_Rec[INDEX0].Previous_Err;
			DutyDiff=PID_Cal(INDEX0,TempCurrent);
		}
		else
			DutyDiff=MAX_DUTY;
		
		if((DutyDiff)>MAX_DUTY)
				DutyDiff=MAX_DUTY;
		else if	((DutyDiff)<MIN_DUTY)
					DutyDiff=MIN_DUTY;	
		DutyCycle=DutyDiff;
	
		if( (TempDelta>1) || (TempDelta<-1))
				flag_update_PWM=1;
////////////////////////////////////////////////		
		if(flag_update_PWM) 
				TIM4->CCR3=DutyCycle;
	} 
	
	flag_update_PWM=0;
	if((TickLock & 0x2)!=0) 
	{

        //@2018-02-24
    	#ifdef __FAN_EXT_AFTER_CYCLE

      if( PCR_Cycle_Control.CycleValid == EXT_FAN)
      {
            PushMsg(SS2_FX_TICK);
      }
      else
      {
    #endif	
		

		#ifdef __USE_LYSIS   //ver1.14 @ 2018-06-23 support lysis

		if(CycleSTS == LYSIS_RUN)
		{
			
			   if(TempCurrent_2 < 60)
				 {
						PWM_reg_floor=40;//60;
						PWM_reg_top=50;//80;
				 }
				 else if(TempCurrent_2 < 70)
							 {
									PWM_reg_floor=50;//80;
									PWM_reg_top=60;//100;
							 }				 
							 else if(TempCurrent_2 < 80)
									 {
											PWM_reg_floor=60;//100;
											PWM_reg_top=80;//120;
									 }
									 else  if(TempCurrent_2 < 90)
												 {
														PWM_reg_floor=80;//120;
														PWM_reg_top=120;//160;
												 }
												 else  if(TempCurrent_2 > 90)
															 {
																	PWM_reg_floor=160;
																	PWM_reg_top=200;
															 }
			
				 if(Lysis_Start_Thresh > TempCurrent_2)
				 {
						 Lysis_Tick=0;
					   Lysis_temp_reg = Lysis_Start_Thresh;
					 
						 TempDelta_2 = Lysis_Start_Thresh - TempCurrent_2;
						 if(TempDelta_2 > 10)
						 {
								flag_update_PWM = 1;
								DutyDiff = (MAX_DUTY>>1);
						 }
						 else if(TempDelta_2 < 10)
										 {
												flag_update_PWM = 1;
												PWM_Lysis_Reg =DutyDiff = 60;
										 }
				 }
				 else  // in lysis 
				 {
						TempDelta_2 = TempCurrent_2-Lysis_temp_reg;
					 
						if(Lysis_Tick < (TICK_PER_SEC - 1))
								Lysis_Tick ++;
						else
						{
								Lysis_Tick = 0;
								if(TempDelta_2 > 0.3)
								{
									flag_update_PWM = 1;
									PWM_Lysis_Reg = (PWM_Lysis_Reg>PWM_reg_floor)? (PWM_Lysis_Reg - 4): PWM_reg_floor;
								}
								else if(TempDelta_2 < 0.1)
												{
													flag_update_PWM = 1;
													PWM_Lysis_Reg = (PWM_Lysis_Reg<PWM_reg_top)? (PWM_Lysis_Reg+4):PWM_reg_top;
												}
								Lysis_temp_reg = TempCurrent_2;
								
						}
						DutyDiff=PWM_Lysis_Reg;
				 }

		#ifdef __QPCR_HW
			/// qPCR HW has limit of buty/cycle 0.97
			#ifdef __USE_IC_DRIVE
						DutyDiff= (DutyDiff>DUTY_LIMIT)? DUTY_LIMIT : DutyDiff;
			#endif
		#endif

             

             if(TempCurrent_2 >= Lysis_Stop_Thresh)
             {
                 TempCtrl_Active_2=4;
                 if(CycleSTS != READY)
                 {
                     PushMsg(SS2_TMOUT);   
                 }
             }
             else
             {
                    if(TempCurrent_2 >= Lysis_Start_Thresh)
                        trigger_out();

             }
		}
		else
		{
			
		#endif
		

		if(TempCtrl_Active_2==3)
		{
			TempTickLength_2--;
			fTemp=TempTickLength_2;
			if(!WaitOverShot)
			{
				if(TickLength_Reg_2>=TempTickLength_2)
				{
						TempSet_2=TempSet_2_Reg;
						WaitOverShot=1;
				}
			}
			if((PCR_Cycle_Control.CycleValid==CYCLE_RUN) && (trg_assert==IS_ACTIVE)) // trigger stage
			{
					fTemp--;
					if((fTemp*SAMPLE_CLOCK/10)<=integrationDur)
							trigger_out();
			}
		}
		if(TempTickLength_2==0)
		{
			if(TempCtrl_Active_2 !=4)
			{
					TempCtrl_Active_2=4;
					if(CycleSTS != READY)
					{
						PushMsg(SS2_TMOUT);	  // @0607, sensor2 timout msg push 
						if((( PCR_Cycle_Control.TotalCycle==Cycle_Check())&&(PCR_Cycle_Control.TotalStage==Stage_Check()))) // No cycle control
							PushMsg(SS2_CY_CMP);
					}
			}
		}

		if(Control_2_Dir==POS_DIR)		
				RegE_2=TempDelta_2=TempSet_2-TempCurrent_2;
		else if(Control_2_Dir==NEG_DIR)
		{
				RegE_2=TempDelta_2=TempCurrent_2-TempSet_2;
				if((TempDelta_2<Fan_Gap_temp)&&(TempCtrl_Active_2 !=4))  // to swap the current of peltier
				{
					Peltier_Swap_Msg |= SENSOR_2;
				}
		}
		
		if(TempCtrl_Active_2==2) 
		{

			//if((TempDelta_2>=-1) && (TempDelta_2<=1))  
			 // change  @2018-4-22
      if(TempDelta_2<=1)
      {         
					TempCtrl_Active_2=3;

					//////////////////////////////////////////
					#ifdef __RAMP_TEST

						if(RampFlag == ACTIVE)
						{
								RampFlag = FAN_EXT; 
								RampTimeCnt =0;
								PID_Clear(SENSOR2_INDEX);
						}

					#endif
					//////////////////////////////////////////      
       }
		}
		#ifndef __RAMP_TEST


					if(((Control_2_Dir==POS_DIR) && ( TempDelta_2< DELTA_THRES)) || ((Control_2_Dir==NEG_DIR)&&(TempDelta_2< NEG_DELTA_THRES)))
					{
						PID_Err_Rec[INDEX1].Previous_Err=PID_Err_Rec[INDEX1].Current_Err;
						PID_Err_Rec[INDEX1].Current_Err= (Control_2_Dir==POS_DIR)? (TempSet_2 -TempCurrent_2):(TempCurrent_2-TempSet_2);
						
						PID_Err_Rec[INDEX1].Previous_Derr= PID_Err_Rec[INDEX1].Current_Derr;
						PID_Err_Rec[INDEX1].Current_Derr= PID_Err_Rec[INDEX1].Current_Err-PID_Err_Rec[INDEX1].Previous_Err;
					}
		#else

					if(RampFlag != ACTIVE)
					{
							if(((Control_2_Dir==POS_DIR) && ( TempDelta_2< DELTA_THRES)) || ((Control_2_Dir==NEG_DIR)&&(TempDelta_2< NEG_DELTA_THRES)))
							{
									PID_Err_Rec[INDEX1].Previous_Err=PID_Err_Rec[INDEX1].Current_Err;
									PID_Err_Rec[INDEX1].Current_Err= (Control_2_Dir==POS_DIR)? (TempSet_2 -TempCurrent_2):(TempCurrent_2-TempSet_2);
									
									PID_Err_Rec[INDEX1].Previous_Derr= PID_Err_Rec[INDEX1].Current_Derr;
									PID_Err_Rec[INDEX1].Current_Derr= PID_Err_Rec[INDEX1].Current_Err-PID_Err_Rec[INDEX1].Previous_Err;
							}										
					}
					else
					{
											// FUZZY PID @2018-01-02
						
							if((RampTimeCnt % TICK_PER_SEC)==0) 
							{
								 if(RampTimeCnt < RampFullStep)
									 ThermoSlopRate[1] = (direction==0)? (TempPerStep * (RampTimeCnt/TICK_PER_SEC + 1) + TempStart) : (TempStart-TempPerStep * (RampTimeCnt/TICK_PER_SEC + 1));
								 else
											ThermoSlopRate[1] = TempSet_2;
							}
							RampTimeCnt ++;
							
							PID_Err_Rec[INDEX1].Previous_Err=PID_Err_Rec[INDEX1].Current_Err;
							PID_Err_Rec[INDEX1].Current_Err= (direction==0)? (ThermoSlopRate[1] -TempCurrent_2):(TempCurrent_2-ThermoSlopRate[1]);
							
							PID_Err_Rec[INDEX1].Previous_Derr= PID_Err_Rec[INDEX1].Current_Derr;
							PID_Err_Rec[INDEX1].Current_Derr= PID_Err_Rec[INDEX1].Current_Err-PID_Err_Rec[INDEX1].Previous_Err;

							/*
							TempErr[0] =  TempErr[1];
							TempErr[1] = ThermoSlopRate[1] - TempCurrent_2;
									
							TempErrDiff[0] = TempErrDiff[1];
							TempErrDiff[1] = TempErr[1] - TempErr[0];
							*/
							#ifdef __USE_FUZZY_PID
											if((PID_Err_Rec[INDEX1].Current_Err > FUZZY1_E_THRESH) || (PID_Err_Rec[INDEX1].Current_Err < -FUZZY1_E_THRESH) )
											{
													// run fuzzy 1
													u8temp = Fuzzy1Cal(PID_Err_Rec[INDEX1].Current_Err , PID_Err_Rec[INDEX1].Current_Derr);
											}
											else
											{
													// run fuzzy 2
													u8temp = OUT_RUN_PID;
													Fuzzy2Cal(PID_Err_Rec[INDEX1].Current_Err , PID_Err_Rec[INDEX1].Current_Derr);
											} 
							#endif

					}
    
        

		#endif

		if(!((TempDelta_2>= -0.25) && (TempDelta_2<= 0.25)))
		{

         #ifndef __RAMP_TEST
         
								 if(Control_2_Dir==POS_DIR)
								 {
										if( TempDelta_2< DELTA_THRES) 
											DutyDiff=PID_Cal(INDEX1,TempCurrent_2);
										else
											DutyDiff=MAX_DUTY;
								 }
								 else
								 {
										if( TempDelta_2< NEG_DELTA_THRES)
											DutyDiff=PID_Cal(INDEX1,TempCurrent_2);
										else
											DutyDiff=MAX_DUTY_NEG;
								 }
         #else

                 flag_update_PWM=1;
								 if(RampFlag != ACTIVE)
								 {
										 if(Control_2_Dir==POS_DIR)
										 {
												if( TempDelta_2< DELTA_THRES) 
													DutyDiff=PID_Cal(INDEX1,TempCurrent_2);
												else
													DutyDiff=MAX_DUTY;
										 }
										 else
										 {
												if( TempDelta_2< NEG_DELTA_THRES)
													DutyDiff=PID_Cal(INDEX1,TempCurrent_2);
												else
													DutyDiff=MAX_DUTY_NEG;
										 } 
								 }
								 else
								 { 
									 #ifndef __USE_FUZZY_PID
											DutyDiff=PID_Cal(INDEX1,TempCurrent_2);
									 #else
											switch (u8temp)
											{
													case OUT_ZO:
																	flag_update_PWM=0;
															break;

													case OUT_VS:
																	DutyDiff = DutyDiff  + 10; 
															break;

													case OUT_PS:
																	DutyDiff = DutyDiff  - 5;
															break;

													case OUT_PM:
																	DutyDiff = DutyDiff  - 10;
															break;

													case OUT_PB:
																	DutyDiff = DutyDiff  - 20;
															break;

													case OUT_VB:
																	DutyDiff = DutyDiff  - 40;
															break;

											
													case OUT_P_FLOW:
																	DutyDiff = MAX_DUTY_NEG;
															break;

											
													case OUT_N_FLOW:
																	DutyDiff = 0;
															break;

													default:
																	DutyDiff = PID_Cal_Fuzzy2(INDEX1,TempCurrent_2);
															break;

															
											}
									#endif
                                        
								 }

         #endif
				
			if((DutyDiff)>MAX_DUTY)
				 DutyDiff=MAX_DUTY;

			else if	((DutyDiff)<MIN_DUTY)
						DutyDiff=MIN_DUTY;

			#ifdef __QPCR_HW
				/// qPCR HW has limit of buty/cycle 0.97
				#ifdef __USE_IC_DRIVE
				if	((DutyDiff)>DUTY_LIMIT)
							DutyDiff=DUTY_LIMIT;
				#endif
			#endif	
		
			Diff_Reg=DutyDiff;

         #ifndef __RAMP_TEST
			flag_update_PWM=1;
		#else
             if(flag_update_PWM)
             {
         #endif
            		//////////////////////////
            			if(Control_2_Dir==NEG_DIR)
            			{
            				//DutyDiff=25;
            			}else if(DutyDiff==0)
            				{
            					if(Pwm_Pos_Lock==OPEN)
            					{
            						Pwm_Pos_Lock=SHUT;
            						GPIOA->CRH&=0XFFFFFFF0; 
            						GPIOA->CRH|=0X00000003;
            						GPIOA->ODR&=~(u32)(1<<8);
            					}
            				}else if(Pwm_Pos_Lock==SHUT)
            					{
            						Pwm_Pos_Lock=OPEN;
            						GPIOA->CRH&=0XFFFFFFF0; 
            						GPIOA->CRH|=0X0000000B;	
            					}
         #ifdef __RAMP_TEST                  
             }
         #endif
		}
		//////////////////////////
#ifdef __USE_RAM_DEBUG
		if(TempCtrl_Active_2==3)  //only dump for tjermo keeping stage
		{
				if(RamDumpFlag==0)	
				{
						RamDumpFlag=1;  // dumping
					  RAM_Dump_index=0;
					  TRG_OUT=1;
				}
				
				if(RamDumpFlag==1)  // dumping 
				{
					
						if(TickDump==0) // dump per second
						{
								TickDump=TICK_PER_SEC; 
							
								fTemper.float_num=TempCurrent_2;
								RamDumpBlock[RAM_Dump_index++]=fTemper.tempd.byte0;
								RamDumpBlock[RAM_Dump_index++]=fTemper.tempd.byte1;
								RamDumpBlock[RAM_Dump_index++]=fTemper.tempd.byte2;
								RamDumpBlock[RAM_Dump_index++]=fTemper.tempd.byte3;
								
								RamDumpBlock[RAM_Dump_index++]=Diff_Reg & 0xFF;
								RamDumpBlock[RAM_Dump_index++]=(Diff_Reg & 0xFF00)>>8;
								
								if(RAM_Dump_index==DUMPSIZE)
								{
											RamDumpFlag=2; // DUMP done, lock ram
											TRG_OUT=0;
								}
						}
						else
						{
								TickDump--;
								//TRG_OUT=0;
						}
				}
		}
		else if(RamDumpFlag==1)
					{
							RamDumpFlag=2;  // if temper stage is done before time out, also lock ram too
							TRG_OUT=0;
					}
#endif

#ifdef __USE_LYSIS   //ver1.14 @ 2018-06-23 support lysis
		
      }	// end of not in LYSIS state
#endif

		if(flag_update_PWM)
	  {
			#ifdef  __USE_TAC_SMOOTH   //test @ 2018-06-02
			    if(Control_2_Dir==POS_DIR)
				  {
							if((TempCurrent_2>60) && (TempCurrent_2<85))
								DutyDiff = (DutyDiff > TAC_MAX_DUTY)? TAC_MAX_DUTY:DutyDiff;
					}
			#endif
			
			Channel_2_Load(Control_2_Dir,DutyDiff);
		}
            //@2018-02-24
            	#ifdef __FAN_EXT_AFTER_CYCLE
              } // end of if( PCR_Cycle_Control.CycleValid == EXT_FAN)       
            #endif
		}  
}


#else


////// handler for BETA version as below

// __RPI_THERMO
void TIM6_IRQHandler(void)
{ 	

	u8 flag_update_PWM=0;
	int j=0;
	int DutyDiff=0;	
  	float fTemp=0;	
	TIM6->SR&=~(1<<0);//清除中断标志位
	rec=~rec;
	LED1=rec;


	if((TickLock & 0x1) !=0)
	{				
		RegE=TempDelta=TempSet-TempCurrent;

		#ifdef __DEBUG_BETA_ERROR   // ERR: to skip BETA error on LEON's test @2018-0401
				TempDelta=0;
		#endif
		if(TempCtrl_Active==2) 
		{
			//if((TempDelta>=-2) && (TempDelta<=2))
			if((TempDelta>=-1) && (TempDelta<=1))
			{
				TempCtrl_Active=3;
				if(CycleSTS == WAIT)   // add @0607, push sensor1 active3 msg
					PushMsg(SS1_ACT3);
			}
		}

		if( (TempDelta>0.25) || (TempDelta<-0.25))
		{
			flag_update_PWM=1;
			if((TempDelta<0)||((TempDelta>0)&&(TempDelta<DELTA_THRES)))
			{
				PID_Err_Rec[INDEX0].Previous_Err=PID_Err_Rec[INDEX0].Current_Err;
				PID_Err_Rec[INDEX0].Current_Err= TempSet -TempCurrent;
				//PID_Err_Rec[INDEX0].Inte_Err+=PID_Err_Rec[INDEX0].Current_Err;
				PID_Err_Rec[INDEX0].Previous_Derr= PID_Err_Rec[INDEX0].Current_Derr;
				PID_Err_Rec[INDEX0].Current_Derr= PID_Err_Rec[INDEX0].Current_Err-PID_Err_Rec[INDEX0].Previous_Err;
				DutyDiff=PID_Cal(INDEX0,TempCurrent);
			}
			else
				DutyDiff=MAX_DUTY;
			
			if((DutyDiff)>MAX_DUTY)
					DutyDiff=MAX_DUTY;
			else if	((DutyDiff)<MIN_DUTY)
						DutyDiff=MIN_DUTY;	
			DutyCycle=DutyDiff;
		}
		if(flag_update_PWM) TIM4->CCR3=DutyCycle;
	} 
	
	flag_update_PWM=0;
    
	if((TickLock & 0x2)!=0) 
	{	

        /////////////////////////////////////
        /// first release only support 1s as time unit
        ///
        ////////////////////////////////////
                //@2018-02-24
    #ifdef __FAN_EXT_AFTER_CYCLE
        
      if( PCR_Cycle_Control.CycleValid == EXT_FAN)
      {
            PushMsg(SS2_FX_TICK);
      }
      else
      {
    #endif

				if( PCR_Cycle_Control.CycleValid == NULL_STAGE)
				{
						if(Flag_Current_forever == 0) 
						{
								if(TempTickLength_2 > 0)
										TempTickLength_2--;
								else
										PushMsg(SS2_TMOUT);	        
						}

						
						if(trg_assert==IS_ACTIVE)
						{
								if(SnapTime_count_2 > 0)
										SnapTime_count_2 --;
								else
										trigger_out();
						}
						else if(trg_assert==IS_ASSERT)
										trigger_idle();
					
				}
				else
				{
						#ifdef __USE_1SEC_UNIT
        
        		if(TempCtrl_Active_2==3)
        		{

								if(!WaitOverShot)
								{
									 if(TempTickLength_2 > 0)
												TempTickLength_2--;

										if(TickLength_Reg_2>=TempTickLength_2)
										{
												TempSet_2=TempSet_2_Reg;
												WaitOverShot=1;
										}
                }
                else
								{  
										if(trg_assert==IS_ACTIVE)
										{

												if(SnapTime_count_2 > 0)
														SnapTime_count_2 --;
												else
														trigger_out();
										}
										else if(trg_assert==IS_ASSERT)
														trigger_idle();
										
										if(Flag_Current_forever == 0) 
										{
												TempTickLength_2--;

												if(TempTickLength_2==0)
												{
														if(TempCtrl_Active_2 !=4)
														{
																TempCtrl_Active_2=4;
																if(CycleSTS != READY)
																{
																		PushMsg(SS2_TMOUT);	  // @0607, sensor2 timout msg push 
																		if((( PCR_Cycle_Control.TotalCycle==Cycle_Check())&&(PCR_Cycle_Control.TotalStage==Stage_Check()))) // No cycle control
																				PushMsg(SS2_CY_CMP);
																}
														}
												}         
										}
										else
												TempTickLength_2 = 1;
								}
        		}
						#endif
        /////////////////////////////////////
        ////////////////////////////////////


						if(Control_2_Dir==POS_DIR)		
								RegE_2=TempDelta_2=TempSet_2-TempCurrent_2;
						else if(Control_2_Dir==NEG_DIR)
						{
										RegE_2=TempDelta_2=TempCurrent_2-TempSet_2;
							if((TempDelta_2<Fan_Gap_temp)&&(TempCtrl_Active_2 !=4))  // to swap the current of peltier
							{
								Peltier_Swap_Msg |= SENSOR_2;
							}
						}
						if(TempCtrl_Active_2==2) 
						{
							//if((TempDelta_2>=-2) && (TempDelta_2<=2))
							if((TempDelta_2>=-1) && (TempDelta_2<=1))
								TempCtrl_Active_2=3;
						}

						if(((Control_2_Dir==POS_DIR) && ( TempDelta_2< DELTA_THRES)) || ((Control_2_Dir==NEG_DIR)&&(TempDelta_2< NEG_DELTA_THRES)))
						{
							PID_Err_Rec[INDEX1].Previous_Err=PID_Err_Rec[INDEX1].Current_Err;
							PID_Err_Rec[INDEX1].Current_Err= (Control_2_Dir==POS_DIR)? (TempSet_2 -TempCurrent_2):(TempCurrent_2-TempSet_2);
							
							PID_Err_Rec[INDEX1].Previous_Derr= PID_Err_Rec[INDEX1].Current_Derr;
							PID_Err_Rec[INDEX1].Current_Derr= PID_Err_Rec[INDEX1].Current_Err-PID_Err_Rec[INDEX1].Previous_Err;
						}


						if(!((TempDelta_2>= -0.25) && (TempDelta_2<= 0.25)))
						{
							 if(Control_2_Dir==POS_DIR)
							 {
									if( TempDelta_2< DELTA_THRES) 
											DutyDiff=PID_Cal(INDEX1,TempCurrent_2);
									else
											DutyDiff=MAX_DUTY;
							 }
							 else
							 {
									if( TempDelta_2< NEG_DELTA_THRES)
											DutyDiff=PID_Cal(INDEX1,TempCurrent_2);
									else
											DutyDiff=MAX_DUTY_NEG;
							 }
								
							if((DutyDiff)>MAX_DUTY)
								 DutyDiff=MAX_DUTY;

							else if	((DutyDiff)<MIN_DUTY)
										DutyDiff=MIN_DUTY;

						
							Diff_Reg=DutyDiff;
							flag_update_PWM=1;
						
						//////////////////////////
							if(Control_2_Dir==NEG_DIR)
							{
								//DutyDiff=25;
							}else if(DutyDiff==0)
										{
												if(Pwm_Pos_Lock==OPEN)
												{
														Pwm_Pos_Lock=SHUT;
														GPIOA->CRH&=0XFFFFFFF0; 
														GPIOA->CRH|=0X00000003;
														GPIOA->ODR&=~(u32)(1<<8);
												}
										}else if(Pwm_Pos_Lock==SHUT)
													{
															Pwm_Pos_Lock=OPEN;
															GPIOA->CRH&=0XFFFFFFF0; 
															GPIOA->CRH|=0X0000000B;	
													}
							}

							if(flag_update_PWM) 
									Channel_2_Load(Control_2_Dir,DutyDiff);
	
				} // @2018-03-31  end of if(NULL_STAGE)  else
    //@2018-02-24
    #ifdef __FAN_EXT_AFTER_CYCLE
      } // end of if( PCR_Cycle_Control.CycleValid == EXT_FAN)       
    #endif
		}  
}



	void TIM3_IRQHandler(void)
	{ 
	    //u8 temp=0;
        
		TIM3->CR1&=~(u32)(0x01);		   
		TIM3->SR&=~(1<<0);//清除中断标志位

		if((TickLock & 0x2)!=0) 
		{		
			if(TempCtrl_Active_2==3)
			{

                    TempTickLength_2--;
                
                    if(!WaitOverShot)
                    {
                        	if(TickLength_Reg_2>=TempTickLength_2)
                        	{
                        		TempSet_2=TempSet_2_Reg;
                        		WaitOverShot=1;
                        	}
                    }
                    else if (trg_assert==IS_ACTIVE)
                             {
                                 SnapTime_count_2--;
                                 if(SnapTime_count_2 ==0)
                                         trigger_out();
                             }else if (trg_assert==IS_ASSERT)
                                        trigger_idle();
      

        			if(TempTickLength_2==0)
        			{
        				if(TempCtrl_Active_2 !=4)
        				{
        					TempCtrl_Active_2=4;
        					if(CycleSTS != READY)
                                  PushMsg(SS2_TMOUT);	  // @0607, sensor2 timout msg push 

        				}
        			}
             }
		}
         TIM3->CR1 |=(u32)(0x01);   
    }

#endif


float Kp[PID_SLOP_SEG_MAX]={100,300};//18;//20;
float Ki[PID_SLOP_SEG_MAX]={0.01,0.575};//0.03;
float Kd[PID_SLOP_SEG_MAX]={0.5,1};
float Kl[PID_SLOP_SEG_MAX]={0,0};

float Ktm[2]={70,70}; // mid point, temperature, dual slop

/*
u16 PID_Cal(float Delta)
{
	u16 Val;
	if(Delta>50)
	{
		Kp=20;
	}else if(Delta>25)
		{
			Kp=15;	
		}else if(Delta>10)			
			 {
			 	Kp=10;
			 }
			 else 
			 	Kp=5;
	Val=((float)Kp*Delta)/10;
	return Val;
}
*/
#ifdef __USE_PI



int PID_Cal(u8 sensor_index,float temperature)
{
	int Val;
	///////////////
	u8 idx=0;
	if(temperature>Ktm[sensor_index])
	 	idx=1;

	PID_Err_Rec[sensor_index].Inte_Err=(float)Ki[idx] * PID_Err_Rec[sensor_index].Current_Err + PID_Err_Rec[sensor_index].Inte_Err;
	if(sensor_index==0)
		Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet* Kl[idx];
	else if(sensor_index==1)
	{
		if(TempCtrl_Active_2!=3)
			Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx];
		else
			Val=(float) 150*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx];

	}
		return Val;
}

#else
#ifdef __USE_PID


#ifndef __RAMP_TEST

        int PID_Cal(u8 sensor_index,float temperature)
        {
        	int Val;
        	///////////////
        	u8 idx=0;
        	if(temperature>Ktm[sensor_index])
        	 	idx=1;

        	PID_Err_Rec[sensor_index].Inte_Err=(float)Ki[idx] * PID_Err_Rec[sensor_index].Current_Err + PID_Err_Rec[sensor_index].Inte_Err;
        	if(sensor_index==0)
        		Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        	else if(sensor_index==1)
        	{
        		Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        		if(TempCtrl_Active_2!=3)
        			Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        		else 
        		{if(TempSet_2<60)
        						Val=(float)60*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;				
        					else 
        						Val=(float)100*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        					
        		}
        	}
        		return Val;
        }
#else
    //{PB,PM,PS,ZO,NS,NM,NB}   // increas big ---> zero  --> decrease big
    float Kp_fuzzy[7]={300,200,175,150,125,100,25};//18;//20;
    float Ki_fuzzy[7]={0.75, 0.4, 0.35, 0.25, 0.15, 0.1, 0.01};//0.03;
    float Kd_fuzzy[7]={1, 0.75, 0.6, 0.5, 0.4, 0.25, 0.05};
    //float Kl_fuzzy[7]=0;

    u8 kp_idx=MIDIUM_OFFSET;
    u8 ki_idx=MIDIUM_OFFSET;
    u8 kd_idx=MIDIUM_OFFSET;
    
    int PID_Cal(u8 sensor_index,float temperature)
    {
        float Val;
			  float Tempe;
        int RVal=0;
        ///////////////
        u8 idx=0;
        if(temperature>Ktm[sensor_index])
            idx=1;

        PID_Err_Rec[sensor_index].Inte_Err=(float)Ki[idx] * PID_Err_Rec[sensor_index].Current_Err + PID_Err_Rec[sensor_index].Inte_Err;
        if(sensor_index==0)
            Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        else if(sensor_index==1)
        {
					  
						Tempe = (RampFlag == ACTIVE)? ThermoSlopRate[1] : TempSet_2;
	      /*		   
            	//Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + Tempe* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
            if(TempCtrl_Active_2!=3)
            	Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + Tempe* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr + (loop-1)*PID_Err_Rec[sensor_index].Inte_Err;
            else 
            {if(TempSet_2<60)
                            Val=(float)60*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;             
                        else 
                            Val=(float)100*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
                        
            }
          */


                		Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        		if(TempCtrl_Active_2!=3)
        			Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        		else 
        		{if(TempSet_2<60)
        						Val=(float)60*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;				
        					else 
        						Val=(float)100*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
        					
        		}
        }
        RVal = (Val < 1)? 0: (int)Val; 
				return RVal;
    }


    int PID_Cal_Fuzzy2(u8 sensor_index,float temperature)
    {
        float Val;
              float Tempe;
        int RVal=0;
        ///////////////
        u8 idx=0;

        
        if(sensor_index==1)
        {
                      
            Tempe = (RampFlag == ACTIVE)? ThermoSlopRate[1] : TempSet_2;
                       
                //Val=(float)Kp[idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + Tempe* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
            if(TempCtrl_Active_2!=3)
            {
                PID_Err_Rec[sensor_index].Inte_Err=(float)Ki_fuzzy[ki_idx] * PID_Err_Rec[sensor_index].Current_Err + PID_Err_Rec[sensor_index].Inte_Err;
                Val=(float)Kp_fuzzy[kp_idx]*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + Kd_fuzzy[kd_idx]*PID_Err_Rec[sensor_index].Current_Derr;
            }
            else 
            {
                PID_Err_Rec[sensor_index].Inte_Err=(float)Ki[idx] * PID_Err_Rec[sensor_index].Current_Err + PID_Err_Rec[sensor_index].Inte_Err;
                if(TempSet_2<60)
                    Val=(float)60*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;             
                else 
                    Val=(float)100*PID_Err_Rec[sensor_index].Current_Err+PID_Err_Rec[sensor_index].Inte_Err + TempSet_2* Kl[idx]+Kd[idx]*PID_Err_Rec[sensor_index].Current_Derr;
                        
            }
        }
        RVal = (Val < 1)? 0: (int)Val; 
                return RVal;
    }


#endif



#endif
#endif

void Channel_Swap(u8 sensor_num)
{
 	if(sensor_num==SENSOR_2)  // only sensor_2 is valid now
	{
		
		TIM6->DIER&= ~0x1;
		Peltier_Swap_Msg &=  ~SENSOR_2;
		if(Control_2_Dir==NEG_DIR)
		{
			PID_Clear(SENSOR2_INDEX);
			Control_2_Dir=POS_DIR;
			PWM1_Init();
			Channel2_PWM_Start(Control_2_Dir);
#ifndef  __FAN_ALWAYS_ON   // version 1.6 @2017-04-15: request to enable FAN in cycle
			
			FanCtrl_Auto(0);  // auto stop fan in case fan runing in cooling stage	
#endif
		}
		TIM6->DIER|= 0x1;	

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


#ifdef __SENSOR_TMP100
bool Sensor_Cfg_Write(u8 idx, u8 dt)
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
				//IIC_Start();
				//IIC_Send_Byte(Sensor_Addr[idx]);
				//if(IIC_Wait_Ack()==0)
				{
					IIC_Send_Byte(dt);
					IIC_Stop();
					TMP_NACK=0;
				}
				//else
				//	TMP_NACK++;	
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

u8 Sensor_Cfg_Read(u8 idx)
{
   u8 dt,TMP_NACK;

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
				IIC_Start();
				IIC_Send_Byte((Sensor_Addr[idx] |0x1));
				if(IIC_Wait_Ack()==0)
				{
					dt=IIC_Read_Byte(0);
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
#endif

#ifdef __SENSOR_TMP112
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
				//IIC_Start();
				//IIC_Send_Byte(Sensor_Addr[idx]);
				//if(IIC_Wait_Ack()==0)
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
				//else
				//	TMP_NACK++;	
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
#endif

void Sensor_Res_Init(void)
{
	u8 i;
	#ifdef __SENSOR_TMP100
	u8 u8temp;
	#else 
	#ifdef __SENSOR_TMP112
	u16 u16temp;
	#endif
	#endif

	for(i=0;i<SENSOR_TOTAL;i++)
	{
		#ifdef __SENSOR_TMP100
		u8temp=Sensor_Cfg_Read(i);
		u8temp &= ~SENSOR_CFG_MSK;
		u8temp |= SENSOR_R1_R0;
		Sensor_Cfg_Write(i,u8temp);	
	    //u8temp=Sensor_Cfg_Read(SENSOR2_INDEX);
		#else 
		#ifdef __SENSOR_TMP112
		u16temp=Sensor_Cfg_Read(i);
		u16temp |= (SENSOR_CR1_CR0<<6);
		Sensor_Cfg_Write(i,u16temp);
		#endif
		#endif
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

u8 trg_assert=IS_IDLE;

#ifdef __RPI_THERMO

		void Cycle_null_stage(void)
		{
        ClearMsg();
        TempCtrl_Active_2=0;
        TempValid &= ~(1<<1);
        TempControl_2_stop();
        //trigger_reset();
        RampTimeCnt=0;   

				PCR_Cycle_Control.CycleValid=NULL_STAGE; 
				CycleSTS=NULL_STAGE;

				Tick_Init();
				TickLock |= (1<<1);
		}

    void Cycle_Stop_BETA(u8 FanExt)
    {
        ClearMsg();
        TempCtrl_Active_2=0;
        TempValid =0;
        TempControl_2_stop();
        TempControl_stop();
        trigger_reset();
        RampTimeCnt=0;   

        TempCurrent_2_reg=TempCurrent_reg=0;
        #ifdef __FAN_EXT_AFTER_CYCLE
            if(FanExt)
            {
                PCR_Cycle_Control.CycleValid=EXT_FAN; 
                CycleSTS=FAN_EXT;
                FanCtrl_Auto(1); 

                //@2018-02-24
    			Tick_Init();
    			TickLock |= (1<<1);
    			///////////////////
            }
            else
            {
               PCR_Cycle_Control.CycleValid=INVALID;
               FanCtrl_Auto(0);
               CycleSTS=READY;
            }
        #else                   
            PCR_Cycle_Control.CycleValid=INVALID;
            FanCtrl_Auto(0);
            CycleSTS=READY;
        #endif                       
    }


	void MsgHandler_Beta(void)
	{
	    u8 temp=0;
		if(MsgStk !=EMPTY)
		{
			TickLock &= ~0x2;
			switch (PCR_Cycle_Control.CycleValid)
			{
				case INIT:
									if(AssertMsg(SS1_ACT3))
									{
											ClearMsg();
											RpiSetting.ThermoStatus=0;
											
											if(RpiSetting.RpiSetIdx & 1) // 
													TempCtrl_Reload(1);
											
											if(RpiSetting.ThermoStatus !=0)
											{
													if(PCR_Cycle_SetPoint[0].SetPoint.float_num != -1000)
													{
															PCR_Cycle_Control.CycleValid=CYCLE_RUN;//PUMP_RUN;
															CycleSTS=ACTIVE;
															TickLock |= 0x2;
															#ifdef __FAN_ALWAYS_ON  // version 1.6 @2017-04-15: always enable FAN in cycle
																	FanCtrl_Auto(FAN_CTRL_MSK);
															#endif
													}
													else
															Cycle_null_stage();

											}
											else
											{
													Set_Cycle_Run_Status(RUN_REJECT);
												
													#ifndef __FAN_EXT_AFTER_CYCLE 
															Cycle_Stop_BETA(0);
													#else
															Cycle_Stop_BETA(1);
													#endif
											}
									}
							break;

				case CYCLE_RUN:
					
									if(AssertMsg(SS2_TMOUT))
									{
										  ClearMsg();
											RpiSetting.ThermoStatus=0;    

											if(RpiSetting.RpiSetIdx & 1)
														TempCtrl_Reload(1);
											
											if(RpiSetting.ThermoStatus!=0)  
											{
													if(PCR_Cycle_SetPoint[0].SetPoint.float_num != -1000)
															Cycle_null_stage();
											}	
											else		// cycle all done
											{	
													Set_Cycle_Run_Status(RUN_REJECT);
													#ifndef __FAN_EXT_AFTER_CYCLE 
															Cycle_Stop_BETA(0);
													#else
															Cycle_Stop_BETA(1);
													#endif
											}

									}

									if(trg_assert==IS_ASSERT)
											trigger_idle();
						break;

	#ifdef __FAN_EXT_AFTER_CYCLE						 
				case 	EXT_FAN:
							/*
							TempCurrent_2=TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
							if(TempCurrent_2 < FAN_EXT_COOL_THRES)
							{
									PCR_Cycle_Control.CycleValid=INVALID;
									CycleSTS=READY;
									FanCtrl_Auto(0);	
							}
                            */
                          //@2018-02-24
                          if(AssertMsg(SS2_TMOUT))
                          {
                             ClearMsg();
        
                             TempCurrent_2=TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
                             if(TempCurrent_2 < FAN_EXT_COOL_THRES)
                             {
                                  PCR_Cycle_Control.CycleValid=VALID;
                                  CycleSTS=READY;
                                  TempControl_2_stop();
                                  // clear TickLock , and stop timer6
                             }
                         }
                          
						break;
	#endif
												 
				////// @2018-03-31: new add to support -1000 cycle control
				case NULL_STAGE:
					
										if(AssertMsg(SS2_TMOUT))
										{
												ClearMsg();
												RpiSetting.ThermoStatus=0;    

												if(RpiSetting.RpiSetIdx & 1)
															TempCtrl_Reload(1);
												
												if(RpiSetting.ThermoStatus !=0)
												{
														if(PCR_Cycle_SetPoint[0].SetPoint.float_num != -1000)
														{
																PCR_Cycle_Control.CycleValid=CYCLE_RUN;
																CycleSTS=ACTIVE;
																TickLock |= 0x2;
																#ifdef __FAN_ALWAYS_ON  // version 1.6 @2017-04-15: always enable FAN in cycle
																		FanCtrl_Auto(FAN_CTRL_MSK);
																#endif
														}
														else
																Cycle_null_stage();
												}
												else
												{

														#ifndef __FAN_EXT_AFTER_CYCLE 
																Cycle_Stop_BETA(0);
														#else
																Cycle_Stop_BETA(1);
														#endif
												}

										}

										if(trg_assert==IS_ASSERT)
												trigger_idle();
						break;
												 
				default:
		                    Cycle_Stop_BETA(0);
						break;
			}
            
            if(RpiSetting.ThermoStatus!=0)
                        TickLock |= 0x2;
		}
	}

    /*
    float RampRateCal(float temperature, u32 tickcnt)
        {
            float ftemp=0;
            ftemp= (THERMO_PSC_VAL+1) / SYS_CLK_M / 1000000;  // get real tick time
            ftemp = ftemp * (float)tickcnt;
            return (temperature / ftemp);                    // C/ sec
        }
    */
#else


    #ifdef __RAMP_TEST
        /*
        float RampRateCal(float temperature, u32 tickcnt)
        {
            float ftemp=0;
            // get real tick time
            ftemp = TICK_PER_SEC * (float)tickcnt;
            return (temperature / ftemp);                    // C/ sec
        }
        */
        //// fuzzy version 

        
        u8 Fuzzy1Cal(float err , float errdiff)
        {
            u8  err_delta, errDiff_delta;
            u8  u1=0;;
            
            if(err < NB_THRES) 
            {
                return OUT_N_FLOW;
            }
            else if(err < NM_THRES)
                    err_delta = NB;
                 else if(err < NS_THRES)
                            err_delta = NM; 
                     else if(err < ZO_THRES)
                                err_delta = NS;
                             else if(err < -ZO_THRES)
                                        err_delta = ZO;
                                 else if(err < PS_THRES)
                                            err_delta = PS;
                                     else if(err < PM_THRES)
                                                err_delta = PM;
                                            else  if(err < PB_THRES)
                                                    err_delta = PB;  
                                                  else
                                                        return OUT_P_FLOW;



            if(errdiff < NB_THRES) 
            {
                return OUT_N_FLOW;
            }
            else if(errdiff < NM_THRES)
                    errDiff_delta = NB;
                 else if(errdiff < NS_THRES)
                            errDiff_delta = NM; 
                     else if(errdiff < ZO_THRES)
                                errDiff_delta = NS;
                             else if(errdiff < -ZO_THRES)
                                        errDiff_delta = ZO;
                                 else if(errdiff < PS_THRES)
                                            errDiff_delta = PS;
                                     else if(errdiff < PM_THRES)
                                                errDiff_delta = PM;
                                            else  if(errdiff < PB_THRES)
                                                    errDiff_delta = PB;  
                                                  else
                                                        return OUT_P_FLOW;



           switch(err_delta)
           {
                    case NB:
                                switch(errDiff_delta)
                                {
                                    case NB:
                                            u1 = OUT_VB; 
                                        break;
                                    case NM:
                                            u1 = OUT_VB; 
                                        break;
                                    case NS:
                                            u1 = OUT_PB; 
                                        break;
                                    case ZO:
                                            u1 = OUT_PB; 
                                        break;
                                    case PS:
                                            u1 = OUT_PM; 
                                        break;
                        
                                    case PM:
                                            u1 = OUT_PM; 
                                        break;
            
                                    default:
                                            u1 = OUT_PS; 
                                        break;
                                }
                        
            
                            
                        break;
            
                    case NM:
                                switch(errDiff_delta)
                                  {
                                      case NB:
                                              u1 = OUT_VB; 
                                          break;
                                      case NM:
                                              u1 = OUT_PB; 
                                          break;
                                      case NS:
                                              u1 = OUT_PB; 
                                          break;
                                      case ZO:
                                              u1 = OUT_PM; 
                                          break;
                                      case PS:
                                              u1 = OUT_PM; 
                                          break;
                                
                                      case PM:
                                              u1 = OUT_PS; 
                                          break;
                                
                                      default:
                                              u1 = OUT_PS; 
                                          break;
                                  }
            
                            break;
                    case NS:
                               switch(errDiff_delta)
                                {
                                    case NB:
                                            u1 = OUT_VB; 
                                        break;
                                    case NM:
                                            u1 = OUT_PB; 
                                        break;
                                    case NS:
                                            u1 = OUT_PM; 
                                        break;
                                    case ZO:
                                            u1 = OUT_PM; 
                                        break;
                                    case PS:
                                            u1 = OUT_PS; 
                                        break;
                        
                                    case PM:
                                            u1 = OUT_PS; 
                                        break;
            
                                    default:
                                            u1 = OUT_PS; 
                                        break;
                                }               
                        break;
                    
                    case ZO:
                              switch(errDiff_delta)
                                {
                                    case NB:
                                            u1 = OUT_PB; 
                                        break;
                                    case NM:
                                            u1 = OUT_PM; 
                                        break;
                                    case NS:
                                            u1 = OUT_PM; 
                                        break;
                                    case ZO:
                                            u1 = OUT_PS; 
                                        break;
                                    case PS:
                                            u1 = OUT_PS; 
                                        break;
                        
                                    case PM:
                                            u1 = OUT_VS; 
                                        break;
            
                                    default:
                                            u1 = OUT_VS; 
                                        break;
                                }                
                        break;
                    case PS:
                              switch(errDiff_delta)
                                {
                                    case NB:
                                            u1 = OUT_PM; 
                                        break;
                                    case NM:
                                            u1 = OUT_PM; 
                                        break;
                                    case NS:
                                            u1 = OUT_PS; 
                                        break;
                                    case ZO:
                                            u1 = OUT_PS; 
                                        break;
                                    case PS:
                                            u1 = OUT_VS; 
                                        break;
                        
                                    case PM:
                                            u1 = OUT_VS; 
                                        break;
            
                                    default:
                                            u1 = OUT_ZO; 
                                        break;
                                }                
                        break;
                    
                    case PM:
                              switch(errDiff_delta)
                                {
                                    case NB:
                                            u1 = OUT_PS; 
                                        break;
                                    case NM:
                                            u1 = OUT_PS; 
                                        break;
                                    case NS:
                                            u1 = OUT_VS; 
                                        break;
                                    case ZO:
                                            u1 = OUT_VS; 
                                        break;
                                    case PS:
                                            u1 = OUT_VS; 
                                        break;
                        
                                    case PM:
                                            u1 = OUT_ZO; 
                                        break;
            
                                    default:
                                            u1 = OUT_ZO; 
                                        break;
                                }                
                        break;
                    default:
                              switch(errDiff_delta)
                                {
                                    case NB:
                                            u1 = OUT_PS; 
                                        break;
                                    case NM:
                                            u1 = OUT_VS; 
                                        break;
                                    case NS:
                                            u1 = OUT_VS; 
                                        break;
                                    case ZO:
                                            u1 = OUT_VS; 
                                        break;
                                    case PS:
                                            u1 = OUT_ZO; 
                                        break;
                        
                                    case PM:
                                            u1 = OUT_ZO; 
                                        break;
            
                                    default:
                                            u1 = OUT_ZO; 
                                        break;
                                }                    
                        break;                        
            
                        
           }
           return u1;
    }



    void Fuzzy2Cal(float err , float errdiff)
    {
        u8  err_delta, errDiff_delta;
        
          if(err < F2_NM_THRES)
                err_delta = NB;
             else if(err < F2_NS_THRES)
                        err_delta = NM; 
                 else if(err < F2_ZO_THRES)
                            err_delta = NS;
                         else if(err < -F2_ZO_THRES)
                                    err_delta = ZO;
                             else if(err < F2_PS_THRES)
                                        err_delta = PS;
                                 else if(err < F2_PM_THRES)
                                            err_delta = PM;
                                        else  
                                            err_delta = PB;  
    
    
    
          if(errdiff < F2_NM_THRES)
                errDiff_delta = NB;
             else if(errdiff < F2_NS_THRES)
                        errDiff_delta = NM; 
                 else if(errdiff < F2_ZO_THRES)
                            errDiff_delta = NS;
                         else if(errdiff < -F2_ZO_THRES)
                                    errDiff_delta = ZO;
                             else if(errdiff < F2_PS_THRES)
                                        errDiff_delta = PS;
                                 else if(errdiff < F2_PM_THRES)
                                            errDiff_delta = PM;
                                        else  
                                            errDiff_delta = PB;  
    
    
    
       switch(err_delta)
       {
                case NB:
                            switch(errDiff_delta)
                            {
                                case NB:
                                        kp_idx = PB; 
                                        ki_idx = NB;
                                        kd_idx = PB;
                                    break;
                                case NM:
                                        kp_idx = PB; 
                                        ki_idx = NB;
                                        kd_idx = PM;

                                    break;
                                case NS:
                                        kp_idx = PB; 
                                        ki_idx = NB;
                                        kd_idx = PS;

                                    break;
                                case ZO:
                                        kp_idx = PB; 
                                        ki_idx = NB;
                                        kd_idx = PS; 
                                    break;
                                case PS:
                                        kp_idx = PB; 
                                        ki_idx = NB;
                                        kd_idx = PS;

                                    break;
                    
                                case PM:
                                        kp_idx = PB; 
                                        ki_idx = NB;
                                        kd_idx = PM;

                                    break;

                                default:
                                        kp_idx = PB; 
                                        ki_idx = NB;
                                        kd_idx = PB;

                                    break;
                            }
                    

                        
                    break;

                case NM:
                            switch(errDiff_delta)
                              {
                                  case NB:
                                      kp_idx = PM; 
                                      ki_idx = NM;
                                      kd_idx = NS;

                                      break;
                                  case NM:
                                      kp_idx = PM; 
                                      ki_idx = NM;
                                      kd_idx = NS;


                                      break;
                                  case NS:
                                      kp_idx = PM; 
                                      ki_idx = NM;
                                      kd_idx = NS;


                                      break;
                                  case ZO:
                                      kp_idx = PM; 
                                      ki_idx = NM;
                                      kd_idx = NS;

                                      break;
                                  case PS:
                                      kp_idx = PM; 
                                      ki_idx = NM;
                                      kd_idx = NS;

                                      break;
                            
                                  case PM:
                                      kp_idx = PM; 
                                      ki_idx = NM;
                                      kd_idx = NS;


                                      break;
                            
                                  default:
                                      kp_idx = PB; 
                                      ki_idx = NM;
                                      kd_idx = PM;

                                      break;
                              }

                        break;
                case NS:
                           switch(errDiff_delta)
                            {
                                case NB:
                                    kp_idx = PS; 
                                    ki_idx = NS;
                                    kd_idx = PS;

                                    break;
                                case NM:
                                    kp_idx = PS; 
                                    ki_idx = ZO;
                                    kd_idx = NS;


                                    break;
                                case NS:
                                    kp_idx = PS; 
                                    ki_idx = PM;
                                    kd_idx = NM;


                                    break;
                                case ZO:
                                    kp_idx = PS; 
                                    ki_idx = PM;
                                    kd_idx = PM;


                                    break;
                                case PS:
                                    kp_idx = PS; 
                                    ki_idx = PM;
                                    kd_idx = NM;

                                    break;
                    
                                case PM:
                                    kp_idx = PS; 
                                    ki_idx = ZO;
                                    kd_idx = NS;


                                    break;

                                default:
                                    kp_idx = PS; 
                                    ki_idx = NS;
                                    kd_idx = PS;

                                    break;
                            }               
                    break;
                
                case ZO:
                          switch(errDiff_delta)
                            {
                                case NB:
                                    kp_idx = PS; 
                                    ki_idx = ZO;
                                    kd_idx = PS;

                                    break;
                                case NM:
                                    kp_idx = ZO; 
                                    ki_idx = PS;
                                    kd_idx = NS;


                                    break;
                                case NS:
                                    kp_idx = ZO; 
                                    ki_idx = PM;
                                    kd_idx = NM;

                                    break;
                                case ZO:
                                    kp_idx = NS; 
                                    ki_idx = PB;
                                    kd_idx = NM;

                                    break;
                                case PS:
                                    kp_idx = ZO; 
                                    ki_idx = PM;
                                    kd_idx = NM;

                                    break;
                    
                                case PM:
                                    kp_idx = ZO; 
                                    ki_idx = PS;
                                    kd_idx = NS;

                                    break;

                                default:
                                    kp_idx = PS; 
                                    ki_idx = ZO;
                                    kd_idx = PS;

                                    break;
                            }                
                    break;
                case PS:
                          switch(errDiff_delta)
                            {
                                case NB:
                                        kp_idx = PS; 
                                        ki_idx = PM;
                                        kd_idx = PS; 
                                    break;
                                case NM:
                                    kp_idx = PS; 
                                    ki_idx = ZO;
                                    kd_idx = PS; 


                                    break;
                                case NS:
                                    kp_idx = PS; 
                                    ki_idx = PM;
                                    kd_idx = NM; 


                                    break;
                                case ZO:
                                    kp_idx = PS; 
                                    ki_idx = PM;
                                    kd_idx = PM; 

                                    break;
                                case PS:
                                    kp_idx = PS; 
                                    ki_idx = PM;
                                    kd_idx = NM; 

                                    break;
                    
                                case PM:
                                    kp_idx = PS; 
                                    ki_idx = ZO;
                                    kd_idx = NS; 

                                    break;

                                default:
                                    kp_idx = PS; 
                                    ki_idx = NS;
                                    kd_idx = PS;

                                    break;
                            }                
                    break;
                
                case PM:
                          switch(errDiff_delta)
                            {
                                case NB:
                                        kp_idx = PM; 
                                        ki_idx = NM;
                                        kd_idx = PM; 
                                    break;
                                case NM:
                                    kp_idx = PM; 
                                    ki_idx = NM;
                                    kd_idx = NS; 


                                    break;
                                case NS:
                                    kp_idx = PM; 
                                    ki_idx = NM;
                                    kd_idx = NS; 

                                    break;
                                case ZO:
                                    kp_idx = PM; 
                                    ki_idx = NM;
                                    kd_idx = NS; 

                                    break;
                                case PS:
                                    kp_idx = PM; 
                                    ki_idx = NM;
                                    kd_idx = NS; 

                                    break;
                    
                                case PM:
                                    kp_idx = PM; 
                                    ki_idx = NM;
                                    kd_idx = NS; 

                                    break;

                                default:
                                    kp_idx = PB; 
                                    ki_idx = NM;
                                    kd_idx = NS; 


                                    break;
                            }                 
                    break;
                default:
                    switch(errDiff_delta)
                      {
                          case NB:
                                  kp_idx = PB; 
                                  ki_idx = NB;
                                  kd_idx = PB; 
                              break;
                          case NM:
                              kp_idx = PM; 
                              ki_idx = NB;
                              kd_idx = PM;
                    
                              break;
                          case NS:
                              kp_idx = PS; 
                              ki_idx = NB;
                              kd_idx = PS;
                    
                              break;
                          case ZO:
                                  kp_idx = PS; 
                                  ki_idx = NB;
                                  kd_idx = PS; 
                              break;
                          case PS:
                              kp_idx = PS; 
                              ki_idx = NB;
                              kd_idx = PS; 

                              break;
                    
                          case PM:
                                  kp_idx = PM; 
                                  ki_idx = NB;
                                  kd_idx = PM; 
                              break;
                    
                          default:
                              kp_idx = PB; 
                              ki_idx = NB;
                              kd_idx = PB;
                    
                              break;
                      }   

                                                
                    break;                             
           }
    }





    #endif


	//add @2018-04-22 accident brake protection
	void Brake_assert(void)
	{

			ClearMsg();
			//PCR_Cycle_Control.CycleValid=INVALID;
			TempCtrl_Active_2=0;
			TempValid =0;
			TempControl_2_stop();
			TempControl_stop();
			trigger_reset();
			TempCurrent_2_reg=TempCurrent_reg=0;
		 
#ifdef __FAN_EXT_AFTER_CYCLE					 
			PCR_Cycle_Control.CycleValid=EXT_FAN; 
			CycleSTS=FAN_EXT;
			FanCtrl_Auto(1);
		 
			 //@2018-02-24
			 Tick_Init();
			 TickLock |= (1<<1);
			 ///////////////////
#else					
			PCR_Cycle_Control.CycleValid=VALID;
			CycleSTS=READY;
#endif						 

		if(trg_assert==IS_ASSERT)
				trigger_idle();
	
	}


	void MsgHandler(void)
	{
		u8 u8temp=TickLock;
		//if(MsgStk !=EMPTY)  // marked @2018-07-22 to stop lysis in any time
		{
			TickLock &= ~0x2;
			switch (PCR_Cycle_Control.CycleValid)
			{
				case INIT:
						if(AssertMsg(SS1_ACT3))
						{
							ClearMsg();
							SetPoint_PrePump_Start();
							PCR_Cycle_Control.CycleValid=PUMP_RUN;		
	#ifdef __FAN_ALWAYS_ON  // version 1.6 @2017-04-15: always enable FAN in cycle
			FanCtrl_Auto(FAN_CTRL_MSK);	
	#endif				
						}
					break;
				case PUMP_RUN:
						if(AssertMsg(SS2_TMOUT))
						{
							ClearMsg();
							SetPoint_Cycle_Start(PCR_Cycle_SetPoint);
							PCR_Cycle_Control.CycleValid=CYCLE_RUN;
							CycleSTS=ACTIVE;

                        #ifdef __HAS_CONFIGURABLE_SNAP    // if flexible configue
                                                      
                              if(PCR_Cycle_Control.HasSnapCfg == 1)   // has setting
                              {
                                 if(PCR_Cycle_SetPoint[0].SnapValid == 1)
                                 {
                                    if(PcrMskReg != 0)  
								 {
									if(trg_assert==IS_IDLE)
										trigger_active();
								 }
                                 }
                              }
                              
                        #endif

                            
						}
					break;
				case CYCLE_RUN:			  
						if(AssertMsg(SS2_CY_CMP))
						{
							ClearMsg();
							SetPoint_Extension_Start();
							PCR_Cycle_Control.CycleValid=EXT_RUN;
		
						}else if(AssertMsg(SS2_TMOUT))
							{
								ClearMsg();
								TickLock &= ~0x2;
								if(PCR_Cycle_Control.TotalStage==Stage_Check())
								{				
									Cycle_INC();
									Stage_Set(0);
									
									
										trigger_reset();
								}
								else 
								{
										Stage_INC();	

                                    #ifdef __HAS_CONFIGURABLE_SNAP    

                                        if(PCR_Cycle_Control.HasSnapCfg == 0)   // if no flexible configue
                                        {
                                            if(PCR_Cycle_Control.TotalStage==Stage_Check())
    									    {
        										if(PcrMskReg != 0)  
        										{
        											if(trg_assert==IS_IDLE)            												
        												trigger_active();
        										}
    									    }	
    									    else if(trg_assert!=IS_IDLE)
    												trigger_reset();
                                        }
                                        
                                    #else
                                         
									if(PCR_Cycle_Control.TotalStage==Stage_Check())
									{
        										if(PcrMskReg != 0)  
        										{
        											if(trg_assert==IS_IDLE)
        												//trg_assert=IS_ACTIVE; 
        												trigger_active();
        										}
									}	
									else if(trg_assert!=IS_IDLE)
													trigger_reset();
                                    #endif
									////////////////////////////////////////////////////////
									
								 
								}

/*
                                    #ifdef __HAS_CONFIGURABLE_SNAP    
                                
                                            if(PCR_Cycle_Control.HasSnapCfg == 1)   // if flexible configue
                                            {
    
                                                if(PCR_Cycle_SetPoint[Current_Stage].SnapValid == 1)
                                                {
                                                    if(PcrMskReg != 0)  
                                                    {
                                                        if(trg_assert==IS_IDLE)
                                                            trigger_active();
                                                    }
                                                }
    
                                            }
                                                                        
                                    #endif
*/
                                
								TempCtrl_Reload_InCycle();


                                
								TickLock |= 0x2;
							}
					break;
					
				case EXT_RUN:
						 if(AssertMsg(SS2_TMOUT))
						 {
								ClearMsg();
								//PCR_Cycle_Control.CycleValid=INVALID;

	    	#ifdef __USE_LYSIS		// @2018-06-23 version 1.14 for lysis
                                   if(Lysis_request > READY)
                                   {
                                        Lysis_request=ACTIVE;
                                        Lysis_Tick=0;
        								CycleSTS=LYSIS_RUN;
        								FanCtrl_Auto(1);	

                                        				PE_Cycle_SetPoint[0].SetPoint.float_num = Lysis_Stop_Thresh;
                                        
        								SetPoint_PrePump_Start();                   // presume pump point is higher than lysis
        								PCR_Cycle_Control.CycleValid=LYSIS;
                                   }
                                   else
                                   {
                                       TempCtrl_Active_2=0;
                                       TempValid =0;
                                       TempControl_2_stop();
                                       TempControl_stop();
                                       trigger_reset();
                                       TempCurrent_2_reg=TempCurrent_reg=0;
                                #ifdef __FAN_EXT_AFTER_CYCLE                     
                                               PCR_Cycle_Control.CycleValid=EXT_FAN; 
                                               CycleSTS=FAN_EXT;
                                               FanCtrl_Auto(1);
                                            
                                                //@2018-02-24
                                                Tick_Init();
                                                TickLock |= (1<<1);
                                                ///////////////////
                                #else                   
                                               PCR_Cycle_Control.CycleValid=VALID;
                                               CycleSTS=READY;
                                #endif

                                   }
		#else

								TempCtrl_Active_2=0;
								TempValid =0;
								TempControl_2_stop();
								TempControl_stop();
								trigger_reset();
								TempCurrent_2_reg=TempCurrent_reg=0;
				#ifdef __FAN_EXT_AFTER_CYCLE					 
										PCR_Cycle_Control.CycleValid=EXT_FAN; 
										CycleSTS=FAN_EXT;
										FanCtrl_Auto(1);
									 
										 //@2018-02-24
										 Tick_Init();
										 TickLock |= (1<<1);
										 ///////////////////
				#else					
										PCR_Cycle_Control.CycleValid=VALID;
										CycleSTS=READY;
				#endif
		#endif
						 }
							if(trg_assert==IS_ASSERT)
									trigger_idle();
						break;

	#ifdef __FAN_EXT_AFTER_CYCLE						 
				case 	EXT_FAN:
									//@2018-02-24
									if(AssertMsg(SS2_FX_TICK))
									{
												 ClearMsg();

        							   TempCurrent_2=TempSensorRead(TEMP_SENSOR_2_ADDR_WR);
        					       if(TempCurrent_2 < FAN_EXT_COOL_THRES)
        							   {
														PCR_Cycle_Control.CycleValid=VALID;
														CycleSTS=READY;
        								    TempControl_2_stop();
                                // clear TickLock , and stop timer6
        							   }
                  }
						break;
	#endif	


	#ifdef __USE_LYSIS

        case LYSIS_PENDING:
					
						if(Lysis_request == READY)
						{
								Lysis_request=READY;
								TempCtrl_Active_2=0;
								TempValid =0;
								TempControl_2_stop();
								TempControl_stop();
								trigger_reset();
								TempCurrent_2_reg=TempCurrent_reg=0;
							
				 #ifdef __FAN_EXT_AFTER_CYCLE					 
								PCR_Cycle_Control.CycleValid=EXT_FAN; 
								CycleSTS=FAN_EXT;
								FanCtrl_Auto(1);
							 
								 //@2018-02-24
								 Tick_Init();
								 TickLock |= (1<<1);
								 ///////////////////
																			 #else					
								PCR_Cycle_Control.CycleValid=VALID;
								CycleSTS=READY;
				 #endif

						}			
				
            if(AssertMsg(SS1_ACT3))
						{
							ClearMsg();
							if(Lysis_request != READY)
							{
									Lysis_request=ACTIVE;
									Lysis_Tick=0;
									CycleSTS=LYSIS_RUN;
									FanCtrl_Auto(1);														
									SetPoint_PrePump_Start();                   // presume pump point is higher than lysis
									
									TempSet_2= Lysis_Stop_Thresh + Temper_OverShot;  // change ther target temperature to LYSIS setting
									TempSet_2_Reg=Lysis_Stop_Thresh;
																	
									PCR_Cycle_Control.CycleValid=LYSIS;
							}
						} 
						
					break;
                    

				case LYSIS:

                          if(trg_assert!=IS_IDLE)
							trigger_reset();
						
						if(Lysis_request == READY)
						{
								Lysis_request=READY;
								TempCtrl_Active_2=0;
								TempValid =0;
								TempControl_2_stop();
								TempControl_stop();
								trigger_reset();
								TempCurrent_2_reg=TempCurrent_reg=0;
							
				 #ifdef __FAN_EXT_AFTER_CYCLE					 
								PCR_Cycle_Control.CycleValid=EXT_FAN; 
								CycleSTS=FAN_EXT;
								FanCtrl_Auto(1);
							 
								 //@2018-02-24
								 Tick_Init();
								 TickLock |= (1<<1);
								 ///////////////////
																			 #else					
								PCR_Cycle_Control.CycleValid=VALID;
								CycleSTS=READY;
				 #endif

						}	
						
						if(AssertMsg(SS2_TMOUT))
						{
						    Lysis_request=READY;
							ClearMsg();
							TempCtrl_Active_2=0;
							TempValid =0;
							TempControl_2_stop();
							TempControl_stop();
							trigger_reset();
							TempCurrent_2_reg=TempCurrent_reg=0;
	                                       #ifdef __FAN_EXT_AFTER_CYCLE					 
									PCR_Cycle_Control.CycleValid=EXT_FAN; 
									CycleSTS=FAN_EXT;
									FanCtrl_Auto(1);
								 
									 //@2018-02-24
									 Tick_Init();
									 TickLock |= (1<<1);
									 ///////////////////
	                                       #else					
									PCR_Cycle_Control.CycleValid=VALID;
									CycleSTS=READY;
	                                       #endif

						}

					break;
	#endif
				default:
		
						break;
			}
			TickLock=u8temp;
		}
	}

#endif


void PopMsg(u8 msk)
{
	MsgStk &= ~msk;
}

void PushMsg(u8 msk)
{
	MsgStk |= msk;
}
