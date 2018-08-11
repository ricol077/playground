#include "sys.h"	
#include "delay.h"	 	 
#include "rtc.h"	 	 
#include "adc.h" 	 	 
#include "dma.h" 	 
#include "spi.h"
#include "user_def.h"
#include "PCR_Cycle.h"
#include "Temperature.h"
//#include "PID_LIB.lib"
u8 PIX_Buf[16]={0};
u8 REG_Buf[16]={0};
u8 OSC_Status=0;
u8 OSC_mode=0;
u8 OSC_Busy=0;
u16 UserCountMS=1; // LPG: this is user command data, 1ms/count
u16 BaseCounter=0; // LPG: this is to count the base tick --1ms

static u8 Current_Stage=0;
static u8 Current_Cycle=0;  // record cycle & stage, be protected
static u8 Current_Sect=0;
static u8 FanCSR=0;         // FanCSR:  status register /* xxxxxx mode, status*/

//////////////////////////
float 	TempSet_2_Reg=0;
TimType TickLength_Reg_2=0;
//////////////////////////
// transfer the command packet
// input:
//	   Command, targt_row, how many byte data, data buffer pointer
///////////////////////////

															   
u8 Send_Command(u8 cmd, u8 row_num, u8 length, u8 * pbuf)
{
	u8 i;
  u8 val=NO_ERR;
	if(length>MAX_LENGTH_VALID)
		val=ERR_LENGTH;
/*
	if(row_num>MAX_ROW_NUM)
		return ERR_CMD;
*/
 	switch(cmd)
	{
		case PXL_DRIVE:
				if(row_num < PIX_TOTAL_ROW)
				{
			  		SPI_2_SendBuf[0]= (PXL_DRIVE<<5)| row_num;
					for(i=0;i<length;i++)
						SPI_2_SendBuf[1+i]= *(pbuf+i);	
					SPI_2_Trans(length+1);
				}else
					val=ERR_ROW;
					
			break;	
			
		case REG_WRITE:
				if(row_num <= REG_MAX_NUM)
				{
			  		SPI_2_SendBuf[0]= (REG_WRITE<<5)| row_num;
					for(i=0;i<length;i++)
						SPI_2_SendBuf[1+i]= *(pbuf+i);	
					SPI_2_Trans(length+1);
				}else
					val=ERR_ROW;
			break;	

		case ADC_START:
					//OSC_Ctrl=OSC_ON;
					//OSC_Status=1;
					//OSC_Busy=1;

			  		SPI_2_SendBuf[0]= (ADC_START<<5);	
					SPI_2_Trans(1);
					
			break;
		
		case PCR_ADC_READ:
			 		SPI_2_SendBuf[0]= 0xff;	  /*set to a invalid command, in case of misfunction*/
					SPI_2_Trans((PIX_TOTAL_COL<<1));
					SPI_Rev_Data_Copy((PIX_TOTAL_COL<<1));
			break;
									   
		default:
				val=ERR_CMD;
			break;
	}
	return 	val;
}
/////////////////
//  format the ADC data received from SPI
//  SWAP byte order, remove the 4 additional bits
//  input:
//			target u16 buffer pointer, how many bytes to be formated in SPI buffer
////////////////

void Format_ADC_Data(u8 *pu16Buf, u8 Byte_Num)
{
 	u8 i;
	u16 u16temp;
	for (i=0; i<Byte_Num;i+=2)
	{
		u16temp = SPI_2_RcvBuf[i+1];
		u16temp <<= 4;
		u16temp |= 	SPI_2_RcvBuf[i];
		*(pu16Buf++)=u16temp;
	}
}

////////////////
//	external OSC control initialize
//
//////////////
void OSC_Ctrl_Init(void)
{

	RCC->APB2ENR|=1<<8;     //Ê¹ÄÜPORTGÊ±ÖÓ
   	GPIOG->CRL&=0XFFFFF0FF;	
	GPIOG->CRL|=0X00000300;
	GPIOG->ODR|=1<<2; 
}

/////PCR cycle control initial
void PCR_Cycle_Init(void)
{
	u8 i;
	PCR_Cycle_Control.CycleValid=INVALID;
	PCR_Cycle_Control.TotalCycle=0;
	PCR_Cycle_Control.TotalSect=0;
	PCR_Cycle_Control.TotalStage=0;	
	for(i=0;i<MAX_STAGE;i++)
	{
		PCR_Cycle_SetPoint[i].SetPoint.float_num=0;
		PCR_Cycle_SetPoint[i].SetTime_Union.SetTime=0;

	}

	CycleSTS=READY;
}

/* check whether new data is out of range*/
/* parameter: Stage_Num, stage number in buffer*/
/*			  &buffer,   buffer pointer  		*/

u8 SetPoint_Check(u8 Stage_Num,CycleTempType *pCycleArray)  /* check whether new data is out of range*/
{
	u8 i;
	for(i=0;i<Stage_Num;i++)
	{
		if((pCycleArray[i].SetPoint.float_num < MIN_TEMPERATURE) || (pCycleArray[i].SetPoint.float_num>MAX_TEMPERATURE))
			return INVALID;
		if(pCycleArray[i].SetTime_Union.SetTime<1)
			return INVALID;
	}
	return VALID;	
}

/* copy the new setpoint into set register*/
/* parameter : stage number , source address, destination address*/
void SetPoint_Copy(u8 Stage_Num,CycleTempType *pSource, CycleTempType *pDest)
{
	u8 i;
	if(Stage_Num>0)
	{
		for(i=0;i<Stage_Num;i++)
		{
				pDest[i].SetPoint.float_num=pSource[i].SetPoint.float_num;
				pDest[i].SetTime_Union.SetTime=	pSource[i].SetTime_Union.SetTime;

				//add @2081-04-20 to support ramp parameter
				pDest[i].HasValidRampdata=pSource[i].HasValidRampdata;
				pDest[i].SetRamp_Union = pSource[i].SetRamp_Union;
		}		
	}
}


void SetPoint_Cycle_Start(CycleTempType *p)
{
   
    #ifdef __RPI_THERMO
    /*
            if((slot>0) && (slot < 3))
            {        
                p[0].SetPoint= RpiSetting.ThermoSetting[slot-1].fTemperature;
                p[0].SetTime_Union.SetTime=RpiSetting.ThermoSetting[slot-1].u32HoldingTime;
                p[0].SnapTime_Union.SnapTime=RpiSetting.ThermoSetting[slot-1].u32SnapTime;
            }
            else
                return;
    */
    #endif
    

	if(((p[0].SetPoint.float_num-TempSet_2)>5) || ((p[0].SetPoint.float_num-TempSet_2)<-5))
	{
				TempSet_2_Reg=p[0].SetPoint.float_num;
				TempCtrlTime_Sec_2= p[0].SetTime_Union.SetTime;

				#ifndef __RPI_THERMO
				
						TickLength_Reg_2= TempCtrlTime_Sec_2 * TICK_PER_SEC;
						TempTickLength_2= TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_OverShot;

				#else
						TickLength_Reg_2= TempCtrlTime_Sec_2 ;
						TempTickLength_2= TempCtrlTime_Sec_2 + Tick_OverShot;
						SnapTime_count_2= p[0].SnapTime_Union.SnapTime;

						RampTimeCnt =0;
						RampFlag=WAIT;
						
						trigger_idle();
						if((SnapTime_count_2>0) && (PcrMskReg != 0))
						{
								trigger_active();
						}

                                
				#endif
				
				WaitOverShot=0;
		
			if((p[0].SetPoint.float_num-TempSet_2)>5)
				TempSet_2= p[0].SetPoint.float_num + Temper_OverShot;
			else
			{
				// @2017-10-14 version 1.9 to support independent overshot for colding
				#ifdef __USE_TWO_OVERSHOT
						TempSet_2= p[0].SetPoint.float_num - Temper_colding_OverShot;

						#ifndef __RPI_THERMO				
				    		TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_colding_OverShot;
						#else
							TempTickLength_2=  TempCtrlTime_Sec_2 + Tick_colding_OverShot;
						#endif
				#else
						TempSet_2= p[0].SetPoint.float_num - Temper_OverShot;
				#endif
				FanCtrl_Auto(FAN_CTRL_MSK);
			}
			#ifdef __FAN_ALWAYS_ON  // version 1.6 @2017-04-15: always enable FAN in cycle
					FanCtrl_Auto(FAN_CTRL_MSK);	
			#endif         
	}
	else
	{
			TempSet_2=TempSet_2_Reg= p[0].SetPoint.float_num;
			TempCtrlTime_Sec_2= p[0].SetTime_Union.SetTime;
			TempTickLength_2=TickLength_Reg_2= TempCtrlTime_Sec_2 * TICK_PER_SEC;
			WaitOverShot=1;
	}
	
             //////////////////////////////////////////////////////////////////////
             #ifdef __RAMP_TEST
             
                     RampTimeCnt = 0;
                     RampFlag = WAIT;
                     loop=0;
                     FirstSlop=0;

                     //add @2018-04-20 to support ramp parameter in alpha
                     if(p[0].HasValidRampdata==1)
                     {
                        RampCtrl.HasValidRamp=1;
                        RampCtrl.RampRate = p[0].SetRamp_Union.SetRampTempPerSec;
                     }
                     else
                        RampCtrl.HasValidRamp=0;
             #endif   	
	
	
	TempCtrl_Active_2=1;
	TempValid |= 2;
	PID_Clear(SENSOR2_INDEX);
	Current_Stage=0;
	Current_Cycle=0;
}

void SetPoint_PrePump_Start(void)
{
	//TempSet_2= PE_Cycle_SetPoint[0].SetPoint.float_num;
	TempSet_2= PE_Cycle_SetPoint[0].SetPoint.float_num + Temper_OverShot;
	TempSet_2_Reg=PE_Cycle_SetPoint[0].SetPoint.float_num;
	
	TempCtrlTime_Sec_2= PE_Cycle_SetPoint[0].SetTime_Union.SetTime;
	//TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC;
	TickLength_Reg_2= TempCtrlTime_Sec_2 * TICK_PER_SEC;
	TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_OverShot;
	TempCtrl_Active_2=1;
	TempValid |= 2;
	WaitOverShot=0;
	PID_Clear(SENSOR2_INDEX);
}

void SetPoint_Extension_Start(void)
{
	TempValid &= ~0x2;
	TempCtrl_Active_2=1;	
  PID_Clear(SENSOR2_INDEX);
	

	WaitOverShot=0;
	TempCtrlTime_Sec_2=PE_Cycle_SetPoint[1].SetTime_Union.SetTime;
	TickLength_Reg_2= TempCtrlTime_Sec_2 * TICK_PER_SEC;
	TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_OverShot;
	
	TempSet_2_Reg=PE_Cycle_SetPoint[1].SetPoint.float_num;
	

	if((PE_Cycle_SetPoint[1].SetPoint.float_num-TempSet_2)>5)
		{
#ifndef __FAN_ALWAYS_ON  // version 1.6 @2017-04-15: always enable FAN in cycle
			FanCtrl_Auto(0);
#endif
			TempSet_2= PE_Cycle_SetPoint[1].SetPoint.float_num + Temper_OverShot;
		}
	else if((PE_Cycle_SetPoint[1].SetPoint.float_num-TempSet_2)<-5) 
				{
			// @2017-10-14 version 1.9 to support independent overshot for colding
			#ifdef __USE_TWO_OVERSHOT
					TempSet_2= PE_Cycle_SetPoint[1].SetPoint.float_num - Temper_colding_OverShot;
					TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_colding_OverShot;
			#else
					TempSet_2= PE_Cycle_SetPoint[1].SetPoint.float_num - Temper_OverShot;
			#endif
					FanCtrl_Auto(FAN_CTRL_MSK);
				}else
						{
							TempSet_2=TempSet_2_Reg;
							TempTickLength_2=TickLength_Reg_2;
							WaitOverShot=1;
						}
						
						 //////////////////////////////////////////////////////////////////////
             #ifdef __RAMP_TEST
             
                     RampTimeCnt = 0;
                     RampFlag = WAIT;
                     loop=0;
                     FirstSlop=0;

                     //add @2018-04-20 to support ramp parameter in alpha
                     if(PE_Cycle_SetPoint[1].HasValidRampdata==1)
                     {
                        RampCtrl.HasValidRamp=1;
                        RampCtrl.RampRate = PE_Cycle_SetPoint[1].SetRamp_Union.SetRampTempPerSec;
                     }
                     else
                        RampCtrl.HasValidRamp=0;
             #endif 
										
						
	TempValid |= 2;	

	
/*	
	if(TempSet_2>PE_Cycle_SetPoint[1].SetPoint.float_num)
	{
    	FanCtrl_Auto(FAN_CTRL_MSK);	 // cooling stage, Fan runs
		  TempSet_2= PE_Cycle_SetPoint[1].SetPoint.float_num-Temper_OverShot;
	}
	else
	{		
	    FanCtrl_Auto(0);
		  TempSet_2= PE_Cycle_SetPoint[1].SetPoint.float_num + Temper_OverShot;
	}
	//TempSet_2= PE_Cycle_SetPoint[1].SetPoint.float_num;
	TempSet_2_Reg= PE_Cycle_SetPoint[1].SetPoint.float_num;
	TempCtrlTime_Sec_2= PE_Cycle_SetPoint[1].SetTime_Union.SetTime;
	TickLength_Reg_2= TempCtrlTime_Sec_2 * TICK_PER_SEC;
	TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_OverShot;;
	TempCtrl_Active_2=1;
	TempValid |= 2;	
	WaitOverShot=0;
*/
}

u8 Cycle_Check(void)
{
	return Current_Cycle;
}

u8 Sect_Check(void)
{
	return Current_Sect;
}

u8 Stage_Check(void)
{
	return Current_Stage;
}

void Cycle_Set(u8 cycle)
{
	Current_Cycle=cycle;	
}
void Sect_Set(u8 sect)
{
	Current_Sect=sect;	
}
void Stage_Set(u8 stage)
{
	Current_Stage=stage;
}
void Cycle_INC(void)
{
	Current_Cycle++;
}

void Sect_INC(void)
{
	Current_Sect++;
}
void Stage_INC(void)
{
	Current_Stage++;
}

#ifdef __RPI_THERMO
	void TempCtrl_Reload(u8 slot)
	{

        
		TempValid &= ~0x2;
         TickLock  &= ~0x2;
        
		TempCtrl_Active_2=1;	
	  	PID_Clear(SENSOR2_INDEX);

		if((slot>0) && (slot < 3))
		{
             Cycle_Set(0);
	         Stage_Set(0);
             
			PCR_Cycle_SetPoint[0].SetPoint.float_num= RpiSetting.ThermoSetting[slot-1].fTemperature;
			PCR_Cycle_SetPoint[0].SetTime_Union.SetTime=RpiSetting.ThermoSetting[slot-1].u32HoldingTime;
			PCR_Cycle_SetPoint[0].SnapTime_Union.SnapTime=RpiSetting.ThermoSetting[slot-1].u32SnapTime;
  
			WaitOverShot=0;
            
			TempCtrlTime_Sec_2=PCR_Cycle_SetPoint[0].SetTime_Union.SetTime;

             #ifdef __USE_1SEC_UNIT
                     
        			TickLength_Reg_2= TempCtrlTime_Sec_2 * TICK_PER_SEC;
			        if(PCR_Cycle_SetPoint[0].SetPoint.float_num != -1000)
									TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_OverShot;
							else
									TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC;
        			TempSet_2_Reg=PCR_Cycle_SetPoint[0].SetPoint.float_num;
              SnapTime_count_2=PCR_Cycle_SetPoint[0].SnapTime_Union.SnapTime * TICK_PER_SEC;// + Tick_OverShot;
                     
             #endif


             RpiSetting.RpiSetCurrentIdx ++;
             RpiSetting.ThermoStatus = 1;
             RpiSetting.RpiSetIdx &= ~(1<<(slot-1));
             /////////////////////////////////
             // if a infinite cycle?
             /////////////////////////////////

             if(TickLength_Reg_2 == 0)
						 {
                   Flag_Current_forever = 1;
						 }
             else
                   Flag_Current_forever = 0;
             
             
             RampTimeCnt=0;
             
             trigger_idle();
             if((SnapTime_count_2>0) && (PcrMskReg != 0))
             {
                 trigger_active();
             }
            

			if((PCR_Cycle_SetPoint[0].SetPoint.float_num-TempSet_2)>5)
			{
			#ifndef __FAN_ALWAYS_ON  // version 1.6 @2017-04-15: always enable FAN in cycle
				FanCtrl_Auto(0);
			#endif
				TempSet_2= PCR_Cycle_SetPoint[0].SetPoint.float_num + Temper_OverShot;
			}
			else if((PCR_Cycle_SetPoint[0].SetPoint.float_num-TempSet_2)<-5) 
				{
									// @2017-10-14 version 1.9 to support independent overshot for colding
				#ifdef __USE_TWO_OVERSHOT
						TempSet_2= PCR_Cycle_SetPoint[0].SetPoint.float_num - Temper_colding_OverShot;
						TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_colding_OverShot;
				#else
						TempSet_2= PCR_Cycle_SetPoint[0].SetPoint.float_num - Temper_OverShot;
				#endif
						FanCtrl_Auto(FAN_CTRL_MSK);
				}
				else
					{
						TempSet_2=TempSet_2_Reg;
						TempTickLength_2=TickLength_Reg_2;
						WaitOverShot=1;
					}
			TempValid |= 2;
             TickLock  |= 2;
		}
        else
        {
             // means no pending stage in queue
             // the personal idea to go to FAN_EXt stage
             TempCtrl_Active_2=0; 
        }
	}
#endif


void TempCtrl_Reload_InCycle(void)
{
	TempValid &= ~0x2;
	TempCtrl_Active_2=1;	
    PID_Clear(SENSOR2_INDEX);
	

#ifdef __HAS_CONFIGURABLE_SNAP	  
	
			if(PCR_Cycle_Control.HasSnapCfg == 1)	// if flexible configue
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




	
	WaitOverShot=0;
	TempCtrlTime_Sec_2=PCR_Cycle_SetPoint[Current_Stage].SetTime_Union.SetTime;
	TickLength_Reg_2= TempCtrlTime_Sec_2 * TICK_PER_SEC;
	TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_OverShot;
	
	TempSet_2_Reg=PCR_Cycle_SetPoint[Current_Stage].SetPoint.float_num;
	
	if((PCR_Cycle_SetPoint[Current_Stage].SetPoint.float_num-TempSet_2)>5)
		{
#ifndef __FAN_ALWAYS_ON  // version 1.6 @2017-04-15: always enable FAN in cycle
			FanCtrl_Auto(0);
#endif
			TempSet_2= PCR_Cycle_SetPoint[Current_Stage].SetPoint.float_num + Temper_OverShot;

		}
	else if((PCR_Cycle_SetPoint[Current_Stage].SetPoint.float_num-TempSet_2)<-5) 
				{
								// @2017-10-14 version 1.9 to support independent overshot for colding
			#ifdef __USE_TWO_OVERSHOT
					TempSet_2= PCR_Cycle_SetPoint[Current_Stage].SetPoint.float_num - Temper_colding_OverShot;
					TempTickLength_2=  TempCtrlTime_Sec_2 * TICK_PER_SEC + Tick_colding_OverShot;
			#else
					TempSet_2= PCR_Cycle_SetPoint[Current_Stage].SetPoint.float_num - Temper_OverShot;
			#endif
					FanCtrl_Auto(FAN_CTRL_MSK);

            
				}else
						{
							TempSet_2=TempSet_2_Reg;
							TempTickLength_2=TickLength_Reg_2;
							WaitOverShot=1;
						}
						
					/////////////////////////////////////
				#ifdef __RAMP_TEST
				
						RampTimeCnt = 0;
						RampFlag = WAIT;

						//add @2018-04-20 to support ramp parameter in alpha
						 if(PCR_Cycle_SetPoint[Current_Stage].HasValidRampdata==1)
						 {
								RampCtrl.HasValidRamp=1;
								RampCtrl.RampRate = PCR_Cycle_SetPoint[Current_Stage].SetRamp_Union.SetRampTempPerSec;
						 }
						 else
								RampCtrl.HasValidRamp=0;
				 
				#endif
				////////////////////////////////////					
						
	TempValid |= 2;	

    
}


void FanCtrl_Init(void)
{
  FAN=LOW;
  FanCSR=0;
//  Fan_Echo(FanCSR);
}

void FanCtrl_Force(u8 ctrl)	 // manual control the fan, mode bit toggle too.
{
	FanCSR = (FAN_MODE_MSK | (ctrl & FAN_CTRL_MSK));
	FAN = (ctrl & FAN_CTRL_MSK);
//	Fan_Echo(FanCSR);
}

void FanCtrl_Auto(u8 ctrl)	// auto control the fan, based on CSR mode bit
{
	FanCSR = (FanCSR & FAN_MODE_MSK)? FanCSR:(ctrl & FAN_CTRL_MSK); 
	FAN = (FanCSR & FAN_CTRL_MSK);
//	Fan_Echo(FanCSR);
}

void FanMode_Clear(void)
{
	FanCSR &= ~(FAN_MODE_MSK);	
}

u8 FanCSR_Read(void)
{
	return(FanCSR & (FAN_MODE_MSK | FAN_CTRL_MSK));
}

void Cycle_Control_Start(void)
{
	PCR_Cycle_Control.CycleValid=INIT;
	Current_Cycle=Current_Sect=Current_Stage=0;
	CycleSTS=WAIT;			 // going to start backgroud sensor
	TempSet=TempPumpSet;
	TempCtrl_Active=1;
	TempValid |= 1;
	TempTickLength=1;        // in case that timout
	ClearMsg();				 // in case that any pend msg 
}

void Trim_Reset(void)
{
    #ifndef __RPI_THERMO
            	u8 i;
            	for(i=0;i<MAX_PCR_CH;i++)
            	{
            		PCR_Regs[i].InteTime=1;
            		if((PcrMskReg >>i) & 0x1)
            			integrationDur +=PCR_Regs[i].InteTime*4;
            	}               
    #endif
	trigger_reset();
}

void trigger_time_Cal(void)
{
	u8 i;
	for(i=0;i<MAX_PCR_CH;i++)
	{
		if((PcrMskReg >>i) & 0x1)
		{
			if(PCR_Regs[i].InteTime>10)
				integrationDur +=PCR_Regs[i].InteTime;
			else
				integrationDur +=PCR_Regs[i].InteTime*12;
		}
	}
	integrationDur+=SetTm_LED_Delay;
//////////include the time delay between interline
  integrationDur = integrationDur+120; 	
/////////////////////////////// add more margin of LED
}

void trigger_out(void)
{
    #ifndef __RPI_THERMO
          if(PcrMskReg & 0x1) TRG_0=1;
        	 if(PcrMskReg & 0x2) TRG_1=1;
        	 if(PcrMskReg & 0x4) TRG_2=1;
        	 if(PcrMskReg & 0x8) TRG_3=1;
	#else
          if(PcrMskReg & 0x1) TRG_0=1;
    #endif
	trg_assert=IS_ASSERT;
}

void trigger_reset(void)
{
    #ifndef __RPI_THERMO
	    TRG_0=TRG_1=TRG_2=TRG_3=0;
    #else
         TRG_0=0;
    #endif
	trg_assert=IS_IDLE;
    
}	
void trigger_done(void)
{
    #ifndef __RPI_THERMO
	     TRG_0=TRG_1=TRG_2=TRG_3=0;
    #else
         TRG_0=0;
    #endif

	trg_assert=IS_DONE;
}

void trigger_idle(void)
{
	#ifndef __RPI_THERMO
	    TRG_0=TRG_1=TRG_2=TRG_3=0;
    #else
         TRG_0=0;
    #endif
	trg_assert=IS_IDLE;
}

void trigger_active(void)
{
	#ifndef __RPI_THERMO
	    TRG_0=TRG_1=TRG_2=TRG_3=0;
    #else
         TRG_0=0;
    #endif
	trg_assert=IS_ACTIVE;
}


